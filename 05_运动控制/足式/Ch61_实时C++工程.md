> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 61 章 实时 C++ 工程——PREEMPT_RT + 无堆分配 + 无锁通信 + ros2_control

> **难度**: ⭐⭐ ~ ⭐⭐⭐ | **预计学时**: 30-40 小时(1.5 周) | **text:code = 6:4**（工程实践为主章节）
>
> **一句话概要**: 实时 C++ 是让控制算法从"仿真能跑"到"机器人不摔"的最后一公里——本章从 Linux 调度内核到无锁数据结构,从 `EIGEN_RUNTIME_NO_MALLOC` 到 `ros2_control` 实时架构,系统构建腿足机器人的实时软件栈。

**向前承接**: Ch53 引入了 `EIGEN_RUNTIME_NO_MALLOC` 作为 WBC 的硬约束,Ch55 展示了 OCS2 的双线程 Triple Buffer 架构。本章将这些零散的实时概念统一为完整的工程方法论。

**向后指向**: Ch67(Perceptive MPC)和 Ch68(legged_control 精读)将在本章的实时基础设施上构建感知-控制闭环。

---

## 前置自测

📋 **答不出 $\geq$ 2 题 → 先回 Ch17-20(并发)、Ch35(pmr)复习**

1. `std::mutex` 在无竞争时的加锁开销大约是多少纳秒?在有竞争时呢?为什么这个差异对 1 kHz 控制循环至关重要?
2. `SCHED_FIFO` 和 `SCHED_OTHER`(CFS) 的根本区别是什么?一个优先级为 80 的 `SCHED_FIFO` 线程能被优先级为 90 的 CFS 线程抢占吗?
3. 什么是 priority inversion(优先级反转)?经典的 Mars Pathfinder 案例中发生了什么?
4. C++17 的 `std::pmr::monotonic_buffer_resource` 的分配策略是什么?为什么它特别适合实时系统?
5. `volatile` 关键字能替代 `std::atomic` 吗?为什么?

## 本章目标

学完本章,你应能:

1. 在 Ubuntu 上部署 PREEMPT_RT 内核并用 `cyclictest` 验证延迟 $< 100 \mu s$
2. 编写满足硬实时约束的 C++ 控制循环——无堆分配、无阻塞、无 I/O
3. 实现 SPSC 无锁队列和 Triple Buffer,理解 `memory_order` 的选择
4. 使用 `ros2_control` 框架部署控制器插件,理解 `RealtimePublisher` 和 `RealtimeBuffer`/`RealtimeThreadSafeBox` 的设计
5. 用 `cyclictest`、`ftrace`、`trace-cmd` 诊断实时性问题
6. 将 Ch55 的双线程 MPC 架构落地为生产级实时代码

---

## 61.1 为什么需要实时 ⭐

### 动机:一个 5 ms 的延迟如何让机器人摔倒

想象你的四足机器人正在 trot 步态下以 0.5 m/s 行走。WBC 控制循环运行在 1 kHz(1 ms 周期)。此时,Linux 的 CFS 调度器决定把 CPU 让给一个正在编译 ROS 2 package 的 `colcon build` 进程——你的 WBC 线程被挂起了 5 ms。

这 5 ms 内发生了什么?

| 时间 | 事件 | 后果 |
|------|------|------|
| $t_0$ | WBC 线程被 CFS 调度器挂起 | 关节扭矩命令冻结在 $t_0$ 时刻的值 |
| $t_0 + 1\text{ms}$ | 机器人质心已移动 0.5 mm,姿态开始偏离 | 电机驱动器继续执行过时的扭矩 |
| $t_0 + 3\text{ms}$ | 摆动腿即将触地,但足端轨迹未更新 | 触地位置误差约 1.5 mm |
| $t_0 + 5\text{ms}$ | WBC 线程恢复,但状态已严重偏离预期 | 扭矩补偿过大,引发振荡 |
| $t_0 + 10\text{ms}$ | 振荡累积,一条支撑腿打滑 | 机器人失稳 |
| $t_0 + 50\text{ms}$ | 四足机器人侧翻 | **硬件可能损坏** |

这不是假设——在没有实时保证的系统上,这种场景每隔几分钟就可能发生一次。MIT Mini Cheetah 的开发团队在早期就遇到过类似问题:MPC 求解偶尔超时导致机器人在后空翻落地时失控。

### 如果不用实时会怎样:三个真实故事

**故事 1:Mars Pathfinder 的优先级反转(1997)**

1997 年,NASA 的 Mars Pathfinder 着陆器在火星表面开始出现频繁的系统重启。根源是经典的优先级反转:低优先级的气象数据采集任务持有一个互斥锁,中优先级的通信任务抢占了它,导致高优先级的总线管理任务无限期等待。看门狗计时器检测到总线管理任务停止响应,触发系统重启。NASA 工程师通过远程上传补丁,启用了 VxWorks RTOS 的**优先级继承**(priority inheritance)机制才解决问题。

**故事 2:某四足公司的 `std::cout` 事故(2022)**

一位工程师在 WBC 控制循环中留下了一行调试用的 `std::cout << "tau: " << tau.transpose() << std::endl;`。在实验室低负载时一切正常,因为 `std::cout` 通常只需 10-50 $\mu s$。但在展会现场,笔记本同时运行可视化和录制视频,`stdout` 缓冲区满了——`std::cout` 阻塞了 8 ms。机器人当场摔倒,腿部碳纤维断裂。

**故事 3:`std::vector::push_back` 的定时炸弹(2023)**

另一个团队在 WBC 中用 `std::vector<ContactForce>` 存储接触力。大多数时候 vector 容量足够,`push_back` 只是指针移动(约 1 ns)。但当接触模式切换(trot 转 gallop)导致接触点数量超过 vector 容量时,`push_back` 触发 realloc——一次 `malloc` + `memcpy` 花了 200 $\mu s$,控制循环超时。

### 硬实时 vs 软实时:严格定义

这两个术语在工业界和学术界经常被混用,但它们的区别是本质性的:

| 属性 | 硬实时 (Hard Real-Time) | 软实时 (Soft Real-Time) |
|------|------------------------|------------------------|
| **Deadline 违反的后果** | **系统失败**(不是"性能差") | QoS 下降,但系统继续运行 |
| **允许违反次数** | 0 次 | 偶尔可以,有统计约束 |
| **典型场景** | 关节扭矩控制(1 kHz) | SLAM tracking(30-100 Hz) |
| **延迟要求** | Worst-case bounded | Average-case 足够好 |
| **验证方法** | 形式化验证 / WCET 分析 | 统计测试(p99, p999) |

**腿足控制系统的实时层次**:

```
┌─────────────────────────────────────────────────────────────┐
│ 电机伺服环 (2-10 kHz, 硬实时)                                │
│ 通常由电机驱动器 MCU 内部处理, 不在 Linux 上运行              │
│ Deadline: 100-500 μs                                        │
├─────────────────────────────────────────────────────────────┤
│ WBC / 关节力矩控制 (500 Hz - 1 kHz, 硬实时)                  │
│ 本章重点! 运行在 PREEMPT_RT Linux 上                         │
│ Deadline: 1-2 ms                                            │
├─────────────────────────────────────────────────────────────┤
│ MPC 求解 (20-100 Hz, 准实时/软实时)                           │
│ 允许偶尔超时, 用上一次解的外推代替                             │
│ Deadline: 10-50 ms (弹性)                                    │
├─────────────────────────────────────────────────────────────┤
│ 状态估计 (200-1000 Hz, 软实时)                                │
│ IMU 融合需要高频, 但偶尔丢一帧不致命                          │
│ Deadline: 1-5 ms                                            │
├─────────────────────────────────────────────────────────────┤
│ 感知 / 规划 (1-30 Hz, 非实时)                                │
│ 地形建图、路径规划                                            │
│ 无严格 deadline                                              │
└─────────────────────────────────────────────────────────────┘
```

### 历史:实时计算的演进

实时系统不是一个新概念。它的历史与嵌入式系统紧密交织:

- **1960s-70s**: 早期实时系统用于航空航天(Apollo 导航计算机)。软件直接在裸机上运行,没有操作系统层。
- **1980s**: VxWorks (1987) 和 QNX (1982) 等商业 RTOS 出现。它们提供微秒级确定性调度,但生态封闭、昂贵。
- **1990s**: Linux 开始被尝试用于实时场景。RTLinux (1996, FSMLabs) 和 RTAI (1999) 通过"双内核"架构实现:一个微内核处理实时任务,Linux 作为最低优先级任务运行。
- **2000s**: Ingo Molnar 和 Thomas Gleixner 开始 PREEMPT_RT 项目——不是双内核,而是**让 Linux 自身变得可抢占**。这条路更难走(改造整个内核),但收益更大(实时任务可以直接使用所有 Linux API)。
- **2024**: **里程碑!** Linux 6.12 (2024 年 11 月发布) 正式将 PREEMPT_RT 合并入主线内核,支持 x86_64、ARM64 和 RISC-V。不再需要打外部补丁。
- **2025-2026**: PREEMPT_RT 成为主线技术,持续优化中。主线内核之外仍有额外的 RT patch 提供尚未合入的优化。

> 💡 **为什么 Linux 而不是 RTOS?** 机器人系统需要 ROS 2、OpenCV、PyTorch、Pinocchio 等大量库——这些库只在 Linux(和部分在 macOS/Windows)上可用。用 VxWorks 或 FreeRTOS 意味着放弃整个生态。PREEMPT_RT 的价值在于:**在保留 Linux 完整生态的同时,获得接近 RTOS 的实时性能**。

### ⚠️ 常见陷阱

> ⚠️ **概念误区:认为"平均延迟低"等于"实时"**
>
> **新手想法**: "我的 WBC 平均执行时间只有 0.3 ms,周期 1 ms,绰绰有余啊!"
>
> **现象**: 99.9% 的时候没问题,但每运行 10 分钟就摔倒一次。
>
> **根本原因**: 实时关心的不是平均值,而是**最坏情况**(worst-case)。平均 0.3 ms 不代表最大不会超过 1 ms。一次 `malloc` 可能导致 5 ms 延迟,一次 page fault 可能导致 10 ms 延迟。这些"长尾"事件虽然罕见,但在 1 kHz 的循环中,1 小时 = 3,600,000 次循环,即使 0.001% 的概率也意味着 36 次超时。
>
> **正确做法**: 关注 **worst-case execution time (WCET)** 和调度延迟的 **max latency**。用 `cyclictest` 持续运行 24 小时,在满负载下测量 max latency。

> 🧠 **思维陷阱:认为"RL 策略推理快就不需要实时内核"**
>
> **新手想法**: "我的 RL policy 只需要 0.2 ms 推理,比 MPC 快 100 倍,不需要 PREEMPT_RT 了吧?"
>
> **实际上**: RL 推理时间是**计算时间**,不是**端到端延迟**。即使推理只需 0.2 ms,如果 Linux 调度器在推理前挂起线程 5 ms,总延迟就是 5.2 ms——远超 1 ms deadline。**PREEMPT_RT 保护的是调度延迟,与算法计算时间无关。**
>
> **延伸**: RL + MPC 混合架构(Ch65)仍然需要实时系统支撑。

### 练习

**练习 61.1a** ⭐: 在你的笔记本上运行 `stress -c $(nproc)` 制造 CPU 满负载,同时运行一个 1 kHz 的循环打印每次循环的实际耗时。统计 max jitter。在标准 Ubuntu 内核和(如果有)RT 内核上分别测试,对比结果。

**练习 61.1b** ⭐⭐: 列出你的控制系统中所有可能的延迟源(至少 8 个),按"可控性"分为三类:可消除(如 malloc)、可缓解(如 cache miss)、不可控(如 CPU 中断)。对每个延迟源估算最坏情况延迟。

---

上一节我们理解了"为什么需要实时"——本质是控制循环的 deadline 不能被违反。但认识到问题只是第一步,接下来我们需要理解 Linux 内核提供了哪些机制来保证实时性。

## 61.2 Linux 实时调度 ⭐⭐

### 动机:标准 Linux 内核为什么不是实时的

在普通的 Ubuntu 桌面内核(CONFIG_PREEMPT_VOLUNTARY 或 CONFIG_PREEMPT)下,内核代码的大部分区域**不可被用户空间任务抢占**。当你的 WBC 线程需要 CPU 时,如果内核正在处理一个网络包、一次文件系统操作或一个 GPU 驱动调用,WBC 线程必须等到内核主动让出 CPU。

这种等待可以有多长?用 `cyclictest` 实测:

```bash
# On standard Ubuntu 24.04 kernel (6.8), under CPU stress
sudo cyclictest -p 99 -t 4 -n -m -D 60
# Typical result:
# T: 0 ( 1234) P:99 I:1000 C: 60000 Min:3 Act:15 Avg:22 Max:45000
#                                                         ^^^^^^
#                                                         45 ms!
```

**45 ms 的最大延迟是 1 ms 控制周期的 45 倍**——这在腿足机器人上完全不可接受。

### 如果不解决调度延迟会怎样

不解决调度延迟意味着你的 WBC 循环在任何时刻都可能被挂起 10-50 ms。这等价于"每隔几分钟让机器人闭眼走 10-50 步"。结果是不可预测的失稳和摔倒。

### 历史:Linux 抢占模型的四个级别

Linux 内核的抢占模型经历了从"完全不可抢占"到"几乎完全可抢占"的演进:

| 级别 | CONFIG 选项 | 引入时间 | 行为 | 典型用途 |
|------|------------|---------|------|---------|
| Level 0 | `PREEMPT_NONE` | 原始 | 内核代码不可抢占 | 服务器(吞吐优先) |
| Level 1 | `PREEMPT_VOLUNTARY` | 2.6 | 在少量"抢占点"可抢占 | 桌面(响应改善) |
| Level 2 | `PREEMPT` | 2.6 | 大部分内核代码可抢占(除 spinlock 区域) | 低延迟音频/视频 |
| Level 3 | `PREEMPT_RT` | 6.12 主线 | **几乎全部**内核代码可抢占 | **机器人硬实时** |

**PREEMPT_RT 做了什么?**

PREEMPT_RT 之于普通 Linux 内核，好比急诊室分诊系统之于普通门诊排队——普通门诊按挂号顺序看病，即使心脏骤停患者也得等前面的人看完；急诊分诊则严格按病情紧急程度排序，心脏骤停的患者（高优先级 WBC 线程）可以立刻打断正在看感冒的医生（低优先级后台任务）。PREEMPT_RT(由 Ingo Molnar 和 Thomas Gleixner 自 2004 年开始开发)对 Linux 内核进行了三个核心改造:

1. **将 spinlock 替换为可抢占的 rt_mutex**: 普通内核中,持有 spinlock 的代码段不可被抢占。PREEMPT_RT 将大部分 spinlock 替换为支持优先级继承的 rt_mutex,使得持有锁的代码段也可以被更高优先级的任务抢占。只有极少数内核关键路径保留 `raw_spinlock`(如调度器自身、中断描述符操作)。

2. **将硬中断处理移入内核线程**: 普通内核中,硬件中断(如网卡收包)直接在中断上下文中处理,不可被任何用户任务抢占。PREEMPT_RT 将中断处理函数封装为内核线程(`irq/xxx`),这些线程参与调度,可以被更高优先级的实时任务抢占。

3. **优先级继承(Priority Inheritance)**: 当高优先级任务等待低优先级任务持有的锁时,低优先级任务临时"继承"高优先级任务的优先级,防止中优先级任务抢占低优先级任务导致的优先级反转。

### PREEMPT_RT 的部署

**Linux 6.12+ 主线内核已包含 PREEMPT_RT**——这是 2024 年 11 月的里程碑。但"包含"不等于"默认启用",你仍需要编译或安装启用了 `CONFIG_PREEMPT_RT` 的内核。

```bash
# Method 1: Install pre-built RT kernel on Ubuntu 24.04+
sudo apt install linux-image-rt-amd64          # Debian family
# Or:
sudo apt install linux-image-6.12.0-1001-realtime   # Ubuntu realtime
# (exact package name varies; use apt search linux-image.*rt to find)

# Method 2: Build from mainline kernel source (recommended, customizable)
wget https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-6.12.tar.xz
tar xf linux-6.12.tar.xz && cd linux-6.12
# For latest RT optimizations not yet merged, apply the extra RT patch:
# wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.12/patch-6.12-rt*.patch.xz
make menuconfig
# -> General setup -> Preemption Model -> Fully Preemptible Kernel (Real-Time)
make -j$(nproc) && sudo make modules_install && sudo make install
sudo update-grub && sudo reboot
```

**验证安装**:

```bash
uname -v
# Should contain "PREEMPT_RT" or "PREEMPT DYNAMIC"

# Check current preemption model
cat /sys/kernel/realtime
# Output "1" means RT mode is active
```

**系统调优**(重要——装完内核只是第一步):

```bash
# 1. Lock CPU frequency to maximum (disable dynamic scaling)
sudo cpupower frequency-set -g performance

# 2. Disable turbo boost (optional, improves consistency at cost of peak)
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo

# 3. Disable deep CPU idle states (C-states)
# In GRUB_CMDLINE_LINUX_DEFAULT add:
# processor.max_cstate=0 idle=poll    (extreme, high power draw)
# or intel_idle.max_cstate=1          (compromise)

# 4. Isolate CPU cores for realtime tasks
# GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3"
```

### 调度策略详解:SCHED_FIFO vs SCHED_RR vs SCHED_DEADLINE

Linux 提供三种实时调度策略,加上默认的非实时策略:

| 策略 | 类型 | 优先级范围 | 调度算法 | 特点 |
|------|------|-----------|---------|------|
| `SCHED_OTHER` | 非实时 | 0 (nice -20~19) | CFS (Completely Fair) | 公平共享,延迟不可预测 |
| `SCHED_FIFO` | 实时 | 1-99 | 先到先服务 | 最高优先级跑完才让,**无时间片** |
| `SCHED_RR` | 实时 | 1-99 | 轮转 | 同优先级间时间片轮转 |
| `SCHED_DEADLINE` | 实时 | 无固定优先级 | EDF (Earliest Deadline First) | 基于 deadline 调度,**优于所有 FIFO/RR** |

**`SCHED_FIFO`——腿足控制的传统选择**:

```cpp
#include <pthread.h>
#include <sched.h>

void configureRealtimeThread(pthread_t thread, int priority) {
  sched_param param;
  param.sched_priority = priority;  // 1-99, higher = more urgent
  int ret = pthread_setschedparam(thread, SCHED_FIFO, &param);
  if (ret != 0) {
    // Common cause: missing CAP_SYS_NICE or insufficient rtprio limit
    perror("Failed to set SCHED_FIFO");
  }
}
```

**`SCHED_DEADLINE`——更先进的选择**:

`SCHED_DEADLINE` 使用 EDF (Earliest Deadline First) + CBS (Constant Bandwidth Server) 算法。它不使用固定优先级,而是让每个任务声明自己的时间需求:

```cpp
#include <linux/sched.h>
#include <sys/syscall.h>

struct sched_attr {
  uint32_t size;
  uint32_t sched_policy;
  uint64_t sched_flags;
  int32_t  sched_nice;
  uint32_t sched_priority;
  uint64_t sched_runtime;   // CPU time needed per period (ns)
  uint64_t sched_deadline;  // Must finish within this time (ns)
  uint64_t sched_period;    // Activation period (ns)
};

void configureDeadlineThread(pid_t tid) {
  struct sched_attr attr = {};
  attr.size = sizeof(attr);
  attr.sched_policy = SCHED_DEADLINE;
  attr.sched_runtime  =  500000;  // 500 us of CPU per period
  attr.sched_deadline = 1000000;  // 1 ms deadline
  attr.sched_period   = 1000000;  // 1 ms period

  // sched_setattr is a Linux-specific syscall, no glibc wrapper
  int ret = syscall(SYS_sched_setattr, tid, &attr, 0);
  if (ret < 0) perror("sched_setattr DEADLINE");
}
```

**SCHED_FIFO vs SCHED_DEADLINE:该用哪个?**

| 维度 | SCHED_FIFO | SCHED_DEADLINE |
|------|-----------|----------------|
| **优先级模型** | 固定优先级(人工分配) | 动态优先级(EDF 自动) |
| **任务隔离** | 无——高优先级可饿死低优先级 | 有——CBS 保证带宽隔离 |
| **超时保护** | 无——死循环会锁死系统 | 有——超时后任务被挂起到下一周期 |
| **可组合性** | 差——新增任务需重新分配优先级 | 好——每个任务独立声明需求 |
| **生态成熟度** | 非常成熟,所有 RT 框架都支持 | 较新,ros2_control 等框架支持有限 |
| **适用场景** | 任务少、优先级关系明确 | 任务多、需要时间隔离 |

**现实选择**: 目前腿足社区主流仍使用 `SCHED_FIFO`(legged_control、ros2_control 默认都是 SCHED_FIFO),因为生态支持更好。但 `SCHED_DEADLINE` 在理论上更优,预计未来会逐步普及。

### 优先级反转与优先级继承

**优先级反转**(Priority Inversion)是实时系统中最危险的 bug 之一。它的经典场景:

```
时间 ──────────────────────────────────────────>

高优先级 H:  [运行]──[等待锁]─────────────────[获得锁, 继续]
中优先级 M:  ────────[抢占 L]──[运行很久]─────[完成]
低优先级 L:  ──[持有锁, 运行]──[被 M 抢占]────[恢复, 释放锁]

问题: H 等待 L 释放锁, 但 L 被 M 抢占了!
      H 的实际等待时间 = M 的运行时间 (可能很长)
      H 被 M "间接阻塞", 违反了优先级语义
```

**优先级继承**(Priority Inheritance)的解决方案:

```
时间 ──────────────────────────────────────>

高优先级 H:  [运行]──[等待锁]──────[获得锁, 继续]
中优先级 M:  ────────[想抢占 L, 但 L 优先级已提升!]──[运行]
低优先级 L:  ──[持有锁]──[继承 H 的优先级, 不被 M 抢占]──[释放锁, 恢复]

L 临时获得 H 的优先级 → M 无法抢占 L → L 快速释放锁 → H 快速恢复
```

**PREEMPT_RT 内核中,内核态的 `rt_mutex` 默认启用优先级继承**。但用户空间的 `std::mutex`（底层为 POSIX `pthread_mutex`）**并不自动获得 PI 支持**——POSIX 标准中 mutex 的默认协议是 `PTHREAD_PRIO_NONE`。要在用户空间使用 PI mutex,需要显式设置 `pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT)`。无论如何,**最好的做法是在实时路径上完全避免锁**。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:优先级设为 99 导致系统卡死**
>
> **错误做法**: `sched_param.sched_priority = 99;` 给 WBC 线程设最高优先级。
>
> **现象**: 如果 WBC 线程出现死循环或死锁,整个系统冻结——连 SSH 都连不上。
>
> **根本原因**: 优先级 99 的 SCHED_FIFO 线程可以抢占几乎所有内核线程。如果它不主动释放 CPU,没有任何线程能运行。
>
> **正确做法**: 使用 50-90 范围。保留 91-99 给内核关键线程。在 `/etc/security/limits.conf` 中限制最大 rtprio。另外,使用 `SCHED_DEADLINE` 的 CBS 机制可以自动防止这个问题——超时后任务会被挂起。

> 💡 **概念误区:认为 PREEMPT_RT 让一切变快**
>
> **新手想法**: "装了 RT 内核,我的 MPC 求解速度就更快了!"
>
> **实际上**: PREEMPT_RT 不加速计算,它甚至可能让平均吞吐量**下降** 5-15%(因为更多的抢占点和优先级继承增加了开销)。PREEMPT_RT 的价值是**降低最坏情况延迟**——把 max latency 从 50 ms 降到 50 $\mu s$。平均性能和最坏性能是两个完全不同的指标。

> ⚠️ **编程陷阱:忘记配置 `/etc/security/limits.conf`**
>
> **现象**: 程序报 `pthread_setschedparam: Operation not permitted`。
>
> **根本原因**: 普通用户默认没有设置实时优先级的权限。
>
> **正确做法**:
> ```
> # /etc/security/limits.conf
> @robotics - rtprio 95      # allow setting priority 1-95
> @robotics - memlock unlimited  # allow unlimited memory locking
> ```
> 然后将用户加入 `robotics` 组:`sudo usermod -aG robotics $USER`。需要重新登录生效。

### 练习

**练习 61.2a** ⭐: 在你的系统上分别用标准内核和 RT 内核运行 `cyclictest -p 99 -t 4 -n -m -D 300`(5 分钟),同时用 `stress -c $(nproc) --io 4` 制造负载。对比两组的 Min/Avg/Max latency。画出延迟分布直方图(用 `-h 1000` 参数导出)。

**练习 61.2b** ⭐⭐: 写一个程序创建 3 个线程:H(优先级 80)、M(优先级 60)、L(优先级 40)。L 获取一个 mutex,然后 busy-wait 10 ms;M 在 L 获取锁 1 ms 后启动,进行 CPU 密集计算;H 在 L 获取锁 2 ms 后启动,尝试获取同一个 mutex。测量 H 从尝试获取锁到成功获取锁的时间。分别在有/无优先级继承的情况下测试:使用 `pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT)` 创建 PI mutex,与默认的 `PTHREAD_PRIO_NONE` mutex 对比。在 PREEMPT_RT 内核上运行以获得可复现的调度行为。

---

理解了 Linux 如何调度实时线程之后,下一个问题是:实时线程本身应该怎么写?普通的 C++ 编程习惯(随意 new/delete、调用 printf、使用 STL 容器)在实时上下文中全是地雷。

## 61.3 实时线程编程 ⭐⭐

### 动机:一个"正常"的 C++ 程序在实时环境下的表现

考虑这段看起来完全正常的 C++ 代码:

```cpp
// ❌ This "normal" C++ code has multiple realtime violations!
void controlLoop() {
  while (running) {
    auto start = std::chrono::steady_clock::now();

    auto q = readJointPositions();                    // OK
    Eigen::VectorXd tau = wbc.compute(q);             // VectorXd may malloc!
    writeJointCommands(tau);                          // OK

    std::cout << "tau: " << tau.transpose() << "\n";  // I/O blocks!

    auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    std::this_thread::sleep_for(1ms - elapsed);       // Imprecise timing!
  }
}
```

这段代码至少有 3 个实时违规:
1. `Eigen::VectorXd` 是动态大小,构造/赋值可能触发 `malloc`
2. `std::cout` 是阻塞 I/O,可能等待 stdout 缓冲区刷新
3. `std::this_thread::sleep_for` 的精度只有约 1-10 ms(取决于内核配置),不适合精确定时

### 实时线程的四大禁忌

在 Ch53 中我们已经初步接触了"控制循环的禁忌"。这里做系统性的梳理:

| 禁忌 | 为什么危险 | 最坏延迟 | 替代方案 |
|------|-----------|---------|---------|
| **堆分配** (`malloc`/`new`) | 分配器内部锁 + 可能触发 `mmap` 系统调用 + 缺页中断 | 10 $\mu s$ - 10 ms | 预分配 + 固定大小 + pmr |
| **阻塞锁** (`mutex`/`semaphore`) | 优先级反转 + 不确定等待时间 | 1 $\mu s$ - 100 ms | 无锁数据结构(61.5 节) |
| **系统 I/O** (`printf`/`write`/`send`) | 内核缓冲区 + 磁盘/网络阻塞 | 10 $\mu s$ - 100 ms | 无锁日志队列 + 异步写入 |
| **缺页中断** (page fault) | 访问未映射的虚拟页触发内核换页 | 1 ms - 100 ms | `mlockall` + 栈预触碰 |

### `mlockall`——锁定所有内存页

Linux 的虚拟内存系统有一个特性:虚拟地址空间中的页面可能尚未映射到物理内存(lazy allocation),或者已被 swap 到磁盘。当程序访问这样的页面时,CPU 触发缺页中断(page fault),内核需要分配物理页或从磁盘读回数据——这个过程可能耗时数毫秒。

`mlockall` 强制将所有内存页锁定在物理内存中,防止 swap:

```cpp
#include <sys/mman.h>
#include <cstring>

void setupRealtimeMemory() {
  // Lock all current and future memory pages into RAM
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    // MCL_CURRENT: lock all currently mapped pages
    // MCL_FUTURE:  lock all pages mapped in the future
    // Requires: ulimit -l unlimited or CAP_IPC_LOCK
  }
}
```

**但 `mlockall` 不够**——它只防止已映射页被 swap,不防止**新页面的首次访问**触发 minor page fault(将虚拟页映射到物理页)。解决方案是**栈预触碰**(stack prefault):

```cpp
void prefaultStack() {
  // Pre-touch 8 MB of stack to force physical page allocation
  constexpr size_t STACK_SIZE = 8 * 1024 * 1024;  // 8 MB
  volatile char stack_probe[STACK_SIZE];
  // Touch every page (4 KB boundary)
  for (size_t i = 0; i < STACK_SIZE; i += 4096) {
    stack_probe[i] = 0;
  }
  // Now all stack pages are physically allocated and locked
}
```

> ⚠️ **编程陷阱:忘记预触碰栈导致首次循环延迟突增**
>
> **现象**: 控制循环的前 10-20 次迭代延迟明显高于后续(可能高 10-100 倍)。
>
> **根本原因**: 即使 `mlockall(MCL_FUTURE)` 锁定了未来分配的页面,首次写入新栈帧时仍会触发 minor page fault(约 1-5 $\mu s$ each)。如果控制循环中有深层函数调用链,首次迭代需要分配多个栈页面。
>
> **正确做法**: 在进入控制循环前调用 `prefaultStack()`。或者使用 `pthread_attr_setstack` 设置预分配的栈空间。

### CPU 亲和性与核心隔离

**CPU 亲和性**(CPU Affinity)将线程绑定到特定的 CPU 核心,避免核间迁移(migration)带来的 cache 失效:

```cpp
#include <pthread.h>
#include <sched.h>

void pinThreadToCore(pthread_t thread, int core_id) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  int ret = pthread_setaffinity_np(thread, sizeof(cpuset), &cpuset);
  if (ret != 0) {
    perror("pthread_setaffinity_np");
  }
}
```

**核心隔离**(Core Isolation)更进一步——把某些 CPU 核心从 Linux 调度器中**完全移除**,只有显式绑定的任务才能在上面运行:

```bash
# /etc/default/grub
GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3"
```

| 参数 | 作用 | 效果 |
|------|------|------|
| `isolcpus=2,3` | 从 CFS 调度器移除 CPU 2, 3 | 普通进程不会被调度到这些核心 |
| `nohz_full=2,3` | 禁用 CPU 2, 3 的 tick 中断 | 当只有一个任务运行时,消除 tick 中断干扰 |
| `rcu_nocbs=2,3` | 将 RCU 回调移到其他 CPU | 避免 RCU 回调在隔离核心上执行 |

**典型的腿足机器人 CPU 分配**:

```
┌──────────────────────────────────────────────────┐
│ CPU 0: System + ROS 2 communication + DDS        │
│ CPU 1: MPC solver (non-isolated, SCHED_FIFO 70)  │
│ CPU 2: WBC loop (ISOLATED, SCHED_FIFO 85)        │
│ CPU 3: Hardware driver (ISOLATED, SCHED_FIFO 80) │
│ CPU 4-7: State estimation, perception, logging   │
└──────────────────────────────────────────────────┘
```

### 精确定时:`clock_nanosleep`

`std::this_thread::sleep_for` 的精度取决于内核 tick 频率(通常 250 Hz,即 4 ms 粒度),完全不适合 1 kHz 控制循环。正确的做法是使用 POSIX 的 `clock_nanosleep` 配合**绝对时间**:

```cpp
#include <time.h>

void realtimeLoop() {
  struct timespec next_wakeup;
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

  constexpr long PERIOD_NS = 1000000;  // 1 ms = 1,000,000 ns

  while (running) {
    // --- realtime work begins ---
    doControlComputation();
    // --- realtime work ends ---

    // Advance to next period (absolute time)
    next_wakeup.tv_nsec += PERIOD_NS;
    if (next_wakeup.tv_nsec >= 1000000000L) {
      next_wakeup.tv_sec += 1;
      next_wakeup.tv_nsec -= 1000000000L;
    }

    // Sleep until absolute wakeup time
    // TIMER_ABSTIME: sleep until next_wakeup, not "for" a duration
    // This prevents drift accumulation!
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
  }
}
```

**为什么用绝对时间而非相对时间?** 如果用 `sleep_for(1ms - elapsed)`,每次循环的计算时间误差会累积——100 次循环后,时钟可能漂移数百微秒。用绝对时间定位下一个唤醒点,**计算时间变化不影响周期精度**。

### NUMA 感知

在多 socket 服务器(如工控机)上,内存访问时间取决于 CPU 和内存的物理距离。**NUMA**(Non-Uniform Memory Access)意味着 CPU 0 访问"自己的"内存可能只需 40 ns,但访问另一个 socket 的内存需要 100+ ns。

对于大多数腿足机器人(单 socket 的 Intel NUC / Jetson / 工控机),NUMA 不是问题。但如果你的系统是双 socket 服务器,需要确保实时线程和它使用的内存在同一个 NUMA 节点:

```bash
# Check NUMA topology
numactl --hardware

# Pin process to NUMA node 0
numactl --cpunodebind=0 --membind=0 ./my_controller
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:`sleep_for` 在实时循环中导致周期漂移**
>
> **错误做法**: `std::this_thread::sleep_for(std::chrono::milliseconds(1) - elapsed);`
>
> **现象**: 控制循环频率从 1000 Hz 逐渐降到 990-995 Hz,10 分钟后漂移数秒。
>
> **根本原因**: `sleep_for` 是相对时间,每次的微小误差会累积。另外,`sleep_for` 的分辨率在非 RT 内核上可能只有 1-10 ms。
>
> **正确做法**: 使用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` 配合绝对唤醒时间。

> 🧠 **思维陷阱:认为"核心隔离后就完全不会被干扰"**
>
> **新手想法**: "我用了 `isolcpus`,这个核心完全属于我的实时线程了!"
>
> **实际上**: `isolcpus` 只隔离了 CFS 调度器。以下中断源仍然会打断你的线程:
> - 硬件中断(如 NMI、SMI、local APIC timer)
> - 内核的 tick 中断(除非也配了 `nohz_full`)
> - RCU 回调(除非也配了 `rcu_nocbs`)
> - 其他 SCHED_FIFO 线程(如果也绑定到同一核心)
>
> **正确做法**: `isolcpus` + `nohz_full` + `rcu_nocbs` 三件套一起用。并用 `cyclictest` 验证效果。

### 练习

**练习 61.3a** ⭐⭐: 编写一个完整的实时线程模板,包含:SCHED_FIFO 设置、mlockall、栈预触碰、CPU 亲和性、`clock_nanosleep` 绝对定时。在循环内执行一个 12x12 矩阵乘法(模拟 WBC 计算),统计 10,000 次循环的 jitter 分布(min/avg/max/p99)。

**练习 61.3b** ⭐⭐: 对比以下三种定时方式的精度:① `std::this_thread::sleep_for` ② `usleep` ③ `clock_nanosleep(TIMER_ABSTIME)`。在 RT 内核和标准内核上分别测试 1 ms 周期的 jitter。用直方图可视化结果。

---

实时线程编程解决了"环境保障"问题——正确的调度、内存锁定、精确定时。但仅有环境保障还不够,控制循环内部的代码本身也必须避免一切不确定性延迟。其中最大的隐患就是堆分配。

## 61.4 无堆分配编程 ⭐⭐

### 动机:一次 `malloc` 的真实代价

`malloc` 不只是"分配一块内存"那么简单。在 glibc 的实现中,一次 `malloc` 可能涉及:

1. **获取分配器内部锁**: glibc 的 ptmalloc2 使用 arena-based 策略,每个 arena 有一个 mutex。多线程场景下可能竞争。延迟: 0.1-10 $\mu s$
2. **搜索 free list**: 在已释放的块中寻找合适大小的块。延迟取决于碎片化程度: 0.1-5 $\mu s$
3. **`mmap` 系统调用**: 如果 free list 中没有合适的块,需要向内核申请新页面。延迟: 10-100 $\mu s$
4. **缺页中断**: 新映射的页面首次写入时触发 minor fault。延迟: 1-10 $\mu s$ per page
5. **TLB 刷新**: 新页面的虚拟-物理映射需要更新 TLB。在多核系统上可能触发 IPI(Inter-Processor Interrupt)。延迟: 1-5 $\mu s$

**总计最坏情况: 数百微秒到数毫秒**——对 1 ms 控制周期来说是致命的。内存池的思路好比预装弹夹——士兵在战场上不会现场去弹药库领子弹（`malloc`），而是提前把弹夹装满（预分配 buffer），战斗中直接换弹夹即可（O(1) 分配）。`pmr::monotonic_buffer_resource` 就是这样的"预装弹夹"。

### `EIGEN_RUNTIME_NO_MALLOC`——开发期的安全网

Ch53 已经详细介绍了 `EIGEN_RUNTIME_NO_MALLOC` 的原理和实践(见 53.6 节)。本节在此基础上,从**工程集成**的角度进一步展开。

**回顾核心机制**: Ch53.6 建立了 Eigen 实时安全编程的基础——通过 `EIGEN_RUNTIME_NO_MALLOC` 宏,Eigen 的内部分配函数在每次 `malloc` 前检查全局标志 `is_malloc_allowed`。在 Ch53 中我们用它保护 WBC 的 QP 求解循环；这里同样的机制被扩展到整个控制管线。具体而言: 当定义了该宏时,如果 `Eigen::internal::set_is_malloc_allowed(false)` 被调用后仍有分配请求,触发 `eigen_assert` 失败,立即暴露违规代码的位置。

**Ch53 未覆盖的工程问题——CI/CD 集成**:

在持续集成中自动检测 Eigen 内存违规:

```cmake
# In CMakeLists.txt: enable for all build types during CI
option(ENFORCE_RT_MALLOC_CHECK
  "Enable EIGEN_RUNTIME_NO_MALLOC in all builds (for CI)" OFF)

if(ENFORCE_RT_MALLOC_CHECK OR CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_definitions(my_wbc PRIVATE EIGEN_RUNTIME_NO_MALLOC)
endif()
```

```yaml
# In .github/workflows/ci.yml or similar
- name: Build with malloc detection
  run: |
    cmake -B build -DENFORCE_RT_MALLOC_CHECK=ON
    cmake --build build
- name: Run RT safety tests
  run: |
    cd build && ctest --label-regex "rt_safety"
```

**哪些 Eigen 操作会触发堆分配?**

Ch53.6.4 列出了常见违规操作。这里用表格形式补充完整对照:

| 操作 | 示例 | 是否分配 | 说明 |
|------|------|---------|------|
| 构造 | `VectorXd v(12);` | **是** | 动态大小需要 malloc |
| 赋值(大小变化) | `A = B;`(A 未初始化或大小不同) | **是** | 需要 realloc |
| 赋值(大小相同) | `A = B;`(A 已有正确大小) | **否** | 直接 memcpy |
| 乘法 | `C = A * B;`(C 未预分配) | **是** | 结果需要存储空间 |
| 乘法 | `C.noalias() = A * B;`(C 已预分配) | **否** | 直接写入 C |
| `resize()` | `A.resize(m, n);` | **视情况** | 大小不变则不分配 |
| `conservativeResize()` | `A.conservativeResize(m, n);` | **是** | 总是分配+拷贝 |
| 块操作 | `A.block(0,0,3,3)` | **否** | 返回引用,不分配 |
| `eval()` | `(A * B).eval()` | **是** | 创建临时矩阵 |

### 预分配模式:三种策略

**策略 1:固定大小矩阵(编译期已知维度)**

```cpp
// ✅ Best: compile-time size, stack allocation, SIMD-friendly
Eigen::Matrix<double, 12, 12> M;       // 1152 bytes on stack
Eigen::Matrix<double, 12, 1> tau;      // 96 bytes on stack
Eigen::Matrix<double, 3, 3> R;         // 72 bytes on stack
// No malloc, ever. Compiler can use AVX2/SSE for operations.
```

**策略 2:预分配动态矩阵(运行期决定维度,但大小不变)**

```cpp
class WBC {
public:
  WBC(int n_dof) {
    // Allocate once during construction (non-RT context)
    M_.resize(n_dof, n_dof);
    J_.resize(6, n_dof);
    tmp_.resize(n_dof);
  }

  void update() {
    // ✅ No allocation: M_, J_, tmp_ already have correct size
    M_ = data_.M;                  // Size matches -> no realloc
    tmp_.noalias() = M_ * v_;      // Output to pre-allocated tmp_
  }

private:
  Eigen::MatrixXd M_, J_;
  Eigen::VectorXd tmp_;
};
```

**策略 3:对象池(运行期数量变化)**

当接触点数量在步态切换时变化(例如 trot 4 个接触点转 gallop 2 个),`std::vector<ContactForce>` 的 `push_back` 可能触发 realloc。对象池预先分配最大容量:

```cpp
// ✅ Object pool: pre-allocate max capacity, never realloc
template <typename T, size_t MaxSize>
class FixedPool {
public:
  T& acquire() {
    assert(size_ < MaxSize && "Pool exhausted!");
    return data_[size_++];
  }

  void releaseAll() { size_ = 0; }  // O(1) reset, no deallocation

  size_t size() const { return size_; }
  T& operator[](size_t i) { return data_[i]; }

private:
  std::array<T, MaxSize> data_;  // Stack-allocated fixed array
  size_t size_ = 0;
};

// Usage in WBC:
FixedPool<Eigen::Vector3d, 8> contact_forces_;  // Max 8 contact points
```

### `std::pmr`——C++17 的实时友好分配器

Ch35 介绍了 `std::pmr` 的基础,Ch53.6.5 展示了在 WBC 中使用 `pmr::vector` 的基本模式。这里深入讲解 `monotonic_buffer_resource` 为什么特别适合实时控制的**原理**。

`std::pmr::monotonic_buffer_resource` 是一个"只分配不释放"的分配器——它在一个预分配的缓冲区上做 bump allocation(指针递增)。为什么这对实时系统如此重要?

1. **分配操作是 O(1)**:只需要将指针向前移动,不需要搜索 free list
2. **延迟完全确定**:约 2-5 ns per allocation,无论分配了多少次
3. **无锁**:单线程使用时不需要任何同步
4. **无系统调用**:所有内存来自预分配的缓冲区,不会触发 `mmap` 或 `brk`

```cpp
#include <memory_resource>
#include <vector>

class RealtimeWBC {
public:
  RealtimeWBC()
    : pool_(buffer_, sizeof(buffer_),     // Use pre-allocated buffer
            std::pmr::null_memory_resource())  // Fail if buffer exhausted
    , contact_wrenches_(&pool_)
  {}

  void compute() {
    // 每个控制周期只清空逻辑内容，不释放容量、不重置 pool。
    // reserve() 已在构造函数中完成，因此 push_back 不会触发分配。
    contact_wrenches_.clear();
    for (int i = 0; i < n_contacts_; ++i) {
      contact_wrenches_.push_back(computeContactWrench(i));  // No malloc!
    }
  }

private:
  alignas(64) std::byte buffer_[4096];  // 4 KB pre-allocated buffer
  std::pmr::monotonic_buffer_resource pool_;
  std::pmr::vector<Eigen::Matrix<double, 6, 1>> contact_wrenches_;
  int n_contacts_ = 4;
};
```

> ⚠️ **C++ 陷阱**：`pmr::monotonic_buffer_resource::release()` 会使所有已分配的内存失效。如果 `pmr::vector` 仍持有来自该 pool 的 storage，就不能在实时循环中调用 `release()`；`shrink_to_fit()` 也不是实时安全接口。需要每周期临时分配时，应把 `pmr` 容器放在局部作用域内，让容器先析构，再在非实时或受控位置 `release()`。

**为什么用 `null_memory_resource` 作为 upstream?** 如果 monotonic buffer 耗尽且 upstream 是默认的 `new_delete_resource`,它会 fallback 到 `malloc`——这违反了实时约束。`null_memory_resource` 会直接抛异常(或在 `-fno-exceptions` 下 abort),让 bug 在测试时暴露而非在运行时静默分配。

### 需要避免的 C++ 标准库设施

| STL 设施 | 是否 RT 安全 | 原因 | 替代方案 |
|---------|-------------|------|---------|
| `std::vector` | ❌ | `push_back` 可能 realloc | `std::array` / `FixedPool` / `pmr::vector` |
| `std::string` | ❌ | 长字符串(>15 chars)堆分配 | `std::string_view` / 固定长度 `char[]` |
| `std::function` | ❌ | 类型擦除内部可能堆分配 | 模板 / `std::function_ref`(C++26) / `inplace_function` |
| `std::map/set` | ❌ | 每次 insert 都是一次 `new` | `std::array` + 手动排序 / flat_map |
| `std::shared_ptr` | ❌ | 控制块堆分配 + 原子引用计数 | `std::unique_ptr`(非 RT 路径) / 裸指针(RT 路径) |
| `std::array` | ✅ | 栈分配,编译期大小 | — |
| `std::span` | ✅ | 非拥有视图,无分配 | — |
| `std::optional` | ✅ | 内联存储,无分配 | — |
| `std::variant` | ✅ | 内联存储,无分配 | — |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:`Eigen::VectorXd` 的隐式 realloc**
>
> **错误做法**:
> ```cpp
> Eigen::VectorXd tau;  // Size 0
> tau = wbc.solve();    // solve() returns VectorXd of size 12
>                       // -> tau needs realloc from 0 to 12!
> ```
>
> **现象**: `EIGEN_RUNTIME_NO_MALLOC` 开启后 assert 失败。关闭后运行正常但偶尔卡顿。
>
> **正确做法**:
> ```cpp
> Eigen::VectorXd tau(12);  // Pre-allocate in constructor
> tau = wbc.solve();        // Size already matches -> no realloc
> ```

> 💡 **概念误区:认为 `std::vector::reserve` 就完全安全了**
>
> **新手想法**: "我 `reserve(100)` 了,`push_back` 100 次内不会 realloc。"
>
> **部分正确**: `reserve` 确实预分配了容量。但如果你调用 `clear()` 后再 `push_back`,容量保持(安全)。然而,如果你赋值 `vec = other_vec;` 且 `other_vec` 容量不同,**容量可能缩小**,后续 `push_back` 又会 realloc。
>
> **更安全的做法**: 使用 `pmr::vector` + `monotonic_buffer_resource`,或者使用固定大小的 `std::array`。

> ⚠️ **编程陷阱:异常对象的堆分配**
>
> **错误做法**: 在实时循环中用 `try-catch`:
> ```cpp
> try {
>   tau = wbc.compute(q, v);
> } catch (const std::exception& e) {
>   // Exception object is heap-allocated!
>   // Even just constructing the exception may malloc
> }
> ```
>
> **正确做法**: 实时路径上用返回值/错误码替代异常。在 CMake 中为实时库禁用异常:
> ```cmake
> target_compile_options(my_rt_lib PRIVATE -fno-exceptions)
> ```

### 练习

**练习 61.4a** ⭐⭐: 写一个程序,开启 `EIGEN_RUNTIME_NO_MALLOC`,然后故意触发以下 5 种 Eigen 堆分配并逐一修复:①未初始化 VectorXd 赋值 ②矩阵乘法结果未预分配 ③`conservativeResize` ④`eval()` 临时变量 ⑤动态大小矩阵构造。验证修复后在 `set_is_malloc_allowed(false)` 下不触发 assert。

**练习 61.4b** ⭐⭐: 实现一个 `pmr::monotonic_buffer_resource` 支持的 WBC 内存管理器:预分配 64 KB 缓冲区,支持每帧 `reset()`。用 `std::chrono::high_resolution_clock` 测量 1000 次分配的平均时间,与 `new/delete` 对比。

**练习 61.4c** ⭐⭐⭐: 阅读 Eigen 源码中 `Memory.h` 的 `check_that_malloc_is_allowed()` 函数实现。解释为什么 `EIGEN_RUNTIME_NO_MALLOC` 是**运行时**检查而非编译期检查——这意味着什么局限性?有没有办法做到编译期检测?

---

消除了堆分配之后,实时路径上的另一个主要延迟源是线程间通信。传统的 `std::mutex` 会导致优先级反转和不确定性阻塞。解决方案是无锁数据结构——这是本章难度最高的部分。

## 61.5 无锁数据结构 ⭐⭐⭐

### 动机:MPC 和 WBC 之间的数据传递问题

回顾 Ch55 的双线程架构:

```
MPC 线程 (20-100 Hz)                  WBC 线程 (1 kHz)
┌──────────────────┐                  ┌──────────────────┐
│ Solve OCP        │   How to pass    │ Read MPC result  │
│ -> MPC solution  │ ===============> │ -> Compute torque│
│ (x_ref, u_ref,   │   data safely?   │ -> Send to motors│
│  value function) │                  │                  │
└──────────────────┘                  └──────────────────┘
  ~20-50 ms per solve                   1 ms per cycle
```

**问题**: MPC 每 20-50 ms 产出一个新解,WBC 每 1 ms 需要读取最新的 MPC 解。如果用 `mutex`:

- WBC 获取锁 → 读数据(约 10 $\mu s$) → 释放锁: 正常情况下很快
- 但如果 WBC 获取锁时 MPC 正在写数据(持有锁) → WBC **阻塞等待** → 等待时间 = MPC 的写入时间(可能几十 $\mu s$) → **不确定性延迟**
- 更糟:如果 WBC 被阻塞,优先级反转可能让等待时间膨胀到毫秒级

### 无锁的核心思想

**无锁**(lock-free)不等于"没有同步"——它是用**原子操作**(`std::atomic`)替代锁,保证至少一个线程总在前进(progress guarantee)。Triple Buffer 的工作方式好比餐厅传菜窗口的旋转托盘——厨师（MPC 线程）把做好的菜放在托盘的一个槽位上，服务员（WBC 线程）随时从另一个槽位取最新的菜，两人互不等待、互不阻塞，托盘的原子旋转保证了服务员总能拿到最新出品:

| 属性 | 有锁 (mutex) | 无锁 (lock-free) | 无等待 (wait-free) |
|------|-------------|-----------------|-------------------|
| **阻塞** | 可能无限期阻塞 | 不阻塞,但可能 spin | 永不 spin |
| **进度保证** | 无 | 系统级: 总有线程在前进 | 线程级: 每个线程都在前进 |
| **优先级反转** | 可能 | 不可能 | 不可能 |
| **实现复杂度** | 低 | 中-高 | 高 |
| **适合实时** | 不适合 | **适合** | 最适合(但难实现) |

### SPSC 无锁环形队列

**SPSC**(Single-Producer Single-Consumer)队列是最简单也最高效的无锁数据结构。它只允许一个线程写、一个线程读,这恰好匹配 MPC→WBC 的通信模式。

**为什么 SPSC 比 MPMC 简单?** 因为只有一个写者和一个读者,不需要 CAS(Compare-And-Swap)循环,只需要 `release/acquire` 语义的原子变量:

```cpp
#include <atomic>
#include <array>
#include <cstddef>

// Wait-free SPSC ring buffer
// Based on rigtorp/SPSCQueue design
template <typename T, size_t Capacity>
class SPSCQueue {
  static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0,
                "Capacity must be power of 2");
public:
  // Called only by producer thread
  bool tryPush(const T& item) {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) & (Capacity - 1);

    if (next == tail_.load(std::memory_order_acquire)) {
      return false;  // Queue full, don't block!
    }

    data_[head] = item;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Called only by consumer thread
  bool tryPop(T& item) {
    const size_t tail = tail_.load(std::memory_order_relaxed);

    if (tail == head_.load(std::memory_order_acquire)) {
      return false;  // Queue empty, don't block!
    }

    item = data_[tail];
    tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
    return true;
  }

private:
  // Avoid false sharing: put head and tail on different cache lines
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
  std::array<T, Capacity> data_;
};
```

**关键设计决策**:

1. **`memory_order_release/acquire`** 而非 `seq_cst`: `seq_cst` 在 x86 上需要 `MFENCE` 指令(约 20 ns),而 `release/acquire` 只需要编译器屏障(在 x86 上约 0 ns 额外开销,因为 x86 的 TSO 模型天然保证 acquire/release 语义)。在 ARM 上,`release` 使用 `DMB` 指令,比 `seq_cst` 轻量。

2. **Cache line padding** (`alignas(64)`): 如果 `head_` 和 `tail_` 在同一条 cache line(64 bytes)上,一个线程写 `head_` 会导致另一个线程的 `tail_` 缓存失效——这叫 **false sharing**,可能增加 20-50 ns 延迟。

3. **Power-of-2 容量**: 使用 `& (Capacity - 1)` 代替 `% Capacity`,位运算比取模快 5-10 倍。

### Triple Buffer

Ch53.8.2 和 Ch55.7 已经从不同角度深入介绍了 Triple Buffer——Ch53 讲了 MPC+WBC 集成的概念和简化实现,Ch55 讲了 OCS2 的 `BufferedValue` 工业级实现和 `memory_order` 选择。本节聚焦于**选型决策**和**实际部署经验**。

**Triple Buffer vs SPSC Queue: 何时用哪个?**

| 场景 | Triple Buffer | SPSC Queue |
|------|-------------|-----------|
| 数据语义 | "最新值"(overwrite) | "所有值"(stream) |
| 读者跳过旧值 | ✅ 自动跳过 | ❌ 必须逐个读 |
| 写者等待读者 | ❌ 永不等待 | 可能队列满 |
| 典型用途 | MPC 解 → WBC | 日志消息 → 磁盘 |
| 延迟确定性 | 常数时间 | 常数时间(tryPush) |

**MPC→WBC 应该用 Triple Buffer**(如 Ch55 所述),因为 WBC 只需要**最新的** MPC 解,不需要历史值。如果 MPC 在 WBC 两次读取之间产出了 2 个解,WBC 应该跳过旧的、只读最新的。

**实际部署中的 Triple Buffer 实现**: Ch55.7 详细介绍了 OCS2 中 Triple Buffer 的架构设计——三个槽位（write / middle / read）通过原子交换实现无锁通信,写者永远不阻塞、读者总是拿到最新数据；Ch53.8.2 则从 WBC 集成的角度说明了 MPC 解如何通过 Triple Buffer 传递给实时 WBC 循环。下面给出一个简化但功能完整的 C++ 实现,供部署参考:

```cpp
#include <atomic>
#include <array>

template <typename T>
class TripleBuffer {
public:
  // Producer writes to the "write" slot, then publishes it
  T& getWriteBuffer() { return buffers_[writeIdx_]; }

  void publishWrite() {
    // Swap write index with middle index (atomic)
    writeIdx_ = middleIdx_.exchange(writeIdx_, std::memory_order_acq_rel);
    newData_.store(true, std::memory_order_release);
  }

  // Consumer reads from the "read" slot, optionally swapping with middle
  const T& getReadBuffer() {
    if (newData_.load(std::memory_order_acquire)) {
      // Swap read index with middle index (atomic)
      readIdx_ = middleIdx_.exchange(readIdx_, std::memory_order_acq_rel);
      newData_.store(false, std::memory_order_release);
    }
    return buffers_[readIdx_];
  }

private:
  std::array<T, 3> buffers_;
  int writeIdx_ = 0;   // Only accessed by producer
  int readIdx_ = 2;    // Only accessed by consumer
  std::atomic<int> middleIdx_{1};  // Shared swap point
  std::atomic<bool> newData_{false};
};
```

### Seqlock——读者优化的无锁同步

Seqlock(序列锁)适用于**写少读多**且**读者可以容忍偶尔重试**的场景。在腿足机器人中,状态估计数据(IMU 姿态)就是典型场景:状态估计线程每 1 ms 更新一次,多个消费者(WBC、MPC、日志)需要读取。

```cpp
#include <atomic>

template <typename T>
class Seqlock {
public:
  // Writer (must be single writer)
  void write(const T& value) {
    seq_.store(seq_.load(std::memory_order_relaxed) + 1,
               std::memory_order_release);  // Odd = writing in progress
    data_ = value;
    seq_.store(seq_.load(std::memory_order_relaxed) + 1,
               std::memory_order_release);  // Even = write complete
  }

  // Reader (can be multiple readers, lock-free with retry)
  T read() const {
    T result;
    unsigned seq0, seq1;
    do {
      seq0 = seq_.load(std::memory_order_acquire);
      if (seq0 & 1) continue;  // Writer is in progress, retry immediately
      result = data_;
      seq1 = seq_.load(std::memory_order_acquire);
    } while (seq0 != seq1);    // Data changed during our read, retry
    return result;
  }

private:
  std::atomic<unsigned> seq_{0};
  T data_;
};
```

**Seqlock vs Triple Buffer**:

| 属性 | Seqlock | Triple Buffer |
|------|---------|-------------|
| 读者数量 | 多个 | 单个 |
| 写者数量 | 单个 | 单个 |
| 读者重试 | 可能(写入期间) | 无重试 |
| 内存开销 | 1 份数据 + 1 个计数器 | 3 份数据 |
| 适用场景 | 小数据,多读者 | 大数据,单读者 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:`std::atomic` 操作不一定是无锁的**
>
> **新手想法**: "我用了 `std::atomic<MyStruct>`,就是无锁的了!"
>
> **实际上**: `std::atomic<T>` 只有在 `T` 的大小 $\leq$ 平台原子操作宽度(x86_64 上是 8 或 16 bytes)时才是真正无锁的。更大的类型会退化为内部使用 mutex。
>
> **自检方法**: `static_assert(std::atomic<T>::is_always_lock_free, "Not lock-free!");`
>
> **正确做法**: 如果数据太大(如 MPC 解包含完整轨迹),不要用 `atomic<MPCSolution>`——用 Triple Buffer 或 SPSC 队列传递。

> 🧠 **思维陷阱:认为"无锁一定比有锁快"**
>
> **新手想法**: "无锁数据结构是终极方案,一定比 mutex 快!"
>
> **实际上**: 在**无竞争**时,mutex 的开销只有约 15-25 ns(一次 futex 不进入内核),而 CAS 循环在高竞争时可能 spin 很久。无锁的优势不在于速度,而在于**确定性**——延迟有上界,不会因优先级反转而无限增长。
>
> **正确理解**: 选择无锁的理由是**实时安全性**(可预测的最坏延迟),而非性能。

> ⚠️ **编程陷阱:忽略 `memory_order` 导致 ARM 上出错**
>
> **错误做法**: 在所有原子操作上使用 `memory_order_relaxed`。
>
> **现象**: x86 上一切正常(因为 TSO 模型天然较强),ARM/Jetson 上出现数据撕裂。
>
> **根本原因**: ARM 的弱内存模型允许指令重排,`relaxed` 不提供任何排序保证。数据写入可能在标志位更新之后才对其他核心可见。
>
> **正确做法**: 生产者用 `release`,消费者用 `acquire`——这是最小充分的排序约束。如果不确定,用 `seq_cst`(性能差一些但永远正确)。

### 练习

**练习 61.5a** ⭐⭐: 实现上述 SPSC 环形队列,并用两个线程测试:生产者每 20 ms 写入一个递增计数器,消费者每 1 ms 读取。验证:①消费者从不阻塞 ②读到的值单调递增(允许重复但不允许跳回) ③队列满时生产者不阻塞。

**练习 61.5b** ⭐⭐⭐: 实现 Triple Buffer 模板类。用 Google Benchmark 测量在 MPC(30 ms 周期)+ WBC(1 ms 周期)模拟场景下的读写延迟。与 `std::mutex` 方案对比,画出延迟分布图。

**练习 61.5c** ⭐⭐⭐: 在你的 Triple Buffer 实现中,故意使用 `memory_order_relaxed` 替换所有 `acquire/release`。在 ARM 平台(如 Raspberry Pi 或 Jetson)上用 ThreadSanitizer 运行测试。是否能检测到数据竞争?在 x86 上呢?解释差异。

---

前面四节建立了实时编程的基础设施——调度、内存、同步。现在该把这些技术集成到 ROS 2 的控制框架中。`ros2_control` 是 ROS 2 官方的硬件抽象和控制器管理框架,腿足社区(包括 legged_control)都基于它构建。

## 61.6 ros2_control 实时架构 ⭐⭐

### 动机:为什么不直接写 `while(true)` 循环

你可能会想:"我已经知道怎么写实时线程了——`SCHED_FIFO` + `mlockall` + `clock_nanosleep`。为什么还需要 `ros2_control` 这个框架?"

答案是**解耦与复用**:

```
没有框架:                         有 ros2_control:
┌──────────────┐                  ┌──────────────┐
│ 你的代码      │                  │ 你的代码      │
│ - 硬件通信    │                  │ - 只写控制算法│
│ - 定时循环    │                  │ (ControllerInterface)│
│ - 控制算法    │                  └───────┬──────┘
│ - ROS 通信    │                          │ update() called by framework
│ - 参数管理    │                  ┌───────▼──────┐
│ - 生命周期    │                  │ ros2_control  │
│ 全部混在一起! │                  │ - 硬件抽象    │
└──────────────┘                  │ - 定时循环    │
                                  │ - ROS 通信    │
硬件换了?重写。                    │ - 生命周期    │
算法换了?重写。                    └──────────────┘
两人不能并行开发。
                                  硬件换了?换 HardwareInterface。
                                  算法换了?换 Controller。
                                  两人并行开发。
```

### ros2_control 架构总览

```
┌──────────────────────────────────────────────────────────────┐
│                     Controller Manager                       │
│  (Lifecycle Node, manages all controllers and HW interfaces) │
│                                                              │
│  ┌────────────────────┐  ┌────────────────────┐             │
│  │ Controller A       │  │ Controller B       │             │
│  │ (e.g., WBC)        │  │ (e.g., PD fallback)│             │
│  │ update() @ 1kHz    │  │ update() @ 1kHz    │             │
│  └────────┬───────────┘  └────────┬───────────┘             │
│           │ state_interfaces       │ command_interfaces      │
│  ┌────────▼───────────────────────▼───────────┐             │
│  │          Resource Manager                   │             │
│  │  (manages all state/command interfaces)     │             │
│  └────────┬───────────────────────┬───────────┘             │
│  ┌────────▼──────┐  ┌────────────▼──────┐                   │
│  │ HW Interface A│  │ HW Interface B    │                   │
│  │ (Unitree SDK) │  │ (EtherCAT)        │                   │
│  │ read()/write()│  │ read()/write()    │                   │
│  └───────────────┘  └───────────────────┘                   │
└──────────────────────────────────────────────────────────────┘
```

**Controller Manager 的实时循环**(伪代码简化):

```cpp
// This runs in a SCHED_FIFO thread
void ControllerManager::readWriteCycleUpdate() {
  // 1. Read from all hardware interfaces (get sensor data)
  for (auto& hw : hardware_interfaces_) {
    hw.read(time, period);  // Read joint positions, velocities, efforts
  }

  // 2. Update all active controllers (compute commands)
  for (auto& ctrl : active_controllers_) {
    ctrl.update(time, period);  // WBC, PD, etc.
  }

  // 3. Write to all hardware interfaces (send commands)
  for (auto& hw : hardware_interfaces_) {
    hw.write(time, period);  // Send torques to motors
  }
}
```

### `realtime_tools`——ros2_control 的实时工具库

`realtime_tools` 提供了几个关键组件,让 Controller 在实时线程中安全地与 ROS 2 的非实时部分交互。

**RealtimePublisher——安全地从实时线程发布 ROS 消息**:

ROS 2 的 `rclcpp::Publisher` 不是实时安全的——`publish()` 内部可能分配内存(序列化)、获取锁(DDS 层)。`RealtimePublisher` 的解决方案:实时线程只填充消息数据(无锁),一个非实时辅助线程负责实际发布:

```cpp
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class LeggedController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State&) override {
    // Create publisher (non-RT context, allocation OK)
    rt_publisher_ = std::make_shared<
        realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/joint_torques", rclcpp::SystemDefaultsQoS()));

    // ⚠️ 关键：在 configure 阶段预分配消息和 data vector
    // Float64MultiArray.data 是 std::vector<double>，
    // 如果在 RT update() 中构造或 resize 会触发 malloc！
    msg_.data.resize(12);  // 预分配 12 个关节的空间

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override {
    // RT-safe: 只写入预分配的 buffer，不做任何内存分配
    // try_publish is non-blocking: if the non-RT thread is busy
    // publishing the previous message, this just returns false
    for (int i = 0; i < 12; ++i) {
      msg_.data[i] = tau_[i];  // 写入已有 buffer，无 malloc
    }
    rt_publisher_->try_publish(msg_);  // Non-blocking!

    return controller_interface::return_type::OK;
  }

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<
      std_msgs::msg::Float64MultiArray>> rt_publisher_;
  std_msgs::msg::Float64MultiArray msg_;  // 预分配的消息对象（成员变量）
};
```

> **API 变更注意**: 自 Jazzy 版本起,`RealtimePublisher` 的 API 已更新。旧版使用 `rt_publisher_->lock()` + `rt_publisher_->msg_` + `rt_publisher_->unlockAndPublish()`,新版推荐使用 `try_publish(msg)` 方法。旧 API 已标记为 deprecated。同样,`RealtimeBox` 已被 `RealtimeThreadSafeBox` 取代。

**RealtimeThreadSafeBox——实时安全的"最新值"容器**:

用于在 RT 和非 RT 线程之间交换单个值:

```cpp
#include <realtime_tools/realtime_thread_safe_box.hpp>

class MyController : public controller_interface::ControllerInterface {
  // Non-RT thread writes new reference (e.g., from a subscriber callback)
  // RT thread reads the latest reference
  //
  // ⚠️ RT 安全警告：Eigen::VectorXd 是动态大小类型，
  // try_get() 返回 std::optional 时内部 copy 可能触发堆分配！
  // 生产代码应使用固定尺寸类型避免 RT 路径上的 malloc：
  //   方案1: Eigen::Matrix<double, 12, 1>（编译期固定大小，栈分配）
  //   方案2: std::array<double, 12>（POD-like，零分配开销）
  //   方案3: 自定义 POD struct（如 struct MpcRef { double data[24]; }）
  // 此处为演示 API 用法，使用 VectorXd；部署时务必替换。
  realtime_tools::RealtimeThreadSafeBox<Eigen::VectorXd> mpc_reference_;

  void mpcCallback(const MpcMsg::SharedPtr msg) {
    // Non-RT callback
    Eigen::VectorXd ref = convertMsgToEigen(msg);
    mpc_reference_.try_set(ref);  // Non-blocking write
  }

  controller_interface::return_type update(
      const rclcpp::Time&, const rclcpp::Duration&) override {
    // RT update
    auto ref = mpc_reference_.try_get();  // Non-blocking read
    if (ref) {
      wbc_.setReference(*ref);
    }
    // ...compute and send torques...
    return controller_interface::return_type::OK;
  }
};
```

### Controller 生命周期

ros2_control 的 Controller 遵循 ROS 2 的 Lifecycle Node 状态机:

```
                ┌──────────────┐
                │ Unconfigured │
                └──────┬───────┘
                       │ on_configure()  <- allocate memory, create publishers
                ┌──────▼───────┐
                │   Inactive   │
                └──────┬───────┘
                       │ on_activate()   <- start receiving state_interfaces
                ┌──────▼───────┐
                │    Active    │ <- update() called here by RT loop!
                └──────┬───────┘
                       │ on_deactivate()
                ┌──────▼───────┐
                │   Inactive   │
                └──────────────┘
```

**关键设计原则**: 所有**内存分配**必须在 `on_configure()` 中完成(非实时上下文)。`update()` 在实时循环中被调用,**禁止任何分配**。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:在 `update()` 中创建 ROS 消息**
>
> **错误做法**:
> ```cpp
> controller_interface::return_type update(...) override {
>   auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
>   // make_shared -> heap allocation in RT loop!
>   publisher_->publish(*msg);
>   // publish() -> DDS serialization -> more allocations!
> }
> ```
>
> **正确做法**: 使用 `RealtimePublisher` 的 `try_publish` 方法。消息对象在栈上创建(小消息)或预分配为成员变量(大消息)。

> 💡 **概念误区:认为 `ros2_control` 的实时循环自动是实时的**
>
> **新手想法**: "我的 Controller 在 ros2_control 的实时循环里运行,所以它自动是实时安全的!"
>
> **实际上**: ros2_control 的 Controller Manager 确实在 SCHED_FIFO 线程中运行 `update()`,但**你的代码在 update() 中做什么完全取决于你**。如果你在 `update()` 中调用 `malloc`、`std::cout` 或等待 mutex,实时性仍然会被破坏。ros2_control 提供的是实时调度环境,不是实时保证——保证需要你自己来实现。

> ⚠️ **编程陷阱:硬件接口中的阻塞通信**
>
> **错误做法**: 在 `HardwareInterface::read()` 中用阻塞式 socket 读取 EtherCAT 数据。
>
> **现象**: 当网络延迟波动时,整个控制循环被卡住。
>
> **根本原因**: `read()` 在 Controller Manager 的实时循环中被调用。任何阻塞操作都会延迟所有 Controller 的 `update()`。
>
> **正确做法**: 使用非阻塞 I/O 或 DMA。ethercat_driver_ros2(ICube-Robotics 开发,基于 IgH EtherCAT Master)提供了非阻塞的 EtherCAT 通信实现,可作为参考。

### 练习

**练习 61.6a** ⭐⭐: 写一个最简的 `ros2_control` Controller 插件:订阅 `cmd_vel`,在 `update()` 中做 PD 控制,通过 `RealtimePublisher` 发布 debug 信息。用 `ros2_control_demos` 的 example hardware interface 测试。确认 `update()` 中没有堆分配。

**练习 61.6b** ⭐⭐: 阅读 `controller_manager/src/controller_manager.cpp` 的主循环代码。回答:①循环用什么定时机制?②如果某个 Controller 的 `update()` 超时怎么处理?③如何配置循环频率?

---

代码写好之后,如何**验证**它确实满足实时要求?不能只靠"跑几分钟没摔倒"来判断——你需要系统性的测量和诊断工具。

## 61.7 实时性测量与调试 ⭐⭐

### 动机:不可测量就不可保证

实时系统的一个基本原则:**如果你无法测量最坏情况延迟,你就无法保证实时性**。在实验室里"跑了 10 分钟没问题"不代表产品级可靠——你需要在满负载、长时间运行下量化延迟分布。

### `cyclictest`——调度延迟的黄金标准

`cyclictest` 是 Real-Time Linux 社区开发的标准延迟测量工具。它创建高优先级线程,用 `clock_nanosleep` 精确定时,测量实际唤醒时间与期望唤醒时间的差异:

```bash
# Basic latency test (5 minutes, 4 threads, priority 99)
sudo cyclictest -p 99 -t 4 -n -m -D 300

# With histogram output (for plotting latency distribution)
sudo cyclictest -p 99 -t 4 -n -m -D 300 -h 1000 > latency_hist.txt

# Parameters explained:
# -p 99:   SCHED_FIFO priority 99
# -t 4:    4 measurement threads
# -n:      use clock_nanosleep (most accurate)
# -m:      mlockall (lock all memory)
# -D 300:  duration 300 seconds
# -h 1000: histogram up to 1000 us
```

**解读输出**:

```
# /dev/cpu_dma_latency set to 0us
T: 0 ( 1234) P:99 I:1000 C: 300000 Min:3 Act:12 Avg:8 Max:67
T: 1 ( 1235) P:99 I:1500 C: 200000 Min:4 Act:11 Avg:9 Max:72
```

| 字段 | 含义 | 腿足要求 |
|------|------|---------|
| Min | 最小延迟 | 参考值,通常 2-5 $\mu s$ |
| Avg | 平均延迟 | < 20 $\mu s$ |
| Max | **最大延迟** | **< 100-200 $\mu s$**(关键指标!) |

**不同硬件平台的典型结果**(PREEMPT_RT 内核,满负载下):

| 硬件 | CPU | Max Latency | 适合腿足? |
|------|-----|------------|----------|
| Intel NUC i7 (12th gen) | x86_64 | 40-80 $\mu s$ | ✅ 推荐 |
| Jetson Orin NX | ARM Cortex-A78AE | 80-150 $\mu s$ | ✅ 可用 |
| Raspberry Pi 4 | ARM Cortex-A72 | 100-300 $\mu s$ | ⚠️ 勉强 |
| 普通笔记本(标准内核) | 各种 | 10,000-50,000 $\mu s$ | ❌ 不可用 |

### `ftrace` 和 `trace-cmd`——内核级延迟追踪

当 `cyclictest` 报告了一个异常的 max latency,你需要找到**具体是什么导致了这个延迟**。`ftrace`(Function Tracer)是 Linux 内核内置的追踪框架:

```bash
# Method 1: Using trace-cmd (recommended, more user-friendly)
# Record function tracing with wake-up latency analysis
sudo trace-cmd record -p function_graph -P $(pidof my_controller) sleep 10
sudo trace-cmd report > trace_report.txt

# Method 2: Using raw ftrace
# Enable the wakeup latency tracer
echo wakeup_rt > /sys/kernel/debug/tracing/current_tracer
echo 1 > /sys/kernel/debug/tracing/tracing_on
sleep 10
echo 0 > /sys/kernel/debug/tracing/tracing_on
cat /sys/kernel/debug/tracing/trace > wakeup_trace.txt
```

**`rtla`——更现代的 RT 分析工具**:

`rtla`(Real-Time Linux Analysis)是 2022 年引入 Linux 内核源码树的工具,比 `cyclictest` 更强大:

```bash
# Timer latency analysis (similar to cyclictest but with more detail)
sudo rtla timerlat -P f:99 -c 2 -d 30s

# OS noise measurement (identify all interrupt sources)
sudo rtla osnoise -P f:90 -c 2,3 -d 60s -T 1000
# This shows every interrupt, NMI, and softirq that hit your isolated CPUs
```

### 应用层延迟测量

除了系统级工具,你还需要在**应用代码内部**测量控制循环的实际耗时:

```cpp
#include <chrono>
#include <algorithm>
#include <numeric>
#include <array>
#include <cstdio>

class LoopTimer {
public:
  void tick() {
    auto now = std::chrono::steady_clock::now();
    if (initialized_) {
      auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                    now - last_time_).count();
      if (count_ < MAX_SAMPLES) {
        samples_[count_++] = dt;
        max_dt_ = std::max(max_dt_, dt);
        min_dt_ = std::min(min_dt_, dt);
      }
    }
    last_time_ = now;
    initialized_ = true;
  }

  void report() const {
    // Called from non-RT thread only!
    double avg = std::accumulate(samples_.begin(),
                     samples_.begin() + count_, 0.0) / count_;
    // Sort a copy for percentiles
    auto sorted = std::vector<long>(samples_.begin(),
                      samples_.begin() + count_);
    std::sort(sorted.begin(), sorted.end());
    long p99 = sorted[static_cast<size_t>(count_ * 0.99)];
    long p999 = sorted[static_cast<size_t>(count_ * 0.999)];

    printf("Loop timing: min=%ld avg=%.1f p99=%ld p999=%ld max=%ld us\n",
           min_dt_, avg, p99, p999, max_dt_);
  }

private:
  static constexpr size_t MAX_SAMPLES = 1000000;
  std::array<long, MAX_SAMPLES> samples_;  // Pre-allocated, no malloc!
  size_t count_ = 0;
  long max_dt_ = 0, min_dt_ = 999999;
  std::chrono::steady_clock::time_point last_time_;
  bool initialized_ = false;
};
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:用空闲系统测 cyclictest 就满足了**
>
> **错误做法**: 在空闲系统上跑 `cyclictest` 1 分钟,看到 Max = 20 $\mu s$,认为"OK了"。
>
> **现象**: 实际部署后,在 ROS 2 节点、Gazebo、rviz2 同时运行时 Max latency 飙到 500+ $\mu s$。
>
> **根本原因**: 空闲系统不会触发内核的长延迟路径(如 NUMA 迁移、RCU 回调、网络中断处理)。只有在满负载下测量才有意义。
>
> **正确做法**: 在 `cyclictest` 运行的同时,用 `stress -c $(nproc) --io 4 --vm 2` 制造 CPU、I/O 和内存压力。测试时间至少 30 分钟,生产级验证需要 24 小时。

> 💡 **概念误区:只看 p99 不看 Max**
>
> **新手想法**: "p99 = 30 $\mu s$,虽然 Max = 500 $\mu s$,但 99% 的时候都好,足够了。"
>
> **实际上**: 1 kHz 循环运行 1 小时 = 3,600,000 次。p99 外的 1% = 36,000 次超过 30 $\mu s$。而 Max = 500 $\mu s$ 意味着至少有一次接近 deadline 的一半。在硬实时系统中,**一次超时就是失败**。必须确保 Max 小于 deadline 的 50%。

### 练习

**练习 61.7a** ⭐⭐: 在你的系统上运行 `cyclictest` 30 分钟(同时施加负载),导出延迟直方图。用 Python 画出延迟分布图(x 轴:延迟 $\mu s$,y 轴:频次,对数刻度)。标注 p50, p99, p999, Max。

**练习 61.7b** ⭐⭐: 用 `ftrace` 的 `function_graph` tracer 追踪你的控制循环一次执行。找出耗时最长的 3 个函数调用。是否有意外的内核调用?

---

前面的内容建立了通用的实时编程技术。现在让我们把这些技术应用到一个具体的、工程意义重大的场景——Ch55 中 OCS2 的双线程 MPC 架构的实时实现。

## 61.8 OCS2 双线程 MPC 的实时工程 ⭐⭐⭐

### 动机:理论架构到生产代码的鸿沟

Ch55 详细介绍了 OCS2 的双线程架构:**MPC Node**(异步求解)和 **MRT Node**(实时查询,Motion Reference Tracking)通过 `BufferedValue` 通信。`BufferedValue` 本质是 Triple Buffer——Ch55.7.2-55.7.7 从"为什么不用 mutex"出发，推导了三槽位原子交换的设计,证明了其读写无阻塞、数据总是最新的性质。那一章聚焦于架构设计和算法层面。本节不重复 Ch55 的内容,而是补充 Ch55 未覆盖的**部署层实时工程**——线程配置、失败恢复、诊断集成。

### MRT 端的实时关键路径

将 61.1-61.7 的所有技术综合应用到 MRT 循环中:

```cpp
// MRT loop: the realtime-critical path
// Combines: SCHED_FIFO (61.2) + mlockall (61.3) + no-malloc (61.4)
//         + Triple Buffer read (61.5/Ch55.7) + clock_nanosleep (61.3)
void mrtLoop() {
  // --- Setup (non-RT context, before loop starts) ---
  setupRealtimeThread(SCHED_FIFO, 85, /*cpu=*/2);  // 61.2 + 61.3
  mlockall(MCL_CURRENT | MCL_FUTURE);               // 61.3
  prefaultStack();                                   // 61.3

  struct timespec next_wakeup;
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
  LoopTimer timer;  // 61.7

  // ⚠️ 预分配 WBC 输出 buffer（循环外，非 RT 上下文）
  // 使用固定尺寸类型避免 VectorXd 的动态堆分配
  Eigen::Matrix<double, 12, 1> tau;  // 栈上固定大小，零 malloc

  while (running_) {
    timer.tick();

    // 1. Check for new MPC solution (non-blocking atomic read)
    //    Uses BufferedValue from Ch55.7 (atomic swap, ~5 ns)
    if (mpc_mrt_interface_.hasNewPolicy()) {
      mpc_mrt_interface_.updatePolicy();
    }

    // 2. Evaluate policy at current time + run WBC
    Eigen::internal::set_is_malloc_allowed(false);  // 61.4

    auto current_state = stateEstimator_.getState();
    auto [x_ref, u_ref] = mpc_mrt_interface_.evaluatePolicy(
        current_time_, current_state);

    // wbc_.compute() 必须返回固定尺寸类型或写入预分配 buffer
    tau = wbc_.compute(
        current_state.q, current_state.v, x_ref, u_ref);

    hardware_.writeJointCommands(tau);

    Eigen::internal::set_is_malloc_allowed(true);

    // 3. Advance to next period (absolute time, no drift)
    advanceWakeup(next_wakeup, PERIOD_NS);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
  }

  timer.report();  // Print statistics (non-RT context)
}
```

**MPC 端的注意事项**:

MPC 求解不在实时关键路径上(允许变化的求解时间),但仍需注意:

```cpp
void mpcLoop() {
  // Lower priority, non-isolated CPU (ok to share with other tasks)
  setupRealtimeThread(SCHED_FIFO, 70, /*cpu=*/1);

  while (running_) {
    auto state = stateEstimator_.getState();  // Non-blocking read
    auto solution = mpc_solver_.solve(state, current_time_);  // 10-50 ms
    mpc_mrt_interface_.setPolicy(solution);   // BufferedValue swap, ~5 ns

    // No strict timing - solve as fast as possible
  }
}
```

### 失败恢复策略

当 MPC 求解超时或失败时,MRT 需要有 fallback。这在 Ch55 中以架构形式提到,这里给出具体的实时安全实现:

```cpp
void mrtUpdateWithFallback() {
  auto policy_age = current_time_ - last_policy_time_;

  if (policy_age < MAX_POLICY_AGE) {
    // Normal operation: use MPC solution
    tau = wbc_.compute(state, mpc_ref);
  } else if (policy_age < EMERGENCY_TIMEOUT) {
    // MPC stale: extrapolate last solution + increase damping
    tau = wbc_.computeWithExtrapolation(state, last_mpc_ref_, policy_age);
    // Push warning to lock-free log queue (non-blocking)
    log_queue_.tryPush("WARN: MPC policy stale");
  } else {
    // Critical: transition to pure damping mode
    tau = dampingController_.compute(state);
    log_queue_.tryPush("CRIT: emergency damping mode");
  }
}
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:MPC 和 MRT 使用同一个 Pinocchio `Data` 对象**
>
> **错误做法**: MPC 和 WBC 共享 `pinocchio::Data data_;`。
>
> **现象**: 间歇性的数值错误——WBC 计算的扭矩偶尔出现异常跳变。
>
> **根本原因**: `pinocchio::Data` 包含大量中间缓存(如 `data_.J`, `data_.M`)。MPC 在后台线程更新这些缓存的同时,WBC 正在读取——数据撕裂(data race)。即使单个元素的读写是原子的,矩阵整体不是。
>
> **正确做法**: MPC 和 WBC 各自维护独立的 `pinocchio::Data` 对象。它们可以共享同一个 `pinocchio::Model`(只读,线程安全),但 `Data` 必须独立。

> 🧠 **思维陷阱:认为"MPC 求解越快越好"**
>
> **新手想法**: "MPC 应该尽可能快,这样 WBC 总是用最新的解。"
>
> **部分正确但有陷阱**: MPC 求解频率太高(如 200 Hz)可能导致:①求解时间不够导致 SQP 不收敛 ②过度消耗 CPU,干扰其他线程 ③解的质量反而下降(SQP-RTI 只做一次迭代时需要足够的 warm-start 质量)。
>
> **正确理解**: MPC 频率应该匹配预测窗口长度和系统动态。典型的四足 trot:50 Hz MPC(20 ms 周期,预测 1 s)+ 1 kHz WBC 就足够了。

### 练习

**练习 61.8a** ⭐⭐⭐: 基于 OCS2 的 `legged_robot` 示例,在仿真中部署双线程 MPC+WBC。用 `htop -t` 观察线程分布。用本节的 `LoopTimer` 测量 MRT 循环的 jitter。验证:①MRT 的 max jitter < 200 $\mu s$ ②MPC 求解时间分布 ③BufferedValue 的 swap 频率。

**练习 61.8b** ⭐⭐⭐: 在 MPC 线程中故意注入一个随机延迟(模拟偶尔的长求解时间):每 100 次有 1 次延迟 200 ms。观察 WBC 如何处理 MPC 解的"过期"。实现一个 fallback 策略:如果 MPC 解龄 > 100 ms,切换到纯 PD 站立控制。

---

掌握了所有实时编程技术之后,最后需要一个"错误库"——记录常见的实时违规及其表现,帮助你在调试时快速定位问题。

## 61.9 常见实时违规案例库 ⭐⭐

### 动机:从别人的错误中学习

实时 bug 的特点是**难以复现、难以诊断**——它们往往在特定负载、特定时序下才出现。以下案例库整理了腿足机器人开发中最常见的实时违规,每个案例都包含:症状、根因、诊断方法、修复方案。

### 案例 1:隐藏的 `std::string` 堆分配

```cpp
// ❌ Hidden allocation in what looks like a simple log
void WBC::compute(...) {
  if (tau.norm() > torque_limit_) {
    // This string concatenation allocates heap memory!
    std::string msg = "Torque limit exceeded on joint " +
                      std::to_string(worst_joint_) +
                      ": " + std::to_string(tau[worst_joint_]);
    logger_.warn(msg);  // Also might block (I/O)
  }
}
```

```cpp
// ✅ Fix: use pre-formatted fixed buffer + async logging
void WBC::compute(...) {
  if (tau.norm() > torque_limit_) {
    // Write to pre-allocated char buffer (no heap allocation)
    snprintf(log_buf_, sizeof(log_buf_),
             "Torque limit: joint %d = %.3f", worst_joint_,
             tau[worst_joint_]);
    // Push to lock-free queue (non-blocking)
    log_queue_.tryPush(log_buf_);
  }
}
```

### 案例 2:`std::function` 的类型擦除分配

```cpp
// ❌ std::function may heap-allocate for large captures
class Controller {
  std::function<Eigen::VectorXd(const State&)> policy_;

  void setPolicy(const Eigen::MatrixXd& large_matrix) {
    // If the lambda captures more than ~32 bytes (implementation-defined),
    // std::function allocates on the heap to store the capture
    policy_ = [this, large_matrix](const State& s) {
      return large_matrix * s.q;  // large capture -> heap allocation
    };
  }
};
```

```cpp
// ✅ Fix: store the captured data as a member variable instead
class Controller {
  Eigen::MatrixXd policy_matrix_;  // Pre-allocated member

  void setPolicy(const Eigen::MatrixXd& m) {
    policy_matrix_ = m;  // Copy in non-RT context
  }

  Eigen::VectorXd evaluatePolicy(const State& s) {
    return policy_matrix_ * s.q;  // No type erasure, no allocation
  }
};
```

### 案例 3:Eigen 的隐式临时变量

```cpp
// ❌ Temporary matrix created by expression evaluation
Eigen::MatrixXd result = A * B + C;
// Eigen may evaluate A*B into a temporary, then add C
// The temporary -> heap allocation if dynamic size!

// ✅ Fix: use .noalias() and pre-allocated result
result.noalias() = A * B;  // Direct write to result, no temp
result += C;               // In-place addition, no temp
```

### 案例 4:ROS 2 参数回调中的锁

```cpp
// ❌ Parameter callback holds a mutex, blocking RT thread
rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter>& params) {
  std::lock_guard<std::mutex> lock(param_mutex_);  // RT thread waits here!
  for (const auto& p : params) {
    if (p.get_name() == "kp") kp_ = p.as_double();
  }
  return result;
}
```

```cpp
// ✅ Fix: use atomic for parameter updates
std::atomic<double> kp_{10.0};
// Parameter callback (non-RT)
kp_.store(new_kp, std::memory_order_release);
// RT thread
double kp = kp_.load(std::memory_order_acquire);
```

### 案例 5:Linux 内核的透明大页(THP)

```
症状:   cyclictest Max latency 偶尔突增到 2-5 ms, 但 99.99% 正常
诊断:   perf top 显示 khugepaged 内核线程消耗 CPU
根因:   Linux THP (Transparent Huge Pages) 后台线程合并 4KB 页为 2MB 大页,
        过程中需要迁移页面并刷新 TLB, 可能中断实时线程
修复:   echo never > /sys/kernel/mm/transparent_hugepages/enabled
```

### 案例 6:`std::chrono::system_clock` vs `steady_clock`

```cpp
// ❌ system_clock can jump (NTP adjustment)
auto t1 = std::chrono::system_clock::now();
doWork();
auto t2 = std::chrono::system_clock::now();
auto dt = t2 - t1;  // May be NEGATIVE if NTP adjusts the clock!

// ✅ steady_clock is monotonic, cannot jump
auto t1 = std::chrono::steady_clock::now();
doWork();
auto t2 = std::chrono::steady_clock::now();
auto dt = t2 - t1;  // Always >= 0
```

### 快速检查清单

在部署实时代码前,对照此清单逐条检查:

| # | 检查项 | 方法 |
|---|--------|------|
| 1 | 内核: PREEMPT_RT 已启用 | `cat /sys/kernel/realtime` 输出 1 |
| 2 | cyclictest Max < 100 $\mu s$(满负载) | `cyclictest -p99 -D300` |
| 3 | CPU governor: performance | `cpupower frequency-info` |
| 4 | 实时核心已隔离 | `cat /sys/devices/system/cpu/isolated` |
| 5 | THP 已禁用 | `cat /sys/.../transparent_hugepages/enabled` |
| 6 | 控制线程: SCHED_FIFO, priority 50-90 | `chrt -p <PID>` |
| 7 | mlockall 已调用 | 检查 `VmLck` in `/proc/<PID>/status` |
| 8 | EIGEN_RUNTIME_NO_MALLOC(Debug) | 编译 Debug + 运行测试 |
| 9 | 无 `std::cout/printf` 在 RT 路径 | `grep` 代码审查 |
| 10 | 无 `std::mutex` 在 RT 路径 | `grep` + ThreadSanitizer |
| 11 | 无 `new/delete/malloc` 在 RT 路径 | `EIGEN_RUNTIME_NO_MALLOC` + valgrind |
| 12 | 使用 `clock_nanosleep` 绝对定时 | 代码审查 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:在 Debug 中开了 `EIGEN_RUNTIME_NO_MALLOC`,Release 中关了**
>
> **新手想法**: "Debug 测过没问题,Release 也没问题。"
>
> **实际上**: Release 优化可能改变代码路径——编译器可能内联/消除某些函数调用,导致 Debug 中不触发的分配在 Release 中触发(或反过来)。建议在 CI 中对所有 build type 都开启该宏进行测试。

> 🧠 **思维陷阱:认为"jitter 小就行,偶尔一个大延迟没关系"**
>
> **实际上**: 对硬实时系统,max latency 就是一切。1 小时内的 1 次 2 ms 延迟就可能导致摔倒。要么保证 max < deadline,要么接受系统会偶尔失败。**没有"几乎实时"这回事。**

### 练习

**练习 61.9a** ⭐⭐: 写一段故意包含 5 种实时违规的控制循环(malloc、printf、mutex、system_clock、THP 不禁用)。用 `cyclictest`(或应用层 LoopTimer)量化每种违规对 max latency 的影响。按影响大小排序。

**练习 61.9b** ⭐⭐: 对你的某个现有 ROS 2 Controller 代码进行实时审计:用 `grep` 搜索所有可能的违规点(malloc/new/cout/mutex/string/vector/function/shared_ptr),列出发现,并提出修复方案。

---

## 常见故障与排查

实时系统的故障往往表现为"偶尔发生"——平时正常、压力下崩溃，极难复现。以下是腿足实时 C++ 工程中最典型的故障场景。

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 控制循环偶尔超时（max latency > 1 ms 但 avg < 0.1 ms） | 页面缺失（page fault）——首次访问未预触碰的栈页或 THP 合并 | 1. `perf stat -e page-faults` 统计缺页次数 2. 确认 `mlockall(MCL_CURRENT \| MCL_FUTURE)` 已调用 3. 确认栈预触碰函数在循环前执行 4. 禁用 THP | 61.3 |
| `EIGEN_RUNTIME_NO_MALLOC` 断言失败但找不到哪行代码分配了 | Eigen 动态矩阵的隐式 resize——赋值 `A = B` 时 A 大小与 B 不同触发 realloc | 1. 在 assert 处设断点，查看调用栈 2. 检查是否有 `MatrixXd` 在循环内首次赋值 3. 将所有动态矩阵在循环外预分配为正确大小 | 61.4 |
| 线程优先级反转——高优先级 WBC 线程被低优先级日志线程阻塞 | 使用了 `std::mutex` 而非 `pthread_mutex` 且未启用优先级继承协议 | 1. `chrt -p <pid>` 检查各线程调度策略和优先级 2. 改用 SPSC 无锁队列替代 mutex 保护的共享缓冲 3. 如必须用锁,启用 `PTHREAD_PRIO_INHERIT` | 61.2, 61.5 |
| MPC 解到达 WBC 延迟 > 2 个控制周期 | MPC 线程与 WBC 线程共享同一 CPU 核心导致调度竞争 | 1. 检查 CPU 亲和性配置 2. 用 `isolcpus` 隔离 WBC 核心 3. 确认 MPC→WBC 使用 Triple Buffer 而非队列 | 61.3, 61.5 |
| `cyclictest` 最大延迟 > 500 $\mu$s | 内核未启用 PREEMPT_RT、或存在不可抢占的内核路径（如 GPU 驱动） | 1. `uname -a` 确认内核含 `PREEMPT_RT` 标识 2. 用 `trace-cmd` 抓取高延迟事件 3. 禁用不需要的内核模块（如 GPU 显示驱动） | 61.2, 61.9 |

---

## 61.10 本章小结

### 知识点总结

| 知识点 | 核心要点 | 难度 |
|--------|---------|------|
| 硬实时 vs 软实时 | 硬实时 = 一次超时就是系统失败;WBC 是硬实时,MPC 是准实时 | ⭐ |
| PREEMPT_RT | Linux 6.12+ 主线支持;将 spinlock 替换为 rt_mutex + 中断线程化 | ⭐⭐ |
| SCHED_FIFO/DEADLINE | FIFO 用优先级调度,DEADLINE 用 EDF;FIFO 更成熟,DEADLINE 更先进 | ⭐⭐ |
| 优先级反转/继承 | 低优先级持锁阻塞高优先级;PI 让低优先级临时升级 | ⭐⭐ |
| mlockall + 栈预触碰 | 防止 page fault;mlockall 锁页,预触碰强制物理分配 | ⭐⭐ |
| CPU 隔离 | isolcpus + nohz_full + rcu_nocbs 三件套 | ⭐⭐ |
| clock_nanosleep | 绝对时间定时,防止周期漂移 | ⭐⭐ |
| EIGEN_RUNTIME_NO_MALLOC | 运行时检测 Eigen 堆分配;开发期安全网(Ch53.6 建立原理,本章 61.4 补充 CI 集成) | ⭐⭐ |
| 预分配模式 | 固定大小矩阵 / 预分配动态矩阵 / 对象池 / pmr | ⭐⭐ |
| SPSC 无锁队列 | 单生产者单消费者,release/acquire 语义,cache line 对齐 | ⭐⭐⭐ |
| Triple Buffer | 读最新值,写不阻塞;MPC→WBC 的标准通信方式(Ch55.7 架构设计,本章 61.5 部署实现) | ⭐⭐⭐ |
| Seqlock | 读多写少,读者可能重试;适合状态估计数据 | ⭐⭐⭐ |
| ros2_control | Controller + HardwareInterface + ResourceManager;update() 在 RT 循环 | ⭐⭐ |
| RealtimePublisher | 非阻塞发布 ROS 消息;try_publish API(Jazzy+) | ⭐⭐ |
| cyclictest | 测量调度延迟;Max < 100 $\mu s$ 是腿足的最低要求 | ⭐⭐ |
| OCS2 双线程部署 | MPC(异步) + MRT(实时) + BufferedValue + 失败恢复 | ⭐⭐⭐ |

### 本章的设计哲学

实时 C++ 工程的核心哲学可以归结为一句话:**在正确的时间做正确的事情——不多不少,不早不晚。**

具体来说:
- **不多**:不分配不需要的内存,不获取不需要的锁,不做不需要的 I/O
- **不少**:在启动时预分配所有需要的资源,在所有路径上保证确定性行为
- **不早不晚**:`clock_nanosleep` 精确到微秒地唤醒,Triple Buffer 保证读到的总是最新可用数据

本章的内容与 Ch53(WBC 的 `EIGEN_RUNTIME_NO_MALLOC` 和 MPC+WBC 集成架构)和 Ch55(OCS2 的双线程 Triple Buffer 架构)形成三角关系:Ch53 和 Ch55 从**算法和架构**角度提出了实时需求,本章从**系统和工程**角度给出了完整的实现方案。三章合起来覆盖了腿足实时控制的完整知识链。

## 累积项目:本章新增模块

**四足站立控制器进度更新**:

```
Ch47: 加载 URDF + Pinocchio 建模           [完成]
Ch49: 正/逆运动学 + 接触 Jacobian           [完成]
Ch50: QP 求解器集成                         [完成]
Ch52: 摩擦锥约束                            [完成]
Ch53: WBC 实现 (加权 QP)                    [完成]
Ch55: OCS2 MPC 集成                         [完成]
Ch61: 实时部署框架                          <-- 本章新增
  |-- RT 线程模板 (SCHED_FIFO + mlockall + clock_nanosleep)
  |-- SPSC 无锁日志队列
  |-- LoopTimer 诊断工具
  |-- ros2_control Controller 插件封装
  |-- 部署检查清单 + cyclictest 验证脚本
```

**本章新增的具体任务**:
1. 将 WBC 封装为 `ros2_control` 的 `ControllerInterface` 插件
2. 实现 MPC→WBC 的通信(使用 OCS2 的 `MPC_MRT_Interface`,基于 Ch55)
3. 编写 SCHED_FIFO + mlockall + clock_nanosleep 的实时线程模板
4. 用 cyclictest 验证部署环境的延迟 < 100 $\mu s$
5. 确保 `EIGEN_RUNTIME_NO_MALLOC` 测试在 CI 中通过

## 延伸阅读

**内核与实时基础** ⭐:
- Linux Foundation Real-Time Wiki (`wiki.linuxfoundation.org/realtime/start`) — PREEMPT_RT 官方文档,包含版本对照表和入门指南
- Thomas Gleixner, "The Rise of Realtime Linux" (LPC 2024 Talk) — PREEMPT_RT 合入主线的里程碑演讲

**实时编程** ⭐⭐:
- stulp/eigenrealtime (GitHub) — Eigen 实时编程的完整教程和示例,包含 `EIGEN_RUNTIME_NO_MALLOC` 最佳实践
- John Googley, "A Checklist for Writing Linux Real-Time Applications" (LWN.net, 2021) — 实时 Linux 开发者清单

**无锁编程** ⭐⭐⭐:
- rigtorp/SPSCQueue (GitHub) — 工业级 SPSC 无锁队列实现,附详细性能分析
- Jeff Preshing, "An Introduction to Lock-Free Programming" (preshing.com, 2012) — 无锁编程的经典入门教程
- Anthony Williams, *C++ Concurrency in Action* (2nd ed., Manning, 2019) — Ch5-7 深入讲解 `memory_order` 和无锁数据结构

**ros2_control** ⭐⭐:
- ros2_control 官方文档 Jazzy (`control.ros.org/jazzy/`) — 包含 realtime_tools API 参考和迁移指南
- realtime_tools API Rolling (`control.ros.org/master/doc/realtime_tools/doc/index.html`) — 最新 API,包含 `RealtimeThreadSafeBox`

**EtherCAT** ⭐⭐⭐:
- ethercat_driver_ros2 (`github.com/ICube-Robotics/ethercat_driver_ros2`) — ros2_control 的 EtherCAT 硬件接口框架,基于 IgH EtherCAT Master

**学术论文** ⭐⭐⭐:
- Farshidian et al., "An Efficient Optimal Planning and Control Framework For Quadrupedal Locomotion" (ICRA 2017) — OCS2 双线程架构的原始论文
- de Wit et al., "A Preliminary Assessment of the Real-Time Capabilities of ROS 2" (OSPERT, 2024) — ROS 2 实时性的系统评估
- Grandia et al., "Perceptive Locomotion Through NMPC" (T-RO, 2023) — OCS2 生产版本,100 Hz NMPC + 实时感知
