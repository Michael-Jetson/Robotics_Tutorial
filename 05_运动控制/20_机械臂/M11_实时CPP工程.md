> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M11 实时 C++ 工程——PREEMPT_RT + 无堆分配 + realtime_tools

> **本章定位**：规划算法输出的是关节位置/速度序列，要**真正让机械臂动起来**还需要确保控制循环满足 1 kHz 硬实时要求。SLAM 工程师写惯了"能跑就行"的 C++——`std::vector` 随意 push_back、`std::string` 拼接日志、`new` 对象不释放靠智能指针——这些在 10 Hz 的 SLAM 前端完全没问题，但在 1 kHz 的机械臂伺服循环中，任何一次意外的 `malloc` 都可能导致 50-500 us 的延迟尖峰，让控制器错过 deadline，机器人失控。本章从"为什么 SLAM 的 C++ 不够"出发，系统讲解 RT 安全编程的完整禁区清单、Linux PREEMPT_RT 主线化里程碑、realtime_tools 无锁组件、libfranka 实时接口精读，以及与足式机器人实时工程的对比。
>
> **共享属性**：✅ **规控方向共享**——机械臂 1 kHz 伺服、四足 500 Hz-1 kHz WBC、人形 WBC 全部需要实时 C++ 工程能力。
>
> **前置依赖**：`02_C++基础与进阶/20_并发与系统编程/10_线程管理与互斥同步` + `20_原子操作与内存模型`（并发基础——std::thread / std::mutex / std::atomic）、`02_C++基础与进阶/20_并发与系统编程/50_内存分配策略与pmr`（pmr 内存资源）、`02_C++基础与进阶/10_C++语言核心/40_RAII与智能指针`（RAII）
>
> **下游章节**：M12（ros2_control 硬件接口）、M14（MoveIt2 系统集成）
>
> **建议用时**：1.5 周（理论 3 天 + 实验 4 天 + 源码精读 3 天）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **线程同步**：`std::mutex` 的 `lock()` 和 `unlock()` 如何保护共享数据？如果线程 A 持有 mutex 而线程 B 试图获取，B 会怎样？ | `02_C++基础与进阶/20_并发/10_线程管理` |
| 2 | **原子操作**：`std::atomic<int>` 的 `store` 和 `load` 操作为什么不需要 mutex？`memory_order_relaxed` 和 `memory_order_seq_cst` 的区别是什么？ | `02_C++基础与进阶/20_并发/20_原子操作` |
| 3 | **RAII**：写出一个 RAII 风格的文件句柄类。为什么 RAII 在实时编程中特别重要？（提示：析构函数中的资源释放是确定性的） | `02_C++基础与进阶/10_C++/40_RAII与智能指针` |
| 4 | **Eigen 内存**：为什么 `Eigen::Matrix3d` 是栈分配的而 `Eigen::MatrixXd` 是堆分配的？`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 解决什么问题？ | `02_C++基础与进阶/40_通用库/20_Eigen深入` |
| 5 | **进程调度**：Linux 的 `SCHED_FIFO` 和 `SCHED_OTHER` 有什么区别？为什么实时任务需要 `SCHED_FIFO`？ | 操作系统基础 |

---

## 本章目标

学完本章后，你应该能够：

1. **解释** PREEMPT_RT 已于 2024 年 11 月进入 Linux 6.12 主线的里程碑意义，理解这对机器人实时编程的影响
2. **背诵** RT 安全 C++ 编程的完整禁区清单——哪些操作在 RT 线程中绝对禁止，哪些是安全的
3. **配置** `SCHED_FIFO` + `mlockall` + `EIGEN_RUNTIME_NO_MALLOC` 的标准三件套
4. **使用** realtime_tools 包的三大核心组件：RealtimeBuffer、RealtimePublisher、LockFreeSPSCQueue（及 LockFreeMPMCQueue）
5. **理解** 优先级反转问题和优先级继承 mutex 的解法
6. **精读** libfranka 的 1 kHz 实时控制接口，理解 callback 风格的 RT 安全设计
7. **对比** 机械臂与足式机器人在实时工程上的异同

### 本章知识导航

本章覆盖实时 C++ 工程的完整知识体系，知识点之间存在明确的递进关系：

```
实时 C++ 工程知识体系
│
├── 基础层：为什么需要实时（§1）
│   ├── 1 ms 预算分析
│   ├── SLAM C++ vs RT C++ 差异
│   └── 实时 vs 低延迟概念辨析
│
├── 内核层：操作系统支持（§2）
│   ├── PREEMPT_RT 主线化历程
│   ├── 内核编译与配置实战
│   ├── cyclictest 验证方法论
│   └── Xenomai 对比与选型决策
│
├── 编程层：RT 安全编码规范（§3-4）
│   ├── 禁止操作清单（§3）——核心必背
│   ├── 允许操作清单（§3）
│   ├── SCHED_FIFO + mlockall + EIGEN_NO_MALLOC 三件套（§4）
│   ├── pmr 内存池在 RT 中的应用（§4）
│   └── CPU 频率锁定与核心隔离（§4）
│
├── 通信层：RT-非RT 线程间数据交换（§5-6）
│   ├── RealtimeBuffer / RealtimeThreadSafeBox（§5）
│   ├── RealtimePublisher（§5）
│   ├── Lock-Free Queue（SPSC / MPMC）（§5）
│   ├── Triple Buffer（§5）
│   └── 优先级反转与解决方案（§6）
│
├── 接口层：工业级 RT API（§7-8）
│   ├── libfranka 1 kHz 回调精读（§7）
│   └── EtherCAT DC 同步与 PDO 映射（§8）
│
├── 框架层：ROS2 实时控制架构（§10-11）
│   ├── ros2_control 主循环精读（§10）
│   ├── ros2_control 线程模型（§11）
│   └── Orocos RTT 替代方案简析（§10）
│
├── 调试层：非侵入式性能分析（§12）
│   ├── ftrace 内核跟踪
│   ├── perf 性能采样
│   └── trace-cmd + KernelShark 可视化
│
└── 对比层：机械臂 vs 足式实时需求（§9）
    ├── 架构差异（同步 vs 异步）
    ├── 安全差异（ISO 10218 vs 无标准）
    └── 调试难度差异
```

**阅读路径建议**：

| 读者类型 | 推荐路径 | 预计时间 |
|---------|---------|---------|
| **SLAM 工程师转控制** | §1 → §3（背诵禁区）→ §4 → §5 → §7 → §12 | 精读 6 小时 |
| **已有 RT 经验** | §2.7（新实践）→ §5.4-5.6（无锁队列）→ §7.2（walkthrough）→ §8.3-8.4（EtherCAT 细节） | 精读 3 小时 |
| **足式转机械臂** | §9 → §7 → §8 → §11 | 精读 2 小时 |
| **速查用途** | 本章小结 + API 速查表 + 故障排查手册 | 30 分钟 |

### 如果跳过本章会怎样

1. **你的 1 kHz 控制循环会偶发超时**：在 SLAM 中养成的 `std::vector` / `std::string` 编程习惯，会在控制循环中埋下隐藏的 malloc 地雷。第一次 push_back 时程序表现正常，但当 capacity 不足触发重分配时，50-500 us 的延迟尖峰让 libfranka 报 `communication_constraints_violation`，机器人停机。你无法定位 bug，因为这是概率性的——只有在内存碎片化严重时才触发。

2. **你不知道如何在 RT 和非 RT 线程间传递数据**：你可能会用 `std::mutex` 保护共享数据——这在 SLAM 中完全正确，但在 RT 上下文中会导致优先级反转。高优先级控制线程被低优先级日志线程"间接阻塞"——1997 年火星探路者号就因此反复重启。

### 预计阅读时间

| 模式 | 内容覆盖 | 时间 |
|------|---------|------|
| **精读** | 全部内容 + 代码实践 + 练习 | 10-12 小时 |
| **速读** | 跳过 ⭐⭐⭐ 和 ⭐⭐⭐⭐ 节，重点读禁区清单、三件套、realtime_tools | 4-5 小时 |
| **速查** | 直接翻本章小结 + API 速查表 + 故障排查手册 | 30 分钟 |

---

### 本章知识导航

```
M11 实时 C++ 工程
│
├── §1 为什么 SLAM 的 C++ 不够用 ⭐
│   ├── 1 ms 铁律与控制循环预算
│   ├── SLAM 代码在 RT 中的地雷
│   └── 实时 vs 低延迟概念辨析
│
├── §2 PREEMPT_RT 主线化 ⭐
│   ├── 历史：20 年补丁进入 Linux 6.12 主线
│   ├── RT 内核编译实战
│   ├── cyclictest 完整验证流程
│   ├── Xenomai 对比
│   └── 实时 Rust 前沿展望 ⭐⭐⭐⭐
│
├── §3 RT 安全 C++ 禁区清单 ⭐
│   ├── 绝对禁止的操作（malloc / throw / cout / mutex）
│   ├── 允许的操作（atomic / fixed Eigen / memcpy）
│   └── 灰色地带评估
│
├── §4 标准三件套 ⭐⭐
│   ├── SCHED_FIFO 实时调度策略
│   ├── mlockall 内存页面锁定
│   ├── EIGEN_RUNTIME_NO_MALLOC 堆分配审计
│   └── CPU 频率锁定与核心隔离
│
├── §5 无锁数据结构 ⭐⭐
│   ├── RealtimeBuffer / RealtimePublisher
│   ├── SPSC Lock-Free Ring Buffer
│   ├── Triple Buffer
│   └── std::atomic 内存序与 ARM 陷阱
│
├── §6 优先级反转与解决方案 ⭐⭐
│
├── §7 libfranka 实时接口精读 ⭐⭐
│   ├── 1 kHz callback 模式
│   ├── 完整 walkthrough（PD + 重力补偿 + 安全检查）
│   └── 计算预算分析
│
├── §8 EtherCAT 实时通信 ⭐⭐⭐
│   ├── DC 同步与 PDO 映射
│   └── 与足式硬件栈对比
│
├── §9 与足式实时工程的对比 ⭐⭐
│   ├── 同步 vs 异步控制架构
│   ├── 安全响应时间差异
│   └── 调试难度差异
│
├── §10 ros2_control 主循环精读 ⭐⭐⭐
│
├── §11 ros2_control 线程模型 ⭐⭐
│   ├── RT / 非 RT 线程分离
│   └── 控制器生命周期与 RT 安全
│
└── §12 实时调试方法论 ⭐⭐⭐
    ├── ftrace / perf / trace-cmd
    └── 实时调试决策树
```

### 预计阅读时间

| 阅读模式 | 时间估算 | 适用场景 |
|---------|---------|---------|
| **精读** | 5-7 小时 | 首次学习，逐段理解 RT 原理和代码实现 |
| **速读** | 2-3 小时 | 已有 Linux 系统编程经验，重点看禁区清单和 realtime_tools |
| **速查** | 30-45 分钟 | 查阅 RT 配置三件套代码模板、cyclictest 合格标准、调试决策树 |

> 精读模式不含实验时间。建议配合 RT 内核环境实际运行 cyclictest（§2）和 EIGEN_RUNTIME_NO_MALLOC 审计（§4）。

---

## 1. 为什么 SLAM 的 C++ 在这里不够用 ⭐

### 1.1 动机：1 ms 的铁律 ⭐

一个典型的机械臂伺服控制循环：

```
┌──────────────────── 1 ms 预算 ────────────────────┐
│                                                    │
│  read()    → 读关节编码器 (~10 us)                  │
│  compute() → 控制算法                               │
│    ├── RNEA 重力补偿 (~2 us, Pinocchio)             │
│    ├── PD 控制律计算 (~1 us)                        │
│    └── 安全检查 (~1 us)                             │
│  write()   → 发送力矩指令 (~10 us)                  │
│                                                    │
│  总计: ~25 us (典型)                                │
│  余量: ~975 us (看起来很充裕)                        │
│                                                    │
│  !! 但是 !!                                        │
│  一次 malloc:      50-500 us ← 吃掉一半余量          │
│  一次 cout:        100-1000 us ← 可能超出 1 ms       │
│  一次 mutex 竞争:  不确定上界 ← 可能无限等待          │
└────────────────────────────────────────────────────┘
```

**关键洞察**：控制算法本身只用 25 us，但**一次意外的系统调用就可能让整个 1 ms 预算超支**。这不是"代码写得不够优化"——是"C++ 的某些基本操作在实时上下文中根本不安全"。

### 1.2 SLAM 工程师最常踩的坑 ⭐

在 SLAM 系统（如 ORB-SLAM3）中，以下代码完全正常：

```cpp
// SLAM 前端: 30 Hz, 每帧 33 ms 预算
void ProcessFrame(const Frame& frame) {
    // ✅ SLAM 中没问题: 33 ms 预算, 偶尔慢点无所谓
    std::vector<cv::KeyPoint> keypoints;  // 堆分配
    detector_->detect(frame.image, keypoints);  // 可能分配更多内存

    std::string log_msg = "Detected " + std::to_string(keypoints.size())
                        + " keypoints in frame " + std::to_string(frame.id);
    spdlog::info(log_msg);  // 日志输出

    std::lock_guard<std::mutex> lock(map_mutex_);  // 加锁写地图
    map_->InsertKeyPoints(keypoints);  // 可能触发 rehash -> malloc
}
```

把这段代码"搬到"1 kHz 控制循环中——每一行都是地雷：

```cpp
// ❌ 控制循环: 1 kHz, 1 ms 预算
void update() {
    std::vector<double> torques;  // ❌ malloc
    torques.reserve(7);           // ❌ 首次仍 malloc

    std::string log = "tau=" + std::to_string(torques[0]);  // ❌ 堆分配
    spdlog::info(log);  // ❌ 锁 + 文件 I/O

    std::lock_guard<std::mutex> lock(cmd_mutex_);  // ❌ 可能等待毫秒级
}
```

> **反事实推理**：如果你在 Franka Panda 的 1 kHz 控制循环中做了一次 `std::vector::push_back()`（触发堆分配），会发生什么？libfranka 的实时通信协议要求每 1 ms 发送一个 UDP 包。如果控制循环耗时超过 1 ms，libfranka 会抛出 `franka::ControlException`（"communication_constraints_violation"），机器人立即停机。一次 malloc 可能需要 200 us（正常）到 500 us（内存碎片化时），虽然不会直接超 1 ms，但如果叠加 OS 调度延迟（非 RT 内核下可达 700+ us），总延迟可能超过 2 ms——控制器连续丢失 2 个通信包，触发安全停机。

### 1.3 实时 vs 低延迟——概念辨析 ⭐

| 概念 | 含义 | SLAM 需要吗？ | 控制需要吗？ |
|------|------|-------------|------------|
| **低延迟** | 平均延迟小 | ✅（快速处理帧） | ✅ |
| **硬实时** | 最坏情况延迟有界 | ❌（偶尔丢帧可接受） | ✅（必须） |
| **确定性** | 每次执行时间相同 | ❌ | ✅（必须） |

> **"不是X而是Y"句式**：实时编程的目标不是"让代码跑得更快"（低延迟），而是"保证代码在最坏情况下也不超时"（确定性）。一个平均 10 us 但偶尔 5 ms 的函数在 SLAM 中完全可用（平均很快），但在 1 kHz 控制中是灾难（最坏超时）。

### ⚠️ 常见陷阱

> 💡 **概念误区**：认为"用 C++ 就是实时的"
>
> **新手想法**："C++ 比 Python 快 100 倍，用 C++ 写控制循环就够了"
>
> **实际上**：C++ 的性能优势是平均性能。但 `new`/`delete`、`std::mutex`、`std::cout` 等操作的最坏情况延迟是不确定的——取决于 OS 调度、内存碎片化、其他线程行为。实时编程需要消除所有不确定性来源，不仅仅是"用快的语言"

> 🧠 **思维陷阱**：认为 SLAM 不需要实时是因为"SLAM 不重要"
>
> **新手想法**："SLAM 不是实时的，所以 SLAM 对延迟不敏感？"
>
> **实际上**：SLAM 不需要硬实时的原因是**丢帧的后果不同**。SLAM 丢一帧→跟踪精度略降（可通过后续帧恢复）。控制丢一帧→机器人 1 ms 没收到力矩指令→力矩保持上一值或归零→轨迹偏差累积→可能失控。后果的严重程度决定了实时需求

### 练习

1. ⭐ 列出你在 SLAM 代码中常用的 5 个 C++ 操作，判断每个在 RT 线程中是否安全。
2. ⭐⭐ 解释为什么 `std::shared_ptr` 在 RT 线程中不安全。（提示：引用计数递减到零时触发析构 → `delete` → `free`）
3. ⭐⭐ 为什么 Python 在任何情况下都不适合写实时控制循环？（提示：GIL、垃圾回收、解释器开销——全是不确定性来源）

---

## 2. PREEMPT_RT 主线化——2024 年 11 月的里程碑 ⭐

### 2.1 历史：20 年的补丁终于进入主线 ⭐

**Linux 6.12 内核（2024 年 11 月 17 日发布）正式合并了 PREEMPT_RT**，结束了长达 20 年的补丁开发历程。这意味着：

- 不再需要自定义编译打补丁的内核
- 主流发行版（Ubuntu 25.04+、Fedora 42+）直接提供 RT 内核选项
- 启用方式：内核配置中开启 `CONFIG_PREEMPT_RT`（可能需先启用 `CONFIG_EXPERT`）

**对机器人工程师的影响**：以前搭建 RT 环境需要下载内核源码、打补丁、编译、处理驱动不兼容——可能花 1-3 天。现在：`apt install linux-image-rt` 或在内核配置中勾选一个选项。实时 Linux 从"专家操作"变成了"普通配置"。

### 2.2 PREEMPT_RT 内核编译实战 ⭐⭐

对于需要定制内核参数的场景（如特殊硬件驱动、嵌入式平台），仍需手动编译 RT 内核。以下是 Ubuntu 24.04 上的完整流程。

**Step 1: 获取内核源码与 RT 补丁**

```bash
# 确定当前内核版本
uname -r
# 例: 6.12.0-generic

# 下载对应版本的内核源码
cd /usr/src
sudo apt install linux-source
# 或直接从 kernel.org 下载
wget https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-6.12.tar.xz
tar xf linux-6.12.tar.xz
cd linux-6.12

# 下载 RT 补丁 (6.12+ 已主线化, 此步骤可选)
# 对于旧版本内核:
# wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.8/patch-6.8-rt8.patch.xz
# xzcat patch-6.8-rt8.patch.xz | patch -p1
```

**Step 2: 配置内核**

```bash
# 以当前内核配置为基础
cp /boot/config-$(uname -r) .config

# 进入菜单配置
make menuconfig
# 导航到:
#   General setup →
#     Preemption Model →
#       选择 "Fully Preemptible Kernel (Real-Time)"
#
# 其他推荐配置:
#   Processor type → Timer frequency → 1000 Hz
#   Power management → CPU idle → 关闭 (避免 C-state 延迟)
#   General setup → CONFIG_EXPERT → 启用
#   General setup → CONFIG_PREEMPT_RT → 启用
```

**Step 3: 编译与安装**

```bash
# 编译 (使用全部 CPU 核心)
make -j$(nproc) deb-pkg LOCALVERSION=-rt

# 安装 (生成 .deb 文件)
sudo dpkg -i ../linux-headers-6.12.0-rt*.deb
sudo dpkg -i ../linux-image-6.12.0-rt*.deb

# 更新 GRUB
sudo update-grub

# 重启并选择 RT 内核
sudo reboot
```

**Step 4: 验证 RT 内核**

```bash
# 确认内核版本
uname -a
# 应包含 "PREEMPT_RT" 字样

# 检查 RT 配置
zcat /proc/config.gz | grep PREEMPT
# CONFIG_PREEMPT_RT=y

# 验证实时调度器可用
chrt -m
# SCHED_FIFO min/max priority : 1/99
```

> ⚠️ **编程陷阱**：编译 RT 内核时忘记关闭 CPU idle states
>
> **错误做法**：保留默认的电源管理配置
>
> **现象**：cyclictest 偶尔出现 100+ us 延迟尖峰
>
> **根本原因**：CPU 进入深度 C-state（C3/C6）后唤醒需要 50-200 us。即使有 RT 调度器保证线程优先执行，CPU 硬件唤醒延迟不可消除
>
> **正确做法**：内核配置关闭深度 C-state，或在 GRUB 中添加 `idle=poll`（高功耗但零唤醒延迟）或 `processor.max_cstate=1`

### 2.3 cyclictest 完整验证流程 ⭐

`cyclictest` 是 RT 延迟测试的标准工具。以下是一套完整的验证流程，不只是"跑一下看数字"。

**基准测试（无压力）**：

```bash
# 安装
sudo apt install rt-tests stress-ng

# 基准: 5 线程, 优先级 80, 1ms 周期, 运行 5 分钟
sudo cyclictest -t5 -p80 -n -i1000 -l300000 -m -a2,3 \
    --histfile=hist_baseline.txt

# -m: mlockall (必须!)
# -a2,3: 绑定到 CPU 2,3 (应预先 isolcpus)
# --histfile: 输出延迟直方图
```

**压力测试（模拟真实负载）**：

```bash
# 终端 1: 启动 cyclictest
sudo cyclictest -t5 -p80 -n -i1000 -l300000 -m -a2,3 \
    --histfile=hist_stress.txt

# 终端 2: 施加多维度压力
stress-ng --cpu 4 --io 4 --vm 2 --vm-bytes 512M \
    --hdd 2 --timer 4 --timeout 300s

# 终端 3: 网络压力 (如果有 EtherCAT 网卡)
sudo ping -f -s 65507 192.168.1.1
```

**合格标准**：

| 测试条件 | 最大延迟阈值 | 判定 |
|---------|------------|------|
| 基准（无压力） | < 30 us | 1 kHz 控制安全 |
| CPU + IO 压力 | < 50 us | 1 kHz 控制安全 |
| 全压力 | < 100 us | 500 Hz 控制安全 |
| 任何条件 | > 500 us | **不合格** |

**直方图分析**：

```python
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('hist_stress.txt', skiprows=1)
latencies = data[:, 1]  # us

plt.figure(figsize=(10, 4))
plt.hist(latencies, bins=100, log=True)
plt.axvline(x=50, color='r', linestyle='--', label='50 us threshold')
plt.xlabel('Latency (us)')
plt.ylabel('Count (log)')
plt.title('cyclictest Latency Distribution under Stress')
plt.legend()
plt.savefig('rt_latency_hist.png')
```

> **跨领域类比**：cyclictest 之于 RT 系统，就像压力测试之于结构工程。你不能只在"好天气"（空载）下测试——必须在"暴风雨"（满载+网络+磁盘 IO）下验证最坏情况。RT 系统的可靠性取决于最坏情况，不是平均情况。

### 2.4 延迟对比数据 ⭐

| 平台 | 平均延迟 | 最大延迟（空载） | 最大延迟（压力下） |
|------|---------|--------------|----------------|
| x86 (Xeon/i7) + PREEMPT_RT | 2-6 us | 10-20 us | **20-50 us** |
| Raspberry Pi 4 + PREEMPT_RT | 10 us | 40-80 us | 100-150 us |
| Jetson Orin + PREEMPT_RT | 5-15 us | 30-60 us | 80-120 us |
| **非 RT 内核（压力下）** | 10-50 us | 700+ us | **不确定上界** |

**关键数字**：1 kHz 控制循环预算 1000 us。非 RT 内核压力下最大延迟 700+ us——光"等内核调度"就吃掉 70% 预算。加上控制计算，很容易超时。

### 2.5 用 cyclictest 建立直觉（快速版） ⭐

`cyclictest` 是 RT 延迟测试的标准工具。教学中**必须**让学员亲自运行（完整验证流程见 2.3）：

```bash
# 安装
sudo apt install rt-tests stress

# 在 RT 内核上运行 (5 线程, 优先级 80, 1ms 周期, 10000 循环)
sudo cyclictest -t5 -p80 -n -i1000 -l10000

# 同时在另一终端施加压力
stress --cpu 4 --io 2 --vm 2 --vm-bytes 256M
```

**预期结果**：

```
RT 内核:     max latency = 20-50 us   ← 1 kHz 循环安全
非 RT 内核:  max latency = 700-2000 us ← 1 kHz 循环可能超时
```

### 2.6 Xenomai vs PREEMPT_RT ⭐⭐

| 维度 | PREEMPT_RT | Xenomai |
|------|-----------|---------|
| 架构 | 单内核（修改 Linux 调度器） | 双内核（Xenomai + Linux） |
| 最坏延迟 | 20-100 us | **5-20 us** |
| 驱动兼容性 | **完全兼容** Linux 驱动 | 需要专用驱动或 RTDM 封装 |
| 工具链 | 标准 GCC/GDB | 需要 Xenomai skin API |
| ROS2 兼容 | **原生支持** | 需额外集成 |
| 主流趋势 | ✅ 主线化、标准化 | 逐步收缩 |

**决策规则**：除非需要 < 20 us 保证延迟（快速 EtherCAT 周期），否则选 PREEMPT_RT。整个 ROS2 生态标准化在 PREEMPT_RT 上。

### 2.7 Linux 6.12+ PREEMPT_RT 主线化后的新实践 ⭐⭐

PREEMPT_RT 进入 Linux 6.12 主线后，实际工程实践发生了几个重要变化，值得关注。

**变化一：发行版原生 RT 内核支持**

Ubuntu 25.04+（Plucky Puffin）、Fedora 42+ 已在官方仓库中提供预编译的 `linux-image-rt` 或等效包。这意味着：

- 安装一行命令即可完成：`sudo apt install linux-image-6.12-rt-generic`（Ubuntu）
- 无需手动下载源码、打补丁、编译——将 RT 环境搭建时间从 1-3 天缩短到 5 分钟
- 主流硬件驱动（包括 Intel/AMD 网卡、USB 串口、NVIDIA GPU 驱动）已在主线内核中适配 PREEMPT_RT 的 threaded IRQ 模型，不再需要额外的 out-of-tree 补丁

**变化二：SCHED_EXT（可扩展调度器）**

Linux 6.12 同时引入了 `SCHED_EXT`（Extensible Scheduler），允许通过 eBPF 程序在用户态定义自定义调度策略。虽然 SCHED_EXT 本身不是为硬实时设计的（它主要面向数据中心和桌面场景的调度优化），但它为 RT 社区提供了一个有趣的实验平台：研究者可以用 eBPF 快速原型化新的 RT-aware 调度策略，而无需修改内核源码。目前这仍处于实验阶段，生产环境仍应使用 `SCHED_FIFO`。

**变化三：printk 线程化完成**

PREEMPT_RT 长期以来的一个痛点是 `printk` 在中断上下文中可能阻塞 RT 任务。6.12+ 内核完成了 printk 的完全线程化——所有 printk 输出都由低优先级内核线程异步处理，不再干扰 RT 调度。这对 ROS2 控制栈的影响是：即使系统中有大量内核日志输出（如驱动警告），也不会导致 RT 循环延迟尖峰。

> **本质洞察**：PREEMPT_RT 主线化不仅是一个技术里程碑，更标志着实时 Linux 从"专家工具"变成了"标准基础设施"。正如 TCP/IP 进入操作系统内核推动了互联网普及，PREEMPT_RT 进入 Linux 主线将推动实时机器人控制系统的标准化——未来"机器人用 Linux"不再需要加"RT 补丁"的定语。

### 2.8 实时 Rust 的前沿展望 ⭐⭐⭐⭐

C++ 是当前 RT 系统编程的主力语言，但它的内存安全问题（use-after-free、data race、buffer overflow）在 RT 上下文中尤其危险——RT 线程通常以高权限运行（`SCHED_FIFO` + root），内存错误可能导致不可预测的硬件行为。

Rust 语言的所有权系统在编译期消除了这类错误，使其成为 RT 系统编程的一个值得关注的候选。目前生态发展包括：

| 项目 | 状态 | 描述 |
|------|------|------|
| `rt-PREEMPT` Rust 绑定 | 实验阶段 | 封装 `sched_setscheduler`、`mlockall` 等 POSIX RT API 的安全抽象 |
| `embedded-hal` / RTIC | 成熟 | 嵌入式实时框架，已在 ARM Cortex-M 上广泛使用 |
| `ros2_rust` | 早期开发 | ROS2 的 Rust 客户端库，支持基本的 pub/sub 和 service |
| Rust for Linux | 合并中 | Linux 内核模块可用 Rust 编写（6.1+ 内核），未来 RT 驱动可能用 Rust 实现 |

**Rust 在 RT 中的核心优势**：

1. **编译期数据竞争消除**：Rust 的 borrow checker 在编译时阻止对共享数据的不安全并发访问。在 C++ 中，RT 线程和非 RT 线程之间的数据竞争是最难调试的 bug 类型之一（症状是偶发的、不可复现的延迟尖峰），Rust 从根源上消除了这个问题
2. **零成本抽象的内存安全**：Rust 的安全保证不引入运行时开销——没有垃圾回收器、没有引用计数（除非显式使用 `Rc`/`Arc`）、没有异常展开（Rust 用 `Result<T, E>` 替代异常）
3. **`no_std` 支持**：Rust 可以在不依赖标准库的环境下运行，适合裸机或受限 RT 环境

**当前局限**：ROS2 生态几乎完全是 C++ 的；Pinocchio、MoveIt2、ros2_control 等关键库没有 Rust 绑定；`unsafe` 的 FFI 调用仍然是 Rust-C++ 互操作的常态。因此目前 Rust 在机器人 RT 系统中仅适合底层驱动和独立安全模块——距离替代 C++ 成为主力语言还有很长的路。

> **反事实推理**：如果 ros2_control 的 Hardware Interface 是用 Rust 写的会怎样？`read()` 和 `write()` 中的共享缓冲区访问会在编译期被 borrow checker 验证安全性——不需要人工审查"RT 线程中是否有 data race"。但代价是：当前所有 C++ 控制器都需要通过 FFI 调用 Rust 驱动，增加了集成复杂度。这也是为什么"在现有 C++ 生态中逐步引入 Rust 安全模块"比"完全用 Rust 重写"更现实。

### ⚠️ 常见陷阱

> 💡 **概念误区**：认为 PREEMPT_RT 能让代码"变快"
>
> **新手想法**："打了 RT 补丁，代码运行速度会提升"
>
> **实际上**：PREEMPT_RT 不改变代码的平均执行速度——它改变的是**最坏情况延迟**。RT 内核通过中断处理线程化、自旋锁可抢占等手段，保证高优先级任务不被低优先级阻塞太久。代码的"快慢"取决于算法本身，RT 内核保证的是"不管其他任务多忙，控制循环都能按时运行"

### 练习

1. ⭐ 在你的机器上分别用 RT 和非 RT 内核运行 `cyclictest`。记录最大延迟。
2. ⭐⭐ 解释为什么非 RT 内核的最大延迟"不确定"。
3. ⭐⭐ 什么场景下应选 Xenomai 而非 PREEMPT_RT？给出至少 2 个具体工业场景。

---

## 3. RT 安全 C++ 编程的禁区清单 ⭐

### 3.1 这是本章最核心的内容 ⭐

**SLAM 工程师转向规控最容易犯的错误就在这里。** 以下清单必须**背诵**——在写 RT 代码时像本能一样避开。

### 3.2 RT 线程中绝对禁止的操作 ⭐

| 操作 | 延迟范围 | 为什么危险 |
|------|---------|-----------|
| `new` / `delete` / `malloc` / `free` | 50-500 us | 内存分配器加锁（glibc malloc 的 arena mutex），可能触发 mmap |
| `throw` / `catch` | 100+ us | 异常涉及堆分配（异常对象）和栈展开 |
| `std::vector::push_back()` | 50-500 us | capacity 不足时触发重分配 |
| `std::string` 拼接 | 50-200 us | 超出 SSO 限制（16-23 字节）就堆分配 |
| `std::map` / `std::set` 插入 | 50-200 us | 红黑树节点堆分配 |
| `std::mutex` 锁 | 不确定 | 其他线程持有锁时当前线程被挂起 |
| `printf` / `std::cout` / `spdlog` | 100-1000 us | 文件 I/O + 内部 mutex + 格式化分配 |
| 文件读写 | 1000+ us | 磁盘 I/O 是最不确定的操作 |
| `sleep(CLOCK_REALTIME)` | 不确定 | REALTIME 时钟受 NTP 调整影响 |
| 线程创建 / 销毁 | 1000+ us | `pthread_create` 涉及内核数据结构分配 |
| `std::shared_ptr` 析构（最后一个） | 触发 `delete` | 引用计数归零时执行析构 + free |

### 3.3 RT 线程中允许的操作 ⭐

| 操作 | 延迟 | 说明 |
|------|------|------|
| `clock_gettime(CLOCK_MONOTONIC)` | < 1 us | 系统时钟读取 |
| `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` | 确定性 | 绝对时间睡眠，不受 NTP 影响 |
| 固定大小 Eigen 算术 | 确定性 | `Matrix3d`、`Vector4d` 全栈分配 |
| `std::atomic` 操作 | < 1 us | 无锁，硬件保证原子性 |
| 无锁队列 push/pop | < 1 us | SPSC/MPMC 无锁队列 |
| 预分配内存的读写 | 确定性 | `std::array`、`std::span`、栈变量 |
| `memcpy` / `memmove` | 确定性 | 固定大小内存拷贝 |

### 3.4 灰色地带——需要仔细评估 ⭐⭐

| 操作 | 是否安全 | 条件 |
|------|---------|------|
| `Eigen::VectorXd` 乘法 | ⚠️ | 编译期维度已知（如 typedef 为 `Vector7d`）则安全；动态维度可能内部分配临时变量 |
| `std::function` 调用 | ⚠️ | 小闭包（SBO，16-32 字节）安全；大闭包堆分配 |
| `std::optional<T>` | ✅ | 值语义，不分配 |
| `std::variant<T1, T2>` | ✅ | 值语义，不分配 |
| `std::span<T>` | ✅ | 纯引用，零拷贝 |

> **跨领域类比**：RT 安全编程的禁区清单就像飞行员的检查清单——不是因为飞行员不知道操作危险，而是因为在压力下人会忘记。SLAM 工程师转向控制时，写了几年的 `std::vector` 和 `std::string` 的肌肉记忆会在不经意间把你带入禁区。清单的价值在于：每次写 RT 代码前过一遍，确认没有违规。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：在 RT 线程中使用 `Eigen::MatrixXd`（动态大小矩阵）
>
> **错误做法**：`Eigen::MatrixXd A(7, 7); A = J.transpose() * J;`
>
> **现象**：偶尔延迟尖峰（Eigen 创建临时变量时触发 malloc）
>
> **根本原因**：`MatrixXd` 存储在堆上。某些 Eigen 操作创建临时变量（如 `.eval()`、`auto` 导致延迟求值失败）
>
> **正确做法**：使用 `Eigen::Matrix<double, 7, 7>` 固定大小。启用 `EIGEN_RUNTIME_NO_MALLOC` 审计

> ⚠️ **编程陷阱**：用 `std::lock_guard<std::mutex>` 在 RT 线程中保护共享数据
>
> **错误做法**：RT 线程和非 RT 线程用 `std::mutex` 共享数据
>
> **现象**：偶尔 RT 线程卡住数毫秒
>
> **根本原因**：**优先级反转**——高优先级 RT 线程等待低优先级非 RT 线程释放 mutex
>
> **正确做法**：使用 `realtime_tools::RealtimeBuffer`（RT 端非阻塞读取，内部实现以当前版本为准）或 `std::atomic`

### 练习

1. ⭐ 审计以下代码，标出所有 RT 不安全的操作：
   ```cpp
   void update() {
       std::vector<double> q(7);
       for (int i = 0; i < 7; ++i) q[i] = read_encoder(i);
       auto tau = compute_control(q);
       std::cout << "tau = " << tau[0] << std::endl;
       { std::lock_guard<std::mutex> lock(mtx); shared_state = q; }
   }
   ```
2. ⭐⭐ 改写为 RT 安全版本（`std::array`、`EIGEN_RUNTIME_NO_MALLOC`、`RealtimeBuffer`）。
3. ⭐⭐ 解释 `std::shared_ptr` 在 RT 中的危险。设计测试：1000 个 `shared_ptr` 指向同一对象，RT 线程中逐个释放，观察哪次触发 `delete`。

---

## 4. 标准三件套：SCHED_FIFO + mlockall + EIGEN_RUNTIME_NO_MALLOC ⭐⭐

### 4.1 为什么需要三件套 ⭐⭐

即使有 PREEMPT_RT 内核，RT 线程仍可能被干扰：

| 干扰源 | 解决方案 |
|--------|---------|
| 调度器抢占（CFS 不保证 RT 优先） | `SCHED_FIFO` |
| 页面换出（swap → 磁盘 I/O） | `mlockall` |
| 隐藏堆分配（Eigen 临时变量） | `EIGEN_RUNTIME_NO_MALLOC` |

### 4.2 SCHED_FIFO——实时调度策略 ⭐⭐

```cpp
#include <pthread.h>
#include <sched.h>

void setup_realtime_thread() {
    struct sched_param param;
    param.sched_priority = 80;  // 范围 1-99, 越大越优先
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        perror("sched_setscheduler failed");
        // 需要 root 权限或 CAP_SYS_NICE
    }
}
```

**SCHED_FIFO vs SCHED_OTHER**：

| 维度 | SCHED_OTHER (CFS) | SCHED_FIFO |
|------|-------------------|-----------|
| 抢占 | 时间片用完后被抢占 | 只被更高优先级任务抢占 |
| 延迟 | 不确定 | 确定性 |
| 公平性 | ✅ 所有任务公平 | ❌ 低优先级可能饿死 |
| 权限 | 普通用户 | root 或 CAP_SYS_NICE |

**优先级选择指南**：

| 范围 | 推荐用途 |
|------|---------|
| 90-99 | 内核线程（不要占用） |
| 80-89 | 控制循环（1 kHz 伺服） |
| 50-79 | 传感器采集、通信 |
| 1-49 | 规划、日志、监控 |

### 4.3 mlockall——内存页面锁定 ⭐⭐

```cpp
#include <array>
#include <pthread.h>
#include <sys/mman.h>

void lock_memory() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }
}

// 分块触碰当前线程栈，避免一次性声明 8 MB 局部数组导致栈溢出。
// 调用前应通过 pthread_attr_setstacksize() 给 RT 线程设置足够大的栈。
[[gnu::noinline]] void prefault_stack(std::size_t bytes) {
    constexpr std::size_t kChunk = 64 * 1024;
    constexpr std::size_t kPage = 4096;
    std::array<char, kChunk> chunk{};
    volatile char* pages = chunk.data();

    for (std::size_t i = 0; i < kChunk; i += kPage) {
        pages[i] = 0;
    }
    if (bytes > kChunk) {
        prefault_stack(bytes - kChunk);
    }
}

void create_rt_thread() {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 16 * 1024 * 1024);  // 16 MB

    pthread_t thread;
    pthread_create(&thread, &attr, [](void*) -> void* {
        lock_memory();
        prefault_stack(2 * 1024 * 1024);  // 按实际最坏栈深度留余量
        // rt_loop();  // RT 循环中只使用已预分配、已触碰的工作集
        return nullptr;
    }, nullptr);
    pthread_attr_destroy(&attr);
}
```

> **反事实推理**：如果不调用 `mlockall`，在 RT 循环中访问一个很久没用的变量（已被换出到 swap），会触发 major page fault——内核从磁盘读页面回内存。延迟可达毫秒级（HDD）到百微秒级（SSD），远超控制预算。`mlockall` 禁止 OS 换出任何页面。

### 4.4 EIGEN_RUNTIME_NO_MALLOC——堆分配审计 ⭐⭐

```cpp
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>

void realtime_loop() {
    Eigen::internal::set_is_malloc_allowed(false);  // 进入 RT 段

    Eigen::Matrix<double, 7, 7> M;  // ✅ 固定大小, 栈分配
    Eigen::Matrix<double, 7, 1> q;  // ✅

    // Eigen::MatrixXd bad(7, 7);   // ❌ 会 abort!

    Eigen::internal::set_is_malloc_allowed(true);   // 退出 RT 段
}
```

**这是调试利器**：开发阶段启用，任何隐藏的 Eigen 堆分配立即暴露（程序 abort + 堆栈追踪）。

### 4.5 EIGEN_RUNTIME_NO_MALLOC 实战：找出隐藏的堆分配 ⭐⭐

`EIGEN_RUNTIME_NO_MALLOC` 是调试 RT 代码中 Eigen 堆分配的最有力工具。但很多隐蔽的堆分配不是一眼能看出的。

**隐蔽的 Eigen 堆分配来源**：

```cpp
// 场景 1: auto 关键字导致延迟求值失败
Eigen::Matrix<double, 7, 7> J;
auto JtJ = J.transpose() * J;
// JtJ 的类型是 Eigen::Product<...>, 不是 Matrix7x7!
// 后续 JtJ(0,0) 触发求值 → 可能创建临时变量

// ✅ 正确写法: 强制求值
Eigen::Matrix<double, 7, 7> JtJ = J.transpose() * J;

// 场景 2: 混合固定和动态大小
Eigen::Matrix<double, 7, 1> q;
Eigen::VectorXd weights(7);  // 动态大小!
auto result = q.array() * weights.array();
// result 是动态大小, 触发堆分配

// ✅ 正确写法: 全部固定大小
Eigen::Matrix<double, 7, 1> weights_fixed;
auto result = q.array() * weights_fixed.array();

// 场景 3: Pinocchio 返回动态大小
// pinocchio::computeJointJacobians 返回 MatrixXd
// ❌ 直接在 RT 中调用 → 堆分配!
// ✅ 预分配并使用 in-place 版本:
Eigen::Matrix<double, 6, Eigen::Dynamic> J_pre(6, model.nv);
pinocchio::computeJointJacobians(model, data);
// 使用 data.J 已预分配的缓存

// 场景 4: Eigen 的 .inverse()
Eigen::Matrix<double, 7, 7> M;
auto M_inv = M.inverse();
// 对固定大小矩阵, inverse() 用栈分配 → ✅ 安全
// 但对 MatrixXd, inverse() 触发堆分配 → ❌
```

**完整审计流程**：

```cpp
// Step 1: 在 CMakeLists.txt 中为 Debug 模式启用
target_compile_definitions(my_controller PRIVATE
    $<$<CONFIG:Debug>:EIGEN_RUNTIME_NO_MALLOC>
)

// Step 2: 在 RT 入口和出口标记
void update() {
    #ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
    #endif

    // ... 所有 RT 代码 ...

    #ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
    #endif
}

// Step 3: Debug 模式编译并运行
// 任何隐藏的 Eigen 堆分配 → 立即 abort + backtrace
// 用 GDB 定位:
//   gdb ./my_controller
//   run
//   # abort 时自动停下
//   bt       # 查看调用栈, 找到触发分配的代码行
```

**更进一步：用 LD_PRELOAD 拦截所有 malloc**：

```cpp
// malloc_interpose.cpp — 拦截所有 malloc/free 调用
#include <dlfcn.h>
#include <cstdio>
#include <atomic>

static std::atomic<bool> in_rt_section{false};

extern "C" void* malloc(size_t size) {
    if (in_rt_section.load(std::memory_order_relaxed)) {
        // RT 段中触发 malloc → 报告并终止
        fprintf(stderr, "!!! malloc(%zu) called in RT section !!!\n",
                size);
        // 打印调用栈
        void* bt[20];
        int n = backtrace(bt, 20);
        backtrace_symbols_fd(bt, n, 2);
        abort();
    }
    // 非 RT 段: 正常调用原始 malloc
    static auto* real_malloc =
        (void*(*)(size_t))dlsym(RTLD_NEXT, "malloc");
    return real_malloc(size);
}

// RT 代码中:
void enter_rt() { in_rt_section.store(true); }
void leave_rt() { in_rt_section.store(false); }
```

```bash
# 编译拦截库
g++ -shared -fPIC -o libmalloc_interpose.so malloc_interpose.cpp -ldl

# 运行时拦截
LD_PRELOAD=./libmalloc_interpose.so ./my_controller
```

这个方法比 `EIGEN_RUNTIME_NO_MALLOC` 更彻底——它拦截**所有**堆分配，不仅仅是 Eigen 的。`std::string` 拼接、`std::vector` 扩容、甚至 `printf` 的内部缓冲区分配都会被捕获。

### 4.6 CPU 频率锁定与核心隔离 ⭐⭐

```bash
# CPU governor 设为 performance (固定最高频率)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 核心隔离 (GRUB 参数)
# isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3

# RT 线程绑定到隔离核心
taskset -c 2 ./my_rt_controller
```

`isolcpus` 告诉调度器不在这些核心上运行普通任务。`nohz_full` 关闭定时器中断。`rcu_nocbs` 把 RCU 回调移走——消除周期性内核干扰。

### 4.7 pmr 内存池——在 RT 上下文中使用动态容器的唯一安全方式 ⭐⭐⭐

前面的禁区清单禁止了 `std::vector`、`std::string` 等动态容器。但某些场景确实需要在 RT 线程中使用变长数据——例如关节数量在运行时确定的通用控制器、或需要动态长度消息的诊断接口。C++17 引入的 PMR（Polymorphic Memory Resource，多态内存资源）为此提供了一条出路。

**核心思想**：PMR 将"内存分配策略"从容器类型中解耦。`std::pmr::vector<double>` 和 `std::vector<double>` 接口完全相同，但前者的内存来源由一个 `std::pmr::memory_resource` 对象控制——你可以让它从预分配的固定大小内存池中分配，而非调用 `malloc`。

回顾前置章节 `02_C++基础与进阶/20_并发与系统编程/50_内存分配策略与pmr`：PMR 的核心抽象是 `std::pmr::memory_resource`，它定义了 `allocate()` / `deallocate()` 虚接口。`std::pmr::monotonic_buffer_resource` 从预分配缓冲区中线性分配，永不调用 `free`——这是 RT 场景的理想选择，因为分配操作的时间复杂度为 $O(1)$（仅移动指针）。

```cpp
#include <memory_resource>
#include <vector>
#include <array>

// === 在 on_configure() 中预分配（非 RT 阶段） ===
// 预分配 4 KB 缓冲区，足够 500 个 double
alignas(alignof(double)) std::array<std::byte, 4096> buffer;

// monotonic_buffer_resource: 从 buffer 中线性分配
// 第二个参数 nullptr 表示耗尽时不回退到默认分配器（直接失败）
std::pmr::monotonic_buffer_resource pool(
    buffer.data(), buffer.size(), std::pmr::null_memory_resource());

// === 在 RT 线程中使用 ===
void update_rt(std::pmr::monotonic_buffer_resource& pool) {
    pool.release();  // 重置分配指针（O(1)，不调用 free）

    // pmr::vector 从 pool 分配，不触发 malloc
    std::pmr::vector<double> joint_torques(7, &pool);  // ✅ RT 安全
    for (int i = 0; i < 7; ++i) {
        joint_torques[i] = compute_torque(i);
    }

    // 即使 push_back 也安全（只要 pool 有足够空间）
    // joint_torques.push_back(extra_value);  // ✅ 从 pool 分配

    // pool.release() 在下次调用时批量"回收"所有分配
    // 无 free/delete，零碎片，确定性时间
}
```

**pmr 在 RT 中的使用规则**：

| 规则 | 原因 |
|------|------|
| 缓冲区必须在非 RT 阶段预分配 | 避免 RT 中的 `malloc` |
| 使用 `null_memory_resource` 作为上游 | 溢出时立即报错而非回退到 `malloc` |
| 每个 RT 周期开头调用 `release()` | 重置分配指针，"批量回收"上周期的所有分配 |
| 缓冲区大小必须覆盖最坏情况 | 溢出 = 运行时错误（而非延迟尖峰） |

> **反事实推理**：如果不用 `null_memory_resource` 作为上游会怎样？默认的上游是 `std::pmr::new_delete_resource()`——当预分配缓冲区耗尽时，PMR 会回退到 `new`，即 `malloc`。这完全违背了 RT 安全的初衷。`null_memory_resource` 在溢出时直接抛出 `std::bad_alloc`（在开发阶段暴露问题），而非静默地调用 `malloc`（在生产环境中埋雷）。

**pmr vs 固定大小 Eigen 的选型**：

| 场景 | 推荐方案 | 原因 |
|------|---------|------|
| 关节数编译期已知（如 Franka 7-DOF） | `Eigen::Matrix<double, 7, 1>` | 零分配，栈上 |
| 关节数运行期确定（通用控制器） | `std::pmr::vector<double>` + 预分配池 | 灵活且 RT 安全 |
| 需要 Eigen 矩阵运算的通用 DOF | `Eigen::VectorXd` + 预分配 in-place | Pinocchio 接口需要 |
| 诊断/日志数据（变长） | `std::pmr::string` + 预分配池 | 避免堆分配 |

> **本质洞察**：PMR 的设计哲学是"控制策略而非控制类型"——同一个 `std::pmr::vector<double>` 在非 RT 线程中可以用默认分配器（方便），在 RT 线程中切换到 `monotonic_buffer_resource`（安全）。容器的接口不变，只有内存来源不同。这种"正交分离"的设计在 C++ 实时编程中至关重要：它让你不需要为 RT 重写所有数据结构，只需替换内存来源。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：忘记预 fault 栈空间
>
> **错误做法**：`mlockall()` 后直接进入 RT 循环
>
> **现象**：RT 循环前几次迭代出现延迟尖峰
>
> **根本原因**：`mlockall` 只锁定已映射页面。栈按需增长——新页面首次访问触发 minor page fault
>
> **正确做法**：为 RT 线程显式设置足够大的栈，启动阶段分块预触碰实际需要的栈深度；大型工作缓冲区改为启动阶段预分配并触碰，避免在函数里声明 8 MB 级局部数组

### 练习

1. ⭐ 写完整 RT 配置函数：SCHED_FIFO(80) + mlockall + 核心绑定。用 `chrt -p <pid>` 验证。
2. ⭐⭐ 启用 `EIGEN_RUNTIME_NO_MALLOC`，故意用 `MatrixXd` 触发 abort，然后改为固定大小通过。
3. ⭐⭐ 对比有无 CPU 频率锁定的 cyclictest 结果。

---

## 5. 无锁数据结构——realtime_tools 包 ⭐⭐

### 5.1 核心问题：RT 线程和非 RT 线程如何通信 ⭐⭐

```
非 RT 线程 (ROS2 回调)              RT 线程 (控制循环)
┌─────────────────────┐            ┌──────────────────┐
│ subscriber 回调:     │  ──?──→   │ cmd = read()     │
│   new_command = msg  │            │ tau = PD(cmd, q) │
│                      │            │ write(tau)       │
│ publisher:           │  ←─?──    │ state = q        │
│   msg.position = q   │            │                  │
└─────────────────────┘            └──────────────────┘

用 mutex? → RT 线程可能被阻塞!
用无锁? → RT 线程永不阻塞!  ✅
```

### 5.2 RealtimeBuffer<T>——RT 端非阻塞读取 ⭐⭐

```cpp
#include <realtime_tools/realtime_buffer.h>

using Vector7d = Eigen::Matrix<double, 7, 1>;

struct JointCommandRT {
    Vector7d position = Vector7d::Zero();
};

realtime_tools::RealtimeBuffer<JointCommandRT> command_buffer;

// 初始化阶段 (on_configure, 非 RT)
void configure() {
    JointCommandRT initial;
    command_buffer.writeFromNonRT(initial);
}

// 非 RT 线程 (subscriber 回调):
void cmd_callback(const JointCommand::SharedPtr msg) {
    JointCommandRT cmd;
    for (int i = 0; i < 7; ++i) {
        cmd.position(i) = msg->position[i];
    }
    command_buffer.writeFromNonRT(cmd);
}

// RT 线程 (控制循环):
void update() {
    const JointCommandRT* cmd = command_buffer.readFromRT();  // 永不阻塞!
    const Vector7d& q_ref = cmd->position;                    // 固定大小, 不分配
    // 使用 q_ref 计算控制...
}
```

**实现提醒**：`RealtimeBuffer` 的价值在于给 RT 线程提供非阻塞读取接口，而不是承诺某一种固定的内部锁实现。不同 `realtime_tools` 版本可能在内部使用 mutex、try-lock 或其它同步细节；工程上只依赖它的公开 API 语义，不依赖内部实现。

**RT 细节**：不要在这里用 `RealtimeBuffer<Eigen::VectorXd>` 再在 `update()` 中 `auto cmd = *readFromRT()`。`VectorXd` 持有堆内存，动态大小对象的复制/临时变量可能触发分配或不可控复制成本。DOF 固定时用 `Eigen::Matrix<double, 7, 1>`；DOF 可变时用 `std::array<double, MAX_DOF>` 加有效长度，或在配置阶段完成所有缓冲区预分配。

### 5.3 RealtimePublisher<T>——RT 安全的 ROS2 发布

```cpp
#include <realtime_tools/realtime_publisher.h>

// 初始化 (on_configure, 非 RT 阶段)
auto rt_pub = std::make_shared<
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        node_, "/joint_states", 10);

// RT 循环中:
if (rt_pub->trylock()) {           // 非阻塞! 获取不到就跳过
    rt_pub->msg_.position = current_positions;
    rt_pub->msg_.velocity = current_velocities;
    rt_pub->unlockAndPublish();    // 异步发送
}
```

**关键设计**：`trylock()` 非阻塞——上次发布还在处理中时立即返回 `false`，RT 线程不等待。某些周期的状态不被发布，但 RT 线程永不阻塞。

### 5.4 Lock-Free Queue 实现——SPSC Ring Buffer ⭐⭐

RealtimeBuffer 解决的是"最新值覆盖"的场景。但某些场景需要**队列语义**——生产者按顺序推入数据，消费者按顺序取出，不丢失。典型场景：RT 线程产生的日志/诊断数据，需要完整记录。

**SPSC（Single-Producer, Single-Consumer）无锁环形缓冲区**：

```cpp
#include <atomic>
#include <array>
#include <cstddef>

// SPSC Lock-Free Ring Buffer
// 仅适用于单生产者 + 单消费者场景
template <typename T, size_t Capacity>
class SPSCRingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be power of 2");  // 用位运算取模
public:
    bool push(const T& item) noexcept {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = (head + 1) & (Capacity - 1);

        // 检查是否满
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;  // 队列满, 不阻塞!
        }

        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& item) noexcept {
        const size_t tail = tail_.load(std::memory_order_relaxed);

        // 检查是否空
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // 队列空, 不阻塞!
        }

        item = buffer_[tail];
        tail_.store((tail + 1) & (Capacity - 1),
                    std::memory_order_release);
        return true;
    }

    size_t size() const noexcept {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t tail = tail_.load(std::memory_order_relaxed);
        return (head - tail) & (Capacity - 1);
    }

private:
    // 缓存行对齐: 防止 head_ 和 tail_ 在同一缓存行上
    // → false sharing (两个核心反复使对方缓存失效)
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
    std::array<T, Capacity> buffer_;
};
```

**使用示例**：

```cpp
// RT 线程推诊断数据, 非 RT 线程取数据写日志
struct DiagData {
    double timestamp;
    std::array<double, 7> joint_positions;
    std::array<double, 7> joint_torques;
};

SPSCRingBuffer<DiagData, 1024> diag_queue;  // 1024 槽, 全栈

// RT 线程 (1 kHz):
void rt_update() {
    DiagData d{get_time(), current_q, current_tau};
    diag_queue.push(d);  // O(1), 无锁, 无分配
    // 满了就丢——RT 线程绝不阻塞
}

// 非 RT 线程 (100 Hz 或异步):
void logger_thread() {
    DiagData d;
    while (diag_queue.pop(d)) {
        write_to_file(d);  // 文件 I/O 只在非 RT 线程中!
    }
}
```

**关键设计决策**：

| 决策 | 原因 |
|------|------|
| `Capacity` 必须是 2 的幂 | 用 `& (Capacity - 1)` 替代 `% Capacity`——无除法指令 |
| `alignas(64)` 缓存行对齐 | 防止 false sharing：head_ 和 tail_ 在不同核心频繁写入 |
| `push` 返回 bool 不阻塞 | RT 线程永不等待——满了就丢弃，保证确定性 |
| 无 `new`/`delete` | `std::array` 全栈分配，容量编译期确定 |

### 5.5 Triple Buffer——比双缓冲更灵活的通信方式 ⭐⭐⭐

RealtimeBuffer 的双缓冲有一个限制：如果写入频率远高于读取频率，大量写入被"浪费"（覆盖了还没被读取的数据）。Triple Buffer 引入第三个缓冲区，让读写完全独立。

```cpp
#include <atomic>
#include <array>

// Triple Buffer: 写者永不阻塞, 读者永远读最新完整帧
// 适用场景: 高频写入(RT) + 低频读取(非RT/可视化)
template <typename T>
class TripleBuffer {
public:
    TripleBuffer() : flags_(0b110) {}  // 初始: read=0, write=1, middle=2

    // 写者端 (RT 线程调用)
    T& get_write_buffer() noexcept {
        return buffers_[write_index()];
    }

    void publish() noexcept {
        // 原子交换 write 和 middle 索引
        // 设置 "new data available" 标志位
        uint8_t flags = flags_.load(std::memory_order_relaxed);
        uint8_t new_flags;
        do {
            new_flags = (flags & 0b011)         // 保留 read 索引
                      | (write_index() << 2)    // middle = 旧 write
                      | 0b100000;               // new data flag
        } while (!flags_.compare_exchange_weak(
            flags, new_flags,
            std::memory_order_acq_rel,
            std::memory_order_relaxed));
    }

    // 读者端 (非 RT 线程调用)
    bool new_data_available() const noexcept {
        return (flags_.load(std::memory_order_acquire) & 0b100000) != 0;
    }

    const T& read() noexcept {
        if (new_data_available()) {
            // 原子交换 read 和 middle 索引
            // 清除 "new data available" 标志位
            swap_read_middle();
        }
        return buffers_[read_index()];
    }

private:
    std::array<T, 3> buffers_;
    std::atomic<uint8_t> flags_;  // 编码三个索引 + new data flag

    uint8_t write_index() const noexcept { /* 从 flags 解码 */ }
    uint8_t read_index() const noexcept { /* 从 flags 解码 */ }
    void swap_read_middle() noexcept { /* 原子交换 */ }
};
```

**Triple Buffer vs Double Buffer vs Queue**：

| 维度 | Double Buffer | Triple Buffer | Lock-Free Queue |
|------|--------------|---------------|-----------------|
| 语义 | 最新值覆盖 | 最新值覆盖 | 先进先出 |
| 写者阻塞 | 极少 | **永不** | 满时丢弃 |
| 读者得到 | 最新完整帧 | 最新完整帧 | 按顺序所有帧 |
| 适用场景 | 命令传递 | 状态传递/可视化 | 日志/诊断 |

### 5.6 realtime_tools 新一代 API：LockFreeSPSCQueue 与 LockFreeMPMCQueue ⭐⭐

从 2025 年起，`realtime_tools` 包对无锁通信组件进行了系统性重构，引入了基于 Boost.Lockfree 的 `LockFreeSPSCQueue` 和 `LockFreeMPMCQueue`，同时将 `RealtimeBuffer` 的语义重新定义为 `RealtimeThreadSafeBox`（线程安全盒子）。这些变化反映了 ros2_control 社区对 RT 通信模式更精细化的认知。

**新旧 API 映射**：

| 旧名称 / 概念 | 新名称 | 语义 |
|--------------|--------|------|
| `RealtimeBuffer<T>` | `RealtimeThreadSafeBox<T>` | 最新值覆盖（只关心最新消息） |
| 自定义 SPSC 队列 | `LockFreeSPSCQueue<T>` | 先进先出（不丢中间消息，单生产者单消费者） |
| 自定义 MPMC 队列 | `LockFreeMPMCQueue<T>` | 先进先出（多生产者多消费者） |
| `RealtimePublisher<T>` | `RealtimePublisher<T>` | RT 安全的 ROS2 发布（不变） |

**选型决策树**：

```
需要在 RT 和非 RT 线程间传递数据？
│
├── 只关心最新值？（如目标位姿、控制命令）
│   └── 用 RealtimeThreadSafeBox<T>
│
├── 中间消息也重要？（如日志、诊断、轨迹采样点）
│   ├── 单生产者 + 单消费者？
│   │   └── 用 LockFreeSPSCQueue<T>（性能最优）
│   └── 多生产者或多消费者？
│       └── 用 LockFreeMPMCQueue<T>
│
├── 需要从 RT 线程发布 ROS2 话题？
│   └── 用 RealtimePublisher<T>
│
└── 数据是简单标量（int/double/bool）？
    └── 用 std::atomic<T>（最简洁）
```

**LockFreeSPSCQueue 使用示例**：

```cpp
#include <realtime_tools/lock_free_queue.hpp>

struct TrajectoryPoint {
    double timestamp;
    std::array<double, 7> positions;
    std::array<double, 7> velocities;
};

// 队列容量 256，在 on_configure() 中创建（非 RT）
realtime_tools::LockFreeSPSCQueue<TrajectoryPoint> traj_queue(256);

// 非 RT 线程：规划器推入轨迹点
void planner_callback(const TrajectoryMsg::SharedPtr msg) {
    TrajectoryPoint pt;
    pt.timestamp = msg->header.stamp.sec;
    std::copy_n(msg->positions.begin(), 7, pt.positions.begin());
    std::copy_n(msg->velocities.begin(), 7, pt.velocities.begin());
    traj_queue.push(pt);  // 底层 Boost.Lockfree，非阻塞
}

// RT 线程：控制器取出轨迹点
void update_rt() {
    TrajectoryPoint pt;
    if (traj_queue.pop(pt)) {
        // 成功取到新点，更新控制目标
        execute_trajectory_point(pt);
    } else {
        // 队列空，继续使用上次的控制目标
    }
}
```

> **"不是X而是Y"句式**：`LockFreeSPSCQueue` 不是"更好的 RealtimeBuffer"——两者解决的是不同问题。RealtimeBuffer（Box）是"最新值语义"——写入总是覆盖旧值，读取总是得到最新值。Queue 是"先进先出语义"——所有写入都被保留（除非队列满），读取按顺序取出。选错了数据结构比用了次优实现更危险：如果用 Queue 传递控制命令，当命令更新频率高于消费频率时，控制器会在过时的命令上落后越来越远。

### 5.7 std::atomic 的内存序——ARM 陷阱 ⭐⭐⭐

| 内存序 | 保证 | 用途 | ARM 开销 |
|--------|------|------|---------|
| `relaxed` | 仅原子性 | 计数器、统计值 | 免费 |
| `acquire` | 此读之后的操作不重排到此读之前 | 消费者端 | Load 屏障 |
| `release` | 此写之前的操作不重排到此写之后 | 生产者端 | Store 屏障 |
| `seq_cst` | 所有 seq_cst 操作全局有序 | 默认 | 全屏障（最贵） |

**关键陷阱**：x86（TSO）上 acquire/release 几乎免费，但 **ARM/RISC-V（弱序）上需要实际屏障指令**。代码在 x86 上"正确"但在 ARM 上崩溃是真实 bug——Jetson 平台（ARM）是常见踩雷场景。

**具体场景**：你在 x86 开发机上用 `memory_order_relaxed` 写了一个无锁 flag 来通知 RT 线程"新数据已就绪"。x86 的 TSO（Total Store Order）模型恰好保证了 store 的顺序——所以即使用了 relaxed，数据也在 flag 之前可见。但部署到 Jetson Orin（ARM Cortex-A78AE）后，ARM 的弱内存序允许 store 被重排——RT 线程可能看到 flag=true 但读到的数据还是旧的。修复方法：flag 的写入用 `memory_order_release`，读取用 `memory_order_acquire`。

```cpp
// ❌ x86 上"碰巧正确"，ARM 上数据竞争
std::atomic<bool> data_ready{false};
SharedData shared_data;

// 写者（非 RT）
shared_data.value = new_value;                           // store 1
data_ready.store(true, std::memory_order_relaxed);       // store 2
// ARM 上 store 2 可能重排到 store 1 之前!

// ✅ 正确：release/acquire 配对
shared_data.value = new_value;                           // store 1
data_ready.store(true, std::memory_order_release);       // store 2
// release 保证 store 1 在 store 2 之前对其他线程可见

// 读者（RT）
if (data_ready.load(std::memory_order_acquire)) {        // load 1
    use(shared_data.value);                              // load 2
    // acquire 保证 load 2 看到 store 2 之前的所有写入
}
```

### 5.8 ros2_control 的设计原则 ⭐⭐

> **本质洞察**：ros2_control 的整个设计围绕一个核心原则——**所有内存分配都在 `on_configure()`（非 RT 阶段）完成，`update()`（RT 阶段）中不涉及任何分配**。realtime_tools 存在的根本原因就是确保 RT-非RT 通信不需要在 RT 端做任何分配或等待。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：在 RealtimeBuffer 中存储包含 `std::string` 的消息
>
> **错误做法**：`RealtimeBuffer<std_msgs::msg::String> buf;`
>
> **现象**：偶尔延迟尖峰——读取长字符串时拷贝触发堆分配
>
> **正确做法**：用 `const auto&` 接收引用避免拷贝；不在 RT 端传递含堆成员的类型；用固定大小 POD 类型

### 练习

1. ⭐ 用 `RealtimeBuffer<std::array<double, 7>>` 实现 subscriber→RT 循环的数据传递。
2. ⭐⭐ 精读当前版本的 `realtime_buffer.h`。画出 `writeFromNonRT()` 与 `readFromRT()` 的公开 API 时序图，并标注 RT 端为什么不能等待非 RT 线程。
3. ⭐⭐ 用 `RealtimePublisher` 在 1 kHz 循环中发布 `/joint_states`。用 `ros2 topic hz` 观察实际频率。

---

## 6. 优先级反转与解决方案 ⭐⭐

### 6.1 问题定义 ⭐⭐

**优先级反转**是 RT 系统经典问题——1997 年火星探路者号软件重启事件的元凶。

```
线程 H: 高优先级 (控制, priority=80)
线程 M: 中优先级 (数据处理, priority=50)
线程 L: 低优先级 (日志, priority=10)
资源: mutex (保护共享配置)

时间线:
  t0: L 获得 mutex
  t1: H 需要 mutex → 被阻塞
  t2: M 变为可运行 → 抢占 L (M > L)
  t3: M 运行... L 被阻塞... H 也被阻塞!
      → H 被 M "间接阻塞" → 优先级反转!
```

### 6.2 解决方案

**优先级继承**：高优先级线程等待 mutex 时，暂时**提升**持有者的优先级：

```cpp
pthread_mutex_t pi_mutex;
pthread_mutexattr_t attr;
pthread_mutexattr_init(&attr);
pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
pthread_mutex_init(&pi_mutex, &attr);
```

**更好的方案**：完全避免在 RT 线程中使用 mutex——用 RealtimeBuffer、atomic、无锁队列替代。这是 ros2_control 的设计选择。

### 练习

1. ⭐⭐ 设计最小示例复现优先级反转。切换到 `PTHREAD_PRIO_INHERIT` 后观察改善。
2. ⭐⭐ 解释 `std::mutex` 为什么不支持优先级继承。（C++ 标准不规定线程优先级——需要直接用 `pthread_mutex`。）

---

## 7. libfranka 实时接口精读 ⭐⭐

### 7.1 libfranka 的 1 kHz 控制回调

libfranka 的实时控制采用 **callback 模式**——用户提供函数，libfranka 每 1 ms 调用一次：

```cpp
#include <franka/robot.h>
#include <franka/model.h>

franka::Robot robot("192.168.1.100");
franka::Model model = robot.loadModel();

robot.control([&model](const franka::RobotState& state,
                       franka::Duration dt) -> franka::Torques
{
    // !! RT 线程中执行 !!
    // !! 不能 new/delete/throw/cout/mutex !!

    std::array<double, 7> q = state.q;
    std::array<double, 7> dq = state.dq;
    std::array<double, 7> gravity = model.gravity(state);

    std::array<double, 7> tau_cmd;
    for (int i = 0; i < 7; ++i) {
        tau_cmd[i] = gravity[i]
                   + kp[i] * (q_desired[i] - q[i])
                   + kd[i] * (0.0 - dq[i]);
    }

    return franka::Torques(tau_cmd);
});
```

### 7.2 libfranka 控制循环完整代码 Walkthrough ⭐⭐

下面是一个生产级 libfranka 力矩控制器的完整 walkthrough，包括 RT 配置、安全检查、和异常处理。

```cpp
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <cmath>

// === 编译期类型定义 (零堆分配) ===
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

// === 控制参数 (编译期常量) ===
constexpr std::array<double, 7> kp = {600, 600, 600, 600, 250, 150, 50};
constexpr std::array<double, 7> kd = {50, 50, 50, 50, 30, 25, 15};
constexpr double kDeltaTauMax = 1.0;   // 力矩变化率限制 (Nm/ms)
constexpr double kTauAbsMax = 87.0;    // 绝对力矩限制 (Nm)

int main() {
    try {
        // === Step 1: 连接机器人 (非 RT 阶段) ===
        franka::Robot robot("192.168.1.100");
        robot.automaticErrorRecovery();  // 清除之前的错误

        // === Step 2: 加载动力学模型 (非 RT 阶段, 涉及堆分配) ===
        franka::Model model = robot.loadModel();

        // === Step 3: 读取初始状态作为目标 (非 RT 阶段) ===
        franka::RobotState initial_state = robot.readOnce();
        Vector7d q_desired = Eigen::Map<const Vector7d>(
            initial_state.q.data());

        // === Step 4: 预热 Eigen (避免 RT 中首次调用触发分配) ===
        // 某些 Eigen 操作首次调用会初始化内部状态
        {
            Matrix7d dummy_M = Matrix7d::Identity();
            Vector7d dummy_v = Vector7d::Zero();
            (void)(dummy_M * dummy_v);  // 预热
        }

        // === Step 5: 进入 RT 控制回调 ===
        #ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(false);
        #endif

        std::array<double, 7> tau_prev{};  // 上一周期力矩 (零初始化)

        robot.control([&](const franka::RobotState& state,
                         franka::Duration dt) -> franka::Torques
        {
            // !! 以下所有代码在 RT 线程中执行 !!
            // !! 禁止: new/delete/throw/cout/mutex/vector/string !!

            // --- 读取当前状态 (零拷贝, std::array) ---
            Vector7d q = Eigen::Map<const Vector7d>(state.q.data());
            Vector7d dq = Eigen::Map<const Vector7d>(state.dq.data());

            // --- 重力补偿 (Pinocchio/libfranka 内部固定大小计算) ---
            std::array<double, 7> gravity_arr = model.gravity(state);
            Vector7d gravity = Eigen::Map<const Vector7d>(
                gravity_arr.data());

            // --- PD + 重力补偿控制律 ---
            Vector7d tau_task = Vector7d::Zero();
            for (int i = 0; i < 7; ++i) {
                tau_task(i) = kp[i] * (q_desired(i) - q(i))
                            + kd[i] * (0.0 - dq(i));
            }
            Vector7d tau_cmd = tau_task + gravity;

            // --- 安全检查 1: 绝对力矩限幅 ---
            for (int i = 0; i < 7; ++i) {
                tau_cmd(i) = std::clamp(tau_cmd(i),
                    -kTauAbsMax, kTauAbsMax);
            }

            // --- 安全检查 2: 力矩变化率限幅 ---
            // 防止力矩跳变损坏减速器
            std::array<double, 7> tau_out{};
            for (int i = 0; i < 7; ++i) {
                double delta = tau_cmd(i) - tau_prev[i];
                delta = std::clamp(delta,
                    -kDeltaTauMax, kDeltaTauMax);
                tau_out[i] = tau_prev[i] + delta;
                tau_prev[i] = tau_out[i];
            }

            // --- 安全检查 3: NaN 检测 ---
            for (int i = 0; i < 7; ++i) {
                if (std::isnan(tau_out[i])) {
                    // 紧急停机: 输出零力矩
                    return franka::Torques({0,0,0,0,0,0,0});
                }
            }

            return franka::Torques(tau_out);
        });

        #ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(true);
        #endif

    } catch (const franka::Exception& e) {
        std::cerr << "franka::Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```

**逐行 RT 安全分析**：

| 代码行 | RT 安全？ | 理由 |
|--------|----------|------|
| `Eigen::Map<const Vector7d>(...)` | ✅ | 零拷贝，只是指针包装 |
| `model.gravity(state)` | ✅ | libfranka 内部用 `std::array` |
| `std::clamp(...)` | ✅ | 内联函数，零分配 |
| `std::isnan(...)` | ✅ | 编译器内建函数 |
| `kp[i] * (q_desired(i) - q(i))` | ✅ | 固定大小 Eigen 标量运算 |
| `Vector7d::Zero()` | ✅ | 固定大小，栈分配 |

> **"不是X而是Y"句式**：这段代码的安全检查不是"多此一举"的防御编程，而是**工业机械臂的硬性要求**。ISO 10218 规定力矩控制器必须有独立的力矩限幅层。力矩变化率限幅（$\Delta\tau_{max}$ = 1 Nm/ms）直接保护减速器——谐波减速器的瞬时扭转刚度约 100 Nm/rad，超过此限可能导致柔轮齿面永久变形。

### 7.3 libfranka 的 RT 设计要点

| 设计决策 | 理由 |
|---------|------|
| `std::array<double, 7>` | 固定大小，零堆分配 |
| callback 返回值是 POD | 无析构函数，无隐含 delete |
| NaN → `std::invalid_argument` | 在回调**外部**抛异常 |
| 超时 → `ControlException` | 同上 |
| 必须直连以太网 | 消费级交换机延迟不确定 |
| 必须 PREEMPT_RT | 非 RT 内核通信超时 |

### 7.4 计算预算

callback 的时间预算不是 1 ms——libfranka 的 UDP 通信本身需要约 200 us。控制计算预算约 **300-500 us**。复杂 MPC（1-5 ms）不能放在 callback 中——需要独立线程异步计算，通过 RealtimeBuffer 传递结果。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：在 libfranka 回调中使用 Eigen 动态类型
>
> **错误做法**：`Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(state.q.data(), 7);`
>
> **现象**：`EIGEN_RUNTIME_NO_MALLOC` abort
>
> **正确做法**：`Eigen::Matrix<double, 7, 1> q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());`

> 💡 **概念误区**：认为 callback 可以做"任何计算只要 < 1 ms"
>
> **实际上**：计算预算约 300-500 us（扣除通信开销）。复杂 MPC 必须异步

### 练习

1. ⭐ 阅读 `libfranka/examples/generate_joint_velocity_motion.cpp`。标注所有 RT 安全设计。
2. ⭐⭐ 实现关节空间 PD + 重力补偿。测量单次 callback 执行时间。
3. ⭐⭐⭐ 精读 `libfranka/src/robot.cpp` 中 `control()` 的 UDP 实时通信。标注 RT 线程创建、SCHED_FIFO、nanosleep、超时检测。

---

## 8. EtherCAT 实时通信 ⭐⭐⭐

### 8.1 为什么机械臂需要 EtherCAT

工业机械臂（UR、KUKA、ABB）的关节驱动器通过 **EtherCAT** 总线通信——实时以太网协议，保证微秒级确定性通信。

| 维度 | 普通以太网 | EtherCAT |
|------|----------|---------|
| 延迟 | 不确定（交换机缓冲） | 确定性（< 100 us 完整周期） |
| 拓扑 | 星形（需交换机） | 菊花链（无交换机） |
| 实时性 | 无保证 | 硬实时 |
| 协议栈 | 内核 TCP/IP | 直接操作以太网帧 |

### 8.2 主流 EtherCAT 主站库

| 库 | 特性 | 许可证 |
|----|------|--------|
| **SOEM** | 轻量，C，最常用 | GPL-2.0 |
| **EtherLab** (IgH) | 功能全面，内核模块 | GPL-2.0 |
| **ros2_control EtherCAT** | 基于 SOEM 的 ROS2 封装 | Apache-2.0 |

### 8.3 Distributed Clock（DC）同步 ⭐⭐⭐

EtherCAT 的 DC 同步是保证多关节协调运动的关键。没有 DC 同步，各从站（驱动器）的采样时刻不一致——关节 1 的位置比关节 7 "晚了" 几十微秒。在 1 kHz 控制、高速运动下，这种时间偏移导致笛卡尔空间的轨迹偏差。

**DC 同步的工作原理**：

```
主站 (PC)          从站 1           从站 2           从站 3
   │                  │                │                │
   │─── EtherCAT Frame ──────────────────────────────→│
   │                  │                │                │
   │    ┌─ SYNC0 ─┐  │   ┌─ SYNC0 ─┐ │  ┌─ SYNC0 ─┐  │
   │    │ 精确对齐 │  │   │ 精确对齐 │ │  │ 精确对齐 │  │
   │    └─────────┘  │   └─────────┘ │  └─────────┘  │
   │                  │                │                │
 t=0              t=0+δ₁         t=0+δ₂          t=0+δ₃
                   δ₁ < 100ns      δ₂ < 100ns     δ₃ < 100ns
```

**SYNC0 信号**：EtherCAT 主站计算出每个从站的传播延迟（通过 FRMW 命令测量往返时间），然后配置每个从站的本地时钟偏移，使所有从站的 SYNC0 中断在**同一物理时刻**（偏差 < 100 ns）触发。驱动器在 SYNC0 中断时刻采样编码器并更新力矩输出。

**配置 DC 同步（SOEM 示例）**：

```c
#include "ethercat.h"

// 配置 DC 同步 (在 OP 状态之前)
void setup_dc_sync(uint16 slave_id) {
    // SYNC0 周期 = 控制周期 = 1 ms = 1000000 ns
    uint32_t cycle_ns = 1000000;

    // SYNC0 激活, SYNC1 不使用
    // 第一个参数: 1 = 激活 SYNC0
    // 第二个参数: cycle 时间 (ns)
    // 第三个参数: SYNC0 相对于周期起点的偏移
    ec_dcsync0(slave_id, TRUE, cycle_ns, 0);
}

// 主循环中同步主站时钟
void rt_loop() {
    while (running) {
        // 1. 发送/接收 EtherCAT 帧
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        // 2. 补偿主站时钟漂移
        ec_sync(ec_DCtime, cycle_ns, &toff);

        // 3. 等待下一周期 (使用补偿后的时间)
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                        &next_wakeup, NULL);
        add_timespec(&next_wakeup, cycle_ns + toff);
    }
}
```

### 8.4 PDO 映射——从站数据交换的底层机制 ⭐⭐⭐

PDO（Process Data Object）定义了主站与从站之间交换哪些数据。EtherCAT 的 PDO 映射决定了每个控制周期中从站发送/接收的具体数据项。

**典型机械臂驱动器 PDO 映射**：

| 方向 | PDO 索引 | 数据项 | 类型 | 大小 |
|------|---------|--------|------|------|
| **RxPDO**（主→从） | 0x1600 | 目标力矩 | INT32 | 4 B |
| | 0x1601 | 控制字 | UINT16 | 2 B |
| | 0x1602 | 运行模式 | INT8 | 1 B |
| **TxPDO**（从→主） | 0x1A00 | 实际位置 | INT32 | 4 B |
| | 0x1A01 | 实际速度 | INT32 | 4 B |
| | 0x1A02 | 实际力矩 | INT16 | 2 B |
| | 0x1A03 | 状态字 | UINT16 | 2 B |

**7 关节机械臂的总线数据量**：

```
每关节 RxPDO: 4 + 2 + 1 = 7 B
每关节 TxPDO: 4 + 4 + 2 + 2 = 12 B
7 关节总计: 7 × (7 + 12) = 133 B (+ EtherCAT 帧头)
整帧大小 ≈ 200 B → 100 Mbps 以太网传输时间 < 20 us
```

> **本质洞察**：EtherCAT 的高效不在于"网络速度快"——100 Mbps 并不比普通以太网快。它的核心优势是**确定性**：所有从站在一个以太网帧的"飞过"过程中完成数据交换（每个从站延迟 < 1 us），不需要协议栈的发送/确认/重传。这就是 EtherCAT 能保证 < 100 us 总线周期的原因。

### 8.5 与足式硬件栈的对比

| 维度 | 机械臂 | 足式 |
|------|--------|------|
| 从站数 | 6-7 | 12（4 腿 x 3） |
| 周期 | 1 ms / 250 us | 500 us - 1 ms |
| 控制模式 | 力矩/速度/位置 | 力矩为主 |
| 驱动器 | 商用（Elmo、Maxon） | 常自研/准直驱 |
| 安全协议 | FSoE | 通常无 |

### ⚠️ 常见陷阱

> 🧠 **思维陷阱**：认为 EtherCAT "自动实时"
>
> **实际上**：EtherCAT 只保证**总线层**实时。主站软件运行在通用 OS 上——如果 OS 调度延迟了主站发送/接收函数，总线帧也会延迟。EtherCAT 需要 PREEMPT_RT 保证主站的确定性

### 练习

1. ⭐⭐ 对比 libfranka（UDP 直连）和 EtherCAT。为什么 Franka 选 UDP？
2. ⭐⭐⭐ 用 SOEM `slaveinfo` 扫描 EtherCAT 网络（需真实硬件或仿真从站）。

---

## 9. 与足式实时工程的对比 ⭐⭐

### 9.1 相同点

机械臂和足式共享大部分 RT 原则：PREEMPT_RT、SCHED_FIFO + mlockall、无堆分配、无锁通信、Eigen 固定大小。

### 9.2 差异点

| 维度 | 机械臂 | 足式 |
|------|--------|------|
| 控制频率 | 1 kHz（标准） | 500 Hz - 1 kHz |
| 计算量 | RNEA ~2 us（7-DOF） | RNEA ~5 us（18-DOF 浮动基座） |
| WBC/MPC | 可选 | 几乎必须 |
| MPC 预算 | 300-500 us（libfranka 约束） | 1-5 ms（通常异步） |
| 安全标准 | ISO 10218 | 通常无工业标准 |
| 接触状态 | 持续/已知 | 离散切换（步态） |
| 通信协议 | EtherCAT / 专用 UDP | EtherCAT / CAN / SPI |
| 关节类型 | 谐波减速器（高减速比） | 准直驱/行星减速器（低减速比） |
| 失控后果 | 碰撞工件/人员 | 摔倒 |

**可复用的经验**：RT 线程配置、realtime_tools 使用、Eigen 固定大小习惯、无锁 buffer 设计。

**需要注意的差异**：libfranka callback vs 足式 polling；工业安全标准（FSoE）；机械臂的更紧凑计算预算（300-500 us vs 足式的 1-5 ms）。

### 9.3 详细对比：机械臂 vs 足式实时需求差异 ⭐⭐

回顾足式方向的 足式/170_实时CPP工程（如果已学习）：足式控制栈的实时需求与机械臂有结构性差异。

**差异 1：同步 vs 异步控制架构**

机械臂（libfranka 模式）：
```
┌────────────── 1 ms ──────────────┐
│ read → compute → write (全同步)   │
│ 计算必须在 300-500 us 内完成      │
│ 超时 → 通信中断 → 安全停机        │
└──────────────────────────────────┘
```

足式（典型架构）：
```
┌── MPC 线程 (异步, 20-50 Hz) ──┐
│ 求解时间 1-20 ms, 不受控制频率限制 │
└──────────────┬───────────────┘
               │ RealtimeBuffer
               ▼
┌── WBC/PD 线程 (同步, 500-1kHz) ──┐
│ 读 MPC 结果 → WBC QP → PD → 写    │
│ 计算预算 500-1000 us               │
└───────────────────────────────────┘
```

**关键差异**：足式的 MPC 可以在独立线程中以较低频率异步运行（上一次 MPC 结果在新结果到来前持续使用），而机械臂的计算**必须**在每个 1 ms 周期内同步完成。这意味着机械臂无法使用耗时超过 300 us 的在线优化算法（如 MPC），除非采用足式的异步架构——而这需要 RealtimeBuffer 传递 MPC 结果。

**差异 2：安全响应时间**

| 安全事件 | 机械臂响应要求 | 足式响应要求 |
|---------|-------------|-------------|
| 碰撞检测 | < 5 ms（ISO 10218） | 无硬性标准 |
| 关节限位 | < 1 ms（硬件级） | 软限位，可恢复 |
| 力矩超限 | 立即停机 | 降额运行 |
| 通信中断 | 1-2 周期→停机 | 可用上次指令续跑 |

**差异 3：调试难度**

机械臂调试更困难的原因：工业机器人出错后的恢复成本高（可能损坏工件、末端工具、甚至人身安全）。足式机器人摔倒后通常可以自行站起或手动扶起。这导致机械臂的 RT 代码需要更多防御性检查（力矩限幅、速度监控、碰撞检测），这些检查本身也必须是 RT 安全的。

**差异 4：接触状态建模**

机械臂和足式机器人的接触状态有本质区别，这深刻影响了控制架构中实时计算的内容。

| 维度 | 机械臂 | 足式 |
|------|--------|------|
| 接触状态 | 持续且已知（末端持续接触工件或自由运动） | 离散切换（摆动相→支撑相，每步切换一次） |
| 接触力建模 | 阻抗/导纳控制，连续力反馈 | 接触力受步态时序约束，需要接触检测 |
| RT 计算内容 | PD + 重力补偿 + 力矩限幅（计算量小） | WBC QP + MPC + 接触力分配（计算量大） |
| 状态估计 | 编码器直读（高精度） | IMU + 腿部运动学融合（需 EKF/UKF） |

**关键洞察**：足式机器人的 RT 线程需要在每个周期内完成**状态估计 + WBC QP 求解 + 接触力分配**，计算量远大于机械臂的 PD 控制。但足式的控制频率通常允许 500 Hz（2 ms 预算），而机械臂在 libfranka 模式下只有 300-500 us 计算预算——预算更紧但计算量更小。这是两种系统在"计算复杂度 vs 时间预算"维度上的不同权衡。

**差异 5：通信拓扑与冗余**

| 维度 | 机械臂 | 足式 |
|------|--------|------|
| 总线拓扑 | 菊花链 EtherCAT（串行依赖） | EtherCAT / CAN / SPI（可按腿分组） |
| 单点故障影响 | 一个从站故障→整条链路中断 | 一条腿故障→三腿运动降级 |
| 冗余设计 | 通常无（工业标准靠 FSoE 安全协议） | 可按腿独立供电和通信 |

> **反事实推理**：如果把足式的异步 MPC + WBC 架构直接移植到机械臂会怎样？MPC 以 20-50 Hz 异步运行，WBC 在 RT 线程中以 1 kHz 执行。这在原理上可行——问题在于 libfranka 的 callback 模式要求**所有计算都在一个函数中完成**，不允许在回调之间保持异步线程的共享状态（libfranka 的 UDP 协议假设每个回调是独立的）。要实现异步架构，必须绕过 libfranka 的 callback 模式，直接使用 ros2_control 的 read/update/write 循环——这正是 `franka_ros2` 的做法。

### 练习

1. ⭐⭐ 列出 MIT Cheetah 和 Franka Panda 控制循环中共同的 RT 安全设计。至少 5 个。
2. ⭐⭐ 为什么足式把 MPC 放在独立线程异步执行（5-50 ms），而机械臂把控制计算放在 callback 中同步执行？
3. ⭐⭐⭐ 如果要在 Franka Panda 上运行 MPC（求解时间 5 ms），设计一个异步架构方案：MPC 线程和 RT 控制线程如何通信？当 MPC 求解未完成时，RT 线程使用什么？

---

## 10. ros2_control 主循环精读 ⭐⭐⭐

### 10.1 RT 主循环结构

`controller_manager/src/ros2_control_node.cpp` 是 ros2_control 的 RT 主循环。精读要点：

```cpp
// 简化的 RT 主循环
void rt_main_loop() {
    configure_sched_fifo(priority);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    while (rclcpp::ok()) {
        auto start = clock_gettime(CLOCK_MONOTONIC);

        hardware_interface->read(time, period);     // Step 1: 读
        for (auto& ctrl : active_controllers)
            ctrl->update(time, period);              // Step 2: 算
        hardware_interface->write(time, period);    // Step 3: 写

        auto next = start + period_ns;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}
```

**关键设计**：
- `TIMER_ABSTIME` 绝对时间睡眠——不因计算时间变化累积误差
- `read → update → write` 三步循环，每步最坏时间已知

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：在控制器 `update()` 中做 ROS2 service 调用
>
> **错误做法**：调用 `rclcpp::Client::call()` 查询参数
>
> **现象**：RT 线程阻塞到 service 响应——可能等数毫秒到数秒
>
> **正确做法**：`on_configure()` 中预加载参数。运行时更新通过 `RealtimeBuffer` 从非 RT 线程传入

### 10.2 Orocos RTT——ros2_control 之外的替代框架 ⭐⭐⭐

在 ros2_control 成为 ROS2 生态标准之前，**Orocos RTT**（Open Robot Control Software, Real-Time Toolkit）是机器人实时控制的主流框架。理解 Orocos RTT 有两个意义：（1）许多工业机器人系统仍在使用它；（2）对比两种框架的设计哲学有助于深入理解"实时控制框架应该解决什么问题"。

**Orocos RTT 核心概念**：

| 概念 | Orocos RTT | ros2_control 对应 |
|------|-----------|------------------|
| **TaskContext** | 实时组件的基类，包含 `updateHook()` | `ControllerInterface::update()` |
| **Port** | 组件间数据流通道（RT 安全） | `state_interfaces` / `command_interfaces` |
| **Activity** | 线程调度策略（周期/非周期） | `ControllerManager` 的 RT 主循环 |
| **Property** | 运行时可配置参数 | ROS2 parameter |
| **Operation** | 远程可调用方法 | ROS2 service |
| **Deployment** | XML 配置组件连接关系 | `ros2_control` YAML + URDF |

**Orocos RTT 的独特优势**：

1. **组件间通信天然 RT 安全**：Orocos 的 Port 使用无锁环形缓冲区实现数据流——这与 ros2_control 中手动使用 `RealtimeBuffer` 不同。Orocos 的设计假设是"所有组件都在 RT 上下文中"，因此默认通信方式就是 RT 安全的
2. **细粒度调度控制**：每个 TaskContext 可以绑定到不同的 Activity（不同频率、不同优先级）。ros2_control 中所有控制器共享一个 RT 主循环——如果某个控制器的 `update()` 偶尔超时，会影响所有其他控制器
3. **无 ROS 依赖**：Orocos RTT 可以独立于 ROS 运行，适合没有 ROS 的嵌入式场景

**为什么 ros2_control 逐渐取代了 Orocos**：

1. **生态系统**：ros2_control 与 MoveIt2、nav2、Gazebo 深度集成——Orocos 的孤立生态无法竞争
2. **学习曲线**：Orocos 的 Deployment XML 和 scripting 语言学习成本高；ros2_control 用标准 ROS2 工具（YAML、launch 文件）
3. **社区活跃度**：Orocos 的核心开发团队小（主要是 KU Leuven），ros2_control 有 Bosch、PAL Robotics、PickNik 等公司持续投入
4. **PREEMPT_RT 主线化的影响**：Orocos 早期的价值之一是封装了 RT 线程管理的复杂性。PREEMPT_RT 主线化后，直接用 POSIX RT API 的门槛大幅降低，Orocos 的这一优势减弱

> **跨领域类比**：Orocos RTT 之于 ros2_control，类似于 CMake 之于 Bazel。前者功能强大、概念完备，但生态孤立、学习曲线陡峭；后者利用现有生态的网络效应，用"够用"的功能覆盖 90% 场景。两者在特定场景下都有存在价值：需要超细粒度 RT 调度的工业系统可能仍需 Orocos，正如超大规模 monorepo 仍需 Bazel。但对大多数机器人项目而言，ros2_control 是更务实的选择。这个类比的局限在于：CMake 和 Bazel 可以共存于同一项目，但 Orocos 和 ros2_control 很难在同一控制栈中混用。

### 练习

1. ⭐⭐ 精读 `ros2_control_node.cpp` 主循环。标注 SCHED_FIFO、nanosleep、read/update/write。
2. ⭐⭐⭐ 用 `CLOCK_MONOTONIC` 测量 read+update+write 执行时间。运行 10000 次画直方图。
3. ⭐⭐⭐ 对比 Orocos RTT 的 `updateHook()` 和 ros2_control 的 `update()` 在 RT 安全保证上的差异。哪种框架更容易让初学者犯 RT 安全错误？

---

## 11. ros2_control 实时执行器的线程模型 ⭐⭐

### 11.1 线程架构全景

ros2_control 的线程模型是理解 ROS2 实时控制的关键。

```
┌─── ros2_control 线程模型 ───────────────────────┐
│                                                   │
│  ┌── RT 线程 (SCHED_FIFO, 优先级 80) ──────────┐  │
│  │                                              │  │
│  │  while(running) {                            │  │
│  │    hardware_interface->read(t, dt);          │  │
│  │    for (ctrl : active_controllers)           │  │
│  │      ctrl->update(t, dt);                    │  │
│  │    hardware_interface->write(t, dt);          │  │
│  │    clock_nanosleep(MONOTONIC, ABSTIME, ...); │  │
│  │  }                                           │  │
│  └──────────────────────────────────────────────┘  │
│       ↑ RealtimeBuffer     ↓ RealtimePublisher     │
│  ┌── 非 RT 线程 (executor) ─────────────────────┐  │
│  │  - ROS2 subscription callbacks               │  │
│  │  - service handlers                          │  │
│  │  - parameter updates                         │  │
│  │  - lifecycle state transitions               │  │
│  └──────────────────────────────────────────────┘  │
│       ↑                     ↓                      │
│  ┌── 非 RT 线程 (DDS) ──────────────────────────┐  │
│  │  - 网络收发                                    │  │
│  │  - 序列化/反序列化                              │  │
│  │  - QoS 管理                                   │  │
│  └──────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────┘
```

**关键设计原则**：

| 原则 | 实现方式 |
|------|---------|
| RT 线程不做任何 ROS2 通信 | 通过 RealtimeBuffer 中转 |
| 所有内存在 configure 阶段预分配 | `on_configure()` 中创建所有对象 |
| update() 中零分配 | 固定大小 Eigen + std::array |
| 控制器之间不直接通信 | 通过 state_interfaces/command_interfaces |

### 11.2 控制器生命周期与 RT 安全

ros2_control 的控制器遵循 ROS2 Lifecycle：

```
  Unconfigured → on_configure() → Inactive → on_activate() → Active
                 [非 RT: 可 malloc]         [非 RT: 可 malloc]
                                                  │
                                                  ▼
                                           update() [RT!]
                                           read() [RT!]
                                           write() [RT!]
```

**规则**：`on_configure()` 和 `on_activate()` 在非 RT 线程中执行——这里可以做所有的内存分配、ROS2 订阅创建、参数读取。`update()` 在 RT 线程中执行——禁区清单全部适用。

```cpp
class MyController : public controller_interface::ControllerInterface {
    // 预分配的 RT 安全数据结构
    std::array<double, 7> joint_positions_{};
    std::array<double, 7> joint_commands_{};
    realtime_tools::RealtimeBuffer<std::array<double, 7>> target_buffer_;
    rclcpp::Subscription<JointCommand>::SharedPtr cmd_sub_;  // 必须是类成员，否则出作用域即销毁

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override
    {
        // ✅ 非 RT 阶段: 可以做分配
        cmd_sub_ = get_node()->create_subscription<JointCommand>(
            "/commands", 10,
            [this](const JointCommand::SharedPtr msg) {
                std::array<double, 7> cmd;
                std::copy_n(msg->data.begin(), 7, cmd.begin());
                target_buffer_.writeFromNonRT(cmd);
            });
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override
    {
        // !! RT 线程 !!
        auto cmd = *target_buffer_.readFromRT();  // 无锁读
        for (size_t i = 0; i < 7; ++i) {
            command_interfaces_[i].set_value(cmd[i]);
        }
        return controller_interface::return_type::OK;
    }
};
```

### 11.3 hardware_interface 的 RT 要求

`hardware_interface::SystemInterface` 的 `read()` 和 `write()` 在 RT 线程中被调用。硬件接口实现者必须保证这两个函数 RT 安全：

| 允许 | 禁止 |
|------|------|
| 直接读写 EtherCAT/UDP 缓冲区 | 打开/关闭网络连接 |
| `memcpy` 固定大小数据 | 日志输出 |
| 原子状态标志更新 | 异常抛出 |
| 预分配缓冲区的索引操作 | 动态容器操作 |

### 练习

1. ⭐⭐ 阅读 `ros2_control` 源码中 `controller_manager.cpp` 的主循环。画出从 `read()` 到 `write()` 的完整数据流。
2. ⭐⭐ 实现一个最小的 `hardware_interface::SystemInterface`，用共享内存模拟硬件。确保 `read()/write()` 零分配。

---

## 12. 实时调试方法论——ftrace / perf / trace-cmd ⭐⭐⭐

### 12.1 为什么 GDB 在 RT 调试中不够用

GDB 断点会**暂停整个进程**——包括 RT 线程。在断点处暂停超过 1 ms 就触发通信超时。GDB 适合调试逻辑错误，但不适合调试延迟问题。

**RT 调试的正确工具**：非侵入式跟踪（tracing）——不暂停进程，只记录事件时间戳。

### 12.2 ftrace——内核级函数跟踪

ftrace 是 Linux 内核内置的跟踪框架，零额外安装。

```bash
# === Step 1: 启用 ftrace ===
echo 0 > /sys/kernel/debug/tracing/tracing_on
echo function_graph > /sys/kernel/debug/tracing/current_tracer

# 只跟踪 RT 线程相关的内核函数
echo 'schedule' > /sys/kernel/debug/tracing/set_ftrace_filter
echo 'try_to_wake_up' >> /sys/kernel/debug/tracing/set_ftrace_filter
echo 'hrtimer_interrupt' >> /sys/kernel/debug/tracing/set_ftrace_filter

# 只跟踪特定 PID
echo $RT_THREAD_PID > /sys/kernel/debug/tracing/set_ftrace_pid

echo 1 > /sys/kernel/debug/tracing/tracing_on

# === Step 2: 运行控制器一段时间 ===
# ...

# === Step 3: 读取跟踪结果 ===
echo 0 > /sys/kernel/debug/tracing/tracing_on
cat /sys/kernel/debug/tracing/trace > ftrace_output.txt
```

**ftrace 输出解读**：

```
# tracer: function_graph
#
#  TIME     CPU  DURATION                  FUNCTION
# |          |   |   |                     |   |
 1234.567 |  2)   0.523 us | schedule();
 1234.568 |  2)   0.112 us | try_to_wake_up();
 1235.567 |  2)   0.498 us | schedule();     ← 1.000 ms 间隔: 完美!
 1235.568 |  2)   0.108 us | try_to_wake_up();
 1236.567 |  2)   0.534 us | schedule();
 1236.568 |  2)  47.231 us | schedule();     ← 异常! 调度延迟 47 us
```

### 12.3 perf——性能事件采样

`perf` 可以定位 RT 线程中的热点函数和延迟来源。

```bash
# 采样 RT 线程的 CPU 周期分布
sudo perf record -g -p $RT_PID -F 10000 -- sleep 10
sudo perf report

# 跟踪调度延迟
sudo perf sched record -- sleep 10
sudo perf sched latency
# 输出每个线程的调度延迟统计:
# Task          | Runtime   | Switches | Avg delay | Max delay
# controller    |  8234 ms  |  10000   |   5.2 us  |  23.1 us  ← 合格
# logger        |  1523 ms  |  5000    |  12.3 us  |  89.2 us

# 检测缓存未命中 (影响确定性)
sudo perf stat -e cache-misses,cache-references \
    -p $RT_PID -- sleep 5
```

### 12.4 trace-cmd——ftrace 的高级前端

`trace-cmd` 是 ftrace 的用户友好封装，支持图形化分析（配合 KernelShark）。

```bash
# 安装
sudo apt install trace-cmd kernelshark

# 记录: 调度事件 + 中断 + wakeup
sudo trace-cmd record -e sched -e irq -e timer \
    -P $RT_PID -o trace.dat

# 图形化分析
kernelshark trace.dat

# 文本分析: 找最大调度延迟
trace-cmd report trace.dat | grep "sched_switch" | \
    awk '{print $NF}' | sort -n | tail -10
```

### 12.5 实时调试决策树

```
延迟问题?
│
├── 偶发尖峰 (99% 正常, 1% 超时)
│   ├── 检查 CPU governor → performance?
│   ├── 检查 isolcpus → RT 核心隔离?
│   ├── ftrace 查看中断 → 网络/USB 中断占用?
│   └── perf sched → 被谁抢占?
│
├── 周期性延迟 (每 N 秒出现)
│   ├── 检查 RCU callback → rcu_nocbs?
│   ├── 检查 kworker → 磁盘刷新?
│   └── 检查 timer → 内核定时器?
│
└── 持续高延迟 (平均都慢)
    ├── EIGEN_RUNTIME_NO_MALLOC → 有堆分配?
    ├── LD_PRELOAD malloc 拦截 → 非 Eigen 堆分配?
    ├── perf record → 热点函数是什么?
    └── 算法本身太慢? → 优化或异步化
```

### 12.6 实战调试案例 ⭐⭐⭐

以下三个案例来自真实的机械臂 RT 开发场景，展示如何用上述工具链定位根因。

**案例 1：每 30 秒一次 200 us 延迟尖峰**

- **症状**：cyclictest 压力测试下，每 30 秒出现一次 200 us 尖峰，其余时间 < 20 us
- **排查过程**：
  1. ftrace 追踪 `sched_switch` 事件——发现尖峰时刻 RT 线程被 `kworker` 抢占
  2. 进一步追踪 `kworker` 的调用栈——指向 `writeback_sb_inodes`（磁盘脏页回刷）
  3. 检查 `/proc/sys/vm/dirty_writeback_centisecs`——默认 500（即 5 秒触发一次 writeback），但 writeback 的实际执行间隔受脏页量影响
- **根因**：系统日志（syslog/journald）持续写磁盘，积累的脏页触发周期性回刷。回刷的 `kworker` 线程优先级高于某些内核线程，间接延迟了 RT 线程的中断响应
- **修复**：`echo 0 > /proc/sys/vm/dirty_writeback_centisecs`（禁用周期回刷），同时将日志输出重定向到 tmpfs 或 ramdisk

**案例 2：Pinocchio RNEA 偶发 500 us 延迟**

- **症状**：控制循环中调用 `pinocchio::rnea()` 偶尔耗时 500 us（正常 2-3 us）
- **排查过程**：
  1. `EIGEN_RUNTIME_NO_MALLOC` 审计——未触发 abort，说明 Eigen 没有堆分配
  2. `LD_PRELOAD` malloc 拦截——捕获到 `pinocchio::Data` 内部的一次 `realloc`
  3. GDB backtrace 定位——`pinocchio::Data::oMi` 向量在首次调用时被 lazy-initialized
- **根因**：Pinocchio 的 `Data` 结构中某些成员采用延迟初始化（lazy initialization），首次 `rnea()` 调用触发内部缓冲区分配。后续调用复用已分配缓冲区
- **修复**：在非 RT 阶段（`on_configure()`）预调用一次 `pinocchio::rnea()` "预热" Data 结构，确保所有延迟初始化在 RT 循环前完成

**案例 3：ARM 平台（Jetson Orin）上无锁队列数据损坏**

- **症状**：SPSC Ring Buffer 在 Jetson Orin（ARM Cortex-A78AE）上偶发数据损坏，x86 开发机上从未出现
- **排查过程**：
  1. 检查 `memory_order`——`push()` 中 `head_` 的 store 使用了 `memory_order_relaxed`
  2. 在 x86（TSO 内存模型）上 relaxed store 等价于 release store——ARM 上不等价
  3. 消费者 `pop()` 可能在 ARM 上看到更新后的 `head_` 但读到**未完全写入的** `buffer_[head]`
- **根因**：ARM 的弱序内存模型允许 store 重排——`buffer_[head] = item` 可能在 `head_.store(next)` **之后**对其他核心可见
- **修复**：将 `head_.store(next, std::memory_order_relaxed)` 改为 `head_.store(next, std::memory_order_release)`，保证 buffer 写入在 head 更新之前对消费者可见

> **"不是X而是Y"句式**：这类跨平台 bug 不是"代码写错了"——在 x86 上它是完全正确的。这是"内存模型假设不匹配"——代码隐含地依赖了 x86 的强序保证，而 ARM 不提供该保证。教训：无锁代码必须在目标平台上测试，不能只在开发机上验证。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：在 RT 线程中使用 `printf` 做"临时调试"
>
> **错误做法**：延迟异常时插入 `printf("latency = %f\n", dt)` 调试
>
> **现象**：插入 printf 后延迟更严重（甚至本来正常的也变异常）
>
> **根本原因**：`printf` 自身涉及 mutex + 缓冲区分配 + 系统调用，可能引入 100-1000 us 延迟。你在用"导致问题的工具"来调试"这个问题"——海森堡 bug
>
> **正确做法**：用原子变量记录时间戳，非 RT 线程异步读取并打印。或用 ftrace/perf 非侵入式跟踪

### 练习

1. ⭐⭐ 用 ftrace 跟踪你的 RT 线程 10 秒。找到最大调度延迟。分析是什么内核事件导致的。
2. ⭐⭐⭐ 用 `perf sched` 分析你的控制循环。对比有无 `isolcpus` 时的调度延迟分布。
3. ⭐⭐ 实现一个非侵入式延迟记录器：RT 线程中只做 `clock_gettime` + 写入预分配环形缓冲区，非 RT 线程读取并输出直方图。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "用 C++ 就自动是实时的" | C++ 的平均性能高，但 `new/delete/mutex/cout` 的最坏延迟不确定。实时需要消除所有不确定性来源 |
| "PREEMPT_RT 让代码跑得更快" | PREEMPT_RT 不改变平均速度，只保证最坏情况延迟有界。RT 内核下代码的平均性能甚至可能略降（更多抢占点的开销） |
| "实时 = 低延迟" | 低延迟是平均值小，实时是最坏值有界。一个平均 10 us 但最坏 5 ms 的函数不是实时安全的 |
| "EtherCAT 自动保证实时" | EtherCAT 只保证总线层实时。主站软件仍需 PREEMPT_RT + SCHED_FIFO 保证调度确定性 |
| "`std::shared_ptr` 是安全的" | `shared_ptr` 的引用计数归零时触发 `delete` → `free`，这在 RT 线程中是不确定性来源 |
| "`std::vector` 预留够容量就安全" | 首次 `reserve()` 本身就是 `malloc`。即使容量足够，某些操作（如 `insert`）仍可能触发重分配。用 `std::array` 或 `std::pmr::vector` + 预分配池 |
| "只要代码 < 1 ms 就不会超时" | libfranka 的 UDP 通信本身占 200 us，留给用户计算的只有 300-500 us |
| "`memory_order_relaxed` 在 x86 上没问题" | x86 的 TSO 模型恰好掩盖了 relaxed 的不安全性。部署到 ARM（Jetson）后会出现数据竞争 |
| "printf 可以临时用来调试延迟问题" | `printf` 自身引入 100-1000 us 延迟，用"导致问题的工具"调试该问题——海森堡 bug |
| "RT 线程用 `std::mutex` 保护数据就行" | `std::mutex` 不支持优先级继承，会导致优先级反转。用 `RealtimeBuffer` 或 `std::atomic` |
| "Orocos RTT 比 ros2_control 更好" | Orocos 的 RT 隔离更彻底，但生态孤立。对大多数项目，ros2_control 的生态优势远大于 Orocos 的 RT 细粒度优势 |
| "PMR 内存池解决了所有动态分配问题" | PMR 需要 `null_memory_resource` 作上游防止回退到 `malloc`；缓冲区大小必须覆盖最坏情况；仍需在非 RT 阶段预分配 |

---

## 本章小结

### 知识点总表

| 知识点 | 核心内容 | 难度 | 关键收获 |
|--------|---------|------|---------|
| SLAM C++ 不够用 | malloc/cout/mutex 在 RT 中危险 | ⭐ | 建立"RT 不安全"直觉 |
| PREEMPT_RT 主线化 | Linux 6.12 合并，编译实战，cyclictest 验证 | ⭐ | 20 年里程碑 |
| RT 禁区清单 | 禁止/允许/灰色操作列表 | ⭐ | 必须背诵 |
| 标准三件套 | SCHED_FIFO + mlockall + EIGEN_NO_MALLOC | ⭐⭐ | RT 环境标准配置 |
| pmr 内存池 | 预分配缓冲区 + monotonic_buffer_resource | ⭐⭐⭐ | RT 中使用动态容器 |
| 内存分配检测 | EIGEN_NO_MALLOC + LD_PRELOAD malloc 拦截 | ⭐⭐ | 彻底消除隐藏分配 |
| realtime_tools | RealtimeBuffer + RealtimePublisher + LockFreeQueue | ⭐⭐ | 无锁 RT-非RT 通信 |
| Lock-Free 数据结构 | SPSC Ring Buffer + Triple Buffer | ⭐⭐ | RT 安全的队列/缓冲 |
| 优先级反转 | 问题定义 + 优先级继承 | ⭐⭐ | 经典 RT 问题 |
| libfranka RT | callback 模式 + 完整 walkthrough | ⭐⭐ | 工业级 RT API |
| EtherCAT | DC 同步 + PDO 映射 + SOEM | ⭐⭐⭐ | 工业通信协议 |
| 足式对比 | 架构差异 + 安全差异 + 调试差异 | ⭐⭐ | 跨方向迁移 |
| ros2_control 线程模型 | 生命周期 + RT/非RT 分离 | ⭐⭐ | ROS2 控制架构核心 |
| Orocos RTT | ros2_control 之外的替代框架 | ⭐⭐⭐ | 框架选型视野 |
| RT 调试方法论 | ftrace / perf / trace-cmd | ⭐⭐⭐ | 非侵入式延迟分析 |

### 术语速查表

| 术语 | 英文 | 一句话定义 |
|------|------|-----------|
| 硬实时 | Hard Real-Time | 最坏情况执行时间有确定上界，超过 deadline 即为系统失败 |
| 确定性 | Determinism | 每次执行的时间偏差在已知范围内 |
| PREEMPT_RT | Fully Preemptible Kernel | Linux 内核实时补丁（6.12+ 已主线化），使几乎所有内核代码可被抢占 |
| SCHED_FIFO | First-In First-Out Scheduling | POSIX 实时调度策略，任务只被更高优先级任务抢占 |
| mlockall | Memory Lock All | 锁定进程所有当前和未来内存页，防止被换出到磁盘 |
| 优先级反转 | Priority Inversion | 高优先级任务被低优先级任务间接阻塞的现象 |
| 优先级继承 | Priority Inheritance | 持有 mutex 的低优先级线程暂时提升到等待者的优先级 |
| RealtimeBuffer | Realtime Thread-Safe Box | ros2_control 的无锁数据交换容器（最新值覆盖语义） |
| SPSC | Single-Producer Single-Consumer | 单生产者单消费者无锁队列 |
| MPMC | Multiple-Producer Multiple-Consumer | 多生产者多消费者无锁队列 |
| EtherCAT | Ethernet for Control Automation Technology | 工业实时以太网协议，保证微秒级确定性通信 |
| DC 同步 | Distributed Clock Synchronization | EtherCAT 从站间亚微秒级时钟同步机制 |
| PDO | Process Data Object | EtherCAT 周期性数据交换的数据映射单元 |
| SOEM | Simple Open EtherCAT Master | 轻量级开源 EtherCAT 主站库（C 语言） |
| PMR | Polymorphic Memory Resource | C++17 多态内存资源，将分配策略从容器类型中解耦 |
| ftrace | Function Tracer | Linux 内核内置的函数跟踪框架 |
| cyclictest | Cyclic Latency Test | RT 延迟测试标准工具，测量周期性唤醒的最坏延迟 |
| False Sharing | 伪共享 | 不同核心写同一缓存行的不同变量，导致反复缓存失效 |
| SSO | Small String Optimization | `std::string` 短字符串直接存储在对象内部（通常 16-23 字节），避免堆分配 |
| Orocos RTT | Open Robot Control Software Real-Time Toolkit | 比利时 KU Leuven 开发的实时机器人控制框架 |

---

## 累积项目：本章新增模块

**项目名称**：从零构建 7-DOF 机械臂控制栈

| 章节 | 新增模块 | 功能 |
|------|---------|------|
| M01 | URDF 加载 + FK/Jacobian | Pinocchio 基础设施 |
| M05 | QP 求解器层 | 瞬态 IK QP |
| M08 | 轨迹优化 | NLP 轨迹规划 |
| M10 | 时间参数化层 | 路径 → 轨迹 |
| **M11（本章）** | **实时控制层** | **1 kHz RT 循环 + 无锁通信** |

---

## 延伸阅读

| 资源 | 内容 | 难度 |
|------|------|------|
| ros2_control realtime_tools 源码 | 无锁 RT 工具 | ⭐⭐ |
| libfranka 实时示例 | callback RT 控制 | ⭐⭐ |
| PREEMPT_RT 官方 wiki | RT Linux 配置 | ⭐ |
| cyclictest + rt-tests | RT 延迟测量 | ⭐ |
| Jeff Preshing, *Lock-Free Programming* 博客系列 | 无锁编程深度 | ⭐⭐⭐ |
| C++ Concurrency in Action (Williams, 2019) | Ch5-7 原子与内存序 | ⭐⭐⭐ |
| Linux kernel `Documentation/scheduler/` | 调度器设计 | ⭐⭐⭐ |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| cyclictest max > 500 us | 非 RT 内核 / CPU 频率波动 | 1. `uname -a` 确认 RT 内核 2. 检查 governor 3. 检查 isolcpus | M11.2-4 |
| libfranka `communication_constraints_violation` | 回调超时 | 1. 测量回调执行时间 2. 检查堆分配 3. 确认直连以太网 | M11.7 |
| `EIGEN_RUNTIME_NO_MALLOC` abort | 动态 Eigen 类型 | 1. GDB backtrace 2. 改为固定大小 | M11.4 |
| RT 优先级设置失败 | 权限不足 | 1. sudo 2. `setcap cap_sys_nice+ep` 3. `/etc/security/limits.conf` | M11.4 |
| RealtimePublisher 频率低 | DDS 发布耗时 | 1. 降低发布频率 2. 减小消息体积 3. 检查 DDS 配置 | M11.5 |

---

## 跨章综合练习 ⭐⭐⭐

**题目**：综合 M01（Pinocchio）+ M05（QP）+ M10（Ruckig）+ M11（实时 C++），实现完整实时控制栈：

1. PREEMPT_RT 环境配置（SCHED_FIFO + mlockall + isolcpus）
2. 1 kHz RT 循环中：
   - 读取仿真关节状态
   - Pinocchio 计算 Jacobian + 重力补偿（固定大小 Eigen）
   - ProxQP 求解 IK QP
   - Ruckig 在线轨迹平滑
   - RealtimeBuffer 接收目标位姿
   - RealtimePublisher 发布关节状态
3. `EIGEN_RUNTIME_NO_MALLOC` 审计全循环无堆分配
4. cyclictest 验证最坏延迟 < 50 us
5. 测量单次循环总时间 < 300 us

**评分标准**：零堆分配、cyclictest < 50 us、循环 < 300 us、目标切换时轨迹连续。

---

## 本章与后续章节的关系

| 后续章节 | 关系 | 本章铺垫的知识点 |
|---------|------|----------------|
| **M12 ros2_control 硬件接口** | 直接后继 | RT 线程模型（§11）、RealtimeBuffer（§5）、hardware_interface 的 read/write RT 要求（§11.3）。M12 在 M11 的基础上实现具体的硬件驱动——本章教"RT 循环怎么跑"，M12 教"循环里怎么和硬件通信" |
| **M14 MoveIt2 系统集成** | 上下文依赖 | MoveIt2 的轨迹规划结果通过 RealtimeBuffer 传入控制循环。理解 RT 与非 RT 的边界（§5-6）是集成 MoveIt2 和 ros2_control 的前提 |
| **足式/170 实时 C++ 工程** | 对照章节 | 本章 §9 的对比内容直接对接足式章节。RT 配置三件套、realtime_tools 使用、Eigen 固定大小习惯完全复用；差异在于架构（同步 vs 异步）和安全标准 |

---

## API 速查表

### POSIX RT API

| API | 头文件 | 用途 | RT 线程中可调用 |
|-----|--------|------|:---:|
| `sched_setscheduler(pid, SCHED_FIFO, &param)` | `<sched.h>` | 设置实时调度策略和优先级 | 仅初始化阶段 |
| `mlockall(MCL_CURRENT \| MCL_FUTURE)` | `<sys/mman.h>` | 锁定所有当前和未来内存页 | 仅初始化阶段 |
| `clock_gettime(CLOCK_MONOTONIC, &ts)` | `<time.h>` | 读取单调递增时钟 | ✅ |
| `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL)` | `<time.h>` | 绝对时间高精度睡眠 | ✅ |
| `pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT)` | `<pthread.h>` | 启用优先级继承 mutex | 仅初始化阶段 |
| `pthread_attr_setstacksize(&attr, size)` | `<pthread.h>` | 设置线程栈大小 | 仅初始化阶段 |

### realtime_tools API

| 类 | 关键方法 | 语义 | RT 端行为 |
|----|---------|------|----------|
| `RealtimeBuffer<T>` / `RealtimeThreadSafeBox<T>` | `writeFromNonRT(val)` / `readFromRT()` | 最新值覆盖 | 非阻塞读取 |
| `RealtimePublisher<T>` | `trylock()` / `unlockAndPublish()` | RT 安全 ROS2 发布 | trylock 失败即跳过 |
| `LockFreeSPSCQueue<T>` | `push(val)` / `pop(val)` | FIFO 单生产者单消费者 | 非阻塞 |
| `LockFreeMPMCQueue<T>` | `push(val)` / `pop(val)` | FIFO 多生产者多消费者 | 非阻塞 |

### Eigen RT 安全 API

| 操作 | RT 安全 | 条件 |
|------|:-------:|------|
| `Matrix<double, N, M>` 构造 | ✅ | N, M 为编译期常量 |
| `MatrixXd` 构造 | ❌ | 触发 `malloc` |
| `Map<const VectorNd>(ptr)` | ✅ | 零拷贝指针包装 |
| `A * B`（固定大小） | ✅ | 栈上临时变量 |
| `A.inverse()`（固定大小） | ✅ | 栈上 |
| `A.inverse()`（动态大小） | ❌ | 触发 `malloc` |
| `set_is_malloc_allowed(false)` | - | 审计模式：任何 Eigen 堆分配立即 abort |

### libfranka RT 回调 API

| 方法 | 返回类型 | 说明 |
|------|---------|------|
| `robot.control(callback)` | void | 启动 1 kHz RT 控制循环 |
| `state.q` / `state.dq` | `std::array<double, 7>` | 当前关节位置/速度（RT 安全） |
| `model.gravity(state)` | `std::array<double, 7>` | 重力补偿力矩（RT 安全） |
| `model.mass(state)` | `std::array<double, 49>` | 质量矩阵（7x7 展平，RT 安全） |
| `franka::Torques(array)` | POD | RT 安全的力矩返回值 |

---

## 研究实践建议

### 入门级（⭐-⭐⭐）

1. 在 Ubuntu 上安装 RT 内核（`apt install linux-image-rt` 或手动编译），运行 cyclictest 并与非 RT 内核对比
2. 编写一个最小的 1 kHz 循环：`SCHED_FIFO` + `mlockall` + `clock_nanosleep(ABSTIME)`，用 `CLOCK_MONOTONIC` 测量每次循环时间并输出直方图
3. 在循环中故意加入 `std::vector::push_back` / `std::cout`，观察延迟尖峰

### 进阶级（⭐⭐-⭐⭐⭐）

1. 用 `EIGEN_RUNTIME_NO_MALLOC` + `LD_PRELOAD` malloc 拦截审计一个完整的 ros2_control 控制器
2. 精读 `realtime_tools` 源码，理解 `RealtimeBuffer` 的内部同步机制
3. 在 libfranka 仿真环境中实现 PD + 重力补偿控制器，测量单次回调执行时间

### 研究级（⭐⭐⭐-⭐⭐⭐⭐）

1. 对比 `monotonic_buffer_resource` 和自定义定长内存池在 RT 上下文中的分配延迟分布
2. 在 ARM 平台（如 Jetson Orin）上复现 `memory_order_relaxed` 导致的数据竞争 bug
3. 研究 Rust 的 borrow checker 能否在编译期检测 RT 线程中的 `malloc` 调用——目前 Rust 社区有 `#[no_alloc]` 属性的提案

---

## 版本信息速查

| 组件 | 版本 | 说明 |
|------|------|------|
| Linux 内核（PREEMPT_RT 主线化） | **6.12+**（2024-11-17 发布） | 6.12 起无需外部补丁 |
| Ubuntu RT 内核支持 | **25.04+**（Plucky Puffin） | `apt install linux-image-rt-generic` |
| libfranka | **0.14.x / 0.15.x** | Franka Emika/Franka Robotics 官方 C++ 库 |
| realtime_tools（ROS2 Rolling） | **2025 年重构版** | 新增 LockFreeSPSCQueue / LockFreeMPMCQueue |
| SOEM（EtherCAT 主站） | **1.4.x** | rt-labs 维护的轻量 C 库 |
| Orocos RTT | **2.9** | KU Leuven 维护 |
| Eigen | **3.4.x** | `EIGEN_RUNTIME_NO_MALLOC` 自 3.3 起可用 |
| cyclictest（rt-tests 套件） | 最新稳定版 | `apt install rt-tests` |
