# 实时 Linux 与实时安全 C++ 编程

> 本章定位：把“代码平均很快”升级为“控制线程在最坏情况下仍能按时完成”。
> 机器人实时编程关注的不是吞吐量峰值，而是延迟上界、阻塞来源、内存行为和故障降级。

## 前置自测

1. 1 kHz 控制循环的周期是多少？如果一次循环偶发耗时 2 ms，会发生什么？
2. `std::mutex` 为什么可能导致优先级反转？
3. 动态内存分配的平均耗时很低，为什么仍不适合硬实时路径？
4. `CLOCK_MONOTONIC` 与 `CLOCK_REALTIME` 的差异是什么？
5. Eigen 动态矩阵在什么情况下会分配堆内存？

## 本章目标

学完本章后，你应该能解释 PREEMPT_RT 解决了什么、没有解决什么。
你应该能写出一个基本的 1 kHz 实时循环，并明确哪些 C++ 操作不能进入实时路径。
你还应该能用 cyclictest、内存锁定、调度策略、预分配和非实时日志线程建立最小可用的实时工程边界。

### 知识树

```text
实时 Linux 与安全 C++ 编程
├── 实时性的真正含义 ⭐（deadline vs 吞吐，软/硬/工程实时）
├── PREEMPT_RT 与调度 ⭐⭐
│   ├── Linux 6.12+ 主线化（不再需要外部补丁）
│   ├── 调度策略：SCHED_FIFO / SCHED_RR / SCHED_DEADLINE
│   ├── cyclictest 测量与压力测试
│   ├── 绝对时间睡眠 vs 相对睡眠
│   └── CPU 亲和性、IRQ 隔离、nohz_full
├── 实时安全 C++ 禁区 ⭐⭐⭐
│   ├── 堆分配（new/malloc/push_back）
│   ├── I/O 与阻塞锁
│   ├── 异常与线程创建
│   ├── Eigen 动态矩阵审计
│   └── 内存锁定与预触页
├── 线程同步 ⭐⭐⭐
│   ├── 优先级反转与三层处理
│   ├── 双缓冲与 SPSC 队列
│   ├── PI mutex 与 try_lock
│   └── 非实时日志线程
├── ROS2 control 实时边界 ⭐⭐
│   ├── 生命周期阶段与 update() 实时路径
│   ├── RealtimeBuffer 思想
│   └── 实时发布器
└── 完整工程闭环 ⭐⭐⭐
    ├── 优先级层级设计
    ├── 时间账本与周期测量
    ├── 故障案例复盘
    └── 实时安全判断标准
```

---

## 2.1 实时性的真正含义 ⭐

这一节解决的问题是：为什么“平均耗时 0.2 ms”仍可能不是实时控制代码。

### 动机：机器人控制看最坏情况

普通后端服务常看平均延迟和吞吐量。
机器人控制线程看 deadline。
1 kHz 控制意味着每 1 ms 必须读状态、计算控制量、发送命令。
其中任何一次超过周期，都可能造成旧命令重复发送或新命令来不及生效。

| 指标 | 普通计算 | 实时控制 |
|------|----------|----------|
| 关注点 | 平均吞吐、平均延迟 | 最大延迟和抖动 |
| 偶发慢请求 | 可以重试或排队 | 可能导致失稳 |
| 资源策略 | 充分利用 CPU | 保留预算和隔离干扰 |
| 正确性 | 最终结果正确 | 结果必须准时到达 |

实时性可以类比乐队节拍。
一个鼓手平均节奏准确没有意义。
只要关键拍子延迟，整首曲子就会乱。
控制线程也是如此：准时性是语义的一部分。

### 反面失败：空载测试通过，压力下摔倒

许多控制程序在空载桌面上运行良好。
一旦同时打开日志、网络、可视化和磁盘写入，延迟尖峰会突然出现。
非 RT 内核在压力下可能出现数百微秒甚至毫秒级调度延迟。
对 1 kHz 控制来说，1 ms 延迟已经等于错过整个周期。

如果只在空载下测试，就像只在实验室平地验证足式机器人。
真正上机时，负载、网络中断、页面缺失和锁竞争都会成为扰动。

> 本质洞察：实时系统不是“跑得快的系统”，而是“能证明或测量出最坏情况下仍按时完成的系统”。
> 平均性能属于性能优化，延迟上界属于实时正确性。

### 软实时、硬实时与机器人常见边界

| 类别 | 含义 | 机器人例子 | 失败后果 |
|------|------|------------|----------|
| 软实时 | 偶发错过 deadline 可接受 | UI 可视化、地图刷新 | 画面卡顿 |
| 准实时 | 少量错过可降级 | 局部规划、MPC 重规划 | 跟踪变差 |
| 硬实时 | 不允许错过 | 电流环、关节安全保护 | 硬件风险 |
| 工程实时 | 通过测量和保守设计满足平台需求 | 500 Hz WBC、1 kHz 状态回调 | 失稳或停机 |

大多数 Linux 机器人控制属于工程实时。
PREEMPT_RT 能显著降低调度延迟，但仍需要工程纪律。
实时安全 C++ 不是语言自动保证，而是约束代码路径。

### 练习

1. 对一个 1 kHz 控制器，列出读状态、估计、控制、发送四段预算，要求总和不超过 700 us。
2. 解释为什么 p99 延迟不等于最大延迟，真实上机时两者都要记录。
3. 用“排队买票”的类比解释实时系统中 deadline 和吞吐量的区别。

---

## 2.2 PREEMPT_RT、调度与测量 ⭐⭐

这一节解决的问题是：Linux 实时内核能提供什么基础设施，以及怎样测量它是否足够。

### PREEMPT_RT 的作用与 Linux 6.12+ 主线化

PREEMPT_RT 将更多内核路径变为可抢占，把许多自旋锁转换为可睡眠锁，并改善中断线程化。
它的核心改动包括：

| 改动 | 效果 | 对机器人的意义 |
|------|------|----------------|
| 自旋锁转可睡眠锁 | 高优先级线程不必等锁持有者 | 控制线程被内核路径阻塞的概率降低 |
| 中断线程化 | 硬件中断变成可调度线程 | 中断不再无条件打断控制线程 |
| 可抢占内核路径增多 | 内核代码执行期间可被实时线程抢占 | 内核延迟的最坏值显著降低 |
| 高精度定时器改进 | 纳秒级定时器精度 | 1 kHz 控制循环的唤醒抖动减小 |

**Linux 6.12+ 的重要变化**：2024 年 10 月发布的 Linux 6.12 把 PREEMPT_RT 的核心补丁合入了主线内核。
这意味着不再需要从独立仓库下载和维护 RT 补丁。
Ubuntu 24.04 LTS 的后续 HWE 内核和其他主流发行版将逐步默认支持 `PREEMPT_RT` 配置。

这对机器人工程师的实际影响：

1. **不再需要手动打 RT 补丁**：之前使用 PREEMPT_RT 需要下载对应内核版本的补丁、编译自定义内核。现在可以直接使用发行版提供的 RT 内核包。
2. **版本跟踪更简单**：RT 补丁不再有独立版本号，直接跟随 Linux 主线版本。
3. **硬件支持更好**：主线化意味着更多驱动和子系统与 RT 兼容，减少外设不工作的风险。
4. **仍需配置**：合入主线不等于默认启用。需要在内核配置中选择 `CONFIG_PREEMPT_RT=y`，或使用发行版的 `-rt` 内核包。

```bash
# 检查当前内核是否启用 PREEMPT_RT
uname -a  # 看是否包含 PREEMPT_RT 或 -rt 标志

# 在 Ubuntu 上安装 RT 内核（如果发行版提供）
# sudo apt install linux-image-*-rt

# 检查内核抢占模式
cat /sys/kernel/realtime  # 如果存在且输出 1，表示 RT 模式
```

> **本质洞察**：PREEMPT_RT 主线化是生态事件，不是技术事件。
> 它不会让你的控制代码变得更实时。
> 它只是让获取和维护 RT 内核变得更容易。
> 实时安全仍然需要工程师在代码中保证。

但 PREEMPT_RT 不是魔法。
它不能修复你在实时线程里打印日志。
它不能让动态内存分配变成确定耗时。
它也不能让错误的锁顺序变安全。

### SCHED_DEADLINE：基于预算的实时调度

除了传统的 `SCHED_FIFO` 和 `SCHED_RR`，Linux 还提供 `SCHED_DEADLINE` 调度策略。
它基于 Earliest Deadline First（EDF）算法，允许你为线程声明三个参数：

| 参数 | 含义 | 例子 |
|------|------|------|
| runtime | 每个周期需要多少 CPU 时间 | 500 us |
| deadline | 必须在多长时间内完成 | 1 ms |
| period | 多久触发一次 | 1 ms |

`SCHED_DEADLINE` 的优势是调度器可以做准入控制：在设置调度参数时就拒绝不可满足的任务组合。
它不需要手动设定优先级数字，而是让调度器根据 deadline 自动决定谁先运行。

但 `SCHED_DEADLINE` 在机器人控制中使用不如 `SCHED_FIFO` 普遍，原因包括：

1. 工具链和库支持较少。
2. 与 ROS2 和 ros2_control 的集成需要额外工作。
3. 准入控制可能拒绝合法但峰值较高的任务。
4. 调试和监控工具不如 FIFO 成熟。

对于教学和大多数工程项目，`SCHED_FIFO` 加上合理的优先级层级仍然是最常见的选择。
`SCHED_DEADLINE` 更适合需要严格时间保证且愿意投入额外配置的嵌入式平台。

### cyclictest 的意义

`cyclictest` 测量线程按周期唤醒时的延迟。
它不是控制器本身，但能揭示调度环境的最坏情况。

```bash
sudo cyclictest -p 80 -t1 -n -i 1000 -l 100000
```

参数含义：

| 参数 | 含义 | 为什么重要 |
|------|------|------------|
| `-p 80` | 实时优先级 80 | 接近控制线程优先级 |
| `-t1` | 一个测试线程 | 先测单核基本情况 |
| `-n` | 使用 clock_nanosleep | 接近控制循环唤醒方式 |
| `-i 1000` | 1000 us 周期 | 对应 1 kHz |
| `-l 100000` | 循环次数 | 观察尾部延迟 |

压力测试应同时运行 CPU、内存、磁盘和网络负载。
没有压力的延迟数据只能说明”当前很安静”。

### cyclictest 结果解读

cyclictest 输出的关键数据是 min/avg/max 延迟。
对实时控制最重要的是 max 延迟。

| 最大延迟范围 | 含义 | 建议 |
|-------------|------|------|
| < 20 us | 非常好 | 适合 1 kHz 控制 |
| 20-50 us | 良好 | 适合大多数机器人控制 |
| 50-100 us | 可接受 | 需要预留更多安全余量 |
| 100-500 us | 偏高 | 检查 IRQ 和非 RT 干扰 |
| > 500 us | 不适合实时控制 | 需要 RT 内核或硬件更换 |

这些数字不是绝对标准。
它们取决于你的控制周期和安全余量。
对 1 ms 周期，50 us 调度延迟占 5%，通常可以接受。
对 500 us 周期（2 kHz），50 us 就占 10%，需要更谨慎。

### 压力测试方法论

只跑 cyclictest 不够。
应同时制造 CPU、内存、I/O 和网络压力。

```bash
# 终端 1：运行 cyclictest
sudo cyclictest -p 80 -t1 -n -i 1000 -l 100000 -m

# 终端 2：CPU 压力
stress-ng --cpu $(nproc) --timeout 120s

# 终端 3：内存压力
stress-ng --vm 2 --vm-bytes 512M --timeout 120s

# 终端 4：I/O 压力
stress-ng --io 4 --timeout 120s

# 终端 5：网络压力（如果适用）
iperf3 -c localhost -t 120 -P 4
```

至少运行 10 分钟。
短时间测试可能漏掉低概率尖峰。
工业实践中常运行 24 小时以上的压力测试。

### cyclictest 与应用延迟的关系

cyclictest 测量的是操作系统调度延迟。
它不测量你的应用代码耗时。
总延迟 = 调度延迟 + 应用计算耗时。

即使 cyclictest 显示 max 20 us，如果你的 QP 求解偶发 5 ms，控制仍会超时。
因此需要同时测量两者：
系统级用 cyclictest。
应用级用内部 CycleTrace 或 Tracy 标记。

### 调度策略深入

Linux 常用实时调度策略是 `SCHED_FIFO` 和 `SCHED_RR`。
控制线程通常使用 `SCHED_FIFO`。
高优先级 FIFO 线程会一直运行，直到主动阻塞或被更高优先级线程抢占。

这意味着实时线程不能写成无限忙等。
如果它不睡眠，会饿死低优先级系统任务。

`SCHED_FIFO` 和 `SCHED_RR` 的区别在于同优先级的处理方式：

| 策略 | 同优先级行为 | 适用场景 |
|------|-------------|----------|
| `SCHED_FIFO` | 先到先运行，不时间片轮转 | 大多数控制线程 |
| `SCHED_RR` | 同优先级之间时间片轮转 | 多个等优先级任务需要公平 |
| `SCHED_OTHER` | 普通 CFS 调度 | 非实时任务 |

机器人控制线程通常用 `SCHED_FIFO`，因为每个控制线程的优先级不同。
如果有多个传感器接收线程优先级相同，可以考虑 `SCHED_RR`。
但更好的做法是让每个线程有独特的优先级，避免依赖时间片轮转。

实时优先级的取值范围通常是 1 到 99。
内核中某些关键服务（如 migration 线程）可能使用很高的优先级。
因此用户实时线程不应使用 99，常用范围是 50 到 90。

```text
实时优先级经验范围

99: 内核 migration 等核心任务
90: 电机安全保护（项目中最高）
85: 传感器总线接收
80: 1 kHz 控制循环
70: 状态估计
50: MPC/局部规划
1-49: 可选实时后台任务
SCHED_OTHER: 日志、可视化、参数服务
```

> **反事实推理**：如果把控制线程和日志线程都设为同一优先级 80 的 SCHED_RR 会怎样？
> 两者会轮流获得时间片。
> 当日志线程占用时间片时，控制线程在等待。
> 如果日志线程做了文件 I/O，它的时间片可能很长。
> 控制线程就会错过 deadline。
> 这就是为什么不同功能的线程应有不同优先级。

```cpp
#include <pthread.h>
#include <sched.h>
#include <stdexcept>

void setCurrentThreadRealtime(int priority) {
    sched_param param{};
    param.sched_priority = priority;
    const int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        // 这个函数应在启动阶段调用。
        // 不要在实时循环中抛异常或打印。
        throw std::runtime_error("failed to set realtime priority");
    }
}
```

### 绝对时间睡眠 vs 相对睡眠

周期循环应使用绝对时间，而不是相对睡眠。
相对睡眠会累积漂移。
绝对睡眠把每个周期锚定到同一时间轴。

两者差异的直觉：相对睡眠像"每次做完饭再等 30 分钟做下一顿"。
如果做饭花的时间不固定，吃饭时间就会越来越晚。
绝对睡眠像"每天 7 点、12 点、18 点开饭"。
不管上一顿做了多久，下一顿的时间不变。

| 方式 | 行为 | 漂移 | 适用 |
|------|------|------|------|
| `usleep(1000)` | 相对当前时间等 1 ms | 累积 | 仅限非实时场景 |
| `clock_nanosleep(ABSTIME)` | 等到指定绝对时间 | 不累积 | 控制循环 |
| `std::this_thread::sleep_for` | 相对 | 累积 | 非实时 C++ 代码 |
| `std::this_thread::sleep_until` | 绝对 | 不累积 | 可用但精度受限 |

实时控制应使用 POSIX 的 `clock_nanosleep` 配合 `CLOCK_MONOTONIC`。
`CLOCK_MONOTONIC` 不受系统时间调整影响（例如 NTP 修正）。
`CLOCK_REALTIME` 可能在 NTP 调整时跳变，对控制循环不安全。

```cpp
#include <time.h>
#include <cstdint>

timespec addNanoseconds(timespec t, int64_t ns) {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1000000000L) {
        t.tv_nsec -= 1000000000L;
        ++t.tv_sec;
    }
    return t;
}

void runControlLoop() {
    constexpr int64_t kPeriodNs = 1000000;  // 1 ms
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (true) {
        next = addNanoseconds(next, kPeriodNs);

        // 这里执行控制计算。
        // 计算必须在 next 之前完成，否则下一次唤醒已经错过周期。
        // readState();
        // computeCommand();
        // sendCommand();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}
```

### CPU 亲和性与隔离

把控制线程固定到某个 CPU 核可以减少迁移造成的缓存扰动。
更严格的系统会用 `isolcpus`、`nohz_full` 或 cgroup 隔离控制核。
但隔离不是越多越好。
如果把所有系统任务赶到一个核心，可能造成网络和日志线程拥塞。

| 措施 | 价值 | 风险 |
|------|------|------|
| CPU affinity | 减少迁移 | 配置错误会集中负载 |
| IRQ affinity | 避免中断打断控制核 | 可能影响设备吞吐 |
| RT priority | 降低调度延迟 | 优先级过高会饿死系统 |
| memory locking | 避免缺页 | 锁太多内存会影响系统 |
| `isolcpus` | 完全隔离控制核 | 系统进程无法使用该核 |
| `nohz_full` | 控制核不收到 tick 中断 | 需要仔细配置 |

### CPU 隔离的完整配置

CPU 隔离涉及内核启动参数、IRQ 亲和性和线程亲和性三层。
只做其中一层通常不够。

**内核启动参数示例**：

```bash
# GRUB 内核参数（以 4 核 CPU 为例，隔离 CPU 2 和 3 给控制线程）
# isolcpus=2,3     : 调度器不会把普通任务放在这两个核上
# nohz_full=2,3    : 减少这两个核上的 tick 中断
# rcu_nocbs=2,3    : RCU 回调不在这两个核上执行
# irqaffinity=0,1  : 设备中断优先打到 CPU 0 和 1

GRUB_CMDLINE_LINUX="isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0,1"
```

**IRQ 亲和性设置**：

```bash
# 把所有 IRQ 迁移到非控制核（0,1），避免中断打断控制线程。
for irq in /proc/irq/*/smp_affinity_list; do
    echo "0,1" > "$irq" 2>/dev/null
done
```

**线程亲和性设置**：

```cpp
#include <pthread.h>
#include <sched.h>

void pinToCore(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    // 把当前线程固定到指定核心。
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}
```

三层配置应一起生效。
只设线程亲和性但不隔离核心，其他线程仍可能被调度到同一核。
只隔离核心但不固定线程，控制线程可能在非隔离核上运行。
只设 IRQ 亲和性但不隔离核心，内核其他任务仍可能打扰控制核。

> **反事实推理**：如果不做 CPU 隔离，控制线程在所有核心之间自由迁移。
> 每次迁移会使 L1/L2 缓存失效，控制周期的前几次内存访问变慢。
> 对 1 kHz 控制来说，一次缓存失效可能增加几微秒到几十微秒。
> 更重要的是，迁移的时机不可控，它会出现在 p99 和最大延迟中。

### 练习

1. 在同一台机器上分别测空载和压力下 cyclictest 最大延迟，记录最大值而不是只看平均值。
2. 修改示例循环，把相对睡眠换成绝对睡眠，解释漂移如何被消除。
3. 设计一个线程优先级表：电机保护、状态接收、WBC、MPC、日志分别设置多少优先级，并说明理由。

---

## 2.3 实时安全 C++ 的禁区与替代方案 ⭐⭐⭐

这一节解决的问题是：哪些常见 C++ 写法会破坏实时边界，应该如何替换。

### 实时安全 C++ 的核心原则

实时安全 C++ 的核心可以归纳为三条铁律：

1. **不分配**（No dynamic allocation）：控制路径中不调用 `malloc`、`new`、`operator new`，也不触发隐式分配（如 `std::vector` 扩容、`std::string` 拼接、异常分配）。
2. **不阻塞**（No unbounded blocking）：控制路径中不等待可能无限期占用的资源（如无超时 mutex、文件 I/O、网络 I/O）。
3. **不调用不可预测系统调用**（No unpredictable syscalls）：控制路径中不调用耗时不可控的内核服务（如 `printf`、动态库加载、信号处理）。

这三条铁律不是 C++ 语言的限制，而是实时系统对确定性的要求。
C++ 语言本身允许在任何地方分配、阻塞和调用系统服务。
实时安全要求程序员主动约束自己。

这类似无菌手术室的规则。
手术室里的每个人都有能力打喷嚏。
但规则要求他们戴口罩。
实时线程里的每段代码都有能力分配内存。
但规则要求它们不这样做。

### 禁区清单

| 操作 | 为什么危险 | 替代 |
|------|------------|------|
| `new/delete/malloc/free` | 堆分配耗时不可预测，可能触发锁 | 启动阶段预分配 |
| `std::vector::push_back` 扩容 | 可能重新分配和拷贝 | `reserve` 或固定容量容器 |
| `std::string` 拼接 | 可能分配 | 固定缓冲或非实时日志 |
| `std::cout/printf` | I/O 锁和系统调用不可预测 | 无锁队列交给日志线程 |
| `std::mutex` 无边界等待 | 可能阻塞和优先级反转 | try-lock、双缓冲、PI mutex |
| 抛异常 | 栈展开和分配不可控 | 启动阶段用异常，实时阶段用状态码 |
| 创建线程 | 内核资源分配 | 启动阶段创建线程池 |
| `std::shared_ptr` 创建/析构 | 原子引用计数和可能的堆释放 | 固定所有权或启动阶段分配 |
| `std::map/unordered_map` 插入 | 节点分配 | 预分配或固定数组查表 |
| 动态加载库 `dlopen` | 文件 I/O 和符号解析 | 启动阶段加载 |
| 日志框架的格式化 | 可能分配和 I/O | 无锁 trace 环形缓冲 |

### 常见隐式分配陷阱

许多 C++ 标准库操作会隐式分配内存。
初学者常不知道它们会触发堆操作。

| 代码 | 是否可能分配 | 说明 |
|------|-------------|------|
| `std::vector<double> v(12)` | 是 | 构造时分配 |
| `v.push_back(x)` | 可能 | 容量不足时重新分配 |
| `v[i] = x` | 否 | 只写已有元素 |
| `std::string s = "hello"` | 可能 | SSO 内短字符串不分配，长字符串分配 |
| `s += "world"` | 可能 | 容量不足时重新分配 |
| `auto p = std::make_shared<T>()` | 是 | 分配控制块和对象 |
| `Eigen::VectorXd v(n)` | 是 | 动态向量分配堆内存 |
| `Eigen::Matrix3d m` | 否 | 固定大小在栈上 |
| `Eigen::MatrixXd m(n,n)` | 是 | 动态矩阵分配堆内存 |
| `m.resize(n,n)` | 可能 | 尺寸变化时重新分配 |

判断一个操作是否安全的方法是：
用 `EIGEN_RUNTIME_NO_MALLOC` 和自定义 malloc 钩子在调试构建中运行控制循环。
任何意外分配都会触发断言，帮助定位问题。

### 动态分配的反面失败

下面的代码看似只是追加一个调试值。
如果容量不足，`push_back` 会分配新内存，复制旧元素，再释放旧内存。
这在平均情况下很快，但最坏情况不可接受。

```cpp
#include <vector>

void badRealtimeUpdate(double value) {
    static std::vector<double> history;
    // 错误：容量不足时会分配内存。
    // 即使多数周期不分配，也会在某个周期产生延迟尖峰。
    history.push_back(value);
}
```

正确思路是固定容量。

```cpp
#include <array>
#include <cstddef>

class FixedHistory {
public:
    void push(double value) {
        data_[write_index_] = value;
        write_index_ = (write_index_ + 1) % data_.size();
    }

private:
    std::array<double, 1024> data_{};  // 生命周期固定，不在循环中分配
    std::size_t write_index_ = 0;
};
```

### 实时安全容器选择

标准库容器的实时安全性差异很大。
选择容器时应问"它在我关心的操作中是否可能分配内存"。

| 容器 | 实时安全操作 | 非实时安全操作 | 替代 |
|------|-------------|----------------|------|
| `std::array<T, N>` | 所有操作 | 无 | 首选固定大小容器 |
| `std::vector<T>` | `operator[]`, `begin/end` | `push_back`（扩容时）, `resize` | 预分配 + 不扩容 |
| `std::string` | 短字符串读取 | 拼接、赋值长字符串 | 固定缓冲 `char[N]` |
| `std::map<K,V>` | `find`, `operator[]`（键存在时） | `insert`, `emplace` | 预分配数组 + 排序查找 |
| `std::unordered_map` | `find`（键存在时） | `insert`, rehash | 预分配数组 + 哈希表 |
| `std::deque<T>` | 无完全安全操作 | 几乎所有修改 | `std::array` 环形缓冲 |
| `std::list<T>` | 遍历已有节点 | `push_back`, `push_front` | 固定节点池 |

反事实地看，如果 `std::vector` 的 `push_back` 在每次调用时都分配内存（不做摊还），它在实时路径中就完全不能用。
实际上 `push_back` 大多数时候不分配，只在容量不足时才分配。
问题是"不确定性"——你无法预测哪一次会触发扩容。
实时系统要求的是"每次都一样"，而不是"平均很好"。

### 内存锁定与预触页

`mlockall` 可以避免已映射页面被换出。
但只调用 `mlockall` 不够。
程序还应在启动阶段访问将来要用的缓冲区，让页面真正建立映射。

```cpp
#include <sys/mman.h>
#include <vector>
#include <stdexcept>

void lockAndPrefaultMemory(std::vector<double>& buffer) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        throw std::runtime_error("mlockall failed");
    }

    // 预触页：逐元素写入，避免实时循环第一次访问时缺页。
    for (double& v : buffer) {
        v = 0.0;
    }
}
```

### Eigen 的实时边界

固定大小 Eigen 类型通常在栈上。
动态大小 `MatrixXd` 和 `VectorXd` 可能分配堆内存。
实时循环可以使用动态对象，但必须在循环前 `resize` 完成，并在循环内只写数值。

Eigen 的实时安全性可以分成三类：

| 类型 | 分配行为 | 实时路径中的使用建议 |
|------|----------|---------------------|
| `Matrix3d`, `Vector3d` 等固定大小 | 栈上，不分配堆 | 安全，推荐 |
| `Matrix<double, 12, 1>` 固定大小 | 栈上，不分配堆 | 安全，适合控制器 |
| `VectorXd(12)` 动态大小 | 构造时分配堆 | 启动阶段构造，循环中只写值 |
| `MatrixXd m; m.resize(n,n)` | resize 时可能分配 | 启动阶段 resize，循环中不改尺寸 |
| `(A * B).eval()` | 可能生成临时 | 视尺寸而定，大矩阵可能分配 |
| `A.block<3,3>(0,0)` | 不分配（视图） | 安全，但注意 auto 的陷阱 |

回顾第五章 Eigen 高级用法：`EIGEN_RUNTIME_NO_MALLOC` 宏可以在调试构建中检测实时路径的堆分配。
它不是性能工具，而是实时安全审计工具。

```cpp
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>

class RealtimeEigenBlock {
public:
    RealtimeEigenBlock(int n) : H_(n, n), g_(n), x_(n) {
        H_.setZero();
        g_.setZero();
        x_.setZero();
    }

    void updateNoMalloc() {
        Eigen::internal::set_is_malloc_allowed(false);
        // 以下操作只写已有内存。
        H_.diagonal().setOnes();
        g_.setConstant(0.1);
        x_.noalias() = H_ * g_;
        Eigen::internal::set_is_malloc_allowed(true);
    }

private:
    Eigen::MatrixXd H_;
    Eigen::VectorXd g_;
    Eigen::VectorXd x_;
};
```

注意：`EIGEN_RUNTIME_NO_MALLOC` 适合调试实时路径。
生产代码仍应通过结构设计避免分配，而不是依赖断言救场。

### 实时安全函数的设计模式

实时安全函数有几个共同特征：
它们返回状态码而不是抛异常。
它们操作预分配的输出缓冲而不是返回动态对象。
它们对所有失败路径都有有界耗时。

下面是一个实时安全控制函数的骨架：

```cpp
#include <Eigen/Dense>

enum class ControlResult {
    kOk,
    kStateTooOld,
    kSolverFailed,
    kTorqueSaturated
};

struct ControlContext {
    // 所有缓冲在启动阶段分配，控制循环只读写已有内存。
    Eigen::Matrix<double, 12, 1> q;
    Eigen::Matrix<double, 12, 1> dq;
    Eigen::Matrix<double, 12, 1> tau;
    Eigen::Matrix<double, 12, 1> kp;
    Eigen::Matrix<double, 12, 1> kd;
    double max_torque = 20.0;
};

ControlResult computePD(ControlContext& ctx) {
    // 固定大小矩阵运算，不分配，不抛异常，不调用系统服务。
    ctx.tau = -ctx.kp.cwiseProduct(ctx.q) - ctx.kd.cwiseProduct(ctx.dq);

    // 力矩限幅：逐元素钳位，不分配。
    bool saturated = false;
    for (int i = 0; i < 12; ++i) {
        if (ctx.tau[i] > ctx.max_torque) {
            ctx.tau[i] = ctx.max_torque;
            saturated = true;
        } else if (ctx.tau[i] < -ctx.max_torque) {
            ctx.tau[i] = -ctx.max_torque;
            saturated = true;
        }
    }

    return saturated ? ControlResult::kTorqueSaturated : ControlResult::kOk;
}
```

这段代码的关键不是 PD 控制律本身，而是三个设计决策：

1. 所有数据通过预分配的 `ControlContext` 传入传出，没有动态分配。
2. 饱和时返回状态码而不是打印或抛异常。
3. 整个函数的耗时是确定的——不依赖输入大小或外部资源。

### 两阶段设计模式：配置期 vs 运行期

实时安全 C++ 的核心架构模式是两阶段设计：

| 阶段 | 允许操作 | 不允许操作 | 生命周期 |
|------|----------|-----------|----------|
| 配置期 | 分配内存、打开文件、解析参数、创建线程、抛异常 | 输出控制命令 | 启动到激活 |
| 运行期 | 固定时间计算、写已有缓冲、读共享快照、返回状态码 | 分配、阻塞、I/O、异常 | 激活到停止 |

这和硬件设备的上电流程一致。
先检查、配置、校准，再允许运行。
不要在运行期做配置期的事。

### 非实时日志线程

日志要从实时路径移出去。
实时线程只写固定大小记录，日志线程异步格式化和输出。

```cpp
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

struct LogRecord {
    int64_t timestamp_ns;
    double value;
    int code;
};

class SpscLogRing {
public:
    bool push(const LogRecord& record) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = (head + 1) % buffer_.size();
        if (next == tail_.load(std::memory_order_acquire)) {
            // 队列满时丢弃日志，而不是阻塞控制线程。
            return false;
        }
        buffer_[head] = record;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(LogRecord* record) {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        *record = buffer_[tail];
        tail_.store((tail + 1) % buffer_.size(), std::memory_order_release);
        return true;
    }

private:
    std::array<LogRecord, 4096> buffer_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};
```

> 本质洞察：实时线程中的“失败策略”应优先保护控制周期。
> 日志可以丢，统计可以少一次，非关键可视化可以跳帧；控制命令不能因为这些辅助功能阻塞。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 循环里 `push_back` | 偶发 100 us 以上尖峰 | vector 扩容分配 | 固定容量或预留 |
| 编程 | `std::cout` 调试控制量 | 控制周期随机超时 | I/O 锁和系统调用 | 环形队列异步日志 |
| 概念 | 认为固定大小 Eigen 永远安全 | 栈过大导致问题 | 大固定矩阵会占大量栈 | 大矩阵预分配到对象 |
| 思维 | 只在 Debug 下测实时 | Release 后行为不同 | 优化和断言改变路径 | 两种构建都测，以上机配置为准 |

### 练习

1. 写一个固定容量环形缓冲，要求 push 在满队列时返回 false，不能阻塞。
2. 在 Eigen 代码中启用 `EIGEN_RUNTIME_NO_MALLOC`，故意在实时路径调用 `resize`，观察断言。
3. 把一个直接 `printf` 的控制循环改成异步日志结构，说明丢日志策略为什么合理。

---

## 2.4 线程同步、优先级反转与数据交换 ⭐⭐⭐

这一节解决的问题是：控制线程如何和估计、规划、日志线程交换数据而不被阻塞。

### 优先级反转：从 Mars Pathfinder 到机器人控制

优先级反转发生在高优先级线程等待低优先级线程持有的锁时。
如果中优先级线程持续运行，低优先级线程拿不到 CPU 释放锁，高优先级线程就被间接饿死。

这不是理论问题。
历史上最著名的优先级反转事故发生在 1997 年 Mars Pathfinder 任务中。
火星探测器的高优先级总线管理任务被低优先级数据收集任务阻塞。
中间优先级的通信任务持续运行，导致低优先级任务无法释放锁。
最终看门狗超时导致系统重启。
NASA 工程师通过远程命令开启了优先级继承才解决问题。

这个故事说明优先级反转不是学术概念，而是真实系统会遇到的工程问题。
机器人系统中常见模式是：

```text
低优先级日志线程持有 mutex
高优先级控制线程尝试读取同一数据而阻塞
中优先级感知线程占用 CPU
低优先级日志线程无法运行并释放 mutex
控制线程错过 deadline
```

### 替代策略：单写单读双缓冲

如果数据是一整帧状态，双缓冲通常比锁更合适。
写者写非活动缓冲，完成后原子切换索引。
读者读取当前活动缓冲。

```cpp
#include <array>
#include <atomic>

struct RobotState {
    double q[12];
    double dq[12];
    int64_t stamp_ns;
};

class StateDoubleBuffer {
public:
    void writeFromEstimator(const RobotState& state) {
        const int next = 1 - active_.load(std::memory_order_relaxed);
        buffers_[next] = state;
        // release 保证 state 内容先写完，再发布索引。
        active_.store(next, std::memory_order_release);
    }

    RobotState readFromController() const {
        // acquire 保证读到索引后，也能看到对应缓冲区的完整内容。
        const int idx = active_.load(std::memory_order_acquire);
        return buffers_[idx];
    }

private:
    std::array<RobotState, 2> buffers_{};
    std::atomic<int> active_{0};
};
```

双缓冲适合”读最新完整帧”。
它不适合需要处理每一个事件的场景。
事件流应使用 SPSC 队列（详见第三章无锁数据结构）。

> **本质洞察**：双缓冲的价值不是”比锁更快”，而是”消除了等待的概念”。
> 读者永远不等写者。写者永远不等读者。
> 代价是读者可能错过中间帧。
> 对控制线程来说，”最新状态”比”每一帧状态”更重要，因此双缓冲是最自然的选择。

### 三种数据交换模式对比

| 模式 | 实现 | 读者行为 | 写者行为 | 适用 |
|------|------|----------|----------|------|
| 互斥锁 | `std::mutex` | 可能等待 | 可能等待 | 低频、非实时 |
| 双缓冲 | 原子索引 + 两个缓冲 | 永远不等 | 永远不等 | 最新帧语义 |
| SPSC 队列 | 环形缓冲 + release/acquire | 空时返回 false | 满时返回 false | 每条事件 |
| try_lock + 旧值 | `mutex.try_lock()` | 失败用旧值 | 可能等待 | 低频参数 |

不同数据流应使用不同模式。
状态用双缓冲。
日志用 SPSC 队列。
参数用 try_lock + 旧值。
急停标志用 atomic 加 release/acquire。

### 实时线程中的错误处理

实时线程不能用异常处理错误。
但不处理错误也不行——无视错误可能输出危险命令。

| 错误类型 | 处理方式 | 禁止的做法 |
|----------|----------|-----------|
| 输入数据过旧 | 输出零力矩或保持 | 继续用旧数据计算 |
| 求解器失败 | 使用上一帧命令并记录 | 重试到成功 |
| 力矩超限 | 钳位并标记 | 忽略限幅 |
| NaN/Inf 出现 | 输出安全命令并报警 | 继续计算传播 NaN |
| 通信超时 | 进入安全模式 | 无限等待 |

每种错误都应有固定耗时的处理路径。
“固定耗时”是关键——处理错误本身不能成为另一个延迟来源。

```cpp
#include <cmath>

struct SafeOutput {
    double tau[12] = {};
    bool emergency = false;
};

SafeOutput safeClamp(const double* raw_tau, int n, double limit) {
    SafeOutput out;
    for (int i = 0; i < n && i < 12; ++i) {
        if (!std::isfinite(raw_tau[i])) {
            // NaN 或 Inf 直接输出零力矩并触发紧急标志。
            out.emergency = true;
            out.tau[i] = 0.0;
        } else if (raw_tau[i] > limit) {
            out.tau[i] = limit;
        } else if (raw_tau[i] < -limit) {
            out.tau[i] = -limit;
        } else {
            out.tau[i] = raw_tau[i];
        }
    }
    return out;
}
```

这段代码不分配、不打印、不阻塞、不抛异常。
它在固定时间内完成所有检查。
紧急标志由上层逻辑决定是否停机。

### 优先级继承 mutex

有些临界区无法避免锁，例如调用外部驱动接口。
此时应使用支持优先级继承的 pthread mutex，并让临界区极短。

```cpp
#include <pthread.h>
#include <stdexcept>

class PriorityInheritanceMutex {
public:
    PriorityInheritanceMutex() {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        if (pthread_mutex_init(&mutex_, &attr) != 0) {
            throw std::runtime_error("mutex init failed");
        }
        pthread_mutexattr_destroy(&attr);
    }

    ~PriorityInheritanceMutex() {
        pthread_mutex_destroy(&mutex_);
    }

    void lock() { pthread_mutex_lock(&mutex_); }
    void unlock() { pthread_mutex_unlock(&mutex_); }

private:
    pthread_mutex_t mutex_{};
};
```

优先级继承不是免死金牌。
它只是降低反转时间，不会让长临界区变实时。

### 练习

1. 画出一个三线程优先级反转时序图，并指出哪个线程应继承优先级。
2. 把共享机器人状态从 `std::mutex` 改成双缓冲，说明读者可能错过中间帧为什么可以接受。
3. 找一个必须加锁的资源，设计临界区边界，要求临界区内不做日志、不分配、不调用未知耗时函数。

---

## 2.5 ROS2 control 与实时边界 ⭐⭐

这一节解决的问题是：ROS2 生态中哪些接口位于实时路径，哪些必须放在生命周期阶段。

### 生命周期阶段

ros2_control 控制器通常有配置、激活、更新等阶段。
内存分配、参数解析、接口查找应在配置和激活阶段完成。
`update()` 是实时路径。

| 阶段 | 允许操作 | 不应操作 |
|------|----------|----------|
| `on_init` | 声明接口、初始化成员 | 启动实时循环 |
| `on_configure` | 读取参数、分配内存 | 假设硬件已激活 |
| `on_activate` | 重置状态、准备缓冲 | 大量动态加载 |
| `update` | 固定时间控制计算 | 分配、阻塞、日志输出 |

### RealtimeBuffer 的思想

`RealtimeBuffer` 的核心价值是把非实时参数更新和实时读取分离。
非实时线程写入新命令。
实时线程读取最近一次完整命令。

```cpp
struct Command {
    double desired_position[12];
    double kp[12];
    double kd[12];
};

class ControllerCore {
public:
    void writeCommandFromRosCallback(const Command& cmd) {
        // ROS 回调是非实时上下文，可以做少量拷贝。
        command_buffer_.writeFromNonRT(cmd);
    }

    void updateRealtime() {
        const Command* cmd = command_buffer_.readFromRT();
        if (cmd == nullptr) {
            return;
        }
        // update 中只读已经准备好的命令，不等待 ROS 回调。
        // computeTorque(*cmd);
    }

private:
    // 示例表达概念，真实项目可使用 realtime_tools::RealtimeBuffer<Command>。
    realtime_tools::RealtimeBuffer<Command> command_buffer_;
};
```

### 发布消息的边界

实时线程直接发布 ROS 消息可能阻塞。
应使用实时发布器或把数据交给非实时线程。
发布失败时，控制线程不应等待。

`realtime_tools::RealtimePublisher` 的核心思想是：
实时线程尝试写入消息缓冲。
如果非实时线程正在发布上一条消息，实时线程不等待，直接跳过本次发布。
这保证了控制周期不被消息发布阻塞。

```cpp
// 概念示例：展示 RealtimePublisher 的使用模式
// #include <realtime_tools/realtime_publisher.h>
// #include <sensor_msgs/msg/joint_state.hpp>

// realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState> rt_pub_;

void publishFromRealtimeLoop() {
    // 尝试获取发布锁。如果上一条消息还没发完，直接返回。
    // if (rt_pub_.trylock()) {
    //     rt_pub_.msg_.header.stamp = now;
    //     rt_pub_.msg_.position = current_positions;
    //     rt_pub_.unlockAndPublish();
    // }
    // 控制线程不因为发布而等待。
}
```

### ros2_control 中的实时安全检查清单

| 检查项 | 合格标准 | 常见违反 |
|--------|---------|----------|
| `on_configure` 完成所有分配 | `update()` 中无 `new`/`resize` | 在回调中动态创建订阅器 |
| `update()` 不调用 ROS 参数接口 | 不出现 `get_parameter()` | 每周期查询参数 |
| `update()` 不打印日志 | 不出现 `RCLCPP_INFO/WARN/ERROR` | 异常分支打印调试信息 |
| 命令通过 RealtimeBuffer 接收 | 非实时回调写，实时循环读 | 直接在 `update()` 中处理回调 |
| 状态通过 RealtimePublisher 发布 | 尝试发布，失败不阻塞 | 普通 publish 阻塞控制 |

### 练习

1. 列出一个 ros2_control 控制器中所有内存分配点，并把它们移动到 `on_configure` 或 `on_activate`。
2. 解释为什么参数动态更新不能直接改实时线程正在读的结构体。
3. 设计一个“发布控制状态”的机制，要求实时线程不阻塞，非实时线程可以丢帧。

---

## 2.6 从调度配置到故障演练：实时系统的完整闭环 ⭐⭐⭐

这一节解决的问题是：如何把实时 Linux、实时安全 C++、测量方法和故障处理串成一个可执行的工程闭环。

### 从理论到实践：一个完整的四足控制实时配置

下面把前面讨论过的所有概念整合成一个四足机器人控制进程的完整配置示例。

**硬件平台**：Intel NUC 或工控机，4 核 CPU，8 GB RAM，Ubuntu 24.04 + RT 内核。

**内核配置**：
```text
# GRUB 启动参数
isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0,1
```

**线程架构**：

| 线程 | 调度策略 | 优先级 | CPU | 职责 |
|------|----------|--------|-----|------|
| 急停监控 | SCHED_FIFO | 90 | 2 | 读急停按钮和过流检测 |
| 总线接收 | SCHED_FIFO | 85 | 2 | 接收关节编码器和 IMU |
| 控制循环 | SCHED_FIFO | 80 | 3 | 1 kHz WBC/PD |
| 状态估计 | SCHED_FIFO | 70 | 3 | 状态融合和滤波 |
| MPC 重规划 | SCHED_FIFO | 50 | 0 | 30 Hz 轨迹优化 |
| ROS2 回调 | SCHED_OTHER | - | 0,1 | 参数、可视化、命令 |
| 日志写盘 | SCHED_OTHER | - | 1 | 异步格式化和存储 |

**数据流**：

```text
总线接收 ─── LatestFrame<JointState> ──────► 状态估计
                                              │
状态估计 ─── LatestFrame<RobotState> ────────► 控制循环
MPC 重规划 ── TripleBuffer<MpcTrajectory> ──► 控制循环
ROS2 回调 ── LatestFrame<ControllerParams> ─► 控制循环
控制循环 ─── SpscRing<ControlTrace> ─────────► 日志写盘
控制循环 ─── SpscRing<MotorCommand> ─────────► 总线发送
```

**延迟预算**（1 kHz 控制循环，周期 1000 us）：

| 阶段 | 预算 | 备注 |
|------|------|------|
| 调度唤醒 | 50 us | 由 cyclictest 验证 |
| 状态快照读取 | 20 us | 双缓冲 acquire |
| 状态估计更新 | 100 us | EKF 预测+更新 |
| WBC QP 求解 | 400 us | 含矩阵装配 |
| 安全检查和限幅 | 50 us | NaN 检查、力矩钳位 |
| 命令写入总线 | 30 us | SPSC push |
| trace 记录 | 10 us | SPSC push |
| 安全余量 | 340 us | 应对尖峰和负载变化 |

安全余量应至少占 30% 的周期。
如果余量被吃掉到 10% 以下，说明系统在边界运行，需要优化或降低频率。

### 调度优先级不是越高越好

初学者第一次接触实时调度时，很容易把优先级理解成”越大越安全”。
这个理解只对了一半。
高优先级确实能减少被普通任务打断的概率，但过高的优先级也可能饿死系统服务、网络线程和安全监控。
实时系统不是把一个线程抬到最高就结束，而是设计一套优先级层级，让关键任务按物理时间尺度排列。

可以用机器人控制链路的频率来排序：

| 任务 | 典型频率 | 延迟容忍 | 建议优先级关系 | 说明 |
|------|----------|----------|----------------|------|
| 电机安全保护 | 1-10 kHz | 最低 | 最高 | 过流、过温、急停必须先处理 |
| 状态接收 | 1 kHz | 很低 | 高于控制计算 | 没有新状态时控制量意义下降 |
| WBC/关节控制 | 500 Hz-1 kHz | 很低 | 高 | 输出本周期命令 |
| 状态估计 | 200 Hz-1 kHz | 中低 | 接近控制 | 取决于是否在控制前必须完成 |
| MPC/局部规划 | 20-100 Hz | 中 | 低于 WBC | 可以复用上一帧计划 |
| 日志/可视化 | 1-50 Hz | 高 | 普通优先级 | 可丢帧，不应阻塞控制 |

这张表背后的原则是：越靠近硬件安全，优先级越高；越偏向记录和展示，优先级越低。
如果把日志线程设得比控制线程还高，就像让会议纪要打断手术操作，信息保存得再完整也没有意义。

一个常见的线程表可以写成：

```text
线程优先级示例

SCHED_FIFO 90: 电机保护与急停
SCHED_FIFO 85: 总线状态接收
SCHED_FIFO 80: 1 kHz 控制循环
SCHED_FIFO 70: 状态估计融合
SCHED_FIFO 50: MPC 重规划
SCHED_OTHER : 日志、可视化、参数服务
```

如果不这样分层会怎样？
如果 MPC 线程优先级高于 WBC，它在复杂地形上偶发多迭代时可能阻塞高频控制。
如果日志线程和控制线程同级，它在磁盘抖动时可能抢占控制。
如果状态接收优先级太低，控制线程会准时运行，却使用旧状态计算新命令。
因此优先级设计要围绕“本周期需要什么数据”和“错过一次的代价是什么”来判断。

> **本质洞察**：实时调度不是奖励重要代码，而是把物理世界的时间约束映射到操作系统的抢占规则。
> 频率越高、错过后果越严重、越靠近安全闭环，优先级就越应该靠前。
> 反过来，任何可以丢帧、复用旧值或延迟处理的任务，都不应该站在控制线程前面。

### 周期循环的时间账本

1 kHz 不是“每秒跑 1000 次”的口号，而是每 1 ms 都有一张时间账本。
这张账本需要同时记录计算耗时和唤醒延迟。

```text
一次 1 kHz 控制周期

0 us     线程被唤醒
20 us    读取共享状态快照
120 us   状态估计或滤波更新
450 us   控制律或 WBC QP
650 us   安全检查与限幅
750 us   写入总线发送缓冲
1000 us  下一个 deadline
```

这里有两个容易混淆的量：

| 指标 | 含义 | 例子 | 处理方向 |
|------|------|------|----------|
| 唤醒延迟 | 本该醒来的时间到实际运行的差 | 目标 0 us，实际晚 80 us | 调度、CPU、IRQ、内核 |
| 执行耗时 | 控制代码自身运行多久 | WBC 求解 550 us | 算法、分配、锁、缓存 |

如果只测执行耗时，可能看不到调度问题。
如果只跑 cyclictest，可能看不到代码内部的锁竞争和分配。
真实系统需要两类测量同时存在：系统级调度测量和应用级周期测量。

下面是应用内记录周期指标的最小结构。
它不做动态分配，适合在实时路径写入。

```cpp
#include <array>
#include <atomic>
#include <cstdint>

struct CycleSample {
    int64_t wake_late_ns = 0;
    int64_t compute_ns = 0;
    bool missed_deadline = false;
};

class CycleTrace {
public:
    void push(const CycleSample& sample) {
        // 固定容量环形记录；满了就覆盖最旧样本。
        const std::size_t idx = write_index_.load(std::memory_order_relaxed);
        samples_[idx % samples_.size()] = sample;
        write_index_.store(idx + 1, std::memory_order_release);
    }

    std::array<CycleSample, 4096> snapshot() const {
        // 非实时线程可以复制快照后再统计 p99 和最大值。
        return samples_;
    }

private:
    std::array<CycleSample, 4096> samples_{};
    std::atomic<std::size_t> write_index_{0};
};
```

这段代码故意不在实时线程里计算 p99。
p99 需要排序或桶统计，可能带来额外开销。
实时线程只负责把关键样本写入固定内存；非实时线程负责统计和打印。

### 优先级反转的三层处理

优先级反转不是“用了 mutex 就一定错”，而是“高优先级线程等待低优先级线程时，等待时间可能失控”。
处理它有三层策略。

| 层级 | 方法 | 适用场景 | 边界 |
|------|------|----------|------|
| 避免共享 | 双缓冲、SPSC 队列、快照 | 状态、命令、日志 | 适合数据复制成本可控 |
| 限制等待 | `try_lock`、超时、短临界区 | 偶尔读取配置 | 需要定义失败路径 |
| 修正调度 | PI mutex、优先级天花板 | 无法避免共享设备 | 不能拯救长临界区 |

最优先考虑的是避免共享。
如果控制线程只需要“最新完整状态”，双缓冲比互斥锁更自然。
如果它需要“每个事件都处理”，SPSC 队列更自然。
只有在资源本身不可复制时，才考虑锁。

用 `try_lock` 时必须有明确失败策略。
失败策略不是随便返回，而是要符合控制语义。

```cpp
#include <mutex>

struct Gains {
    double kp[12];
    double kd[12];
};

class GainReader {
public:
    Gains readForRealtime() {
        // 控制线程不等待配置锁。
        // 拿不到新参数时继续使用上一份稳定参数。
        if (mutex_.try_lock()) {
            cached_ = shared_;
            mutex_.unlock();
        }
        return cached_;
    }

    void writeFromConfigThread(const Gains& gains) {
        // 配置线程不是实时路径，可以使用普通锁保护共享参数。
        std::lock_guard<std::mutex> lock(mutex_);
        shared_ = gains;
    }

private:
    std::mutex mutex_;
    Gains shared_{};
    Gains cached_{};
};
```

这个例子里，`try_lock` 失败不是错误。
它表示本周期继续使用旧增益。
这种策略适合低频参数更新，但不适合传感器状态。
传感器状态如果拿不到新值，需要记录状态年龄，并决定是否降级或停机。

### 内存锁定必须配合栈和堆的预热

`mlockall(MCL_CURRENT | MCL_FUTURE)` 只能避免已映射页面被换出或未来映射页面被换出。
它不等于“以后不会缺页”。
如果某个大数组在实时循环第一次访问时才真正触碰到页面，仍可能产生 minor page fault。

因此启动阶段要做三件事：

1. 锁定当前和未来映射页面。
2. 预分配堆内存并逐页写入。
3. 预触实时线程栈空间。

```cpp
#include <array>
#include <cstddef>
#include <stdexcept>
#include <sys/mman.h>
#include <vector>

void prefaultStack() {
    // 在启动阶段触碰一段栈内存，降低实时循环首次使用栈页时缺页的概率。
    // volatile 防止编译器把写入优化掉。
    volatile std::array<char, 64 * 1024> stack_buffer{};
    for (std::size_t i = 0; i < stack_buffer.size(); i += 4096) {
        stack_buffer[i] = 0;
    }
}

void lockAndWarmMemory(std::vector<double>& heap_buffer) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        throw std::runtime_error("mlockall failed");
    }

    // 逐页写入堆缓冲，确保页表已经建立。
    const std::size_t doubles_per_page = 4096 / sizeof(double);
    for (std::size_t i = 0; i < heap_buffer.size(); i += doubles_per_page) {
        heap_buffer[i] = 0.0;
    }

    prefaultStack();
}
```

如果不预触会怎样？
程序启动后一切看似正常，第一次进入复杂工况时才访问到某个大缓存的后半段。
此时内核需要建立页表，控制周期突然多出几十到几百微秒。
这类尖峰很难用平均耗时发现，但在最大延迟中会非常明显。

内存锁定也有边界。
不要把整个大型日志缓存、地图和神经网络权重都锁进内存。
锁太多页面会挤压系统其他任务，反而造成整体不稳定。
实时路径需要什么，就锁什么；非实时数据保持普通内存策略。

### 实时安全 C++ 的判断标准

实时安全 C++ 不是一张禁用清单那么简单。
同一个操作在启动阶段安全，在控制循环里就可能不安全。
判断标准可以分成四个问题：

| 问题 | 如果答案不清楚 | 例子 |
|------|----------------|------|
| 是否可能分配内存 | 不进实时路径 | `std::vector::push_back` |
| 是否可能阻塞 | 不进实时路径 | `std::mutex::lock`、文件 I/O |
| 是否可能调用未知耗时系统调用 | 不进实时路径 | `printf`、动态加载 |
| 是否有有界失败策略 | 没有就不能用 | 队列满时阻塞 |

这可以写成接口层面的规则。
实时路径的函数应尽量返回状态码，而不是抛异常、打印或等待。

```cpp
#include <array>
#include <cstddef>
#include <cstdint>

enum class RtResult {
    kOk,
    kNoNewState,
    kCommandSaturated,
    kInputTooOld
};

struct ControlInput {
    double q[12];
    double dq[12];
    int64_t age_ns;
};

struct ControlOutput {
    double tau[12];
};

class RealtimeController {
public:
    RtResult update(const ControlInput& input, ControlOutput* output) {
        if (output == nullptr) {
            return RtResult::kNoNewState;
        }
        if (input.age_ns > 2'000'000) {
            // 状态超过 2 ms，输出保持保守值。
            zeroTorque(output);
            return RtResult::kInputTooOld;
        }

        for (std::size_t i = 0; i < 12; ++i) {
            // 示例 PD 控制；真实工程中还会加入限幅和安全检查。
            output->tau[i] = -kp_[i] * input.q[i] - kd_[i] * input.dq[i];
        }
        return RtResult::kOk;
    }

private:
    void zeroTorque(ControlOutput* output) {
        for (double& tau : output->tau) {
            tau = 0.0;
        }
    }

    std::array<double, 12> kp_{};
    std::array<double, 12> kd_{};
};
```

这里的关键不是 PD 公式，而是失败路径。
输入太旧时函数仍然在固定时间内返回，并输出保守命令。
它没有分配、没有打印、没有抛异常、没有等待其他线程。

### 测量要覆盖空载、压力和真实负载

实时测量至少要覆盖三种环境：

| 环境 | 目的 | 可能暴露的问题 |
|------|------|----------------|
| 空载 | 确认基本配置正确 | 调度策略、权限、代码明显超时 |
| 人工压力 | 放大系统干扰 | IRQ、CPU 争用、内存压力、磁盘抖动 |
| 真实负载 | 还原上机场景 | 总线通信、传感器回调、规划计算 |

人工压力不是为了“折磨系统”，而是为了提前暴露尾部延迟。
可以组合 CPU、内存和 I/O 压力，再运行 cyclictest 和应用内周期统计。

```bash
# 终端 1：测 1 kHz 唤醒延迟，优先级 80，运行 100000 次
sudo cyclictest -p 80 -t1 -n -i 1000 -l 100000

# 终端 2：制造 CPU 压力，观察最大延迟是否明显变大
stress-ng --cpu 4 --timeout 120s

# 终端 3：运行控制程序自己的周期统计
./realtime_controller --print-latency-summary
```

判断数据时不要只看平均值。
至少记录：

1. 最大唤醒延迟。
2. 最大执行耗时。
3. deadline miss 次数。
4. 连续 miss 的最长长度。
5. 状态数据年龄最大值。

连续 miss 比单次 miss 更危险。
单次 miss 可能被下一周期修正；连续 miss 会让控制器长期使用旧状态或重复命令。

### 故障案例 1：日志没有错，控制错过了周期

现象：机器人在推搡测试中偶发抖动，日志文件完整记录了所有中间量。
表面看日志越完整越好，但这恰好提示日志可能在实时路径上。

故障链通常是：

```text
控制线程进入异常分支
→ 直接 printf 或写文件
→ stdio 内部锁和文件系统路径耗时不可控
→ 本周期命令发送延迟
→ 下一周期状态误差变大
→ 异常分支更频繁触发
```

修复方法不是减少日志文字，而是改变日志路径。
实时线程只写固定大小事件，后台日志线程决定如何格式化。
队列满时丢弃低优先级日志。

| 修复点 | 判断标准 |
|--------|----------|
| 实时线程不调用格式化输出 | 搜索 `printf`、`std::cout`、日志宏 |
| 队列写入固定时间 | 满队列返回 false |
| 日志线程可落后 | 日志丢失计数单独记录 |
| 控制安全不依赖日志 | 日志失败不改变控制命令 |

### 故障案例 2：第一次触碰大矩阵导致缺页

现象：程序启动后前几秒正常，第一次进入某个步态切换时控制周期突然超时。
检查求解器耗时并不高，矩阵装配也没有明显变慢。

常见原因是某个矩阵或缓存虽然已经 `resize`，但实际页面没有被访问。
步态切换时第一次写入该矩阵的某些块，触发缺页。

排查步骤：

1. 记录每个周期的 minor page fault 计数。
2. 在启动阶段遍历所有实时路径缓冲。
3. 把步态切换前后的最大周期耗时单独统计。
4. 用 `mlockall` 后再次比较结果。

在 Linux 上可以用 `/proc/self/stat` 或 `getrusage` 观察缺页变化。
实时路径里不建议频繁调用这些接口，但调试构建可以短时间打开。

```cpp
#include <sys/resource.h>

long readMinorFaultsForDebug() {
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    // ru_minflt 是 minor page faults 计数，用于调试缺页趋势。
    return usage.ru_minflt;
}
```

### 故障案例 3：std::string 在异常分支分配

现象：控制器在正常运行时非常稳定，但在机器人被推倒后恢复过程中出现周期抖动。
经分析，异常恢复分支构造了一个 `std::string` 错误消息。

```cpp
// 错误写法
if (fell_down) {
    std::string msg = "Robot fell at time " + std::to_string(timestamp);
    // std::string 拼接和 to_string 都可能分配堆内存
    logError(msg);
}
```

正确做法是使用固定大小缓冲或只写数值到预分配 trace。

```cpp
// 正确写法
if (fell_down) {
    FallEvent event;
    event.timestamp = timestamp;
    event.type = EventType::kFallDetected;
    // 固定大小结构，不分配，写入无锁队列。
    trace_queue.push(event);
}
```

这个故障特别隐蔽，因为异常分支很少执行。
只有在压力测试中反复触发跌倒恢复才能暴露。

### 故障案例 4：优先级反转被误判为算法慢

现象：WBC 平均求解 400 us，但偶发周期达到 2 ms。
开发者先怀疑 QP 求解器，换求解器后问题仍然存在。

实际故障链可能是：

```text
低优先级参数线程持有 mutex
→ 控制线程读取参数时阻塞
→ 中优先级感知线程占用 CPU
→ 参数线程迟迟不能释放 mutex
→ 控制线程表现为“算法耗时变长”
```

排查时要把锁等待时间和算法耗时分开记录。
如果只在 `update()` 函数入口和出口计时，就会把锁等待误算成控制算法慢。

```cpp
#include <chrono>
#include <mutex>

struct LockTiming {
    double wait_us = 0.0;
    bool locked = false;
};

LockTiming tryLockWithTiming(std::timed_mutex& mutex) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = mutex.try_lock_for(std::chrono::microseconds(10));
    const auto t1 = std::chrono::steady_clock::now();
    LockTiming timing;
    timing.wait_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    timing.locked = ok;
    return timing;
}
```

这段代码用于诊断思路。
真正的硬实时路径更常用无阻塞数据交换，而不是带超时的锁。
如果确实必须使用锁，临界区内应只做固定时间的数据拷贝。

### 故障案例 4：动态矩阵在运行期偷偷 resize

现象：控制器正常运行，但每隔一段时间出现一次 2-3 ms 周期尖峰。
通过 CycleTrace 观察，尖峰只出现在某些步态切换时。

排查发现：某个 QP 装配函数根据接触脚数量动态调整约束矩阵大小。
当接触脚从 4 变成 2 时，矩阵的行数减少，`resize` 可能不分配。
当接触脚从 2 变回 4 时，矩阵行数增加，`resize` 触发堆分配。

```cpp
// 错误写法
void assembleConstraints(int n_contacts) {
    // 每次切换接触数时可能重新分配。
    Eigen::MatrixXd A(n_contacts * 3, n_var);  // 这里分配堆内存
    // ...
}

// 正确写法
class WbcSolver {
    Eigen::MatrixXd A_;  // 启动阶段按最大接触数分配

    void init(int max_contacts, int n_var) {
        A_.resize(max_contacts * 3, n_var);  // 一次分配
    }

    void assembleConstraints(int n_contacts) {
        // 只使用 A_ 的前 n_contacts*3 行，不改变分配。
        auto A_active = A_.topRows(n_contacts * 3);
        // ...
    }
};
```

这个问题用 `EIGEN_RUNTIME_NO_MALLOC` 可以直接捕获。

### 故障案例 5：CPU 亲和性配置正确，但中断打在控制核上

现象：控制线程已经固定到独立 CPU，仍然出现周期尖峰。
原因可能不是线程迁移，而是设备中断或内核任务仍然在同一核心运行。

排查步骤：

```bash
# 查看中断计数，观察网卡、USB、CAN 等中断是否集中在控制核
cat /proc/interrupts

# 查看线程当前运行在哪些 CPU 上
ps -eLo pid,tid,psr,cls,rtprio,comm | head

# 查看某个进程的 CPU 亲和性
taskset -pc <pid>
```

CPU 亲和性只约束线程运行位置，不自动迁移所有中断。
如果 CAN 或以太网中断频繁打到控制核，控制线程仍可能被打断。
更严格的部署会同时配置线程亲和性、IRQ affinity 和内核启动参数。

但隔离也不是越多越好。
如果所有非实时任务被挤到一个核心，日志、网络和规划可能互相拥塞。
实时系统需要的是干扰可控，而不是盲目把所有任务推到同一个角落。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念 | 把最高优先级给控制算法 | 系统服务被饿死 | 忽略安全保护和状态接收 | 按物理时间尺度分层 |
| 编程 | 只调用 `mlockall` 不预触 | 首次复杂工况超时 | 页面第一次访问仍会缺页 | 锁内存后遍历实时缓冲 |
| 编程 | 用 `mutex.lock()` 读参数 | 偶发长周期 | 锁等待无上界 | 双缓冲或 `try_lock` 加旧值策略 |
| 测量 | 只看 cyclictest | 应用仍超时 | 代码内部锁和分配未测 | 同时记录应用周期指标 |
| 思维 | 日志必须完整 | 控制被日志拖慢 | 把诊断优先级放到控制之上 | 低优先级日志可丢弃 |

### 练习

1. 为一个四足机器人控制进程设计线程优先级表，包含急停、总线接收、WBC、MPC、日志和可视化，并说明每个优先级的依据。
2. 给现有 1 kHz 循环加入 `CycleTrace`，记录唤醒延迟、计算耗时和 deadline miss，压力测试 10 分钟后输出最大值。
3. 把一个实时路径中的 `std::mutex::lock` 改成双缓冲或 `try_lock` 旧值策略，解释丢失一次参数更新为什么比阻塞控制更可接受。
4. 在启动阶段加入内存锁定、堆预触和栈预触，然后对比第一次步态切换前后的最大周期耗时。

---

### 实时部署启动检查清单

在机器人上线前，应逐项确认以下检查清单。
不通过的项不应进入实机运行。

| 检查项 | 验证方法 | 合格标准 |
|--------|----------|----------|
| RT 内核已启用 | `uname -a` 或 `/sys/kernel/realtime` | 包含 PREEMPT_RT 标志 |
| cyclictest max < 阈值 | 压力下运行 10+ 分钟 | 依据控制频率设定 |
| 控制线程 SCHED_FIFO | `ps -eLo cls,rtprio,comm` | 正确策略和优先级 |
| CPU 亲和性已设 | `taskset -pc <pid>` | 控制线程在隔离核 |
| 内存已锁定 | `/proc/<pid>/status` VmLck | 非零 |
| 预触页已完成 | 首次周期无缺页 | `getrusage` minflt 稳定 |
| 控制循环无 malloc | EIGEN_RUNTIME_NO_MALLOC | 无断言触发 |
| 日志不在实时路径 | 搜索 printf/cout/RCLCPP_INFO | 不在 update() 中 |
| 求解器已预初始化 | init 在启动阶段完成 | solve 只更新数值 |
| 降级策略已测试 | 注入失败场景 | 输出安全命令 |
| 周期统计已接入 | CycleTrace 或类似工具 | 记录 p99 和 max |
| IRQ 已配置 | `/proc/interrupts` | 控制核无高频中断 |

### 完整启动序列示例

```text
机器人控制进程启动序列

1. 解析配置文件和参数
2. 加载 URDF 和动力学模型
3. 初始化求解器（分配内存、建立稀疏结构）
4. 分配所有实时缓冲（状态、命令、trace、日志队列）
5. 调用 mlockall 和预触页
6. 创建日志线程（SCHED_OTHER）
7. 创建状态接收线程（SCHED_FIFO 85）
8. 创建控制线程（SCHED_FIFO 80）
9. 设置控制线程 CPU 亲和性
10. 等待传感器数据有效
11. 进入控制循环（绝对时间睡眠）
12. 循环中：读状态 → 计算 → 限幅 → 发送命令 → 写 trace
```

注意步骤 1-9 都是配置期，允许分配和异常。
步骤 11 起进入运行期，不再允许分配、阻塞和 I/O。

> **本质洞察**：实时安全不是一个开关，而是两个阶段的边界。
> 配置期可以做任何 C++ 允许的事。
> 运行期只能做确定耗时的事。
> 这条边界必须在代码架构中显式表达，不能靠口头约定。

---

## 2.7 实时性能测量方法论 ⭐⭐

这一节解决的问题是：如何系统地测量、记录和报告实时控制系统的性能数据。

### 动机：感觉快不等于实时安全

很多开发者通过观察机器人运动来判断控制器"够不够快"。
这种方法有两个根本缺陷。
第一，人眼无法分辨毫秒级延迟差异。
第二，偶发的 deadline miss 可能不会立刻导致明显异常，但会在特定动态条件下放大。

实时性能必须用数据说话。
数据不是平均值。
数据是分布：p50、p95、p99、max、miss count。

### 四类测量

| 类别 | 测量什么 | 工具 | 关注指标 |
|------|----------|------|----------|
| 调度延迟 | 线程唤醒与期望时间的偏差 | cyclictest | max 延迟 |
| 计算耗时 | 控制代码执行时间 | CycleTrace、Tracy | p99 和 max |
| 内存行为 | 堆分配次数和位置 | EIGEN_RUNTIME_NO_MALLOC、ASan | 分配计数 |
| 端到端延迟 | 从传感器到执行器的完整链路 | 硬件时间戳对比 | 链路延迟 |

这四类测量应同时进行。
只测一类可能漏掉其他类别的问题。

### 报告模板

实时性能报告应包含以下内容：

```text
实时性能报告

硬件：Intel NUC i7-12700H, 4 核隔离 2+3
内核：Linux 6.12.8-rt, PREEMPT_RT
构建：RelWithDebInfo, -O2, Eigen 3.4.0

测试时长：10 分钟（600000 个控制周期）
控制频率：1 kHz

调度延迟（cyclictest -p80 -t1 -n -i1000）：
  min/avg/max = 3/8/42 us

控制周期耗时：
  p50 = 380 us
  p95 = 510 us
  p99 = 620 us
  max = 850 us
  deadline miss (>1000 us) = 0

堆分配：0 次（EIGEN_RUNTIME_NO_MALLOC 验证通过）

压力条件：stress-ng --cpu 4 --vm 2 --io 4 同时运行
```

### 如何解读数据

| 观察 | 可能原因 | 下一步 |
|------|----------|--------|
| p99 远大于 p50 | 偶发慢路径 | 用 Tracy 标记找出哪个阶段慢 |
| max 远大于 p99 | 系统级干扰或内存事件 | 检查 IRQ、缺页、锁等待 |
| 压力下 max 剧增 | CPU 或内存竞争 | 加强隔离或降低频率 |
| 无压力也有 miss | 代码本身超预算 | 优化算法或降低频率 |
| 分配次数非零 | 实时路径有堆操作 | 定位并移到启动阶段 |

> **本质洞察**：实时性能不是一个数字，而是一个分布。
> 平均值只说明长期负载。
> p99 说明常见的慢路径。
> max 说明最坏情况。
> 控制系统的安全性取决于 max，不取决于 mean。

### 练习

1. 在你的控制器中加入 CycleTrace 统计，运行 10 分钟后输出 p50/p95/p99/max，并与 cyclictest 结果对比。
2. 写一份实时性能报告，包含硬件、内核、编译选项、压力条件和五项指标。
3. 解释为什么 mean=200 us 但 max=5 ms 的控制器对 1 kHz 不安全。

---

### 跨章综合练习

1. 结合本章实时 Linux 和第一章 QP 求解器知识：设计一个完整的 1 kHz WBC 控制线程，要求 SCHED_FIFO 优先级 80，CPU 固定到隔离核，内存锁定，QP 求解器在启动阶段初始化，控制循环只做数值更新和求解，日志通过 SPSC 队列输出。画出完整的线程优先级表和数据流图。

2. 结合本章实时安全 C++ 和第三章无锁数据结构知识：设计一个"状态估计到控制到日志"的三线程数据交换架构，要求状态用双缓冲传递，日志用 SPSC 队列传递，参数用 try_lock 加旧值策略更新。说明每个通道的失败策略。

3. 结合本章 PREEMPT_RT 和第八章测试知识：设计一个实时性能测试，用 cyclictest 测调度延迟，用应用内 CycleTrace 测计算耗时，用 ASan 检查内存安全，分别在空载和压力下运行 10 分钟，输出 p50/p95/p99/max。

### 版本与生态更新

| 技术 | 当前状态（2026） | 工程影响 |
|------|-----------------|----------|
| PREEMPT_RT | Linux 6.12+ 主线合入 | 不再需要独立 RT 补丁 |
| SCHED_DEADLINE | 主线支持但工具链不够成熟 | 大多数项目仍用 SCHED_FIFO |
| ros2_control | Jazzy/Rolling 已稳定 | `update()` 是实时路径的共识 |
| realtime_tools | 持续维护 | RealtimeBuffer 和 RealtimePublisher |
| cyclictest | rt-tests 套件的一部分 | 压力测试的标准工具 |
| Eigen malloc 检测 | `EIGEN_RUNTIME_NO_MALLOC` | 调试构建中检测实时路径分配 |

## 本章小结

| 主题 | 关键结论 | 工程动作 |
|------|----------|----------|
| 实时定义 | 准时是正确性的一部分 | 记录最大延迟和 deadline miss |
| PREEMPT_RT | Linux 6.12+ 主线化，降低内核调度延迟 | 配合 cyclictest 和压力测试 |
| SCHED_DEADLINE | 基于预算的调度，适合特定场景 | 大多数项目仍用 SCHED_FIFO |
| CPU 隔离 | isolcpus + nohz_full + IRQ affinity 三层 | 减少控制核干扰 |
| C++ 禁区 | 不分配、不阻塞、不调用不可控系统调用 | 预分配、环形队列、双缓冲 |
| 隐式分配 | 很多标准库操作会偷偷 malloc | EIGEN_RUNTIME_NO_MALLOC 审计 |
| 同步 | 锁会带来优先级反转 | 优先使用无阻塞数据交换 |
| ROS2 control | `update()` 是实时路径 | 生命周期阶段完成配置和分配 |
| 故障案例 | 日志、缺页、优先级反转、IRQ 都可能超时 | 分别测量和隔离 |

## 累积项目：1 kHz 实时控制骨架

本章新增模块是 `realtime_loop`。

阶段 1：实现基于 `CLOCK_MONOTONIC` 和 `TIMER_ABSTIME` 的 1 kHz 循环。
阶段 2：启动阶段设置线程优先级、CPU 亲和性、内存锁定和预触页。
阶段 3：加入固定容量日志环形队列，非实时线程负责输出。
阶段 4：加入 Eigen malloc 检测，保证控制更新路径不分配。
阶段 5：运行压力测试，记录周期耗时、最大唤醒延迟和 deadline miss 次数。
阶段 6：配置 CPU 隔离和 IRQ 亲和性，对比隔离前后的最大唤醒延迟。
阶段 7：接入第一章的 QP 求解器适配层，在实时循环中调用 `updateNumerics()` 和 `solve()`。
阶段 8：为实时循环添加参数双缓冲接口，通过 ROS2 回调更新 PID 参数。
阶段 9：添加故障注入测试：模拟状态超时、日志队列满和求解器失败，验证降级路径。
阶段 10：在目标硬件（如 UP Board、Jetson 或工控机）上运行完整实时骨架，记录真实硬件上的延迟数据。

## 延伸阅读

| 资料 | 难度 | 阅读目的 |
|------|------|----------|
| Linux PREEMPT_RT wiki 和 lwn.net 6.12 合入文章 | ⭐⭐ | 理解 RT 主线化历史和能力边界 |
| `cyclictest` 和 rt-tests 手册 | ⭐ | 学会测调度延迟 |
| SCHED_DEADLINE 内核文档 | ⭐⭐⭐ | 理解 EDF 调度参数 |
| ros2_control 控制器示例 | ⭐⭐ | 学习生命周期和实时更新 |
| realtime_tools ROS2 包 | ⭐⭐ | RealtimeBuffer 和 RealtimePublisher |
| C++ Core Guidelines | ⭐⭐ | 识别资源所有权和异常边界 |
| Eigen 文档 malloc 检测章节 | ⭐⭐ | 审计线性代数分配 |
| Jan Altenberg, RT Linux Performance Analysis | ⭐⭐⭐ | 系统级延迟分析方法 |
| kernel.org Documentation/scheduler | ⭐⭐⭐ | 调度策略内核实现 |

## 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 处理 |
|------|----------|----------|------|
| 控制周期偶发超时 | 实时路径分配或 I/O | 开启 malloc 检测，移除打印 | 预分配和异步日志 |
| cyclictest 压力下延迟大 | 中断或普通任务打断控制核 | 检查 CPU 和 IRQ 亲和性 | 隔离控制核或调整 IRQ |
| 高优先级线程卡住 | 等待低优先级线程持锁 | 记录锁等待时间 | 双缓冲或 PI mutex |
| 程序启动后首次控制很慢 | 页面第一次访问缺页 | 统计首次周期耗时 | `mlockall` 加预触页 |
| ROS 参数更新导致抖动 | 回调直接改实时数据 | 检查共享数据路径 | 用 RealtimeBuffer 交换 |
| 日志完整但控制失稳 | 控制线程为日志阻塞 | 检查日志调用栈 | 满队列丢日志，不阻塞 |
| RT 内核安装后延迟仍大 | 未启用 PREEMPT_RT | 检查 `/sys/kernel/realtime` | 确认内核配置 |
| CPU 隔离后网络变慢 | 所有非 RT 任务挤在一个核 | 检查网络线程和 IRQ 分配 | 合理分配非隔离核 |
| Eigen 动态矩阵在实时路径分配 | 未预分配或调用了 resize | 开启 EIGEN_RUNTIME_NO_MALLOC | 预分配并固定尺寸 |
| SCHED_DEADLINE 设置失败 | 预算参数不合理 | 检查 runtime/deadline/period | 确保 runtime <= deadline <= period |
| 控制线程周期不稳定 | 用了相对睡眠 | 检查 sleep 模式 | 改为 CLOCK_MONOTONIC + TIMER_ABSTIME |
| `mlock` 后系统可用内存不足 | 锁定了过多内存 | 检查 `VmLck` 和系统总内存 | 只锁定实时线程需要的页面，或调整 `RLIMIT_MEMLOCK` |
| 实时线程启动后其他线程饿死 | SCHED_FIFO 优先级过高且计算量大 | 检查非 RT 线程的调度统计 | 合理设置优先级层级，确保 RT 线程有明确的让出点 |
| EtherCAT 通信偶发超时 | 网络中断与控制线程竞争 CPU | 用 `cat /proc/interrupts` 检查网卡中断分布 | 将网卡中断绑定到非隔离核 |

---

## 实时调试与性能分析工具链 ⭐⭐⭐

### 工程问题：实时系统的 bug 难以用传统方法复现

实时系统的故障通常具有概率性——在空载下完全正常，在负载下偶尔出现一次延迟毛刺。传统的断点调试（GDB）会暂停整个进程，破坏实时性；`printf` 调试可能本身就引入 malloc 和 I/O 阻塞。需要专门的工具链来诊断实时问题而不干扰实时行为。

### 非侵入式诊断工具

| 工具 | 用途 | 侵入性 | 使用场景 |
|------|------|--------|---------|
| `cyclictest` | 测量调度延迟 | 低 | 评估内核和硬件的实时能力 |
| `perf sched` | 分析线程调度事件 | 低 | 查找优先级反转和调度延迟来源 |
| `ftrace` | 内核级函数跟踪 | 中 | 分析内核路径的延迟贡献 |
| `trace-cmd` | ftrace 的用户态前端 | 中 | 录制和分析调度、中断和系统调用事件 |
| `bpftrace` | eBPF 脚本化跟踪 | 低 | 自定义探针，如统计特定系统调用的频率 |
| `hwlatdetect` | 检测硬件级延迟（SMI） | 极低 | 排除 BIOS/SMM 引入的不可屏蔽延迟 |

### 应用层计时框架

在控制循环内部，需要一个零分配的计时框架来记录每帧的关键时间点：

```cpp
// CycleTrace：预分配的实时计时器
class CycleTrace {
    struct Entry {
        uint64_t timestamp_ns;
        uint16_t tag;  // 预定义的检查点编号
    };

    std::array<Entry, 8192> buffer_;  // 预分配环形缓冲区
    size_t write_pos_ = 0;

public:
    void mark(uint16_t tag) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);  // 无系统调用开销（vDSO）
        buffer_[write_pos_ & 8191] = {
            static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec,
            tag
        };
        ++write_pos_;
    }
};

// 在控制循环中使用
void controlLoop(CycleTrace& trace) {
    trace.mark(0);  // 循环开始
    readSensors();
    trace.mark(1);  // 传感器读取完成
    computeControl();
    trace.mark(2);  // 控制计算完成
    writeActuators();
    trace.mark(3);  // 执行器输出完成
}
```

`clock_gettime(CLOCK_MONOTONIC)` 在 Linux 上通过 vDSO 实现，不需要真正的系统调用，延迟约 20-30 纳秒。环形缓冲区预分配在栈上，写入只涉及一次时间读取和一次内存写入，对实时路径的干扰可以忽略。

### SMI（System Management Interrupt）的影响

硬件级延迟中最隐蔽的来源是 SMI——BIOS 的系统管理中断。SMI 会暂停所有 CPU 核心（包括隔离核），持续时间从几十微秒到几百毫秒不等。常见的 SMI 触发源包括：温度管理、ECC 内存纠错、USB 传统模式。

`hwlatdetect` 可以检测 SMI 的存在和持续时间。如果检测到 SMI 延迟超过控制周期的 10%，需要在 BIOS 中禁用可能的 SMI 源（如关闭 USB Legacy Support、禁用 BIOS 温度管理改用 OS 级管理）。

> **反事实推理**：如果 Linux 的调度延迟已经足够低（<10μs），为什么还需要 CPU 隔离和 IRQ 亲和性？
> 因为调度延迟只是延迟来源的一部分。即使 PREEMPT_RT 内核保证了调度器的确定性，
> 中断处理、缓存冲刷和 SMI 仍然可以引入不可控的延迟。
> CPU 隔离消除了中断和其他线程的干扰，但无法消除 SMI——这是硬件层面的问题，需要硬件层面的解决方案。

---

## 实时安全的 C++ 标准库替代方案 ⭐⭐⭐

标准库中很多组件在实时路径上不安全，但完全避免标准库会严重降低开发效率。工程上的做法是识别哪些标准库组件可以在实时路径使用、哪些必须替代。

| 标准库组件 | 实时安全性 | 原因 | 替代方案 |
|-----------|-----------|------|---------|
| `std::vector`（已 reserve） | 安全 | 不触发 realloc | 直接使用 |
| `std::vector`（未 reserve） | 不安全 | `push_back` 可能 realloc | 预分配或用 `std::array` |
| `std::array` | 安全 | 栈分配，零动态分配 | 直接使用 |
| `std::string` | 不安全 | SSO 阈值以上分配堆内存 | 预分配 `std::string` 或固定 buffer |
| `std::map`/`std::set` | 不安全 | 每次插入分配节点 | 预分配的数组 + 排序，或 `std::pmr` |
| `std::unordered_map` | 不安全 | rehash 触发批量分配 | 预分配并 reserve |
| `std::shared_ptr` | 有条件安全 | 最后一个引用释放时析构 | 明确析构不在实时路径 |
| `std::mutex` | 不安全 | 阻塞等待，优先级反转 | 无锁数据结构或 `PI mutex` |
| `std::cout`/`printf` | 不安全 | I/O 系统调用，内部有锁 | 异步日志队列 |
| `std::chrono` | 安全 | 编译期计算或 vDSO | 直接使用 |
| `Eigen::Matrix3d` | 安全 | 栈分配 | 直接使用 |
| `Eigen::MatrixXd`（已 resize） | 安全 | 不再分配 | 预分配并固定尺寸 |
| `Eigen::MatrixXd`（首次 resize） | 不安全 | 堆分配 | 在初始化阶段完成 |

这张表的核心原则是：**实时路径上不允许任何可能触发系统调用的操作**。`malloc`/`free` 是系统调用（严格说是库调用，可能触发 `brk`/`mmap` 系统调用），I/O 是系统调用，锁争用可能导致线程阻塞。把这些操作推到初始化阶段或非实时线程，是实时 C++ 编程的核心纪律。

> **本质洞察**：实时编程不是"写更快的代码"，而是"消除最慢的可能性"。
> 平均延迟不重要，最大延迟才重要。
> 一个平均 100μs 但最大 10ms 的控制循环，不如平均 200μs 但最大 300μs 的控制循环。
> 实时性的本质是确定性，不是速度。

---

## 异常处理在实时路径中的角色 ⭐⭐⭐

### 工程问题：C++ 异常与实时性的矛盾

C++ 异常（`throw`/`catch`）在实时路径中的安全性是一个有争议的话题。异常的正常路径（no-throw path）在现代编译器实现中几乎零开销——GCC/Clang 使用基于表的异常处理（table-based exception handling），正常执行时不需要任何额外指令。但异常抛出路径的开销不可预测：栈展开（stack unwinding）需要遍历调用栈、执行析构函数、查找匹配的 catch 块，延迟从几十微秒到几毫秒不等。

| 路径 | 开销 | 可预测性 | 实时安全性 |
|------|------|---------|-----------|
| 正常路径（无异常抛出） | 零额外开销 | 完全可预测 | 安全 |
| 异常抛出路径 | 几十μs 到几 ms | 不可预测（取决于栈深度） | 不安全 |
| 异常规格说明（`noexcept`） | 零开销 | 完全可预测 | 安全 |

工程上的处理策略是：在实时路径的所有函数上标记 `noexcept`，用返回值或状态码替代异常来报告错误。

```cpp
// ❌ 实时路径使用异常
double computeTorque(const JointState& state) {
    if (state.position > limit) {
        throw std::runtime_error("joint limit exceeded");  // 不可预测延迟
    }
    return pid_.compute(state);
}

// ✅ 实时路径使用返回码
enum class ControlStatus { Ok, JointLimit, SolverFailed, Timeout };

ControlStatus computeTorque(const JointState& state, double& torque) noexcept {
    if (state.position > limit) {
        return ControlStatus::JointLimit;  // 确定性返回
    }
    torque = pid_.compute(state);
    return ControlStatus::Ok;
}
```

`noexcept` 标记不仅是文档——它让编译器知道这个函数不会抛出异常，从而可以省略异常处理表的生成，减少代码体积。更重要的是，如果 `noexcept` 函数内部确实抛出了异常，程序会调用 `std::terminate()` 而非尝试栈展开——这是一个可预测的结果（虽然是终止），比不可预测的栈展开更适合安全关键系统。

> **反事实推理**：如果 C++ 异常的抛出路径延迟是 O(1) 可预测的，实时编程能简单多少？
> 我们不需要返回码，不需要 `expected<T, E>`，不需要 `noexcept` 审计。
> 异常的 zero-cost 设计（正常路径零开销、抛出路径高开销）是为通用编程优化的——
> 假设异常是"罕见的"，所以把开销从正常路径转移到异常路径。
> 但在实时系统中，最坏情况决定一切，这个设计假设不再成立。

### `std::expected`（C++23）作为返回码的类型安全替代

C++23 的 `std::expected<T, E>` 提供了比裸返回码更安全的错误处理方式——它要求调用者显式处理错误，编译器会在未处理时发出警告：

```cpp
#include <expected>

std::expected<double, ControlStatus> computeTorque(
    const JointState& state) noexcept {
    if (state.position > limit) {
        return std::unexpected(ControlStatus::JointLimit);
    }
    return pid_.compute(state);
}

// 调用端必须处理两种情况
auto result = computeTorque(state);
if (result.has_value()) {
    applyTorque(result.value());
} else {
    handleError(result.error());  // 编译器鼓励处理错误
}
```

`std::expected` 在栈上存储，不涉及动态分配，`noexcept` 友好，是实时 C++ 中替代异常和裸返回码的推荐方案。

---

## 实时系统的分层错误处理架构 ⭐⭐⭐

在完整的机器人控制系统中，错误处理不是单一策略，而是分层架构。每一层有不同的延迟预算和错误响应方式：

| 层级 | 延迟预算 | 错误检测方式 | 错误响应 |
|------|---------|------------|---------|
| **硬件层**（电机驱动、传感器） | <100 μs | 通信超时、CRC 校验 | 使用上一帧数据，计数超时次数 |
| **控制层**（QP 求解、PD 控制） | <1 ms | 求解器状态码、关节限位 | 降级到安全命令 |
| **规划层**（MPC、步态规划） | <10 ms | 约束可行性、轨迹平滑度 | 延长当前步态周期 |
| **感知层**（状态估计、地图） | <50 ms | 协方差爆炸、创新值异常 | 重置滤波器 |
| **系统层**（ROS2 节点、进程） | <1 s | 心跳超时、资源耗尽 | 重启节点、紧急停机 |

每一层的错误处理必须在其延迟预算内完成。控制层不能等待规划层重新规划——它必须立即用降级策略（如保持上一帧输出、PD 阻尼）响应。

```cpp
// 分层错误处理示例
struct ControlOutput {
    Eigen::VectorXd torques;
    ControlStatus status;
};

ControlOutput controlLoop(const SensorData& data) noexcept {
    // 层 1：传感器数据检查
    if (!data.is_valid()) {
        return {last_safe_torque_, ControlStatus::SensorTimeout};
    }

    // 层 2：状态估计
    auto state = estimator_.update(data);
    if (!state.has_value()) {
        return {last_safe_torque_, ControlStatus::EstimatorFailed};
    }

    // 层 3：QP 求解
    auto qp_result = wbc_.solve(state.value());
    if (qp_result.status != QpStatus::Optimal) {
        // 降级：使用阻尼 PD 控制
        return {computeDampingTorque(state.value()), ControlStatus::QpDegraded};
    }

    last_safe_torque_ = qp_result.torques;
    return {qp_result.torques, ControlStatus::Ok};
}
```

> **类比**：分层错误处理像飞机的冗余系统。
> 液压失效时切换到电动备份（层级降级），电动也失效时切换到手动控制（更深层降级）。
> 每一层的降级都在更高延迟预算内提供更简单但更安全的替代方案。
> 机器人控制中，QP 失败时降级到 PD，PD 失败时降级到零力矩，最后才是紧急停机。
