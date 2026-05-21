# CUDA 基础概念与 Thrust 库

> **难度**：⭐⭐⭐⭐ | **建议用时**：2 周 | **前置要求**：并行编程框架 并行框架，内存分配策略与pmr 内存分配策略，缓存优化与数据布局 缓存优化与数据布局

---

## 前置自测

答不出两题以上，建议先复习数据并行、归约、SoA 布局、内存带宽和 CMake target-based 构建。
GPU 编程不是“把循环丢到显卡上”。
它要求你理解哪些计算足够规则，哪些数据值得传输，哪些中间结果应该留在 GPU 上。

1. GPU 为什么适合点云逐点变换，却不一定适合复杂分支状态机？
2. Thread、Block、Grid 分别是什么？
3. CPU 到独立 GPU 的数据传输为什么可能抵消计算加速？
4. `thrust::device_vector<T>` 和 `std::vector<T>` 的心智模型有什么相似和不同？
5. `transform`、`reduce`、`sort`、`scan`、`reduce_by_key` 分别对应哪些 SLAM 操作？
6. 为什么体素下采样常用“key 排序 + 按 key 归约”的 GPU 形式？
7. Unified Memory 为什么易用？为什么仍然需要理解数据迁移？
8. Jetson 这类共享内存平台和桌面独立 GPU 的数据传输瓶颈有什么不同？
9. Thrust 什么时候足够，什么时候需要手写 CUDA kernel 或 CUB？
10. CMake 中启用 CUDA 语言和只链接一个 CUDA 库有什么区别？

---

## 本章目标

学完本章，你将能够：

- 建立 CUDA 的基本心智模型：host/device、thread/block/grid、SIMT、kernel launch。
- 判断一个 SLAM 子任务是否适合 GPU：数据规模、规则性、算术强度、传输成本和延迟要求。
- 区分 global memory、shared memory、register、host memory 和 unified memory 的角色。
- 使用 Thrust 的 `device_vector`、`transform`、`remove_if`、`sort`、`reduce`、`inclusive_scan`、`reduce_by_key` 表达标准数据并行模式。
- 用 Thrust 实现点云坐标变换、NaN 过滤、残差求和和体素质心归约。
- 理解 GPU 加速中的端到端计时：上传、计算、下载、同步都要计入。
- 编写最小 CMake CUDA target，并知道如何设置 CUDA 架构与编译选项。
- 识别常见错误：隐式同步、频繁小 kernel、host/device lambda 限制、设备内存生命周期和结果未同步。
- 完成 Mini SLAM Thrust Kernels：GPU 点云变换、范围过滤、体素 key 排序和体素质心计算。

**知识树**：

```text
CUDA 基础与 Thrust
├── GPU 适合什么（37.1）
│   ├── CPU vs GPU 设计哲学
│   ├── SIMT 与延迟隐藏
│   └── 盈亏平衡点分析
├── CUDA 执行模型（37.2 - 37.3）
│   ├── Thread / Block / Grid
│   ├── SM / Warp / Occupancy
│   ├── GPU 内存层级（global/shared/register）
│   ├── coalesced access
│   └── Unified Memory
├── Thrust 标准并行模式（37.4 - 37.9）
│   ├── device_vector / host_vector
│   ├── transform（点云变换）
│   ├── remove_if（过滤）
│   ├── reduce / transform_reduce（归约）
│   ├── scan（前缀和）
│   └── sort_by_key + reduce_by_key（体素化）
├── 工程集成（37.10 - 37.17）
│   ├── CMake CUDA 集成
│   ├── 错误处理与同步
│   ├── GPU 工作区复用
│   └── CUDA events 与端到端 benchmark
└── CUDA 12.x 新特性概览
    ├── CUDA Graphs
    └── Cooperative Groups
```

**本章在课程中的位置**：并行编程框架 讲 CPU 多线程并行，缓存优化与数据布局 讲 CPU cache 与数据布局。
本章把数据并行推到 GPU。
GPU 的优势不是单个线程快，而是大量线程同时执行规则计算。
因此，GPU 加速首先是数据组织问题，其次才是 API 问题。

---

## 37.1 GPU 适合什么：规则的大规模数据并行 ⭐⭐

> **这一节解决什么问题**：GPU 不是"更多核心的 CPU"——为什么 GPU 需要上万线程？什么样的计算才能真正受益于 GPU？

### GPU 不是"更多核心的 CPU"：从 SIMT 和延迟隐藏理解 GPU 的设计哲学

初学者最常犯的认知错误是把 GPU 想成"一个有几千个核心的 CPU"。如果 CPU 有 8 个核心、每个核心 4GHz，那 GPU 就是 4096 个核心、每个核心 1.5GHz——好像只是"数量更多但每个更慢"。这个类比从根本上是错的，理解为什么是错的，是正确使用 GPU 的前提。

**CPU 和 GPU 的设计目标完全不同**。CPU 被优化为"让单个线程尽可能快地跑完"——为此它投入了大量晶体管在乱序执行（out-of-order execution）、分支预测（branch prediction）、大容量缓存（L1 64KB + L2 256KB + L3 数十 MB per core）上。这些机制让 CPU 在面对复杂控制流和不规则数据访问时仍然高效。代价是每个核心占用大量芯片面积，所以核心数量有限（桌面 8-24 核，服务器 64-128 核）。

GPU 被优化为"让大量线程的总吞吐最大化"——为此它把每个核心设计得极其简单（没有复杂的乱序执行和分支预测），用省下来的芯片面积堆叠大量计算单元。一个现代 GPU 可能有 10000+ 个 CUDA core，但每个 core 只是一个简单的算术逻辑单元（ALU），没有独立的指令调度器和分支预测器。GPU 的应对策略不是"让每个线程跑得快"，而是"当一个线程等待内存时，立刻切换到另一个线程"——这就是**延迟隐藏**（latency hiding）。

**关键数字对比**：

| 指标 | CPU（如 i9-13900K） | GPU（如 RTX 4090） | 差异倍数 |
|------|---------------------|---------------------|---------|
| 核心数 | 24（8P+16E） | 16384 CUDA cores | ~680x |
| 单线程频率 | 5.8 GHz | 2.52 GHz | 0.43x |
| L1 缓存/核 | 80 KB | 128 KB/SM（共享） | - |
| L2 缓存总量 | 36 MB | 72 MB | 2x |
| 内存带宽 | ~90 GB/s | ~1008 GB/s | ~11x |
| 单线程延迟 | 极低（深流水线+乱序） | 高（简单有序执行） | - |
| 适合的并行度 | 10-100 线程 | 10000-100000 线程 | ~1000x |

注意最后一行：GPU 不只是"核心更多"，它**要求**的线程数也是 CPU 的 1000 倍以上。如果你只有 200 个独立任务，GPU 的 99% 的计算单元在空闲——还不如用 CPU。只有当问题天然能分解成数万个独立的小任务时，GPU 的架构优势才能体现。

这就是为什么下面的适合度判断不只看"有没有并行性"，还要看"并行度是否足够大"和"计算模式是否规则"。

### 工程问题：SLAM 里不是所有慢点都适合 GPU

SLAM 系统中常见任务：

| 任务 | 是否适合 GPU | 原因 |
|------|--------------|------|
| 点云坐标变换 | 适合 | 每个点独立、计算规则 |
| NaN / 距离过滤 | 适合 | 每个点独立 |
| 残差计算 | 适合 | 每个匹配对独立 |
| 体素 key 计算 | 适合 | 每个点独立 |
| 体素分组归约 | 适合但需要排序/归约 | 标准并行模式 |
| 图优化稀疏求解 | 依赖库和问题结构 | 稀疏依赖复杂 |
| 回环检测决策 | 不一定 | 分支多、任务不规则 |
| 状态机和模式切换 | 不适合 | 控制逻辑强 |
| ROS2 通信 | 不适合直接 GPU 化 | 受中间件和生命周期约束 |

GPU 适合把相同或相似操作应用到大量元素。
如果任务本身充满分支、锁、随机小对象和复杂依赖，GPU 未必是好选择。

### 反面失败：把小循环搬到 GPU

```text
输入：200 个点
操作：每点一次简单坐标变换
流程：CPU 上传 -> GPU kernel -> CPU 下载
```

GPU 计算可能只需要几微秒。
但 kernel launch 和数据传输可能远大于计算。
最终比 CPU 更慢。

GPU 加速需要足够大的数据规模，或者需要把多个阶段都留在 GPU 上。

> **本质洞察**：GPU 加速的本质不是"让每个线程更快"——GPU 的单线程实际上比 CPU 慢得多。GPU 的优势是**用大量轻量线程的总吞吐覆盖调度和传输成本**。这意味着 GPU 加速有一个"盈亏平衡点"：数据规模必须足够大，大到并行计算节省的时间超过上传/下载/启动的固定开销。

GPU 和 CPU 的关系，类似于货轮和快递摩托车的关系。快递摩托车（CPU）送一个包裹很快，但一次只能送几个。货轮（GPU）装货卸货需要很长时间，但一次能运送成千上万个集装箱。如果你只需要送 3 个包裹，用货轮显然不合理——光装卸时间就比摩托车送完所有包裹还多。但如果要送 10 万个包裹，货轮就远比派 10 万辆摩托车高效。

如果把一个只有 200 个点的小循环搬到 GPU 会怎样？kernel launch 开销约 5-20 微秒，H2D/D2H 传输至少几十微秒，而 200 个点的变换在 CPU 上可能只需 1-2 微秒。GPU 版本反而慢了 10-50 倍。这不是 GPU 的错，而是问题规模不匹配执行模型。

> ⚠️ **编程陷阱：kernel launch overhead 比计算本身还大**
> **错误做法**：为几百个元素的小循环启动 GPU kernel。
> **现象**：GPU 版本比 CPU 版本慢 10 倍以上。
> **根本原因**：kernel launch 有固定开销（通常 5-20 微秒），与计算量无关。如果计算本身不到 1 微秒，launch overhead 就是主要成本。
> **正确做法**：设置最小数据量阈值（如 5000 个点以上才走 GPU 路径），或把多个小 kernel 合并。

> 💡 **概念误区：认为 GPU 适合所有并行任务**
> **新手想法**："只要有循环就可以搬到 GPU 上。"
> **实际上**：GPU 适合规则的大规模数据并行——每个线程做相似计算、分支少、数据访问规则。复杂控制流（状态机、分支密集的决策逻辑）、小规模计算、频繁 CPU-GPU 交互的任务都不适合。

### 练习

1. **[估算题]** kernel launch 开销约 10 微秒，H2D 传输 10 万个 `Point3f`（1.2MB）约 0.1ms，GPU 变换计算约 0.05ms，D2H 下载约 0.1ms。CPU 串行变换约 0.8ms。算出端到端 GPU 耗时，判断是否比 CPU 快。最少需要多少点才能让 GPU 更快？
2. **[分类题]** 判断以下任务的 GPU 适合度（强适合/条件适合/不适合）：(a) 100K 点坐标变换；(b) 6x6 矩阵求解；(c) 500 个点的 NaN 过滤；(d) 1000 万像素的立体匹配；(e) ROS2 消息序列化。

### 抽象不变量：GPU 收益来自并行计算量大于调度和传输成本

端到端耗时可粗略写成：

$$
T_{\text{gpu}} =
T_{\text{upload}} +
T_{\text{launch}} +
T_{\text{compute}} +
T_{\text{download}} +
T_{\text{sync}}
$$

如果只比较 `T_compute`，会高估 GPU 收益。
对于桌面独立 GPU，上传和下载经常是关键。
对于共享内存嵌入式平台，数据迁移成本不同，但内存带宽和同步仍然重要。

### 工程边界：实时路径关注延迟，不只关注吞吐

GPU 很适合高吞吐批处理。
但实时系统还关心：

1. kernel launch 抖动。
2. GPU 是否被其他任务占用。
3. CPU/GPU 同步点。
4. 数据是否需要立刻回到 CPU 控制路径。
5. 失败时是否有 CPU fallback。

控制环通常不应依赖一个不可控 GPU 长尾。
SLAM 前端、地图更新、感知加速更常见。

### 代码验证：先写 CPU 参考

```cpp
#include <vector>

struct Point3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Transform3f {
    float m[12]{};
};

Point3f transformOne(const Point3f& p, const Transform3f& T) {
    return Point3f{
        T.m[0] * p.x + T.m[1] * p.y + T.m[2] * p.z + T.m[3],
        T.m[4] * p.x + T.m[5] * p.y + T.m[6] * p.z + T.m[7],
        T.m[8] * p.x + T.m[9] * p.y + T.m[10] * p.z + T.m[11]};
}

std::vector<Point3f> transformCpu(const std::vector<Point3f>& points,
                                  const Transform3f& T) {
    std::vector<Point3f> out;
    out.reserve(points.size());
    for (const Point3f& p : points) {
        out.push_back(transformOne(p, T));
    }
    return out;
}
```

GPU 版本必须和 CPU 参考比较正确性。
性能优化没有正确性基线，后续调试会非常困难。

---

## 37.2 CUDA 执行模型：Thread、Block、Grid ⭐⭐

### 工程问题：GPU 线程很多，但每个线程很轻

CUDA kernel 由大量线程执行。
线程组织为：

```text
Grid
  Block 0
    Thread 0
    Thread 1
    ...
  Block 1
    Thread 0
    Thread 1
    ...
```

每个线程通常处理一个或几个数据元素。
点云变换中，线程 `i` 可以处理点 `i`。

### 抽象不变量：每个线程要能独立决定自己的数据索引

手写 kernel 常见索引：

```cpp
__global__ void transformKernel(const Point3f* input,
                                Point3f* output,
                                std::size_t n,
                                Transform3f T) {
    const std::size_t i =
        blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    output[i] = transformOneDevice(input[i], T);
}
```

`blockIdx`、`blockDim`、`threadIdx` 共同决定全局索引。
边界检查必须保留，因为线程总数通常向上取整。

### 反面失败：把 CPU 线程心智套到 GPU

CPU 线程通常承担复杂任务：

```text
读取队列
处理一帧
写入地图
等待条件变量
```

GPU 线程更像大量轻量执行单元：

```text
处理一个点
处理一个像素
处理一个残差
```

不要在 GPU 线程里设计复杂生命周期、动态任务队列或系统状态机。
那不是 GPU 的强项。

### GPU 计算模型的硬件基础：SM、Warp 和延迟隐藏

在转向 Thrust 之前，有必要从硬件角度理解 GPU 的执行模型——这不是 CUDA 语法问题，而是理解"为什么 GPU 需要大量线程"和"为什么某些操作在 GPU 上不高效"的物理基础。

**SM（Streaming Multiprocessor）是 GPU 的基本计算单元**。一个 GPU 芯片包含多个 SM（消费级 GPU 通常有 30-80 个，数据中心 GPU 可能超过 100 个）。每个 SM 内部包含多组 CUDA core（整数/浮点运算单元）、寄存器文件、shared memory、warp 调度器和特殊函数单元。SM 之于 GPU，类似于核心之于 CPU——但有一个根本区别：CPU 核心设计为快速执行单个线程的复杂指令流（乱序执行、分支预测、大缓存），SM 设计为同时管理成百上千个简单线程。

**Warp 是 GPU 的基本调度单位**。一个 block 中的线程被分成每 32 个一组的 warp（在当前所有 NVIDIA GPU 上，warp 大小固定为 32）。同一 warp 中的 32 个线程以 SIMT 方式执行——它们共享同一个程序计数器（PC），每个时钟周期执行同一条指令，但可以操作不同的数据。这就是为什么 GPU 被称为 SIMT（Single Instruction, Multiple Threads）而不是严格的 SIMD：每个线程有自己的寄存器和执行状态，可以在分支处"分叉"（diverge），但分叉会导致效率下降（warp divergence，见 37.15 节）。

**为什么 GPU 需要大量线程？从延迟隐藏（latency hiding）角度理解**。这是理解 GPU 编程模型的最关键洞察。

GPU 的 global memory 延迟约 400-800 个时钟周期——这比 CPU 访问主存还慢（因为 GPU 的主存通常是 GDDR 或 HBM，延迟和 DDR 在同一量级，但 GPU 时钟频率更高，所以按周期计算延迟更大）。如果一个 warp 发起内存读取请求，然后等待 400 个周期直到数据到达，这 400 个周期 SM 完全在空转——这对大量计算单元来说是极大的浪费。

GPU 的解决方案不是像 CPU 那样用复杂的缓存层级和预取机制来减少延迟（GPU 的 L1/L2 缓存远小于 CPU），而是**用大量活跃 warp 来隐藏延迟**。具体机制如下：

1. 一个 SM 上可以同时驻留多个 warp（通常 32-64 个 warp，即 1024-2048 个线程）。
2. 当 warp A 发起内存请求后，warp A 被挂起等待数据。
3. SM 的 warp 调度器**立即切换到另一个就绪的 warp B**，执行 warp B 的指令。
4. 当 warp B 也因为内存请求而挂起时，切换到 warp C...
5. 如果活跃 warp 数量足够多，到 warp A 的数据从内存返回时，SM 已经执行了其他 warp 的足够多指令——warp A 可以无等待地继续执行。

**关键区别**：CPU 的上下文切换开销很大（保存/恢复寄存器、刷新流水线、可能触发 TLB miss），所以 CPU 只维护少量线程。GPU 的 warp 切换**零开销**——所有 warp 的寄存器都常驻在 SM 的寄存器文件中，切换只需要调度器选择一个新 warp，一个时钟周期即可完成。这就是为什么 GPU 需要大量线程：不是因为有那么多并行工作要做，而是为了在等待内存时有其他 warp 可以执行，把内存延迟"藏起来"。

**Occupancy 的含义**：occupancy 是"SM 上实际活跃的 warp 数 / SM 能驻留的最大 warp 数"。更高的 occupancy 意味着更多 warp 可以参与延迟隐藏。但 occupancy 受三个资源的约束：(1) 每个线程使用的寄存器数量——寄存器文件是固定大小的，每个线程用的寄存器越多，能驻留的线程总数越少；(2) shared memory 用量——如果每个 block 使用大量 shared memory，SM 上能放的 block 数减少；(3) block 大小——SM 对同时驻留的 block 数有上限。这三个约束中的最严格者决定了实际 occupancy。

**这对 SLAM 意味着什么？** SLAM 中的点云变换、残差计算等操作通常是内存密集型（memory-bound）——每个点只做少量乘加运算，但需要读写多个 float。对于这类操作，GPU 的加速来源不是计算能力（CUDA core 数量），而是**内存带宽**（HBM/GDDR 的总带宽可以达到 400-2000 GB/s，远超 CPU 的 30-80 GB/s）。但要利用这个带宽，必须有足够多的活跃 warp 来产生足够多的并发内存请求——这又回到了"为什么需要大量线程"和"为什么小规模数据不适合 GPU"的核心问题。

### 工程边界：本课程优先 Thrust

手写 kernel 能帮助理解。
但本章的工程重点是 Thrust。
Thrust 把许多标准并行模式封装成类似 STL 的算法。
对于点云变换、过滤、排序、归约，Thrust 通常已经足够表达。

只有当标准算法组合无法满足需求，或者 profiling 明确显示 Thrust 组合不是瓶颈最优时，再考虑手写 kernel 或更底层的 CUB。

> ⚠️ **编程陷阱：host/device 内存指针混淆**
> **错误做法**：把 `thrust::device_vector` 的 `data()` 指针当成 host 指针传给 CPU 函数。
> **现象**：segfault 或非法内存访问，且只在有 GPU 的机器上崩溃。
> **根本原因**：`device_vector::data()` 返回的是 device 指针，CPU 代码不能直接解引用。这是 host/device 内存空间隔离的基本规则。
> **正确做法**：用 `thrust::copy` 或 `cudaMemcpy` 把数据从 device 拷贝到 host 后再在 CPU 上使用。

> 🧠 **思维陷阱：把 GPU 线程类比为 CPU 线程**
> **新手想法**："GPU 线程和 CPU 线程一样，只是数量更多。"
> **实际上**：GPU 线程比 CPU 线程轻量得多——没有独立栈、没有复杂调度优先级、不适合做复杂状态管理。GPU 线程更像流水线上的工人：每个人做一个简单重复的操作，大量工人同时工作。CPU 线程更像项目经理：每个人管理复杂任务，数量少但能力强。
> **正确思维**：GPU 线程 = 简单数据元素处理器。不要在 GPU 线程里设计队列、状态机或动态任务分发。

### 练习

1. **[概念题]** 解释 Thread、Block、Grid 的层次关系。如果有 100K 个点需要变换，Block 大小是 256，需要多少个 Block？Grid 大小是多少？
2. **[代码题]** 写一个手写 kernel 的索引计算 `size_t i = blockIdx.x * blockDim.x + threadIdx.x;`，解释为什么还需要 `if (i >= n) return;` 的边界检查。

---

## 37.3 GPU 内存模型与数据传输 ⭐⭐

### 工程问题：数据在哪里，比计算怎么写更重要

CUDA 程序通常有 host 和 device：

| 名称 | 含义 |
|------|------|
| Host | CPU 侧代码和内存 |
| Device | GPU 侧代码和显存 |
| Host-to-Device | CPU 数据上传到 GPU |
| Device-to-Host | GPU 结果下载到 CPU |

如果每个阶段都来回传输，GPU 算得再快也可能没收益。

### 反面失败：每个小步骤都下载结果

```text
CPU -> GPU: 上传原始点云
GPU: 去除 NaN
GPU -> CPU: 下载过滤结果
CPU -> GPU: 上传过滤结果
GPU: 坐标变换
GPU -> CPU: 下载变换结果
CPU -> GPU: 上传变换结果
GPU: 体素化
GPU -> CPU: 下载体素结果
```

这种流程把 GPU 当成单个函数加速器。
传输成本会吞噬收益。

更好的策略：

```text
CPU -> GPU: 上传原始点云
GPU: NaN 过滤
GPU: 坐标变换
GPU: 体素 key
GPU: sort/reduce_by_key
GPU -> CPU: 下载最终结果
```

### 抽象不变量：上传后尽量在 GPU 上完成连续阶段

设计 GPU pipeline 时先画数据流：

```text
哪些数据必须来自 CPU？
哪些中间结果可以留在 GPU？
最终 CPU 真正需要什么？
是否只需要一个位姿、一个 cost、一个小矩阵？
```

如果最终 CPU 只需要 6x6 Hessian 和 6x1 梯度，就不要把每个点残差全部下载。
在 GPU 上归约后下载小结果更合理。

### GPU 内存层级：global、shared、register 的分工

Host/device 只回答“数据在 CPU 还是 GPU”。
真正写 kernel 或分析 Thrust 性能时，还要知道 GPU 内部有不同层级：

| 层级 | 生命周期 | 访问者 | 典型用途 | 主要风险 |
|------|----------|--------|----------|----------|
| register | 单个线程 | 当前 thread | 临时标量、局部坐标、残差 | 使用过多会降低 occupancy |
| shared memory | 一个 block | 同 block 内所有 thread | tile、局部归约、邻域缓存 | bank conflict、同步错误 |
| global memory | 整个 device | 所有 kernel/thread | 点云数组、体素 key、输出结果 | 非合并访问导致带宽浪费 |
| constant memory | 整个 device | 只读广播 | 小型标定参数、固定矩阵 | 大量随机访问不合适 |
| unified memory | CPU/GPU 统一地址模型 | host 和 device | 原型验证、共享内存平台 | 页面迁移和同步成本不透明 |

SLAM 点云处理中最常见的瓶颈不是浮点乘加，而是 global memory 带宽。
例如点云坐标变换每个点只做少量矩阵乘法，却要读取 `x,y,z,intensity` 并写回结果。
如果线程访问的地址连续，GPU 可以把相邻线程的访问合并成少数几次内存事务。
如果每个线程随机访问点，带宽会被浪费在大量离散事务上。

这就是 coalescing 的核心直觉：

```text
相邻 thread 访问相邻地址 -> 合并访问 -> 带宽利用率高
相邻 thread 访问随机地址 -> 分散访问 -> 带宽利用率低
```

shared memory 的意义不是“比 global memory 永远快”这么简单。
它适合一个 block 内多个线程反复使用同一小块数据。
如果数据只读一次，把它搬进 shared memory 反而增加步骤。
如果多个线程访问 shared memory 的不同地址映射到同一个 bank，还会产生 bank conflict。

register 则是最接近线程的临时存储。
每个线程使用太多 register，会让同一个 SM 上能同时驻留的 warp 数下降。
这会降低 occupancy，使 GPU 难以用其他 warp 隐藏内存延迟。

因此，GPU 内存设计的教学结论是：

1. 先保证 global memory 访问连续。
2. 再考虑是否有重复访问值得放入 shared memory。
3. 不要让每个线程维护过大的局部状态。
4. 用 profiling 看带宽、occupancy 和 memory transaction，而不是凭直觉判断。

### Unified Memory 的边界

Unified Memory 允许 CPU 和 GPU 使用同一个指针模型。
它降低编程复杂度。
但并不意味着数据迁移成本消失。
在独立 GPU 上，页面仍可能在 CPU/GPU 之间迁移。
在共享内存 SoC 上，物理拷贝成本不同，但同步和带宽仍然存在。

教学结论：

1. Unified Memory 适合原型和平台共享内存场景。
2. 性能关键路径仍要测迁移和同步。
3. 不要把易用性误认为零成本。

> ⚠️ **编程陷阱：每个小步骤都做 H2D/D2H 往返传输**
> **错误做法**：GPU 过滤后下载到 CPU，再上传做变换，再下载做体素化，再上传做归约。
> **现象**：端到端 GPU 版本比纯 CPU 版本还慢。profiling 显示 80% 时间花在数据传输上。
> **根本原因**：PCIe 传输带宽有限（通常 10-16 GB/s），每次传输还有固定延迟。N 次往返的开销是 N 倍，而且阻止了 CPU-GPU 重叠执行。
> **正确做法**：上传一次后，在 GPU 上连续完成所有阶段（过滤->变换->体素化->归约），最后只下载最终小结果（位姿、Hessian 等）。

> 💡 **概念误区：认为 Unified Memory 消除了传输成本**
> **新手想法**："用了 Unified Memory，数据自动在 CPU/GPU 之间同步，不需要手动管传输了。"
> **实际上**：Unified Memory 把显式传输变成了隐式页面迁移。在独立 GPU 上，首次访问一页数据仍然触发页面迁移（page fault + migration），延迟可能比显式 `cudaMemcpy` 更不可预测。在性能关键路径上，显式传输通常更可控。

### Kernel launch overhead 的量化分析

"kernel launch overhead"经常被提到但很少被量化。理解它的构成有助于判断何时 GPU 有收益。

**一次 kernel launch 的全过程**：从 CPU 调用 `kernelFunc<<<grid, block>>>(...)` 到 GPU 上第一条指令开始执行，中间经历多个步骤：

1. **CPU 侧：参数打包与驱动程序调用**（~2-5 微秒）。CUDA runtime 把 kernel 参数复制到一个 staging buffer，调用 NVIDIA 驱动程序的 ioctl 接口。这个过程涉及用户态到内核态的切换，以及驱动内部的命令队列管理。
2. **CPU 侧：命令入队**（~1-3 微秒）。驱动程序把 launch 命令写入 GPU 的命令队列（ring buffer）。这个操作通过 MMIO（Memory-Mapped I/O）写入 GPU 可见的内存区域。
3. **GPU 侧：命令读取与调度**（~2-5 微秒）。GPU 的命令处理器从命令队列读取命令，解析 grid/block 维度，开始向各 SM 分发 block。
4. **GPU 侧：warp 初始化**（~1-2 微秒）。SM 为分配到的 block 初始化 warp 上下文——分配寄存器、设置 shared memory、加载第一条指令。

总计约 **5-20 微秒**，取决于 GPU 代次、驱动版本和系统负载。这个开销是**固定的**——与 kernel 中的数据量无关。一个处理 100 个点的 kernel 和一个处理 100 万个点的 kernel，launch overhead 相同。

**量化盈亏平衡点**：假设 kernel launch overhead 为 10 微秒，H2D 传输 $N$ 个 `Point3f`（12 字节/点）在 PCIe 3.0 x16（约 12 GB/s 有效带宽）上耗时 $N \times 12 / (12 \times 10^9)$ 秒，GPU 计算每个点约 5ns。端到端 GPU 耗时为：

$$T_{\text{GPU}} = T_{\text{launch}} + T_{\text{H2D}} + T_{\text{compute}} + T_{\text{D2H}}$$

$$= 10\mu s + \frac{N \times 12}{12 \times 10^9} + N \times 5ns + \frac{N \times 12}{12 \times 10^9}$$

CPU 串行变换每个点约 15ns：$T_{\text{CPU}} = N \times 15ns$。

令 $T_{\text{GPU}} < T_{\text{CPU}}$ 求解 $N$：大约在 $N \approx 5000-10000$ 时两者相当。低于此阈值，GPU 因固定开销反而更慢。

**这个分析对 SLAM 的意义**：LiDAR 点云通常有 10K-300K 个点，远超盈亏平衡点——GPU 有明确收益。但某些子任务（如对 50 个关键帧的小矩阵操作）点数太少，应该留在 CPU 上。

### PCIe 带宽瓶颈：CPU-GPU 通信的物理限制

PCIe 是独立 GPU（桌面/服务器）和 CPU 之间的物理连接。它的带宽是 CPU-GPU 协作系统的硬天花板。

| PCIe 版本 | 单通道带宽 | x16 总带宽（单向） | x16 有效带宽 |
|-----------|-----------|-------------------|-------------|
| PCIe 3.0 | 1 GB/s | 16 GB/s | ~12 GB/s（编码开销） |
| PCIe 4.0 | 2 GB/s | 32 GB/s | ~25 GB/s |
| PCIe 5.0 | 4 GB/s | 64 GB/s | ~50 GB/s |

而 GPU 的 global memory 带宽：RTX 3090 约 936 GB/s（GDDR6X），A100 约 2039 GB/s（HBM2e）。这意味着 GPU 内部计算和访存速度远超 CPU-GPU 之间的传输速度——**PCIe 是系统瓶颈，不是 GPU 本身**。

传输 100K 个 `Point3f`（每点 12 字节，共 1.2MB）在 PCIe 3.0 上需要约 $1.2 \times 10^6 / (12 \times 10^9) \approx 0.1ms$。如果每帧需要上传一次、下载一次，传输就占用约 0.2ms。对于 100ms 帧预算这不是问题，但对于追求极低延迟的路径，传输开销可能成为主导因素。

**嵌入式共享内存平台（Jetson）的不同**：Jetson 系列使用统一内存架构——CPU 和 GPU 共享同一块物理 DRAM。没有 PCIe，也没有物理数据传输。但这不意味着零开销：CPU 和 GPU 仍然需要缓存一致性管理和 TLB（页表）同步，而且共享的内存带宽（通常 25-100 GB/s）要同时服务 CPU 和 GPU。38.9 节会详细讨论 Jetson 部署的注意事项。

### 练习

1. **[设计题]** 画出一个 LiDAR 前端的 GPU 数据流图：原始点云上传 -> NaN 过滤 -> 去畸变 -> 体素 key -> 排序 -> 归约 -> 下载质心。标注哪些数据留在 GPU，哪些需要回到 CPU。
2. **[分析题]** 对比 `cudaMemcpy` 和 Unified Memory 在以下场景的优劣：(a) 100K 点一次性上传；(b) 1000 个小对象逐个访问；(c) Jetson 共享内存平台。
3. **[估算题]** 一个 SLAM 前端每帧处理 50K 个点（每点 16 字节），需要上传一次、下载 6x6 Hessian 和 6x1 梯度。在 PCIe 3.0 系统上，估算传输总耗时。如果改为每帧上传+下载全部点，传输耗时增加多少？

---

## 37.4 Thrust 心智模型：GPU 侧 STL ⭐⭐

### 工程问题：SLAM 工程师不应该从手写 kernel 起步

Thrust 的 API 很像 STL：

| STL | Thrust |
|-----|--------|
| `std::vector` | `thrust::host_vector`, `thrust::device_vector` |
| `std::transform` | `thrust::transform` |
| `std::sort` | `thrust::sort` |
| `std::reduce` | `thrust::reduce` |
| `std::remove_if` | `thrust::remove_if` |
| `std::inclusive_scan` | `thrust::inclusive_scan` |

这让你先用算法组合表达数据并行。
只有热点不满足时，再下沉到 kernel。

### 代码验证：device_vector 与 transform

```cpp
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <vector>

struct ScaleFunctor {
    float scale = 1.0f;

    __host__ __device__
    float operator()(float x) const {
        return x * scale;
    }
};

void scaleOnGpu(const std::vector<float>& input,
                std::vector<float>& output,
                float scale) {
    thrust::device_vector<float> d_input(input.begin(), input.end());
    thrust::device_vector<float> d_output(input.size());

    thrust::transform(d_input.begin(),
                      d_input.end(),
                      d_output.begin(),
                      ScaleFunctor{scale});

    output.resize(input.size());
    thrust::copy(d_output.begin(), d_output.end(), output.begin());
}
```

这个例子包含上传、计算、下载。
真实 benchmark 要分别计时。

### 工程边界：functor 要能在 device 侧执行

Thrust functor 如果在 GPU 执行，`operator()` 需要可在 device 调用。
不要在其中使用：

1. 普通 `std::cout`。
2. 主机侧指针。
3. 不能在 device 执行的标准库函数。
4. 捕获复杂 C++ 对象的 lambda。
5. 动态分配和异常。

尽量让 functor 是小型、平凡、只含数值字段的对象。

> ⚠️ **编程陷阱：在 device functor 中使用 host-only 函数**
> **错误做法**：在 `__device__` functor 的 `operator()` 中调用 `std::cout`、`std::string` 或标准库 I/O。
> **现象**：nvcc 编译报 "calling a __host__ function from a __device__ function is not allowed"。
> **根本原因**：GPU 线程不能执行 host-only 代码。标准库的大部分函数（I/O、容器、异常）都只能在 host 上运行。
> **正确做法**：device functor 只使用基本数值类型、简单数学函数（`floorf`、`sqrtf` 等）和 CUDA 内置函数。调试信息用 `printf`（CUDA 支持有限的 device `printf`）或回到 host 后处理。

> **本质洞察**：Thrust 的价值不是"让 GPU 编程变简单"，而是**让你用算法组合表达数据并行模式，而不是用线程索引和内存地址表达**。`transform` 说的是"对每个元素做什么"，`reduce` 说的是"怎么合并结果"，`sort_by_key` 说的是"按什么顺序组织"。这些抽象和 STL 一致，让 SLAM 工程师可以用算法思维而非硬件思维设计 GPU 管线。

### 练习

1. **[代码题]** 写一个 Thrust functor，把点云的 x/y/z 同时缩放 0.5 倍。注意 `__host__ __device__` 标注和不能使用的 host 函数。
2. **[分析题]** `thrust::device_vector<T>` 的构造函数接受 host 迭代器时会触发 H2D 拷贝。在一个 10Hz 的帧处理循环中，每帧都 `thrust::device_vector<Point3f> d(input.begin(), input.end());` 会产生什么问题？如何改进？

---

## 37.5 GPU 点云坐标变换 ⭐⭐

### 工程问题：点云变换是最标准的数据并行任务

每个点独立执行：

$$
p'_i = R p_i + t
$$

没有点间依赖。
输出 `i` 只依赖输入 `i`。
这非常适合 `transform`。

### 代码验证：Thrust 点云变换

```cpp
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/transform.h>
#include <vector>

struct Point3f {
    float x;
    float y;
    float z;
};

struct Transform3f {
    float m[12];
};

struct TransformPoint {
    Transform3f T;

    __host__ __device__
    Point3f operator()(const Point3f& p) const {
        return Point3f{
            T.m[0] * p.x + T.m[1] * p.y + T.m[2] * p.z + T.m[3],
            T.m[4] * p.x + T.m[5] * p.y + T.m[6] * p.z + T.m[7],
            T.m[8] * p.x + T.m[9] * p.y + T.m[10] * p.z + T.m[11]};
    }
};

void transformGpu(const thrust::device_vector<Point3f>& input,
                  const Transform3f& T,
                  thrust::device_vector<Point3f>& output) {
    output.resize(input.size());
    thrust::transform(input.begin(),
                      input.end(),
                      output.begin(),
                      TransformPoint{T});
}

std::vector<Point3f>
transformGpuEndToEnd(const std::vector<Point3f>& input,
                     const Transform3f& T) {
    thrust::device_vector<Point3f> d_input(input.begin(), input.end());
    thrust::device_vector<Point3f> d_output;

    transformGpu(d_input, T, d_output);

    std::vector<Point3f> output(d_output.size());
    thrust::copy(d_output.begin(), d_output.end(), output.begin());
    return output;
}
```

这里故意把“GPU 内核阶段”和“端到端上传下载阶段”拆成两个函数。
`transformGpu()` 只处理已经在 GPU 上的数据，并把结果写入调用者提供的输出 buffer。
`transformGpuEndToEnd()` 才负责从 CPU 上传、调用 GPU 阶段、再下载。

这个拆分能防止一个常见误解：把包含上传下载的函数叫作“GPU 变换”，然后只测其中的 kernel 时间。
接口本身应该暴露数据边界，让 benchmark 能清楚地区分传输成本和计算成本。

### 工程边界：AoS 与 SoA

`Point3f` 是 AoS。
对于简单变换，它很直观。
但 GPU 更喜欢相邻线程访问相邻内存。
AoS 中 x/y/z 交错。
SoA 中所有 x 连续、所有 y 连续、所有 z 连续。

哪个更快要测。
如果后续 GPU 算法也是 SoA，直接保持 SoA 更可能获益。
如果边界大量使用 PCL/AoS，转换成本也要计入。

### 练习

1. **[代码题]** 实现 `transformGpuEndToEnd`：从 `std::vector<Point3f>` 上传到 GPU，用 Thrust `transform` 做坐标变换，下载回 `std::vector<Point3f>`。分别计时上传/计算/下载，打印各阶段占比。
2. **[正确性题]** 用 CPU 版本和 GPU 版本对同一组输入做变换，比较输出差异。最大差异应该在什么量级？用 `near(a, b, 1e-5f)` 做断言。

---

## 37.6 过滤：`remove_if` 与 compact ⭐⭐

### 工程问题：GPU 过滤不是简单地从 vector 中 erase

CPU 中可以：

```cpp
points.erase(std::remove_if(points.begin(), points.end(), pred), points.end());
```

Thrust 提供类似模式。
GPU 过滤本质是并行 predicate + compact。

### 代码验证：去除非法点

```cpp
#include <cmath>
#include <thrust/device_vector.h>
#include <thrust/remove.h>

struct IsInvalidPoint {
    float max_range2 = 100.0f;

    __host__ __device__
    bool operator()(const Point3f& p) const {
        const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
        return !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) ||
               r2 > max_range2;
    }
};

void removeInvalidGpu(thrust::device_vector<Point3f>& points,
                      float max_range2) {
    auto new_end = thrust::remove_if(points.begin(),
                                     points.end(),
                                     IsInvalidPoint{max_range2});
    points.erase(new_end, points.end());
}
```

注意 `erase` 操作在 device_vector 上会调整容器大小。
它不代表每个元素逐个搬到 CPU。
但仍然会触发设备侧数据移动。

### 工程边界：过滤后索引关系会改变

如果还有其他数组与点云一一对应：

```text
points[i]
timestamps[i]
rings[i]
labels[i]
```

只过滤 `points` 会破坏索引关系。
需要：

1. 使用 zip iterator 同步过滤多个数组。
2. 生成有效 mask，再按 mask compact 多个数组。
3. 只输出保留索引，后续统一处理。

数据布局越复杂，过滤越要小心。

> ⚠️ **编程陷阱：GPU 过滤后关联数组索引不一致**
> **错误做法**：只对 `points` 做 `remove_if`，没有同步过滤 `timestamps` 和 `labels` 数组。
> **现象**：过滤后 `points[i]` 对应的 `timestamps[i]` 是错误的——它们不再是同一个点的数据。
> **根本原因**：`remove_if` 移动了 `points` 中的元素，但没有同步移动关联数组。
> **正确做法**：使用 zip iterator 同步过滤多个数组，或先生成有效 mask，再用 mask 对所有数组做 compact。

### 练习

1. **[代码题]** 用 Thrust `remove_if` 过滤掉距离大于 100m 的点。输入是 `device_vector<Point3f>`，需要同时保持 `device_vector<float> timestamps` 的一致性。
2. **[分析题]** `thrust::remove_if` 和 `thrust::copy_if` 的区别是什么？在哪种场景下用 `copy_if` 更合适？

---

## 37.7 归约：残差和统计量 ⭐⭐

> **这一节解决什么问题**：上一节讲了如何在 GPU 上过滤数据。但 SLAM 的最终目标不是过滤——而是从大量数据中计算出小的汇总结果（cost、Hessian、梯度）。归约（reduce）就是"从大数据变成小结果"的核心操作，它是 GPU SLAM pipeline 中减少 CPU-GPU 传输量的关键环节。

### 工程问题：CPU 不需要下载每个残差

配准里常计算总 cost：

$$
J = \sum_i r_i^2
$$

如果 GPU 计算每个残差后全部下载到 CPU，再由 CPU 求和，会产生大量传输。
更合理的是 GPU 上归约成一个标量。

### 代码验证：残差平方和

```cpp
#include <thrust/device_vector.h>
#include <thrust/transform_reduce.h>

struct PointToPlaneResidual2 {
    Plane plane;

    __host__ __device__
    float operator()(const Point3f& p) const {
        const float r = plane.nx * p.x +
                        plane.ny * p.y +
                        plane.nz * p.z +
                        plane.d;
        return r * r;
    }
};

float costGpu(const thrust::device_vector<Point3f>& points,
              const Plane& plane) {
    return thrust::transform_reduce(points.begin(),
                                    points.end(),
                                    PointToPlaneResidual2{plane},
                                    0.0f,
                                    thrust::plus<float>{});
}
```

`transform_reduce` 把 map 和 reduce 合并。
这避免显式保存每个残差。

### 工程边界：浮点归约顺序不固定

并行归约改变加法顺序。
结果可能与 CPU 串行版本有微小差异。
测试应使用误差容忍。
如果需要确定性更强的结果，要设计固定分块归约，或者使用更高精度累加。

如果不在 GPU 上做归约，而是把所有残差下载到 CPU 再求和会怎样？假设有 50K 个残差，每个 4 字节，下载 200KB 数据约 0.02ms。但 GPU 归约只需要下载 4 字节（一个 float 结果），省去了 99.998% 的传输量。对于 Hessian 和梯度，GPU 归约从下载 50K 个 6x6 矩阵变成只下载一个 6x6 矩阵——传输量减少 5 万倍。

> ⚠️ **编程陷阱：GPU 归约结果未同步就使用**
> **错误做法**：调用 `thrust::reduce` 后立刻在 CPU 上使用返回值，没有确认 GPU 计算是否完成。
> **现象**：大部分时候结果正确，偶尔结果是上一帧的值或者随机值。
> **根本原因**：某些 Thrust 操作可能是异步的。如果在默认 stream 上，通常会隐式同步，但依赖这种行为是脆弱的。
> **正确做法**：在需要使用 GPU 结果的位置之前确保同步（`cudaDeviceSynchronize` 或 event synchronize）。

### 练习

1. **[代码题]** 用 `thrust::transform_reduce` 计算点云所有点到原点的距离平方和。比较与 CPU `std::accumulate` 版本的结果差异。
2. **[设计题]** 如果 CPU 优化器需要 `double` 精度的 Hessian，但 GPU 残差计算使用 `float`。设计一个混合精度归约方案：GPU `float` 残差 -> GPU `double` 局部累加 -> CPU 下载 `double` 结果。

---

## 37.8 scan：前缀和与压缩输出位置 ⭐⭐⭐

### 工程问题：并行过滤需要知道每个元素写到哪里

很多 GPU 算法使用前缀和。
例如给每个点一个保留标志：

```text
keep: 1 0 1 1 0
scan: 1 1 2 3 3
```

第 0 个保留点写到位置 0。
第 2 个保留点写到位置 1。
第 3 个保留点写到位置 2。

Thrust 的 `inclusive_scan` / `exclusive_scan` 可以表达这类操作。

### 代码验证：保留标志前缀和

```cpp
#include <thrust/device_vector.h>
#include <thrust/scan.h>

void prefixKeepFlags(const thrust::device_vector<int>& keep,
                     thrust::device_vector<int>& offsets) {
    offsets.resize(keep.size());
    thrust::exclusive_scan(keep.begin(), keep.end(), offsets.begin());
}
```

很多看似复杂的 GPU compaction、histogram、分桶，都能拆成：

```text
标记 -> scan -> scatter
```

Thrust 让这些模式更容易组合。

---

## 37.9 sort + reduce_by_key：GPU 体素下采样 ⭐⭐

> **这一节解决什么问题**：前面学了 transform（逐点计算）、remove_if（过滤）、reduce（全局归约）、scan（前缀和）。体素下采样把这些操作组合在一起，形成一个完整的 GPU 数据处理链路——这正是 38.2 节分析的"体素滤波为什么强适合 GPU"的实现方式。

### 工程问题：体素下采样需要把相同体素的点聚合

CPU 上做体素下采样最直接的方式是哈希表：

```text
voxel key -> sum/count
```

但 GPU 上大量线程同时更新哈希表非常复杂——哈希冲突需要原子操作或锁，这正是 GPU 最不擅长的模式。GPU 友好的替代方案是把"随机插入哈希表"变成"排序后连续归约"——这是一个重要的思想转变：**GPU 宁可多做一次 $O(n \log n)$ 的排序，也要避免随机内存访问和原子操作**，因为排序后的连续访问在 GPU 上比随机访问快 10-100 倍。

另一种常见思路是：

```text
每个点计算 voxel key
按 key 排序
相同 key 连续
对相同 key 做 reduce_by_key
```

这把随机插入问题变成排序和归约问题。
GPU 很擅长排序和归约。

### 抽象不变量：相同 key 连续后才能按 key 归约

流程：

```text
points -> keys
keys, points 按 keys 排序
reduce_by_key(keys, point_sum)
reduce_by_key(keys, count)
centroid = sum / count
```

如果 point 是 x/y/z，可以分别归约 sum_x、sum_y、sum_z。
也可以定义可加的结构体。

### 代码验证：体素 key 计算

```cpp
#include <thrust/device_vector.h>
#include <thrust/transform.h>

struct VoxelKeyFunctor {
    float inv_leaf = 1.0f;

    __host__ __device__
    std::uint64_t operator()(const Point3f& p) const {
        const auto ix = static_cast<std::uint32_t>(floorf(p.x * inv_leaf));
        const auto iy = static_cast<std::uint32_t>(floorf(p.y * inv_leaf));
        const auto iz = static_cast<std::uint32_t>(floorf(p.z * inv_leaf));
        return morton3DDevice(ix, iy, iz);
    }
};

thrust::device_vector<std::uint64_t>
computeVoxelKeys(const thrust::device_vector<Point3f>& points,
                 float leaf) {
    thrust::device_vector<std::uint64_t> keys(points.size());
    thrust::transform(points.begin(),
                      points.end(),
                      keys.begin(),
                      VoxelKeyFunctor{1.0f / leaf});
    return keys;
}
```

负坐标需要额外处理。
教学示例可以假设坐标已平移到非负范围。
真实地图必须定义坐标编码策略。

### sort_by_key

```cpp
#include <thrust/sort.h>

void sortPointsByKey(thrust::device_vector<std::uint64_t>& keys,
                     thrust::device_vector<Point3f>& points) {
    thrust::sort_by_key(keys.begin(), keys.end(), points.begin());
}
```

排序后，相同 key 的点连续。

### reduce_by_key

```cpp
#include <thrust/reduce.h>

struct AddPoint {
    __host__ __device__
    Point3f operator()(const Point3f& a, const Point3f& b) const {
        return Point3f{a.x + b.x, a.y + b.y, a.z + b.z};
    }
};

struct MakeOne {
    __host__ __device__
    int operator()(const Point3f&) const {
        return 1;
    }
};

struct DividePoint {
    __host__ __device__
    Point3f operator()(const thrust::tuple<Point3f, int>& item) const {
        const Point3f sum = thrust::get<0>(item);
        const int count = thrust::get<1>(item);
        const float inv = 1.0f / static_cast<float>(count);
        return Point3f{sum.x * inv, sum.y * inv, sum.z * inv};
    }
};
```

完整实现会涉及 temporary buffers 和 zip iterator。
教学重点是算法形态：

```text
key -> sort -> reduce_by_key -> centroid
```

### 工程边界：排序成本不低

排序是昂贵操作。
如果每帧点数不大，CPU 哈希可能更快。
如果点数很大，或者后续 GPU 阶段继续使用排序结果，GPU 方案才更有价值。
再次强调：端到端测量决定选择。

> 🧠 **思维陷阱：认为 GPU 排序比 CPU 排序"总是更快"**
> **新手想法**："GPU 有海量线程，排序肯定比 CPU 快。"
> **实际上**：GPU 排序（如 `thrust::sort`）的优势在大数据量时才体现。对于 1 万以下的元素，CPU 的 `std::sort`（基于缓存友好的 introsort）通常更快，因为 GPU 排序有 kernel launch 开销和数据传输成本。此外，GPU 排序需要临时显存，这也要纳入资源预算。
> **正确思维**：排序的 CPU/GPU 盈亏平衡点通常在 1-10 万元素之间，具体取决于元素大小和硬件。

> ⚠️ **编程陷阱：`sort_by_key` 后忘记关联数组已被重排**
> **错误做法**：`thrust::sort_by_key(keys, keys_end, points)` 后，仍然使用旧的 points 索引对应关系。
> **现象**：后续按原始索引访问 `timestamps[i]` 或 `labels[i]` 时，数据对不上。
> **根本原因**：`sort_by_key` 会同步重排 values（points），但不会重排其他没有作为 values 传入的数组。
> **正确做法**：要么用 zip iterator 把所有关联数组一起排序，要么先生成索引数组 `[0,1,2,...]`，以 key 排序索引，再用索引做 gather。

### 练习

1. **[代码题]** 用 Thrust 实现 GPU 体素下采样的完整流程：计算 key -> sort_by_key -> reduce_by_key 得到每个体素的坐标和与点数 -> 计算质心。用 CPU 版本验证结果一致性。
2. **[性能题]** 对 100K 和 10M 个点分别测量 `thrust::sort` 和 CPU `std::sort` 的耗时。在什么规模下 GPU 排序开始胜出？

---

## 37.10 CMake CUDA 集成 ⭐

### 工程问题：CUDA 是一种编译语言，不只是链接库

如果项目包含 `.cu` 文件，需要让 CMake 启用 CUDA 语言：

```cmake
cmake_minimum_required(VERSION 3.24)
project(mini_slam_cuda LANGUAGES CXX CUDA)

add_library(cuda_kernels
    src/point_transform.cu
)

target_compile_features(cuda_kernels
    PUBLIC
        cxx_std_17
)

set_target_properties(cuda_kernels
    PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
)
```

CUDA 架构应根据目标 GPU 设置。
不要把某个架构列表当成永远正确的固定答案。
实际项目应根据部署硬件和 CUDA 工具链说明配置。

### 反面失败：开发机能跑，部署机不能跑

如果只为开发机 GPU 编译，部署到另一张 GPU 可能性能差或无法运行。
如果只生成 PTX，首次运行可能触发 JIT 编译，带来启动延迟。
如果架构列表过多，编译时间和二进制体积增加。

### 工程边界：CUDA 版本、驱动和平台要成组管理

CUDA 项目部署要同时考虑：

1. 驱动版本。
2. CUDA Toolkit。
3. GPU 架构。
4. 第三方库编译版本。
5. 容器镜像或系统环境。
6. Jetson 平台的 JetPack 绑定关系。

版本细节变化快。
更稳妥的写法是给出检查方法和边界，而不是把某个版本号写成永久事实。

---

## 37.11 错误处理与同步点 ⭐⭐

### 工程问题：GPU 调用常常是异步的

很多 CUDA 操作对 CPU 是异步的。
如果不显式同步，计时和错误检查可能误导。

错误模式：

```cpp
auto t0 = Clock::now();
launchKernel();
auto t1 = Clock::now();
```

这可能只测到了 launch 成本，没有测到 kernel 完成时间。

### 代码验证：同步后计时

```cpp
#include <chrono>
#include <cuda_runtime.h>
#include <stdexcept>

void checkCuda(cudaError_t error) {
    if (error != cudaSuccess) {
        throw std::runtime_error(cudaGetErrorString(error));
    }
}

template <class Func>
double timeGpuMs(Func func) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    func();
    checkCuda(cudaDeviceSynchronize());
    const auto t1 = Clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}
```

正式测量可使用 CUDA events。
CPU 计时加同步适合教学和端到端观察。

### 工程边界：同步会影响 pipeline

过度同步会破坏 GPU/CPU 重叠。
但缺少同步会让结果还没完成就被读取。
工程上要明确同步点：

1. 何时必须等待 GPU 完成？
2. 何时可以让 CPU 做其他工作？
3. 哪些数据还在 GPU 上？
4. 哪些数据已经可供 CPU 使用？

---

## 37.12 Streams、异步拷贝与 pinned memory ⭐⭐⭐

### 工程问题：GPU 加速不应让 CPU 原地等待

如果每帧都按下面顺序执行：

```text
CPU 上传点云
等待上传完成
GPU 计算
等待计算完成
CPU 下载结果
等待下载完成
CPU 处理下一帧
```

CPU 和 GPU 大量时间都在互相等待。
更理想的 pipeline 是：

```text
CPU 准备下一帧
GPU 处理当前帧
DMA 传输上一阶段数据
```

CUDA stream 用来表达一条设备侧任务序列。
同一 stream 内操作按顺序执行。
不同 stream 之间可能重叠，具体取决于硬件、驱动和资源。

### 反面失败：默认 stream 隐式同步

很多示例代码使用默认 stream。
这对入门友好，但容易隐藏同步边界。
如果所有操作都落在默认 stream，原本可以重叠的上传、计算和下载可能被串行化。

教学阶段可以先用默认 stream 保证正确。
进入性能优化时，必须把 stream 和同步点显式画出来。

### 抽象不变量：异步需要三个条件

要让拷贝和计算真正重叠，通常需要：

1. 使用非默认或明确管理的 stream。
2. 使用异步拷贝 API。
3. host 侧内存适合异步 DMA，例如 pinned memory。

pinned memory 是页锁定内存。
它不能被操作系统随意换出，因此更适合 GPU DMA。
代价是它会减少系统可分页内存，分配和释放成本也更高。
不要为所有普通数据都使用 pinned memory。

### 代码验证：stream 生命周期的 RAII 封装

```cpp
#include <cuda_runtime.h>
#include <stdexcept>

class CudaStream {
public:
    CudaStream() {
        if (cudaStreamCreate(&stream_) != cudaSuccess) {
            throw std::runtime_error("cudaStreamCreate failed");
        }
    }

    ~CudaStream() {
        if (stream_) {
            cudaStreamDestroy(stream_);
        }
    }

    CudaStream(const CudaStream&) = delete;
    CudaStream& operator=(const CudaStream&) = delete;

    cudaStream_t get() const {
        return stream_;
    }

    void synchronize() const {
        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            throw std::runtime_error("cudaStreamSynchronize failed");
        }
    }

private:
    cudaStream_t stream_{};
};
```

析构函数中不抛异常更稳妥。
这里为了教学简化，只在显式 `synchronize()` 中报告错误。
真实项目可以把错误检查封装成统一工具。

### 工程边界：Thrust 与 stream

Thrust 支持执行策略和 stream 绑定。
具体 API 形式随 CUDA/Thrust 版本演进，以当前官方文档为准。
工程上要记住的是：

```text
如果你希望多个 Thrust 操作进入指定 stream，
就不能只依赖默认调用形式。
```

同时，Thrust 算法之间可能分配临时存储。
如果这是性能热点，要结合 Thrust/CUB 文档或自定义 allocator 继续优化。

---

## 37.13 GPU 数据布局：AoS、SoA 与 zip iterator ⭐⭐⭐

### 工程问题：GPU 线程束喜欢相邻线程访问相邻地址

GPU 上多个相邻线程通常一起执行。
如果相邻线程访问连续地址，内存访问更容易合并。
这叫 coalesced access。

AoS 点云：

```text
x0 y0 z0 | x1 y1 z1 | x2 y2 z2 | ...
```

SoA 点云：

```text
x0 x1 x2 ...
y0 y1 y2 ...
z0 z1 z2 ...
```

如果 kernel 只处理 x，SoA 更理想。
如果每个线程同时读取 x/y/z，AoS 也可能可接受。
是否更快取决于访问模式、对齐、编译器和后续算法。

### 反面失败：CPU 为 PCL 保持 AoS，GPU 每步都拆成 SoA

如果每个 GPU 阶段都做：

```text
AoS -> SoA -> 计算 -> AoS
```

转换成本可能很高。
更好的做法是：

```text
进入 GPU pipeline 时转换一次
多个 GPU 阶段保持 SoA
离开 GPU pipeline 时必要时再转换
```

### 抽象不变量：布局转换要跨阶段摊销

布局转换的价值来自后续多个阶段复用：

| 转换后阶段数 | 是否值得 |
|--------------|----------|
| 1 个很小 kernel | 通常不值得 |
| 多个点云内核 | 可能值得 |
| 全 GPU pipeline | 更可能值得 |
| 输出仍留 GPU | 收益更高 |

这和 缓存优化与数据布局 的结论一致：布局优化必须端到端测。

### 代码验证：zip iterator 表达 SoA 点

Thrust 可以用 zip iterator 把多个数组组合成一个逻辑元素：

```cpp
#include <thrust/device_vector.h>
#include <thrust/for_each.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

struct ScaleSoA {
    float scale = 1.0f;

    template <class Tuple>
    __host__ __device__
    void operator()(Tuple t) const {
        thrust::get<0>(t) *= scale;
        thrust::get<1>(t) *= scale;
        thrust::get<2>(t) *= scale;
    }
};

void scaleSoA(thrust::device_vector<float>& x,
              thrust::device_vector<float>& y,
              thrust::device_vector<float>& z,
              float scale) {
    auto begin = thrust::make_zip_iterator(
        thrust::make_tuple(x.begin(), y.begin(), z.begin()));
    auto end = thrust::make_zip_iterator(
        thrust::make_tuple(x.end(), y.end(), z.end()));

    thrust::for_each(begin, end, ScaleSoA{scale});
}
```

这个例子展示 zip iterator 的思想。
实际项目还要检查 in-place 修改、执行策略和编译器支持。

### 工程边界：zip iterator 提升表达力，也会增加模板复杂度

Thrust 的 iterator 组合很强大。
但错误信息会变长，编译时间可能增加。
教学和项目中应先保证算法结构清楚。
如果 zip iterator 让代码难以维护，可以写小型结构体或手写 kernel。

---

## 37.14 Thrust、CUB 与手写 kernel 的边界 ⭐⭐⭐

### 工程问题：高层算法不总是最终答案

Thrust 适合快速表达标准模式。
CUB 提供更底层的 block/device primitive。
手写 kernel 提供最大控制。

| 工具 | 优势 | 代价 |
|------|------|------|
| Thrust | 开发快、类似 STL、组合标准算法 | 中间临时对象和调度不总透明 |
| CUB | 高性能 primitive、可控临时存储 | API 更底层 |
| 手写 kernel | 最大控制、可融合多个操作 | 开发和调试成本高 |

### 反面失败：过早手写 kernel

如果一个体素下采样还没确定算法流程，就直接写复杂 kernel：

1. 正确性难验证。
2. 边界条件难覆盖。
3. 性能瓶颈未必在 kernel。
4. 后续改算法代价大。

更稳妥顺序：

```text
CPU 参考
Thrust 原型
分阶段 profile
必要时 CUB 或 kernel 优化热点
```

### 抽象不变量：先组合，再融合

Thrust 组合可能产生多个 pass：

```text
transform -> remove_if -> sort -> reduce_by_key
```

如果 profiling 显示内存流量过大，可以考虑融合：

```text
一个 kernel 同时做 transform + predicate
```

但融合会降低可读性。
只有热点明确时才值得。

### 工程边界：临时存储也要纳入内存策略

排序、scan、reduce 可能需要临时存储。
高层 API 会帮你管理，但不代表没有分配。
如果临时分配成为瓶颈，要研究具体库的 temporary storage 机制。
这与 内存分配策略与pmr 的内存策略是同一个问题在 GPU 侧的表现。

---

## 37.15 Warp divergence 与 occupancy 直觉 ⭐⭐

> **这一节解决什么问题**：warp divergence 不是背概念——从"if-else 在 GPU 上会发生什么？"这个具体场景推导出为什么分支会让 GPU 效率暴跌。

### 工程问题：if-else 在 GPU 上到底会发生什么？

在 CPU 上，`if-else` 是最基本的控制流——分支预测器猜对了就零开销，猜错了也只损失 10-20 个时钟周期。但在 GPU 上，`if-else` 的代价可能是**执行时间翻倍**。要理解为什么，必须从 SIMT 的物理执行机制推导。

**回顾 37.2 节的关键事实**：一个 warp 包含 32 个线程，它们共享同一个程序计数器（PC）。这意味着在同一时刻，一个 warp 中的所有线程必须执行**同一条指令**。这是 GPU 高吞吐的基础——一个 warp 调度器发出一条指令，32 个线程同时执行它，相当于一条指令的调度成本分摊给了 32 个计算单元。

**那么遇到 if-else 时怎么办？** 假设一个 kernel 包含以下分支：

```cpp
if (point_is_ground) {
    processGroundPoint();   // 路径 A：20 条指令
} else {
    processObstaclePoint(); // 路径 B：30 条指令
}
```

考虑一个 warp 中 32 个线程处理 32 个点。假设其中 16 个点是地面点（走路径 A），16 个是障碍点（走路径 B）。由于所有线程必须执行同一条指令，GPU **不能**让 16 个线程走 A、另外 16 个同时走 B。它必须这样做：

1. **Phase 1**：执行路径 A 的 20 条指令。16 个地面点线程正常执行，16 个障碍点线程被**掩码禁用**（masked off）——它们的执行单元通电但不写入结果。
2. **Phase 2**：执行路径 B 的 30 条指令。16 个障碍点线程正常执行，16 个地面点线程被掩码禁用。
3. **合并**：所有线程回到 if-else 之后的共同路径。

总共执行了 20 + 30 = 50 条指令，但每个线程只有效执行了 20 或 30 条。如果没有分支，32 个线程都做相同的 20 条指令只需 20 个周期。有分支后变成 50 个周期——**效率降低到 40-60%**。

**更坏的情况**：如果 32 个线程中有 31 个走 A、只有 1 个走 B，仍然需要执行两条路径——那 1 个"异类"线程拖慢了整个 warp。极端情况下，一个 warp 中每个线程走不同的分支（如 `switch` 有 32 个 case），效率降到 $1/32 \approx 3\%$。

**最好的情况**：如果一个 warp 中所有 32 个线程走同一条路径（全是地面点或全是障碍点），就不需要两次执行——效率 100%。这就是为什么 GPU 编程强调"让相邻线程做相同的事"：相邻线程通常在同一个 warp 中，如果它们的分支决策一致，就不会发生 divergence。

> **本质洞察**：Warp divergence 的本质不是"分支本身慢"，而是**SIMT 模型中分支导致执行资源空闲**。32 个线程中只要有 1 个走不同路径，其他 31 个就要等它。这和 CPU 的分支预测失败完全不同——CPU 上分支预测失败只影响一个线程的流水线，GPU 上分支不一致影响同一 warp 的 32 个线程。

### 反面失败：在一个 kernel 中混合大量异构逻辑

错误倾向：

```text
一个 kernel 同时处理：
    NaN 过滤
    地面点分类
    障碍点聚类
    动态物体判断
    特殊语义标签处理
```

这会让每个线程走不同路径。
GPU 的并行度被复杂分支消耗。

更适合 GPU 的方式是把流程拆成规则阶段：

```text
阶段 1：计算简单 predicate
阶段 2：compact
阶段 3：对同类数据执行规则计算
```

### 抽象不变量：让同一批线程做相似工作

GPU 热点应尽量满足：

1. 相邻线程执行相同指令。
2. 相邻线程访问相邻地址。
3. 每个线程工作量接近。
4. 分支少且分布均匀。
5. 中间结果尽量留在 device。

这和 CPU 多线程的负载均衡相似，但粒度更细。

### occupancy 的边界

occupancy 通常描述一个 SM 上活跃 warp 的比例。
更高 occupancy 可以帮助隐藏内存延迟。
但 occupancy 不是越高越好。

如果一个 kernel 已经受内存带宽限制，继续追求 occupancy 不一定提升。
如果为了提高 occupancy 减少寄存器使用，却导致更多 global memory 访问，也可能变慢。

教学阶段记住：

```text
occupancy 是诊断指标，不是唯一目标。
```

### 代码验证：先分离 predicate

```cpp
#include <thrust/device_vector.h>
#include <thrust/transform.h>

struct IsNearGround {
    float threshold = 0.2f;

    __host__ __device__
    int operator()(const Point3f& p) const {
        return p.z < threshold ? 1 : 0;
    }
};

thrust::device_vector<int>
classifyGroundMask(const thrust::device_vector<Point3f>& points,
                   float threshold) {
    thrust::device_vector<int> mask(points.size());
    thrust::transform(points.begin(),
                      points.end(),
                      mask.begin(),
                      IsNearGround{threshold});
    return mask;
}
```

后续可以根据 mask 做 compact，把地面点和非地面点分开处理。
这比在一个复杂 kernel 里做所有逻辑更容易分析。

---

## 37.16 设备内存生命周期与 GPU 侧 RAII ⭐⭐⭐

### 工程问题：device_vector 也有生命周期和分配成本

`thrust::device_vector<T>` 管理 GPU 内存。
构造、resize、赋值都可能触发 device memory 分配或拷贝。
如果每帧创建大量 device_vector，GPU 侧也会出现分配开销和长尾。

这和 CPU 侧 `std::vector` 是同一个问题。
区别是内存位于 device，错误更难调试。

### 反面失败：每帧重复创建所有 GPU 缓冲

```cpp
GpuResult processFrameGpu(const std::vector<Point3f>& input) {
    thrust::device_vector<Point3f> d_points(input.begin(), input.end());
    thrust::device_vector<Point3f> d_filtered;
    thrust::device_vector<std::uint64_t> d_keys;
    thrust::device_vector<Point3f> d_output;
    // ...
}
```

这段代码适合原型。
但如果每帧都分配多个 device_vector，长期运行可能不稳定。

### 抽象不变量：GPU 工作区也应复用

可以把 GPU 临时缓冲放进工作区对象：

```cpp
class GpuWorkspace {
public:
    void reserve(std::size_t max_points) {
        points_.reserve(max_points);
        filtered_.reserve(max_points);
        keys_.reserve(max_points);
        output_.reserve(max_points);
    }

    thrust::device_vector<Point3f>& points() {
        return points_;
    }

    thrust::device_vector<Point3f>& filtered() {
        return filtered_;
    }

    thrust::device_vector<std::uint64_t>& keys() {
        return keys_;
    }

    thrust::device_vector<Point3f>& output() {
        return output_;
    }

private:
    thrust::device_vector<Point3f> points_;
    thrust::device_vector<Point3f> filtered_;
    thrust::device_vector<std::uint64_t> keys_;
    thrust::device_vector<Point3f> output_;
};
```

每帧只调整 size，尽量不重新分配 capacity。
这和 内存分配策略与pmr 的容器复用完全一致。

### 工程边界：reserve 不等于没有临时分配

你自己的 device_vector 可以 reserve。
但 Thrust 算法内部仍可能需要临时 storage。
例如 sort、scan、reduce_by_key。
如果这些临时分配成为瓶颈，需要进一步使用库支持的 allocator 或下沉到 CUB。
这属于进阶优化，必须由 profiling 驱动。

### 代码验证：端到端工作区接口

```cpp
#include <stdexcept>

class ThrustPointPipeline {
public:
    explicit ThrustPointPipeline(std::size_t max_points)
        : max_points_(max_points) {
        workspace_.reserve(max_points_);
    }

    std::vector<Point3f> run(const std::vector<Point3f>& input,
                             const Transform3f& T) {
        if (input.size() > max_points_) {
            throw std::runtime_error("input cloud exceeds GPU workspace capacity");
        }

        auto& d_points = workspace_.points();
        auto& d_transformed = workspace_.output();
        d_points.assign(input.begin(), input.end());
        d_transformed.resize(d_points.size());

        transformGpu(d_points, T, d_transformed);
        removeInvalidGpu(d_transformed, 100.0f);

        std::vector<Point3f> output(d_transformed.size());
        thrust::copy(d_transformed.begin(), d_transformed.end(), output.begin());
        return output;
    }

private:
    std::size_t max_points_ = 0;
    GpuWorkspace workspace_;
};
```

这里让 `transformGpu()` 接收输出 buffer，而不是返回新的 `device_vector`。
这个接口形式看起来比“返回一个结果容器”笨一些，却更符合实时系统的需要：工作区拥有临时显存，算法阶段只改变其中的有效 size。
这样每一帧都复用同一批 device buffer，避免把显存分配隐藏在函数返回值里。

`max_points_` 是这个接口的实时边界。
如果输入超过预留容量还默默扩容，程序仍然能跑，但“热路径不分配”的承诺就被破坏了。
真实系统可以选择抛错、降采样、裁剪或回退 CPU；无论采用哪一种，都应显式写在接口语义里。

这个设计和 CPU 侧 `std::pmr` 的思想一致。
区别只是资源位置从 host 变成 device，本质问题仍然是：热路径里的临时对象必须有明确所有者，分配次数必须可预测。

---

## 37.17 CUDA events 与端到端 benchmark ⭐⭐

### 工程问题：CPU 计时和 GPU 计时回答不同问题

CPU 侧计时回答：

```text
主线程等待了多久？
端到端延迟是多少？
```

CUDA event 回答：

```text
某个 stream 上两个事件之间的 GPU 时间是多少？
```

两者都重要。
SLAM 系统最终关心端到端。
GPU 优化时也需要知道具体 kernel 或算法阶段耗时。

### 代码验证：CUDA event RAII

```cpp
#include <cuda_runtime.h>
#include <stdexcept>

class CudaEvent {
public:
    CudaEvent() {
        if (cudaEventCreate(&event_) != cudaSuccess) {
            throw std::runtime_error("cudaEventCreate failed");
        }
    }

    ~CudaEvent() {
        if (event_) {
            cudaEventDestroy(event_);
        }
    }

    CudaEvent(const CudaEvent&) = delete;
    CudaEvent& operator=(const CudaEvent&) = delete;

    cudaEvent_t get() const {
        return event_;
    }

private:
    cudaEvent_t event_{};
};

float elapsedMs(const CudaEvent& begin, const CudaEvent& end) {
    float ms = 0.0f;
    if (cudaEventElapsedTime(&ms, begin.get(), end.get()) != cudaSuccess) {
        throw std::runtime_error("cudaEventElapsedTime failed");
    }
    return ms;
}
```

使用时要在正确 stream 上 record，并在读取 elapsed 前确保事件完成。

### benchmark 表格

报告 GPU 加速时，至少写清楚：

| 项目 | 是否计入 |
|------|----------|
| host 数据准备 | 是/否 |
| H2D 上传 | 是/否 |
| kernel 或 Thrust 算法 | 是 |
| D2H 下载 | 是/否 |
| 同步等待 | 是/否 |
| 首次初始化 | 通常单独报告 |
| 数据布局转换 | 必须说明 |

如果论文或报告只写“kernel 0.2ms”，不能直接推断系统端到端延迟也是 0.2ms。

---

## 37.18 Mini SLAM Thrust Kernels

### 工程问题：把标准 GPU 模式组合成小型点云管线

本章项目实现：

```text
CPU raw cloud
  -> upload
  -> remove invalid
  -> transform
  -> compute voxel key
  -> sort by key
  -> reduce by key
  -> download centroids
```

目标不是写最底层 kernel。
目标是用 Thrust 组合标准并行模式。

### 模块结构

```text
mini_slam_cuda/
  include/
    cuda_point_types.cuh
    thrust_transform.cuh
    thrust_filter.cuh
    thrust_voxel.cuh
    cuda_timer.hpp
  src/
    thrust_pipeline.cu
  tests/
    test_transform_correctness.cpp
    test_filter_correctness.cpp
    test_voxel_downsample.cpp
    test_cpu_gpu_tolerance.cpp
```

### 正确性测试

```cpp
bool near(float a, float b, float eps) {
    return std::abs(a - b) <= eps;
}

void testTransformGpuMatchesCpu() {
    const std::vector<Point3f> input = makeTestCloud();
    const Transform3f T = makeTestTransform();

    const auto cpu = transformCpu(input, T);
    const auto gpu = transformGpuEndToEnd(input, T);

    assert(cpu.size() == gpu.size());
    for (std::size_t i = 0; i < cpu.size(); ++i) {
        assert(near(cpu[i].x, gpu[i].x, 1e-5f));
        assert(near(cpu[i].y, gpu[i].y, 1e-5f));
        assert(near(cpu[i].z, gpu[i].z, 1e-5f));
    }
}
```

GPU 和 CPU 浮点结果不一定逐 bit 一致。
测试应使用合理误差。

### benchmark 输出

至少分开记录：

| 阶段 | 说明 |
|------|------|
| upload | CPU 到 GPU |
| filter | 过滤 |
| transform | 坐标变换 |
| voxel key | key 计算 |
| sort | 排序 |
| reduce | 归约 |
| download | GPU 到 CPU |
| total | 端到端 |

只报告 kernel 时间会误导。
SLAM 系统关心端到端。

---

## 🔧 故障排查手册

| 现象 | 常见原因 | 检查方法 | 修复方向 |
|------|----------|----------|----------|
| GPU 版本更慢 | 数据太小或传输太多 | 分阶段计时 | 扩大批量或保留 GPU 中间结果 |
| 结果未更新 | 缺少同步 | 加 `cudaDeviceSynchronize` 验证 | 明确同步点 |
| 编译失败 | functor 使用 host-only API | 看 nvcc 报错 | 简化 device functor |
| 运行时报非法访问 | device 指针生命周期错误 | cuda-memcheck 类工具 | 检查 vector/指针范围 |
| 体素结果错 | 负坐标 key 编码错误 | 构造边界测试 | 定义坐标偏移 |
| CPU/GPU 结果微差 | 浮点顺序不同 | 误差比较 | 调整容忍或归约策略 |
| 首次运行很慢 | JIT 或初始化 | 丢弃首轮计时 | 预热 |
| CMake 找不到 CUDA | 未启用 CUDA 语言或环境不匹配 | 查看 CMake 日志 | 配置工具链 |
| Jetson 上性能不稳 | 功耗模式/温度影响 | 记录频率温度 | 固定功耗模式并散热 |
| sort 占主要时间 | 排序成本高 | 分阶段 profile | 改算法或减少排序规模 |

### GPU 任务选择自检清单

选择 GPU 前先回答：

1. 数据规模是否足够大？
2. 计算是否规则、分支是否少？
3. 输入是否已经在 GPU 上？
4. 输出是否必须回 CPU？
5. 是否能把多个阶段串在 GPU 上？
6. CPU 版本是否已经用多线程和布局优化过？
7. 目标平台是独立 GPU 还是共享内存 SoC？
8. 是否需要 CPU fallback？
9. benchmark 是否包含上传和下载？
10. 测试是否允许合理浮点误差？

如果这些问题没有答案，不要急着写 kernel。

---

## 37.20 练习与跨章综合题

1. 用 Thrust 实现一个点云强度缩放函数，要求输入输出分离，并写 CPU 版本对照测试。
2. 为 GPU 计时写两个 benchmark：一个只测 kernel，一个测 upload+kernel+download，解释两者差异。
3. 构造一个 CPU/GPU 浮点结果不逐位一致的归约例子，用相对误差和绝对误差判断是否可接受。
4. 把一个 AoS 点云拆成 SoA device buffer，说明哪些访问会更容易 coalescing，哪些接口会因此变复杂。
5. 跨章综合题：结合 缓存优化与数据布局 的数据布局和 CUDA在SLAM中的应用 的 fallback 思路，为“GPU 体素下采样”设计接口。要求支持 CPU-only 编译、CUDA 可用性检查、端到端计时和结果误差测试。

这些练习的重点是把 CUDA 当作系统能力，而不是孤立语法。

---

## 37.21 本章小结

CUDA 的核心不是语法，而是数据并行心智模型。
GPU 适合大规模、规则、可批处理的数据任务。
Thrust 让许多标准模式可以像 STL 一样表达：transform、filter、sort、scan、reduce。

本章关键判断：

1. 端到端计时必须包含上传、计算、下载和同步。
2. 上传后尽量让多个阶段留在 GPU 上。
3. Thrust 适合先表达标准并行模式。
4. 手写 kernel 应在 profiling 后再引入。
5. GPU 结果与 CPU 结果应做误差比较。
6. CMake、CUDA 架构、驱动和部署环境要成组管理。
7. 数据布局仍然重要，GPU 加速不能弥补混乱的数据流。

下一章会从基础 API 转向实际 SLAM 系统：怎样使用已有 CUDA 加速库、怎样做 CPU/GPU fallback、怎样避免把 GPU 加速改造成不可维护的工程负担。

---

## 延伸阅读

1. **NVIDIA CUDA Programming Guide（CUDA 12.x）** ⭐⭐——权威参考，重点阅读 Thread Hierarchy、Memory Hierarchy、Execution Configuration 和 CUDA Graphs 章节。CUDA 12.x 引入了 CUDA Graphs 的动态并行和 cooperative groups 增强。
2. **NVIDIA Thrust / CCCL 官方文档** ⭐⭐——Thrust 在 CUDA 12.x 中已迁入 CCCL（CUDA C++ Core Libraries）统一仓库，算法接口基本兼容但编译模型有调整。
3. **CUB 文档** ⭐⭐⭐——当 Thrust 组合无法满足性能需求时，CUB 提供更底层的并行 primitive（block-level reduce、scan、radix sort），可以精确控制 shared memory 和 warp 行为。
4. **CMake CUDA language 文档** ⭐——`enable_language(CUDA)`、`CMAKE_CUDA_ARCHITECTURES`、`set_target_properties` 等 CUDA 特定设置。
5. **NVIDIA Nsight Systems / Nsight Compute** ⭐⭐⭐——GPU profiling 工具，用于分析 kernel 占用率、内存带宽利用率和 warp 调度效率。
6. **Sam Williams et al., "Roofline: An Insightful Visual Performance Model", CACM 2009** ⭐⭐⭐——Roofline 模型的原始论文，理解算术强度如何决定 GPU 优化方向。
5. GPU 性能分析工具文档：Nsight Systems、Nsight Compute、CUDA events。

---

## 37.22 Unified Memory 与零拷贝传输 ⭐⭐⭐

### 工程问题：CPU-GPU 数据传输是最常见的性能瓶颈

前面讨论的端到端计时揭示了一个反复出现的问题：很多 CUDA 加速项目的瓶颈不是 kernel 计算，而是 CPU-GPU 之间的数据传输。对于十万量级的点云，上传和下载的 PCIe 传输时间可能超过 kernel 执行时间。

CUDA 提供了两种减少传输开销的机制：Unified Memory（统一内存）和 Zero-Copy（零拷贝）内存。

**Unified Memory**（`cudaMallocManaged`）让 CPU 和 GPU 通过同一个指针访问同一块内存。驱动在后台按需迁移页面——当 GPU 访问某个页面时，驱动自动将它从 CPU 内存搬到 GPU 内存。

```cpp
// Unified Memory 方式
float* points;
cudaMallocManaged(&points, n * sizeof(float));

// CPU 填充数据
for (int i = 0; i < n; ++i) points[i] = loadFromSensor(i);

// GPU 直接使用同一指针，驱动自动迁移
transformKernel<<<blocks, threads>>>(points, n);
cudaDeviceSynchronize();

// CPU 直接读取结果，驱动自动回迁
float result = points[0];

cudaFree(points);
```

**Zero-Copy**（`cudaHostAllocMapped`）在 CPU 端分配固定内存（pinned memory），GPU 通过 PCIe 直接访问 CPU 内存，完全不做数据拷贝。

```cpp
// Zero-Copy 方式
float* h_points;
cudaHostAlloc(&h_points, n * sizeof(float), cudaHostAllocMapped);
float* d_points;
cudaHostGetDevicePointer(&d_points, h_points, 0);

// CPU 填充
for (int i = 0; i < n; ++i) h_points[i] = loadFromSensor(i);

// GPU 通过 PCIe 远程访问 CPU 内存
transformKernel<<<blocks, threads>>>(d_points, n);
```

### 选型决策

| 方式 | 适用场景 | 不适用场景 |
|------|---------|-----------|
| 显式 `cudaMemcpy` | kernel 需要多次遍历数据；独立 GPU | 数据量小或一次性访问 |
| Unified Memory | 原型开发；访问模式不规则 | 对延迟敏感的实时路径（迁移时机不可控） |
| Zero-Copy | 数据只读一次且量不大；Jetson 等共享内存 SoC | 独立 GPU 上大量数据的反复访问（PCIe 带宽成瓶颈） |

在 Jetson 平台上，CPU 和 GPU 共享物理内存，Zero-Copy 几乎没有额外开销——这使得 Jetson 上的 CUDA 代码可以比桌面 GPU 更简单。在独立 GPU 上，Zero-Copy 的每次 GPU 访问都要穿越 PCIe 总线，对数据密集型 kernel 不可接受。

> **反事实推理**：如果所有 GPU 都像 Jetson 一样共享内存，CUDA 编程会简单得多。
> 我们不需要显式管理上传/下载，不需要双缓冲来隐藏传输延迟，不需要在 CPU 和 GPU 之间同步数据所有权。
> 但独立 GPU 通过专用显存和高带宽内存（HBM）获得了远超共享内存的带宽——带宽换来的代价是编程复杂度。

---

## 37.23 CUDA 与 C++ 标准并行算法的对比 ⭐⭐

C++17 引入了标准并行算法（如 `std::transform` 的 parallel 执行策略），C++20/23 进一步扩展了 `std::execution` 框架。一个自然的问题是：什么时候用 C++ 标准并行，什么时候用 CUDA/Thrust？

| 维度 | C++ 标准并行 | CUDA/Thrust |
|------|-------------|-------------|
| 硬件 | CPU 多核 | NVIDIA GPU |
| 部署 | 任何 C++17 编译器 | 需要 NVIDIA GPU + CUDA 工具链 |
| 数据量阈值 | 万级开始有收益 | 十万级以上才值得传输开销 |
| 编程模型 | STL 接口，无需修改数据布局 | 需要管理 device 内存和数据传输 |
| 加速比 | 2-16x（核数决定） | 10-1000x（取决于计算/传输比） |
| 实时可预测性 | 线程调度可控 | GPU 调度由驱动控制 |

对于 SLAM 的前端处理（体素滤波、法线估计），数据量通常在十万到百万级，GPU 加速显著。对于后端优化（位姿图、BA），虽然计算密集，但稀疏矩阵操作的并行度不高，GPU 加速的收益取决于问题规模。

C++26 提案 P2300（`std::execution`，即 Senders/Receivers）试图统一 CPU 和 GPU 的异步执行模型，NVIDIA 的 stdexec 是其参考实现。长期来看，C++ 标准可能提供一个统一的异步执行框架，减少 CPU/GPU 编程的割裂。但在 2026 年的工程实践中，CUDA + Thrust 仍然是 GPU 计算的主流选择。

### 编写 CPU/GPU 可切换的算法接口

在实际项目中，应当设计能在 CPU 和 GPU 之间切换的算法接口，而非让 CUDA 代码散落在各处。

```cpp
// 接口层——调用者不关心后端
class VoxelDownsampler {
public:
    virtual ~VoxelDownsampler() = default;
    virtual std::vector<Eigen::Vector3f> downsample(
        const std::vector<Eigen::Vector3f>& input,
        float voxel_size) = 0;
};

// CPU 实现
class CpuVoxelDownsampler : public VoxelDownsampler {
    std::vector<Eigen::Vector3f> downsample(
        const std::vector<Eigen::Vector3f>& input,
        float voxel_size) override;
};

// GPU 实现
class CudaVoxelDownsampler : public VoxelDownsampler {
    std::vector<Eigen::Vector3f> downsample(
        const std::vector<Eigen::Vector3f>& input,
        float voxel_size) override;
    // 内部管理 device memory 的生命周期
};

// 工厂——运行时选择后端
std::unique_ptr<VoxelDownsampler> makeDownsampler() {
    if (cudaAvailable()) return std::make_unique<CudaVoxelDownsampler>();
    return std::make_unique<CpuVoxelDownsampler>();
}
```

这种设计的好处是：(1) 在没有 NVIDIA GPU 的机器上自动 fallback 到 CPU；(2) 测试可以只验证 CPU 路径而不需要 GPU 环境；(3) 新增后端（如 OpenCL、SYCL）不影响调用端。本章介绍的继承与多态（来自 继承与多态深入）在这里得到了直接应用。

---

## 37.24 CUDA Graphs 与计算图优化 ⭐⭐⭐

CUDA 12.x 引入的 CUDA Graphs 机制允许将一系列 kernel 启动和内存操作预先录制为一个"计算图"，然后一次性提交执行。这减少了每次 kernel 启动的 CPU 端开销（launch overhead）。

在 SLAM 的点云处理管线中，同一序列的操作每帧重复执行：下采样 → 法线估计 → 特征提取 → 配准。如果每个阶段都单独启动 kernel，CPU 端的启动开销在高频场景（30+ Hz）中可能显著。用 CUDA Graph 录制整条管线后，只需一次启动调用就能触发整条管线。

```cpp
// 录制阶段（只执行一次）
cudaGraph_t graph;
cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal);

// 录制管线中的所有 kernel
downsampleKernel<<<...>>>(d_input, d_downsampled, ...);
normalEstimationKernel<<<...>>>(d_downsampled, d_normals, ...);
featureExtractionKernel<<<...>>>(d_normals, d_features, ...);

cudaStreamEndCapture(stream, &graph);

// 实例化
cudaGraphExec_t graphExec;
cudaGraphInstantiate(&graphExec, graph, nullptr, nullptr, 0);

// 执行阶段（每帧调用）
cudaGraphLaunch(graphExec, stream);  // 一次调用执行整条管线
```

CUDA Graphs 的工程注意事项：录制的图假设 kernel 参数（如指针地址）不变。如果每帧的输入缓冲区地址不同，需要使用 `cudaGraphExecKernelNodeSetParams` 更新参数，或使用固定的 device buffer 并在执行前拷贝数据进去。对于 SLAM 管线，输入缓冲区通常可以预分配为固定大小（按最大点云容量），每帧只更新数据内容和有效点数。

### 故障排查手册（补充条目）

| 症状 | 可能原因 | 排查步骤 | 处理 |
|------|----------|----------|------|
| Unified Memory 方式比显式 `cudaMemcpy` 更慢 | 页面迁移在 kernel 执行期间发生，导致 GPU 停顿 | 用 `nvprof` 或 `nsight-sys` 检查 page fault 事件 | 使用 `cudaMemPrefetchAsync` 在 kernel 前预迁移页面 |
| CUDA Graph 录制后修改参数导致崩溃 | 参数地址在录制和执行之间发生变化 | 检查 device buffer 是否在录制后 `cudaFree`/重新分配 | 使用固定地址的预分配 buffer |
| CPU fallback 路径结果与 GPU 路径不一致 | 浮点运算顺序不同导致舍入差异 | 用相对误差（而非逐位比较）验证一致性 | 设置合理的误差阈值（如 $10^{-5}$），测试覆盖边界情况 |
| Thrust `sort_by_key` 在 host 和 device 上行为不同 | 相同键值的元素排序不稳定 | 检查是否依赖了排序的稳定性 | 使用 `stable_sort_by_key` 或在键中加入唯一性后缀 |
| CUDA 编译时间过长 | 为过多 GPU 架构生成代码 | 检查 `CMAKE_CUDA_ARCHITECTURES` 设置 | 开发阶段只编译当前 GPU 架构，发布时再编译多架构 |
| `cudaDeviceSynchronize` 后 CPU 利用率降低 | 同步调用阻塞了 CPU 线程 | 用 `nsight-sys` 查看 CPU-GPU 时间线 | 使用 stream 和异步 API 让 CPU 在 GPU 计算期间做其他工作 |
