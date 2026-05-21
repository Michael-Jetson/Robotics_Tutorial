# CUDA 在 SLAM 中的实际应用

> **定位**：这一章不是继续讲 CUDA 语法，而是讲怎样把 GPU 计算放进真实 SLAM 系统。
>
> 第 37 章解决的是“怎样写一个 GPU 计算阶段”。
>
> 本章解决的是“怎样让 GPU 计算在完整系统里真的变快、稳定、可维护”。

---

## 本章目标

学完本章后，你应该能够回答五个工程问题：

1. SLAM 中哪些模块适合 CUDA 加速。
2. 为什么单个 kernel 很快，不代表整条 SLAM 链路会变快。
3. 如何把 GPU 配准库、OpenCV CUDA、LibTorch C++ 放进 C++ 系统。
4. 如何设计 CPU/GPU fallback，避免 CUDA 变成部署单点故障。
5. 如何用端到端 benchmark 判断一次 GPU 改造是否值得。

本章强调一个原则：

> GPU 加速不是“把函数换成 CUDA 版本”。
>
> GPU 加速是对数据流、同步点、部署环境和故障边界的重新设计。

---

## 前置自测

如果下面的问题答不出两题以上，建议先回顾 实时约束与高性能数据传递、内存分配策略与pmr、缓存优化与数据布局 和 CUDA基础与Thrust：

1. 为什么 CUDA kernel launch 是异步的？
2. Host-to-Device 和 Device-to-Host 传输为什么可能吞掉 kernel 加速收益？
3. AoS 和 SoA 对相邻线程访问相邻地址有什么影响？
4. 为什么实时路径中不能临时创建大量 device buffer？
5. CPU/GPU fallback 为什么不能只看“主后端返回 false”？

这些问题决定本章的阅读重点。
本章不是教更多 CUDA 语法，而是教怎样把 GPU 放进一个会失败、会降级、会部署到不同机器的机器人系统。

---

## 章节依赖与知识树

阅读本章前，建议已经掌握：

| 前置章节 | 需要的内容 |
|----------|------------|
| 线程管理与互斥同步-实时约束与高性能数据传递 | 线程、异步任务、队列、同步和实时边界 |
| 内存分配策略与pmr | 内存分配策略和临时缓冲区复用 |
| 缓存优化与数据布局 | cache、AoS/SoA、数据布局 |
| CUDA基础与Thrust | CUDA 执行模型、Thrust、stream、event |

**知识树**：

```text
CUDA 在 SLAM 中的实际应用
├── 系统级判断（38.1 - 38.2）
│   ├── 哪些模块值得 GPU 化
│   ├── Amdahl 定律的系统视角
│   └── 算术强度与 Roofline 分析
├── CPU/GPU 边界工程（38.3）
│   ├── PCIe 带宽瓶颈
│   ├── CUDA Stream 流水线
│   └── Unified Memory 的实现机制
├── 应用模块（38.4 - 38.8）
│   ├── 点云配准（GICP/VGICP）
│   ├── GPU 因子图加速
│   ├── OpenCV CUDA 视觉前端
│   ├── LibTorch C++ 推理集成
│   └── 3D Gaussian Splatting
├── 部署与集成（38.9 - 38.11）
│   ├── Jetson 嵌入式部署
│   ├── CMake 可选 CUDA
│   └── 运行时 backend 选择
├── 质量保证（38.12 - 38.18）
│   ├── 端到端 benchmark
│   ├── 浮点精度与正确性验证
│   ├── CPU/GPU fallback
│   ├── 异步 pipeline
│   └── GPU 内存池复用
└── 设计方法论（38.20）
    └── 自顶向下 GPU 改造流程
```

回顾 线程管理与互斥同步-实时约束与高性能数据传递：
并发章节解决的是“多个线程如何交换数据而不破坏不变量”，实时章节解决的是“每个控制周期最多允许做多少工作”。
本章把这两件事放到 GPU 场景里：CUDA 后端不能只是算得快，还必须有明确队列边界、同步点和 fallback 策略。

如果只想先看应用，可以重点看：

1. 38.1：判断哪些模块值得上 GPU。
2. 38.4：点云配准库的接入方式。
3. 38.10-38.11：CMake 与运行时后端选择。
4. 38.12：benchmark 方法。
5. 38.19：常见失败模式。

---

## 38.1 为什么 SLAM 需要 GPU ⭐⭐

SLAM 的实时性压力来自三个方向：

1. 传感器频率越来越高。
2. 点云、图像、特征和地图规模越来越大。
3. 后端优化、局部建图、语义感知被塞进同一个实时系统。

一个移动机器人如果使用 10Hz LiDAR，每帧预算大约是 100ms。
如果再叠加相机、IMU、轮速计、局部规划和控制，单独留给 SLAM 前端的预算可能只有几十毫秒。

对于视觉 SLAM，30Hz 相机意味着每帧只有 33ms。
如果前端特征提取耗掉 20ms，后端优化偶尔再卡 50ms，系统就会出现延迟堆积。

对于 LiDAR SLAM，单帧点云可能有几万到几十万个点。
如果每个点都要参与滤波、坐标变换、邻域搜索、体素聚合、残差计算，CPU 端很容易被点云前端吃满。

这正是 GPU 有吸引力的地方。
SLAM 中大量计算不是复杂控制流，而是规则数据并行：

| 模块 | 典型并行粒度 |
|------|--------------|
| 图像去畸变 | 每个像素 |
| 金字塔构建 | 每个像素 |
| 特征描述子 | 每个关键点 |
| 深度估计 | 每个像素或每个匹配候选 |
| 点云变换 | 每个点 |
| 点云滤波 | 每个点 |
| 法向量/协方差估计 | 每个点及邻域 |
| ICP 残差 | 每个匹配 |
| 体素统计 | 每个点、每个体素 |
| 神经网络推理 | 张量批处理 |
| 3D Gaussian Splatting | 每个 splat、每个 tile、每个像素 |

但是 SLAM 又不是单纯的数据处理程序。
它有强烈的系统属性：

1. 输入来自实时传感器。
2. 输出会驱动定位、规划和控制。
3. 算法内部有状态。
4. 每帧数据量波动很大。
5. 失败帧不能让整个系统崩掉。
6. 机器人平台可能没有桌面 GPU。

所以 GPU 在 SLAM 中的第一条规则是：

> 只加速热点，不加速想象中的热点。

真正的热点需要 measurement，而不是凭经验猜。

> **本质洞察**：GPU 加速在 SLAM 中的价值不是"让某个函数更快"，而是**让连续的数据并行链路共享同一份 GPU 数据，减少 CPU-GPU 往返**。单点替换几乎总是得不偿失；链路级优化才能真正改变端到端延迟。

GPU 加速就像在工厂中引入一条自动化流水线：如果只把一个工位换成机器人而其他工位仍然手工操作，中间每次都要把工件从流水线上取下来再放上去，效率提升非常有限。真正的收益来自让多个连续工位都自动化，工件在流水线上一路通过不需要反复装卸。在 SLAM 中，"装卸"就是 CPU-GPU 数据传输，"连续自动化"就是让去畸变、滤波、体素化、残差计算都留在 GPU 上。

### 一个典型错误

假设某个 LiDAR 前端有如下耗时：

| 阶段 | CPU 耗时 |
|------|----------|
| 读取消息 | 2ms |
| 点云转换 | 4ms |
| 去畸变 | 8ms |
| 体素滤波 | 10ms |
| 配准 | 45ms |
| 关键帧判断 | 1ms |
| 发布结果 | 2ms |

总耗时是 72ms。
如果只把体素滤波改成 GPU，滤波阶段从 10ms 降到 2ms，看起来加速 5 倍。
但总耗时从 72ms 降到 64ms，只加速 1.125 倍。

如果 GPU 版本还需要额外上传和下载：

| 阶段 | 耗时 |
|------|------|
| upload | 3ms |
| GPU filter | 2ms |
| download | 3ms |

实际滤波阶段变成 8ms。
总耗时只从 72ms 降到 70ms。

这不是 CUDA 不快。
这是系统边界选错了。

更合理的改造是：

1. 点云上传一次。
2. 去畸变、滤波、协方差计算、配准残差尽量留在 GPU。
3. 只把最终位姿、Hessian、梯度或少量关键统计量传回 CPU。

### Amdahl 视角

如果一个系统中有比例 \(p\) 的部分可以被加速，且这部分加速比为 \(s\)，整体加速比为：

$$
S = \frac{1}{(1-p) + \frac{p}{s}}
$$

如果只有 20% 的时间可加速，即使那部分无限快：

$$
S_{\max}=\frac{1}{0.8}=1.25
$$

如果 80% 的时间可加速，且 GPU 加速 10 倍：

$$
S=\frac{1}{0.2 + 0.08}=3.57
$$

所以在 SLAM 中，GPU 加速的关键不是单点替换。
关键是扩大 \(p\)，也就是让尽可能多的连续阶段共享同一份 GPU 数据。

---

## 38.2 SLAM 中适合 CUDA 的模块地图 ⭐⭐

不同 SLAM 系统的瓶颈不同。
但从数据形态看，可以把 CUDA 适用性分成四类。

### 第一类：强适合

强适合的模块通常满足：

1. 数据量大。
2. 每个元素计算相似。
3. 分支少。
4. 输入输出结构固定。
5. 可以批处理。

典型例子：

| 模块 | 原因 |
|------|------|
| 图像金字塔 | 每个像素独立或局部相关 |
| 立体匹配 | 每个像素搜索视差 |
| 点云坐标变换 | 每个点同一矩阵 |
| 点云滤波 | 每个点同一规则 |
| 体素统计 | 大量点聚合 |
| ICP 残差计算 | 每个匹配独立计算残差和雅可比 |
| 神经网络推理 | 张量计算天然适合 GPU |
| 3DGS 渲染 | 大量 splat 和像素 tile |

### 第二类：条件适合

条件适合的模块本身有并行性，但是否值得上 GPU 取决于规模和数据流。

| 模块 | 适合条件 |
|------|----------|
| ORB 特征 | 高分辨率图像、特征量较大 |
| KLT 光流 | 多层金字塔、多特征点 |
| KNN 搜索 | 点数足够大，数据常驻 GPU |
| 小规模矩阵求解 | 批量很多时适合 |
| 局部地图更新 | 数据结构设计得足够规则 |
| 回环候选评分 | 候选数量和描述子规模足够大 |

### 第三类：通常不适合

这类模块不是不能用 GPU，而是上 GPU 后经常得不偿失。

| 模块 | 原因 |
|------|------|
| 小规模状态机 | 控制流复杂，数据少 |
| 单个 6x6 线性方程求解 | 规模太小 |
| ROS 消息调度 | 系统调用和内存管理为主 |
| 参数读取 | 不是热点 |
| 日志输出 | IO 主导 |
| 地图拓扑管理 | 分支和指针结构复杂 |

### 第四类：应该谨慎

这类模块很诱人，但需要强工程约束。

| 模块 | 风险 |
|------|------|
| 后端因子图全量 GPU 化 | 稀疏结构、动态拓扑和 CPU 库交互复杂 |
| 动态场景语义分割 | 模型延迟波动影响定位链路 |
| 在线重训练 | 资源竞争大，实时性难控 |
| 大规模地图重建 | 显存占用可能失控 |

### 深入分析：为什么 KNN 不适合 GPU 但体素滤波适合？

上面的分类表可能让人困惑：KNN 搜索和体素滤波都是点云操作，为什么体素滤波"强适合" GPU 而 KNN 只是"条件适合"？要回答这个问题，需要从 GPU 执行模型的两个核心限制出发分析——**分支发散（warp divergence）**和**内存访问模式（memory access pattern）**。

**体素滤波为什么适合 GPU？**

体素滤波的算法流程可以分解为：(1) 每个点计算所属体素的整数坐标 `(ix, iy, iz)`；(2) 把三维坐标编码为一维 key（如 Morton 编码或简单拼接）；(3) 按 key 排序所有点；(4) 对连续相同 key 的点做归约（求和/计数/取均值）。

从 GPU 执行模型看，每一步都是"GPU 友好"的：

- **步骤 1-2（key 计算）**：每个点独立计算，相邻线程执行完全相同的指令序列（几次乘法、取整、位操作），**没有分支**。内存访问是连续读取点云数组——相邻线程读相邻地址，完美 coalesced。
- **步骤 3（排序）**：Thrust 的 `sort_by_key` 使用 radix sort 或 merge sort，这些排序算法被设计为 GPU 友好——访问模式规则，分支极少。
- **步骤 4（归约）**：`reduce_by_key` 对连续相同 key 的元素做累加。由于排序后相同 key 的点连续存储，归约的内存访问也是连续的。

整个流程中，几乎所有线程在任何时刻都执行相同的指令，内存访问几乎总是连续的——这正是 GPU 最擅长的模式。

**KNN 搜索为什么不那么适合 GPU？**

KNN（K-Nearest Neighbors）搜索的典型实现是 KD-tree 遍历。对于每个查询点，搜索过程是：从根节点开始，根据点的坐标值决定走左子树还是右子树，递归直到叶节点，然后回溯检查是否需要访问另一侧子树。

从 GPU 执行模型看，KNN 有两个根本性问题：

**问题 1：严重的分支发散**。考虑一个 warp 中的 32 个查询点。在 KD-tree 的第一层，根据分割平面，一些点走左子树，另一些走右子树。这意味着同一 warp 中的线程要走不同的代码路径（左子树的递归 vs 右子树的递归）。GPU 的 SIMT 模型要求同一 warp 中所有活跃线程执行相同指令——当线程分叉时，GPU 必须**串行执行两条路径**，先执行左子树路径（右子树线程空闲等待），再执行右子树路径（左子树线程空闲等待）。树的每一层都可能产生分叉，深度为 D 的树最坏情况下 warp 效率降低到 $1/2^D$。对于 20 层的 KD-tree，理论最坏效率不到百万分之一。

**问题 2：不规则的内存访问**。KD-tree 的节点通常分散在内存各处。搜索过程中，每一步都根据分割条件跳转到一个可能完全不相邻的内存位置——这是典型的指针追逐（pointer chasing）。同一 warp 中的 32 个线程各自在树的不同位置，访问的内存地址完全不连续——无法 coalesced，GPU 的内存系统需要为每个线程发起独立的内存事务，带宽利用率极低。

**问题 3：不均匀的计算量**。不同查询点的搜索深度和回溯次数差异很大（取决于查询点附近的数据分布）。一个 warp 中最慢的线程决定了整个 warp 的完成时间——如果 31 个线程只需要搜索 10 步但 1 个线程需要搜索 100 步，整个 warp 都要等那 1 个线程。

**GPU 友好的 KNN 替代方案**：正因为 KD-tree KNN 不适合 GPU，SLAM 社区发展了更 GPU 友好的邻域搜索方法：

- **体素化邻域（如 iVox）**：把空间划分为体素网格，每个查询点只在其所属体素及相邻体素中搜索。体素查找用整数运算，邻域范围有限且固定，分支极少。
- **排序 + 二分搜索**：先按空间填充曲线（Morton code）排序所有点，然后对每个查询点做二分搜索找到附近的区域。排序和二分搜索都是 GPU 友好操作。
- **固定半径搜索**：只查找固定半径内的点，把问题转化为"判断两点距离是否小于阈值"——每个点独立计算，完美的数据并行。

这个分析揭示了一个更一般的原则：GPU 偏爱**结构化的计算模式**（所有线程做同样的事、访问连续内存），厌恶**数据依赖的控制流**（根据数据值走不同路径、指针追逐）。选择 GPU 加速目标时，不能只看"有没有并行性"，还要看"并行的结构是否规则"。

**KNN vs 体素滤波的 GPU 适合度量化对比**：

| 维度 | 体素滤波 | KD-tree KNN |
|------|---------|-------------|
| 分支发散 | 几乎为零（每点计算相同的乘法/取整） | 严重（每层树节点走左/右子树） |
| 内存访问模式 | 连续读点云数组，完美 coalesced | 指针追逐，每个线程访问不同节点 |
| 计算量均匀度 | 每个点计算量完全相同 | 不同查询点搜索深度差异大 |
| Warp 效率（估计） | >95% | 可低至 10-30%（深层树） |
| GPU vs CPU 加速比 | 通常 5-20x | 通常 1-3x（小数据可能更慢） |

这个对比说明：**GPU 适合度不是由"是否有并行性"决定的——KNN 有完美的数据并行性（每个查询独立），但它的控制流和内存访问模式与 GPU 架构不匹配**。这就是为什么 SLAM 社区在 GPU 上不使用 KD-tree KNN，而是用体素化邻域（如 iVox）或排序 + 二分搜索等 GPU 友好的替代方案。

**算术强度（Arithmetic Intensity）：判断 GPU 收益的量化工具**

上面的分析从定性角度判断 KNN 和体素滤波的 GPU 适合度。更精确的判断可以用**算术强度**这个指标——它是 Roofline 模型（缓存优化与数据布局 简要提到）的核心概念。

算术强度定义为每字节数据传输所执行的浮点运算数：

$$AI = \frac{\text{FLOP}}{\text{bytes transferred}}$$

对于一个给定的 GPU，有两个性能上限：**计算上限**（FLOP/s）和**内存带宽上限**（bytes/s）。如果 $AI$ 很低（每读一字节数据只做少量计算），性能受内存带宽限制（memory-bound）。如果 $AI$ 很高（每字节数据做大量计算），性能受计算能力限制（compute-bound）。两个上限的交叉点称为**ridge point**。

**SLAM 中典型操作的算术强度**：

| 操作 | 每点 FLOP | 每点字节数 | $AI$ (FLOP/byte) | 瓶颈类型 |
|------|-----------|-----------|-------------------|---------|
| 坐标变换（3x4 矩阵乘） | ~15 | 24 (读12+写12) | 0.625 | 极度 memory-bound |
| 体素 key 计算 | ~10 | 16 (读12+写4) | 0.625 | memory-bound |
| ICP 残差+雅可比 | ~100 | 72 (读24+写48) | 1.4 | memory-bound |
| 稠密矩阵乘（1024x1024） | ~2G | ~24M | ~85 | compute-bound |
| 神经网络推理（典型） | 变化大 | 变化大 | 10-100 | 通常 compute-bound |

大多数 SLAM 操作的算术强度远低于 GPU 的 ridge point（典型值 10-50 FLOP/byte）。这意味着 SLAM 的 GPU 加速主要受益于 GPU 的**高内存带宽**（400-2000 GB/s），而不是其**高计算吞吐**（数十 TFLOP/s）。

这个分析有几个实践意义：

1. **不需要最高端 GPU**。既然瓶颈是内存带宽而非计算能力，中端 GPU 和高端 GPU 在 SLAM 中的差距没有在深度学习训练中那么大。
2. **数据布局比算法优化更重要**。对于 memory-bound 操作，减少不必要的数据读取（SoA vs AoS、减少无用字段）比减少计算量更有效。
3. **合并操作可以提高算术强度**。如果一个 kernel 读取一次点坐标就同时完成变换、key 计算和有效性检查，$AI$ 会比三个独立 kernel 分别读取同一数据高得多——这就是 kernel fusion 的收益来源。

本章后面会不断回到一个判断：

> GPU 更适合吞吐型计算。
>
> SLAM 还要求延迟稳定。
>
> 吞吐提升必须被延迟预算接受，才算真正有用。

> ⚠️ **编程陷阱：只看 GPU kernel 时间就宣称"加速 10 倍"**
> **错误做法**：报告中写"GPU 配准 kernel 耗时 2ms，CPU 版本 20ms，加速 10 倍"。
> **现象**：实际系统端到端延迟从 20ms 降到 15ms，只加速了 1.33 倍。
> **根本原因**：只测了 kernel 时间，没有计入上传（3ms）、下载（2ms）、同步（1ms）和预处理（7ms 不变）。端到端加速比远低于 kernel 加速比。
> **正确做法**：benchmark 必须报告端到端延迟，包含所有传输、同步和预处理阶段。

> 💡 **概念误区：认为"条件适合"等于"一定值得上 GPU"**
> **新手想法**："KLT 光流在 GPU 上有加速论文，我也应该用 GPU 版本。"
> **实际上**："条件适合"意味着必须满足前提条件——数据规模够大、数据已在 GPU 上、后续阶段也在 GPU 上。如果你的系统只有 200 个特征点，且特征点结果需要立刻回 CPU 做 PnP，GPU 光流可能比 CPU 更慢。

> 🧠 **思维陷阱：把"不适合 GPU"理解为"不重要"**
> **新手想法**："状态机和参数管理不适合 GPU，说明它们不是性能瓶颈。"
> **实际上**：它们不适合 GPU 是因为计算模式不匹配（分支多、数据少），不是因为它们不重要。有时优化 CPU 端的状态管理（如减少锁竞争、优化数据结构）比引入 GPU 的收益更大。
> **正确思维**：先做 profiling 确定真正的热点，然后判断热点是否匹配 GPU 的执行模型。

### 练习

1. **[分析题]** 一个 LiDAR 前端有 5 个阶段，耗时分别为：读取 2ms、预处理 8ms、配准 45ms、关键帧判断 1ms、发布 2ms。如果把预处理和配准都搬到 GPU（各加速 5 倍），但引入 3ms 上传和 2ms 下载，端到端加速比是多少？
2. **[设计题]** 为上述前端画出两种 GPU 改造方案：(a) 只把配准搬到 GPU；(b) 把预处理+配准整体搬到 GPU。分析各方案的传输次数和端到端延迟。

---

## 38.3 CPU/GPU 边界：真正决定速度的地方 ⭐⭐

> **这一节解决什么问题**：CPU-GPU 数据传输是 GPU 加速中最大的瓶颈。用 PCIe 带宽 vs GPU 内存带宽的数量级差异来量化这个问题。

### 数量级差异：为什么 CPU-GPU 传输常常吃掉 GPU 加速收益

CUDA 初学者容易把问题看成：

```text
CPU function -> GPU function
```

真实 SLAM 系统里的问题更接近：

```text
sensor -> message -> host buffer -> device buffer -> kernels -> sync -> host result -> estimator
```

每一个箭头都有成本。理解这些成本的**数量级关系**是判断 GPU 加速是否值得的前提。

先看一组关键数字——它们揭示了 GPU 加速中最容易被忽视的瓶颈：

| 数据路径 | 典型带宽 | 传输 1.2MB 点云的耗时 |
|---------|---------|---------------------|
| GPU 内部（GDDR6X，RTX 3090） | ~936 GB/s | ~0.0013 ms |
| GPU 内部（HBM2e，A100） | ~2039 GB/s | ~0.0006 ms |
| CPU-GPU（PCIe 3.0 x16） | ~12 GB/s | ~0.1 ms |
| CPU-GPU（PCIe 4.0 x16） | ~25 GB/s | ~0.048 ms |
| CPU 内存（DDR4-3200 双通道） | ~51 GB/s | ~0.024 ms |

GPU 内部带宽比 PCIe 带宽大 **40-170 倍**。这意味着：一旦数据在 GPU 显存中，GPU 处理速度极快。但把数据从 CPU 搬到 GPU 的过程本身就是严重瓶颈。打一个直观的比方：GPU 内部像高速公路（时速 300km/h），PCIe 像进出高速的匝道（限速 30km/h）。如果你的工件需要频繁上下匝道，高速公路再宽也没用——系统吞吐被匝道限制住了。

### 数据移动成本：PCIe 带宽是最大瓶颈

桌面独立显卡通常通过 PCIe 与 CPU 通信。
这意味着 CPU 内存和 GPU 显存是两块不同的物理存储。
数据从 CPU 到 GPU 要 upload。
数据从 GPU 到 CPU 要 download。

**PCIe 带宽的物理限制是 GPU 加速中被低估最多的瓶颈。** 要理解为什么 SLAM 的 GPU 改造常常令人失望，必须先理解 PCIe 总线的实际性能：

| PCIe 版本 | 单向理论带宽（x16） | 实测有效带宽 | 与 GPU 显存带宽对比 |
|-----------|---------------------|-------------|-------------------|
| PCIe 3.0 | ~16 GB/s | ~12 GB/s | GPU 显存 400-900 GB/s |
| PCIe 4.0 | ~32 GB/s | ~25 GB/s | GPU 显存 400-900 GB/s |
| PCIe 5.0 | ~64 GB/s | ~50 GB/s | GPU 显存 400-2000 GB/s |

注意两个数字的巨大差距：GPU 内部的显存带宽比 PCIe 带宽大 **10-50 倍**。这意味着：一旦数据在 GPU 显存中，GPU 处理速度极快；但把数据从 CPU 搬到 GPU 的过程本身就是一个严重瓶颈。

**具体到 SLAM 的数据规模**：一帧 LiDAR 点云有 100K 个点，每个点 `Point3f`（12 字节），总计 1.2 MB。通过 PCIe 3.0 传输 1.2 MB 约需要 0.1ms——看起来不多。但如果传输是同步的（CPU 等待传输完成再继续），这 0.1ms 就是白白浪费的 CPU 时间。而且每帧如果有 3 次往返（上传原始点云、中间同步、下载结果），就是 0.6ms——在 100ms 的帧预算中已经占了 0.6%。如果数据量更大（比如稠密深度图 640x480 float = 1.2MB），或者传输更频繁（每个阶段都往返），传输成本会迅速累积。

**更隐蔽的成本是传输的固定延迟**：即使传输数据量很小（比如只下载一个 6x6 Hessian 矩阵，288 字节），每次 `cudaMemcpy` 仍然有一个固定的启动延迟，通常在 5-15 微秒。这是因为 CPU 需要向 GPU 驱动发送命令、驱动需要设置 DMA 控制器、DMA 控制器需要获取 PCIe 总线访问权。对于频繁的小传输，这个固定延迟比数据传输本身还大。

**嵌入式平台（如 Jetson）的情况不同但不是没有代价**：Jetson 系列使用统一内存架构（CPU 和 GPU 共享同一块物理 DRAM），没有 PCIe 总线的带宽限制。但共享内存也意味着 CPU 和 GPU 共享内存带宽——如果 GPU 密集访问内存，CPU 的实时线程也可能被拖慢。而且 Jetson 的总内存带宽（通常 25-60 GB/s）远低于桌面 GPU 的显存带宽，这意味着内存密集型 kernel 在 Jetson 上的性能上限更低。

如果每一帧都这样：

```text
upload raw cloud
run GPU filter
download filtered cloud
upload filtered cloud
run GPU registration
download transformed cloud
upload transformed cloud
run GPU residual
download residuals
```

那么 GPU 算得再快也会被传输吞掉。

正确的数据流应该更像：

```text
upload raw cloud once
run deskew
run filter
run voxel statistics
run correspondence
run residual reduction
download pose update or H/b
```

也就是说，GPU 加速要尽量形成连续链条。

### CUDA Stream 的并发模型：流水线化传输和计算

即使做到了"上传一次、GPU 连续处理、只下载小结果"，仍然有一个问题：在上传期间 GPU 在空闲，在计算期间 CPU 在空闲。CUDA stream 的设计目标就是让这些阶段重叠起来。

**Stream 是什么？** Stream 是 GPU 侧的一个有序操作序列。同一 stream 中的操作按提交顺序执行——在前一个操作完成之前，后一个操作不会开始。但**不同 stream 中的操作可以并发执行**（取决于 GPU 硬件是否有足够的资源）。

**默认 stream 的问题**：CUDA 的所有操作如果不指定 stream，都会进入默认 stream（也叫 stream 0）。默认 stream 有一个特殊行为：它会和所有其他 stream 同步。这意味着如果你的所有操作都在默认 stream 中，传输和计算就是严格串行的：

```text
时间线（默认 stream，全部串行）：
CPU:  [准备数据]----[等待]---------[等待]---------[处理结果]
GPU:  -----------[上传][计算阶段1][计算阶段2][下载]
```

**使用多个 stream 实现流水线**：通过把上传、计算和下载放入不同 stream（或同一 stream 内利用异步 API），可以实现重叠：

```text
时间线（多 stream，流水线化）：
Stream 1 (传输):  [上传帧N]-----------[上传帧N+1]--------
Stream 2 (计算):  ---------[计算帧N]------------[计算帧N+1]
Stream 3 (下载):  ------------------[下载帧N]------------
CPU:              [准备N+1][准备N+2]---[处理N的结果]------
```

在理想情况下，帧 N 的计算和帧 N+1 的上传同时进行，CPU 也在同时准备帧 N+2 的数据——三个不同的硬件单元（DMA 引擎、计算单元、CPU）同时工作，总吞吐量显著提高。

**流水线化的前提条件**：

1. **异步传输 API**：必须使用 `cudaMemcpyAsync`（而不是 `cudaMemcpy`），否则传输函数本身会阻塞 CPU。
2. **Pinned memory**：异步 DMA 传输要求 host 端内存是页锁定的（pinned memory，通过 `cudaMallocHost` 或 `cudaHostAlloc` 分配）。普通 `malloc` 分配的内存可能被操作系统换出到磁盘，DMA 引擎无法直接访问，所以 `cudaMemcpyAsync` 在内部会退化为同步传输。
3. **显式 stream 管理**：必须创建非默认 stream，并把操作提交到指定 stream。

**为什么很多 SLAM 项目不使用 stream 流水线？** 因为流水线化引入了显著的工程复杂度：你需要管理多帧的缓冲区（帧 N 的数据在 GPU 上计算时，帧 N+1 的数据需要另一块 GPU buffer 来接收上传），需要处理帧间的数据依赖（如果帧 N 的配准结果影响帧 N+1 的初始猜测），需要正确设置 stream 间的事件同步。对于教学项目和原型，使用默认 stream + 同步点是完全合理的起步方式。只有当 profiling 明确显示"CPU 等待 GPU"或"GPU 等待 CPU"是瓶颈时，才需要引入 stream 流水线。

### Unified Memory 的实现原理

CUDA基础与Thrust 提到 Unified Memory 降低了编程复杂度，但没有详细解释它的实现机制。理解这个机制对于判断"什么时候用 Unified Memory 可以接受"至关重要。

**核心机制：页错误驱动的按需迁移**。Unified Memory（通过 `cudaMallocManaged` 分配）创建的内存页面在 CPU 和 GPU 之间按需迁移。初始分配时，页面不一定在任何一侧的物理内存中。当 CPU 首次访问某一页时，CUDA 驱动发现该页当前不在 CPU 可访问的内存中（要么在 GPU 显存中，要么还没分配物理页面），触发一次**页错误（page fault）**，驱动把该页从 GPU 显存迁移到 CPU 内存（或首次分配物理页面）。反过来，当 GPU kernel 首次访问某一页时，也触发 GPU 端的页错误，驱动把该页从 CPU 内存迁移到 GPU 显存。

**为什么在离散 GPU 上性能通常不如显式管理？** 有几个原因：

1. **页错误延迟远高于 DMA 传输延迟**。一次 `cudaMemcpy` 启动 DMA 传输引擎，以 PCIe 总线全速度连续传输所有数据。而 Unified Memory 的页错误是逐页触发的——每一页（通常 4KB 或 64KB）都需要一次独立的迁移操作，每次迁移有独立的启动开销。如果 GPU kernel 随机访问 1GB 的 Unified Memory 数据，可能产生数千次页错误，每次页错误的处理延迟约 10-50 微秒——总延迟远超一次批量 DMA 传输。

2. **页面可能"乒乓迁移"**。如果 CPU 和 GPU 在短时间内交替访问同一页数据，该页面会在两侧内存之间反复迁移，每次迁移都有 PCIe 延迟。这在某些使用模式下（比如 GPU 计算结果后 CPU 读取检查，然后 GPU 继续计算）可能导致性能灾难。

3. **预取优化需要程序员干预**。CUDA 提供了 `cudaMemPrefetchAsync` 来主动把 Unified Memory 页面迁移到指定设备，避免运行时页错误。但使用这个 API 的代码和显式 `cudaMemcpyAsync` 的代码几乎一样复杂——Unified Memory 的"自动化"优势被削弱了。

**什么时候 Unified Memory 是合理的选择？**

- **共享内存平台（Jetson）**：CPU 和 GPU 共享物理 DRAM，Unified Memory 不需要真正的数据迁移，只需要调整页表映射。性能损失很小，编程便利性的收益很大。
- **原型和算法验证**：在确认算法正确性阶段，不需要追求极致性能，Unified Memory 大幅降低代码复杂度。
- **CPU 和 GPU 交替访问模式不频繁的场景**：如果数据上传到 GPU 后连续处理多个 kernel，最后才回到 CPU——页错误只在边界处发生，中间阶段的性能和显式管理接近。

### 同步成本

CUDA kernel 默认是异步提交。
如果代码中频繁调用同步函数，GPU 和 CPU 就会互相等待。

常见隐式同步点包括：

1. `cudaDeviceSynchronize()`。
2. 从 device 数据拷贝到 host。
3. 某些错误检查宏。
4. 某些 Thrust 算法返回前的同步。
5. 获取 GPU 结果用于 CPU 分支判断。

同步不是坏事。
没有同步，系统不知道结果何时可用。
问题在于无意识同步。

错误模式：

```cpp
for (const auto& stage : stages) {
  stage.launch();
  cudaDeviceSynchronize();
}
```

更合理：

```cpp
for (const auto& stage : stages) {
  stage.launch(stream);
}

cudaEventRecord(done, stream);
cudaEventSynchronize(done);
```

第一种写法让每个阶段都阻塞。
第二种写法允许多个 GPU 阶段在同一个 stream 中连续排队，只在真正需要结果时同步。

### 数据结构边界

很多 SLAM 项目最难 GPU 化的地方不是 kernel。
而是数据结构。

例如 CPU 端点云常用：

```cpp
struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;
};
```

这个结构对单点操作很直观。
但如果 GPU kernel 每次只需要 `x,y,z`，而 intensity 很少用，AoS 可能导致多余读。

另一种布局是 SoA：

```cpp
struct DeviceCloudSoA {
  float* x;
  float* y;
  float* z;
  float* intensity;
  int size;
};
```

SoA 更适合某些 GPU 访问模式。
但它会让 CPU 端接口变复杂。

所以工程上常见折中是：

1. 外部接口保持 PCL 或 ROS 常用格式。
2. GPU 后端内部转换成适合计算的布局。
3. 多个 GPU 阶段共享内部布局。
4. 最后只在系统边界转换一次。

### 状态边界

SLAM 不是单帧算法。
它有地图、关键帧、局部窗口、历史状态。

GPU 端状态需要回答：

1. 谁拥有 device memory？
2. 哪些数据跨帧复用？
3. 地图更新时，GPU 缓存如何失效？
4. CPU 和 GPU 状态是否可能不一致？
5. 异常发生时怎样释放资源？

这就是为什么本章不断强调 RAII。
如果 GPU 后端只是临时函数，资源生命周期很容易散落在系统各处。
如果 GPU 后端被封装为对象，就可以把显存、stream、event、scratch buffer 放在同一个生命周期里。

CPU/GPU 边界和跨国供应链的海关是类似的概念：数据过"海关"（PCIe 传输）需要时间和成本。减少过关次数（减少传输往返）比加快过关速度（增加带宽）更有效。最好的策略是让原材料进入目的国后在当地完成所有加工步骤，最终只把成品运回——对应"上传原始点云，GPU 上完成所有处理，只下载位姿更新"。

如果每个 GPU 阶段之间都做一次同步会怎样？假设 pipeline 有 6 个阶段，每次 `cudaDeviceSynchronize` 约 50 微秒（包含 CPU-GPU 协调开销）。6 次同步就是 300 微秒——可能比某些阶段的 kernel 本身还长。更严重的是，同步阻止了 CPU 在等待期间做其他工作（如准备下一帧、处理 IMU 数据），白白浪费 CPU 时间。

> ⚠️ **编程陷阱：频繁 `cudaDeviceSynchronize` 破坏 GPU pipeline**
> **错误做法**：每个 Thrust 调用后都加 `cudaDeviceSynchronize()` "确保安全"。
> **现象**：GPU 利用率只有 20-30%，大量时间 CPU 和 GPU 互相等待。
> **根本原因**：过度同步把本应异步执行的 GPU 操作串行化。GPU 和 CPU 失去了重叠执行的机会。
> **正确做法**：只在真正需要 GPU 结果的位置同步。同一 stream 内的操作已经按顺序执行，不需要额外同步。

> ⚠️ **编程陷阱：pinned memory 滥用导致系统可用内存不足**
> **错误做法**：为所有数据结构都使用 `cudaHostAlloc` 分配 pinned memory，"因为它传输更快"。
> **现象**：程序运行后系统可用内存急剧下降，其他进程开始 swap，整体性能恶化。
> **根本原因**：pinned memory 锁定物理页面，不能被操作系统换出。大量 pinned memory 会减少系统可分页内存，影响其他所有进程。
> **正确做法**：只对 GPU 异步传输的 staging buffer 使用 pinned memory，总量控制在物理内存的 10-20% 以内。

### 练习

1. **[设计题]** 画出一个 6 阶段 GPU pipeline 的时序图：过度同步版本（每步都 sync）和最小同步版本（只在最后 sync）。标注 CPU/GPU 的空闲时间差异。
2. **[分析题]** SoA 和 AoS 在 GPU 上的 coalesced access 差异是什么？如果一个 SLAM 系统外部接口是 PCL（AoS），GPU 内部使用 SoA，在哪里做转换成本最低？

---

## 38.4 点云配准库：从 GICP 到 GPU VGICP ⭐⭐

LiDAR SLAM 中，配准经常是最主要的热点。
典型任务是：

> 给定当前帧点云和局部地图，估计二者之间的刚体变换。

经典 ICP 的目标可以写成：

$$
\min_{\mathbf{T}\in SE(3)}
\sum_i
\left\|
\mathbf{p}_i^{map} -
\mathbf{T}\mathbf{p}_i^{scan}
\right\|^2
$$

点到点 ICP 简单，但对噪声、采样密度和局部几何不够稳。
它把每个点都当成各向同性测量。
如果目标点云来自墙面，沿墙面切向滑动一点，几何意义上并不严重；但点到点误差会把切向和法向误差同等惩罚。
这会让优化器在平面、走廊、地面这类结构化场景里过度相信并不可靠的方向。

点到面 ICP 使用法向量：

$$
r_i =
\mathbf{n}_i^\top
(\mathbf{p}_i^{map} - \mathbf{T}\mathbf{p}_i^{scan})
$$

GICP 进一步把局部协方差引入误差度量。
直觉上，点不是无限精确的球形测量，而是有局部面结构的不确定性。
如果一个点的邻域像平面，法向方向不确定性小，切向方向不确定性大；误差度量就应该更重视法向偏差，而不是把三个方向混在一起。
这条演进线可以概括为：

| 方法 | 误差假设 | 解决的问题 | 新代价 |
|------|----------|------------|--------|
| 点到点 ICP | 点是各向同性测量 | 最简单的刚体配准 | 对局部几何不敏感 |
| 点到面 ICP | 目标局部近似平面 | 平面场景更稳 | 需要法向估计 |
| GICP | 每点有局部协方差 | 能表达各向异性不确定性 | 邻域统计和矩阵运算更重 |
| VGICP | 体素代表局部统计 | 统计更规整，适合大规模点云 | 受体素分辨率影响 |

VGICP 又把点云聚合到体素中，用体素内统计量近似局部结构。
这让邻域搜索和统计计算更规整，也更适合并行。
这也是它适合 GPU 的原因：体素 key 计算、排序、分组、每体素统计和每点残差都能拆成大量相似任务。
GPU 不喜欢复杂指针拓扑，却很擅长这种“对许多点执行同一类局部计算”的模式。

### GPU 在配准中的位置

配准里适合 GPU 的部分包括：

| 阶段 | GPU 价值 |
|------|----------|
| 点云变换 | 每点独立 |
| 体素构建 | 每点计算 key，再排序/聚合 |
| 协方差估计 | 每点或每体素统计 |
| 最近邻/邻域搜索 | 大量独立查询 |
| 残差计算 | 每个匹配独立 |
| Hessian/gradient 累加 | 大量小矩阵归约 |

不一定适合 GPU 的部分包括：

| 阶段 | 原因 |
|------|------|
| 6x6 线性方程求解 | 规模太小，CPU 足够快 |
| 外层迭代逻辑 | 分支和收敛判断较多 |
| 参数管理 | 不是热点 |
| ROS 消息转换 | 主要是格式和内存管理 |

上面两张表清楚地展示了一个关键的工程判断：GPU 加速不是"全或无"的选择——大多数成功的 GPU SLAM 系统都是混合架构。GPU 负责数据密集的并行计算（数万个点的变换、残差、归约），CPU 负责控制逻辑和小规模线性代数（6x6 矩阵求解只需纳秒级，启动一次 GPU kernel 的开销就已经远超计算本身）。这种分工不是"GPU 化不彻底"，而是让每种硬件做最擅长的事情。

所以很多工程实现会采用混合架构：

```text
GPU: correspondence + residual + H/b
CPU: solve 6x6 update + convergence logic
```

这不是“GPU 化不彻底”。
这是合理分工。

### 接入现成 GPU 配准库

项目中常见做法不是从零写 CUDA kernel，而是接入成熟库。例如某些 GICP/VGICP 库提供了 CPU 版本和 CUDA 版本，并尽量保持相似接口。

**为什么要用虚接口封装 GPU 后端？** 这不只是"好的软件工程"——在机器人系统中它是**部署必需**。一个 SLAM 系统可能需要部署到三种平台：桌面开发机（有 RTX 3090）、嵌入式平台（Jetson，有集成 GPU 但性能有限）、纯 CPU 平台（某些工业 ARM 板，没有 GPU）。如果 SLAM 主循环直接依赖 CUDA 头文件和 GPU 类名，那么在没有 GPU 的平台上连**编译都不行**——即使你只想用 CPU 后端。通过虚接口隔离，主循环完全不知道底层是 CPU 还是 GPU，编译时通过 CMake 选项决定链接哪个后端。

概念上可以这样封装：

```cpp
#include <cuda_runtime.h>
#include <memory>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class RegistrationBackend {
public:
  using PointT = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointT>;

  virtual ~RegistrationBackend() = default;

  // 统一接口暴露“是否使用 GPU”，避免主流程依赖具体后端类型。
  virtual bool usesGpu() const = 0;

  virtual bool align(const Cloud& source,
                     const Cloud& target,
                     const Eigen::Matrix4f& initial_guess,
                     Eigen::Matrix4f* result) = 0;
};
```

CPU 后端：

```cpp
class CpuRegistrationBackend final : public RegistrationBackend {
public:
  bool usesGpu() const override {
    return false;
  }

  bool align(const Cloud& source,
             const Cloud& target,
             const Eigen::Matrix4f& initial_guess,
             Eigen::Matrix4f* result) override {
    if (result == nullptr) {
      return false;
    }

    // CPU 后端可以封装 PCL ICP、NDT、GICP 或 CPU VGICP。
    // 主流程只关心统一接口，不关心底层库类名。
    Eigen::Matrix4f estimate = initial_guess;

    // runCpuRegistration(source, target, initial_guess, &estimate);

    *result = estimate;
    return true;
  }
};
```

GPU 后端：

```cpp
class GpuRegistrationBackend final : public RegistrationBackend {
public:
  bool usesGpu() const override {
    return true;
  }

  bool available() const {
    // 工厂函数用这个接口在启动阶段判断 GPU 后端是否可用。
    return isCudaAvailable();
  }

  bool align(const Cloud& source,
             const Cloud& target,
             const Eigen::Matrix4f& initial_guess,
             Eigen::Matrix4f* result) override {
    if (result == nullptr) {
      return false;
    }

    if (!available()) {
      return false;
    }

    // 真实项目中，这里会调用 fast GICP / VGICP 类库的 CUDA 后端。
    // 不同版本的类名和参数可能变化，应以项目头文件为准。
    Eigen::Matrix4f estimate = initial_guess;

    // gpu_registration_.setInputSource(source.makeShared());
    // gpu_registration_.setInputTarget(target.makeShared());
    // gpu_registration_.align(output_cloud, initial_guess);
    // estimate = gpu_registration_.getFinalTransformation();

    *result = estimate;
    return true;
  }

private:
  bool isCudaAvailable() const {
    int device_count = 0;
    const cudaError_t err = cudaGetDeviceCount(&device_count);
    return err == cudaSuccess && device_count > 0;
  }
};
```

上面代码故意不把库类名写死。
原因是这里真正要强调的是接口边界：

1. SLAM 主流程不应直接依赖某个 GPU 类。
2. GPU 后端失败时应能回退 CPU。
3. 配准结果和状态要通过统一接口返回。
4. benchmark 要在统一接口外层统计。

### 后端选择

一个简单的工厂函数：

```cpp
#include <memory>
#include <stdexcept>
#include <string>

std::unique_ptr<RegistrationBackend> createRegistrationBackend(
    const std::string& backend_name) {
  if (backend_name == "gpu") {
    auto backend = std::make_unique<GpuRegistrationBackend>();
    return backend;
  }

  return std::make_unique<CpuRegistrationBackend>();
}
```

真实项目中还需要更细：

```cpp
struct RegistrationConfig {
  std::string backend = "auto";
  double voxel_resolution = 0.5;
  int max_iterations = 32;
  int num_threads = 4;
  bool allow_cpu_fallback = true;
  bool keep_device_map = true;
};
```

选择逻辑：

```cpp
std::unique_ptr<RegistrationBackend> createRegistrationBackend(
    const RegistrationConfig& config) {
  if (config.backend == "cpu") {
    return std::make_unique<CpuRegistrationBackend>();
  }

  if (config.backend == "gpu" || config.backend == "auto") {
    auto gpu = std::make_unique<GpuRegistrationBackend>();

    // GPU 后端不可用时不要把失败状态伪装成一个可用对象。
    if (gpu->available()) {
      return gpu;
    }

    if (config.backend == "gpu" && !config.allow_cpu_fallback) {
      throw std::runtime_error("CUDA backend required but unavailable");
    }
  }

  return std::make_unique<CpuRegistrationBackend>();
}
```

教学重点不是这段代码本身。
重点是：

> 配准后端是策略，不应该散落在 SLAM 主循环里。

> **本质洞察**：CPU/GPU 后端切换不是不是"if-else 两行代码"的问题，而是**接口设计问题**。如果 SLAM 主循环直接依赖 GPU 类名和 CUDA 头文件，那么没有 GPU 的机器连编译都不行。正确的设计是通过虚接口或策略模式隔离后端，让主循环完全不知道底层用的是 CPU 还是 GPU。

配准后端的策略模式和 并行编程框架 的执行策略（SerialPolicy/OpenMPPolicy/TBBPolicy）是完全同构的设计：都是把"做什么"和"怎么做"分离，让调用者选择实现方式而不改变算法逻辑。

> ⚠️ **编程陷阱：GPU 配准首帧延迟尖峰**
> **错误做法**：没有预热，直接在第一帧调用 GPU 配准。
> **现象**：第一帧延迟 200ms+，后续帧稳定在 10ms。
> **根本原因**：首次 CUDA 调用触发驱动初始化、JIT 编译（如果只有 PTX）和内存池建立。这些初始化成本只发生一次，但如果没有预热，就落在了第一帧的实时预算内。
> **正确做法**：在系统启动阶段（非实时路径中）做一次 dummy 计算预热 GPU 后端。把首帧延迟从 benchmark 报告中单独列出。

### 练习

1. **[设计题]** 为配准后端设计一个工厂函数：根据配置字符串 `"cpu"/"gpu"/"auto"` 创建对应后端。当 `"gpu"` 不可用且 `allow_cpu_fallback=false` 时应该怎么处理？
2. **[分析题]** GPU 配准在室内低线数雷达（每帧 5000 点）和室外 128 线雷达（每帧 200K 点）上的收益差异可能有多大？从 Amdahl 定律和传输占比两个角度分析。

### 端到端对比

评估 GPU 配准时，不要只看库内部打印的 kernel 时间。
至少记录：

| 指标 | 含义 |
|------|------|
| preprocess time | 点云转换、去畸变、滤波 |
| upload time | CPU 到 GPU |
| registration time | 配准核心 |
| download time | GPU 到 CPU |
| solve time | 小规模线性求解 |
| total frontend time | 前端总耗时 |
| tracking lost rate | 跟踪失败比例 |
| trajectory error | 轨迹误差 |
| max latency | 最坏帧耗时 |
| memory usage | 显存和主存占用 |

如果 GPU 版本平均更快，但最坏帧更慢，机器人可能仍然不稳定。
定位系统关心的是实时闭环，而不只是平均吞吐。

### 配准 GPU 化的常见坑

| 问题 | 表现 | 原因 | 处理 |
|------|------|------|------|
| 小点云更慢 | 室内低线数雷达上 GPU 版本慢 | 上传和 launch 成本占比高 | 设置最小点数阈值 |
| 首帧很慢 | 第一帧延迟尖峰 | CUDA 初始化或缓存构建 | 启动阶段预热 |
| 局部地图太大 | 显存持续增长 | 地图缓存未裁剪 | 维护局部窗口 |
| 结果与 CPU 不完全一致 | 轨迹有微小差异 | 浮点归约顺序不同 | 用误差阈值比较 |
| 偶发超时 | 某些场景匹配数量暴涨 | 动态点或退化环境 | 限制输入规模并做退化检测 |

---

## 38.5 GPU 因子：把加速放进优化问题 ⭐⭐⭐

SLAM 后端常写成因子图：

$$
\min_{\mathbf{x}}
\sum_i
\left\|
\mathbf{r}_i(\mathbf{x})
\right\|_{\Sigma_i}^{2}
$$

其中状态 \(\mathbf{x}\) 可以包含位姿、速度、偏置、地图点。
因子 \(\mathbf{r}_i\) 可以来自 IMU、LiDAR、视觉、轮速计或先验。

对于 LiDAR 因子，残差数量可能很大。
每个残差的计算很适合并行。
但最终优化器通常需要 Hessian 和 gradient：

$$
\mathbf{H} = \sum_i \mathbf{J}_i^\top \mathbf{W}_i \mathbf{J}_i
$$

$$
\mathbf{b} = \sum_i \mathbf{J}_i^\top \mathbf{W}_i \mathbf{r}_i
$$

这给了 GPU 一个自然位置：

1. GPU 上为每个匹配计算残差和雅可比。
2. GPU 上并行归约得到 \(\mathbf{H}\) 和 \(\mathbf{b}\)。
3. CPU 只接收小矩阵。
4. CPU 端优化器继续负责稀疏图结构和求解。

这种设计避免把完整后端全部搬到 GPU，它保留了成熟 CPU 优化库的稳定性，同时让最重的残差计算上 GPU。**量化这个设计的数据传输量**：假设一帧有 50000 个匹配点，每个匹配产生一个 6 维残差和一个 $6 \times 6$ 的小 Hessian。如果把所有残差下载到 CPU 再累加，需要传输 $50000 \times (6 + 36) \times 8 = 16.8$ MB。但如果在 GPU 上先归约，只需下载一个 $6 \times 6$ 的 H 矩阵和一个 $6 \times 1$ 的 b 向量，总共 $42 \times 8 = 336$ 字节——传输量减少了 **50000 倍**。在 PCIe 3.0 上，16.8 MB 需要约 1.4ms，336 字节几乎瞬间完成。这就是"GPU 归约后只下载小结果"策略的巨大价值。

### 因子接口示意

概念上，一个 GPU LiDAR 因子可以长这样：

```cpp
struct LinearizedFactor {
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;
  double cost = 0.0;
  int num_residuals = 0;
};

class GpuLidarFactor {
public:
  void setSourceCloud(DeviceCloudHandle source);
  void setTargetMap(DeviceMapHandle map);
  void setNoiseModel(double sigma);

  LinearizedFactor linearize(const Eigen::Isometry3d& T_world_lidar);

private:
  DeviceCloudHandle source_;
  DeviceMapHandle map_;
  double sigma_ = 0.1;
};
```

这里的核心是 `linearize()`。
它不返回全部残差。
它返回优化器需要的小结果。

这比下载所有匹配点更合理。

### 为什么不直接把因子图全搬上 GPU

完整因子图后端有几个特点：

1. 图结构动态变化。
2. 稀疏矩阵结构复杂。
3. 边缘化会改变先验。
4. 回环会引入非局部连接。
5. 需要鲁棒核和退化处理。
6. CPU 端成熟库生态更完整。

GPU 当然可以做稀疏线性代数。
但在教学和工程项目里，先做局部残差 GPU 化通常更稳。

一个务实路线是：

| 阶段 | 建议 |
|------|------|
| 初始系统 | CPU 因子图 |
| 前端成为瓶颈 | GPU 配准 |
| LiDAR 残差成为瓶颈 | GPU factor linearization |
| 大规模稠密建图 | GPU map representation |
| 特定平台深度优化 | 再考虑更多 GPU 求解 |

---

## 38.6 OpenCV CUDA：视觉前端的 GPU 接入 ⭐⭐

视觉 SLAM 的前端常见任务包括：

1. 图像去畸变。
2. 灰度转换。
3. 金字塔构建。
4. 特征提取。
5. 描述子计算。
6. 光流跟踪。
7. 立体匹配。
8. 深度图滤波。

OpenCV 提供 CUDA 模块时，核心对象是 `cv::cuda::GpuMat`。
它对应 GPU 端图像。

最小使用模式：

```cpp
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>

cv::Mat image_bgr = readImageFromCamera();

cv::cuda::GpuMat d_bgr;
cv::cuda::GpuMat d_gray;

d_bgr.upload(image_bgr);
cv::cuda::cvtColor(d_bgr, d_gray, cv::COLOR_BGR2GRAY);

cv::Mat gray;
d_gray.download(gray);
```

这段代码能跑，但不是高性能模式。
原因是它每帧都 upload 和 download。

如果后续算法也在 GPU 上，应让数据留在 `GpuMat`：

```cpp
struct GpuImageFrame {
  cv::cuda::GpuMat gray;
  cv::cuda::GpuMat pyramid_level0;
  cv::cuda::GpuMat pyramid_level1;
  cv::cuda::GpuMat pyramid_level2;
  double timestamp = 0.0;
};
```

然后视觉前端围绕 GPU frame 工作：

```cpp
class GpuVisualPreprocessor {
public:
  GpuImageFrame run(const cv::Mat& bgr, double timestamp) {
    GpuImageFrame frame;
    frame.timestamp = timestamp;

    d_bgr_.upload(bgr, stream_);
    cv::cuda::cvtColor(d_bgr_, frame.gray, cv::COLOR_BGR2GRAY, 0, stream_);
    cv::cuda::pyrDown(frame.gray, frame.pyramid_level1, stream_);
    cv::cuda::pyrDown(frame.pyramid_level1, frame.pyramid_level2, stream_);

    stream_.waitForCompletion();
    frame.pyramid_level0 = frame.gray;
    return frame;
  }

private:
  cv::cuda::GpuMat d_bgr_;
  cv::cuda::Stream stream_;
};
```

### 光流跟踪

KLT 光流适合 GPU 的原因是每个特征点的局部窗口搜索结构相似。
OpenCV CUDA 中可以使用稀疏金字塔光流接口。
不同 OpenCV 版本的函数签名可能略有差异，工程中应以本机头文件为准。

概念接口：

```cpp
#include <opencv2/cudaoptflow.hpp>

class GpuKltTracker {
public:
  void initialize(const cv::cuda::GpuMat& gray,
                  const cv::Mat& initial_points) {
    previous_gray_ = gray.clone();
    previous_points_.upload(initial_points);
  }

  void track(const cv::cuda::GpuMat& gray,
             cv::Mat* tracked_points,
             cv::Mat* status) {
    optical_flow_->calc(previous_gray_,
                        gray,
                        previous_points_,
                        current_points_,
                        status_gpu_);

    current_points_.download(*tracked_points);
    status_gpu_.download(*status);

    previous_gray_ = gray.clone();
    previous_points_ = current_points_;
  }

private:
  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> optical_flow_ =
      cv::cuda::SparsePyrLKOpticalFlow::create();

  cv::cuda::GpuMat previous_gray_;
  cv::cuda::GpuMat previous_points_;
  cv::cuda::GpuMat current_points_;
  cv::cuda::GpuMat status_gpu_;
};
```

这里要注意：

1. 如果每帧特征点很少，GPU 光流可能不快。
2. 如果特征点最终还要回 CPU 做 PnP，download 成本要算进去。
3. 如果整条视觉前端都在 GPU 上，收益才更明显。

### 立体匹配

立体匹配更容易受益于 GPU。
因为每个像素都有视差搜索。

概念流程：

```text
left image -> upload
right image -> upload
rectify if needed
StereoBM or StereoSGM
filter disparity
convert disparity to depth
```

如果深度图后面用于稠密建图，最好继续留在 GPU。
如果只需要少量 3D 点，可以下载稀疏结果。

### ORB 特征

ORB 包括 FAST 角点、方向估计、BRIEF 描述子。
GPU 版本在高分辨率图像和大量特征点时更有价值。

但是 ORB 还有一个工程问题：

> 描述子匹配、地图点管理、关键帧选择通常在 CPU 端。

如果 GPU 只负责提特征，后面立刻下载所有 keypoint 和 descriptor，收益可能被削弱。

一种折中是：

1. GPU 提特征和描述子。
2. GPU 做描述子匹配或候选筛选。
3. CPU 只接收通过筛选的少量匹配。

### OpenCV CUDA 的部署边界

OpenCV 是否包含 CUDA 模块取决于编译选项。
很多系统包里的 OpenCV 并没有启用 CUDA。

CMake 中应明确检查：

```cmake
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

if(WITH_OPENCV_CUDA)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc cudaarithm cudaimgproc cudaoptflow)
  add_compile_definitions(HAS_OPENCV_CUDA=1)
else()
  add_compile_definitions(HAS_OPENCV_CUDA=0)
endif()
```

C++ 中也要做编译期隔离：

```cpp
#if HAS_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaoptflow.hpp>
#endif
```

不要在无 CUDA OpenCV 的平台上包含 CUDA 头文件。
否则 fallback 还没运行，项目就已经编译失败。

> ⚠️ **编程陷阱：公共头文件无条件包含 CUDA 头**
> **错误做法**：在 `registration_backend.hpp` 中写 `#include <cuda_runtime.h>` 和 `cudaStream_t stream_;`。
> **现象**：没有 CUDA 环境的 CI 或嵌入式平台编译失败。
> **根本原因**：公共头文件被所有使用配准接口的代码包含。如果公共头依赖 CUDA 头，整个项目就对 CUDA 产生了硬依赖。
> **正确做法**：公共接口只使用标准 C++ 类型。CUDA 类型只出现在 `.cu` 文件或 CUDA 私有头文件中。通过虚接口或 pimpl 隔离 CUDA 实现细节。

### 练习

1. **[设计题]** 画出一个 SLAM 项目的 CMake 依赖图：`slam_core`（纯 C++）、`slam_cuda`（CUDA 后端）、`slam_app`（可执行程序）。说明 `slam_core` 如何在不链接 CUDA 的情况下定义配准接口，`slam_cuda` 如何实现该接口。
2. **[代码题]** 为 OpenCV CUDA 模块写编译期隔离：当 `HAS_OPENCV_CUDA=1` 时提供 GPU 预处理器，否则提供 CPU fallback。两者共享同一个公共接口。

---

## 38.7 LibTorch C++：学习模块进入 SLAM ⭐⭐⭐

现代 SLAM 越来越多地接入学习模块：

1. SuperPoint 类特征。
2. SuperGlue 类匹配。
3. 深度估计网络。
4. 动态物体分割。
5. 语义地图构建。
6. 3D Gaussian Splatting 重建。

很多研究代码最初用 Python 写。
但真实机器人系统通常需要 C++ 主流程。
LibTorch 的作用是让 C++ 程序直接调用 PyTorch 模型。

### 最小推理流程

```cpp
#include <torch/script.h>
#include <torch/torch.h>

class TorchFeatureExtractor {
public:
  explicit TorchFeatureExtractor(const std::string& model_path) {
    module_ = torch::jit::load(model_path);

    if (torch::cuda::is_available()) {
      device_ = torch::Device(torch::kCUDA);
    } else {
      device_ = torch::Device(torch::kCPU);
    }

    module_.to(device_);
    module_.eval();
  }

  torch::Tensor infer(const cv::Mat& gray) {
    torch::NoGradGuard no_grad;

    if (gray.empty() || gray.type() != CV_8UC1) {
      throw std::invalid_argument("TorchFeatureExtractor expects non-empty CV_8UC1 image");
    }

    // OpenCV 的 ROI 可能不是连续内存；from_blob 默认按紧凑连续布局解释。
    // 这里先把输入规整成连续单通道图像，再 clone 成由 Tensor 自己拥有的内存。
    const cv::Mat contiguous = gray.isContinuous() ? gray : gray.clone();

    torch::Tensor input =
        torch::from_blob(contiguous.data,
                         {1, 1, contiguous.rows, contiguous.cols},
                         torch::TensorOptions().dtype(torch::kUInt8))
            .clone();

    input = input.to(torch::kFloat32).div_(255.0);
    input = input.to(device_);

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input);

    torch::Tensor output = module_.forward(inputs).toTensor();
    return output;
  }

private:
  torch::jit::script::Module module_;
  torch::Device device_ = torch::Device(torch::kCPU);
};
```

这里有三个关键点。

第一，`torch::from_blob` 不拥有 `cv::Mat` 内存。
如果异步使用这个 Tensor，必须确保图像数据仍然有效。

第二，`input.to(device_)` 可能发生 CPU/GPU 拷贝。
这一步要计入时间。

第三，输出 Tensor 如果转回 CPU：

```cpp
torch::Tensor output_cpu = output.to(torch::kCPU);
```

这里也会同步并发生下载。

### 与 SLAM 主流程的关系

学习模块不应该直接控制 SLAM 状态。
更稳的方式是把它作为观测生成器：

```text
image
  -> learned frontend
  -> keypoints / descriptors / masks / depth
  -> geometric estimator
  -> state update
```

也就是说，神经网络提供候选观测。
几何模块仍然负责一致性检查。

例如动态物体分割可以输出 mask。
SLAM 前端用 mask 排除动态区域。
但最终是否接受位姿更新，还要看重投影误差、匹配数量、退化检测。

### GPU 资源竞争

如果同一块 GPU 同时跑：

1. 点云配准。
2. 神经网络推理。
3. 可视化。
4. 局部地图重建。

就会出现资源竞争。

表现为：

1. 平均耗时可接受，但某些帧突然很慢。
2. 网络推理和配准互相抢显存。
3. CUDA stream 太多，实际调度不可控。
4. 可视化窗口让定位延迟波动。

解决方法不是盲目加 stream。
更重要的是调度策略。

一种简单策略：

| 任务 | 优先级 | 处理 |
|------|--------|------|
| 位姿跟踪 | 最高 | 必须按帧运行 |
| IMU 预积分 | 最高 | CPU 或轻量并行 |
| 动态 mask | 中 | 可以降频 |
| 稠密重建 | 低 | 可以跳帧 |
| 可视化 | 低 | 不应阻塞定位 |

代码上可以这样表达：

```cpp
struct GpuTaskBudget {
  double tracking_ms = 20.0;
  double perception_ms = 10.0;
  double mapping_ms = 30.0;
  bool allow_dense_update = true;
};
```

当跟踪耗时接近预算时，降低低优先级任务：

```cpp
if (last_tracking_ms > budget.tracking_ms) {
  budget.allow_dense_update = false;
}
```

学习模块很强，但不能破坏定位闭环。

> ⚠️ **编程陷阱：`torch::from_blob` 不拥有底层内存**
> **错误做法**：`torch::from_blob(mat.data, ...)` 后异步使用 Tensor，但 `mat` 已经被析构或覆盖。
> **现象**：推理结果偶尔出现随机噪声或全零输出。
> **根本原因**：`from_blob` 创建的 Tensor 不拥有内存——它只是一个指向外部缓冲区的视图。如果底层 `cv::Mat` 被释放或修改，Tensor 就指向无效或已变化的数据。
> **正确做法**：在 `from_blob` 后立刻 `.clone()` 让 Tensor 拥有自己的内存副本，或确保 `cv::Mat` 的生命周期覆盖整个推理过程。

> 🧠 **思维陷阱：认为"有 GPU 就应该跑 GPU 版模型"**
> **新手想法**："机器人上有 GPU，所有网络推理都应该走 GPU。"
> **实际上**：GPU 资源是有限的。如果定位配准已经占用 GPU 大部分计算能力，再加一个语义分割网络可能导致两者互相抢占，p99 延迟大幅上升。更好的策略是：定位优先保障，语义等感知模块可以降频运行或用 CPU 小模型替代。
> **正确思维**：GPU 资源是系统级预算，不是单个模块的私有财产。高优先级任务（定位）应该有保障的 GPU 时间片。

### 练习

1. **[设计题]** 一个 SLAM 系统同时需要 GPU 配准和 GPU 语义分割。设计一个 GPU 任务调度方案，保证定位以 10Hz 稳定运行，语义分割在剩余时间内尽量运行。
2. **[分析题]** `torch::from_blob` 和 `torch::Tensor::clone()` 的性能差异是什么？在什么条件下可以安全省略 `clone()`？

---

## 38.8 3D Gaussian Splatting 与 CUDA 渲染链路 ⭐⭐⭐⭐

一些新型视觉 SLAM 或重建系统会把 3D Gaussian Splatting 引入地图表达。
这类系统通常包含：

1. 位姿估计。
2. 高斯地图维护。
3. CUDA rasterizer。
4. photometric loss。
5. 反向传播或在线优化。

3DGS 渲染适合 CUDA 的原因是：

1. 大量 Gaussian 投影到图像。
2. 每个 tile 可以并行处理。
3. 每个像素颜色由多个 splat 累积。
4. forward/backward 都是大规模并行。

但它也会给 SLAM 系统带来新问题：

| 问题 | 说明 |
|------|------|
| 显存占用 | 高斯数量增长后显存压力很大 |
| 延迟波动 | 优化迭代会造成帧间耗时不稳定 |
| 线程边界 | 渲染和跟踪可能竞争 GPU |
| 地图一致性 | 位姿更新会影响渲染监督 |
| 部署复杂度 | C++、CUDA、深度学习框架混合 |

所以工程上常见分离：

```text
tracking thread: real-time pose estimation
mapping thread: lower-frequency map update
rendering module: GPU rendering and loss evaluation
```

这里如果出现 GPU 竞争，tracking 应该优先。
地图质量可以延迟，定位闭环不能长期阻塞。

### 可维护的 CUDA rasterizer 接口

渲染器不应直接暴露一堆裸指针给 SLAM 主程序。
可以把 GPU 资源封装起来：

```cpp
struct CameraParameters {
  Eigen::Matrix3f K;
  int width = 0;
  int height = 0;
};

struct RenderOutput {
  torch::Tensor color;
  torch::Tensor depth;
  torch::Tensor alpha;
};

class GaussianRenderer {
public:
  RenderOutput render(const torch::Tensor& gaussian_state,
                      const Eigen::Matrix4f& T_world_camera,
                      const CameraParameters& camera);
};
```

如果系统不用 LibTorch，也可以用自定义 device buffer。
关键是接口要表达领域对象，而不是泄露底层 kernel 参数。

---

## 38.9 Jetson 与嵌入式部署 ⭐⭐

桌面 GPU 和 Jetson 平台在工程特性上差异很大。

桌面平台通常是：

```text
CPU memory <-> PCIe <-> GPU memory
```

Jetson 这类 SoC 平台通常是共享物理内存架构：

```text
CPU and GPU share system memory
```

这不代表没有成本。
它只是改变了成本结构。

### 共享内存不等于零成本：统一内存架构的物理约束

在共享内存平台上，CPU/GPU 拷贝成本可能低很多。
但仍然要考虑：

1. cache 一致性。
2. 内存带宽。
3. page migration。
4. GPU 和 CPU 抢带宽。
5. 功耗和温度限制。

如果误以为共享内存就是无限快，仍然会写出低效系统。

**统一内存架构（UMA）的物理约束**需要详细理解。在 Jetson 上，CPU 和 GPU 共享同一块 LPDDR 内存。这消除了 PCIe 瓶颈，但引入了新的约束：

**带宽竞争**。Jetson Orin 的 LPDDR5 带宽约 204 GB/s（理论峰值），看起来很高。但这个带宽要同时服务 CPU 核心（最多 12 个 Cortex-A78AE）、GPU（Ampere 架构，2048 CUDA cores）、DLA（深度学习加速器）和多媒体引擎。在满负载下，每个模块分到的有效带宽远低于峰值。如果 SLAM 的 GPU kernel 和 CPU 线程同时密集访问内存，两者都会变慢——这在离散 GPU 系统中不存在（CPU 和 GPU 各有独立的内存系统）。

**缓存一致性开销**。虽然物理内存共享，CPU 和 GPU 的缓存体系是独立的。当 CPU 写入一段数据后 GPU 要读取，必须确保 CPU 缓存中的脏数据已经写回到主存，或者 GPU 的缓存能直接读到 CPU 缓存中的最新值。Jetson 提供了硬件缓存一致性支持（通过 AXI ACE 或 CHI 协议），但一致性维护本身有开销——每次跨 CPU/GPU 的数据共享都可能触发缓存探测（snoop）和无效化。

**功耗与热管理对性能的影响**。这是 Jetson 部署中最容易被忽略的因素。Jetson 有多个功耗模式（如 Orin 的 15W/30W/50W 模式），不同模式下 CPU/GPU 的时钟频率差异巨大。在 15W 模式下，GPU 频率可能被限制在最大值的 50%，性能相应下降。更隐蔽的是热降频（thermal throttling）：即使设置了高功耗模式，如果散热不足（散热器太小、环境温度太高），芯片温度超过阈值后会自动降低频率。在实验室中运行良好的 GPU pipeline，部署到机器人上（密封外壳、高环境温度）后可能因热降频而性能骤降 30-50%。

**Jetson 部署的实践建议**：(1) 固定功耗模式和 GPU/CPU 频率（通过 `jetson_clocks`），避免动态调频带来的性能波动。(2) 在目标散热条件下做 benchmark，不要在开放环境下测试。(3) 监控 GPU/CPU 温度和频率——如果运行中频率下降，说明散热不足。(4) 优先使用 Unified Memory 或 zero-copy 内存映射，减少不必要的数据拷贝。(5) 控制 GPU 和 CPU 的并发负载——避免两者同时密集访问内存。

Jetson 的统一内存和桌面 GPU 的独立显存就像城市内部物流和跨国物流的区别：城市内部运输（Jetson 统一内存）不需要过海关（PCIe 传输），但城市内部道路（内存带宽）仍然有限，高峰期仍然堵车。而且 CPU 和 GPU 共用同一条路（共享带宽），一方大量占用时另一方也会受影响。

如果在 Jetson 上使用桌面 GPU 的优化策略（如大量 pinned memory）会怎样？Jetson 的物理内存通常只有 8-32GB，且被 CPU 和 GPU 共享。大量 pinned memory 会直接减少 CPU 可用内存。在桌面上合理的策略，在 Jetson 上可能导致 OOM。

> ⚠️ **编程陷阱：Jetson 上 Unified Memory 性能不可预测**
> **错误做法**：在 Jetson 上使用 `cudaMallocManaged`，假设"共享内存平台上 Unified Memory 零成本"。
> **现象**：某些访问模式下延迟不稳定，出现数百微秒的尖峰。
> **根本原因**：即使物理内存共享，Unified Memory 仍涉及 page migration、cache coherence 和 TLB 管理。在 CPU 和 GPU 交替频繁访问同一页时，一致性协议开销可能很高。
> **正确做法**：对性能关键路径仍然使用显式内存管理和同步。Unified Memory 适合原型和非实时路径。

### 练习

1. **[部署题]** 列出从桌面 GPU 开发环境迁移到 Jetson 部署环境时需要检查的 5 个关键差异（CUDA 版本、内存架构、功耗模式、驱动绑定、编译架构）。
2. **[分析题]** 在 Jetson 上，CPU 线程和 GPU kernel 同时大量访问内存时，为什么可能互相影响？从内存带宽共享的角度解释。

### JetPack 与 CUDA 版本

Jetson 平台上的 CUDA 版本通常与 JetPack 绑定。
不要像桌面 Linux 那样随意升级 CUDA。

工程建议：

1. 在文档中记录 JetPack 版本。
2. 在 Dockerfile 中固定基础镜像。
3. 在 CMake 输出 CUDA 版本。
4. 在程序启动日志中打印 GPU 名称和驱动信息。
5. 不要把桌面编译产物直接拷到 Jetson。

示例启动日志：

```cpp
#include <cuda_runtime.h>
#include <iostream>

void printCudaDeviceInfo() {
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess) {
    std::cout << "CUDA device count query failed\n";
    return;
  }

  std::cout << "CUDA device count: " << count << "\n";

  for (int i = 0; i < count; ++i) {
    cudaDeviceProp prop{};
    cudaGetDeviceProperties(&prop, i);
    std::cout << "device " << i << ": " << prop.name
              << ", compute capability "
              << prop.major << "." << prop.minor << "\n";
  }
}
```

### 功耗模式

移动机器人不是只看最高性能。
还要看电池、散热和稳定性。

同一个 CUDA 程序在不同功耗模式下可能差异很大。
如果 benchmark 时使用最高功耗，部署时使用低功耗，结论就不可信。

记录 benchmark 时至少写：

1. 设备型号。
2. JetPack 版本。
3. 功耗模式。
4. 温度范围。
5. 是否开启风扇。
6. 输入数据集。
7. 点云或图像规模。
8. 是否包含可视化。

### Docker

Docker 可以降低部署差异。
但 GPU Docker 仍然依赖宿主机驱动和 runtime。

一个项目可以提供：

```text
docker/
  Dockerfile.desktop
  Dockerfile.jetson
  compose.desktop.yaml
  compose.jetson.yaml
```

桌面镜像和 Jetson 镜像不要强行合并。
它们的基础镜像、CUDA 版本和系统库经常不同。

---

## 38.10 CMake：让 CUDA 成为可选能力 ⭐

一个可维护的 SLAM 项目不应该要求所有用户都有 CUDA。
尤其是教学项目，CPU 版本应该始终可运行。

推荐使用选项控制：

```cmake
option(WITH_CUDA "Enable CUDA acceleration" ON)
option(WITH_TORCH "Enable LibTorch modules" OFF)
option(WITH_OPENCV_CUDA "Enable OpenCV CUDA modules" OFF)
```

然后按模块启用：

```cmake
cmake_minimum_required(VERSION 3.22)
project(gpu_slam_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_CUDA "Enable CUDA acceleration" ON)

add_library(slam_core
  src/slam_system.cpp
  src/cpu_registration_backend.cpp
)

target_include_directories(slam_core PUBLIC include)

if(WITH_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  add_library(slam_cuda
    src/gpu_registration_backend.cu
    src/device_cloud.cu
  )

  target_compile_features(slam_cuda PUBLIC cxx_std_17)
  target_link_libraries(slam_cuda PUBLIC slam_core CUDA::cudart)
  target_compile_definitions(slam_cuda PUBLIC SLAM_HAS_CUDA=1)
else()
  target_compile_definitions(slam_core PUBLIC SLAM_HAS_CUDA=0)
endif()
```

更常见的做法是不要让 `slam_core` 反向依赖 `slam_cuda`。
上面的 `slam_cuda` 可以被最终可执行程序或后端工厂链接，但 `slam_core` 不应再反向链接 `slam_cuda`，否则会把可选 GPU 后端变成核心库的强依赖。
可以把 CUDA 后端作为插件式库：

```cmake
add_library(slam_core
  src/slam_system.cpp
  src/registration_backend.cpp
)

if(WITH_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  add_library(slam_cuda_backend MODULE
    src/gpu_registration_backend.cu
    src/gpu_backend_factory.cpp
  )

  target_link_libraries(slam_cuda_backend PRIVATE slam_core CUDA::cudart)
endif()
```

这样 CPU 主程序可以在没有 CUDA 的环境编译。
GPU 后端在运行时存在就加载，不存在就使用 CPU。

### 编译期隔离

头文件也要隔离。
不要在公共头文件里无条件包含 CUDA 头。

不推荐：

```cpp
#include <cuda_runtime.h>

class RegistrationBackend {
  cudaStream_t stream_;
};
```

这样任何包含该头文件的 CPU 代码都需要 CUDA 头。

推荐：

```cpp
class RegistrationBackend {
public:
  virtual ~RegistrationBackend() = default;
  virtual bool align(...) = 0;
};
```

然后在 `.cu` 或 CUDA 私有头里使用 CUDA 类型。

### 架构设置

CUDA 编译需要指定目标架构。
教学项目可以先用 CMake 自动值。
部署项目应明确记录目标平台。

```cmake
if(WITH_CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)

  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75 86 89)
  endif()
endif()
```

对于发布包，不一定适合使用 `native`。
因为构建机器和运行机器可能不同。
这时应根据目标设备设定架构列表。
如果确实希望按构建机自动探测架构，应确认当前 CMake 版本支持 `CMAKE_CUDA_ARCHITECTURES=native`，并在构建日志中记录探测结果。

---

## 38.11 运行时 backend 选择 ⭐⭐

一个机器人系统应允许在配置中选择后端：

```yaml
registration:
  backend: auto
  allow_cpu_fallback: true
  min_points_for_gpu: 5000
  max_points: 120000
  voxel_resolution: 0.4
  max_iterations: 32
```

语义：

| 字段 | 含义 |
|------|------|
| backend | `cpu`、`gpu` 或 `auto` |
| allow_cpu_fallback | GPU 不可用时是否回退 |
| min_points_for_gpu | 点太少时直接用 CPU |
| max_points | 输入裁剪上限 |
| voxel_resolution | 降采样尺度 |
| max_iterations | 配准迭代上限 |

选择逻辑：

```cpp
enum class BackendKind {
  kCpu,
  kGpu,
  kAuto,
  kUnavailable,
};

struct BackendDecision {
  bool ok = true;
  BackendKind selected = BackendKind::kCpu;
  std::string reason;
};

BackendDecision chooseBackend(const RegistrationConfig& config,
                              int num_points,
                              bool cuda_available) {
  if (config.backend == "cpu") {
    return {true, BackendKind::kCpu, "configured CPU backend"};
  }

  if (config.backend == "gpu") {
    if (cuda_available) {
      return {true, BackendKind::kGpu, "configured GPU backend"};
    }
    if (config.allow_cpu_fallback) {
      return {true, BackendKind::kCpu, "configured GPU unavailable, fallback to CPU"};
    }
    return {false, BackendKind::kUnavailable, "configured GPU backend unavailable"};
  }

  if (config.backend != "auto") {
    return {false, BackendKind::kUnavailable, "unknown backend name"};
  }

  if (!cuda_available) {
    if (config.allow_cpu_fallback) {
      return {true, BackendKind::kCpu, "CUDA unavailable, fallback to CPU"};
    }
    return {false, BackendKind::kUnavailable, "CUDA required but unavailable"};
  }

  if (num_points < config.min_points_for_gpu) {
    return {true, BackendKind::kCpu, "point count below GPU threshold"};
  }

  return {true, BackendKind::kGpu, "CUDA backend selected"};
}
```

这个函数看起来简单，但它解决了很多部署问题。

1. 小数据不强行上 GPU。
2. 没有 CUDA 时给出明确原因。
3. 配置和运行环境不匹配时可诊断。
4. 日志中可以记录每次选择。

这里把“CUDA 必须存在但实际不可用”编码成 `ok=false`，而不是假装选择了 GPU。
状态建模要避免把失败状态塞进成功枚举里。
否则后续代码很容易沿着 GPU 路径继续执行，最终在更远的位置崩溃。

### 失败策略

GPU 后端可能运行中失败。
例如显存不足、设备丢失、非法访问。

不能简单忽略。
可以分级处理：

| 错误 | 处理 |
|------|------|
| 初始化失败 | 启动时回退 CPU 或终止 |
| 单帧输入过大 | 裁剪输入或回退 CPU |
| 单帧配准不收敛 | 标记本帧失败 |
| CUDA runtime error | 记录错误，重建后端或停机 |
| 显存不足 | 降低地图窗口或关闭低优先级模块 |

示例：

```cpp
struct RegistrationResult {
  bool ok = false;
  bool used_gpu = false;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  double elapsed_ms = 0.0;
  std::string message;
};
```

不要只返回 `bool`。
SLAM 系统需要知道失败原因。

---

## 38.12 benchmark：不要只测 kernel ⭐⭐

GPU 改造必须用 benchmark 闭环。
否则很容易产生错觉。

### 分层计时

建议每帧记录：

```cpp
struct FrontendTiming {
  double total_ms = 0.0;
  double input_conversion_ms = 0.0;
  double upload_ms = 0.0;
  double preprocess_ms = 0.0;
  double registration_ms = 0.0;
  double solve_ms = 0.0;
  double download_ms = 0.0;
  double publish_ms = 0.0;
};
```

CPU 时间用 `std::chrono`：

```cpp
class CpuTimer {
public:
  void tic() {
    start_ = Clock::now();
  }

  double tocMs() const {
    const auto end = Clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point start_;
};
```

GPU 阶段用 CUDA event：

```cpp
#include <cuda_runtime.h>

#include <stdexcept>
#include <string>

inline void checkCuda(cudaError_t status, const char* context) {
  if (status != cudaSuccess) {
    throw std::runtime_error(
        std::string(context) + ": " + cudaGetErrorString(status));
  }
}

class GpuTimer {
public:
  explicit GpuTimer(cudaStream_t stream) : stream_(stream) {
    checkCuda(cudaEventCreate(&start_), "cudaEventCreate(start)");
    checkCuda(cudaEventCreate(&stop_), "cudaEventCreate(stop)");
  }

  ~GpuTimer() noexcept {
    if (start_ != nullptr) {
      cudaEventDestroy(start_);
    }
    if (stop_ != nullptr) {
      cudaEventDestroy(stop_);
    }
  }

  GpuTimer(const GpuTimer&) = delete;
  GpuTimer& operator=(const GpuTimer&) = delete;
  GpuTimer(GpuTimer&&) = delete;
  GpuTimer& operator=(GpuTimer&&) = delete;

  void tic() {
    checkCuda(cudaEventRecord(start_, stream_), "cudaEventRecord(start)");
  }

  float tocMs() {
    checkCuda(cudaEventRecord(stop_, stream_), "cudaEventRecord(stop)");
    checkCuda(cudaEventSynchronize(stop_), "cudaEventSynchronize(stop)");

    float ms = 0.0f;
    checkCuda(cudaEventElapsedTime(&ms, start_, stop_), "cudaEventElapsedTime");
    return ms;
  }

private:
  cudaStream_t stream_ = nullptr;
  cudaEvent_t start_ = nullptr;
  cudaEvent_t stop_ = nullptr;
};
```

注意：`tocMs()` 会同步。
不要在每个小 kernel 后都调用。
可以按大阶段计时。

### 统计分布

SLAM 不是只看平均值。
至少统计：

1. mean。
2. median。
3. p90。
4. p99。
5. max。
6. lost frames。
7. tracking failure count。

一个简单统计结构：

```cpp
#include <algorithm>
#include <vector>

struct LatencySummary {
  double mean = 0.0;
  double median = 0.0;
  double p90 = 0.0;
  double p99 = 0.0;
  double max = 0.0;
};

LatencySummary summarizeLatency(std::vector<double> values) {
  LatencySummary s;
  if (values.empty()) {
    return s;
  }

  std::sort(values.begin(), values.end());

  double sum = 0.0;
  for (double v : values) {
    sum += v;
  }

  auto percentile = [&](double p) {
    const double index = p * static_cast<double>(values.size() - 1);
    return values[static_cast<std::size_t>(index)];
  };

  s.mean = sum / static_cast<double>(values.size());
  s.median = percentile(0.50);
  s.p90 = percentile(0.90);
  s.p99 = percentile(0.99);
  s.max = values.back();
  return s;
}
```

### 轨迹质量

GPU 版本不能只快。
还要保持定位质量。

至少比较：

| 指标 | 用途 |
|------|------|
| ATE | 全局轨迹误差 |
| RPE | 局部相对误差 |
| tracking lost count | 跟踪丢失次数 |
| accepted frame count | 成功处理帧数 |
| loop closure count | 回环数量 |
| map consistency | 地图是否扭曲 |

如果 GPU 版本因为浮点差异导致极少量帧分支不同，轨迹可能逐渐偏离。
所以应在同一数据集上跑完整序列，而不是只测单帧。

### 可复现实验记录

benchmark 输出建议包含：

```yaml
benchmark:
  dataset: campus_loop_01
  frames: 4200
  platform:
    cpu: example_cpu
    gpu: example_gpu
    memory_gb: 32
  software:
    os: ubuntu
    cuda: recorded_at_runtime
    opencv: recorded_at_runtime
  configuration:
    backend: gpu
    voxel_resolution: 0.4
    max_iterations: 32
  latency_ms:
    mean: 18.4
    median: 16.9
    p90: 27.1
    p99: 48.5
    max: 71.2
  quality:
    ate_rmse_m: 0.12
    tracking_lost: 0
```

数字本身要来自实际实验。
模板中不要写成“官方结果”。

---

## 38.13 正确性验证：GPU 结果不必逐位一致 ⭐⭐

### 为什么 CPU 和 GPU 的浮点结果不可能完全一致？

这个问题的答案不是"GPU 精度更差"——CPU 和 GPU 都遵循 IEEE 754 浮点标准（至少对于 `float` 和 `double` 的基本运算是如此）。不一致的根本原因在于**浮点运算不满足结合律**和**不同执行模型导致的运算顺序差异**。

**原因 1：并行归约改变加法顺序**。考虑对 $N$ 个浮点数求和。CPU 串行版本的顺序是 $((a_1 + a_2) + a_3) + \cdots + a_N$——严格从左到右。GPU 并行归约通常使用树形归约：先把相邻元素两两相加，再把结果两两相加，直到只剩一个值。两种顺序在数学上等价，但因为浮点加法不满足结合律（$(a + b) + c \neq a + (b + c)$，除非所有值相同或为零），结果可能在最后几位有效数字上不同。对于 `float`（约 7 位十进制精度），差异通常在 $10^{-5}$ 到 $10^{-3}$ 量级；对于 `double`（约 15 位精度），差异通常在 $10^{-12}$ 到 $10^{-9}$ 量级。

**原因 2：FMA（Fused Multiply-Add）的差异**。GPU 的 CUDA core 天然支持 FMA 操作——`a * b + c` 在一条指令内完成，中间结果不做舍入。CPU 是否使用 FMA 取决于架构（支持 AVX2+FMA 的 CPU 才有）和编译选项。如果 GPU 使用 FMA 而 CPU 不使用（或反之），即使输入完全相同，$a \times b + c$ 的结果也可能不同——因为是否对中间乘积做舍入会影响最终结果。差异通常是最后一位有效数字（1 ULP）。

**原因 3：`--use_fast_math` 的影响**。CUDA 的 `--use_fast_math` 编译选项会启用若干近似优化：把单精度除法替换为快速倒数乘法（精度约 2 ULP 而非 0.5 ULP）、把 `sinf/cosf` 替换为快速近似版本、启用 denormal flush-to-zero。这些优化可以显著提高吞吐量（某些操作快 2-3 倍），但会引入额外的数值误差。对于 SLAM 中的点云变换和残差计算，这个额外误差通常完全可以接受；但对于需要高精度的后端优化（如 Cholesky 分解），应该谨慎使用。

**原因 4：排序稳定性**。GPU 的 radix sort 通常是稳定排序（相同 key 的元素保持原始顺序），但 Thrust 的某些排序变体不保证稳定性。如果体素滤波中多个点映射到同一体素 key，排序后这些点的顺序可能与 CPU 版本不同——导致归约结果（如体素质心）略有差异。这不是精度问题，而是**顺序不确定性**。

**对 SLAM 的实际影响**：SLAM 配准的最终目标是计算位姿变换，位姿的精度通常受传感器噪声（毫米级）限制，而 CPU-GPU 的浮点差异在微米甚至纳米级——远小于传感器噪声。因此，CPU 和 GPU 版本的 SLAM 轨迹应该在传感器精度范围内一致，但不应期望逐位相同。

所以测试不应该要求完全相等。
而应该使用误差阈值。

### 点云变换测试

```cpp
bool isApprox(const Eigen::Vector3f& a,
              const Eigen::Vector3f& b,
              float eps) {
  return (a - b).norm() <= eps;
}
```

对点云结果：

```cpp
void compareClouds(const std::vector<Eigen::Vector3f>& cpu,
                   const std::vector<Eigen::Vector3f>& gpu) {
  if (cpu.size() != gpu.size()) {
    throw std::runtime_error("cloud size mismatch");
  }

  constexpr float kEps = 1e-4f;

  for (std::size_t i = 0; i < cpu.size(); ++i) {
    if (!isApprox(cpu[i], gpu[i], kEps)) {
      throw std::runtime_error("cloud value mismatch");
    }
  }
}
```

### 归约测试

归约误差会随数据规模变化。
可以比较相对误差：

```cpp
double relativeError(double a, double b) {
  const double denom = std::max(1.0, std::abs(a));
  return std::abs(a - b) / denom;
}
```

对 cost：

```cpp
if (relativeError(cpu_cost, gpu_cost) > 1e-5) {
  throw std::runtime_error("cost mismatch");
}
```

对位姿：

```cpp
double translationError(const Eigen::Isometry3d& a,
                        const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm();
}

double rotationErrorRad(const Eigen::Isometry3d& a,
                        const Eigen::Isometry3d& b) {
  const Eigen::Matrix3d R = a.rotation().transpose() * b.rotation();
  const Eigen::AngleAxisd aa(R);
  return std::abs(aa.angle());
}
```

测试断言：

```cpp
if (translationError(cpu_pose, gpu_pose) > 1e-3) {
  throw std::runtime_error("translation mismatch");
}

if (rotationErrorRad(cpu_pose, gpu_pose) > 1e-4) {
  throw std::runtime_error("rotation mismatch");
}
```

### 退化场景测试

SLAM 配准还要测退化场景：

| 场景 | 目的 |
|------|------|
| 空点云 | 不崩溃 |
| 少量点 | fallback 或返回失败 |
| 共线点 | 检测退化 |
| 共面点 | 不误判完整约束 |
| 大量重复点 | 归约稳定 |
| NaN/Inf 点 | 被过滤 |
| 动态物体占比高 | 鲁棒核或筛选有效 |
| 初值很差 | 收敛失败可诊断 |

GPU 后端最怕的是错误输入触发非法内存访问。
所以输入验证不能省。

---

## 38.14 mini pipeline：GPU 点云前端的完整骨架 ⭐⭐

下面构造一个教学版 LiDAR 前端。
它不绑定具体 SLAM 框架。
目标是展示数据流。

### 数据结构

```cpp
struct LidarFrame {
  double timestamp = 0.0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
};

struct LidarFrontendOutput {
  bool ok = false;
  bool used_gpu = false;
  Eigen::Matrix4f T_world_lidar = Eigen::Matrix4f::Identity();
  FrontendTiming timing;
  std::string message;
};
```

### 前端类

```cpp
class LidarFrontend {
public:
  explicit LidarFrontend(RegistrationConfig config)
      : config_(std::move(config)) {
    backend_ = createRegistrationBackend(config_);
  }

  LidarFrontendOutput process(const LidarFrame& frame) {
    LidarFrontendOutput output;

    CpuTimer total_timer;
    total_timer.tic();

    if (!frame.cloud || frame.cloud->empty()) {
      output.message = "empty input cloud";
      return output;
    }

    pcl::PointCloud<pcl::PointXYZI> filtered;
    preprocess(*frame.cloud, &filtered, &output.timing);

    if (!local_map_) {
      initializeMap(filtered);
      output.ok = true;
      output.message = "map initialized";
      output.timing.total_ms = total_timer.tocMs();
      return output;
    }

    Eigen::Matrix4f T = frame.initial_guess;
    const bool aligned = backend_->align(filtered, *local_map_, frame.initial_guess, &T);

    if (!aligned) {
      output.message = "registration failed";
      output.timing.total_ms = total_timer.tocMs();
      return output;
    }

    updateMap(filtered, T);

    output.ok = true;
    output.T_world_lidar = T;
    output.used_gpu = backend_->usesGpu();
    output.timing.total_ms = total_timer.tocMs();
    return output;
  }

private:
  void preprocess(const pcl::PointCloud<pcl::PointXYZI>& input,
                  pcl::PointCloud<pcl::PointXYZI>* output,
                  FrontendTiming* timing) {
    CpuTimer timer;
    timer.tic();

    // 真实项目中这里包括去畸变、滤波、强度筛选和坐标变换。
    *output = input;

    timing->preprocess_ms = timer.tocMs();
  }

  void initializeMap(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    local_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>(cloud));
  }

  void updateMap(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                 const Eigen::Matrix4f& T_world_lidar) {
    (void)cloud;
    (void)T_world_lidar;
    // 教学骨架不展开地图管理。
    // 真正系统应维护局部地图窗口和体素索引。
  }

  RegistrationConfig config_;
  std::unique_ptr<RegistrationBackend> backend_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_;
};
```

### GPU 化位置

上面的 skeleton 里，最容易 GPU 化的是：

1. `preprocess()`。
2. `backend_->align()`。
3. `updateMap()` 内的体素地图维护。

但不要一次全改。
建议顺序：

1. 先接入 GPU 配准后端。
2. 保持 CPU 预处理和 CPU 地图。
3. 跑完整数据集，确认轨迹一致。
4. 再把预处理搬到 GPU。
5. 最后考虑 GPU 地图缓存。

这样每一步都有可比较的基线。

---

## 38.15 CPU/GPU fallback 的实现细节 ⭐⭐

fallback 不是一句“失败时用 CPU”。
它需要处理状态一致性。

### 无状态 fallback

如果 GPU 后端每帧只读输入、输出位姿，那么 fallback 简单：

```text
GPU align failed -> CPU align same input -> output result
```

### 有状态 fallback

如果 GPU 后端维护了 device map，fallback 就复杂。

可能状态：

1. CPU local map。
2. GPU local map。
3. CPU keyframe list。
4. GPU voxel index。

如果 GPU 更新地图失败，CPU map 是否已经更新？
如果 CPU fallback 成功，GPU map 是否需要同步？

一种稳妥策略：

1. CPU map 是权威状态。
2. GPU map 是加速缓存。
3. GPU map 可随时丢弃重建。
4. 每次关键帧更新先提交 CPU map，再标记 GPU cache dirty。

结构：

```cpp
class LocalMap {
public:
  void addKeyframe(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                   const Eigen::Matrix4f& pose) {
    keyframes_.push_back({cloud, pose});
    gpu_cache_dirty_ = true;
  }

  bool gpuCacheDirty() const {
    return gpu_cache_dirty_;
  }

  void markGpuCacheClean() {
    gpu_cache_dirty_ = false;
  }

private:
  struct Keyframe {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Matrix4f pose;
  };

  std::vector<Keyframe> keyframes_;
  bool gpu_cache_dirty_ = true;
};
```

GPU backend 看到 dirty 后重建缓存：

```cpp
if (map.gpuCacheDirty()) {
  rebuildDeviceMap(map);
  map.markGpuCacheClean();
}
```

如果重建失败，CPU 仍然可以用权威地图继续运行。

这比让 GPU map 成为唯一状态安全得多。

> 💡 **概念误区：认为 "GPU fallback 到 CPU" 只需要捕获异常**
> **新手想法**："GPU 后端抛异常时 catch 住，然后调用 CPU 后端就行。"
> **实际上**：如果 GPU 后端在失败前已经部分修改了状态（如更新了 device map 的一部分），CPU fallback 可能看到不一致的状态。fallback 的正确性取决于"失败时状态是否干净"——这需要在接口设计层面保证，而不是在异常处理层面凑合。
> **正确做法**：把状态修改延迟到确认成功之后（类似数据库的 commit），或让 CPU map 始终是权威状态、GPU map 只是缓存。

### 练习

1. **[设计题]** 设计一个有状态 fallback 方案：CPU map 是权威状态，GPU map 是加速缓存。当 GPU 配准失败时，CPU 后端如何使用权威 map 继续运行？当 GPU 恢复后，如何重建 GPU 缓存？
2. **[分析题]** 无状态 fallback 和有状态 fallback 分别适合什么场景？如果配准后端没有内部状态（每帧只读输入、输出位姿），fallback 设计会简单多少？

---

## 38.16 异步 pipeline：让 GPU 和 CPU 同时工作 ⭐⭐⭐

### 异步 pipeline 的理论基础：为什么 CPU-GPU 重叠能提高吞吐

GPU 和 CPU 是两个独立的处理单元，加上 DMA 引擎（负责数据传输），一个系统中实际上有**三个并行执行单元**。如果这三者在任何时刻只有一个在工作，系统利用率最多 33%。异步 pipeline 的目标就是让这三个单元尽可能同时工作。

**形式化分析**。设一帧的处理分为三个阶段：上传（DMA，耗时 $T_u$）、计算（GPU，耗时 $T_c$）、下载+CPU 后处理（耗时 $T_d$）。

同步执行：每帧总耗时 $T_{\text{sync}} = T_u + T_c + T_d$，$N$ 帧总耗时 $N \cdot (T_u + T_c + T_d)$。

流水线执行：稳态下，三个阶段分别处理不同帧——DMA 上传帧 $k+1$，GPU 计算帧 $k$，CPU 后处理帧 $k-1$。稳态吞吐受最慢阶段限制：$T_{\text{pipeline}} = \max(T_u, T_c, T_d)$。$N$ 帧总耗时约 $T_u + T_c + T_d + (N-1) \cdot \max(T_u, T_c, T_d)$。

加速比 $= \frac{T_u + T_c + T_d}{\max(T_u, T_c, T_d)}$。如果三个阶段耗时相近，加速比接近 3。如果一个阶段远长于其他（比如 GPU 计算占 80%），加速比接近 $1.25$——即使流水线化也受最慢阶段限制。

**SLAM 的特殊约束：帧间数据依赖**。SLAM 不是无状态的批处理——帧 $k$ 的配准结果（位姿）是帧 $k+1$ 的初始猜测。这个依赖关系限制了流水线深度：你不能在帧 $k$ 的 GPU 配准完成之前开始帧 $k+1$ 的配准，因为不知道初始猜测。但你**可以**在帧 $k$ 配准期间同时做帧 $k+1$ 的去畸变、滤波和 KNN——这些阶段不依赖帧 $k$ 的配准结果。所以 SLAM 的 GPU 流水线通常是**部分重叠**，而非完全流水线。

SLAM 每帧有多个阶段。
如果设计得好，CPU 和 GPU 可以重叠工作。

例如：

```text
frame k:
  GPU runs registration

CPU simultaneously:
  reads frame k+1
  parses IMU
  predicts initial guess
  handles publish from frame k-1
```

这需要队列和明确状态。

### 双缓冲输入

```cpp
template <typename T>
class DoubleBuffer {
public:
  T& writeBuffer() {
    return buffers_[write_index_];
  }

  const T& readBuffer() const {
    return buffers_[1 - write_index_];
  }

  void swap() {
    write_index_ = 1 - write_index_;
  }

private:
  std::array<T, 2> buffers_;
  int write_index_ = 0;
};
```

注意：这只是单线程示意。
如果跨线程使用，还需要同步协议。

### 异步提交

```cpp
class AsyncGpuRegistration {
public:
  AsyncGpuRegistration() {
    checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
    try {
      checkCuda(cudaEventCreate(&done_), "cudaEventCreate(done)");
    } catch (...) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
      throw;
    }
  }

  ~AsyncGpuRegistration() noexcept {
    if (done_ != nullptr) {
      cudaEventDestroy(done_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  AsyncGpuRegistration(const AsyncGpuRegistration&) = delete;
  AsyncGpuRegistration& operator=(const AsyncGpuRegistration&) = delete;
  AsyncGpuRegistration(AsyncGpuRegistration&&) = delete;
  AsyncGpuRegistration& operator=(AsyncGpuRegistration&&) = delete;

  void submit(const DeviceCloud& source,
              const DeviceMap& target,
              const Eigen::Matrix4f& initial_guess) {
    launchRegistrationKernels(source, target, initial_guess, stream_);
    checkCuda(cudaEventRecord(done_, stream_), "cudaEventRecord(done)");
    pending_ = true;
  }

  bool ready() const {
    if (!pending_) {
      return false;
    }

    const cudaError_t status = cudaEventQuery(done_);
    if (status == cudaSuccess) {
      return true;
    }
    if (status == cudaErrorNotReady) {
      return false;
    }
    throw std::runtime_error(
        std::string("cudaEventQuery(done): ") + cudaGetErrorString(status));
  }

  RegistrationResult collect() {
    checkCuda(cudaEventSynchronize(done_), "cudaEventSynchronize(done)");
    pending_ = false;
    return downloadSmallResult();
  }

private:
  cudaStream_t stream_ = nullptr;
  cudaEvent_t done_ = nullptr;
  bool pending_ = false;
};
```

这种模式适合把 GPU 配准放到异步前端。
但它也引入延迟。
如果下一帧初值依赖当前帧结果，就不能无限异步。

SLAM pipeline 的异步边界必须服从算法依赖。

---

## 38.17 内存池与临时缓冲区 ⭐⭐

### GPU 显存分配的特殊性：为什么 `cudaMalloc` 比 CPU `malloc` 更危险

GPU 显存分配比 CPU 内存分配有更严重的延迟和同步问题。理解这些差异是设计 GPU 内存策略的基础。

**`cudaMalloc` 是同步操作**。不同于 CPU 的 `malloc`（在用户态缓存命中时几乎是纯用户态操作），`cudaMalloc` 必须通过 CUDA 驱动程序与 GPU 硬件通信：分配请求被发送到 GPU 的内存管理单元（MMU），MMU 在显存中找到合适的空闲区域，更新页表映射，然后把结果返回给 CPU 侧。这个过程涉及 CPU-GPU 间的命令发送和确认，耗时通常在 **50-500 微秒**——比 CPU `malloc` 的快速路径（20-50 纳秒）慢 1000 倍以上。

更严重的是，`cudaMalloc` 会隐式同步——它会等待当前 GPU 上的所有待完成操作结束后再执行分配。这意味着如果你在 GPU pipeline 中间调用 `cudaMalloc`，前面提交的所有异步 kernel 和传输都会被强制完成，GPU pipeline 被打断。这就是为什么实时 GPU 路径中**绝对不能**在运行时调用 `cudaMalloc`。

**`cudaFree` 的代价更大**。释放显存不仅需要和 GPU 通信，还可能触发 GPU 的内存碎片整理。一次 `cudaFree` 的耗时可能达到毫秒级——对于 100ms 的帧预算来说，这是不可接受的。

**GPU 内存池的设计原则**和 CPU 的 pmr 策略（内存分配策略与pmr）完全一致：在初始化阶段一次性分配足够的显存，运行时只在这块预分配的显存中管理临时缓冲区。CUDA 11.2 引入的 `cudaMemPool` API 提供了内置的异步内存池支持（通过 `cudaMallocAsync`/`cudaFreeAsync`），它在 stream 上下文中管理显存分配，避免了同步开销。但对于本课程的教学项目，手动管理预分配的 workspace 更清晰也更可控。

GPU SLAM 中反复分配显存会造成抖动。
临时 buffer 应该复用。

典型临时数据：

1. 当前帧 device cloud。
2. 过滤 mask。
3. 体素 key。
4. 排序索引。
5. 残差数组。
6. 每点雅可比。
7. 归约 scratch。

可以用 workspace 管理：

```cpp
#include <cstdint>
#include <thrust/device_vector.h>

class GpuFrontendWorkspace {
public:
  void reserve(std::size_t max_points) {
    points_.resize(max_points);
    filtered_points_.resize(max_points);
    voxel_keys_.resize(max_points);
    indices_.resize(max_points);
    residuals_.resize(max_points);
  }

  DevicePoint* points() {
    // 指针只在下一次 resize/reserve 之前有效；调用方不能长期保存。
    // 当前有效点数由上层 pipeline 单独传入，workspace 只管理容量。
    return thrust::raw_pointer_cast(points_.data());
  }

  std::uint64_t* voxelKeys() {
    // 体素/Morton key 可能超过 32 bit，使用 uint64_t 避免大地图溢出。
    return thrust::raw_pointer_cast(voxel_keys_.data());
  }

private:
  thrust::device_vector<DevicePoint> points_;
  thrust::device_vector<DevicePoint> filtered_points_;
  thrust::device_vector<std::uint64_t> voxel_keys_;
  thrust::device_vector<int> indices_;
  thrust::device_vector<float> residuals_;
};
```

输入超过容量时有两种策略：

1. 扩容。
2. 降采样或裁剪。

实时系统中，扩容可能造成延迟尖峰。
所以可以设置硬上限：

```cpp
if (num_points > max_points_) {
  downsampleToLimit(&cloud, max_points_);
}
```

这不是为了偷懒。
这是为了保证实时边界。

GPU 工作区的复用和 内存分配策略与pmr 的帧级 arena 是完全同构的设计：都是"初始化时分配，运行时只改变有效大小"。区别只是资源位置从 host memory 变成 device memory。这再次证明了一个通用原则：**热路径中的临时对象必须有明确所有者和预分配容量，无论资源在 CPU 还是 GPU 上。**

> ⚠️ **编程陷阱：每帧重新创建 `thrust::device_vector` 导致 GPU 分配抖动**
> **错误做法**：每帧函数内创建多个 `thrust::device_vector`，函数结束后析构释放显存。
> **现象**：运行数分钟后 GPU 延迟出现周期性尖峰。
> **根本原因**：`device_vector` 的构造/析构触发 `cudaMalloc/cudaFree`，这些操作有锁和系统调用开销。长期运行后显存碎片化加剧问题。
> **正确做法**：把 `device_vector` 作为长期对象放在 workspace 中，每帧只用 `resize`（不缩小时不重分配）和 `assign`。

### 练习

1. **[代码题]** 实现一个 `GpuFrontendWorkspace`，预分配 5 个 `device_vector`（points, filtered, keys, indices, residuals），提供 `reserve(max_points)` 方法。写测试验证多次帧处理后不会触发新的显存分配。
2. **[设计题]** 如果输入点数偶尔超过预分配容量（如突然进入密集区域），应该选择扩容、降采样还是裁剪？分析各方案对实时性和定位精度的影响。

---

## 38.18 数据精度：float、double 与可重复性 ⭐⭐

GPU 上 float 通常更快。
SLAM 中点云坐标和图像计算也常用 float。
但后端优化和状态估计经常使用 double。

常见分工：

| 数据 | 建议 |
|------|------|
| 原始点云 | float |
| 图像 | uint8 / float |
| 网络输入 | float16 / float32 |
| 残差计算 | float 或 double，按精度需求 |
| 6x6 Hessian | double 更稳 |
| 位姿状态 | double |

一个常见混合流程：

```text
GPU float residuals
GPU reduction to double or compensated float
CPU double solve
CPU double pose update
```

如果只用 float，长序列轨迹可能出现微小差异。
如果全部 double，GPU 速度可能明显下降。

所以要按误差预算选择。

如果整条 SLAM 管线全部使用 `float` 会怎样？对于单帧配准，`float` 的精度通常足够。但位姿是累积计算的：每帧的小误差会累积到长序列轨迹中。如果初始位姿在 `(100.0, 200.0, 50.0)` 附近，`float` 只有约 6-7 位有效数字，即约 0.01m 的分辨率。经过 10000 帧的累积，漂移误差可能比 `double` 版本大一个数量级。这就是为什么后端求解和位姿状态通常坚持使用 `double`。

> ⚠️ **编程陷阱：GPU float 归约精度不足导致优化器不收敛**
> **错误做法**：在 GPU 上用 `float` 累加 50K 个残差的 Hessian 矩阵。
> **现象**：优化器迭代次数比 CPU 版本多 2-3 倍，或者直接报告"not converged"。
> **根本原因**：大量 `float` 累加的舍入误差导致 Hessian 矩阵的精度下降，条件数变差，线性求解器需要更多迭代或无法收敛。
> **正确做法**：GPU 单点残差可以用 `float`，但归约到 Hessian/gradient 时使用 `double` 累加。只下载一个 `double` 精度的 6x6 矩阵和 6x1 向量。

### 练习

1. **[实验题]** 用 `float` 和 `double` 分别累加 100K 个大小在 $[0.1, 100]$ 范围内的随机数。比较两者与高精度参考值的相对误差。解释为什么误差不成比例增长。
2. **[设计题]** 为一个 GPU ICP 管线设计混合精度方案：哪些阶段用 `float`？哪些阶段用 `double`？精度切换点在哪里？

### 累加误差

并行归约的误差与顺序相关。
对大量残差求和时，可以：

1. 使用 double 累加。
2. 分块求和再合并。
3. 做固定顺序归约。
4. 对测试使用容忍阈值。

示例：

```cpp
struct HessianBlock {
  double data[36];
  double b[6];
  double cost;
};
```

即使单点残差用 float，每个 block 的归约结果也可以转 double。

---

## 🔧 故障排查手册

| 现象 | 可能原因 | 检查方法 | 修复方向 |
|------|----------|----------|----------|
| GPU 版本平均快但机器人卡顿 | p99 延迟高 | 统计延迟分布 | 限制输入规模，降低低优先级 GPU 任务 |
| 首帧延迟很大 | CUDA 初始化 | 分离首帧计时 | 启动阶段预热 |
| 小场景 GPU 更慢 | 数据规模太小 | 按点数分桶统计 | 设置 CPU 阈值 |
| 显存持续增长 | 地图缓存不释放 | 记录显存 | 局部地图裁剪 |
| 偶发非法访问 | device 指针失效 | CUDA 检查工具 | 用 RAII 封装 buffer |
| 轨迹与 CPU 版本分叉 | 浮点差异影响分支 | 跑完整序列 | 调整阈值和排序稳定性 |
| Jetson 上速度低 | 功耗模式限制 | 记录频率温度 | 固定功耗和散热 |
| 编译在一台机器成功另一台失败 | CUDA/OpenCV 版本不同 | 输出依赖版本 | 固定镜像和 CMake 检查 |
| 没有 CUDA 时无法编译 | 头文件隔离失败 | CPU-only build | 公共头移除 CUDA 类型 |
| 可视化影响定位 | GPU 资源竞争 | 关闭可视化对比 | 可视化降频或单独进程 |

---

## 38.20 设计检查清单 ⭐⭐

### GPU 加速改造的系统工程方法论

在给出检查清单之前，有必要讨论 GPU 加速改造的**方法论**——为什么很多 SLAM 项目的 GPU 改造失败了？不是因为 CUDA 代码写错了，而是因为改造顺序和系统设计出了问题。

**失败模式 1：自底向上改造**。最常见的失败模式是"先写 kernel，再找地方插入"。开发者先实现一个 GPU 体素滤波 kernel，测试单独运行很快，然后试图把它塞进已有的 SLAM pipeline——发现需要在 kernel 前后加大量数据转换代码，需要修改数据结构的生命周期，需要处理 GPU 初始化失败的情况……最终工程复杂度远超预期，而端到端加速可能只有 5-10%。

**正确方法：自顶向下设计**。GPU 改造应该从**系统级数据流分析**开始：

1. **Profiling 先行**。用 `perf`、`gprof` 或 `Tracy` 测量完整系统的耗时分布，找到真正的热点。不要凭直觉——很多"看起来该优化的地方"实际上不是瓶颈。
2. **画出数据流图**。标注每个阶段的输入/输出、数据量、耗时。找到最长的连续数据并行链路——这是 GPU 改造收益最大的目标。
3. **确定 CPU/GPU 边界**。选择让尽可能多的连续阶段留在 GPU 上的边界。理想的边界是"上传原始数据，下载最终小结果"。
4. **设计 fallback 策略**。GPU 可能不可用（没有 CUDA）、可能失败（显存不足）、可能太慢（数据太少）。每种情况都需要优雅地回退到 CPU 路径。
5. **实现和验证**。实现 GPU 路径，用 CPU 结果做正确性参照（允许浮点误差），用端到端 benchmark（不是单 kernel benchmark）验证加速。

**失败模式 2：忽视部署多样性**。SLAM 系统需要部署在不同平台——桌面 GPU（PCIe，大显存）、Jetson（共享内存，小显存）、无 GPU 的嵌入式平台、CI/CD 环境（通常无 GPU）。如果 GPU 代码和系统核心耦合太紧，每个平台都需要单独维护——维护成本指数增长。正确的做法是通过接口隔离（CUDA在SLAM中的应用.11 的运行时 backend 选择）让 GPU 成为**可选能力**，而不是必要依赖。

开始改造前，先问：

1. 当前瓶颈是否经过测量？
2. 热点是否足够大？
3. 该热点输入是否已经在 GPU 上？
4. 输出是否必须回 CPU？
5. 能否把多个阶段串成 GPU pipeline？
6. CPU 版本是否有清晰接口？
7. GPU 后端失败时如何回退？
8. 状态权威版本在 CPU 还是 GPU？
9. benchmark 是否包含 p99 和 max？
10. 是否跑过完整数据集？
11. 是否比较了轨迹误差？
12. 是否记录了平台、功耗和版本？
13. 是否有 CPU-only 编译路径？
14. 是否有输入规模上限？
15. 是否有显存上限？
16. 是否有启动预热？
17. 是否把低优先级 GPU 任务与定位闭环隔离？
18. 是否避免在公共头中暴露 CUDA 类型？
19. 是否有退化场景测试？
20. 是否给浮点差异设置合理容忍？

如果这些问题没有答案，先补系统设计，再写 CUDA。

---

## 38.21 实战练习：给 SLAM 前端加可选 GPU 配准

目标：

> 在不改变 SLAM 主循环语义的前提下，为配准模块增加 CPU/GPU 可选后端。

### Step 1：定义统一接口

```cpp
class RegistrationBackend {
public:
  virtual ~RegistrationBackend() = default;

  virtual RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZI>& source,
      const pcl::PointCloud<pcl::PointXYZI>& target,
      const Eigen::Matrix4f& initial_guess) = 0;
};
```

### Step 2：实现 CPU 基线

```cpp
class CpuBackend final : public RegistrationBackend {
public:
  RegistrationResult align(const pcl::PointCloud<pcl::PointXYZI>& source,
                           const pcl::PointCloud<pcl::PointXYZI>& target,
                           const Eigen::Matrix4f& initial_guess) override {
    RegistrationResult result;
    result.used_gpu = false;

    CpuTimer timer;
    timer.tic();

    (void)source;
    (void)target;

    result.T = initial_guess;
    result.ok = true;
    result.elapsed_ms = timer.tocMs();
    return result;
  }
};
```

### Step 3：实现 GPU 后端

```cpp
class GpuBackend final : public RegistrationBackend {
public:
  RegistrationResult align(const pcl::PointCloud<pcl::PointXYZI>& source,
                           const pcl::PointCloud<pcl::PointXYZI>& target,
                           const Eigen::Matrix4f& initial_guess) override {
    RegistrationResult result;
    result.used_gpu = true;

    CpuTimer timer;
    timer.tic();

    if (!cudaReady()) {
      result.ok = false;
      result.message = "CUDA unavailable";
      return result;
    }

    (void)source;
    (void)target;

    result.T = initial_guess;
    result.ok = true;
    result.elapsed_ms = timer.tocMs();
    return result;
  }

private:
  bool cudaReady() const {
    int count = 0;
    return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
  }
};
```

### Step 4：加入 fallback

fallback 不能对所有失败一视同仁。
下面假设 `RegistrationResult` 除了 `ok`、`message` 以外，还携带 `failure` 和 `state_mutated` 两个字段，用来区分失败原因和后端状态是否已经被部分更新。

```cpp
enum class RegistrationFailure {
  kNone,
  kCudaUnavailable,
  kOutOfMemory,
  kNotConverged,
  kInvalidInput,
  kBackendStateDirty,
};

RegistrationResult alignWithFallback(
    RegistrationBackend& primary,
    RegistrationBackend& fallback,
    const pcl::PointCloud<pcl::PointXYZI>& source,
    const pcl::PointCloud<pcl::PointXYZI>& target,
    const Eigen::Matrix4f& initial_guess) {
  RegistrationResult result = primary.align(source, target, initial_guess);

  if (result.ok) {
    return result;
  }

  const bool fallback_is_safe =
      result.failure == RegistrationFailure::kCudaUnavailable ||
      result.failure == RegistrationFailure::kOutOfMemory;

  if (!fallback_is_safe || result.state_mutated) {
    // 状态一致性边界：
    // 输入非法、配准不收敛或后端已经部分更新状态时，不能盲目换 CPU 再跑一次。
    return result;
  }

  RegistrationResult fallback_result =
      fallback.align(source, target, initial_guess);

  if (fallback_result.ok) {
    fallback_result.message = "primary failed, fallback succeeded";
  }

  return fallback_result;
}
```

### Step 5：记录 benchmark

```cpp
void logRegistrationResult(const RegistrationResult& result) {
  std::cout << "registration ok=" << result.ok
            << " used_gpu=" << result.used_gpu
            << " elapsed_ms=" << result.elapsed_ms
            << " message=" << result.message
            << "\n";
}
```

练习要求：

1. CPU-only 编译必须通过。
2. GPU 编译必须通过。
3. 没有 GPU 的机器上程序能运行 CPU 路径。
4. 同一数据集记录 CPU/GPU 延迟分布。
5. 输出轨迹误差对比。

---

## 38.22 本章小结

CUDA 在 SLAM 中最有价值的位置，不是孤立的小函数。
而是连续的数据并行链路。

本章的核心判断：

1. GPU 加速必须从系统瓶颈出发。
2. upload、download 和同步点经常决定真实收益。
3. 点云配准、视觉前端、深度估计、残差计算、神经网络推理是常见热点。
4. 小规模控制逻辑和状态机不适合为了 CUDA 而 CUDA。
5. 现成库优先于从零写 kernel。
6. CUDA 后端应通过统一接口接入。
7. CPU fallback 是工程能力，不是附加功能。
8. CPU map 常适合作为权威状态，GPU map 作为加速缓存。
9. benchmark 要看端到端延迟、p99、max 和轨迹质量。
10. Jetson 部署要记录功耗、温度、JetPack 和 CUDA 版本。
11. 公共头文件不要泄露 CUDA 细节。
12. GPU 结果不必逐位一致，但必须在误差预算内。

如果只记住一句话：

> 在 SLAM 中，GPU 加速的目标不是让某个函数更快，而是让实时定位链路在完整数据集上更稳定、更快、更可部署。

下一章会进入多传感器 SLAM 架构。
那里关注的不再是单个计算阶段，而是 LiDAR、相机、IMU、轮速计、地图和后端优化之间如何组织成可维护的系统。

---

## 延伸阅读

1. **Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA 2021** ⭐⭐——VGICP 的原始论文，解释了体素化协方差估计如何让 GICP 适合 GPU 并行。
2. **NVIDIA CUDA 12.x Programming Guide: CUDA Graphs 章节** ⭐⭐⭐——CUDA Graphs 通过预录制 kernel 调度序列来消除 launch overhead，对固定流程（如每帧相同的 pipeline）可以进一步降低延迟。
3. **NVIDIA Jetson 官方文档与 JetPack SDK** ⭐⭐——嵌入式 GPU 部署的权威参考，包含功耗管理、温度限制和共享内存架构的工程建议。
4. **OpenCV CUDA 模块文档** ⭐⭐——`cv::cuda::GpuMat`、`cv::cuda::Stream`、CUDA 加速的特征检测和光流算法。
5. **LibTorch C++ API 文档** ⭐⭐⭐——在 C++ SLAM 系统中集成 PyTorch 推理模型，包括 `torch::jit::load`、`torch::Tensor` 设备管理和推理优化。
6. **Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering", SIGGRAPH 2023** ⭐⭐⭐⭐——3DGS 的原始论文，CUDA 渲染管线的设计对理解 GPU 在新一代 SLAM 中的角色有重要参考价值。
7. **small_gicp、fast_gicp 等开源 GPU 配准库源码** ⭐⭐——实际 GPU SLAM 项目中配准接口设计、workspace 管理和 fallback 策略的工程参考。
