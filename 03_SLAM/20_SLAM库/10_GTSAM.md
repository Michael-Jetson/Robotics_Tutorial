# 第25章：GTSAM——因子图与增量优化

> **定位**：掌握 GTSAM 因子图建模和 iSAM2 增量优化，理解现代 SLAM 后端的核心数据结构与算法。
> **前置依赖**：通用库·Eigen Eigen 深入、通用库·李群manif 李群与 manif 库、通用库·Ceres Ceres Solver。
> **配套项目**：Mini-LIO（本章新增因子图后端模块）。

---

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回前置章节复习）

1. **[通用库·李群manif]** 李群 $SE(3)$ 的切空间维度是多少？`Exp` 映射和 `Log` 映射分别做什么？
2. **[通用库·李群manif]** manif 库中 `Pose3d::rplus(tangent)` 执行的是左扰动还是右扰动？写出数学表达式。
3. **[通用库·Ceres]** Ceres Solver 中 `CostFunction` 的 `Evaluate()` 方法返回什么？Jacobian 矩阵的维度如何确定？
4. **[通用库·Ceres]** 什么是鲁棒核函数（Robust Kernel）？它如何抑制外点对优化的影响？
5. **[通用库·Eigen]** Eigen 中固定大小矩阵和动态大小矩阵的内存分配方式有何区别？为什么在 SLAM 中偏好固定大小？

---

## 本章目标

学完本章后，你将能够：

1. **理解因子图的数学结构**：区分因子图、马尔可夫随机场和贝叶斯网络，理解变量消元、fill-in 和边缘化的概念
2. **使用 GTSAM 核心 API 构建 SLAM 问题**：用 `NonlinearFactorGraph` + `Values` 建模，用 `PriorFactor`、`BetweenFactor`、`GPSFactor` 等添加约束，选择合适的噪声模型和优化器
3. **掌握 iSAM2 增量优化**：理解 Bayes Tree 数据结构、增量变量消元、重线性化阈值机制，实现实时 SLAM 后端
4. **理解 IMU 预积分在因子图中的角色**：使用 `ImuFactor` 融合惯性测量
5. **编写自定义因子**：继承 `NoiseModelFactorN` 实现 `evaluateError()`，或用 Expression API 自动微分
6. **精读 LIO-SAM 代码**：理解 `mapOptimization.cpp` 中因子图的完整构建流程
7. **查询边缘协方差并诊断退化**：用 `Marginals`/`ISAM2::marginalCovariance` 评估估计的不确定性，定位 `IndeterminantLinearSystemException`

---

## 本章知识导航

在动手之前，先建立一张"全景地图"。GTSAM 的知识体系不是一堆零散 API，而是一条从**概率建模**到**增量推理**再到**工程落地**的主干，本章按这条主干组织：

```text
                       ┌─────────────────────────────────────────┐
                       │      问题：如何高效求解大规模 SLAM 后端？      │
                       └─────────────────────────────────────────┘
                                          │
          ┌───────────────────────────────┼───────────────────────────────┐
          │                               │                               │
   【概率建模层】                    【推理算法层】                    【工程落地层】
          │                               │                               │
  25.1 因子图基础                  25.3 iSAM2 增量优化              25.6 与其他库集成
   · 二部图 / MAP                   · Bayes Tree                    · manif / PCL / Ceres
   · 消元与 fill-in                 · 增量消元                       25.7 LIO-SAM 精读
   · 边缘化（理论）                  · 流式重线性化                    · 真实工业级因子图
          │                               │                          25.8 工程边界与验证
  25.2 核心 API                    25.4 IMU 预积分                  25.9 边缘协方差 Marginals
   · Key/Symbol/Values             · 预积分理论                      · 不确定性查询
   · 因子 / 噪声模型                 · ImuFactor                      · 退化诊断
   · 优化器                         25.5 自定义因子                   25.10 鲁棒优化与 GNC
                                    · evaluateError / 雅可比          · 鲁棒核 / 外点拒绝
```

**三层主干之间的递进关系**：

| 层 | 回答的问题 | 本质 | 一旦缺失会怎样 |
|----|-----------|------|---------------|
| 概率建模层（25.1-25.2） | SLAM 问题"长什么样"？ | 把联合分布写成因子乘积 | 不知道残差/噪声的概率含义，调参全靠玄学 |
| 推理算法层（25.3-25.5） | 怎么"高效求解"？ | 利用图的稀疏性做增量消元 | 每帧从头优化，无法实时 |
| 工程落地层（25.6-25.10） | 真实系统怎么"用对"？ | 身份管理、库适配、不确定性量化 | 因子图能跑但概率解释失效，错误被 Bayes Tree 缓存 |

**推荐阅读路径**：

- **速查（已用过 GTSAM，查某个 API）**：直接看 25.2 + 章末「API 速查表」。
- **核心路径（第一次系统学）**：25.1 → 25.2 → 25.3 → 25.5 → 25.8，先打通"建模—增量—自定义—边界"主干，IMU 与 LIO-SAM 按需补。
- **完整精读（要做 LIO/VIO 后端）**：全章顺序读，重点啃 25.3（Bayes Tree）、25.4（预积分）、25.7（LIO-SAM）、25.9（Marginals）。

### 前置知识桥接

本章站在三块前置内容之上，这里用 2-3 行各激活一次核心要点，让你不翻回去也能跟上：

- **回顾 通用库·李群manif**：$SE(3)$ 是 6 维李群，`Exp`/`Log` 在李代数 $\mathfrak{se}(3) \cong \mathbb{R}^6$ 与流形之间往返；右加 `rplus` 定义为 $T \oplus \xi = T \cdot \text{Exp}(\xi)$。**本章复用**：GTSAM 的 `Pose3::retract` 用的就是这个右扰动，所有位姿因子的雅可比都是"对右扰动 $\xi$ 求导"。
- **回顾 通用库·Ceres**：Ceres 把优化写成 `CostFunction` 列表，`Evaluate()` 返回残差与雅可比，雅可比维度 = 残差维度 × 参数块切空间维度。**本章对照**：GTSAM 的 `evaluateError()` 扮演同样角色，但额外把"图拓扑"显式编码进 `NonlinearFactorGraph`，从而支持增量更新。
- **回顾 通用库·Eigen**：固定大小矩阵（`Matrix6d`）栈分配、可 SIMD，动态大小（`MatrixXd`）堆分配。**本章复用**：GTSAM 内部用 Eigen 表示所有雅可比与信息矩阵，因子残差维度固定时务必用固定大小矩阵（如 `Matrix16`、`Matrix66`）。

### 如果跳过本章会怎样

- **场景一（实时性崩溃）**：你用 Ceres 实现激光 SLAM 后端，每来一帧就重建 `Problem` 全量优化。前 500 帧还能跑，到 3000 帧时单帧耗时涨到 300ms，建图进程被 LiDAR 队列阻塞、点云大面积丢帧——因为你不知道 iSAM2 只需更新 Bayes Tree 的受影响子树。
- **场景二（回环后地图撕裂）**：你照着 LIO-SAM 抄了因子图代码，回环检测触发后位姿整体扭曲、墙面错位。你以为是回环检测错了，反复调阈值无果——其实是回环噪声设得过小 + 回环后只 `update()` 一次，修正没传播开。不理解 25.3 和 25.9，你无法判断问题出在建模、推理还是不确定性。

### 预计阅读时间

| 档位 | 时间 | 覆盖范围 |
|------|------|----------|
| 速查 | 20-30 分钟 | 25.2 API + 章末速查表，定位具体写法 |
| 速读 | 2-3 小时 | 25.1-25.3 主干 + 各节陷阱，建立整体认知 |
| 精读 | 8-12 小时 | 全章 + 手推雅可比 + 跑通累积项目 + LIO-SAM 源码对照 |

---

## 25.1 因子图基础 ⭐⭐

### 这一节解决什么问题

SLAM 后端需要同时优化数百甚至数千个位姿和路标变量。通用库·Ceres 中我们学了 Ceres Solver，它把优化问题表示为"代价函数的集合"——但 Ceres 不关心代价函数之间的**图结构**。因子图（Factor Graph）是一种**显式编码变量之间依赖关系的图模型**，它让我们可以利用图的稀疏结构进行高效的推理和增量更新。

### 动机：为什么需要因子图？

想象一个机器人在走廊中导航，它每走一步就产生一个位姿估计（里程计），每看到一个路标就产生一个观测。经过 1000 步后，我们有：

- 1000 个位姿变量 $x_0, x_1, \ldots, x_{999}$
- 若干路标变量 $l_0, l_1, \ldots, l_M$
- 999 个里程计约束（相邻位姿之间）
- 数千个观测约束（位姿到路标）
- 偶尔的回环约束（非相邻位姿之间）

如果我们把这个问题交给 Ceres，Ceres 会构建一个大的 Hessian 矩阵 $H \in \mathbb{R}^{6000 \times 6000}$（假设每个位姿 6 自由度），然后用稀疏矩阵分解器求解。**但 Ceres 每次都从头求解**——即使只新增了一个位姿和几个观测。

因子图的价值在于：

| 特性 | Ceres（无图结构） | GTSAM（因子图） |
|------|-------------------|-----------------|
| 问题表示 | 代价函数列表 | 因子图（显式图拓扑） |
| 求解方式 | 每次从头构建并分解 Hessian | 增量更新 Bayes Tree |
| 新增变量/约束 | 重新求解整个问题 | 只更新受影响的子树 |
| 边缘化旧变量 | 不直接支持 | 原生支持 |
| 适合场景 | 离线批量优化 | 在线实时 SLAM |

### 什么是因子图？

**因子图（Factor Graph）** 是一种二部图（bipartite graph），包含两类节点：

1. **变量节点（Variable Nodes）**：用圆圈 ○ 表示，代表待估计的未知量（如位姿 $x_i$、路标 $l_j$）
2. **因子节点（Factor Nodes）**：用方块 ■ 表示，代表对变量的约束（如里程计、观测、先验）

因子节点通过边连接到它所约束的变量节点。因子图编码的是所有变量的**联合概率分布的因式分解**：

$$
p(X) \propto \prod_{i} f_i(X_i)
$$

其中 $X = \{x_0, x_1, \ldots, x_n, l_0, l_1, \ldots, l_m\}$ 是所有变量的集合，$f_i(X_i)$ 是第 $i$ 个因子，$X_i \subseteq X$ 是该因子所关联的变量子集。

**关键直觉**：每个因子就像一个"软约束"，它说"这几个变量应该满足某种关系，偏离越多代价越高"。最大后验估计（MAP）等价于最小化所有因子的负对数之和：

$$
X^* = \arg\max_X p(X) = \arg\min_X \sum_{i} \| h_i(X_i) - z_i \|^2_{\Sigma_i}
$$

其中 $h_i$ 是测量模型，$z_i$ 是实际测量值，$\Sigma_i$ 是噪声协方差。这就是我们熟悉的**非线性最小二乘问题**——和 通用库·Ceres Ceres 求解的问题本质相同，但因子图额外提供了**图拓扑信息**。

### 因子图如何表示 SLAM

一个典型的 2D SLAM 因子图包含以下元素：

```text
因子图结构示意：

  [先验]         [里程计]        [里程计]        [里程计]
    ■──── ○ x0 ────■──── ○ x1 ────■──── ○ x2 ────■──── ○ x3
                    |              |                      |
                   ■ 观测         ■ 观测                 ■ 观测
                    |              |                      |
                   ○ l0           ○ l0                   ○ l1

                   ○ x0 ──────────────────■──────────── ○ x3
                                       [回环约束]
```

| 因子类型 | 连接变量 | 物理含义 | GTSAM 类 |
|---------|---------|---------|----------|
| 先验因子 | $x_0$ | 固定初始位姿 | `PriorFactor<Pose3>` |
| 里程计因子 | $x_i, x_{i+1}$ | 相邻位姿的相对变换 | `BetweenFactor<Pose3>` |
| 观测因子 | $x_i, l_j$ | 从位姿 $x_i$ 观测到路标 $l_j$ | `BearingRangeFactor` |
| 回环因子 | $x_i, x_j$ | 非相邻位姿的相对变换 | `BetweenFactor<Pose3>` |
| GPS 因子 | $x_i$ | 全局位置约束 | `GPSFactor` |

### 消元排序与 fill-in

因子图的优势在于可以利用图的稀疏结构进行高效求解。求解过程的核心是**变量消元（Variable Elimination）**——类似于高斯消元法，但在图结构上进行。

**消元过程**：选择一个变量，收集所有与之相关的因子，将它们组合成一个新的因子（条件分布），然后从图中移除该变量。这个过程会产生新的边——称为 **fill-in**。

**消元排序为什么重要？** 不同的消元顺序会产生不同数量的 fill-in，直接影响计算效率：

$$
\text{计算复杂度} \propto \sum_i |C_i|^3
$$

其中 $|C_i|$ 是消元变量 $i$ 时的 clique 大小（与之相关的变量数）。fill-in 越多，clique 越大，计算越慢。

**例子**：考虑一个链式因子图 $x_0 - x_1 - x_2 - x_3 - x_4$：

- **从前往后消元**（$x_0 \to x_1 \to x_2 \to x_3 \to x_4$）：每次消元只涉及 2 个变量，零 fill-in，效率最高
- **从中间开始消元**（先消 $x_2$）：会在 $x_1$ 和 $x_3$ 之间产生 fill-in 边，后续消元效率降低

GTSAM 内部使用 **COLAMD**（Column Approximate Minimum Degree）算法自动选择近似最优的消元排序，用户通常不需要手动指定。但理解这个概念对理解 iSAM2 的增量更新至关重要。

### 边缘化：移除旧变量

在长时间运行的 SLAM 系统中，变量数量会持续增长。**边缘化（Marginalization）** 允许我们移除旧变量，同时保留它们对剩余变量的约束信息。

数学上，如果我们想从联合分布 $p(x_{\text{old}}, x_{\text{new}})$ 中移除 $x_{\text{old}}$，边缘化计算：

$$
p(x_{\text{new}}) = \int p(x_{\text{old}}, x_{\text{new}}) \, dx_{\text{old}}
$$

对于高斯分布，这等价于在信息矩阵上执行 Schur 补：

$$
\Lambda_{\text{new}} = \Lambda_{nn} - \Lambda_{no} \Lambda_{oo}^{-1} \Lambda_{on}
$$

其中 $\Lambda$ 是信息矩阵（Hessian）的分块。

> **与 Ceres 的对比**：Ceres 没有内置边缘化机制。如果你想在 Ceres 中实现滑动窗口优化，需要手动计算 Schur 补并构造先验因子——这正是 VINS-Mono 中 `marginalization_factor.cpp` 做的事情。而 GTSAM 的因子图天然支持边缘化操作。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：混淆因子图、马尔可夫随机场和贝叶斯网络**
>
> **新手想法**："因子图和贝叶斯网络不都是概率图模型吗？应该可以互换使用。"
>
> **实际上**：三者有本质区别：
>
> | 图模型 | 边的含义 | 因式分解 | 适用推理 |
> |--------|---------|---------|---------|
> | 贝叶斯网络（有向图） | 因果关系 | $p(X) = \prod p(x_i \mid \text{parents}(x_i))$ | 前向采样、d-分离 |
> | 马尔可夫随机场（无向图） | 相关性 | $p(X) \propto \prod \psi_c(X_c)$ | 势函数、配分函数 |
> | **因子图（二部图）** | 变量-约束关系 | $p(X) \propto \prod f_i(X_i)$ | **消元、边缘化、增量更新** |
>
> **根本原因**：因子图是比 MRF 更精细的表示。一个 MRF 中的势函数 $\psi_c(X_c)$ 可能由多个因子构成，因子图显式地区分了它们。这种精细表示对于消元排序和增量更新至关重要。
>
> **正确理解**：因子图 → MRF（每个因子变成势函数）→ 贝叶斯网络（需要指定方向）。因子图信息量最大，转换过程会丢失信息。

> ⚠️ **思维陷阱：认为边缘化是"无损"的**
>
> **新手想法**："边缘化只是移除了变量，信息都保留在 Schur 补中，所以没有任何损失。"
>
> **实际上**：边缘化在**线性高斯**情况下确实是无损的。但在**非线性** SLAM 中，边缘化会将非线性约束线性化后固化为线性先验——这个先验的线性化点被永久固定。如果后续优化导致剩余变量大幅偏离边缘化时的线性化点，这个先验就会引入误差。
>
> **后果**：在视觉 SLAM 中，过早边缘化关键帧可能导致地图漂移。这就是为什么 ORB-SLAM 选择不边缘化而是维护关键帧数据库。
>
> **正确做法**：理解边缘化的线性化误差，在决定边缘化策略时权衡计算效率和精度。

### 练习

1. **[手推]** 画出以下 SLAM 场景的因子图：机器人在 3 个位姿 $x_0, x_1, x_2$ 之间移动，观测到 2 个路标 $l_0, l_1$，其中 $x_0$ 有先验，$x_0$ 和 $x_2$ 之间有回环约束。列出所有因子及其连接的变量。

2. **[思考]** 对于上题中的因子图，分别尝试消元顺序 $(l_0, l_1, x_0, x_1, x_2)$ 和 $(x_0, x_1, x_2, l_0, l_1)$，哪个会产生更多 fill-in？为什么 SLAM 中通常先消元路标变量？

3. **[编程]** 写出 3x3 信息矩阵 $\Lambda$ 的 Schur 补公式，用 Eigen 实现对第一个变量的边缘化。验证边缘化后的信息矩阵维度和数值。

---

## 25.2 GTSAM 核心 API ⭐⭐

### 这一节解决什么问题

理解了因子图的数学原理后，现在要学习如何用 GTSAM 库在 C++ 中构建和求解因子图。GTSAM（Georgia Tech Smoothing and Mapping）是由 Frank Dellaert 团队开发的开源库（GitHub 3.2k+ stars，C++），它是 LIO-SAM、LINS、SC-LIO-SAM 等主流 SLAM 系统的后端核心。

### Key 与 Symbol：变量的身份标识

GTSAM 中每个变量用一个 `Key`（本质是 `uint64_t`）来标识。为了方便区分不同类型的变量，GTSAM 提供了 `Symbol` 类，它将一个字符和一个整数索引编码为一个 `Key`：

```cpp
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X;  // X(i) 表示第 i 个位姿
using gtsam::symbol_shorthand::L;  // L(j) 表示第 j 个路标
using gtsam::symbol_shorthand::V;  // V(i) 表示第 i 个速度
using gtsam::symbol_shorthand::B;  // B(i) 表示第 i 个 IMU 偏差

gtsam::Key pose_key = X(0);      // 第 0 个位姿的 Key
gtsam::Key landmark_key = L(5);  // 第 5 个路标的 Key
```

**为什么这样设计？** `Symbol` 将字符编码在 `uint64_t` 的高位，整数索引在低位，确保不同类型的变量不会发生 Key 冲突。例如 `X(0)` 和 `L(0)` 对应完全不同的 `Key` 值。这比简单用整数编号要安全得多——想象一下 100 个位姿和 100 个路标共用 `0~199` 的编号，稍不注意就会混淆。

### NonlinearFactorGraph 与 Values

GTSAM 的两个核心容器是：

- **`NonlinearFactorGraph`**：存储所有因子（约束）的容器
- **`Values`**：存储所有变量当前估计值的字典（Key → Value）

```cpp
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

gtsam::NonlinearFactorGraph graph;   // 因子图（约束集合）
gtsam::Values initial_estimate;      // 初始估计（变量值）
```

**与 Ceres 的对比**：

| 概念 | Ceres | GTSAM |
|------|-------|-------|
| 约束容器 | `Problem` | `NonlinearFactorGraph` |
| 变量存储 | 原始 `double*` 数组 | `Values`（类型安全的字典） |
| 变量标识 | 内存地址 | `Key`（`Symbol` 编码） |
| 类型安全 | 无（全是 `double*`） | 有（`Values::at<Pose3>(key)`） |

GTSAM 的类型安全设计意味着你不会不小心把一个 `Pose3` 当作 `Point3` 来用——如果类型不匹配，运行时会抛出异常。

### 核心因子类型

GTSAM 内置了大量 SLAM 常用的因子。最重要的几个：

**1. PriorFactor<T>：先验因子（一元因子）**

给单个变量施加先验约束，"钉住"初始位姿或给 GPS 位置一个软约束。

```cpp
#include <gtsam/slam/PriorFactor.h>

// 给 x0 施加先验：初始位姿为单位变换，噪声很小
auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01,   // 旋转 (rad)
                          0.01, 0.01, 0.01     // 平移 (m)
    ).finished()
);
graph.addPrior(X(0), gtsam::Pose3::Identity(), prior_noise);
```

**2. BetweenFactor<T>：相对约束因子（二元因子）**

约束两个变量之间的相对关系，是 SLAM 中最常用的因子。

```cpp
#include <gtsam/slam/BetweenFactor.h>

// 里程计约束：x0 到 x1 的相对变换
gtsam::Pose3 odom_measurement(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.1),
                               gtsam::Point3(1.0, 0.0, 0.0));
auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()
);
graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    X(0), X(1), odom_measurement, odom_noise
);
```

**BetweenFactor 的数学含义**：对于 $SE(3)$ 上的 `BetweenFactor`，其误差函数为：

$$
e(x_i, x_j) = \text{Log}(T_{ij}^{-1} \cdot x_i^{-1} \cdot x_j)
$$

其中 $T_{ij}$ 是测量的相对变换，$\text{Log}$ 是李群的对数映射（通用库·李群manif）。误差为零当且仅当 $x_i^{-1} \cdot x_j = T_{ij}$，即两个位姿的实际相对变换等于测量值。

**3. BearingRangeFactor：方位-距离观测因子**

```cpp
#include <gtsam/slam/BearingRangeFactor.h>

// 从 x0 观测到 l0 的方位角和距离
graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>(
    X(0), L(0),
    gtsam::Rot2::fromAngle(0.5),  // 方位角
    3.0,                           // 距离
    bearing_range_noise
);
```

**4. GPSFactor：全局位置约束**

```cpp
#include <gtsam/navigation/GPSFactor.h>

gtsam::Point3 gps_position(100.0, 200.0, 50.0);
auto gps_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);  // 1m 精度
graph.emplace_shared<gtsam::GPSFactor>(X(5), gps_position, gps_noise);
```

### 噪声模型

GTSAM 的噪声模型体系比 Ceres 更加丰富和结构化：

```cpp
// 1. 对角噪声（各维度独立，不同标准差）
auto diag = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished()
);

// 2. 各向同性噪声（所有维度相同标准差）
auto iso = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);  // 3维，σ=0.5

// 3. 鲁棒噪声（包裹普通噪声模型，抑制外点）
auto robust = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(1.345),  // Huber 核
    gtsam::noiseModel::Isotropic::Sigma(6, 0.1)
);
```

| 噪声模型 | 适用场景 | 数学表达 |
|---------|---------|---------|
| `Diagonal::Sigmas` | 各维度噪声不同 | $\Sigma = \text{diag}(\sigma_1^2, \ldots, \sigma_n^2)$ |
| `Isotropic::Sigma` | 各维度噪声相同 | $\Sigma = \sigma^2 I$ |
| `Constrained` | 硬约束（误差必须为零） | $\sigma \to 0$ |
| `Robust` | 存在外点 | $\rho(\|e\|_\Sigma)$，$\rho$ 为鲁棒核 |

### traits 系统与流形操作

GTSAM 通过 `traits<T>` 模板特化来定义变量类型的流形操作。对于 `Pose3`：

```cpp
// GTSAM 内部（用户通常不需要直接调用）：
namespace gtsam {
  template<> struct traits<Pose3> : public internal::LieGroup<Pose3> {};
}

// traits 定义的关键操作：
// 1. Retract: 从当前点沿切向量移动到新点
//    retract(xi) = this * Exp(xi)        ← 右扰动！
// 2. Local: 计算两点之间的切向量差
//    local(other) = Log(this^{-1} * other)
```

**重要约定：GTSAM 使用右扰动（right perturbation）**。这意味着：

$$
\text{retract}(T, \xi) = T \cdot \text{Exp}(\xi)
$$

这与 通用库·李群manif 中 manif 库的默认约定一致（`rplus`）。对于 `Pose3` 的切向量，GTSAM 使用**旋转在前、平移在后**的排列：

$$
\xi = [\omega_x, \omega_y, \omega_z, v_x, v_y, v_z]^T \in \mathbb{R}^6
$$

> **注意**：`Pose2` 的切向量排列是 $[v_x, v_y, \theta]^T$（平移在前、旋转在后），与 `Pose3` **不同**。这是 GTSAM 的历史遗留问题，初学者很容易搞混。

### 基本优化器

GTSAM 提供两种基本的批量优化器：

```cpp
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// 方式 A：Gauss-Newton（收敛快，但初值要好）
gtsam::GaussNewtonParams gn_params;
gn_params.maxIterations = 100;
gn_params.relativeErrorTol = 1e-5;
gtsam::GaussNewtonOptimizer gn_optimizer(graph, initial_estimate, gn_params);
gtsam::Values result_gn = gn_optimizer.optimize();

// 方式 B：Levenberg-Marquardt（更鲁棒，初值可以差一些）
gtsam::LevenbergMarquardtParams lm_params;
lm_params.maxIterations = 100;
lm_params.lambdaInitial = 1e-5;
gtsam::LevenbergMarquardtOptimizer lm_optimizer(graph, initial_estimate, lm_params);
gtsam::Values result_lm = lm_optimizer.optimize();
```

**选择指南**：

| 场景 | 推荐优化器 | 原因 |
|------|-----------|------|
| 初始估计质量好（IMU 预积分提供） | Gauss-Newton | 收敛速度快 |
| 初始估计可能较差 | Levenberg-Marquardt | 阻尼项提供鲁棒性 |
| 在线实时优化 | **iSAM2**（25.3 节） | 增量更新，避免重复求解 |

### 完整 2D SLAM 示例

下面是一个完整的 2D 位姿图 SLAM 示例，展示 GTSAM 的典型使用流程：

```cpp
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

using namespace gtsam;
using symbol_shorthand::X;

int main() {
    // ========== Step 1: 构建因子图 ==========
    NonlinearFactorGraph graph;

    // 先验因子：固定 x0 的初始位姿
    auto prior_noise = noiseModel::Diagonal::Sigmas(
        Vector3(0.3, 0.3, 0.1)  // (x, y, theta)
    );
    graph.addPrior(X(0), Pose2(0.0, 0.0, 0.0), prior_noise);

    // 里程计因子：x0 → x1 → x2 → x3 → x4
    auto odom_noise = noiseModel::Diagonal::Sigmas(
        Vector3(0.2, 0.2, 0.1)
    );
    graph.emplace_shared<BetweenFactor<Pose2>>(
        X(0), X(1), Pose2(2.0, 0.0, 0.0), odom_noise);
    graph.emplace_shared<BetweenFactor<Pose2>>(
        X(1), X(2), Pose2(2.0, 0.0, M_PI_2), odom_noise);
    graph.emplace_shared<BetweenFactor<Pose2>>(
        X(2), X(3), Pose2(2.0, 0.0, M_PI_2), odom_noise);
    graph.emplace_shared<BetweenFactor<Pose2>>(
        X(3), X(4), Pose2(2.0, 0.0, M_PI_2), odom_noise);

    // 回环因子：x4 → x0（机器人回到原点附近）
    auto loop_noise = noiseModel::Diagonal::Sigmas(
        Vector3(0.2, 0.2, 0.1)
    );
    graph.emplace_shared<BetweenFactor<Pose2>>(
        X(4), X(0), Pose2(2.0, 0.0, M_PI_2), loop_noise);

    // ========== Step 2: 设置初始估计 ==========
    // 初始估计要有一定噪声（模拟真实情况）
    Values initial;
    initial.insert(X(0), Pose2(0.2, -0.3, 0.05));
    initial.insert(X(1), Pose2(2.3,  0.1, -0.1));
    initial.insert(X(2), Pose2(4.1,  0.1,  M_PI_2 - 0.1));
    initial.insert(X(3), Pose2(4.0,  2.0,  M_PI - 0.05));
    initial.insert(X(4), Pose2(2.1,  2.1,  -M_PI_2 + 0.1));

    // ========== Step 3: 优化 ==========
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();

    // ========== Step 4: 查看结果 ==========
    result.print("Optimized Result:\n");

    // 提取单个位姿
    Pose2 optimized_x2 = result.at<Pose2>(X(2));
    std::cout << "x2 = (" << optimized_x2.x()
              << ", " << optimized_x2.y()
              << ", " << optimized_x2.theta() << ")" << std::endl;

    return 0;
}
```

**关键观察**：

1. 初始估计有噪声，但回环约束会"拉回"整个轨迹，使其整体一致
2. 如果去掉回环因子，优化后的轨迹会有累积漂移——回环是闭合漂移的关键
3. 先验因子的噪声不宜太小，否则会过度约束 $x_0$，导致其他位姿无法自由调整

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：Key 冲突**
>
> **错误做法**：用裸整数作为 Key，不同类型的变量编号重叠
> ```cpp
> // ❌ 错误：位姿和路标都用 0, 1, 2...
> initial.insert(0, Pose3(...));   // Key = 0
> initial.insert(0, Point3(...));  // Key = 0，冲突！运行时崩溃
> ```
>
> **现象**：`Values::insert` 抛出异常 "Key already exists"，或者更糟——悄悄覆盖了之前的值（取决于 GTSAM 版本）。
>
> **正确做法**：始终使用 `Symbol` 或 `symbol_shorthand`：
> ```cpp
> // ✅ 正确：X(0) 和 L(0) 有不同的 Key 值
> initial.insert(X(0), Pose3(...));
> initial.insert(L(0), Point3(...));
> ```

> ⚠️ **编程陷阱：噪声模型维度不匹配**
>
> **错误做法**：`Pose3` 因子用 3 维噪声（应该是 6 维）
> ```cpp
> // ❌ 错误：Pose3 的切空间是 6 维，但噪声只给了 3 维
> auto wrong_noise = noiseModel::Isotropic::Sigma(3, 0.1);
> graph.addPrior(X(0), Pose3::Identity(), wrong_noise);
> // 运行时崩溃：维度不匹配
> ```
>
> **根本原因**：噪声模型的维度必须等于因子残差向量的维度。对于 `Pose3`，残差是 6 维的切向量 $[\omega_x, \omega_y, \omega_z, v_x, v_y, v_z]^T$。
>
> **正确做法**：
> ```cpp
> // ✅ 正确：6 维噪声对应 Pose3 的 6 维切空间
> auto correct_noise = noiseModel::Isotropic::Sigma(6, 0.1);
> ```
>
> **自检方法**：记住常用类型的维度——`Pose2` → 3 维，`Pose3` → 6 维，`Point2` → 2 维，`Point3` → 3 维。

> ⚠️ **概念误区：GTSAM 的 Pose3 切向量排列与论文惯例不同**
>
> **新手想法**："SE(3) 的切向量应该是 $[v, \omega]^T$，平移在前旋转在后。"
>
> **实际上**：GTSAM 的 `Pose3` 切向量排列是 $[\omega, v]^T$（**旋转在前，平移在后**）。这与 Sophus 库（$[v, \omega]^T$）恰好相反。而 `Pose2` 又是 $[v_x, v_y, \theta]^T$（平移在前）。
>
> **后果**：如果按错误的顺序设置噪声模型的 sigma 值，旋转的噪声会被当作平移处理，反之亦然。优化结果会非常奇怪但不会报错。
>
> **正确做法**：查看 GTSAM 文档确认排列，或直接在代码中测试：
> ```cpp
> Pose3 T = Pose3::Identity();
> Vector6 xi; xi << 0.1, 0, 0, 0, 0, 0;  // 只有 omega_x
> Pose3 T2 = T.retract(xi);
> // 如果 T2 只有旋转变化，说明前三维是旋转
> ```

### 练习

1. **[编程]** 修改上面的 2D SLAM 示例：(a) 去掉回环因子，对比优化结果；(b) 将先验噪声缩小 10 倍，观察对其他位姿估计的影响；(c) 将里程计噪声放大 5 倍，观察优化结果的变化。

2. **[编程]** 扩展示例，添加 2 个路标变量 `L(0)` 和 `L(1)`，使用 `BearingRangeFactor<Pose2, Point2>` 添加从部分位姿到路标的观测。验证路标位置在优化后的精度。

3. **[思考]** 为什么 GTSAM 不直接用 `std::map<int, Eigen::VectorXd>` 来存储变量，而要设计 `Values` 容器和 `traits` 系统？从类型安全和流形操作两个角度分析。

---

## 25.3 iSAM2 增量优化 ⭐⭐

### 这一节解决什么问题

在实时 SLAM 中，机器人每 100ms 就会产生新的位姿和观测。如果每次都用 Levenberg-Marquardt 从头优化整个因子图，计算量会随时间**线性增长**，最终无法满足实时性要求。iSAM2（incremental Smoothing and Mapping 2）是 GTSAM 的核心算法——它只更新受新因子影响的变量，实现**增量优化**。

### 为什么批量优化不够用？

考虑一个运行 10 分钟、10Hz 的激光 SLAM 系统：

- 6000 个位姿变量（每个 6 维）→ 36000 维优化问题
- 每帧新增 1 个位姿 + 若干因子
- LM 优化需要迭代构建和求解 $36000 \times 36000$ 的线性系统

即使 Hessian 矩阵是稀疏的，每帧完整求解仍需要约 50-200ms。而 iSAM2 每帧只需要 5-20ms，因为它**只重新处理受影响的变量子集**。

关键数据对比：

| 方法 | 每帧耗时（6000 变量） | 耗时增长 | 适用场景 |
|------|---------------------|---------|---------|
| LM 批量优化 | 50-200ms | 随变量数线性增长 | 离线地图构建 |
| iSAM2 增量优化 | 5-20ms | 近乎常数 | 实时 SLAM |

### Bayes Tree 数据结构

iSAM2 的核心数据结构是 **Bayes Tree**（贝叶斯树）。要理解它，需要先理解因子图 → 贝叶斯网络 → Bayes Tree 的转换链：

**Step 1：因子图 → 贝叶斯网络**

对因子图进行变量消元（Variable Elimination），得到一组条件分布，构成一个有向无环图（DAG）——贝叶斯网络。消元过程类似于对 Hessian 矩阵做 Cholesky 分解：

$$
\Lambda = R^T R
$$

其中 $R$ 是上三角矩阵，$R$ 的非零模式对应贝叶斯网络的结构。

**Step 2：贝叶斯网络 → Bayes Tree**

将贝叶斯网络中具有相同分离集（separator）的条件分布聚合成 **clique**，形成树结构。每个 clique 存储一个小的条件高斯分布。

**Bayes Tree 的关键性质**：

1. **根节点的 clique** 包含最晚消元的变量
2. **叶节点的 clique** 包含最早消元的变量
3. **父子关系**编码了变量之间的条件依赖
4. **求解**只需要从叶到根的反向替代（back-substitution）

```text
Bayes Tree 示意图：

          [x3, x4]          ← 根 clique
          /       \
     [x1 | x3]  [x2 | x4]  ← 子 clique（分离变量在 | 右侧）
         |
    [l0 | x1]               ← 叶 clique
```

### iSAM2 算法：增量更新

当新因子到来时，iSAM2 不需要重新构建整个 Bayes Tree。它的工作流程是：

**Step 1：确定受影响的 clique**

新因子涉及的变量对应 Bayes Tree 中的某些 clique。从这些 clique 向上追溯到根节点，所有路径上的 clique 都"受影响"。

**Step 2：从 Bayes Tree 中拆除受影响的子树**

将受影响的 clique 从树中取出，转换回因子的形式。

**Step 3：添加新因子并重新消元**

将新因子和拆除的旧因子合并，在受影响的变量上重新执行消元，生成新的 clique。

**Step 4：将新 clique 插回 Bayes Tree**

重新接入树结构。只需要更新一个子树，其余部分不变。

**直觉理解**：想象一棵真实的树。你在某个枝干上嫁接了一根新枝（新因子），只需要重新修剪从嫁接点到树干的部分，其他枝干完全不受影响。

### GTSAM 中的 iSAM2 API

```cpp
#include <gtsam/nonlinear/ISAM2.h>

// ========== 配置 iSAM2 参数 ==========
gtsam::ISAM2Params params;

// 重线性化阈值：变量估计变化超过此值才重新线性化
params.relinearizeThreshold = 0.1;

// 重线性化跳过次数：每隔多少次 update 才检查是否需要重线性化
params.relinearizeSkip = 1;

// 因子化方式：CHOLESKY（默认）或 QR
params.factorization = gtsam::ISAM2Params::CHOLESKY;

// 是否缓存线性化因子（提高速度，增加内存）
params.cacheLinearizedFactors = true;

// 创建 iSAM2 实例
gtsam::ISAM2 isam2(params);
```

**在线增量更新循环**：

```cpp
// 模拟在线 SLAM 循环
for (int i = 0; i < num_frames; ++i) {
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;

    // 第一帧：添加先验
    if (i == 0) {
        new_factors.addPrior(X(0), initial_pose, prior_noise);
        new_values.insert(X(0), initial_pose);
    }

    // 添加里程计因子
    if (i > 0) {
        Pose3 odom = getOdometry(i);  // 从传感器获取
        new_factors.emplace_shared<BetweenFactor<Pose3>>(
            X(i-1), X(i), odom, odom_noise
        );
        // 初始估计 = 上一帧结果 + 里程计
        Pose3 prev_pose = isam2.calculateEstimate<Pose3>(X(i-1));
        new_values.insert(X(i), prev_pose * odom);
    }

    // 如果检测到回环
    if (hasLoopClosure(i)) {
        auto [loop_from, loop_to, loop_measurement] = getLoopClosure(i);
        new_factors.emplace_shared<BetweenFactor<Pose3>>(
            X(loop_from), X(loop_to), loop_measurement, loop_noise
        );
    }

    // ========== 核心：增量更新 ==========
    isam2.update(new_factors, new_values);

    // 可以多次调用 update() 进行额外的迭代
    // isam2.update();  // 无新因子，但重新线性化受影响的变量

    // 获取当前最优估计
    gtsam::Values current_estimate = isam2.calculateEstimate();
    Pose3 current_pose = current_estimate.at<Pose3>(X(i));
}
```

### 重线性化阈值

iSAM2 的一个关键创新是 **fluid relinearization**（流式重线性化）。在非线性优化中，每次迭代都需要在当前估计点重新线性化所有因子。但如果某个变量的估计变化很小，重新线性化的收益很低。

iSAM2 为每个变量维护一个"自上次线性化以来的估计变化量" $\|\Delta x_i\|$：

- 如果 $\|\Delta x_i\| > \text{relinearizeThreshold}$，则重新线性化与 $x_i$ 相关的因子
- 否则，复用上次的线性化结果

这带来了巨大的计算节省——在大多数帧中，只有少数变量需要重新线性化（通常是最近几帧和回环涉及的变量）。

**`relinearizeThreshold` 参数的权衡**：

| 值 | 重线性化频率 | 精度 | 速度 |
|----|-------------|------|------|
| 太小（0.001） | 几乎每帧都重线性化 | 高 | 慢（接近批量优化） |
| 适中（0.01-0.1） | 只在必要时重线性化 | 良好 | 快（默认选择） |
| 太大（10.0） | 几乎从不重线性化 | 差（线性化误差累积） | 最快但不准 |

### 与 Ceres 的对比

| 特性 | Ceres Solver | GTSAM iSAM2 |
|------|-------------|-------------|
| 更新方式 | 每次从头求解 | 增量更新 Bayes Tree |
| 新增变量/因子 | 重建整个 Problem | `update(new_factors, new_values)` |
| 移除因子 | 不直接支持 | `update()` 的 `removeFactorIndices` 参数 |
| 变量排序 | 用户可提供 ordering hint | GTSAM 自动维护近似最优排序 |
| 变量边缘化 | 需手动实现 Schur 补 | Bayes Tree 天然支持 |
| 并行性 | 支持多线程求解 | Bayes Tree 子树可并行 |
| 适合场景 | 批量优化、标定、BA | 在线 SLAM、实时状态估计 |

**一句话总结**：Ceres 是"通用瑞士军刀"——什么优化问题都能解但不针对 SLAM 优化；GTSAM+iSAM2 是"SLAM 专用利器"——利用因子图的结构实现增量更新。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：不调用额外的 `update()` 导致线性化陈旧**
>
> **错误做法**：每帧只调用一次 `update(new_factors, new_values)`，即使回环闭合后也不额外迭代
>
> **现象**：回环闭合后，远处的位姿没有被充分修正，地图闭合不完整
>
> **根本原因**：单次 `update()` 只执行一轮增量更新。对于回环闭合这种影响范围很大的约束，一轮更新可能不够——需要多次重线性化才能让修正传播到整个图。
>
> **正确做法**：在回环闭合后额外调用几次 `update()`：
> ```cpp
> isam2.update(new_factors, new_values);  // 第一次：添加回环因子
> isam2.update();  // 第二次：无新因子，重新线性化受影响变量
> isam2.update();  // 第三次：进一步传播修正
> ```
>
> **自检方法**：对比 `isam2.update()` 一次和多次后的结果差异。如果差异很大，说明一次更新不够。

> ⚠️ **概念误区：认为 iSAM2 每帧都是"精确"解**
>
> **新手想法**："iSAM2 是增量优化，所以每帧给出的结果和批量 LM 优化一样。"
>
> **实际上**：iSAM2 通过延迟重线性化换取速度。在两次重线性化之间，它使用的是旧的线性化点，因此结果可能与批量优化有微小差异。这个差异由 `relinearizeThreshold` 控制。
>
> **正确理解**：iSAM2 在精度和速度之间做了精心的权衡。对于大多数 SLAM 应用，这个近似是完全可接受的。如果需要最高精度（如离线地图构建），可以最后再跑一次批量优化。

> ⚠️ **思维陷阱：认为 iSAM2 可以无限增长因子图**
>
> **新手想法**："iSAM2 是增量的，所以可以一直往里加因子，不需要管内存。"
>
> **实际上**：虽然每帧更新是增量的，但 Bayes Tree 的内存占用随变量数量线性增长。长时间运行（数小时）后，内存会成为瓶颈。生产系统（如 LIO-SAM）通常配合滑动窗口或关键帧策略来限制图的大小。
>
> **正确做法**：监控因子图大小，必要时进行边缘化或因子裁剪。

### 练习

1. **[编程]** 将 25.2 节的 2D SLAM 示例改为使用 iSAM2 增量求解：每次添加一个位姿和对应的里程计因子，最后添加回环。对比每步 `calculateEstimate()` 的结果与批量 LM 的结果。

2. **[实验]** 分别设置 `relinearizeThreshold` 为 0.001、0.01、0.1、1.0，在一个包含回环的数据集上运行 iSAM2。记录每帧平均耗时和最终轨迹误差，画出精度-速度权衡曲线。

3. **[思考]** 假设一个因子图是链式的（$x_0 - x_1 - \cdots - x_{999}$），在末端 $x_{999}$ 添加一个到 $x_0$ 的回环因子。iSAM2 需要重新处理多少个 clique？为什么回环是 iSAM2 中计算代价最高的操作？

---

## 25.4 IMU 预积分 ⭐⭐⭐

### 这一节解决什么问题

在 LiDAR-Inertial SLAM（如 LIO-SAM）中，IMU 提供高频（100-400Hz）的角速度和加速度测量。如何将这些高频测量整合到低频（10Hz）的因子图中？直接为每个 IMU 测量创建一个因子会导致因子图爆炸。**IMU 预积分（Preintegration）** 将两个关键帧之间的所有 IMU 测量"压缩"成一个因子。

### 预积分理论回顾

回忆 通用库·李群manif 中李群的指数映射。IMU 测量的连续时间模型：

$$
\dot{R}(t) = R(t) \cdot [\tilde{\omega}(t) - b_g - \eta_g]_\times
$$
$$
\dot{v}(t) = R(t) \cdot (\tilde{a}(t) - b_a - \eta_a) + g
$$
$$
\dot{p}(t) = v(t)
$$

其中 $\tilde{\omega}$ 和 $\tilde{a}$ 是 IMU 原始测量，$b_g$ 和 $b_a$ 是陀螺仪和加速度计偏差，$\eta_g$ 和 $\eta_a$ 是白噪声，$g$ 是重力向量。

**预积分的核心思想**：在两个关键帧 $i$ 和 $j$ 之间，将 IMU 积分结果定义在**局部参考系**（关键帧 $i$ 的坐标系）中，使得积分结果与关键帧 $i$ 的全局位姿 $R_i, v_i, p_i$ 无关：

$$
\Delta R_{ij} = \prod_{k=i}^{j-1} \text{Exp}((\tilde{\omega}_k - b_g^i) \Delta t)
$$
$$
\Delta v_{ij} = \sum_{k=i}^{j-1} \Delta R_{ik} \cdot (\tilde{a}_k - b_a^i) \Delta t
$$
$$
\Delta p_{ij} = \sum_{k=i}^{j-1} \left[ \Delta v_{ik} \Delta t + \frac{1}{2} \Delta R_{ik} \cdot (\tilde{a}_k - b_a^i) \Delta t^2 \right]
$$

**为什么需要预积分？** 如果直接积分（不用预积分），每次偏差 $b_g, b_a$ 更新后，都需要重新从 $i$ 到 $j$ 积分所有 IMU 测量。预积分通过一阶近似，允许在偏差变化较小时只做一次修正：

$$
\Delta R_{ij}(b_g) \approx \Delta R_{ij}(\hat{b}_g) \cdot \text{Exp}\left(\frac{\partial \Delta R_{ij}}{\partial b_g} \delta b_g\right)
$$

这意味着当优化器更新了偏差估计（$\delta b_g = b_g^{\text{new}} - \hat{b}_g$），我们不需要重新积分所有 IMU 测量——只需要用上面的一阶修正公式即可。这个近似在 $\delta b_g$ 较小时非常精确，极大地提高了计算效率。

### GTSAM 中的 IMU 因子

GTSAM 提供两种 IMU 因子：

| 因子 | 变量 | 偏差处理 |
|------|------|---------|
| `ImuFactor` | $(x_i, v_i, x_j, v_j, b_i)$ — 5 路 | 偏差参与优化但不约束前后一致性（需单独添加偏差随机游走因子） |
| `CombinedImuFactor` | $(x_i, v_i, x_j, v_j, b_i, b_j)$ — 6 路 | **同时优化前后偏差** |

LIO-SAM 使用 `ImuFactor`（5 路因子）+ 单独的 `BetweenFactor<imuBias::ConstantBias>` 来约束偏差随机游走。这与 `CombinedImuFactor`（6 路因子，内置偏差约束）的方式不同——LIO-SAM 选择前者是因为解耦后调参更灵活。

> ⚠️ **常见误解：LIO-SAM 使用 CombinedImuFactor**
>
> **错误认知**：许多教程声称 LIO-SAM 使用 `CombinedImuFactor`。
>
> **实际上**：LIO-SAM 的 `imuPreintegration.cpp` 使用 `ImuFactor` + `PreintegratedImuMeasurements`，并单独添加 `BetweenFactor<imuBias::ConstantBias>` 约束偏差连续性。
>
> **验证**：查看 LIO-SAM 源码 `imuPreintegration.cpp` 中的 `imuIntegratorOpt_` 类型即可确认。

```cpp
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

// ========== Step 1: 配置 IMU 参数 ==========
auto imu_params = gtsam::PreintegrationParams::MakeSharedU();
// MakeSharedU() — U = "Up"，假设 Z 轴朝上（NED 用 MakeSharedD()）

// 加速度计噪声密度 [m/s^2/√Hz]
imu_params->accelerometerCovariance =
    gtsam::I_3x3 * pow(0.01, 2);

// 陀螺仪噪声密度 [rad/s/√Hz]
imu_params->gyroscopeCovariance =
    gtsam::I_3x3 * pow(0.001, 2);

// 积分不确定性（数值积分引入的误差）
imu_params->integrationCovariance =
    gtsam::I_3x3 * pow(1e-4, 2);

// ========== Step 2: 创建预积分器 ==========
gtsam::imuBias::ConstantBias prior_bias;  // 零偏差初始值
gtsam::PreintegratedImuMeasurements preintegrated(imu_params, prior_bias);

// ========== Step 3: 积分 IMU 测量 ==========
// 在两个关键帧之间，逐个积分 IMU 测量
for (const auto& imu_msg : imu_measurements_between_frames) {
    gtsam::Vector3 acc(imu_msg.ax, imu_msg.ay, imu_msg.az);
    gtsam::Vector3 gyro(imu_msg.wx, imu_msg.wy, imu_msg.wz);
    double dt = imu_msg.dt;
    preintegrated.integrateMeasurement(acc, gyro, dt);
}

// ========== Step 4: 创建 IMU 因子并添加到图 ==========
gtsam::ImuFactor imu_factor(
    X(i), V(i),    // 前一帧位姿和速度
    X(j), V(j),    // 后一帧位姿和速度
    B(i),           // 偏差（仅前一帧）
    preintegrated
);
graph.add(imu_factor);

// 单独添加偏差随机游走约束（LIO-SAM 的做法）
auto bias_noise = noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001).finished());
graph.add(BetweenFactor<imuBias::ConstantBias>(
    B(i), B(j), imuBias::ConstantBias(), bias_noise));

// 为下一帧重置预积分器（使用当前偏差估计）
preintegrated.resetIntegrationAndSetBias(current_bias_estimate);
```

### NavState：位姿 + 速度

GTSAM 定义了 `NavState` 来封装导航状态（位姿 + 速度）：

```cpp
#include <gtsam/navigation/NavState.h>

gtsam::NavState state(
    gtsam::Pose3(R, t),   // 位姿
    gtsam::Vector3(vx, vy, vz)  // 世界坐标系下的速度
);

// 使用预积分结果预测下一帧的 NavState
gtsam::NavState predicted_state = preintegrated.predict(
    prev_state, prev_bias
);
```

### LIO-SAM 中的 IMU 因子构建

在 LIO-SAM 的 `imuPreintegration.cpp` 中，IMU 预积分的使用流程：

1. **收集 IMU 测量**：在两个 LiDAR 帧之间，收集所有 IMU 数据
2. **预积分**：调用 `integrateMeasurement()` 逐个积分
3. **添加 IMU 因子**：创建 `ImuFactor` + `BetweenFactor<imuBias::ConstantBias>` 添加到因子图
4. **用 iSAM2 优化**：同时估计位姿、速度和偏差
5. **重置预积分器**：用优化后的偏差重置

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：IMU 坐标系方向错误**
>
> **错误做法**：不区分 NED（North-East-Down）和 ENU（East-North-Up）坐标系
> ```cpp
> // ❌ 错误：IMU 是 NED 坐标系，但用了 ENU 的参数
> auto params = PreintegrationParams::MakeSharedU();
> // MakeSharedU() 假设 Z 轴朝上（ENU），重力 g = [0, 0, -9.81]
> // 如果 IMU 实际是 NED（Z 轴朝下），重力方向反了！
> ```
>
> **现象**：预积分的位移估计完全错误（例如：向上变成向下），系统在几秒内发散。
>
> **根本原因**：`MakeSharedU()` 设置 $g = [0, 0, -9.81]^T$（Z 朝上），`MakeSharedD()` 设置 $g = [0, 0, 9.81]^T$（Z 朝下）。如果选错，加速度计的重力补偿方向相反。
>
> **正确做法**：确认你的 IMU 驱动输出的坐标系约定：
> ```cpp
> // 如果 Z 轴朝上（ENU，大多数 ROS 传感器）
> auto params = PreintegrationParams::MakeSharedU();
>
> // 如果 Z 轴朝下（NED，航空惯例）
> auto params = PreintegrationParams::MakeSharedD();
> ```

> ⚠️ **概念误区：忘记在偏差更新后重置预积分器**
>
> **错误做法**：优化后偏差改变了，但继续使用旧偏差的预积分器
>
> **现象**：预积分结果越来越不准，因为内部的一阶偏差修正项 $\delta b$ 越来越大，超出了线性近似的有效范围。
>
> **正确做法**：
> ```cpp
> // 优化完成后，获取最新偏差估计
> auto updated_bias = result.at<imuBias::ConstantBias>(B(i));
> // 用最新偏差重置预积分器
> preintegrated.resetIntegrationAndSetBias(updated_bias);
> ```

### 练习

1. **[手推]** 写出 $\Delta R_{ij}$ 对陀螺仪偏差 $b_g$ 的一阶近似修正公式。说明为什么这个近似在偏差变化较小时有效。

2. **[编程]** 生成合成 IMU 数据（恒定角速度 + 恒定加速度 + 重力），使用 GTSAM 的 `PreintegratedImuMeasurements` 进行预积分，对比预积分预测的状态与解析积分的结果。

3. **[思考]** 对比 `ImuFactor`（5 路）和 `CombinedImuFactor`（6 路）的设计差异：LIO-SAM 为什么选择前者 + 单独的偏差 BetweenFactor？分别在什么场景下各有优势？

---

## 25.5 自定义因子 ⭐⭐

### 这一节解决什么问题

GTSAM 内置的因子覆盖了大部分常见场景，但实际项目中你经常需要**自定义约束**。例如：点到面 ICP 约束、高度约束、速度约束等。GTSAM 提供两种方式来创建自定义因子：手写 Jacobian 和 Expression 自动微分。

### 方式 A：继承 NoiseModelFactorN

最直接的方式是继承 `NoiseModelFactor1`（一元因子）、`NoiseModelFactor2`（二元因子）等模板类，实现 `evaluateError()` 方法。

**为什么这样设计？** GTSAM 的因子体系遵循一个清晰的继承链：

```text
NonlinearFactor                        ← 抽象基类
  └─ NoiseModelFactor                  ← 带噪声模型的因子
       ├─ NoiseModelFactor1<VALUE1>    ← 一元因子
       ├─ NoiseModelFactor2<V1, V2>    ← 二元因子
       ├─ NoiseModelFactor3<V1,V2,V3>  ← 三元因子
       └─ ...
```

每个 `NoiseModelFactorN` 已经处理了：

1. 从 `Values` 中按 Key 提取变量
2. 计算加权残差 $\Sigma^{-1/2} \cdot e$
3. 将 Jacobian 传递给优化器

你只需要实现 `evaluateError()`——计算**未加权的残差向量**和（可选的）Jacobian 矩阵。

**示例：自定义高度约束因子**

假设你有一个气压计，测量机器人的高度 $z$。需要创建一个一元因子，约束 `Pose3` 的平移 z 分量。

```cpp
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

class HeightFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
    double measured_height_;  // 气压计测量的高度

public:
    // 构造函数
    HeightFactor(gtsam::Key pose_key, double measured_height,
                 const gtsam::SharedNoiseModel& noise_model)
        : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
          measured_height_(measured_height) {}

    // 核心：计算残差（和可选的 Jacobian）
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        gtsam::OptionalMatrixType H = nullptr  // GTSAM 4.2+ 用 OptionalMatrixType
    ) const override {

        // 残差 = 预测值 - 测量值
        double predicted_height = pose.translation().z();
        gtsam::Vector1 error;
        error << predicted_height - measured_height_;

        // 如果需要 Jacobian（优化器会传入非空指针）
        if (H) {
            // Jacobian: d(error) / d(xi)，其中 xi 是 Pose3 的 6 维切向量
            // xi = [omega_x, omega_y, omega_z, v_x, v_y, v_z]^T
            // error = f(pose) = pose.translation().z()
            // 只有 v_z（第 6 维）有贡献
            // 正确的 Jacobian：对一般旋转 R，d(height)/d(xi) = [0_{1x3}, R的第3行]
            *H = (gtsam::Matrix16() << 0, 0, 0,
                  pose.rotation().matrix()(2,0),
                  pose.rotation().matrix()(2,1),
                  pose.rotation().matrix()(2,2)).finished();
        }

        return error;
    }
};

// 使用
graph.emplace_shared<HeightFactor>(
    X(5), 10.0,  // 位姿 X(5) 的高度约束为 10m
    gtsam::noiseModel::Isotropic::Sigma(1, 0.5)  // 1 维，σ=0.5m
);
```

**Jacobian 推导详解**：

误差函数 $e = t_z - z_{\text{meas}}$，其中 $t_z$ 是位姿平移的 z 分量。

GTSAM 使用右扰动，所以我们需要计算：

$$
\frac{\partial e}{\partial \xi} = \lim_{\xi \to 0} \frac{e(T \cdot \text{Exp}(\xi)) - e(T)}{\xi}
$$

其中 $\xi = [\omega^T, v^T]^T \in \mathbb{R}^6$。由于 $T \cdot \text{Exp}(\xi)$ 的平移部分近似为 $t + R \cdot v$（一阶近似），$z$ 分量的变化为 $R_{3,:} \cdot v$。如果我们只关心单位变换附近的 Jacobian（$R \approx I$），则 $\frac{\partial e}{\partial v_z} = 1$，其余为 0。

> **注意**：对于一般的 $R$，精确的 Jacobian 是 $[0_{1 \times 3}, R_{3,:}]$。上面的简化版本 $[0,0,0,0,0,1]$ 只在 $R$ 接近单位矩阵时精确。实际中 GTSAM 的线性化在当前估计点进行，所以用精确版本更好。

### 示例：点到面 ICP 因子

这是 SLAM 中非常实用的因子——约束一个点投影到目标平面上的距离为零。

```cpp
class PointToPlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
    gtsam::Point3 source_point_;   // 源点云中的点（body 坐标系）
    gtsam::Point3 target_normal_;  // 目标平面的法向量（world 坐标系）
    double target_d_;              // 目标平面的 d 参数（ax+by+cz+d=0）

public:
    PointToPlaneFactor(gtsam::Key pose_key,
                       const gtsam::Point3& source_point,
                       const gtsam::Point3& target_normal,
                       double target_d,
                       const gtsam::SharedNoiseModel& noise)
        : NoiseModelFactor1<gtsam::Pose3>(noise, pose_key),
          source_point_(source_point),
          target_normal_(target_normal),
          target_d_(target_d) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        gtsam::OptionalMatrixType H = nullptr
    ) const override {

        // 将源点变换到世界坐标系
        gtsam::Matrix36 H_transform;
        gtsam::Point3 world_point = pose.transformFrom(
            source_point_, H ? &H_transform : nullptr
        );

        // 残差 = 点到面距离 = n^T * p + d
        double distance = target_normal_.dot(world_point) + target_d_;

        if (H) {
            // d(distance) / d(xi) = n^T * d(world_point) / d(xi)
            *H = target_normal_.transpose() * H_transform;  // 1x6
        }

        return gtsam::Vector1(distance);
    }
};
```

**为什么使用 `pose.transformFrom()` 而不是手动 `R * p + t`？** 因为 `transformFrom()` 已经内置了精确的 Jacobian 计算（通过 `OptionalMatrixType`），我们可以利用链式法则复用它，避免手推整个 Jacobian。这是 GTSAM 设计的精妙之处——几何操作都提供了可选的 Jacobian 输出。

### 方式 B：Expression API（自动微分）

如果你不想手推 Jacobian，GTSAM 提供了 Expression API，它使用**反向模式自动微分（reverse-mode AD）**来自动计算 Jacobian——输出维度通常小于输入维度，反向模式更高效。

```cpp
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>

// 创建 Expression 对象（延迟计算）
gtsam::Expression<gtsam::Pose3> pose_expr(X(0));    // 代表变量 X(0)
gtsam::Expression<gtsam::Point3> point_expr(source_point);  // 常量

// 组合 Expression：将点从 body 变换到 world
gtsam::Expression<gtsam::Point3> world_point_expr =
    gtsam::transformFrom(pose_expr, point_expr);

// 进一步组合：提取 z 分量等
// 可以用 lambda 或自定义函数
```

**Expression API vs 手写 Jacobian 对比**：

| 特性 | 手写 Jacobian | Expression API |
|------|-------------|---------------|
| 开发速度 | 慢（需要推导） | 快（自动计算） |
| 运行性能 | 最优 | 略慢（约 10-30%） |
| 调试难度 | 高（容易出错） | 低（自动微分几乎不会出错） |
| 适用场景 | 性能关键的因子 | 快速原型、复杂函数 |
| GTSAM 版本 | 所有版本 | GTSAM 4.0+ |

**推荐工作流**：

1. **原型阶段**：用 Expression API 快速实现因子，验证数学模型的正确性
2. **优化阶段**：如果 profiling 显示该因子是瓶颈，改为手写 Jacobian
3. **验证**：用 GTSAM 的 `numericalDerivative` 函数对比手写 Jacobian 和数值微分

```cpp
#include <gtsam/base/numericalDerivative.h>

// 验证手写 Jacobian 的正确性
auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
    [&](const gtsam::Pose3& p) -> gtsam::Vector1 {
        return factor.evaluateError(p);
    },
    pose_value
);

// 与手写的 H 对比
std::cout << "Max Jacobian error: "
          << (numerical_H - analytical_H).lpNorm<Eigen::Infinity>()
          << std::endl;
// 应该 < 1e-5
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：Jacobian 维度错误**
>
> **错误做法**：返回的 Jacobian 矩阵行列数不对
> ```cpp
> // ❌ 错误：残差是 1 维，Pose3 切空间是 6 维
> // Jacobian 应该是 1x6，但写成了 6x1
> *H = (gtsam::Matrix61() << 0, 0, 0, 0, 0, 1).finished();
> ```
>
> **现象**：运行时 GTSAM 抛出矩阵维度不匹配的断言错误，或更糟——悄悄产生错误结果。
>
> **根本原因**：Jacobian $H$ 的维度是 **残差维度 x 变量切空间维度**。残差 1 维 + Pose3 6 维 → $1 \times 6$ 矩阵。
>
> **正确做法**：
> ```cpp
> // Jacobian 维度 = (残差维度) x (变量维度)
> // HeightFactor: 1 x 6
> // BetweenFactor<Pose3>: 6 x 6 (对每个变量)
> // BearingRangeFactor: 2 x 3 (对 Point2 变量)
> ```

> ⚠️ **编程陷阱：忘记检查 optional Jacobian**
>
> **错误做法**：不检查 H 是否为 nullptr 就直接赋值
> ```cpp
> // ❌ 错误：H 可能是 nullptr（优化器有时不需要 Jacobian）
> *H = ...;  // 空指针解引用！段错误！
> ```
>
> **正确做法**：
> ```cpp
> if (H) {
>     *H = ...;
> }
> ```
>
> **为什么 GTSAM 有时不要 Jacobian？** 在纯误差评估（如计算总代价）时不需要导数。`OptionalMatrixType` 的设计允许同一个函数灵活用于两种场景。

### 练习

1. **[编程]** 实现一个 `GPSAltitudeFactor`：一元因子，约束 `Pose3` 的 z 分量等于 GPS 高度测量值。写出 `evaluateError()` 和手写 Jacobian，并用 `numericalDerivative` 验证。

2. **[编程]** 将上面的 `PointToPlaneFactor` 改为 Expression API 实现，对比两种方式的代码量和运行时间。

3. **[思考]** 对于一个连接 3 个变量的因子（如 `NoiseModelFactor3<Pose3, Pose3, Point3>`），`evaluateError()` 需要返回几个 Jacobian 矩阵？每个矩阵的维度是什么？（假设残差是 3 维的。）

---

## 25.6 GTSAM 与其他库集成 ⭐⭐

### 这一节解决什么问题

实际的 SLAM 系统不是只用一个库。你需要 GTSAM 做后端优化，PCL 做点云处理，manif 做李群操作，有时还需要 Ceres 做特定的标定任务。本节讨论 GTSAM 与这些库的集成方法和常见问题。

### GTSAM + manif 适配

GTSAM 和 manif 都定义了 `Pose3`（$SE(3)$），但它们的内部表示略有不同：

| 特性 | GTSAM `Pose3` | manif `SE3d` |
|------|-------------|-------------|
| 底层存储 | `Rot3`（四元数或矩阵）+ `Point3` | $4 \times 4$ 齐次矩阵 |
| 切向量排列 | $[\omega, v]^T$（旋转在前） | $[v, \omega]^T$（平移在前） |
| 扰动约定 | 右扰动 | 右扰动（默认） |
| 四元数顺序 | $(w, x, y, z)$ | $(x, y, z, w)$（Eigen 约定） |

适配代码：

```cpp
#include <gtsam/geometry/Pose3.h>
#include <manif/SE3.h>

// manif → GTSAM
gtsam::Pose3 toGtsam(const manif::SE3d& manif_pose) {
    Eigen::Matrix4d T = manif_pose.transform();
    gtsam::Rot3 R(T.block<3,3>(0,0));
    gtsam::Point3 t(T.block<3,1>(0,3));
    return gtsam::Pose3(R, t);
}

// GTSAM → manif
manif::SE3d toManif(const gtsam::Pose3& gtsam_pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = gtsam_pose.rotation().matrix();
    T.block<3,1>(0,3) = gtsam_pose.translation();
    return manif::SE3d(T);
}

// 切向量转换（注意排列不同！）
gtsam::Vector6 tangentManifToGtsam(const manif::SE3Tangentd& manif_xi) {
    // manif: [v_x, v_y, v_z, omega_x, omega_y, omega_z]
    // GTSAM: [omega_x, omega_y, omega_z, v_x, v_y, v_z]
    gtsam::Vector6 gtsam_xi;
    gtsam_xi << manif_xi.coeffs().tail<3>(),  // omega (旋转)
                manif_xi.coeffs().head<3>();   // v (平移)
    return gtsam_xi;
}
```

### GTSAM + PCL：ICP 结果作为因子

常见模式：用 PCL 的 ICP 配准得到相对变换，然后作为 `BetweenFactor` 添加到 GTSAM 因子图。

```cpp
#include <pcl/registration/icp.h>

// PCL ICP 配准
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setInputSource(source_cloud);
icp.setInputTarget(target_cloud);
pcl::PointCloud<pcl::PointXYZ> aligned;
icp.align(aligned);

if (icp.hasConverged() && icp.getFitnessScore() < threshold) {
    // ICP 结果 → GTSAM Pose3
    Eigen::Matrix4f T_icp = icp.getFinalTransformation();
    gtsam::Pose3 relative_pose(
        gtsam::Rot3(T_icp.block<3,3>(0,0).cast<double>()),
        gtsam::Point3(T_icp.block<3,1>(0,3).cast<double>())
    );

    // ICP fitness score → 噪声模型（越好的配准，噪声越小）
    double fitness = icp.getFitnessScore();
    auto icp_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.05, 0.05, 0.05,
                              fitness, fitness, fitness).finished()
    );

    // 添加为 BetweenFactor
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(i), X(j), relative_pose, icp_noise
    );
}
```

### GTSAM vs Ceres：何时用哪个？

| 场景 | 推荐 | 原因 |
|------|------|------|
| 在线 SLAM 后端 | **GTSAM** | iSAM2 增量优化，原生因子图 |
| 离线 Bundle Adjustment | **Ceres** | 稀疏 Schur 结构优化成熟 |
| 相机-IMU 联合标定 | **Ceres** | Kalibr 用 Ceres，通用性好 |
| LiDAR-Inertial 融合 | **GTSAM** | LIO-SAM、SC-LIO-SAM 的标准选择（注：FAST-LIO2 使用 iEKF，不用 GTSAM） |
| 自定义代价函数原型 | **Ceres** | AutoDiff 更简单直接 |
| 需要边缘化/滑动窗口 | **GTSAM** | 因子图天然支持 |

### GTSAM + ROS

在 ROS 项目中使用 GTSAM 的典型模式：

```cmake
# CMakeLists.txt
find_package(GTSAM REQUIRED)
target_link_libraries(my_node gtsam)
```

**常见问题**：GTSAM 自带一个 Eigen 版本（在 `gtsam/3rdparty/Eigen`），可能与系统 Eigen 或其他库（PCL、OpenCV）使用的版本冲突。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：Eigen 版本冲突**
>
> **现象**：编译时出现大量模板错误，或运行时出现段错误、结果异常。
>
> **根本原因**：GTSAM 默认使用自带的 Eigen（可能是 3.3.x），而 PCL 和 OpenCV 链接系统 Eigen（可能是 3.4.x）。如果两者的 ABI 不兼容，混合链接会导致未定义行为。
>
> **正确做法**：
> ```cmake
> # 编译 GTSAM 时使用系统 Eigen
> cmake -DGTSAM_USE_SYSTEM_EIGEN=ON \
>       -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
>       ..
> ```
>
> **自检方法**：用 `ldd` 检查你的可执行文件链接了哪些 Eigen 相关的共享库，确保只有一个版本。

> ⚠️ **概念误区：把 GTSAM 的 Pose3 和 PCL 的 Eigen::Matrix4f 直接混用**
>
> **错误做法**：忘记 float/double 的转换
> ```cpp
> // ❌ 错误：PCL 返回 Matrix4f，GTSAM 期望 double
> Eigen::Matrix4f T = icp.getFinalTransformation();
> gtsam::Pose3 pose(T);  // 编译错误或精度丢失
> ```
>
> **正确做法**：显式转换
> ```cpp
> Eigen::Matrix4d T_d = T.cast<double>();
> gtsam::Pose3 pose(gtsam::Rot3(T_d.block<3,3>(0,0)),
>                   gtsam::Point3(T_d.block<3,1>(0,3)));
> ```

### 练习

1. **[编程]** 写一个完整的 `manif_gtsam_adapter.h`，包含 `Pose3`、切向量、协方差矩阵的双向转换函数。注意切向量排列和协方差矩阵行列的对应调整。

2. **[思考]** 如果你需要同时使用 GTSAM（后端优化）和 Ceres（LiDAR-camera 外参标定），在同一个 CMake 项目中如何组织依赖？可能会遇到什么编译问题？

---

## 25.7 SLAM 代码精读：LIO-SAM ⭐⭐⭐

### 这一节解决什么问题

理论和 API 都学了，现在来看一个**真实的、工业级的 SLAM 系统**如何使用 GTSAM。LIO-SAM（LiDAR Inertial Odometry via Smoothing and Mapping）是最佳教学案例——它的 `mapOptimization.cpp` 完整展示了因子图的构建、iSAM2 的使用、以及多传感器融合的工程实践。

### LIO-SAM 系统架构

LIO-SAM 的数据流可以分为四个模块：

```text
                    IMU (400Hz)
                       ↓
               [imuPreintegration]
                    ↙      ↘
            IMU 里程计    IMU 预积分
            (实时位姿)    (因子)
                ↓            ↓
           LiDAR (10Hz) → [mapOptimization] → 优化后位姿
                              ↑
                         GPS (1Hz)
                         回环检测
```

`mapOptimization.cpp` 是后端核心，负责管理因子图。它内部维护了一个 `ISAM2` 实例和一个持续增长的因子图。

### 因子图构建流程

以下是 `mapOptimization.cpp` 中因子图构建的完整流程，按代码执行顺序排列：

**Step 1：初始化 iSAM2**

```cpp
// LIO-SAM 的 iSAM2 参数配置
gtsam::ISAM2Params parameters;
parameters.relinearizeThreshold = 0.1;
parameters.relinearizeSkip = 1;
gtsam::ISAM2 isam;
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values isamCurrentEstimate;
```

**Step 2：第一帧——添加先验因子**

当第一个 LiDAR 帧到达时：

```cpp
// 先验因子：固定初始位姿
gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
    gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI,  // 旋转方差 (rad^2)
                              1e-2, 1e-2, 1e-2          // 平移方差 (m^2)
        ).finished()
    );

gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, initialPose, priorNoise));
initialEstimate.insert(0, initialPose);
```

> **注意**：LIO-SAM 直接用整数作为 Key（没有用 `Symbol`），因为它只有位姿变量，不会冲突。在更复杂的系统中应该用 `Symbol`。

**Step 3：每帧——添加 LiDAR 里程计因子**

```cpp
// LiDAR 配准得到的相对变换
gtsam::Pose3 poseFrom = ...;  // 上一帧位姿
gtsam::Pose3 poseTo   = ...;  // 当前帧位姿（由 scan-to-map 配准给出）

gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
    gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6,
                              1e-4, 1e-4, 1e-4).finished()
    );

gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    cloudKeyPoses3D->size() - 1,  // 上一帧 Key
    cloudKeyPoses3D->size(),       // 当前帧 Key
    poseFrom.between(poseTo),      // 相对变换
    odometryNoise
));

initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
```

**Step 4：可选——添加 GPS 因子**

```cpp
if (gpsAvailable) {
    gtsam::noiseModel::Diagonal::shared_ptr gpsNoise =
        gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(3) << gpsCovariance, gpsCovariance,
                                  gpsCovariance * 2  // 高度精度通常较差
            ).finished()
        );

    // GPSFactor 只约束 3D 位置，不约束朝向
    gtSAMgraph.add(gtsam::GPSFactor(
        cloudKeyPoses3D->size(), gpsPosition, gpsNoise
    ));
}
```

**Step 5：可选——添加回环约束因子**

```cpp
if (loopClosureDetected) {
    // ICP 验证回环的相对变换
    gtsam::Pose3 loopTransform = ...;
    float noiseScore = icp.getFitnessScore();

    gtsam::noiseModel::Diagonal::shared_ptr loopNoise =
        gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << noiseScore, noiseScore, noiseScore,
                                  noiseScore, noiseScore, noiseScore
            ).finished()
        );

    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        loopKeyCur, loopKeyPre, loopTransform, loopNoise
    ));
}
```

**Step 6：iSAM2 更新**

```cpp
// 增量更新
isam.update(gtSAMgraph, initialEstimate);
isam.update();  // 额外一次迭代

// 如果有回环，多迭代几次
if (loopClosureDetected) {
    isam.update();
    isam.update();
    isam.update();
}

// 清空临时容器（因子已传入 iSAM2）
gtSAMgraph.resize(0);
initialEstimate.clear();

// 获取最新估计
isamCurrentEstimate = isam.calculateEstimate();

// 提取最新位姿
gtsam::Pose3 latestEstimate =
    isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
```

### 关键实现细节

**1. 关键帧选择**

LIO-SAM 不是每帧都加入因子图——只有当位姿变化超过阈值时才添加新的关键帧：

```cpp
// 距离上一关键帧的位移和旋转
if (abs(roll  - lastRoll)  < surroundingkeyframeAddingAngleThreshold &&
    abs(pitch - lastPitch) < surroundingkeyframeAddingAngleThreshold &&
    abs(yaw   - lastYaw)   < surroundingkeyframeAddingAngleThreshold &&
    sqrt(pow(x-lastX,2) + pow(y-lastY,2) + pow(z-lastZ,2))
        < surroundingkeyframeAddingDistThreshold)
    return;  // 变化太小，不添加关键帧
```

**2. 滑动窗口（变相实现）**

严格来说，LIO-SAM 不做变量边缘化，而是通过关键帧选择控制因子图的增长速度。长时间运行时，因子图会持续增长——这是 LIO-SAM 的一个已知局限。

**3. 噪声模型的动态调整**

LIO-SAM 根据配准质量动态设置噪声：ICP fitness score 越小（配准越好），噪声越小（约束越强）。这种自适应噪声模型是工程中的常见做法。

### 因子图的完整生命周期

将上述步骤串起来，因子图在运行过程中的演化：

```text
帧 0:  [Prior] ── x0

帧 1:  [Prior] ── x0 ──[Odom]── x1

帧 5:  [Prior] ── x0 ──[Odom]── x1 ──[Odom]── ... ──[Odom]── x5
                                                        |
                                                    [GPS] (如果可用)

帧 50: [Prior] ── x0 ── ... ── x50
                  |                |
                  └──[Loop]────────┘  (回环检测到 x0 和 x50 是同一地点)
```

每次 `isam.update()` 后，所有变量的估计值都会更新——回环约束会"拉回"整条轨迹，消除累积漂移。

### ⚠️ 常见陷阱

> ⚠️ **思维陷阱：不理解 iSAM2 就读 LIO-SAM 代码**
>
> **新手想法**："LIO-SAM 的代码不复杂，直接读就好了。"
>
> **实际上**：如果不理解 Bayes Tree 和增量消元，你会把 `isam.update()` 当作黑盒。当系统出问题（如回环后位姿跳变、GPS 约束无效）时，你无法判断是因子图构建的问题还是 iSAM2 内部的问题。
>
> **正确做法**：先理解 25.1-25.3 节的理论，再读 LIO-SAM。重点关注：(a) 每种因子的噪声模型如何设定；(b) `update()` 调用的时机和次数；(c) 初始估计的来源。

> ⚠️ **编程陷阱：LIO-SAM 的 Key 管理方式不可扩展**
>
> **LIO-SAM 做法**：用 `cloudKeyPoses3D->size()` 作为整数 Key
>
> **问题**：如果你扩展系统加入路标变量、IMU 偏差变量等，整数 Key 会冲突。
>
> **正确做法**：在自己的系统中使用 `Symbol`：
> ```cpp
> using gtsam::symbol_shorthand::X;  // 位姿
> using gtsam::symbol_shorthand::V;  // 速度
> using gtsam::symbol_shorthand::B;  // IMU 偏差
> using gtsam::symbol_shorthand::L;  // 路标
> ```

> ⚠️ **概念误区：认为 LIO-SAM 中有两个独立的因子图**
>
> **新手想法**："LIO-SAM 有 `mapOptimization.cpp` 和 `imuPreintegration.cpp`，它们有各自的因子图，互不相关。"
>
> **实际上**：这两个因子图有**不同的角色和生命周期**：
>
> | | `mapOptimization` 因子图 | `imuPreintegration` 因子图 |
> |---|---|---|
> | 作用 | 全局地图优化 | 实时 IMU 里程计 |
> | 变量 | 关键帧位姿 | 最近两帧的状态+偏差 |
> | 持续时间 | 整个运行周期 | 定期重置 |
> | 频率 | ~10Hz（关键帧率） | ~400Hz（IMU 频率） |
>
> `imuPreintegration` 利用 `mapOptimization` 的输出来修正 IMU 偏差估计，这是它们之间的耦合点。

### 练习

1. **[代码阅读]** 克隆 LIO-SAM 仓库（`https://github.com/TixiaoShan/LIO-SAM`），在 `mapOptimization.cpp` 中找到以下函数：`addOdomFactor()`、`addGPSFactor()`、`addLoopFactor()`、`saveKeyFramesAndFactor()`。画出它们的调用关系图。

2. **[编程]** 模拟 LIO-SAM 的因子图构建：生成一个矩形轨迹（100 帧），添加里程计因子（带噪声），在第 100 帧添加到第 1 帧的回环约束，使用 iSAM2 优化。可视化回环前后的轨迹。

3. **[思考]** LIO-SAM 在回环后调用多次 `isam.update()`。如果只调用一次，回环修正会不完整吗？设计实验验证。

---

## 本章小结

| 知识点 | 核心要点 | 对应 GTSAM API |
|--------|---------|---------------|
| 因子图 | 二部图，变量 + 因子，编码联合分布的因式分解 | `NonlinearFactorGraph`, `Values` |
| 变量标识 | `Symbol` 编码字符 + 整数为 `Key` | `symbol_shorthand::X`, `L`, `V`, `B` |
| 先验因子 | 固定单个变量，一元约束 | `PriorFactor<T>` |
| 里程计/回环因子 | 约束两个变量的相对关系 | `BetweenFactor<T>` |
| GPS 因子 | 全局位置约束（只约束平移） | `GPSFactor` |
| 噪声模型 | 对角/各向同性/鲁棒 | `noiseModel::Diagonal`, `Isotropic`, `Robust` |
| 批量优化 | Gauss-Newton / LM | `GaussNewtonOptimizer`, `LevenbergMarquardtOptimizer` |
| **iSAM2** | 基于 Bayes Tree 的增量优化，核心算法 | `ISAM2`, `ISAM2Params` |
| 重线性化 | 变量变化超阈值才重新线性化 | `relinearizeThreshold` |
| IMU 预积分 | 压缩两帧间的 IMU 测量为一个因子 | `PreintegratedImuMeasurements`/`ImuFactor`（LIO-SAM）, `CombinedImuFactor`（含偏差约束） |
| 自定义因子 | 继承 `NoiseModelFactorN`，实现 `evaluateError` | `NoiseModelFactor1/2/3` |
| Expression API | 自动微分替代手写 Jacobian | `Expression<T>`, `ExpressionFactor` |
| **右扰动约定** | $\text{retract}(T, \xi) = T \cdot \text{Exp}(\xi)$ | `traits<Pose3>` |
| Pose3 切向量 | $[\omega_x, \omega_y, \omega_z, v_x, v_y, v_z]^T$ | 旋转在前，平移在后 |

**关键公式汇总**：

$$
\text{MAP 估计}: X^* = \arg\min_X \sum_i \| h_i(X_i) - z_i \|^2_{\Sigma_i}
$$

$$
\text{BetweenFactor 残差}: e(x_i, x_j) = \text{Log}(T_{ij}^{-1} \cdot x_i^{-1} \cdot x_j)
$$

$$
\text{右扰动 retract}: T' = T \cdot \text{Exp}(\xi), \quad \xi \in \mathbb{R}^6
$$

$$
\text{Schur 补}: \Lambda_{\text{new}} = \Lambda_{nn} - \Lambda_{no}\Lambda_{oo}^{-1}\Lambda_{on}
$$

---

## 累积项目：Mini-LIO 因子图后端模块

**本章新增模块**：在 Mini-LIO 项目中添加基于 GTSAM 的因子图后端。

**目标**：

1. 创建 `FactorGraphManager` 类，封装 `ISAM2` 和因子图管理
2. 支持添加先验因子、里程计因子、GPS 因子
3. 实现关键帧选择策略
4. 增量优化并输出优化后的轨迹
5. 预留回环因子接口（下一章实现）

**代码骨架**：

```cpp
class FactorGraphManager {
public:
    FactorGraphManager() {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip = 1;
        isam_ = std::make_unique<gtsam::ISAM2>(params);
    }

    // 添加关键帧（里程计因子 + 可选 GPS）
    void addKeyFrame(const gtsam::Pose3& pose,
                     const gtsam::Pose3& relative_odom,
                     const std::optional<gtsam::Point3>& gps = std::nullopt);

    // 添加回环因子
    void addLoopClosure(int from_idx, int to_idx,
                        const gtsam::Pose3& relative_pose, double fitness);

    // 获取优化后的第 i 个位姿
    gtsam::Pose3 getOptimizedPose(int idx) const;

    // 获取所有优化后的位姿
    std::vector<gtsam::Pose3> getAllPoses() const;

private:
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
    int key_count_ = 0;
};
```

**验证方法**：在 KITTI 数据集的前 100 帧上运行，对比使用/不使用 GPS 约束时的轨迹精度（使用 EVO 工具评估 APE 和 RPE）。

---

## 延伸阅读

### 论文

| 论文 | 内容 | 难度 |
|------|------|------|
| Dellaert & Kaess, "Factor Graphs for Robot Perception", *Found. Trends Robot.*, 2017 | 因子图综述，GTSAM 设计哲学 | ⭐⭐ |
| Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree", *IJRR*, 2012 | iSAM2 算法原始论文 | ⭐⭐⭐ |
| Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", *IEEE T-RO*, 2017 | IMU 预积分理论 | ⭐⭐⭐ |
| Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping", *IROS*, 2020 | LIO-SAM 系统论文 | ⭐⭐ |
| Dellaert, "Factor Graphs and GTSAM: A Hands-on Introduction", Tech Report, 2012 | GTSAM 入门教程 | ⭐ |

### 在线资源

- **GTSAM 官方教程**：`https://gtsam.org/tutorials/intro.html` ⭐
- **GTSAM API 文档**：`https://gtsam.org/doxygen/` ⭐⭐
- **GTSAM by Example**：`https://gtbook.github.io/gtsam-examples/` ⭐
- **LIO-SAM 源码**：`https://github.com/TixiaoShan/LIO-SAM` ⭐⭐
- **GTSAM 用户论坛**：`https://groups.google.com/g/gtsam-users` ⭐⭐

### 源码推荐阅读

| 文件 | 内容 | 行数 |
|------|------|------|
| `gtsam/nonlinear/ISAM2.cpp` | iSAM2 核心算法实现 | ~800 行 |
| `gtsam/slam/BetweenFactor.h` | BetweenFactor 模板实现（学习 evaluateError 写法） | ~150 行 |
| `gtsam/navigation/ImuFactor.cpp` | ImuFactor 实现（LIO-SAM 使用） | ~400 行 |
| `LIO-SAM/src/mapOptimization.cpp` | 完整的因子图管理工程实践 | ~1500 行 |

> 📎 **附录参考**：GTSAM vs g2o 性能对比、Ceres vs GTSAM 使用场景划分、Eigen 版本一致性警告，详见**附录 J：数学库选型对比参考**。

---

## 25.8 GTSAM 工程边界与验证清单 ⭐⭐

> **这一节解决什么问题**：GTSAM 的抽象层比 g2o 更高，容易让人误以为"因子图写出来就一定正确"。本节强调 Key、Values、噪声模型、增量更新和边缘化的工程边界。

### 动机：因子图错误会被 Bayes Tree 缓存下来

回顾 通用库·Ceres：Ceres 的参数块是稳定地址；GTSAM 则把变量存进 `Values`，用 `Key` 标识身份。这个抽象更安全，但也带来新的错误形式：Key 重复、初值缺失、噪声维度不匹配、增量因子没有及时清空。iSAM2 会把线性化结果维护在 Bayes Tree 中，如果错误因子进入系统，后续增量更新可能长期受影响。

如果不做验证会怎样？系统可能不是立刻崩溃，而是 iSAM2 逐渐变慢、某些变量永远不更新、协方差查询失败，或者回环加入后整棵 Bayes Tree 大面积重线性化。

> **本质洞察**：GTSAM 的核心边界是"符号身份 + 概率语义"。Key 决定变量是谁，noise model 决定观测有多可信；两者任何一个错，因子图仍可运行，但概率解释已经失效。

### 因子图构建前的四项检查

| 检查项 | 失败现象 | 推荐做法 |
|--------|----------|----------|
| Key 唯一性 | `Values` 插入失败或覆盖逻辑混乱 | 统一用 `Symbol('x', i)`、`Symbol('v', i)`、`Symbol('b', i)` |
| 初值完整性 | 优化时报 missing key | 每添加新变量同时插入 initial value |
| 噪声维度 | 运行时异常或残差维度不匹配 | `noiseModel::Diagonal::Sigmas(Vector::Constant(dim, sigma))` |
| 因子批次清理 | 同一批因子重复加入 | `isam.update(new_factors, new_values)` 后清空容器 |

这四项检查可以类比数据库事务：Key 是主键，Values 是待写入记录，因子是关系约束，噪声模型是这条关系的置信度。事务提交后，临时缓冲区必须清空，否则同一批数据会被重复提交。

### 增量更新的安全封装

下面的代码展示一个最小的 iSAM2 更新边界：添加因子和初值、提交、清空、再从结果中查询。关键注释说明了对象生命周期和异常边界。

```cpp
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

class IncrementalGraph {
public:
    void addOdometry(int i,
                     const gtsam::Pose3& initial_pose,
                     const gtsam::Pose3& relative,
                     const gtsam::SharedNoiseModel& noise) {
        const auto xi = gtsam::Symbol('x', i);

        if (i == 0) {
            new_values_.insert(xi, initial_pose);
            new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(xi, initial_pose, noise));
        } else {
            const auto x_prev = gtsam::Symbol('x', i - 1);
            new_values_.insert(xi, initial_pose);
            new_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(x_prev, xi, relative, noise));
        }
    }

    void update() {
        // update 可能抛出异常：缺少 Key、噪声维度错误、线性化失败等。
        isam_.update(new_factors_, new_values_);

        // 提交后必须清空增量容器，避免下一次重复添加同一批因子。
        new_factors_.resize(0);
        new_values_.clear();
    }

    gtsam::Pose3 pose(int i) const {
        const auto result = isam_.calculateEstimate();
        return result.at<gtsam::Pose3>(gtsam::Symbol('x', i));
    }

private:
    gtsam::ISAM2 isam_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
};
```

GTSAM 对象通常用值语义和 `shared_ptr` 噪声模型管理内存，避免了 Ceres/g2o 中大量裸指针所有权问题。但这并不意味着没有生命周期边界：`new_factors_` 和 `new_values_` 是增量缓冲区，只应保存尚未提交的数据；`calculateEstimate()` 返回的是当前估计快照，不应被误认为会自动随下一次 `update()` 变化。

### iSAM2 参数的工程含义

| 参数 | 太小/太频繁的后果 | 太大/太稀疏的后果 | 调参建议 |
|------|------------------|-------------------|----------|
| `relinearizeThreshold` | 频繁重线性化，CPU 升高 | 线性化点陈旧，精度下降 | 先用 0.1，再看轨迹误差和耗时 |
| `relinearizeSkip` | 每次都检查，实时性下降 | 延迟更新，回环后收敛慢 | 高频里程计可设 5-10 |
| `factorization` | Cholesky 快但对病态敏感 | QR 稳但更慢 | 病态或退化场景用 QR |
| `enablePartialRelinearizationCheck` | 检查开销变化 | 可能跳过深层变量 | 大图实时系统建议开启测试 |

不要把 iSAM2 理解成"每帧完整优化的快速版"。它的优势来自复用 Bayes Tree 的局部结构；当回环或错误初值导致大范围变量都需要重线性化时，它仍可能接近批量优化成本。

### 边缘化与协方差查询边界

GTSAM 的边缘化和协方差查询有两个常见边界：

1. **边缘化不等于删除变量的所有影响**。被边缘化变量的信息会转化为剩余变量之间的先验，可能使图更稠密。
2. **协方差查询依赖当前线性化点和变量可观测性**。位姿图未固定规范自由度时，边际协方差可能奇异或不可解释。

在 LIO-SAM 这类系统中，通常用 iSAM2 维护全局图，不频繁手动边缘化；滑动窗口 VIO 则更常在 Ceres 中做边缘化先验。选择哪种方式取决于问题规模和实时预算。

### 练习

1. **Key 诊断题**：故意用同一个 `Symbol('x', 0)` 插入两个不同初值，观察 GTSAM 的异常信息。然后设计一个 `KeyFormatter` 打印所有 Key。
2. **增量清理题**：注释掉 `new_factors_.resize(0)` 和 `new_values_.clear()`，运行 10 次更新，观察因子数量和运行时间如何变化。
3. **跨章综合题**：用 通用库·Ceres 的 Ceres 和本章 GTSAM 分别实现同一个 2D 位姿图，比较固定首帧前后的协方差是否可计算。

---

## 25.9 边缘协方差 Marginals 与退化诊断 ⭐⭐⭐

### 这一节解决什么问题

到目前为止，我们只关心 SLAM 后端给出的**点估计**（每个变量的最优值）。但工程中还有一个同等重要的问题：**这个估计有多可信？** 机器人在长走廊里直行 50 米，横向（沿墙方向）的位置可能误差几厘米，而前进方向因为缺乏特征约束误差可能高达数米——点估计本身完全看不出这种差异。下游模块（路径规划的安全裕度、多传感器融合的卡尔曼增益、回环检测的搜索半径、主动 SLAM 的信息增益）都需要知道估计的**不确定性**，也就是后验协方差。本节讲 GTSAM 如何用 `Marginals` 查询边缘协方差，以及当协方差查询失败（`IndeterminantLinearSystemException`）时如何诊断系统退化。

### 动机：点估计不告诉你"哪个方向不确定"

回顾 25.1：MAP 估计最小化 $\sum_i \|h_i(X_i) - z_i\|^2_{\Sigma_i}$。在最优点 $X^*$ 附近，把目标函数做二阶 Taylor 展开，后验近似为一个高斯分布：

$$
p(X \mid Z) \approx \mathcal{N}\!\left(X^*, \; \Lambda^{-1}\right), \qquad \Lambda = \sum_i J_i^\top \Sigma_i^{-1} J_i
$$

其中 $\Lambda$ 是**信息矩阵**（也就是高斯-牛顿近似下的 Hessian），$J_i$ 是因子 $i$ 在 $X^*$ 处对各变量切空间的雅可比。**整个轨迹的联合协方差**是 $\Sigma = \Lambda^{-1}$，但它是一个 $6N \times 6N$ 的稠密大矩阵（$N$ 个位姿），既算不动也存不下。

工程上我们几乎从不需要完整的 $\Sigma$，而是只需要：

- **单变量的边缘协方差** $\Sigma_{x_i}$：变量 $x_i$ 自身的 $6\times 6$ 不确定性块；
- **少数几个变量的联合边缘协方差**：例如回环候选帧 $x_i$ 和当前帧 $x_j$ 的 $12 \times 12$ 联合块，里面包含 $x_i$ 与 $x_j$ 的**互协方差**（cross-covariance）。

> **本质洞察**：边缘协方差不是"把大协方差矩阵切一块出来"那么简单。$\Sigma_{x_i}$ 是 $\Lambda^{-1}$ 的对角块，而 $(\Lambda^{-1})_{ii} \neq (\Lambda_{ii})^{-1}$——除非 $x_i$ 与其余所有变量都不相关。换句话说，单个变量的不确定性**必须**通过整张图的耦合才能正确算出。这正是"边缘化"在概率上的含义：把其余变量积分掉，它们的不确定性会"渗透"进 $x_i$。直接对局部信息块求逆得到的是**条件协方差**（假设其余变量已知），它系统性地偏小、过度自信。

### 如果不查协方差会怎样

- **回环搜索半径写死**：不看协方差，回环检测只能用一个固定的欧氏距离阈值找候选帧。在长走廊里前进方向漂了 3 米，固定半径要么漏掉真回环（半径太小），要么引入大量错误候选（半径太大）。正确做法是用 $x_i$ 与 $x_j$ 的**相对位姿协方差**自适应地确定马氏距离搜索椭球。
- **多传感器融合增益失配**：把 GTSAM 的位姿输出喂给一个外层 EKF 时，如果不传协方差而是拍一个固定的 $R$ 矩阵，滤波器无法判断该信任 SLAM 还是信任新传感器——退化方向上 SLAM 明明很不可信，滤波器却当成精确观测，导致融合结果被拉偏。
- **退化不可见**：在纯走廊、白墙、隧道等场景里，几何约束在某个方向上消失（gauge 在该方向自由）。点估计照样收敛到某个值，但那个方向的协方差应当是巨大的甚至无穷。不查协方差，你根本不知道系统已经退化，直到建图整体漂飞才发现。

### Marginals 的基本用法

GTSAM 用 `Marginals` 类封装协方差查询。它的构造需要因子图和**优化后的** `Values`（线性化点必须是最优点，否则协方差没有意义）：

```cpp
#include <gtsam/nonlinear/Marginals.h>

// 假设已完成批量优化，得到 result
gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
gtsam::Values result = optimizer.optimize();

// ========== 创建 Marginals 对象 ==========
// 第三个参数选择因子化方式：CHOLESKY（默认，快）或 QR（慢但数值稳）
gtsam::Marginals marginals(graph, result, gtsam::Marginals::CHOLESKY);

// ========== 查询单变量边缘协方差 ==========
// 返回 6x6 矩阵（Pose3 的切空间维度）
gtsam::Matrix cov_x5 = marginals.marginalCovariance(X(5));
std::cout << "Covariance of x5:\n" << cov_x5 << std::endl;

// 也可以查信息矩阵（协方差的逆）
gtsam::Matrix info_x5 = marginals.marginalInformation(X(5));
```

`marginalCovariance` 返回的 $6\times 6$ 矩阵，其行列顺序与 GTSAM `Pose3` 的切向量排列一致，即 $[\omega_x,\omega_y,\omega_z,v_x,v_y,v_z]$（回顾 25.2：旋转在前、平移在后）。因此左上 $3\times 3$ 是姿态不确定性（单位 $\text{rad}^2$），右下 $3\times 3$ 是平移不确定性（单位 $\text{m}^2$）。**这一点极易看错**：很多人想当然以为前三维是平移。

> **本质洞察**：`marginalCovariance` 返回的协方差是定义在**右扰动切空间**上的，不是定义在某个固定全局参考系里的。它度量的是"绕当前估计 $T^*$ 的右扰动 $\xi$ 的二阶矩"，即 $\Sigma = \mathbb{E}[\xi\xi^\top]$ 其中 $T = T^* \cdot \mathrm{Exp}(\xi)$。这与 25.2 的右扰动约定、25.5 自定义因子里"对 $\xi$ 求雅可比"是同一套坐标——三者必须用同一个扰动定义，协方差才自洽。如果你把它当成欧氏空间的位置方差直接画椭圆，在大旋转下会错位。

### 联合边缘协方差与互协方差

回环检测、相对位姿一致性检验需要的是**两个变量的联合分布**。单独查 $\Sigma_{x_i}$ 和 $\Sigma_{x_j}$ 拿不到它们之间的相关性——而相对位姿 $x_i^{-1}x_j$ 的不确定性恰恰依赖于互协方差 $\Sigma_{ij}$：

$$
\Sigma_{\text{rel}} \approx J_i \, \Sigma_{ii} \, J_i^\top + J_j \, \Sigma_{jj} \, J_j^\top + J_i \, \Sigma_{ij} \, J_j^\top + J_j \, \Sigma_{ji} \, J_i^\top
$$

如果忽略后两项互协方差，对于沿同一条轨迹的两帧（高度相关），$\Sigma_{\text{rel}}$ 会被严重高估——把本来很确定的相对关系当成很不确定，回环搜索椭球被无谓地撑大。

```cpp
// ========== 查询联合边缘协方差 ==========
gtsam::KeyVector keys{X(2), X(8)};
gtsam::JointMarginal joint = marginals.jointMarginalCovariance(keys);

// JointMarginal 按 (行变量, 列变量) 取块
gtsam::Matrix cov_22 = joint(X(2), X(2));  // x2 的 6x6 边缘块
gtsam::Matrix cov_88 = joint(X(8), X(8));  // x8 的 6x6 边缘块
gtsam::Matrix cov_28 = joint(X(2), X(8));  // x2-x8 的 6x6 互协方差块（一般非零、非对称）

// 拼成完整的 12x12 联合协方差
gtsam::Matrix fullJoint = joint.fullMatrix();
```

> ⚠️ **概念误区：联合边缘 ≠ 两次单变量边缘拼起来**
>
> **新手想法**："`jointMarginalCovariance({X(2),X(8)})` 的对角块，和分别调两次 `marginalCovariance` 应该一样，所以联合查询只是多给了个互协方差块。"
>
> **实际上**：两者的对角块在数学上确实相等（都是各自的边缘协方差），但**联合查询额外给出的互协方差块是单变量查询永远拿不到的**，而它正是相对不确定性计算的核心。把两个独立查询的结果当成"对角块 + 零互协方差"来用，等价于假设 $x_2$ 与 $x_8$ 不相关——对同一轨迹上的两帧这个假设严重错误。
>
> **正确做法**：凡是需要"两个变量之间相对量"的不确定性，一律用 `jointMarginalCovariance` 并取出互协方差块。

### iSAM2 中的协方差查询

在线 SLAM 里我们用 iSAM2 而不是批量优化。iSAM2 自身就提供 `marginalCovariance`，它直接复用 Bayes Tree 的因子化结果，**不需要重新构造 `Marginals`**：

```cpp
// iSAM2 内部已维护 Bayes Tree 的 R 因子，可直接查
gtsam::Matrix cov = isam2.marginalCovariance(X(currentKey));
```

这与对"iSAM2 当前因子 + 当前线性化点"构造一个 `Marginals` 得到的结果一致——因为 Bayes Tree 本质就是 $\Lambda = R^\top R$ 的 Cholesky 因子（回顾 25.3），从 $R$ 回代即可得到协方差列，无需重新分解。

> ⚠️ **思维陷阱：以为 iSAM2 取协方差和取点估计一样廉价**
>
> **新手想法**："`calculateEstimate()` 很快，那 `marginalCovariance()` 也应该是 $O(1)$ 的。"
>
> **实际上**：取点估计只需从 Bayes Tree 根到叶做一次回代。取某个变量的边缘协方差需要沿着该变量在树中到**根**的路径回代求解多列，代价与该变量的"深度"和相关 clique 大小相关。频繁对**所有**变量查协方差（例如每帧给上千个历史位姿都算一遍）会成为新的瓶颈。
>
> **正确做法**：只对真正需要的变量查协方差（当前帧、回环候选帧）。需要可视化整条轨迹的不确定性时，降采样或离线计算。

### 退化诊断：IndeterminantLinearSystemException

当系统**规范自由度（gauge freedom）未被固定**，或某个方向上完全没有约束时，信息矩阵 $\Lambda$ 奇异，Cholesky 分解失败，GTSAM 抛出 `IndeterminantLinearSystemException`，异常里会带上**触发问题的变量 Key**。这是 GTSAM 工程中最常见、也最让初学者困惑的崩溃之一。

**根本原因可分两类**，必须分开诊断：

| 类别 | 物理含义 | 典型场景 | 修复方向 |
|------|----------|----------|----------|
| **规范自由度未固定** | 整张图可以整体平移/旋转而代价不变（零空间维度 = 6 或 4） | 只加了 `BetweenFactor`，一个 `PriorFactor` 都没有 | 加一个 prior 钉住首帧（或任意一帧） |
| **结构退化** | 某变量在某方向上没有任何约束（信息为零） | 走廊里前进方向无特征；某路标只被一帧观测到（深度不可观） | 补充约束（IMU、GPS、轮速）或将该方向设为弱先验 |

```cpp
try {
    gtsam::Matrix cov = marginals.marginalCovariance(X(k));
} catch (const gtsam::IndeterminantLinearSystemException& e) {
    // e.nearbyVariable() 返回触发奇异的变量 Key —— 诊断的起点
    std::cerr << "Indeterminate system near key: "
              << gtsam::DefaultKeyFormatter(e.nearbyVariable()) << std::endl;
    // 排查：该变量是否缺少 prior？是否只被一个因子约束？
}
```

> **本质洞察**：`IndeterminantLinearSystemException` 不是"GTSAM 的 bug"，而是**它在替你阻止一个数学上无解的问题**。位姿图若不固定规范自由度，"最优位姿"根本不唯一（有无穷多个仅差一个全局刚体变换的等价解），协方差在该方向上是无穷大。Ceres 在同样情况下往往**不报错**而是默默返回一个由初值偶然决定的解，反而更危险。GTSAM 选择显式抛异常，是把"不可观测"这个本来隐蔽的建模错误暴露在你面前。

**诊断流程**（按顺序排查）：

1. **查异常 Key**：`e.nearbyVariable()` 指出哪个变量触发奇异，用 `DefaultKeyFormatter` 打印成可读形式（如 `x0`）。
2. **检查是否有 prior**：整张图至少需要一个一元因子固定规范。位姿图缺 prior 是最常见原因。
3. **检查该变量的因子数**：用图遍历统计连到该 Key 的因子数。只有 1 个因子的位姿、只被 1 帧观测的路标，通常不可观测。
4. **切到 QR 因子化复算**：`Marginals(graph, result, Marginals::QR)`。QR 对接近奇异的系统更稳，若 QR 能算出而 Cholesky 失败，说明系统**接近但未完全**退化（病态），提示约束太弱而非完全缺失。
5. **检查零空间维度**：对小问题可直接对 $\Lambda$ 做 SVD，零奇异值的个数就是不可观测方向数（位姿图未固定时通常为 6，平面 SLAM 为 3）。

### 与 Ceres 的对比

| 维度 | Ceres Solver | GTSAM `Marginals` / iSAM2 |
|------|-------------|---------------------------|
| 协方差接口 | `Covariance` 类，需显式指定变量对并 `Compute()` | `Marginals::marginalCovariance(key)` 一行查询 |
| 增量场景 | 每次重算（无增量协方差） | iSAM2 复用 Bayes Tree 因子，增量回代 |
| 坐标约定 | 取决于用户的 `LocalParameterization`/`Manifold` | 统一右扰动切空间（与因子雅可比一致） |
| 退化处理 | 默认静默返回，需手动检查 rank | 显式抛 `IndeterminantLinearSystemException` 并指出 Key |
| 数值选项 | `SPARSE_QR` / `DENSE_SVD` 等 | `Marginals::CHOLESKY` / `QR` |

**一句话总结**：Ceres 让你"自己决定要不要算协方差、自己检查退化"；GTSAM 把协方差查询和退化检测做成了一等公民，代价是你必须理解它的右扰动坐标约定，否则会把切空间协方差误读成欧氏方差。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：在未优化的 Values 上构造 Marginals**
>
> **错误做法**：用 `initial`（初始估计）而不是 `result`（优化结果）构造 `Marginals`
> ```cpp
> // ❌ 错误：协方差在非最优点没有意义
> gtsam::Marginals marginals(graph, initial);
> ```
>
> **现象**：协方差数值看起来"能算出来"，但完全错误——它是在一个随意的线性化点上算的，既不是后验协方差，也无法解释。
>
> **根本原因**：协方差 $\Lambda^{-1}$ 来自在**最优点** $X^*$ 处的二阶展开。在非最优点，梯度非零，二阶近似不成立，$\Lambda$ 也不是真正的后验信息矩阵。
>
> **正确做法**：先 `optimizer.optimize()` 得到 `result`，再 `Marginals(graph, result)`。

> ⚠️ **概念误区：把切空间协方差当欧氏位置方差画椭圆**
>
> **新手想法**："`marginalCovariance(X(i))` 的右下 $3\times 3$ 块就是位置 $(x,y,z)$ 的方差，可以直接画 $3\sigma$ 椭球。"
>
> **实际上**：那 $3\times 3$ 块是**右扰动平移分量** $v$ 的协方差，定义在 $T^*$ 的局部坐标系里，不是全局位置 $t$ 的协方差。两者通过伴随矩阵相关：全局位置扰动 $\delta t = R^* \, v$，所以全局位置协方差是 $R^* \, \Sigma_{vv} \, R^{*\top}$。
>
> **后果**：在大姿态角下直接画局部块，椭球朝向会错。对小旋转或只关心量级时影响不大，但严格可视化必须做伴随变换。
>
> **正确做法**：需要全局坐标椭球时，用 $R^*$ 把平移块旋转过去；或直接对感兴趣的几何量（如 2D 位置）写一个投影函数，用其雅可比传播协方差。

> ⚠️ **思维陷阱：协方差越小越好**
>
> **新手想法**："优化后某变量协方差非常小，说明估计非常准，系统很健康。"
>
> **实际上**：协方差**异常小**往往是危险信号——通常意味着某个因子的噪声模型 sigma 设得过小（过度自信），把本不该那么强的约束当成了铁律。这会让 iSAM2 在该变量上拒绝任何修正，回环来了也拉不动（回顾 25.8 工程边界）。
>
> **正确做法**：协方差要和传感器的真实精度对齐，而不是越小越好。怀疑过度自信时，对照传感器 datasheet 反查 sigma 设置。

### 练习

1. **[编程]** 在 25.2 节的 2D 位姿图上，优化后用 `Marginals` 查询每个位姿的 $3\times 3$ 边缘协方差，打印其行列式（不确定性体积）。验证"离 prior 越远的位姿协方差越大"，以及"加入回环因子后所有协方差整体减小"。

2. **[编程]** 构造一个只有 `BetweenFactor`、没有任何 `PriorFactor` 的位姿链，调用 `marginalCovariance`，捕获 `IndeterminantLinearSystemException` 并打印 `nearbyVariable()`。然后加一个 prior，确认异常消失。

3. **[思考]** 对回环候选帧 $x_i$ 和当前帧 $x_j$，写出用 `jointMarginalCovariance` 计算相对位姿 $x_i^{-1}x_j$ 协方差的完整步骤（含所需雅可比 $J_i, J_j$）。解释为什么忽略互协方差块会高估相对不确定性，进而影响回环搜索椭球的大小。

---

## 25.10 鲁棒优化与 GNC ⭐⭐⭐

### 这一节解决什么问题

回环检测会出错。视觉/激光的特征匹配会产生误匹配。GPS 在城市峡谷里会有多径跳变。这些**外点（outlier）** 一旦作为因子进入图，普通最小二乘会被它们带偏——因为二次代价对大残差的惩罚是平方增长的，一个错误回环可以把整条轨迹扭曲（回顾"如果跳过本章会怎样"的场景二）。25.2 介绍过 `noiseModel::Robust` 鲁棒核，本节系统地讲鲁棒优化：M-估计鲁棒核的原理与局限，以及 GTSAM 的 `GncOptimizer`（Graduated Non-Convexity，渐进非凸）如何在初值很差时仍能正确剔除外点。

### 动机：一个外点污染整张图

设想 100 个正确回环 + 1 个错误回环。错误回环的残差很大，在二次代价下它贡献的代价 $\propto \|e\|^2$ 可能超过其余所有因子之和。优化器为了"照顾"这个最大的代价项，会把相关位姿硬拉向错误方向，结果 100 个正确约束被一个错误约束牺牲掉。

$$
\text{二次代价}: \rho(e) = \tfrac12 e^2 \;\Rightarrow\; \frac{\partial \rho}{\partial e} = e \quad(\text{残差越大，拉力越大，无上限})
$$

这就是为什么裸最小二乘对外点**零容忍**。解决思路有两条主线：**M-估计鲁棒核**（改造代价函数，给大残差"封顶"）和 **GNC**（先把问题凸化求一个不被外点带偏的解，再逐步恢复非凸性以锐利地区分内外点）。

### 主线一：M-估计鲁棒核

M-估计把二次代价 $\tfrac12 e^2$ 换成一个增长更慢的鲁棒核 $\rho(e)$，使得大残差的梯度（"拉力"）被压制甚至归零。GTSAM 在 `noiseModel::mEstimator` 下提供了多种核：

| 鲁棒核 | $\rho$ 的大残差行为 | 影响函数 $\psi=\rho'$ | 特点 |
|--------|--------------------|----------------------|------|
| `Huber` | 大残差处线性增长 | 饱和到常数 | 温和，大残差仍有恒定拉力，适合"轻度"外点 |
| `Cauchy` | 对数增长 | 趋于 0 | 较激进，远外点拉力趋零 |
| `GemanMcClure` | 趋于常数上限 | 快速趋 0 | 强力压制远外点 |
| `Tukey` | 超阈值后完全平坦 | 超阈值**恒为 0** | 最激进，远外点拉力**精确为零**（完全忽略） |
| `DCS` | 动态协方差缩放 | —— | 等价于对信息矩阵动态降权 |

```cpp
#include <gtsam/linear/NoiseModel.h>

// 用 Huber 核包裹基础噪声模型
auto base = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2).finished());
auto robust = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(1.345),  // 1.345 → 95% 渐近效率
    base);

// 给可能是外点的回环因子套上鲁棒核
graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    X(i), X(j), loopMeasurement, robust);
```

**鲁棒核的工作机制**（IRLS 视角）：GTSAM 在每次线性化时，把鲁棒核等价转换为对该因子的一个**权重** $w(e) = \psi(e)/e$，残差越大权重越小。于是非线性鲁棒优化变成一系列"加权最小二乘"——这就是**迭代重加权最小二乘（Iteratively Reweighted Least Squares, IRLS）**。Huber 的权重从 1 平滑降到 $\propto 1/|e|$，Tukey 的权重在超阈值后直接归零。

> **本质洞察**：鲁棒核不是"另一种噪声模型"，而是**对每个因子做了自适应降权**。M-估计的全部魔法都在影响函数 $\psi(e)=\rho'(e)$ 上：$\psi$ 在大残差处是否饱和（Huber）还是归零（Tukey），决定了外点是"被削弱"还是"被彻底踢出"。理解这一点，你就能根据外点的严重程度选核，而不是照抄默认值。

> ⚠️ **思维陷阱：鲁棒核能解决一切外点问题**
>
> **新手想法**："套上 Huber 或 Cauchy 核，外点就自动被处理了，初值差也没关系。"
>
> **实际上**：M-估计鲁棒核**强烈依赖初值**。鲁棒代价函数是非凸的，有多个局部极小。如果初值已经被外点带到错误盆地里，IRLS 会收敛到那个错误解——此时鲁棒核反而"锁死"了错误：它给当前看起来残差小的（其实是错误的）因子高权重，给当前残差大的（其实是正确的）因子低权重。
>
> **后果**：回环外点比例高、或里程计初值漂移大时，单纯加鲁棒核可能不收敛到正确解，且失败时毫无征兆。
>
> **正确做法**：初值可能很差、外点比例高时，用下面的 GNC——它先求凸化解避开局部极小，再逐步恢复非凸性。

### 主线二：Graduated Non-Convexity（GNC）

GNC 的核心思想：**鲁棒代价非凸，难优化；但它可以被一族带控制参数 $\mu$ 的"代理函数"逼近**。$\mu$ 取一个极端值时代理函数（近似）凸，容易求全局解；然后逐步把 $\mu$ 调向另一端，代理函数连续地变形回原始的非凸鲁棒代价。每一步都用上一步的解作为初值，于是解被"牵引"着从凸问题的全局最优，平滑地过渡到非凸问题的正确极小，从而避开被外点造成的坏局部极小。

```text
GNC 控制参数 μ 的演化（以 TLS 为例）：

μ 初始（接近凸）        μ 中间               μ 终止（恢复非凸/锐利）
代价 ≈ 二次             代价开始"封顶"        代价 = 真正的 TLS
所有因子近似等权    →   外点权重开始下降   →   内点权重→1，外点权重→0
求得不被外点带偏的解     解被持续牵引          内外点被干净二分
```

GTSAM 的 `GncOptimizer` 把这套流程封装好了。它在内部对每个因子维护一个权重 $w_i \in [0,1]$，随 $\mu$ 演化迭代更新；终止时内点权重趋于 1、外点趋于 0，相当于自动完成了一次**全局外点剔除**。

```cpp
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

// GncParams 包裹一个基础优化器参数（这里用 LM）
using GncLMParams = gtsam::GncParams<gtsam::LevenbergMarquardtParams>;

GncLMParams gncParams;
gncParams.setLossType(gtsam::GncLossType::TLS);  // TLS（截断最小二乘）或 GM
gncParams.setMaxIterations(20);                  // GNC 外层迭代上限
// 已知一定是内点的因子索引（如 prior、里程计），可固定为内点不参与剔除
// gncParams.setKnownInliers(knownInlierIndices);

// 用置信度反推内点代价阈值（按 chi-square 分布；需先指定残差维度）
gncParams.setInlierCostThresholdsAtProbability(0.99);

// 构造 GNC 优化器：模板参数是基础优化器的参数类型
gtsam::GncOptimizer<GncLMParams> gnc(graph, initial, gncParams);
gtsam::Values result = gnc.optimize();

// optimize() 内部完成 μ 的渐进演化 + 每步加权 LM 求解
```

**TLS vs GM 怎么选**：

| 损失 | 行为 | 适用 |
|------|------|------|
| `GncLossType::TLS`（截断最小二乘） | 内点严格二次、外点严格归零，二分干净 | 外点/内点界限清晰，追求硬剔除 |
| `GncLossType::GM`（Geman-McClure） | 权重平滑过渡，无硬截断 | 内外点边界模糊，需要软降权 |

> **本质洞察**：GNC 与 M-估计的根本区别在于**对初值的依赖**。M-估计在固定的非凸代价上做 IRLS，初值决定落入哪个盆地；GNC 用 $\mu$ 把优化路径从一个凸问题出发，**主动控制**它落入正确盆地。代价是更多的迭代和计算——GNC 是"用算力换鲁棒性"。这也解释了它的定位：离线/准实时的位姿图优化、回环验证；而强实时前端仍倾向用便宜的 Huber + 几何预筛。

> ⚠️ **概念误区：GNC 不需要噪声模型，可以替代鲁棒核**
>
> **新手想法**："用了 GncOptimizer 就不用设噪声模型了，它会全自动处理。"
>
> **实际上**：GNC **仍然需要每个因子的基础噪声模型**——它据此计算加权残差，并用 `setInlierCostThresholdsAtProbability` 把"多大的加权残差算外点"换算成内点阈值。噪声模型设错（如 sigma 过小），内点阈值就错，GNC 会把正确因子误判为外点。GNC 是在噪声模型之上做外点剔除，不是替代它。
>
> **正确做法**：先把基础噪声模型按传感器精度设对，再用 GNC 处理"超出该精度的"外点。

### 鲁棒优化方案选型

把两条主线和"几何预筛"放在一起，给出工程决策：

| 方案 | 何时用 | 优点 | 代价/局限 |
|------|--------|------|-----------|
| 几何/一致性预筛（不进优化器） | 所有场景的第一道防线 | 最便宜，直接拒掉明显错误回环 | 只能挡明显外点 |
| Huber / Cauchy 鲁棒核 | 初值较好、外点温和 | 实时、实现简单、单次优化 | 依赖初值，非凸局部极小 |
| GNC（TLS/GM） | 初值差、外点比例高、可离线 | 不依赖初值，全局剔除外点 | 计算量大，偏离强实时 |
| Switchable Constraints / Dynamic Covariance Scaling | 位姿图回环鲁棒化 | 把开关变量并入优化 | 增加变量与调参复杂度 |

> **本质洞察**：鲁棒性不是单一开关，而是**分层防御**——前端几何预筛挡掉明显错误，中间噪声模型 + 鲁棒核处理常规外点，后端 GNC 兜底处理初值差或高外点比例的硬场景。任何一层都不足以独当一面；真实系统（如带回环的 LIO/VIO）是三层叠加。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：对 prior 和里程计也套强鲁棒核**
>
> **错误做法**：图里所有因子统一套 Tukey/TLS
> ```cpp
> // ❌ 错误：连 prior 都用强截断核
> graph.addPrior(X(0), pose0, robustTukeyNoise);
> ```
>
> **现象**：求解不稳定甚至发散——规范由 prior 提供，但强鲁棒核在残差稍大时把 prior 权重也压到接近 0，等于偷偷移除了 prior，系统退化（回到 25.9 的 `IndeterminantLinearSystemException`）。
>
> **根本原因**：鲁棒核是为"可能是外点"的观测设计的。prior 和高可信里程计**不是外点**，给它们加强鲁棒核反而破坏了图的可观测性骨架。
>
> **正确做法**：只对可能含外点的因子（回环、远距离观测、GPS）加鲁棒核；prior、相邻里程计用普通高斯噪声。用 GNC 时把这些可信因子通过 `setKnownInliers` 固定为内点。

> ⚠️ **思维陷阱：把 GNC 当成强实时前端的默认优化器**
>
> **新手想法**："GNC 这么鲁棒，干脆每帧都用它替代 iSAM2。"
>
> **实际上**：GNC 的 $\mu$ 渐进需要多轮重优化，单次代价远高于一次 iSAM2 增量更新；它本质是**批量**鲁棒优化，不是增量算法。每帧跑 GNC 会彻底破坏实时性。
>
> **正确做法**：前端用 iSAM2 增量 + 轻量鲁棒核保实时；GNC 用于回环验证、地图后处理、或周期性的全局位姿图鲁棒重优化。

### 练习

1. **[编程]** 在一个带 100 个正确回环的 2D 位姿图里，人为注入 5 个错误回环（随机相对位姿）。分别用 (a) 纯高斯噪声、(b) Huber 核、(c) `GncOptimizer`(TLS) 优化，对比最终轨迹误差（APE）。验证 GNC 在高外点比例下显著优于前两者。

2. **[实验]** 对上题，逐渐增大错误回环的比例（5%→20%→40%），记录 Huber 和 GNC 各自开始失效的临界点。解释为什么 GNC 的临界点更高。

3. **[思考]** `setInlierCostThresholdsAtProbability(0.99)` 内部用 chi-square 分布把置信度换算成内点代价阈值。为什么这个阈值依赖**因子残差的维度**？对 `Pose3` 的 6 维回环因子和 `GPSFactor` 的 3 维因子，同一置信度对应的阈值是否相同？

---

> **下一章预告**：SLAM库·g2o 将学习 g2o——另一个广泛使用的图优化库。g2o 采用不同于 GTSAM 的设计哲学（直接操作图结构而非因子图），在 ORB-SLAM、RTAB-Map 等系统中被大量使用。我们将对比 GTSAM 和 g2o 的优劣，帮助你在实际项目中做出正确的库选型决策。

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|----------|----------|----------|
| `ValuesKeyDoesNotExist` 或 missing key | 添加了因子但没有为新变量插入初值 | 1. 打印因子涉及的 Key；2. 检查 `new_values.exists(key)`；3. 确保每个新变量先插入初值 | SLAM库·GTSAM |
| iSAM2 运行越来越慢 | 增量因子/初值没有清空，或回环导致大范围重线性化 | 1. 每次 update 后打印新增因子数；2. 清空 `new_factors_`/`new_values_`；3. 检查 `relinearizeThreshold` | SLAM库·GTSAM |
| IMU 预积分结果漂移异常 | 重力方向、噪声单位、bias 更新或时间间隔错误 | 1. 打印每段 `dt`；2. 确认加速度单位是 m/s²；3. 每次 bias 更新后 reset 预积分 | 通用库·李群manif, SLAM库·GTSAM |
| 边际协方差查询失败或数值巨大 | 未固定规范自由度，信息矩阵奇异 | 1. 添加 prior factor；2. 检查图连通性；3. 对比固定首帧前后的 marginal covariance | 通用库·Ceres, SLAM库·GTSAM |
| 回环加入后轨迹整体扭曲 | 回环相对位姿错误或噪声设得过小 | 1. 先用几何验证过滤回环；2. 增大回环噪声；3. 打印回环因子优化前残差 | SLAM库·GTSAM, 通用库·PCL |
| `IndeterminantLinearSystemException`（抛异常并指向某 Key） | 规范自由度未固定（缺 prior），或某变量在某方向不可观测 | 1. 用 `e.nearbyVariable()` 定位 Key；2. 确认至少有一个 prior；3. 统计该 Key 的因子数；4. 切 `Marginals::QR` 复算判断是病态还是全奇异 | SLAM库·GTSAM §25.9 |
| 协方差画出的不确定性椭球朝向错误 | 把右扰动切空间协方差当成全局欧氏位置方差 | 1. 确认取的是右下 $3\times3$ 平移块；2. 用 $R^*\Sigma_{vv}R^{*\top}$ 转到全局系；3. 大旋转下尤其明显 | SLAM库·GTSAM §25.9, 通用库·李群manif |
| 加了鲁棒核仍被错误回环带偏 | 初值差落入坏盆地，M-估计锁死错误解 | 1. 先做几何/一致性预筛；2. 改用 `GncOptimizer`(TLS) 避开局部极小；3. 把 prior/里程计设为 `setKnownInliers` | SLAM库·GTSAM §25.10 |

---

## 本章常见误解汇总

把全章散落在各节的概念误区集中成一张表，便于复习前快速自检。每一条左边是初学者的典型错误认知，右边是正确理解（详细分析见对应小节）。

| 常见误解 | 正确理解 | 对应节 |
|---------|---------|--------|
| 因子图就是马尔可夫随机场/贝叶斯网络，可互换 | 因子图是更精细的二部图表示，显式区分每个因子；信息量 因子图 > MRF > 贝叶斯网络，转换会丢信息 | 25.1 |
| 边缘化是无损的，信息全保留在 Schur 补里 | 仅在线性高斯下无损；非线性 SLAM 中边缘化会固化线性化点，后续大偏离会引入误差 | 25.1 |
| `Pose3` 切向量是 $[v,\omega]$（平移在前） | GTSAM `Pose3` 是 $[\omega,v]$（旋转在前），与 Sophus 相反；`Pose2` 又是 $[v_x,v_y,\theta]$ | 25.2 |
| 用裸整数当 Key 没问题 | 多类型变量混用裸整数会 Key 冲突；应统一用 `Symbol`/`symbol_shorthand` | 25.2 / 25.8 |
| iSAM2 每帧给出的解和批量 LM 完全一致 | iSAM2 用延迟重线性化换速度，两次重线性化之间是近似解，差异由 `relinearizeThreshold` 控制 | 25.3 |
| LIO-SAM 使用 `CombinedImuFactor` | LIO-SAM 用 `ImuFactor`(5 路) + 单独的偏差 `BetweenFactor`，便于解耦调参 | 25.4 |
| 自定义因子的 Jacobian 维度是"变量维度 × 残差维度" | 是"残差维度 × 变量切空间维度"；如 `Pose3` 上的 1 维约束 Jacobian 是 $1\times 6$ | 25.5 |
| 单变量边缘协方差 = 对局部信息块求逆 | 那是条件协方差（过度自信）；边缘协方差是 $\Lambda^{-1}$ 的对角块，须经整图耦合 | 25.9 |
| `marginalCovariance` 右下 $3\times3$ 块是全局位置方差 | 是右扰动平移分量协方差，定义在局部系；全局需 $R^*\Sigma_{vv}R^{*\top}$ | 25.9 |
| 协方差越小越好 | 异常小常是噪声 sigma 设得过小（过度自信），会让 iSAM2 拒绝修正、回环拉不动 | 25.9 |
| 鲁棒核能解决一切外点、初值差也没关系 | M-估计依赖初值，坏初值会锁死错误解；高外点比例/差初值需 GNC | 25.10 |
| 用了 `GncOptimizer` 就不需要噪声模型 | GNC 仍需基础噪声模型计算加权残差并定阈值；它是在噪声模型之上做外点剔除 | 25.10 |

---

## API 速查表

本章涉及的 GTSAM 核心 API 签名与一句话说明，按主题分组。完整签名以 GTSAM 4.x 官方 Doxygen 为准。

**容器与变量标识**

| API | 说明 |
|-----|------|
| `gtsam::NonlinearFactorGraph` | 因子（约束）容器；`.add(factor)` / `.emplace_shared<F>(...)` / `.addPrior(key, val, noise)` |
| `gtsam::Values` | 变量估计字典（Key→Value）；`.insert(key, val)` / `.at<T>(key)` / `.exists(key)` / `.update(key,val)` |
| `gtsam::Symbol('x', i)` / `symbol_shorthand::X(i)` | 把字符 + 整数编码为唯一 `Key`，避免多类型变量冲突 |
| `gtsam::DefaultKeyFormatter` | 把 `Key` 打印成可读形式（如 `x0`），调试与异常定位用 |

**因子类型**

| API | 说明 |
|-----|------|
| `gtsam::PriorFactor<T>` | 一元先验因子，固定单个变量；位姿图必须至少有一个以固定规范 |
| `gtsam::BetweenFactor<T>` | 二元相对约束；里程计/回环用；残差 $\mathrm{Log}(T_{ij}^{-1}x_i^{-1}x_j)$ |
| `gtsam::BearingRangeFactor<P,L>` | 方位-距离观测因子 |
| `gtsam::GPSFactor` | 全局 3D 位置约束（只约束平移，不约束朝向） |
| `gtsam::ImuFactor` / `gtsam::CombinedImuFactor` | IMU 预积分因子（5 路 / 6 路含偏差约束） |
| `gtsam::NoiseModelFactor1/2/3<...>` | 自定义因子基类，重写 `evaluateError(..., OptionalMatrixType H)` |

**噪声模型**

| API | 说明 |
|-----|------|
| `noiseModel::Diagonal::Sigmas(v)` / `::Variances(v)` | 对角噪声，按标准差 / 方差给定 |
| `noiseModel::Isotropic::Sigma(dim, σ)` | 各向同性噪声 |
| `noiseModel::Constrained` | 硬约束（$\sigma\to 0$） |
| `noiseModel::Robust::Create(mEstimator, base)` | 鲁棒核包裹基础噪声模型 |
| `noiseModel::mEstimator::Huber/Cauchy/GemanMcClure/Tukey/DCS` | 各类 M-估计鲁棒核 |

**优化器**

| API | 说明 |
|-----|------|
| `GaussNewtonOptimizer(graph, values, params)` | 高斯-牛顿批量优化，收敛快需好初值 |
| `LevenbergMarquardtOptimizer(graph, values, params)` | LM 批量优化，带阻尼更鲁棒 |
| `ISAM2(params)` + `.update(newFactors, newValues)` + `.calculateEstimate()` | 增量优化，在线 SLAM 核心 |
| `ISAM2Params`: `relinearizeThreshold` / `relinearizeSkip` / `factorization` | iSAM2 关键调参 |
| `GncOptimizer<GncParams<LMParams>>(graph, init, gncParams)` + `.optimize()` | GNC 鲁棒批量优化（全局外点剔除） |
| `GncParams`: `setLossType(GncLossType::TLS/GM)` / `setMaxIterations` / `setKnownInliers` / `setInlierCostThresholdsAtProbability` | GNC 调参 |

**预积分与导航状态**

| API | 说明 |
|-----|------|
| `PreintegrationParams::MakeSharedU()` / `MakeSharedD()` | IMU 参数，Z 轴朝上（ENU）/ 朝下（NED） |
| `PreintegratedImuMeasurements`: `integrateMeasurement(acc,gyro,dt)` / `predict(state,bias)` / `resetIntegrationAndSetBias(bias)` | 预积分器主接口 |
| `gtsam::NavState` | 封装位姿 + 速度的导航状态 |
| `gtsam::imuBias::ConstantBias` | IMU 偏差变量类型 |

**协方差与几何**

| API | 说明 |
|-----|------|
| `Marginals(graph, result, Marginals::CHOLESKY/QR)` | 在优化结果上构造边缘协方差查询器 |
| `Marginals::marginalCovariance(key)` / `marginalInformation(key)` | 单变量边缘协方差 / 信息矩阵（右扰动切空间坐标） |
| `Marginals::jointMarginalCovariance(keys)` → `JointMarginal` | 多变量联合边缘协方差，含互协方差块 |
| `ISAM2::marginalCovariance(key)` | iSAM2 复用 Bayes Tree 直接查协方差（无需重建 Marginals） |
| `Pose3::transformFrom(p, H)` / `Pose3::retract(ξ)` / `Pose3::between(other)` | 几何操作，带可选 Jacobian 输出 |
| `numericalDerivative11/21/...` | 数值微分，验证手写 Jacobian 正确性 |

---

## 研究实践建议

面向不同目标的读者，给出分层次的实践路线。

**目标一：能用 GTSAM 搭一个能跑的位姿图后端（工程入门）**

1. 先吃透 25.2 的最小闭环：`NonlinearFactorGraph` + `Values` + `PriorFactor` + `BetweenFactor` + LM，跑通 2D 位姿图示例并能解释回环为何消除漂移。
2. 把它改写成 iSAM2 增量版本（25.3 练习 1），亲手感受批量与增量在耗时随帧数增长上的差异。
3. 给每个位姿查一次 `marginalCovariance`（25.9 练习 1），建立"点估计 + 不确定性"一起看的习惯。

**目标二：能读懂并改造工业级 LIO/VIO 后端（工程进阶）**

1. 先打通理论骨架 25.1→25.3，再带着"每种因子的噪声怎么设、`update()` 调几次、初值从哪来"三个问题精读 LIO-SAM `mapOptimization.cpp`（25.7）。
2. 重点对照 25.4：确认 LIO-SAM 用的是 `ImuFactor` + 偏差 `BetweenFactor` 而非 `CombinedImuFactor`，并理解每次 bias 更新后必须 reset 预积分器。
3. 把回环鲁棒化补齐：前端几何预筛 + 回环因子加 Huber 核（25.10），并在离线复盘时用 `GncOptimizer` 验证高外点比例下的鲁棒性。

**目标三：做 SLAM 后端方向的研究（研究级）**

1. 精读 iSAM2 原论文（Kaess 2012）与本章 25.3 对照，弄清 Bayes Tree 增量消元与 fluid relinearization 的关系；能回答"回环为何是 iSAM2 最贵的操作"（25.3 练习 3）。
2. 围绕不确定性做文章：理解右扰动切空间协方差与全局量协方差的伴随变换（25.9），这是主动 SLAM、信息论式视角选择、退化检测的数学基础。
3. 沿鲁棒后端前沿推进：从 M-估计→Switchable Constraints→GNC（25.10）梳理鲁棒位姿图优化的演进，复现 GNC 在不同外点比例下相对 Huber 的优势（25.10 练习 1-2）。
4. 跨库对照：用 通用库·Ceres 和本章 GTSAM 实现同一问题（25.8 练习 3），从因子图结构、增量能力、协方差接口、退化处理四个维度形成自己的库选型判断；下一章 SLAM库·g2o 会补上第三个参照系。
