# 李群与 manif 库——SLAM 中的刚体变换编程

> **难度**：⭐⭐～⭐⭐⭐ | **建议用时**：1周 | **前置要求**：通用库·Eigen Eigen深入、线性代数基础（旋转矩阵、齐次坐标）

---

## 前置自测

> 📋 答不出 >= 2 题 → 先回顾 Eigen 基础章节和线性代数旋转部分

1. 旋转矩阵 $R \in \mathbb{R}^{3\times3}$ 满足什么约束条件？为什么 $R^{-1} = R^T$？这个性质的几何含义是什么？
2. 齐次变换矩阵 $T \in \mathbb{R}^{4\times4}$ 的结构是什么？给定 $T_1, T_2$ 分别表示"从世界坐标系到相机1"和"从相机1到相机2"的变换，如何计算"从世界坐标系到相机2"的变换？
3. 什么是反对称矩阵（skew-symmetric matrix）？给定向量 $\omega = [\omega_1, \omega_2, \omega_3]^T$，写出其反对称矩阵 $[\omega]_\times$。
4. 矩阵指数 $e^A$ 的定义是什么？对于 $A = \theta [\omega]_\times$（$\|\omega\|=1$），$e^A$ 的结果是什么？
5. `Eigen::Quaterniond` 和 `Eigen::Matrix3d` 表示旋转时各有什么优缺点？什么时候用哪个？

---

## 知识树

```
李群与 manif 库
├── 数学基础
│   ├── 23.1  李群/李代数工程回顾（SO(3)/SE(3)、不是向量）
│   ├── 23.2  群作用（compose vs act、链式变换）
│   ├── 23.3  切空间与李代数（hat/vee、角速度矩阵）
│   └── 23.4  指数映射完整推导（Rodrigues、Exp/Log）
├── 流形优化核心
│   ├── 23.5  Plus/Minus 算子（⊕/⊖：优化器与流形的桥梁）
│   ├── 23.6  伴随矩阵 Adjoint（左右扰动转换 J_L = Ad·J_R）
│   ├── 23.8  左扰动 vs 右扰动（约定一致性）
│   ├── 23.9  流形上的 Jacobian（5 类基础模块 + 链式法则）
│   └── 23.10 不确定性与协方差传播（切空间高斯）
├── 工程实践
│   ├── 23.7  manif 核心 API（compose/inverse/log/exp/act）
│   ├── 23.11 离散积分与 IMU 预积分基础
│   ├── 23.12 SO(3)/SE(3) 公式速查表
│   ├── 23.13 manif vs Sophus 对比
│   ├── 23.14 manif 与 Ceres 集成（Manifold 接口）
│   ├── 23.15 Sophus 实战（ORB-SLAM3/FAST-LIVO2）
│   └── 23.16 流形代码验证与工程边界
└── 前沿扩展
    └── 23.17 扩展李群与前沿应用（SE_2(3)、等变网络）
```

---

## 本章目标

学完本章，你将能够：

- 理解李群 SO(3)、SE(3) 的数学本质，以及 exp/log 映射为什么是 SLAM 优化的基础工具
- 掌握群作用（Group Actions）的公理和多坐标系链式变换
- 从旋转运动学方程出发理解切空间、李代数、hat/vee 算子的来源
- 完整推导 Rodrigues 公式和 SE(3) 指数映射闭式解，区分大写 Exp/Log 与小写 exp/log
- 理解 Plus/Minus 算子和伴随矩阵在优化器中的作用
- 区分左扰动与右扰动约定，掌握流形上的 Jacobian 定义和基础 Jacobian 模块
- 理解流形上的不确定性模型和协方差传播
- 理解离散积分与 IMU 预积分的流形基础
- 使用 manif 库的核心 API 完成位姿的组合、逆运算、切空间操作和 Jacobian 计算
- 对比 manif 和 Sophus 的设计哲学与适用场景，阅读 ORB-SLAM3 和 FAST-LIVO2 源码中的 Sophus 用法
- 将 manif 与 Ceres Manifold 接口集成，构建基于流形优化的 SLAM 后端

**本章在课程中的位置**：前面的 Eigen 章节让我们掌握了矩阵运算的底层工具。但在 SLAM 中，我们操作的不是一般矩阵——而是旋转和刚体变换。这些对象生活在弯曲的流形上，不能像普通向量那样直接做加减法。如果不理解这一点，你写出的优化器要么不收敛，要么收敛到错误的解。本章就是要把"旋转和位姿"从"我知道用矩阵表示"提升到"我知道如何在流形上正确地做优化"。

---

## 23.1 李群/李代数工程回顾 ⭐⭐

> **本节在全章中的位置**：概览李群、李代数、exp/log 映射的核心概念，建立整章的知识框架。后续各节将逐一深入。

### 动机：旋转不是向量，但优化器需要向量

假设你正在做视觉 SLAM，需要优化相机的位姿。优化器（如 Gauss-Newton）的核心操作是：

$$x_{k+1} = x_k + \Delta x$$

这里 $x_k$ 是当前估计，$\Delta x$ 是更新增量。对于位置（3D 向量），这个加法完全自然。但对于旋转呢？

旋转矩阵 $R \in SO(3)$ 是一个 $3\times3$ 矩阵，满足 $R^T R = I$ 且 $\det(R) = 1$。如果我们天真地做 $R_{k+1} = R_k + \Delta R$，结果几乎肯定不再是旋转矩阵——因为 $R_k + \Delta R$ 不满足正交约束。

这就是 SLAM 中最根本的数学障碍之一：**旋转空间不是向量空间，无法直接做加法**。

### 如果不用李群会怎样

**方案 A：直接优化旋转矩阵的9个元素**

旋转矩阵有 9 个元素，但只有 3 个自由度（因为 $R^T R = I$ 提供了 6 个约束）。如果用 9 个变量做优化，你需要额外加上 6 个约束。这不仅增加了问题的规模，而且约束条件是非线性的（$r_i^T r_j = \delta_{ij}$），让优化变得更难。更严重的是，每次迭代后你需要"投影"回 SO(3)（通过 SVD 找最近的正交矩阵），这个投影步骤本身就有计算开销和数值误差。

**方案 B：用四元数但不考虑范数约束**

四元数 $q = [w, x, y, z]$ 有 4 个参数表示 3 个自由度，需要保持 $\|q\| = 1$。VINS-Mono 就是这种方案——用 `Eigen::Quaterniond` 直接做优化，每次迭代后手动归一化。这比方案 A 好，但归一化步骤破坏了优化器的内部假设（Gauss-Newton 假设参数空间是欧几里得的），导致收敛速度变慢。

**方案 C：用欧拉角**

欧拉角只用 3 个参数，似乎恰好匹配 3 个自由度。但欧拉角有万向锁（Gimbal Lock）问题——当 pitch 角接近 $\pm 90°$ 时，yaw 和 roll 变得不可区分，Jacobian 矩阵退化为奇异矩阵，优化器直接崩溃。在飞行器 SLAM 中，这不是理论上的担忧——无人机做翻滚动作时 pitch 完全可能达到 $90°$。

这三种方案的共同问题是：**它们都试图把旋转塞进欧几里得空间，但旋转空间天生就不是欧几里得的**。李群理论提供了正确的解决方案。

### 理论：SO(3) 与 SE(3)

**李群（Lie Group）**是一个同时具有群结构和光滑流形结构的数学对象。对 SLAM 而言，最重要的两个李群是：

**SO(3)——三维旋转群**

$$SO(3) = \{R \in \mathbb{R}^{3\times3} \mid R^T R = I, \det(R) = 1\}$$

SO(3) 是一个 3 维流形。直觉上，你可以把它想象成一个 3 维球面（准确说是 $\mathbb{RP}^3$，实射影空间）。群操作是矩阵乘法：两个旋转的组合 $R_1 R_2$ 仍然是旋转，逆元 $R^{-1} = R^T$ 也是旋转。

**对应的李代数 $\mathfrak{so}(3)$** 是 SO(3) 在单位元（即 $I$）处的切空间。几何上，它是所有可能的"瞬时角速度"的集合。代数上，$\mathfrak{so}(3)$ 是所有 $3\times3$ 反对称矩阵的集合：

$$\mathfrak{so}(3) = \{[\omega]_\times \in \mathbb{R}^{3\times3} \mid \omega \in \mathbb{R}^3\}$$

其中 $[\omega]_\times$ 是 $\omega$ 对应的反对称矩阵。因为反对称矩阵完全由 3 个参数决定，所以 $\mathfrak{so}(3) \cong \mathbb{R}^3$——这正是我们需要的 3 个自由度！

**SE(3)——三维刚体运动群**

$$SE(3) = \left\{\begin{bmatrix} R & t \\ 0^T & 1 \end{bmatrix} \in \mathbb{R}^{4\times4} \mid R \in SO(3), t \in \mathbb{R}^3\right\}$$

SE(3) 是一个 6 维流形（3 个旋转自由度 + 3 个平移自由度）。群操作是齐次矩阵乘法。

**对应的李代数 $\mathfrak{se}(3)$** 由 6 维向量参数化，称为"扭转"（twist）：

$$\xi = \begin{bmatrix} \rho \\ \omega \end{bmatrix} \in \mathbb{R}^6$$

其中 $\rho \in \mathbb{R}^3$ 是平移分量，$\omega \in \mathbb{R}^3$ 是旋转分量。

**指数映射（exp）与对数映射（log）**

指数映射 $\exp: \mathfrak{g} \to G$ 将李代数元素映射到李群元素。对 SO(3)，这就是 Rodrigues 公式：

$$\exp(\theta [\hat\omega]_\times) = I + \sin\theta \, [\hat\omega]_\times + (1 - \cos\theta) \, [\hat\omega]_\times^2$$

其中 $\hat\omega = \omega / \|\omega\|$ 是单位旋转轴，$\theta = \|\omega\|$ 是旋转角度。关键性质 $[\hat\omega]_\times^3 = -[\hat\omega]_\times$ 使得 Taylor 级数化简为这个紧凑的三角函数形式。**23.4 节将从 ODE 出发给出完整的推导过程。**

对数映射 $\log: G \to \mathfrak{g}$ 是指数映射的逆运算。对 SO(3)：

$$\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right), \quad \hat\omega = \frac{1}{2\sin\theta}\begin{bmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{bmatrix}$$

这里有两个数值上需要注意的边界情况：
- 当 $\theta \to 0$ 时，$\sin\theta \to 0$，分母趋于零。需要用 Taylor 展开做近似。
- 当 $\theta \to \pi$ 时，$\sin\theta \to 0$，同样有数值问题。需要特殊处理。

**为什么 SLAM 需要李群？**

有了 exp/log 映射，我们可以这样做优化：

$$R_{k+1} = R_k \cdot \exp([\Delta\phi]_\times)$$

其中 $\Delta\phi \in \mathbb{R}^3$ 是李代数空间中的增量。这个更新保证 $R_{k+1}$ 仍然是合法的旋转矩阵，因为 $\exp([\Delta\phi]_\times) \in SO(3)$，而两个 SO(3) 元素的乘积仍在 SO(3) 中。

类似地，对 SE(3)：

$$T_{k+1} = T_k \cdot \exp([\Delta\xi]_\wedge)$$

其中 $\Delta\xi \in \mathbb{R}^6$ 是 6 维增量。优化器在 $\mathbb{R}^6$（欧几里得空间）中计算增量，然后通过 exp 映射将其"投影"回 SE(3) 流形。这就是所谓的"流形上的优化"。

**伴随表示（Adjoint Representation）**

伴随映射 $\text{Ad}_T: \mathfrak{se}(3) \to \mathfrak{se}(3)$ 描述了坐标系变换下李代数元素如何变换。伴随在 SLAM 中有两个核心应用：(1) 将机体坐标系下的速度/扰动转换到世界坐标系下；(2) 在左右扰动 Jacobian 之间转换。**23.6 节将从定义出发完整推导 SO(3) 和 SE(3) 的伴随矩阵**。

| 概念 | 维度 | 直觉 | SLAM应用 |
|------|------|------|---------|
| SO(3) | 3 | 旋转 | 相机朝向、IMU姿态 |
| $\mathfrak{so}(3)$ | 3 | 角速度 | 陀螺仪测量、旋转增量 |
| SE(3) | 6 | 刚体运动 | 相机位姿、激光雷达位姿 |
| $\mathfrak{se}(3)$ | 6 | 扭转速度 | 位姿增量、优化变量 |
| exp | $\mathfrak{g} \to G$ | 从切空间到流形 | 优化器更新步 |
| log | $G \to \mathfrak{g}$ | 从流形到切空间 | 计算两个位姿的"差" |
| Adjoint | $6\times6$ 矩阵 | 坐标系变换 | 协方差传播、Jacobian 转换 |

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：用欧拉角做 SLAM 优化**

- **错误做法**：把旋转表示为 yaw/pitch/roll 三个角度，直接作为优化变量
- **现象**：优化在大多数情况下正常工作，但当 pitch 接近 $\pm 90°$ 时突然发散、NEES 飙高
- **根本原因**：欧拉角在 pitch $= \pm 90°$ 时存在万向锁（Gimbal Lock），此时 Jacobian $\partial(\text{rotation})/\partial(\text{euler angles})$ 的秩从 3 降为 2，优化器的正规方程 $J^T J \Delta x = -J^T r$ 变得欠定。这不是数值精度问题，而是表示本身的拓扑缺陷——$\mathbb{R}^3$ 无法全局覆盖 SO(3)
- **正确做法**：使用李代数参数化旋转。manif 和 Sophus 都在内部处理了 exp/log 的边界情况，不会出现万向锁

**💡 概念误区：认为 $\exp(\phi_1) \cdot \exp(\phi_2) = \exp(\phi_1 + \phi_2)$**

- **新手想法**："矩阵指数和标量指数一样，$e^a \cdot e^b = e^{a+b}$"
- **实际上**：这个等式只在 $\phi_1$ 和 $\phi_2$ 对应的反对称矩阵可交换时成立（即 $[\phi_1]_\times [\phi_2]_\times = [\phi_2]_\times [\phi_1]_\times$），也就是两个旋转绕同一轴旋转。一般情况下，$\exp(\phi_1) \cdot \exp(\phi_2) = \exp(\phi_1 + \phi_2 + \frac{1}{2}[\phi_1, \phi_2] + \cdots)$，其中 $[\phi_1, \phi_2]$ 是李括号，后面是 Baker-Campbell-Hausdorff（BCH）公式的高阶项
- **为什么重要**：如果你在代码中写 `SO3::Exp(phi1 + phi2)` 来表示两个旋转的组合，结果是错误的。正确做法是 `SO3::Exp(phi1).compose(SO3::Exp(phi2))`
- **自检方法**：取 $\phi_1 = [0.5, 0, 0]$，$\phi_2 = [0, 0.5, 0]$，比较 $\exp(\phi_1)\exp(\phi_2)$ 和 $\exp(\phi_1 + \phi_2)$，会发现差异约为 $0.12$ 弧度

**🧠 思维陷阱：认为所有旋转表示是等价的**

- **新手想法**："旋转矩阵、四元数、欧拉角、轴角、李代数——都是旋转的不同表示，想用哪个用哪个"
- **实际上**：不同表示有本质差异。四元数和旋转矩阵是 SO(3) 的全局参数化（但有冗余），欧拉角只是局部参数化（有万向锁），轴角/李代数是切空间参数化（只在单位元附近有效，远离单位元时 exp 映射不再是双射——因为旋转 $\theta$ 和 $\theta + 2\pi$ 对应同一个旋转矩阵）
- **正确思维**：选择旋转表示要问三个问题：(1) 是否需要全局无奇异？ (2) 存储/计算效率如何？ (3) 是否需要做优化？不同场景答案不同——存储用四元数（4个数），插值用李代数（exp/log），优化用李代数参数化 + 四元数/旋转矩阵存储
- **自检方法**：列出 SO(3) 的 5 种常见参数化，比较它们的维度、奇异性、是否全局覆盖

### 练习

**练习 23.1.1**（⭐⭐）：手工验证 Rodrigues 公式。取 $\omega = [0, 0, 1]^T$，$\theta = \pi/4$（绕 z 轴转 45 度），用 Rodrigues 公式计算 $R = \exp(\theta [\hat\omega]_\times)$，然后与直接构造的 $R_z(\pi/4)$ 比较。要求写出中间每一步的数值计算过程。

**练习 23.1.2**（⭐⭐）：证明 $[\hat\omega]_\times^3 = -[\hat\omega]_\times$（其中 $\|\hat\omega\| = 1$）。提示：先计算 $[\hat\omega]_\times^2$，利用 $[\hat\omega]_\times^2 = \hat\omega \hat\omega^T - I$。

**练习 23.1.3**（⭐⭐⭐）：用 Eigen 实现一个函数 `Eigen::Matrix3d rodrigues(const Eigen::Vector3d& omega)`，输入是 3 维旋转向量（方向是旋转轴，模长是旋转角度），输出是旋转矩阵。要求正确处理 $\theta \to 0$ 的边界情况（用 Taylor 展开到二阶项）。然后与 `Eigen::AngleAxisd(theta, axis).toRotationMatrix()` 的结果比较，验证误差小于 $10^{-12}$。

---

## 23.2 群作用（Group Actions）⭐⭐

### 这一节解决什么问题

前面我们知道了 SO(3) 和 SE(3) 是"旋转群"和"刚体运动群"。但群本身只是一个代数结构——它描述的是变换之间如何组合。真正让李群在 SLAM 中有用的，是群如何"作用"于空间中的具体对象（点、向量、坐标系）。当我们说"用 SE(3) 位姿变换一个 3D 点"时，我们其实是在使用群作用。如果不严格理解群作用的公理和性质，就容易犯"左乘右乘分不清"的错误——尤其在多坐标系链式变换中。

### 动机：一个点在多个坐标系之间的变换

考虑一个 SLAM 系统中的典型场景：LiDAR 扫描到一个 3D 点 $p_L$（在 LiDAR 坐标系中），需要将其转换到世界坐标系 $p_W$。变换链是：

$$p_W = T_{WB} \cdot T_{BL} \cdot p_L$$

其中 $T_{WB}$ 是 body（机体）相对世界的位姿，$T_{BL}$ 是 LiDAR 相对 body 的外参。这里的"$\cdot$"号到底是什么运算？它不是普通的矩阵乘法（$p_L$ 是 3 维向量，$T$ 是 $4\times4$ 矩阵，维度都对不上），而是群对空间的**作用（action）**。

### 理论：群作用的公理化定义

**定义**：设 $G$ 是一个群，$V$ 是一个集合。群作用（group action）是一个映射 $\cdot: G \times V \to V$，满足两条公理：

| 公理 | 数学表达 | 直觉 |
|------|---------|------|
| 恒等性（Identity） | $E \cdot v = v$ | 不做任何变换 = 什么都不变 |
| 相容性（Compatibility） | $(X \circ Y) \cdot v = X \cdot (Y \cdot v)$ | 先组合再作用 = 逐步作用 |

其中 $E$ 是群的单位元，$\circ$ 是群操作（compose）。

为什么需要这两条公理？恒等性保证了"什么都不做"确实不改变任何东西。相容性保证了坐标系链式变换是自洽的——你可以先把 $T_{WB}$ 和 $T_{BL}$ 组合成 $T_{WL} = T_{WB} \circ T_{BL}$，再用 $T_{WL}$ 作用于 $p_L$，得到的结果与先用 $T_{BL}$ 把 $p_L$ 变换到 body 系、再用 $T_{WB}$ 变换到世界系完全相同。如果相容性不成立，SLAM 中的多级坐标变换就会失去一致性。

### SO(3) 的群作用：旋转一个点

SO(3) 对 $\mathbb{R}^3$ 的作用就是矩阵-向量乘法：

$$R \cdot p = Rp$$

验证公理：
- 恒等性：$I \cdot p = Ip = p$ ✓
- 相容性：$(R_1 R_2) \cdot p = R_1 R_2 p = R_1 \cdot (R_2 \cdot p)$ ✓

几何含义：旋转矩阵 $R$ 把向量 $p$ 绕原点旋转。注意 SO(3) 的作用保持向量的长度不变（$\|Rp\| = \|p\|$），这是因为 $R$ 是正交矩阵。这个性质在 SLAM 中意味着：旋转不会改变两个点之间的距离，因此 ICP 匹配的残差只依赖于旋转是否正确，不会被旋转本身"放大"。

### SE(3) 的群作用：刚体运动

SE(3) 对 $\mathbb{R}^3$ 的作用稍微复杂一些：

$$M \cdot p = Rp + t$$

其中 $M = \begin{bmatrix} R & t \\ 0^T & 1 \end{bmatrix} \in SE(3)$。

验证公理：
- 恒等性：$\begin{bmatrix} I & 0 \\ 0^T & 1 \end{bmatrix} \cdot p = Ip + 0 = p$ ✓
- 相容性：设 $M_1 = (R_1, t_1)$，$M_2 = (R_2, t_2)$，则：
  - $M_1 \circ M_2 = (R_1 R_2, R_1 t_2 + t_1)$
  - $(M_1 \circ M_2) \cdot p = R_1 R_2 p + R_1 t_2 + t_1$
  - $M_1 \cdot (M_2 \cdot p) = R_1(R_2 p + t_2) + t_1 = R_1 R_2 p + R_1 t_2 + t_1$ ✓

注意一个重要的区别：SE(3) 的群操作是 $4\times4$ 矩阵乘法（输入和输出都是 SE(3) 元素），但群作用是把 SE(3) 元素作用于 $\mathbb{R}^3$ 中的点（输入是 SE(3) + 点，输出是点）。初学者常混淆这两者。

### $S^3$ 四元数群作用：双覆盖

单位四元数群 $S^3$ 也可以作用于 $\mathbb{R}^3$，但方式不同于 SO(3) 的矩阵乘法：

$$q \cdot x = q \, x \, q^*$$

其中 $x$ 被嵌入为纯四元数 $[0, x_1, x_2, x_3]$，$q^*$ 是 $q$ 的共轭。

这个作用有一个重要特性：$q$ 和 $-q$ 给出**完全相同**的旋转。这就是所谓的"双覆盖"（double cover）——$S^3$ 到 SO(3) 是 2:1 的映射。在 SLAM 工程中，这意味着用四元数做优化时，$q$ 和 $-q$ 是等价解。如果你的损失函数在 $q$ 处有最小值，那么在 $-q$ 处也有一个完全相同的最小值。这不会影响正确性（两个解对应同一个旋转），但可能影响优化器的收敛行为——如果初始值接近 $-q_{\text{opt}}$ 而不是 $q_{\text{opt}}$，梯度方向仍然是正确的。

### 群作用对比表

| 群 | 作用对象 | 作用公式 | 几何含义 | SLAM 应用 |
|---|---------|---------|---------|----------|
| SO(3) | $\mathbb{R}^3$ | $R \cdot p = Rp$ | 绕原点旋转 | 旋转 IMU 测量到世界坐标系 |
| SE(3) | $\mathbb{R}^3$ | $M \cdot p = Rp + t$ | 刚体运动 | LiDAR 点云变换到世界坐标系 |
| $S^3$ | $\mathbb{R}^3$ | $q \cdot x = qxq^*$ | 旋转（双覆盖） | 四元数形式的旋转操作 |
| SO(2) | $\mathbb{R}^2$ | $R \cdot p = Rp$ | 平面旋转 | 2D SLAM 中旋转激光扫描 |
| SE(2) | $\mathbb{R}^2$ | $M \cdot p = Rp + t$ | 平面刚体运动 | 2D 栅格地图中位姿变换 |

### 为什么群作用在 SLAM 中至关重要

群作用的相容性公理 $(X \circ Y) \cdot v = X \cdot (Y \cdot v)$ 直接保证了坐标系链式变换的正确性。在一个典型的 LiDAR-IMU-Camera 多传感器系统中，一个点 $p_C$（相机坐标系）要变换到世界坐标系，需要经过：

$$p_W = T_{WI} \cdot T_{IC} \cdot p_C$$

如果没有相容性公理，你无法确定"先变换到 IMU 系再变换到世界系"和"直接用组合变换"的结果是否一致。更进一步，群作用的 Jacobian 是 SLAM 优化中代价函数 Jacobian 的核心构建块——几乎所有的视觉/LiDAR 残差都涉及"用位姿变换一个点，然后投影到图像/匹配最近邻"。

### manif 代码：`act` 与 Jacobian

manif 通过 `act` 方法实现群作用，并支持同时计算两个 Jacobian：

```cpp
#include <manif/SE3.h>

manif::SE3d T_world_body = manif::SE3d::Random();
Eigen::Vector3d p_body(1.0, 2.0, 3.0);

// 基本用法：变换点
Eigen::Vector3d p_world = T_world_body.act(p_body);
// 等价于 R * p_body + t

// 带 Jacobian 的用法（SLAM 优化必备）
Eigen::Matrix<double, 3, 6> J_T;   // d(act) / d(T)，3x6 矩阵
Eigen::Matrix<double, 3, 3> J_p;   // d(act) / d(p)，3x3 矩阵
Eigen::Vector3d p_world2 = T_world_body.act(p_body, J_T, J_p);

// J_T 是 act 对位姿的 Jacobian（右扰动约定）：
//   J_T = d(Rp+t) / d(delta)|_{delta=0}
//   其中 T' = T * Exp(delta)
// 数学上：J_T = [-R [p]_x,  R]  (3x6，注意 manif 的 [rho, omega] 顺序)

// J_p 是 act 对点的 Jacobian：
//   J_p = d(Rp+t) / dp = R
// 即旋转矩阵本身
```

需要特别注意 `J_T` 的结构：它是一个 $3 \times 6$ 矩阵。前 3 列对应平移分量的扰动（$\partial(Rp+t)/\partial \delta_\rho = R$），后 3 列对应旋转分量的扰动（$\partial(Rp+t)/\partial \delta_\omega = -R[p]_\times$）。这个顺序与 manif 的 twist 约定 $[\rho, \omega]$ 一致。

**验证群作用公理的代码**：

```cpp
// 验证恒等性：Identity.act(p) == p
manif::SE3d I = manif::SE3d::Identity();
Eigen::Vector3d p(1.0, 2.0, 3.0);
assert((I.act(p) - p).norm() < 1e-12);

// 验证相容性：(T1 * T2).act(p) == T1.act(T2.act(p))
manif::SE3d T1 = manif::SE3d::Random();
manif::SE3d T2 = manif::SE3d::Random();
Eigen::Vector3d lhs = (T1 * T2).act(p);
Eigen::Vector3d rhs = T1.act(T2.act(p));
assert((lhs - rhs).norm() < 1e-10);
```

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：用齐次坐标矩阵乘法代替 `act`，忽略 Jacobian**

- **错误做法**：`Eigen::Vector4d p_h; p_h << p, 1.0; Eigen::Vector4d result = T.transform() * p_h;`，然后手动提取前 3 维
- **现象**：变换结果正确，但当你需要 Jacobian 时不得不手工推导和实现，容易在 twist 分量顺序上出错
- **根本原因**：齐次坐标乘法得到的是正确的变换结果，但它绕过了 manif 的类型系统和自动 Jacobian 功能。在 SLAM 优化中，你几乎总是需要 Jacobian
- **正确做法**：用 `T.act(p, J_T, J_p)` 一步到位获取变换结果和两个 Jacobian

**💡 概念误区：认为 SE(3) 群作用和 SO(3) 群作用是独立的概念**

- **新手想法**："SO(3) 的作用是旋转，SE(3) 的作用是刚体运动，这是两种不同的操作"
- **实际上**：SE(3) 的群作用 $M \cdot p = Rp + t$ 可以分解为先执行 SO(3) 的作用（旋转 $Rp$），再执行平移（$+t$）。SE(3) 不是"另一种"群作用，而是 SO(3) 群作用的自然扩展。理解这个层级关系，才能理解为什么 SE(3) 的 Jacobian 中包含了 SO(3) 的 Jacobian 作为子块
- **延伸**：SE(3) 可以看作 SO(3) 和 $\mathbb{R}^3$ 的半直积（semidirect product），记作 $SE(3) = SO(3) \ltimes \mathbb{R}^3$。"半直积"意味着旋转和平移不是简单地"叠加"，而是旋转会影响平移（$R_1 t_2 + t_1$ 中的 $R_1 t_2$ 项）

**🧠 思维陷阱：认为 $q$ 和 $-q$ 的双覆盖在实际编程中无关紧要**

- **新手想法**："既然 $q$ 和 $-q$ 对应同一个旋转，那忽略它也没问题"
- **实际上**：在四元数插值（SLERP）中，如果 $q_1 \cdot q_2 < 0$（两个四元数在 $S^3$ 上的内积为负），直接插值会走"长弧"（绕远路）而不是"短弧"。正确做法是在插值前检查内积符号，如果为负则取反其中一个四元数。manif 内部使用四元数存储 SO(3)，但在 `log()` 等操作中已经处理了符号问题，用户通常不需要手动处理。但如果你直接操作底层的 `Eigen::Quaterniond`，就必须自己注意

### 练习

**练习 23.2.1**（⭐⭐）：手工验证 SE(3) 群作用的相容性。取 $T_1 = (R_z(30°), [1,0,0]^T)$，$T_2 = (R_x(45°), [0,1,0]^T)$，$p = [1,1,1]^T$。分别计算 $(T_1 \circ T_2) \cdot p$ 和 $T_1 \cdot (T_2 \cdot p)$，验证结果相同。写出每一步的数值计算过程。

**练习 23.2.2**（⭐⭐⭐）：用 manif 实现一个多坐标系链式变换。设有世界系 W、IMU 系 I、相机系 C、LiDAR 系 L。给定外参 $T_{IC}$、$T_{IL}$，以及 IMU 位姿 $T_{WI}$，将相机坐标系中的点 $p_C$ 和 LiDAR 坐标系中的点 $p_L$ 都变换到世界坐标系。用 `act` 方法获取对 $T_{WI}$ 的 Jacobian，验证数值 Jacobian 与解析 Jacobian 的一致性。

---

## 23.3 切空间与李代数深入 ⭐⭐⭐

### 这一节解决什么问题

23.1 节提到李代数是"单位元处的切空间"，但没有解释"切空间"到底是什么意思。本节从微分几何的角度深入理解李代数的来源：为什么反对称矩阵就是 SO(3) 的切空间？hat 和 vee 算子的数学含义是什么？为什么我们可以用 $\mathbb{R}^m$ 中的向量代替李代数中的矩阵来做计算？搞清楚这些问题，才能真正理解 exp/log 映射的数学基础，而不仅仅是"会用公式"。

### 动机：速度是什么

假设一个旋转矩阵 $R(t)$ 随时间变化，表示一个旋转的刚体在 $t$ 时刻的朝向。那么 $\dot{R}(t) = \frac{dR}{dt}$ 是什么？

直觉上，$\dot{R}(t)$ 应该是某种"角速度"。但 $\dot{R}(t)$ 是一个 $3\times3$ 矩阵——它显然不是我们熟悉的 3D 角速度向量 $\omega \in \mathbb{R}^3$。那么 $\dot{R}(t)$ 和 $\omega$ 之间是什么关系？这个问题的答案就是李代数理论的起点。

### 如果不理解切空间会怎样

如果你只是把李代数当作"一个 3 维向量空间，和 $\mathbb{R}^3$ 一样"，你可能会：

1. 不理解为什么 hat 算子把 3D 向量映射到 $3\times3$ 矩阵——"明明可以直接用 3D 向量，为什么要多此一举搞个矩阵？"
2. 不理解 exp 映射的 ODE 推导——"为什么 $\dot{X} = X\hat{v}$ 的解是 $X(t) = X(0)\exp(\hat{v}t)$？"
3. 在 SE(3) 中搞不清 twist 的 6 个分量到底表示什么——"$[\rho, \omega]$ 中 $\rho$ 为什么不是普通的平移速度？"

### 理论：从约束推导切空间

**Step 1：旋转矩阵的约束**

$R(t) \in SO(3)$ 意味着 $R(t)^T R(t) = I$ 对所有 $t$ 成立。对这个等式关于 $t$ 求导：

$$\frac{d}{dt}(R^T R) = \dot{R}^T R + R^T \dot{R} = 0$$

这告诉我们 $R^T \dot{R}$ 是一个**反对称矩阵**（因为 $A + A^T = 0$ 的矩阵就是反对称矩阵）。

为什么 $R^T \dot{R}$ 是反对称的？从 $\dot{R}^T R + R^T \dot{R} = 0$ 出发，设 $\Omega = R^T \dot{R}$，则 $\Omega^T = \dot{R}^T R = \dot{R}^T (R^T)^T$。代入约束：$\Omega^T + \Omega = 0$，即 $\Omega$ 是反对称的。

**Step 2：定义角速度**

定义 body 坐标系下的角速度矩阵：

$$[\omega]_\times \triangleq R^T \dot{R} \in \mathfrak{so}(3)$$

等价地：

$$\dot{R} = R [\omega]_\times$$

这就是旋转运动学方程。它说明：旋转矩阵的时间导数等于旋转矩阵本身乘以一个反对称矩阵。$\omega \in \mathbb{R}^3$ 就是我们熟悉的角速度向量——陀螺仪测量的就是这个量。

注意"body 坐标系下"的限定。如果定义 $[\omega']_\times = \dot{R} R^T$，这是 world 坐标系下的角速度，它们之间的关系是 $\omega' = R\omega$（通过伴随变换）。在 SLAM/VIO 中，IMU 的陀螺仪测量的是 body 系角速度 $\omega$（因为陀螺仪安装在机体上），所以 $\dot{R} = R[\omega]_\times$ 是最自然的形式。

**Step 3：切空间的几何意义**

$\dot{R}(t)$ 是 $R(t)$ 在 SO(3) 流形上的切向量，它属于 $R(t)$ 处的切空间 $T_{R(t)} SO(3)$。但 $\dot{R}$ 的形式取决于 $R(t)$ 的位置——不同的 $R$ 处，切空间"长得不一样"。

关键观察：通过左移（left-translation）$R^T \dot{R}$，我们把 $R(t)$ 处的切向量"搬运"到了单位元 $I$ 处的切空间 $T_I SO(3)$。单位元处的切空间就是**李代数** $\mathfrak{so}(3)$。

为什么要搬到单位元？因为单位元处的切空间是所有切空间中最特殊的一个——它同时具有向量空间结构和李括号结构，允许我们用线性代数的工具来处理流形上的问题。这就是李群理论的核心思想：**用单位元处的线性近似来研究整个群**。

### Hat 算子与 Vee 算子

**Hat 算子**（$\wedge$ 或 $\hat{\cdot}$）：将向量映射到李代数矩阵

对 SO(3)，hat 算子把 3D 向量 $\omega = [\omega_1, \omega_2, \omega_3]^T$ 映射为 $3\times3$ 反对称矩阵：

$$\omega^\wedge = [\omega]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$$

这个矩阵的特殊性质：$[\omega]_\times v = \omega \times v$（矩阵-向量乘法等于叉积）。这不是巧合——反对称矩阵的定义就是为了把叉积表示为矩阵乘法。

**Vee 算子**（$\vee$）：hat 的逆操作，将反对称矩阵映射回向量

$$[\omega]_\times^\vee = \omega$$

**为什么需要 hat/vee？** 因为李代数 $\mathfrak{so}(3)$ 的元素是矩阵（$3\times3$ 反对称矩阵），但它只有 3 个自由度。hat/vee 在 $\mathbb{R}^3$ 和 $\mathfrak{so}(3)$ 之间建立了一一对应（同构），让我们可以用 3 维向量代替 $3\times3$ 矩阵来做计算。这大大简化了编程——存储 3 个数比存储 9 个数高效得多。

### SO(3) 的生成元

$\mathfrak{so}(3)$ 作为一个 3 维向量空间，有一组自然的基底——称为**生成元**（generators）：

$$E_1 = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix}, \quad E_2 = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 0 & 0 \\ -1 & 0 & 0 \end{bmatrix}, \quad E_3 = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$

$E_1, E_2, E_3$ 分别对应绕 $x, y, z$ 轴的单位角速度。任何 $\mathfrak{so}(3)$ 元素都可以表示为它们的线性组合：

$$[\omega]_\times = \omega_1 E_1 + \omega_2 E_2 + \omega_3 E_3$$

这正是 hat 算子的另一种写法。生成元的几何含义：$\exp(\theta E_i)$ 是绕第 $i$ 个坐标轴旋转 $\theta$ 角度的旋转矩阵。例如 $\exp(\theta E_3)$ 就是绕 $z$ 轴旋转 $\theta$ 的矩阵 $R_z(\theta)$。

### SE(3) 的李代数 $\mathfrak{se}(3)$

SE(3) 的李代数元素是 $4\times4$ 矩阵：

$$\hat{\tau} = \begin{bmatrix} [\theta]_\times & \rho \\ 0^T & 0 \end{bmatrix} \in \mathfrak{se}(3)$$

其中 $\theta \in \mathbb{R}^3$ 是旋转分量，$\rho \in \mathbb{R}^3$ 是平移分量。对应的向量表示（vee 之后）是：

$$\tau = \begin{bmatrix} \rho \\ \theta \end{bmatrix} \in \mathbb{R}^6$$

注意顺序：manif 和 Sola 论文都采用 $[\rho, \theta]$ 的约定（平移在前）。hat 算子把这个 6 维向量映射到 $4\times4$ 矩阵，vee 算子做逆操作。

$\mathfrak{se}(3)$ 有 6 个生成元：$E_1, \ldots, E_6$。前 3 个对应纯平移（沿 $x, y, z$ 轴），后 3 个对应纯旋转（绕 $x, y, z$ 轴）。

### 关键同构：$\mathfrak{m} \cong \mathbb{R}^m$

hat 和 vee 算子建立的同构 $\mathfrak{m} \cong \mathbb{R}^m$ 是整个李群理论在工程中可用的关键。它意味着：

- $\mathfrak{so}(3)$ 中的 $3\times3$ 反对称矩阵可以用 $\mathbb{R}^3$ 中的 3D 向量完全替代
- $\mathfrak{se}(3)$ 中的 $4\times4$ 矩阵可以用 $\mathbb{R}^6$ 中的 6D 向量完全替代
- 所有的优化、插值、滤波操作都可以在 $\mathbb{R}^m$ 中完成，只在最后需要群元素时才通过 exp 映射回去

没有这个同构，我们就不得不在矩阵空间中做优化（受制于 $R^T R = I$ 等非线性约束）。有了这个同构，SLAM 优化器只需要处理普通的欧几里得向量——这正是 Ceres 和 GTSAM 能够高效工作的数学基础。

### manif 代码：hat 和生成元

```cpp
#include <manif/SO3.h>
#include <manif/SE3.h>

// === hat 算子 ===
manif::SO3Tangentd omega;
omega.coeffs() << 0.1, 0.2, 0.3;

// hat(): R^3 -> so(3)，返回 3x3 反对称矩阵
Eigen::Matrix3d omega_hat = omega.hat();
// omega_hat 应该是：
//   [  0   -0.3  0.2 ]
//   [ 0.3   0   -0.1 ]
//   [-0.2  0.1   0   ]

// SE3 的 hat
manif::SE3Tangentd tau;
tau.coeffs() << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;  // [rho, omega]

// hat(): R^6 -> se(3)，返回 4x4 矩阵
Eigen::Matrix4d tau_hat = tau.hat();
// tau_hat 应该是：
//   [  0   -0.3  0.2  1.0 ]
//   [ 0.3   0   -0.1  2.0 ]
//   [-0.2  0.1   0    3.0 ]
//   [  0    0    0    0   ]

// === 生成元 ===
// manif::SO3Tangentd::Generator(i) 返回第 i 个生成元
Eigen::Matrix3d E0 = manif::SO3Tangentd::Generator(0);  // 绕 x 轴
Eigen::Matrix3d E1 = manif::SO3Tangentd::Generator(1);  // 绕 y 轴
Eigen::Matrix3d E2 = manif::SO3Tangentd::Generator(2);  // 绕 z 轴

// 验证：omega.hat() == omega_1 * E0 + omega_2 * E1 + omega_3 * E2
Eigen::Matrix3d omega_hat_check = 0.1 * E0 + 0.2 * E1 + 0.3 * E2;
assert((omega_hat - omega_hat_check).norm() < 1e-12);
```

### ⚠️ 常见陷阱

**💡 概念误区：认为 $\dot{R}$ 本身就是角速度**

- **新手想法**："$\dot{R}$ 是旋转的导数，所以它就是角速度"
- **实际上**：$\dot{R}$ 是一个 $3\times3$ 矩阵，属于 $R$ 处的切空间 $T_R SO(3)$，它不是角速度。角速度 $\omega$ 是一个 3D 向量，需要通过 $[\omega]_\times = R^T \dot{R}$（或 $[\omega']_\times = \dot{R} R^T$）来提取。$\dot{R}$ 和 $\omega$ 的关系就像"流形上的速度"和"切空间中的坐标"的关系
- **为什么重要**：如果你在代码中用有限差分 $\dot{R} \approx (R(t+\Delta t) - R(t))/\Delta t$ 来估计角速度，得到的是一个 $3\times3$ 矩阵，不能直接当 3D 向量用。正确做法是 $(R(t)^T R(t+\Delta t))$ 的 log 再除以 $\Delta t$

**⚠️ 编程陷阱：混淆 body 系角速度和 world 系角速度**

- **错误做法**：IMU 陀螺仪输出角速度 $\omega$（body 系），直接用 $\dot{R} = [\omega]_\times R$（这是 world 系公式）来积分
- **现象**：积分出来的旋转轨迹完全错误，转的方向不对
- **根本原因**：$\dot{R} = R[\omega]_\times$ 是 body 系角速度的公式，$\dot{R} = [\omega']_\times R$ 是 world 系角速度的公式。IMU 测量的是 body 系角速度，必须用 $\dot{R} = R[\omega]_\times$
- **正确做法**：`R_new = R * manif::SO3d::Exp(omega * dt);`（body 系角速度 + 右乘更新）

**🧠 思维陷阱：认为 hat/vee 只是"存储格式转换"**

- **新手想法**："hat 只是把 3 个数塞进 $3\times3$ 矩阵，没什么深刻含义"
- **实际上**：hat 算子的核心性质是 $[\omega]_\times v = \omega \times v$，它把叉积（一个非线性的向量运算）表达为矩阵乘法（一个线性运算）。这使得叉积可以参与线性代数的推导框架（如求 Jacobian、做矩阵分解）。整个 Rodrigues 公式的推导都依赖于这个性质
- **延伸**：hat 算子的另一个重要性质是 $R[\omega]_\times R^T = [R\omega]_\times$，即"旋转一个反对称矩阵等于旋转其对应的向量"。这个性质在推导伴随矩阵时是核心工具

### 练习

**练习 23.3.1**（⭐⭐）：手工推导 SO(3) 切空间。从约束 $R^T R = I$ 出发，对 $t$ 求导，证明 $R^T \dot{R}$ 是反对称矩阵。然后写出 $3\times3$ 反对称矩阵的一般形式，确认它有 3 个自由度。

**练习 23.3.2**（⭐⭐⭐）：验证性质 $R[\omega]_\times R^T = [R\omega]_\times$。取 $R = R_z(30°)$，$\omega = [1, 2, 3]^T$，分别计算等式两边并比较。然后在草稿纸上对一般的 $R$ 和 $\omega$ 证明这个性质（提示：利用 $[\omega]_\times v = \omega \times v$ 和旋转对叉积的保持性 $R(\omega \times v) = (R\omega) \times (Rv)$）。

---

## 23.4 指数映射完整推导 ⭐⭐⭐

### 这一节解决什么问题

23.1 节给出了 Rodrigues 公式作为 SO(3) 的指数映射结果，但没有展示完整的推导过程。本节从运动微分方程 (ODE) 出发，一步步推导出 Rodrigues 公式和 SE(3) 的指数映射闭式解。同时严格定义大写的 Exp/Log（从 $\mathbb{R}^m$ 直接到群/从群直接到 $\mathbb{R}^m$）与小写 exp/log（从李代数矩阵到群/从群到李代数矩阵）的区别。理解完整推导是真正掌握李群理论的分水岭。

### 动机：exp 映射从何而来

在 23.3 节中我们得到了旋转运动学方程 $\dot{R} = R[\omega]_\times$。如果角速度 $\omega$ 恒定，这个 ODE 的解是什么？在标量情况下，$\dot{x} = ax$ 的解是 $x(t) = x(0)e^{at}$。类推到矩阵情况，$\dot{R} = R[\omega]_\times$ 的解是不是 $R(t) = R(0) \exp([\omega]_\times t)$？这个类推是正确的——而这就是 exp 映射的物理来源。

### ODE 推导

考虑一般的李群运动方程。设 $X(t) \in G$ 是一条群上的曲线，在单位元处的初始条件为 $X(0) = E$，以恒定"速度" $\hat{v} \in \mathfrak{g}$ 运动：

$$\dot{X}(t) = X(t) \hat{v}$$

这个 ODE 的解是：

$$X(t) = \exp(\hat{v} t)$$

其中矩阵指数通过 Taylor 级数定义：

$$\exp(\hat{v} t) = I + \hat{v}t + \frac{(\hat{v}t)^2}{2!} + \frac{(\hat{v}t)^3}{3!} + \cdots = \sum_{k=0}^{\infty} \frac{(\hat{v}t)^k}{k!}$$

验证这确实是解：$\frac{d}{dt}\exp(\hat{v}t) = \hat{v} + \hat{v}^2 t + \frac{\hat{v}^3 t^2}{2!} + \cdots = \hat{v} \cdot \exp(\hat{v}t) = \exp(\hat{v}t) \cdot \hat{v}$。最后一个等号成立是因为 $\hat{v}$ 和 $\exp(\hat{v}t)$ 可交换（$\exp(\hat{v}t)$ 是 $\hat{v}$ 的幂级数，和 $\hat{v}$ 本身当然可交换）。

如果初始条件不是单位元而是 $X(0) = X_0$，则解为：

$$X(t) = X_0 \cdot \exp(\hat{v}t)$$

这就是 SLAM 中位姿更新公式 $T_{k+1} = T_k \cdot \exp(\hat{\xi}\Delta t)$ 的数学来源。

### Rodrigues 公式的完整推导

现在我们具体推导 SO(3) 的指数映射闭式解。目标是从 Taylor 级数 $\exp([\theta]_\times) = \sum_{k=0}^{\infty} \frac{[\theta]_\times^k}{k!}$ 化简为紧凑的公式。

**Step 1：分解为轴和角**

将旋转向量分解为 $\theta = u\theta$，其中 $u = \theta / \|\theta\|$ 是单位旋转轴，$\theta = \|\theta\|$ 是旋转角度。则 $[\theta]_\times = \theta [u]_\times$。

**Step 2：计算 $[u]_\times$ 的幂次**

首先计算 $[u]_\times^2$。利用反对称矩阵的性质（可以直接矩阵乘法验证，或利用 $[u]_\times v = u \times v$ 及向量三重积公式 $u \times (u \times v) = u(u^T v) - v(u^T u) = uu^T v - v$）：

$$[u]_\times^2 = uu^T - I$$

其中 $u^T u = 1$（因为 $u$ 是单位向量）。

然后计算 $[u]_\times^3$：

$$[u]_\times^3 = [u]_\times \cdot [u]_\times^2 = [u]_\times (uu^T - I) = [u]_\times uu^T - [u]_\times$$

注意 $[u]_\times u = u \times u = 0$，所以 $[u]_\times uu^T = 0$。因此：

$$[u]_\times^3 = -[u]_\times$$

这是一个关键的**循环性质**！它意味着更高次幂可以递推得到：

| 幂次 $k$ | $[u]_\times^k$ |
|---------|----------------|
| 0 | $I$ |
| 1 | $[u]_\times$ |
| 2 | $uu^T - I$ |
| 3 | $-[u]_\times$ |
| 4 | $-[u]_\times^2 = I - uu^T$ |
| 5 | $[u]_\times$ |
| 6 | $[u]_\times^2$ |
| ... | 周期为 2（奇数幂 $\propto [u]_\times$，偶数幂 $\propto [u]_\times^2$） |

**Step 3：代入 Taylor 级数并分组**

$$\exp(\theta [u]_\times) = I + \theta [u]_\times + \frac{\theta^2}{2!} [u]_\times^2 + \frac{\theta^3}{3!} [u]_\times^3 + \frac{\theta^4}{4!} [u]_\times^4 + \cdots$$

用循环性质替换高次幂：

$$= I + \theta [u]_\times + \frac{\theta^2}{2!} [u]_\times^2 - \frac{\theta^3}{3!} [u]_\times - \frac{\theta^4}{4!} [u]_\times^2 + \frac{\theta^5}{5!} [u]_\times + \cdots$$

按 $[u]_\times$ 和 $[u]_\times^2$ 分组：

奇数幂项（含 $[u]_\times$）：

$$\left(\theta - \frac{\theta^3}{3!} + \frac{\theta^5}{5!} - \cdots\right) [u]_\times = \sin\theta \cdot [u]_\times$$

偶数幂项（含 $[u]_\times^2$）：

$$\left(\frac{\theta^2}{2!} - \frac{\theta^4}{4!} + \frac{\theta^6}{6!} - \cdots\right) [u]_\times^2 = (1 - \cos\theta) \cdot [u]_\times^2$$

**Step 4：合并为 Rodrigues 公式**

$$\boxed{R = \exp([\theta]_\times) = I + \sin\theta \cdot [u]_\times + (1 - \cos\theta) \cdot [u]_\times^2}$$

其中 $u = \theta / \|\theta\|$，$\theta = \|\theta\|$。

这个公式的优美之处在于：它把一个无穷级数化简为只含三角函数的有限表达式。计算复杂度从无穷降到 $O(1)$。这就是为什么 Rodrigues 公式是 SLAM 中最重要的数学工具之一——每次位姿更新都要调用它。

### 大写 Exp 与 Log

为了简化记号，Sola 论文引入了大写的 Exp/Log：

**大写 Exp**：$\text{Exp}: \mathbb{R}^m \to \mathcal{M}$ 是小写 exp 与 hat 的组合：

$$\text{Exp}(\tau) \triangleq \exp(\hat{\tau})$$

对 SO(3)：$\text{Exp}(\theta) = \exp([\theta]_\times) = \text{Rodrigues}(\theta)$

输入是 $\mathbb{R}^3$ 中的旋转向量，输出是 SO(3) 旋转矩阵。这让我们在编程时直接用向量，不需要手动构造反对称矩阵。

**大写 Log**：$\text{Log}: \mathcal{M} \to \mathbb{R}^m$ 是 vee 与小写 log 的组合：

$$\text{Log}(X) \triangleq \log(X)^\vee$$

对 SO(3)：从旋转矩阵 $R$ 提取旋转向量 $\theta = \theta u$：

$$\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right), \quad u = \frac{1}{2\sin\theta}\begin{bmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{bmatrix}$$

| 映射 | 记号 | 输入 | 输出 | 关系 |
|------|------|------|------|------|
| 小写 exp | $\exp(\hat{\tau})$ | 李代数矩阵 $\hat{\tau} \in \mathfrak{g}$ | 群元素 $X \in G$ | 矩阵级数 |
| 大写 Exp | $\text{Exp}(\tau)$ | 向量 $\tau \in \mathbb{R}^m$ | 群元素 $X \in G$ | $\exp \circ \text{hat}$ |
| 小写 log | $\log(X)$ | 群元素 $X \in G$ | 李代数矩阵 $\hat{\tau} \in \mathfrak{g}$ | exp 的逆 |
| 大写 Log | $\text{Log}(X)$ | 群元素 $X \in G$ | 向量 $\tau \in \mathbb{R}^m$ | $\text{vee} \circ \log$ |

在 manif 和 Sophus 的代码中，API 对应的是**大写** Exp/Log——输入输出都是向量而不是矩阵。

### SE(3) 的指数映射

SE(3) 的指数映射比 SO(3) 复杂，因为旋转和平移不是独立的——旋转会影响平移的路径。

给定 twist 向量 $\tau = [\rho, \theta]^T \in \mathbb{R}^6$，SE(3) 的指数映射为：

$$M = \text{Exp}(\tau) = \begin{bmatrix} \text{Exp}(\theta) & V(\theta)\rho \\ 0^T & 1 \end{bmatrix}$$

其中 $\text{Exp}(\theta)$ 是 SO(3) 的 Rodrigues 公式，$V(\theta)$ 是一个 $3\times3$ 矩阵：

$$V(\theta) = I + \frac{1 - \cos\|\theta\|}{\|\theta\|^2} [\theta]_\times + \frac{\|\theta\| - \sin\|\theta\|}{\|\theta\|^3} [\theta]_\times^2$$

**$V(\theta)$ 的直觉含义**

为什么平移部分不是简单的 $\rho$ 而是 $V(\theta)\rho$？考虑一个刚体以恒定 twist $[\rho, \theta]$ 运动。如果没有旋转（$\theta = 0$），刚体走直线，位移就是 $\rho$。但如果同时有旋转，刚体走的是一条弯曲的螺旋路径。$V(\theta)$ 正是"修正"了这个弯曲效应——它把"如果走直线的位移 $\rho$"修正为"实际走弯曲路径后的位移 $V\rho$"。

当 $\theta \to 0$ 时，$V(\theta) \to I$（没有旋转就不需要修正）。当 $\theta$ 增大时，$V(\theta)$ 与 $I$ 的差异越来越大，修正效应越强。

> 这个 $V(\theta)$ 并非 SE(3) 特有的新构造：它在数学上精确等于 SO(3) 的左 Jacobian $\mathbf{J}_l(\theta)$。两者的 Taylor 级数逐项相同（下文推导中可直接验证），其深层原因将在 23.9 节的右 Jacobian $\mathbf{J}_r$ 推导（基于 BCH 公式）中阐明。完整的等式证明与几何解释详见 `01_数学/20_微分几何与李群/30_李群基础与SO3_SE3.md` 第 11 节。

**推导 $V(\theta)$**

$V(\theta)$ 的推导来自 SE(3) 指数映射的 Taylor 级数展开。$\mathfrak{se}(3)$ 元素的 $n$ 次幂具有特殊结构：

$$\hat{\tau}^n = \begin{bmatrix} [\theta]_\times^n & [\theta]_\times^{n-1}\rho \\ 0^T & 0 \end{bmatrix}$$

将 $\exp(\hat{\tau}) = \sum_{n=0}^{\infty} \frac{\hat{\tau}^n}{n!}$ 展开，旋转块给出 SO(3) 的 Rodrigues 公式，平移块给出：

$$t = \left(\sum_{n=1}^{\infty} \frac{[\theta]_\times^{n-1}}{n!}\right) \rho = \left(I + \frac{[\theta]_\times}{2!} + \frac{[\theta]_\times^2}{3!} + \cdots\right) \rho = V(\theta)\rho$$

利用与 Rodrigues 公式相同的循环性质 $[u]_\times^3 = -[u]_\times$，将级数化简为三角函数形式即得 $V(\theta)$ 的闭式表达。

### SE(3) 的对数映射

已知 SE(3) 元素 $M = (R, t)$，求 twist $\tau = [\rho, \theta]^T$：

**Step 1**：提取旋转部分 $\theta = \text{Log}(R)$（用 SO(3) 的 Log）

**Step 2**：计算 $V(\theta)$，然后 $\rho = V(\theta)^{-1} t$

$V(\theta)$ 的逆为：

$$V(\theta)^{-1} = I - \frac{1}{2}[\theta]_\times + \left(\frac{1}{\|\theta\|^2} - \frac{1 + \cos\|\theta\|}{2\|\theta\|\sin\|\theta\|}\right) [\theta]_\times^2$$

这个公式在 $\|\theta\| \to 0$ 和 $\|\theta\| \to \pi$ 时需要特殊处理（Taylor 展开），manif 和 Sophus 内部都做了这些数值处理。

### 指数映射的性质

指数映射有两个重要的代数性质：

**性质 1**：$\exp((t+s)\hat{\tau}) = \exp(t\hat{\tau}) \cdot \exp(s\hat{\tau})$

这说明沿同一方向 $\hat{\tau}$ 运动 $t$ 秒再运动 $s$ 秒，等于直接运动 $t+s$ 秒。注意：这只在两个 exp 的参数**方向相同**（只差一个标量倍数）时成立。对于不同方向的切向量，$\exp(\hat{\tau}_1) \cdot \exp(\hat{\tau}_2) \neq \exp(\hat{\tau}_1 + \hat{\tau}_2)$——这就是 BCH 公式要解决的问题。

**性质 2**：$\exp(-\hat{\tau}) = \exp(\hat{\tau})^{-1}$

反方向运动等于原运动的逆。对 SO(3)：沿 $-\theta$ 旋转等于沿 $\theta$ 旋转的逆（反向旋转）。

### manif 代码

```cpp
#include <manif/SO3.h>
#include <manif/SE3.h>

// === SO(3) 的 Exp/Log ===
Eigen::Vector3d theta(0.1, 0.2, 0.3);  // 旋转向量

// 大写 Exp：R^3 -> SO(3)
manif::SO3d R = manif::SO3d::Exp(theta);

// 大写 Log：SO(3) -> R^3
Eigen::Vector3d theta_recovered = R.log().coeffs();
// 验证往返一致性
assert((theta - theta_recovered).norm() < 1e-12);

// === SE(3) 的 Exp/Log ===
Eigen::Matrix<double, 6, 1> tau;
tau << 0.5, 1.0, -0.3, 0.1, 0.2, 0.3;  // [rho, theta]

// 大写 Exp：R^6 -> SE(3)
manif::SE3d T = manif::SE3d::Exp(tau);

// 大写 Log：SE(3) -> R^6
Eigen::Matrix<double, 6, 1> tau_recovered = T.log().coeffs();
assert((tau - tau_recovered).norm() < 1e-10);

// === 验证性质 1：exp((t+s)*tau) = exp(t*tau) * exp(s*tau) ===
double t = 0.3, s = 0.7;
manif::SE3d lhs = manif::SE3d::Exp((t + s) * tau);
manif::SE3d rhs = manif::SE3d::Exp(t * tau) * manif::SE3d::Exp(s * tau);
assert((lhs.inverse() * rhs).log().coeffs().norm() < 1e-10);

// === 验证性质 2：exp(-tau) = exp(tau)^{-1} ===
manif::SE3d exp_neg = manif::SE3d::Exp(-tau);
manif::SE3d exp_inv = manif::SE3d::Exp(tau).inverse();
assert((exp_neg.inverse() * exp_inv).log().coeffs().norm() < 1e-12);
```

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：$\theta \to 0$ 时直接用 Rodrigues 公式导致除零**

- **错误做法**：实现 Rodrigues 时不检查 $\|\theta\|$ 是否接近零，直接计算 $\sin\theta / \theta$ 和 $(1-\cos\theta)/\theta^2$
- **现象**：当 $\|\theta\| < 10^{-15}$ 时输出 NaN，当 $\|\theta\| \approx 10^{-8}$ 时精度严重下降
- **根本原因**：$\sin\theta / \theta$ 在 $\theta = 0$ 处虽然有极限值 $1$，但浮点计算 $\sin(10^{-15}) / 10^{-15}$ 会因为分子分母都接近零而产生大的相对误差
- **正确做法**：当 $\|\theta\| < \epsilon$（$\epsilon \approx 10^{-10}$）时使用 Taylor 展开：$\sin\theta/\theta \approx 1 - \theta^2/6$，$(1-\cos\theta)/\theta^2 \approx 1/2 - \theta^2/24$。manif 内部已经做了这个处理

**💡 概念误区：混淆大写 Exp/Log 和小写 exp/log**

- **新手想法**："Exp 和 exp 不就是同一个东西吗？"
- **实际上**：$\exp$ 的输入是李代数矩阵（$\hat{\tau} \in \mathfrak{g}$），$\text{Exp}$ 的输入是向量（$\tau \in \mathbb{R}^m$）。关系是 $\text{Exp}(\tau) = \exp(\hat{\tau})$。在代码中我们几乎总是用大写 Exp（因为处理向量比处理矩阵方便），但在数学推导中两者的区别很重要——很多公式（如 BCH 公式、伴随性质）是用小写 exp 写的
- **自检方法**：问自己"这个函数的输入是向量还是矩阵？"——向量用 Exp，矩阵用 exp

**🧠 思维陷阱：认为 $V(\theta)$ 只是一个"技术细节"**

- **新手想法**："SE(3) 的 exp 不就是旋转部分用 Rodrigues、平移部分直接用 $\rho$ 吗？$V(\theta)$ 只是一个小修正"
- **实际上**：$V(\theta)$ 是 SE(3) 指数映射的核心。如果忽略它（令 $t = \rho$），你得到的变换会有 $O(\|\theta\|)$ 量级的误差。对于 SLAM 中常见的每帧 $5°\sim 10°$ 的旋转，这个误差可以达到几厘米——足以让 ICP 不收敛。$V(\theta)$ 不是"小修正"，它是旋转和平移耦合效应的精确表达

### 练习

**练习 23.4.1**（⭐⭐）：手推 Rodrigues 公式中 $[u]_\times^3 = -[u]_\times$ 的证明。取 $u = [0, 0, 1]^T$，分别计算 $[u]_\times$、$[u]_\times^2$、$[u]_\times^3$，验证循环性质。然后对一般的单位向量 $u$ 证明（提示：利用 $[u]_\times^2 = uu^T - I$ 和 $[u]_\times u = 0$）。

**练习 23.4.2**（⭐⭐⭐）：用 manif 数值验证 SE(3) 指数映射中 $V(\theta)$ 的作用。构造一个 twist $\tau = [1.0, 0.5, -0.3, 0.0, 0.0, 0.8]$（注意有显著的旋转分量 $\theta_z = 0.8$ rad $\approx 46°$）。计算 $T = \text{Exp}(\tau)$，提取平移部分 $t$。然后比较 $t$ 与 twist 的平移分量 $\rho = [1.0, 0.5, -0.3]^T$。计算 $\|t - \rho\| / \|\rho\|$，这就是"忽略 $V(\theta)$"导致的相对误差。

---

## 23.5 Plus/Minus 算子 ⭐⭐⭐

### 这一节解决什么问题

前面我们知道了 exp 映射可以把切空间中的向量映射到群元素。但在 SLAM 优化中，我们面临一个更具体的问题：如何对一个**已有的**群元素施加一个小的扰动？优化器计算出来的增量 $\Delta\xi \in \mathbb{R}^6$ 是切空间中的向量——怎么把它"加"到当前位姿 $T$ 上得到更新后的位姿？普通的加法 $T + \Delta\xi$ 在流形上没有意义。$\oplus$ 和 $\ominus$ 算子正是为此定义的。

### 动机：优化器需要"加法"和"减法"

考虑 SLAM 后端优化的迭代过程：

$$T_{k+1} = T_k \oplus \Delta\xi_k$$

这里 $T_k$ 是当前的位姿估计（SE(3) 群元素），$\Delta\xi_k \in \mathbb{R}^6$ 是 Gauss-Newton 求解出的增量。$\oplus$ 定义了如何把一个切空间增量"加"到流形元素上。

同样，计算两个位姿之间的"差异"也需要定义：

$$\Delta\xi = T_2 \ominus T_1$$

$\ominus$ 把两个群元素之间的"距离"提取到切空间中，作为一个向量。这个向量可以被优化器理解和处理（因为优化器工作在欧几里得空间中）。

### 右加（Right-$\oplus$）

右加定义为扰动在群元素的**右边**（即在 body/局部坐标系中施加扰动）：

$$Y = X \oplus {}^X\tau \triangleq X \circ \text{Exp}({}^X\tau)$$

其中 ${}^X\tau \in \mathbb{R}^m$ 是在 $X$ 的局部坐标系中表达的切向量。上标 $X$ 强调这个切向量是在 $X$ 的坐标系中定义的。

**几何直觉**：想象你站在某个位置朝某个方向（这就是 $X$），然后你在自己的 body 坐标系中做一个小位移 ${}^X\tau$（比如"向前走 1 步，右转 5 度"）。右加就是这个操作。

### 右减（Right-$\ominus$）

右减是右加的逆操作：

$${}^X\tau = Y \ominus X \triangleq \text{Log}(X^{-1} \circ Y)$$

**几何直觉**：从 $X$ 的视角看，$Y$ 相对于自己在 body 坐标系中的"偏差"是多少？

### 左加（Left-$\oplus$）

左加定义为扰动在群元素的**左边**（即在 world/全局坐标系中施加扰动）：

$$Y = {}^\varepsilon\tau \oplus X \triangleq \text{Exp}({}^\varepsilon\tau) \circ X$$

其中 ${}^\varepsilon\tau$ 是在全局坐标系中表达的切向量。上标 $\varepsilon$（代表单位元/全局坐标系）强调这个切向量是在世界坐标系中定义的。

**几何直觉**：先在世界坐标系中施加一个变换 $\text{Exp}({}^\varepsilon\tau)$（比如"在世界坐标系中向北走 1 步，绕世界 z 轴转 5 度"），然后再应用原来的 $X$。

### 左减（Left-$\ominus$）

$${}^\varepsilon\tau = Y \ominus X \triangleq \text{Log}(Y \circ X^{-1})$$

注意左减和右减的公式区别：
- 右减：$\text{Log}(X^{-1} \circ Y)$ — 先求 $X$ 的逆，再右乘 $Y$
- 左减：$\text{Log}(Y \circ X^{-1})$ — 先乘 $Y$，再右乘 $X$ 的逆

### 局部坐标系 vs 全局坐标系：什么时候用哪个？

| 场景 | 推荐约定 | 原因 |
|------|---------|------|
| IMU 预积分 | 右扰动（body 系） | IMU 测量的是 body 系下的加速度和角速度 |
| Ceres + manif | 右扰动 | manif 默认右扰动，Jacobian 天然匹配 |
| GTSAM 因子 | 右扰动 | GTSAM 的 `retract(v)` 实现为 `this * Exp(v)`，即右乘 |
| ORB-SLAM3 | 右扰动 | 使用 Sophus（右扰动约定） |
| FAST-LIO | 右扰动 | 使用自定义的 SO(3)/SE(3)，右乘更新 |

**一般原则**：manif 默认使用右扰动，本教程也以右扰动为主。如果你使用 GTSAM，需要注意其扰动约定可能与你的期望不同——23.8 节将详细讨论左右扰动约定和转换方法。

### manif 代码

manif 使用**右加/右减**作为默认操作：

```cpp
#include <manif/SE3.h>

manif::SE3d T = manif::SE3d::Random();

// === Right-plus (⊕): T ⊕ delta = T * Exp(delta) ===
Eigen::Matrix<double, 6, 1> delta;
delta << 0.01, 0.02, -0.01, 0.005, -0.003, 0.01;  // 小增量 [rho, omega]
manif::SE3d T_new = T.plus(manif::SE3Tangentd(delta));
// 等价写法
manif::SE3d T_new2 = T + manif::SE3Tangentd(delta);
// 底层实现：T * SE3d::Exp(delta)

// === Right-minus (⊖): delta = T2 ⊖ T1 = Log(T1^{-1} * T2) ===
manif::SE3d T2 = manif::SE3d::Random();
manif::SE3Tangentd diff = T2.minus(T);
// 等价写法
manif::SE3Tangentd diff2 = T2 - T;
// 底层实现：(T.inverse() * T2).log()

// === 往返一致性验证 ===
manif::SE3d T_roundtrip = T.plus(T2.minus(T));
// T_roundtrip 应该等于 T2
assert((T_roundtrip.inverse() * T2).log().coeffs().norm() < 1e-10);
```

### 具体例子：扰动一个相机位姿

假设 SLAM 后端优化给出了一个 6D 增量 $\Delta\xi = [0.01, -0.02, 0.03, 0.005, -0.003, 0.001]^T$（单位：米和弧度）。当前位姿 $T_{WC}$ 表示相机相对世界的变换。

```cpp
// 当前位姿
manif::SE3d T_WC = /* 从 SLAM 前端获得 */;

// 优化器计算的增量（右扰动）
Eigen::Matrix<double, 6, 1> delta_xi;
delta_xi << 0.01, -0.02, 0.03, 0.005, -0.003, 0.001;

// 更新位姿
manif::SE3d T_WC_updated = T_WC + manif::SE3Tangentd(delta_xi);
// 数学含义：T_WC_updated = T_WC * Exp(delta_xi)
// 物理含义：在相机的 body 坐标系中施加一个小位移和小旋转

// 如果目标框架使用左扰动（注意：GTSAM 实际使用右扰动，此处仅作转换示例）：
// T_WC_updated = Exp(delta_xi) * T_WC
// 物理含义：在世界坐标系中施加一个小位移和小旋转
// manif 中需要手动实现：
manif::SE3d T_WC_left = manif::SE3d::Exp(delta_xi) * T_WC;
```

### 与优化的联系

$\oplus$ 算子是 SLAM 优化器与李群流形之间的桥梁。在 Ceres Solver 中，`LocalParameterization`（旧版）或 `Manifold`（新版）接口的核心就是定义 $\oplus$：

- Ceres 在欧几里得空间 $\mathbb{R}^m$ 中计算增量 $\Delta\xi$
- 通过 $\oplus$ 把增量施加到流形上的当前估计
- 结果自动满足流形约束（$R^T R = I$ 等）

在 ESKF（Error-State Kalman Filter，如 FAST-LIO 使用的）中，状态估计分为"名义状态"（nominal state）$\bar{X}$ 和"误差状态"（error state）$\delta x$：

$$X = \bar{X} \oplus \delta x$$

ESKF 用线性卡尔曼滤波器估计 $\delta x$（因为 $\delta x$ 是欧几里得空间中的向量），然后通过 $\oplus$ 更新名义状态。这正是 $\oplus$ 在 SLAM 中最重要的应用。

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：把 $\oplus$ 实现为普通向量加法**

- **错误做法**：`T_new.translation() = T.translation() + delta.head<3>(); T_new.rotation() = T.rotation() * SO3d::Exp(delta.tail<3>());`
- **现象**：当旋转增量较大时，平移部分的更新方向错误
- **根本原因**：$\oplus$ 不是分别更新旋转和平移。正确的公式 $T \cdot \text{Exp}(\delta)$ 中，$\text{Exp}(\delta)$ 的平移部分是 $V(\theta)\rho$（而不是 $\rho$），旋转会影响平移的更新方向。如果你分别更新旋转和平移，就丢失了 $V(\theta)$ 的修正
- **正确做法**：用 `T.plus(delta)` 或 `T + delta`，让 manif 处理内部的耦合效应

**💡 概念误区：认为右加和左加的结果只是"差一个旋转"**

- **新手想法**："右加和左加不就是一个用 body 系、一个用 world 系吗？它们的结果应该差不多吧"
- **实际上**：当扰动 $\tau$ 很小时（优化中通常如此），右加和左加的差异确实是二阶小量，所以"差不多"。但数学上，$X \cdot \text{Exp}({}^X\tau) \neq \text{Exp}({}^\varepsilon\tau) \cdot X$，除非 ${}^\varepsilon\tau = \text{Ad}_X \cdot {}^X\tau$（通过伴随矩阵转换）。两者的 Jacobian 也不同。在 SLAM 优化中选错约定会导致收敛变慢甚至发散——23.8 节详细讨论这个问题

### 练习

**练习 23.5.1**（⭐⭐）：用 manif 验证 $\oplus$ 和 $\ominus$ 的互逆性。随机生成 10 对 SE(3) 元素 $(T_1, T_2)$，验证 $T_1 \oplus (T_2 \ominus T_1) = T_2$（误差小于 $10^{-10}$）。然后随机生成 10 个 $(T, \delta)$（$\|\delta\| < 0.5$），验证 $(T \oplus \delta) \ominus T = \delta$。

**练习 23.5.2**（⭐⭐⭐）：比较右加和左加的差异。取 $T = (R_z(45°), [1,0,0]^T)$ 和 $\delta = [0, 0, 0, 0, 0, 0.1]^T$（纯旋转扰动 $\Delta\theta_z = 0.1$ rad）。分别计算右加 $T_R = T \cdot \text{Exp}(\delta)$ 和左加 $T_L = \text{Exp}(\delta) \cdot T$。比较 $T_R$ 和 $T_L$ 的旋转和平移部分，分析差异的来源（提示：旋转部分应该相同，但平移部分不同——为什么？）。

---

## 23.6 伴随矩阵（Adjoint）详解 ⭐⭐⭐

### 这一节解决什么问题

23.5 节中我们定义了左加和右加，它们分别在全局坐标系和局部坐标系中施加扰动。但在实际 SLAM 工程中，你经常需要在两种坐标系之间转换——比如你用 manif（右扰动）计算了一个 Jacobian，但需要传给使用不同约定的优化器。伴随矩阵（Adjoint matrix）正是完成这个转换的工具。23.1 节简要提到了伴随表示，本节从定义出发完整推导 SO(3) 和 SE(3) 的伴随矩阵。

### 动机：为什么不能简单地左乘/右乘切换

假设你有一个右扰动下的 Jacobian $J_R$，想转换成左扰动下的 $J_L$。直觉上，可能会想"从右边移到左边不就是乘个旋转矩阵吗？"但实际上对 SE(3) 来说，$J_L$ 和 $J_R$ 之间的关系不是一个简单的 $3\times3$ 旋转，而是一个 $6\times6$ 矩阵——伴随矩阵 $\text{Ad}_X$。这是因为 SE(3) 的切空间有 6 维（3 旋转 + 3 平移），旋转分量的坐标系变换会影响平移分量（旋转和平移是耦合的）。

### 理论：伴随映射的定义

**定义**：伴随映射 $\text{Ad}_X: \mathfrak{g} \to \mathfrak{g}$ 定义为群元素 $X$ 对李代数元素的共轭作用：

$$\text{Ad}_X(\hat{\tau}) \triangleq X \hat{\tau} X^{-1}$$

直觉上，伴随映射把一个在"局部坐标系"中表达的切向量转换到"全局坐标系"中。

**伴随矩阵**：由于李代数 $\mathfrak{g} \cong \mathbb{R}^m$（通过 hat/vee），伴随映射可以用一个 $m \times m$ 矩阵表示。对于向量 $\tau \in \mathbb{R}^m$：

$${}^\varepsilon\tau = \text{Ad}_X \cdot {}^X\tau$$

其中 $\text{Ad}_X \in \mathbb{R}^{m \times m}$ 就是伴随矩阵。

### 关键性质：交换扰动的位置

伴随矩阵最重要的性质是它允许我们把扰动从一侧"搬"到另一侧：

$$X \oplus {}^X\tau = (\text{Ad}_X \cdot {}^X\tau) \oplus X$$

展开来说：

$$X \cdot \text{Exp}({}^X\tau) = \text{Exp}(\text{Ad}_X \cdot {}^X\tau) \cdot X$$

这个等式的证明来自伴随的定义：

$$X \cdot \exp(\hat{\tau}) \cdot X^{-1} = \exp(\text{Ad}_X(\hat{\tau})) = \exp(\widehat{\text{Ad}_X \cdot \tau})$$

两边同时右乘 $X$：

$$X \cdot \exp(\hat{\tau}) = \exp(\widehat{\text{Ad}_X \cdot \tau}) \cdot X$$

这就是上面的等式。

**为什么这在 SLAM 中重要？** 因为它给出了左扰动和右扰动 Jacobian 之间的转换公式：

$$J_L = \text{Ad}_{f(X)} \cdot J_R$$

当你需要在右扰动和左扰动 Jacobian 之间转换时（例如移植论文中使用左扰动的公式到 manif 框架中），只需左乘一个伴随矩阵。

### SO(3) 的伴随矩阵

对 SO(3)，推导非常直接。设 $R \in SO(3)$，$[\omega]_\times \in \mathfrak{so}(3)$：

$$\text{Ad}_R([\omega]_\times) = R [\omega]_\times R^{-1} = R [\omega]_\times R^T$$

利用 23.3 节中的性质 $R[\omega]_\times R^T = [R\omega]_\times$：

$$\text{Ad}_R([\omega]_\times) = [R\omega]_\times$$

取 vee 得到：

$$\text{Ad}_R \cdot \omega = R\omega$$

所以 **SO(3) 的伴随矩阵就是旋转矩阵本身**：

$$\boxed{\text{Ad}_R = R}$$

这个结果非常直观：把一个角速度从 body 系转换到 world 系，就是用旋转矩阵旋转一下。

### SE(3) 的伴随矩阵

SE(3) 的推导稍复杂。设 $M = \begin{bmatrix} R & t \\ 0^T & 1 \end{bmatrix}$，$\hat{\tau} = \begin{bmatrix} [\theta]_\times & \rho \\ 0^T & 0 \end{bmatrix}$。

**Step 1**：计算 $M \hat{\tau} M^{-1}$

$$M^{-1} = \begin{bmatrix} R^T & -R^T t \\ 0^T & 1 \end{bmatrix}$$

$$M \hat{\tau} = \begin{bmatrix} R[\theta]_\times & R\rho + t \cdot 0 \\ 0^T & 0 \end{bmatrix} + \begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$$

实际上我们需要完整计算：

$$M \hat{\tau} M^{-1} = \begin{bmatrix} R & t \\ 0^T & 1 \end{bmatrix} \begin{bmatrix} [\theta]_\times & \rho \\ 0^T & 0 \end{bmatrix} \begin{bmatrix} R^T & -R^T t \\ 0^T & 1 \end{bmatrix}$$

先算 $M \hat{\tau}$：

$$M \hat{\tau} = \begin{bmatrix} R[\theta]_\times & R\rho \\ 0^T & 0 \end{bmatrix}$$

再右乘 $M^{-1}$：

$$M \hat{\tau} M^{-1} = \begin{bmatrix} R[\theta]_\times R^T & -R[\theta]_\times R^T t + R\rho \\ 0^T & 0 \end{bmatrix}$$

利用 $R[\theta]_\times R^T = [R\theta]_\times$：

$$= \begin{bmatrix} [R\theta]_\times & -[R\theta]_\times t + R\rho \\ 0^T & 0 \end{bmatrix} = \begin{bmatrix} [R\theta]_\times & [t]_\times R\theta + R\rho \\ 0^T & 0 \end{bmatrix}$$

最后一步利用了 $-[R\theta]_\times t = [t]_\times R\theta$（因为 $a \times b = -b \times a$，即 $[a]_\times b = -[b]_\times a$）。

**Step 2**：取 vee，提取伴随矩阵

比较 $M \hat{\tau} M^{-1}$ 的结构与 $\hat{\tau}' = \begin{bmatrix} [\theta']_\times & \rho' \\ 0^T & 0 \end{bmatrix}$：

$$\theta' = R\theta, \quad \rho' = [t]_\times R\theta + R\rho$$

写成矩阵形式（注意 manif 的 $[\rho, \theta]$ 顺序）：

$$\begin{bmatrix} \rho' \\ \theta' \end{bmatrix} = \begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix} \begin{bmatrix} \rho \\ \theta \end{bmatrix}$$

因此：

$$\boxed{\text{Ad}_M = \begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix} \in \mathbb{R}^{6\times6}}$$

注意这个矩阵的分块结构：左下角是零（旋转不受平移影响），右上角是 $[t]_\times R$（平移受旋转影响），对角块都是 $R$。这反映了 SE(3) 的半直积结构。

### 伴随矩阵的性质

| 性质 | 公式 | 含义 |
|------|------|------|
| 逆元 | $\text{Ad}_{X^{-1}} = \text{Ad}_X^{-1}$ | 逆变换的伴随 = 伴随的逆 |
| 同态性 | $\text{Ad}_{XY} = \text{Ad}_X \cdot \text{Ad}_Y$ | 组合变换的伴随 = 伴随的组合 |
| 单位元 | $\text{Ad}_E = I_{m \times m}$ | 单位元的伴随是单位矩阵 |

同态性的含义特别重要：它说明伴随映射 $X \mapsto \text{Ad}_X$ 是一个群同态（group homomorphism），从 $G$ 到 $GL(m)$（$m \times m$ 可逆矩阵群）。这保证了多次坐标系变换可以通过矩阵乘法链接——$\text{Ad}_{T_1 T_2 T_3} = \text{Ad}_{T_1} \text{Ad}_{T_2} \text{Ad}_{T_3}$。

### manif 代码

```cpp
#include <manif/SE3.h>

manif::SE3d T = manif::SE3d::Random();

// 获取 6x6 伴随矩阵
Eigen::Matrix<double, 6, 6> Ad_T = T.adj();

// 验证性质：Ad_{T^{-1}} = Ad_T^{-1}
Eigen::Matrix<double, 6, 6> Ad_T_inv = T.inverse().adj();
Eigen::Matrix<double, 6, 6> Ad_T_inv_check = Ad_T.inverse();
assert((Ad_T_inv - Ad_T_inv_check).norm() < 1e-10);

// 验证同态性：Ad_{T1*T2} = Ad_{T1} * Ad_{T2}
manif::SE3d T1 = manif::SE3d::Random();
manif::SE3d T2 = manif::SE3d::Random();
Eigen::Matrix<double, 6, 6> Ad_T12 = (T1 * T2).adj();
Eigen::Matrix<double, 6, 6> Ad_T12_check = T1.adj() * T2.adj();
assert((Ad_T12 - Ad_T12_check).norm() < 1e-10);

// 验证交换扰动位置：T * Exp(delta) == Exp(Ad_T * delta) * T
Eigen::Matrix<double, 6, 1> delta;
delta << 0.01, 0.02, -0.01, 0.005, -0.003, 0.01;
manif::SE3d right_plus = T * manif::SE3d::Exp(delta);
manif::SE3d left_plus = manif::SE3d::Exp(Ad_T * delta) * T;
assert((right_plus.inverse() * left_plus).log().coeffs().norm() < 1e-10);

// 实际应用：将右扰动 Jacobian 转换为左扰动 Jacobian（用于与使用左扰动约定的论文/代码对接）
Eigen::Vector3d point(1.0, 2.0, 3.0);
Eigen::Matrix<double, 3, 6> J_act_T_right;
Eigen::Matrix<double, 3, 3> J_act_p;
T.act(point, J_act_T_right, J_act_p);

// J_act_T_right 是右扰动 Jacobian
// 转换为左扰动 Jacobian（供使用左扰动约定的论文/代码使用，注意 GTSAM 本身用右扰动）：
Eigen::Matrix<double, 3, 6> J_act_T_left = J_act_T_right * T.adj().inverse();
// 或等价地：
Eigen::Matrix<double, 3, 6> J_act_T_left2 = J_act_T_right * T.inverse().adj();
```

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：忘记 SE(3) 伴随矩阵中 $[t]_\times R$ 项**

- **错误做法**：手动构造 SE(3) 伴随矩阵时，写 $\text{Ad}_M = \begin{bmatrix} R & 0 \\ 0 & R \end{bmatrix}$（忽略了 $[t]_\times R$ 项）
- **现象**：对纯旋转变换（$t = 0$）结果正确，但对有平移的变换结果错误。左/右扰动 Jacobian 转换不一致
- **根本原因**：SE(3) 不是 SO(3) 和 $\mathbb{R}^3$ 的直积——旋转和平移是耦合的。$[t]_\times R$ 项反映了"旋转一个切向量时，平移分量也会随之改变"
- **正确做法**：用 manif 的 `T.adj()` 方法获取完整的 $6\times6$ 伴随矩阵，不要手动构造

**💡 概念误区：认为 SO(3) 的伴随矩阵"太简单了，不重要"**

- **新手想法**："$\text{Ad}_R = R$，这也太 trivial 了。伴随的概念是不是被过度复杂化了？"
- **实际上**：SO(3) 的伴随恰好等于旋转矩阵本身，这是因为 SO(3) 是**紧致**的且其李代数 $\mathfrak{so}(3)$ 是 $\mathbb{R}^3$（维度等于群的维度）。对于 SE(3)，伴随矩阵就不再是 trivial 的了——它是一个 $6\times6$ 矩阵，包含了旋转-平移耦合信息。理解 SO(3) 的简单情况有助于理解伴随的几何含义，但不要因此低估 SE(3) 伴随的复杂性
- **延伸**：对于更复杂的群（如 SE_2(3)，即"扩展位姿群"，用于 IMU 预积分中包含速度），伴随矩阵是 $9\times9$ 的，结构更复杂

### 练习

**练习 23.6.1**（⭐⭐）：手动构造一个 SE(3) 伴随矩阵。取 $T = (R_z(90°), [1, 0, 0]^T)$，计算 $R$、$[t]_\times$、$[t]_\times R$，然后组装 $6\times6$ 的 $\text{Ad}_T$。用 manif 的 `T.adj()` 验证你的结果。

**练习 23.6.2**（⭐⭐⭐）：用伴随矩阵实现左/右扰动 Jacobian 的转换。以 $f(T) = T \cdot T_{\text{ref}}^{-1}$（其中 $T_{\text{ref}}$ 是一个固定的参考位姿）为例：(1) 用 manif 的 `compose` 带 Jacobian 输出计算右扰动 Jacobian $J_R$；(2) 用 $J_L = \text{Ad}_{f(T)} \cdot J_R$ 计算左扰动 Jacobian；(3) 用数值有限差分（左扰动方式）验证 $J_L$ 的正确性。

---

## 23.7 manif 库核心 API ⭐⭐

### 动机：手写 exp/log 太容易出错

上一节我们看到了 Rodrigues 公式和 SE(3) 的 exp/log 映射的数学推导。如果每次使用旋转和位姿时都要手动实现这些公式，会有几个严重的问题：

1. **边界情况处理**：$\theta \to 0$ 和 $\theta \to \pi$ 时需要特殊处理，数值上非常微妙。即使是经验丰富的工程师，也容易忽略某个边界条件。
2. **Jacobian 计算**：SLAM 优化需要 exp/log 对参数的 Jacobian，手工推导和实现这些 Jacobian 极其容易出错。
3. **群操作的一致性**：组合、逆、伴随等操作之间有严格的数学关系（如 $\text{Ad}_{T^{-1}} = \text{Ad}_T^{-1}$），手工实现很难保证所有操作之间的一致性。

manif 库就是为了解决这些问题而生的。它的配套论文《A micro Lie theory for state estimation in robotics》（Sola et al., 2018/2021）本身就是李群理论最友好的入门材料之一——**强烈建议先读论文再学库**。

### 如果不用 manif 会怎样

在 manif 出现之前（以及在不使用任何李群库的项目中），SLAM 工程师们通常的做法是：

1. 用 `Eigen::Quaterniond` 表示旋转，每次更新后手动归一化
2. 手写 exp/log 函数，每个项目都重新实现一遍
3. 用有限差分近似 Jacobian（慢且不精确）
4. 每换一个李群（从 SO(3) 到 SE(3)，或从 SE(3) 到 SE_2(3)）就要重新实现所有操作

manif 的核心设计哲学是**群无关（group-agnostic）**：用一套统一的接口（compose, inverse, log, exp, act, Jacobian）覆盖所有李群（SO(2), SO(3), SE(2), SE(3), SE_2(3) 等）。这是通过 C++ 的 CRTP（Curiously Recurring Template Pattern）实现的——所有李群共享同一个基类模板，具体的群操作由派生类提供。

### 理论应用：manif 的核心类型与操作

**安装 manif**

manif 是 header-only 库，安装非常简单：

```cpp
// CMakeLists.txt 中添加：
find_package(manif REQUIRED)
target_link_libraries(your_target PRIVATE manif::manif)
```

如果没有系统安装，也可以用 `add_subdirectory` 或 `FetchContent` 直接引入源码。manif 依赖 Eigen（>=3.3）和一个 C++14 编译器。

**SO3d 和 SE3d 的构造**

首先理解 manif 的命名约定：类型名 = 群名 + 标量类型后缀。`SO3d` 表示 "SO(3) with double"，`SE3f` 表示 "SE(3) with float"。

为什么 manif 要用后缀区分标量类型而不是模板参数？这是为了和 Eigen 的命名风格一致（`Matrix3d` vs `Matrix3f`），减少用户的认知负担。底层实现确实是模板 `SO3<double>` 和 `SO3<float>`，但日常使用中 typedef 更方便。

```cpp
#include <manif/SO3.h>
#include <manif/SE3.h>

// === SO3d 构造 ===

// 方式 A：从四元数构造（manif 内部用四元数存储 SO3）
Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
manif::SO3d R_A(q);

// 方式 B：从旋转向量（轴角）构造——通过 Exp 映射
Eigen::Vector3d rotation_vec(0.0, 0.0, M_PI / 4);
manif::SO3d R_B = manif::SO3d::Exp(rotation_vec);

// 方式 C：单位元
manif::SO3d R_identity = manif::SO3d::Identity();

// === SE3d 构造 ===

// 方式 A：从旋转 + 平移构造
Eigen::Vector3d translation(1.0, 2.0, 3.0);
manif::SE3d T_A(translation, q);

// 方式 B：从 6 维切向量（twist）构造
Eigen::Matrix<double, 6, 1> twist;
twist << 1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 4;
manif::SE3d T_B = manif::SE3d::Exp(twist);
```

这里有一个关键的设计决策值得注意：**manif 的 SE3 twist 约定是 $[\rho, \omega]$（平移在前，旋转在后）**。这和某些教科书（如 Murray, Li & Sastry 的 *A Mathematical Introduction to Robotic Manipulation*）的约定 $[\omega, \rho]$ 相反。**注意：当前版本的 Sophus 也使用 $[\rho, \omega]$（平移在前）的约定**，所以 manif 和 Sophus 的 twist 顺序实际上是**相同的**。

从数学上看，$\mathfrak{se}(3)$ 的基底可以任意排列，两种约定都是合法的。manif 和 Sophus 都选择了 $[\rho, \omega]$，这与 Sola 论文一致。

**群操作：compose, inverse, log**

群操作是 manif 的核心。理解这些操作的数学含义和代码实现之间的对应关系，是正确使用 manif 的基础。

```cpp
manif::SE3d T1 = manif::SE3d::Random();
manif::SE3d T2 = manif::SE3d::Random();

// 组合（compose）：相当于齐次矩阵乘法 T1 * T2
manif::SE3d T12 = T1.compose(T2);
manif::SE3d T12_alt = T1 * T2;  // 运算符重载，等价写法

// 逆（inverse）
manif::SE3d T1_inv = T1.inverse();

// 验证：T1 * T1^{-1} 应该等于 Identity
manif::SE3d should_be_identity = T1 * T1_inv;

// 对数映射（log）：SE3d -> SE3Tangentd（6维切向量）
manif::SE3Tangentd xi = T1.log();

// 指数映射（Exp）：SE3Tangentd -> SE3d
manif::SE3d T1_recovered = manif::SE3d::Exp(xi.coeffs());

// 两个位姿之间的"差"
// 方法：T1^{-1} * T2 的 log
manif::SE3Tangentd delta = (T1.inverse() * T2).log();
```

注意 `compose` 和运算符 `*` 的等价性。manif 同时提供两种风格的原因是：`compose` 更明确地表达了群操作的语义（这是流形上的组合，不是普通的矩阵乘法），而 `*` 更简洁、更符合线性代数的习惯。在群无关的模板代码中，建议用 `compose`；在具体的 SE3 代码中，用 `*` 更自然。

**切空间类型：Tangent**

manif 为每个李群定义了对应的切空间类型。`SO3Tangentd` 是一个 3 维向量（角速度），`SE3Tangentd` 是一个 6 维向量（twist）。切空间类型不仅仅是 `Eigen::VectorXd` 的别名——它携带了群的结构信息，支持 hat（向量到矩阵）和 vee（矩阵到向量）操作：

```cpp
manif::SO3Tangentd omega;
omega.coeffs() << 0.1, 0.2, 0.3;

// hat: 将 3 维向量映射到 3x3 反对称矩阵
Eigen::Matrix3d omega_hat = omega.hat();

// exp: 切向量 -> 群元素
manif::SO3d R = omega.exp();

// 直接访问底层数据
Eigen::Vector3d omega_vec = omega.coeffs();
```

为什么 manif 要定义一个独立的 Tangent 类型，而不是直接用 `Eigen::Vector3d`？因为 Tangent 类型知道自己属于哪个群。`SO3Tangentd` 的 `exp()` 返回 `SO3d`，`SE3Tangentd` 的 `exp()` 返回 `SE3d`——如果只用 `Eigen::VectorXd`，编译器无法知道这个向量应该被 exp 映射到哪个群。这种类型安全设计在编译期就能发现"把 SO3 的切向量当 SE3 的切向量用"这类错误。

**群作用：act**

`act` 方法将群元素"作用"于空间中的点。对 SO(3)，act 就是旋转一个 3D 点；对 SE(3)，act 就是刚体变换一个 3D 点。

```cpp
manif::SE3d T_world_body = /* ... */;
Eigen::Vector3d p_body(1.0, 0.0, 0.0);

// 将 body 坐标系中的点变换到 world 坐标系
Eigen::Vector3d p_world = T_world_body.act(p_body);
```

act 做的确实是 $Rp + t$（对 SE3 而言），但它的真正价值在于可以同时输出关于群元素和点的 Jacobian。如果你只做变换不需要 Jacobian，直接用矩阵乘法也行。但在 SLAM 优化中，你几乎总是需要 Jacobian，此时 `act` 比手动计算更简洁且不容易出错。

**Jacobian 计算——manif 的核心优势**

manif 最强大的特性是自动计算群操作的解析 Jacobian。几乎所有操作都可以返回关于输入的 Jacobian：

```cpp
manif::SE3d T1 = manif::SE3d::Random();
manif::SE3d T2 = manif::SE3d::Random();

// compose 的 Jacobian
manif::SE3d::Jacobian J_compose_T1, J_compose_T2;
manif::SE3d T12 = T1.compose(T2, J_compose_T1, J_compose_T2);
// J_compose_T1: d(T1*T2) / dT1，6x6 矩阵
// J_compose_T2: d(T1*T2) / dT2，6x6 矩阵

// act 的 Jacobian
Eigen::Vector3d point(1.0, 2.0, 3.0);
Eigen::Matrix<double, 3, 6> J_act_T;
Eigen::Matrix<double, 3, 3> J_act_p;
Eigen::Vector3d p_transformed = T1.act(point, J_act_T, J_act_p);
// J_act_T: d(T*p) / dT，3x6 矩阵
// J_act_p: d(T*p) / dp，3x3 矩阵（就是旋转矩阵 R）

// log 的 Jacobian
manif::SE3d::Jacobian J_log;
manif::SE3Tangentd xi = T1.log(J_log);
// J_log: d(log(T)) / dT，6x6 矩阵
```

这里需要理解"d(T1*T2) / dT1"是什么意思。T1 是一个流形上的元素，不能直接求导。这里的 Jacobian 实际上是：

$$J = \frac{\partial \log(T_1^{-1} \cdot f(T_1 \oplus \delta))}{\partial \delta}\bigg|_{\delta=0}$$

其中 $T_1 \oplus \delta = T_1 \cdot \exp(\delta)$（右扰动）。这就引出了下一节的核心内容。

**CRTP 设计：群无关编程**

manif 使用 CRTP 实现群无关接口。这意味着你可以写出适用于任何李群的模板函数：

```cpp
template <typename Derived>
typename Derived::Scalar compute_distance(
    const manif::LieGroupBase<Derived>& T1,
    const manif::LieGroupBase<Derived>& T2) {
    return (T1.inverse() * T2).log().coeffs().norm();
}

// 同一个函数适用于 SO3 和 SE3
manif::SO3d R1 = manif::SO3d::Random();
manif::SO3d R2 = manif::SO3d::Random();
double d_rot = compute_distance(R1, R2);

manif::SE3d T1 = manif::SE3d::Random();
manif::SE3d T2 = manif::SE3d::Random();
double d_pose = compute_distance(T1, T2);
```

这种设计的好处是：当你从 SO(3) 切换到 SE(3)，或者将来 manif 添加新的群（如 SE_2(3)），你的算法代码不需要任何修改。对比 Sophus 的设计（每个群独立实现，没有公共基类），如果你想写一个适用于 SO3 和 SE3 的函数，在 Sophus 中必须写两个重载版本或者用 `std::variant`。

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：SE3d twist 分量顺序混淆**

- **错误做法**：假设 manif 的 twist 顺序是 $[\omega, \rho]$（旋转在前，平移在后）
- **现象**：构造的 SE3d 位姿完全错误——旋转和平移互换了
- **根本原因**：manif 的 twist 约定是 $[\rho, \omega]$（平移在前，旋转在后），这和 Sola 论文的约定一致，但与某些教科书（如 Murray 的 "A Mathematical Introduction to Robotic Manipulation"）的约定相反
- **正确做法**：始终用 `manif::SE3Tangentd` 类型来构造 twist，通过语义明确的方式赋值；或者直接从旋转四元数+平移向量构造 SE3d，绕过 twist 构造

**⚠️ 编程陷阱：把 `log()` 的返回值当普通 `Eigen::VectorXd` 使用**

- **错误做法**：`Eigen::VectorXd xi = T.log();`——试图隐式转换
- **现象**：编译错误，因为 `SE3Tangentd` 不能直接隐式转换为 `Eigen::VectorXd`
- **根本原因**：`manif::SE3Tangentd` 是一个独立的类型，不是 Eigen 向量的 typedef。它包含群结构信息，支持 `hat()`、`exp()` 等操作
- **正确做法**：使用 `.coeffs()` 方法获取底层的 Eigen 向量：`Eigen::Matrix<double,6,1> xi = T.log().coeffs();`

**💡 概念误区：认为 manif 的 `act(point)` 和矩阵乘法 $Tp$ 完全等价**

- **新手想法**："act 就是做一次矩阵乘法，没什么特别的"
- **实际上**：`act` 做的确实是 $Rp + t$（对 SE3 而言），但 act 方法的真正价值在于它可以同时输出关于群元素和点的 Jacobian。如果你只是做变换不需要 Jacobian，直接用矩阵乘法也行。但在 SLAM 优化中，你几乎总是需要 Jacobian，此时 `act` 比手动计算更简洁且不容易出错
- **延伸**：对于 SO(3) 的 act，结果就是 $Rp$，Jacobian $\partial(Rp)/\partial R$ 在右扰动下等于 $-R[p]_\times$

**🧠 思维陷阱：因为 manif 有 Jacobian 就不去理解 Jacobian 的推导**

- **新手想法**："manif 能自动算 Jacobian，我只需要调用 API 就行了"
- **实际上**：manif 确实能给你正确的 Jacobian，但你需要知道这个 Jacobian 是关于什么的（右扰动下的切空间增量）、输出的是什么（目标函数在切空间中的变化），才能正确地把它传给 Ceres 或 GTSAM。如果你不理解 Jacobian 的含义，即使拿到了正确的矩阵也不知道怎么用
- **正确思维**：至少手推一次 $\partial(R \cdot p) / \partial R$（在右扰动下），确认自己理解"对流形元素求导"的含义

### 练习

**练习 23.7.1**（⭐⭐）：用 manif 实现 SE3 位姿的线性插值。给定两个位姿 $T_0, T_1$ 和比例参数 $t \in [0, 1]$，用 log/exp 实现球面线性插值（SLERP on SE3）：$T(t) = T_0 \cdot \exp(t \cdot \log(T_0^{-1} T_1))$。取 $T_0$ 为单位元，$T_1$ 为绕 z 轴旋转 $90°$ + 沿 x 轴平移 $1\text{m}$，用 10 个均匀采样的 $t$ 值计算插值结果，输出每个位姿的平移和旋转角度。

**练习 23.7.2**（⭐⭐⭐）：数值验证 manif Jacobian 的正确性。以 `SE3d::compose` 为例，计算解析 Jacobian（由 manif 给出）和数值 Jacobian（有限差分），比较误差。有限差分方法：$J_{ij} \approx \frac{\log(f(x \oplus h \cdot e_j))_i - \log(f(x))_i}{h}$，其中 $e_j$ 是第 $j$ 个单位向量，$h = 10^{-7}$。要求最大绝对误差小于 $10^{-5}$。

---

## 23.8 左扰动 vs 右扰动 ⭐⭐⭐

### 动机：同一个 Jacobian，两种完全不同的结果

假设你在写一个 Ceres 代价函数，计算"两个位姿之间的误差"：

$$e(T) = \log(T_{\text{meas}}^{-1} \cdot T)$$

你需要计算 $\partial e / \partial T$。但"$\partial / \partial T$"是什么意思？T 是流形上的点，不是向量空间中的元素，不能直接求偏导。你必须先定义一个"扰动"方向，然后对扰动参数求导。问题是，扰动有两种放法：

**右扰动**（Right perturbation，manif 的默认约定）：

$$\frac{\partial e}{\partial \delta}\bigg|_{\delta=0}, \quad \text{where } T \oplus \delta = T \cdot \exp(\delta)$$

扰动 $\delta$ 作用在 T 的**右边**（即在 body 坐标系中施加扰动）。

**左扰动**（Left perturbation）：

$$\frac{\partial e}{\partial \delta}\bigg|_{\delta=0}, \quad \text{where } T \oplus \delta = \exp(\delta) \cdot T$$

扰动 $\delta$ 作用在 T 的**左边**（即在 world 坐标系中施加扰动）。

这两种约定给出的 Jacobian 是**不同的矩阵**。如果你用右扰动约定计算的 Jacobian 传给一个使用左扰动约定的优化器，结果就是错误的——优化器可能不收敛，或者收敛到错误的解。

### 如果不分清左/右扰动会怎样

这个问题在实际 SLAM 开发中极其常见且极其难调试。考虑这个场景：

假设你在一个使用左扰动约定的框架中写了一个 ICP 代价函数，Jacobian 是 $J_L$。然后你把这个 Jacobian 传给一个使用右扰动约定的优化器。优化器期望的是 $J_R$，但你给了 $J_L$。

会发生什么？优化器不会报错——它收到的是一个 $6\times6$ 矩阵，格式完全正确。但梯度方向是错的。在小角度时，$J_L \approx J_R$（因为 $\text{Ad} \approx I$），所以优化可能看起来"差不多"能工作。但当旋转较大时（如初始化误差超过 $30°$），$J_L$ 和 $J_R$ 之间的差异变得显著，优化器开始"打滑"——迭代次数剧增，最终可能收敛到一个次优解或完全发散。

最坑的地方在于：这种 bug 通过单元测试几乎发现不了（因为单元测试通常用小扰动，此时 $J_L \approx J_R$），只在真实数据上运行时才暴露。

### 理论：两种约定的数学关系

**右扰动下的 Jacobian**

设 $f: G \to G$ 是一个李群上的函数。右扰动下的 Jacobian 定义为：

$$J_R = \lim_{\delta \to 0} \frac{\log(f(T)^{-1} \cdot f(T \cdot \exp(\delta)))}{\delta}$$

这个定义的含义是：在 T 处施加一个 body 坐标系中的微小扰动 $\delta$，观察输出在 $f(T)$ 处的切空间中的变化。

**左扰动下的 Jacobian**

$$J_L = \lim_{\delta \to 0} \frac{\log(f(\exp(\delta) \cdot T) \cdot f(T)^{-1})}{\delta}$$

这里扰动在 world 坐标系中施加，输出的变化也在 $f(T)$ 的 world 端测量。

**转换关系**

两种 Jacobian 通过伴随矩阵（Adjoint）关联：

$$J_L = \text{Ad}_{f(T)} \cdot J_R$$

或者等价地：

$$J_R = \text{Ad}_{f(T)}^{-1} \cdot J_L = \text{Ad}_{f(T)^{-1}} \cdot J_L$$

这个关系的直觉理解：左扰动和右扰动的区别在于扰动的参考坐标系不同。伴随映射正是做坐标系变换的工具——它把 body 坐标系中的切向量转换到 world 坐标系中。所以 $J_L = \text{Ad} \cdot J_R$ 完全自然。

**具体例子：位姿逆的 Jacobian**

考虑 $f(T) = T^{-1}$，这是 SLAM 中最常用的操作之一（计算 $T_{cw}$ 和 $T_{wc}$ 之间的转换）。

右扰动推导：$T \oplus \delta = T \cdot \exp(\delta)$，则 $(T \oplus \delta)^{-1} = \exp(-\delta) \cdot T^{-1}$。

输出变化在 $f(T) = T^{-1}$ 的右侧测量：

$$\log((T^{-1})^{-1} \cdot (\exp(-\delta) \cdot T^{-1})) = \log(T \cdot \exp(-\delta) \cdot T^{-1})$$

利用伴随的定义 $T \cdot \exp(\xi) \cdot T^{-1} = \exp(\text{Ad}_T \cdot \xi)$：

$$= \log(\exp(-\text{Ad}_T \delta)) = -\text{Ad}_T \delta$$

所以 $J_R^{\text{inv}} = -\text{Ad}_T$。

左扰动推导：$T \oplus \delta = \exp(\delta) \cdot T$，则 $(\exp(\delta) \cdot T)^{-1} = T^{-1} \cdot \exp(-\delta)$。

输出变化在 $f(T) = T^{-1}$ 的左侧测量：

$$\log(T^{-1} \cdot \exp(-\delta) \cdot (T^{-1})^{-1}) = \log(\exp(-\text{Ad}_{T^{-1}} \delta))$$

等一下，这样推导不对。左扰动 Jacobian 的定义是 $\log(f(T \oplus \delta) \cdot f(T)^{-1})$：

$$\log(T^{-1} \cdot \exp(-\delta) \cdot T) = \log(\exp(-\text{Ad}_{T^{-1}} \delta)) = -\text{Ad}_{T^{-1}} \delta$$

所以 $J_L^{\text{inv}} = -\text{Ad}_{T^{-1}}$。

可以验证转换关系成立：$J_L = \text{Ad}_{f(T)} \cdot J_R = \text{Ad}_{T^{-1}} \cdot (-\text{Ad}_T) = -\text{Ad}_{T^{-1}} \cdot \text{Ad}_T = -I_{6\times6}$。

等等——我们刚才算出 $J_L^{\text{inv}} = -\text{Ad}_{T^{-1}}$，但转换公式给出 $J_L = -I$。哪个对？让我们重新仔细检查。关键在于左扰动 Jacobian 的输出变化的测量方式。如果输出变化在 $f(T)$ 的左侧测量，即 $\log(f(T \oplus_L \delta) \cdot f(T)^{-1})$：

$$f(\exp(\delta) \cdot T) \cdot f(T)^{-1} = T^{-1} \exp(-\delta) \cdot T = \exp(-\text{Ad}_{T^{-1}} \delta)$$

所以 $J_L = -\text{Ad}_{T^{-1}}$。

而用转换公式：$J_L = \text{Ad}_{T^{-1}} \cdot J_R = \text{Ad}_{T^{-1}} \cdot (-\text{Ad}_T) = -I$。这两个结果矛盾了！

原因是：转换公式 $J_L = \text{Ad}_{f(T)} \cdot J_R$ 要求左右两边的"输出变化测量方式"一致（都在右侧测量，或都在左侧测量）。上面的推导中，$J_R$ 的输出在**右侧**测量，而 $J_L$ 的输出在**左侧**测量，所以不能直接用转换公式。

这个微妙之处正是左右扰动最容易出错的地方。在实际编程中，请始终明确你的 Jacobian 用的是什么约定——不仅输入端（左/右扰动），还有输出端（变化在哪里测量）。

| 操作 | manif 给出的 Jacobian（右扰动，输出右测量） |
|------|-------------------------------------------|
| $f(T) = T_a \cdot T$ | $J = I_{6\times6}$ |
| $f(T) = T \cdot T_b$ | $J = \text{Ad}_{T_b^{-1}}$ |
| $f(T) = T^{-1}$ | $J = -\text{Ad}_{T}$ |
| $f(T) = T_a^{-1} \cdot T$ | $J = I_{6\times6}$ |

### 实际工程中的约定选择

**manif**：使用**右扰动**。所有 Jacobian 方法（`J_compose_a`, `J_compose_b`, `J_act_T` 等）返回的都是右扰动 Jacobian。

**GTSAM**：默认使用**右扰动**。GTSAM 的 `retract(v)` 实现为 `this * Exp(v)`，即右乘。`BetweenFactor` 计算误差 $e = \log(T_{\text{meas}}^{-1} \cdot T_i^{-1} \cdot T_j)$，其 Jacobian 在右扰动约定下计算。因此 **manif 和 GTSAM 的扰动约定实际上是一致的**（都是右扰动），在大多数情况下可以直接使用 manif 的 Jacobian。

**Ceres**：Ceres 本身不规定扰动约定——它通过 `Manifold` 接口让用户自己定义 `Plus` 操作（即 $\oplus$）。manif 提供的 `CeresLocalParameterizationFunctor` 使用右扰动。

**manif 的 Jacobian 与 GTSAM 的兼容性**：

由于 manif 和 GTSAM 都使用右扰动约定，大多数情况下 manif 的 Jacobian 可以直接用于 GTSAM 的自定义因子。但需要注意 GTSAM 的 `evaluateError` 中 `H` 矩阵的具体定义——它是对 `localCoordinates`（即 `Log(T^{-1} * T')` ）的导数。建议用 GTSAM 的 `traits<T>::Retract` 做有限差分验证：

```cpp
// GTSAM 自定义因子中验证 Jacobian
// 用数值差分验证解析 Jacobian 是否正确
boost::function<gtsam::Vector(const gtsam::Pose3&)> f =
    [&](const gtsam::Pose3& p) { return evaluateError(p); };
gtsam::Matrix J_numerical = gtsam::numericalDerivative11(f, pose);
```

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：在优化框架的自定义因子中直接使用 manif 的 Jacobian 而不验证**

- **错误做法**：在 GTSAM 或其他优化框架的 `evaluateError` 中调用 manif 的群操作得到 Jacobian，不经验证直接填入 `H` 矩阵
- **现象**：优化不收敛或收敛到错误解
- **根本原因**：虽然 manif 和 GTSAM 都使用右扰动，但 GTSAM 的 `H` 矩阵要求的导数定义是相对于 `localCoordinates` 的，可能与 manif 的 Jacobian 定义有微妙的差异（如误差定义方向、切空间排列顺序）。不验证就用会引入 bug
- **正确做法**：始终用数值有限差分（如 `gtsam::numericalDerivative`）对解析 Jacobian 做验证。如果差异超过 1e-5，检查误差函数的定义方向和 Jacobian 的链式法则

**💡 概念误区：认为"左右扰动只影响 Jacobian 的符号"**

- **新手想法**："左扰动和右扰动的 Jacobian 可能就差一个负号或者转置"
- **实际上**：差异是一个完整的伴随矩阵 $\text{Ad}_T$（$6\times6$ 矩阵），包含旋转和耦合项。只有在 SO(3) 且绕单一轴旋转时，伴随矩阵才退化为简单的旋转矩阵。在 SE(3) 中，伴随矩阵包含 $[t]_\times R$ 耦合项，左右扰动 Jacobian 的差异远不是一个符号那么简单
- **自检方法**：对一个具体的 SE3d 位姿，同时计算 compose 的左右扰动 Jacobian，打印出来比较。你会发现它们是完全不同的 $6\times6$ 矩阵

**🧠 思维陷阱：混淆"左/右 Jacobian"（BCH）和"左/右扰动 Jacobian"**

- **新手想法**："右 Jacobian 就是右扰动下的 Jacobian"
- **实际上**：这是两个完全不同的概念。"左/右 Jacobian"（$J_l, J_r$）来自 BCH 公式，是 $\exp(\phi + \delta) \approx \exp(\phi) \cdot \exp(J_r^{-1} \delta)$ 中的那个 $J_r$。"左/右扰动 Jacobian"是对群操作 $f(T)$ 的求导约定。它们之间有联系（某些群操作的扰动 Jacobian 可以用 $J_l$ 或 $J_r$ 表达），但概念上不是一回事
- **正确思维**：遇到"Jacobian"时，先问两个问题：(1) 这是哪个操作的 Jacobian？(2) 用的是左扰动还是右扰动？

**⚠️ 编程陷阱：数值验证 Jacobian 时不区分扰动约定**

- **错误做法**：用 $f(T \cdot \exp(\delta))$ 做有限差分，但比较的对象是左扰动 Jacobian
- **现象**：数值 Jacobian 和解析 Jacobian 不匹配，误以为解析 Jacobian 有 bug
- **根本原因**：有限差分的方式决定了你计算的是哪种 Jacobian。$f(T \cdot \exp(\delta))$ 对应右扰动，$f(\exp(\delta) \cdot T)$ 对应左扰动。两边必须用同一种约定
- **正确做法**：验证 manif 的 Jacobian 时，有限差分必须用 $T \cdot \exp(h \cdot e_j)$（右扰动），然后比较输出变化 $\log(f(T)^{-1} \cdot f(T \cdot \exp(h \cdot e_j)))$ 除以 $h$

### 练习

**练习 23.8.1**（⭐⭐⭐）：对 SE3d 的 inverse 操作，分别计算右扰动和左扰动下的数值 Jacobian（有限差分），并与理论值比较。理论值：右扰动下 $J_R^{\text{inv}} = -\text{Ad}_T$。提示：右扰动有限差分为 $\log((T\exp(h \cdot e_j))^{-1} \cdot T^{-1 \, -1}) / h$，注意输出变化的测量方向要和理论值一致。要求数值和理论的最大绝对误差小于 $10^{-5}$。

**练习 23.8.2**（⭐⭐⭐）：写一个函数，将 manif 给出的 compose 操作的右扰动 Jacobian 转换为左扰动 Jacobian（通过 Adjoint），并用数值差分验证转换后的结果。

---

## 23.9 流形上的 Jacobian 详解 ⭐⭐⭐⭐

### 这一节解决什么问题

前面我们已经知道了 $\oplus$、$\ominus$、伴随矩阵的定义。但在 SLAM 优化中，仅仅会做这些运算还不够——优化器（Gauss-Newton、Levenberg-Marquardt）的核心需要的是 **Jacobian 矩阵**。当残差函数 $f(\mathcal{X})$ 的输入或输出"生活"在流形上时，普通的偏导数 $\partial f / \partial x$ 失去意义：你不能对流形元素做"加 $\epsilon$"的扰动（因为 $\mathcal{X} + \epsilon$ 不在流形上）。我们需要一套全新的 Jacobian 定义——基于 $\oplus/\ominus$ 的"流形导数"。

本节基于 Sola et al. (arXiv:1812.01537) 第 III 节，系统推导 SLAM 中最常用的 Jacobian 模块。掌握这些模块后，你可以像搭积木一样组合出**任意 SLAM 因子**的 Jacobian，而不需要每次都从头推导。

### 动机：为什么普通导数在流形上失效

考虑一个最简单的残差函数：给定观测到的相对位姿 $\Delta T_{\text{obs}}$，预测值为 $\Delta T_{\text{pred}} = T_1^{-1} \circ T_2$，残差为：

$$r = \Delta T_{\text{pred}} \ominus \Delta T_{\text{obs}} = \text{Log}(\Delta T_{\text{obs}}^{-1} \circ T_1^{-1} \circ T_2)$$

优化器需要 $\frac{\partial r}{\partial T_1}$ 和 $\frac{\partial r}{\partial T_2}$。但 $T_1, T_2 \in SE(3)$ 不是向量，$\partial T_1$ 没有意义。

**如果不理解流形 Jacobian 会怎样**：你可能会把 SE(3) 参数化成 6 维向量（比如平移 + 欧拉角），然后对这 6 个参数求偏导。这在数学上是可以做的，但有两个致命问题：(1) 欧拉角有万向锁奇点，在奇点附近 Jacobian 矩阵变得病态（条件数爆炸）；(2) 不同参数化的 Jacobian 不同，代码无法在不同库之间迁移。流形 Jacobian 避免了这些问题：它直接在切空间中定义，与具体参数化无关。

### 理论：右 Jacobian 的定义

**右 Jacobian**（right Jacobian，使用右扰动约定）定义为：

$$\frac{^{\mathcal{X}}Df(\mathcal{X})}{D\mathcal{X}} \triangleq \lim_{\tau \to 0} \frac{f(\mathcal{X} \oplus \tau) \ominus f(\mathcal{X})}{\tau}$$

这个定义的每一部分都有明确的含义，我们逐一拆解：

| 符号 | 含义 | 为什么这样设计 |
|------|------|--------------|
| $\mathcal{X} \oplus \tau$ | 对输入施加右扰动 | 用 $\oplus$ 代替 $+$，保证扰动后仍在流形上 |
| $f(\mathcal{X} \oplus \tau)$ | 扰动后的函数值 | 函数值可能在流形上，也可能在向量空间中 |
| $\ominus f(\mathcal{X})$ | 测量输出的变化量 | 用 $\ominus$ 代替 $-$，把变化量映射到切空间 |
| $/ \tau$ | 除以扰动量 | $\tau \in \mathbb{R}^m$ 是向量，除法有意义 |
| 左上标 ${}^{\mathcal{X}}$ | 标明扰动在 $\mathcal{X}$ 的局部坐标系 | 右扰动的标志 |

**如果函数的输出在向量空间 $\mathbb{R}^n$ 中**（比如残差向量），则 $\ominus$ 退化为普通减法 $-$：

$$\frac{^{\mathcal{X}}Df(\mathcal{X})}{D\mathcal{X}} = \lim_{\tau \to 0} \frac{f(\mathcal{X} \oplus \tau) - f(\mathcal{X})}{\tau}$$

**左 Jacobian** 的定义类似，但扰动从左边施加：

$$\frac{^{\varepsilon}Df(\mathcal{X})}{D\mathcal{X}} \triangleq \lim_{\tau \to 0} \frac{f(\tau \oplus \mathcal{X}) \ominus f(\mathcal{X})}{\tau}$$

两者的关系通过伴随矩阵联系：

$$\frac{^{\varepsilon}Df}{D\mathcal{X}} = \frac{^{\mathcal{X}}Df}{D\mathcal{X}} \cdot \text{Ad}_{\mathcal{X}}$$

这意味着：一旦你算出右 Jacobian，左 Jacobian 可以通过一次矩阵乘法得到。这也是为什么 manif 默认只计算右 Jacobian——另一个方向的总是可以转换得到。

### 基础 Jacobian 模块：SLAM 的积木

以下 5 类 Jacobian 是所有 SLAM 因子的基本构建块。理解它们后，任何复杂的残差函数都可以通过链式法则分解为这些基本模块的组合。

#### 模块 1：逆（Inverse）

**问题**：函数 $f(\mathcal{X}) = \mathcal{X}^{-1}$，求其右 Jacobian。

**推导**：

$$\frac{^{\mathcal{X}}D(\mathcal{X}^{-1})}{D\mathcal{X}} = \lim_{\tau \to 0} \frac{(\mathcal{X} \oplus \tau)^{-1} \ominus \mathcal{X}^{-1}}{\tau}$$

展开 $\oplus$ 的定义（$\mathcal{X} \oplus \tau = \mathcal{X} \circ \text{Exp}(\tau)$）：

$$= \lim_{\tau \to 0} \frac{\text{Log}\left(\mathcal{X}^{-1} \circ (\mathcal{X} \circ \text{Exp}(\tau))^{-1 \, -1}\right)}{\tau}$$

等等，这里需要仔细推导。令 $\mathcal{Y} = \mathcal{X}^{-1}$，则 $f(\mathcal{X} \oplus \tau) = (\mathcal{X} \circ \text{Exp}(\tau))^{-1} = \text{Exp}(\tau)^{-1} \circ \mathcal{X}^{-1} = \text{Exp}(-\tau) \circ \mathcal{Y}$。

因此：

$$f(\mathcal{X} \oplus \tau) \ominus f(\mathcal{X}) = \text{Log}\left(f(\mathcal{X})^{-1} \circ f(\mathcal{X} \oplus \tau)\right) = \text{Log}\left(\mathcal{Y}^{-1} \circ \text{Exp}(-\tau) \circ \mathcal{Y}\right)$$

利用伴随表达：$\mathcal{Y}^{-1} \circ \text{Exp}(-\tau) \circ \mathcal{Y} = \text{Exp}(-\text{Ad}_{\mathcal{Y}^{-1}} \cdot \tau)$（这正是伴随矩阵 23.6 节中推导的核心性质）。

因此：

$$f(\mathcal{X} \oplus \tau) \ominus f(\mathcal{X}) = -\text{Ad}_{\mathcal{Y}^{-1}} \cdot \tau = -\text{Ad}_{\mathcal{X}} \cdot \tau$$

（最后一步用到 $\mathcal{Y}^{-1} = (\mathcal{X}^{-1})^{-1} = \mathcal{X}$。）

$$\boxed{\mathbf{J}_{\mathcal{X}}^{\mathcal{X}^{-1}} = -\text{Ad}_{\mathcal{X}}}$$

**具体化**：

| 群 | $\text{Ad}_{\mathcal{X}}$ | Inverse Jacobian |
|---|--------------------------|-----------------|
| SO(3) | $R$ | $-R$ |
| SE(3) | $\begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix}$ | $-\begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix}$ |

**为什么这个结果是 $-\text{Ad}$ 而不是简单的 $-\mathbf{I}$？** 直觉上，取逆确实是"反方向"，所以有一个负号。但流形上的"反方向"取决于你在哪个坐标系中测量——伴随矩阵正是做了这个坐标系转换。

```cpp
// manif 代码：计算 inverse 的 Jacobian
#include <manif/SE3.h>

manif::SE3d T = manif::SE3d::Random();
manif::SE3d::Jacobian J_inv;  // 6×6 矩阵

// inverse() 可以同时返回 Jacobian
manif::SE3d T_inv = T.inverse(J_inv);

// 验证：J_inv 应该等于 -Ad(T)
Eigen::Matrix<double, 6, 6> neg_Ad = -T.adj();  // adj() 返回 Ad 矩阵
std::cout << "误差: " << (J_inv - neg_Ad).norm() << std::endl;
// 输出应为 ~1e-15（机器精度）
```

#### 模块 2：组合（Composition）

**问题**：函数 $f(\mathcal{X}, \mathcal{Y}) = \mathcal{X} \circ \mathcal{Y}$，分别对 $\mathcal{X}$ 和 $\mathcal{Y}$ 求右 Jacobian。

**对 $\mathcal{X}$ 的 Jacobian**：

$$\frac{^{\mathcal{X}}D(\mathcal{X} \circ \mathcal{Y})}{D\mathcal{X}} = \lim_{\tau \to 0} \frac{((\mathcal{X} \circ \text{Exp}(\tau)) \circ \mathcal{Y}) \ominus (\mathcal{X} \circ \mathcal{Y})}{\tau}$$

利用结合律重新分组：

$$= \lim_{\tau \to 0} \frac{(\mathcal{X} \circ \text{Exp}(\tau) \circ \mathcal{Y}) \ominus (\mathcal{X} \circ \mathcal{Y})}{\tau}$$

关键步骤：把 $\text{Exp}(\tau)$ 从中间"搬"到最右边。利用伴随性质 $\text{Exp}(\tau) \circ \mathcal{Y} = \mathcal{Y} \circ \text{Exp}(\text{Ad}_{\mathcal{Y}^{-1}} \tau)$：

$$= \lim_{\tau \to 0} \frac{(\mathcal{X} \circ \mathcal{Y} \circ \text{Exp}(\text{Ad}_{\mathcal{Y}^{-1}} \tau)) \ominus (\mathcal{X} \circ \mathcal{Y})}{\tau} = \text{Ad}_{\mathcal{Y}^{-1}}$$

$$\boxed{\mathbf{J}_{\mathcal{X}}^{\mathcal{X} \circ \mathcal{Y}} = \text{Ad}_{\mathcal{Y}^{-1}}}$$

**对 $\mathcal{Y}$ 的 Jacobian**（更简单）：

$$\frac{^{\mathcal{Y}}D(\mathcal{X} \circ \mathcal{Y})}{D\mathcal{Y}} = \lim_{\tau \to 0} \frac{(\mathcal{X} \circ \mathcal{Y} \circ \text{Exp}(\tau)) \ominus (\mathcal{X} \circ \mathcal{Y})}{\tau} = \lim_{\tau \to 0} \frac{\tau}{\tau} = \mathbf{I}$$

$$\boxed{\mathbf{J}_{\mathcal{Y}}^{\mathcal{X} \circ \mathcal{Y}} = \mathbf{I}}$$

**直觉**：为什么对 $\mathcal{Y}$ 的 Jacobian 是单位矩阵？因为 $\mathcal{Y}$ 的右扰动直接"传递"到结果 $\mathcal{X} \circ \mathcal{Y}$ 的右侧——扰动没有被"搬运"过，所以不需要坐标变换。而对 $\mathcal{X}$ 的扰动需要"穿过" $\mathcal{Y}$，伴随矩阵 $\text{Ad}_{\mathcal{Y}^{-1}}$ 正是完成这个"搬运"的工具。

```cpp
// manif 代码：计算 compose 的 Jacobian
manif::SE3d T1 = manif::SE3d::Random();
manif::SE3d T2 = manif::SE3d::Random();
manif::SE3d::Jacobian J_T1, J_T2;

manif::SE3d T12 = T1.compose(T2, J_T1, J_T2);

// 验证
Eigen::Matrix<double, 6, 6> Ad_T2_inv = T2.inverse().adj();
std::cout << "J_T1 误差: " << (J_T1 - Ad_T2_inv).norm() << std::endl;
std::cout << "J_T2 误差: " << (J_T2 - Eigen::Matrix<double, 6, 6>::Identity()).norm() << std::endl;
```

#### 模块 3：指数映射的右 Jacobian $\mathbf{J}_r(\tau)$

**问题**：函数 $f(\tau) = \text{Exp}(\tau)$，$\tau \in \mathbb{R}^m$（切空间），求 $\frac{Df}{D\tau}$。

这个 Jacobian 通常记为 $\mathbf{J}_r(\tau)$（right Jacobian of the exponential map），它描述了指数映射对切向量参数的敏感度。

**定义**：

$$\text{Exp}(\tau + \delta\tau) = \text{Exp}(\tau) \circ \text{Exp}(\mathbf{J}_r(\tau) \cdot \delta\tau) + O(\|\delta\tau\|^2)$$

或者等价地：

$$\mathbf{J}_r(\tau) = \lim_{\delta\tau \to 0} \frac{\text{Exp}(\tau + \delta\tau) \ominus \text{Exp}(\tau)}{\delta\tau}$$

**SO(3) 的闭式公式**：

对 $\tau = \theta \hat{u}$（$\theta = \|\tau\|$ 是角度，$\hat{u}$ 是旋转轴的单位向量，$[\tau]_\times$ 是对应的反对称矩阵），SO(3) 的右 Jacobian 有优美的闭式解：

$$\mathbf{J}_r(\tau) = \mathbf{I} - \frac{1 - \cos\theta}{\theta^2}[\tau]_\times + \frac{\theta - \sin\theta}{\theta^3}[\tau]_\times^2$$

推导思路：将 BCH 公式展开到一阶，对比两侧得到 $\mathbf{J}_r$。完整推导需要矩阵指数的级数展开，此处给出关键步骤：

**Step 1**：BCH 公式的一阶截断告诉我们 $\text{Log}(\text{Exp}(\tau) \circ \text{Exp}(\delta\tau)) \approx \tau + \mathbf{J}_r^{-1}(\tau) \delta\tau$。

**Step 2**：将 $\mathbf{J}_r^{-1}$ 的级数展开为 $\mathbf{J}_r^{-1} = \sum_{k=0}^{\infty} \frac{B_k}{k!} [\tau]_\times^k$（$B_k$ 是 Bernoulli 数），取前三项：

$$\mathbf{J}_r^{-1}(\tau) = \mathbf{I} + \frac{1}{2}[\tau]_\times + \left(\frac{1}{\theta^2} - \frac{1+\cos\theta}{2\theta\sin\theta}\right)[\tau]_\times^2$$

**Step 3**：对 $\mathbf{J}_r^{-1}$ 求逆，利用 $[\tau]_\times^3 = -\theta^2 [\tau]_\times$ 的循环性质简化，得到上述闭式公式。

**数值注意**：当 $\theta \to 0$ 时，公式中的 $\frac{1-\cos\theta}{\theta^2}$ 和 $\frac{\theta-\sin\theta}{\theta^3}$ 都趋近于有限值（分别为 $\frac{1}{2}$ 和 $\frac{1}{6}$），但直接计算会产生 $0/0$ 的数值不稳定。工程中需要对小角度用 Taylor 展开替代。manif 内部已做此处理。

**SE(3) 的右 Jacobian**：

SE(3) 的右 Jacobian 有 $6 \times 6$ 的分块结构：

$$\mathbf{J}_r^{SE(3)}(\tau) = \begin{bmatrix} \mathbf{J}_r^{SO(3)}(\theta) & \mathbf{Q}_r(\rho, \theta) \\ \mathbf{0}_{3\times3} & \mathbf{J}_r^{SO(3)}(\theta) \end{bmatrix}$$

其中 $\tau = [\rho^T, \theta^T]^T$（$\rho$ 是平移相关部分，$\theta$ 是旋转相关部分），$\mathbf{Q}_r$ 是一个耦合项，由 Barfoot (2017) 给出了完整的闭式公式。manif 的切空间向量排布是 $[\rho; \theta]$（平移在前，旋转在后），使用时需要注意与其他参考文献的排列顺序可能不同。

```cpp
// manif 代码：计算 Exp 的 Jacobian
Eigen::Vector3d omega(0.1, -0.2, 0.3);
manif::SO3d::Jacobian J_exp;

manif::SO3d R = manif::SO3d::Exp(omega, J_exp);

// J_exp 即 J_r(omega)，是 3×3 矩阵
std::cout << "J_r =\n" << J_exp << std::endl;

// 数值验证：有限差分
double h = 1e-8;
Eigen::Matrix3d J_num;
for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d omega_plus = omega;
    omega_plus(j) += h;
    manif::SO3d R_plus = manif::SO3d::Exp(omega_plus);
    J_num.col(j) = (R_plus.rminus(R)).coeffs() / h;  // rminus = right-⊖
}
std::cout << "数值 J_r =\n" << J_num << std::endl;
std::cout << "差异: " << (J_exp - J_num).norm() << std::endl;
```

#### 模块 4：左右 Jacobian 的关系

**左 Jacobian** $\mathbf{J}_l(\tau)$ 的定义是：

$$\text{Exp}(\tau + \delta\tau) = \text{Exp}(\mathbf{J}_l(\tau) \cdot \delta\tau) \circ \text{Exp}(\tau) + O(\|\delta\tau\|^2)$$

左右 Jacobian 之间有一个极其重要的关系：

$$\boxed{\mathbf{J}_l(\tau) = \mathbf{J}_r(-\tau)}$$

推导：从 $\text{Exp}(\tau + \delta\tau) = \text{Exp}(\tau) \circ \text{Exp}(\mathbf{J}_r(\tau)\delta\tau)$ 出发，两边取逆：

$$\text{Exp}(-\tau - \delta\tau) = \text{Exp}(-\mathbf{J}_r(\tau)\delta\tau) \circ \text{Exp}(-\tau)$$

令 $\tau' = -\tau$，$\delta\tau' = -\delta\tau$，得：

$$\text{Exp}(\tau' + \delta\tau') = \text{Exp}(\mathbf{J}_r(\tau) \delta\tau') \circ \text{Exp}(\tau')$$

对比左 Jacobian 的定义，$\mathbf{J}_l(\tau') = \mathbf{J}_r(\tau) = \mathbf{J}_r(-\tau')$，因此 $\mathbf{J}_l(\tau) = \mathbf{J}_r(-\tau)$。

**为什么这很重要？** 因为你可能阅读的论文使用左 Jacobian（例如 Barfoot 的教材中大量使用左扰动），而你的代码使用 manif（右扰动）。有了这个关系，你只需要翻转符号即可。一些基于 Barfoot 教材的实现使用左扰动，如果你要移植这样的因子到 manif 框架中，这个关系是关键。

还有一个常用的等价形式：

$$\mathbf{J}_l(\tau) = \mathbf{J}_r(\tau)^T \quad \text{（仅对 SO(3)，因为 $[\tau]_\times$ 的反对称性）}$$

注意这个转置关系仅在 SO(3) 中成立，SE(3) 中不成立。

#### 模块 5：群作用 Jacobian

**问题**：函数 $f(\mathcal{X}, v) = \mathcal{X} \cdot v$（群作用），分别对 $\mathcal{X}$ 和 $v$ 求 Jacobian。

**SO(3) 的群作用 Jacobian**：

对于 $f(R, v) = Rv$：

对 $R$ 的右 Jacobian（通过扰动 $R' = R \circ \text{Exp}(\tau) = R \cdot \text{Exp}(\tau)$）：

$$\frac{^{R}D(Rv)}{DR} = \lim_{\tau \to 0} \frac{R \cdot \text{Exp}(\tau) \cdot v - Rv}{\tau}$$

当 $\tau$ 很小时，$\text{Exp}(\tau) \approx I + [\tau]_\times$：

$$\approx \lim_{\tau \to 0} \frac{R(I + [\tau]_\times)v - Rv}{\tau} = \lim_{\tau \to 0} \frac{R[\tau]_\times v}{\tau}$$

利用反对称矩阵性质 $[\tau]_\times v = -[v]_\times \tau$：

$$= \lim_{\tau \to 0} \frac{-R[v]_\times \tau}{\tau} = -R[v]_\times$$

$$\boxed{\mathbf{J}_{R}^{Rv} = -R[v]_\times}$$

对 $v$ 的 Jacobian（$v$ 是普通向量，直接对向量分量求导）：

$$\boxed{\mathbf{J}_{v}^{Rv} = R}$$

**SE(3) 的群作用 Jacobian**：

对于 $f(T, p) = Rp + t$（其中 $T = (R, t)$），manif 的切空间排布为 $[\rho; \theta]$（平移在前）：

$$\boxed{\mathbf{J}_{T}^{T \cdot p} = \begin{bmatrix} R & -R[p]_\times \end{bmatrix}}$$

这个 $3 \times 6$ 的 Jacobian 在 ICP、点到平面匹配、特征点重投影等残差中无处不在。

```cpp
// manif 代码：计算 act 的 Jacobian
manif::SE3d T = manif::SE3d::Random();
Eigen::Vector3d p(1.0, 2.0, 3.0);

Eigen::Matrix<double, 3, 6> J_T;    // 3×6，act 输出 3D 点对 6D 位姿扰动的 Jacobian
Eigen::Matrix<double, 3, 3> J_p;   // 3×3，对 p 的 Jacobian

Eigen::Vector3d p_world = T.act(p, J_T, J_p);

// 验证
Eigen::Matrix3d R = T.rotation();
Eigen::Matrix<double, 3, 6> J_T_expected;
J_T_expected.block<3,3>(0,0) = R;
J_T_expected.block<3,3>(0,3) = -R * manif::skew(p);
// 注意：manif 的切空间排布可能与论文不同，需检查列的顺序
std::cout << "J_T 误差: " << (J_T - J_T_expected).norm() << std::endl;
std::cout << "J_p 误差: " << (J_p - R).norm() << std::endl;
```

### 流形上的链式法则

有了基本模块后，复杂残差函数的 Jacobian 可以通过**链式法则**组合得到：

$$\mathbf{J}_{\mathcal{X}}^{\mathcal{Z}} = \mathbf{J}_{\mathcal{Y}}^{\mathcal{Z}} \cdot \mathbf{J}_{\mathcal{X}}^{\mathcal{Y}}$$

这和标准微积分中的链式法则完全类似——唯一的区别是这里的 Jacobian 是流形 Jacobian，其定义使用了 $\oplus/\ominus$ 而不是 $+/-$。

**示例：位姿图的相对位姿残差**

给定两帧位姿 $T_i, T_j$ 和观测到的相对位姿 $\Delta T_{ij}^{\text{obs}}$，残差为：

$$r = \text{Log}(\Delta T_{ij}^{\text{obs} \, -1} \circ T_i^{-1} \circ T_j)$$

要求 $\frac{\partial r}{\partial T_i}$ 和 $\frac{\partial r}{\partial T_j}$。把这个函数分解为基本模块的组合：

1. $\mathcal{A} = T_i^{-1}$（Inverse） $\Rightarrow$ $\mathbf{J}_{T_i}^{\mathcal{A}} = -\text{Ad}_{T_i}$
2. $\mathcal{B} = \mathcal{A} \circ T_j$（Composition） $\Rightarrow$ $\mathbf{J}_{\mathcal{A}}^{\mathcal{B}} = \text{Ad}_{T_j^{-1}}$，$\mathbf{J}_{T_j}^{\mathcal{B}} = \mathbf{I}$
3. $\mathcal{C} = \Delta T^{\text{obs} \, -1} \circ \mathcal{B}$（Composition with constant） $\Rightarrow$ $\mathbf{J}_{\mathcal{B}}^{\mathcal{C}} = \mathbf{I}$
4. $r = \text{Log}(\mathcal{C})$（Log 映射） $\Rightarrow$ $\mathbf{J}_{\mathcal{C}}^{r} = \mathbf{J}_r^{-1}(\text{Log}(\mathcal{C}))$

链式法则组合：

$$\mathbf{J}_{T_j}^{r} = \mathbf{J}_{\mathcal{C}}^{r} \cdot \mathbf{J}_{\mathcal{B}}^{\mathcal{C}} \cdot \mathbf{J}_{T_j}^{\mathcal{B}} = \mathbf{J}_r^{-1}(r) \cdot \mathbf{I} \cdot \mathbf{I} = \mathbf{J}_r^{-1}(r)$$

$$\mathbf{J}_{T_i}^{r} = \mathbf{J}_{\mathcal{C}}^{r} \cdot \mathbf{J}_{\mathcal{B}}^{\mathcal{C}} \cdot \mathbf{J}_{\mathcal{A}}^{\mathcal{B}} \cdot \mathbf{J}_{T_i}^{\mathcal{A}} = \mathbf{J}_r^{-1}(r) \cdot \text{Ad}_{T_j^{-1}} \cdot (-\text{Ad}_{T_i})$$

当残差很小（$r \approx 0$）时，$\mathbf{J}_r^{-1}(r) \approx \mathbf{I}$，上式简化为 $-\text{Ad}_{T_j^{-1}} \cdot \text{Ad}_{T_i} = -\text{Ad}_{T_j^{-1} T_i}$。

### Plus/Minus Jacobian（派生结果）

利用 Composition + Exp 的 Jacobian，可以直接推导 $\oplus$ 和 $\ominus$ 的 Jacobian：

**$\oplus$ 的 Jacobian**（$\mathcal{Y} = \mathcal{X} \oplus \tau = \mathcal{X} \circ \text{Exp}(\tau)$）：

这是 Composition 的特例（$\mathcal{Y} = \mathcal{X} \circ \text{Exp}(\tau)$），用链式法则：

$$\mathbf{J}_{\mathcal{X}}^{\mathcal{X} \oplus \tau} = \text{Ad}_{\text{Exp}(\tau)^{-1}} = \text{Ad}_{\text{Exp}(-\tau)}$$

$$\mathbf{J}_{\tau}^{\mathcal{X} \oplus \tau} = \mathbf{J}_r(\tau)$$

**$\ominus$ 的 Jacobian**（$\tau = \mathcal{Y} \ominus \mathcal{X} = \text{Log}(\mathcal{X}^{-1} \circ \mathcal{Y})$）：

$$\mathbf{J}_{\mathcal{Y}}^{\mathcal{Y} \ominus \mathcal{X}} = \mathbf{J}_r^{-1}(\tau)$$

$$\mathbf{J}_{\mathcal{X}}^{\mathcal{Y} \ominus \mathcal{X}} = -\mathbf{J}_l^{-1}(\tau)$$

这些公式在 SLAM 优化中频繁出现——每当你定义一个残差为"两个位姿之间的差"时（$r = T_{\text{pred}} \ominus T_{\text{obs}}$），就需要这些 Jacobian。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：忘记 Jacobian 输出参数的初始化**
> 错误做法：声明 `manif::SE3d::Jacobian J;` 后不经过任何计算就使用 `J`。
> 现象：`J` 中是未初始化的垃圾值，但代码不会报错，优化器可能"看起来收敛"但结果完全错误。
> 根本原因：C++ 的栈变量不自动初始化。`manif::SE3d::Jacobian` 是 `Eigen::Matrix<double,6,6>` 的别名，不会零初始化。
> 正确做法：始终通过 `compose(T2, J1, J2)` 等函数填充 Jacobian 值，不要手动创建后直接使用。如果需要零初始化，写 `manif::SE3d::Jacobian J = manif::SE3d::Jacobian::Zero();`。

> 💡 **概念误区：认为对 $\mathcal{Y}$ 的 compose Jacobian 总是单位矩阵**
> 新手想法："manif 算出来 $J_{\mathcal{Y}} = I$，那所有 compose 的第二个参数的 Jacobian 都是 $I$。"
> 实际上：$J_{\mathcal{Y}} = I$ 仅对**右扰动**成立。如果你使用左扰动约定，$\mathbf{J}_{\mathcal{Y}}^{\mathcal{X}\circ\mathcal{Y}} = \text{Ad}_{\mathcal{X}}$。在 GTSAM 中混用 manif 的 Jacobian 时会出错。
> 正确做法：明确你的代码和论文使用哪种扰动约定，所有 Jacobian 必须在同一约定下计算。

> 🧠 **思维陷阱：跳过"小量近似"检查**
> 新手想法："理论公式推出来了，直接代入代码就行。"
> 实际上：理论 Jacobian 与数值有限差分 Jacobian 比较是**必做的验证步骤**。很多 bug 来自切空间排列顺序不同（$[\rho; \theta]$ vs $[\theta; \rho]$）、左右扰动混淆、或符号错误。
> 正确做法：对每个新写的 Jacobian，都用 $10^{-8}$ 量级的有限差分验证，最大绝对误差应 $< 10^{-5}$。

> ⚠️ **编程陷阱：manif 和论文的切空间排列顺序不同**
> 错误做法：按论文中 $\xi = [\theta^T, \rho^T]^T$（旋转在前）的顺序写 Jacobian。
> 现象：Jacobian 的行列被交换，优化器不收敛或收敛到错误解。
> 根本原因：manif 使用 $[\rho^T, \theta^T]^T$（平移在前），而 Sola 论文和 Barfoot 教材使用 $[\theta^T, \rho^T]^T$（旋转在前）。
> 正确做法：检查 `manif::SE3Tangentd::coeffs()` 的排列顺序，编写单元测试确认。

### 练习

**练习 23.9.1**（⭐⭐⭐）：对 SE3d 的群作用 $T \cdot p$，用有限差分验证 `T.act(p, J_T, J_p)` 返回的两个 Jacobian。有限差分方法：对 $T$ 的第 $j$ 个切方向施加 $h = 10^{-8}$ 的扰动，$J_{\text{num}}(:,j) = (T \oplus h \cdot e_j \cdot p - T \cdot p) / h$。要求误差 $< 10^{-5}$。

**练习 23.9.2**（⭐⭐⭐⭐）：实现一个完整的"位姿图二元因子" Jacobian：给定 $T_i, T_j$ 和观测 $\Delta T_{ij}$，写出残差 $r = \text{Log}(\Delta T_{ij}^{-1} \circ T_i^{-1} \circ T_j)$ 对 $T_i$ 和 $T_j$ 的 Jacobian。用 manif 的基本模块和链式法则实现，并用数值差分验证。

**练习 23.9.3**（⭐⭐⭐）：解释为什么 $\mathbf{J}_r(0) = \mathbf{I}$（即零切向量处的右 Jacobian 是单位矩阵）。从定义和闭式公式两个角度说明。这个性质的工程含义是什么？（提示：当残差很小时，Jacobian 可以用单位矩阵近似。）

---

## 23.10 不确定性与协方差传播 ⭐⭐⭐

### 这一节解决什么问题

在 SLAM 系统中，我们不仅需要估计位姿的"最优值"，还需要知道估计的**不确定性**有多大。比如：IMU 积分一段时间后，得到的位姿 $T$ 是一个"带噪声"的估计——这个噪声有多大？它在空间中的分布形状是什么（是各向同性的球形，还是某些方向上不确定性更大的椭球？）。

在向量空间中，不确定性用协方差矩阵 $\Sigma \in \mathbb{R}^{n \times n}$ 描述，高斯噪声 $x \sim \mathcal{N}(\bar{x}, \Sigma)$ 的含义很直接。但在流形上，$T + \text{noise}$ 没有意义——我们不能对 SE(3) 元素做加法。本节基于 Sola et al. 第 II-H 节，讲解如何在流形上定义和传播不确定性。

### 动机：EKF 中的协方差传播

EKF（扩展卡尔曼滤波）的核心循环是：

$$\bar{x}_{k+1} = f(\bar{x}_k, u_k) \qquad \Sigma_{k+1} = F_k \Sigma_k F_k^T + Q_k$$

其中 $F_k = \frac{\partial f}{\partial x}\big|_{\bar{x}_k}$ 是状态转移函数的 Jacobian，$Q_k$ 是过程噪声协方差。

当状态包含位姿（$x = [T, v, b_g, b_a]$，$T \in SE(3)$）时，$\frac{\partial f}{\partial x}$ 中涉及对 SE(3) 元素的"偏导"——这就需要流形 Jacobian。如果不正确处理，EKF 传播出的协方差 $\Sigma$ 会失去物理意义（比如出现负的特征值），导致滤波器发散。

### 如果不理解流形上的不确定性会怎样

常见错误：用 $4 \times 4$ 矩阵的 16 个元素的协方差来表示 SE(3) 的不确定性。这是错误的，因为：

1. SE(3) 只有 6 个自由度，16 元素的协方差矩阵有 16×16 = 256 个分量，大量冗余且相互矛盾
2. "矩阵 + 高斯噪声"的结果不再满足 $R^TR = I$ 约束——噪声把状态推到了流形外面
3. 协方差矩阵的特征值无法对应物理方向（"沿 x 轴平移的不确定性"无法直接读出）

### 理论：切空间中的扰动模型

正确的做法是在**切空间**中定义不确定性。设 $\bar{\mathcal{X}} \in \mathcal{M}$ 是流形上的均值（最优估计），真实值 $\mathcal{X}$ 与 $\bar{\mathcal{X}}$ 的偏差用切空间中的随机向量 $\tau$ 表示：

$$\mathcal{X} = \bar{\mathcal{X}} \oplus \tau, \quad \tau \sim \mathcal{N}(0, \Sigma_{\mathcal{X}})$$

其中 $\Sigma_{\mathcal{X}} \in \mathbb{R}^{m \times m}$ 是**切空间中的协方差矩阵**。这里 $m$ 是流形的维度（SO(3) 为 3，SE(3) 为 6）。

这个模型的直觉是：$\tau$ 是从均值 $\bar{\mathcal{X}}$ 出发，在切空间中的"小偏差"。通过 $\oplus$ 映射到流形上后，得到真实值 $\mathcal{X}$。当 $\tau$ 很小时（即不确定性不大），这个映射近似线性，高斯分布保持为高斯分布。

**协方差的正式定义**：

$$\Sigma_{\mathcal{X}} = \mathbb{E}[(\mathcal{X} \ominus \bar{\mathcal{X}})(\mathcal{X} \ominus \bar{\mathcal{X}})^T] = \mathbb{E}[\tau \tau^T]$$

注意这是一个 $m \times m$ 矩阵，是正定的（假设不确定性在所有方向上都非零）。

### 局部协方差 vs 全局协方差

使用右扰动（$\mathcal{X} = \bar{\mathcal{X}} \oplus {}^{\mathcal{X}}\tau$）时，$\tau$ 在 $\bar{\mathcal{X}}$ 的局部坐标系中。使用左扰动（$\mathcal{X} = {}^{\varepsilon}\tau \oplus \bar{\mathcal{X}}$）时，$\tau$ 在全局坐标系中。两者的协方差通过伴随矩阵转换：

$${}^{\varepsilon}\Sigma = \text{Ad}_{\bar{\mathcal{X}}} \cdot {}^{\mathcal{X}}\Sigma \cdot \text{Ad}_{\bar{\mathcal{X}}}^T$$

**直觉**：这和经典的"坐标变换下协方差的变换规则"完全类似——$\Sigma' = A \Sigma A^T$，只不过这里的"坐标变换"是伴随矩阵。

**为什么区分很重要？** 在 FAST-LIO 等 ESKF 系统中，IMU 噪声在 body 系中建模（右扰动），但你可能需要在世界系中可视化不确定性椭球。如果不做伴随变换就直接画椭球，椭球的朝向和大小都是错的。

### 协方差通过函数传播

设 $\mathcal{Y} = f(\mathcal{X})$，$\mathcal{X}$ 的不确定性为 $\Sigma_{\mathcal{X}}$，要计算 $\mathcal{Y}$ 的不确定性 $\Sigma_{\mathcal{Y}}$。

一阶近似（线性化）：

$$\mathcal{Y} = f(\bar{\mathcal{X}} \oplus \tau) \approx f(\bar{\mathcal{X}}) \oplus \mathbf{J} \tau$$

其中 $\mathbf{J} = \frac{^{\mathcal{X}}Df}{D\mathcal{X}}\big|_{\bar{\mathcal{X}}}$ 是流形 Jacobian（23.9 节定义的）。

因此：

$$\mathcal{Y} \ominus \bar{\mathcal{Y}} \approx \mathbf{J} \tau$$

$$\boxed{\Sigma_{\mathcal{Y}} \approx \mathbf{J} \Sigma_{\mathcal{X}} \mathbf{J}^T}$$

这和经典的"非线性函数的协方差传播" $\Sigma_y = F \Sigma_x F^T$ 形式完全一致——唯一的区别是 $\mathbf{J}$ 是流形 Jacobian 而不是普通 Jacobian。

**多变量的推广**：如果 $\mathcal{Z} = f(\mathcal{X}, \mathcal{Y})$，且 $\mathcal{X}, \mathcal{Y}$ 独立，则：

$$\Sigma_{\mathcal{Z}} \approx \mathbf{J}_{\mathcal{X}} \Sigma_{\mathcal{X}} \mathbf{J}_{\mathcal{X}}^T + \mathbf{J}_{\mathcal{Y}} \Sigma_{\mathcal{Y}} \mathbf{J}_{\mathcal{Y}}^T$$

### 与 EKF 协方差更新的联系

现在可以回到 EKF 的协方差传播公式。假设状态包含位姿 $T \in SE(3)$ 和速度 $v \in \mathbb{R}^3$，状态转移函数为：

$$T_{k+1} = T_k \circ \text{Exp}(\omega_k \delta t), \quad v_{k+1} = v_k + R_k (a_k - b_a) \delta t + g \delta t$$

则 EKF 的状态转移 Jacobian 的"位姿-位姿"块为：

$$F_{TT} = \frac{^{T_k}D(T_{k+1})}{DT_k} = \text{Ad}_{\text{Exp}(\omega_k \delta t)^{-1}}$$

这正是 23.9 节中 Composition Jacobian 的结果！EKF 的协方差传播 $\Sigma_{k+1} = F \Sigma_k F^T + Q$ 正是流形上协方差传播的特例。

### manif 代码：不确定性传播示例

```cpp
#include <manif/SE3.h>
#include <Eigen/Dense>

// 设定初始位姿及其协方差
manif::SE3d T_mean = manif::SE3d::Identity();

// 切空间中的 6×6 协方差：
// 平移方向（前3维）不确定性 0.01m，旋转方向（后3维）不确定性 0.005rad
Eigen::Matrix<double, 6, 6> Sigma_T = Eigen::Matrix<double, 6, 6>::Zero();
Sigma_T.block<3,3>(0,0) = 0.01 * 0.01 * Eigen::Matrix3d::Identity();  // 平移
Sigma_T.block<3,3>(3,3) = 0.005 * 0.005 * Eigen::Matrix3d::Identity(); // 旋转

// 施加一个变换：T_new = T_mean ∘ T_delta
manif::SE3d T_delta = manif::SE3d(0.1, 0, 0, 0, 0, 0.1);  // 前进 0.1m，绕 z 轴转 0.1rad
manif::SE3d::Jacobian J_mean, J_delta;

manif::SE3d T_new = T_mean.compose(T_delta, J_mean, J_delta);

// 假设 T_delta 也有不确定性（来自 IMU 噪声）
Eigen::Matrix<double, 6, 6> Sigma_delta = Eigen::Matrix<double, 6, 6>::Zero();
Sigma_delta.block<3,3>(0,0) = 0.001 * 0.001 * Eigen::Matrix3d::Identity();
Sigma_delta.block<3,3>(3,3) = 0.0005 * 0.0005 * Eigen::Matrix3d::Identity();

// 传播协方差
Eigen::Matrix<double, 6, 6> Sigma_new = J_mean * Sigma_T * J_mean.transpose()
                                       + J_delta * Sigma_delta * J_delta.transpose();

std::cout << "传播后的协方差对角线:\n" << Sigma_new.diagonal().transpose() << std::endl;
// 可以观察到不确定性随着变换的累积而增大
```

### 局部 vs 全局协方差转换代码

```cpp
// 将右扰动（body 系）协方差转换为左扰动（world 系）协方差
Eigen::Matrix<double, 6, 6> Ad = T_mean.adj();  // 6×6 伴随矩阵
Eigen::Matrix<double, 6, 6> Sigma_global = Ad * Sigma_T * Ad.transpose();

// 在世界坐标系中，不确定性椭球的朝向与 body 系不同
// 如果 T_mean 有较大的旋转，Sigma_global 的对角线分布会与 Sigma_T 不同
std::cout << "Body 系协方差对角线: " << Sigma_T.diagonal().transpose() << std::endl;
std::cout << "World 系协方差对角线: " << Sigma_global.diagonal().transpose() << std::endl;
```

### ⚠️ 常见陷阱

> 💡 **概念误区：认为流形上的"高斯分布"是真正的高斯分布**
> 新手想法："$\tau \sim \mathcal{N}(0, \Sigma)$ 且 $\mathcal{X} = \bar{\mathcal{X}} \oplus \tau$，所以 $\mathcal{X}$ 服从高斯分布。"
> 实际上：$\tau$ 在切空间（$\mathbb{R}^m$）中是高斯分布，但通过指数映射到流形后，$\mathcal{X}$ 的分布**不再是高斯的**——它在流形上是"集中在均值附近的近似高斯"。当不确定性很大时（如旋转不确定性超过 30 度），高斯近似失效，需要使用更复杂的分布模型（如 von Mises-Fisher 分布）。
> 正确做法：在典型 SLAM 场景中，帧间旋转不确定性通常 < 1 度，高斯近似非常好。但在初始化阶段或退化场景中要警惕。

> ⚠️ **编程陷阱：协方差矩阵没有保持对称正定**
> 错误做法：多次传播后直接使用 $\Sigma_{\text{new}} = J \Sigma J^T$，不做对称化处理。
> 现象：经过多次迭代后，由于浮点误差累积，$\Sigma$ 可能变得不严格对称，导致 Cholesky 分解失败。
> 根本原因：理论上 $J \Sigma J^T$ 是对称的，但浮点运算的舍入误差会破坏严格对称性。
> 正确做法：每次传播后强制对称化：`Sigma = 0.5 * (Sigma + Sigma.transpose());`

### 练习

**练习 23.10.1**（⭐⭐⭐）：对一个 SE(3) 位姿施加 100 次随机小扰动（每次扰动从 $\mathcal{N}(0, \Sigma_{\delta})$ 采样），模拟 IMU 积分的不确定性累积。分别用蒙特卡洛方法（采样 1000 次轨迹后统计协方差）和解析传播公式计算最终的协方差，比较两者的差异。

**练习 23.10.2**（⭐⭐⭐）：验证局部/全局协方差转换：构造一个有明显旋转的位姿 $T$（如绕 z 轴转 45 度），设定 body 系中的协方差为对角阵（$\sigma_x^2 = 1, \sigma_y^2 = 4, \sigma_z^2 = 1$），转换到全局坐标系后，验证协方差椭球的主轴方向确实随 $T$ 的旋转而旋转。

---

## 23.11 离散积分与 IMU 预积分基础 ⭐⭐⭐

### 这一节解决什么问题

在 SLAM 系统中，IMU 以 100-1000 Hz 的频率输出角速度 $\omega$ 和线加速度 $a$。将这些测量值积分（integration）可以得到位姿的变化。但"积分"这个操作对旋转来说并不平凡——天真的欧拉积分 $R_{k+1} = R_k + \dot{R} \delta t$ 会让结果离开 SO(3) 流形（$R_{k+1}$ 不再是正交矩阵）。

本节基于 Sola et al. 第 II-I 节，讲解如何在流形上正确地做离散积分，并建立与 IMU 预积分（Forster et al., 2017）的联系。这是理解 VINS-Mono、ORB-SLAM3、FAST-LIO 等系统中 IMU 处理模块的基础。

### 动机：为什么需要"在流形上积分"

考虑一个陀螺仪，每隔 $\delta t = 0.01$s 给出一个角速度测量 $\omega_k$。我们想从 $R_0 = I$ 开始，积分得到每个时刻的旋转 $R_k$。

**天真方法（欧拉积分）**：

$$R_{k+1} = R_k + \dot{R}_k \cdot \delta t = R_k + R_k [\omega_k]_\times \cdot \delta t = R_k (I + [\omega_k]_\times \delta t)$$

问题：$I + [\omega_k]_\times \delta t$ 不是旋转矩阵！它不满足正交约束 $R^TR = I$。经过 100 步积分后，$R_{100}$ 的行列式可能偏离 1，旋转矩阵的正交性被破坏。

如果你在每步之后做重正交化（如 QR 分解或 SVD 投影），虽然能把结果"拉回"到 SO(3) 上，但（1）计算量增大，（2）投影方向的选择引入额外误差。

**正确方法（流形积分）**：

### 理论：连续时间与离散时间

**连续时间的旋转运动方程**：

$$\dot{R}(t) = R(t) [\omega(t)]_\times$$

其中 $\omega(t) \in \mathbb{R}^3$ 是 body 坐标系中的角速度。这个方程说的是：旋转矩阵的时间导数等于当前旋转乘以角速度的反对称矩阵。

如果 $\omega$ 是常数，这个 ODE 的精确解是：

$$R(t) = R(0) \cdot \exp([\omega]_\times t) = R(0) \cdot \text{Exp}(\omega t)$$

**离散化**：在每个时间步内假设 $\omega_k$ 恒定（零阶保持，即 IMU 测量在采样间隔内不变），则离散积分为：

$$R_{k+1} = R_k \circ \text{Exp}(\omega_k \delta t)$$

用 $\oplus$ 算子表达：

$$R_{k+1} = R_k \oplus (\omega_k \delta t)$$

这个公式保证 $R_{k+1}$ 始终在 SO(3) 上——因为 $\text{Exp}(\omega_k \delta t)$ 是旋转矩阵，两个旋转矩阵的乘积仍然是旋转矩阵。

**为什么这比欧拉积分好？** 比较两种方法：

| 方面 | 欧拉积分 $R(I + [\omega]\delta t)$ | 流形积分 $R \circ \text{Exp}(\omega \delta t)$ |
|------|------|------|
| 是否保持在流形上 | 否（$\det \neq 1$） | 是（群封闭性） |
| 精度（$\delta t$ 趋近 0） | $O(\delta t)$ 一阶 | $O(\delta t)$ 一阶（但常数更小） |
| 长时间积分 | 误差不断累积+漂移出流形 | 误差累积但保持在流形上 |
| 需要重正交化 | 是 | 否 |

对于**四元数**表示的旋转，流形积分的形式为：

$$q_{k+1} = q_k \cdot \text{Exp}(\omega_k \delta t)$$

其中四元数的 Exp 定义为：

$$\text{Exp}(\omega) = \begin{cases} [\cos(\theta/2), \sin(\theta/2) \hat{u}] & \text{if } \theta = \|\omega\| > 0 \\ [1, 0, 0, 0] & \text{if } \theta = 0 \end{cases}$$

这里 $\hat{u} = \omega / \theta$ 是旋转轴。

### SE(3) 的离散积分

对于完整的刚体运动（旋转 + 平移），离散积分需要同时更新旋转和位置。典型的 IMU 运动模型是：

$$R_{k+1} = R_k \cdot \text{Exp}(\omega_k \delta t)$$

$$v_{k+1} = v_k + R_k (a_k - b_a) \delta t + g \delta t$$

$$p_{k+1} = p_k + v_k \delta t + \frac{1}{2} [R_k (a_k - b_a) + g] \delta t^2$$

其中 $a_k$ 是加速度计测量，$b_a$ 是加速度计偏差，$g$ 是重力向量（在世界坐标系中）。

注意：位置和速度的更新使用普通的向量加法（因为它们在 $\mathbb{R}^3$ 中），只有旋转使用流形积分。

### 与 IMU 预积分的联系

**问题**：在 SLAM 优化中，如果每次调整位姿 $T_i$ 的值，就需要重新从 $T_i$ 开始积分 IMU 数据到 $T_j$——这在优化的每次迭代中都要做，计算量巨大。

**IMU 预积分**（Forster et al., 2017）的核心思想：把两个关键帧之间的 IMU 测量"预先积分"为一个与关键帧位姿无关的增量 $(\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij})$：

$$\Delta R_{ij} = \prod_{k=i}^{j-1} \text{Exp}((\omega_k - b_g) \delta t)$$

$$\Delta v_{ij} = \sum_{k=i}^{j-1} \Delta R_{ik} (a_k - b_a) \delta t$$

$$\Delta p_{ij} = \sum_{k=i}^{j-1} \left[\Delta v_{ik} \delta t + \frac{1}{2} \Delta R_{ik} (a_k - b_a) \delta t^2\right]$$

这些增量只依赖于 IMU 测量值和偏差估计，不依赖于关键帧位姿 $T_i, T_j$。因此在优化迭代中不需要重新积分。

**预积分与流形积分的关系**：$\Delta R_{ij}$ 的计算本身就是流形积分——每步用 $\text{Exp}(\cdot)$ 确保结果在 SO(3) 上。如果用欧拉积分，不仅会累积数值误差，还会导致 $\Delta R_{ij}$ 不再是旋转矩阵，后续的优化会出问题。

**偏差更新**：当优化器调整了偏差估计 $b_g, b_a$ 后，预积分量需要做一阶修正（而不是重新积分）：

$$\Delta R_{ij}(b_g + \delta b_g) \approx \Delta R_{ij}(b_g) \cdot \text{Exp}\left(\frac{\partial \Delta R_{ij}}{\partial b_g} \delta b_g\right)$$

这里的 $\frac{\partial \Delta R_{ij}}{\partial b_g}$ 正是在积分过程中累积的流形 Jacobian——又回到了 23.9 节的内容。

### manif 代码：流形积分示例

```cpp
#include <manif/SO3.h>
#include <manif/SE3.h>
#include <vector>

// 模拟陀螺仪数据：绕 z 轴匀速旋转
double omega_z = 0.5;  // rad/s
double dt = 0.01;      // 100 Hz
int N = 100;           // 1 秒的数据

// === 方法 1：流形积分（正确）===
manif::SO3d R_manifold = manif::SO3d::Identity();
for (int k = 0; k < N; ++k) {
    Eigen::Vector3d omega(0, 0, omega_z);
    R_manifold = R_manifold.compose(manif::SO3d::Exp(omega * dt));
    // 等价于 R_manifold = R_manifold + (omega * dt)，
    // 其中 + 是 manif 的 operator+ 即 ⊕
}

// === 方法 2：欧拉积分（错误，仅用于对比）===
Eigen::Matrix3d R_euler = Eigen::Matrix3d::Identity();
for (int k = 0; k < N; ++k) {
    Eigen::Vector3d omega(0, 0, omega_z);
    Eigen::Matrix3d omega_hat;
    omega_hat <<      0, -omega.z(),  omega.y(),
               omega.z(),          0, -omega.x(),
              -omega.y(),  omega.x(),          0;
    R_euler = R_euler * (Eigen::Matrix3d::Identity() + omega_hat * dt);
}

// 比较
std::cout << "流形积分 det(R): " << R_manifold.rotation().determinant() << std::endl;
// 应精确为 1.0
std::cout << "欧拉积分 det(R): " << R_euler.determinant() << std::endl;
// 会偏离 1.0，如 1.0025

std::cout << "流形积分 R^T R - I:\n"
          << (R_manifold.rotation().transpose() * R_manifold.rotation()
              - Eigen::Matrix3d::Identity()).norm() << std::endl;
// 应为 ~1e-15

std::cout << "欧拉积分 R^T R - I:\n"
          << (R_euler.transpose() * R_euler
              - Eigen::Matrix3d::Identity()).norm() << std::endl;
// 会是 ~1e-3 量级
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：IMU 角速度单位错误**
> 错误做法：将角速度从 deg/s 转换为 rad/s 时忘记乘以 $\pi/180$，或在某些 IMU 驱动中已经自动转换但又手动转了一次。
> 现象：积分出的旋转偏大（多转了约 57 倍）或偏小，轨迹完全错误。
> 根本原因：`Exp(omega * dt)` 期望 `omega` 的单位是 rad/s。
> 正确做法：在 IMU 数据预处理阶段统一转换为 rad/s 和 m/s^2，添加单元测试验证。

> 🧠 **思维陷阱：认为"流形积分就不需要高阶方法了"**
> 新手想法："既然流形积分保持在 SO(3) 上，精度就不是问题了。"
> 实际上：流形积分的零阶保持（$\omega$ 在一个步长内视为常数）仍然只有一阶精度。当 IMU 频率低或旋转速度快时，误差依然显著。更高阶的方法包括：中值积分（使用 $(\omega_k + \omega_{k+1})/2$，二阶精度）和四阶 Runge-Kutta（流形上的 RK4）。ORB-SLAM3 使用中值积分。
> 正确做法：理解精度阶数的概念，根据应用场景选择合适的积分方法。

### 练习

**练习 23.11.1**（⭐⭐⭐）：实现并比较三种旋转积分方法：(1) 欧拉积分 + SVD 重正交化；(2) 零阶保持流形积分；(3) 中值法流形积分（使用 $\omega_{k+1/2} = (\omega_k + \omega_{k+1})/2$）。对绕非轴方向（如 $\omega = [1, 2, 3]$ rad/s）旋转 10 秒的场景，比较三者与解析解（$R(t) = \text{Exp}(\omega t)$）的旋转误差。

**练习 23.11.2**（⭐⭐⭐）：写一个简单的 IMU 预积分类，累积 $\Delta R_{ij}$ 和 $\Delta v_{ij}$。输入是一系列 $(a_k, \omega_k, \delta t)$，输出是预积分量。验证当偏差为零时，预积分结果与直接积分一致。

---

## 23.12 SO(3)/SE(3) 公式速查表 ⭐⭐

### 这一节解决什么问题

前面几节推导了大量公式，实际编程时经常需要快速查阅具体表达式。本节基于 Sola et al. 论文附录 B 和 D，将 SO(3) 和 SE(3) 最常用的公式整理为速查表。建议打印出来或保存为随手可查的文档。

### SO(3) 速查表

**基本量**：$\tau = \theta \hat{u} \in \mathbb{R}^3$，$\theta = \|\tau\|$ 是旋转角度，$\hat{u} = \tau / \theta$ 是旋转轴单位向量。

| 操作 | 公式 | 说明 |
|------|------|------|
| **指数映射 Exp($\tau$)** | $\mathbf{I} + \frac{\sin\theta}{\theta}[\tau]_\times + \frac{1 - \cos\theta}{\theta^2}[\tau]_\times^2$ | Rodrigues 公式 |
| **对数映射 Log($R$)** | $\theta \hat{u}$，其中 $\theta = \arccos\frac{\text{tr}(R) - 1}{2}$，$[\hat{u}]_\times = \frac{R - R^T}{2\sin\theta}$ | 注意 $\theta = 0$ 和 $\theta = \pi$ 的特殊处理 |
| **伴随矩阵 Ad$_R$** | $R$（就是旋转矩阵本身） | SO(3) 的伴随最简单 |
| **逆** | $R^{-1} = R^T$ | 正交矩阵的性质 |
| **群作用** | $R \cdot v = Rv$ | 矩阵-向量乘法 |
| **Hat 算子 $[\cdot]_\times$** | $[\tau]_\times = \begin{bmatrix} 0 & -\tau_3 & \tau_2 \\ \tau_3 & 0 & -\tau_1 \\ -\tau_2 & \tau_1 & 0 \end{bmatrix}$ | $[\tau]_\times v = \tau \times v$ |

**Jacobian 速查**：

| Jacobian | 公式 | 维度 |
|----------|------|------|
| **右 Jacobian $\mathbf{J}_r(\tau)$** | $\mathbf{I} - \frac{1-\cos\theta}{\theta^2}[\tau]_\times + \frac{\theta-\sin\theta}{\theta^3}[\tau]_\times^2$ | $3 \times 3$ |
| **右 Jacobian 逆 $\mathbf{J}_r^{-1}(\tau)$** | $\mathbf{I} + \frac{1}{2}[\tau]_\times + \left(\frac{1}{\theta^2} - \frac{1+\cos\theta}{2\theta\sin\theta}\right)[\tau]_\times^2$ | $3 \times 3$ |
| **左 Jacobian $\mathbf{J}_l(\tau)$** | $\mathbf{I} + \frac{1-\cos\theta}{\theta^2}[\tau]_\times + \frac{\theta-\sin\theta}{\theta^3}[\tau]_\times^2$ | $3 \times 3$ |
| **Inverse: $\mathbf{J}_{R}^{R^{-1}}$** | $-R$ | $3 \times 3$ |
| **Compose: $\mathbf{J}_{R_1}^{R_1 R_2}$** | $R_2^T$ | $3 \times 3$ |
| **Compose: $\mathbf{J}_{R_2}^{R_1 R_2}$** | $\mathbf{I}$ | $3 \times 3$ |
| **Action: $\mathbf{J}_{R}^{Rv}$** | $-R[v]_\times$ | $3 \times 3$ |
| **Action: $\mathbf{J}_{v}^{Rv}$** | $R$ | $3 \times 3$ |

**注意左右 Jacobian 的区别**：$\mathbf{J}_l$ 和 $\mathbf{J}_r$ 仅在 $[\tau]_\times$ 项的符号不同——$\mathbf{J}_l$ 中是 $+$，$\mathbf{J}_r$ 中是 $-$。这与 $\mathbf{J}_l(\tau) = \mathbf{J}_r(-\tau)$ 一致（取反 $\tau$ 等于取反 $[\tau]_\times$，翻转符号）。

**小角度近似**（$\theta < 10^{-10}$）：

| 量 | 近似值 |
|---|--------|
| $\text{Exp}(\tau)$ | $\mathbf{I} + [\tau]_\times$ |
| $\mathbf{J}_r(\tau)$ | $\mathbf{I} - \frac{1}{2}[\tau]_\times$ |
| $\mathbf{J}_r^{-1}(\tau)$ | $\mathbf{I} + \frac{1}{2}[\tau]_\times$ |

### SE(3) 速查表

**基本量**：$\tau = [\rho^T, \theta^T]^T \in \mathbb{R}^6$（manif 排列：平移在前），$M = \begin{bmatrix} R & t \\ 0^T & 1 \end{bmatrix} \in SE(3)$。

| 操作 | 公式 | 说明 |
|------|------|------|
| **指数映射 Exp($\tau$)** | $\begin{bmatrix} \text{Exp}(\theta) & V(\theta)\rho \\ 0^T & 1 \end{bmatrix}$ | $t = V(\theta)\rho$，不是 $\rho$ 本身！ |
| **$V(\theta)$** | $\mathbf{I} + \frac{1-\cos\theta}{\theta^2}[\theta]_\times + \frac{\theta-\sin\theta}{\theta^3}[\theta]_\times^2$ | 左 Jacobian $\mathbf{J}_l$ 的 SO(3) 版本 |
| **对数映射 Log($M$)** | $\begin{bmatrix} V^{-1}(\theta)t \\ \text{Log}(R) \end{bmatrix}$ | $\rho = V^{-1}t$，需要 $V$ 的逆 |
| **$V^{-1}(\theta)$** | $\mathbf{I} - \frac{1}{2}[\theta]_\times + \frac{1}{\theta^2}\left(1 - \frac{\theta\sin\theta}{2(1-\cos\theta)}\right)[\theta]_\times^2$ | 数值实现时注意 $\theta \approx 0$ 的情况 |
| **伴随矩阵 Ad$_M$** | $\begin{bmatrix} R & [t]_\times R \\ \mathbf{0}_{3\times3} & R \end{bmatrix}$ | $6 \times 6$ 矩阵 |
| **逆 $M^{-1}$** | $\begin{bmatrix} R^T & -R^T t \\ 0^T & 1 \end{bmatrix}$ | 不是简单的转置！ |
| **群作用** | $M \cdot p = Rp + t$ | 刚体运动 |

**Jacobian 速查**：

| Jacobian | 公式 | 维度 |
|----------|------|------|
| **Inverse: $\mathbf{J}_{M}^{M^{-1}}$** | $-\text{Ad}_M = -\begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix}$ | $6 \times 6$ |
| **Compose: $\mathbf{J}_{M_1}^{M_1 M_2}$** | $\text{Ad}_{M_2^{-1}}$ | $6 \times 6$ |
| **Compose: $\mathbf{J}_{M_2}^{M_1 M_2}$** | $\mathbf{I}_6$ | $6 \times 6$ |
| **Action: $\mathbf{J}_{M}^{M \cdot p}$** | $\begin{bmatrix} R & -R[p]_\times \end{bmatrix}$ | $3 \times 6$ |
| **Action: $\mathbf{J}_{p}^{M \cdot p}$** | $R$ | $3 \times 3$ |

### 切空间排列顺序对照

不同的库和论文使用不同的切空间排列，这是 bug 的主要来源之一：

| 参考 | 切空间排列 | 说明 |
|------|-----------|------|
| **manif** | $[\rho^T, \theta^T]^T$（平移在前） | 本教程的默认约定 |
| **Sola et al. 论文** | $[\theta^T, \rho^T]^T$（旋转在前） | 论文的理论推导使用此约定 |
| **Sophus** | $[\rho^T, \theta^T]^T$（平移在前） | 与 manif 相同 |
| **GTSAM** | $[\theta^T, \rho^T]^T$（旋转在前） | 右扰动（`retract` = 右乘），旋转在前 |
| **Ceres SE3Parameterization** | 取决于实现 | 需检查源码 |

**转换方法**：如果需要在不同排列之间转换 Jacobian，可以用置换矩阵 $P$：

$$J_{\text{manif}} = P \cdot J_{\text{Sola}} \cdot P^T$$

其中 $P = \begin{bmatrix} 0_{3\times3} & I_3 \\ I_3 & 0_{3\times3} \end{bmatrix}$ 交换前 3 行/列和后 3 行/列。

### manif 验证代码

```cpp
// 验证速查表中的公式
#include <manif/SO3.h>
#include <manif/SE3.h>

// === SO(3) Rodrigues 公式验证 ===
Eigen::Vector3d tau(0.3, -0.5, 0.7);
double theta = tau.norm();
Eigen::Matrix3d tau_hat = manif::skew(tau);

// 手动计算 Rodrigues 公式
Eigen::Matrix3d R_manual = Eigen::Matrix3d::Identity()
    + (std::sin(theta) / theta) * tau_hat
    + ((1.0 - std::cos(theta)) / (theta * theta)) * tau_hat * tau_hat;

// manif 计算
manif::SO3d R_manif = manif::SO3d::Exp(tau);

std::cout << "Rodrigues 验证误差: "
          << (R_manual - R_manif.rotation()).norm() << std::endl;
// 应为 ~1e-15

// === 右 Jacobian 公式验证 ===
Eigen::Matrix3d Jr_manual = Eigen::Matrix3d::Identity()
    - ((1.0 - std::cos(theta)) / (theta * theta)) * tau_hat
    + ((theta - std::sin(theta)) / (theta * theta * theta)) * tau_hat * tau_hat;

manif::SO3d::Jacobian Jr_manif;
manif::SO3d::Exp(tau, Jr_manif);

std::cout << "J_r 验证误差: " << (Jr_manual - Jr_manif).norm() << std::endl;
// 应为 ~1e-15

// === SE(3) 伴随矩阵验证 ===
manif::SE3d T = manif::SE3d::Random();
Eigen::Matrix3d R = T.rotation();
Eigen::Vector3d t = T.translation();
Eigen::Matrix<double, 6, 6> Ad_manual;
Ad_manual << R, manif::skew(t) * R,
             Eigen::Matrix3d::Zero(), R;

std::cout << "Ad 验证误差: " << (Ad_manual - T.adj()).norm() << std::endl;
// 应为 ~1e-15
```

### 练习

**练习 23.12.1**（⭐⭐）：手动验证 SO(3) 的关系 $\mathbf{J}_l(\tau) = \mathbf{J}_r(-\tau)$：取 $\tau = [0.2, -0.3, 0.5]^T$，分别用速查表中的公式计算 $\mathbf{J}_l(\tau)$ 和 $\mathbf{J}_r(-\tau)$，确认两者相等。

**练习 23.12.2**（⭐⭐⭐）：写一个函数，输入一个 SE(3) 元素的 manif 表示，输出论文排列（旋转在前）的 Jacobian。具体来说，对 `T1.compose(T2, J1, J2)` 返回的 $J_1$，转换为 Sola 论文中旋转在前的排列顺序。用数值差分验证转换结果的正确性。

---

## 23.13 manif vs Sophus 对比 ⭐⭐

### 动机：为什么需要了解两个库

在 SLAM 社区中，manif 和 Sophus 是最常用的两个 C++ 李群库。它们服务于同一个目标——提供 SO(3)/SE(3) 的高效实现——但设计哲学截然不同。你需要了解两者的原因是：

1. **新项目推荐 manif**：群无关接口、活跃维护、配套论文、Ceres 集成好
2. **读现有代码需要 Sophus**：ORB-SLAM3、FAST-LIVO2、slambook2 等大量经典项目使用 Sophus
3. **面试和论文**：理解两者的设计取舍体现了你对 C++ 库设计和 SLAM 工程实践的深入理解

### 如果只学一个会怎样

如果你只学 manif，打开 ORB-SLAM3 的源码时会一脸茫然——`Sophus::SE3f` 是什么？`se3().log()` 和 manif 的 `log()` 返回值一样吗？twist 的分量顺序一样吗？

如果你只学 Sophus，开始新项目时可能继续用 Sophus，但 Sophus 自 2024 年起进入维护模式（维护者 Hauke Strasdat 声明不再添加新功能），bug 修复也变得缓慢。长期来看，manif 是更可持续的选择。

### 理论：设计哲学对比

**架构层面的差异**

Sophus 为每个群提供独立的类实现：`SO3<Scalar>`, `SE3<Scalar>`, `Sim3<Scalar>` 各自有完整的接口，彼此之间没有继承关系。这意味着如果你写了一个接受 `Sophus::SE3d` 的函数，它不能自动适用于 `Sophus::SO3d`——你需要为每个群写一个重载版本。

manif 则使用 CRTP 定义了一个 `LieGroupBase<Derived>` 基类模板，所有群（SO2, SO3, SE2, SE3, SE_2_3 等）都继承自这个基类。基类中定义了 `compose`, `inverse`, `log`, `exp`, `act` 等通用接口，派生类只需要实现少量群特有的操作。这让你可以写出群无关的模板函数（如上文的 `compute_distance` 例子）。

这两种设计各有优劣。Sophus 的优势是简单直接——每个群的实现完全独立，易于理解和调试。manif 的优势是通用性——一套代码适配所有群，但 CRTP 的模板元编程增加了编译时间和编译错误信息的复杂度。

**API 风格对比**

下面是同一操作在两个库中的写法对比。注意构造函数参数顺序和 twist 分量顺序的差异——这是两个库之间最容易出错的地方：

```cpp
// === 构造 ===

// manif：SE3d(translation, quaternion)——平移在前
manif::SE3d T_manif(translation, quaternion);

// Sophus：SE3d(quaternion, translation)——旋转在前
Sophus::SE3d T_sophus(quaternion, translation);

// === 组合 ===

// manif：两种等价写法
manif::SE3d T12 = T1.compose(T2);  // 或 T1 * T2

// Sophus：只用运算符，没有 compose 方法名
Sophus::SE3d T12 = T1 * T2;

// === 对数映射 ===

// manif：返回 Tangent 类型，需要 .coeffs() 获取向量
manif::SE3Tangentd xi = T.log();
Eigen::Matrix<double, 6, 1> xi_vec = xi.coeffs();
// xi_vec 顺序：[rho_x, rho_y, rho_z, omega_x, omega_y, omega_z]

// Sophus：直接返回 Eigen 向量
Eigen::Matrix<double, 6, 1> xi_vec_s = T_sophus.log();
// xi_vec_s 顺序：[rho_x, rho_y, rho_z, omega_x, omega_y, omega_z]（与 manif 相同）

// === 指数映射 ===

// manif：大写 Exp
manif::SE3d T = manif::SE3d::Exp(xi_vec);

// Sophus：小写 exp
Sophus::SE3d T = Sophus::SE3d::exp(xi_vec);

// === 作用于点 ===

// manif：act 方法（可同时输出 Jacobian）
Eigen::Vector3d p_world = T.act(p_body);

// Sophus：运算符重载
Eigen::Vector3d p_world = T * p_body;
```

| 特性 | manif | Sophus |
|------|-------|--------|
| 设计模式 | CRTP 群无关接口 | 每群独立实现 |
| C++ 标准 | C++14（最低要求） | C++17（曾为 C++14） |
| twist 顺序 | $[\rho, \omega]$（平移在前） | $[\rho, \omega]$（平移在前，与 manif 相同） |
| 构造函数 | `SE3d(t, q)` | `SE3d(q, t)` |
| 指数映射 | `SE3d::Exp(v)` | `SE3d::exp(v)` |
| Jacobian | 内置解析 Jacobian（所有群操作） | 提供部分导数辅助函数（`Dx_exp_x` 等） |
| 支持的群 | SO2, SO3, SE2, SE3, SE\_2(3), Bundle | SO2, SO3, SE2, SE3, Sim3, SE\_2(3) |
| Sim3 支持 | 不支持（截至 2025 年） | 支持 |
| Ceres 集成 | 官方提供 Manifold functor | 社区贡献，需手动适配 |
| 维护状态 | 活跃（artivis/manif, ~1.7k stars） | 维护模式（strasdat/Sophus, ~2.2k stars, 2024 年起） |
| 配套论文 | Sola et al., 2021 | 无 |
| 典型用户 | 新项目、学术研究 | ORB-SLAM3, FAST-LIVO2, slambook2 |

**Sim3 支持——为什么重要**

Sophus 支持 Sim(3)（相似变换群），manif 不支持。Sim(3) 在单目 SLAM 中至关重要——因为单目 SLAM 无法恢复绝对尺度，位姿只能恢复到相似变换。ORB-SLAM3 的回环检测就使用 Sim(3) 来校正尺度漂移。

Sim(3) 是一个 7 维李群，元素形如：

$$S = \begin{bmatrix} sR & t \\ 0^T & 1 \end{bmatrix}, \quad s > 0, R \in SO(3), t \in \mathbb{R}^3$$

其中 $s$ 是尺度因子。当 $s = 1$ 时，Sim(3) 退化为 SE(3)。

如果你做激光 SLAM 或双目/RGBD SLAM，尺度是已知的（深度传感器直接给出），SE(3) 足够，manif 是更好的选择。如果你做单目 SLAM 且需要 Sim(3)，Sophus 仍然是必要的。

**选型建议决策流程**

1. 新项目，不需要 Sim3 → **manif**
2. 新项目，需要 Sim3 → Sophus（但要做好长期维护的心理准备）
3. 阅读 ORB-SLAM3/FAST-LIVO2 代码 → 必须理解 **Sophus**
4. 需要群无关的模板编程 → **manif**（Sophus 做不到）
5. 需要内置解析 Jacobian → **manif**

### ⚠️ 常见陷阱

**💡 概念误区：认为 manif 和 Sophus 的 twist 分量顺序不同**

- **新手想法**："manif 用 $[\rho, \omega]$，Sophus 用 $[\omega, \rho]$，需要交换前后三个分量"
- **实际上**：当前版本的 Sophus 和 manif **都使用 $[\rho, \omega]$（平移在前，旋转在后）**。Sophus 的 `hat()` 函数将 `a.head<3>()` 映射到平移列、`a.tail<3>()` 映射到旋转块，与 manif 一致
- **根本原因**：一些旧版 Sophus 或教科书使用 $[\omega, \rho]$ 约定，导致网上流传错误的转换代码。实际使用时务必检查你安装的 Sophus 版本的 `hat()/vee()` 实现
- **正确做法**：直接传递 twist 向量，无需交换分量

```cpp
// Sophus -> manif 转换：twist 顺序一致，无需交换
Eigen::Matrix<double, 6, 1> sophus_twist = T_sophus.log();
manif::SE3d T_manif = manif::SE3d::Exp(sophus_twist);  // 直接使用
```

**⚠️ 编程陷阱：混淆两个库的构造函数参数顺序**

- **错误做法**：`Sophus::SE3d T(translation, quaternion);`——按 manif 的顺序传参
- **现象**：编译错误（类型不匹配），或者如果类型恰好兼容则得到旋转和平移互换的错误位姿
- **根本原因**：manif 构造 `SE3d(t, q)` 平移在前，Sophus 构造 `SE3d(q, t)` 旋转在前
- **正确做法**：查看 IDE 的函数签名提示，或在代码中用注释标明参数含义

**💡 概念误区：认为 Sophus 进入维护模式意味着不能用了**

- **新手想法**："Sophus 不再更新了，之前用它的代码要赶紧迁移到 manif"
- **实际上**：维护模式意味着不再添加新功能，但会继续修复关键 bug。ORB-SLAM3 和 FAST-LIVO2 的代码库已经稳定，Sophus 对它们来说完全够用。"迁移到 manif" 需要修改所有 Lie group 操作的调用点，工作量巨大且容易引入新 bug，对已有项目通常不值得
- **正确思维**：对于新代码用 manif，对于已有项目的 Sophus 代码保持不动（除非遇到 Sophus 无法解决的问题）

### 练习

**练习 23.13.1**（⭐⭐）：写一个对比程序，分别用 manif 和 Sophus 对同一个旋转做 exp 到 log 往返，验证结果一致（注意转换 twist 顺序）。取 3 个测试用例：小角度（$0.01$ rad）、中角度（$1.0$ rad）、接近 $\pi$ 的角度（$3.1$ rad）。

**练习 23.13.2**（⭐⭐）：制作一个表格，列出你在 SLAM 开发中最常用的 10 个李群操作（如 compose、inverse、act、log、exp、adjoint 等），写出在 manif 和 Sophus 中的对应 API 调用。这个表格将成为你日后阅读代码的快速参考。

---

## 23.14 manif 与 Ceres 集成 ⭐⭐⭐

### 动机：SLAM 后端优化需要流形参数化

SLAM 的后端优化本质上是一个非线性最小二乘问题：

$$\min_{T_1, \ldots, T_n} \sum_{(i,j) \in \mathcal{E}} \|e_{ij}(T_i, T_j)\|^2_{\Sigma_{ij}}$$

其中 $T_i \in SE(3)$ 是位姿，$e_{ij}$ 是测量误差（如 ICP 残差、视觉重投影误差等）。

Ceres Solver 是一个通用的非线性最小二乘求解器。默认情况下，Ceres 把所有参数当作欧几里得空间中的向量来处理：更新步是 $x_{k+1} = x_k + \Delta x$。但正如 23.1 节所述，SE(3) 上的更新不能用简单的加法——我们需要 $T_{k+1} = T_k \cdot \exp(\Delta\xi)$。

Ceres 通过 `Manifold` 接口（Ceres 2.1 引入，替代了旧的 `LocalParameterization` 接口）来处理这个问题。`Manifold` 定义了两个核心操作：

- `Plus(x, delta, x_plus_delta)`：流形上的"加法"，$x \oplus \delta$
- `Minus(y, x, y_minus_x)`：流形上的"减法"，$y \ominus x$

对于 SE(3) 的右扰动：
- `Plus(T, delta) = T * Exp(delta)`
- `Minus(T2, T1) = Log(T1^{-1} * T2)`

manif 提供了一个现成的 functor 来完成这个适配。

### 如果不用 Manifold 接口会怎样

如果你不使用 Manifold 接口，直接把四元数的 4 个参数 + 平移的 3 个参数（共 7 个）作为 Ceres 的参数块，你需要：

1. 手动在每次迭代后归一化四元数（否则四元数会偏离单位球面）
2. 处理 7 个参数但只有 6 个自由度的过参数化问题——法方程 $J^T J$ 是 $7 \times 7$ 矩阵但秩只有 6，导致奇异性
3. Jacobian 是 $m \times 7$ 而不是 $m \times 6$，浪费计算

使用 Manifold 接口后，Ceres 知道参数空间是 6 维的（虽然存储可能用了 7 个数），更新步在 $\mathbb{R}^6$ 中计算，然后通过 Plus 映射回 SE(3)。法方程变成 $6 \times 6$，没有奇异性问题。

### 理论应用：完整的 Ceres + manif 工作流

**Step 1：定义参数块和 Manifold**

```cpp
#include <ceres/ceres.h>
#include <manif/SE3.h>
#include <manif/ceres/ceres.h>

ceres::Problem problem;

// SE3d 的内部存储：7 个 double（四元数 + 平移）
// 但 Manifold 告诉 Ceres 实际自由度只有 6
std::vector<manif::SE3d> poses(num_poses, manif::SE3d::Identity());

for (int i = 0; i < num_poses; ++i) {
    problem.AddParameterBlock(
        poses[i].data(), 7,
        new manif::CeresLocalParameterizationFunctor<manif::SE3d>());
}
```

这段代码的关键点：`poses[i].data()` 返回 manif 内部存储的 7 个 double 的指针（**平移 xyz + 四元数 xyzw**——注意 manif 的存储顺序是平移在前）。`CeresLocalParameterizationFunctor` 告诉 Ceres：这 7 个数只有 6 个自由度，`Plus` 操作是 $T \cdot \exp(\delta)$（右扰动）。

**注意**：`CeresLocalParameterizationFunctor` 是 Ceres 2.1 之前的接口。Ceres 2.1 引入了 `Manifold` 接口，Ceres 2.2 移除了旧的 `LocalParameterization`。对于 Ceres 2.2+，应使用 `ceres::AutoDiffManifold` 配合 manif 的 functor。

**Step 2：定义代价函数**

关键问题是如何在 Ceres 的 AutoDiff 中使用 manif 类型。manif 的所有操作都支持 `ceres::Jet` 类型（Ceres 用于自动微分的双数类型），所以可以直接在 `operator()` 中使用 manif 操作：

```cpp
struct PoseErrorFunctor {
    manif::SE3d T_measured;

    PoseErrorFunctor(const manif::SE3d& T_meas) : T_measured(T_meas) {}

    template <typename T>
    bool operator()(const T* const pose_i_raw,
                    const T* const pose_j_raw,
                    T* residuals_raw) const {
        Eigen::Map<const manif::SE3<T>> Ti(pose_i_raw);
        Eigen::Map<const manif::SE3<T>> Tj(pose_j_raw);

        // 残差：log(T_measured^{-1} * Ti^{-1} * Tj)
        typename manif::SE3<T>::Tangent residual =
            (T_measured.template cast<T>().inverse() *
             Ti.inverse() * Tj).log();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals_raw);
        res = residual.coeffs();
        return true;
    }
};
```

这段代码有几个值得深入理解的细节：

1. `Eigen::Map<const manif::SE3<T>>` 将 Ceres 的 raw 指针零拷贝包装为 manif 类型——没有内存分配开销
2. `T_measured.template cast<T>()` 将 double 精度的测量值转换为 Jet 类型，让 AutoDiff 能追踪对它的依赖
3. 整个残差计算链（inverse, compose, log）都在 Jet 类型上操作，Ceres 自动追踪每一步的偏导数

**Step 3：添加残差块**

```cpp
for (const auto& edge : pose_graph_edges) {
    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<
        PoseErrorFunctor, 6, 7, 7>(
        new PoseErrorFunctor(edge.T_measured));

    problem.AddResidualBlock(
        cost,
        new ceres::HuberLoss(1.0),
        poses[edge.i].data(),
        poses[edge.j].data());
}
```

注意模板参数 `<PoseErrorFunctor, 6, 7, 7>`：
- `6`：残差维度（SE(3) 的 6 自由度）
- `7`：第一个参数块的存储大小（四元数 4 + 平移 3）
- `7`：第二个参数块的存储大小

残差维度是 6 而不是 7，因为残差生活在切空间（6 维），而参数块的存储维度是 7（嵌入空间维度）。这个区别是初学者最常犯的错误之一。

**Step 4：求解**

```cpp
ceres::Solver::Options options;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
options.max_num_iterations = 100;
options.minimizer_progress_to_stdout = true;

ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);

std::cout << summary.FullReport() << std::endl;
```

`SPARSE_NORMAL_CHOLESKY` 是位姿图优化的常用线性求解器。位姿图的 Hessian 矩阵（$J^TJ$）是稀疏的——每个残差只依赖两个位姿，所以 Hessian 中大部分元素是零。`SPARSE_NORMAL_CHOLESKY` 利用这种稀疏结构，比稠密求解器快几个数量级。

**AutoDiff vs 解析 Jacobian**

上面的例子使用了 AutoDiff（自动微分）。Ceres 的 AutoDiff 通过 `ceres::Jet` 类型在编译时生成 Jacobian 计算代码，效率接近手写解析 Jacobian。对于大多数 SLAM 应用，AutoDiff 的性能是足够的。

如果你需要手写解析 Jacobian（例如在极端性能要求下），可以实现 `CostFunction` 并在 `Evaluate` 方法中手动填入 Jacobian 矩阵。此时，manif 的 Jacobian 方法（如 `compose` 返回的 `J_compose_a`）非常有用。但务必注意：manif 的 Jacobian 是关于**切空间增量**的（6 维），而 Ceres 期望的 Jacobian 是关于**参数块**的（7 维），中间需要通过 Manifold 的 `PlusJacobian` 做一次链式法则变换。使用 AutoDiff + Manifold 时，Ceres 在内部自动完成这个变换，你不需要关心这个细节。

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：Ceres 参数块大小和残差维度不匹配**

- **错误做法**：`AutoDiffCostFunction<Functor, 7, 7, 7>`——残差维度写成 7（和参数块大小一样）
- **现象**：编译通过，运行时 Ceres 报错或产生错误结果
- **根本原因**：SE(3) 的残差是 6 维的（切空间维度），不是 7 维的（参数存储维度）。残差维度应该等于流形的内在维度（tangent space dimension），而不是参数的嵌入维度（ambient dimension）
- **正确做法**：残差维度 = 6，参数块大小 = 7

**⚠️ 编程陷阱：忘记设置 Manifold 导致四元数不归一化**

- **错误做法**：`problem.AddParameterBlock(pose.data(), 7)` 不带 Manifold 参数
- **现象**：优化前几步正常，之后四元数范数逐渐偏离 1（如变成 1.002, 1.015, ...），旋转矩阵不再正交，最终 exp/log 计算产生 NaN
- **根本原因**：没有 Manifold，Ceres 在 $\mathbb{R}^7$ 中做更新步 $q_{k+1} = q_k + \Delta q$，不保证 $\|q_{k+1}\| = 1$
- **正确做法**：始终使用 Manifold 参数化。或者至少使用 Ceres 内置的 `ceres::EigenQuaternionManifold` + 单独的平移参数块（但 manif 的一体化方案更简洁）

**💡 概念误区：认为 AutoDiff 比解析 Jacobian 慢很多**

- **新手想法**："自动微分毕竟是'自动'的，肯定比手写的慢"
- **实际上**：Ceres 的 AutoDiff 基于 `Jet` 类型的正向模式自动微分，在编译时展开为高效的内联代码。实测中，AutoDiff 通常只比手写解析 Jacobian 慢 1.5-3 倍。考虑到 SLAM 后端优化的时间主要花在求解线性方程组（Cholesky 分解）而不是 Jacobian 计算上，这个开销几乎可以忽略
- **延伸**：如果你的 Jacobian 有特殊的稀疏结构（如对角块结构），手写解析 Jacobian 可以利用这个结构获得更大的加速。但对于一般的位姿图优化，AutoDiff 是最佳的工程性价比选择

### 练习

**练习 23.14.1**（⭐⭐⭐）：实现一个简化的 2D 位姿图优化。用 manif 的 `SE2d` 和 Ceres 构建一个包含 5 个位姿和 8 条边（含一条回环边）的位姿图。初始值在真值基础上加入高斯噪声，运行 Ceres 优化后比较优化结果和真值的误差。使用 2D 而不是 3D 是为了降低调试难度——原理完全相同。

**练习 23.14.2**（⭐⭐⭐）：在上一题的基础上，将 `SE2d` 替换为 `SE3d`，构建一个 3D 位姿图优化问题。比较 AutoDiff 和手写解析 Jacobian 两种方式的运行时间（用 `std::chrono` 计时）。提示：手写解析 Jacobian 时使用 manif 的 `compose` Jacobian 输出。

---

## 23.15 Sophus 实战（ORB-SLAM3/FAST-LIVO2 代码解读）⭐⭐⭐

### 动机：读懂 SLAM 经典代码必须理解 Sophus

ORB-SLAM3 和 FAST-LIVO2 是 SLAM 领域最有影响力的两个开源系统。ORB-SLAM3 是视觉（+惯性）SLAM 的标杆，FAST-LIVO2 是激光-视觉-惯性融合 SLAM 的代表作。**它们都使用 Sophus 库处理旋转和位姿**。要深入理解这些系统的代码，Sophus 的 API 是绕不过去的。

### 如果不了解 Sophus 在真实 SLAM 中的用法

你可能在 API 文档层面了解 Sophus，但不知道在真实 SLAM 系统中，Sophus 是如何被使用的。比如：

- ORB-SLAM3 为什么用 `SE3f`（float）而不是 `SE3d`（double）？
- FAST-LIVO2 的 ESKF 中，`SO3d` 的 `log()` 用在了什么场景？
- 状态更新时的 `exp(delta) * R_old` 和 `R_old * exp(delta)` 有什么区别？

这些问题只有在具体的代码上下文中才能回答。

### 理论应用：Sophus 核心 API

**SO3d 和 SE3d**

Sophus 在内部使用四元数存储 SO(3)（和 manif 一样），使用"四元数 + 平移向量"存储 SE(3)。

```cpp
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

// === SO3d 构造 ===
Sophus::SO3d R_identity;
Sophus::SO3d R_from_quat(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ())));
Sophus::SO3d R_from_matrix(rotation_matrix);

// 从旋转向量构造
Eigen::Vector3d rot_vec(0.0, 0.0, M_PI/4);
Sophus::SO3d R_from_exp = Sophus::SO3d::exp(rot_vec);

// === SE3d 构造 ===
Sophus::SE3d T_identity;
Sophus::SE3d T_from_parts(R_from_quat, Eigen::Vector3d(1.0, 2.0, 3.0));
// Sophus 构造函数是 (SO3, translation)，旋转在前

// 6 维 twist 构造
// Sophus 的 twist 顺序：[rho_x, rho_y, rho_z, omega_x, omega_y, omega_z]（与 manif 相同）
Eigen::Matrix<double, 6, 1> twist;
twist << 1.0, 2.0, 3.0, 0.0, 0.0, M_PI/4;
Sophus::SE3d T_from_exp = Sophus::SE3d::exp(twist);
```

**常用操作**

```cpp
Sophus::SE3d T1 = Sophus::SE3d::exp(Sophus::SE3d::Tangent::Random());
Sophus::SE3d T2 = Sophus::SE3d::exp(Sophus::SE3d::Tangent::Random());

// 组合
Sophus::SE3d T12 = T1 * T2;

// 逆
Sophus::SE3d T1_inv = T1.inverse();

// 对数映射
Sophus::SE3d::Tangent xi = T1.log();

// 作用于点
Eigen::Vector3d p_body(1.0, 0.0, 0.0);
Eigen::Vector3d p_world = T1 * p_body;

// 访问旋转和平移分量
Sophus::SO3d R = T1.so3();
Eigen::Vector3d t = T1.translation();
Eigen::Matrix3d R_mat = T1.rotationMatrix();
Eigen::Matrix4d T_mat = T1.matrix();

// Adjoint
Eigen::Matrix<double, 6, 6> Ad = T1.Adj();
```

注意 Sophus 的 `Adj()` 方法用大写 A（不同于 manif 的 `adj()` 小写 a）。这种细微的命名差异在两个库混用时需要特别留意。

### ORB-SLAM3 中的 Sophus 用法

ORB-SLAM3 使用 `Sophus::SE3<float>`（即 `SE3f`）来表示相机位姿。使用 float 而不是 double 的原因是：ORB-SLAM3 的前端（特征提取、匹配、位姿估计）需要处理大量数据，float 可以减少一半的内存占用和带宽，且 SIMD 指令一次可以处理更多 float 数据（SSE 一次处理 4 个 float vs 2 个 double）。对于前端的精度需求来说，float 足够了。

以下是 ORB-SLAM3 中典型的 Sophus 用法模式：

**位姿表示和存储**

```cpp
// ORB-SLAM3 的 KeyFrame 类中（简化）
class KeyFrame {
public:
    Sophus::SE3f GetPose();         // world -> camera 变换 Tcw
    Sophus::SE3f GetPoseInverse();  // camera -> world 变换 Twc
    void SetPose(const Sophus::SE3f& Tcw);

private:
    Sophus::SE3f mTcw;
};
```

注意变量命名约定：`Tcw` 表示"从 world 到 camera 的变换"（c = camera, w = world），`Twc` 表示"从 camera 到 world"。下标的顺序是"目标_源"，即 $T_{cw}$ 将 world 坐标变换到 camera 坐标。这个约定在整个 ORB-SLAM3 代码库中一致使用，理解了它就能读懂大部分位姿相关的代码。

**点的坐标变换**

```cpp
// 将 3D 地图点从世界坐标系变换到相机坐标系
Eigen::Vector3f Pc = Tcw * Pw;
```

这一行看起来简单，但隐含了 `Sophus::SE3f::operator*(const Eigen::Vector3f&)` 的调用，内部执行 $R_{cw} \cdot P_w + t_{cw}$。

**相对位姿计算**

```cpp
// 计算关键帧 i 和 j 之间的相对位姿
Sophus::SE3f Tij = Tciw * Tcjw.inverse();
// Tij = Tci_w * Tw_cj = Tci_cj
```

这里 `Tcjw.inverse()` 是 $T_{wc_j}$（从 camera j 到 world），然后左乘 $T_{c_iw}$（从 world 到 camera i），得到 $T_{c_ic_j}$（从 camera j 到 camera i）。这种"链式"乘法是 SLAM 代码中最常见的操作模式。

**IMU 预积分中的增量应用**

```cpp
// 应用 IMU 预积分得到的旋转增量（简化）
Sophus::SO3f delta_R = Sophus::SO3f::exp(dtheta);
Sophus::SO3f R_new = R_old * delta_R;
```

右乘：在 body 坐标系中应用旋转增量。因为 IMU 的角速度是在 body 坐标系中测量的，所以增量自然作用在右边。

### FAST-LIVO2 中的 Sophus 用法

FAST-LIVO2 使用 Error-State Kalman Filter (ESKF) 作为状态估计框架，Sophus 主要用于旋转部分的处理。FAST-LIVO2 与 ORB-SLAM3 的关键区别在于：ORB-SLAM3 用 SE3f 一体化表示位姿，FAST-LIVO2 将旋转（SO3d）和平移（Vector3d）分开存储。

**状态定义（简化）**

```cpp
struct State {
    Sophus::SO3d R;       // 旋转（世界到机体）
    Eigen::Vector3d p;    // 位置（世界坐标系）
    Eigen::Vector3d v;    // 速度（世界坐标系）
    Eigen::Vector3d bg;   // 陀螺仪零偏
    Eigen::Vector3d ba;   // 加速度计零偏
};
```

为什么分开存储而不用 SE3d？因为 ESKF 的状态不仅包含位姿，还包含速度和 IMU 零偏。将旋转和平移分开更方便和 ESKF 的 15 维状态向量对应。

**ESKF 的预测步**

```cpp
void predict(State& state, const Eigen::Vector3d& gyro,
             const Eigen::Vector3d& accel, double dt) {
    Eigen::Vector3d omega = gyro - state.bg;
    Eigen::Vector3d a = accel - state.ba;

    // 旋转更新：R_new = R_old * Exp(omega * dt)
    // 右乘——因为 omega 在 body 坐标系中
    state.R = state.R * Sophus::SO3d::exp(omega * dt);

    // 速度更新（世界坐标系下）
    Eigen::Vector3d a_world = state.R * a + Eigen::Vector3d(0, 0, -9.81);
    state.v += a_world * dt;

    // 位置更新
    state.p += state.v * dt + 0.5 * a_world * dt * dt;
}
```

**ESKF 的更新步中的误差状态**

```cpp
void update(State& state, const Eigen::Matrix<double, 15, 1>& delta_x) {
    // 旋转误差用左扰动：R_corrected = Exp(delta_theta) * R_old
    Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
    state.R = Sophus::SO3d::exp(delta_theta) * state.R;

    // 其他状态的更新是简单的加法
    state.p += delta_x.segment<3>(3);
    state.v += delta_x.segment<3>(6);
    state.bg += delta_x.segment<3>(9);
    state.ba += delta_x.segment<3>(12);
}
```

注意这里的关键细节：**预测步用右乘，更新步用左乘**。这不是随意的选择：

- 预测步右乘 `R * exp(omega*dt)`：IMU 角速度 $\omega$ 在 body 坐标系中测量，增量自然在 body 系中应用（右乘）
- 更新步左乘 `exp(delta_theta) * R`：误差状态 $\delta\theta$ 定义在 world 坐标系中（FAST-LIVO2 的 ESKF 约定），所以修正在 world 系中应用（左乘）

如果你把更新步也写成右乘，Jacobian 推导和协方差传播都需要相应修改。两种约定都是合法的，但**必须全程一致**。

**关键 API 差异总结**

| 场景 | ORB-SLAM3 | FAST-LIVO2 |
|------|-----------|------------|
| 标量类型 | `float` | `double` |
| 主要群 | `SE3f` | `SO3d` + 单独的平移 |
| 状态估计方法 | 因子图优化（g2o） | ESKF |
| 旋转增量方向 | 右乘 | 预测右乘，更新左乘 |
| 位姿存储 | `Sophus::SE3f mTcw` | `Sophus::SO3d R` + `Eigen::Vector3d p` |
| 命名约定 | `Tcw`/`Twc`（目标_源） | `R`/`p`（分开存储） |

### ⚠️ 常见陷阱

**⚠️ 编程陷阱：ORB-SLAM3 用 float，你的代码用 double，类型不匹配**

- **错误做法**：`Sophus::SE3d my_pose = orb_slam_frame->GetPose();`——SE3f 赋给 SE3d
- **现象**：编译错误，Sophus 不允许 SE3f 和 SE3d 之间的隐式转换
- **根本原因**：Sophus（和 Eigen）为了防止精度丢失的静默 bug，禁止 float/double 之间的隐式转换
- **正确做法**：显式 cast：`Sophus::SE3d my_pose = orb_slam_frame->GetPose().cast<double>();`

**⚠️ 编程陷阱：混淆预测步和更新步的乘法方向**

- **错误做法**：更新步中写 `state.R = state.R * Sophus::SO3d::exp(delta_theta)`（右乘，但 ESKF 的 Jacobian 是按左乘推导的）
- **现象**：第一次更新后状态看起来变好了，但之后越更新越差。协方差矩阵不收敛
- **根本原因**：ESKF 的误差状态动力学方程（$F$ 矩阵和 $G$ 矩阵）是在特定扰动约定下推导的。如果你的代码用了不同的约定，$F$ 和 $G$ 与实际更新不匹配，协方差传播就是错的
- **正确做法**：严格按照 ESKF 推导时的约定执行更新。如果论文/教材用左扰动推导，代码就用左乘

**💡 概念误区：认为 ESKF 中预测和更新必须用同一种扰动约定**

- **新手想法**："既然更新步用左扰动，那预测步也应该用左扰动，即 R_new = Exp(omega*dt) * R_old"
- **实际上**：预测步中 IMU 的角速度 $\omega$ 是在 body 坐标系中测量的，所以旋转增量 $\exp(\omega \cdot dt)$ 自然要**右乘**（即在 body 坐标系中施加旋转）。如果你左乘，意味着在 world 坐标系中施加一个 body 坐标系下的角速度旋转——这在物理上是错误的
- **正确理解**：预测步的"左乘还是右乘"由物理含义决定（IMU 测量在 body 系 -> 右乘），更新步的扰动约定由你的 Jacobian 推导约定决定（可以选左或右，但要和协方差传播一致）

**🧠 思维陷阱：认为读懂 SLAM 代码只需要理解算法**

- **新手想法**："我理解了 ESKF 的数学推导，就能读懂 FAST-LIVO2 的代码"
- **实际上**：算法理解只是第一步。真实代码中还有大量的工程细节：坐标系约定（哪个是 world，哪个是 body？）、变量命名约定（Tcw 还是 Twc？）、数据类型选择（float 还是 double？）、库 API 差异（manif 的 log 顺序和 Sophus 的 log 顺序不同）。这些细节在论文中通常不会提到，但在代码中处处可见
- **正确思维**：读 SLAM 代码时，先搞清楚三件事：(1) 坐标系约定，(2) 扰动方向约定，(3) 使用的库及其 API 约定。这三件事搞清楚了，代码的逻辑就清晰了

### 练习

**练习 23.15.1**（⭐⭐⭐）：用 Sophus 实现一个简化的 ESKF 旋转更新。给定初始旋转 $R_0 = I$，模拟 100 步 IMU 数据（常数角速度 $\omega = [0.1, 0, 0]$ rad/s，$dt = 0.01$ s），执行预测步（右乘）。然后模拟一次外部观测修正：真实旋转为 $R_{\text{true}}$，计算旋转误差 $\delta\theta = \log(R_{\text{true}} \cdot R_{\text{pred}}^{-1})$，执行更新步（左乘）。验证更新后的旋转更接近真值。

**练习 23.15.2**（⭐⭐）：阅读 ORB-SLAM3 源码中 `Tracking` 类的位姿更新部分，识别出所有 `Sophus::SE3f` 的操作，列出使用了哪些 API（构造、组合、inverse、log、作用于点等），并用 manif 等价 API 写出对应代码。注意转换 twist 顺序和构造函数参数顺序。

**练习 23.15.3**（⭐⭐⭐）：写一个完整的程序，模拟一个简化的 FAST-LIVO2 ESKF 流程：(1) 初始化状态为原点零速度；(2) 输入 200 帧 IMU 数据做预测；(3) 每 10 帧输入一次"激光雷达观测"（即真值位姿加噪声），做 ESKF 更新。绘制预测轨迹和更新后轨迹的对比，验证更新有效修正了 IMU 积分漂移。

---

## 23.16 流形代码验证与工程边界 ⭐⭐⭐

> **这一节解决什么问题**：李群代码最常见的问题不是公式背不下来，而是左/右扰动、切向量排列、四元数布局和数值边界在工程代码中交叉出现。本节给出可执行的验证框架，把"公式正确"转化为"代码在边界情况下仍正确"。

### 动机：流形错误通常表现为慢性发散

回顾 通用库·Eigen：Eigen 的 `Map` 和 `noalias()` 错误会直接污染矩阵。李群错误更隐蔽：如果你把右扰动 Jacobian 用在左扰动更新里，前几次迭代可能仍然下降，但收敛速度异常慢，最后停在一个看似合理却有系统偏差的位置。

如果不做流形验证会怎样？你会把优化发散归咎于 Ceres 参数、鲁棒核函数或初始值，而实际根因可能是 $\text{Log}(\Delta T^{-1}T_i^{-1}T_j)$ 的扰动方向和 `Plus` 实现不一致。

> **本质洞察**：流形代码的正确性不是由单个公式保证的，而是由一组约定共同保证的：坐标系、左右扰动、切向量排列、四元数内存布局、残差定义和 Jacobian 测量方向必须闭合。

### 四个必须自动化的验证

| 验证项 | 数学条件 | 工程实现 |
|--------|----------|----------|
| Plus/Minus 互逆 | $(T \oplus \delta) \ominus T = \delta$ | 随机采样小扰动，误差 < $10^{-10}$ |
| 群运算一致 | $(T_1T_2)p = T_1(T_2p)$ | 随机位姿和点，比较作用结果 |
| Jacobian 正确 | 解析 Jacobian ≈ 有限差分 | 固定扰动约定和残差测量方向 |
| 边界稳定 | $\theta \to 0$、$\theta \to \pi$ 不爆炸 | 小角 Taylor，接近 $\pi$ 时降低步长 |

这些验证类似单元测试中的"公理测试"。它们不依赖某个具体数据集，而是验证库封装是否满足李群结构本身。只要你换了 Sophus/manif/GTSAM/Ceres Manifold，就应该重新跑一次。

### 有限差分验证模板

下面以右扰动下的群作用 $r(T) = T \cdot p$ 为例。关键是：扰动施加在右侧，差分残差在欧氏空间中直接相减，因为输出是 3D 点而不是李群元素。

```cpp
#include <manif/SE3.h>
#include <Eigen/Dense>
#include <iostream>

Eigen::Matrix<double, 3, 6> numericActJacobianRight(const manif::SE3d& T,
                                                     const Eigen::Vector3d& p) {
    const double h = 1e-8;
    Eigen::Matrix<double, 3, 6> J;
    const Eigen::Vector3d y0 = T.act(p);

    for (int k = 0; k < 6; ++k) {
        manif::SE3d::Tangent delta = manif::SE3d::Tangent::Zero();
        delta.coeffs()(k) = h;

        // 右扰动：T_plus = T * Exp(delta)。必须和解析 Jacobian 的约定一致。
        const manif::SE3d T_plus = T + delta;
        const Eigen::Vector3d y_plus = T_plus.act(p);
        J.col(k) = (y_plus - y0) / h;
    }
    return J;
}

void verifyActJacobian(const manif::SE3d& T, const Eigen::Vector3d& p) {
    Eigen::Matrix<double, 3, 6> J_T;
    Eigen::Matrix3d J_p;
    (void)T.act(p, J_T, J_p);

    const auto J_num = numericActJacobianRight(T, p);
    const double err = (J_T - J_num).lpNorm<Eigen::Infinity>();
    if (err > 1e-5) {
        std::cerr << "act Jacobian mismatch: " << err << "\n";
    }
}
```

对象生命周期边界在这里同样重要：`T` 和 `p` 以 const 引用传入，只在函数内部使用；`J_T` 和 `J_num` 都是值对象，不返回对局部变量的引用。后续把它接入 Ceres 时，参数块内存还要由外层稳定容器持有。

### 接近零和接近 $\pi$ 的边界

SO(3) 的 `Exp`/`Log` 有两个数值敏感区：

| 区域 | 问题 | 正确处理 |
|------|------|----------|
| $\theta \to 0$ | $\sin\theta/\theta$、$(1-\cos\theta)/\theta^2$ 出现 $0/0$ | 使用 Taylor 展开 |
| $\theta \to \pi$ | 旋转轴符号不唯一，Log 可能跳变 | 限制单步增量，避免跨越分支切线 |

这不是 manif 或 Sophus 的缺陷，而是旋转表示的拓扑边界。三维旋转空间不是一个普通的 $\mathbb{R}^3$ 球，$\theta=\pi$ 的边界上相反旋转轴表示同一个旋转。工程上要避免一次优化步跨过这个边界；如果残差接近 $\pi$，先用更好的初始值或更强的阻尼。

### 与 Ceres/GTSAM 的接口边界

| 库 | 默认约定/风险 | 对接建议 |
|----|---------------|----------|
| manif | `SE3d` tangent 通常按 $[\rho,\omega]$ | 统一写转换函数，不在业务代码散落 `segment` |
| Sophus | `SE3::log()` 也常用平移在前 | 读旧代码时确认构造函数和存储顺序 |
| Ceres | 参数块是裸 `double*`，Manifold 管切空间 | 参数布局必须与 `Map` 和 Manifold 一致 |
| GTSAM | traits 定义 `Retract`/`Local` | 自定义类型先写 traits 测试 |

流形接口不是简单的"把李群对象塞进优化器"。优化器只看到数组和切空间增量，李群库负责把它们解释成几何对象。解释规则一旦不一致，优化器不会报错，只会沿错误方向走。

### 练习

1. **互逆验证题**：随机生成 100 个 `SE3d` 和小扰动 $\delta$，验证 `(T + delta) - T` 与 $\delta$ 的最大误差。分别测试 $\|\delta\|=10^{-8}, 10^{-3}, 0.1$。
2. **边界题**：构造接近 $\pi$ 的旋转，比较 manif 和 Sophus 的 `log()` 输出。解释为什么两个输出可能轴向符号不同但表示同一旋转。
3. **跨章综合题**：把本节有限差分验证函数接入 通用库·Ceres 的 Ceres 位姿图残差，对同一条边比较 AutoDiff Jacobian、manif 解析 Jacobian 和有限差分 Jacobian。

---

## 23.17 扩展李群与前沿应用 ⭐⭐⭐⭐

### $SE_2(3)$：扩展位姿群

前面的 SE(3) 将旋转和平移组合在一起。但在惯性导航中，状态还包括速度。ESKF（如 FAST-LIO2）用一个 15 或 18 维向量管理位置、速度、姿态、IMU 偏置——这些状态中旋转和速度之间有耦合，普通的 SE(3) 无法完整捕捉。

$SE_2(3)$（也写作 $SE_2(3)$ 或"扩展位姿群"）将旋转、平移和速度统一为一个李群：

$$SE_2(3) = \left\{\begin{bmatrix} R & v & p \\ 0_{1\times3} & 1 & 0 \\ 0_{1\times3} & 0 & 1 \end{bmatrix} \in \mathbb{R}^{5\times5} \mid R \in SO(3), v, p \in \mathbb{R}^3\right\}$$

其中 $R$ 是旋转矩阵，$v$ 是速度，$p$ 是位置。对应的李代数 $\mathfrak{se}_2(3)$ 是 9 维的（3 个旋转 + 3 个速度 + 3 个平移自由度）。

**为什么需要 $SE_2(3)$？** 传统 ESKF 将旋转、速度、位置作为独立的状态分量，在切空间中分别做加法更新。这种做法在数学上虽然可行，但忽略了一个事实：IMU 预积分的误差传播中，旋转误差会通过重力项耦合到速度和位置误差上。$SE_2(3)$ 的群结构恰好编码了这种耦合——它的伴随矩阵自然包含旋转到速度/位置的交叉项，使得协方差传播更精确。

Barrau 和 Bonnabel 在 2017 年提出了基于 $SE_2(3)$ 的"不变扩展卡尔曼滤波器"（InEKF），证明在该群上做滤波可以获得更好的一致性和收敛性。InEKF 的核心洞察是：如果你选对了群结构，状态估计的误差动力学就变成线性的（不依赖于当前状态），从而避免了 EKF 中线性化带来的不一致性。

> **反事实推理**：如果不用 $SE_2(3)$ 而直接用 SE(3) + $\mathbb{R}^3$（把速度当作普通向量）会怎样？ESKF 在大多数场景下也能工作，但在高动态运动（如无人机急速转弯）或长时间自由飞行中，旋转-速度的耦合误差会被放大。$SE_2(3)$ 的不变性保证了误差模型不随状态变化，从而在极端条件下仍有良好的收敛性。

### 群论在等变神经网络中的基础 ⭐⭐⭐⭐

李群不仅在传统几何中有用——它们也是"等变神经网络"（Equivariant Neural Networks）的数学基础。这是一个正在改变机器人感知和操作的前沿方向。

**核心概念**：一个函数 $f$ 关于群 $G$ 是等变的，如果群的作用和函数的计算可以交换：

$$f(g \cdot x) = g \cdot f(x), \quad \forall g \in G$$

直觉上，如果你旋转了输入，输出也应该以同样的方式旋转。例如，一个检测杯子把手的网络，把输入图像旋转 90 度，检测结果也应该旋转 90 度——而不需要数据增强来学习这个性质。

**在机器人学中的应用**：

| 应用 | 等变群 | 效果 |
|------|--------|------|
| 3D 点云分类/分割 | SO(3) | 对任意旋转的鲁棒性，无需旋转数据增强 |
| 抓取姿态预测 | SE(3) | 物体旋转后抓取姿态自然跟随旋转 |
| 动力学学习 | SE(3) 或 $SE_2(3)$ | 学习的动力学模型在任何参考系下一致 |
| 强化学习策略 | SO(2)、SO(3) | 对环境旋转不变的策略，泛化性更强 |

这些方法背后的数学工具——群表示论、Haar 测度、Peter-Weyl 定理——都建立在本章讲解的李群/李代数框架之上。理解了 SO(3)/SE(3) 的群结构、伴随表示和群作用，就为阅读等变网络的论文（如 E3NN、EGNN、VN-PointNet）打下了坚实基础。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为 $SE_2(3)$ 比 SE(3) "更好"因此应该替换所有 SE(3)**
>
> **实际上**：$SE_2(3)$ 只在状态包含速度且旋转-速度耦合显著时才有优势。纯位姿估计（如视觉 SLAM 的关键帧位姿）仍然用 SE(3)。选择群结构要匹配物理问题——群"越大"不等于"越好"，它意味着更多的约定需要维护和更复杂的 Jacobian。

> 🧠 **思维陷阱：认为等变网络不需要理解李群**
>
> **新手想法**："等变网络是深度学习方向，和经典李群理论没关系。"
>
> **实际上**：等变层的数学定义直接使用群表示、李代数和指数映射。不理解这些概念就无法正确构建等变层、调试对称性破缺、或理解为什么某些架构选择是必要的。本章的所有内容——群作用、伴随表示、切空间——都是等变网络论文中反复出现的工具。

### 练习

**练习 23.17.1** ⭐⭐⭐：写出 $SE_2(3)$ 的 $5 \times 5$ 矩阵表示的群操作（乘法）和逆运算。验证结果满足群公理（封闭性、结合律、单位元、逆元）。

**练习 23.17.2** ⭐⭐⭐⭐：阅读 Barrau & Bonnabel 2017 "The Invariant Extended Kalman Filter as a Stable Observer" 的第 2-3 节，总结 InEKF 相比 ESKF 的核心优势。用自己的语言解释"不变误差动力学"是什么意思。

---

## 本章小结

| 节号 | 知识点 | 难度 | 核心要点 | 常见误区 |
|------|--------|------|---------|---------|
| 23.1 | 李群/李代数概览 | ⭐⭐ | 旋转/位姿不是向量，不能直接加减 | 用欧拉角做优化会遇到万向锁 |
| 23.2 | 群作用 | ⭐⭐ | 群对空间的作用满足恒等性和相容性 | 混淆群操作（compose）和群作用（act） |
| 23.3 | 切空间与李代数 | ⭐⭐⭐ | 角速度矩阵 $R^T\dot{R}$ 属于 $\mathfrak{so}(3)$ | 认为 $\dot{R}$ 本身就是角速度 |
| 23.4 | Exp/Log 完整推导 | ⭐⭐⭐ | Rodrigues 公式和 SE(3) 指数映射闭式解 | $\theta\to 0$ 时除零；忽略 $V(\theta)$ 的修正 |
| 23.5 | Plus/Minus 算子 | ⭐⭐⭐ | $\oplus$ 是优化器与流形之间的桥梁 | 把 $\oplus$ 实现为旋转/平移分别更新 |
| 23.6 | 伴随矩阵 | ⭐⭐⭐ | $J_L = \text{Ad} \cdot J_R$，用于转换扰动约定 | 忘记 SE(3) 伴随中 $[t]_\times R$ 耦合项 |
| 23.7 | manif 核心 API | ⭐⭐ | compose, inverse, log, exp, act, Jacobian | twist 顺序 $[\rho, \omega]$，log 返回 Tangent 类型不是 VectorXd |
| 23.8 | 左/右扰动 | ⭐⭐⭐ | manif 和 GTSAM 都使用右扰动 | 混用约定导致 Jacobian 错误，优化不收敛 |
| 23.9 | 流形 Jacobian | ⭐⭐⭐⭐ | 5 类基础 Jacobian 模块 + 链式法则 | 混淆 BCH 的左/右 Jacobian 和扰动约定的左/右 |
| 23.10 | 不确定性与协方差 | ⭐⭐⭐ | 在切空间中定义高斯分布和协方差传播 | 用矩阵 16 元素的协方差表示 SE(3) 不确定性 |
| 23.11 | 离散积分与预积分 | ⭐⭐⭐ | 流形积分保持 SO(3) 约束 | 欧拉积分导致旋转矩阵离开 SO(3) |
| 23.12 | 公式速查表 | ⭐⭐ | SO(3)/SE(3) 常用公式和 Jacobian 速查 | 不同库的切空间排列顺序不同 |
| 23.13 | manif vs Sophus | ⭐⭐ | 新项目用 manif，读旧代码需要 Sophus | twist 顺序相同（均为 $[\rho,\omega]$），但构造函数参数顺序不同 |
| 23.14 | Ceres + manif | ⭐⭐⭐ | Manifold 接口提供正确的流形参数化 | 残差维度 6 vs 参数块大小 7，忘记设 Manifold |
| 23.15 | Sophus 实战 | ⭐⭐⭐ | ORB-SLAM3 用 SE3f，FAST-LIVO2 用 ESKF | float/double 隐式转换被禁止；混淆预测和更新的乘法方向 |

---

## 累积项目：Mini-LIO 新增模块——流形位姿表示

在之前的 Mini-LIO 项目中，位姿用 `Eigen::Matrix4d` 表示。本章将其升级为 manif 的 `SE3d`。

**本章新增**：

1. 将 Mini-LIO 的位姿类型从 `Eigen::Matrix4d` 改为 `manif::SE3d`
2. 将 ICP 的 Gauss-Newton 更新步从"直接修改变换矩阵元素"改为"在切空间计算增量，通过 exp 映射更新"
3. 添加位姿图优化模块：用 Ceres + manif 实现一个简单的位姿图优化，输入是相邻帧的 ICP 结果和（可选的）回环边

**预期效果**：ICP 的收敛速度应该更快（因为流形参数化避免了过参数化导致的正规方程退化），且不再需要手动做 SVD 投影来保证变换矩阵的正交性。

**代码结构建议**：

```text
mini_lio/
  src/
    pose.h          // manif::SE3d 的封装，提供与 Eigen::Matrix4d 的转换接口
    icp_manifold.h  // 基于 manif 的 ICP 实现
    pose_graph.h    // Ceres + manif 位姿图优化
  test/
    test_pose.cpp           // 位姿操作的单元测试
    test_icp_manifold.cpp   // ICP 正确性测试
    test_pose_graph.cpp     // 位姿图优化测试
```

---

## 延伸阅读

| 资料 | 难度 | 说明 |
|------|------|------|
| Sola et al., "A micro Lie theory for state estimation in robotics" (2021) | ⭐⭐ | manif 配套论文，17 页，李群入门最佳材料。**强烈建议在学习本章之前先读这篇论文** |
| slambook2 第 4 章 | ⭐⭐ | Sophus 的基本用法，中文教材，适合 Sophus 入门 |
| manif GitHub: artivis/manif | ⭐⭐ | 官方仓库，examples/ 目录有完整的使用示例 |
| Sophus GitHub: strasdat/Sophus | ⭐⭐ | 官方仓库，test/ 目录有大量用法参考 |
| Barfoot, "State Estimation for Robotics" (2017) | ⭐⭐⭐ | 第 7-9 章系统讲解了李群在状态估计中的应用，包括左右 Jacobian 的详细推导 |
| Chirikjian, "Stochastic Models, Information Theory, and Lie Groups" (2012) | ⭐⭐⭐⭐ | 研究级教材，从概率论角度理解李群上的不确定性传播 |
| Ceres Solver 文档: Manifold 接口 | ⭐⭐⭐ | 官方文档中关于 Manifold（前身 LocalParameterization）的详细说明 |
| ORB-SLAM3 论文 + 源码 | ⭐⭐⭐ | 理解 Sophus 在大型 SLAM 系统中的实际应用 |
| FAST-LIVO2 论文 + 源码 | ⭐⭐⭐ | 理解 Sophus + ESKF 的组合使用 |
| Eade, "Lie Groups for 2D and 3D Transformations" (2017) | ⭐⭐ | 短小精悍的李群入门 notes，和 Sola 的论文互为补充 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|----------|----------|----------|
| 位姿图优化前几步下降，随后停在错误形状 | 左/右扰动 Jacobian 与 `Plus` 更新方向不一致 | 1. 明确残差测量方向；2. 用有限差分验证每个参数块 Jacobian；3. 检查是否需要 Adjoint 转换 | 通用库·李群manif, 通用库·Ceres |
| `Exp(Log(T))` 对某些旋转误差很大 | 旋转接近 $\pi$，Log 分支出现轴向跳变 | 1. 打印旋转角；2. 降低优化步长或改善初始值；3. 避免用接近 $\pi$ 的单步残差 | 通用库·李群manif |
| Ceres 中旋转和平移错位 | 参数块布局与 `Eigen::Map` 或 Manifold 假设不一致 | 1. 打印参数数组前 7 个元素；2. 确认四元数是 xyzw 还是 wxyz；3. 用单位位姿残差做最小测试 | 通用库·Eigen, 通用库·李群manif, 通用库·Ceres |
| ESKF 协方差方向看起来旋转了 90 度 | 局部协方差和全局协方差混用 | 1. 标注协方差所在坐标系；2. 用 Adjoint 做坐标变换；3. 画 2D 协方差椭圆验证主轴方向 | 通用库·李群manif |
| 流形积分后旋转矩阵不再正交 | 用欧拉积分直接更新矩阵，未通过 Exp 保持在 SO(3) 上 | 1. 检查 $R^TR-I$ 范数；2. 改为 `R = R * Exp(omega*dt)`；3. 对比解析解 | 通用库·李群manif |
