# Ceres Solver——非线性优化与自动微分

> **难度**：⭐⭐～⭐⭐⭐⭐ | **建议用时**：2周 | **前置要求**：通用库·Eigen Eigen深入、通用库·李群manif 李群与manif库

---

## 前置自测

> 📋 答不出 ≥ 2 题 → 先回顾 通用库·Eigen～通用库·李群manif

1. 什么是最小二乘问题？给定 $m$ 个观测方程 $f_i(x) = 0$，如何用最小二乘法求解？
2. Gauss-Newton 法和梯度下降法有什么区别？为什么 Gauss-Newton 对最小二乘问题更高效？
3. 四元数有 4 个参数但只有 3 个自由度——在优化中这会带来什么问题？通用库·李群manif 中的 Manifold 接口如何解决？
4. 什么是自动微分（Automatic Differentiation）？它和数值微分、符号微分有什么区别？
5. 在 SLAM 的 Bundle Adjustment 中，为什么 Jacobian 矩阵是稀疏的？这个稀疏性如何被利用？

---

## 本章目标

学完本章，你将能够：

- 理解非线性最小二乘问题的数学结构，以及为什么 SLAM 的后端优化本质上就是一个大规模最小二乘问题
- 掌握 Ceres 的 Problem → CostFunction → Solver 工作流，能独立构建优化问题
- 透彻理解 Jet 双数（dual number）自动微分的原理——知道 Ceres 如何在不需要你写 Jacobian 的情况下精确计算导数
- 使用 LossFunction（鲁棒核函数）处理外点，理解 Huber/Cauchy 如何"降权"异常观测
- 正确使用 Manifold 接口处理四元数、SO(3) 等过参数化表示，衔接 通用库·李群manif 的 manif 库
- 根据问题结构选择合适的 Solver 策略和线性求解器，理解 Schur 补在 BA 中的关键作用
- 从优化结果中提取协方差信息，评估解的不确定性

**本章在课程中的位置**：通用库·李群manif 讲了如何用 manif/Sophus 表示和操作刚体变换。但在 SLAM 中，我们不只是表示位姿——我们需要**优化**位姿。给定一系列传感器观测（图像特征匹配、激光点云配准、IMU 读数），如何找到最优的位姿和地图？这就是非线性最小二乘问题，而 Ceres 是解决这类问题最强大的工具。VINS-Mono、COLMAP、Cartographer 的核心优化引擎都是 Ceres。

---

## 24.1 非线性最小二乘问题 ⭐⭐

### 动机：SLAM 后端在解什么问题？

SLAM 的前端（Tracking/Odometry）给出帧间运动的粗略估计，但这些估计会累积漂移。后端（Optimization/Mapping）的任务是利用所有观测的冗余信息，联合优化所有位姿和地图点，消除累积误差。

这个联合优化在数学上是一个**非线性最小二乘问题**：

$$\min_{x} \sum_{i=1}^{m} \|f_i(x)\|^2$$

其中 $x$ 是所有待优化变量（相机位姿、3D 地图点、IMU 偏置等），$f_i(x)$ 是第 $i$ 个残差函数——它衡量当前估计与实际观测之间的差异。

| SLAM 场景 | 残差函数 $f_i(x)$ | 待优化变量 $x$ |
|-----------|-------------------|---------------|
| 视觉 BA | 重投影误差：像素观测 - 投影(相机位姿, 3D点) | 相机位姿 + 3D 地图点 |
| 位姿图优化 | 相对位姿误差：测量值 - 计算值($T_i^{-1} T_j$) | 所有位姿 |
| IMU 预积分 | 预积分残差：IMU 积分值 - 状态差 | 位姿 + 速度 + 偏置 |
| 激光 ICP | 点到面/点到点距离 | 当前帧位姿 |

### 如果不用优化器会怎样

**方案一：手写梯度下降。** 可以工作但极慢。梯度下降的步长难以选择——太大不收敛，太小需要上万次迭代。更重要的是，梯度下降不利用最小二乘的特殊结构：Jacobian $J$ 的信息被浪费了。

**方案二：手写 Gauss-Newton。** 比梯度下降快得多（二阶收敛），但你需要自己推导并实现每个残差函数的 Jacobian、构建法方程 $J^T J \Delta x = -J^T f$、选择稀疏求解器、处理数值不稳定性。对于一个有 100 万残差和 10 万变量的 BA 问题，这是数月的工程量。

Ceres 把这些都做了：你只需要定义残差函数 $f_i(x)$，Ceres 自动计算 Jacobian（通过自动微分）、构建法方程、选择最优的稀疏求解器、处理数值问题。

### Gauss-Newton 与 Levenberg-Marquardt ⭐⭐

标准的非线性最小二乘求解使用迭代法。每次迭代，在当前估计 $x_k$ 处将残差函数线性化：

$$f_i(x_k + \Delta x) \approx f_i(x_k) + J_i \Delta x$$

其中 $J_i = \frac{\partial f_i}{\partial x}\big|_{x_k}$ 是 Jacobian 矩阵。将线性化代入最小二乘目标，得到**法方程（Normal Equations）**：

$$(J^T J) \Delta x = -J^T f$$

这就是 **Gauss-Newton** 法——解法方程得到增量 $\Delta x$，更新 $x_{k+1} = x_k + \Delta x$，重复直到收敛。

Gauss-Newton 的问题在于：当线性化不够精确时（远离最优解），$J^T J$ 可能病态甚至奇异，导致增量 $\Delta x$ 过大。**Levenberg-Marquardt (LM)** 通过添加阻尼项解决这个问题：

$$(J^T J + \lambda I) \Delta x = -J^T f$$

> 此处展示的是 Levenberg (1944) 的简化形式 $D=I$；完整的 Marquardt (1963) 形式将 $I$ 替换为 $D = \text{diag}(\sqrt{\text{diag}(J^TJ)})$，使步长在各参数方向上自适应缩放——参数梯度大的方向步长更保守，梯度小的方向步长更积极。两种形式的推导与对比详见 `01_数学/30_优化理论/50_非线性优化.md`。

无论采用哪种形式，$\lambda$ 都是阻尼因子：$\lambda$ 大时退化为梯度下降（保守、小步长），$\lambda$ 小时接近 Gauss-Newton（激进、大步长）。LM 在每次迭代中自适应调整 $\lambda$——如果这次迭代降低了目标函数就减小 $\lambda$（更信任线性化），否则增大 $\lambda$（更保守）。

**Ceres 默认使用 LM**，它在 SLAM 的各种场景下都有良好的收敛行为。

### Trust Region 与 Line Search：两大策略家族 ⭐⭐⭐

非线性优化有两大策略家族，理解它们的区别对于选择 Ceres 配置至关重要。

**Trust Region（信赖域）方法**的思路是：在当前点附近划一个"信赖域"（半径为 $\Delta$），在这个区域内用线性化模型近似原问题，求解近似问题得到增量。如果近似效果好（实际下降量/预测下降量的比值 $\rho$ 大），就扩大信赖域；如果效果差，就缩小信赖域。

$$\min_{\Delta x} \frac{1}{2}\|J \Delta x + f\|^2 \quad \text{s.t.} \quad \|\Delta x\| \leq \Delta$$

LM 可以理解为 Trust Region 的一个变种——阻尼因子 $\lambda$ 隐式地控制了步长。

**Line Search（线搜索）方法**的思路不同：先计算一个搜索方向（如梯度方向或拟牛顿方向），然后沿这个方向搜索最优步长。经典的 L-BFGS 就是线搜索方法。

| 特性 | Trust Region | Line Search |
|------|-------------|-------------|
| 每步计算 | 求解带约束子问题 | 方向 + 一维搜索 |
| 稀疏利用 | 天然支持（Schur 补） | 不太自然 |
| SLAM 适用性 | **极好**（默认选择） | 较少使用 |
| 大规模 BA | 高效 | 不推荐 |
| Ceres 支持 | `TRUST_REGION` | `LINE_SEARCH` |

**SLAM 中几乎总是使用 Trust Region。** Line Search 在某些无约束优化问题（如机器学习训练）中有优势，但在 SLAM 的稀疏最小二乘问题中，Trust Region + Schur 补的组合远远更高效。

### Powell's Dogleg：信赖域的几何直觉 ⭐⭐⭐

除了 LM，Ceres 还支持 Powell's Dogleg 信赖域策略。理解它的几何直觉有助于深入理解 Trust Region 方法。

Dogleg 的核心思想：计算两个特殊的步：

1. **Cauchy 步** $\Delta x_C$：沿负梯度方向走到使线性模型最小的点。这是最保守的步——纯梯度下降。

$$\Delta x_C = -\frac{\|g\|^2}{g^T H g} g, \quad g = J^T f$$

2. **Gauss-Newton 步** $\Delta x_{GN}$：直接解法方程 $(J^T J)\Delta x = -J^T f$。这是最激进的步。

Dogleg 用一条"狗腿形"折线连接原点 → Cauchy 点 → GN 点。在信赖域半径 $\Delta$ 内，沿这条折线找最远的点作为实际步长。

```text
         GN 步
        /
       / (第二段)
      /
     * Cauchy 步
    /
   / (第一段)
  /
 O 原点
```

**Dogleg vs LM 的关键区别**：LM 每次迭代如果步长不合适，需要调整 $\lambda$ 重新解线性方程；而 Dogleg 只需在折线上重新截断，不用重解线性方程。当线性方程求解昂贵时（大规模问题），Dogleg 每次迭代可以更快。

Ceres 还提供 `SUBSPACE_DOGLEG`，它不只考虑折线，而是在 Cauchy 步和 GN 步张成的二维子空间中求最优解——精度更高但计算量略大。

```cpp
// 使用 Dogleg 策略
options.trust_region_strategy_type = ceres::DOGLEG;
options.dogleg_type = ceres::TRADITIONAL_DOGLEG;  // 或 SUBSPACE_DOGLEG
```

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为最小二乘只能处理线性问题**
>
> **新手想法**："最小二乘不就是 $Ax = b$ 的超定方程组吗？"
>
> **实际上**：线性最小二乘（$\min \|Ax - b\|^2$）只是一个特例。SLAM 中的残差函数 $f_i(x)$ 通常是高度非线性的——涉及旋转矩阵、投影变换、指数映射。Ceres 解决的是**非线性**最小二乘，通过迭代线性化来逼近最优解。
>
> **正确理解**：非线性最小二乘 = 反复解线性最小二乘。每次迭代在当前点做线性化，解一个线性问题得到增量，更新估计，重新线性化。

> 💡 **概念误区：认为 Gauss-Newton 的 $J^T J$ 就是 Hessian 矩阵**
>
> **新手想法**："$J^T J$ 是目标函数的二阶导数矩阵。"
>
> **实际上**：目标函数 $F(x) = \frac{1}{2} \sum \|f_i\|^2$ 的真正 Hessian 是 $H = J^T J + \sum_i f_i \cdot \nabla^2 f_i$。Gauss-Newton **丢弃了** $\sum f_i \cdot \nabla^2 f_i$ 这一项，用 $J^T J$ 近似 Hessian。这个近似在残差 $f_i$ 较小时（接近最优解）非常好，但在残差很大时（远离最优解或有大量外点）会不准确——这就是为什么需要 LM 阻尼和鲁棒核函数。

> 🧠 **思维陷阱：认为 Trust Region 总是比 Line Search 好**
>
> **新手想法**："既然 SLAM 都用 Trust Region，那 Line Search 就是劣等方法。"
>
> **实际上**：两者适用于不同问题结构。Trust Region 在稀疏最小二乘中表现极好（因为能利用 Schur 补），但在参数空间极大且无稀疏结构的问题（如某些深度学习优化）中，Line Search（如 L-BFGS）可能更高效，因为它不需要显式求解线性方程组。
>
> **正确思维**：算法选择取决于问题结构，没有"总是最好"的方法。

### 练习

1. **手推题**：对一维曲线拟合 $y = ae^{bx}$，写出残差函数 $f_i(a,b) = y_i - ae^{bx_i}$，推导 Jacobian $J_i = [\frac{\partial f_i}{\partial a}, \frac{\partial f_i}{\partial b}]$，并写出法方程的具体形式。

2. **思考题**：为什么 SLAM 的 Bundle Adjustment 中 $J^T J$ 是稀疏的？提示：一个 3D 地图点只出现在观测到它的几个相机的残差中。

3. **实践题**：用 NumPy 手写一个 Gauss-Newton 求解器，拟合 $y = ae^{bx}$ 曲线。观察它从不同初始值出发的收敛行为。

4. **几何题**：在草稿纸上画出 Powell's Dogleg 的几何图形。标出 Cauchy 步、GN 步和不同信赖域半径下的实际步长。

---

## 24.2 Ceres 核心架构 ⭐⭐

### 动机：一个框架解决所有非线性最小二乘

Ceres Solver 是 Google 开源的 C++ 非线性最小二乘优化库（github.com/ceres-solver/ceres-solver），最初为 Google Street View 的大规模 Bundle Adjustment 开发。它被 VINS-Mono/Fusion、COLMAP（3D 重建标准工具）、Cartographer、OKVIS2 等核心 SLAM/SfM 系统使用。

Ceres 的设计哲学是**声明式**——你描述问题是什么（残差函数、参数、约束），Ceres 决定怎么解（线性化、求解器、收敛判断）。

### Ceres 的四个核心概念

| 概念 | 作用 | 类比 |
|------|------|------|
| `ceres::Problem` | 定义优化问题的容器 | 一张待填写的考卷 |
| `ceres::CostFunction` | 残差函数 $f_i(x)$ | 考卷上的每道题 |
| `ceres::LossFunction` | 鲁棒核函数（降权外点） | 评分时忽略抄错的答案 |
| `ceres::Solver` | 求解算法和配置 | 阅卷老师的评分策略 |

### 最简完整示例：曲线拟合 ⭐⭐

```cpp
#include <ceres/ceres.h>
#include <vector>
#include <iostream>

struct ExponentialResidual {
    double x_, y_;
    ExponentialResidual(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const {
        residual[0] = y_ - ceres::exp(params[0] * x_ * x_
                                     + params[1] * x_
                                     + params[2]);
        return true;
    }
};

int main() {
    double params[3] = {0.0, 0.0, 0.0};

    std::vector<double> xs = {0.0, 0.5, 1.0, 1.5, 2.0};
    std::vector<double> ys = {1.0, 1.3, 2.7, 7.4, 20.1};

    ceres::Problem problem;
    for (size_t i = 0; i < xs.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 3>(
                new ExponentialResidual(xs[i], ys[i])),
            nullptr,
            params);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "a=" << params[0] << " b=" << params[1] << " c=" << params[2] << "\n";
    return 0;
}
```

关键点：`operator()` 模板化于 `T`——当 `T=double` 时计算值，当 `T=Jet` 时同时计算值和导数（24.3 节详述）。`AutoDiffCostFunction<Functor, 1, 3>` 的模板参数：仿函数类型、残差维度(1)、参数块维度(3)。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：在 AutoDiff 仿函数中使用 `double` 而非 `T`**
>
> **错误做法**：`residual[0] = y_ - std::exp((double)params[0] * x_);`
>
> **现象**：编译可能通过，但 Jacobian 全为零——自动微分失效。优化完全不动。
>
> **根本原因**：将 `T`（可能是 `Jet`）转为 `double` 丢失了导数信息。
>
> **正确做法**：仿函数内所有计算都用 `T` 类型。调用数学函数用 `ceres::exp`/`ceres::cos` 而非 `std::exp`/`std::cos`。

> ⚠️ **编程陷阱：忘记 `operator()` 返回 `true`**
>
> **错误做法**：不写 `return true;`
>
> **现象**：Ceres 认为残差计算失败，跳过该残差块。优化不收敛但无编译错误。
>
> **根本原因**：返回值告诉 Ceres 残差是否有效。不写 `return` 是未定义行为。
>
> **正确做法**：始终在末尾写 `return true;`。仅在数学无效时（如对负数取对数）返回 `false`。

### 练习

1. **实践题**：编译并运行上面的曲线拟合代码。修改初始值为 `{10.0, -5.0, 3.0}`，观察 LM 是否仍然收敛。

2. **修改题**：将代码改为多参数块版本——`a`、`b`、`c` 分别作为独立的参数块。

---

## 24.3 AutoDiffCostFunction 与 Jet 类型 ⭐⭐⭐

### 动机：谁来写 Jacobian？

一个 BA 重投影残差的 Jacobian 是 2×9 矩阵（2 维残差对 6 维位姿 + 3 维点坐标）。数万个残差意味着数万个 Jacobian 矩阵——手动推导和实现极其耗时且易错。

**自动微分**解决了这个问题：你只写前向计算，Ceres 精确计算 Jacobian（不是近似）。

### 三种微分方式对比

| 方式 | 精度 | 速度 | 实现成本 | 适用场景 |
|------|------|------|---------|---------|
| 手写解析 | 精确 | 最快 | 极高 | 性能关键、残差简单 |
| 自动微分 (AD) | 精确 | 快（比解析慢 ~2-5x） | 低 | **默认选择** |
| 数值微分 | 近似 | 最慢 | 最低 | 黑盒函数、验证用 |

### Jet 双数的原理 ⭐⭐⭐

Jet 是一种扩展的数类型，每个值包含两部分：实数值 $a$ 和导数向量 $\mathbf{v}$。算术运算通过运算符重载自动传播导数：

| 运算 | 值 | 导数 |
|------|-----|------|
| $\text{Jet}(a, \mathbf{u}) + \text{Jet}(b, \mathbf{v})$ | $a + b$ | $\mathbf{u} + \mathbf{v}$ |
| $\text{Jet}(a, \mathbf{u}) \times \text{Jet}(b, \mathbf{v})$ | $a \times b$ | $a\mathbf{v} + b\mathbf{u}$ |
| $\sin(\text{Jet}(a, \mathbf{v}))$ | $\sin(a)$ | $\cos(a) \cdot \mathbf{v}$ |
| $\exp(\text{Jet}(a, \mathbf{v}))$ | $\exp(a)$ | $\exp(a) \cdot \mathbf{v}$ |

乘法的导数规则 $a\mathbf{v} + b\mathbf{u}$ 就是链式法则。无论前向计算多复杂，只要每步基本运算正确传播导数，最终得到精确 Jacobian。

**手动追踪示例**：$f(x, y) = x^2 + xy$ 在 $(3, 2)$ 处：

```text
x = Jet(3, [1,0]),  y = Jet(2, [0,1])

x² = Jet(3,[1,0]) × Jet(3,[1,0]) = Jet(9, [6,0])     ← ∂(x²)/∂x=6 ✓
xy = Jet(3,[1,0]) × Jet(2,[0,1]) = Jet(6, [2,3])     ← ∂(xy)/∂x=2, ∂(xy)/∂y=3 ✓
f  = Jet(9,[6,0]) + Jet(6,[2,3]) = Jet(15, [8,3])    ← ∂f/∂x=8, ∂f/∂y=3 ✓
```

验证：$\partial f/\partial x = 2x + y = 8$，$\partial f/\partial y = x = 3$。

### 24.3.5 Jet 源码剖析 ⭐⭐⭐

理解 Jet 的实现对于调试自动微分问题至关重要。下面我们深入 Ceres 源码中 `include/ceres/jet.h` 的核心结构。

**Jet 的数据结构**

Jet 在数学上是 $n$ 维对偶数（dual number）。普通对偶数只有一个无穷小量 $\epsilon$（$\epsilon^2 = 0$），而 Jet 扩展到 $n$ 个独立的无穷小量 $\epsilon_1, \epsilon_2, \ldots, \epsilon_n$，满足 $\epsilon_i \epsilon_j = 0$。一个 Jet 值表示为：

$$a + v_1 \epsilon_1 + v_2 \epsilon_2 + \cdots + v_n \epsilon_n$$

在 C++ 中的表示：

```cpp
// ceres/jet.h 核心结构（简化）
template <typename T, int N>
struct Jet {
    T a;               // 实数部分（函数值）
    Eigen::Matrix<T, N, 1> v;  // 无穷小部分（导数向量）

    // 默认构造
    Jet() : a(), v() {}

    // 从标量构造（常数，导数为零）
    explicit Jet(const T& value) : a(value) {
        v.setZero();
    }

    // 从标量和导数向量构造
    Jet(const T& value, int k) : a(value) {
        v.setZero();
        v[k] = T(1.0);  // 第 k 个分量设为 1
    }
};
```

这里模板参数 `N` 是导数向量的维度，等于**所有参数块维度之和**。例如一个残差函数有两个参数块——6 维位姿和 3 维点坐标——则 $N = 6 + 3 = 9$。`v` 是一个 $N$ 维向量，`v[k]` 存储函数值对第 $k$ 个参数的偏导数。

**运算符重载的关键——乘法**

乘法是最能体现 Jet 自动微分原理的运算。根据对偶数乘法规则：

$$(a + \mathbf{u}^T \boldsymbol{\epsilon}) \times (b + \mathbf{v}^T \boldsymbol{\epsilon}) = ab + (a\mathbf{v} + b\mathbf{u})^T \boldsymbol{\epsilon}$$

其中高阶项 $\mathbf{u}^T \boldsymbol{\epsilon} \cdot \mathbf{v}^T \boldsymbol{\epsilon} = 0$（因为 $\epsilon_i \epsilon_j = 0$）。

```cpp
// ceres/jet.h 中的乘法运算符
template <typename T, int N>
inline Jet<T, N> operator*(const Jet<T, N>& f, const Jet<T, N>& g) {
    return Jet<T, N>(f.a * g.a, f.a * g.v + g.a * f.v);
}
```

这正是乘法的链式法则 $\frac{\partial(fg)}{\partial x_k} = f \frac{\partial g}{\partial x_k} + g \frac{\partial f}{\partial x_k}$。

**Ceres 如何为每个参数块初始化 Jet**

当 Ceres 调用你的 `operator()` 时，它这样初始化 Jet：

```text
参数块 0 (维度 6): x[0] → Jet(x[0], e_0), x[1] → Jet(x[1], e_1), ..., x[5] → Jet(x[5], e_5)
参数块 1 (维度 3): y[0] → Jet(y[0], e_6), y[1] → Jet(y[1], e_7), y[2] → Jet(y[2], e_8)
```

其中 $e_k$ 是第 $k$ 个标准基向量。这样经过前向计算后，输出残差 `residual[i]` 的 `v` 向量的前 6 个分量就是对第一个参数块的 Jacobian，后 3 个分量就是对第二个参数块的 Jacobian。

**除法和复合函数**

```cpp
// 除法: d(f/g)/dx = (g·df - f·dg) / g²
template <typename T, int N>
inline Jet<T, N> operator/(const Jet<T, N>& f, const Jet<T, N>& g) {
    T g_inv = T(1.0) / g.a;
    return Jet<T, N>(f.a * g_inv, (f.v - f.a * g_inv * g.v) * g_inv);
}

// exp: d(exp(f))/dx = exp(f) · df/dx
template <typename T, int N>
inline Jet<T, N> exp(const Jet<T, N>& f) {
    T ef = std::exp(f.a);
    return Jet<T, N>(ef, ef * f.v);
}

// atan2: d(atan2(y,x))/dx = (-y·dx + x·dy) / (x² + y²)
template <typename T, int N>
inline Jet<T, N> atan2(const Jet<T, N>& y, const Jet<T, N>& x) {
    T denom = x.a * x.a + y.a * y.a;
    return Jet<T, N>(std::atan2(y.a, x.a),
                      (x.a * y.v - y.a * x.v) / denom);
}
```

> ⚠️ **编程陷阱：在仿函数中用 `if (params[0] > 0)` 做分支**
>
> **错误做法**：`if (params[0] > 0.5) { ... } else { ... }`
>
> **现象**：Jet 类型没有到 `bool` 的隐式转换，编译失败。
>
> **根本原因**：`params[0]` 的类型是 `T`（可能是 `Jet`），不能直接比较。
>
> **正确做法**：使用 `ceres::abs` 等函数，或者基于 `a` 成员做判断：`if (f.a > 0.5)`——但这意味着导数在分支点不连续。对于大多数 SLAM 残差，应该避免不连续分支。

**为什么 N 必须在编译期确定？**

`AutoDiffCostFunction` 的模板参数要求在编译期知道所有参数块的维度，因为 `Jet<T, N>` 的 `N` 是模板参数。这使得编译器能够：

1. 将 `v` 数组放在栈上（而非堆），避免 `malloc` 开销
2. 展开循环，启用 SIMD 向量化
3. 内联所有运算符重载

这就是为什么 `AutoDiffCostFunction<Functor, 1, 6, 3>` 比动态版本快得多——编译器在编译期就知道 $N=9$，可以做极致优化。

### 24.3.6 DynamicAutoDiffCostFunction ⭐⭐⭐

**动机：当参数维度在编译期未知时**

`AutoDiffCostFunction` 要求在模板参数中写死每个参数块的维度。但有些场景下，参数块的数量或维度只有运行时才知道：

- **SLAM 滑动窗口**：边缘化因子的参数块数量取决于窗口中的关键帧数
- **贝塞尔曲线拟合**：控制点数量不固定
- **神经网络**：层的权重维度可变

这时需要 `DynamicAutoDiffCostFunction`。

**API 差异**

```cpp
// ===== 静态版本（编译期确定维度）=====
// 模板参数: Functor, 残差维度, 参数块1维度, 参数块2维度, ...
auto* cost = new ceres::AutoDiffCostFunction<MyFunctor, 2, 6, 3>(
    new MyFunctor());

// ===== 动态版本（运行时确定维度）=====
auto* cost = new ceres::DynamicAutoDiffCostFunction<MyDynamicFunctor>(
    new MyDynamicFunctor());
cost->AddParameterBlock(6);   // 第一个参数块: 6 维
cost->AddParameterBlock(3);   // 第二个参数块: 3 维
cost->SetNumResiduals(2);     // 残差维度: 2
```

**仿函数接口的区别**

静态版本的仿函数 `operator()` 每个参数块有独立指针：

```cpp
// 静态 AutoDiff 仿函数
template <typename T>
bool operator()(const T* const pose, const T* const point, T* residual) const;
```

动态版本的仿函数接收指针数组：

```cpp
// 动态 AutoDiff 仿函数
template <typename T>
bool operator()(T const* const* parameters, T* residuals) const {
    // parameters[0] → 第一个参数块
    // parameters[1] → 第二个参数块
    // ...
}
```

**SLAM 中的典型应用：边缘化先验因子**

VINS-Mono 的边缘化模块将旧的观测信息压缩为一个先验因子。这个先验因子连接的参数块数量不固定（取决于滑动窗口中与被边缘化状态关联的变量数量）。这正是需要运行时确定参数块的场景。

VINS-Mono 实际上使用了继承 `CostFunction` 基类的自定义实现（`MarginalizationFactor`），手动管理变维参数。如果用 `DynamicAutoDiffCostFunction` 来实现类似功能，代码会更简洁但性能略低：

```cpp
struct MarginalizationResidual {
    Eigen::VectorXd b_;          // 线性化点的残差
    Eigen::MatrixXd J_;          // 线性化点的 Jacobian
    std::vector<int> block_sizes_;
    std::vector<double> x0_;     // 线性化点

    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {
        int offset = 0;
        Eigen::Matrix<T, Eigen::Dynamic, 1> dx(J_.cols());
        for (size_t i = 0; i < block_sizes_.size(); ++i) {
            for (int j = 0; j < block_sizes_[i]; ++j) {
                dx[offset + j] = parameters[i][j] - T(x0_[offset + j]);
            }
            offset += block_sizes_[i];
        }
        Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> r(residuals, b_.size());
        r = b_.cast<T>() + J_.cast<T>() * dx;
        return true;
    }
};
```

**性能：Stride 参数**

`DynamicAutoDiffCostFunction` 有一个模板参数 `Stride`（默认 4），控制每次"pass"处理几个参数的导数。Stride 越大，pass 次数越少但每次计算量越大。对于参数维度较小的问题，默认值通常足够；对于高维问题（如大型边缘化因子），可能需要调大 Stride。

> ⚠️ **编程陷阱：能用静态版本时却用了动态版本**
>
> **错误做法**：所有 CostFunction 都用 `DynamicAutoDiffCostFunction`
>
> **现象**：性能下降 2-10 倍，因为编译器无法做 Jet 维度的编译期优化。
>
> **正确做法**：只有参数块数量/维度确实在编译期未知时才用动态版本。标准的重投影因子、IMU 因子等维度固定的残差，一定用静态 `AutoDiffCostFunction`。

### SizedCostFunction：手写解析 Jacobian ⭐⭐⭐

当 AutoDiff 性能不够时（VINS-Mono 的 IMU 因子每次优化调用数百次），继承 `SizedCostFunction` 并实现 `Evaluate`。**Jacobian 矩阵是行优先（RowMajor）**——Ceres 的约定，和 Eigen 默认列优先不同。

```cpp
class PointError : public ceres::SizedCostFunction<3, 3> {
public:
    bool Evaluate(double const* const* parameters,
                  double* residuals,
                  double** jacobians) const override {
        Eigen::Map<const Eigen::Vector3d> p(parameters[0]);
        Eigen::Map<Eigen::Vector3d> r(residuals);
        r = p - target_;

        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobians[0]);
            J = Eigen::Matrix3d::Identity();
        }
        return true;
    }
};
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：在 AutoDiff 仿函数中调用 `std::` 数学函数**
>
> **错误做法**：`residual[0] = std::cos(params[0]);`
>
> **现象**：当 `T = Jet` 时可能编译失败或丢失导数。
>
> **正确做法**：使用 `ceres::cos`、`ceres::sin`、`ceres::exp`、`ceres::sqrt`——对 `double` 和 `Jet` 都有正确实现。

> ⚠️ **编程陷阱：手写 Jacobian 时忘记行优先**
>
> **错误做法**：`Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobians[0]);`（默认列优先）
>
> **现象**：Jacobian 被转置存储，优化收敛到错误解或不收敛。
>
> **正确做法**：`Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[0]);`

### 练习

1. **手推题**：对 $f(x,y) = \sin(x) \cdot e^y$，用 Jet 追踪在 $(\pi/4, 1)$ 处的 Jacobian。

2. **实践题**：用 `NumericDiffCostFunction` 替换 `AutoDiffCostFunction`，对比结果和运行时间。

3. **高级题**：为 2D 重投影手写 `SizedCostFunction`，用 `GradientChecker` 验证与 AutoDiff 一致。

4. **源码题**：阅读 `ceres/jet.h` 中 `atan2` 的实现，手动验证其导数公式与链式法则一致。

---

## 24.4 LossFunction：鲁棒核函数 ⭐⭐

### 动机：外点会毁掉优化

一个残差为 100 的外点贡献 $10000$ 的代价，远大于 99 个正常观测的总贡献 99。优化器会花大量精力"解释"外点，严重扭曲结果。

### 鲁棒核函数原理 ⭐⭐

替换平方代价 $\|f\|^2$ 为增长更慢的函数 $\rho(\|f\|^2)$：

| 核函数 | $\rho(s)$ | 大残差行为 | 适用场景 |
|--------|-----------|-----------|---------|
| 平方（无核） | $s$ | 平方增长 | 无外点 |
| Huber | 小 $s$: $s$; 大 $s$: 线性 | 线性增长 | 通用首选 |
| Cauchy | $\delta^2 \log(1 + s/\delta^2)$ | 对数增长 | 外点多 |
| Tukey | 大 $s$ 时封顶 | 常数（忽略外点） | 外点极多 |

```cpp
problem.AddResidualBlock(cost_function,
    new ceres::HuberLoss(1.0),  // δ=1.0：残差>1.0时降权
    params);
```

$\delta$ 设为正常残差的 1-2 倍。视觉重投影正常约 1-2 像素，所以 `HuberLoss(1.0)` 合理。

### 24.4.5 鲁棒核函数的数学推导 ⭐⭐⭐

理解核函数的数学本质需要引入两个关键概念：**影响函数**（influence function）和**权重函数**（weight function）。这些概念来自鲁棒统计学，它们揭示了核函数如何"柔性地"处理外点。

**从代价函数到影响函数**

令 $s = \|f_i(x)\|^2$ 为残差的平方，核函数将总代价从 $\sum s_i$ 变为 $\sum \rho(s_i)$。对 $x$ 求导时，需要用到 $\rho$ 的导数：

$$\frac{\partial \rho(s)}{\partial x} = \rho'(s) \cdot \frac{\partial s}{\partial x} = \rho'(s) \cdot 2 f^T J$$

其中 $\rho'(s)$ 就是**影响函数** $\psi(s)$。它决定了每个残差对梯度的"贡献权重"。

**Huber 核函数推导**

Huber 核函数的定义（注意 Ceres 中 $s = \|f\|^2$，是残差的**平方**；下式中的 $\delta$ 即代码里 `HuberLoss(delta)` 的构造参数，判别阈值是 $\sqrt{s}=\delta$，对应 $s=\delta^2$）：

$$\rho_{\text{Huber}}(s) = \begin{cases} s & \text{if } s \leq \delta^2 \\ 2\delta\sqrt{s} - \delta^2 & \text{if } s > \delta^2 \end{cases}$$

对 $s$ 求导得影响函数：

$$\psi_{\text{Huber}}(s) = \rho'(s) = \begin{cases} 1 & \text{if } s \leq \delta^2 \\ \delta / \sqrt{s} & \text{if } s > \delta^2 \end{cases}$$

**物理含义**：当残差较小（$\sqrt{s} \leq \delta$）时，影响函数为 1，等同于普通最小二乘。当残差较大时，影响函数为 $\delta/\sqrt{s}$，随残差增大而衰减——大残差的影响力被"压缩"。

**Cauchy 核函数推导**

$$\rho_{\text{Cauchy}}(s) = \delta^2 \log\left(1 + \frac{s}{\delta^2}\right)$$

对 $s$ 求导：

$$\psi_{\text{Cauchy}}(s) = \frac{\delta^2}{\delta^2 + s} = \frac{1}{1 + s/\delta^2}$$

**物理含义**：影响函数是一个 Lorentzian 形状的衰减。当 $s \gg \delta^2$ 时，$\psi \approx \delta^2/s \to 0$。Cauchy 核比 Huber 核对外点的降权更剧烈（$1/s$ vs $1/\sqrt{s}$）。

**权重函数与 IRLS（迭代重加权最小二乘）**

定义权重函数 $w(s) = \rho'(s) / s$（当 $s > 0$ 时），则带核函数的优化等价于：

$$\min_x \sum_i w(s_i) \cdot s_i = \min_x \sum_i w(\|f_i\|^2) \cdot \|f_i\|^2$$

这就是一个**加权最小二乘**问题！权重 $w$ 依赖于当前残差，所以需要迭代求解——这就是 **IRLS**（Iteratively Reweighted Least Squares）。

| 核函数 | $\rho'(s)$（影响函数） | $w(s) = \rho'(s)/s$（权重函数） |
|--------|----------------------|-------------------------------|
| 平方 | $1$ | $1/s$（不降权） |
| Huber | $\min(1, \delta/\sqrt{s})$ | $\min(1/s, \delta/s^{3/2})$ |
| Cauchy | $1/(1 + s/\delta^2)$ | $1/(s + s^2/\delta^2)$ |

**Ceres 内部的处理**

Ceres 并不直接实现 IRLS。它将核函数的效果嵌入到法方程的构建中。具体地，Ceres 通过将残差 $f$ 乘以 $\sqrt{\rho'(s)}$ 来修正 Jacobian 和残差，使得修正后的法方程等价于带核函数的优化。这个技巧在 `corrector.cc` 中实现。

> ⚠️ **概念误区：认为 Cauchy 核总是比 Huber 核更好**
>
> **新手想法**："Cauchy 降权更狠，所以总是更鲁棒。"
>
> **实际上**：Cauchy 的代价函数是有界的（从下方 bounded away from convexity），这意味着优化问题可能有更多局部最小值。Huber 核保持了凸性（对大残差是线性），所以优化景观更"友好"。实际选择取决于外点比例——外点少用 Huber（更稳定的收敛），外点多用 Cauchy（更强的抑制力）。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：$\delta$ 设得太小**
>
> **错误做法**：`HuberLoss(0.01)` 用于像素级残差
>
> **现象**：几乎所有观测被降权，优化等于没有观测。
>
> **正确做法**：先不带 LossFunction 跑一次，统计残差分布，再设 $\delta$。

> 💡 **概念误区：认为核函数能"删除"外点**
>
> **实际上**：核函数降权但不删除。彻底排除外点用两阶段策略：先带核函数优化，根据残差剔除外点，再无核函数精优化。

### 练习

1. **实验题**：在曲线拟合中加入外点，对比无核、Huber、Cauchy 的拟合结果。

2. **思考题**：从代价函数增长速度分析 Cauchy 为什么比 Huber 更鲁棒。

3. **手推题**：推导 Tukey Bisquare 核函数 $\rho(s) = \frac{\delta^2}{6}[1 - (1 - s/\delta^2)^3]$（$s \leq \delta^2$）的影响函数和权重函数，画出它们的图像。

---

## 24.5 Manifold 接口 ⭐⭐⭐

### 动机：四元数 4 参数只有 3 自由度

四元数约束 $\|q\| = 1$——生活在 3 维超球面上。直接对 4 参数无约束优化，更新后 $q + \Delta q$ 不满足单位约束。

### Ceres Manifold 接口 ⭐⭐⭐

Ceres 2.1 引入 `Manifold` 接口（Ceres 2.2 移除旧的 `LocalParameterization`）。定义：

| 方法 | 作用 | 四元数示例 |
|------|------|----------|
| `AmbientSize()` | 参数维度 | 4 |
| `TangentSize()` | 自由度 | 3 |
| `Plus(x, delta, x_plus_delta)` | $x \oplus \delta$ | $q \cdot \text{Exp}(\delta)$ |
| `Minus(y, x, y_minus_x)` | $y \ominus x$ | $\text{Log}(x^{-1} \cdot y)$ |

与 通用库·李群manif manif 衔接：

```cpp
// Ceres 2.2+ 使用 AutoDiffManifold 包装 manif
problem.SetManifold(
    pose.data(),
    new ceres::AutoDiffManifold<
        manif::CeresManifoldFunctor<manif::SE3d>, 7, 6>());
```

**内置 Manifold**：`EigenQuaternionManifold`（xyzw 顺序）、`QuaternionManifold`（wxyz 顺序）、`SphereManifold<N>`、`ProductManifold`。

### 24.5.5 Manifold 迁移指南：Ceres 1.x → 2.x ⭐⭐

**背景：为什么要迁移？**

Ceres 2.1 将 `LocalParameterization` 标记为 deprecated，Ceres 2.2 彻底移除。如果你维护的代码基于旧版 Ceres（VINS-Mono/Fusion 原始代码用的就是 `LocalParameterization`），升级 Ceres 版本时必须迁移。

**核心区别**

旧的 `LocalParameterization` 只需要定义 `Plus` 和 `ComputeJacobian`（Plus 操作的 Jacobian）。新的 `Manifold` 额外要求 `Minus` 和 `MinusJacobian`。增加 `Minus` 的动机是：某些算法（如 Trust Region 的子空间方法）需要计算两点之间的"距离"，而不仅仅是"加上增量"。

| 旧 API（Ceres 1.x） | 新 API（Ceres 2.x） |
|---------------------|---------------------|
| `LocalParameterization` | `Manifold` |
| `Plus(x, delta, x_plus_delta)` | `Plus(x, delta, x_plus_delta)` |
| `ComputeJacobian(x, jacobian)` | `PlusJacobian(x, jacobian)` |
| — | `Minus(y, x, y_minus_x)` |
| — | `MinusJacobian(x, jacobian)` |
| `GlobalSize()` | `AmbientSize()` |
| `LocalSize()` | `TangentSize()` |
| `problem.SetParameterization()` | `problem.SetManifold()` |

**内置类迁移对照表**

| 旧类名 | 新类名 |
|--------|--------|
| `QuaternionParameterization` | `QuaternionManifold` |
| `EigenQuaternionParameterization` | `EigenQuaternionManifold` |
| `SubsetParameterization` | `SubsetManifold` |
| `ProductParameterization` | `ProductManifold` |
| `AutoDiffLocalParameterization` | `AutoDiffManifold` |

**迁移示例：自定义 SE3 参数化**

旧版代码：

```cpp
// Ceres 1.x: 继承 LocalParameterization
class SE3Parameterization : public ceres::LocalParameterization {
public:
    bool Plus(const double* x, const double* delta,
              double* x_plus_delta) const override {
        // ... Plus 实现 ...
    }
    bool ComputeJacobian(const double* x, double* jacobian) const override {
        // ... Jacobian 实现 ...
    }
    int GlobalSize() const override { return 7; }
    int LocalSize() const override { return 6; }
};
// 使用
problem.SetParameterization(pose, new SE3Parameterization());
```

新版代码：

```cpp
// Ceres 2.x: 继承 Manifold
class SE3Manifold : public ceres::Manifold {
public:
    bool Plus(const double* x, const double* delta,
              double* x_plus_delta) const override { /* 同上 */ }
    bool PlusJacobian(const double* x, double* jacobian) const override { /* 同上 */ }
    bool Minus(const double* y, const double* x,
               double* y_minus_x) const override {
        // 新增：计算 Log(x^{-1} * y)
    }
    bool MinusJacobian(const double* x, double* jacobian) const override {
        // 新增：Minus 的 Jacobian
    }
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }
};
// 使用
problem.SetManifold(pose, new SE3Manifold());
```

> ⚠️ **编程陷阱：迁移时只改了类名没实现 Minus**
>
> **错误做法**：从 `LocalParameterization` 改为 `Manifold`，但 `Minus` 返回全零。
>
> **现象**：使用 Dogleg 策略时不收敛（Dogleg 需要 Minus），LM 可能碰巧正常（LM 不依赖 Minus）。
>
> **正确做法**：`Minus` 必须正确实现为 `Plus` 的逆操作。对于 SE(3)，`Minus(y, x)` 应返回 $\text{Log}(x^{-1} \cdot y)$ 的 6 维李代数向量。

> ⚠️ **编程陷阱：Ceres 2.2 中混用旧 API**
>
> **错误做法**：`#include <ceres/local_parameterization.h>`
>
> **现象**：编译失败，因为 Ceres 2.2 已完全移除该头文件。
>
> **正确做法**：只使用 `#include <ceres/manifold.h>`。如果需要渐进迁移，先升级到 Ceres 2.1（两套 API 共存），验证后再升到 2.2。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：用 `QuaternionManifold` 处理 Eigen 四元数**
>
> **错误做法**：`problem.SetManifold(q.coeffs().data(), new ceres::QuaternionManifold());`
>
> **现象**：旋转结果完全错误。
>
> **根本原因**：Eigen `coeffs()` 返回 [x,y,z,w]，`QuaternionManifold` 期望 [w,x,y,z]。
>
> **正确做法**：Eigen 四元数用 `EigenQuaternionManifold`。

> 💡 **概念误区：认为不用 Manifold "优化完归一化就行"**
>
> **问题**：(1) 中间步骤的未归一化四元数不是有效旋转；(2) $J^T J$ 维度变大（4 vs 3），浪费计算；(3) 秩亏导致数值不稳定。
>
> **正确理解**：Manifold 在每步保证参数在流形上，切空间中计算正确维度的增量。

### 练习

1. **实践题**：用 Ceres + `EigenQuaternionManifold` 优化一个旋转估计问题。

2. **对比题**：4 参数无约束 + 后归一化 vs Manifold，对比迭代次数和精度。

3. **迁移题**：找到 VINS-Fusion 仓库中的 `PoseLocalParameterization`，将其迁移为 Ceres 2.2 的 `Manifold` 接口。验证 `Plus` 和 `Minus` 互为逆操作。

---

## 24.6 Solver 选项与性能调优 ⭐⭐⭐

### 动机：不同配置性能差 100 倍

1000 帧 BA 问题，默认配置 60 秒，调优后 0.5 秒。差异来自线性求解器选择和稀疏结构利用。

### 线性求解器选择 ⭐⭐⭐

| 求解器 | 适用场景 | Ceres 常量 |
|--------|---------|-----------|
| Dense QR | 变量 < 100 | `DENSE_QR` |
| Dense Schur | BA（少相机） | `DENSE_SCHUR` |
| Sparse Schur | BA（多相机） | `SPARSE_SCHUR` |
| Sparse Normal Cholesky | 位姿图 | `SPARSE_NORMAL_CHOLESKY` |
| Iterative Schur | 超大 BA | `ITERATIVE_SCHUR` |

### Schur 补在 BA 中的作用 ⭐⭐⭐

BA 变量分"相机"（少量）和"3D 点"（大量）。$J^T J$ 有箭头稀疏结构。Schur 补先消元 3D 点（它们之间无直接耦合），得到只关于相机的缩减方程——维度小几个数量级。

### Trust Region 策略与 Dogleg ⭐⭐⭐

Ceres 支持两种 Trust Region 策略：

```cpp
// 默认：Levenberg-Marquardt
options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

// 替代：Dogleg（兼容直接法求解器如 DENSE_QR/SPARSE_SCHUR/SPARSE_NORMAL_CHOLESKY，
//              不兼容迭代法求解器如 ITERATIVE_SCHUR/CGNR，因为 Dogleg 需要精确 GN 步）
options.trust_region_strategy_type = ceres::DOGLEG;
options.dogleg_type = ceres::TRADITIONAL_DOGLEG;  // 或 SUBSPACE_DOGLEG
```

**何时考虑 Dogleg**：当每次解线性方程的开销远大于信赖域调整的开销时（大型密集问题），Dogleg 可能更快。但对于 SLAM 中的稀疏问题，LM 通常表现更好，因为稀疏求解已经很快，LM 的自适应阻尼能更精细地控制步长。

### 收敛判据详解 ⭐⭐

Ceres 使用三个容忍度来判断收敛，理解它们有助于诊断优化问题：

| 参数 | 含义 | 默认值 | 收敛条件 |
|------|------|--------|---------|
| `function_tolerance` | 目标函数相对下降 | $10^{-6}$ | $\|F_{k+1} - F_k\| / F_k < \text{tol}$ |
| `gradient_tolerance` | 梯度最大分量 | $10^{-10}$ | $\|g\|_\infty < \text{tol}$ |
| `parameter_tolerance` | 参数相对变化 | $10^{-8}$ | $\|\Delta x\| / (\|x\| + \text{tol}) < \text{tol}$ |

**实用指导**：

- **视觉 SLAM 实时系统**：可以放松容忍度（如 `function_tolerance = 1e-4`）换取更快收敛，因为前端提供了较好初始值，不需要精确到机器精度。

- **离线 SfM（COLMAP）**：使用默认或更紧的容忍度，追求最高精度。COLMAP 甚至将 `function_tolerance` 设为 0，完全依赖 `max_num_iterations` 来控制。

- **收敛慢的诊断**：如果 `termination_type` 是 `NO_CONVERGENCE`（达到最大迭代但未收敛），检查：
  1. 初始值是否太差？（SLAM 的好初始值来自前端）
  2. 残差中是否有 NaN/Inf？（检查 `summary.num_residuals_reduced`）
  3. 是否缺少 Manifold？（过参数化导致 $J^T J$ 秩亏）

```cpp
// 实时系统的宽松配置
options.function_tolerance = 1e-4;
options.gradient_tolerance = 1e-6;
options.parameter_tolerance = 1e-5;
options.max_num_iterations = 20;  // 限制最大迭代

// 检查收敛状态
ceres::Solve(options, &problem, &summary);
if (summary.termination_type != ceres::CONVERGENCE) {
    LOG(WARNING) << "Optimization did not converge: "
                 << summary.message;
}
```

### 实用配置模板

```cpp
// 位姿图
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
options.max_num_iterations = 100;

// 大规模 BA
options.linear_solver_type = ceres::SPARSE_SCHUR;
options.max_num_iterations = 50;
options.num_threads = std::thread::hardware_concurrency();

// 超大 BA（COLMAP 级别）
options.linear_solver_type = ceres::ITERATIVE_SCHUR;
options.preconditioner_type = ceres::SCHUR_JACOBI;
```

### 24.6.5 协方差估计 ⭐⭐⭐

**动机：优化结果有多可信？**

优化给出了位姿和地图的最优估计，但没告诉你这个估计有多"确定"。在实际系统中，下游模块需要不确定性信息：

- **路径规划**：位姿不确定性大 → 保守规划，留更大安全裕量
- **回环检测**：位姿协方差大 → 更积极搜索回环来减小不确定性
- **传感器融合**：协方差矩阵是卡尔曼滤波器的核心输入
- **主动感知**：选择下一步动作时，考虑哪个动作能最大程度减小不确定性

**数学基础**

对于非线性最小二乘问题 $\min_x \frac{1}{2}\|f(x)\|^2$，最优解 $x^*$ 处的协方差矩阵近似为：

$$\Sigma = \sigma^2 (J^T J)^{-1}$$

其中 $\sigma^2 = \frac{\|f(x^*)\|^2}{m - n}$ 是残差方差的无偏估计（$m$ 个残差，$n$ 个参数），$J$ 是最优解处的 Jacobian。$(J^T J)^{-1}$ 就是信息矩阵的逆。

**为什么不直接求 $(J^T J)^{-1}$？** 因为 $J^T J$ 可能非常大（SLAM 中有数万参数），完整求逆需要 $O(n^3)$ 时间和 $O(n^2)$ 存储。但通常我们只需要**部分**协方差——比如只关心最后几帧位姿的不确定性，而不关心所有地图点的协方差。

**Ceres Covariance API**

```cpp
#include <ceres/covariance.h>

// 1. 先完成优化
ceres::Solve(options, &problem, &summary);

// 2. 配置协方差计算
ceres::Covariance::Options cov_options;
cov_options.algorithm_type = ceres::SPARSE_QR;  // 推荐
// cov_options.algorithm_type = ceres::DENSE_SVD;  // 小问题可用

ceres::Covariance covariance(cov_options);

// 3. 指定需要的协方差块
std::vector<std::pair<const double*, const double*>> covariance_blocks;
// 位姿 i 自身的协方差（对角块）
covariance_blocks.push_back(std::make_pair(pose_i, pose_i));
// 位姿 i 和位姿 j 之间的互协方差（非对角块）
covariance_blocks.push_back(std::make_pair(pose_i, pose_j));

// 4. 计算
bool success = covariance.Compute(covariance_blocks, &problem);

// 5. 提取结果
Eigen::Matrix<double, 7, 7, Eigen::RowMajor> cov_ii;
covariance.GetCovarianceBlock(pose_i, pose_i, cov_ii.data());

Eigen::Matrix<double, 7, 7, Eigen::RowMajor> cov_ij;
covariance.GetCovarianceBlock(pose_i, pose_j, cov_ij.data());
```

**注意**：如果使用了 `Manifold`，`GetCovarianceBlock()` 返回的协方差矩阵维度是 **ambient size**（参数维度）。对于四元数参数块（ambient=4），协方差是 4×4，但只有 3×3 有效。Ceres 提供了更方便的替代方案：

```cpp
// 推荐：直接获取切空间协方差（Ceres 2.1+）
Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov_tangent;
covariance.GetCovarianceBlockInTangentSpace(pose_i, pose_i, cov_tangent.data());
// 返回 tangent_size × tangent_size 矩阵（SE3 为 6×6），无需手动投影
```

如果需要手动投影（旧代码兼容），公式为：

$$\Sigma_{\text{tangent}} = J_{\text{minus}} \cdot \Sigma_{\text{ambient}} \cdot J_{\text{minus}}^T$$

> ⚠️ **编程陷阱：对大规模问题计算完整协方差**
>
> **错误做法**：`covariance_blocks.push_back({all_params, all_params});`
>
> **现象**：内存溢出或计算时间数小时。
>
> **正确做法**：只计算需要的协方差块。通常只需要最近几帧位姿的协方差，不需要所有地图点。

> ⚠️ **编程陷阱：在协方差计算中忽略核函数的影响**
>
> **现象**：如果优化时使用了 `LossFunction`，Ceres 协方差估计会考虑核函数的影响。但如果核函数将某些残差权重降到接近零，协方差估计可能不准确。
>
> **正确做法**：先带核函数优化（处理外点），然后剔除外点，最后不带核函数重新优化并计算协方差。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：BA 问题用 `SPARSE_NORMAL_CHOLESKY`**
>
> **现象**：BA 极慢——数分钟而非数秒。
>
> **根本原因**：未利用 Schur 补消元 3D 点。`SPARSE_SCHUR` 对 BA 快数十到数百倍。

> 💡 **概念误区：认为更多迭代总更好**
>
> **实际上**：良好初始化的 SLAM 问题 20-50 次迭代就收敛。关注 `termination_type`——如果已是 `CONVERGENCE`，增加迭代无意义。

### 练习

1. **实验题**：用 BAL 数据集，对比 `DENSE_SCHUR`、`SPARSE_SCHUR`、`SPARSE_NORMAL_CHOLESKY` 的求解时间。

2. **思考题**：位姿图为什么不需要 Schur 补？

3. **实践题**：在位姿图优化后，用 `ceres::Covariance` 提取最后一帧位姿的协方差。将 3σ 椭圆画在 2D 轨迹上，观察不确定性如何随距离原点增大而增长。

---

## 24.7 SLAM 实战 ⭐⭐⭐

### VINS-Mono：Ceres 的深度应用

VINS-Mono（Qin et al., 2018, T-RO）使用三种自定义代价函数：

| 因子 | 残差维度 | 实现方式 |
|------|---------|---------|
| 重投影 | 2 | `SizedCostFunction` |
| IMU 预积分 | 15 | `SizedCostFunction` |
| 边缘化先验 | 变维 | `CostFunction` |

VINS-Mono 选择手写 Jacobian（解析 Jacobian 比 AutoDiff 快 3-5 倍，IMU 因子每次优化调用数百次）。

### ORB-SLAM3 为什么不用 Ceres？

| 特性 | Ceres | g2o |
|------|-------|-----|
| 微分方式 | AutoDiff（默认） | 必须手写 Jacobian |
| 问题定义 | 通用残差块 | 图结构（顶点+边） |
| 学习曲线 | 较低 | 较高 |
| 性能 | 大规模 BA 更快 | 小规模位姿图轻量 |
| 维护 | Google 维护 | 社区维护 |

ORB-SLAM3 用 g2o 主要是历史原因。**新项目建议用 Ceres**。

### 位姿图优化示例 ⭐⭐⭐

```cpp
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct PoseGraphError {
    Eigen::Vector3d t_meas;
    Eigen::Quaterniond q_meas;

    PoseGraphError(const Eigen::Vector3d& t, const Eigen::Quaterniond& q)
        : t_meas(t), q_meas(q) {}

    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    T* residuals) const {
        Eigen::Map<const Eigen::Quaternion<T>> q_i(pose_i);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_i(pose_i + 4);
        Eigen::Map<const Eigen::Quaternion<T>> q_j(pose_j);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_j(pose_j + 4);

        Eigen::Quaternion<T> q_ij = q_i.conjugate() * q_j;
        Eigen::Matrix<T, 3, 1> t_ij = q_i.conjugate() * (t_j - t_i);

        Eigen::Map<Eigen::Matrix<T, 3, 1>> r_t(residuals);
        r_t = t_ij - t_meas.cast<T>();

        Eigen::Quaternion<T> dq = q_meas.cast<T>().conjugate() * q_ij;
        residuals[3] = T(2.0) * dq.x();
        residuals[4] = T(2.0) * dq.y();
        residuals[5] = T(2.0) * dq.z();
        return true;
    }
};
```

使用时为每个位姿设 Manifold 并固定第一帧：

```cpp
for (auto& pose : poses) {
    problem.SetManifold(pose.data(),
        new ceres::ProductManifold<ceres::EigenQuaternionManifold,
                                   ceres::EuclideanManifold<3>>());
}
problem.SetParameterBlockConstant(poses[0].data());
```

### 24.7.5 Cartographer 的 Ceres 用法 ⭐⭐⭐

**Google Cartographer 架构概述**

Cartographer（Hess et al., ICRA 2016）是 Google 开源的 2D/3D SLAM 系统，其后端优化完全基于 Ceres。与 VINS-Mono 不同，Cartographer 主要处理激光雷达数据，并将 Ceres 用于两个层级的优化。

**Local SLAM：Ceres 扫描匹配**

在 Local SLAM 阶段，Cartographer 用 Ceres 优化当前激光帧的位姿，使其与已有子图（submap）最佳对齐。核心代价函数有两类：

1. **占据概率匹配代价**：将激光点投影到子图的占据栅格上，残差为 $1 - M(\text{proj}(p_i, T))$，其中 $M$ 是子图在该网格位置的占据概率，$T$ 是待优化的位姿。概率越高（该位置越可能被占据），残差越小。

2. **平移/旋转先验代价**：约束优化结果不要偏离初始猜测太远。权重可配置（`translation_weight` 和 `rotation_weight`）。

```cpp
// Cartographer 风格的 2D 扫描匹配代价函数（简化）
struct OccupiedSpaceCostFunction2D {
    const sensor::PointCloud& point_cloud_;
    const Grid2D& grid_;

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        T cos_theta = ceres::cos(pose[2]);
        T sin_theta = ceres::sin(pose[2]);
        for (size_t i = 0; i < point_cloud_.size(); ++i) {
            // 将点变换到子图坐标系
            T x = pose[0] + cos_theta * T(point_cloud_[i].x())
                           - sin_theta * T(point_cloud_[i].y());
            T y = pose[1] + sin_theta * T(point_cloud_[i].x())
                           + cos_theta * T(point_cloud_[i].y());
            // 双线性插值获取占据概率（支持 Jet 类型）
            residual[i] = T(1.0) - interpolate(grid_, x, y);
        }
        return true;
    }
};
```

关键技术点：栅格地图的值需要通过**双线性插值**获取，因为 Jet 类型的坐标不一定落在整数格点上。Cartographer 在 `CubicInterpolation` 中实现了对 `Jet` 友好的插值。

**Global SLAM：Ceres 位姿图优化**

在全局优化阶段，Cartographer 构建一个位姿图，节点是子图位姿和激光帧位姿，边是扫描匹配约束（包括回环约束）。优化问题：

$$\min_{T_i} \sum_{(i,j)} \rho\left(\|T_i^{-1} T_j - \hat{T}_{ij}\|^2_{\Sigma_{ij}}\right)$$

Cartographer 在全局优化中使用 `HuberLoss` 处理错误的回环检测。

**Cartographer vs VINS-Mono 的 Ceres 用法对比**

| 方面 | Cartographer | VINS-Mono |
|------|-------------|-----------|
| 传感器 | 激光雷达 | 单目相机 + IMU |
| 微分方式 | AutoDiff | 手写解析 Jacobian |
| 2D/3D | 都支持 | 3D |
| 核函数 | HuberLoss | CauchyLoss（用于重投影因子），IMU/边缘化因子无核 |
| 参数化 | SE2/SE3 角度表示 | 四元数 + 平移 |
| 边缘化 | 不使用 | 核心机制 |

> ⚠️ **概念误区：认为 Cartographer 只用 Ceres 做扫描匹配**
>
> **实际上**：Cartographer 的全局位姿图优化也用 Ceres。整个系统有两级 Ceres 优化——Local（实时扫描匹配）和 Global（后台位姿图优化）。两级用不同的 Solver 配置：Local 用 `DENSE_QR`（变量少），Global 用 `SPARSE_NORMAL_CHOLESKY`（变量多但无 BA 结构）。

### 24.7.6 COLMAP 的 Ceres 用法 ⭐⭐⭐

**COLMAP 简介**

COLMAP（Schonberger & Frahm, CVPR 2016）是目前最广泛使用的离线 Structure-from-Motion 系统。它从无序图片集合中恢复相机位姿和稀疏 3D 点云，广泛用于 3D 重建、NeRF 数据准备、文化遗产数字化等场景。COLMAP 的 Bundle Adjustment 完全基于 Ceres。

**COLMAP 的 BA 特点**

与 SLAM 的在线 BA 不同，COLMAP 面对的是离线、大规模问题：

| 特性 | SLAM BA（如 VINS） | COLMAP BA |
|------|-------------------|-----------|
| 规模 | ~100 帧 | ~10000 帧 |
| 实时性 | 必须 | 不需要 |
| 3D 点数量 | ~1000 | ~100万 |
| 相机模型 | 通常固定 | 多种模型，参数可优化 |
| 求解器 | `SPARSE_SCHUR` | `ITERATIVE_SCHUR` |

**相机模型的灵活处理**

COLMAP 支持多种相机模型（针孔、径向畸变、鱼眼等），每种模型的参数数量不同。Ceres 的模板化 `AutoDiffCostFunction` 完美支持这种需求：

```cpp
// COLMAP 风格的重投影代价函数（简化）
template <typename CameraModel>
struct BundleAdjustmentCostFunction {
    Eigen::Vector2d observed_;

    template <typename T>
    bool operator()(const T* const qvec,      // 四元数 [4]
                    const T* const tvec,      // 平移 [3]
                    const T* const point3D,   // 3D 点 [3]
                    const T* const camera,    // 相机内参 [变长]
                    T* residuals) const {
        // 世界坐标 → 相机坐标
        T p_cam[3];
        ceres::QuaternionRotatePoint(qvec, point3D, p_cam);
        p_cam[0] += tvec[0];
        p_cam[1] += tvec[1];
        p_cam[2] += tvec[2];

        // 相机模型投影（不同模型有不同参数）
        T predicted_x, predicted_y;
        CameraModel::WorldToImage(camera, p_cam[0], p_cam[1], p_cam[2],
                                  &predicted_x, &predicted_y);

        residuals[0] = predicted_x - T(observed_.x());
        residuals[1] = predicted_y - T(observed_.y());
        return true;
    }
};
```

**ITERATIVE_SCHUR 求解器**

对于超大规模 BA（数千相机 + 百万 3D 点），直接求解 Schur 补方程仍然很慢。`ITERATIVE_SCHUR` 使用共轭梯度法（CG）迭代求解 Schur 补方程，配合 `SCHUR_JACOBI` 预条件子：

```cpp
// COLMAP 的 BA Solver 配置
ceres::Solver::Options options;
options.linear_solver_type = ceres::ITERATIVE_SCHUR;
options.preconditioner_type = ceres::SCHUR_JACOBI;
options.max_num_iterations = 100;
options.num_threads = std::thread::hardware_concurrency();

// COLMAP 的特殊设置
options.function_tolerance = 0.0;  // 不依赖函数值收敛
options.gradient_tolerance = 1e-10;
options.parameter_tolerance = 1e-8;
```

**BAL 数据集基准测试**

BAL（Bundle Adjustment in the Large）数据集（Agarwal et al., ECCV 2010）是测试 BA 求解器性能的标准基准。包含从 49 帧/7776 点到 13682 帧/4456117 点的不同规模问题。COLMAP 在这些数据集上的表现验证了 `ITERATIVE_SCHUR` + `SCHUR_JACOBI` 的有效性。

| BAL 问题 | 相机数 | 3D点数 | SPARSE_SCHUR | ITERATIVE_SCHUR |
|----------|--------|--------|-------------|-----------------|
| 小规模 | 49 | 7,776 | 0.1s | 0.2s |
| 中规模 | 1,778 | 993,923 | 45s | 12s |
| 大规模 | 13,682 | 4,456,117 | OOM | 180s |

**关键观察**：小问题 `SPARSE_SCHUR` 更快（没有迭代开销），大问题 `ITERATIVE_SCHUR` 显著优于直接法（内存和时间都更优）。

> ⚠️ **编程陷阱：COLMAP 中 `function_tolerance = 0` 的含义**
>
> **新手疑问**："tolerance 设为 0 不是永远不收敛吗？"
>
> **实际上**：Ceres 有多个收敛条件，满足任一即停止。`function_tolerance = 0` 意味着不使用函数值变化作为停止条件，而是完全依赖 `gradient_tolerance` 和 `max_num_iterations`。COLMAP 这样做是因为离线场景下，它宁愿多迭代几次以确保梯度足够小，也不愿因为函数值"恰好"变化很小而过早停止。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：位姿图不固定任何位姿**
>
> **现象**：轨迹形状正确但位置任意偏移（gauge freedom）。
>
> **根本原因**：相对位姿约束不约束绝对位置。$J^T J$ 秩亏 6（3 平移 + 3 旋转）。
>
> **正确做法**：`problem.SetParameterBlockConstant(poses[0].data());`

> ⚠️ **编程陷阱：位姿参数布局不匹配 Manifold**
>
> **错误做法**：参数存为 `[tx,ty,tz,qw,qx,qy,qz]` 但 Manifold 假设 `[qx,qy,qz,qw,tx,ty,tz]`
>
> **现象**：旋转和平移完全错乱。
>
> **正确做法**：确认参数内存布局和 Manifold/Map 的假设一致。Eigen 四元数 `coeffs()` 返回 [x,y,z,w]。

### 练习

1. **实践题**：实现 2D 位姿图优化（SE(2), 3 维 $[x, y, \theta]$）。10 帧环形轨迹 + 噪声 + 回环。

2. **高级题**：加入一个错误回环，对比无核函数和 `CauchyLoss` 对轨迹变形的影响。

3. **代码阅读**：找到 VINS-Mono `projection_factor.cpp`，分析残差计算和 Jacobian 填充。

4. **对比题**：下载 BAL 数据集的一个小问题（problem-49-7776），分别用 `DENSE_SCHUR`、`SPARSE_SCHUR`、`ITERATIVE_SCHUR` 求解，记录时间和最终代价。

5. **思考题**：Cartographer 的 Local SLAM 扫描匹配中，为什么需要双线性插值？如果只用最近邻取值会怎样？（提示：考虑对 Jet 类型的影响。）

---

## 24.8 残差块、所有权与 Problem 生命周期 ⭐⭐⭐

> **这一节解决什么问题**：Ceres 的 API 看起来像"把 cost function 加进 problem"，但工程上真正容易出错的是所有权和生命周期：谁拥有 `CostFunction`、谁拥有参数数组、什么时候参数地址会失效、异常和线程边界在哪里。

### 动机：优化变量不是对象，而是稳定地址

回顾 通用库·Eigen：`Eigen::Map` 只是借用一段内存；回顾 通用库·李群manif：Manifold 把一段 `double*` 解释为流形上的点。Ceres 同样如此：`Problem` 中的参数块不是 `Pose` 对象，而是一段稳定的 `double*` 地址。优化器会反复读写这些地址。

如果不理解这一点会怎样？最常见的错误是把位姿存在 `std::vector<std::array<double, 7>>` 中，添加残差后继续 `push_back`，触发 vector 重新分配。Ceres 仍然保存旧地址，下一次 `Solve()` 就会读写已经失效的内存。这类错误可能表现为 NaN、随机崩溃，也可能表现为轨迹突然飞走。

> **本质洞察**：Ceres 的 `Problem` 拥有的是"优化图结构"，不是业务对象。`CostFunction`、`LossFunction`、`Manifold` 默认由 `Problem` 接管所有权；参数块内存由调用者持有，且必须在 `Problem` 和 `Solve()` 生命周期内保持地址稳定。

### Ceres 对象所有权表

| 对象 | 谁创建 | 默认谁释放 | 工程边界 |
|------|--------|------------|----------|
| `CostFunction` | 用户 `new` 或工厂函数 | `Problem` 默认接管 | 不要手动 `delete` 已加入的 cost |
| `LossFunction` | 用户 `new` | `Problem` 默认接管 | 多个残差块可共享同一个 loss，但所有权策略要一致 |
| `Manifold` | 用户 `new` | `Problem` 默认接管 | 同一参数块只设置一个 Manifold |
| 参数数组 `double*` | 用户容器 | 用户释放 | 地址必须稳定，维度必须匹配 |
| `Solver::Summary` | 用户栈对象 | 用户释放 | 求解后读取，不跨线程写 |

Ceres 的 `Problem::Options` 可以改变所有权策略，例如 `DO_NOT_TAKE_OWNERSHIP`。但教学和大多数 SLAM 工程中建议保留默认策略，让 `Problem` 管理 cost/loss/manifold 的释放，减少泄漏风险。

### 稳定参数块写法

下面的示例展示 Mini-LIO 位姿图后端中更稳妥的参数存储方式。核心是：先准备完整容器，避免添加残差后改变参数块地址；或者使用 `std::deque`/`std::map` 等地址稳定容器。

```cpp
#include <array>
#include <deque>
#include <ceres/ceres.h>

using PoseBlock = std::array<double, 7>;  // [qx, qy, qz, qw, tx, ty, tz]

struct PoseGraphStorage {
    std::deque<PoseBlock> poses;  // deque 追加元素时已有元素地址通常保持稳定

    double* addPose(const PoseBlock& initial) {
        poses.push_back(initial);
        return poses.back().data();
    }
};

void buildPoseGraph(ceres::Problem& problem,
                    PoseGraphStorage& storage,
                    const std::vector<PoseBlock>& initial_poses) {
    for (const auto& pose : initial_poses) {
        double* ptr = storage.addPose(pose);

        // Ceres 2.x：Eigen 四元数内存顺序为 xyzw，因此使用 EigenQuaternionManifold。
        // 参数块内存由 storage 持有，Manifold 对象默认由 problem 接管。
        auto* manifold = new ceres::ProductManifold<
            ceres::EigenQuaternionManifold,
            ceres::EuclideanManifold<3>>{
                ceres::EigenQuaternionManifold{},
                ceres::EuclideanManifold<3>{}};
        problem.SetManifold(ptr, manifold);
    }
}
```

如果项目使用 `std::vector`，应在添加任何残差前 `poses.reserve(num_poses)`，并在 `Problem` 构建完成后不再改变容量。更稳妥的做法是把参数块存到对象池或 `deque` 中。

### 残差 API 的异常和线程边界

`CostFunction::Evaluate()` 和 AutoDiff 仿函数的 `operator()` 返回 `bool`。这个布尔值不是装饰，它是残差有效性的边界：

| 情况 | 推荐处理 |
|------|----------|
| 正常数值 | 填充 residual，返回 `true` |
| 输入导致数学无效，例如深度 $z \leq 0$ | 返回 `false` 或给出受控大残差，不能产生 NaN |
| 维度/指针错误 | 在构建阶段抛异常，不要等到 Evaluate |
| 运行时可恢复外点 | 用 LossFunction 或外点剔除，不在残差里随机跳过 |

Ceres 可以多线程评估残差，因此残差仿函数应当是只读的：不要在 `operator()` 中写共享计数器、追加日志、修改缓存。如果必须统计调试信息，应在单线程模式下做，或使用线程安全的外部机制。

### 残差块的最小可诊断性

每个残差块至少要能回答四个问题：

1. 它连接哪些参数块？
2. 残差维度是多少？
3. 残差单位是什么，是否已经白化？
4. 正常残差范围是多少，超过多少应视为外点？

这些信息应该体现在类型名、构造参数和注释中。一个叫 `Factor` 的类无法帮助你调试；一个叫 `ReprojectionResidualPixel2D` 的类能立刻说明残差单位是像素且维度为 2。

### 练习

1. **生命周期题**：用 `std::vector<PoseBlock>` 构建一个小位姿图，故意不 `reserve` 并在添加残差后继续 `push_back`。用 AddressSanitizer 观察问题，然后改为 `deque` 或提前 `reserve`。
2. **所有权题**：阅读 Ceres `Problem::Options` 中 cost/loss/manifold ownership 的默认值，解释为什么同一个 `LossFunction*` 被多个 `Problem` 共享会有释放风险。
3. **线程题**：在 AutoDiff 仿函数中加入一个非原子计数器，设置 `options.num_threads=8`，观察计数是否稳定。改为无副作用仿函数后再次验证。

---

## 24.9 AutoDiff、Loss 与 Manifold 的联调验证 ⭐⭐⭐

> **这一节解决什么问题**：自动微分、鲁棒核函数和 Manifold 单独看都容易理解，组合起来却很容易出错。本节给出从残差值、Jacobian、核函数尺度到切空间维度的验证流程。

### 动机：优化失败前先验证残差

Ceres 调参的常见误区是：一看到不收敛就换求解器、调 `max_num_iterations`、换核函数。正确顺序相反：先确认残差函数本身在当前参数处输出有限值，Jacobian 非零，残差单位和 loss 尺度匹配，Manifold 的 tangent 维度正确。只有这些都通过，求解器选型才有意义。

如果不这样做会怎样？一个 `std::sin` 用在 `Jet` 上导致 Jacobian 为零，你可能会误以为 LM 阻尼太大；一个像素残差没有除以标准差，`HuberLoss(1.0)` 会把所有观测当外点；一个四元数参数块没设 Manifold，法方程会出现规范自由度。

### 残差白化与核函数尺度

Ceres 的目标函数实际是：

$$\min_x \frac{1}{2}\sum_i \rho_i\left(\|r_i(x)\|^2\right)
$$

工程上更常用的是白化残差：

$$r_i(x) = \Sigma_i^{-1/2}\left(z_i - h_i(x)\right)
$$

这样不同单位的残差会进入同一尺度：像素误差除以像素标准差，IMU 残差乘以预积分协方差的平方根信息矩阵，位姿图残差乘以测量信息矩阵的 Cholesky 因子。核函数的 $\delta$ 应该作用在白化后的残差上，而不是原始单位上。

| 残差类型 | 原始单位 | 白化方式 | 常见核函数尺度 |
|----------|----------|----------|----------------|
| 重投影 | 像素 | 除以像素噪声 $\sigma_{px}$ | $\delta=1\sim2$ |
| 点到面 ICP | 米 | 除以点面噪声 $\sigma_m$ | $\delta=1\sim3$ |
| IMU 预积分 | 混合单位 | 乘以 $\Sigma^{-1/2}$ | 通常不用核 |
| 位姿图 | 米 + 弧度 | 乘以信息矩阵平方根 | $\delta=1\sim5$ |

> **本质洞察**：LossFunction 不是"外点开关"，而是对白化残差的影响函数。残差没有白化时，核函数尺度没有可比意义。

### 使用 Problem::Evaluate 做残差体检

`Problem::Evaluate()` 可以在不运行优化的情况下计算当前 cost、residual、gradient 和 Jacobian。它是 Ceres 调试中最应该先用的工具。

```cpp
#include <ceres/ceres.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

void inspectProblem(ceres::Problem& problem) {
    ceres::Problem::EvaluateOptions eval_options;
    eval_options.apply_loss_function = false;  // 先看原始残差，避免核函数掩盖问题

    double cost = 0.0;
    std::vector<double> residuals;
    std::vector<double> gradient;
    ceres::CRSMatrix jacobian;

    const bool ok = problem.Evaluate(eval_options, &cost, &residuals,
                                     &gradient, &jacobian);
    if (!ok) {
        throw std::runtime_error("Ceres Problem::Evaluate failed");
    }

    double max_abs_residual = 0.0;
    for (double r : residuals) {
        if (!std::isfinite(r)) {
            throw std::runtime_error("Residual contains NaN or Inf");
        }
        max_abs_residual = std::max(max_abs_residual, std::abs(r));
    }

    std::cout << "cost=" << cost
              << " residuals=" << residuals.size()
              << " jacobian_rows=" << jacobian.num_rows
              << " jacobian_cols=" << jacobian.num_cols
              << " max_abs_residual=" << max_abs_residual << "\n";
}
```

这段代码的边界很清楚：只读 `Problem`，不改变参数；`apply_loss_function=false` 用于观察原始残差；真正求解前再打开 loss。若 `jacobian_cols` 比预期大，通常说明四元数没有设置 Manifold，ambient 维度进入了线性系统。

### AutoDiff 与有限差分的对照

自动微分本身通常可靠，问题多半来自仿函数写法：类型被转成 `double`、分支不连续、调用不支持 Jet 的函数、Eigen 映射布局错误。验证方式是：在小数据上用数值差分或 Ceres `GradientChecker` 对照 AutoDiff。

| 现象 | 可能根因 | 验证方法 |
|------|----------|----------|
| Jacobian 全零 | 仿函数中把 `T` 转成 `double` | 搜索 `(double)`、`.a`、`std::` 数学函数 |
| Jacobian 某几列错 | 参数块布局或 `Map` offset 错 | 单独扰动每个参数，打印数值差分列 |
| 接近阈值时不稳定 | `if` 分支导致不可导 | 用连续近似或把外点逻辑放到 LossFunction |
| Manifold 后维度不对 | 未设置或设置了错误 Manifold | 比较 ambient/tangent size |

### Manifold 联调顺序

1. 单独测试 `Plus`/`Minus` 互逆。
2. 用单位扰动验证 tangent 维度，例如 SE(3) 应为 6。
3. 在残差为零的合成数据上运行一次 `Problem::Evaluate()`，确认 cost 接近零。
4. 加入小噪声，确认一次求解后 cost 下降。
5. 最后加入鲁棒核和真实数据。

这个顺序避免把多个不确定因素叠在一起。Manifold 错、残差错、loss 尺度错和求解器选错会产生相似症状，必须逐层排除。

### 练习

1. **残差体检题**：对本章位姿图示例添加 `inspectProblem()`，分别在优化前、优化后打印 cost、最大残差和 Jacobian 尺寸。
2. **核函数尺度题**：构造像素噪声 $\sigma=2$ 的重投影残差，分别对白化前后使用 `HuberLoss(1.0)`，解释哪些观测被降权。
3. **跨章综合题**：用 通用库·李群manif 的 finite difference 方法验证 Ceres 位姿图边对两个 SE(3) 参数块的 Jacobian，并解释 ambient 维度 7 与 tangent 维度 6 的区别。

---

## 24.10 求解器选型与工程边界 ⭐⭐⭐

> **这一节解决什么问题**：Ceres 的 Solver 选项很多，但 SLAM 选型并不是玄学。核心原则是先识别问题结构，再选择线性求解器、信赖域策略、线程数、迭代预算和终止条件。

### 选型从问题结构开始

| 问题 | 结构 | 首选线性求解器 | 典型配置 |
|------|------|----------------|----------|
| 2D 扫描匹配 | 变量 3 维，残差较多 | `DENSE_QR` | 小变量实时优化 |
| 位姿图 | 位姿-位姿边，无 landmark | `SPARSE_NORMAL_CHOLESKY` | 固定首帧消除规范自由度 |
| 小/中规模 BA | 相机 + 3D 点，Schur 结构 | `SPARSE_SCHUR` | 直接法，精度高 |
| 超大规模 BA | 相机/点极多，内存敏感 | `ITERATIVE_SCHUR` | `SCHUR_JACOBI` 预条件 |
| 黑盒残差 | 无法自动微分 | `DENSE_QR` 或稀疏法 | 先用数值微分验证，不追求最终性能 |

Schur 补不是"更高级的 Cholesky"，它只适用于有二分块结构的问题。位姿图中没有大量可独立消元的 landmark，强行用 Schur 不会带来 BA 那样的收益。

### 实时系统的边界

在线 SLAM 的后端优化必须服从时间预算。一个实用配置通常包含：

```cpp
ceres::Solver::Options options;
options.minimizer_type = ceres::TRUST_REGION;
options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
options.max_num_iterations = 20;
options.function_tolerance = 1e-4;
options.gradient_tolerance = 1e-6;
options.parameter_tolerance = 1e-5;
options.num_threads = 4;  // 根据前端线程和 CPU 核数预留余量
```

这里的关键不是某个数值固定不变，而是权衡：在线系统宁愿每次优化达到"足够好"，也不要为最后 1% 的 cost 下降阻塞前端。离线 SfM 则相反，可以用更多迭代和更严格容忍度换精度。

### 工程边界：规范自由度、尺度与参数锁定

| 边界 | 现象 | 处理方式 |
|------|------|----------|
| 位姿图整体平移/旋转不可观 | Hessian 秩亏，轨迹形状对但漂在任意位置 | 固定第一帧或加先验 |
| 单目 BA 尺度不可观 | 轨迹可任意缩放 | 固定尺度或加入 IMU/深度/先验 |
| 部分相机内参不应优化 | 内参漂移吸收位姿误差 | `SetParameterBlockConstant` 或 `SubsetManifold` |
| 外点过多 | 鲁棒核仍无法恢复 | 先 RANSAC/几何验证，再优化 |
| 初始值太差 | LM 进入错误局部极小 | 粗配准、PnP、回环几何验证先提供初值 |

> **本质洞察**：求解器不能创造可观测性。没有绝对约束的自由度必须通过固定变量、先验或传感器信息补上；否则再强的线性求解器也只是在秩亏空间里漂移。

### 练习

1. **选型题**：给出三个问题：局部扫描匹配、1000 帧位姿图、500 相机 + 10 万点 BA。分别选择 Ceres 线性求解器并说明理由。
2. **规范自由度题**：在位姿图示例中去掉首帧固定，观察 `summary.FullReport()` 中的线性求解信息和最终轨迹。再加回先验因子对比。
3. **实时预算题**：设置 `max_num_iterations` 为 5、20、100，记录每次耗时和最终 cost。解释哪个配置更适合 10 Hz 后端。

---

## 24.11 Ceres 与 GTSAM 的对比 ⭐⭐⭐

### 这一节解决什么问题

SLAM 后端优化有两个主流框架：Ceres 和 GTSAM。它们的设计哲学截然不同——Ceres 是通用非线性最小二乘求解器，GTSAM 是专为因子图优化设计的框架。理解两者的差异有助于在项目中做出正确的技术选型。

### 设计哲学对比

| 维度 | Ceres | GTSAM |
|------|-------|-------|
| **定位** | 通用非线性最小二乘 | 因子图推理 |
| **核心抽象** | Problem + CostFunction | Factor Graph + Values |
| **变量类型** | 原始 `double*` 数组 | 类型化的 Key-Value |
| **流形支持** | Manifold 接口（2.x） | 内置 traits 系统 |
| **微分** | AutoDiff / 手写 / 数值 | 表达式模板 / 手写 |
| **增量求解** | 不支持（每次从头求解） | iSAM2（增量式） |
| **边缘化** | 需手动实现 | 内置 BayesTree |
| **维护方** | Google | Georgia Tech / Borglab |

**Ceres 的优势**在于通用性和工程成熟度。它不限于 SLAM——任何最小二乘问题（曲线拟合、相机标定、Bundle Adjustment）都可以用 Ceres 解决。AutoDiff 让你几乎不需要推导 Jacobian。Google 的持续维护保证了代码质量和跨平台兼容性。

**GTSAM 的优势**在于 SLAM 特化。iSAM2 的增量求解让在线 SLAM 不需要每次优化整个图——只更新受新观测影响的部分。内置的边缘化和 BayesTree 数据结构让滑动窗口优化变得简单。GTSAM 的因子图抽象也更贴近 SLAM 的概念模型。

### 实际部署中的选型建议

| 场景 | 推荐 | 理由 |
|------|------|------|
| 离线 SfM / BA | Ceres | COLMAP 验证，大规模 Schur 补成熟 |
| 实时 VIO | 两者均可 | VINS-Mono 用 Ceres，GTSAM 用 iSAM2 |
| 实时 LIO | GTSAM 稍优 | LIO-SAM 用 GTSAM + iSAM2，增量式更高效 |
| 通用优化问题 | Ceres | 不限于 SLAM，AutoDiff 通用 |
| 需要增量更新 | GTSAM | iSAM2 是杀手级特性 |
| 需要边缘化 | GTSAM 更方便 | 内置 BayesTree 边缘化 |

> **本质洞察**：Ceres 和 GTSAM 不是同一个问题的两种解法，而是两种不同抽象层次的工具。Ceres 是"优化器"——给我残差函数和变量，我找最优解。GTSAM 是"推理引擎"——给我因子图，我做贝叶斯推理。在 SLAM 的语境下，推理和优化殊途同归（MAP 推理等价于最小二乘），但 GTSAM 的抽象更贴合 SLAM 的概率建模。

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：认为必须在 Ceres 和 GTSAM 之间二选一**
>
> **实际上**：许多系统混合使用。LIO-SAM 的后端用 GTSAM，但标定和某些离线优化用 Ceres。两者可以在同一个项目中共存——只要注意李群约定的一致性（GTSAM 默认右扰动，manif/Ceres 通常也采用右扰动，因此约定上天然兼容）。

### SLAM 后端的实际部署考量 ⭐⭐⭐

在实际机器人系统中部署 Ceres 后端，还需要考虑工程层面的问题：

**线程模型**

Ceres 的 `Solve()` 是阻塞调用——在求解完成前不会返回。在实时 SLAM 中，后端优化通常运行在独立线程中，避免阻塞前端跟踪。典型的线程模型：

```text
前端线程（高频 30Hz）      后端线程（低频 1-5Hz）
    │                          │
    ├── 提取特征                 │
    ├── 跟踪                    │
    ├── 判断是否为关键帧 ───────→ 接收关键帧
    │                          ├── 构建 Problem
    │                          ├── Solve()（阻塞 50-200ms）
    │                          └── 更新全局地图 ←─── 加锁
    └── 读取最新地图 ←──────────── 加锁
```

**内存管理**

一个长时间运行的 SLAM 系统可能累积数千个关键帧和数十万个地图点。如果每次优化都构建完整的 `Problem`，内存分配和释放的开销会显著增加。实际系统中的常见策略：

| 策略 | 做法 | 适用场景 |
|------|------|---------|
| **滑动窗口** | 只优化最近 N 帧，旧帧边缘化为先验 | VINS-Mono、OKVIS2 |
| **关键帧选取** | 只保留"信息量大"的帧参与优化 | ORB-SLAM3 |
| **分层优化** | Local BA（实时）+ Global BA（后台） | Cartographer |
| **延迟优化** | 积累观测后批量优化 | 离线建图 |

**Ceres 2.x 版本变化摘要**

Ceres 从 1.x 到 2.x 有一些需要注意的 API 变化：

| 变化 | Ceres 1.x | Ceres 2.x |
|------|-----------|-----------|
| 流形接口 | `LocalParameterization` | `Manifold`（2.1 新增，2.2 移除旧接口） |
| 命名空间 | `ceres::` | `ceres::` （不变） |
| C++ 标准 | C++14 | C++17 （2.2 起要求） |
| 最低 CMake | 3.10 | 3.16 |
| Eigen 版本 | 3.3+ | 3.4+ （2.2 起推荐） |

**从 Ceres 1.x 升级到 2.x 的检查清单**：

1. 将所有 `LocalParameterization` 替换为 `Manifold`
2. 实现 `Minus` 和 `MinusJacobian`（新增要求）
3. 将 `SetParameterization()` 改为 `SetManifold()`
4. 确认编译器支持 C++17
5. 运行完整的回归测试，验证优化结果与旧版本一致

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：Ceres 2.2 中使用已移除的 API**
>
> **错误做法**：代码中仍然使用 `ceres::LocalParameterization`
>
> **现象**：编译失败，找不到头文件或类定义
>
> **根本原因**：Ceres 2.2 完全移除了 `LocalParameterization`，不像 2.1 那样两套 API 共存
>
> **正确做法**：使用 `ceres::Manifold` 接口。如果需要渐进迁移，先升级到 Ceres 2.1（两套 API 共存），验证后再升到 2.2

### 练习

1. **对比题**：用 Ceres 和 GTSAM 分别实现同一个 2D 位姿图优化问题。比较代码行数、收敛速度和最终代价。分析两者的 API 设计对开发效率的影响。

2. **选型题**：给定以下三个项目需求，分别选择 Ceres 或 GTSAM 并说明理由：(a) 无人机实时 VIO，关键帧数 < 50；(b) 自动驾驶离线建图，关键帧数 > 10000；(c) 多传感器融合，需要滑动窗口和增量更新。

3. **迁移题**：找到一个使用 Ceres 1.x `LocalParameterization` 的开源项目（如 VINS-Mono 或 VINS-Fusion），将其迁移到 Ceres 2.2 的 `Manifold` 接口。记录遇到的问题和解决方案。

---

## 本章小结

| 知识点 | 核心要义 | 难度 |
|--------|---------|------|
| 非线性最小二乘 | SLAM 后端 = $\min \sum \|f_i(x)\|^2$, LM 求解 | ⭐⭐ |
| Trust Region vs Line Search | Trust Region + Schur 补是 SLAM 标准选择 | ⭐⭐⭐ |
| Powell's Dogleg | 折线法避免重复解线性方程 | ⭐⭐⭐ |
| Ceres 架构 | Problem → CostFunction → Solver 声明式工作流 | ⭐⭐ |
| AutoDiff / Jet | 双数自动传播导数，精确到机器精度 | ⭐⭐⭐ |
| Jet 源码 | `Jet<T,N>` 的 `a` + `v[N]` 结构，运算符重载实现链式法则 | ⭐⭐⭐ |
| DynamicAutoDiffCostFunction | 运行时确定参数维度，边缘化因子的选择 | ⭐⭐⭐ |
| LossFunction | Huber/Cauchy 降权外点，$\delta$ 设为正常残差 1-2 倍 | ⭐⭐ |
| 影响函数与 IRLS | $\psi(s) = \rho'(s)$ 控制外点贡献，等价加权最小二乘 | ⭐⭐⭐ |
| Manifold | 处理过参数化，Plus/Minus 定义流形操作 | ⭐⭐⭐ |
| Manifold 迁移 | Ceres 1.x → 2.x：新增 Minus，API 重命名 | ⭐⭐ |
| Solver 调优 | BA 用 SPARSE_SCHUR，位姿图用 SPARSE_NORMAL_CHOLESKY | ⭐⭐⭐ |
| 收敛判据 | function/gradient/parameter tolerance 三重判断 | ⭐⭐ |
| 协方差估计 | `ceres::Covariance` 提取优化后不确定性 | ⭐⭐⭐ |
| Cartographer | 双层 Ceres：Local 扫描匹配 + Global 位姿图优化 | ⭐⭐⭐ |
| COLMAP | 大规模 SfM，ITERATIVE_SCHUR + SCHUR_JACOBI | ⭐⭐⭐ |
| SLAM 实战 | VINS-Mono 三因子，Ceres vs g2o 选型 | ⭐⭐⭐ |
| 残差块生命周期 | Cost/Loss/Manifold 默认由 Problem 接管，参数内存由用户保证稳定 | ⭐⭐⭐ |
| AutoDiff 联调 | 先用 `Problem::Evaluate()` 验证残差、Jacobian、白化和 Manifold 维度 | ⭐⭐⭐ |
| 求解器工程边界 | 选型取决于结构；求解器不能修复不可观自由度和坏初值 | ⭐⭐⭐ |
| Ceres vs GTSAM | 通用最小二乘 vs 因子图推理；新项目按场景选型 | ⭐⭐⭐ |
| SLAM 部署 | 线程模型、内存管理、滑动窗口、Ceres 2.x 迁移 | ⭐⭐⭐ |

**关键记忆点**：

1. **Ceres 的核心循环是 `Problem → Solve → Summary`**——声明式定义问题，引擎负责求解
2. **AutoDiff + Jet 是默认选择**——除非性能分析证明需要手写 Jacobian，否则不要手写
3. **LossFunction 作用在白化残差上**——残差没有白化时核函数尺度没有意义
4. **Manifold 不是可选的优化**——过参数化表示（四元数、SO(3)）必须设置 Manifold，否则法方程秩亏
5. **BA 用 `SPARSE_SCHUR`，位姿图用 `SPARSE_NORMAL_CHOLESKY`**——选错求解器可能慢 100 倍
6. **`Problem::Evaluate()` 是调试第一步**——在怀疑求解器之前先检查残差和 Jacobian
7. **参数块地址必须稳定**——不要在添加残差后改变参数容器的容量
8. **Ceres 2.2 移除了 `LocalParameterization`**——迁移到 `Manifold` 需要实现 `Minus`

---

## 累积项目：本章新增模块

| 章节 | 模块 | 状态 |
|------|------|------|
| 通用库·文件IO | 文件 I/O | ✅ |
| 通用库·Eigen | Eigen 优化层 | ✅ |
| 通用库·李群manif | manif SE3d 位姿 | ✅ |
| **通用库·Ceres** | **Ceres 后端优化** | **本章新增** |

**本章任务**：用 Ceres 实现 Mini-LIO 位姿图后端——ICP 帧间位姿 → 位姿图 → Ceres 全局优化。manif + AutoDiffManifold 处理 SE(3)。

---

## 延伸阅读

| 资源 | 内容 | 难度 |
|------|------|------|
| [Ceres 官方教程](http://ceres-solver.org/tutorial.html) | 从曲线拟合到 BA | ⭐⭐ |
| [Ceres Modeling 文档](http://ceres-solver.org/nnls_modeling.html) | Problem/CostFunction/Manifold API | ⭐⭐ |
| [Ceres Solving 文档](http://ceres-solver.org/nnls_solving.html) | Solver 选项详解 | ⭐⭐⭐ |
| [Ceres Covariance 文档](http://ceres-solver.org/nnls_covariance.html) | 协方差估计 API | ⭐⭐⭐ |
| [Ceres Automatic Derivatives](http://ceres-solver.org/automatic_derivatives.html) | Jet 和 AutoDiff 详解 | ⭐⭐⭐ |
| Agarwal et al., "Bundle Adjustment in the Large" (ECCV 2010) | BA 稀疏求解，BAL 数据集 | ⭐⭐⭐ |
| Hess et al., "Real-Time Loop Closure in 2D LIDAR SLAM" (ICRA 2016) | Cartographer 论文 | ⭐⭐⭐ |
| Schonberger & Frahm, "Structure-from-Motion Revisited" (CVPR 2016) | COLMAP 论文 | ⭐⭐⭐ |
| slambook2 第 6 章（高翔） | Ceres 和 g2o 入门对比 | ⭐⭐ |
| VINS-Mono `factor/` 目录 | 工业级 Ceres 代价函数 | ⭐⭐⭐⭐ |
| [Ceres Jet 源码](https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/jet.h) | Jet 双数完整实现 | ⭐⭐⭐ |
| Nocedal & Wright, *Numerical Optimization* Ch4, C++语言核心/Lambda与STL算法 | Trust Region 和 LM 数学推导 | ⭐⭐⭐⭐ |
| [COLMAP GitHub](https://github.com/colmap/colmap) | 大规模 SfM 源码 | ⭐⭐⭐ |
| [Cartographer GitHub](https://github.com/cartographer-project/cartographer) | Google 激光 SLAM 源码 | ⭐⭐⭐ |
| Dellaert & Kaess, "Factor Graphs for Robot Perception", Foundations and Trends in Robotics, 2017 | GTSAM 理论基础，因子图推理与 Ceres 最小二乘的对比 | ⭐⭐⭐ |
| [GTSAM GitHub](https://github.com/borglab/gtsam) | 因子图优化库，与 Ceres 互补的 SLAM 后端选择 | ⭐⭐⭐ |
| Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree", IJRR 2012 | 增量式图优化，GTSAM 的核心算法，理解增量 vs 批量的差异 | ⭐⭐⭐⭐ |

---

## 知识树

本章的知识结构如下：

```text
SLAM 后端优化
├── 数学基础
│   ├── 非线性最小二乘（24.1）
│   ├── Gauss-Newton / LM / Dogleg（24.1）
│   └── Trust Region vs Line Search（24.1）
├── Ceres 工具链
│   ├── Problem / CostFunction / Solver（24.2）
│   ├── AutoDiff + Jet 双数（24.3）
│   ├── LossFunction 鲁棒核（24.4）
│   ├── Manifold 流形接口（24.5）
│   ├── Solver 调优 + 协方差（24.6）
│   └── 所有权与生命周期（24.8）
├── SLAM 实战
│   ├── VINS-Mono / Cartographer / COLMAP（24.7）
│   ├── 联调验证流程（24.9）
│   └── 求解器选型边界（24.10）
└── 生态对比
    └── Ceres vs GTSAM 选型（24.11）
```

从底层的数学原理（最小二乘、LM 算法）到中间的工具使用（AutoDiff、Manifold、Solver 配置），再到顶层的系统集成（SLAM 实战、联调验证），形成了一个完整的知识链。理解底层原理后使用工具会更有信心——知道 LM 的阻尼机制，就能理解为什么 `function_tolerance` 设为 0 在某些场景下是合理的；知道 Schur 补的数学含义，就能理解 BA 为什么一定要用 `SPARSE_SCHUR` 而不是 `SPARSE_NORMAL_CHOLESKY`。

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|----------|----------|----------|
| `Solve()` 后 cost 不下降 | Jacobian 全零、残差尺度极端、初始值太差 | 1. 用 `Problem::Evaluate()` 打印残差和 Jacobian 尺寸；2. 检查仿函数中是否把 `T` 转成 `double`；3. 用合成小数据验证 | 通用库·Eigen, 通用库·Ceres |
| 优化过程中出现 NaN | 残差中除以零、深度为负、`sqrt/log` 输入非法 | 1. 在残差函数开头检查参数 `isfinite`；2. 对无效观测返回 `false` 或剔除；3. 打印首个 NaN 残差对应观测 | 通用库·Ceres, 通用库·OpenCV |
| 位姿图轨迹形状对但整体漂移 | 未固定首帧或未加先验，存在规范自由度 | 1. 固定第一帧；2. 检查 Hessian 秩亏提示；3. 加入绝对位姿/GPS 先验 | 通用库·Ceres, SLAM库·GTSAM, SLAM库·g2o |
| 四元数优化后不再单位化 | 未设置 `EigenQuaternionManifold` 或参数布局错 | 1. 检查 `Problem` 中每个四元数参数块的 Manifold；2. 确认 xyzw/wxyz 顺序；3. 打印优化后四元数范数 | 通用库·李群manif, 通用库·Ceres |
| 加了 `HuberLoss` 后所有观测几乎无效 | 残差未白化或 $\delta$ 设得过小 | 1. 先关闭 loss 统计残差分布；2. 白化残差；3. 将 $\delta$ 设为正常残差 1-2 倍 | 通用库·Ceres |
| BA 求解极慢或内存暴涨 | 未使用 Schur 结构，或小问题误用迭代法 | 1. BA 首选 `SPARSE_SCHUR`；2. 超大问题再考虑 `ITERATIVE_SCHUR`；3. 对比 `summary.FullReport()` 的线性求解耗时 | 通用库·Ceres |
| Ceres 2.2 编译报 `LocalParameterization` 不存在 | 旧 API 已移除 | 1. 改用 `ceres::Manifold`；2. `SetParameterization` 改为 `SetManifold`；3. 实现 `Minus` 和 `MinusJacobian` | 通用库·李群manif, 通用库·Ceres |
| 优化后轨迹局部正确但整体扭曲 | 信息矩阵尺度不一致或先验权重过大 | 1. 检查所有因子的信息矩阵量纲是否一致；2. 打印先验因子和里程计因子的残差量级对比；3. 验证回环约束的权重是否合理 | 通用库·Ceres |
| `Problem::Evaluate()` 返回的 Jacobian 全为零 | AutoDiff 仿函数中对 Jet 类型做了 `double` 截断 | 1. 检查仿函数中是否有 `static_cast<double>(x)` 或 `double(x)` 调用；2. 所有计算必须使用模板类型 `T`；3. 用有限差分交叉验证 AutoDiff 结果 | 通用库·Ceres |

---

## 24.12 Ceres 的多线程求解与性能调优 ⭐⭐⭐

### 并行化的层次

Ceres Solver 内部有多个可以并行化的层次，理解这些层次有助于合理配置求解器参数。

| 层次 | 并行方式 | 控制参数 | 适用场景 |
|------|---------|---------|---------|
| 残差求值 | 多线程并行计算各残差块 | `num_threads` | 残差块数量多（>100）时有效 |
| 线性求解 | Eigen/SuiteSparse 的多线程内核 | 底层库配置 | 大规模稀疏矩阵 |
| 信赖域步计算 | 并行 Schur 补计算 | 自动 | BA 问题 |

```cpp
ceres::Solver::Options options;
options.num_threads = 4;  // 残差求值的并行线程数
options.linear_solver_type = ceres::SPARSE_SCHUR;
// Schur 补的内部并行由 SuiteSparse/Eigen 控制
```

一个常见的性能陷阱是设置过多的线程数。当残差块数量少于线程数时（如只有 50 个位姿约束），线程创建和同步的开销反而超过并行收益。经验法则是：残差块数量 / 线程数 > 50 时并行才值得。

### Jacobian 求值策略的性能对比

Ceres 提供三种 Jacobian 计算方式，性能差异显著：

| 方式 | 精度 | 速度 | 适用场景 |
|------|------|------|---------|
| AutoDiff | 机器精度 | 基线 | 默认选择 |
| NumericDiff | 取决于步长 | 2-10x 慢于 AutoDiff | 残差中调用黑盒库 |
| AnalyticDiff（手写） | 机器精度 | 可能 2-5x 快于 AutoDiff | 性能瓶颈在 Jacobian 计算 |

AutoDiff 的 Jet 类型在每次前向传播时同时计算函数值和所有偏导数。对于参数维度为 $n$ 的残差块，Jet 向量的大小为 $n$，每个算术运算的开销从 1 次浮点操作增加到 $1 + n$ 次。当 $n$ 很大（如 BA 中单个残差涉及 50+ 参数）时，AutoDiff 的开销变得显著。

如果 profiling 显示 `CostFunction::Evaluate()` 占据了求解时间的主体（>60%），且参数维度较大，可以考虑手写 Jacobian。但手写 Jacobian 极易出错——建议先用 AutoDiff 正确实现，然后在手写版本中用 `ceres::GradientChecker` 做交叉验证。

### 增量求解与 Ceres 的局限

SLAM 后端的一个核心需求是增量优化——每帧新增少量约束后快速更新解，而非从头求解整个问题。Ceres 是批量求解器，每次 `Solve()` 都重新构建法方程并从头迭代。这意味着随着位姿图增长，求解时间线性或超线性增长。

对于需要增量更新的场景（>1000 个位姿），GTSAM 的 iSAM2 是更合适的选择——它维护一棵 Bayes 树，每次只重新计算受新约束影响的变量子集。本章 24.11 节对比了 Ceres 和 GTSAM 的选型边界，这里进一步明确：

- **位姿 < 500，批量优化**：Ceres 是更简单的选择，求解时间在毫秒级
- **位姿 500-5000，滑动窗口**：Ceres 配合固定旧变量和边缘化，仍然可行
- **位姿 > 5000，全量增量优化**：GTSAM iSAM2 是更高效的选择
- **BA 问题（路标 >> 位姿）**：Ceres 的 SPARSE_SCHUR 是标准选择，利用了路标-位姿稀疏结构

> **类比**：Ceres 和 GTSAM 的关系类似于 SQLite 和 PostgreSQL。
> Ceres（SQLite）简单、嵌入式、无外部依赖，适合中小规模问题。
> GTSAM（PostgreSQL）功能更强、支持增量更新，但引入了更多概念（因子图、Bayes 树、变量消元序）。
> 选哪个取决于问题规模和是否需要增量能力，不是"哪个更好"的问题。

---

## 24.13 仿函数中的数值稳定性技巧 ⭐⭐⭐

编写 Ceres 仿函数时，数值稳定性是一个容易被忽视但影响严重的问题。AutoDiff 的 Jet 类型忠实地传播导数，也忠实地传播数值不稳定性。

### 常见的数值陷阱

**1. `acos` 和 `asin` 在边界点的导数发散**

```cpp
// ❌ 不稳定：当 dot_product 接近 1 时，acos 的导数趋向无穷
T angle = ceres::acos(dot_product);

// ✅ 稳定：用 atan2 替代 acos
T cross_norm = cross_product.norm();
T angle = ceres::atan2(cross_norm, dot_product);
```

`acos(x)` 在 $x = \pm 1$ 处的导数为 $-1/\sqrt{1-x^2} \to -\infty$。当两个向量几乎平行时，Jet 向量中的导数分量爆炸，导致 Gauss-Newton 步方向完全错误。`atan2` 在所有方向上都有有限的导数。

**2. 四元数归一化的微分**

```cpp
// ❌ 在 Manifold 外部手动归一化——破坏了流形约束
template<typename T>
bool operator()(const T* q, T* residual) const {
    T norm = ceres::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    T qn[4] = {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
    // ... 使用 qn 计算残差 ...
}

// ✅ 使用 EigenQuaternionManifold，让 Ceres 在流形上迭代
// 仿函数直接使用 q，Manifold 保证 q 始终在单位四元数流形上
```

手动归一化四元数会引入不必要的非线性（除法），且归一化操作的 Jacobian 秩亏——Ceres 的 Manifold 机制是正确的处理方式。

**3. 小角度近似的连续性**

```cpp
// 当旋转角度接近零时，Rodrigues 公式中的 sin(θ)/θ 需要特殊处理
template<typename T>
void angleAxisToRotation(const T* angle_axis, T* R) {
    T theta = ceres::sqrt(angle_axis[0]*angle_axis[0] +
                          angle_axis[1]*angle_axis[1] +
                          angle_axis[2]*angle_axis[2]);

    // Ceres 的 ceres::atan2 和条件判断对 Jet 类型安全
    // 但简单的 if (theta < eps) 分支在 theta = eps 处导数不连续
    // 推荐使用 Ceres 内置的 ceres::AngleAxisRotatePoint()
}
```

> **本质洞察**：AutoDiff 是精确的数学工具，但它无法修复数学公式本身的数值问题。
> 如果解析公式在某个点导数发散，AutoDiff 计算出的导数也会发散。
> 仿函数的数值稳定性责任在于编写者，不在于 Ceres。

### 数值稳定性检查清单

在提交仿函数之前，按以下清单检查：

1. 仿函数中是否使用了 `acos`/`asin`？如果是，输入值是否可能达到 $\pm 1$ ？
2. 是否有除法操作？除数是否可能接近零？
3. 是否有条件分支？分支边界处的函数值和导数是否连续？
4. 四元数和旋转矩阵是否通过 Manifold 处理而非手动归一化？
5. 用合成数据测试时，边界情况（零旋转、零平移、共面点）是否稳定？

---

## 24.14 Ceres 的内存管理与大规模问题 ⭐⭐⭐

### 工程问题：大规模 BA 的内存占用

在大规模 Structure-from-Motion（如 COLMAP 处理上千张图片）或大规模位姿图优化（如自动驾驶离线建图）中，Ceres 的内存占用可能成为瓶颈。理解 Ceres 内部的内存使用模式有助于做出合理的工程决策。

Ceres 的主要内存消耗来源：

| 组件 | 内存占用 | 与问题规模的关系 |
|------|---------|----------------|
| 参数块存储 | 由用户管理 | $O(n)$，$n$ = 参数数量 |
| 残差块元数据 | `Problem` 内部 | $O(m)$，$m$ = 残差块数量 |
| Jacobian 矩阵 | 稀疏或密集 | $O(\text{nnz})$，nnz = 非零元素数 |
| 法方程 $J^T J$ | 取决于线性求解器 | 稀疏：$O(\text{nnz}_H)$；密集：$O(n^2)$ |
| 工作空间 | 迭代求解器内部 | 取决于求解器类型 |

对于 BA 问题，Jacobian 的非零元素数与观测数成正比。一个有 1000 个相机和 100000 个路标的 BA 问题，Jacobian 可能有几千万个非零元素，占用数百 MB 内存。

### 减少内存占用的策略

**1. 使用 `ITERATIVE_SCHUR` 替代 `SPARSE_SCHUR`**

`SPARSE_SCHUR` 显式构建 Schur 补矩阵 $S = H_{pp} - H_{pl} H_{ll}^{-1} H_{lp}$，内存占用为 $O(p^2)$（$p$ = 位姿参数数量）。`ITERATIVE_SCHUR` 使用 Conjugate Gradient 迭代求解，不显式构建 $S$，内存占用显著减少。

```cpp
ceres::Solver::Options options;
// 大规模 BA：使用迭代 Schur 减少内存
options.linear_solver_type = ceres::ITERATIVE_SCHUR;
options.preconditioner_type = ceres::SCHUR_JACOBI;  // 推荐的预条件
options.max_num_iterations = 100;
```

**2. 参数块的内存管理**

Ceres 要求参数块的地址在整个求解过程中稳定。如果参数存储在 `std::vector` 中，`push_back` 触发的 reallocation 会使所有地址失效。

```cpp
// ❌ 危险：vector realloc 使所有参数地址失效
std::vector<double> all_params;
for (const auto& pose : poses) {
    all_params.push_back(pose.x());  // 这里可能 realloc
    // 之前添加到 Problem 的参数块地址已失效！
}

// ✅ 安全方式一：预分配
std::vector<double> all_params;
all_params.reserve(poses.size() * 7);  // 预留空间
// 后续 push_back 不会 realloc

// ✅ 安全方式二：使用 deque（不 realloc）
std::deque<std::array<double, 7>> pose_params;
for (const auto& pose : poses) {
    pose_params.push_back({...});
    problem.AddParameterBlock(pose_params.back().data(), 7);
}
```

### Ceres 与 GPU 加速

截至 2026 年，Ceres 本身不支持 GPU 加速。所有计算（残差求值、Jacobian 计算、线性求解）都在 CPU 上完成。对于需要 GPU 加速的大规模问题，有两条路径：

1. **GPU 加速残差求值**：将残差计算（如 BA 中的重投影）用 CUDA 实现，然后通过 `CostFunction::Evaluate` 的回调接口返回结果。Jacobian 仍需在 CPU 端组装。

2. **替代求解器**：DeepLM（ECCV 2022）和 GPUSLAM 等研究项目探索了在 GPU 上实现完整的 LM 求解。但这些项目通常针对特定问题结构（如 BA），不具备 Ceres 的通用性。

对于大多数 SLAM 后端任务，Ceres 的 CPU 实现已经足够——位姿图优化（几百个顶点）在毫秒级完成，全量 BA（几千张图片）在秒级完成。GPU 加速的收益更多体现在大规模离线重建（几万张图片）中。

---

## 24.15 GradientChecker 与调试工作流 ⭐⭐

### 调试 Jacobian 的标准方法

AutoDiff 的 Jacobian 是精确的（到机器精度），但仿函数的实现可能有 bug。手写 Jacobian（`AnalyticDiff`）更容易出错。Ceres 提供 `GradientChecker` 用于对比 AutoDiff/Analytic 和有限差分的 Jacobian：

```cpp
// 用 GradientChecker 验证手写 Jacobian
ceres::CostFunction* analytic_cost = new MyAnalyticCost(...);
ceres::CostFunction* auto_cost = new ceres::AutoDiffCostFunction<MyFunctor, 3, 7>(...);

// 在当前参数值处对比两种 Jacobian
ceres::GradientChecker checker(analytic_cost, /*manifolds=*/nullptr, {});
ceres::GradientChecker::ProbeResults results;
double params[7] = {...};
double* param_ptrs[] = {params};

if (!checker.Probe(param_ptrs, 1e-6, &results)) {
    // Jacobian 不一致——打印详细对比
    std::cerr << "Jacobian mismatch:\n"
              << "AutoDiff:\n" << results.jacobians[0] << "\n"
              << "Analytic:\n" << results.local_jacobians[0] << "\n"
              << "Max diff: " << results.maximum_relative_error << "\n";
}
```

### 完整的 Ceres 调试工作流

当优化结果不正确时，按以下顺序排查：

1. **检查初始残差**：`Problem::Evaluate()` 打印初始 cost，确认问题定义正确
2. **检查 Jacobian**：用 `GradientChecker` 对比 AutoDiff 和有限差分
3. **检查 Manifold**：确认流形维度（`AmbientSize` vs `TangentSize`）正确
4. **检查信息矩阵**：残差是否白化（除以标准差），各因子的残差量级是否一致
5. **检查收敛**：阅读 `summary.FullReport()`，关注 `termination_type` 和 `final_cost`
6. **简化问题**：用 2-3 个位姿的小问题验证正确性，再扩展到完整数据

这个工作流从数据层面（残差值）到算法层面（Jacobian）到系统层面（收敛性），逐层排除问题来源。大多数 Ceres 调试问题可以在前两步发现——最常见的 bug 是仿函数中的类型截断（把 Jet 转成 double）和参数布局错误（xyzw vs wxyz）。

---

## 跨章综合练习（补充）

4. **[综合 通用库·Ceres + 通用库·Eigen + 通用库·李群manif]** 从零实现一个最小化 SE(3) 位姿图优化器：
   - (a) 用 manif 的 `SE3d` 生成一个 10 帧的合成轨迹（圆形运动），添加高斯噪声作为里程计约束。
   - (b) 用 Ceres 的 `AutoDiffCostFunction` + `AutoDiffManifold<SE3d>` 建立位姿图，固定首帧。
   - (c) 添加一条回环约束（第 1 帧和第 10 帧），观察优化前后轨迹的变化。
   - (d) 用 `ceres::Covariance` 提取每个位姿的不确定性，用 Eigen 计算协方差椭球的长轴方向，验证回环约束是否减小了位姿不确定性。
   - (e) 比较 `SPARSE_NORMAL_CHOLESKY` 和 `DENSE_QR` 的求解时间，解释为什么位姿图应该用稀疏求解器。

---

## 24.16 Ceres Solver 版本演进与迁移指南 ⭐⭐

### 从 Ceres 1.x 到 2.x 的关键变化

Ceres Solver 从 1.x 到 2.x 经历了重要的 API 重构。对于维护既有 SLAM 代码库（如从 VINS-Mono、ORB-SLAM3 等项目迁移）的工程师来说，了解这些变化至关重要。

| 特性 | Ceres 1.x | Ceres 2.x | 迁移工作量 |
|------|-----------|-----------|-----------|
| 流形约束 | `LocalParameterization` | `Manifold`（需实现 `Minus`） | 中等 |
| 参数设置 | `SetParameterization()` | `SetManifold()` | 简单重命名 |
| 自动微分流形 | 不支持 | `AutoDiffManifold` | 新功能 |
| 损失函数所有权 | 手动管理或 Problem 接管 | Problem 默认接管 | 检查所有权 |
| 线程模型 | OpenMP | 内置线程池 | 无需修改 |
| C++ 标准 | C++11 | C++17 | 更新编译器 |

最常见的迁移陷阱是 `Manifold` 接口要求实现 `Minus` 和 `MinusJacobian`——这两个方法在 `LocalParameterization` 中不存在。`Minus(y, x, delta)` 计算从 $x$ 到 $y$ 的切空间增量 $\delta$，使得 $y = x \oplus \delta$。对于四元数和 SO(3)，`Minus` 是对数映射。

```cpp
// Ceres 1.x 风格
class QuaternionParameterization : public ceres::LocalParameterization {
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
    bool ComputeJacobian(const double* x, double* jacobian) const override;
    int GlobalSize() const override { return 4; }
    int LocalSize() const override { return 3; }
};

// Ceres 2.x 风格——新增 Minus 和 MinusJacobian
class QuaternionManifold : public ceres::Manifold {
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
    bool PlusJacobian(const double* x, double* jacobian) const override;
    bool Minus(const double* y, const double* x, double* y_minus_x) const override;  // 新增
    bool MinusJacobian(const double* x, double* jacobian) const override;              // 新增
    int AmbientSize() const override { return 4; }
    int TangentSize() const override { return 3; }
};
```

> **反事实推理**：如果 Ceres 1.x 的 `LocalParameterization` 设计已经足够好，为什么要引入 `Manifold`？
> 因为 `LocalParameterization` 只定义了 `Plus`（从切空间到流形），没有定义 `Minus`（从流形到切空间）。
> 这意味着 Ceres 1.x 无法在流形上计算两个点之间的距离——一些高级功能（如流形上的线搜索、协方差估计）受到限制。
> `Manifold` 接口的 `Plus`/`Minus` 构成了完整的流形映射对，使 Ceres 2.x 能够在流形上做更多操作。

### 版本兼容性策略

对于需要同时支持 Ceres 1.x 和 2.x 的项目（如开源库），可以用条件编译处理 API 差异：

```cpp
#include <ceres/ceres.h>

#if CERES_VERSION_MAJOR >= 2
    using ManifoldType = ceres::EigenQuaternionManifold;
    #define SET_MANIFOLD(problem, block, manifold) \
        (problem)->SetManifold((block), (manifold))
#else
    using ManifoldType = ceres::EigenQuaternionParameterization;
    #define SET_MANIFOLD(problem, block, manifold) \
        (problem)->SetParameterization((block), (manifold))
#endif
```

这种方式虽然不够优雅，但在过渡期是实用的解决方案。长期来看应当完全迁移到 Ceres 2.x。
