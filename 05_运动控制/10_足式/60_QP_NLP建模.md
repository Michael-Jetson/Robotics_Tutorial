> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第50章 QP/NLP建模——从SLAM无约束优化到规控有约束优化

> **学习周期**：1周（20-25小时）  
> **难度分布**：⭐基础概念 → ⭐⭐核心算法 → ⭐⭐⭐工程精通  
> **前置依赖**：01_数学/60_概率与估计/50_因子图（Ceres+GTSAM 无约束优化）、02_基础/10_Eigen（Eigen 稀疏矩阵）、足式/30_Pinocchio深度精读（Pinocchio 动力学生态）  
> **适用方向**：✅ 腿足 · ✅ 机械臂 · ✅ 移动机器人 · ✅ 具身智能——所有子方向共享

---

## 50.0 前置自测

在开始本章之前，请独立回答以下5个问题。如果能答对4个以上，本章可快速浏览；否则请逐节精读。

| # | 问题 | 期望答案要点 |
|---|------|-------------|
| 1 | Ceres Solver 的核心算法是什么？它能处理不等式约束吗？ | Gauss-Newton / Levenberg-Marquardt 求解 $\min \|r(x)\|^2$；**不能**处理通用不等式约束 $h(x) \le 0$ |
| 2 | Gauss-Newton 法的 Hessian 近似公式是什么？为什么它比 Newton 法便宜？ | $H \approx J^T J$（省去了二阶导数 $\sum r_i \nabla^2 r_i$），只需一阶 Jacobian |
| 3 | Eigen 的 `SparseMatrix<double>` 默认使用什么压缩格式？为什么 QP 求解器关心这个？ | CSC（Compressed Sparse Column）；QP 求解器内部做稀疏分解时要求特定格式 |
| 4 | Pinocchio 的 `rnea()` 计算的是什么量？它在优化中通常作为什么角色？ | 逆动力学 $\tau = M\ddot{q} + C\dot{q} + g$；在优化中作为**等式约束**（动力学可行性） |
| 5 | "有约束优化"和"无约束优化"在算法层面的根本区别是什么？ | 有约束需要处理 KKT 条件（引入拉格朗日乘子）；无约束只需梯度为零 |

> ⚠️ **常见陷阱**：很多 SLAM 背景的同学认为"加个正则项 penalty 就能处理约束"——这在控制中**几乎不可行**。关节限位、力矩饱和、摩擦锥是**硬约束**（物理不可违反），penalty 方法无法保证可行性。本章将给出正确的处理范式。

## 50.0.1 本章目标

学完本章后，你应该能够：

1. **区分** QP 与 NLP 的问题结构差异，判断一个机器人优化问题应该建模为 QP 还是 NLP
2. **手写** KKT 条件并解释互补松弛条件的物理含义——为什么"顶在约束边界上"的约束对应非零对偶变量
3. **使用 OSQP / ProxQP** 独立求解一个包含动力学等式约束和摩擦锥不等式约束的 QP，并理解 warm-start 对实时性的关键作用
4. **使用 Ifopt / CasADi** 建模并求解一个非线性轨迹优化问题，理解 SQP 和 Interior-Point 两种 NLP 求解策略的区别
5. **做出选型决策**：根据问题规模、实时性要求和约束类型，在 OSQP / ProxQP / HPIPM / Ipopt / acados 中选择合适的求解器

---

## 50.1 范式跨越：从SLAM到规控 ⭐

### 50.1.1 问题的根本差异 ⭐

**核心命题**：SLAM 和控制的优化问题在数学形式上有本质区别——前者是无约束最小二乘，后者是有约束非线性规划。理解这个跨越是从感知工程师转向控制工程师的第一步。

| 维度 | SLAM（01_数学/60_概率与估计/50_因子图） | 规控（本章开始） |
|------|----------------|----------------|
| **目标函数** | $\min_x \sum_i \|r_i(x)\|^2_{\Sigma_i}$ | $\min_x f(x)$（可以不是最小二乘） |
| **约束** | 无（或仅流形约束） | $g(x)=0$（等式）+ $h(x)\le 0$（不等式） |
| **典型求解器** | Ceres / g2o / GTSAM | OSQP / ProxQP / Ipopt / acados |
| **Hessian 结构** | $J^T J$（Gauss-Newton 近似） | 完整 Hessian 或 QP 的 $H$ 矩阵 |
| **问题规模** | $10^3 \sim 10^6$ 变量 | $10^1 \sim 10^4$ 变量（但实时性要求极高） |
| **求解频率** | 离线或 1-10 Hz | 100-1000 Hz（MPC/WBC 控制周期） |
| **失败后果** | 地图漂移 | **硬件损坏、机器人摔倒** |
| **凸性** | 局部最小二乘（非凸） | QP 全局凸；NLP 通常非凸，只能保证局部最优 |

### 50.1.2 为什么控制需要约束？ ⭐

SLAM 的优化变量是"位姿"和"路标点"——它们本质上是自由的（在流形上），没有物理约束。但控制的变量是：

```
关节角度 q ∈ [q_min, q_max]          — 关节限位（硬件结构决定）
关节力矩 τ ∈ [τ_min, τ_max]          — 电机饱和（物理极限）
接触力 f, 满足 |f_t| ≤ μ f_n         — 摩擦锥（物理定律）
质心加速度 ẍ_CoM                      — 动力学可行性（牛顿-欧拉定律）
```

这些约束是**不可违反**的。关节角超限 → 机械结构损坏；力矩超限 → 电机过热保护；接触力出摩擦锥 → 脚打滑跌倒。

**Ceres 为什么不行？**

```cpp
// Ceres 的 Problem 接口——没有 AddInequalityConstraint()！
ceres::Problem problem;
problem.AddResidualBlock(cost, loss, &x);  // ✅ 代价函数
problem.SetParameterLowerBound(&x, 0, lb); // ⚠️ 只有简单盒约束
// problem.AddConstraint(g, &x);           // ❌ 不存在这个接口！
```

Ceres 的 `SetParameterLowerBound/UpperBound` 只支持**简单盒约束**（变量上下界），无法表达：
- 线性不等式 $Ax \le b$
- 非线性不等式 $h(x) \le 0$
- 一般等式约束 $g(x) = 0$（除流形约束外）

从"变量自由"到"变量受限"，这一步跨越的不仅是数学形式——它改变了整个求解器的设计哲学。这好比从"在空旷草原上骑马"到"在拥挤城市里开车"：SLAM 的优化变量可以自由漫步（流形上无障碍），规控的变量被关节限位、力矩饱和、摩擦锥层层围堵，必须在狭窄的可行域通道中找到最优路径。接下来我们正式进入有约束优化的数学框架。

### 50.1.3 从无约束到有约束的数学框架 ⭐⭐

**无约束优化**（SLAM paradigm）：

$$\min_{x \in \mathbb{R}^n} f(x)$$

最优性条件：$\nabla f(x^*) = 0$（梯度为零）

**有约束优化**（Control paradigm）：

$$\min_{x \in \mathbb{R}^n} f(x) \quad \text{s.t.} \quad g_i(x) = 0, \; i=1,...,p; \quad h_j(x) \le 0, \; j=1,...,m$$

最优性条件：**KKT条件**（Karush-Kuhn-Tucker）——需要引入新的数学工具。

### 50.1.4 KKT条件初步 ⭐⭐

定义 Lagrangian 函数：

$$\mathcal{L}(x, \lambda, \mu) = f(x) + \sum_{i=1}^{p} \lambda_i g_i(x) + \sum_{j=1}^{m} \mu_j h_j(x)$$

其中 $\lambda_i$ 是等式约束的 Lagrange 乘子，$\mu_j$ 是不等式约束的 Lagrange 乘子。

**KKT 必要条件**（一阶最优性）：

$$\begin{aligned}
\nabla_x \mathcal{L} &= 0 & \text{（驻点条件）} \\
g_i(x^*) &= 0, \quad \forall i & \text{（等式约束满足）} \\
h_j(x^*) &\le 0, \quad \forall j & \text{（不等式约束满足）} \\
\mu_j &\ge 0, \quad \forall j & \text{（对偶可行性）} \\
\mu_j h_j(x^*) &= 0, \quad \forall j & \text{（互补松弛条件）}
\end{aligned}$$

**互补松弛条件**的物理意义（机器人视角）：
- 如果约束 $h_j$ 不激活（$h_j(x^*) < 0$），则乘子 $\mu_j = 0$（约束没施加力）
- 如果乘子 $\mu_j > 0$（约束在施加力），则 $h_j(x^*) = 0$（约束正好在边界上）

**机器人例子**：如果关节没顶到限位（$q < q_{\max}$），限位不产生反力（$\mu = 0$）；如果限位产生了反力（$\mu > 0$），那关节一定顶在限位上（$q = q_{\max}$）。

| 概念 | SLAM 对应 | 控制对应 |
|------|-----------|---------|
| 决策变量 | 位姿 $T_i$、路标 $l_j$ | 关节角 $q$、力矩 $\tau$、接触力 $f$ |
| 目标函数 | 重投影误差 $\|r\|^2$ | 跟踪误差 + 能量消耗 |
| 等式约束 | 无 | 动力学方程 $M\ddot{q}+h=S^T\tau+J^Tf$ |
| 不等式约束 | 无 | 关节限位、力矩限位、摩擦锥 |
| 最优性条件 | $\nabla f = 0$ | KKT 条件（5个方程组） |
| 对偶变量 | 无 | 接触力、约束反力 |

> ⚠️ **常见陷阱**：很多教材把 KKT 条件写成"必要条件"就结束了。在凸优化（特别是 QP）中，KKT 条件是**充分必要条件**——求解 KKT 系统等价于求解原问题。这是 QP 求解器的理论基础。

### 50.1.5 三类优化问题的层次 ⭐

```
┌─────────────────────────────────────────────────────────┐
│                 通用 NLP (Nonlinear Programming)          │
│         min f(x) s.t. g(x)=0, h(x)≤0                   │
│         求解器: Ipopt, SNOPT, WORHP                      │
│   ┌─────────────────────────────────────────────────┐   │
│   │          QP (Quadratic Programming)              │   │
│   │     min ½x'Hx+f'x s.t. Ax≤b, Cx=d             │   │
│   │     求解器: OSQP, ProxQP, qpOASES, PIQP         │   │
│   │   ┌─────────────────────────────────────────┐   │   │
│   │   │   OCP-structured QP (MPC-specific)       │   │   │
│   │   │   带时域 banded-sparse 结构              │   │   │
│   │   │   求解器: HPIPM, acados                  │   │   │
│   │   └─────────────────────────────────────────┘   │   │
│   └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

**练习 50.1** ⭐：写出以下控制问题的优化形式（明确变量、目标、约束），并判断是 QP 还是 NLP：
1. 7-DOF 机械臂静态力矩分配（给定末端力，求关节力矩使能量最小）
2. 四足 WBC（给定期望质心加速度，求接触力和关节力矩）
3. 单腿跳跃轨迹优化（给定初末状态，求中间轨迹使能量最小）

**练习 50.2** ⭐：对 $\min x^2+y^2 \text{ s.t. } x+y \ge 1$ 手动推导 KKT 条件并求解。验证 $x^*=y^*=0.5$，$\mu^*=1$。

---

## 50.2 QP标准形式与KKT条件 ⭐⭐

### 50.2.1 QP标准形式 ⭐⭐

二次规划（Quadratic Programming）是控制中最常见的优化子问题。标准形式：

$$\min_{x \in \mathbb{R}^n} \quad \frac{1}{2} x^T H x + f^T x$$

$$\text{s.t.} \quad A_{\text{eq}} x = b_{\text{eq}}, \quad A_{\text{ineq}} x \le b_{\text{ineq}}$$

其中：
- $H \in \mathbb{R}^{n \times n}$：Hessian 矩阵（半正定 $H \succeq 0$ 保证凸性）
- $f \in \mathbb{R}^n$：线性项
- $A_{\text{eq}} \in \mathbb{R}^{p \times n}$：等式约束矩阵
- $A_{\text{ineq}} \in \mathbb{R}^{m \times n}$：不等式约束矩阵

**凸性保证**：当 $H \succeq 0$（半正定）时，QP 是凸优化问题，局部最优就是全局最优。这是 QP 相比通用 NLP 的巨大优势。

### 50.2.2 不同求解器的约束形式约定 ⭐⭐

| 求解器 | 目标函数 | 约束形式 | 矩阵格式 |
|--------|---------|---------|----------|
| **OSQP** | $\min \frac{1}{2}x^TPx + q^Tx$ | $l \le Ax \le u$（上下界统一） | CSC (Eigen SparseMatrix) |
| **ProxQP** | $\min \frac{1}{2}x^THx + g^Tx$ | $Ax = b$, $Cx \le u$ | Eigen Dense/Sparse 原生 |
| **qpOASES** | $\min \frac{1}{2}x^THx + g^Tx$ | $l_A \le Ax \le u_A$, $l \le x \le u$ | Dense row-major |
| **HPIPM** | OCP 结构 | 动力学等式 + 路径不等式 + 盒约束 | Block-sparse (BLASFEO) |
| **PIQP** | $\min \frac{1}{2}x^TPx + c^Tx$ | $Ax=b$, $Gx\le h$, $x_l\le x\le x_u$ | CSC 或 Dense |

> ⚠️ **常见陷阱**：不同求解器的"标准形式"不同！OSQP 把等式和不等式统一为 $l \le Ax \le u$（等式就是 $l_i = u_i$），而 ProxQP/PIQP 分开写。第一次集成新求解器时，**矩阵维度错误**是最常见的 bug。

### 50.2.3 QP的KKT系统完整推导 ⭐⭐⭐

对标准 QP 写 Lagrangian：

$$\mathcal{L}(x, \lambda, \mu) = \frac{1}{2}x^THx + f^Tx + \lambda^T(A_{\text{eq}}x - b_{\text{eq}}) + \mu^T(A_{\text{ineq}}x - b_{\text{ineq}})$$

KKT 条件展开：

$$\begin{aligned}
\nabla_x \mathcal{L} &= Hx + f + A_{\text{eq}}^T\lambda + A_{\text{ineq}}^T\mu = 0 \\
A_{\text{eq}}x &= b_{\text{eq}} \\
A_{\text{ineq}}x &\le b_{\text{ineq}} \\
\mu &\ge 0 \\
\mu_j (A_{\text{ineq},j}x - b_{\text{ineq},j}) &= 0, \quad \forall j
\end{aligned}$$

当我们知道哪些不等式约束是**激活的**（active set $\mathcal{A}$），问题简化为线性方程组：

$$\begin{bmatrix} H & A_{\text{eq}}^T & A_{\mathcal{A}}^T \\ A_{\text{eq}} & 0 & 0 \\ A_{\mathcal{A}} & 0 & 0 \end{bmatrix} \begin{bmatrix} x \\ \lambda \\ \mu_{\mathcal{A}} \end{bmatrix} = \begin{bmatrix} -f \\ b_{\text{eq}} \\ b_{\mathcal{A}} \end{bmatrix}$$

这就是 **KKT 线性系统**——所有 QP 求解器的核心都是在求解这个系统（或它的等价变形）。

### 50.2.4 对偶变量的物理意义 ⭐⭐

在机器人控制中，拉格朗日乘子 $\lambda, \mu$ 有明确的物理含义：

| 约束类型 | 对偶变量含义 | 例子 |
|---------|------------|------|
| 动力学等式 $M\ddot{q}+h=\tau+J^Tf$ | 约束力 | 关节约束反力 |
| 关节限位 $q \le q_{\max}$ | 限位反力 | 碰到机械止挡时的力 |
| 摩擦锥 $\|f_t\| \le \mu f_n$ | 滑动趋势 | 接近滑动时乘子增大 |
| 力矩限制 $\|\tau\| \le \tau_{\max}$ | 性能降级指标 | 乘子大 → 接近饱和 |

**工程价值**：对偶变量告诉你"约束有多紧"。在 WBC 中，监控摩擦锥约束的乘子可以预警"即将打滑"。如果不监控对偶变量，工程师就像开车不看油表——不知道距离"燃料耗尽"（约束被违反）还有多远，直到机器人突然打滑跌倒才发现问题。

> **本质洞察**：KKT 条件中的互补松弛条件 $\mu_j h_j(x^*) = 0$ 揭示了有约束优化的一个深层真相：约束和自由是此消彼长的。每一个激活的约束都"消耗"一个自由度——它把优化问题从 $n$ 维空间投影到一个更低维的超平面上。对偶变量 $\mu_j$ 的大小告诉你这个约束有多"值钱"——放松它一点，目标函数能改善多少。这就是为什么 QP 的对偶解在工程中和主解（primal）同样重要。

### 50.2.5 Active Set 概念 ⭐⭐

**Active Set**（激活集）= 在最优解处"顶在边界上"的约束集合。

```
约束空间示意（2D 例子）：
         
    ╲  可行域  ╱
     ╲       ╱
      ╲ x*  ╱  ← x* 在两条约束边界的交点 → Active Set = {c1, c2}
       ╲   ╱
        ╲ ╱
         V
        
如果 x* 在可行域内部 → Active Set = ∅
如果 x* 在一条边界上 → Active Set = {那条约束}
```

**两类 QP 算法的区别**：
- **Active-set 方法**（qpOASES）：猜测 active set，求解等式 KKT 系统，调整猜测，迭代
- **Interior-point 方法**（HPIPM, PIQP）：从内部逼近边界，用 barrier 函数松弛互补条件
- **First-order 方法**（OSQP/ADMM, ProxQP）：用算子分裂或近端算子迭代逼近

| 方法类 | 代表求解器 | 优点 | 缺点 |
|--------|-----------|------|------|
| Active-set | qpOASES | 精确解、支持热启动 | 最坏指数复杂度 |
| Interior-point | HPIPM, PIQP | 多项式复杂度、结构利用 | 热启动困难 |
| First-order | OSQP, ProxQP | 快速近似解、热启动优秀 | 高精度收敛慢 |

**练习 50.3** ⭐⭐：对 $\min \frac{1}{2}(x_1^2 + x_2^2)$ s.t. $x_1 + x_2 \ge 1$, $x_1 \ge 0$, $x_2 \ge 0$，手动求 active set 和 KKT 解。

---

## 50.3 QP求解器全景 ⭐⭐⭐

### 50.3.1 2025-2026 年 QP 求解器版图 ⭐

基于 qpsolvers v4.11.0（2026年3月）和 Simple-Robotics/proxqp_benchmark 的实测数据：

| 求解器 | 版本 | 算法 | 语言 | License | 热启动 | 典型场景 |
|--------|------|------|------|---------|--------|---------|
| **OSQP** | 1.0 | ADMM（一阶） | C + osqp-eigen | Apache-2.0 | ✅优秀 | 通用 QP，生态最大 |
| **ProxQP** | 0.6+ | 近端增广 Lagrangian | C++17 Eigen | BSD-2 | ✅✅卓越 | 机器人 QP（WBC/IK） |
| **qpOASES** | 3.2 | Online active-set | C++ | LGPL-2.1 | ✅良好 | 遗留 MPC（MIT Cheetah） |
| **HPIPM** | 0.1.x (upstream tags) | IPM + Riccati 递推 | C (BLASFEO) | BSD-2 | 部分 | **MPC 专用** |
| **PIQP** | 0.4+ | 近端 interior-point | C++14 header-only | BSD-2 | ✅良好 | 新兴、嵌入式友好 |
| **qpSWIFT** | 1.0 | IPM + Mehrotra | ANSI C | GPL-3.0 | 部分 | Ghost Robotics |
| **Clarabel** | 0.7+ | IPM (锥规划) | Rust/Julia | Apache-2.0 | 部分 | 锥规划、SDP |

### 50.3.2 性能基准（RSS 2022 + 2025更新数据） ⭐⭐

**典型机器人 QP 场景**（Bambade et al., RSS 2022; Simple-Robotics benchmark 2025）：

| 问题类型 | 规模 | ProxQP | OSQP | qpOASES | PIQP |
|---------|------|--------|------|---------|------|
| 逆运动学 QP | n=7, m=20 | **24±7 us** | 167±93 us | 89±15 us | 45±12 us |
| 逆动力学 QP | n=18, m=40 | **25±6 us** | 441±193 us | 112±28 us | 67±18 us |
| WBC (全身) | n=30, m=60 | **52±11 us** | 890±310 us | 205±45 us | 98±25 us |
| 大规模 MPC | n=500, m=1000 | 3.2 ms | 5.8 ms | N/A (dense) | 2.1 ms |

**关键发现**：
- ProxQP 在中小规模密集 QP 上比 OSQP **快 7-18 倍**
- ProxQP 在高精度（eps=1e-9）下优势更明显（OSQP 作为一阶方法收敛慢）
- HPIPM 在结构化 MPC-QP 上比通用求解器快 **2-10 倍**
- PIQP 的 header-only 设计使其编译集成最简单

### 50.3.3 五大求解器算法概览 ⭐⭐

**OSQP — ADMM 算法**：
```
将 QP 分裂为两个子问题：
  x-update: 求解线性系统 (P + σI + ρA'A)x = ...
  z-update: 投影到约束集 z = Π_C(Ax + ...)
  对偶更新: y = y + ρ(Ax - z)
迭代直到 primal/dual residual < ε
```

**ProxQP — 近端增广 Lagrangian**：
```
核心: 在增广 Lagrangian 上做近端迭代
  primal: x^{k+1} = argmin L_ρ(x, λ^k, μ^k) + ½σ‖x-x^k‖²
  dual:   λ^{k+1}, μ^{k+1} = 对偶更新
关键创新: 自适应 ρ + 精确线性系统求解（vs OSQP 的近似）
```

**qpOASES — Online Active-Set**：
```
维护 active set 工作集 W
每次迭代: 添加/删除一条约束 → 低秩更新 KKT 因式分解
特点: 精确解、可预测迭代次数（最坏 2^n 但实际 O(n)）
```

**HPIPM — 结构化 IPM**：
```
利用 OCP 的 banded-sparse 结构:
  KKT 系统是 block-tridiagonal → Riccati 递推 O(N·n³)
  vs 通用稀疏: O(N³·n³) 或 O(N·n^{3.5})
关键: BLASFEO 高性能线性代数后端
```

**PIQP — 近端 Interior-Point**：
```
Interior-point + proximal regularization:
  解决 IPM 的数值不稳定性（barrier 参数趋近零时）
  支持 warm-start（传统 IPM 难以 warm-start）
  Header-only C++14，零外部依赖
```

### 50.3.4 何时选择哪个？ ⭐⭐

```
你的 QP 问题特征是什么？
│
├─ MPC 结构（时域 horizon, 状态-输入-动力学）？
│   └─ YES → HPIPM (via acados/OCS2)
│
├─ 中小规模（n<100），实时性极高（<100us）？
│   └─ YES → ProxQP (dense mode)
│
├─ 通用稀疏 QP，需要最广泛的社区支持？
│   └─ YES → OSQP
│
├─ 嵌入式部署，需要 header-only、零 malloc？
│   └─ YES → PIQP
│
├─ 遗留代码集成（MIT Cheetah / 早期 MPC）？
│   └─ YES → qpOASES (但新项目请避免)
│
└─ 锥规划（二阶锥约束/摩擦锥直接建模）？
    └─ YES → Clarabel 或 ECOS
```

> ⚠️ **常见陷阱**：不要被"OSQP 比 ProxQP 慢"就放弃 OSQP。OSQP 的优势是：(1) 社区最大、文档最全；(2) Python/MATLAB/Julia/R 全平台支持；(3) 代码生成器 OSQP-Codegen 适合嵌入式。**选求解器要看生态，不只看微秒级速度**。

**练习 50.4** ⭐⭐：查阅 OSQP 和 ProxQP 的 GitHub README，列出它们各自的 C++ 依赖项（几个库？需要哪些编译工具？）。哪个集成更简单？

---

## 50.4 OSQP深度精读 ⭐⭐

### 50.4.1 ADMM算法推导 ⭐⭐⭐

OSQP 求解的标准形式：

$$\min_x \frac{1}{2}x^TPx + q^Tx \quad \text{s.t.} \quad l \le Ax \le u$$

引入辅助变量 $z = Ax$，等价于：

$$\min_{x,z} \frac{1}{2}x^TPx + q^Tx \quad \text{s.t.} \quad Ax = z, \; l \le z \le u$$

ADMM（Alternating Direction Method of Multipliers）将其分裂：

**x-update**（无约束二次优化）：

$$x^{k+1} = \arg\min_x \frac{1}{2}x^TPx + q^Tx + \frac{\rho}{2}\|Ax - z^k + y^k\|^2$$

$$\Rightarrow (P + \rho A^TA)x^{k+1} = -q + \rho A^T(z^k - y^k)$$

**z-update**（投影到盒约束）：

$$z^{k+1} = \Pi_{[l,u]}(Ax^{k+1} + y^k)$$

$$\Pi_{[l,u]}(v)_i = \text{clip}(v_i, l_i, u_i)$$

**对偶更新**：

$$y^{k+1} = y^k + Ax^{k+1} - z^{k+1}$$

**OSQP 的核心优化**：$(P + \rho A^TA)$ 在问题结构不变时只需**因式分解一次**（LDL分解），后续只做前向/后向替代。这是 OSQP 能在 MPC 循环中保持 warm-start 的关键。

### 50.4.2 关键参数 rho 的影响 ⭐⭐

| rho 值 | 效果 | 适用场景 |
|--------|------|---------|
| rho 太小 | 约束满足快，但目标收敛慢 | 约束紧的问题 |
| rho 太大 | 目标收敛快，但约束满足慢 | 几乎无约束的问题 |
| rho 自适应 | OSQP 默认策略：根据 primal/dual residual 比值动态调整 | 推荐 |

**OSQP 的 rho 自适应规则**：
```
if primal_res / dual_res > mu:
    rho <- tau * rho        // 增大 rho，加速约束满足
elif dual_res / primal_res > mu:
    rho <- rho / tau        // 减小 rho，加速目标收敛
```

默认 $\mu = 10$, $\tau = 2$。

> ⚠️ **常见陷阱**：当 rho 更新时，OSQP 需要**重新做 LDL 分解**（因为 $P + \rho A^TA$ 变了）。如果 rho 频繁更新，性能会下降。设置 `adaptive_rho_interval` 可以控制更新频率。

### 50.4.3 Warm-Starting 的工程价值 ⭐⭐

在 MPC/WBC 循环中，每个控制周期的 QP 问题**只有微小变化**（初始状态更新、参考轨迹平移）。Warm-start 把上一次的解 $(x^k, z^k, y^k)$ 作为下一次的初始值：

```cpp
// OSQP-Eigen warm-start 示例
#include <OsqpEigen/OsqpEigen.h>

OsqpEigen::Solver solver;
solver.settings()->setWarmStart(true);

// 首次求解
solver.initSolver();
solver.solveProblem();
Eigen::VectorXd x_prev = solver.getSolution();

// 后续 MPC 周期: 更新问题参数 + warm-start
solver.updateGradient(q_new);           // 更新线性项
solver.updateBounds(l_new, u_new);      // 更新约束界
solver.setPrimalVariable(x_prev);       // warm-start primal
solver.setDualVariable(y_prev);         // warm-start dual
solver.solveProblem();                  // 通常 3-10 次迭代收敛（vs 冷启动 50-200 次）
```

### 50.4.4 完整 WBC 集成示例 ⭐⭐

一个简化的 Whole-Body Control QP（12自由度四足，站立平衡）：

```cpp
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

/**
 * WBC QP: min 1/2 ||tau||^2 + w * 1/2 ||f - f_des||^2
 * s.t. M*qddot + h = S'*tau + J'*f    (动力学等式)
 *      f_z >= f_min                    (接触力正)
 *      |f_x| <= mu*f_z                 (摩擦锥线性化)
 *      tau_min <= tau <= tau_max        (力矩限位)
 *
 * 决策变量: x = [tau(12); f(12)]  共24维
 * 等式约束: 浮动基座动力学 6 维
 * 不等式约束: 摩擦锥(16) + 法向力下界(4) = 20 维
 */
class SimpleWBC {
public:
    static constexpr int n_joints = 12;              // 关节数
    static constexpr int n_contacts = 4;             // 接触脚数
    static constexpr int n_force = 3 * n_contacts;   // 接触力维度 = 12
    static constexpr int n_var = n_joints + n_force; // 决策变量 = 24
    static constexpr int n_eq = 6;                   // 浮动基座动力学
    static constexpr int n_ineq = 20;                // 摩擦锥 + 法向力
    static constexpr int n_con = n_eq + n_ineq;      // 总约束 = 26

    void setup() {
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setAbsoluteTolerance(1e-4);
        solver_.settings()->setRelativeTolerance(1e-4);
        solver_.settings()->setMaxIteration(200);
        
        solver_.data()->setNumberOfVariables(n_var);
        solver_.data()->setNumberOfConstraints(n_con);
        
        // 构建 Hessian P = diag(I_tau, w*I_f)
        Eigen::SparseMatrix<double> P(n_var, n_var);
        std::vector<Eigen::Triplet<double>> P_triplets;
        for (int i = 0; i < n_joints; ++i)
            P_triplets.emplace_back(i, i, 1.0);           // tau 权重
        for (int i = 0; i < n_force; ++i)
            P_triplets.emplace_back(n_joints+i, n_joints+i, weight_);  // f 权重
        P.setFromTriplets(P_triplets.begin(), P_triplets.end());
        
        // 构建约束矩阵 A (OSQP 格式: l <= Ax <= u)
        // 前6行: 动力学 [S' | J_c'] * [tau; f] (等式: l=u=M*qddot_des+h)
        // 后20行: 摩擦锥线性化 (不等式)
        Eigen::SparseMatrix<double> A(n_con, n_var);
        buildConstraintMatrix(A);
        
        solver_.data()->setHessianMatrix(P);
        solver_.data()->setLinearConstraintsMatrix(A);
        solver_.initSolver();
    }

    Eigen::VectorXd solve(const Eigen::VectorXd& dynamics_rhs,
                          const Eigen::VectorXd& f_des) {
        // 更新梯度 q = [0; -w*f_des]
        Eigen::VectorXd gradient(n_var);
        gradient.head(n_joints).setZero();
        gradient.tail(n_force) = -weight_ * f_des;
        solver_.updateGradient(gradient);
        
        // 更新约束界 (动力学右端项随状态变化)
        Eigen::VectorXd lower(n_con), upper(n_con);
        lower.head(n_eq) = dynamics_rhs;   // 等式: l = u
        upper.head(n_eq) = dynamics_rhs;
        lower.tail(n_ineq) = friction_lower_;  // 摩擦锥下界
        upper.tail(n_ineq) = friction_upper_;  // 摩擦锥上界
        solver_.updateBounds(lower, upper);
        
        solver_.solveProblem();
        return solver_.getSolution();
    }

private:
    OsqpEigen::Solver solver_;
    double weight_ = 10.0;  // 接触力跟踪权重
    Eigen::VectorXd friction_lower_, friction_upper_;
    
    void buildConstraintMatrix(Eigen::SparseMatrix<double>& A) {
        // 由 Pinocchio 计算的 Selection matrix S 和 Contact Jacobian J_c 填充。
        // 足式/90_WBC分层优化与TSID 将把浮动基座动力学方程 M*ddq + h = S'*tau + Jc'*lambda
        // 拆分为浮基行（6 维等式约束）和关节行（力矩表达式），
        // 并将摩擦锥线性化为 A_friction * f <= b_friction 的不等式约束，
        // 组装为完整的 WBC-QP。
    }
};
```

### 50.4.5 OSQP 调参指南 ⭐⭐

| 参数 | 默认值 | 建议值（WBC） | 说明 |
|------|--------|-------------|------|
| `eps_abs` | 1e-3 | 1e-4 ~ 1e-5 | 绝对精度（力控需要更高） |
| `eps_rel` | 1e-3 | 1e-4 | 相对精度 |
| `max_iter` | 4000 | 100 ~ 300 | MPC 有严格时间限制 |
| `rho` | 0.1 | 自适应 | 让 OSQP 自己调 |
| `sigma` | 1e-6 | 1e-6 | 正则化（数值稳定性） |
| `alpha` | 1.6 | 1.6 | 过松弛参数 |
| `polish` | false | true（如果有时间） | 精确解打磨 |
| `scaled_termination` | false | true | 缩放终止条件 |

> ⚠️ **常见陷阱**：OSQP 的 `updateHessianMatrix()` 只能更新**非零元素的值**，不能改变稀疏模式。如果你的 QP 在运行时稀疏模式会变（例如接触脚数变化），必须用 `solver.clearSolver()` + `solver.initSolver()` 重新初始化——这会导致一次冷启动，可能超出实时限制。

**练习 50.5** ⭐⭐：用 OSQP-Eigen 实现一个 10 维 QP，带 20 条随机线性不等式约束。分别测试 cold-start 和 warm-start（修改一个约束界再求解）的迭代次数差异。预期 warm-start 减少 80%+ 迭代。

---

## 50.5 ProxQP与HPIPM ⭐⭐⭐

### 50.5.1 ProxQP：近端增广 Lagrangian 算法 ⭐⭐⭐

**ProxQP**（Simple Robotics / INRIA, 2022-2026）的核心创新：

标准 QP：$\min \frac{1}{2}x^THx + g^Tx$ s.t. $Ax = b$, $Cx \le u$

**增广 Lagrangian**：

$$\mathcal{L}_\rho(x, \lambda, \mu) = \frac{1}{2}x^THx + g^Tx + \lambda^T(Ax-b) + \frac{\rho_{\text{eq}}}{2}\|Ax-b\|^2 + \text{ineq terms}$$

**近端迭代**在每步添加正则项 $\frac{\sigma}{2}\|x - x^k\|^2$：

$$x^{k+1} = \arg\min_x \mathcal{L}_\rho(x, \lambda^k, \mu^k) + \frac{\sigma}{2}\|x - x^k\|^2$$

展开后变成求解线性系统：

$$(H + \rho_{\text{eq}} A^TA + \rho_{\text{ineq}} C_{\mathcal{A}}^TC_{\mathcal{A}} + \sigma I) x = \text{rhs}$$

**相比 OSQP 的关键区别**：

| 特性 | OSQP (ADMM) | ProxQP |
|------|-------------|--------|
| 线性系统求解 | 近似（用固定 factorization） | **精确**（每步 refactor） |
| 收敛阶 | 线性 O(1/k) | **超线性** |
| 高精度 (eps=1e-9) | 需 1000+ 迭代 | 通常 <20 迭代 |
| 自适应参数 | rho 调整（间接） | rho + sigma 联合自适应 |
| API 设计 | C + wrapper | **Eigen 原生** |
| 内存分配 | 初始化时分配 | 初始化时分配 |

**为什么 ProxQP 更快？** OSQP 固定 $(P + \rho A^TA)$ 的分解以节省时间，但代价是每步只能做"近似"牛顿方向。ProxQP 选择每步都精确求解（重新分解），但它的近端正则项 $\sigma I$ 保证了数值稳定性，使得总迭代次数大幅减少。在中小规模问题上，"少迭代 * 精确步"比"多迭代 * 近似步"更快。这就像导航中的"走大路 vs 抄近路"：OSQP 每步走得轻松（近似分解）但绕路多（线性收敛），ProxQP 每步走得费力（精确分解）但直奔终点（超线性收敛）——对于 WBC 这种几十维的小规模问题，"抄近路"显然更划算。假如没有近端正则项 $\sigma I$，精确求解策略在病态问题上会因为 KKT 矩阵接近奇异而数值崩溃——正则项的作用相当于给优化景观的最低点加了一层"安全垫"。

### 50.5.2 ProxQP 代码集成 ⭐⭐

```cpp
#include <proxsuite/proxqp/dense/dense.hpp>
using namespace proxsuite::proxqp;

// === 基础用法: 6 行解一个 QP ===

// 创建 QP 对象: n 变量, n_eq 等式约束, n_ineq 不等式约束
dense::QP<double> qp(n, n_eq, n_ineq);

// 设置精度
qp.settings.eps_abs = 1e-9;
qp.settings.max_iter = 100;

// 初始化并求解 (所有参数都是 Eigen 矩阵直接传入!)
qp.init(H, g, A, b, C, l, u);
qp.solve();

// 获取结果
Eigen::VectorXd x_opt = qp.results.x;      // primal 解
Eigen::VectorXd y_opt = qp.results.y;      // 等式对偶变量
Eigen::VectorXd z_opt = qp.results.z;      // 不等式对偶变量
double obj_val = qp.results.info.objValue;  // 目标函数值
```

**Warm-start（连续 MPC 场景）**：
```cpp
// 后续求解周期: 更新参数，ProxQP 自动利用上次结果
qp.update(H_new, g_new, A_new, b_new, C_new, l_new, u_new);
qp.solve();  // 自动 warm-start，通常 2-5 次迭代收敛

// 或者只更新部分参数（更高效）
qp.update(std::nullopt, g_new,                  // 只更新 gradient
           std::nullopt, std::nullopt,
           std::nullopt, l_new, u_new);          // 和约束界
qp.solve();
```

**稀疏模式**（大规模 QP）：
```cpp
#include <proxsuite/proxqp/sparse/sparse.hpp>

// 稀疏 QP: 输入是 Eigen::SparseMatrix
sparse::QP<double, int> qp_sparse(n, n_eq, n_ineq);
qp_sparse.init(H_sparse, g, A_sparse, b, C_sparse, l, u);
qp_sparse.solve();
```

### 50.5.3 ProxQP 在 WBC 中的集成（pinocchio 生态） ⭐⭐

**pinocchio + ProxQP 天然集成**（同属 Simple Robotics / INRIA 生态）：

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

class ProxQPWholeBodyController {
public:
    ProxQPWholeBodyController(const pinocchio::Model& model)
        : model_(model), data_(model),
          qp_(n_var_, n_eq_, n_ineq_) {
        // 初始化 QP 设置
        qp_.settings.eps_abs = 1e-6;
        qp_.settings.max_iter = 50;
        qp_.settings.verbose = false;
    }
    
    struct WBCSolution {
        Eigen::VectorXd tau;      // 关节力矩
        Eigen::VectorXd f;        // 接触力
        Eigen::VectorXd lambda;   // 对偶变量（约束松紧度）
    };
    
    WBCSolution solve(const Eigen::VectorXd& q,
                      const Eigen::VectorXd& v,
                      const Eigen::VectorXd& a_des,
                      const std::vector<int>& contact_frames) {
        // 1. Pinocchio 计算动力学量
        pinocchio::computeAllTerms(model_, data_, q, v);
        
        // 2. 构建等式约束: M*a_des + nle = S'*tau + J_c'*f
        // nle = 非线性效应项 (Coriolis + gravity)
        Eigen::VectorXd nle = data_.nle;  // pinocchio 计算的 h(q, v)
        Eigen::MatrixXd M = data_.M;       // 质量矩阵
        
        // 等式约束右端项
        Eigen::VectorXd b_eq = M * a_des + nle;  // 取浮动基座的6行
        
        // 3. 构建 QP 并求解
        buildQPMatrices(contact_frames, b_eq);
        qp_.update(H_, g_, A_eq_, b_eq_6_, C_ineq_, l_ineq_, u_ineq_);
        qp_.solve();
        
        // 4. 提取结果
        WBCSolution sol;
        sol.tau = qp_.results.x.head(n_joints_);
        sol.f = qp_.results.x.tail(n_force_);
        sol.lambda = qp_.results.z;  // 不等式对偶变量
        return sol;
    }
    
private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    proxsuite::proxqp::dense::QP<double> qp_;
    
    static constexpr int n_joints_ = 12;
    static constexpr int n_force_ = 12;     // 4 脚 * 3D
    static constexpr int n_var_ = 24;
    static constexpr int n_eq_ = 6;         // 浮动基座动力学
    static constexpr int n_ineq_ = 20;      // 摩擦锥
    
    Eigen::MatrixXd H_, A_eq_, C_ineq_;
    Eigen::VectorXd g_, b_eq_6_, l_ineq_, u_ineq_;
    
    void buildQPMatrices(const std::vector<int>& contacts,
                         const Eigen::VectorXd& dynamics_rhs) {
        // H = diag(W_tau, W_f): 力矩和力的权重矩阵
        // A_eq = [S' | J_c'] 的浮动基座行 (6 x 24)
        // C_ineq: 摩擦锥线性化约束矩阵 (20 x 24)
        // 足式/90_WBC分层优化与TSID WBC 章节将详细展开这一构建过程：H 矩阵由力矩跟踪权重
        // 和接触力正则化权重组成，A_eq 的前 6 行对应浮动基座的
        // Newton-Euler 方程（欠驱动约束），C_ineq 编码每只接触脚的
        // 线性化摩擦锥（4 或 8 面体近似）。
    }
};
```

### 50.5.4 HPIPM：MPC 结构化 QP 的王者 ⭐⭐⭐

**核心洞察**：MPC 的 QP 有极强的结构——KKT 系统是 **block-tridiagonal** 的。

**MPC 的数学形式**（Horizon $N$、状态 $n_x$、输入 $n_u$）：

$$\begin{aligned}
\min_{\{x_k, u_k\}} \quad & \sum_{k=0}^{N-1} \left(\frac{1}{2} x_k^T Q_k x_k + \frac{1}{2} u_k^T R_k u_k + q_k^T x_k + r_k^T u_k\right) + \frac{1}{2} x_N^T Q_N x_N \\
\text{s.t.} \quad & x_{k+1} = A_k x_k + B_k u_k + b_k \quad (k = 0, \dots, N-1) \\
& x_0 = \bar{x}_0 \\
& \underline{u}_k \le u_k \le \bar{u}_k \\
& \underline{x}_k \le x_k \le \bar{x}_k \\
& C_k x_k + D_k u_k \le d_k \quad \text{(路径约束)}
\end{aligned}$$

**展开成大 QP 的结构**：

```
通用稀疏 QP 的 KKT 矩阵:        MPC-QP 的 KKT 矩阵:
┌─ ─ ─ ─ ─ ─ ─ ─┐              ┌─────┬─────┬─────┬───┐
│ * * . . * . . * │              │ K_0 │ E_0'│     │   │
│ * * * . . . * . │              ├─────┼─────┼─────┤   │
│ . * * * . * . . │              │ E_0 │ K_1 │ E_1'│   │
│ . . * * * . . . │              │     ├─────┼─────┼───┤
│ * . . * * * . . │              │     │ E_1 │ K_2 │...│
│ . . * . * * * . │              │     │     ├─────┼───┤
│ . * . . . * * * │              │     │     │ ... │K_N│
│ * . . . . . * * │              └─────┴─────┴─────┴───┘
└─ ─ ─ ─ ─ ─ ─ ─┘              Block-tridiagonal!
任意稀疏 → 通用 LDL 分解         → Riccati 递推 O(N*(nx+nu)^3)
```

**HPIPM 的 Riccati 递推**利用这个结构：

- 通用稀疏 QP 分解：$O(N^2 \cdot (n_x+n_u)^3)$ 或更差
- HPIPM Riccati：$O(N \cdot (n_x+n_u)^3)$ — **线性于 horizon 长度**

**数值例子**：$N=20$, $n_x=12$, $n_u=6$
- 展开的大 QP 维度：$n = N \cdot (n_x + n_u) = 20 \times 18 = 360$ 变量
- OSQP 处理 360 维稀疏 QP：约 2-5 ms
- HPIPM 利用结构：约 0.1-0.5 ms — **10-50 倍加速**

### 50.5.5 HPIPM 集成模式 ⭐⭐⭐

| 集成方式 | 适用场景 | 抽象层次 | 代码量 |
|---------|---------|---------|-------|
| **acados** (Python/C) | 完整 NMPC 流水线 | 高 | 最少 |
| **OCS2** (C++) | 腿足全身 MPC | 高 | 中等 |
| **直接调用** (C) | 嵌入式定制 | 低 | 最多 |
| **hpipm-python** | 研究原型 | 中 | 少 |

**OCS2 的 HpipmInterface 简化流程**：

```cpp
// 文件: ocs2_sqp/src/HpipmInterface.cpp (简化展示)

class HpipmInterface {
public:
    struct OcpQpData {
        std::vector<Eigen::MatrixXd> A, B;   // 动力学 x_{k+1} = A_k x_k + B_k u_k
        std::vector<Eigen::MatrixXd> Q, R;   // 代价 Hessian
        std::vector<Eigen::VectorXd> q, r;   // 代价 gradient
        std::vector<Eigen::VectorXd> x_lb, x_ub, u_lb, u_ub;  // 盒约束
    };
    
    struct OcpQpSolution {
        std::vector<Eigen::VectorXd> delta_x;  // 状态增量
        std::vector<Eigen::VectorXd> delta_u;  // 控制增量
    };
    
    OcpQpSolution solve(const OcpQpData& data, int N) {
        // 1. 设置 HPIPM dimensions
        int dim_size = d_ocp_qp_dim_memsize(N);
        // ... 分配内存 (HPIPM 使用自定义内存管理)
        
        // 2. 填充 HPIPM 的 d_ocp_qp 结构
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_A(k, data.A[k].data(), &qp_);
            d_ocp_qp_set_B(k, data.B[k].data(), &qp_);
            d_ocp_qp_set_Q(k, data.Q[k].data(), &qp_);
            d_ocp_qp_set_R(k, data.R[k].data(), &qp_);
            // ... bounds
        }
        
        // 3. 调用 HPIPM IPM 求解
        d_ocp_qp_ipm_solve(&qp_, &sol_, &arg_, &ws_);
        
        // 4. 提取解
        OcpQpSolution result;
        for (int k = 0; k <= N; ++k) {
            result.delta_x.push_back(extractState(k));
            if (k < N) result.delta_u.push_back(extractControl(k));
        }
        return result;
    }
    
private:
    struct d_ocp_qp qp_;
    struct d_ocp_qp_sol sol_;
    struct d_ocp_qp_ipm_arg arg_;
    struct d_ocp_qp_ipm_ws ws_;
};
```

### 50.5.6 BLASFEO：HPIPM 的高性能后端 ⭐⭐⭐

HPIPM 的速度秘诀不仅在于算法（Riccati），还在于底层线性代数库 **BLASFEO**：

| 特性 | 标准 BLAS (OpenBLAS/MKL) | BLASFEO |
|------|-------------------------|---------|
| 目标规模 | 大矩阵 (n>100) | **小矩阵** (n=4~50) |
| 内存布局 | 列优先 (column-major) | **panel-major** (cache-line 对齐) |
| 函数调用 | 通用接口 | **内联 + 手写汇编** |
| 启动开销 | 较大 (参数检查) | **极小** |
| 适用场景 | 科学计算 | **嵌入式实时控制** |

**为什么标准 BLAS 不够快？** MPC 中的矩阵很小（典型 12x12 或 18x18），标准 BLAS 的函数调用开销和参数检查占据了大部分时间。BLASFEO 专门为这种"小矩阵密集运算"优化，在 ARM Cortex-A 和 x86 上都有手写 SIMD 内核。

### 50.5.7 ProxQP vs HPIPM 选型总结 ⭐⭐

| 场景 | 推荐 | 原因 |
|------|------|------|
| WBC (单步 QP, n<100) | **ProxQP** | 密集 QP 最快，Eigen 原生 |
| IK (单步 QP, n<50) | **ProxQP** | 同上 |
| 线性 MPC (N=20, nx=12, nu=6) | **HPIPM** | 结构利用，10x 加速 |
| 非线性 MPC (SQP 内层) | **HPIPM** (via acados) | OCP 结构 + SQP-RTI |
| 通用 QP + 最大社区支持 | **OSQP** | 文档最全，debugging 容易 |
| 嵌入式 (无 malloc) | **PIQP** | Header-only, allocation-free |

> ⚠️ **常见陷阱**：HPIPM **只能解 OCP 结构的 QP**。如果你的 QP 不具备时域阶段结构（例如纯 WBC 单步优化），HPIPM 无法利用任何结构，性能反而不如 ProxQP。**结构利用是双刃剑——用错了反而更慢**。

**练习 50.6** ⭐⭐⭐：对一个 horizon=20, nx=12, nu=6 的线性 MPC 问题：
1. 如果用 OSQP 展开成大 QP，决策变量和约束各多少维？
2. HPIPM Riccati 递推的 flops 约为 $N \cdot (n_x+n_u)^3 = 20 \times 18^3 \approx$ 多少？
3. 通用 LDL 分解 360 维稀疏 QP 的 flops 量级？两者比值约多少？

---

## 50.6 NLP求解器：Ipopt与acados ⭐⭐⭐

### 50.6.1 从 QP 到 NLP：为什么需要 NLP？ ⭐⭐

QP 的限制：目标函数必须是二次的，约束必须是线性的。但很多机器人问题的约束**天然非线性**：

| 非线性来源 | 数学形式 | 为什么不能线性化？ |
|-----------|---------|-----------------|
| 正运动学 FK(q) | $T_{ee} = \prod T_i(q_i)$ | 连续关节角的三角函数链 |
| 逆动力学 | $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$ | M(q) 依赖配置 |
| 摩擦锥（精确） | $\sqrt{f_x^2 + f_y^2} \le \mu f_z$ | 二次锥约束 |
| 角动量 | $L = I(q)\omega$ | 惯性张量依赖配置 |

**解决方案**：使用通用 NLP 求解器，或者在 NLP 外层做 SQP（Sequential QP）迭代——每步线性化为 QP 子问题。SQP 的思想类似于用"逐段直线拟合曲线"——在当前点用 QP（直线/二次面）近似原来的非线性问题，求解后移到新的点再做一次近似，反复迭代直到收敛。假如不做这种逐步线性化而试图一次性求解完整的非线性问题，求解器很容易被复杂的非凸景观困住，陷入极差的局部最优。

> **本质洞察**：QP 与 NLP 的关系不是"简单 vs 复杂"，而是"全局 vs 局部"。QP 是凸的，KKT 条件给出全局最优解——这是工程师可以"闭眼信任"的结果。NLP 是非凸的，只能保证局部最优——初始猜测的好坏直接决定解的质量。这就是为什么 MPC 中尽量把问题建模为 QP（通过线性化动力学、线性化摩擦锥）：不是因为 QP 更"精确"（它其实是近似），而是因为 QP 的解是可靠的。

### 50.6.2 Ipopt：通用 NLP 的工业标准 ⭐⭐

**Ipopt**（Interior Point OPTimizer）是最广泛使用的开源 NLP 求解器。

**算法概要**：

$$\min_x f(x) \quad \text{s.t.} \quad g(x) = 0, \; h(x) \le 0$$

1. 把不等式约束转化为等式 + 松弛变量：$h(x) + s = 0$, $s \ge 0$
2. 用 barrier 函数替代 $s \ge 0$：$\min f(x) - \mu \sum \ln(s_i)$
3. 对 barrier 问题求 KKT 系统（Newton 步）
4. 逐步减小 $\mu \to 0$，解收敛到原问题最优

**Ipopt 需要用户提供**：
- $f(x)$ 的值和梯度
- $g(x)$, $h(x)$ 的值和 Jacobian
- Lagrangian 的 Hessian（或其近似）

**线性求解器选择**（Ipopt 最关键的性能因素）：

| 线性求解器 | License | 性能 | 适用场景 |
|-----------|---------|------|---------|
| **MUMPS** | 免费 | 中等 | 默认选择、教学 |
| **MA27** | HSL 学术免费 | 良好 | 稀疏对称不定 |
| **MA57** | HSL 学术免费 | **优秀** | 推荐生产环境 |
| **MA86/97** | HSL 学术免费 | 卓越 | 大规模并行 |
| **Pardiso** | Intel MKL | 卓越 | 有 MKL 就用 |

> ⚠️ **常见陷阱**：Ipopt 默认用 MUMPS 线性求解器，性能比 MA57 慢 2-5 倍。对于任何严肃的机器人轨迹优化，**必须安装 HSL**（学术用途免费申请）。

### 50.6.3 acados：实时 NLP 求解 ⭐⭐⭐

**acados** 是专为实时 MPC/OCP 设计的框架，核心特性：

| 特性 | Ipopt | acados |
|------|-------|--------|
| 目标问题 | 通用 NLP | **OCP 结构化 NLP** |
| 算法 | Interior-point | **SQP / SQP-RTI** |
| QP 子问题 | 内部 | **HPIPM**（外部高性能） |
| 代码生成 | 无 | **CasADi → C 代码** |
| 实时性 | 非实时 | **硬实时** (< 1ms) |
| 语言 | C/Fortran | C + Python 接口 |

### 50.6.4 SQP-RTI 方案 ⭐⭐⭐

**SQP**（Sequential Quadratic Programming）：在每次迭代中，将 NLP 线性化为 QP 子问题并求解。

**SQP-RTI**（Real-Time Iteration）：只做**一步 SQP 迭代**就输出控制，下个周期继续迭代。

```
传统 SQP (离线):
  while not converged:
    线性化 NLP → QP
    求解 QP → Δx
    更新 x ← x + Δx
  end

SQP-RTI (实时):
  每个控制周期:
    Preparation phase: 线性化 NLP → QP (可以提前做)
    Feedback phase:    收到新状态 x_0 → 更新 QP 初始条件 → 求解 QP → 输出 u_0
  (只做 1 步 SQP，下个周期继续 "收敛")
```

**为什么 RTI 能工作？** 在连续运行的 MPC 中，问题变化缓慢。上一周期的解已经接近最优，一步 SQP 就足够追踪最优解的漂移。这被称为**实时逼近**（Real-Time Approximation）。

### 50.6.5 acados 完整示例：倒立摆 MPC ⭐⭐

```python
# acados Python 接口 (高层建模 → 自动生成 C 代码)
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np

# 1. CasADi 建模
x = ca.SX.sym('x', 4)      # [位置, 角度, 速度, 角速度]
u = ca.SX.sym('u', 1)      # 力

# 倒立摆动力学 (连续时间)
m_c, m_p, l, g_val = 1.0, 0.1, 0.8, 9.81
sin_th = ca.sin(x[1])
cos_th = ca.cos(x[1])
denom = m_c + m_p * sin_th**2

x_dot = ca.vertcat(
    x[2],
    x[3],
    (u[0] + m_p * sin_th * (l * x[3]**2 + g_val * cos_th)) / denom,
    (-u[0] * cos_th - m_p * l * x[3]**2 * sin_th * cos_th
     - (m_c + m_p) * g_val * sin_th) / (l * denom)
)

# 2. 构建 acados OCP
model = AcadosModel()
model.name = 'pendulum'
model.x = x
model.u = u
model.f_expl_expr = x_dot

ocp = AcadosOcp()
ocp.model = model
ocp.dims.N = 20                    # horizon
ocp.cost.cost_type = 'LINEAR_LS'   # 线性最小二乘代价
ocp.cost.W = np.diag([10, 100, 1, 1, 0.1])  # Q + R 权重

# LINEAR_LS 代价需要的矩阵: ||Vx @ x + Vu @ u - yref||_W^2
nx = 4; nu = 1; ny = nx + nu
ocp.cost.Vx = np.eye(ny, nx)                    # 状态选择矩阵
ocp.cost.Vu = np.zeros((ny, nu))
ocp.cost.Vu[nx:, :] = np.eye(nu)                # 输入选择矩阵
ocp.cost.yref = np.array([0, 0, 0, 0, 0])       # 参考输出 (x_ref, u_ref)

# 终端代价
ocp.cost.cost_type_e = 'LINEAR_LS'
ocp.cost.W_e = np.diag([10, 100, 1, 1])
ocp.cost.Vx_e = np.eye(nx)
ocp.cost.yref_e = np.array([0, 0, 0, 0])

# 约束
ocp.constraints.lbu = np.array([-40.0])     # 力矩下界
ocp.constraints.ubu = np.array([40.0])      # 力矩上界
ocp.constraints.idxbu = np.array([0])       # 受约束的输入索引
ocp.constraints.x0 = np.array([0, 3.14, 0, 0])  # 初始状态

# 求解器配置
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # QP 后端!
ocp.solver_options.nlp_solver_type = 'SQP_RTI'             # RTI 方案!
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.tf = 2.0                                # 预测时域 2s

# 3. 生成 C 代码并编译
solver = AcadosOcpSolver(ocp, json_file='pendulum.json')

# 4. 在线求解 (每个控制周期调用)
for step in range(500):
    # 获取新状态
    x_current = get_state()
    
    # 设置初始状态约束
    solver.set(0, 'lbx', x_current)
    solver.set(0, 'ubx', x_current)
    
    # RTI 求解 (< 1ms)
    status = solver.solve()
    
    # 提取第一个控制量
    u_opt = solver.get(0, 'u')
    apply_control(u_opt)
```

### 50.6.6 acados 内部结构 ⭐⭐⭐

```
acados 求解流程:
┌─────────────────────────────────────────────────┐
│ CasADi 符号模型                                  │
│   ↓ code generation                             │
│ C 代码 (动力学/代价/约束 及其导数)                │
│   ↓                                             │
│ ┌─────────────────────────────────────────────┐ │
│ │ SQP / SQP-RTI 外层                          │ │
│ │   每步:                                      │ │
│ │   1. 积分 (ERK/IRK) → 离散化动力学            │ │
│ │   2. 线性化 → 构建 OCP-QP                    │ │
│ │   3. 求解 QP:                                │ │
│ │      ┌───────────────────────────────────┐  │ │
│ │      │ HPIPM (Riccati IPM)               │  │ │
│ │      │ 或 PARTIAL_CONDENSING + HPIPM     │  │ │
│ │      │ 或 FULL_CONDENSING + qpOASES      │  │ │
│ │      └───────────────────────────────────┘  │ │
│ │   4. 线搜索 + 更新                          │ │
│ └─────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────┘
```

**Condensing 策略**：
- **Full condensing**：消去状态变量，只剩输入 $u_0, ..., u_{N-1}$ → 密集 QP → qpOASES
- **Partial condensing**：部分消去 → 中等规模 QP → HPIPM
- **No condensing**：保留全部变量 → 大稀疏 QP → HPIPM 原生

| 策略 | QP 维度 | 求解器 | 适用场景 |
|------|---------|--------|---------|
| Full condensing | $N \cdot n_u$ (小) | qpOASES | 短 horizon, 少输入 |
| Partial condensing | 中等 | HPIPM | **通用推荐** |
| No condensing | $N \cdot (n_x+n_u)$ (大) | HPIPM | 长 horizon, 结构利用 |

> ⚠️ **常见陷阱**：acados 的 `SQP_RTI` 只做 **1 步 SQP**，不保证收敛到最优！在初始化阶段（问题远离最优时），应该先用 `SQP`（多步）求解一次完整解，然后切换到 RTI 进行在线跟踪。

**练习 50.7** ⭐⭐⭐：解释为什么 acados 使用 HPIPM 作为 QP 后端，而不是 ProxQP 或 OSQP。从算法结构和性能两个角度回答。

---

## 50.7 CasADi符号框架 ⭐⭐⭐

### 50.7.1 CasADi vs CppAD：两种自动微分哲学 ⭐⭐

| 维度 | CasADi | CppAD (足式/40_CppAD与代码生成) |
|------|--------|-------------|
| **微分方式** | 符号 DAG（有向无环图） | 运行时 tape 记录 |
| **建模时机** | **离线**构建表达式 | **在线**执行记录 |
| **优势** | 符号简化、稀疏性分析、代码生成 | 零运行时开销、与 C++ 原生集成 |
| **语言** | C++ / **Python** (主力) / MATLAB | **C++** only |
| **典型用户** | 控制研究者、acados | Pinocchio、OCS2、Crocoddyl |
| **代码生成** | `Function::generate()` → C | CppADCodeGen → C |
| **集成方式** | 独立框架 | 嵌入现有 C++ 代码 |

**核心区别图解**：

```
CppAD (tape-based):
  1. 写 C++ 函数 f(x)
  2. 用 AD<double> 类型执行一次 → 记录 "tape"
  3. tape.Forward(0, x_val) → 求值
  4. tape.Jacobian(x_val)  → 求导

CasADi (symbolic):
  1. 声明符号变量 x = SX.sym('x', n)
  2. 构建表达式 f = sin(x[0]) + x[1]^2  (不执行，只建图)
  3. F = Function('F', [x], [f])           (封装为函数对象)
  4. F(x_val)            → 求值
  5. F.jacobian()        → 自动构建导数函数
  6. F.generate('f.c')   → 导出 C 代码
```

### 50.7.2 SX vs MX：何时用哪个？ ⭐⭐

CasADi 有两种符号类型：

| 特性 | SX (Scalar eXpression) | MX (Matrix eXpression) |
|------|----------------------|----------------------|
| 粒度 | 标量级运算 | 矩阵级运算 |
| 表达式树 | 深（每个加/乘是一个节点） | 浅（矩阵乘法是一个节点） |
| 代码生成 | **极致优化**（展开一切） | 调用 BLAS 内核 |
| 适用场景 | 小模型、嵌入式部署 | 大模型、调用外部函数 |
| 控制流 | 不支持 if/for | 支持（通过 callback） |

**规则**：
- 维度 < 50 的问题 → **SX**（生成的代码最快）
- 维度 > 50 或需要调用外部函数（如 Pinocchio）→ **MX**
- 需要嵌入式部署 → **SX + 代码生成**

### 50.7.3 CasADi 完整 NLP 示例 ⭐⭐

```cpp
#include <casadi/casadi.hpp>
using namespace casadi;

int main() {
    // === 7-DOF 臂轨迹优化 (简化版) ===
    // min Σ ||q_k - q_ref||² + w * Σ ||τ_k||²
    // s.t. 动力学: M(q)·qddot + C(q,qdot)·qdot + g(q) = τ
    //      关节限: q_min ≤ q ≤ q_max
    //      力矩限: τ_min ≤ τ ≤ τ_max
    //      初末条件: q(0) = q_start, q(T) = q_goal
    
    int N = 50;          // 时间步数
    int nq = 7;          // 关节数
    double dt = 0.02;    // 时间步长
    
    // 符号变量
    SX q = SX::sym("q", nq);
    SX qdot = SX::sym("qdot", nq);
    SX tau = SX::sym("tau", nq);
    
    // 简化动力学 (实际应用中可调用 Pinocchio 或 CasADi 内建)
    // qddot = M^{-1} * (tau - C*qdot - g)
    SX M = SX::eye(nq) * 2.0;   // 简化: 常数质量矩阵
    SX C_mat = SX::zeros(nq, nq);
    SX g_vec = SX::zeros(nq, 1);
    g_vec(0) = 9.81 * 0.5;      // 只有第一关节有重力项
    
    SX qddot = mtimes(inv(M), tau - mtimes(C_mat, qdot) - g_vec);
    
    // 离散化动力学 (Euler)
    SX q_next = q + qdot * dt;
    SX qdot_next = qdot + qddot * dt;
    
    // 构建 NLP
    std::vector<SX> w, w0_vec, lbw_vec, ubw_vec;  // 决策变量
    std::vector<SX> g_con, lbg_vec, ubg_vec;       // 约束
    SX J = 0;                                       // 目标函数
    
    // 参考轨迹
    SX q_ref = SX::zeros(nq, 1);
    q_ref(0) = 1.0;  // 目标关节角
    
    // 初始状态变量
    SX q_k = SX::sym("q_0", nq);
    SX qdot_k = SX::sym("qdot_0", nq);
    w.push_back(q_k);
    w.push_back(qdot_k);
    
    // 初始条件约束
    g_con.push_back(q_k);
    // lbg = ubg = q_start (稍后设置数值)
    
    for (int k = 0; k < N; ++k) {
        // 控制变量
        SX tau_k = SX::sym("tau_" + std::to_string(k), nq);
        w.push_back(tau_k);
        
        // 目标函数: 跟踪误差 + 力矩惩罚
        SX q_err = q_k - q_ref;
        J += mtimes(q_err.T(), q_err) + 0.01 * mtimes(tau_k.T(), tau_k);
        
        // 动力学约束 (射击法 shooting)
        Function f_dyn("f", {q, qdot, tau}, {q_next, qdot_next});
        std::vector<SX> dyn_result = f_dyn({q_k, qdot_k, tau_k});
        
        // 下一步状态变量
        SX q_next_k = SX::sym("q_" + std::to_string(k+1), nq);
        SX qdot_next_k = SX::sym("qdot_" + std::to_string(k+1), nq);
        w.push_back(q_next_k);
        w.push_back(qdot_next_k);
        
        // 连续性约束: x_{k+1} = f(x_k, u_k)
        g_con.push_back(q_next_k - dyn_result[0]);
        g_con.push_back(qdot_next_k - dyn_result[1]);
        
        q_k = q_next_k;
        qdot_k = qdot_next_k;
    }
    
    // 终端代价
    SX q_err_N = q_k - q_ref;
    J += 100 * mtimes(q_err_N.T(), q_err_N);
    
    // 打包 NLP
    SX w_all = vertcat(w);
    SX g_all = vertcat(g_con);
    
    SXDict nlp = {{"x", w_all}, {"f", J}, {"g", g_all}};
    
    // 创建 Ipopt 求解器
    Dict opts;
    opts["ipopt.linear_solver"] = "mumps";
    opts["ipopt.max_iter"] = 500;
    opts["ipopt.print_level"] = 3;
    Function solver = nlpsol("solver", "ipopt", nlp, opts);
    
    // 设置约束界并求解
    DMDict arg;
    arg["lbg"] = DM::zeros(g_all.size1(), 1);   // 等式约束 = 0
    arg["ubg"] = DM::zeros(g_all.size1(), 1);
    // arg["lbx"], arg["ubx"] 设置关节和力矩限
    
    DMDict result = solver(arg);
    std::cout << "Optimal cost: " << result["f"] << std::endl;
    
    return 0;
}
```

### 50.7.4 CasADi 代码生成 ⭐⭐⭐

CasADi 最强大的功能之一是将符号表达式导出为纯 C 代码：

```python
import casadi as ca

# Python 中建模（快速原型）
x = ca.SX.sym('x', 7)
f = ca.sin(x[0]) * x[1] + x[2]**2

# 创建函数
F = ca.Function('my_func', [x], [f, ca.jacobian(f, x)])

# 生成 C 代码
F.generate('my_func.c', {'with_header': True})
# 生成文件: my_func.c, my_func.h
# 编译: gcc -shared -O2 -o my_func.so my_func.c
```

**生成的 C 代码特点**：
- 零依赖（不需要 CasADi 运行时库）
- 可直接嵌入嵌入式系统
- 所有矩阵运算展开为标量操作（SX 模式）
- 自动计算稀疏性并跳过零元素

### 50.7.5 CasADi 与各求解器的集成 ⭐⭐

```
CasADi 符号模型
    │
    ├── nlpsol("ipopt", ...) → Ipopt 求解 NLP
    ├── nlpsol("sqpmethod", ...) → 内置 SQP
    ├── qpsol("osqp", ...) → OSQP 求解 QP
    ├── qpsol("qpoases", ...) → qpOASES
    │
    └── acados export → 生成 C 代码 → acados 实时求解
            │
            └── QP subproblem → HPIPM
```

### 50.7.6 选型：CasADi vs Ifopt vs 直接调用求解器 ⭐⭐

| 维度 | CasADi | Ifopt | 直接 API |
|------|--------|-------|----------|
| 建模速度 | **最快**（Python 原型） | 中等（C++ 类） | 最慢（手写矩阵） |
| 自动微分 | ✅符号 | ❌手写 Jacobian | ❌手写 |
| 代码生成 | ✅ | ❌ | ❌ |
| 运行时性能 | 中等 | 良好 | **最快** |
| 调试难度 | 高（符号图难 debug） | **低**（C++ 原生） | 中等 |
| 适合团队 | 研究组 | 工程团队 | 嵌入式团队 |
| 代表项目 | bipedal_locomotion, acados | **TOWR** | OCS2 |

> ⚠️ **常见陷阱**：CasADi 的 MX 类型**不能在 for 循环中展开**（那样会创建巨大的表达式图）。对于 horizon=100 的 MPC，必须用 CasADi 的 `map()` 或 `fold()` 函数来高效构建。否则模型编译时间可能达到分钟级。

**练习 50.8** ⭐⭐⭐：用 CasADi Python 接口实现 Rosenbrock 函数 $\min (1-x)^2 + 100(y-x^2)^2$ 的 NLP 求解（调用 Ipopt）。对比代码行数和 Ifopt 版本的差异。

---

## 50.8 Ifopt建模框架 ⭐⭐

### 50.8.1 Ifopt 的设计哲学 ⭐⭐

**Ifopt**（ETH Zurich, Alexander Winkler）为 **TOWR 腿足轨迹优化**而生，核心设计原则：

1. **极简**：约 3000 行代码（vs CasADi 百万行）
2. **C++ 原生**：不需要外部语言/代码生成
3. **面向对象**：用继承分离 "变量/约束/代价"
4. **后端无关**：同一模型可以对接 Ipopt 或 SNOPT

### 50.8.2 三个核心抽象 ⭐⭐

```
┌─────────────────────────────────────────┐
│            ifopt::Problem               │
│                                         │
│  ┌───────────┐ ┌─────────────┐ ┌─────┐ │
│  │VariableSet│ │ConstraintSet│ │Cost │ │
│  │           │ │             │ │Term │ │
│  │ GetValues │ │ GetValues   │ │     │ │
│  │ SetValues │ │ GetBounds   │ │GetCo│ │
│  │ GetBounds │ │ FillJacobian│ │st   │ │
│  └───────────┘ └─────────────┘ └─────┘ │
│         │              │            │   │
│         └──────────────┼────────────┘   │
│                        ↓                │
│              Ipopt / SNOPT              │
└─────────────────────────────────────────┘
```

**关键接口**：

```cpp
// VariableSet: 决策变量的一个分组
class VariableSet : public Component {
    virtual VectorXd GetValues() const = 0;          // 返回当前值
    virtual void SetVariables(const VectorXd&) = 0;  // 被求解器更新
    virtual VecBound GetBounds() const = 0;          // 变量上下界
};

// ConstraintSet: 约束的一个分组
class ConstraintSet : public Component {
    virtual VectorXd GetValues() const = 0;           // 计算约束值 g(x)
    virtual VecBound GetBounds() const = 0;           // 约束界 [lb, ub]
    virtual void FillJacobianBlock(                   // 填充 Jacobian 子块
        std::string var_set_name,
        Jacobian& jac) const = 0;
};

// CostTerm: 标量代价
class CostTerm : public Component {
    virtual double GetCost() const = 0;               // 计算代价值 f(x)
    virtual void FillJacobianBlock(                   // 代价梯度
        std::string var_set_name,
        Jacobian& jac) const = 0;
};
```

### 50.8.3 Ifopt 完整示例：Rosenbrock 问题 ⭐⭐

$$\min_{x,y} (1-x)^2 + 100(y-x^2)^2 \quad \text{s.t.} \quad x^2 + y^2 \le 2$$

```cpp
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>

// === 变量 ===
class RosenbrockVars : public ifopt::VariableSet {
public:
    RosenbrockVars() : VariableSet(2, "xy") {
        x_ = Eigen::Vector2d(-1.0, 1.0);  // 初始猜测
    }
    
    VectorXd GetValues() const override { return x_; }
    void SetVariables(const VectorXd& v) override { x_ = v; }
    
    VecBound GetBounds() const override {
        return {ifopt::Bounds(-2.0, 2.0),   // x 范围
                ifopt::Bounds(-2.0, 2.0)};  // y 范围
    }
private:
    Eigen::Vector2d x_;
};

// === 约束: x² + y² ≤ 2 ===
class CircleConstraint : public ifopt::ConstraintSet {
public:
    CircleConstraint() : ConstraintSet(1, "circle") {}
    
    VectorXd GetValues() const override {
        auto v = GetVariables()->GetComponent("xy")->GetValues();
        Eigen::VectorXd c(1);
        c[0] = v[0]*v[0] + v[1]*v[1];  // x² + y²
        return c;
    }
    
    VecBound GetBounds() const override {
        return {ifopt::Bounds(-ifopt::inf, 2.0)};  // ≤ 2
    }
    
    void FillJacobianBlock(std::string var_name, Jacobian& jac) const override {
        if (var_name == "xy") {
            auto v = GetVariables()->GetComponent("xy")->GetValues();
            jac.coeffRef(0, 0) = 2.0 * v[0];  // d(x²+y²)/dx = 2x
            jac.coeffRef(0, 1) = 2.0 * v[1];  // d(x²+y²)/dy = 2y
        }
    }
};

// === 代价: (1-x)² + 100(y-x²)² ===
class RosenbrockCost : public ifopt::CostTerm {
public:
    RosenbrockCost() : CostTerm("rosenbrock") {}
    
    double GetCost() const override {
        auto v = GetVariables()->GetComponent("xy")->GetValues();
        double x = v[0], y = v[1];
        return (1-x)*(1-x) + 100*(y-x*x)*(y-x*x);
    }
    
    void FillJacobianBlock(std::string var_name, Jacobian& jac) const override {
        if (var_name == "xy") {
            auto v = GetVariables()->GetComponent("xy")->GetValues();
            double x = v[0], y = v[1];
            // df/dx = -2(1-x) + 100*2*(y-x²)*(-2x)
            jac.coeffRef(0, 0) = -2.0*(1-x) - 400.0*x*(y - x*x);
            // df/dy = 100*2*(y-x²)
            jac.coeffRef(0, 1) = 200.0*(y - x*x);
        }
    }
};

// === 组装并求解 ===
int main() {
    ifopt::Problem nlp;
    nlp.AddVariableSet(std::make_shared<RosenbrockVars>());
    nlp.AddConstraintSet(std::make_shared<CircleConstraint>());
    nlp.AddCostSet(std::make_shared<RosenbrockCost>());
    
    ifopt::IpoptSolver solver;
    solver.SetOption("linear_solver", "mumps");
    solver.SetOption("jacobian_approximation", "exact");  // 用我们提供的 Jacobian
    solver.Solve(nlp);
    
    Eigen::VectorXd x_opt = nlp.GetOptVariables()->GetValues();
    std::cout << "x* = " << x_opt[0] << ", y* = " << x_opt[1] << std::endl;
    // 预期: x* ≈ 1.0, y* ≈ 1.0 (Rosenbrock 最小值在 (1,1))
    return 0;
}
```

### 50.8.4 TOWR 如何用 Ifopt 建模腿足轨迹优化 ⭐⭐⭐

TOWR 的 NLP 变量分组（每个是一个 `VariableSet`）：

| 变量组 | 维度 | 物理含义 |
|--------|------|---------|
| `base_linear` | 6N | 基座线性轨迹（位置 spline 系数） |
| `base_angular` | 6N | 基座角度轨迹 |
| `ee_motion_0..3` | 各 3N | 每条腿末端轨迹 |
| `ee_force_0..3` | 各 3N | 每条腿接触力 |
| `ee_timing_0..3` | 各 K | 接触时序（步态参数） |

TOWR 的约束分组（每个是一个 `ConstraintSet`）：

| 约束组 | 含义 |
|--------|------|
| `dynamic` | SRBD 动力学：$m\ddot{c} = \sum f_i + mg$ |
| `range_of_motion` | 足端相对基座的运动范围 |
| `force` | 接触力只在接触相有效 |
| `swing` | 摆动相足端离地 |
| `terrain` | 足端不穿透地形 |

**这种分组设计的价值**：每个约束的 Jacobian 只对"相关的变量组"有非零值。Ifopt 自动组装块状稀疏 Jacobian，传给 Ipopt。

### 50.8.5 何时选 Ifopt vs CasADi vs 直接 API ⭐⭐

| 场景 | 推荐 | 原因 |
|------|------|------|
| C++ 项目，固定问题结构 | **Ifopt** | 代码清晰，debugging 容易 |
| 快速原型，频繁改模型 | **CasADi** (Python) | 符号微分 + 1 行改约束 |
| 需要代码生成部署 | **CasADi** → C | 生成零依赖 C 代码 |
| 已有 Pinocchio/OCS2 | **直接 API** | 避免额外依赖层 |
| 教学/理解 NLP 结构 | **Ifopt** | 手写 Jacobian 加深理解 |

> ⚠️ **常见陷阱**：Ifopt 的 `FillJacobianBlock` 要求你**手动计算并填充 Jacobian**。如果 Jacobian 写错了（符号错误、维度错误），Ipopt 不会报错，但会收敛到错误的解或根本不收敛。**一定要用有限差分验证你的解析 Jacobian**（Ipopt 有 `derivative_test` 选项）。

**练习 50.9** ⭐⭐：用 Ifopt 建模 7-DOF 臂 IK 问题（变量 = 关节角 q，约束 = 末端位置误差 < epsilon + 关节限位，代价 = 关节角变化量最小）。在 `FillJacobianBlock` 中调用 Pinocchio 的 `computeFrameJacobian`。

---

## 50.9 实战选型指南 ⭐⭐

### 50.9.1 决策树：从问题到求解器 ⭐⭐

```
┌─────────── 你的优化问题 ───────────┐
│                                     │
│  目标函数是否二次？约束是否线性？      │
│                                     │
├── YES (QP) ─────────────────────────┤
│                                     │
│  ├── 有 OCP/MPC 时域结构？           │
│  │   ├── YES → HPIPM                │
│  │   │   (via acados 或 OCS2)       │
│  │   │                              │
│  │   └── NO ↓                       │
│  │                                   │
│  ├── 实时性要求 < 100us?             │
│  │   ├── YES → ProxQP (dense)       │
│  │   └── NO ↓                       │
│  │                                   │
│  ├── 需要最大社区支持/文档？          │
│  │   ├── YES → OSQP                 │
│  │   └── NO ↓                       │
│  │                                   │
│  └── 嵌入式/header-only?            │
│      ├── YES → PIQP                 │
│      └── NO → OSQP/ProxQP           │
│                                     │
├── NO (NLP) ─────────────────────────┤
│                                     │
│  ├── 需要实时 (< 5ms)?              │
│  │   ├── YES → acados (SQP-RTI)    │
│  │   └── NO ↓                       │
│  │                                   │
│  ├── 问题规模 > 10000 变量?          │
│  │   ├── YES → Ipopt + HSL          │
│  │   └── NO ↓                       │
│  │                                   │
│  ├── 需要代码生成?                   │
│  │   ├── YES → CasADi + Ipopt      │
│  │   └── NO ↓                       │
│  │                                   │
│  ├── 已有 C++ 代码库?               │
│  │   ├── YES → Ifopt + Ipopt       │
│  │   └── NO → CasADi (Python)      │
│                                     │
└─────────────────────────────────────┘
```

### 50.9.2 按机器人类型的推荐 ⭐⭐

| 机器人类型 | 典型问题 | 推荐方案 | 代表项目 |
|-----------|---------|---------|---------|
| **四足步行** | WBC (1kHz) | ProxQP / OSQP | Unitree, ANYmal |
| **四足步行** | MPC (20-30Hz) | HPIPM via acados 或 qpOASES | ANYmal, MIT Cheetah3 |
| **四足步行** | 轨迹优化 | CasADi + Ipopt 或 Crocoddyl | 离线规划 |
| **双足** | WBC (1kHz) | ProxQP | TSID, pinocchio-WBC |
| **双足** | MPC (100Hz) | acados (SQP-RTI + HPIPM) | bipedal_locomotion |
| **7-DOF 臂** | IK (1kHz) | ProxQP (dense) | 工业臂控制 |
| **7-DOF 臂** | MPC (100Hz) | OSQP 或 acados | 协作臂 |
| **7-DOF 臂** | 轨迹优化 | Ifopt + Ipopt | TOWR 思路扩展 |
| **移动操作** | 全身 MPC | OCS2 (SQP + HPIPM) | Mobile Manipulation |
| **无人机** | 轨迹优化 | CasADi + Ipopt | Minimum-snap 扩展 |
| **无人机** | MPC (100Hz) | acados | PX4/Betaflight |

### 50.9.3 性能-开发效率-维护性三角 ⭐⭐

```
         性能
        /    \
       /  理想 \
      /  区域   \
     /___________\
  开发效率 ─────── 维护性

直接 API:    性能 ⭐⭐⭐  开发效率 ⭐    维护性 ⭐⭐
Ifopt:      性能 ⭐⭐    开发效率 ⭐⭐  维护性 ⭐⭐⭐
CasADi:     性能 ⭐⭐    开发效率 ⭐⭐⭐ 维护性 ⭐⭐
acados:     性能 ⭐⭐⭐  开发效率 ⭐⭐  维护性 ⭐⭐
```

### 50.9.4 常见错误选型案例 ⭐⭐

| 错误 | 表现 | 正确做法 |
|------|------|---------|
| 用 OSQP 解 MPC-QP | 慢 10 倍，且不稳定 | 用 HPIPM |
| 用 HPIPM 解 WBC | 构建 OCP 结构反而浪费时间 | 用 ProxQP |
| 用 Ipopt 解线性 MPC | 大炮打蚊子，太慢 | 用 HPIPM/acados |
| 用 qpOASES 解大规模 QP | 密集矩阵内存爆炸 | 用 OSQP (sparse) |
| 用 CasADi 部署但不做代码生成 | Python 运行时开销大 | 一定要 `.generate()` |
| Ifopt 不验证 Jacobian | 收敛到错误解 | 打开 `derivative_test` |

> ⚠️ **常见陷阱**：**不存在万能求解器**。每个求解器都有其最佳使用场景。把"ProxQP 最快"作为万金油使用是错误的——它在 MPC 结构问题上不如 HPIPM，在超大规模稀疏 QP 上不如 OSQP。**理解问题结构，匹配求解器**。

**练习 50.10** ⭐⭐：你正在为一个 12-DOF 双足机器人设计控制系统。需要：(1) 1kHz WBC 计算接触力和关节力矩；(2) 50Hz MPC 规划未来 1 秒轨迹；(3) 离线轨迹优化生成步态库。分别选择什么求解器？给出理由。

---

## 50.10 从 QP/NLP 到下游章节 ⭐

### 50.10.1 本章知识在后续章节的应用 ⭐

```
足式/60_QP_NLP建模 QP/NLP 建模基础
    │
    ├──→ 足式/80_接触力学与约束优化 接触力学与约束优化
    │      摩擦锥约束 → QP 的不等式约束 (本章 50.2)
    │      线性化摩擦锥 → 4 面体近似写入约束矩阵
    │
    ├──→ 足式/90_WBC分层优化与TSID WBC 分层优化与 TSID
    │      分层 QP → 多个 QP 级联求解 (ProxQP/OSQP)
    │      TSID = 加权 QP，结构同 50.4 WBC 示例
    │
    ├──→ 足式/100_DDP家族与Crocoddyl DDP 家族与 Crocoddyl
    │      DDP ≈ 无约束版的 MPC (每步解 Riccati)
    │      约束 DDP (FDDP/ProxDDP) → 本章 KKT 条件的推广
    │
    ├──→ 足式/110_OCS2完整栈与双线程MPC OCS2 完整栈
    │      OCS2 SQP → 外层 NLP，内层 HPIPM QP (50.5)
    │      HpipmInterface 直接对应 50.5.5 的代码
    │
    ├──→ S05 可微 MPC (研究前沿)
    │      Differentiable QP → 本章 KKT 条件求导
    │      OptNet / QPyTorch → QP 对偶变量的梯度
    │
    └──→ 全局: 任何需要"优化 + 物理约束"的机器人问题
```

### 50.10.2 具体衔接路径 ⭐

**足式/80_接触力学与约束优化 摩擦锥**：本章的线性不等式约束 $Cx \le d$ 在 足式/80_接触力学与约束优化 中具象化为：

$$\begin{bmatrix} 1 & 0 & -\mu \\ -1 & 0 & -\mu \\ 0 & 1 & -\mu \\ 0 & -1 & -\mu \end{bmatrix} \begin{bmatrix} f_x \\ f_y \\ f_z \end{bmatrix} \le \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

**足式/90_WBC分层优化与TSID WBC**：本章 50.4 的 OSQP/ProxQP 集成在 足式/90_WBC分层优化与TSID 中扩展为**分层优先级 QP**（Task-Space Inverse Dynamics）。

**足式/110_OCS2完整栈与双线程MPC OCS2**：本章 50.5 的 HPIPM + SQP 在 足式/110_OCS2完整栈与双线程MPC 中作为 OCS2 的完整 MPC 后端使用，包括双线程架构（前端 MPC 求解 + 后端 WBC 执行）。

### 50.10.3 研究前沿方向 ⭐⭐⭐

| 方向 | 核心思想 | 代表工作 | 与本章关系 |
|------|---------|---------|-----------|
| **GPU QP** | 把 ADMM/IPM 搬到 GPU 并行 | cuOSQP (2024) | OSQP 的 GPU 版 |
| **Learned warm-start** | 用 NN 预测 QP 的 active set | OptNet, L2WS (2023) | 加速 50.4 的 warm-start |
| **Differentiable QP** | QP 解对参数的梯度 | OptNet, QPLayer | KKT 条件隐函数微分 |
| **Contact-implicit TO** | 不预设接触序列的 NLP | Dojo (2022) | 扩展 50.6 NLP 范围 |
| **ProxDDP** | 把 ProxQP 思想用于 DDP | Aligator (T-RO 2025) | ProxQP → DDP 的推广 |

#### L1-Cost 在鲁棒 QP 中的应用

标准 QP 使用 L2 代价（最小二乘），但当接触力测量含有离群噪声或模型不确定性较大时，L2 代价对异常值极其敏感——一个大残差的平方会主导整个目标函数。**L1-cost**（即 $\min \|r\|_1 = \sum |r_i|$）对异常值具有天然鲁棒性，因为线性增长的惩罚不会被单个大残差主导。

**为什么 L1 鲁棒？** 考虑残差向量 $r = [0.1, 0.2, 10.0]$（第三项是离群值）。L2 代价 $\|r\|_2^2 = 0.01 + 0.04 + 100 = 100.05$，几乎完全由离群值决定。L1 代价 $\|r\|_1 = 0.1 + 0.2 + 10.0 = 10.3$，离群值的影响被大幅削弱。这就是为什么鲁棒估计和鲁棒控制中偏好 L1 范数。

**L1 如何嵌入 QP？** $|r_i|$ 不可微，不能直接作为 QP 目标。标准技巧是引入松弛变量 $s_i^+, s_i^- \geq 0$，令 $r_i = s_i^+ - s_i^-$，则 $|r_i| = s_i^+ + s_i^-$（因为两者不会同时非零）。这样 L1 最小化等价于：

$$\min \sum_i (s_i^+ + s_i^-) \quad \text{s.t.} \quad r_i(x) = s_i^+ - s_i^-, \; s_i^+, s_i^- \geq 0$$

这是一个线性规划（LP），可以进一步加入二次正则项变为 QP。在足式控制中，L1-cost 常用于对接触力残差的鲁棒惩罚——当力传感器偶尔出现尖峰噪声时，L1 代价不会让控制器过度反应。

#### CasADi vs Drake MathematicalProgram：2025 对比

| 维度 | CasADi | Drake MathematicalProgram |
|------|--------|--------------------------|
| **定位** | 符号建模 + 代码生成框架 | 完整机器人工具箱中的优化模块 |
| **建模方式** | 符号计算图（SX/MX），延迟求值 | C++ 直接构建约束/代价对象 |
| **自动微分** | 内置 SX 符号微分 + CppAD 后端 | 依赖 Drake 的 AutoDiffXd |
| **求解器后端** | Ipopt / SNOPT / qpOASES / OSQP / Bonmin / Gurobi | SNOPT / Ipopt / OSQP / Gurobi / SCS / Mosek / CSDP |
| **代码生成** | 核心优势：生成无依赖 C 代码，适合嵌入式 | 不支持独立代码生成 |
| **接触建模** | 需手动编码接触约束 | 内置 ContactSolver / Hydroelastic / SAP |
| **Python 绑定** | 原生 Python 接口，零配置 | pydrake，需编译安装 |
| **实时 MPC** | 通过 acados 集成实现 RTI | 无专用实时框架，需自行搭建 |
| **生态集成** | 独立工具，与 Pinocchio/acados 配合 | 与 Drake MultibodyPlant 深度集成 |
| **学习曲线** | 符号编程思维，对初学者有门槛 | 面向对象 API，文档丰富 |

**选型建议**：需要代码生成部署到嵌入式或需要 acados RTI 的场景选 CasADi；需要完整的接触仿真+优化一体化或使用 Drake 生态的场景选 MathematicalProgram；纯研究原型两者皆可，CasADi 的 Python 原型速度更快。

> **本质洞察**：CasADi 和 Drake 的根本区别不在于"哪个求解器更快"，而在于**抽象层次**——CasADi 让你操控符号表达式然后生成代码，本质是一个"优化问题的编译器"；Drake 让你在一个完整的物理引擎中声明约束，本质是一个"机器人系统的操作系统"。选择哪个取决于你要解决的是"一个优化问题"还是"一个机器人系统问题"。

> **本质洞察**：从 SLAM 的无约束优化到控制的有约束优化，本质跨越不是"多了几行约束代码"，而是**求解器设计哲学的根本转变**——无约束优化只需沿梯度方向走到谷底，有约束优化必须在狭窄的可行域通道中寻路。KKT 条件的对偶变量就是这条通道的"墙壁压力"——它告诉你哪面墙在阻挡你、阻挡得有多厉害。

---

### 跨章综合练习 ⭐⭐⭐

**综合题：从 RNEA 动力学到 WBC-QP 的完整组装**

本题需要综合 足式/50_空间向量与浮动基座动力学（RNEA 逆动力学）、足式/80_接触力学与约束优化（摩擦锥线性化）和本章（QP 建模与求解）三章知识。

**问题描述**：为 Go2 四足机器人的站立平衡场景手动组装一个完整的 WBC-QP：

1. **等式约束组装**（来自 足式/50_空间向量与浮动基座动力学）：写出浮动基座全身动力学 $M\ddot{q} + h = S^T\tau + J_c^T\lambda$ 在给定 $q, \dot{q}$ 下的等式约束形式 $A_{eq} x = b_{eq}$，其中决策变量 $x = [\ddot{q}^T, \lambda^T, \tau^T]^T$。指出 $A_{eq}$ 中各分块的来源（$M$ 来自 CRBA、$J_c$ 来自接触 Jacobian、$S$ 是选择矩阵）。

2. **不等式约束组装**（来自 足式/80_接触力学与约束优化）：对每只着地脚，写出 $k=4$ 棱线性化摩擦锥的不等式约束 $C_\mu \lambda_i \leq 0$，其中 $\lambda_i \in \mathbb{R}^3$。将 4 只脚的摩擦锥约束堆叠为 $C x \leq d$。

3. **QP 目标函数**（本章）：设计二次代价 $\frac{1}{2} x^T H x + g^T x$，包含：(a) 跟踪 MPC 给定的期望加速度 $\ddot{q}_{\text{des}}$ 的权重 $w_1$，(b) 最小化接触力的正则项权重 $w_2$，(c) 最小化关节力矩的权重 $w_3$。写出 $H$ 和 $g$ 的显式表达式。

4. **求解与验证**：使用 OSQP 或 ProxQP 求解上述 QP。检查：(a) 等式约束残差 $\|A_{eq} x^* - b_{eq}\| < 10^{-6}$，(b) 所有摩擦锥约束满足，(c) 接触力法向分量 $\lambda_z > 0$。

---

## 🔧 故障排查手册

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| OSQP 返回 `OSQP_PRIMAL_INFEASIBLE`，QP 无解 | 等式约束与不等式约束互相矛盾（如摩擦锥约束过紧、接触状态设置错误导致无法满足动力学） | 放宽摩擦系数 $\mu$ 检验是否有解；逐步去掉不等式约束定位冲突源；检查接触调度是否与实际步态一致 |
| QP 解中出现异常大的接触力（$>1000$ N） | Hessian 矩阵 $H$ 中接触力的正则化权重过小，或等式约束的 $b$ 向量数值错误 | 增大 $R$ 矩阵中接触力的惩罚权重；打印等式约束右端向量 $b$ 确认动力学残差合理 |
| Warm-start 后 OSQP 迭代次数反而增加 | 上一步的解由于接触模式切换已不在当前问题的可行域内，warm-start 起点比冷启动更差 | 在接触模式切换帧使用冷启动；或仅 warm-start 对偶变量而重置原始变量 |
| Ipopt 报 "Restoration phase failed" | NLP 初始点远离可行域，或约束 Jacobian 在当前点接近奇异 | 提供更好的初始猜测（如上一帧的轨迹）；检查约束函数在初始点处是否可微；降低 NLP 的 horizon 长度 |
| HPIPM 求解时间远超预期（>10 ms） | OCP-QP 的 horizon 过长或状态/控制维度过高，超出了 Riccati 回扫的实时预算 | 缩短 horizon 或增大时间步长；检查是否有冗余约束增大了 KKT 系统维度；确认编译时开启了 `-O2` 优化 |

---

## 本章小结

### API 速查表

| 需求 | 一句话方案 | 代码入口 |
|------|-----------|---------|
| 最快上手 QP | `OsqpEigen::Solver` | `#include <OsqpEigen/OsqpEigen.h>` |
| 最快机器人 QP | `proxsuite::proxqp::dense::QP<double>` | `#include <proxsuite/proxqp/dense/dense.hpp>` |
| 结构化 MPC-QP | HPIPM via acados/OCS2 | `d_ocp_qp_ipm_solve()` |
| 通用 NLP | Ipopt via Ifopt | `ifopt::IpoptSolver` |
| 符号建模 + 代码生成 | CasADi | `casadi::nlpsol("ipopt", ...)` |
| 实时 NLP MPC | acados SQP-RTI | `AcadosOcpSolver` |
| 嵌入式 QP | PIQP | `#include <piqp/piqp.hpp>` |

### 关键概念回顾

| 概念 | 一句话定义 |
|------|-----------|
| QP | 目标二次 + 约束线性的凸优化 |
| NLP | 目标和约束都可以非线性的优化 |
| KKT 条件 | 有约束优化的一阶最优性必要条件 |
| Active set | 在最优解处"顶在边界上"的约束集合 |
| ADMM | 交替方向乘子法，把约束问题分裂为两个简单子问题 |
| Interior-point | 用 barrier 函数把不等式约束"软化"，从内部逼近最优 |
| Riccati 递推 | 利用 MPC 的时域带状结构做高效 KKT 分解 |
| SQP-RTI | 每个控制周期只做一步 SQP 迭代的实时策略 |
| Warm-start | 用上次的解初始化下次求解，大幅减少迭代 |
| 代码生成 | 把符号表达式编译为无依赖的 C 代码 |

### 一周学习计划

| 天数 | 内容 | 时间 | 输出 |
|------|------|------|------|
| Day 1 | 50.0-50.1 概念理解 + KKT 手推 | 3h | 练习 50.1-50.2 完成 |
| Day 2 | 50.2-50.3 QP 标准形式 + 求解器对比 | 3h | 练习 50.3-50.4 完成 |
| Day 3 | 50.4 OSQP 实战 | 4h | 练习 50.5 代码跑通 |
| Day 4 | 50.5 ProxQP + HPIPM 精读 | 4h | 练习 50.6 计算完成 |
| Day 5 | 50.6-50.7 NLP 求解器 + CasADi | 4h | 练习 50.7-50.8 完成 |
| Day 6 | 50.8 Ifopt 实战 + 50.9 选型 | 4h | 练习 50.9-50.10 完成 |
| Day 7 | 源码阅读 (TOWR / OCS2 HpipmInterface) + 总结 | 3h | 能独立做选型决策 |

### 延伸阅读

| # | 资料 | 难度 | 推荐理由 |
|---|------|------|---------|
| 1 | Stellato B., et al. (Math. Prog. Comp. 2020) — "OSQP: An operator splitting solver for quadratic programs" | ⭐⭐ | ADMM-based QP 求解器原理，本章 50.4 的核心参考 |
| 2 | Bambade A., et al. (RSS 2022) — "PROXQP: Yet another Quadratic Programming Solver for Robotics and beyond" | ⭐⭐⭐ | 机器人 QP 最新 SOTA，理解 proximal augmented Lagrangian 方法 |
| 3 | Frison G., Diehl M. (IFAC 2020) — "HPIPM: A high-performance quadratic programming framework for MPC" | ⭐⭐⭐ | 结构化 OCP-QP 的 Riccati 递推求解，足式/110_OCS2完整栈与双线程MPC OCS2 的后端 |
| 4 | Andersson J., et al. (Math. Prog. Comp. 2019) — "CasADi: a software framework for nonlinear optimization and optimal control" | ⭐⭐ | 符号建模 + 代码生成框架，本章 50.7 的核心参考 |
| 5 | Schwan R., et al. (CDC 2023) — "PIQP: A Proximal Interior-Point Quadratic Programming Solver" | ⭐⭐⭐ | 融合 proximal 与 interior-point 的新型 QP 求解器 |
| 6 | Verschueren R., et al. (Math. Prog. Comp. 2022) — "acados: a modular open-source framework for fast embedded optimal control" | ⭐⭐⭐⭐ | 实时 NLP MPC 完整框架，SQP-RTI + 代码生成 |
| 7 | Winkler A. (ETH PhD Thesis, 2018) — "Optimization-Based Motion Planning for Legged Robots" (TOWR) | ⭐⭐ | Ifopt 框架设计 + 腿足轨迹优化经典案例 |
| 8 | Jallet W., et al. (T-RO 2025) — "ProxDDP: Proximal Constrained Trajectory Optimization" (Aligator) | ⭐⭐⭐⭐ | 研究前沿：ProxQP 思想推广到 DDP，Pinocchio 生态最新进展 |

---

**下一章预告**：足式/70_腿足简化模型理论 将进入腿足简化模型理论（LIPM / SLIP / Centroidal），这些模型为本章的 QP/NLP 提供了**目标函数和约束的物理来源**——你将看到"为什么 WBC 的代价函数长那个样子"以及"MPC 的动力学约束从哪里来"。

---

## 累积项目：四足控制器——QP 约束组装与求解器集成模块

> 本模块是"累积项目：四足控制器"的优化求解层。完成后你将拥有一套可在 WBC（足式/90_WBC分层优化与TSID）和 MPC（足式/110_OCS2完整栈与双线程MPC）中直接复用的 QP/NLP 求解器封装，支持热启动和实时性能监控。

**阶段 1：QP 求解器封装（2-3 天）**
- 实现统一 QP 接口 `QpSolver`：封装 `solve(H, g, A, lb, ub, C, lc, uc)` → 返回最优解 $x^*$ 和对偶变量 $\lambda^*$
- 后端实现 OSQP 版本：使用 `OsqpEigen` 封装稀疏矩阵输入，配置 warm-start、最大迭代次数、精度容忍度
- 后端实现 ProxQP 版本：使用 `proxsuite::proxqp::dense::QP`，对比与 OSQP 的精度和速度差异
- 单元测试：构造已知解析解的 QP（如带盒约束的最小二乘），验证求解精度 $< 10^{-8}$

**阶段 2：约束组装工具（2-3 天）**
- 实现动力学等式约束组装：从 足式/50_空间向量与浮动基座动力学 模块获取 $M$, $h$, $J_c$, $S$，组装为 $Ax = b$ 形式
- 实现摩擦锥不等式约束组装：给定摩擦系数 $\mu$ 和接触状态，生成线性化摩擦锥 $Cx \le d$
- 实现关节限位/力矩限位约束：从 URDF 读取 `effort`/`lower`/`upper`，自动生成盒约束
- 集成验证：组装完整 WBC-QP 并求解，检查约束违反量 $< 10^{-6}$

**阶段 3：NLP 求解器封装（2 天）**
- 实现 Ifopt 封装：定义 `VariableSet`、`ConstraintSet`、`CostTerm` 的模板基类
- 实现 CasADi 符号建模示例：用 CasADi 构造一个简单的单步 NLP 并求解
- 性能对比：QP vs NLP 在相同问题上的求解时间差异记录

**阶段 4：实时性验证（1 天）**
- Warm-start 效果测量：连续求解 100 步 QP，对比冷启动 vs 热启动的迭代次数和耗时
- 实时性预算检查：单次 QP 求解 $< 1$ ms（WBC 场景）、$< 5$ ms（MPC 场景）
- 导出统一接口：`solveWBC(tasks, contacts)` 和 `solveMPC(dynamics, horizon)` 供后续章节调用

---
