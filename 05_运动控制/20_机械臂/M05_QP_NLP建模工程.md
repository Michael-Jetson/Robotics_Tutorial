> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M05 QP / NLP 建模工程——从 SLAM 无约束优化到规控约束优化的范式跨越

> **本章定位**：QP（二次规划）和 NLP（非线性规划）是机械臂规控的数学基石。SLAM 工程师习惯了 Ceres/GTSAM 的无约束最小二乘世界，但规控要求处理硬约束——关节限位不能"软化"、碰撞不能"惩罚到差不多"、力矩饱和是物理极限。本章从"为什么 Ceres 不够用"出发，逐层拆解六大求解器的算法原理与 C++ 集成，通过三大典型优化问题（IK QP / 力控 QP / MPC QP）的完整建模模板，让 SLAM 工程师完成从无约束到有约束的认知跨越。
>
> **共享属性**：✅ **规控方向共享**——腿足 MPC、无人机 MPC、机械臂 MPC 全部使用同一批 QP/NLP 求解器。这是 SLAM 工程师进入规控领域最大的认知跨越。
>
> **前置依赖**：Ceres Solver 基础（非线性最小二乘与自动微分，见 02_基础 相关章节）、M01（Pinocchio 动力学引擎）
>
> **下游章节**：M03（IK 求解器深度）、M08（轨迹优化）、M10（时间参数化）
>
> **建议用时**：2 周（理论 1 周 + 实战 1 周）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **Ceres 的问题形式**：写出 Ceres 求解的标准非线性最小二乘问题 $\min_x \sum_i \|f_i(x)\|^2$。Ceres 如何处理"关节角不能超过 ±170°"这种约束？（提示：它只支持简单边界约束 $lb \leq x \leq ub$） | Ceres Solver 基础章节 |
| 2 | **雅可比矩阵**：对于 7-DOF 机械臂，末端笛卡尔速度与关节速度的关系 $v = J(q)\dot{q}$ 中，$J$ 的维度是多少？如何用 Pinocchio 计算？ | M01 Pinocchio |
| 3 | **凸优化基础**：什么是凸集？什么是凸函数？为什么二次函数 $\frac{1}{2}x^TQx + c^Tx$（$Q \succeq 0$）是凸的？凸优化问题有什么特殊性质？ | 线性代数 / 凸优化导论 |
| 4 | **KKT 条件**：写出等式约束优化 $\min f(x) \text{ s.t. } g(x)=0$ 的 Lagrangian 和一阶必要条件。Lagrange 乘子 $\lambda$ 的物理意义是什么？ | 最优化理论 |
| 5 | **自动微分**：Ceres 的 `AutoDiffCostFunction` 如何自动计算 Jacobian？正向模式和反向模式 AD 的区别是什么？ | Ceres Solver 基础章节 |

---

## 本章目标

学完本章后，你应该能够：

1. **解释** Ceres 无约束最小二乘与 QP/NLP 有约束优化之间的本质差异，理解为什么规控问题必须使用约束优化
2. **选型** 六大主流 QP/NLP 求解器（OSQP、ProxQP、qpOASES、HPIPM、Ipopt、CasADi），根据问题规模、实时性需求和约束类型做出合理选择
3. **集成** OSQP-Eigen 和 ProxSuite 到 C++ 项目中，独立完成从矩阵组装到求解的完整流程
4. **建模** 机械臂三大典型优化问题：瞬态 IK QP、力控 TSID QP、滚动时域 MPC QP
5. **使用** CasADi 进行符号化 NLP 建模，理解"符号框架 + 求解器后端"的现代 MPC 工作流
6. **调试** QP/NLP 求解中的常见故障——不可行、数值不稳定、求解时间过长

---

## 1. 从 Ceres 到约束优化——问题建模范式的根本改变 ⭐

### 1.1 你在基础课程中学到的 Ceres 世界 ⭐

回顾 Ceres Solver 基础：Ceres Solver 解决的是非线性最小二乘（NLS）问题：

$$\min_x \sum_{i=1}^{m} \|f_i(x)\|^2$$

这个形式覆盖了 SLAM 的核心问题——因子图优化、Bundle Adjustment、滑窗估计。Ceres 的设计哲学是**最小二乘专家**：它利用了残差 $f_i(x)$ 的结构，通过 $J^TJ$ 近似 Hessian（Gauss-Newton），实现了比通用优化器快一个数量级的收敛速度。

**Ceres 能处理的约束**极为有限：

| 约束类型 | Ceres 支持 | 说明 |
|---------|-----------|------|
| 无约束 | ✅ 原生 | SLAM 因子图的主场 |
| 边界约束 $lb \leq x \leq ub$ | ✅ 简单 | `SetParameterLowerBound/UpperBound` |
| 线性不等式 $Ax \leq b$ | ❌ | 无法表达 |
| 非线性等式 $g(x) = 0$ | ❌（软化） | 只能加大权重模拟硬约束 |
| 非线性不等式 $h(x) \leq 0$ | ❌ | 无法表达 |

> **本质洞察**：Ceres 和 GTSAM 的设计哲学是"世界是高斯的、约束是柔的"——传感器噪声是高斯分布，先验是高斯分布，连鲁棒核函数（Huber/Cauchy）也是对高斯的修正。在这个世界里，所有"约束"都以代价函数（cost function）的形式存在，权衡是通过权重矩阵 $\Sigma^{-1}$ 完成的。

### 1.2 规控的世界：约束不是建议，是物理极限 ⭐

现在考虑机械臂控制的典型问题：

**逆运动学（IK）**：给定末端目标位姿，求关节角度

```
  min_q̇   ‖J(q)·q̇ - v_desired‖² + λ·‖q̇ - q̇_nominal‖²
  s.t.    q̇_lb ≤ q̇ ≤ q̇_ub                 (关节速度限制)
          A_collision·q̇ ≤ b_collision      (碰撞回避的一阶近似)
          q_lb ≤ q + q̇·dt ≤ q_ub          (关节位置限位)
```

**模型预测控制（MPC）**：预测 N 步，最小化跟踪误差

```
  min_{u_0,...,u_{N-1}}  Σ_t  ‖x_t - x_ref‖²_Q + ‖u_t‖²_R
  s.t.    x_{t+1} = A·x_t + B·u_t          (动力学约束，等式)
          u_lb ≤ u_t ≤ u_ub                 (控制限制，不等式)
          x_lb ≤ x_t ≤ x_ub                 (状态限制，不等式)
```

**这些约束为什么不能"软化"成代价函数？**

| 约束 | 如果用 Ceres 软化处理 | 后果 |
|------|---------------------|------|
| 关节限位 $q \in [-170°, 170°]$ | $w \cdot \max(0, |q|-170°)^2$ | 权重 $w$ 太小则穿限位导致电机堵转损坏；$w$ 太大则数值病态 |
| 力矩饱和 $|\tau| \leq \tau_{max}$ | $w \cdot \max(0, |\tau|-\tau_{max})^2$ | 违反后电机驱动器硬件截断，实际轨迹与规划不符，导致失控 |
| 动力学方程 $M\ddot{q} + h = \tau$ | $w \cdot \|M\ddot{q} + h - \tau\|^2$ | 违反物理定律，规划出的轨迹不可执行 |
| 碰撞约束 $d(q) \geq d_{safe}$ | $w \cdot \max(0, d_{safe} - d(q))^2$ | 微小违反导致机械臂撞到工件或人，引发安全事故 |

> **反事实推理**：如果我们坚持用 Ceres 的软约束方式做 MPC，会发生什么？假设关节力矩限 $\tau_{max} = 87$ Nm（Franka Panda 关节 1 的限制），你把它写成权重为 $10^6$ 的惩罚项。当优化器在某次迭代中试探了 $\tau = 88$ Nm 的解，惩罚项的梯度变成 $2 \times 10^6 \times 1 = 2 \times 10^6$，而跟踪误差的梯度可能只有 $10^2$ 量级。这个 4 个数量级的梯度差异会导致 Hessian 矩阵条件数飙升到 $10^8$，Gauss-Newton 迭代会在约束附近来回震荡——Ceres 的 Levenberg-Marquardt 阻尼器会拼命增大 $\lambda$，收敛变得极慢。更糟的是，即使收敛了，也只是"几乎满足"约束——$\tau = 87.003$ Nm 在优化器看来是合格解（惩罚项很小），但硬件驱动器会将其截断为 87 Nm，产生不可预测的轨迹偏差。

**结论**：规控问题需要**硬约束**处理能力。这不是"换一个更好的 Ceres"就能解决的——需要从根本上改变问题建模范式，从无约束最小二乘转向有约束 QP/NLP。

### 1.3 QP 与 NLP 的数学定义 ⭐

**二次规划（QP）**——目标函数是二次的，约束是线性的：

$$\min_x \frac{1}{2} x^T P x + q^T x$$
$$\text{s.t.} \quad Ax = b \quad (\text{等式约束})$$
$$\quad Gx \leq h \quad (\text{不等式约束})$$
$$\quad lb \leq x \leq ub \quad (\text{边界约束})$$

其中 $P \in \mathbb{R}^{n \times n}$ 是对称半正定矩阵（Hessian），$q \in \mathbb{R}^n$ 是线性项，$A \in \mathbb{R}^{m_e \times n}$ 是等式约束矩阵，$G \in \mathbb{R}^{m_i \times n}$ 是不等式约束矩阵。

**非线性规划（NLP）**——目标函数和约束都可以是非线性的：

$$\min_x f(x)$$
$$\text{s.t.} \quad g(x) = 0$$
$$\quad h(x) \leq 0$$
$$\quad lb \leq x \leq ub$$

QP 是 NLP 的特殊情况（线性约束 + 二次目标）。正因如此，QP 有专门的高效算法——利用了二次+线性的结构，比通用 NLP 算法快 1-2 个数量级。

**机械臂规控中 QP 和 NLP 的角色分工**：

| 问题类型 | 数学形式 | 求解频率 | 典型求解器 | 应用场景 |
|---------|---------|---------|-----------|---------|
| 瞬态 IK | QP | 1 kHz（每周期一次） | ProxQP、OSQP | 速度级逆运动学 |
| 力分配/WBC | QP | 1 kHz | ProxQP、eiquadprog | 全身控制 |
| 线性 MPC | QP | 50-200 Hz | HPIPM、OSQP | 线性化动力学 MPC |
| 轨迹优化 | NLP | 离线/1-10 Hz | Ipopt、SNOPT | 运动规划 |
| 非线性 MPC | NLP（SQP→QP 子问题） | 10-50 Hz | OCS2(SQP+HPIPM) | 全模型 MPC |

### 1.4 从 Gauss-Newton 到 KKT——算法层面的跨越 ⭐⭐

**Ceres 的求解核心**（Gauss-Newton / Levenberg-Marquardt）：

```
在每次迭代中:
  1. 计算 Jacobian: J_i = ∂f_i/∂x
  2. 组装法方程: (J^T J + λI) δx = -J^T r    (r 是残差向量)
  3. 求解增量: δx = solve(H, -g)
  4. 线搜索/信赖域更新
```

这里的法方程 $(J^TJ + \lambda I)\delta x = -J^Tr$ 是一个**无约束线性系统**。Ceres 用稀疏 Cholesky 分解（`SPARSE_SCHUR`）或 PCG 迭代求解——这些方法在 Ceres Solver 基础章节已经详细讲解过。

**QP/NLP 的求解核心**（KKT 系统）：

对于带约束的优化问题，最优解必须满足 **Karush-Kuhn-Tucker（KKT）条件**：

$$\nabla f(x^*) + A^T \lambda^* + G^T \mu^* = 0 \quad (\text{驻点条件})$$
$$Ax^* = b \quad (\text{等式约束原始可行性})$$
$$Gx^* \leq h \quad (\text{不等式约束原始可行性})$$
$$\mu^* \geq 0 \quad (\text{对偶可行性})$$
$$\mu_i^* (Gx^* - h)_i = 0 \quad (\text{互补松弛条件})$$

互补松弛条件是关键：它说的是"每个不等式约束要么取等（$\mu > 0$，约束活跃），要么松弛（$\mu = 0$，约束不活跃）"。这个条件引入了**组合复杂性**——$m$ 个不等式约束有 $2^m$ 种可能的活跃组合，这正是 QP 求解的核心困难。

**三大求解策略应对这个组合问题**：

| 策略 | 代表算法 | 核心思想 | 适用场景 |
|------|---------|---------|---------|
| **活跃集法（Active Set）** | qpOASES | 维护"当前活跃约束"集合，每次迭代添加/删除一个约束 | warm-start 场景（MPC 相邻时刻） |
| **内点法（Interior Point）** | HPIPM、Ipopt | 用对数障碍函数将不等式约束嵌入目标，逐步逼近边界 | 大规模问题、高精度要求 |
| **算子分裂法（Operator Splitting）** | OSQP (ADMM) | 将 QP 分裂成多个简单子问题交替求解 | 通用性强、冷启动稳定 |

> **跨领域类比**：三种求解策略可以类比为三种搜索算法——活跃集法像**深度优先搜索**（从一个活跃集出发，每次改变一个约束，沿着约束边界"行走"直到最优）；内点法像**中心路径法**（从内部出发，沿着对数障碍函数的中心路径逐步逼近最优边界）；ADMM 像**分治法**（把大问题拆成小子问题，各自求解后协调）。不同的是，活跃集法在 warm-start 场景下接近 $O(1)$，内点法总是 $O(\sqrt{n})$ 次迭代但每次迭代高质量，ADMM 迭代廉价但可能需要较多次数。

### 1.5 SLAM 到规控的认知跨越清单 ⭐

| 维度 | SLAM（Ceres/GTSAM） | 规控（QP/NLP） |
|------|---------------------|--------------|
| 问题形式 | $\min \sum \|f_i(x)\|^2$ | $\min f(x) \text{ s.t. } g(x)=0, h(x)\leq 0$ |
| 约束处理 | 无约束 / 软约束 | 硬约束（KKT 条件） |
| 求解频率 | 离线 / 低频（10 Hz BA） | 实时高频（1 kHz IK，100 Hz MPC） |
| warm-start | 上一帧的边缘化先验 | 上一周期的解 / 活跃集 |
| Hessian 来源 | $J^TJ$（Gauss-Newton 近似） | QP 的 $P$ 是精确的；NLP 需要 $\nabla^2 L$ |
| 线性系统 | $(J^TJ + \lambda I)\delta x = -J^Tr$ | KKT 系统（含约束乘子 $\lambda, \mu$） |
| 主要困难 | 稀疏结构利用、边缘化 | 约束可行性、组合爆炸、数值稳定性 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：把 QP 求解器当 Ceres 用——试图把所有约束都写成代价函数
>
> **错误做法**：`cost += 1e6 * max(0, q - q_max)^2`
>
> **现象**：权重太小时约束被违反（关节超限位），权重太大时 Hessian 条件数爆炸（数值不稳定，求解器报 NaN）
>
> **根本原因**：Ceres 的 cost function 和 QP 的 constraint 是两种本质不同的数学对象——前者是"偏好"（violation 有代价但允许），后者是"必须"（violation 不可行）
>
> **正确做法**：用 QP 求解器的约束接口 `l <= Ax <= u` 表达硬约束，用 cost 表达优化目标。不要混淆两者的角色

> 💡 **概念误区**：认为 NLP 比 QP 更"高级"，所以应该直接学 NLP
>
> **新手想法**："NLP 是通用的，QP 是特殊情况，学通用的不就够了？"
>
> **实际上**：QP 的线性约束 + 二次目标结构允许专门的高效算法，性能比通用 NLP 好 10-100 倍。机械臂 1 kHz IK 用 ProxQP 需要 24 us，用 Ipopt 需要 1-5 ms——差两个数量级。**能用 QP 的场景绝不要用 NLP**
>
> **正确思维**：先判断问题能否线性化为 QP（大部分 IK 和线性 MPC 可以），只有必须处理非线性约束时才用 NLP

> 🧠 **思维陷阱**：认为"约束越多越安全"
>
> **新手想法**："把所有能想到的安全约束都加上，QP 就能保证安全了"
>
> **实际上**：过多的约束可能导致**不可行**（feasibility problem）——QP 求解器返回 INFEASIBLE，机器人完全停摆。工程中需要分层：硬约束（关节限位、碰撞）用不等式，软约束（参考轨迹跟踪）用代价函数，并设计 fallback 策略
>
> **正确思维**：约束设计需要权衡安全性和可行性。TSID/WBC 框架用优先级栈（priority stack）解决这个问题——高优先级约束（安全）必须满足，低优先级目标（性能）在可行范围内尽力

### 练习

1. ⭐ 写出 Ceres `AutoDiffCostFunction` 和 QP 约束 $Ax \leq b$ 在数学上的区别。如果把关节限位 $q \leq q_{max}$ 分别用 Ceres 惩罚项和 QP 不等式表达，画出 cost landscape 的定性对比图。
2. ⭐⭐ 对于 7-DOF 机械臂 IK，如果有 7 个关节速度上下限、7 个关节位置上下限、3 个碰撞回避约束，KKT 系统有多少个变量（原始 + 对偶）？画出 KKT 矩阵的稀疏结构。
3. ⭐⭐ 用 GTSAM 对 SLAM 后端添加"机器人不能穿过墙壁"的硬约束。你会发现 GTSAM 的因子图 API 无法直接表达不等式约束——这是 SLAM 与规控生态分野的根源。描述你会如何绕过这个限制，以及为什么这个绕过方案是"不优雅的"。

---

## 2. QP 求解器全景——六大求解器深度对比 ⭐⭐

上一节解释了为什么 SLAM 工程师需要从 Ceres 跨越到约束优化。但约束优化不是一个求解器就能覆盖的领域——不同的求解器在算法、性能特征、API 风格上差异巨大。本节系统对比六大主流 QP 求解器，建立选型框架。

### 2.1 动机：为什么有这么多 QP 求解器 ⭐⭐

在 SLAM 领域，你基本只需要 Ceres 和 GTSAM——一个做通用 NLS，一个做因子图。但在规控领域，QP 求解器的选择异常丰富，因为**不同的 QP 算法在不同场景下有数量级的性能差异**：

- 冷启动大规模稀疏 QP → OSQP（ADMM 一阶算法，每次迭代成本低）
- warm-start 小规模密集 QP → qpOASES（活跃集法，相邻问题只需几次迭代）
- 机器人中小规模高精度 QP → ProxQP（近端增广拉格朗日，Eigen 原生）
- MPC 多阶段结构 QP → HPIPM（Riccati 递推利用带状稀疏，$O(N)$ 复杂度）

选错求解器可能导致 10-100 倍的性能损失。这不是夸张——下文的基准测试数据会证明。

### 2.2 六大求解器总览 ⭐⭐

| 求解器 | Stars | 语言 | 算法 | 许可证 | Eigen 原生 | 典型用户 | 实时性 |
|--------|-------|------|------|--------|-----------|---------|--------|
| **OSQP** | ~2k | C(osqp-eigen) | ADMM | Apache-2.0 | via osqp-eigen | Drake、Autoware、OCS2 | ✅ 优秀 |
| **qpOASES** | ~520 | C++ | 在线活跃集 | LGPL-2.1 | ❌ 裸数组 | MIT Cheetah、acados | ✅ 良好 |
| **HPIPM** | ~650 | C | IPM + Riccati | BSD-2 | ❌(需 BLASFEO) | acados 默认、OCS2 | ✅✅ 卓越 |
| **ProxQP**(ProxSuite) | ~500 | C++17 | 近端增广拉格朗日 | BSD-2 | ✅ 原生 | Crocoddyl、TSID | ✅✅ 卓越 |
| **qpSWIFT** | ~155 | C | IPM + Nesterov-Todd | GPL-3.0 | ❌ | Ghost Robotics Vision60 | ✅ 良好 |
| **PIQP** | ~120 | C++14 header-only | IPM + PMM | BSD-2 | ✅ 原生 | EPFL 最优控制 | ✅ 良好 |

**ProxQP 的性能优势已被量化验证**（RSS 2022 论文，Bambade 等）：

| 场景 | ProxQP | OSQP | 加速比 |
|------|--------|------|--------|
| 逆运动学 QP | 24±7 us | 167±93 us | **7x** |
| 逆动力学 QP | 25±6 us | 441±193 us | **18x** |
| 高精度 eps=1e-9 | — | — | **1.7-2.9x** |

### 2.3 算法内核对比——四大家族 ⭐⭐⭐

为了理解性能差异的根源，我们需要深入理解四种算法家族的核心思想。每种算法的诞生都有其历史背景和工程动机——不是凭空创造的，而是对前一代方法局限性的回应。

#### 2.3.1 ADMM（Alternating Direction Method of Multipliers）——OSQP 的算法内核

**历史起源**：ADMM 最早可追溯到 1975 年 Gabay 和 Mercier 的工作，但真正让它在工程界走红的是 2011 年 Boyd 等人的综述论文《Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers》。这篇论文展示了 ADMM 在分布式优化、信号处理和机器学习中的广泛应用。OSQP 团队（Stellato, Banjac, Goulart, Bemporad, Boyd, 2020）将 ADMM 专门适配到 QP 上，做了大量工程优化。

**核心思想**：将 QP 分裂为两个容易求解的子问题，交替迭代。ADMM 的精髓是"把一个难问题拆成两个容易的子问题"——一个处理目标函数（二次的，有闭式解），一个处理约束（投影到约束集，逐元素操作）。

```
ADMM 三步迭代:
  Step 1 (x-update):  x^{k+1} = (P + sigma*I)^{-1} (sigma*z^k - y^k - q)
                       → 解一个线性系统
                       → 关键: P + sigma*I 在迭代中不变, 只需一次 LDL^T 分解!

  Step 2 (z-update):  z^{k+1} = Pi_{[l,u]}(A*x^{k+1} + y^k/rho)
                       → 逐元素投影到约束盒 [l,u]（极廉价, O(m) 操作）
                       → z_i = clamp(v_i, l_i, u_i) = max(l_i, min(u_i, v_i))

  Step 3 (y-update):  y^{k+1} = y^k + rho*(A*x^{k+1} - z^{k+1})
                       → 对偶变量更新（纯向量加法, O(m) 操作）
```

**关键洞察**：Step 1 需要求解线性系统 $(P + \sigma I + \rho A^TA)^{-1}$，但因为 $P$、$\sigma$、$\rho$（大部分时间）在迭代过程中不变，所以**只需一次矩阵分解**（LDL^T），后续迭代只做 $O(\text{nnz})$ 的前向/回代。这使得 ADMM 每次迭代成本很低——但收敛可能需要 50-200 次迭代（冷启动）。

**工程优势**：
- 稀疏矩阵原生支持（CSC 格式，配合稀疏 LDLT 分解）
- `osqp-codegen` 可把求解器编译为自包含 C 代码——嵌入式部署友好
- 一阶算法的**鲁棒性**：即使问题接近不可行，ADMM 也不会崩溃——它会慢慢收敛到"最不可行"的解，给出有用的诊断信息

**与 SLAM 的类比**：ADMM 的分裂思想类似于 SLAM 中的 Gauss-Seidel 迭代——把大系统拆成小块轮流求解。但 ADMM 有更好的收敛保证（凸问题全局收敛），而 Gauss-Seidel 在非对称问题上可能发散。

#### 2.3.2 活跃集法（Active Set Method）——qpOASES 的算法内核

**历史起源**：活跃集法的思想由 Goldfarb 和 Idnani（1983）正式提出，但根源更早——Dantzig 1947 年的单纯形法本质上就是线性规划的活跃集法。qpOASES（Ferreau, Kirches, Potschka, Bock, Diehl, 2014）将活跃集法扩展为"在线"版本——利用参数化 QP 序列的连续性，这正是 MPC 所需要的。

**核心思想**：维护"当前活跃约束"集合 $\mathcal{W}$（Working Set），在每次迭代中添加或删除一个约束。

```
Active Set 算法:
  1. 初始化: 选择一个可行的初始活跃集 W (warm-start: 用上次的 W)
  2. 求解等式约束子问题: 只考虑 W 中的约束(当作等式)
     → 解一个等式约束 QP, 有闭式解 (KKT 系统)
  3. 检查:
     - 如果解满足所有约束 → 检查 Lagrange 乘子
       - 所有 lambda >= 0 → 最优解! 算法终止
       - 存在 lambda < 0 → 从 W 中删除对应约束 (该约束不应活跃)
     - 如果解违反某个非活跃约束 → 将其加入 W
  4. 重复直到收敛
```

**为什么 warm-start 对活跃集法特别有效？** 考虑 MPC 在 $t$ 和 $t+\Delta t$ 两个相邻时刻的 QP。由于系统状态只变化了一小步，最优活跃集通常只有 0-2 个约束不同：

```
时刻 t 的最优活跃集:     {u1_max, x3_min, u5_max}  (3 个约束取等)
时刻 t+dt 的最优活跃集:  {u1_max, x3_min, u6_max}  (只有 1 个约束变了)
```

活跃集法从 $t$ 时刻的活跃集出发，只需**删除 $u5_{max}$、添加 $u6_{max}$**——两步迭代即可收敛。MIT Cheetah 3 的凸 MPC 正是利用这个特性，用 qpOASES 在 **1 ms 内求解 10-16 步时域**。

**工程特性**：
- **无动态内存分配**（嵌入式实时安全）
- Primal/Dual 两种初始化策略可切换
- **代码生成支持**——可编译为嵌入式 C

**现状**：项目基本停更（最后一次实质性更新约 2017 年），新项目建议用 ProxQP 或 PIQP。但**大量遗留代码仍在使用**（acados、MIT Cheetah、rpg_mpc 无人机控制），工程师必须能读懂其接口。

#### 2.3.3 内点法（Interior Point Method）——HPIPM 和 Ipopt 的算法内核

**历史起源**：Karmarkar 1984 年的多项式时间算法震动了线性规划领域——在此之前只有 Dantzig 的单纯形法，最坏情况是指数时间（虽然实践中通常很快）。Karmarkar 证明了线性规划可以在多项式时间内求解，开启了内点法的黄金时代。此后 Nesterov 和 Nemirovsky（1994）建立了内点法的一般理论（自协调障碍函数），统一了 LP、QP、SOCP、SDP 的内点法框架。HPIPM（Frison & Diehl 2020）将内点法适配到 MPC 的多阶段结构——这是将算法理论落地到工程的典范。

**核心思想**：用对数障碍函数将不等式约束嵌入目标，沿着中心路径逐步逼近边界：

$$\min_x f(x) - \mu \sum_{i=1}^{m} \ln(s_i) \quad \text{s.t.} \quad g(x) + s = 0, \; s > 0$$

其中 $\mu > 0$ 是障碍参数，$s$ 是松弛变量。当 $\mu \to 0$ 时，解趋近于原问题最优解。

**直觉解释**：对数障碍函数 $-\mu\ln(s)$ 在 $s \to 0^+$ 时趋于 $+\infty$——它在约束边界处设置了一个"无穷高的墙"，阻止解越过约束。$\mu$ 控制墙的"软硬程度"：$\mu$ 大时，墙很软，解离边界远；$\mu$ 小时，墙很硬，解贴近边界。内点法就是从大 $\mu$（简单问题，容易求解）逐步减小到 $\mu \approx 0$（逼近原问题最优解）。

**HPIPM 的特殊优化**：MPC QP 的 Hessian 和约束矩阵有**带状稀疏结构**——动力学约束只耦合相邻时刻状态。HPIPM 识别这个结构，用 **Riccati 递推**闭式求解每次 IPM 迭代的 KKT 系统：

```
通用 QP 的 KKT 系统: O(N^2 * n^3) 或 O(N * n^3) + 稀疏求解器开销
HPIPM 的 Riccati:     O(N * n^3)    ← 线性于时域步数 N, 常数极小!
```

对于典型 MPC（N=20，n_x=12），这意味着 10-20 倍加速。更重要的是，HPIPM 内部使用 BLASFEO——一个专门优化小矩阵运算（4x4 到 64x64）的 BLAS 实现，利用了处理器的向量寄存器布局，在机器人典型矩阵尺寸上比通用 BLAS（OpenBLAS、MKL）快 2-3 倍。

#### 2.3.4 近端增广拉格朗日法——ProxQP 的算法内核

**历史起源**：ProxQP（Bambade, El-Kazdadi, Justin, Carpentier, 2022）结合了增广拉格朗日法和近端算子理论。增广拉格朗日法由 Hestenes（1969）和 Powell（1969）独立提出，近端算子理论由 Moreau（1965）奠基。ProxQP 的创新在于将两者融合，并针对机器人 QP 的特点（中小规模、密集、高精度）做了系统优化。

**核心思想**：在增广拉格朗日框架下，用近端算子平滑化 KKT 条件，使得即使在约束不满足的情况下也能稳定迭代：

$$L_\rho(x, \lambda, \mu) = \frac{1}{2}x^THx + g^Tx + \frac{1}{2\mu_e}\|Ax - b + \mu_e\lambda\|^2 + \frac{1}{2\mu_i}\left(\|\max(0, Cx - u)\|^2 + \|\max(0, l - Cx)\|^2\right)$$

近端项 $\frac{1}{2\mu}\|x - x^k\|^2$ 的作用有三：

1. **保证子问题 Hessian 正定**：$H + \frac{1}{\mu}I + \frac{1}{\mu_e}A^TA + \frac{1}{\mu_i}C_\mathcal{A}^TC_\mathcal{A} \succ 0$（即使 $H$ 只是半正定的）
2. **控制步长**：$\mu$ 越小，步长越保守（更稳定但更慢）；$\mu$ 越大，步长越激进
3. **自然提供迭代方向**：当 $\mu \to \infty$ 时，近端项消失，退化为标准增广拉格朗日

**对 OSQP 的加速来源**（7-18 倍加速的原因）：

| 加速来源 | 解释 |
|---------|------|
| **更好的条件化** | 近端项使子问题条件数有界，减少内部迭代 |
| **Eigen 原生** | 无 CSC 格式转换开销，密集矩阵运算直接利用 BLAS/LAPACK |
| **精度控制** | 增广拉格朗日法的外层迭代提供精确的约束满足度控制（OSQP 的 ADMM 在高精度时收敛变慢） |
| **活跃集检测** | ProxQP 在迭代过程中识别活跃约束，减少后续迭代的计算量 |

### 2.4 选型决策流程 ⭐⭐

```
你的问题是什么类型？
│
├── 已知结构的 MPC（多阶段时域）
│   └─→ HPIPM (via acados)     ← O(N) Riccati, MPC 无出其右
│
├── 机器人 IK/WBC（中小规模 <500 变量）
│   ├── 需要 Eigen 原生 API → ProxQP   ← Pinocchio/TSID 生态首选
│   └── 需要最大社区支持   → OSQP      ← Drake/Autoware 生态
│
├── 遗留代码集成 / 无动态内存需求
│   └─→ qpOASES                ← 停更但稳定, 嵌入式友好
│
├── Header-only 零依赖需求
│   └─→ PIQP                  ← 新项目、教学实验
│
└── 大规模通用 QP（>10000 变量）
    └─→ OSQP                  ← 稀疏 ADMM 的强项
```

> **本质洞察**：QP 求解器的选择不是"哪个最好"，而是"哪个最匹配你的问题结构"。选型的核心维度是三个：**问题规模**（决定算法复杂度）、**稀疏结构**（决定能否利用结构化求解）、**warm-start 需求**（决定活跃集法 vs 一阶方法）。忽略问题结构盲目选择"最新"的求解器，可能得到比"最老"的求解器更差的性能。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：混淆 OSQP 标准形式中的约束表达
>
> **错误做法**：把等式约束 $Ax = b$ 写成两个不等式 $Ax \leq b$ 和 $Ax \geq b$
>
> **现象**：求解变慢（ADMM 需要更多迭代来"发现"两个不等式实际是等式），精度下降
>
> **根本原因**：OSQP 的标准形式是 $l \leq Ax \leq u$。等式约束应该写成 $l = u = b$，让求解器内部识别并特殊处理
>
> **正确做法**：对等式约束行设置相同的上下界 $l_i = u_i = b_i$

> 💡 **概念误区**：认为 ProxQP 在所有场景下都比 OSQP 好
>
> **新手想法**："ProxQP 快 7 倍，应该所有场景都用 ProxQP"
>
> **实际上**：ProxQP 的性能优势集中在**密集/中小规模 QP**（<500 变量）。对于大规模稀疏 QP（如 Autoware 的路径规划 QP，可能有数千个变量），OSQP 的稀疏 ADMM 更合适。另外 ProxQP 需要 C++17，在 C++11/14 项目中可能需要额外配置

### 练习

1. ⭐ 对于 Franka Panda 7-DOF IK QP（7 个变量、14 个关节限位约束、6 个碰撞约束），计算 KKT 系统的总维度。这个规模属于"小规模密集"——对应推荐哪个求解器？
2. ⭐⭐ 画出 MPC QP（N=10 步，n_x=6 状态，n_u=3 控制）的 Hessian 矩阵和约束矩阵的稀疏模式。解释为什么这个带状结构对 HPIPM 的 Riccati 递推至关重要。
3. ⭐⭐⭐ 阅读 OSQP 论文（Stellato et al. 2020）的 Section 3.1，理解 ADMM 中 $\rho$ 参数对收敛速度的影响。如果 $\rho$ 选得太大会怎样？太小呢？OSQP 如何自适应调整 $\rho$？

---

## 3. OSQP 算法与 C++ 集成 ⭐⭐

### 3.1 动机：为什么从 OSQP 入门 ⭐⭐

在众多 QP 求解器中，OSQP 是入门的最佳选择——不是因为它最快，而是因为三个务实的理由：

1. **文档最全**：官方文档覆盖 Python/C/C++/Julia/Matlab 五种语言，每种都有完整示例
2. **社区最大**：Drake、Autoware、OCS2 等主流框架都集成了 OSQP，遇到问题有大量参考
3. **API 最直观**：标准 QP 形式 $\min \frac{1}{2}x^TPx + q^Tx \text{ s.t. } l \leq Ax \leq u$，无隐含假设

学 OSQP 不是为了永远用它——而是为了建立 QP 概念，之后切换到 ProxQP 或 HPIPM 只需改几行代码。

### 3.2 OSQP 的标准 QP 形式 ⭐⭐

OSQP 要求将 QP 统一写成：

$$\min_x \frac{1}{2} x^T P x + q^T x \qquad \text{s.t.} \quad l \leq Ax \leq u$$

注意：所有约束——等式、不等式、边界——都统一在 $l \leq Ax \leq u$ 中。

**把标准 QP 形式映射到 OSQP**：

| 标准形式 | OSQP 映射 | 说明 |
|---------|----------|------|
| 等式 $Cx = d$ | $A$ 的对应行，$l_i = u_i = d_i$ | 上下界相同 |
| 不等式 $Gx \leq h$ | $A$ 的对应行，$l_i = -\infty$，$u_i = h_i$ | 上界约束 |
| 不等式 $Gx \geq h$ | $A$ 的对应行，$l_i = h_i$，$u_i = +\infty$ | 下界约束 |
| 边界 $lb \leq x \leq ub$ | $A$ 中添加单位矩阵 $I$ 的行 | 或拼入已有约束 |

这种统一表达虽然需要一些组装工作，但好处是求解器内部无需区分约束类型——统一处理。

### 3.3 C++ 集成（通过 osqp-eigen） ⭐⭐

osqp-eigen（`robotology/osqp-eigen`）由 IIT（意大利理工学院）的 Stefano Dafarra 开发，是 OSQP C 接口的 Eigen 封装层。它将 OSQP 的 C 风格 API（裸指针、CSC 手工组装）包装成 Eigen 友好的 C++ 接口。

**完整的使用流程**：

```cpp
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

// === Step 1: 构造问题数据 ===
// 示例: 2 变量, 3 约束
int n = 2;  // 变量数
int m = 3;  // 约束数

// Hessian P (稀疏对称矩阵；底层 OSQP 只存上三角)
Eigen::SparseMatrix<double> P(n, n);
P.insert(0, 0) = 4.0;    // P 的 (0,0) 元素
P.insert(0, 1) = 1.0;    // P 的 (0,1) 元素 (上三角)
P.insert(1, 1) = 2.0;    // P 的 (1,1) 元素
P.makeCompressed();       // 必须! 转为 CSC 格式

// 线性项 q
Eigen::VectorXd q(n);
q << 1.0, 1.0;

// 约束矩阵 A (稀疏)
Eigen::SparseMatrix<double> A(m, n);
A.insert(0, 0) = 1.0;    // 约束 1: x0 的系数
A.insert(1, 1) = 1.0;    // 约束 2: x1 的系数
A.insert(2, 0) = 1.0;    // 约束 3: x0 + x1 = 1
A.insert(2, 1) = 1.0;
A.makeCompressed();       // 必须!

// 上下界
Eigen::VectorXd l(m), u(m);
l << -OsqpEigen::INFTY, -OsqpEigen::INFTY, 1.0;   // 约束 3 下界 = 1
u << 1.0, 1.0, 1.0;                                 // 约束 3 上界 = 1 (等式)

// === Step 2: 配置求解器 ===
OsqpEigen::Solver solver;
solver.settings()->setVerbosity(false);
solver.settings()->setWarmStart(true);         // MPC 必须开启
solver.settings()->setAbsoluteTolerance(1e-6);
solver.settings()->setRelativeTolerance(1e-6);
solver.settings()->setMaxIteration(4000);

solver.data()->setNumberOfVariables(n);
solver.data()->setNumberOfConstraints(m);
solver.data()->setHessianMatrix(P);
solver.data()->setGradient(q);
solver.data()->setLinearConstraintsMatrix(A);
solver.data()->setLowerBound(l);
solver.data()->setUpperBound(u);

// === Step 3: 初始化并求解 ===
if (!solver.initSolver()) {
    std::cerr << "OSQP init failed!" << std::endl;
    return -1;
}

auto status = solver.solveProblem();
if (status != OsqpEigen::ErrorExitFlag::NoError) {
    std::cerr << "OSQP solve failed!" << std::endl;
    return -1;
}
if (solver.getStatus() != OsqpEigen::Status::Solved &&
    solver.getStatus() != OsqpEigen::Status::SolvedInaccurate) {
    std::cerr << "OSQP did not find a feasible optimum. status = "
              << static_cast<int>(solver.getStatus()) << std::endl;
    return -1;
}

// === Step 4: 提取结果 ===
Eigen::VectorXd x_opt = solver.getSolution();
double obj_val = solver.getObjValue();
std::cout << "x* = " << x_opt.transpose() << std::endl;
std::cout << "f* = " << obj_val << std::endl;
```

### 3.4 OSQP 的工程关键参数 ⭐⭐

| 参数 | 默认值 | 含义 | 调参建议 |
|------|--------|------|---------|
| `eps_abs` | 1e-3 | 绝对容差 | IK 用 1e-6，MPC 用 1e-4 |
| `eps_rel` | 1e-3 | 相对容差 | 同上 |
| `rho` | 0.1 | ADMM 步长 | 自适应调整（`adaptive_rho=true`） |
| `sigma` | 1e-6 | 正则化 | 保持默认 |
| `max_iter` | 4000 | 最大迭代 | 实时场景设 200-500 |
| `warm_start` | true | 热启动 | MPC 必须开启 |
| `scaling` | 10 | 预缩放迭代 | 首次求解开启，warm-start 关闭 |
| `polish` | false | 精修步 | 需要高精度时开启（增加 ~20% 时间） |

> **反事实推理**：如果在 1 kHz IK 场景中忘记设置 `max_iter`，OSQP 会用默认的 4000 次迭代上限。当问题接近不可行时（如机械臂接近奇异构型），ADMM 可能需要数百次迭代，导致求解时间从 100 us 飙升到数毫秒，超出 1 ms 控制预算。工程中应根据实时约束设置硬上限，并在达到上限时使用上一周期的解作为 fallback。

### 3.5 warm-start 的正确使用 ⭐⭐

warm-start 是 MPC 场景中最关键的加速手段。OSQP 支持同时提供原始变量 $x$ 和对偶变量 $y$ 的初始猜测：

```cpp
// MPC 控制循环中的 warm-start 模板
Eigen::VectorXd x_prev, y_prev;
bool first_solve = true;

for (int t = 0; t < total_steps; ++t) {
    // 更新 QP 参数(新的状态、新的参考轨迹)
    solver.updateGradient(q_new);
    solver.updateBounds(l_new, u_new);

    // warm-start: 用上一周期的解
    if (!first_solve) {
        solver.setWarmStart(x_prev, y_prev);
    }

    // 求解
    solver.solveProblem();

    // 保存本周期的解用于下一次 warm-start
    x_prev = solver.getSolution();
    y_prev = solver.getDualSolution();
    first_solve = false;

    // 执行控制
    apply_control(x_prev.head(n_u));
}
```

**warm-start 的效果量化**：对于 MPC QP（N=20，n_x=12，n_u=6），warm-start 可以将迭代次数从冷启动的 80-150 次减少到 15-30 次——这对满足实时约束至关重要。

### 3.6 与 ProxSuite 的代码量对比 ⭐⭐

同一问题用 ProxSuite 求解：

```cpp
// ProxSuite: 6 行核心代码
#include <proxsuite/proxqp/dense/dense.hpp>
proxsuite::proxqp::dense::QP<double> qp(n, n_eq, n_in);
qp.settings.eps_abs = 1e-9;
qp.init(H, g, A_eq, b_eq, C_ineq, l_ineq, u_ineq);
qp.solve();
Eigen::VectorXd x_opt = qp.results.x;
```

OSQP-Eigen 需要约 15 行样板代码（SparseMatrix 组装、data/settings 配置），而 ProxSuite 直接接受 Eigen 密集矩阵。对于密集 QP 场景，ProxSuite 的开发体验明显更好。但 OSQP 在大规模稀疏场景下的优势不可替代。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：混淆 OSQP 底层 CSC 存储和 OSQP-Eigen 的对称矩阵语义
>
> **错误做法**：把 $P$ 当成一般非对称矩阵组装，或在调用 OSQP C 接口 / `update` 接口时把上下三角都塞进底层 CSC 数组
>
> **现象**：低层 OSQP C API 只按上三角解释 Hessian，`Px` 更新数组也必须沿用初始化时的上三角非零模式；如果把完整矩阵的 CSC 数值按普通稀疏矩阵顺序传给低层更新接口，数值会对应到错误的 Hessian 元素。通过 OSQP-Eigen 的 `setHessianMatrix(P)` 时，语义上应提供对称 Hessian，wrapper/版本会按 OSQP 需要抽取或使用三角部分。
>
> **根本原因**：OSQP 的目标函数只接受对称半正定 Hessian，底层存储为了避免重复只保留上三角；OSQP-Eigen 则把 Eigen 矩阵当作这个对称 Hessian 的表达，不应把它理解成"上下三角分别有物理意义"的普通矩阵。
>
> **正确做法**：用 OSQP C 接口或直接更新 CSC 数值时，只初始化和更新上三角非零项，并保持相同 sparsity pattern；用 OSQP-Eigen 时确保 $P=P^\top$，示例中只填上三角是为了和底层 OSQP 存储一致、减少歧义。可用 `P.triangularView<Eigen::Upper>()` 或对称性检查做自检。
>
> **自检方法**：对比 OSQP 结果和 Matlab `quadprog` 或 Python `cvxpy` 的结果

> ⚠️ **编程陷阱**：忘记调用 `makeCompressed()` 就传给 OSQP
>
> **错误做法**：`Eigen::SparseMatrix` 在 `insert()` 后处于 "uncompressed" 状态，直接传给 OSQP
>
> **现象**：段错误（segfault）或静默错误结果
>
> **根本原因**：OSQP 的 C 接口要求 CSC 格式——`outerIndexPtr()`、`innerIndexPtr()`、`valuePtr()` 必须指向连续内存。未压缩的 SparseMatrix 内部是链表结构，这些指针指向碎片化内存
>
> **正确做法**：每次修改 SparseMatrix 后调用 `.makeCompressed()`

### 练习

1. ⭐ 用 OSQP-Eigen 求解以下 QP：$\min \frac{1}{2}(x_1^2 + 2x_2^2) + x_1 + x_2 \text{ s.t. } x_1 + x_2 = 1, x_1 \geq 0, x_2 \geq 0$。手算最优解并验证。
2. ⭐⭐ 把同一问题用 Python 的 `cvxpy` 建模并求解。对比 cvxpy 的声明式 API（`cp.Variable`, `cp.Minimize`, `constraints=[...]`）和 OSQP-Eigen 的矩阵 API——哪个更直观？为什么 C++ 求解器不采用声明式接口？（提示：运行时开销、模板推导困难）
3. ⭐⭐ 对比冷启动和 warm-start 的迭代次数差异。构造一系列参数逐步变化的 QP（模拟 MPC 滚动），记录每次求解的迭代次数并画出趋势图。

---

## 4. ProxQP——Pinocchio 生态的新一代 QP 求解器 ⭐⭐

### 4.1 动机：为什么 Pinocchio 生态需要自己的 QP 求解器 ⭐⭐

回顾 M01：Pinocchio 是 INRIA 学派的动力学内核。TSID（基于 Pinocchio 的全身控制框架）最初使用 eiquadprog 作为 QP 后端——这是一个 Eigen 接口的双精度 QP 求解器，性能尚可但精度有限。

随着 TSID 和 Crocoddyl 的用户增长，社区发现了两个痛点：

1. **eiquadprog 不支持不等式约束的高精度求解**——在 eps=1e-9 精度下经常报 "no solution found"
2. **OSQP 的 C 接口与 Pinocchio 的 Eigen 生态不匹配**——需要 osqp-eigen 中间层，且密集 QP 强制用稀疏格式存储造成开销

ProxSuite 团队（INRIA Simple-Robotics，与 Pinocchio 同一团队）在 2022 年推出 ProxQP 正是为了解决这两个问题——一个 Eigen 原生、高精度、为机器人 QP 尺寸优化的求解器。

### 4.2 ProxQP 的 C++ 集成 ⭐⭐

ProxSuite 提供密集（`dense`）和稀疏（`sparse`）两种接口。对于机器人 QP（通常 <500 变量），密集版本性能更好：

```cpp
#include <proxsuite/proxqp/dense/dense.hpp>
using namespace proxsuite::proxqp;

// 问题尺寸
int n = 7;      // 7 关节
int n_eq = 0;   // 无等式约束
int n_in = 7;   // 7 个双边速度界

// 构造问题数据 (全部 Eigen 密集矩阵!)
Eigen::MatrixXd H = J.transpose() * J
                   + lambda * Eigen::MatrixXd::Identity(n, n);
Eigen::VectorXd g = -J.transpose() * v_desired;

// 不等式约束: dq_min <= dq <= dq_max
// ProxQP 的不等式接口是 l <= C*x <= u。
Eigen::MatrixXd C(n_in, n);
C.setIdentity();
Eigen::VectorXd l_in = dq_min;
Eigen::VectorXd u_in = dq_max;

// 创建求解器并求解
dense::QP<double> qp(n, n_eq, n_in);
qp.settings.eps_abs = 1e-9;
qp.settings.max_iter = 500;
qp.init(H, g, std::nullopt, std::nullopt,
        C, l_in, u_in);
qp.solve();

// 提取结果
if (qp.results.info.status == QPSolverOutput::PROXQP_SOLVED) {
    Eigen::VectorXd dq_opt = qp.results.x;
    std::cout << "Solved in " << qp.results.info.iter
              << " iterations, " << qp.results.info.run_time
              << " seconds" << std::endl;
}
```

**关键 API 差异（对比 OSQP）**：

| 维度 | OSQP-Eigen | ProxSuite |
|------|-----------|-----------|
| 矩阵格式 | `Eigen::SparseMatrix` | `Eigen::MatrixXd`（密集）或 `SparseMat`（稀疏） |
| 约束形式 | $l \leq Ax \leq u$（等式通常写成 $l=u$） | $Ax = b$（等式）+ $l \leq Cx \leq u$（双边不等式） |
| 初始化 | 分步 `setXxx()` | 单次 `init()` |
| warm-start | `setWarmStart(x, y)` | `qp.results.x = x_prev; qp.solve()` |

### 4.3 ProxQP vs OSQP：工程决策矩阵 ⭐⭐

| 维度 | ProxQP | OSQP |
|------|--------|------|
| **API 风格** | Eigen 原生密集/稀疏 | C 接口 + osqp-eigen 封装 |
| **问题规模** | 中小规模密集 (<500 变量) | 大规模稀疏 (>500 变量) |
| **精度** | 高精度稳定 (eps=1e-9) | 高精度时收敛变慢 |
| **warm-start** | 支持 | 支持 |
| **代码生成** | 不支持 | 支持 (osqp-codegen) |
| **嵌入式** | 需要 C++17 | C 核心，嵌入式友好 |
| **社区** | Pinocchio/TSID/Crocoddyl | Drake/Autoware/通用 |
| **维护** | 活跃（2022-至今） | 活跃 |

**决策规则**：
- 如果你在 Pinocchio/TSID/Crocoddyl 生态中 → ProxQP
- 如果你在 Drake/Autoware 生态中 → OSQP
- 如果你需要嵌入式部署（C/C++11）→ OSQP 或 qpOASES
- 如果你做 MPC 且预测时域 N>10 → HPIPM（结构化求解）

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：混淆 ProxQP 与 OSQP 的矩阵组织方式
>
> **OSQP**：把等式和不等式统一放进一个矩阵，写成 $l \leq Ax \leq u$；等式用 $l_i=u_i$ 表达。
> **ProxQP**：等式单独传 `A, b`，不等式单独传 `C, l, u`，同样支持 $l \leq Cx \leq u$；单边约束用 `±inf` 边界表达。
>
> **现象**：把 OSQP 的整体约束矩阵原样塞进 ProxQP 的 `C`，或者忘记把等式拆到 `A,b`，会导致维度、对偶变量和 warm-start 都对不上。
>
> **正确做法**：先写一个小测试用例（2 变量、1 个等式、1 个双边不等式）验证 `A,b,C,l,u` 的维度和符号，再迁移真实问题。

> 💡 **概念误区**：认为 ProxQP 的"近端"指的是"近似"
>
> **新手想法**："近端 = 近似，ProxQP 是一个近似求解器？"
>
> **实际上**："近端"（proximal）来自近端算子理论（proximal operator），是优化理论的标准术语。近端算子 $\text{prox}_f(v) = \arg\min_x \{f(x) + \frac{1}{2}\|x-v\|^2\}$ 并非"近似"，而是一种精确的数学运算。ProxQP 在有限次迭代后给出的是精确解（到 eps 精度），不是近似解

### 练习

1. ⭐ 用 ProxQP 求解 M05 §3 中同一个 QP。对比代码行数和运行时间。
2. ⭐⭐ 在 Pinocchio 中计算 Franka Panda 在某个关节构型下的 Jacobian $J$，然后用 ProxQP 求解瞬态 IK QP。测量求解时间，验证是否接近论文中报告的 24 us。
3. ⭐⭐⭐ 阅读 ProxSuite 源码 `include/proxsuite/proxqp/dense/solver.hpp`，找到近端增广拉格朗日的主循环。标注外层 ALM 迭代、内层 Newton 求解、近端参数 $\mu$ 的调整策略。

---

## 5. qpOASES 与 HPIPM——遗留经典与结构化专家 ⭐⭐

### 5.1 qpOASES——活跃集法的工程标杆 ⭐⭐

**历史地位**：qpOASES 由 Ferreau 等人（2014）在 KU Leuven 开发，是第一个专门为嵌入式 MPC 设计的开源 QP 求解器。MIT Cheetah 3 的凸 MPC、acados 的早期版本、大量无人机 MPC 代码都使用 qpOASES。

**C++ 接口特点**（裸数组，无 Eigen）：

```cpp
#include <qpOASES.hpp>
using namespace qpOASES;

// qpOASES 用行优先密集矩阵 (double 数组)
real_t H[2*2] = { 4.0, 1.0,
                  1.0, 2.0 };
real_t g[2]   = { 1.0, 1.0 };
real_t A[1*2] = { 1.0, 1.0 };
real_t lbA[1] = { 1.0 };
real_t ubA[1] = { 1.0 };
real_t lb[2]  = { 0.0, 0.0 };
real_t ub[2]  = { INFTY, INFTY };

int nWSR = 100;  // 最大工作集重计算次数
QProblem qp(2, 1);  // 2 变量, 1 约束
qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

real_t x_opt[2];
qp.getPrimalSolution(x_opt);
```

**与现代 C++ 的差距**：裸指针、C 风格数组、无 RAII、行优先存储（而 Eigen 默认列优先）——在新项目中不推荐直接使用。但理解其 API 对于阅读 MIT Cheetah、acados、rpg_mpc 等遗留代码至关重要。

### 5.2 HPIPM——MPC 结构化 QP 的隐藏冠军 ⭐⭐⭐

**HPIPM 不是"又一个 QP 求解器"——它是专门为 MPC 设计的结构化求解器。** 理解 HPIPM 需要理解 MPC QP 的特殊结构。

**MPC QP 的带状稀疏结构**：

```
KKT 矩阵结构:

  ┌─────┬─────┬─────┬─────┐
  │ K_0 │ B_0 │     │     │
  ├─────┼─────┼─────┼─────┤
  │B_0^T│ K_1 │ B_1 │     │
  ├─────┼─────┼─────┼─────┤
  │     │B_1^T│ K_2 │ B_2 │
  ├─────┼─────┼─────┼─────┤
  │     │     │B_2^T│ K_3 │
  └─────┴─────┴─────┴─────┘

  ← 带状稀疏! 每个 K_t 是 (n_x+n_u)x(n_x+n_u), B_t 是动力学耦合
```

通用 QP 求解器忽略这个结构，HPIPM 用 **Riccati 递推**利用它：

| 方法 | 复杂度 | N=20, n_x=12 的相对时间 |
|------|--------|---------------------|
| 通用稀疏求解 | $O(N^2 n^3)$ | 1x (baseline) |
| HPIPM Riccati | $O(N n^3)$ | **~0.05x (20 倍加速)** |

**工程限制**：HPIPM 的 C API 非常底层——需要用 BLASFEO 的数据结构。推荐通过 **acados** 间接使用（M08 轨迹优化章节会详细讲 acados 工作流）。

### ⚠️ 常见陷阱

> 🧠 **思维陷阱**：认为"通用求解器总是安全选择"
>
> **新手想法**："OSQP 什么 QP 都能解，何必学 HPIPM 这种专用求解器？"
>
> **实际上**：对于 MPC QP，HPIPM 比 OSQP 快 10-20 倍。在 1 kHz 控制循环中，这个差距决定了你能用多长的预测时域——HPIPM 允许 N=50（500 ms 前瞻），而 OSQP 在同样时间内只能做 N=5（50 ms 前瞻）。预测时域直接影响 MPC 的前瞻能力和鲁棒性
>
> **正确思维**：先识别问题结构（是否有带状稀疏？是否是多阶段时域？），然后选择能利用这个结构的求解器

### 练习

1. ⭐ 手写一个 2 状态、1 控制、5 步时域的 MPC QP。展开所有变量，写出完整的 Hessian $P$ 和约束矩阵 $A$。画出非零元素模式。
2. ⭐⭐ 对上述 MPC QP，分别用 OSQP 和 HPIPM（通过 acados Python 接口）求解。对比求解时间。增加时域步数到 N=50，观察时间增长趋势（OSQP: 超线性；HPIPM: 线性）。
3. ⭐⭐⭐ 对 Riccati 递推公式手动展开 N=3 的情况。验证反向扫描得到的 $P_0$ 等价于直接求解完整 KKT 系统得到的解。

---

## 6. Ipopt——NLP 的事实标准 ⭐⭐

### 6.1 从 QP 到 NLP——当线性约束不够用时 ⭐⭐

上面四个求解器处理的都是 QP——二次目标 + 线性约束。但机械臂的许多问题天然是非线性的：

| 问题 | 非线性来源 | 能用 QP 近似吗？ |
|------|-----------|----------------|
| 位姿级 IK | $\text{FK}(q) = T_{target}$ 是 $q$ 的非线性函数 | ✅ 线性化后用 QP 迭代 |
| 轨迹优化 | 动力学 $M(q)\ddot{q} + h(q,\dot{q}) = \tau$ 非线性 | ⚠️ 可以，但线性化误差大 |
| 碰撞约束 | 距离函数 $d(q)$ 非线性（尤其在接近碰撞时） | ❌ 线性化不准确 |
| 非线性 MPC | 模型非线性 + 状态约束非线性 | ❌ SQP 需要 NLP 子问题 |

当 QP 近似不够准确时，需要直接求解 NLP。Ipopt（Interior Point OPTimizer）是这个领域的事实标准——由 CMU 的 Andreas Waechter 开发，2004 年发布，至今仍是学术界和工业界使用最广泛的开源 NLP 求解器。

### 6.2 Ipopt 的内点法原理 ⭐⭐⭐

Ipopt 使用**原始-对偶内点法**。核心迭代：在每个障碍参数 $\mu$ 值下，用 Newton 法求解修正的 KKT 系统：

$$\begin{bmatrix} W + \Sigma & A^T & C^T \\ A & 0 & 0 \\ C & 0 & -\text{diag}(s/z) \end{bmatrix} \begin{bmatrix} \Delta x \\ \Delta \lambda \\ \Delta z \end{bmatrix} = -\begin{bmatrix} \nabla f + A^T\lambda + C^T z \\ g(x) \\ h(x) + s - \mu/z \end{bmatrix}$$

其中 $W = \nabla^2_{xx} L$ 是 Lagrangian 的 Hessian。**收敛策略**：$\mu$ 从大值逐步减小到 0——每个 $\mu$ 值下做几次 Newton 迭代。

### 6.3 Ipopt 的 C++ 接口——TNLP 虚函数模式 ⭐⭐

**Ipopt 的 API 风格与 Ceres 截然不同**——Ceres 是模板仿函数 `operator()`，Ipopt 是虚函数继承 `TNLP`：

```cpp
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>

class ArmTrajectoryNLP : public Ipopt::TNLP {
public:
    // === 必须实现的纯虚函数（至少 8 个：含 eval_h 和 finalize_solution）===
    // 若使用 L-BFGS 近似 Hessian，可在 eval_h 中返回 false 并设置
    // options: "hessian_approximation" -> "limited-memory"

    // 1. 问题维度
    bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m,
                      Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag,
                      IndexStyleEnum& index_style) override {
        n = num_joints * num_timesteps;  // 变量数
        m = num_dynamics_constraints;     // 约束数
        nnz_jac_g = ...;  // 约束 Jacobian 非零元素数
        nnz_h_lag = ...;  // Lagrangian Hessian 非零元素数
        index_style = TNLP::C_STYLE;
        return true;
    }

    // 2. 变量和约束边界
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l,
                         Ipopt::Number* x_u,
                         Ipopt::Index m, Ipopt::Number* g_l,
                         Ipopt::Number* g_u) override {
        for (int i = 0; i < n; ++i) {
            x_l[i] = joint_lower[i % num_joints];
            x_u[i] = joint_upper[i % num_joints];
        }
        for (int j = 0; j < m; ++j) {
            g_l[j] = 0.0;  // 等式约束
            g_u[j] = 0.0;
        }
        return true;
    }

    // 3-7: get_starting_point, eval_f, eval_grad_f, eval_g, eval_jac_g
    // 8: eval_h (Lagrangian Hessian，若用 L-BFGS 可返回 false)
    // 9: finalize_solution (求解完成回调，可为空实现)
    // (结构类似, 需要用户提供函数值和 Jacobian/Hessian 的数值)
};
```

### 6.4 ifopt——Ipopt 的 Eigen 友好封装 ⭐⭐

直接使用 Ipopt 的 `TNLP` 接口工作量大。ETH ADRL 的 **ifopt**（`ethz-adrl/ifopt`，838 stars）提供了 Eigen 接口封装，使 NLP 构建变得模块化：

```cpp
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

// 定义变量
class MyVariables : public ifopt::VariableSet {
public:
    MyVariables() : VariableSet(2, "q") {
        x_ = Eigen::Vector2d::Zero();
    }
    void SetVariables(const VectorXd& x) override { x_ = x; }
    VectorXd GetValues() const override { return x_; }
    VecBound GetBounds() const override {
        return { {-M_PI, M_PI}, {-M_PI, M_PI} };
    }
private:
    Eigen::Vector2d x_;
};

// 定义代价
class MyCost : public ifopt::CostTerm {
public:
    MyCost() : CostTerm("cost") {}
    double GetCost() const override {
        auto x = GetVariables()->GetComponent("q")->GetValues();
        // FK 误差的范数
        Eigen::Vector2d p = forward_kinematics(x);
        return (p - p_target).squaredNorm();
    }
    void FillJacobianBlock(std::string var_set, Jacobian& jac) const override {
        auto x = GetVariables()->GetComponent("q")->GetValues();
        // 手动填 Jacobian (或用有限差分)
        Eigen::Matrix2d J_fk = compute_jacobian(x);
        Eigen::Vector2d p = forward_kinematics(x);
        Eigen::Vector2d grad = 2.0 * J_fk.transpose() * (p - p_target);
        jac.coeffRef(0, 0) = grad(0);
        jac.coeffRef(0, 1) = grad(1);
    }
};

// 组装并求解
ifopt::Problem nlp;
nlp.AddVariableSet(std::make_shared<MyVariables>());
nlp.AddCostSet(std::make_shared<MyCost>());

ifopt::IpoptSolver solver;
solver.SetOption("print_level", 0);
solver.SetOption("linear_solver", "mumps");  // 或 "ma57"
solver.Solve(nlp);
```

ifopt 的设计理念："每个 VariableSet / CostTerm / ConstraintSet 独立实现，通过 `AddXxxSet` 组装"——这种模块化设计使得复杂 NLP 的构建变得可管理。TOWR（ETH 的腿足轨迹优化器）正是基于 ifopt 构建的。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：Ipopt 的线性求解器选择对性能影响巨大
>
> **默认行为**：Ipopt 默认使用 MUMPS 开源稀疏求解器
>
> **现象**：性能比预期慢 2-5 倍
>
> **根本原因**：HSL MA57（商业，学术免费申请）在中小规模 NLP 上显著快于 MUMPS。对于机械臂轨迹优化（100-1000 变量），MA57 可以快 3 倍
>
> **正确做法**：申请 HSL 学术许可证（www.hsl.rl.ac.uk，免费），编译 Ipopt 时链接 HSL。设置 `solver.SetOption("linear_solver", "ma57")`

> 💡 **概念误区**：认为 NLP 的局部最优是"失败"
>
> **新手想法**："Ipopt 只能找到局部最优解，不够好——应该找全局最优"
>
> **实际上**：对于机械臂轨迹优化这类非凸 NLP，全局最优是 NP-hard 问题。实际工程中，一个好的局部最优解（满足所有约束、轨迹光滑、力矩不超限）就是可用的解。关键是提供好的初始猜测——线性插值、上一次优化结果、或 RRT 规划的粗糙路径
>
> **正确思维**：与其追求全局最优，不如投入精力在初始猜测策略上

### 练习

1. ⭐ 用 ifopt 求解 2-DOF 平面机械臂到达目标点的 IK 问题。对比 Pinocchio 数值 IK 和 Ipopt NLP 的解。
2. ⭐⭐ 同一问题分别用 MUMPS 和 MA57 线性求解器，记录迭代次数和总时间对比。
3. ⭐⭐⭐ 用 Ipopt 直接 API（继承 `TNLP`）求解 3-DOF 机械臂 10 步轨迹优化。手动提供 Jacobian 稀疏结构。

---

## 7. CasADi——符号建模框架 ⭐⭐

### 7.1 动机：为什么需要符号框架 ⭐⭐

上一节暴露了 Ipopt 使用的一个痛点：用户必须**手动**提供 Jacobian 和 Hessian 的稀疏结构和数值。对于 7-DOF 机械臂 + 50 步时域的 MPC（变量数 350，约束数可能达数百），手写 Jacobian 几乎不可行。

**CasADi 解决的正是这个问题**——它不是求解器，而是**符号建模框架**：

```
CasADi 工作流:
  1. 用 SX/MX 符号变量写数学表达式 (像写数学公式一样自然)
  2. CasADi 自动推导梯度、Jacobian、Hessian (符号自动微分)
  3. 可选: 生成 C 代码 (NLP 的函数评估, 不含求解器)
  4. 链接到 Ipopt/qpOASES/HPIPM 求解器
```

> **跨领域类比**：CasADi 之于 Ipopt，就像 TensorFlow 之于 CUDA——TensorFlow 不直接做矩阵乘法，它构建计算图、自动微分、然后把计算分派给 CUDA/CPU。同样，CasADi 不直接求解优化问题，它构建符号表达式、自动微分、然后把 NLP 分派给 Ipopt 或其他求解器。这种"建模层 + 求解层"的分离是现代 MPC 工程的标准架构。

### 7.2 SX vs MX——两种符号表示 ⭐⭐

CasADi 提供两种符号变量类型，选择取决于问题规模：

| 类型 | 粒度 | 内存 | 速度特征 | 适用场景 |
|------|------|------|---------|---------|
| **SX** | 标量级 | 大 | 微分快，评估慢 | 小规模 (<100 变量) |
| **MX** | 矩阵级 | 小 | 微分慢，评估快 | 大规模 MPC (>100 变量) |

**SX 示例**（小规模 NLP）：

```python
import casadi as ca

# SX: 标量符号
x = ca.SX.sym('x', 2)  # 2 维向量
f = x[0]**2 + x[1]**2   # 标量目标
g = x[0] + x[1] - 1     # 等式约束

# 自动微分
J = ca.jacobian(g, x)    # 1x2 Jacobian: [1, 1]
H = ca.hessian(f, x)[0]  # 2x2 Hessian: [[2,0],[0,2]]

# 构建并求解 NLP
nlp = {'x': x, 'f': f, 'g': g}
solver = ca.nlpsol('solver', 'ipopt', nlp,
                   {'ipopt.print_level': 0})
sol = solver(x0=[0.5, 0.5], lbg=0, ubg=0)
print(f"x* = {sol['x']}")  # [0.5, 0.5]
```

**MX 示例**（MPC 轨迹优化）：

```python
import casadi as ca

N = 20        # 预测步数
n_x = 14      # 状态: [q, dq] 各 7 维
n_u = 7       # 控制: 关节力矩

# 构建决策变量
X = ca.MX.sym('X', n_x, N + 1)
U = ca.MX.sym('U', n_u, N)

# 动力学 (简化: 双积分器)
def dynamics(x, u, dt=0.01):
    q, dq = x[:7], x[7:]
    ddq = u  # 简化
    return ca.vertcat(q + dq*dt + 0.5*ddq*dt**2,
                      dq + ddq*dt)

# 组装目标和约束
cost = 0
constraints = []
for t in range(N):
    cost += ca.sumsqr(X[:, t] - x_ref[:, t]) + 0.01 * ca.sumsqr(U[:, t])
    constraints.append(X[:, t+1] - dynamics(X[:, t], U[:, t]))

# 构建 NLP
opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
nlp = {'x': opt_vars, 'f': cost, 'g': ca.vertcat(*constraints)}
solver = ca.nlpsol('solver', 'ipopt', nlp)
```

### 7.3 代码生成——CasADi 到 C 的桥梁 ⭐⭐

CasADi 的 `CodeGenerator` 可以将符号函数导出为纯 C 代码：

```python
# 构建函数
f_eval = ca.Function('f_eval', [x], [f, ca.gradient(f, x)])

# 生成 C 代码
cg = ca.CodeGenerator('my_nlp.c',
                       {'main': False, 'mex': False, 'with_header': True})
cg.add(f_eval)
cg.generate()
```

生成的 C 代码包含目标函数、梯度、Jacobian 的数值评估函数——**不含求解器**。如果需要完整的"求解器 + 函数评估"自包含代码（用于嵌入式部署），应使用 **acados**（见 M08 轨迹优化）。

### 7.4 CasADi + Pinocchio 的接口 ⭐⭐⭐

Pinocchio 3.x 支持 CasADi 标量类型——这意味着可以用 CasADi 符号作为 Pinocchio 算法的输入，获得动力学函数的符号表达式：

```python
import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as ca

# 加载模型
model = pin.buildModelFromUrdf("panda.urdf")
cmodel = cpin.Model(model)
cdata = cmodel.createData()

# CasADi 符号关节角
q_sym = ca.SX.sym('q', model.nq)
v_sym = ca.SX.sym('v', model.nv)
a_sym = ca.SX.sym('a', model.nv)

# 符号化 RNEA -- Pinocchio 的算法对 CasADi 标量透明!
cpin.rnea(cmodel, cdata, q_sym, v_sym, a_sym)
tau_sym = cdata.tau  # 得到 tau 的符号表达式

# 自动微分: 解析 Jacobian!
dtau_dq = ca.jacobian(tau_sym, q_sym)  # 7x7 矩阵, 每个元素是 q 的函数
```

这个接口使得"Pinocchio 动力学 + CasADi 自动微分 + Ipopt 求解"的三件套成为现代机械臂 MPC 的标准工作流。

### 7.5 CasADi 实战：完整 7-DOF 轨迹优化 Pipeline ⭐⭐⭐

下面给出一个工程级别的完整示例——用 CasADi 构建 Franka Panda 的多路径点轨迹优化 NLP，包含关节限位、速度限制和平滑性约束。这不是教学简化版，而是可以直接用于实际项目的模板。

```python
import casadi as ca
import numpy as np

# ========== 问题参数 ==========
n_joints = 7          # Franka Panda 7-DOF
N = 30                # 离散化步数
dt = 0.05             # 步长 50ms，总时长 1.5s

# 关节限位（Franka Panda 官方参数）
q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718,
                  -2.8973, -0.0175, -2.8973])
q_max = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,
                   2.8973,  3.7525,  2.8973])
dq_max = np.array([2.175, 2.175, 2.175, 2.175,
                   2.610, 2.610, 2.610])  # rad/s
ddq_max = np.array([15.0, 7.5, 10.0, 12.5,
                    15.0, 20.0, 20.0])    # rad/s^2

# ========== 构建决策变量 ==========
opti = ca.Opti()  # CasADi 的高级 API（比 nlpsol 更简洁）
Q = opti.variable(n_joints, N + 1)   # 关节角度 q(0)..q(N)
dQ = opti.variable(n_joints, N)      # 关节速度 dq(0)..dq(N-1)

# ========== 约束 ==========
# 1. 运动学一致性：q(t+1) = q(t) + dq(t) * dt
for t in range(N):
    opti.subject_to(Q[:, t+1] == Q[:, t] + dQ[:, t] * dt)

# 2. 关节限位
for t in range(N + 1):
    opti.subject_to(opti.bounded(q_min, Q[:, t], q_max))

# 3. 速度限制
for t in range(N):
    opti.subject_to(opti.bounded(-dq_max, dQ[:, t], dq_max))

# 4. 加速度限制（有限差分近似）
for t in range(N - 1):
    ddq_approx = (dQ[:, t+1] - dQ[:, t]) / dt
    opti.subject_to(opti.bounded(-ddq_max, ddq_approx, ddq_max))

# 5. 初始和终端约束
q_start = np.array([0.0, -0.785, 0.0, -2.356,
                    0.0, 1.571, 0.785])
q_goal  = np.array([1.0, -0.5, 0.5, -1.5,
                    0.3, 1.2, 0.5])
opti.subject_to(Q[:, 0] == q_start)
opti.subject_to(Q[:, N] == q_goal)
opti.subject_to(dQ[:, 0] == 0)    # 起始静止
opti.subject_to(dQ[:, N-1] == 0)  # 终端静止

# ========== 目标函数 ==========
# 最小化：控制能量 + 加速度平滑性
cost = 0
w_effort = 1.0     # 控制能量权重
w_smooth = 10.0    # 平滑性权重

for t in range(N):
    cost += w_effort * ca.sumsqr(dQ[:, t])
for t in range(N - 1):
    ddq_approx = (dQ[:, t+1] - dQ[:, t]) / dt
    cost += w_smooth * ca.sumsqr(ddq_approx)

opti.minimize(cost)

# ========== 求解器配置 ==========
p_opts = {'expand': True}  # 展开 MX 表达式以加速
s_opts = {
    'print_level': 3,
    'max_iter': 500,
    'tol': 1e-6,
    'linear_solver': 'mumps',   # 默认；有 HSL 则用 'ma57'
    'warm_start_init_point': 'yes',
}
opti.solver('ipopt', p_opts, s_opts)

# 初始猜测：线性插值（关键！好的初始猜测决定收敛速度）
for t in range(N + 1):
    alpha = t / N
    opti.set_initial(Q[:, t],
                     (1 - alpha) * q_start + alpha * q_goal)

sol = opti.solve()

# ========== 提取结果 ==========
q_traj = sol.value(Q)   # (7, 31) numpy 数组
dq_traj = sol.value(dQ)  # (7, 30) numpy 数组
print(f"最优代价: {sol.value(cost):.4f}")
print(f"Ipopt 迭代次数: {sol.stats()['iter_count']}")
```

**为什么用 `ca.Opti()` 而非底层 `ca.nlpsol()`？**

| 特性 | `ca.Opti()` 高级 API | `ca.nlpsol()` 底层 API |
|------|---------------------|----------------------|
| 约束表达 | `opti.subject_to(x <= 1)` 自然 | 需要手动组装 `g` 向量和 `lbg/ubg` |
| 初始猜测 | `opti.set_initial(x, val)` 逐变量设 | 需要组装完整 `x0` 向量 |
| 调试 | `opti.debug.value(x)` 查看中间值 | 不支持 |
| warm-start | `opti.set_initial(x, sol.value(x))` | 手动传入 `x0=prev_sol` |
| 适用场景 | 交互式建模、研究原型 | 高频 MPC（需要最小开销） |

**经验法则**：研究和原型阶段用 `Opti()`，部署到实时 MPC 时切换到 `nlpsol()` + 代码生成。

### 7.6 Ipopt 参数调优实战手册 ⭐⭐⭐

Ipopt 有 100+ 个可调参数，但实际工程中只有 10 个左右对性能有显著影响。以下是基于大量机械臂优化实践总结的参数调优表：

**核心参数表**：

| 参数 | 默认值 | 机械臂推荐值 | 效果 | 何时调整 |
|------|-------|-------------|------|---------|
| `linear_solver` | `mumps` | `ma57`（需 HSL） | **最重要**：性能差 2-5x | 申请 HSL 后立刻切换 |
| `tol` | `1e-8` | `1e-4` ~ `1e-6` | 降低精度换速度 | 实时 MPC 场景 |
| `max_iter` | 3000 | 100 ~ 500 | 防止超时 | 有实时性要求时 |
| `mu_strategy` | `monotone` | `adaptive` | 自适应障碍参数更新 | 收敛振荡时 |
| `warm_start_init_point` | `no` | `yes` | 利用上一次解 | MPC 滚动求解 |
| `warm_start_bound_push` | 1e-2 | 1e-6 | warm-start 时松弛变量初始化 | warm-start 不稳定时 |
| `warm_start_bound_frac` | 1e-2 | 1e-6 | 配合 bound_push 使用 | 同上 |
| `print_level` | 5 | 0 | 关闭输出减少 I/O 开销 | 部署/benchmark 时 |
| `mehrotra_algorithm` | `no` | `yes` | 使用 Mehrotra 预测-校正步 | 中小规模问题 |
| `nlp_scaling_method` | `gradient-based` | `gradient-based` | 自动缩放 NLP | 通常保持默认 |

**线性求解器对比**（同一个 7-DOF 50 步轨迹优化问题）：

| 线性求解器 | 可用性 | 总时间 | 单次因式分解 | 适用规模 |
|-----------|-------|-------|------------|---------|
| MUMPS | 开源，apt 安装 | 85ms | 2.1ms | 通用 |
| MA27 | HSL 学术免费 | 52ms | 1.3ms | 中小规模 |
| MA57 | HSL 学术免费 | 38ms | 0.9ms | 中小规模（**推荐**） |
| MA86 | HSL 学术免费 | 41ms | 1.0ms | 大规模（多线程） |
| Pardiso | Intel MKL 内置 | 35ms | 0.8ms | 大规模+Intel 平台 |

> **反事实推理**：如果你不调优 Ipopt 参数，直接使用默认配置会怎样？默认 MUMPS + tol=1e-8 + monotone mu + 无 warm-start，在实时 MPC 场景下求解时间可能是调优后的 5-10 倍。一个本来可以 50 Hz 运行的 MPC 降到 5-10 Hz，控制性能急剧下降。参数调优不是"锦上添花"——在实时系统中，它是"能否工作"的决定因素。

### 7.7 QP 求解器 Warm-Start 策略详解 ⭐⭐⭐

Warm-start（热启动）是 MPC 实时性的关键技术。其核心思想是：MPC 在相邻时刻求解的 QP/NLP 只有微小差异（初始状态变了、参考轨迹移了一步），因此上一时刻的最优解是当前时刻的极好初始猜测。

**三层 Warm-Start 策略**：

| 层级 | 传递的信息 | 适用求解器 | 加速效果 |
|------|-----------|-----------|---------|
| **L1: 原始变量** | 上一时刻的 $x^*$ | 所有求解器 | 2-5x |
| **L2: 原始+对偶** | $x^*, \lambda^*, \mu^*$ | ProxQP, OSQP | 5-10x |
| **L3: 活跃集** | $x^*, \lambda^*, \mu^*$ + 活跃约束集合 $\mathcal{W}^*$ | qpOASES | 10-50x |

**OSQP Warm-Start 实现**：

```cpp
// MPC 循环中的 OSQP warm-start
OsqpEigen::Solver solver;
solver.settings()->setWarmStart(true);
Eigen::VectorXd prev_primal, prev_dual;

// 首次求解（冷启动）
solver.initSolver();
solver.solveProblem();  // 可能需要 50-200 次 ADMM 迭代
prev_primal = solver.getSolution();
prev_dual = solver.getDualSolution();

// 后续求解（warm-start）
for (int mpc_step = 1; mpc_step < horizon; ++mpc_step) {
    // 更新参数（状态变了，参考轨迹移了一步）
    update_linear_cost(solver, new_q_ref);
    update_bounds(solver, new_state);

    // 设置 warm-start 初始值
    solver.setWarmStart(prev_primal, prev_dual);

    // 求解（warm-start 通常只需 5-20 次迭代）
    solver.solveProblem();

    prev_primal = solver.getSolution();
    prev_dual = solver.getDualSolution();
}
```

**ProxQP Warm-Start 实现**：

```cpp
proxsuite::proxqp::dense::QP<double> qp(n, n_eq, n_in);
qp.settings.initial_guess =
    proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

// 首次求解
qp.init(H, g, A, b, C, l, u);
qp.solve();

// 后续求解
for (int step = 0; step < mpc_steps; ++step) {
    // 只更新变化的部分（ProxQP 支持稀疏更新）
    qp.update(H_new, g_new,
              std::nullopt, std::nullopt,  // 等式约束不变
              std::nullopt, l_new, u_new); // 只更新不等式边界
    qp.solve();
    // ProxQP 自动使用上一次的 x, y, z 作为初始值
}
```

**MPC 时域移位（Shift）技巧**：

滚动时域 MPC 每个周期前移一步。最常用的 warm-start 初始化策略是**时域移位**：

```
时刻 t 的最优解:   [u*_0, u*_1, u*_2, ..., u*_{N-1}]
时刻 t+1 的初始值: [u*_1, u*_2, ..., u*_{N-1}, u*_{N-1}]
                    ↑ 左移一步                    ↑ 最后一步重复

时刻 t 的最优状态:   [x*_0, x*_1, ..., x*_N]
时刻 t+1 的初始状态: [x_measured, x*_2, ..., x*_N, f(x*_N, u*_{N-1})]
                      ↑ 实测值     ↑ 左移一步       ↑ 前向仿真一步
```

```python
# CasADi MPC warm-start 时域移位
def shift_warm_start(prev_X, prev_U, x_measured, dynamics):
    """将上一时刻 MPC 解左移一步作为新的初始猜测"""
    N = prev_U.shape[1]

    # 控制量左移，最后一步重复
    U_init = ca.horzcat(prev_U[:, 1:], prev_U[:, -1])

    # 状态量：第一个用实测值，其余左移，末端前向仿真
    X_init = ca.MX(prev_X.shape[0], N + 1)
    X_init[:, 0] = x_measured
    for t in range(1, N):
        X_init[:, t] = prev_X[:, t + 1]
    X_init[:, N] = dynamics(prev_X[:, N], prev_U[:, -1])

    return X_init, U_init
```

> **本质洞察**：Warm-start 不仅仅是"更好的初始猜测"——对于活跃集法，它直接给出了最优活跃集的近似，将组合搜索从 $2^m$ 降到 $O(1)$；对于 ADMM，它让原始-对偶残差从大值变为小值，减少了迭代次数；对于内点法，它提供了一个接近中心路径的起点。理解了这一点，就知道为什么不同求解器的 warm-start 效果差异很大——活跃集法受益最多（因为组合问题被直接跳过），ADMM 受益最少（因为一阶算法本身不利用高阶信息）。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：混淆 CasADi 符号变量和 NumPy 数组
>
> **错误做法**：`result = np.sin(casadi_sym)` — 对 CasADi 符号用 NumPy 函数
>
> **现象**：TypeError 或静默返回错误结果
>
> **根本原因**：CasADi 的 SX/MX 类型有自己的数学函数库。NumPy 函数只接受数值数组
>
> **正确做法**：始终用 `ca.sin()`, `ca.cos()`, `ca.mtimes()`（矩阵乘法）等 CasADi 函数

> 🧠 **思维陷阱**：把 CasADi 当作"更好的 Ipopt"
>
> **新手想法**："CasADi 比 Ipopt 更高级，所以用 CasADi 就不需要 Ipopt 了"
>
> **实际上**：CasADi 不是求解器——它是符号框架，负责构建表达式、自动微分、代码生成。最终的优化求解仍然由 Ipopt/SNOPT/qpOASES 完成。CasADi + Ipopt 是"建模层 + 求解层"的分离
>
> **正确理解**：CasADi 解决了"如何高效地为 Ipopt 提供梯度信息"的问题，是 Ipopt 的上游工具而非替代品

### 练习

1. ⭐ 用 CasADi Python 构建 2-DOF 平面臂到达目标点的 NLP。用 Ipopt 求解。
2. ⭐⭐ 用 CasADi 的 `ca.jacobian` 和 `ca.hessian` 提取目标函数的梯度和 Hessian。验证 Hessian 在最优解处是半正定的。
3. ⭐⭐⭐ 用 CasADi + Pinocchio（`pinocchio.casadi`）对 Franka Panda 构建符号化 RNEA。提取 $\partial \tau / \partial q$，生成 C 代码并在 C++ 端调用。

---

## 8. 机械臂三大典型优化问题建模模板 ⭐⭐

### 8.1 总览 ⭐⭐

前面各节分别讲了求解器的算法和 API。本节把它们落地到机械臂的三大核心优化问题上——每个问题给出完整的数学建模和 C++ 实现思路。

| 模板 | 决策变量 | 约束 | 求解频率 | 推荐求解器 |
|------|---------|------|---------|-----------|
| IK QP | 关节速度 $\dot{q}$ | 速度限、碰撞、关节限位 | 1 kHz | ProxQP |
| 力控 QP (TSID AccForce) | $\ddot{q}$ + 接触力 $f$（$\tau$ 由动力学恢复） | 接触、摩擦锥、力矩限 | 1 kHz | ProxQP |
| MPC QP | $x_{0:N}$ + $u_{0:N-1}$ | 动力学、控制限、状态限 | 50-200 Hz | HPIPM |

### 8.2 模板 1：瞬态 IK QP ⭐⭐

**问题**：给定当前关节角 $q$ 和目标末端速度 $v_{des}$，求关节速度 $\dot{q}$。

**数学建模**：

$$\min_{\dot{q}} \frac{1}{2}\|J(q)\dot{q} - v_{des}\|^2 + \frac{\lambda}{2}\|\dot{q} - \dot{q}_{nom}\|^2$$

$$\text{s.t.} \quad \dot{q}_{lb} \leq \dot{q} \leq \dot{q}_{ub}$$

$$\quad \frac{q_{lb} - q}{\Delta t} \leq \dot{q} \leq \frac{q_{ub} - q}{\Delta t}$$

**转化为标准 QP**：$P = J^TJ + \lambda I$，$q_{vec} = -J^T v_{des} - \lambda \dot{q}_{nom}$。

**Pinocchio + ProxQP C++ 实现**：

```cpp
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

Eigen::VectorXd solveIKQP(
    const pinocchio::Model& model, pinocchio::Data& data,
    const Eigen::VectorXd& q,
    pinocchio::FrameIndex ee_frame_id,
    const Eigen::VectorXd& v_des,
    double dt, double lambda)
{
    const int nv = model.nv;  // 7 for Panda

    // 计算末端 Frame Jacobian。
    // v_des 是在 LOCAL_WORLD_ALIGNED 中表达的 6D twist：
    // 参考点在末端 frame 原点，线速度/角速度方向与世界坐标系平行。
    Eigen::MatrixXd J(6, nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, ee_frame_id,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);

    // 组装 QP
    Eigen::MatrixXd H = J.transpose() * J
                       + lambda * Eigen::MatrixXd::Identity(nv, nv);
    Eigen::VectorXd g = -J.transpose() * v_des;

    // 约束: 关节速度限 + 关节位置限
    int n_in = 2 * nv;
    Eigen::MatrixXd C(n_in, nv);
    C.topRows(nv) = Eigen::MatrixXd::Identity(nv, nv);
    C.bottomRows(nv) = -Eigen::MatrixXd::Identity(nv, nv);

    Eigen::VectorXd ub(n_in);
    // 速度上限 和 位置限约束取交集 (更紧的那个)
    for (int i = 0; i < nv; ++i) {
        double vel_ub = 2.0;  // 示例速度上限
        double pos_ub = (model.upperPositionLimit(i) - q(i)) / dt;
        ub(i) = std::min(vel_ub, pos_ub);

        double vel_lb = -2.0;
        double pos_lb = (model.lowerPositionLimit(i) - q(i)) / dt;
        ub(nv + i) = std::min(-vel_lb, -pos_lb);
    }

    // ProxQP 求解
    proxsuite::proxqp::dense::QP<double> qp(nv, 0, n_in);
    qp.settings.eps_abs = 1e-8;
    qp.init(H, g, std::nullopt, std::nullopt,
            C, Eigen::VectorXd::Constant(n_in, -1e30), ub);
    qp.solve();

    return qp.results.x;
}
```

### 8.3 模板 2：力控 QP（TSID 风格） ⭐⭐⭐

**问题**：给定期望末端加速度 $a_{des}$，求关节加速度 $\ddot{q}$ 和接触力 $f$，再由动力学关系恢复执行器力矩 $\tau$。

回顾 M01：Pinocchio 的动力学方程为 $M(q)\ddot{q} + h(q,\dot{q}) = \tau + J_c^T f$。

**数学建模**：

$$\min_{\ddot{q}, f} \frac{1}{2}\|J_{task}\ddot{q} + \dot{J}_{task}\dot{q} - a_{des}\|^2 + \frac{w_f}{2}\|f\|^2$$

$$\text{s.t.} \quad J_c\ddot{q} + \dot{J}_c\dot{q} = 0 \quad (\text{无滑移接触})$$
$$\quad \tau_{min} \leq \tau(\ddot q,f) \leq \tau_{max} \quad (\text{力矩限})$$
$$\quad |f_{tangent}| \leq \mu f_{normal} \quad (\text{摩擦锥})$$

其中力矩不是独立优化变量，而是求解后的派生量：

$$\tau(\ddot q,f)=M\ddot{q}+h-J_c^T f \quad (\text{固定基座力矩恢复})$$

> **实现口径**：TSID 的 `InverseDynamicsFormulationAccForce` 以加速度和接触力为优化变量，力矩由求解结果解码得到。若把 $\tau$ 也作为原始决策变量，那是更一般的 inverse-dynamics WBC QP 写法；两者都合理，但不要在同一个公式里混用。

### 8.4 模板 3：MPC QP（滚动时域） ⭐⭐⭐

**密集形式 vs 稀疏形式**：

| 形式 | 变量 | QP 维度 | 适用求解器 |
|------|------|---------|-----------|
| **密集**（消去 $x$，只留 $u$） | $u_{0:N-1}$ | $N \cdot n_u$ | OSQP、ProxQP |
| **稀疏**（保留 $x$ 和 $u$） | $x_{0:N}, u_{0:N-1}$ | $(N+1)n_x + Nn_u$ | HPIPM |

密集形式变量少但 Hessian 密集；稀疏形式变量多但 Hessian 有带状稀疏结构——HPIPM 的 Riccati 递推专门利用这个结构。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：IK QP 中混淆 Jacobian 的参考坐标系
>
> **错误做法**：用 `LOCAL` 参考系的 Jacobian，但 $v_{des}$ 是世界坐标系下的速度
>
> **现象**：末端运动方向错误——机械臂"往奇怪的方向动"
>
> **根本原因**：`LOCAL` Jacobian 映射的是末端坐标系下的速度，`LOCAL_WORLD_ALIGNED` 映射的是世界坐标系下的速度。两者在末端有旋转时差异很大
>
> **正确做法**：确保 Jacobian 参考系和 $v_{des}$ 坐标系一致。推荐使用 `LOCAL_WORLD_ALIGNED`

> 💡 **概念误区**：认为 MPC QP 的终端代价 $Q_f$ 不重要
>
> **新手想法**："终端代价只是最后一步的权重，设成和 $Q$ 一样就行"
>
> **实际上**：$Q_f$ 近似了预测时域之外的代价，是 MPC 稳定性的关键。理论上 $Q_f$ 应设为无穷时域 LQR 的代价矩阵 $P_\infty$（离散代数 Riccati 方程的稳态解）
>
> **正确做法**：用 `dare(A, B, Q, R)` 求解离散 Riccati 方程，用稳态解作为 $Q_f$

### 练习

1. ⭐ 用 Pinocchio + ProxQP 实现 Franka Panda 的瞬态 IK QP。目标：末端沿 z 轴匀速上升。验证求解时间 < 50 us。
2. ⭐⭐ 同一 IK QP 分别用 OSQP、ProxQP、qpOASES 求解。Google Benchmark 对比——验证 ProxQP 约 7 倍于 OSQP。
3. ⭐⭐ 实现简化 MPC QP（双积分器，2 状态 1 控制，N=20）。分别用 OSQP 和 acados+HPIPM 求解对比。
4. ⭐⭐⭐ 阅读 TSID 的 `InverseDynamicsFormulationAccForce::computeProblemData()` 源码，标注动力学如何组装到 QP。

---

## 9. 进阶专题：力分配与碰撞约束 ⭐⭐⭐

### 9.1 力分配 QP——多点接触与夹持 ⭐⭐⭐

当机械臂执行夹持任务时，末端执行器的多个接触点需要协调施力。力分配 QP 确定每个接触点的力，使得合力满足任务需求，同时每个接触力在摩擦锥内。

**摩擦锥的线性化**：圆锥 $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$ 是二阶锥约束（SOCP），QP 求解器无法直接处理。常用做法是用 $k$ 面正多边形内接近似（$k=4$ 或 $k=8$）：

```
   摩擦锥(圆锥)        线性化(k=4)
      /|\               /|\
     / | \             / | \
    /  |  \           /__|__\
   /   |   \         |   |   |
  /____|____\        |___|___|
```

$k=8$ 时最大近似误差 $1 - \cos(\pi/8) \approx 3.8\%$，工程上可以接受。

### 9.2 碰撞回避约束的一阶近似 ⭐⭐⭐

碰撞约束 $d(q) \geq d_{safe}$ 中的 $d(q)$ 是非线性的。一阶 Taylor 近似：

$$d(q + \dot{q}\Delta t) \approx d(q) + \nabla_q d(q)^T \dot{q} \Delta t \geq d_{safe}$$

其中 $\nabla_q d(q) = J_{col}^T \hat{n}$，$J_{col}$ 是碰撞最近点的 Jacobian，$\hat{n}$ 是法向量。Pinocchio 的 Coal（原 hpp-fcl）库可计算这些量。

> **反事实推理**：如果碰撞约束不用一阶近似而是直接作为非线性约束放入 NLP，会怎样？NLP 求解器（Ipopt）可以精确处理，但求解时间从 us 级增长到 ms 级——无法满足 1 kHz 实时要求。工程上的做法是：远距离用代价函数（软约束），近距离用线性化硬约束，通过安全距离阈值 $d_{thres}$ 切换策略。

### ⚠️ 常见陷阱

> 🧠 **思维陷阱**：TrajOpt 把碰撞约束放在 cost 里而非 constraint 里
>
> **新手困惑**：碰撞不是硬约束吗？
>
> **工程原因**：距离函数 $d(q)$ 在远离碰撞时梯度平缓、接近碰撞时梯度剧烈。作为硬约束会导致优化器在约束边界附近震荡。放在 cost 里允许"有代价地接近障碍物"，优化行为更平滑
>
> **最佳实践**：远距离用代价函数，近距离用硬约束，通过阈值切换

### 练习

1. ⭐⭐ 实现 2D 力分配 QP：2 个接触点，每个 2D 力，任务力向下 10 N，摩擦系数 0.5，$k=4$ 线性化。用 ProxQP 求解。
2. ⭐⭐⭐ 在 Pinocchio + Coal 中计算机械臂某 link 到障碍物的最短距离和法向量。构造碰撞回避线性约束，集成到 IK QP 中。

---

## 10. 前沿进展：Pinocchio 3.x 原生 NLP 接口与 Drake/CasADi 2026 对比 ⭐⭐⭐⭐

### 10.1 Pinocchio 3.x 的原生 NLP 接口

Pinocchio 3.x（2024-2026）在 CasADi 和 CppAD 后端之外，新增了与 NLP 求解器更紧密的集成路径。核心改进包括：

- **`pinocchio::casadi` 模块成熟化**：Pinocchio 3.x 全面支持 `ModelTpl<casadi::SX>` 和 `ModelTpl<casadi::MX>`，使得通过 CasADi 直接构建动力学约束的 NLP 变得一行可写。不再需要手动拼接 RNEA 的符号输出——`cpin.rnea(cmodel, cdata, q_sym, v_sym, a_sym)` 返回的 `cdata.tau` 已经是完整的 CasADi 符号表达式
- **约束直接导出**：Pinocchio 3.x 支持将关节限位、速度限、碰撞约束等直接导出为 CasADi 约束表达式，减少了手动约束组装的工作量
- **`pinocchio::MathematicalProgram`（实验性）**：参考 Drake 的 `MathematicalProgram` 抽象，Pinocchio 社区正在开发统一的 NLP 构建接口，允许用 Pinocchio 模型直接组装优化问题并分派到 Ipopt/ProxQP/HPIPM

> **本质洞察**：Pinocchio 3.x 的发展方向是从"动力学计算库"演化为"机器人优化建模平台"。这反映了规控领域的趋势：动力学和优化不再是分离的模块，而是融合为一个统一的"可微动力学 + 约束优化"栈。

### 10.2 Drake vs CasADi 2026 生态对比 ⭐⭐⭐

截至 2026 年，Drake 和 CasADi 是机械臂 NLP 建模的两大主流框架。它们的设计哲学和适用场景截然不同：

| 维度 | Drake (MIT/TRI) | CasADi + Pinocchio |
|------|-----------------|-------------------|
| **语言** | C++（Python 绑定 via pybind11） | Python 优先，C 代码生成 |
| **动力学来源** | MultibodyPlant 内置 | Pinocchio 外部库 |
| **NLP 建模** | `MathematicalProgram` 统一接口 | `ca.Opti()` 或 `ca.nlpsol()` |
| **QP 后端** | OSQP/Gurobi/MOSEK/SCS | Ipopt/qpOASES/HPIPM |
| **NLP 后端** | SNOPT/Ipopt | Ipopt/SNOPT |
| **接触建模** | 原生接触力优化（凸互补） | 需手动添加互补约束 |
| **代码生成** | 不支持 | 支持（C 代码 + acados） |
| **GPU 加速** | 不支持 | 通过 JAX/MJX 间接支持 |
| **嵌入式部署** | 困难（依赖链重） | 通过 acados 成熟 |
| **许可证** | BSD-3 | LGPL-3.0 |
| **典型用户** | TRI、Boston Dynamics、研究机构 | INRIA、工业 MPC、学术界 |

**选型建议**：
- **接触丰富操作**（装配、抓取、多指操控）→ Drake（原生接触力优化是杀手特性）
- **实时 MPC 部署**（1-100 Hz 控制循环）→ CasADi + acados（代码生成是杀手特性）
- **学术研究/快速原型**→ CasADi `Opti()` API（开发效率最高）
- **系统集成/工业认证**→ Drake（更完整的仿真-控制-验证栈）

> **反事实推理**：如果你选择 Drake 做实时 MPC，会遇到什么问题？Drake 的 `MathematicalProgram` 每次调用都重新构建符号图，无法做 CasADi 式的"一次编译、多次调用"代码生成。对于 100 Hz MPC，这意味着每次 solve 都有 1-5 ms 的 NLP 构建开销——这在 CasADi + acados 中被代码生成完全消除（NLP 函数评估直接执行编译后的 C 代码，零构建开销）。反过来，如果你用 CasADi 做接触操控，则需要手动实现互补约束和摩擦锥的数值处理，而 Drake 的 `MultibodyPlant` 原生集成了接触力学。

---

## 11. 与下游章节的接口 ⭐

### 11.1 与 M03 IK 求解器的关系 ⭐

M03 中的 TRAC-IK、BioIK 等高级 IK 求解器内部都使用 QP/SQP 作为子问题。理解本章 QP 建模后，你可以理解 TRAC-IK 为什么同时运行 KDL 数值迭代和 SQP 两个求解器并取较快收敛的结果，也可以自己在 QP 层添加碰撞约束（TRAC-IK 原生不支持）。

### 11.2 与 M08 轨迹优化的关系 ⭐

M08 中的 TrajOpt、Crocoddyl、OCS2 全部基于本章的 NLP/QP 求解器：
- TrajOpt 用序列 QP（SQP）：每次线性化约束然后求解 QP
- Crocoddyl 用 DDP/iLQR：一种特殊 NLP 求解器，利用最优控制的 Bellman 结构
- OCS2 用 SQP + HPIPM：外层 SQP 迭代，内层用 HPIPM 求解结构化 QP

### 11.3 与 M10 时间参数化的关系 ⭐

M10 中的 TOPP-RA 内部在每个路径点求解一个小规模 LP（线性规划，QP 的特例——目标函数是线性的）。理解 QP 后，LP 是自然的简化。

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关键收获 |
|--------|---------|------|---------|
| Ceres→QP/NLP 跨越 | 无约束 NLS → 有约束优化 | ⭐ | 硬约束 vs 软约束的本质区别 |
| QP 算法四大家族 | ADMM / 活跃集 / 内点法 / 近端 ALM | ⭐⭐ | 理解性能差异的算法根源 |
| OSQP 集成 | osqp-eigen C++ 接口 | ⭐⭐ | 入门 QP 的标准路径 |
| ProxQP 集成 | Eigen 原生 API | ⭐⭐ | Pinocchio 生态首选 |
| qpOASES & HPIPM | 活跃集 + 结构化 IPM | ⭐⭐ | 遗留代码 + MPC 结构利用 |
| Ipopt NLP | 内点法 + TNLP/ifopt | ⭐⭐ | 非线性约束优化 |
| CasADi 符号框架 | SX/MX + 自动微分 + CodeGen | ⭐⭐ | 现代 MPC 标准建模范式 |
| 三大建模模板 | IK QP / TSID QP / MPC QP | ⭐⭐ | 理论到工程的完整映射 |
| 力分配与碰撞 | 摩擦锥线性化、距离约束 | ⭐⭐⭐ | 进阶约束建模 |

---

## 累积项目：本章新增模块

**项目名称**：从零构建 7-DOF 机械臂控制栈

| 章节 | 新增模块 | 功能 |
|------|---------|------|
| M01 | URDF 加载 + FK/Jacobian | Pinocchio 基础设施 |
| M03 | IK 求解器 | 数值 IK |
| **M05（本章）** | **QP 求解器层** | **瞬态 IK QP + 约束处理** |
| M08 | 轨迹优化 | NLP 轨迹规划 |
| M10 | 时间参数化 | 速度曲线后处理 |

---

## 延伸阅读

| 资源 | 内容 | 难度 |
|------|------|------|
| Boyd & Vandenberghe, *Convex Optimization* (2004) | Ch4-5 QP/LP, Ch11 内点法 | ⭐⭐ |
| Nocedal & Wright, *Numerical Optimization* (2006) | Ch16 QP, Ch18 SQP, Ch19 IPM | ⭐⭐⭐ |
| Stellato et al., *OSQP*, Math. Prog. Comp. 2020 | ADMM 算法详解 | ⭐⭐ |
| Bambade et al., *ProxQP*, RSS 2022 | 近端增广拉格朗日 | ⭐⭐⭐ |
| Frison & Diehl, *HPIPM*, IFAC 2020 | 结构化 QP + Riccati | ⭐⭐⭐ |
| Andersson et al., *CasADi*, Math. Prog. Comp. 2019 | 符号框架全景 | ⭐⭐ |
| Ferreau et al., *qpOASES*, Math. Prog. Comp. 2014 | 在线活跃集法 | ⭐⭐⭐ |
| Drake *MathematicalProgram* 文档 | 统一 QP/NLP/SDP/MIP 接口 | ⭐⭐ |
| ifopt GitHub: `ethz-adrl/ifopt` | Eigen-based Ipopt 封装 | ⭐ |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| QP 返回 INFEASIBLE | 约束互相矛盾 | 1. 检查 $l \leq u$ 2. 逐步移除约束定位冲突 3. 检查关节是否在限位边缘 | M05 §8 |
| QP 返回 NaN | Hessian $P$ 不正定 | 1. 检查特征值 2. 添加正则化 $P + \epsilon I$ 3. 检查 Jacobian 奇异性 | M05 §8.2, M01 |
| 求解时间超出预算 | 问题规模过大或 warm-start 失效 | 1. 打印迭代次数 2. 检查 max_iter 3. 验证 warm-start 传入正确 4. 降低精度 | M05 §3-4 |
| OSQP warm-start 后结果变差 | warm-start 解在新约束下不可行 | 1. 检查约束变化幅度 2. 约束变化大时关闭 warm-start | M05 §7.7 |
| CasADi C 代码编译失败 | 头文件或库缺失 | 1. 检查 casadi/casadi_c.h 路径 2. 链接 libcasadi.so 3. 检查函数签名 | M05 §7 |

---

## 跨章综合练习 ⭐⭐⭐

**题目**：综合 M01（Pinocchio 动力学）+ M05（QP 建模）+ M03（IK 求解器），实现一个带碰撞约束的实时 IK 系统：

1. 用 Pinocchio 加载 Franka Panda URDF，计算当前构型下的 Jacobian 和碰撞信息
2. 用本章 IK QP 模板组装 QP（跟踪任务 + 关节限位 + 碰撞回避）
3. 用 ProxQP 求解
4. 在 MuJoCo 仿真中可视化末端跟踪一个圆形轨迹，路径中放置球形障碍物
5. 验证：末端成功跟踪圆形轨迹，且机械臂与障碍物距离始终 $> d_{safe}$

**评分标准**：求解时间 < 100 us，碰撞约束无违反，跟踪误差 < 5 mm。
