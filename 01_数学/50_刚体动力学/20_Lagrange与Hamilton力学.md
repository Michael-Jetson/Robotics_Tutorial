# Lagrange 力学、Hamilton 力学与关节空间动力学方程

> **所属**：刚体动力学专题 4-2 | **前置**：10_空间向量代数（6D 速度/力、Plucker 变换、空间惯性）、变分法基础（Euler-Lagrange 方程）
> **交叉引用**：→ 30_ON动力学递推算法（RNEA 计算 $M\ddot{q}+C\dot{q}+g$）、→ 50_动力学解析微分（$\partial\tau/\partial q$ 需要本章的 Christoffel 结构）、→ 70_辛结构与动量映射（Hamilton 方程的辛几何深化）

---

## 前置自测

> 📋 **前置自测**（答不出 ≥ 2 题 → 先回前置章节复习）
>
> 1. 什么是空间向量（spatial vector）？运动向量和力向量有什么对偶关系？（回顾 10_空间向量代数）
> 2. 对称正定矩阵有哪些性质？为什么 $x^T A x > 0$（$x \neq 0$）意味着 $A$ 的所有特征值为正？
> 3. 什么是变分法的 Euler-Lagrange 方程？对泛函 $J[y] = \int F(x, y, y') dx$ 的极值条件是什么？
> 4. 什么是 Christoffel 符号？它在微分几何中扮演什么角色？
> 5. Lyapunov 稳定性的基本定义是什么？如何构造 Lyapunov 函数证明系统稳定？

## 本章目标

学完本章后，你将能够：

1. **从 d'Alembert 原理出发推导 Euler-Lagrange 方程**，理解其与 Hamilton 原理的等价性
2. **手推任意 2R/3R 机械臂的完整动力学方程** $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$
3. **用三种方法构造 C(q,q') 矩阵**：Christoffel 符号法、Ṁ-2C 斜对称性约束、空间向量递归
4. **证明 Ṁ-2C 的斜对称性**，理解其对 Slotine-Li 控制器设计的意义
5. **推导 Hamilton 正则方程并理解辛结构**，为后续保辛积分和最优控制打基础
6. **在 Pinocchio/Drake 中调用 CRBA/RNEA 计算 M、C、g**，并与手推结果交叉验证

---

## 知识树

```
Lagrange与Hamilton力学
├── 1. 为什么这是动力学的经典起点（动机）
├── 2. 广义坐标与约束
│   ├── 2.1 位形空间与广义坐标
│   ├── 2.2 完整约束（holonomic）
│   └── 2.3 非完整约束（non-holonomic）
├── 3. d'Alembert原理与虚功
│   ├── 3.1 虚位移的定义
│   ├── 3.2 d'Alembert 原理的陈述与推导
│   └── 3.3 从虚功到广义力
├── 4. Euler-Lagrange 方程的完整推导
│   ├── 4.1 Hamilton 原理（最小作用量原理）
│   ├── 4.2 变分推导的每一步
│   └── 4.3 非保守力的处理
├── 5. 标准方程 M(q)q̈+C(q,q̇)q̇+g(q)=τ
│   ├── 5.1 动能 T 与质量矩阵 M(q)
│   ├── 5.2 M(q) 的性质：对称正定性证明
│   ├── 5.3 C(q,q̇) 的 Christoffel 符号构造
│   ├── 5.4 Ṁ-2C 斜对称性的完整证明
│   └── 5.5 重力项 g(q) = ∂V/∂q
├── 6. Hamilton 力学
│   ├── 6.1 Legendre 变换
│   ├── 6.2 Hamilton 正则方程
│   ├── 6.3 Poisson 括号与守恒量
│   └── 6.4 辛结构预告
├── 7. 算法实现
│   ├── 7.1 CRBA 计算 M(q)
│   ├── 7.2 RNEA 计算 C·q̇+g
│   └── 7.3 Pinocchio/Drake 代码
└── 8. 典型例题
    ├── 8.1 2R 机械臂完整推导
    └── 8.2 3R 空间机械臂
```

---

## 1. 为什么 Lagrange-Hamilton 框架是动力学的经典起点 ⭐

### 动机：一个方程统治所有机器人控制

**标准机器人运动方程**：

$$\boxed{M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau}$$

这是一切现代机器人控制、仿真、学习的**共同语言**：

| 应用场景 | 如何使用标准方程 |
|---------|----------------|
| **MPC** | 作为等式约束 $M\ddot{q} + h = \tau$ |
| **被动性控制** | $\dot{M}-2C$ 斜对称性保证 Lyapunov 下降 |
| **计算力矩控制** | 反馈线性化：$\tau = M(q)\ddot{q}_d + C\dot{q} + g$ |
| **Model-based RL** | 作为环境模型/学习目标 |
| **iLQR/DDP** | 线性化提供状态方程 $\dot{x} = f(x, u)$ |
| **系统辨识** | 估计 $M$, $C$, $g$ 中的惯性参数 |
| **仿真器** | 正动力学 $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$ |

> **本质洞察**：标准方程不仅是"描述机器人运动的方程"——它是连接物理世界和数学优化世界的桥梁。会推导这个方程，就意味着能把"机器人是一个二阶受控动力系统"这句话落实到可以用代码求解的数学结构中。

### 如果不学 Lagrange 方法会怎样

你可能会问：直接用 Newton-Euler 不行吗？确实可以——RNEA 算法就是基于 Newton-Euler 的。但：

1. **Newton-Euler 给出的是力/力矩**，不直接给出广义坐标下的方程形式。要得到 $M(q)\ddot{q} + ...$，你需要额外的"汇编"步骤
2. **控制器设计需要 M、C、g 的显式结构**——被动性、自适应控制器的 Lyapunov 证明需要 $\dot{M}-2C$ 的性质，而这个性质只在 Lagrangian 框架下自然显现
3. **Hamilton 力学提供能量守恒和辛结构**——这对长时仿真的数值积分器选择至关重要
4. **Noether 定理通过对称性直接给出守恒量**——浮基机器人的动量守恒、旋转对称系统的角动量守恒，都从 Lagrangian 的对称性直接读出

> **反事实推理**：如果你只学了 Newton-Euler 而跳过 Lagrange/Hamilton，你能跑 RNEA 算逆动力学，但面对 Slotine-Li 自适应控制器的论文时，会不知道 $\dot{M}-2C$ 的斜对称性从何而来、为什么只有 Christoffel 构造才有强版本。你也无法理解 MPC 中 symplectic integrator 为什么能避免能量漂移。

### 历史脉络

| 年代 | 人物 | 贡献 |
|------|------|------|
| 1743 | d'Alembert | 虚功原理——将动力学化为静力学 |
| 1788 | Lagrange | *Mecanique Analytique*——用广义坐标统一力学 |
| 1833 | Hamilton | Hamilton 原理（最小作用量）、正则方程 |
| 1835 | Hamilton, Jacobi | Hamilton-Jacobi 方程 |
| 1870s | Lie | 连续对称群 |
| 1918 | Noether | 对称性与守恒量的对应 |
| 1960s | Arnold | 辛几何形式化 |
| 1982 | Walker-Orin | CRBA 算法——用 RNEA 构造 M(q) |
| 1987 | Slotine-Li | 利用 $\dot{M}-2C$ 设计自适应控制器 |
| 2018 | Carpentier-Mansard | Pinocchio 的解析微分——让 MPC 实时 |

---

## 2. 广义坐标与约束 ⭐

### 2.1 位形空间与广义坐标

**位形空间** $\mathcal{Q}$ 是描述机器人所有可能构型的集合。对于 $n$ 自由度的机械臂：

$$q = (q_1, q_2, \ldots, q_n) \in \mathcal{Q}$$

每个 $q_i$ 称为一个**广义坐标**。"广义"意味着它不必是 Cartesian 坐标——可以是角度、弧长、或任何方便的参数。

**为什么用广义坐标而非 Cartesian 坐标？**

一个 $n$-DOF 串联机械臂有 $n$ 个连杆，每个连杆在空间中有 6 个自由度。如果用 Cartesian 坐标描述所有连杆，需要 $6n$ 个变量加上约束方程（关节把连杆连在一起）。用广义坐标，只需 $n$ 个变量——约束已被坐标选择自动满足。

| 机器人类型 | 位形空间 | 广义坐标 | 维度 |
|-----------|---------|---------|------|
| 平面 $n$-R 臂 | $\mathbb{T}^n$（$n$-环面） | 关节角 $q_1, \ldots, q_n$ | $n$ |
| 空间 6-DOF 臂 | $\mathbb{T}^6$ 或混合 | 关节角/位移 | 6 |
| 浮基四足 | $SE(3) \times \mathbb{T}^{12}$ | 基座位姿 + 12关节角 | 18 |
| 人形（30-DOF） | $SE(3) \times \mathbb{T}^{30}$ | 基座 + 关节 | 36 |

> **跨领域类比**：广义坐标之于刚体动力学，就像极坐标之于中心力问题——选择与问题对称性匹配的坐标，让方程变得简洁。错误的坐标选择会让简单问题变得不必要地复杂。

### 2.2 完整约束（Holonomic Constraints）

**定义**：可以写成位置级等式 $\phi(q, t) = 0$ 的约束。

**特征**：完整约束限制了系统能到达的位形——位形空间被约束从 $\mathbb{R}^N$ 缩减为低维流形。

**机器人中的例子**：
- 串联机械臂的关节约束（相邻连杆固定在铰链处）——这些约束通过广义坐标选择已被消除
- 闭链机构（如 Delta 并联机器人）的回路约束：$\phi(q) = p_{end_1}(q) - p_{end_2}(q) = 0$
- 保持末端执行器在某平面上：$z_{ee}(q) = z_0$

当使用最小坐标集（$n$ 个广义坐标描述 $n$-DOF 系统）时，所有完整约束已被坐标选择隐式满足。但闭链机构和接触约束需要用 Lagrange 乘子显式处理。

### 2.3 非完整约束（Non-holonomic Constraints）⭐⭐

**定义**：只能写成速度级等式 $A(q)\dot{q} = 0$，且**不可积**（不存在 $\phi(q)$ 使得 $\nabla\phi \cdot \dot{q} = 0$ 等价于 $A(q)\dot{q} = 0$）。

**不可积性意味着什么？** 虽然系统在速度空间被约束（不能沿某些方向瞬时运动），但通过适当的路径，它可以到达位形空间中的**任何点**。

**经典例子**：

**差速驱动轮式机器人**：两个轮子在平面上无滑滚动。位形 $q = (x, y, \theta)$（位置和朝向），约束：

$$\dot{x}\sin\theta - \dot{y}\cos\theta = 0 \quad \text{（不能侧向移动）}$$

这个约束不可积——机器人不能瞬时侧移，但通过"先转再直走"，它可以到达平面上任何点。

**对 Lagrange 方程的影响**：非完整约束不能通过减少广义坐标来消除。必须用 Lagrange 乘子处理：

$$\frac{d}{dt}\frac{\partial L}{\partial \dot{q}} - \frac{\partial L}{\partial q} = \tau + A^T(q)\lambda$$

其中 $\lambda$ 是约束力的 Lagrange 乘子。

> **反事实推理**：如果轮式机器人的无滑约束是完整的，那它的位形空间就只有 1 维（只能沿一条固定路径运动）。正因为约束是非完整的（不可积），机器人保留了在整个 $SE(2)$ 上的可达性——这是非完整系统强大的运动能力的数学根源。

---

## 3. d'Alembert 原理与虚功 ⭐⭐

### 3.1 虚位移的定义

**虚位移** $\delta q$ 是在给定时刻 $t$，与约束兼容的**假想的无穷小位移**。它不是真实发生的运动——真实运动需要时间推进，虚位移是在时间"冻结"时的假想测试。

为什么要引入这个概念？因为它让我们能把动力学问题转化为"投影"问题——把 Newton 定律投影到约束允许的方向上。

### 3.2 d'Alembert 原理的陈述与推导

**陈述**：对所有满足约束的虚位移 $\delta r_i$：

$$\sum_{i=1}^N (F_i - m_i \ddot{r}_i) \cdot \delta r_i = 0$$

**为什么这是对的？**

Newton 第二定律：$F_i + R_i = m_i \ddot{r}_i$，其中 $R_i$ 是约束力。

移项：$(F_i - m_i \ddot{r}_i) = -R_i$

关键假设：**理想约束**——约束力不做虚功：$\sum R_i \cdot \delta r_i = 0$

因此：$\sum (F_i - m_i \ddot{r}_i) \cdot \delta r_i = -\sum R_i \cdot \delta r_i = 0$

> **"不是X而是Y"**：d'Alembert 原理不是 Newton 定律的替代——它是 Newton 定律在**理想约束**假设下的等价重述。它的威力在于：约束力 $R_i$ 从方程中消失了！对于串联机器人，关节反力（约束力）不出现在最终方程中，大大简化了问题。

### 3.3 从虚功到广义力

设每个质点的位置 $r_i = r_i(q_1, \ldots, q_n, t)$，则虚位移：

$$\delta r_i = \sum_{j=1}^n \frac{\partial r_i}{\partial q_j} \delta q_j$$

代入 d'Alembert 原理：

$$\sum_i (F_i - m_i \ddot{r}_i) \cdot \sum_j \frac{\partial r_i}{\partial q_j} \delta q_j = 0$$

由于 $\delta q_j$ 是独立的（对于完整约束系统），每个系数必须为零：

$$\sum_i F_i \cdot \frac{\partial r_i}{\partial q_j} - \sum_i m_i \ddot{r}_i \cdot \frac{\partial r_i}{\partial q_j} = 0, \quad j = 1, \ldots, n$$

定义**广义力**：

$$Q_j = \sum_i F_i \cdot \frac{\partial r_i}{\partial q_j}$$

这就把 $3N$ 维的 Newton 方程化为了 $n$ 维的广义坐标方程。下一步就是把 $\sum_i m_i \ddot{r}_i \cdot \frac{\partial r_i}{\partial q_j}$ 表达为 $q, \dot{q}, \ddot{q}$ 的函数——这正是 Euler-Lagrange 方程要做的事。

---

## 4. Euler-Lagrange 方程的完整推导 ⭐

### 4.1 Hamilton 原理（最小作用量原理）

**Hamilton 原理**：在所有连接初态 $q(t_1)$ 和末态 $q(t_2)$ 的路径中，真实路径使**作用量**取驻值：

$$\delta S = \delta \int_{t_1}^{t_2} L(q, \dot{q}, t) \, dt = 0$$

其中 **Lagrangian** 定义为：

$$L(q, \dot{q}, t) = T(q, \dot{q}) - V(q)$$

$T$ 是动能，$V$ 是势能。

**为什么是 $T - V$ 而不是 $T + V$？** 直觉上：真实路径是"惯性力"（想继续运动，$T$ 大）和"保守力"（想回到低势能，$V$ 小）之间的平衡。$T - V$ 的驻值条件恰好表达了这种平衡。

### 4.2 变分推导的每一步

**Step 1**：对路径做变分。设真实路径为 $q(t)$，虚拟路径为 $q(t) + \epsilon \eta(t)$，其中 $\eta(t_1) = \eta(t_2) = 0$（端点固定）。

$$\delta S = \frac{d}{d\epsilon}\bigg|_{\epsilon=0} \int_{t_1}^{t_2} L(q + \epsilon\eta, \dot{q} + \epsilon\dot{\eta}, t) \, dt = 0$$

**Step 2**：展开到一阶：

$$\delta S = \int_{t_1}^{t_2} \left[ \frac{\partial L}{\partial q_i} \eta_i + \frac{\partial L}{\partial \dot{q}_i} \dot{\eta}_i \right] dt = 0$$

**Step 3**：对第二项分部积分：

$$\int_{t_1}^{t_2} \frac{\partial L}{\partial \dot{q}_i} \dot{\eta}_i \, dt = \left[\frac{\partial L}{\partial \dot{q}_i} \eta_i\right]_{t_1}^{t_2} - \int_{t_1}^{t_2} \frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} \eta_i \, dt$$

边界项为零（$\eta(t_1) = \eta(t_2) = 0$）。

**Step 4**：合并：

$$\delta S = \int_{t_1}^{t_2} \left[ \frac{\partial L}{\partial q_i} - \frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} \right] \eta_i \, dt = 0$$

**Step 5**：由变分法基本引理（对任意 $\eta_i$ 积分为零 $\Rightarrow$ 被积函数为零）：

$$\boxed{\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = 0, \quad i = 1, \ldots, n}$$

这就是 **Euler-Lagrange 方程**。

### 4.3 非保守力的处理

对于机器人，关节力矩 $\tau$ 是非保守力。非保守广义力不能从势能导出，需要单独添加：

$$\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = \tau_i$$

其中 $\tau_i$ 是第 $i$ 个广义坐标对应的非保守广义力（对旋转关节就是力矩 N·m，对平移关节就是力 N）。

#### 为什么 Lagrange 方法不需要知道约束力

回顾 d'Alembert 原理：理想约束力不做虚功。在 Lagrange 框架中，这些约束力完全不出现——它们被广义坐标的选择自动消除了。这是 Lagrange 方法相比 Newton-Euler 的核心优势之一：**你不需要计算关节间的内力**。

> **跨领域类比**：Lagrange 方法之于 Newton 方法，就像电路分析中的网孔法之于节点法——选择正确的"坐标"（网孔电流/广义坐标），让一部分变量（支路电流/约束力）自动被消除。

---

## 5. 标准方程 M(q)q̈+C(q,q̇)q̇+g(q)=τ 的完整推导 ⭐

### 5.1 动能 T 与质量矩阵 M(q)

#### Jacobian 叠加法推导

对串联 $n$-DOF 机械臂的第 $i$ 个连杆，其质心速度和角速度通过 Jacobian 与关节速度关联：

$$v_{c_i} = J_{v_i}(q) \dot{q}, \quad \omega_i = J_{\omega_i}(q) \dot{q}$$

第 $i$ 连杆的动能：

$$T_i = \frac{1}{2} m_i \|v_{c_i}\|^2 + \frac{1}{2} \omega_i^T {}^0I_{c_i} \omega_i$$

其中 ${}^0I_{c_i} = R_i(q) I_{c_i}^{body} R_i^T(q)$ 是惯性张量在世界系中的表达。

代入 Jacobian 关系：

$$T_i = \frac{1}{2} \dot{q}^T \left[ m_i J_{v_i}^T J_{v_i} + J_{\omega_i}^T {}^0I_{c_i} J_{\omega_i} \right] \dot{q}$$

总动能：

$$T = \sum_{i=1}^n T_i = \frac{1}{2} \dot{q}^T \underbrace{\left[ \sum_{i=1}^n \left( m_i J_{v_i}^T J_{v_i} + J_{\omega_i}^T {}^0I_{c_i} J_{\omega_i} \right) \right]}_{M(q)} \dot{q}$$

这就是**质量矩阵**（也叫惯性矩阵）的显式构造。

#### 空间向量法推导

回顾上一专题：用第 $i$ 连杆的空间速度 $v_i \in \mathbb{R}^6$ 和空间惯性 $\mathcal{I}_i \in \mathbb{R}^{6\times 6}$：

$$T = \frac{1}{2} \sum_{i=1}^n v_i^T \mathcal{I}_i v_i$$

由关节约束 $v_i = {}^iX_{\lambda(i)} v_{\lambda(i)} + S_i \dot{q}_i$，展开后得 $T = \frac{1}{2} \dot{q}^T M(q) \dot{q}$，且 $M(q)$ 可由 **CRBA 递推** $O(n^2)$ 构造。

两种方法给出相同的 $M(q)$——前者适合手动符号推导（2-3 DOF），后者适合高效数值计算（任意 DOF）。

### 5.2 M(q) 的性质：对称正定性证明 ⭐⭐

**定理**：质量矩阵 $M(q)$ 对所有 $q \in \mathcal{Q}$ 是**对称正定**的。

**证明**：

(1) **对称性**：$M(q) = M(q)^T$

从定义 $M = \sum_i (m_i J_{v_i}^T J_{v_i} + J_{\omega_i}^T I_{c_i} J_{\omega_i})$，每一项都是 $A^T B A$ 的形式（$B$ 对称），所以和也是对称的。$\square$

(2) **正定性**：对任意 $\dot{q} \neq 0$，$\dot{q}^T M(q) \dot{q} > 0$

$$\dot{q}^T M \dot{q} = 2T = \sum_i \left( m_i \|J_{v_i}\dot{q}\|^2 + (J_{\omega_i}\dot{q})^T I_{c_i} (J_{\omega_i}\dot{q}) \right)$$

每一项非负。要证明总和严格正：

- 如果 $J_{v_1}\dot{q} = 0$ 且 $J_{\omega_1}\dot{q} = 0$，意味着第一个连杆既不平移也不旋转
- 但第一个关节直接驱动连杆 1，所以 $J_{\omega_1}$ 的第一列非零（旋转关节）或 $J_{v_1}$ 的第一列非零（平移关节）
- 对串联机械臂，不存在 $\dot{q} \neq 0$ 使所有连杆都静止

更严格地：$M$ 正定等价于 $\text{rank}([J_{v_1}^T, J_{\omega_1}^T, \ldots, J_{v_n}^T, J_{\omega_n}^T]^T) = n$。对串联机械臂，这在所有构型下成立（无关节空间奇异）。$\square$

> **本质洞察**：$M(q)$ 正定的物理含义是——无论机器人处于什么构型，只要关节有非零速度，系统就有正的动能。不存在"零能量的运动"。这看似显然，但对数值算法意义重大：正定保证了 $M$ 可逆（正动力学 $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$ 总是有唯一解）、Cholesky 分解总是成功。

**M(q) 正定性失效的情况**：
- URDF 中惯性参数设为零或负值——$M$ 退化
- 运动学奇异（$J$ 失秩）——**操作空间惯性** $\Lambda = (JM^{-1}J^T)^{-1}$ 退化，但 $M$ 本身仍正定！
- 并联机构的冗余约束——需要约去冗余坐标后 $M$ 才正定

### 5.3 C(q,q̇) 的 Christoffel 符号构造 ⭐⭐

#### 从 Euler-Lagrange 到标准形式

对 $L = T - V = \frac{1}{2}\dot{q}^T M(q) \dot{q} - V(q)$：

$$\frac{\partial L}{\partial \dot{q}_i} = \sum_j m_{ij} \dot{q}_j$$

$$\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} = \sum_j m_{ij} \ddot{q}_j + \sum_j \dot{m}_{ij} \dot{q}_j = \sum_j m_{ij} \ddot{q}_j + \sum_{j,k} \frac{\partial m_{ij}}{\partial q_k} \dot{q}_k \dot{q}_j$$

$$\frac{\partial L}{\partial q_i} = \frac{1}{2} \sum_{j,k} \frac{\partial m_{jk}}{\partial q_i} \dot{q}_j \dot{q}_k - \frac{\partial V}{\partial q_i}$$

代入 Euler-Lagrange 方程 $\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = \tau_i$：

$$\sum_j m_{ij} \ddot{q}_j + \sum_{j,k} \frac{\partial m_{ij}}{\partial q_k} \dot{q}_k \dot{q}_j - \frac{1}{2}\sum_{j,k}\frac{\partial m_{jk}}{\partial q_i}\dot{q}_j\dot{q}_k + \frac{\partial V}{\partial q_i} = \tau_i$$

#### 对称化技巧——Christoffel 符号的由来

关键步骤是对 $\sum_{j,k} \frac{\partial m_{ij}}{\partial q_k} \dot{q}_j \dot{q}_k$ 做对称化。由于 $j$ 和 $k$ 是哑指标：

$$\sum_{j,k} \frac{\partial m_{ij}}{\partial q_k} \dot{q}_j \dot{q}_k = \frac{1}{2}\sum_{j,k}\left(\frac{\partial m_{ij}}{\partial q_k} + \frac{\partial m_{ik}}{\partial q_j}\right)\dot{q}_j\dot{q}_k$$

（交换 $j \leftrightarrow k$ 并取平均。）

与 $-\frac{1}{2}\sum_{j,k}\frac{\partial m_{jk}}{\partial q_i}\dot{q}_j\dot{q}_k$ 合并：

$$\sum_{j,k} \frac{1}{2}\left(\frac{\partial m_{ij}}{\partial q_k} + \frac{\partial m_{ik}}{\partial q_j} - \frac{\partial m_{jk}}{\partial q_i}\right)\dot{q}_j\dot{q}_k$$

定义**第一类 Christoffel 符号**：

$$\boxed{c_{ijk}(q) = \frac{1}{2}\left(\frac{\partial m_{ij}}{\partial q_k} + \frac{\partial m_{ik}}{\partial q_j} - \frac{\partial m_{jk}}{\partial q_i}\right)}$$

则 Coriolis/离心力项的矩阵元素为：

$$C_{ij}(q,\dot{q}) = \sum_k c_{ijk}(q) \dot{q}_k$$

#### 为什么叫 Christoffel 符号

这不是巧合——这与微分几何中的 Christoffel 联络完全相同！位形空间 $\mathcal{Q}$ 配以度量 $g_{ij} = m_{ij}$（质量矩阵作为 Riemannian 度量），Christoffel 符号定义了这个度量的 Levi-Civita 联络。运动方程 $M\ddot{q} + C\dot{q} + g = \tau$ 在无外力时就是测地线方程——自由运动沿位形空间的"直线"（测地线）行进。

> **跨领域类比**：Christoffel 符号在机器人动力学中的角色，就像在广义相对论中一样——它们描述"坐标系弯曲"导致的表观力。Coriolis 力和离心力不是"真实的力"，而是因为广义坐标系在旋转而产生的"惯性力"。这正是为什么它们可以完全用质量矩阵的偏导数（即度量张量的导数）来表达。

### 5.4 Ṁ-2C 斜对称性的完整证明 ⭐⭐

这是本章最重要的定理之一，对控制器设计至关重要。

#### 定理陈述

**强版本**：当 $C$ 由 Christoffel 符号构造时，矩阵 $N(q,\dot{q}) := \dot{M}(q) - 2C(q,\dot{q})$ 是**反对称**（skew-symmetric）的：

$$N^T = -N, \quad \text{即} \quad x^T N x = 0 \quad \forall x \in \mathbb{R}^n$$

#### 完整证明

**Step 1**：计算 $\dot{M}$ 的元素。

$$\dot{m}_{ij} = \sum_k \frac{\partial m_{ij}}{\partial q_k} \dot{q}_k$$

**Step 2**：计算 $N_{ij} = \dot{m}_{ij} - 2C_{ij}$。

$$N_{ij} = \sum_k \frac{\partial m_{ij}}{\partial q_k} \dot{q}_k - 2\sum_k c_{ijk}\dot{q}_k$$

$$= \sum_k \left[\frac{\partial m_{ij}}{\partial q_k} - \left(\frac{\partial m_{ij}}{\partial q_k} + \frac{\partial m_{ik}}{\partial q_j} - \frac{\partial m_{jk}}{\partial q_i}\right)\right]\dot{q}_k$$

$$= \sum_k \left(\frac{\partial m_{jk}}{\partial q_i} - \frac{\partial m_{ik}}{\partial q_j}\right)\dot{q}_k$$

**Step 3**：检验反对称性。

$$N_{ji} = \sum_k \left(\frac{\partial m_{ik}}{\partial q_j} - \frac{\partial m_{jk}}{\partial q_i}\right)\dot{q}_k = -N_{ij} \quad \square$$

#### 弱版本 vs 强版本的区别

| | 弱版本 | 强版本 |
|---|--------|--------|
| 陈述 | $\dot{q}^T(\dot{M} - 2C)\dot{q} = 0$ | $x^T(\dot{M} - 2C)x = 0, \forall x$ |
| 适用条件 | 任何使 $C\dot{q}$ 正确的 $C$ 矩阵 | **仅 Christoffel 构造的 $C$** |
| 物理含义 | 能量守恒（保守系统 $\dot{T} = \dot{q}^T\tau$） | 矩阵本身反对称 |
| 在控制中的用途 | 证明被动性 | **Slotine-Li 自适应控制器** |

> ⚠️ **概念误区**：认为"任何合法的 C 矩阵都使 Ṁ-2C 反对称"
>
> **新手想法**："$C$ 矩阵代表 Coriolis 力，所以 Ṁ-2C 应该总是反对称的"
>
> **实际上**：$C$ 不唯一！给定 $M$，只要 $C(q,\dot{q})\dot{q}$ 等于正确的 Coriolis/离心力项，$C$ 就合法。例如 $C' = C + S$（$S\dot{q} = 0$）也合法。但只有 Christoffel 构造保证**全向量**斜对称
>
> **为什么重要**：Slotine-Li 的 Lyapunov 证明中需要 $s^T(\dot{M}-2C)s = 0$（$s \neq \dot{q}$），这需要强版本。如果用了"非 Christoffel"的 $C$，证明失败
>
> **正确做法**：在需要斜对称性的控制器设计中，总是使用 Christoffel 构造的 $C$

### 5.5 重力项 g(q) = ∂V/∂q

势能为所有连杆的重力势能之和：

$$V(q) = \sum_{i=1}^n m_i g_0^T p_{c_i}(q)$$

其中 $g_0 = (0, 0, -9.81)^T$ 是重力加速度向量，$p_{c_i}(q)$ 是第 $i$ 连杆质心的世界坐标位置。

重力项：

$$g_i(q) = \frac{\partial V}{\partial q_i} = \sum_{j=1}^n m_j g_0^T \frac{\partial p_{c_j}}{\partial q_i}$$

**空间向量等价**：$g(q) = \text{RNEA}(q, 0, 0, g_0)$（关闭速度和加速度，只开重力）。

**重力补偿控制**：最简单的位置控制器 $\tau = g(q) + K_p(q_d - q) - K_d\dot{q}$ 在关节空间稳定——只要 $K_p, K_d > 0$。证明使用储能函数 $V_{ctrl} = \frac{1}{2}\dot{q}^T M\dot{q} + \frac{1}{2}e^T K_p e + V(q) - V(q_d)$。

---

## 6. Hamilton 力学 ⭐⭐

### 6.1 Legendre 变换

#### 从 $(q, \dot{q})$ 到 $(q, p)$

**广义动量**定义为：

$$p_i = \frac{\partial L}{\partial \dot{q}_i} = \sum_j m_{ij}(q) \dot{q}_j$$

矩阵形式：$p = M(q)\dot{q}$

**Legendre 变换**的前提是 $L$ 对 $\dot{q}$ 严格凸——等价于 $\frac{\partial^2 L}{\partial \dot{q}^2} = M(q) \succ 0$。由 $M$ 正定，条件满足。

逆关系：$\dot{q} = M^{-1}(q) p$

#### 为什么要做 Legendre 变换

1. **将二阶 ODE 变为一阶**：Lagrange 方程是 $n$ 个二阶方程；Hamilton 方程是 $2n$ 个一阶方程——对数值积分更方便
2. **辛结构**：Hamilton 方程有天然的辛几何结构，保证了相空间体积守恒
3. **最优控制**：Pontryagin 极大值原理直接用 Hamilton 形式
4. **量子化**：正则量子化从 Hamilton 力学出发

### 6.2 Hamilton 正则方程

**Hamiltonian**（总能量）：

$$H(q, p) = p^T\dot{q} - L = p^T M^{-1}p - \left(\frac{1}{2}p^T M^{-1}p - V\right) = \frac{1}{2}p^T M^{-1}(q)p + V(q)$$

**Hamilton 正则方程**：

$$\boxed{\dot{q}_i = \frac{\partial H}{\partial p_i} = [M^{-1}(q)p]_i}$$

$$\boxed{\dot{p}_i = -\frac{\partial H}{\partial q_i} + \tau_i}$$

第一个方程是 $\dot{q} = M^{-1}p$ 的分量形式。第二个方程展开：

$$\dot{p}_i = -\frac{1}{2} p^T \frac{\partial M^{-1}}{\partial q_i} p - \frac{\partial V}{\partial q_i} + \tau_i$$

利用 $\frac{\partial M^{-1}}{\partial q_i} = -M^{-1}\frac{\partial M}{\partial q_i}M^{-1}$：

$$\dot{p}_i = \frac{1}{2}\dot{q}^T \frac{\partial M}{\partial q_i}\dot{q} - g_i(q) + \tau_i$$

#### 能量守恒

保守系统（$\tau = 0$）下：

$$\frac{dH}{dt} = \sum_i \left(\frac{\partial H}{\partial q_i}\dot{q}_i + \frac{\partial H}{\partial p_i}\dot{p}_i\right) = \sum_i \left(\frac{\partial H}{\partial q_i}\frac{\partial H}{\partial p_i} - \frac{\partial H}{\partial p_i}\frac{\partial H}{\partial q_i}\right) = 0$$

即**机械能守恒** $H = T + V = \text{const}$。

### 6.3 Poisson 括号与守恒量 ⭐⭐⭐

**定义**：对相空间上任意两个光滑函数 $F, G$：

$$\{F, G\} = \sum_{i=1}^n \left(\frac{\partial F}{\partial q_i}\frac{\partial G}{\partial p_i} - \frac{\partial F}{\partial p_i}\frac{\partial G}{\partial q_i}\right)$$

**性质**：
1. 反对称：$\{F, G\} = -\{G, F\}$
2. 双线性
3. Leibniz 律：$\{F, GH\} = \{F,G\}H + G\{F,H\}$
4. **Jacobi 恒等式**：$\{\{F,G\},H\} + \{\{G,H\},F\} + \{\{H,F\},G\} = 0$
5. 基本关系：$\{q_i, q_j\} = 0$, $\{p_i, p_j\} = 0$, $\{q_i, p_j\} = \delta_{ij}$

**运动方程的 Poisson 括号形式**：

$$\dot{F} = \{F, H\} + \frac{\partial F}{\partial t}$$

**守恒定理**：$F$ 是守恒量 $\Leftrightarrow$ $\{F, H\} = 0$（且 $F$ 不显含 $t$）。

**Noether 定理的 Hamilton 版本**：如果 Hamiltonian 对某变换不变（对称性），则对应的广义动量守恒。例如：
- $H$ 不依赖 $q_k$ $\Rightarrow$ $p_k$ 守恒（$\{p_k, H\} = -\frac{\partial H}{\partial q_k} = 0$）
- $H$ 旋转不变 $\Rightarrow$ 角动量守恒

### 6.4 辛结构预告 ⭐⭐⭐⭐

相空间 $T^*\mathcal{Q}$ 上存在**辛形式**：

$$\omega = \sum_{i=1}^n dq^i \wedge dp_i$$

Hamilton 流 $\phi_t$（时间演化映射）**保持辛形式**：$\phi_t^* \omega = \omega$。

**推论（Liouville 定理）**：相空间体积 $\omega^n/n!$ 被保持——"相空间中的墨水滴不会被压缩或膨胀"。

**对数值积分的意义**：普通的 Runge-Kutta 方法不保辛——长时间仿真会导致能量漂移。**辛积分器**（Stormer-Verlet, Leapfrog, 隐式中点法）保持辛结构，保证能量在有界范围内振荡而不单调增长。

$$\text{MPC 跑 1000 步不漂移能量} \Leftarrow \text{用辛积分器} \Leftarrow \text{Hamilton 方程有辛结构}$$

---

## 7. 算法实现：Pinocchio/Drake 中的 M, C, g ⭐⭐

### 7.1 CRBA 计算 M(q)

**Composite Rigid Body Algorithm** (Walker-Orin 1982) 的思想：

$M(q)$ 的第 $j$ 列 $M_{:,j}$ 等于"关节 $j$ 以单位加速度运动、其他关节静止、无重力"时 RNEA 返回的关节力矩。

复杂度：$O(n^2)$（需要对每个关节做一次回溯，共 $n$ 列）。

实际上 CRBA 更聪明——用**复合刚体惯性**递推，只需一次前向扫描 + 一次后向复合惯性累加 + $n$ 次列计算：

```
Pass 1 (前向): 计算关节运动学
Pass 2 (后向): I_c[i] = I[i] + sum(X_child^T I_c[child] X_child) 
Pass 3 (列填充): 
  for j = n to 1:
    M[j,j] = S[j]^T I_c[j] S[j]
    f = I_c[j] S[j]
    i = parent[j]
    while i > 0:
      f = X[i]^T f
      M[i,j] = S[i]^T f
      M[j,i] = M[i,j]  (对称性)
      i = parent[i]
```

### 7.2 RNEA 计算 C·q̇+g

利用 RNEA 的线性性：

$$\text{RNEA}(q, \dot{q}, \ddot{q}, g_0) = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$$

所以：
- **C(q,q̇)q̇ + g(q)**：`RNEA(q, q̇, 0, g₀)` —— 一次 $O(n)$
- **C(q,q̇)q̇**：`RNEA(q, q̇, 0, 0)` —— 关闭重力
- **g(q)**：`RNEA(q, 0, 0, g₀)` —— 关闭速度

### 7.3 Pinocchio/Drake 完整代码示例

```python
"""
完整验证：M*a + C*v + g == RNEA(q, v, a)
"""
import pinocchio as pin
import numpy as np

# === 加载模型 ===
# 使用 Pinocchio 的示例模型
from pinocchio.robot_wrapper import RobotWrapper
robot = RobotWrapper.BuildFromURDF(
    "path/to/panda.urdf",
    package_dirs=["path/to/meshes"]
)
model, data = robot.model, robot.data

# === 随机状态 ===
np.random.seed(42)
q = pin.randomConfiguration(model)
v = np.random.randn(model.nv) * 0.5  # 关节速度
a = np.random.randn(model.nv) * 0.1  # 关节加速度

# === 方法1：直接 RNEA ===
tau_rnea = pin.rnea(model, data, q, v, a)

# === 方法2：M*a + (C*v + g) ===
# 计算 M(q)
M = pin.crba(model, data, q)
# CRBA 只填上三角，需要补全
M_full = np.triu(M) + np.triu(M, 1).T

# 计算非线性项 nle = C*v + g
nle = pin.nonLinearEffects(model, data, q, v)

tau_manual = M_full @ a + nle

# === 验证 ===
error = np.max(np.abs(tau_rnea - tau_manual))
print(f"Max error |RNEA - (M*a + nle)|: {error:.2e}")
assert error < 1e-10, "Verification failed!"

# === 分离 C*v 和 g ===
g_vec = pin.computeGeneralizedGravity(model, data, q)
Cv = nle - g_vec  # C(q,v)*v = nle - g

# 验证 C*v = RNEA(q, v, 0, g=0)
tau_coriolis = pin.rnea(model, data, q, v, np.zeros(model.nv))
# 注意：rnea 默认包含重力，需要模型的重力设为0
# 替代方法：Cv = nle - g
print(f"Max error |Cv|: {np.max(np.abs(Cv - (tau_coriolis - g_vec))):.2e}")

# === 验证 Ṁ-2C 的斜对称性 ===
# 计算显式 C 矩阵
C_mat = pin.computeCoriolisMatrix(model, data, q, v)

# 计算 Ṁ (数值近似)
dt = 1e-8
q_plus = pin.integrate(model, q, v * dt)
M_plus = pin.crba(model, data, q_plus).copy()
M_plus = np.triu(M_plus) + np.triu(M_plus, 1).T
M_current = M_full.copy()
Mdot = (M_plus - M_current) / dt

# 检查 N = Mdot - 2C 的反对称性
N = Mdot - 2 * C_mat
skew_error = np.max(np.abs(N + N.T))
print(f"Skew-symmetry check |N + N^T|: {skew_error:.2e}")
# 应该接近零（受数值微分精度限制）

# === 验证 x^T N x = 0 对任意 x ===
for trial in range(10):
    x = np.random.randn(model.nv)
    quadratic_form = x @ N @ x
    print(f"  x^T N x = {quadratic_form:.2e}")
# 所有值应接近零
```

```python
"""
Drake 等价代码
"""
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
import numpy as np

# 创建植物和解析器
plant = MultibodyPlant(time_step=0.0)
parser = Parser(plant)
parser.AddModelFromFile("path/to/panda.urdf")
plant.Finalize()

context = plant.CreateDefaultContext()

# 设置状态
q = np.random.randn(plant.num_positions())
v = np.random.randn(plant.num_velocities())
plant.SetPositions(context, q)
plant.SetVelocities(context, v)

# 计算 M(q)
M = plant.CalcMassMatrix(context)

# 计算 bias term = C*v + g
Cv_plus_g = plant.CalcBiasTerm(context)

# 注意 Drake 的 CalcGravityGeneralizedForces 返回 -g(q)
neg_g = plant.CalcGravityGeneralizedForces(context)
g_drake = -neg_g  # Drake 约定：返回的是重力作为广义力（方向相反）

print(f"M shape: {M.shape}")
print(f"Bias term: {Cv_plus_g[:3]}")
```

---

## 8. 典型例题 ⭐⭐

### 8.1 2R 平面机械臂完整推导

**模型**：两个旋转关节在竖直平面内，连杆长 $\ell_1, \ell_2$，质量 $m_1, m_2$ 集中在连杆末端（简化模型）。关节角 $q_1$（从水平逆时针为正）、$q_2$（相对于连杆 1）。重力 $g$ 向下。

**Step 1：质心位置**

$$x_1 = \ell_1 \cos q_1, \quad y_1 = \ell_1 \sin q_1$$
$$x_2 = \ell_1 \cos q_1 + \ell_2 \cos(q_1 + q_2), \quad y_2 = \ell_1 \sin q_1 + \ell_2 \sin(q_1 + q_2)$$

**Step 2：速度（对 t 求导）**

$$\dot{x}_1 = -\ell_1 \sin q_1 \cdot \dot{q}_1$$
$$\dot{y}_1 = \ell_1 \cos q_1 \cdot \dot{q}_1$$

$$\dot{x}_2 = -\ell_1 \sin q_1 \cdot \dot{q}_1 - \ell_2 \sin(q_1+q_2)(\dot{q}_1+\dot{q}_2)$$
$$\dot{y}_2 = \ell_1 \cos q_1 \cdot \dot{q}_1 + \ell_2 \cos(q_1+q_2)(\dot{q}_1+\dot{q}_2)$$

**Step 3：动能**

$$T_1 = \frac{1}{2}m_1(\dot{x}_1^2 + \dot{y}_1^2) = \frac{1}{2}m_1 \ell_1^2 \dot{q}_1^2$$

$$T_2 = \frac{1}{2}m_2(\dot{x}_2^2 + \dot{y}_2^2)$$

展开 $\dot{x}_2^2 + \dot{y}_2^2$：

$$= \ell_1^2\dot{q}_1^2 + \ell_2^2(\dot{q}_1+\dot{q}_2)^2 + 2\ell_1\ell_2\dot{q}_1(\dot{q}_1+\dot{q}_2)\cos q_2$$

（利用 $\sin A \sin B + \cos A \cos B = \cos(A-B)$）

所以：
$$T = \frac{1}{2}(m_1+m_2)\ell_1^2\dot{q}_1^2 + \frac{1}{2}m_2\ell_2^2(\dot{q}_1+\dot{q}_2)^2 + m_2\ell_1\ell_2\cos q_2 \cdot \dot{q}_1(\dot{q}_1+\dot{q}_2)$$

写成 $T = \frac{1}{2}\dot{q}^T M(q)\dot{q}$：

$$\boxed{M(q) = \begin{pmatrix} (m_1+m_2)\ell_1^2 + m_2\ell_2^2 + 2m_2\ell_1\ell_2\cos q_2 & m_2\ell_2^2 + m_2\ell_1\ell_2\cos q_2 \\ m_2\ell_2^2 + m_2\ell_1\ell_2\cos q_2 & m_2\ell_2^2 \end{pmatrix}}$$

简记：$m_{11} = (m_1+m_2)\ell_1^2 + m_2\ell_2^2 + 2m_2\ell_1\ell_2 c_2$，$m_{12} = m_{21} = m_2\ell_2^2 + m_2\ell_1\ell_2 c_2$，$m_{22} = m_2\ell_2^2$。

其中 $c_2 = \cos q_2$, $s_2 = \sin q_2$。

**Step 4：C(q,q̇) 的 Christoffel 构造**

需要偏导数：
- $\frac{\partial m_{11}}{\partial q_1} = 0$, $\frac{\partial m_{11}}{\partial q_2} = -2m_2\ell_1\ell_2 s_2$
- $\frac{\partial m_{12}}{\partial q_1} = 0$, $\frac{\partial m_{12}}{\partial q_2} = -m_2\ell_1\ell_2 s_2$
- $\frac{\partial m_{22}}{\partial q_1} = 0$, $\frac{\partial m_{22}}{\partial q_2} = 0$

定义 $h = -m_2\ell_1\ell_2 \sin q_2$：

Christoffel 符号：
- $c_{112} = \frac{1}{2}(\frac{\partial m_{11}}{\partial q_2} + \frac{\partial m_{12}}{\partial q_1} - \frac{\partial m_{12}}{\partial q_1}) = \frac{1}{2}(-2m_2\ell_1\ell_2 s_2) = h$
- $c_{121} = \frac{1}{2}(\frac{\partial m_{11}}{\partial q_2} + \frac{\partial m_{11}}{\partial q_2} - \frac{\partial m_{12}}{\partial q_1}) = h$ （对称）
- $c_{122} = \frac{1}{2}(\frac{\partial m_{12}}{\partial q_2} + \frac{\partial m_{12}}{\partial q_2} - \frac{\partial m_{22}}{\partial q_1}) = -m_2\ell_1\ell_2 s_2 = h$
- $c_{211} = \frac{1}{2}(\frac{\partial m_{21}}{\partial q_1} + \frac{\partial m_{21}}{\partial q_1} - \frac{\partial m_{11}}{\partial q_2}) = \frac{1}{2}(0 + 0 + 2m_2\ell_1\ell_2 s_2) = -h$
- $c_{212} = c_{221} = 0$, $c_{222} = 0$

C 矩阵：

$$\boxed{C(q,\dot{q}) = \begin{pmatrix} h\dot{q}_2 & h(\dot{q}_1+\dot{q}_2) \\ -h\dot{q}_1 & 0 \end{pmatrix}}$$

**Step 5：重力项**

$$V = m_1 g y_1 + m_2 g y_2 = m_1 g \ell_1 \sin q_1 + m_2 g [\ell_1 \sin q_1 + \ell_2 \sin(q_1+q_2)]$$

$$\boxed{g(q) = \begin{pmatrix} (m_1+m_2)g\ell_1\cos q_1 + m_2 g\ell_2\cos(q_1+q_2) \\ m_2 g\ell_2\cos(q_1+q_2) \end{pmatrix}}$$

**Step 6：验证 Ṁ-2C 反对称**

$$\dot{M} = \begin{pmatrix} -2m_2\ell_1\ell_2 s_2 \dot{q}_2 & -m_2\ell_1\ell_2 s_2 \dot{q}_2 \\ -m_2\ell_1\ell_2 s_2 \dot{q}_2 & 0 \end{pmatrix} = \begin{pmatrix} 2h\dot{q}_2 & h\dot{q}_2 \\ h\dot{q}_2 & 0 \end{pmatrix}$$

$$\dot{M} - 2C = \begin{pmatrix} 2h\dot{q}_2 - 2h\dot{q}_2 & h\dot{q}_2 - 2h(\dot{q}_1+\dot{q}_2) \\ h\dot{q}_2 + 2h\dot{q}_1 & 0 - 0 \end{pmatrix} = \begin{pmatrix} 0 & -h(2\dot{q}_1+\dot{q}_2) \\ h(2\dot{q}_1+\dot{q}_2) & 0 \end{pmatrix}$$

确实反对称！$N_{12} = -N_{21}$, $N_{11} = N_{22} = 0$。$\square$

### 8.2 SymPy 验证脚本

```python
"""
用 SymPy 自动推导并验证 2R 臂的动力学
"""
from sympy import *

# 符号定义
q1, q2 = symbols('q1 q2')
dq1, dq2 = symbols('dq1 dq2')
ddq1, ddq2 = symbols('ddq1 ddq2')
l1, l2, m1, m2, g_val = symbols('l1 l2 m1 m2 g', positive=True)
t = symbols('t')

# 质心位置
x1 = l1 * cos(q1)
y1 = l1 * sin(q1)
x2 = l1*cos(q1) + l2*cos(q1+q2)
y2 = l1*sin(q1) + l2*sin(q1+q2)

# 速度
dx1 = diff(x1, q1)*dq1
dy1 = diff(y1, q1)*dq1
dx2 = diff(x2, q1)*dq1 + diff(x2, q2)*dq2
dy2 = diff(y2, q1)*dq1 + diff(y2, q2)*dq2

# 动能
T = Rational(1,2)*m1*(dx1**2 + dy1**2) + Rational(1,2)*m2*(dx2**2 + dy2**2)
T = expand(trigsimp(T))

# 势能
V = m1*g_val*y1 + m2*g_val*y2

# Lagrangian
L = T - V

# Euler-Lagrange 方程
# d/dt(dL/ddq_i) - dL/dq_i = tau_i
# 需要把 dq1, dq2 当作 q1(t), q2(t) 的导数处理
# 简化：直接提取 M, C, g

# 提取 M 矩阵
q_vec = Matrix([q1, q2])
dq_vec = Matrix([dq1, dq2])

# M_ij = d^2T / (d dq_i d dq_j)
M11 = diff(T, dq1, dq1)
M12 = diff(T, dq1, dq2)
M22 = diff(T, dq2, dq2)
M_sym = Matrix([[M11, M12], [M12, M22]])
M_sym = trigsimp(M_sym)
print("M(q) =")
pprint(M_sym)

# 计算 C 用 Christoffel 符号
def christoffel(M, q, i, j, k):
    """计算 c_{ijk}"""
    return Rational(1,2)*(diff(M[i,j], q[k]) + diff(M[i,k], q[j]) - diff(M[j,k], q[i]))

C = zeros(2, 2)
q_list = [q1, q2]
for i in range(2):
    for j in range(2):
        for k in range(2):
            C[i,j] += christoffel(M_sym, q_list, i, j, k) * dq_vec[k]
C = trigsimp(C)
print("\nC(q,dq) =")
pprint(C)

# 重力项
g_vec_sym = Matrix([diff(V, q1), diff(V, q2)])
g_vec_sym = trigsimp(g_vec_sym)
print("\ng(q) =")
pprint(g_vec_sym)

# 验证 Mdot - 2C 反对称
Mdot = zeros(2, 2)
for i in range(2):
    for j in range(2):
        for k in range(2):
            Mdot[i,j] += diff(M_sym[i,j], q_list[k]) * dq_vec[k]

N = trigsimp(Mdot - 2*C)
print("\nN = Mdot - 2C =")
pprint(N)
print("\nN + N^T =")
pprint(trigsimp(N + N.T))  # 应为零矩阵
```

---

## 9. 标准方程的各项深入解析 ⭐⭐

### 9.1 M(q)q̈ 项——配置依赖的惯性力

$M(q)\ddot{q}$ 表示"在给定构型 $q$ 下，产生关节加速度 $\ddot{q}$ 所需的等效力矩"。

**关键特征**：
- $M$ 随 $q$ 变化——同样的加速度在不同构型下需要不同力矩
- 对角元素 $m_{ii}$ 是关节 $i$ 的等效惯量——包括直接连接的连杆惯量加上所有下游连杆的耦合惯量
- 非对角元素 $m_{ij}$（$i \neq j$）是惯性耦合——关节 $j$ 加速会在关节 $i$ 上产生"虚拟力矩"

**惯性耦合的物理含义**：

考虑 2R 臂：$m_{12} = m_2\ell_2^2 + m_2\ell_1\ell_2\cos q_2$。当 $q_2 = 0$（两连杆一条直线）时 $m_{12}$ 最大——因为此时连杆 2 的运动对关节 1 的影响最大（力臂最长）。当 $q_2 = 90°$ 时 $m_{12} = m_2\ell_2^2$（只剩连杆 2 自身惯量的耦合）。

### 9.2 C(q,q̇)q̇ 项——速度依赖的"表观力"

$C(q,\dot{q})\dot{q}$ 项包含两类表观力：

**Coriolis 力**（$\propto \dot{q}_i \dot{q}_j$, $i \neq j$）：
- 产生原因：两个不同关节同时运动时，一个关节的运动"拖拽"另一个
- 例子：你在旋转木马上走路时感受到的侧向力

**离心力**（$\propto \dot{q}_i^2$）：
- 产生原因：单个关节旋转产生的径向"离心"效应
- 例子：快速旋转手臂时感受到的向外拉力

> **反事实推理**：如果 $C(q,\dot{q})\dot{q} = 0$（没有 Coriolis/离心力），会怎样？机器人的运动方程退化为 $M(q)\ddot{q} + g(q) = \tau$——加速度只与位置和力矩有关，与速度无关。这在低速情况下近似成立（C 项与 $\dot{q}^2$ 成正比），但高速运动时完全不能忽略。工业机器人以 6 rad/s 关节速度运动时，C 项可达 M 项的 30-50%。

### 9.3 g(q) 项——保守力

$g(q) = \partial V / \partial q$ 是势能对广义坐标的梯度。对于重力场中的机器人：

- $g(q)$ 只依赖位置 $q$，不依赖速度
- 在水平面内运动的机器人（如 SCARA），$g(q) = 0$
- 重力补偿 $\tau_{gc} = g(q)$ 是最简单的控制策略——消除重力后系统变成"自由"的

### 9.4 三项之间的数量级关系

在典型工作条件下：

| 条件 | M项 | C项 | g项 | 主导项 |
|------|-----|-----|-----|--------|
| 低速运动 | 小 | 极小($\propto \dot{q}^2$) | 可观 | g |
| 高速运动 | 大 | 可观 | 可观 | M, C |
| 静态保持 | 0 | 0 | 可观 | g |
| 微重力环境 | 大 | 可观 | ≈0 | M, C |
| 快速变向 | 大 | 可观 | 可观 | 三者均重要 |

**工程启示**：低速精密操作时，重力补偿是最重要的；高速运动时，必须完整考虑所有三项。

---

## 10. 被动性理论与控制器设计 ⭐⭐

### 10.1 什么是被动性

**被动性**（passivity）是系统论中的一个基本概念，它说的是："系统不会自发产生能量"。

**形式定义**：一个系统（输入 $u$，输出 $y$，状态 $x$）是被动的，如果存在一个非负的**储能函数** $V(x) \geq 0$ 使得：

$$\dot{V}(x) \leq u^T y$$

即储能函数的增长率不超过外部注入的功率。

### 10.2 机器人系统的被动性

对于机器人 $M\ddot{q} + C\dot{q} + g = \tau$，选择储能函数为机械能：

$$V(q, \dot{q}) = \frac{1}{2}\dot{q}^T M(q)\dot{q} + V_{grav}(q)$$

沿系统轨迹的时间导数：

$$\dot{V} = \dot{q}^T M\ddot{q} + \frac{1}{2}\dot{q}^T\dot{M}\dot{q} + \dot{q}^T g(q)$$

代入运动方程 $M\ddot{q} = \tau - C\dot{q} - g$：

$$\dot{V} = \dot{q}^T(\tau - C\dot{q} - g) + \frac{1}{2}\dot{q}^T\dot{M}\dot{q} + \dot{q}^T g$$

$$= \dot{q}^T\tau - \dot{q}^T C\dot{q} + \frac{1}{2}\dot{q}^T\dot{M}\dot{q}$$

$$= \dot{q}^T\tau + \frac{1}{2}\dot{q}^T(\dot{M} - 2C)\dot{q}$$

由弱斜对称性 $\dot{q}^T(\dot{M} - 2C)\dot{q} = 0$：

$$\boxed{\dot{V} = \dot{q}^T\tau}$$

即 $\dot{V} =$ 输入功率。系统从端口 $(\tau, \dot{q})$ 看是**被动的**。

### 10.3 计算力矩控制（Computed Torque Control）

**思想**：用精确的动力学模型做前馈，把非线性系统变成线性系统。

控制律：
$$\tau = M(q)(\ddot{q}_d + K_d\dot{e} + K_p e) + C(q,\dot{q})\dot{q} + g(q)$$

其中 $e = q_d - q$, $\dot{e} = \dot{q}_d - \dot{q}$。

代入运动方程 $M\ddot{q} + C\dot{q} + g = \tau$：

$$M\ddot{q} + C\dot{q} + g = M(\ddot{q}_d + K_d\dot{e} + K_p e) + C\dot{q} + g$$

$$M(\ddot{q} - \ddot{q}_d) = M(K_d\dot{e} + K_p e)$$

$$\ddot{e} + K_d\dot{e} + K_p e = 0$$

这是一个**线性二阶误差动力学**！选择 $K_p, K_d > 0$ 使特征方程稳定即可。

> **反事实推理**：如果模型不精确（$\hat{M} \neq M$, $\hat{C} \neq C$, $\hat{g} \neq g$），计算力矩控制会怎样？误差动力学变成 $\ddot{e} + K_d\dot{e} + K_p e = M^{-1}(\hat{M}-M)\ddot{q}_d + \ldots$，即模型误差作为扰动输入。对 20% 的参数误差，跟踪误差可能增大 5-10 倍。这就是 Slotine-Li 自适应控制的动机——在线估计参数。

### 10.4 PD+ 重力补偿控制

不需要完整模型的简单方案：

$$\tau = g(q) + K_p(q_d - q) - K_d\dot{q}$$

**稳定性证明**：Lyapunov 函数 $V = \frac{1}{2}\dot{q}^T M\dot{q} + \frac{1}{2}(q-q_d)^T K_p(q-q_d)$。

$$\dot{V} = \dot{q}^T M\ddot{q} + \frac{1}{2}\dot{q}^T\dot{M}\dot{q} + \dot{q}^T K_p(q-q_d) \cdot (-1)$$

代入 $M\ddot{q} = \tau - C\dot{q} - g = K_p(q_d-q) - K_d\dot{q} - C\dot{q}$：

$$\dot{V} = \dot{q}^T[K_p(q_d-q) - K_d\dot{q} - C\dot{q}] + \frac{1}{2}\dot{q}^T\dot{M}\dot{q} - \dot{q}^T K_p(q-q_d)$$

$$= -\dot{q}^T K_d\dot{q} - \dot{q}^T C\dot{q} + \frac{1}{2}\dot{q}^T\dot{M}\dot{q}$$

$$= -\dot{q}^T K_d\dot{q} + \frac{1}{2}\dot{q}^T(\dot{M}-2C)\dot{q} = -\dot{q}^T K_d\dot{q} \leq 0$$

$\dot{V}$ 半负定。由 LaSalle 不变性原理，$\dot{V} = 0$ 要求 $\dot{q} = 0$，代回方程得 $K_p(q_d-q) = g(q) - g(q)$... 等等，这里有问题——$g(q) \neq g(q_d)$ 时平衡点不在 $q_d$！

**修正**：实际上 PD+ 控制的稳定性依赖于 $K_p$ 足够大使得 $V$ 有唯一最小值。严格证明需要 Lyapunov 函数包含 $V_{grav}(q) - V_{grav}(q_d) - g(q_d)^T(q-q_d)$（势能的修正）。这在 Spong §8 中有详细讨论。

### 10.5 Slotine-Li 自适应控制器 ⭐⭐⭐

**问题**：当 $M$, $C$, $g$ 中的惯性参数（质量、惯性矩、质心位置）不精确时，如何设计自适应控制器？

**关键性质**：标准方程对惯性参数**线性**。即存在**回归矩阵** $Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r) \in \mathbb{R}^{n \times p}$ 和**参数向量** $\theta \in \mathbb{R}^p$ 使得：

$$M(q)\ddot{q}_r + C(q,\dot{q})\dot{q}_r + g(q) = Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r)\theta$$

这里 $\dot{q}_r, \ddot{q}_r$ 是"参考速度/加速度"（不一定等于 $\dot{q}_d, \ddot{q}_d$）。

**Slotine-Li 控制律**：

定义：
- 误差：$\tilde{q} = q - q_d$
- 参考速度：$\dot{q}_r = \dot{q}_d - \Lambda\tilde{q}$（$\Lambda > 0$）
- 滑模变量：$s = \dot{q} - \dot{q}_r = \dot{\tilde{q}} + \Lambda\tilde{q}$

控制律：
$$\tau = Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r)\hat{\theta} - K_s s$$

自适应律：
$$\dot{\hat{\theta}} = -\Gamma Y^T s$$

**Lyapunov 证明**：取 $V = \frac{1}{2}s^T M s + \frac{1}{2}\tilde{\theta}^T\Gamma^{-1}\tilde{\theta}$（$\tilde{\theta} = \theta - \hat{\theta}$）。

$$\dot{V} = s^T M\dot{s} + \frac{1}{2}s^T\dot{M}s + \tilde{\theta}^T\Gamma^{-1}\dot{\tilde{\theta}}$$

利用运动方程 $M\dot{s} = \tau - C\dot{q} - g - M\ddot{q}_r - C\dot{q}_r + C\dot{q}_r$（加减技巧）：

$$M\dot{s} = Y\hat{\theta} - K_s s - Y\theta + (\dot{M}-2C)... $$

经过仔细计算（此处省略中间步骤，详见 Slotine-Li 1987）：

$$\dot{V} = -s^T K_s s + s^T Y\tilde{\theta} + \frac{1}{2}s^T(\dot{M}-2C)s + \tilde{\theta}^T\Gamma^{-1}\dot{\tilde{\theta}}$$

由**强斜对称性** $s^T(\dot{M}-2C)s = 0$（注意这里 $s \neq \dot{q}$，所以需要强版本！）：

$$\dot{V} = -s^T K_s s + s^T Y\tilde{\theta} - \tilde{\theta}^T\Gamma^{-1}\Gamma Y^T s = -s^T K_s s \leq 0$$

由 Barbalat 引理：$s \to 0$，即 $\dot{\tilde{q}} + \Lambda\tilde{q} \to 0$，进而 $\tilde{q} \to 0$。$\square$

> **为什么强版本至关重要？** 在证明中，$s = \dot{q} - \dot{q}_r \neq \dot{q}$。弱版本只保证 $\dot{q}^T(\dot{M}-2C)\dot{q} = 0$，不保证 $s^T(\dot{M}-2C)s = 0$。如果用了非 Christoffel 构造的 $C$，这一步就失败了——整个自适应控制器的稳定性证明就垮了。这是 Christoffel 构造在控制理论中不可替代的原因。

---

## 11. 标准方程的广义坐标 vs 空间向量双视角 ⭐⭐

上一章（空间向量代数）给出了几何工具，本章给出了分析力学工具。两种视角在标准方程中如何统一？

### 11.1 对照表

| 对象 | 广义坐标形式 | 空间向量形式 |
|------|------------|------------|
| 动能 $T$ | $\frac{1}{2}\dot{q}^T M(q)\dot{q}$ | $\frac{1}{2}\sum_i v_i^T \mathcal{I}_i v_i$ |
| 质量矩阵 $M(q)$ | $\sum_i (J_{v_i}^T m_i J_{v_i} + J_{\omega_i}^T I_{c_i} J_{\omega_i})$ | CRBA 递推 $O(n^2)$ |
| Coriolis 项 $C\dot{q}$ | $\sum_k c_{ijk}\dot{q}_k\dot{q}_j$（Christoffel, $O(n^3)$） | RNEA$(q,\dot{q},0,0)$，$O(n)$ |
| 重力项 $g(q)$ | $\partial V/\partial q$ | RNEA$(q,0,0,g_0)$，$O(n)$ |
| 完整方程 | $M\ddot{q} + C\dot{q} + g = \tau$ | $\tau = \text{RNEA}(q,\dot{q},\ddot{q},g_0)$，$O(n)$ |
| 正动力学 | $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$，$O(n^3)$ | ABA$(q,\dot{q},\tau)$，$O(n)$ |

### 11.2 互译口诀

> *"广义坐标看全局一个大向量 $q$；空间向量看每个连杆自己的 6D 速度 $v_i$，然后用关节把它们接起来。"*

前者便于符号推导（SymPy）和控制律设计（Lyapunov 分析需要显式 $M$, $C$）。

后者便于 $O(n)$ 数值算法和软件实现（RNEA/ABA/CRBA）。

**优秀的机器人学家能在两者间自由切换**——读控制论文用广义坐标视角，读算法论文用空间向量视角，写代码时调用 Pinocchio 的 RNEA/CRBA。

---

## 12. 约束系统的 Lagrange 乘子法 ⭐⭐⭐

### 12.1 带约束的运动方程

当系统有额外约束（闭链机构、接触、任务约束）时：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + J_c^T(q)\lambda$$

其中 $J_c(q) = \partial\phi/\partial q$ 是约束 Jacobian，$\lambda$ 是 Lagrange 乘子（约束力）。

加速度级约束：

$$J_c(q)\ddot{q} + \dot{J}_c(q,\dot{q})\dot{q} = 0$$

联立得 KKT 系统：

$$\begin{pmatrix} M & -J_c^T \\ J_c & 0 \end{pmatrix}\begin{pmatrix} \ddot{q} \\ \lambda \end{pmatrix} = \begin{pmatrix} \tau - C\dot{q} - g \\ -\dot{J}_c\dot{q} \end{pmatrix}$$

### 12.2 Baumgarte 稳定化

纯加速度级约束在数值积分中会发生**约束漂移**——位置级约束 $\phi(q) = 0$ 被逐步违反。

**Baumgarte 方法**（仅适用于完整约束）：用 PD 反馈修正约束违反：

$$J_c\ddot{q} + \dot{J}_c\dot{q} + 2\alpha J_c\dot{q} + \beta^2\phi(q) = 0$$

参数 $\alpha, \beta > 0$ 决定修正速度。典型选择 $\alpha = \beta = 10/\Delta t$。

> ⚠️ **概念误区**：认为 Baumgarte 适用于所有约束
>
> **实际上**：Baumgarte 公式含 $\phi(q)$——位置级约束函数。非完整约束（$A(q)\dot{q} = 0$）不可积、不存在 $\phi(q)$，所以 Baumgarte 不适用。对非完整约束需要用纯速度级反馈或投影方法

### 12.3 在机器人控制中的应用

| 场景 | 约束类型 | 处理方式 |
|------|---------|---------|
| 闭链机构 | 完整约束 $\phi(q) = 0$ | KKT + Baumgarte |
| 足式机器人地面接触 | 单边完整约束 | LCP/NCP 或凸接触模型 |
| 固定末端位姿 | 任务空间完整约束 | 操作空间控制或 QP |
| 滚动约束 | 非完整 $A\dot{q} = 0$ | 约束力 + 速度级反馈 |

Pinocchio 的 `forwardDynamics(model, data, q, v, tau, J, gamma)` 内部就是求解上述 KKT 系统。

---

## 13. 标准方程在现代机器人系统中的应用 ⭐⭐

### 13.1 MPC 中的动力学约束

模型预测控制将轨迹优化写成有限时域 NLP：

$$\min_{\{x_k, u_k\}} \sum_{k=0}^{N} \ell(x_k, u_k) + \ell_f(x_N)$$

$$\text{s.t.} \quad x_{k+1} = f(x_k, u_k) \quad \text{（动力学约束）}$$

其中 $f$ 就是标准方程的离散化：$x = (q, \dot{q})$, $u = \tau$。

**Pinocchio + CasADi + Crocoddyl** 的工具链：
1. Pinocchio 提供 $M$, $C$, $g$ 的计算和解析导数
2. CasADi 做自动微分和 NLP 求解
3. Crocoddyl 封装 DDP/FDDP 求解器

### 13.2 强化学习中的环境模型

Model-based RL 用标准方程作为"已知的物理先验"：

$$\dot{q}_{k+1} = \dot{q}_k + \Delta t \cdot M^{-1}(q_k)[\tau_k - C(q_k,\dot{q}_k)\dot{q}_k - g(q_k)]$$

**Hamiltonian Neural Networks**（Greydanus et al., NeurIPS 2019）更进一步——不学 $\dot{x} = f(x, u)$，而是学 Hamiltonian $H(q, p)$，然后用 Hamilton 方程生成动力学。这保证了能量守恒，提高了长时预测精度。

### 13.3 系统辨识

标准方程对惯性参数线性：$\tau = Y(q, \dot{q}, \ddot{q})\theta$。

给定一组轨迹数据 $(q(t), \dot{q}(t), \ddot{q}(t), \tau(t))$，可以用最小二乘估计参数：

$$\hat{\theta} = (Y^T Y)^{-1} Y^T \tau$$

**激励轨迹设计**（excitation trajectory）：为了使 $Y^T Y$ 条件良好，需要设计遍历性的运动轨迹——通常用 Fourier 级数参数化，优化使 $\text{cond}(Y^T Y)$ 最小。

---

## 14. Euler-Poincare 方程预告——从广义坐标到 Lie 群 ⭐⭐⭐⭐

### 14.1 当位形空间是 Lie 群

对于浮基机器人，位形空间是 $\mathcal{Q} = SE(3) \times \mathbb{T}^n$。基座部分 $g \in SE(3)$ 不是向量——它是李群元素。传统的 Euler-Lagrange 方程假设 $q \in \mathbb{R}^n$，不能直接使用。

解决方案：**Euler-Poincare 方程**。

当 Lagrangian 在群作用下**左不变**（$L(g, \dot{g}) = \ell(\xi)$，其中 $\xi = g^{-1}\dot{g} \in \mathfrak{g}$ 是体坐标系速度），运动方程为：

$$\frac{d}{dt}\frac{\partial \ell}{\partial \xi} = \text{ad}^*_\xi \frac{\partial \ell}{\partial \xi} + \tau$$

### 14.2 SO(3) 特例——Euler 方程

对刚体绕质心旋转（$g \in SO(3)$，$\xi = \omega$），$\ell(\omega) = \frac{1}{2}\omega^T I\omega$：

$$\frac{d}{dt}(I\omega) = \omega \times (I\omega) + \tau$$

即经典的 **Euler 旋转方程** $I\dot{\omega} + \omega \times (I\omega) = \tau$。

这正是空间向量 Newton-Euler 方程中力矩分量的来源！

### 14.3 与标准方程的关系

对于浮基 $n$-DOF 机器人，完整方程是 Euler-Poincare（基座部分）+ Euler-Lagrange（关节部分）的混合：

$$\begin{pmatrix} M_{bb} & M_{bj} \\ M_{jb} & M_{jj} \end{pmatrix}\begin{pmatrix} \dot{v}_b \\ \ddot{q}_j \end{pmatrix} + \begin{pmatrix} h_b \\ h_j \end{pmatrix} = \begin{pmatrix} f_b \\ \tau_j \end{pmatrix}$$

其中 $v_b \in \mathfrak{se}(3) \cong \mathbb{R}^6$ 是基座的空间速度（Lie 代数元素），$h_b$ 包含 Coriolis 项和 $\text{ad}^*$ 项。

这是下一专题（SE(3) 几何力学）的核心内容。

---

## 15. 数值积分方法比较 ⭐⭐

### 15.1 为什么积分方法的选择很重要

标准方程 $M\ddot{q} + C\dot{q} + g = \tau$ 是一个非线性 ODE。数值积分方法的选择直接影响：

1. **精度**：截断误差的阶数
2. **稳定性**：步长多大才不发散
3. **能量行为**：长时仿真中能量是否漂移

### 15.2 常用方法对比

| 方法 | 阶数 | 保辛？ | 能量行为 | 适用场景 |
|------|------|--------|---------|---------|
| 显式 Euler | 1 | 否 | 能量增长 | 仅教学用 |
| 半隐式 Euler | 1 | 是 | 能量振荡有界 | MuJoCo 默认 |
| RK4 | 4 | 否 | 缓慢能量漂移 | 短时仿真 |
| Stormer-Verlet | 2 | 是 | 能量振荡有界 | 长时仿真 |
| 隐式中点 | 2 | 是 | 能量振荡有界 | 精确仿真 |
| Radau IIA | 5 | 否 | 数值耗散 | 刚性系统 |

### 15.3 辛积分器的威力

Stormer-Verlet（又叫 Leapfrog）算法：

$$p_{n+1/2} = p_n - \frac{h}{2}\frac{\partial H}{\partial q}\bigg|_{q_n}$$
$$q_{n+1} = q_n + h \frac{\partial H}{\partial p}\bigg|_{p_{n+1/2}}$$
$$p_{n+1} = p_{n+1/2} - \frac{h}{2}\frac{\partial H}{\partial q}\bigg|_{q_{n+1}}$$

对于机器人：
$$p_{n+1/2} = p_n + \frac{h}{2}(\tau_n - g(q_n) + \text{Coriolis correction})$$
$$q_{n+1} = q_n + h \cdot M^{-1}(q_n) p_{n+1/2}$$
$$p_{n+1} = p_{n+1/2} + \frac{h}{2}(\tau_{n+1} - g(q_{n+1}) + \text{Coriolis correction})$$

**为什么保辛很重要？** 保辛积分器保持相空间体积，这意味着能量误差是**有界振荡**的而非**单调增长**的。跑 10000 步后，RK4 的能量误差可能增长到 1%，而 Verlet 的能量误差仍在 0.001% 以内振荡。

```python
"""
对比 RK4 和 Stormer-Verlet 的能量漂移
"""
import numpy as np

# 简单摆：q'' + g/l * sin(q) = 0
# M = ml^2, V = -mgl*cos(q)
# H = p^2/(2ml^2) - mgl*cos(q)

m, l, g_val = 1.0, 1.0, 9.81
dt = 0.01
N = 100000  # 100秒

# 初始条件
q0, p0 = 1.0, 0.0  # 初始角度1弧度，零速

def H(q, p):
    """Hamiltonian"""
    return p**2 / (2*m*l**2) - m*g_val*l*np.cos(q)

# === RK4 ===
def rk4_step(q, p, dt):
    def dqdt(q, p): return p / (m*l**2)
    def dpdt(q, p): return -m*g_val*l*np.sin(q)
    
    k1q = dqdt(q, p)
    k1p = dpdt(q, p)
    k2q = dqdt(q+dt/2*k1q, p+dt/2*k1p)
    k2p = dpdt(q+dt/2*k1q, p+dt/2*k1p)
    k3q = dqdt(q+dt/2*k2q, p+dt/2*k2p)
    k3p = dpdt(q+dt/2*k2q, p+dt/2*k2p)
    k4q = dqdt(q+dt*k3q, p+dt*k3p)
    k4p = dpdt(q+dt*k3q, p+dt*k3p)
    
    return (q + dt/6*(k1q+2*k2q+2*k3q+k4q),
            p + dt/6*(k1p+2*k2p+2*k3p+k4p))

# === Stormer-Verlet ===
def verlet_step(q, p, dt):
    p_half = p - dt/2 * m*g_val*l*np.sin(q)
    q_new = q + dt * p_half / (m*l**2)
    p_new = p_half - dt/2 * m*g_val*l*np.sin(q_new)
    return q_new, p_new

# 运行并记录能量
H0 = H(q0, p0)

q_rk, p_rk = q0, p0
q_vl, p_vl = q0, p0
energy_err_rk = []
energy_err_vl = []

for i in range(N):
    q_rk, p_rk = rk4_step(q_rk, p_rk, dt)
    q_vl, p_vl = verlet_step(q_vl, p_vl, dt)
    
    if i % 100 == 0:
        energy_err_rk.append(abs(H(q_rk, p_rk) - H0))
        energy_err_vl.append(abs(H(q_vl, p_vl) - H0))

print(f"RK4 final energy error: {energy_err_rk[-1]:.6e}")
print(f"Verlet final energy error: {energy_err_vl[-1]:.6e}")
# RK4 误差随时间单调增长
# Verlet 误差有界振荡
```

---

## ⚠️ 常见陷阱

> ⚠️ **编程陷阱**：Pinocchio CRBA 只填充上三角
>
> **错误做法**：直接使用 `data.M` 进行矩阵运算，假设它是完整对称矩阵
>
> **现象**：$M\ddot{q}$ 计算结果不对称，与 RNEA 不一致
>
> **根本原因**：Pinocchio 的 `crba()` 为节省计算，只填充上三角。下三角保留旧值（可能是垃圾）
>
> **正确做法**：`M = data.M; M = np.triu(M) + np.triu(M,1).T` 或使用 `data.M.selfadjointView<Eigen::Upper>()`

> 💡 **概念误区**：认为"C 矩阵唯一确定"
>
> **新手想法**："给定 $M(q)$，$C(q,\dot{q})$ 就确定了"
>
> **实际上**：$C$ 不唯一！只有 $C(q,\dot{q})\dot{q}$（向量值）唯一。任何 $C' = C + S$（其中 $S\dot{q} = 0$）也合法。Christoffel 构造是一种特定选择，它保证了强版本斜对称性。不同库返回的 C 矩阵可能略有不同
>
> **正确认知**：当需要显式 C 矩阵时（如自适应控制），总是用 Christoffel 构造。当只需要 $C\dot{q}$ 时（如仿真），用 RNEA 更高效

> 🧠 **思维陷阱**：认为"M(q) 条件数大 = 运动学奇异"
>
> **新手想法**："$M(q)$ 快不可逆了，说明机器人在奇异位形"
>
> **实际上**：关节空间奇异（$M$ 退化）几乎不会发生——$M$ 对物理有效的参数总是正定的。工程师通常担心的是**操作空间奇异**（Jacobian $J$ 失秩），这使操作空间惯性 $\Lambda = (JM^{-1}J^T)^{-1}$ 退化。但此时 $M$ 本身仍然条件良好
>
> **正确认知**：区分关节空间（$M$）和操作空间（$\Lambda$）的条件数

> ⚠️ **编程陷阱**：Hamilton 方程中 p 和 (q, q̇) 的同步
>
> **错误做法**：数值积分 Hamilton 方程时，更新 $p$ 后不重新计算 $\dot{q} = M^{-1}(q_{new})p_{new}$
>
> **现象**：能量缓慢漂移，看似稳定但长时间后发散
>
> **根本原因**：$p = M(q)\dot{q}$ 是约束关系；如果 $q$ 变了但 $p$ 用旧的 $M$ 计算，$(\dot{q}, p)$ 不一致
>
> **正确做法**：使用专门的辛积分器（Stormer-Verlet），它在结构上保证一致性

---

## 练习题

### 基础练习（⭐-⭐⭐）

**练习 1**：对 2R 臂（上面 §8.1 的模型），用 $\ell_1 = \ell_2 = 1$ m, $m_1 = m_2 = 1$ kg, $q = (30°, 45°)$, $\dot{q} = (1, -0.5)$ rad/s：
(a) 数值计算 $M(q)$ 并验证正定（计算特征值）
(b) 数值计算 $C(q,\dot{q})\dot{q}$ 和 $g(q)$
(c) 若 $\ddot{q} = (0.1, -0.2)$，求所需关节力矩 $\tau$

**练习 2**：证明对任意 2×2 正定对称矩阵 $M$，由 Christoffel 符号构造的 $C$ 总使 $\dot{M}-2C$ 反对称。（这是上面证明的特殊化，但要求完全展开每一步。）

**练习 3**：写出 1-DOF 单摆（质量 $m$、长 $\ell$、角度 $\theta$）的标准方程。验证 $M = m\ell^2$（常数），$C = 0$，$g = mg\ell\cos\theta$。为什么 $C = 0$？

### 进阶练习（⭐⭐⭐）

**练习 4**（编程）：用 SymPy 自动推导 3R 空间机械臂的 $M$、$C$、$g$。与 Pinocchio 的数值结果对比。

**练习 5**：对 2R 臂，写出 Hamilton 正则方程。用 RK4 和 Stormer-Verlet 分别仿真 100 秒的自由运动（$\tau = 0$），对比两种方法的能量误差 $|H(t) - H(0)|$。

**练习 6**：证明 Slotine-Li 自适应控制器的全局渐近稳定性。需要：(a) 写出 Lyapunov 函数 (b) 对 $V$ 求导并利用 $\dot{M}-2C$ 的强斜对称性 (c) 用 Barbalat 引理得出 $s \to 0$。

### 跨章综合练习

**练习 7**：结合本章的 Lagrange 方程和上一章的空间向量，对 2R 臂：
(a) 用 Christoffel 符号计算 $C(q,\dot{q})\dot{q}$
(b) 用 RNEA 空间向量递推计算相同的量
(c) 证明两者数值相等
(d) 讨论两种方法的计算复杂度对比

---

## 本章小结

| 概念 | 核心公式 | 物理含义 | 工程应用 |
|------|---------|---------|---------|
| 广义坐标 | $q \in \mathcal{Q}$ | 最小坐标描述构型 | URDF 关节角 |
| d'Alembert 原理 | $\sum(F_i - m_i a_i)\cdot\delta r_i = 0$ | 约束力消失 | 免去计算内力 |
| Euler-Lagrange | $\frac{d}{dt}\frac{\partial L}{\partial\dot{q}} - \frac{\partial L}{\partial q} = \tau$ | 最小作用量 | 推导运动方程 |
| 质量矩阵 | $M(q) = \sum J_i^T B_i J_i$ | 广义惯量 | `pin::crba` |
| Christoffel 符号 | $c_{ijk} = \frac{1}{2}(\partial_k m_{ij} + \partial_j m_{ik} - \partial_i m_{jk})$ | Coriolis/离心力 | `pin::computeCoriolisMatrix` |
| 斜对称性 | $\dot{M} - 2C$ 反对称 | 能量守恒结构 | 被动性/自适应控制 |
| Hamilton | $\dot{q} = \partial H/\partial p$, $\dot{p} = -\partial H/\partial q + \tau$ | 辛结构 | 保辛积分/MPC |
| Poisson 括号 | $\{F,G\} = \partial_q F \cdot \partial_p G - \partial_p F \cdot \partial_q G$ | 守恒量判据 | 对称性分析 |

---

## 累积项目：本章新增模块

**项目**：从零构建多体动力学验证工具

本章新增：**Lagrangian 动力学模块**

```python
# 文件: lagrangian_dynamics.py
# 本章新增功能：
# 1. SymbolicManipulator 类：SymPy 自动推导 M, C, g
# 2. ChristoffelC 类：Christoffel 符号构造 C 矩阵
# 3. verify_skew_symmetry()：数值验证 Ṁ-2C 反对称
# 4. compare_with_pinocchio()：与 Pinocchio RNEA 对比
# 5. hamilton_simulation()：Hamilton 方程 + 辛积分器

# 后续章节将在此基础上添加：
# Ch3: RNEA/ABA/CRBA 算法实现
# Ch4: 被动性/自适应控制器
# Ch5: MPC 动力学约束
```

---

## 16. 3R 空间机械臂完整推导 ⭐⭐⭐

### 16.1 模型描述

三个旋转关节，关节轴分别为 $z$、$y$、$y$（DH 约定常见配置）。连杆参数：

| 连杆 | 长度 | 质量 | 质心位置（沿连杆） | 惯性 |
|------|------|------|------------------|------|
| 1 | $\ell_1$ | $m_1$ | $\ell_1/2$ | $I_1 = m_1\ell_1^2/12$ |
| 2 | $\ell_2$ | $m_2$ | $\ell_2/2$ | $I_2 = m_2\ell_2^2/12$ |
| 3 | $\ell_3$ | $m_3$ | $\ell_3/2$ | $I_3 = m_3\ell_3^2/12$ |

### 16.2 为什么 3R 比 2R 复杂得多

从 2R 到 3R，复杂度不是线性增长而是**组合爆炸**：

| 量 | 2R | 3R | 增长 |
|----|----|----|------|
| $M$ 的独立元素 | 3 | 6 | 2× |
| Christoffel 符号 $c_{ijk}$ | $2^3 = 8$ 个（大部分为零） | $3^3 = 27$ 个 | 3.4× |
| $C$ 的每个元素求和项 | 2 项 | 3 项 | 1.5× |
| 总公式长度 | ≈1 页 | ≈5 页 | 5× |

这就是为什么 3-DOF 以上几乎不做手动推导——用 SymPy 或 RNEA。

### 16.3 SymPy 自动推导

```python
"""
3R 空间臂的自动动力学推导
"""
from sympy import *

# 广义坐标
q1, q2, q3 = symbols('q1 q2 q3')
dq1, dq2, dq3 = symbols('dq1 dq2 dq3')
l1, l2, l3 = symbols('l1 l2 l3', positive=True)
m1, m2, m3 = symbols('m1 m2 m3', positive=True)
g_sym = symbols('g', positive=True)

# 关节1绕z轴，关节2绕y轴，关节3绕y轴
# 连杆质心位置（世界系）

# 连杆1：绕z轴旋转q1，质心在半高处
x_c1 = 0  # z轴旋转不改变质心的xy位置（假设关节在原点）
y_c1 = 0
z_c1 = l1/2  # 质心在连杆中点

# 连杆2的基座在(0, 0, l1)，绕y轴旋转q2
x_c2 = l2/2 * sin(q2) * cos(q1)
y_c2 = l2/2 * sin(q2) * sin(q1)
z_c2 = l1 + l2/2 * cos(q2)

# 连杆3的基座在连杆2末端
x_c3_base = l2 * sin(q2) * cos(q1)
y_c3_base = l2 * sin(q2) * sin(q1)
z_c3_base = l1 + l2 * cos(q2)

x_c3 = x_c3_base + l3/2 * sin(q2+q3) * cos(q1)
y_c3 = y_c3_base + l3/2 * sin(q2+q3) * sin(q1)
z_c3 = z_c3_base + l3/2 * cos(q2+q3)

# 速度
q_list = [q1, q2, q3]
dq_list = [dq1, dq2, dq3]

def velocity_squared(x, y, z, q_list, dq_list):
    """计算 v^2 = dx^2 + dy^2 + dz^2"""
    vx = sum(diff(x, q_list[i]) * dq_list[i] for i in range(len(q_list)))
    vy = sum(diff(y, q_list[i]) * dq_list[i] for i in range(len(q_list)))
    vz = sum(diff(z, q_list[i]) * dq_list[i] for i in range(len(q_list)))
    return expand(vx**2 + vy**2 + vz**2)

# 动能（简化：只考虑平动动能，忽略转动惯量——教学演示）
T = (Rational(1,2) * m1 * velocity_squared(x_c1, y_c1, z_c1, q_list, dq_list) +
     Rational(1,2) * m2 * velocity_squared(x_c2, y_c2, z_c2, q_list, dq_list) +
     Rational(1,2) * m3 * velocity_squared(x_c3, y_c3, z_c3, q_list, dq_list))

T = trigsimp(expand(T))

# 提取 M 矩阵
n = 3
M = zeros(n, n)
for i in range(n):
    for j in range(n):
        # M_ij = d^2T / (d dq_i d dq_j)
        M[i,j] = diff(diff(T, dq_list[i]), dq_list[j])
M = trigsimp(M)

print("M(q) for 3R arm:")
pprint(M)

# 势能
V = m1*g_sym*z_c1 + m2*g_sym*z_c2 + m3*g_sym*z_c3

# 重力项
g_vec = Matrix([diff(V, qi) for qi in q_list])
g_vec = trigsimp(g_vec)

print("\ng(q) for 3R arm:")
pprint(g_vec)

# Christoffel 符号和 C 矩阵
def christoffel(M, q, i, j, k):
    return Rational(1,2)*(diff(M[i,j], q[k]) + diff(M[i,k], q[j]) - diff(M[j,k], q[i]))

C = zeros(n, n)
for i in range(n):
    for j in range(n):
        for k in range(n):
            C[i,j] += christoffel(M, q_list, i, j, k) * dq_list[k]
C = trigsimp(C)

print("\nC(q,dq) for 3R arm:")
pprint(C)

# 验证斜对称性
Mdot = zeros(n, n)
for i in range(n):
    for j in range(n):
        for k in range(n):
            Mdot[i,j] += diff(M[i,j], q_list[k]) * dq_list[k]

N = trigsimp(Mdot - 2*C)
print("\nN = Mdot - 2C:")
pprint(N)
print("\nN + N^T (should be zero):")
pprint(trigsimp(N + N.T))
```

### 16.4 Pinocchio 对比验证

```python
"""
与 Pinocchio 数值对比验证 3R 臂推导
"""
import pinocchio as pin
import numpy as np

# 手动构建 3R 模型
model = pin.Model()

# 关节1：绕z轴
joint1_id = model.addJoint(0, pin.JointModelRZ(), pin.SE3.Identity(), "joint1")
model.appendBodyToJoint(joint1_id, 
    pin.Inertia(1.0, np.array([0, 0, 0.5]), np.diag([0.01, 0.01, 0.083])),
    pin.SE3.Identity())

# 关节2：绕y轴，在关节1上方l1处
T_12 = pin.SE3(np.eye(3), np.array([0, 0, 1.0]))  # l1 = 1.0
joint2_id = model.addJoint(joint1_id, pin.JointModelRY(), T_12, "joint2")
model.appendBodyToJoint(joint2_id,
    pin.Inertia(1.0, np.array([0.5, 0, 0]), np.diag([0.01, 0.083, 0.083])),
    pin.SE3.Identity())

# 关节3：绕y轴，在关节2处偏移l2
T_23 = pin.SE3(np.eye(3), np.array([1.0, 0, 0]))  # l2 = 1.0
joint3_id = model.addJoint(joint2_id, pin.JointModelRY(), T_23, "joint3")
model.appendBodyToJoint(joint3_id,
    pin.Inertia(1.0, np.array([0.5, 0, 0]), np.diag([0.01, 0.083, 0.083])),
    pin.SE3.Identity())

data = model.createData()

# 随机构型验证
q = np.array([0.5, 0.3, -0.2])
v = np.array([1.0, -0.5, 0.3])
a = np.array([0.1, -0.2, 0.15])

# RNEA
tau = pin.rnea(model, data, q, v, a)

# CRBA + nle
M = pin.crba(model, data, q).copy()
M = np.triu(M) + np.triu(M, 1).T
nle = pin.nonLinearEffects(model, data, q, v)

tau_verify = M @ a + nle
print(f"RNEA vs M*a+nle: max error = {np.max(np.abs(tau - tau_verify)):.2e}")
```

---

## 17. 摩擦模型在标准方程中的位置 ⭐⭐

### 17.1 为什么需要摩擦模型

理想的标准方程 $M\ddot{q} + C\dot{q} + g = \tau$ 假设关节无摩擦。但真实机器人的关节摩擦可以占总力矩的 10-30%，尤其在低速运动时。忽略摩擦会导致：

- 位置控制器的稳态误差
- 系统辨识的参数偏差
- 仿真与实际的差距（sim-to-real gap 的重要来源之一）

### 17.2 常用摩擦模型

修正后的标准方程：

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) + \tau_f(\dot{q}) = \tau$$

**Coulomb + Viscous 模型**（最常用）：

$$\tau_{f,i} = \mu_{c,i} \text{sign}(\dot{q}_i) + \mu_{v,i} \dot{q}_i$$

- $\mu_c$：Coulomb 摩擦系数（与速度大小无关，只与方向有关）
- $\mu_v$：粘性摩擦系数（与速度成正比）

**Stribeck 模型**（更精确）：

$$\tau_{f,i} = \left[\mu_{c,i} + (\mu_{s,i} - \mu_{c,i})e^{-(\dot{q}_i/v_s)^2}\right]\text{sign}(\dot{q}_i) + \mu_{v,i}\dot{q}_i$$

- $\mu_s$：静摩擦系数（$\mu_s > \mu_c$）
- $v_s$：Stribeck 速度（过渡速度）

**数值问题**：$\text{sign}(\dot{q})$ 在 $\dot{q} = 0$ 处不连续，导致仿真器困难。常用的光滑化近似：

$$\text{sign}(\dot{q}) \approx \tanh(\dot{q}/\epsilon)$$

### 17.3 对控制器设计的影响

- **被动性**：加入粘性摩擦后系统更"被动"（$\mu_v \dot{q}^2 > 0$ 总是耗散能量），稳定性更好
- **Christoffel 斜对称性**：摩擦不影响 $\dot{M}-2C$ 的性质——摩擦项独立于 $M$, $C$
- **自适应控制**：$\mu_c$, $\mu_v$ 也可以作为未知参数纳入回归矩阵 $Y$

---

## 18. 电机动力学与反射惯量 ⭐⭐

### 18.1 电机力矩与关节力矩的关系

关节由电机通过减速器驱动。设减速比为 $r_i$：

$$\tau_{joint,i} = r_i \tau_{motor,i}$$

$$\dot{q}_{joint,i} = \dot{q}_{motor,i} / r_i$$

### 18.2 反射惯量

电机转子的惯量 $J_{m,i}$ 反映到关节空间：

$$J_{reflected,i} = r_i^2 J_{m,i}$$

对于高减速比（$r = 100$），反射惯量可以很大——$r^2 = 10000$ 倍的放大！

修正后的质量矩阵（假设转子惯量与连杆惯量解耦）：

$$\bar{M}(q) = M(q) + B$$

其中 $B = \text{diag}(r_1^2 J_{m,1}, \ldots, r_n^2 J_{m,n})$。

### 18.3 实际影响

| 关节类型 | 典型减速比 | 反射惯量占比 |
|---------|-----------|------------|
| 直驱（quasi-direct drive） | 6-10 | 5-15% |
| 谐波减速器 | 50-160 | 50-80% |
| RV 减速器 | 30-100 | 30-60% |
| 行星减速器 | 3-10 | 10-25% |

**忽略反射惯量的后果**：
- 质量矩阵偏小 20-50%
- 控制器增益需要更大才能达到相同响应速度
- 系统辨识结果出现系统性偏差

> **反事实推理**：如果不考虑反射惯量，计算力矩控制器 $\tau = \hat{M}\ddot{q}_d + C\dot{q} + g$ 中 $\hat{M}$ 偏小。结果是实际加速度 $\ddot{q}_{actual} = M_{true}^{-1}\hat{M}\ddot{q}_d < \ddot{q}_d$——机器人总是"反应迟钝"。增大控制增益可以部分补偿，但会恶化噪声放大和振荡。正确做法是在 $\hat{M}$ 中加入反射惯量。

---

## 19. 广义坐标 vs 操作空间的动力学 ⭐⭐⭐

### 19.1 操作空间动力学方程

Khatib (1987) 将关节空间方程变换到操作空间（末端执行器的任务空间）：

$$\Lambda(x)\ddot{x} + \mu(x, \dot{x}) + \rho(x) = F$$

其中：
- $\Lambda(x) = (J M^{-1} J^T)^{-1}$：操作空间惯性矩阵
- $\mu = \Lambda(J M^{-1} C - \dot{J})\dot{q}$：操作空间 Coriolis 项
- $\rho = \Lambda J M^{-1} g$：操作空间重力项
- $F = \Lambda J M^{-1} \tau$：操作空间力

### 19.2 动力学一致伪逆

从关节力矩到操作空间力的映射：

$$\tau = J^T F + (I - J^T J^{-T})N_{null}\tau_0$$

其中 $J^{-T}$ 是**动力学一致伪逆**的转置。$N_{null} = I - J^{\#}J$ 是零空间投影。

> **跨领域类比**：操作空间动力学方程之于关节空间方程，就像笛卡尔坐标方程之于极坐标方程——本质是同一个物理系统，但从不同"视角"描述。在需要控制末端轨迹时，操作空间描述更自然；在需要控制关节力矩时，关节空间更直接。全身控制（WBC）中两种视角经常交替使用。

### 19.3 与全身控制的接口

全身控制器（Whole-Body Controller）通常将标准方程作为等式约束，加上接触力和任务优先级：

$$\min_{\ddot{q}, \tau, \lambda} \sum_k w_k \|J_k\ddot{q} + \dot{J}_k\dot{q} - \ddot{x}_k^{des}\|^2$$

$$\text{s.t.}\quad M\ddot{q} + C\dot{q} + g = S^T\tau + J_c^T\lambda$$

$$\quad\quad J_c\ddot{q} + \dot{J}_c\dot{q} = 0 \quad \text{（接触约束）}$$

$$\quad\quad \tau_{min} \leq \tau \leq \tau_{max}$$

这正是 TSID/OCS2 等框架的核心——标准方程是所有约束中最基本的一个。

---

## 20. 核心教材深度对照表 ⭐

| 教材 | 核心章节 | 最佳用途 |
|------|---------|---------|
| **Goldstein** 3rd ed. | Ch.1-2 Lagrange, Ch.8 Hamilton, Ch.9 Poisson | 分析力学最严谨的物理教材，适合深入理论 |
| **Murray-Li-Sastry** | Ch.4 动力学 | 机器人动力学的数学首推，螺旋/指数风格统一 |
| **Lynch-Park** | Ch.8 | 推导最清晰，配套视频+代码，工程师首选 |
| **Featherstone** | Ch.5 RNEA, Ch.6 CRBA | 空间向量算法唯一权威，搭配上一专题 |
| **Spong** 2nd ed. | Ch.7 | Christoffel 和 Ṁ-2C 讲得最好 |
| **Siciliano** | Ch.7-8 | 欧洲派，推导细致，与 MLS/Spong 并列"三大" |
| **Arnold** | Ch.3, 8-10 | 辛几何数学家视角，难但启发性极强 |
| **Marsden-Ratiu** | Ch.2, 5, 11, 13 | 几何力学标准参考，为后续专题打基础 |

**阅读建议**：
- 第一遍用 Lynch-Park 建立直觉
- 第二遍用 Spong 深入 Christoffel 和控制性质
- 需要算法实现时查 Featherstone
- 做研究时参考 Murray-Li-Sastry 和 Marsden-Ratiu

---

## 延伸阅读

| 资源 | 难度 | 推荐理由 |
|------|------|---------|
| Goldstein, *Classical Mechanics* 3rd ed. Ch.1-2, Ch.8 | ⭐⭐ | 分析力学圣经，物理直觉最好 |
| Murray-Li-Sastry, *A Mathematical Introduction to Robotic Manipulation* Ch.4 | ⭐⭐ | 机器人动力学的数学严格处理 |
| Lynch-Park, *Modern Robotics* Ch.8 | ⭐⭐ | 推导清晰，有配套视频 |
| Featherstone, *RBDA* Ch.2, 5, 6 | ⭐⭐ | 空间向量视角的标准方程 |
| Spong-Hutchinson-Vidyasagar, *Robot Modeling and Control* Ch.7 | ⭐ | Christoffel 和斜对称性讲得最好 |
| Slotine-Li 1987 IJRR 论文 | ⭐⭐⭐ | 自适应控制经典，用 Ṁ-2C |
| Arnold, *Mathematical Methods of Classical Mechanics* | ⭐⭐⭐⭐ | 辛几何形式化，数学家视角 |
| Marsden-Ratiu, *Introduction to Mechanics and Symmetry* | ⭐⭐⭐⭐ | 几何力学标准参考 |

---

## 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| M 矩阵非正定（Cholesky 失败） | URDF 惯性参数非法 | 1. 检查所有连杆的惯性张量正定性 2. 验证三角不等式 3. 确认质量 > 0 | §5.2 |
| RNEA 结果与 M*a+C*v+g 不一致 | CRBA 只填上三角 | 1. 补全 M 的下三角 2. 确认重力方向一致 3. 检查关节顺序 | §7 |
| Ṁ-2C 不反对称（数值误差 > 1e-6） | C 不是 Christoffel 构造 | 1. 确认用 `computeCoriolisMatrix` 2. 检查数值微分步长 3. 避免奇异构型附近 | §5.4 |
| 被动性控制器能量增长 | 实现了错误的 C 矩阵 | 1. 验证 $\dot{q}^T(\dot{M}-2C)\dot{q} \approx 0$ 2. 检查重力补偿符号 3. 确认离散化不引入能量 | §5.4 |
| Hamilton 仿真能量漂移 | 使用了非辛积分器 | 1. 改用 Stormer-Verlet 2. 减小步长确认是否为截断误差 3. 监控 $|H(t)-H(0)|$ | §6.4 |
| SymPy 推导结果与 Pinocchio 不匹配 | 坐标系/参考点约定不同 | 1. 确认质心位置定义 2. 检查角度零位定义 3. 统一重力方向 | §8 |
| 自适应控制器参数不收敛 | 激励不足或增益过大 | 1. 检查回归矩阵 $Y$ 的条件数 2. 降低自适应增益 $\Gamma$ 3. 设计激励轨迹 | §10.5 |
| 反射惯量导致系统辨识偏差 | 未将电机惯量纳入模型 | 1. 在 M 中加入 $r^2 J_m$ 2. 检查减速比是否正确 3. 对比空载与负载辨识结果 | §18 |

---

## 附录 A：核心公式速查表

### A.1 标准方程及其分量

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$$

| 符号 | 含义 | 维度 | 计算方法 | Pinocchio API |
|------|------|------|---------|--------------|
| $M(q)$ | 质量矩阵 | $n \times n$ | CRBA $O(n^2)$ | `crba(model, data, q)` |
| $C(q,\dot{q})$ | Coriolis 矩阵 | $n \times n$ | Christoffel $O(n^3)$ | `computeCoriolisMatrix(model, data, q, v)` |
| $g(q)$ | 重力向量 | $n \times 1$ | RNEA $O(n)$ | `computeGeneralizedGravity(model, data, q)` |
| $C\dot{q}+g$ | 非线性项 | $n \times 1$ | RNEA $O(n)$ | `nonLinearEffects(model, data, q, v)` |
| $\tau$ | 逆动力学 | $n \times 1$ | RNEA $O(n)$ | `rnea(model, data, q, v, a)` |
| $\ddot{q}$ | 正动力学 | $n \times 1$ | ABA $O(n)$ | `aba(model, data, q, v, tau)` |

### A.2 关键性质

| 性质 | 公式 | 前提条件 | 应用 |
|------|------|---------|------|
| $M$ 对称 | $M = M^T$ | 总是成立 | Cholesky 分解可用 |
| $M$ 正定 | $\dot{q}^T M \dot{q} > 0$ ($\dot{q} \neq 0$) | 串联臂总是成立 | $M^{-1}$ 存在 |
| 弱斜对称 | $\dot{q}^T(\dot{M}-2C)\dot{q} = 0$ | 任何合法 $C$ | 被动性证明 |
| 强斜对称 | $x^T(\dot{M}-2C)x = 0, \forall x$ | **仅 Christoffel 构造** | Slotine-Li |
| 参数线性 | $M\ddot{q}_r + C\dot{q}_r + g = Y\theta$ | 总是成立 | 自适应控制/系统辨识 |
| 能量守恒 | $\dot{H} = \dot{q}^T\tau$ | 总是成立 | Lyapunov 分析 |

### A.3 三种 C 矩阵构造方法

| 方法 | 公式 | 复杂度 | 保证斜对称？ | 适用场景 |
|------|------|--------|------------|---------|
| Christoffel | $C_{ij} = \sum_k c_{ijk}\dot{q}_k$ | $O(n^3)$ | 强版本 | 控制器设计 |
| RNEA | $C\dot{q} = \text{RNEA}(q,\dot{q},0,0)$ | $O(n)$ | 仅向量乘积 | 仿真/数值 |
| 数值微分 | $C_{ij} \approx [M(q+\epsilon e_j) - M(q-\epsilon e_j)] / 2\epsilon$ | $O(n^3)$ | 否 | 调试验证 |

---

## 附录 B：与后续专题的桥梁

### 向前回顾

本章使用了以下前置知识：

- **10_空间向量代数**：RNEA 的递推公式、6×6 空间惯性、运动/力叉积——用于理解 C 矩阵的空间向量构造
- **线性代数**：正定矩阵、Cholesky 分解、特征值——用于理解 $M$ 的性质
- **微分几何基础**：流形、切空间——用于理解位形空间和广义速度

### 向后预告

本章建立的知识将在以下后续章节中使用：

| 后续专题 | 使用本章的内容 | 具体接口 |
|---------|--------------|---------|
| **RNEA/ABA/CRBA 算法** | 标准方程的算法实现 | 本章 §7 概述，下章详细展开 |
| **SE(3) 几何力学** | Euler-Poincare 方程 | 本章 §14 预告 → 下章正式推导 |
| **解析微分** | $\partial(M\ddot{q}+C\dot{q}+g)/\partial q$ | Carpentier-Mansard RSS 2018 |
| **约束动力学** | 带 $J_c^T\lambda$ 的运动方程 | 本章 §12 预热 → 专门章节展开 |
| **辛结构与动量映射** | Hamilton 方程的辛几何 | 本章 §6.4 预告 → 深入展开 |
| **MPC/轨迹优化** | 标准方程作为等式约束 | 本章 §13.1 |
| **全身控制 WBC** | 标准方程 + 接触约束 + 任务优先级 | 本章 §19.3 |

### 核心衔接

下一专题（RNEA/ABA/CRBA 三大算法）将详细展开本章已预告的三个 $O(n)$-$O(n^2)$ 算法。关键衔接点：

- 本章的 $M(q) = \sum J_i^T B_i J_i$（Jacobian 叠加法）→ 下章的 CRBA 递推（不显式形成 $J_i$）
- 本章的 $C\dot{q} = \text{RNEA}(q,\dot{q},0,0)$ → 下章 RNEA 的逐步伪代码
- 本章的 $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$（$O(n^3)$）→ 下章的 ABA（$O(n)$）

---

## 附录 C：关键定理与核心公式索引

| # | 定理/公式 | 位置 | 档位 |
|---|----------|------|------|
| 1 | d'Alembert 原理 | §3.2 | ⭐⭐ |
| 2 | Hamilton 原理（最小作用量） | §4.1 | ⭐ |
| 3 | Euler-Lagrange 方程 | §4.2 | ⭐ |
| 4 | 质量矩阵 Jacobian 叠加 | §5.1 | ⭐ |
| 5 | $M(q)$ 正定性证明 | §5.2 | ⭐⭐ |
| 6 | Christoffel 符号定义 | §5.3 | ⭐⭐ |
| 7 | $\dot{M}-2C$ 斜对称性证明 | §5.4 | ⭐⭐ |
| 8 | Legendre 变换 | §6.1 | ⭐⭐ |
| 9 | Hamilton 正则方程 | §6.2 | ⭐⭐ |
| 10 | Poisson 括号与守恒量 | §6.3 | ⭐⭐⭐ |
| 11 | 辛形式与 Liouville 定理 | §6.4 | ⭐⭐⭐⭐ |
| 12 | 被动性定理 | §10.2 | ⭐⭐ |
| 13 | 计算力矩控制 | §10.3 | ⭐⭐ |
| 14 | Slotine-Li 自适应控制 | §10.5 | ⭐⭐⭐ |
| 15 | Euler-Poincare 方程（预告） | §14 | ⭐⭐⭐⭐ |
| 16 | 2R 臂完整闭式 | §8.1 | ⭐ |
| 17 | 操作空间动力学 | §19 | ⭐⭐⭐ |

---

## 附录 D：C++ 库 API 对照速查

| 计算对象 | Pinocchio | Drake | RBDL | MuJoCo |
|---------|-----------|-------|------|--------|
| $M(q)$ | `crba(m,d,q)` | `CalcMassMatrix(ctx,&M)` | `CompositeRigidBodyAlgorithm(m,q,H)` | `mj_fullM(m,d,M)` |
| $C\dot{q}+g$ | `nonLinearEffects(m,d,q,v)` | `CalcBiasTerm(ctx,&Cv)` | `NonlinearEffects(m,q,dq,Tau)` | `d->qfrc_bias` |
| $g(q)$ | `computeGeneralizedGravity(m,d,q)` | `-CalcGravityForces(ctx)` | 需自行分离 | `d->qfrc_bias - d->qfrc_passive` |
| $\tau = \text{ID}(q,v,a)$ | `rnea(m,d,q,v,a)` | `CalcInverseDynamics(ctx,vdot,F)` | `InverseDynamics(m,q,dq,ddq,T)` | `mj_rne(m,d,1,res)` |
| $\ddot{q} = \text{FD}(q,v,\tau)$ | `aba(m,d,q,v,tau)` | 内部积分器 | `ForwardDynamics(m,q,dq,T,QDD)` | `mj_forward(m,d)` |
| 显式 $C$ | `computeCoriolisMatrix(m,d,q,v)` | 无直接API | 无直接API | 无直接API |

> **注意**：Drake 的 `CalcGravityGeneralizedForces` 返回 $-g(q)$（重力作为广义力，与标准方程中 $+g(q)$ 符号相反）。这是最常见的跨库 bug 来源之一。

---

## 附录 E：Noether 定理与机器人守恒量 ⭐⭐⭐

### E.1 Noether 定理的陈述

**定理**（Noether, 1918）：如果 Lagrangian $L(q, \dot{q})$ 在某单参数连续变换群 $q \to q + \epsilon \delta q$ 下不变（$\delta L = 0$），则存在一个守恒量：

$$I = \sum_i \frac{\partial L}{\partial \dot{q}_i} \delta q_i = p^T \delta q$$

其中 $p = \partial L / \partial \dot{q} = M(q)\dot{q}$ 是广义动量。

### E.2 机器人中的对称性与守恒量

| 对称性 | 变换 | 守恒量 | 机器人例子 |
|--------|------|--------|-----------|
| 时间平移 | $t \to t + \epsilon$ | 能量 $H = T + V$ | 保守系统（无驱动） |
| 空间平移 | $x \to x + \epsilon$ | 线动量 $p = m\dot{x}$ | 自由浮动航天器 |
| 空间旋转 | $\theta \to \theta + \epsilon$ | 角动量 $L = I\omega$ | 陀螺仪、自由旋转体 |
| 关节平移 | $q_k \to q_k + \epsilon$ | $p_k = M_{k,:}\dot{q}$ | $L$ 不依赖 $q_k$ 时 |

### E.3 浮基机器人的动量守恒

对于自由浮动的机器人（无外力），总动量守恒：

$$h = \begin{pmatrix} k \\ l \end{pmatrix} = \begin{pmatrix} M_{bb} & M_{bj} \\ M_{jb} & M_{jj} \end{pmatrix}\begin{pmatrix} v_b \\ \dot{q}_j \end{pmatrix} = \text{const}$$

这意味着基座速度由关节运动决定：

$$v_b = M_{bb}^{-1}(h_{total} - M_{bj}\dot{q}_j)$$

**应用**：太空机器人、自由落体中的人形机器人翻转——通过改变关节角来改变基座姿态，同时保持总动量不变。

### E.4 角动量守恒在步态中的应用

即使四足/人形机器人有地面接触（外力非零），在某些阶段（飞行相、短暂无支撑），角动量近似守恒。这是步态规划中的重要约束：

$$\dot{L}_{total} = \sum_i f_{contact,i} \times r_i \approx 0 \quad \text{（飞行相）}$$

Dai et al. (2014) 利用此约束设计人形机器人翻滚动作；Wensing et al. (2013) 用角动量约束改善了四足跳跃的落地稳定性。

---

## 附录 F：自测题完整版

### 核心推导题（档位 3）

**Q1**：为 2R 平面串联机械臂（参数见 §8.1），完整手推 $M(q)$, $C(q,\dot{q})$, $g(q)$。

**Q2**：用 Christoffel 符号定义证明 $\dot{M}-2C$ 反对称。然后构造一个合法但非 Christoffel 的 $C'$ 矩阵，使 $\dot{q}^T(\dot{M}-2C')\dot{q} = 0$ 仍成立，但存在 $x \neq \dot{q}$ 使 $x^T(\dot{M}-2C')x \neq 0$。

**Q3**：证明恒等式 $C(q,\dot{q})\dot{q} + g(q) = \text{RNEA}(q, \dot{q}, 0, g_0)$。

### 编程题（档位 3）

**Q4**：用 Pinocchio 加载 Panda URDF，在随机 $(q, \dot{q})$ 下验证 $\tau = M\ddot{q} + C\dot{q} + g$ 与 $\tau = \text{RNEA}(q,\dot{q},\ddot{q})$ 一致（误差 < $10^{-10}$）。

**Q5**：为 2R 臂写出 Hamilton 正则方程；用 RK4 和 Verlet 分别跑 100s 保守运动，记录能量误差曲线。

### 研究级题目（档位 4）

**Q6**：证明 Hamilton 流保辛（$\phi_t^*\omega = \omega$）。推论：Liouville 体积守恒。

**Q7**：实现 PD+ 控制器，用 Lyapunov $V = \frac{1}{2}\dot{q}^T M\dot{q} + \frac{1}{2}e^T K_p e$ 监控稳定性。验证 $V$ 单调下降。

### 跨章综合题

**Q8**（综合空间向量 + Lagrange）：对 2R 臂，分别用 (a) Christoffel 符号 (b) RNEA 空间向量递推 计算 $C(q,\dot{q})\dot{q}$，验证数值相等，并讨论复杂度差异。

**Q9**（综合 Lagrange + Hamilton + 控制）：设计一个能量整形控制器（energy shaping controller），把 2R 臂的闭环 Hamiltonian 修改为 $H_d(q,p) = \frac{1}{2}p^T M^{-1}p + V_d(q)$（$V_d$ 在 $q_d$ 有唯一最小值）。推导控制律并证明稳定性。

---

## 结语

本章把机器人动力学方程 $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$ 从"公式"升级为"一个由 Hamilton 原理、Lagrange 变分、Hamilton 辛几何、Featherstone 空间向量四条路径**殊途同归**的结构"。

每一条路径回答一个不同的问题：

- **变分原理**告诉我们方程为什么是这个形状（作用量驻值）
- **Christoffel 构造**告诉我们 $C$ 应该怎么写（保证 $\dot{M}-2C$ 斜对称）
- **Hamilton 形式**告诉我们能量如何守恒（辛流形 + 保辛流）
- **空间向量递归**告诉我们怎么快速算出来（$O(n)$）

掌握本章后你会获得三种新能力：
1. **读懂所有现代机器人论文**——MPC、自适应控制、Hamiltonian NN 都以这些方程为语言
2. **在 Pinocchio/Drake/MuJoCo 间自由切换**——你知道每个 API 计算哪一项
3. **为后续专题打下稳固地基**——算法细节、Lie 群推广、辛结构与动量映射

最后的"护身符"：**C 矩阵不唯一、$\dot{M}-2C$ 斜对称需要 Christoffel、RNEA 能代替 $C\dot{q}+g$**——记住这三条能避免 90% 的动力学 bug。

---

## 附录 G：视频与在线资源

| 资源 | 讲师/机构 | 覆盖内容 | 推荐度 |
|------|---------|---------|--------|
| MIT OCW 8.09 | Iain Stewart | Lagrange/Hamilton 分析力学 | ⭐⭐⭐ |
| Stanford CS223A | Oussama Khatib | 动力学 Lec 10-14 | ⭐⭐⭐ |
| Northwestern Modern Robotics Ch8 | Kevin Lynch | 动力学完整 8 讲 | ⭐⭐⭐⭐ |
| ETH Robotic Systems | Marco Hutter | 动力学+控制 | ⭐⭐⭐ |
| MIT Underactuated Robotics | Russ Tedrake | Lagrangian+控制 | ⭐⭐⭐⭐ |
| David Tong 讲义 | Cambridge | Lagrangian/Hamiltonian 物理 | ⭐⭐⭐ |
| Featherstone ICRA tutorial | Roy Featherstone | 空间向量+RNEA/ABA | ⭐⭐⭐ |

### 中文资源

| 资源 | 说明 | URL 线索 |
|------|------|---------|
| 中国大学 MOOC 熊蓉老师《机器人学导论》 | 含动力学章节 | icourse163.org |
| 深蓝学院《机器人动力学》 | Lagrange + 空间向量 + Pinocchio | shenlanxueyuan.com |
| 知乎专栏"机器人动力学 Lagrange" | 社区讨论 | zhihu.com |
| B站高翔《SLAM 与机器人数学》 | 李群+动力学入门 | bilibili.com |

### 代码仓库

| 仓库 | 用途 |
|------|------|
| `stack-of-tasks/pinocchio` | 最快的开源刚体动力学库 (C++/Python) |
| `RobotLocomotion/drake` | MIT/TRI 全家桶 |
| `NxRLab/ModernRobotics` | Lynch-Park 教材配套代码 |
| `petercorke/robotics-toolbox-python` | 教学工具箱 |
| `stack-of-tasks/crocoddyl` | Pinocchio 之上的 DDP 求解器 |

---

## 附录 H：学习时间预算

| 阶段 | 内容 | 建议时间 | 里程碑 |
|------|------|---------|--------|
| W1 | §1-4：广义坐标 + EL 推导 | 8-10h | 手推 2R 臂的 $T$ 和 $M(q)$ |
| W2 | §5：M/C/g 三构造 + 斜对称证明 | 10-12h | Christoffel 推导 + $\dot{M}-2C$ 证明 |
| W3 | §6+10：Hamilton + 被动性控制 | 8-10h | PD+/Slotine-Li 仿真跑通 |
| W4 | §7-8：算法实现 + 例题 | 6-8h | Pinocchio CRBA/RNEA 验证脚本 |
| **合计** | | **32-40h** | 交付验证脚本 + 被动性控制仿真 |

**建议节奏**：每天 2-3 小时 x 2 周。关键是**每周至少一次手推 + 一次编程验证**。

---

## 练习

### 练习 1：Christoffel 符号与 C 矩阵构造（基础）

对 2R 平面机械臂，质量矩阵为：
$$M(q) = \begin{pmatrix} m_1 l_{c1}^2 + m_2(l_1^2 + l_{c2}^2 + 2l_1 l_{c2}\cos q_2) + I_1 + I_2 & m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 \\ m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 & m_2 l_{c2}^2 + I_2 \end{pmatrix}$$

(a) 计算所有 Christoffel 符号 $c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)$（共 $2^3 = 8$ 个，利用对称性简化）。

(b) 构造 Coriolis 矩阵 $C(q, \dot q)$，其中 $C_{ij} = \sum_k c_{ijk} \dot q_k$。

(c) 验证 $\dot M - 2C$ 是反对称矩阵（这保证了被动性 $\dot V \leq u^T \dot q$）。提示：直接计算 $(\dot M - 2C) + (\dot M - 2C)^T$，利用 Christoffel 符号的对称性。

(d) 给出一组具体数值（如 $q_2 = \pi/4$, $\dot q_1 = 1$, $\dot q_2 = 2$），验证 $\dot q^T(\dot M - 2C)\dot q = 0$。

### 练习 2：Hamilton 正则方程与 Legendre 变换（中等）

从 2R 臂的 Lagrangian $L(q, \dot q) = \frac{1}{2}\dot q^T M(q) \dot q - V(q)$ 出发：

(a) 执行 Legendre 变换：$p = \partial L / \partial \dot q = M(q)\dot q$。反解 $\dot q = M^{-1}(q) p$。

(b) 写出 Hamiltonian $H(q, p) = p^T \dot q - L = \frac{1}{2} p^T M^{-1}(q) p + V(q)$。

(c) 推导 Hamilton 正则方程 $\dot q = \partial H/\partial p$，$\dot p = -\partial H / \partial q + \tau$。特别注意 $\partial H/\partial q$ 包含 $\partial M^{-1}/\partial q$ 项——用恒等式 $\partial M^{-1}/\partial q_i = -M^{-1}(\partial M/\partial q_i)M^{-1}$ 展开。

(d) 验证 $\dot H = \tau^T \dot q$（当 $\tau = 0$ 时能量守恒）。这是辛结构的直接推论，与下一章 §70（辛结构与动量映射）的联系点。

### 练习 3：CRBA 算法手算验证（编程 + 理论）

用 Pinocchio 加载一个 3-DOF 机械臂模型（或手动构造）：

(a) 对给定构型 $q_0$，用 `pin.crba(model, data, q0)` 获得 $M(q_0)$。

(b) 用数值微分验证 $M$ 是正确的：对每个 $\dot q = e_i$（单位向量），计算 $T = \frac{1}{2}e_i^T M e_i$，同时用 `pin.computeKineticEnergy(model, data, q0, e_i)` 获得动能，两者应相等。

(c) 验证 $M$ 的正定性：计算特征值，确认全部为正。

(d) 验证 Coriolis 项：用 `pin.computeCoriolisMatrix(model, data, q, dq)` 获得 $C$，用数值微分 $\dot M \approx (M(q+\epsilon\dot q) - M(q))/\epsilon$ 估算 $\dot M$，验证 $\dot M - 2C$ 的反对称性。

### 练习 4：被动性控制仿真（工程应用）

对重力环境下的 2-DOF 机械臂：

(a) 推导 PD+ 控制律：$\tau = M(q)\ddot q_d + C(q,\dot q)\dot q_d + g(q) - K_p e - K_d \dot e$（其中 $e = q - q_d$）。

(b) 写出闭环误差动力学方程，证明 Lyapunov 函数 $V = \frac{1}{2}\dot e^T M \dot e + \frac{1}{2}e^T K_p e$ 的导数 $\dot V = -\dot e^T K_d \dot e \leq 0$。

(c) 编程实现：在 Pinocchio + 数值积分器中仿真闭环系统，画出关节误差收敛曲线。

(d) 对比：去掉 $C\dot q_d$ 前馈项（退化为普通 PD + 重力补偿），观察在高速运动时跟踪误差的增大。解释为何 $\dot M - 2C$ 斜对称性是被动性控制的数学保障。

### 练习 5：跨章综合——从 Lagrange 到空间向量再到 EP 方程（进阶）

本题连接本章（Lagrange 力学）、前一章（空间向量代数）和后续章节（SE(3) 几何力学）。

(a) 对单刚体自由飞行（6 DOF，无关节），写出 body frame 中的 Lagrangian：$L = \frac{1}{2}\xi^T G_b \xi$，其中 $\xi = (\omega; v)$ 是 body twist，$G_b$ 是 spatial inertia。

(b) 直接对这个 Lagrangian 应用本章的 Euler-Lagrange 方程——但注意这里的"广义坐标"生活在 SE(3) 上，不是 $\mathbb{R}^n$！解释为什么朴素的 $\frac{d}{dt}\frac{\partial L}{\partial \dot q} - \frac{\partial L}{\partial q} = 0$ 不能直接用。

(c) 说明为什么需要 Euler-Poincare 约化（见第 40 章）：左不变 Lagrangian 允许在 Lie 代数 $\mathfrak{se}(3)$ 上写运动方程，避免了对 SE(3) 的坐标参数化。

(d) 用 Euler-Poincare 方程 $G_b\dot\xi = \mathrm{ad}^*_\xi(G_b\xi)$ 推导单刚体 Euler 方程 $I\dot\omega + \omega \times I\omega = 0$，并与本章用 Lagrange 方法推导的结果对比。

(e) 总结三者的关系链：$\text{Lagrange 力学} \xrightarrow{\text{Lie 群约化}} \text{Euler-Poincare} \xrightarrow{\text{空间向量编码}} \text{RNEA 递推}$。每一步的"信息损失"和"计算增益"分别是什么？

---
