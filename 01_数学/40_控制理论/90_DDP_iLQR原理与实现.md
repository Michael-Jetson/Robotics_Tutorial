# 专题 3.9：DDP/iLQR 原理与实现 ⭐⭐

> 博士前数学路线图 · 第三批 · 专题 3.9
> 前置：3.5 LQR 与 Riccati 方程、3.3 动态规划与 Bellman 方程、3.2 Pontryagin 极大值原理
> 面向：机器人算法工程师（RL motion control、具身智能、SLAM），C++/Python

---

## 前置自测

📋 **前置自测**（答不出 >=2 题 → 先回 3.5/3.3/3.2 复习）

1. 写出离散时间 LQR 的 Riccati 递推方程，说明 $P_k$ 的物理含义是什么？
2. Bellman 最优性原理的核心表述是什么？它把 $N$ 步决策问题如何分解？
3. 离散 PMP 的共态方程 $\lambda_k = \partial H / \partial x_k$ 中，$H$ 的定义是什么？
4. Gauss-Newton 法与完整 Newton 法的区别在哪里？各自的收敛阶是什么？
5. 什么是 Armijo 准则？为什么 line search 不能只检查函数值下降？

---

## 本章目标

学完本专题后，你应当能够：
- 从 Bellman 方程出发，独立完整推导 DDP 的 Backward pass（Q 函数六系数）和 Forward pass
- 精确说明 DDP 与 iLQR 的数学差异（张量缩并项的有无）及其对收敛率的影响
- 实现一个带 Tassa 2012 正则化与 Armijo line search 的 cart-pole iLQR
- 理解 DDP 与 PMP、DP、LQR 三条理论线的精确统一关系
- 为进入 3.10（约束 DDP/Crocoddyl）、3.11-3.12（MPC）做好数学准备

---

## 知识树概览

```
DDP/iLQR 知识树
├── 根：非线性最优控制的局部迭代求解
│   ├── 主干 1：Bellman 方程二阶展开 → Q 函数六系数
│   │   ├── 枝：阶段代价 ℓ 的展开
│   │   ├── 枝：动力学 f 的展开（含三阶张量）
│   │   └── 枝：值函数 V_{k+1} 的展开 → 链式法则
│   ├── 主干 2：Backward Pass → Riccati-like 递推
│   │   ├── 枝：最优控制修正 δu* = k + Kδx
│   │   ├── 枝：值函数递推 V_x, V_{xx}
│   │   └── 枝：正则化保证 Q_{uu} ≻ 0
│   ├── 主干 3：Forward Pass → Line Search
│   │   ├── 枝：期望下降 ΔJ(α) = αΔ₁ + α²Δ₂/2
│   │   ├── 枝：Armijo 准则
│   │   └── 枝：backtracking 策略
│   ├── 主干 4：iLQR = Gauss-Newton 近似
│   │   ├── 枝：丢弃 f_{xx}, f_{ux}, f_{uu}
│   │   ├── 枝：超线性收敛 vs 二次收敛
│   │   └── 枝：Q_{uu}^{iLQR} 自动正定
│   └── 主干 5：三条理论线统一
│       ├── 枝：DDP ↔ LQR（线性退化）
│       ├── 枝：DDP ↔ PMP（共态方程）
│       └── 枝：DDP ↔ DP（局部二次近似）
└── 应用分支
    ├── RTI / MPC warm-start
    ├── Differentiable MPC
    └── MPPI-iLQR 等价性
```

---

## §3.9.1 非线性轨迹优化问题的标准形式 ⭐⭐

### 动机：为什么需要 DDP/iLQR

在专题 3.1-3.8 中，我们已经分别研究了三条最优控制的理论线：

| 理论线 | 核心方程 | 给出什么 | 局限 |
|--------|---------|---------|------|
| PMP（3.2） | $\dot\lambda = -\partial H/\partial x$ | 一阶必要条件（开环） | 不给反馈、边值问题难解 |
| DP/HJB（3.3-3.4） | $V(x)=\min_u[\ell + V(f)]$ | 全局最优性（值函数） | 维数灾难、无法全空间求解 |
| LQR（3.5） | $P_k = Q + A^\top P_{k+1} A - \cdots$ | 闭式 Riccati 递推 | 仅限线性动力学+二次代价 |

真实机器人——四足、人形、机械臂、四旋翼——都是**高维非线性**的。它们的状态空间典型维度为 $n=12$（四旋翼）到 $n=74$（人形全身），既不能解析求解 HJB PDE，也不能在全状态空间上网格化做 DP。

> **本质洞察**：DDP 的核心思想是——**沿当前名义轨迹做 Bellman 方程的二阶 Taylor 展开**，把局部问题变成一个 TVLQR，用 3.5 的 Riccati 后向递推解出局部反馈 $\delta u^\star = k + K\,\delta x$，再前向 roll-out 得到新轨迹，迭代收敛。

这个思想为什么重要？因为它是：
- MPC 的内层求解器（3.11-3.12 的核心引擎）
- Crocoddyl/Aligator/MJPC/ALTRO 的默认算法
- Guided Policy Search 与 Differentiable MPC 的可微规划层
- 2012 年 Tassa 等把它做成 22-DoF humanoid 的在线 MPC
- 2018 年 Neunert 等在四足 ANYmal 上实现 190 Hz 全身 NMPC
- 2020 年 Mastalli 等发布 Crocoddyl 使其成为欧洲 MEMMO 项目标准求解器

类比理解：如果把非线性最优控制比作"在崎岖山谷中找到最低点"，那么 DP 是"画出整个山谷的等高线图"（维数灾难使之不可能），PMP 是"找到一条下坡路径但不告诉你路滑了怎么办"（开环无反馈），而 DDP 则是"站在当前位置，用二次曲面拟合你脚下的局部地形，沿曲面最低方向走一步，再重新拟合"。这既避免了全局计算，又提供了"如果偏离轨迹怎么修正"的反馈策略。

### 如果不用 DDP 会怎样

如果你坚持用前几章学到的方法处理非线性最优控制：

**方案 A：暴力 DP（网格化）**。对 $n=14$（7-DoF 机械臂）的状态空间做网格化，每维 100 个点需要 $100^{14} = 10^{28}$ 个网格点。即便每点只存一个 `double`（8 字节），也需要 $8\times 10^{28}$ 字节 $\approx 10^{17}$ TB 存储——这比地球上所有硬盘容量还多 $10^7$ 倍。

**方案 B：纯 PMP 打靶法**。把共态 $\lambda_0$ 作为未知数，用牛顿法解两点边值问题 $\lambda_N = \nabla \ell_f(x_N)$。问题：(i) 初值 $\lambda_0$ 的猜测极度敏感；(ii) 无反馈结构，开环执行时扰动导致发散；(iii) 不给 $K$ 矩阵（feedback gain），无法做 MPC。

**方案 C：直接用通用 NLP solver（如 IPOPT）**。把轨迹优化写成大规模 NLP，变量 $\sim N(n+m) \sim 5000$，约束 $\sim Nn \sim 3000$。可以工作，但：(i) cold-start 需要秒级；(ii) 不自然地给出反馈增益 $K$；(iii) warm-start 不如 DDP 高效。

DDP 同时解决了这三个问题：**线性复杂度 $O(N)$、自然产生反馈增益 $K$、warm-start 极佳**。

### 标准形式定义

考虑离散时间有限时域最优控制问题（OCP）：

$$
\min_{u_{0:N-1}}\ J(x_{0:N}, u_{0:N-1}) = \sum_{k=0}^{N-1} \ell(x_k, u_k) + \ell_f(x_N)
$$
$$
\text{s.t.}\quad x_{k+1} = f(x_k, u_k),\quad x_0\ \text{given.}
$$

其中：
- $x_k \in \mathbb{R}^n$：状态向量（关节角度+速度，或 SE(3) 位姿+twist）
- $u_k \in \mathbb{R}^m$：控制输入（关节力矩，或推力/力矩）
- $\ell(x_k, u_k)$：阶段代价（stage cost），衡量每一步的"坏"
- $\ell_f(x_N)$：终端代价（terminal cost），衡量最终状态的"坏"
- $f(x_k, u_k)$：离散动力学（通常来自 ODE 的 RK4 积分）
- $N$：预测时域（horizon），典型 20-200

**DDP 的单射击表述**：把 $u_{0:N-1}$ 视为唯一决策变量，状态由 $x_{k+1} = f(x_k, u_k)$ 前向推出并消去。这使问题退化为对 $u$ 的**无约束非线性优化**（暂不计 box/path 约束）。

> **与直接配点法的对比**：直接配点法（3.12 DirCol）把 $x_{0:N}$ 也作为决策变量、动力学作为等式约束，规模更大但结构更稀疏。DDP 的精髓在于：**用 Bellman 原理在时间维上分解这 $Nm$ 维问题为 $N$ 个 $m$ 维子问题**。

### 历史脉络

| 年代 | 里程碑 | 贡献者 |
|------|--------|--------|
| 1966 | DDP 算法首次提出 | Mayne, *Int. J. Control* 3(1):85-95 |
| 1970 | DDP 专著出版，完整理论 | Jacobson & Mayne, Elsevier |
| 1988 | DDP = stagewise Newton 的等价性证明 | Pantoja, *Int. J. Control* 47:1539 |
| 1991 | DDP 严格二次收敛证明 | Liao & Shoemaker, *IEEE TAC* |
| 2004 | iLQR 提出（丢弃动力学 Hessian） | Li & Todorov, ICINCO |
| 2005 | iLQG（stochastic 版本） | Todorov & Li, ACC |
| 2012 | 工程化 DDP（正则化+line search） | Tassa, Erez & Todorov, IROS |
| 2014 | Box-DDP（控制约束） | Tassa, Mansard & Todorov, ICRA |
| 2018 | 四足全身 iLQR-MPC 190Hz | Neunert et al., RA-L |
| 2020 | Crocoddyl/FDDP 框架 | Mastalli et al., ICRA |
| 2022 | Box-FDDP（控制箱 + 多射击） | Mastalli, Autonomous Robots |
| 2025 | ProxDDP 硬约束原生 | Jallet et al., IEEE T-RO |

Mayne 在 1966 年提出 DDP 时，计算机的计算能力还不足以处理高维系统。他的核心贡献不在于算法的效率，而在于**把 Bellman 方程的精确递推替换为局部二阶近似**这一思想范式的突破。随后 Jacobson 在 1970 年的专著中系统化了理论，证明了收敛性。但真正让 DDP 在机器人领域"复活"的，是 2012 年 Tassa 团队在 IROS 上发表的工程化版本——加入了正则化、line search、阻尼策略，使算法在高维非线性系统上鲁棒可用。

### 问题规模的具体感知

为了让读者对 OCP 的计算规模有直觉，给出几个典型系统：

| 系统 | $n$ | $m$ | 典型 $N$ | $\Delta t$ | 总变量 $Nm$ | 约束(多射击) $Nn$ |
|------|-----|-----|---------|-----------|------------|-----------------|
| Cart-pole 摆起 | 4 | 1 | 100 | 20 ms | 100 | 400 |
| 四旋翼激进 | 12 | 4 | 50 | 10 ms | 200 | 600 |
| 7-DoF 臂 reaching | 14 | 7 | 50 | 20 ms | 350 | 700 |
| ANYmal 四足 | 24 | 12 | 25 | 20 ms | 300 | 600 |
| Talos 人形全身 | 74 | 30 | 100 | 10 ms | 3000 | 7400 |

### ⚠️ 常见陷阱

💡 **概念误区：把 DDP 当成"一种近似 DP"**

新手想法："DDP 是 DP 的近似，所以精度不高"。

实际上：DDP 不是在近似值函数，而是**迭代求解最优控制**。在光滑问题上，DDP 的收敛点就是 PMP 的一阶必要条件满足点——这与全空间 DP 给出的解完全一致（对无约束凸问题即全局最优）。DDP 的"近似"仅在于**每次迭代内用二阶展开代替精确 Bellman 极小化**，但多次迭代后收敛到精确解。

🧠 **思维陷阱：认为"变量少=简单"**

单射击 DDP 变量只有 $Nm$ 个（仅控制），看起来比多射击的 $N(n+m)$ "简单"。但在长时域+不稳定系统中，单射击的 reduced Hessian 条件数 $\sim e^{2\lambda T}$（$\lambda$ 为不稳定模态），实际上**更难求解**。多射击（FDDP）把非线性"分配到每个节点"，每段只承担一步动力学，条件数仅 $\sim e^{2\lambda T/N}$。

💡 **概念误区：把"单射击"与"间接法"混为一谈**

单射击是指把状态消去后只对控制优化的**变量组织方式**；间接法是指通过一阶必要条件（PMP）把 OCP 转化为边值问题的**数学路径**。DDP 是单射击的但结合了间接法（backward pass = 共态积分）和直接法（forward pass = 仿真评估）的特点。

### 练习

1. 对一个 $n=4, m=1, N=100, \Delta t = 0.02s$ 的 cart-pole 摆起问题，计算 DDP 的决策变量维度。与 IPOPT 直接转录（多射击）的变量+约束数对比。
2. 若系统有不稳定模态 $\lambda = 5$（对应倒立摆），预测时域 $T = 2s$，计算单射击 reduced Hessian 的条件数下界 $e^{2\lambda T}$，并说明为什么需要正则化。
3. 从 DDP 的历史脉络中，指出 2012 年 Tassa 工作的三大工程化贡献是什么？为什么这些贡献对实机部署至关重要？

---

## §3.9.2 DDP 的完整推导——Backward Pass ⭐⭐

### 动机：从 Bellman 方程到局部二次近似

回顾 3.3 的 Bellman 最优性原理：值函数 $V_k(x)$ 满足

$$
V_k(x) = \min_u \big[\ell(x,u) + V_{k+1}(f(x,u))\big], \qquad V_N(x) = \ell_f(x).
$$

这个方程精确成立，但对非线性 $f$ 和非二次 $\ell$，我们无法解析求解 $\min_u$。DDP 的关键一步是：**沿当前名义轨迹 $(\bar x_{0:N}, \bar u_{0:N-1})$ 做二阶 Taylor 展开，把每步的 $\min_u$ 变成一个二次规划。**

这一步的深层意义在于：它把全局的 Bellman 方程"局域化"了——不再需要在整个状态空间上知道 $V(x)$，只需要在名义轨迹 $\bar x_k$ 的邻域内知道 $V$ 的二阶行为（由 $V_x, V_{xx}$ 描述）。这使得计算从维数灾难的 $O(|X|^n)$ 降为 $O(N \cdot m^3)$。

### 如果不做二阶展开会怎样（反面）

如果只做**一阶展开**（即梯度法）：$\delta u_k = -\alpha \cdot Q_u$。这给出的是最速下降方向，每步迭代代价函数下降 $O(\alpha)$。但梯度法有两个致命缺陷：

1. **没有反馈结构**：$\delta u$ 不依赖 $\delta x$，开环执行时无法补偿状态偏差
2. **收敛极慢**：线性收敛率，对 ill-conditioned 问题需要数千次迭代

如果做**三阶展开**：理论上能获得三次收敛率，但计算量暴增——需要四阶张量 $f_{xxx}$（$n\times n\times n\times n$），且无法保证正定性，子问题不再是凸的。

二阶展开（DDP/iLQR）给出的是 $\delta u = k + K\delta x$，其中 $K$ 是**反馈增益矩阵**——这正是 MPC 需要的闭环控制律，且具有二次/超线性收敛率。这是精度与计算量的最佳平衡。

### Q 函数的定义

沿名义轨迹 $(\bar x_{0:N}, \bar u_{0:N-1})$ 定义第 $k$ 步的 **action-value 函数**（也称 Q 函数）：

$$
Q_k(\delta x, \delta u) \triangleq \ell(\bar x_k + \delta x, \bar u_k + \delta u) + V_{k+1}\big(f(\bar x_k + \delta x, \bar u_k + \delta u)\big) - \text{const}
$$

这里的 $\delta x = x - \bar x_k$，$\delta u = u - \bar u_k$ 是相对于名义点的偏差。我们要对 $Q_k$ 做二阶 Taylor 展开，得到关于 $(\delta x, \delta u)$ 的二次函数，然后解析极小化。

**为什么减去常数？** 常数项 $V_k(\bar x_k)$ 不影响极小化的解 $\delta u^\star$，去掉它使得 $Q_k(0,0) = 0$，更清爽。

### Q 函数六系数的完整推导

这是本专题最核心的推导，每一步都不跳过。

**Step 1：对阶段代价 $\ell$ 做二阶 Taylor 展开**

$$
\ell(\bar x + \delta x, \bar u + \delta u) \approx \ell(\bar x, \bar u) + \ell_x^\top \delta x + \ell_u^\top \delta u + \frac{1}{2}\delta x^\top \ell_{xx} \delta x + \delta u^\top \ell_{ux} \delta x + \frac{1}{2}\delta u^\top \ell_{uu} \delta u
$$

其中 $\ell_x = \partial\ell/\partial x|_{\bar x,\bar u} \in \mathbb{R}^n$，$\ell_u = \partial\ell/\partial u|_{\bar x,\bar u} \in \mathbb{R}^m$，$\ell_{xx} = \partial^2\ell/\partial x^2 \in \mathbb{R}^{n\times n}$，$\ell_{ux} = \partial^2\ell/\partial u\partial x \in \mathbb{R}^{m\times n}$，$\ell_{uu} = \partial^2\ell/\partial u^2 \in \mathbb{R}^{m\times m}$。

**为什么 $\ell_{ux}$ 通常为零？** 对最常见的二次代价 $\ell = \frac{1}{2}(x-x_\text{ref})^\top Q(x-x_\text{ref}) + \frac{1}{2}u^\top Ru$，交叉项 $\ell_{ux} = 0$。但对**tracking + regularization**混合代价、或物理上状态与控制耦合的场景（如变传动比），$\ell_{ux} \neq 0$。为通用性我们保留它。

**Step 2：对动力学 $f$ 做 Taylor 展开**

$$
f(\bar x + \delta x, \bar u + \delta u) \approx \underbrace{f(\bar x, \bar u)}_{\bar x'} + f_x \delta x + f_u \delta u + \frac{1}{2}\begin{pmatrix}\delta x \\ \delta u\end{pmatrix}^\top \mathcal{F} \begin{pmatrix}\delta x \\ \delta u\end{pmatrix}
$$

其中 $\bar x' = f(\bar x, \bar u)$ 是名义下一状态，$f_x = \partial f/\partial x \in \mathbb{R}^{n\times n}$，$f_u = \partial f/\partial u \in \mathbb{R}^{n\times m}$。

二阶项 $\mathcal{F}$ 是一个**三阶张量**的向量化：对每个输出维度 $i = 1,\ldots,n$，有一个 $(n+m)\times(n+m)$ 的 Hessian 矩阵 $\nabla^2 f^i$。把它分块写：

$$
\nabla^2 f^i = \begin{pmatrix} f^i_{xx} & f^i_{xu} \\ f^i_{ux} & f^i_{uu} \end{pmatrix}
$$

其中 $f^i_{xx} \in \mathbb{R}^{n\times n}$，$f^i_{xu} \in \mathbb{R}^{n\times m}$，$f^i_{uu} \in \mathbb{R}^{m\times m}$。三阶张量 $f_{xx}$ 的含义是"动力学第 $i$ 个输出关于状态的二阶偏导"堆叠而成。

**Step 3：对 $V_{k+1}$ 做 Taylor 展开**

设 $\delta x' = f(\bar x + \delta x, \bar u + \delta u) - \bar x'$ 为下一状态的偏差，则：

$$
V_{k+1}(\bar x' + \delta x') \approx V_{k+1}(\bar x') + V'_x{}^\top \delta x' + \frac{1}{2}\delta x'{}^\top V'_{xx} \delta x'
$$

其中 $V'_x = \partial V_{k+1}/\partial x|_{\bar x'} \in \mathbb{R}^n$，$V'_{xx} = \partial^2 V_{k+1}/\partial x^2|_{\bar x'} \in \mathbb{R}^{n\times n}$。

**关键假设**：我们假设 $V_{k+1}$ 的二阶展开已知（由后一步的递推给出）。这是 backward recursion 的归纳假设。基情形：$V_N(x) = \ell_f(x)$，$V_{N,x} = \nabla\ell_f(\bar x_N)$，$V_{N,xx} = \nabla^2\ell_f(\bar x_N)$。

**Step 4：链式法则——把 $\delta x'$ 代入 $V_{k+1}$ 的展开**

由 Step 2，$\delta x'$ 到一阶精度为：

$$
\delta x' = f_x \delta x + f_u \delta u + \text{(二阶项)}
$$

代入 $V'_x{}^\top \delta x'$（一阶项）：

$$
V'_x{}^\top \delta x' = V'_x{}^\top (f_x \delta x + f_u \delta u) + \underbrace{V'_x{}^\top \cdot \frac{1}{2}\text{(二阶项)}}_{\text{DDP only 项的来源}}
$$

代入 $\frac{1}{2}\delta x'{}^\top V'_{xx} \delta x'$（二阶项）：

$$
\frac{1}{2}\delta x'{}^\top V'_{xx} \delta x' = \frac{1}{2}(f_x\delta x + f_u\delta u)^\top V'_{xx}(f_x\delta x + f_u\delta u) + O(3)
$$

展开得到：

$$
= \frac{1}{2}\delta x^\top f_x^\top V'_{xx} f_x\,\delta x + \delta u^\top f_u^\top V'_{xx} f_x\,\delta x + \frac{1}{2}\delta u^\top f_u^\top V'_{xx} f_u\,\delta u
$$

**Step 5：组合——按 $(\delta x, \delta u)$ 的阶次整理**

将 Step 1、Step 3-4 的所有项合并，按零阶、一阶（$\delta x$、$\delta u$）、二阶（$\delta x^2$、$\delta u\delta x$、$\delta u^2$）整理，得到 **Q 函数的六个系数**：

$$
\boxed{
\begin{aligned}
Q_x &= \ell_x + f_x^\top V'_x \\
Q_u &= \ell_u + f_u^\top V'_x \\
Q_{xx} &= \ell_{xx} + f_x^\top V'_{xx} f_x + \underbrace{\sum_{i=1}^n (V'_x)_i \cdot f^i_{xx}}_{\text{DDP only: 动力学曲率项}} \\
Q_{ux} &= \ell_{ux} + f_u^\top V'_{xx} f_x + \underbrace{\sum_{i=1}^n (V'_x)_i \cdot f^i_{ux}}_{\text{DDP only}} \\
Q_{uu} &= \ell_{uu} + f_u^\top V'_{xx} f_u + \underbrace{\sum_{i=1}^n (V'_x)_i \cdot f^i_{uu}}_{\text{DDP only}}
\end{aligned}
}
$$

### 张量缩并的完整维度说明与物理解释

"DDP only" 项涉及**三阶张量与向量的缩并**。以 $Q_{uu}$ 中的 DDP 项为例：

- $V'_x$ 是 $n$ 维向量（值函数梯度 = 共态）
- $f_{uu}$（即 $\partial^2 f^i / \partial u^j \partial u^k$）是形状为 $(n, m, m)$ 的三阶张量
- 缩并操作 $\sum_i (V'_x)_i \cdot f^i_{uu}$：对每个输出维 $i$，取权重 $(V'_x)_i$，乘以对应的 $m\times m$ Hessian $f^i_{uu}$，求和。结果是 $m\times m$ 矩阵

**物理含义**：这个张量缩并项衡量的是"**当状态偏离名义轨迹时，动力学的曲率如何被值函数梯度"加权"后影响控制方向的选择**"。具体来说：

- 值函数梯度 $V'_x$ 的分量 $(V'_x)_i$ 代表"第 $i$ 个状态维度偏离对未来总代价的影响"
- 动力学 Hessian $f^i_{uu}$ 代表"控制的二阶效应如何影响第 $i$ 个状态维度的下一步值"
- 两者的缩并代表："如果某个状态维度对未来代价很敏感（$V'_x$ 大），且控制对该维度有二阶效应（$f_{uu}$ 大），那么 $Q_{uu}$ 就需要额外修正"

> **本质洞察**：这三个张量缩并项代表了**动力学的曲率**对最优控制方向的影响。当动力学近似线性时（如短采样间隔），这些项很小，可以安全丢弃——这正是 iLQR 的数学依据。

**计算量估计**：对 $n$ 维状态、$m$ 维控制，计算 $V'_x \cdot f_{uu}$ 需要 $n$ 个 $m\times m$ 矩阵的加权求和，即 $O(nm^2)$；$V'_x \cdot f_{xx}$ 需要 $O(n^3)$；$V'_x \cdot f_{ux}$ 需要 $O(n^2m)$。对 7-DoF 臂（$n=14, m=7$），这三项总共约 $14\times 14^2 + 14\times 7^2 + 14\times 14\times 7 \approx 5000$ 次乘法——而 $f_x^\top V'_{xx} f_x$ 需要 $14^3 \approx 2700$ 次乘法。所以 DDP 项的额外开销约 2 倍。但高维人形（$n=74$）时额外开销增长为 $O(n^3/n^2m) = O(n/m)$ 倍，代价显著。

### 最优控制修正量

对 $Q$ 函数关于 $\delta u$ 求导令其为零（极小化条件）：

$$
\frac{\partial Q}{\partial (\delta u)} = Q_u + Q_{uu} \delta u + Q_{ux} \delta x = 0
$$

这是一个关于 $\delta u$ 的线性方程组（因为 $Q$ 关于 $\delta u$ 是二次的）。解得：

$$
\boxed{\delta u^\star(\delta x) = k + K\,\delta x}
$$

其中**开环修正**（feedforward）和**反馈增益**（feedback gain）分别为：

$$
k = -Q_{uu}^{-1} Q_u, \qquad K = -Q_{uu}^{-1} Q_{ux}
$$

**三个重要观察**：

1. **$k$ 的含义**：即使 $\delta x = 0$（轨迹没有偏差），也需要修正 $\delta u = k$ 来使 $Q_u$ 趋零——这是"把当前控制向最优方向推进"的开环校正。收敛时 $k \to 0$（$Q_u = 0$ 是一阶最优性条件）。

2. **$K$ 的含义**：线性反馈增益，把状态偏差 $\delta x$ 转化为控制修正。这正是 MPC 闭环控制律的来源——即使不做完整优化，只用 $K_0$ 做瞬时反馈就能稳定系统。

3. **$Q_{uu} \succ 0$ 的必要性**：只有 $Q_{uu}$ 正定时，上述极小化才有唯一最小值（而非鞍点或最大值）。若 $Q_{uu}$ 半正定或不定，$\delta u^\star$ 不存在或方向错误——此时必须正则化（§3.9.6）。

### 值函数递推（Riccati-like 后向传播）

将 $\delta u^\star = k + K\delta x$ 代回 $Q$ 函数，得到 $V_k$ 在 $\bar x_k$ 的二阶展开系数：

$$
\boxed{
\begin{aligned}
\Delta V_k &= -\frac{1}{2} k^\top Q_{uu} k \\
V_x &= Q_x + K^\top Q_{uu} k + K^\top Q_u + Q_{ux}^\top k \\
V_{xx} &= Q_{xx} + K^\top Q_{uu} K + K^\top Q_{ux} + Q_{ux}^\top K
\end{aligned}
}
$$

**这三个公式的推导**：把 $\delta u^\star = k + K\delta x$ 代入 $Q(\delta x, k + K\delta x)$：

$$
\begin{aligned}
Q(\delta x, k+K\delta x) &= Q_x^\top \delta x + Q_u^\top(k+K\delta x) \\
&\quad + \frac{1}{2}\delta x^\top Q_{xx}\delta x + (k+K\delta x)^\top Q_{ux}\delta x + \frac{1}{2}(k+K\delta x)^\top Q_{uu}(k+K\delta x)
\end{aligned}
$$

按 $\delta x$ 的零阶、一阶、二阶项分别整理：

- 零阶（$\delta x = 0$ 时）：$\Delta V = Q_u^\top k + \frac{1}{2}k^\top Q_{uu}k = -k^\top Q_{uu}k + \frac{1}{2}k^\top Q_{uu}k = -\frac{1}{2}k^\top Q_{uu}k$（用了 $Q_u = -Q_{uu}k$）
- 一阶：$V_x = Q_x + K^\top Q_u + Q_{ux}^\top k + K^\top Q_{uu}k$
- 二阶：$V_{xx} = Q_{xx} + K^\top Q_{ux} + Q_{ux}^\top K + K^\top Q_{uu}K$

利用 $k = -Q_{uu}^{-1}Q_u$ 和 $K = -Q_{uu}^{-1}Q_{ux}$ 进一步化简：

$$
V_x = Q_x - Q_{ux}^\top Q_{uu}^{-1} Q_u, \qquad V_{xx} = Q_{xx} - Q_{ux}^\top Q_{uu}^{-1} Q_{ux}
$$

注意第二个式子——这恰好是 **Schur 补** 的形式！回顾 3.5 的 TVLQR Riccati 递推：

$$
P_k = Q + A_k^\top P_{k+1} A_k - A_k^\top P_{k+1} B_k (R + B_k^\top P_{k+1} B_k)^{-1} B_k^\top P_{k+1} A_k
$$

对比可见，DDP 的 $V_{xx}$ 递推就是**非线性版的 Riccati 递推**，其中：
- $Q_{xx}$ 对应 LQR 的 $Q + A^\top P' A$（含动力学 Hessian 贡献）
- $Q_{uu}$ 对应 LQR 的 $R + B^\top P' B$
- $Q_{ux}$ 对应 LQR 的 $B^\top P' A$

**终端初始化**：$V_N = \ell_f(\bar x_N)$，$V_{N,x} = \nabla \ell_f(\bar x_N)$，$V_{N,xx} = \nabla^2 \ell_f(\bar x_N)$。

从 $k = N-1$ 递推到 $k = 0$，即完成一次 **Backward Pass**。

### $V_{xx}$ 正定性的保持（归纳证明）

**命题**：若 $\ell_f$ 凸（$V_{N,xx} = \nabla^2\ell_f \succeq 0$），且每步 $Q_{uu} \succ 0$，则 $V_{k,xx} \succeq 0$ 对所有 $k$ 成立（iLQR 情形）。

**证明**：$V_{xx} = Q_{xx} - Q_{ux}^\top Q_{uu}^{-1}Q_{ux}$ 是分块矩阵 $\begin{pmatrix}Q_{xx} & Q_{ux}^\top \\ Q_{ux} & Q_{uu}\end{pmatrix}$ 关于 $Q_{uu}$ 的 Schur 补。由 Schur 补定理，当 $Q_{uu} \succ 0$ 时，$V_{xx} \succeq 0$ 当且仅当整个分块矩阵 $\succeq 0$。对 iLQR，$Q_{xx} = \ell_{xx} + f_x^\top V'_{xx}f_x \succeq 0$（若 $\ell_{xx} \succeq 0$ 且归纳假设 $V'_{xx} \succeq 0$），配合 $Q_{uu} \succ 0$，Schur 补非负确保 $V_{xx} \succeq 0$。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：$V_{xx}$ 的对称性丢失**

在浮点运算中，反复做矩阵乘法后 $V_{xx}$ 可能丧失精确对称性（差异在 $10^{-15}$ 量级，但累积后可达 $10^{-12}$）。若 Cholesky 分解 $Q_{uu}$ 时输入了非对称的 $V_{xx}$，会导致数值错误或 Cholesky 失败。

正确做法：每步递推后强制对称化 $V_{xx} \leftarrow (V_{xx} + V_{xx}^\top)/2$。

💡 **概念误区：认为 $\Delta V$ 就是"这步的代价改进"**

$\Delta V = -\frac{1}{2}k^\top Q_{uu} k$ 是 $V_k$ 在名义点处的**零阶修正量**，不是轨迹代价的实际改进。真正的代价改进在 Forward Pass 中通过 line search 实测得到。$\sum_k \Delta V$ 只是"期望改进"的二阶估计。

⚠️ **编程陷阱：$Q_u$ 和 $Q_x$ 的维度搞错**

$Q_u \in \mathbb{R}^m$（控制维），$Q_x \in \mathbb{R}^n$（状态维）。如果代码中 $f_u$ 的形状写反（$(m,n)$ vs $(n,m)$），$Q_u = \ell_u + f_u^\top V'_x$ 的矩阵乘法会出错但不一定报 shape error（如果 $n=m$），导致极隐蔽的 bug。建议用 assert 检查所有中间量维度。

### 练习

1. 对线性系统 $f(x,u) = Ax + Bu$，验证 $f_{xx} = f_{ux} = f_{uu} = 0$，并证明此时 DDP 的 backward pass 退化为 3.5 的 Riccati 递推。
2. 对一维系统 $x_{k+1} = x_k + u_k + 0.5 x_k u_k$，$\ell = \frac{1}{2}(x^2 + u^2)$，手算 $Q_x, Q_u, Q_{xx}, Q_{ux}, Q_{uu}$（DDP 版），并写出 iLQR 版的差异。
3. 从 $V_{xx}$ 的 Schur 补形式出发，证明：若 $Q_{uu}$ 的条件数为 $\kappa$，则 $V_{xx}$ 的数值精度受 $O(\kappa \cdot \epsilon_{\text{machine}})$ 限制。这解释了为什么高正则化（大 $\mu$）反而可能产生数值不稳。

---

## §3.9.3 iLQR 作为 DDP 的 Gauss-Newton 近似 ⭐⭐

### 动机

完整 DDP 需要计算动力学的**二阶导数**（$f_{xx}, f_{ux}, f_{uu}$ 三阶张量），这在高维系统中代价极高：

- 对 7-DoF 机械臂（$n=14, m=7$）：$f_{xx}$ 有 $14 \times 14 \times 14 = 2744$ 个元素
- 对人形上身（$n=37, m=17$）：$f_{xx}$ 有 $37 \times 37 \times 37 = 50653$ 个元素
- 对全身人形（$n=74, m=30$）：$f_{xx}$ 有 $74^3 = 405224$ 个元素

计算这些张量需要对每个动力学输出的每对输入变量求二阶偏导——用有限差分需要 $O(n(n+m)^2)$ 次仿真调用。即使用 AD，内存开销也随 $n^2$ 增长。

**Li 与 Todorov（2004）的关键观察**：在大多数机器人问题中，动力学几乎是线性的（短采样间隔 + 物理系统的平滑性），张量缩并项 $V'_x \cdot f_{xx}$ 等远小于 $f_x^\top V'_{xx} f_x$ 等项。丢弃它们能大幅降低计算量，且收敛率仅从"二次"降为"超线性"。

### 严格定义

**iLQR（Iterative Linear Quadratic Regulator）丢弃全部三个张量缩并项**：

$$
\boxed{
\begin{aligned}
Q_{xx}^{\text{iLQR}} &= \ell_{xx} + f_x^\top V'_{xx} f_x \\
Q_{uu}^{\text{iLQR}} &= \ell_{uu} + f_u^\top V'_{xx} f_u \\
Q_{ux}^{\text{iLQR}} &= \ell_{ux} + f_u^\top V'_{xx} f_x
\end{aligned}
}
$$

其余（$Q_x, Q_u$）不变。这三个公式的形式与 LQR 完全相同——唯一的区别是 $f_x, f_u$ 每步都不同（因为沿非线性轨迹线性化）。

> **"不是 X 而是 Y" 澄清**：iLQR 不是"DDP 的低精度版本"，而是"DDP 的 Gauss-Newton 近似"。Gauss-Newton 在最小二乘问题中是一个非常成功的策略——在残差小时接近 Newton 精度，但避免了 Hessian 的不定性问题。

### 与 Gauss-Newton 的精确等价性

**定理（Giftthaler et al. 2018, IROS）**：将 OCP 写成非线性最小二乘形式 $\min_U \|r(U)\|^2$（其中 $r$ 是包含状态跟踪误差的残差向量），则：
- **DDP = 完整 Newton 法**：$H = J_r^\top J_r + \sum_i r_i \nabla^2 r_i$
- **iLQR = Gauss-Newton 法**：$H_{\text{GN}} = J_r^\top J_r$（丢弃 $\sum_i r_i \nabla^2 r_i$）

**为什么这个等价性很深刻？** 它揭示了：

1. DDP 的张量缩并项 $V'_x \cdot f_{xx}$ 对应非线性最小二乘中的**残差 Hessian 项** $r_i \nabla^2 r_i$
2. Gauss-Newton 丢弃残差 Hessian 的理由——"残差小时该项小"——与 iLQR 丢弃动力学 Hessian 的理由完全一致
3. Gauss-Newton 的 $J_r^\top J_r$ 自动半正定——对应 iLQR 的 $f_u^\top V'_{xx} f_u$ 自动半正定——这是 iLQR 无需正则化（通常情况下）的数学原因

> **本质洞察**：iLQR 不只是"省计算"，它同时"省麻烦"——通过牺牲从二次到超线性的收敛率降级，换来了 $Q_{uu}$ 的自动正定性和实现简洁性。这是一个非常划算的 trade-off。

### 精确推导：为什么 Gauss-Newton 项对应 $f_x^\top V'_{xx} f_x$

设 OCP 的代价可写为最小二乘形式：

$$
J = \sum_{k=0}^{N-1} \frac{1}{2}\|r_k(x_k, u_k)\|^2 + \frac{1}{2}\|r_N(x_N)\|^2
$$

其中 $r_k = \begin{pmatrix} Q^{1/2}(x_k - x_\text{ref}) \\ R^{1/2} u_k \end{pmatrix}$ 是残差向量。对整体残差 $r(U) = (r_0, r_1, \ldots, r_N)$ 构造 Jacobian $J_r = \partial r / \partial U$，则：

- **Gauss-Newton Hessian**：$H_{\text{GN}} = J_r^\top J_r$。通过 backward recursion 展开，$J_r$ 的结构是时间上的链式法则 $\prod f_x$，与 $V'_{xx}$ 的含义一致——$(J_r^\top J_r)$ 的"控制块"正是 $f_u^\top V'_{xx} f_u + \ell_{uu}$。
- **残差 Hessian 项**：$\sum_i r_i \nabla^2 r_i$。状态残差 $r_k$ 关于 $u_k$ 的依赖通过动力学链传播，其二阶项恰好包含 $V'_x \cdot f_{uu}$。

### 收敛率对比

| 方法 | 收敛率 | 条件 | 每步计算 |
|------|--------|------|---------|
| DDP | **二次**：$\|e^{k+1}\| \le C\|e^k\|^2$ | $Q_{uu} \succ 0$，$f \in C^3$ | $O(N(n^3 + n^2 m))$ + 张量 |
| iLQR | **超线性**（零残差时近二次） | 同上减 $C^3$ | $O(N(n^2 m + nm^2))$ |

**"超线性"精确含义**：$\|e^{k+1}\| / \|e^k\| \to 0$ 但不如 $C\|e^k\|$ 那么快。对零残差问题（如从静止摆起到直立），iLQR 实际表现接近二次收敛——因为 $V'_x \to 0$（收敛时值函数梯度趋零），使得被丢弃的 $V'_x \cdot f_{xx} \to 0$。

**实际工程体验**：iLQR 每次迭代比 DDP 快 3-5 倍（省去张量计算），但需要 1.5-2 倍迭代数。**总耗时大致相当**。对光滑问题 DDP 略胜；对接触/非光滑问题 iLQR 反而更稳（因为 $f_{xx}$ 在接触切换处噪声大、可能使 $Q_{uu}$ 不定）。

### $Q_{uu}^{\text{iLQR}}$ 的正定性保证

**命题**：若 $\ell_{uu} \succ 0$（即控制代价 $R \succ 0$）且 $V'_{xx} \succeq 0$（由归纳假设保证），则 $Q_{uu}^{\text{iLQR}} = \ell_{uu} + f_u^\top V'_{xx} f_u \succ 0$。

**证明**：$f_u^\top V'_{xx} f_u$ 是半正定矩阵 $V'_{xx}$ 的合同变换 $B^\top A B$（$B = f_u$），结果仍半正定。加上正定的 $\ell_{uu}$ 得到正定。

这是 iLQR 的一大工程优势：**只要 $R \succ 0$，$Q_{uu}$ 自动正定，无需正则化**（除非 $V'_{xx}$ 被数值误差污染）。相比之下，完整 DDP 的 $Q_{uu}$ 含有 $V'_x \cdot f_{uu}$ 项，该项可正可负（取决于动力学曲率与值函数梯度方向），可能破坏正定性。

从优化理论的角度看，$Q_{uu}^{\text{iLQR}} \succ 0$ 保证了每步 backward pass 的子问题是**严格凸的**——唯一最小值存在且可通过 Cholesky 分解稳定求得。这消除了 DDP 中最常见的数值故障来源（Cholesky 失败），使得 iLQR 在实际部署中几乎不需要正则化——一个显著降低了工程复杂度的优势。

### 什么时候必须用完整 DDP（不能用 iLQR）

1. **高精度需求 + 动力学强非线性**：如卫星再入大气层（$f_{xx}$ 很大）、柔性机器人（弹性非线性）
2. **长采样间隔**：$\Delta t > 50$ ms 时线性化误差大，iLQR 的 GN 近似失效
3. **学术验证**：证明算法收敛率时需要区分二次与超线性

在绝大多数机器人实时控制场景（$\Delta t = 5$-$20$ ms、刚性关节）中，**iLQR 是唯一正确的选择**。

### ⚠️ 常见陷阱

💡 **概念误区：混淆 DDP 与 iLQR**

很多论文和代码库把 iLQR 标注为 "DDP"。严格来说：
- **DDP**：保留 $f_{xx}, f_{ux}, f_{uu}$ 张量缩并 → Newton 法 → 二次收敛
- **iLQR**：丢弃全部张量缩并 → Gauss-Newton → 超线性收敛

除非你使用 Crocoddyl 的 `SolverDDP`（真 DDP）或手写了 dynamics Hessian，绝大多数实现实际上是 iLQR。Tedrake 的建议：除非有很强理由，一律用 iLQR。

🧠 **思维陷阱：认为"二次收敛一定比超线性好"**

二次收敛 $\|e^{k+1}\| \le C\|e^k\|^2$ 中的常数 $C$ 可能很大！如果 DDP 的张量计算引入数值噪声（接触场景），$C$ 会爆炸，导致实际收敛不如 iLQR。收敛率是渐近概念——它描述的是"离最优点足够近时"的行为，不保证全局性能。

### 练习

1. 对 $n=14, m=7$ 的 7-DoF 机械臂，估算 DDP 的张量缩并项 $V'_x \cdot f_{uu}$ 的计算量（flops），与 iLQR 的 $f_u^\top V'_{xx} f_u$ 对比。
2. 设计一个实验：对同一 cart-pole 摆起问题，分别跑 DDP 和 iLQR，记录迭代次数和总 wall-clock time。在什么情况下 DDP 总耗时更少？
3. 解释为什么 iLQR 在残差不为零时（如 $x_\text{ref}$ 不可达）收敛会变慢。给出一个具体的数值例子。

---

## §3.9.4 与 PMP/DP/LQR 的精确关系——统一视角 ⭐⭐⭐

### 动机

DDP/iLQR 不是一个孤立的算法，而是 PMP、DP、LQR 三条理论线的**精确交汇点**。理解这三条关系，不仅能加深对算法的洞察，更能在面对新问题时灵活选择合适的方法。

### 关系 1：DDP 与 LQR（→ 3.5 TVLQR）

**定理（Mayne 1966）**：若 $\ell$ 是二次、$f$ 是线性，则 DDP 一次迭代即收敛到全局最优。

**证明**：线性 $f$ 意味着 $f_{xx} = f_{ux} = f_{uu} = 0$，故 DDP = iLQR。二次 $\ell$ 加线性 $f$ 意味着 $Q$ 函数的 Taylor 展开**精确**（无高阶余项），backward pass 的递推是 3.5 的差分 Riccati 方程：

$$
P_k = Q + A^\top P_{k+1} A - A^\top P_{k+1} B(R + B^\top P_{k+1} B)^{-1} B^\top P_{k+1} A
$$

对应关系：$V_{xx} \leftrightarrow P_k$，$Q_{uu} \leftrightarrow R + B^\top P_{k+1} B$，$K \leftrightarrow -(R + B^\top P' B)^{-1} B^\top P' A$。

> **本质洞察**：DDP 可以理解为"在非线性问题上反复做 LQR"。每次 backward pass 相当于沿当前名义轨迹建立一个 TVLQR 问题并解之；forward pass 用新的控制律重新仿真得到新名义轨迹。迭代直到名义轨迹不再变化（收敛）。

这个理解方式的实践价值：如果你需要调 DDP 的超参数（$Q, R, Q_f$），可以先在最终名义轨迹的线性化上调 LQR 参数使其满意，然后直接用于 DDP——因为收敛后两者完全等价。

### 关系 2：DDP 与 PMP（→ 3.2 共态方程）

定义**共态**（costate）$\lambda_k \triangleq V_{k,x}(\bar x_k)$。回忆离散 Hamiltonian：

$$
H_k(x, u, \lambda) = \ell(x, u) + \lambda^\top f(x, u)
$$

则：
- $Q_x = \ell_x + f_x^\top V'_x = \ell_x + f_x^\top \lambda_{k+1} = \frac{\partial H_k}{\partial x}$
- $Q_u = \ell_u + f_u^\top V'_x = \ell_u + f_u^\top \lambda_{k+1} = \frac{\partial H_k}{\partial u}$

backward pass 递推 $V_x$ 即在积分**共态方程**：

$$
\lambda_k = V_{k,x} = Q_x + K^\top Q_u = \ell_x + f_x^\top \lambda_{k+1} + \text{(feedback correction)}
$$

**DDP 收敛时 $Q_u = 0$**，这正是 PMP 的一阶最优性条件 $\partial H / \partial u = 0$。

> **双重解读**：
> - 角度 1（DP 视角）：DDP 是 Bellman 方程的迭代局部二次近似
> - 角度 2（PMP 视角）：DDP 是共态方程的 Newton 法求解

反馈增益 $K_k = -Q_{uu}^{-1} Q_{ux}$ 是**共态反馈的矩阵实现**：它把"如果状态偏移了 $\delta x$，共态应该如何调整"翻译成"控制应该如何修正"。

**Pantoja 1988 的深层结果**：DDP 的 backward pass 等价于对离散 PMP 的 KKT 系统做 **stagewise Newton**。具体来说，KKT 系统是：

$$
\begin{cases}
\nabla_u H_k = 0 & \text{（最优性）} \\
x_{k+1} = f(x_k, u_k) & \text{（状态方程）} \\
\lambda_k = \nabla_x H_k = \ell_x + f_x^\top \lambda_{k+1} & \text{（共态方程）}
\end{cases}
$$

对这个 $3Nn$ 维系统做 Newton 法求解，利用时间结构分解为 $N$ 步递推——正是 DDP backward pass。

### 关系 3：DDP 与 DP（→ 3.3 Bellman 方程）

HJB/DP 给出全空间、全时刻的值函数——维数灾难。DDP 放弃全空间，**只在名义轨迹邻域做二阶 Taylor 展开**。几何图像：

- DP：在整个状态空间 $\mathbb{R}^n$ 上精确求解值函数面 $V(x)$
- DDP：只在名义轨迹 $\bar x_k$ 处拟合值函数的**局部二次抛物面** $V_k(\bar x_k) + V_x^\top \delta x + \frac{1}{2}\delta x^\top V_{xx} \delta x$

这把 HJB PDE 变成一族耦合的 **Riccati 方程**（ODE/差分方程），避开维数灾难。代价是：只得到**局部最优**（需要好的初始猜测）。

类比：DP 像用 CT（计算机断层扫描）把整个山谷的三维结构完全重建出来；DDP 像在登山时只用激光测距仪扫描脚下 3 米范围的地形——信息有限但足以决定下一步方向。每走一步就重新扫描，最终能到达山谷最低点（局部）。

从信息论角度看，DP 存储了完整的全局值函数 $V(x)$（需要 $O(|X|^n)$ 空间），而 DDP 只存储了值函数在一条轨迹上的二阶展开系数 $(V_x, V_{xx})$（需要 $O(N \cdot n^2)$ 空间）。这是从**全局信息**到**局部信息**的压缩，压缩比为 $|X|^n / (N \cdot n^2)$——对 $n = 14$ 的机械臂，这个比值是天文数字级的。DDP 用极小的内存换来了计算的可行性，代价是失去了全局最优保证。

> **反事实推理**：如果能廉价存储完整的 $V(x)$（如用神经网络近似），是否能兼得 DP 的全局性和 DDP 的效率？这正是 **Fitted Value Iteration** 和 **Deep RL** 的研究方向——用函数逼近器（如 MLP）表示 $V$，但面临的是逼近误差累积和泛化失败的问题。从这个角度看，DDP 可以被视为"用二阶多项式做 fitted value iteration，但只在一条轨迹上拟合"。

### 三条线的统一表

| 维度 | PMP 视角 | DP 视角 | LQR 视角 |
|------|---------|---------|---------|
| $V_x$ | 共态 $\lambda$ | 值函数梯度 | $Px$（线性情形） |
| $V_{xx}$ | 共态灵敏度 $\partial\lambda/\partial x$ | 值函数 Hessian | Riccati 矩阵 $P$ |
| $K$ | 共态反馈增益 | 值函数曲率→控制 | LQR 增益 |
| $Q_u = 0$ | PMP 最优性条件 | Bellman 极小化 FOC | $\nabla_u H = 0$ |
| backward pass | 积分共态 ODE | 值迭代的二阶版 | Riccati 递推 |
| forward pass | 用反馈律仿真 | 策略改进 | 闭环 rollout |

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为"DDP 是间接法"**

DDP 的分类实际上是**混合的**：
- Backward pass 是**间接法**风格（等价于积分共态+灵敏度矩阵，PMP 味道）
- Forward pass 是**直接法**风格（仿真 + line search，直接评估 $J$）

这种混合结构使 DDP 兼具两者优点：间接法的二阶收敛速度 + 直接法的全局搜索能力（通过 line search）。

### 练习

1. （跨章综合）设 $\lambda_{k+1} = V_{k+1,x}(\bar x_{k+1})$。证明 backward pass 的 $V_x$ 递推在 $\delta u = 0$ 时恰是离散 PMP 的共态方程 $\lambda_k = \ell_x + f_x^\top \lambda_{k+1}$。
2. 从 DP 视角解释：为什么 DDP 的 forward pass 中 $K$ 反馈项不随 $\alpha$ 缩放？（提示：$K$ 编码的是值函数曲率信息，与步长无关。）
3. 用 Pantoja 1988 的视角解释：为什么 DDP 的计算复杂度是 $O(N)$ 而不是 $O(N^3)$？（提示：KKT 系统的 banded 结构。）

---

## §3.9.5 Forward Pass 与 Line Search ⭐⭐

### 动机

Backward pass 给出了局部二次模型下的最优修正 $(k_k, K_k)$，但这只是"方向"——我们还需要确定"步长"。直接取 $\alpha = 1$（full step）在远离最优时可能导致代价增加甚至数值爆炸。Line search 是 DDP 从"玩具算法"变为"真机可用"的关键。

类比：backward pass 像规划了一条"理论最优路径"，但因为地形模型（二次近似）与真实地形有偏差，如果全速前进可能掉进沟里。Line search 是"先走一小步确认路是对的，再加速"。

### Forward Pass 公式

给定步长 $\alpha \in (0, 1]$，新轨迹为：

$$
\hat x_0 = x_0, \qquad \hat u_k = \bar u_k + \alpha k_k + K_k(\hat x_k - \bar x_k), \qquad \hat x_{k+1} = f(\hat x_k, \hat u_k)
$$

**关键设计决策**：$K$ 反馈项**不随 $\alpha$ 缩放**。

这不是随意选择。原因有三：

1. **闭环稳定性**：$K$ 编码了值函数的二阶信息（曲率），在任何步长下都应该保持轨迹跟踪能力
2. **数学正确性**：$\delta u = \alpha k + K \delta x$ 中，$\delta x$ 本身已经是 $O(\alpha)$ 量级（因为 $\hat x - \bar x \propto \alpha$），所以 $K \delta x$ 本身就是 $O(\alpha)$ 的
3. **非线性隐式处理**：小 $\alpha$ 时 $k$ 项主导（开环修正），大 $\alpha$ 时 $K$ 项发挥闭环跟踪作用——这使得 DDP 在大步长下仍保持轨迹稳定

### 期望下降量的完整推导

沿步长 $\alpha$ 的代价改进可以用二阶模型预测。设 $J(\alpha)$ 为步长 $\alpha$ 下的代价，则其关于 $\alpha$ 的二阶 Taylor 展开为：

$$
J(\alpha) - J(0) \approx \alpha \underbrace{\sum_{k=0}^{N-1} k_k^\top Q_{u,k}}_{\Delta_1} + \frac{\alpha^2}{2} \underbrace{\sum_{k=0}^{N-1} k_k^\top Q_{uu,k} k_k}_{\Delta_2}
$$

**推导细节**：

- $\Delta_1 = \sum_k k_k^\top Q_{u,k}$：这是代价对 $\alpha$ 的一阶导数在 $\alpha=0$ 处的值。由于 $k_k = -Q_{uu,k}^{-1}Q_{u,k}$，有 $k_k^\top Q_{u,k} = -Q_{u,k}^\top Q_{uu,k}^{-1} Q_{u,k} < 0$（负定二次型）。所以 $\Delta_1 < 0$——**方向确实是下降的**。
- $\Delta_2 = \sum_k k_k^\top Q_{uu,k} k_k > 0$（因为 $Q_{uu} \succ 0$）：这是二阶修正项，使得预测改进在 $\alpha = 1$ 时为 $\Delta_1 + \Delta_2/2$。

期望代价下降：$\Delta J(\alpha) = \alpha\Delta_1 + \frac{\alpha^2}{2}\Delta_2$。

### Armijo 准则（Tassa 2012 修正版）

定义**实际 vs 预测下降比**：

$$
z = \frac{J(\bar u) - J(\hat u)}{-\Delta J(\alpha)}
$$

接受 $\alpha$ 当且仅当：$c_1 < z < c_2$

典型参数：$c_1 = 0.1$（确保有足够实际下降），$c_2 = 10$（防止预测与实际偏差太大，表明二次模型已不可信）。

**Backtracking 序列**：$\alpha \in \{1, 0.5, 0.25, \ldots, 2^{-10}\}$。当所有 $\alpha$ 都失败 → 增大正则化 $\mu$ 重新 backward。

### 为什么不能只检查 $J$ 下降

如果只要求 $J(\hat u) < J(\bar u)$（无 Armijo 比率检查），可能接受一个"微小但浪费"的步长——改进极小，迭代极慢。Armijo 准则保证每步改进与预测一致，防止算法退化为"一阶梯度法的速度+二阶方法的计算量"。

更严格地说，不检查 $z < c_2$ 可能在二次模型严重偏离实际时接受一个"侥幸下降"的步——下一步突然代价暴增。$c_2 = 10$ 的上界防止了这种情况。

### 全步 $\alpha = 1$ 的特殊含义

当 $\alpha = 1$ 被接受时，意味着二次模型与真实代价面非常吻合——此时 DDP 处于"纯 Newton"模式，享有二次收敛率。实际上，收敛后期 $\alpha = 1$ 几乎总是被接受（这是二阶方法的标志）。

**如果 $\alpha = 1$ 在前几次迭代就被接受**：可能表明 (a) 初始猜测已经很好，或 (b) 正则化 $\mu$ 过大使步方向退化为梯度方向（小幅改进总能被接受）。需要监控 $\Delta_1$ 的绝对值来区分。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Forward pass 反馈项随 $\alpha$ 缩放**

错误写法：$\hat u = \bar u + \alpha(k + K\Delta x)$

正确写法：$\hat u = \bar u + \alpha k + K\Delta x$

前者在 $\alpha \to 0$ 时反馈也消失，闭环退化为开环——小步长时轨迹无人"看护"，可能漂移。后者保证任何步长下都有闭环稳定化。

⚠️ **编程陷阱：期望下降量只算一阶项**

若 $\Delta J$ 只取 $\alpha \Delta_1$（忽略 $\alpha^2 \Delta_2 / 2$），则 $z$ 的分母在 $\alpha = 1$ 时偏大（因为真正的期望下降应含二阶修正），导致 $z$ 偏小，频繁拒绝好的步长。必须包含二阶项。

💡 **概念误区：认为 line search 失败是"灾难"**

Line search 失败（所有 $\alpha$ 都不满足 Armijo）是正常情况——它意味着二次模型在当前点失效（如接近约束边界、非线性极强处）。正确响应是增大 $\mu$ 缩小 trust region，而非报错退出。

### 练习

1. 证明：当 $Q_{uu} \succ 0$ 时，$\Delta_1 = \sum k^\top Q_u < 0$（即方向确实是下降的）。
2. 设计一个实验：对 cart-pole 问题，分别用 (a) 只检查 $J$ 下降、(b) Armijo $c_1 = 0.1$ 两种 line search，记录迭代次数。哪种更快收敛？
3. 计算：对 $\alpha = 0.5$ 时，如果实际下降是预测的 0.8 倍（$z = 0.8$），这个步是否被接受？解释 $z < 1$ 的物理含义。

---

## §3.9.6 正则化技巧——从 Levenberg-Marquardt 到 Trust Region ⭐⭐

### 动机

当 $Q_{uu}$ 非正定时（如接近奇异姿态、$V'_{xx}$ 丢失正定性、或 DDP 的张量项贡献负特征值），Cholesky 分解失败，backward pass 崩溃。正则化的作用是**保证 $Q_{uu} \succ 0$**，同时尽量不破坏搜索方向的质量。

正则化不是"打补丁"——它有深刻的优化理论含义。在 Levenberg-Marquardt / Trust-Region 框架下，正则化参数 $\mu$ 等价于 trust-region 半径的 Lagrange 对偶变量：$\mu$ 大 = 信任区域小 = 保守步（接近梯度方向）；$\mu$ 小 = 信任区域大 = 大胆步（接近 Newton 方向）。

### 方案 A：控制空间正则化（经典 Jacobson-Mayne 版）

$$
\tilde Q_{uu} = Q_{uu} + \mu I_m
$$

**数学含义**：等价于在子问题中加约束 $\|\delta u\|^2 \le \Delta^2$（$\mu$ 是 KKT 乘子）。或者等价于在代价中加惩罚 $\frac{\mu}{2}\|\delta u\|^2$——惩罚控制修正量的大小。

**优点**：实现简单，直接加对角项。

**缺点**：
- $\mu \to \infty$ 时 $K = -\tilde Q_{uu}^{-1} Q_{ux} \to 0$：反馈消失，算法退化为开环梯度法
- 不同时刻的扰动惩罚耦合 $f_u$ 方向，效果非各向同性
- 物理上：惩罚的是"控制修正量"，但真正应该惩罚的是"状态偏离名义轨迹的量"

为什么 $K \to 0$ 是灾难性的？在 humanoid 站立这样的不稳定系统中，失去反馈增益意味着 forward pass 的轨迹会迅速发散（$e^{\lambda T}$ 指数增长），即使 $\alpha$ 很小也救不回来。

### 方案 B：状态空间正则化（Tassa 2012 改进）

把 $V'_{xx}$ 替换为 $V'_{xx} + \mu I_n$：

$$
\tilde Q_{uu} = \ell_{uu} + f_u^\top (V'_{xx} + \mu I_n) f_u, \qquad \tilde Q_{ux} = \ell_{ux} + f_u^\top (V'_{xx} + \mu I_n) f_x
$$

**数学含义**：等价于在代价中加惩罚 $\frac{\mu}{2}\|\delta x_{k+1}\|^2$——惩罚下一步状态偏离名义值的量。这正是 **trust-region 在状态空间** 的含义。

**优点**：
- $\mu \to \infty$ 时 $K$ 不消失——$K \to -(f_u^\top f_u)^{-1}f_u^\top f_x$（伪逆）。反馈把新轨迹拉向旧轨迹
- 等价于在**上一步状态轨迹周围**加二次罚 $\frac{\mu}{2}\|\delta x_{k+1}\|^2$
- 对量纲更鲁棒（$\mu$ 的单位是"状态曲率"，与系统尺度无关）

**与 Levenberg-Marquardt 的精确等价性**：

考虑子问题 $\min_{\delta u} Q(\delta x, \delta u)$ s.t. $\|\delta x'\|^2 \le \Delta^2$（trust-region on next state）。其 Lagrangian：

$$
\mathcal{L} = \frac{1}{2}\delta u^\top Q_{uu}\delta u + Q_u^\top\delta u + \mu(\|f_x\delta x + f_u\delta u\|^2 - \Delta^2)
$$

对 $\delta u$ 求导令零：$(Q_{uu} + \mu f_u^\top f_u)\delta u = -(Q_u + \mu f_u^\top f_x\delta x)$

这恰好是 Tassa 正则化后的 backward pass 方程！所以：

**Tassa 状态正则化 $\equiv$ Levenberg-Marquardt 的 trust-region 形式**

$\mu$ 是 trust-region 约束的 Lagrange 乘子；自适应 $\mu$ 调度等价于 LM 的 trust-region 半径自适应。

### $\mu$ 自适应调度（Tassa 2012 二次调度）

参数：$\mu_{\min} = 10^{-6}$，$\Delta_0 = 2$（调度因子）。

**失败时**（Cholesky 失败或 line search 全部失败）：
$$
\Delta \leftarrow \max(\Delta_0, \Delta \cdot \Delta_0), \qquad \mu \leftarrow \max(\mu_{\min}, \mu \cdot \Delta)
$$

**成功时**：
$$
\Delta \leftarrow \min(1/\Delta_0, \Delta / \Delta_0), \qquad \mu \leftarrow \mu \cdot \Delta \quad (\text{若} < \mu_{\min} \text{则清零})
$$

直觉：失败 → 加大正则化（$\mu$ 倍增式上升）；成功 → 尝试减小正则化（$\mu$ 指数下降），让算法回到纯 Newton/GN 步。

**为什么用指数而非线性调度？** 因为 $Q_{uu}$ 的特征值可能跨越多个数量级（$10^{-3}$ 到 $10^3$），$\mu$ 需要能快速跨越这个范围。线性调度（$\mu \pm c$）在 $Q_{uu}$ 条件数为 $10^6$ 时需要百万步才能从 $\mu = 10^{-6}$ 调到 $\mu = 1$。

### 混合方案：状态 + 最小控制正则化

实际工程中，最安全的做法是两者结合：

$$
\tilde Q_{uu} = \ell_{uu} + f_u^\top(V'_{xx} + \mu I_n)f_u + \epsilon I_m
$$

其中 $\epsilon = 10^{-8}$ 是极小的控制正则化，防止 $\ell_{uu} = 0$ 且 $f_u^\top(V'_{xx}+\mu I)f_u$ 恰好奇异的病态情形。

### 三类正则化策略的系统比较

工程上常遇到的正则化策略不止两种。下面给出三类策略的完整对比，帮助读者在实际系统中做出明智选择。

| 维度 | 控制正则化 $\mu I_m$ | 状态正则化 $\mu f_u^\top f_u$ | 混合正则化（Tassa + $\epsilon$） |
|------|---------------------|------------------------------|-------------------------------|
| $\mu \to \infty$ 时 $K$ | $\to 0$（开环） | 保持非零（轨迹跟踪） | 保持非零 |
| Trust-region 语义 | 惩罚 $\|\delta u\|$ | 惩罚 $\|\delta x'\|$ | 状态 TR + 最小控制保护 |
| 对 humanoid 奇异姿态 | 差（反馈丢失） | 好（保持闭环） | 最鲁棒 |
| 实现复杂度 | 最简 | 稍复杂（需 $f_u$） | 中等 |
| 推荐场景 | 教学/低维 | 工程/高维 | 实机部署 |
| LM/TR 等价 | 控制 TR | 状态 TR | 组合 TR |
| $\text{cond}(\tilde Q_{uu})$ 对 $\mu$ 的敏感度 | 高（各向异性） | 低（自然对齐 $f_u$） | 最低 |

**为什么混合方案在实机中更常见？** 纯状态正则化 $\tilde Q_{uu} = \ell_{uu} + f_u^\top(V'_{xx}+\mu I)f_u$ 虽然理论性质最好，但存在一个微妙的边界情形：当 $f_u$ 的某些奇异值极小（如冗余自由度方向），$f_u^\top(\cdot)f_u$ 在该方向上的正则化效果几乎为零，Cholesky 仍可能失败。混合方案加的 $\epsilon I_m$（$\epsilon = 10^{-8}$）恰好覆盖了这个漏洞——它以极小的代价换取数值上的完全安全。

### 正则化对收敛轨迹的影响

正则化改变的不仅是当前步的搜索方向，还影响了整个迭代的收敛路径。从优化景观的角度看：

- **低 $\mu$（接近纯 Newton）**：搜索方向由 $Q_{uu}$ 的曲率决定，步幅大、沿最陡方向前进。在凸区域效率最高（二次收敛），但在非凸区域可能跳过好的局部最优。
- **中 $\mu$（Levenberg-Marquardt 区间）**：搜索方向是 Newton 方向与梯度方向的插值。这在从远处接近最优解时最有用——既保留了二阶信息，又不至于在曲率为负的方向上走错。
- **高 $\mu$（接近梯度法）**：搜索方向退化为梯度方向 $-Q_u$。虽然每步改进小（线性收敛），但方向始终是下降的——全局收敛性最强。

> **反事实推理**：如果不做正则化调度（固定 $\mu$）会怎样？$\mu$ 太小→第一步 Cholesky 就失败；$\mu$ 太大→100 次迭代还在做梯度法。自适应调度让算法在"安全区"用小 $\mu$ 享受 Newton 速度，在"危险区"用大 $\mu$ 保证全局收敛——这是 trust-region 方法的精髓。

### 对比表（原始简版）

| 维度 | 控制正则化 $\mu I_m$ | 状态正则化 $\mu f_u^\top f_u$ |
|------|---------------------|------------------------------|
| $\mu \to \infty$ 时 $K$ | $\to 0$（开环） | 保持非零（轨迹跟踪） |
| Trust-region 语义 | 惩罚 $\|\delta u\|$ | 惩罚 $\|\delta x'\|$ |
| 对 humanoid 奇异姿态 | 差（反馈丢失） | 好（保持闭环） |
| 实现复杂度 | 最简 | 稍复杂（需 $f_u$） |
| 推荐场景 | 教学/低维 | 工程/高维 |
| LM/TR 等价 | 控制 TR | 状态 TR |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：$Q_{uu}$ 非 PD 不处理直接求逆 → NaN**

在不加正则化的情况下对 $Q_{uu}$ 做 `np.linalg.inv` 或 Cholesky，若 $Q_{uu}$ 有负特征值，会得到垃圾值或 NaN。整个 backward pass 被污染。

正确做法：先尝试 Cholesky 分解；若失败，增大 $\mu$ 并重启 backward pass。永远不要用 `inv`，用 `cho_solve`。

💡 **概念误区：认为"正则化越大越安全"**

$\mu$ 过大会使步方向退化为梯度方向（一阶），丧失 Newton 的二阶优势。极端情况下，$\mu = 10^6$ 的 DDP 还不如一阶梯度法快——因为 backward pass 的计算量白白付出了。

⚠️ **编程陷阱：$\mu$ 调度的初始值过大**

如果初始 $\mu = 100$（远大于 $Q_{uu}$ 的特征值 $\sim 1$），前几次迭代全是梯度方向，浪费计算。应从 $\mu = 10^{-6}$ 或 $\mu = 0$ 开始，只在失败时增大。

### 练习

1. 推导 Tassa 的状态空间正则化如何等价于对 $\|\delta x_{k+1}\|^2$ 的 trust-region 约束（提示：写出带约束的 QP 的 KKT 条件）。
2. 为什么 $\mu$ 调度用**指数**而非线性递增？（提示：考虑 $Q_{uu}$ 的特征值可能跨越多个数量级。）
3. 对一个数值例子：$Q_{uu} = \text{diag}(0.01, 100)$，分别用控制正则化 $\mu = 1$ 和状态正则化 $\mu = 1$ 计算修正后的条件数。哪种更均匀？

---

## §3.9.7 连续时间 DDP 与采样时间选择 ⭐⭐

### 连续时间 vs 离散时间

**连续时间 DDP**（Jacobson-Mayne 1970 原著）推导的是**微分 Riccati 方程**：

$$
-\dot P = A^\top P + PA - PBR^{-1}B^\top P + Q
$$

值函数 $V_x(t), V_{xx}(t)$ 由连续 ODE 后向积分；前向 pass 是连续 ODE 仿真。

**离散时间 DDP**（Mayne 1966）用差分 Riccati，工程更直接：不需要 ODE 积分器，直接递推矩阵序列。

现代实践几乎**全用离散时间**——原因是数字控制器本身是离散的，且离散公式与 Pinocchio 等刚体动力学库的接口更自然。OCS2 是例外——它在连续时间做 SLQ，用 ODE45 积分 Riccati ODE。

### 从连续到离散的精确关系

设连续系统 $\dot x = f_c(x, u)$，用 Euler 离散化：

$$
x_{k+1} = x_k + f_c(x_k, u_k)\Delta t
$$

则离散 Jacobian：$f_x = I + \frac{\partial f_c}{\partial x}\Delta t$，$f_u = \frac{\partial f_c}{\partial u}\Delta t$。

带入 iLQR 的 $Q_{uu}^{\text{iLQR}}$：

$$
Q_{uu} = \ell_{uu} + f_u^\top V'_{xx} f_u = R\Delta t + (\frac{\partial f_c}{\partial u}\Delta t)^\top V'_{xx}(\frac{\partial f_c}{\partial u}\Delta t)
$$

当 $\Delta t \to 0$：$Q_{uu} \to R\Delta t$，$Q_{uu}^{-1} \to (R\Delta t)^{-1}$——反馈增益 $K \sim 1/\Delta t$ 发散！这不是 bug 而是物理正确的：连续反馈增益是 $K_c = R^{-1}B^\top P$（有限），离散反馈 $K_d \approx K_c / \Delta t$。

### 采样时间 $\Delta t$ 的选择

$\Delta t$ 是 DDP 的隐含超参数，影响深远：

| 因素 | $\Delta t$ 太大 | $\Delta t$ 太小 |
|------|----------------|----------------|
| 离散化误差 | 大（iLQR 近似失效） | 小 |
| 接触检测 | 跨越接触事件 | 正常 |
| $N$ = $T/\Delta t$ | 小（视域覆盖不足） | 大（计算量线性膨胀 $O(N)$） |
| $f_x$ 条件数 | 远离 $I$（强非线性效应） | 接近 $I + A\Delta t$（温和） |
| DDP-only 项 | 大（动力学曲率显著） | 小（iLQR 近似佳） |

**经验值**：

| 应用 | $\Delta t$ | $N$ | $T$ |
|------|-----------|-----|-----|
| 操作臂 reaching | 10-50 ms | 20-100 | 1-2 s |
| 四足/人形步态 | 5-20 ms | 25-50 | 0.5-1 s |
| 四旋翼激进机动 | 2-10 ms | 50-200 | 0.5-1 s |
| 自动驾驶 | 50-100 ms | 20-40 | 2-4 s |

**变步长 DDP**：在接触切换时加密网格（$\Delta t = 1$ ms）、自由飞行段加粗（$\Delta t = 20$ ms），可节省 50%+ 计算量。Crocoddyl 通过 `IntegratedActionModel` 支持每节点独立 $\Delta t$。

### RK4 vs Euler 离散化

| 积分器 | 阶 | 每步动力学调用 | 离散化误差 | 推荐 |
|--------|---|------------|-----------|------|
| 前向 Euler | 1 | 1 | $O(\Delta t^2)$ | 原型，$\Delta t < 5$ ms |
| 半隐式 Euler | 1 | 1 | $O(\Delta t^2)$，保辛 | 接触场景 |
| RK4 | 4 | 4 | $O(\Delta t^5)$ | 精度需求高 |
| 隐式中点 | 2 | 迭代 | $O(\Delta t^3)$，保辛 | 长时域 + 保能量 |

对 iLQR 而言，**Euler + 短 $\Delta t$ 通常优于 RK4 + 长 $\Delta t$**——因为 Euler 的 $f_x = I + A\Delta t$ 导数计算最简单，且短 $\Delta t$ 使线性化误差本身很小。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：连续 vs 离散代价权重混淆**

若连续代价为 $\int_0^T \frac{1}{2}x^\top Q x\,dt$，离散化后阶段代价应是 $\frac{1}{2}x_k^\top (Q\Delta t) x_k$。忘乘 $\Delta t$ 会导致权重随 $\Delta t$ 变化——改变采样率时"最优轨迹"莫名其妙地变了。

终端代价**不乘** $\Delta t$——它是"瞬时"评估，不依赖时间步长。

💡 **概念误区：认为"$\Delta t$ 越小精度越高一定更好"**

$\Delta t$ 过小使 $N = T/\Delta t$ 暴增，不仅增加计算量，还可能引入**累积舍入误差**（$N$ 次矩阵乘法的误差以 $\sqrt{N}$ 增长）。对 $T = 2$ s，$\Delta t = 0.1$ ms 给出 $N = 20000$——单次 backward pass 需要 20000 步递推，数值误差可能达到 $10^{-8} \times \sqrt{20000} \approx 10^{-6}$。

### 练习

1. 对 cart-pole 系统，分别用 $\Delta t = 0.01, 0.02, 0.05$ s 跑 iLQR，记录收敛所需迭代次数和轨迹质量。哪个 $\Delta t$ 给出最佳 trade-off？
2. 解释为什么接触密集的腿足问题需要更小的 $\Delta t$（提示：接触力在碰撞瞬间是 $\delta$ 函数量级）。
3. 推导 RK4 离散化后 $f_x$ 的表达式（关于连续 $A = \partial f_c/\partial x$），并说明它包含哪些 $\Delta t$ 的次方项。

---

## §3.9.8 DDP/iLQR 完整实现 ⭐⭐

### 主循环伪代码

```
function iLQR(x0, U_init, Kmax, εJ, εg)
  输入: 初始状态 x0, 初始控制序列 U = [u0,...,uN-1], 最大迭代 Kmax
  初始化: μ=1.0, Δ=1.0, 前向仿真得 X = rollout(x0, U), J_old = cost(X, U)
  
  for iter = 1..Kmax:
    ═══ Step 1: 计算动力学与代价导数 ═══
    for k = 0..N-1:
      fx[k], fu[k] ← dynamics_derivatives(X[k], U[k])    // Pinocchio 或 AD
      lx[k], lu[k], lxx[k], lux[k], luu[k] ← cost_derivatives(X[k], U[k])
    lfx, lfxx ← terminal_derivatives(X[N])
    
    ═══ Step 2: Backward pass ═══
    Vx ← lfx;  Vxx ← lfxx
    dJ1, dJ2 ← 0, 0
    backward_ok ← true
    for k = N-1..0:
      // Q 系数（iLQR 版，状态正则化）
      Vxx_reg = Vxx + μ * I_n
      Qx  = lx[k] + fx[k]^T @ Vx
      Qu  = lu[k] + fu[k]^T @ Vx
      Qxx = lxx[k] + fx[k]^T @ Vxx_reg @ fx[k]
      Qux = lux[k] + fu[k]^T @ Vxx_reg @ fx[k]
      Quu = luu[k] + fu[k]^T @ Vxx_reg @ fu[k]
      
      // Cholesky 分解（检测正定性）
      try L = cholesky(Quu):
      except:
        μ ← max(μ_min, μ * Δ);  Δ ← max(Δ0, Δ * Δ0)
        backward_ok ← false;  break
      
      // 最优增益
      kk[k] ← -cho_solve(L, Qu)           // 前馈修正
      KK[k] ← -cho_solve(L, Qux)          // 反馈增益
      
      // 期望下降量
      dJ1 += kk[k]^T @ Qu
      dJ2 += kk[k]^T @ Quu @ kk[k]
      
      // 值函数递推
      Vx  ← Qx + KK[k]^T @ Quu @ kk[k] + KK[k]^T @ Qu + Qux^T @ kk[k]
      Vxx ← Qxx + KK[k]^T @ Quu @ KK[k] + KK[k]^T @ Qux + Qux^T @ KK[k]
      Vxx ← (Vxx + Vxx^T) / 2    // 强制对称
    
    if not backward_ok: continue  // 用更大 μ 重试 backward
    
    ═══ Step 3: Forward pass (line search) ═══
    accepted ← false
    for α in [1.0, 0.5, 0.25, ..., 2^{-10}]:
      X_new[0] = x0
      for k = 0..N-1:
        U_new[k] = U[k] + α * kk[k] + KK[k] @ (X_new[k] - X[k])
        X_new[k+1] = dynamics(X_new[k], U_new[k])
      J_new = cost(X_new, U_new)
      expected = -α * dJ1 - 0.5 * α^2 * dJ2
      z = (J_old - J_new) / max(expected, 1e-8)
      if 0.1 < z < 10:
        accepted ← true; break
    
    if not accepted:
      μ ← max(μ_min, μ * Δ);  Δ ← max(Δ0, Δ * Δ0)
      continue
    
    ═══ Step 4: 更新并尝试减小正则化 ═══
    X ← X_new;  U ← U_new;  J_old ← J_new
    Δ ← min(1/Δ0, Δ / Δ0);  μ ← μ * Δ  (若 < μ_min 则置零)
    
    ═══ Step 5: 收敛判据 ═══
    grad_norm = max_k ||Qu[k]||_∞
    if |J_old - J_new| / max(|J_old|, 1) < εJ  and  grad_norm < εg:
      break
  
  return X, U, {KK[k]}  // 返回最优轨迹与反馈增益序列
```

### 计算瓶颈分析

对 $n = 37, m = 17$（Talos 人形上身），$N = 100$：

| 步骤 | 计算量 | 方法 | 耗时估计 |
|------|--------|------|---------|
| $f_x, f_u$ 有限差分 | $(n+m)\times N$ 次仿真 | $54 \times 100 = 5400$ 次 | 50-100 ms |
| $f_x, f_u$ Pinocchio 解析 | $N$ 次 `computeABADerivatives` | 100 次 | 2-5 ms |
| Backward pass | $N$ 次 $m\times m$ Cholesky | $100 \times 17^3 \approx 5\times10^5$ flops | 0.1 ms |
| Forward pass | $N$ 次动力学仿真 | 100 次 | 1-2 ms |

**结论**：**瓶颈在动力学导数**。Pinocchio 的解析导数 (`computeABADerivatives`) 比有限差分快 10-50 倍，是机器人 DDP 从论文走向实机的关键工程门槛。

### Python 参考实现骨架（JAX）

```python
import jax
import jax.numpy as jnp
from functools import partial

def dynamics(x, u, params):
    """Cart-pole 动力学（Euler 离散化）
    x = [pos, theta, vel, omega], u = [force]
    """
    dt = params['dt']
    mc, mp, l, g = params['mc'], params['mp'], params['l'], params['g']
    pos, theta, vel, omega = x[0], x[1], x[2], x[3]
    F = u[0]
    
    # 连续动力学（标准 cart-pole 方程）
    sin_t, cos_t = jnp.sin(theta), jnp.cos(theta)
    total_mass = mc + mp
    denom = total_mass - mp * cos_t**2
    
    acc = (F + mp * l * omega**2 * sin_t - mp * g * sin_t * cos_t) / denom
    alpha = (g * sin_t * total_mass - cos_t * (F + mp * l * omega**2 * sin_t)) / (l * denom)
    
    # Euler 积分
    x_next = jnp.array([
        pos + vel * dt,
        theta + omega * dt,
        vel + acc * dt,
        omega + alpha * dt
    ])
    return x_next

def stage_cost(x, u, params):
    """二次阶段代价"""
    Q, R = params['Q'], params['R']
    x_ref = params['x_ref']
    dx = x - x_ref
    return 0.5 * dx @ Q @ dx + 0.5 * u @ R @ u

def terminal_cost(x, params):
    """终端代价"""
    Qf = params['Qf']
    x_ref = params['x_ref']
    dx = x - x_ref
    return 0.5 * dx @ Qf @ dx

@partial(jax.jit, static_argnums=(4,))
def ilqr_backward(X, U, dyn_params, cost_params, mu):
    """iLQR backward pass，返回 (kk, KK, dJ1, dJ2)"""
    N = U.shape[0]
    n, m = X.shape[1], U.shape[1]
    
    # 终端初始化
    Vx = jax.grad(terminal_cost)(X[-1], cost_params)
    Vxx = jax.hessian(terminal_cost)(X[-1], cost_params)
    
    kk_list, KK_list = [], []
    dJ1, dJ2 = 0.0, 0.0
    
    for k in range(N-1, -1, -1):
        # 动力学 Jacobian（JAX 自动微分）
        fx = jax.jacobian(dynamics, 0)(X[k], U[k], dyn_params)
        fu = jax.jacobian(dynamics, 1)(X[k], U[k], dyn_params)
        
        # 代价导数
        lx = jax.grad(stage_cost, 0)(X[k], U[k], cost_params)
        lu = jax.grad(stage_cost, 1)(X[k], U[k], cost_params)
        lxx = jax.hessian(stage_cost, 0)(X[k], U[k], cost_params)
        luu = jax.hessian(stage_cost, 1)(X[k], U[k], cost_params)
        lux = jax.jacobian(jax.grad(stage_cost, 1), 0)(X[k], U[k], cost_params)
        
        # Q 系数（状态正则化）
        Vxx_reg = Vxx + mu * jnp.eye(n)
        Qx = lx + fx.T @ Vx
        Qu = lu + fu.T @ Vx
        Qxx = lxx + fx.T @ Vxx_reg @ fx
        Qux = lux + fu.T @ Vxx_reg @ fx
        Quu = luu + fu.T @ Vxx_reg @ fu
        
        # Cholesky 分解 + 求解
        L = jnp.linalg.cholesky(Quu)
        k_k = -jax.scipy.linalg.cho_solve((L, True), Qu)
        K_k = -jax.scipy.linalg.cho_solve((L, True), Qux)
        
        # 期望下降
        dJ1 += k_k @ Qu
        dJ2 += k_k @ Quu @ k_k
        
        # 值函数递推
        Vx = Qx + K_k.T @ Quu @ k_k + K_k.T @ Qu + Qux.T @ k_k
        Vxx = Qxx + K_k.T @ Quu @ K_k + K_k.T @ Qux + Qux.T @ K_k
        Vxx = (Vxx + Vxx.T) / 2  # 强制对称
        
        kk_list.append(k_k)
        KK_list.append(K_k)
    
    return jnp.stack(kk_list[::-1]), jnp.stack(KK_list[::-1]), dJ1, dJ2


def ilqr_forward(X, U, kk, KK, alpha, dyn_params):
    """Forward pass with line search step α"""
    N = U.shape[0]
    X_new = [X[0]]
    U_new = []
    
    for k in range(N):
        dx = X_new[k] - X[k]
        u_new = U[k] + alpha * kk[k] + KK[k] @ dx  # K 不乘 α！
        x_next = dynamics(X_new[k], u_new, dyn_params)
        U_new.append(u_new)
        X_new.append(x_next)
    
    return jnp.stack(X_new), jnp.stack(U_new)
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：忘记在 rollout 中 clip $u$ 到物理约束**

大 $\alpha$ 可能使 $\hat u_k$ 超出执行器极限（如电机最大力矩 30 N·m）。超限输入送入仿真器会导致数值爆炸。应在 forward pass 内部做 clip 或直接用 Box-DDP。

⚠️ **编程陷阱：$N$ 太大未用并行导数**

$N = 500$ 时顺序计算动力学导数可能 > 1 s。应使用 OpenMP（C++）或 `jax.vmap`（Python）并行每步导数计算。

💡 **概念误区：认为 Warm-start 时可以重用上次的 $K$**

MPC 中 shifting warm-start 把旧 $K_{1:N-1}$ 直接用到新周期第 $0:N-2$ 上，但物理时间已偏移一步。应至少跑 1 次 backward 重算——这是 RTI（Real-Time Iteration）的核心思想（§3.9.10）。

⚠️ **编程陷阱：JAX 循环中 append 列表导致重新编译**

JAX 的 `jit` 要求静态形状。上面的 `for k in range(N)` 配合 Python list append 会导致每次 `N` 变化时重新编译。生产代码应用 `jax.lax.scan` 替代：

```python
def backward_step(carry, inputs):
    Vx, Vxx, dJ1, dJ2 = carry
    fx_k, fu_k, lx_k, lu_k, lxx_k, lux_k, luu_k = inputs
    # ... 单步 backward 计算 ...
    return (Vx_new, Vxx_new, dJ1_new, dJ2_new), (k_k, K_k)

# 用 scan 替代循环
final_carry, (kk, KK) = jax.lax.scan(backward_step, init_carry, all_inputs, reverse=True)
```

### 练习

1. 为 cart-pole ($n=4, m=1$) 实现完整 iLQR：用 $\ell(x,u) = \frac{1}{2}x^\top Q x + \frac{1}{2}u^\top R u$，$\ell_f(x) = \frac{1}{2}(x-x_g)^\top Q_f (x-x_g)$，$Q_f = \text{diag}(100,100,10,10)$。用 JAX 自动微分实现 $f_x, f_u$；记录 10 次迭代的 $J$ 与 $\|Q_u\|_\infty$。
2. （跨章综合题）用同一 cart-pole 问题，对比 (a) iLQR + 有限差分、(b) iLQR + JAX AD、(c) IPOPT + Direct Collocation（CasADi）三种方法的收敛速度和解质量。
3. 在上述实现中加入 $\mu$ 自适应调度。观察：失败次数如何随初始猜测质量变化？好初始猜测（如线性插值）vs 坏初始猜测（零控制）的差异有多大？

---

## §3.9.9 可微动力学与自动微分 ⭐⭐

### 动力学导数的四种获取方式

DDP 每步需要 $f_x, f_u$（iLQR），或额外需要 $f_{xx}, f_{ux}, f_{uu}$（full DDP）。获取方式决定了工程效率：

| 方式 | 精度 | 速度（7-DoF） | 开发成本 | 适用 |
|------|------|--------------|---------|------|
| 有限差分（前向） | $O(\epsilon)$ | 慢（$n+m$ 次仿真） | 最低 | 原型验证 |
| 有限差分（中心） | $O(\epsilon^2)$ | 更慢（$2(n+m)$ 次仿真） | 最低 | 高精度原型 |
| Pinocchio 解析 | 精确 | 极快（3-17 μs） | 中 | 刚性关节机器人 |
| CppAD/CppADCodeGen | 精确 | 快（解析 2-5x） | 中 | OCS2/acados |
| JAX `jax.jacfwd`/`jacrev` | 精确 | 中等 | 最低 | Python 原型 |

### Pinocchio 解析导数的核心算法

**Carpentier-Mansard RSS 2018** 的贡献是对 ABA（Articulated Body Algorithm）与 RNEA 关于 $(q, \dot q, \tau)$ 求**解析偏导的递归算法**。

核心思想：ABA 的正向通过 $n$ 个关节做正向递推（速度、加速度），再反向递推（力）。对每个中间量关于输入的偏导，可以用**链式法则沿递推结构传播**，无需重复计算整体。

C++ API：

```cpp
#include <pinocchio/algorithm/aba-derivatives.hpp>

// 计算 ABA 导数：∂q̈/∂q, ∂q̈/∂v, ∂q̈/∂τ
pinocchio::computeABADerivatives(model, data, q, v, tau);
// 结果存储在：
//   data.ddq_dq: ∂q̈/∂q ∈ ℝ^{n×n}
//   data.ddq_dv: ∂q̈/∂v ∈ ℝ^{n×n}  
//   data.Minv:   ∂q̈/∂τ = M^{-1} ∈ ℝ^{n×n}（质量矩阵的逆）
```

从 ABA 导数得到离散化 Jacobian：对 semi-implicit Euler $v_{k+1} = v_k + \ddot q \cdot\Delta t$，$q_{k+1} = q_k \oplus v_{k+1}\Delta t$：

$$
f_x = \begin{pmatrix} I + J_q\Delta t^2 & I\Delta t + J_v\Delta t^2 \\ J_q\Delta t & I + J_v\Delta t \end{pmatrix}, \quad
f_u = \begin{pmatrix} M^{-1}\Delta t^2 \\ M^{-1}\Delta t \end{pmatrix}
$$

其中 $J_q = \text{data.ddq\_dq}$，$J_v = \text{data.ddq\_dv}$。

**这是 Crocoddyl 达到毫秒级 whole-body MPC 的关键单点**：比有限差分快 10-50 倍。

### 有限差分的正确实现

```python
def finite_diff_fx(dynamics, x, u, params, eps=1e-6):
    """前向有限差分计算 f_x ∈ ℝ^{n×n}"""
    n = x.shape[0]
    f0 = dynamics(x, u, params)
    fx = np.zeros((n, n))
    for i in range(n):
        x_plus = x.copy()
        x_plus[i] += eps
        fx[:, i] = (dynamics(x_plus, u, params) - f0) / eps
    return fx
```

**最优 $\epsilon$ 选取**：
- 前向差分：$\epsilon_\text{opt} = \sqrt{\epsilon_\text{machine}} \approx 1.5\times 10^{-8}$（64-bit double）
- 中心差分：$\epsilon_\text{opt} = \epsilon_\text{machine}^{1/3} \approx 6\times 10^{-6}$，精度 $O(\epsilon^2)$ 但需 2 倍函数调用

### 可微仿真器生态

| 仿真器 | 可微方式 | 二阶导 | 接触 | 生态 |
|--------|---------|--------|------|------|
| MuJoCo 3.x | `mjd_transitionFD`（有限差分） | 否 | 原生 | MJPC |
| Brax（Google JAX） | 全链路 AD | 否 | 弹簧近似 | RL |
| Drake | `AutoDiffXd`（一阶） | 否 | LCP | 教学 |
| Dojo（Julia） | Smooth contact AD | 是 | 可微接触 | 研究 |
| Nimble（diff physics） | 符号+AD | 是 | 可微接触 | 研究 |
| Pinocchio + CppAD | 解析 + AD | 是 | 无接触 | Crocoddyl |

**工程建议**：刚性关节用 Pinocchio 解析；接触/软体/流体用可微仿真 AD；原型开发用 JAX。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：有限差分 $\epsilon$ 选得不当**

最优 $\epsilon$ 为 $\sqrt{\text{machine epsilon}} \approx 10^{-8}$（64-bit）。过大（$10^{-4}$）→ 截断误差主导；过小（$10^{-12}$）→ 舍入误差主导。

⚠️ **编程陷阱：对四元数/李群状态直接做有限差分**

浮动基座的配置空间包含 $SE(3)$（或四元数+位置），不能对四元数的 4 个分量独立加扰动（会破坏单位约束 $\|q\| = 1$）。正确做法：在切空间（李代数 $\mathfrak{se}(3)$）上加扰动 $\delta\xi$，用 $q_\text{perturbed} = q \oplus \epsilon\,e_i$ 的指数映射。Pinocchio 的 `integrate` 函数自动处理此问题。

### 练习

1. 对 7-DoF Panda 机械臂，分别用有限差分（前向/中心）和 Pinocchio `computeABADerivatives` 计算 $f_x$，比较精度和速度。
2. 尝试用 JAX 对 cart-pole 动力学求二阶导 $f_{xx}$（用 `jax.hessian`），验证其维度为 $4 \times 4 \times 4$（输出 $\times$ 输入 $\times$ 输入）。
3. 解释为什么 CppADCodeGen 在 OCS2 中比 Pinocchio 解析导数还快？（提示：CodeGen 把整个动力学+积分器编译为单个函数，避免了虚函数开销。）

---

## §3.9.10 在线 iLQR / Warm-Starting MPC ⭐⭐⭐

### iLQR 作为 MPC 内层求解器

以 $\Delta t_{\text{MPC}} = 10$-$50$ ms 为周期，每周期解一个短时域 OCP（$T = 0.5$-$2$ s，$N = 50$-$200$）。关键是 **warm-start**——利用上一周期的解来初始化下一周期。

**为什么 warm-start 对 iLQR/MPC 如此重要？** 因为 iLQR 是**局部优化**——给定不同初值可能收敛到不同局部极小。在 MPC 中，相邻时刻的 OCP 问题非常相似（只移动了一步），旧解经过简单平移后就是新问题的极好初始猜测。这使得 1-3 次迭代即可"跟踪"到新最优解。

### Shifting Warm-Start

上一周期的解 $(x^\star_{0:N}, u^\star_{0:N-1})$ → 下一周期初值：

$$
\tilde u_k^{\text{new}} = u^\star_{k+1}^{\text{old}}, \quad k = 0, \ldots, N-2
$$

尾部 $\tilde u_{N-1}^{\text{new}}$ 填 LQR 稳定控制或重复最后值。

状态 warm-start 需要 multiple-shooting 表述（FDDP）才能直接注入 $x_k$；单射击 DDP 只能 warm-start 控制序列。

**warm-start 失效场景**：
- 步态切换（trot → gallop）：旧解结构完全不同
- 大扰动（摔倒恢复）：旧最优轨迹远离当前状态
- 目标突变：终端代价突然改变

这些场景需要**重新规划**（cold-start 或 MPPI/CEM warm-start）。

### Real-Time Iteration（RTI）

**极端做法**：每周期只做**一次** SQP/iLQR 迭代，拆分为两阶段：

| 阶段 | 内容 | 触发时机 | 典型耗时 |
|------|------|---------|---------|
| **Preparation** | shift warm-start + 计算导数 + backward pass | 上一周期末（无需新测量） | 3-10 ms |
| **Feedback** | 注入新状态 $\hat x$ + forward pass 第一步 + 输出 $u_0$ | 新测量到达后 | 0.1-1 ms |

Feedback 阶段的超低延迟保证了控制的实时性。Neunert 2016/2018 在四足实机上以 190 Hz 运行 iLQR-RTI。

**RTI 的数学保证（Diehl-Bock-Schloder 2005）**：设 NMPC 问题满足 strong regularity（LICQ + SOSC + strict complementarity），且扰动 $\|\hat x_0^{k+1} - \hat x_0^k\| \le \delta$ 充分小，则 RTI 闭环满足**收缩性**：

$$
\|w^{k+1} - w^\star_{k+1}\| \le \kappa\|w^k - w^\star_k\| + \omega\|\Delta x_0\|
$$

其中 $\kappa < 1$（内层单步收缩因子），$\omega > 0$（外层扰动敏感度），$w = (x,u,\lambda)$ 是 KKT 向量。这意味着：即使每步只做一次不精确的 SQP 迭代，闭环误差仍然有界且渐近收敛。

### 完整 RTI 时间线

```
时间 ──────────────────────────────────────────→

周期 k:
  |── Preparation(k) ──|── wait ──|── Feedback(k) ─|
  t_k-1                            t_k              t_k + τ_fb

  Preparation:
    1. shift warm-start: u_new[0:N-2] = u_old[1:N-1], u_new[N-1] = u_LQR
    2. rollout: X_new = rollout(x_old[1], U_new)
    3. 计算 fx, fu, lx, lu, lxx, luu（对 X_new, U_new）
    4. backward pass → kk, KK
    （以上全部不需要当前周期的新测量！）

  Feedback:
    5. 接收新状态测量 x̂
    6. u_applied = U_new[0] + kk[0] + KK[0] @ (x̂ - X_new[0])
    7. 发送 u_applied 到执行器
    （延迟 = 步骤 6-7 的时间 ≈ 0.1 ms）
```

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为"一次迭代不够精确所以不能用"**

RTI 的数学保证不需要精确解——只要**warm-start 足够好**且**系统变化足够慢**（$\|\hat x_0^{k+1} - \hat x_0^k\| < \delta$），单次迭代就能保持 ISS 稳定性。这就是为什么腿足 MPC 可以只跑 1-3 次 iLQR 迭代：上一步的解已经是非常好的初始猜测。

⚠️ **编程陷阱：RTI 的 preparation 阶段用了新测量**

如果 preparation 等待新测量才开始，延迟增加一个完整的 preparation 时间。正确做法：用旧的 shifted 解做 preparation，新测量只在 feedback 阶段使用。

### 练习

1. 对 cart-pole 摆起问题实现 MPC 循环：每 20 ms 调用一次 iLQR（最多 3 次迭代），用 shift warm-start。对比无 warm-start（每次 cold-start 50 次迭代）的控制质量和计算时间。
2. 实现 RTI 的 preparation/feedback 分离：在 preparation 阶段完成 backward pass，在 feedback 阶段只做 forward pass 第一步。测量 feedback 延迟。
3. 设计一个实验观察 RTI 的"收缩性"：绘制 $\|u_0^k - u_0^{\star,k}\|$ 随 MPC 周期 $k$ 的变化，验证指数衰减。

---

## §3.9.11 Model-Based RL 中的 iLQR ⭐⭐⭐

### 三大 MBRL 规划器对比

| 规划器 | 需要梯度 | 收敛率 | 并行性 | 适用动力学 |
|--------|---------|--------|--------|-----------|
| iLQR | 是（$f_x, f_u$） | 超线性 | 差（串行 Riccati） | 光滑可微 |
| CEM/PETS | 否 | 无保证 | 好（并行采样） | 任意（黑箱） |
| MPPI | 否 | 无保证 | 极好（GPU 万级 rollout） | 任意（含非光滑） |

**MPPI 与 iLQR 的深层联系**：Williams et al. 2017 证明，当代价二次 + 动力学线性 + 噪声高斯时，path integral control 的闭式最优分布退化为 LQG 的 Riccati 解。因此 **iLQR 是 MPPI 在二次近似下的确定性极限**。

类比：MPPI 像用蒙特卡洛方法在控制空间中"撒网捕鱼"——大量随机试探然后加权平均；iLQR 像用解析几何精确计算鱼的位置——高效但需要"水清"（动力学可微）。当"水浑"（接触/非光滑）时，撒网比计算更可靠。

### Differentiable MPC（Amos et al. 2018 NeurIPS）

把 box-iLQR 视作**可微层**：通过 KKT 固定点的隐式微分反向传播关于 cost/dynamics 参数的梯度。

核心方程：在不动点 $(x^\star, u^\star)$ 处，$\partial u^\star / \partial \theta$ 由 KKT 线性系统给出：

$$
\frac{\partial u^\star}{\partial \theta} = -\left(\frac{\partial^2 \mathcal{L}}{\partial u^2}\right)^{-1} \frac{\partial^2 \mathcal{L}}{\partial u\,\partial \theta}
$$

该线性系统本身具有 LQR 结构（banded Hessian），可用 Riccati 反向递推 $O(Nn^3)$ 求解——远优于通用 KKT LU $O(N^3n^3)$。

**应用**：端到端策略学习（学习代价函数参数）、inverse RL（从演示恢复代价）、sim2real transfer（微调动力学参数）。

### TD-MPC2 与隐空间规划

Hansen et al. 2023 (TD-MPC2) 在**隐空间**（latent space）做 iLQR/MPPI：

1. 编码器把观测 $o$ 映射到隐状态 $z$
2. 隐空间世界模型 $z' = f_\phi(z, a)$（可微）
3. 在隐空间做 CEM/MPPI（采样高效，维度 $z \sim 50$）
4. 解码器/价值函数评估轨迹质量

iLQR 在此场景中的优势：可以利用 $\partial f_\phi / \partial a$（神经网络 Jacobian 通过 backprop 自动获得）做高效梯度规划。但需注意隐空间的流形结构不一定满足 iLQR 的欧氏假设。

### ⚠️ 常见陷阱

💡 **概念误区：把 acados 当作 DDP**

acados 是直接 multiple-shooting + 结构化 QP（HPIPM/BLASFEO），**不是 DDP 家族**。它与 DDP 的相似之处是都用 Riccati 结构求解块稀疏 KKT，但数学形式不同。判断依据：acados 的决策变量是 $(x, u)$ 全部节点（primal-dual），DDP 只优化 $u$ 序列（primal-only + Bellman 消去 $x$）。

💡 **概念误区：认为"可微 MPC = 把 iLQR 展开为计算图"**

展开（unrolling）iLQR 的所有迭代形成计算图再反向传播——可以工作但梯度会**爆炸或消失**（经过 20+ 次迭代的展开）。正确做法是**隐式微分**：只在不动点处用 KKT 条件的隐函数定理求导，避免展开。

### 练习

1. 用 `trajax`（JAX）实现 cart-pole 的 iLQR，与 CEM（100 个采样）对比收敛速度和计算时间。
2. 阐述：为什么接触/非光滑动力学下 MPPI 比 iLQR 更稳健？（提示：$f_{xx}$ 在接触点处不连续。）
3. 推导 Differentiable MPC 中 $\partial u^\star_0 / \partial Q_f$ 的表达式（$Q_f$ 是终端代价权重矩阵）。说明它如何通过 Riccati backward 计算。

---

## §3.9.12 与 SQP/直接法的对比 ⭐⭐⭐

### 对比表

| 维度 | iLQR/DDP | SQP + Direct Collocation (IPOPT) |
|------|----------|----------------------------------|
| 变量数 | $Nm$（仅 $u$） | $N(n+m)$（$x$ 和 $u$） |
| 稀疏度 | 时间 banded $O(N)$ | KKT banded $O(N)$ |
| Hessian | 自然从 $V_{xx}$ 得 | BFGS/Exact，稀疏 |
| 约束 | AL/barrier/projected | 原生 KKT + active set |
| Warm-start | 单射击差、FDDP 好 | 优 |
| 光滑问题 | 二次收敛、快 | 超线性 |
| 反馈增益 | 自然得 $K_k$ | 需额外计算 |
| 机器人实时 MPC | **首选** | 慢 3-10x |
| 轨迹设计（离线） | 够用 | **首选**，精度高 |
| 稀疏约束 | 不擅长 | 擅长 |

### DDP 与 Direct Multiple-Shooting SQP 的深层等价

**Bock-Plitt 1984** 的多射击 SQP 把 OCP 写成 NLP：

$$
\min_{x_{0:N}, u_{0:N-1}} J \quad \text{s.t.} \quad x_{k+1} - f(x_k, u_k) = 0
$$

对这个 NLP 做一步 SQP（Newton on KKT），然后用 Riccati 递推（block elimination）求解 KKT 系统——得到的步方向与 FDDP 的 defect-corrected backward pass **完全等价**（Mastalli 2020 Prop. 2）。

所以：**FDDP = 多射击 SQP 的 condensed Riccati 解法**。两者的区别不在数学，在于代码组织和工程优化：
- FDDP 用 Bellman 语言思考（Q 函数、值函数）
- 多射击 SQP 用 NLP 语言思考（KKT、active set、稀疏线性代数）

### 选型决策流程

```
需要实时 MPC (>50 Hz)?
├── 是 → 有复杂状态约束?
│       ├── 是 → Crocoddyl BoxFDDP / Aligator ProxDDP
│       └── 否 → iLQR/DDP（最快）
└── 否 → 离线轨迹优化?
        ├── 是 → CasADi + IPOPT（精度高、约束处理强）
        └── 否 → 学习场景？
                ├── 是 → trajax / DiffMPC（可微）
                └── 否 → acados RTI（嵌入式部署）
```

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为"直接法一定比间接法好"**

这是 2010-2020 年的流行观点（受 IPOPT 成功影响）。但在机器人实时 MPC 中，DDP 家族（"间接-直接混合"）重新崛起——因为它**自然给出反馈增益 $K$**、**warm-start 极佳**、**不需要稀疏线性代数库**。2020-2025 年 Crocoddyl/Aligator/OCS2 的成功证明：对机器人，DDP 类方法比通用 NLP solver 更合适。

### 练习

1. 对 7-DoF reaching 问题（$n=14, m=7, N=50$），估算 iLQR 与 IPOPT-DirCol 的单步 Newton 迭代计算量（flops）。
2. 解释为什么 iLQR 不擅长处理稀疏路径约束（如 $\|x_k - x_\text{obs}\| > d$）。ALTRO 如何解决这个问题？
3. 从"DDP = condensed Riccati on multiple-shooting KKT"出发，解释为什么 FDDP 的 defect correction $V_x' \leftarrow V_x' + V_{xx}'\bar f$ 是必要的。

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 工程价值 |
|--------|---------|------|---------|
| OCP 标准形式 | 单射击表述 + Bellman 分解 | ⭐⭐ | 理解所有 DDP 变体的基础 |
| DDP Backward Pass | Q 函数六系数 + Riccati-like 递推 | ⭐⭐ | 手推能力 |
| iLQR = GN 近似 | 丢弃动力学 Hessian → 超线性 | ⭐⭐ | 95% 工程场景的默认选择 |
| PMP/DP/LQR 统一 | 三条线的精确交汇 | ⭐⭐⭐ | 理论理解的"灵魂" |
| Forward Pass + Line Search | Armijo 准则 + backtracking | ⭐⭐ | 鲁棒收敛的关键 |
| 正则化 | 状态正则化 > 控制正则化（LM 等价） | ⭐⭐ | 工程必备 |
| 采样时间 | $\Delta t$ 选择与变步长 | ⭐⭐ | 性能调优 |
| 计算瓶颈 | 动力学导数 → Pinocchio 解析 | ⭐⭐ | 10-50x 加速 |
| MPC warm-start | Shift + RTI 分离 | ⭐⭐⭐ | 实时控制核心 |
| MBRL 交叉 | DiffMPC + MPPI-iLQR 等价 | ⭐⭐⭐ | 研究前沿 |
| SQP 对比 | DDP = condensed Riccati SQP | ⭐⭐⭐ | 架构选型依据 |

---

## 累积项目：本章新增模块

**DDP/MPC 全栈项目进度**：

| 章节 | 新增模块 | 实现目标 |
|------|---------|---------|
| 3.5 LQR | `tvlqr_solver.py` | Riccati 递推 + 反馈增益 |
| **3.9 DDP/iLQR** | **`ilqr_solver.py`** | **完整 iLQR + cart-pole 摆起** |
| 3.10 约束 DDP | `box_ddp.py` | 控制约束 + AL-iLQR |
| 3.11 MPC 稳定性 | `mpc_terminal.py` | 终端代价/集合设计 |
| 3.12 MPC 数值 | `acados_quad.py` | acados RTI + 四旋翼 |

本章实现目标：`ilqr_solver.py` 包含完整的 backward pass（iLQR 版）、forward pass（Armijo line search）、Tassa 状态正则化、$\mu$ 自适应调度。在 cart-pole 摆起任务（从下垂到直立）上 10-20 次迭代收敛，wall-clock time < 50 ms（JAX JIT 后）。

验收标准：
1. 摆起轨迹使终端角度误差 < 0.01 rad
2. $\|Q_u\|_\infty < 10^{-6}$（一阶最优性）
3. 能画出 $J$ 与 $\|Q_u\|$ 随迭代下降的 log-scale 图
4. 能输出 $K_0$ 并用 $u = u_0^\star + K_0(x - x_0^\star)$ 做闭环仿真

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Tedrake《Underactuated》Ch.10 | ⭐⭐ | 最易读的 DDP/iLQR 引论 |
| Tassa-Erez-Todorov IROS 2012 | ⭐⭐ | 工程化 DDP 标准参考 |
| CMU 16-745 Lecture 10 视频 | ⭐⭐ | Manchester 边推导边编码 |
| Giftthaler et al. IROS 2018 | ⭐⭐⭐ | iLQR = GN 的精确证明 |
| Liao-Shoemaker IEEE TAC 1991 | ⭐⭐⭐⭐ | DDP 二次收敛的严格分析 |
| Jacobson-Mayne 1970 专著 | ⭐⭐⭐⭐ | DDP 原始理论（历史文献） |
| Amos et al. NeurIPS 2018 | ⭐⭐⭐⭐ | Differentiable MPC |
| Pantoja IJC 1988 | ⭐⭐⭐⭐ | DDP = stagewise Newton |
| Diehl-Bock-Schloder Automatica 2005 | ⭐⭐⭐ | RTI 理论保证 |
| Mastalli ICRA 2020 (Crocoddyl) | ⭐⭐⭐ | FDDP + 框架设计 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| Backward pass Cholesky 失败 | $Q_{uu}$ 非 PD | 1. 检查 $V'_{xx}$ 是否正定（打印最小特征值） 2. 增大 $\mu$ 3. 确认 $R \succ 0$ 4. 若用 full DDP 检查张量项符号 | §3.9.6 |
| Line search 全部失败 | 二次模型失效 / $\mu$ 太小 | 1. 打印 $\Delta_1, \Delta_2$ 2. 检查 $z$ 值分布 3. 增大 $\mu$ 4. 检查动力学有无不连续（接触切换） | §3.9.5 |
| 收敛但代价远非最优 | 局部最小值 | 1. 用多个随机初始控制 2. 增大 $N$（视域不足） 3. 尝试 MPPI warm-start 4. 减小 $Q_f$（终端代价过强可能锁定次优） | §3.9.11 |
| 迭代次数 > 100 仍不收敛 | ill-conditioning / $\Delta t$ 不当 | 1. 检查系统时间常数 vs $\Delta t$ 2. 打印 $\text{cond}(Q_{uu})$ 3. 切多射击（FDDP） 4. 加更多正则 | §3.9.7 |
| Forward pass 数值爆炸（NaN） | 控制超限 / 仿真失稳 | 1. clip $u$ 到物理范围 2. 减小 $\alpha$ 起始值 3. 检查动力学积分器稳定性（$\Delta t$ vs 刚度） 4. 加 Box-DDP | §3.9.8 |
| 收敛速度突然变慢（进入"平台"） | 正则化 $\mu$ 未及时减小 | 1. 打印 $\mu$ 随迭代的变化 2. 检查 $\mu$ 调度是否 stuck 在高值 3. 手动降低 $\mu$ 看是否恢复 | §3.9.6 |

---

## 关键定理清单

1. **Bellman 最优性（3.3）**：$V_k(x) = \min_u[\ell + V_{k+1}(f)]$ 是最优性充要条件
2. **DDP Backward Pass 局部最优性**：$Q_{uu} \succ 0 \Rightarrow \delta u^\star = k + K\delta x$ 唯一最小，$V_{xx} \succ 0$ 保持
3. **DDP 局部二次收敛（Mayne 1966 + Liao-Shoemaker 1991）**：$\|u^{(k+1)} - u^\star\| \le C\|u^{(k)} - u^\star\|^2$
4. **iLQR = Gauss-Newton 超线性（Giftthaler 2018）**：零残差时近二次，否则超线性
5. **LQR 一步收敛（Mayne 1966）**：线性动力学 + 二次代价 → DDP = iLQR = TVLQR，一次迭代即最优
6. **iLQR ≡ 单射击 GN（Giftthaler 2018）**：步方向与 Gauss-Newton 完全一致
7. **Box-DDP 保持二次收敛（Tassa 2014, Bertsekas 1982）**：活动集正确辨识后 Projected Newton 收敛
8. **RTI 收缩性（Diehl 2005）**：$\|w^{k+1} - w^\star_{k+1}\| \le \kappa\|w^k - w^\star_k\| + \omega\|\Delta x_0\|$
9. **PMP-DDP 对偶（Pantoja 1988）**：$\lambda_k = V_{k,x}$，backward pass = 共态方程 + 灵敏度 Riccati
10. **FDDP = condensed Riccati SQP（Mastalli 2020）**：FDDP 方向等价于多射击 NLP 的 KKT Newton 步

---

## 与后续专题桥梁

**→ 3.10 约束 DDP/Crocoddyl**：本专题的 Box-DDP、AL-iLQR、FDDP 概念直接展开为 Crocoddyl 的 FDDP/Box-FDDP solver 实现。Pinocchio 后端、多接触 DAM 抽象、Aligator 的 ProxDDP 约束扩展。§3.9.2 的 Q 系数递推原样保留，只是 $\delta u$ 的极小化被替换为 box-QP / KKT 子问题。

**→ 3.11 MPC 稳定性**：§3.9.10 的 warm-start 与 RTI 是 MPC 的核心计算机制；3.11 将系统讲 receding horizon 原理、终端代价/终端约束设计的 Mayne 2000 四条件、tube MPC、economic MPC。

**→ 3.12 MPC 数值求解**：§3.9.12 的对比表直接引出 direct transcription、condensing vs sparse、acados RTI scheme 的系统讲解。

**→ 第六批 MBRL**：§3.9.11 的 Differentiable MPC、MPPI-iLQR 等价性预备了 model-based RL 的规划组件。

---

## 自测题（5 道，含跨章综合）

1. **（推导题）** 写出 DDP 的 $Q_{uu}$ 完整表达式（含 $f_{uu}$ 张量缩并项）与 iLQR 的 $Q_{uu}$；证明当 $f$ 线性时两者等价。推导 $k, K$ 的闭式解并说明当 $Q_{uu}$ 非 PD 时会出现何种数值问题。

2. **（连接题）** 设 $\lambda_{k+1} = V_{k+1,x}(\bar x_{k+1})$。证明 backward pass 的 $V_x$ 递推恰是离散 PMP 的共态方程 $\lambda_k = \ell_x + f_x^\top \lambda_{k+1}$（在收敛时）；并说明 $K_k$ 为何是共态反馈的矩阵实现。

3. **（实现题）** 为 cart-pole ($n=4, m=1$) 编写 iLQR 代码。用 JAX 自动微分实现 $f_x, f_u$；实现状态正则化 + 二次调度；记录 10 次迭代的 $J$ 与 $\|Q_u\|_\infty$。与 `trajax.ilqr` 对比。

4. **（对比题）** 给定 7-DoF reaching ($n=14, m=7, N=100, \Delta t=20\text{ms}$)，用 (a) iLQR+有限差分 (b) iLQR+Pinocchio 解析 (c) DDP+解析 (d) IPOPT+DirCol 四种方法求解，预估单次迭代和总耗时。

5. **（跨章综合题）** 解释 Tassa 2012 "状态正则化"优于"控制正则化"的三个原因：(i) $\mu \to \infty$ 时对 $K$ 的影响（结合 3.5 LQR 反馈结构）；(ii) trust-region 语义等价性（结合 3.3 Bellman 方程的局部性）；(iii) 为什么对 humanoid 奇异姿态特别重要（结合 3.2 的不稳定模态分析）。

---

## 常见陷阱汇总（12 条）

| # | 陷阱 | 类型 | 后果 | 正确做法 |
|---|------|------|------|---------|
| 1 | $Q_{uu}$ 非 PD 不处理直接 Cholesky | 编程 | NaN 传播 | 加状态正则化 + Cholesky try-catch |
| 2 | Line search 不用二阶期望下降 | 编程 | 频繁拒绝步长 | $\Delta J$ 含 $\alpha^2\Delta_2/2$ 项 |
| 3 | Forward pass 反馈项随 $\alpha$ 缩放 | 编程 | 小 $\alpha$ 时开环不稳 | $K$ 不乘 $\alpha$ |
| 4 | 控制超限未 clip | 编程 | 仿真爆炸 | Forward pass 内 clip 或 Box-DDP |
| 5 | 有限差分 $\epsilon$ 不当 | 编程 | 精度差 | $\epsilon = \sqrt{\text{eps}} \approx 10^{-8}$ |
| 6 | $N$ 太大未并行导数 | 工程 | 慢 | OpenMP / `jax.vmap` |
| 7 | $R = 0$ 导致 $Q_{uu}$ 非 PD 风险 | 设计 | Cholesky 失败 | 始终 $R \succeq 10^{-3} I$ |
| 8 | 接触切换未加密采样 | 设计 | $f_x$ 跳变致振荡 | 变步长或 contact-implicit |
| 9 | 连续/离散代价权重混淆 | 概念 | 权重失衡 | 离散 stage cost 乘 $\Delta t$ |
| 10 | Warm-start 重用旧 $K$ 不重算 | 工程 | 反馈失效 | 至少 1 次 backward |
| 11 | 把 iLQR 当 DDP 期待二次收敛 | 概念 | 误判迭代数 | iLQR 超线性，DDP 二次 |
| 12 | $V_{xx}$ 不强制对称化 | 编程 | 累积误差致 Cholesky 失败 | 每步 $(V+V^\top)/2$ |

---

## C++/Python 库映射

| 库 | 语言 | 支持算法 | 推荐场景 |
|---|---|---|---|
| **Crocoddyl** | C++/Py | DDP, FDDP, Box-FDDP | 四足/人形研究 MPC |
| **Aligator** | C++/Py | ProxDDP（硬约束原生） | 下一代约束 MPC |
| **OCS2** | C++/ROS | SLQ（连续 iLQR） | ETH 腿足生态 |
| **MJPC** | C++/Py | iLQG, Predictive Sampling | MuJoCo 生态 RL |
| **trajax** | Py (JAX) | iLQR, CEM, TVLQR | Python 原型 |
| **ALTRO** | Julia/C++ | AL-iLQR + Projected Newton | 通用约束 |
| **DiffMPC** | PyTorch | 可微 Box-iLQR | 端到端学习 |
| **Drake** | C++/Py | DirCol/DirTran（无内置 iLQR） | 教学/离线 |
| **acados** | C | SQP-RTI（非 DDP 系） | 嵌入式 NMPC |
| **Pinocchio** | C++/Py | 解析 $\partial\ddot q / \partial(q,\dot q,\tau)$ | 导数加速 |

**选型建议**：
- 离线 prototyping → trajax（JAX 生态，自动微分免费）
- 接触丰富的机器人研究 → Crocoddyl 或 Aligator
- 四足/人形实机 MPC → OCS2 或 Crocoddyl
- MuJoCo 生态 RL → MJPC
- 强化学习可微规划 → DiffMPC / trajax
- 嵌入式部署（无 Python） → acados

上述选型要结合实际的 CPU 预算、Python/C++ 生态偏好、以及是否需要 ROS 集成来综合决策。没有"最好的"库，只有"最适合当前需求的"库。

---

## §3.9.13 DDP/iLQR 与 SQP 的统一视角 ⭐⭐⭐

### 动机：为什么需要统一视角

到目前为止，我们从 Bellman 方程出发推导了 DDP，从 Gauss-Newton 近似得到了 iLQR，又从 PMP 视角理解了共态方程的联系。但在 2018-2025 年的文献中，越来越多的工作把 DDP/iLQR 放在**序列二次规划 (SQP)** 的框架下理解。这个统一视角不仅在理论上更清晰，更让我们能直接对比 DDP 与 acados、IPOPT 等"直接法"求解器的本质异同。

### DDP 是单射击 SQP 的 Riccati 求解

把 OCP 写成无约束 NLP（单射击，状态消去后）：$\min_{U} J(U)$。对 $J$ 做二阶 Taylor 展开并解 Newton 方程 $\nabla^2 J \cdot \delta U = -\nabla J$。由于 $J$ 的 Hessian 具有时间递归结构（每一步只依赖前后步），Newton 方程可以**从后向前逐步消去**——这个消去过程恰好是 DDP 的 backward pass（Pantoja 1988）。

因此：**DDP backward pass = 单射击 Newton 的 Riccati factorization**。

类比理解：如果把 Newton 方程视为一个 $Nm \times Nm$ 的大线性系统，DDP 不是"暴力"用 LU 分解（$O(N^3 m^3)$），而是利用时间结构做 block elimination（$O(Nm^3)$）——精神上与 FFT 利用频率结构把 DFT 从 $O(N^2)$ 降为 $O(N\log N)$ 完全一致。

### FDDP 是多射击 SQP 的 Riccati 求解

当我们转到多射击表述（把 $x_k$ 也作为决策变量，动力学作等式约束），NLP 变成：

$$
\min_{X, U} J(X, U) \quad \text{s.t.} \quad x_{k+1} = f(x_k, u_k)
$$

对这个 NLP 做一步 SQP（Newton on KKT），KKT 系统具有 block-tridiagonal 结构。用 Riccati 递推对 KKT 做 block elimination——得到的步方向与 FDDP 的 defect-corrected backward pass **完全等价**（Mastalli 2020 Proposition 2）。

$$
\text{FDDP backward pass} \equiv \text{Riccati condensation of multi-shooting KKT}
$$

这个等价性的深层意义：FDDP 不是 DDP 的 ad-hoc 修改，而是**严格的多射击 SQP 在 Bellman 语言下的重新表述**。defect 修正项 $V_x' \leftarrow V_x' + V_{xx}'\bar f$ 不是"补丁"，而是 KKT 中动力学残差吸收进值函数梯度的数学必然。

### iLQR 是单射击 Gauss-Newton SQP

把 OCP 代价写成最小二乘形式 $J = \frac{1}{2}\|r(U)\|^2$，对残差 $r$ 做一阶展开 $r(U + \delta U) \approx r(U) + J_r\,\delta U$，极小化 $\|r + J_r\delta U\|^2$ 得到 Gauss-Newton 方程 $J_r^\top J_r\,\delta U = -J_r^\top r$。

利用残差 Jacobian $J_r$ 的时间递归结构做 Riccati factorization——所得步方向与 iLQR backward pass 完全一致。

因此：**iLQR = 单射击 Gauss-Newton 的 Riccati 求解**。

### 统一谱系表

| 方法 | NLP 表述 | Hessian 近似 | KKT 求解 | 等价 |
|------|---------|-------------|---------|------|
| DDP | 单射击 | Exact Newton | Riccati | stagewise Newton |
| iLQR | 单射击 | Gauss-Newton | Riccati | stagewise GN |
| FDDP | 多射击 | GN + defect | Riccati condensation | condensed SQP |
| acados SQP | 多射击 | GN（显式） | HPIPM/Riccati | sparse SQP |
| IPOPT | 多射击 | Exact / L-BFGS | 稀疏 LDL | general NLP |

> **本质洞察**：DDP 家族和 acados 的区别不在数学（两者求解的是同一个 KKT 系统），而在代码组织：DDP 用 Bellman 语言（Q 函数、值函数），SQP 用 NLP 语言（Lagrangian、active set）。Bellman 语言的优势是**自然给出反馈增益 $K$**；NLP 语言的优势是**原生支持路径约束**。

### 收敛率的统一理解

从 SQP 框架看收敛率变得更透明：

| 近似策略 | NLP 对应 | 每步复杂度 | 局部收敛率 |
|---------|---------|-----------|-----------|
| Exact Hessian | Newton SQP | $O(N(n^3 + n^2m))$ | 二次 |
| GN Hessian | GN SQP | $O(N n^2 m)$ | 超线性（零残差二次） |
| Quasi-Newton (BFGS) | BFGS SQP | $O(N n^2)$ | 超线性 |
| Identity Hessian | 梯度法 | $O(N nm)$ | 线性 |

DDP 用 exact Hessian（含动力学 Hessian 张量），所以二次收敛。iLQR 用 GN Hessian，所以超线性。BFGS 介于两者之间——OCS2 内部的 SLQ 在某些模式下就用 BFGS 更新。梯度法（一阶 DDP，即只用 $Q_u$ 不用 $Q_{uu}$）线性收敛且极慢，除了理论分析外几乎不使用。

### 对 §3.10 约束 DDP 的预告

这个统一视角直接连接到下一章：

- Box-DDP = 单射击 Newton + box-constrained QP 子问题
- ALTRO = 多射击 GN SQP + 增广 Lagrangian 外层
- FDDP = 多射击 GN SQP 的 condensed Riccati 实现
- ProxDDP = 多射击 proximal SQP 的 Riccati 实现

所有约束 DDP 方法都可以在 SQP 框架下统一理解——区别仅在于约束处理方式和 KKT 求解策略。

---

## §3.9.14 DDP 的收敛性分析——严格数学处理 ⭐⭐⭐⭐

### DDP 的局部二次收敛定理

**定理（Liao-Shoemaker 1991, IEEE TAC）**：设 OCP 的最优解 $(x^\star, u^\star)$ 满足：
1. $f \in C^3$（动力学三次连续可微）
2. $\ell, \ell_f \in C^3$（代价三次连续可微）
3. 在 $(x^\star, u^\star)$ 处 $Q_{uu,k} \succ 0$ 对所有 $k$（二阶充分条件）
4. $\alpha = 1$ 在最优解邻域内被 Armijo 准则接受

则存在 $\delta > 0$ 和 $C > 0$，使得当 $\|U^{(j)} - U^\star\| < \delta$ 时：

$$
\|U^{(j+1)} - U^\star\| \le C\,\|U^{(j)} - U^\star\|^2
$$

其中 $U = (u_0, \ldots, u_{N-1})$ 是控制序列的堆叠向量。

**证明的关键步骤**：

Step 1. 把 DDP 的一次 backward+forward 看作从 $U^{(j)}$ 到 $U^{(j+1)}$ 的映射 $\Phi$：$U^{(j+1)} = \Phi(U^{(j)})$。

Step 2. 在 $U^\star$ 处做 Taylor 展开：$\Phi(U^\star + \delta U) = U^\star + D\Phi|_{U^\star}\,\delta U + O(\|\delta U\|^2)$。

Step 3. 证明 $D\Phi|_{U^\star} = 0$（即 DDP 在最优点的 Jacobian 为零）。这是二次收敛的特征——一阶项消失。

Step 4. $D\Phi = 0$ 的原因：在 $U^\star$ 处，$k_k = -Q_{uu}^{-1}Q_u = 0$（因为 $Q_u = 0$ 是一阶最优性条件）。forward pass 的修正 $\delta u = \alpha k + K\delta x$ 中，$\alpha k = 0$（开环修正为零），$K\delta x$ 也为零（因为 $\delta x$ 是前面步骤的 $k$ 通过动力学传播产生的，全部为零）。所以 $\Phi(U^\star) = U^\star$ 且 $D\Phi = 0$。

Step 5. 由 $D\Phi = 0$，$\|\Phi(U) - U^\star\| = O(\|U - U^\star\|^2)$。常数 $C$ 依赖于 $D^2\Phi$（Hessian of the iteration map），其有界性由 $f, \ell \in C^3$ 和 $Q_{uu} \succ 0$ 保证。

**为什么 iLQR 是超线性而非二次？**

iLQR 丢弃了 $V'_x \cdot f_{xx}$ 等项后，$Q$ 系数有 $O(\|V'_x\|)$ 的误差。在最优点 $V'_x \ne 0$（除非终端代价为零），所以 $D\Phi|_{U^\star} \ne 0$ 但 $\|D\Phi\| \propto \|V'_x^\star\|$。当残差趋零时 $\|V'_x^\star\| \to 0$，$D\Phi \to 0$——超线性收敛。

### iLQR 在非零残差下的收敛分析

**命题（Giftthaler et al. 2018）**：当 OCP 的最优残差 $\|r(U^\star)\| > 0$（如 $x_\text{ref}$ 不可达），iLQR 的收敛率退化为**线性**，收敛因子：

$$
\rho = \frac{\|V'_x \cdot f_{xx}\|}{\|Q_{uu}\|} \le \frac{\max_k \|V'_{k,x}\| \cdot \max_k \|f_{k,xx}\|}{\min_k \lambda_{\min}(Q_{uu,k})}
$$

当 $\rho < 1$ 时线性收敛；$\rho \ge 1$ 时 iLQR 可能不收敛（需正则化或切换为 full DDP）。

**工程含义**：对"可达问题"（$x_\text{ref}$ 可精确到达），$V'_x^\star \approx 0$，$\rho \approx 0$，iLQR 和 DDP 性能几乎一样。对"不可达问题"（如 $Q_f$ 权重小、轨迹不可精确跟踪），$V'_x^\star$ 较大，$\rho$ 可能接近 1，此时 iLQR 收敛变慢——但通常仍可接受（$\rho = 0.7$ 意味着每步误差缩减 30%）。

### 收敛率的实际测量方法

在工程实践中，如何判断你的 iLQR 实现是否达到了理论预期的收敛率？以下方法系统化了诊断流程：

**方法 1：误差比序列**

记录每次迭代的 $\|Q_u\|_\infty$（一阶最优性残差），计算相邻迭代的比率 $r_k = \|Q_u^{(k+1)}\|/\|Q_u^{(k)}\|$。

- **二次收敛**：$r_k \to 0$（比率趋零），且 $\|Q_u^{(k+1)}\| / \|Q_u^{(k)}\|^2 \to C$（有限常数）
- **超线性收敛**：$r_k \to 0$ 但 $\|Q_u^{(k+1)}\| / \|Q_u^{(k)}\|^2 \to \infty$（比率趋零但不够快）
- **线性收敛**：$r_k \to \rho \in (0, 1)$（比率趋常数）
- **不收敛**：$r_k \ge 1$

**方法 2：log-log 图**

在 log-log 坐标下画 $\log\|Q_u^{(k+1)}\|$ vs $\log\|Q_u^{(k)}\|$。斜率 $\approx 2$ 表示二次收敛，$> 1$ 但递增表示超线性，$= 1$ 表示线性。

**方法 3：自适应步长监控**

如果 $\alpha = 1$ 在后期迭代中连续被接受，且 Armijo 比率 $z \approx 1$，说明二次模型与实际代价面高度一致——这是二次/超线性收敛的工程标志。如果 $\alpha$ 持续很小，说明模型失效、收敛可能退化为线性。

### 全局收敛性（弱结果）

DDP 没有**全局**二次收敛保证——它依赖初始猜测落在最优解的 attraction basin 内。实际中通过以下手段扩大吸引盆：

1. **Line search + 正则化**：保证每步代价不增（全局下降性），防止发散
2. **MPPI/CEM warm-start**：用采样方法找到一个粗略的好初始猜测
3. **FDDP 多射击**：允许 infeasible 初始化，吸引盆更大
4. **课程学习**（curriculum）：从简单问题开始，逐步增加难度，每步用上一步的解作 warm-start

> **反事实推理**：如果 DDP 有全局收敛保证，它将能在 $O(N)$ 时间内解决任意非凸 OCP——这将使 NP-hard 问题变为 P，显然不可能。DDP 的局部收敛是对其高效性的必然代价。

### DDP 收敛性与 SQP 收敛性的桥梁

回顾 §3.9.13 的统一视角：DDP 等价于单射击 Newton SQP。因此 DDP 的收敛性可以直接从 SQP 的收敛理论导出。具体地：

- **DDP 的二次收敛** $\iff$ **Newton 法在 $C^2$ 函数上的二次收敛**（Kantorovich 定理的特化）
- **iLQR 的超线性收敛** $\iff$ **Gauss-Newton 在零/小残差最小二乘上的超线性收敛**（Dennis-Moré 定理的特化）
- **DDP + Armijo 的全局收敛** $\iff$ **阻尼 Newton + 充分下降条件的全局收敛**（Zoutendijk 定理的特化）

这意味着 DDP 的收敛理论并非"独立发明"——它继承了非线性优化的完整理论体系。理解这个桥梁，可以让我们在遇到新问题时直接借用 NLP 文献的分析工具，而非从头推导。

### ⚠️ 常见陷阱

🧠 **思维陷阱：用迭代次数判断算法"好坏"**

"iLQR 10 次收敛、DDP 6 次收敛，所以 DDP 更好"——这忽略了每次迭代的计算量。正确的比较维度是 **wall-clock time to tolerance**。对高维问题，iLQR 的 6 次 × 2 ms/次 = 12 ms 可能快于 DDP 的 4 次 × 5 ms/次 = 20 ms。

### 收敛率总结表

| 场景 | DDP 收敛率 | iLQR 收敛率 | 推荐选择 |
|------|-----------|------------|---------|
| 零残差（$x_\text{ref}$ 可达） | 二次 $O(\|e\|^2)$ | 近二次（$V'_x \to 0$） | iLQR（计算量低 3-5x） |
| 小残差（$x_\text{ref}$ 近可达） | 二次 | 超线性 | iLQR |
| 大残差（$x_\text{ref}$ 远不可达） | 二次 | 线性（$\rho \approx \|V'_x \cdot f_{xx}\|/\|Q_{uu}\|$） | DDP（若 $\rho$ 接近 1） |
| 接触/非光滑动力学 | 可能不定（$f_{xx}$ 噪声大） | 更稳（GN 自动 PSD） | iLQR |
| 长采样间隔（$\Delta t > 50$ ms） | 二次（曲率项有用） | 可能变慢 | DDP |
| MPC warm-start（已近最优） | 1 步（$\alpha = 1$） | 1 步（$\alpha = 1$） | iLQR（更快） |

这张表的核心信息：**在 95% 的机器人实时控制场景中，iLQR 是正确选择**。只有在"大残差 + 光滑动力学 + 有计算预算"的少数场景下，完整 DDP 才显著优于 iLQR。

### 练习

1. 对 cart-pole 的 iLQR 实现，记录每次迭代的 $\|U^{(j)} - U^{(j-1)}\|$，绘制 log-scale 图。观察是否呈现超线性下降（斜率递增）。
2. 构造一个"非零残差"问题（把 $x_\text{ref}$ 设为不可达状态），观察 iLQR 的收敛率是否退化为线性。计算实际 $\rho$ 并与理论估计对比。
3. 解释为什么 DDP 的收敛证明要求 $f \in C^3$ 而非 $C^2$。（提示：$D^2\Phi$ 的有界性需要动力学的三阶导数有界。）

---

## §3.9.15 DDP 在非欧状态空间上的推广 ⭐⭐⭐

### 动机：为什么需要李群 DDP

浮动基座机器人的配置空间包含 $SE(3)$（或其紧致表示 $SO(3) \times \mathbb{R}^3$），状态空间是 $T(SE(3) \times \mathbb{R}^{n_j})$。四元数/旋转矩阵不能做欧氏减法——如果强行 $x_1 - x_2$，会破坏四元数的单位约束、旋转矩阵的正交约束，产生虚假的 defect。

### 李群 DDP 的核心修改

在流形 $\mathcal{M}$（如 $SE(3)$）上，标准 DDP 的三个操作需要替换：

| 欧氏空间操作 | 李群替换 | 说明 |
|------------|---------|------|
| $\delta x = x - \bar x$ | $\delta\xi = x \ominus \bar x = \log(\bar x^{-1} \circ x)$ | 切空间差分 |
| $x + \delta x$ | $x = \bar x \oplus \delta\xi = \bar x \circ \exp(\delta\xi)$ | 指数映射积分 |
| $f_x = \partial f/\partial x$ | $f_\xi = J_\text{right/left}^{-1} \cdot \partial f/\partial x$ | Jacobian 需经 $J_\log$ 修正 |

Pinocchio 的 `StateMultibody` 类和 Crocoddyl 的 `state.diff` / `state.integrate` 自动处理这些替换。用户只需确保**不手动做欧氏减法**。

### 对 Q 系数递推的影响

$\delta x$ 现在是切空间向量 $\delta\xi \in \mathfrak{g}$，维度仍为 $n$（如 $SE(3)$ 的切空间是 6 维）。Q 系数的公式**形式不变**——只是所有 Jacobian 需要乘以 $J_\log$（对数映射的 Jacobian）来补偿流形曲率。

对 iLQR 用户来说，**只要使用 Pinocchio + Crocoddyl，这些修正是透明的**——库内部自动处理。但如果手写 iLQR，必须注意：

1. Defect 计算用 `state.diff` 而非减法
2. Forward pass 用 `state.integrate` 而非加法
3. 代价函数的残差用 $\log_{SE(3)}$ 而非欧氏距离

### ⚠️ 常见陷阱

⚠️ **编程陷阱：对四元数状态做 $q_1 - q_2$**

四元数减法 $(q_1 - q_2)$ 不是旋转差——它甚至不在单位四元数流形上。正确做法：$\delta\omega = 2 \cdot \text{Im}(q_2^{-1} \cdot q_1)$（轴角形式的旋转差）。在 Pinocchio 中：`pinocchio.difference(model, q1, q2)`。

### 练习

1. 对浮动基座的 12-DOF 四旋翼（状态含四元数），解释为什么 iLQR 的 $\delta x$ 维度是 12 而非 13（四元数 4D 但切空间 3D）。
2. 推导 $SO(3)$ 上的 $J_\log$（即 $\log$ 映射的 Jacobian），并说明当旋转角 $\theta \to 0$ 和 $\theta \to \pi$ 时的行为。

---

## §3.9.16 计算示例：Cart-Pole 摆起的完整数值流程 ⭐⭐

### 问题设置

```
系统：Cart-Pole（小车+倒摆）
状态：x = [位置, 角度, 速度, 角速度] ∈ ℝ⁴
控制：u = [水平力] ∈ ℝ¹
参数：m_cart=1 kg, m_pole=0.1 kg, l=1 m, g=9.81 m/s²
目标：从 x₀=[0, π, 0, 0]（下垂）摆到 x_goal=[0, 0, 0, 0]（直立）
时域：T=2 s, Δt=20 ms, N=100
代价：ℓ = 0.5 u² × 0.01,  ℓ_f = 0.5 (x-x_goal)ᵀ Q_f (x-x_goal)
      Q_f = diag(100, 100, 10, 10)
控制限制：|u| ≤ 30 N
```

### 迭代过程展示

```
Iter |    J     | ||Qu||∞ |   α   |   μ    | 活动集
-----|----------|---------|-------|--------|--------
  1  | 1234.56  | 89.3    | 0.125 | 1e-6   | {0,1,...,30}
  2  |  567.89  | 45.1    | 0.25  | 1e-6   | {5,6,...,25}
  3  |  234.12  | 23.7    | 0.5   | 0      | {10,...,20}
  4  |  123.45  | 12.1    | 1.0   | 0      | {12,13,14}
  5  |   89.67  |  5.4    | 1.0   | 0      | {13}
  6  |   78.34  |  2.1    | 1.0   | 0      | {}
  7  |   76.89  |  0.8    | 1.0   | 0      | {}
  8  |   76.54  |  0.3    | 1.0   | 0      | {}
  9  |   76.51  |  0.05   | 1.0   | 0      | {}
 10  |   76.50  |  0.002  | 1.0   | 0      | {}（收敛）
```

**观察**：
1. 前 3 次迭代步长小（$\alpha < 1$），因为二次模型远离真实
2. 第 4 次开始 $\alpha = 1$（进入 Newton 收敛域）
3. 活动集从 30+ 个维度逐步减少到 0——说明最终最优解不饱和
4. 最后 4 次迭代 $\|Q_u\|$ 指数下降：$5.4 \to 2.1 \to 0.8 \to 0.3 \to 0.05$（超线性）

### 物理直觉

最优轨迹的物理策略：先向左猛推（积累动量）→ 在合适时机反向推（让摆杆甩起来）→ 直立附近精细平衡。这是一个"能量泵注"过程——初期力矩接近饱和（活动集非空），直立附近力矩很小（无活动约束）。

### 反馈增益 $K_0$ 的使用

收敛后 $K_0 \in \mathbb{R}^{1\times 4}$ 是第一步的反馈增益。在 MPC 中，如果求解器来不及重新规划，可以直接用：

$$
u_\text{applied} = u_0^\star + K_0(x_\text{measured} - x_0^\star)
$$

这提供了**亚毫秒级**的反馈响应——比重新跑 iLQR 快 1000 倍。

### 典型调参经验

| 参数 | 推荐初始值 | 调节方向 | 影响 |
|------|-----------|---------|------|
| $Q$ (state weight) | $\text{diag}(1,\ldots,1)$ | 按物理单位归一化 | 状态跟踪精度 |
| $R$ (control weight) | $0.01 \cdot I$ | 越小→力矩越大→可能饱和 | 能量消耗 vs 速度 |
| $Q_f$ (terminal) | $100 \times Q$ 或 LQR $P$ | 过大→局部极小；过小→terminal miss | 终端精度 |
| $\Delta t$ | 10-20 ms | 太大→线性化差；太小→$N$ 膨胀 | 精度 vs 计算量 |
| $N$ (horizon) | $T / \Delta t$，$T = 1$-$2$ s | 覆盖 2-3 个时间常数 | 视域足够性 |
| $\mu_0$ (reg init) | $10^{-6}$ 或 0 | 仅在失败时增大 | 鲁棒性 vs 速度 |
| $\Delta_0$ (reg factor) | 2 | 越大→$\mu$ 变化越剧烈 | 恢复速度 |
| $\varepsilon_J$ (cost tol) | $10^{-7}$ | 越小→迭代越多 | 解精度 |
| $\varepsilon_g$ (grad tol) | $10^{-4}$ | 越小→迭代越多 | 一阶最优性 |

**调参策略**：先把 $R$ 设得较大（如 $R = I$），确保 iLQR 收敛（不饱和）；然后逐步减小 $R$，让力矩增加直到接近物理限制。如果需要更激进的动作，加 Box-DDP 而非继续减小 $R$。

### 常见失败模式诊断

```
症状：代价不下降但 ||Qu|| 也不减
诊断：正则化 μ stuck 在高值
处方：手动设 μ=0 跑一步看是否改善

症状：α 只能取 2^{-10}
诊断：二次模型严重偏离真实
处方：(a) 减小 Δt (b) 增大 N (c) 检查动力学导数正确性

症状：前几步 J 下降很快然后 plateau
诊断：进入局部极小
处方：(a) 多初始点 (b) MPPI warm-start (c) 增大 Qf 拉向目标

症状：收敛后闭环执行差
诊断：K 矩阵在奇异姿态附近不稳
处方：(a) 增大 R 使 K 更保守 (b) 用 Box-DDP 限制力矩 (c) MPC 频率不够高
```

### 与其他优化方法的计算量对比

对 $n=14, m=7, N=50$ 的典型 7-DoF reaching 问题：

| 方法 | 变量数 | 每次迭代 | 典型迭代数 | 总 wall-clock |
|------|--------|---------|-----------|-------------|
| iLQR (Pinocchio) | 350 | 0.5 ms | 15 | 7.5 ms |
| iLQR (有限差分) | 350 | 5 ms | 15 | 75 ms |
| DDP (Pinocchio) | 350 | 1.2 ms | 10 | 12 ms |
| IPOPT (DirCol) | 1050+700 约束 | 20 ms | 50 | 1000 ms |
| acados (RTI) | 1050 | 0.3 ms | 1 | 0.3 ms |

**结论**：iLQR + Pinocchio 解析导数 = ms 级求解，比通用 NLP 快 100x，比 acados RTI 慢 10x（但 acados 是 direct method 不给 K）。

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| iLQR 和 DDP 是完全不同的算法 | iLQR 是 DDP 的 Gauss-Newton 近似——在 backward pass 中丢弃 $V_{xx}$ 的二阶项 $f_{xx}^\top V_x$；两者共享相同的 Bellman 递推结构，仅 $Q_{uu}$ 的 Hessian 近似方式不同 |
| DDP 是全局最优算法 | DDP/iLQR 本质是牛顿法（或 GN 法），只能找到局部最优；不同初始轨迹可能收敛到不同局部极小，实践中需要多初始点或 MPPI warm-start |
| 正则化参数 $\mu$ 是固定超参数 | $\mu$ 需要自适应调整——backward pass $Q_{uu}$ 不正定时增大（Levenberg-Marquardt 策略），line search 成功后减小；固定 $\mu$ 会导致收敛缓慢或不收敛 |
| DDP 比 direct method（直接配点法）更优 | DDP 利用 Bellman 结构实现 $O(Nm^3)$ 复杂度，适合无约束或简单约束；但 direct method + 稀疏 NLP solver 对复杂约束更灵活，两者互补 |
| 有限差分计算导数足够用 | 有限差分引入 $O(\epsilon + h^2)$ 的截断/舍入误差，且计算量是解析导数的 $O(n+m)$ 倍；Pinocchio 的解析导数在精度和速度上均远优于有限差分 |
| iLQR 的 forward pass 只是简单积分 | forward pass 是非线性 rollout + 沿 $\alpha$ 的 line search——步长 $\alpha$ 的选择直接影响收敛率；贪心地取 $\alpha=1$ 在远离最优时常导致发散 |
| DDP 只能处理欧氏状态空间 | 李群 DDP 已将算法推广到 $SO(3)$、$SE(3)$ 等流形——关键修改是用 $\oplus/\ominus$ 替代加减法，用流形上的 Jacobian 替代欧氏导数 |

---
