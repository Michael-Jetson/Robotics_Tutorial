# 专题 3.10：约束 DDP 家族与 Crocoddyl ⭐⭐

> 博士前数学路线图 · 第三批 · 专题 3.10
> 前置：3.9 DDP/iLQR 原理与实现、3.5 LQR 与 Riccati 方程、3.3 Bellman 方程
> 面向：机器人算法工程师（whole-body MPC、腿足/操作规划），C++/Python

---

## 前置自测

📋 **前置自测**（答不出 >=2 题 → 先回 3.9/3.5 复习）

1. 写出 iLQR backward pass 的 Q 函数六系数（$Q_x, Q_u, Q_{xx}, Q_{ux}, Q_{uu}$），说明每个系数的物理含义。
2. Tassa 2012 的状态正则化 $V'_{xx} + \mu I$ 为什么优于控制正则化 $Q_{uu} + \mu I$？
3. DDP 收敛时 $Q_u = 0$ 对应什么一阶最优性条件？
4. 增广拉格朗日方法（ALM）的基本思想是什么？它比纯二次罚的优势何在？
5. Armijo line search 的两个参数 $c_1, c_2$ 分别控制什么？

---

## 本章目标

学完本专题后，你应当能够：
- 理解约束如何破坏 DDP 的解析极小化，以及四大修复策略（Box/AL/FDDP/Prox）
- 独立推导 Box-DDP 的 Projected Newton 活动集算法
- 掌握 ALTRO 的 AL-iLQR + Projected Newton 两阶段架构及其收敛理论
- 理解 FDDP 的 defect 修正如何等价于多射击 SQP
- 读懂 Crocoddyl 源码的 Action Model 抽象与 Solver 选择逻辑
- 了解 ProxDDP/Aligator 的 primal-dual 近端策略

---

## 知识树概览

```
约束 DDP 知识树
├── 根：约束破坏了 DDP 的解析极小化 δu* = -Q_{uu}^{-1}Q_u
│   ├── 主干 1：Box-DDP（Tassa 2014）
│   │   ├── 枝：Projected Newton 活动集
│   │   ├── 枝：自由/夹紧分块 Cholesky
│   │   └── 枝：K[clamped] = 0 的物理含义
│   ├── 主干 2：AL-iLQR / ALTRO（Howell 2019）
│   │   ├── 枝：增广拉格朗日函数 ℒ_A
│   │   ├── 枝：外层乘子更新 + 罚因子递增
│   │   ├── 枝：Projected Newton 精炼阶段
│   │   └── 枝：锥约束扩展（ALTRO-C）
│   ├── 主干 3：FDDP / Box-FDDP（Mastalli 2020/2022）
│   │   ├── 枝：defect / gap 的定义与传播
│   │   ├── 枝：(★) defect 修正公式
│   │   ├── 枝：ℓ₁ merit 与 gap 线性收缩
│   │   └── 枝：与多射击 SQP 的精确等价
│   ├── 主干 4：ProxDDP / Aligator（Jallet 2025）
│   │   ├── 枝：近端点算法嵌入 Bellman
│   │   ├── 枝：primal-dual 乘子同步更新
│   │   └── 枝：硬等式/不等式/SOC 原生支持
│   └── 主干 5：Crocoddyl 框架
│       ├── 枝：ActionModel / DAM / IAM 抽象
│       ├── 枝：ShootingProblem + Solver 家族
│       ├── 枝：CostModelSum + Residual + Activation
│       └── 枝：Pinocchio 解析导数后端
└── 应用分支
    ├── 多接触运动规划（OCP 建模）
    ├── 摩擦锥约束处理
    ├── 终端约束与 MPC 递归可行性
    └── Warm-start 与实时部署
```

---

## §3.10.1 约束轨迹优化的标准形式与约束分类 ⭐⭐

### 动机：为什么需要约束

回顾 §3.9 的无约束 DDP：backward pass 得到 $\delta u^\star = -Q_{uu}^{-1}Q_u$，这是一个**无约束**二次极小化的闭式解。但真实机器人必须满足：

- 关节力矩不能超过电机极限（$|u_i| \le \bar\tau_i$）
- 关节角度不能超过机械限位（$\underline q \le q \le \bar q$）
- 接触力必须在摩擦锥内（$\sqrt{f_x^2+f_y^2} \le \mu f_n$）
- 接触点速度为零（$J_c\dot q = 0$）
- 终端状态必须到达目标（$x_N = x_\text{goal}$）

任何一条约束都会使 $\delta u^\star = -Q_{uu}^{-1}Q_u$ 这个漂亮的解析公式失效。我们需要在 Bellman 递推的框架内，找到处理约束的方法。

> **本质洞察**：约束 DDP 的所有方法，本质上都在做同一件事——**在每步 backward pass 的子问题中用某种方式处理约束**，同时保持 $O(N)$ 的时间复杂度结构。区别只在于"处理约束的具体手段"和"约束信息如何在时间步之间传递"。

类比：无约束 DDP 像在一张白纸上自由画线找最短路径；约束 DDP 像在一座有墙的迷宫中找路——你仍然用相同的"局部二次近似"思想（每步看脚下地形），但现在走到墙边时要么贴墙走（Box-DDP 投影），要么在墙上加弹簧把自己弹回来（AL 罚），要么记住墙的位置提前绕路（ProxDDP）。

### 标准约束离散 OCP

$$
\min_{\{x_k\},\{u_k\}} \; \ell_N(x_N) + \sum_{k=0}^{N-1} \ell_k(x_k, u_k)
$$
$$
\text{s.t.}\;\; x_{k+1} = f_k(x_k, u_k),\quad g_k(x_k, u_k) \le 0,\quad h_k(x_k, u_k) = 0,\quad \underline u \le u_k \le \bar u,\quad x_N \in \mathcal X_f.
$$

若状态 $x$ 位于李群（如浮动基 $SE(3)\times\mathbb R^{n_j}$），动力学等式应理解为 $x_{k+1} = x_k \oplus f_c(x_k, u_k)\Delta t$，差分算子 $\ominus$ 把切空间向量带回 $\mathfrak{se}(3)$。

### 约束分类

| 约束类型 | 数学形式 | 机器人实例 | 对 DDP 的挑战 | 处理方法 |
|---|---|---|---|---|
| Box（简单界） | $\underline u \le u \le \bar u$ | 关节扭矩极限 | 解被边界截断 | Box-DDP 投影 |
| 线性等式 | $Ax + Bu = b$ | 接触零速度 $J_c\dot q = 0$ | 降维到零空间 | 零空间投影 / AL |
| 非线性不等式 | $g(x,u) \le 0$ | 障碍物 SDF、关节限位 | 需外层乘子更新 | AL / IPM / ProxDDP |
| 二阶锥 | $\|f_t\| \le \mu f_n$ | 库仑摩擦锥 | SOCP 或多面体近似 | ALTRO-C / ProxDDP |
| 互补 | $0 \le \lambda \perp \phi(q) \ge 0$ | 接触法向冲量 | 非光滑 | MPCC 松弛 |
| 终端 | $x_N = x_\text{goal}$ | 站稳、门到门 | MPC 稳定性核心 | 硬等式 / 大权重 |

### 核心命题：约束破坏了 DDP 的解析极小化

**命题 3.10.1** 设第 $k$ 步 Bellman 算子为 $V_k(x) = \min_u \{\ell_k(x,u) + V_{k+1}(f_k(x,u)) \mid g_k(x,u) \le 0\}$。则在活动集 $\mathcal A_k = \{i : g_k^i(x,u^\star) = 0\}$ 下，最优 $\delta u^\star$ 满足 **KKT 系统**：

$$
\begin{bmatrix} Q_{uu} & G_{\mathcal A}^\top \\ G_{\mathcal A} & 0 \end{bmatrix}
\begin{bmatrix} \delta u^\star \\ \lambda_{\mathcal A} \end{bmatrix}
= -\begin{bmatrix} Q_u \\ g_{\mathcal A} \end{bmatrix}
$$

而不再是 $\delta u^\star = -Q_{uu}^{-1}Q_u$。其中 $G_{\mathcal A} = \partial g_{\mathcal A}/\partial u$ 是活动约束的 Jacobian。

**证明**：

Step 1. 对带约束子问题写 Lagrangian：
$$
\mathcal{L}(\delta u, \lambda) = \frac{1}{2}\delta u^\top Q_{uu}\delta u + Q_u^\top\delta u + \lambda^\top g_{\mathcal A}(\bar u + \delta u)
$$

Step 2. KKT 驻点条件：$\nabla_{\delta u}\mathcal{L} = Q_{uu}\delta u + Q_u + G_{\mathcal A}^\top\lambda = 0$

Step 3. 原始可行性：$g_{\mathcal A}(\bar u + \delta u) \approx g_{\mathcal A} + G_{\mathcal A}\delta u = 0$（活动约束在边界上）

Step 4. 联立 Step 2 和 Step 3 得到上述 KKT 线性系统。

Step 5. 当 $\mathcal A = \varnothing$（无活动约束）时，$G_{\mathcal A}$ 为空，系统退化为 $Q_{uu}\delta u^\star = -Q_u$，即无约束 DDP。

### 四大约束处理策略概览

| 策略 | 核心思想 | 代表方法 | 复杂度变化 | 约束类型 |
|------|---------|---------|-----------|---------|
| 投影 | 在边界上做 projected Newton | Box-DDP (Tassa 2014) | $O(Nm^3)$ 同阶 | 仅 box |
| 外层乘子 | AL 外循环 + 无约束 iLQR 内层 | ALTRO (Howell 2019) | $O(N(n+m+p)^3)$ | 任意 |
| 动力学松弛 | 允许 gap（infeasible）+ merit | FDDP (Mastalli 2020) | $O(Nm^3)$ 同阶 | 动力学等式 |
| 近端 primal-dual | proximal AL 嵌入 Riccati | ProxDDP (Jallet 2025) | $O(N(n+m)^3)$ | 等式/不等式/锥 |

### 与前后专题桥梁

- **承 §3.9**：无约束 DDP 的 $Q$ 系数递推和正则化策略在此**原样保留**，只有 $\delta u$ 的极小化被替换
- **启 §3.11**：终端约束 $x_N \in \mathcal X_f$ 和终端代价 $\ell_N$ 是 MPC 递归可行性与渐近稳定性的前提（Mayne 2000 四条件）

### ⚠️ 常见陷阱

💡 **概念误区：认为"加个 clip 就行"**

新手直觉：DDP 照常求 $\delta u^\star = -Q_{uu}^{-1}Q_u$，把最终控制 $\hat u = u + \alpha k + K\Delta x$ 做逐元素 clip 到 $[\underline u, \bar u]$。Tassa 2014 指出三大缺陷：(i) clip 后 $Q_u^\top\delta u$ 不再保证为负，方向不再下降；(ii) 求逆时忽略边界耦合项 $Q_{fc}\delta u_c$；(iii) 反馈 $K$ 行在活动约束上本应置零，否则闭环振荡。

🧠 **思维陷阱：认为"约束越多越难"**

约束的数量不是关键——关键是**活动约束的数量和识别**。100 个 box 约束中只有 3 个活动，则 Box-DDP 的额外计算量极小（只多 3 次 1D 投影）。反之，若约束全部活动（如饱和操作），等价于完全锁定控制维度，问题退化。

### 练习

1. 对一维系统 $x_{k+1} = x + u$，$\ell = \frac{1}{2}(x^2+u^2)$，$|u| \le 1$。手写 KKT 系统并求解 $\delta u^\star$。验证当 $|Q_{uu}^{-1}Q_u| \le 1$ 时退化为无约束解。
2. 解释为什么 $\tanh$ squashing $u = \bar u \cdot \tanh(v)$ 处理控制界是一个坏主意。（提示：饱和区 $\partial u/\partial v \to 0$，$Q_{uu}$ 变成什么？）

---

## §3.10.2 Box-DDP：Tassa-Mansard-Todorov ICRA 2014 ⭐⭐

### 动机与历史

Box 约束（$\underline u \le u \le \bar u$）是机器人控制中最普遍的约束——每个关节电机都有力矩/电流极限。Tassa、Mansard 和 Todorov 在 2014 ICRA 上提出了一种优雅的方法：**把 backward pass 的每步无约束极小化替换为 box-constrained QP**，然后用 Projected Newton 活动集算法高效求解。

**为什么不直接用通用 QP solver？** 因为每步的 QP 只有 $m$ 个变量（控制维度，典型 7-30），且 box 约束有特殊结构——投影算子只是逐元素 clamp。通用 QP solver（如 OSQP、ProxQP）的 setup 开销远超 Box-DDP 的 inline Projected Newton。

### 严格定义

在第 $k$ 步 backward pass，把无约束极小化 $\delta u^\star = -Q_{uu}^{-1}Q_u$ 替换为 **box-QP**：

$$
\min_{\delta u}\; \frac{1}{2} \delta u^\top Q_{uu}\delta u + Q_u^\top \delta u \quad \text{s.t.}\quad \underline u - \bar u_k \le \delta u \le \bar u - \bar u_k
$$

记边界为 $\ell_i = \underline u_i - \bar u_{k,i}$（下界扰动）和 $b_i = \bar u_i - \bar u_{k,i}$（上界扰动）。

定义**活动集**（clamped set）$\mathcal C = \{i : \bar u_i + \delta u_i^\star \in \{\underline u_i, \bar u_i\}\}$，**自由集** $\mathcal F = \mathcal C^c$。

### Projected Newton 活动集算法

**核心思想**：对自由维度做 Newton，对活动维度锁定到边界。

**Step 1：分块**

将 $Q_{uu}$ 按 $(\mathcal F, \mathcal C)$ 分块：

$$
Q_{uu} = \begin{pmatrix} Q_{ff} & Q_{fc} \\ Q_{cf} & Q_{cc} \end{pmatrix}, \quad Q_u = \begin{pmatrix} Q_{u,f} \\ Q_{u,c} \end{pmatrix}
$$

**Step 2：自由块 Newton 方向**

活动（夹紧）维度锁定到边界：$\delta u_c = \text{clip}(\delta u_c, \ell_c, b_c)$。

对自由维度，解 Newton 方程：

$$
Q_{ff}\,\delta u_f = -(Q_{u,f} + Q_{fc}\,\delta u_c)
$$

用 Cholesky 分解 $Q_{ff} = L_f L_f^\top$，解 $\delta u_f = -Q_{ff}^{-1}(Q_{u,f} + Q_{fc}\delta u_c)$。

**为什么是 $Q_{ff}$ 而不是 $Q_{uu}$？** 因为活动约束已经"消耗"了 $|\mathcal C|$ 个自由度——在这些维度上，$\delta u$ 的值由边界决定（不由梯度决定）。优化只发生在剩余的 $|\mathcal F| = m - |\mathcal C|$ 个自由维度上。

**Step 3：投影 line-search**

$$
\delta u \leftarrow \Pi_{[\ell, b]}(\bar\delta u + \alpha\,d) - \bar\delta u
$$

其中 $\Pi_{[\ell,b]}$ 是逐元素投影（clamp），$d$ 是 Newton 方向。这个投影 line-search 允许活动集在单次回溯中增/减——某些维度可能在步长 $\alpha$ 下从自由变为夹紧（触碰边界），或从夹紧变为自由（离开边界）。

**Step 4：反馈增益分块置零**

$$
K_k[\mathcal C, :] = 0, \qquad K_{k,\mathcal F} = -Q_{ff}^{-1}Q_{\mathcal F x}
$$

**为什么活动行置零？** 物理含义：如果第 $i$ 个控制已经饱和在边界上，那么无论状态如何偏离（$\delta x \neq 0$），该控制都不应改变——它已经"顶到天花板"了。如果不置零，反馈项 $K_i\delta x$ 会试图修改已饱和的控制，导致闭环振荡。

### 算法收敛性

**定理 3.10.2（Bertsekas 1982 + Tassa 2014 特化）** 设 $Q_{uu}\succ 0$，解 $\delta u^\star$ 满足严格互补条件（活动约束处乘子 $> 0$）与二阶充分条件。则存在 $\delta u^\star$ 的邻域使得 $\varepsilon$-活动集 Projected Newton 方法在有限步内正确辨识 $\mathcal C^\star$，其后等价于自由子空间纯 Newton，具有**局部 q-二次收敛**。

**严格互补条件的含义与检验**

严格互补要求每个活动约束的 Lagrange 乘子严格为正：$\lambda_i^\star > 0$ for all $i \in \mathcal{C}^\star$。如果某个活动约束的乘子 $\lambda_i^\star = 0$（**退化互补**），则活动集辨识变得困难——微小扰动可能导致该约束在活动与非活动之间反复切换（chattering）。

在 Box-DDP 的上下文中，乘子可以从 KKT 条件直接读出。对上界活动的约束 $\delta u_i = b_i$：

$$
\lambda_i = -(Q_{uu}\delta u + Q_u)_i > 0 \quad (\text{严格互补})
$$

如果 $\lambda_i$ 很小但正（比如 $\lambda_i = 10^{-8}$），虽然技术上满足严格互补，但数值上可能导致活动集在相邻迭代间振荡。工程解决方案是使用 $\varepsilon$-活动集定义：把 $\lambda_i < \varepsilon$ 的约束视为"准退化"，用特殊处理（如 warm-start 锁定）避免振荡。

**从活动集辨识到二次收敛的完整推导**

一旦 $\mathcal{C}^\star$ 被正确辨识，约束 QP 退化为自由子空间上的无约束 QP：

$$
\min_{\delta u_{\mathcal{F}}} \frac{1}{2} \delta u_{\mathcal{F}}^\top Q_{ff} \delta u_{\mathcal{F}} + (Q_{u,f} + Q_{fc}\delta u_c)^\top \delta u_{\mathcal{F}}
$$

其中 $\delta u_c$ 已由边界固定。这个子问题的 Newton 步就是 $\delta u_{\mathcal{F}} = -Q_{ff}^{-1}(Q_{u,f} + Q_{fc}\delta u_c)$——精确一步到位。

在外层 DDP 迭代中，$Q_{ff}$ 和 $Q_{u,f}$ 每步都在变化（因为轨迹在更新），但在最优解附近变化是 Lipschitz 的。标准 Newton 收敛理论（Kantorovich 定理）保证了自由子空间上的 q-二次收敛。

**直觉解释**：一旦活动集被正确识别（知道哪些维度在边界上），问题退化为 $|\mathcal F|$ 维的无约束 Newton——自然二次收敛。难点在于"辨识"本身，但严格互补保证了活动约束"显著"（乘子远离零），连续性保证了邻域内活动集不变。

**实际表现**：对典型机器人问题（$m = 12$-$30$ 维控制），box-QP 通常只需 1-2 次 Projected Newton 迭代（即 1-2 次 Cholesky 分解）即收敛，因为前一步的活动集是极好的 warm-start。

### 完整伪代码

```
function BOX-DDP(x0, U, N)
  X ← rollout(x0, U)
  J ← cost(X, U)
  μ ← 0; Δ ← 1
  
  repeat
    # === Backward pass ===
    Vx, Vxx ← terminal_derivatives(X[N])
    for k = N-1 .. 0:
      compute Qx, Qu, Qxx, Qux, Quu      # 同 §3.9 公式（含正则化）
      
      # Box-QP 替代无约束极小化
      (kk[k], KK[k], C[k]) ← BoxQP(Quu, Qu, Qux, lb[k], ub[k]; warm=C_prev[k])
      KK[k][C[k], :] ← 0                   # 活动行反馈置零
      
      # 值函数递推（同 §3.9）
      Vx  ← Qx + KK^T Quu kk + KK^T Qu + Qux^T kk
      Vxx ← Qxx + KK^T Quu KK + KK^T Qux + Qux^T KK
      Vxx ← symmetrize(Vxx)
    
    # === Forward pass (projected line search) ===
    for α in [1, 0.5, 0.25, ..., 2^{-10}]:
      X̂[0] = x0
      for k = 0..N-1:
        Û[k] = clip(U[k] + α*kk[k] + KK[k]@(X̂[k]-X[k]), lb_abs, ub_abs)
        X̂[k+1] = dynamics(X̂[k], Û[k])
      Ĵ = cost(X̂, Û)
      if Armijo_OK(J, Ĵ, expected_decrease, α):
        accepted = true; break
    
    if accepted:
      X ← X̂; U ← Û; J ← Ĵ
      decrease_regularization(μ, Δ)
    else:
      increase_regularization(μ, Δ)
    
  until converged(||kk||, |ΔJ|)
  return X, U, KK
```

### BoxQP 子函数

```
function BoxQP(H, g, Hux, lb, ub; warm_C):
  # H = Quu ∈ ℝ^{m×m}, g = Qu ∈ ℝ^m
  # 初始化活动集为 warm_C
  δu ← clip(-H^{-1}g, lb, ub)   # 初始猜测
  
  for iter = 1..max_iter:
    C ← {i : δu_i == lb_i or δu_i == ub_i}  # 活动集
    F ← complement(C)
    
    # 自由块 Newton
    H_ff ← H[F,F]
    g_f ← g[F] + H[F,C] @ δu[C]
    L ← cholesky(H_ff)
    δu_f_new ← -cho_solve(L, g_f)
    
    # 检查自由块解是否在 box 内
    δu[F] ← clip(δu_f_new, lb[F], ub[F])
    
    # 收敛检查：gradient projected onto feasible directions
    grad ← H @ δu + g
    grad_proj ← 0
    for i in F: grad_proj += grad[i]^2
    for i in C:
      if δu[i] == lb[i] and grad[i] > 0: grad_proj += grad[i]^2
      if δu[i] == ub[i] and grad[i] < 0: grad_proj += grad[i]^2
    
    if sqrt(grad_proj) < tol: break
  
  # 计算增益
  kk ← δu
  KK_F ← -cho_solve(L, Hux[F,:])
  KK_C ← zeros
  KK ← stack(KK_F, KK_C, indexed by F, C)
  return kk, KK, C
```

### 为什么 naive clipping 会毁掉收敛

Tassa 2014 §III.A 详细分析了三种"简单"处理方式的缺陷：

| 方法 | 描述 | 失败原因 |
|------|------|---------|
| Post-hoc clip | DDP 正常跑，最后 clip $\hat u$ | 方向不下降、$K$ 行未置零致振荡 |
| $\tanh$ squashing | $u = \bar u\tanh(v)$，对 $v$ 做 DDP | 饱和区 $Q_{uu} \to 0$，Hessian 奇异 |
| Barrier / log | 在边界加 $-\mu\log(\bar u - u)$ | 远离边界时梯度微弱，近边界时 ill-cond |

> **反事实推理**：如果 naive clip 可行，Tassa 2014 这篇论文就不会存在——之前 10 年所有人都在 clip，但实际工程中观察到的是"DDP 在力矩饱和任务上不收敛或收敛极慢"。Box-DDP 通过正确的活动集处理，使力矩饱和场景的收敛速度从"不收敛"变为"<20 次迭代"。

### 工程含义

- 每时刻 box-QP 典型只需 **1-2 次 Cholesky 再分解**，总复杂度 $\mathcal O(N m^3)$，与无约束 DDP **同阶**
- 必须 warm-start 上一步的活动集 $\mathcal C_{\text{prev}}$，利用 MPC 的时序相似性
- **仅支持控制箱**——状态约束、摩擦锥约束需交给 §3.10.3-3.10.4 的 AL 或 ProxDDP

### ⚠️ 常见陷阱

⚠️ **编程陷阱：对 box-DDP 忘把夹紧行 $K_{\mathcal C}$ 置零**

完整 $K$ 做前向反馈会在边界附近 chattering。物理原因：如果电机已经在最大力矩，状态偏差来了，反馈试图加更多力矩——但 clip 会截断，下一步又减小——形成振荡。$K[\mathcal C,:]=0$ 告诉控制器："这个维度已经尽力了，别再管它。"

💡 **概念误区：认为 box-QP 子问题很贵**

对 $m = 12$（四足机器人），box-QP 是 $12\times 12$ 的 Cholesky（$\sim 300$ flops）+ 几次 clamp 检查。比动力学导数计算快 1000 倍。box-QP 不是瓶颈。

### 练习

1. 对 $m=2$ 的简单 box-QP：$Q_{uu} = \begin{pmatrix}2&1\\1&3\end{pmatrix}$，$Q_u = (1, -2)^\top$，$-1 \le \delta u \le 1$。手算无约束解、活动集、有约束解。
2. 实现 BoxQP 函数并在 cart-pole 的 iLQR 中替换无约束极小化。对比 naive clip vs Box-DDP 的收敛曲线。
3. 解释：为什么 Box-DDP 在活动集频繁变化时（如交替饱和/不饱和）收敛变慢？ALTRO 如何避免此问题？

---

## §3.10.3 增广拉格朗日 iLQR 与 ALTRO ⭐⭐

### 动机

Box-DDP 只能处理**控制箱约束**。但真实轨迹优化还有：
- 状态约束：关节限位 $\underline q \le q \le \bar q$、避障 $\|x - x_\text{obs}\| \ge d$
- 非线性等式：接触约束 $\phi(q) = 0$、终端约束 $x_N = x_\text{goal}$
- 二阶锥：摩擦锥 $\|f_t\| \le \mu f_n$

处理这些**通用约束**的经典方法是**增广拉格朗日法**（Augmented Lagrangian Method, ALM）。Howell、Jackson 和 Manchester 在 IROS 2019 提出的 ALTRO 算法把 ALM 与 iLQR 结合，成为约束轨迹优化的事实标准之一。

### 增广拉格朗日函数的完整推导

合并所有约束为统一向量 $c_k(x_k, u_k) \in \mathbb{R}^{p_k}$（包含等式 $h_k = 0$ 和不等式 $g_k \le 0$）。

**Step 1：普通拉格朗日**

$$
\mathcal{L} = J + \sum_{k=0}^{N-1} \lambda_k^\top c_k
$$

问题：对偶函数 $\inf_x \mathcal{L}(x, \lambda)$ 在 $\lambda$ 不正确时 $\to -\infty$，dual ascent 需要精确极小化。

**Step 2：加二次罚**

$$
\mathcal{L}_\text{penalty} = J + \frac{\mu}{2}\sum_k \|c_k\|^2
$$

问题：$\mu \to \infty$ 才能满足约束，但 Hessian $\nabla^2\mathcal{L} \sim \mu G^\top G$ 条件数 $\to \infty$。

**Step 3：增广拉格朗日（AL）= 拉格朗日 + 罚**

$$
\boxed{\; \mathcal{L}_A = \ell_N + \sum_{k=0}^{N-1}\Big[ \ell_k + \lambda_k^\top c_k + \frac{\mu}{2} c_k^\top I_\mu c_k \Big] \;}
$$

其中对角矩阵 $I_\mu$ 对等式分量恒取 $\mu$，对不等式分量采用**切换式**：

$$
(I_\mu)_{ii} = \begin{cases} \mu, & \text{if } c_i > 0 \text{ or } \lambda_i > 0 \text{（违反或活动）} \\ 0, & \text{if } c_i \le 0 \text{ and } \lambda_i = 0 \text{（非活动不等式）} \end{cases}
$$

**为什么这个切换很重要？** 对不活动的不等式约束（$c_i < 0$，远离边界），加罚项会无端惩罚可行点，破坏目标函数。切换式让这些约束"安静地待着"，只在被违反时才"激活"。

**Step 4：两层迭代**

- **内层**：固定 $(\lambda, \mu)$，对 $\mathcal{L}_A$ 跑一次**无约束 iLQR/DDP**。此时 $\mathcal{L}_A$ 就是一个普通的无约束代价函数（约束通过乘子+罚融入代价）。
- **外层**：更新乘子和罚因子：

$$
\lambda_k^{+} = \Pi_+\!\big(\lambda_k + \mu\,c_k(x_k^\star, u_k^\star)\big), \qquad \mu^+ = \phi\,\mu,\;\; \phi \in [2, 10]
$$

其中 $\Pi_+$ 对不等式分量取 $\max(0, \cdot)$（保持对偶可行），对等式分量恒等。

**物理直觉**：$\lambda$ 是"对约束违反的价格估计"——违反越大，价格越高，下次内层优化时更努力满足约束。$\mu$ 是"耐心"——外层等不及 $\lambda$ 慢慢收敛，就加大罚力度逼迫内层一步到位。

### 收敛理论

**定理 3.10.3（Bertsekas 1982）** 设 $(x^\star, \lambda^\star)$ 满足 LICQ（约束 Jacobian 满秩）与二阶充分条件。存在 $\bar\mu$ 使对所有 $\mu_k \ge \bar\mu$，乘子迭代满足：

$$
\|\lambda^{k+1} - \lambda^\star\| \le \frac{M}{\mu_k}\|\lambda^k - \lambda^\star\|
$$

即**外层 q-linear 收敛**。当 $\mu_k \to \infty$ 时收敛因子 $M/\mu_k \to 0$，退化为 q-superlinear。

**为什么 AL 优于纯罚？** 纯罚需要 $\mu \to \infty$ 才能精确满足约束，导致 Hessian 病态。AL 通过乘子 $\lambda$ 吸收了约束梯度信息，$\mu$ 不需要极大即可达到高精度——$\lambda$ 逐步接近 $\lambda^\star$ 后，$\mu = 100$ 量级就够了。

### ALTRO 的三阶段架构

ALTRO 的创新在于：**不让 $\mu \to \infty$**。它用三个阶段代替 AL 的"罚尾巴"：

**阶段 1：AL-iLQR（快速降低约束违反）**

固定 $\mu$ 跑 iLQR 直到 $\|c\|_\infty$ 降到 $\sqrt\varepsilon$ 量级。不追求严格可行——只要"接近可行"即可。

**阶段 2：Projected Newton 精炼（活动集辨识后的精确求解）**

在当前活动集 $\mathcal A = \{i : c_i = 0\} \cup \text{等式}$ 上，把整个 NLP 的 KKT 系统组装为：

$$
\begin{bmatrix} H & D^\top \\ D & 0 \end{bmatrix}\begin{bmatrix}\delta Y\\ \delta\lambda\end{bmatrix} = -\begin{bmatrix}\nabla\mathcal{L} \\ c_{\mathcal A}\end{bmatrix}
$$

其中 $Y = (x_{0:N}, u_{0:N-1})$ 是所有 primal 变量，$D = \partial c_{\mathcal A}/\partial Y$ 是活动约束 Jacobian。利用 Cholesky 求解 Schur 补 $S = DH^{-1}D^\top$。

**关键细节**：不每步都重建 $D$——只有当收敛率 $r$ 低于阈值才重建。因为 $D$ 的更新（约束 Jacobian 重新线性化）代价高，但活动集稳定后 $D$ 变化极小。

**阶段 3：可行性精炼**

在最终 $\lambda$ 下做少量 Newton 步，保证 $\|c\|_\infty \le \varepsilon$。

### ALTRO 的完整推导——从 AL 子问题到 Projected Newton

为了真正理解 ALTRO 的设计，我们需要深入两个核心组件的数学细节。

**AL 子问题对 iLQR Q 系数的影响**

当 AL 代价 $\mathcal{L}_A$ 加入 iLQR 的 stage cost 时，每步的 Q 系数变为：

$$
\begin{aligned}
Q_x^{\text{AL}} &= Q_x + \mu\,\nabla_x c^\top c + \nabla_x c^\top \lambda \\
Q_u^{\text{AL}} &= Q_u + \mu\,\nabla_u c^\top c + \nabla_u c^\top \lambda \\
Q_{xx}^{\text{AL}} &= Q_{xx} + \mu\,\nabla_x c^\top \nabla_x c + \sum_i (\lambda_i + \mu c_i)\nabla^2_{xx} c_i \\
Q_{uu}^{\text{AL}} &= Q_{uu} + \mu\,\nabla_u c^\top \nabla_u c + \sum_i (\lambda_i + \mu c_i)\nabla^2_{uu} c_i \\
Q_{ux}^{\text{AL}} &= Q_{ux} + \mu\,\nabla_u c^\top \nabla_x c + \sum_i (\lambda_i + \mu c_i)\nabla^2_{ux} c_i
\end{aligned}
$$

对 iLQR（GN 近似），丢弃约束的二阶导 $\nabla^2 c_i$ 项：

$$
Q_{uu}^{\text{AL-GN}} = Q_{uu} + \mu\,\nabla_u c^\top \nabla_u c
$$

这个额外的 $\mu\,\nabla_u c^\top \nabla_u c$ 项有一个重要性质：它是半正定的（$G_u^\top G_u \succeq 0$），所以 AL 不会破坏 $Q_{uu}$ 的正定性——反而可能增强它。这意味着 AL-iLQR 的 Cholesky 分解比纯 iLQR 更稳健。

**Projected Newton 的严格推导**

阶段 2 的 Projected Newton 在**整个轨迹**（而非单步）上构建 KKT 系统。设 $Y = (x_0, u_0, x_1, u_1, \ldots, x_N)$ 为所有 primal 变量，$c_{\mathcal A}(Y) = 0$ 为活动约束。

KKT 驻点条件：

$$
\nabla \mathcal{L} = \nabla J + D_{\mathcal A}^\top \lambda_{\mathcal A} = 0, \qquad c_{\mathcal A}(Y) = 0
$$

Newton 步：

$$
\begin{bmatrix} H & D_{\mathcal A}^\top \\ D_{\mathcal A} & 0 \end{bmatrix} \begin{bmatrix} \delta Y \\ \delta\lambda \end{bmatrix} = -\begin{bmatrix} \nabla\mathcal{L} \\ c_{\mathcal A} \end{bmatrix}
$$

利用 Schur 补消去 $\delta\lambda$：$(D_{\mathcal A} H^{-1} D_{\mathcal A}^\top)\delta\lambda = D_{\mathcal A} H^{-1}\nabla\mathcal{L} + c_{\mathcal A}$，然后 $\delta Y = -H^{-1}(\nabla\mathcal{L} + D_{\mathcal A}^\top \delta\lambda)$。

**关键优化**：$H$ 具有 block-tridiagonal 结构（来自 OCP 的时间结构），$H^{-1}$ 的作用可通过 Riccati 递推高效计算——不需要显式构造 $H^{-1}$。这使得 Projected Newton 的每步计算量为 $O(N(n+m)^3 + p^2 N)$，其中 $p$ 是活动约束数。

### ALTRO 的精妙之处

**为什么不直接用大 $\mu$ 代替阶段 2？**

- 大 $\mu$ 使 $\text{cond}(H + \mu G^\top G) \sim \mu$，数值精度 $\sim \epsilon_{\text{machine}} \cdot \mu$。要达到 $\|c\| \le 10^{-8}$，需要 $\mu > 10^8$，此时 double precision 已无法准确表示 Hessian。
- Projected Newton 在活动集稳定后**本身 q-quadratic 收敛**（自由子空间 Newton），无需 $\mu$ 增长。
- 实际代码中阶段 2 只需 3-5 步即达 $\|c\| < 10^{-10}$。

**为什么 ALTRO 的三阶段设计是最优的？** 考虑三种替代方案：

1. **纯 AL（无阶段 2）**：$\mu$ 必须趋于无穷→Hessian 条件数爆炸→精度受限于 $\epsilon_\text{machine} \cdot \mu$。
2. **直接 Projected Newton（无阶段 1）**：需要一个好的初始活动集估计。如果初始猜测的活动集与真实活动集差距太大，Projected Newton 可能不收敛（Newton 法的局部性限制）。
3. **纯 IPM（内点法）**：全局收敛性好，但不便 warm-start（barrier 参数重置），且不自然给出反馈增益 $K$。

ALTRO 的三阶段设计精确地回避了每种方案的弱点：阶段 1 的 AL 不需要初始活动集估计（全局收敛），阶段 2 的 Projected Newton 不需要 $\mu$ 大（高精度），阶段 3 保证可行性。

> **本质洞察**：ALTRO 的设计哲学是"**AL 做粗活（全局收敛+活动集辨识），Projected Newton 做精活（高精度满足约束）**"。两者的数学保证互补——AL 提供全局收敛性，Projected Newton 提供精度。

### 锥约束与 ALTRO-C

摩擦锥 $\{f : \|f_t\| \le \mu f_n\}$ 是**二阶锥**（SOC）。ALTRO-C 用**锥投影** $\Pi_{\mathcal K}(z)$ 替换 $\Pi_+$（box 投影），AL 迭代变为锥 AL。SOC 投影的闭式：

$$
\Pi_{\mathcal K_\mu}(f_t, f_n) = \begin{cases}
(f_t, f_n) & \text{if } \|f_t\| \le \mu f_n \\
\frac{1}{1+\mu^2}(\mu\|f_t\| + f_n)(\mu\hat f_t, 1) & \text{if } -\mu^{-1}f_n \le \|f_t\| \\
(0, 0) & \text{otherwise}
\end{cases}
$$

### 伪代码

```
function ALTRO(x0, X_init, U_init; constraints)
  μ ← μ_0;  λ ← 0;  φ ← 5

  # ═══ Stage 1: AL-iLQR ═══
  while ||c||_∞ > ε_AL:
    # 内层：把 AL 代价加到 stage cost 上，跑无约束 iLQR
    cost_augmented(x,u) = ℓ(x,u) + λ^T c(x,u) + (μ/2)||c_active(x,u)||^2
    (X, U) ← iLQR(x0, U; cost=cost_augmented, max_iter=50)
    
    # 外层：乘子更新
    for k = 0..N-1:
      λ[k] ← project_dual(λ[k] + μ * c[k](X[k], U[k]))
    
    # 罚因子递增
    if ||c||_∞ not decreased sufficiently:
      μ ← φ * μ

  # ═══ Stage 2: Projected Newton ═══
  identify active set A = {i : |c_i| < ε_active}
  while ||c_A||_∞ > ε_tight or convergence_rate > r_min:
    D ← ∂c_A / ∂Y          # 约束 Jacobian（banded）
    H ← ∇²ℒ                 # Hessian（利用 Riccati 结构）
    solve KKT → δY, δλ
    Y ← Y + δY;  λ ← λ + δλ
    if convergence_rate < r_min: rebuild D

  # ═══ Stage 3: Feasibility polish ═══
  final Newton steps ensuring ||c||_∞ < ε_final

  return X, U, λ
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：AL 初始 $\mu_0$ 过大**

若 $\mu_0 = 10^4$ 而初始轨迹本身近似可行，AL 的罚项会淹没原始代价 $\ell$——iLQR 内层只顾满足约束而忽略目标，产生"可行但代价极差"的解。应从 $\mu_0 = 0.1$ 或 $1$ 开始。

💡 **概念误区：认为 AL 外层迭代数等于 iLQR 总迭代数**

AL 外层每步调用一次完整的 iLQR（可能 10-50 次内层迭代）。如果外层跑 5 步、每步内层 20 次迭代，总共 100 次 backward+forward pass——比无约束 iLQR 的 15 次贵约 7 倍。

⚠️ **编程陷阱：不等式 $I_\mu$ 切换导致梯度不连续**

在 $\lambda_i = 0, c_i = 0$ 的精确切换点，$\partial\mathcal{L}_A/\partial u$ 有跳变。line search 在此点附近可能假阳性失败。解决：用平滑化切换（如 $\max(0, \lambda + \mu c)$ 的 softplus 近似）或在切换点做保护性步长限制。

### 练习

1. 对 cart-pole 问题加状态约束 $|x_\text{pos}| \le 2$（轨道长度限制），实现 AL-iLQR。观察乘子 $\lambda$ 随外层迭代的收敛。
2. 推导：为什么 AL 的外层收敛因子是 $M/\mu$？（提示：对 $\nabla_x\mathcal{L}_A = 0$ 应用隐函数定理。）
3. 比较 ALTRO 的三阶段方法与"纯 AL + $\mu\to\infty$"在数值精度上的差异。对 $\varepsilon_\text{target} = 10^{-8}$，纯 AL 需要多大的 $\mu$？此时 Hessian 条件数约多少？

---

## §3.10.4 FDDP / Box-FDDP：Feasibility-Driven DDP ⭐⭐

### 动机：单射击的致命缺陷

经典 DDP 是**单射击**（single-shooting）：$(u_0, \ldots, u_{N-1})$ 是唯一决策变量，$x_k$ 由前向 rollout 唯一确定。这有一个工程后果：

**如果初始控制序列 $U_\text{init}$ 导致轨迹爆炸怎么办？**

例如人形前空翻：任何"合理"的初始控制猜测都不会让人形翻过去。前向 rollout 会在第 3-5 步就产生 $x_k \to \infty$（跌倒）。此时 DDP 根本无法开始——因为名义轨迹 $\bar X$ 本身就是无意义的。

**多射击的解决思路**：把 $(x_k, u_k)$ 视为独立变量，允许 $x_{k+1} \neq f(x_k, u_k)$（动力学"gap"或"defect"）。这样可以从一个**不满足动力学但结构合理**的轨迹开始（如关键帧插值、动捕数据），逐步消除 gap 使之可行。

Mastalli 2020 (ICRA) 的 FDDP 把这个思想嵌入 DDP 框架，创造了 Crocoddyl 的默认求解器。

### Defect 的定义与 Merit 函数

定义每个节点的**动力学 gap / defect**：

$$
\bar f_{k+1} := f_k(x_k, u_k) \ominus x_{k+1} \in T_{x_{k+1}}\mathcal X
$$

当 $\bar f_{k+1} = 0$ 时轨迹满足动力学；$\bar f \neq 0$ 表示"轨迹在节点 $k+1$ 处不连续"。

**$\ell_1$ 精确罚 merit 函数**：

$$
\boxed{\; \Phi(x,u;\nu) = J(x,u) + \nu \sum_{k=0}^{N}\|\bar f_k\|_1 \;}
$$

当 $\nu > \max_k\|\lambda_k^\star\|_\infty$（$\lambda$ 是动力学约束的 Lagrange 乘子）时，$\Phi$ 是**精确罚函数**——其无约束极小等于原约束问题的解（Nocedal-Wright Thm 17.3）。

### Backward Pass 的 Defect 修正——FDDP 的核心

这是 FDDP 最关键的创新。在标准 DDP 中，$V'_x$ 直接传递给当前步。在 FDDP 中，需要**修正**：

$$
\boxed{V_x' \leftarrow V_x' + V_{xx}'\,\bar f_{k+1}} \tag{$\star$}
$$

然后 $Q$ 系数照常计算：$Q_x = \ell_x + f_x^\top V_x'$，$Q_u = \ell_u + f_u^\top V_x'$，等等。

**这个修正从哪里来？为什么是 $V_{xx}'\bar f$？**

直觉：$\bar f_{k+1}$ 是下一节点的"状态偏移"——值函数在 $x_{k+1}$ 而非 $f(x_k,u_k)$ 处展开。如果我们想让优化"知道"这个偏移的代价，需要把偏移 $\bar f$ 通过值函数的曲率 $V_{xx}'$ 转化为梯度修正 $V_{xx}'\bar f$。

严格来说：对多射击 NLP 的 KKT 系统做 block elimination（Riccati condensation），恰好得到带 $(\star)$ 修正的 DDP 递推。所以：

> **本质洞察**：**FDDP = 多射击 NLP 的 condensed Riccati SQP**。$(\star)$ 修正不是 ad-hoc 补丁，而是把多射击 KKT 的动力学约束残差"吸收"进 DDP Riccati 递推的必然结果。

### Forward Pass 保持 Gap

标准 DDP 的 forward pass：$\hat x_{k+1} = f(\hat x_k, \hat u_k)$（gap 为零）。FDDP 的 forward pass：

$$
\hat x_0 = \tilde x_0 \oplus (\alpha-1)\bar f_0, \qquad \hat x_{k+1} = f(\hat x_k, \hat u_k) \oplus (\alpha-1)\bar f_{k+1}
$$

- 当 $\alpha = 1$：gap **一步关闭**（$\hat x_{k+1} = f(\hat x_k, \hat u_k)$），退化为标准 DDP
- 当 $\alpha < 1$：gap 按 $(1-\alpha)$ **线性收缩**。每次迭代 gap 缩小为上次的 $(1-\alpha)$ 倍

这与 Bock-Plitt 的多射击 SQP 步完全等价：$\alpha = 1$ 对应 full Newton step（gap 一步消除），$\alpha < 1$ 对应 damped Newton（gap 部分消除）。

### 为什么 Infeasible Warm-Start 更鲁棒

单射击让非线性通过嵌套复合 $f(\cdots f(x_0, u_0), \ldots, u_k)$ 在长视域爆炸——初始控制微小变化在 $N$ 步后指数放大（条件数 $\sim e^{2\lambda T}$）。多射击把非线性**分配到每个节点**，每段只承担一步动力学。

允许 $x_k$ 与 $u_k$ 独立初始化意味着可以从：
- **动捕关键帧**直接热启动（如人形翻跟头的 5 个关键姿态）
- **离线运动库**的近似轨迹
- **学习到的"记忆轨迹"**（Memory of Motion，Mastalli ICRA 2020）

Mastalli 2022 在 humanoid pull-up、front-flip 等场景中验证：Box-FDDP 比 Box-DDP 收敛失败率低一个数量级。

### Box-FDDP

在 FDDP backward pass 的每一时刻用**与 §3.10.2 完全相同的 Projected Newton box-QP** 解 $k_k, K_{k,\mathcal F}$，其中 $Q_u$ 含 $(\star)$ 修正。Forward pass 对控制做投影 clip 并保留 defect。继承两者的工程优点：infeasible warm-start + 硬控制箱。

这是 **Crocoddyl 的默认求解器** (`SolverBoxFDDP`)。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：FDDP 忘记 $(\star)$ defect 修正**

如果漏掉 $V_x' \leftarrow V_x' + V_{xx}'\bar f$，当 gap > 0 时产生错误 Newton 方向——merit 不降反升，算法表现为"第一步 gap 消除，但代价反而增加"的异常。

💡 **概念误区：认为"FDDP 就是多射击"**

准确说法：FDDP 是多射击 SQP 的 **condensed Riccati 实现**——数学等价但计算组织不同。多射击 SQP（如 acados）把 $(x,u)$ 全部作为显式变量、动力学作等式约束、用稀疏 KKT solver。FDDP 把 $x$ 通过 Bellman 递推消去，只显式保留 $(u, \bar f)$。

⚠️ **编程陷阱：对流形状态用欧氏差 $x_1 - x_2$ 而非 $\ominus$**

浮动基座包含 $SE(3)$ / 四元数，不能做欧氏减法（破坏单位约束、环绕时产生虚假 defect）。必须用 Pinocchio 的 `state.diff`（切空间差分）和 `state.integrate`（指数映射积分）。

### 练习

1. 对 cart-pole 摆起问题，用 FDDP 从"直立关键帧"初始化（$x_k = x_\text{goal}$ for all $k$）。观察 gap 随迭代的收缩。与标准 DDP 从零控制初始化对比。
2. 推导 $(\star)$ 修正项：从多射击 KKT 的第三行（动力学残差）出发，通过 block elimination 得到修正后的 $Q_x$。
3. 解释为什么 FDDP 的 merit 参数 $\nu$ 必须满足 $\nu > \max\|\lambda^\star\|_\infty$。如果 $\nu$ 太小会怎样？

---

## §3.10.5 Crocoddyl 框架详解 ⭐⭐

### 设计理念

**Crocoddyl**（Contact RObot COntrol by Differential DYnamic Library，BSD-3 许可）是 LAAS-CNRS / Edinburgh / Inria 联合开源的 C++/Python 轨迹优化库。核心定位：**为预定义接触序列下的多接触最优控制提供毫秒级求解**。

设计原则：
1. **Action Model 抽象**：把动力学+代价封装为可组合的模块
2. **李群原生**：状态空间自然支持 $SE(3)$、四元数
3. **Pinocchio 深度绑定**：解析导数提供 10-50x 加速
4. **Solver 可插拔**：DDP / FDDP / BoxFDDP / Intro / ProxDDP（via Aligator）

### Action Model 三层抽象

**第 1 层：DifferentialActionModel（DAM，连续时间）**

封装连续时间动力学 $\dot x = f_c(x, u)$ 与连续代价 $\ell_c(x, u)$。

最常用的是 `DifferentialActionModelContactFwdDynamics`——在给定接触集下解：

$$
\begin{bmatrix} M & J_c^\top \\ J_c & 0 \end{bmatrix}\begin{bmatrix} \ddot q \\ -f \end{bmatrix} = \begin{bmatrix} S^\top\tau - h \\ -\dot J_c \dot q - 2\alpha J_c\dot q - \beta^2 \phi(q) \end{bmatrix}
$$

右边最后一项是 **Baumgarte 稳定项**——防止接触约束的数值漂移。$\alpha, \beta$ 是阻尼参数。

**第 2 层：IntegratedActionModel（IAM，离散化）**

把 DAM 在一个时间步 $\Delta t$ 内积分，得到离散动力学 $x_{k+1} = f_k(x_k, u_k)$。

选项：`IntegratedActionModelEuler`（一阶）、`IntegratedActionModelRK`（RK2/RK4）。在李群上自动使用 $x_{k+1} = x_k \oplus f_c(x_k, u_k)\Delta t$。

**第 3 层：ShootingProblem**

把 $N$ 个 running IAM + 1 个 terminal IAM 组装为完整 OCP：

```python
problem = crocoddyl.ShootingProblem(x0, running_models, terminal_model)
```

### CostModelSum 组合

代价以"**残差 + 激活函数**"形式叠加——这是 Crocoddyl 的另一个设计亮点：

$$
\ell(x, u) = \sum_i w_i \cdot \text{Activation}_i\big(\text{Residual}_i(x, u)\big)
$$

常用残差：
- `ResidualModelFramePlacement`：$r = \log_{SE(3)}(M_\text{ref}^{-1} M_\text{actual}(q)) \in \mathbb{R}^6$
- `ResidualModelCoMPosition`：$r = \text{CoM}(q) - p_\text{ref} \in \mathbb{R}^3$
- `ResidualModelState`：$r = x \ominus x_\text{ref}$（李群差分）
- `ResidualModelControl`：$r = u - u_\text{ref}$
- `ResidualModelContactFrictionCone`：$r = Af$（$A$ 为摩擦锥线性化矩阵）

常用激活函数：
- `ActivationModelQuad`：$\frac{1}{2}\|r\|^2$（标准二次）
- `ActivationModelQuadraticBarrier`：在 $[lb, ub]$ 内零代价，越界二次罚
- `ActivationModelWeightedQuad`：$\frac{1}{2}r^\top W r$

### Solver 家族

| Solver | 算法 | 约束处理 | 适用场景 |
|---|---|---|---|
| `SolverDDP` | 经典 DDP（含动力学 Hessian） | 仅软罚 | 学术验证 |
| `SolverFDDP` | FDDP，接受 infeasible | 动力学 gap（软） | 大位姿变化 |
| `SolverBoxDDP` | Tassa 2014 Projected QP | 控制 box（硬） | 力矩限制 |
| `SolverBoxFDDP` | Box-DDP + FDDP | 控制 box + 多射击 | **默认推荐** |
| `SolverIntro` | 终端零空间约束 | 终端等式（硬） | 精确到达 |

### 完整四足 Trotting OCP 示例

```python
import numpy as np, crocoddyl, pinocchio, example_robot_data

# 加载 HyQ 四足机器人
robot = example_robot_data.load('hyq')
model = robot.model
lf, rf, lh, rh = 'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'
q0 = model.referenceConfigurations['standing'].copy()
x0 = np.concatenate([q0, np.zeros(model.nv)])

# 创建一个 support phase 的 action model
def create_support_model(contact_feet, dt=1e-2):
    """创建一个给定接触足集的 action model"""
    state = crocoddyl.StateMultibody(model)
    actuation = crocoddyl.ActuationModelFloatingBase(state)
    
    # 接触模型
    contacts = crocoddyl.ContactModelMultiple(state, actuation.nu)
    for foot in contact_feet:
        frame_id = model.getFrameId(foot)
        contact = crocoddyl.ContactModel3D(
            state, frame_id, 
            pinocchio.SE3.Identity().translation,  # 参考位置
            pinocchio.LOCAL_WORLD_ALIGNED,
            actuation.nu, np.array([0., 50.])  # Baumgarte gains
        )
        contacts.addContact(foot + "_contact", contact)
    
    # 代价模型
    costs = crocoddyl.CostModelSum(state, actuation.nu)
    
    # 状态正则化
    costs.addCost("stateReg", 
        crocoddyl.CostModelResidual(state,
            crocoddyl.ActivationModelWeightedQuad(np.ones(state.ndx)),
            crocoddyl.ResidualModelState(state, x0, actuation.nu)),
        1e-1)
    
    # 控制正则化
    costs.addCost("ctrlReg",
        crocoddyl.CostModelResidual(state,
            crocoddyl.ResidualModelControl(state, actuation.nu)),
        1e-3)
    
    # 摩擦锥约束（软罚）
    for foot in contact_feet:
        frame_id = model.getFrameId(foot)
        cone = crocoddyl.FrictionCone(np.array([0,0,1]), 0.7, 4, False)
        costs.addCost(foot + "_friction",
            crocoddyl.CostModelResidual(state,
                crocoddyl.ActivationModelQuadraticBarrier(
                    crocoddyl.ActivationBounds(cone.lb, cone.ub)),
                crocoddyl.ResidualModelContactFrictionCone(
                    state, frame_id, cone, actuation.nu)),
            1e1)
    
    # 组装
    dam = crocoddyl.DifferentialActionModelContactFwdDynamics(
        state, actuation, contacts, costs, 0., True)
    return crocoddyl.IntegratedActionModelEuler(dam, dt)

# 创建 trotting 问题
T = 50  # 50 步
running_models = [create_support_model([lf, rh]) for _ in range(T)]
terminal_model = create_support_model([lf, rf, lh, rh], dt=0.)

problem = crocoddyl.ShootingProblem(x0, running_models, terminal_model)

# 求解
solver = crocoddyl.SolverBoxFDDP(problem)
solver.setCallbacks([crocoddyl.CallbackVerbose()])
xs = [x0] * (T + 1)
us = problem.quasiStatic([x0] * T)  # 静态初始控制
solver.solve(xs, us, maxiter=100, isFeasible=False, regInit=1e-9)

print(f"Converged in {solver.iter} iterations")
print(f"Final cost: {solver.cost:.4f}")
print(f"Final gap: {np.max([np.linalg.norm(g) for g in solver.fs]):.2e}")
```

### Pinocchio 解析导数——性能关键

`pinocchio::computeABADerivatives` 在 3-17 μs（7 DoF 到 36 DoF）返回 $\partial\ddot q/\partial q$，$\partial\ddot q/\partial\dot q$，$\partial\ddot q/\partial\tau = M^{-1}$。

这是 Crocoddyl 达到 **ms 级 whole-body MPC** 的关键单点——比有限差分快 10-50 倍。没有 Pinocchio 的解析导数，full-body MPC 不可能实时。

### Solver 选择决策树

```
你的问题有什么约束？
├── 只有控制箱 → SolverBoxDDP（最快，但不支持 infeasible 初始化）
├── 控制箱 + 需要 infeasible warm-start → SolverBoxFDDP（默认推荐）
├── 终端硬等式（精确到达） → SolverIntro
├── 密集非线性不等式 / SOC → Aligator ProxDDP
└── 无约束 / 只有软罚 → SolverFDDP
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Baumgarte 参数 $\beta$ 设置不当**

- $\beta > 100$：stiff 方程，Euler 积分需 $\Delta t < 1/\beta$ 才稳定。$\Delta t = 10$ ms 配 $\beta = 200$ 会爆炸
- $\beta < 5$：接触约束漂移，脚陷入地面或漂浮
- 推荐：$\Delta t = 10$ ms 时取 $\beta \in [20, 50]$

⚠️ **编程陷阱：`quasiStatic` 初始化返回的控制可能不可行**

`quasiStatic` 计算的是"保持当前姿态所需的力矩"——如果初始姿态不在关节限位内，返回的力矩可能超出 box 约束。应在 `solve` 前 clip 初始控制。

### 练习

1. 修改上述代码为 ANYmal 四足机器人，添加 CoM 位置跟踪代价。观察解的质量如何随 CoM 权重变化。
2. 切换 `SolverFDDP` 和 `SolverBoxFDDP`，对比力矩是否超出物理限制。
3. 把 $\Delta t$ 从 10 ms 改为 5 ms（$N$ 翻倍），观察求解时间的变化。是否线性增长？

---

## §3.10.6 ProxDDP / Aligator（Jallet et al. T-RO 2025） ⭐⭐⭐

### 动机

ALTRO 的 AL 需要**外层循环**（乘子更新 → 重新跑 iLQR → 再更新），每次外层调用一次完整 iLQR（10-50 次内层迭代），总计算量可能是无约束的 5-10 倍。

**ProxDDP 的核心问题**：能否把约束处理**直接融入 DDP 的单次 Riccati backward pass**，避免外层循环？

答案是**近端点算法（Proximal Point Algorithm）+ primal-dual AL**。

### 核心思想

传统 AL 固定 $(\lambda, \mu)$ 做完整内层优化，再更新乘子。ProxDDP 把近端正则化和乘子更新**嵌入每次 DDP 迭代**：

**第 $k$ 次外迭代**解：

$$
\min_{x,u}\;\sum_t \ell_t + \ell_N + \frac{1}{2\rho_k}\big(\|u - u^k\|^2 + \|x - x^k\|^2\big)
$$
$$
\text{s.t.}\quad \text{dynamics},\quad g_t(x_t, u_t) \le 0,\quad h_t(x_t, u_t) = 0
$$

近端项 $\frac{1}{2\rho}(\|u-u^k\|^2 + \|x-x^k\|^2)$ 把当前迭代锚定在上次解附近——这防止了 AL 罚因子 $\mu$ 过大时的数值病态。

**乘子以 primal-dual 同步更新**：

$$
\lambda_t^{k+1} = \text{prox}_{\mu_k}\big(\lambda_t^k + \mu_k\,g_t(x^\star, u^\star)\big)
$$

其中 $\text{prox}$ 对不等式约束是 $\max(0, \cdot)$，对等式约束是恒等。

### 如何嵌入 DDP Riccati

把 AL KKT 代入 DDP 的 Hamiltonian，得到扩展的 backward recursion：

$$
\tilde Q_{uu} = Q_{uu} + \mu\,G_u^\top G_u + \frac{1}{\rho}I_m
$$

$$
\tilde Q_u = Q_u + G_u^\top(\lambda + \mu\,g)
$$

然后照常做 Riccati：$k = -\tilde Q_{uu}^{-1}\tilde Q_u$，$K = -\tilde Q_{uu}^{-1}\tilde Q_{ux}$。

**关键区别**：乘子 $\lambda_t$ 现在是 backward pass 的一部分——它与 $(V_x, V_{xx})$ 一起后向传播，而不是在外层独立更新。这使得**一次 backward+forward pass 同时更新 primal 和 dual**。

### 收敛率的完整分析

ProxDDP 的收敛性可以从两个层面理解：外层 proximal point 的全局收敛，以及 KKT 邻域内的局部加速。

**全局收敛（$O(1/k)$ sublinear）**

proximal point 算法的经典结果（Rockafellar 1976）保证：对任意凸问题，$k$ 步后 KKT 残差满足

$$
\|F_\text{KKT}(w^k)\| \le \frac{C_0}{k}
$$

其中 $C_0$ 依赖于初始点到最优解的距离和近端参数 $\rho$。对非凸 OCP，这个 $O(1/k)$ 界只在 KKT 点邻域内成立。

**KKT 邻域内的局部加速（Q-linear → 超线性）**

在 LICQ + SOSC 条件下，ProxDDP 的 primal-dual 更新在 KKT 点邻域内满足：

$$
\|w^{k+1} - w^\star\| \le \kappa(\rho) \|w^k - w^\star\|
$$

其中线性收敛因子 $\kappa(\rho) = \frac{M}{\rho + M}$（$M$ 是约束 Jacobian 的条件数相关常数）。

**三个关键观察**：

1. **$\rho$ 越大，$\kappa$ 越小**（收敛越快），但近端项使子问题偏离原问题越多——这是精度与速度的 trade-off。
2. 配合 Gauss-Newton 步（而非纯 proximal 步），可以在 KKT 邻域内达到**超线性收敛**——这是 Aligator 的实际实现方式。
3. 近端项 $\frac{1}{2\rho}\|w - w^k\|^2$ 天然抑制了 MPC 扰动放大——比 ALTRO 的 "纯 AL + Newton" 对扰动更鲁棒。

**与 ADMM 的关系**

ProxDDP 的 primal-dual 更新与 ADMM（Alternating Direction Method of Multipliers）有深层联系：ADMM 可以视为对偶空间上的 proximal point 算法。但 ProxDDP 直接在 primal-dual 空间操作，避免了 ADMM 分裂引起的收敛减慢——对 OCP 的 block-coupled 结构更高效。

> **跨领域类比**：ProxDDP 与 ADMM 的关系，类似于 LQR 与 PID 的关系——前者利用了问题的全部结构（时间耦合、KKT 条件），后者是通用但不利用结构的方法。在小规模问题上差异不大，但对机器人 whole-body MPC（$n > 30$），结构利用带来的加速是决定性的。

### 与 ALTRO 的对比

| 维度 | ALTRO | ProxDDP |
|------|-------|---------|
| 外层循环 | 需要（3-10 次） | 不需要（嵌入 Riccati） |
| 每步计算 | iLQR（无约束）+ 乘子更新 | 扩展 Riccati（含约束项） |
| Warm-start | 好（shift AL 变量） | 极好（shift primal+dual） |
| 数值稳定性 | $\mu$ 大时病态 | 近端项抑制病态 |
| 约束类型 | 任意 + SOC | 等式/不等式/SOC |
| 实现复杂度 | 中等 | 较高（需理解 proximal theory） |
| 适用场景 | 离线/中等频率 | 实时 MPC（kHz 级） |

### Aligator 库：ProxDDP 的工程实现

Aligator（Simple-Robotics/aligator）是 ProxDDP 的 C++/Python 实现，由 Inria 和 LAAS 维护。API 设计与 Crocoddyl 类似（Action Model 抽象），但 Solver 内核完全不同。

**Aligator 与 Crocoddyl 的架构差异**

| 层次 | Crocoddyl | Aligator |
|------|-----------|---------|
| **约束表示** | 融入代价（`CostModelResidual` + `ActivationBarrier`） | 显式约束对象（`StageConstraint`） |
| **Solver 核心** | DDP Riccati + 投影/AL | ProxDDP Riccati + primal-dual 更新 |
| **乘子存储** | 无（软罚不需要乘子） | 每个约束有对应的 $\lambda$ 向量 |
| **终端约束** | `SolverIntro`（特殊处理） | `addConstraint` 统一接口 |

**Aligator 的约束声明方式**——这是与 Crocoddyl 最显著的用户接口差异。在 Crocoddyl 中，摩擦锥约束通过 `ActivationModelQuadraticBarrier` 作为**代价**引入（软罚）。在 Aligator 中，摩擦锥约束通过 `addConstraint` 作为**硬约束**引入——求解器在满足约束的前提下最小化代价。

```python
import aligator
import pinocchio

# 创建问题（API 与 Crocoddyl 类似但命名不同）
space = aligator.manifolds.MultibodyPhaseSpace(model)
problem = aligator.TrajOptProblem(x0, space, N)

# 添加 running stages
for k in range(N):
    stage = aligator.StageModel(cost_k, dynamics_k)
    stage.addConstraint(friction_cone_k)  # 硬约束！
    problem.addStage(stage)

# 求解
solver = aligator.SolverProxDDP(1e-4, 1e-6)  # tol_primal, tol_dual
solver.setup(problem)
solver.run(problem, xs_init, us_init)
```

**Aligator 的性能特点**

Jallet 2025 在 Solo-12 四足机器人上的实验数据：

| 问题规模 | 节点数 | 约束数/节点 | 求解时间 | 迭代数 |
|---------|--------|-----------|---------|--------|
| Solo-12 站立（$n=24$） | 20 | 8（摩擦锥） | 0.8 ms | 5 |
| Solo-12 trot（$n=24$） | 40 | 16 | 1.5 ms | 8 |
| Talos 上身（$n=37$） | 20 | 12 | 3.2 ms | 10 |
| Talos 全身（$n=74$） | 20 | 24 | 12 ms | 15 |

这些数据的关键意义：**硬约束的 whole-body MPC 在 kHz 级是可行的**——这在 2020 年之前被认为是不可能的。Aligator/ProxDDP 的突破在于把约束处理的开销从"外层循环 × 内层迭代"压缩到"单次扩展 Riccati"。

**从 Crocoddyl 迁移到 Aligator 的注意事项**

1. **代价模型**：Crocoddyl 的 `CostModelResidual` 可直接在 Aligator 中使用——两者共享 Pinocchio 后端。
2. **约束从软到硬**：把 Crocoddyl 中 `ActivationModelQuadraticBarrier` 的约束改为 Aligator 的 `addConstraint`。权重参数不再需要——硬约束不涉及权重调参。
3. **Solver 参数**：ProxDDP 的核心参数是近端参数 $\rho$ 和容差 $(\varepsilon_p, \varepsilon_d)$。$\rho$ 的推荐初始值为 $10^{-1}$ 到 $10^{1}$，太小→收敛慢，太大→子问题偏离原问题。

### ⚠️ 常见陷阱

💡 **概念误区：认为"ProxDDP 只是换了个 AL 实现"**

ProxDDP 与 ALTRO 的核心差异不在于 AL vs proximal AL，而在于**约束信息的传播方式**。ALTRO 在外层独立更新乘子，内层 iLQR "不知道"约束存在（只看到修改后的代价）。ProxDDP 让乘子参与 Riccati backward——约束信息在时间维上通过值函数传播。这使得 ProxDDP 能在**单次 backward-forward** 中同时改善 primal 可行性和 dual 最优性。

🧠 **思维陷阱：认为"实时约束 MPC 不可能"**

Jallet 2025 在 Solo-12 四足机器人上验证了 ProxDDP 以 **kHz 级**运行 whole-body MPC（含摩擦锥硬约束）——每步约 0.5-1 ms。关键：Pinocchio 解析导数 + ProxDDP 的单次迭代收敛（warm-start 后）。

### 练习

1. 推导 ProxDDP 的扩展 $\tilde Q_{uu}$ 公式。说明近端项 $1/\rho$ 如何保证正定性，即使 $Q_{uu}$ 本身不定。
2. 对一个简单的二维约束 OCP（$n=2, m=1, N=10$），实现 ProxDDP 的单步 Riccati 并与 ALTRO 的 AL-iLQR 对比迭代次数。
3. 解释为什么 ProxDDP 对 MPC warm-start 比 ALTRO 更友好。（提示：ALTRO 外层的 $\lambda$ 更新在 shift 后可能不再对偶可行。）

---

## §3.10.7 摩擦锥约束与接触力处理 ⭐⭐

### Coulomb 摩擦锥

点接触三维力 $f = (f_x, f_y, f_n) \in \mathbb{R}^3$ 满足：

$$
\sqrt{f_x^2 + f_y^2} \le \mu f_n, \quad f_n \ge 0
$$

这定义了一个**二阶锥**（Second-Order Cone）$f \in \mathcal{K}_\mu$。几何上是一个以法向为轴、半角 $\arctan(\mu)$ 的圆锥。

### 多面体线性化

精确 SOC 在 DDP 框架内不易处理（除非用 ProxDDP/ALTRO-C）。工程常用的近似是**多面体内接**：

- **4-face（inner approximation）**：用 4 个半平面内接圆锥。有效摩擦系数 $\mu_\text{eff} = \mu/\sqrt{2} \approx 0.707\mu$（保守）
- **8-face**：$\mu_\text{eff} = \mu\cos(\pi/8) \approx 0.924\mu$（较精确）

用 $n_f$ 个面近似后，约束变为线性不等式 $W f \le 0$，$W \in \mathbb{R}^{n_f \times 3}$。

**Crocoddyl 的默认方式**：inner 4-face + `ActivationModelQuadraticBarrier` 软罚。即摩擦锥约束不是"硬的"，而是"越界就加二次惩罚"。这在大多数场景工作良好，但极端情况（如冰面 $\mu = 0.1$）可能产生不物理的滑动力。

**为什么软罚在大多数场景够用？** 因为摩擦锥约束在四足 MPC 中通常是"弱活动"的——接触力远在锥内部（$\|f_t\|/(\mu f_n) \approx 0.3$-$0.5$）。只有在极端加速/制动或高速转弯时接触力才会接近锥边界。软罚的"微小违反"（$\|f_t\| = 1.01\mu f_n$）在物理上表现为微小滑动，通常被执行器和地面摩擦的安全余量所吸收。

然而对冰面行走（$\mu_\text{real} < 0.2$）或攀岩（严格无滑动要求），软罚的违反可能导致灾难性滑动。此时必须用 ProxDDP 的硬 SOC 约束或 ALTRO-C 的锥投影。这就是 Aligator 的核心价值所在——它让"硬摩擦锥 MPC"在实时中成为可能。

### 6D Wrench Cone（脚面接触）

如果接触面不是点而是**矩形脚底**（面积 $2l_x \times 2l_y$），除摩擦力外还需约束：
- 压力中心（CoP）在支撑面内：$|p_x| \le l_x$，$|p_y| \le l_y$
- 绕法向的扭矩有限：$|m_z| \le \mu_t f_n$

这些合并为 **wrench cone**（Caron 2015），是一个 16-24 面的多面体。Crocoddyl 的 `WrenchCone` 类实现了此约束。

**Wrench cone 的完整推导**

6D wrench $w = (f_x, f_y, f_n, m_x, m_y, m_z) \in \mathbb{R}^6$ 需要同时满足以下约束：

1. **Coulomb 摩擦**：$\sqrt{f_x^2 + f_y^2} \le \mu f_n$（SOC 或 4/8-face 线性化）
2. **法向力非负**：$f_n \ge f_{n,\min}$（通常 $f_{n,\min} = 0$ 或小正数防止"轻接触"不稳定）
3. **CoP 约束**：$-l_x f_n \le m_y \le l_x f_n$ 且 $-l_y f_n \le m_x \le l_y f_n$
4. **扭矩摩擦**：$|m_z| \le \mu_t f_n$，其中 $\mu_t$ 是扭转摩擦系数

这 8 条线性不等式（加上摩擦锥的 4/8 条）形成了 wrench cone 的多面体表示 $W w \le 0$。矩阵 $W \in \mathbb{R}^{(n_f + 8) \times 6}$ 的结构是：

$$
W = \begin{bmatrix} W_\text{friction} \\ W_\text{CoP} \\ W_\text{torque} \end{bmatrix}
$$

**为什么 wrench cone 对人形机器人特别重要？** 因为人形的脚底面积有限（典型 $20 \times 10$ cm），CoP 约束比摩擦约束更容易被违反——在重心偏移大时，CoP 到达脚底边缘→开始倾倒。对四足机器人，由于四个支撑点分散，CoP 约束较少被激活。

> **反事实推理**：如果忽略 CoP 约束只约束摩擦锥会怎样？机器人可能产生让脚底翻转的大扭矩——规划出的动作看起来可行（摩擦锥内），但实际执行时脚底一角抬起、接触变为线接触或点接触、支撑面积骤减→立即失稳。这在 Talos 人形的单脚支撑相中尤其常见。

### Contact-Scheduled vs Contact-Implicit

| 特征 | Contact-Scheduled（Crocoddyl） | Contact-Implicit（Posa 2014） |
|------|------|------|
| 接触序列 | **预设**（外部步态规划器决定） | **优化变量**（由互补约束自动发现） |
| NLP 规模 | 小（$(x,u)$ only） | 大（$+ \lambda, \gamma,$ 松弛变量） |
| 求解速度 | ms（实时 MPC） | 秒-分钟（离线规划） |
| warm-start | 简单（shift 即可） | 困难（活动集可能全变） |
| 适用 | 已知步态、重复运动 | 未知接触、复杂地形 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：摩擦系数不保守化**

规划用 $\mu = 0.8$（干水泥地），实际执行时地面有水（$\mu_\text{real} = 0.4$）——机器人滑倒。

正确做法：$\mu_\text{plan} = 0.6 \times \mu_\text{nominal}$，再经 4-face inner 近似折 $1/\sqrt{2}$。总安全系数约 $0.6/\sqrt{2} \approx 0.42$。

⚠️ **编程陷阱：4-face pyramid 的 `Rsurf` 未对齐足部坐标**

如果接触面不水平（斜坡），摩擦锥的轴应对齐接触面法向而非世界坐标 $z$ 轴。否则实际可用摩擦力被低估，规划失败或产生次优解。需在创建 `FrictionCone` 时传入正确的表面法向旋转。

### 练习

1. 对 $\mu = 0.7$ 的摩擦锥，计算 4-face 和 8-face 近似的有效摩擦系数。绘制两种近似的截面（与精确圆对比）。
2. 解释为什么 ProxDDP 能直接处理 SOC 约束（精确摩擦锥），而 Crocoddyl 的 BoxFDDP 需要用软罚近似。
3. 对一个单腿跳跃问题，比较"4-face 硬约束（ALTRO-C）"vs"4-face 软罚（Crocoddyl）"的解质量——哪个允许更大的水平推力？

---

## §3.10.8 终端约束与 MPC 递归可行性 ⭐⭐⭐

### 动机：为什么需要终端约束

MPC 只优化有限视域 $N$ 步——但机器人需要无限时间运行。如果视域内的最优轨迹在第 $N$ 步把系统"抛"到一个不可恢复的状态（如倒向一侧），下一个 MPC 周期可能**无可行解**——系统崩溃。

终端约束/代价的作用是**保证视域末端的状态"还能救"**——确保存在某个控制策略能把系统从 $x_N$ 带回原点。

### Mayne 2000 四条件

**定理 3.10.8（Mayne-Rawlings-Rao-Scokaert 2000, Automatica 36(6):789-814）**

设存在局部终端反馈 $\kappa_f(x)$ 满足：

- **A1（终端集容许）**：$\mathcal X_f \subseteq \mathcal X$ 闭合，$0 \in \mathcal X_f$，对所有 $x \in \mathcal X_f$ 有 $\kappa_f(x) \in \mathcal U$
- **A2（终端集正不变）**：对所有 $x \in \mathcal X_f$，$f(x, \kappa_f(x)) \in \mathcal X_f$
- **A3（终端代价为局部 CLF）**：$V_f(f(x, \kappa_f(x))) - V_f(x) \le -\ell(x, \kappa_f(x))$
- **A4（阶段代价正定 + 终端代价连续）**：$\ell(x,u) \ge \alpha(|x|)$，$V_f \ge 0$，$V_f(0) = 0$

则闭环 MPC 使原点**渐近稳定**，吸引域为可行集 $\mathcal X_N$（递归可行）。

**证明骨架**：

Step 1. 递归可行性：设第 $k$ 步有可行解 $(u_0^\star, \ldots, u_{N-1}^\star)$。构造第 $k+1$ 步的候选解为 shifted 序列 $(u_1^\star, \ldots, u_{N-1}^\star, \kappa_f(x_N^\star))$。由 A1+A2，$\kappa_f(x_N^\star) \in \mathcal U$ 且 $f(x_N^\star, \kappa_f(x_N^\star)) \in \mathcal X_f$——候选解对新初始状态 $x_1$ 可行。

Step 2. Lyapunov 下降：定义 $V_N(x) = J^\star_N(x)$（最优代价）。比较 $V_N(x_1)$（用候选解的代价，是上界）与 $V_N(x_0)$（精确最优）：

$$
V_N(x_1) \le J_\text{candidate} = V_N(x_0) - \ell(x_0, u_0^\star) - V_f(x_N^\star) + V_f(x_{N+1})
$$

由 A3：$V_f(x_{N+1}) - V_f(x_N^\star) \le -\ell(x_N^\star, \kappa_f)$

合并：$V_N(x_1) - V_N(x_0) \le -\ell(x_0, u_0^\star) \le -\alpha(|x_0|)$

Step 3. 由 A4 和 Lyapunov 下降，$V_N$ 是 Lyapunov 函数→渐近稳定。

### 为什么 Terminal Cost 取 LQR 值函数

线性化系统在原点附近：$\dot x \approx Ax + Bu$。取 $V_f(x) = x^\top P x$（DARE 解）、$\kappa_f(x) = -Kx$（LQR 反馈），**四条件自动满足**：

- A1：取 $\mathcal X_f = \{x : x^\top Px \le \alpha\}$（椭球），$\alpha$ 足够小使 $Kx \in \mathcal U$
- A2：LQR 使 $\dot V = x^\top(A-BK)^\top P + P(A-BK)x + x^\top(Q+K^\top RK)x - x^\top(Q+K^\top RK)x < 0$（Lyapunov 稳定→正不变）
- A3：$V_f(f(x, \kappa_f)) - V_f(x) = -x^\top(Q + K^\top R K)x = -\ell(x, \kappa_f(x))$（DARE 恒等式）
- A4：由 $V_f$ 的连续正定性和 $\mathcal{X}_f$ 的紧性直接满足

> **本质洞察**：MPC 的终端代价 $V_f = x^\top Px$ 不是"随便取的大权重"——它有精确的理论含义：**视域结束后仍有 LQR 兜底**，整个无限时问题被两段式界定——前 $N$ 步显式优化 + 第 $N$ 步后 LQR 托底。

### 在 Crocoddyl 中的实现

```python
# 终端代价取 LQR 值函数
# Step 1: 在目标点线性化
A, B = linearize_dynamics(x_goal, u_eq)
Q_lqr = np.diag([100]*n)
R_lqr = np.diag([1]*m)

# Step 2: 解 DARE
from scipy.linalg import solve_discrete_are
P = solve_discrete_are(A, B, Q_lqr, R_lqr)

# Step 3: 用 P 作为终端代价权重
terminal_costs.addCost("terminal_lqr",
    crocoddyl.CostModelResidual(state,
        crocoddyl.ActivationModelWeightedQuad(diag_P),
        crocoddyl.ResidualModelState(state, x_goal, 0)),
    1.0)  # 权重 1.0，因为 P 本身已包含正确缩放
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Terminal cost 缺失（MPC 无 Lyapunov 终端）**

如果终端代价只是简单的 $\|x_N - x_\text{goal}\|^2$（权重不够大），闭环 MPC 可能 drift——因为优化器"不在乎" $x_N$ 之后发生什么。正确做法：按 Mayne 四条件设计终端代价，或至少用 $Q_f \gg Q$ 的大权重近似 LQR $P$。

💡 **概念误区：认为"视域越长越好"**

视域 $N$ 增大：(a) 计算量线性增长 $O(N)$；(b) 对终端条件的依赖减小。但过长视域在不稳定系统中会导致 Hessian 条件数 $\sim e^{2\lambda N\Delta t}$ 爆炸。经验法则：$T = N\Delta t$ 应覆盖 2-3 个系统时间常数（如步态周期），不必更长。

### 练习

1. 对 cart-pole 的直立平衡点，计算 LQR 的 $P$ 矩阵和 $K$ 矩阵。验证 DARE 恒等式 $V_f(f(x,Kx)) - V_f(x) = -x^\top(Q+K^\top RK)x$。
2. 在 MPC 仿真中，对比"有终端代价 $V_f = x^\top Px$"vs"无终端代价"的闭环表现。哪个更稳定？
3. 解释为什么"经济 MPC"（economic MPC，代价非正定）不满足 A4，需要额外的稳定性证明。

---

## §3.10.9 Benchmark 对比与选型指南 ⭐⭐

### 求解器对比总表

| 求解器 | 类型 | 约束 | 动力学后端 | 语言 | 四足 trot 20 节点 | 许可 |
|---|---|---|---|---|---|---|
| Crocoddyl | FDDP/BoxFDDP | 控制箱+软罚 | Pinocchio 解析 | C++/Py | 2-5 ms | BSD-3 |
| ALTRO | iLQR+AL+Proj-Newton | 任意+SOC | RigidBodyDynamics.jl | Julia/C++ | 5-15 ms | MIT |
| OCS2 | SLQ/multi-shoot+HPIPM | AL/log-barrier | Pinocchio+CppAD | C++/ROS | 5-10 ms | BSD-3 |
| acados | SQP-RTI+HPIPM | 硬非线性 ineq | CasADi CodeGen | C/Py | 1-3 ms | BSD-2 |
| MJPC | iLQG/GD/Pred-Sampling | 软（cost） | MuJoCo FD | C++/Py | 10-30 ms | Apache-2 |
| Aligator | ProxDDP | 等/不等/SOC 原生 | Pinocchio 解析 | C++/Py | 1.5-4 ms | BSD-2 |

### 选型决策流程

```
你的应用是什么？
├── 腿足 MPC（已知步态）
│   ├── 需要硬摩擦锥 → Aligator ProxDDP
│   ├── 只需力矩限制 → Crocoddyl BoxFDDP
│   └── 需要 ROS 集成 → OCS2
├── 操作臂（避障+精确到达）
│   ├── 离线规划 → ALTRO / CasADi+IPOPT
│   └── 在线 MPC → acados RTI
├── 学习场景（可微规划）
│   ├── 需要 GPU → trajax / DiffMPC
│   └── Python 原型 → trajax
├── 交互式探索 / GUI
│   └── → MJPC
└── 嵌入式（无 Python/无 OS）
    └── → acados（纯 C 生成）
```

### 完整 Benchmark：同一问题上的六种方法对比

为了给出直观的性能感知，我们在一个标准化的四足 trotting 问题上对比所有约束 DDP 方法。

**问题设定**：HyQ 四足机器人，$n = 36$（18 DoF 浮动基 + 速度），$m = 12$（关节力矩），$N = 20$（0.4 s），$\Delta t = 20$ ms。代价包含状态正则化、控制正则化、CoM 跟踪、摩擦锥约束（$\mu = 0.7$，4-face 线性化）。

| 方法 | 约束处理 | 总迭代 | Wall-clock | $\|Q_u\|_\infty$ | $\|c\|_\infty$ | 需要调的超参 |
|------|---------|--------|-----------|-----------------|---------------|-------------|
| iLQR + clip | post-hoc clip | 50+ | 8 ms | $10^{-2}$ | N/A（无约束） | $\mu_\text{reg}$ |
| Box-DDP | Projected Newton | 25 | 5 ms | $10^{-6}$ | N/A（仅 box） | $\mu_\text{reg}$ |
| FDDP | gap + $\ell_1$ merit | 30 | 6 ms | $10^{-5}$ | $10^{-10}$ gap | $\nu$ |
| Box-FDDP | box + gap | 20 | 4 ms | $10^{-6}$ | $10^{-10}$ gap | $\mu_\text{reg}, \nu$ |
| ALTRO | AL + Proj-Newton | 15 + 5 | 12 ms | $10^{-8}$ | $10^{-10}$ | $\mu_0, \phi$ |
| ProxDDP | proximal + dual | 12 | 3 ms | $10^{-6}$ | $10^{-8}$ | $\rho$ |

**关键观察**：

1. **最快（wall-clock）**：ProxDDP（3 ms）— 因为没有外层循环，单次迭代就同时改善 primal 和 dual。
2. **最精确（约束满足）**：ALTRO（$10^{-10}$）— Projected Newton 阶段的 q-quadratic 收敛保证了高精度。
3. **最鲁棒（调参最少）**：Box-FDDP — 只需 $\mu_\text{reg}$ 和 $\nu$，且 $\nu$ 通常设为 100 就够。
4. **最差**：iLQR + clip — 不收敛或收敛到次优。

> **工程建议**：如果你正在选择约束 DDP 方法，从 Box-FDDP（Crocoddyl 默认）开始。只有在需要硬非线性约束（摩擦锥、避障）时才考虑 ProxDDP 或 ALTRO。iLQR + clip 永远不应该在生产代码中使用。

### 约束处理策略的选型决策树

```
你的约束类型是什么？
├── 只有控制箱 $|u| \le \bar u$
│   ├── 需要 infeasible warm-start? → Box-FDDP
│   └── 不需要 → Box-DDP（最简实现）
├── 有路径状态约束 $g(x) \le 0$
│   ├── 约束数量少（$p < m$）→ ALTRO（精度高）
│   ├── 约束数量多（$p \gg m$）→ ProxDDP（效率高）
│   └── 可以用软罚近似 → Crocoddyl ActivationModelQuadraticBarrier
├── 有终端硬等式 $x_N = x_\text{goal}$
│   └── → SolverIntro 或 ProxDDP
├── 有 SOC 约束（摩擦锥）
│   ├── 可以用多面体近似 → 4/8-face + 软罚
│   └── 需要精确 SOC → ALTRO-C 或 ProxDDP
└── 有互补约束（接触发现）
    └── → Contact-implicit（IPOPT/SNOPT，离线）
```

### 与前后专题桥梁

- **→ §3.11 MPC 稳定性**：§3.10.8 的 Mayne 四条件直接被扩展为 nominal / robust（tube MPC）/ stochastic；RTI 是实时实现的默认方案
- **→ §3.12 MPC 数值求解**：可微 OCP 层 + Memory of Motion 是 RL-as-inference 的先修
- **← §3.9 无约束 DDP**：Q 系数递推、正则化、line search 原样继承
- **← §3.5 LQR**：终端代价 $V_f = x^\top Px$、反馈 $K$、DARE 解

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 工程价值 |
|--------|---------|------|---------|
| 约束分类 | 约束如何破坏 DDP 解析极小化 → KKT 子问题 | ⭐⭐ | 选型基础 |
| Box-DDP | Projected Newton 活动集 + $K[\mathcal C]=0$ | ⭐⭐ | 力矩限制处理 |
| AL-iLQR/ALTRO | 外层乘子 + 内层无约束 + Projected Newton 精炼 | ⭐⭐ | 通用约束处理 |
| FDDP | defect 修正 $(\star)$ + $\ell_1$ merit + infeasible warm-start | ⭐⭐ | 多射击 warm-start |
| Box-FDDP | FDDP + Box-QP | ⭐⭐ | Crocoddyl 默认 |
| ProxDDP | 近端 primal-dual AL 嵌入 Riccati | ⭐⭐⭐ | 下一代硬约束 MPC |
| Crocoddyl | ActionModel 抽象 + Solver 家族 + Pinocchio | ⭐⭐ | 框架使用能力 |
| 摩擦锥 | SOC / 多面体近似 / wrench cone | ⭐⭐ | 接触力正确性 |
| 终端约束 | Mayne 四条件 → 递归可行性 → 渐近稳定 | ⭐⭐⭐ | MPC 理论基础 |
| 收敛理论 | AL q-linear / Box q-quadratic / ProxDDP $O(1/k)$ | ⭐⭐⭐ | 算法理解深度 |

---

## 累积项目：本章新增模块

**DDP/MPC 全栈项目进度**：

| 章节 | 新增模块 | 实现目标 |
|------|---------|---------|
| 3.5 LQR | `tvlqr_solver.py` | Riccati 递推 + 反馈增益 |
| 3.9 DDP/iLQR | `ilqr_solver.py` | 完整 iLQR + cart-pole 摆起 |
| **3.10 约束 DDP** | **`box_ddp.py` + `crocoddyl_quad.py`** | **Box-QP iLQR + Crocoddyl 四足 OCP** |
| 3.11 MPC 稳定性 | `mpc_terminal.py` | 终端代价/集合设计 |
| 3.12 MPC 数值 | `acados_quad.py` | acados RTI + 四旋翼 |

本章实现目标：
1. `box_ddp.py`：在 §3.9 的 iLQR 基础上实现 BoxQP backward pass。在 cart-pole + 力矩限制 $|u| \le 10$ N 的问题上验证。
2. `crocoddyl_quad.py`：用 Crocoddyl 构建一个 HyQ/ANYmal 四足 trotting OCP，用 BoxFDDP 求解，可视化轨迹。

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Tassa-Mansard-Todorov ICRA 2014 | ⭐⭐ | Box-DDP 原始论文 |
| Howell-Jackson-Manchester IROS 2019 | ⭐⭐ | ALTRO 论文 |
| Jackson AL-iLQR Tutorial | ⭐⭐ | 极好的 AL-iLQR 推导笔记 |
| Mastalli ICRA 2020 (Crocoddyl) | ⭐⭐ | FDDP 论文 + 框架设计 |
| Mastalli Autonomous Robots 2022 | ⭐⭐⭐ | Box-FDDP 完整理论 |
| Jallet T-RO 2025 (ProxDDP) | ⭐⭐⭐⭐ | 近端约束 DDP 理论 |
| Bertsekas 1982 Projected Newton | ⭐⭐⭐⭐ | Box-QP 收敛理论原始文献 |
| Mayne et al. Automatica 2000 | ⭐⭐⭐ | MPC 稳定性四条件 |
| CMU 16-745 Lecture 10 | ⭐⭐ | Manchester 非线性轨迹优化视频 |
| Rawlings-Mayne-Diehl MPC 2e | ⭐⭐⭐ | MPC 教科书（免费 PDF） |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| Box-DDP 在边界附近振荡 | $K[\mathcal C]$ 未置零 | 1. 检查 boxQP 后 K 的活动行 2. 确认 forward pass 的 clip 一致 | §3.10.2 |
| AL-iLQR 外层不收敛 | $\mu$ 初始过大或增长过快 | 1. 打印 $\|c\|_\infty$ 每轮变化 2. 降低 $\mu_0$ 3. 减小 $\phi$ | §3.10.3 |
| FDDP gap 不收敛 | merit $\nu$ 太小 | 1. 打印 $\max\|\lambda^\star\|$ 2. 增大 $\nu$ 3. 检查 defect 修正 $(\star)$ 是否正确实现 | §3.10.4 |
| Crocoddyl solve 报 NaN | Baumgarte 参数不匹配 $\Delta t$ | 1. 检查 $\beta < 1/\Delta t$ 2. 降低 contact gains 3. 确认 URDF 惯性合理 | §3.10.5 |
| 摩擦锥约束软罚下接触力异常 | 摩擦锥方向未对齐接触面 | 1. 打印接触法向 2. 检查 `Rsurf` 旋转 3. 可视化接触力向量 | §3.10.7 |
| MPC 闭环 drift | 终端代价不足 | 1. 计算 LQR P 矩阵 2. 增大 terminal state 权重 3. 检查 Mayne 四条件 | §3.10.8 |

---

## §3.10.10 多接触运动规划的 OCP 建模 ⭐⭐⭐

### 接触调度（Contact Schedule）

混合系统划分为 $P$ 相 $\{[t_p, t_{p+1})\}$，每相绑定接触集 $\mathcal C_p \subseteq \{1,\ldots,n_\text{ee}\}$。例如四足 trot：

- Phase 1 ($0$-$0.2$s)：左前+右后着地，右前+左后摆动
- Phase 2 ($0.2$-$0.4$s)：右前+左后着地，左前+右后摆动
- 循环...

每相内的 OCP 是光滑的（固定接触 → 固定约束集）；相切换时 Jacobian 跳变但由 mode schedule 预定义。

### 动力学模型层级

| 层级 | 状态维（四足） | 方程 | 适用 | 每节点耗时 |
|------|-------------|------|------|-----------|
| Full-body | 36 | $M\ddot q + h = S^\top\tau + J_c^\top f$ | Whole-body MPC | 1-10 ms |
| Centroidal | 9 | $\dot h_G = \sum(p_i-c)\times f_i + mg$ | CoM+动量规划 | 0.1-1 ms |
| SRBD | 12 | $m\ddot p = \sum f_i - mg$, $I\dot\omega = \sum r_i\times f_i$ | 线性 MPC | 0.1 ms |
| LIPM | 2-4 | $\ddot c = (g/z_c)(c - p_\text{ZMP})$ | 步态节拍 | <0.1 ms |

**选择哪个层级？** 取决于**计算预算**和**精度需求**：

- 实时全身 MPC（如 Dantec ICRA 2021 在 Talos 上 100 Hz）→ Full-body + Pinocchio
- 步态规划+CoM 轨迹（如 MIT Cheetah 的 convex MPC）→ SRBD + QP
- 落脚点规划（如 Raibert 启发式）→ LIPM

### 摆动脚轨迹的代价设计

摆动脚需要跟踪一个"拱形"参考轨迹（抬脚→前移→落脚）。在 Crocoddyl 中：

```python
# 摆动脚 waypoint 代价
for k in swing_knots:
    # 抬脚高度目标
    foot_ref = pinocchio.SE3(np.eye(3), np.array([x_target, 0, step_height]))
    costs.addCost(f"swing_{k}",
        crocoddyl.CostModelResidual(state,
            crocoddyl.ActivationModelWeightedQuad(np.array([0,0,1,0,0,0])),  # 只罚 z
            crocoddyl.ResidualModelFramePlacement(state, foot_id, foot_ref, nu)),
        1e4)
```

**关键设计**：
- 摆动中期加大 $z$ 方向权重（确保抬脚高度）
- 摆动末期加大 $xyz$ 全方向权重（精确落脚）
- 落脚瞬间切换为 `ImpulseActionModel`（处理碰撞动力学 $M(\dot q^+ - \dot q^-) = J_c^\top\Lambda$）

### 冲量动力学

落脚（触地）瞬间，速度发生突变。Crocoddyl 用 `ActionModelImpulseFwdDynamics` 处理：

$$
M(\dot q^+ - \dot q^-) = J_c^\top\Lambda, \qquad J_c\dot q^+ = 0 \text{ (non-rebound)}
$$

解得冲量 $\Lambda = (J_c M^{-1} J_c^\top)^{-1} J_c\dot q^-$ 和落地后速度 $\dot q^+ = \dot q^- - M^{-1}J_c^\top\Lambda$。

### 多接触 OCP 的完整建模流程

把一个实际的多接触问题从物理描述转化为可求解的 OCP，需要经过以下系统化步骤。我们以四足 trotting 为例展示完整流程。

**Step 1：定义相序列（Phase Sequence）**

四足 trot 的一个完整周期包含 2 个支撑相：

| 相编号 | 支撑足 | 摆动足 | 持续时间 |
|--------|--------|--------|---------|
| Phase A | 左前(LF) + 右后(RH) | 右前(RF) + 左后(LH) | 0.2 s |
| Phase B | 右前(RF) + 左后(LH) | 左前(LF) + 右后(RH) | 0.2 s |

每相的动力学方程不同——支撑相有接触约束 $J_c\dot q = 0$，摆动相无接触。

**Step 2：为每相构建 Action Model**

每个 phase 内部的 action model 包含：
- **动力学**：接触正动力学 $M\ddot q + h = S^\top\tau + J_c^\top f$（DifferentialActionModelContactFwdDynamics）
- **阶段代价**：状态正则化 + 控制正则化 + 摩擦锥约束（软罚）+ 摆动脚轨迹跟踪
- **积分器**：Euler 或 RK4，步长 $\Delta t = 10$-$20$ ms

**Step 3：组装 ShootingProblem**

把所有 phase 的 action models 按时间顺序串联，加上终端 action model（四足站立）：

$$
\text{Problem} = [A_0, A_1, \ldots, A_{N_A-1}] \cup [B_0, B_1, \ldots, B_{N_B-1}] \cup \cdots \cup [T]
$$

**Step 4：设计代价权重**

| 代价项 | 权重量级 | 作用 |
|--------|---------|------|
| 状态正则化 $\|x - x_\text{ref}\|^2$ | $10^{-1}$ | 保持站姿不漂移 |
| 控制正则化 $\|u\|^2$ | $10^{-3}$ | 平滑力矩 |
| 摩擦锥 $Af \le 0$ | $10^{1}$ | 防止滑动 |
| 摆动脚高度 $\|z_\text{foot} - z_\text{ref}\|^2$ | $10^{4}$ | 抬脚高度（摆动中期） |
| 落脚位置 $\|p_\text{foot} - p_\text{target}\|^2$ | $10^{4}$ | 精确落脚（摆动末期） |
| CoM 位置 $\|\text{CoM} - \text{ref}\|^2$ | $10^{2}$ | 重心跟踪 |

**权重调参经验**：摩擦锥和落脚位置的权重必须远大于正则化权重（$10^4$ vs $10^{-1}$），否则优化器会"牺牲落脚精度来降低正则化代价"——这在物理上意味着机器人步子不准。

**Step 5：选择求解器并求解**

对已知步态的实时 MPC → `SolverBoxFDDP`；需要硬摩擦锥 → Aligator `SolverProxDDP`。

### 与 MPC Receding Horizon 部署

长时 OCP 截为短滚动窗 $[t, t+T_\text{MPC}]$：
- 典型 $T_\text{MPC} = 0.5$ s、$\Delta t = 20$ ms → 25 nodes
- 每个 MPC tick 做 1-3 次 FDDP 迭代 + shift warm-start
- 控制律：$u = u_0^\star + K_0(x - x_0^\star)$（Riccati feedback）
- Dantec et al. (ICRA 2021) 在 Talos 上以 100 Hz 运行 whole-body Crocoddyl MPC

**MPC 部署的四个工程关键点**：

1. **Contact schedule 必须与 MPC 窗口对齐**：如果 trot 周期为 0.4 s 而 MPC 窗口为 0.5 s，则窗口内会包含 1.25 个步态周期。末端可能卡在 phase 切换中间——需要合理处理末端 phase 的 action model。

2. **Shift warm-start 与 phase 切换的协调**：shift 操作把上一步的解平移一格。如果 shift 后新窗口的第一个节点的 phase 与旧解的第二个节点不同（phase 已切换），必须重新初始化该节点的 action model 和接触集。

3. **实时性保证**：whole-body FDDP 的单次迭代在 i7-12700H 上约 2 ms（20 节点、36 维状态）。MPC 跑 2 次迭代 = 4 ms，加上通信延迟 1 ms，总延迟 5 ms → 理论 200 Hz。实际安全运行取 100-150 Hz。

4. **Feedback gain 的使用**：在两次 MPC 求解之间，用 $u = u_0^\star + K_0(x - x_0^\star)$ 做高频反馈（500-1000 Hz）。这相当于在 MPC 的两拍之间用"冻结的 TVLQR"稳定系统——与 §3.9.10 的 RTI 思想完全一致。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：接触切换 ramp-up 力以"平滑"模态**

新手想法：落脚时让接触力从 0 线性增加到正常值（"平滑过渡"）。实际上这引入非物理拉力——因为在 ramp-up 期间法向力为正但很小，可能不满足摩擦锥。正确做法：用 `ImpulseActionModel` 处理瞬时碰撞，之后立即按正常接触力计算。

💡 **概念误区：认为"节点越多越精确"**

对固定相长度（如 $0.2$ s）的 trot，用 10 节点（$\Delta t = 20$ ms）通常就足够了。增加到 100 节点不会显著改善轨迹质量，但计算量增加 10 倍。关键是在**接触切换瞬间**加密网格，而非全局加密。

### 练习

1. 为一个 12-DoF 四足机器人设计 2 步 trotting 的 contact schedule。写出每相的接触足集和对应的 knot 数。
2. 解释为什么 centroidal 模型不能直接用于 Crocoddyl（提示：它不包含关节角度，无法直接连接 Pinocchio 的运动学）。
3. 对比"固定 $\Delta t$ 全局均匀"vs"接触切换加密"两种网格策略在解质量和计算时间上的差异。

---

## §3.10.11 OCS2 框架简介（选学）⭐⭐⭐

### 核心定位

OCS2（ETH Zurich Robotic Systems Lab 开源）采用**连续时间 SLQ**（Sequential Linear Quadratic）= 连续时间 iLQR + switched system OCP + CppAD codegen。

与 Crocoddyl 的主要区别：
- **连续时间**：Riccati ODE $-\dot S = Q + A^\top S + SA - (SB+N)R^{-1}(B^\top S + N^\top)$ 由 ODE45 积分
- **Switched system**：对模态切换时间 $\tau_i$ 也可优化 $\partial V/\partial\tau_i$
- **CppADCodeGen**：把整个动力学+积分编译为 C 代码，5-50x 加速
- **ROS 深度集成**：`ocs2_ros_interfaces` 提供 MPC server + controller client

### 应用

- Grandia 2023 (T-RO)：ANYmal 上 100 Hz whole-body NMPC，elevation map 解析为凸 steppability 约束
- `legged_control`（qiayuanl）：Unitree 四足的完整 OCS2 MPC 栈
- Minniti 2021：ANYmal-C + 机械臂协调操作

### 与 Crocoddyl 的对比

| 维度 | Crocoddyl | OCS2 |
|------|-----------|------|
| 时间域 | 离散 | 连续（内部离散化） |
| 约束 | 软罚 / Box / Intro | AL / log-barrier |
| 微分方式 | Pinocchio 解析 | CppAD CodeGen |
| 代码生成 | 无 | 有（核心加速手段） |
| ROS 集成 | 无内置 | 深度集成 |
| 社区 | 欧洲（LAAS/Edinburgh/Inria） | ETH + 中国（Unitree 生态） |
| 多线程 | OpenMP 节点并行 | 双线程（MPC server + controller） |
| 典型延迟 | 2-5 ms（whole-body） | 5-10 ms（whole-body + CodeGen） |
| 学习门槛 | 中等（Action Model 抽象） | 较高（ROS + CppAD + OCS2 特有概念） |

**为什么两个框架并存？** Crocoddyl 和 OCS2 解决的是同一类问题（whole-body MPC），但设计哲学不同。Crocoddyl 追求的是"求解器核心的极致效率"——Pinocchio 解析导数 + Box-FDDP 的工程优化。OCS2 追求的是"从仿真到实机的完整工具链"——ROS 接口 + mode schedule + CppAD 代码生成。

这种分野反映了机器人社区的两种工程文化：欧洲团队（LAAS/Inria/Edinburgh）倾向于"做好一个核心算法并发表理论论文"，ETH 团队倾向于"构建完整的系统栈并在实机上验证"。两种文化各有贡献，用户应根据自己的目标和生态偏好选择。

2024-2025 年出现的趋势是**两者的融合**：Aligator（Inria）开始提供 ROS 接口，OCS2 开始支持 Pinocchio 后端。未来两个框架可能在底层共享更多组件——Pinocchio 作为统一的刚体动力学引擎已经成为事实标准。

**如何选择？**
- 如果你的目标是**发论文、做算法研究** → Crocoddyl（Python 接口友好，实验迭代快）
- 如果你的目标是**把 MPC 部署到 ROS 实机上** → OCS2（现成的 `ocs2_ros_interfaces`，省去大量胶水代码）
- 如果你的目标是**最高性能的嵌入式 MPC** → 两者都不是最佳，考虑 acados（纯 C + BLASFEO）

**OCS2 的连续时间 SLQ 与 Crocoddyl 的离散 FDDP 在数学上的关系**

OCS2 的 SLQ（Sequential Linear Quadratic）在连续时间做 Riccati ODE 积分 $-\dot S = Q + A^\top S + SA - (SB+N)R^{-1}(B^\top S + N^\top)$，然后用积分器（ODE45）离散化。Crocoddyl 直接在离散时间做 Riccati 差分方程递推。

当 OCS2 用一阶 Euler 积分 Riccati ODE 时，两者的步方向**完全相同**。差异出现在高阶积分器上——OCS2 的 RK4 积分 Riccati ODE 理论上更精确，但实际上离散 FDDP 用 RK4 积分动力学后再做离散 Riccati 也能达到同等精度。因此两者在实践中的性能差异主要来自工程实现（CppAD vs Pinocchio、ROS 封装开销），而非数学层面。

---

## §3.10.12 Contact-Implicit 轨迹优化（选学）��⭐⭐⭐

### 核心思想

Contact-scheduled 方法预设接触序列——如果你不知道应该在哪里踩（如攀岩、复杂地形），需要让优化器**自动发现接触**。

**Posa-Cantu-Tedrake 2014 (IJRR)**：把接触作为优化变量，通过**互补约束**建模：

$$
0 \le \phi(q) \perp \lambda_n \ge 0
$$

含义：间隙 $\phi(q) \ge 0$（不穿透）、法向力 $\lambda_n \ge 0$（只推不拉）、互补 $\phi \cdot \lambda_n = 0$（有接触时间隙为零，无接触时力为零）。

### 互补约束的数值处理

互补条件 $\phi \cdot \lambda = 0$ 非光滑（L 形可行集），直接用 NLP solver 处理会导致 LICQ 违反、乘子无界。常用松弛：

$$
\phi^\top\lambda_n \le s_k, \quad s_k \downarrow 0 \text{ (外层收紧)}
$$

或 **Fischer-Burmeister 函数**：

$$
\phi_\text{FB}(a, b) = a + b - \sqrt{a^2 + b^2}
$$

$\phi_\text{FB}(a,b) = 0 \iff a \ge 0, b \ge 0, ab = 0$。零水平集是非负象限两条坐标轴的 L 形。平滑版 $\sqrt{a^2+b^2+\varepsilon^2}$ 把拐角抹圆，使 NLP 可用。

**三种互补松弛方法的对比**

| 方法 | 数学形式 | 优点 | 缺点 | 代表实现 |
|------|---------|------|------|---------|
| 线性松弛 | $\phi\lambda \le s$，$s \downarrow 0$ | 简单、通用 NLP 可用 | 外层需多次收紧 | Posa 2014 |
| Fischer-Burmeister | $\phi_\text{FB} = 0$ | 等式约束形式、无外层 | 梯度在 $(0,0)$ 不存在 | CALIPSO |
| Sigmoid 近似 | $\lambda = \max(0, \lambda + \mu\phi)$ | 单向可微 | 精度受限 | ProxDDP |

**工程经验**：线性松弛最稳健但最慢（需要 3-5 次外层收紧）。Fischer-Burmeister 配合 IPOPT 在单次 solve 中就能达到 $10^{-6}$ 精度，但需要特殊的 Hessian 正则化处理不光滑点。对实时 MPC，contact-implicit 方法目前仍不适合——计算量比 contact-scheduled 高 100 倍以上。

### 与 DDP 的混合

纯 contact-implicit 用 SNOPT/IPOPT（秒级），不适合实时。近年混合方法：
- Kim 2022 IROS：CI-DDP（forward LCP 硬、backward 松弛）
- Le Cleac'h-Howell 2024 T-RO：Fast CI-MPC
- CALIPSO（Howell 2022）：锥 AL + IPM + 可微

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为"contact-implicit 一定比 contact-scheduled 好"**

Contact-implicit 的优势是"自动发现接触"，但代价是：(a) NLP 规模增大 2-3 倍（$+\lambda, \gamma$ 变量）；(b) 互补约束引入多局部极小；(c) 求解时间 100-10000x 于 contact-scheduled。对已知步态的实时 MPC，contact-scheduled（Crocoddyl/OCS2）远远更合适。

---

## §3.10.13 约束 DDP 的收敛性理论总结 ⭐⭐⭐⭐

### 各方法收敛率汇总

| 方法 | 全局收敛 | 局部收敛率 | 条件 |
|------|---------|-----------|------|
| Box-DDP | 有（Armijo + merit） | q-quadratic（活动集稳定后） | $Q_{uu}\succ 0$ + 严格互补 |
| AL-iLQR/ALTRO | 有（AL 框架） | 外层 q-linear，阶段2 q-quadratic | LICQ + SOSC |
| FDDP | 有（$\ell_1$ merit 下降） | 未正式证明二阶，实验超线性 | $Q_{uu}\succ 0$，$\nu > \|\lambda^\star\|$ |
| ProxDDP | 有（proximal point） | 一般 $O(1/k)$，KKT 邻域 Q-linear | LICQ + SOSC |

### 关键理论贡献的依赖关系

```
Bellman (1957) → Mayne (1966) DDP
                       ↓
Bertsekas (1982) Projected Newton ──→ Tassa (2014) Box-DDP
                       ↓
Bertsekas (1982) AL theory ──→ Howell (2019) ALTRO
                       ↓
Bock-Plitt (1984) Multiple Shooting ──→ Mastalli (2020) FDDP
                       ↓
Rockafellar (1976) Proximal Point ──→ Jallet (2025) ProxDDP
```

### FDDP 收敛性的争议与最新进展

arXiv:2403.00748 "Primal-Dual iLQR"（2024）批评 FDDP 缺乏正式的二阶收敛证明。Mastalli 2020 只证明了**merit 下降方向**（Prop. 1），但没有证明收敛率。实验观察到的"超线性"可能是 gap 闭合后 Box-DDP 的 q-quadratic 特性在起作用——但理论保证缺失。

**FDDP 收敛性的三层理解**：

1. **Merit 下降（已证明）**：Mastalli 2020 Proposition 1 证明了 FDDP 的搜索方向是 $\ell_1$ merit 函数 $\Phi(x,u;\nu)$ 的下降方向——前提是 $\nu$ 足够大。这保证了 line search 一定能找到可接受的步长 $\alpha$，即**全局收敛性**（代价+gap 单调减少）。

2. **Gap 线性收缩（已证明）**：每次 forward pass 中，gap 按 $\|\bar f^{k+1}\| \le (1-\alpha^k)\|\bar f^k\|$ 收缩。当 $\alpha = 1$ 被接受时 gap 一步关闭。实际中 gap 通常在 3-5 次迭代内闭合。

3. **局部收敛率（未正式证明）**：gap 闭合后（$\bar f = 0$），FDDP 退化为标准 Box-DDP——此时 Tassa 2014 的 q-quadratic 收敛理论适用（活动集稳定后）。但 gap 闭合的**速度**（即达到 $\bar f = 0$ 需要多少步）缺乏理论界。

**实际影响**：对工程使用几乎没有影响——FDDP 在 Crocoddyl 中经过数万次实验验证，从未出现收敛问题。实验数据显示 FDDP 典型 10-30 次迭代收敛到 $\|Q_u\| < 10^{-6}$，$\|\bar f\| < 10^{-10}$。但对学术论文的"理论贡献"声称需要谨慎。

**2024-2025 年的理论进展**：

- Jallet et al. 2025 的 ProxDDP 为约束 DDP 提供了完整的收敛证明（proximal point + KKT 正则性），绕过了 FDDP 的理论空白。
- Primal-Dual iLQR（2024）提供了一个替代 FDDP 的方法，同时更新 primal 和 dual 变量，并证明了 Q-linear 收敛。
- 未来可能的方向：用 SQP 框架下的 Dennis-Moré 定理统一证明 FDDP 的超线性收敛，但需要处理 gap 闭合过程的非光滑性。

### 练习

1. 证明 Box-DDP 在活动集 $\mathcal C^\star$ 正确辨识后的步方向与"自由子空间纯 Newton"等价。
2. 解释 ProxDDP 的 $O(1/k)$ 收敛率为什么比 ALTRO 的 q-linear 慢？在什么条件下两者等价？
3. FDDP 的 defect 修正 $(\star)$ 如何影响 KKT 残差的下降？在数值上验证：有 vs 无 $(\star)$ 修正时 $\|c\|_\infty$ 的收敛曲线。

---

## §3.10.14 Warm-Starting 与实时性策�� ⭐⭐⭐

### Shifting Warm-Start 的完整流程

```
时刻 t 的解：(X*, U*, K*, λ*, f̄*)

时刻 t+1 的初始化：
  U_init[0:N-2] = U*[1:N-1]        # 控制左移
  U_init[N-1]   = K_LQR @ x_N*     # 尾部补 LQR
  X_init[0:N-1] = X*[1:N]          # 状态左移（FDDP）
  X_init[N]     = f(X*[N], U_LQR)  # 尾部前向积分
  λ_init        = λ*[1:N] ++ 0     # 乘子左移（ALTRO）
  f̄_init       = f̄*[1:N] ++ 0    # defect 左移（FDDP）
```

### 实时迭代（RTI）在约束 DDP 中的应用

对 Box-FDDP，RTI 方案：
- **Preparation**（无需新测量）：shift + 计算导数 + backward pass（含 box-QP）
- **Feedback**（新测量到达后）：$u_0 = \text{clip}(u_0^\star + K_0(x - x_0^\star), \underline u, \bar u)$

注意 feedback 阶段也需要 clip——即使 $u_0^\star$ 满足 box，$K_0\delta x$ 可能使其超出。

### 计算性能剖析

典型四足 trot（20 节点，whole-body，i7-12700H）：

| 组件 | 耗时 | 占比 |
|------|------|------|
| Pinocchio ABA + 导数 | 1.2 ms | 60% |
| Riccati backward（含 box-QP） | 0.5 ms | 25% |
| Forward rollout | 0.2 ms | 10% |
| 代价导数 + 残差计算 | 0.1 ms | 5% |
| **总计** | **2.0 ms** | 100% |

单次迭代 2 ms → 500 Hz 理论上限。实际 MPC 跑 1-3 次迭代 → 150-500 Hz 可行。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：MPC warm-start 相位错位**

如果步态相位跨越了 MPC 周期边界（如 trot 的左右切换恰好在 warm-start shift 时发生），旧解的接触模式与新问题不匹配——box-QP 的活动集完全错误，第一步 gap 爆炸。

正确做法：warm-start 时检查 contact schedule 一致性；如果相位变了，用 quasiStatic 重新初始化对应节点。

⚠️ **编程陷阱：多线程 benchmark 不可复现**

Crocoddyl 内部使用 OpenMP 并行计算导数。线程调度的不确定性导致浮点运算顺序变化→结果微变→MPC 轨迹不完全可复现。调参时设 `OMP_NUM_THREADS=1`。

### 练习

1. 实现 shift warm-start 并测量"warm-start iLQR 3 iter"vs"cold-start iLQR 50 iter"的 MPC 闭环性能差异。
2. 对 RTI 方案，测量 feedback 延迟（从新测量到达到 $u_0$ 输出的时间）。与 full iLQR 的延迟对比。
3. 在 Crocoddyl 中对比 `OMP_NUM_THREADS=1` 和 `=4` 的求解时间。加速比是多少？接近线性吗？

---

## §3.10.15 可微约束轨迹优化与学习 ⭐��⭐⭐

### 核心思想

把约束 OCP 视为**可微函数**：输入是代价/动力学参数 $\theta$，输出是最优轨迹 $\tau^\star(\theta)$。通过 KKT 不动点的隐式微分，可以计算 $\partial\tau^\star/\partial\theta$——用于端到端学习、逆强化学习、sim2real transfer。

### OptNet / CVXPYLayers

Amos-Kolter 2017 (OptNet)：凸 QP 的最优解作为神经网络层。通过 KKT 条件对 $\theta$ 求导：

$$
\frac{\partial z^\star}{\partial\theta} = -\left(\frac{\partial F}{\partial z}\right)^{-1}\frac{\partial F}{\partial\theta}
$$

其中 $F(z, \theta) = 0$ 是 KKT 驻点条件。CVXPYLayers (Agrawal 2019) 把此推广到一般凸规划。

### Theseus / DiffMPC

- **Theseus**（Meta/Facebook 2022）：可微非线性最小二乘 + Lie groups + 稀疏 Cholesky + 批量 GPU。用于 SLAM、BA、tactile learning。
- **DiffMPC**（arXiv:2510.06179, 2025）：GPU + PCG 把 Theseus/mpc.pytorch/trajax 再加速 >4x。

**应用**：把约束 OCP 作为可微 policy layer 嵌入 RL 的 actor 网络：

$$
\pi_\theta(x) = \text{OCP-Solve}(x; \theta) = u_0^\star(x; \theta)
$$

用隐式微分求 $\partial u_0^\star/\partial\theta$，通过 reward 反向传播学习代价参数 $\theta$。

### Memory of Motion（学习 Warm-Start）

Mastalli-Lembono-Fernbach-Mansard (ICRA 2020)：
1. 离线用 Loco3D 生成大量轨迹
2. 训练 k-NN/GPR/GMM 把**任务描述**（目标位置、步态类型）映射到**初始轨迹猜测** $\tilde y$
3. 用 $\tilde y$ warm-start Crocoddyl FDDP

效果：无 Memory 时 FDDP 需要 50+ 迭代；有 Memory 时 3-5 迭代收敛。这使得复杂运动（如跳跃、前空翻）也能实时规划。

2024-2025 年后继：**Diffusion Policy 作为 warm-start generator**——用扩散模型生成初始轨迹，比 k-NN 更灵活。

### ⚠️ 常见陷阱

💡 **概念误区：把 "unrolling iLQR" 当作可微 MPC**

展开 iLQR 的 20 次迭代形成计算图再 backprop——梯度会爆炸（经过 20 层 Riccati 递推的嵌套）。正确做法：**隐式微分**（只在不动点处用 KKT 隐函数定理），避免展开。

### 练习

1. 对 cart-pole iLQR，实现隐式微分 $\partial u_0^\star / \partial Q_f$。通过有限差分验证。
2. 解释为什么 Memory of Motion 的 warm-start 比"上一步 shift"更适合**新任务**（如从未执行过的跳跃高度）。
3. 设计一个实验：用 RL（PPO）学习 Crocoddyl 的代价权重 $\theta$，使四足机器人最大化前进速度。

---

## 附录 A：免费资源

以上所有资源均可免费获取，建议按顺序阅读。

| 资源 | URL |
|------|-----|
| Tassa 2014 Control-limited DDP PDF | https://roboti.us/lab/papers/TassaICRA14.pdf |
| Howell 2019 ALTRO IROS PDF | https://rexlab.ri.cmu.edu/papers/altro-iros.pdf |
| Jackson AL-iLQR Tutorial PDF | https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf |
| Mastalli 2020 Crocoddyl/FDDP | https://arxiv.org/abs/1909.04947 |
| Jallet 2025 ProxDDP | https://inria.hal.science/hal-04332348/document |
| Crocoddyl GitHub | https://github.com/loco-3d/crocoddyl |
| Aligator GitHub | https://github.com/Simple-Robotics/aligator |
| ALTRO.jl GitHub | https://github.com/RoboticExplorationLab/Altro.jl |
| OCS2 GitHub | https://github.com/leggedrobotics/ocs2 |
| MEMMO Summer School Slides | https://memory-of-motion.github.io/summer-school/ |

---

## 附录 B：视频与课程

| 课程 | 讲师 | URL |
|------|------|-----|
| CMU 16-745 Lec 10 非线性轨迹优化 | Manchester | https://www.youtube.com/watch?v=t0vaNTZIC20 |
| MEMMO Crocoddyl slides | Mastalli | https://memory-of-motion.github.io/summer-school/ |
| MIT 6.832 Underactuated Robotics | Tedrake | https://underactuated.csail.mit.edu/ |
| B 站 CMU 16-745 中英字幕 | - | https://www.bilibili.com/video/BV1KMdNYuEa2/ |

**学习路径建议**：先看 CMU 16-745 Lecture 10 的视频理解 DDP/Box-DDP 的直觉，然后读 Tassa 2014 原文掌握 Box-QP 细节，接着读 Mastalli 2020 理解 FDDP 的 defect 修正。Howell 2019 的 ALTRO 论文数学最严谨，适合需要理论深度的读者。ProxDDP（Jallet 2025）最前沿但也最抽象，建议在理解前四种方法后再阅读。

**代码学习路径**：从 Crocoddyl 的 `examples/` 目录开始（`double_pendulum.py` → `bipedal_walking.ipynb`），然后尝试修改代价权重和约束、切换求解器。当你能在 HyQ 四足上跑出稳定的 trot 步态时，就可以进入 MPC 部署阶段——这正是 §3.11-§3.12 的主题。

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| 约束 DDP 只需要在无约束 DDP 上加 `clip` 就行 | `clip` 破坏了 backward pass 的 Riccati 结构——被裁剪的分量对应的反馈增益 $K$ 行必须置零（活动集），否则 $Q_{uu}$ 的条件化信息被丢弃，导致收敛退化甚至发散 |
| 增广拉格朗日（AL）的罚参数 $\mu$ 越大收敛越快 | $\mu$ 过大导致 Hessian $Q_{uu} + \mu I_\mathcal{A}$ 的条件数恶化，内层 DDP 本身难以收敛；正确做法是从小 $\mu_0$（如 0.1-1）开始逐步递增 |
| FDDP 和 DDP 的区别只是多射击 vs 单射击 | FDDP 的核心创新是允许动力学约束在迭代过程中被违反（infeasible warm-start），通过 defect 修正项 $V_x' \leftarrow V_x' + V_{xx}'\bar{f}$ 逐步恢复可行性；这使得 warm-start 策略根本性不同 |
| Crocoddyl 的 `ActionModel` 中代价和动力学是独立的 | Crocoddyl 的 CRTP 设计将代价和动力学封装在同一个 `ActionModel` 中并共享同一次 Pinocchio 前向运动学调用——这不是偶然的 API 设计，而是为了避免重复的运动学/动力学计算 |
| 摩擦锥约束用 4 面近似（线性化锥）就足够 | 4 面锥相比真实 Coulomb 锥最大偏差约 29%，在边界情况（如斜面行走）可能导致接触力不可行；8 面或 SOC 形式更安全 |
| 终端代价 $V_f = x^\top P x$ 的 $P$ 矩阵可以随意取大值 | $P$ 必须是 DARE 的解——对应 LQR 无限时域值函数；随意取大 $P$ 破坏了 Mayne 四条件中的 A3（代价递减），可能导致 MPC 闭环不稳定 |
| ProxDDP 比 Box-DDP 总是更好 | ProxDDP 在任意约束（等式+不等式+锥）上更通用，但对简单 box 约束场景，Box-DDP 的 Projected Newton 子问题更轻量；选择取决于约束结构 |

---

## 附录 C：常见陷阱速查表（完整版）

| # | 陷阱 | 类型 | 正确做法 |
|---|------|------|---------|
| 1 | 用 `clip(u+αk+K(x̂-x))` 代替 box-QP | 编程 | 用 Tassa 2014 Projected Newton box-QP |
| 2 | 对 box-DDP 忘把 $K[\mathcal C]$ 置零 | 编程 | 活动行反馈必须为零 |
| 3 | 用 $\tanh$ squashing 处理控制界 | 设计 | 饱和区 $Q_{uu}\to 0$ 奇异；用 box-QP |
| 4 | AL $\mu$ 爆炸（每轮 ×10 致 $10^{10}$） | 设计 | 用 ALTRO 的 Projected Newton 代替罚尾 |
| 5 | AL 初始 $\mu_0$ 过大 | 设计 | $\mu_0 = 0.1$ 或 $1$，不要 $100$ |
| 6 | AL $I_\mu$ 切换致梯度不连续 | 编程 | 切换点做保护性步长限制 |
| 7 | FDDP 忘记 $(\star)$ defect 修正 | 编程 | $V_x' \leftarrow V_x' + V_{xx}'\bar f$ |
| 8 | FDDP merit $\nu$ 过小 | 设计 | $\nu > \max\|\lambda^\star\|_\infty$ |
| 9 | 对流形用欧氏差 $x_1-x_2$ | 编程 | 用 `state.diff` / $\ominus$ |
| 10 | Baumgarte $\beta > 100$ | 设计 | $\beta \in [20,50]$ for $\Delta t = 10$ ms |
| 11 | 摩擦系数不保守化 | 设计 | $\mu_\text{plan} = 0.6\mu_\text{nom}$ |
| 12 | 4-face 锥未对齐足底法向 | 编程 | rotate `Rsurf` to contact normal |
| 13 | FramePlacement 用 $\ell_2$ 直接对矩阵元素 | 设计 | 必须 $\log_{SE(3)}$ |
| 14 | Terminal cost 缺失 | 设计 | LQR $P$ 或大权重 StateReg |
| 15 | 接触切换 ramp-up 力 | 设计 | 用 ImpulseActionModel |
| 16 | MPC warm-start 相位错位 | 工程 | 检查 contact schedule 一致性 |
| 17 | 多线程下 benchmark 不可复现 | 工程 | `OMP_NUM_THREADS=1` 做调参 |

---

## 附录 D：约束 DDP 方法的演进关系图

```
1966 DDP (Mayne)
  │
  ├── 1982 Projected Newton (Bertsekas)
  │     └── 2014 Box-DDP (Tassa-Mansard-Todorov)
  │           └── 2022 Box-FDDP (Mastalli)
  │
  ├── 1982 AL Theory (Bertsekas)
  │     ├── 2019 ALTRO (Howell-Jackson-Manchester)
  │     │     └── 2020 ALTRO-C (锥约束)
  │     └── 2025 ProxDDP (Jallet-Bambade-Mansard-Carpentier)
  │
  ├── 1984 Multiple Shooting (Bock-Plitt)
  │     └── 2020 FDDP (Mastalli et al.)
  │           └── 2022 Box-FDDP
  │
  └── 2000 MPC Stability (Mayne-Rawlings)
        ├── 2005 RTI (Diehl-Bock-Schloder)
        └── 2023 Perceptive MPC (Grandia et al.)
```

---

## 附录 E：自测题（5 道，含跨章综合）

1. **（推导题）** 写出 Box-DDP 中活动集 $\mathcal C$ 已知时的自由块 Newton 方程。证明当 $\mathcal C = \varnothing$ 时退化为无约束 DDP。

2. **（理论题）** 推导 ALTRO 的外层乘子更新 $\lambda^+ = \Pi_+(\lambda + \mu c)$ 的 q-linear 收敛因子 $M/\mu$。说明为什么 $\mu$ 不需要趋于无穷（对比纯罚方法）。

3. **（连接题，跨章综合）** 从 §3.9 的 DDP backward pass 公式出发，推导 FDDP 的 $(\star)$ defect 修正。证明当 $\bar f = 0$ 时退化为标准 DDP。解释 $V_{xx}'\bar f$ 的物理含义。

4. **（实现题）** 用 Crocoddyl 构建一个四足机器人的 trot 步态 OCP（至少 2 个步态周期）。用 BoxFDDP 求解并可视化结果。报告：迭代次数、求解时间、最终 gap、力矩是否在限制内。

5. **（对比题）** 对同一个四足 trot 问题，分别用 (a) Crocoddyl BoxFDDP (b) ALTRO.jl (c) 纯 iLQR + clip 三种方法求解。对比：收敛速度、约束满足精度、最终代价。解释为什么 (c) 的解通常最差。

---

## 附录 F：时间预算（学习计划）

建议 **4-6 周**，每周 12-15 小时：

**第 1 周——约束分类 + Box-DDP**：
- 读 Tassa 2014 原文 + CMU 16-745 Lec 10
- 在 Crocoddyl `examples/double_pendulum.py` 切换 `SolverBoxDDP`
- 对比 naive clipping 的发散曲线
- 手写 box-QP Projected Newton（$m = 2$）

**第 2 周——AL-iLQR / ALTRO**：
- 读 Howell 2019 IROS + Jackson tutorial
- 在 Altro.jl 跑 `quadrotor_obstacle.jl`
- 推导外层 q-linear 收敛
- 理解 Projected Newton 阶段的作用

**第 3 周——FDDP + Crocoddyl + 多接触**：
- 读 Mastalli 2020 ICRA + 2022 Autonomous Robots
- 跑 Crocoddyl `bipedal_walking.ipynb`
- 推导 $(\star)$ defect 修正
- 改 Baumgarte 参数观察脚穿透/漂移

**第 4 周——摩擦锥 + 终端约束**：
- 读 Caron 2015 wrench cone + Mayne 2000
- 切换 FrictionCone 4-face vs 8-face
- 实现 shift warm-start MPC
- 白板推导 Mayne 四条件

**第 5-6 周（选学）——ProxDDP / OCS2 / 前沿**：
- 跑 Aligator `examples/solo12.py`
- Clone legged_control 跑 Unitree 仿真
- 读 Grandia 2023 T-RO perceptive locomotion
- 实现简单的 Memory of Motion warm-start

---

## §3.10.16 约束 DDP 的收敛诊断实践 ⭐⭐

### 如何判断约束 DDP 是否正常收敛

在实际部署中，仅看"求解器返回成功"是不够的——需要系统化地监控多个指标来确保解的质量。

**五个核心监控指标**

| 指标 | 含义 | 健康范围 | 报警阈值 |
|------|------|---------|---------|
| $\|Q_u\|_\infty$ | 一阶最优性残差 | $< 10^{-4}$ | $> 10^{-2}$ |
| $\max_k\|\bar f_k\|$ | 动力学 gap（FDDP） | $< 10^{-8}$ | $> 10^{-4}$ |
| $\|c\|_\infty$ | 约束违反量 | $< 10^{-6}$ | $> 10^{-3}$ |
| $\alpha_\text{accepted}$ | 被接受的步长 | $\ge 0.25$ | $< 2^{-5}$ |
| $\mu_\text{current}$ | 当前正则化参数 | $< 10^{-2}$ | $> 10^2$ |

**典型异常模式及诊断**

| 模式 | $\|Q_u\|$ 趋势 | $\|\bar f\|$ 趋势 | $\alpha$ | 诊断 | 修复 |
|------|---------------|-----------------|---------|------|------|
| 正常收敛 | 指数下降 | 快速闭合 | $\to 1$ | -- | -- |
| 梯度停滞 | 平台不降 | 已闭合 | $< 0.1$ | $\mu$ 过大 | 手动降 $\mu$ |
| Gap 不闭 | 不定 | 不降 | $< 0.5$ | merit $\nu$ 太小 | 增大 $\nu$ |
| 活动集振荡 | 振荡 | 已闭合 | 交替 | Box-QP 退化互补 | 加 $\varepsilon$-活动集 |
| 约束驱动 | 下降但慢 | 已闭合 | $1.0$ | AL $\mu$ 不够 | 增大 AL $\mu$ |
| 代价爆炸 | NaN | NaN | -- | 动力学失稳 | 检查 $\Delta t$、URDF |

**实时 MPC 中的简化监控**

在 MPC 中不可能打印详细日志。推荐的最小监控集是：

1. **每周期记录** `solver.cost`、`solver.iter`、`max(solver.fs)`（gap）
2. **设置硬超时**：`solver.set_callbacks([crocoddyl.CallbackTimer(budget_ms)])`
3. **如果连续 3 个 MPC 周期 iter = max_iter**：触发降级模式（切到 LQR 反馈）

> **本质洞察**：约束 DDP 的"收敛"不是一个二值状态（收敛/不收敛），而是一个多维质量空间。一阶最优性 $\|Q_u\|$、可行性 $\|\bar f\|$、约束满足 $\|c\|$ 三个指标必须**同时**达到阈值，才能称为"真正收敛"。只看其中一个（如只看代价下降）可能误判。

### 约束 DDP 与后续章节的理论桥梁

约束 DDP（本章）→ MPC 稳定性（§3.11）→ MPC 数值求解（§3.12）构成了一个完整的理论-算法-实现三层体系：

| 层次 | 关注点 | 本章贡献 | 后续章节衔接 |
|------|--------|---------|------------|
| **理论层** | 约束何时可解、解的性质 | KKT 子问题、收敛率 | §3.11 的 Mayne 四条件 |
| **算法层** | 如何高效求解 | Box-QP、AL、ProxDDP | §3.12 的 condensing、RTI |
| **实现层** | 代码、部署、调参 | Crocoddyl/Aligator | §3.12 的 acados、BLASFEO |

---

## 结语：约束 DDP 是 Bellman 与 KKT 的桥

无约束 DDP 用 Bellman 原理把无限维 OCP 降为 $N$ 次二次极小化；真实机器人把这个二次极小化变成了 box-QP（Tassa 2014）、AL 子问题（Howell 2019）、多射击 SQP（Mastalli 2020）、或 primal-dual proximal（Jallet 2025）。

> **本质洞察**：这条谱系的核心数学是——KKT 系统 + 活动集 + 乘子更新如何"嵌入"Bellman 递推而不破坏 $\mathcal O(N)$ 的结构性复杂度。Box-DDP 保住了 $\mathcal O(Nm^3)$；ALTRO 以维度膨胀 $\mathcal O(N(n+m+p)^3)$ 换来任意约束；FDDP 以"允许动力学违反"换来 infeasible warm-start；ProxDDP 用近端正则把 primal-dual AL 收进 Riccati。

**给博士前预备生的三条忠告**：

第一，**先把 Tassa 2014 的 box-QP 与 Bertsekas 1982 的 Projected Newton 看透**——这是所有后续方法的数学地基。box-QP 的分块 Cholesky + 活动集置零是所有约束 DDP 方法的原子操作——无论是 ALTRO 的阶段 2、FDDP 的 Box-FDDP 变体、还是 ProxDDP 的扩展 Riccati，底层都在反复调用这个子程序。

第二，**一定要理解 FDDP 的 $(\star)$ defect 修正**——它是把 DDP 理解为多射击 SQP condensed form 的钥匙，也是 Crocoddyl 默认 FDDP 的原因。如果你能手推 $V_x' \leftarrow V_x' + V_{xx}'\bar f$ 的来源（从多射击 KKT 的 block elimination 出发），你就真正理解了 DDP 与 SQP 的统一视角。

第三，**动手在 Crocoddyl 上跑一个四足 trot**，并尝试把 `SolverFDDP` 换成 `SolverBoxFDDP` 再换成 Aligator ProxDDP，亲眼看 solve time、constraint violation、warm-start 敏感度的差异。理论分析可以告诉你"哪个方法收敛率更好"，但只有实验能告诉你"在你的具体问题上哪个更快"——因为常数因子、内存带宽、cache 命中率等工程因素往往比渐近收敛率更重要。

第四（bonus），**学会读 Crocoddyl/Aligator 的源码**。这两个库的 C++ 代码质量极高，是学习现代 C++ 模板元编程（CRTP、表达式模板）和高性能数值计算的优秀教材。从 `ActionModelAbstractTpl` 的虚函数调度到 `SolverBoxFDDP` 的 backward pass 循环，每一行都有设计意图。

理解这些后，你就站在了 2026 年 whole-body MPC 的知识前沿，可以无缝对接 §3.11 MPC 理论与后续研究。

---
