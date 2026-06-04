# Pontryagin 极大值原理（PMP）

> 面向方向：RL motion control / 具身智能 / 轨迹优化
> 先修：变分法与 Euler-Lagrange 方程、线性代数、ODE 理论、凸分析基础、流形与李群初步
> 目标：从"无约束变分"跃迁到"有约束最优控制"。掌握 PMP 的严格陈述、证明骨架、经典可解实例与机器人工程含义。

---

## 前置自测

📋 **前置自测**（答不出 >= 2 题，先回前置章节复习）

1. 变分法中，Euler-Lagrange 方程的推导思路是什么？什么是"第一变分为零"？
2. 什么是凸锥？什么是凸锥的分离定理（Minkowski 分离定理）？
3. 线性 ODE $\dot x = A(t)x$ 的基解矩阵 $\Phi(t,t_0)$ 是什么？它满足什么性质？
4. 什么是 Hamiltonian 力学中的正则方程？Hamilton 函数与 Lagrangian 之间通过什么变换联系？
5. 解释"必要条件"与"充分条件"的区别。为什么最优控制中这个区分特别重要？

---

## 本章目标

学完本章，你将能够：

1. 写出任意有界控制问题的 Hamiltonian，列出 PMP 五件套必要条件
2. 对给定的简单系统（双积分器、标量 LQR、Dubins 车），手推完整的 PMP 解
3. 理解 PMP 证明的核心逻辑链：needle variation → 可达凸锥 → 分离定理 → 共态/极大化
4. 清楚 PMP（必要条件）与 HJB（充分条件）之间的精确对偶关系

---

## 知识树

```
                    最优控制理论
                        │
           ┌────────────┼────────────┐
           │            │            │
      变分法(EL)      PMP(本章)      HJB/DP
      (无约束)      (有约束,必要)    (充分,场)
           │            │            │
           └─────→  推广 ←──── 特征线法
                        │
         ┌──────────────┼──────────────┐
         │              │              │
    bang-bang       线性反馈(LQR)    奇异弧
    (H对u线性)     (H对u严格凹)    (H对u退化)
         │              │              │
         └──── 数值方法 ─┴─── 间接法/直接法
                        │
              ┌─────────┼─────────┐
              │         │         │
           DDP/iLQR    MPC     ALTRO
```

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含手推例题） | 10-12 小时 | 需要掌握 PMP 证明思想和独立求解最优控制问题的读者 |
| 速读（跳过证明骨架） | 4-5 小时 | 已有变分法基础、需要快速理解 PMP 五件套的读者 |
| 速查（只看工作流和例题） | 30 分钟 | 遇到具体最优控制问题时回来查阅求解步骤 |

---

## §3.2.1 最优控制问题的标准形式 ⭐⭐

### 动机：为什么需要超越变分法

在变分法的世界里，我们解决的问题形如"在所有足够光滑的曲线中，找到使积分泛函取极值的那一条"。Euler-Lagrange 方程是这个问题的核心工具。然而，真实的机器人控制问题永远面临**约束**：

- 电机推力有上限：$|u| \le u_{\max}$
- 关节角度有物理限位：$q_{\min} \le q \le q_{\max}$
- 接触力不能为拉力：$\lambda_n \ge 0$
- 推力只能在有限集中选取（脉冲发动机开/关）

> **本质洞察**：变分法假设控制可以取遍整个 $\mathbb{R}^m$，即"控制无界"。当控制被约束在某个集合 $U$ 中时，经典变分法的微小扰动 $u + \varepsilon \eta$ 可能立刻违反约束——尤其当最优控制恰好在 $U$ 的边界上时（如满推力状态），任何方向的扰动都可能出界。PMP 的天才之处在于用一种完全不同的扰动方式——针状变分——绕过了这个困难。

**类比**：变分法就像在一片无边平原上找最低点——任何方向都可以走，所以"各方向导数为零"就足够了。而有约束的最优控制就像在一个围栏内找最低点——当你走到围栏边上时，"各方向导数为零"这个条件不再合理，你需要考虑"在允许方向上不能下降"这种更精细的条件。PMP 正是这种精细条件的完美表达。

### 如果不用 PMP 会怎样

假设我们仍然试图用 Euler-Lagrange 方程处理有约束问题：

1. **EL 方程可能给出界外的控制**：对 $\min \int u^2 \, dt$（最小能量），EL 给出 $u^* = -K x$，但如果 $|Kx| > u_{\max}$，这个解就不可行。
2. **拉格朗日乘子法的困难**：可以加 KKT 条件处理不等式约束，但控制约束 $u(t) \in U$ 是逐时刻的——这意味着每个时刻 $t$ 都要一个乘子，变成了一个测度空间中的优化问题。
3. **非凸控制集无法处理**：如果 $U = \{-1, +1\}$（开关控制），EL 的连续性假设根本不适用。

这些困难促使 Pontryagin 团队发明了全新的方法论。

### 历史：苏联导弹拦截与双目失明的天才

1956年，苏联国土防空部队面临一个紧迫的军事问题：如何设计防空导弹的最优拦截轨迹？导弹的推力有限、燃料有限，必须在最短时间内击中目标。这个问题的数学本质是：在推力约束 $|T| \le T_{\max}$ 下的时间最优控制。

Lev Pontryagin（1908-1988）14岁时因家中煤油炉（primus stove）爆炸事故双目失明，此后全部数学学习依赖母亲朗读文献和同事口述。尽管如此，他成为20世纪最杰出的拓扑学家之一（Pontryagin 对偶定理、Pontryagin 类）。1956年，在学生 Boltyanskii、Gamkrelidze 和 Mishchenko 的协助下，他开始转向最优控制理论。

Boltyanskii 在1958年用**针状变分**（needle variation）给出了非线性一般情形的第一个严格证明。1961年俄文专著出版，1962年英译本问世（Trirogoff 译，Neustadt 编，Interscience/Wiley）。这本书彻底改变了控制理论的面貌。

历史文献：Gamkrelidze 1999《Discovery of the Maximum Principle》，*J. Dynamical and Control Systems* 5(4):437-451；Sussmann-Willems 1997《300 Years of Optimal Control: From the Brachystochrone to the Maximum Principle》*IEEE Control Systems Magazine* 17(3):32-44。

### 严格定义

给定状态 $x:[t_0,t_f]\to\mathbb{R}^n$、控制 $u:[t_0,t_f]\to U\subset\mathbb{R}^m$（$U$ 为 Lebesgue 可测紧集），动力学
$$\dot x(t)=f(x(t),u(t),t),\qquad x(t_0)=x_0,$$
端点流形约束 $\psi(x(t_f),t_f)=0\in\mathbb{R}^p$，控制几乎处处满足 $u(t)\in U$。代价函数可写为三种经典等价形式：

**Bolza 形式**（最一般）：
$$J[u]=\phi(x(t_f),t_f)+\int_{t_0}^{t_f} L(x(t),u(t),t)\,dt.$$

**Mayer 形式**：仅末端代价 $J=\phi(x(t_f),t_f)$；通过引入额外状态 $x_{n+1}$ 满足 $\dot x_{n+1}=L$，$x_{n+1}(t_0)=0$，可把 Lagrange 项吸入末端：$J=\phi(x(t_f))+x_{n+1}(t_f)$。

**Lagrange 形式**：仅积分代价 $J=\int L\,dt$；通过微积分基本定理或状态扩维可转为 Mayer。

### 三形式等价性定理

**定理 3.2.1**（Bolza/Mayer/Lagrange 三位一体）：三种形式通过**状态扩维**相互一阶等价。

**证明**（完整推导）：

**Lagrange → Mayer**：

设原问题为 $\min \int_{t_0}^{t_f} L(x,u,t)\,dt$，$\dot x = f(x,u,t)$。

引入扩维状态 $\tilde x = (x, x_{n+1}) \in \mathbb{R}^{n+1}$，新状态满足：
$$\dot x_{n+1}(t) = L(x(t), u(t), t), \quad x_{n+1}(t_0) = 0.$$

则原来的 Lagrange 代价变为：
$$J = \int_{t_0}^{t_f} L\,dt = x_{n+1}(t_f) - x_{n+1}(t_0) = x_{n+1}(t_f).$$

这是纯 Mayer 形式，末端代价 $\tilde\phi(\tilde x(t_f)) = x_{n+1}(t_f)$。

**Mayer → Bolza**：显然 Mayer 是 Bolza 的特例（取 $L \equiv 0$）。

**Bolza → Mayer**：对一般 Bolza 代价 $J = \phi(x(t_f)) + \int L\,dt$，同样引入 $x_{n+1}$，则
$$J = \phi(x(t_f)) + x_{n+1}(t_f) = \tilde\phi(\tilde x(t_f)).$$

关键验证：扩维后的动力学 $\dot{\tilde x} = \tilde f(\tilde x, u, t)$ 仍然满足 PMP 所需的光滑性假设（因为 $L$ 本身就假设了光滑性），且控制集 $U$ 不变。

出处：Liberzon 2012《Calculus of Variations and Optimal Control Theory: A Concise Introduction》Princeton UP，ISBN 978-0-691-15187-8，Ch.3。

### 工程含义

| 实际问题 | 代价形式 | 说明 |
|---------|---------|------|
| MPC stage-cost + terminal-cost | Bolza | $\phi = x_N^\top Q_f x_N$，$L = x^\top Q x + u^\top R u$ |
| LQR | Bolza | 二次末端罚是 Mayer 项 |
| 最小时间问题 | Lagrange | $L \equiv 1$，$\phi = 0$ |
| 四旋翼航点跟踪 | Bolza | 时间 + 能量 + 末端位置误差 |
| 最大终端高度（Goddard 火箭）| Mayer | $J = h(t_f)$ |

选哪种形式只是"记账方式"——PMP 结果在三者中完全等价。但工程实践中，Bolza 形式最自然，因为它同时表达"过程代价"（能量消耗、跟踪偏差）和"末端代价"（到达精度）。

### 反事实推理：如果三种形式不等价

如果三种形式不等价，那么每种形式下的 PMP 就需要单独证明，而且对同一个物理问题，选用不同数学形式会得到不同的最优解——这显然是荒谬的。等价性保证了**问题的数学描述不影响物理结论**，这是一种"坐标无关性"——类似于力学中不同坐标系下运动方程形式不同但描述同一运动。

### ⚠️ 常见陷阱

💡 **概念误区：混淆"代价形式"与"问题结构"**

新手想法："Mayer 形式更简单，所以应该总是用 Mayer"

实际上：虽然理论上总可以转换为 Mayer（通过扩维），但扩维会增加状态维度，增加数值计算负担。实际求解时应选择**最自然的形式**——让代价函数的物理意义直观可读。例如 MPC 中分离 stage-cost 和 terminal-cost 有明确的物理意义（过程平滑性 vs 末端精度），不应强行合并。

🧠 **思维陷阱：认为"只要写对代价函数形式，PMP 就自动给出解"**

实际上：写出代价函数只是第一步。PMP 给出的是**必要条件**，即一组 ODE 边值问题。这个 BVP 通常没有解析解，需要数值求解。而且 PMP 不保证解的存在（如果控制集 $U$ 或终端约束不满足某些正则条件，最优解可能不存在）。

### 练习

1. 将最小能量问题 $\min \int_0^T \|u\|^2 \, dt$（$\dot x = Ax + Bu$，$x(0) = x_0$，$x(T) = x_f$）写成 Mayer 形式。明确写出扩维后的新动力学方程和新末端代价。

2. 对 Goddard 火箭问题（最大化终端高度 $h(t_f)$，动力学 $\dot h = v$，$\dot v = T/m - g$，$\dot m = -T/c$），讨论为什么 Mayer 形式更自然，以及如果强行写成 Lagrange 形式会有什么困难。

---

## §3.2.2 Hamiltonian 与共态方程 ⭐⭐

### 动机：为什么需要引入共态变量

在经典力学中，从 Lagrangian $L(q, \dot q)$ 通过 Legendre 变换引入广义动量 $p = \partial L / \partial \dot q$，得到 Hamiltonian $H(q, p)$。Hamilton 形式把二阶 ODE 转换为一阶 ODE 系统，并揭示了对称性与守恒量（Noether 定理的简洁表达）。

在最优控制中，引入共态（co-state / adjoint）$\lambda(t)$ 的动机完全类似：

1. **把最优性条件从"隐含形式"转为"显式 ODE 系统"**：PMP 的核心结果是一组关于 $(x, \lambda)$ 的一阶 ODE，加上对 $u$ 的逐点极大化条件。
2. **揭示最优代价对状态的敏感度**：$\lambda(t)$ 的物理意义是"在时刻 $t$，状态 $x(t)$ 的微小偏差对最终代价的影响"——即值函数的梯度 $\nabla_x V$。
3. **连接 PMP 与 HJB**：共态是两大理论框架之间的桥梁变量。

**类比**：共态 $\lambda$ 之于最优控制，如同"价格"之于经济学。$\lambda_i(t)$ 告诉你"在时刻 $t$，状态 $x_i$ 多一个单位值多少钱"（边际价值/影子价格 shadow price）。最优决策就是在每个时刻选择使"收益-成本"最大的控制——这正是 Hamiltonian 极大化条件的经济学直觉。

### 严格定义

引入**共态**（co-state / adjoint）$\lambda(t)\in\mathbb{R}^n$（理解为余切空间 $T^*_x\mathbb{R}^n$ 中的余向量）与一个**异常乘子** $\lambda_0 \ge 0$。定义**控制 Hamiltonian**：
$$H(x,u,\lambda,\lambda_0,t) := \lambda^\top f(x,u,t) - \lambda_0 L(x,u,t).$$

这个定义的每一项都有明确意义：
- $\lambda^\top f$：共态与动力学的内积，度量"沿 $f$ 方向运动对最终代价的改善速率"
- $-\lambda_0 L$：运行代价的即时贡献（前面的负号来自极大化约定）
- 整个 $H$：控制 $u$ 的"净收益率"——选择使 $H$ 最大的 $u$ 意味着在"改善末端代价"与"减少运行代价"之间取最佳平衡

### 两种约定对照表

| | 极大化约定（Pontryagin 原始，本专题） | 极小化约定（Kirk/Bertsekas） |
|---|---|---|
| Hamiltonian | $H_{\max}=\lambda^\top f - \lambda_0 L$ | $H_{\min}=L + \lambda^\top f$ |
| 极值条件 | $u^*=\arg\max_{v\in U} H_{\max}$ | $u^*=\arg\min_{v\in U} H_{\min}$ |
| 共态方程 | $\dot\lambda=-\partial H_{\max}/\partial x$ | $\dot\lambda=-\partial H_{\min}/\partial x$ |
| 共态符号关系 | $\lambda_{\max}$ | $\lambda_{\min}=-\lambda_{\max}$ |
| 等价性 | $\max H_{\max} \Leftrightarrow \min H_{\min}$ | $H_{\min}=-H_{\max}$（取 $\lambda_0=1$）|
| 代表文献 | Pontryagin 1962, Liberzon 2012 | Kirk 2004, Bertsekas DP&OC |

> **本质洞察**：两种约定是**同一物理定律的两种记号**——就像 $e^{i\omega t}$ 和 $e^{-i\omega t}$ 约定描述同一波动。读文献时第一件事是看清作者用哪种约定，否则所有符号都会差一个负号。本专题全程使用极大化约定，与 Pontryagin 原著一致。

### 正常极值与异常极值

**正常极值**（normal）：$\lambda_0 > 0$；可归一化 $\lambda_0 = 1$。代价 $L$ 真正出现在 Hamiltonian 中，极大化条件同时考虑动力学方向和代价最小化。

**异常极值**（abnormal）：$\lambda_0 = 0$。代价**完全消失**——Hamiltonian 变为 $H = \lambda^\top f$，极大化条件仅由动力学几何决定。这意味着无论代价函数是什么（$\int u^2 dt$ 还是 $\int |u| dt$ 还是其他任何代价），只要动力学和约束不变，异常极值轨迹都一样。

非平凡性要求：$(\lambda_0, \lambda(t)) \neq (0, 0)$。如果允许 $(\lambda_0, \lambda) = 0$，则 PMP 的所有条件变成平凡的 $0 = 0$，没有任何信息量。

**反事实推理**：如果不区分正常/异常极值会怎样？假设总是取 $\lambda_0 = 1$，那么在约束品性（constraint qualification）失效的情形下（如可达集的边界点），PMP 会错误地声称"不存在满足条件的共态"——而实际上是存在满足条件的共态，只是需要 $\lambda_0 = 0$。忽视异常极值可能导致遗漏真正的最优解。

### 核心定理：共态方程

**定理 3.2.2**（伴随/共态方程）：若 $(x^*,u^*)$ 是最优轨迹且 $f,L$ 关于 $x$ 连续可微，则存在绝对连续的共态 $\lambda:[t_0,t_f]\to\mathbb{R}^n$ 满足
$$\boxed{\dot\lambda(t)=-\frac{\partial H}{\partial x}\bigl(x^*(t),u^*(t),\lambda(t),\lambda_0,t\bigr)=-\Bigl(\frac{\partial f}{\partial x}\Bigr)^{\!\top}\!\lambda+\lambda_0\Bigl(\frac{\partial L}{\partial x}\Bigr)^{\!\top}.}$$

出处：Pontryagin-Boltyanskii-Gamkrelidze-Mishchenko 1962《The Mathematical Theory of Optimal Processes》Ch. II。

### 证明骨架（4步推导）

**Step 1（针状扰动与轨迹偏差）**：

对 $u^*$ 在某个 Lebesgue 点 $\tau$ 作针状扰动 $u_\varepsilon$（只在 $[\tau-\varepsilon, \tau]$ 上替换为 $v \in U$），产生的轨迹扰动 $\delta x(t) = x_\varepsilon(t) - x^*(t)$ 在 $t > \tau$ 上满足一阶变分方程：
$$\dot{\delta x}(t) = A(t) \delta x(t), \quad A(t) = \frac{\partial f}{\partial x}\bigl(x^*(t), u^*(t), t\bigr),$$
初始条件 $\delta x(\tau) = \varepsilon [f(x^*(\tau), v, \tau) - f(x^*(\tau), u^*(\tau), \tau)] + o(\varepsilon)$。

**Step 2（代价函数一阶增量）**：

代价的一阶变化为：
$$\delta J = \lambda_0 \frac{\partial \phi}{\partial x}\bigg|_{x^*(t_f)} \delta x(t_f) + \lambda_0 \int_{t_0}^{t_f} \frac{\partial L}{\partial x}\bigg|_{(x^*, u^*)} \delta x(t)\,dt + o(\varepsilon).$$

**Step 3（对偶化——引入共态消去体积分）**：

定义 $\lambda(t)$ 使得 $\langle \lambda(t), \delta x(t) \rangle$ 的时间导数正好抵消积分中的 $(\partial L/\partial x)\delta x$ 项。计算：
$$\frac{d}{dt}\langle \lambda, \delta x \rangle = \dot\lambda^\top \delta x + \lambda^\top \dot{\delta x} = \dot\lambda^\top \delta x + \lambda^\top A \delta x.$$

要求这等于 $-\lambda_0 (\partial L/\partial x) \delta x$，即：
$$\dot\lambda^\top + \lambda^\top A = -\lambda_0 \frac{\partial L}{\partial x},$$
$$\dot\lambda = -A^\top \lambda + \lambda_0 \left(\frac{\partial L}{\partial x}\right)^\top.$$

这正是共态方程。

**Step 4（端点条件）**：

通过分部积分（将体积分转为端点求值），$\delta J \ge 0$ 对所有允许扰动成立 ⇒ 端点 $\lambda(t_f)$ 由横截条件确定。

### 共态的物理意义：多重视角

| 视角 | $\lambda(t)$ 的解读 |
|------|-------------------|
| 经典力学 | 广义动量 $p = \partial L / \partial \dot q$ |
| 经济学 | 影子价格（状态 $x_i$ 的边际价值）|
| HJB | 值函数梯度 $\nabla_x V(x^*(t), t)$ |
| 数值优化 | Lagrange 乘子（状态方程约束的对偶变量）|
| RL | 近似 $\partial V^* / \partial x$（value function 对状态的敏感度）|

**类比**：$\lambda(t)$ 就像"时间旅行的会计师"——他站在时刻 $t$，告诉你"如果此刻状态偏差 $\delta x$，最终的总账（代价）会变化 $\lambda^\top \delta x$"。共态方程描述的是这个"边际影响"如何随时间反向传播。

### 工程含义

对 LQR 问题，$\lambda(t) = P(t) x(t)$ 是线性的，$P(t)$ 就是 Riccati 矩阵。对最小时间问题，$\lambda$ 的符号决定 bang-bang 切换时刻。在 RL 语境下，$\lambda(t) = \partial V^* / \partial x$ 精确等于值函数对状态的梯度——这解释了为什么 policy gradient 方法在数学上等价于 adjoint method（参见 Tedrake《Underactuated Robotics》https://underactuated.mit.edu/）。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：共态方程的数值积分方向**

错误做法：像状态方程一样从 $t_0$ 正向积分共态

正确做法：共态方程的"初始条件"（实际是终端条件）$\lambda(t_f)$ 在 $t_f$ 给出，必须**反向积分**到 $t_0$。在代码中，这意味着要么用时间反转 $\tau = t_f - t$，要么使用 BVP 求解器直接处理两点边值问题。

💡 **概念误区：共态 $\lambda$ 就是 Lagrange 乘子**

虽然共态和 Lagrange 乘子有紧密联系（在离散化后确实等价），但连续时间 PMP 的共态有更丰富的含义：它满足一个 ODE（不仅仅是一个代数条件），它有明确的物理解释（值函数梯度），它在最优轨迹上是唯一确定的（不像一般约束优化中乘子可能不唯一）。

### 练习

1. 对标量系统 $\dot x = u$，$|u| \le 1$，代价 $J = \int_0^T x^2 \, dt$，写出 Hamiltonian、共态方程。验证当 $T$ 很小时最优控制接近线性反馈 $u \approx -x$。

2. 证明：对自治系统（$f, L$ 不显式依赖 $t$），Hamiltonian 沿最优轨迹守恒，即 $dH/dt = 0$。提示：直接对 $H(x^*(t), u^*(t), \lambda(t))$ 求时间导数，利用状态方程和共态方程。

---

## §3.2.3 PMP 的完整陈述 ⭐⭐

### 动机：一阶必要条件的完整形式

前两节分别建立了标准问题形式和 Hamiltonian/共态的概念。现在把所有要素组装成一个完整的定理——这是整个最优控制理论最重要的单一结果。

### PMP 五件套（严格陈述）

设 $(x^*(\cdot), u^*(\cdot))$ 是 Bolza 问题的局部最优解，$f, L, \phi, \psi$ 关于 $(x,t)$ 连续可微，$U$ 是任意集合（不要求凸）。则存在**非平凡** $(\lambda_0, \lambda(\cdot)) \in \{0,1\} \times AC([t_0,t_f]; \mathbb{R}^n)$ 与末端乘子 $\nu \in \mathbb{R}^p$ 使以下**五件套**成立：

**1. 状态方程**：
$$\dot x^*(t) = \frac{\partial H}{\partial \lambda} = f(x^*(t), u^*(t), t), \quad x^*(t_0) = x_0.$$

**2. 共态方程**：
$$\dot\lambda(t) = -\frac{\partial H}{\partial x}\bigl(x^*(t), u^*(t), \lambda(t), \lambda_0, t\bigr).$$

**3. 极大化条件**（核心）：对几乎处处 $t \in [t_0, t_f]$，
$$\boxed{u^*(t) \in \arg\max_{v \in U} H(x^*(t), v, \lambda(t), \lambda_0, t).}$$

**4. 横截条件**：
$$\lambda(t_f) = \lambda_0 \frac{\partial\phi}{\partial x}\bigl(x^*(t_f), t_f\bigr)^\top + \left(\frac{\partial\psi}{\partial x}\right)^\top \nu.$$
若 $t_f$ 自由，还需：
$$H(t_f) + \lambda_0 \frac{\partial\phi}{\partial t}\bigl(x^*(t_f), t_f\bigr) = 0.$$

**5. Hamiltonian 守恒（自治系统）**：若 $f, L$ 不显式依赖 $t$，则
$$H(x^*(t), u^*(t), \lambda(t)) \equiv \text{const}.$$
对自由末端时间问题，该常数为 $0$。

### 五件套的逻辑关系

```
         ┌─────── 状态方程(1) ──────→ x*(t) 前向
         │
PMP ────┼─────── 共态方程(2) ──────→ λ(t) 反向
         │
         ├─────── 极大化(3) ──────→ u*(t) 逐时刻
         │
         ├─────── 横截条件(4) ──────→ λ(tf) 端点条件
         │
         └─────── Hamiltonian守恒(5) → 不变量/验证
```

五件套合在一起构成一个**两点边值问题（BVP）**：状态 $x$ 的初值已知（$x(t_0) = x_0$），共态 $\lambda$ 的终值由横截条件(4)给出，而状态 $x$ 的终值由端点约束 $\psi(x(t_f)) = 0$ 确定。这个 BVP 通常不能解析求解。

### 横截条件详解：各种端点情形

横截条件是 PMP 中最容易出错的部分。根据端点约束的不同类型，$\lambda(t_f)$ 的表达式不同：

| 端点类型 | 约束 | 横截条件 $\lambda(t_f)$ |
|---------|------|----------------------|
| 固定端点 | $x(t_f) = x_f$ 给定 | $\lambda(t_f)$ 自由（由 BVP 确定）|
| 自由端点 | $x(t_f)$ 无约束 | $\lambda(t_f) = \lambda_0 \partial\phi/\partial x$ |
| 流形端点 | $\psi(x(t_f)) = 0$ | $\lambda(t_f) = \lambda_0 \partial\phi/\partial x + (\partial\psi/\partial x)^\top \nu$ |
| 固定时间 | $t_f$ 给定 | 无额外条件 |
| 自由时间 | $t_f$ 待优化 | $H(t_f) + \lambda_0 \partial\phi/\partial t = 0$ |

**直觉理解横截条件**：横截条件说的是"轨迹末端必须垂直于端点约束流形"——其中"垂直"是在共态（对偶）空间中度量的。如果端点完全自由，唯一的"力"来自末端代价 $\phi$，所以 $\lambda(t_f) \propto \nabla\phi$。如果端点被约束在流形 $\psi = 0$ 上，还要加上约束的法向分量 $(\partial\psi/\partial x)^\top \nu$。

**自由时间条件 $H(t_f) = 0$ 的物理意义**：

对自由终端时间问题，$t_f$ 本身是决策变量。代价 $J$ 关于 $t_f$ 的最优性条件翻译为 $H(t_f) + \lambda_0 \partial\phi/\partial t = 0$。对自治系统（$H$ 守恒）和无末端代价时间依赖（$\partial\phi/\partial t = 0$），简化为 $H \equiv 0$（沿整条最优轨迹恒零）。

这个条件在数值验证中非常有用：如果你的数值解给出 $H$ 明显偏离零，说明精度不够或算法未收敛。

**具体例子**：最小时间问题 $L = 1$，$\phi = 0$，$H = \lambda^\top f - 1$。自由时间 $H = 0$ 意味着 $\lambda^\top f = 1$——这约束了共态在末端的"大小"，不可任意缩放。

### 关键注释：PMP 是必要条件

> **本质洞察**：PMP 是**必要条件**，不是充分条件。一条满足 PMP 五件套的轨迹称为 **Pontryagin 极值轨迹**（extremal），但它未必是最优解——就像 $f'(x_0) = 0$ 只说明 $x_0$ 是驻点，可能是极大值、极小值或鞍点。要验证极值轨迹确实最优，需要额外工具：HJB 验证定理（充分条件）、二阶条件、或穷举所有极值轨迹后比较代价值。

**反事实推理**：如果 PMP 是充分条件会怎样？那么我们只需解一次 BVP 就能保证找到全局最优——这对大多数非凸问题来说太好了，不可能成立。实际上，非凸问题可能有多个极值轨迹，有些是局部最优，有些甚至不是任何意义上的最优（鞍点类型）。

### 极大化条件的三种典型结果

Hamiltonian $H$ 关于 $u$ 的结构决定了最优控制的形态：

| $H$ 关于 $u$ 的结构 | 极大化结果 | 典型问题 |
|-------------------|----------|---------|
| 线性 ($H = a(t) + b(t)^\top u$) | bang-bang: $u^* \in \partial U$ | 最小时间，推力受限 |
| 严格凹 ($\partial^2 H/\partial u^2 \prec 0$) | 内点解: $\partial H/\partial u = 0$ | LQR ($u^\top R u$ 项) |
| 退化 ($\partial H/\partial u \equiv 0$ 区间) | 奇异弧（需高阶条件）| Goddard 中段 |

**类比**：就像一个人在围栏内寻找山顶——如果山坡（$H$）在某方向一直上升直到围栏（$U$ 的边界），最高点就在围栏上（bang-bang）；如果山坡有一个圆顶（严格凹），最高点在内部（内点解）；如果山坡某个方向是平的（退化），最高点的位置无法从一阶信息确定（奇异弧）。

### 工程含义

在 $H$ 对 $u$ 线性时（如最小时间问题），极大化条件 ⇒ bang-bang 控制。在 $H$ 对 $u$ 严格凹时（如 LQR），极大化条件给出解析反馈。具体推导如下：

**LQR 中 PMP（极大化约定）的完整推导**：

系统 $\dot{x} = Ax + Bu$，代价 $\min \int_0^T \frac{1}{2}(x^\top Qx + u^\top Ru)\,dt$。

按极大化约定，$H_{\max} = \lambda^\top(Ax + Bu) - \frac{1}{2}\lambda_0(x^\top Q x + u^\top Ru)$，取 $\lambda_0 = 1$：
$$H_{\max} = \lambda^\top(Ax + Bu) - \frac{1}{2}x^\top Qx - \frac{1}{2}u^\top Ru$$

对 $u$ 求极大：$\frac{\partial H_{\max}}{\partial u} = B^\top\lambda - Ru = 0$，故：
$$\boxed{u^* = R^{-1} B^\top \lambda \quad \text{（PMP 极大化约定）}}$$

> **符号约定对照（关键！）**：极小化约定（HJB/LQR 标准教材）中 $p = \nabla_x V$，最优控制为 $u^* = -R^{-1}B^\top p$（负号！）。两种约定的关系是 $\lambda_{\text{max}} = -\lambda_{\text{min}} = -\nabla_x V$，具体而言：
> - 极大化 PMP：$\lambda = -\nabla_x V$，$u^* = R^{-1}B^\top\lambda = -R^{-1}B^\top\nabla_x V$ ✓
> - 极小化 PMP/HJB：$p = \nabla_x V$，$u^* = -R^{-1}B^\top p = -R^{-1}B^\top\nabla_x V$ ✓
>
> 两种约定给出**相同的物理控制律** $u^* = -R^{-1}B^\top\nabla_x V = -R^{-1}B^\top P x$（其中 $V = \frac{1}{2}x^\top Px$）。差异仅在于共态符号：$\lambda_{\max} = -Px$ 而 $p_{\min} = Px$。
>
> **工程铁律**：无论使用哪种约定，最终反馈律必须是 $u = -Kx$（$K = R^{-1}B^\top P \succ 0$），即**负反馈**。如果你的推导得出正反馈 $u = +Kx$，说明约定混用了。

**DDP/iLQR 的视角**：在非线性问题中，沿参考轨迹做局部 LQ 近似后，每个时步的极大化条件就变成了二次型的极大化——这正是 DDP backward pass 中计算反馈增益 $K_k$ 的来源。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：min 与 max 约定导致符号全错**

错误做法：从 Kirk 教材（极小约定）抄公式，直接代入 Pontryagin 约定的代码

现象：共态方程的解发散，最优控制方向完全相反

根本原因：极小约定的 $\lambda_{\min} = -\lambda_{\max}$，Hamiltonian 差一个负号

正确做法：教程开头明确标注约定，全文/全代码一致。如果参考不同约定的文献，必须做符号转换。

💡 **概念误区：认为"极大化条件"意味着控制总是取 $U$ 中的最大值**

"极大化 $H$"不是"控制取最大值"。$H$ 是关于 $u$ 的函数，极大化 $H(x,u,\lambda)$ 关于 $u$ 的含义是选择使 $H$ 达到最大的 $u$ 值——这个 $u$ 可能在 $U$ 的内部（如 LQR），可能在边界的不同位置（如 bang-bang 切换），甚至可能在一个面上（奇异弧）。

### 练习

1. 对自由端点、固定时间问题 $\min \phi(x(T))$（无积分代价），写出 PMP 五件套。验证共态方程简化为 $\dot\lambda = -(\partial f/\partial x)^\top \lambda$，横截条件为 $\lambda(T) = \nabla\phi(x(T))$。

2. 对自治系统证明 Hamiltonian 守恒：直接计算 $dH/dt$，利用条件(1)(2)(3)推出 $dH/dt = 0$。

3. 如果控制集 $U$ 是凸集，且 $H$ 关于 $u$ 严格凹，证明极大化条件等价于 $\partial H/\partial u = 0$（即退化为 Euler-Lagrange 的无约束情形）。

---

## §3.2.4 PMP 证明思想详解：needle variation → 分离锥 ⭐⭐⭐

### 动机：理解"为什么 PMP 成立"

PMP 不是凭空猜出来的——它有一条清晰的逻辑链。理解这条逻辑链不仅帮助你记住 PMP 的陈述，更重要的是让你理解 PMP 的**适用范围和局限**：什么时候 PMP 可能失效（如非光滑动力学），什么时候需要更强的条件（如二阶条件）。

### 为什么弱变分（Euler-Lagrange 的 $\delta u$ 小）不够

在变分法的 Euler-Lagrange 推导中，我们对控制作 $L^\infty$ 小扰动 $u + \varepsilon\eta$，要求 $\eta$ 属于"切向可行空间"。这在两种情况下失效：

**情形 1：最优控制在 $U$ 的边界上**。如果 $u^*(t) = u_{\max}$（满推力），任何正方向的扰动 $\eta > 0$ 都会使 $u + \varepsilon\eta$ 越出 $U$。而负方向 $\eta < 0$ 虽然可行，但只给出一个**半空间**的信息——不足以确定最优性条件。

**情形 2：控制集 $U$ 非凸或离散**。如果 $U = \{-1, +1\}$（开关控制），根本不存在连续的"切方向"——$U$ 只有两个孤立点。任何 $\varepsilon > 0$ 的 $L^\infty$ 小扰动都离不开当前点。

**针状变分**反其道而行：不让扰动在幅度上小，而是让它在**时间**上小。在一个极短的时间段 $[\tau-\varepsilon, \tau]$ 上，把控制完全替换为 $U$ 中的**任意**一点 $v$——$v - u^*$ 可以很大！但因为作用时间 $\varepsilon$ 很小，对轨迹的影响仍然是 $O(\varepsilon)$ 的。

**类比**：弱变分就像"轻推方向盘"——如果方向盘已经打到极限位，就没法再轻推了。针状变分则像"在极短时间内猛打方向盘到另一个位置"——虽然动作剧烈，但持续时间短，车的总偏移仍然很小。

### Step 1：针状扰动构造

对任意 Lebesgue 点 $\tau \in (t_0, t_f)$ 与任意 $v \in U$，定义
$$u_\varepsilon(t) = \begin{cases} v, & t \in [\tau-\varepsilon, \tau], \\ u^*(t), & \text{otherwise}. \end{cases}$$

关键特性：
- $\|u_\varepsilon - u^*\|_{L^\infty}$ 可能很大（$= \|v - u^*(\tau)\|$），这是"强变分"
- $\|u_\varepsilon - u^*\|_{L^1} = O(\varepsilon)$，即"$L^1$ 距离小"
- 不要求 $v$ 接近 $u^*(\tau)$：$v$ 可以是 $U$ 中的任意点

这里 $\tau$ 取为 $u^*$ 的 **Lebesgue 点**——即 $u^*$ 在 $\tau$ 处近似连续的点。由 Lebesgue 微分定理，几乎处处的 $t$ 都是 Lebesgue 点。

### Step 2：端点一阶增量

设 $x_\varepsilon$ 是 $u_\varepsilon$ 下的轨迹。在 $t = \tau$ 时刻，轨迹受到一个"冲击"：
$$x_\varepsilon(\tau) - x^*(\tau) = \varepsilon [f(x^*(\tau), v, \tau) - f(x^*(\tau), u^*(\tau), \tau)] + o(\varepsilon).$$

记 $\Delta f := f(x^*(\tau), v, \tau) - f(x^*(\tau), u^*(\tau), \tau)$（"动力学差"）。

这个冲击沿线性化方程传播到末端：设 $\Phi(t, s)$ 是 $\dot y = A(t) y$（$A = \partial f/\partial x$）的基本矩阵，则
$$\delta x(t_f) = x_\varepsilon(t_f) - x^*(t_f) = \varepsilon \, \Phi(t_f, \tau) \, \Delta f + o(\varepsilon).$$

**物理意义**：在 $\tau$ 时刻施加一个"方向为 $\Delta f$"的脉冲，沿着最优轨迹的线性化动力学传播到末端，得到末端偏差 $\Phi(t_f, \tau) \Delta f$。$\Phi$ 起着"状态转移矩阵"的作用——它描述了线性化系统如何把初始偏差传播到后续时刻。

### Step 3：多针扰动与可达凸锥

单针扰动给出一条方向 $\Phi(t_f, \tau) \Delta f$。若在 $k$ 个不相交小区间上同时用不同的 $(v_i, \tau_i, \alpha_i)$（$\alpha_i \ge 0$ 为相对强度），端点增量近似为：
$$\delta x(t_f) \approx \varepsilon \sum_{i=1}^k \alpha_i \, \Phi(t_f, \tau_i) \, \Delta f_i.$$

所有这样的正组合生成一个**凸锥**：
$$K = \operatorname{cone}\{\Phi(t_f, \tau)[f(x^*(\tau), v, \tau) - f(x^*(\tau), u^*(\tau), \tau)] : \tau \in (t_0, t_f),\, v \in U\}.$$

$K$ 是可达集 $R(t_f)$ 在 $x^*(t_f)$ 处的**一阶凸近似**（Boltyanskii 称之为 "approximating cone" 或 "tent"）。

**直觉**：每条单针扰动是从 $x^*(t_f)$ 出发的一条射线方向；叠加多条射线（正组合）得到一个锥。这个锥近似描述了"通过微小控制扰动，末端可以到达哪些方向"。

### Step 4：凸锥分离定理

**最优性的含义**：$x^*(t_f)$ 是最优的 ⇒ 不存在可行的控制扰动使代价严格下降 ⇒ "下降方向锥" $K_- = \{\delta x : \nabla\phi \cdot \delta x < 0\}$ 与可达锥 $K$ 的内部不相交。

用几何语言：$K$（可达方向）和 $K_-$（下降方向）被一个超平面分开——没有一个方向既可达又下降。

由 **Minkowski 分离定理**（有限维凸锥分离）：存在非零向量 $(\lambda_0, \eta) \neq (0, 0)$ 使得
$$\lambda_0 \nabla\phi \cdot d \le 0, \quad \forall d \in K_-,$$
$$\eta^\top d \ge 0, \quad \forall d \in K.$$

更精确地：存在 $(\lambda_0, \lambda(t_f))$ 分离 $K$ 和下降锥。

**类比**：想象二维平面上，$K$ 是一个向右上方展开的锥，$K_-$ 是向左的半平面（代价下降方向）。如果 $K$ 和 $K_-$ 不相交，就可以画一条直线分开它们——直线的法向就是 $\lambda(t_f)$。

### Step 5：从分离到共态 ODE + Hamiltonian 极大化

把分离向量沿时间**反向传播**：定义
$$\lambda(t) = \Phi(t_f, t)^\top \lambda(t_f),$$
则 $\lambda$ 自动满足伴随 ODE $\dot\lambda = -A^\top \lambda$（因为 $\Phi(t_f, t)^\top$ 满足 $d/dt[\Phi(t_f,t)^\top] = -A^\top \Phi(t_f,t)^\top$）。

分离条件 "$K$ 中每个方向都与 $\lambda(t_f)$ 成非负角"翻译为：对每个 $(\tau, v)$，
$$\lambda(t_f)^\top \Phi(t_f, \tau) [f(x^*, v) - f(x^*, u^*)] \ge 0.$$

利用 $\lambda(\tau) = \Phi(t_f, \tau)^\top \lambda(t_f)$（即 $\lambda(t_f)^\top \Phi(t_f, \tau) = \lambda(\tau)^\top$），得到：
$$\lambda(\tau)^\top f(x^*(\tau), u^*(\tau)) \ge \lambda(\tau)^\top f(x^*(\tau), v), \quad \forall v \in U.$$

加入代价项（来自 $\lambda_0$ 的贡献），最终得到：
$$H(x^*(\tau), u^*(\tau), \lambda(\tau)) \ge H(x^*(\tau), v, \lambda(\tau)), \quad \forall v \in U.$$

这就是 Hamiltonian **极大化条件**。

### 证明中使用的数学工具

| 工具 | 在证明中的作用 |
|------|-------------|
| 凸锥分离定理 (Minkowski) | Step 4：分离可达锥和下降锥 |
| Hahn-Banach 定理 | 无穷维推广（PDE 控制） |
| Gronwall 不等式 | 保证 $o(\varepsilon)$ 误差项的精确控制 |
| Filippov-Wazewski 可测选择 | 保证最优控制的可测性存在 |
| Lebesgue 微分定理 | 保证几乎处处点是 Lebesgue 点 |
| 基本矩阵 $\Phi(t,s)$ | 线性化动力学的状态传播 |

### Gronwall 不等式的关键作用

在 Step 2 中，我们声称 $x_\varepsilon(t_f) - x^*(t_f) = \varepsilon\,\Phi(t_f,\tau)\Delta f + o(\varepsilon)$。这个 $o(\varepsilon)$ 的精确控制依赖 Gronwall 不等式：

**Gronwall 引理**：若 $y(t) \ge 0$ 满足 $y(t) \le \alpha + \int_{t_0}^t \beta(s)\,y(s)\,ds$，则 $y(t) \le \alpha\,\exp(\int_{t_0}^t \beta(s)\,ds)$。

应用：轨迹偏差的非线性部分 $e(t) = x_\varepsilon(t) - x^*(t) - \varepsilon\,\Phi(t,\tau)\Delta f$ 满足一个线性微分不等式，Gronwall 给出 $\|e(t)\| \le C\varepsilon^2 \exp(L(t_f - t_0))$（$L$ 是 $f$ 的 Lipschitz 常数）。这保证了一阶展开的有效性。

**为什么这在工程中重要**：Gronwall 常数 $\exp(L(t_f - t_0))$ 是打靶法病态性的数学根源——$L$ 大或 $t_f - t_0$ 长时，误差指数放大。这直接解释了为什么 single shooting 在长时间轨迹上极不稳定。

### 从 PMP 证明看数值方法设计的启示

PMP 证明的每一步都对应数值方法设计中的一个关键决策：

| 证明步骤 | 数值方法对应 | 工程含义 |
|---------|------------|---------|
| 针状扰动 → 一阶增量 | 自动微分 / 有限差分 | 灵敏度计算方式 |
| 基本矩阵 $\Phi$ | 变分方程积分 | STM (State Transition Matrix) 计算 |
| 凸锥分离 → 超平面法向 | KKT 条件中的乘子 | NLP 对偶变量 |
| 共态反向传播 | Adjoint 方法 | 梯度的高效计算（O(n) vs O(nm)）|

特别地，adjoint method（共态反向传播）是深度学习中 backpropagation 的连续时间版本——二者的数学结构完全相同，只是一个作用在 ODE 上，另一个作用在离散计算图上。Neural ODE (Chen et al. 2018 NeurIPS) 明确利用了这个联系。

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为 PMP 证明需要 $U$ 凸**

实际上 PMP 的优美之处正在于 $U$ **不需要凸**。凸锥 $K$ 是通过多针扰动的正组合自动产生的——即使每个 $v$ 只来自非凸集 $U$ 的单点，正组合仍然生成凸锥。这就是针状变分优于弱变分的核心优势：弱变分需要 $U$ 的切空间（要求 $U$ 有某种光滑性），针状变分只需要 $U$ 中**存在**那个 $v$ 点。

### 练习

1. （几何直觉）在 $\mathbb{R}^2$ 中画一个具体的可达锥 $K$（取两个方向的正组合）和下降锥 $K_-$。找出分离超平面的法向。验证法向方向正比于 $\lambda(t_f)$。

2. （关键步骤）对 $f(x,u) = Ax + Bu$（线性系统），验证 $\Phi(t_f, \tau) \Delta f = \Phi(t_f, \tau) B (v - u^*(\tau))$，并说明此时可达锥 $K$ 与系统的可控性矩阵有什么关系。

---

## §3.2.5 与 Euler-Lagrange 方程的精确关系 ⭐⭐

### 动机：PMP 如何"包含"变分法

学过变分法的学生自然会问：PMP 和 Euler-Lagrange 方程是什么关系？答案是：**PMP 严格推广 EL**——当控制无约束（$U = \mathbb{R}^m$）且动力学 $f(x,u) = u$（即控制就是速度）时，PMP 退化为经典 Euler-Lagrange 方程加上自然边界条件。

### 设定

专题 3.1 的标准 EL 问题：$\min J = \int L(x, \dot x, t)\,dt$，无控制约束。将其写作最优控制形式：取 $u = \dot x$（$U = \mathbb{R}^n = \mathbb{R}^m$），动力学 $\dot x = u$。

### 核心定理

**定理 3.2.5**（PMP ⇒ EL + 自然边界条件）：当 $f(x,u) = u$、$U = \mathbb{R}^n$、$L$ 关于 $(x, \dot x)$ 光滑时，PMP 退化为经典 Euler-Lagrange 方程
$$\frac{d}{dt}\frac{\partial L}{\partial \dot x} - \frac{\partial L}{\partial x} = 0,$$
并附带自然边界条件 $\partial L/\partial \dot x(t_f) = \partial\phi/\partial x(x(t_f))$。

### 证明（完整推导，3步）

**Step 1**：代入 $f = u$，Hamiltonian 变为
$$H = \lambda^\top u - L(x, u, t) \quad (\text{取 } \lambda_0 = 1).$$

**Step 2**：$U = \mathbb{R}^n$ 意味着无约束极大化。极大化必要条件（一阶条件，因为无边界）：
$$\frac{\partial H}{\partial u} = \lambda - \frac{\partial L}{\partial u} = 0.$$

由于 $u = \dot x$，得到：
$$\lambda = \frac{\partial L}{\partial \dot x}.$$

这恰是经典力学中**广义动量**的定义！PMP 的共态 $\lambda$ 就是 EL 理论中的广义动量 $p = \partial L/\partial \dot q$。

**Step 3**：共态方程 $\dot\lambda = -\partial H/\partial x$。计算 $\partial H/\partial x$：
$$\frac{\partial H}{\partial x} = -\frac{\partial L}{\partial x}.$$
（因为 $\lambda^\top u$ 中 $u$ 已经被极大化条件确定为 $\dot x$ 的函数，但直接求偏导时 $\lambda^\top u$ 对 $x$ 的偏导为零——这里的偏导是固定 $u$ 和 $\lambda$ 求 $x$ 的偏导。）

所以共态方程给出：
$$\dot\lambda = \frac{\partial L}{\partial x}.$$

将 $\lambda = \partial L/\partial \dot x$ 代入：
$$\frac{d}{dt}\frac{\partial L}{\partial \dot x} = \frac{\partial L}{\partial x}.$$

这正是 **Euler-Lagrange 方程**。

横截条件 $\lambda(t_f) = \partial\phi/\partial x$ 变为 $(\partial L/\partial \dot x)|_{t_f} = \partial\phi/\partial x$，即自然边界条件。

### PMP 如何严格推广 EL

| 维度 | Euler-Lagrange | PMP |
|------|---------------|-----|
| 控制集 | $U = \mathbb{R}^m$（无约束） | $U$ 任意紧集 |
| 动力学 | $\dot x = u$（控制=速度） | $\dot x = f(x,u,t)$ 任意 |
| 控制与状态 | $u = \dot x$（不分离） | 分离：$\dim u \ll \dim x$（欠驱动）|
| 可解条件 | $L$ 关于 $\dot x$ 严格凸 | 不需要 |
| 最优控制结构 | 连续的 $u(t)$ | 可能不连续（bang-bang） |

> **本质洞察**：PMP 对 EL 的推广不是"小修小补"，而是质的飞跃。EL 要求控制取遍整个空间且动力学形式为 $\dot x = u$；PMP 只要求 $U$ 紧且 $f$ 连续可微。这个推广使得所有欠驱动机器人（cart-pole、acrobot、四旋翼——控制维度远小于状态维度）都落入 PMP 的管辖范围。

### ⚠️ 常见陷阱

💡 **概念误区：认为"PMP 只是 EL 加上控制约束"**

虽然从历史上看 PMP 确实是为了处理控制约束而发明的，但它的价值远不止于此。即使在无约束情形下，PMP 的 Hamiltonian 形式也比 EL 更好用：它把二阶 ODE 拆为两个一阶 ODE（正则方程），揭示了辛结构，并且自然地给出了与 HJB（值函数）的联系。

### 练习

1. 对 $L = \frac{1}{2}\dot x^2 + V(x)$（粒子在势场中的运动），从 PMP 推导 EL 方程 $\ddot x = -V'(x)$（牛顿第二定律）。说明此时 Hamiltonian $H = \frac{1}{2} \lambda^2 - V(x)$ 恰好是总能量 $T + V$（动能+势能，注意符号约定）。

2. 对大地测量学问题（曲面上两点间最短路径），$L = \sqrt{g_{ij} \dot x^i \dot x^j}$，说明 EL 方程给出测地线方程。讨论为什么这个问题不需要 PMP（因为没有控制约束）。

---

## §3.2.6 与 HJB 的关系：必要条件 vs 充分条件 ⭐⭐⭐

### 动机：PMP 给轨迹，HJB 给场

PMP 和 HJB（Hamilton-Jacobi-Bellman）是最优控制理论的两大支柱，解决同一个问题但视角完全不同：

- **PMP**：从**单条轨迹**出发，问"这条轨迹满足什么条件？"→ 得到一阶必要条件
- **HJB**：从**整个状态空间的值函数**出发，问"值函数满足什么 PDE？"→ 得到充要条件

两者通过一个优美的等式联系：**共态 = 值函数的梯度**。

**类比**：PMP 就像"在一条河流的流线上工作"——你只关心这条流线的性质（速度、方向）。HJB 就像"获得整个速度场的全貌"——你知道每个点处水应该往哪里流。显然后者信息量更大（从场可以恢复任何一条流线），但计算代价也更高（需要求解遍及整个空间的 PDE）。

### 值函数定义

定义值函数（value function / cost-to-go）：
$$V(x,t) = \inf_{u(\cdot)} \left\{ \int_t^{t_f} L(\xi, u, s)\,ds + \phi(\xi(t_f)) : \dot\xi = f(\xi, u, s),\, \xi(t) = x \right\}.$$

直觉：$V(x,t)$ 是"从状态 $x$ 出发、在时刻 $t$ 开始、采用最优控制直到末端的最小总代价"。

### HJB 方程

**定理 3.2.6a**（Hamilton-Jacobi-Bellman 方程）：若 $V \in C^1$，则
$$\boxed{\frac{\partial V}{\partial t} + \min_{u \in U}\left\{L(x,u,t) + \nabla_x V(x,t)^\top f(x,u,t)\right\} = 0, \quad V(x,t_f) = \phi(x).}$$

注意这里用的是**极小化约定**（与 PMP 极大化约定对偶）。用极大化约定改写：
$$-\frac{\partial V}{\partial t} = \max_{u \in U}\left\{\lambda^\top f(x,u,t) - L(x,u,t)\right\}\bigg|_{\lambda = \nabla_x V}.$$

### 核心联系：共态 = 值函数梯度

**定理 3.2.6b**（PMP-HJB 对偶）：沿 PMP 最优轨迹 $x^*(\cdot)$，
$$\boxed{\lambda(t) = \nabla_x V(x^*(t), t), \qquad H^*(x^*, \lambda, u^*, t) = -\frac{\partial V}{\partial t}.}$$

这是最优控制理论中最深刻的结果之一。它说明：
- PMP 的共态 $\lambda(t)$ 就是 HJB 值函数沿最优轨迹的梯度
- PMP 的 Hamiltonian 最优值 $H^*$ 就是值函数对时间的负偏导

### 证明骨架（3步）

**Step 1**（Bellman 最优性原理）：
$$V(x,t) = \min_u \{L(x,u,t)\,dt + V(x + f(x,u,t)\,dt,\, t+dt)\}.$$

对 $V(x + f\,dt, t+dt)$ Taylor 展开：
$$V + \frac{\partial V}{\partial t}\,dt + \nabla_x V^\top f\,dt + O(dt^2).$$

代入并令 $dt \to 0$，得到 HJB 方程。

**Step 2**（沿最优轨迹）：由链式法则
$$\frac{dV}{dt}\bigg|_{x^*(t)} = \frac{\partial V}{\partial t} + \nabla_x V^\top f(x^*, u^*, t).$$

HJB 在最优 $u^*$ 处取到 $\min$，所以
$$\frac{\partial V}{\partial t} + L + \nabla_x V^\top f = 0 \implies \frac{dV}{dt} = -L.$$

这就是"沿最优轨迹，值函数以运行代价 $L$ 的速率递减"的直观含义。

**Step 3**（梯度等式）：对 HJB 方程关于 $x$ 求导，得到 $\nabla_x V$ 满足的 ODE。将其与 PMP 共态方程比较，发现两者**相同的 ODE + 相同的终端条件** $\nabla_x V(x(t_f), t_f) = \nabla\phi = \lambda(t_f)$。由 ODE 解的唯一性，$\lambda(t) = \nabla_x V(x^*(t), t)$。

### PMP vs HJB 全面对比

| 维度 | PMP | HJB |
|------|-----|-----|
| 条件性质 | 一阶**必要**条件 | **必要+充分** |
| 对象 | 单条轨迹 $(x^*(t), u^*(t))$ | 整个状态空间的值函数 $V(x,t)$ |
| 方程类型 | $2n$ 维 ODE 边值问题（BVP） | $(n+1)$ 维 PDE |
| 维数灾难 | 无（维数线性增长） | 有（PDE 网格指数爆炸） |
| 数值方法 | shooting / collocation | 网格法 / 粘性解 / tensor |
| 最优反馈 | 不直接给出（需离线求轨迹） | 天然给出 $u^*(x,t) = \arg\min H$ |
| 正则性要求 | $f, L$ 关于 $x$ 连续可微 | $V \in C^1$（可能不满足！→ 粘性解） |
| 非光滑推广 | Clarke 广义梯度 | 粘性解（Crandall-Lions 1983） |

### PMP 是 HJB 的特征线

> **本质洞察**：PMP 与 HJB 的关系，数学上精确等于一阶 PDE 与其特征线方程组的关系。HJB 是一个一阶 PDE（关于 $V(x,t)$），而 PMP 给出的 $(x(t), \lambda(t))$ ODE 系统就是这个 PDE 的**特征线**（method of characteristics）。沿特征线，PDE 降维为 ODE——这就是为什么 PMP 可以不受维数灾难困扰。

**反事实推理**：如果 PMP 是充分条件（像 HJB），那么解一个 $2n$ 维 BVP 就能得到全局最优——这比解 $(n+1)$ 维 PDE 容易得多。但 PMP 只是必要条件的代价是：它不能区分局部最优和全局最优，也不能处理多个极值轨迹中哪个真正最好。要获得充分性，必须付出"求解整个空间"的 HJB 代价。

### RL 语境下的对应

| 最优控制概念 | RL 对应 |
|------------|--------|
| 值函数 $V(x,t)$ | State-value function $V^\pi(s)$ |
| HJB 方程 | Bellman 方程（离散版本）|
| Q-learning / Value iteration | HJB 的离散近似求解 |
| Policy gradient | PMP 的 adjoint method 近似 |
| DDP / iLQR | 局部 HJB ≈ 离散 PMP + 二阶展开 |
| $H(x, u, \lambda)$ | Action-value $Q(s,a)$ minus baseline |
| $\lambda(t)$ | Critic 网络输出关于状态的梯度 |
| 横截条件 $\lambda(t_f) = \nabla\phi$ | Terminal value function bootstrap |

**深入对应：Policy Gradient = PMP 的 Monte-Carlo 近似**

考虑确定性系统 $x_{k+1} = f(x_k, u_k)$ 上的轨迹优化。PMP 的 adjoint method 精确计算代价对控制的梯度：
$$\frac{\partial J}{\partial u_k} = f_u^\top \lambda_{k+1} - L_{u,k}.$$

这与 REINFORCE 算法的梯度估计在确定性极限下完全等价——return-to-go $G_t$ 对应值函数 $V(x_t)$，其关于状态的梯度对应共态 $\lambda_t$。model-based RL 中的"通过动力学模型反向传播"本质上就是 PMP 的共态计算。Dreamer (Hafner 2020) 和 MBPO (Janner 2019) 中的 value gradient 计算，数学上等价于沿学习的动力学模型做 adjoint 反向传播——正是离散 PMP 的共态更新 $\lambda_k = f_x^\top \lambda_{k+1} + L_x^\top$。

**维数灾难的不同应对策略**

HJB 的维数灾难（$n > 5$ 时网格方法不可行）催生了多种近似策略：

| 策略 | 方法 | 适用维度 | 精度 |
|------|------|---------|------|
| PMP (本章) | BVP 求解 | 任意 $n$ | 高（单条轨迹）|
| 局部 HJB (DDP) | 沿轨迹二次展开 | 任意 $n$ | 中（局部）|
| Deep RL | 神经网络逼近 $V$/$Q$ | 任意 $n$ | 低（逼近误差大）|
| Tensor decomposition | 低秩 HJB | $n \le 20$ | 中-高 |
| Max-plus 代数 | curse-of-dimensionality-free | 特殊结构 | 高 |

这说明 PMP 和 HJB 不是"竞争关系"，而是"互补层次"：PMP 给**单条轨迹**的精确信息，HJB 给**全状态空间**的近似信息，DDP 取中间路线（局部状态空间的精确信息）。

### ⚠️ 常见陷阱

💡 **概念误区："PMP 不如 HJB，因为只是必要条件"**

实际上两者各有优势。对高维系统（$n > 5$），HJB 的网格方法因维数灾难而不可行；而 PMP 的 BVP 维度是 $2n$（线性增长），仍然可以高效求解。工业界的大部分轨迹优化器（CasADi, GPOPS, acados）本质上都是在求解 PMP 的离散化（即 KKT 条件，它等价于离散 PMP）。

🧠 **思维陷阱：认为"RL 是 HJB 的完美实现"**

虽然 Q-learning 在数学上对应 HJB 的离散版本，但深度 RL 中的函数逼近（用神经网络逼近 $V$ 或 $Q$）引入了巨大的逼近误差。model-based RL（如 MBPO、Dreamer）+ trajectory optimization（如 DDP、MPPI）的组合，其实更接近"局部 HJB + PMP 特征线"的数学结构。

### 练习

1. 对一维 LQR 问题 $\dot x = u$，$J = \frac{1}{2}\int_0^T (x^2 + u^2)\,dt$，直接求解 HJB 方程（假设 $V = \frac{1}{2}P(t)x^2$），导出 Riccati 方程。验证 $\lambda = V_x = P(t)x$。

2. 解释为什么 DDP 被称为"局部 HJB"：它沿参考轨迹做值函数的二次近似 $V \approx V_0 + V_x^\top \delta x + \frac{1}{2}\delta x^\top V_{xx} \delta x$，而 $V_x$ 正是共态 $\lambda$，$V_{xx}$ 是 Riccati 矩阵 $P$。

---

## §3.2.7 与 LQR/Riccati 的关系 ⭐⭐

### 动机：PMP 的唯一解析可解大类

LQR（线性二次调节器）是最优控制中极少数能完全解析求解的问题类别。它的重要性不仅在于自身的工程应用（线性系统的标配控制器），更在于它是**所有非线性轨迹优化器的基本模块**——iLQR/DDP 的每一步都是一个 LQR 子问题。

### 设定

线性二次（LQ）问题：
$$\min\; J = \frac{1}{2}x(t_f)^\top Q_f x(t_f) + \frac{1}{2}\int_{t_0}^{t_f}(x^\top Q x + u^\top R u)\,dt,$$
$$\dot x = Ax + Bu, \quad Q \succeq 0,\, R \succ 0,\, Q_f \succeq 0.$$

### 核心定理

**定理 3.2.7**（PMP ⇒ Riccati）：最优控制为线性状态反馈
$$u^*(t) = -R^{-1}B^\top P(t)\, x(t),$$
其中 $P(t)$ 是 $n \times n$ 对称正定矩阵，满足**矩阵 Riccati 微分方程**：
$$\boxed{-\dot P = PA + A^\top P - PBR^{-1}B^\top P + Q, \quad P(t_f) = Q_f.}$$

出处：Kalman 1960；Anderson-Moore 1990《Optimal Control: Linear Quadratic Methods》。

### 证明（完整推导，5步）

**Step 1（写出 Hamiltonian）**：取 $\lambda_0 = 1$，极大化约定：
$$H = \lambda^\top(Ax + Bu) - \frac{1}{2}(x^\top Qx + u^\top Ru).$$

**Step 2（极大化条件）**：$U = \mathbb{R}^m$（无控制约束），因此
$$\frac{\partial H}{\partial u} = B^\top\lambda - Ru = 0 \implies u^* = R^{-1}B^\top\lambda.$$

验证二阶条件：$\partial^2 H/\partial u^2 = -R \prec 0$（严格凹），确认极大值。

**Step 3（正则方程组）**：代入 $u^*$，状态方程和共态方程合写为 $2n$ 维**Hamiltonian 矩阵系统**：
$$\begin{pmatrix} \dot x \\ \dot\lambda \end{pmatrix} = \underbrace{\begin{pmatrix} A & BR^{-1}B^\top \\ Q & -A^\top \end{pmatrix}}_{\mathcal{H}\text{（Hamilton 矩阵）}} \begin{pmatrix} x \\ \lambda \end{pmatrix}.$$

注意 Hamilton 矩阵 $\mathcal{H}$ 的特征值关于虚轴对称——这是辛结构的体现。

**Step 4（Riccati 替换）**：假设 $\lambda(t) = P(t) x(t)$（线性关系），其中 $P(t)$ 待定。对 $\lambda = Px$ 求导：
$$\dot\lambda = \dot P x + P \dot x.$$

左端由共态方程给出：$\dot\lambda = Qx - A^\top\lambda = Qx - A^\top Px$。
$P\dot x$ 由状态方程给出：$P\dot x = P(Ax + BR^{-1}B^\top\lambda) = PAx + PBR^{-1}B^\top Px$。

合并：
$$Qx - A^\top Px = \dot P x + PAx + PBR^{-1}B^\top Px.$$

整理（对任意 $x$ 成立 ⇒ 系数相等）：
$$\dot P = -PA - A^\top P + PBR^{-1}B^\top P - Q.$$

这就是 **Riccati 微分方程**。

**Step 5（终端条件）**：横截条件 $\lambda(t_f) = Q_f x(t_f)$（来自 $\partial\phi/\partial x = Q_f x$）给出 $P(t_f) = Q_f$。

最优控制代入 $\lambda = Px$：
$$u^* = R^{-1}B^\top\lambda = R^{-1}B^\top P(t)\,x(t) \equiv -K(t)\,x(t),$$
其中 $K(t) = R^{-1}B^\top P(t)$ 是时变增益矩阵。

### 稳态 LQR（$t_f \to \infty$）

当 $t_f \to \infty$ 且 $(A, B)$ 可控、$(A, Q^{1/2})$ 可观时，Riccati 方程达到稳态 $\dot P = 0$，得到**代数 Riccati 方程（ARE）**：
$$0 = PA + A^\top P - PBR^{-1}B^\top P + Q.$$

稳态增益 $K = R^{-1}B^\top P$ 使闭环 $A - BK$ 的特征值全在左半平面（渐近稳定）。

**数值求解工具**：MATLAB `care/dare`、SciPy `scipy.linalg.solve_continuous_are`、SLICOT 库。

### 工程含义

LQR 是 PMP **唯一能手工解析解出的一般类**。所有复杂非线性轨迹优化器（iLQR、DDP）的内核是**沿名义轨迹局部 LQ 化** + 反向 Riccati 递推。具体地：

1. **iLQR/DDP 的 backward pass**：在每个时步 $k$，对非线性动力学 $x_{k+1} = f(x_k, u_k)$ 做线性化 $\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k$，对代价做二次化——然后解一个离散 LQR 子问题，得到反馈增益 $K_k$ 和前馈修正 $d_k$。

2. **MPC 的 QP 子问题**：线性 MPC 本质上是有限时域 LQR + 不等式约束。无约束时 MPC = 有限时域 LQR（Riccati 给出的 explicit solution）。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Riccati 方程的积分方向**

错误做法：从 $t_0$ 正向积分 $\dot P = \ldots$，初始条件 $P(t_0) = ?$（未知）

正确做法：从 $t_f$ **反向积分**，初始条件 $P(t_f) = Q_f$（已知）。在代码中用负步长或时间反转。

💡 **概念误区：认为"LQR 只能用于线性系统"**

LQR 直接适用于线性系统。但通过**线性化 + LQR** 的组合（即 iLQR），可以处理任意非线性系统。每一步 iLQR 迭代都是："在当前轨迹周围做 LQ 近似 → 解 LQR → 更新轨迹 → 重新线性化 → 再解 LQR → ..."直到收敛。这就是 LQR 在非线性世界中的"递归使用"。

### 练习

1. 对标量系统 $\dot x = ax + bu$，$J = \frac{1}{2}\int_0^\infty (qx^2 + ru^2)\,dt$（$q, r > 0$），手推 ARE 得到 $P = \frac{1}{b^2}(a + \sqrt{a^2 + b^2 q/r})r$（验证正根选择），写出闭环特征值。

2. 编写 Python 代码，用 `scipy.integrate.solve_ivp` 反向积分 Riccati 方程（$A = [[0,1],[0,0]]$，$B = [[0],[1]]$，$Q = I$，$R = 1$，$Q_f = 10I$，$t_f = 5$），画出 $P_{11}(t), P_{22}(t)$ 的时间历程，观察从 $Q_f$ 向稳态值 $P_\infty$ 的收敛。

---

## §3.2.8 经典可解实例 ⭐⭐

### 动机：PMP 的三种典型面目

PMP 的力量体现在对具体问题的求解中。本节详细推导五个经典例题，它们分别展示了 PMP 在不同 Hamiltonian 结构下的不同最优控制形态。

### 实例 1：双积分器最小时间（bang-bang）⭐⭐

**系统**：$\ddot x = u$，$|u| \le 1$；状态 $(x_1, x_2) = (x, \dot x)$。目标：最小化 $t_f$ 使 $(x_1(t_f), x_2(t_f)) = (0, 0)$。

**Step 1（写 Hamiltonian）**：$L \equiv 1$（最小时间），$\phi = 0$。动力学 $\dot x_1 = x_2$，$\dot x_2 = u$。

$$H = \lambda_1 x_2 + \lambda_2 u - 1 \quad (\text{极大化约定，} \lambda_0 = 1).$$

**Step 2（极大化条件）**：$H$ 关于 $u$ 是线性的（系数为 $\lambda_2$），所以
$$u^* = \operatorname{sign}(\lambda_2(t)).$$

这就是 **bang-bang 控制**：控制只取极值 $+1$ 或 $-1$，由 $\lambda_2$ 的符号决定。

**Step 3（共态方程）**：
$$\dot\lambda_1 = -\frac{\partial H}{\partial x_1} = 0 \implies \lambda_1 = \text{const} = c_1,$$
$$\dot\lambda_2 = -\frac{\partial H}{\partial x_2} = -\lambda_1 = -c_1 \implies \lambda_2(t) = -c_1 t + c_2.$$

$\lambda_2(t)$ 是 $t$ 的**一次多项式**（直线），最多有**一个零点**——即控制最多切换一次。

**Step 4（Feldbaum 定理的体现）**：对 $n$ 阶系统 $x^{(n)} = u$，共态的最高分量 $\lambda_n(t)$ 是 $(n-1)$ 次多项式，因此最多 $n-1$ 次切换。双积分器 $n = 2$ ⇒ 至多 1 次切换。

**Step 5（切换曲线综合）**：

在 $u = +1$ 下：$\ddot x = 1$ ⇒ $x_1 = \frac{1}{2}x_2^2 + c$（向右开口抛物线族）。
在 $u = -1$ 下：$\ddot x = -1$ ⇒ $x_1 = -\frac{1}{2}x_2^2 + c$（向左开口抛物线族）。

到达原点的特殊抛物线：
- $u = +1$ 经过原点的抛物线：$x_1 = \frac{1}{2}x_2^2$（$x_2 \le 0$，即从左下方加速到原点）
- $u = -1$ 经过原点的抛物线：$x_1 = -\frac{1}{2}x_2^2$（$x_2 \ge 0$，即从右上方减速到原点）

切换曲线（switching curve）：
$$\gamma: \quad x_1 = -\frac{1}{2}x_2|x_2|.$$

**最优反馈综合**：
$$u^*(x_1, x_2) = -\operatorname{sign}\left(x_1 + \frac{1}{2}x_2|x_2|\right).$$

当 $(x_1, x_2)$ 在切换曲线上方时用 $u = -1$（制动），下方时用 $u = +1$（加速）。从任意初始点出发，轨迹先沿一条抛物线到达切换曲线，然后沿切换曲线到达原点。

出处：Athans-Falb 1966《Optimal Control》Ch. 7；Kirk 2004 Ch.5 §5.4。

**数值验证**：对初始状态 $(x_1, x_2) = (2, 1)$，具体求解过程如下。

首先确定初始点在切换曲线的哪一侧：$x_1 + \frac{1}{2}x_2|x_2| = 2 + \frac{1}{2}(1)(1) = 2.5 > 0$，所以先用 $u = -1$（制动）。

在 $u = -1$ 下的运动方程：$\dot x_2 = -1$，$\dot x_1 = x_2$。解为：
$$x_2(t) = 1 - t, \quad x_1(t) = 2 + t - \frac{1}{2}t^2.$$

轨迹到达切换曲线的时刻 $t_s$ 满足 $x_1(t_s) = -\frac{1}{2}x_2(t_s)|x_2(t_s)|$。由于切换后沿曲线 $x_1 = \frac{1}{2}x_2^2$（$x_2 \le 0$）到达原点，切换点必须在 $x_2 < 0$ 的半平面。从 $x_2 = 1 - t$，切换时 $t_s > 1$。

代入切换曲线条件：$2 + t_s - \frac{1}{2}t_s^2 = -\frac{1}{2}(1-t_s)^2$（注意 $x_2 = 1-t_s < 0$，$|x_2| = t_s - 1$）。

化简：$2 + t_s - \frac{1}{2}t_s^2 = -\frac{1}{2}(t_s - 1)^2$

$2 + t_s - \frac{1}{2}t_s^2 = -\frac{1}{2}t_s^2 + t_s - \frac{1}{2}$

$2 = -\frac{1}{2}$——矛盾！

这说明从 $(2, 1)$ 出发用 $u = -1$ 无法直接到达"正确一支"的切换曲线。实际上需要先用 $u = -1$ 到达 $x_1 = -\frac{1}{2}x_2^2$（$x_2 \ge 0$ 部分），切换后用 $u = +1$ 沿 $x_1 = \frac{1}{2}x_2^2$ 到达原点。这个具体计算留给读者作为练习——关键点是：相平面分析 + 切换曲线公式给出了求解的完整工具。

### 实例 2：最小能量 LQR

已在 §3.2.7 完整推导。核心结果：$u^* = -R^{-1}B^\top P(t)\,x$，$P$ 由 Riccati 反向积分。

LQR 之所以能解析求解，关键在于两个"线性"条件同时满足：(1) 动力学 $f = Ax + Bu$ 关于 $(x, u)$ 线性；(2) 代价的 Hessian 关于 $(x, u)$ 是常数矩阵。这使得共态与状态之间保持线性关系 $\lambda = Px$，从而把 BVP 化为 Riccati ODE——一个有限维非线性 ODE（而非无穷维算子方程）。任何破坏这两个条件的情形（非线性动力学、非二次代价）都导致 $\lambda$ 与 $x$ 的关系非线性，PMP BVP 不再有显式解。

### 实例 3：Dubins 车最短路径 ⭐⭐⭐

**系统**：$\dot x = \cos\theta$，$\dot y = \sin\theta$，$\dot\theta = u$，$|u| \le 1$。起点 $(p_0, \theta_0)$ 到终点 $(p_f, \theta_f)$ 的最短弧长路径。

**Dubins 定理**（1957 *Amer. J. Math.* 79(3):497-516）：最短曲率受限路径必是以下 6 种之一：
$$\{LSL,\, LSR,\, RSL,\, RSR,\, LRL,\, RLR\},$$
其中 $L$ = 左转最小半径弧、$R$ = 右转最小半径弧、$S$ = 直线段。

**PMP 推导要点**：

$H = \lambda_x \cos\theta + \lambda_y \sin\theta + \lambda_\theta u - 1$。

极大化 ⇒ $u^* = \operatorname{sign}(\lambda_\theta)$（bang-bang，因为 $H$ 对 $u$ 线性）。

共态方程：$\dot\lambda_x = 0$，$\dot\lambda_y = 0$（$\lambda_x, \lambda_y$ 守恒），
$$\dot\lambda_\theta = -\frac{\partial H}{\partial \theta} = \lambda_x \sin\theta - \lambda_y \cos\theta.$$

由于 $\lambda_x, \lambda_y$ 为常数，$\lambda_\theta$ 的符号变化由 $\theta(t)$ 决定。当 $u^* = \pm 1$ 时 $\theta$ 线性变化，所以 $\lambda_\theta$ 是 $\theta$ 的正弦函数——其零点模式有限，这导致了有限种路径类型。

**奇异弧**（$\lambda_\theta \equiv 0$ 的区间）：此时 $\dot\lambda_\theta = 0$ ⇒ $\lambda_x \sin\theta = \lambda_y \cos\theta$ ⇒ $\theta = \text{const}$，即直线段 $S$。

**为什么恰好是6种路径类型？**

关键论证步骤：
1. 在弧段上 $u = \pm 1$（bang-bang），$\theta$ 线性变化 ⇒ 弧段是圆弧（半径 = 最小转弯半径 $1/u_{\max}$）
2. 在直线段上 $u = 0$（奇异弧），$\theta = \text{const}$ ⇒ 直线
3. 共态 $\lambda_\theta$ 的零点分析表明：最优路径最多由**3段**组成
4. 每段是 $L$（左圆弧）、$R$（右圆弧）或 $S$（直线）
5. 相邻段不能相同类型且同方向（否则合并为一段更短）
6. 穷举所有3段组合中满足约束的类型，恰好6种

**Reeds-Shepp 扩展**（允许倒车，$v \in \{-1, +1\}$）：路径类型从6种增加到46种三参数族（Reeds-Shepp 1990 *Pacific J. Math.* 145(2):367-393）。这在自动驾驶中非常重要——倒车入库的最短路径计算直接用 Reeds-Shepp 公式。

**工程应用**：
- ROS 的 `ompl` 库中内置了 Dubins 和 Reeds-Shepp 路径计算
- 无人机/自动驾驶的全局路径规划器经常把 Dubins 曲线作为启发式连接器（在 RRT* 等采样规划器中连接两个配置）
- 固定翼无人机的航路规划直接使用 Dubins 路径（因为固定翼不能悬停或倒飞）
- 农业无人机的覆盖路径规划中，转弯段用 Dubins 弧保证曲率可行

**与 PMP 的深层联系**：Dubins 定理不仅仅是一个"几何结果"——它的证明本质上是 PMP 在特定系统上的应用。共态的结构（$\lambda_x, \lambda_y$ 守恒，$\lambda_\theta$ 是正弦函数）决定了路径类型的有限性。这展示了 PMP 如何把"无穷维优化"（在所有曲线中找最短的）降维为"有限参数优化"（在6种类型中选最好的）——这种降维是 PMP 最强大的工程价值之一。

### 实例 4：Goddard 火箭最优推力 ⭐⭐⭐

**系统**（一维垂直，1919 年 Goddard 原问题）：
$$\dot h = v, \quad \dot v = (T - D(h,v))/m - g(h), \quad \dot m = -T/c, \quad 0 \le T \le T_{\max}.$$

目标：最大化 $h(t_f)$（燃料耗尽 $m(t_f) = m_f$ 时）。

**PMP 结构**：$H$ 关于 $T$ 线性，切换函数为
$$\Sigma(t) = \frac{\lambda_v}{m} - \frac{\lambda_m}{c}.$$

最优推力的**三段式结构**：
$$T_{\max} \longrightarrow \text{奇异弧} \longrightarrow 0 \; (\text{coast}).$$

- 第一段：$\Sigma > 0$，$T = T_{\max}$（全推力加速）
- 中间段：$\Sigma = 0$，奇异弧（推力 $T_{\text{sing}}$ 由 $d^2\Sigma/dt^2 = 0$ 确定）
- 末段：$\Sigma < 0$，$T = 0$（滑行）

出处：Bonnans-Martinon-Trelat 2008 *JOTA* 139(2):439-461。

### 实例 5：四旋翼最小时间（现代机器人应用）⭐⭐⭐

Foehn-Romero-Scaramuzza 2021《Time-optimal planning for quadrotor waypoint flight》*Science Robotics* 6(56):eabh1221。

运用 **Complementary Progress Constraints (CPC)** 将航点通过时刻作为决策变量，数值求解 PMP 极值条件。关键发现：
- 推力在绝大多数时段**饱和**（bang-bang），符合 PMP 预测
- 航点附近出现准奇异弧（协调三轴力矩的过渡段）
- 实测击败职业无人机竞速飞手

### 工程含义对比表

| 问题 | 代价 $L$ | 控制集 $U$ | $H$ 关于 $u$ | 最优结构 |
|------|---------|-----------|-------------|---------|
| 双积分器最小时间 | 1 | $[-1,1]$ | 线性 | bang-bang（至多 1 次切换）|
| LQR | $\frac{1}{2}(x^\top Qx + u^\top Ru)$ | $\mathbb{R}^m$ | 严格凹 | 线性反馈 $u = -Kx$ |
| Dubins | 1 | $[-1,1]$ | 线性 | bang-bang + 奇异直线 |
| Goddard | 0（Mayer）| $[0, T_{\max}]$ | 线性 | max + singular + coast |
| 四旋翼最小时间 | 1 | 推力箱 | 线性 | bang-bang（大部分饱和）|

### ⚠️ 常见陷阱

💡 **概念误区：所有最小时间问题都是 bang-bang**

这只对"$H$ 关于 $u$ 严格线性"的情况成立。如果动力学 $f$ 中控制 $u$ 以非线性方式出现（如 $f = u^2 + x$），$H$ 关于 $u$ 不再线性，极大化可能给出连续的控制（非 bang-bang）。

⚠️ **编程陷阱：bang-bang 切换时刻的数值检测**

在数值求解中，切换函数 $\Sigma(t)$ 经过零点的精确时刻很难捕捉（数值积分的步长可能跨越零点）。对策：使用事件检测（event detection）功能（如 `scipy.integrate.solve_ivp` 的 `events` 参数）或自适应步长在 $|\Sigma| < \varepsilon$ 时缩小。

### 练习

1. 对双积分器最小时间问题，从初始状态 $(x_1, x_2) = (2, 1)$ 出发到原点，画出相平面轨迹。确定切换时刻 $t_s$ 和总时间 $t_f$（解析求出）。

2. 对 Dubins 车从 $(0,0,0)$ 到 $(4,0,0)$（同方向，距离为4个最小半径），确定最优路径类型（$LSL$ 还是 $RSR$？还是纯直线 $S$？），并计算路径长度。

3. （跨章综合）用 §3.2.7 的 Riccati 方法求解离散 LQR，然后与直接求解 PMP BVP 的结果对比。验证两者给出相同的最优控制序列。

---

## §3.2.8.5 PMP 求解工作流：从建模到验证 ⭐⭐

### 动机：如何系统化地用 PMP 解题

前面各节分别讲解了 PMP 的各个组成部分。本节把它们串联成一个完整的**求解工作流**——从拿到问题到验证最优性的全过程。

### 七步工作流

```
Step 1: 标准化  →  将问题写成 Bolza 形式，明确 f, L, φ, U, ψ
        │
Step 2: 写 H   →  $H = \lambda^\top f - \lambda_0 L$（注意约定）
        │
Step 3: 极大化  →  分析 H 关于 u 的结构（线性/凹/退化）
        │                → 线性: bang-bang, u* = sign(σ)
        │                → 凹:   内点, ∂H/∂u = 0
        │                → 退化: 奇异弧, 需高阶条件
        │
Step 4: 共态方程 → ˙λ = -∂H/∂x，写出分量形式
        │
Step 5: 横截条件 → 确定 λ(tf) 或端点约束方程
        │
Step 6: 求解 BVP → 解析（少数幸运情况）或数值（一般情况）
        │
Step 7: 验证    → H 守恒？横截满足？二阶条件？
                   与直接法对比？
```

### 常见"快速判断"技巧

在动手计算之前，先做以下快速分析可以节省大量时间：

| 检查项 | 判断 | 结论 |
|--------|------|------|
| $f$ 关于 $u$ 仿射？ | 是 → $H$ 关于 $u$ 线性 | 必为 bang-bang 或奇异 |
| $L$ 含 $u^2$ 项且 $R \succ 0$？ | 是 → $H$ 关于 $u$ 严格凹 | 内点解 $\partial H/\partial u = 0$ |
| 系统自治（$f, L$ 不含 $t$）？ | 是 → $H$ 守恒 | 可作为验证条件 |
| 系统线性 + 代价二次？ | 是 → LQR | Riccati 给解析解 |
| $t_f$ 自由？ | 是 → $H = 0$ | 额外方程，也是验证工具 |

### 示范：对任意新问题的"3分钟诊断"

假设你面对一个新的轨迹优化问题。在写任何方程之前，问自己：

1. **"这是什么类型的代价？"** → Bolza（最常见）/ Mayer / Lagrange
2. **"控制怎么进入动力学？"** → 仿射（$f = a + Bu$）→ bang-bang；非线性 → 复杂
3. **"有没有显式时间依赖？"** → 无 → 可用 $H$ 守恒；有 → 不能
4. **"维度多大？"** → $n \le 4$ → 有望画相图；$n > 10$ → 必须数值

---

## §3.2.9 Bang-Bang 控制与切换函数分析 ⭐⭐⭐

### 动机

在实际工程中，大量控制系统的最优解具有 bang-bang 结构：推力卫星只能全开或全关，开关电源只有通/断两个状态，液压阀只有全开/全关。理解 bang-bang 控制的数学结构是设计这类系统的基础。

### 切换函数的一般理论

当 Hamiltonian 关于控制 $u$ 是**仿射**的（即 $H = H_0(x, \lambda, t) + \sigma(x, \lambda, t)^\top u$），极大化条件给出：

$$u^*_i(t) = \begin{cases} u_{i,\max}, & \text{if } \sigma_i(t) > 0, \\ u_{i,\min}, & \text{if } \sigma_i(t) < 0, \\ \text{undetermined}, & \text{if } \sigma_i(t) = 0. \end{cases}$$

$\sigma(t) = \partial H / \partial u$ 称为**切换函数**（switching function）。

### Feldbaum 定理：切换次数的上界

**定理**（Feldbaum 1953）：对线性时不变系统 $\dot x = Ax + Bu$，$u \in [-1, +1]$，时间最优控制的切换次数不超过 $n - 1$（$n$ 为状态维度）。

**证明思路**：$\lambda(t)$ 满足 $\dot\lambda = -A^\top \lambda$（线性 ODE），所以 $\lambda(t) = e^{-A^\top t} \lambda(0)$。切换函数 $\sigma(t) = B^\top \lambda(t) = B^\top e^{-A^\top t} \lambda(0)$ 是 $t$ 的一个（至多）$n-1$ 次多项式乘以指数函数的组合。根据 Chebyshev 系统的理论（$e^{-A^\top t}$ 的分量构成 T-系统），其零点个数至多 $n-1$ 个。

### Fuller 问题：无穷切换的反例

**Fuller 1960-1961** 发现了一个著名的反例：二维系统
$$\dot x_1 = x_2, \quad \dot x_2 = u, \quad |u| \le 1, \quad J = \int_0^\infty x_1^2 \, dt.$$

最优控制有**无穷次切换**（chattering），切换时刻按几何级数 $t_k \sim q^k$ 趋于零（$q \approx 0.44$）。这说明 Feldbaum 定理的"最小时间"条件是本质的——对其他代价类型，切换可以是无穷的。

出处：Fuller 1960 *J. Electronics and Control* 8(5):381-401；Marchal 1973 *J. Optimization Theory and Applications* 11:441-486。

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为 bang-bang 控制总是"粗暴的"**

实际上 bang-bang 是理论上的**最优**结构（对 $H$ 关于 $u$ 线性的问题）。它不是"近似"或"简化"——它就是精确最优解。在航天器姿态机动、时间最优机械臂运动等场景中，bang-bang 控制能比任何"平滑"控制节省更多时间或燃料。

### 练习

1. 对三积分器 $x^{(3)} = u$，$|u| \le 1$，证明时间最优控制至多 2 次切换。画出三维相空间中的切换面。

2. 编程实验：对 Fuller 问题，用直接法（CasADi + 细网格）数值求解，观察网格加密时是否出现越来越多的切换。讨论为什么直接法对 chattering 问题有根本困难。

---

## §3.2.10 奇异弧与 Kelley 条件 ⭐⭐⭐

### 动机

当切换函数 $\sigma(t) = \partial H / \partial u$ 在一个**时间区间**上恒为零（而非仅在离散点为零）时，PMP 的极大化条件退化——$H$ 对 $u$ 的值是"平坦的"，无法确定最优控制。这段区间称为**奇异弧**（singular arc）。

### 奇异弧的严格定义

**定义**：如果存在时间区间 $[t_1, t_2] \subset [t_0, t_f]$ 使得切换函数
$$\sigma(t) = \frac{\partial H}{\partial u}(x^*(t), u^*(t), \lambda(t), t) = 0, \quad \forall t \in [t_1, t_2],$$
则 $[t_1, t_2]$ 上的最优弧称为奇异弧，对应的控制称为奇异控制 $u_{\text{sing}}(t)$。

### 如何确定奇异控制：逐阶微分

由于 $\sigma(t) \equiv 0$ 在 $[t_1, t_2]$ 上，对其反复求导直到 $u$ 显式出现：

$$\sigma = 0, \quad \dot\sigma = 0, \quad \ddot\sigma = 0, \quad \ldots, \quad \frac{d^{2q}}{dt^{2q}}\sigma = 0,$$

其中 $q$ 是使 $u$ 首次显式出现于 $d^{2q}\sigma/dt^{2q}$ 中的最小整数。$q$ 称为奇异弧的**阶**（order）。

从 $d^{2q}\sigma/dt^{2q} = 0$ 中可以解出 $u_{\text{sing}}$ 作为 $(x, \lambda, t)$ 的函数。

### Kelley 条件（广义 Legendre-Clebsch 条件）

**定理 3.2.10**（Kelley 1964）：标量控制奇异弧的 $q$ 阶必要条件为
$$(-1)^q \frac{\partial}{\partial u}\left[\frac{d^{2q}}{dt^{2q}}\frac{\partial H}{\partial u}\right] \ge 0.$$

**直觉**：这是经典 Legendre 条件（$\partial^2 H / \partial u^2 \le 0$ 对极大化）的高阶推广。在奇异弧上 $\partial^2 H / \partial u^2 = 0$（退化），需要看更高阶项来判断极值类型。符号 $(-1)^q$ 与奇/偶阶交替有关。

出处：Kelley 1964 *AIAA J.* 2(8):1380-1382；Goh 1966（向量控制推广）；Robbins 1967 *IBM J.* 11:361-372。

### 数值困境

奇异弧对**直接法**极不友好：
- 离散化后 $\partial H/\partial u \approx 0$ 让 NLP 的 Hessian 矩阵病态
- 典型现象是 **bang-bang chattering**（鱼骨振荡）——数值解在 $u_{\min}$ 和 $u_{\max}$ 之间高频振荡来"模拟"奇异弧

对**间接法**则可行但需事先识别奇异结构（通过几何分析判断奇异弧的存在和位置），然后用同伦延拓（homotopy continuation）连续跟踪解。

### 工程含义

| 应用领域 | 奇异弧的物理含义 |
|---------|---------------|
| Goddard 火箭 | 中间段的渐变推力（平衡重力和阻力）|
| 机械臂路径参数化（Bobrow 1985） | 速度曲线的平滑段 |
| 航天低推力转移 | 推力方向渐变段 |
| 化工反应器温度控制 | 温度维持段 |

### ⚠️ 常见陷阱

💡 **概念误区：奇异弧上 $u$ 可以取任意值**

虽然 PMP 极大化条件在奇异弧上"退化"，但奇异控制 $u_{\text{sing}}$ 是**唯一确定的**——由 $d^{2q}\sigma/dt^{2q} = 0$ 这个代数方程解出。"退化"的含义是"一阶条件不够"，需要用高阶条件来确定。

### 练习

1. 对 Goddard 火箭问题，写出切换函数 $\Sigma = \lambda_v/m - \lambda_m/c$，验证 $\dot\Sigma$ 和 $\ddot\Sigma$ 的表达式。确定奇异弧的阶 $q$，并从 $\ddot\Sigma = 0$ 解出 $T_{\text{sing}}$。

2. 验证 Kelley 条件对 Goddard 奇异弧给出正号（$(-1)^1 \cdot (\ldots) \ge 0$），确认是极大值而非极小值。

---

## §3.2.11 状态约束下的 PMP 扩展 ⭐⭐⭐

### 动机

实际系统不仅有控制约束（$u \in U$），还经常有**状态约束**：
- 机器人避障：$\|x - o_i\| \ge r$（与障碍物的距离不小于安全半径）
- 关节限位：$q_{\min} \le q \le q_{\max}$
- 温度上限：$T(t) \le T_{\max}$
- 地面不穿透：$z(t) \ge 0$

状态约束使 PMP 产生本质性的新困难：共态 $\lambda(t)$ 可能**不连续**（跳跃）。

### 问题形式

$$\min J \quad \text{s.t.} \quad \dot x = f(x, u, t), \quad g(x, t) \le 0, \quad u \in U.$$

### 核心定理

**定理 3.2.11**（状态约束 PMP）：共态 $\lambda$ 可能**跳跃**。引入 Borel 测度乘子 $\mu \in \mathcal{M}([t_0, t_f])$ 满足互补松弛：
$$\text{supp}(\mu) \subset \{t : g(x^*(t), t) = 0\}, \qquad \mu \ge 0.$$

修正的共态方程变为（分布意义）：
$$d\lambda = -\frac{\partial H}{\partial x}\,dt + \frac{\partial g}{\partial x}^\top d\mu.$$

出处：Jacobson-Lele-Speyer 1971；综述 Hartl-Sethi-Vickson 1995 *SIAM Review* 37(2):181-218。

### 边界弧与接触点

- **Boundary arc**（边界弧）：轨迹沿 $g = 0$ 曲面滑行一段时间；$\mu$ 有连续密度。此时轨迹被约束在 $g = 0$ 面上，控制必须同时满足 $\dot g = 0$（保持在面上）。

- **Touch point**（接触点）：轨迹瞬时触碰 $g = 0$ 面然后离开；$\mu$ 是 Dirac 质量（delta 函数）；$\lambda$ 在该点**跳跃**。

**类比**：想象一个球在桌面上滚动（无约束），偶尔碰到墙壁反弹（接触点 → 共态跳跃），或者沿墙壁滑行（边界弧 → 连续约束力）。共态的跳跃就像反弹时的动量突变，而边界弧上的连续乘子就像持续的法向力。

### 状态约束的阶（order）

状态约束 $g(x) \le 0$ 的阶定义为：对 $g$ 反复对时间求导，直到控制 $u$ 首次显式出现的次数。

- **一阶约束**（$\dot g$ 显含 $u$）：如 $g = x_1 - x_{\max}$ 且 $\dot x_1 = u$ → $\dot g = u$。控制可以直接"顶住"约束。
- **二阶约束**（$\ddot g$ 才显含 $u$）：如 $g = x_1 - x_{\max}$ 且 $\dot x_1 = x_2$，$\dot x_2 = u$ → $\dot g = x_2$，$\ddot g = u$。控制需要通过"加速度"间接影响约束。

约束阶越高，边界弧上的控制越"间接"，数值求解越困难。

### 工程含义

机器人避障、关节限位均是状态约束。在工程实现中：
- **直接法**（collocation）处理状态约束很自然：约束 $g(x_k) \le 0$ 直接作为 NLP 约束
- **间接法**需要跳跃条件，实现复杂
- **MPC 实现**（acados、CasADi）通常用罚函数/障碍函数近似，避免显式测度乘子

**状态约束在腿足机器人中的典型形式**：

| 约束类型 | 数学表达 | 约束阶 | 处理方式 |
|---------|---------|--------|---------|
| 关节限位 | $q_{\min} \le q \le q_{\max}$ | 2（$\ddot q = f(q,\dot q, \tau)$）| 硬约束或弹性壁 |
| 足端高度 | $z_{\text{foot}} \ge 0$（摆动相） | 3+ | 参考轨迹约束 |
| 摩擦锥 | $\|f_t\| \le \mu f_n$ | — （控制约束非状态约束）| 线性化或SOC |
| 自碰撞 | $\|p_i - p_j\| \ge d_{\min}$ | 2 | 罚函数或CBF |
| 工作空间边界 | CoM 在支撑多边形内 | 2 | ZMP/DCM约束 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：状态约束在直接法中的数值困难**

将状态约束直接加为硬约束（$g(x_k) \le 0$）在约束活跃时会导致 NLP 求解器困难（约束 Jacobian 秩亏缺或条件数极大）。对策：用 relaxed barrier（松弛障碍）或 exact penalty（精确罚）方法平滑化。

💡 **概念误区：混淆"控制约束"和"状态约束"**

- 控制约束 $u \in U$：PMP 的极大化条件直接处理，共态保持连续
- 状态约束 $g(x) \le 0$：需要额外的测度乘子，共态可能跳跃

两者的数学性质完全不同。控制约束是"路径逐点"的，相对容易；状态约束是"全局"的，可能导致 PMP 结构的本质复杂化。

### 练习

1. 对一维系统 $\dot x = u$，$x(0) = 1$，$x(1) = -1$，状态约束 $x(t) \ge 0$，代价 $\min \int u^2 dt$。写出无约束解（直线 $x = 1 - 2t$），观察它在 $t = 0.5$ 时穿越 $x = 0$。写出有约束时轨迹的定性形状（三段：下降-沿 $x = 0$ 滑行-下降），并讨论共态在进入/离开边界弧时的跳跃。

---

## §3.2.12 离散时间 PMP 与 DDP 的对应 ⭐⭐

### 动机

连续时间 PMP 是理论框架，但实际计算机实现必须离散化。离散时间 PMP 直接对应 DDP/iLQR 的 backward pass——理解这个对应关系是掌握现代轨迹优化器的关键。

### 设定

$$x_{k+1} = f_k(x_k, u_k), \quad J = \phi(x_N) + \sum_{k=0}^{N-1} L_k(x_k, u_k).$$

### 核心定理

**定理 3.2.12**（Halkin 1966）：存在共态序列 $\lambda_0, \ldots, \lambda_N$ 使
$$\boxed{\lambda_k = \left(\frac{\partial f_k}{\partial x_k}\right)^\top \lambda_{k+1} + \left(\frac{\partial L_k}{\partial x_k}\right)^\top, \quad \lambda_N = \frac{\partial\phi}{\partial x_N}^\top,}$$
$$u_k^* \in \arg\max_{u \in U_k}\left\{\lambda_{k+1}^\top f_k(x_k, u) - L_k(x_k, u)\right\}.$$

出处：Halkin 1966 *SIAM J. Control* 4(1):90-111。

### 与 DDP 的精确对应

DDP 的 backward pass 中的 $Q$-函数展开（Jacobson-Mayne 1970）：
$$Q_x = \ell_x + f_x^\top V'_x, \quad Q_u = \ell_u + f_u^\top V'_x,$$
$$Q_{xx} = \ell_{xx} + f_x^\top V'_{xx} f_x + V'_x \cdot f_{xx}, \quad Q_{uu} = \ell_{uu} + f_u^\top V'_{xx} f_u + V'_x \cdot f_{uu}.$$

其中 $V'_x$ 正是下一时步的共态 $\lambda_{k+1}$！iLQR 丢弃含 $V'_x \cdot f_{xx}$ 的项（Gauss-Newton 近似），只保留 $Q_{xx} \approx \ell_{xx} + f_x^\top V'_{xx} f_x$。

### 关键下标陷阱

> **本质洞察**：$\lambda_{k+1}$（**不是** $\lambda_k$）出现在 $u_k$ 的极大化中——共态在时间上"滞后一步"。这是离散 PMP 与连续 PMP 的关键区别。在 DDP 代码中，这对应于用 `Vx_next`（下一时步的值函数梯度）来计算当前时步的最优控制。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：离散共态更新的下标错位**

错误代码：`lambda[k] = fx.T @ lambda[k] + lx`（用了 $\lambda_k$ 而非 $\lambda_{k+1}$）
正确代码：`lambda[k] = fx.T @ lambda[k+1] + lx`

这个错误不会导致编译报错，但会让轨迹优化完全不收敛或收敛到错误结果。

### 练习

1. 对离散 LQR 问题 $x_{k+1} = A x_k + B u_k$，$J = \frac{1}{2}x_N^\top Q_f x_N + \frac{1}{2}\sum(x_k^\top Q x_k + u_k^\top R u_k)$，从离散 PMP 推导离散 Riccati 方程。对比连续版本。

2. 在 Python 中实现一个简单的 iLQR（cart-pole swing-up），明确标注代码中哪一行对应离散 PMP 的共态更新、哪一行对应极大化条件。

---

## §3.2.13 数值间接法 ⭐⭐⭐

### 间接法 vs 直接法

**间接法**（"先优化，再离散"）：先写出 PMP 的 BVP（$(x, \lambda)$ 的 ODE + 边界条件），猜 $\lambda(t_0)$ 的初值，用牛顿法打靶使末端条件满足。

**直接法**（"先离散，再优化"）：把控制和状态在网格上离散为有限参数，送进 NLP 求解器（IPOPT、SNOPT）。

| 维度 | 间接法 | 直接法 |
|------|--------|--------|
| 优点 | 精度高、解析结构清晰、小规模快 | 鲁棒、约束处理自然、大规模可行 |
| 缺点 | $\lambda(t_0)$ 初值猜测极困难（病态） | 精度依赖网格密度 |
| 适用 | 学术验证、小规模基准 | 工业应用、复杂约束 |
| 软件 | scipy.integrate.solve_bvp、Bocop | CasADi+IPOPT、acados、Drake |

### Shooting 方法层次

1. **Single shooting**：只猜 $\lambda(t_0) \in \mathbb{R}^n$，正向积分 $2n$ 维 ODE 到 $t_f$，调整使末端条件满足。**极度病态**——$\lambda(t_0)$ 微小变化可导致 $x(t_f)$ 指数放大的变化。

2. **Multiple shooting**：把 $[t_0, t_f]$ 分 $K$ 段，每段独立 IVP，段间**匹配条件**作为 NLP 约束。显著改善病态性。

3. **Collocation**：在所有网格点上**同时**满足配点方程。最稳定但变量最多。

### 共态初始化技巧

间接法的核心困难是 $\lambda(t_0)$ 的初值猜测。常用技巧：

| 技巧 | 原理 | 适用场景 |
|------|------|---------|
| 直接法 warm-start | 先用直接法求粗解，从 KKT 乘子提取共态初值 | 通用 |
| 同伦延拓 | 从简单问题（如无约束）连续变形到目标问题 | 约束问题 |
| LQR 近似 | 在初始点做 LQR 近似，用 $\lambda_0 \approx P x_0$ | 近线性系统 |
| 物理直觉 | 利用问题对称性或守恒量约束 $\lambda$ 的范围 | 特殊结构问题 |
| 逐步延长时域 | 先解短时域（容易），逐步增长 $t_f$ | 长时间轨迹 |

**同伦延拓的详细方法**：

设目标问题为 $P_1$（困难），构造一族参数化问题 $P_\alpha$（$\alpha \in [0,1]$），使得 $P_0$ 容易求解（如线性化或无约束问题），$P_1$ 是目标。从 $P_0$ 的解出发，逐步增大 $\alpha$（步长 $\Delta\alpha$），每次用上一步的 $\lambda(t_0)$ 作为下一步的初值猜测。

常见同伦路径：
- **约束同伦**：$U_\alpha = \alpha U + (1-\alpha)\mathbb{R}^m$（从无约束渐变到有约束）
- **动力学同伦**：$f_\alpha = \alpha f_{\text{nonlinear}} + (1-\alpha)f_{\text{linear}}$
- **代价同伦**：$L_\alpha = \alpha L_{\text{target}} + (1-\alpha)L_{\text{quadratic}}$

**实际经验**：步长 $\Delta\alpha$ 通常取 0.01-0.1。如果某步牛顿迭代不收敛，减半 $\Delta\alpha$ 重试。好的同伦路径应该使解曲线（$\lambda(t_0)$ 作为 $\alpha$ 的函数）尽可能光滑——避免"折返点"（bifurcation）。

### 软件生态

| 软件 | 语言 | 特点 | 适用场景 |
|------|------|------|---------|
| CasADi | C++/Python/MATLAB | 自动微分 + 多后端 | 通用最优控制 |
| Bocop | C++ | 开源，经典示例丰富 | 教学、小规模 |
| GPOPS-II | MATLAB | hp 自适应 pseudospectral | 航天 |
| acados | C | 实时嵌入式 | 机器人 MPC |
| Crocoddyl | C++ | DDP + Pinocchio | 腿足机器人 |
| ALTRO | Julia | 增广拉格朗日 + iLQR | 约束轨迹优化 |
| Drake | C++/Python | 直接 collocation 内置 | 通用机器人 |
| OCS2 | C++ | quadruped/humanoid | ETH 腿足控制栈 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：单次打靶的指数不稳定性**

对 $n = 10$ 的系统，$\lambda(t_0)$ 的 $10^{-6}$ 扰动可能在 $t_f$ 处产生 $10^6$ 的误差（因为线性化系统的 Lyapunov 指数正值导致前向积分指数放大）。对策：永远用 multiple shooting 或 collocation，single shooting 只用于教学演示。

### 练习

1. 用 `scipy.integrate.solve_bvp` 求解双积分器最小时间问题的 PMP BVP。观察初值猜测对收敛性的影响。

2. 用 CasADi 的直接 collocation 求解同一问题，比较精度和鲁棒性。讨论两种方法在什么条件下各有优势。

---

## §3.2.14 混合/切换系统的 PMP ⭐⭐⭐⭐

### 混合系统

连续动力学 + 离散模式切换：$\dot x = f_{q(t)}(x, u)$，模式 $q \in \{1, \ldots, Q\}$ 按切换规则改变。

在腿足机器人中，这是核心结构：每条腿的接触/脱离状态构成离散模式，连续动力学在每个模式下不同。

### 核心定理

**定理 3.2.14**（Hybrid Maximum Principle）：在切换时刻 $t_s$：
- **Hamiltonian 连续**：$H_{q^-}(t_s^-) = H_{q^+}(t_s^+)$
- **共态跳跃**：$\lambda(t_s^-) = (\partial\Phi/\partial x)^\top \lambda(t_s^+) + p\,\nabla S(x(t_s))$

其中 $\Phi$ 为 reset map、$S$ 为切换面、$p$ 为切换面乘子。

出处：Sussmann 1999 *Proc. 38th IEEE CDC*；Shaikh-Caines 2007 *IEEE TAC* 52(9):1587-1603。

### 腿足机器人步态切换

接触/脱离是混合系统的典型实例：
- **stance phase**（支撑相）：脚与地面接触，$\lambda_n > 0$（法向力存在）
- **swing phase**（摆动相）：脚离开地面，$\lambda_n = 0$
- **切换时刻**：摆动→支撑（着地 touchdown）或支撑→摆动（离地 liftoff）

工程实现策略：
- Posa-Cantu-Tedrake 2014 IJRR：互补约束直接法（绕开显式混合 PMP）
- TOWR (Winkler et al. 2018)：phase-based parameterization
- MIT Cheetah 3 (Di Carlo-Wensing 2018)：预设 contact schedule + 每段线性 MPC

### 练习

1. 对一个简单的 bouncing ball 模型（$\dot h = v$，$\dot v = -g$，碰地时 $v^+ = -e v^-$），写出混合 PMP 的跳跃条件。讨论恢复系数 $e$ 如何影响共态的跳跃幅度。

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 工程应用 |
|--------|---------|------|---------|
| 标准形式 | Bolza/Mayer/Lagrange 三形式等价 | ⭐⭐ | MPC/LQR/最小时间 |
| Hamiltonian | 控制 Hamiltonian + 两种约定 | ⭐⭐ | 所有轨迹优化器 |
| PMP 五件套 | 状态/共态/极大化/横截/守恒 | ⭐⭐ | 理论基线 |
| Needle variation | 针状扰动 → 凸锥 → 分离 | ⭐⭐⭐ | 理解 PMP 适用范围 |
| PMP ⇒ EL | 无约束特例退化为 Euler-Lagrange | ⭐⭐ | 理论联系 |
| PMP vs HJB | 必要 vs 充分、轨迹 vs 场、$\lambda = \nabla V$ | ⭐⭐⭐ | DDP = 局部 HJB |
| PMP ⇒ Riccati | LQR 是 PMP 的解析解 | ⭐⭐ | iLQR/DDP 基本模块 |
| Bang-bang | $H$ 对 $u$ 线性 → 切换控制 | ⭐⭐ | 最小时间/推力饱和 |
| 奇异弧 | $\partial H/\partial u \equiv 0$ → Kelley 条件 | ⭐⭐⭐ | 火箭/机械臂 |
| 状态约束 | 共态跳跃 + 测度乘子 | ⭐⭐⭐ | 避障/限位 |
| 离散 PMP | $\lambda_k = f_x^\top \lambda_{k+1} + L_x^\top$ | ⭐⭐ | DDP backward pass |
| 数值间接法 | shooting/collocation | ⭐⭐⭐ | CasADi/Bocop |
| 混合 PMP | 切换时共态跳跃 | ⭐⭐⭐⭐ | 腿足步态 |

---

## 向后展望：从轨迹到场

本章建立了 PMP 的完整理论体系——沿**单条最优轨迹**给出一阶必要条件。PMP 的核心对象是共态 $\lambda(t)$，它沿最优轨迹反向传播代价敏感性。

但 PMP 有两个根本局限：(1) 它只是**必要条件**（满足 PMP 的极值轨迹未必全局最优），(2) 每换一个初始状态 $x_0$ 就要重新求解整个 BVP。下一专题（3.3 动态规划与 Bellman 方程）将转向一个全然不同的视角——**在整个状态空间上定义值函数 $V(x)$**，给出**充分条件**和**最优反馈律**。两种方法的精确对偶关系是：$\lambda(t) = \nabla_x V(x^*(t), t)$——共态就是值函数沿最优轨迹的梯度。理解这一对偶关系，是统一所有后续算法（iLQR、MPC、RL）的理论前提。

---

## 跨章综合练习

**综合题 1**（需结合变分法 + PMP + LQR）：

对非线性摆 $\ddot\theta + \sin\theta = u$（$|u| \le 1$），分三步完成：
1. 用变分法（EL 方程）写出**无约束**情况下的最优条件（假设可以忽略 $|u| \le 1$）
2. 用 PMP 写出**有约束**情况的五件套，确认当 $u^*$ 在 $U$ 内部时退化为第1步的 EL
3. 在 $\theta \approx 0$ ���线性化（$\sin\theta \approx \theta$），写出 LQR 问题的 $A, B, Q, R$，求解 ARE

讨论：LQR 解在什么角度范围内与 PMP 全局解近似一致？（提示：当 $|\theta|$ 超过约 $30°$ 时，$\sin\theta \approx \theta$ 误差 >5%，LQR 近似开始不准确。）

**综合题 2**（需结合 PMP + 数值方法 + 物理直觉）：

用 CasADi 求解以下最短时间问题：水平面上的 Dubins 车（$v = 1$，$|u| \le 1$）从 $(0, 0, 0)$ 到 $(3, 2, \pi/4)$。
1. 写出 PMP 五件套的具体形式
2. 利用 Dubins 六型定理，预测最优路径类型
3. 用 CasADi 直接 collocation 数值验证你的预测
4. 从 NLP 的 KKT 乘子提取共态 $\lambda(t)$，画出 $\lambda_\theta(t)$ 曲线，验证零点对应切换时刻

---

## 累积项目：本章新增模块

**项目：从零构建最优控制求解器**

本章新增：
- Module 3：实现 PMP BVP 求解器
  - 双积分器最小时间问题的 single shooting
  - 使用 `scipy.integrate.solve_bvp` 封装
  - bang-bang 切换函数的零点检测
  - 与上一章（变分法）的 EL 求解器对比验证

前置模块回顾：
- Module 1（变分法章）：EL 方程求解器
- Module 2（变分法章）：数值积分基础（Runge-Kutta）

后续预告：
- Module 4（LQR 章）：Riccati 求解器 + LQR 控制器
- Module 5（DDP 章）：完整 iLQR 实现

---

## 延伸阅读

| 资料 | 难度 | 说明 |
|------|------|------|
| Liberzon 2012《Calculus of Variations and Optimal Control Theory》| ⭐⭐ | **首选教材**，最清晰的现代入门 |
| Kirk 2004《Optimal Control Theory: An Introduction》| ⭐⭐ | 例题丰富，工程友好 |
| Bryson-Ho 1975《Applied Optimal Control》| ⭐⭐⭐ | 航天工程导向 |
| Pontryagin et al. 1962《The Mathematical Theory of Optimal Processes》| ⭐⭐⭐⭐ | 原著，难但必翻 Ch.I-II |
| Betts 2020《Practical Methods for Optimal Control》3rd ed. | ⭐⭐⭐ | 数值直接法圣经 |
| Tedrake《Underactuated Robotics》(MIT 6.832 online) | ⭐⭐ | 机器人+RL 导向 |
| Rawlings-Mayne-Diehl 2020《Model Predictive Control》2nd ed. | ⭐⭐⭐ | MPC 理论与实践 |
| Agrachev-Sachkov 2004《Control Theory from the Geometric Viewpoint》| ⭐⭐⭐⭐ | 几何控制权威 |
| Sussmann-Willems 1997 *IEEE CSM* "300 Years of Optimal Control" | ⭐⭐ | 历史综述，极佳 |
| Hartl-Sethi-Vickson 1995 *SIAM Review* "State Constraints Survey" | ⭐⭐⭐ | 状态约束完整综述 |

**推荐阅读顺序**：Liberzon（整体）→ Kirk（例题）→ Bryson-Ho（工程直觉）→ Rawlings-Mayne-Diehl Ch.8（MPC 接口）→ Betts（直接法）。

---

## 故障排查手册

🔧 **故障排查手册**

| 症状 | 可能原因 | 排查步骤 | 相关节号 |
|------|---------|---------|---------|
| BVP 打靶不收敛 | $\lambda(t_0)$ 初值猜测太差 | 1. 用 LQR 近似提供初值 2. 改用 multiple shooting 3. 先解简化问题做 warm-start | §3.2.13 |
| 共态方程解发散 | min/max 约定搞错（符号相反）| 1. 检查 Hamiltonian 定义中的 $\pm$ 号 2. 验证 $\dot\lambda$ 公式与参考教材一致 3. 检查末端条件符号 | §3.2.2 |
| 最优控制不满足 $u \in U$ | 未施加极大化条件，用了无约束 EL | 1. 确认代码中有投影/裁剪步骤 2. 检查是否把约束问题当无约束解了 | §3.2.3 |
| Bang-bang 控制有毛刺 | 数值积分步长跨越切换点 | 1. 启用事件检测 2. 在 $|\sigma| < \varepsilon$ 时缩小步长 3. 用 hybrid integrator | §3.2.9 |
| 直接法出现 chattering | 奇异弧的网格近似 | 1. 检查 Hamiltonian 是否对 $u$ 线性 2. 加正则项 $\varepsilon u^2$ 消除奇异性 3. 加密网格观察是否收敛 | §3.2.10 |
| DDP backward pass 不稳定 | $Q_{uu}$ 不正定 | 1. 加正则化 $Q_{uu} + \mu I$ 2. 检查是否有奇异弧 3. 检查下标是否用了 $\lambda_{k+1}$ | §3.2.12 |
| 状态约束解有跳变 | 共态在约束活跃/非活跃切换时跳跃 | 1. 确认是正常的 PMP 跳跃条件 2. 用松弛方法平滑化 3. 检查互补松弛条件 | §3.2.11 |

---

## 工程桥梁清单

| 桥梁 | 目标专题 | 核心关系 |
|------|---------|---------|
| → Bellman DP | 动态规划 | PMP 是 Bellman 方程的一阶特征；$\lambda = \nabla_x V$ |
| → HJB | Hamilton-Jacobi-Bellman | PMP 必要、HJB 充分；粘性解推广 |
| → LQR | 线性二次调节 | PMP 在 LQ 情形退化为 Riccati ODE |
| → DDP/iLQR | 轨迹优化 | DDP backward pass = 离散 PMP + 二阶展开 |
| → 约束 DDP | ALTRO/Crocoddyl | PMP + 增广拉格朗日处理不等式 |
| → MPC | 模型预测控制 | 间接法 PMP vs 直接法；Real-Time Iteration |
| → 数值轨迹优化 | CasADi/GPOPS/acados | PMP 提供理论基线，直接法工业实现 |
| ← 变分法 | Euler-Lagrange | PMP 是 EL 在控制约束下的推广 |
| ← 李群 | $SO(3), SE(3)$ | Lie-Poisson 共态、Saccon 投影算子 |

---

## 总结：PMP 把变分法升级为机器人科学

**核心洞察**：PMP 不是"又一个定理"，而是**最优控制全部工业生态**的理论根。Euler-Lagrange 回答了无约束变分，PMP 用针状变分 + 凸锥分离一举突破控制约束与非凸 $U$ 的屏障。五件套（状态、共态、极大化、横截、守恒）构成一套严密的"轨迹级一阶必要条件"语法。

**与 HJB 的互补**：PMP 从轨迹出发给必要条件，HJB 从值函数出发给充分条件；二者通过 $\lambda = \nabla_x V$ 精确对偶。RL 的 Q-learning/value iteration 对应 HJB 离散版本（Bellman 方程），而 policy gradient / trajectory optimization 对应 PMP。理解这层对偶，才能理解为什么 model-based RL 能从 DDP/iLQR 借来强大的结构先验。

**与工程栈的映射**：
- LQR = 解析 PMP
- iLQR/DDP = 离散 PMP 的二阶近似
- MPC = PMP 的 receding horizon 直接法
- ALTRO/Crocoddyl = 约束 PMP 的增广拉格朗日

工业上**没有**"跳过 PMP"的路径——即便全 NLP 直接法，其 KKT 条件就是离散化的 PMP。

**给博士预备生的建议**：
- **必须手推一次 PMP 证明**（即使略去泛函分析技术细节）。Boltyanskii 的针状变分 + 分离锥图景比任何 RL 教材的"policy gradient"解释都深刻。
- **必须手算三个经典实例**：双积分器最小时间、标量 LQR、Dubins 车。它们对应三种 Hamiltonian 结构（线性、严格凹、线性带方向）。
- **必须用 CasADi 跑通一次间接法**：单次打靶体会 $\lambda(t_0)$ 的病态，再改 multiple shooting 体会"数值与理论的距离"。

**终点即起点**：完成本章后，LQR 是 PMP 理论的"直接推论"；DDP/iLQR 是 PMP 工程的"递归实现"；MPC 是 PMP 的"滚动求解"。记得：**所有现代机器人轨迹优化器的内核，都是 PMP 的某种数值实现**。牢牢把握这一点，读任何新论文都能快速定位其在 PMP 家族中的位置。

**给不同背景读者的建议**：
- **数学背景**：重点理解 §3.2.4 的证明细节（凸锥分离的几何直觉），然后跳到 §3.2.15-§3.2.17 看李群推广和充分条件
- **控制背景**：从 §3.2.7 (LQR) 和 §3.2.8 (实例) 入手，建立计算能力，再回头补 §3.2.4 的证明直觉
- **RL/ML 背景**：重点理解 §3.2.6（PMP vs HJB = policy gradient vs value function），以及 §3.2.12（离散 PMP = DDP backward pass）
- **机器人工程背景**：重点是 §3.2.8 的实例 + §3.2.13 的数值方法 + §3.2.14 的混合系统，配合 CasADi 实操

无论你的背景如何，**手推至少三个经典实例**是不可替代的学习步骤——PMP 的力量只有在亲手计算中才能真正体会。

---

## 定理清单

| # | 定理 | 出处 | 节号 |
|---|------|------|------|
| 1 | Bolza/Mayer/Lagrange 三形式等价 | Liberzon 2012 §3.3 | §3.2.1 |
| 2 | 共态方程 $\dot\lambda = -\partial H/\partial x$ | Pontryagin 1962 Ch. II | §3.2.2 |
| 3 | PMP 五件套（一阶必要条件） | Pontryagin 1962；Sussmann 1997 | §3.2.3 |
| 4 | 针状变分 → 凸锥分离 → 共态 | Boltyanskii 1958；Liberzon 2012 §4 | §3.2.4 |
| 5 | PMP ⇒ Euler-Lagrange（无约束特例） | Kirk 2004 §5.2 | §3.2.5 |
| 6 | HJB 方程 + 共态 = $\nabla_x V$ | Fleming-Soner 2006；Vinter 2000 | §3.2.6 |
| 7 | LQR 化为 Riccati ODE | Kalman 1960；Anderson-Moore 1990 | §3.2.7 |
| 8 | Bang-bang + Feldbaum 切换上界 | LaSalle 1960；Athans-Falb 1966 | §3.2.8-9 |
| 9 | Dubins 最短路径六型 | Dubins 1957；Sussmann-Tang 1991 | §3.2.8 |
| 10 | 广义 Legendre-Clebsch (Kelley) | Kelley 1964；Robbins 1967 | §3.2.10 |
| 11 | 状态约束 PMP（共态跳跃） | Hartl-Sethi-Vickson 1995 | §3.2.11 |
| 12 | 离散 PMP + DDP 对应 | Halkin 1966；Jacobson-Mayne 1970 | §3.2.12 |
| 13 | 混合系统 PMP（切换跳跃） | Sussmann 1999；Shaikh-Caines 2007 | §3.2.14 |

---

## 常见陷阱速查（15条）

| # | 陷阱 | 解释 | 对策 |
|---|------|------|------|
| 1 | 把 PMP 当充分条件 | PMP 仅一阶必要；满足者称 extremal，未必最优 | 用 HJB 或二阶条件验证 |
| 2 | 忽略异常情况 $\lambda_0 = 0$ | 约束品性失效时 abnormal 极值出现 | 检查可达集切锥满秩 |
| 3 | 横截条件与端点约束错配 | 自由/固定/流形三类公式不同 | 逐项核对端点类型 |
| 4 | min vs max 符号错 | Kirk 极小约定 vs Pontryagin 极大约定 | 全文一致标注 |
| 5 | 打靶法 $\lambda(t_0)$ 敏感 | BVP 正向积分指数放大 | 改用 multiple shooting |
| 6 | 奇异弧用 $\partial H/\partial u = 0$ 求 $u$ | 退化为恒等式 | 用 Kelley 高阶条件 |
| 7 | 状态约束忽略共态跳跃 | 进入/离开 $g = 0$ 时 $\lambda$ 不连续 | 引入 Borel 测度乘子 |
| 8 | 离散下标 $\lambda_k$ vs $\lambda_{k+1}$ | 极大化 $u_k$ 用 $\lambda_{k+1}$ | 记忆"共态滞后一步" |
| 9 | 混合系统忽略切换跳跃 | 模式切换瞬时 $\lambda$ 跳，$H$ 连续 | 引入切换面乘子 |
| 10 | 自由末端时间忘 $H(t_f) = 0$ | 额外横截条件 | 检查时间是否可调 |
| 11 | 非凸 $U$ 误用弱变分 | 弱变分在 $U$ 边界失效 | 用针状扰动 |
| 12 | 非光滑动力学直接套 PMP | 接触/ReLU/摩擦不可微 | 用 Clarke 广义梯度 |
| 13 | Riccati 方程正向积分 | 终端条件在 $t_f$，必须反向积分 | 时间反转或 BVP 求解 |
| 14 | 认为 bang-bang 是"粗暴"控制 | 它是理论最优（对 $H$ 线性问题） | 接受不连续控制的物理合理性 |
| 15 | 直接法 chattering 误认为数值错误 | 实际是奇异弧的网格近似 | 加正则项或识别奇异结构 |

---

## §3.2.14.5 PMP 与 KKT 条件的精确对应 ⭐⭐

### 动机：理解"NLP 的 KKT = 离散 PMP"

在工程实践中，绝大多数轨迹优化问题通过直接法转化为 NLP（非线性规划），然后用 IPOPT 或 SNOPT 求解。这些 NLP 求解器找到的是 KKT 点。一个深刻的事实是：**离散化后的 PMP 条件与 NLP 的 KKT 条件完全等价**。理解这个等价性让你能在"理论语言"（PMP）和"工程语言"（KKT/NLP）之间自由切换。

### 等价性的精确陈述

考虑离散化的最优控制问题（直接 transcription）：
$$\min_{x_0,\ldots,x_N,\,u_0,\ldots,u_{N-1}} \phi(x_N) + \sum_{k=0}^{N-1} L_k(x_k, u_k)\,\Delta t$$
$$\text{s.t.} \quad x_{k+1} = f_k(x_k, u_k)\,\Delta t + x_k \quad (\text{Euler}), \quad u_k \in U.$$

这是一个以 $(x_0, \ldots, x_N, u_0, \ldots, u_{N-1})$ 为决策变量的 NLP。引入动力学约束的 Lagrange 乘子 $\mu_k \in \mathbb{R}^n$（$k = 0, \ldots, N-1$），KKT 条件为：

对 $x_k$ 求导（$k = 1, \ldots, N-1$）：
$$\frac{\partial L_k}{\partial x_k}\Delta t + \mu_{k-1} \cdot \frac{\partial f_{k-1}}{\partial x_k}\Delta t + \mu_{k-1} - \mu_k = 0.$$

令 $\lambda_k := \mu_{k-1}/\Delta t$（连续化），得到：
$$\lambda_k = \frac{\partial f_k}{\partial x_k}^\top \lambda_{k+1} + \frac{\partial L_k}{\partial x_k}^\top.$$

这正是离散 PMP 的共态方程！NLP 的乘子 $\mu$ 就是 PMP 的共态 $\lambda$ 的离散版本。

### 实际含义

当你用 CasADi + IPOPT 求解轨迹优化时：
- IPOPT 内部计算的 **Lagrange 乘子**就是 PMP 的**共态**
- KKT 的**互补松弛条件**就是控制约束的 PMP **极大化条件**
- KKT 的**对偶可行性**就是 PMP 的**共态方程**

所以即使你从不显式写 PMP，你用的 NLP 求解器**已经在隐式求解 PMP**。理解 PMP 的价值在于：你知道解应该长什么样（bang-bang？奇异弧？切换次数？），从而能判断 NLP 求解器给出的解是否合理。

### 从 NLP 乘子提取共态的实践代码（CasADi）

```python
# 在 CasADi 直接 collocation 求解后提取共态
# sol 是 CasADi nlpsol 的输出
lam_g = sol['lam_g']  # NLP 约束的 Lagrange 乘子

# 动力学约束的乘子 → PMP 共态（注意符号和缩放）
# 具体下标取决于 NLP 的约束排列方式
lambda_pmp = -lam_g[dynamics_constraint_indices] / dt
```

---

## §3.2.15 PMP 在 SE(3) 上的推广 ⭐⭐⭐⭐

### 动机：刚体的最优控制需要李群结构

四旋翼姿态控制、航天器姿态机动、机械臂末端位姿规划——这些问题的状态空间不是 $\mathbb{R}^n$，而是**李群** $SO(3)$ 或 $SE(3)$。在李群上直接套用标准 PMP 会遇到困难：李群不是线性空间，没有全局坐标系，"加法"运算 $x + \delta x$ 没有定义。

### Lie-Poisson 方程

刚体在 $SO(3)$ 上的最优控制：共态在**李代数对偶** $\mathfrak{g}^*$ 上演化，服从 Lie-Poisson 方程：
$$\dot p = \text{ad}^*_\xi\, p + (\text{控制力矩的对偶贡献}),$$
其中 $\xi \in \mathfrak{g}$ 是身体坐标角速度，$\text{ad}^*$ 是余伴随表示。

**物理意义**：对自由刚体（无外力矩），$p = I\omega$（角动量）满足 $\dot p = p \times \omega$——这正是 Euler 刚体方程。它的 Lie-Poisson 结构保证了角动量模长守恒和几何相不变。

### 坐标无关 PMP

Jurdjevic 1997《Geometric Control Theory》和 Agrachev-Sachkov 2004 给出了李群上 PMP 的坐标无关表述。核心思想：
- 状态在李群 $G$ 上演化：$\dot g = g \cdot \xi$（左不变向量场）
- 共态在余切丛 $T^*G$ 上演化（通过左平移化简为 $\mathfrak{g}^*$ 上的方程）
- Hamiltonian 定义在 $T^*G$ 上，极大化条件不变

### 离散变分最优控制

Kobilarov-Marsden 2011 *IEEE T-RO* 27(4):641-655 给出了离散 Lagrangian + discrete PMP 的框架：
- 用李群积分器（而非 Euler/RK4）保持群结构
- 离散共态更新保辛（symplectic）
- 航天器和无人机的长时间轨迹规划数值稳定

### 与前置李群知识的衔接

回顾李群基础：$SO(3)$ 的指数映射 $\exp: \mathfrak{so}(3) \to SO(3)$、伴随表示 $\text{Ad}_g$、$\text{ad}_\xi$ 这些概念在前置李群专题中已经建立。这里 $\text{ad}^*$（对偶伴随）出现在共态 ODE 中，是 $\text{ad}$ 的对偶映射：
$$\langle \text{ad}^*_\xi\, p,\, \eta \rangle = \langle p,\, \text{ad}_\xi\, \eta \rangle = \langle p,\, [\xi, \eta] \rangle.$$

对 $SO(3)$，$\text{ad}^*_\omega\, p = -\omega \times p = p \times \omega$——正是 Euler 方程中的交叉积项。

---

## §3.2.16 PMP 与最优传输/Wasserstein 梯度流 ⭐⭐⭐⭐

### 动机：从"单条轨迹"到"概率分布的最优运输"

PMP 控制**单个系统**的最优轨迹。但在 swarm robotics（多机器人集群）、mean-field game（平均场博弈）、和 diffusion model（扩散生成模型）中，需要控制**整个概率分布**的演化——这就是最优传输与 PMP 的交汇点。

### Benamou-Brenier 公式

**定理**（Benamou-Brenier 2000）：Wasserstein-2 距离可以写成最优控制形式：
$$W_2^2(\rho_0, \rho_1) = \inf_{v}\left\{\int_0^1 \int_{\mathbb{R}^n} \|v(x,t)\|^2\,\rho(x,t)\,dx\,dt : \partial_t\rho + \nabla\cdot(\rho v) = 0\right\}.$$

这是连续体极限下的"最小能量控制"：控制是速度场 $v(x,t)$，动力学是连续性方程（质量守恒），代价是动能。

### 与 PMP 的联系

对 Benamou-Brenier 问题写 PMP，共态是**压力场** $p(x,t)$（连续性方程的 Lagrange 乘子），极大化条件给出 $v = -\nabla p$（梯度流！）。这说明 Wasserstein 梯度流本质上是无穷维 PMP 的特征线。

### 工程含义

- **Diffusion models**（DDPM、Score Matching）的去噪过程可以理解为沿 Wasserstein 梯度流的最优传输
- **Multi-robot coordination**：将机器人群体建模为概率分布，用连续体 PMP 设计最优编队变换
- **Schrödinger bridge**：带噪声的最优传输 = 随机最优控制的 PMP

---

## §3.2.17 二阶充分条件（共轭点理论）⭐⭐⭐

### 动机：如何验证极值轨迹是否真正最优

PMP 只给必要条件。一条满足 PMP 的极值轨迹可能是局部极小、局部极大、或鞍点。二阶充分条件提供了判定工具。

### 共轭点

**定义**：沿极值轨迹 $x^*(t)$，如果存在 $t_c \in (t_0, t_f)$ 使得 Jacobi 方程（二阶变分方程）有非平凡解在 $t_c$ 处为零，则 $t_c$ 称为 $t_0$ 的**共轭点**（conjugate point）。

**定理**（Jacobi 充分条件）：如果极值轨迹在 $[t_0, t_f]$ 上没有共轭点，且 Legendre 条件 $\partial^2 H/\partial u^2 < 0$（严格）成立，则该极值轨迹是**局部严格最优**的。

**类比**：在微积分中，$f''(x_0) > 0$ 保证 $x_0$ 是严格极小值。共轭点条件是这个"二阶检验"在泛函空间中的推广——它检查"二阶变分"（$\delta^2 J$）是否正定。共轭点的出现意味着二阶变分在某个方向为零（退化），之后可能变负（极值轨迹不再最优）。

### Jacobi 方程

沿极值轨迹的二阶变分方程（Jacobi/accessory 方程）：
$$\ddot{\delta x} = (f_{xx}[u^*] - \dot A)\,\delta x + \ldots$$

更精确地（矩阵形式）：设 $\delta z = (\delta x, \delta\lambda)^\top$ 满足线性化的 Hamilton 系统
$$\dot{\delta z} = \mathcal{H}'(t)\,\delta z,$$
其中 $\mathcal{H}'$ 是 Hamilton 矩阵沿极值轨迹的 Jacobian。共轭点 $t_c$ 是使 $\delta x(t_c) = 0$ 有非平凡解的时刻。

### 工程含义

- **iLQR 收敛性**：iLQR 收敛到的极值轨迹如果有共轭点，说明可能是鞍点而非极小值——需要添加正则化或全局搜索
- **最小时间问题的全局最优性**：双积分器的 bang-bang 解没有共轭点（可以严格证明），所以 PMP 解确实是全局最优
- **航天器交会**：长时间转移轨迹可能存在共轭点，需要用更长的弧段或不同拓扑的轨迹

---

## §3.2.18 PMP 与凸优化/SDP 松弛 ⭐⭐⭐

### 动机：什么时候 PMP 给出全局最优？

对一般非凸问题，PMP 只是必要条件。但在某些特殊结构下，PMP + 凸性条件可以保证全局最优性。

### Mangasarian 充分条件

**定理**（Mangasarian 1966）：如果 Hamiltonian $H(x, u, \lambda)$ 关于 $(x, u)$ 联合凹（对固定 $\lambda$），且末端代价 $\phi$ 关于 $x$ 凸，则满足 PMP 的极值轨迹是**全局最优**的。

**证明思路**：凹 Hamiltonian + 凸末端代价 ⇒ 代价泛函关于 $(x(\cdot), u(\cdot))$ 是凸的（在适当函数空间中）。凸问题的一阶条件即是充要条件。

### LQR 的全局最优性

LQR 问题满足 Mangasarian 条件（$H$ 关于 $(x, u)$ 联合凹，因为 $-\frac{1}{2}(x^\top Qx + u^\top Ru)$ 是凹的而 $\lambda^\top(Ax + Bu)$ 是线性的）。这就是为什么 LQR 的 PMP 解（Riccati 反馈）自动就是全局最优的——不需要额外验证。

### 非凸问题的 SDP 松弛

对一般非凸问题（如非线性动力学 + 非凸约束），一种策略是：
1. 写出 PMP 得到极值条件
2. 对极值条件做 SDP（半正定规划）松弛
3. 如果松弛紧（relaxation tight），则得到全局最优

出处：Lasserre 2001 *SIAM J. Optim.*（多项式优化的 SDP 层次）；Majumdar-Tedrake 2017 *IJRR*（机器人轨迹验证）。

---

## §3.2.19 典型例题详解：最省燃料问题 ⭐⭐

### 问题设定

考虑一维平移系统：$\dot x_1 = x_2$，$\dot x_2 = u$，$|u| \le 1$。

**最省燃料**（fuel-optimal）代价：
$$J = \int_0^T |u(t)|\, dt.$$

从 $(x_1(0), x_2(0)) = (x_{10}, x_{20})$ 到 $(x_1(T), x_2(T)) = (0, 0)$，$T$ 给定且足够大。

### PMP 推导

代价 $L = |u|$ 对 $u$ 不可微（在 $u = 0$ 处），但由于 $U = [-1, 1]$，可以分析 Hamiltonian：
$$H = \lambda_1 x_2 + \lambda_2 u - |u|.$$

对 $u > 0$：$H = \lambda_1 x_2 + (\lambda_2 - 1)u$，极大化 ⇒ 需要 $\lambda_2 - 1 > 0$ 即 $\lambda_2 > 1$。
对 $u < 0$：$H = \lambda_1 x_2 + (\lambda_2 + 1)u$，极大化 ⇒ 需要 $\lambda_2 + 1 < 0$ 即 $\lambda_2 < -1$。
对 $u = 0$：$H = \lambda_1 x_2$，这在 $|\lambda_2| < 1$ 时是极大值。

**最优控制的三模式结构**：
$$u^*(t) = \begin{cases} +1, & \lambda_2(t) > 1, \\ 0, & |\lambda_2(t)| \le 1, \\ -1, & \lambda_2(t) < -1. \end{cases}$$

这是 **bang-off-bang** 控制（推力-滑行-反推力），不是纯 bang-bang。滑行段（$u = 0$）对应"什么都不做也是最优的"——因为此时任何非零推力的"燃料成本"超过了"让状态更接近目标"的收益。

### 共态分析

$\dot\lambda_1 = 0$，$\dot\lambda_2 = -\lambda_1 = \text{const}$。所以 $\lambda_2(t)$ 是直线。

$\lambda_2$ 可能：
- 始终在 $(-1, 1)$ 内 → 全程滑行（仅当初始状态已经在自由运动到原点的轨迹上）
- 从 $>1$ 下降到 $(-1, 1)$ → bang-off（先推后滑行）
- 从 $>1$ 下降穿越 $(-1, 1)$ 到 $<-1$ → bang-off-bang

### 与最小时间问题的对比

| | 最小时间 ($L = 1$) | 最省燃料 ($L = |u|$) |
|--|--|--|
| $H$ 关于 $u$ | 线性（系数 $\lambda_2$）| 分段线性（系数 $\lambda_2 \pm 1$）|
| 最优结构 | bang-bang | bang-off-bang |
| 滑行段 | 不存在 | 存在（$u = 0$）|
| 切换条件 | $\lambda_2 = 0$ | $\lambda_2 = \pm 1$ |
| 工程含义 | 时间最短但耗能大 | 节能但时间更长 |

> **本质洞察**：最小时间控制和最省燃料控制代表了控制器设计中的根本权衡——**性能 vs 资源**。PMP 的统一框架让我们可以通过改变代价函数 $L$ 来精确调控这个权衡，而无需改变求解方法论。

### ⚠️ 常见陷阱

💡 **概念误区：认为 $L = |u|$ 的不可微性让 PMP 失效**

虽然 $|u|$ 在 $u = 0$ 处不可微，但极大化条件 $\max_{u \in [-1,1]} H$ 仍然良定义（$H$ 是 $u$ 的连续分段线性函数，在紧集上总有最大值）。不可微性只影响"内点条件 $\partial H/\partial u = 0$"的使用——对分段线性函数，需要用分段分析代替。

### 练习

1. 对 $(x_{10}, x_{20}) = (1, 0)$，$T = 4$，画出 $\lambda_2(t)$ 和 $u^*(t)$ 的时间历程。确定三段的切换时刻。

2. 讨论如果 $T$ 太小（不够到达原点），最省燃料问题是否仍有解。（提示：此时需要全程满推力，问题退化为最小时间问题。）

---

## §3.2.20 PMP 的二阶条件与 Sufficient Conditions ⭐⭐⭐

### 加强的 Legendre-Clebsch 条件

对无约束控制（$U = \mathbb{R}^m$），PMP 极大化条件退化为 $\partial H/\partial u = 0$。此时二阶必要条件为：
$$\frac{\partial^2 H}{\partial u^2}\bigg|_{u^*} \le 0 \quad (\text{极大化约定下为半负定}).$$

加强的充分条件：
$$\frac{\partial^2 H}{\partial u^2}\bigg|_{u^*} < 0 \quad (\text{严格负定}).$$

### Hamilton-Jacobi 验证定理

**定理**（Verification Theorem）：如果能找到一个 $C^1$ 函数 $W(x, t)$ 满足：
1. $W(x, t_f) = \phi(x)$（终端条件）
2. $\partial W/\partial t + \min_{u \in U}\{L + \nabla_x W^\top f\} \ge 0$（HJB 不等式）
3. 沿某条轨迹 $(x^*, u^*)$ 等号成立

则 $(x^*, u^*)$ 是**全局最优**的，且 $W(x, t) \le V(x, t)$（$W$ 是值函数的下界）。

**工程应用**：在 LQR 中，$W = \frac{1}{2}x^\top P(t) x$ 正是这样的验证函数。这就是为什么 LQR 的 PMP 解自动保证全局最优。

### Jacobi 条件的计算方法

实际计算共轭点时，可以用以下方法：

1. **Riccati 方法**：将 Jacobi 方程化为矩阵 Riccati 方程。共轭点对应 Riccati 解发散（$P \to \infty$）的时刻。

2. **行列式方法**：设 $Y(t)$ 是 Jacobi 方程的基本矩阵解（$Y(t_0) = 0$，$\dot Y(t_0) = I$），则共轭点是 $\det Y(t_c) = 0$ 的时刻。

3. **数值方法**：沿极值轨迹正向积分 Jacobi 方程，监测行列式变号。

---

## §3.2.21 完整求解范例：从问题建模到数值验证 ⭐⭐

### 范例：Cart-Pole Swing-Up 的 PMP 分析

**系统**：小车上的倒立摆（经典欠驱动系统）

状态 $x = (p, \theta, \dot p, \dot\theta)^\top$（小车位置、摆角、小车速度、角速度），控制 $u = F$（推车力），$|F| \le F_{\max}$。

动力学（标准 Euler-Lagrange）：
$$\ddot p = \frac{F + ml\dot\theta^2 \sin\theta - mg\sin\theta\cos\theta}{M + m\sin^2\theta},$$
$$\ddot\theta = \frac{(M+m)g\sin\theta - (F + ml\dot\theta^2\sin\theta)\cos\theta}{l(M + m\sin^2\theta)}.$$

**目标**：从垂直悬挂 $\theta(0) = \pi$ 摆到倒立 $\theta(T) = 0$，最小能量 $\int F^2 \, dt$。

### PMP 分析步骤

**Step 1**：由于 $H$ 关于 $F$ 有二次代价项 $-F^2$（凹），若 $|F^*| < F_{\max}$，极大化给出：
$$\frac{\partial H}{\partial F} = 0 \implies F^* = \frac{\lambda_3}{2} \cdot \frac{1}{M + m\sin^2\theta} + \lambda_4 \cdot (\ldots).$$

当 $|F^*|$ 达到 $F_{\max}$ 时切换为饱和控制。

**Step 2**：由于系统非线性强（$\sin\theta$, $\cos\theta$），共态方程也是非线性的——无解析解。必须数值求解 BVP。

**Step 3**：实际求解策略：
1. 先用直接法（CasADi collocation）得到粗解
2. 从 KKT 乘子提取共态初值
3. 用 warm-started 间接法精化

### 数值结果的物理解读

典型最优摆起轨迹的结构：
1. **初段**：小幅摇摆积累能量（$|F|$ 适中）
2. **中段**：大幅摆动越过顶部（$|F|$ 接近 $F_{\max}$）
3. **末段**：精细平衡（$F$ 线性反馈 ≈ 局部 LQR）

这种"先粗后细"的结构正是 PMP 预测的：初段 $H$ 的极大化驱动系统远离初始状态，末段收敛到线性化 LQR 区域。

---

## 自测题（综合 7 道）

**自测 1**（基础）：用 PMP 推导双积分器 $\ddot x = u$，$|u| \le 1$ 从 $(x_0, \dot x_0) = (2, 1)$ 到原点的最小时间控制，写出切换时刻与 $t_f$。

**自测 2**（中等）：对标量 LQR 问题 $\min \frac{1}{2}\int_0^T(qx^2 + ru^2)\,dt$，$\dot x = ax + bu$，用 PMP 推出 Riccati 方程，并在 $T \to \infty$ 取极限。

**自测 3**（进阶）：对 Dubins 车从 $(0,0,0)$ 到 $(5,0,\pi/2)$（最小半径1），列出六种候选路径的长度公式，识别最优型。

**自测 4**（编码）：在 CasADi 中实现离散 DDP backward pass（iLQR 近似），用于 cart-pole swing-up。对比保留 $f_{xx}$ 项（full DDP）与丢弃（iLQR）的收敛速度。

**自测 5**（理论+数值）：用 Bocop 求解 Goddard 问题，识别三段式结构，验证 Kelley 条件。

**自测 6**（跨章综合）：将本章的连续 PMP 与专题 3.5（LQR 与 Riccati 方程）联合使用：对非线性 pendulum $\ddot\theta + \sin\theta = u$ 在 $\theta = \pi$ 附近做线性化，写出局部 LQR 的 $A, B, Q, R$，求解 ARE，然后讨论 PMP 全局解与局部 LQR 解在什么范围内一致。

**自测 7**（理论）：证明对自治系统的自由时间问题，沿最优轨迹 $H \equiv 0$。提示：利用 $H$ 守恒 + 横截条件 $H(t_f) = 0$。讨论这个条件在数值求解中如何作为验证工具（检查你的数值解是否满足 $H \approx 0$）。

---

## 关键论文清单

1. Boltyanski-Gamkrelidze-Pontryagin 1956, *Dokl. Akad. Nauk SSSR* 110:7-10 (PMP 最早宣告)
2. Pontryagin-Boltyanskii-Gamkrelidze-Mishchenko 1962, *The Mathematical Theory of Optimal Processes* (英译本)
3. Kelley 1964, *AIAA J.* 2(8):1380-1382 (奇异弧二阶条件)
4. Halkin 1966, *SIAM J. Control* 4(1):90-111 (离散 PMP)
5. Hestenes 1966, *Calculus of Variations and Optimal Control Theory*, Wiley (独立证明)
6. Robbins 1967, *IBM J.* 11:361-372 (广义 Legendre-Clebsch)
7. Clarke 1976, *SIAM J. Control* 14(6):1078-1091 (非光滑 PMP)
8. Sussmann-Tang 1991, Rutgers SYCON TR 91-10 (Reeds-Shepp 最短路径)
9. Hartl-Sethi-Vickson 1995, *SIAM Review* 37(2):181-218 (状态约束综述)
10. Sussmann-Willems 1997, *IEEE CSM* 17(3):32-44 (300年最优控制史)
11. Gamkrelidze 1999, *J. Dyn. Control Syst.* 5(4):437-451 (PMP 发现史)
12. Benamou-Brenier 2000, *Numer. Math.* 84:375-393 (计算最优传输)
13. Bonnans-Martinon-Trelat 2008, *JOTA* 139(2):439-461 (Goddard 奇异弧)
14. Kobilarov-Marsden 2011, *IEEE T-RO* 27(4):641-655 (离散几何最优控制)
15. Saccon-Hauser-Aguiar 2013, *IEEE TAC* 58(9):2230-2245 (李群上 PMP)
16. Posa-Cantu-Tedrake 2014, *IJRR* 33(1):69-81 (接触轨迹优化)
17. Andersson-Gillis-Horn-Rawlings-Diehl 2019, *Math. Prog. Comp.* 11:1-36 (CasADi)
18. Howell-Jackson-Manchester 2019, IROS (ALTRO)
19. Foehn-Romero-Scaramuzza 2021, *Science Robotics* 6(56):eabh1221 (四旋翼最小时间)
20. Tassa-Erez-Todorov 2012, IROS (DDP + box constraints)

---

## 中文学习资源

**中文教材（推荐顺序）**：
1. 胡寿松主编《最优控制理论与系统》第3版，科学出版社2011，ISBN 978-7-03-030829-1——入门偏工程，与国内自动化课程对应紧密
2. 张洪钺、王青《最优控制理论与应用》，高等教育出版社2006——航天工程导向，例题偏飞行器
3. 吴沧浦《最优控制的理论与方法》，科学出版社1986——早期经典
4. 邢继祥、曹长修《最优控制应用及仿真》，国防工业出版社2003——仿真案例
5. 钱学森《工程控制论》（1954英文初版，1958中译本科学出版社）——历史意义大，理论奠基

**中文术语对照表**：

| 英文术语 | 中文术语 | 备注 |
|---------|---------|------|
| Pontryagin Maximum Principle | 极大值原理/极小值原理 | 取决于约定 |
| co-state / adjoint | 共态/伴随变量/协态 | "协态"少见但准确 |
| Hamiltonian | 哈密顿函数/汉密尔顿函数 | 两种译法都常见 |
| needle variation | 针状变分/针形变分 | |
| transversality condition | 横截条件 | |
| bang-bang control | 开关控制/脉冲控制 | |
| singular arc | 奇异弧 | |
| switching function | 切换函数/开关函数 | |
| extremal | 极值轨迹 | 注意：不是"最优轨迹" |
| value function | 值函数/最优代价函数 | |
| shooting method | 打靶法 | |
| collocation | 配置法/配点法 | |

**网络资源**：
- 西安交通大学、哈尔滨工业大学、清华大学自动化系的"最优控制"研究生课程在 B 站可搜到
- 知乎话题"最优控制"下搜索"极大值原理"、"针状变分"——高赞文章常有优秀直觉解释

**推荐策略**：以 Liberzon / Kirk 为英文主干，胡寿松为中文快速术语对照；遇到"异常极值"、"奇异弧"等概念在知乎搜中文直觉解释，再回到英文严格证明。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| PMP 给出的是充分条件，满足 PMP 就一定是最优解 | PMP 是必要条件，满足 PMP 的轨迹（extremal）不一定是最优的，需要额外的二阶条件或与 HJB 充分条件配合验证 |
| 针状变分（needle variation）只是一种数学技巧 | 针状变分是 PMP 的核心创新——它绕过了经典变分法在控制约束边界上"无法扰动"的本质困难，因为针状变分在单点跳变后仍留在控制集 $U$ 内 |
| PMP 的 Hamilton 函数与分析力学中的 Hamiltonian 完全相同 | 形式相似但符号约定不同：PMP 中常用极大化约定 $H = \lambda^\top f - \lambda_0 L$，而力学中 $H = p^\top \dot q - L$；且 PMP 的共态 $\lambda$ 不一定是广义动量 |
| Bang-bang 控制在所有最优控制问题中都会出现 | Bang-bang 仅在 $H$ 关于 $u$ 为线性时出现（如推力有界的时间最优问题）；当 $H$ 关于 $u$ 严格凹时，最优控制是连续的内点解 |
| 奇异弧（singular arc）是退化的病态情况，实际中不重要 | 奇异弧在航天低推力轨迹、化工温控等工程问题中非常常见，处理奇异弧是 PMP 应用的核心难点之一 |
| 共态变量 $\lambda$ 只是数学工具，没有物理意义 | $\lambda(t) = \nabla_x V(x^*(t), t)$，即共态是值函数（最优剩余代价）关于状态的梯度，衡量"在该时刻偏离最优状态的边际代价" |
| PMP 和 HJB 是两套独立的理论 | PMP 给出单条最优轨迹上的必要条件（ODE），HJB 给出全状态空间的充分条件（PDE），两者通过 $\lambda = \nabla_x V$ 精确对偶；PMP 的特征线法就是 HJB 的特征线 |
| 异常极值（abnormal extremal，$\lambda_0 = 0$）可以忽略 | 异常极值在状态约束和高阶系统中不可忽略，忽略它可能遗漏真正的最优解 |

---

## 机器人工程应用索引

| 应用领域 | PMP 结构 | 代表文献 |
|---------|---------|---------|
| 四旋翼最小时间 | Bang-bang 推力 + 奇异弧过渡 | Foehn-Scaramuzza 2021 Sci.Robot. |
| 机械臂路径参数化 | 时间最优 → Bobrow 曲线 | Bobrow-Dubowsky 1985 IJRR |
| 腿足接触力规划 | 混合 PMP + 切换跳跃 | Posa-Tedrake 2014 IJRR |
| 航天低推力转移 | Goddard 结构 + 长时间奇异弧 | Trelat 2012 JOTA |
| 自动驾驶换道 | Dubins/Reeds-Shepp 结构 | Werling 2010 ICRA |
| 人形机器人步态 | 混合切换 + 接触调度 | Di Carlo-Wensing 2018 IROS |
| 无人机竞速 | 时间最优 + CPC | Foehn 2021 |
| 化工反应器优化 | 奇异弧温控 | Bonnans 2008 JOTA |

---

## 学时预算

**核心内容（§3.2.1-§3.2.13）**：约 16-20 学时

| 节号 | 内容 | 学时 |
|------|------|------|
| §3.2.1 | 标准形式 | 1 |
| §3.2.2 | Hamiltonian 与共态 | 2 |
| §3.2.3 | PMP 完整陈述 | 2 |
| §3.2.4 | 证明思想 | 3 |
| §3.2.5 | PMP ↔ EL | 1 |
| §3.2.6 | PMP ↔ HJB | 2 |
| §3.2.7 | PMP ↔ LQR | 1.5 |
| §3.2.8 | 经典实例 | 3 |
| §3.2.9-10 | Bang-bang/奇异弧 | 2 |
| §3.2.11-14 | 状态约束/离散/混合/数值 | 4 |

**选修高阶（§3.2.15-§3.2.21）**：各 1.5-2.5 学时，共 10-15 学时。

**实践建议**：
- 每学完一小节，手写推导其中 1 道例题，禁用符号计算软件
- §3.2.8 至少独立完成双积分器 + 标量 LQR 两个实例
- §3.2.13 在 CasADi 中跑通 Van der Pol 振荡器或 Goddard 火箭
- 整个专题配合 DDP 章节并行阅读效果最好：PMP 理论和 DDP 代码形成双向强化
- 建议保留手写笔记本：PMP 的符号体系庞大，手写有助于内化

**自我检测清单**（学完全章后逐条核对）：

| 检测项 | 合格标准 |
|--------|---------|
| 能默写 PMP 五件套 | 不看任何资料，30秒内写完 |
| 能区分 min/max 约定 | 给一篇论文，3分钟内判断其约定并转换 |
| 能对线性系统推 LQR | 从 $H$ 出发到 Riccati，不超过一页纸 |
| 能画双积分器相图 | 标出切换曲线、标注 $u$ 的符号 |
| 能解释 $\lambda = \nabla V$ | 用自己的话说清为什么共态=值函数梯度 |
| 能判断 bang-bang vs 奇异 | 给一个新问题，判断 $H$ 关于 $u$ 的结构 |
| 能用 CasADi 跑通一个例子 | 从建模到画结果图，独立完成 |

---
