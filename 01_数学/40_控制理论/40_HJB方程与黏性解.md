# 专题 3.4：Hamilton-Jacobi-Bellman 方程与黏性解——最优控制的"场论"

---

## 前置自测

> 📋 **前置自测**（答不出 ≥ 2 题 → 先回专题 3.2-3.3 复习）

1. 写出连续时间最优控制问题的 Bolza 形式，并标出 running cost 与 terminal cost。
2. Bellman 最优性原理的离散版核心公式是什么？（专题 3.3 的核心 DP 方程）
3. Pontryagin 极大值原理（PMP）的五件套必要条件分别是什么？（专题 3.2，注意专题 3.2 使用极大化约定）
4. 什么是一阶偏微分方程的特征线方法？特征线相交时会发生什么？
5. LQR 问题的最优反馈增益 $K$ 与 Riccati 方程的关系是什么？

---

## 本章目标

学完本专题后，你将能够：

1. **从离散 Bellman 方程出发**，通过严格的连续时间极限推导出 HJB 方程，理解每一步假设的含义；
2. **解释为什么 HJB 通常没有经典解**，能举出至少三类典型反例（Eikonal、bang-bang、状态约束）；
3. **陈述并解释黏性解的完整定义**，理解测试函数、超微分/次微分的几何含义；
4. **掌握比较原理与存在唯一性的证明思路**，理解"变量翻倍"技巧；
5. **利用验证定理**将 HJB 的经典解（当存在时）连接到最优控制策略；
6. **阐明 HJB 与 PMP 的对偶关系**：共态（协态）= 值函数梯度，特征线 = Hamilton 方程；
7. **了解数值方法**（Lax-Friedrichs、ENO/WENO、Level-Set、Semi-Lagrangian）及其收敛理论；
8. **能手动求解一维 HJB**（LQR、时间最优、Eikonal）并验证黏性解条件；
9. **理解现代进展**：DeepReach、路径积分控制（MPPI）、HJ Reachability 的工程应用。

---

## 知识树

```
                     Hamilton-Jacobi-Bellman 方程与黏性解
                                    │
        ┌───────────────────────────┼──────────────────────────┐
        │                           │                          │
  §3.4.1 值函数与 DPP         §3.4.5 经典解不存在          §3.4.7 LQR Riccati
        │                           │                          │
  §3.4.2 HJB 推导             §3.4.6 黏性解定义            §3.4.8 数值方法
        │                           │                          │
  §3.4.3 验证定理             比较原理·唯一性              维数灾难
        │                           │                          │
  §3.4.4 HJB 与 PMP          Perron 方法·存在性           现代路线
        │                                                      │
        │                    ┌─────────────────────┐           │
        └────────────────────┤  选学专题 (档位 4)   ├───────────┘
                             └─────────────────────┘
                              A: 随机 HJB (Itô-Bellman)
                              B: 最优停止与自由边界
                              C: 微分博弈与 Isaacs
                              D: HJ Reachability
                              E: 路径积分控制 (MPPI)
                              F: PINN/DeepReach
```

---

## 历史脉络：Hamilton 1834 → Jacobi 1837 → Bellman 1957 → Crandall-Lions 1983

理解一个理论的最好方式是追溯它的诞生过程。HJB 方程不是某个天才一次性发明的，而是跨越 150 年、由四代数学家接力完成的智力工程。

**Hamilton (1834)**：爱尔兰数学家 William Rowan Hamilton 在研究光学（费马原理）和经典力学（最小作用量原理）时，发现粒子轨迹可以用一个标量场（作用量函数 $S$）的梯度来描述。他建立了 Hamilton-Jacobi 方程的力学版本：

$$\frac{\partial S}{\partial t} + H\!\left(q, \frac{\partial S}{\partial q}\right) = 0$$

这里 $H(q,p)$ 是系统的 Hamiltonian，$S(q,t)$ 是 Hamilton 主函数（从初始构型到 $(q,t)$ 的最小作用量）。Hamilton 的洞察是：**粒子力学（ODE）可以用波动场论（PDE）来等价描述**——这比量子力学的"粒子-波对偶"早了近 100 年。

**Jacobi (1837)**：Carl Gustav Jacob Jacobi 将 Hamilton 的方法系统化，证明了如果能找到 Hamilton-Jacobi 方程的完全积分（含 $n$ 个独立常数的解），就能通过代数方法得到运动方程的通解，无需真正解微分方程。这就是经典力学中的 **Hamilton-Jacobi 理论**，它把求解力学问题转化为寻找正则变换。

> **本质洞察**：Hamilton-Jacobi 理论的深刻之处在于，它把**轨迹优化问题**（找最优路径）转化为**场方程问题**（解 PDE）。轨迹是 PDE 的特征线，而 PDE 的解给出了整个状态空间上的"最优代价地图"。这个从"线"到"场"的升维思想，正是 150 年后 HJB 方程与现代强化学习值函数的核心。

**Bellman (1957)**：Richard Bellman 在 RAND 公司研究多阶段决策问题时，独立发现了**动态规划原理**（最优策略的子策略仍然最优）。他的离散时间 Bellman 方程

$$V_k(x) = \min_u \{\ell(x,u) + V_{k+1}(f(x,u))\}$$

在取连续时间极限后，自然产生一个偏微分方程——这就是 HJB 方程。Bellman 本人并未严格处理解的正则性问题，他假设值函数足够光滑。

**Crandall-Lions (1983)**：Michael Crandall 和 Pierre-Louis Lions 在 *Transactions of the AMS* 上发表了划时代论文《Viscosity solutions of Hamilton-Jacobi equations》。他们认识到，对于大多数实际问题，HJB 的值函数**不可微**——特征线会相交、bang-bang 控制会制造拐点、状态约束会撕裂梯度。传统的弱解概念（Sobolev 解、分布解）对一阶非线性 PDE 不适用（无唯一性）。Crandall-Lions 的天才在于：**用光滑测试函数从上/下方触碰值函数，把不可微点的梯度条件"外包"给测试函数**。这个简洁的定义一举解决了存在性、唯一性、稳定性三大问题，开创了非线性 PDE 理论的新纪元。Lions 因此获得 1994 年 Fields 奖。

| 年代 | 人物 | 贡献 | 关键思想 |
|------|------|------|----------|
| 1834 | Hamilton | 力学中的 HJ 方程 | 粒子轨迹 = 标量场的梯度线 |
| 1837 | Jacobi | 完全积分理论 | PDE 解 → 力学通解 |
| 1957 | Bellman | 动态规划原理 | 最优子结构 + 连续极限 |
| 1983 | Crandall-Lions | 黏性解 | 测试函数外包不可微梯度 |
| 1991 | Barles-Souganidis | 数值收敛理论 | 单调+一致+稳定 → 收敛 |
| 1992 | Crandall-Ishii-Lions | User's Guide | 二阶黏性解完整理论 |
| 2005 | Mitchell-Tomlin | HJ Reachability | 安全可达集 = 零水平集 |
| 2021 | Bansal-Tomlin | DeepReach | 神经网络突破维数灾难 |

---

## §3.4.1 值函数与动态规划原理的连续版本 ⭐

### 动机

在专题 3.3 中，我们建立了离散时间动态规划的完整框架：值函数 $V_k(x)$ 满足 Bellman 方程，反向递推求解。但现实中的物理系统是连续时间的——电机转矩是连续施加的、关节角度是连续变化的、代价是连续累积的。我们需要将离散框架推广到连续时间。

这不是简单的"令 $\Delta t \to 0$"。连续时间带来了新的数学困难：积分代替求和、泛函分析代替线性代数、PDE 代替递推方程。但物理直觉是不变的：**最优策略的尾部仍然最优**。

### 严格定义

设状态 $x \in \mathbb{R}^n$，控制 $u \in U \subset \mathbb{R}^m$（$U$ 紧），动力学：

$$\dot{x}(s) = f(x(s), u(s)), \quad x(t) = x$$

在容许控制类 $\mathcal{U}_{t,T} = \{u:[t,T] \to U \text{ 可测}\}$ 上定义 **Bolza 型值函数**：

$$V(x,t) = \inf_{u(\cdot) \in \mathcal{U}_{t,T}} \left\{\int_t^T L(x(s),u(s))\,ds + \Phi(x(T))\right\}, \quad V(x,T) = \Phi(x)$$

这里 $L$ 是 running cost（即时代价率），$\Phi$ 是 terminal cost（终端代价）。

**技术假设**（保证问题良定义）：
- $f$ 连续，且对 $x$ 满足 Lipschitz 条件：$|f(x_1,u) - f(x_2,u)| \le L_f|x_1 - x_2|$，对所有 $u \in U$
- $L$ 连续有界下方
- $\Phi$ 连续
- $U$ 紧

在这些假设下，对任何容许控制 $u(\cdot)$，ODE 存在唯一解（Picard-Lindelof 定理）；$V$ 在紧集上有限且连续（Bardi-Capuzzo-Dolcetta 1997, Prop. III.1.6）。

> **跨领域类比**：值函数 $V(x,t)$ 类比于地形图上的"到山顶的最小体力消耗"。从任何位置 $x$ 出发，$V(x,t)$ 告诉你"从现在到终点，最少还需要花多少代价"。地形图是静态的（$V$ 不依赖你走哪条路），但最优路径取决于你的起点——这正是"场论"与"轨迹论"的区别。

### 动态规划原理（DPP）

**定理（Bellman 1957 的连续版本；Bardi-Capuzzo-Dolcetta 1997, Thm III.2.1）**。对任意 $(x,t)$ 与 $h \in [0, T-t]$：

$$\boxed{V(x,t) = \inf_{u \in \mathcal{U}_{t,t+h}} \left\{\int_t^{t+h} L(x(s),u(s))\,ds + V(x(t+h), t+h)\right\}}$$

**直觉**：不论你把问题切成几段，每一段内的策略都必须对"从该段末尾到终点"的子问题最优。这就是 Bellman 的 "principle of optimality"。

**证明思路**（两个方向）：

**方向一**（$\le$）：取 $\varepsilon$-最优控制 $u^\varepsilon(\cdot)$ 在 $[t,T]$ 上，则

$$V(x,t) + \varepsilon \ge \int_t^{t+h} L\,ds + \int_{t+h}^T L\,ds + \Phi(x(T))$$

后面两项 $\ge V(x(t+h), t+h)$（因为 $V$ 是 inf），所以

$$V(x,t) + \varepsilon \ge \int_t^{t+h} L\,ds + V(x(t+h), t+h) \ge \text{右边的 inf}$$

令 $\varepsilon \to 0$ 得 $V(x,t) \ge$ 右边。

**方向二**（$\ge$）：对任意 $u_1 \in \mathcal{U}_{t,t+h}$ 和任意 $\varepsilon > 0$，取 $u_2^\varepsilon \in \mathcal{U}_{t+h,T}$ 使得

$$V(x(t+h), t+h) + \varepsilon \ge \int_{t+h}^T L(x_2,u_2^\varepsilon)\,ds + \Phi(x_2(T))$$

拼接 $u_1$ 和 $u_2^\varepsilon$ 得 $[t,T]$ 上的容许控制，其总代价

$$\int_t^{t+h} L\,ds + \int_{t+h}^T L\,ds + \Phi \le \int_t^{t+h} L\,ds + V(x(t+h),t+h) + \varepsilon$$

对左边取 inf（在所有 $[t,T]$ 上的控制中）得 $V(x,t) \le$ 右边 $+ \varepsilon$。由于 $u_1$ 和 $\varepsilon$ 任意，得 $V(x,t) \le$ 右边。$\square$

### 与专题 3.3 离散 DP 的精确对应

回顾专题 3.3：离散时间 Bellman 方程为 $V_k(x) = \min_u\{\ell_k + V_{k+1}(x + \Delta f)\}$，其中 $\ell_k = \Delta L(x_k, u_k)$ 是一步代价。在那里我们用值迭代和策略迭代来求解这个递推方程，得到的是一张"离散状态空间上的最优代价表"。现在我们把时间步长 $\Delta \to 0$，$V_k(x) = V(x, t_k)$，离散 Bellman 方程在极限下变为 DPP。**两者只差一个 Taylor 展开**——这个展开将在 §3.4.2 中产生 HJB 方程。

让我们精确地写出这个对应关系。离散系统 $x_{k+1} = x_k + \Delta f(x_k, u_k)$ 的 Bellman 方程：

$$V_k(x) = \min_{u \in U}\{\Delta L(x, u) + V_{k+1}(x + \Delta f(x,u))\}$$

令 $t_k = t$，$t_{k+1} = t + \Delta$，$V_k(x) = V(x, t)$，$V_{k+1}(\cdot) = V(\cdot, t+\Delta)$：

$$V(x,t) = \min_{u \in U}\{\Delta L(x,u) + V(x + \Delta f(x,u), t+\Delta)\}$$

这正是 DPP 取 $h = \Delta$ 的版本。对右边做 Taylor 展开就得到 HJB——这个过程在下一节完成。

| 离散（专题 3.3） | 连续（本节） |
|-----------------|-------------|
| $V_k(x) = \min_u\{\ell_k + V_{k+1}(f_k(x,u))\}$ | $V(x,t) = \inf_u\{\int_t^{t+h} L\,ds + V(x(t+h),t+h)\}$ |
| 求和 $\sum$ | 积分 $\int$ |
| min（有限集） | inf（函数空间） |
| 递推（倒序 $k$） | PDE（反向时间） |
| $O(|X|^2 \cdot |U|)$ 每步 | 连续，需离散化 |

### 工程含义

DPP 是所有基于值函数的连续控制/RL 算法的地基：

1. **Actor-Critic 的 critic 更新**：TD 误差 $\delta = r + \gamma V(s') - V(s)$ 是 DPP 的一步残差
2. **iLQR/DDP 的 backward pass**：在每个时间步展开 DPP 到二阶
3. **MPC 的 terminal cost**：$V_f(x(T))$ 是 DPP 的截断——当 $V_f$ 近似真值函数时，有限时域 MPC 近似无限时域最优

> **反事实推理**：如果没有 DPP——即最优策略的子策略不一定最优——会怎样？那我们就无法把长时域问题分解为短时域子问题，必须在整个 $[0,T]$ 上同时搜索最优控制。对连续系统，这意味着在无限维函数空间中做全局优化，计算上完全不可行。DPP 的存在使得我们可以"从后往前"逐步求解，把无限维问题化为有限维（或至少是可处理的）。

### ⚠️ 常见陷阱

**陷阱 1：混淆 DPP 的时间方向**

- 错误做法：认为 DPP 是"从初始时刻向前推"
- 正确理解：DPP 是**后向**的——终端条件 $V(x,T) = \Phi(x)$ 已知，然后向 $t=0$ 反向递推
- 根本原因：代价是从 $t$ 到 $T$ 的积分，只有知道"后面的最优代价"才能做当前决策

**陷阱 2：inf 与 min 的区别**

- 错误做法：在连续控制问题中写 $\min_u$
- 正确理解：控制在紧集 $U$ 上取值时，min 存在（Weierstrass 定理）；但如果 $U$ 非紧（如 $U = \mathbb{R}^m$），需写 $\inf$ 并额外验证极值是否可达
- 工程影响：无界控制问题中，最优控制可能不存在（只有逼近序列）

### 练习

1. ⭐ 对一维系统 $\dot{x} = u$，$|u| \le 1$，running cost $L = 1$，terminal cost $\Phi = 0$，终端时间 $T$ 固定。写出 DPP 并直接求解 $V(x,t)$（提示：这是到原点的最小时间问题）。

2. ⭐⭐ 证明：如果 $f$ 和 $L$ 不显含 $t$，且 $T = \infty$（无限时域折扣问题），则值函数 $V(x)$ 不依赖时间。写出此时 DPP 的简化形式。

3. ⭐⭐ 解释为什么 MPC 的 terminal cost $V_f$ 选取很重要：如果 $V_f \equiv 0$（无终端代价），在什么条件下有限时域 MPC 仍能保证闭环稳定性？

---

## §3.4.2 HJB 方程的正式推导 ⭐

### 动机

DPP 给出了值函数的**积分方程**——对任何时间切片 $h$ 都成立。但积分方程不方便直接求解。如果值函数足够光滑（$C^1$），我们可以对 DPP 做 Taylor 展开、令 $h \to 0$，得到一个**微分方程**——这就是 HJB 方程。

### 从 DPP 到 HJB 的完整推导

**假设** $V \in C^1(\mathbb{R}^n \times [0,T])$（后面会看到这个假设通常不成立，但先在光滑框架下推导）。

**Step 1**：在 DPP 中取 $h$ 很小，将被积函数视为常数：

$$V(x,t) = \inf_{u \in U} \left\{h \cdot L(x,u) + V(x + hf(x,u), t+h) + o(h)\right\}$$

这里用了 $\int_t^{t+h} L(x(s),u(s))\,ds = hL(x,u) + o(h)$（因为 $x(s) = x + O(h)$）。

**Step 2**：对 $V(x + hf, t+h)$ 做 Taylor 展开：

$$V(x + hf, t+h) = V(x,t) + h\frac{\partial V}{\partial t}(x,t) + h\nabla_x V(x,t) \cdot f(x,u) + o(h)$$

**Step 3**：代入 DPP：

$$V(x,t) = \inf_{u \in U} \left\{hL(x,u) + V(x,t) + hV_t + h\nabla_x V \cdot f(x,u) + o(h)\right\}$$

**Step 4**：两边消去 $V(x,t)$，除以 $h$：

$$0 = \inf_{u \in U} \left\{L(x,u) + V_t(x,t) + \nabla_x V(x,t) \cdot f(x,u)\right\} + o(1)$$

**Step 5**：令 $h \to 0$（$o(1) \to 0$），并注意 $V_t$ 不依赖 $u$，可以提到 inf 外面：

$$\boxed{V_t(x,t) + \min_{u \in U} \left\{L(x,u) + \nabla_x V(x,t) \cdot f(x,u)\right\} = 0, \quad V(x,T) = \Phi(x)}$$

这就是 **Hamilton-Jacobi-Bellman 方程**。

### Hamiltonian 形式

定义 **最优控制 Hamiltonian**：

$$\mathcal{H}(x,p) = \min_{u \in U}\{L(x,u) + p \cdot f(x,u)\}$$

其中 $p = \nabla_x V$ 扮演"广义动量"的角色。HJB 方程紧凑地写为：

$$V_t + \mathcal{H}(x, \nabla_x V) = 0$$

**符号约定对照**：本专题使用极小化约定（$\mathcal{H} = \min_u\{L + p \cdot f\}$），专题 3.2（PMP）使用 Pontryagin 原始的极大化约定（$H_{\max} = \lambda^\top f - \lambda_0 L$）。两者的关系是 $\mathcal{H} = -H_{\max}$（取 $\lambda_0 = 1$），即 $H_{\max}(x,p) = \max_u\{p \cdot f - L\} = -\mathcal{H}(x,p)$。在极大化约定下 HJB 变为 $-V_t + H_{\max}(x, \nabla_x V) = 0$。专题 3.3（DP）的离散 Bellman 算子 $\mathcal{T}V = \min_u\{L + \gamma V'\}$ 也是极小化约定，与本专题一致。**三个专题的结论完全等价，只需注意 $H$ 的符号**。

> **共态与值函数梯度的精确关系**（常见出错点）：
> - 极小化约定（本专题）：共态 $p = \nabla_x V$，LQR 最优控制 $u^* = -R^{-1}B^\top p = -R^{-1}B^\top \nabla_x V$
> - 极大化约定（专题 3.2）：共态 $\lambda = -\nabla_x V$，LQR 最优控制 $u^* = R^{-1}B^\top\lambda = R^{-1}B^\top(-\nabla_x V) = -R^{-1}B^\top\nabla_x V$
> - 两者给出**完全相同的控制律**：$u^* = -R^{-1}B^\top\nabla_x V = -R^{-1}B^\top Px$（负反馈）
>
> 转换规则：$\lambda_{\text{PMP(max)}} = -p_{\text{HJB(min)}} = -\nabla_x V$。切换文献时只需记住"极大化共态 = 负的值函数梯度"。

> **类比**：HJB 方程之于最优控制，就像 Navier-Stokes 方程之于流体力学。两者都是描述某个"场"（值函数场/速度场）的偏微分方程，都很难求解，但一旦解出来就给出了系统的全部信息。区别在于：Navier-Stokes 是二阶半线性的，HJB 是一阶完全非线性的——后者的非线性来自 $\min_u$ 操作。

### 无限时域稳态 HJB

当 $T = \infty$、$L$ 和 $f$ 不显含时间时，值函数 $V(x) = \inf_u \int_0^\infty L\,ds$ 仅依赖状态 $x$，$V_t = 0$：

$$0 = \min_{u \in U}\{L(x,u) + \nabla V(x) \cdot f(x,u)\}$$

**折扣版**（折扣率 $\beta > 0$）：$V(x) = \inf_u \int_0^\infty e^{-\beta s} L\,ds$ 满足：

$$\beta V(x) = \min_{u \in U}\{L(x,u) + \nabla V(x) \cdot f(x,u)\}$$

这对应离散 Bellman 方程 $V = \min_u\{\ell + \gamma V'\}$ 在 $\gamma = e^{-\beta\Delta} \approx 1 - \beta\Delta$ 极限下的形式。

> **本质洞察**：折扣稳态 HJB 的左边 $\beta V$ 可以理解为"持有值函数的利息成本"——你每时刻都在"支付" $\beta V$ 的代价来维持当前状态的价值。右边是"当前即时收益 + 值函数增长率"。平衡条件说的是：**值函数的利息成本 = 最优策略下的即时代价 + 值函数的改善速率**。这与金融中资产定价的 no-arbitrage 条件完全同构。

### 最优反馈律

如果 $\min_u$ 的极值点唯一，则 HJB 方程隐含了最优反馈控制律：

$$u^*(x,t) = \arg\min_{u \in U}\{L(x,u) + \nabla_x V(x,t) \cdot f(x,u)\}$$

这是一个**状态反馈**策略——给定当前状态 $x$ 和时间 $t$，直接计算最优控制。这比 PMP（需要求解两点边值问题）方便得多，但代价是需要知道整个状态空间上的 $V$。

### 与 RL 的连接

**连续时间 Q-learning（Doya 2000）**：RL 中 critic 网络 $V_\theta(x)$ 的训练损失是折扣 HJB 的残差：

$$\mathcal{L}_{\text{critic}} = \frac{1}{2}\left(\beta V_\theta(x) - L(x,u) - \nabla V_\theta(x) \cdot f(x,u)\right)^2$$

最小化这个损失就是在求解 HJB 方程——这是 PINN（Physics-Informed Neural Networks）方法应用于值函数学习的理论基础。

### ⚠️ 常见陷阱

**陷阱 1：HJB 的时间方向**

- 错误做法：将 HJB 视为初值问题（给定 $V(x,0)$，向前积分到 $T$）
- 正确做法：HJB 是**终值问题**（给定 $V(x,T) = \Phi(x)$，向后积分到 $t=0$）
- 根本原因：DPP 的信息流是"从未来到过去"——先知道终端代价，才能决定当前最优决策
- 工程影响：数值求解时必须反向时间积分，或等价地令 $\tau = T-t$ 变换为正向

**陷阱 2：$C^1$ 假设的本质性**

- 错误做法：认为 HJB 推导是无条件成立的
- 正确做法：推导中用了 Taylor 展开，要求 $V \in C^1$。如果 $V$ 不可微（§3.4.5 会看到这是常态），HJB 方程的经典形式无意义——需要黏性解理论
- 这不是技术细节，而是本专题后半段（§3.4.5-§3.4.6）的核心动机

**陷阱 3：$\min$ 与 $\inf$ 的混用**

- 当 $U$ 紧凑且 $L + p \cdot f$ 对 $u$ 连续时，$\min$ 存在（Weierstrass 定理），可以安全使用 $\min$
- 当 $U$ 非紧凑时（如无约束 LQR），需验证极值是否可达
- LQR 情况：$u^* = -R^{-1}B^\top p$ 是无约束极小（因为 $R \succ 0$ 保证凸性），极值在 $\mathbb{R}^m$ 上可达

### 练习

1. ⭐ 对一维系统 $\dot{x} = u$，$L = \frac{1}{2}u^2$，$\Phi(x) = \frac{1}{2}x^2$，直接求解 HJB 方程。提示：猜测 $V(x,t) = \frac{1}{2}P(t)x^2$，代入 HJB 得 $P(t)$ 的 ODE。

2. ⭐⭐ 推导无限时域折扣 HJB：从 DPP $V(x) = \inf_u\{\int_0^h e^{-\beta s}L\,ds + e^{-\beta h}V(x(h))\}$ 出发，对 $e^{-\beta h} \approx 1-\beta h$ 展开得到 $\beta V = \min_u\{L + \nabla V \cdot f\}$。

3. ⭐⭐⭐ 对控制仿射系统 $\dot{x} = f_0(x) + G(x)u$，$L = q(x) + \frac{1}{2}u^\top R u$（$R \succ 0$），显式写出最优控制 $u^* = -R^{-1}G(x)^\top \nabla V$ 并将其代回 HJB，得到关于 $V$ 的闭式 PDE。

---

## §3.4.3 验证定理——HJB 是充分条件 ⭐⭐

### 动机

PMP（专题 3.2）给出的是**必要条件**——满足 PMP 的轨迹不一定最优（可能是鞍点或极大）。而 HJB 的验证定理说的是：**如果你能找到一个 $C^1$ 函数 $W$ 满足 HJB 方程和终端条件，那么 $W$ 就是值函数，而且相应的反馈律全局最优。** 这是一个**充分条件**，比 PMP 更强。

> **反事实推理**：如果 HJB 只给出必要条件（而非充分条件）会怎样？那我们即使解出了 HJB 方程、找到了 $W$，也无法确定 $W = V^*$——还需要额外验证。验证定理免除了这个后顾之忧：只要 $W$ 满足 HJB + 终端条件 + 反馈律的 ODE 有解，就铁定是最优的。

### 定理陈述

**定理（Fleming-Rishel 1975；Fleming-Soner 2006, Thm IV.3.1）**。设 $W \in C^1([t_0,T] \times \mathbb{R}^n)$ 满足：

1. HJB 方程：$W_t + \min_{u \in U}\{L(x,u) + \nabla_x W \cdot f(x,u)\} = 0$
2. 终端条件：$W(x,T) = \Phi(x)$
3. 对每个 $(x,t)$，存在 $u^*(x,t) \in \arg\min_{u \in U}\{L + \nabla W \cdot f\}$
4. 闭环 ODE $\dot{x} = f(x, u^*(x,t))$ 有容许解 $x^*(\cdot)$

则 $u^*$ **全局最优**，且 $W \equiv V^*$（即 $W$ 就是值函数）。

### 完整证明

证明的核心思想非常优美：**利用 HJB 方程的不等式方向证明任何容许控制不比 $u^*$ 好**。

**Step 1**：对任意容许控制 $u(\cdot)$ 及其对应轨迹 $x(\cdot)$，考虑 $W$ 沿轨迹的变化率：

$$\frac{d}{ds}W(x(s),s) = W_t(x(s),s) + \nabla_x W(x(s),s) \cdot \dot{x}(s) = W_t + \nabla_x W \cdot f(x,u)$$

**Step 2**：由 HJB 方程，对**极小化的** $u^*$ 有 $W_t + L(x,u^*) + \nabla W \cdot f(x,u^*) = 0$。对**任意** $u \in U$：

$$W_t + L(x,u) + \nabla W \cdot f(x,u) \ge W_t + \min_{u'}\{L(x,u') + \nabla W \cdot f(x,u')\} = 0$$

因此：

$$\frac{d}{ds}W(x(s),s) = W_t + \nabla W \cdot f(x,u) \ge -L(x(s),u(s))$$

**Step 3**：从 $t$ 到 $T$ 积分：

$$W(x(T),T) - W(x,t) \ge -\int_t^T L(x(s),u(s))\,ds$$

即：

$$\int_t^T L(x(s),u(s))\,ds + \Phi(x(T)) \ge W(x,t)$$

（用了 $W(x(T),T) = \Phi(x(T))$）

**Step 4**：对所有容许 $u$ 取 inf：

$$V^*(x,t) = \inf_u \left\{\int_t^T L\,ds + \Phi(x(T))\right\} \ge W(x,t)$$

**Step 5**：对最优控制 $u^*$，Step 2 中的不等式变为等式（因为 $u^*$ 实现了 min），因此 Step 3 中也是等式：

$$\int_t^T L(x^*(s),u^*(x^*(s),s))\,ds + \Phi(x^*(T)) = W(x,t)$$

这说明 $V^*(x,t) \le W(x,t)$。

**结论**：$V^*(x,t) = W(x,t)$，且 $u^*$ 全局最优。$\square$

### HJB（充分）vs PMP（必要）的对偶

这是本专题与专题 3.2 的关键对偶关系：

| 方面 | **PMP（专题 3.2）** | **HJB（本专题）** |
|------|---------------------|-------------------|
| 条件类型 | 必要条件 | 充分条件（经典 $C^1$ 情形） |
| 对象 | 单条最优轨迹 $(x^*, \lambda^*, u^*)$ | 全状态空间值函数 $V(x,t)$ |
| 维数 | $2n$ 维 ODE（两点边值问题） | $(n+1)$ 维 PDE |
| 信息量 | 给出一条最优路径 | 给出从任何初始状态出发的最优策略 |
| 数值方法 | Shooting / 间接法 / CasADi | 网格 / WENO / PINN |
| 维数灾难 | 轻（可处理高维） | 重（$d \gtrsim 6$ 经典方法失效） |
| 唯一性 | 多个候选需额外筛选 | 黏性解唯一 |
| 工程用途 | 单次轨迹优化 | 离线求值函数 + 在线查表反馈 |

> **本质洞察**：PMP 像 GPS 导航——只告诉你从 A 到 B 的最佳路线。HJB 像完整地图——告诉你从任何点出发的最佳路线。GPS 便宜但每次换起点要重新规划；完整地图贵但一次计算永久使用。这就是**轨迹优化 vs 策略优化**的工程权衡。

### 工程含义：从 HJB 不等式到 Lyapunov/CBF

验证定理有一个重要推论：如果 $W$ 不完全满足 HJB 等式，而只满足**不等式**：

$$W_t + \min_u\{L + \nabla W \cdot f\} \ge 0$$

则 $W \le V^*$，即 $W$ 是值函数的**下界**。这个观察连接了三个看似不同的工程框架：

- **Lyapunov 函数**：$\dot{W} \le -\alpha(W)$ 相当于 $W$ 满足某种 HJB 次解条件
- **Control Barrier Function (CBF)**：$\dot{h} + \alpha(h) \ge 0$ 保证安全集正不变
- **SOS (Sum-of-Squares) 控制**：用多项式 $W$ 满足 HJB 不等式，通过 SDP 验证

### ⚠️ 常见陷阱

**陷阱 1：验证定理需要 $C^1$ 正则性**

- 问题：大多数 HJB 没有 $C^1$ 解（§3.4.5），所以验证定理"字面意义上"不适用
- 解决：黏性解理论提供了推广版验证定理——值函数是 HJB 的唯一黏性解（§3.4.6）

**陷阱 2：闭环 ODE 的解必须存在**

- 错误做法：找到 $u^*(x,t)$ 后不验证闭环系统是否有解
- 正确做法：需要 $f(x,u^*(x,t))$ 至少是 $x$ 的 Lipschitz 函数
- 实际问题：当 $u^*$ 不连续（bang-bang 控制）时，闭环可能不是经典 ODE 解——需要 Filippov/Caratheodory 解

### 练习

1. ⭐ 对一维 LQR（$\dot{x} = ax + bu$，$L = \frac{1}{2}(qx^2 + ru^2)$），构造 $W(x,t) = \frac{1}{2}P(t)x^2$ 并用验证定理证明其为最优值函数。

2. ⭐⭐ 证明：如果 $W$ 满足 HJB 不等式 $W_t + \min_u\{L + \nabla W \cdot f\} \ge 0$ 且 $W(x,T) \le \Phi(x)$，则 $W(x,t) \le V^*(x,t)$。

3. ⭐⭐⭐ 思考：验证定理为什么不需要凸性假设？（提示：PMP 的充分条件——Mangasarian 条件——需要 Hamiltonian 关于 $(x,u)$ 联合凸）

---

## §3.4.4 HJB 与 PMP 的精确关系——特征线与对偶 ⭐⭐

### 动机

专题 3.2（PMP）和本专题（HJB）是最优控制的"两面"。一个自然的问题是：它们之间的精确数学关系是什么？答案深刻而优美：**PMP 的共态 $\lambda(t)$ 就是值函数沿最优轨迹的梯度，Hamilton 方程组就是 HJB 方程的特征线系统。**

**符号约定提醒**：专题 3.2 使用 Pontryagin 的极大化约定（$H_{\max} = \lambda^\top f - \lambda_0 L$，$u^* = \arg\max H_{\max}$），本专题使用极小化约定（$\mathcal{H} = \min_u\{L + p \cdot f\}$）。两者的关系是 $\mathcal{H} = -H_{\max}$（取 $\lambda_0 = 1$）。下面的推导在极小化约定下进行，但结论对两种约定等价——只需注意共态方程的符号一致性。

这个关系不仅是美学上的满足，更有实际工程价值：理解了对偶，你就知道 iLQR 的 backward pass 为什么计算 $V_x, V_{xx}$（HJB 的局部展开），也知道间接法 shooting 为什么等价于沿 HJB 特征线积分。

### 核心恒等式

**定理**。设 $V \in C^2$，$x^*(\cdot)$ 是最优轨迹，$u^*(\cdot)$ 是最优控制。定义共态 $\lambda(t) = \nabla_x V(x^*(t), t)$。则 $\lambda(t)$ 满足 PMP 的伴随方程（极小化约定下）：

$$\dot{\lambda}(t) = -\frac{\partial}{\partial x}\left[L(x^*,u^*) + \lambda \cdot f(x^*,u^*)\right] = -L_x - f_x^\top \lambda$$

$$\boxed{\lambda(t) = \nabla_x V(x^*(t), t)}$$

**完整推导**：

**Step 1**：令 $\lambda(t) = V_x(x^*(t), t)$（$V_x$ 表示 $\nabla_x V$）。沿最优轨迹求时间导数：

$$\dot{\lambda} = V_{tx}(x^*,t) + V_{xx}(x^*,t) \cdot \dot{x}^* = V_{tx} + V_{xx} f(x^*,u^*)$$

**Step 2**：对 HJB 方程 $V_t + \mathcal{H}(x, V_x) = 0$ 关于 $x$ 求偏导（$\mathcal{H}(x,p) = \min_u\{L + p \cdot f\}$）：

$$V_{tx} + \frac{\partial \mathcal{H}}{\partial x} + \frac{\partial \mathcal{H}}{\partial p} \cdot V_{xx} = 0$$

其中，在最优控制 $u^*$ 处（由包络定理）：
- $\frac{\partial \mathcal{H}}{\partial p}\big|_{u^*} = f(x^*, u^*)$
- $\frac{\partial \mathcal{H}}{\partial x}\big|_{u^*} = L_x(x^*,u^*) + \lambda \cdot f_x(x^*,u^*)$

**Step 3**：将 Step 2 代入 Step 1：

$$\dot{\lambda} = V_{tx} + V_{xx} f = -\frac{\partial \mathcal{H}}{\partial x} - \frac{\partial \mathcal{H}}{\partial p} V_{xx} + V_{xx} f = -\frac{\partial \mathcal{H}}{\partial x}$$

即：

$$\dot{\lambda} = -L_x(x^*,u^*) - f_x(x^*,u^*)^\top \lambda$$

这**正是 PMP 的伴随方程**！

**Step 4**：由 HJB 的 $\min_u$ 条件，$u^*$ 实现 $L + \lambda \cdot f$ 的极小。注意本专题使用极小化约定（$\mathcal{H} = \min_u\{L + p \cdot f\}$），而专题 3.2 使用 Pontryagin 原始的极大化约定（$H_{\max} = \lambda^\top f - \lambda_0 L$，$u^* = \arg\max H_{\max}$）。两者是**同一条件的两种等价记号**——极小化 $L + \lambda \cdot f$ 等价于极大化 $\lambda^\top f - L$。因此 HJB 的极小化条件**正是 PMP 极大化条件的对偶形式**。

### 特征线方法视角

一阶 PDE $V_t + \mathcal{H}(x, \nabla V) = 0$ 的**特征线 ODE 系统**（经典 PDE 理论）为：

$$\dot{x} = \frac{\partial \mathcal{H}}{\partial p}(x,p), \quad \dot{p} = -\frac{\partial \mathcal{H}}{\partial x}(x,p), \quad \dot{z} = p \cdot \frac{\partial \mathcal{H}}{\partial p} - \mathcal{H}$$

其中 $z = V$ 沿特征线。前两个方程正是 **Hamilton 典则方程**——即 PMP 的状态-共态系统！

> **本质洞察**：**PMP 是 HJB 的特征线，HJB 是 PMP 的"包络面"。** 这个对偶如同几何光学中的费马原理（光线 = 特征线）与波动方程（波前 = PDE 解的等值面）之间的关系。光线求解容易（ODE），但只给单条路径；波前求解困难（PDE），但给出整个空间的信息。

### 特征线相交——经典解崩溃的几何原因

考虑从不同初始状态出发的最优轨迹（特征线）。当两条特征线在某点相遇时，$V$ 的梯度在该点有两个不同的值——这意味着 $V$ 在该点**不可微**。

几何上：值函数的等值面是特征线的"包络面"。当特征线相交时，包络面产生**折叠**或**尖点**——类似于光学中的焦散面（caustic）。

物理上：两个不同的初始状态经过不同的最优轨迹到达同一个状态，说明存在"多个最优路径"——值函数在该点是连续的但不可微的（梯度从左右极限不同）。

这正是下一节（§3.4.5）要深入讨论的问题，也是引入黏性解的直接动机。

### 工程含义

1. **间接法 shooting**（CasADi、GPOPS）：本质是在求 HJB 的特征线——给定终端条件 $\lambda(T) = \nabla\Phi(x^*(T))$，反向积分 Hamilton 方程
2. **iLQR/DDP 的 backward pass**：计算 $V_x, V_{xx}$ 就是在特征线附近做二阶展开——Riccati 方程是 HJB 的二阶近似
3. **CBF-QP**：$\min_u\{\|u-u_{\text{nom}}\|^2 : \nabla h \cdot f + \alpha(h) \ge 0\}$ 可视为 HJB 极小化条件的单点投影

### ⚠️ 常见陷阱

**陷阱 1：$V \in C^2$ 假设**

- 对偶关系的推导用了 $V_{xx}$（二阶导数），需要 $V \in C^2$
- 实际上 $V$ 通常不是 $C^1$（更别说 $C^2$），此时 $\lambda = \nabla V$ 不存在
- 解决：黏性解框架下，$\lambda$ 对应 $V$ 的超微分/次微分中的元素

**陷阱 2：PMP 共态的非唯一性**

- 当 $V$ 不可微时，$\nabla V$ 不唯一（有多个次梯度方向）
- 对应 PMP：同一状态可能有多条满足必要条件的轨迹——PMP 不保证唯一性

### 练习

1. ⭐ 对 LQR 系统，验证 $\lambda(t) = P(t)x^*(t)$（即 Riccati 矩阵给出共态与状态的线性关系）。

2. ⭐⭐ 对一维问题 $\dot{x} = u$，$|u| \le 1$，$L = 0$，$\Phi(x) = |x|$（最小时间到原点），画出特征线并指出在哪里相交。

3. ⭐⭐⭐ 证明：Hamilton 典则方程 $\dot{x} = \mathcal{H}_p$，$\dot{p} = -\mathcal{H}_x$ 保持 Hamiltonian $\mathcal{H}$ 沿轨迹为常数（能量守恒），并解释其对应于 HJB 中 $V_t = -\mathcal{H}$ 的物理含义。

---

## §3.4.5 经典解为何不存在——HJB 的核心病理 ⭐⭐

### 动机

到目前为止，我们在 §3.4.2-3.4.4 中一直假设 $V \in C^1$（甚至 $C^2$）。但现在必须面对一个残酷的现实：**对绝大多数有实际意义的最优控制问题，值函数 $V$ 都不是 $C^1$ 的。** 这不是技术上的小毛病，而是 HJB 方程的根本性困难——它使得 §3.4.2 的推导在严格意义上**失效**，验证定理也无法直接使用。

如果不解决这个问题，整个 HJB 理论只是漂亮的形式主义，无法应用于真实问题。这正是 Crandall-Lions 1983 发明黏性解的动机。

### 为什么经典解不存在？

HJB 方程 $V_t + \mathcal{H}(x, \nabla V) = 0$ 是**一阶完全非线性** PDE，与双曲守恒律（如 Burgers 方程）有相同的病理：**特征线会相交**。相交处 $\nabla V$ 多值，经典解不存在。

下面通过四个递进的例子来理解这个困难。

### 例 1：Eikonal 方程 $|\nabla V| = 1$ ⭐⭐

**问题**：在有界区域 $\Omega$ 中，求到边界 $\partial\Omega$ 的距离函数：

$$|\nabla V(x)| = 1, \quad x \in \Omega; \qquad V|_{\partial\Omega} = 0$$

**控制论解释**：这是到边界的最小时间问题（$\dot{x} = u$，$|u| \le 1$，$L = 1$），值函数 $V(x) = \text{dist}(x, \partial\Omega)$。

**为什么没有 $C^1$ 解？** 考虑简单情况 $\Omega = (-1,1) \subset \mathbb{R}$：

- 距离函数 $V(x) = \min(|x-(-1)|, |x-1|) = \min(x+1, 1-x)$
- 这是一个"帐篷函数"，在 $x = 0$ 处有尖角：$V'(0^-) = 1$，$V'(0^+) = -1$
- 在 $x = 0$ 处 $V$ 不可微！

高维情况：$V(x) = \text{dist}(x, \partial\Omega)$ 在**中轴**（medial axis，即到 $\partial\Omega$ 最近点不唯一的集合）上不可微。中轴是一个余维 1 的集合（在 2D 中是曲线，3D 中是曲面）。

> **类比**：想象在一个房间里每个点到最近墙壁的距离。距离场在"等距于两面墙"的位置产生棱线（ridge）——这些棱线就是中轴，距离函数在棱线上不可微。机器人路径规划中的"Voronoi 图"正是中轴的离散版本。

**关键观察**：$-V(x) = -\text{dist}(x, \partial\Omega)$ 也几乎处处满足 $|\nabla(-V)| = 1$（a.e.），但它**不是**正确的物理解。我们需要一种弱解概念来**区分** $V$ 和 $-V$——黏性解正是能做到这一点的工具。

### 例 2：双积分器最小时间问题 ⭐⭐

**问题**：$\ddot{q} = u$，$|u| \le 1$，到原点 $(q, \dot{q}) = (0,0)$ 的最小时间。

等价为状态空间 $(x_1, x_2) = (q, \dot{q})$：$\dot{x}_1 = x_2$，$\dot{x}_2 = u$，$|u| \le 1$。

**PMP 分析**（专题 3.2 的典型例题）：共态满足 $\dot{\lambda}_1 = 0$，$\dot{\lambda}_2 = -\lambda_1$。Hamiltonian 极大化条件（专题 3.2 的极大化约定下等价于极小化 $L + \lambda \cdot f$）给出：

$$u^* = -\text{sgn}(\lambda_2(t)) = -\text{sgn}(\lambda_{2}(0) - \lambda_1 t)$$

这是 **bang-bang 控制**：$u^*$ 只取 $\pm 1$，最多切换一次。

**切换曲线**：$u^*$ 从 $+1$ 切换到 $-1$（或反之）的状态构成曲线

$$\Sigma = \{(x_1, x_2) : x_1 + \frac{1}{2}x_2|x_2| = 0\}$$

值函数（最小时间）$T(x_1, x_2)$ 在切换曲线 $\Sigma$ 上**连续但不可微**：来自两侧的特征线以不同角度汇聚到 $\Sigma$，法向导数发生跳跃。

### 例 3：状态约束边界 ⭐⭐⭐

**问题**：机器人关节角 $q \in [-\pi, \pi]$（状态约束），最优控制问题限制在 $\bar{K} = [-\pi, \pi]$ 内。

当最优轨迹"撞到"约束边界 $\partial K$ 时，值函数在边界上不可微——因为边界处的"有效控制集"被约束切割（只能选择不违反约束的控制）。

这需要 **state-constrained viscosity solution**（Soner 1986）：内部用标准黏性解定义，边界上只要求**上解条件**（放宽下解条件）。

### 例 4：非光滑终端代价 ⭐⭐

即使动力学和 running cost 都光滑，如果终端代价 $\Phi$ 不光滑，值函数也会继承不光滑性。

**例子**：$\dot{x} = xu$，$|u| \le 1$，$\Phi(x) = -|x|$。值函数 $V(x,t) = -|x|e^{T-t}$，在 $x = 0$ 处不可微。

终端不光滑性通过特征线向内传播——就像波前的折叠一样。

### 总结：经典解失败的系统性原因

| 原因 | 物理机制 | 典型例子 | 数学表现 |
|------|---------|---------|---------|
| 特征线相交 | 多条最优路径到达同一状态 | Eikonal 中轴 | $\nabla V$ 多值 |
| Bang-bang 控制 | 最优控制不连续切换 | 时间最优 | 切换面上不可微 |
| 状态约束 | 控制集被约束截断 | 关节限位 | 边界处梯度跳跃 |
| 非光滑数据 | 终端/边界条件本身不光滑 | $\Phi = |x|$ | 不光滑性传播 |

**结论**：对有界 Lipschitz 数据，经典 $C^1$ 解**一般不存在**，但值函数 $V$ 仍然是 Lipschitz 连续的（有界梯度但不处处可微）。我们需要一种弱解概念：

1. 放弃处处可微的要求
2. 保留**唯一性**（从无穷多个 a.e. 解中选出物理正确的那一个）
3. 保留 $V = V^*$ 的等价关系（弱解就是值函数）

> **反事实推理**：如果我们不引入黏性解，而是用传统的弱解概念（如 Sobolev 解或分布解），会怎样？对于线性 PDE（如拉普拉斯方程），Sobolev 解理论完美工作。但对一阶非线性 PDE，Sobolev 解**不唯一**——例如 Eikonal 方程有无穷多个 Lipschitz 函数满足 $|\nabla V| = 1$ a.e.（想象在中轴两侧选不同符号）。黏性解的核心贡献正是在这些弱解中挑选出唯一正确的那个。

### 消失粘性法——黏性解名称的来源

在引入严格定义之前，先理解 Crandall-Lions 定义的物理直觉。考虑 HJB 方程的**正则化版本**（加入人工粘性 $\varepsilon > 0$）：

$$V_t^\varepsilon + \mathcal{H}(x, \nabla V^\varepsilon) = \varepsilon \Delta V^\varepsilon$$

右边的 Laplacian 项使方程变为二阶抛物 PDE，有**唯一光滑解** $V^\varepsilon$。当 $\varepsilon \to 0$ 时，$V^\varepsilon$ 收敛到某个极限函数 $V$——这就是"黏性解"（viscosity solution）。

名称来源：$\varepsilon \Delta V$ 类比于流体力学中的粘性项（viscosity）。加粘性后解光滑（层流），去粘性后可能出现不光滑（激波/间断）。"消失粘性"选出了物理正确的弱解——在流体中是满足熵条件的解，在最优控制中是值函数。

但 Crandall-Lions 的天才在于：他们给出了一个**不依赖 $\varepsilon$** 的等价定义——纯粹用测试函数来刻画，不需要真正计算极限。

### ⚠️ 常见陷阱

**陷阱 1：认为"不可微"意味着"不连续"**

- 值函数通常是 **Lipschitz 连续**但不可微。Lipschitz = 有界梯度，只是梯度可能跳跃
- 不连续的值函数只出现在非常病态的问题中

**陷阱 2：用标准 FEM 求解 HJB**

- FEM（有限元）基于弱形式/变分原理，适用于二阶椭圆/抛物 PDE
- HJB 是一阶非线性 PDE，标准 Galerkin 方法会产生振荡（不满足单调性）
- 正确方法：upwind 差分、WENO、ENO、半拉格朗日（满足 Barles-Souganidis 条件）

### 练习

1. ⭐ 画出一维 Eikonal 方程 $|V'(x)| = 1$，$V(0) = V(1) = 0$ 在 $[0,1]$ 上的所有 Lipschitz 解。说明为什么只有 $V(x) = \min(x, 1-x)$ 是黏性解。

2. ⭐⭐ 对双积分器最小时间问题，在切换曲线 $\Sigma$ 上取一点，计算 $V$ 的左右方向导数，验证其不相等。

3. ⭐⭐⭐ 对 Burgers 方程 $u_t + uu_x = 0$ 和 HJB 方程 $V_t + H(V_x) = 0$（$H$ 凸），解释两者特征线相交时的物理区别：Burgers 形成激波（间断解），HJB 形成"拐角"（连续不可微解）。

---

## §3.4.6 黏性解的严格定义与核心理论 ⭐⭐⭐

### 动机

§3.4.5 告诉我们经典解不存在。现在需要一个弱解概念：(1) 足够弱，使解存在；(2) 足够强，保证唯一性；(3) 与值函数等价。Crandall-Lions 1983 的黏性解完美满足这三个要求。

### 黏性解的严格定义

考虑一般的一阶 PDE：

$$F(x, V(x), \nabla V(x)) = 0, \quad x \in \Omega \subset \mathbb{R}^n$$

（对 HJB，取 $F(x, r, p) = r_t + \mathcal{H}(x, p_x)$ 其中 $(r_t, p_x)$ 分别对应时间和空间分量）

**定义（Crandall-Lions 1983, *Trans. AMS* 277:1-42）**。设 $V \in C(\Omega)$（连续函数）：

**(i) 黏性次解（viscosity subsolution）**：$\forall \varphi \in C^1(\Omega)$，若 $V - \varphi$ 在 $x_0 \in \Omega$ 取**局部最大值**，则：

$$F(x_0, V(x_0), \nabla\varphi(x_0)) \le 0$$

**(ii) 黏性上解（viscosity supersolution）**：$\forall \varphi \in C^1(\Omega)$，若 $V - \varphi$ 在 $x_0 \in \Omega$ 取**局部最小值**，则：

$$F(x_0, V(x_0), \nabla\varphi(x_0)) \ge 0$$

**(iii) 黏性解（viscosity solution）**：同时为次解和上解。

### 几何直觉——为什么这个定义有效？

**关键洞察**：在 $V$ 不可微的点 $x_0$，我们无法计算 $\nabla V(x_0)$。但我们可以找一个 $C^1$ 函数 $\varphi$ 从**上方**或**下方**"触碰" $V$：

- **从上方触碰**：$V - \varphi \le V(x_0) - \varphi(x_0)$ 在 $x_0$ 附近。这意味着 $\varphi$ 在 $x_0$ 处"从上方贴住" $V$，所以 $\nabla\varphi(x_0)$ 是 $V$ 在 $x_0$ 处的一个"超梯度"（supergradient）。
- **从下方触碰**：$V - \varphi \ge V(x_0) - \varphi(x_0)$ 在 $x_0$ 附近。$\nabla\varphi(x_0)$ 是"次梯度"（subgradient）。

黏性解定义说的是：**对所有可能的超梯度，PDE 的左边 $\le 0$；对所有可能的次梯度，PDE 的左边 $\ge 0$。** 在 $V$ 可微的点，超梯度 = 次梯度 = $\nabla V$，两个不等式合起来就是 $F = 0$——恢复经典解。

> **跨领域类比**：黏性解的测试函数方法类比于分布理论中的"试探函数"。在分布理论中，$\delta$-函数没有点值，但可以通过对光滑函数的积分来定义：$\langle\delta, \varphi\rangle = \varphi(0)$。同样，黏性解在不可微点没有梯度，但可以通过与光滑测试函数的"接触"来定义。两种理论都把"坏对象"（不可微函数/$\delta$-函数）的性质"外包"给"好对象"（光滑测试函数/试探函数）。区别在于：分布理论用积分、黏性解理论用极值——这反映了一阶非线性 PDE 的本质不同于线性 PDE。

### 等价刻画：超微分与次微分

**定义**。$V$ 在 $x_0$ 的**超微分**（superdifferential）：

$$D^+ V(x_0) = \{p \in \mathbb{R}^n : \limsup_{y \to x_0} \frac{V(y) - V(x_0) - p \cdot (y-x_0)}{|y-x_0|} \le 0\}$$

**次微分**（subdifferential）：

$$D^- V(x_0) = \{p \in \mathbb{R}^n : \liminf_{y \to x_0} \frac{V(y) - V(x_0) - p \cdot (y-x_0)}{|y-x_0|} \ge 0\}$$

**等价定义**：
- $V$ 是次解 $\iff$ 对所有 $x_0$、所有 $p \in D^+ V(x_0)$：$F(x_0, V(x_0), p) \le 0$
- $V$ 是上解 $\iff$ 对所有 $x_0$、所有 $p \in D^- V(x_0)$：$F(x_0, V(x_0), p) \ge 0$

**几何解释**：$D^+ V(x_0)$ 是所有"从上方贴合 $V$ 的超平面"的斜率集合（一个凸集）。$D^- V(x_0)$ 是所有"从下方贴合的超平面"的斜率集合。当 $V$ 在 $x_0$ 可微时，$D^+ V = D^- V = \{\nabla V(x_0)\}$。

### 一维例子：验证黏性解

**例子**：Eikonal 方程 $|V'(x)| = 1$，$x \in (0,1)$，$V(0) = V(1) = 0$。

候选解 1：$V(x) = \min(x, 1-x)$（帐篷函数，顶点在 $x=1/2$）

验证 $x = 1/2$ 处（唯一不可微点）：
- $D^+ V(1/2) = [-1, 1]$（所有斜率在 $-1$ 到 $1$ 之间的直线都从上方触碰）
- 次解条件：对所有 $p \in D^+ V = [-1,1]$，需要 $|p| - 1 \le 0$，即 $|p| \le 1$。对 $p \in [-1,1]$ 确实成立。✓
- $D^- V(1/2) = \emptyset$（没有直线从下方触碰尖角处）
- 上解条件：空集上的条件空真满足。✓

在 $x \ne 1/2$ 处，$V$ 可微：$V'(x) = 1$（$x < 1/2$）或 $V'(x) = -1$（$x > 1/2$），满足 $|V'| = 1$。✓

结论：$V(x) = \min(x, 1-x)$ 是黏性解。

候选解 2：$W(x) = -\min(x, 1-x) = \max(-x, x-1)$（倒帐篷，谷底在 $x=1/2$）

验证 $x = 1/2$ 处：
- $D^+ W(1/2) = \emptyset$（没有直线从上方触碰谷底）
- 次解条件：空集上空真满足
- $D^- W(1/2) = [-1, 1]$
- 上解条件：对所有 $p \in [-1,1]$，需要 $|p| - 1 \ge 0$，即 $|p| \ge 1$。但 $p = 0 \in [-1,1]$ 时 $|0| = 0 < 1$。**违反！** ✗

结论：$W$ **不是**黏性解（不是上解）。

> **本质洞察**：黏性解定义的精妙之处在于它的**非对称性**。对于凸的 Hamiltonian $H$（如 Eikonal 的 $H(p) = |p|$），黏性解选出的是**凹的一支**（帐篷而非倒帐篷）。这对应于最优控制中值函数的**凹性质**（取 inf 的结果）——inf 保持凹性。这个选择不是数学上的任意约定，而是反映了"从无穷多个 a.e. 解中选出物理正确解"的物理要求。

### 比较原理与唯一性

**定理（Crandall-Ishii-Lions 1992, *Bull. AMS* 27:1-67）**。设 $F$ 满足"proper"条件（关于 $r$ 严格单调或退化椭圆），且在适当的结构条件下：若 $u \in \text{USC}(\bar\Omega)$ 为次解、$v \in \text{LSC}(\bar\Omega)$ 为上解，且 $u \le v$ 在 $\partial\Omega$ 上，则 $u \le v$ 在 $\bar\Omega$ 上。

**推论**：黏性解若存在则**唯一**。

证明：设 $V_1, V_2$ 都是黏性解。则 $V_1$ 是次解、$V_2$ 是上解，由比较原理 $V_1 \le V_2$。交换角色得 $V_2 \le V_1$。故 $V_1 = V_2$。

### 比较原理的证明——变量翻倍技巧

证明思路（Crandall-Lions 的核心创新）：

**Step 1（反证）**：假设 $\sup_\Omega(u-v) = \delta > 0$。

**Step 2（变量翻倍）**：引入辅助函数（$\alpha > 0$ 为参数）：

$$\Phi_\alpha(x,y) = u(x) - v(y) - \frac{\alpha}{2}|x-y|^2$$

由于 $u$ USC、$v$ LSC，且 $\bar\Omega$ 紧，$\Phi_\alpha$ 在 $(\bar{x}_\alpha, \bar{y}_\alpha)$ 取最大值。

**Step 3**：$\alpha \to \infty$ 时 $\alpha|\bar{x}_\alpha - \bar{y}_\alpha|^2 \to 0$（惩罚项迫使 $\bar{x}_\alpha \to \bar{y}_\alpha$），且 $u(\bar{x}_\alpha) - v(\bar{y}_\alpha) \to \delta > 0$。对 $\alpha$ 足够大，$(\bar{x}_\alpha, \bar{y}_\alpha)$ 在内部。

**Step 4（构造测试函数）**：

- 对次解 $u$：固定 $\bar{y}$，$x \mapsto u(x) - \frac{\alpha}{2}|x-\bar{y}|^2$ 在 $\bar{x}$ 取最大。取测试函数 $\varphi(x) = v(\bar{y}) + \frac{\alpha}{2}|x-\bar{y}|^2$，次解条件给出：

$$F(\bar{x}, u(\bar{x}), \alpha(\bar{x}-\bar{y})) \le 0$$

- 对上解 $v$：固定 $\bar{x}$，$y \mapsto v(y) + \frac{\alpha}{2}|\bar{x}-y|^2$ 在 $\bar{y}$ 取最小。上解条件给出：

$$F(\bar{y}, v(\bar{y}), \alpha(\bar{x}-\bar{y})) \ge 0$$

**Step 5（得矛盾）**：两式相减，利用 $F$ 的结构条件（对 $r$ 严格单调或 Lipschitz 连续性），在 $\alpha \to \infty$ 极限下得到 $0 \ge \delta > 0$，矛盾。$\square$

> **反事实推理**：如果没有"变量翻倍"技巧会怎样？直接取 $\max(u-v)$ 的点 $x_0$ 看似可行：在 $x_0$ 处 $u-v$ 取最大，所以 $u-v$ 的测试函数条件可以应用。但问题是：$u$ 和 $v$ 的测试函数方向可能不同——$u$ 的超微分和 $v$ 的次微分在同一点不一定有共同方向。变量翻倍的妙处在于：把一维的 $\max(u-v)$ 问题提升到二维 $(x,y)$ 空间中，惩罚项 $\frac{\alpha}{2}|x-y|^2$ 的梯度 $\alpha(x-y)$ **同时**作为 $u$ 和 $v$ 的测试方向——这解决了方向不匹配的难题。

### 存在性——Perron 方法

**定理**。在适当条件下（$F$ 连续、比较原理成立、存在次解和上解），HJB 方程存在唯一黏性解。

**Perron 方法**（类比于调和函数的 Perron 方法）：

$$V(x) = \sup\{w(x) : w \text{ 是次解}, w \le \bar{v}\}$$

其中 $\bar{v}$ 是任一上解（barrier）。

**证明思路**：
1. $V$ 的上半连续包 $V^*$ 是次解（次解族的上确界的 USC 包仍为次解——关键引理）
2. $V$ 的下半连续包 $V_*$ 是上解（由 Perron 定义和比较原理推出）
3. 由比较原理：$V^* \le V_*$（次解 $\le$ 上解）
4. 由定义直接得 $V_* \le V^*$，故 $V^* = V_* = V$ 连续，且同时为次解和上解

**消失粘性验证**：$-\varepsilon\Delta V^\varepsilon + F(x, V^\varepsilon, \nabla V^\varepsilon) = 0$ 的古典解 $V^\varepsilon$ 在 $\varepsilon \to 0$ 时局部一致收敛于唯一黏性解 $V$。这证实了"黏性解"名称的合理性——它确实是消失粘性极限。

### 与值函数等价

**定理（Bardi-Capuzzo-Dolcetta 1997, Thm III.3.2）**。最优控制问题的值函数 $V(x,t)$ 是 HJB 方程的**唯一**黏性解。

**证明骨架**：
- **$V$ 是次解**：由 DPP 的 "$\le$" 方向（取 $\varepsilon$-最优控制并让 $h \to 0$）
- **$V$ 是上解**：由 DPP 的 "$\ge$" 方向（对任意控制取 $h \to 0$）
- **唯一性**：由比较原理

这个定理的重要性在于：它把"求解最优控制问题"完全等价于"求解 HJB 方程的黏性解"——两者给出完全相同的函数。

### ⚠️ 常见陷阱

**陷阱 1：混淆 USC/LSC 与连续性**

- 次解只要求上半连续（USC），上解只要求下半连续（LSC）
- 黏性解要求同时满足，因此是连续的
- 但比较原理的表述中 $u$ 是 USC、$v$ 是 LSC——这允许两者不一定连续

**陷阱 2：认为黏性解概念只适用于一阶 PDE**

- 实际上 Crandall-Ishii-Lions 1992 已经推广到二阶退化椭圆 PDE（随机 HJB）
- 二阶情形需要用 **semijets** $J^{2,\pm}V$ 代替超/次微分

**陷阱 3：忽略 "proper" 条件**

- 比较原理需要 $F$ 对 $r$（即 $V$ 本身）满足某种单调性
- 对 HJB $V_t + \mathcal{H}(x, \nabla V) = 0$，$F$ 不依赖 $r$（零阶项为零），需要额外的强制性条件或有界域
- 折扣 HJB $\beta V + \mathcal{H} = 0$ 天然满足 proper 条件（$F$ 对 $r$ 严格递增）

### 练习

1. ⭐⭐ 验证 $V(x) = |x|$ 是方程 $|V'| = 1$（$x \in \mathbb{R}$，$V(0) = 0$）的黏性解。计算 $x=0$ 处的 $D^+V$ 和 $D^-V$，检验次解和上解条件。

2. ⭐⭐⭐ 对一维 HJ 方程 $V_t + \frac{1}{2}|V_x|^2 = 0$（$t \ge 0$），终端条件 $V(x,0) = -\cos(x)$。
   - (a) 用特征线方法求解，找到特征线首次相交的时间 $t_c$
   - (b) 解释为什么 $t > t_c$ 后经典解不存在
   - (c) 用 Hopf-Lax 公式 $V(x,t) = \inf_y\{V_0(y) + \frac{|x-y|^2}{2t}\}$ 构造黏性解

3. ⭐⭐⭐⭐ （研究级）阅读 Crandall-Ishii-Lions 1992 "User's Guide" 的 Theorem 3.2（Ishii 引理），描述在二阶黏性解中"变量翻倍"如何推广——为什么需要矩阵不等式 $\begin{pmatrix} X & 0 \\ 0 & -Y \end{pmatrix} \le A + \varepsilon A^2$？

---

## §3.4.7 LQR——HJB 的唯一精确可解类 ⭐

### 动机

前面的理论看起来很抽象——黏性解、比较原理、Perron 方法。有没有一个问题可以精确求解 HJB，让我们看到理论是如何工作的？

答案是 **LQR（Linear-Quadratic Regulator）**——线性动力学 + 二次代价。这是**唯一**非平凡但 HJB 精确可解的大类问题。LQR 的值函数是二次的，反馈是线性的，HJB 退化为 Riccati ODE——在专题 3.2（PMP 路径）和 3.3（离散 DP 路径）中我们已经见过。本节从 HJB 路径再推一遍，完成"三路闭环"。

### 问题设置

$$\dot{x} = Ax + Bu, \quad J = \frac{1}{2}x(T)^\top Q_f x(T) + \frac{1}{2}\int_0^T (x^\top Qx + u^\top Ru)\,dt$$

假设：$Q \succeq 0$（状态代价半正定），$R \succ 0$（控制代价正定），$Q_f \succeq 0$（终端代价半正定）。

### HJB 方程

代入 $L = \frac{1}{2}(x^\top Qx + u^\top Ru)$，$f = Ax + Bu$：

$$V_t + \min_u\left\{\frac{1}{2}x^\top Qx + \frac{1}{2}u^\top Ru + \nabla_x V \cdot (Ax + Bu)\right\} = 0$$

终端条件：$V(x,T) = \frac{1}{2}x^\top Q_f x$。

### 二次猜测（Ansatz）

**猜测** $V(x,t) = \frac{1}{2}x^\top P(t)x$，其中 $P(t) = P(t)^\top \succeq 0$。则：

- $V_t = \frac{1}{2}x^\top \dot{P}x$
- $\nabla_x V = P(t)x$

代入 HJB 中的极小化：

$$\min_u\left\{\frac{1}{2}u^\top Ru + (Px)^\top Bu\right\} = \min_u\left\{\frac{1}{2}u^\top Ru + x^\top PBu\right\}$$

对 $u$ 求导令其为零：$Ru + B^\top Px = 0$，得：

$$u^* = -R^{-1}B^\top P(t)x = -K(t)x$$

代回 HJB，比对 $x^\top(\cdot)x$ 的系数（注意左边是 $\frac{1}{2}x^\top[\ldots]x$ 的形式，两边消去 $\frac{1}{2}x^\top[\ldots]x$）：

$$\dot{P} + A^\top P + PA - PBR^{-1}B^\top P + Q = 0$$

终端条件：$P(T) = Q_f$。

写成标准形式（**微分 Riccati 方程 DRE**）：

$$\boxed{-\dot{P}(t) = A^\top P + PA - PBR^{-1}B^\top P + Q, \quad P(T) = Q_f}$$

### 无限时域稳态

令 $T \to \infty$、$\dot{P} = 0$，得**代数 Riccati 方程（CARE）**：

$$0 = A^\top P + PA - PBR^{-1}B^\top P + Q$$

**存在唯一正定稳定解**的条件：$(A,B)$ 可稳定 + $(Q^{1/2}, A)$ 可检测。

### 三路闭环——殊途同归

| 路径 | 推导方法 | 关键步骤 | 得到 |
|------|---------|---------|------|
| PMP（专题 3.2） | Hamilton 方程组 + $\lambda = Px$ | 从 $\dot\lambda = -A^\top\lambda - Qx$ 消去 $\lambda$ | Riccati |
| 离散 DP（专题 3.3） | $V_k = \frac{1}{2}x^\top P_k x$ + 离散 Bellman | 离散 Riccati → $\Delta \to 0$ | DRE |
| HJB（本节） | $V = \frac{1}{2}x^\top Px$ + HJB 方程 | 代入消去 $x$ | DRE |

三种完全不同的出发点，得到**完全相同**的 Riccati 方程——这不是巧合，而是 PMP、DP、HJB 三种最优性条件在 LQR 这个特殊类上的等价性的体现。

> **本质洞察**：LQR 的特殊性在于：线性动力学 + 二次代价使得值函数**恰好**是状态的二次型。这使 HJB 从 PDE 退化为 ODE（Riccati 方程），避免了"全状态空间求解 PDE"的维数灾难。iLQR/DDP 的核心思想正是：对非线性问题做**局部二次化**，每步解一个局部 LQR——本质上是用 Riccati 近似 HJB 的局部曲率。

### 工程实现

```python
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp

# === 示例 1: 稳态 CARE（无限时域）===
# 双积分器 LQR: 位置+速度状态
A = np.array([[0, 1], [0, 0]])  # 状态矩阵: dx1/dt=x2, dx2/dt=u
B = np.array([[0], [1]])         # 输入矩阵: 力直接作用于加速度

Q = np.diag([10.0, 1.0])  # 位置误差权重大（精确定位）
R = np.array([[0.1]])       # 控制代价小 → 允许较大控制力

# 求解代数 Riccati 方程 (CARE)
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ P)  # 最优增益 K = R^{-1} B^T P

# 验证 HJB 残差（应接近机器精度零）
residual = A.T @ P + P @ A - P @ B @ np.linalg.solve(R, B.T @ P) + Q
print(f"HJB residual norm: {np.linalg.norm(residual):.2e}")  # ~1e-14
print(f"Optimal gain K: {K}")
print(f"Closed-loop eigenvalues: {np.linalg.eigvals(A - B @ K)}")
# 闭环极点应在左半平面（稳定）

# === 示例 2: 有限时域 DRE（微分 Riccati）===
def riccati_ode(t, P_flat, A, B, Q, R):
    """微分 Riccati 方程的右端（注意：反向时间积分）"""
    n = A.shape[0]
    P = P_flat.reshape(n, n)
    # -dP/dt = A^T P + P A - P B R^{-1} B^T P + Q
    # 等价于 dP/d(T-t) = -(上式)
    dP = -(A.T @ P + P @ A - P @ B @ np.linalg.solve(R, B.T @ P) + Q)
    return dP.flatten()

T = 5.0  # 终端时间
Qf = np.eye(2)  # 终端代价矩阵
P0 = Qf.flatten()  # 终端条件 P(T) = Qf

# 从 T 反向积分到 0（设 tau = T-t，正向积分 tau）
sol = solve_ivp(riccati_ode, [0, T], P0, args=(A, B, Q, R),
                max_step=0.01, dense_output=True)

# 检验：tau → T 时（即 t → 0），P(t) 应趋近稳态 CARE 解
P_at_t0 = sol.y[:, -1].reshape(2, 2)
print(f"\nP(t=0) vs P_CARE 差异: {np.linalg.norm(P_at_t0 - P):.4f}")
# 当 T 足够大时，差异趋近于零
```

**代码解读**：
- 示例 1 验证了稳态 CARE 解满足 HJB 残差为零（机器精度）
- 示例 2 展示有限时域 DRE 的数值积分；当 $T$ 足够大时，$P(0)$ 趋近稳态解——这验证了"无限时域是有限时域的极限"
- 注意 DRE 是反向积分的：从 $t=T$（已知 $P(T)=Q_f$）积到 $t=0$

### Kalman 滤波的对偶

控制 CARE 与 Kalman 滤波 CARE 有精确的对偶关系：

| | LQR（控制） | Kalman（估计） |
|--|-----------|-------------|
| 系统 | $\dot{x} = Ax + Bu$ | $\dot{x} = Ax + w$，$y = Cx + v$ |
| Riccati | $A^\top P + PA - PBR^{-1}B^\top P + Q = 0$ | $A\Sigma + \Sigma A^\top - \Sigma C^\top V^{-1}C\Sigma + W = 0$ |
| 解的含义 | $P$ = 值函数曲率 | $\Sigma$ = 估计误差协方差 |
| 增益 | $K = R^{-1}B^\top P$（控制增益） | $L = \Sigma C^\top V^{-1}$（Kalman 增益） |
| 对偶变换 | $(A, B, Q, R)$ | $(A^\top, C^\top, W, V)$ |

### ⚠️ 常见陷阱

**陷阱 1：Riccati 方程的时间方向**

- DRE 是**终值问题**（$P(T) = Q_f$，反向积分到 $t=0$）
- 如果写成 $\dot{P} = -(A^\top P + PA - PBR^{-1}B^\top P + Q)$，注意负号
- 数值积分时设 $\tau = T-t$，变成正向问题

**陷阱 2：$R$ 必须正定**

- 如果 $R = 0$（无控制代价），$u^* = -R^{-1}B^\top Px$ 无意义（$R$ 不可逆）
- 物理含义：零控制代价意味着"免费使用无限大控制"→ 问题退化
- 工程上总是取 $R \succ 0$（即使很小），作为正则化

### 练习

1. ⭐ 手算一维 LQR：$\dot{x} = -x + u$，$L = \frac{1}{2}(x^2 + u^2)$，无限时域。求 CARE 的解 $P$、最优增益 $K$、闭环极点。

2. ⭐⭐ 对有限时域 DRE，证明如果 $P(T) = Q_f \succeq 0$ 且 $Q \succeq 0, R \succ 0$，则 $P(t) \succeq 0$ 对所有 $t \le T$ 成立（提示：矛盾法，利用 $P(t)$ 首次触零时 $\dot{P}$ 的符号）。

3. ⭐⭐ 写代码比较有限时域 DRE 解 $P(t)$ 在 $t \to -\infty$ 时是否收敛到 CARE 的稳态解。

---

## §3.4.8 HJB 的数值方法与维数灾难 ⭐⭐

### 动机

除了 LQR 等极特殊问题，HJB 通常无法解析求解。需要数值方法。但 HJB 的数值求解面临两大挑战：(1) 非线性一阶 PDE 的特殊离散化需求（不能用标准方法）；(2) 指数级的维数灾难（状态空间维度 $n \ge 6$ 时经典网格法失效）。

### 经典网格法

在状态空间 $\mathbb{R}^n$ 的笛卡尔网格上离散 $V$，关键是对 $\nabla V$ 的差分近似必须满足**单调性**——否则会产生震荡或收敛到错误的解。

**Lax-Friedrichs 数值 Hamiltonian**：

对 Hamiltonian $H(p) = H(p_1, \ldots, p_n)$，用"人工耗散"版本：

$$\hat{H}^{LF}(p^+, p^-) = H\left(\frac{p^+ + p^-}{2}\right) - \frac{1}{2}\sum_{i=1}^n \alpha_i(p_i^+ - p_i^-)$$

其中 $p_i^+, p_i^-$ 是第 $i$ 方向的前向/后向差商，$\alpha_i = \max|\partial H/\partial p_i|$ 是耗散系数。

时间离散：TVD-RK2 或 TVD-RK3（Total Variation Diminishing Runge-Kutta），受 CFL 条件约束：

$$\Delta t \le C \frac{\Delta x}{\max_i \alpha_i}$$

**高阶空间离散**：使用 **WENO5**（Weighted Essentially Non-Oscillatory，5 阶）重构空间导数，平衡高精度与无振荡。

### Barles-Souganidis 收敛定理

**定理（Barles-Souganidis 1991, *Asymptotic Anal.* 4:271-283）**。数值格式 $S^\rho(x, V^\rho, V^\rho) = 0$（$\rho$ 为网格参数）若同时满足：

1. **单调性（Monotonicity）**：$S(x, r, u) \le S(x, r, v)$ 当 $u \ge v$（增大邻居值不减小方程左边）
2. **稳定性（Stability）**：$\|V^\rho\|_\infty \le C$（一致有界）
3. **一致性（Consistency）**：对光滑函数 $\varphi$，$S^\rho(x, \varphi, \varphi) \to F(x, \varphi, \nabla\varphi)$

且极限方程具备**强比较原理**，则 $V^\rho$ 局部一致收敛于唯一黏性解。

> **本质洞察**：这三个条件是"金科玉律"——任何声称求解 HJB 的数值方法都必须验证。单调性保证数值解趋近正确的弱解（而非另一支）；稳定性防止爆炸；一致性保证逼近正确的方程。PINN/神经网络方法**不满足**单调性条件——这就是它们没有收敛保证的根本原因。

### 半拉格朗日（Semi-Lagrangian）格式

基于 DPP 的直接离散：

$$V(x,t) \approx \min_{a \in U}\{\Delta t \cdot L(x,a) + I_h[V](x + \Delta t \cdot f(x,a), t+\Delta t)\}$$

$I_h$ 为网格上的 $k$-邻居插值。

**优势**：
- 天然单调（DPP 本身具有单调结构）
- CFL 无条件稳定（时间步不受空间网格约束）
- 概念简单，易于实现

**劣势**：
- 精度依赖插值质量（低阶插值会耗散）
- 高维时插值复杂度指数增长

参考：Falcone-Ferretti 2014, *Semi-Lagrangian Approximation Schemes for Linear and Hamilton-Jacobi-Bellman Equations* (SIAM)。

### 静态 HJ 的快速算法

对 Eikonal 类方程 $|\nabla V| = f(x)$（$f > 0$）：

- **Fast Marching Method**（Sethian 1996）：$O(N \log N)$，类似 Dijkstra 算法的连续版本
- **Fast Sweeping Method**（Zhao 2005）：$O(N)$，交替方向扫描

这些算法利用了 Eikonal 方程的**因果结构**（信息单向传播）。机器人路径规划中的 A*/Dijkstra 是 Fast Marching 的离散原型。

### 维数灾难

**根本困难**：笛卡尔网格在 $n$ 维空间中有 $O(N^n)$ 个点（$N$ 为每维网格数）。存储和计算量：

| 维度 $n$ | $N=100$ 网格点数 | 内存（float64） | 实际可行性 |
|----------|-----------------|----------------|-----------|
| 2 | $10^4$ | 80 KB | 毫秒 |
| 3 | $10^6$ | 8 MB | 秒 |
| 4 | $10^8$ | 800 MB | 分钟 |
| 5 | $10^{10}$ | 80 GB | 需集群 |
| 6 | $10^{12}$ | 8 TB | 极限 |
| 7+ | $10^{14+}$ | 不可能 | 完全不可行 |

这就是为什么经典网格法在 $n \ge 6$ 时失效——即使有超级计算机，指数级增长也无法承受。

### 三条现代缓解路线

**路线 1：稀疏网格与张量分解**

- 稀疏网格（Kang-Wilcox 2017）：利用混合光滑性假设，网格点数从 $O(N^n)$ 降到 $O(N(\log N)^{n-1})$
- 张量火车分解（Dolgov-Kalise-Kunisch 2021）：将 $n$-维函数分解为低秩张量积，复杂度 $O(nr^2 N)$（$r$ 为秩）

**路线 2：凸分解与 Hopf-Lax**

对 $H$ 凸或初值凸的情形，Darbon-Osher 2016 证明：

$$V(x,t) = \min_y\left\{J(y) + tH^*\left(\frac{x-y}{t}\right)\right\}$$

其中 $H^*$ 是 Hamiltonian 的 Legendre 变换。这把 PDE 变成**单点凸优化**——复杂度在维度 $n$ 上几乎线性！

限制：需要 $H$ 凸（许多工程问题满足）且无状态依赖（$H = H(p)$，不含 $x$）。

**路线 3：深度学习（DeepReach/PINN）**

用神经网络 $V_\theta(x,t)$ 逼近值函数，通过最小化 PDE 残差训练。复杂度取决于值函数的**复杂度**（而非状态维度），在高维问题中有优势。

详见 §3.4.F（PINN/DeepReach 专节）。

### 库映射

| 库 | 语言 | 特点 | 典型维度 |
|---|---|---|---|
| toolboxLS + helperOC | MATLAB | WENO5+TVD-RK，机器人社区标配 | 3-5D |
| hj_reachability (StanfordASL) | Python/JAX | GPU/TPU，jit/vmap | 4-6D |
| optimized_dp (SFU-MARS) | Python/HeteroCL | 自动生成 CUDA | 5-6D |
| BEACLS | C++/CUDA | toolboxLS 的 C++ 高性能重写 | 5-6D |
| DeepReach | PyTorch | 高维但需训练 | 9-10D |
| FEniCS / deal.II | Python/C++ | FEM，学术 PDE 通用 | 2-3D |

### ⚠️ 常见陷阱

**陷阱 1：用 FEM 求解一阶 HJB**

- 标准 FEM（Galerkin）对一阶双曲 PDE 会产生震荡（不满足单调性）
- 正确选择：upwind 有限差分、DG-FEM（间断 Galerkin）、或半拉格朗日

**陷阱 2：CFL 条件不满足导致爆炸**

- 显式时间积分要求 $\Delta t \le C\Delta x/\max_i\alpha_i$
- 高维问题中 $\Delta x$ 很大（网格粗），CFL 可能很宽松——但如果 Hamiltonian 有大梯度（高速系统），$\alpha_i$ 很大，CFL 变严
- 使用隐式或半拉格朗日方法可避免 CFL 限制

**陷阱 3：维数 6+ 仍用网格法**

- 即使有 GPU 集群，$n \ge 7$ 的网格法内存/时间都不可行
- 正确做法：考虑系统分解（将 $n$-D 问题分解为多个低维子问题）、Hopf-Lax（如果 Hamiltonian 凸）、或 DeepReach

### 练习

1. ⭐ 实现一维 Eikonal 方程 $|V'| = 1$，$V(0) = V(1) = 0$ 的 upwind 差分格式。观察网格加密时解收敛到 $\min(x, 1-x)$。

2. ⭐⭐ 实现二维 Eikonal 方程（正方形域）的 Fast Marching Method。绘制等值线并与解析解比较。

3. ⭐⭐⭐ 对一维 HJ 方程 $V_t + \frac{1}{2}V_x^2 = 0$，分别实现 Lax-Friedrichs 和 Godunov 数值 Hamiltonian，比较两者的耗散性（在特征线相交区域观察解的锐利程度）。

---

## §3.4.9 Hamilton-Jacobi 理论在经典力学中的深层根源 ⭐⭐⭐

### 动机

HJB 方程不是 Bellman 1957 凭空发明的——它的数学结构源自 Hamilton 1834 和 Jacobi 1837 的经典力学理论。理解这个根源不仅有美学意义，更有实践价值：经典力学中的正则变换、作用量-角度变量、Liouville 定理等工具，直接映射到现代最优控制中的对偶变量、可积性条件、相空间体积守恒等概念。

### Hamilton 主函数与力学中的 HJ 方程

在经典力学中，系统的 Hamilton 主函数 $S(q, t)$ 定义为沿真实轨迹的作用量：

$$S(q, t) = \int_{t_0}^{t} L(q(s), \dot{q}(s))\,ds$$

其中 $L = T - V$ 是 Lagrangian（动能减势能）。Hamilton 证明了 $S$ 满足偏微分方程：

$$\frac{\partial S}{\partial t} + H\left(q, \frac{\partial S}{\partial q}, t\right) = 0$$

其中 $H(q, p, t) = p\dot{q} - L$ 是 Hamiltonian，$p = \partial S/\partial q$ 是广义动量。

这与 HJB 方程 $V_t + \min_u\{L + \nabla V \cdot f\} = 0$ 的结构完全相同！区别仅在于：

- 力学中没有"控制选择"（$\min_u$ 不存在），轨迹由初条件唯一确定
- 控制中有多个可选控制，需要在所有可能中选最优

### 正则变换与完全可分性

Jacobi 的贡献在于将 HJ 方程与**正则变换**联系起来。他证明：如果能找到 HJ 方程的**完全积分**（即含 $n$ 个独立常数 $\alpha_1, \ldots, \alpha_n$ 的解 $S(q, \alpha, t)$），则通过代数关系

$$p_i = \frac{\partial S}{\partial q_i}, \quad \beta_i = \frac{\partial S}{\partial \alpha_i}$$

可以直接得到运动方程的通解 $q(t), p(t)$，无需真正求解微分方程。

**完全可分情况**：如果 Hamiltonian 可以写成 $H = H_1(q_1, p_1) + H_2(q_2, p_2) + \ldots$（各自由度解耦），则 HJ 方程用分离变量法可解：$S = S_1(q_1) + S_2(q_2) + \ldots - Et$。这对应于可积系统——行星运动、谐振子等。

### 作用量-角度变量

对周期运动，定义**作用量变量**：

$$J_i = \frac{1}{2\pi}\oint p_i\,dq_i$$

（沿闭轨道的面积）。在作用量-角度坐标 $(J, \theta)$ 中：

- $\dot{J}_i = 0$（作用量守恒）
- $\dot{\theta}_i = \omega_i(J)$（角度均匀旋转）
- 频率 $\omega_i = \partial H/\partial J_i$

这给出了可积系统的完全解：相空间中的运动住在 $n$-环面 $\mathbb{T}^n$ 上。

**与最优控制的类比**：作用量-角度变量把复杂动力学简化为"匀速旋转"。类似地，最优控制中的值函数把复杂最优性条件简化为"梯度下降"——沿 $-\nabla V$ 方向就是最优方向。

### 从力学 HJ 到控制 HJB——关键区别

| 方面 | 经典力学 HJ | 最优控制 HJB |
|------|-----------|-------------|
| 系统类型 | 无控、保守 | 有控、耗散 |
| 方程 | $S_t + H(q, \nabla S) = 0$ | $V_t + \min_u\{L + \nabla V \cdot f\} = 0$ |
| 解的角色 | 作用量（生成函数） | 值函数（最小代价） |
| 梯度含义 | $p = \nabla S$（动量） | $\lambda = \nabla V$（共态/影子价格） |
| 唯一性 | 一般不唯一（多值解合法） | 黏性解唯一 |
| 物理约束 | 能量守恒 $H = E$ | 无类似守恒 |

> **本质洞察**：力学中的 HJ 方程允许多值解（对应不同能量面上的运动），因为力学不需要"选最优"。但控制中的 HJB 必须有唯一解——黏性解理论正是为了保证这个唯一性。从这个角度看，黏性解是 HJ 理论从力学到控制的"升级补丁"。

### Liouville 定理与信息守恒

Hamilton 正则方程 $\dot{q} = H_p$，$\dot{p} = -H_q$ 保持相空间体积：$\text{div}_{(q,p)}(H_p, -H_q) = H_{pq} - H_{qp} = 0$。这就是 **Liouville 定理**。

在最优控制中，类似的结构意味着：沿最优轨迹束，"状态-共态"空间的体积守恒。这解释了为什么间接法 shooting 在长时间积分时可能不稳定——微小的初始误差不会指数增长（体积守恒），但会在不同方向上"拉伸和压缩"（几何不稳定性）。

### 练习

1. ⭐⭐ 对一维谐振子 $H = \frac{1}{2}(p^2 + \omega^2 q^2)$，用分离变量法求 HJ 方程的完全积分 $S(q, E, t) = W(q, E) - Et$。验证 $p = \partial S/\partial q$ 给出正确的动量。

2. ⭐⭐⭐ 对 Kepler 问题 $H = \frac{p_r^2}{2m} + \frac{p_\theta^2}{2mr^2} - \frac{k}{r}$，写出 HJ 方程并用分离变量求解。指出作用量变量 $J_r$ 和 $J_\theta$ 的物理含义。

3. ⭐⭐⭐ 思考：为什么可积系统（有 $n$ 个独立守恒量的 $n$ 自由度系统）的 HJ 方程总可以用分离变量法求解？这与最优控制中的什么条件类比？（提示：考虑 LQR 的可解性条件）

---

## §3.4.10 微分博弈与 Isaacs 方程 ⭐⭐⭐

### 动机

到目前为止，我们假设系统中只有一个决策者（控制器 $u$）。但在安全关键应用中，系统还面临**对抗性扰动** $d$（或"对手"）。这引出**零和微分博弈**——控制器试图最小化代价，扰动试图最大化代价。对应的 PDE 就是 **Isaacs 方程**（Isaacs 1965），它是 HJB 的博弈推广。

### 零和微分博弈

**动力学**：$\dot{x} = f(x, u, d)$，$u \in U$（控制），$d \in D$（扰动）。

**代价函数**：$J = \int_0^T L(x, u, d)\,dt + \Phi(x(T))$

**博弈结构**：$u$ 最小化 $J$，$d$ 最大化 $J$。

**上值函数**（控制先选，扰动后选——最坏情况）：

$$V^+(x,t) = \inf_u \sup_d J$$

**下值函数**（扰动先选，控制后选——最好情况）：

$$V^-(x,t) = \sup_d \inf_u J$$

由构造直接得 $V^- \le V^+$（后选者有信息优势）。

### Isaacs 方程

对上值函数，推导类似 HJB：

$$V_t^+ + \min_u \max_d\{L + \nabla V^+ \cdot f(x,u,d)\} = 0$$

对下值函数：

$$V_t^- + \max_d \min_u\{L + \nabla V^- \cdot f(x,u,d)\} = 0$$

### Isaacs 条件与值的存在

**Isaacs 条件**：如果 Hamiltonian 满足鞍点条件

$$\min_u \max_d\{L + p \cdot f\} = \max_d \min_u\{L + p \cdot f\}$$

则 $V^+ = V^- = V$（博弈有值），且 $V$ 满足统一的 Isaacs 方程。

**充分条件**：$f$ 对 $(u,d)$ 分离（$f = f_0 + g_u u + g_d d$）且 $L$ 对 $u$ 凸、对 $d$ 凹。

### 与 $H_\infty$ 控制的对接

$H_\infty$ 鲁棒控制可视为微分博弈的特殊情况：

- 被控输出 $z$，扰动 $w$
- 目标：$\sup_w \frac{\|z\|^2}{\|w\|^2} \le \gamma^2$（$L_2$ 增益界）
- 等价于求解 Isaacs PDE；线性系统退化为 $H_\infty$ Riccati：

$$A^\top P + PA + P(\gamma^{-2}B_w B_w^\top - B_u R^{-1}B_u^\top)P + Q = 0$$

注意与标准 LQR Riccati 的区别：多了 $\gamma^{-2}B_w B_w^\top P$ 项（扰动的"贡献"）。当 $\gamma \to \infty$（不在意扰动），退化为 LQR。

### 工程含义

- **鲁棒 MPC**（对接专题 3.13）：min-max MPC 的终端成本 = Isaacs 值函数
- **Tube MPC**：管道（tube）由 Isaacs 可达集定义
- **HJ Reachability**（§3.4.D）：本质上是 Isaacs 方程的零水平集问题

### 练习

1. ⭐⭐ 对一维系统 $\dot{x} = u + d$，$|u| \le 1$，$|d| \le 0.5$，$L = x^2$。写出上值函数和下值函数的 Isaacs 方程。验证 Isaacs 条件成立。

2. ⭐⭐⭐ 推导线性系统 $\dot{x} = Ax + Bu + Gw$ 的 $H_\infty$ Riccati 方程。解释 $\gamma$ 的物理含义和"临界 $\gamma_{\text{opt}}$"的工程意义。

---

## §3.4.A 随机 HJB——Itô-Bellman 方程 ⭐⭐⭐

### 动机

到目前为止，动力学 $\dot{x} = f(x,u)$ 是确定性的。但真实机器人系统有噪声：传感器噪声、模型不确定性、外部扰动。当噪声用 Brownian motion 建模时（连续时间随机过程），值函数满足的 PDE 多出一个**二阶项**——这就是随机 HJB（或 Itô-Bellman）方程。

### 受控扩散过程

状态方程为 **Itô SDE**：

$$dX_s = f(X_s, u_s)\,ds + \sigma(X_s, u_s)\,dW_s, \quad X_t = x$$

其中 $W_s$ 是 $m$-维 Brownian motion，$\sigma$ 是扩散矩阵。值函数：

$$V(x,t) = \inf_{u(\cdot)} \mathbb{E}\left[\int_t^T L(X_s, u_s)\,ds + \Phi(X_T)\right]$$

### 推导（DPP + Itô 公式）

对 $V(t+h, X_{t+h})$ 用 **Itô 公式**（随机积分中的 Taylor 展开）：

$$V(t+h, X_{t+h}) - V(t,x) = \int_t^{t+h}\left[V_t + \nabla V \cdot f + \frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)\right]ds + \int_t^{t+h}\nabla V^\top \sigma\,dW_s$$

取期望（Itô 积分的鞅性质使随机项消失）：

$$\mathbb{E}[V(t+h, X_{t+h})] - V(t,x) = \mathbb{E}\int_t^{t+h}\left[V_t + \nabla V \cdot f + \frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)\right]ds$$

代入 DPP 的随机版本、令 $h \to 0$：

$$\boxed{V_t + \min_{u \in U}\left\{L + \nabla V \cdot f + \frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)\right\} = 0, \quad V(T,x) = \Phi(x)}$$

**二阶项** $\frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)$ 来自 Itô 修正（$dW$ 的二次变差 $(dW)^2 = dt$），代价是 PDE 从一阶升到二阶。

> **反事实推理**：如果用 Stratonovich 积分（而非 Itô）会怎样？Stratonovich 积分没有 $(dW)^2 = dt$ 的修正项，Taylor 展开与确定性情况相同——但代价是随机积分不再是鞅（$\mathbb{E}[\int \cdot dW] \ne 0$），DPP 的推导失败。Itô 积分的鞅性质是随机最优控制理论的基石。

### 退化椭圆性

如果 $\sigma\sigma^\top \succ 0$（正定），方程是（严格）椭圆的；如果 $\sigma\sigma^\top \succeq 0$（半正定，部分方向无噪声），方程是**退化椭圆**的。退化情况需要 Crandall-Ishii-Lions 1992 的完整二阶黏性解理论。

### 工程应用

- 金融：Merton 投资组合、期权定价（Black-Scholes 是特殊情况）
- 机器人：随机 MPC、风险敏感控制
- RL：策略梯度的连续时间版本（随机策略评估）

### 与专题 3.3 随机 DP 的对接

离散随机 Bellman $V_k = \min_u\{\ell + \mathbb{E}[V_{k+1}]\}$ 在 $\Delta \to 0$ 极限下得随机 HJB；二阶项由 $\mathbb{E}[V(x + \Delta f + \sqrt{\Delta}\sigma\xi)]$ 的 Taylor 展开（$\xi \sim N(0,I)$）自然产生：

$$\mathbb{E}[V(x + \Delta f + \sqrt{\Delta}\sigma\xi)] \approx V + \Delta(\nabla V \cdot f) + \frac{\Delta}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V) + O(\Delta^{3/2})$$

---

## §3.4.D HJ Reachability——机器人安全的理论工具 ⭐⭐⭐

### 动机

机器人安全的核心问题：**从哪些初始状态出发，我能保证系统不会进入危险区域？** 这就是"可达集"问题。HJ Reachability 把它转化为 HJB（实际上是 HJ-Isaacs）方程的零水平集计算。

### 后向可达管道（BRT）

目标集（危险区域）$\mathcal{T} = \{x : l(x) \le 0\}$。控制 $u$ 试图**避开** $\mathcal{T}$，扰动 $d$ 试图**驱入** $\mathcal{T}$。定义值函数：

$$V(x, \tau) = \min\left\{l(x), \quad \max_u \min_d \int_0^\tau \text{(implicit)}...\right\}$$

实际使用的 BRT 方程（Mitchell-Bayen-Tomlin 2005, *IEEE TAC*）：

$$V_\tau + \min\{0, \max_u \min_d \nabla V \cdot f(x,u,d)\} = 0, \quad V(x,0) = l(x)$$

则 $\text{BRT}(\tau) = \{x : V(x,\tau) \le 0\}$ 给出：在时间 $\tau$ 内，无论扰动 $d$ 如何选择，控制 $u$ **无法保证**系统不进入 $\mathcal{T}$ 的初始状态集合。

安全集 = BRT 的补集 = 所有能保证安全的初始状态。

### 工程应用

1. **空中避撞**：两无人机相对动力学（3D Dubins 模型），BRT 给出最小安全距离
2. **FaSTrack**（Herbert 2017）：将规划与安全解耦——规划器在简化模型上跑，可达集提供"追踪误差界"
3. **人-机共享控制**（Fisac 2019）：人驾驶为主、系统在逼近 BRT 边界时接管
4. **着陆包络**：直升机/四旋翼在引擎故障时的可达着陆区域

### 与 CBF 的关系

| 方面 | CBF（专题 3.8） | HJ Reachability |
|------|----------------|-----------------|
| 安全集定义 | 手工设计 $h(x) \ge 0$ | PDE 精确计算 |
| 计算复杂度 | 在线 QP（毫秒） | 离线 PDE（分钟-小时）+ 在线查表 |
| 维度限制 | 无（适用高维） | $\le 6$（经典）/ $\le 10$（DeepReach） |
| 严格性 | 需证明 forward-invariant | 严格最优 |
| 保守性 | 可能过于保守 | 最小保守（tight） |

**两者互补**：低维关键安全（如相对位置）用 HJ Reachability 精确计算，高维约束用 CBF-QP 近似处理。

### 实际计算流程

使用 `hj_reachability` 库（JAX）的典型工作流：

```python
import jax
import jax.numpy as jnp
import hj_reachability as hj

# Step 1: 定义动力学（以 Dubins 车为例）
class DubinsCar(hj.ControlAndDisturbanceAffineDynamics):
    def open_loop_dynamics(self, x, time):
        # x = [px, py, theta], 速度固定 v=1
        return jnp.array([jnp.cos(x[2]), jnp.sin(x[2]), 0.0])
    
    def control_jacobian(self, x, time):
        return jnp.array([[0.0], [0.0], [1.0]])  # 控制 = 转弯率
    
    def disturbance_jacobian(self, x, time):
        return jnp.zeros((3, 1))  # 无扰动

# Step 2: 定义网格和目标集
grid = hj.Grid.from_lattice_parameters_and_boundary_conditions(
    domain=hj.sets.Box(lo=jnp.array([-3, -3, -jnp.pi]),
                       hi=jnp.array([3, 3, jnp.pi])),
    shape=(101, 101, 101)
)

# 目标集: 圆形障碍物 (px^2 + py^2 <= r^2)
obstacle_radius = 0.5
target_values = jnp.sqrt(grid.states[..., 0]**2 + 
                         grid.states[..., 1]**2) - obstacle_radius

# Step 3: 求解 BRT（反向可达管道）
solver_settings = hj.SolverSettings.with_accuracy("very_high")
result = hj.solve(solver_settings, dynamics, grid, 
                  time_span=jnp.linspace(0, 2.0, 201),
                  initial_values=target_values)

# Step 4: 安全集 = BRT 的补集（V > 0 的区域）
# 在线使用: 插值查询 V(x) 的符号
```

### 练习

1. ⭐⭐ 对二维双积分器（$\dot{x}_1 = x_2$，$\dot{x}_2 = u$，$|u| \le 1$），用 `hj_reachability` 计算到目标集 $\{|x_1| \le 0.1, |x_2| \le 0.1\}$ 的后向可达集。绘制不同时间的 BRT 边界。

2. ⭐⭐⭐ 比较 CBF 方法和 HJ Reachability 方法在相同安全约束下的保守性：用 Dubins 车避开圆形障碍物，比较两种方法允许的最小安全距离。

---

## §3.4.E 路径积分控制——从 HJB 到 Monte Carlo ⭐⭐⭐

### 动机

HJB 方程很美，但有维数灾难。有没有办法**不求解 PDE**，而是直接采样估计最优控制？Kappen 2005 的路径积分控制给出了肯定答案——在特定条件下（控制仿射 + 二次控制代价），HJB 可以**精确线性化**，然后用 Feynman-Kac 公式转为前向 Monte Carlo 采样。

### Kappen 变换

**条件**：控制仿射系统 + 二次控制代价 + 噪声比例条件：

$$dX = [f_0(X) + G(X)u]\,dt + B(X)\,dW$$
$$L(x,u) = q(x) + \frac{1}{2}u^\top R u$$
$$\text{且 } \sigma\sigma^\top = GR^{-1}G^\top \cdot \lambda \quad (\text{噪声与控制代价成比例})$$

**desirability 变换**：$\psi(x,t) = \exp(-V(x,t)/\lambda)$

HJB 在这个变换下**精确线性化**为后向热方程：

$$\partial_t \psi + \mathcal{L}_0 \psi - \frac{q(x)}{\lambda}\psi = 0$$

其中 $\mathcal{L}_0$ 是无控扩散的生成元。

### Feynman-Kac 表示

由 Feynman-Kac 定理：

$$\psi(x,t) = \mathbb{E}_{X_t = x}\left[\exp\left(-\int_t^T \frac{q(X_s)}{\lambda}\,ds - \frac{\Phi(X_T)}{\lambda}\right)\right]$$

最优控制：$u^* = \lambda R^{-1}G^\top \nabla\log\psi$

**核心洞察**：**反向 PDE 变成了前向 Monte Carlo 采样**——无需网格，无维数灾难！

### MPPI（Model Predictive Path Integral）

Williams et al. 2017 将路径积分控制工程化为 **MPPI** 算法：

1. 在 GPU 上并行采样 $K$ 条噪声控制序列 $\{\delta u^k\}_{k=1}^K$
2. 对每条轨迹计算代价 $S_k$
3. 按 softmax 加权合成最优控制更新：

$$u_t^{\text{new}} = \sum_k w_k(u_t + \delta u_t^k), \quad w_k \propto \exp(-S_k/\lambda)$$

**特点**：完全无梯度（无需 $\nabla f$）、可用黑盒模拟器（包括神经网络模型）、天然并行（GPU 友好）。

### 机器人部署

- **AutoRally**（Georgia Tech）：1/5 越野车，40 Hz GPU MPPI，高速漂移
- **MuSHR**（University of Washington）：1/10 赛车
- **Dial-MPC / Diffusion-MPPI**（2024-2025）：结合扩散模型生成更好的控制采样

### 理论局限

Kappen 变换要求 $\sigma\sigma^\top \propto GR^{-1}G^\top$（噪声协方差与控制代价成比例）。这不是通用条件，但在以下情况自然满足：

- 控制通道直接加噪声的系统
- 人为设计噪声协方差以满足条件（MPPI 的工程做法）

### MPPI 的详细算法流程

```python
# MPPI 伪代码（核心循环）
def mppi_step(x_current, u_nominal, model, K=1000, lambda_=1.0):
    """
    x_current: 当前状态
    u_nominal: 标称控制序列 [T, m]
    model: 动力学模型 x_next = model(x, u)
    K: 采样数
    lambda_: 温度参数
    """
    T, m = u_nominal.shape
    
    # Step 1: 采样 K 条噪声控制扰动
    delta_u = np.random.randn(K, T, m) * noise_std  # [K, T, m]
    
    # Step 2: 并行前向模拟（GPU 友好）
    costs = np.zeros(K)
    for k in range(K):
        x = x_current.copy()
        for t in range(T):
            u = u_nominal[t] + delta_u[k, t]  # 扰动控制
            x = model(x, u)  # 前向积分
            costs[k] += running_cost(x, u)  # 累积代价
        costs[k] += terminal_cost(x)  # 终端代价
    
    # Step 3: 计算权重（softmax with temperature）
    costs_shifted = costs - np.min(costs)  # 数值稳定
    weights = np.exp(-costs_shifted / lambda_)
    weights /= np.sum(weights)  # 归一化
    
    # Step 4: 加权更新标称控制
    u_new = u_nominal + np.einsum('k,ktm->tm', weights, delta_u)
    
    return u_new
```

**关键工程细节**：
- $\lambda$ 太小 → 权重集中在极少数样本（variance 大，greedy）
- $\lambda$ 太大 → 权重均匀（exploration 过多，收敛慢）
- 实践中 $K = 1000-10000$，在 GPU 上并行，单步 $< 1$ms

### ⚠️ 常见陷阱

**陷阱 1：MPPI 不是简单的随机搜索**

- 错误理解：MPPI 只是"随机试很多控制然后选最好的"（这是 random shooting）
- 正确理解：MPPI 是路径积分控制的 Monte Carlo 实现——它的 softmax 加权有严格的理论依据（Kappen 变换 + Feynman-Kac），不是 heuristic

**陷阱 2：温度 $\lambda$ 与噪声 $\sigma$ 的耦合**

- Kappen 理论要求 $\lambda = \text{tr}(\sigma\sigma^\top R^{-1})$ 或某个与之相关的量
- 实践中工程师常独立调 $\lambda$ 和 $\sigma$——这可能破坏理论保证
- 安全做法：固定 $\sigma$，按 Williams 2017 的公式设定 $\lambda$

### 练习

1. ⭐⭐ 实现一个简单的 MPPI 控制器，控制一维双积分器到原点。比较 $K=100$ 和 $K=5000$ 的性能差异。

2. ⭐⭐⭐ 推导：当 $K \to \infty$ 时，MPPI 的加权平均收敛于路径积分的精确解。提示：大数定律 + importance sampling。

---

## §3.4.F PINN 求解 HJB——神经网络做"场" ⭐⭐⭐

### 动机

网格法有维数灾难，路径积分有条件限制。能否用万能逼近器（神经网络）直接参数化值函数 $V_\theta(x,t)$，通过最小化 HJB 残差来训练？这就是 PINN（Physics-Informed Neural Networks）方法。

### PINN 框架

用 MLP $V_\theta(x,t)$ 同时拟合**边界/终端数据**与 **PDE 残差**：

$$\mathcal{L} = \underbrace{\mathbb{E}_{x}[|V_\theta(x,T) - \Phi(x)|^2]}_{\text{终端条件}} + \lambda_{\text{pde}} \underbrace{\mathbb{E}_{(x,t)}\left[\left(V_{\theta,t} + \min_u\{L + \nabla V_\theta \cdot f\}\right)^2\right]}_{\text{HJB 残差}}$$

关键技术：通过**自动微分**计算 $V_{\theta,t}$ 和 $\nabla_x V_\theta$——无需有限差分。

### DeepReach（Bansal-Tomlin 2021）

- **网络**：SIREN（周期激活 MLP，Sitzmann 2020），对高频细节友好
- **课程学习**：先在 $t \approx 0$ 附近匹配终端条件，逐步反向展开时间范围
- **实证**：9D 多机避撞（3 架飞机 × 3 状态）、10D 窄通道导航
- **突破**：经典网格法在 6D 已是极限，DeepReach 推到 10D

### 理论局限

PINN **不满足** Barles-Souganidis 的单调-一致-稳定三条件——因此**没有收敛保证**。

- Lin 2020（arXiv:2007.14385）证明：如果训练 loss $\to 0$，则 $V_\theta$ 一致收敛于黏性解
- 但训练中不保证 loss 为零（非凸优化的固有困难）
- 实用策略：加 conformal bound（Feng-Qiu-Bansal 2025）给出误差置信区间

### 与 RL 的连接

| RL 组件 | HJB 对应 |
|---------|---------|
| Critic 网络 $V_\theta(s)$ | HJB 值函数的 PINN 近似 |
| TD 误差 $\delta$ | HJB 残差 $V_t + \mathcal{H}$ |
| SAC 的最大熵目标 | Todorov LMDP 的连续化 |
| PPO/TD3 | HJB 的无模型 Monte Carlo 版 |

---

## §3.4.11 最优停止与自由边界问题 ⭐⭐⭐⭐

### 动机

前面讨论的问题都有固定终端时间 $T$。但在很多实际问题中，决策者可以**选择何时停止**——比如机器人决定何时从行走切换到跑步、何时抓取物体、何时终止一次搜索。这类问题称为**最优停止**，值函数满足的不再是纯 PDE，而是一个**变分不等式**（variational inequality），涉及自由边界。

### 问题形式化

停止时间 $\tau$ 是决策变量，值函数：

$$V(x) = \sup_\tau \mathbb{E}\left[\int_0^\tau e^{-\beta s} L(X_s)\,ds + e^{-\beta\tau}\Phi(X_\tau)\right]$$

其中 $\Phi$ 是停止收益。直觉：每时刻可以选择"继续运行"或"立即停止拿 $\Phi(x)$"。

### 变分不等式

$V$ 满足：

$$\min\{-\mathcal{L}V - L, \; V - \Phi\} = 0$$

等价于三个条件同时成立：
1. $V \ge \Phi$（值不低于立即停止收益）
2. $-\mathcal{L}V - L \ge 0$（在继续区域满足 HJB 不等式）
3. $(-\mathcal{L}V - L)(V - \Phi) = 0$（互补松弛）

**停止域** $\{V = \Phi\}$：在这些状态应该立即停止。
**继续域** $\{V > \Phi\}$：在这些状态应该继续运行。
两者的分界面称为**自由边界**——它不是预先给定的，而是解的一部分。

### Smooth Pasting 条件

在自由边界上，$V$ 不仅连续，而且 $C^1$ 粘合（smooth pasting）：

$$V|_{\text{boundary}} = \Phi|_{\text{boundary}}, \quad \nabla V|_{\text{boundary}} = \nabla\Phi|_{\text{boundary}}$$

这个条件可以从黏性解理论推导出来，也可以从最优性的一阶条件直接得到。

### 工程类比

- **American option**：金融中可以提前行权的期权（经典自由边界问题）
- **机器人模态切换**：步态转换（行走↔跑步）的最优切换时机
- **主动探测终止**：何时停止探索、开始利用
- **机器学习的 early stopping**：何时终止训练

---

## §3.4.12 黏性解的稳定性与逼近 ⭐⭐⭐

### 动机

工程中我们永远无法精确求解 HJB——总是用某种近似方法。关键问题是：**近似解是否"接近"真解？** 黏性解理论的一个重要优势是**稳定性**：如果一列函数"几乎满足" HJB（残差很小），则它们在极限下收敛到真正的黏性解。

### 稳定性定理

**定理**。设 $V^\varepsilon$ 是一列连续函数，在黏性解意义下"$\varepsilon$-几乎满足" HJB（即 $|F(x, V^\varepsilon, \nabla V^\varepsilon)| \le \varepsilon$ 在黏性意义下）。若 $V^\varepsilon$ 局部一致有界，则任何收敛子列的极限都是 HJB 的黏性解。

**推论**：如果黏性解唯一（比较原理成立），则 $V^\varepsilon$ 的**全列**收敛到唯一黏性解。

### 半连续包络方法

对于不连续的逼近序列，需要用**半松弛极限**（half-relaxed limits）：

$$\bar{V}(x) = \limsup_{\varepsilon \to 0, y \to x} V^\varepsilon(y) \quad \text{（上半连续包络）}$$
$$\underline{V}(x) = \liminf_{\varepsilon \to 0, y \to x} V^\varepsilon(y) \quad \text{（下半连续包络）}$$

Barles-Perthame 1987 证明：$\bar{V}$ 是次解、$\underline{V}$ 是上解。由比较原理 $\bar{V} \le \underline{V}$，但由次解/上解定义直接得 $\underline{V} \le \bar{V}$，所以 $\bar{V} = \underline{V} = V$（唯一黏性解），即 $V^\varepsilon$ 局部一致收敛。

这个结果的工程意义：**只要你的数值方法满足稳定性条件（解有界）和一致性条件（局部逼近正确的方程），收敛就是自动保证的**——不需要额外的收敛率估计。

### 与数值方法的连接

Barles-Souganidis 1991 的三条件（单调+一致+稳定）正是稳定性定理的具体化：
- 单调性 → 保证数值解是"某种弱解"
- 一致性 → 保证弱解对应正确的方程
- 稳定性 → 保证有界（配合 CFL）

### 练习

1. ⭐⭐⭐ 验证 Lax-Friedrichs 格式满足单调性条件。提示：写出格式并检验对邻居节点值的偏导数符号。

2. ⭐⭐⭐ 解释为什么 PINN 方法不满足单调性条件——神经网络的全局逼近性质如何破坏局部单调结构？

---

## 典型例题精讲 ⭐⭐

### 例题 1：一维 LQR 的 HJB 完整求解

**问题**：$\dot{x} = u$，$L = \frac{1}{2}(x^2 + u^2)$，$\Phi(x) = \frac{1}{2}\alpha x^2$，时间 $[0,T]$。

**Step 1**：写 HJB：$V_t + \min_u\{\frac{1}{2}x^2 + \frac{1}{2}u^2 + V_x \cdot u\} = 0$

**Step 2**：对 $u$ 求极小：$u + V_x = 0 \Rightarrow u^* = -V_x$

**Step 3**：代回：$V_t + \frac{1}{2}x^2 + \frac{1}{2}V_x^2 - V_x^2 = V_t + \frac{1}{2}x^2 - \frac{1}{2}V_x^2 = 0$

**Step 4**：猜 $V = \frac{1}{2}P(t)x^2$。则 $V_t = \frac{1}{2}\dot{P}x^2$，$V_x = P(t)x$。代入：

$$\frac{1}{2}\dot{P}x^2 + \frac{1}{2}x^2 - \frac{1}{2}P^2 x^2 = 0$$

消去 $\frac{1}{2}x^2$：$\dot{P} + 1 - P^2 = 0$，即 $\dot{P} = P^2 - 1$。

**Step 5**：终端条件 $P(T) = \alpha$。这是 Bernoulli ODE，可分离变量：

$$\frac{dP}{P^2-1} = dt \Rightarrow \frac{1}{2}\ln\frac{P-1}{P+1} = t + C$$

对 $\alpha = 1$（特殊情况）：$P(t) \equiv 1$ 是稳态解。

一般解：$P(t) = \frac{(\alpha+1)e^{2(T-t)} + (\alpha-1)}{(\alpha+1)e^{2(T-t)} - (\alpha-1)}$

**验证**：$t \to -\infty$ 时 $P \to 1$（稳态 CARE 解）。

### 例题 2：时间最优控制的 HJB

**问题**：$\dot{x} = u$，$|u| \le 1$，到原点的最小时间。

**HJB 方程**：$V_t + \min_{|u|\le 1}\{1 + V_x \cdot u\} = 0$

极小化：$\min_{|u|\le 1}\{V_x u\} = -|V_x|$（取 $u = -\text{sgn}(V_x)$）

所以：$V_t + 1 - |V_x| = 0$，即 $V_t = |V_x| - 1$。

**稳态**（$V_t = 0$）：$|V_x| = 1$，即 Eikonal 方程！

**解**：$V(x) = |x|$（到原点距离），$u^* = -\text{sgn}(x)$（朝原点全速前进）。

这个解在 $x = 0$ 不可微——正是 §3.4.5 讨论的典型情况。$V(x) = |x|$ 是**黏性解**（§3.4.6 已验证）。

### 例题 3：Hamilton-Jacobi 方程的特征线与解的崩溃

**问题**：$V_t + \frac{1}{2}V_x^2 = 0$，初始条件 $V(x,0) = -\cos(x)$。

这个问题很好地展示了"特征线相交 → 经典解崩溃 → 需要黏性解"的完整过程。

**Step 1：特征线方程**。HJ 方程 $V_t + H(V_x) = 0$（$H(p) = \frac{1}{2}p^2$）的特征线 ODE：

$$\dot{x} = H'(p) = p, \quad \dot{p} = 0, \quad \dot{V} = pH'(p) - H(p) = p^2 - \frac{1}{2}p^2 = \frac{1}{2}p^2$$

解：$p(t) = p_0 = V_x(x_0, 0) = \sin(x_0)$（动量守恒），$x(t) = x_0 + p_0 t = x_0 + t\sin(x_0)$。

沿特征线：$V(x(t), t) = V_0(x_0) + \frac{1}{2}p_0^2 t = -\cos(x_0) + \frac{1}{2}t\sin^2(x_0)$。

**Step 2：特征线的相交**。从不同 $x_0$ 出发的特征线何时相交？

映射 $x_0 \mapsto x(t) = x_0 + t\sin(x_0)$ 变为不可逆当 Jacobian 为零：

$$\frac{\partial x}{\partial x_0} = 1 + t\cos(x_0) = 0$$

最早发生在 $\cos(x_0) = -1$（即 $x_0 = \pi$），此时 $t_c = 1$。

**物理解释**：$x_0 = \pi$ 附近的特征线"聚焦"到同一点——类似光学中的焦散。$t > t_c = 1$ 后，同一个 $x$ 可由多条特征线到达，给出不同的 $V$ 值——经典解不再唯一。

**Step 3：黏性解（Hopf-Lax 公式）**。对 $H$ 凸的 HJ 方程，黏性解由 Hopf-Lax 公式给出：

$$V(x,t) = \min_y\left\{V_0(y) + t \cdot H^*\!\left(\frac{x-y}{t}\right)\right\} = \min_y\left\{-\cos(y) + \frac{(x-y)^2}{2t}\right\}$$

其中 $H^*(v) = \frac{1}{2}v^2$ 是 $H$ 的 Legendre 变换（对二次函数，Legendre 变换是自身）。

这个公式的美妙之处：即使 $t > t_c$，它仍然给出良定义的连续函数——$\min$ 操作自动"选择"了正确的一支（代价最小的那条特征线的值）。

**Step 4：验证黏性解条件**。在 $t < t_c$ 时，Hopf-Lax 解光滑，等于经典解。在 $t > t_c$ 的不可微点处，可以验证 $D^+V$ 和 $D^-V$ 分别满足次解和上解条件。

### 例题 4：二维 Eikonal 方程与距离函数

**问题**：在单位圆盘 $\Omega = \{(x,y) : x^2 + y^2 < 1\}$ 中，求到边界的距离函数：

$$|\nabla V| = 1, \quad V|_{\partial\Omega} = 0$$

**解析解**：$V(x,y) = 1 - \sqrt{x^2 + y^2}$（到圆心的距离 = 1 减去到圆心的距离）。

**不可微点**：只在圆心 $(0,0)$（中轴退化为一个点）处 $V$ 不可微（$\nabla V$ 指向径向外，在原点方向不确定）。

**控制论解释**：从 $(x,y)$ 出发，单位速度走到边界的最短时间。最优策略：径向向外走。

**工程意义**：机器人工作空间中到障碍物的距离场正是 Eikonal 方程的解——这是路径规划（势场法、Fast Marching）的数学基础。

### 例题 5：折扣无限时域 HJB

**问题**：$\dot{x} = -x + u$，$L = \frac{1}{2}(x^2 + u^2)$，折扣率 $\beta = 1$。

**折扣 HJB**：$V + \min_u\{\frac{1}{2}x^2 + \frac{1}{2}u^2 + V'(-x+u)\} = 0$

对 $u$ 求极小：$u + V' = 0 \Rightarrow u^* = -V'$

代回：$V + \frac{1}{2}x^2 + \frac{1}{2}(V')^2 + V'(-x-V') = V + \frac{1}{2}x^2 - \frac{1}{2}(V')^2 - V'x = 0$

猜 $V = \frac{1}{2}Px^2$：$\frac{1}{2}Px^2 + \frac{1}{2}x^2 - \frac{1}{2}P^2x^2 - Px^2 = 0$

整理：$\frac{1}{2}P + \frac{1}{2} - \frac{1}{2}P^2 - P = 0$，即 $P^2 + P - 1 = 0$

解：$P = \frac{-1 + \sqrt{5}}{2} \approx 0.618$（取正根）

最优增益：$u^* = -Px = -0.618x$，闭环极点：$-(1+P) = -1.618 < 0$（稳定）。

> 有趣的是，$P = \frac{\sqrt{5}-1}{2}$ 正是黄金比例的倒数！

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| HJB 方程总有光滑（古典）解 | 大多数非线性最优控制问题的值函数不光滑（特征线交叉、bang-bang 切换），必须在黏性解框架下理解 |
| 黏性解是"近似解"或"弱意义下的次优解" | 黏性解是精确的最优值函数，"黏性"指的是解的定义方式（通过光滑测试函数的接触），而非解的精度 |
| PMP 和 HJB 是两套独立理论 | 两者存在精确对偶：PMP 的协态 $\lambda(t) = \nabla V(x^*(t), t)$，HJB 的特征线就是 Hamilton 正则方程，PMP 是 HJB 沿最优轨迹的"截面" |
| 数值求解 HJB 可以处理任意高维问题 | 传统网格方法（Level-Set）受维数灾难限制在 $d \le 6$；高维需借助 DeepReach 等神经网络逼近或降维技巧 |
| 验证定理（Verification Theorem）对所有情况都适用 | 验证定理要求值函数 $V \in C^1$，而这恰恰在最需要的非光滑情况下失效；黏性解理论正是为填补这个缺口而发展 |
| 消失粘性法 $-\varepsilon \Delta V$ 只是数值技巧 | 消失粘性法揭示了黏性解的物理本质——它是确定性系统的"零噪声极限"，与随机最优控制的联系是深层的（Fleming 1969） |
| Isaacs 条件（$\max\min = \min\max$）对所有微分博弈都成立 | Isaacs 条件是额外假设，仅在 Hamiltonian 关于控制/扰动可分离或满足凸-凹结构时才成立；不满足时上/下值函数不同 |

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 工程用途 |
|--------|---------|------|---------|
| DPP | $V(x,t) = \inf_u\{\int L + V(x(t+h),t+h)\}$ | ⭐ | 所有值函数方法的基础 |
| HJB 推导 | $V_t + \min_u\{L + \nabla V \cdot f\} = 0$ | ⭐ | 最优控制的场方程 |
| 验证定理 | $C^1$ 解 → 充分最优性 | ⭐⭐ | Lyapunov/CBF 合成 |
| HJB-PMP 对偶 | $\lambda = \nabla V$，特征线 = Hamilton 方程 | ⭐⭐ | 理解间接法/iLQR |
| 经典解不存在 | 特征线交叉、bang-bang、约束 | ⭐⭐ | 理解黏性解的必要性 |
| 黏性解定义 | 测试函数外包梯度条件 | ⭐⭐⭐ | 弱解框架 |
| 比较原理 | 次解 ≤ 上解 → 唯一性 | ⭐⭐⭐ | 理论基石 |
| LQR Riccati | $-\dot{P} = A^\top P + PA - PBR^{-1}B^\top P + Q$ | ⭐ | iLQR/DDP 的基础 |
| 数值方法 | Lax-Friedrichs, WENO, Semi-Lagrangian | ⭐⭐ | 低维精确求解 |
| 维数灾难 | $O(N^n)$ → $n \ge 6$ 不可行 | ⭐⭐ | 选择替代方法 |
| 随机 HJB | 多出 $\frac{1}{2}\text{tr}(\sigma\sigma^\top\nabla^2 V)$ | ⭐⭐⭐ | 随机控制/RL |
| HJ Reachability | BRT = 零水平集 | ⭐⭐⭐ | 机器人安全 |
| 路径积分/MPPI | Kappen 变换线性化 HJB | ⭐⭐⭐ | 高速无模型 MPC |
| DeepReach/PINN | 神经网络逼近值函数 | ⭐⭐⭐ | 高维 HJB |

---

## 累积项目：本章新增模块

**项目**：从零构建最优控制求解器

| 章节 | 新增模块 | 功能 |
|------|---------|------|
| 3.2（PMP） | `indirect_shooter.py` | 间接法求解两点边值问题 |
| 3.3（DP） | `discrete_dp.py` | 离散状态空间值迭代 |
| **3.4（HJB）** | **`hjb_solver_1d.py`** | **一维 HJB 的 upwind 差分求解** |
| **3.4（HJB）** | **`lqr_riccati.py`** | **连续时间 LQR 的 Riccati 求解+验证** |
| **3.4（HJB）** | **`eikonal_fast_march.py`** | **二维 Eikonal 的 Fast Marching** |

本章新增三个模块：
1. `hjb_solver_1d.py`：实现一维 HJB 的 Lax-Friedrichs 格式（含 TVD-RK 时间积分）
2. `lqr_riccati.py`：求解 CARE，验证 HJB 残差，绘制闭环相轨迹
3. `eikonal_fast_march.py`：二维 Fast Marching，可视化等时线

---

## 延伸阅读

**教材（按难度排序）**：

| 书目 | 难度 | 适读人群 |
|------|------|---------|
| Kirk 2004, *Optimal Control Theory* Ch.3-4 | ⭐ | 工程入门 |
| Liberzon 2012, *Calculus of Variations and Optimal Control* Ch.5 | ⭐⭐ | 数学本科 |
| Evans 2010, *PDE* 2e Ch.10 | ⭐⭐⭐ | PDE 背景 |
| Bardi-Capuzzo-Dolcetta 1997, *Optimal Control and Viscosity Solutions of HJB Equations* | ⭐⭐⭐⭐ | 一阶理论圣经 |
| Fleming-Soner 2006, *Controlled Markov Processes and Viscosity Solutions* 2e | ⭐⭐⭐⭐ | 二阶理论圣经 |
| Crandall-Ishii-Lions 1992, "User's Guide to Viscosity Solutions" *Bull. AMS* 27:1-67 | ⭐⭐⭐⭐ | 综述必读 |

**关键论文（含 DOI/arXiv）**：

- Crandall-Lions 1983, *Trans. AMS* 277:1-42 — DOI:10.1090/S0002-9947-1983-0690039-8
- Crandall-Ishii-Lions 1992, *Bull. AMS* 27:1-67 — arXiv:math/9207212
- Barles-Souganidis 1991, *Asymptotic Anal.* 4:271-283 — DOI:10.3233/ASY-1991-4305
- Mitchell-Bayen-Tomlin 2005, *IEEE TAC* 50(7):947-957 — DOI:10.1109/TAC.2005.851439
- Bansal-Tomlin 2021, *ICRA*, DeepReach — arXiv:2011.02082
- Kappen 2005, *J. Stat. Mech.* P11011 — arXiv:physics/0505066
- Williams et al. 2017, *JGCD* 40(2):344-357 — DOI:10.2514/1.G001921
- Darbon-Osher 2016, *Res. Math. Sci.* 3:19 — DOI:10.1186/s40687-016-0068-7
- Raissi et al. 2019, *JCP* 378:686-707 — DOI:10.1016/j.jcp.2018.10.045

**代码仓库**：

- toolboxLS + helperOC: https://github.com/HJReachability/helperOC
- hj_reachability (JAX): https://github.com/StanfordASL/hj_reachability
- optimized_dp (CUDA): https://github.com/SFU-MARS/optimized_dp
- DeepReach: https://github.com/smlbansal/deepreach
- MPPI/AutoRally: https://github.com/AutoRally/autorally
- DeepXDE: https://github.com/lululxvi/deepxde
- CasADi: https://github.com/casadi/casadi

---

## 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| HJB 数值解震荡/不收敛 | 空间离散不满足单调性 | 1. 检查是否用了 upwind 差分 2. 换 Lax-Friedrichs 3. 降低网格精度看趋势 | §3.4.8 |
| 值函数在某区域为 NaN | CFL 条件违反导致爆炸 | 1. 检查 $\Delta t$ vs $\Delta x/\alpha$ 2. 降低 $\Delta t$ 3. 用隐式或 SL 格式 | §3.4.8 |
| LQR Riccati 解不对称/不正定 | 数值积分误差累积 | 1. 每步对称化 $P \leftarrow (P+P^\top)/2$ 2. 用更高阶 ODE 积分器 3. 检查终端条件 | §3.4.7 |
| DeepReach loss 不下降 | 时域太长/激活函数不适 | 1. 用课程学习（从短时域开始）2. 换 SIREN 激活 3. 检查学习率和batch size | §3.4.F |
| MPPI 行为过于保守/激进 | 温度 $\lambda$ 不当 | 1. $\lambda$ 太小 → 过于集中(greedy) 2. $\lambda$ 太大 → 过于分散(random) 3. 按 Williams 2017 自适应调节 | §3.4.E |
| 可达集边界不光滑 | 网格太粗/数值耗散 | 1. 加密网格 2. 用 WENO5 代替 upwind1 3. 检查 $\alpha_i$ 的计算 | §3.4.D |

---

## 跨章综合练习

**综合题 1（需要专题 3.2 + 3.4）**：对双积分器 $\ddot{q} = u$，$|u| \le 1$，到原点最小时间问题：
- (a) 用 PMP 求解最优轨迹和切换曲线（专题 3.2 方法）
- (b) 写出对应的 HJB 方程并解释为什么经典解不存在（本专题方法）
- (c) 验证 PMP 的共态 $\lambda(t)$ 等于 $\nabla V$ 沿最优轨迹的值（对偶关系）
- (d) 用数值方法（Lax-Friedrichs）求解 HJB 并绘制值函数等值线，验证切换曲线对应 $V$ 的不可微线

**综合题 2（需要专题 3.3 + 3.4 + 3.7）**：设计一个对比实验：
- 离散 DP（专题 3.3 的值迭代）
- 连续 HJB（本专题的网格法）
- iLQR（利用 HJB 的 Riccati 近似）
在倒立摆摆起问题（$n=2$，$\dot\theta = \omega$，$\dot\omega = \frac{g}{l}\sin\theta + \frac{u}{ml^2}$）上比较三者的解精度和计算时间。

---

## Hamilton-Jacobi 理论在经典力学中的起源 ⭐⭐⭐

### 正则变换与生成函数

在经典力学中，Hamilton-Jacobi 理论的原始目标是：寻找一个**正则变换** $(q,p) \to (Q,P)$ 使得新 Hamiltonian $K(Q,P) = 0$。在 $K = 0$ 的坐标中，$\dot{Q} = 0$，$\dot{P} = 0$——运动方程变为平凡的！

这个变换的生成函数 $S(q, P, t)$（Hamilton 主函数）满足：

$$\frac{\partial S}{\partial t} + H\left(q, \frac{\partial S}{\partial q}, t\right) = 0$$

这正是 Hamilton-Jacobi 方程。$p = \partial S/\partial q$ 给出旧动量与新坐标的关系。

### 作用量-角度变量

对可积系统（如 Kepler 问题、谐振子），Hamilton-Jacobi 方法的终极成果是**作用量-角度变量** $(J, \theta)$：
- 作用量 $J_i = \frac{1}{2\pi}\oint p_i\,dq_i$ 是守恒量（对应闭轨道的"面积"）
- 角度 $\theta_i$ 线性增长：$\dot\theta_i = \omega_i(J)$

这给出了可积系统的完全解——轨道住在环面上，角度均匀旋转。

### 从力学到控制——150 年的桥梁

Hamilton 研究的是**无控系统**的运动：给定初条件，轨迹唯一确定。Bellman 研究的是**有控系统**的优化：寻找使代价最小的控制。

两者的连接：
- Hamilton-Jacobi 方程中的 $S$（作用量）对应 HJB 中的 $V$（值函数）
- 力学中 $p = \partial S/\partial q$ 对应控制中 $\lambda = \nabla V$（共态 = 值函数梯度）
- 力学中的 Hamilton 典则方程对应控制中的 PMP 状态-共态系统

> **本质洞察**：最优控制理论是经典力学的**推广**。力学中只有一个"控制"（由 Lagrangian 确定的唯一运动），对应于代价函数 $L$ 选定后的极值路径。当引入多个可选控制时，极值路径变成最优轨迹，Hamilton-Jacobi 方程变成 HJB 方程。这就是为什么最优控制继承了力学的全部数学结构（Hamiltonian、正则方程、变分原理）。

---

## 现代进展与前沿 ⭐⭐⭐⭐

### DeepReach 及其后续（2021-2025）

Bansal-Tomlin 2021 的 DeepReach 开创了用神经网络求解高维 HJI 方程的先河。后续发展：

- **Conformal DeepReach**（Feng-Qiu-Bansal 2025）：给值函数逼近加置信区间，解决"训练好了但不知道误差多大"的问题
- **Decomposition**（Chen et al. 2018）：将高维系统分解为多个低维子系统，各自求解后组合
- **Warm-starting**：用上一个时间步的解初始化下一步训练，大幅加速

### 与强化学习的融合

- **Continuous-Time RL**（Doya 2000, Munos-Bourgine 2006）：TD 误差 → HJB 残差
- **HJB 正则化的 Actor-Critic**：在 RL 训练中加入 HJB 残差作为物理约束
- **Safety RL via HJ**：用 HJ 可达集约束 RL 策略的探索范围

### 量子计算求解 HJB

2024-2025 年出现用量子退火/VQE 求解 HJB 的探索性工作——目前只在极小维度上有概念验证，但展示了量子优势的可能性。

---

## 常见陷阱总表

> 将全章陷阱系统化整理，供快速查阅。

| 编号 | 陷阱类型 | 错误描述 | 根本原因 | 正确做法 |
|------|---------|---------|---------|---------|
| T1 | 概念 | 把 HJB 当初值问题 | HJB 是后向方程 | 终端条件已知，反向积分 |
| T2 | 概念 | 认为 HJB 总有 $C^1$ 解 | 特征线相交是常态 | 使用黏性解框架 |
| T3 | 概念 | 混淆 PMP（必要）与 HJB（充分） | 两者是对偶关系 | 明确各自的条件类型和适用场景 |
| T4 | 编程 | 用标准 FEM 解 HJB | 一阶非线性 PDE 不适用 Galerkin | 用 upwind/WENO/SL |
| T5 | 编程 | CFL 条件不满足 | 显式格式的稳定性要求 | $\Delta t \le C\Delta x/\max\alpha$ |
| T6 | 编程 | 维度 6+ 仍用网格法 | 指数维数灾难 | 换 Hopf-Lax / DeepReach |
| T7 | 概念 | 忽略黏性解选择正确一支 | a.e. 解不唯一 | 始终验证黏性解条件 |
| T8 | 概念 | Riccati 方程时间方向写反 | DRE 是终值问题 | 注意负号：$-\dot{P} = \ldots$ |
| T9 | 编程 | DeepReach 不做课程学习 | 长时域+硬约束导致训练不稳定 | 从短时域逐步扩展 |
| T10 | 概念 | 认为 PINN 有收敛保证 | 不满足 Barles-Souganidis | 加 conformal bound 或后验验证 |
| T11 | 编程 | MPPI 温度 $\lambda$ 设定不当 | Kappen 变换的噪声条件 | 按 Williams 2017 方法自适应调节 |
| T12 | 思维 | 认为 HJB 和 RL 是不同体系 | RL = HJB 的无模型/采样版本 | 理解统一框架：值函数 PDE ↔ 时序差分 |

---

## 思维方式总结——如何在工程中使用 HJB 理论

### 决策流程图

面对一个最优控制问题，如何选择方法？

```
问题维度 n = ?
    │
    ├─ n ≤ 4：直接网格法（toolboxLS, hj_reachability）
    │         精确黏性解，离线计算+在线查表
    │
    ├─ n = 5-6：优化网格法（optimized_dp, BEACLS）
    │           GPU 加速，仍可精确但需集群
    │
    ├─ n = 7-10：DeepReach / 系统分解
    │            近似但突破维数墙
    │
    └─ n > 10：放弃全状态 HJB
              改用：
              ├─ PMP 间接法（单轨迹）
              ├─ iLQR/DDP（局部二次化）
              ├─ MPC（滚动时域）
              └─ RL（采样+逼近）
```

### "不是 X 而是 Y" 的核心认知纠正

1. HJB **不是**一种数值方法，**而是**最优控制的理论框架（方程本身是精确的）
2. 黏性解 **不是**对经典解的近似，**而是**更广泛的精确解概念
3. 维数灾难 **不是**HJB 理论的缺陷，**而是**"完整信息"的固有代价（知道所有状态的最优策略需要 $O(N^n)$ 信息）
4. MPPI **不是**HJB 的替代品，**而是**特殊条件下 HJB 的等价重写
5. DeepReach **不是**已经解决了维数灾难，**而是**用精度换维度（无收敛保证）

### 从 HJB 看各方法论的统一性

最优控制的各种方法看似独立，实际上都是 HJB 的不同近似/求解策略：

```
                    HJB 方程 (精确)
                        │
         ┌──────────────┼──────────────┐
         │              │              │
    全状态空间求解    局部展开        采样逼近
         │              │              │
    ┌────┴────┐    ┌────┴────┐    ┌────┴────┐
    │         │    │         │    │         │
  网格法   PINN   iLQR    DDP    MPPI    RL
  (精确)  (近似)  (二阶)  (二阶)  (MC)   (TD)
```

- **网格法**：在全空间离散求解 HJB，精确但维数有限
- **PINN/DeepReach**：用神经网络参数化解，高维但不精确
- **iLQR/DDP**：在当前轨迹附近二阶展开 HJB（Riccati 近似）
- **MPPI**：利用 Kappen 变换将 HJB 化为前向 Monte Carlo
- **RL（TD/Actor-Critic）**：用采样估计 HJB 残差，无需模型

> **本质洞察**：所有这些方法都在回答同一个问题："从状态 $x$ 出发，最小总代价是多少？"HJB 给出了这个问题的精确数学表达。不同方法只是在"精度 vs 维度 vs 模型需求 vs 在线/离线"四个维度上做不同的权衡。理解 HJB 框架，就能在 debug 时定位"哪个近似层出了问题"。

---

## 结论：HJB 作为最优控制的统一场论

本专题完成了从离散 Bellman 方程到连续时间 HJB 方程的完整升维，建立了最优控制的"场论"视角。回顾本章开头提出的九个学习目标——现在每一个都已覆盖。

三条核心认识值得带走：

**第一，黏性解不是装饰，而是必需品。** 机器人中所有有趣的问题（bang-bang 切换、状态约束、障碍避让）都会撕裂值函数的经典梯度。Crandall-Lions 1983 的黏性解通过测试函数外包了不可微点的梯度条件，保留了唯一性与值函数等价性。比较原理的"变量翻倍"技巧和 Barles-Souganidis 的"单调-一致-稳定"收敛条件是理论与数值的两大基石。

**第二，维数灾难推动了方法论的三分野。** 经典网格法止步 $d \approx 6$，Darbon-Osher 的 Hopf-Lax 用凸分解（条件严格但维度近线性）、DeepReach 用神经网络（无收敛保证但实证有效）、FaSTrack/分解法用系统降维（需要物理洞察但通用性高）——三条路线互补而非替代，生产中常组合使用。

**第三，HJB 是 RL、MPC、CBF、MPPI 的共同祖先。** Actor-Critic 的 critic = 离散化 HJB；iLQR 每步 Riccati = HJB 二阶近似；CBF-QP = HJB 极小化的单点投影；MPPI = 线性化 HJB 的 Monte Carlo 前向采样。理解 HJB 让工程师在不同范式间自如切换，并在 debug 时知道哪个近似层在犯错——这是算法成熟度的分水岭。

**展望**。下一个专题（3.5 逼近动态规划与神经值函数）将把本专题的 PINN/DeepReach 扩展为完整的"值函数学习"体系，把连续时间的"场论"正式接入深度强化学习的工程栈。从 Bellman 1957 到 AlphaGo 2016，从 Crandall-Lions 1983 到 DeepReach 2021，HJB 理论的 70 年演进史证明：**深刻的数学结构不会过时——它们只是不断找到新的计算化身。**

---
