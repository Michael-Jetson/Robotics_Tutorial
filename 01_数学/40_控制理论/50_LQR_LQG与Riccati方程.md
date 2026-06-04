## 博士前数学路线图 · 第三批 · 专题 3.5：LQR / LQG 与 Riccati 方程

> **档位**：核心档位 3（博士入学）＋ 进阶档位 4（博士毕业＋）
> **建议时长**：档位 3 约 12–15 h；档位 4 额外 8–12 h
> **前置**：专题 3.1 变分法、3.2 PMP、3.3 离散 DP、3.4 HJB 方程；第零批线性代数＋实分析；第二批凸分析
> **下游**：专题 3.9（iLQR/DDP）、3.11–3.12（MPC）、第六批（RL）

---

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回专题 3.1-3.4 复习）

1. HJB 方程的一般形式是什么？它与 Bellman 方程的关系？
2. PMP 的一阶必要条件中，Hamilton 函数、共态方程和横截条件分别是什么？
3. 什么是正定矩阵？如何判断一个对称矩阵是否正定？Cholesky 分解与正定性的关系？
4. Lyapunov 稳定性的含义？如何用 Lyapunov 函数证明一个线性系统渐近稳定？
5. 线性系统的可控性和可观性的 Kalman 秩条件是什么？

---

## 本章目标

学完本章后，你应当能够：
- 从 HJB、DP、PMP、完成平方法四条路径独立推导出 Riccati 方程，并理解它们为何殊途同归
- 判定给定系统的 ARE 是否有唯一稳定化解（可稳定化 + 可检测条件），并用数值方法求解
- 建立 LQR-Kalman-LQG 的完整对偶框架，严格证明分离原理
- 分析 LQR 的鲁棒性（回差不等式）以及 LQG 为何丧失鲁棒性（Doyle 1978）
- 将 LQR 应用于倒立摆、四旋翼等典型机器人系统的控制器设计

---

## 知识树

```
LQR/LQG 与 Riccati 方程
├── 问题定义：为什么线性+二次=闭式解（§3.5.1）
├── 有限时域 LQR
│   ├── HJB 路径 → Riccati 微分方程（§3.5.2）⭐⭐
│   ├── DP 路径 → Riccati 差分方程（§3.5.3）⭐⭐
│   ├── PMP 路径 → Hamiltonian 矩阵（§3.5.5）⭐⭐
│   └── 完成平方法 → 纯代数验证（§3.5.3b）⭐⭐
├── 无限时域 LQR
│   ├── CARE/DARE 代数方程（§3.5.4）⭐⭐⭐
│   ├── 存在唯一性证明（§3.5.4）⭐⭐⭐
│   └── 数值算法：Schur/Newton-Kleinman/SDA（§3.5.I）⭐⭐⭐
├── 鲁棒性分析
│   ├── 回差不等式与经典裕度（§3.5.5b）⭐⭐
│   └── LQG 鲁棒性丧失（Doyle 1978）（§3.5.7）⭐⭐⭐
├── LQG 框架
│   ├── Kalman 滤波器（§3.5.8）⭐⭐
│   ├── 分离原理完整证明（§3.5.7）⭐⭐⭐
│   └── Kalman-LQR 对偶性（§3.5.8）⭐⭐
├── 推广与前沿
│   ├── H2/H∞ 与博弈 Riccati（§3.5.A）⭐⭐⭐⭐
│   ├── LTR（§3.5.B）⭐⭐⭐
│   ├── 流形 LQR / TVLQR / iLQR（§3.5.C）⭐⭐⭐
│   └── Policy Gradient on LQR（§3.5.F）⭐⭐⭐⭐
└── 典型例题
    ├── 倒立摆 LQR 设计全流程
    └── 卫星姿态控制 LQR
```

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含推导练习） | 10-12 小时 | 需要完整掌握 Riccati 四条推导路径和 LQR-RL 连接的读者 |
| 速读（跳过证明细节） | 4-5 小时 | 已有控制理论基础、需要快速理解 LQR 框架的读者 |
| 速查（只看表格和速查卡） | 30 分钟 | 遇到具体 ARE 求解或 LQR 调参问题时回来查阅 |

---

### 0. 为什么 LQR 是最优控制的"果蝇"

**线性二次调节器（Linear Quadratic Regulator, LQR）是整个最优控制理论中唯一一个能被 PMP、HJB、DP、完备平方四套机器同时闭式求解的问题**，而答案最终汇聚到同一个对象——**Riccati 方程**。这是 LQR 作为"共同验证平台"的第一层独特地位。

果蝇（*Drosophila*）基因组足够简单，让生物学家能够彻底理解一个完整生命系统。LQR 在数学上扮演同样的角色：动力学 $\dot x=Ax+Bu$ 线性，代价 $J=\tfrac12\int(x^\top Q x+u^\top R u)\,dt$ 二次，解 $u^\star=-Kx$ 是状态的线性反馈——**结构简单到可以把理论每个假设都校验一遍**，复杂到足够涵盖现实工程中 80% 的稳定化任务。

第二层独特性：LQR 是连接经典控制、现代控制、鲁棒控制与学习型控制的**枢纽**。经典频域的增益/相位裕度、现代状态空间的 Riccati、鲁棒控制的 $H_\infty$ game-theoretic Riccati、RL 的 policy gradient 全局收敛分析，都以它为核心基准。Fazel 等（ICML 2018）把它称作"强化学习理论的 benchmark"。

第三层：**机器人直接使用**。四旋翼姿态、自动驾驶横向控制、SLAM 里的 EKF、MPC 的终端代价、iLQR 的 backward pass，全部是 LQR 的变体或特例。掌握 LQR = 掌握一把能拆解绝大多数控制问题的万能钥匙。

本专题的主线：**从四条独立路径（HJB、DP、PMP、Lagrange）推出同一个 Riccati 方程 → 在 LQG 中统一控制与估计（分离原理＋对偶性）→ 在 $H_\infty$ 中对抗最坏情形扰动 → 在流形上推广到 SE(3) → 在学习视角下讨论样本复杂度与 in-context learning**。

---

### 一、核心章节（档位 3）

#### §3.5.1 问题定义与"线性 + 二次 ⇒ 闭式解"之谜

**连续时间 LQR**：给定线性时不变系统
$$\dot x(t)=Ax(t)+Bu(t),\qquad x(0)=x_0,\quad x\in\mathbb R^n,\ u\in\mathbb R^m,$$
最小化二次代价
$$J(u)=\frac12\int_0^T\bigl(x^\top Qx+u^\top Ru\bigr)\,dt+\frac12 x(T)^\top Q_f x(T),\qquad Q\succeq0,\ R\succ0,\ Q_f\succeq0.$$

**为什么闭式解不是巧合**。三个代数事实决定了一切：(i) 线性动力学对状态仿射；(ii) 二次代价关于 $(x,u)$ 为正定二次型；(iii) 叠加原理让值函数必须是 $x$ 的二次型 $V(x,t)=\tfrac12 x^\top P(t)x$。结合 HJB 方程对 $u$ 求最小，就是**求一个二次函数的最小值**——总存在显式解。**任何破坏这三条中一条的改动（非线性动力学、非凸代价、约束）都会毁掉闭式性**。这也解释了为什么 iLQR（§3.5.C）的核心思想是"在轨迹附近线性化 + 二次化，把 LQR 当子程序反复使用"。

**离散时间 LQR**：$x_{k+1}=A x_k+B u_k$，代价 $J=\tfrac12\sum_{k=0}^{N-1}(x_k^\top Q x_k+u_k^\top R u_k)+\tfrac12 x_N^\top Q_f x_N$。结构完全平行，Riccati 从微分方程变为差分方程。

#### §3.5.2 有限时域连续 LQR：从 HJB 到 Riccati 微分方程 ⭐⭐

##### 动机：为什么值函数是二次的

在一般最优控制问题中，HJB 方程是一个非线性偏微分方程，求解极其困难。但 LQR 有一个"奇迹"：由于动力学线性、代价二次，值函数 $V(x,t)$ 必然是状态 $x$ 的二次型。这不是猜测，而是结构必然——叠加原理保证任何初值的代价都是初值各分量的二次组合。

> **本质洞察**：LQR 的闭式解不是数学技巧的产物，而是线性叠加 + 二次代价这一对称结构的必然后果。一旦破坏任何一个条件（加入状态约束、非线性动力学、非二次代价），闭式性立刻消失——这正是为什么 MPC 和 iLQR 需要在线优化。

##### 完整推导过程

HJB 方程（专题 3.4 结果）：
$$-\frac{\partial V}{\partial t}=\min_u\Bigl\{\tfrac12 x^\top Q x+\tfrac12 u^\top R u+(\nabla_x V)^\top(Ax+Bu)\Bigr\},\quad V(x,T)=\tfrac12 x^\top Q_f x.$$

**Step 1：二次型 Ansatz。** 设 $V(x,t)=\tfrac12 x^\top P(t)x$，其中 $P(t)=P(t)^\top\succeq0$。这一假设的合理性来自三重论证：(i) 终端条件 $V(x,T)=\tfrac12 x^\top Q_f x$ 是二次的；(ii) 对线性动力学，若当前值函数是二次的，则往前一步的值函数仍是二次的（归纳）；(iii) 由叠加原理，$V(\alpha x,t)=\alpha^2 V(x,t)$（齐次性）强制 $V$ 为二次型。

**Step 2：计算梯度与代入。** 由 $V=\tfrac12 x^\top Px$，得 $\nabla_x V=Px$，$\partial V/\partial t=\tfrac12 x^\top\dot P x$。大括号内表达式为：
$$L(x,u)=\tfrac12 x^\top Qx+\tfrac12 u^\top Ru+x^\top P(Ax+Bu)=\tfrac12 x^\top Qx+\tfrac12 u^\top Ru+x^\top PAx+x^\top PBu.$$

**Step 3：对 $u$ 求极小（一阶条件）。** 对 $u$ 求导并令其为零：
$$\frac{\partial L}{\partial u}=Ru+B^\top Px=0\quad\Longrightarrow\quad u^\star=-R^{-1}B^\top P(t)x.$$
由于 $R\succ0$，Hessian $\partial^2 L/\partial u^2=R\succ0$，确认这是最小值而非鞍点。

**Step 4：代回 HJB 提取 Riccati。** 将 $u^\star=-R^{-1}B^\top Px$ 代入 HJB：
$$-\tfrac12 x^\top\dot P x=\tfrac12 x^\top Qx+\tfrac12 x^\top PBR^{-1}RR^{-1}B^\top Px+x^\top PAx-x^\top PBR^{-1}B^\top Px.$$
化简（注意 $\tfrac12 u^\star{}^\top Ru^\star=\tfrac12 x^\top PBR^{-1}B^\top Px$，而交叉项为 $-x^\top PBR^{-1}B^\top Px$，合并得 $-\tfrac12 x^\top PBR^{-1}B^\top Px$）：
$$-\tfrac12 x^\top\dot P x=\tfrac12 x^\top\bigl(Q+PA+A^\top P-PBR^{-1}B^\top P\bigr)x.$$
由于这对所有 $x$ 成立（$P$ 对称），提取系数得到 **Riccati 微分方程（RDE）**：

$$\boxed{-\dot P=A^\top P+PA-PBR^{-1}B^\top P+Q,\qquad P(T)=Q_f.}$$

**Step 5：最优反馈控制律。**
$$u^\star(t)=-K(t)x(t),\qquad K(t)=R^{-1}B^\top P(t).$$
这是**全状态线性反馈**——不是开环轨迹，而是随时间变化的增益矩阵乘以当前状态。数值上倒向积分 RDE（从 $t=T$ 积到 $t=0$）得到 $P(t)$，再正向仿真闭环系统即可。

##### 为什么 RDE 有"负号"

读者容易困惑 $-\dot P=\ldots$ 中的负号。这来自 **时间反向的因果性**：HJB 是从终端向初始"传播"信息的方程（未来代价"投影"到当下），因此自然要求倒向积分。如果写成 $\tau=T-t$（倒向时间），则 $dP/d\tau=A^\top P+PA-PBR^{-1}B^\top P+Q$ 是正号——这是标准的正向 ODE。

##### Riccati 方程的非线性项的物理意义

RDE 中的 $-PBR^{-1}B^\top P$ 项是**控制的"回报"**：当控制代价 $R$ 小时，$R^{-1}$ 大，系统能花更多控制努力让 $P$ 不过大（值函数降低）；当 $B$ 的列空间宽时，系统有更多方向可以施加控制。**如果 $B=0$（无控制），RDE 退化为 Lyapunov 微分方程** $-\dot P=A^\top P+PA+Q$，$P$ 会单调增长（无控制只能积累代价）。

##### 关键结论：RDE 不爆破

**定理（RDE 全局存在性）**：只要 $Q\succeq0$，$R\succ0$，$Q_f\succeq0$，RDE 在 $[0,T]$ 上有唯一 $C^1$ 对称正半定解 $P(t)$，且对所有 $t\in[0,T]$ 有 $0\preceq P(t)\preceq P_{\max}$（有界）。

**证明思路**：值函数的物理解释 $V(x,t)=\min_u J_{[t,T]}$ 对任何有限时域都有限（因为 $u\equiv0$ 已给出有限上界 $J\le\tfrac12\|x\|^2\int_t^T\|e^{A\tau}\|^2\|Q\|d\tau+\tfrac12\|e^{AT}x\|^2\|Q_f\|$）。由 $P(t)$ 就是该值函数的 Hessian，它必然有界。

**如果不这样会怎样**：对比一般的矩阵 Riccati 方程 $\dot X=AX+XD+XBX+C$（Bernoulli 型），当符号不对时（例如 $R$ 不正定），解可以在有限时间内爆破（blowup）——这正是"LQR 奇迹"的反面。在 $H_\infty$ 控制中，$\gamma$ 选太小就会触发 Riccati 的有限时间爆破，对应着博弈无解。

#### §3.5.3 有限时域离散 LQR：从 DP 到 Riccati 差分方程 ⭐⭐

##### 动机：离散时间才是计算机实现的实际形态

虽然连续时间 LQR 在理论上更优美，但真实机器人的控制器跑在数字处理器上——采样周期 $\Delta t$ 把连续问题离散化。离散 LQR 直接对应 MPC 的核心递推，也是 iLQR backward pass 的精确形式。

##### 完整推导

**系统**：$x_{k+1}=Ax_k+Bu_k$，代价 $J=\tfrac12\sum_{k=0}^{N-1}(x_k^\top Qx_k+u_k^\top Ru_k)+\tfrac12 x_N^\top Q_f x_N$。

**DP 反向递推（Bellman 方程）**：定义 cost-to-go $V_k(x)=\min_{u_k,\ldots,u_{N-1}}J_{[k,N]}$。Bellman 给出：
$$V_k(x)=\min_u\bigl\{\tfrac12 x^\top Qx+\tfrac12 u^\top Ru+V_{k+1}(Ax+Bu)\bigr\},\quad V_N(x)=\tfrac12 x^\top Q_f x.$$

**Step 1：归纳假设 $V_{k+1}(x)=\tfrac12 x^\top P_{k+1}x$。** 终端条件验证：$V_N=\tfrac12 x^\top Q_f x$ 确实是二次型（$P_N=Q_f$）。

**Step 2：展开 Bellman。**
$$V_k(x)=\min_u\bigl\{\tfrac12 x^\top Qx+\tfrac12 u^\top Ru+\tfrac12(Ax+Bu)^\top P_{k+1}(Ax+Bu)\bigr\}.$$
展开二次型：
$$=\min_u\bigl\{\tfrac12 x^\top(Q+A^\top P_{k+1}A)x+x^\top A^\top P_{k+1}Bu+\tfrac12 u^\top(R+B^\top P_{k+1}B)u\bigr\}.$$

**Step 3：对 $u$ 求极小。** 令关于 $u$ 的梯度为零：
$$(R+B^\top P_{k+1}B)u+B^\top P_{k+1}Ax=0\quad\Longrightarrow\quad u_k^\star=-(R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}Ax_k.$$
注意 $R+B^\top P_{k+1}B\succ0$（因为 $R\succ0$ 且 $P_{k+1}\succeq0$），所以逆总存在。

**Step 4：代回得到 Riccati 差分方程（DRE 离散版）。**
$$\boxed{P_k=Q+A^\top P_{k+1}A-A^\top P_{k+1}B(R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A,\quad P_N=Q_f.}$$
最优反馈增益：$K_k=(R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A$。

##### 离散→连续的极限过渡

把差分方程取极限 $\Delta t\to0$。设连续系统为 $\dot x=A_c x+B_c u$，离散化为 $A=I+A_c\Delta t+O(\Delta t^2)$，$B=B_c\Delta t+O(\Delta t^2)$。代入离散 Riccati：

$$P_k=Q\Delta t+(I+A_c^\top\Delta t)P_{k+1}(I+A_c\Delta t)-(I+A_c^\top\Delta t)P_{k+1}B_c\Delta t(R+B_c^\top P_{k+1}B_c\Delta t^2)^{-1}B_c^\top\Delta t P_{k+1}(I+A_c\Delta t).$$

展开到 $O(\Delta t)$ 阶并令 $(P_{k+1}-P_k)/\Delta t\to\dot P$，利用 Neumann 级数 $(R+\epsilon M)^{-1}\approx R^{-1}-\epsilon R^{-1}MR^{-1}+\ldots$，严格恢复连续 RDE：
$$-\dot P=A_c^\top P+PA_c-PB_c R^{-1}B_c^\top P+Q.$$

这一极限过渡的意义是**离散与连续最优控制的同构**——它保证用高频离散 LQR 近似连续 LQR 的误差随 $\Delta t\to0$ 消失，也是 MPC 与 LQR 等价性的数学基础。

#### §3.5.3b 完备平方法：纯代数的第四条路径 ⭐⭐

##### 动机：不依赖最优性原理的独立推导

前面三条路径（HJB、DP、PMP）都依赖某种最优性原理或变分条件。但 Riccati 方程还有第四条推导路径——**完备平方法（Completion of Squares）**——它是纯代数操作，不需要任何最优控制先验知识。这条路径的深刻意义在于：它证明了 Riccati 方程的正确性可以通过**直接验证**而非构造性推导来确认。

> **跨领域类比**：完备平方法之于 LQR，就像配方法之于一元二次方程 $ax^2+bx+c=0$ 的求根公式。我们可以"猜到"答案 $x=(-b\pm\sqrt{b^2-4ac})/(2a)$，然后通过配方来验证它确实是最小值——不需要求导。LQR 的完备平方法做的是高维版本的同一件事。

##### 完整推导（离散时间）

**目标**：把总代价 $J=\tfrac12\sum_{k=0}^{N-1}(x_k^\top Qx_k+u_k^\top Ru_k)+\tfrac12 x_N^\top Q_f x_N$ 改写为"与 $u$ 无关的常数项 + 关于 $u$ 的非负二次型"。

**Step 1：引入待定矩阵 $P_k$**。我们猜测存在一组对称矩阵 $\{P_0,P_1,\ldots,P_N\}$ 使得：
$$J = \tfrac12 x_0^\top P_0 x_0 + \tfrac12\sum_{k=0}^{N-1}(u_k+K_k x_k)^\top M_k(u_k+K_k x_k),$$
其中 $M_k\succ0$（正定），$K_k$ 待确定。如果这一改写成立，第一项是常数（$x_0$ 给定），第二项非负，在 $u_k=-K_k x_k$ 时取零——这就是最优控制。

**Step 2：逐步配方**。从终端开始。最后一步的代价：
$$\tfrac12 x_{N-1}^\top Qx_{N-1}+\tfrac12 u_{N-1}^\top Ru_{N-1}+\tfrac12 x_N^\top Q_f x_N.$$
代入 $x_N=Ax_{N-1}+Bu_{N-1}$：
$$=\tfrac12 x_{N-1}^\top(Q+A^\top Q_f A)x_{N-1}+x_{N-1}^\top A^\top Q_f Bu_{N-1}+\tfrac12 u_{N-1}^\top(R+B^\top Q_f B)u_{N-1}.$$

**Step 3：对 $u_{N-1}$ 配方**。令 $M_{N-1}=R+B^\top Q_f B$ 和 $K_{N-1}=M_{N-1}^{-1}B^\top Q_f A$（注意 $M_{N-1}\succ0$）：
$$=\tfrac12(u_{N-1}+K_{N-1}x_{N-1})^\top M_{N-1}(u_{N-1}+K_{N-1}x_{N-1})+\tfrac12 x_{N-1}^\top\underbrace{(Q+A^\top Q_f A-A^\top Q_f B M_{N-1}^{-1}B^\top Q_f A)}_{=:P_{N-1}}x_{N-1}.$$

配方的关键代数恒等式：$ax^2+bx+c=a(x+b/2a)^2+(c-b^2/4a)$。在矩阵版本中，交叉项 $x^\top Hu$ 被吸收进 $(u+M^{-1}H^\top x)^\top M(u+M^{-1}H^\top x)$ 中，余项给出 $P_{N-1}$。

**Step 4：递推**。把 $\tfrac12 x_{N-1}^\top P_{N-1}x_{N-1}$ 与前一步的代价合并，对 $u_{N-2}$ 重复配方。归纳证明 $P_k$ 满足：
$$P_k=Q+A^\top P_{k+1}A-A^\top P_{k+1}B(R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A,\quad P_N=Q_f.$$

这正是**离散 Riccati 差分方程**！最终得到：
$$\boxed{J=\tfrac12 x_0^\top P_0 x_0+\tfrac12\sum_{k=0}^{N-1}\|u_k+K_k x_k\|^2_{R+B^\top P_{k+1}B}.}$$

**Step 5：最优性立刻可见**。第一项不依赖 $u$（固定初值下为常数），第二项是非负二次型，最小值在 $u_k=-K_k x_k$ 时取到，最优代价为 $J^\star=\tfrac12 x_0^\top P_0 x_0$。

##### 为什么完备平方法重要

1. **独立验证**：它不需要 HJB 或 PMP 作为前提，提供了第四条独立路径到 Riccati 方程。四条路径的一致性增强了我们对结果正确性的信心。
2. **鲁棒控制的基础**：在 $H_\infty$ 控制中，S-procedure 和完备平方法的推广版本是处理不确定性的核心工具。
3. **充分性证明**：HJB 和 PMP 给出的是必要条件（一阶条件），完备平方法直接证明了**充分性**——因为它把代价写成了明确的"常数 + 非负项"形式。

##### 四条路径的统一对比

| 路径 | 核心工具 | 关键假设 | 给出的是 | 优缺点 |
|------|---------|---------|---------|--------|
| HJB | 值函数偏微分方程 | $V$ 可微 | 充分条件（验证定理） | 结构最清晰，但需要猜 $V$ 的形式 |
| DP（Bellman） | 最优性原理递推 | 无特殊假设 | 充分+必要 | 离散天然，但连续极限需小心 |
| PMP | 协态方程+极大值原理 | 共态连续 | 必要条件 | 最一般，但需补充 $\lambda=Px$ 假设 |
| 完备平方 | 纯代数配方 | 无 | 充分+必要 | 最简洁，但需"从天而降"猜结构 |

> **本质洞察**：四条路径殊途同归的深层原因在于——LQR 问题只有**一个**"数学真理"（Riccati 方程），不同路径只是从不同角度观察同一数学结构。这就像欧拉公式 $e^{i\pi}=-1$ 可以从 Taylor 展开、微分方程、复平面旋转等多种角度推出——答案唯一，视角多元。

##### 连续时间的完备平方法

对连续时间 LQR，完备平方法给出：
$$J=\tfrac12 x(0)^\top P(0)x(0)+\tfrac12\int_0^T\|u+R^{-1}B^\top P(t)x\|^2_R\,dt.$$
其中 $P(t)$ 满足连续 RDE。推导过程需要 Ito-积分式的"完成平方"技巧（对确定性系统退化为普通积分），具体步骤：

1. 定义 Lyapunov-like 函数 $W(t)=\tfrac12 x(t)^\top P(t)x(t)$
2. 计算 $\tfrac{d}{dt}W=\tfrac12 x^\top\dot P x+x^\top P\dot x=\tfrac12 x^\top\dot P x+x^\top P(Ax+Bu)$
3. 利用 RDE 展开 $\dot P$ 项，代入后与阶段代价 $\tfrac12(x^\top Qx+u^\top Ru)$ 合并
4. 合并得到 $\tfrac12(x^\top Qx+u^\top Ru)+\tfrac{d}{dt}W=\tfrac12\|u+R^{-1}B^\top Px\|^2_R$
5. 两端从 $0$ 到 $T$ 积分，利用 $W(T)=\tfrac12 x(T)^\top Q_f x(T)$，得到闭式恒等式

#### §3.5.4 无限时域 LQR、ARE 与 DARE ⭐⭐⭐

##### 从有限时域到无限时域：稳态 Riccati 的诞生

令 $T\to\infty$、$Q_f=0$。物理直觉是：如果控制任务没有截止时间，那么"还剩多少时间"不再影响最优策略——控制器变成**时不变的**。数学上，这意味着 RDE 的解 $P(t)$ 在 $t\to-\infty$（记住是倒向积分）应该收敛到一个稳态值 $P_\infty$。

在稳态 $\dot P=0$，RDE 退化为**连续代数 Riccati 方程（CARE）**：
$$\boxed{A^\top P+PA-PBR^{-1}B^\top P+Q=0.}$$

离散版本（**DARE**）：
$$\boxed{P=A^\top PA-A^\top PB(R+B^\top PB)^{-1}B^\top PA+Q.}$$

##### 存在唯一性定理的完整证明

**定理（Kalman 1960；Wonham 1968；Anderson-Moore Ch.3）**：若 $(A,B)$ 可稳定化（stabilizable）且 $(A,Q^{1/2})$ 可检测（detectable），则 CARE 存在唯一对称正半定解 $P^\star\succeq0$，并且闭环矩阵 $A_{cl}=A-BK^\star$（$K^\star=R^{-1}B^\top P^\star$）所有特征值具有严格负实部（Hurwitz 稳定）。

**完整证明**：

**第一步：存在性——通过 DRE 的单调收敛。** 考虑 $Q_f=0$，从 $t=T$ 倒向积分 RDE 得到 $P_T(t)$（下标 $T$ 表示终端时间）。由 $P_T(T)=0\preceq Q_f'$ 对任何 $Q_f'\succeq0$，比较定理给出 $P_T(0)\preceq P_{T'}(0)$ 当 $T\le T'$。因此 $\{P_T(0)\}_{T>0}$ 是单调不减序列。

**上界论证**：$(A,B)$ 可稳定化意味着存在 $K_0$ 使 $A-BK_0$ Hurwitz。取次优控制 $u=-K_0 x$ 得到有限代价上界：
$$V(x,0)\le\tfrac12 x^\top\underbrace{\int_0^\infty e^{(A-BK_0)^\top t}(Q+K_0^\top RK_0)e^{(A-BK_0)t}dt}_{=:\bar P}x.$$
其中 $\bar P$ 有限（因为 $A-BK_0$ Hurwitz 保证积分收敛）。所以 $P_T(0)\preceq\bar P$ 对所有 $T$。

**单调有界序列收敛**：$P_\infty:=\lim_{T\to\infty}P_T(0)$ 存在且 $0\preceq P_\infty\preceq\bar P$。由 $P_\infty$ 是 RDE 的稳态，它满足 CARE。

**第二步：闭环稳定性——用可检测性。** 设 $A_{cl}=A-BR^{-1}B^\top P_\infty$。把 CARE 改写为 Lyapunov 方程形式：
$$A_{cl}^\top P_\infty+P_\infty A_{cl}=-(Q+P_\infty BR^{-1}B^\top P_\infty)\preceq-Q.$$
设 $V(x)=x^\top P_\infty x$，沿闭环轨迹 $\dot V=-x^\top(Q+K^{\star\top}RK^\star)x\le-x^\top Qx\le0$。

如果 $Q\succ0$，则 $\dot V<0$ 对 $x\ne0$，直接得 $A_{cl}$ Hurwitz。

如果 $Q\succeq0$（半正定），需要 LaSalle。$\dot V=0$ 的集合是 $\{x:Q^{1/2}x=0\}\cap\{x:K^\star x=0\}$。可检测性 $(A,Q^{1/2})$ 意味着：$Q^{1/2}x(t)\equiv0$ 对所有 $t$ 强制 $x(t)$ 沿 $A_{cl}$ 的不可观子空间演化，但该子空间的所有模态必须稳定（可检测性定义）。因此 LaSalle 不变集只含原点，$A_{cl}$ Hurwitz。

**第三步：唯一性——用 Lyapunov 方程的唯一性。** 假设存在另一个稳定化解 $P'$（即 $A-BR^{-1}B^\top P'$ 也 Hurwitz）。定义 $\Delta=P_\infty-P'$。将两个 CARE 相减：
$$A_{cl}^\top\Delta+\Delta A_{cl}+\Delta BR^{-1}B^\top\Delta=0.$$
由 Lyapunov 方程理论，$A_{cl}$ Hurwitz 且右端半正定意味着 $\Delta\succeq0$。交换角色（用 $P'$ 对应的 $A_{cl}'$）得 $\Delta\preceq0$。故 $\Delta=0$，即 $P_\infty=P'$。$\blacksquare$

##### 可稳定化与可检测的物理意义

| 条件 | 数学定义 | 物理意义 | 如果不满足 |
|------|---------|---------|-----------|
| $(A,B)$ 可稳定化 | 不稳定模态全部可控 | 所有"发散方向"都能被控制器抑制 | 存在不可控的发散模态，$J=\infty$ |
| $(A,Q^{1/2})$ 可检测 | 不可观模态全部稳定 | 代价函数"看不见"的模态不会发散 | $P^\star$ 不唯一或闭环不稳定 |

**工程直觉**：可稳定化说"我有能力管住系统"；可检测说"我的代价函数关心了所有需要关心的东西"。两者缺一，LQR 要么无解（$J=\infty$），要么解不唯一或不稳定。

##### CARE 解的个数与非稳定化解

CARE 一般有**多个**对称解（最多 $2^n$ 个，对应 Hamiltonian 矩阵 $2n$ 个特征值的 $n$ 维子空间选取方式）。但在可稳定化+可检测条件下，**唯一的稳定化解 $P^\star$ 就是最大的正半定解**——所有其他解 $P'\preceq P^\star$。在非标准问题（如不定权重 LQ、$H_\infty$ 博弈）中，可能需要选取非最大解，此时 Hamiltonian 矩阵的其他不变子空间就派上用场了。

##### CARE 与 H2 最优控制的联系

将 LQR 写成传递函数语言：令评价输出 $z=\begin{pmatrix}Q^{1/2}x\\R^{1/2}u\end{pmatrix}$，闭环从扰动 $w$ 到 $z$ 的传递函数 $T_{zw}(s)=\begin{pmatrix}Q^{1/2}\\-R^{1/2}K\end{pmatrix}(sI-A_{cl})^{-1}$，则 LQR 的最优代价恰为 $H_2$ 范数的平方：
$$J^\star=\tfrac12 x_0^\top P^\star x_0=\tfrac12\|T_{zw}\|_2^2\cdot\|x_0\|^2.$$
这一等价性意味着 LQR 就是 $H_2$ 最优控制——在能量意义下最好的线性反馈。从 $H_2$ 到 $H_\infty$（专题 3.6）只需把"能量最优"改为"最坏情况最优"。

#### §3.5.4b ARE 的深层性质：单调性、解的参数化与极值原理 ⭐⭐⭐

##### Riccati 解关于参数的单调性

**定理（比较定理/单调性）**：设 $(A,B,Q_1,R)$ 和 $(A,B,Q_2,R)$ 两个 LQR 问题，若 $Q_1\preceq Q_2$（状态权重更大=对偏差惩罚更重），则对应的 CARE 解满足 $P_1^\star\preceq P_2^\star$。

**物理意义**：增大状态惩罚使系统更"急迫"地回到原点——需要更大的最优代价（$P$ 更大），对应更激进的控制增益 $K=R^{-1}B^\top P$。

**对偶单调性**：增大控制权重 $R_1\preceq R_2$（控制更"贵"），则 $P_1^\star\succeq P_2^\star$（最优代价反而更大，因为控制受限）。控制变"贵"→ 不敢大力控制 → 状态偏差累积更多 → 总代价更高。

这一单调性在工程调参中极有用：**增大 $Q$ 的对角元素总会使对应状态的增益变大**——不会出现"增大惩罚反而增益减小"的反直觉现象。

##### Riccati 解的极值性质

**定理（最大半正定解）**：CARE 的稳定化解 $P^\star$ 是所有对称半正定解中的**最大**者。即若 $P'\succeq0$ 满足 CARE，则 $P'\preceq P^\star$。

**证明思路**：取任何半正定解 $P'$，对应的"反馈"$K'=R^{-1}B^\top P'$ 不一定稳定化系统。但 $P'$ 仍可解释为某种代价函数（可能发散的）的"二次近似"。稳定化解 $P^\star$ 对应的代价是从稳定轨迹积分而来（有限值），它必然是所有解中最大的——因为它"看到了"最多的未来代价。

**如果不这样会怎样**（反事实推理）：如果我们选取 CARE 的一个非稳定化解 $P'\prec P^\star$ 作为反馈增益基础，得到的 $K'=R^{-1}B^\top P'$ 不稳定化系统。闭环在某些方向发散——$P'$ 没有"预见"到这些方向的无穷代价，所以它比 $P^\star$ 小。这就是为什么数值求解时必须选取**稳定不变子空间**对应的解。

##### ARE 解的个数：Hamiltonian 矩阵的几何图景

CARE 可能有多达 $2^n$ 个对称解（对应 Hamiltonian 矩阵 $2n$ 个特征值中选 $n$ 个的组合方式）。每种选法对应一个不同的 $n$ 维不变子空间，每个不变子空间给出一个 Riccati 解。

| 选取的子空间 | 对应的 Riccati 解 | 闭环性质 |
|---|---|---|
| 稳定子空间（所有 $\mathrm{Re}(\mu)<0$ 的特征值） | $P^\star$（最大正半定解） | $A_{cl}$ Hurwitz（稳定） |
| 反稳定子空间（所有 $\mathrm{Re}(\mu)>0$ 的特征值） | $P_{\min}$（最小正半定解） | $A_{cl}$ 所有特征值 $\mathrm{Re}>0$（不稳定） |
| 混合子空间（部分稳定+部分不稳定） | 中间解 | 闭环部分稳定、部分不稳定 |

在标准 LQR 中我们只关心稳定子空间对应的 $P^\star$。但在 $H_\infty$ 控制和博弈论 Riccati 中，有时需要考虑其他子空间——例如当 $\gamma$ 参数跨越临界值时，稳定子空间的维数发生变化，对应着博弈的可解性边界。

##### Riccati 方程与 Lyapunov 方程的层级关系

三类矩阵方程形成清晰的层级：

| 方程 | 形式 | 线性/非线性 | 物理含义 |
|------|------|-----------|---------|
| Sylvester | $AX+XB=C$ | 线性 | 两个系统间的耦合 |
| Lyapunov | $A^\top P+PA=-Q$ | 线性（Sylvester 特例） | 给定控制器的闭环代价 |
| Riccati | $A^\top P+PA-PBR^{-1}B^\top P+Q=0$ | **二次非线性** | **最优**控制器的闭环代价 |

回顾专题 3.7（Lyapunov 稳定性理论 §7.6）：Lyapunov 方程 $A^\top P+PA=-Q$ 的正定解 $P$ 存在当且仅当 $A$ Hurwitz，且此时 $V(x)=x^\top Px$ 正好是线性系统 $\dot x=Ax$ 的 Lyapunov 函数。Riccati 方程在此基础上多了一个 $-PBR^{-1}B^\top P$ 项——这正是"优化"的代价。如果我们固定某个反馈 $K$（不做优化），CARE 退化为 Lyapunov 方程 $A_K^\top P_K+P_K A_K+Q+K^\top RK=0$。Newton-Kleinman 迭代正是利用这一层级关系：在当前 $K_k$ 处解线性 Lyapunov（快！），然后更新 $K_{k+1}$。

#### §3.5.4c 跟踪问题与仿射 LQR（LQT）⭐⭐

##### 动机：实际系统需要跟踪参考轨迹，不是调节到零

标准 LQR 把状态调节到零——但实际机器人需要跟踪时变参考 $r(t)$（如四旋翼跟踪路径、机械臂跟踪关节轨迹）。**线性二次跟踪器（LQT）**把代价改为：
$$J=\tfrac12\int_0^T\bigl((x-r)^\top Q(x-r)+u^\top Ru\bigr)dt+\tfrac12(x(T)-r(T))^\top Q_f(x(T)-r(T)).$$

##### 完整推导

**Step 1：定义误差 $e=x-r$**。动力学变为 $\dot e=Ae+Bu+(Ar-\dot r)$。定义仿射项 $d(t)=Ar(t)-\dot r(t)$（如果 $r$ 恰好是无控制自由响应则 $d=0$）。

**Step 2：值函数变为仿射二次型**。设 $V(e,t)=\tfrac12 e^\top P(t)e+e^\top g(t)+c(t)$。代入 HJB 方程后配方。

**Step 3：最优控制的仿射形式**。
$$u^\star(t)=-R^{-1}B^\top P(t)e-R^{-1}B^\top g(t)=-K(t)(x-r)-R^{-1}B^\top g(t),$$
其中 $P(t)$ 仍满足标准 RDE（与参考无关！），$g(t)$ 满足**线性倒向 ODE**：
$$-\dot g=(A-BR^{-1}B^\top P)^\top g+Pd,\qquad g(T)=Q_f(x_f-r(T))=0\text{ (若终端匹配)}.$$

**工程意义**：跟踪 LQR 分解为两部分——
1. **反馈项** $-K(t)(x-r)$：抑制偏差，增益由 Riccati 方程决定
2. **前馈项** $-R^{-1}B^\top g(t)$：预补偿参考轨迹的变化，由线性 ODE 决定

前馈项是 LQT 相比 LQR 的本质新增——没有它，跟踪会有**持续稳态误差**（特别是对快速变化的参考）。自动驾驶横向控制中的"前瞻量"本质上就是这个 $g(t)$。

##### 无穷时域跟踪的稳态解

当参考为常值 $r=\mathrm{const}$、时域无穷时，$P=P^\star$（ARE 解），$g$ 满足稳态线性方程：
$$(A-BR^{-1}B^\top P^\star)^\top g_\infty=-P^\star(Ar-\dot r)=-P^\star Ar.$$
由于 $A_{cl}=A-BK^\star$ Hurwitz，转置 $A_{cl}^\top$ 也 Hurwitz，方程有唯一解。最终稳态前馈：
$$u_{ff}=-R^{-1}B^\top g_\infty.$$

#### §3.5.4d 典型例题：倒立摆 LQR 设计全流程 ⭐⭐

##### 系统建模

考虑经典的小车-倒立摆（Cart-Pole）系统。设小车质量 $M=1\,\text{kg}$，摆杆质量 $m=0.1\,\text{kg}$，摆杆半长 $l=0.5\,\text{m}$，重力 $g=9.81\,\text{m/s}^2$。

状态向量 $x=[p,\dot p,\theta,\dot\theta]^\top$（小车位置、速度、摆角、角速度），控制 $u$ 为施加在小车上的力。

非线性动力学在竖直平衡点 $(\theta=0,\dot\theta=0)$ 线性化：
$$A=\begin{pmatrix}0&1&0&0\\0&0&-mg/M&0\\0&0&0&1\\0&0&(M+m)g/(Ml)&0\end{pmatrix},\quad B=\begin{pmatrix}0\\1/M\\0\\-1/(Ml)\end{pmatrix}.$$

代入数值：
$$A=\begin{pmatrix}0&1&0&0\\0&0&-0.981&0\\0&0&0&1\\0&0&21.582&0\end{pmatrix},\quad B=\begin{pmatrix}0\\1\\0\\-2\end{pmatrix}.$$

##### Step 1：验证可控性

可控性矩阵 $\mathcal C=[B,AB,A^2B,A^3B]$：
$$\mathcal C=\begin{pmatrix}0&1&0&-0.981\\1&0&-0.981&0\\0&-2&0&43.164\\-2&0&43.164&0\end{pmatrix}.$$
$\mathrm{rank}(\mathcal C)=4=n$，系统完全可控。

##### Step 2：选择权重矩阵

选择 $Q=\mathrm{diag}(10,1,100,1)$（摆角偏差惩罚最大，位置偏差次之），$R=0.1$（控制成本低，允许较大力）。

权重选择的物理直觉：
- $Q_{33}=100\gg Q_{11}=10$：保持摆杆竖直比精确定位更重要（安全优先）
- $R=0.1<1$：我们"舍得"用力——实际电机足够强，不是瓶颈

##### Step 3：求解 CARE

用 Python 实现：
```python
import numpy as np
from scipy.linalg import solve_continuous_are

# 系统参数
M, m, l, g = 1.0, 0.1, 0.5, 9.81
A = np.array([[0, 1, 0, 0],
              [0, 0, -m*g/M, 0],
              [0, 0, 0, 1],
              [0, 0, (M+m)*g/(M*l), 0]])
B = np.array([[0], [1/M], [0], [-1/(M*l)]])
Q = np.diag([10, 1, 100, 1])
R = np.array([[0.1]])

# 求解 ARE
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ P)  # 最优增益 K = R^{-1} B^T P

print(f"P =\n{P}")
print(f"K = {K}")
print(f"闭环极点: {np.linalg.eigvals(A - B @ K)}")
```

##### Step 4：验证闭环稳定性

计算闭环极点 $\sigma(A-BK)$。典型结果（取决于精确参数）：所有极点实部为负，确认 Hurwitz 稳定。

##### Step 5：物理解读增益矩阵

假设得到 $K\approx[10.0,\ 6.8,\ 72.3,\ 16.5]$（数值示例）。各分量含义：
- $K_1=10.0$：位置偏差 1m → 施力 10N（把小车拉回来）
- $K_2=6.8$：速度 1m/s → 施力 6.8N（阻尼，防止过冲）
- $K_3=72.3$：摆角偏差 1rad → 施力 72.3N（**最大增益！** 摆角是最敏感状态）
- $K_4=16.5$：角速度 1rad/s → 施力 16.5N（角速度阻尼）

**如果不这样设计会怎样**（反事实）：若取 $Q=I$（不重视摆角），得到的 $K_3$ 会小很多，系统对微小角度扰动的恢复力不足——摆杆在大扰动下倒塌。这就是为什么工程中 $Q$ 的对角元选择必须反映物理优先级。

##### ⚠️ 倒立摆 LQR 的常见陷阱

**编程陷阱**：线性化时把 $\sin\theta\approx\theta$ 的有效范围估计过大。当 $|\theta|>15°$（$\approx0.26\,\text{rad}$）时，$\sin\theta$ 与 $\theta$ 的误差超过 3%，LQR 增益不再有效。解决：限制初始扰动，或使用 TVLQR/iLQR 处理大角度。

**概念误区**：认为"增大 $R$ 就能省电"。增大 $R$ 确实使控制力减小，但响应变慢——如果响应过慢，摆杆可能在恢复前就超出线性化有效范围而倒塌。$R$ 的选择必须保证闭环带宽高于系统的自然不稳定频率 $\sqrt{g/l}\approx4.4\,\text{rad/s}$。

#### §3.5.4e 典型例题：卫星姿态控制 LQR ⭐⭐⭐

##### 问题描述

考虑三轴稳定卫星的姿态控制。绕惯性主轴的 Euler 方程在小角度下线性化为：
$$I_x\ddot\phi=\tau_x,\quad I_y\ddot\theta=\tau_y,\quad I_z\ddot\psi=\tau_z,$$
其中 $(\phi,\theta,\psi)$ 为滚转/俯仰/偏航角，$I_x,I_y,I_z$ 为主惯量，$\tau$ 为反作用轮力矩。

状态 $x=[\phi,\dot\phi,\theta,\dot\theta,\psi,\dot\psi]^\top\in\mathbb R^6$，控制 $u=[\tau_x,\tau_y,\tau_z]^\top\in\mathbb R^3$。

$$A=\begin{pmatrix}0&1&&&\\ &&0&1&&\\ &&&&0&1\\ \end{pmatrix}_{6\times6}\text{ (块对角)},\quad B=\begin{pmatrix}0&0&0\\1/I_x&0&0\\0&0&0\\0&1/I_y&0\\0&0&0\\0&0&1/I_z\end{pmatrix}.$$

##### 设计要点

- **$Q$ 选择**：指向精度要求 $|\phi|<0.01°$ → 大 $Q_{11}$；角速度约束来自星敏感器曝光 $|\dot\phi|<0.001°/s$ → 大 $Q_{22}$
- **$R$ 选择**：反映反作用轮力矩限制。实际卫星 $|\tau_{\max}|\approx0.01\,\text{Nm}$，选 $R$ 使稳态增益不超出此范围
- **多输入优势**：三轴独立时 LQR 退化为三个标量 LQR，但当存在陀螺耦合时（$A$ 有非对角元素），多输入 LQR 自动生成**交叉耦合补偿**

> **跨领域类比**：卫星姿态 LQR 与四旋翼内环控制在数学上完全同构——都是双积分器 + 惯量归一化。区别仅在物理约束：卫星的 $R$ 由反作用轮力矩限制决定，四旋翼的 $R$ 由电机推力饱和决定。

---

#### §3.5.5 PMP 视角：Hamiltonian 矩阵与 Ansatz $\lambda=Px$

Pontryagin 最大值原理（专题 3.2）给出共态方程：
$$\dot\lambda=-\frac{\partial H}{\partial x}=-A^\top\lambda-Qx,\qquad u^\star=\arg\min_u H=-R^{-1}B^\top\lambda.$$
整理得到 $2n$ 维线性系统：
$$\begin{pmatrix}\dot x\\\dot\lambda\end{pmatrix}=\underbrace{\begin{pmatrix}A & -BR^{-1}B^\top\\ -Q & -A^\top\end{pmatrix}}_{H\ (\text{Hamiltonian matrix})}\begin{pmatrix}x\\\lambda\end{pmatrix}.$$

##### Hamiltonian 矩阵的辛结构与特征值对称性

**辛矩阵定义**：令 $J=\bigl(\begin{smallmatrix}0&I\\-I&0\end{smallmatrix}\bigr)\in\mathbb R^{2n\times 2n}$。矩阵 $H\in\mathbb R^{2n\times 2n}$ 是 **Hamiltonian 的**（属于辛李代数 $\mathfrak{sp}(2n)$），若 $JH+H^\top J=0$，即 $(JH)^\top=JH$（$JH$ 对称）。

**验证**：令 $S=BR^{-1}B^\top\succeq0$。LQR 的 Hamiltonian 矩阵为 $H=\begin{pmatrix}A&-S\\-Q&-A^\top\end{pmatrix}$。计算：
$$JH=\begin{pmatrix}0&I\\-I&0\end{pmatrix}\begin{pmatrix}A&-S\\-Q&-A^\top\end{pmatrix}=\begin{pmatrix}-Q&-A^\top\\-A&S\end{pmatrix},$$
这显然是对称的（$(-Q)^\top=-Q$，$S^\top=S$，右下角 $S$ 对称，非对角块互为转置）。所以 $H$ 确实是 Hamiltonian 的。

**特征值定理**：Hamiltonian 矩阵的特征值关于虚轴对称——若 $\mu$ 是特征值，则 $-\bar\mu$ 也是（对实矩阵则简化为：若 $\mu$ 是特征值，则 $-\mu$ 也是）。

**证明**：设 $Hv=\mu v$。由 $(JH)^\top=JH$，有 $H^\top J^\top=JH$，即 $H^\top(-J)=JH$，所以 $-H^\top=JHJ^{-1}$。因此 $H$ 与 $-H^\top$ 相似（$J$ 为相似变换），同谱。$-H^\top$ 的特征值是 $H^\top$ 的特征值取负，即 $-\bar\mu$（实矩阵时 $\bar\mu=\mu$ 故为 $-\mu$）。$\blacksquare$

**推论**：在 CARE 良定条件下（$(A,B)$ 可稳定化 + $(A,Q^{1/2})$ 可检测），$H$ 没有纯虚轴特征值，所以恰有 $n$ 个特征值在开左半平面，$n$ 个在开右半平面。

##### 稳定不变子空间与 Riccati 解的几何构造

设 $H$ 的 $n$ 个稳定特征值对应的**稳定不变子空间**为 $\mathcal V_-\subset\mathbb R^{2n}$，维数为 $n$。将 $\mathcal V_-$ 的基写为 $2n\times n$ 矩阵 $\begin{pmatrix}X_1\\X_2\end{pmatrix}$，其中 $X_1,X_2\in\mathbb R^{n\times n}$。

**关键定理（稳定不变子空间表示）**：若 $X_1$ 可逆，则 $P^\star=X_2 X_1^{-1}$ 是 CARE 的唯一稳定化解。

**证明**：不变子空间满足 $H\begin{pmatrix}X_1\\X_2\end{pmatrix}=\begin{pmatrix}X_1\\X_2\end{pmatrix}\Lambda$，$\Lambda$ 的特征值全在左半平面。展开：
$$AX_1-SX_2=X_1\Lambda,\qquad -QX_1-A^\top X_2=X_2\Lambda.$$
令 $P=X_2 X_1^{-1}$。从第一式 $A-SP=X_1\Lambda X_1^{-1}$，即闭环 $A-BK$ 与 $\Lambda$ 相似（$K=R^{-1}B^\top P$），因此 Hurwitz。从两式消 $\Lambda$：将第一式乘 $X_1^{-1}$ 得 $A-SP=X_1\Lambda X_1^{-1}$；第二式乘 $X_1^{-1}$ 得 $-Q-A^\top P=PX_1\Lambda X_1^{-1}=P(A-SP)$。展开后得 $A^\top P+PA-PSP+Q=0$，即 CARE。$\blacksquare$

> **本质洞察**：Riccati 的 Ansatz $\lambda=Px$ 不是天才的"猜测"——它是辛几何的必然：稳定不变子空间是 $2n$ 维相空间中的 $n$ 维 Lagrangian 子空间，而 Lagrangian 子空间恰好可以表示为"图"（graph）$\{(x,Px):x\in\mathbb R^n\}$。Riccati 方程就是 Lagrangian 子空间在 Hamiltonian 流下演化的方程。

##### 矩阵符号函数法（Matrix Sign Function）

**Roberts 1980** 提出了一种优雅的迭代方法：对 Hamiltonian 矩阵 $H$ 定义**矩阵符号函数** $\mathrm{sign}(H)=H(H^2)^{-1/2}$，其特征值为原矩阵特征值的符号（$+1$ 或 $-1$）。迭代格式：
$$H_0=H,\qquad H_{k+1}=\tfrac12(H_k+H_k^{-1}).$$
收敛后 $H_\infty=\mathrm{sign}(H)$，稳定投影为 $\Pi_-=\tfrac12(I-\mathrm{sign}(H))$。从 $\Pi_-$ 提取稳定不变子空间基底即可算出 $P^\star$。

**优势**：只用矩阵求逆，适合并行计算和大规模稀疏系统。**Roberts-Higham 改进**把非对称逆替换为对称逆，提升数值稳定性。这是 §3.5.I 中介绍的数值方法之一，在嵌入式 MPC 的内部 Riccati 递推中（acados/HPIPM）被广泛使用。

**Laub 1979 的 Schur 方法**（见 §3.5.I）是对稳定不变子空间做数值构造的另一经典方法：对 $H$ 做实 Schur 分解 $H=USU^\top$，按特征值实部排序使左上 $n\times n$ 块对应稳定子空间，然后从 $U$ 的前 $n$ 列提取 $X_1,X_2$。这是 MATLAB `care` 和 SciPy `solve_continuous_are` 的默认算法。

#### §3.5.5b LQR 的经典鲁棒裕度：回差不等式 ⭐⭐

##### 为什么 LQR 天生鲁棒

LQR 最令人惊叹的性质之一是：**不需要额外的鲁棒设计，单输入 LQR 就自带 $\ge60°$ 相位裕度和 $[1/2,\infty)$ 增益裕度**。这不是巧合，而是 Riccati 方程结构的深刻后果。

##### 回差不等式的完整证明

**定义**：回差函数（return difference）为 $F(s)=I+K(sI-A)^{-1}B$，其中 $K=R^{-1}B^\top P$ 是 LQR 增益。$F(j\omega)$ 在频率 $\omega$ 处衡量"闭环相对于开环的改善"。

**定理（Kalman 1964）**：对 LQR 最优反馈，回差满足
$$F(j\omega)^* R F(j\omega)\succeq R,\quad\forall\omega\in\mathbb R,$$
即 $\|R^{1/2}F(j\omega)R^{-1/2}\|\ge1$。单输入时简化为 $|1+K(j\omega I-A)^{-1}B|\ge1$。

**证明**：CARE 可改写为 $A_{cl}^\top P+PA_{cl}+Q+K^\top RK=0$，其中 $A_{cl}=A-BK$。将 Lyapunov 方程两边乘 $(j\omega I-A_{cl})^{-1}$ 和 $(-j\omega I-A_{cl}^\top)^{-1}$ 后求迹，利用 $F(j\omega)=I+K(j\omega I-A)^{-1}B$ 以及 $K=R^{-1}B^\top P$，经过代数操作（详见 Anderson-Moore Ch.11 或 Zhou-Doyle-Glover §14.4）得：
$$F(j\omega)^*RF(j\omega)=R+B^\top(−j\omega I−A^\top)^{−1}Q(j\omega I−A)^{−1}B\succeq R.$$

**推论（单输入裕度）**：当 $m=1$（单输入），$R$ 为正标量 $r$，回差不等式化为 $|F(j\omega)|^2\ge1$，即 Nyquist 图上 $L(j\omega)=K(j\omega I-A)^{-1}B$ 永远不进入以 $-1$ 为圆心、半径为 1 的**单位圆**。这一几何条件立刻给出：
- **增益裕度**：$L$ 可以乘以任何 $k\in[1/2,\infty)$ 而不穿过 $-1$，即 $\text{GM}\in[1/2,\infty)$（6 dB 下限、无上限）。
- **相位裕度**：$|1+L(j\omega)|=1$ 对应 $L$ 在单位圆上，此时相角至少 $\pm60°$，即 $\text{PM}\ge60°$。

##### 反事实：如果不满足回差不等式会怎样

考虑一个简单的比例控制器 $u=-kx$（不是 LQR 最优的）。它的回差 $F(j\omega)=1+k/(j\omega-a)$。如果 $k$ 选得不好，$|F|$ 可以在某些频率小于 1——此时系统对该频率的增益扰动"脆弱"。**LQR 通过 Riccati 方程的结构自动避免了这一情况**——这是最优性赋予的"免费午餐"。

但需要强调：**多输入 LQR 不保证有同样好的裕度**——Safonov-Athans 1977 证明多输入 LQR 仍满足回差不等式（矩阵版本），但对单个通道的裕度需要额外分析。而且这些裕度只针对**回路断开点**处的不确定性，对参数不确定性（如质量变化）没有直接保证。

#### §3.5.6 可控性、可观性与 Gramian

**可控性 Gramian** $W_c(T)=\int_0^T e^{A\tau}BB^\top e^{A^\top\tau}d\tau$ 满足 Lyapunov 方程 $AW_c+W_c A^\top+BB^\top=0$（无穷时域稳定系统）。**可观性 Gramian** $W_o$ 对偶定义。

**Hautus / PBH 判据**：$(A,B)$ 可控 $\Leftrightarrow$ $\operatorname{rank}[sI-A\ |\ B]=n,\ \forall s\in\mathbb C$；可稳定化只需 $s$ 在闭右半平面。对偶版本给出可观/可检测。

**与 Riccati 的桥梁**：当 $Q\succ0$、$R\succ0$ 时 CARE 解正定，但实际工程常取 $Q=C^\top C\succeq0$，此时 $P^\star\succ0$ 要可检测 $(A,C)$。换言之，**Gramian 与 Riccati 同属"能量沿动力学传播"的二次型**，Gramian 描述开环能量，Riccati 描述闭环最优代价。

#### §3.5.7 LQG 与分离原理

**LQG 问题**：
$$\dot x=Ax+Bu+w,\qquad y=Cx+v,\qquad w\sim\mathcal N(0,\Sigma_w),\ v\sim\mathcal N(0,\Sigma_v),$$
最小化 $\mathbb E\!\int(x^\top Qx+u^\top Ru)\,dt$。未知的是状态 $x$，只能观测 $y$。

**分离原理（separation principle）**：最优控制器等于
1. **Kalman 滤波器**产生状态的条件均值 $\hat x=\mathbb E[x\mid y_{[0,t]}]$（见 §3.5.8）；
2. **LQR 反馈** $u=-K\hat x$，其中 $K$ 是假设状态可测时的 LQR 增益。

**完整证明思路**（确定性等价, certainty equivalence）：取 $V(x,t)=\tfrac12\hat x^\top P\hat x+\tfrac12\operatorname{tr}(P\Sigma)+c(t)$，其中 $\Sigma$ 是滤波协方差。HJB 在高斯-线性-二次框架下可显式求解，且最小化项只依赖于 $\hat x$。完整证明见 Åström《Introduction to Stochastic Control》Ch.8 或 Anderson-Moore Ch.7。

##### 分离原理的完整证明

分离原理的证明是 LQG 理论最核心的推导。其精妙之处在于把**部分可观**的随机最优控制问题拆解为两个**完全可解**的子问题。

**证明（确定性等价，Certainty Equivalence）**：

**Step 1：正交分解。** 令估计误差 $\tilde x=x-\hat x$，其中 $\hat x=\mathbb E[x\mid\mathcal Y_t]$（给定观测历史 $\mathcal Y_t=\{y(\tau):\tau\le t\}$ 的条件均值）。由正交投影定理，$\tilde x$ 与 $\hat x$ 不相关：$\mathbb E[\tilde x\hat x^\top]=0$。

**Step 2：代价分解。** 代价中的状态二次型可分解：
$$\mathbb E[x^\top Qx]=\mathbb E[(\hat x+\tilde x)^\top Q(\hat x+\tilde x)]=\mathbb E[\hat x^\top Q\hat x]+\mathbb E[\tilde x^\top Q\tilde x]+2\underbrace{\mathbb E[\hat x^\top Q\tilde x]}_{=0}.$$
因此总代价：
$$J=\mathbb E\int_0^\infty(\hat x^\top Q\hat x+u^\top Ru)dt+\underbrace{\mathbb E\int_0^\infty\tilde x^\top Q\tilde x\,dt}_{\text{仅取决于滤波器，与 }u\text{ 无关}}.$$

**Step 3：估计误差独立于控制。** 对线性高斯系统，Kalman 滤波器的误差协方差 $\Sigma(t)=\mathbb E[\tilde x\tilde x^\top]$ 满足 Riccati 方程 $\dot\Sigma=A\Sigma+\Sigma A^\top-\Sigma C^\top\Sigma_v^{-1}C\Sigma+\Sigma_w$，其中**不含 $u$**（这是线性-高斯假设的关键后果——控制输入不影响估计精度）。因此第二项是常数，最优化只需处理第一项。

**Step 4：对 $\hat x$ 的确定性等价。** $\hat x$ 的动力学为 $\dot{\hat x}=A\hat x+Bu+L(y-C\hat x)$。第一项 $\int(\hat x^\top Q\hat x+u^\top Ru)dt$ 恰好是以 $\hat x$ 为状态、$u$ 为控制的**确定性 LQR 问题**（$\hat x$ 的"噪声"来自滤波器创新过程，但期望代价中创新项贡献为零）。

**Step 5：结论。** 最优 $u=-K\hat x$，$K=R^{-1}B^\top P^\star$（LQR 增益），$\hat x$ 由 Kalman 滤波器产生。两个子问题**独立设计**，组合后仍全局最优。$\blacksquare$

**为什么分离原理只在线性-高斯-二次下严格成立**：非线性系统中 $\mathbb E[\tilde x\hat x^\top]\ne0$（估计误差与状态估计相关），代价无法正交分解；非高斯噪声中条件分布不由均值和协方差完全表征，最优估计不是 Kalman 滤波器。SLAM 中常用的 EKF + LQR 组合是**近似分离**——在线性化误差小时有效，大扰动下可能失效。

##### LQG 的致命短板：Doyle 1978

**分离原理保证性能最优（均方代价），但不保证鲁棒裕度**。LQR 自带的 $\geq60°$ 相位裕度、$[1/2,\infty)$ 增益裕度，**在 LQG 中可以坍缩到任意小**。

**Doyle 1978 的构造**：考虑标量系统 $\dot x=ax+bu+w$，$y=cx+v$，选择 $a>0$（不稳定）、$\Sigma_w$ 大（过程噪声强）、$\Sigma_v$ 小（测量噪声弱）。Kalman 增益 $L$ 很大（几乎全信观测），LQG 的回路传递函数变为 $K(sI-A+LC)^{-1}B$，其中观测器极点 $A-LC$ 深入左半平面。但在回路断开点看，高增益观测器引入的相位延迟可以使相位裕度任意接近零。

"There are none."——Doyle 论文摘要仅此四字，成为控制史上最短最震撼的摘要之一。这四个字的意思是：**LQG 控制器没有保证的稳定裕度（guaranteed margins are none）**。

**物理根源**：Kalman 滤波器是"开环观测器"——它基于标称模型 $(A,C)$ 构建，遇到参数失配时没有内在的修正机制。而 LQR 的鲁棒性来源于反馈的"回差性质"$|1+L(j\omega)|\ge1$，这一性质在加入观测器后被观测器极点破坏。

**三条解药**：(i) **LTR**（§3.5.B）通过虚拟过程噪声恢复 LQR 裕度；(ii) **$H_\infty$ 综合**（§3.5.A）直接对抗最坏扰动——把 LQR 的 $H_2$ 代价升级为 minimax 博弈，用博弈型 Riccati 方程求解；(iii) **$\mu$-synthesis**（专题 3.6 §3.6.10）处理**结构化**不确定性——当不确定性具有块对角结构时，$H_\infty$ 范数过于保守，结构奇异值 $\mu$ 给出更紧的鲁棒性度量，D-K 迭代在频率依赖的 D-scaling 和控制器 $K$ 之间交替优化。

#### §3.5.8 Kalman–LQR 对偶性

Kalman 滤波器的误差协方差 $\Sigma(t)$ 满足**滤波 Riccati 方程**：
$$\dot\Sigma=A\Sigma+\Sigma A^\top-\Sigma C^\top\Sigma_v^{-1}C\Sigma+\Sigma_w.$$
Kalman 增益 $L=\Sigma C^\top\Sigma_v^{-1}$。与控制 Riccati 对比：

| 控制（LQR）         | 估计（Kalman）        |
|---------------------|------------------------|
| $A$                 | $A^\top$               |
| $B$                 | $C^\top$               |
| $Q$                 | $\Sigma_w$             |
| $R$                 | $\Sigma_v$             |
| $P^\star$ 值函数    | $\Sigma^\star$ 协方差  |
| $K=R^{-1}B^\top P$  | $L=\Sigma C^\top\Sigma_v^{-1}$ |
| 稳定化条件 $(A,B)$ stab. | 可检测 $(A,C)$ det. |

**严格对偶陈述**：若 $P$ 是系统 $(A,B,Q,R)$ 的 CARE 解，则 $\Sigma:=P$ 是系统 $(A^\top,C^\top,\Sigma_w,\Sigma_v)$ 的 CARE 解——这是 Kalman 1960 两篇奠基论文（控制一篇、滤波一篇）之间的镜面关系。**对偶不仅是形式上的**：它意味着一套数值代码就能同时求解控制和估计问题。

##### 对偶性的深层几何意义

对偶性不是偶然的符号巧合，而是反映了一个深刻的物理事实：**控制是从现在向未来"注入信息"（通过 $u$），估计是从过去向现在"提取信息"（通过 $y$）**。两者都是沿着时间轴的"信息传播"，只是方向相反。在 Hamiltonian 框架下：

- LQR 的 Hamiltonian：状态 $x$ 正向演化，协态 $\lambda$ 倒向演化。
- Kalman 的 Hamiltonian：误差协方差 $\Sigma$ 正向演化，信息矩阵 $\Sigma^{-1}$ 倒向演化。

时间反转 $t\mapsto-t$ 把一个变成另一个。这就是为什么同一个 Schur 分解算法（§3.5.I）既能解控制 Riccati，也能解滤波 Riccati——数值上就是换一下矩阵参数的事。

> **本质洞察**：LQR-Kalman 对偶是"控制即反向估计"的数学表述。你可以把 LQR 理解为"从未来的终端代价向现在传播最优策略"，把 Kalman 理解为"从过去的初始不确定性向现在传播最优估计"。两者是同一枚硬币的正反面。

##### 对偶性在工程中的直接应用

1. **代码复用**：Python 中 `scipy.linalg.solve_continuous_are(A, B, Q, R)` 既用于 LQR（传入 $A,B,Q,R$），也用于 Kalman（传入 $A^\top,C^\top,\Sigma_w,\Sigma_v$）——因为是同一个方程。
2. **条件迁移**：LQR 的可稳定化条件 $(A,B)$ 对偶地变成 Kalman 的可检测条件 $(A,C)$。如果你的 SLAM 系统状态不可观（如单目尺度），对偶地看就是"控制不到"——滤波器的某些误差模态无法收敛。
3. **增益关系**：LQR 增益 $K=R^{-1}B^\top P$ 对偶于 Kalman 增益 $L=\Sigma C^\top\Sigma_v^{-1}$。增大过程噪声 $\Sigma_w$（对偶于增大状态权重 $Q$）会让 Kalman 增益变大（更相信观测），对偶地 LQR 增益变大（更积极控制）。

#### §3.5.9 有限时域→无穷时域的极限：收敛速率与指数稳定性 ⭐⭐⭐

##### 动机：MPC 为什么需要"足够长"的 horizon

在 MPC 中，有限时域 LQR 的解 $P(t)$ 必须近似无穷时域的 $P^\star$，否则终端代价选取不当会导致闭环不稳定。理解 $P(t)\to P^\star$ 的**收敛速率**直接决定了 MPC 所需的最小预测步长。

##### 指数收敛定理

**定理（Anderson-Moore Ch.4）**：在 CARE 良定条件下，RDE 的解从任何初始条件 $P(T)=Q_f\succeq0$ 出发，以指数速率收敛到稳态：
$$\|P(t)-P^\star\|\le C\cdot e^{-2\alpha(T-t)},\quad t\le T,$$
其中 $\alpha=\min_i|\mathrm{Re}(\lambda_i(A_{cl}))|$ 是闭环系统的最慢衰减模态。

**物理解读**：$\alpha$ 越大（闭环越快），$P(t)$ 越快收敛到 $P^\star$。这意味着：快速闭环系统只需要短 horizon 的 MPC 就能达到近似最优；慢系统需要长 horizon。

**对 MPC 设计的直接指导**：如果闭环最慢极点为 $-\alpha$，则 horizon $T\ge 4/\alpha$ 保证 $\|P(0)-P^\star\|\le e^{-8}\|P(T)-P^\star\|\approx0.03\%$——此时有限时域 LQR 与无穷时域几乎无区别。

##### 终端代价选择的理论基础

MPC 稳定性理论（Mayne-Rawlings-Rao-Scokaert 2000）的核心定理：若终端代价 $V_f(x)=x^\top P^\star x$（ARE 解），终端约束 $x_N\in\Omega$（$P^\star$ 的某水平集），则 receding-horizon MPC 闭环渐近稳定。

**为什么选 $P^\star$ 而不是其他正定矩阵**：$P^\star$ 恰好使得 $V_f$ 是闭环系统在 horizon 外"余下"的最优 cost-to-go——选其他矩阵要么过于保守（$P>P^\star$，MPC 过于激进控制），要么不满足 Lyapunov 递减条件（$P<P^\star$，稳定性无保证）。

> **本质洞察**：有限时域 LQR 的 Riccati 解 $P(t)$ 向无穷时域 ARE 解 $P^\star$ 的收敛，本质上是动态规划的"值迭代"在连续时间的表现。每多积一步 $\Delta t$ 的 Riccati ODE，相当于多做一步 Bellman 更新——值函数逐步从终端条件"扩散"到全空间，直到达到不动点 $P^\star$。

##### 练习

1. 对倒立摆系统（§3.5.4d），用 Python 的 `solve_ivp` 倒向积分 RDE，绘制 $\|P(t)-P^\star\|_F$ 关于 $(T-t)$ 的曲线。验证指数收敛，并估算收敛时间常数 $1/\alpha$。
2. 改变终端条件 $Q_f=0$ vs $Q_f=P^\star$ vs $Q_f=10P^\star$，比较收敛速率。解释为什么 $Q_f=P^\star$ 使 $P(t)\equiv P^\star$（不动点性质）。

---

#### §3.5.10 本章小结

| 知识点 | 核心结果 | 关键条件 | 工程应用 |
|--------|---------|---------|---------|
| 有限时域 LQR（HJB） | RDE: $-\dot P=A^\top P+PA-PBR^{-1}B^\top P+Q$ | $Q\succeq0,R\succ0$ | 批次控制、轨迹跟踪 |
| 有限时域 LQR（DP） | 离散 DRE 反向递推 | 同上 | MPC 核心递推 |
| 完备平方法 | $J=\tfrac12 x_0^\top P_0 x_0+\text{非负项}$ | 同上 | 独立验证/充分性 |
| 无穷时域 LQR | CARE: $A^\top P+PA-PBR^{-1}B^\top P+Q=0$ | $(A,B)$ 可稳定化+$(A,Q^{1/2})$ 可检测 | 稳态调节器 |
| Hamiltonian 矩阵 | $P^\star=X_2 X_1^{-1}$（稳定不变子空间） | 无虚轴特征值 | Schur 算法基础 |
| LQR 鲁棒裕度 | $|1+L(j\omega)|\ge1$ → PM$\ge60°$, GM$\in[1/2,\infty)$ | 单输入 | 控制器鲁棒性评估 |
| LQG 分离原理 | 最优=$\text{Kalman}+\text{LQR}$，独立设计 | 线性+高斯+二次 | 部分可观系统 |
| Doyle 反例 | LQG 无保证裕度 | 任何 LQG | 需 LTR 或 $H_\infty$ 补救 |
| Kalman-LQR 对偶 | $(A,B,Q,R)\leftrightarrow(A^\top,C^\top,\Sigma_w,\Sigma_v)$ | 双方 CARE 互映 | 代码复用/条件迁移 |
| Newton-Kleinman | 解 Lyapunov → 更新 $K$ → 二次收敛 | 初始 $K_0$ 稳定化 | 大规模 ARE |
| 跟踪 LQR（LQT） | 反馈+前馈：$u=-K(x-r)-R^{-1}B^\top g$ | $g$ 满足线性倒向 ODE | 路径跟踪 |

---

### 🔧 故障排查手册（核心章节）

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| `solve_continuous_are` 报 LinAlgError | 系统不可稳定化 | 1. 检查 $(A,B)$ 可控性矩阵秩 2. 检查 PBH 条件 3. 检查 $R\succ0$ | §3.5.4 |
| LQR 闭环振荡不衰减 | $(A,Q^{1/2})$ 不可检测 | 1. 检查 $Q$ 是否惩罚了所有不稳定模态 2. 增大 $Q$ 对应元素 | §3.5.4 |
| 仿真中状态发散但 ARE 有解 | 线性化有效范围过小 | 1. 检查 $\|x\|$ 是否超出线性化邻域 2. 改用 TVLQR 或 iLQR | §3.5.4d |
| Riccati 倒向积分数值发散 | $\Delta t$ 太大或 stiff | 1. 减小积分步长 2. 使用隐式积分器（如 BDF） | §3.5.2 |
| LQG 控制器在真实系统上不稳定 | 模型参数失配，Doyle 效应 | 1. 计算回路传递函数裕度 2. 加 LTR 或换 $H_\infty$ | §3.5.7 |

---

### 二、进阶章节（档位 4）

#### §3.5.A $H_2/H_\infty$ 控制与零和博弈 Riccati

把 LQR 写作 $H_2$ 范数最小化：传递函数 $T_{zw}$ 从扰动 $w$ 到评价输出 $z=\bigl(\begin{smallmatrix}Q^{1/2}x\\R^{1/2}u\end{smallmatrix}\bigr)$，最小化 $\|T_{zw}\|_2$ 即 LQR。

**$H_\infty$ 控制**把这变成**零和微分博弈**——控制器 $u$ 最小化代价，自然界的"对手" $w$ 最大化代价：
$$\min_u\max_w\int_0^\infty\bigl(\|z\|^2-\gamma^2\|w\|^2\bigr)dt.$$

**博弈解释**：这是一个两人零和微分博弈。$\gamma$ 是"扰动衰减水平"——设计者承诺：无论对手 $w$ 怎么使坏，评价输出 $z$ 的能量不超过 $\gamma^2$ 倍的扰动能量。$\gamma$ 越小，设计者的承诺越强，控制器越"鲁棒"但越"保守"（控制能量越大）。

**博弈型 Riccati（game-theoretic ARE）**：假设扰动通过 $\dot x=Ax+Bu+Dw$ 进入，博弈的纳什均衡（鞍点策略）为：
$$u^\star=-R^{-1}B^\top Px,\qquad w^\star=\gamma^{-2}D^\top Px.$$
$P$ 满足：
$$A^\top P+PA+P(\gamma^{-2}DD^\top-BR^{-1}B^\top)P+Q=0.$$

**与 LQR 的精确关系**：当 $\gamma\to\infty$，$\gamma^{-2}DD^\top\to0$，博弈 Riccati 退化为标准 CARE——$H_\infty$ 是 LQR 的"最坏情况推广"。当 $\gamma$ 逐渐减小，$\gamma^{-2}$ 项的正贡献使 Riccati 右端越来越大，直到某个临界 $\gamma^\star$ 处 ARE 不再有正定解——这对应**系统无法抵抗这么强的扰动**。

**$\gamma$-迭代**：最小的 $\gamma^\star$ 使 ARE 存在稳定化正半定解，即为 $\|T_{zw}\|_\infty$——闭环从 $w$ 到 $z$ 的最坏增益。计算上用二分法搜索 $\gamma$，每次解一个 ARE 判断是否可行。

> **跨领域类比**：$H_\infty$ 设计如同博弈论中的 minimax 策略——不追求"平均情况最好"（那是 $H_2$/LQR），而是追求"最坏情况不太差"。这与 robust MDP（专题 3.6 §3.6.10）中的 $\min_\pi\max_{P\in\mathcal P}$ 是**完全相同的数学结构**——domain randomization 在 RL 中的数学根源就在这里。

**Doyle 1978 反例**：构造二阶系统使 LQG 相位裕度可小于 $\epsilon$。物理根源：Kalman 滤波器是"被扰动撕开"的开环积分，其极点被标称动力学放置但遇参数失配无补偿能力。解药：$H_\infty$ 综合（把扰动视作对手）或 LTR（§3.5.B）。详见 Zhou-Doyle-Glover Ch.14、Doyle-Francis-Tannenbaum。

**μ-synthesis**：对结构化不确定性（块对角 $\Delta$）用结构奇异值 $\mu$ 刻画鲁棒性，D-K 迭代综合控制器。MATLAB Robust Control Toolbox 有成熟实现。

##### $H_\infty$ 博弈 Riccati 的完整推导

**Step 1：定义 Isaacs 值函数**。对微分博弈 $\min_u\max_w\int_0^\infty(\|z\|^2-\gamma^2\|w\|^2)dt$，定义值函数 $V(x)=\min_u\max_w J_{[0,\infty]}$。

**Step 2：博弈 HJB（Isaacs 方程）**。假设 $V(x)=\tfrac12 x^\top Px$：
$$0=\min_u\max_w\{\tfrac12 x^\top Qx+\tfrac12 u^\top Ru-\tfrac12\gamma^2 w^\top w+x^\top P(Ax+Bu+Dw)\}.$$

**Step 3：鞍点条件**。对 $u$ 求最小：$u^\star=-R^{-1}B^\top Px$。对 $w$ 求最大：$w^\star=\gamma^{-2}D^\top Px$。

**Step 4：代回 Isaacs 方程**，提取 $P$ 系数得到博弈 ARE：
$$A^\top P+PA+Q-PBR^{-1}B^\top P+\gamma^{-2}PDD^\top P=0.$$

注意：$-PBR^{-1}B^\top P$ 是控制的"好处"（减小代价），$+\gamma^{-2}PDD^\top P$ 是对手的"破坏"（增大代价）。两者竞争决定了 ARE 是否有解——如果对手太强（$\gamma$ 太小），ARE 无正定解，意味着**系统无法保证这一鲁棒水平**。

##### $H_2$ vs $H_\infty$ 的工程决策

| 准则 | $H_2$（LQR） | $H_\infty$ |
|------|---|---|
| 优化目标 | 平均代价（期望值） | 最坏代价（sup over $w$） |
| 对扰动假设 | 白噪声（随机） | 有界能量（确定性最坏） |
| 保守程度 | 最低（如果模型精确） | 较高（但模型不确定时更安全） |
| 计算 | 一次 CARE | 带 $\gamma$-搜索的 CARE |
| 适用场景 | 模型精确、扰动统计已知 | 模型不确定、扰动统计未知 |

**工程决策规则**：如果你能精确建模系统并知道噪声统计 → 用 LQR/LQG。如果参数有 $>10\%$ 不确定性或环境变化剧烈 → 用 $H_\infty$。对大多数机器人系统（参数不确定但有界），$H_\infty$ 更安全。

#### §3.5.B Loop Transfer Recovery (LTR) ⭐⭐⭐

##### 动机：如何"修复"LQG 的鲁棒性

Doyle 1978 告诉我们 LQG 没有保证裕度。工程上最常用的修复方法是 **LTR（Loop Transfer Recovery）**：通过人为增大过程噪声，使 Kalman 增益变大，LQG 的回路传��函数渐近趋近 LQR 的回路传递函数——从而"恢复"LQR 的好裕度。

##### LTR 的数学机制

**步骤**：取 $\Sigma_w=\Sigma_{w,0}+qBB^\top$（增加虚拟过程噪声），$q\to\infty$。

当 $q\to\infty$ 时，Kalman 增益 $L=\Sigma C^\top\Sigma_v^{-1}$ 变大（因为 $\Sigma$ 在 $B$ 的列空间方向增大）。此时观测器变得"极不信任模型、极其信任观测"——$\hat x\approx C^{-1}y$（如果 $C$ 可逆）。

回路传递函数从 $L_{\text{LQG}}(s)=K(sI-A+BK+LC)^{-1}LC\cdot C(sI-A)^{-1}B$ 退化为 $L_{\text{LQR}}(s)=K(sI-A)^{-1}B$——因为观测器极点被推到远离虚轴处，不再影响回路特性。

##### LTR 的限制

1. **仅对最小相位系统有效**：如果开环有右半平面零点（非最小相位），LTR 无法完全恢复 LQR 裕度
2. **噪声放大**：增大虚拟 $\Sigma_w$ 使 Kalman 增益大 → 观测噪声 $v$ 被放大注入状态估计 → 估计噪声增大 → 控制信号抖动
3. **仅恢复输出端裕度**：LTR 恢复的是开环断开点的裕度，不是输入端裕度

**Anderson-Moore Part III** 有完整工程流程，包括如何选择 $q$ 的大小（trade-off：鲁棒性 vs 噪声性能）。

> **"不是X而是Y"句式**：LTR 不是让 LQG "变成" LQR——它是让 LQG 的**频率响应特性**近�� LQR，同时保留观测器的**状态估计功能**。代价是噪声性能退化和对最小相位的依赖。

#### §3.5.C SE(3) / 流形上的 LQR：TVLQR 与 iLQR

**TVLQR（Time-Varying LQR）—— Tedrake 流派**：给定标称轨迹 $(x^\star(t),u^\star(t))$（由 trajectory optimization 得到），在邻域内一阶 Taylor 展开动力学 $\delta\dot x=A(t)\delta x+B(t)\delta u$，时变二次代价下用**反向积分 RDE** 得 $P(t)$ 与时变反馈 $K(t)$。TVLQR 即用于**轨迹镇定**：非线性系统做轨迹跟踪时的局部稳定反馈。Tedrake《Underactuated Robotics》Ch.8 / Ch.10 是权威入门。

**iLQR（iterative LQR）—— Todorov 流派**：在当前轨迹附近做 LQR 反向 pass 得反馈，前向 roll-out 更新轨迹，迭代至收敛。**iLQR 的 backward pass 就是一次 TVLQR**；这就是为什么专题 3.9（iLQR/DDP）必须以 3.5 为前置。DDP 多一个二阶动力学项（exact Newton，收敛快但需要二阶导）。

**SO(3)/SE(3) 上的 LQR**：状态空间是流形，不能直接线性化。标准做法是 **tangent-space LQR**：在 $SO(3)$ 上用指数坐标 $\xi\in\mathfrak{so}(3)$，动力学在 Lie 代数里线性化 $\dot\xi=A(t)\xi+B(t)\delta u$，再用常规 TVLQR。四旋翼姿态稳定、机械臂末端跟踪均用此框架。参考 CMU 16-745（Manchester/Kuindersma）关于 quaternion + LQR 的四旋翼实验。

#### §3.5.D 随机 LQR：乘性噪声与平均场

**乘性噪声 LQR（Wonham 1968）**：$dx=(Ax+Bu)dt+\sum_i(C_ix+D_iu)\,dW_i$，即噪声强度随状态/控制放大。对应**广义 Riccati**：
$$A^\top P+PA+\sum C_i^\top PC_i-\bigl(PB+\sum C_i^\top PD_i\bigr)\bigl(R+\sum D_i^\top PD_i\bigr)^{-1}(\cdot)^\top+Q=0.$$
现实意义：执行器增益不确定、通道衰减等。

**平均场 LQR（Huang–Malhamé–Caines 2006）**：$N$ 个弱耦合代理，当 $N\to\infty$ 用代表性代理 + McKean-Vlasov 一致性方程得到**去中心化 $\varepsilon$-Nash 均衡**；是 MFG 理论两大源头之一（另一支为 Lasry-Lions）。应用：群体机器人控制、智能交通。

#### §3.5.E 分布鲁棒 LQR（Wasserstein 模糊集）

**Van Parys-Kuhn-Goulart-Morari (IEEE TAC 2016)**：噪声分布只知前两阶矩，用分布鲁棒 chance / CVaR 约束，仿射控制器综合凸化求解。

**Taşkesen-Iancu-Koçyiğit-Kuhn (NeurIPS 2023, arXiv:2305.17037)**：噪声分布在以高斯为圆心的 **Wasserstein 球**内，证明最优策略仍为**线性观测反馈**，用 Frank-Wolfe + Kalman 滤波高效求解。这是当前分布鲁棒 LQR 的最前沿成果，直接对接安全 RL 的分布偏移问题。

#### §3.5.F Policy Gradient on LQR：Fazel 2018 的里程碑

设参数化策略 $u=-Kx$，代价 $C(K)=\mathbb E_{x_0}\tfrac12 x_0^\top P_K x_0$，其中 $P_K$ 满足 $A_K^\top P_K+P_K A_K+Q+K^\top RK=0$（$A_K:=A-BK$）。

**Fazel-Ge-Kakade-Mesbahi (ICML 2018, arXiv:1801.05039)** 证明：
1. $C(K)$ **不凸、不拟凸、但具有梯度优势（gradient dominance / PL 条件）**：$C(K)-C(K^\star)\leq\mu\|\nabla C(K)\|^2$。
2. **无虚假局部极小**：每个驻点要么是全局最优，要么梯度能突破。
3. （投影）梯度下降 / Gauss-Newton / **自然策略梯度**（Fisher 矩阵预条件）**线性全局收敛**。
4. Model-free 估计梯度（zeroth-order 扰动）带多项式样本复杂度。

**自然 PG 的 Fisher 几何解释**：自然梯度 $\tilde\nabla=F^{-1}\nabla$，在 LQR 下 Fisher 信息恰与闭环 Gramian 成正比；自然 PG 等价于 **Gauss-Newton** on Riccati 残差——**RL 与经典 Kleinman 迭代（1968）相遇**。

意义：LQR 作为"RL 理论的果蝇"，是**第一个非凸连续控制问题被严格证明全局收敛**的案例，后续工作（Bu 等、Malik 等）推广到 $H_\infty$、LQG、risk-sensitive 等。

#### §3.5.G Model-Free LQR 与样本复杂度

**Dean-Mania-Matni-Recht-Tu (FoCM 2020, arXiv:1710.01688)** 提出 **Coarse-ID Control** 两阶段：
1. 用独立激励数据做**系统辨识**得 $(\hat A,\hat B)$ 与误差球 $\|\hat A-A\|,\|\hat B-B\|\leq\varepsilon$；
2. 在误差球内做**鲁棒综合**（基于 **System Level Synthesis, SLS**）得到可证明次优性 $O(\varepsilon)$ 的控制器。
端到端样本复杂度：**$\widetilde O((n+m)/\varepsilon^2)$** 样本保证 $\varepsilon$-次优。

**Abbasi-Yadkori-Szepesvári (COLT 2011)**：在线 adaptive LQ，基于自归一化高概率置信集与"面对不确定性的乐观"原则，首次证明 $\widetilde O(\sqrt{T})$ 悔界。计算上难（需解 OFU 优化），但理论界至今仍是标杆。

这两类工作使 LQR 成为**带理论保证的 data-driven 控制**的试金石，为 SLAM 里的在线辨识 + 控制、基于学习的 MPC 提供样本复杂度参考。

#### §3.5.H LQR 与 Transformer / In-Context Learning

**Garg-Tsipras-Liang-Valiant (NeurIPS 2022, arXiv:2208.01066)** 本身是关于线性回归 / 决策树 / 两层网络的 in-context learning，**并不直接针对 LQR**——任务描述稍有偏差。**真正把 transformer 与 LQR / Kalman 对接的是**：

- **Goel-Bartlett (L4DC 2024, arXiv:2312.06937)**"Can a Transformer Represent a Kalman Filter?"：显式构造权重证明因果 softmax self-attention 可一致近似任意可观 LTI 系统的 Kalman 滤波器，进一步推广到 LQG。
- **Akyürek 等 (ICLR 2023, arXiv:2211.15661)** 与 **von Oswald 等 (ICML 2023, arXiv:2212.07677)**：证明 transformer 在 ICL 中实现梯度下降 / 闭式岭回归，为 Goel-Bartlett 提供理论基石。

**惊人桥梁**：transformer 的前向推理等价于在上下文中执行一次经典控制算法。对机器人而言意味着：**预训练一次，在任务时只靠上下文就能适配新系统**——这为"foundation model for robotics / 具身智能"提供了严格数学依据。

#### §3.5.I Riccati 方程的数值方法

| 方法 | 原论文 | 适用 | 特点 |
|------|--------|------|------|
| Schur 分解 | Laub 1979（IEEE TAC 24(6), DOI 10.1109/TAC.1979.1102178） | CARE/DARE | 数值稳定，$O(n^3)$；MATLAB `care/dare`、SciPy `solve_*_are` 默认 |
| QZ 算法 | Arnold-Laub 1984 | 广义 ARE | 处理病态 $E\neq I$ 情形 |
| Newton 迭代 | Kleinman 1968（IEEE TAC 13(1), DOI 10.1109/TAC.1968.1098829） | ARE | 需初始稳定化 $K_0$；每步解 Lyapunov，二次收敛 |
| 辛 SDA（doubling） | Chu 等 2005 | 大规模稀疏 | 收敛快，适合 MPC 内部调用 |
| 矩阵符号函数 | Roberts 1980 | 并行计算 | 只用矩阵逆，适合 GPU |

##### Newton-Kleinman 迭代的完整推导与收敛证明 ⭐⭐⭐

**动机**：Schur 分解是"一步到位"的直接法，但对大规模系统（$n>100$）代价高昂。Newton-Kleinman 迭代是一种**策略迭代**方法——从一个初始稳定增益 $K_0$ 出发，反复求解 Lyapunov 方程（线性！），以二次速度收敛到 Riccati 解。这正是 RL 中 policy iteration 的连续时间版本。

**算法（Kleinman 1968）**：
1. **初始化**：选择 $K_0$ 使 $A_0=A-BK_0$ Hurwitz（可用极点配置或 LQR 粗解）。
2. **策略评估（Policy Evaluation）**：解 Lyapunov 方程
$$A_k^\top P_k+P_k A_k+Q+K_k^\top RK_k=0$$
得 $P_k$（唯一正定解存在，因为 $A_k$ Hurwitz 且右端正定）。
3. **策略改进（Policy Improvement）**：更新增益
$$K_{k+1}=R^{-1}B^\top P_k.$$
4. **重复**直到 $\|P_{k+1}-P_k\|<\varepsilon$。

**为什么这是 Newton 法**：CARE 残差 $\mathcal R(P)=A^\top P+PA-PBR^{-1}B^\top P+Q$。对 $\mathcal R$ 在当前 $P_k$ 处做 Newton 线性化：
$$\mathcal R'(P_k)\Delta P=(A-BK_k)^\top\Delta P+\Delta P(A-BK_k)=-\mathcal R(P_k).$$
这恰好是步骤 2 的 Lyapunov 方程（$\Delta P=P_{k+1}-P_k$ 满足 $A_k^\top\Delta P+\Delta P A_k=-\mathcal R(P_k)$）。因此 Kleinman 迭代 = 应用于 Riccati 算子的 Newton 法。

**收敛性定理（Kleinman 1968, Hewer 1971）**：
- **单调性**：$P_0\succeq P_1\succeq\cdots\succeq P^\star$（每步迭代值函数不增）。
- **稳定性保持**：每个 $A_k=A-BK_k$ 都是 Hurwitz（不会中途"跳出"稳定域）。
- **二次收敛**：$\|P_{k+1}-P^\star\|\le C\|P_k-P^\star\|^2$（与 Newton 法相同的超线性收敛）。
- **全局收敛**（在稳定初始化条件下）：从任何使 $A_0$ Hurwitz 的 $K_0$ 出发都收敛到 $P^\star$。

**证明思路（单调性）**：令 $V_k(x)=x^\top P_k x$，它是使用次优控制 $u=-K_k x$ 的闭环代价（因为 $P_k$ 满足以 $K_k$ 为反馈的 Lyapunov 方程）。而 $K_{k+1}=R^{-1}B^\top P_k$ 是在 $P_k$ 的基础上做一步贪心改进（最小化 Bellman 右端），所以 $V_{k+1}\le V_k$，即 $P_{k+1}\preceq P_k$。

**与 RL Policy Iteration 的精确对应**：

| Kleinman 迭代 | RL Policy Iteration |
|---|---|
| $K_k$（当前增益） | $\pi_k$（当前策略） |
| 解 Lyapunov 得 $P_k$ | 评估 $V^{\pi_k}$（policy evaluation） |
| $K_{k+1}=\arg\min_K\{K^\top RK+...\}$ | $\pi_{k+1}=\arg\max_a Q^{\pi_k}(s,a)$（policy improvement） |
| 二次收敛 | 有限步收敛（离散） |

**Fazel 2018 的 insight**：PG on LQR 的自然梯度步等价于 Kleinman 的一步迭代（当精确计算 Fisher 信息和梯度时）。这揭示了经典控制与现代 RL 在算法层面的**完美统一**。

##### 辛矩阵 Doubling Algorithm（SDA）

对大规模稀疏系统（如 MPC 中的多步 Riccati 递推），SDA 利用辛矩阵的"翻倍"性质：每次迭代等效地把时间步长加倍，使收敛步数为 $O(\log(1/\varepsilon))$ 而非 $O(1/\varepsilon)$。具体地，构造辛矩阵对 $(E_k, A_k)$：
$$E_{k+1}=E_k(E_k+A_k)^{-1}E_k,\quad A_{k+1}=A_k(E_k+A_k)^{-1}A_k.$$
$E_k\to0$，$A_k\to0$，而 $G_k=(E_k-A_k)(E_k+A_k)^{-1}\to P^\star$。这是 HPIPM（acados 的内部 QP 求解器）使用的核心算法。

**工具映射**：
- Python：`scipy.linalg.solve_continuous_are(A,B,Q,R)`、`solve_discrete_are`；`control.lqr/dlqr/lqe`（v0.10.2）
- MATLAB：推荐 `icare/idare`（R2019a+，取代旧 `care/dare`）
- 底层 Fortran：**SLICOT** 库（`slicot.org`），python-control 通过 `slycot` 绑定

---

### 三、核心教材深度对照

| 书名 | 作者 | 年份 | LQR 章节 | 免费 PDF | 评价 |
|------|------|------|----------|----------|------|
| *Optimal Control: Linear Quadratic Methods* | Anderson & Moore | Dover 2007 重印 | 全书 | 无（购书 Dover） | **LQR/LQG 老圣经**，含 LTR、frequency shaping、完整习题解答 |
| *Dynamic Programming and Optimal Control* Vol. I, 4e | Bertsekas | Athena 2017 | Ch.3, 4, 7 | 样章 + 解答部分 | DP 视角严密；LQR 与 MPC、近似 DP/RL 连贯 |
| *Optimal Control and Estimation* | Stengel | Dover 1994 | Ch.5–6 | 无 | 航空航天风格，Apollo 主设计师手笔 |
| *Robust and Optimal Control* | Zhou-Doyle-Glover | Prentice-Hall 1996 | Ch.12, 14 | 无（勘误 ece.lsu.edu/kemin/robust.htm） | **$H_\infty$ 圣经**，DGKF 状态空间解完整 |
| *Multivariable Feedback Control* 2e | Skogestad-Postlethwaite | Wiley 2005 | Ch.9 | **官方 PDF** folk.ntnu.no/skoge/book/ps/bookall.pdf | 工业 MIMO 设计首选，频域直觉强 |
| *Underactuated Robotics* | Tedrake | 在线持续更新 | Ch.8 LQR、Ch.10 TVLQR | **underactuated.mit.edu/lqr.html** | 机器人最推荐，Drake 代码可直接跑 Colab |
| *Calculus of Variations and Optimal Control Theory* | Liberzon | Princeton 2012 | Ch.6 | **liberzon.csl.illinois.edu/teaching/cvoc.pdf** | 简洁优美；PMP→HJB→LQR 一气呵成 |
| *Optimal Control Theory: An Introduction* | Kirk | Dover 2004（原 1970） | Ch.5 | 无 | 最温和入门，证明详细 |
| *Feedback Control Theory* | Doyle-Francis-Tannenbaum | Dover 2009 重印 | Ch.7–11 | **control.utoronto.ca/people/profs/francis/dft.pdf** | $H_\infty$ 入门经典，SISO 严格 |

**学习路径建议**：Kirk → Liberzon Ch.6 → Anderson-Moore Part I/II → Tedrake Ch.8（做代码） → Bertsekas Ch.4（LQG） → Zhou-Doyle-Glover Ch.12,14（进阶 $H_\infty$） → Skogestad-Postlethwaite（工程 MIMO）。

---

### 四、关键论文清单

1. Kalman 1960a，*ASME JBE* 82(1):35–45——Kalman 滤波奠基；PDF cs.unc.edu/~welch/kalman/media/pdf/Kalman1960.pdf
2. Kalman 1960b，*Bol. Soc. Mat. Mex.* 5(2):102–119——LQR 奠基；PDF ee.iitb.ac.in/~belur/ee640/optimal-classic-paper.pdf
3. Wonham 1968，*SIAM J. Control* 6(4):681–697——随机 Riccati；DOI 10.1137/0306044
4. Doyle 1978，*IEEE TAC* 23(4):756–757——"There are none."；PDF cds.caltech.edu/~murray/wiki/images/b/b4/Guaranteed_margins_for_LQG_regulators_-_doyle.pdf
5. Kleinman 1968，*IEEE TAC* 13(1):114–115——ARE 的 Newton 迭代
6. Laub 1979，*IEEE TAC* 24(6):913–921——Schur 方法
7. Huang-Malhamé-Caines 2006，*Commun. Inf. Syst.* 6(3):221–252——平均场 LQG
8. Abbasi-Yadkori-Szepesvári 2011，*COLT*——$\widetilde O(\sqrt T)$ 悔界；PMLR v19
9. Van Parys-Kuhn-Goulart-Morari 2016，*IEEE TAC* 61(2):430–442——Wasserstein 分布鲁棒
10. Fazel-Ge-Kakade-Mesbahi 2018，*ICML* / arXiv:1801.05039——PG on LQR 全局收敛
11. Dean-Mania-Matni-Recht-Tu 2020，*FoCM* 20:633–679 / arXiv:1710.01688——样本复杂度
12. Akyürek 等 2023，*ICLR* / arXiv:2211.15661——ICL 作为线性回归
13. von Oswald 等 2023，*ICML* / arXiv:2212.07677——ICL 即梯度下降
14. Taşkesen-Iancu-Koçyiğit-Kuhn 2023，*NeurIPS* / arXiv:2305.17037——分布鲁棒 LQ
15. Goel-Bartlett 2024，*L4DC* / arXiv:2312.06937——Transformer 表示 Kalman 滤波器

---

### 五、C++ / Python 库映射

- **python-control** 0.10.2（`pypi.org/project/control/`）：`K,S,E = control.lqr(A,B,Q,R)` / `dlqr` / `lqe`；支持 `method='slycot' | 'scipy'`。
- **SciPy** `scipy.linalg.solve_continuous_are(A,B,Q,R)`、`solve_discrete_are`——最底层 Riccati 求解器。
- **MATLAB Control System Toolbox**：`[K,S,P]=lqr(sys,Q,R,N)`、`dlqr`、`icare/idare`（新，推荐）、`kalman`、`lqg`；Robust Control Toolbox 提供 `hinfsyn`、`musyn`。
- **Drake (pydrake)**：`LinearQuadraticRegulator(A,B,Q,R)`、`FiniteHorizonLinearQuadraticRegulator(system, context, t0, tf, Q, R, options)`（即 TVLQR）；文档 drake.mit.edu/pydrake/pydrake.systems.controllers.html。
- **CasADi**：不提供 `lqr` 原语；典型用法=SciPy 求 Riccati + CasADi 构造 MPC。官方示例 github.com/casadi/casadi/blob/main/docs/examples/python/lqr_control.py。
- **SLICOT**（slicot.org）：Fortran 77 控制库 570+ 子程序，python-control 与 MATLAB 部分函数的底层。
- **safe-control-gym**（github.com/utiasDSL/safe-control-gym）：PyBullet + CasADi，LQR/iLQR/MPC 基准，直接做四旋翼/倒立摆 benchmark。

**十行最小示例**（cartpole 稳定化）：
```python
import numpy as np, control
A = np.array([[0,1,0,0],[0,0,-1,0],[0,0,0,1],[0,0,11,0]])
B = np.array([[0],[1],[0],[-1]])
Q = np.diag([10,1,10,1]); R = np.array([[0.1]])
K, S, E = control.lqr(A, B, Q, R)
print("gain:", K, "\nclosed-loop eigenvalues:", E)
```

---

### 六、机器人工程应用

**四旋翼姿态稳定化（SE(3) LQR）**：在悬停点线性化得 12 维 LTI（位置-姿态-线速度-角速度），用 LQR 得到姿态-推力反馈；流形部分用四元数或指数坐标处理。CMU 16-745、Tedrake Ch.3 有完整案例。

**MPC 终端代价 = 无穷时域 LQR 值函数**：为证明 MPC 闭环稳定性，常取终端代价 $V_f(x)=x^\top P_\infty x$（LQR 值函数）＋终端约束 $x_N\in\Omega$，让 MPC "在 horizon 末端无缝接入 LQR"——这是 Mayne-Rawlings-Mayne-Rao-Scokaert 2000 经典 MPC 稳定性理论核心，直接衔接专题 3.11–3.12。

**iLQR / DDP 的 backward pass = TVLQR**：每次 backward pass 对线性化动力学做反向 RDE 递推。专题 3.9 直接继承本专题 §3.5.C。

**自动驾驶横向控制（path-tracking LQR）**：把 lateral error + heading error 作为状态，Bicycle 模型线性化后用 LQR 得前轮转角反馈；Apollo、Autoware 均用该方案。

**RL 基准**：LQR 是 **连续控制 RL 的"已知最优"**，PPO/SAC 的实现在 linear-quadratic benchmark 上必须能匹敌 $K^\star$；否则算法实现有 bug。见 OpenAI Gym 的 `LQR-v0`、safe-control-gym。

**SLAM 中的 Kalman 滤波器**：EKF-SLAM、MSCKF（Mourikis-Roumeliotis 2007，VIO 业界标准）本质是**带非线性观测的 Kalman 滤波器**——分离原理的估计侧。控制侧若仍走 LQR/LQG，结合 MSCKF 就是一条完整的 SLAM + 控制闭环。

---

### 七、关键定理清单

1. **LQR 最优性定理**（HJB 路径）：$(A,B,Q,R)$ 给定、$R\succ0$，RDE 唯一解 $P(t)$ 存在，最优控制 $u^\star=-R^{-1}B^\top P x$。
2. **DP 等价定理**：离散 RDE 由 Bellman 方程直接导出；连续极限恢复微分 Riccati。
3. **PMP–Riccati 等价定理**：共态 $\lambda=Px$ ⇔ $P$ 满足 RDE；$\lambda$ 沿 Hamiltonian 矩阵的稳定不变子空间演化。
4. **稳态 LQR 存在唯一性**（Kalman-Wonham）：$(A,B)$ 可稳定化 + $(A,Q^{1/2})$ 可检测 ⇒ CARE 有唯一 $P^\star\succeq0$，$A-BK^\star$ Hurwitz。
5. **分离原理 / 确定性等价**：LQG 最优控制器 = Kalman 滤波器 $\hat x$ + LQR 增益 $K$ 作用于 $\hat x$。
6. **Kalman-LQR 对偶**：$(A,B,Q,R)\leftrightarrow(A^\top,C^\top,\Sigma_w,\Sigma_v)$ 双方 CARE 互映。
7. **LQR 鲁棒裕度**：单输入 LQR 相位裕度 $\geq60°$、增益裕度 $[1/2,\infty)$（return difference $|1+L(j\omega)|\geq1$）。
8. **Doyle 1978 反例**：LQG 不保证任何鲁棒裕度。
9. **Fazel 等 2018 全局收敛**：$C(K)$ 满足梯度优势；PG/NPG 线性收敛到 $K^\star$，无虚假局部极小。
10. **Dean 等 2020 样本复杂度**：ID + SLS 鲁棒综合给出 $\widetilde O((n+m)/\varepsilon^2)$ 样本上界。
11. **Laub 1979 Schur 定理**：Hamiltonian 矩阵稳定不变子空间 ⇒ CARE 解的数值稳定算法。

---

### 八、学习资源

#### 英文视频 / 讲义
- **Steve Brunton "Control Bootcamp"**：youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m（LQR、Kalman、LQG、Robust Control 各 10–25 min）。
- **Brian Douglas, MATLAB Tech Talk "State Space Part 4: What is LQR control?"**：youtube.com/watch?v=E_RDCFOlJx4。
- **MIT OCW 6.241J (Dahleh)**：ocw.mit.edu/courses/6-241j-dynamic-systems-and-control-spring-2011/，讲义含 LQR/LQG/$H_\infty$。
- **MIT OCW 6.231 (Bertsekas)**：ocw.mit.edu/courses/6-231-dynamic-programming-and-stochastic-control-fall-2015/。
- **MIT 6.832 Underactuated Robotics**：underactuated.mit.edu/lqr.html（LQR）、trajopt.html（TVLQR）；OCW 2009 lecture 10 iLQR 视频 ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2009/resources/lecture-10-trajectory-stabilization-and-iterative-linear-quadratic-regulator/。
- **Stanford EE363 (Boyd)**：stanford.edu/class/ee363/ ；关键 PDF `dlqr.pdf`、`clqr.pdf`、`riccati-derivation.pdf`、`allslides.pdf`。
- **Doyle-Francis-Tannenbaum 免费 PDF**：control.utoronto.ca/people/profs/francis/dft.pdf；Caltech 按章拆分镜像 cds.caltech.edu/~murray/courses/cds110/wi03/dft.html。

#### 中文资源
- **DR_CAN B 站 "最优控制" 系列**：bilibili.com/video/BV1dm4y177tA/（LQR 推导，7 万+ 播放）；"Advanced 控制理论" 第 8 集 LQR Simulink 建模（bilibili.com/video/av17876846/，20 万+ 播放）。
- **CMU 16-745 Optimal Control 2023 中文字幕搬运**：bilibili.com/video/BV1KMdNYuEa2/（36 讲）。
- 知乎 **"欠驱动机器人（七）—— LQR"**：zhuanlan.zhihu.com/p/142483990（Tedrake Ch.8 精读）。
- 知乎 **"LQR 与 iLQR：从理论到实践"**：zhuanlan.zhihu.com/p/715102938。
- 知乎 **"Lecture 18–19 随机最优控制与 LQG"**：zhuanlan.zhihu.com/p/642192501。

#### 代码 / 教程
- Drake TVLQR 示例：underactuated.mit.edu/trajopt.html + notebooks；Gallery drake.mit.edu/gallery.html。
- CasADi LQR 示例：github.com/casadi/casadi/blob/main/docs/examples/python/lqr_control.py。
- safe-control-gym：github.com/utiasDSL/safe-control-gym（四旋翼 / cartpole LQR baseline）。
- python-control 官方文档：python-control.readthedocs.io/en/latest/。

---

### 九、自测题（5 道）

1. **推导等价性**：考虑标量系统 $\dot x=ax+bu$，代价 $J=\tfrac12\int_0^\infty(qx^2+ru^2)dt$。分别用 (i) HJB Ansatz、(ii) DP 离散化取极限、(iii) PMP + Hamiltonian 矩阵三种方法解出稳态 $P$，并证明三结果相同。验证闭环稳定性条件 $q>0$ 或 $a<0$。
2. **Doyle 反例**：取 $A=\bigl(\begin{smallmatrix}1&1\\0&1\end{smallmatrix}\bigr)$、$B=\bigl(\begin{smallmatrix}0\\1\end{smallmatrix}\bigr)$、$C=(1\ 0)$，$Q=qI$、$R=1$，$\Sigma_w=\sigma I$、$\Sigma_v=1$。计算 LQG 控制器，绘制开环 Bode 图，说明当 $q,\sigma\to\infty$ 时相位裕度坍缩到零。
3. **对偶性编程验证**：用 `scipy.linalg.solve_continuous_are` 求 $(A,B,Q,R)$ 的 $P$；再用 $(A^\top,C^\top,\Sigma_w,\Sigma_v)$ 求 $\Sigma$。构造对偶参数使 $P=\Sigma$，数值验证。
4. **TVLQR 实现**：对倒立摆非线性模型 $\ddot\theta=\sin\theta+u$，给定上摆轨迹 $(\theta^\star(t),u^\star(t))$（$t\in[0,2]$），离散化后反向积分时变 Riccati 得 $K(t)$；用 Monte Carlo 测试闭环轨迹对初值扰动的鲁棒性。
5. **PG on LQR 收敛**：实现零阶策略梯度 $\hat\nabla C(K)=\tfrac{d}{2\sigma^2}\mathbb E_U[C(K+\sigma U)U]$，对 2D LQR 运行；绘制 $\log(C(K_t)-C(K^\star))$ 关于迭代步的曲线，验证线性收敛率（Fazel 2018 Theorem 7）。

---

### 十、常见陷阱

1. **$R$ 奇异 = 控制成本为零的维度** → Riccati 方程分母爆掉；调试时加小正则 $R+\epsilon I$ 再减小 $\epsilon$。
2. **$Q$ 选成 $C^\top C$ 但 $(A,C)$ 不可检测** → CARE 有解但非唯一正定；闭环有不稳定模态。必须先用 PBH 检查。
3. **连续↔离散 Riccati 混淆**：DARE 中 $R+B^\top PB$ 的 $P$ 下标应为 $k+1$（后一时刻），而非 $k$。手推时常错。
4. **有限时域 Riccati 记号**：$-\dot P=\ldots$ 的负号来源于时间反向；$P(T)=Q_f$ 是终端条件，数值积分应**倒向**。
5. **LQG "最优" ≠ 鲁棒**：不要把 LQG 当默认稳健方案；有参数不确定时务必上 $H_\infty$ 或 LTR。
6. **分离原理只在精确高斯-线性下严格**：非线性或非高斯噪声（如 SLAM 的视觉测量）只有**近似分离**，EKF-based 控制性能可能意外崩溃。
7. **TVLQR 邻域限制**：反馈只在标称轨迹的一阶 Taylor 邻域内有效。远离轨迹时应用 MPC 或 region-of-attraction（SOS 验证）。
8. **SE(3) 上 LQR 陷阱**：四元数的双覆盖（$q$ 和 $-q$ 同姿态）必须在代价中处理，否则反馈会把姿态"转远路"。
9. **Policy Gradient 需初始稳定**：若初始 $K_0$ 使 $A-BK_0$ 不稳定则 $C(K_0)=\infty$；必须先找一个稳定 $K_0$ 再做 PG，否则梯度未定义。
10. **数值 `care` 返回非正定** 往往是 $Q,R$ 对称性未强制或系统不可稳定化；先 `Q=(Q+Q')/2` 并 PBH 检查。

---

### 十一、与后续专题的桥梁

- **→ 专题 3.6（辨识、鲁棒与频域）**：LQR 是"间接法"的闭式典范，也是鲁棒控制 $H_\infty$ 综合的基准对照物；打靶法/collocation（→ 专题 3.12）在非线性问题上用 LQR 的 $K(t)$ 做 warm-start 与轨迹镇定。
- **→ 专题 3.9（iLQR/DDP）**：backward pass = TVLQR；本章 §3.5.C 是该专题直接前置。
- **→ 专题 3.10（约束 DDP 家族与 Crocoddyl）**：约束 DDP 的 SQP 视角中，每步就是二次规划 + 线性约束，LQR 是其无约束子情形。
- **→ 专题 3.11–3.12（MPC）**：终端代价取 LQR 值函数保证稳定性；tube MPC、SLS-MPC 都基于 LQR 构造名义控制器 + 鲁棒修正。
- **→ 第六批（RL）**：LQR 是所有连续控制 RL 算法的"果蝇"验证场；Fazel 2018、Dean 2020、Goel-Bartlett 2024 是 RL 理论入门必读。
- **→ 第七批（具身智能 / foundation model）**：Transformer 做 in-context Kalman/LQG 是"机器人基础模型"的理论内核。

#### §3.5.Ia Schur 分解法求解 ARE 的完整算法推导 ⭐⭐⭐

##### 动机与历史

Laub（1979）提出了利用 Hamiltonian 矩阵的实 Schur 分解来求解 ARE 的算法。这一方法是当今所有主流数值库（MATLAB `care`、SciPy `solve_continuous_are`、SLICOT）的默认实现。理解它不仅是为了"会用"，更是为了诊断数值问题——当 `care` 报错时，知道它内部在做什么才能有效排查。

##### 算法步骤

**输入**：系统矩阵 $(A,B,Q,R)$，满足 $(A,B)$ 可稳定化，$(A,Q^{1/2})$ 可检测。

**Step 1：构造 Hamiltonian 矩阵**
$$H=\begin{pmatrix}A & -BR^{-1}B^\top\\-Q & -A^\top\end{pmatrix}\in\mathbb R^{2n\times 2n}.$$

**Step 2：计算实 Schur 分解**
对 $H$ 做正交相似变换 $U^\top HU=T$（$U$ 正交，$T$ 上三角或准上三角），并对特征值进行排序，使 $T$ 的左上 $n\times n$ 块 $T_{11}$ 对应所有 $\mathrm{Re}(\lambda)<0$ 的特征值。

具体地：先用 QR 迭代得到 Schur 形式，再用交换算法（Givens 旋转）把稳定特征值排到左上角。LAPACK 的 `dgees` + 排序选项直接完成此步。

**Step 3：提取稳定不变子空间**
将 $U$ 的前 $n$ 列分块为：
$$U(:,1:n)=\begin{pmatrix}U_{11}\\U_{21}\end{pmatrix},\quad U_{11},U_{21}\in\mathbb R^{n\times n}.$$
这 $n$ 列张成 $H$ 的稳定不变子空间 $\mathcal V_-$。

**Step 4：计算 Riccati 解**
$$P^\star=U_{21}U_{11}^{-1}.$$

**Step 5：验证**
- 检查 $P^\star=P^{\star\top}$（对称性，数值上强制 $P\leftarrow(P+P^\top)/2$）
- 检查残差 $\|A^\top P+PA-PBR^{-1}B^\top P+Q\|<\epsilon$
- 检查 $\sigma(A-BR^{-1}B^\top P)\subset\mathbb C_-$（闭环稳定）

##### 为什么 Schur 方法比直接特征分解好

| 方面 | 特征分解 | Schur 分解 |
|------|---------|-----------|
| 数值稳定性 | 特征向量矩阵可能病态（条件数大） | $U$ 正交 → 条件数=1 |
| 复数运算 | $H$ 有复特征值时需复数运算 | 实 Schur 形式全程实数 |
| 对称性保持 | 不自动保证 $P=P^\top$ | 容易后处理强制对称 |
| 计算复杂度 | $O(n^3)$ | $O(n^3)$，但常数更小 |

##### Python 实现要点

```python
import numpy as np
from scipy.linalg import schur, ordqz

def care_schur(A, B, Q, R):
    """用 Schur 分解求解 CARE（教学实现）"""
    n = A.shape[0]
    S = B @ np.linalg.solve(R, B.T)  # S = B R^{-1} B^T
    # 构造 Hamiltonian
    H = np.block([[A, -S], [-Q, -A.T]])
    # 实 Schur 分解，按特征值实部排序（负在前）
    T, U, sdim = schur(H, output='real', sort='lhp')
    # 提取稳定子空间
    U11 = U[:n, :n]
    U21 = U[n:, :n]
    # 计算 P = U21 * U11^{-1}
    P = np.linalg.solve(U11.T, U21.T).T
    P = (P + P.T) / 2  # 强制对称
    return P
```

##### 数值稳定性的进一步讨论

当系统接近不可稳定化边界时（某个不稳定模态的可控度很弱），Hamiltonian 矩阵的特征值接近虚轴——Schur 排序在此处变得敏感。SLICOT 使用 Hamiltonian Schur 分解（Benner-Mehrmann-Xu 1997）的改进版本，利用辛结构获得额外的数值精度。

**实际建议**：生产代码中始终使用成熟库（SciPy/SLICOT），自己实现只用于教学验证。当库函数报错时，首先检查可稳定化/可检测条件（这是最常见的失败原因），其次检查矩阵对称性（`Q=(Q+Q.T)/2`）。

#### §3.5.Ib 广义 Riccati 与描述子系统 ⭐⭐⭐⭐

对具有代数约束的描述子系统 $E\dot x=Ax+Bu$（$E$ 奇异），标准 CARE 不适用，需要**广义 ARE**：
$$A^\top X E+E^\top X A-E^\top X B R^{-1}B^\top X E+Q=0.$$
求解需要广义 Schur 分解（QZ 算法）而非普通 Schur。SciPy 的 `solve_continuous_are` 接受可选参数 `e=E` 处理此情形。

应用场景：多体系统的 DAE 形式（如 MuJoCo 内部）、电力系统的微分-代数模型。

---

### 十一b. 累积项目：本章新增模块

**累积项目名称**：从零构建线性最优控制器

| 章节 | 新增模块 | 功能 |
|------|---------|------|
| 本章 §3.5.2-3.5.3 | `riccati_solver.py` | 实现 Schur 法求解 CARE/DARE |
| 本章 §3.5.4 | `lqr_design.py` | 封装 LQR 设计流程：可控性检查 → ARE 求解 → 增益计算 → 闭环极点验证 |
| 本章 §3.5.4d | `cartpole_lqr.py` | 倒立摆线性化 + LQR 稳定 + 闭环仿真 |
| 本章 §3.5.7-8 | `lqg_design.py` | Kalman 滤波器设计 + LQR + 组合为 LQG |
| 后续 §3.9 | `ilqr.py` | 基于本章 Riccati 递推的 iLQR 实现 |

**本章代码里程碑**：运行 `python cartpole_lqr.py` 能看到倒立摆从初始偏角 $5°$ 恢复平衡的动画，控制力不超过 10N。

---

### 十一c. 值迭代与策略迭代在 LQR 上的精确对应 ⭐⭐⭐

##### 值迭代（Value Iteration）= Riccati 递推

离散 LQR 的 Bellman 递推 $P_k=Q+A^\top P_{k+1}A-A^\top P_{k+1}B(R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A$ 本质上就是**值迭代**：从终端 $P_N=Q_f$ 出发，逐步向前传播值函数。

对无穷时域问题，从 $P_0=0$（或任意初值）出发迭代同一递推（去掉时间下标），$P_k\to P^\star$。收敛是**线性的**——每步误差乘以 $\rho(A_{cl}^{\otimes2})<1$（闭环系统 Kronecker 积的谱半径）。

##### 策略迭代（Policy Iteration）= Newton-Kleinman

Newton-Kleinman 迭代（§3.5.I）本质上就是**策略迭代**：
1. **评估**当前策略 $K_k$：解 Lyapunov 方程得 $P_k$
2. **改进**策略：$K_{k+1}=R^{-1}B^\top P_k A$（贪心改进）

收敛是**二次的**——比值迭代快得多，但每步需要解一个 $n\times n$ Lyapunov 方程（$O(n^3)$ 对比值迭代的矩阵乘法）。

##### 收敛速率对比

| 方法 | 收敛阶 | 每步代价 | 总代价（到精度 $\varepsilon$） | 适用场景 |
|------|--------|---------|------|---------|
| 值迭代 | 线性 $O(\rho^k)$ | $O(n^3)$（矩阵乘） | $O(n^3\log(1/\varepsilon))$ | 简单实现、大规模稀疏 |
| 策略迭代 | 二次 $O(\rho^{2^k})$ | $O(n^3)$（Lyapunov） | $O(n^3\log\log(1/\varepsilon))$ | 快速收敛、中等规模 |
| Schur 直接法 | 一步 | $O(n^3)$ | $O(n^3)$ | 精确解、标准用法 |

> **本质洞察**：值迭代 vs 策略迭代的区别，在 RL 中是 Q-learning vs Actor-Critic 的区别。LQR 是唯一能精确分析两者收敛率差异的问题——策略迭代的二次收敛（等价于 Newton 法）解释了为什么 Actor-Critic 方法在实践中通常比纯 Q-learning 更快收敛。

##### 从 Riccati 到强化学习的概念映射

| LQR/Riccati 概念 | RL 对应 | 关系 |
|---|---|---|
| 值函数 $V(x)=x^\top Px$ | 值函数 $V^\pi(s)$ | LQR 的值函数恰为二次型 |
| DARE 递推 | Bellman 最优方程 | DARE = 最优 Bellman 的闭式版本 |
| $K=R^{-1}B^\top PA$ | $\pi^\star=\arg\max_a Q(s,a)$ | LQR 的 greedy 策略恰为线性 |
| Newton-Kleinman | Policy Iteration | 精确等价 |
| 值迭代 $P_{k+1}=T(P_k)$ | Value Iteration | 精确等价 |
| Fazel 梯度下降 | Policy Gradient (REINFORCE) | PG 在 LQR 上的特化 |
| 自然策略梯度 | Natural Policy Gradient | Fisher 预条件 = Gramian 归一化 |

这张表揭示了为什么 LQR 被称为"RL 的果蝇"——所有 RL 的核心算法在 LQR 上都有精确的数学对应物，可以严格分析收敛性。

##### 练习

1. 实现值迭代和策略迭代两种方法求解 DARE（倒立摆离散化系统）。绘制 $\|P_k-P^\star\|_F$ vs 迭代步数的对比图（双对数坐标），验证线性 vs 二次收敛。
2. 对策略迭代，记录每步的闭环极点 $\sigma(A-BK_k)$。验证所有中间极点都在单位圆内（单调稳定性保持）。
3. 讨论：如果值迭代从 $P_0=100I$（过大初值）出发，会发生什么？与 $P_0=0$ 出发的收敛方向比较。

---

### 十一d. LQR 与最优控制的其他变体关系 ⭐⭐

##### LQR 与 MPC 的精确关系

**无约束线性 MPC = receding-horizon 有限时域 LQR**。每个 MPC 时刻解一次离散有限时域 LQR（$N$ 步 Riccati 递推），只施加第一步控制 $u_0=-K_0 x$，然后滑动窗口。

当 $N\to\infty$ 或终端代价取 $Q_f=P^\star$（ARE 解），MPC 等价于稳态 LQR。当有约束时，MPC 变成在线 QP——但在约束不 active 的区域，解退化为 LQR。这就是为什么理解 LQR 是理解 MPC 的前提。

##### LQR 与极点配置的关系

极点配置（pole placement）指定闭环极点位置——自由度恰好等于 $nm$（$n$ 维状态 $\times$ $m$ 个输入）。LQR 也配置极点，但不是任意位置——它选择使 Riccati 代价最小的极点配置。

**LQR 极点的特殊性质（Butterworth 模式）**：对标量系统 $\dot x=ax+u$，LQR 闭环极点满足 $s=-\sqrt{a^2+q/r}$（总是实数、负）。对高阶系统，LQR 极点趋向于 Butterworth 滤波器的极点分布——在左半平面均匀展开。这不是巧合，而是二次代价的对称性决定的。

##### LQR 与线性矩阵不等式（LMI）

CARE 可以等价地写成 LMI 形式（矩阵 Schur 补）：
$$\begin{pmatrix}A^\top P+PA+Q & PB\\B^\top P & R\end{pmatrix}\succeq0\quad\text{且}\quad P\succeq0.$$
这一 LMI 表示在鲁棒控制中极为有用——可以把参数不确定性作为附加 LMI 约束加入，用 SDP 求解器（如 MOSEK、SeDuMi）统一处理。

> **"不是X而是Y"句式**：LMI 形式不是 CARE 的"简化版本"——它实际上是更一般化的表示，因为 LMI 允许处理**不确定系统**（robust LQR）和**结构约束**（分散控制）这些 CARE 无法处理的问题。CARE 是 LMI 在确定性无约束情形下的**特化**。

---

### 十二、时间预算

**核心档位 3（12–15 h）**：
- §3.5.1–3.5.2（问题定义 + HJB 推导）：2 h
- §3.5.3（离散 DP）：1 h
- §3.5.4（ARE/DARE、存在唯一性）：2 h
- §3.5.5（PMP + Hamiltonian 矩阵）：1.5 h
- §3.5.6（Gramian、PBH）：1.5 h
- §3.5.7–3.5.8（LQG、分离原理、对偶）：3 h
- 代码实验（cartpole LQR + Kalman + LQG）：2.5 h
- 自测题 1–3：1.5 h

**进阶档位 4（+8–12 h）**：
- §3.5.A $H_\infty$ + Doyle 反例重现：2 h
- §3.5.B LTR：1 h
- §3.5.C 流形 TVLQR + iLQR 概念：2 h
- §3.5.D–E 随机/分布鲁棒 LQR：2 h
- §3.5.F–G PG on LQR + 样本复杂度（读 Fazel 2018 Section 1–3，Dean 2020 Section 2）：2–3 h
- §3.5.H Transformer × LQR（读 Goel-Bartlett 2024）：1 h
- §3.5.I 数值方法实现（手写 Kleinman 迭代）：1–2 h
- 自测题 4–5：1 h

---

### 十三、总结

**LQR 不是一种控制器，而是一整套思想的最小可工作样例**。它把最优控制四根独立支柱——变分法（共态）、PMP（极大值原理）、DP（Bellman）、HJB（动态规划偏微分方程）——拧成一股，把答案写在同一张纸上：**Riccati 方程**。任何对 LQR 的一点理解加深，都会在 PMP、HJB、DP 里产生连锁反应。

**LQG 是一把双刃剑**。分离原理带来的清晰结构让人可以"把估计和控制分开写代码"，而 Doyle 1978 的四字摘要提醒我们：清晰并不等于鲁棒。这条边界催生了 $H_\infty$、μ-synthesis、LTR，也催生了现代数据驱动鲁棒控制（Dean 2020、Taşkesen 2023）。

**LQR 是 RL 与经典控制的交汇点**。Fazel 等（2018）证明了 policy gradient 在一个非凸非凸问题上的全局收敛，开启"可证明 RL"时代；Dean 等（2020）把"系统辨识 + 鲁棒综合"翻译为样本复杂度语言；Goel-Bartlett（2024）表明 Transformer 能在上下文里表示 Kalman 滤波器——**LQR 正在成为未来机器人基础模型的 benchmark**。

**对机器人交叉方向博士前研究者的建议**：
1. 先手推一次连续和离散 RDE，再写 100 行 Python 在 cartpole 上跑出 LQR+KF+LQG。
2. 用 Drake/pydrake 的 `FiniteHorizonLinearQuadraticRegulator` 在非线性系统（Acrobot 或四旋翼）上做一次轨迹镇定（TVLQR）——这正是专题 3.9 iLQR 的预热。
3. 精读 Doyle 1978 与 Fazel 2018 Introduction；这两篇分别代表了 LQG 的"脆弱一面"与 LQR 的"理论美一面"，合起来就是最优控制 60 年张力的最佳缩影。
4. 把 LQR 作为你以后学 MPC、iLQR、RL、SLAM 控制器的"正确性基线"：任何新算法在 LQ 问题上跑不过 $K^\star$，说明实现错了。

掌握 LQR，就掌握了从经典控制跨越到现代学习型机器人控制的**主干道路**。本专题的每一节都会在后续专题中反复复现——它是整个博士前路线图里复用率最高的数学机器之一。

---

### 十四、延伸阅读（按难度标注）

| 资源 | 难度 | 推荐理由 |
|------|------|---------|
| Kirk *Optimal Control Theory: An Introduction* Ch.5 | ⭐ | 最温和入门，证明极其详细 |
| Tedrake *Underactuated Robotics* Ch.8 | ⭐⭐ | 机器人视角+Drake 代码可跑 |
| Liberzon *Calculus of Variations* Ch.6 | ⭐⭐ | 简洁优美，PMP→HJB→LQR 一气呵成 |
| Anderson-Moore *Optimal Control: LQ Methods* | ⭐⭐ | LQR/LQG 工程圣经，含 LTR |
| Bertsekas *DP and Optimal Control* Vol.I Ch.3-4 | ⭐⭐⭐ | DP 视角严密，连贯到 RL |
| Zhou-Doyle-Glover *Robust and Optimal Control* Ch.12,14 | ⭐⭐⭐ | $H_\infty$ 圣经，DGKF 解完整 |
| Lancaster-Rodman *Algebraic Riccati Equations* | ⭐⭐⭐⭐ | ARE 代数理论最完整的专著 |
| Bini-Iannazzo-Meini *Numerical Solution of ARE* | ⭐⭐⭐⭐ | ARE 数值方法现代综述 |
| Fazel et al. 2018 (arXiv:1801.05039) | ⭐⭐⭐ | PG on LQR 全局收敛里程碑 |
| Dean et al. 2020 (arXiv:1710.01688) | ⭐⭐⭐ | Model-based RL 样本复杂度金标准 |

---

### 十五、Riccati 方程完成平方法的连续时间完整证明 ⭐⭐

本节给出连续时间完备平方法的**逐步展开**，作为 §3.5.3b 的详细补充。

##### 定理（连续完备平方恒等式）

设 $P(t)$ 为 RDE 的解（$-\dot P=A^\top P+PA-PBR^{-1}B^\top P+Q$，$P(T)=Q_f$），则对**任意**容许控制 $u(\cdot)$：
$$J[u]=\tfrac12 x(0)^\top P(0)x(0)+\tfrac12\int_0^T\|u(t)+R^{-1}B^\top P(t)x(t)\|^2_R\,dt.$$

##### 完整证明

**Step 1**：定义 $W(t)=\tfrac12 x(t)^\top P(t)x(t)$。

**Step 2**：沿**任意**轨迹（不必最优！）计算 $\dot W$：
$$\dot W=\tfrac12 x^\top\dot Px+x^\top P\dot x=\tfrac12 x^\top\dot Px+x^\top P(Ax+Bu).$$

**Step 3**：用 RDE 替换 $\dot P$。由 $\dot P=-(A^\top P+PA-PBR^{-1}B^\top P+Q)$：
$$\tfrac12 x^\top\dot Px=-\tfrac12 x^\top(A^\top P+PA)x+\tfrac12 x^\top PBR^{-1}B^\top Px-\tfrac12 x^\top Qx.$$

**Step 4**：代入 Step 2：
$$\dot W=-\tfrac12 x^\top(A^\top P+PA)x+\tfrac12 x^\top PBR^{-1}B^\top Px-\tfrac12 x^\top Qx+x^\top PAx+x^\top PBu.$$
化简（注意 $x^\top PAx=\tfrac12 x^\top PAx+\tfrac12 x^\top A^\top Px$ 利用对称性与合并）：
$$\dot W=\tfrac12 x^\top PBR^{-1}B^\top Px-\tfrac12 x^\top Qx+x^\top PBu.$$

**Step 5**：与阶段代价合并。阶段代价 $l(x,u)=\tfrac12 x^\top Qx+\tfrac12 u^\top Ru$，所以：
$$l+\dot W=\tfrac12 u^\top Ru+x^\top PBu+\tfrac12 x^\top PBR^{-1}B^\top Px.$$
右端正好是：
$$=\tfrac12(u+R^{-1}B^\top Px)^\top R(u+R^{-1}B^\top Px).$$
这一步用了二次型配方：$\tfrac12 u^\top Ru+u^\top(B^\top Px)+\tfrac12(B^\top Px)^\top R^{-1}(B^\top Px)=\tfrac12\|u+R^{-1}B^\top Px\|_R^2$。

**Step 6**：两端从 $0$ 到 $T$ 积分：
$$\int_0^T l\,dt+W(T)-W(0)=\tfrac12\int_0^T\|u+R^{-1}B^\top Px\|_R^2\,dt.$$
注意 $W(T)=\tfrac12 x(T)^\top Q_f x(T)$（RDE 终端条件），所以左端 $=J[u]-W(0)$。移项得：
$$J[u]=W(0)+\tfrac12\int_0^T\|u+R^{-1}B^\top Px\|_R^2\,dt=\tfrac12 x(0)^\top P(0)x(0)+\tfrac12\int_0^T\|u+R^{-1}B^\top Px\|_R^2\,dt.\quad\blacksquare$$

##### 最优性的直接推论

由于 $\|u+R^{-1}B^\top Px\|_R^2\ge0$ 且 $=0$ 当且仅当 $u=-R^{-1}B^\top Px$，故：
- 最优代价 $J^\star=\tfrac12 x(0)^\top P(0)x(0)$
- 最优控制 $u^\star=-R^{-1}B^\top P(t)x(t)$
- **充分性自动成立**——不需要额外检查二阶条件

这就是完备平方法的威力：一个代数恒等式同时给出了**最优控制+最优代价+充分性证明**。

---

### 十六、LQR 鲁棒性深化：多输入回差不等式与 disk margin ⭐⭐⭐

##### 多输入系统的回差不等式

§3.5.5b 给出了单输入 LQR 的经典裕度。对多输入系统（$m>1$），Safonov-Athans 1977 证明了矩阵版本的回差不等式仍然成立：
$$F(j\omega)^*RF(j\omega)\succeq R,\quad\forall\omega,$$
其中 $F(j\omega)=I_m+K(j\omega I-A)^{-1}B$ 是 $m\times m$ 回差矩阵。

但这**不**直接给出每个通道的标量裕度！多输入 LQR 对**同时**在所有通道施加相同增益/相位扰动时裕度很好，但对**单通道**扰动（只有一个执行器故障）裕度可能很差。

##### Disk Margin 概念

现代鲁棒控制用 **disk margin** 取代经典 GM/PM。设扰动矩阵 $\Delta=\mathrm{diag}(\delta_1,\ldots,\delta_m)$，$|\delta_i-1|<r$（每个通道的增益扰动在复数圆盘内）。系统能容忍的最大 $r$ 即 disk margin。

对单输入 LQR：disk margin $\ge1/2$（对应 GM$\in[1/2,\infty)$、PM$\ge60°$ 的经典结果）。

对多输入 LQR：disk margin 取决于 $B$ 的结构，一般小于 $1/2$。MATLAB R2020a 起提供 `diskmargin` 函数直接计算。

##### 为什么 LQR 有好裕度但 PID 不一定有

回差不等式 $|1+L(j\omega)|\ge1$ 是**最优性的后果**——Riccati 方程的解恰好使回差满足这一不等式。非最优的反馈（如手调 PID）没有这一保证。

> **反事实推理**：如果我们"几乎最优"但不精确地解了 ARE（例如用截断的 Newton-Kleinman 迭代），回差不等式是否仍然成立？**不一定**——次优解的裕度可能比最优解差很多。这就是为什么工程中必须用高精度求解器（收敛到机器精度）而不是粗略近似。

##### LQR 裕度的工程局限

LQR 的裕度保证有三个重要**前提**：
1. **状态完全可测**：如果状态是估计值（LQG），裕度丧失（Doyle 1978）
2. **模型精确**：裕度是对**回路增益**扰动的，不是对**模型参数**扰动。质量变化 20% 不一定在裕度范围内
3. **无约束**：实际执行器有饱和。当控制信号 clip 后，系统不再是线性反馈，裕度失效

这三点解释了为什么现代控制实践中 LQR 常作为"标称设计"，然后用 $H_\infty$ 或鲁棒 MPC 处理不确定性。

---

### 十七、Kalman 滤波器完整推导与 LQG 框架总结 ⭐⭐

##### 系统模型

$$\dot x=Ax+Bu+w,\quad y=Cx+v,$$
其中 $w\sim\mathcal N(0,\Sigma_w)$（过程噪声），$v\sim\mathcal N(0,\Sigma_v)$（观测噪声），$w$ 与 $v$ 不相关。

##### 最优估计的定义

**最优估计器**是使均方误差 $\mathbb E[\|x-\hat x\|^2]$ 最小的线性无偏估计器。对高斯系统，条件均值 $\hat x=\mathbb E[x\mid\mathcal Y_t]$ 恰为最优估计。

##### Kalman 滤波器方程推导

**Step 1**：令误差 $\tilde x=x-\hat x$。设估计器为线性形式 $\dot{\hat x}=A\hat x+Bu+L(y-C\hat x)$，误差动力学：
$$\dot{\tilde x}=(A-LC)\tilde x+w-Lv.$$

**Step 2**：误差协方差 $\Sigma=\mathbb E[\tilde x\tilde x^\top]$ 的演化：
$$\dot\Sigma=(A-LC)\Sigma+\Sigma(A-LC)^\top+\Sigma_w+L\Sigma_v L^\top.$$

**Step 3**：选择 $L$ 使 $\mathrm{tr}(\Sigma)$ 最小（最小均方误差准则）。展开上式并对 $L$ 求导：
$$\frac{\partial\,\mathrm{tr}(\dot\Sigma)}{\partial L}=-2\Sigma C^\top+2L\Sigma_v=0\quad\Rightarrow\quad L=\Sigma C^\top\Sigma_v^{-1}.$$

**Step 4**：代回得到**滤波 Riccati 方程**：
$$\dot\Sigma=A\Sigma+\Sigma A^\top-\Sigma C^\top\Sigma_v^{-1}C\Sigma+\Sigma_w.$$

##### 稳态 Kalman 增益的物理意义

$$L_\infty=\Sigma_\infty C^\top\Sigma_v^{-1}.$$

- $\Sigma_v\to0$（观测极精确）：$L\to\infty$，"完全信任观测"
- $\Sigma_w\to0$（模型极精确）：$\Sigma_\infty\to0$，$L\to0$，"完全信任预测"
- 一般情况：$L$ 是模型可信度与观测可信度的**最优权衡**

> **跨领域类比**：Kalman 增益 $L$ 如同人类在"看地图"（模型预测）和"看路况"（传感器观测）之间的注意力分配。GPS 信号好时多看 GPS（$L$ 大），GPS 进隧道时多靠惯导（$L$ 小）。SLAM 中的 EKF 做的正是这件事。

##### LQG 完整框架总结

```
              ┌──────────────┐
    r(t) ──→ │   LQR 增益    │──→ u(t) ──→ [系统 + 噪声]
              │  u = -K·x_hat │              │
              └───────┬──────┘              ▼
                      │                  y(t) = Cx + v
                      │                     │
              ┌───────┴──────┐              │
              │ Kalman 滤波器  │◄─────────────┘
              │  估计 x_hat   │
              └──────────────┘
```

**设计步骤**：
1. 解控制 CARE → $P^\star$，得 $K=R^{-1}B^\top P^\star$
2. 解滤波 CARE（对偶）→ $\Sigma^\star$，得 $L=\Sigma^\star C^\top\Sigma_v^{-1}$
3. 组合：$u=-K\hat x$，$\dot{\hat x}=(A-LC)\hat x+Bu+Ly$
4. 闭环稳定性保证：$(A-BK)$ Hurwitz + $(A-LC)$ Hurwitz → 整体 Hurwitz

---

### 十八、分离原理的深化讨论与失效条件 ⭐⭐⭐

##### 分离原理为什么在实际中"几乎从不"严格成立

分离原理依赖三个关键假设：线性动力学、高斯噪声、二次代价。任何一个假设的破坏都会使分离失效：

| 被破坏的假设 | 后果 | 工程补救 |
|---|---|---|
| 非线性动力学 | $\mathbb E[\tilde x\hat x^\top]\ne0$，代价不可正交分解 | EKF+LQR（近似分离） |
| 非高斯噪声 | 条件分布非正态，均值不是最优估计 | 粒子滤波+后验均值控制 |
| 非二次代价 | 值函数不再是二次型，HJB 无闭式解 | NMPC 在线优化 |
| 状态约束 | 最优控制为 bang-bang，非线性反馈 | 约束 MPC（在线 QP） |

**在机器人中的实际影响**：SLAM 中的 EKF + LQR 组合就是"近似分离"。当线性化误差小时，近似分离工作良好。但在大扰动下，EKF 估计偏差增大，LQR 基于错误状态施加控制 → 系统发散。这是为什么足式机器人在激烈运动时需要非线性估计器（如不变 EKF、因子图优化）和非线性控制器（MPC/WBC）的根本原因。

##### 分离原理的信息论视角

从信息论角度看，分离原理说的是：**在 LQG 问题中，信息获取（估计）和信息利用（控制）是正交的两个任务**。这是因为：
- 高斯分布完全由均值和协方差表征
- 线性系统中协方差的演化不依赖控制（$\Sigma$ 的 Riccati 不含 $u$）
- 二次代价对高斯分布只关心均值和协方差

一旦破坏任何一条，"获取信息"本身就变成了控制目标的一部分——这就是**对偶控制（dual control）**的领域，也是主动 SLAM（active SLAM）的数学基础。

---

### 十九、LQR 工程调参指南 ⭐⭐

##### $Q$ 和 $R$ 的系统化选择方法

**方法 1：Bryson 规则（经验起点）**
$$Q_{ii}=\frac{1}{x_{i,\max}^2},\qquad R_{jj}=\frac{1}{u_{j,\max}^2},$$
其中 $x_{i,\max}$ 是状态 $i$ 的"最大可接受偏差"，$u_{j,\max}$ 是输入 $j$ 的"最大可用范围"。

**物理解释**：代价 $x_i^2/x_{i,\max}^2$ 在偏差达到最大允许值时恰好为 1。这使得不同状态的代价在"归一化"意义下可比较。

**方法 2：频率加权**
对高频振动不可接受的系统（如卫星柔性附件），用频率加权 $Q(s)$ 在特定频段放大状态惩罚。

**方法 3：Pareto 扫描**
系统地扫描 $\rho\in[0.01,100]$，令 $Q=\rho Q_0$、$R=R_0$，对每个 $\rho$ 计算闭环性能指标（上升时间、超调量、控制能量），绘制 Pareto ��沿，选取"拐点"附近的设计。

##### 调参流程

1. **用 Bryson 规则得到初始 $Q_0,R_0$**
2. **检查闭环极点**：确保实部足够负（带宽满足要求），虚部不太大（避免振荡）
3. **检查增益大小**：$K$ 的每个元素不应使标称运行点附近的控制超出执行器饱和
4. **仿真验证**：加入实际非线性、饱和、延迟，验证性能
5. **迭代调整**：根据仿真结果微调 $Q,R$ 的对角元素

**切忌**：不要把 $Q,R$ 选成单位矩阵就结束——这几乎从不是好的设计。$Q=I,R=I$ 意味着"所有状态同等重要、所有输入同等昂贵"——这在物理上几乎不可能正确。

##### ⚠️ ���参常见陷阱

**概念误区**：认为"$Q$ 越大系统越稳定"。增大 $Q$ 确实使增益 $K$ 增大，但**过大的增益会放大观测噪声**（在 LQG 中）和**触发执行器饱和**（在实际系统中）。$Q$ 的选择必须与执行器能力和噪声水平匹配——这是 LQR 设计中最需要工程判断力的环节。

**思维陷阱**：认为"LQR 是最优的所以不需要调参"。LQR 最优性是**相对于给定 $(Q,R)$ 的**——$(Q,R)$ 本身是设计者对"什么是好的性能"的编码。选错 $(Q,R)$ 等于问了错误的问题，得到的"最优"答案也是错的。

---

### 二十、Schur 分解法求解 ARE 的完整算法 ⭐⭐⭐

##### 算法步骤

**输入**：系统矩阵 $(A,B,Q,R)$，满足 $(A,B)$ 可稳定化，$(A,Q^{1/2})$ 可检测。

**Step 1：构造 Hamiltonian 矩阵**
$$H=\begin{pmatrix}A & -BR^{-1}B^\top\\-Q & -A^\top\end{pmatrix}\in\mathbb R^{2n\times 2n}.$$

**Step 2：计算实 Schur 分解**
对 $H$ 做正交相似变换 $U^\top HU=T$（$U$ 正交，$T$ 准上三角），按特征值实部排序使 $T$ 的左上 $n\times n$ 块对应所有 $\mathrm{Re}(\lambda)<0$ 的特征值。

**Step 3：提取稳定不变子空间**
将 $U$ 的前 $n$ 列分块为 $U(:,1:n)=\begin{pmatrix}U_{11}\\U_{21}\end{pmatrix}$。

**Step 4：计算 Riccati 解**
$$P^\star=U_{21}U_{11}^{-1}.$$

**Step 5：后处理**
- 强制对称：$P\leftarrow(P+P^\top)/2$
- 验证残差：$\|A^\top P+PA-PBR^{-1}B^\top P+Q\|_F<\varepsilon$
- 验证稳定性：$\sigma(A-BR^{-1}B^\top P)\subset\mathbb C_-$

##### Python 教学实现

```python
import numpy as np
from scipy.linalg import schur

def care_schur(A, B, Q, R):
    """用 Schur 分解求解 CARE（教学实现，不用于生产）"""
    n = A.shape[0]
    S = B @ np.linalg.solve(R, B.T)  # S = B R^{-1} B^T
    # 构造 Hamiltonian 矩阵
    H = np.block([[A, -S], [-Q, -A.T]])
    # 实 Schur 分解，按特征值实部排序（负实部在前）
    T, U, sdim = schur(H, output='real', sort='lhp')
    # 提取稳定子空间（前 n 列）
    U11 = U[:n, :n]
    U21 = U[n:, :n]
    # 计算 P = U21 * U11^{-1}
    P = np.linalg.solve(U11.T, U21.T).T
    P = (P + P.T) / 2  # 强制对称
    return P
```

##### 与特征分解的数值稳定性对比

| 方面 | 特征分解 | Schur 分解 |
|------|---------|-----------|
| 变换矩阵条件数 | 可能很大（病态特征向量） | $\kappa(U)=1$（正交！） |
| 复数运算 | 必须处理（$H$ 常有复特征值） | 实 Schur 全程实数 |
| 对称性 | 不保证 | 后处理即可 |
| LAPACK 支持 | `dgeev` | `dgees`（更稳定） |

---

### 二十一、跨章综合练习 ⭐⭐⭐

**综合题 1**（结合专题 3.2 PMP + 本章 LQR）：对标量系统 $\dot x=ax+bu$ 最小化 $J=\int_0^\infty(qx^2+ru^2)dt$，分别用 (a) PMP 完整流程（写 Hamilton 函数→协态方程→$u^\star$→Ansatz $\lambda=px$→ARE）；(b) HJB + 值函数 Ansatz；(c) 完备平方法。三种方法得到同一个 ARE $2ap-p^2b^2/r+q=0$。解出 $p=r(a+\sqrt{a^2+qb^2/r})/b^2$，讨论当 $a>0$（不稳定）和 $a<0$（稳定）时解的行为。

**综合题 2**（结合专题 3.3 DP + 本章离散 LQR）：对双积分器 $x_{k+1}=\begin{pmatrix}1&\Delta t\\0&1\end{pmatrix}x_k+\begin{pmatrix}\Delta t^2/2\\\Delta t\end{pmatrix}u_k$，$Q=I$，$R=1$。手动执行 3 步 Bellman 递推（$P_3=0$→$P_2$→$P_1$→$P_0$），并与 `scipy.linalg.solve_discrete_are` 的稳态解比较。

**综合题 3**（结合专题 3.4 HJB + 本章鲁棒性）：考虑带不确定性的系统 $\dot x=(a+\delta)x+bu$，$|\delta|<\epsilon$。(a) 用标称 $a$ 设计 LQR。(b) 计算闭环稳定性对 $\delta$ 的容忍范围（利用 Lyapunov 分析）。(c) 与 $H_\infty$ 设计比较：把 $\delta$ 视为对手的扰动，写出博弈 Riccati，验证鲁棒性更好但代价更高。

---

### 二十二、PMP 路径推导 Riccati 的详细展开 ⭐⭐

本节把 §3.5.5 中的 PMP 推导展开为初学者可跟随的逐步形式。

##### Step-by-Step PMP 推导

**问题**：$\dot x=Ax+Bu$，$J=\tfrac12\int_0^T(x^\top Qx+u^\top Ru)dt+\tfrac12 x(T)^\top Q_f x(T)$。

**Step 1：写 Hamilton 函数**（PMP 第一步总是写 Hamiltonian）
$$\mathcal H(x,u,\lambda,t)=\tfrac12 x^\top Qx+\tfrac12 u^\top Ru+\lambda^\top(Ax+Bu).$$
其中 $\lambda\in\mathbb R^n$ 是**共态**（协态/伴随变量/Lagrange 乘子的动力学版本）。

**Step 2：最小化条件**（Hamiltonian 最小化）
$$\frac{\partial\mathcal H}{\partial u}=Ru+B^\top\lambda=0\quad\Rightarrow\quad u^\star=-R^{-1}B^\top\lambda.$$
（$R\succ0$ 保证二阶条件满足：$\partial^2\mathcal H/\partial u^2=R\succ0$，确认是最小值。）

**Step 3：共态方程**（PMP 第二条件）
$$\dot\lambda=-\frac{\partial\mathcal H}{\partial x}=-Qx-A^\top\lambda.$$

**Step 4：横截条件**（PMP 对自由终端状态的条件）
$$\lambda(T)=\frac{\partial}{\partial x}\bigl[\tfrac12 x(T)^\top Q_f x(T)\bigr]=Q_f x(T).$$

**Step 5：关键 Ansatz $\lambda(t)=P(t)x(t)$**。这一假设的动机：
- 横截条件 $\lambda(T)=Q_f x(T)=P(T)x(T)$ 在 $P(T)=Q_f$ 时成立
- 系统是线性的，直觉上共态应是状态的线性函数
- 数学上可严格证明：线性系统+二次代价 → 值函数二次 → $\nabla_x V=Px=\lambda$

**Step 6：对 Ansatz 求时间导数**
$$\dot\lambda=\dot P x+P\dot x=\dot P x+P(Ax+Bu)=\dot P x+P(Ax-BR^{-1}B^\top Px).$$
（最后一步代入了 $u^\star=-R^{-1}B^\top\lambda=-R^{-1}B^\top Px$。）

**Step 7：与共态方程比较**。由 Step 3：$\dot\lambda=-Qx-A^\top\lambda=-Qx-A^\top Px$。令 Step 6 = Step 7：
$$\dot P x+PAx-PBR^{-1}B^\top Px=-Qx-A^\top Px.$$
整理（所有项写成 $(\cdots)x$ 的形式，因为对所有 $x$ 成立，系数必须相等）：
$$\dot P=-A^\top P-PA+PBR^{-1}B^\top P-Q.$$
移项得到**标准形式**：
$$\boxed{-\dot P=A^\top P+PA-PBR^{-1}B^\top P+Q,\quad P(T)=Q_f.}$$

这与 HJB 路径（§3.5.2）得到的结果**完全相同**！

##### 为什么两条路径必然给出同一方程

这不是巧合，而是深刻的数学必然。PMP 和 HJB 是最优控制理论的**两根支柱**：
- PMP 从"轨迹"视角出发——对每条可能的轨迹施加必要条件
- HJB 从"值函数"视角出发——对所有状态的最优代价施加充分条件

两者在 LQR 中的交汇点就是 $\lambda=\nabla_x V=Px$——共态（PMP 对象）恰好等于值函数梯度（HJB 对象）。这一等式是 Pontryagin 与 Bellman 两个学派在 1960 年代最重要的和解。

> **本质洞察**：PMP 的共态 $\lambda$ 和 HJB 的值函数梯度 $\nabla_x V$ 是同一个数学对象的两种面貌。$\lambda$ 携带的信息是"从当前状态出发的未来最优代价对当前状态的敏感度"——这恰好就是值函数的梯度。Riccati 方程是它们共同的"底层语言"。

##### 练习

1. 对标量系统 $\dot x=2x+u$，$J=\int_0^1(x^2+u^2)dt+3x(1)^2$。用 PMP 路径写出 Hamiltonian、最优条件、共态方程、横截条件。假设 $\lambda=p(t)x$，推导出标量 RDE $-\dot p=4p-p^2+1$，$p(1)=3$。
2. 用 Python 的 `solve_ivp` 倒向积分该标量 RDE（提示：做时间变换 $\tau=1-t$，得到正向 ODE $dp/d\tau=4p-p^2+1$，$p(0)=3$），绘制 $p(t)$ 的图形。
3. 验证：当 $T\to\infty$ 时 $p(t)\to p_\infty$，其中 $p_\infty$ 满足标量 ARE $4p-p^2+1=0$。解出 $p_\infty=2+\sqrt{5}$（取使闭环稳定的根）。

---

### 二十三、LQR 设计的完整工程流程总结 ⭐⭐

以下是从问题建模到部署的完整流程，适用于任何线性化后可用 LQR 的机器人系统：

```
[物理系统]
    │
    ▼ ① 建模与线性化
[状态空间模型 (A, B, C)]
    │
    ▼ ② 可控性/可检测性验证
[PBH 检查 → 确认 ARE 良定]
    │
    ▼ ③ 权重选择
[Bryson 规则 → 初始 Q, R]
    │
    ▼ ④ 求解 CARE/DARE
[scipy.solve_continuous_are → P, K]
    │
    ▼ ⑤ 闭环分析
[极点检查 + Bode 图 + 时域仿真]
    │
    ▼ ⑥ 鲁棒性评估
[增益/相位裕度 + 参数灵敏度]
    │
    ▼ ⑦ 非线性仿真验证
[全模型 + 饱和 + 延迟 + 噪声]
    │
    ▼ ⑧ 如果不满足 → 回到③调参 或 改用 MPC/H∞
[部署]
```

##### ⚠️ 全流程常见陷阱

**思维陷阱**：跳过步骤②直接设计。如果系统不可控（如某个电机断线），ARE 无解但不一定报错——可能返回数值垃圾。**永远先检查可控性**。

**编程陷阱**：把连续系统 $(A_c, B_c)$ 的 CARE 解直接用于离散控制器。必须先用 ZOH 离散化 $A_d=e^{A_c\Delta t}$，$B_d=\int_0^{\Delta t}e^{A_c\tau}d\tau\cdot B_c$，然后解 DARE。用 `scipy.signal.cont2discrete` 完成此步。

**概念误区**：认为"LQR 给出的 $K$ 是全局最优的"。$K$ 只在线性化点附近的一个邻域内最优——远离平衡点时增益方向可能完全错误。对于大范围运动，必须用 TVLQR（沿轨迹线性化）或 MPC（在线重新优化）。

---

### 附录 A：扩展书单

**A.1 控制理论纵深**
- **Lancaster & Rodman**, *Algebraic Riccati Equations*, Oxford 1995——ARE 的代数理论圣经，含全部存在性/唯一性/参数化定理。
- **Sontag**, *Mathematical Control Theory*, 2nd ed., Springer 1998——可控/可观的几何理论参考；免费 PDF: http://www.sontaglab.org/FTPDIR/sontag_mathematical_control_theory_springer98.pdf
- **Trentelman, Stoorvogel, Hautus**, *Control Theory for Linear Systems*, Springer 2001——几何控制理论（不变子空间方法）。

**A.2 鲁棒与最优控制**
- **Skogestad & Postlethwaite**, *Multivariable Feedback Control*, 2nd ed., Wiley 2005——LQG/LQR/H∞ 工程实务首选。
- **Dullerud & Paganini**, *A Course in Robust Control Theory*, Springer 2000——LMI 方法。

**A.3 数值线性代数（解 ARE 必备）**
- **Mehrmann**, *The Autonomous Linear Quadratic Control Problem*, Springer LNCS 1991——Hamilton 矩阵数值方法专著。
- **Bini, Iannazzo, Meini**, *Numerical Solution of Algebraic Riccati Equations*, SIAM 2012——ARE 数值解法的现代综述。

**A.4 学习与控制交叉**
- **Vrabie, Vamvoudakis, Lewis**, *Optimal Adaptive Control and Differential Games by RL Principles*, IET 2013——ADP/IRL 全景。
- **Bertsekas**, *Reinforcement Learning and Optimal Control*, Athena Scientific 2019——DP-RL 桥梁权威，免费章节 PDF: http://web.mit.edu/dimitrib/www/RLbook.html

### 附录 B：博客与笔记速查

| 资源 | URL | 重点 |
|---|---|---|
| Recht "Outsider's Tour" Part 4 | https://archives.argmin.net/2018/02/08/lqr/ | LQR 作为 RL 起点 |
| Recht Part 6 (Policy Gradient) | https://archives.argmin.net/2018/02/20/reinforce/ | REINFORCE vs LQR 直觉 |
| Recht Part 13 (Coarse-ID) | https://archives.argmin.net/2018/05/11/coarse-id-control/ | model-based RL 核心 |
| Tedrake Ch.8 在线书 | https://underactuated.mit.edu/lqr.html | 含 Drake notebook |
| Tedrake Ch.11 Policy Search | https://underactuated.mit.edu/policy_search.html | LQR 视角看 policy gradient |
| CMU 16-745 notebooks | https://github.com/Optimal-Control-16-745/lecture-notebooks | Julia 实战 |
| Boyd EE363 全 slides | http://web.stanford.edu/class/ee363/lectures/allslides.pdf | 一次下载 |
| Bansal LQR 综合讲义 | https://smlbansal.github.io/Papers/lqrlecture.pdf | 凝练 30 页 |
| DR_CAN B 站合集 | https://space.bilibili.com/230105574 | 中文最严谨视频 |
| 知乎 LQR/iLQR 实战 | https://zhuanlan.zhihu.com/p/715102938 | 中文最完整 iLQR |

### 附录 C：证明索引

下面 4 条证明是本专题的"硬骨头"，建议**手写一遍后扫描存档**。

**C.1 Riccati 方程推导（PMP 路径）**
文献：Liberzon §6.1 + Anderson-Moore Ch.2。关键步骤：(1) Hamilton 函数 $H = \tfrac12 (x^\top Q x + u^\top R u) + p^\top (Ax + Bu)$；(2) $\partial_u H = 0 \Rightarrow u^* = -R^{-1}B^\top p$；(3) 协态方程 $\dot p = -\partial_x H = -Qx - A^\top p$；(4) 假设 $p = P(t)x$，对时间求导 $\dot p = \dot P x + P \dot x = \dot P x + P(Ax - BR^{-1}B^\top P x)$；(5) 比较得 $\dot P x = -A^\top P x - Qx + PBR^{-1}B^\top P x - P A x$，即 $-\dot P = A^\top P + PA - PBR^{-1}B^\top P + Q$。终端条件 $P(T) = Q_f$ 来自横截条件 $p(T) = Q_f x(T)$。

**C.2 ARE 稳定化解的存在唯一性**
文献：Anderson-Moore Ch.3 / Kwakernaak-Sivan §3.4 / Lancaster-Rodman Ch.7。证明大纲：(1) $(A,B)$ 可稳定 ⇒ DRE 在任意 $T$ 上有解 $P_T$；(2) $P_T$ 关于 $T$ 单调不减且有上界（用代价泛函的 Bellman 不等式）；(3) 由单调收敛 $P_T \to P_\infty$ 满足 ARE；(4) $(A, Q^{1/2})$ 可检测 ⇒ 闭环 $A_{cl} = A - BR^{-1}B^\top P_\infty$ Hurwitz；(5) 唯一性反证：另一稳定化解 $P'$ 满足 Lyapunov 方程 $A_{cl}^\top (P_\infty - P') + (P_\infty - P') A_{cl} = 0$，由 $A_{cl}$ Hurwitz ⇒ $P_\infty = P'$。

**C.3 分离原理 / Certainty Equivalence**
文献：Kwakernaak-Sivan §5.5 / Anderson-Moore Ch.7 / Bertsekas DP Vol.1 Ch.4.2。证明大纲：(1) 写出 LQG 值函数 $V_t(\mathcal I_t) = \mathbb E[x_t^\top P_t x_t + \alpha_t | \mathcal I_t]$，其中 $\mathcal I_t$ 为信息集；(2) 用塔性质把 $\mathbb E[x_t^\top P_t x_t | \mathcal I_t] = \hat x_t^\top P_t \hat x_t + \mathrm{tr}(P_t \Sigma_t)$；(3) 第二项 $\mathrm{tr}(P_t \Sigma_t)$ 不依赖 $u$（Kalman 估计误差与控制无关）；(4) 第一项关于 $\hat x_t$ 是确定性 LQR 问题，最优 $u^* = -K_t \hat x_t$；(5) $K_t$ 与无噪声 LQR 完全相同，$\hat x$ 由 Kalman 滤波器算出。

**C.4 Policy Gradient 全局收敛（Fazel et al. 2018）**
文献：Fazel et al. 2018 §3–4 (https://arxiv.org/abs/1801.05039)。证明大纲：(1) $C(K) = \mathbb E[\sum_t x_t^\top (Q + K^\top R K) x_t]$ for $u_t = -Kx_t$；(2) $C(K) = \mathrm{tr}(P_K \Sigma_0)$，$P_K$ 满足 Lyapunov 方程 $P_K = Q + K^\top R K + (A-BK)^\top P_K (A-BK)$；(3) 计算 $\nabla C(K) = 2 E_K \Sigma_K$，其中 $E_K = (R + B^\top P_K B) K - B^\top P_K A$，$\Sigma_K$ 为闭环状态协方差；(4) **gradient dominance**：$C(K) - C(K^*) \le \|\Sigma_{K^*}\| \cdot \mathrm{tr}(E_K^\top (R+B^\top P_K B)^{-1} E_K) \le \mu^{-1} \|\nabla C(K)\|_F^2$；(5) 由此 GD with stepsize $\eta = 1/L$ 满足 $C(K_{t+1}) - C(K^*) \le (1 - \mu/L)(C(K_t) - C(K^*))$，**线性收敛**到全局最优。

### 附录 D：分级习题（18 题）

**入门（D1–D6，每题 15–25 分钟）**

D1. 标量 LQR：$\dot x = ax + bu$, $J = \int_0^\infty (qx^2 + ru^2)dt$。手写 ARE 解析解 $P_\infty = (a + \sqrt{a^2 + b^2 q/r})r/b^2$，并讨论 $a < 0, a > 0$ 时的物理意义。

D2. 二维双积分器 $\ddot q = u$，写成状态空间，求 $Q = I, R = 1$ 时的 $K_\infty$。验证闭环极点为 $-1 \pm j$。

D3. 用 `scipy.linalg.solve_continuous_are` 重做 D2，比较手算与数值结果。

D4. 验证 `python-control` 的 `lqr(A,B,Q,R)` 返回与 D3 一致的 $K, P$。

D5. 给定 $A = \begin{pmatrix} 0 & 1 \\ -1 & 0 \end{pmatrix}, B = \begin{pmatrix} 0 \\ 1 \end{pmatrix}$，验证 $(A,B)$ 可控、$A$ 临界稳定。计算 controllability Gramian $W_c = \int_0^T e^{At} BB^\top e^{A^\top t}dt$ 解析形式。

D6. 写一段 20 行 Python，对单摆 $\ddot \theta + \sin\theta = u$ 在 $\theta = 0$ 线性化并设计 LQR，比较 $Q=I, R=1$ 与 $Q=10I, R=1$ 的暂态响应。

**中阶（D7–D13，每题 30–45 分钟）**

D7. 推导离散 LQR 与 DARE。验证当 $A = e^{A_c \Delta t}, B = \int_0^{\Delta t} e^{A_c \tau} B_c d\tau$ 时，$\Delta t \to 0$ 极限下 DARE 退化为 CARE。

D8. 实现 Hamilton 矩阵 + Schur 解 ARE 的算法（不用 scipy 内置）。在 D2 上验证。

D9. 实现 Newton-Kleinman 迭代解 ARE：给定稳定化 $K_0$，反复解 Lyapunov 方程 $A_k^\top P_k + P_k A_k = -(Q + K_k^\top R K_k)$, $K_{k+1} = R^{-1}B^\top P_k$。证明二次收敛。

D10. 复现 Doyle 1978 反例：构造 LQG 系统使其增益裕度 < 0.1。用 `python-control` 画 Bode 图。

D11. 对四旋翼线性化模型（Tedrake Ch.3.6）设计 hover LQR，用 Drake 仿真验证恢复时间 < 2s。

D12. 用 `FiniteHorizonLinearQuadraticRegulator`（Drake）做 cart-pole swing-up 后的稳定，与无 LQR 的开环 swing-up 比较。

D13. 实现 LSPI（Bradtke-Ydstie-Barto 1994 的 modern 版）：给定离散 LQR，仅用 rollout 数据 $(x_t, u_t, x_{t+1})$ 估计 Q 函数 $Q(x,u) = (x^\top, u^\top) H (x^\top, u^\top)^\top$，并提取最优策略。在 D7 系统上验证收敛到 DARE 解。

**进阶（D14–D18，每题 60–90 分钟）**

D14. 复现 Fazel et al. 2018 实验：实现 zero-order policy gradient（用两点 finite-difference 估梯度），在 $d_x = 5, d_u = 3$ 随机 LQR 上画 $C(K_t) - C(K^*)$ vs iteration（log scale 应见线性下降）。

D15. 复现 Dean et al. 2020 coarse-ID 实验：用 $N$ 步 rollout 数据估 $(\hat A, \hat B)$，画次优性 vs $N$（应见 $1/N$ 下降）。可用作者 GitHub: https://github.com/modestyachts/robust-adaptive-lqr

D16. 实现 iLQR（Li-Todorov 2004），在 cart-pole swing-up 上跑通；与 Crocoddyl 的 SolverDDP 输出对比。

D17. 实现 LQG 控制器：先设计 Kalman 滤波器（用对偶 ARE），再设计 LQR 反馈，组合后用 Drake 仿真带噪测量的 hover。

D18. 设计实验对比 model-based LQR（直接解 ARE）与 model-free policy gradient 在同一 LQR 系统上的样本复杂度。重现 Tu-Recht 2019 的 $\Theta(d_x)$ 倍差距结论。

### 附录 E：LQR 变体速查表

| 变体 | 动力学 | 代价 | 解 | 何时用 | 库 API |
|---|---|---|---|---|---|
| **连续有限时域 LQR** | $\dot x = Ax + Bu$ | $\int_0^T (x^\top Qx + u^\top Ru)dt + x_T^\top Q_f x_T$ | DRE backward, $K(t) = R^{-1}B^\top P(t)$ | 终端时间确定的批次任务 | `LinearQuadraticRegulator` (Drake) + 时间反向积分 |
| **连续无限时域 LQR** | $\dot x = Ax + Bu$ | $\int_0^\infty (x^\top Qx + u^\top Ru)dt$ | CARE, $K = R^{-1}B^\top P_\infty$ | 调节稳定（regulation） | `scipy.solve_continuous_are`, `control.lqr` |
| **离散有限时域 LQR** | $x_{k+1} = Ax_k + Bu_k$ | $\sum_{k=0}^{N-1} (x_k^\top Qx_k + u_k^\top Ru_k) + x_N^\top Q_f x_N$ | DRE 离散版 backward | 控制周期固定的 MPC 内核 | 自实现 + numpy |
| **离散无限时域 LQR** | $x_{k+1} = Ax_k + Bu_k$ | $\sum_{k=0}^\infty (x_k^\top Qx_k + u_k^\top Ru_k)$ | DARE, $K = (R + B^\top P B)^{-1} B^\top P A$ | 数字控制系统稳态调节 | `scipy.solve_discrete_are`, `control.dlqr`, MATLAB `dlqr/idare` |
| **TVLQR（轨迹跟踪）** | $\dot{\delta x} = A(t)\delta x + B(t)\delta u$ 沿名义 $(\bar x, \bar u)$ | $\int_0^T (\delta x^\top Q \delta x + \delta u^\top R \delta u)dt + \delta x_T^\top Q_f \delta x_T$ | 时变 DRE backward, $K(t)$ 时变 | 跟踪非线性轨迹 | Drake `FiniteHorizonLinearQuadraticRegulator` |
| **跟踪 LQR (LQT)** | $\dot x = Ax + Bu$ | $\int_0^T \|x - r(t)\|_Q^2 + \|u\|_R^2 dt$ | DRE + 仿射前馈项 $g(t)$ backward | 跟踪外部参考信号 $r(t)$ | Lewis-Vrabie-Syrmos Ch.4 |
| **输出反馈 LQR** | $\dot x = Ax + Bu, y = Cx$ | 同 LQR | 投影梯度法或带 $C$ 约束的 ARE（无闭式） | 不可测全状态 | 自实现，常用 LQG 替代 |
| **LQG** | LQR + 高斯噪声 + 噪声测量 | 期望 LQR 代价 | LQR ARE × Kalman ARE，$u = -K_\infty \hat x$ | 部分可观高斯系统 | MATLAB `lqg`, `kalman` |
| **iLQR** | $x_{k+1} = f(x_k, u_k)$ 非线性 | 一般非线性 | TVLQR backward + forward rollout 迭代 | 非线性轨迹优化 | Crocoddyl `SolverDDP/SolverFDDP`, Drake `TrajectoryOptimization` |
| **Constrained LQR (= linear MPC)** | 同离散 LQR + $u \in \mathcal U, x \in \mathcal X$ | 同 LQR + 终端约束/代价 | 在线 QP（active set / interior point） | 输入饱和、状态安全 | acados, OSQP, FORCES Pro |

### 附录 F：LQR 与 RL 方法对照表

| 方法 | 在 LQR 上的形式 | 收敛性 | 样本复杂度 | 模型需求 | 关键论文 |
|---|---|---|---|---|---|
| **Riccati 直接解** | 解 ARE | 一次完成 | $0$（已知模型） | 已知 $(A,B,Q,R)$ | Kalman 1960 |
| **Value Iteration** | $P_{k+1} = Q + A^\top P_k A - A^\top P_k B(R + B^\top P_k B)^{-1} B^\top P_k A$ | 线性收敛到 ARE 解 | $0$ | 已知模型 | Bertsekas DP Vol.1 |
| **Policy Iteration** | 交替：解 Lyapunov 给 $P_K$，更新 $K = (R + B^\top P B)^{-1}B^\top P A$ | 二次收敛（= Newton-Kleinman） | $0$ | 已知模型 | Hewer 1971 |
| **LSTD / LSPI** | 用 rollout 估 $Q(x,u) = z^\top H z$，$z = (x; u)$ | 收敛到 ARE（持续激励下） | $\mathcal O(d^2/\epsilon^2)$ | 模型无关 | Bradtke-Ydstie-Barto 1994 |
| **Policy Gradient (Vanilla)** | $K \leftarrow K - \eta \nabla C(K) = K - 2\eta E_K \Sigma_K$ | 线性收敛到 $K^*$（PL 不等式） | $\mathrm{poly}(d_x, d_u, 1/\epsilon)$ | 需 model 算 $\nabla C$ | Fazel et al. 2018 |
| **Natural Policy Gradient** | $K \leftarrow K - \eta \nabla C(K) \Sigma_K^{-1} = K - 2\eta E_K$ | 线性收敛，更快 | 同上 | 同上 | Fazel et al. 2018 |
| **Zero-Order / Random Search** | 两点 finite-difference 估 $\nabla C$ | 同 PG，常数差 | 多 $\mathcal O(d)$ 倍 | 模型无关 | Mania et al. 2018 (ARS) / Malik et al. 2020 |
| **Q-Learning (LQ)** | 学 $H$ 矩阵，$u = -H_{uu}^{-1} H_{ux} x$ | 收敛（Robbins-Monro 假设） | 类似 LSPI | 模型无关 | Bradtke et al. 1994 |
| **Certainty Equivalence (Coarse-ID)** | 估 $(\hat A, \hat B)$，再解 ARE | 次优性 $\sim \|\hat\theta - \theta\|^2$ | $\tilde{\mathcal O}((d_x+d_u)/\epsilon)$ — minimax 最优 | model-based | Mania-Tu-Recht 2019 |
| **System Level Synthesis (SLS)** | 鲁棒凸优化 over response maps | 给出 $H_2$ 次优界 | 同 CE | model-based | Dean et al. 2020 |
| **OFU / Optimism** | 在置信集内选最乐观 $(A,B)$ | $\tilde{\mathcal O}(\sqrt T)$ regret | minimax 最优 | model-based | Abbasi-Yadkori-Szepesvári 2011, Cohen et al. 2019 |
| **Thompson Sampling** | 从后验采样 $(A,B)$，解 LQR | $\tilde{\mathcal O}(\sqrt T)$ regret | model-based | Ouyang-Gagrani-Jain 2017 |
| **iLQR / DDP** | 沿名义轨迹反复解 TVLQR | 局部二次收敛 | 模型无关（用 AD） | 需 $f, \nabla f, \nabla^2 f$ | Li-Todorov 2004 / Tassa 2014 |
| **Guided Policy Search** | iLQR 生成 trajectory → 训神经网 policy | 经验性 | 实用样本量 | 需局部模型 | Levine-Koltun 2013 |



---

### 深入学习路线图

本节将 LQR/LQG 的学习分为核心与进阶两个档位，并给出详细的教材、学时和实验指导。

### 学习目标细化

完成本专题后，你应当能够：

1. **从两条独立路径推导 Riccati 方程**——既能用 PMP 沿协态方程 $\dot p = -\partial H/\partial x$ 假设 $p = P(t)x$ 导出微分 Riccati 方程 (DRE)；也能用动态规划假设值函数 $V(t,x) = \tfrac12 x^\top P(t)x$ 由 HJB 导出同一方程。能在面试中 10 分钟内推完连续与离散两种情形。
2. **判定无限时域 LQR 是否良定**——给出 $(A,B,Q,R)$ 时立刻判断 ARE 是否存在唯一稳定化对称半正定解（可稳定 + 可检测 + $R\succ 0$, $Q\succeq 0$），并能用 Lyapunov 论证证明闭环稳定。
3. **数值求解 ARE**——掌握 Hamilton 矩阵 + Schur 分解算法，理解为什么 `scipy.linalg.solve_continuous_are` 等所有现代求解器都基于这条路线；能解释何时改用 Newton-Kleinman 迭代或 doubling 算法。
4. **建立 LQR-Kalman-LQG 的对偶认识**——把 Kalman 滤波器视为 LQR 在时间反向 + $(A,B,Q,R) \to (A^\top, C^\top, W, V)$ 下的对偶，并用分离原理证明 LQG 控制器的最优性。理解 Doyle 1978 反例为什么不破坏分离原理本身。
5. **把 LQR 嵌入 RL 框架**——能写出 LQR 的 Bellman 算子，证明它是 PL 几何下的全局收敛策略梯度问题；能用 Dean-Mania-Matni-Recht 的 coarse-ID 框架估算样本复杂度；能解释为何 certainty equivalence 在 LQR 上达到 minimax 最优 regret。
6. **在机器人项目中部署 LQR/TVLQR/iLQR**——能用 Drake 的 `LinearQuadraticRegulator` 稳定四旋翼 hover、用 `FiniteHorizonLinearQuadraticRegulator` 跟踪轨迹；能解释 iLQR 为什么是"沿名义轨迹反复 LQR"，并把它接入 Crocoddyl 的 DDP 框架。

---

### 核心章节清单（带教材与学时）

下表为核心档位的 22 小时基线方案，章节按教材正文顺序，**学时含动手实验**。

| 模块 | 章节与教材 | 学时 | 关键产出 |
|---|---|---|---|
| **M1 有限时域 LQR 双重推导** | Liberzon §6.1 + Bertsekas Vol.1 Ch.3.2 + Tedrake Ch.8.1–8.2 | 3.5h | 手写 PMP 与 DP 两套推导，得到同一个 DRE |
| **M2 无限时域 LQR 与 ARE** | Anderson-Moore Ch.2–3 + Liberzon §6.2 + Kwakernaak-Sivan Ch.3 | 4h | 证明稳定化解的存在唯一性；ARE 数值实验 |
| **M3 可控/可观与 Riccati 良定性** | Kwakernaak-Sivan Ch.1 + Åström-Murray Ch.6–7 + Boyd EE363 Lec1–4 | 3h | 写出 Kalman 秩、PBH、Gramian 三种检验的 Python 代码 |
| **M4 LQR-LQG 对偶与分离原理** | Anderson-Moore Ch.5–7 + Zhou-Doyle-Glover Ch.14 + Kwakernaak-Sivan Ch.4–5 | 3.5h | 推导对偶方程；复现 Doyle 1978 反例 |
| **M5 LQR 作为 RL 特例** | Recht 2019 综述 + Fazel 2018 + Dean 2020 + Bertsekas DP Ch.3.2 | 4.5h | 实现 policy gradient + certainty-equivalence baseline |
| **M6 机器人实战：TVLQR/iLQR/MPC** | Tedrake Ch.8.3–8.7 + Manchester 16-745 Lec5–9 + Li-Todorov 2004 | 3.5h | Drake 四旋翼 LQR + Crocoddyl iLQR 跑通 |

进阶档位（25h）扩展项见下节。**每模块结束都需在自测题 + 习题 D 中各完成 2 题**，否则不得继续下一模块——这是控制系工程师常见的"会算不会判"陷阱的唯一防御。

---

### 进阶章节（+3h 扩展）

在核心档位基础上选做以下专题之一即可，**强烈推荐 ADV-1 与 ADV-2 任选**，因为它们直接关联 RL 方向的进一步学习：

**ADV-1 Policy Gradient 全局收敛的完整证明**（+1.5h）：精读 Fazel et al. 2018 §3–4，亲手验证 LQR cost $C(K)$ 的 PL 不等式 $\|\nabla C(K)\|_F^2 \ge \mu (C(K)-C^*)$，理解 gradient dominance 为何打破"非凸 ⇒ 局部极小"的直觉。

**ADV-2 Coarse-ID Control 与 SLS**（+1.5h）：精读 Dean et al. 2020 §3，理解如何把 $(\hat A, \hat B)$ 的高斯置信椭球转换成 System Level Synthesis 的鲁棒凸优化；用作者公开的 [SLSpy](https://github.com/sls-caltech/sls-python) 复现 Fig. 3。

**ADV-3 H₂/H∞ 与 LQR 的频域桥梁**（+1.5h）：Zhou-Doyle-Glover Ch.14 §14.4–14.10。理解 Kalman 1964 的回差判据，证明 LQR 至少有 60° 相位裕度与 (½, ∞) 增益裕度，并对照 Doyle 1978 看为何 LQG 丢失这些裕度。

**ADV-4 几何 Riccati：辛流形上的拉格朗日子空间**（+1.5h）：Liberzon §6.2 末附注 + Anderson-Moore Appendix B。把 Hamilton 矩阵 $\mathcal H$ 的稳定不变子空间识别为辛 $\mathbb R^{2n}$ 上的拉格朗日子空间，理解 Schur 解法的几何根源。这是连接到第一批李群、第三批 PMP 的辛几何视角。

---

### 核心教材对照

下表为本专题的"金字塔"参考书。**优先阅读顺序：Tedrake Ch.8 → Liberzon Ch.6 → Anderson-Moore Part I → Bertsekas Vol.1 Ch.3 → Kwakernaak-Sivan Ch.3-5 → Zhou-Doyle-Glover Ch.14 → Lewis-Vrabie-Syrmos Ch.11**。

| 教材 | 出版信息 | 角色 | 关键章节与链接 |
|---|---|---|---|
| **Anderson & Moore**, *Optimal Control: Linear Quadratic Methods* | Prentice-Hall 1990 / Dover 2007 (ISBN 0486457664) | LQG 工程圣经；含频域稳定裕度、loop recovery、controller reduction | Part I（基础）+ Part II（工程性质）+ Part III（LQG）；PDF: https://www.e-booksdirectory.com/details.php?ebook=3777 |
| **Bertsekas**, *Dynamic Programming and Optimal Control*, Vol. 1, 4th ed. | Athena Scientific 2017 | DP 视角推导 LQR，与值迭代/RL 衔接最自然 | **Ch. 3.2** "Linear Quadratic Problems"；目录预览: https://web.mit.edu/dimitrib/www/DP1_Short_View.pdf；解答: http://athenasc.com/DP_4thEd_theo_sol_Vol1.pdf |
| **Kwakernaak & Sivan**, *Linear Optimal Control Systems* | Wiley-Interscience 1972 | 经典数学严谨度最高，LQR/Kalman-Bucy/分离定理标准证明 | Ch. 3 (LQR), Ch. 4 (Kalman-Bucy), Ch. 5 (LQG)；Internet Archive 借阅: https://archive.org/details/linearoptimalcon0000kwak |
| **Zhou, Doyle & Glover**, *Robust and Optimal Control* | Prentice Hall 1996 (ISBN 0-13-456567-3) | LQR/H₂/H∞ 统一框架，鲁棒性视角 | **Ch. 14** "LQR and H₂ Control"，含 §14.4 稳定裕度、§14.9 分离定理；勘误: https://www.ece.lsu.edu/kemin/robust.htm |
| **Tedrake**, *Underactuated Robotics* | MIT 在线教材，2024 持续更新，CC 授权 | 机器人导向，含 Drake 代码示例 | **Ch. 8** LQR: https://underactuated.mit.edu/lqr.html；Ch. 11 Policy Search: https://underactuated.mit.edu/policy_search.html |
| **Liberzon**, *Calculus of Variations and Optimal Control Theory* | Princeton 2012 (ISBN 978-0-691-15187-8) | 简洁严谨，从变分法到 LQR 一气呵成 | Ch. 6 LQR；预印 PDF: https://liberzon.csl.illinois.edu/teaching/cvoc.pdf |
| **Lewis, Vrabie & Syrmos**, *Optimal Control*, 3rd ed. | Wiley 2012 | 唯一系统讲 ADP/RL 与经典 LQR 关系的教材 | Ch. 3 (LQR)、Ch. 4 (跟踪)、**Ch. 11** "RL and Optimal Adaptive Control"；PDF: https://lewisgroup.uta.edu/FL%20books/Lewis%20optimal%20control%203rd%20edition%202012.pdf |
| **Åström & Murray**, *Feedback Systems*, 2nd ed. | Princeton 2020 | 入门级状态空间+频域统一视角；配套 Murray *Optimization-Based Control* | Ch. 6–7；FBSwiki: https://fbswiki.org；OBC PDF: https://www.cds.caltech.edu/~murray/books/AM05/pdf/obc-author_21Dec21.pdf |

**单一推荐**：若只读一本，选 **Tedrake Ch.8 + Liberzon Ch.6** 组合——前者给工程直觉与代码，后者给数学严密性，两小时阅读量即可覆盖核心 70%。

---

### 关键定理速查（10 条）

下面 10 条定理是本专题的"骨架"，**每条都需自己手写至少一遍证明**或熟到可在白板上 5 分钟讲完。

**T1（有限时域 LQR 最优性）** 设 $\dot x = Ax + Bu$，代价 $J = \tfrac12 x_T^\top Q_f x_T + \tfrac12 \int_0^T (x^\top Q x + u^\top R u)\,dt$，$Q,Q_f \succeq 0$, $R \succ 0$。则最优反馈律为 $u^*(t) = -K(t)x(t)$，其中 $K(t) = R^{-1}B^\top P(t)$，$P(t)$ 为微分 Riccati 方程 $-\dot P = A^\top P + PA - PBR^{-1}B^\top P + Q$，$P(T)=Q_f$ 的唯一对称半正定解。

**T2（DRE 解的全局存在性）** 在 T1 假设下，DRE 在 $[0,T]$ 上存在唯一解，**不会有限时间爆破**——这是 LQR 的"奇迹"，与一般 Riccati 方程对照。证明依赖比较定理与值函数表示。

**T3（CARE 稳定化解的存在唯一性）** $(A,B)$ 可稳定且 $(A,Q^{1/2})$ 可检测 ⟺ 连续 ARE $A^\top P + PA - PBR^{-1}B^\top P + Q = 0$ 存在唯一对称半正定解 $P_\infty$，且闭环矩阵 $A_{cl} = A - BR^{-1}B^\top P_\infty$ Hurwitz。这是无限时域 LQR 的"主定理"。

**T4（Hamilton 矩阵谱结构）** 定义 $\mathcal H = \begin{pmatrix} A & -BR^{-1}B^\top \\ -Q & -A^\top \end{pmatrix}$。在 T3 条件下，$\mathcal H$ 没有纯虚特征值，且其稳定不变子空间 $\mathcal X_-$ 形如 $\operatorname{Im}\begin{pmatrix} I \\ P_\infty \end{pmatrix}$。这一结构是 **Schur 法解 ARE 的几何根据**。

**T5（LQR 闭环稳定性，Lyapunov 论证）** 取 $V(x) = x^\top P_\infty x$。沿闭环轨迹 $\dot V = -x^\top (Q + P_\infty B R^{-1} B^\top P_\infty) x \le 0$，再用 LaSalle 不变集 + 可检测性给出渐近稳定。

**T6（LQR 经典稳定裕度，Kalman 1964）** 单输入 LQR 的回差函数满足 $|1 + L(j\omega)| \ge 1, \forall \omega$，由此立刻得到 **≥ 60° 相位裕度** 与 **(1/2, ∞) 增益裕度**——LQR "天生鲁棒"的频域证据。

**T7（LQR-LQF 对偶）** 把 LQR $(A,B,Q,R)$ 在 $t \to T-t$ 的时间反演下与 Kalman 滤波器 $(A^\top, C^\top, W, V)$ 严格对偶。Kalman 滤波器的 Riccati 方程 $\dot \Sigma = A\Sigma + \Sigma A^\top - \Sigma C^\top V^{-1} C \Sigma + W$ 与 LQR 的 DRE 在变量替换下完全相同。

**T8（分离原理 / certainty equivalence）** 对线性 Gauss 系统 + 二次代价 + 白噪声扰动 + Gauss 测量噪声，LQG 最优控制律为 $u^*(t) = -K_\infty \hat x(t|t)$，其中 $K_\infty$ 由 LQR ARE 算出，$\hat x$ 由 Kalman 滤波器算出——**估计与控制可独立设计**。证明依赖条件期望的塔性质与 LQ 值函数的二次结构。

**T9（LQR 策略梯度的 Gradient Dominance, Fazel et al. 2018）** 定义 $C(K) = \mathbb E_{x_0 \sim \mathcal D}[\sum_{t=0}^\infty x_t^\top Q x_t + u_t^\top R u_t]$ with $u_t = -Kx_t$。则在所有稳定化 $K$ 处 $C(K) - C(K^*) \le \frac{\|\Sigma_{K^*}\|}{\sigma_{\min}(R)\sigma_{\min}(\Sigma_0)^2} \|\nabla C(K)\|_F^2$，即 **PL 不等式**成立。从而梯度下降 / 自然梯度 / 零阶随机搜索均全局收敛到 $K^*$。

**T10（LQR Sample Complexity, Dean-Mania-Matni-Recht 2020）** 用 $N$ 步独立 rollout 数据做最小二乘辨识 $(\hat A, \hat B)$ 后，coarse-ID + SLS 控制器的次优性满足 $C(\hat K) - C(K^*) \lesssim \frac{(d_x + d_u)\log(1/\delta)}{N}$，与参数数目数量级最优。这是 model-based RL 的样本复杂度黄金标准。

---

### 必读论文（10 篇）

10 篇论文按"经典源头 → 现代 RL 理论 → 算法工程"三条线分组。**优先级 ★★★ 必读 / ★★ 选读 / ★ 参考**。

**经典源头（理解 LQR 的诞生）**

1. ★★★ **Kalman, R. E. (1960)** "Contributions to the theory of optimal control" *Bol. Soc. Mat. Mexicana* 5:102–119. 状态空间 + LQR 的诞生论文。PDF: https://boletin.math.org.mx/pdf/2/5/BSMM(2).5.102-119.pdf
2. ★★★ **Kalman, R. E. (1960)** "A new approach to linear filtering and prediction" *ASME J. Basic Eng.* 82(1):35–45. DOI: https://doi.org/10.1115/1.3662552 卡尔曼滤波诞生，与 LQR 形成 LQG 对偶。
3. ★★ **Kalman, R. E. (1964)** "When is a linear control system optimal?" *ASME J. Basic Eng.* 86(1):51–60. DOI: https://doi.org/10.1115/1.3653115 inverse optimality + 回差判据，给出 LQR 鲁棒裕度的频域刻画。
4. ★★★ **Doyle, J. C. (1978)** "Guaranteed margins for LQG regulators" *IEEE TAC* 23(4):756–757. DOI: https://doi.org/10.1109/TAC.1978.1101812 一页反例打破"LQG 像 LQR 一样鲁棒"的幻想，催生 H∞ 与 μ 综合。

**现代 LQR-RL 理论（学生兴趣的核心）**

5. ★★★ **Recht, B. (2019)** "A Tour of Reinforcement Learning: The View from Continuous Control" *Annu. Rev. Control Robot. Auton. Syst.* 2:253–279. arXiv: https://arxiv.org/abs/1806.09460 把 LQR 立为连续控制 RL 的"果蝇模型"，**全篇必读**。
6. ★★★ **Dean, S., Mania, H., Matni, N., Recht, B., Tu, S. (2020)** "On the Sample Complexity of the Linear Quadratic Regulator" *Found. Comput. Math.* 20(4):633–679. arXiv: https://arxiv.org/abs/1710.01688 model-based 的样本复杂度黄金论文。
7. ★★★ **Fazel, M., Ge, R., Kakade, S. M., Mesbahi, M. (2018)** "Global Convergence of Policy Gradient Methods for the LQR" *ICML* PMLR 80:1467–1476. arXiv: https://arxiv.org/abs/1801.05039 policy gradient 在非凸 LQR 上全局收敛，PL 不等式登场。
8. ★★ **Mania, H., Tu, S., Recht, B. (2019)** "Certainty Equivalence is Efficient for Linear Quadratic Control" *NeurIPS*. arXiv: https://arxiv.org/abs/1902.07826 朴素 plug-in 控制器次优性是参数误差²，比预期快一阶。
9. ★★ **Simchowitz, M., Foster, D. J. (2020)** "Naive Exploration is Optimal for Online LQR" *ICML* PMLR 119:8937–8948. arXiv: https://arxiv.org/abs/2001.09576 在线 LQR 的 minimax regret 是 $\tilde\Theta(\sqrt T)$，ε-greedy 即可达到。
10. ★★ **Bradtke, S. J., Ydstie, B. E., Barto, A. G. (1994)** "Adaptive Linear Quadratic Control Using Policy Iteration" *ACC* 3475–3479. 把 Q-learning 引入 LQR 的开山小论文，LSTD/LSPI 的源头。

**附加（备查）**：Tu & Recht 2019 (https://arxiv.org/abs/1812.03565) model-based vs model-free 渐近差距；Cohen-Koren-Mansour 2019 (https://arxiv.org/abs/1902.06223) 首个高效 $\sqrt T$-regret 算法；Malik et al. 2020 (https://arxiv.org/abs/1812.08305) 零阶方法；Li & Todorov 2004 (https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf) iLQR 开山。

---

### C++/Python 库映射

下面是 LQR 工程实现的"五件套"，**每个库都至少跑通一个示例**。

**SciPy（最薄、最透明）**：
- `scipy.linalg.solve_continuous_are(A, B, Q, R, e=None, s=None, balanced=True)` 求 CARE，底层用 QZ 分解处理 Hamilton 束。文档：https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html
- `scipy.linalg.solve_discrete_are(...)` 离散版。文档：https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html
- 用法：`P = solve_continuous_are(A,B,Q,R); K = np.linalg.solve(R, B.T @ P)`。

**python-control（控制工程师默认库）**：
- `K, S, E = control.lqr(A, B, Q, R, N=0)` 直接返回反馈增益、ARE 解、闭环极点。文档：https://python-control.readthedocs.io/en/latest/generated/control.lqr.html
- `control.dlqr` 离散版；`control.ctrb`, `control.obsv` 可控/可观矩阵；`control.gram` 计算 Gramian。

**Drake（机器人优化控制）**：
- `LinearQuadraticRegulator(A, B, Q, R, N=[], F=[]) -> (K, S)` 直接接 `LinearSystem` 或 `System+Context`，自动线性化非线性系统。
- `FiniteHorizonLinearQuadraticRegulator(system, context, t0, tf, options)` **TVLQR**，沿名义轨迹做线性化 + 终端代价 $Q_f$，返回时变 $K(t), S(t)$。文档：https://drake.mit.edu/pydrake/pydrake.systems.controllers.html

**Crocoddyl（DDP/iLQR 工业级）**：
- `crocoddyl.ActionModelLQR(nx, nu, drift_free=True)` 离散 LQR action model；`crocoddyl.DifferentialActionModelLQR(nq, nu)` 连续微分形式，需配 `IntegratedActionModelEuler/RK` 离散化。主要作为 DDP 求解器的 unit-test/baseline，而非 Riccati 解算器。GitHub: https://github.com/loco-3d/crocoddyl

**MATLAB Control System Toolbox**（工业 + 教学事实标准）：
- `[K,S,P] = lqr(sys,Q,R,N)` / `dlqr(A,B,Q,R,N)`。
- ARE: `icare`/`idare`（推荐，R2019a+），`care`/`dare`（旧 API）。
- LQG: `kalman`, `lqg`, `lqi`, `lqgreg`。文档入口：https://www.mathworks.com/help/control/ref/lti.lqr.html

**acados（嵌入式 NMPC，无独立 LQR）**：通过 OCP 配置 + HPIPM 内部 Riccati 递推得到 LQR；`solver_options.qp_solver_ric_alg` 选经典 vs 平方根 Riccati。文档：https://docs.acados.org/python_interface/

**实战提示**：在 SLAM/机器人 stack 里，长期闭环用 `scipy.solve_continuous_are` + 手写反馈最稳；轨迹跟踪用 Drake `FiniteHorizonLinearQuadraticRegulator`；MPC 上线用 acados；做 RL benchmark 用 `python-control` 配 `gym`-like wrapper。

---

### 在线学习资源

**课程讲义（按推荐度排序）**

- **Boyd EE363 "Linear Dynamical Systems" (Stanford)**：所有 slides 仍可访问。
  - Discrete-time LQR: https://stanford.edu/class/ee363/lectures/dlqr.pdf
  - LQR via Lagrange: https://stanford.edu/class/ee363/lectures/lqr-lagrange.pdf
  - Continuous-time LQR: https://stanford.edu/class/ee363/lectures/clqr.pdf
  - Riccati 推导专题: https://stanford.edu/class/ee363/notes/riccati-derivation.pdf
- **Tedrake 6.832 "Underactuated Robotics" (MIT)**：在线书 + Drake notebooks，https://underactuated.mit.edu/lqr.html
- **Manchester 16-745 "Optimal Control & RL" (CMU)**：手写讲义 + Julia notebooks，https://github.com/Optimal-Control-16-745/lecture-notebooks ；YouTube: https://www.youtube.com/watch?v=Kj88Nory8ec
- **Levine CS285 "Deep RL" (Berkeley)**：从 RL 视角看 LQR/iLQR，Lec 10 slides: https://rail.eecs.berkeley.edu/deeprlcourse/static/slides/lec-10.pdf
- **Abbeel CS287 "Advanced Robotics" (Berkeley)**：Lec 5 LQR: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/slides/Lec5-LQR.pdf

**博客与综述**

- **Ben Recht "An Outsider's Tour of RL"** (LQR 视角)，Part 4 LQR: https://archives.argmin.net/2018/02/08/lqr/ ；Part 6 Policy Gradient: https://archives.argmin.net/2018/02/20/reinforce/ ；Part 13 Coarse-ID: https://archives.argmin.net/2018/05/11/coarse-id-control/
- **Steve Brunton "Control Bootcamp"** YouTube 列表，覆盖可控/LQR/Kalman: https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m

**中文资源**

- **DR_CAN 最优控制系列（B 站）**：LQR 详细推导 https://www.bilibili.com/video/BV1dm4y177tA/ ；案例分析 https://www.bilibili.com/video/BV11V4y1t7z7/ ；个人空间 https://space.bilibili.com/230105574
- **知乎专栏精选**：LQR 与 iLQR 实战 https://zhuanlan.zhihu.com/p/715102938 ；离散 LQR 拉格朗日推导 https://zhuanlan.zhihu.com/p/525603535 ；Underactuated Robotics 中文精读 https://zhuanlan.zhihu.com/p/142483990

---

### 时间预算

核心档位（22h 基线）：

| 模块 | 学时 | 阅读 | 推导/证明 | 编程实验 |
|---|---|---|---|---|
| M1 有限时域 LQR | 3.5h | 1.0 | 1.5 | 1.0 |
| M2 ARE 与无限时域 | 4.0h | 1.5 | 1.5 | 1.0 |
| M3 可控/可观 | 3.0h | 1.0 | 1.0 | 1.0 |
| M4 LQG/分离原理 | 3.5h | 1.5 | 1.5 | 0.5 |
| M5 LQR-RL 连接 | 4.5h | 2.0 | 1.0 | 1.5 |
| M6 机器人实战 | 3.5h | 1.0 | 0.5 | 2.0 |
| **合计** | **22h** | **8.0** | **7.0** | **7.0** |

进阶档位（25h）：在核心基础上加 ADV-1（Fazel 证明，1.5h）+ ADV-3（$H_2/H_\infty$ 桥梁，1.5h）；或选 ADV-2 + ADV-4 组合。**节奏建议**：连续 4 周，每周 5--6h，周末 2h 实战 + 周中 1h/天理论；不要一次性赶完，Riccati 的几何直觉需要"睡眠加固"。

---

### 章末自测题（5 道）

完成本专题后，闭卷在 90 分钟内做完以下 5 题。**通过线：4/5 正确，且每题需写出关键步骤而非只给答案**。

**Q1（Riccati 推导，20 分钟）** 给定 $\dot x = Ax + Bu$，$J = \int_0^T (x^\top Qx + u^\top Ru)dt + x_T^\top Q_f x_T$。**(a)** 用 PMP 写出 Hamilton 函数 $H$，求 $\partial H/\partial u = 0$ 得 $u^* = -R^{-1}B^\top p$，假设 $p = P(t)x$ 推出 DRE。**(b)** 用 DP 假设 $V = \tfrac12 x^\top P(t)x$，从 HJB $-\partial_t V = \min_u [\tfrac12 x^\top Q x + \tfrac12 u^\top R u + (\nabla_x V)^\top (Ax+Bu)]$ 推出同一 DRE。**(c)** 解释为什么这两条路径必然给出同一方程。

**Q2（ARE 良定性，15 分钟）** 给定 $A = \begin{pmatrix} 0 & 1 \\ 0 & 0 \end{pmatrix}, B = \begin{pmatrix} 0 \\ 1 \end{pmatrix}, Q = \mathrm{diag}(1, 0), R = 1$。**(a)** 检验 $(A,B)$ 可控性、$(A, Q^{1/2})$ 可检测性。**(b)** 解 ARE，得到 $P_\infty$ 与 $K_\infty$。**(c)** 验证闭环 Hurwitz。

**Q3（Hamilton 矩阵 + Schur，15 分钟）** 写出 Q2 中 Hamilton 矩阵 $\mathcal H \in \mathbb R^{4 \times 4}$，求其特征值，验证一对 $\pm$ 对称。说明如何从稳定不变子空间 $\begin{pmatrix} V_1 \\ V_2 \end{pmatrix}$ 恢复 $P_\infty = V_2 V_1^{-1}$。

**Q4（LQR-Kalman 对偶，15 分钟）** 写出 LQR 的 ARE 与 Kalman 滤波器稳态 ARE。给出明确的变量替换 $(A,B,Q,R) \leftrightarrow (?,?,?,?)$ 使两者形式上完全相同。解释为什么 SciPy 用 `solve_continuous_are` 同时支持 LQR 与 Kalman 是合理的。

**Q5（LQR-RL，25 分钟）** **(a)** 写出折扣 LQR 的 Bellman 算子 $T(V)(x)$，证明若 $V(x) = x^\top P x$ 则 $T(V)$ 仍为二次型，由此导出 ARE。**(b)** 给出 $C(K) = \mathbb E[x_0^\top P_K x_0]$ 关于 $K$ 的解析梯度（提示：用 Lyapunov 方程表示 $P_K$ 与状态协方差 $\Sigma_K$，最终 $\nabla C(K) = 2(R K - B^\top P_K (A - BK)) \Sigma_K$，可在 Fazel 2018 §3 核对）。**(c)** 解释为何这表明 policy gradient 在 LQR 上等价于"近似牛顿法 + 协方差预条件"。

---

### 易错陷阱汇总（10 条）

1. **混淆 PMP 协态 $p$ 与 Riccati $P$ 的角色**——前者是逐轨迹的对偶变量（向量），后者是状态独立的反馈增益生成矩阵；只有线性协态假设 $p = P(t)x$ 才把两者联系起来。这一假设的合法性来自 LQR 的二次值函数结构，对一般非线性系统不成立。

2. **把 `scipy.solve_continuous_are` 的输出直接用于离散系统**。**连续 ARE 与离散 DARE 是两个不同方程**；离散版本是 $P = A^\top P A - A^\top P B(R + B^\top P B)^{-1} B^\top P A + Q$，注意 $R + B^\top P B$ 可逆才有解。机器人控制周期 $\Delta t$ 大于 5ms 时差异显著。

3. **$R \succ 0$ 与 $R \succeq 0$ 不能混用**。$R$ 必须严格正定，否则 $u^* = -R^{-1}B^\top P x$ 无定义；$Q \succeq 0$ 即可，但需 $(A, Q^{1/2})$ 可检测，否则 $P$ 不稳定。

4. **以为 LQG = LQR + Kalman 就万无一失**。Doyle 1978 已证 LQG 没有保证的稳定裕度——一个微小的模型误差就能让整个闭环不稳。生产环境 LQG 要么做 LTR（loop transfer recovery），要么用 H∞ 替代。

5. **在欠驱动系统上对未线性化的非线性模型直接应用 LQR**。LQR 只对线性化点附近 $\mathcal O(\epsilon)$ 邻域有效；离开邻域后增益符号都可能错。Tedrake 第 8 章详述如何用 ROA（吸引域）估计安全边界。

6. **TVLQR 的终端代价 $Q_f$ 选错导致 P(t) 在 $t \to T$ 附近爆炸**。$Q_f$ 应为 LQR 无限时域 ARE 的解 $P_\infty$（cost-to-go 平稳），否则 backward Riccati 在终端附近震荡。

7. **可控但不稳定化 ≠ 可稳定**。可控蕴含可稳定，但可稳定（仅要求不稳模态可控）才是 ARE 良定的最小条件。在欠驱动机器人上常见——某些不稳模态在工作点不可控但稳定（无害），仍可设计 LQR。

8. **policy gradient 起点 $K_0$ 必须稳定化**。Fazel et al. 2018 假设算法从某稳定 $K_0$ 出发；若 $K_0$ 不稳定则 $C(K_0) = \infty$，梯度无定义。实战中需先用 nominal LQR 求 $K_0$ 再做 RL 微调。

9. **certainty equivalence ≠ 忽略不确定性**。Mania-Tu-Recht 2019 的 $\epsilon^2$ 速率仅对**完全可观**与适当激励下成立；对部分可观或激励不足的系统，需用 SLS / robust LQR / Thompson sampling。

10. **iLQR 的"LQR 部分"是局部二次近似**，不是真 LQR——它每次迭代解一个 TVLQR 子问题，但全局是 Newton-like 方法。把 iLQR 误用为 LQR 替代品会丢掉非线性收敛保证；务必接 line search 或 trust region。

---

### 综合练习题

**练习 LQR.1** ⭐（手算 ARE）：对一维不稳定系统 $\dot{x} = 2x + u$，$Q = 1$，$R = 1$，手算连续 ARE 的正定解 $P$。验证闭环极点 $a - b^2P/r = 2 - P$ 在左半平面。

**练习 LQR.2** ⭐⭐（离散 Riccati 收敛）：对 cart-pole 系统在倒立平衡点线性化，用 backward Riccati 递推从 $P_N = Q_f$ 出发。画出 $\|P_k - P_\infty\|$ 随 $N-k$ 的曲线，验证指数收敛速度。解释：收敛速率与 Hamilton 矩阵的哪些特征值有关？

**练习 LQR.3** ⭐⭐（LQG 实现）：在仿真中实现完整的 LQG 控制器（Kalman 滤波 + LQR 反馈）。(a) 对双积分器加过程噪声和测量噪声；(b) 比较"全状态反馈 LQR"vs"LQG（只测位置）"的跟踪性能；(c) 验证分离原理——独立调 $Q,R$ 和 $V,W$ 不会相互影响闭环稳定性。

**练习 LQR.4** ⭐⭐⭐（增益裕度验证）：对 MIMO LQR（$R = \rho I$），数值验证 Anderson-Moore 定理（phase margin $\ge 60°$）。方法：计算开环传递函数 $G(s) = K(sI - A)^{-1}B$ 的 Nyquist 图，验证它不进入以 $(-1,0)$ 为圆心的单位圆。改变 $\rho$ 观察增益裕度的变化。

**练习 LQR.5** ⭐⭐⭐（跨章 — LQR 作为 MPC 内核）：实现一个 unconstrained linear MPC（有限时域 LQR + receding horizon）。(a) 验证当 horizon $N \to \infty$ 时策略收敛到无限时域 LQR；(b) 加入 box 约束 $|u| \le 1$，观察 MPC 与 LQR 的行为差异；(c) 当约束不 active 时，验证 MPC 输出与 LQR 完全一致。

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| LQR 是过时的经典方法，RL 已经替代了它 | LQR 是现代 RL 理论的核心 benchmark，Policy Gradient 全局收敛的证明首先在 LQR 上完成（Fazel 2018） |
| Riccati 方程只是一个特殊的数学公式 | Riccati 方程是 HJB、DP、PMP、完成平方法四条独立路径的汇聚点，其不爆破性源于 Hamilton 矩阵的辛几何结构 |
| LQG = LQR + Kalman 滤波器，自动继承 LQR 的鲁棒性 | Doyle 1978 证明 LQG 没有保证的增益/相位裕度，一个微小的模型误差就能让闭环不稳定 |
| $Q$ 和 $R$ 越大控制效果越好 | $Q$ 和 $R$ 的相对比例才有意义：$Q/R$ 大则响应快但控制量大，$Q/R$ 小则响应慢但节能；绝对大小只影响 $P$ 的缩放 |
| LQR 的最优增益 $K$ 在全状态空间都有效 | $K$ 仅在线性化点的邻域内最优，远离平衡点时增益方向可能完全错误，需用 TVLQR 或 MPC |
| 可控性和可稳定性是同一回事 | 可控蕴含可稳定，但可稳定（仅要求不稳模态可控）才是 ARE 良定的最小条件 |
| iLQR 就是 LQR 的非线性版本 | iLQR 每次迭代解一个 TVLQR 子问题，但全局是 Newton-like 方法，必须配合 line search 使用 |
| Policy Gradient 在 LQR 上全局收敛意味着非凸优化总能找到全局最优 | 全局收敛依赖 LQR 代价函数满足 PL 不等式（gradient dominance），一般非凸问题不具备此性质 |

---

### 与前后专题的衔接

**承接（来自前批）**

- **第二批专题 1–3（凸分析/对偶/凸优化）**：LQR 是 KKT 系统块消元的标本——把 PMP 的协态方程视为大型 KKT 的对偶变量，Riccati 即为 Schur 补。Schur 解法的稳定子空间就是凸分析里的拉格朗日子空间。
- **第二批专题 4（非线性优化）**：iLQR 是 Gauss-Newton + Bellman 递推的混合物；信任域、line search、Wolfe 条件全部沿用。
- **专题 3.1--3.2（变分法 + PMP）**：LQR 是 PMP 的"全展开"案例——四条件全部退化为线性方程，HJB 退化为 Riccati。学完本专题再回看 PMP 会对"协态-值梯度"对偶有几何直觉。

**衔接（去往后续）**

- **专题 3.4（HJB 方程与黏性解）**：LQR 是 HJB 在 LQ 假设下的解析特例，是检验 HJB 数值算法（finite difference、policy iteration、SOC）的金标准。
- **专题 3.9（DDP/iLQR）**：iLQR 直接以 LQR 为内核；理解本专题后专题 3.9 阅读量减半。Crocoddyl 的 `ActionModelLQR` 即为 unit test。
- **专题 3.11--3.13（MPC 系列）**：unconstrained 线性 MPC = 有限时域 LQR + receding horizon；约束 MPC 在没有 active constraint 时退化为 LQR。理解 LQR 即理解 MPC 内核。
- **第五批（状态估计 / SLAM）**：Kalman 滤波器 = LQR 的对偶；EKF 在测量更新阶段解一个微型 LQ 问题。LQG 直接复用 SLAM 的状态估计 + 控制律设计。
- **学习控制 / RL 理论**：LQR 的 RL 连接（Policy Gradient 全局收敛、Coarse-ID 样本复杂度）是整个 RL 理论的"证明范本"，所有 sample complexity / regret 论证都先在 LQR 上验证再推广。

---

### 本章总结

LQR 不是一个"老控制方法"，而是 **现代连续控制 RL 的果蝇**：足够简单可证明，足够复杂有代表性。学习本章的核心投入应集中在三件事上——(1) 把 PMP 与 DP 两条路径推导到可以随时复现；(2) 把 ARE 数值解、可控/可观、闭环稳定性串成一条因果链；(3) 把 Fazel-Dean-Mania-Simchowitz 这条 RL 理论叙事吃透。

最深的 takeaway 有三条。**第一，LQR 的"奇迹"不在线性二次本身，而在 Riccati 方程不爆破——这是辛几何里 Hamilton 矩阵的稳定不变子空间永远存在的几何事实**。理解这一点后，对一般 HJB 数值困难（值函数可能非光滑、最优控制可能 chattering）会有正确的悲观。**第二，LQG 的分离原理是 RL 理论中"决策-估计可分"梦想的唯一精确实例；任何更复杂的设定（部分可观 RL、POMDP）都要为打破这一对偶付出复杂度代价**。**第三，policy gradient 在 LQR 上全局收敛说明"非凸 ≠ 不可学"，但代价是 $d_x$ 阶的样本复杂度损失（Tu-Recht 2019）；这给 RL 工程师一个清晰的判据：能用 model-based 就别用 model-free**。

完成本章后建议立刻进入 HJB 方程与 DDP/iLQR 专题，它们会让你在四旋翼、机械臂、足式机器人上把 LQR 的内核反复重用。RL 方向则推进到 Recht 综述 + Fazel + Dean，三篇精读后即可独立阅读 minimax LQR、robust adaptive control、neural-LQR 全部当代文献。

---

