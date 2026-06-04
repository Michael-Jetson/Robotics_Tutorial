# 动态规划与 Bellman 方程

> **定位**：博士前数学路线图 · 第三批 · 专题 3.3（离散时间）
> **前置**：3.1 变分法与 Euler-Lagrange 方程、3.2 Pontryagin 极大值原理（PMP）
> **后续**：3.4 HJB、3.5 LQR 与 Lyapunov、3.9 iLQR/DDP、3.11 线性 MPC、3.12 非线性 MPC、3.13 随机/鲁棒 MPC、第六批（6.1–6.8）强化学习理论

---

## 前置自测

📋 **前置自测**（答不出 2 题以上，先回 3.1/3.2 与泛函分析复习）

1. 什么是完备度量空间？Banach 不动点定理的陈述是什么？（泛函分析 B3）
2. 离散 PMP 的协态方程 $\lambda_k = \nabla_x L_k + [\nabla_x f_k]^\top \lambda_{k+1}$ 是怎么推出来的？（3.2.6）
3. 什么是矩阵谱半径？$(A,B)$ 可稳定性意味着什么？（线性代数）
4. 马尔可夫性（无记忆性）的精确数学表述是什么？（概率论）
5. 什么是 $\sup$-范数（$\|\cdot\|_\infty$）？为什么有界函数空间在此范数下完备？（泛函分析）

---

## 本章目标

学完本章后，你将能够：

1. **从零推导** Bellman 最优性原理并给出严格反证法证明，理解其与最优子结构的关系
2. **证明** Bellman 算子的 $\gamma$-压缩性，由此推出值迭代的几何收敛率
3. **区分** VI、PI、MPI 三种算法的收敛性质与工程适用场景
4. **直接从 Bellman 方程推导** LQR 的 Riccati 差分方程，理解 PMP 与 DP 的对偶
5. **精确描述** 维数灾难的三重含义及五种主要缓解策略
6. **建立** 从离散 Bellman 方程到连续 HJB 方程的极限过渡
7. **将** MPC、iLQR、DQN、Dijkstra 统一归入 Bellman 方程的求解框架

---

## 知识树

```
动态规划与 Bellman 方程
├── 根：最优决策问题的两种求解哲学（轨迹级 PMP vs 场级 DP）
├── 主干1：Bellman 最优性原理
│   ├── 严格陈述与证明（反证法）
│   ├── 反例分析（何时失效）
│   └── 与 CLRS "最优子结构" 的对应
├── 主干2：离散 DP 算法族
│   ├── 值迭代 (VI) + Banach 压缩映射收敛证明
│   ├── 策略迭代 (PI) + 有限步终止 + Newton 解释
│   ├── Modified PI (MPI) + 与 Actor-Critic 的关系
│   └── 收敛速度对比与工程选择
├── 主干3：LQR 作为 DP 精确可解特例
│   ├── 从 Bellman 方程直接推出 Riccati
│   ├── 无限时域 → DARE
│   ├── 确定性等价原理 (LQG 分离)
│   └── 与 PMP 路径的闭环验证
├── 主干4：维数灾难
│   ├── 三重维数灾的精确含义
│   ├── 五种缓解策略
│   └── 近似 DP 与神经网络的理论边界
├── 主干5：随机 DP 与 MDP
│   ├── MDP 形式化
│   ├── 随机 Bellman 方程
│   ├── Blackwell 最优性
│   └── 期望代价与风险敏感变体
├── 主干6：从离散到连续（HJB 预告）
│   ├── Taylor 展开极限过程
│   ├── Viscosity solution 简介
│   └── 两种离散化范式
└── 分支：应用与延伸
    ├── 图搜索 = DP 的半环视角
    ├── 博弈论 DP 与鲁棒 MPC
    └── 与强化学习的逻辑链
```

---

## §3.3.0 战略定位：从轨迹级必要条件到场级充分条件 ⭐

### 动机：为什么需要动态规划？

在专题 3.2 中，我们学习了 Pontryagin 极大值原理 (PMP)。PMP 沿一条最优轨迹给出一阶必要条件——共态（协态）$\lambda_k^*$ 只需定义在这条轨迹上。这已经足够强大：iLQR、DDP、shooting 方法都基于 PMP。但 PMP 有两个根本局限：

> **术语统一说明**：co-state / adjoint 在中文文献中有三种译法——"共态"（专题 3.2 使用）、"协态"、"伴随变量"。三者完全等价，本专题沿用"协态"以与 Bertsekas 中译本一致，但在引用专题 3.2 结论时可能出现"共态"字样——请读者理解为同一概念。

1. **仅是必要条件**：满足 PMP 的轨迹可能是鞍点而非极小值
2. **仅对单一初始条件有效**：换一个 $x_0$ 就要重新求解整个边值问题

> **本质洞察**：PMP 好比"沿着一条河流找到最低点"——你只看到了河流经过的那条路径。DP 则是"画出整座山的等高线图"——你对每一个位置都知道最优下山路线。

### 如果不用 DP 会怎样？

设想一个四足机器人的 MPC 控制器：每 30ms 需要对当前状态 $x_0$ 求解一个最优控制问题。如果仅用 PMP（shooting 方法），每次都要从头猜测初始协态 $\lambda_0$、做多次迭代才能收敛到边值问题的解。而如果预先计算了值函数 $V(x)$，则直接查表 $u^* = \arg\min_u \{L + V(f(x,u))\}$ 即可——这就是 DP 给出的**最优反馈律**。

当然，"预先计算整个状态空间的 $V$" 在高维下不可行（这正是维数灾难），但理解 DP 的理论结构是理解所有近似方法（MPC、iLQR、RL）的前提。

### 历史：Bellman 1957 的贡献

Richard Bellman（1920–1984）在 RAND Corporation 工作期间，于 1953-1957 年间系统发展了动态规划理论。他在 1957 年的专著 *Dynamic Programming*（Princeton University Press, ISBN 978-0-691-14668-3）中首次：

- 提出"最优性原理"（Principle of Optimality, p.83）
- 创造"curse of dimensionality"一词
- 将递归决策问题统一为函数方程（Bellman 方程）

> **反事实推理**：如果 Bellman 没有提出最优性原理，我们仍然可以用 PMP 解决最优控制问题。但我们将缺乏：(1) 最优反馈律的理论保证；(2) VI/PI 等迭代算法的收敛性框架；(3) 整个强化学习的理论基础。换言之，RL 的全部数学正当性都建立在 Bellman 1957 的工作之上。

### DP 与 PMP 的精确对偶

| 维度 | PMP（轨迹级） | DP（场级） |
|------|--------------|-----------|
| 定义域 | 单条最优轨迹 | 整个状态空间 |
| 条件类型 | 一阶必要条件 | 充分条件（全局最优） |
| 核心对象 | 共态（协态）$\lambda_k$ | 值函数 $V(x)$ |
| 关系 | $\lambda_k = \nabla_x V_k(x_k^*)$ | |
| 计算代价 | $\mathcal{O}(n)$ ODE/差分方程 | $\mathcal{O}(N^d)$ 内存（维数灾难） |
| 工程实现 | shooting、collocation | VI、PI、MPC、RL |
| 适用场景 | 轨迹优化（已知初态） | 反馈控制（任意初态） |

**符号约定说明**：本专题的 Bellman 方程和离散 Hamiltonian 采用**极小化约定**（$V = \min_u\{L + V'\}$，$H_k = L_k + \lambda^\top f_k$），与 Bertsekas 教材和下一专题（3.4 HJB）一致。专题 3.2（PMP）使用 Pontryagin 原始的极大化约定（$H_{\max} = \lambda^\top f - L$，$u^* = \arg\max H_{\max}$）。两者等价（$H_k = -H_{\max}$），但符号差异经常是新手出错的根源——请始终检查当前上下文使用的是哪种约定。

**这一对偶是现代机器人学全部决策-控制-学习算法的理论脊柱**：
- **MPC**（3.11–3.13）= 沿退行视界反复做有限时域 DP
- **iLQR/DDP**（3.9）= 在名义轨迹邻域做局部二次 DP
- **强化学习全部算法**（第六批）= 从样本近似求解 Bellman 方程
- **Dijkstra/A\***（运动规划）= 确定性无折扣 DP 的正向求解器

---

## §3.3.1 离散时间最优控制问题的标准形式 ⭐

### 动机

在写 Bellman 方程之前，我们必须先严格定义"什么是离散时间最优控制问题"。这不是走形式——不同的问题结构（有限/无限时域、确定/随机、折扣/平均）对应的 Bellman 方程形式不同，收敛性质也不同。

### 严格定义

**有限时域确定性问题 (DOCP)**：给定离散动力学 $x_{k+1}=f_k(x_k,u_k)$，$x_k\in\mathcal{X}_k\subseteq\mathbb{R}^n$，$u_k\in\mathcal{U}_k(x_k)\subseteq\mathbb{R}^m$，阶段代价 $L_k(x_k,u_k)$，末端代价 $\Phi(x_N)$，初始态 $x_0$ 给定。求策略序列 $\pi=\{\mu_0,\ldots,\mu_{N-1}\}$ 使
$$J_\pi(x_0)=\Phi(x_N)+\sum_{k=0}^{N-1}L_k(x_k,\mu_k(x_k))$$
最小，其中 $x_{k+1}=f_k(x_k,\mu_k(x_k))$。

**有限时域随机问题 (SOCP)**：动力学改写为 $x_{k+1}=f_k(x_k,u_k,w_k)$，$w_k$ 独立取值于 $\mathcal{W}_k$ 具有已知分布，目标函数取期望
$$J_\pi(x_0)=\mathbb{E}_{w_0,\ldots,w_{N-1}}\!\left[\Phi(x_N)+\sum_{k=0}^{N-1}L_k(x_k,\mu_k(x_k),w_k)\right].$$

**无限时域 $\gamma$-折扣问题**：$N\to\infty$，代价为 $\sum_{k=0}^\infty\gamma^kL(x_k,u_k,w_k)$，$\gamma\in[0,1)$。假设 $L$ 有界。

**无限时域平均代价问题**：目标为 $\limsup_{N\to\infty}\frac{1}{N}\mathbb{E}\sum_{k=0}^{N-1}L(x_k,u_k,w_k)$。

### 四个维度的分类表

| 维度 | 选项 A | 选项 B |
|------|--------|--------|
| 时域 | 有限时域 $N<\infty$（MPC、DDP） | 无限时域（经典 LQR、RL） |
| 动态 | 确定性 $f(x,u)$（轨迹优化） | 随机 $f(x,u,w)$（MDP、stochastic MPC） |
| 代价聚合 | 折扣 $\sum\gamma^kL$（RL 主流） | 平均 $\lim\frac{1}{N}\sum L$（持续运行任务） |
| 信息模式 | 完全状态可观 | 部分可观（POMDP，超出本专题） |

### 工程对应与物理直觉

| 机器人任务 | 对应问题类型 | 理由 |
|-----------|-------------|------|
| 倒立摆稳定 | 无限时域 $\gamma$-折扣 LQR | 持续运行，线性化有效 |
| 抓取与放置 | 有限时域确定性 DOCP | 有明确的终止时间，确定性模型足够 |
| 风中无人机追踪 | 有限时域随机 SOCP | 风扰动为随机过程 |
| 仓储机器人 24/7 运行 | 平均代价问题 | 永不终止，关心长期平均效率 |

**折扣因子 $\gamma$ 的物理意义**：$\gamma$ 是"未来代价的时间贴现率"。在 $\gamma=0.99$、控制频率 100 Hz 下，有效视界约 $1/(1-\gamma)=100$ 步 = 1 秒，这恰是人类行走控制的典型 preview 时间。$\gamma=0.999$ 对应 10 秒视界——适合慢速任务如导航。

> **类比**：折扣因子类似于经济学中的"贴现率"——$\gamma=0.99$ 意味着"明天的一块钱只值今天的 0.99 元"。在机器人学中，这对应着"远处的代价不如近处的代价重要"，这既反映了模型预测能力的衰减，也是数学上保证级数收敛的需要。

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 1 | 混淆"策略"与"控制序列" | 策略 $\mu_k(x)$ 是从状态到动作的**映射**（反馈律）；控制序列 $\{u_0,\ldots\}$ 是开环的。DP 求的是策略。 |
| 2 | $\gamma=1$ 时假设级数收敛 | $\gamma=1$ 必须额外假设（如有吸收态、有限时域）才保证代价有限 |
| 3 | 忽略动作约束 $u\in\mathcal{U}(x)$ | 约束使 Bellman 方程的 $\min$ 变为约束优化，LQR 的解析解不再适用 |

### 练习

1. 写出一个双积分器 $x_{k+1} = x_k + v_k\Delta t$, $v_{k+1} = v_k + u_k\Delta t$ 的有限时域 DOCP 标准形式（定义状态、动作、代价）
2. 对于 $\gamma=0.95$ 和 $\gamma=0.999$，分别计算有效视界。如果控制频率为 500Hz，哪个 $\gamma$ 更适合四足机器人的步态控制？

---

## §3.3.2 Bellman 最优性原理：严格陈述与完整证明 ⭐⭐

### 动机：为什么需要"最优性原理"？

在普通优化中（如 $\min_x f(x)$），我们不需要什么"原理"来拆解问题——直接求梯度即可。但序贯决策问题不同：我们面对的不是"一次性选择最优 $x$"，而是"在每个时间步根据当前状态选择最优动作"。这意味着最优策略是一个**函数序列** $\{\mu_0, \mu_1, \ldots\}$，搜索空间巨大。

Bellman 最优性原理告诉我们：**不需要一次性搜索整个策略序列**。可以逐步反向求解——先解最后一步的子问题，再解倒数第二步的子问题（利用已解的最后一步结果），以此类推。这种"分而治之"的合法性，正是最优性原理所保证的。

> **类比**：最优性原理好比"最短路径的任意子路径仍是最短路径"。如果北京到广州的最短路经过武汉，则北京-武汉段和武汉-广州段也分别是各自的最短路。这不是显然的——对"最美路径"就不成立（北京-广州的最美路径的子段未必是子段的最美路径）。关键在于"代价可加"。

### Bellman 1957 原话

**Bellman 1957 原话**（*Dynamic Programming*, Princeton University Press, p.83）：

> *"An optimal policy has the property that whatever the initial state and initial decision are, the remaining decisions must constitute an optimal policy with regard to the state resulting from the first decision."*

### 现代严格形式

**Proposition 3.3.1（最优性原理，Bertsekas Vol.I, 4th ed. 2017, Prop. 1.3.1）**：设 $\pi^*=\{\mu_0^*,\ldots,\mu_{N-1}^*\}$ 是 SOCP 的最优策略，$J^*$ 是最优值。固定任意阶段 $i\in\{0,\ldots,N-1\}$ 与任意在 $\pi^*$ 下可达的状态 $x_i$。考虑以 $x_i$ 为初始状态、阶段 $i,\ldots,N-1$ 上的子问题：

$$\min_{\{\mu_i,\ldots,\mu_{N-1}\}}\mathbb{E}\left[\Phi(x_N)+\sum_{k=i}^{N-1}L_k(x_k,\mu_k(x_k),w_k)\right]$$

则**截断策略** $\{\mu_i^*,\mu_{i+1}^*,\ldots,\mu_{N-1}^*\}$ 对此子问题也最优。

### 完整严格证明（反证法 + 子策略分解）

**步骤 1（反设）**：假设 $\{\mu_i^*,\ldots,\mu_{N-1}^*\}$ 对子问题非最优。则存在策略 $\{\mu_i',\ldots,\mu_{N-1}'\}$ 使子问题成本严格更低：

$$J^{\mathrm{sub}}_{\pi'}(x_i) < J^{\mathrm{sub}}_{\pi^*}(x_i)$$

**步骤 2（构造拼接策略）**：定义新的全局策略

$$\pi'':=\{\mu_0^*,\ldots,\mu_{i-1}^*,\mu_i',\ldots,\mu_{N-1}'\}$$

即保持前 $i$ 步不变，从第 $i$ 步开始使用假设中的更优子策略。

**步骤 3（代价可加性与马尔可夫性的关键作用）**：

总代价可以写为前缀（前 $i$ 步）加后缀（后 $N-i$ 步）。由于：
- 代价具有**可加结构**：$J = (\text{前} i \text{步代价}) + (\text{后} N-i \text{步代价})$
- 动力学具有**马尔可夫性**：$x_{k+1}$ 只依赖 $x_k, u_k, w_k$，不依赖更早的历史

因此后缀代价**只依赖** $x_i$（而非整个历史），且前缀代价不受后缀策略变化的影响：

$$J_{\pi''}(x_0) = \underbrace{\mathbb{E}\left[\sum_{k=0}^{i-1}L_k(x_k,\mu_k^*(x_k),w_k)\right]}_{\text{前缀代价，与 }\pi^*\text{ 相同}} + \underbrace{\mathbb{E}\left[J^{\mathrm{sub}}_{\pi'}(x_i)\right]}_{< \mathbb{E}[J^{\mathrm{sub}}_{\pi^*}(x_i)]}$$

**步骤 4（矛盾）**：由步骤 3，

$$J_{\pi''}(x_0) < J_{\pi^*}(x_0)$$

这与 $\pi^*$ 是最优策略的假设矛盾。证毕。$\square$

### 证明中的关键假设分析

| 假设 | 在证明中的作用 | 若违反会怎样 |
|------|--------------|-------------|
| **代价可加性** | 允许将总代价分解为前缀+后缀 | 乘积型代价 $\prod L_k$ 时原理可能失效 |
| **马尔可夫性** | 保证后缀代价只依赖 $x_i$，不依赖历史 | 非马尔可夫时需扩充状态（信息状态） |
| **策略为确定性函数** | 保证拼接后仍是合法策略 | 对随机策略原理仍成立（取期望） |
| **$x_i$ 在 $\pi^*$ 下可达** | 保证拼接策略物理上可实现 | 不可达的状态不在讨论范围内 |

### 反例分析：什么时候最优性原理失效？

**反例 1：代价非可加（乘积型）**

考虑 $J = \prod_{k=0}^{N-1} L_k(x_k, u_k)$，其中 $L_k > 0$。全局最优的后缀子策略不一定是子问题最优，因为前缀的"乘积因子"会改变后缀的相对重要性。

修复方法：取对数变换 $\log J = \sum \log L_k$，回到可加结构。

**反例 2：状态非马尔可夫（隐含历史依赖）**

设决策者只能观测 $y_k = h(x_k) + v_k$（噪声观测），其最优决策依赖于整个观测历史 $\{y_0, \ldots, y_k\}$。此时"状态"$x_k$ 不满足马尔可夫性（相对于可用信息而言）。

修复方法：将"信息状态"（如后验分布 $b_k = P(x_k | y_0, \ldots, y_k)$）作为新的马尔可夫状态，在信念空间上做 DP——这就是 POMDP 的本质。

**反例 3：风险敏感代价（指数型）**

$J = \mathbb{E}[\exp(\sum L_k)]$ 不等于 $\exp(\sum \mathbb{E}[L_k])$。指数的非线性破坏了可加分解。

修复方法：使用对数变换或 Denardo 的抽象 DP 框架（半环 DP）。

> **反事实推理**：如果 Bellman 没有要求"代价可加"，DP 理论将极大受限。幸运的是，大多数工程问题（能量最小化、时间最短、距离最短）天然具有可加结构。少数例外（如鲁棒控制中的 min-max、金融中的风险敏感控制）需要专门的修改，但核心思想仍保留。

### 与 CLRS "最优子结构"的对应

*Introduction to Algorithms*（Cormen et al., 3rd ed., §15.3）列出 DP 适用性两条件：
- **Optimal substructure**：问题最优解包含子问题最优解
- **Overlapping subproblems**：递归反复遭遇相同子问题

这恰是 Bellman 最优性原理在算法教科书中的翻译。区别在于 CLRS 面向离散组合问题（字符串、图），而 Bellman 面向连续状态控制问题。但数学本质完全相同。

### 从最优性原理直接推出 DP 算法

由最优性原理，定义值函数（cost-to-go）：

$$V_k(x) := \min_{\{\mu_k,\ldots,\mu_{N-1}\}} \mathbb{E}\left[\Phi(x_N) + \sum_{j=k}^{N-1} L_j\right], \quad x_k = x$$

则 DP 反向递推算法为：

$$\boxed{V_N(x) = \Phi(x), \qquad V_k(x) = \min_{u\in\mathcal{U}_k(x)} \mathbb{E}_{w_k}\left[L_k(x,u,w_k) + V_{k+1}(f_k(x,u,w_k))\right]}$$

最优反馈律：$\mu_k^*(x) \in \arg\min_{u} \{L_k(x,u) + V_{k+1}(f_k(x,u))\}$。

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 1 | 把原理理解为"所有最优策略的截断都最优" | 原理只保证"存在一个最优截断"，并列冗余策略不受约束 |
| 2 | 用原理证明局部最优 | 原理作用于全局最优；局部最优需 PMP |
| 3 | 忽略"$\pi^*$ 下可达"条件 | 对不可达状态，原理无意义 |
| 4 | 代价非可加（max 型、乘积型）误用标准 DP | 切换到 multiplicative DP 或 Denardo 抽象框架 |

### 练习

1. 对一个简单的 3 步确定性问题（$N=3$，状态空间 $\{1,2,3\}$），手写 DP 反向递推过程
2. 构造一个代价为 $\max_k L_k$ 的问题实例，证明标准 Bellman 分解不成立。提出修复方法。
3. （跨章）回忆 3.2 PMP 给出的是必要条件。给出一个 PMP 满足但非全局最优的例子，说明为什么 DP 不会犯这个错。

---

## §3.3.3 值函数与协态的对偶：DP 与 PMP 的统一 ⭐⭐

### 动机

在 3.2 中我们辛辛苦苦推导了离散 PMP 的协态方程。在本节中，我们将揭示一个惊人的事实：**协态就是值函数的梯度**。这不仅在概念上统一了两种方法，还为实际计算提供了新的工具——例如，iLQR 中的"反向 Riccati 扫描"同时计算值函数 Hessian（$P_k$）和梯度（$s_k$），二者分别是协态的线性化系数和偏移。

### 核心定理

**Proposition 3.3.3（Bertsekas Vol.I §3.3；Kirk 2004 Ch.3）**：若 $V_{k+1}$ 在 $f_k(x_k^*,u_k^*)$ 处可微、$u_k^*$ 是内点且一阶最优条件成立，则

$$\boxed{\lambda_k^* := \nabla_x V_k(x_k^*)}$$

满足离散 PMP 伴随方程：

$$\lambda_k^* = \nabla_x L_k(x_k^*,u_k^*) + [\nabla_x f_k(x_k^*,u_k^*)]^\top \lambda_{k+1}^*$$

终端条件 $\lambda_N^* = \nabla\Phi(x_N^*)$，且 $u_k^* \in \arg\min_u H_k(x_k^*,u,\lambda_{k+1}^*)$，其中 $H_k := L_k + \lambda^\top f_k$（极小化约定，对应 Bertsekas/Kirk；专题 3.2 使用极大化约定 $H_{\max} = \lambda^\top f - L$，$u^* = \arg\max H_{\max}$。两者等价：$H_k = -H_{\max}$）。

### 完整证明（包络定理方法）

对 Bellman 方程 $V_k(x) = \min_u \{L_k(x,u) + V_{k+1}(f_k(x,u))\}$ 两侧关于 $x$ 求梯度。

设 $u^*(x)$ 为取得极小的控制，则：

$$\nabla_x V_k(x) = \nabla_x L_k(x,u^*) + [\nabla_x f_k(x,u^*)]^\top \nabla_x V_{k+1}(f_k(x,u^*))$$
$$\quad + \underbrace{[\nabla_x u^*(x)]^\top \left\{\nabla_u L_k + [\nabla_u f_k]^\top \nabla V_{k+1}\right\}}_{= 0 \text{ 由一阶最优条件}}$$

包络项为零（因为 $u^*$ 满足 $\nabla_u L_k + [\nabla_u f_k]^\top \nabla V_{k+1} = 0$）。

在最优轨迹 $x_k^*$ 处评估，并代入 $\lambda_k := \nabla_x V_k(x_k^*)$，得到：

$$\lambda_k^* = \nabla_x L_k(x_k^*,u_k^*) + [\nabla_x f_k(x_k^*,u_k^*)]^\top \lambda_{k+1}^*$$

这正是离散 PMP 的伴随方程！$\square$

> **本质洞察**：协态 $\lambda_k$ 不是某个神秘的"拉格朗日乘子"——它的物理意义是清晰的：**在状态 $x_k$ 处微小扰动 $\delta x_k$ 导致剩余最优代价的线性变化 $\delta V_k \approx \lambda_k^\top \delta x_k$**。这是"边际成本"的精确数学表述。

### 工程含义

这一 PMP-DP 对偶关系是以下所有"沿轨迹反向传播梯度"算法的理论出处：

| 算法 | 如何使用此对偶 |
|------|--------------|
| iLQR/DDP (Jacobson-Mayne 1970, Tassa 2012) | 二阶版：$P_k = \nabla^2 V_k$（Hessian），$s_k = \nabla V_k - P_k x_k^{\text{ref}}$ |
| Differentiable MPC (Amos-Kolter 2017) | 通过隐函数定理对 KKT 条件求导，等价于值函数梯度传播 |
| Neural ODE adjoint (Chen-Rubanova 2018, NeurIPS) | 连续时间版的 $\lambda(t) = \nabla_x V(x^*(t), t)$ |

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 5 | $V_k$ 不可微处直接求梯度 | 使用 Clarke 次梯度或 viscosity solution |
| 6 | 把离散协态方程写成连续形式 | 离散：向后乘 $\nabla f^\top$；连续：$\dot\lambda = -\partial H/\partial x$ |
| 7 | 认为 $\lambda_k = \nabla V_k$ 在所有点成立 | 只在**最优轨迹上**成立 |

### 练习

1. 对 LQR 问题 $V_k(x) = \frac{1}{2}x^\top P_k x$，验证 $\lambda_k = P_k x_k^*$ 满足离散伴随方程
2. 在 iLQR 中，$\delta V_k = s_k^\top \delta x + \frac{1}{2}\delta x^\top P_k \delta x$。识别哪个是"一阶协态"，哪个是"二阶信息"

---

## §3.3.4 无限时域折扣 Bellman 方程与 Banach 压缩映射 ⭐⭐

### 动机

有限时域 DP 的反向递推是直接的——从 $V_N = \Phi$ 开始逐步向前算。但无限时域问题没有"终点"，我们不能从无穷远开始反向扫描。问题变成：**Bellman 方程 $V = \mathcal{T}V$ 是否有解？解是否唯一？能否通过迭代找到？**

这三个问题的统一回答来自泛函分析中的 Banach 压缩映射定理——而关键在于证明 Bellman 算子是 $\gamma$-压缩的。

> **类比**：想象你在雾天站在山上，想找到最低点。你不知道山的全貌（没有终端条件），但你知道每走一步，你与最低点的距离至少缩短为原来的 $\gamma$ 倍。那么无论你从哪里开始，反复走下去就能收敛到最低点。$\gamma$-压缩性正是这种"保证每步缩近"的数学表述。

### Bellman 算子的形式定义

设状态空间 $\mathcal{X}$ 有限或紧致，动作空间 $\mathcal{U}(x)$ 紧致，瞬时代价 $L:\mathcal{X}\times\mathcal{U}\times\mathcal{W}\to\mathbb{R}$ 有界（$\|L\|_\infty \leq L_{\max}$），$\gamma\in[0,1)$。

值函数空间：$B(\mathcal{X}) = \{V:\mathcal{X}\to\mathbb{R} \mid \|V\|_\infty < \infty\}$，在 $\sup$-范数 $\|V\|_\infty = \sup_{x\in\mathcal{X}} |V(x)|$ 下是 **Banach 空间**（完备赋范向量空间）。

**Bellman 最优性算子** $\mathcal{T}: B(\mathcal{X}) \to B(\mathcal{X})$：

$$(\mathcal{T}V)(x) := \min_{u\in\mathcal{U}(x)}\left\{L(x,u) + \gamma\,\mathbb{E}_w[V(f(x,u,w))]\right\}$$

**策略评估算子** $\mathcal{T}^\mu$（固定策略 $\mu$）：

$$(\mathcal{T}^\mu V)(x) := L(x,\mu(x)) + \gamma\,\mathbb{E}_w[V(f(x,\mu(x),w))]$$

### 核心定理：$\gamma$-压缩性

**定理 3.3.4（$\gamma$-压缩）**：对任意 $V_1, V_2 \in B(\mathcal{X})$，

$$\boxed{\|\mathcal{T}V_1 - \mathcal{T}V_2\|_\infty \leq \gamma\|V_1 - V_2\|_\infty}$$

同样，$\|\mathcal{T}^\mu V_1 - \mathcal{T}^\mu V_2\|_\infty \leq \gamma\|V_1 - V_2\|_\infty$。

**出处**：Blackwell 1965, *Ann. Math. Stat.* 36(1):226–235, DOI:10.1214/aoms/1177700285；Denardo 1967, *SIAM Review* 9(2):165–177, DOI:10.1137/1009030。

### 完整证明

**关键引理（$\min$ 的 1-Lipschitz 性）**：对任意函数 $f, g: A \to \mathbb{R}$，

$$|\inf_{a\in A} f(a) - \inf_{a\in A} g(a)| \leq \sup_{a\in A} |f(a) - g(a)|$$

*引理证明*：设 $a_g^* = \arg\min_{a} g(a)$（或取 $\epsilon$-近似）。则：

$$\inf_a f - \inf_a g \leq f(a_g^*) - g(a_g^*) \leq \sup_a |f - g|$$

对称地，交换 $f, g$ 得到反向不等式。取绝对值得证。$\square$

**主定理证明**：

固定任意 $x \in \mathcal{X}$。定义：

$$F_i(u) := L(x,u) + \gamma\,\mathbb{E}_w[V_i(f(x,u,w))], \quad i = 1, 2$$

则 $(\mathcal{T}V_i)(x) = \min_u F_i(u)$。

**Step 1**：由引理，

$$|(\mathcal{T}V_1)(x) - (\mathcal{T}V_2)(x)| = |\min_u F_1(u) - \min_u F_2(u)| \leq \sup_u |F_1(u) - F_2(u)|$$

**Step 2**：计算 $|F_1(u) - F_2(u)|$：

$$|F_1(u) - F_2(u)| = |\gamma\,\mathbb{E}_w[V_1(f(x,u,w)) - V_2(f(x,u,w))]|$$
$$\leq \gamma\,\mathbb{E}_w[|V_1(f(x,u,w)) - V_2(f(x,u,w))|]$$
$$\leq \gamma \|V_1 - V_2\|_\infty$$

最后一步用了 $|V_1(y) - V_2(y)| \leq \|V_1 - V_2\|_\infty$ 对所有 $y$。

**Step 3**：综合 Step 1 和 Step 2：

$$|(\mathcal{T}V_1)(x) - (\mathcal{T}V_2)(x)| \leq \gamma\|V_1 - V_2\|_\infty$$

**Step 4**：对 $x$ 取 $\sup$：

$$\|\mathcal{T}V_1 - \mathcal{T}V_2\|_\infty = \sup_x |(\mathcal{T}V_1)(x) - (\mathcal{T}V_2)(x)| \leq \gamma\|V_1 - V_2\|_\infty$$

证毕。$\square$

> **反事实推理**：如果 $\gamma = 1$（无折扣），证明中 Step 2 给出 $|F_1(u) - F_2(u)| \leq \|V_1 - V_2\|_\infty$，即算子变为**非扩张**而非压缩。非扩张映射不保证收敛到唯一不动点——可能有多个不动点，迭代可能震荡。这就是为什么无折扣问题需要额外结构（如 SSP 框架中的"proper policy"假设）。

### 推论：Banach 不动点 → 最优值函数存在唯一

回顾 Banach 压缩映射定理（泛函分析 B3）：

**Banach 定理**：设 $(X, d)$ 为完备度量空间，$T: X \to X$ 满足 $d(Tx, Ty) \leq \gamma \cdot d(x,y)$，$\gamma < 1$。则：
1. $T$ 有唯一不动点 $x^*$
2. 对任意 $x_0 \in X$，$T^k x_0 \to x^*$
3. 收敛速度：$d(T^k x_0, x^*) \leq \gamma^k d(x_0, x^*)$

应用到 Bellman 算子：

**推论 3.3.4.1**：Bellman 最优方程 $\mathcal{T}V^* = V^*$ 在 $B(\mathcal{X})$ 中有**唯一解** $V^*$，即最优值函数。对任意初始猜测 $V_0$，值迭代 $V_{k+1} = \mathcal{T}V_k$ 几何收敛到 $V^*$。

### 值迭代 (VI) 收敛率的精确刻画

**定理 3.3.4.2（几何收敛）**：

$$\|V_k - V^*\|_\infty \leq \gamma^k \|V_0 - V^*\|_\infty$$

达到 $\epsilon$-精度所需迭代数：

$$k \geq \frac{\log(\|V_0 - V^*\|_\infty / \epsilon)}{\log(1/\gamma)} = \mathcal{O}\left(\frac{1}{1-\gamma}\log\frac{1}{\epsilon}\right)$$

**数值例子**：$\gamma = 0.99$，$\|V_0 - V^*\| = 100$，$\epsilon = 0.01$：

$$k \geq \frac{\log(10000)}{\log(1/0.99)} \approx \frac{9.21}{0.01005} \approx 917 \text{ 步}$$

**实用停止准则（Puterman Thm. 6.3.1(c)）**：若 $\|V_{k+1} - V_k\|_\infty < (1-\gamma)\epsilon/(2\gamma)$，则贪心策略 $\mu_{k+1}$ 满足 $\|V^{\mu_{k+1}} - V^*\|_\infty < \epsilon$。

### 工程含义与直觉

| 现象 | 理论根源 | 工程应对 |
|------|---------|---------|
| $\gamma$ 逼近 1 时 VI 极慢 | 迭代数 $\propto 1/(1-\gamma)$ | 使用 PI 或 MPI 替代 VI |
| DQN target network | 实现"冻结右端的 Banach 迭代" | 每 $C$ 步更新 target |
| RL 长视野任务训练困难 | $\gamma = 0.999$ 需 ~1000 步收敛 | 分层 RL、goal-conditioned |

> **本质洞察**：$\gamma$-压缩性意味着 Bellman 算子每次迭代都在**缩小误差**。$\gamma$ 越接近 1，缩小得越慢——这不是算法的缺陷，而是**问题本身的内在困难**：看得越远（$\gamma$ 越大），不确定性越大，需要越多信息才能确定最优策略。

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 7 | $\gamma=1$ 直接套压缩性 | 只保证非扩张；须用 SSP + proper policy 或加权范数 |
| 8 | 无界回报下 $B(\mathcal{X})$ 不含 $V^*$ | 切换到加权 $\sup$-范数 $\|V\|_w = \sup_x |V(x)|/w(x)$ |
| 9 | 近似 VI 误差等于单步误差 | 误差被 $1/(1-\gamma)$ 放大 |

### 练习

1. **手算**：对 $\mathcal{X} = \{1,2\}$，$\gamma = 0.9$，$(\mathcal{T}V)(1) = \min\{1+0.9V(2), 2+0.9V(1)\}$，$(\mathcal{T}V)(2) = \min\{3+0.9V(1)\}$，从 $V_0 = (0,0)$ 出发做 5 次 VI 迭代
2. 证明 $\|\mathcal{T}^\mu V_1 - \mathcal{T}^\mu V_2\| \leq \gamma\|V_1 - V_2\|$（策略评估算子也压缩）
3. 若 $\gamma = 0.95$，需要多少步 VI 才能使误差从初始 50 降到 0.1？

---

## §3.3.5 策略迭代 (PI)、Modified PI 与收敛性 ⭐⭐

### 动机：为什么不只用 VI？

VI 简单直观：反复应用 $V_{k+1} = \mathcal{T}V_k$ 直到收敛。但它的收敛速度仅是**线性的**（率为 $\gamma$）。对于 $\gamma = 0.99$ 的问题，可能需要近千步。能否更快？

Howard 1960 提出的策略迭代（PI）给出了肯定回答：PI 在有限 MDP 上**有限步终止**，实践中通常 5-20 步。代价是每步需要求解一个 $n \times n$ 线性方程组。

> **类比**：VI 像梯度下降——每步改进一点点，步数多但每步便宜。PI 像牛顿法——每步跳得远（二次收敛），步数少但每步贵（需要求 Hessian/解方程组）。MPI 则像截断牛顿法——介于二者之间。

### Howard 策略迭代算法

**Howard 1960**（*Dynamic Programming and Markov Processes*, MIT Press）：

**输入**：MDP $(\mathcal{S}, \mathcal{A}, P, R, \gamma)$，初始策略 $\mu_0$

**重复**（直至 $\mu_{k+1} = \mu_k$）：

1. **策略评估**：精确求解 $V^{\mu_k} = \mathcal{T}^{\mu_k} V^{\mu_k}$

   由于 $\mathcal{T}^{\mu_k}$ 是仿射映射，这等价于线性方程组：
   $$V^{\mu_k} = r^{\mu_k} + \gamma P^{\mu_k} V^{\mu_k}$$
   即 $(I - \gamma P^{\mu_k}) V^{\mu_k} = r^{\mu_k}$
   
   其中 $[P^{\mu_k}]_{ij} = P(j|i, \mu_k(i))$，$[r^{\mu_k}]_i = R(i, \mu_k(i))$。
   
   由于 $\gamma < 1$，矩阵 $(I - \gamma P^{\mu_k})$ 严格对角占优，因此可逆：
   $$V^{\mu_k} = (I - \gamma P^{\mu_k})^{-1} r^{\mu_k}$$

2. **策略改进**：
   $$\mu_{k+1}(x) \in \arg\min_{u\in\mathcal{A}(x)} \left\{R(x,u) + \gamma \sum_{x'} P(x'|x,u) V^{\mu_k}(x')\right\}$$

### 核心定理：PI 有限步终止

**定理 3.3.5.1（Puterman Thm. 6.4.2）**：有限 $|\mathcal{S}|, |\mathcal{A}|$，$\gamma\in[0,1)$，Howard PI 至多 $|\mathcal{A}|^{|\mathcal{S}|}$ 步终止于最优 $\mu^*$。

**完整证明**：

**步骤 1（策略改进引理）**：若 $\mu_{k+1} \neq \mu_k$，则 $V^{\mu_{k+1}}(x) \leq V^{\mu_k}(x)$ 对所有 $x$，且至少一个 $x$ 严格小于。

*证明*：由改进步定义，$\mathcal{T}^{\mu_{k+1}} V^{\mu_k} \leq \mathcal{T}^{\mu_k} V^{\mu_k} = V^{\mu_k}$。

记 $W_0 = V^{\mu_k}$，$W_{j+1} = \mathcal{T}^{\mu_{k+1}} W_j$。由 $\mathcal{T}^{\mu_{k+1}}$ 单调性：
$$W_1 = \mathcal{T}^{\mu_{k+1}} W_0 \leq W_0$$
$$W_2 = \mathcal{T}^{\mu_{k+1}} W_1 \leq \mathcal{T}^{\mu_{k+1}} W_0 = W_1$$
归纳得 $W_j$ 单调递减且有下界，极限为 $\mathcal{T}^{\mu_{k+1}}$ 的不动点 $V^{\mu_{k+1}}$。

因此 $V^{\mu_{k+1}} \leq V^{\mu_k}$。若 $\mu_{k+1} \neq \mu_k$，则存在 $x_0$ 使 $\mathcal{T}^{\mu_{k+1}} V^{\mu_k}(x_0) < V^{\mu_k}(x_0)$（严格改进）。$\square$

**步骤 2（有限终止）**：确定性平稳策略总数为 $|\mathcal{A}|^{|\mathcal{S}|} < \infty$。策略改进引理保证每步严格改进（$V$ 严格减少），因此不会重复访问同一策略。

**步骤 3（终止即最优）**：终止条件 $\mu_{k+1} = \mu_k$ 意味着 $\mathcal{T}^{\mu_k} V^{\mu_k} = V^{\mu_k}$ 且 $\mathcal{T}V^{\mu_k} = V^{\mu_k}$，即 $V^{\mu_k} = V^*$。$\square$

### Ye 2011 强多项式界

**定理 3.3.5.2（Ye 2011, *Math. Oper. Res.* 36(4):593, DOI:10.1287/moor.1110.0516）**：

$$\text{PI 迭代数} \leq \mathcal{O}\left(\frac{mn}{1-\gamma}\log\frac{n}{1-\gamma}\right)$$

其中 $n = |\mathcal{S}|$，$m = \sum_s |\mathcal{A}_s|$，**强多项式**（不依赖回报的数值大小）。

### PI 作为 Newton 法

**定理 3.3.5.3（Puterman-Brumelle 1979, *Math. Oper. Res.* 4(1):60）**：PI 等价于 Newton 迭代作用于方程 $F(V) := V - \mathcal{T}V = 0$。

直觉：策略评估 = 解线性化方程（Newton 方向），策略改进 = 更新线性化点。这解释了为什么 PI 有**局部二次收敛**率。

### Modified PI (MPI)

**Puterman-Shin 1978** (*Mgmt. Sci.* 24(11):1127)：策略评估只做 $m$ 次 $\mathcal{T}^{\mu_{k+1}}$ 迭代而非精确求解：

- $m = 1$：退化为 **VI**
- $m = \infty$：退化为 **PI**
- $m$ 有限：**MPI**，收敛率为 $\gamma^m$（比 VI 的 $\gamma$ 更快）

### 收敛速度对照表

| 算法 | 评估开销 | 改进开销 | 渐近收敛率 | 有限步终止 |
|------|---------|---------|-----------|-----------|
| 值迭代 VI | — | $\mathcal{O}(mn)$ | 线性，率 $\gamma$ | 否 |
| 策略迭代 PI | $\mathcal{O}(n^3)$ | $\mathcal{O}(mn)$ | 二次（局部） | 是 |
| Modified PI ($m$步) | $\mathcal{O}(mn \cdot m)$ | $\mathcal{O}(mn)$ | 线性，率 $\gamma^m$ | 否 |

### 与 Actor-Critic 的深层联系

> **本质洞察**：Actor-Critic 是近似 MPI。Critic（价值网络）做若干次 TD 更新 = 有限次策略评估。Actor（策略网络）做策略梯度更新 = 近似策略改进。**LSPI**（Lagoudakis-Parr 2003）= 线性函数近似版 PI。

| RL 算法 | 对应 DP 算法 | 近似方式 |
|---------|------------|---------|
| Q-learning (tabular) | VI | 单样本异步更新 |
| SARSA | $\mathcal{T}^\mu$ 的单样本版 | 策略评估 |
| A2C/PPO | MPI | Critic=有限步评估，Actor=梯度改进 |
| LSPI | PI | 线性函数近似评估 |

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 10 | 认为 PI 总比 VI 快 | 大规模时 PI 的 $\mathcal{O}(n^3)$ 评估不可行 |
| 11 | 无折扣 MDP 套 Ye 2011 | Ye 界依赖 $\gamma<1$；无折扣可能指数步 |
| 12 | smallest-index 旋转规则 | 可能导致指数步；必须用 Dantzig 规则 |

### 练习

1. 对 §3.3.4 练习 1 的 2 状态 MDP，用 PI 求解（初始策略 $\mu_0(1)=\mu_0(2)=a$），验证 1 轮即终止
2. 证明 MPI 在 $m=2$ 时的收敛率为 $\gamma^2$
3. 解释为什么 PPO 的 "multiple epochs of minibatch SGD" 相当于增大 MPI 的 $m$

---

## §3.3.6 随机动态规划与 MDP 形式化 ⭐⭐

### 动机

前面几节主要处理确定性或简单随机情形。本节将 DP 理论完整嵌入**马尔可夫决策过程 (MDP)** 框架——这是强化学习的标准数学语言。理解 MDP 就是理解 RL 的全部数学基础。

### MDP 的严格定义

**MDP** $\mathcal{M} = (\mathcal{S}, \mathcal{A}, P, R, \gamma)$：

- **状态空间** $\mathcal{S}$（可数或连续）
- **动作空间** $\mathcal{A}$
- **转移核** $P(s'|s,a)$：在状态 $s$ 执行动作 $a$ 后转移到 $s'$ 的概率
- **即时奖励** $R: \mathcal{S}\times\mathcal{A} \to \mathbb{R}$（有界）
- **折扣因子** $\gamma \in [0,1)$

**策略**：$\pi: \mathcal{S} \to \Delta(\mathcal{A})$（从状态到动作分布的映射）

**值函数**：$V^\pi(s) = \mathbb{E}_\pi\left[\sum_{t=0}^\infty \gamma^t R(s_t, a_t) \mid s_0 = s\right]$

**动作值函数**：$Q^\pi(s,a) = R(s,a) + \gamma\,\mathbb{E}_{s'\sim P}[V^\pi(s')]$

### 随机 Bellman 方程

**Bellman 期望方程**（给定策略 $\pi$）：

$$V^\pi(s) = \sum_a \pi(a|s)\left[R(s,a) + \gamma\sum_{s'} P(s'|s,a) V^\pi(s')\right]$$

**Bellman 最优方程**：

$$V^*(s) = \max_a \left\{R(s,a) + \gamma\sum_{s'} P(s'|s,a) V^*(s')\right\}$$

$$Q^*(s,a) = R(s,a) + \gamma\sum_{s'} P(s'|s,a) \max_{a'} Q^*(s',a')$$

### 与确定性 DP 的统一

| 确定性 DP 语言 | MDP 语言 | 对应关系 |
|--------------|---------|---------|
| 动力学 $x' = f(x,u)$ | 转移核 $P(x'|x,u) = \delta(x'-f(x,u))$ | 确定性 = Dirac 分布 |
| 代价最小化 $\min L$ | 奖励最大化 $\max R = -L$ | 符号翻转 |
| 值函数 $V_k(x)$ | $V^\pi(s)$ 或 $V^*(s)$ | |
| Bellman 方程 $V = \mathcal{T}V$ | $V^* = \max_a\{R+\gamma PV^*\}$ | |

所有 §3.3.2–§3.3.5 的定理在 MDP 形式下保持不变。

### 期望代价 vs 风险敏感变体

标准 MDP 优化**期望**代价。但工程中有时需要考虑风险：

| 代价准则 | 数学形式 | 适用场景 |
|---------|---------|---------|
| 期望（风险中性） | $\mathbb{E}[\sum \gamma^t R_t]$ | 大多数 RL |
| CVaR（条件风险） | $\text{CVaR}_\alpha[\sum R_t]$ | 安全关键系统 |
| 指数效用（风险敏感） | $-\frac{1}{\theta}\log\mathbb{E}[e^{-\theta \sum R_t}]$ | 金融、鲁棒控制 |
| Worst-case（极端保守） | $\min_w \sum R_t(w)$ | $H_\infty$ 控制 |

### Blackwell 最优性

**定义**：策略 $\pi^*$ 为 **Blackwell 最优**，当存在 $\gamma_0 < 1$ 使 $\pi^*$ 对所有 $\gamma \in [\gamma_0, 1)$ 都最优。

**定理 3.3.6.1（Blackwell 1962）**：有限 MDP 存在 Blackwell 最优平稳策略。

**工程意义**：长期运行的服务机器人（家政、自动驾驶）应使用 average-reward MDP 或 Blackwell 最优框架，避免固定 $\gamma$ 引入的"折扣近视"。

### 练习

1. 将 CartPole 环境形式化为 MDP：明确 $\mathcal{S}, \mathcal{A}, P, R, \gamma$
2. 证明 $Q^*(s,a) = R(s,a) + \gamma\sum_{s'}P(s'|s,a)\max_{a'}Q^*(s',a')$ 也有唯一解
3. 对 2 状态 MDP $\mathcal{S}=\{1,2\}$，$\mathcal{A}=\{a,b\}$，$\gamma=0.9$，$R(1,a)=1, R(1,b)=2, R(2,a)=0, R(2,b)=3$，$P(2|1,a)=P(1|2,a)=1$，$P(1|1,b)=P(2|2,b)=1$，手工求 $V^*$

---

## §3.3.7 维数灾难：精确含义与缓解方法 ⭐⭐⭐

### 动机

前面的理论很美——Bellman 方程有唯一解、VI 保证收敛、PI 有限步终止。但为什么工程中不直接用 tabular DP 解所有问题？答案是**维数灾难 (curse of dimensionality)**。

### Bellman 原话

Bellman 1957 在前言中首次提出这一术语；在 1961 年的 *Adaptive Control Processes* (Princeton UP, p.94) 中系统展开：

> *"It is the curse of dimensionality, a malediction that has plagued the scientist from earliest days."*

### 严格陈述

**命题 3.3.7**：网格化 $[0,1]^d$ 为每维 $N$ 格，则总网格点数 $|\mathcal{S}| = N^d$。一次完整 VI 扫描需 $\mathcal{O}(N^{2d}|\mathcal{A}|)$ 浮点运算，内存 $\mathcal{O}(N^d)$。

**数值例子**：

| 状态维度 $d$ | 每维 100 格 | 总格点数 | 内存 (64-bit) |
|-------------|------------|---------|--------------|
| 2 | $100^2$ | 10,000 | 80 KB |
| 6 | $100^6$ | $10^{12}$ | 8 TB |
| 12 | $100^{12}$ | $10^{24}$ | 不可能 |

一个 6-DoF 机械臂的位形+速度空间 $d=12$，直接网格化完全不可行。

### Powell 2011 的三重维数灾

Powell (*Approximate Dynamic Programming*, 2nd ed. Wiley 2011) 指出真实问题同时面临：

1. **状态维数灾** $|\mathcal{S}| \sim N^{d_x}$：存储 $V$ 的内存爆炸
2. **动作维数灾**：连续 $\mathcal{U} \subseteq \mathbb{R}^m$，枚举 $\arg\min_u$ 不可行（人形机器人 $m \sim 30$）
3. **结果维数灾**：计算 $\mathbb{E}_w[\cdot]$ 在高维噪声下样本复杂度爆炸

> **反事实推理**：如果没有维数灾难，我们可以直接用 tabular VI 精确求解所有最优控制问题——不需要 MPC、不需要 iLQR、不需要 RL。这三大方法家族的**存在理由**正是维数灾难：它们各自用不同方式绕过维数灾难。

### 五种主要缓解策略

| 策略 | 核心思想 | 代表方法 | 机器人场景 | 代价 |
|------|---------|---------|-----------|------|
| **局部二次 DP** | 只在名义轨迹附近做二次逼近 | iLQR/DDP | 抓取、行走、飞行 | 仅局部最优 |
| **退行视界** | 只看有限步，滚动求解 | MPC | 四足、无人机 | 需要终端代价保证稳定性 |
| **函数近似** | $V \approx V_\theta$（NN/线性） | DQN、fitted VI | Atari、机械臂 | 逼近误差 $\times 1/(1-\gamma)^2$ |
| **稀疏采样** | Monte Carlo 替代穷举 | MCTS/AlphaZero | 棋类、离散决策 | 对连续空间需离散化 |
| **路径积分** | 利用 HJB 线性化 | MPPI | 赛车、四旋翼 | 需特殊代价结构 |

### 近似 DP (ADP) 的误差放大问题

**定理（Munos-Szepesv\'ari 2008, *JMLR* 9:815）**：若每步近似误差 $\|\mathcal{T}V_k - V_{k+1}\|_\infty \leq \delta$，则

$$\limsup_{k\to\infty} \|V_k - V^*\|_\infty \leq \frac{\delta}{1-\gamma}$$

**关键洞察**：单步误差 $\delta$ 被 $1/(1-\gamma)$ **放大**。对 $\gamma=0.99$，放大 100 倍！

> **类比**：想象你在跟踪一个运动目标，每步估计误差为 $\delta$。如果你只看一步，误差就是 $\delta$。但如果你要预测 100 步之后的位置（对应 $1/(1-\gamma)=100$），误差会累积放大。这正是长视野 RL 训练困难的理论根源。

### "神经网络解决维数灾"是误解

深度网络提供表示上的灵活性，但统计样本复杂度仍可能随 $d$ 增长。Tsitsiklis-Van Roy 1997 给出了**线性函数近似下 Q-learning 发散**的经典反例。

现代成功的 RL 案例依赖：
- 结构先验（CNN 的平移不变性、GNN 的图结构）
- 海量并行采样（MuJoCo $10^8$ 步）
- 分布式 critic 稳定化（distributed TD）

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 13 | 直接把 tabular VI 搬到 12-DoF | 切换到局部 DP 或 MPC |
| 14 | 认为"NN 自动克服维数灾" | 检查样本复杂度、训练稳定性 |
| 15 | 忽略近似误差放大 | 记住 $1/(1-\gamma)$ 或 $1/(1-\gamma)^2$ 放大因子 |

### 练习

1. 计算：一个人形机器人（30 自由度，位形+速度 60 维状态空间）如果每维离散为 10 格，需要多少内存存储 $V$？
2. 解释为什么 MPC 不受维数灾困扰（提示：它只解有限时域的优化问题，不试图计算全局 $V$）
3. （跨章）在 3.9 iLQR 中，状态空间维度如何影响计算量？与 tabular DP 对比。

---

## §3.3.8 LQR 作为 DP 的精确可解特例 ⭐⭐

### 动机

在所有 DP 问题中，**LQR 是唯一可以精确解析求解的**。这不是巧合——线性动力学 + 二次代价 → 值函数保持二次型 → Bellman 方程退化为 Riccati 矩阵方程。理解这一推导过程至关重要：

1. 它提供了 DP 理论的**具体验证**——可以手算检验
2. 它是 iLQR/DDP 的**基石**——非线性问题在名义轨迹附近做二次展开就变成 LQR
3. 它揭示了 **PMP 与 DP 的精确对偶**——两条路径给出同一 Riccati 方程

### 问题定义

**有限时域离散 LQR**：

$$x_{k+1} = Ax_k + Bu_k \quad (+w_k)$$

$$J = \frac{1}{2}x_N^\top Q_f x_N + \sum_{k=0}^{N-1}\frac{1}{2}(x_k^\top Q x_k + u_k^\top R u_k)$$

假设：$Q \succeq 0$（半正定），$R \succ 0$（正定），$Q_f \succeq 0$。若存在噪声，$w_k \sim \mathcal{N}(0, \Sigma_w)$ 为高斯零均值。

### 从 Bellman 方程直接推出 Riccati

**定理 3.3.8.1（Bertsekas Vol.I, Prop. 4.1.1）**：LQR 最优值函数保持二次型

$$V_k(x) = \frac{1}{2}x^\top P_k x + c_k$$

其中 $P_k$ 满足**离散 Riccati 差分方程 (DRE)**：

$$\boxed{P_k = Q + A^\top P_{k+1}A - A^\top P_{k+1}B(R + B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A, \quad P_N = Q_f}$$

最优反馈律：

$$\boxed{u_k^* = -K_k x_k, \qquad K_k = (R + B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A}$$

### 完整推导（DP 归纳法）

**基础步**：$V_N(x) = \frac{1}{2}x^\top Q_f x$，故 $P_N = Q_f$，$c_N = 0$。

**归纳步**：假设 $V_{k+1}(x) = \frac{1}{2}x^\top P_{k+1}x + c_{k+1}$。代入 Bellman 方程：

$$V_k(x) = \min_u \left\{\frac{1}{2}x^\top Q x + \frac{1}{2}u^\top R u + V_{k+1}(Ax + Bu)\right\}$$

展开 $V_{k+1}(Ax + Bu)$（先忽略噪声项）：

$$= \min_u \left\{\frac{1}{2}x^\top Q x + \frac{1}{2}u^\top R u + \frac{1}{2}(Ax+Bu)^\top P_{k+1}(Ax+Bu) + c_{k+1}\right\}$$

**Step 1**：展开二次型：

$$(Ax+Bu)^\top P_{k+1}(Ax+Bu) = x^\top A^\top P_{k+1}Ax + 2x^\top A^\top P_{k+1}Bu + u^\top B^\top P_{k+1}Bu$$

**Step 2**：目标函数关于 $u$ 是严格凸二次（因为 $R + B^\top P_{k+1}B \succ 0$），一阶条件：

$$\frac{\partial}{\partial u}\left[\frac{1}{2}u^\top(R + B^\top P_{k+1}B)u + u^\top B^\top P_{k+1}Ax\right] = 0$$

$$(R + B^\top P_{k+1}B)u + B^\top P_{k+1}Ax = 0$$

$$u^* = -(R + B^\top P_{k+1}B)^{-1}B^\top P_{k+1}Ax = -K_k x$$

**Step 3**：代入 $u^* = -K_k x$ 配方。记 $M = R + B^\top P_{k+1}B$，则：

$$V_k(x) = \frac{1}{2}x^\top\left[Q + A^\top P_{k+1}A - A^\top P_{k+1}BM^{-1}B^\top P_{k+1}A\right]x + c_{k+1}$$

因此 $P_k = Q + A^\top P_{k+1}A - A^\top P_{k+1}BM^{-1}B^\top P_{k+1}A$。

**Step 4**（含噪声时）：$\mathbb{E}[V_{k+1}(Ax+Bu+w)] = V_{k+1}(Ax+Bu) + \frac{1}{2}\text{tr}(P_{k+1}\Sigma_w)$，故 $c_k = c_{k+1} + \frac{1}{2}\text{tr}(P_{k+1}\Sigma_w)$。

归纳完成。$\square$

### 无限时域 → DARE

**定理 3.3.8.2**：若 $(A,B)$ 可稳定、$(A,Q^{1/2})$ 可检测，则当 $N-k \to \infty$ 时 $P_k \to P_\infty$，$P_\infty$ 是**离散代数 Riccati 方程 (DARE)** 的唯一正半定稳定化解：

$$P = Q + A^\top PA - A^\top PB(R + B^\top PB)^{-1}B^\top PA$$

闭环极点 $\text{eig}(A - BK_\infty)$ 全部在单位圆内。

### 确定性等价原理 (Separation Principle)

**定理 3.3.8.3**：加性高斯噪声 $w_k \sim \mathcal{N}(0,\Sigma_w)$ 仅贡献常数 $c_k$，不改变 $K_k$。

**物理意义**：噪声不影响最优控制策略——"你该怎么控制就怎么控制，噪声只增加不可避免的代价"。这就是 **LQG = LQR + Kalman 分离**的数学根源。

> **反事实推理**：如果噪声是**乘性的**（$x_{k+1} = (A + \xi_k)x_k + Bu_k$，其中 $\xi_k$ 随机），确定性等价原理失效。此时最优增益 $K_k$ 确实依赖于噪声统计量——需要 Kleinman 1969 的乘性噪声 LQR 理论。

### 与 PMP→Riccati 的闭环验证

回顾 3.2.7：通过 PMP 沿轨迹推导出 $\lambda_k = P_k x_k + s_k$，以及线性两点边值问题。

本节通过 DP 直接从 $V_k = \frac{1}{2}x^\top P_k x$ 推导 Riccati。

**两条路径给出同一 Riccati 方程**——精确对偶：

$$P_k = \nabla^2_x V_k \quad \text{（值函数 Hessian）}$$
$$P_k x_k^* + s_k = \nabla_x V_k = \lambda_k \quad \text{（值函数梯度 = 协态）}$$

### 工程代码（Python）

```python
import numpy as np
from scipy.linalg import solve_discrete_are

# 双积分器系统：位置+速度
dt = 0.01
A = np.array([[1, dt], [0, 1]])
B = np.array([[0], [dt]])
Q = np.diag([10.0, 1.0])    # 位置权重大
R = np.array([[0.1]])         # 控制代价小

# 方法1：直接求解 DARE
P_dare = solve_discrete_are(A, B, Q, R)
K_dare = np.linalg.solve(R + B.T @ P_dare @ B, B.T @ P_dare @ A)

# 方法2：反向 DRE 迭代验证
P = Q.copy()  # 终端 P_N = Q（简化）
for _ in range(2000):
    M = R + B.T @ P @ B
    K = np.linalg.solve(M, B.T @ P @ A)
    P_new = Q + A.T @ P @ A - A.T @ P @ B @ K
    P = P_new

# 验证两种方法一致
assert np.allclose(P, P_dare, atol=1e-6), "DRE 未收敛到 DARE 解！"
print(f"最优增益 K = {K_dare}")
print(f"闭环极点 = {np.linalg.eigvals(A - B @ K_dare)}")
```

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 15 | $R$ 半正定（奇异 LQR） | $(R+B^\top PB)^{-1}$ 可能不存在；用 Moore-Penrose |
| 16 | DARE 多解混淆 | `dare`/`dlqr` 返回稳定化解；手工 DRE 做交叉验证 |
| 17 | 乘性噪声仍用确定性等价 | 失效！用 Kleinman 1969 |

### 练习

1. 对上述双积分器，改变 $Q$ 和 $R$ 的比值，观察闭环极点如何变化。给出直觉解释。
2. 证明：若 $R \to 0^+$，则 $K \to B^{-1}A$（dead-beat 控制）。什么条件下这是可行的？
3. （跨章综合）回忆 3.2.7 PMP 推导的 LQR Riccati。验证两种推导路径给出完全相同的矩阵方程。

---

## §3.3.9 从离散 DP 到连续 HJB：极限过渡 ⭐⭐⭐

### 动机

离散 Bellman 方程和连续时间 Hamilton-Jacobi-Bellman (HJB) 方程是同一事物的两面。理解二者之间的极限关系，既加深对 Bellman 方程本质的理解，又为后续专题 3.4 做铺垫。

### Taylor 展开极限过程

考虑连续时间系统 $\dot x = f(x,u)$ 的 $\Delta t$ 离散化。离散 Bellman 方程：

$$V(x,t) = \min_u \left\{L(x,u)\Delta t + e^{-\beta\Delta t}V(x + f(x,u)\Delta t, t+\Delta t)\right\}$$

其中 $\beta > 0$ 为连续时间折扣率（$\gamma = e^{-\beta\Delta t}$）。

**Step 1**：Taylor 展开 $V(x+f\Delta t, t+\Delta t)$：

$$V(x+f\Delta t, t+\Delta t) = V(x,t) + \nabla_x V \cdot f\Delta t + V_t \Delta t + \mathcal{O}(\Delta t^2)$$

**Step 2**：展开 $e^{-\beta\Delta t} = 1 - \beta\Delta t + \mathcal{O}(\Delta t^2)$

**Step 3**：代入并整理（保留到 $\mathcal{O}(\Delta t)$）：

$$V = \min_u \left\{L\Delta t + (1-\beta\Delta t)(V + \nabla_x V \cdot f\Delta t + V_t\Delta t)\right\} + \mathcal{O}(\Delta t^2)$$

$$0 = \min_u \left\{L\Delta t + \nabla_x V \cdot f\Delta t + V_t\Delta t - \beta V\Delta t\right\} + \mathcal{O}(\Delta t^2)$$

**Step 4**：除以 $\Delta t$ 并令 $\Delta t \to 0$：

$$\boxed{0 = V_t + \min_u \left\{L(x,u) + \nabla_x V \cdot f(x,u)\right\} - \beta V}$$

重写为标准 HJB 形式：

$$\beta V(x,t) = V_t + \min_u \{L(x,u) + \nabla_x V \cdot f(x,u)\}$$

无限时域稳态版（$V_t = 0$）：

$$\beta V(x) = \min_u \{L(x,u) + \nabla_x V(x) \cdot f(x,u)\}$$

### 两种离散化范式

| 范式 | 描述 | 代表 | 理论保证 |
|------|------|------|---------|
| **D-then-O** | 先离散动力学（RK4 等），再对离散 MDP 做 DP | MPC, iLQR | 依赖离散化精度 |
| **O-then-D** | 先写连续 HJB PDE，再用有限差分/FEM 求解 | HJ reachability | Barles-Souganidis 1991 收敛定理 |

> **本质洞察**：D-then-O 和 O-then-D 在 $\Delta t \to 0$ 时收敛到同一连续时间解。但有限 $\Delta t$ 下它们可能给出不同答案——D-then-O 的误差来自动力学离散化，O-then-D 的误差来自 PDE 数值格式。工程中 D-then-O 更常用（因为机器人本身就是离散控制的）。

### Viscosity Solution 简介

HJB 方程通常**没有经典 $C^1$ 解**——典型例子是最短时间问题 $|V'(x)| = 1$ 在 $[-1,1]$ 上：经典解不存在，但 $V(x) = |x|$ 是"正确答案"（在 $x=0$ 不可微）。

**Crandall-Lions 1983** 定义的 viscosity solution 用"光滑测试函数"替代不光滑的 $V$ 本身：
- $V$ 是黏性下解：$\forall \phi\in C^1$，若 $V-\phi$ 在 $x_0$ 取极大，则 $F(x_0, V(x_0), \nabla\phi(x_0)) \leq 0$
- 上解对偶；黏性解 = 同时是上解和下解

### LQR 作为连续时间 HJB 的验证

对连续时间 LQR：$\dot x = Ax + Bu$，$J = \int_0^\infty \frac{1}{2}(x^\top Qx + u^\top Ru)dt$。

猜测 $V(x) = \frac{1}{2}x^\top Px$（稳态二次型），代入稳态 HJB（$\beta=0$ 无折扣，但任务在到达原点后终止）：

$$0 = \min_u \left\{\frac{1}{2}(x^\top Qx + u^\top Ru) + x^\top P(Ax + Bu)\right\}$$

对 $u$ 求导：$Ru + B^\top Px = 0$，得 $u^* = -R^{-1}B^\top Px$。代入：

$$0 = \frac{1}{2}x^\top Qx + x^\top PAx - \frac{1}{2}x^\top PBR^{-1}B^\top Px$$

由 $x$ 任意性：$PA + A^\top P - PBR^{-1}B^\top P + Q = 0$——这就是**连续代数 Riccati 方程 (CARE)**。

离散 DARE 在 $\Delta t \to 0$ 时收敛到 CARE——可以用 $A_d = I + A\Delta t$, $B_d = B\Delta t$ 代入 DARE 并取极限验证。

### 二种离散化范式的工程对比

| 属性 | D-then-O | O-then-D |
|------|---------|---------|
| 主流工程方法 | MPC, iLQR, DDP | HJ Reachability |
| 动力学处理 | 先 RK4/Euler 离散化 | 保持连续 PDE |
| 最优性保证 | 依赖步长 $\Delta t$ | 收敛到黏性解（Barles-Souganidis） |
| 维度限制 | 无固有限制 | $\leq 5$-$6$ 维（网格爆炸） |
| 典型工具 | CasADi, Drake, Crocoddyl | helperOC, hj\_reachability |
| 实时性 | 可实时（MPC 10-100Hz） | 离线计算，在线查表 |

### 从 Bellman 到 HJB 的完整对应表

| 离散 Bellman | 连续 HJB | 对应关系 |
|-------------|---------|---------|
| $V_k(x)$ | $V(x,t)$ | 值函数 |
| $\min_u\{L + V_{k+1}(f)\}$ | $\min_u\{L + \nabla V \cdot f\}$ | Bellman 算子/Hamiltonian |
| $\gamma V_{k+1}$ | $e^{-\beta\Delta t}V \approx V - \beta V\Delta t$ | 折扣 |
| DRE (Riccati 差分方程) | Riccati ODE $\dot P + A^\top P + PA - PBR^{-1}B^\top P + Q = 0$ | LQR |
| $\lambda_k = \nabla V_k$ | $\lambda(t) = \nabla_x V(x^*(t),t)$（极小化约定） | 共态-梯度对偶（详见专题 3.4.4） |

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 18 | 把离散 $\gamma$ 和连续 $\beta$ 混淆 | $\gamma = e^{-\beta\Delta t}$，$1-\gamma \approx \beta\Delta t$ |
| 19 | 假设 HJB 有 $C^1$ 解 | 一般只有 Lipschitz 黏性解 |
| 20 | 用标准有限差分格式解 HJB | 必须用 upwind 格式保证收敛到黏性解 |

### 练习

1. 对双积分器 $\dot x_1 = x_2$, $\dot x_2 = u$, $|u| \leq 1$，写出最短时间问题的 HJB 方程。这个 HJB 有经典解吗？
2. 取 $\Delta t = 0.1, 0.01, 0.001$，用离散 DARE 逼近连续 CARE（对双积分器 LQR）。观察收敛速率。
3. 解释为什么 MPC 使用 D-then-O 而非 O-then-D（提示：维度、实时性、迭代优化）

---

## §3.3.10 与强化学习的逻辑链 ⭐⭐

### 动机

本节回答一个关键问题：**DP 和 RL 是什么关系？** 答案是：RL 是 DP 在"无模型 + 函数近似 + 有限样本"约束下的近似求解方法。

### 逻辑链：DP → TD 学习 → Q-learning

| 层次 | 方法 | 知道什么 | 如何计算 Bellman 更新 |
|------|------|---------|---------------------|
| 精确 DP | VI / PI | 完整模型 $P, R$ | $V(s) = \min_u\{R + \gamma\sum P V(s')\}$ 精确计算 |
| Model-based RL | Dyna | 学习的模型 $\hat P, \hat R$ | 用估计的模型做 VI |
| TD(0) | Model-free | 无模型，只有样本 | $V(s) \leftarrow V(s) + \alpha[r + \gamma V(s') - V(s)]$ |
| Q-learning | Model-free | 无模型，只有样本 | $Q(s,a) \leftarrow Q + \alpha[r + \gamma\max_{a'}Q(s',a') - Q(s,a)]$ |
| DQN | NN-based | 无模型 + 函数近似 | $\min_\theta (r + \gamma\max_{a'}Q_{\theta^-}(s',a') - Q_\theta(s,a))^2$ |

> **本质洞察**：TD(0) 更新 $V(s) \leftarrow V(s) + \alpha[r + \gamma V(s') - V(s)]$ 中的 $\delta = r + \gamma V(s') - V(s)$ 就是 **Bellman 残差的单样本估计**。$\mathbb{E}[\delta | s] = (\mathcal{T}V)(s) - V(s)$。因此 TD(0) 是"随机梯度下降最小化 Bellman 残差"——RL 本质上就是在近似求解 Bellman 方程。

### DQN 的三重 DP 对应

| DQN 技巧 | ADP 数学等价 | 理论依据 |
|----------|-------------|---------|
| Replay Buffer | Fitted Q-Iteration 的批量训练集 | Ernst et al. 2005 |
| Target Network $\theta^-$ | FVI 的"冻结右端" | Banach 迭代的数值实现 |
| $\epsilon$-greedy 探索 | 保证 concentrability 系数有限 | Munos-Szepesv\'ari 2008 |

### 近似 VI 误差界

**定理（Munos-Szepesv\'ari 2008）**：设每步近似误差 $\epsilon_k$，则

$$\|V^* - V^{\pi_K}\| \leq \frac{2\gamma}{(1-\gamma)^2} C(\rho,\mu)^{1/p} \max_k \epsilon_k + \mathcal{O}(\gamma^K)$$

$C(\rho,\mu)$ 为 concentrability 系数——offline RL 核心理论困难。

### DP → Model-based RL → Model-free RL 的渐进放松

理解 DP 与 RL 的关系，关键在于识别"从 DP 到 RL"过程中**放松了哪些假设**：

| 假设 | 精确 DP | Model-based RL | Model-free RL |
|------|---------|---------------|--------------|
| 完整转移模型 $P(s'|s,a)$ | 需要 | 学习 $\hat P$ | 不需要 |
| 完整奖励函数 $R(s,a)$ | 需要 | 学习 $\hat R$ | 不需要 |
| 全状态空间可枚举 | 需要 | 可能需要 | 不需要 |
| 值函数精确表示 | 表格（精确） | 参数化（近似） | 参数化（近似） |
| 充分采样 | 不需要（知道 $P$） | 需要（建模） | 需要（学习） |

每放松一个假设，就引入一个新的误差源：
- 放松模型 → 模型误差（model bias）
- 放松表格 → 逼近误差（approximation error）
- 放松充分采样 → 统计误差（sampling error）

> **类比**：精确 DP 好比"有地图导航"——你知道每条路的距离，直接算最短路。Model-based RL 好比"边走边画地图"——地图可能有误。Model-free RL 好比"凭经验导航"——你不画地图，直接记住"从这里往左转通常更快"。三者的最终目标相同（到达目的地），但信息获取方式不同。

### Bellman 残差最小化：RL 训练的统一视角

几乎所有 value-based RL 算法都可以理解为"最小化某种 Bellman 残差"：

$$\text{目标} = \min_\theta \mathbb{E}_{(s,a,r,s')\sim\mathcal{D}} \left[(\underbrace{r + \gamma V_\theta(s')}_{\text{Bellman target}} - \underbrace{V_\theta(s)}_{\text{当前估计}})^2\right]$$

| 算法 | Bellman 残差形式 | 特殊处理 |
|------|-----------------|---------|
| Tabular TD(0) | $\delta = r + \gamma V(s') - V(s)$ | 单样本更新 |
| DQN | $\delta = r + \gamma\max_{a'}Q_{\theta^-}(s',a') - Q_\theta(s,a)$ | Target network |
| SAC | $\delta = r + \gamma(V_\theta(s') - \tau\log\pi)- Q_\theta(s,a)$ | 熵正则化 |
| TD3 | $\delta = r + \gamma\min_{i=1,2}Q_{\theta_i^-}(s',a') - Q_\theta(s,a)$ | Clipped double Q |

### Tsitsiklis-Van Roy 1997 发散反例

**为什么 off-policy + 函数近似 + bootstrapping 可能发散？**

经典反例：2 状态 MDP，线性函数近似 $V_\theta(s) = \theta\phi(s)$，off-policy 分布导致 TD 更新方向不再是 Bellman 残差的梯度方向。投影算子 $\Pi$ 和 Bellman 算子 $\mathcal{T}$ 的复合 $\Pi\mathcal{T}$ 不再是压缩映射——即使 $\mathcal{T}$ 和 $\Pi$ 各自都是非扩张/压缩的。

**三个要素的"致命三角"**（Sutton-Barto 2018 §11.3）：
1. **函数近似**（非精确表示）
2. **Bootstrapping**（用估计值更新估计值）
3. **Off-policy**（行为策略 ≠ 目标策略）

三者同时存在时可能发散。去掉任意一个都安全：
- 去掉函数近似 = tabular（安全）
- 去掉 bootstrapping = MC 方法（安全但高方差）
- 去掉 off-policy = on-policy TD（安全：SARSA、A2C）

### 练习

1. 证明 TD(0) 的期望更新等于 Bellman 残差
2. DQN 的 target network 每 $C$ 步更新一次。从 Banach 压缩映射的角度解释为什么这比每步更新更稳定
3. 解释 offline RL 为什么比 online RL 更难（提示：concentrability 系数）
4. 构造一个 2 状态 MDP + 线性函数近似的例子，说明 off-policy TD(0) 可能发散

---

## §3.3.11 典型例题 ⭐⭐

### 例题 1：最短路径（Bellman-Ford = VI）

**问题**：有向图 $G=(V,E)$，边权 $w(i,j) \geq 0$，求从源 $s$ 到所有节点的最短路径。

**DP 形式化**：状态 = 节点，动作 = 选择下一条边，代价 = 边权，目标态 = $s$。

$$V^*(i) = \min_{j:(i,j)\in E} \{w(i,j) + V^*(j)\}, \quad V^*(s) = 0$$

**VI 实现**（= Bellman-Ford 算法）：$V_0(i) = \infty\ (\forall i \neq s)$，$V_0(s) = 0$。迭代：

$$V_{k+1}(i) = \min_{j:(i,j)\in E} \{w(i,j) + V_k(j)\}$$

$|V|-1$ 轮迭代保证收敛（无负环时）。

**Dijkstra = 有序 VI**：每次只更新"距离最小的未处理节点"，等价于按优先级顺序做 VI。

### 例题 2：库存管理

**问题**：仓库容量 $C$，每期需求 $d_k \sim \text{Poisson}(\lambda)$，订货成本 $c$，持有成本 $h$，缺货惩罚 $p$。决策：每期订多少货 $u_k$？

**状态**：库存水平 $x_k \in \{0,1,\ldots,C\}$

**动力学**：$x_{k+1} = \min(x_k + u_k - d_k, C)^+$

**代价**：$L(x,u,d) = cu + h\max(x+u-d, 0) + p\max(d-x-u, 0)$

**Bellman 方程**：

$$V^*(x) = \min_{0 \leq u \leq C-x} \left\{\mathbb{E}_d[L(x,u,d) + \gamma V^*(x+u-d)^+]\right\}$$

这是一个经典的 $(s,S)$ 策略问题：当库存低于 $s$ 时订货到 $S$。

### 例题 3：有限时域 LQR 完整求解

**问题**：双积分器 $x = [p, v]^\top$，$A = \begin{bmatrix}1 & \Delta t \\ 0 & 1\end{bmatrix}$，$B = \begin{bmatrix}0 \\ \Delta t\end{bmatrix}$。

$Q = \text{diag}(1, 0.1)$，$R = 0.01$，$Q_f = \text{diag}(10, 1)$，$N = 50$。

**求解步骤**：

1. 初始化 $P_{50} = Q_f$
2. 反向迭代 DRE：$P_k = Q + A^\top P_{k+1}A - A^\top P_{k+1}B(R + B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A$
3. 计算增益序列 $K_k = (R + B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A$
4. 正向仿真：$x_0 = [1, 0]^\top$，$u_k = -K_k x_k$，$x_{k+1} = Ax_k + Bu_k$

```python
import numpy as np
import matplotlib.pyplot as plt

dt = 0.1; N = 50
A = np.array([[1, dt], [0, 1]])
B = np.array([[0], [dt]])
Q = np.diag([1.0, 0.1])
R = np.array([[0.01]])
Qf = np.diag([10.0, 1.0])

# 反向 DRE
P = [None] * (N+1)
K = [None] * N
P[N] = Qf
for k in range(N-1, -1, -1):
    M = R + B.T @ P[k+1] @ B
    K[k] = np.linalg.solve(M, B.T @ P[k+1] @ A)
    P[k] = Q + A.T @ P[k+1] @ A - A.T @ P[k+1] @ B @ K[k]

# 正向仿真
x = np.zeros((N+1, 2))
x[0] = [1.0, 0.0]
u_hist = np.zeros(N)
for k in range(N):
    u_hist[k] = (-K[k] @ x[k])[0]
    x[k+1] = A @ x[k] + B.flatten() * u_hist[k]

print(f"终端状态: p={x[-1,0]:.4f}, v={x[-1,1]:.4f}")
print(f"总代价: {0.5*x[-1]@Qf@x[-1] + 0.5*sum(x[k]@Q@x[k]+R[0,0]*u_hist[k]**2 for k in range(N)):.4f}")
```

---

## §3.3.12 数值 DP 方法 ⭐⭐⭐

### 网格离散化方法

对连续状态空间 $\mathcal{X} \subseteq \mathbb{R}^d$，最直接的方法是均匀网格化。

**步骤**：
1. 将 $\mathcal{X}$ 离散为 $N^d$ 个网格点 $\{x_1, \ldots, x_{N^d}\}$
2. 在每个网格点上存储 $V(x_i)$
3. 对非网格点用插值：$V(x) \approx \sum_i w_i(x) V(x_i)$（线性插值、样条等）
4. VI 更新：$V(x_i) \leftarrow \min_u \{L(x_i, u) + \gamma\sum_j P(x_j|x_i,u)V(x_j)\}$

**优缺点**：

| 优点 | 缺点 |
|------|------|
| 概念简单 | 维数灾难：$N^d$ 个点 |
| 收敛有理论保证 | 高维不可行 |
| 无逼近误差（在网格点上） | 插值引入误差 |

### 函数近似方法

用参数化函数 $V_\theta(x)$ 近似值函数：

**线性函数近似**：$V_\theta(x) = \sum_{i=1}^k \theta_i \phi_i(x) = \phi(x)^\top \theta$

基函数 $\phi_i(x)$ 的选择决定逼近能力：
- 多项式基：$\{1, x, x^2, xy, \ldots\}$
- 径向基（RBF）：$\phi_i(x) = \exp(-\|x-c_i\|^2/\sigma^2)$
- 傅里叶基：$\phi_i(x) = \cos(i\pi x/L)$

**神经网络近似**：$V_\theta(x) = \text{NN}_\theta(x)$（DQN、SAC 等）

### Galerkin 方法

**思想**：要求 Bellman 残差在有限维子空间上为零（弱形式）。

设 $V \approx \Phi\theta$（$\Phi$ 为基函数矩阵），要求：

$$\langle V_\theta - \mathcal{T}V_\theta, \phi_i \rangle_\mu = 0, \quad i = 1, \ldots, k$$

即 Bellman 残差在测试函数 $\phi_i$ 方向上的投影为零。这等价于**LSTD (Least-Squares Temporal Difference)** 方法。

### ALP（近似线性规划）

**de Farias-Van Roy 2003**：将 Bellman 方程的 LP 形式中 $V$ 限制为 $\Phi\theta$：

$$\max_\theta \mu^\top \Phi\theta \quad \text{s.t.} \quad \Phi\theta \leq \mathcal{T}\Phi\theta$$

性能界：$\|V^* - \Phi\tilde\theta\|_{1,\mu} \leq \frac{2}{1-\gamma}\min_\theta \|V^* - \Phi\theta\|_{\infty}$

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 21 | 线性插值在不连续值函数上产生大误差 | 使用单调格式或自适应网格 |
| 22 | 神经网络近似 VI 震荡不收敛 | Target network + 经验回放 |
| 23 | Galerkin 方法选错基函数 | 基函数需能表示值函数的主要特征 |

---

## §3.3.13 图搜索与半环统一视角 ⭐⭐⭐

### Dijkstra/A*/Bellman-Ford 全是 DP

确定性无折扣最短路径问题：$V(s) = \min_a\{c(s,a) + V(f(s,a))\}$

| 算法 | DP 解读 | 特点 |
|------|--------|------|
| **Dijkstra 1959** | 正向优先队列 VI | 非负权下每状态 pop 一次即最终 |
| **Bellman-Ford** | 同步 Jacobi VI | 允许负权（无负环），$|V|-1$ 次迭代 |
| **A*** (Hart et al. 1968) | 启发式 DP | $f = g + h$，可采纳 $h$ 保证最优 |

### 半环 (Semiring) 统一

通用 DP 方程可以在半环 $(\mathbb{K}, \oplus, \otimes, \bar 0, \bar 1)$ 上统一表述：

| 半环 | $\oplus$ | $\otimes$ | 应用 |
|------|---------|---------|------|
| Tropical (min-plus) | $\min$ | $+$ | 最短路径 |
| Max-product | $\max$ | $\times$ | Viterbi (HMM) |
| Sum-product | $+$ | $\times$ | 概率边缘化 |
| Max-sum | $\max$ | $+$ | Log-Viterbi |
| Boolean | $\vee$ | $\wedge$ | 可达性分析 |

### SLAM 中图优化 vs DP 的区分

**关键区分**：pose-graph SLAM 的 $\min_x \sum\|z_{ij} - h(x_i,x_j)\|^2$ 是**MAP 推断**（估计过去），不是 DP（决策未来）。联系仅在因子图消去的代数结构——GTSAM 的变量消去是 sum-product 半环的 DP 式扫描。

---

## §3.3.14 博弈论 DP、Minimax 与鲁棒控制 ⭐⭐⭐⭐

### Minimax DP

对抗干扰 $w \in W$（非随机、非合作）：

$$V(x) = \min_u \max_{w\in W} \{L(x,u,w) + \gamma V(f(x,u,w))\}$$

**Shapley 1953** 证明了此算子仍为压缩映射，不动点唯一——这是压缩映射用于 DP 的**最早论证**。

### 与 $H_\infty$ 控制的联系

离散 $H_\infty$ 控制等价于 min-max 二次代价博弈。线性系统下给出耦合 Riccati 方程。

**Robust Tube MPC**（Mayne-Seron-Rakovic 2005）使用有界扰动集 + terminal set 逻辑，属 Bertsekas-Rhodes 框架。

---

## 本章小结

| 知识点 | 核心结论 | 工程意义 | 难度 |
|--------|---------|---------|------|
| 最优性原理 | 最优子策略最优 | DP 反向递推的合法性基础 | ⭐ |
| Bellman 方程 | $V = \mathcal{T}V$ | 所有最优控制/RL 的统一母方程 | ⭐ |
| $\gamma$-压缩性 | $\|\mathcal{T}V_1-\mathcal{T}V_2\| \leq \gamma\|V_1-V_2\|$ | VI 收敛的理论保证 | ⭐⭐ |
| VI 几何收敛 | $\|V_k-V^*\| \leq \gamma^k\|V_0-V^*\|$ | 估计 VI 所需迭代数 | ⭐⭐ |
| PI 有限步终止 | $\leq |\mathcal{A}|^{|\mathcal{S}|}$ 步 | 中小规模 MDP 首选 PI | ⭐⭐ |
| PI = Newton | 局部二次收敛 | 解释 PI 为何远快于 VI | ⭐⭐⭐ |
| LQR Riccati | $V_k = \frac{1}{2}x^\top P_k x$ | iLQR/DDP 的基石 | ⭐⭐ |
| 协态=梯度 | $\lambda_k = \nabla V_k$ | PMP-DP 统一 | ⭐⭐ |
| 维数灾难 | $N^d$ 指数爆炸 | MPC/iLQR/RL 的存在理由 | ⭐⭐ |
| 确定性等价 | 加性噪声不改变 $K$ | LQG 分离定理 | ⭐⭐ |
| 离散→连续极限 | Bellman → HJB | 连接离散与连续控制 | ⭐⭐⭐ |
| TD = Bellman 残差 | $\delta = r + \gamma V(s') - V(s)$ | RL 的 DP 本质 | ⭐⭐ |

---

## 累积项目：本章新增模块

**项目进度**（从零构建最优控制求解器）：

- ✅ Ch 3.1：变分法 + Euler-Lagrange 方程求解器
- ✅ Ch 3.2：离散 PMP + shooting 方法
- **本章新增**：
  1. `value_iteration.py`：tabular VI/PI 实现，测试 FrozenLake
  2. `lqr_riccati.py`：DRE 反向扫描 + DARE 交叉验证
  3. `dp_examples.py`：最短路径（Bellman-Ford）、库存管理 DP

---

## 延伸阅读

| 资源 | 定位 | 难度 |
|------|------|------|
| Bertsekas, *DP & OC Vol.I* (2017) | 本章主教材 | ⭐⭐ |
| Bertsekas, *DP & OC Vol.II* (2012) | ADP 深化 | ⭐⭐⭐ |
| Puterman 1994, *MDP* | MDP 严格理论 | ⭐⭐⭐ |
| Sutton-Barto 2018, *RL Introduction* | RL 视角 DP | ⭐⭐ |
| Tedrake, *Underactuated Robotics* Ch.7-8 | 机器人 DP 应用 | ⭐⭐ |
| Bellman 1957, *Dynamic Programming* | 历史原著 | ⭐⭐⭐ |
| Ye 2011, *Math. Oper. Res.* | PI 强多项式界 | ⭐⭐⭐⭐ |
| Munos-Szepesv\'ari 2008, *JMLR* | 近似 VI 误差界 | ⭐⭐⭐⭐ |
| Rawlings-Mayne-Diehl, *MPC* (2017) | MPC 视角 DP | ⭐⭐⭐ |
| Fazel et al. 2018, ICML | PG on LQR 全局收敛 | ⭐⭐⭐⭐ |

---

## 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| VI 不收敛 | $\gamma = 1$ 或无吸收态 | 1. 检查 $\gamma < 1$ 2. 验证问题有有限解 3. 切换 SSP 框架 | §3.3.4 |
| PI 策略不改进 | 已到最优或数值误差 | 1. 打印 Bellman 残差 2. 检查浮点精度 3. 验证策略评估精确 | §3.3.5 |
| LQR Riccati 不收敛 | $(A,B)$ 不可稳定 | 1. 检查可控性矩阵秩 2. 检查 $Q^{1/2}$ 可检测 3. 增大 $R$ | §3.3.8 |
| 近似 VI 误差过大 | concentrability 过高 | 1. 检查训练数据覆盖 2. 增大 replay buffer 3. 减小 $\gamma$ | §3.3.10 |
| DRE 反向扫描数值不稳定 | $P_k$ 增长过快 | 1. 检查 $(A,B)$ 可稳定性 2. 用 doubling 算法 3. 规范化 | §3.3.8 |

---

## 深化专题

以下三个专题是对前面核心内容的理论深化，分别覆盖近似方法的误差界、LP 视角下的 DP、以及随机控制的 Ito-Bellman 方程。

### A. 近似动态规划 (ADP) 与 DQN 的理论根源 ⭐⭐⭐

**Fitted Value Iteration (FVI)**：$V_{k+1} = \Pi_{\mathcal{F}} \mathcal{T} V_k$，在函数类 $\mathcal{F}$ 上投影。

**核心定理（有限时间误差界）**：

$$\|V^* - V^{\pi_K}\|_{p,\rho} \leq \frac{2\gamma}{(1-\gamma)^2} C(\rho,\mu)^{1/p} \max_k \epsilon_k + \mathcal{O}(\gamma^K)$$

近似误差被 $1/(1-\gamma)^2$ **二次放大**。

### B. 线性规划方法

由 $V \leq \mathcal{T}V \Rightarrow V \leq V^*$（单调性），LP 形式：

$$\max_V \sum_x \mu(x)V(x) \quad \text{s.t.} \quad V(x) \leq L(x,u) + \gamma\sum_{x'}P(x'|x,u)V(x'), \ \forall x,u$$

对偶 LP 给出占用测度 (occupancy measure)——CMDP 和逆强化学习的理论基础。

### C. 随机最优控制与 Ito-Bellman

系统 $dx_t = f(x_t,u_t)dt + \sigma(x_t,u_t)dW_t$，受控生成元：

$$\mathcal{L}^u V = f\cdot\nabla V + \frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)$$

随机 HJB：$V_t + \min_u\{L + \mathcal{L}^u V\} = 0$

**Path Integral Control (Kappen 2005, Todorov 2009)**：当 $\sigma\sigma^\top \propto R^{-1}$，随机 HJB 经对数变换变为线性 PDE，Feynman-Kac 给出路径积分解析解——**MPPI 算法**的理论支柱。

### D. 方法谱系速查表

| 方法 | 时间结构 | 状态空间 | 需要模型？ | 值函数表示 | 典型工具 |
|------|---------|---------|-----------|-----------|---------|
| 精确 DP (VI/PI) | 离散 | 有限/网格 | 是 | 表格 | pymdptoolbox |
| LQR (DARE) | 离散/连续 | 连续（线性） | 是 | 二次型 $x^\top Px$ | scipy, Drake |
| HJB (viscosity) | 连续 | 低维连续 | 是 | Lipschitz | ToolboxLS |
| iLQR/DDP | 离散 | 中等连续 | 是 | 局部二次 | Crocoddyl, Drake |
| MPC | 离散 | 中等连续 | 是 | 终端值函数 | acados, CasADi |
| TD(0)/SARSA | 离散 | 任意 | 否 | 线性/NN | SB3, CleanRL |
| DQN/SAC | 离散 | 任意 | 否 | NN | SB3, RLlib |
| MPPI | 连续 | 中等 | 是 | 路径积分 | Autorally |

---

## 核心教材与论文索引

### 教材

| 书 | 用于 | 免费资源 |
|---|------|---------|
| **Bertsekas, DP & OC Vol.I** (4th ed. 2017) | 主教材 | MIT OCW 6.231 |
| **Bertsekas, DP & OC Vol.II** (4th ed. 2012) | ADP | 作者主页 |
| **Puterman 1994, MDP** (Wiley) | 严格理论 | Wiley Online |
| **Sutton-Barto 2018, RL Introduction** | RL 视角 | incompleteideas.net |
| **Bellman 1957, Dynamic Programming** | 原著 | Princeton UP |

### 核心论文

| 论文 | DOI/链接 | 核心贡献 |
|------|---------|---------|
| Bellman 1957 | Princeton UP | 最优性原理、DP 奠基 |
| Blackwell 1965 | DOI:10.1214/aoms/1177700285 | 折扣 DP 压缩性 |
| Howard 1960 | MIT Press | 策略迭代 |
| Ye 2011 | DOI:10.1287/moor.1110.0516 | PI 强多项式界 |
| Mnih et al. 2015 | DOI:10.1038/nature14236 | DQN |
| Munos-Szepesv\'ari 2008 | JMLR 9:815 | 近似 VI 误差界 |
| Fazel et al. 2018 | ICML | PG on LQR 全局收敛 |

---

## §3.3.15 十大必会定理清单

| # | 定理 | 出处 | 一句话核心 |
|---|------|------|-----------|
| 1 | 最优性原理 | Bellman 1957 | 最优子策略最优 |
| 2 | DP 反向递推 | Bellman 1957 | $V_k = \min\{L + V_{k+1}\circ f\}$ |
| 3 | 协态 = 值函数梯度 | Bertsekas I §3.3 | $\lambda_k = \nabla_x V_k$ |
| 4 | Bellman 算子压缩 | Blackwell 1965 | $\|\mathcal{T}V_1-\mathcal{T}V_2\| \leq \gamma\|V_1-V_2\|$ |
| 5 | VI 几何收敛 | Puterman Thm 6.3.1 | $\gamma^k$ 速率 |
| 6 | PI 有限步终止 | Howard 1960 | $\leq |\mathcal{A}|^{|\mathcal{S}|}$ |
| 7 | PI = Newton | Puterman-Brumelle 1979 | 局部二次收敛 |
| 8 | Ye 强多项式界 | Ye 2011 | $\mathcal{O}(mn/(1-\gamma)\log n)$ |
| 9 | LQR Riccati | Bertsekas I Prop 4.1.1 | $V_k = \frac{1}{2}x^\top P_k x$ |
| 10 | 确定性等价 LQG | Astrom 1970 | 加性噪声不改变 $K$ |

---

## §3.3.16 自测与综合练习

### 基础题

**Q1**：证明有限 $|\mathcal{S}|, |\mathcal{A}|$、$\gamma\in[0,1)$ 下，Bellman 最优方程有唯一解。给出 Banach 不动点证明。

**Q2**：2 状态 MDP，$\gamma=0.9$，$R(1,b)=2, P(1|1,b)=1$，$R(2,b)=3, P(2|2,b)=1$，$R(1,a)=1, P(2|1,a)=1$，$R(2,a)=0, P(1|2,a)=1$。手工跑 3 轮 VI 与 1 轮 PI。

**Q3**：推导 $\gamma$-折扣 LQR 的 Riccati 方程。

### 进阶题

**Q4**：证明 fitted VI 误差界 $\limsup\|V_k - V^*\| \leq \delta/(1-\gamma)$。

**Q5**：Dijkstra 是 VI 还是 PI？论证你的选择。

### 跨章综合题

**Q6**（综合 3.1+3.2+3.3）：对同一个倒立摆问题，分别用 (a) Euler-Lagrange 求运动方程，(b) PMP 求开环最优轨迹，(c) DP/LQR 求最优反馈律。比较三种方法的适用范围和计算代价。

---

## §3.3.17 时间预算

| 阶段 | 内容 | 建议时长 | 产出 |
|------|------|---------|------|
| 第 1 周 | §3.3.0–§3.3.3 | 5–7 小时 | 最优性原理证明 + DP 反向递推 |
| 第 2 周 | §3.3.4–§3.3.6 | 5–7 小时 | 压缩性证明 + VI/PI Python 实现 |
| 第 3 周 | §3.3.7–§3.3.9 | 5–6 小时 | LQR Riccati + HJB 极限推导 |
| 档位 4 | §3.3.10–§3.3.14 | 8–10 小时 | ADP/RL 连接 + 选读 Bertsekas Vol.II |

**总计**：核心 15–20 小时；含选学 25–30 小时。

---

## §3.3.19 Bellman 算子的单调性与其他结构性质 ⭐⭐⭐

### 动机

$\gamma$-压缩性保证了收敛，但 Bellman 算子还有其他重要性质——单调性、平移不变性——它们是 PI 正确性证明和误差界推导的基石。本节系统梳理这些性质。

### 单调性

**命题**：若 $V_1(x) \leq V_2(x)$ 对所有 $x$，则 $(\mathcal{T}V_1)(x) \leq (\mathcal{T}V_2)(x)$ 对所有 $x$。

**证明**：对任意 $x, u$，

$$L(x,u) + \gamma\mathbb{E}[V_1(f(x,u,w))] \leq L(x,u) + \gamma\mathbb{E}[V_2(f(x,u,w))]$$

因为 $V_1 \leq V_2$ 逐点成立且 $\gamma \geq 0$。对两侧同时取 $\min_u$ 得 $\mathcal{T}V_1 \leq \mathcal{T}V_2$。$\square$

**工程意义**：单调性使我们可以用**下界**和**上界**来夹逼 $V^*$。如果构造了 $V_{\text{lo}} \leq V^*$ 和 $V_{\text{up}} \geq V^*$，则 $\mathcal{T}^k V_{\text{lo}} \leq V^* \leq \mathcal{T}^k V_{\text{up}}$ 对所有 $k$ 成立——双边逼近。

### 平移不变性

**命题**：$\mathcal{T}(V + c\mathbf{1}) = \mathcal{T}V + \gamma c\mathbf{1}$，其中 $c\mathbf{1}$ 表示常函数。

**证明**：

$$(\mathcal{T}(V+c))(x) = \min_u\{L + \gamma\mathbb{E}[(V+c)(f)]\} = \min_u\{L + \gamma\mathbb{E}[V(f)] + \gamma c\} = (\mathcal{T}V)(x) + \gamma c$$

$\square$

**工程意义**：这解释了为什么 VI 不需要知道 $V^*$ 的绝对值——从任意 $V_0$（甚至全零）出发都能收敛，因为常数偏移每步衰减为 $\gamma$ 倍。

### Blackwell 充分条件

**定理（Blackwell 1965）**：若算子 $\mathcal{T}$ 满足：
1. **单调性**：$V_1 \leq V_2 \Rightarrow \mathcal{T}V_1 \leq \mathcal{T}V_2$
2. **$\gamma$-折扣**：$\mathcal{T}(V + c\mathbf{1}) \leq \mathcal{T}V + \gamma c\mathbf{1}$（$\gamma < 1$, $c \geq 0$）

则 $\mathcal{T}$ 是 $\gamma$-压缩映射。

这给出了验证压缩性的简洁充分条件——比直接证 $\|\mathcal{T}V_1 - \mathcal{T}V_2\| \leq \gamma\|V_1-V_2\|$ 更容易。

### Bellman 算子的不动点特征化

**命题**：$V^*$ 是 $\mathcal{T}$ 的唯一不动点，且对任意 $V$：
- $V \leq \mathcal{T}V \Rightarrow V \leq V^*$（$V$ 是下界）
- $V \geq \mathcal{T}V \Rightarrow V \geq V^*$（$V$ 是上界）

**证明**（以下界为例）：$V \leq \mathcal{T}V$。由单调性，$\mathcal{T}V \leq \mathcal{T}^2 V$。归纳得 $V \leq \mathcal{T}^k V$。取 $k \to \infty$：$V \leq \lim \mathcal{T}^k V = V^*$。$\square$

> **本质洞察**：这一不动点特征化是 LP 方法的理论基础——LP $\max c^\top V$ s.t. $V \leq \mathcal{T}V$ 的最优解就是 $V^*$，因为约束 $V \leq \mathcal{T}V$ 等价于"$V$ 是 $V^*$ 的下界"。

### 策略次优差界

**命题（Bertsekas Vol.II Prop. 2.1.1）**：若 $\mu$ 对值函数 $\tilde V$ 贪心（即 $\mathcal{T}^\mu \tilde V = \mathcal{T}\tilde V$），则

$$\|V^\mu - V^*\|_\infty \leq \frac{2\gamma}{1-\gamma}\|\tilde V - V^*\|_\infty$$

**工程意义**：即使值函数近似有误差 $\epsilon = \|\tilde V - V^*\|$，贪心策略的次优差最多为 $\frac{2\gamma}{1-\gamma}\epsilon$。对 $\gamma=0.99$，放大因子约 198 倍——这就是为什么近似 DP/RL 对值函数精度如此敏感。

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 24 | 认为单调性意味着 VI 单调收敛 | 不是！$V_k$ 可能先升后降或震荡，但最终收敛 |
| 25 | 把策略次优差界的 $2\gamma/(1-\gamma)$ 忽略 | 这是 offline RL 必须关注的核心量 |

---

## §3.3.20 Bellman 方程的 LP 对偶与占用测度 ⭐⭐⭐

### 动机

Bellman 方程不仅可以用迭代法（VI/PI）求解，还可以转化为线性规划 (LP)。LP 视角揭示了一个深刻结构：最优策略等价于一个特殊的**状态-动作占用测度** (occupancy measure)。这一视角是约束 MDP (CMDP)、逆强化学习 (IRL)、以及 offline RL 的理论基础。

### 原始 LP（Manne 1960）

由 §3.3.19 的不动点特征化，$V \leq \mathcal{T}V \Rightarrow V \leq V^*$。因此：

$$\max_V \sum_x \mu(x)V(x) \quad \text{s.t.} \quad V(x) \leq L(x,u) + \gamma\sum_{x'}P(x'|x,u)V(x'), \quad \forall x, u$$

其中 $\mu > 0$ 为任意 state-relevance weights。最优解恰为 $V^*$。

**约束数量**：$|\mathcal{S}| \times |\mathcal{A}|$ 个线性约束，$|\mathcal{S}|$ 个变量。

### 对偶 LP：占用测度

LP 对偶：

$$\min_{\lambda \geq 0} \sum_{x,u} \lambda(x,u) L(x,u)$$
$$\text{s.t.} \quad \sum_u \lambda(x',u) = \mu(x') + \gamma\sum_{x,u}P(x'|x,u)\lambda(x,u), \quad \forall x'$$

其中 $\lambda(x,u)$ 是**折扣状态-动作占用测度**：

$$\lambda(x,u) = (1-\gamma)\sum_{t=0}^\infty \gamma^t P(s_t=x, a_t=u | \pi, \mu_0)$$

最优策略恢复为：$\pi^*(u|x) = \lambda^*(x,u) / \sum_{u'}\lambda^*(x,u')$

### 约束 MDP (CMDP) 与安全 RL

**Altman 1999** (*Constrained Markov Decision Processes*, Chapman & Hall)：

$$\min_\pi \mathbb{E}_\pi\left[\sum \gamma^t c_t\right] \quad \text{s.t.} \quad \mathbb{E}_\pi\left[\sum \gamma^t d^i_t\right] \leq \alpha_i, \quad i=1,\ldots,k$$

LP 形式天然编码约束：$\min\langle c,\lambda\rangle$ s.t. flow 约束 + $\langle d^i,\lambda\rangle \leq \alpha_i$。

**关键定理**（Altman 1999）：CMDP 最优策略通常是**随机的**（最多 $k$ 个状态需要随机化，$k$ = 约束数）。

**Lagrangian RL**（RCPO, Tessler et al. ICLR 2019）是此 LP 对偶的 primal-dual 实现——安全 RL 的标准方法。

> **反事实推理**：如果没有 LP/占用测度视角，我们只能用 Lagrangian 松弛 + 二分搜索来处理约束 MDP。LP 对偶直接给出了约束 MDP 的**精确解**结构——最优策略至多在 $k$ 个状态随机化。这一结构洞察无法从 VI/PI 中获得。

### 与逆强化学习 (IRL) 的联系

IRL 问题：给定专家演示轨迹，恢复奖励函数 $R$。

LP 视角：专家占用测度 $\lambda_E$ 已知，寻找 $R$ 使 $\lambda_E$ 为对偶 LP 的最优解。这等价于 Abbeel-Ng 2004 的特征匹配条件。

---

## §3.3.21 Bellman 方程在控制论中的具体应用示例 ⭐⭐

### 示例 1：倒立摆的无限时域 LQR

**系统**：倒立摆在竖直位置线性化，$\dot\theta = \omega$，$\dot\omega = \frac{g}{l}\theta - \frac{1}{ml^2}u$。

离散化（$\Delta t = 0.01$s）：

$$A = \begin{bmatrix} 1 & \Delta t \\ g\Delta t/l & 1 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ -\Delta t/(ml^2) \end{bmatrix}$$

取 $g=9.81$, $l=0.5$, $m=0.1$：

```python
import numpy as np
from scipy.linalg import solve_discrete_are

dt = 0.01; g = 9.81; l = 0.5; m = 0.1
A = np.array([[1, dt], [g*dt/l, 1]])
B = np.array([[0], [-dt/(m*l**2)]])
Q = np.diag([100.0, 1.0])  # 角度权重远大于角速度
R = np.array([[0.01]])

# 检查可控性
from numpy.linalg import matrix_rank
Wc = np.hstack([B, A@B])
print(f"可控性矩阵秩 = {matrix_rank(Wc)} (应=2)")

# 求解 DARE
P = solve_discrete_are(A, B, Q, R)
K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
print(f"K = {K}")
print(f"闭环极点 = {np.linalg.eigvals(A - B@K)}")

# 仿真
x = np.array([0.1, 0.0])  # 初始偏转 0.1 rad
x_hist = [x.copy()]
for _ in range(500):
    u = -K @ x
    x = A @ x + B.flatten() * u[0, 0]
    x_hist.append(x.copy())
x_hist = np.array(x_hist)
print(f"500步后: theta={x_hist[-1,0]:.2e}, omega={x_hist[-1,1]:.2e}")
```

**物理解释**：$K = [K_1, K_2]$，$u = -K_1\theta - K_2\omega$。$K_1$ 是"位置反馈"（类似弹簧），$K_2$ 是"速度反馈"（类似阻尼）。$Q$ 中角度权重大 → $K_1$ 大 → 更强的位置恢复力。

### 示例 2：网格世界的 VI 与 PI 实现

```python
import numpy as np

# 4x4 网格世界，目标在 (3,3)
n_states = 16
n_actions = 4  # 上下左右
gamma = 0.9

# 转移矩阵（确定性，撞墙不动）
def next_state(s, a):
    r, c = s // 4, s % 4
    if a == 0: r = max(r-1, 0)    # 上
    elif a == 1: r = min(r+1, 3)  # 下
    elif a == 2: c = max(c-1, 0)  # 左
    elif a == 3: c = min(c+1, 3)  # 右
    return r * 4 + c

# 奖励：到达目标得 +1，其余 -0.04（鼓励快速到达）
goal = 15
R = np.full((n_states, n_actions), -0.04)
for a in range(n_actions):
    for s in range(n_states):
        if next_state(s, a) == goal:
            R[s, a] = 1.0

# 值迭代
V = np.zeros(n_states)
for iteration in range(100):
    V_new = np.zeros(n_states)
    for s in range(n_states):
        if s == goal:
            V_new[s] = 0  # 吸收态
            continue
        values = [R[s, a] + gamma * V[next_state(s, a)] for a in range(n_actions)]
        V_new[s] = max(values)
    if np.max(np.abs(V_new - V)) < 1e-6:
        print(f"VI 收敛于第 {iteration+1} 步")
        break
    V = V_new

# 打印值函数
print("值函数 (4x4):")
print(V.reshape(4, 4).round(2))

# 提取最优策略
policy = np.zeros(n_states, dtype=int)
arrows = ['↑', '↓', '←', '→']
for s in range(n_states):
    values = [R[s, a] + gamma * V[next_state(s, a)] for a in range(n_actions)]
    policy[s] = np.argmax(values)
print("最优策略:")
for r in range(4):
    print(' '.join(arrows[policy[r*4+c]] for c in range(4)))
```

### 示例 3：随机库存管理 DP

```python
import numpy as np
from scipy.stats import poisson

# 参数
C = 10        # 最大库存
lam = 3.0     # 泊松需求率
c_order = 2   # 订货成本/单位
c_hold = 1    # 持有成本/单位
c_short = 5   # 缺货惩罚/单位
gamma = 0.95

# 状态空间：库存水平 0, 1, ..., C
n_states = C + 1

# 动作空间：订货量 0, 1, ..., C-x
def get_actions(x):
    return list(range(C - x + 1))

# 期望单步代价
def expected_cost(x, u):
    cost = c_order * u
    y = x + u  # 补货后库存
    for d in range(20):  # 截断泊松
        p = poisson.pmf(d, lam)
        surplus = max(y - d, 0)
        shortage = max(d - y, 0)
        cost += p * (c_hold * surplus + c_short * shortage)
    return cost

# 转移概率
def transition_prob(x, u):
    """返回 {next_state: probability} 字典"""
    y = x + u
    trans = {}
    for d in range(20):
        p = poisson.pmf(d, lam)
        ns = max(min(y - d, C), 0)
        trans[ns] = trans.get(ns, 0) + p
    return trans

# 值迭代
V = np.zeros(n_states)
for it in range(500):
    V_new = np.zeros(n_states)
    for x in range(n_states):
        best = np.inf
        for u in get_actions(x):
            cost = expected_cost(x, u)
            future = sum(p * V[ns] for ns, p in transition_prob(x, u).items())
            total = cost + gamma * future
            best = min(best, total)
        V_new[x] = best
    if np.max(np.abs(V_new - V)) < 1e-6:
        print(f"收敛于第 {it+1} 步")
        break
    V = V_new

# 最优策略
opt_policy = np.zeros(n_states, dtype=int)
for x in range(n_states):
    best_u, best_val = 0, np.inf
    for u in get_actions(x):
        cost = expected_cost(x, u)
        future = sum(p * V[ns] for ns, p in transition_prob(x, u).items())
        total = cost + gamma * future
        if total < best_val:
            best_val = total
            best_u = u
    opt_policy[x] = best_u

print("最优策略 (库存水平 -> 订货量):")
for x in range(n_states):
    print(f"  库存={x:2d} -> 订货={opt_policy[x]}")
```

---

## §3.3.22 VI/PI 的工程实现考量 ⭐⭐

### 异步值迭代

标准（同步）VI 每轮更新所有状态。**异步 VI** 每次只更新一个或少数状态。

**Gauss-Seidel VI**：使用最新可用的 $V$ 值（而非上一轮的快照）：

$$V(x_i) \leftarrow \min_u \{L(x_i,u) + \gamma\sum_j P(x_j|x_i,u) V(x_j)\}$$

其中 $V(x_j)$ 对已更新的 $j < i$ 使用新值，对未更新的 $j > i$ 使用旧值。

**收敛性**：异步 VI 在满足"每个状态被无限次更新"条件下仍收敛到 $V^*$（Bertsekas Vol.II §2.2）。

**工程优势**：
- 可以优先更新"最不确定"的状态（优先扫描）
- 天然适合并行/分布式计算
- 在线学习中新状态到达时即可更新

### 优先扫描 (Prioritized Sweeping)

**Moore-Atkeson 1993**：维护一个优先队列，按 Bellman 残差 $|\mathcal{T}V(x) - V(x)|$ 排序，每次更新残差最大的状态。

直觉：如果某状态的 $V$ 值刚发生大变化，其前驱状态的 $V$ 也可能需要更新。只更新"需要更新"的状态，跳过已稳定的。

### 策略评估的数值方法

精确 PI 需要求解 $(I - \gamma P^\mu)V = r^\mu$。对大规模问题：

| 方法 | 复杂度 | 适用规模 |
|------|--------|---------|
| 直接求逆 $(I-\gamma P)^{-1}r$ | $\mathcal{O}(n^3)$ | $n < 10^3$ |
| 迭代法（Jacobi/Gauss-Seidel） | $\mathcal{O}(mn\cdot k)$ | $n < 10^5$ |
| 共轭梯度 | $\mathcal{O}(mn\cdot k')$ | 稀疏 $P$ |
| TD(0)/TD($\lambda$) | $\mathcal{O}(1)$ 每样本 | 任意规模 |

### 值函数初始化的影响

虽然 VI 从任意 $V_0$ 都收敛，但好的初始化可以大幅加速：

- **乐观初始化**：$V_0 = V_{\max} = L_{\max}/(1-\gamma)$。保证 $V_0 \geq V^*$，使 VI 单调递减——便于判断收敛
- **悲观初始化**：$V_0 = 0$。适合最大化问题中的"exploration bonus"
- **上一轮解初始化**（MPC warm-start）：用上一时间步的 $V^*$ 初始化本步 VI

### 数值精度与终止准则的权衡

| 准则 | 公式 | 保证 | 缺点 |
|------|------|------|------|
| 绝对残差 | $\|V_{k+1}-V_k\|_\infty < \epsilon_{\text{abs}}$ | 无直接策略保证 | 可能过早终止 |
| Puterman 准则 | $\|V_{k+1}-V_k\| < (1-\gamma)\epsilon/(2\gamma)$ | $\|V^{\mu_{k+1}}-V^*\| < \epsilon$ | 保守 |
| 策略不变 | $\mu_{k+1} = \mu_k$ | PI 终止 | 可能数值噪声导致误判 |

---

## §3.3.23 Bellman 方程与信息论的交叉：熵正则化 DP ⭐⭐⭐

### 动机

标准 Bellman 方程的最优策略是确定性的（$\arg\min$ 通常唯一）。但在探索、鲁棒性、多模态行为等场景中，我们希望策略具有随机性。**熵正则化** DP 通过在代价中添加策略熵项来实现这一点。

### Soft Bellman 方程

添加熵奖励 $\mathcal{H}(\pi(\cdot|s)) = -\sum_a \pi(a|s)\log\pi(a|s)$：

$$V^*_{\text{soft}}(s) = \max_{\pi(\cdot|s)} \sum_a \pi(a|s)\left[Q(s,a) - \tau\log\pi(a|s)\right]$$

其中 $\tau > 0$ 为温度参数。最优策略为 Boltzmann 分布：

$$\pi^*(a|s) = \frac{\exp(Q(s,a)/\tau)}{\sum_{a'}\exp(Q(s,a')/\tau)}$$

代入得 **soft value function**：

$$V^*_{\text{soft}}(s) = \tau\log\sum_a \exp(Q^*(s,a)/\tau)$$

这就是 **log-sum-exp**——"soft maximum"。

### Soft Bellman 算子

$$(\mathcal{T}_{\text{soft}}Q)(s,a) = R(s,a) + \gamma\sum_{s'}P(s'|s,a)\tau\log\sum_{a'}\exp(Q(s',a')/\tau)$$

**定理**：Soft Bellman 算子仍是 $\gamma$-压缩映射，因此有唯一不动点。

**证明关键**：log-sum-exp 是 1-Lipschitz 的（关于 $\|\cdot\|_\infty$），因此 soft max 不破坏压缩性。

### 与 SAC 算法的联系

**Soft Actor-Critic (SAC)**（Haarnoja et al. 2018, ICML）就是 soft Bellman 方程的 off-policy actor-critic 实现：
- Critic：拟合 soft Q 函数
- Actor：最小化 KL 散度到 Boltzmann 策略

### 与 Path Integral Control 的深层联系

> **本质洞察**：熵正则化 DP 与 Path Integral Control（Kappen 2005, Todorov 2009）具有同一数学结构。当 $\tau = \sigma^2/2$（噪声方差的一半），soft Bellman 方程等价于对数变换后的线性 HJB。这就是为什么 MPPI（Williams 2017）和 SAC 在数学上是"同一件事的两种离散化"。

| 方法 | 连续时间对应 | 离散时间对应 | 工程实现 |
|------|------------|------------|---------|
| MPPI | 线性化 HJB (path integral) | 前向采样 + 指数加权 | GPU 并行采样 |
| SAC | 熵正则化 HJB | Soft Bellman 方程 | Off-policy NN 训练 |
| SQL | 同上 | Soft Q-learning | Replay buffer |

### 温度参数 $\tau$ 的物理意义

| $\tau$ 值 | 策略行为 | 等价于 |
|-----------|---------|--------|
| $\tau \to 0$ | 确定性贪心 | 标准 Bellman |
| $\tau$ 适中 | 适度探索 | SAC 默认设置 |
| $\tau \to \infty$ | 均匀随机 | 纯探索 |

SAC 中的自动温度调节：$\tau$ 通过对偶梯度下降自适应选择，约束目标熵 $\mathcal{H}(\pi) \geq \mathcal{H}_{\text{target}}$。

### ⚠️ 常见陷阱

| # | 陷阱 | 正确做法 |
|---|------|---------|
| 26 | 认为 SAC 的 "soft" 意味着近似 | "Soft" 指熵正则化，是精确的——有唯一不动点 |
| 27 | $\tau$ 选太大导致策略过于随机 | 使用自动温度调节或根据环境 reward scale 设定 |
| 28 | 把 max-entropy RL 和 exploration bonus 混淆 | 前者修改 Bellman 方程本身；后者修改奖励 |

---

## §3.3.24 DP 的计算复杂性理论视角 ⭐⭐⭐⭐

### MDP 求解的计算复杂性

**定理（Papadimitriou-Tsitsiklis 1987, *Math. Oper. Res.* 12(3):441）**：

- 有限 MDP 的精确最优策略可在**多项式时间**内求解（通过 LP）
- 但 POMDP（部分可观 MDP）的精确求解是 **PSPACE-hard**

**意义**：完全可观 MDP 在计算复杂性意义上是"容易"的（P 类）。DP/VI/PI 的挑战不在于计算复杂性理论，而在于**状态空间的规模**——$|\mathcal{S}|$ 可能指数于问题维度。

### 近似 MDP 的困难

**定理（Farias-Van Roy 2006）**：对一般 MDP，找到 $\epsilon$-最优策略（$V^\pi \leq V^* + \epsilon$）在最坏情况下需要 $\Omega(|\mathcal{S}|)$ 时间——没有普遍适用的多项式时间近似方案。

**但是**：具有特殊结构的 MDP（如分解结构、低秩转移矩阵）可以高效近似求解。这解释了为什么 RL 实践中需要归纳偏置（CNN、GNN、物理约束）。

### 样本复杂度

**定理（Azar-Munos-Kappen 2013, *JMLR*）**：使用生成模型（可查询 $P(s'|s,a)$），找到 $\epsilon$-最优策略的 minimax 样本复杂度为

$$\Theta\left(\frac{|\mathcal{S}||\mathcal{A}|}{(1-\gamma)^3\epsilon^2}\right)$$

**工程解读**：$1/(1-\gamma)^3$ 的依赖意味着长视野问题（$\gamma$ 接近 1）样本需求**立方增长**。这是 RL 训练困难的信息论下界——不是算法不好，而是问题本身就需要这么多信息。

---

## §3.3.25 常见陷阱速查总表

| # | 陷阱 | 症状 | 修复 |
|---|------|------|------|
| 1 | 代价非可加误用标准 DP | Bellman 方程无解 | 切换到 multiplicative DP / Denardo 抽象 |
| 2 | 状态非马氏 | 最优性原理失效 | 扩充为信息状态 |
| 3 | $\gamma=1$ 直接套压缩 | VI 不收敛 | SSP 框架 + proper policy |
| 4 | 近似 VI 误差线性估计 | 策略远差于预期 | 记住 $1/(1-\gamma)^2$ 放大 |
| 5 | 忽略 concentrability | offline RL 性能崩塌 | 检查数据分布覆盖 |
| 6 | 把协态当独立变量 | 无初值猜测 | $\lambda_k = \nabla_x V_k$，从值函数获取 |
| 7 | LQR 中 $R$ 半正定 | 反演奇异 | Moore-Penrose 或显式建模 |
| 8 | DARE 多解随机选 | 闭环不稳定 | 明确要求 stabilizing solution |
| 9 | 乘性噪声用确定性等价 | 控制增益错误 | Kleinman 1969 乘性 LQR |
| 10 | Howard PI 无折扣下强多项式 | 可能指数步 | Ye 2011 依赖 $\gamma<1$ |
| 11 | 认为 PI 总比 VI 快 | 大规模下评估太贵 | 使用 MPI |
| 12 | 神经网络自动克服维数灾 | 训练不稳定 | 检查样本复杂度和逼近能力 |

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| DP 只能处理离散问题 | DP 原理（Bellman 最优性原理）同样适用于连续时间/连续状态的最优控制，HJB 方程就是连续版 Bellman 方程 |
| 值迭代（VI）和策略迭代（PI）一定收敛到同一结果 | 在精确计算下确实收敛到同一最优值函数，但近似 DP（函数逼近）中两者可能收敛到不同解甚至发散 |
| 折扣因子 $\gamma$ 越接近 1 越好 | $\gamma \to 1$ 虽然更重视长期回报，但会导致收敛速度变慢（压缩系数 $\gamma$ 越大，Banach 不动点收敛越慢），且数值精度要求更高 |
| "维数灾难"可以被神经网络自动克服 | 神经网络提供函数逼近能力，但样本复杂度仍随状态维度指数增长（Bellman 1961 的本质困难未被消除），训练不稳定是常态 |
| Bellman 方程只与强化学习有关 | Bellman 方程是 LQR、MPC、iLQR/DDP、路径积分控制等所有基于值函数方法的共同数学基础，RL 只是其近似求解的一种途径 |
| 策略迭代总比值迭代快 | PI 每步需要完整的策略评估（解线性方程组），大规模问题中评估代价可能远超 VI 的一步更新；修正策略迭代（MPI）是实践中的平衡方案 |
| 连续状态 DP 必须先离散化 | 除网格离散化外，还有函数逼近（拟合值迭代）、核方法、神经网络等连续空间直接处理的手段；离散化在高维下因维数灾难而不可行 |

---

## §3.3.18 总结：DP 是你未来三年学习的总索引

本专题完成后，你掌握了**机器人控制-决策-学习三位一体**的元数学语言。所有你将遇到的算法都是 Bellman 方程 $V = \mathcal{T}V$ 的某种求解、某种近似、某种对偶。

**三条主脊柱已就位**：

1. **压缩映射 → Banach 不动点 → VI 几何收敛**：一切迭代型算法的收敛保证（TD learning、DQN target update、MPC warm-start）
2. **最优性原理 → DP 反向递推 → $\lambda_k = \nabla V_k$**：一切沿轨迹梯度反向传播的数学正当性（iLQR、DDP、neural ODE adjoint）
3. **LQR 精确解 → Riccati → 确定性等价**：一切"局部化"策略的半径扩张基础（iLQR、TVLQR、Kalman-LQG）

**六条通向未来的桥梁**：

| 方向 | 对应章节 | DP 中的根源 |
|------|---------|-----------|
| HJB | 3.4 | $\Delta t \to 0$ 极限 |
| LQR+Lyapunov | 3.5 | §3.3.8 无限时域稳定性 |
| iLQR/DDP | 3.9 | 局部二次 DP + 沿轨迹 Riccati |
| MPC | 3.11–3.13 | 有限时域 DP 的退行视界 |
| 强化学习 | 6.1–6.8 | Bellman 方程的无模型近似求解 |
| 运动规划 | 图搜索 | 确定性无折扣 DP |

> **最终判断**：机器人学的下一个十年仍属于 DP。世界模型（Dreamer 系列）在学习环境后做规划 = DP；Diffusion Policy 用 path integral 做连续 HJB 的变分近似；大模型 high-level planner 在做 hierarchical DP。把 Bellman 方程刻进本能，你就有了理解一切机器人 AI 新方法的 $\mathcal{O}(1)$ 索引。

---

---

## 附录：Bellman 的历史贡献与 DP 的命名故事

Richard Bellman（1920-1984）是美国应用数学家，在 RAND Corporation 工作期间系统发展了动态规划理论。关于"Dynamic Programming"这个名字的由来，Bellman 在自传 *Eye of the Hurricane* (1984, World Scientific) 中有一段著名的叙述：

> *"I spent the Fall quarter (of 1950) at RAND. My first task was to find a name for multistage decision processes... I wanted to get across the idea that this was dynamic, this was multistage, this was time-varying... I thought dynamic programming was a good name. It was something not even a Congressman could object to."*

Bellman 选择"动态规划"这个名字，部分原因是为了避免当时美国国会中反数学研究的政治压力——"programming"听起来像实际计划安排，而非纯数学研究。

### 学术谱系与影响

Bellman 的工作直接影响了：

| 后续发展 | 创始人 | 与 Bellman 的关系 |
|---------|--------|-----------------|
| 策略迭代 | Howard 1960 | Bellman 的学生 |
| MDP 形式化 | Puterman 1994 | 系统化 Bellman 理论 |
| 随机 DP | Blackwell 1962, 1965 | 压缩映射严格化 |
| 最优控制 HJB | Fleming, Lions | 连续时间推广 |
| 强化学习 | Sutton, Barto | 无模型近似版 |
| 图搜索 | Dijkstra 1959 | 独立发现，后归入 DP 框架 |

Bellman 发表了超过 600 篇论文和 40 余部著作，涉及控制论、数学生物学、医学决策等领域。*Dynamic Programming* (1957) 至今仍是控制理论和运筹学的基础参考之一。

### 概念演进时间线

| 年份 | 事件 | 意义 |
|------|------|------|
| 1952 | Bellman 在 RAND 提出 DP 基本思想 | 奠基 |
| 1957 | *Dynamic Programming* 出版 | 最优性原理首次系统陈述 |
| 1959 | Dijkstra 发表最短路径算法 | 图搜索中的 DP 独立发现 |
| 1960 | Howard 提出策略迭代 | 计算效率突破 |
| 1962 | Blackwell 折扣 DP | 数学严格化 |
| 1965 | Blackwell 压缩映射 | 收敛性完整证明 |
| 1967 | Denardo 抽象 DP | 统一框架 |
| 1978 | Puterman-Shin Modified PI | VI/PI 之间的插值 |
| 1992 | Watkins Q-learning 博士论文 | 无模型 DP 的突破 |
| 2013 | Mnih DQN (Atari) | 深度学习 + DP 首次大规模成功 |
| 2015 | DQN *Nature* 论文 | DP 理论进入工业界视野 |
| 2018 | Haarnoja SAC | 熵正则化 DP 成为机器人学标准 |

---

*版本：v2.0，2026-05-16。对应路线图 3.3，依赖 3.1、3.2；衔接 3.4–3.13 与第六批。*
