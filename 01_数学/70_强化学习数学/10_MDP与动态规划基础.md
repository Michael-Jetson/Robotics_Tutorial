# MDP 与动态规划基础

> **定位**：整个"强化学习数学"方向的地基。本章的目标不是教你调 PPO，而是把 RL 严格化为 **Banach 空间上的一个不动点问题** ——这是后续一切收敛性、样本复杂度、策略梯度理论的起点。
> **风格**：以 Bertsekas (2019) 与 Meyn (2022) 的控制人视角重写 RL，沿袭本项目"推导-例子-工程直觉三合一"编排。
> **前置**：泛函分析基础（Banach 空间、压缩映射、算子范数）；最优控制基础（Bellman 原理、LQR 直觉）。

---

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回前置章节复习）

1. 什么是完备度量空间？为什么完备性对不动点定理至关重要？
2. 陈述 Banach 压缩映射定理（一句话版本），并说明收敛速率。
3. 什么是 sup-norm $\|f\|_\infty$？为什么有界函数空间在 sup-norm 下是 Banach 空间？
4. 离散时间最优控制中的 Bellman 原理是什么？（用一句话描述）
5. LQR 问题的最优值函数为什么是二次型 $V^*(x) = x^\top P x$？

---

## 本章目标

学完本章后，你应当能够：

1. 用严格的数学语言写出 MDP 的完整定义（七元组），并理解每个元素在机器人控制中的物理对应
2. 独立完成 Bellman 算子 $\gamma$-压缩性的完整证明，从 Banach 不动点定理推出最优值函数的存在唯一性
3. 实现值迭代与策略迭代算法，理解两者的收敛速率差异与数学本质
4. 画出"MDP $\leftrightarrow$ DP $\leftrightarrow$ LQR $\leftrightarrow$ HJB"的血缘图，理解强化学习与控制理论是同一数学对象的不同面目

---

## 知识树总览

在开始具体内容之前，先看清本章的完整知识结构：

```
MDP 与动态规划基础
├── 1. MDP 严格定义（七元组形式化）
│   ├── 状态空间、动作空间、转移核
│   ├── 奖励函数、折扣因子
│   ├── Markov 性质的数学含义
│   └── 与 POMDP 的关系
├── 2. 值函数与 Bellman 方程
│   ├── 状态值函数 V^π
│   ├── 动作值函数 Q^π
│   ├── Bellman 期望方程
│   └── Bellman 最优方程
├── 3. Bellman 算子与压缩映射（核心）
│   ├── 算子的形式定义
│   ├── γ-压缩性完整证明
│   ├── Banach 不动点定理的应用
│   └── 存在唯一性定理
├── 4. 策略评估
│   ├── 矩阵形式 (I-γP^π)^{-1} r^π
│   ├── 迭代法与收敛速度
│   └── 数值稳定性
├── 5. 策略迭代
│   ├── 策略改进定理
│   ├── 单调性与有限步终止
│   └── 与 Newton 法的类比
├── 6. 值迭代
│   ├── 几何收敛率
│   ├── 误差界与停机条件
│   └── 异步值迭代
├── 7. 折扣因子的数学与直觉
├── 8. MDP 与最优控制的统一
├── 9. 占用测度与 LP 对偶
└── 10. 典型例题
```

---

## 1. MDP 严格定义：从决策过程到数学对象 ⭐

### 1.1 动机：为什么需要 MDP

在进入形式化定义之前，我们先问一个根本问题：**机器人为什么需要 MDP 这套数学框架？**

考虑一个四足机器人在复杂地形上行走的场景。每一时刻，机器人需要根据当前的身体状态（关节角度、角速度、身体姿态）决定下一步的关节目标位置。这个决策面临两个核心困难：

1. **不确定性**：地面摩擦系数未知、传感器有噪声、电机响应有延迟——即使执行相同的动作，结果也不完全确定
2. **序列性**：当前的决策影响未来的状态。如果现在迈步过大导致重心偏移，下一步就需要更大的修正力矩

> **本质洞察**：MDP 的核心贡献是把"在不确定性下做序列决策"这个看似无法数学化的问题，转化为一个结构清晰的数学对象——一个概率转移图上的优化问题。

如果没有 MDP 框架会怎样？我们只能采用手工设计的有限状态机（FSM）：在状态 A 执行动作 1，在状态 B 执行动作 2……这种方法对简单场景有效，但状态空间稍微变大（比如 48 维的本体感知向量），手工设计就完全失效。MDP 给了我们一个**算法化**的方法：只要能定义出状态、动作、转移和奖励，就有一套通用的数学工具自动求解最优策略。

### 1.2 历史追溯 ⭐

MDP 的数学框架有明确的历史谱系：

- **1957, Richard Bellman**：在《Dynamic Programming》一书中首次系统化了最优性原理（Principle of Optimality），建立了递推求解的数学基础。Bellman 的工作直接来源于二战期间的军事运筹学——如何在资源约束下做最优的多阶段决策。
- **1960, Ronald Howard**：在《Dynamic Programming and Markov Processes》中提出策略迭代算法。Howard 的贡献是发现"先评估、再改进"的两步循环，比 Bellman 的值迭代更快收敛。
- **1994, Martin Puterman**：《Markov Decision Processes: Discrete Stochastic Dynamic Programming》成为 MDP 理论的权威参考，几乎所有后续教材的定义都源于此书。

这条线索告诉我们：MDP 不是 RL 社区凭空发明的，它是**运筹学与控制理论的产物**，后来被 RL 社区继承并发扬光大。

### 1.3 七元组形式化定义 ⭐

一个（折扣无限时域）**马尔可夫决策过程（Markov Decision Process, MDP）**是一个七元组：

$$
\mathcal{M} = (\mathcal{S},\ \mathcal{A},\ P,\ r,\ \gamma,\ \mu_0,\ H)
$$

> **关于定义格式的说明**：标准文献（Puterman 1994、Bertsekas 2012、Sutton-Barto 2018）通常使用四元组 $(\mathcal{S},\mathcal{A},P,r)$ 或五元组 $(\mathcal{S},\mathcal{A},P,r,\gamma)$。这里采用七元组是为教学目的的扩展格式，将决策问题实例的全部组成部分一并纳入。

下面逐一解释每个元素：

**状态空间 $\mathcal{S}$**

状态空间是系统所有可能状态的集合。在不同场景下，$\mathcal{S}$ 的性质差异巨大：

| 场景 | 状态空间 | 维度 | 性质 |
|------|---------|------|------|
| Grid World | $\{(i,j): 0\le i,j \le 3\}$ | 16 个离散点 | 有限可数 |
| 机械臂关节控制 | $\mathbb{R}^{14}$（7 关节 $\times$ 位置+速度） | 14 维连续 | 有界子集 |
| 四足机器人 locomotion | $\mathbb{R}^{48}$（IMU+关节+指令） | 48 维连续 | 有界子集 |
| 围棋 | $\{0,1,2\}^{19\times 19}$ | $3^{361}$ 量级 | 有限但天文数字 |

在本章中，除非特别说明，我们取 $\mathcal{S}$ **有限**（$|\mathcal{S}| < \infty$）以聚焦算法推导。这不影响定理的正确性——一般测度空间上的推广只需要在技术细节上做调整（用有界可测函数空间 $\mathcal{B}(\mathcal{S})$ 代替 $\mathbb{R}^{|\mathcal{S}|}$）。

**动作空间 $\mathcal{A}$**

动作空间是智能体在每个状态可以选择的动作集合。动作空间可以是：
- 有限离散：如上下左右四个方向
- 连续有界：如关节目标位置 $a \in [-\pi, \pi]^{12}$
- 状态依赖：$\mathcal{A}(s)$ 随状态变化（如象棋中合法走法取决于当前棋局）

**转移核 $P: \mathcal{S} \times \mathcal{A} \to \Delta(\mathcal{S})$**

$P(s'|s,a)$ 是在状态 $s$ 执行动作 $a$ 后进入状态 $s'$ 的概率。$\Delta(\mathcal{S})$ 表示 $\mathcal{S}$ 上的概率分布集合。

转移核必须满足：
$$
P(s'|s,a) \ge 0, \quad \sum_{s' \in \mathcal{S}} P(s'|s,a) = 1, \quad \forall (s,a)
$$

> **工程直觉**：在物理仿真器（MuJoCo、PhysX）中，转移核就是"给定当前状态和动作，仿真一步后的状态分布"。对于确定性系统 $s' = f(s,a)$，转移核退化为 Dirac 分布 $P(s'|s,a) = \delta_{f(s,a)}(s')$。

**奖励函数 $r: \mathcal{S} \times \mathcal{A} \to \mathbb{R}$**

奖励函数将状态-动作对映射到一个实数标量，表示执行该动作获得的即时回报。关键性质：**有界性** $\|r\|_\infty \le R_{\max}$。

有些文献写作 $r(s,a,s')$（依赖下一状态）或 $r(s)$（只依赖状态），三者可通过条件期望互相转化：
$$
r(s,a) = \sum_{s'} P(s'|s,a) \cdot r(s,a,s') = \mathbb{E}[r(s,a,S') | S=s, A=a]
$$

**折扣因子 $\gamma \in [0,1)$**

折扣因子严格小于 1，这是保证 Bellman 算子压缩性的数学关键。关于 $\gamma$ 的深层含义，我们在第 7 节专门讨论。

**初始状态分布 $\mu_0 \in \Delta(\mathcal{S})$**

智能体开始交互时的初始状态从 $\mu_0$ 中采样。在 legged locomotion 中，$\mu_0$ 通常是"站立姿态附近加一些随机扰动"。

**视界 $H \in \mathbb{N} \cup \{\infty\}$**

$H < \infty$ 给出有限时域 MDP（对应 MPC 的 receding horizon）；$H = \infty$ 是折扣无限时域 MDP（本章主场）。

### 1.4 Markov 性质：数学含义与证明 ⭐⭐

MDP 的"M"——Markov 性质——是整个框架的数学基石。直观地说：

> **Markov 性质**：给定当前状态 $S_t$，未来 $S_{t+1}, S_{t+2}, \ldots$ 与过去 $S_0, S_1, \ldots, S_{t-1}$ 条件独立。

形式化：
$$
\mathbb{P}(S_{t+1} = s' | S_t = s, A_t = a, S_{t-1}, A_{t-1}, \ldots, S_0, A_0) = \mathbb{P}(S_{t+1} = s' | S_t = s, A_t = a) = P(s'|s,a)
$$

**为什么 Markov 性质如此重要？** 如果不满足 Markov 性质，当前状态就无法充分决定未来——我们需要记住完整的历史才能做最优决策。这使得问题在计算上不可处理（历史空间随时间指数增长）。Markov 性质允许我们把"无限历史上的优化"压缩为"当前状态上的递推"——这正是 Bellman 方程能够成立的原因。

> **反事实推理**：如果 Markov 性质不成立会怎样？考虑一个记忆迷宫：当前看到的走廊长得完全一样，但你需要记住"之前走过左边"才能决定现在走右边。此时 $S_t$ 不包含足够信息，$P(S_{t+1}|S_t, A_t)$ 与 $P(S_{t+1}|S_0,\ldots,S_t, A_0,\ldots,A_t)$ 不同。这正是 POMDP（部分可观测 MDP）的领地——解决方案是把 belief state（对隐状态的后验分布）作为新的"状态"，在 belief 空间上重新建立 Markov 性。

**实践中如何保证 Markov 性？**

在机器人控制中，通常通过"状态堆叠"来近似满足 Markov 性：
- 如果只给关节角度 $q_t$，那么仅凭位置无法预测下一步（缺少速度信息），不满足 Markov
- 加上关节速度 $(q_t, \dot{q}_t)$ 后，对于二阶动力学系统，状态就足够了
- Rudin et al. (2022) 的 ANYmal locomotion 还把上一步动作 $a_{t-1}$ 纳入状态——因为真实电机有延迟，当前关节扭矩依赖于上一步的指令

### 1.5 策略的数学定义 ⭐

**策略（policy）**是智能体的决策规则。一般地，策略是一族条件分布：
$$
\pi = (\pi_t)_{t \ge 0}, \quad \pi_t: \text{Histories} \to \Delta(\mathcal{A})
$$

我们主要关心两类特殊策略：

| 策略类型 | 定义 | 特点 |
|---------|------|------|
| **马尔可夫策略** | $\pi_t(a|s)$：只依赖当前状态与时间 | 利用 Markov 性质 |
| **平稳策略** | $\pi(a|s)$：时间无关 | 无限时域 MDP 的最优策略可取此形式 |
| **确定性策略** | $\pi(s) \in \mathcal{A}$：一个点映射 | 有限 MDP 中最优策略可取此形式 |

> **关键定理预告**：对有限状态有限动作的折扣 MDP，存在最优策略是确定性平稳策略。这是一个非平凡的结果——它说明我们不需要在"随机策略"的无限维空间中搜索，只需要在 $|\mathcal{A}|^{|\mathcal{S}|}$ 个确定性策略中找最优的那个。

### 1.6 折扣累积回报 ⭐

给定策略 $\pi$ 与初始分布 $\mu_0$，MDP 产生一条随机轨迹：
$$
\tau = (S_0, A_0, R_0, S_1, A_1, R_1, \ldots)
$$

其中 $S_0 \sim \mu_0$，$A_t \sim \pi(\cdot|S_t)$，$S_{t+1} \sim P(\cdot|S_t,A_t)$，$R_t = r(S_t, A_t)$。

**折扣累积回报（discounted return）**定义为：
$$
G_0 := \sum_{t=0}^{\infty} \gamma^t R_t
$$

由 $|R_t| \le R_{\max}$ 和 $\gamma < 1$，有上界：
$$
|G_0| \le \sum_{t=0}^{\infty} \gamma^t R_{\max} = \frac{R_{\max}}{1 - \gamma}
$$

这个有界性保证了值函数是良定义的（不会发散到 $\pm\infty$）。

### 1.7 POMDP 简介与 Belief MDP ⭐⭐⭐

当智能体无法直接观测到完整状态 $s$ 时，MDP 退化为**部分可观测 MDP（POMDP）**。POMDP 在 MDP 基础上增加两个要素：

- 观测空间 $\mathcal{O}$
- 观测核 $O(o|s,a)$：在状态 $s$ 执行动作 $a$ 后观测到 $o$ 的概率

**Belief State（信念状态）**$b_t \in \Delta(\mathcal{S})$ 是智能体对真实状态的后验分布，由贝叶斯滤波更新：
$$
b_{t+1}(s') \propto O(o_{t+1}|s', a_t) \sum_{s} P(s'|s, a_t) b_t(s)
$$

**Astrom (1965) 的核心洞察**：在 belief 空间 $\Delta(\mathcal{S})$ 上，POMDP 化归为一个（连续状态）完全可观测 MDP——称为 **belief MDP**。这意味着 MDP 的所有理论工具（Bellman 方程、值迭代、策略迭代）在原则上仍然适用，只是状态空间变成了单纯形。

> **机器人连接**：视觉 legged locomotion（Miki et al. 2022, Science Robotics）本质上是 POMDP——机器人只能观测到自身本体感觉和有噪声的深度图像，无法直接获取完整的地形信息。他们的解决方案是用 RNN/Transformer 隐式维护一个 belief state。

### ⚠️ 常见陷阱

**概念误区：认为"Markov 性质意味着不需要记忆"**

> **新手想法**：既然 Markov 说"未来只取决于现在"，那智能体不需要任何记忆。
>
> **实际上**：Markov 性质说的是**状态**已经包含了所有必要信息。如果你的"状态"定义不够完整（比如只有位置没有速度），那么它不满足 Markov 性质——你需要扩充状态定义或引入记忆机制。
>
> **正确理解**：Markov 性质不是系统的固有属性，而是**状态表示**的属性。同一个物理系统，用 $(q)$ 表示不满足 Markov，用 $(q, \dot{q})$ 表示就满足了。
>
> **自检方法**：问自己"仅凭当前状态，能否预测下一状态的分布？"如果不能，就需要扩充状态。

**编程陷阱：状态归一化缺失**

> **错误做法**：直接把原始传感器数据（关节角度 $\in [-\pi, \pi]$、角速度 $\in [-10, 10]$ rad/s、高度 $\in [0, 1]$ m）作为状态输入神经网络。
>
> **后果**：不同维度量纲不同、数值范围差异大，网络学习困难，收敛极慢甚至不收敛。
>
> **根本原因**：神经网络对输入的尺度敏感，未归一化的输入导致梯度在不同方向上量级差异过大。
>
> **正确做法**：对每个状态维度做 running mean/std 归一化，或者在环境中固定归一化参数。Isaac Lab 的标准做法是 `obs = (obs_raw - obs_mean) / obs_std`。

---

## 2. Bellman 方程与值函数 ⭐

### 2.1 动机：为什么需要值函数

我们的终极目标是找到最优策略 $\pi^*$。但"最优策略"是一个抽象的概念——怎么比较两个策略哪个更好？

> **类比**：值函数之于策略，就像考试成绩之于学生。我们无法直接比较两个学生"谁更优秀"，但可以通过标准化考试的分数来量化和排序。值函数 $V^\pi(s)$ 就是策略 $\pi$ 在状态 $s$ 上的"成绩"。

没有值函数，我们只能通过大量采样来比较策略——每个策略跑几千条轨迹、算平均回报、再做统计检验。值函数把这个过程压缩成了一个满足递推关系的函数，可以高效计算。

### 2.2 状态值函数 $V^\pi$ 的定义 ⭐

**定义**：策略 $\pi$ 的状态值函数（state-value function）是从状态 $s$ 出发、遵循策略 $\pi$ 的期望累积回报：

$$
V^\pi(s) := \mathbb{E}^\pi\left[\sum_{t=0}^{\infty} \gamma^t R_t \,\Big|\, S_0 = s\right], \quad V^\pi \in \mathbb{R}^{|\mathcal{S}|}
$$

这个定义的关键点：
- **"遵循策略 $\pi$"**意味着所有后续动作都按 $\pi$ 选取
- **期望**是对轨迹随机性（来自策略的随机性和转移的随机性）取的
- **$V^\pi$ 是一个向量**：$|\mathcal{S}|$ 维，每个分量对应一个状态的值

### 2.3 动作值函数 $Q^\pi$ 的定义 ⭐

**定义**：策略 $\pi$ 的动作值函数（action-value function, Q-function）：

$$
Q^\pi(s,a) := \mathbb{E}^\pi\left[\sum_{t=0}^{\infty} \gamma^t R_t \,\Big|\, S_0 = s, A_0 = a\right]
$$

$Q^\pi(s,a)$ 与 $V^\pi(s)$ 的区别：第一步动作被**固定**为 $a$（不管 $\pi$ 在 $s$ 处怎么选），从第二步起才按 $\pi$ 行动。

**两者之间的关系**：
$$
V^\pi(s) = \sum_{a \in \mathcal{A}} \pi(a|s) \cdot Q^\pi(s,a)
$$

这个关系的直觉：$V^\pi(s)$ 是"按 $\pi$ 选动作后的平均 Q 值"——先按概率 $\pi(a|s)$ 选动作，再看这个动作的 Q 值，取加权平均。

### 2.4 Bellman 期望方程的推导 ⭐

现在我们推导 MDP 理论中最核心的递推关系。起点是回报的递归分解：

$$
G_t = R_t + \gamma R_{t+1} + \gamma^2 R_{t+2} + \cdots = R_t + \gamma G_{t+1}
$$

将定义代入并利用全期望公式：

$$
\begin{aligned}
V^\pi(s) &= \mathbb{E}^\pi[G_t | S_t = s] \\
&= \mathbb{E}^\pi[R_t + \gamma G_{t+1} | S_t = s] \\
&= \mathbb{E}^\pi[R_t | S_t = s] + \gamma \mathbb{E}^\pi[G_{t+1} | S_t = s]
\end{aligned}
$$

第一项：即时奖励的期望。对动作和下一状态展开：
$$
\mathbb{E}^\pi[R_t | S_t = s] = \sum_{a \in \mathcal{A}} \pi(a|s) \cdot r(s,a)
$$

第二项：未来回报的期望。利用 Markov 性质和全概率公式：
$$
\begin{aligned}
\mathbb{E}^\pi[G_{t+1} | S_t = s] &= \sum_{s' \in \mathcal{S}} \mathbb{E}^\pi[G_{t+1} | S_{t+1} = s'] \cdot \mathbb{P}(S_{t+1} = s' | S_t = s) \\
&= \sum_{s'} V^\pi(s') \cdot \sum_{a} \pi(a|s) P(s'|s,a)
\end{aligned}
$$

> **注意**：第一个等号用了全期望公式 $\mathbb{E}[X] = \sum_{y} \mathbb{E}[X|Y=y]P(Y=y)$；第二个等号用了 **Markov 性质**——给定 $S_{t+1} = s'$，$G_{t+1}$ 的期望等于 $V^\pi(s')$，与 $S_t$ 无关。这正是 Markov 性质的威力所在。

合并两项，得到 **Bellman 期望方程**：

$$
\boxed{V^\pi(s) = \sum_{a \in \mathcal{A}} \pi(a|s) \left[ r(s,a) + \gamma \sum_{s' \in \mathcal{S}} P(s'|s,a) V^\pi(s') \right]}
$$

类似地，$Q^\pi$ 满足：
$$
Q^\pi(s,a) = r(s,a) + \gamma \sum_{s'} P(s'|s,a) \sum_{a'} \pi(a'|s') Q^\pi(s',a')
$$

> **本质洞察**：Bellman 期望方程把"无限未来的回报期望"分解为"一步即时奖励 + 折扣后的下一步价值期望"。这是一个**自举（bootstrapping）**关系——$V^\pi$ 的值由它自己在下一个时步的值决定。正是这种递归结构，使得动态规划方法成为可能。

### 2.5 Bellman 最优方程 ⭐

**最优值函数**定义为所有策略中最好的那个：
$$
V^*(s) = \sup_\pi V^\pi(s), \quad Q^*(s,a) = \sup_\pi Q^\pi(s,a)
$$

最优值函数满足 **Bellman 最优方程**：

$$
\boxed{V^*(s) = \max_{a \in \mathcal{A}} \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V^*(s') \right]}
$$

$$
Q^*(s,a) = r(s,a) + \gamma \sum_{s'} P(s'|s,a) \max_{a'} Q^*(s',a')
$$

**期望方程 vs 最优方程的关键区别**：

| | Bellman 期望方程 | Bellman 最优方程 |
|---|---|---|
| 算子性质 | 线性算子（$\sum_a \pi(a|s)[\cdot]$） | 非线性算子（$\max_a[\cdot]$） |
| 求解 | 线性方程组 → 直接求逆 | 非线性方程 → 需要迭代 |
| 物理意义 | "按策略 $\pi$ 行动"的值 | "按最优策略行动"的值 |
| 使用场景 | 策略评估 | 策略优化 |

### 2.6 有限时域版本与经典动态规划 ⭐⭐

当 $H < \infty$ 时，Bellman 方程变为**后向递推**：
$$
V^*_t(s) = \max_a \left[ r_t(s,a) + \sum_{s'} P_t(s'|s,a) V^*_{t+1}(s') \right], \quad V^*_H = g_{\text{terminal}}
$$

这就是 **Bellman (1957) 的经典动态规划**——与 MPC 中的 cost-to-go 递推完全同一个方程！区别仅在于：
- MPC 用 cost（取 min），RL 用 reward（取 max）
- MPC 通常 $\gamma = 1$、有限时域，RL 通常 $\gamma < 1$、无限时域

### ⚠️ 常见陷阱

**概念误区：混淆 $V^\pi$ 与 $V^*$ 的 Bellman 方程**

> **错误**：把 $V^\pi$ 的方程（含 $\sum_a \pi(a|s)$）和 $V^*$ 的方程（含 $\max_a$）搞混。
>
> **后果**：用期望方程的求解方法（矩阵求逆）去解最优方程——失败，因为后者是非线性的。
>
> **根本原因**：$T^\pi$ 是线性算子（凸组合），$T^*$ 是非线性算子（max 操作）。两者的不动点也不同：$T^\pi$ 的不动点是 $V^\pi$，$T^*$ 的不动点是 $V^*$。
>
> **记忆方法**：$V^\pi$ 的方程是"给定策略评估"（evaluation），用 $\sum$；$V^*$ 的方程是"寻找最优策略"（optimization），用 $\max$。

### 练习

1. **手推 Bellman 方程**：设 $\mathcal{S} = \{s_1, s_2\}$，$\mathcal{A} = \{a_1, a_2\}$，自选 $P, r$。写出 $V^\pi$ 的 Bellman 期望方程的显式展开（$2 \times 2$ 线性方程组）。
2. **Q 与 V 的互推**：已知 $V^\pi$，写出 $Q^\pi(s,a)$ 的表达式（不含 $Q$ 本身）。反过来，已知 $Q^\pi$，写出 $V^\pi(s)$ 的表达式。
3. **有限时域退化**：设 $H=2$，$\gamma=1$，只有 2 个状态和 2 个动作。手写 Bellman 递推的每一步，验证它给出了"前看两步"的最优决策。

---

## 3. Bellman 算子与压缩映射定理 ⭐

### 3.1 动机：为什么要把 Bellman 方程变成算子

Bellman 方程写作 $V = T V$（$V$ 是某个算子 $T$ 的不动点）。为什么要这样做？

> **类比**：求解 $x = \cos(x)$ 可以用不动点迭代 $x_{k+1} = \cos(x_k)$。如果 $\cos$ 是压缩映射，那么无论初值如何，序列 $x_k$ 都几何收敛到唯一不动点。同理，如果 Bellman 算子是压缩映射，值迭代 $V_{k+1} = TV_k$ 就保证收敛。

**这就是把 RL 变成不动点问题的核心动机**：一旦证明了压缩性，存在性、唯一性、收敛性全部作为 Banach 定理的推论自动获得——不需要单独证明。

### 3.2 Banach 空间设置 ⭐⭐

把值函数看作 **Banach 空间** $(\mathbb{R}^{|\mathcal{S}|}, \|\cdot\|_\infty)$ 中的向量。

- **sup-norm（无穷范数）**：$\|V\|_\infty := \max_{s \in \mathcal{S}} |V(s)|$
- **完备性**：$\mathbb{R}^{|\mathcal{S}|}$ 在 sup-norm 下是完备度量空间（有限维空间在任何范数下都完备）
- **一般情形**：若 $\mathcal{S}$ 不可数，用有界可测函数空间 $\mathcal{B}(\mathcal{S})$ 配 sup-norm，它仍是 Banach 空间

> **为什么用 sup-norm 而不是 $L^2$ 范数？** 因为 Bellman 算子在 sup-norm 下是 $\gamma$-压缩，但在 $L^2$ 范数下不一定是。sup-norm 的物理含义是"最差状态的误差"——我们关心的是最坏情况下的近似精度。

### 3.3 Bellman 算子的形式定义 ⭐

**Bellman 期望算子** $T^\pi: \mathcal{B}(\mathcal{S}) \to \mathcal{B}(\mathcal{S})$：
$$
(T^\pi V)(s) := \sum_{a} \pi(a|s) \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s') \right]
$$

**Bellman 最优算子** $T^*: \mathcal{B}(\mathcal{S}) \to \mathcal{B}(\mathcal{S})$：
$$
(T^* V)(s) := \max_{a} \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s') \right]
$$

用算子语言，Bellman 方程写成：
- $V^\pi = T^\pi V^\pi$（$V^\pi$ 是 $T^\pi$ 的不动点）
- $V^* = T^* V^*$（$V^*$ 是 $T^*$ 的不动点）

### 3.4 $\gamma$-压缩性的完整证明 ⭐⭐

**定理 3.1（核心定理：Bellman 算子的 $\gamma$-压缩性）**：对任意 $V, V' \in \mathcal{B}(\mathcal{S})$：
$$
\|T^\pi V - T^\pi V'\|_\infty \le \gamma \|V - V'\|_\infty
$$
$$
\|T^* V - T^* V'\|_\infty \le \gamma \|V - V'\|_\infty
$$

**证明（$T^*$ 情形，逐步展开，不省略任何一步）**：

**Step 1**：固定任意状态 $s$，写出 $(T^*V)(s) - (T^*V')(s)$ 的表达式：

$$
(T^*V)(s) - (T^*V')(s) = \max_a \left[r(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s')\right] - \max_a \left[r(s,a) + \gamma \sum_{s'} P(s'|s,a) V'(s')\right]
$$

**Step 2**：利用 **"max 的差 $\le$ 差的 max 绝对值"引理**。

> **引理（Max-Difference Lemma）**：对有界函数 $f, g: X \to \mathbb{R}$：
> $$|\sup_x f(x) - \sup_x g(x)| \le \sup_x |f(x) - g(x)|$$
> **证明**：设 $\sup f = f(x^*)$。则 $\sup f - \sup g = f(x^*) - \sup g \le f(x^*) - g(x^*) \le \sup_x(f(x)-g(x)) \le \sup_x|f(x)-g(x)|$。对称地交换 $f,g$ 可得另一方向。$\square$

应用该引理，令 $f(a) = r(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s')$，$g(a) = r(s,a) + \gamma \sum_{s'} P(s'|s,a) V'(s')$：

$$
|(T^*V)(s) - (T^*V')(s)| \le \max_a \left| \gamma \sum_{s'} P(s'|s,a) [V(s') - V'(s')] \right|
$$

注意 $r(s,a)$ 在 $f$ 和 $g$ 中相同，相减后消去。

**Step 3**：利用 $P(\cdot|s,a)$ 是概率分布的性质（非负且和为 1）：

$$
\left| \sum_{s'} P(s'|s,a) [V(s') - V'(s')] \right| \le \sum_{s'} P(s'|s,a) |V(s') - V'(s')| \le \sum_{s'} P(s'|s,a) \|V - V'\|_\infty = \|V - V'\|_\infty
$$

第一个 $\le$ 是三角不等式（绝对值的和 $\ge$ 和的绝对值），第二个 $\le$ 是用 sup-norm 放大每一项，最后一个 $=$ 是概率归一 $\sum_{s'} P(s'|s,a) = 1$。

**Step 4**：合并 Step 2 和 Step 3：
$$
|(T^*V)(s) - (T^*V')(s)| \le \gamma \|V - V'\|_\infty, \quad \forall s
$$

**Step 5**：对 $s$ 取 sup：
$$
\|T^*V - T^*V'\|_\infty = \max_s |(T^*V)(s) - (T^*V')(s)| \le \gamma \|V - V'\|_\infty
$$

$\square$

**$T^\pi$ 情形的证明**（更简单）：

$$
\begin{aligned}
|(T^\pi V)(s) - (T^\pi V')(s)| &= \left| \sum_a \pi(a|s) \gamma \sum_{s'} P(s'|s,a) [V(s') - V'(s')] \right| \\
&\le \sum_a \pi(a|s) \cdot \gamma \sum_{s'} P(s'|s,a) |V(s') - V'(s')| \\
&\le \gamma \|V - V'\|_\infty \cdot \underbrace{\sum_a \pi(a|s)}_{=1} \cdot \underbrace{\sum_{s'} P(s'|s,a)}_{=1} \\
&= \gamma \|V - V'\|_\infty
\end{aligned}
$$

对 $s$ 取 sup 即得。$T^\pi$ 情形更直接是因为策略 $\pi$ 给出的是凸组合（不是 max）。$\square$

> **本质洞察**：压缩性的物理来源是 $\gamma < 1$——每传播一步，未来的影响被衰减 $\gamma$ 倍。如果 $\gamma = 1$，压缩性失效，Banach 定理不适用——这就是为什么平均奖励 MDP（$\gamma = 1$）需要完全不同的理论工具。

### 3.5 Banach 不动点定理的应用 ⭐⭐

回顾前置知识（泛函分析）中的 **Banach 压缩映射定理**：

> **定理（Banach 1922）**：设 $(X, d)$ 是完备度量空间，$T: X \to X$ 是 $\gamma$-压缩映射（$d(Tx, Ty) \le \gamma \cdot d(x,y)$，$\gamma \in [0,1)$）。则：
> 1. $T$ 有**唯一**不动点 $x^* = Tx^*$
> 2. 对任意初值 $x_0$，迭代序列 $x_{k+1} = Tx_k$ 满足 $d(x_k, x^*) \le \gamma^k d(x_0, x^*)$
> 3. **先验误差界**：$d(x_k, x^*) \le \frac{\gamma^k}{1-\gamma} d(x_0, Tx_0)$

将 Banach 定理应用到我们的设置：

- 完备度量空间：$(\mathbb{R}^{|\mathcal{S}|}, \|\cdot\|_\infty)$ ✓
- $\gamma$-压缩映射：$T^\pi$ 或 $T^*$（刚证明）✓

**推论 3.2**：

1. $V^\pi$ 是 $T^\pi$ 的**唯一**不动点——即 Bellman 期望方程有唯一解
2. $V^*$ 是 $T^*$ 的**唯一**不动点——即 Bellman 最优方程有唯一解
3. 迭代 $V_{k+1} = T^* V_k$ 从**任意**初值出发，以速率 $\gamma^k$ 几何收敛到 $V^*$

> **为什么这是"心脏"**：这个定理把 RL 里本来复杂、时间结构纠缠的"寻找最优策略"问题，压缩成了一个简单而纯粹的对象——**Banach 空间上的不动点**。从此，所有主流 RL 算法都只是这个不动点求解器的变体：
> - VI = 纯迭代 $V_{k+1} = T^*V_k$
> - PI = Newton 型迭代（每步精确解一个线性系统）
> - TD(0) = 随机逼近 $T^\pi$
> - Q-learning = 随机逼近 $T^*$
> - DQN = 神经网络参数化 + 随机逼近 $T^*$
> - PPO = 在策略空间上的软化 PI

### 3.6 最优策略的存在性与唯一性 ⭐⭐

**定理 3.3（Bellman 最优性原理, Bellman 1957）**：最优策略的尾部仍然是最优策略——若 $\pi^*$ 在 $s$ 最优，则它从任意后继状态 $s'$ 起的行为也是从 $s'$ 起的最优策略。

**定理 3.4（折扣 MDP 的存在唯一性）**：在有限 MDP、$\gamma \in [0,1)$、$r$ 有界下：

1. Bellman 最优方程 $V = T^*V$ 有**唯一**解 $V^*$
2. 关于 $V^*$ 贪婪的确定性平稳策略 $\pi^*$ 存在且是**最优**的
3. 最优策略不一定唯一，但 $V^*$ 唯一

**证明**：

(1) Banach 不动点定理直接给出。

(2) 构造贪婪策略：
$$
\pi^*(s) \in \arg\max_a \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V^*(s') \right]
$$

有限动作空间保证 $\arg\max$ 存在。由构造，$(T^{\pi^*} V^*)(s) = (T^* V^*)(s) = V^*(s)$。

因此 $V^*$ 是 $T^{\pi^*}$ 的不动点。但 $T^{\pi^*}$ 的不动点唯一（推论 3.2），且等于 $V^{\pi^*}$。所以 $V^* = V^{\pi^*}$。

对任意策略 $\pi$，$V^\pi \le V^*$ 逐点成立（由 $V^*$ 的定义）。故 $\pi^*$ 最优。$\square$

### ⚠️ 常见陷阱

**思维陷阱：认为"$\gamma$ 越大越好"**

> **新手想法**：$\gamma$ 越大意味着智能体越有远见，应该总是取 $\gamma$ 尽量接近 1。
>
> **实际问题**：$\gamma \to 1$ 时，$(1-\gamma)^{-1} \to \infty$，停机误差界 $\varepsilon/(1-\gamma)$ 爆炸。这意味着：(a) 需要更多迭代才能收敛；(b) 值函数的数值范围变大，数值不稳定；(c) 信用分配变难——遥远的奖励和近期的奖励几乎同等重要。
>
> **正确思维**：$\gamma$ 是"有效视界 $1/(1-\gamma)$"的倒推。$\gamma = 0.99$ 意味着有效视界 100 步。选择 $\gamma$ 应匹配任务的自然时间尺度——locomotion 通常 $\gamma = 0.99$（20s 仿真中 100 步 = 2s 前瞻），而简单导航可能 $\gamma = 0.95$ 就够了。

### 练习

1. **补全 $T^\pi$ 压缩证明**：不参考本文，从零写出 $T^\pi$ 在 sup-norm 下 $\gamma$-压缩的完整证明。
2. **反例构造**：构造一个例子说明当 $\gamma = 1$ 时，$T^*$ 不再是压缩映射（提示：周期转移）。
3. **单调性**：证明 $T^*$ 是单调的——即 $V \le V'$ 逐点蕴含 $T^*V \le T^*V'$ 逐点。

---

## 4. 策略评估：计算 $V^\pi$ ⭐

### 4.1 动机：为什么需要策略评估

策略迭代的第一步就是"给当前策略打分"。你有了一个策略 $\pi$（可能很差），你需要精确知道它在每个状态的值 $V^\pi(s)$，然后才能找到改进方向。

策略评估回答的问题是：**给定一个固定策略 $\pi$，求解 $T^\pi V = V$ 这个不动点方程。**

> **反事实推理**：如果不做策略评估会怎样？我们可以直接用 Monte Carlo 方法——跑大量轨迹、算 $G_t$ 的平均。但 MC 方法方差大（每条轨迹的 $G_t$ 差异很大）、且需要完整的 episode。策略评估利用 Bellman 方程的递推结构，用"自举"方式获得低方差、在线的估计。

### 4.2 矩阵形式 ⭐

Bellman 期望方程是**线性**的——这是因为 $\pi$ 是固定的，$\sum_a \pi(a|s)[\cdot]$ 只是一个加权平均。

定义策略诱导的转移矩阵和奖励向量：
$$
P^\pi_{ss'} := \sum_a \pi(a|s) P(s'|s,a), \quad r^\pi_s := \sum_a \pi(a|s) r(s,a)
$$

则 Bellman 期望方程的矩阵形式为：
$$
V^\pi = r^\pi + \gamma P^\pi V^\pi
$$

整理得线性系统：
$$
(I - \gamma P^\pi) V^\pi = r^\pi
$$

**闭形式解**：
$$
\boxed{V^\pi = (I - \gamma P^\pi)^{-1} r^\pi}
$$

**为什么 $(I - \gamma P^\pi)$ 可逆？**

$P^\pi$ 是随机矩阵（每行和为 1），所以其谱半径 $\rho(P^\pi) \le 1$。由于 $\gamma < 1$，$\gamma P^\pi$ 的谱半径 $\le \gamma < 1$。因此 $I - \gamma P^\pi$ 的所有特征值的模 $\ge 1 - \gamma > 0$，矩阵可逆。

更进一步，$(I - \gamma P^\pi)^{-1}$ 有 Neumann 级数展开：
$$
(I - \gamma P^\pi)^{-1} = \sum_{k=0}^{\infty} (\gamma P^\pi)^k = I + \gamma P^\pi + \gamma^2 (P^\pi)^2 + \cdots
$$

> **物理解读**：$(P^\pi)^k$ 是"按策略 $\pi$ 走 $k$ 步后的状态转移矩阵"。所以 $(I - \gamma P^\pi)^{-1} r^\pi$ 是"所有未来步的折扣奖励之和"——即时奖励 + 1 步后的折扣奖励 + 2 步后的二次折扣奖励 + $\cdots$。这正是值函数定义的矩阵表述。

### 4.3 迭代法（策略评估迭代）⭐

直接求逆的复杂度是 $O(|\mathcal{S}|^3)$——对大状态空间不可行。替代方案是迭代法：

**算法：迭代策略评估**

$$
V_{k+1} = T^\pi V_k = r^\pi + \gamma P^\pi V_k, \quad V_0 \text{ 任意初值}
$$

**收敛性**：由 $T^\pi$ 的 $\gamma$-压缩性：
$$
\|V_k - V^\pi\|_\infty \le \gamma^k \|V_0 - V^\pi\|_\infty
$$

**收敛速度分析**：

定义误差 $\delta_k := V_k - V^\pi$。由 Bellman 方程 $V^\pi = r^\pi + \gamma P^\pi V^\pi$，代入迭代公式：
$$
\begin{aligned}
\delta_{k+1} &= V_{k+1} - V^\pi = (r^\pi + \gamma P^\pi V_k) - (r^\pi + \gamma P^\pi V^\pi) \\
&= \gamma P^\pi (V_k - V^\pi) = \gamma P^\pi \delta_k
\end{aligned}
$$

递推展开：
$$
\delta_k = (\gamma P^\pi)^k \delta_0
$$

由于 $P^\pi$ 是随机矩阵且 $\gamma < 1$，$\|(\gamma P^\pi)^k\|_\infty \le \gamma^k$。因此：
$$
\|\delta_k\|_\infty \le \gamma^k \|\delta_0\|_\infty
$$

这证明了**几何收敛**，收敛率由 $\gamma$ 决定。$\gamma$ 越接近 1，收敛越慢——这是"远视"的代价。

| $\gamma$ | 达到 $10^{-6}$ 精度需要的迭代次数 |
|----------|------------------------------|
| 0.9 | $\lceil \log(10^{-6}) / \log(0.9) \rceil = 132$ |
| 0.99 | $\lceil \log(10^{-6}) / \log(0.99) \rceil = 1379$ |
| 0.999 | $\lceil \log(10^{-6}) / \log(0.999) \rceil = 13812$ |

### 4.4 停机条件 ⭐

实际实现中，当 $\|V_{k+1} - V_k\|_\infty < \theta$（用户设定的阈值）时停止迭代。此时真实误差有界：
$$
\|V_k - V^\pi\|_\infty \le \frac{\theta}{1 - \gamma}
$$

> **为什么不是直接 $\le \theta$？** 因为 $\|V_{k+1} - V_k\|$ 度量的是"相邻两次迭代的变化"，不是"与真值的距离"。两者之间有 $(1-\gamma)^{-1}$ 的比例关系——这来自于几何级数的尾和。

### ⚠️ 常见陷阱

**编程陷阱：原地更新 vs 创建新数组**

> **错误做法**：在策略评估循环中，用 `V[s] = r + gamma * sum(P * V)` 直接原地更新 $V$。
>
> **后果**：更新 $V(s_1)$ 后，计算 $V(s_2)$ 时用的已经是新值——这不再是标准的同步 Bellman 更新，而是 **Gauss-Seidel 异步更新**。两者都收敛，但收敛速率不同，且 Gauss-Seidel 的收敛与状态遍历顺序有关。
>
> **正确做法（同步更新）**：`V_new = r + gamma * P @ V_old`，更新完所有状态后再替换 `V_old = V_new`。
>
> **自检方法**：每步更新后检查 $\|V_{new} - T^\pi V_{old}\|$ 是否为 0——如果不是，说明更新顺序出了问题。

### 练习

1. 对 2 个状态的 MDP，手动写出 $(I - \gamma P^\pi)$ 并验证其可逆性。与迭代法的结果对比。
2. 编程实现：用 NumPy 分别实现矩阵求逆法和迭代法，比较在 $|\mathcal{S}| = 100$ 时的运行时间。

---

## 5. 策略迭代：两步循环与有限步收敛 ⭐

### 5.1 动机：从评估到改进

策略评估告诉我们一个策略有多好。下一个自然问题是：**怎么让策略变得更好？**

> **类比**：策略迭代就像一个学生的学习循环——先做模拟考试评估自己（策略评估），再根据错题改进复习计划（策略改进）。每轮循环后，成绩单调提升，直到满分（最优）。

### 5.2 策略迭代算法 ⭐

**策略迭代（Policy Iteration, PI）**从任意策略 $\pi_0$ 开始，交替执行两步：

1. **策略评估（Policy Evaluation）**：求 $V^{\pi_k}$

   解 $(I - \gamma P^{\pi_k}) V^{\pi_k} = r^{\pi_k}$

2. **策略改进（Policy Improvement）**：

   $$\pi_{k+1}(s) \in \arg\max_a \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V^{\pi_k}(s') \right]$$

```python
# 策略迭代的 Python 伪代码
def policy_iteration(S, A, P, r, gamma, tol=1e-8):
    # 初始化：随机策略
    pi = random_policy(S, A)
    
    while True:
        # Step 1: 策略评估（矩阵求逆）
        P_pi = compute_transition_matrix(P, pi)  # |S| x |S|
        r_pi = compute_reward_vector(r, pi)      # |S| x 1
        V = np.linalg.solve(np.eye(len(S)) - gamma * P_pi, r_pi)
        
        # Step 2: 策略改进（贪婪）
        pi_new = greedy_policy(V, P, r, gamma)
        
        # 检查是否收敛
        if np.all(pi_new == pi):
            break
        pi = pi_new
    
    return pi, V
```

### 5.3 策略改进定理（Howard 1960）⭐⭐

**定理 5.1（Policy Improvement Theorem）**：设 $\pi'$ 关于 $V^\pi$ 贪婪，即对所有 $s$：
$$
\pi'(s) \in \arg\max_a \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V^\pi(s') \right]
$$

则 $V^{\pi'}(s) \ge V^\pi(s)$，对所有 $s$ 成立。等号对所有 $s$ 成立当且仅当 $\pi$ 已经是最优策略。

**完整证明**：

**Step 1**：由贪婪构造，对所有 $s$：
$$
(T^{\pi'} V^\pi)(s) = (T^* V^\pi)(s) \ge (T^\pi V^\pi)(s) = V^\pi(s)
$$

不等号成立是因为 $\max_a \ge \sum_a \pi(a|s)$ （max $\ge$ 加权平均）。最后的等号是因为 $V^\pi$ 是 $T^\pi$ 的不动点。

因此我们有：
$$
T^{\pi'} V^\pi \ge V^\pi \quad \text{（逐点）}
$$

**Step 2**：对两边再应用 $T^{\pi'}$。关键引理——$T^{\pi'}$ 是**单调的**：若 $U \ge W$ 逐点，则 $T^{\pi'} U \ge T^{\pi'} W$ 逐点。

证明单调性：$(T^{\pi'} U)(s) - (T^{\pi'} W)(s) = \gamma \sum_{s'} P^{\pi'}(s'|s)[U(s') - W(s')] \ge 0$，因为 $P^{\pi'}(s'|s) \ge 0$ 且 $U(s') - W(s') \ge 0$。

由单调性，对 $T^{\pi'} V^\pi \ge V^\pi$ 两边应用 $T^{\pi'}$：
$$
(T^{\pi'})^2 V^\pi \ge T^{\pi'} V^\pi \ge V^\pi
$$

**Step 3**：递推 $k$ 次：
$$
(T^{\pi'})^k V^\pi \ge V^\pi, \quad \forall k \ge 1
$$

**Step 4**：取 $k \to \infty$。由于 $T^{\pi'}$ 是 $\gamma$-压缩映射，$(T^{\pi'})^k V^\pi$ 收敛到 $T^{\pi'}$ 的唯一不动点 $V^{\pi'}$。因此：
$$
V^{\pi'} = \lim_{k \to \infty} (T^{\pi'})^k V^\pi \ge V^\pi
$$

**Step 5**（等号条件）：若 $V^{\pi'} = V^\pi$ 对所有 $s$，则 $V^\pi = T^{\pi'} V^\pi = T^* V^\pi$，即 $V^\pi$ 是 $T^*$ 的不动点。由唯一性，$V^\pi = V^*$，故 $\pi$ 已最优。$\square$

> **本质洞察**：策略改进定理说的是"贪婪改进永远不会变差"。这不是一个平凡的结论——贪婪地改变策略可能让某些状态变好、某些变差。定理告诉我们：**所有状态同时变好**（或不变）。这是因为 $T^{\pi'}$ 的单调性保证了改进"传播"到所有状态。

### 5.4 有限步终止证明 ⭐⭐

**定理 5.2（PI 在有限 MDP 中有限步终止）**：若 $|\mathcal{S}|, |\mathcal{A}| < \infty$，则策略迭代至多 $|\mathcal{A}|^{|\mathcal{S}|}$ 步终止于最优策略。

**证明**：

1. 确定性平稳策略的总数是 $|\mathcal{A}|^{|\mathcal{S}|}$（每个状态选一个动作）——有限。
2. 策略改进定理保证每步 $V^{\pi_{k+1}} \ge V^{\pi_k}$，且严格大于（除非已最优）。
3. 严格改进意味着 $V^{\pi_{k+1}} \ne V^{\pi_k}$，因此 $\pi_{k+1} \ne \pi_k$。
4. 不同的确定性策略只有有限多个，且序列严格改进（不重复），故必在有限步后无法再改进——即到达最优。$\square$

**更精细的界**：Ye (2011) 证明了 PI 的强多项式界 $O\left(\frac{|\mathcal{S}|^2 |\mathcal{A}|}{1-\gamma} \log \frac{1}{1-\gamma}\right)$——远小于 $|\mathcal{A}|^{|\mathcal{S}|}$ 的朴素界。

### 5.5 PI 的 Newton 法直觉 ⭐⭐⭐

策略迭代可以理解为 **Newton 法在 Bellman 方程上的对应**（Puterman 1994, Ch.6）。

值迭代是"一阶"方法（每步只做一次 $T^*$ 应用），收敛速率是线性的（$\gamma^k$）。策略迭代每步精确解一个线性系统——这相当于在 $T^* - I = 0$ 上做 Newton 步。Newton 法在适当条件下有二次收敛率——这解释了为什么 PI 在实践中通常只需要很少的迭代。

> **类比**：值迭代是梯度下降（小步前进），策略迭代是 Newton 法（大步跳到当前二次近似的极值点）。两者都收敛到同一个解，但 Newton 法更快——代价是每步计算量更大（需要解线性系统）。

### ⚠️ 常见陷阱

**概念误区：认为"策略改进必定严格增大"**

> **错误理解**：每次策略改进后，$V^{\pi_{k+1}}(s) > V^{\pi_k}(s)$ 对所有 $s$ 严格成立。
>
> **实际上**：改进定理只保证 $\ge$。如果 $\pi_k$ 已经是最优策略（或在某些状态已最优），改进后值不变。算法正是通过检测"改进后策略是否变化"来判断终止。
>
> **正确理解**：$V^{\pi_{k+1}} \ge V^{\pi_k}$ 逐点成立。严格 $>$ 在至少一个状态成立，**除非** $\pi_k$ 已经最优。

### 练习

1. **手算策略迭代**：对 $|\mathcal{S}|=2, |\mathcal{A}|=2$ 的 MDP，从随机策略出发，手动执行策略迭代直到收敛。记录每步的策略和值函数。
2. **证明单调性引理**：证明 Bellman 算子 $T^\pi$ 是单调的（$U \ge W \Rightarrow T^\pi U \ge T^\pi W$）。
3. （跨章综合）**PI 与 Newton 法对比**：回顾优化理论中 Newton 法的收敛性证明。类比地解释为什么 PI 比 VI 更快。具体地，PI 每步求解 $(I - \gamma P^{\pi_k}) V = r^{\pi_k}$ 对应 Newton 法中的哪一步？

---

## 6. 值迭代：收敛性、速率与异步变体 ⭐

### 6.1 动机：比策略迭代更简单

策略迭代每步需要精确求解线性系统——当 $|\mathcal{S}|$ 很大时代价高昂。值迭代（Value Iteration, VI）用"一步 $T^*$ 应用"代替"精确策略评估"，牺牲收敛速度换取每步的低计算量。

> **类比**：策略迭代像是"做一整套模拟题、改完所有错再做下一套"；值迭代像是"做一题改一题"。后者每步更快，但可能需要更多轮。

### 6.2 算法定义 ⭐

**值迭代（Value Iteration, VI）**：
$$
V_{k+1} := T^* V_k, \quad V_0 \in \mathcal{B}(\mathcal{S}) \text{ 任意初值}
$$

展开：
$$
V_{k+1}(s) = \max_a \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V_k(s') \right], \quad \forall s
$$

### 6.3 几何收敛定理 ⭐

**定理 6.1（VI 的 $\gamma^k$ 几何收敛）**：
$$
\|V_k - V^*\|_\infty \le \gamma^k \|V_0 - V^*\|_\infty
$$

**证明**：由 $V^* = T^* V^*$ 与 $T^*$ 的 $\gamma$-压缩性：
$$
\|V_{k+1} - V^*\|_\infty = \|T^* V_k - T^* V^*\|_\infty \le \gamma \|V_k - V^*\|_\infty
$$

递推 $k$ 次即得。$\square$

### 6.4 可实施停机条件与误差界 ⭐⭐

在实际计算中，$V^*$ 未知，无法直接计算 $\|V_k - V^*\|$。但相邻迭代的差是可观测的。

**定理 6.2（停机误差界）**：若 $\|V_{k+1} - V_k\|_\infty \le \varepsilon$，则：

$$
\|V_k - V^*\|_\infty \le \frac{\varepsilon}{1-\gamma}
$$

**证明**：由三角不等式和压缩性：
$$
\begin{aligned}
\|V_k - V^*\| &\le \|V_k - V_{k+1}\| + \|V_{k+1} - V^*\| \\
&\le \varepsilon + \gamma \|V_k - V^*\|
\end{aligned}
$$

整理得 $(1-\gamma)\|V_k - V^*\| \le \varepsilon$，即 $\|V_k - V^*\| \le \varepsilon/(1-\gamma)$。$\square$

**贪婪策略的误差界**：设 $\pi_k$ 是 $V_k$ 导出的贪婪策略，则：
$$
\|V^{\pi_k} - V^*\|_\infty \le \frac{2\gamma\varepsilon}{(1-\gamma)^2}
$$

> **工程含义**：$(1-\gamma)^{-2}$ 因子被称为 **horizon blow-up**。$\gamma = 0.99$ 时，$1/(1-\gamma)^2 = 10000$——即使值函数误差 $\varepsilon = 10^{-4}$，贪婪策略的性能差距可能高达 $2 \times 0.99 \times 10^{-4} \times 10000 \approx 2$。这提示我们：对 $\gamma$ 接近 1 的问题，需要非常精确的值函数估计。

### 6.5 VI 与 PI 的关系：修正策略迭代 ⭐⭐

**修正策略迭代（Modified PI, MPI）**：把策略评估中的"精确求解"换成"$m$ 步迭代"：
$$
V^{\pi_k} \approx (T^{\pi_k})^m V_k
$$

| $m$ | 算法名称 | 特点 |
|-----|---------|------|
| $m = 1$ | 值迭代 | 每步最快，线性收敛 |
| $m = \infty$ | 策略迭代 | 每步最慢，但总迭代次数最少 |
| $1 < m < \infty$ | 修正策略迭代 | 在两者之间平衡 |

实践中，$m = 5 \sim 20$ 通常是一个好的折中。

### 6.6 异步值迭代 ⭐⭐⭐

标准 VI 每步更新所有状态——在大状态空间中代价过高。**异步 VI** 允许每步只更新部分状态。

**定理 6.3（异步 VI 收敛性, Bertsekas 1982）**：只要每个状态被更新无限多次，异步 VI 仍然收敛到 $V^*$。

常见的异步策略：
- **Gauss-Seidel**：按固定顺序遍历状态，原地更新
- **优先扫描（Prioritized Sweeping）**：优先更新 Bellman 残差 $|V(s) - T^*V(s)|$ 最大的状态
- **Real-Time DP**：只更新当前轨迹经过的状态

> **工程连接**：Prioritized Sweeping 的思想直接影响了 DQN 中的 Prioritized Experience Replay——优先回放 TD 误差大的样本，就是异步 VI 在函数逼近设置下的类比。

### 6.7 复杂度总结 ⭐

| 算法 | 每步复杂度 | 迭代次数到 $\varepsilon$ 精度 | 总复杂度 |
|------|-----------|----------------------------|---------|
| 值迭代 | $O(|\mathcal{S}|^2 |\mathcal{A}|)$ | $O\left(\frac{1}{1-\gamma} \log \frac{1}{\varepsilon}\right)$ | $O\left(\frac{|\mathcal{S}|^2 |\mathcal{A}|}{1-\gamma} \log \frac{1}{\varepsilon}\right)$ |
| 策略迭代（矩阵求逆） | $O(|\mathcal{S}|^3 + |\mathcal{S}|^2 |\mathcal{A}|)$ | 有限步（Ye 2011 多项式界） | 通常远少于 VI |
| 策略迭代（迭代评估） | $O(m \cdot |\mathcal{S}|^2 |\mathcal{A}|)$ | 较少 | 依赖 $m$ 的选择 |

### ⚠️ 常见陷阱

**编程陷阱：停机条件误用**

> **错误**：把停机条件写成 `if max|V_new - V_old| < epsilon: break`，然后直接把 `epsilon` 当作真实误差。
>
> **后果**：真实误差是 $\varepsilon / (1-\gamma)$，当 $\gamma = 0.99$ 时真实误差是停机阈值的 100 倍！如果用这个 $V$ 导出贪婪策略，性能可能远不如预期。
>
> **正确做法**：如果需要 $V$ 的精度为 $\delta$，停机阈值应设为 $\theta = \delta \cdot (1-\gamma)$。

### 练习

1. **实验验证收敛率**：在 FrozenLake 环境上实现 VI，画出 $\log\|V_k - V^*\|_\infty$ vs $k$ 的图，验证斜率 $\approx \log\gamma$。
2. **MPI 实验**：实现修正策略迭代，比较 $m = 1, 5, 20, \infty$ 时的总计算量。
3. **异步 VI**：实现 Gauss-Seidel 和同步 VI，比较收敛速度。

---

## 7. 折扣因子的数学与直觉 ⭐⭐

### 7.1 折扣因子的三重含义

折扣因子 $\gamma \in [0,1)$ 在 MDP 理论中有三层含义：

**第一层：数学保障**

$\gamma < 1$ 保证：
- 累积回报有界：$|G_t| \le R_{\max}/(1-\gamma)$
- Bellman 算子是压缩映射：收敛性依赖于 $\gamma < 1$
- 值函数良定义：不会出现无穷大

> **反事实推理**：如果 $\gamma = 1$ 会怎样？考虑一个永不终止的 MDP，每步奖励 +1。累积回报 $G = 1 + 1 + 1 + \cdots = \infty$。值函数无定义。即使奖励有正有负，$G$ 的期望也可能不存在（条件期望不绝对收敛）。

**第二层：时间偏好**

$\gamma$ 体现"即时奖励比远期奖励更有价值"：
- $\gamma = 0$：完全短视，只看当前一步
- $\gamma \to 1$：极度远视，同等重视所有未来

**有效视界**：$\gamma^t$ 在 $t = 1/(1-\gamma)$ 步后衰减到约 $1/e \approx 0.37$。所以 $1/(1-\gamma)$ 是"有效考虑的未来步数"。

| $\gamma$ | 有效视界 | 物理时间（dt=0.02s） | 典型应用 |
|----------|---------|---------------------|---------|
| 0.9 | 10 步 | 0.2s | 短期稳定 |
| 0.99 | 100 步 | 2s | locomotion |
| 0.999 | 1000 步 | 20s | 长期规划 |
| 0.9999 | 10000 步 | 200s | 接近平均奖励 |

**第三层：不确定性惩罚**

$\gamma$ 还可以解释为"智能体存活的概率"。如果在每个时步，智能体以概率 $1-\gamma$ 意外终止（"死亡"），则期望累积奖励恰好是折扣累积回报。这给出了 $\gamma$ 的另一个直觉：未来越远越不确定，所以越打折。

### 7.2 折扣因子与几何级数的关系 ⭐

折扣因子使得累积回报成为几何级数，满足优美的数学性质：

$$
\sum_{t=0}^{\infty} \gamma^t = \frac{1}{1-\gamma}, \quad \sum_{t=0}^{\infty} t \gamma^t = \frac{\gamma}{(1-\gamma)^2}
$$

第一个公式给出值函数的上界，第二个给出"平均考虑的未来时步"。

### 7.3 工程中如何选择 $\gamma$ ⭐

选择 $\gamma$ 的原则：**有效视界应匹配任务的自然时间尺度。**

- locomotion（$dt = 0.02s$，一个步态周期 $\approx 0.5s$）：$\gamma = 0.99$，有效视界 2s ≈ 4 个步态周期。足够看到"这一步的决策对下几步的影响"
- dexterous manipulation（$dt = 0.01s$，动作持续 $\sim 5s$）：$\gamma = 0.995 \sim 0.999$
- 棋类游戏（每步对应一个落子，一局 $\sim 200$ 步）：$\gamma = 0.99 \sim 0.999$

> **不是X而是Y**：$\gamma$ 的选择不是"越大越好"（远视），而是"匹配任务时间尺度"。过大的 $\gamma$ 导致：(a) 收敛慢；(b) 值函数数值范围大；(c) 信用分配困难（无法区分哪步动作导致了远期奖励）。

### 练习

1. 对同一个 GridWorld MDP，分别用 $\gamma = 0.5, 0.9, 0.99$ 求解最优策略。策略有什么不同？直觉上为什么？
2. 推导：如果每步有 $1-\gamma$ 的概率终止，期望累积奖励恰好等于折扣回报的期望。

---

## 8. MDP 与最优控制的统一 ⭐⭐

### 8.1 动机：为什么要统一

如果你已经学过 MPC/LQR，你可能觉得 MDP 是"另一个世界"的东西。事实并非如此——**MDP 和最优控制是同一数学对象的不同面目**。

> **类比**：波粒二象性——光既是波又是粒子，取决于你用什么实验去观察它。同理，折扣随机动态规划既是 MDP（RL 社区的语言）又是最优控制问题（控制社区的语言），取决于你用什么记号去书写它。

### 8.2 完整对应表 ⭐⭐

| 控制语言 | MDP / RL 语言 | 备注 |
|---------|--------------|------|
| 状态 $x_t \in \mathbb{R}^n$ | 状态 $s_t \in \mathcal{S}$ | 连续 vs 离散 |
| 控制 $u_t \in \mathbb{R}^m$ | 动作 $a_t \in \mathcal{A}$ | — |
| 动力学 $x_{t+1} = f(x_t, u_t, w_t)$ | 转移核 $P(s_{t+1}|s_t, a_t)$ | RL 不要求 $f$ 显式 |
| 阶段成本 $\ell(x_t, u_t)$ | 奖励 $-r(s_t, a_t)$ | 符号相反！ |
| 终端成本 $\phi(x_H)$ | 终端奖励 | 有限时域 |
| cost-to-go $J^*_t(x)$ | 值函数 $V^*_t(s)$ | **同一对象** |
| Bellman 方程（控制） | Bellman 最优方程（MDP） | **同一定理** |
| MPC / receding-horizon | 近似 DP + rollout | Bertsekas (2019) |
| LQR | 特殊 MDP，闭形式 $V^*(x) = x^\top P x$ | Riccati 方程 |
| HJB 方程 | 连续时间 Bellman 方程 | $dt \to 0$ |
| Pontryagin 极大值原理 | 对偶问题 | 伴随态 = $\nabla V$ |

### 8.3 从 MDP 到 LQR ⭐⭐

把 MDP 的各元素特殊化：
- $\mathcal{S} = \mathbb{R}^n$，$\mathcal{A} = \mathbb{R}^m$
- 动力学线性：$x_{t+1} = Ax_t + Bu_t + w_t$，$w_t \sim \mathcal{N}(0, \Sigma)$
- 奖励二次：$r(x,u) = -(x^\top Q x + u^\top R u)$，$Q \succeq 0, R \succ 0$

代入 Bellman 最优方程，猜测 $V^*(x) = -x^\top P x + c$（二次型）：

$$
-x^\top P x + c = \max_u \left[ -(x^\top Q x + u^\top R u) + \gamma \mathbb{E}[-(Ax+Bu+w)^\top P(Ax+Bu+w) + c] \right]
$$

展开、对 $u$ 求导令其为零，得到最优控制律：
$$
u^* = -(R + \gamma B^\top P B)^{-1} \gamma B^\top P A \cdot x := -K x
$$

将 $u^*$ 回代，得到 $P$ 满足的**离散代数 Riccati 方程（DARE）**：
$$
P = Q + \gamma A^\top P A - \gamma^2 A^\top P B(R + \gamma B^\top P B)^{-1} B^\top P A
$$

> **本质洞察**：LQR 之所以有闭形式解，是因为线性动力学+二次成本保证了值函数是二次型。一般非线性 MDP 的值函数没有简单形式——这就是为什么需要函数逼近（神经网络）来表示值函数。

### 8.4 从 MDP 到 HJB ⭐⭐⭐

将时间连续化（$dt \to 0$），离散 Bellman 方程退化为 **Hamilton-Jacobi-Bellman (HJB) 方程**：

$$
-\frac{\partial V}{\partial t}(x,t) = \min_u \left[ \ell(x,u) + \nabla_x V(x,t) \cdot f(x,u) \right]
$$

推导思路：
1. 离散 Bellman：$V(x_t) = \min_u [\ell(x_t, u_t) \cdot dt + V(x_{t+1})]$
2. 展开 $V(x_{t+1}) = V(x_t + f(x_t, u_t) \cdot dt) \approx V(x_t) + \nabla V \cdot f \cdot dt$
3. 代入消去 $V(x_t)$，两边除以 $dt$ 取极限

### 8.5 统一视角的实用价值 ⭐⭐

理解这种统一关系的实用价值：

1. **如果你会 MPC**：MDP/Bellman 方程对你只是换了记号。困难点是理解"为什么还要搞 RL"——答案是当 $f$ 未知、解析不可行、或维度过高时，VI/PI 的样本版（TD、Q-learning、PPO）比 MPC 更可扩展。

2. **交叉引用**：本章的 Bellman 压缩性与控制理论方向的 DP/HJB 章节直接对应。MPC 的收缩性分析（Grüne & Pannek 2017）本质上是 Bellman 压缩的有限时域版本。

3. **算法迁移**：iLQR/DDP 可以看作"在连续空间上的近似策略迭代"——每步线性化动力学+二次化成本，然后在 LQR 子问题上做精确 PI。

### 练习

1. 对一维 LQR（$n=m=1, A=1.1, B=1, Q=1, R=0.1, \gamma=0.99$），手动推导 Riccati 方程并求解 $P$。验证最优增益 $K$。
2. （跨章综合）比较 MPC 与值迭代：对同一个线性系统，分别用有限时域 DP（$H=10$）和折扣无限时域 VI（$\gamma=0.99$）求解。最优策略相同吗？为什么？

---

## 9. 占用测度与 LP 对偶 ⭐⭐⭐

### 9.1 动机：值函数之外的另一个视角

前面所有内容都围绕"值函数"展开。但 MDP 还有另一个等价的数学表述——**占用测度（Occupancy Measure）**视角。这个视角是现代 RL 理论（GAIL、Imitation Learning、Offline RL、策略梯度定理的几何基础）的标准语言。

> **类比**：值函数视角看 MDP 就像"从单个状态出发看未来"；占用测度视角看 MDP 就像"从上帝视角看智能体在状态-动作空间中的访问频率分布"。

### 9.2 占用测度的定义 ⭐⭐⭐

**定义**：策略 $\pi$ 的**折扣占用测度（discounted occupancy measure）**是：
$$
d^\pi(s,a) := (1-\gamma) \sum_{t=0}^{\infty} \gamma^t \mathbb{P}^\pi_{\mu_0}(S_t = s, A_t = a)
$$

乘以 $(1-\gamma)$ 是为了归一化：$\sum_{s,a} d^\pi(s,a) = 1$，使其成为真正的概率分布。

**物理含义**：$d^\pi(s,a)$ 是"按策略 $\pi$ 运行时，智能体在 $(s,a)$ 上花费的折扣时间比例"。

### 9.3 MDP 的 LP 表示 ⭐⭐⭐

有限折扣 MDP 可以等价地表示为**线性规划**（de Ghellinck 1960; Manne 1960; Puterman 1994 Ch.6.9）。

**原始 LP**（以值函数 $V$ 为决策变量）：
$$
\min_V \sum_s \mu_0(s) V(s) \quad \text{s.t.} \quad V(s) \ge r(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s'), \quad \forall (s,a)
$$

**对偶 LP**（以**未归一化**占用测度 $\rho(s,a)$ 为决策变量）：
$$
\max_{\rho \ge 0} \sum_{s,a} \rho(s,a) r(s,a) \quad \text{s.t.} \quad \sum_a \rho(s',a) = \mu_0(s') + \gamma \sum_{s,a} P(s'|s,a) \rho(s,a), \quad \forall s'
$$

> **归一化约定说明**：此处 $\rho(s,a)=\sum_{t=0}^\infty\gamma^t P(S_t=s,A_t=a|\pi)$ 是**未归一化**版本，满足 $\sum_{s,a}\rho(s,a) = \frac{1}{1-\gamma}$。与 §9.2 中的归一化定义 $d^\pi = (1-\gamma)\rho^\pi$ 的关系为 $\rho = \frac{1}{1-\gamma}d^\pi$。LP 对偶使用未归一化版本的原因是：流量守恒约束的右端 $\mu_0(s')$ 是概率分布（归一到 1），与未归一化 $\rho$ 配合后约束结构最自然——若改用归一化 $d$，流量约束变为 $\sum_a d(s',a) = (1-\gamma)\mu_0(s') + \gamma\sum_{s,a}P(s'|s,a)d(s,a)$，多出一个 $(1-\gamma)$ 因子。

**对偶约束的物理意义**：状态 $s'$ 的"流入量"（从初始分布 $\mu_0(s')$ + 从其他状态折扣转移 $\gamma\sum P\rho$）等于"流出量"（在 $s'$ 处对所有动作的占用总和 $\sum_a\rho(s',a)$）。这是一个**流量守恒约束**（Bellman flow constraint）。

**强对偶性**：LP 强对偶成立，原始和对偶的最优目标值相等。对偶 LP 的最优解 $\rho^*$ 给出最优策略：
$$
\pi^*(a|s) = \frac{\rho^*(s,a)}{\sum_{a'} \rho^*(s, a')}
$$

### 9.4 占用测度视角的价值 ⭐⭐⭐

为什么要引入这个看似多余的表述？因为：

1. **策略梯度定理**：$\nabla_\theta J(\pi_\theta) = \frac{1}{1-\gamma}\mathbb{E}_{s \sim d^{\pi_\theta},a\sim\pi_\theta}[\nabla_\theta \log \pi_\theta(a|s) \cdot Q^{\pi_\theta}(s,a)]$——梯度是在归一化占用测度 $d^{\pi_\theta}$ 下的期望，$\frac{1}{1-\gamma}$ 因子来自归一化常数。
2. **模仿学习 (GAIL)**：最小化策略的占用测度与专家的占用测度之间的 f-divergence。
3. **约束 MDP (CMDP)**：约束条件自然写成占用测度的线性函数。
4. **Offline RL**：分布偏移问题的核心是"学习策略的占用测度偏离数据收集策略的占用测度"。

### 练习

1. 验证占用测度的归一化：证明 $\sum_{s,a} d^\pi(s,a) = 1$。
2. 对 $|\mathcal{S}|=2, |\mathcal{A}|=2$ 的 MDP，手动写出对偶 LP 并用单纯形法求解。与 VI 的结果对比。

---

## 10. 典型例题 ⭐

### 10.1 Grid World 完整求解

**问题设置**：$4 \times 4$ 网格世界（FrozenLake 变体）。

- 状态：$\mathcal{S} = \{(i,j): 0 \le i,j \le 3\}$，$|\mathcal{S}| = 16$
- 动作：$\mathcal{A} = \{\text{上, 下, 左, 右}\}$
- 转移：以 0.7 概率到达目标方向，0.1 概率滑到其他三个方向之一。撞墙则留在原地
- 奖励：到达右下角 $(3,3)$ 得 +1，掉入陷阱 $(1,1), (2,3)$ 得 -1，其余为 -0.01
- 折扣：$\gamma = 0.9$

**手动执行 VI 的前几步**：

初始化 $V_0(s) = 0$ 对所有 $s$。

第 1 步：只有与终点相邻的状态获得非零值。$V_1(3,2) = 0.7 \times 1 + 0.1 \times 0 + \cdots = 0.7$（近似）。

第 2 步：非零值传播到更远的状态。

经过约 50 次迭代，$V_k$ 收敛。最优策略指向右下角，同时避开陷阱。

```python
import numpy as np

def value_iteration_gridworld(gamma=0.9, tol=1e-6):
    """4x4 Grid World 值迭代完整实现"""
    n_states = 16
    n_actions = 4  # 上下左右
    
    # 构建转移矩阵 P[s, a, s'] 和奖励 R[s, a]
    P = np.zeros((n_states, n_actions, n_states))
    R = np.zeros((n_states, n_actions))
    
    # ... (转移和奖励的具体构造省略，见练习)
    
    # 值迭代主循环
    V = np.zeros(n_states)
    for iteration in range(10000):
        V_new = np.max(R + gamma * np.einsum('ijk,k->ij', P, V), axis=1)
        if np.max(np.abs(V_new - V)) < tol:
            print(f"收敛于第 {iteration} 次迭代")
            break
        V = V_new
    
    # 导出贪婪策略
    Q = R + gamma * np.einsum('ijk,k->ij', P, V)
    policy = np.argmax(Q, axis=1)
    return V, policy
```

### 10.2 库存管理 MDP ⭐⭐

**问题**：商店每天决定进货量，面对随机需求。

- 状态 $s$：当前库存水平，$\mathcal{S} = \{0, 1, \ldots, M\}$
- 动作 $a$：进货量，$\mathcal{A}(s) = \{0, 1, \ldots, M-s\}$（不超过仓库容量）
- 转移：需求 $D \sim \text{Poisson}(\lambda)$，下一状态 $s' = \max(0, s + a - D)$
- 奖励：$r(s,a) = p \cdot \min(s+a, D) - c \cdot a - h \cdot \max(0, s+a-D)$
  - 销售收入 $p \cdot \text{销量}$ - 进货成本 $c \cdot a$ - 持有成本 $h \cdot \text{剩余}$

这是一个经典的运筹学/库存管理问题。最优策略是 $(s, S)$ 策略：库存低于 $s$ 时补货到 $S$。

### 10.3 机器人导航 MDP ⭐⭐

**问题**：移动机器人在带障碍物的 2D 环境中导航到目标。

- 状态：机器人位姿 $(x, y, \theta)$ 的离散化
- 动作：前进、左转、右转
- 转移：动作成功概率 0.8，失败（打滑/碰撞）概率 0.2
- 奖励：到达目标 +10，碰撞障碍物 -5，每步时间惩罚 -0.1

**与图搜索的关系**：当转移确定（$P = 1$）、$\gamma = 1$ 时，VI 退化为 **Dijkstra 最短路径算法**。MDP 的一般形式处理的是带不确定性的路径规划——这正是 Probabilistic Roadmap + belief propagation 等方法的数学基础。

### 练习

1. 完成 Grid World 的完整实现（构造 $P$ 和 $R$），跑 VI 和 PI，比较迭代次数。
2. 对库存管理 MDP，用 $M=10, \lambda=3, p=10, c=3, h=1, \gamma=0.95$ 的参数求解最优策略，验证是否符合 $(s,S)$ 结构。
3. （跨章综合）将机器人导航 MDP 的离散化版本与 A* 算法比较。什么情况下 MDP 方法优于确定性图搜索？

---

## 11. 深入理解：从多角度看 Bellman 方程 ⭐⭐

### 11.1 矩阵视角：线性代数解读

把 Bellman 期望方程写成矩阵形式后，我们可以用线性代数的眼光看它。

定义策略 $\pi$ 诱导的 **转移矩阵** $P^\pi \in \mathbb{R}^{|\mathcal{S}| \times |\mathcal{S}|}$，其中 $P^\pi_{ss'} = \sum_a \pi(a|s) P(s'|s,a)$。这个矩阵是**随机矩阵（stochastic matrix）**——每行非负且和为 1。

随机矩阵有以下重要性质：

| 性质 | 数学表述 | 物理意义 |
|------|---------|---------|
| 非负 | $P^\pi \ge 0$（逐元素） | 概率不能为负 |
| 行和为 1 | $P^\pi \mathbf{1} = \mathbf{1}$ | 从任何状态出发必定到某处 |
| 谱半径 $\le 1$ | $\rho(P^\pi) \le 1$ | 概率不会放大 |
| 特征值 1 的特征向量 | $P^\pi \mathbf{1} = 1 \cdot \mathbf{1}$ | 平稳分布存在 |

因此 $\gamma P^\pi$ 的谱半径 $\le \gamma < 1$，$I - \gamma P^\pi$ 的所有特征值实部 $> 0$，矩阵正定（在适当意义下），求逆是良定义的。

**Neumann 级数的物理解读**：
$$
(I - \gamma P^\pi)^{-1} = I + \gamma P^\pi + \gamma^2 (P^\pi)^2 + \gamma^3 (P^\pi)^3 + \cdots
$$

- $I$：当前时刻（$t=0$）的贡献
- $\gamma P^\pi$：下一步（$t=1$）的折扣转移
- $\gamma^2 (P^\pi)^2$：两步后（$t=2$）的折扣转移
- $\gamma^k (P^\pi)^k$：$k$ 步后的折扣转移

所以 $(I - \gamma P^\pi)^{-1} r^\pi$ 是"对所有未来时步的折扣奖励求和"——这正是值函数定义的矩阵形式验证。

### 11.2 概率论视角：鞅与条件期望

从概率论角度看，值函数 $V^\pi$ 与随机过程的条件期望紧密相关。定义过程：
$$
M_t := \sum_{k=0}^{t-1} \gamma^k R_k + \gamma^t V^\pi(S_t)
$$

**断言**：$\{M_t\}_{t \ge 0}$ 是一个**鞅（martingale）**（关于自然滤波）。

验证：
$$
\begin{aligned}
\mathbb{E}[M_{t+1} | \mathcal{F}_t] &= \mathbb{E}\left[\sum_{k=0}^{t} \gamma^k R_k + \gamma^{t+1} V^\pi(S_{t+1}) \,\Big|\, \mathcal{F}_t\right] \\
&= \sum_{k=0}^{t-1} \gamma^k R_k + \gamma^t R_t + \gamma^{t+1} \mathbb{E}[V^\pi(S_{t+1}) | S_t] \\
&= \sum_{k=0}^{t-1} \gamma^k R_k + \gamma^t \underbrace{\left(R_t + \gamma \mathbb{E}[V^\pi(S_{t+1}) | S_t]\right)}_{= V^\pi(S_t) \text{（Bellman 方程！）}} \\
&= M_t
\end{aligned}
$$

这个鞅性质正是 **TD 学习**的理论基础——TD 误差 $\delta_t = R_t + \gamma V^\pi(S_{t+1}) - V^\pi(S_t)$ 是鞅差序列（martingale difference sequence），期望为零。

### 11.3 控制论视角：Lyapunov 稳定性

从控制论角度看，值迭代的收敛可以用 **Lyapunov 函数** 来理解。

定义 Lyapunov 函数 $L_k := \|V_k - V^*\|_\infty$。则：
$$
L_{k+1} = \|T^*V_k - T^*V^*\|_\infty \le \gamma L_k
$$

这是一个**指数稳定**的离散 Lyapunov 方程——$L_k$ 以速率 $\gamma$ 指数衰减到零。对于控制工程师来说，这与线性系统 $x_{k+1} = Ax_k$ 在 $\rho(A) < 1$ 时的指数稳定性完全类比。

> **双重解读**：
> - **RL 语言**：值迭代以几何速率收敛到最优值函数
> - **控制语言**：Bellman 算子作为离散动力系统，以 $V^*$ 为全局渐近稳定平衡点

### 11.4 信息论视角：折扣因子与信息衰减 ⭐⭐⭐

$\gamma$ 可以理解为"信息衰减系数"。在每一步转移中，关于初始状态的信息以速率 $\gamma$ 衰减——这与通信中信号在噪声信道中的衰减类比。

具体地，考虑从两个不同初始值函数 $V_0, V_0'$ 出发的 VI 序列。经过 $k$ 步迭代后：
$$
\|V_k - V_k'\|_\infty \le \gamma^k \|V_0 - V_0'\|_\infty
$$

这意味着**初始条件的影响以指数速率被遗忘**——不管初始猜测多差，最终都会收敛到同一个 $V^*$。

> **反事实推理**：如果 $\gamma = 1$，初始误差永远不会衰减。这就是为什么平均奖励 MDP 的求解比折扣 MDP 困难得多——没有天然的"信息遗忘"机制来保证收敛。

---

## 12. 算法实现的工程细节 ⭐⭐

### 12.1 数值精度与稳定性

在实现 VI/PI 时，需要注意以下数值问题：

**浮点精度**：当 $\gamma$ 接近 1 时，$(I - \gamma P^\pi)$ 的条件数增大。对 $\gamma = 0.999$，$\kappa(I - \gamma P^\pi) \approx 1000$——这意味着输入误差被放大 1000 倍。

**解决方案**：
- 使用双精度浮点（float64），不要用 float32
- 避免直接求逆 $(I - \gamma P^\pi)^{-1}$，改用 `numpy.linalg.solve`
- 对大系统，使用迭代法（Gauss-Seidel）代替直接法

### 12.2 稀疏转移矩阵

实际 MDP 中，转移矩阵 $P$ 通常是**高度稀疏**的——从一个状态只能转移到少数几个邻近状态。

- Grid World：每个状态最多 4 个后继 → 每行最多 4 个非零元素
- 机器人离散化：每个关节 $\pm 1$ 格点 → 稀疏度 $\sim |\mathcal{A}|/|\mathcal{S}|$

利用稀疏性：
```python
import scipy.sparse as sp

# 稀疏转移矩阵存储
P_sparse = sp.csr_matrix(P)  # 压缩稀疏行格式

# 矩阵向量乘法 O(nnz) 而非 O(|S|^2)
V_new = r + gamma * P_sparse @ V
```

### 12.3 并行化

现代 RL 训练（如 Isaac Lab）使用 GPU 并行化：
- 数千个环境同时运行 → 数千个独立的 MDP 实例
- 状态、动作、奖励都是批量张量
- VI/PI 的矩阵运算可以批量化（PyTorch / JAX）

### 12.4 完整 Python 实现框架

```python
import numpy as np
from typing import Tuple

class TabularMDP:
    """Tabular MDP 数据结构"""
    def __init__(self, n_states: int, n_actions: int, 
                 P: np.ndarray, r: np.ndarray, gamma: float):
        """
        参数:
            P: 转移矩阵 [n_states, n_actions, n_states]
            r: 奖励函数 [n_states, n_actions]
            gamma: 折扣因子, 必须严格 < 1
        """
        assert 0 <= gamma < 1, "gamma 必须在 [0, 1) 内"
        assert np.allclose(P.sum(axis=-1), 1.0), "P 的每行必须和为 1"
        assert np.all(P >= 0), "P 必须非负"
        
        self.n_states = n_states
        self.n_actions = n_actions
        self.P = P
        self.r = r
        self.gamma = gamma
    
    def bellman_optimality_operator(self, V: np.ndarray) -> np.ndarray:
        """T^* V: Bellman 最优算子的一次应用"""
        # Q(s,a) = r(s,a) + gamma * sum_s' P(s'|s,a) * V(s')
        Q = self.r + self.gamma * np.einsum('ijk,k->ij', self.P, V)
        return np.max(Q, axis=1)
    
    def bellman_policy_operator(self, V: np.ndarray, 
                                 pi: np.ndarray) -> np.ndarray:
        """T^pi V: Bellman 期望算子"""
        # P^pi[s, s'] = sum_a pi(a|s) * P(s'|s,a)
        P_pi = np.einsum('sa,sak->sk', pi, self.P)
        r_pi = np.einsum('sa,sa->s', pi, self.r)
        return r_pi + self.gamma * P_pi @ V
    
    def value_iteration(self, tol: float = 1e-8, 
                        max_iter: int = 10000) -> Tuple[np.ndarray, np.ndarray]:
        """值迭代算法
        返回: (V^*, 贪婪策略)
        """
        V = np.zeros(self.n_states)
        errors = []
        
        for k in range(max_iter):
            V_new = self.bellman_optimality_operator(V)
            error = np.max(np.abs(V_new - V))
            errors.append(error)
            
            if error < tol:
                break
            V = V_new
        
        # 导出贪婪策略
        Q = self.r + self.gamma * np.einsum('ijk,k->ij', self.P, V)
        policy = np.argmax(Q, axis=1)
        
        return V, policy, errors
    
    def policy_iteration(self, max_iter: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
        """策略迭代算法
        返回: (V^*, 最优策略)
        """
        # 初始化随机确定性策略
        policy = np.random.randint(0, self.n_actions, size=self.n_states)
        
        for k in range(max_iter):
            # Step 1: 策略评估（矩阵求逆）
            # 构造 P^pi 和 r^pi
            P_pi = self.P[np.arange(self.n_states), policy, :]
            r_pi = self.r[np.arange(self.n_states), policy]
            
            # 求解 (I - gamma * P^pi) V = r^pi
            A = np.eye(self.n_states) - self.gamma * P_pi
            V = np.linalg.solve(A, r_pi)
            
            # Step 2: 策略改进
            Q = self.r + self.gamma * np.einsum('ijk,k->ij', self.P, V)
            new_policy = np.argmax(Q, axis=1)
            
            # 检查收敛
            if np.all(new_policy == policy):
                break
            policy = new_policy
        
        return V, policy
```

### ⚠️ 常见陷阱

**编程陷阱：einsum 索引顺序错误**

> **错误**：写 `np.einsum('ijk,j->ik', P, V)` 来计算 $\sum_{s'} P(s'|s,a) V(s')$。
>
> **后果**：$V$ 的维度是 $|\mathcal{S}|$（对应 $s'$），但 `j` 对应 $P$ 的第二维（动作维度），导致维度不匹配或静默错误。
>
> **正确做法**：明确 $P$ 的索引含义。如果 $P[s,a,s']$ 表示从 $s$ 执行 $a$ 到 $s'$ 的概率，则 $\sum_{s'} P[s,a,s'] V[s']$ 应写作 `np.einsum('ijk,k->ij', P, V)`。
>
> **自检方法**：检查 einsum 结果的 shape 是否符合预期（应为 `[n_states, n_actions]`）。

---

## 13. 值迭代与策略迭代的深层比较 ⭐⭐

### 13.1 收敛速度实验对比

以一个具体例子说明 VI 和 PI 的收敛差异。

设 $|\mathcal{S}| = 100, |\mathcal{A}| = 4, \gamma = 0.99$。随机生成 MDP，跑两种算法：

| 算法 | 迭代次数（$\varepsilon = 10^{-8}$） | 每步时间 | 总时间 |
|------|----------------------------------|---------|--------|
| VI | $\sim 1800$ 次 | $\sim 0.1$ ms | $\sim 180$ ms |
| PI | $\sim 8$ 次 | $\sim 2$ ms | $\sim 16$ ms |

PI 虽然每步计算量大（需要求解 $100 \times 100$ 线性系统），但总迭代次数远少——体现了"Newton 法 vs 一阶迭代"的经典权衡。

### 13.2 何时选 VI、何时选 PI

| 场景 | 推荐算法 | 原因 |
|------|---------|------|
| $|\mathcal{S}|$ 很大（$> 10^4$） | VI 或 Modified PI | 矩阵求逆 $O(|\mathcal{S}|^3)$ 太贵 |
| $|\mathcal{S}|$ 中等（$< 10^3$） | PI | Newton 法优势明显 |
| $\gamma$ 很接近 1（$> 0.999$） | PI | VI 需要太多迭代 |
| $\gamma$ 较小（$< 0.9$） | VI | 收敛足够快，每步便宜 |
| 在线/实时更新 | 异步 VI | 不需要全局扫描 |
| 已有好的初始策略 | PI（从该策略出发） | 少数改进即可到达最优 |

### 13.3 修正策略迭代的最优选择

理论上，MPI 的 $m$（内层评估迭代次数）应如何选择？

Puterman (1994) 的分析表明：对大多数实际问题，$m \in [5, 20]$ 是一个稳健的选择。直觉是：
- $m$ 太小（如 $m=1$，退化为 VI）：外层需要很多次才能收敛
- $m$ 太大（如 $m=\infty$，退化为 PI）：每步评估过精确，浪费计算在"中间策略"上
- 中间值：评估精度"够用即可"——不需要精确到 $V^\pi$，只需要"大致方向正确"就能保证改进

> **类比**：这和优化中的"inexact Newton method"一样——不需要精确解 Newton 方程，只需要近似解满足一定的充分下降条件即可。

---

## 14. Bellman 最优性原理的深层理解 ⭐⭐

### 14.1 最优性原理的精确陈述

**Bellman 最优性原理（Principle of Optimality, Bellman 1957）**：

> 一个最优策略具有这样的性质：无论初始状态和初始决策如何，剩余的决策必须构成关于由初始决策所产生的状态的最优策略。

形式化：设 $\pi^*$ 是最优策略，$(S_0, A_0, S_1, A_1, \ldots)$ 是 $\pi^*$ 生成的轨迹。则对任意 $t \ge 1$ 和任意 $\pi^*$ 可达的 $s'$：从 $s'$ 起遵循 $\pi^*$ 的尾部策略仍然是从 $s'$ 起的最优策略。

**为什么这不是平凡的？** 考虑一个非 Markov 的决策问题——最优策略可能依赖于"到达 $s'$ 的路径"。Markov 性质保证了"不管怎么到达 $s'$，从 $s'$ 起的最优行为是一样的"——这就是最优性原理的本质。

### 14.2 最优性原理的反例（非 Markov 情形）

考虑一个简单的记忆迷宫：

```
     [A] ---→ [B] ---→ [C] (目标, +10)
      |                  ↑
      |                  |
      ↓                  |
     [D] ---→ [E] ---→ [F]
```

如果智能体在 [B] 和 [E] 看到的观测**完全相同**（部分可观测），那么最优行为依赖于"是从 [A] 还是从 [D] 来的"——即依赖于历史。此时最优性原理不成立（在观测空间上），必须转到 belief 空间才能恢复 Markov 性和最优性原理。

### 14.3 最优性原理与子结构

最优性原理直接导出 Bellman 方程的递推结构：

$$
V^*(s) = \max_a \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) \underbrace{V^*(s')}_{\text{子问题的最优解}} \right]
$$

这个方程说："$s$ 的最优值 = 当前最优动作的即时奖励 + 折扣后子问题的最优值"。

> **与计算机科学的连接**：最优性原理等价于动态规划的"最优子结构"性质。最短路径问题（Dijkstra）、背包问题、编辑距离——所有能用 DP 求解的问题都具有最优子结构。MDP 是这一类问题在**随机环境**下的推广。

---

## 15. 从 MDP 到现代 RL：概念桥梁 ⭐⭐

### 15.1 Tabular 方法的局限与函数逼近的必要性

本章讨论的所有方法（VI, PI）都是 **tabular** 的——为每个状态维护一个独立的值。当 $|\mathcal{S}|$ 很大时，这不可行：

| 问题 | $|\mathcal{S}|$ | 存储 $V$ 所需内存 | 可行性 |
|------|----------------|-----------------|--------|
| 4x4 Grid World | 16 | 128 字节 | 轻松 |
| 围棋 | $\sim 10^{170}$ | 不可能 | 不可行 |
| 四足机器人（连续） | $\infty$ | 不可能 | 不可行 |

**解决方案**：用函数逼近器（神经网络、线性基函数）表示 $V(s) \approx V_\theta(s)$。这引入了两个新挑战：
1. **逼近误差**：$V_\theta$ 无法精确表示 $V^*$
2. **采样误差**：无法遍历所有状态，只能用样本估计 Bellman 更新

这两个挑战催生了整个现代 Deep RL 领域——DQN、PPO、SAC 都是在解决"有逼近误差和采样误差的 Bellman 不动点"问题。

### 15.2 TD 学习：从 Bellman 方程到样本更新

TD(0) 是策略评估的**样本版本**。把 Bellman 期望方程：
$$
V^\pi(s) = \mathbb{E}[R + \gamma V^\pi(S') | S=s]
$$

转化为一个基于单样本 $(s, r, s')$ 的增量更新：
$$
V(s) \leftarrow V(s) + \alpha \underbrace{[r + \gamma V(s') - V(s)]}_{\text{TD 误差 } \delta}
$$

**TD 误差 $\delta$ 的含义**：如果 $V = V^\pi$，则 $\mathbb{E}[\delta | S=s] = 0$（因为 Bellman 方程成立）。因此 TD 更新是"用随机梯度下降逼近 Bellman 方程的不动点"。

### 15.3 Q-learning：从 VI 到样本版本

Q-learning 是值迭代的**样本版本**。把 Bellman 最优方程的 Q 形式：
$$
Q^*(s,a) = \mathbb{E}[R + \gamma \max_{a'} Q^*(S', a') | S=s, A=a]
$$

转化为样本更新：
$$
Q(s,a) \leftarrow Q(s,a) + \alpha [r + \gamma \max_{a'} Q(s', a') - Q(s,a)]
$$

> **本质洞察**：从 tabular DP 到 model-free RL 的核心跨越是：用**单次采样**代替**完整期望**（$\sum_{s'} P(s'|s,a)[\cdot]$ 变成了对单个 $s'$ 的观测）。收敛性从"压缩映射的确定性迭代"变成了"随机逼近理论（Robbins-Monro）下的收敛"——数学上更困难，但实用性大大增强。

### 15.4 PPO：从策略迭代到深度强化学习

PPO（Proximal Policy Optimization）可以理解为策略迭代的"软化"版本：

| PI 步骤 | PPO 对应 |
|---------|---------|
| 精确策略评估 $V^{\pi_k}$ | 用 GAE（广义优势估计）从样本近似 $A^{\pi_k}$ |
| 贪婪策略改进 $\pi_{k+1} = \arg\max$ | 在 trust region 内对 $\pi_\theta$ 做梯度上升 |
| 改进保证 $V^{\pi_{k+1}} \ge V^{\pi_k}$ | 单调改进的近似保证（通过 clipping） |

PPO 的 trust region 约束 $|\pi_\theta(a|s) / \pi_{\theta_k}(a|s) - 1| \le \epsilon$ 就是为了保证"改进不会太大"——太大的改进可能破坏评估的准确性（因为评估是用旧策略的样本做的）。

---

## 16. 经典例题详解：两状态 MDP 的完整求解 ⭐

### 16.1 问题设置

设 $\mathcal{S} = \{s_1, s_2\}$，$\mathcal{A} = \{a_1, a_2\}$，$\gamma = 0.9$。

转移概率：
$$
P(s_1 | s_1, a_1) = 0.7, \quad P(s_2 | s_1, a_1) = 0.3
$$
$$
P(s_1 | s_1, a_2) = 0.4, \quad P(s_2 | s_1, a_2) = 0.6
$$
$$
P(s_1 | s_2, a_1) = 0.5, \quad P(s_2 | s_2, a_1) = 0.5
$$
$$
P(s_1 | s_2, a_2) = 0.2, \quad P(s_2 | s_2, a_2) = 0.8
$$

奖励：
$$
r(s_1, a_1) = 5, \quad r(s_1, a_2) = 10, \quad r(s_2, a_1) = -1, \quad r(s_2, a_2) = 2
$$

### 16.2 值迭代手算

初始化 $V_0 = [0, 0]^\top$。

**第 1 次迭代**：
$$
V_1(s_1) = \max\{r(s_1,a_1) + 0.9[0.7 \cdot 0 + 0.3 \cdot 0],\ r(s_1,a_2) + 0.9[0.4 \cdot 0 + 0.6 \cdot 0]\} = \max\{5, 10\} = 10
$$
$$
V_1(s_2) = \max\{-1 + 0, \ 2 + 0\} = 2
$$

$V_1 = [10, 2]^\top$，贪婪策略 $\pi_1 = [a_2, a_2]$。

**第 2 次迭代**：
$$
V_2(s_1) = \max\{5 + 0.9[0.7 \cdot 10 + 0.3 \cdot 2],\ 10 + 0.9[0.4 \cdot 10 + 0.6 \cdot 2]\}
$$
$$
= \max\{5 + 0.9 \cdot 7.6,\ 10 + 0.9 \cdot 5.2\} = \max\{11.84, 14.68\} = 14.68
$$
$$
V_2(s_2) = \max\{-1 + 0.9[0.5 \cdot 10 + 0.5 \cdot 2],\ 2 + 0.9[0.2 \cdot 10 + 0.8 \cdot 2]\}
$$
$$
= \max\{-1 + 5.4,\ 2 + 3.24\} = \max\{4.4, 5.24\} = 5.24
$$

$V_2 = [14.68, 5.24]^\top$，贪婪策略 $\pi_2 = [a_2, a_2]$。

继续迭代直到收敛……最终 $V^* \approx [47.32, 37.56]^\top$，最优策略 $\pi^* = [a_2, a_2]$。

> **验证**：可以直接解 Bellman 最优方程组。以 $\pi^*=[a_2,a_2]$ 为例，转移矩阵 $P^{\pi^*}=\bigl(\begin{smallmatrix}0.4&0.6\\0.2&0.8\end{smallmatrix}\bigr)$，奖励 $r^{\pi^*}=(10,-1+3)^\top=(10,2)^\top$。解 $(I-0.9P^{\pi^*})V=r^{\pi^*}$ 即得 $V^*\approx(47.32,37.56)^\top$。

### 16.3 策略迭代手算

初始策略 $\pi_0 = [a_1, a_1]$（两个状态都选动作 1）。

**策略评估**：
$$
P^{\pi_0} = \begin{pmatrix} 0.7 & 0.3 \\ 0.5 & 0.5 \end{pmatrix}, \quad r^{\pi_0} = \begin{pmatrix} 5 \\ -1 \end{pmatrix}
$$

解 $(I - 0.9 P^{\pi_0}) V = r^{\pi_0}$：
$$
\begin{pmatrix} 1 - 0.63 & -0.27 \\ -0.45 & 1 - 0.45 \end{pmatrix} V = \begin{pmatrix} 5 \\ -1 \end{pmatrix}
$$
$$
\begin{pmatrix} 0.37 & -0.27 \\ -0.45 & 0.55 \end{pmatrix} V = \begin{pmatrix} 5 \\ -1 \end{pmatrix}
$$

解得 $V^{\pi_0} \approx [30.24, 22.93]^\top$。

> **手算验证**：$\det\bigl(\begin{smallmatrix}0.37&-0.27\\-0.45&0.55\end{smallmatrix}\bigr) = 0.37\times0.55 - 0.27\times0.45 = 0.2035-0.1215=0.082$。由 Cramer 法则 $V(s_1)=(5\times0.55+(-1)\times0.27)/0.082=(2.75-0.27)/0.082=2.48/0.082\approx30.24$，$V(s_2)=(0.37\times(-1)+0.45\times5)/0.082=(-0.37+2.25)/0.082=1.88/0.082\approx22.93$。

**策略改进**：计算 $Q(s, a)$ 对所有 $(s,a)$，选 argmax。经过 2-3 轮迭代即收敛到最优策略。

### 16.4 收敛曲线验证

画出 $\log_{10}\|V_k - V^*\|_\infty$ vs $k$：

- 理论预测：斜率 $= \log_{10}(0.9) \approx -0.046$
- 实验验证：斜率确实接近 $-0.046$，确认几何收敛

这个实验清晰地验证了定理 6.1：VI 的收敛率由 $\gamma$ 唯一决定。

---

## 17. Legged Locomotion 的 MDP 设计实例 ⭐⭐

### 17.1 ANYmal 四足机器人的 MDP（Rudin et al. 2022）

按 Rudin et al. (2022) 对 ANYmal 的形式化：

**状态** $s \in \mathbb{R}^{48}$：
- 基座线速度（3 维，IMU 估计）
- 基座角速度（3 维，IMU）
- 重力方向在体坐标系的投影（3 维）
- 12 维关节位置（相对于默认站立姿态的偏差）
- 12 维关节速度
- 3 维速度指令 $(v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_{\text{yaw}}^{\text{cmd}})$
- 12 维上一步动作（处理通信延迟）

**动作** $a \in \mathbb{R}^{12}$：关节目标位置的偏置。经 PD 控制器映射到力矩：
$$
\tau = K_p (a - q) + K_d (0 - \dot{q})
$$

> **注意**：动作不是力矩！而是 PD 控制器的目标位置偏置。这是一个关键的工程设计决策——让策略学习"位置空间的偏移"比直接学习"力矩"更容易，因为位置空间更平滑。

**转移** $P$：MuJoCo/PhysX 仿真器前进一步（$dt = 0.02s$）。数千个并行环境。

**奖励** $r = \sum_i w_i r_i$：
| 奖励分项 | 公式 | 权重方向 |
|---------|------|---------|
| 线速度跟踪 | $\exp(-\|v_{xy} - v_{xy}^{\text{cmd}}\|^2 / \sigma)$ | + |
| 角速度跟踪 | $\exp(-\|\omega_z - \omega_z^{\text{cmd}}\|^2 / \sigma)$ | + |
| 关节扭矩惩罚 | $\|\tau\|^2$ | - |
| 关节加速度惩罚 | $\|\ddot{q}\|^2$ | - |
| 足部滑动惩罚 | $\sum_{\text{feet}} \|v_{\text{foot}}\| \cdot \mathbb{1}_{\text{contact}}$ | - |
| 动作变化率 | $\|a_t - a_{t-1}\|^2$ | - |

**折扣**：$\gamma = 0.99$，回合长度 1000 步（20 秒仿真时间）。

### 17.2 从 MDP 视角理解 Sim-to-Real

为什么仿真中学到的策略能迁移到真实机器人？从 MDP 角度看：

1. **Domain Randomization**：训练时随机化 MDP 的参数（摩擦系数、质量、电机延迟）。这相当于在"MDP 集合"上求解一个鲁棒策略——使其对 MDP 参数的扰动不敏感。

2. **Privileged Learning**：教师网络访问完整状态（包含仿真器内部状态），学生网络只用传感器观测。从 MDP/POMDP 角度看：教师在 MDP 上训练，学生通过行为蒸馏学会了一个隐式的 belief state encoder。

> **不是X而是Y**：Domain Randomization 不是让仿真更接近真实，而是让策略对参数变化更鲁棒。真实世界只是随机化覆盖的分布中的一个点。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "折扣因子 $\gamma$ 只是为了让级数收敛的数学技巧" | $\gamma$ 有明确的物理含义：它编码了对未来奖励的时间偏好。$\gamma$ 小意味着"短视"（更看重近期回报），$\gamma$ 接近 1 意味着"远视"。在机器人控制中，$\gamma$ 的选择直接影响策略是否愿意"承受短期代价以换取长期收益"（如步态切换时的瞬态不稳定） |
| "值迭代和策略迭代最终结果一样，所以无所谓用哪个" | 两者虽都收敛到最优策略，但收敛速率和计算特性完全不同。策略迭代在每一轮都做完整的策略评估（精确求解线性系统），因此迭代次数少（通常 $O(\frac{SA}{1-\gamma}\log\frac{1}{1-\gamma})$），但每轮计算量大；值迭代每轮只做一次 Bellman 更新，计算量小但总迭代次数多（$O(\frac{1}{1-\gamma}\log\frac{1}{\varepsilon(1-\gamma)})$）。工程中应根据状态空间规模和精度要求选择 |
| "Bellman 方程只是一种递推公式" | Bellman 方程的本质是 Banach 空间上的算子不动点方程 $T^*V = V$。压缩映射定理不仅保证了不动点的**存在唯一性**，还给出了求解算法（迭代法）和**误差收缩速率**（$\gamma^k$）。这个算子视角是后续 TD 学习、投影 Bellman 算子、致命三元组等所有理论的出发点 |
| "MDP 的 Markov 性质太强了，真实机器人不满足" | 关键在于**状态定义**。如果状态包含了足够的信息（如关节位置+速度+接触力+延迟补偿），系统就满足 Markov 性。状态设计的艺术正是把"历史依赖"压缩进"当前状态"——这也是为什么机器人 RL 的观测通常包含多帧历史或使用 RNN/Transformer 做状态编码 |
| "策略改进定理说明贪心就是最优" | 策略改进定理说的是"对当前值函数贪心得到的策略不差于当前策略"，但这不意味着一步贪心就能跳到全局最优。它保证的是**单调改进**，需要迭代执行"评估→改进→评估→..."才能收敛到最优。贪心操作是局部的（对 $V^\pi$ 贪心），但迭代过程是全局收敛的 |

---

## 本章小结

| 概念 | 核心内容 | 关键公式 | 档位 |
|------|---------|---------|------|
| MDP 七元组 | $(\mathcal{S}, \mathcal{A}, P, r, \gamma, \mu_0, H)$ | 定义 1.3 | ⭐ |
| Markov 性质 | 给定当前，未来与过去独立 | $P(s'|s,a,\text{history}) = P(s'|s,a)$ | ⭐ |
| 状态值函数 | 从 $s$ 出发遵循 $\pi$ 的期望回报 | $V^\pi(s) = \mathbb{E}^\pi[\sum \gamma^t R_t | S_0=s]$ | ⭐ |
| Bellman 期望方程 | $V^\pi$ 的递推关系 | $V^\pi = r^\pi + \gamma P^\pi V^\pi$ | ⭐ |
| Bellman 最优方程 | $V^*$ 的递推关系 | $V^* = \max_a[r + \gamma P V^*]$ | ⭐ |
| $\gamma$-压缩性 | Bellman 算子的核心性质 | $\|TV - TV'\| \le \gamma\|V-V'\|$ | ⭐⭐ |
| Banach 不动点 | 存在唯一性+几何收敛 | 定理 3.2 | ⭐⭐ |
| 策略评估 | 求 $V^\pi$ | $(I-\gamma P^\pi)^{-1} r^\pi$ | ⭐ |
| 策略迭代 | 评估→改进循环 | 定理 5.1, 5.2 | ⭐ |
| 值迭代 | $V_{k+1} = T^*V_k$ | $\|V_k - V^*\| \le \gamma^k \|V_0 - V^*\|$ | ⭐ |
| 折扣因子 | 有效视界 $\approx 1/(1-\gamma)$ | — | ⭐⭐ |
| MDP↔控制 | 同一数学对象 | 对应表 8.2 | ⭐⭐ |
| 占用测度 | 折扣访问频率 | $d^\pi(s,a)$ 的 LP | ⭐⭐⭐ |

**三句话记住本章**：

1. **RL = 在 Banach 空间 $(\mathcal{B}(\mathcal{S}), \|\cdot\|_\infty)$ 上解 Bellman 算子不动点 $V = TV$**；由 $\gamma$-压缩 + Banach 定理得存在唯一性与几何收敛。
2. **VI 线性收敛 ($\gamma^k$)，PI 有限步收敛 (tabular)**；两者被 MPI 连续桥接；所有后续 RL 算法都是这对基石的随机/样本/参数化化身。
3. **MDP = 离散时间折扣随机 DP = 最优控制的离散+随机特例**；去掉随机性得确定型 DP，连续化得 HJB，线性化+二次化得 LQR。

---

## 累积项目：本章新增模块

**项目：从零构建 Tabular RL 求解器**

本章新增：
- `mdp.py`：MDP 数据结构定义（$\mathcal{S}, \mathcal{A}, P, r, \gamma$）
- `value_iteration.py`：VI 实现 + 收敛曲线绘制
- `policy_iteration.py`：PI 实现（矩阵求逆 + 迭代评估两种模式）
- `gridworld.py`：FrozenLake 环境的 $P, R$ 矩阵构造

后续章节将在此基础上添加：TD 学习、Q-learning、策略梯度等模块。

---

## 延伸阅读

| 资源 | 难度 | 推荐理由 |
|------|------|---------|
| Sutton & Barto Ch.3-4 | ⭐⭐ | RL 领域最经典入门，直觉优先 |
| Puterman (1994) Ch.2, 5-6 | ⭐⭐⭐⭐ | MDP 权威参考，定理级严格 |
| Bertsekas (2019) RLOC Ch.1 | ⭐⭐⭐⭐ | 控制人视角的 RL 金本位 |
| Meyn (2022) Part I | ⭐⭐⭐⭐ | 控制+RL 统一叙事 |
| AJKS (2022) Ch.1-2 | ⭐⭐⭐⭐ | 现代样本复杂度视角 |
| Szepesvári (2010) Ch.2 | ⭐⭐⭐ | 90 分钟快速复习 |
| 蘑菇书 EasyRL Ch.1-3 | ⭐⭐ | 中文入门首推 |
| David Silver Lecture 2-3 (YouTube) | ⭐⭐ | 入门金标准视频 |
| OpenAI Spinning Up Part 1 | ⭐⭐ | 工程师最友好 |
| Bertsekas ASU 13 讲 | ⭐⭐⭐⭐ | 控制人最合胃口的视频 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| VI 不收敛（值持续增长） | $\gamma \ge 1$ 或奖励无界 | 1. 检查 $\gamma$ 是否严格 $< 1$；2. 检查 $r$ 是否有上界；3. 检查是否有正奖励循环 | §3, §7 |
| PI 第一步后策略不变 | 初始策略恰好是最优的，或贪婪改进有 tie | 1. 换随机初始策略重试；2. 检查 argmax 的 tie-breaking 规则 | §5 |
| 策略评估迭代极慢 | $\gamma$ 接近 1 | 1. 降低 $\gamma$ 测试；2. 改用矩阵求逆或增加评估步数 $m$ | §4 |
| 贪婪策略性能远不如预期 | 值函数精度不够 + horizon blow-up | 1. 减小停机阈值 $\theta$；2. 使用 $\theta = \text{target\_err} \cdot (1-\gamma)$ | §6 |
| 转移矩阵 $P$ 每行和 $\ne 1$ | 构造 $P$ 时遗漏了某些转移 | 1. 验证 `P.sum(axis=-1)` 全为 1；2. 检查边界状态（撞墙回弹）是否正确处理 | §1 |

---

## 附录 A0：Bellman 算子的额外性质

### A0.1 单调性（Monotonicity）

**命题**：$T^*$ 是单调的——若 $V(s) \le W(s)$ 对所有 $s$，则 $(T^*V)(s) \le (T^*W)(s)$ 对所有 $s$。

**证明**：对任意 $s$，
$$
(T^*V)(s) = \max_a [r(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s')] \le \max_a [r(s,a) + \gamma \sum_{s'} P(s'|s,a) W(s')]= (T^*W)(s)
$$

不等号成立因为 $V(s') \le W(s')$ 对所有 $s'$，而 $P(s'|s,a) \ge 0$，所以 $\sum_{s'} P(s'|s,a) V(s') \le \sum_{s'} P(s'|s,a) W(s')$。max 保持不等式方向。$\square$

**单调性的重要意义**：它保证了策略改进定理中"一步改进传播到所有步"的关键推导。单调性 + 压缩性一起，构成了 MDP 理论中 Bellman 算子的两大核心性质。

### A0.2 并行压缩性（Contraction in Span Semi-Norm）

对于平均奖励 MDP（$\gamma = 1$），Bellman 算子不再是 sup-norm 下的压缩映射。但它仍然是 **span semi-norm** 下的压缩：

$$
\text{sp}(T^*V - T^*W) \le \gamma_{\text{sp}} \cdot \text{sp}(V - W)
$$

其中 $\text{sp}(f) := \max_s f(s) - \min_s f(s)$，$\gamma_{\text{sp}} < 1$ 依赖于转移矩阵的混合性质。这是平均奖励 MDP 收敛性证明的技术基础（Puterman 1994, Ch.8）。

### A0.3 从 $V^*$ 到 $Q^*$ 的转换

已知 $V^*$ 后，$Q^*$ 的计算是直接的：
$$
Q^*(s,a) = r(s,a) + \gamma \sum_{s'} P(s'|s,a) V^*(s')
$$

这不需要额外的迭代——只是一次矩阵乘法。反过来，$V^*(s) = \max_a Q^*(s,a)$。

为什么要同时维护 $V$ 和 $Q$？因为：
- 导出最优策略只需要 $Q$（取 argmax），而不需要知道 $P$
- Model-free 方法（Q-learning）直接学习 $Q^*$，不经过 $V^*$
- 但在 model-based 设置中，维护 $V$ 更节约存储（$|\mathcal{S}|$ vs $|\mathcal{S}| \times |\mathcal{A}|$）

---

## 附录 A：关键证明的技术细节

### A.1 Max-Difference 引理的完整证明

**引理**：对有界函数 $f, g: X \to \mathbb{R}$，$|\sup_x f(x) - \sup_x g(x)| \le \sup_x |f(x) - g(x)|$。

**证明**：
设 $x^* \in \arg\max f(x)$。则：
$$
\sup f - \sup g = f(x^*) - \sup g \le f(x^*) - g(x^*) \le \sup_x [f(x) - g(x)] \le \sup_x |f(x) - g(x)|
$$

第一个 $\le$ 是因为 $\sup g \ge g(x^*)$（sup 不小于任何点的值）。
第二个 $\le$ 是 sup 的定义。
第三个 $\le$ 是 $a \le |a|$。

交换 $f, g$ 的角色得：$\sup g - \sup f \le \sup_x |g(x) - f(x)| = \sup_x |f(x) - g(x)|$。

合并两个方向：$|\sup f - \sup g| \le \sup_x |f(x) - g(x)|$。$\square$

### A.2 $(I - \gamma P^\pi)$ 可逆性的严格证明

**定理**：若 $P^\pi$ 是随机矩阵（行和为 1）且 $\gamma \in [0,1)$，则 $I - \gamma P^\pi$ 可逆。

**证明**（用 Neumann 级数）：
随机矩阵 $P^\pi$ 的谱半径 $\rho(P^\pi) \le \|P^\pi\|_\infty = 1$（因为每行绝对值之和 = 1）。

因此 $\|\gamma P^\pi\|_\infty \le \gamma < 1$，即 $\gamma P^\pi$ 的算子范数严格小于 1。

由 Neumann 级数定理：当 $\|A\| < 1$ 时，$I - A$ 可逆且 $(I-A)^{-1} = \sum_{k=0}^{\infty} A^k$。

取 $A = \gamma P^\pi$：
$$
(I - \gamma P^\pi)^{-1} = \sum_{k=0}^{\infty} (\gamma P^\pi)^k = I + \gamma P^\pi + \gamma^2 (P^\pi)^2 + \cdots
$$

该级数绝对收敛（$\|(\gamma P^\pi)^k\| \le \gamma^k \to 0$）。$\square$

### A.3 策略迭代贪婪策略的推导

为什么贪婪策略改进对每个状态独立地选 argmax 就够了？

$$
\pi_{k+1}(s) = \arg\max_a \left[ r(s,a) + \gamma \sum_{s'} P(s'|s,a) V^{\pi_k}(s') \right]
$$

这等价于：$\pi_{k+1} = \arg\max_\pi (r^\pi + \gamma P^\pi V^{\pi_k})$（整体最大化）。

关键观察：目标函数关于 $\pi$ 是**可分离的**——每个状态 $s$ 的选择独立。具体地：

$$
\sum_s \mu(s) \sum_a \pi(a|s) Q^{\pi_k}(s,a)
$$

对每个 $s$，独立最大化 $\sum_a \pi(a|s) Q^{\pi_k}(s,a)$。由于 $\pi(\cdot|s)$ 是概率分布，最优解是把所有概率放在最大 $Q$ 值对应的动作上——即确定性贪婪策略。

---

## 附录 B：与后续章节的桥梁

**向后（本方向内部）**：

- **策略梯度与 Actor-Critic**：本章的 Bellman 期望算子 $T^\pi$ 是 policy gradient theorem 的起点；occupancy measure 视角（§9）就是 $\nabla_\theta J(\pi_\theta)$ 推导中的核心换元。
- **TD 学习、Q-learning**：TD(0) 是 $T^\pi$ 的随机逼近，Q-learning 是 $T^*$ 的随机逼近。收敛性基于 Robbins-Monro 定理 + 本章的压缩性。
- **LQR / HJB**：本章 §8.3 已预演 LQR-as-MDP；连续时间取极限得 HJB。

**向外（其他方向）**：

- **控制方向 DP/HJB 章节**：MPC 的 receding-horizon 就是本章有限时域 DP 的在线近似。
- **概率论与状态估计**：POMDP（§1.7）与 SLAM 的滤波同源——都是贝叶斯更新。Belief-MDP 是 SLAM-aware planning 的正规数学框架。

**向前回看**：压缩映射定理、Banach 空间、算子范数、完备性——缺一则本章塌陷。

---

## 附录 C：平均奖励 MDP 简介 ⭐⭐⭐⭐

当 $\gamma \to 1$ 时折扣目标退化，常用的替代是**长期平均奖励**：
$$
g^\pi(s) := \liminf_{T\to\infty} \frac{1}{T} \mathbb{E}^\pi\left[\sum_{t=0}^{T-1} r(S_t, A_t) \,\Big|\, S_0 = s\right]
$$

在**单链（unichain）条件**下，$g^\pi(s)$ 不依赖 $s$，记作 $g^\pi$。Bellman 方程变为：
$$
g + h(s) = \max_a \left[ r(s,a) + \sum_{s'} P(s'|s,a) h(s') \right]
$$

其中 $h$ 称为 bias function（相对值函数）。没有 $\gamma$ 作压缩因子，需要 Blackwell 最优性或 vanishing discount 方法建立存在性。

**贴现-平均等价性**：在单链假设下，$(1-\gamma)V^*_\gamma(s) \to g^*$ uniform over $s$。这正是 legged locomotion 这类没有自然终止的任务选 $\gamma = 0.99$ 而不是 $\gamma = 1$ 的数学原因。

---

## 附录 D：约束 MDP（CMDP）简介 ⭐⭐⭐⭐

带约束 MDP：
$$
\max_\pi \mathbb{E}^\pi \sum_t \gamma^t r(S_t, A_t) \quad \text{s.t.} \quad \mathbb{E}^\pi \sum_t \gamma^t c_i(S_t, A_t) \le d_i, \quad i = 1,\ldots,m
$$

**Altman (1999) 的关键结果**：在有限 CMDP 中，最优策略一般是**随机**（而非确定性）平稳策略。这与无约束 MDP 的"最优策略可取确定性"形成鲜明对比。

核心工具是 Lagrangian 松弛：
$$
\mathcal{L}(\pi, \lambda) = \mathbb{E}^\pi \sum_t \gamma^t \left[ r(S_t, A_t) - \sum_i \lambda_i c_i(S_t, A_t) \right] + \sum_i \lambda_i d_i, \quad \lambda \ge 0
$$

这是 Safe RL（CPO、Lagrangian PPO、RCPO）的理论基础，在机器人安全约束（关节限幅、接触力上界）中广泛应用。

---

## 附录 E：核心论文与资源

### 关键论文时间线

| 年份 | 论文 | 贡献 |
|------|------|------|
| 1922 | Banach, "Sur les opérations dans les ensembles abstraits" | 压缩映射定理 |
| 1957 | Bellman, *Dynamic Programming* | DP 圣经，最优性原理 |
| 1960 | Howard, *Dynamic Programming and Markov Processes* | 策略迭代 |
| 1960 | Manne, "Linear Programming and Sequential Decisions" | MDP 的 LP 表示 |
| 1965 | Blackwell, "Discounted Dynamic Programming" | 折扣 DP 严格基础 |
| 1994 | Puterman, *Markov Decision Processes* | MDP 理论权威 |
| 2011 | Ye, "The simplex and PI are strongly polynomial..." | PI 强多项式界 |
| 2018 | Sutton & Barto, *RL: An Introduction* 2nd ed | RL 入门圣经 |
| 2019 | Bertsekas, *RL and Optimal Control* | 控制人的 RL |
| 2022 | Meyn, *Control Systems and RL* | 控制+RL 统一 |

### 免费 PDF 资源

- Sutton & Barto 2nd ed：`http://incompleteideas.net/book/RLbook2020.pdf`
- Bertsekas (2019) 样章+视频：`https://faculty.engineering.asu.edu/bertsekas/`
- Meyn (2022) pre-pub：`https://meyn.ece.ufl.edu/2021/08/01/control-systems-and-reinforcement-learning/`
- AJKS (2022) 蓝皮书：`https://rltheorybook.github.io/rltheorybook_ABJKS.pdf`
- 蘑菇书 EasyRL：`https://datawhalechina.github.io/easy-rl/`
- OpenAI Spinning Up：`https://spinningup.openai.com/`

### 视频课程

- David Silver DeepMind RL (UCL 2015)：Lecture 2-3 覆盖 MDP/DP
- UC Berkeley CS285 (Sergey Levine)：Lecture 2-5
- Bertsekas ASU 13 讲：控制人最合胃口
- 李宏毅《深度强化学习》(B 站)：中文

---

## 附录 F：自测题完整版

| # | 题目 | 档位 | 预计用时 |
|---|------|------|---------|
| Q1 | 不参考本文，独立写出 $T^*$ 在 sup-norm 下 $\gamma$-压缩的完整证明。 | ⭐⭐ | 30 min |
| Q2 | 设 $\mathcal{S}=\{s_1,s_2\}$，$\mathcal{A}=\{a_1,a_2\}$，自选 $P,r$，$\gamma=0.9$。手算 $V^*$：先写 $T^*$ 的显式表达式；再用 VI 迭代 20 步。画出 $\log\|V_k-V^*\|$ vs $k$，验证斜率 $\approx \log 0.9$。 | ⭐⭐ | 2h |
| Q3 | 证明策略改进定理——关键步骤"$(T^{\pi'})^k V^\pi \ge V^\pi$ 的单调性"要自己推完。 | ⭐⭐ | 45 min |
| Q4 | 在 Gymnasium `FrozenLake-v1` 上实现 VI 和 PI，比较迭代次数，与理论界吻合吗？ | ⭐⭐ | 3h |
| Q5 | 推导 $\|V^{\pi_k} - V^*\| \le 2\gamma\varepsilon/(1-\gamma)^2$ 的完整证明。 | ⭐⭐⭐ | 1h |
| Q6 | LQR 作为特殊 MDP：推导 Riccati 方程。 | ⭐⭐⭐ | 2h |
| Q7 | 读 Rudin et al. 2022 的 §III/§IV，用七元组语言写出 ANYmal locomotion MDP。 | ⭐⭐ | 1.5h |
| Q8 | 证明 PI 至多 $|\mathcal{A}|^{|\mathcal{S}|}$ 步终止。 | ⭐⭐ | 20 min |

---

> **完**。
> 本章是"RL 数学"方向的地基。读完后，你应当能用一句话回答："PPO 为什么会收敛？"——答案是："因为它是一次 soft 的策略迭代，每步在 trust region 内做策略改进（定理 5.1），而策略评估由神经网络对 $T^\pi$ 的随机逼近完成。收敛性继承自本章的 Bellman 压缩与 Banach 不动点定理。"

---

## 附录 G：术语对照表

| 英文 | 中文 | 符号 | 首次出现 |
|------|------|------|---------|
| Markov Decision Process | 马尔可夫决策过程 | $\mathcal{M}$ | §1 |
| State space | 状态空间 | $\mathcal{S}$ | §1.3 |
| Action space | 动作空间 | $\mathcal{A}$ | §1.3 |
| Transition kernel | 转移核 | $P(s'|s,a)$ | §1.3 |
| Reward function | 奖励函数 | $r(s,a)$ | §1.3 |
| Discount factor | 折扣因子 | $\gamma$ | §1.3 |
| Policy | 策略 | $\pi$ | §1.5 |
| State-value function | 状态值函数 | $V^\pi(s)$ | §2.2 |
| Action-value function | 动作值函数 | $Q^\pi(s,a)$ | §2.3 |
| Bellman expectation equation | Bellman 期望方程 | $V^\pi = T^\pi V^\pi$ | §2.4 |
| Bellman optimality equation | Bellman 最优方程 | $V^* = T^* V^*$ | §2.5 |
| Contraction mapping | 压缩映射 | $\|Tf - Tg\| \le \gamma\|f-g\|$ | §3.4 |
| Fixed point | 不动点 | $V^* = T^* V^*$ | §3.5 |
| Policy evaluation | 策略评估 | $(I-\gamma P^\pi)^{-1}r^\pi$ | §4 |
| Policy improvement | 策略改进 | $\arg\max_a Q(s,a)$ | §5 |
| Value iteration | 值迭代 | $V_{k+1} = T^*V_k$ | §6 |
| Policy iteration | 策略迭代 | 评估+改进循环 | §5 |
| Occupancy measure | 占用测度 | $d^\pi(s,a)$ | §9 |
| Bellman operator | Bellman 算子 | $T^\pi, T^*$ | §3.3 |
