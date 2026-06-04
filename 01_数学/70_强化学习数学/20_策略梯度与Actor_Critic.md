# 策略梯度与 Actor-Critic 理论

> **档位说明**：⭐ = 必学；⭐⭐ = 核心；⭐⭐⭐ = 进阶；⭐⭐⭐⭐ = 研究级
> **前置**：专题 6.1（MDP 与动态规划基础）——Bellman 算子 $T^\pi$/$T^*$ 的 $\gamma$-压缩性（sup-norm 下）、占用测度 $d^\pi(s)=(1-\gamma)\sum_{t=0}^\infty\gamma^t\Pr(s_t=s|\mu_0,\pi)$ 的对偶 LP 形式化、值迭代/策略迭代收敛性。
> **读者定位**：机器人综合交叉方向博士候选，主力语言 C++ / Python，专攻 RL-based motion control、embodied intelligence、SLAM。
> **核心格言**（Bertsekas 2019）：*"Policy iteration is the engine; policy gradient is its differentiable, scalable form."*

---

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题，先回 6.1 复习）

1. 写出 Bellman 最优方程 $V^*(s)$ 的完整形式，并解释 $\gamma$-压缩算子的含义。
2. 什么是占用测度 $d^\pi(s)$？它与策略 $\pi$ 和初始分布 $\mu_0$ 的关系是什么？
3. 策略迭代（PI）的两步交替是什么？为什么 PI 在 tabular 场景下保证单调改进？
4. 对角高斯分布 $\mathcal{N}(\mu, \text{diag}(\sigma^2))$ 的对数概率密度的梯度 $\nabla_\mu \log \pi(a|\mu,\sigma)$ 是什么？
5. 什么是 log-derivative trick（似然比技巧）？写出 $\nabla_\theta \pi_\theta(a|s) = \pi_\theta(a|s) \nabla_\theta \log \pi_\theta(a|s)$ 的推导。

---

## 本章目标

学完本章后，你将能够：

1. **从零推导策略梯度定理**（Sutton 1999），理解 Neumann 级数展开的每一步；
2. **实现 REINFORCE 并分析其方差**，掌握 baseline 减方差的数学原理；
3. **推导 GAE 的完整公式**，理解 $\lambda$ 如何在 bias-variance 之间取舍；
4. **从 TRPO 到 PPO 的完整推导链**，理解 clipped surrogate 的几何含义；
5. **推导 SAC 的最大熵框架**，理解 soft Bellman 算子与重参数化技巧；
6. **在 Isaac Lab 上训练四足策略**，完成从训练到 ONNX 导出的完整流程。

---

## 知识树

```
策略梯度与 Actor-Critic
├── §1 为什么需要策略梯度（动机与 PI 的局限）
├── §2 策略参数化（softmax / 对角高斯 / 神经网络）
├── §3 策略梯度定理完整证明（Sutton 1999）
│   ├── Neumann 级数展开
│   ├── log-derivative trick
│   └── 三种等价形式（Q / Advantage / Occupancy）
├── §4 REINFORCE 与方差分析
│   ├── 无偏性证明
│   ├── 方差爆炸分析
│   └── Baseline 减方差（最优 baseline 推导）
├── §5 广义优势估计 GAE
│   ├── n-step advantage 统一
│   ├── λ-加权推导
│   └── 工程实现（反向递推）
├── §6 Actor-Critic 架构
│   ├── 与 PI 的精确对应
│   ├── Compatible Function Approximation
│   └── 两时间尺度收敛性
├── §7 自然策略梯度 NPG
│   ├── Fisher 信息矩阵与 Riemannian 度量
│   ├── 参数化不变性证明
│   └── 与 Compatible FA 的连接
├── §8 TRPO：单调改进与 trust region
│   ├── Performance Difference Lemma
│   ├── 单调改进不等式
│   └── 共轭梯度法求解
├── §9 PPO：clip 的艺术
│   ├── 从 TRPO 到 clip 的数学动机
│   ├── 几何含义与非对称性
│   └── 工程 tricks（Implementation Matters）
├── §10 确定性策略梯度 DPG/DDPG/TD3
│   ├── DPG 定理（Silver 2014）
│   ├── DDPG 架构与探索
│   └── TD3 三重修正
├── §11 最大熵 RL 与 SAC
│   ├── 最大熵目标与 soft Bellman 算子
│   ├── soft 策略迭代
│   ├── 重参数化技巧
│   └── SAC v1 → v2 演进
└── §12 机器人应用：PPO 主导地位分析
```

---

## §1 引言：为什么策略梯度是现代 RL 的心脏 ⭐

### 动机：策略迭代的三重墙

在 6.1 中我们证明了策略迭代（Policy Iteration, PI）作为 Banach 不动点算法在 tabular 场景下的二次收敛性。然而一旦离开 tabular，PI 就遭遇三重墙：

| 障碍 | 具体问题 | 四足机器人的例子 |
|------|---------|----------------|
| **连续/高维状态-动作空间** | $|\mathcal{S}|$ 和 $|\mathcal{A}|$ 无穷 | ANYmal: $\mathcal{S} \subset \mathbb{R}^{48}$, $\mathcal{A} \subset \mathbb{R}^{12}$ |
| **贪婪改进无闭式解** | $\pi'(s)=\arg\max_a Q^\pi(s,a)$ 在连续 $\mathcal{A}$ 上无法求解 | 12 维连续扭矩输出，无法枚举 |
| **近似+贪婪的复合不稳定** | 值函数近似误差在贪婪选择下被放大 | 致命三元组问题（留待 6.3） |

**如果没有策略梯度，我们会面临什么困难？** 要么限于有限离散动作（Atari-style DQN），要么必须对连续动作做网格离散化（指数爆炸），要么依赖 model-based 方法需要精确的系统模型（机器人接触动力学中几乎不可能精确建模）。

### 策略梯度给出的第三条路

**策略梯度（Policy Gradient, PG）** 不再维护 $V^\pi$ 并从中贪婪地"读"出策略，而是把策略本身写成参数化族 $\pi_\theta$，在参数空间里做梯度上升直接优化期望回报 $J(\theta)$。

> **本质洞察**：策略梯度把"组合优化 over $\mathcal{A}$"替换成"连续优化 over $\Theta$"。这是 RL 可 scale 到机器人的数学基础。

这条路径在数学上有三大优势：

1. **对连续动作空间天然友好**——只需 $\nabla_\theta\log\pi_\theta$ 可计算；
2. **可表达随机策略**——内嵌探索，无需 $\varepsilon$-greedy；
3. **与深度学习 autograd 管线无缝兼容**——PyTorch `.backward()` 直接给出梯度。

**现代几乎所有大规模 RL 成功案例的底层都是策略梯度家族（PPO 占绝对多数）**：OpenAI Five、AlphaStar、Rudin 2022 的四足分钟级训练、Radosavovic 2024 的 Digit 人形上街、ChatGPT 的 RLHF。

> **跨领域类比**：策略梯度之于 RL，如同反向传播之于深度学习——它提供了一个通用的、可微的优化接口。正如反向传播让我们可以端到端训练任意可微计算图，策略梯度让我们可以端到端优化任意可微策略。区别在于：反向传播优化的是监督信号（标签），策略梯度优化的是延迟的、随机的奖励信号。

---

## §2 从值函数到策略参数化 ⭐

### 目标函数的形式化

我们继承 6.1 的 MDP 定义 $\mathcal{M}=(\mathcal{S},\mathcal{A},P,r,\gamma,\mu_0)$。**期望折扣回报**：

$$
J(\pi)=\mathbb{E}_{\tau\sim\pi}\!\left[\sum_{t=0}^{\infty}\gamma^t r(s_t,a_t)\right]
$$

这既是 value-based 方法的目标（通过 $V^\pi(\mu_0)$ 间接优化）也是 policy-based 方法的目标（直接对 $\pi$ 优化）。

### 参数化族的三种典型形式

| 参数化 | 数学形式 | 适用场景 | 工程代表 |
|--------|---------|---------|---------|
| **Tabular-softmax** | $\pi_\theta(a|s)=\frac{\exp\theta_{s,a}}{\sum_{a'}\exp\theta_{s,a'}}$ | 理论分析 | 证明全局收敛 |
| **神经网络-softmax** | $\pi_\theta(a|s)=\text{softmax}(f_\theta(s))_a$ | 离散动作 | Atari, StarCraft |
| **对角高斯** | $\pi_\theta(a|s)=\mathcal{N}(\mu_\theta(s),\,\text{diag}(\sigma_\theta^2(s)))$ | 连续动作 | 四足/人形机器人 |

**对角高斯的工程细节**：在 legged_gym / Isaac Lab 中，$\sigma$ 通常是与状态无关的 learnable vector（"state-independent std"），即 $\sigma$ 不是 $s$ 的函数，而是独立的可训练参数。这是 sim-to-real 稳定性的关键工程选择——如果 $\sigma$ 依赖于状态，在域随机化下 std 可能在某些罕见状态下爆炸。

**参数化的根本动机**：6.1 的 PI 贪婪改进 $\pi'(s)=\arg\max_a Q^\pi(s,a)$ 在连续动作下没有闭式解，而 $\nabla_\theta J(\theta)$ 总可计算（下一节证明）。

> **反事实推理**：如果不做参数化，而是在连续动作空间上直接做贪婪改进，会怎样？
> - 方案 A：网格离散化 $\mathcal{A}$。12 维动作空间即使每维只取 10 个点，就有 $10^{12}$ 个候选动作——完全不可行。
> - 方案 B：用 CEM/CMA-ES 等黑盒优化在每个状态下搜索最优动作。计算量 $O(|\text{population}| \times |\text{rollout}|)$ per state——对实时控制不可行。
> - 策略梯度绕过了这两种困难，把问题转化为对 $\theta$ 的连续优化。

### 轨迹概率与策略梯度的基本思路

一条轨迹 $\tau = (s_0, a_0, r_0, s_1, a_1, r_1, \ldots)$ 的概率为：

$$
p_\theta(\tau) = p(s_0) \prod_{t=0}^{T-1} \pi_\theta(a_t|s_t) \cdot p(s_{t+1}|s_t,a_t)
$$

其中 $p(s_0)$ 是初始状态分布，$p(s_{t+1}|s_t,a_t)$ 是环境转移概率——**两者都不依赖于 $\theta$**。这一观察是策略梯度能够工作的关键：对 $\log p_\theta(\tau)$ 求导时，只有策略项 $\log\pi_\theta(a_t|s_t)$ 有贡献。

---

## §3 策略梯度定理完整证明 ⭐⭐

### §3.1 定理陈述

**定理 6.2.1（Policy Gradient Theorem, Sutton-McAllester-Singh-Mansour, NeurIPS 1999）**

设 $\pi_\theta$ 对 $\theta$ 可微，$J(\theta)=\mathbb{E}_{s_0\sim\mu_0}[V^{\pi_\theta}(s_0)]$。则

$$
\boxed{\;\nabla_\theta J(\theta)=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^{\pi_\theta},\,a\sim\pi_\theta(\cdot|s)}\!\left[\nabla_\theta\log\pi_\theta(a|s)\cdot Q^{\pi_\theta}(s,a)\right]\;}
$$

其中 $d^{\pi}(s)=(1-\gamma)\sum_{t=0}^\infty\gamma^t\Pr(s_t=s\mid\mu_0,\pi)$ 是**归一化折扣状态占用测度**。

> **归一化约定说明**：$d^\pi$ 含 $(1-\gamma)$ 使其和为 1。boxed 公式中的 $\frac{1}{1-\gamma}$ 在实现中被吸收进学习率（梯度方向不变）。on-policy 采样时经验频率自然正比于未归一化的 $\tilde{d}^\pi$。

### §3.2 完整证明（不跳步）⭐⭐

**为什么这个定理如此重要？** 它告诉我们：尽管 $J(\theta)$ 的梯度看似需要对整个轨迹分布求导（包括环境转移概率），但最终结果只涉及 $\nabla_\theta \log \pi_\theta$ 和 $Q^\pi$——前者是策略的 score function（我们可以计算），后者是值函数（我们可以估计）。**环境模型 $P(s'|s,a)$ 完全不出现在梯度公式中**。

**第 1 步：对值函数做梯度展开。**

由 Bellman 方程 $V^\pi(s)=\sum_a\pi(a|s)Q^\pi(s,a)$，对 $\theta$ 求导（注意 $Q^\pi$ 也隐式依赖 $\theta$）：

$$
\nabla_\theta V^\pi(s)=\sum_a\bigl[\nabla_\theta\pi(a|s)\cdot Q^\pi(s,a)+\pi(a|s)\nabla_\theta Q^\pi(s,a)\bigr]
$$

**为什么有两项？** 因为改变 $\theta$ 既改变了"选动作的概率"（第一项），也改变了"选完动作后的长期价值"（第二项，通过后续状态的 $V^\pi$ 递归传递）。

**第 2 步：展开 $\nabla_\theta Q^\pi(s,a)$。**

$Q^\pi(s,a)=r(s,a)+\gamma\sum_{s'}P(s'|s,a)V^\pi(s')$。由于 $r$ 和 $P$ 不依赖 $\theta$：

$$
\nabla_\theta Q^\pi(s,a)=\gamma\sum_{s'}P(s'|s,a)\nabla_\theta V^\pi(s')
$$

**为什么 $r$ 和 $P$ 不依赖 $\theta$？** 因为奖励函数和环境动力学是外部给定的，策略参数只影响"选择哪个动作"，不影响"环境如何响应"。

**第 3 步：代回递推展开（Neumann 级数）。**

令 $\phi(s):=\sum_a\nabla_\theta\pi(a|s)Q^\pi(s,a)$，代入第 1 步：

$$
\nabla_\theta V^\pi(s)=\phi(s)+\gamma\sum_{s'}\underbrace{\Bigl(\sum_a\pi(a|s)P(s'|s,a)\Bigr)}_{P^\pi(s\to s';1)}\nabla_\theta V^\pi(s')
$$

这是一个递推关系！不断展开（与 6.1 中 $V=\sum\gamma^t (P^\pi)^t r$ 的 Neumann 级数技巧同源）：

$$
\nabla_\theta V^\pi(s)=\sum_{k=0}^{\infty}\gamma^k\sum_{s'}P^\pi(s\to s';k)\,\phi(s')
$$

**物理意义**：从 $s$ 出发，经过 $k$ 步到达 $s'$ 的概率（在策略 $\pi$ 下），乘以 $s'$ 处的"即时策略梯度贡献" $\phi(s')$，对所有步数 $k$ 和所有目标状态 $s'$ 求和。

**第 4 步：对初始分布求期望。**

$$
\nabla_\theta J(\theta)=\sum_s\mu_0(s)\nabla_\theta V^\pi(s)=\sum_{s'}\underbrace{\Bigl[\sum_{k=0}^\infty\gamma^k\sum_s\mu_0(s)P^\pi(s\to s';k)\Bigr]}_{\tilde{d}^\pi(s')}\phi(s')
$$

方括号内正是**未归一化**折扣状态占用测度 $\tilde d^\pi(s')$，$d^\pi=(1-\gamma)\tilde d^\pi$。于是：

$$
\nabla_\theta J(\theta)=\frac{1}{1-\gamma}\sum_{s'}d^\pi(s')\phi(s')=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^\pi}\!\sum_a\nabla_\theta\pi(a|s)\,Q^\pi(s,a)
$$

**第 5 步：log-derivative trick（似然比技巧）。**

这是整个推导中最关键的恒等式：

$$
\nabla_\theta\pi(a|s)=\pi(a|s)\cdot\frac{\nabla_\theta\pi(a|s)}{\pi(a|s)}=\pi(a|s)\nabla_\theta\log\pi(a|s)
$$

代入得到最终形式：

$$
\nabla_\theta J(\theta)=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^\pi,a\sim\pi}\!\bigl[\nabla_\theta\log\pi(a|s)\,Q^\pi(s,a)\bigr] \quad\blacksquare
$$

### §3.3 初始分布项为何消失

许多教材在第 3 步跳步。**关键观察**：$\mu_0$ 是外部给定的、不依赖 $\theta$ 的先验。若误把 $\mu_0$ 视作 $\pi_\theta$ 的函数（例如在 meta-RL 中），则出现 "missing initial distribution term"——off-policy PG 需要 importance sampling 修正的根源。

### §3.4 三种等价形式 ⭐

**（A）Q 形式**：$\nabla_\theta J=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^\pi,a\sim\pi}\bigl[\nabla_\theta\log\pi(a|s)\,Q^\pi(s,a)\bigr]$

> **注意**：此处 $d^\pi(s)=(1-\gamma)\sum_{t=0}^\infty\gamma^t P(S_t=s|\pi)$ 是**归一化**的状态占用测度（$\sum_s d^\pi(s)=1$），因此策略梯度前需要 $\frac{1}{1-\gamma}$ 因子。部分文献（如 Sutton-Barto）使用**未归一化**版本 $\tilde d^\pi=\sum\gamma^t P(S_t=s|\pi)$，此时 $\nabla_\theta J=\mathbb{E}_{s\sim\tilde d^\pi,a\sim\pi}[\nabla_\theta\log\pi\,Q^\pi]$（没有前置因子）。本系列统一采用归一化约定。

**（B）Advantage 形式**：令 $A^\pi(s,a):=Q^\pi(s,a)-V^\pi(s)$。由于

$$
\mathbb{E}_{a\sim\pi}[\nabla_\theta\log\pi(a|s)\cdot V^\pi(s)]=V^\pi(s)\cdot\nabla_\theta\sum_a\pi(a|s)=V^\pi(s)\cdot\nabla_\theta 1=0
$$

所以：$\nabla_\theta J=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^\pi,a\sim\pi}\bigl[\nabla_\theta\log\pi(a|s)\,A^\pi(s,a)\bigr]$

**这是所有 PPO/TRPO/A2C 代码里看到的形式。**

**（C）Occupancy 形式**：用占用测度 $\rho^\pi(s,a)=d^\pi(s)\pi(a|s)$：

$$
\nabla_\theta J=\frac{1}{1-\gamma}\int\rho^\pi(s,a)\,A^\pi(s,a)\,\nabla_\theta\log\pi(a|s)\,ds\,da
$$

> **本质洞察**：策略梯度 = 在占用测度 $\rho^\pi$ 加权下的 advantage-weighted log-likelihood gradient。它不是在最大化某个固定的似然函数，而是在最大化"好动作被选中的倾向"。

### ⚠️ 常见陷阱

| 陷阱 | 表现 | 正确做法 |
|------|------|---------|
| 💡 把 $\nabla_\theta \log\pi$ 对 $a\sim\pi_{\text{old}}$ 算 | Off-policy PG 忘 importance sampling | 显式乘 $\rho_t=\pi_\theta/\pi_{\text{old}}$ |
| 🧠 认为 PG 定理需要环境模型 | 试图估计 $P(s'|s,a)$ | 定理已消去 $P$，只需采样 |
| 💡 推导中漏掉 $\nabla_\theta Q^\pi$ 项 | 只写 $\sum_a \nabla_\theta \pi \cdot Q$ | 必须递推展开第二项 |

### 练习

1. 用 Neumann 级数法从头推导 PG 定理，明确写出第 3 步展开的前三项。
2. 证明：Advantage 形式与 Q 形式给出相同的梯度方向。
3. 在 tabular-softmax 下，写出 $\nabla_\theta \log \pi_\theta(a|s)$ 的具体形式。

---

## §4 REINFORCE 与方差分析 ⭐⭐

### §4.1 REINFORCE 算法推导

**REINFORCE（Williams, *Machine Learning* 8, 1992）** 是 PG 定理的最直接 Monte Carlo 实现。

**基本思路**：按 $\pi_\theta$ 采一条轨迹 $\tau=(s_0,a_0,r_0,\ldots,s_T)$，以回报 $\hat G_t=\sum_{k=t}^T\gamma^{k-t}r_k$ 作为 $Q^\pi(s_t,a_t)$ 的无偏估计：

$$
\hat g_{\text{REINFORCE}}=\sum_{t=0}^T\gamma^t\,\nabla_\theta\log\pi_\theta(a_t|s_t)\,\hat G_t
$$

**算法流程**：

1. 用当前策略 $\pi_\theta$ 采集 $N$ 条轨迹 $\{\tau_i\}_{i=1}^N$
2. 对每条轨迹计算回报 $\hat G_t^{(i)} = \sum_{k=t}^{T} \gamma^{k-t} r_k^{(i)}$
3. 估计策略梯度：$\hat g = \frac{1}{N} \sum_{i=1}^N \sum_{t=0}^T \nabla_\theta \log\pi_\theta(a_t^{(i)}|s_t^{(i)}) \hat G_t^{(i)}$
4. 梯度上升：$\theta \leftarrow \theta + \alpha \hat g$

**定理 6.2.2（REINFORCE 无偏性）** $\mathbb{E}_{\tau\sim\pi_\theta}[\hat g_{\text{REINFORCE}}]=\nabla_\theta J(\theta)$。

**证明**：$\hat G_t$ 是 $Q^\pi(s_t,a_t)$ 的无偏估计（Monte Carlo 的基本性质），代入 PG 定理即得。

### §4.2 因果性改进：Reward-to-go

原始 REINFORCE 中，时刻 $t$ 的梯度被整条轨迹的回报加权——但**未来的动作不应该考虑过去的奖励**。这是一个非因果项。

**改进**：将 $\hat G_t$ 替换为 reward-to-go $\hat R_t = \sum_{k=t}^T \gamma^{k-t} r_k$：

$$
\hat g = \frac{1}{N}\sum_{i=1}^N \sum_{t=0}^T \nabla_\theta\log\pi_\theta(a_t^{(i)}|s_t^{(i)}) \hat R_t^{(i)}
$$

**为什么仍然无偏？** 去掉的项 $\sum_{k=0}^{t-1} \gamma^k r_k$ 与 $\nabla_\theta \log\pi_\theta(a_t|s_t)$ 在条件于 $s_t$ 下独立（马尔可夫性），其期望为零。等价地，$\text{Var}(A+B) = \text{Var}(A) + \text{Var}(B)$ 当 $A,B$ 独立，去掉无关项必然降低方差。

### §4.3 方差爆炸问题

**致命缺陷**：$\hat G_t$ 的方差随 horizon $T$ 增长。直观理解：

$$
\text{Var}(\hat G_t) = \text{Var}\left(\sum_{k=t}^T \gamma^{k-t} r_k\right) \approx \sum_{k=t}^T \gamma^{2(k-t)} \text{Var}(r_k)
$$

当 $T$ 很大时，这个和可以很大。在 MuJoCo `Humanoid-v4` 上，REINFORCE 可能需要 $10^8$ 步才能达到 PPO 两小时内的水平。

> **反事实推理**：如果我们不降低方差会怎样？
> - 学习率必须设得极小（否则梯度噪声会让参数震荡）
> - 小学习率导致收敛极慢
> - 在实际机器人任务中（数百维状态/动作，数千步 horizon），REINFORCE 几乎不可用

### §4.4 Baseline 减方差 ⭐⭐

**核心思想**：引入任意与动作 $a$ 无关的函数 $b(s)$：

$$
\hat g=\sum_t\nabla_\theta\log\pi(a_t|s_t)\bigl[\hat G_t-b(s_t)\bigr]
$$

**引理 6.2.3（Baseline 不引入偏差）**：

$$
\mathbb{E}_{a\sim\pi}[\nabla_\theta\log\pi(a|s)\,b(s)]=b(s)\sum_a\nabla_\theta\pi(a|s)=b(s)\cdot\nabla_\theta\underbrace{\sum_a\pi(a|s)}_{=1}=0
$$

**为什么 $b$ 必须只依赖于 $s$ 而不能依赖于 $a$？** 如果 $b=b(s,a)$，则 $\sum_a \nabla_\theta \pi(a|s) \cdot b(s,a)$ 一般不为零——$b(s,a)$ 无法提到求和号外面，会引入偏差。

**最优 baseline 推导**：设 $g(\tau) = \nabla_\theta \log p_\theta(\tau)$，我们要最小化方差：

$$
\text{Var} = \mathbb{E}[g(\tau)^2(r(\tau)-b)^2] - (\mathbb{E}[g(\tau)(r(\tau)-b)])^2
$$

第二项不依赖于 $b$（因为 $\mathbb{E}[g \cdot b] = 0$）。对第一项关于 $b$ 求导令其为零：

$$
\frac{d}{db}\mathbb{E}[g^2(r-b)^2] = -2\mathbb{E}[g^2 r] + 2b\mathbb{E}[g^2] = 0
$$

解得：

$$
b^*(s) = \frac{\mathbb{E}[g(\tau)^2 \cdot r(\tau)]}{\mathbb{E}[g(\tau)^2]}
$$

**实践中**：几乎总用 $b(s)=V^\pi(s)$（次优但简单，且使 $\hat G_t - V^\pi(s_t) \approx \hat A^\pi(s_t,a_t)$，语义清晰为"advantage"）。

> **跨领域类比**：Baseline 之于策略梯度，如同 control variate 之于蒙特卡罗积分。在金融衍生品定价中，control variate 通过减去一个已知期望的随机变量来降低估计方差——完全相同的数学原理。

### ⚠️ 常见陷阱

| 陷阱类型 | 错误描述 | 后果 | 正确做法 |
|---------|---------|------|---------|
| 概念误区 | 认为 baseline 会改变梯度方向 | 怀疑代码有 bug | baseline 只改方差不改期望 |
| 编程陷阱 | baseline 使用了依赖 $a$ 的函数 | 引入偏差，策略偏离最优 | 只用 $V(s)$ 或常数 |
| 思维陷阱 | 认为方差越小越好 | 使 baseline = reward 导致梯度消失 | 方差为零意味着梯度恒为零 |

### 练习

1. 在一维高斯策略 $\pi_\theta(a) = \mathcal{N}(\theta, 1)$，$r(a)=-a^2$ 下，计算 REINFORCE 梯度的闭式方差。当使用 $b=V^\pi=\mathbb{E}[r]$ 时方差降低多少？
2. 证明 reward-to-go 的改进确实降低方差（提示：利用独立随机变量方差可加性）。
3. 手写一个 50 行 Python 的 REINFORCE + baseline on CartPole-v1，观察带/不带 baseline 的学习曲线差异。

---

## §5 Advantage 函数与广义优势估计 GAE ⭐⭐

### §5.0 Advantage 函数的数学性质

**定义**：$A^\pi(s,a) := Q^\pi(s,a) - V^\pi(s)$

**直觉含义**："在状态 $s$ 下执行动作 $a$ 比'平均表现'好多少。" $V^\pi(s)$ 是基准线（平均表现），$Q^\pi(s,a)$ 是具体动作的价值。

**关键性质**：

1. **期望为零**：$\mathbb{E}_{a\sim\pi}[A^\pi(s,a)] = \mathbb{E}_{a\sim\pi}[Q^\pi(s,a)] - V^\pi(s) = V^\pi(s) - V^\pi(s) = 0$

   这意味着好动作的 $A>0$ 一定被坏动作的 $A<0$ 平衡。策略梯度用 $A$ 加权时，好动作概率增加、坏动作概率减少——实现**相对评价**。

2. **与策略改进的关系**：Policy Improvement Theorem 可以用 $A$ 重写为：
   $$J(\pi') - J(\pi) = \frac{1}{1-\gamma}\mathbb{E}_{s\sim d^{\pi'}, a\sim\pi'}[A^\pi(s,a)]$$
   只要新策略在每个状态下的 advantage 期望非负，就保证改进。

3. **方差优于 $Q$**：用 $A$ 代替 $Q$ 做 PG 权重，减去了 $V(s)$ 这个与 $a$ 无关的部分，降低了梯度估计的方差。

**为什么 Advantage 是"正确的"梯度权重？**

回顾 PG 定理：$\nabla_\theta J = \mathbb{E}[\nabla_\theta\log\pi \cdot Q^\pi]$。$Q$ 中包含了"在状态 $s$ 下所有动作的平均贡献" $V(s)$，这部分对所有动作一视同仁、不提供区分信息。减去它得到 $A$，只保留"这个动作**相对**于平均的贡献"——信噪比更高。

> **反事实推理**：如果我们不用 Advantage 而直接用 $Q$ 会怎样？
> 在高回报环境中（如所有状态的 $V(s)>100$），所有 $Q$ 值都很大正数。即使坏动作的 $Q=101$、好动作的 $Q=103$，REINFORCE 也会增加**所有**动作的概率（因为权重都是正的）。只是好动作增加得稍快一点——但这种微小差异需要大量样本才能统计性地体现。用 $A$：坏动作 $A=-1$（减概率），好动作 $A=+1$（增概率）——信号清晰。

### §5.1 动机：bias-variance 困境

| 估计方式 | 偏差 | 方差 | 特点 |
|---------|------|------|------|
| Monte Carlo $\hat A_t = \hat G_t - V(s_t)$ | 零（若 $V$ 精确） | 大（包含所有未来随机性） | 必须等回合结束 |
| TD 残差 $\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$ | 有偏（$V$ 不准） | 小（只涉及一步） | 可在线更新 |
| $n$-step $\hat A_t^{(n)} = \sum_{l=0}^{n-1}\gamma^l r_{t+l} + \gamma^n V(s_{t+n}) - V(s_t)$ | 中等 | 中等 | 需要选 $n$ |

**问题**：如何在 MC（无偏高方差）和 TD（有偏低方差）之间找到最优折中？

### §5.2 GAE 的完整推导 ⭐⭐

**GAE（Schulman-Moritz-Levine-Jordan-Abbeel, ICLR 2016, arXiv:1506.02438）** 的核心思想：对所有 $n$-step 估计做**指数加权平均**。

**Step 1**：$k$-step advantage 可以展开为 TD 残差之和。

定义 $\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$，则：

$$
\hat A_t^{(1)} = \delta_t
$$
$$
\hat A_t^{(2)} = \delta_t + \gamma\delta_{t+1}
$$

一般地（用归纳法容易证明）：

$$
\hat A_t^{(k)} = \sum_{l=0}^{k-1} \gamma^l \delta_{t+l}
$$

**证明 2-step 情况**：
$$
\hat A_t^{(2)} = r_t + \gamma r_{t+1} + \gamma^2 V(s_{t+2}) - V(s_t)
$$
$$
= [r_t + \gamma V(s_{t+1}) - V(s_t)] + \gamma[r_{t+1} + \gamma V(s_{t+2}) - V(s_{t+1})]
$$
$$
= \delta_t + \gamma\delta_{t+1} \quad\checkmark
$$

**Step 2**：定义 GAE 为 $\lambda$-加权几何平均：

$$
\hat A_t^{\text{GAE}(\gamma,\lambda)} := (1-\lambda)\sum_{n=1}^{\infty}\lambda^{n-1}\hat A_t^{(n)}
$$

权重 $(1-\lambda)\lambda^{n-1}$ 是归一化的几何分布（和为 1）。

**Step 3**：展开求和。

$$
(1-\lambda)\sum_{n=1}^{\infty}\lambda^{n-1}\hat A_t^{(n)} = (1-\lambda)\sum_{n=1}^{\infty}\lambda^{n-1}\sum_{l=0}^{n-1}\gamma^l\delta_{t+l}
$$

交换求和顺序（$l$ 从 0 到 $\infty$，$n$ 从 $l+1$ 到 $\infty$）：

$$
= (1-\lambda)\sum_{l=0}^{\infty}\gamma^l\delta_{t+l}\sum_{n=l+1}^{\infty}\lambda^{n-1}
= (1-\lambda)\sum_{l=0}^{\infty}\gamma^l\delta_{t+l}\cdot\frac{\lambda^l}{1-\lambda}
$$

$$
\boxed{\hat A_t^{\text{GAE}(\gamma,\lambda)} = \sum_{l=0}^{\infty}(\gamma\lambda)^l\delta_{t+l}}
$$

### §5.3 两个极端

| $\lambda$ | GAE 变成 | 偏差 | 方差 | 等价算法 |
|-----------|---------|------|------|---------|
| $\lambda=0$ | $\hat A_t = \delta_t$ | 高（依赖 $V$ 精度） | 低 | TD(0) |
| $\lambda=1$ | $\hat A_t = \hat G_t - V(s_t)$ | 零 | 高 | Monte Carlo |

**工业实践**：legged_gym / Isaac Lab 默认 $\lambda=0.95$，stable-baselines3 默认 $\lambda=0.95$。

> **反事实推理**：如果使用 $\lambda=1$（纯 MC），在足式 RL 中会怎样？
> 轨迹长度 24 步（Rudin 2022 配置），方差还可接受。但如果 horizon=1000（某些操纵任务），MC 的方差会使训练极其不稳定。$\lambda=0.95$ 在 horizon=24 时几乎等同 MC（因为 $0.95^{24} \approx 0.29$ 衰减快），但在长 horizon 时自动截断远期噪声。

### §5.3.5 GAE 的偏差分析

**偏差来源**：GAE 的偏差完全来自 value function 近似误差 $\epsilon_t := V_\phi(s_t) - V^\pi(s_t)$。

定义 $\delta_t^V = r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)$（使用近似 $V$）和 $\delta_t^{V^\pi} = r_t + \gamma V^\pi(s_{t+1}) - V^\pi(s_t)$（使用真实 $V$），则：

$$
\delta_t^V = \delta_t^{V^\pi} + \gamma\epsilon_{t+1} - \epsilon_t
$$

代入 GAE：

$$
\hat A_t^{\text{GAE}} = \sum_l (\gamma\lambda)^l \delta_{t+l}^V = \sum_l (\gamma\lambda)^l [\delta_{t+l}^{V^\pi} + \gamma\epsilon_{t+l+1} - \epsilon_{t+l}]
$$

当 $\lambda < 1$ 时，误差项 $\epsilon$ 的贡献被 $\lambda$ 衰减——**这正是 $\lambda<1$ 能减少偏差敏感度的原因**。

**实际含义**：
- $\lambda=1$（MC）：偏差为 $\gamma\epsilon_T - \epsilon_t$，只在 trajectory 端点有误差
- $\lambda=0$（TD(0)）：偏差为 $\gamma\epsilon_{t+1} - \epsilon_t$，每步都受 $V$ 误差影响
- $\lambda=0.95$：折中，远期 $V$ 误差被衰减

**工程启示**：如果 Critic 训练充分（$\epsilon \approx 0$），用 $\lambda=0$ 就够了（方差最小）。如果 Critic 还不准，用大 $\lambda$ 减少对 Critic 的依赖——代价是方差增大。

### §5.4 GAE 的正确反向实现

```python
# GAE 反向递推实现（CleanRL ppo_continuous_action.py 核心 5 行）
# 为什么反向？因为 A_t 依赖于 A_{t+1}（通过 delta_{t+1}）
lastgaelam = 0
for t in reversed(range(T)):
    # 处理 episode 边界：done=1 时 bootstrap 截断
    nextvalue = values[t+1] if t+1 < T else next_value_bootstrap
    delta = rewards[t] + gamma * nextvalue * (1 - dones[t]) - values[t]
    # 递推公式：A_t = delta_t + gamma*lambda*(1-done_t)*A_{t+1}
    advantages[t] = lastgaelam = delta + gamma * lam * (1 - dones[t]) * lastgaelam
```

> ⚠️ **编程陷阱**：GAE 在 episode 边界处不 mask `done`，会导致 bootstrap 跨 episode，梯度污染。必须 `(1-done[t])` 截断。

### 练习

1. 推导 GAE 恒等式，验证 $\lambda=0$ 和 $\lambda=1$ 两个极端。
2. 在 horizon=24 的任务中，比较 $\lambda=0.9, 0.95, 0.99$ 对训练曲线的影响（提示：用 CleanRL 实验）。
3. 证明：GAE 的偏差 $\text{Bias}(\hat A_t^{\text{GAE}}) = O((1-\lambda)\gamma)$ 当 $V$ 的近似误差有界时。
4. **（跨章综合）** 结合 6.1 的 Bellman 期望算子 $T^\pi$ 和本节的 GAE 公式，证明：当 Critic 精确（$V_\phi = V^\pi$）时，GAE 对任意 $\lambda$ 都是 $A^\pi(s_t,a_t)$ 的无偏估计。提示：利用 $\mathbb{E}[\delta_t|s_t,a_t] = A^\pi(s_t,a_t)$（这正是 Bellman 期望方程的逐点形式）。
5. 在 GAE 的反向实现中，如果忘记 `(1-done)` mask 会产生什么现象？从 bootstrap 跨 episode 传播的角度给出数学解释。

---

## §5.5 GAE 工程总结

上一节从数学上完成了 GAE 的推导与实现。本节把注意力转向工程调参——在不同任务配置下如何选择 $\lambda$ 和相关超参数，以及 GAE 与 6.3 中 TD(λ) 的精确对应关系。

> **跨章桥接**：回顾 6.3 中的 TD(λ)：$G_t^\lambda = (1-\lambda)\sum_{n=1}^\infty \lambda^{n-1}G_t^{(n)}$。GAE 本质上就是 TD(λ) 应用到 advantage 估计——把"估计 $V^\pi$"改为"估计 $A^\pi$"。在 6.3 中我们知道 TD(λ) 收敛到投影 Bellman 不动点，误差界含放大因子 $1/\sqrt{1-\gamma^2}$；GAE 的偏差同样受 Critic 精度制约——这就是为什么 §5.3.5 中分析偏差时需要引入 $\epsilon_t = V_\phi(s_t) - V^\pi(s_t)$。

### 超参数选择指南

| 超参数 | 推荐范围 | 选择逻辑 |
|--------|---------|---------|
| $\lambda$ | 0.9-0.99 | Critic 准 → 小 $\lambda$；Critic 不准 → 大 $\lambda$ |
| $\gamma$ | 0.99-0.999 | 长 horizon 任务 → 大 $\gamma$ |
| horizon $T$ | 16-2048 | GPU 内存决定上限；太短截断偏差大 |
| 归一化 | 开 | per-minibatch normalize advantages |

> **思维陷阱**：初学者常认为"$\lambda$ 越大越好（因为偏差小）"。但在训练初期 Critic 非常不准时，大 $\lambda$ 的低偏差优势被 Critic 误差抵消，反而不如小 $\lambda$。正确思维：$\lambda$ 应该随 Critic 的准确度**动态调整**——虽然实践中通常固定为 0.95。

---

## §6 Actor-Critic 架构 ⭐⭐

### §6.1 核心思想与 PI 的精确对应

**从 REINFORCE 到 Actor-Critic 的演进动机**：

REINFORCE 用 Monte Carlo $\hat G_t$ 估计 advantage，有两个严重问题：

1. **方差极大**：$\hat G_t$ 包含从 $t$ 到 $T$ 所有步的随机性
2. **必须等回合结束**：无法在线更新

**核心思想**：如果我们有一个函数 $V_\phi(s)$ 能近似 $V^\pi(s)$，就可以用 **TD 误差** $\delta_t = r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)$ 替代 $\hat G_t - V(s_t)$。TD 误差只涉及一步真实奖励 + 一步 bootstrap，方差远小于完整回报。

这就引出了 Actor-Critic 的双网络架构：

- **Actor** $\pi_\theta(a|s)$：负责决策，输出动作分布
- **Critic** $V_\phi(s)$：负责评价，预测状态价值

**Actor 的更新逻辑**："Critic 告诉我刚才那一步比平均水平好（$\hat A > 0$），我就增加那个动作的概率；反之减小。"

**Critic 的更新逻辑**："看你在 $s_t$ 拿到了 $r_t$ 并且跑到了 $s_{t+1}$，我发现我之前对 $s_t$ 的估值偏低了，我要修正参数。"

**训练流程**：
1. 从当前策略 $\pi_\theta$ 采样 $(s_t, a_t, r_t, s_{t+1})$
2. 计算 TD 误差 $\delta_t = r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)$
3. 更新 Critic：$\phi \leftarrow \phi - \alpha_\phi \nabla_\phi (V_\phi(s_t) - [r_t + \gamma V_\phi(s_{t+1})])^2$
4. 更新 Actor：$\theta \leftarrow \theta + \alpha_\theta \nabla_\theta \log\pi_\theta(a_t|s_t) \cdot \delta_t$

**为什么 Critic 学习 $V(s)$ 而不是 $Q(s,a)$？** 因为 $V$ 只需要状态作为输入（参数量更少、更容易拟合），并且 baseline 的数学要求是"只依赖于 $s$"。当然，在 off-policy 算法（如 SAC、TD3）中，Critic 学习 $Q(s,a)$ 是因为需要对任意 $(s,a)$ 对评估价值。

**n-step bootstrapping**：TD(0) 偏差大、方差小；MC 无偏、方差大。$n$-step 折中：

$$
\hat A_t^{(n)} = \sum_{k=0}^{n-1}\gamma^k r_{t+k} + \gamma^n V_\phi(s_{t+n}) - V_\phi(s_t)
$$

Mnih 2016 (A3C) 推荐 $n=5$。但 GAE 提供了更优雅的解决方案（§5 已推导）。

Actor-Critic 把 $Q^\pi$ 或 $A^\pi$ 也用一个参数化 **critic** $V_\phi$ 估计。

| 经典 PI (6.1) | Actor-Critic |
|---|---|
| Policy evaluation: 精确求 $V^{\pi_k}$ | Critic 更新 $\phi$ 使 $V_\phi \approx V^{\pi_\theta}$ |
| Policy improvement: $\pi_{k+1}=\arg\max_a Q^{\pi_k}$ | Actor 更新 $\theta \leftarrow \theta + \alpha\nabla_\theta J$ |
| 两步交替（完整求解） | 两步同时（sample-level 交替） |

> **跨领域类比**：Actor-Critic 类似于 GAN 的 Generator-Discriminator 交互——Actor 生成动作，Critic 评价动作质量。两者通过对抗/协作不断改进。区别在于：GAN 的 Discriminator 试图区分真假，Critic 试图预测长期价值。

### §6.2 Compatible Function Approximation ⭐⭐⭐

**问题**：若 critic 用有限参数族 $Q_\psi$，用 $Q_\psi$ 替代 $Q^\pi$ 的"伪 PG"是否还等于真 PG？

**定理 6.2.4（Compatible Function Approximation, Sutton 1999）**

若 critic 满足：
1. **兼容性**：$\nabla_\psi Q_\psi(s,a)=\nabla_\theta\log\pi_\theta(a|s)$
2. **最小化均方误差**：$\psi$ 在 $\mathbb{E}_{d^\pi,\pi}[(Q^\pi-Q_\psi)^2]$ 上达到最小

则 actor-critic 梯度无偏。

**证明**：设 $\psi^*$ 最优，一阶条件 $\mathbb{E}[\nabla_\psi Q_\psi\cdot(Q^\pi-Q_\psi)]=0$。将条件 1 代入：

$$
\mathbb{E}[\nabla_\theta\log\pi\cdot(Q^\pi-Q_{\psi^*})]=0 \implies \mathbb{E}[\nabla_\theta\log\pi\cdot Q^\pi]=\mathbb{E}[\nabla_\theta\log\pi\cdot Q_{\psi^*}] \quad\blacksquare
$$

**推论**：若 critic 取线性形式 $Q_\psi(s,a)=\psi^\top\nabla_\theta\log\pi_\theta(a|s)$（critic 特征 = policy 的 score function），则自动满足条件 1。

> **机器人直觉**：工程实现中 critic 通常是独立 MLP，并**不**满足 compatibility——所以实际 actor-critic 是有偏的，但偏差在 critic 训练充分后较小。

### §6.3 两时间尺度分析 ⭐⭐⭐

Actor-Critic 是典型的**两时间尺度随机逼近系统**（Borkar 2008）：

- **Critic（快时间尺度）**：学习率 $\alpha_\phi$ 较大，快速跟踪当前策略的值函数
- **Actor（慢时间尺度）**：学习率 $\alpha_\theta$ 较小，缓慢改进策略

**收敛条件**（Borkar-Meyn 2000 定理的简化表述）：
1. $\alpha_\theta / \alpha_\phi \to 0$（actor 比 critic 慢无穷多）
2. 标准 Robbins-Monro 条件：$\sum \alpha = \infty$，$\sum \alpha^2 < \infty$
3. Critic 在固定策略下有唯一不动点

满足这些条件时，系统收敛到 actor 的局部最优 + critic 的全局最优（给定当前策略）。

> **与 6.5 随机逼近的桥梁**：这正是 6.5 专题将深入讨论的 ODE method。核心思想：当快变量"几乎收敛"时，慢变量看到的是一个"平均化"后的梯度场——两时间尺度系统可以逐层分析。

### ⚠️ 常见陷阱

| 陷阱 | 表现 | 根本原因 | 正确做法 |
|------|------|---------|---------|
| Actor 和 Critic 用相同学习率 | 训练震荡 | 违反两时间尺度分离 | Critic lr 通常 3-10x Actor lr |
| Critic target 不用 stop_gradient | 梯度回传污染 Actor | TD target 应视为常数 | `td_target.detach()` |
| Critic 网络太小 | Advantage 估计偏差大 | Underfitting | Critic 通常比 Actor 大 |

---

## §7 自然策略梯度 NPG ⭐⭐⭐

### §7.1 动机：参数空间 vs 分布空间

**为什么普通梯度不够好？**

考虑一维 Gaussian $\pi_\theta(a)=\mathcal N(\theta, 0.01^2)$。若 $\theta$ 变 $\Delta\theta=0.1$，策略分布**剧变**（KL 很大，因为 $\sigma$ 很小）。若 $\sigma=100$，同样的 $\Delta\theta$ 策略分布几乎不变。

**问题**：普通梯度 $\nabla_\theta J$ 对参数化方式敏感——同一策略，不同参数化梯度方向不同。

**具体例子**：设策略为 $\pi_\theta(a|s) = \mathcal{N}(\theta_1, e^{\theta_2})$（用 $\theta_2$ 参数化 log-std）。如果我们改为 $\tilde\pi_\phi(a|s) = \mathcal{N}(\phi_1, \phi_2^2)$（直接参数化 std），那么 $\nabla_\theta J \neq J_g^\top \nabla_\phi J$（其中 $J_g$ 是参数变换的雅可比）——因为普通梯度不是张量（不遵守坐标变换规则）。

**这在优化问题中造成什么后果？** 梯度的方向取决于我们"碰巧"选择了什么参数化方式。但策略空间中"最优改进方向"应该是内禀的、与参数化无关的。这就像在弯曲流形上做优化时，欧氏梯度指向的方向可能根本不是流形上的最速下降方向。

> **反事实推理**：如果我们直接用 $\nabla_\theta J$ 做梯度上升会怎样？
> 在 $\sigma$ 很小的维度上，微小的 $\Delta\theta$ 就导致 KL 爆炸，策略跳太远；在 $\sigma$ 很大的维度上，大步 $\Delta\theta$ 几乎不改变策略。结果：参数空间的"等步长"在策略空间是"不等步长"——某些方向走太远，某些方向走太慢。

**历史背景**：这个问题的解决来自 Amari (1998) 的**信息几何**理论。Amari 证明了统计流形（概率分布族构成的空间）上的自然度量是 Fisher 信息矩阵。Kakade (2001) 将这一思想引入 RL，提出了自然策略梯度。

> **跨领域类比**：自然梯度之于策略空间，如同 Newton 法之于欧氏空间。Newton 法用 Hessian 矩阵做预条件，补偿目标函数的曲率；自然梯度用 Fisher 信息矩阵做预条件，补偿策略分布空间的曲率。但两者有本质区别：Hessian 依赖于目标函数（不同目标有不同 Hessian），Fisher 矩阵只依赖于策略分布本身（是"空间的性质"而非"函数的性质"）。

### §7.2 Fisher 信息矩阵作为 Riemannian 度量

**定义**：

$$
F(\theta):=\mathbb{E}_{s\sim d^\pi,a\sim\pi}\!\bigl[\nabla_\theta\log\pi_\theta(a|s)\,\nabla_\theta\log\pi_\theta(a|s)^\top\bigr]
$$

**关键事实**（Amari 信息几何）：$F$ 是 KL 散度 Hessian 的一阶近似：

$$
D_{\mathrm{KL}}(\pi_\theta\|\pi_{\theta+\Delta\theta})\approx\tfrac{1}{2}\Delta\theta^\top F(\theta)\Delta\theta
$$

故 $F$ 定义了**策略分布空间的局部 Riemannian 度量**。在此度量下的 steepest ascent 方向是 $F^{-1}\nabla_\theta J$。

### §7.3 自然梯度定理

**定理 6.2.5（Kakade 2001, NeurIPS）**

自然策略梯度 $\tilde\nabla J(\theta):=F(\theta)^{-1}\nabla_\theta J(\theta)$ 是对参数化不变（covariant）的梯度方向。

**证明**：定义 trust-region 问题

$$
\max_{\Delta\theta}\nabla J^\top\Delta\theta\quad\text{s.t.}\quad D_{\mathrm{KL}}(\pi_\theta\|\pi_{\theta+\Delta\theta})\le\varepsilon
$$

二阶 KL 近似 + 拉格朗日乘子法：$\Delta\theta^* \propto F^{-1}\nabla J$。

对参数化不变性：设 $\theta=g(\phi)$，雅可比 $J_g=\partial\theta/\partial\phi$。则 $F_\phi=J_g^\top F_\theta J_g$，$\nabla_\phi J=J_g^\top\nabla_\theta J$，故 $F_\phi^{-1}\nabla_\phi J=J_g^{-1}F_\theta^{-1}\nabla_\theta J$——方向在策略空间不变。$\blacksquare$

### §7.3.5 Fisher 信息矩阵的具体计算

**对角高斯策略**：$\pi_\theta(a|s) = \mathcal{N}(\mu_\theta(s), \sigma^2 I)$

score function：$\nabla_\theta\log\pi = \frac{1}{\sigma^2}\nabla_\theta\mu_\theta(s)\cdot(a-\mu_\theta(s))$

Fisher 矩阵：$F = \frac{1}{\sigma^2}\mathbb{E}_{s\sim d^\pi}[\nabla_\theta\mu(s)\nabla_\theta\mu(s)^\top]$

**物理意义**：$\sigma$ 越小（策略越确定），Fisher 矩阵越大（策略空间"弯曲"越严重），自然梯度步长 $F^{-1}g$ 越小——自动适应策略的"精度"。

**对 softmax 策略**：$\pi_\theta(a|s) = \text{softmax}(\theta_{s,a})$

Fisher 矩阵的 $(i,j)$ 分量：$F_{ij} = \mathbb{E}_{s\sim d^\pi}[\pi_i(1_{i=j} - \pi_j)]$，其中 $\pi_i = \pi(a_i|s)$。这是对角占优矩阵——可以高效求逆。

### §7.4 与 Compatible FA 的惊人连接

若 critic 取兼容线性形式 $Q_\psi=\psi^\top\nabla_\theta\log\pi$，最小二乘解 $\psi^*=F^{-1}\mathbb{E}[\nabla\log\pi\cdot Q^\pi]=F^{-1}\nabla J=\tilde\nabla J$。

> **本质洞察**：最优兼容 critic 的参数本身就是自然梯度。这是 Kakade 2001 的神来之笔——看似独立的两个概念（critic 参数和自然梯度）在数学上完全等价。

### §7.5 NPG 的实现与 ACKTR

**NPG 的计算瓶颈**：Fisher 矩阵 $F \in \mathbb{R}^{|\theta|\times|\theta|}$，对于百万参数的神经网络，显式存储 $F$ 不可行。

**ACKTR（Wu et al., NeurIPS 2017）**：使用 Kronecker-factored approximate curvature (K-FAC) 近似 Fisher 矩阵。K-FAC 将 $F$ 分解为每层的 Kronecker 积，计算量从 $O(|\theta|^2)$ 降到 $O(|\theta|)$。

**NPG 的历史贡献**：虽然 NPG 本身（因为需要精确 $F^{-1}$）在实践中不如 PPO，但它提供了理解 TRPO/PPO "为什么稳定" 的理论基础——它们都是在策略分布空间（而非参数空间）做有约束的优化。

### ⚠️ NPG/TRPO 常见陷阱

| 陷阱 | 表现 | 正确做法 |
|------|------|---------|
| CG 迭代次数不够 | $F^{-1}g$ 不准，步方向错误 | 至少 10 次 CG 迭代 |
| 忘记 damping | Fisher 矩阵奇异，CG 不收敛 | 加 $\lambda I$ damping |
| Line search 步骤太少 | 步长过大破坏 KL 约束 | 至少 10 次 backtrack |

---

## §8 TRPO：单调改进与 trust region ⭐⭐⭐

### §8.1 动机：从 NPG 到 TRPO

NPG 给出了正确的方向 $F^{-1}\nabla J$，但**步长**怎么选？太大会破坏 surrogate 近似的有效性，太小则浪费计算。TRPO 的回答：**用严格的单调改进不等式来确定最大安全步长**。

**TRPO 要解决的核心问题**：在实际的 RL 中，我们只能用有限样本估计梯度。如果步长太大：
1. surrogate objective $L_\pi(\pi')$ 与真实改进 $J(\pi')-J(\pi)$ 的差距变大
2. 新策略的占用测度 $d^{\pi'}$ 与旧策略的 $d^\pi$ 偏离过远
3. 下一轮采样的数据可能完全"跟不上"策略变化

**TRPO 的历史地位**：它是第一个为 deep RL 提供**理论单调改进保证**的算法（虽然实践中用的是放松版约束）。在 TRPO 之前，deep RL 的策略更新本质上是"试错"——步长是超参数，没有理论指导。TRPO 把"步长选择"转化为"trust region 约束优化"，给出了有数学意义的步长上界。

**TRPO 与经典优化的联系**：TRPO 的 trust-region 子问题在结构上是 SQP（Sequential Quadratic Programming）的一次迭代——目标线性化 + 约束二次化。这让 TRPO 继承了 SQP 的局部收敛性质。

### §8.2 Performance Difference Lemma

**精确恒等式**（Kakade & Langford 2002）：

$$
J(\pi')-J(\pi)=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^{\pi'},a\sim\pi'}[A^\pi(s,a)]
$$

**障碍**：右边用的是 $d^{\pi'}$（新策略的占用测度），更新前拿不到。

### §8.3 Surrogate 目标

用旧占用测度近似：

$$
L_\pi(\pi'):=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^\pi,a\sim\pi'}[A^\pi(s,a)]=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^\pi,a\sim\pi}\!\left[\frac{\pi'(a|s)}{\pi(a|s)}A^\pi(s,a)\right]
$$

### §8.4 单调改进不等式

**定理 6.2.6（Monotonic Improvement Theorem, Schulman 2015）**

$$
J(\pi')\ge L_\pi(\pi')-\frac{4\varepsilon\gamma}{(1-\gamma)^2}\,D_{\mathrm{KL}}^{\max}(\pi,\pi')
$$

其中 $\varepsilon=\max_{s,a}|A^\pi(s,a)|$，$D_{\mathrm{KL}}^{\max}:=\max_s D_{\mathrm{KL}}(\pi(\cdot|s)\|\pi'(\cdot|s))$。

**证明骨架**（TRPO 原文 Appendix A 的简化）：

Step 1：从 Performance Difference Lemma 出发：$J(\pi') - J(\pi) = \frac{1}{1-\gamma}\mathbb{E}_{d^{\pi'}, \pi'}[A^\pi]$

Step 2：将 $d^{\pi'}$ 替换为 $d^\pi$ 引入误差。关键 bound：

$$
|d^{\pi'}(s) - d^\pi(s)| \le \frac{2\gamma}{(1-\gamma)} \max_s D_{\text{TV}}(\pi'(\cdot|s) \| \pi(\cdot|s))
$$

Step 3：Pinsker 不等式：$D_{\text{TV}}(p\|q) \le \sqrt{D_{\text{KL}}(p\|q)/2}$

Step 4：合并，利用 $|A^\pi| \le \varepsilon$ 和 Cauchy-Schwarz，得到最终 bound。

**$\frac{1}{(1-\gamma)^2}$ 因子从何而来？** 两个 $\frac{1}{1-\gamma}$ 的乘积：
- 第一个来自占用测度的定义（无穷步求和）
- 第二个来自 $|d^{\pi'}-d^\pi|_{\text{TV}}$ 的 bound（策略差异在时间轴上的几何级数累积）

**推论（单调改进保证）**：若每次迭代都严格最大化 $L_\pi(\pi') - C\cdot D_{\text{KL}}^{\max}$，则 $J$ 严格单调不减——**RL 中少数几个全局单调保证之一**，与 PI 的 policy improvement theorem 同级。

**从 $D_{\text{KL}}^{\max}$ 到 $\bar D_{\text{KL}}$**：实践中 $\max_s$ 不可微且难估计。TRPO 改用期望 KL $\bar D_{\text{KL}}=\mathbb{E}_{s\sim d^\pi}[D_{\text{KL}}]$。这失去严格单调保证，但实验效果接近。

### §8.5 TRPO 的求解

**Trust region 子问题**：

$$
\max_\theta\; L_{\theta_k}(\theta)\quad\text{s.t.}\quad \bar D_{\mathrm{KL}}(\theta_k,\theta)\le\delta
$$

**求解步骤**：

1. **线性化目标**：$L_{\theta_k}(\theta) \approx g^\top(\theta-\theta_k)$，其中 $g = \nabla_\theta L_{\theta_k}|_{\theta_k}$
2. **二次化约束**：$\bar D_{\text{KL}} \approx \frac{1}{2}(\theta-\theta_k)^\top F (\theta-\theta_k)$
3. **拉格朗日求解**：$\mathcal{L} = g^\top d - \frac{\lambda}{2}d^\top F d$，令 $\nabla_d\mathcal{L}=0$：$d^* = \frac{1}{\lambda}F^{-1}g$
4. **代入约束确定 $\lambda$**：$\frac{1}{2}(d^*)^\top F d^* = \delta$，解得步长缩放因子

最终：$\theta-\theta_k = \sqrt{2\delta/(g^\top F^{-1}g)}\cdot F^{-1}g$

**共轭梯度法求 $F^{-1}g$**：

关键技巧——不需要显式构造 $F$，只需要 **Fisher-vector product** $Fv$：

$$
Fv = \nabla_\theta\left[(\nabla_\theta \bar D_{\text{KL}})^\top v\right]
$$

这可以用 autograd 的两次反向传播在 $O(|\theta|)$ 时间内计算（第一次求 $\nabla_\theta D_{\text{KL}}$，第二次对 $v$ 方向求导）。CG 算法只需要矩阵-向量乘法，不需要显式矩阵——因此可以处理百万维参数。

**Line search backtrack**：即使有二阶近似，实际 KL 可能超标。TRPO 做 backtracking line search：
- 计算步长 $d^*$
- 尝试 $\alpha = 1, 0.5, 0.25, \ldots$
- 接受第一个满足 $\bar D_{\text{KL}}(\theta_k, \theta_k + \alpha d^*) \le \delta$ 且 $L$ 有改进的步长

> **与 SQP 的类比**：TRPO 的 trust-region 子问题结构上是 SQP（Sequential Quadratic Programming）的一次迭代。目标线性化 + 约束二次化。

### ⚠️ TRPO 常见陷阱

| 陷阱 | 表现 | 正确做法 |
|------|------|---------|
| $\delta$ 太大 | 策略跳变，reward 崩塌 | 典型 $\delta=0.01$ |
| CG 不收敛 | 步方向随机 | 加 damping $F + 0.1I$ |
| Line search 全部失败 | 一步都不更新 | 放松 $\delta$ 或检查 advantage 计算 |

---

## §9 PPO：clip 的艺术 ⭐⭐

### §9.1 从 TRPO 到 PPO 的数学动机

**TRPO 的三个工程痛点**：

1. **共轭梯度法复杂**：需要实现 Fisher-vector product、CG 迭代、line search 回溯——代码量大、调试困难
2. **不适配 GPU 大 batch**：CG 每次迭代需要全 batch 的 Fisher-vector product，难以 minibatch 化
3. **不兼容多 epoch 训练**：TRPO 每次 rollout 只做一次更新（CG 求解一次），无法像 SGD 那样对同一批数据做多次 pass

**PPO 的核心洞察**：与其精确求解 trust-region 约束，不如用一个简单的 clip 机制近似达到相同效果——限制每步更新的"实际策略变化量"。

**为什么 clip 能替代 KL 约束？** 考虑概率比 $\rho_t = \pi_\theta(a_t|s_t)/\pi_{\theta_{\text{old}}}(a_t|s_t)$：
- 当 $\rho_t = 1$ 时，新旧策略完全一致（KL = 0）
- 当 $\rho_t$ 偏离 1 时，KL 散度增大
- clip 强制 $\rho_t \in [1-\varepsilon, 1+\varepsilon]$，等价于对**每个样本**施加一个"局部 trust region"

这与 TRPO 的全局 KL 约束不同：TRPO 约束的是所有状态上的平均 KL，PPO 约束的是每个 $(s,a)$ 对上的局部概率比。实践中效果接近，但实现简单得多。

**PPO 相对 TRPO 的质的飞跃**：PPO 允许多 epoch minibatch SGD——这意味着一批数据可以被利用 $K$ 次，数据效率提升 $K$ 倍。TRPO 每批数据只用一次就必须丢弃（因为 CG 求解的是精确的 trust-region 解，多次更新会违反 trust region）。

### §9.2 PPO-Clip 目标

令 $\rho_t(\theta)=\pi_\theta(a_t|s_t)/\pi_{\theta_{\text{old}}}(a_t|s_t)$：

$$
L^{\text{CLIP}}(\theta)=\mathbb{E}_t\bigl[\min\bigl(\rho_t\hat A_t,\ \text{clip}(\rho_t,1-\varepsilon,1+\varepsilon)\hat A_t\bigr)\bigr]
$$

典型 $\varepsilon=0.2$（Isaac Lab、stable-baselines3、CleanRL 默认）。

### §9.3 几何含义

**当 $\hat A_t>0$（好动作）**：

- 如果 $\rho_t < 1+\varepsilon$：梯度正常，推动 $\pi_\theta(a_t|s_t)$ 增大
- 如果 $\rho_t > 1+\varepsilon$：$\min$ 选 clip 分支，**梯度归零**——已经走够远了

**当 $\hat A_t<0$（坏动作）**：

- 如果 $\rho_t > 1-\varepsilon$：梯度正常，推动 $\pi_\theta(a_t|s_t)$ 减小
- 如果 $\rho_t < 1-\varepsilon$：$\min$ 选 clip 分支，**梯度归零**——已经惩罚够了

**关键非对称性**：若 $\hat A_t>0$ 且 $\rho_t<1-\varepsilon$（策略意外把好动作变罕见），梯度**不被 clip**——允许恢复。这体现了 PPO "少拿奖励，多担责任"的保守策略。

> **本质洞察**：PPO-Clip 构成了原始 surrogate 目标的**悲观下界（Pessimistic Lower Bound）**。它总是选择未截断目标和截断目标中较小的那个。这正是"Proximal"（近端）的数学体现——在 $\theta_{\text{old}}$ 的局部邻域内，$L^{\text{CLIP}} \approx L^{\text{CPI}}$（一阶等价）；偏离时 clip 自动"刹车"。

### §9.3.5 PPO-Clip 的数学分析

**定理（PPO-Clip 是 surrogate 的下界）**：对任意 $\hat A_t$ 和 $\rho_t > 0$：

$$
L^{\text{CLIP}}(\theta) \le L^{\text{CPI}}(\theta) = \mathbb{E}[\rho_t \hat A_t]
$$

**证明**：
- 当 $\hat A_t > 0$：$\min(\rho_t, 1+\varepsilon) \le \rho_t$，故 $\min(\rho_t, 1+\varepsilon)\hat A_t \le \rho_t \hat A_t$
- 当 $\hat A_t < 0$：$\max(\rho_t, 1-\varepsilon) \ge \rho_t$，故 $\max(\rho_t, 1-\varepsilon)\hat A_t \le \rho_t \hat A_t$（负数乘更大正数更负）
- 两种情况下 $L^{\text{CLIP}} \le L^{\text{CPI}}$

这意味着 PPO 优化的是一个**悲观目标**——它总是假设最坏情况。这正是训练稳定性的数学来源。

**PPO-KL（较少用的变体）**：

$$
L^{\text{KL}}(\theta)=\mathbb{E}_t[\rho_t\hat A_t]-\beta\,\bar D_{\mathrm{KL}}
$$

$\beta$ 自适应：若 KL > $d_{\text{target}}$ 则 $\beta \leftarrow 2\beta$，否则 $\beta \leftarrow \beta/2$。与 PPO-Clip 近似等价，但需要额外计算 KL。

**PPO 原论文的实验结论**：clip 方法在所有测试环境中得分最高、性能最好，加上实现简单（不需要计算额外的 KL 和动态调整系数），成为主流版本。

### §9.4 PPO 为什么在实践中比 TRPO 稳定

| 因素 | TRPO | PPO | 工程影响 |
|------|------|-----|---------|
| 数据利用 | 1 次 update per rollout | $K$ epochs minibatch SGD (典型 $K$=10) | 数据效率 ×10 |
| 计算 | 需要 $F^{-1}$ (CG) | 只需一阶梯度 | 适配 GPU |
| Trust region | 严格 KL 约束 + line search | clip 提供"柔性" trust region | 无需回溯 |
| 组合工程 | 单独使用 | GAE + value clip + entropy bonus + obs norm | 37 tricks package |

### §9.4.5 PPO 的"37 tricks"（Implementation Matters）

Engstrom et al. (ICLR 2020) "Implementation Matters in Deep Policy Gradients" 发现：PPO 的成功不仅来自 clip 目标函数，更来自一整套工程 tricks 的组合：

| Trick | 作用 | 不用时的后果 |
|-------|------|------------|
| **Advantage normalization** | per-minibatch 归一化 advantage | 梯度尺度剧变，lr 难调 |
| **Value function clipping** | clip critic 更新幅度 | value 估计跳变 |
| **Reward scaling/clipping** | 限制 reward 范围 | 不同任务需要不同 lr |
| **Observation normalization** | running mean/std 归一化 obs | 网络输入尺度不一致 |
| **Gradient clipping** | 限制梯度范数 $\le 0.5$ 或 $1.0$ | 偶尔的大梯度导致参数爆炸 |
| **Entropy bonus** | $+ c \cdot H(\pi)$ | 策略过早崩塌到确定性 |
| **Orthogonal initialization** | 权重初始化为正交矩阵 | 初始策略方差太大或太小 |
| **Learning rate annealing** | 线性衰减 lr 到 0 | 后期仍有大更新导致震荡 |

**关键洞察**：去掉这些 tricks 中的任何一个，PPO 的性能可能下降 20-50%。这意味着"PPO 的成功"并非仅仅是 clip 目标函数的功劳，而是一整套工程实践的结晶。

### §9.5 PPO 关键超参数

| 超参数 | 典型值 | 敏感度 | 足式 RL 特殊性 |
|--------|-------|--------|---------------|
| clip $\varepsilon$ | 0.2 | 中 | 部分任务用 0.1-0.3 |
| GAE $\lambda$ | 0.95 | **高** | Rudin 2022 系统扫过 |
| $\gamma$ | 0.99 | 中 | 决定 horizon 有效长度 |
| epochs $K$ | 5-10 | 中 | $K>10$ 易 KL 爆炸 |
| minibatch size | 4096+ | 低 | GPU 并行越大越稳 |
| entropy coef | 0.0-0.01 | 任务相关 | locomotion 通常 0.01 |
| value clip | 有/无 | 低 | 一般开启 |

### ⚠️ 常见陷阱

| 陷阱 | 表现 | 排查 | 修正 |
|------|------|------|------|
| PPO epochs 太多 | KL 爆炸、reward 震荡 | 监控 approx-KL | $K\le 10$，超 $2\varepsilon$ early stop |
| $\varepsilon$ 过大 | 等同无 clip（A2C） | 检查 clip fraction | 恢复 0.2 |
| advantage 未归一化 | 梯度尺度剧变 | 打印 advantage stats | per-minibatch normalize |
| obs 不 normalize | reward scale 敏感 | 检查 obs 范围 | `VecNormalize`/`RunningMeanStd` |

### 练习

1. 画出 $L^{\text{CLIP}}$ 关于 $\rho_t$ 的函数图，分别对 $\hat A_t > 0$ 和 $\hat A_t < 0$ 的情况。
2. 证明：当 $\varepsilon \to \infty$ 时，PPO-Clip 退化为 vanilla policy gradient。
3. 解释为什么 PPO-Clip 对 $\rho_t$ 的非对称 clip 是合理的；若做对称 clip 会发生什么？

---

## §10 确定性策略梯度 DPG/DDPG/TD3 ⭐⭐⭐

### §10.1 DPG 定理 (Silver 2014)

**动机**：随机策略梯度 $\nabla_\theta J = \mathbb{E}_{a\sim\pi}[\nabla_\theta\log\pi \cdot Q]$ 需要对动作 $a$ 采样——在高维连续动作空间下，这个期望的 MC 估计方差可能很大（因为需要在高维空间中采样）。

**确定性策略的关键优势**：如果策略是确定性的 $\mu_\theta: \mathcal{S} \to \mathcal{A}$（对固定状态 $s$，输出唯一的动作），那么梯度公式中**不需要对 $a$ 积分**——只需要在 $a=\mu_\theta(s)$ 处计算 $\nabla_a Q$，然后通过链式法则传到 $\theta$。

**但确定性策略如何探索？** 这是确定性策略梯度的核心矛盾。解决方案：
1. **Off-policy 训练**：用一个随机行为策略 $\beta$ 采数据（如加 OU 噪声），但训练的目标策略 $\mu_\theta$ 是确定性的
2. 经验回放池中的多样性数据提供了"隐式探索"

**确定性策略适用场景**：
- 动作维度高（12-30 维），随机采样方差太大
- 需要精确的连续控制输出（如关节力矩）
- 有足够的环境交互或 replay buffer 提供探索

确定性策略 $\mu_\theta: \mathcal{S} \to \mathcal{A}$ 的梯度有更简洁的形式。

**定理 6.2.7（Deterministic Policy Gradient Theorem, Silver-Lever-Heess-Degris-Wierstra-Riedmiller, ICML 2014）**

$$
\nabla_\theta J(\mu_\theta)=\frac{1}{1-\gamma}\mathbb{E}_{s\sim d^{\mu_\theta}}\!\bigl[\nabla_\theta\mu_\theta(s)\,\nabla_a Q^{\mu_\theta}(s,a)\big|_{a=\mu_\theta(s)}\bigr]
$$

> **归一化约定一致性**：此处 $d^{\mu_\theta}$ 是归一化折扣占用测度（$\sum_s d^{\mu_\theta}(s)=1$），与本系列§9.2 的定义一致，故前置 $\frac{1}{1-\gamma}$ 因子。Silver et al. (2014) 原文使用未归一化版本 $\rho^\mu$，此时没有前置因子。

**证明骨架**：从随机策略 $\pi_\sigma(a|s)=\mathcal N(\mu_\theta(s),\sigma^2 I)$ 出发，取 $\sigma\to 0$：

1. 随机 PG：$\nabla_\theta J_\sigma=\mathbb{E}[\nabla_\theta\log\pi_\sigma \cdot Q^{\pi_\sigma}]$
2. 代入 $\nabla_\theta\log\pi_\sigma=\frac{1}{\sigma^2}\nabla_\theta\mu_\theta\cdot(a-\mu_\theta)$
3. 泰勒展开 $Q^{\pi_\sigma}(s,a)\approx Q(s,\mu_\theta)+\nabla_a Q\cdot(a-\mu_\theta)$
4. 取 $\sigma\to 0$，得 $\nabla_\theta J=\mathbb{E}[\nabla_\theta\mu_\theta\cdot\nabla_a Q|_{a=\mu_\theta}]$

> ⚠️ **严格性警告**：取 $\sigma\to 0$ 时 $Q^{\pi_\sigma}$ 本身依赖 $\sigma$，严格证明需额外论证收敛性。Silver 2014 原文通过 Regular Policy Gradient Theorem 的极限做了更精细的处理。

### §10.2 DDPG 架构

**DDPG（Lillicrap et al., ICLR 2016）** = DPG + target networks + OU 噪声 + experience replay。本质上是 DQN 思想在连续动作空间的延伸。

**为什么需要四个网络？** 强化学习是自举的（今天的预测值 = 奖励 + 明天的预测值）。如果 Critic 既负责产生训练目标又负责学习，就会出现"狗追尾巴"——目标值和预测值同时移动，训练难以收敛。目标网络通过"缓慢跟随"的方式提供一个近似固定的监督目标。

四个网络：当前 Actor $\mu_\theta$、目标 Actor $\mu_{\theta^-}$、当前 Critic $Q_\psi$、目标 Critic $Q_{\psi^-}$。

**OU 噪声 vs 高斯噪声**：论文选择 Ornstein-Uhlenbeck 噪声而非简单高斯白噪声。OU 噪声具有**时间相关性**（惯性）——如果上一帧噪声向左推，这一帧大概率还向左。这对物理控制重要：高频白噪声会被电机低通特性滤除，OU 噪声产生持续推力，有效探索新状态。

**经验回放池（Replay Buffer）**：存储 $(s,a,r,s')$ 四元组。注意存储的 $a$ 是加了噪声后实际执行的动作。随机采样打破时间相关性，使数据近似 i.i.d.。

**软更新**：$\theta^- \leftarrow \tau\theta + (1-\tau)\theta^-$，$\tau \approx 0.001-0.005$（与 DQN 的硬更新不同）。

**Actor 更新**（确定性策略梯度）：
$$
\nabla_\theta J \approx \frac{1}{N}\sum_i \nabla_a Q_\psi(s,a)|_{a=\mu_\theta(s_i)} \cdot \nabla_\theta \mu_\theta(s_i)
$$

**Critic 更新**（TD 学习）：
$$
L(\psi) = \frac{1}{N}\sum_i (Q_\psi(s_i,a_i) - y_i)^2, \quad y_i = r_i + \gamma Q_{\psi^-}(s_{i+1}, \mu_{\theta^-}(s_{i+1}))
$$

**软更新**：$\theta^- \leftarrow \tau\theta + (1-\tau)\theta^-$，$\tau \approx 0.005$

### §10.3 TD3 三重修正

DDPG 的 Q-overestimation 严重。**核心问题**：神经网络近似 Q 函数时存在误差（近似噪声），而 Q-learning 的 max 操作会系统性地选中正向噪声最大的点——导致 Q 值被高估。由于 TD 学习的自举特性，高估误差会随时间回溯传播、累积，最终发散。

**数学上**：对任何均值为 0 的噪声 $X$，Jensen 不等式给出 $\mathbb{E}[\max(X)] \ge \max(\mathbb{E}[X])$。在 Actor 寻找 $\arg\max_a Q(s,a)$ 时，非常容易选中存在正向噪声的点而非真实最优点。

**TD3（Fujimoto-van Hoof-Meger, ICML 2018）** 的三重修正：

| 修正 | 机制 | 数学原理 | 效果 |
|------|------|---------|------|
| **Twin** (Clipped Double Q) | $y=r+\gamma\min(Q_{\psi_1^-},Q_{\psi_2^-})$ | 两个独立 Critic 的噪声不相关，取 min 削掉假峰值 | 抑制高估 |
| **Delayed** (延迟 Actor 更新) | 每 $d=2$ 个 Critic 步更新 1 次 Actor | TD 误差的相干累积使方差随步数增大，多次 Critic 更新降低方差 | 减少梯度噪声 |
| **Smoothing** (目标策略平滑) | $\tilde a=\mu_{\theta^-}(s')+\text{clip}(\epsilon,-c,c)$ | 相邻动作应有相近 Q 值，加噪声正则化 Q 曲面 | 防止过拟合尖峰 |

**为什么低估比高估安全？** 高估导致 Actor 倾向次优动作 + Critic 进一步高估 = 正反馈发散。低估只是"谨慎"，通过持续探索仍有机会纠正。

**TD3 共 6 个网络**：Actor 当前/目标 + Critic 1 当前/目标 + Critic 2 当前/目标。

**经验结果**：MuJoCo Hopper/Walker2d/HalfCheetah/Ant/Humanoid 上 TD3 与 SAC 打平或略胜 DDPG 50%+。

> **DDPG 在 2026 年基本只有教学价值，生产环境已被 TD3/SAC 取代。** 其历史贡献在于启发了 TD3 的 twin critics 和 target policy smoothing。

### ⚠️ DPG/DDPG/TD3 常见陷阱

| 陷阱 | 表现 | 根本原因 | 正确做法 |
|------|------|---------|---------|
| 忘 target network | Q 值发散 | 自举无固定监督目标 | soft update $\tau\approx 0.005$ |
| OU 噪声 scale 太大 | 策略抖动、reward 低 | 过度探索覆盖学到的好动作 | 训练后期 anneal 噪声 |
| Replay buffer 太小 | 相关性打破不充分 | 采样 i.i.d. 假设不满足 | $10^5 - 10^6$ 大小 |
| Actor 更新太频繁 | Q 值不收敛 | Critic 还没学准就指导 Actor | 延迟更新 $d=2$ |

---

## §11 最大熵 RL 与 SAC ⭐⭐⭐

### §11.1 最大熵 MDP 目标

**动机：为什么标准 RL 目标不够好？**

标准 RL 目标 $\max_\pi \mathbb{E}[\sum \gamma^t r_t]$ 训练出的策略有两个严重问题：

1. **脆弱性**：策略倾向于"坍缩"到一个确定性动作——当 Q 估计不准时，确定性策略会锁定在错误的峰值上，无法通过自身随机性探索逃出
2. **单模态**：如果多个动作的 Q 值接近（如向左绕和向右绕都能到达目标），标准 RL 只会学到其中一个——丧失了"保持多个备选方案"的能力

**最大熵 RL 的哲学**：在统计建模中，最大熵方法的核心原则是"在匹配已知约束的前提下，对未知信息做最少假设"。类比到 RL：在获得足够高的回报的前提下，让策略尽可能随机——这样对环境变化的假设最少，鲁棒性最强。

**动机**：标准 RL 学到的策略可能过于"确定"——一旦环境有微小变化就崩溃。最大熵框架鼓励保持"有用的随机性"。

$$
J_{\text{MaxEnt}}(\pi)=\mathbb{E}_{\pi}\!\left[\sum_{t=0}^{\infty}\gamma^t\bigl(r(s_t,a_t)+\alpha\mathcal{H}(\pi(\cdot|s_t))\bigr)\right]
$$

其中 $\alpha>0$ 为 temperature，$\mathcal{H}(\pi(\cdot|s)) = \mathbb{E}_{a\sim\pi}[-\log\pi(a|s)]$ 为策略熵。

**三大优势**：
1. **探索性**：鼓励 agent 探索更广泛的状态空间
2. **多模态**：策略可以捕获多种近优行为模式
3. **鲁棒性**：对模型误差和估计误差更稳健（Ziebart 2010）

### §11.2 Soft Bellman 算子 ⭐⭐⭐

**Soft value functions**：

$$
V^\pi_{\text{soft}}(s)=\mathbb{E}_{a\sim\pi}[Q^\pi_{\text{soft}}(s,a) - \alpha\log\pi(a|s)]
$$

$$
Q^\pi_{\text{soft}}(s,a)=r(s,a)+\gamma\mathbb{E}_{s'}[V^\pi_{\text{soft}}(s')]
$$

**Soft Bellman optimality operator**：

$$
(\mathcal{T}^*_{\text{soft}}Q)(s,a):=r(s,a)+\gamma\mathbb{E}_{s'}\!\left[\alpha\log\sum_{a'}\exp(Q(s',a')/\alpha)\right]
$$

**定理 6.2.8（Soft Bellman 算子压缩性, Haarnoja 2017）**

$\mathcal{T}^*_{\text{soft}}$ 在 $\|\cdot\|_\infty$ 下是 $\gamma$-压缩。

**证明**：设 $Q_1, Q_2$ 为两个 Q 函数。log-sum-exp 的 Lipschitz 性质：

$$
|\alpha\log\sum_a e^{x_a/\alpha} - \alpha\log\sum_a e^{y_a/\alpha}| \le \|x-y\|_\infty
$$

因此 $\|\mathcal{T}^*_{\text{soft}}Q_1 - \mathcal{T}^*_{\text{soft}}Q_2\|_\infty \le \gamma\|Q_1-Q_2\|_\infty$。$\blacksquare$

> **与 6.1 的桥梁**：这个证明与经典 Bellman 算子的 $\gamma$-压缩证明结构完全平行。log-sum-exp 是 $\max$ 的软化版本，$\max$ 的 Lipschitz 常数为 1，LSE 也是 1。

### §11.3 最优策略 = Boltzmann 分布

**定理 6.2.9**：$\pi^*(a|s)\propto\exp(Q^*_{\text{soft}}(s,a)/\alpha)$

**证明**：最大化 $\mathbb{E}_a[Q-\alpha\log\pi]$ s.t. $\sum\pi=1$。拉格朗日：$\frac{\partial}{\partial\pi(a|s)}[Q/\alpha - \log\pi - 1 - \lambda]=0$，解得 $\pi \propto \exp(Q/\alpha)$。

### §11.4 SAC 算法 ⭐⭐

**SAC（Haarnoja-Zhou-Abbeel-Levine, ICML 2018）** 把 soft PI 做 off-policy 连续动作化：

**三个网络**：
- **Critic**：Twin Q $Q_{\theta_1}, Q_{\theta_2}$（承袭 TD3）
- **Actor**：对角高斯 $\pi_\phi(a|s) = \mathcal{N}(\mu_\phi(s), \sigma_\phi^2(s))$

**重参数化技巧**（Kingma & Welling 2013）：

$$
a = f_\phi(\epsilon; s) = \tanh(\mu_\phi(s) + \sigma_\phi(s) \odot \epsilon), \quad \epsilon \sim \mathcal{N}(0,I)
$$

**为什么需要重参数化？** 策略梯度 $\nabla_\phi \mathbb{E}_{a\sim\pi_\phi}[Q(s,a)]$ 中对 $a$ 的采样阻断了梯度流。重参数化把随机性从"动作分布"挪到"噪声变量 $\epsilon$"上，使梯度可以通过链式法则穿过 $f_\phi$。

**Actor 损失**：

$$
J_\pi(\phi) = \mathbb{E}_{s\sim\mathcal{D}, \epsilon\sim\mathcal{N}}[\alpha\log\pi_\phi(f_\phi(\epsilon;s)|s) - Q_\theta(s, f_\phi(\epsilon;s))]
$$

**Critic 损失**：

$$
J_Q(\theta_i) = \mathbb{E}_{(s,a,r,s')\sim\mathcal{D}}\left[\frac{1}{2}(Q_{\theta_i}(s,a) - y)^2\right]
$$

$$
y = r + \gamma[\min_{j=1,2}Q_{\theta_j^-}(s',\tilde a') - \alpha\log\pi_\phi(\tilde a'|s')], \quad \tilde a'\sim\pi_\phi(\cdot|s')
$$

### §11.4.5 SAC 的训练流程详解

**完整的 SAC 训练循环**：

```
初始化：Critic Q_θ1, Q_θ2, Actor π_φ, Target Q_θ1^-, Q_θ2^-, Replay Buffer D
重复：
  1. 从 π_φ 采样动作 a = tanh(μ_φ(s) + σ_φ(s)⊙ε), ε~N(0,I)
  2. 执行 a，获得 (s, a, r, s')，存入 D
  3. 从 D 中采样 minibatch {(s_i, a_i, r_i, s'_i)}
  4. 计算 target:
     - 采样 ã'~π_φ(·|s')
     - y = r + γ[min(Q_θ1^-(s',ã'), Q_θ2^-(s',ã')) - α log π_φ(ã'|s')]
  5. 更新 Critic: minimize (Q_θi(s,a) - y)^2 for i=1,2
  6. 更新 Actor: minimize α log π_φ(ã|s) - min(Q_θ1(s,ã), Q_θ2(s,ã))
     其中 ã = f_φ(ε;s) (重参数化)
  7. 更新 Temperature: minimize -α(log π_φ(ã|s) + H_target)
  8. 软更新 Target: θ_i^- ← τθ_i + (1-τ)θ_i^-
```

**SAC vs PPO 的工程选型**：

| 维度 | SAC | PPO | 选择建议 |
|------|-----|-----|---------|
| 样本效率 | 高（replay） | 低（on-policy） | 真机训练选 SAC |
| 超参数敏感度 | 低（auto-temp） | 中 | 调参成本选 SAC |
| GPU 并行友好 | 中 | **极高** | 大规模仿真选 PPO |
| 稳定性 | 高 | 高 | 两者接近 |
| sim-to-real | 中 | **高** | 域随机化选 PPO |
| 探索能力 | **强**（内嵌熵） | 中（entropy bonus） | 稀疏奖励选 SAC |

### §11.5 SAC v1 到 v2 的演进 ⭐⭐⭐

| 版本 | 网络 | Temperature | 改进 |
|------|------|-------------|------|
| **v1** (ICML 2018) | Q + V + Actor | 手动固定 $\alpha$ | 额外 V 网络稳定训练 |
| **v2** (arXiv:1812.05905) | 仅 Q + Actor | 自动调节 $\alpha$ | 去掉 V 网络，约束熵目标 |

**自动温度调节**：将 $\alpha$ 视为对偶变量，约束策略熵不低于目标 $\mathcal{H}_{\text{target}}$：

$$
\alpha^* = \arg\min_\alpha \mathbb{E}_{a\sim\pi}[-\alpha\log\pi(a|s) - \alpha\mathcal{H}_{\text{target}}]
$$

$\mathcal{H}_{\text{target}}$ 通常设为 $-\dim(\mathcal{A})$（每个动作维度 1 nat 的熵）。

**自动温度的直觉**：当策略变得太确定（熵低于目标），$\alpha$ 自动增大，强迫策略保持随机；当策略太随机（熵高于目标），$\alpha$ 减小，允许更多确定性。这消除了手动调 $\alpha$ 的需要——这是 SAC v2 相对 v1 最大的工程改进。

**为什么 v2 去掉了 V 网络？** v1 中 V 网络的作用是提供一个稳定的 bootstrap 目标。但 v2 发现可以直接用 $V(s) = \mathbb{E}_{a\sim\pi}[\min_j Q_{\theta_j}(s,a) - \alpha\log\pi(a|s)]$ 替代——无需额外网络，减少了参数量和计算开销。Twin Q 的 min 操作已经提供了足够的稳定性。

### §11.5.5 SAC 的理论局限与适用边界

**SAC 适合的场景**：
- 动作空间连续且维度中等（3-30 维）
- 环境有多种可行策略（多模态奖励景观）
- 需要探索（稀疏奖励或复杂地形）
- 样本珍贵（真机交互）

**SAC 不适合的场景**：
- 大规模并行仿真（on-policy PPO 更简单高效）
- 需要严格确定性输出（如某些安全关键系统）
- 离散动作空间（需要修改为 Discrete SAC）
- Domain randomization 场景（replay buffer 中 DR 参数混合导致 critic 方差大）

> ⚠️ **SAC 的 $\tanh$ squashing Jacobian 陷阱**：
> 由于 $a = \tanh(u)$，对数概率需要修正：
> $$\log\pi(a|s) = \log\pi_{\text{pre}}(u|s) - \sum_{i=1}^d \log(1-\tanh^2(u_i))$$
> 忘记这个 Jacobian 修正是最常见的 SAC 实现 bug。

### §11.6 与变分推断的统一视角 ⭐⭐⭐⭐

**Levine 2018（arXiv:1805.00909）**："RL = Control as Inference"

**核心思想**：引入二值"最优性变量" $\mathcal{O}_t \in \{0,1\}$，表示"时刻 $t$ 的行为是否最优"。定义：

$$
p(\mathcal{O}_t = 1 | s_t, a_t) = \exp(r(s_t, a_t) / \alpha)
$$

即高奖励动作被判定为"最优"的概率更高。在这个概率图模型下：

$$
\pi^*(a|s) = p(a|s, \mathcal{O}_{t:\infty}=1)
$$

最优策略就是"在所有未来时刻都最优的条件下，应该采取什么动作"——这是一个后验推断问题！

**统一框架**：

| 概念 | RL 视角 | 推断视角 |
|------|---------|---------|
| 最优策略 | $\arg\max_\pi J(\pi)$ | $p(a|s, \mathcal{O}=1)$ |
| Soft Q | Soft Bellman 不动点 | $\log p(\mathcal{O}_{t:\infty}=1|s,a)$ |
| Soft V | $\alpha\log\sum_a e^{Q/\alpha}$ | $\log p(\mathcal{O}_{t:\infty}=1|s)$ |
| 策略优化 | 梯度上升 | 变分推断（minimizing KL） |
| SAC | Off-policy AC | Amortized variational inference |

**为什么这个统一视角重要？**

1. 它解释了为什么最大熵策略是 Boltzmann 分布——这就是后验的指数族形式
2. 它把 RL 的算法设计问题转化为概率推断的算法设计——后者有丰富的工具箱（VI, MCMC, normalizing flows）
3. 它自然地导出了 soft Bellman 方程——作为 message passing 的结果

**与 Path Integral Control 的联系**：Kappen (2005) 和 Theodorou (2010) 的 PI$^2$ 算法基于同样的 KL 约束随机最优控制框架。连续时间下，soft Bellman 变为 Chapman-Kolmogorov 方程的 log-transform，控制问题线性化。SAC 是这个连续时间理论的离散时间、神经网络 amortized 版本。

---

---

## §12 机器人应用：PPO 的主导地位 ⭐⭐

### §12.1 为什么 PPO 主导足式 RL

**历史转折点**：2019 年之前，足式 RL 还是"能不能 work"的问题；Hwangbo 2019（ANYmal, Science Robotics）用 TRPO 首次证明了 RL 策略可以 sim-to-real 迁移到真实四足机器人。2022 年 Rudin 的工作彻底改变了格局——从"能不能"变成"多快能训出来"。

**Rudin 2022（arXiv:2109.11978）是分水岭**：Isaac Gym 上 4096 个并行 env，单次 update 约 100k 样本，V100 上 ANYmal 训练 < 20 分钟（较 CPU baseline 快约 10000 倍）。

**为什么不用 SAC（off-policy）？**

| 因素 | On-policy (PPO) | Off-policy (SAC) | 足式 RL 场景判定 |
|------|----------------|-----------------|----------------|
| 样本成本 | 高（需重新采） | 低（replay） | GPU 并行使样本近零成本 → PPO 赢 |
| Domain Randomization | 每 batch 策略-env 匹配 | replay 中 DR 参数混合 | DR 稳定性 → PPO 赢 |
| 探索 | entropy bonus | 内嵌最大熵 | reward shaping 信息量大 → PPO 够 |
| 大 batch | clip 友好 | Q-bootstrap 延迟偏差 | GPU 大并行 → PPO 赢 |
| 工程生态 | Isaac Lab 全栈优化 | 需额外适配 | 一致性 → PPO 赢 |

### §12.2 典型超参数（Rudin 2022 配置）

```
horizon=24, mini-batches=4, K=5 epochs, clip ε=0.2
GAE λ=0.95, γ=0.99, entropy coef=0.01
obs/return normalization=True, gradient clip=1.0
4096 parallel environments, lr=3e-4 (Adam)
```

**超参数选择的深层逻辑**：

- **horizon=24**：对应 24 步 rollout，约 0.48 秒（20ms 控制周期）。为什么这么短？因为足式 RL 中一步摔倒就结束，长 horizon 反而浪费。24 步足够看到一两个完整步态周期。
- **4096 并行 env**：这是 PPO 在 GPU 仿真下工作的关键。4096 个 env 在一块 V100 上同时前向仿真，每次 update 的有效 batch size = 4096 * 24 = 98304。这个 batch 大到足以让梯度估计方差很小，使得小学习率 + 稳定更新成为可能。
- **K=5 epochs**：每批数据重复利用 5 次。超过 10 次通常 KL 爆炸。Rudin 发现 5 是平衡点。
- **$\gamma=0.99$**：有效折扣视界约 100 步。配合 horizon=24 意味着 GAE 在 ~24 步后自然截断（$0.99^{24} \approx 0.79$）。

**同步并行 vs 异步并行**：

Isaac Lab / legged_gym 使用**同步并行**（所有 env 在同一 GPU 上同步仿真），而非 A3C 的异步并行。原因：

1. GPU 张量操作天然同步——所有 4096 个 env 共享一次 forward pass
2. 无策略滞后（所有数据来自完全相同的策略版本）
3. 数据天然组成大 batch，无需额外收集/等待

### §12.3 Dexterous Manipulation：PPO vs SAC 并存

**OpenAI 2019 Rubik's Cube（arXiv:1910.07113）**：使用 PPO + Automatic Domain Randomization (ADR)——证明即使在接触丰富、高维连续控制（Shadow hand 20 DoF）下，PPO 在足够计算下可以扩展。

**Haarnoja 2019 "Learning to Walk via Deep RL"（RSS 2019）**：罕见的真机 SAC 成功案例（Minitaur 四足）——SAC 的样本效率在无大规模并行仿真时仍有优势。

**Open X-Embodiment / RT-2（2023-2024）路线**：偏向 imitation learning + VLA，原因：真机数据代价高、manipulation 任务 reward 稀疏，纯 RL 不如模仿 + fine-tune。

### §12.4 PPO + Teacher-Student 范式

**Miki 2022（感知型四足，Science Robotics）**：

1. **Teacher** 有 privileged info（地形高度图、精确接触力）→ PPO 训练
2. **Student** 只有 proprioception → 模仿 Teacher 的行为（行为克隆 + PPO fine-tune）

后续所有 parkour 工作（Agarwal 2022、Cheng 2023、Zhuang 2023）延续此范式。

> **跨领域类比**：Teacher-Student 范式类似于 VAE 的编码器-解码器结构——Teacher 获取完整信息并压缩为隐表征，Student 从带噪声的输入中学会重构该表征。但不同于 VAE 的重构损失，这里用的是行为克隆损失（Student 模仿 Teacher 的动作而非重构输入）。

### §12.5 人形 RL

**Radosavovic 2024（Digit, Science Robotics 9:eadi9579）**：PPO 训练 causal Transformer，输入 proprioceptive 历史窗口输出 action。**零样本迁移**到真机 Digit 在旧金山街头行走。

**超参细节**：PPO clip 0.2, 5 epochs, minibatch 12288, actor lr $10^{-5}$ cosine + 100 iter warmup, critic lr $5\times10^{-4}$, 不使用 entropy regularization（因 Transformer 已有充分表达性）。

**更进一步**：Radosavovic 2024 "Learning Humanoid Locomotion over Challenging Terrain"（arXiv:2410.03654）——Transformer 先 pretrain (sequence modeling) 再 PPO fine-tune，完成 Berkeley 4 英里徒步。

**工程趋势**：Unitree H1/G1 等大量人形复现都沿用 legged_gym → Isaac Lab 的 PPO 管线。

### §12.6 PPO 在足式 RL 中主导的深层原因分析

**为什么不是 SAC？为什么不是 model-based？为什么不是 offline RL？**

| 替代方案 | 理论优势 | 在足式 RL 中失败的原因 |
|---------|---------|---------------------|
| SAC (off-policy) | 样本效率高 | GPU 并行使样本成本为零；replay 与 DR 冲突 |
| Model-based (MBPO/Dreamer) | 样本效率极高 | 接触动力学建模困难；model error 在接触切换时爆炸 |
| Offline RL (CQL/IQL) | 无需在线交互 | 需要大量高质量 demonstrations；分布偏移严重 |
| Evolution strategies (CMA-ES) | 无需微分 | 参数维度 >$10^5$ 时效率崩塌 |

**PPO 的根本优势**：它是在"massive parallel simulation + domain randomization"这个特定工程范式下的最优选择。当你有 4096 个并行环境，每秒采 $10^6$ 样本时，on-policy 的劣势（样本浪费）完全被抹平，而它的优势（策略-数据分布一致、无 replay 偏差、与 DR 兼容）变得决定性。

---

## §12.7 重要性采样与 Off-Policy 策略梯度 ⭐⭐

### Off-Policy 的动机

策略梯度天然是 on-policy 的——每次更新后旧数据作废。这在真机训练中代价高昂。**重要性采样（Importance Sampling）** 允许用旧策略 $\pi_{\theta_{\text{old}}}$ 的数据来估计新策略 $\pi_\theta$ 的梯度。

### 重要性采样的数学基础

**核心恒等式**：对于任意两个分布 $p(x)$ 和 $q(x)$：

$$
\mathbb{E}_{x\sim p}[f(x)] = \mathbb{E}_{x\sim q}\left[\frac{p(x)}{q(x)}f(x)\right]
$$

**应用到轨迹级别**：

$$
\frac{p_\theta(\tau)}{p_{\theta_{\text{old}}}(\tau)} = \frac{p(s_0)\prod_t \pi_\theta(a_t|s_t)p(s_{t+1}|s_t,a_t)}{p(s_0)\prod_t \pi_{\theta_{\text{old}}}(a_t|s_t)p(s_{t+1}|s_t,a_t)} = \prod_{t=1}^T \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{\text{old}}}(a_t|s_t)}
$$

初始状态和转移概率消去（两策略共享同一环境）。

### 连乘导致的方差爆炸

**致命问题**：即使每步比率是 $1.01$，$T=100$ 步连乘后权重 $\approx 2.7$，方差指数增长。若某步比率为 $0.01$，整条轨迹权重几乎为零——大量数据被浪费。

**这正是 PPO 存在的根本原因**：PPO 的 clip 机制本质上就是一种"截断的重要性采样"——只用相邻策略的数据（$\rho_t$ 接近 1），通过 clip 强制 $\rho_t \in [1-\varepsilon, 1+\varepsilon]$，避免了轨迹级连乘的方差灾难。

> **本质洞察**：PPO 不是真正的 off-policy 算法，而是"局部 off-policy"——它只重用前几步的数据（$K$ 个 epoch），两策略差距很小，重要性权重接近 1。这在数学上避开了完整 off-policy 的方差问题，同时获得了数据复用的好处。

### PPO 的代码实现核心

```python
# PPO 核心更新逻辑（简化自 CleanRL）
def update_ppo(actor, critic, states, actions, old_log_probs, advantages, returns,
               clip_eps=0.2, epochs=10, minibatch_size=64):
    """
    states: 采集的状态 [T, obs_dim]
    actions: 采集的动作 [T, act_dim]
    old_log_probs: 采集时的 log π_old(a|s) [T]
    advantages: GAE 计算的优势 [T]
    returns: GAE 计算的 returns = advantages + values [T]
    """
    # 优势归一化（per-batch）——关键 trick
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
    
    dataset_size = len(states)
    for epoch in range(epochs):
        # 随机打乱，分 minibatch
        indices = torch.randperm(dataset_size)
        for start in range(0, dataset_size, minibatch_size):
            mb_idx = indices[start:start+minibatch_size]
            mb_states = states[mb_idx]
            mb_actions = actions[mb_idx]
            mb_old_logp = old_log_probs[mb_idx]
            mb_adv = advantages[mb_idx]
            mb_returns = returns[mb_idx]
            
            # 计算新策略的 log prob
            new_logp = actor.log_prob(mb_states, mb_actions)
            
            # 概率比 ρ = π_new / π_old（用 log 相减再 exp，数值稳定）
            ratio = torch.exp(new_logp - mb_old_logp)
            
            # PPO-Clip 损失
            surr1 = ratio * mb_adv
            surr2 = torch.clamp(ratio, 1-clip_eps, 1+clip_eps) * mb_adv
            actor_loss = -torch.min(surr1, surr2).mean()
            
            # Critic 损失（可选 value clip）
            critic_loss = F.mse_loss(critic(mb_states).squeeze(), mb_returns)
            
            # 反向传播更新
            actor_optimizer.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.0)  # 梯度裁剪
            actor_optimizer.step()
            
            critic_optimizer.zero_grad()
            critic_loss.backward()
            critic_optimizer.step()
        
        # 早停：监控 approx KL
        with torch.no_grad():
            new_logp_all = actor.log_prob(states, actions)
            approx_kl = (old_log_probs - new_logp_all).mean()
            if approx_kl > 1.5 * clip_eps:  # KL 超标，提前终止
                break
```

---

## §13 与控制理论的映射 ⭐⭐⭐

### §13.1 对应关系总览

| RL 概念 | 控制理论对应 | 联系 |
|---------|------------|------|
| PG 定理 | Adjoint method | $\nabla_a Q$ = discrete-time costate |
| DPG | Pontryagin 原理 | 确定性反馈 $u=\mu(x)$ |
| TRPO trust region | SQP | 目标线性化 + 约束二次化 |
| Soft Bellman | Linearly-solvable MDP (Todorov 2007) | $z=\exp(V/\alpha)$ 线性化 |
| SAC | Path integral control (PI$^2$) | KL 约束下的随机最优控制 |
| LQR PG (Fazel 2018) | LQR $u=-Kx$ | 非凸但 gradient-dominated |

### §13.2 DPG 与 Adjoint Method 的深层联系

经典最优控制中，adjoint method（伴随法）给出：

$$
\nabla_\theta J = \int_0^T \lambda(t)^\top \frac{\partial f}{\partial u}\frac{\partial \mu}{\partial\theta}\, dt
$$

其中 $\lambda(t)$ 是 costate（伴随变量），满足 $\dot\lambda = -\frac{\partial H}{\partial x}$。

DPG 定理：$\nabla_\theta J = \mathbb{E}[\nabla_\theta\mu_\theta(s)\cdot\nabla_a Q|_{a=\mu(s)}]$

**结构同构**：$\nabla_a Q(s,a)|_{a=\mu(s)}$ 正是 discrete-time costate——"动作微小变化对长期回报的影响方向"。$\nabla_\theta\mu$ 将动作空间梯度映射回参数空间。

### §13.3 最大熵 RL 与 linearly-solvable MDP

Todorov (2007) 设 $z(s)=\exp(V(s)/\alpha)$，soft Bellman 方程线性化为 $z = Pz$——控制问题变特征值问题。Path integral control (Kappen 2005, Theodorou 2010) 基于同样的 KL 约束。**SAC 可以看作 path integral control 的神经网络 amortized 版本。**

### §13.4 LQR 策略梯度全局收敛

**Fazel-Ge-Kakade-Mesbahi (ICML 2018)**：线性系统 + 二次代价，策略 $u=-Kx$。$J(K)$ 非凸但 gradient-dominated（PL 条件）：

$$
J(K)-J(K^*) \le C\|\nabla J(K)\|^2
$$

梯度法全局收敛。这是"非凸 PG 有全局保证"的第一个严格结果（详见 6.4）。

### §13.5 PG vs DDP/iLQR

经典 DDP/iLQR 在 trajectory space 迭代（open-loop + local feedback），PG 在 policy function space 迭代（global feedback）。PPO/TRPO 的 trust region 与 DDP 的 regularized Hessian 精神同族——都避免过激更新。

---

## §14 算法全家族对比 ⭐⭐

### 演进路线

```
REINFORCE (1992) --> + Baseline --> Actor-Critic (A2C, 2016)
    |                                    |
    |                              + GAE + NPG
    |                                    |
    |                              TRPO (2015) --> PPO (2017)
    |
    +-- sigma->0 --> DPG (2014) --> DDPG (2016) --> TD3 (2018)
                                         |
                                   + Max-Entropy
                                         |
                                    SAC (2018)
```

### 全家族对比表

| 算法 | On/Off | 策略 | 核心技巧 | 样本效率 | 稳定性 | 首选场景 |
|------|:---:|:---:|---------|:---:|:---:|---------|
| REINFORCE | On | 随机 | MC 估计 | 极低 | 低 | 教学 |
| A2C | On | 随机 | TD + baseline | 低 | 中 | 简单任务 |
| TRPO | On | 随机 | KL + CG | 低 | 高 | 理论研究 |
| **PPO** | On | 随机 | **Clip + GAE** | 低 | **极高** | **足式/人形** |
| DDPG | Off | 确定 | Target + OU | 中 | 低 | 已弃用 |
| **TD3** | Off | 确定 | Twin Q + Delay | 中 | 中 | 确定性控制 |
| **SAC** | Off | 随机 | 最大熵 + auto-$\alpha$ | **高** | **高** | **操纵/探索** |

### 选型决策

- **大规模并行仿真 + sim-to-real** (Isaac Lab): **PPO**
- **真机交互 + 稀疏奖励**: **SAC**
- **确定性精确控制 + Lyapunov 稳定性**: **TD3**
- **需要理论保证**: TRPO / LQR-PG

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "REINFORCE 方差大是因为算法有 bug" | REINFORCE 的高方差是数学固有性质，不是实现问题。$\text{Var}[\nabla\log\pi \cdot R] \propto \text{Var}[R]$，Monte Carlo 回报 $R$ 本身包含从当前到 episode 结束的所有随机性。Baseline 减方差是必须的工程手段，而非可选优化 |
| "PPO 的 clip 就是简单截断，没什么理论基础" | PPO-Clip 构成原始 surrogate 目标的悲观下界（Pessimistic Lower Bound）：$L^{\text{CLIP}} \le L^{\text{CPI}}$。clip 机制对每个 $(s,a)$ 样本施加局部 trust region，在一阶近似下与 TRPO 的全局 KL 约束等价。"简单"的是实现，不是原理 |
| "Actor-Critic 比纯 PG 严格更好" | Actor-Critic 用 Critic 估计 $Q^\pi$ 替代 MC 回报，降低方差但引入偏差。当 Critic 拟合质量差（训练初期或函数逼近能力不足）时，偏差可能比方差更有害。Compatible Function Approximation 定理给出了 Critic 无偏的充要条件，但实践中几乎不满足 |
| "GAE 的 $\lambda$ 越大越好，因为偏差越小" | $\lambda$ 控制偏差-方差权衡：$\lambda \to 1$ 趋向 MC（无偏高方差），$\lambda \to 0$ 趋向 TD(0)（有偏低方差）。最优 $\lambda$ 取决于 Critic 质量和 episode 长度。Isaac Lab 默认 $\lambda=0.95$ 是经验折中，不是理论最优 |
| "PPO 中多 epoch 训练总是有益的" | 多 epoch 复用数据提高了数据效率，但随着 epoch 数增加，$\pi_\theta$ 逐渐偏离 $\pi_{\theta_{\text{old}}}$，importance sampling 比率 $\rho_t$ 偏离 1。虽然 clip 限制了梯度，但值函数 target 的 staleness 问题不受 clip 保护。过多 epoch（如 $>10$）可能导致过拟合当前 batch 的噪声 |

---

## 本章小结

| 知识点 | 核心公式/结论 | 地位 | 工程用途 |
|--------|-------------|------|---------|
| PG 定理 | $\nabla J = \mathbb{E}[\nabla\log\pi \cdot Q]$ | 所有 PG 的根 | 一切 PG 算法的基础 |
| REINFORCE | MC 估计 PG | 教学 | 理解梯度估计 |
| Baseline | 减方差不引偏差 | 核心 | 所有 AC 必用 |
| GAE | $\hat A = \sum(\gamma\lambda)^l\delta_{t+l}$ | 核心 | PPO/TRPO 标配 |
| NPG | $F^{-1}\nabla J$（参数化不变） | 理论 | 理解 TRPO/PPO 的"为什么" |
| TRPO | 单调改进 + KL 约束 | 里程碑 | 被 PPO 实践取代 |
| PPO-Clip | $\min(\rho A, \text{clip}(\rho)A)$ | **工业标准** | 足式/人形/RLHF |
| DPG/TD3 | $\nabla J = \mathbb{E}[\nabla\mu \cdot \nabla_a Q]$ | off-policy | 确定性控制 |
| SAC | 最大熵 + soft Bellman + reparam | off-policy 标准 | 操纵任务 |

---

## §15 部署：从训练到真机推理 ⭐⭐

### ONNX 导出与 C++ 推理

训练完成后的部署路径：

1. **PyTorch Actor 导出 ONNX**：
```python
# 导出时只需 Actor 网络（Critic 在部署时不需要）
dummy_obs = torch.zeros(1, obs_dim).to(device)
torch.onnx.export(
    actor_network,          # 只导出 policy（不含 critic）
    dummy_obs,              # 示例输入（确定输入形状）
    "policy.onnx",
    input_names=["obs"],
    output_names=["action_mean"],  # 部署时通常只用均值（不采样）
    opset_version=11
)
```

2. **C++ 推理（ONNX Runtime）**：
```cpp
// ONNX Runtime C++ 推理（机器人 real-time 控制循环）
#include <onnxruntime_cxx_api.h>

Ort::Session session(env, "policy.onnx", session_options);
// 1kHz 控制循环中：
std::vector<float> obs = get_observation();  // 48 维 proprioception
auto input_tensor = Ort::Value::CreateTensor<float>(..., obs.data(), ...);
auto output = session.Run({}, {"obs"}, {input_tensor}, {"action_mean"});
float* action = output[0].GetTensorMutableData<float>();  // 12 维关节扭矩
send_to_motors(action);
```

3. **关键注意事项**：
   - 部署时使用**确定性推理**（取均值 $\mu_\theta(s)$，不采样）——消除部署随机性
   - **observation normalization 必须同步**：训练时的 running mean/std 必须保存并在部署时使用
   - **action clip 一致**：训练和部署使用完全相同的动作范围限制

---

## 累积项目：本章新增模块

**项目**：从零构建策略梯度训练管线

| 章节 | 新增模块 | 产出物 |
|------|---------|--------|
| §4 | REINFORCE + baseline on CartPole | `reinforce.py` |
| §5-6 | Actor-Critic + GAE on Pendulum | `actor_critic.py` |
| §9 | PPO on MuJoCo HalfCheetah | `ppo.py` |
| §11 | SAC on MuJoCo Humanoid | `sac.py` |
| §12 | PPO on Isaac Lab ANYmal flat | `isaac_ppo.py` + `policy.onnx` |

---

## 延伸阅读

| 资源 | 难度 | 价值 |
|------|------|------|
| Sutton & Barto 2018 Ch.13 | ⭐ | 入门首选 |
| Schulman 博士论文 "Optimizing Expectations" (2016) | ⭐⭐⭐ | TRPO/GAE 一手推导 |
| Agarwal-Kakade-Lee-Mahajan "On the Theory of PG Methods" (JMLR 2021) | ⭐⭐⭐⭐ | PG 全局收敛理论 |
| Levine CS285 Lec 5-10 | ⭐⭐ | 现代 deep RL 全覆盖 |
| CleanRL `ppo_continuous_action.py` | ⭐⭐ | 单文件精读 PPO |
| Spinning Up (OpenAI) | ⭐⭐ | VPG/PPO/SAC 专页 |
| Lilian Weng "Policy Gradient Algorithms" | ⭐⭐ | 全家族对照 |
| Bertsekas 2019 Ch.5 | ⭐⭐⭐ | PI 视角的 PG 理论 |
| Meyn 2022 Ch.9-10 | ⭐⭐⭐⭐ | 随机逼近 + PG 收敛性严格分析 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| PPO reward 不涨/震荡 | 1.奖励缩放失衡 2.clip 过大 3.网络太小 | 1.打印各奖励分项 2.检查 clip fraction 3.监控 approx-KL | §9 |
| SAC Q 值发散 | 1.忘 target network 2.忘 $\tanh$ Jacobian 3.lr 过大 | 1.检查 soft update $\tau$ 2.打印 $\log\pi$ 3.降 critic lr | §11 |
| Actor-Critic 训练崩溃 | 1.同学习率 2.critic 太弱 3.advantage 未归一化 | 1.分离 lr 2.增大 critic 3.per-batch normalize | §6 |
| GAE 计算出 NaN | 1.reward 含 inf 2.done mask 错误 3.value 预测爆炸 | 1.clip reward 2.检查 `(1-done)` 3.加 gradient clip | §5 |
| sim-to-real 策略失败 | 1.obs 分布偏移 2.action clip 不一致 3.std 过大 | 1.对齐 obs normalization 2.训练/部署同步 clip 3.训练后期固定 std | §12 |

---

## §15.5 部署常见问题与排查

| 问题 | 可能原因 | 排查方法 |
|------|---------|---------|
| 真机行为与仿真差异大 | obs normalization 参数未同步 | 对比训练时与部署时的 obs 统计量 |
| 关节过载/电流超限 | action clip 不一致 | 确保训练时的 clip 范围 = 电机真实限制 |
| ONNX 推理输出 NaN | 输入维度不匹配 | 检查 dummy_input 形状与真实 obs 一致 |
| 策略在真机上"抖动" | std 未设为零/部署时仍在采样 | 部署用确定性推理 $\mu(s)$ |
| C++ 推理速度不够 | 未启用 GPU/TRT backend | 使用 ONNX Runtime CUDA provider |

**ONNX vs libtorch 选型**：

| 维度 | ONNX Runtime | libtorch |
|------|-------------|----------|
| 跨平台 | 优秀（ARM/x86/GPU） | 仅 x86/CUDA |
| 推理速度 | 快（TensorRT 后端） | 中等 |
| 自定义算子 | 困难 | 容易 |
| 模型大小 | 小（优化后） | 较大 |
| Jetson 部署 | 原生支持 | 需编译 |

**推荐**：机器人部署首选 ONNX Runtime（简单、快、跨平台）。只在需要自定义前/后处理层时用 libtorch。

---

## §16 策略梯度在 LLM 对齐中的应用 ⭐⭐⭐

### RLHF 中的 PPO

ChatGPT 的对齐训练本质就是策略梯度：

1. **策略** = LLM 的下一词预测分布 $\pi_\theta(\text{token}|s)$
2. **状态** = 当前对话上下文
3. **动作** = 生成的下一个 token（离散动作空间）
4. **奖励** = Reward Model 的打分（从人类偏好训练得来）

**RLHF 的 PPO 与机器人 PPO 的区别**：

| 维度 | 机器人 PPO | RLHF PPO |
|------|-----------|----------|
| 动作空间 | 连续（关节扭矩） | 离散（token vocabulary ~32k-128k） |
| 策略网络 | 小 MLP（2-4 层） | 巨型 Transformer（数十亿参数） |
| 环境 | 物理仿真器 | 自回归生成 + Reward Model |
| KL 约束 | clip $\varepsilon=0.2$ | 额外 KL 惩罚项（与 SFT 模型的 KL） |
| 并行度 | 4096 环境 | 数千 prompt 并行 |

**为什么 RLHF 也选 PPO？** 同样的原因：稳定、简单、对大 batch 友好。

**DPO (Direct Preference Optimization)**：2023 年提出的 PPO 替代方案。核心观察：RLHF 的 "reward model + PPO" 两阶段可以合并为一个闭式损失函数——直接在偏好数据上优化策略，无需训练显式 reward model。

$$
L_{\text{DPO}}(\theta) = -\mathbb{E}_{(x, y_w, y_l)}\left[\log\sigma\left(\beta\log\frac{\pi_\theta(y_w|x)}{\pi_{\text{ref}}(y_w|x)} - \beta\log\frac{\pi_\theta(y_l|x)}{\pi_{\text{ref}}(y_l|x)}\right)\right]
$$

其中 $y_w$ 是偏好的回答，$y_l$ 是不偏好的回答。这等价于在 KL 约束下最大化隐式 reward——数学上与 PPO + reward model 等价，但实现更简单。

**对机器人 RL 的启示**：DPO 式的"preference-based RL"正在向机器人领域渗透——用人类对机器人行为的偏好排序替代手工设计的 reward function。这可能是下一个范式转变。

---

## 与后续专题的桥梁

### 向后指向

- **6.3（TD 学习与函数逼近）**：本章的 critic 训练用均方 TD 误差一带而过；6.3 专讲 TD(λ)、Baird 反例、致命三元组、GTD/ETD 等收敛性修复——**解释 Actor-Critic 的 critic 何时稳定**。

  回顾本章：我们在 §6 中提到 Critic 使用 MSE + TD target 训练，但从未讨论"这个训练什么条件下收敛"。6.3 给出答案：当使用线性函数近似 + off-policy 数据 + bootstrapping 三者同时出现时，TD 可能发散（Baird 1995 反例）。这就是著名的"致命三元组"。

- **6.4（LQR 策略梯度 + SAC-HJB 视角）**：Fazel 2018 的 LQR PG 全局收敛证明；连续时间 HJB 与 soft Bellman 的关系。

  回顾本章：§13 中我们提到 LQR 策略梯度虽然目标非凸，但有全局收敛保证。6.4 将给出完整证明，并展示连续时间极限下 SAC 的 soft Bellman 方程如何变为 HJB 方程——把 RL 与最优控制理论完全桥接。

- **6.5（随机逼近与收敛性）**：Robbins-Monro、ODE method、two-timescale SA——**Actor-Critic 正是典型 two-timescale 系统**，收敛性分析归约为 Borkar 的 SA 理论。

  回顾本章：§6.3 中我们说 Actor-Critic 满足两时间尺度条件时收敛。6.5 将给出严格证明：定义两个 ODE，证明快变量在慢变量几乎不变时收敛到平衡态，然后慢变量在"平均化"梯度场下收敛。

### 向前回顾

- **与 6.1 的关系**：6.1 教了"环境结构已知或可枚举时怎么解"（PI/VI），本章教了"只有采样器时怎么优化"（PG）。两者是对偶关系。
- **Actor-Critic = 可微版 PI**：Critic 做 policy evaluation（对应 PI 的第一步），Actor 做 policy improvement（对应 PI 的第二步）。区别：PI 精确求解，AC 用梯度近似。

---

## §17 总结与核心 Take-Away

**策略梯度定理是一个惊人的数学对象**——它把一个看似需要对 $\mathcal{A}$ 做组合优化的问题（策略改进）转化成了一个可在参数空间做梯度上升的连续优化问题，并给出了一个严格无偏的 Monte Carlo 估计（REINFORCE）。这扇门打开后，过去三十年的 RL 进展几乎全部是在"如何降这个梯度估计的方差、如何控制每步更新的步长、如何添加熵/KL 正则"上的变奏。

**三个核心 take-away**：

1. **PG 定理 + GAE + clipped surrogate = 现代 sim-to-real 的全部数学内核**。Rudin 2022 之后所有四足/人形工作，算法层面几乎是同一个 PPO，区别在 reward shaping、domain randomization 和 observation space 设计。

2. **NPG 与 trust region 的几何视角是理解"为什么 PPO 稳定"的关键**。Fisher 信息矩阵定义了策略分布空间的 Riemannian 结构；自然梯度对策略本身敏感（而非参数化方式）。TRPO 加 KL 约束保证单调改进；PPO 用 clip 做柔性 trust region。

3. **最大熵 RL 把 RL 与变分推断、path integral control、linearly-solvable MDP 统一**。SAC 的成功不是孤立的工程 hack，它背后是 Todorov 2007 以来的深刻控制理论结构。

---

## 附录 0：完整数值示例（Worked Example）

### 一维高斯策略梯度的闭式分析

**设置**：$\pi_\theta(a) = \mathcal{N}(\theta, 1)$，$r(a) = -a^2$。这是最简单的可以完全解析的策略梯度问题。

**Step 1：计算期望回报**

$$
J(\theta) = \mathbb{E}_{a\sim\mathcal{N}(\theta,1)}[-a^2] = -\mathbb{E}[a^2] = -(\text{Var}(a) + (\mathbb{E}[a])^2) = -(1 + \theta^2)
$$

**Step 2：真实梯度**

$$
\nabla_\theta J(\theta) = -2\theta
$$

最优解：$\theta^* = 0$（策略均值在原点，使 $-a^2$ 期望最大化为 $-1$）

**Step 3：REINFORCE 梯度估计**

score function：$\nabla_\theta\log\pi_\theta(a) = \nabla_\theta[-\frac{(a-\theta)^2}{2}] = a - \theta$

REINFORCE 估计：$\hat g = (a-\theta) \cdot (-a^2)$，其中 $a\sim\mathcal{N}(\theta,1)$

验证无偏：$\mathbb{E}[\hat g] = \mathbb{E}[(a-\theta)(-a^2)] = -\mathbb{E}[a^3 - \theta a^2]$

设 $\epsilon = a-\theta \sim \mathcal{N}(0,1)$，$a = \theta+\epsilon$：

$$
\mathbb{E}[\hat g] = -\mathbb{E}[\epsilon(\theta+\epsilon)^2] = -\mathbb{E}[\epsilon\theta^2 + 2\epsilon^2\theta + \epsilon^3]
$$
$$
= -(\theta^2\cdot 0 + 2\theta\cdot 1 + 0) = -2\theta = \nabla_\theta J \quad\checkmark
$$

**Step 4：REINFORCE 梯度的方差**

$$
\text{Var}(\hat g) = \mathbb{E}[\hat g^2] - (\mathbb{E}[\hat g])^2 = \mathbb{E}[(a-\theta)^2 a^4] - 4\theta^2
$$

设 $\epsilon = a-\theta$：$\mathbb{E}[\epsilon^2(\theta+\epsilon)^4]$。展开（利用高斯矩）：

$$
= \mathbb{E}[\epsilon^2]\cdot\theta^4 + 6\theta^2\cdot\mathbb{E}[\epsilon^4] + \mathbb{E}[\epsilon^6] + \ldots = \theta^4 + 6\theta^2\cdot 3 + 15 + \ldots
$$

对于 $\theta=1$：$\text{Var}(\hat g) \approx 1 + 18 + 15 - 4 = 30$。方差很大！

**Step 5：使用 baseline $b = V^\pi = J(\theta) = -(1+\theta^2)$**

新估计：$\hat g_b = (a-\theta)(-a^2 - (-(1+\theta^2))) = (a-\theta)((1+\theta^2) - a^2)$

$= (a-\theta)(1+\theta^2-(\theta+\epsilon)^2) = \epsilon(1+\theta^2-\theta^2-2\theta\epsilon-\epsilon^2)$

$= \epsilon(1-2\theta\epsilon-\epsilon^2)$

方差：$\text{Var}(\hat g_b) = \mathbb{E}[\epsilon^2(1-2\theta\epsilon-\epsilon^2)^2] - 4\theta^2$

对于 $\theta=1$：$\text{Var}(\hat g_b) \approx 8 - 4 = 4$。方差从 30 降到 4——**降低约 7.5 倍**。

> 这个具体数值示例证明了 baseline 减方差的实际效果。在高维问题中，方差降低倍数会更大。

---

## 附录 A：关键定理清单

| 编号 | 定理 | 出处 | 地位 |
|------|------|------|------|
| 6.2.1 | Policy Gradient Theorem | Sutton et al. NeurIPS 1999 | 所有 PG 的根 |
| 6.2.2 | REINFORCE 无偏性 | Williams ML 1992 | Monte Carlo PG 基石 |
| 6.2.3 | Baseline 不引入偏差 | Williams 1992 | 方差缩减合法性 |
| 6.2.4 | Compatible Function Approximation | Sutton 1999 | Actor-Critic 无偏性 |
| 6.2.5 | NPG = 预条件 GD（$F^{-1}$）| Kakade NeurIPS 2001 | NPG/TRPO 几何基础 |
| 6.2.6 | Monotonic Improvement | Schulman ICML 2015 | TRPO 核心 |
| 6.2.7 | Deterministic Policy Gradient | Silver ICML 2014 | DDPG/TD3 根 |
| 6.2.8 | Soft Bellman 算子 $\gamma$-压缩 | Haarnoja ICML 2017 | SAC 收敛性 |
| 6.2.9 | 最优策略 = Boltzmann | Ziebart/Haarnoja | Energy-based policy |

---

## 附录 B：经典论文时间线

| 年份 | 作者 | 论文 | 贡献 |
|------|------|------|------|
| 1992 | Williams | Simple statistical gradient-following algorithms | REINFORCE |
| 1999 | Sutton et al. | Policy Gradient Methods with FA | PG 定理 + compatible FA |
| 2001 | Kakade | A Natural Policy Gradient | Fisher 几何、NPG |
| 2002 | Kakade & Langford | Approximately Optimal Approximate RL | Performance difference lemma |
| 2014 | Silver et al. | Deterministic Policy Gradient | DPG 定理 |
| 2015 | Schulman et al. | Trust Region Policy Optimization | TRPO |
| 2016 | Schulman et al. | High-Dim. Continuous Control via GAE | GAE |
| 2016 | Lillicrap et al. | Continuous Control with Deep RL | DDPG |
| 2017 | Schulman et al. | Proximal Policy Optimization | PPO |
| 2018 | Haarnoja et al. | Soft Actor-Critic | SAC |
| 2018 | Fujimoto et al. | Addressing FA Error in AC | TD3 |
| 2018 | Fazel et al. | Global Convergence of PG for LQR | 非凸 PG 全局收敛 |
| 2021 | Agarwal-Kakade-Lee-Mahajan | On the Theory of PG Methods | 全局收敛理论 |
| 2022 | Rudin et al. | Learning to Walk in Minutes | 大规模并行 PPO |
| 2024 | Radosavovic et al. | Real-World Humanoid Locomotion | 人形 PPO + Transformer |

---

## 附录 C：C++/Python 库对照

| 库 | Stars | 核心算法 | 适用场景 |
|---|---|---|---|
| **stable-baselines3** | ~11k | PPO/SAC/TD3 | 参考级正确性 |
| **rl_games** | ~1.5k | PPO | Isaac Lab 默认 |
| **CleanRL** | ~9k | PPO/SAC/TD3 | 单文件研究 |
| **Isaac Lab** | ~5k | PPO/SAC | 大规模机器人训练 |
| **RSL-RL** | ~2k | PPO | 四足标准 |
| **Spinning Up** | ~11k | VPG/PPO/SAC | 教学必读 |

**库选型建议**：
- 教学/读代码：Spinning Up + CleanRL
- 研究 baseline：SB3
- 四足/人形训练：Isaac Lab + rl_games / RSL-RL
- 操纵任务：SB3-SAC 或 TorchRL-SAC
- 部署到真机 C++：ONNX Runtime 优先，libtorch 备选

---

## 附录 D：跨章综合题

**综合练习 1**（需要 6.1 + 6.2 知识）：

设 MDP 只有 2 个状态 $\{s_1, s_2\}$，2 个动作 $\{a_1, a_2\}$。策略为 tabular-softmax。
a) 写出 PG 定理在此 2-state MDP 上的具体展开形式。
b) 利用 6.1 的策略迭代证明：PG 的不动点包含 PI 的不动点。
c) 讨论：在什么条件下 PG 可能有 PI 不具有的不动点（局部最优）？

**综合练习 2**（需要 6.2 + 6.5 随机逼近知识预览）：

Actor-Critic 的两时间尺度分析：
a) 写出 AC 的两个更新方程（ODE 形式）。
b) 解释 $\alpha_\theta/\alpha_\phi \to 0$ 的物理含义。
c) 这对 PPO 中 actor lr / critic lr 的比例选择有什么指导？

---

## 附录 D2：自测题目

1. 请用 Neumann 级数法从头推导 PG 定理，并明确指出"初始分布 $\mu_0$ 项为何消失"。
2. 证明：用 $V^\pi(s)$ 作为 baseline 不引入梯度偏差；并给出最小方差 baseline 的闭式表达。
3. 推导 GAE 恒等式 $\hat A_t^{\text{GAE}} = (1-\lambda)\sum_n\lambda^{n-1}\hat A_t^{(n)}$，并讨论 $\lambda=0,1$ 极端。
4. 证明 Soft Bellman 算子在 $\|\cdot\|_\infty$ 下 $\gamma$-压缩。提示：LSE 的 Lipschitz 常数为 1。
5. 设 $\pi_\theta(a)=\mathcal N(\theta,1)$，$r(a)=-a^2$。计算 $\nabla_\theta J(\theta)$ 的闭式解与 REINFORCE 方差；使用 $V^\pi$ 作 baseline 时方差降低多少？
6. TRPO 中 $C=4\varepsilon\gamma/(1-\gamma)^2$ 的 $1/(1-\gamma)^2$ 因子从何而来？
7. 解释 PPO-Clip 的非对称性为何合理；若做对称 clip 会发生什么？
8. 推导 DPG 定理，指出从 $\sigma\to 0$ 极限中支配收敛定理在哪一步用到。

---

## 附录 E：时间预算与学习建议

**总时间**：基础（⭐⭐以下）约 3-4 周；完整（含⭐⭐⭐）额外 2-3 周。按 20h/周 估算。

| 任务 | 时间 | 产出物 |
|------|------|--------|
| §1-3 PG 定理阅读 + 亲手推三遍 | 3 天 | 推导笔记（含 Neumann 展开） |
| §4-5 REINFORCE + GAE 复现 | 3 天 | CleanRL 逐行精读 |
| §6 A2C CartPole 从零实现 | 4 天 | 200 行 PyTorch 代码 |
| §9 PPO in MuJoCo Hopper 调通 | 3 天 | 500k 步达 baseline 80% |
| §7-8 NPG/TRPO 手推 | 3 天 | Schulman 博士论文 Ch.3-4 |
| §10 TD3 复现 | 3 天 | CleanRL td3 精读 |
| §11 SAC 复现 + 最大熵数学 | 4 天 | SAC on Humanoid |
| §12 Isaac Lab 训练 ANYmal | 5 天 | sim-to-real 策略 |
| §15 ONNX 导出 + C++ 推理 | 3 天 | libtorch 推理 demo |

---

## 附录 F：免费 PDF / 在线教材

- **Sutton & Barto 2018** —— http://incompleteideas.net/book/the-book-2nd.html
- **Agarwal-Jiang-Kakade-Sun *RL: Theory and Algorithms*** —— https://rltheorybook.github.io/
- **Schulman 博士论文** —— https://www2.eecs.berkeley.edu/Pubs/TechRpts/2016/EECS-2016-217.html
- **Levine 2018 "RL as Inference"** —— https://arxiv.org/abs/1805.00909
- **Meyn 2022 *Control Systems and RL*** —— https://meyn.ece.ufl.edu/control-systems-and-reinforcement-learning/

---

## 附录 F：视频课程

- **UC Berkeley CS285（Sergey Levine）** —— https://rail.eecs.berkeley.edu/deeprlcourse/
- **David Silver UCL/DeepMind RL** —— https://www.deepmind.com/learning-resources
- **Stanford CS234（Emma Brunskill）** —— https://web.stanford.edu/class/cs234/
- **Pieter Abbeel Foundations of Deep RL** —— YouTube 6-Lecture Series

---

## 附录 G：中文资源

- **Easy RL 蘑菇书**（Datawhale）—— https://datawhalechina.github.io/easy-rl/ （入门首选）
- **王树森 Deep RL** —— https://github.com/wangshusen/DRL （Bilibili 同步视频）
- **周博磊 强化学习纲要** —— https://github.com/zhoubolei/introRL （B 站视频）
- **李宏毅 DRL 2018/2023** —— Bilibili 搜"李宏毅 Deep Reinforcement Learning"
- **B 站白板推导系列** 强化学习章节（shuhuai008 等 up 主）
- **知乎专栏**：旺财的搬砖历险记、Frank-Tian RL 笔记等
- **动手学强化学习** —— https://hrl.boyuai.com/ （含 PPO/SAC 完整代码）

## 附录 G2：开源代码仓库

| 仓库 | 链接 | 推荐用途 |
|------|------|---------|
| CleanRL | https://github.com/vwxyzjn/cleanrl | 单文件精读 |
| Stable-Baselines3 | https://github.com/DLR-RM/stable-baselines3 | 参考实现 |
| Spinning Up | https://github.com/openai/spinningup | 教学代码 |
| rl_games | https://github.com/Denys88/rl_games | Isaac Lab 配套 |
| RSL-RL | https://github.com/leggedrobotics/rsl_rl | 四足标准 |
| Isaac Lab | https://github.com/isaac-sim/IsaacLab | 大规模训练 |
| walk-these-ways | https://github.com/Improbable-AI/walk-these-ways | Go1 部署 |
| TorchRL | https://github.com/pytorch/rl | 官方分布式 |

---

---

## 附录 H：社区与论坛

- **r/reinforcementlearning** —— https://www.reddit.com/r/reinforcementlearning/
- **NVIDIA Isaac Lab Discussions** —— https://github.com/isaac-sim/IsaacLab/discussions
- **CleanRL Discord** —— 实现细节讨论
- **Farama Foundation Discord** —— Gymnasium 维护者社区

---

## 附录 I：策略梯度的 14 个关键公式速查

为方便回顾，列出本章所有核心公式：

**1. PG 定理**：$\nabla_\theta J = \frac{1}{1-\gamma}\mathbb{E}_{d^\pi, \pi}[\nabla_\theta\log\pi_\theta(a|s) \cdot Q^\pi(s,a)]$

**2. Advantage 形式**：$\nabla_\theta J = \frac{1}{1-\gamma}\mathbb{E}_{d^\pi,\pi}[\nabla_\theta\log\pi \cdot A^\pi]$

**3. REINFORCE**：$\hat g = \sum_t \nabla_\theta\log\pi(a_t|s_t) \cdot \hat G_t$

**4. Baseline 无偏性**：$\mathbb{E}_{a\sim\pi}[\nabla_\theta\log\pi \cdot b(s)] = 0$

**5. GAE**：$\hat A_t^{\text{GAE}} = \sum_{l=0}^\infty (\gamma\lambda)^l \delta_{t+l}$

**6. TD 残差**：$\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$

**7. Fisher 矩阵**：$F = \mathbb{E}[\nabla_\theta\log\pi \cdot \nabla_\theta\log\pi^\top]$

**8. 自然梯度**：$\tilde\nabla J = F^{-1}\nabla_\theta J$

**9. KL 二阶近似**：$D_{\text{KL}}(\pi_\theta\|\pi_{\theta+\Delta}) \approx \frac{1}{2}\Delta^\top F \Delta$

**10. TRPO 解**：$\Delta\theta^* = \sqrt{2\delta/(g^\top F^{-1}g)} \cdot F^{-1}g$

**11. PPO-Clip**：$L^{\text{CLIP}} = \mathbb{E}[\min(\rho\hat A, \text{clip}(\rho, 1{\pm}\varepsilon)\hat A)]$

**12. DPG**：$\nabla_\theta J = \frac{1}{1-\gamma}\mathbb{E}_{d^{\mu_\theta}}[\nabla_\theta\mu_\theta(s) \cdot \nabla_a Q|_{a=\mu(s)}]$

**13. Soft Bellman**：$(\mathcal{T}^*Q)(s,a) = r + \gamma\mathbb{E}_{s'}[\alpha\log\sum_{a'}\exp(Q(s',a')/\alpha)]$

**14. SAC Actor Loss**：$J_\pi = \mathbb{E}[\alpha\log\pi_\phi(a|s) - Q_\theta(s,a)]$

---

*本章至此结束。下一专题：6.3 TD 学习、函数逼近与致命三元组。*
