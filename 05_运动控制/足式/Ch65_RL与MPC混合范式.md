> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 65 章 RL + MPC 混合范式——MPC-Net / Cafe-MPC VWBC / Residual RL / Differentiable MPC

> **难度**：⭐⭐⭐~⭐⭐⭐⭐ | **建议时间**：1.5 周（25-30 小时） | **前置**：Ch53（WBC/TSID）、Ch54（DDP/Crocoddyl）、Ch55（OCS2）、Ch63-64（RL 训练+部署）

> **一句话概要**：纯 MPC 和纯 RL 各有致命短板——MPC 感知理解弱、推理慢，RL 约束满足弱、可解释性差。本章系统讲解四条主流混合路线（蒸馏、值函数嵌入、分层、残差）以及两个前沿方向（可微 MPC、世界模型），从数学推导到工程选型，帮你在 RL+MPC 的连续光谱中找到自己的研究定位。

---

## 前置自测

📋 **答不出 >= 2 题 → 先回对应章节复习**

1. **[Ch54]** DDP 的 backward pass 中，$Q$-function 的二阶展开 $\delta Q = Q_x \delta x + Q_u \delta u + \frac{1}{2}[\delta x^T, \delta u^T] \begin{bmatrix} Q_{xx} & Q_{xu} \\ Q_{ux} & Q_{uu} \end{bmatrix} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}$ 中，$Q_{uu}$ 的物理含义是什么？为什么 $Q_{uu} \succ 0$ 是 DDP 能求解的必要条件？

2. **[Ch55]** OCS2 的 SQP-RTI 只做 1 次 SQP 迭代就输出控制——为什么这样做仍然稳定？Real-Time Iteration 的收敛性保证来自哪里？

3. **[Ch53]** WBC 的加权 QP 和分层 HQP 的核心区别是什么？为什么加权 QP 中 $w_1 = 100, w_2 = 1$ 不能保证 Task 1 绝对优先？

4. **[Ch63]** PPO 的 clipping 机制 $\min(r_t(\theta) A_t, \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon) A_t)$ 为什么能防止策略更新过大？$\epsilon$ 通常取什么值？

5. **[Ch64]** TorchScript 的 `torch.jit.trace` 和 `torch.jit.script` 各适合什么场景？为什么腿足 RL 策略通常用 `trace`？

---

## 本章目标

学完本章，你应能：
1. **从数学层面理解**纯 MPC（在线优化）和纯 RL（离线优化+在线推理）的本质区别，以及为什么混合是必要的研究方向
2. **推导 MPC-Net 的完整 loss 函数**——包括 $Q$-function 的 Taylor 展开、为什么不只是 MSE、以及 OCS2 的开源实现
3. **理解 VWBC 的数学形式化**——值函数梯度如何嵌入 WBC 的 QP，消除手动调参
4. **分析 Residual RL 的稳定性**——为什么限制残差范围至关重要，收敛性如何保证
5. **推导 Differentiable MPC 的 KKT 隐式微分**——端到端训练 MPC 参数的数学基础
6. **能用决策树为具体工程任务选择最合适的混合路线**

---

## 65.1 纯 MPC 与纯 RL：两种控制范式的数学本质 ⭐⭐

> **本节解决什么问题**：在讨论"混合"之前，必须先从数学层面理解 MPC 和 RL 各自在做什么、擅长什么、不擅长什么。这不是简单的列表对比，而是从最优控制理论的角度统一两者。

### 动机：同一个问题的两种解法

假设你有一只四足机器人，需要在崎岖地形上行走。无论用 MPC 还是 RL，你都在解同一个问题——**顺序决策**：

$$\min_{\pi} \mathbb{E}\left[\sum_{t=0}^{T} c(s_t, a_t)\right] \quad \text{s.t.} \quad s_{t+1} = f(s_t, a_t), \quad g(s_t, a_t) \leq 0$$

但两者的**求解策略**截然不同。

### MPC：在线优化

MPC 的核心思想是**每个控制周期都从头求解一个有限时域优化问题**：

$$\min_{u_0, \ldots, u_{N-1}} \sum_{k=0}^{N-1} l_k(x_k, u_k) + l_N(x_N)$$

$$\text{s.t.} \quad x_{k+1} = f(x_k, u_k), \quad g(x_k, u_k) \leq 0, \quad x_0 = x_{\text{current}}$$

求解后只执行第一步 $u_0^*$，然后下一个周期用新的状态 $x_{\text{current}}$ 重新求解。这就是 Ch55 中 OCS2 做的事情。

**MPC 的数学特性**：

| 特性 | 描述 | 工程含义 |
|------|------|---------|
| **在线求解** | 每个周期都解 NLP/QP | 推理时间 10-50 ms，需要强 CPU |
| **模型依赖** | 需要显式动力学模型 $f$ | 模型不准则控制不好 |
| **约束原生** | 约束 $g$ 直接嵌入优化 | 可以保证"绝对不碰墙" |
| **有限时域** | 只看未来 $N$ 步 | 长期最优性不保证 |
| **确定性** | 通常假设确定性动力学 | 随机扰动需要鲁棒化 |

### RL：离线优化 + 在线推理

RL 的核心思想是**离线训练一个策略网络 $\pi_\theta(a|s)$，部署时直接前向推理**：

**训练阶段**（离线）：

$$\theta^* = \arg\max_\theta \mathbb{E}_{\tau \sim \pi_\theta} \left[\sum_{t=0}^{T} \gamma^t r(s_t, a_t)\right]$$

通过与环境的数百万次交互，用 PPO / SAC 等算法迭代更新 $\theta$。

**部署阶段**（在线）：

$$a_t = \pi_{\theta^*}(s_t) \quad \text{（一次前向传播，100 $\mu$s）}$$

**RL 的数学特性**：

| 特性 | 描述 | 工程含义 |
|------|------|---------|
| **离线训练** | 训练时大量交互 | 需要仿真器，训练 24-72 小时 |
| **在线推理** | 部署时只做前向传播 | 推理 50-200 $\mu$s，极快 |
| **无模型** | 不需要显式 $f$ | 对模型误差鲁棒 |
| **无限时域** | 通过 $\gamma$ 折扣考虑全局 | 长期行为更优 |
| **约束困难** | 奖励设计间接处理约束 | 无法保证硬约束满足 |

### 两者的深层对偶关系

从最优控制理论看，MPC 和 RL 其实在解**同一个 Bellman 方程**的不同近似：

$$V^*(s) = \min_a \left[c(s, a) + \gamma V^*(f(s, a))\right]$$

- **MPC** 通过有限时域展开近似 $V^*$：把 $V^*(x_N)$ 用终端代价 $l_N$ 近似，然后向前展开 $N$ 步
- **RL** 通过参数化函数近似 $V^*$：训练 $V_\theta(s) \approx V^*(s)$，通过大量经验数据拟合

这个视角揭示了一个关键洞察：**MPC 的优化器提供了 $V^*$ 的高质量局部近似（在当前状态附近），而 RL 提供了 $V^*$ 的全局近似（覆盖整个状态空间）**。混合两者的动机正是结合这两种近似的优势。

> **本质洞察**：MPC 和 RL 的关系**不是**"传统方法 vs 现代方法"的对立,**而是**同一个 Bellman 方程在不同计算资源约束下的两种近似策略。MPC 把计算预算花在"此刻此地"(在线求解当前状态附近的局部最优),RL 把计算预算花在"事前准备"(离线遍历整个状态空间训练全局策略)。两者的计算总量可能相当——只是分配在时间轴上的位置不同。

### 纯 MPC 的五大短板

1. **感知理解弱**：原始高程图（200x200 浮点矩阵）或 RGB 图像难以嵌入代价函数。OCS2 的代价函数需要解析梯度（CppAD 自动微分），但 CNN 特征的梯度对 MPC 求解器不友好——高维非凸

2. **非凸问题探索难**：跳越障碍、翻滚等需要跨越多个局部极小值。MPC 用的 SQP/DDP 是局部求解器（Ch54），需要好的初始猜测，而好的初始猜测本身就是最难的部分

3. **调参痛苦**：Ch53 讲过 WBC 的权重调节极其痛苦——每加一个任务要调权重，权重之间相互耦合。MPC 的代价函数权重面临同样的问题，且更严重（因为 MPC 的 horizon 更长，权重的时间依赖性增加了搜索空间）

4. **推理慢**：OCS2 的 SQP-RTI 单次迭代约 10-20 ms（Ch55），这对 100 Hz MPC 已是极限。如果用更复杂的模型（全身动力学而非质心动力学），求解时间可能超过 50 ms，无法实时

5. **对模型精度敏感**：MPC 的控制质量直接取决于动力学模型 $f$ 的准确性。地面摩擦系数估计不准？质心惯量标定有误？致动器模型简化过度？这些都会导致 MPC 性能下降

### 纯 RL 的五大短板

1. **约束满足弱**：RL 通过奖励惩罚间接处理约束，不能保证"绝对不碰墙"或"关节扭矩不超限"。即使加大惩罚系数，也只是降低违约概率，不能消除。对于安全关键场景（如在人群中行走），这不可接受

2. **可解释性差**：策略网络是黑盒——3 层 MLP 有 ~10000 个参数，无法回答"为什么这一步向左迈"。当机器人摔倒时，MPC 可以检查约束违反和代价值，RL 只能看 reward log，调试极其困难

3. **泛化到新机器人难**：每个机器人（Go2 vs ANYmal vs Spot）需要重新训练。即使同一机器人换了负载或关节磨损，策略可能需要微调。MPC 只需更新 URDF 参数

4. **需要大量仿真数据**：PPO 训练一个四足 trot 策略需要 $10^8$-$10^9$ 步交互（Ch63），即使用 IsaacGym 的 4096 并行环境也需要 24-72 小时 GPU 时间

5. **Sim-to-Real Gap**：Domain Randomization（Ch63）能缓解但不能消除仿真与真实之间的差距。地面接触动力学（摩擦、弹性、阻尼）在仿真中的精度有限，尤其是软地面和湿滑表面

### 混合范式的动机

上述分析揭示了一个关键互补性：

```
MPC 擅长的 ←→ RL 不擅长的
━━━━━━━━━━━━━━━━━━━━━━━━━━━
约束满足      ←→  约束处理弱
可解释性      ←→  黑盒
模型换算快    ←→  需要重训
精细跟踪      ←→  精度有限

RL 擅长的 ←→ MPC 不擅长的
━━━━━━━━━━━━━━━━━━━━━━━━━━━
感知理解      ←→  难以嵌入高维感知
快速推理      ←→  在线求解慢
全局探索      ←→  局部求解器
数据驱动      ←→  模型依赖
```

**混合范式的核心思想**：让 RL 处理它擅长的（感知、探索、快速推理），让 MPC 处理它擅长的（约束、精细跟踪、可解释）。

> **跨领域类比**：RL 与 MPC 的混合，类似于人类决策中"直觉"与"推理"的协作。诺贝尔奖得主 Kahneman 将人类认知分为系统 1（快速直觉，对应 RL 的亚毫秒推理）和系统 2（慢速推理，对应 MPC 的在线优化）。日常行走你不需要"计算"每一步（系统 1/RL 就够了），但走钢丝时你必须"思考"每一步（需要系统 2/MPC 来保证约束满足）。四条混合路线本质上是在设计系统 1 和系统 2 之间不同的协作协议。

### 四条主流混合路线

本章接下来将深入讲解这四条路线，每条都有明确的数学基础和工程实现：

```
┌───────────────────────────────────────────────────┐
│ 路线 A：MPC 蒸馏 → RL（MPC-Net, 65.2）           │
│   训练：先跑 MPC 生成专家数据，再训 NN 模仿         │
│   部署：只用 NN（快，100 $\mu$s）                  │
│   数学核心：Q-function Taylor 展开指导学习          │
└───────────────────────────────────────────────────┘
┌───────────────────────────────────────────────────┐
│ 路线 B：RL 学习 MPC 内部（Cafe-MPC/VWBC, 65.3）  │
│   训练：让 RL 学 value function 替代 WBC 权重      │
│   部署：V 网络 + WBC + MPC 三级联动               │
│   数学核心：V(s) 梯度嵌入 WBC 的 QP               │
└───────────────────────────────────────────────────┘
┌───────────────────────────────────────────────────┐
│ 路线 C：RL 高层 + MPC 低层（DTC, 65.4）           │
│   RL 输出参考轨迹，MPC 跟踪执行                    │
│   部署：两者并存                                   │
│   数学核心：分层最优控制                            │
└───────────────────────────────────────────────────┘
┌───────────────────────────────────────────────────┐
│ 路线 D：MPC 主 + RL 残差（Residual RL, 65.5）     │
│   MPC 输出基础动作，RL 学习"修正"                   │
│   部署：action = a_MPC + a_RL                     │
│   数学核心：残差策略梯度                            │
└───────────────────────────────────────────────────┘
```

> **过渡**：理解了"为什么要混合"之后，我们从最早的混合工作——MPC-Net 开始，看看 ETH RSL 如何把 MPC 的知识"蒸馏"到神经网络中。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为 RL 和 MPC 是对立的两种方法**
> 新手想法："RL 和 MPC 是两个阵营，选一个就行了"
> 实际上：它们是同一个 Bellman 方程的不同近似，天然互补。MPC 提供局部精确解，RL 提供全局近似解。混合是趋势，不是折衷。
> 正确思维：把 MPC-RL 看成一个连续光谱，根据具体需求在光谱上选点。

> ⚠️ **思维陷阱：认为"RL 更先进所以应该取代 MPC"**
> 新手想法："深度学习这么强，MPC 是传统方法，迟早被淘汰"
> 实际上：截至 2026 年，ANYbotics、Boston Dynamics、Unitree 的生产代码仍然以 MPC 为主，RL 作为辅助。原因是：(1) 客户不接受黑盒决策（安全认证）；(2) MPC 的 30 年工程经验积累无法短期替代；(3) RL 的可靠性还不够生产级别。
> 正确思维：做能被工业接受的混合范式——不只是 paper，还要考虑部署可行性。

> ⚠️ **编程陷阱：MPC 和 RL 用不同的状态表示**
> 错误做法：MPC 用 $[q, \dot{q}]$（广义坐标），RL 用 $[\text{body\_vel}, \text{joint\_pos}, \text{joint\_vel}]$（归一化观测），直接拼接
> 现象：混合系统性能远不如任一单独系统
> 根本原因：两者的状态空间定义不同。MPC 用世界系坐标，RL 用局部归一化观测。必须设计统一的状态表示或显式转换层
> 正确做法：在 MPC 和 RL 之间设计明确的接口——定义共享状态向量和坐标系变换

### 练习

1. **[数学推导]** 用 Bellman 方程证明：当 MPC 的 horizon $N \to \infty$ 且模型完全准确时，MPC 等价于 RL 的最优策略。在有限 $N$ 时，MPC 的次优性来自哪里？（提示：终端代价 $l_N$ 与 $V^*$ 的差距）

2. **[工程分析]** 假设你的四足机器人需要在工厂车间导航（有人类、叉车、地面杂物）。列出 MPC 和 RL 各自的优劣势，给出你的混合方案建议（需要明确接口定义）。

3. **[开放题]** "约束满足"是 MPC 相对 RL 的核心优势。但近年来有 Constrained Policy Optimization（CPO, 2017）和 Safety Layer 等 safe RL 方法。查阅相关论文，分析这些方法能否替代 MPC 的约束处理能力？局限在哪？

---

## 65.2 路线 A：MPC-Net——从 MPC 蒸馏策略网络 ⭐⭐⭐

> **本节解决什么问题**：MPC-Net 是第一个成功把 MPC 的行为"蒸馏"到神经网络中的工作。关键创新不是简单的 behavior cloning，而是利用 MPC 求解器内部的 $Q$-function 信息指导学习。我们将完整推导其 loss 函数。

### 动机：MPC 太慢，但 Behavior Cloning 效果有限

**场景**：你用 OCS2 的 SQP-RTI 为 ANYmal 生成了优秀的 trot 控制——MPC 在 100 Hz 运行，占用 i7 一个核心的 50% 算力。现在你想把这个控制器部署到 Jetson Nano 上（只有 ARM 四核，算力是 i7 的 1/10）。MPC 跑不动了。

最直觉的想法：用 MPC 生成大量 $(s_t, a_t^{\text{MPC}})$ 数据对，训练一个小 MLP 模仿 MPC 的输出。这就是 **Behavior Cloning**（BC）。

### 如果只用 Behavior Cloning 会怎样

BC 的 loss 是简单的 MSE：

$$L_{\text{BC}} = \mathbb{E}_{(s, a^*) \sim D}\left[\|a_{\text{NN}}(s) - a^*\|^2\right]$$

BC 的问题在于**对偏差不敏感**：如果 NN 在某个状态 $s$ 输出 $a_{\text{NN}}$ 偏离 $a^*$ 一点，MSE 只衡量动作空间的欧氏距离，不知道这个偏差对**控制性能**有多大影响。

考虑两种偏差：
- **偏差 A**：在平坦路面上，关节角度偏离 0.01 rad → 几乎无影响
- **偏差 B**：在摩擦锥边界附近，接触力偏离 0.01 N → 可能导致滑动

MSE 认为两者等价（偏差大小相同），但控制代价可能相差 100 倍。

**更严重的问题是分布偏移**（Distribution Shift）：BC 训练时看到的状态来自 MPC 的分布；部署时 NN 的小偏差会让状态偏离训练分布，偏差累积，最终崩溃。这是 Ross & Bagnell (2010) 指出的 DAgger 动机。

### 历史：MPC-Net 的提出

Carius J., Farshidian F., Hutter M. 于 2020 年在 IEEE Robotics and Automation Letters（RA-L）发表 "MPC-Net: A First Principles Guided Policy Search"。这是 ETH RSL 在 OCS2 框架内部实现的工作，代码开源在 `ocs2_mpcnet/` 目录下。

MPC-Net 的核心思想是：**不只学 MPC 的输出动作，还学 MPC 求解器内部的价值信息**。具体来说，利用 DDP backward pass 产生的 $Q$-function 的 Taylor 展开来构造一个更好的 loss。

### MPC-Net 的完整数学推导

**Step 1：回顾 DDP 的 $Q$-function**

Ch54 讲过，DDP 的 backward pass 在每个时间步 $k$ 计算 action-value function $Q_k(x, u)$ 的二阶 Taylor 展开：

$$Q_k(x_k + \delta x, u_k + \delta u) \approx Q_k(x_k, u_k) + \begin{bmatrix} Q_x \\ Q_u \end{bmatrix}^T \begin{bmatrix} \delta x \\ \delta u \end{bmatrix} + \frac{1}{2} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}^T \begin{bmatrix} Q_{xx} & Q_{xu} \\ Q_{ux} & Q_{uu} \end{bmatrix} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}$$

其中：
- $Q_x = l_x + f_x^T V'_x$ 是 $Q$ 对状态的梯度
- $Q_u = l_u + f_u^T V'_x$ 是 $Q$ 对动作的梯度
- $Q_{uu} = l_{uu} + f_u^T V'_{xx} f_u$ 是 $Q$ 对动作的 Hessian
- $V'$ 是下一时刻的 value function

在最优解处，$Q_u = 0$（一阶最优性条件），因此：

$$Q_k(x_k, u_k^* + \delta u) \approx Q_k^* + \frac{1}{2} \delta u^T Q_{uu} \delta u$$

这个展开告诉我们：在最优动作附近，代价的增量由 $Q_{uu}$ 决定——$Q_{uu}$ 编码了"沿每个方向偏离最优动作的代价有多大"。

**Step 2：构造 MPC-Net 的 loss**

MPC-Net 的 loss 包含两项：

$$L = \mathbb{E}_{(s, a^*, Q_{uu}) \sim D}\left[\underbrace{\|a_{\text{NN}}(s) - a^*\|^2}_{L_{\text{BC}}} + \alpha \underbrace{\delta a^T Q_{uu} \delta a}_{L_Q}\right]$$

其中 $\delta a = a_{\text{NN}}(s) - a^*$。

**为什么 $L_Q$ 项很重要？** 它用 $Q_{uu}$ 矩阵对动作偏差进行**加权**。$Q_{uu}$ 编码了"在哪个方向偏差代价最大"的信息：

- 如果某个关节的控制对整体代价影响很大（如支撑腿的髋关节），$Q_{uu}$ 在该方向的特征值大，NN 的偏差会被重罚
- 如果某个关节影响小（如空中摆动腿的末端），$Q_{uu}$ 在该方向的特征值小，NN 可以有更大容错

**几何直觉**：$Q_{uu}$ 定义了动作空间中的一个椭球体。MSE 用的是球体（各方向等权），$Q_{uu}$ 用的是椭球体（沿代价敏感方向压缩）。椭球体比球体更准确地反映了"偏差的真实代价"。

**Step 3：完整 loss 的梯度推导**

将 $\delta a = a_{\text{NN}}(s) - a^*$ 代入，展开 $L_Q$：

$$L_Q = (a_{\text{NN}} - a^*)^T Q_{uu} (a_{\text{NN}} - a^*)$$

$$= a_{\text{NN}}^T Q_{uu} a_{\text{NN}} - 2 a^{*T} Q_{uu} a_{\text{NN}} + a^{*T} Q_{uu} a^*$$

因为 $a^{*T} Q_{uu} a^*$ 是常数（不依赖 $\theta$），对梯度无贡献，所以实际 loss 的梯度为：

$$\nabla_\theta L = \nabla_\theta a_{\text{NN}}^T \left[2(a_{\text{NN}} - a^*) + 2\alpha Q_{uu} (a_{\text{NN}} - a^*)\right]$$

$$= 2\nabla_\theta a_{\text{NN}}^T \left[(I + \alpha Q_{uu})(a_{\text{NN}} - a^*)\right]$$

这个梯度告诉我们：$(I + \alpha Q_{uu})$ 作为**度量矩阵**，在代价敏感方向提供更强的梯度信号。当 $\alpha = 0$ 退化为普通 BC；当 $\alpha \to \infty$ 退化为纯 $Q_{uu}$ 加权学习。

**Step 4：训练流程**

```
OCS2 MPC 求解器（离线）
    │ 生成数据 {(s_t, a_t^*, Q_{uu,t})}
    ▼
PyTorch 训练（Python）
    │ Loss = BC + alpha * Q-weighted
    │ 优化器：Adam, lr=3e-4
    │ 网络：3层MLP [256, 256, 128]
    ▼
TorchScript 导出（.pt）
    │
    ▼
LibTorch C++ 推理（部署）
    │ 推理时间：100 微秒
    ▼
替代 OCS2 MPC（不再需要在线求解）
```

### OCS2 的开源实现

OCS2 在 `ocs2_mpcnet/` 目录下提供了完整的 MPC-Net 实现：

```python
# ocs2_mpcnet/python/mpcnet/algorithm/mpcnet_algorithm_base.py
# 核心训练逻辑（简化）

class MpcnetAlgorithmBase:
    def compute_loss(self, observation, expert_action, Q_uu):
        """
        observation: [batch, obs_dim] — 归一化状态
        expert_action: [batch, act_dim] — MPC 输出
        Q_uu: [batch, act_dim, act_dim] — Q-function Hessian
        """
        predicted_action = self.policy(observation)  # NN 前向
        delta_a = predicted_action - expert_action

        # BC loss
        loss_bc = (delta_a ** 2).sum(dim=-1).mean()

        # Q-weighted loss
        # delta_a: [batch, act_dim] -> [batch, act_dim, 1]
        delta_a_col = delta_a.unsqueeze(-1)
        loss_q = (delta_a_col.transpose(-1, -2) @ Q_uu @ delta_a_col)
        loss_q = loss_q.squeeze().mean()

        return loss_bc + self.alpha * loss_q
```

```cpp
// ocs2_mpcnet/cpp/include/ocs2_mpcnet/MpcnetInterface.h
// C++ 部署接口（简化）

class MpcnetInterface {
 public:
  // 加载 TorchScript 模型
  void loadPolicy(const std::string& model_path);

  // 单次推理（替代 MPC 求解）
  vector_t computeAction(const vector_t& observation) {
    // Eigen VectorXd 是 double，需显式转为 float
    Eigen::VectorXf obs_float = observation.cast<float>();
    torch::Tensor obs_tensor = torch::from_blob(
        obs_float.data(), {1, obs_float.size()}, torch::kFloat32);
    torch::Tensor action = module_.forward({obs_tensor}).toTensor();
    // 推理时间：~100 微秒（MPC 求解：~20 ms）
    float* out_ptr = action.data_ptr<float>();
    vector_t result(action.size(1));
    for (int i = 0; i < result.size(); ++i) result[i] = static_cast<double>(out_ptr[i]);
    return result;
  }

 private:
  torch::jit::script::Module module_;
};
```

### 性能与局限

| 指标 | MPC（OCS2 SQP-RTI） | MPC-Net |
|------|---------------------|---------|
| 推理时间 | 10-20 ms | ~100 $\mu$s |
| 加速比 | 1x | ~200x |
| 训练分布内性能 | 基准 | ~95-99% MPC 水平 |
| 训练分布外 | 自适应（重新求解） | **可能崩溃** |
| 约束满足 | 保证 | **不保证** |
| 计算资源 | i7 单核 50% | Jetson Nano 可运行 |

**关键局限——分布外失效**：MPC-Net 在训练数据覆盖的状态范围内表现接近 MPC，但超出训练分布时（如从未见过的极端扰动），NN 可能输出荒谬的动作。

**工程解决方案——MPC Fallback**：部署时同时运行 MPC-Net 和 MPC（低频）。MPC-Net 输出动作后，用 MPC 检查该动作是否违反约束。如果违反，切换回 MPC 输出。这增加了计算开销但保证了安全性。

### MPC-Net 的训练数据收集与架构细节

**数据收集策略**：MPC-Net 的训练数据质量直接决定蒸馏效果。OCS2 的 `ocs2_mpcnet` 提供了三种数据收集模式：

```python
# 数据收集伪代码
class MPCNetDataCollector:
    def collect_episode(self, init_state):
        states, actions, Q_matrices = [], [], []
        state = init_state

        for t in range(episode_length):
            # 运行 OCS2 MPC（SQP-RTI）
            mpc_solution = self.mpc.solve(state)

            # 收集 MPC 输出和 Q-function 信息
            states.append(state)
            actions.append(mpc_solution.u[0])
            Q_matrices.append(mpc_solution.Q_uu[0])  # DDP backward pass

            # 用 MPC 输出推进仿真
            state = self.simulator.step(state, mpc_solution.u[0])

        return states, actions, Q_matrices
```

| 模式 | 状态分布 | 数据质量 | 覆盖度 |
|------|---------|---------|-------|
| 在线收集（on-policy） | MPC 自然轨迹 | 高（最优策略） | 窄（只覆盖稳定运动） |
| 随机初始化 | 随机状态 + MPC | 中（部分不可行） | 宽 |
| DAgger（推荐） | NN 策略 + MPC 标注 | 高 + 宽 | 最佳 |

**DAgger（Dataset Aggregation）策略的关键思想**：用 MPC-Net 的当前策略执行动作，然后让 MPC 标注"在这个状态下正确的动作应该是什么"。这样训练数据自然覆盖了 NN 策略可能访问到的状态——包括它犯错后到达的偏离状态。DAgger 通常需要 3-5 轮迭代，每轮收集 1000-5000 条轨迹。

**网络架构选择**：

| 架构 | 参数量 | 推理延迟 | 蒸馏效果 | 推荐场景 |
|------|--------|---------|---------|---------|
| MLP [256, 256, 128] | ~100K | ~50 $\mu$s | 良好 | 基线 |
| MLP [512, 256, 128] | ~200K | ~80 $\mu$s | 更好 | 复杂步态 |
| GRU + MLP | ~300K | ~120 $\mu$s | 最好（含时序信息） | 动态运动 |
| Transformer | ~500K+ | ~200 $\mu$s | 未验证 | 研究探索 |

对于腿足 trot 等规律性步态，3 层 MLP 已经足够。只有在需要处理复杂时序决策（如步态切换、recovery 动作）时，才考虑加入 GRU 等记忆结构。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为 MPC-Net 就是 Behavior Cloning**
> 新手想法："不就是模仿 MPC 嘛，和 BC 有什么区别？"
> 实际上：BC 只用 MSE loss，对所有动作维度等权。MPC-Net 用 $Q_{uu}$ 加权 loss，在代价敏感方向施加更大惩罚。这让 MPC-Net 能学到 MPC 的"鲁棒性"（在关键方向保持精确），而不只是"平均行为"。
> 验证方式：训练同一数据集的 BC 和 MPC-Net，在边界状态对比——MPC-Net 在约束边界附近明显优于 BC。

> ⚠️ **编程陷阱：$Q_{uu}$ 的数值条件**
> 错误做法：直接用 DDP 输出的 $Q_{uu}$ 矩阵，不做任何预处理
> 现象：训练 loss 爆炸或 NaN
> 根本原因：$Q_{uu}$ 的条件数可能很大（$10^4$-$10^6$），导致加权 loss 在某些方向梯度爆炸
> 正确做法：对 $Q_{uu}$ 做正则化——加小的对角阵 $Q_{uu} + \epsilon I$（$\epsilon = 10^{-6}$），或者对 $Q_{uu}$ 的特征值做截断

> ⚠️ **思维陷阱：认为 MPC-Net 训练后 MPC 就完全不需要了**
> 新手想法："MPC-Net 推理 100 $\mu$s，太好了，把 MPC 代码删掉吧"
> 实际上：生产系统必须保留 MPC 作为 fallback。MPC-Net 的分布外行为不可预测，安全关键场景必须有 MPC 兜底。ANYbotics 的做法是 MPC-Net 正常运行 + MPC 在后台低频检查。

### 练习

1. **[数学推导]** 证明：当 $\alpha \to 0$ 时 MPC-Net loss 退化为 BC，当 $\alpha \to \infty$ 时 loss 等价于只用 $Q_{uu}$ 加权的 MSE。分析 $\alpha$ 的最优取值策略。

2. **[代码阅读]** 阅读 OCS2 的 `ocs2_mpcnet/python/mpcnet/algorithm/` 目录，回答：(a) 训练数据如何生成？(b) 观测和动作的归一化策略是什么？(c) $Q_{uu}$ 是否做了正则化？

3. **[实验设计]** 设计一个实验对比 BC、MPC-Net 和 DAgger 三种方法在四足 trot 任务上的表现。需要定义：(a) 评估指标（跟踪误差、约束违反率、推理时间）；(b) 测试条件（分布内 vs 分布外）；(c) 数据量敏感性分析。

---

> **过渡**：MPC-Net 把 MPC 的知识压缩到 NN 中，部署时完全替代 MPC。但如果我们不想放弃 MPC 的约束保证呢？下一节的 Cafe-MPC / VWBC 提供了另一种思路：让 RL 学到的 value function 嵌入 WBC，保留 MPC 的所有约束处理能力。

---

## 65.3 路线 B：Cafe-MPC / VWBC——值函数嵌入 WBC ⭐⭐⭐⭐

> **本节解决什么问题**：传统 WBC 的权重调节极其痛苦（Ch53）。VWBC 用 MPC backward sweep 产生的 action-value function 替代手动权重，消除调参的同时保留 WBC 的所有硬约束处理能力。

### 动机：WBC 调参的困境

回顾 Ch53 中 WBC 的代价函数：

$$\min_{\ddot{q}, \lambda, \tau} \sum_{i=1}^{N_{\text{task}}} w_i \|\mathbf{A}_i \ddot{q} - \mathbf{b}_i\|^2$$

其中 $w_i$ 是每个任务的权重。实际工程中，一个典型的四足 WBC 有 5-10 个任务：

| 任务 | 典型权重 | 描述 |
|------|---------|------|
| 质心位置跟踪 | $w_1 = 100$ | 跟踪 MPC 的质心轨迹 |
| 质心速度跟踪 | $w_2 = 50$ | 跟踪 MPC 的质心速度 |
| 躯体姿态跟踪 | $w_3 = 80$ | 保持躯体水平 |
| 摆动腿位置 | $w_4 = 200$ | 跟踪落脚点 |
| 关节正则化 | $w_5 = 1$ | 防止关节偏离默认位置 |
| 角动量正则化 | $w_6 = 10$ | 防止旋转失控 |

**痛苦之处**：

1. **耦合**：调高 $w_1$ 可能导致 $w_4$ 的效果变差（质心跟踪和摆动腿跟踪争夺关节自由度）
2. **场景依赖**：平地 trot 的权重和崎岖地形的权重可能完全不同——平地重视速度跟踪，崎岖地形重视落脚精度
3. **步态依赖**：trot 和 pace 的权重不同——trot 的对角支撑需要更强的姿态控制
4. **搜索空间**：6 个权重，每个 3 个数量级（1-1000），搜索空间 $10^{18}$

### 反面教材：手工调参的典型困局

一个真实的调参日志（来自四足机器人项目）：

```
Day 1:  w = [100, 50, 80, 200, 1, 10] -> 平地 trot OK
Day 2:  加入崎岖地形 -> 摆动腿跟不上，调高 w4=500 -> 质心跟踪崩了
Day 3:  降低 w1=50 -> 质心漂移，机器人慢慢倒
Day 4:  加入 pace 步态 -> 之前调好的权重全不work -> 重新调...
Day 5-15: 循环 Day 2-4
Day 16: 放弃手调，用网格搜索——10^6 种组合，每次仿真 10s
Day 17: 考虑用 RL 学权重...
```

### 历史：Cafe-MPC 和 VWBC 的提出

He Li 和 Patrick M. Wensing 于 2024 年投稿 IEEE Transactions on Robotics（T-RO），2025 年正式见刊："Cafe-MPC: A Cascaded-Fidelity Model Predictive Control Framework with Tuning-Free Whole-Body Control"（Vol. 41, pp. 837-856, 2025）。

**文献勘误**：Cafe-MPC 的作者是 **He Li 和 Patrick M. Wensing**（University of Notre Dame），不是某些二手资料中误写的 "Chignoli et al."。引用该工作时应以 T-RO 论文和作者主页为准。

### VWBC 的数学形式化

**核心思想**：用 MPC backward sweep 产生的 action-value function $Q(x, u)$ 替代 WBC 的手动权重。

**Step 1：MPC 的 $Q$-function**

MPC 求解后，DDP backward pass 产生每个时间步的 $Q$-function：

$$Q(x, u) = l(x, u) + V'(f(x, u))$$

在当前最优解 $(x^*, u^*)$ 附近做 Taylor 展开：

$$Q(x^* + \delta x, u^* + \delta u) \approx Q^* + \begin{bmatrix} Q_x \\ Q_u \end{bmatrix}^T \begin{bmatrix} \delta x \\ \delta u \end{bmatrix} + \frac{1}{2} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}^T \begin{bmatrix} Q_{xx} & Q_{xu} \\ Q_{ux} & Q_{uu} \end{bmatrix} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}$$

**Step 2：从 $Q$-function 到 WBC 代价**

传统 WBC 的代价是手动设定的加权任务误差：

$$J_{\text{WBC}} = \sum_i w_i \|e_i(\ddot{q})\|^2$$

VWBC 的代价直接使用 $Q$-function：

$$J_{\text{VWBC}} = Q(x + \Delta x(\ddot{q}), u(\ddot{q}))$$

其中 $\Delta x(\ddot{q})$ 是加速度 $\ddot{q}$ 导致的状态增量，$u(\ddot{q})$ 是对应的控制输入。

展开为二次形式（在最优解附近）：

$$J_{\text{VWBC}} \approx Q^* + \begin{bmatrix} Q_x \\ Q_u \end{bmatrix}^T \begin{bmatrix} \frac{\partial \Delta x}{\partial \ddot{q}} \\ \frac{\partial u}{\partial \ddot{q}} \end{bmatrix} \delta \ddot{q} + \frac{1}{2} \delta \ddot{q}^T H \delta \ddot{q}$$

其中 $H$ 是复合 Hessian：

$$H = \begin{bmatrix} \frac{\partial \Delta x}{\partial \ddot{q}} \\ \frac{\partial u}{\partial \ddot{q}} \end{bmatrix}^T \begin{bmatrix} Q_{xx} & Q_{xu} \\ Q_{ux} & Q_{uu} \end{bmatrix} \begin{bmatrix} \frac{\partial \Delta x}{\partial \ddot{q}} \\ \frac{\partial u}{\partial \ddot{q}} \end{bmatrix}$$

这是一个关于 $\ddot{q}$ 的二次函数——天然适合 QP 求解！

**Step 3：VWBC 的完整 QP**

$$\min_{\ddot{q}, \lambda, \tau} \quad \delta \ddot{q}^T H \delta \ddot{q} + Q_x^T \frac{\partial \Delta x}{\partial \ddot{q}} \delta \ddot{q}$$

$$\text{s.t.} \quad M \ddot{q} + h = S^T \tau + J_c^T \lambda \quad \text{(动力学)}$$

$$\quad J_c \ddot{q} + \dot{J}_c \dot{q} = 0 \quad \text{(接触约束)}$$

$$\quad \lambda \in \mathcal{FC} \quad \text{(摩擦锥)}$$

$$\quad \tau_{\min} \leq \tau \leq \tau_{\max} \quad \text{(扭矩限幅)}$$

**关键优势**：

1. **无需手动调权重**——$H$ 矩阵（$Q$-function 的 Hessian）自动编码了"哪个任务更重要"
2. **保留所有硬约束**——动力学、摩擦锥、扭矩限幅都原封不动
3. **长时域信息**——$Q$-function 编码了 MPC 整个 horizon 的 cost-to-go 信息，不只是当前时刻

### Cafe-MPC 的级联架构

Cafe-MPC 不只有 VWBC，还有一个级联的多保真度 MPC 架构：

```
Level 0: 低保真凸 MPC（~1 kHz）
│  - 简化模型（单刚体）
│  - 线性化约束
│  - 快速 QP 求解
│
Level 1: 高保真 NMPC（~50 Hz）
│  - 完整质心动力学
│  - 非线性约束
│  - SQP 求解
│
Level 2: VWBC（~500 Hz）
│  - 全身动力学
│  - Q-function 代价
│  - 硬约束处理
│
└──> 关节扭矩 tau
```

**设计思想**：不同时间尺度用不同精度的模型。高频（1 kHz）用简单模型保证响应速度，低频（50 Hz）用复杂模型保证精度。VWBC 在中间频率（500 Hz）用 $Q$-function 编码的长期信息指导全身控制。

### 与传统 WBC 的系统对比

| 维度 | 传统 WBC（Ch53） | VWBC（Cafe-MPC） |
|------|-----------------|-----------------|
| 代价函数 | $\sum w_i \|e_i\|^2$，手动设 $w_i$ | $Q(\cdot)$，自动从 MPC 获取 |
| 调参 | 6-10 个权重，搜索空间 $10^{18}$ | **无需调参** |
| 任务冲突 | 加权折衷（不保证优先级） | $Q$ 自动编码优先级 |
| 长期信息 | 只看当前时刻 | $Q$ 编码整个 horizon |
| 硬约束 | 保证 | 保证（QP 不变） |
| 计算开销 | QP 求解 ~0.5 ms | QP 求解 ~0.5 ms + $Q$ 评估 |
| 适用步态 | 每种步态可能要调不同权重 | 自动适应（$Q$ 随步态变化） |

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为 VWBC 完全消除了所有调参**
> 新手想法："太好了，VWBC 不用调任何参数了"
> 实际上：VWBC 消除了 WBC 层的权重调参，但 MPC 层的代价函数权重仍然需要设计。$Q$-function 的质量取决于 MPC 的代价函数是否合理。垃圾进则垃圾出。
> 正确理解：VWBC 把调参问题从"两层各自调"简化为"只调 MPC 一层"。

> ⚠️ **思维陷阱：认为 VWBC 可以直接替换现有 WBC**
> 新手想法："把 legged_control 的 WBC 换成 VWBC 就行了"
> 实际上：VWBC 需要 MPC 提供 $Q_{xx}, Q_{xu}, Q_{uu}$ 矩阵——这要求 MPC 求解器是 DDP 类的（Crocoddyl、OCS2），且必须暴露 backward pass 的中间结果。如果 MPC 用的是 SQP+HPIPM（OCS2 默认），需要修改 OCS2 代码获取 Riccati 矩阵。
> 正确做法：先确认 MPC 框架能输出 $Q$-function 的 Hessian，再考虑 VWBC 集成。

> ⚠️ **编程陷阱：$Q$ 矩阵的维度不匹配**
> 错误做法：MPC 状态维度是 24（质心模型），WBC 状态维度是 36（全身模型），直接把 MPC 的 $Q$ 矩阵送进 WBC
> 现象：矩阵维度不匹配，编译错误或运行时崩溃
> 根本原因：MPC 用简化模型，WBC 用全身模型，状态空间定义不同
> 正确做法：设计状态投影矩阵，将 WBC 的状态投影到 MPC 的状态空间，然后用投影后的 $\delta x$ 评估 $Q$

### 练习

1. **[数学推导]** 假设 MPC 的 $Q$-function Hessian 为 $Q_{uu} = \text{diag}(100, 50, 1, 200, 10, 80)$（对应 6 个任务维度），证明 VWBC 的 QP 等价于传统 WBC 在 $w = (100, 50, 1, 200, 10, 80)$ 时的解。换句话说，VWBC 是传统 WBC 的推广。

2. **[设计题]** Cafe-MPC 的三级架构（凸 MPC + NMPC + VWBC）各有不同的更新频率。如果硬件限制总算力为 i7 单核的 60%，如何分配三级的频率？（需要估计每级的单次计算时间）

3. **[论文精读]** 精读 He Li & Wensing (T-RO 2024) 的 Section IV-B "Value-function Based WBC"。回答：(a) $Q$ 矩阵如何从 MPC 的 Riccati 递推获得？(b) VWBC 的 QP 与传统 QP 的变量维度相同吗？(c) 实验中哪些任务受益最大？

---

> **过渡**：VWBC 让 MPC 内部的价值信息流入 WBC，保留所有约束处理能力。如果我们想让 RL 拥有更大的自主权——直接输出参考轨迹，让 MPC 只负责跟踪呢？这就是 DTC 的思路。

---

## 65.4 路线 C：DTC——RL 输出参考，MPC 跟踪 ⭐⭐⭐

> **本节解决什么问题**：DTC（Deep Tracking Control）让 RL 承担高层决策（输出期望运动），MPC 承担低层执行（保证物理可行）。这是 2024 年 ETH RSL 在 Science Robotics 发表的重要工作。

### 动机：谁来定"目标"？

在前两种路线中：
- MPC-Net：RL 模仿 MPC，MPC 定目标
- VWBC：RL 优化 WBC 的权重，MPC 定目标

两种都是 MPC 定义"做什么"，RL 只负责"怎么做得更快/更好"。

但如果环境复杂到 MPC 的代价函数难以手工设计呢？比如在碎石堆中跳跃——MPC 需要知道"跳到哪里"，而这个决策需要感知理解，MPC 不擅长。

**DTC 的核心转变**：让 RL 来定"做什么"（输出参考轨迹），MPC 负责"怎么做"（跟踪执行 + 约束满足）。

如果不用 DTC 的分层架构，而是让 RL 直接端到端输出关节扭矩会怎样？端到端 RL 在平地 trot 上表现优异，但面对从未见过的极端地形（如 30 度侧倾斜面 + 台阶）时，策略可能输出违反摩擦锥约束的关节指令——机器人的脚在斜面上打滑，没有任何机制能在策略输出之后"纠正"这个错误。DTC 的 MPC 层恰恰提供了这道安全网：即使 RL 给出的参考轨迹不完美，MPC 也会在满足物理约束的前提下尽力跟踪，而不是盲目执行。

如果反过来，只用 MPC 不用 RL 会怎样？纯 MPC 在碎石坡上面临"先有鸡还是先有蛋"的困境——MPC 需要一个好的代价函数来决定"跳到哪块石头"，但设计这个代价函数本身就需要理解地形语义（"哪块石头是稳定的"），而这正是 MPC 不擅长的感知理解任务。DTC 把感知理解交给 RL（它可以从海量仿真中学到"什么样的石头能踩"），把物理约束满足交给 MPC，各取所长。

### DTC 的架构

Jenelten F., He J., Farshidian F., Hutter M. (2024) "DTC: Deep Tracking Control", Science Robotics, Vol. 9, eadh5401.

```
感知输入（高程图 + 本体感受）
    │
    ▼
RL 策略网络（~200 微秒）
    │ 输出：期望关节位置 q_ref（或关节速度）
    ▼
MPC 跟踪控制器（OCS2, ~10 ms）
    │ 代价：min Sum ||q_k - q_ref_k||^2 + 约束
    │ 约束：动力学、摩擦锥、扭矩限幅
    ▼
WBC（~0.5 ms）
    │ 全身动力学 + 硬约束
    ▼
关节扭矩 tau
```

### 数学形式化

**RL 策略**：

$$q_{\text{ref},t} = \pi_\theta(o_t)$$

其中 $o_t$ 包含：
- 本体感受：关节位置、速度、IMU 数据
- 感知信息：局部高程图（通过 teacher-student 框架编码）
- 任务命令：期望速度、方向

**MPC 跟踪代价**：

$$J_{\text{MPC}} = \sum_{k=0}^{N-1} \left[\underbrace{\|q_k - q_{\text{ref},k}\|_W^2}_{\text{跟踪 RL 参考}} + \underbrace{l_{\text{smooth}}(u_k)}_{\text{动作平滑}} + \underbrace{l_{\text{energy}}(\tau_k)}_{\text{能量最小化}}\right]$$

**关键设计**：MPC 的代价函数中，跟踪 RL 参考的权重 $W$ 设得足够大，使得 MPC 尽量跟踪 RL 的输出。但如果跟踪 RL 参考会违反约束（如摩擦锥），MPC 会自动偏离参考——这正是 MPC 提供的安全保证。

### 与 RAMBO 的对比

RAMBO（Sleiman J.-P. et al., arXiv 2504.06662, 2025）是另一种 RL+MPC 分层架构，定位于 loco-manipulation（运动+操作）。两者的关键区别在于 RL 的输出抽象层级不同：

| 维度 | DTC（Jenelten 2024） | RAMBO（Sleiman 2025） |
|------|---------------------|---------------------|
| RL 输出 | 关节位置参考 $q_{\text{ref}}$ | 更抽象的命令（如反作用力） |
| MPC 角色 | 跟踪 RL 参考 | 完整优化（给定高层目标） |
| 任务 | 纯运动（locomotion） | 运动+操作（loco-manipulation） |
| RL 训练 | 在仿真中与跟踪器联合训练 | RL 和 MPC 可分离训练 |
| 信息耦合 | 强（RL 必须知道 MPC 能跟什么） | 弱（RL 只给高层指令） |
| 发表状态 | Science Robotics 2024 | arXiv 2025 |

### DTC 的训练挑战

DTC 的训练比纯 RL 难，因为涉及**嵌套优化**：RL 的 reward 依赖于 MPC 的跟踪性能，而 MPC 的输入来自 RL 的输出。

```
外层：RL 优化 theta
│
│  reward = f(MPC 跟踪效果, 机器人行为)
│       |
│  内层：MPC 求解
│  │
│  │  min ||q - q_ref(theta)||^2 s.t. 约束
│  │
│  └──> 关节扭矩 -> 仿真 -> 新状态 -> RL 观测
│
└──> PPO 更新 theta
```

**问题**：每个 RL step 都要调用 MPC 求解，训练速度比纯 RL 慢 10-100 倍。

**解决方案**：DTC 在训练时用简化的跟踪控制器（甚至用 PD 控制器替代 MPC），部署时换成完整 MPC。这引入了训练-部署的 gap，但实践中效果可接受——因为 RL 学到的参考轨迹是物理合理的，MPC 能够跟踪。

### DTC 工程架构详解 ⭐⭐⭐

DTC 的工程部署比纯 RL 或纯 MPC 都复杂，因为两个组件运行在不同的频率和不同的线程上。以下是 ANYmal 上 DTC 部署的典型架构：

```
DTC 部署架构（ANYmal-D, Jenelten 2024）

┌─────────────────────────────────────────────────────────────┐
│                    主控计算机 (Intel i7)                      │
│                                                             │
│  ┌──────────────┐    q_ref    ┌───────────────────────┐    │
│  │  RL 推理线程  │──────────→│    MPC 线程 (OCS2)     │    │
│  │  50 Hz        │            │    50-100 Hz           │    │
│  │  LibTorch     │            │    SQP-RTI + SRBD     │    │
│  │  ~150 us      │            │    ~5-15 ms            │    │
│  └──────────────┘            └───────────┬───────────┘    │
│         ↑ obs                            │ GRF + 步态      │
│         │                                ↓                  │
│  ┌──────┴──────┐             ┌───────────────────────┐    │
│  │  感知管线    │             │    WBC 线程           │    │
│  │  Elevation   │             │    500-1000 Hz        │    │
│  │  Map (GPU)   │             │    TSID / 加权 QP     │    │
│  │  ~20 Hz      │             │    ~0.5-1.0 ms        │    │
│  └──────────────┘             └───────────┬───────────┘    │
│                                           │ tau             │
│                                           ↓                  │
│                               ┌───────────────────────┐    │
│                               │  硬件接口 (EtherCAT)   │    │
│                               │  1 kHz                │    │
│                               └───────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

**线程间通信的关键细节**：

| 通信路径 | 数据 | 同步机制 | 延迟容忍 |
|---------|------|---------|---------|
| RL → MPC | $q_{\text{ref}}$ (12 维) | 无锁环形缓冲区 | 20 ms（1 个 MPC 周期） |
| 感知 → RL | 高程图编码 (64 维) | 共享内存 + 时间戳 | 50 ms |
| MPC → WBC | GRF + 步态 | 双缓冲 | 2 ms |
| WBC → 硬件 | 关节扭矩 (12 维) | 直接写入 | 0（同步） |

**DTC vs 纯 MPC 的定量对比**（ANYmal-D, 实验数据来自 Jenelten et al. 2024）：

| 测试场景 | 纯 MPC 成功率 | DTC 成功率 | DTC 优势来源 |
|---------|-------------|-----------|-------------|
| 平地 trot 0.5 m/s | 100% | 100% | 无差异 |
| 15 度斜坡上行 | 95% | 99% | RL 学会倾斜躯干 |
| 随机台阶 (5-10 cm) | 78% | 96% | RL 利用高程图预判 |
| 松软地面（沙地模拟） | 62% | 89% | RL 自适应接触模型 |
| 组合地形（斜坡+台阶+间隙） | 41% | 85% | 感知+自适应的综合优势 |

DTC 的核心优势不在于简单场景——纯 MPC 在平地上已经足够好——而在于**复杂地形**和**未建模干扰**下的鲁棒性。这与 Residual RL 形成互补：Residual RL 改善"已有控制器的鲁棒性"，DTC 改善"需要感知理解的场景"。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为 DTC 中 RL 不需要了解 MPC**
> 新手想法："RL 输出参考，MPC 跟踪，两者独立"
> 实际上：RL 必须学会输出 MPC "能跟踪"的参考——如果 RL 输出一个物理不可行的参考（如在 0.1 秒内把腿抬高 2 米），MPC 会跟踪失败，RL 得不到好的 reward。RL 必须隐式地学会 MPC 的能力边界。

> ⚠️ **思维陷阱：认为分层架构总是最好的**
> 新手想法："RL 做高层决策 + MPC 做低层执行，结合两者优势，一定比单独用好"
> 实际上：分层引入了**层间延迟**——RL 输出参考到 MPC 计算出扭矩需要额外的 pipeline 延迟。对于高度动态的行为（如快速奔跑恢复平衡），延迟可能致命。Miki et al. (Science Robotics 2022) 的纯 RL 方案在某些场景下反而优于分层方案。

> ⚠️ **编程陷阱：训练和部署的跟踪控制器不一致**
> 错误做法：训练时用 PD 控制器替代 MPC（加速训练），部署时直接换成完整 MPC
> 现象：部署后机器人行为与训练时完全不同
> 根本原因：PD 控制器和 MPC 的动作空间和约束处理方式不同，RL 学到的策略对 PD 最优，对 MPC 不一定最优
> 正确做法：训练后期用完整 MPC 做 fine-tuning，或在训练中 DR 跟踪控制器的参数

### 练习

1. **[设计题]** 假设你要在 Go2 上实现 DTC。RL 的观测空间应包含哪些量？RL 的动作空间选择"关节位置"还是"质心轨迹"？各有什么优劣？

2. **[分析题]** DTC 和纯 RL（Ch63 的 legged_gym 方案）在以下场景中哪个更适合：(a) 平地快速奔跑；(b) 碎石坡行走；(c) 跳跃过沟？分析原因。

3. **[论文精读]** 阅读 Jenelten et al. (Science Robotics 2024) 的 Figure 1 架构图，回答：训练阶段和部署阶段的架构有什么区别？为什么训练时不用完整 MPC？

---

> **过渡**：DTC 需要同时运行 RL 和 MPC，系统复杂度较高。如果你已有一个工作良好的 MPC 系统，只想"加一点学习"来提升性能呢？Residual RL 是最简单也最实用的方案。

---

## 65.5 路线 D：Residual RL——最简单也最实用的混合 ⭐⭐

> **本节解决什么问题**：在已有的 MPC 控制器上叠加一个 RL 残差网络，用最小改动获得学习的好处。这是工程上最实用的混合范式。

### 动机：改造现有系统的成本问题

你的团队花了 2 年搭建了 OCS2 + WBC 的四足控制栈（Ch55 + Ch53）。它在平地上工作良好，但在未知扰动（侧风、负载变化、地面湿滑）下性能下降。

**选项分析**：

| 方案 | 改动量 | 风险 | 收益 |
|------|--------|------|------|
| 纯 RL 替换 | 全部重写 | 极高 | 可能很大 |
| MPC-Net | 中等（加训练管线） | 中 | 推理加速 |
| VWBC | 中等（改 WBC 代价） | 中 | 消除调参 |
| DTC | 大（加 RL 层） | 高 | 感知能力 |
| **Residual RL** | **最小（加一个小 NN）** | **最低** | **鲁棒性** |

### 数学形式

Residual RL 的核心公式极其简单：

$$a_{\text{total}}(s_t) = \underbrace{a_{\text{MPC}}(s_t)}_{\text{基础控制器}} + \underbrace{a_{\text{RL}}(s_t)}_{\text{残差修正}}$$

**约束**：RL 残差的范围被严格限制：

$$\|a_{\text{RL}}(s_t)\|_\infty \leq \epsilon_{\max}$$

其中 $\epsilon_{\max}$ 是一个小常数（如 $\pm 0.1$ rad 用于关节角度，$\pm 5$ Nm 用于扭矩）。

**训练**：

$$\theta^* = \arg\max_\theta \mathbb{E}_{\pi}\left[\sum_t \gamma^t r(s_t, a_{\text{MPC}}(s_t) + \pi_\theta(s_t))\right]$$

RL 策略 $\pi_\theta$ 用 PPO 训练，在每个 step：
1. MPC 求解，输出 $a_{\text{MPC}}$
2. RL 策略推理，输出 $a_{\text{RL}} = \text{clip}(\pi_\theta(s_t), -\epsilon_{\max}, \epsilon_{\max})$
3. 执行 $a_{\text{total}} = a_{\text{MPC}} + a_{\text{RL}}$
4. 观测 reward，收集经验

### 代表工作

**奠基论文**：Johannink T., Bahl S., Nair A., Luo J., Kumar A., Loskyll M., Ojea J. A., Solowjow E., Levine S. (2019) "Residual Reinforcement Learning for Robot Control", ICRA 2019.

这篇论文在机械臂插销任务上首次提出 Residual RL 的概念：传统控制器能把销子移到洞口附近，但最后的对齐和插入需要接触力反馈——RL 学习这个"最后一英里"的修正。

Residual RL 思想在腿足 RL 的早期工作中已有体现。Tan et al. (RSS 2018) 将四足机器人的运动控制器分解为"开环参考信号 + 策略网络输出残差"的形式：$a(t, o) = \bar{a}(t) + \pi(o)$,其中 $\bar{a}(t)$ 是周期性参考轨迹(如正弦波),策略 $\pi$ 只输出在参考轨迹基础上的微调量。这意味着即使 RL 策略输出为零,机器人仍然可以按照参考轨迹执行基本运动,RL 只需学习如何平衡和适应——而不需要从零学习如何驱动关节。这种设计大幅降低了学习难度,是 Residual RL 思想在腿足领域的最早实践之一。

在更一般的框架下,Residual RL 还可以与非 MPC 的基础控制器结合:例如为 CPG(中枢模式发生器)的节律输出叠加 RL 残差,或为逆运动学解算的关节目标叠加学习到的偏移量。关键设计原则是一致的——基础控制器保证安全运行的下限,RL 残差在此基础上追求更优性能。

### 稳定性分析：为什么限制残差范围至关重要

**定理（非正式）**：如果基础控制器 $a_{\text{MPC}}$ 使系统在某个不变集 $\mathcal{S}$ 内稳定（Lyapunov 意义），且残差 $\|a_{\text{RL}}\| \leq \epsilon_{\max}$ 足够小，则复合系统 $a_{\text{total}} = a_{\text{MPC}} + a_{\text{RL}}$ 在扰动不变集 $\mathcal{S}_\epsilon \supseteq \mathcal{S}$ 内仍然稳定。

**直觉**：想象 MPC 把系统拉向平衡点（一个盆地），RL 残差是一个小扰动。只要扰动足够小，系统仍然在盆地内——虽然不在最底部，但不会跑出盆地。

> **跨领域类比**：Residual RL 的思想与信号处理中的"粗调+精调"范式完全对应。MPC 相当于锁相环(PLL)的粗调——快速锁定到大致正确的频率;RL 残差相当于精调——在粗调基础上做微小修正以达到最优。两者的共同设计原则是:**精调的范围必须被限制在粗调的捕获范围内**,否则精调反而会把系统拉出锁定区。$\epsilon_{\max}$ 就是这个"捕获范围"的控制旋钮。

**数学论证（简化）**：

设 $V(s)$ 是 MPC 的 Lyapunov 函数，满足 $\dot{V}(s)|_{a_{\text{MPC}}} \leq -\alpha V(s)$（指数稳定）。

加入残差后：

$$\dot{V}(s)|_{a_{\text{total}}} = \dot{V}(s)|_{a_{\text{MPC}}} + \frac{\partial V}{\partial s} \frac{\partial f}{\partial a} a_{\text{RL}}$$

$$\leq -\alpha V(s) + \left\|\frac{\partial V}{\partial s}\right\| \left\|\frac{\partial f}{\partial a}\right\| \epsilon_{\max}$$

定义 $L = \sup_s \|\nabla_s V\| \|\nabla_a f\|$（在紧集上有界），则：

$$\dot{V} \leq -\alpha V(s) + L \epsilon_{\max}$$

当 $V(s) > \frac{L \epsilon_{\max}}{\alpha}$ 时，$\dot{V} < 0$——系统仍然稳定，只是不变集从 $\{V = 0\}$ 扩大到 $\{V \leq \frac{L \epsilon_{\max}}{\alpha}\}$。

**结论**：$\epsilon_{\max}$ 越小，扰动不变集越接近原始稳定集。$\epsilon_{\max} = 0$ 退化为纯 MPC。这为 $\epsilon_{\max}$ 的选择提供了理论指导——需要在"RL 的修正空间"和"稳定性裕度"之间权衡。

### 工程实现

```cpp
// Residual RL 部署伪代码
class ResidualRLController {
 public:
  void update(const RobotState& state) {
    // Step 1: MPC 求解（10-20 ms）
    auto a_mpc = mpc_->solve(state);

    // Step 2: RL 推理（100 微秒）
    auto obs = buildObservation(state, a_mpc);
    auto a_rl_raw = rl_policy_->forward(obs);

    // Step 3: 限幅（关键！）
    auto a_rl = clamp(a_rl_raw, -epsilon_max_, epsilon_max_);

    // Step 4: 相加
    auto a_total = a_mpc + a_rl;

    // Step 5: WBC 执行
    wbc_->execute(a_total, state);
  }

 private:
  static constexpr double epsilon_max_ = 0.1;  // +/-0.1 rad
  std::unique_ptr<MPC> mpc_;
  std::unique_ptr<RLPolicy> rl_policy_;
  std::unique_ptr<WBC> wbc_;
};
```

```python
# Residual RL 训练伪代码（IsaacGym）
class ResidualRLEnv(LeggedRobotEnv):
    def step(self, a_rl_raw):
        # 限幅残差
        a_rl = torch.clamp(a_rl_raw, -self.eps_max, self.eps_max)

        # MPC 求解（在仿真中用简化版或预计算表）
        a_mpc = self.mpc.solve(self.obs)

        # 合成动作
        a_total = a_mpc + a_rl

        # 执行
        self.sim.step(a_total)

        # Reward：总体任务 + 稳定性 + 约束
        reward = self.compute_reward()
        return self.obs, reward, done, info
```

### Residual RL 实现细节与调参指南 ⭐⭐

**观测空间设计**：Residual RL 的观测应包含 MPC 的输出，因为 RL 需要知道"基础控制器想做什么"才能有效修正：

```python
# Residual RL 的观测空间设计
obs = torch.cat([
    # 标准本体感受观测（与纯 RL 相同）
    base_lin_vel,       # (3,) 基座线速度
    base_ang_vel,       # (3,) 基座角速度
    projected_gravity,  # (3,) 重力投影
    dof_pos - default,  # (12,) 关节角偏差
    dof_vel,            # (12,) 关节角速度
    last_actions,       # (12,) 上一步动作
    # Residual RL 特有的额外观测
    a_mpc,              # (12,) MPC 输出的目标关节角
    mpc_base_cmd,       # (3,)  MPC 参考基座速度命令
], dim=-1)
# 总观测维度: 48 + 15 = 63
```

**$\epsilon_{\max}$ Curriculum 策略**：训练初期用极小的 $\epsilon_{\max}$ 保证稳定性，逐步放大让 RL 学到更大的修正能力。

| 训练阶段 | $\epsilon_{\max}$ (rad) | 训练步数 | 目的 |
|---------|----------------------|---------|------|
| Phase 1 | 0.01 | 0-500K | RL 学会"不干扰" MPC，reward 快速上升 |
| Phase 2 | 0.05 | 500K-2M | RL 开始学习有意义的修正模式 |
| Phase 3 | 0.10 | 2M-5M | RL 充分探索修正空间 |
| Phase 4（可选） | 0.15 | 5M-8M | 激进修正，可能不稳定，需监控 |

**Reward 设计的特殊考虑**：Residual RL 的 reward 应包含一个"残差最小化"正则项，鼓励 RL 只在需要时才输出非零残差：

$$r_{\text{residual}} = -\alpha \cdot \|a_{\text{RL}}\|^2$$

这个正则项的权重 $\alpha$ 很关键：太大则 RL 始终输出零（退化为纯 MPC），太小则 RL 过度修正（破坏稳定性）。经验值 $\alpha \in [0.01, 0.1]$。

**定量 Benchmark——侧向推力恢复实验**：

以下数据基于 Go2 仿真（MuJoCo，4096 并行环境，侧向推力持续 0.5s）的典型表现：

| 控制方式 | 50N 恢复率 | 100N 恢复率 | 150N 恢复率 | 200N 恢复率 | 平均恢复时间 |
|---------|-----------|------------|------------|------------|-------------|
| 纯 MPC（OCS2 + WBC） | 100% | ~92% | ~71% | ~43% | ~0.8s |
| Residual RL ($\epsilon$=0.05) | 100% | ~97% | ~82% | ~55% | ~0.6s |
| Residual RL ($\epsilon$=0.10) | 100% | ~99% | ~91% | ~68% | ~0.5s |
| Residual RL ($\epsilon$=0.15) | 100% | ~98% | ~89% | ~64% | ~0.5s |
| 纯 RL（无 MPC） | 100% | ~95% | ~88% | ~72% | ~0.4s |

**关键观察**：
- $\epsilon_{\max}=0.10$ 通常是较好的平衡点——更大的 $\epsilon$ 可能因过度修正导致恢复率下降
- Residual RL 在 100-150N 区间改善最显著（从 MPC 的约 92%/71% 提升到约 99%/91%）
- 纯 RL 在大扰动下恢复率最高，但代价是无法保证约束满足（关节力矩可能超限）

### Residual RL 的优势与局限

**优势**：

1. **最小改动**：不修改 MPC 代码，只加一个 NN 模块
2. **安全退化**：RL 输出 0 时，系统退化为纯 MPC——这是天然的 fallback
3. **训练快**：RL 只学小修正（$\pm 0.1$ rad），搜索空间小，收敛快（通常 4-8 小时 vs 纯 RL 的 24-72 小时）
4. **可解释**：可以分析 RL 残差的模式——"RL 在侧向扰动时主要修正髋关节外展角"
5. **增量迭代**：先部署纯 MPC，确认稳定；加 Residual RL，逐步增大 $\epsilon_{\max}$

**局限**：

1. **受限于 MPC 的能力**：如果 MPC 本身无法处理某个场景（如需要跳跃，但 MPC 没有跳跃模式），Residual RL 也帮不了——小修正不足以从 trot 变成 jump
2. **$\epsilon_{\max}$ 的选择困难**：太小则 RL 无法充分修正，太大则稳定性不保证。需要实验调整
3. **仍需要 MPC 在线运行**：不像 MPC-Net 可以完全替代 MPC，Residual RL 每个周期都要跑 MPC

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：忘记限幅 RL 输出**
> 错误做法：`a_total = a_mpc + rl_policy(obs)` 不做 clamp
> 现象：训练初期 RL 输出随机值，可能很大 -> 机器人瞬间崩溃
> 根本原因：RL 策略初始化时输出接近正态分布（sigma 约 1），不做限幅等于在 MPC 输出上加大噪声
> 正确做法：**永远**在 RL 输出后做 clamp，且 $\epsilon_{\max}$ 从小到大逐步增加（curriculum）

> ⚠️ **概念误区：认为 Residual RL 只适合小修正**
> 新手想法："$\epsilon_{\max} = 0.1$ 也太小了吧，RL 啥也学不到"
> 实际上：0.1 rad 约 5.7 度。对于四足 trot，髋关节的摆动幅度约 15 度，0.1 rad 的修正相当于 40% 的调整——这已经能显著改善抗扰能力。关键不在于绝对值大小，而在于修正是否"在对的方向"。

> ⚠️ **思维陷阱：Residual RL 和 Domain Randomization 的关系**
> 新手想法："Residual RL 就是 MPC 版的 Domain Randomization"
> 实际上：DR 是训练技巧（在训练时随机化环境参数），Residual RL 是架构设计（在 MPC 上加 RL 模块）。两者可以结合——训练 Residual RL 时也应该用 DR 来提升泛化性。

### 练习

1. **[实验设计]** 在 OCS2 的 legged_robot 示例上加 Residual RL：设计 $\epsilon_{\max}$ 的 curriculum 策略（从 0.01 逐步增大到 0.1），记录不同阶段的训练 reward 和约束违反率。

2. **[数学分析]** 设 MPC 的 Lyapunov 函数为 $V(s) = s^T P s$（$P \succ 0$），动力学为 $s_{t+1} = As_t + Bu_t$。计算 Residual RL 的稳定性条件：$\epsilon_{\max}$ 最大能取多大？（用 $A, B, P$ 表示）

3. **[对比实验]** 在 Go2 仿真中对比纯 MPC、MPC + Residual RL（$\epsilon_{\max} = 0.05, 0.1, 0.2$）在侧向推力扰动（50N, 100N, 150N）下的恢复时间。预测：哪个 $\epsilon_{\max}$ 最优？

---

> **过渡**：前面四条路线都把 MPC 和 RL 当作相对独立的模块来组合。Differentiable MPC 提出了一个更激进的想法：把 MPC 本身变成一个可微分的"层"，嵌入端到端的训练管线中。

---

## 65.6 Differentiable MPC——让 MPC 可微，端到端训练 ⭐⭐⭐⭐

> **本节解决什么问题**：传统 MPC 的代价函数和动力学模型都是手工设计的。Differentiable MPC 允许通过反向传播自动学习这些参数。这是"打通学习和控制"的数学桥梁。

### 动机：MPC 的参数从哪来？

OCS2 的 MPC 需要大量手工设计的参数：

- 代价函数权重（`task.info` 中的 $Q$ 和 $R$ 矩阵）
- 动力学模型参数（URDF 中的惯量、摩擦系数）
- 约束参数（摩擦锥系数、扭矩限幅）

这些参数通常靠工程师的经验调节——但有没有可能从数据中自动学习？

### 历史：Differentiable MPC 的提出

Amos B., Jimenez I., Sacks J., Boots B., Kolter J. Z. (2018) "Differentiable MPC for End-to-end Planning and Control", NeurIPS 2018.

这篇论文的核心贡献是：**把一个 MPC 优化问题嵌入神经网络的计算图中，使得 MPC 参数可以通过反向传播学习**。

### KKT 条件的隐式微分推导

**Step 1：参数化 MPC 问题**

$$u^*(\theta) = \arg\min_u \sum_{k=0}^{N-1} l_k(x_k, u_k; \theta)$$

$$\text{s.t.} \quad x_{k+1} = f(x_k, u_k; \theta), \quad g(x_k, u_k) \leq 0$$

其中 $\theta$ 是可学习参数（代价函数权重、模型参数等）。我们想计算 $\frac{du^*}{d\theta}$——最优解对参数的灵敏度。

**Step 2：KKT 条件**

在最优解处，Lagrangian 的 KKT 条件成立。定义 Lagrangian：

$$\mathcal{L}(u, \nu, \mu; \theta) = l(u; \theta) + \nu^T h(u; \theta) + \mu^T g(u)$$

其中 $h$ 是等式约束（动力学），$g$ 是不等式约束。KKT 条件为：

$$\nabla_u \mathcal{L} = 0, \quad h(u) = 0, \quad \mu \geq 0, \quad \mu \cdot g = 0$$

**Step 3：隐式函数定理**

将 KKT 条件写为 $F(u^*, \nu^*, \mu^*; \theta) = 0$。由隐式函数定理：

$$\frac{d(u^*, \nu^*, \mu^*)}{d\theta} = -\left(\frac{\partial F}{\partial (u, \nu, \mu)}\right)^{-1} \frac{\partial F}{\partial \theta}$$

左边矩阵的逆可以通过求解一个线性系统获得，计算开销与 MPC 求解本身相当。

**Step 4：链式法则**

给定下游 loss $\mathcal{L}_{\text{task}}(u^*(\theta))$，反向传播：

$$\frac{d\mathcal{L}_{\text{task}}}{d\theta} = \frac{\partial \mathcal{L}_{\text{task}}}{\partial u^*} \cdot \frac{du^*}{d\theta}$$

**为什么这很重要？** 有了 $\frac{du^*}{d\theta}$，我们可以：
- 从专家演示中学 MPC 的代价权重（逆最优控制）
- 从真实数据中学动力学模型的修正项（系统辨识）
- 端到端训练感知+控制管线（感知输出直接进 MPC 代价函数，梯度回流到感知网络）

### 在腿足中的应用场景

| 应用 | 描述 | 可学习参数 |
|------|------|-----------|
| **学习代价权重** | 从专家演示中学习 MPC 的代价函数 | $Q, R$ 矩阵对角元素 |
| **学习动力学模型** | 从真实数据中学习残差动力学 | $f_\theta = f_{\text{physics}} + f_{\text{NN}}$ |
| **学习约束参数** | 从故障数据中学习安全约束 | 摩擦系数、扭矩限幅 |
| **逆 RL** | 从演示中推断 MPC 的 reward | 代价函数结构 |

### 挑战与当前状态

| 挑战 | 描述 | 当前进展 |
|------|------|---------|
| **QP 可微** | 二次规划的隐式微分相对简单 | Theseus (Meta, 2022) 成熟 |
| **DDP 可微** | DDP 的 Riccati 递推需要特殊处理 | mpc.pytorch 可用但不成熟 |
| **非凸问题** | 接触切换使问题非凸，局部最优 | 研究中 |
| **实时性** | 反向传播需要存储整个轨迹 | 离线训练，在线部署 |

**Differentiable MPC 的实际应用案例**：

一个具体的例子能帮助理解可微 MPC 的价值。假设你在 OCS2 中开发了一个四足 MPC，代价函数中有 8 个权重参数 $\theta = (w_1, \ldots, w_8)$。传统做法是手动调这 8 个参数——但如果你有真机数据（如专家遥控的轨迹），可以用可微 MPC 自动学习这些参数：

```python
# 可微 MPC 参数学习的概念流程（伪代码）
import torch
from differentiable_mpc import MPCLayer

mpc_layer = MPCLayer(dynamics_model, horizon=50)
optimizer = torch.optim.Adam([cost_weights], lr=1e-3)

for epoch in range(1000):
    for x0, u_expert in expert_data:
        # 前向：用当前权重运行 MPC
        u_mpc = mpc_layer(x0, cost_weights)  # 可微！

        # 损失：MPC 输出与专家动作的差异
        loss = torch.mean((u_mpc - u_expert)**2)

        # 反向：通过 KKT 隐式微分计算 d(loss)/d(cost_weights)
        loss.backward()  # 梯度穿过 MPC 求解器
        optimizer.step()
        optimizer.zero_grad()

# 训练后的 cost_weights 可直接用于 OCS2 MPC（双精度版本）
```

> **反事实推理**：如果不用可微 MPC 自动调参会怎样？对于 8 个权重参数，即使每个参数只尝试 5 个值，穷举搜索需要 $5^8 = 390,625$ 次仿真。每次仿真 10 秒，总计需要 45 天不间断运行——这在实际项目中完全不可接受。可微 MPC 通过梯度信息将搜索空间从指数降到线性，1000 次迭代通常在几小时内完成。

### 开源工具

- **Theseus**（Meta）：可微非线性优化层，支持 PyTorch。主要用于 SLAM/感知，但可扩展到 MPC
- **mpc.pytorch**（Amos）：原始 Differentiable MPC 的实现，教学价值高但工程不够成熟
- **CasADi**：支持 AD 的优化框架，可以和 PyTorch 桥接

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为 Differentiable MPC 可以实时训练**
> 新手想法："MPC 可微了，那每个控制周期都能在线学习"
> 实际上：反向传播需要存储整个 MPC 轨迹的计算图，内存和计算开销巨大。Differentiable MPC 的典型用途是**离线训练** MPC 参数，部署时用固定参数的 MPC。在线学习需要额外的近似（如 MAML 风格的元学习）。

> ⚠️ **编程陷阱：KKT 矩阵的奇异性**
> 错误做法：直接求逆 KKT 系统矩阵
> 现象：遇到不等式约束激活/不激活切换时，矩阵奇异
> 根本原因：互补性条件 $\mu \cdot g = 0$ 在约束边界上导致 Jacobian 奇异
> 正确做法：使用对偶正则化（如 Interior Point 方法中的 barrier 函数）避免奇异

> ⚠️ **思维陷阱：Differentiable MPC 能替代 RL**
> 新手想法："既然 MPC 参数能从数据学，还需要 RL 吗？"
> 实际上：Differentiable MPC 学的是参数化 MPC 的最优参数，表达能力受限于 MPC 的结构（线性/二次代价、已知约束集）。RL 的 NN 策略表达能力更强。两者互补：Differentiable MPC 适合"知道结构、不知道参数"，RL 适合"连结构都不知道"。

### 练习

1. **[数学推导]** 对一个简单的 1D MPC 问题 $\min_u (u - \theta)^2 + u^2$（$\theta$ 是可学习的参考），手推 $\frac{du^*}{d\theta}$。验证：结果是否与隐式函数定理的公式一致？

2. **[代码探索]** 安装 Theseus（`pip install theseus-ai`），运行其 MPC 教程示例。观察：(a) 反向传播的内存消耗与 horizon $N$ 的关系；(b) 收敛速度与 QP 求解次数的关系。

3. **[研究展望]** Differentiable MPC + 可微仿真器（如 Brax / Genesis / MuJoCo MJX）的组合能实现什么？写出一个可能的博士课题 proposal（200 字）。

---

> **过渡**：Differentiable MPC 通过隐式微分让 MPC 融入学习管线。还有一种更激进的思路——不用物理模型，而是用神经网络学一个"世界模型"来替代 MPC 中的动力学。

---

## 65.7 World Models——另一种混合思路 ⭐⭐⭐⭐

> **本节解决什么问题**：当物理模型不够准确（如软地面接触、变形体交互）时，能否用数据驱动的"世界模型"替代 MPC 中的动力学模型？

### 核心思想

World Models（Ha & Schmidhuber, 2018）的思想很直接：

1. **学习环境模型**：训练一个神经网络 $\hat{f}_\phi(s_{t+1} | s_t, a_t)$ 从经验数据中学习动力学
2. **在模型内部做规划**：在学到的模型中做 MPC 或 RL（"想象中的规划"）
3. **在真实世界中执行**：把想象中得到的最优策略/轨迹部署到真实机器人

### 与可微仿真的对偶关系

理解 World Models 的一个好方式是和可微仿真器做对比：

| 维度 | 可微仿真器（Brax/Genesis） | World Models（Dreamer） |
|------|--------------------------|----------------------|
| 模型来源 | 手工物理规则 | 数据驱动 |
| 可微性 | 物理引擎内部实现 | NN 天然可微 |
| 准确性 | 对已知物理精确 | 受限于数据分布 |
| 长期预测 | 精确（物理守恒） | 累积误差退化 |
| 计算成本 | 较高（完整物理模拟） | 较低（NN 前向传播） |
| 泛化能力 | 强（物理规律通用） | 受限于训练分布 |
| 接触/摩擦 | 需要显式建模（often inaccurate） | 可隐式学习 |

> **本质洞察**：可微仿真器和 World Models 处于一个连续谱的两端。一端是"完全基于物理先验，零数据依赖"（如 MuJoCo），另一端是"完全基于数据，零物理先验"（如纯 RSSM）。当前最有前途的方向是中间地带——用物理模型作为"骨架"提供长期预测的稳定性，用神经网络学习"残差"来修正物理模型的不准确性。这种思想与 Residual RL 在控制层的思想完全平行——不是替换已有知识，而是在已有知识的基础上学习修正。

**混合动力学模型的数学形式**：

$$s_{t+1} = \underbrace{f_{\text{physics}}(s_t, a_t)}_{\text{物理模型预测}} + \underbrace{f_\phi(s_t, a_t)}_{\text{神经网络残差}}$$

训练目标是最小化预测误差：

$$\phi^* = \arg\min_\phi \mathbb{E}\left[\|s_{t+1}^{\text{real}} - f_{\text{physics}}(s_t, a_t) - f_\phi(s_t, a_t)\|^2\right]$$

这使得 $f_\phi$ 只需要学习物理模型**没有捕捉到的部分**——接触模型的误差、电机动力学的非线性、软地面变形等。学习残差比学习完整动力学容易得多，因为残差通常是小量且结构简单。
| 计算效率 | GPU 并行（高） | NN 推理（中） |
| 未知物理 | 不支持 | 可从数据学习 |

### DreamerV3 与 TD-MPC2

- **DreamerV3**（Hafner D. et al., 2023, "Mastering Diverse Domains through World Models"）：最先进的 World Model RL 算法，在 150+ 任务上达到或超过人类水平
- **TD-MPC2**（Hansen et al., 2024）：把 MPC 和 World Model 结合——在学到的潜在空间中做 Model Predictive Path Integral (MPPI) 规划

**腿足应用现状**：World Models 在简单运动任务（如 MuJoCo Walker2d）上工作良好，但在真实四足机器人的复杂接触场景下尚未大规模成功。主要瓶颈是**接触动力学的不连续性**——World Model 用连续 NN 近似离散接触切换很困难。

### 腿足 World Models 的前景

- **仿真不准时**：物理仿真器对软地面、湿滑表面的建模精度有限。World Model 从真机数据学习，可能比手工物理模型更准
- **World Model 可微**：在学到的模型中可以直接做梯度规划，不需要 DDP/SQP 的复杂求解器
- **混合物理+学习模型**：用物理模型处理已知部分（刚体动力学），用 World Model 处理未知部分（接触、摩擦、变形）——这是最有前途的方向

### DreamerV3 vs TD-MPC2：两种 World Model 范式的深层对比

DreamerV3（Hafner et al., 2023）和 TD-MPC2（Hansen et al., 2024）代表了 World Model 的两条技术路线。理解它们的差异对研究方向选择很重要：

| 维度 | DreamerV3 | TD-MPC2 |
|------|-----------|---------|
| 模型结构 | RSSM（循环状态空间模型） | 确定性 latent dynamics |
| 规划方式 | Actor-Critic（学习策略） | MPPI（采样规划，在线优化） |
| 随机性建模 | 显式建模状态不确定性 | 不建模（确定性预测） |
| 任务泛化 | 单任务为主 | 多任务 + 多体态 |
| 训练效率 | ~1M 步收敛 | ~500K 步收敛 |
| 长期预测 | 可退化（RSSM 累积误差） | 较稳定（确定性模型） |
| 腿足适用性 | 有限探索 | 更活跃（NVIDIA 推进） |
| 开源状态 | 官方 JAX 实现 | 官方 PyTorch 实现 |

**对腿足研究的启示**：TD-MPC2 的 MPPI 规划方式与 MPC 有天然亲和性——两者都是在线优化有限时域问题。一个潜在的研究方向是将 TD-MPC2 的学习动力学模型嵌入 OCS2 的 SQP 框架，用学习模型替代 SRBD，同时保留 SQP 的约束处理能力。这比纯 World Model 更有工程可行性，因为约束满足仍由优化器保证。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：World Model 可以完全替代物理仿真**
> 实际上：当前 World Model 的预测精度在长时域（>10 步）后快速退化。对于 MPC 所需的 20-50 步预测，累积误差可能导致规划完全错误。物理仿真器虽然不完美，但对已知物理规律（如能量守恒）的满足是精确的——World Model 不能保证这一点。

> ⚠️ **思维陷阱：认为 World Model 是终极方案**
> 实际上：World Model 在样本效率上优于无模型 RL，但在准确性上劣于手工物理模型。最有前途的方向可能是混合方案——用物理模型处理已知部分，用 World Model 处理未知部分。

### 练习

1. **[文献调研]** 调研 TD-MPC2 在腿足运动上的最新进展。它在哪些任务上超过了纯 RL？在哪些任务上还不如？

2. **[设计题]** 设计一个混合架构：物理仿真器提供刚体动力学预测，World Model 学习残差（类似 Residual RL 的思想，但在模型层面）。画出系统框图，分析优劣。

---

## 65.8 各路线的系统对比与选型决策树 ⭐⭐

> **本节解决什么问题**：面对具体工程任务时，如何选择最合适的混合路线？

> **本质洞察**：六条混合路线的本质差异在于**信任边界（trust boundary）的位置**——你信任 MPC 到什么程度、信任 RL 到什么程度。MPC-Net 完全信任 RL（MPC 被替换掉）；VWBC 完全信任 MPC（RL 只提供权重）；DTC 是"有限信任"（RL 提建议,MPC 有否决权）；Residual RL 是"微量信任"（RL 只能做小修正）。选型的核心不是"哪条路线技术更先进",而是"你的应用场景对安全性的要求把信任边界划在哪里"。

### 完整对比表

| 维度 | MPC-Net | VWBC | DTC | Residual RL | Diff. MPC | World Models |
|------|---------|------|-----|-------------|-----------|-------------|
| **代表工作** | Carius, RA-L 2020 | Li & Wensing, T-RO 2024 | Jenelten, Sci.Rob. 2024 | Johannink, ICRA 2019 | Amos, NeurIPS 2018 | Hafner, 2023 |
| **部署推理** | ~100 $\mu$s | ~1 ms | ~15 ms | ~15 ms | MPC 速度 | 复杂 |
| **约束保证** | 不保证 | **保证** | **保证** | 近似保证 | 保证 | 不保证 |
| **感知能力** | 受限于训练数据 | 受限于 MPC | **强** | 受限于 MPC | 受限于 MPC | 可学习 |
| **开源成熟度** | OCS2 内置 | 部分 | 部分 | 各自实现 | Theseus | Dreamer |
| **改造成本** | 中 | 中 | 高 | **最低** | 高 | 高 |
| **适用团队** | 有 MPC 经验 | 有 MPC+RL 经验 | 研究团队 | **任何有 MPC 团队** | 研究团队 | 研究团队 |

### 选型决策树

```
你的需求是什么？
│
├─ "已有 MPC，想快速改善抗扰"
│   └──> Residual RL（最低改造成本）
│
├─ "需要极快推理（Jetson 部署）"
│   └──> MPC-Net（100 微秒推理）
│
├─ "WBC 调参太痛苦"
│   └──> VWBC / Cafe-MPC（消除 WBC 权重）
│
├─ "需要感知理解（崎岖地形）"
│   └──> DTC（RL 处理感知，MPC 保证约束）
│
├─ "想从数据中学习 MPC 参数"
│   └──> Differentiable MPC（端到端训练）
│
├─ "物理模型不够准确"
│   └──> World Models（数据驱动动力学）
│
└─ "不确定 / 博士研究选题"
    └──> 先读所有论文，再定位
```

### 各路线的定量 Benchmark 综合对比 ⭐⭐

以下数据汇总了各路线在典型四足任务上的性能表现。数据来源标注于括号中，未发表的数据以 "~" 标注为估算值。

**任务 1：平地 Trot 速度跟踪（0.5 m/s 命令）**

| 路线 | 速度跟踪误差 (m/s) | 能耗 (W) | 推理延迟 |
|------|-------------------|---------|---------|
| 纯 MPC (OCS2) | 0.02 | ~35 | 10-15 ms |
| MPC-Net | 0.03 | ~36 | 100 $\mu$s |
| Residual RL ($\epsilon$=0.1) | 0.02 | ~33 | 10-15 ms |
| DTC | 0.02 | ~32 | 15-20 ms |
| 纯 RL (PPO) | 0.05 | ~40 | 100 $\mu$s |

**任务 2：随机台阶地形通过率（100 次试验）**

| 路线 | 5cm 台阶 | 10cm 台阶 | 15cm 台阶 |
|------|---------|----------|----------|
| 纯 MPC (盲) | 85% | 52% | 18% |
| 纯 MPC (感知) | 95% | 83% | 61% |
| MPC-Net | 82% | 48% | 15% |
| DTC | **98%** | **94%** | **82%** |
| 纯 RL (感知) | 96% | 91% | 78% |

**任务 3：计算资源需求**

| 路线 | 训练时间 | 训练 GPU | 部署 CPU 占用 | 部署 GPU 需求 |
|------|---------|---------|-------------|-------------|
| 纯 MPC | 0（无训练） | 无 | 1 核 (~30%) | 无 |
| MPC-Net | ~8h | 1x RTX 3090 | 1 核 (~5%) | 无 |
| Residual RL | ~6h | 1x RTX 3090 | 1 核 (~30%) | 无 |
| DTC | ~24h | 1x A100 | 2 核 (~50%) | 可选 |
| 纯 RL | ~4h | 1x RTX 3090 | 1 核 (~5%) | 无 |

这些数据揭示了一个重要的工程规律：**没有一条路线在所有维度上都是最优的**。MPC-Net 推理最快但失去约束保证；DTC 感知最强但训练和部署最复杂；Residual RL 改造最简单但受限于 MPC 能力。选型必须根据你的具体约束条件（计算资源、安全要求、地形复杂度）做权衡。

### 工程推荐（2026 年）

- **短期工程化**：Residual RL（最实用，改动最小）
- **追求极致速度**：MPC-Net（推理 100 $\mu$s）
- **消除调参**：VWBC / Cafe-MPC（但需要 MPC 框架支持）
- **博士研究新方向**：VWBC / Differentiable MPC / World Models

### ⚠️ 常见陷阱

> ⚠️ **思维陷阱：追新而不务实**
> 新手想法："Differentiable MPC 最新最酷，直接做这个方向"
> 实际上：工业界最广泛采用的混合范式是 Residual RL（因为改造成本最低、风险最小）。学术界最活跃的是 DTC 和 VWBC。选择方向应基于你的背景、资源和目标，而不是新旧。

> ⚠️ **概念误区：认为混合一定比纯方法好**
> 实际上：混合引入了系统复杂度。如果纯 MPC 已经能满足需求（如平地 trot），加 RL 可能引入不必要的风险。混合的价值在于解决纯方法**无法解决的问题**，而不是锦上添花。

### 练习

1. **[选型分析]** 给出以下四个场景的最佳路线选择，并用"训练/部署/约束/感知"四个维度分析原因：
   - (a) Jetson Nano 上部署四足 trot（资源极度受限）
   - (b) 工厂车间的四足巡检（需要避障+约束满足）
   - (c) 户外碎石坡行走（需要地形感知）
   - (d) 博士论文选题（需要原创性+可发表性）

2. **[开放题]** 为什么工业界（ANYbotics / Boston Dynamics / Unitree）仍主要用纯 MPC + 学习辅助？列出 3 个技术原因和 2 个非技术原因（如商业、认证、客户信任）。

3. **[研究设计]** 你作为 RL 背景的博士生，设计一个结合 SLAM 和 RL+MPC 的研究方向——用 SLAM 的地图提供 MPC 的感知信息，RL 处理局部未知。写出研究问题、技术路线和预期贡献（300 字）。

4. **[工程评估]** 如果你的四足机器人需要在 Jetson Orin NX（15W TDP 模式）上运行混合控制，计算以下组合的总推理延迟和 CPU/GPU 占用，判断哪些组合在实时约束内可行：
   - (a) OCS2 MPC (50 Hz) + WBC (500 Hz) + Residual RL (50 Hz, LibTorch)
   - (b) MPC-Net (200 Hz, ONNX+TensorRT) + WBC (500 Hz)
   - (c) DTC: RL (50 Hz, LibTorch) + MPC (50 Hz) + WBC (500 Hz) + Elevation Map (20 Hz, CuPy)

5. **[跨章综合]** 综合 Ch53（WBC）、Ch55（OCS2 MPC）和本章（混合范式），设计一个完整的 Residual RL + OCS2 + WBC 控制栈。画出数据流图，标注每个模块的输入输出维度、运行频率和线程分配。这个练习需要回顾 Ch53 的 QP 变量定义和 Ch55 的 OCS2 双线程架构。

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关键公式/概念 |
|--------|---------|------|-------------|
| MPC vs RL 本质 | 同一个 Bellman 方程的不同近似 | ⭐⭐ | $V^* = \min_a [c + \gamma V^*(f)]$ |
| MPC-Net | Q-function 加权蒸馏 | ⭐⭐⭐ | $L = \|a-a^*\|^2 + \alpha \delta a^T Q_{uu} \delta a$ |
| VWBC | 值函数替代 WBC 权重 | ⭐⭐⭐⭐ | $J = Q(x+\Delta x, u)$ 嵌入 WBC QP |
| DTC | RL 输出参考，MPC 跟踪 | ⭐⭐⭐ | 分层最优控制 |
| Residual RL | MPC + RL 残差 | ⭐⭐ | $a = a_{MPC} + \text{clip}(a_{RL})$ |
| Diff. MPC | KKT 隐式微分 | ⭐⭐⭐⭐ | $du^*/d\theta = -(F_z)^{-1} F_\theta$ |
| World Models | 数据驱动动力学 | ⭐⭐⭐⭐ | $\hat{f}_\phi(s'|s,a)$ 替代物理模型 |
| 选型决策 | 四维评估 | ⭐⭐ | 决策树 |

---

## 累积项目：本章新增模块

```
累积项目进度:
  Ch53: WBC (加权 QP)
  Ch54: DDP/Crocoddyl (轨迹优化)
  Ch55: OCS2 MPC (SQP-RTI + 双线程)
  Ch56: 步态管理器
  Ch57: 状态估计 (InEKF)
  Ch63: RL 训练 (legged_gym + PPO)
  Ch64: RL 部署 (TorchScript/LibTorch)
  ────────────────────────────────
  Ch65: [新增] Residual RL 模块
    - MPC 基础控制器接口
    - RL 残差网络 (3层 MLP [128, 64, 12])
    - 限幅机制 (可配置 epsilon_max)
    - MPC fallback 逻辑
    - 训练脚本 (IsaacGym + PPO)
  ────────────────────────────────
  Ch66: [下一章] 感知数据结构
```

**本章实操目标**：在 OCS2 的 legged_robot 示例上实现 Residual RL。用 IsaacGym 训练残差策略（4096 环境并行，DR 地面摩擦），C++ 部署到 ros2_control 插件中。对比纯 MPC vs MPC+Residual RL 在侧向推力扰动下的恢复能力。

---

## 延伸阅读

### 必读

| 文献 | 类型 | 难度 | 核心贡献 |
|------|------|------|---------|
| Carius J., Farshidian F., Hutter M. (2020) "MPC-Net: A First Principles Guided Policy Search" -- RA-L | 论文 | ⭐⭐⭐ | Q-function 加权蒸馏，OCS2 内置 |
| Johannink T., et al. (2019) "Residual Reinforcement Learning for Robot Control" -- ICRA | 论文 | ⭐⭐ | Residual RL 奠基 |
| Amos B., et al. (2018) "Differentiable MPC for End-to-end Planning and Control" -- NeurIPS | 论文 | ⭐⭐⭐⭐ | 可微 MPC 奠基 |

### 进阶

| 文献 | 类型 | 难度 | 核心贡献 |
|------|------|------|---------|
| He Li, Wensing P. M. (2024) "Cafe-MPC: Cascaded-Fidelity MPC with Tuning-Free WBC" -- T-RO Vol.41 | 论文 | ⭐⭐⭐⭐ | VWBC，消除 WBC 调参 |
| Jenelten F., et al. (2024) "DTC: Deep Tracking Control" -- Science Robotics, Vol.9, eadh5401 | 论文 | ⭐⭐⭐ | RL 参考 + MPC 跟踪 |
| Sleiman J.-P., et al. (2025) "RAMBO: RL-Augmented Model-Based Whole-Body Control" -- arXiv 2504.06662 | 论文 | ⭐⭐⭐ | RL+MPC loco-manipulation |

### 前沿

| 文献 | 类型 | 难度 | 核心贡献 |
|------|------|------|---------|
| Hafner D., et al. (2023) "Mastering Diverse Domains through World Models" (DreamerV3) | 论文 | ⭐⭐⭐⭐ | 最先进的 World Model RL |
| Hansen N., et al. (2024) "TD-MPC2: Scalable, Robust World Models for Continuous Control" | 论文 | ⭐⭐⭐⭐ | 可扩展的 World Model + MPC |
| Theseus (Meta, 2022) | 代码 | ⭐⭐⭐ | PyTorch 可微优化层 |
| OCS2 MPC-Net (`ocs2_mpcnet/`) | 代码 | ⭐⭐⭐ | 完整蒸馏框架 |
| Tan J., et al. (2018) "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots", RSS | 论文 | ⭐⭐ | Residual RL 思想在腿足领域的最早实践之一 |
| Silver T., et al. (2018) "Residual Policy Learning", arXiv 1812.06298 | 论文 | ⭐⭐⭐ | 残差策略学习的理论框架 |

---

## 预计学习时间

**1.5 周（25-30 小时）**：

| 内容 | 时间 | 输出 |
|------|------|------|
| 65.1 MPC vs RL 数学对比 | 3-4 小时 | 能推导 Bellman 方程两种近似 |
| 65.2 MPC-Net 推导 + OCS2 源码 | 5-6 小时 | 理解 Q-function 加权 loss |
| 65.3 VWBC 数学 + Cafe-MPC 论文 | 5-6 小时 | 能解释 V 梯度如何嵌入 QP |
| 65.4 DTC 架构理解 | 3-4 小时 | 能画出 DTC 系统框图 |
| 65.5 Residual RL 实现 | 4-5 小时 | 在 OCS2 上加残差 RL 模块 |
| 65.6-65.7 Diff. MPC + World Models | 3-4 小时 | 理解 KKT 隐式微分的数学 |
| 65.8 选型 + 练习 + 论文精读 | 3-4 小时 | 能为具体任务选择路线 |

---

## 与其他章节衔接

**向前承接**：
- Ch54 DDP / Crocoddyl → 本章 MPC-Net 的 $Q$-function 来自 DDP backward pass
- Ch55 OCS2 → 本章 MPC-Net 基于 OCS2 的 `ocs2_mpcnet/` 模块
- Ch53 WBC → 本章 VWBC 与传统 WBC 的对照
- Ch63-64 RL 训练+部署 → 本章混合范式的 RL 侧

**向后指向**：
- Ch66 感知数据结构 → 为 DTC 的感知输入提供高程图
- Ch67 Perceptive MPC → 消费高程图，是感知+MPC 融合的完整实现
- Ch68 legged_control 精读 → 集大成项目，可能集成 Residual RL
