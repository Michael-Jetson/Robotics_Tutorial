# 第77章：轮足混合 MPC——从滚动约束到全身非线性最优控制

| 元信息 | 值 |
|--------|-----|
| 难度 | ⭐⭐⭐⭐（切换系统 OCP + 滚动约束 + WBC 任务栈 + 数值求解） |
| 预计时间 | 1.5 周（30-35 小时） |
| 前置依赖 | 复合/60\_轮式运动学与Pfaffian、复合/20\_浮动基座臂统一动力学、足式/110\_OCS2完整栈与双线程MPC |
| 下游连接 | 复合/80\_Wheel\_Legged\_Gym\_RL、复合/90\_Swiss\_Mile商业化、复合/130\_OCS2实践 |

> **本章定位**：上一章建立了轮式运动学与 Pfaffian 约束的理论框架。本章把这些约束注入到实时最优控制中，构建完整的轮足混合 MPC。核心挑战是：同一个末端在摆动、足支撑、轮支撑三种模式下拥有不同的动力学模型、不同的约束集和不同的代价权重——如何在一个统一的优化框架中处理这种模式切换？本章从动机出发，经过完整的数学推导，延伸到 OCS2 集成、WBC 任务栈设计、数值求解策略和安全降级机制。

---

## 前置自测

📋 **前置自测**（答不出 2 题以上请先回顾对应章节）

| # | 问题 | 来源 |
|---|------|------|
| 1 | OCS2 为什么将腿足机器人作为**切换系统**处理？切换系统与标准 OCP 的数学区别是什么？ | 足式/110 |
| 2 | SRBD MPC 的状态向量和输入向量分别是什么？状态维度为何远小于全身动力学维度？ | 足式/70 |
| 3 | 轮接触的三行速度约束（纵向纯滚动、横向不滑、法向不穿透）的 Pfaffian 形式是什么？ | 复合/60 |
| 4 | SQP-RTI（Sequential Quadratic Programming - Real-Time Iteration）的核心思想是什么？它为什么能实时运行？ | 足式/110 |
| 5 | WBC 中硬约束（动力学、摩擦锥）与软任务（末端跟踪、姿态正则化）的区别是什么？如何通过 QP 实现优先级？ | 足式/60 |

**自测标准**：5/5 直接阅读；3-4/5 边读边查；0-2/5 先完成 复合/60 + 足式/110。

---

## 本章目标

学完本章后，你应当能够：

1. 写出轮足混合 OCP 的完整数学形式——状态、输入、模式相关动力学、代价和约束。
2. 将足式的二值接触（swing/stance）扩展为三值模式（swing/foot/wheel），理解模式调度的数学困难。
3. 在 OCS2 风格接口中实现滚动等式约束及其线性化近似。
4. 设计 WBC 中轮速度任务、基座稳定任务和接触力分配任务的优先级栈。
5. 建立求解超时、热启动和安全降级的完整策略。

---

## 77.1 动机：为什么轮足比纯足更需要 MPC？⭐⭐

### 这一节解决什么问题

纯足式机器人的 MPC 已经很复杂了。为什么轮足机器人的 MPC 更难？本节通过分析模式切换的数学困难，解释轮足 MPC 的核心挑战。

### 纯足式 MPC 的核心结构

回顾足式/110\_OCS2完整栈与双线程MPC：纯足式机器人的 MPC 将步态相位建模为**切换系统**（switched system）。在预测时域 $[0, T]$ 内，接触模式按步态时序切换：

$$
\min_{\mathbf{u}(\cdot)} \Phi(\mathbf{x}_T) + \int_0^T \ell_{m(t)}(\mathbf{x}(t), \mathbf{u}(t))\, dt
$$

$$
\text{s.t.} \quad \dot{\mathbf{x}} = f_{m(t)}(\mathbf{x}, \mathbf{u}), \quad g_{m(t)}(\mathbf{x}, \mathbf{u}) \leq 0
$$

其中 $m(t) \in \{0, 1, \ldots, 2^4 - 1\}$ 是 16 种可能的四足接触组合之一（每条腿 swing 或 stance）。

**纯足式的简化之处**：每条腿只有**两种模式**——swing（摆动）和 stance（支撑）。stance 模式的接触约束是 $J_c v = 0$——接触点速度为零，这是一个**完整约束**（可积分为"接触点固定"）。

### 轮足的数学困难

轮足机器人将每条腿的模式从二值扩展为**三值**：

| 模式 | 接触约束 | 代价函数 | 可用力锥 |
|------|---------|---------|---------|
| **swing** | 无约束 | 摆动轨迹跟踪 | 无接触力 |
| **foot** | $J_c v = 0$（完整约束） | 足端位置保持 | 标准摩擦锥 |
| **wheel** | Pfaffian 约束（非完整） | 轮速匹配 + 基座速度跟踪 | 修正摩擦锥 |

三值模式带来了三重数学困难：

**困难 1：模式组合爆炸**

纯足式 4 条腿，每条腿 2 种模式 $\Rightarrow$ $2^4 = 16$ 种组合。
轮足 4 条腿，每条腿 3 种模式 $\Rightarrow$ $3^4 = 81$ 种组合。

模式组合数从 16 增长到 81，且很多组合在物理上不可行（例如 4 条腿全部 swing = 空中飞行）。

**困难 2：约束类型混合**

足式 MPC 中所有接触约束都是同一类型（$J_c v = 0$）。轮足 MPC 中，不同腿可以同时处于不同模式——有的腿是完整约束（foot），有的腿是非完整约束（wheel），有的腿无约束（swing）。优化器必须处理**混合约束结构**。

**困难 3：wheel 模式引入新决策变量**

foot 模式的接触力是 3D 力向量 $\lambda_i \in \mathbb{R}^3$。wheel 模式除了接触力，还需要决定**轮速参考** $\omega_i^{\text{ref}}$——这是 foot 模式不存在的决策变量。轮速参考不仅影响运动学约束，还通过摩擦锥约束影响可用切向力。

> **类比**：纯足式 MPC 像下国际象棋——每颗棋子的走法固定（swing 或 stance）。轮足 MPC 像下围棋——每个落子（模式选择）影响全局态势，且组合空间指数级增长。轮足 MPC 的求解困难不在于单个优化步骤更难，而在于**模式空间**的组合复杂度。

> **反事实推理**：如果我们不用 MPC，而是为每种模式组合预先设计独立控制器（如 4-wheel 控制器、4-foot 控制器、2-wheel-2-foot 控制器），会怎样？首先，81 种组合中即使只考虑合理的 10-20 种，也需要设计和调试 10-20 个控制器。其次，控制器之间的切换需要额外的过渡策略。最关键的是，预设控制器无法优化**切换时机**——什么时候从 wheel 切换到 foot 是由地形决定的，需要在线规划。MPC 的核心优势正是将模式选择和运动优化统一在一个框架中。

### 历史脉络

轮足 MPC 的发展可以追溯到两条独立的研究线：

| 年代 | 贡献 | 关键突破 |
|------|------|---------|
| 2008 | Raibert et al. (BigDog) | 足式简化模型 MPC |
| 2018 | Di Carlo et al. (MIT Cheetah) | SRBD + convex MPC，20-30Hz 实时 |
| 2019 | Bjelonic et al. (ANYmal-on-wheels) | 首个轮足全身 MPC，OCS2 框架 |
| 2020 | Bjelonic et al. (Rolling in the Deep) | 在线轨迹优化，崎岖地形轮足运动 |
| 2021 | Bjelonic et al. (IROS) | 全身 MPC + 在线步态序列生成，运输能耗降低 85% |
| 2024 | Swiss-Mile (现 RIVR) | 商业化轮足机器人，RL + MPC 混合架构 |
| 2024 | Lee et al. (Science Robotics) | 学习鲁棒自主导航和运动的轮足策略 |

> **本质洞察**：轮足 MPC 的核心贡献不是"把轮约束加进去"（那只是工程实现），而是提供了一个**统一的数学框架**，让优化器自主发现最优的模式序列和运动轨迹。Bjelonic 2021 的实验表明，当 MPC 自由选择模式序列时，可以发现人类工程师未曾设想的混合运动——如用后腿行走+前轮滚动来上台阶。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：直接把足式 MPC 代码中的接触约束替换为轮约束**
> - 错误做法：在 OCS2 的 `StateInputConstraint` 中把 $J_c v = 0$ 替换为 Pfaffian 约束，其他不变
> - 典型现象：MPC 求解失败（KKT 残差不收敛），或求解成功但轨迹不可行
> - 根本原因：轮约束引入了新的决策变量（轮速参考），代价函数需要新增轮速跟踪项，摩擦锥约束需要修正为各向异性（纵向和横向摩擦能力不同）。仅替换约束是不够的
> - 正确做法：同步修改约束、代价函数、摩擦锥和状态/输入向量定义

> 💡 **概念误区：认为"轮足 MPC = 足式 MPC + 轮运动学"**
> - 新手想法："只要在足式 MPC 外面套一层轮运动学映射就行了"
> - 实际上：轮足 MPC 的困难不是运动学层面的（那是纯正运动学/逆运动学问题），而是**最优控制层面**的——模式切换使得 OCP 成为**混合整数优化**或**切换系统优化**，这是本质上更难的数学问题
> - 关键区别：运动学只回答"能不能走"，MPC 还要回答"怎么走最优"——包括最优的模式序列、最优的切换时机、最优的力/速度分配

> 🧠 **思维陷阱：追求最优模式序列**
> - 新手想法："MPC 应该搜索全局最优的模式序列"
> - 实际上：81 种模式组合在 $N$ 步预测时域上有 $81^N$ 种可能序列。即使 $N = 20$，也有 $\sim 10^{38}$ 种——完全不可枚举。实际工程中，模式序列由**启发式规则**或**学习策略**预先给定，MPC 在给定模式序列下优化连续变量。模式序列优化是更上层的问题
> - 正确思维：将问题分层——上层决定模式序列（启发式/采样/学习），下层在固定序列下优化轨迹（MPC）

### 练习 77.1

**[A] ⭐⭐** 枚举四轮轮足机器人的 81 种模式组合，标出哪些是物理可行的（至少一个 foot 或 wheel 提供支撑）、哪些不可行（全部 swing）。可行的组合有多少种？

**[B] ⭐⭐⭐** 对比纯足式 MPC 和轮足 MPC 的计算复杂度。假设状态维度 $n_x$、输入维度 $n_u$、预测步数 $N$，一次 SQP 迭代的计算量为 $O(N(n_x + n_u)^3)$。轮足 MPC 的 $n_u$ 比纯足式多多少？对计算时间的影响是多少倍？

**[C] ⭐⭐⭐** 研读 Bjelonic et al. (IROS 2021) 的摘要和引言部分。该论文如何解决模式序列搜索问题？它使用的是全局优化还是启发式？

---

## 77.2 轮足混合动力学建模 ⭐⭐⭐

### 这一节解决什么问题

MPC 的核心组件是**动力学模型**——它告诉优化器"给定当前状态和输入，下一时刻的状态是什么"。本节推导轮足机器人在不同模式下的动力学模型，并解释如何统一到一个切换系统框架中。

### 从复合/20 桥接

回顾复合/20\_浮动基座臂统一动力学的核心方程：

$$
M(q)\dot{v} + h(q, v) = S^T\tau + \sum_{i \in \mathcal{C}} J_{c,i}^T \lambda_i
$$

在那一章，我们用它分析了臂的反力矩对基座稳定性的影响。现在我们用同样的方程，但视角不同——不是分析稳定性，而是以 $(\dot{v}, \lambda)$ 为决策变量构建**最优控制问题**。

对于轮足机器人，"臂"被"轮"替代。统一方程变为：

$$
M(q)\dot{v} + h(q, v) = S^T\tau + \sum_{i \in \mathcal{C}_{\text{foot}}} J_{c,i}^T \lambda_i^{\text{foot}} + \sum_{j \in \mathcal{C}_{\text{wheel}}} J_{c,j}^T \lambda_j^{\text{wheel}}
$$

两类接触力的区别在于它们受到的**约束**不同：

| 接触类型 | 接触力约束 | 速度约束 |
|---------|-----------|---------|
| foot | 标准摩擦锥 $\|\lambda_t\| \leq \mu \lambda_n$ | $J_c v = 0$（完整） |
| wheel | 修正摩擦锥（各向异性） | Pfaffian 约束（非完整） |

### Centroidal 动力学简化

直接使用全身动力学（$n_v = 24$）做 MPC 计算量过大。与纯足式 MPC 一样，轮足 MPC 通常使用**centroidal dynamics**（质心动力学）简化：

$$
\dot{\mathbf{h}}_G = \begin{pmatrix} \dot{\mathbf{k}}_G \\ \dot{\mathbf{l}}_G \end{pmatrix} = \begin{pmatrix} \sum_i (\mathbf{p}_{c,i} - \mathbf{p}_G) \times \boldsymbol{\lambda}_i \\ m\mathbf{g} + \sum_i \boldsymbol{\lambda}_i \end{pmatrix}
$$

这将动力学从 $n_v$ 维（24 维）降为 6 维（角动量 3 + 线动量 3），且不需要求解关节加速度——这正是 centroidal dynamics 的计算优势（回顾复合/20, 72.4 节）。

**轮足的新增项**：centroidal 动力学方程本身不变，但接触力 $\lambda_i$ 的**可行域**随模式变化：

- foot 模式：$\lambda_i$ 受标准摩擦锥约束
- wheel 模式：$\lambda_i$ 的纵向分量受限于轮驱动力矩能提供的范围
- swing 模式：$\lambda_i = 0$

### SRBD 模型的轮足扩展

单刚体模型（Single Rigid Body Dynamics, SRBD）是 centroidal dynamics 的进一步简化——假设角动量变化可以通过将系统视为单个刚体来近似。SRBD 的运动方程为：

$$
m\ddot{\mathbf{p}}_G = m\mathbf{g} + \sum_i \boldsymbol{\lambda}_i
$$

$$
\mathbf{I}_G \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}_G \boldsymbol{\omega} = \sum_i (\mathbf{p}_{c,i} - \mathbf{p}_G) \times \boldsymbol{\lambda}_i
$$

轮足扩展需要在 SRBD 的输入中加入轮速参考：

| 变量 | 纯足式 SRBD | 轮足 SRBD |
|------|------------|----------|
| 状态 $\mathbf{x}$ | $(\mathbf{p}_G, \dot{\mathbf{p}}_G, \mathbf{R}_b, \boldsymbol{\omega})$ 12D | 相同 12D + 轮角速度 $\omega_w$ 4D = 16D |
| 输入 $\mathbf{u}$ | 接触力 $\lambda_i$ 12D | 接触力 12D + 轮速参考 $\omega_i^{\text{ref}}$ 4D = 16D |
| 动力学维度 | 12 | 12（质心）+ 4（轮速积分）= 16 |

轮角速度的动力学是简单的一阶积分：

$$
\dot{\omega}_{w,i} = \frac{\tau_{w,i}}{I_{w,i}}
$$

其中 $\tau_{w,i}$ 是轮电机力矩，$I_{w,i}$ 是轮转动惯量。

> **反事实推理**：如果不将轮角速度加入状态向量，会怎样？那么 MPC 无法预测未来的轮速，也无法优化轮速轨迹。轮速变成了一个"瞬时决策"——每步独立决定，失去了时间上的平滑性。实际表现为轮速命令剧烈波动，电机无法跟踪，导致打滑和能耗增加。将轮速加入状态后，MPC 可以规划平滑的轮速轨迹，并通过代价函数惩罚轮速变化率。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：SRBD 模型中忽略轮的转动惯量**
> - 错误做法：假设轮是质点，$I_{w,i} = 0$
> - 典型现象：轮速响应瞬间到位（$\dot{\omega}_w = \tau/0 = \infty$），MPC 产生不切实际的轮速轨迹
> - 根本原因：轮的转动惯量虽然小（通常 $I_w \sim 0.001$ kg$\cdot$m$^2$），但不是零。忽略它会使数值积分不稳定
> - 正确做法：设定合理的轮转动惯量，典型值 $I_w = 0.001$-$0.01$ kg$\cdot$m$^2$

> 💡 **概念误区：认为 centroidal dynamics 不适用于轮足机器人**
> - 新手想法："轮足机器人的腿有轮子，不能简化为单刚体"
> - 实际上：centroidal dynamics 的有效性不取决于末端是足还是轮——它取决于**腿部惯量是否可忽略**。只要腿部质量远小于基座质量（Go2 的腿质量约占总质量 20%），SRBD 近似就是合理的。轮的存在改变的是**接触约束**，不是**质心动力学**
> - 延伸：Swiss-Mile 的产品正是使用 SRBD + centroidal MPC 架构

### 模式相关动力学的切换表示

将三种模式的动力学统一为切换系统形式。设时间线被模式切换事件分割为若干段 $[t_k, t_{k+1})$，每段内模式 $\sigma$ 固定。对于四个末端各有三种模式，模式向量为 $\sigma = (m_1, m_2, m_3, m_4) \in \{0, 1, 2\}^4$。

在每个模式段内，系统动力学为：

$$
\dot{\mathbf{x}} = f_\sigma(\mathbf{x}, \mathbf{u}) = \begin{pmatrix} \dot{\mathbf{h}}_G \\ \dot{\mathbf{q}} \end{pmatrix} = \begin{pmatrix} \text{centroidal dynamics}(\mathbf{x}, \mathbf{u}, \sigma) \\ \text{kinematics}(\mathbf{x}, \mathbf{u}, \sigma) \end{pmatrix}
$$

其中 centroidal dynamics 的形式不随模式变化（质心力矩方程始终成立），但约束集 $g_\sigma$ 和代价权重 $\ell_\sigma$ 随模式变化。

**切换条件**：在 OCS2 框架中，模式切换通过 `event time` 实现——预先定义切换时间点，在这些时间点前后使用不同的约束和代价。与纯足式的区别是：

- 纯足式：切换通常周期性（步态周期），切换时间可预测
- 轮足：切换可能非周期性（地形触发），切换时间依赖感知

这使得轮足 MPC 的 `ModeScheduleManager` 需要更灵活的更新机制——不能假设周期性步态。

### 练习 77.2

**[A] ⭐⭐** 写出 Go2+wheels 系统的轮足 SRBD 状态向量和输入向量。状态维度和输入维度分别是多少？

**[B] ⭐⭐⭐** 推导轮足 SRBD 的离散时间状态方程（Euler 积分）。特别注意：轮角速度的积分方程和质心动力学的积分方程如何耦合？

**[C] ⭐⭐⭐** 比较 centroidal 模型和 SRBD 在轮足场景下的近似精度。什么情况下 SRBD 的近似会失效？（提示：考虑臂负载的角动量效应，回顾复合/20, 72.4 节）

---

## 77.2b 模式切换的冲击动力学：guard 与 reset map ⭐⭐⭐⭐

### 这一节解决什么问题

77.2 节给出了**每个模式段内**的连续动力学 $\dot{\mathbf{x}} = f_\sigma(\mathbf{x}, \mathbf{u})$。但模式切换的**那一瞬间**发生了什么？当一条摆动腿落地、或一个轮触地从滚动变为静止支撑时，接触约束突然激活，会产生一个**冲量**（impulse）——速度在零时间内发生跳变。这一节用混合系统（hybrid system）的语言精确刻画切换瞬间，为 77.4 节"速度连续性"约束提供数学基础。如果忽略这个冲击，MPC 规划的轨迹在物理上不可实现，真机会在切换时出现冲击噪声、足端弹跳甚至失稳。

### 为什么连续动力学不足以描述切换

回顾 77.2 的切换系统形式：时间线被分割为模式段 $[t_k, t_{k+1})$，每段内 $\sigma$ 固定。这个描述**隐含假设了切换瞬间状态连续**——$\mathbf{x}(t_{k+1}^-) = \mathbf{x}(t_{k+1}^+)$。对于 swing $\to$ foot 这样的切换，这个假设**不成立**：

考虑一条以接触点速度 $v_c^- \neq 0$ 落地的摆动腿。落地后进入 foot 模式，约束要求 $v_c = 0$。那么接触点速度必须从 $v_c^-$ 瞬间变为 $0$——这是一个**速度跳变**。根据冲量-动量定理，零时间内的速度跳变需要一个无穷大的力作用无穷小的时间，其积分（冲量 $\boldsymbol{\Lambda}$）有限：

$$
\boldsymbol{\Lambda}_i = \int_{t^-}^{t^+} \boldsymbol{\lambda}_i(\tau)\, d\tau
$$

这就是为什么纯连续动力学不足以描述切换——切换瞬间是一个**离散事件**，需要单独的数学对象来刻画。

> **类比（标边界）**：模式切换的冲击与**弹性碰撞**类似——两者都在极短时间内通过冲量改变速度。相似之处：都用冲量-动量定理 $\boldsymbol{\Lambda} = \Delta(\text{动量})$ 描述；都假设位置在碰撞瞬间不变。**不同之处**：弹性碰撞由恢复系数 $e$ 决定反弹速度（$v^+ = -e v^-$），而机器人足端落地通常建模为**完全非弹性碰撞**（$e = 0$，落地后接触点速度归零，不反弹）。**不要把类比延伸到**：弹性碰撞是被动的物理过程，而机器人落地后接触点速度归零是由**约束**强制的——背后是 WBC 主动施加的接触力配合地面反力共同作用的结果，不是单纯的被动反弹。

### 混合系统的三要素：flow、guard、reset

借用混合动力系统（hybrid dynamical system）的标准框架，轮足系统由三个数学对象完整描述：

| 要素 | 符号 | 含义 | 轮足实例 |
|------|------|------|---------|
| **flow（连续流）** | $\dot{\mathbf{x}} = f_\sigma(\mathbf{x}, \mathbf{u})$ | 模式段内的连续演化 | 77.2 的 centroidal + 轮速积分 |
| **guard（切换面）** | $\mathcal{G}_{\sigma \to \sigma'} = \{\mathbf{x} : g(\mathbf{x}) = 0\}$ | 触发切换的状态集 | 足端高度 $h_{\text{foot}}(\mathbf{x}) = 0$（触地） |
| **reset map（重置映射）** | $\mathbf{x}^+ = \Delta_{\sigma \to \sigma'}(\mathbf{x}^-)$ | 切换瞬间的状态跳变 | 速度投影到约束流形 |

flow 在 77.2 已经讲透。这一节聚焦后两者——guard 决定**何时**切换，reset map 决定切换时状态**如何**跳变。

**guard 的物理含义**：guard 是状态空间中的一张超曲面。当连续轨迹 $\mathbf{x}(t)$ 撞到这张曲面时，触发模式切换。对 swing $\to$ foot，guard 是"足端触地"事件：

$$
\mathcal{G}_{\text{swing} \to \text{foot}} = \{\mathbf{x} : h_{\text{foot}}(\mathbf{x}) = 0,\ \dot{h}_{\text{foot}}(\mathbf{x}) < 0\}
$$

第二个条件 $\dot{h}_{\text{foot}} < 0$（足端正在向下运动）很关键——它排除了足端正在离地的情况（那应该是 foot→swing，用另一个 guard）。

### reset map 的推导：投影到约束流形

reset map 回答：落地瞬间速度如何跳变？设落地前广义速度为 $\mathbf{v}^-$，落地后为 $\mathbf{v}^+$。两个条件决定 $\mathbf{v}^+$：

**条件 1（冲量-动量定理）**：冲量只通过接触雅可比作用：

$$
M(\mathbf{q})(\mathbf{v}^+ - \mathbf{v}^-) = J_c^T \boldsymbol{\Lambda}
$$

**条件 2（落地后约束满足）**：完全非弹性碰撞，接触点落地后速度为零：

$$
J_c \mathbf{v}^+ = \mathbf{0}
$$

这是两个未知量（$\mathbf{v}^+$ 和 $\boldsymbol{\Lambda}$）的线性方程组。从条件 1 解出 $\mathbf{v}^+ = \mathbf{v}^- + M^{-1} J_c^T \boldsymbol{\Lambda}$，代入条件 2：

$$
J_c \mathbf{v}^- + J_c M^{-1} J_c^T \boldsymbol{\Lambda} = \mathbf{0}
\;\Rightarrow\;
\boldsymbol{\Lambda} = -(J_c M^{-1} J_c^T)^{-1} J_c \mathbf{v}^-
$$

回代得到 reset map 的显式形式：

$$
\mathbf{v}^+ = \underbrace{\left[ \mathbf{I} - M^{-1} J_c^T (J_c M^{-1} J_c^T)^{-1} J_c \right]}_{\text{动力学一致投影矩阵 } \mathbf{P}_M} \mathbf{v}^-
$$

> **阶段小结**：到这里我们推导出 reset map 就是把落地前速度 $\mathbf{v}^-$ 经过投影矩阵 $\mathbf{P}_M$ 投影到约束流形 $\{J_c \mathbf{v} = 0\}$。接下来分析这个投影的能量含义。

矩阵 $\mathbf{P}_M$ 是关于度量 $M$ 的正交投影——它把速度投影到接触约束的零空间（满足 $J_c \mathbf{v} = 0$ 的子空间）。位置不变（$\mathbf{q}^+ = \mathbf{q}^-$），只有速度跳变。

> **本质洞察**：落地 reset map 的投影矩阵 $\mathbf{P}_M = \mathbf{I} - M^{-1}J_c^T(J_c M^{-1}J_c^T)^{-1}J_c$ 与复合/20、足式/90 中 WBC 的接触零空间投影**形式完全相同**。这不是巧合——落地碰撞和 WBC 接触约束本质上都是"把运动限制到接触流形上"。区别仅在于：碰撞是在速度层（$\mathbf{v}$）做投影且发生在零时间内，WBC 是在加速度层（$\dot{\mathbf{v}}$）做投影且持续作用。掌握了零空间投影这一个工具，就同时理解了碰撞动力学和约束控制两件事。

### 切换的能量损失

完全非弹性碰撞会损失动能。落地前动能 $T^- = \frac{1}{2}(\mathbf{v}^-)^T M \mathbf{v}^-$，落地后 $T^+ = \frac{1}{2}(\mathbf{v}^+)^T M \mathbf{v}^+$。可以证明损失量为：

$$
\Delta T = T^- - T^+ = \frac{1}{2} (J_c \mathbf{v}^-)^T (J_c M^{-1} J_c^T)^{-1} (J_c \mathbf{v}^-) \geq 0
$$

损失正比于落地前接触点速度 $J_c \mathbf{v}^-$ 的平方。这给出一个直接的工程指导：**落地前接触点速度越小，能量损失越小，冲击越柔和**。这正是摆动轨迹规划要求"足端在触地前竖直速度趋近于零"的根本原因。

> **反事实推理**：如果 MPC 完全忽略 reset map（假设速度连续），会怎样？MPC 规划的轨迹会假设落地后速度等于落地前速度。但真机的落地速度被物理投影到了约束流形——两者不一致。这个不一致表现为：（1）MPC 预测的质心轨迹与实际轨迹在每次落地后偏离，预测误差累积；（2）落地冲击产生的能量损失没有被代价函数考虑，MPC 倾向于规划高速触地的摆动轨迹，加剧冲击。把 reset map 纳入 MPC（哪怕只在代价函数中惩罚 $\|J_c \mathbf{v}^-\|^2$）能显著改善落地平顺性。

### 轮足切换的特殊性：foot 与 wheel 互切

swing $\to$ foot 是经典足式问题。轮足新增的是 **foot $\leftrightarrow$ wheel** 切换，它与足式切换有本质区别：

| 切换类型 | guard 条件 | reset map | 冲量 |
|---------|-----------|-----------|------|
| swing $\to$ foot | 足端触地 $h=0$ | 速度投影到 $J_c\mathbf{v}=0$ | 有（落地冲击） |
| foot $\to$ wheel | 控制决策触发 | 速度投影到 Pfaffian 流形 | 通常很小 |
| wheel $\to$ foot | 滑移/地形触发 | 投影到 $e_\parallel^T J_c\mathbf{v}=0$ | 取决于轮速 |

foot $\to$ wheel 切换时，接触点已经在地面上（位置不变），切换的是**约束类型**：从"接触点完全静止"（$J_c\mathbf{v}=0$，3 个约束）放松为"接触点只在横向和法向静止、纵向可滚动"（Pfaffian，仍是 3 个约束但纵向那行变成 $e_\parallel^T J_c \mathbf{v} = r\omega$）。

关键观察：如果切换瞬间轮速参考 $\omega$ 设为使 $r\omega = e_\parallel^T J_c \mathbf{v}^-$（即让轮的线速度匹配当前接触点纵向速度），那么 reset map 退化为恒等映射——**没有速度跳变，没有冲量**。这就是 77.4 节"速度连续性过渡窗口"的数学本质：通过 $\alpha(t)$ 平滑调整 $\omega$，使切换前后接触点纵向速度连续，从而避免冲量。

> **不是 X 而是 Y**：foot→wheel 的平滑切换不是"约束的渐变"（约束方程是离散切换的，不存在中间形态），而是"让切换瞬间的 reset map 成为恒等映射"。我们无法让约束本身连续变化，但可以通过选择切换时机和轮速参考，使得在该时机下状态恰好已经满足新约束——从而让跳变幅度为零。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：MPC 离散化网格点与 guard 触发时刻不对齐**
> - 错误做法：固定时间网格离散化（如每 50 ms 一个节点），假设切换恰好发生在网格点上
> - 典型现象：实际落地发生在两个网格点之间，MPC 在错误的时刻施加约束，预测轨迹与实际偏离，KKT 残差跳变
> - 根本原因：guard 触发时刻 $t_{\text{switch}}$ 由状态决定（$h_{\text{foot}}(\mathbf{x})=0$），一般不落在固定网格上。这是"事件驱动"与"时间驱动"离散化的根本矛盾
> - 正确做法：使用 OCS2 的 event-time 机制，把切换时刻作为额外的离散化节点插入，或使用变步长积分器在 guard 处自动加密网格

> 💡 **概念误区：认为 reset map 会改变机器人位置**
> - 新手想法："碰撞那一瞬间机器人被撞得移动了一下"
> - 实际上：reset map 只改变**速度**，不改变**位置**（$\mathbf{q}^+ = \mathbf{q}^-$）。这是冲量假设的直接结论——冲量作用时间趋于零，位置积分 $\int \mathbf{v}\, dt \to 0$。位置在碰撞瞬间是连续的，只有速度不连续
> - 关键区别：把"碰撞导致位置变化"和"碰撞后速度变化导致后续位置变化"分开——前者错误，后者正确

> 🧠 **思维陷阱：把所有切换都当作有冲击的碰撞**
> - 新手想法："既然 swing $\to$ foot 有冲击，那所有模式切换都要算冲量"
> - 实际上：只有当切换**激活新的速度约束**且当前速度**不满足**该约束时才有冲量。foot $\to$ swing（释放约束）没有冲量——约束消失不会产生冲击力。foot $\to$ wheel 若提前匹配轮速也没有冲量
> - 正确思维：先判断"切换后的约束当前满足吗？"——满足则 reset 是恒等映射（无冲击），不满足则需要投影（有冲击）

### 练习 77.2b

**[A] ⭐⭐⭐** 对一个 2 自由度平面单腿模型（质量 $m$ 集中在足端，雅可比 $J_c = \mathbf{I}_2$），手动计算落地前速度 $\mathbf{v}^- = (1.0, -0.5)^T$ m/s 时的 reset map 输出 $\mathbf{v}^+$ 和能量损失 $\Delta T$。假设落地后约束为 $\mathbf{v}^+ = \mathbf{0}$（双向约束）。

**[B] ⭐⭐⭐⭐** 证明 $\mathbf{P}_M$ 是关于度量 $M$ 的投影矩阵——即 $\mathbf{P}_M^2 = \mathbf{P}_M$，且 $\mathbf{P}_M$ 在内积 $\langle \mathbf{a}, \mathbf{b}\rangle_M = \mathbf{a}^T M \mathbf{b}$ 下自伴随。这与复合/20 中 WBC 零空间投影的性质有何联系？

**[C] ⭐⭐⭐** 设计一个 foot→wheel 的无冲击切换策略：给定切换前接触点纵向速度 $v_\parallel^-$ 和轮半径 $r$，应如何设置切换瞬间的轮速 $\omega$？如果轮速电机无法瞬间达到该值，过渡窗口的最短时间由什么决定？

---

## 77.3 混合 MPC 的状态空间与代价函数 ⭐⭐⭐

### 这一节解决什么问题

有了动力学模型，下一步是定义 MPC 的完整优化问题——状态向量、输入向量、代价函数和约束集。本节给出轮足 MPC 的完整数学形式。

### 状态向量定义

轮足 MPC 的状态向量需要包含质心动力学状态和轮状态：

$$
\mathbf{x} = \begin{pmatrix} \mathbf{h}_G \\ \mathbf{q} \end{pmatrix} = \begin{pmatrix} \mathbf{k}_G \\ \mathbf{l}_G \\ \mathbf{q}_b \\ \mathbf{q}_\ell \\ \boldsymbol{\theta}_w \end{pmatrix} \in \mathbb{R}^{n_x}
$$

| 分量 | 维度 | 含义 |
|------|------|------|
| $\mathbf{k}_G$ | 3 | 绕质心的角动量 |
| $\mathbf{l}_G$ | 3 | 线动量 $= m\dot{\mathbf{p}}_G$ |
| $\mathbf{q}_b$ | 7 | 基座位姿（位置 3 + 四元数 4） |
| $\mathbf{q}_\ell$ | 12 | 腿关节角 |
| $\boldsymbol{\theta}_w$ | 4 | 轮转角（纯足式不存在此项） |

总状态维度：$n_x = 6 + 7 + 12 + 4 = 29$。对比纯足式：$n_x = 6 + 7 + 12 = 25$，多出 4 维。

### 输入向量定义

$$
\mathbf{u} = \begin{pmatrix} \boldsymbol{\lambda}_1 \\ \vdots \\ \boldsymbol{\lambda}_4 \\ \boldsymbol{\omega}_w^{\text{ref}} \\ \boldsymbol{\tau}_\ell \end{pmatrix} \in \mathbb{R}^{n_u}
$$

| 分量 | 维度 | 含义 |
|------|------|------|
| $\boldsymbol{\lambda}_i$ | $3 \times 4 = 12$ | 四个末端的接触力 |
| $\boldsymbol{\omega}_w^{\text{ref}}$ | 4 | 轮速参考（纯足式不存在此项） |
| $\boldsymbol{\tau}_\ell$ | 12 | 腿关节力矩参考 |

总输入维度：$n_u = 12 + 4 + 12 = 28$。

### 代价函数设计

轮足 MPC 的代价函数由多个子项组成，每个子项对应一个控制目标：

$$
\ell_m(\mathbf{x}, \mathbf{u}) = \underbrace{\ell_{\text{com}}}_{\text{质心跟踪}} + \underbrace{\ell_{\text{vel}}}_{\text{基座速度跟踪}} + \underbrace{\ell_{\omega}}_{\text{轮速跟踪}} + \underbrace{\ell_{\text{force}}}_{\text{力正则化}} + \underbrace{\ell_{\text{swing}}}_{\text{摆动腿跟踪}}
$$

各子项的具体形式：

**质心跟踪代价**：

$$
\ell_{\text{com}} = \|\mathbf{p}_G - \mathbf{p}_G^{\text{ref}}\|_{\mathbf{Q}_p}^2 + \|\dot{\mathbf{p}}_G - \dot{\mathbf{p}}_G^{\text{ref}}\|_{\mathbf{Q}_v}^2
$$

**基座角速度跟踪代价**：

$$
\ell_{\text{vel}} = \|\boldsymbol{\omega}_b - \boldsymbol{\omega}_b^{\text{ref}}\|_{\mathbf{Q}_\omega}^2
$$

**轮速匹配代价**（wheel 模式特有）：

$$
\ell_{\omega} = \sum_{i \in \mathcal{C}_{\text{wheel}}} w_\omega \|\omega_{w,i} - \omega_{w,i}^{\text{ref}}\|^2
$$

**力正则化代价**：

$$
\ell_{\text{force}} = \sum_i \|\boldsymbol{\lambda}_i\|_{\mathbf{R}_\lambda}^2
$$

这一项防止接触力过大——鼓励均匀的力分配。

### Q/R 权重矩阵设计：物理含义与调参指南 ⭐⭐⭐

权重矩阵不是凭感觉随意设置的数字。每个对角元素都对应一个物理量的偏差容忍度，权重的数量级由该量的物理单位和典型偏差共同决定。

**权重设计的核心原则**：权重 $w_i$ 的物理含义是"该状态偏差 1 个单位所产生的代价"。如果质心高度偏差 0.01 m 就已经很危险，但轮速偏差 1 rad/s 可以容忍，则 $Q_z$ 应远大于 $Q_\omega$。

下面给出完整的 $\mathbf{Q}$（状态权重）和 $\mathbf{R}$（输入权重）矩阵设计：

**状态权重矩阵 $\mathbf{Q}$**：

| 状态分量 | 权重符号 | 推荐值 | 物理含义 | 设计依据 |
|---------|---------|--------|---------|---------|
| $p_{G,x}$ | $Q_{px}$ | 50 | 纵向位置跟踪 | 允许 ~0.14 m 偏差（$1/\sqrt{50}$）|
| $p_{G,y}$ | $Q_{py}$ | 50 | 横向位置跟踪 | 同上 |
| $p_{G,z}$ | $Q_{pz}$ | 500 | 高度保持 | 允许 ~0.04 m 偏差；高度偏差直接影响稳定性 |
| $\dot{p}_{G,x}$ | $Q_{vx}$ | 20 | 纵向速度跟踪 | 允许 ~0.22 m/s 偏差 |
| $\dot{p}_{G,y}$ | $Q_{vy}$ | 20 | 横向速度跟踪 | 同上 |
| $\dot{p}_{G,z}$ | $Q_{vz}$ | 100 | 竖直速度抑制 | 竖直速度应接近零 |
| $\phi$ (roll) | $Q_\phi$ | 300 | roll 角跟踪 | 允许 $\sim 3.3^\circ$ 偏差；过大 roll 导致侧翻 |
| $\theta$ (pitch) | $Q_\theta$ | 300 | pitch 角跟踪 | 同上 |
| $\psi$ (yaw) | $Q_\psi$ | 100 | yaw 角跟踪 | 可容忍更大偏差 |
| $\omega_x$ | $Q_{\omega x}$ | 10 | roll 角速度 | 抑制摆动 |
| $\omega_y$ | $Q_{\omega y}$ | 10 | pitch 角速度 | 同上 |
| $\omega_z$ | $Q_{\omega z}$ | 5 | yaw 角速度 | yaw 变化通常更平缓 |
| $\theta_{w,i}$ | $Q_{\theta w}$ | 0.1 | 轮转角 | 转角本身不重要，轮速才重要 |

**输入权重矩阵 $\mathbf{R}$**：

| 输入分量 | 权重符号 | 推荐值 | 物理含义 | 设计依据 |
|---------|---------|--------|---------|---------|
| $\lambda_{n,i}$ | $R_n$ | $10^{-4}$ | 法向接触力 | 法向力是维持支撑的必要力，不应过度惩罚 |
| $\lambda_{t,i}$ | $R_t$ | $10^{-3}$ | 切向接触力 | 切向力消耗摩擦裕度，适度惩罚 |
| $\omega_{w,i}^{\text{ref}}$ | $R_\omega$ | $10^{-2}$ | 轮速参考 | 惩罚过高轮速，防止电机饱和 |
| $\tau_{\ell,i}$ | $R_\tau$ | $10^{-5}$ | 腿关节力矩 | 轻微惩罚力矩减少能耗 |

> **本质洞察**：$\mathbf{Q}$ 矩阵中各项的数量级关系反映了控制系统的优先级层次——高度和姿态角（$Q_z = 500$, $Q_\phi = 300$）远高于轮转角（$Q_{\theta w} = 0.1$），这表达了"基座稳定远比轮转角精确更重要"的工程判断。$\mathbf{R}$ 矩阵中法向力权重最小（$R_n = 10^{-4}$），因为法向力是维持站立的必要开销，不应该被"节省"。

**为什么不能用统一权重？** 如果所有 $Q$ 设为相同值（如 $Q_i = 100$），会发生什么？高度偏差 0.1 m（非常危险）和轮转角偏差 0.1 rad（完全无害）产生相同代价。MPC 可能为了减小轮转角偏差而牺牲高度跟踪——这是本末倒置。每个权重必须反映该量偏差的真实"危险程度"。

**调参流程**：

1. **首先确定安全关键量**：$Q_{pz}$、$Q_\phi$、$Q_\theta$ 设为最高值
2. **然后设定跟踪量**：$Q_{px}$、$Q_{py}$、$Q_{vx}$ 设为中等值
3. **最后设定辅助量**：$Q_{\theta w}$ 设为最小值
4. **输入权重**从小开始调：先让 MPC 自由使用力，观察行为后逐步增大正则化
5. **验证方法**：画出 MPC 开环预测的状态和输入时间线，检查是否有量被过度惩罚或不足惩罚

**模式相关的代价权重**：

关键设计决策是代价权重随模式变化：

| 代价项 | swing 模式 | foot 模式 | wheel 模式 |
|--------|-----------|-----------|-----------|
| 接触力正则化 | $w_\lambda = 0$（无力）| $w_\lambda = 1$ | $w_\lambda = 1$ |
| 轮速跟踪 | $w_\omega = 0$ | $w_\omega = 0$ | $w_\omega = 10$（重要）|
| 摆动轨迹跟踪 | $w_{\text{swing}} = 100$（重要）| $w_{\text{swing}} = 0$ | $w_{\text{swing}} = 0$ |
| 足端位置保持 | $w_{\text{foot}} = 0$ | $w_{\text{foot}} = 50$ | $w_{\text{foot}} = 0$ |

> **本质洞察**：轮足 MPC 的代价函数不是一个固定的二次型——它是一个**模式相关的分段二次型**。权重矩阵 $\mathbf{Q}_m, \mathbf{R}_m$ 随模式 $m$ 变化。这种模式依赖性使得 MPC 能够在不同模式下追求不同的优化目标——wheel 模式强调速度匹配，foot 模式强调位置保持，swing 模式强调轨迹跟踪。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：所有模式使用相同的代价权重**
> - 错误做法：wheel 模式和 foot 模式使用相同的接触力权重
> - 典型现象：wheel 模式下接触力分配不合理——纵向力过大导致打滑，或横向力过大导致侧翻
> - 根本原因：wheel 模式的纵向力受轮驱动能力限制（$|\tau_w| \leq \tau_{\max}$），横向力受横向摩擦限制。两个方向的"代价"应该不同
> - 正确做法：为 wheel 模式设计各向异性的力正则化权重——纵向力权重低（鼓励使用纵向驱动力），横向力权重高（惩罚横向力，防止侧滑）

> 💡 **概念误区：认为轮速参考 $\omega_w^{\text{ref}}$ 应该直接由速度命令计算**
> - 新手想法："目标速度 1 m/s，轮半径 0.05 m，轮速参考 = 1/0.05 = 20 rad/s"
> - 实际上：这种**前馈**计算忽略了转弯差速——内外侧轮的目标速度不同。而且 MPC 的优势正是让优化器**自主决定**最优轮速，而非人工预设。$\omega_w^{\text{ref}}$ 应该作为**决策变量**由 MPC 优化，而非固定输入
> - 延伸：前馈轮速可以作为**热启动**的初值，但不应作为硬约束

### 练习 77.3

**[A] ⭐⭐⭐** 写出轮足 MPC 的完整离散时间 OCP（状态方程 + 代价函数 + 约束）。使用 OCS2 风格的 $(h_G, q)$ 状态表示。

**[B] ⭐⭐⭐** 设计一组合理的代价权重。基座速度跟踪权重、轮速匹配权重和力正则化权重的数量级关系应该是什么？（提示：考虑各项的物理单位和典型数值范围）

**[C] ⭐⭐⭐⭐** 讨论：如果将模式选择也作为 MPC 的优化变量（即整数决策变量 $m_i \in \{0, 1, 2\}$），OCP 变成什么类型的优化问题？这种问题的计算复杂度是什么？

### 约束堆叠：从动力学到 QP 的完整组装 ⭐⭐⭐⭐

上面定义了代价函数和权重。但 MPC 求解器需要的是标准 QP 形式。本节展示如何把动力学方程、摩擦锥、关节限位和 Pfaffian 约束堆叠成一个可以直接交给 HPIPM 或 qpOASES 的 QP。

**Step 1：离散化状态方程**

用 Euler 积分将连续动力学 $\dot{\mathbf{x}} = f_\sigma(\mathbf{x}, \mathbf{u})$ 离散化为：

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f_\sigma(\mathbf{x}_k, \mathbf{u}_k)
$$

在当前工作点 $(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k)$ 处线性化：

$$
\mathbf{x}_{k+1} \approx \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k + \mathbf{d}_k
$$

其中 $\mathbf{A}_k = \mathbf{I} + \Delta t \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k}$，$\mathbf{B}_k = \Delta t \frac{\partial f}{\partial \mathbf{u}}\bigg|_{\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k}$，$\mathbf{d}_k$ 是仿射项。

对轮足系统，$\mathbf{A}_k \in \mathbb{R}^{29 \times 29}$，$\mathbf{B}_k \in \mathbb{R}^{29 \times 28}$。

**Step 2：定义决策变量向量**

将所有时步的状态和输入堆叠为一个大向量：

$$
\mathbf{z} = \begin{pmatrix} \mathbf{x}_0 \\ \mathbf{u}_0 \\ \mathbf{x}_1 \\ \mathbf{u}_1 \\ \vdots \\ \mathbf{x}_{N-1} \\ \mathbf{u}_{N-1} \\ \mathbf{x}_N \end{pmatrix} \in \mathbb{R}^{(N+1) n_x + N n_u}
$$

对 $n_x = 29$, $n_u = 28$, $N = 20$：$\dim(\mathbf{z}) = 21 \times 29 + 20 \times 28 = 609 + 560 = 1169$。

**Step 3：堆叠代价函数为二次型**

代价函数 $J = \sum_{k=0}^{N-1} \ell_k(\mathbf{x}_k, \mathbf{u}_k) + \Phi(\mathbf{x}_N)$ 在工作点展开后变为：

$$
J \approx \frac{1}{2} \mathbf{z}^T \mathbf{H} \mathbf{z} + \mathbf{c}^T \mathbf{z}
$$

其中 $\mathbf{H}$ 是块对角矩阵：

$$
\mathbf{H} = \text{blkdiag}(\mathbf{Q}_0, \mathbf{R}_0, \mathbf{Q}_1, \mathbf{R}_1, \ldots, \mathbf{Q}_{N-1}, \mathbf{R}_{N-1}, \mathbf{Q}_N)
$$

这里 $\mathbf{Q}_k$ 可能随模式 $\sigma_k$ 变化——这正是轮足 MPC 与纯足式的关键区别。

**Step 4：堆叠等式约束**

等式约束包含两类：

（1）**动力学约束**（$N$ 条）：

$$
\mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k - \mathbf{x}_{k+1} + \mathbf{d}_k = 0
$$

（2）**Pfaffian 滚动约束**（对 wheel 模式的腿，每步每腿 3 条）：

$$
\mathbf{P}_k(\mathbf{x}_k, \mathbf{u}_k) = 0
$$

线性化后为 $\mathbf{C}_k^{\text{roll}} \mathbf{x}_k + \mathbf{D}_k^{\text{roll}} \mathbf{u}_k + \mathbf{e}_k^{\text{roll}} = 0$。

将所有等式约束堆叠为：

$$
\mathbf{C}_{eq} \mathbf{z} = \mathbf{d}_{eq}
$$

**Step 5：堆叠不等式约束**

不等式约束包含三类：

（1）**摩擦锥约束**（每步每接触腿 $k_{\text{fric}}$ 条线性近似）：

foot 模式使用圆锥的 $k$-面体近似（通常 $k = 8$）：

$$
\mathbf{F}_{\text{fric}} \boldsymbol{\lambda}_i \leq 0
$$

wheel 模式使用椭圆的线性近似。

（2）**关节力矩限幅**：

$$
-\boldsymbol{\tau}_{\max} \leq \boldsymbol{\tau}_\ell \leq \boldsymbol{\tau}_{\max}
$$

（3）**关节位置限位**（如有）：

$$
\mathbf{q}_{\min} \leq \mathbf{q}_\ell \leq \mathbf{q}_{\max}
$$

将所有不等式约束堆叠为：

$$
\mathbf{C}_{ineq} \mathbf{z} \leq \mathbf{d}_{ineq}
$$

**最终 QP 形式**：

$$
\min_{\mathbf{z}} \frac{1}{2} \mathbf{z}^T \mathbf{H} \mathbf{z} + \mathbf{c}^T \mathbf{z}
$$

$$
\text{s.t.} \quad \mathbf{C}_{eq} \mathbf{z} = \mathbf{d}_{eq}, \quad \mathbf{C}_{ineq} \mathbf{z} \leq \mathbf{d}_{ineq}
$$

其中 $\mathbf{H} \in \mathbb{R}^{1169 \times 1169}$，$\mathbf{C}_{eq}$ 约 $N(n_x + n_{\text{roll}}) \approx 20 \times (29 + 12) = 820$ 行，$\mathbf{C}_{ineq}$ 约 $N(k_{\text{fric}} \times 4 + 2 n_a) \approx 20 \times (32 + 24) = 1120$ 行。

> **反事实推理**：如果不用 HPIPM 的带状结构求解器，而是直接用稠密 QP 求解器（如 qpOASES），会怎样？$\mathbf{H}$ 的稠密存储需要 $1169^2 \approx 1.37 \times 10^6$ 个双精度浮点数，分解复杂度 $O(1169^3) \approx 1.6 \times 10^9$ 次浮点运算。而 HPIPM 利用 $\mathbf{H}$ 的块对角结构和 $\mathbf{C}_{eq}$ 的带状结构，复杂度降为 $O(N(n_x + n_u)^3) \approx 20 \times 57^3 \approx 3.7 \times 10^6$——快了约 430 倍。这就是为什么实时 MPC 必须使用结构化 QP 求解器。

### 数值算例：4 步 MPC 预测 ⭐⭐⭐

为了让上述抽象矩阵变得具体，下面展示一个**简化的 4 步数值算例**。使用 2D 平面质心模型（$n_x = 4$: 位置 $x,z$、速度 $\dot{x},\dot{z}$；$n_u = 2$: 水平力 $F_x$、竖直力 $F_z$），单腿 wheel 模式。

**系统参数**：$m = 12$ kg，$g = 9.81$ m/s$^2$，$\Delta t = 0.05$ s，$\mu = 0.7$，$r = 0.05$ m。

**离散化状态方程**（Euler 积分）：

$$
\mathbf{A} = \begin{pmatrix} 1 & 0 & 0.05 & 0 \\ 0 & 1 & 0 & 0.05 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}, \quad
\mathbf{B} = \begin{pmatrix} 0 & 0 \\ 0 & 0 \\ 0.05/12 & 0 \\ 0 & 0.05/12 \end{pmatrix} = \begin{pmatrix} 0 & 0 \\ 0 & 0 \\ 0.00417 & 0 \\ 0 & 0.00417 \end{pmatrix}
$$

$$
\mathbf{d} = \begin{pmatrix} 0 \\ 0 \\ 0 \\ -0.05 \times 9.81 \end{pmatrix} = \begin{pmatrix} 0 \\ 0 \\ 0 \\ -0.4905 \end{pmatrix}
$$

**权重矩阵**：

$$
\mathbf{Q} = \text{diag}(50, 500, 20, 100), \quad \mathbf{R} = \text{diag}(10^{-3}, 10^{-4})
$$

高度权重（$Q_{22} = 500$）是水平位置权重（$Q_{11} = 50$）的 10 倍——反映了"保持高度比保持水平位置更重要"。

**初始状态**：$\mathbf{x}_0 = (0, 0.35, 1.0, 0)^T$（高度 0.35 m，前进速度 1.0 m/s）。

**目标状态**：$\mathbf{x}^{\text{ref}} = (0, 0.35, 1.0, 0)^T$（保持高度和速度不变）。

**摩擦锥约束**（4 面体近似）：

$$
\begin{pmatrix} 1 & -\mu \\ -1 & -\mu \end{pmatrix} \begin{pmatrix} F_x \\ F_z \end{pmatrix} \leq \begin{pmatrix} 0 \\ 0 \end{pmatrix}
$$

即 $|F_x| \leq 0.7 F_z$，同时 $F_z \geq 0$。

**决策变量**：$\mathbf{z} = (\mathbf{x}_0, \mathbf{u}_0, \mathbf{x}_1, \mathbf{u}_1, \mathbf{x}_2, \mathbf{u}_2, \mathbf{x}_3, \mathbf{u}_3, \mathbf{x}_4)^T \in \mathbb{R}^{5 \times 4 + 4 \times 2} = \mathbb{R}^{28}$。

求解这个 QP 后可以得到：

| 步 $k$ | $F_x$ (N) | $F_z$ (N) | 高度 (m) | 速度 (m/s) |
|--------|-----------|-----------|---------|-----------|
| 0 | $\approx 0$ | $\approx 117.7$ | 0.350 | 1.000 |
| 1 | $\approx 0$ | $\approx 117.7$ | 0.350 | 1.000 |
| 2 | $\approx 0$ | $\approx 117.7$ | 0.350 | 1.000 |
| 3 | $\approx 0$ | $\approx 117.7$ | 0.350 | 1.000 |

在匀速直行场景下，最优解是保持恒定法向力 $F_z = mg = 117.7$ N 以抵消重力，水平力 $F_x \approx 0$。这验证了 QP 的正确性——稳态下 MPC 应退化为力平衡。

如果目标速度变为 1.5 m/s（加速），则 $F_x > 0$，且摩擦锥约束 $|F_x| \leq 0.7 F_z \approx 82.4$ N 限制了最大加速度为 $F_x / m \approx 6.87$ m/s$^2$。

```python
import numpy as np
from scipy.optimize import minimize

def simple_wheel_mpc_demo():
    """简化 2D 轮足 MPC 的 4 步数值验证。"""
    m, g, dt = 12.0, 9.81, 0.05
    N = 4
    nx, nu = 4, 2
    
    A = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    B = np.array([[0, 0],
                  [0, 0],
                  [dt/m, 0],
                  [0, dt/m]])
    d = np.array([0, 0, 0, -g * dt])
    
    Q = np.diag([50.0, 500.0, 20.0, 100.0])
    R = np.diag([1e-3, 1e-4])
    x_ref = np.array([0.0, 0.35, 1.0, 0.0])
    x0 = np.array([0.0, 0.35, 1.0, 0.0])
    mu = 0.7
    
    # 中文注释：组装二次型代价矩阵 H 和线性项 c
    H = np.zeros((N*(nx+nu)+nx, N*(nx+nu)+nx))
    c_vec = np.zeros(N*(nx+nu)+nx)
    for k in range(N):
        idx_x = k * (nx + nu)
        idx_u = idx_x + nx
        H[idx_x:idx_x+nx, idx_x:idx_x+nx] = Q
        H[idx_u:idx_u+nu, idx_u:idx_u+nu] = R
        c_vec[idx_x:idx_x+nx] = -Q @ x_ref
    # 终端代价
    idx_xN = N * (nx + nu)
    H[idx_xN:idx_xN+nx, idx_xN:idx_xN+nx] = Q
    c_vec[idx_xN:idx_xN+nx] = -Q @ x_ref
    
    print(f"决策变量维度: {H.shape[0]}")
    print(f"初始状态: {x0}")
    print(f"MPC 稳态法向力 mg = {m * g:.1f} N")

simple_wheel_mpc_demo()
```

这个简化示例展示了完整 QP 组装的逻辑。真实轮足 MPC 的 QP 维度大约 40 倍于此，但结构完全相同——只是 $\mathbf{A}$、$\mathbf{B}$ 更大，约束更多。

---

## 77.4 接触序列与模式调度 ⭐⭐⭐

### 这一节解决什么问题

77.3 节假设模式序列 $\sigma(t) = (m_1(t), m_2(t), m_3(t), m_4(t))$ 是预先给定的。但谁来决定这个序列？本节讨论三种模式调度策略。

### 启发式模式调度

最简单的策略是基于规则的启发式：

```python
def heuristic_mode_scheduler(terrain_info, velocity_cmd, slip_state):
    """
    启发式模式调度器。
    输入：地形信息、速度命令、滑移状态
    输出：四个末端的模式
    """
    modes = ['wheel'] * 4  # 默认全轮
    
    for i in range(4):
        # 规则1：障碍物前方切换为 foot/swing
        if terrain_info[i].obstacle_distance < 0.3:
            modes[i] = 'swing'
        
        # 规则2：台阶处切换为 foot
        elif terrain_info[i].step_height > 0.05:
            modes[i] = 'foot'
        
        # 规则3：滑移过大时切换为 foot
        elif slip_state[i].slip_ratio > 0.3:
            modes[i] = 'foot'
        
        # 规则4：低速时 wheel 更高效
        elif abs(velocity_cmd.linear_x) < 0.1:
            modes[i] = 'foot'  # 低速站立用 foot 更稳
    
    return modes
```

启发式的优点是简单、可解释、确定性。缺点是无法发现非直觉的混合模式（如"前轮滚动+后腿跨越"）。

### 模式切换的工程约束

无论使用何种调度策略，模式切换都必须满足以下工程约束：

**约束 1：最短驻留时间**

$$
T_{\text{dwell}} > T_{\text{solve}} + T_{\text{comm}} + T_{\text{transition}}
$$

其中 $T_{\text{solve}}$ 是 MPC 求解时间（~5-20 ms），$T_{\text{comm}}$ 是通信延迟（~1-5 ms），$T_{\text{transition}}$ 是约束过渡时间（~50-100 ms）。

典型 $T_{\text{dwell}} \geq 100$ ms。切换太频繁会导致优化器无法收敛——因为约束结构在收敛之前就变了。

**约束 2：滞回切换**

模式切换条件必须有**滞回**（hysteresis）——正向切换和反向切换的阈值不同。否则在边界附近会来回抖动。

```python
class ModeGateWithHysteresis:
    def __init__(self, engage_threshold, disengage_threshold, min_dwell_s):
        self.engage = engage_threshold     # 正向切换阈值（如 slip > 0.3）
        self.disengage = disengage_threshold  # 反向切换阈值（如 slip < 0.1）
        self.min_dwell = min_dwell_s
        self.last_switch_time = 0.0
        self.current_mode = 'wheel'
    
    def update(self, now, slip, terrain_score):
        # 最短驻留时间检查
        if now - self.last_switch_time < self.min_dwell:
            return self.current_mode
        
        # 滞回切换逻辑
        if self.current_mode == 'wheel':
            if slip > self.engage or terrain_score < -0.5:
                self.current_mode = 'foot'
                self.last_switch_time = now
        else:  # foot
            if slip < self.disengage and terrain_score > 0.5:
                self.current_mode = 'wheel'
                self.last_switch_time = now
        
        return self.current_mode
```

**约束 3：速度连续性**

切换瞬间，接触点速度不能跳变。从 foot（$v_c = 0$）到 wheel（$v_c^{\text{long}} = r\omega$），需要一个平滑过渡窗口：

$$
v_c^{\text{ref}}(t) = \alpha(t) \cdot r\omega + (1 - \alpha(t)) \cdot 0
$$

其中 $\alpha(t)$ 从 0 平滑过渡到 1，过渡时间 $\sim 50$-$100$ ms。

> **反事实推理**：如果模式切换时不加过渡窗口会怎样？约束从 $J_c v = 0$ 突变为 $e_\parallel^T J_c v = r\omega$，等价于接触点参考速度瞬间跳变。WBC 为了满足这个跳变的参考，会产生**力矩尖峰**——因为要在一个控制周期（1-2 ms）内让接触点从静止加速到 $r\omega$。这个力矩尖峰可能超过电机限幅，导致执行器饱和和控制失败。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：模式消息没有时间戳**
> - 错误做法：MPC 和 WBC 通过共享内存读取同一个模式标志位
> - 典型现象：MPC 在第 $k$ 步用 wheel 模式优化，但 WBC 在第 $k$ 步收到的还是上一步的 foot 模式——两者不同步，产生矛盾的力/速度命令
> - 根本原因：MPC（~30 Hz）和 WBC（~500 Hz）运行在不同频率，模式切换的生效时间必须精确到时间戳
> - 正确做法：模式消息带绝对时间戳 $t_{\text{switch}}$，每个模块根据自己的时钟判断当前应使用哪个模式

> 🧠 **思维陷阱：追求"无缝切换"**
> - 新手想法："好的切换策略应该让切换瞬间完全无感——力、速度、加速度全部连续"
> - 实际上：力的完全连续在物理上不可能——从 foot（接触力可以有任意方向的切向力）到 wheel（切向力受轮驱动能力限制），力的可行域本身就不连续。工程上能做到的是力矩变化率有界（$|\dot{\tau}| \leq \dot{\tau}_{\max}$），即力矩不跳变但可以快速变化
> - 正确目标：追求"力矩变化率有界"而非"力连续"

### 采样规划与模式序列优化

启发式调度简单但保守。对于复杂地形（如混合台阶和坡面），需要**搜索**最优模式序列。Jelavic et al. (RSS 2021) 提出了一种将采样方法与 MPC 结合的方案：

**Step 1**：在模式空间 $\{0, 1, 2\}^4$ 上随机采样 $K$ 个候选模式序列

**Step 2**：对每个候选序列运行一次简化 MPC（使用 SRBD 而非全身模型），评估代价

**Step 3**：选择代价最低的序列作为实际 MPC 的模式输入

**Step 4**：下一个控制周期，用上一步的最优序列平移作为采样中心（类似热启动）

这种方法避免了枚举所有 $3^{4 \times N}$ 种组合，用采样近似全局搜索。典型的采样数 $K = 50$-$200$，配合简化 MPC 的快速求解（~1 ms/次），总计算时间 $\sim 50$-$200$ ms——可以在独立线程中异步运行。

### 学习策略的模式输出

Swiss-Mile 2024 的方法是用 RL 输出模式偏好，再由安全门控过滤。RL 的模式输出不是离散的 $\{0, 1, 2\}$，而是**连续的偏好概率**：

$$
p_i = \text{softmax}(\pi_\theta(o))_i, \quad i \in \{\text{swing}, \text{foot}, \text{wheel}\}
$$

安全门控检查偏好模式的可行性（是否有地面接触、滑移是否过大），然后做最终决策。这种架构允许 RL 学习地形自适应的模式策略，同时保留工程安全保证。

### 练习 77.4

**[A] ⭐⭐** 实现一个完整的 `ModeGateWithHysteresis` 类（包含滞回、最短驻留时间和速度连续性过渡）。用 matplotlib 绘制一个 10 秒的仿真：输入滑移比正弦波，输出模式时间线。

**[B] ⭐⭐⭐** 设计一个"地形得分"函数：输入为局部高程图的统计特征（粗糙度、最大高差、坡度），输出为 $[-1, 1]$ 的得分（1 = 适合 wheel，-1 = 适合 foot）。给出三种典型地形的得分。

**[C] ⭐⭐⭐** 讨论：如果使用 RL 策略输出模式偏好（类似 Swiss-Mile 2024 的方法），RL 输出是否应该直接决定模式？还是应该经过安全门控？给出你的架构设计和理由。

---

## 77.5 数值求解：SQP-RTI 与 OCS2 集成 ⭐⭐⭐⭐

### 这一节解决什么问题

轮足 MPC 的 OCP 是一个带切换的非线性优化问题。如何在 5-20 ms 内求解它？本节介绍 SQP-RTI 求解策略和在 OCS2 框架中的实现方式。

### OCS2 的切换系统接口

回顾足式/110\_OCS2完整栈与双线程MPC：OCS2 将足式机器人建模为 switched system。核心接口包括：

| OCS2 接口 | 功能 | 轮足扩展 |
|-----------|------|---------|
| `SystemDynamics` | 状态方程 $f_m(x, u)$ | 加入轮速积分方程 |
| `StateInputConstraint` | 等式约束 $g_m(x, u) = 0$ | 加入 Pfaffian 滚动约束 |
| `StateInputCost` | 代价函数 $\ell_m(x, u)$ | 加入轮速跟踪代价 |
| `ModeSchedule` | 模式时间线 $\sigma(t)$ | 从二值扩展到三值 |

### 滚动约束的 OCS2 实现

轮足 MPC 中最关键的新增组件是**滚动等式约束**。在 OCS2 中，它作为 `StateInputConstraint` 实现：

```cpp
// OCS2 风格的滚动约束（概念代码）
class RollingConstraint : public StateInputConstraint {
public:
    // 约束值：返回残差向量
    VectorXd getValue(scalar_t t, const VectorXd& x, 
                      const VectorXd& u, const PreComputation& pre) override {
        // 1. 从状态提取配置和速度
        auto q = extractConfig(x);
        auto v = extractVelocity(x, u);
        
        // 2. 计算接触雅可比
        pinocchio::computeJointJacobians(model_, data_, q);
        auto J_c = getContactJacobian(data_, wheelFrameId_);
        
        // 3. 计算局部坐标系
        auto [e_par, e_perp, n] = buildContactFrame(
            getBodyForward(data_), getTerrainNormal());
        
        // 4. 计算三维残差
        VectorXd residual(3);
        auto v_c = J_c * v;  // 接触点速度
        residual(0) = e_par.dot(v_c) - radius_ * u(wheelSpeedIdx_);  // 纵向
        residual(1) = e_perp.dot(v_c);                                // 横向
        residual(2) = n.dot(v_c);                                     // 法向
        
        return residual;
    }
    
    // 线性化近似：返回对状态和输入的导数
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t t, const VectorXd& x, const VectorXd& u, 
        const PreComputation& pre) override {
        // 解析导数或数值差分
        // 解析导数需要 dJ_c/dq，计算复杂但精确
        // 数值差分简单但慢且有截断误差
        // 推荐：先用数值差分验证，再迁移到解析或 CppAD
    }
};
```

### OCS2 集成完整流程：从头搭建轮足 MPC ⭐⭐⭐⭐

OCS2 框架的核心抽象是把最优控制问题拆成五个模块化组件。下面按照实际搭建顺序，逐步展示如何将轮足 MPC 接入 OCS2。

**Step 1：定义系统维度和模式**

```cpp
// OCS2 风格：定义轮足系统维度（概念代码）
constexpr int STATE_DIM = 29;    // 6(momentum) + 7(base) + 12(legs) + 4(wheels)
constexpr int INPUT_DIM = 28;    // 12(forces) + 4(wheel_speed) + 12(joint_torque)
constexpr int NUM_MODES = 81;    // 3^4 种模式组合

// 中文注释：模式编码——每条腿的模式用 0/1/2 表示
// mode_id = m1 * 27 + m2 * 9 + m3 * 3 + m4
// 其中 m_i ∈ {0=swing, 1=foot, 2=wheel}
int encode_mode(int m1, int m2, int m3, int m4) {
    return m1 * 27 + m2 * 9 + m3 * 3 + m4;
}
```

**Step 2：实现 SystemDynamics**

```cpp
// 中文注释：轮足动力学 = centroidal dynamics + 轮速积分
class WheelLeggedDynamics : public SystemDynamics {
public:
    VectorXd computeFlowMap(scalar_t t, const VectorXd& x, 
                            const VectorXd& u, 
                            const PreComputation& pre) override {
        VectorXd dx(STATE_DIM);
        
        // 1. centroidal 动力学（与纯足式相同）
        auto forces = extractContactForces(u);   // 12 维
        dx.head<6>() = centroidalDynamics(x, forces);
        
        // 2. 基座运动学（与纯足式相同）
        dx.segment<7>(6) = baseKinematics(x);
        
        // 3. 腿关节积分（与纯足式相同）
        dx.segment<12>(13) = extractJointVelocities(x);
        
        // 4. 轮速积分（轮足新增）
        auto wheel_torques = extractWheelTorques(u);
        for (int i = 0; i < 4; ++i) {
            dx(25 + i) = wheel_torques(i) / wheel_inertia_;
        }
        
        return dx;
    }
};
```

**Step 3：实现 ModeScheduleManager**

```cpp
// 中文注释：轮足模式调度需要支持三值模式切换
class WheelLeggedModeScheduleManager : public ModeScheduleManager {
public:
    // 关键区别：纯足式按步态周期切换，轮足按地形和规划切换
    ModeSchedule getModeSchedule(scalar_t t) override {
        // 从启发式调度器或 RL 策略获取模式序列
        auto modes = mode_scheduler_->getCurrentModes(t);
        
        // 构建 OCS2 的 event_times 和 mode_sequence
        // 中文注释：每次模式变化插入一个事件时间点
        std::vector<scalar_t> event_times;
        std::vector<size_t> mode_sequence;
        
        for (const auto& [switch_time, mode] : modes) {
            event_times.push_back(switch_time);
            mode_sequence.push_back(encode_mode(
                mode[0], mode[1], mode[2], mode[3]));
        }
        
        return {event_times, mode_sequence};
    }
};
```

**Step 4：注册约束和代价**

```cpp
// 中文注释：按模式注册不同的约束和代价
void setupProblem(OptimalControlProblem& problem) {
    // 动力学约束（所有模式共享）
    problem.dynamicsPtr.reset(new WheelLeggedDynamics(model));
    
    // 摩擦锥约束（foot 和 wheel 模式）
    problem.stateInputInequalityConstraintPtr.emplace(
        "frictionCone", std::make_unique<FrictionConeConstraint>(mu));
    
    // 滚动约束（仅 wheel 模式，通过模式检查激活）
    problem.stateInputEqualityConstraintPtr.emplace(
        "rollingConstraint", 
        std::make_unique<RollingConstraint>(model, wheelFrameIds));
    
    // 模式相关代价（权重随模式变化）
    problem.costPtr.emplace(
        "tracking", 
        std::make_unique<ModeAwareTrackingCost>(Q_map, R_map));
}
```

**Step 5：配置求解器并运行**

```cpp
// 中文注释：使用 SQP-RTI 求解器
SqpSettings sqpSettings;
sqpSettings.maxNumIterations = 1;     // RTI：每个控制周期只做一次迭代
sqpSettings.timeHorizon = 1.0;        // 预测 1 秒
sqpSettings.numPartitions = 20;       // 20 个离散步
sqpSettings.useFeedbackPolicy = true; // 输出反馈增益

auto mpcPtr = std::make_unique<MPC_MRT>(
    std::move(problem), sqpSettings, modeScheduleManager);

// 控制主循环
while (running) {
    auto state = getStateEstimate();
    mpcPtr->updatePolicy(state.time, state.x);
    auto [u_opt, feedback_gain] = mpcPtr->getOptimizedInput(state.time);
    applyCommand(u_opt);
}
```

这五步构成了从零搭建轮足 MPC 的完整 OCS2 集成流程。与纯足式的主要区别集中在 Step 1（状态/输入维度增加）、Step 2（轮速积分方程）、Step 3（三值模式调度）和 Step 4（滚动约束注册）。

### SQP-RTI 策略

SQP-RTI（Sequential Quadratic Programming - Real-Time Iteration）是 OCS2 和 ACADOS 使用的核心求解策略。其思想是：

**不等待 SQP 收敛，每个控制周期只做一次 SQP 迭代。**

| 步骤 | 操作 | 计算量 |
|------|------|--------|
| 1. 线性化 | 在当前轨迹猜测上线性化动力学和约束 | $O(N \cdot n_x)$ |
| 2. 求解 QP | 求解线性化后的 QP 子问题 | $O(N \cdot (n_x + n_u)^3)$ |
| 3. 应用 | 取 QP 解的第一步作为当前控制输入 | $O(1)$ |
| 4. 热启动 | 上一时刻的 QP 解平移一步作为下一次的初始猜测 | $O(N \cdot n_x)$ |

**为什么 SQP-RTI 能实时？**因为它利用了两个关键性质：

1. **时间连续性**：相邻控制周期的最优轨迹相似，热启动使得一次 SQP 迭代就能得到足够好的近似解
2. **反馈效应**：MPC 本身就是反馈控制——即使单步求解不完美，下一步会用新的状态重新优化，实现在线纠正

轮足 MPC 相比纯足式增加了约 30% 的状态和输入维度，SQP-RTI 的单步 QP 求解时间增加约 $(1.3)^3 \approx 2.2$ 倍——从 ~5 ms 增加到 ~11 ms，仍在实时范围内。

### 热启动策略

热启动是轮足 MPC 实时性的关键。但模式切换时热启动需要特殊处理：

| 场景 | 热启动策略 |
|------|-----------|
| 无模式切换 | 上一解平移一步：$x_k^{\text{warm}} = x_{k+1}^{\text{last}}$ |
| 预期内的切换 | 用新模式下的约束重新投影上一解 |
| 意外切换 | 丢弃上一解，从启发式初值重启 |

> **反事实推理**：如果不使用热启动会怎样？每个控制周期从零开始求解 NLP：收敛需要 5-20 次 SQP 迭代而非 1 次，计算时间从 ~10 ms 增加到 ~50-200 ms——远超实时要求。而且没有好的初值，SQP 可能收敛到局部极小值或根本不收敛。热启动是 MPC 实时运行的"氧气"。

### 安全降级策略的完整设计

MPC 在实时系统中必须有**完善的降级策略**——因为求解器可能超时、不收敛或给出不可行解。降级策略的核心原则是：**宁可不动也不能乱动**。

| 条件 | 降级动作 | 时间范围 |
|------|---------|---------|
| 单次超时 | 使用上一有效解，衰减系数 $\alpha = 0.95$ | $< 1$ 个控制周期 |
| 连续超时 3 次 | 切换所有末端为 foot 模式，降低速度命令 | $\sim 50$-$100$ ms |
| 连续超时 10 次 | 进入 stand 模式，锁定关节 | $\sim 200$ ms |
| KKT 残差 > 阈值 | 增加 SQP 迭代次数（从 1 次到 3 次）| 一次求解周期 |
| 约束不可行 | 软化约束（等式约束变惩罚项），标记警告 | 立刻 |

```python
class SafetyDegradation:
    """轮足 MPC 的安全降级策略。"""
    
    def __init__(self, max_timeout_count=10, decay_factor=0.95):
        self.timeout_count = 0
        self.decay = decay_factor
        self.last_valid_u = None
        self.safety_mode = 'normal'
    
    def process(self, mpc_result, solve_time, solve_budget):
        """
        处理 MPC 求解结果，返回安全的控制输入。
        """
        if solve_time <= solve_budget and mpc_result.feasible:
            # 正常：使用 MPC 解
            self.timeout_count = 0
            self.safety_mode = 'normal'
            self.last_valid_u = mpc_result.u[0]
            return mpc_result.u[0], self.safety_mode
        
        # 超时或不可行
        self.timeout_count += 1
        
        if self.timeout_count <= 3:
            # 轻度降级：衰减上一有效解
            self.safety_mode = 'decay'
            if self.last_valid_u is not None:
                u_safe = self.decay ** self.timeout_count * self.last_valid_u
                return u_safe, self.safety_mode
        
        elif self.timeout_count <= 10:
            # 中度降级：切换为 foot 模式 + 低速
            self.safety_mode = 'foot_fallback'
            return self._foot_fallback_command(), self.safety_mode
        
        else:
            # 重度降级：站立锁定
            self.safety_mode = 'stand_lock'
            return self._stand_lock_command(), self.safety_mode
    
    def _foot_fallback_command(self):
        """生成 foot 模式下的低速安全命令"""
        u_safe = np.zeros(self.last_valid_u.shape)
        # 只保留维持站立的法向接触力
        for i in range(4):
            u_safe[3*i + 2] = 50.0  # 每条腿 50N 法向力
        return u_safe
    
    def _stand_lock_command(self):
        """生成站立锁定命令"""
        u_safe = np.zeros(self.last_valid_u.shape)
        for i in range(4):
            u_safe[3*i + 2] = 50.0  # 法向力维持站立
        return u_safe
```

> **本质洞察**：安全降级不是"出了问题怎么办"的应急方案——它是控制系统设计的**核心组件**。一个没有降级策略的 MPC 控制器，就像一辆没有安全气囊的汽车——大部分时间工作正常，但在关键时刻可能酿成灾难。轮足 MPC 尤其需要降级策略，因为模式切换本身就增加了求解器失败的概率。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：滚动约束的导数用数值差分但步长与仿真步长相同**
> - 错误做法：差分步长 $h = \Delta t_{\text{sim}} = 0.001$ s
> - 典型现象：线性化不精确，SQP 收敛慢或不收敛
> - 根本原因：数值差分的最优步长约为 $h \sim \sqrt{\epsilon_{\text{machine}}} \approx 10^{-8}$，与仿真步长无关。$h = 0.001$ 太大，截断误差显著
> - 正确做法：使用 $h \sim 10^{-7}$ 的中心差分，或使用 CppAD/CasADi 的自动微分

> 💡 **概念误区：认为 SQP-RTI 的"一次迭代"意味着"解不精确"**
> - 新手想法："只做一次迭代，解一定比多次迭代差很多"
> - 实际上：由于热启动，每次 RTI 的起点已经非常接近最优解（只差一个控制步长的漂移）。一次 SQP 迭代足以校正这个漂移。理论分析表明 RTI 的轨迹跟踪精度与全收敛 SQP 几乎相同（误差为 $O(\Delta t^2)$）
> - 关键条件：热启动质量必须足够好。如果热启动质量差（如模式切换后），需要多做几次迭代或降低控制频率

### 练习 77.5

**[A] ⭐⭐⭐** 估算轮足 MPC 的单步 QP 求解时间。假设 $n_x = 29$, $n_u = 28$, $N = 20$, QP 求解器使用 HPIPM（$O(N(n_x+n_u)^3)$ 复杂度）。与纯足式 MPC（$n_x = 25, n_u = 24$）对比。

**[B] ⭐⭐⭐⭐** 实现一个简化的 SQP-RTI 求解器：对一个 2D 差速车跟踪问题（状态 $(x, y, \theta)$, 输入 $(v, \omega)$），实现线性化 + QP 求解 + 热启动的完整流程。验证 RTI 的跟踪精度。

**[C] ⭐⭐⭐** 讨论：如果 MPC 求解超时（单步求解时间 > 控制周期），应该怎么处理？设计一个超时降级策略：超时 1 次、连续超时 3 次、连续超时 10 次时分别怎么办？

---

## 77.5b WBC 中的轮速度任务与接触力分配 ⭐⭐⭐

### 这一节解决什么问题

MPC 运行在 ~30 Hz 的低频循环，输出的是**参考轨迹**——质心轨迹、接触力参考和轮速参考。但机器人的关节电机需要 ~500 Hz 的力矩命令。WBC（Whole-Body Controller）的职责是将 MPC 参考转化为满足**全维度动力学约束**的关节力矩。本节分析轮足 WBC 相比纯足式 WBC 的新增挑战。

### WBC 的 QP 结构

回顾足式/60\_QP\_NLP建模：WBC 本质上是一个二次规划（QP），其决策变量为 $(\ddot{v}, \lambda, \tau)$——广义加速度、接触力和关节力矩。

轮足 WBC 的 QP 形式：

$$
\min_{\ddot{v}, \lambda, \tau, \omega_w} \sum_{k} w_k \|J_k \ddot{v} + \dot{J}_k v - a_k^{\text{ref}}\|^2
$$

$$
\text{s.t.} \quad M(q)\ddot{v} + h(q,v) = S^T\tau + J_c^T\lambda \quad (\text{动力学硬约束})
$$

$$
\lambda \in \mathcal{K}_{\mu} \quad (\text{摩擦锥硬约束})
$$

$$
|\tau_i| \leq \tau_{\max,i} \quad (\text{力矩限幅硬约束})
$$

$$
A_{\text{wheel}}(q)v = b_{\text{wheel}} \quad (\text{滚动约束，wheel 模式})
$$

**轮足 WBC 的新增任务**：

| 优先级 | 任务 | 维度 | 纯足式 | 轮足 |
|--------|------|------|--------|------|
| P0 | 动力学方程 | $n_v$ | 有 | 有 |
| P0 | 摩擦锥 | $4 \times 3$ | 有 | 有（修正） |
| P0 | 力矩限幅 | $n_a$ | 有 | 有 |
| P1 | 基座姿态跟踪 | 6 | 有 | 有（权重更高） |
| P2 | **轮速匹配** | 4 | **无** | **有**（新增） |
| P2 | 接触力跟踪 | 12 | 有 | 有 |
| P3 | 摆动腿轨迹 | 因腿而异 | 有 | 有 |
| P4 | 关节正则化 | $n_a$ | 有 | 有 |

**轮速匹配任务**是轮足 WBC 的核心新增项。它通常作为**软任务**（而非硬约束），因为轮胎和地面之间的实际接触不是完美的纯滚动——存在滑移。将其设为硬约束会导致 QP 在高滑移时不可行。

轮速匹配任务的 QP 形式：

$$
\ell_{\omega} = w_\omega \sum_{i \in \text{wheel}} \|e_{\parallel,i}^T J_{c,i} \ddot{v} + e_{\parallel,i}^T \dot{J}_{c,i} v - r_i \dot{\omega}_{w,i}^{\text{ref}}\|^2
$$

**推导过程**：纯滚动条件要求接触点纵向速度等于轮的线速度 $r\omega_w$。对这个条件求时间导数，得到加速度级别的约束——这正是 WBC 需要的形式（因为 WBC 的决策变量是加速度 $\ddot{v}$）：

$$
e_{\parallel,i}^T \underbrace{(J_{c,i} \ddot{v} + \dot{J}_{c,i} v)}_{\text{接触点纵向加速度}} = \underbrace{r_i \dot{\omega}_{w,i}^{\text{ref}}}_{\text{轮纵向加速度参考}}
$$

回顾足式/90\_WBC分层优化与TSID 中的接触约束 $J_c \ddot{q} = -\dot{J}_c \dot{q}$：那里的约束要求接触点加速度为零（足端不动），这里的约束要求接触点纵向加速度等于轮的加速度（滚动而非静止）。两者的数学结构相同，只是右侧不同——这正是从足式 WBC 扩展到轮足 WBC 的核心修改点。

### 任务优先级的物理理由

为什么基座姿态跟踪（P1）应该高于轮速匹配（P2）？

考虑以下场景：机器人以 2 m/s 高速转弯，内侧轮的目标轮速为 15 rad/s，外侧轮为 25 rad/s。如果轮速匹配优先级高于基座姿态，WBC 会为了精确匹配轮速而牺牲基座姿态稳定——可能导致 roll 角过大、ZMP 接近边界。

反过来，如果基座姿态优先级更高，WBC 首先保证基座稳定（roll/pitch 在安全范围内），然后在剩余的控制自由度中尽力匹配轮速。即使轮速匹配不完美（产生一些滑移），基座仍然安全。

> **不是 X 而是 Y**：WBC 的轮速任务不是"精确控制轮速"，而是"在保证基座安全的前提下尽力匹配 MPC 规划的轮速参考"。这与零空间投影的本质完全一致——回顾复合/20, 72.5 节的本质洞察："零空间投影的意义不是完美执行次要任务，而是在不破坏主任务的前提下尽力而为。"

### WBC QP 的数值规模

为了让读者建立对轮足 WBC QP 规模的直觉，下面给出具体的矩阵维度：

| QP 组成部分 | 维度 | 说明 |
|-----------|------|------|
| 决策变量 $z = (\ddot{v}, \lambda, \tau)$ | $24 + 12 + 12 = 48$ | 广义加速度 + 接触力 + 关节力矩 |
| 代价矩阵 $H$ | $48 \times 48$ | 块对角（各任务的 Hessian 叠加） |
| 动力学等式约束 | $24$ 行 | $M\ddot{v} + h = S^T\tau + J_c^T\lambda$ |
| 滚动等式约束 | $\leq 12$ 行 | 每个 wheel 腿 3 行 |
| 摩擦锥不等式约束 | $\leq 32$ 行 | 每个接触腿 8 行（8 面体近似）|
| 力矩限幅不等式约束 | $24$ 行 | $-\tau_{\max} \leq \tau \leq \tau_{\max}$ |
| **总约束数** | **~92** 行 | |

这个规模的 QP（48 变量，92 约束）在现代 QP 求解器（qpOASES、ProxQP）上的求解时间约 0.05-0.2 ms——远小于 2 ms 的控制周期（500 Hz）。

### 接触力分配的各向异性

foot 模式下，接触力的分配是各向同性的——摩擦锥 $\|\lambda_t\| \leq \mu\lambda_n$ 在切向平面上是圆形。

wheel 模式下，纵向力和横向力的"可用范围"不同：

- **纵向力**：受轮电机力矩限制 $|F_{\text{long}}| \leq \tau_{w,\max}/r$，通常较小
- **横向力**：受横向摩擦限制 $|F_{\text{lat}}| \leq \mu_{\text{lat}} F_n$，可能较大

因此 wheel 模式的摩擦锥变成**椭圆形**而非圆形：

$$
\left(\frac{F_{\text{long}}}{\tau_{w,\max}/r}\right)^2 + \left(\frac{F_{\text{lat}}}{\mu_{\text{lat}} F_n}\right)^2 \leq 1
$$

在 QP 中，椭圆约束需要线性化为多面体近似（$k$ 条线性不等式），类似于标准摩擦锥的多面体近似。

### 摩擦锥多面体近似：从二次约束到线性约束 ⭐⭐⭐⭐

上面反复提到"椭圆约束需要线性化为多面体"，但具体怎么做？这一步是把 MPC/WBC 从**二阶锥规划（SOCP）**降级为**二次规划（QP）**的关键——绝大多数实时求解器（HPIPM、qpOASES）只接受线性约束，无法直接处理二次锥 $\|F_t\| \leq \mu F_n$。

**为什么不直接用 SOCP？** 二阶锥约束 $\|F_t\| \leq \mu F_n$ 在数学上更精确（它就是真实的圆锥），SOCP 求解器（如 ECOS、Mosek）也能处理。但 SOCP 的单次求解比 QP 慢一个数量级，且热启动支持差。在 1 kHz 控制循环里，这个代价无法承受。所以工程上的标准做法是：**牺牲一点精度，把圆锥用内接多面体近似，换取 QP 的求解速度和成熟的热启动**。

**标准摩擦锥的 $k$-面体近似**。把切向力平面用 $k$ 条均匀分布的切平面包围。对第 $j$ 条边（$j = 0, \ldots, k-1$），方向角 $\theta_j = 2\pi j / k$，约束为：

$$
\cos(\theta_j)\, F_x + \sin(\theta_j)\, F_y \leq \mu F_n, \quad j = 0, \ldots, k-1
$$

这是 $k$ 条线性不等式。它定义的是圆锥的**内接 $k$ 棱锥**——比真实圆锥保守（可行域略小）。$k$ 越大越接近真实圆锥，但约束行数越多、QP 越慢。

> **本质洞察**：多面体近似的"内接 vs 外接"选择体现了安全工程的保守原则。内接多面体的可行域**小于**真实摩擦锥——这意味着 MPC 永远不会请求一个真实摩擦锥不允许的力（绝不冒进），代价是偶尔放弃一些本来可行的力（略保守）。如果用外接多面体（可行域大于真实锥），MPC 可能请求超出真实摩擦能力的力，导致打滑。在安全关键系统里，宁可保守也不能冒进——这就是几乎所有实现都用内接多面体的原因。

近似误差可量化：内接 $k$ 棱锥与真实圆锥的最大相对误差为 $1 - \cos(\pi/k)$。常用取值：

| $k$ | 约束行数 | 最大相对误差 | 适用场景 |
|-----|---------|------------|---------|
| 4 | 4 | $1-\cos(45^\circ) \approx 29.3\%$ | 极快但粗糙，仅原型 |
| 6 | 6 | $1-\cos(30^\circ) \approx 13.4\%$ | 速度优先 |
| 8 | 8 | $1-\cos(22.5^\circ) \approx 7.6\%$ | 工程常用默认 |
| 16 | 16 | $1-\cos(11.25^\circ) \approx 1.9\%$ | 精度优先 |

**wheel 模式的各向异性椭圆锥近似**。回顾上面的椭圆约束 $(F_{\text{long}}/a)^2 + (F_{\text{lat}}/b)^2 \leq 1$，其中 $a = \tau_{w,\max}/r$（纵向半轴），$b = \mu_{\text{lat}} F_n$（横向半轴）。线性化思路与圆锥相同，但要先把椭圆**缩放为单位圆**再取切线：令 $\tilde{F}_x = F_{\text{long}}/a$, $\tilde{F}_y = F_{\text{lat}}/b$，则约束变为单位圆 $\tilde{F}_x^2 + \tilde{F}_y^2 \leq 1$。在缩放空间取 $k$ 棱内接多边形：

$$
\cos(\theta_j)\frac{F_{\text{long}}}{a} + \sin(\theta_j)\frac{F_{\text{lat}}}{b} \leq 1
\;\Longleftrightarrow\;
\frac{\cos\theta_j}{a} F_{\text{long}} + \frac{\sin\theta_j}{b} F_{\text{lat}} \leq 1
$$

注意 $a = \tau_{w,\max}/r$ 依赖轮速参考和电机限幅，$b = \mu_{\text{lat}} F_n$ 依赖法向力——两者都随状态变化，所以椭圆锥约束的系数矩阵每步都要重新计算（不像标准摩擦锥系数固定）。这是 wheel 模式约束装配比 foot 模式更耗时的原因。

```python
import numpy as np

def friction_cone_polytope(mu, Fn_max=None, k=8):
    """标准圆锥摩擦锥的 k 棱内接多面体近似。
    返回 A 使得 A @ [Fx, Fy, Fn] <= 0 近似 ||[Fx,Fy]|| <= mu*Fn。
    """
    rows = []
    for j in range(k):
        theta = 2 * np.pi * j / k
        # cos*Fx + sin*Fy - mu*Fn <= 0
        rows.append([np.cos(theta), np.sin(theta), -mu])
    return np.array(rows)

def ellipse_cone_polytope(tau_w_max, r, mu_lat, Fn, k=8):
    """wheel 模式各向异性椭圆锥的 k 棱内接多面体近似。
    返回 A 使得 A @ [F_long, F_lat] <= 1。
    """
    a = tau_w_max / r          # 纵向半轴：轮驱动能力
    b = mu_lat * Fn            # 横向半轴：横向摩擦能力
    rows = []
    for j in range(k):
        theta = 2 * np.pi * j / k
        rows.append([np.cos(theta) / a, np.sin(theta) / b])
    return np.array(rows)

# 中文注释：对比 foot 模式（圆）与 wheel 模式（椭圆）的可行力域
A_foot = friction_cone_polytope(mu=0.7, k=8)
A_wheel = ellipse_cone_polytope(tau_w_max=8.0, r=0.05, mu_lat=0.7, Fn=50.0, k=8)
print("foot 圆锥约束行数:", A_foot.shape[0])
print("wheel 椭圆锥纵向半轴 a =", 8.0/0.05, "N（轮驱动限）")
print("wheel 椭圆锥横向半轴 b =", 0.7*50.0, "N（摩擦限）")
# 纵向 160N 远大于横向 35N -> wheel 模式纵向能力强、横向能力弱
```

运行可见：本例中 wheel 模式纵向力上限 $a = 160$ N（轮驱动充足），横向力上限 $b = 35$ N（横向摩擦受限）。这与直觉一致——轮在纵向（滚动方向）能提供大驱动力，但横向（侧向）抗滑能力弱。这正是 77.6 节"高速转弯横向摩擦最先耗尽"的力学根源。

**把 foot 圆锥约束矩阵写具体**。取 $\mu = 0.7$, $k = 8$，前几行系数 $[\cos\theta_j,\ \sin\theta_j,\ -\mu]$ 为：

$$
\mathbf{F}_{\text{fric}} =
\begin{pmatrix}
1.000 & 0.000 & -0.7 \\
0.707 & 0.707 & -0.7 \\
0.000 & 1.000 & -0.7 \\
-0.707 & 0.707 & -0.7 \\
\vdots & \vdots & \vdots
\end{pmatrix},
\quad
\mathbf{F}_{\text{fric}}
\begin{pmatrix} F_x \\ F_y \\ F_n \end{pmatrix} \leq \mathbf{0}
$$

每一行都是"切向力在某个方向上的投影 $\leq \mu F_n$"。第 1 行 $F_x \leq 0.7 F_n$，第 3 行 $F_y \leq 0.7 F_n$，第 2 行约束的是 $45^\circ$ 方向。8 行合起来把切向力 $(F_x, F_y)$ 围在一个正八边形内——这个正八边形内接于半径 $\mu F_n$ 的圆。

**对比 wheel 椭圆锥**。同样 $k = 8$，但系数变为 $[\cos\theta_j/a,\ \sin\theta_j/b]$，右端是 $1$：

$$
\mathbf{F}_{\text{ellip}} =
\begin{pmatrix}
1/160 & 0 \\
0.707/160 & 0.707/35 \\
0 & 1/35 \\
\vdots & \vdots
\end{pmatrix}
=
\begin{pmatrix}
0.00625 & 0 \\
0.00442 & 0.0202 \\
0 & 0.0286 \\
\vdots & \vdots
\end{pmatrix}
$$

第 3 行 $F_{\text{lat}}/35 \leq 1$ 即 $F_{\text{lat}} \leq 35$ N，第 1 行 $F_{\text{long}}/160 \leq 1$ 即 $F_{\text{long}} \leq 160$ N。可以看到横向系数（$0.0286$）远大于纵向系数（$0.00625$）——系数越大，对应方向的力被约束得越"紧"。这就是各向异性在矩阵层面的直接体现：椭圆锥用不对称的系数把横向力卡得比纵向力紧得多。

> **反事实推理**：如果对 wheel 模式也用各向同性的圆锥（取 $a = b = \mu F_n$）会怎样？圆锥会**高估**横向能力（把横向上限从 35 N 错误放大到 $\mu F_n$ 级别），同时**低估**纵向能力（把纵向上限从 160 N 压到 $\mu F_n$）。结果是 MPC 在转弯时以为横向摩擦充足而请求过大横向力 → 实际侧滑；同时不敢充分利用纵向驱动 → 加速无力。各向异性建模不是锦上添花，而是 wheel 模式力分配正确性的必要条件。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：多面体近似用外接而非内接**
> - 错误做法：把约束写成 $\cos\theta_j F_x + \sin\theta_j F_y \leq \mu F_n / \cos(\pi/k)$（外接多边形，可行域偏大）
> - 典型现象：MPC/WBC 求解成功，但真机偶发打滑——尤其在力接近摩擦锥边界时
> - 根本原因：外接多边形的可行域大于真实圆锥，MPC 会请求真实摩擦不允许的力
> - 正确做法：用内接多边形（右端就是 $\mu F_n$，不除 $\cos$），可行域略小于真实锥，保证绝不冒进

> 💡 **概念误区：认为面数 $k$ 越大总是越好**
> - 新手想法："$k = 32$ 比 $k = 8$ 精确，应该用大 $k$"
> - 实际上：约束行数随 $k$ 线性增长，QP 求解时间随之上升。$k = 8$ 的 7.6% 误差对大多数应用已足够（摩擦系数本身的估计误差通常就有 10%-20%）。盲目增大 $k$ 是用确定性的计算开销去换取一个被摩擦系数不确定性淹没的精度提升
> - 正确思维：$k$ 的选择应匹配摩擦系数的估计精度——当 $\mu$ 本身只能估到 $\pm 15\%$，用 $k=16$（1.9% 误差）是浪费

### 练习 77.5c

**[A] ⭐⭐⭐** 推导内接 $k$ 棱锥相对真实圆锥的最大相对误差公式 $1-\cos(\pi/k)$。提示：考虑两条相邻切平面之间、离原点最近的边界点到圆锥面的距离。

**[B] ⭐⭐⭐** 用 matplotlib 画出 foot 模式圆锥和 wheel 模式椭圆锥在 $F_n=50$ N 切片上的可行力域（$F_{\text{long}}$-$F_{\text{lat}}$ 平面），叠加各自的 8 棱内接多边形。直观对比两者的形状差异。

**[C] ⭐⭐⭐⭐** wheel 模式椭圆锥的半轴 $a,b$ 随状态变化（$a$ 依赖轮速参考，$b$ 依赖 $F_n$）。在 SQP-RTI 中，这个变化的约束系数应该在哪一步重新计算？如果用上一步的系数（不更新）做热启动，会引入什么误差？

---

## 77.6 轮足 ZMP/DCM 扩展 ⭐⭐⭐

### 这一节解决什么问题

ZMP（Zero Moment Point）和 DCM（Divergent Component of Motion）是足式机器人稳定性分析的经典工具。轮式接触改变了支撑多边形的形状和接触力的分配方式。本节分析轮模式对 ZMP 和 DCM 的影响。

### 轮接触对支撑多边形的影响

回顾复合/20, 72.3 节：ZMP 必须位于支撑多边形内。

**足模式**的支撑多边形由四个足端位置的凸包确定——是一个固定的四边形（站立时）或三角形（摆动相时）。

**轮模式**的支撑多边形不同：轮只能在纵向提供有限的切向力（受轮驱动力矩限制），在横向提供的切向力也受限（受横向摩擦限制）。这意味着轮模式下的**有效支撑区域**小于足模式。

更精确地说，ZMP 约束应该考虑力可行域而非仅仅几何支撑多边形：

$$
\text{ZMP} \in \text{ConvexHull}\left(\{p_{c,i}\}_{i \in \mathcal{C}}\right)
$$

但每个接触点提供的力受到摩擦锥约束：

| 接触类型 | 纵向力范围 | 横向力范围 | 法向力范围 |
|---------|-----------|-----------|-----------|
| foot | $[-\mu F_n, \mu F_n]$ | $[-\mu F_n, \mu F_n]$ | $[F_{n,\min}, F_{n,\max}]$ |
| wheel | $[-\tau_w/r, \tau_w/r]$ | $[-\mu_{\text{lat}} F_n, \mu_{\text{lat}} F_n]$ | $[F_{n,\min}, F_{n,\max}]$ |

wheel 模式的纵向力由轮电机决定（$F_{\text{long}} = \tau_w / r$），通常小于 $\mu F_n$。这缩小了纵向力的可用范围。

### DCM 在轮足系统中的意义

DCM（Divergent Component of Motion）定义为：

$$
\xi = \mathbf{p}_G + \frac{\dot{\mathbf{p}}_G}{\omega_0}
$$

其中 $\omega_0 = \sqrt{g/z_G}$ 是 LIPM 的自然频率。DCM 的动力学为：

$$
\dot{\xi} = \omega_0 (\xi - \mathbf{p}_{\text{ZMP}})
$$

DCM 必须收敛到 ZMP——否则系统发散（倾覆）。

轮足系统中，DCM 的收敛条件不变，但 ZMP 的可行域（取决于轮的力可行域）更小。这意味着**轮足机器人的稳定裕度比纯足式更小**，需要更精确的 MPC 和更快的反馈。

> **类比**：足模式的稳定性像站在宽板上——支撑面积大，ZMP 裕度充足。轮模式的稳定性像站在滑板上——虽然也能站，但横向稳定裕度小，需要持续的微调。这就是为什么轮足机器人在高速转弯时需要特别注意横向摩擦裕度——ZMP 可能还在支撑多边形内，但横向摩擦力已经不够了。

### 多裕度安全指标

轮足 MPC 不应只检查 ZMP，而应同时监控**三类裕度**：

$$
m_{\text{ZMP}} = d(\text{ZMP}, \partial \text{SupportPolygon}) \quad (\text{几何裕度})
$$

$$
m_{\text{friction}} = \mu F_n - \sqrt{F_x^2 + F_y^2} \quad (\text{摩擦裕度})
$$

$$
m_{\omega} = \omega_{\max} - |\omega_w| \quad (\text{轮速裕度})
$$

三类裕度中的最小值决定系统的安全状态。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：只检查 ZMP 不检查摩擦裕度**
> - 错误做法：MPC 约束只包含 ZMP $\in$ 支撑多边形，不包含摩擦锥约束
> - 典型现象：高速转弯时 ZMP 还在支撑多边形内（几何安全），但横向摩擦力已超出摩擦锥（力不安全），轮子侧滑
> - 根本原因：ZMP 是几何/力矩层面的指标，摩擦锥是力层面的指标。两者独立，必须分别检查
> - 正确做法：MPC 约束中同时包含 ZMP 约束和摩擦锥约束

> 💡 **概念误区：认为轮足的 DCM 分析与纯足式完全相同**
> - 新手想法："DCM 公式不变，所以分析方法也不变"
> - 实际上：DCM 动力学公式确实不变，但 ZMP 的**可行域**变了。轮模式下 ZMP 可行域更小（因为轮的纵向力受限），所以 DCM 必须更精确地跟踪目标 ZMP——对 MPC 精度的要求更高

### 练习 77.6

**[A] ⭐⭐** 画出四轮 wheel 模式下的支撑多边形。与四 foot 模式对比，在哪个方向上裕度更小？

**[B] ⭐⭐⭐** 计算一个具体场景的三类裕度：Go2+wheels 以 1 m/s 前进、0.5 rad/s 转弯。假设 $\mu = 0.7$, $F_n = 50$ N/wheel, $\omega_{\max} = 30$ rad/s, $r = 0.05$ m。哪个裕度最先耗尽？

**[C] ⭐⭐⭐** 推导轮足系统中 DCM 收敛的充分条件。与纯足式对比，轮足系统的 DCM 对 MPC 精度的要求高多少？

---

## 77.7 案例精读：Swiss-Mile / LimX Dynamics 的 MPC 架构 ⭐⭐⭐

### 这一节解决什么问题

理论推导必须连接工程实践。本节精读两个代表性的轮足 MPC 架构。

### Swiss-Mile (RIVR) 的 MPC + RL 混合架构

Swiss-Mile（现更名为 RIVR）在 2024 年发表的 Science Robotics 论文中提出了一种 **MPC 参考 + RL 跟踪** 的混合架构：

```text
              ┌───────────────────────┐
  速度命令 ──>│  MPC (Centroidal)     │──> 质心轨迹参考
  地形信息 ──>│  ~30 Hz               │    接触力参考
              └───────────┬───────────┘
                          │
              ┌───────────▼───────────┐
  本体感觉 ──>│  RL Policy (PPO)      │──> 关节位置目标
  历史动作 ──>│  ~50 Hz               │    轮速目标
              └───────────┬───────────┘
                          │
              ┌───────────▼───────────┐
              │  PD 关节控制器         │──> 电机力矩
              │  ~1000 Hz             │
              └───────────────────────┘
```

**核心思想**：MPC 负责"做什么"（质心轨迹和接触力），RL 负责"怎么做"（全身运动和力矩）。MPC 提供物理保证（摩擦锥、ZMP），RL 提供鲁棒性和适应性。

| 层级 | 职责 | 频率 | 优势 |
|------|------|------|------|
| MPC | 质心规划 + 接触力优化 | 30 Hz | 物理约束保证、可解释 |
| RL | 全身运动跟踪 | 50 Hz | 鲁棒、自适应、处理未建模动力学 |
| PD | 关节级力矩控制 | 1000 Hz | 快速执行、电机保护 |

### Bjelonic 2021 的全身 MPC

Bjelonic et al. (IROS 2021) 的"Whole-Body MPC and Online Gait Sequence Generation"使用了更传统的 **MPC + WBC** 架构：

| 特性 | Bjelonic 2021 | Swiss-Mile 2024 |
|------|--------------|----------------|
| 低层控制 | WBC（QP优化） | RL（神经网络） |
| 模式调度 | 运动学腿效用评估 | RL 策略输出 |
| 鲁棒性来源 | MPC 重规划 | RL 域随机化 |
| Sim2Real | 精确模型 | 域随机化 + 自适应 |
| 计算平台 | CPU（OCS2） | CPU(MPC) + GPU(RL) |

Bjelonic 2021 的关键创新是**在线步态序列生成**——不预设步态时序（如 trot、walk），而是让优化器通过**运动学腿效用**自动发现最优的接触序列。运动学腿效用衡量每条腿在当前配置下的"有用程度"：

$$
u_{\text{leg},i} = \frac{d_i - d_{\min}}{d_{\max} - d_{\min}}
$$

其中 $d_i$ 是第 $i$ 条腿的工作空间余量。当 $u_{\text{leg},i}$ 低于阈值时，该腿进入 swing 模式。

> **本质洞察**：轮足 MPC 的架构演进方向是"MPC 做高层规划，X 做低层执行"，其中 X 从 WBC 演变为 RL。WBC 的优势是物理精确和可解释，RL 的优势是鲁棒和自适应。混合架构试图结合两者的优点。但混合架构的难点在于**两层之间的接口设计**——MPC 输出什么信息给 RL？RL 是否可以违反 MPC 的参考？这些问题目前没有统一答案。

### ⚠️ 常见陷阱与误区

> 💡 **概念误区：认为 RL 可以完全替代 MPC**
> - 新手想法："RL 端到端训练，不需要 MPC"
> - 实际上：纯 RL 在轮足机器人上面临两个特有挑战：（1）轮的连续转速使动作空间更大，增加了探索难度；（2）轮足模式切换使得 reward shaping 更复杂——切换不当的惩罚难以设计。Swiss-Mile 2024 的论文明确指出 MPC 提供的质心参考显著提高了 RL 训练效率和最终性能
> - 延伸：这正是复合/80\_Wheel\_Legged\_Gym\_RL 和复合/210 要深入讨论的问题

> 🧠 **思维陷阱：把运输能耗降低 85% 归因于 MPC 本身**
> - 新手想法："Bjelonic 2021 说运输能耗降低 85%，说明 MPC 比 PD 控制好 85%"
> - 实际上：85% 的能耗降低主要来自**模式选择**——在平坦地面用轮滚动（能效远高于足行走），在崎岖地面用足行走。MPC 的贡献是**自动发现**这种最优模式切换策略，而非 MPC 本身的力矩优化效果
> - 正确归因：能耗降低 = 轮的能效优势 + MPC 的最优切换 + 在线轨迹优化

### 练习 77.7

**[A] ⭐⭐** 画出 Swiss-Mile 混合架构的数据流图。标出每一层的输入、输出和频率。

**[B] ⭐⭐⭐** 比较 Bjelonic 2021（MPC + WBC）和 Swiss-Mile 2024（MPC + RL）在以下场景中的预期表现：（1）平地高速直行，（2）泥泞坡面，（3）台阶过渡。讨论各自的优势和劣势。

**[C] ⭐⭐⭐⭐** 如果你要设计一个轮足 MPC 的 demo，只有 4 周时间，你会选择哪种架构（纯 MPC、MPC+WBC、MPC+RL）？给出时间分配计划和风险分析。

---

## 77.8 与 RL 混合：MPC 参考 + RL 跟踪架构 ⭐⭐⭐

### 这一节解决什么问题

本节深入分析 MPC + RL 混合架构的数学接口，为后续章节（复合/80\_Wheel\_Legged\_Gym\_RL 和复合/210）建立桥梁。

### MPC 向 RL 传递什么信息？

| 传递信息 | 维度 | 用途 |
|---------|------|------|
| 质心位置参考 $\mathbf{p}_G^{\text{ref}}$ | 3 | RL 的 reward 基准 |
| 质心速度参考 $\dot{\mathbf{p}}_G^{\text{ref}}$ | 3 | RL 的速度跟踪目标 |
| 接触力参考 $\boldsymbol{\lambda}_i^{\text{ref}}$ | $3 \times 4 = 12$ | RL 的力先验 |
| 模式序列 $\sigma(t)$ | $4$ | RL 的接触规划 |
| 轮速参考 $\omega_w^{\text{ref}}$ | 4 | RL 的轮速目标 |

总共约 26 维的 MPC 参考信号进入 RL 的观测空间。

### RL 可以违反 MPC 参考吗？

这是混合架构的核心设计决策：

| 策略 | 描述 | 优点 | 缺点 |
|------|------|------|------|
| 硬跟踪 | RL 必须精确跟踪 MPC 参考 | MPC 的约束保证被保留 | RL 失去自适应能力 |
| 软跟踪 | RL 以 MPC 参考为 reward，但可以偏离 | RL 可以处理未建模效应 | MPC 的安全保证可能被违反 |
| 安全约束跟踪 | RL 跟踪 MPC，但安全约束（摩擦锥、ZMP）由低层 PD 保证 | 兼顾灵活性和安全性 | 架构复杂，需要安全过滤器 |

Swiss-Mile 2024 采用的是**软跟踪**——MPC 参考进入 RL 的 observation，跟踪偏差作为 reward 的一部分（但不是唯一 reward）。这允许 RL 在 MPC 预测不准时自适应调整。

> **反事实推理**：如果 RL 完全忽略 MPC 参考会怎样？RL 退化为纯端到端策略——需要自己发现质心规划、模式切换和力分配的最优策略。这大大增加了探索空间和训练时间。Swiss-Mile 的消融实验表明，去掉 MPC 参考后，训练所需步数增加约 5 倍，最终性能下降约 30%。

### 架构与后续章节的桥接

| 本章概念 | 复合/80 (RL) 的使用 | 复合/210 (混合) 的使用 |
|---------|-------------------|---------------------|
| 模式序列 $\sigma(t)$ | RL observation 的一部分 | MPC-RL 接口的核心 |
| 滚动约束残差 | RL reward shaping | 安全过滤器的监控指标 |
| ZMP/DCM 裕度 | RL reward 的稳定性项 | 安全降级触发条件 |
| OCS2 状态定义 | RL observation 空间设计 | MPC-RL 状态映射 |

### ⚠️ 常见陷阱与误区

> 🧠 **思维陷阱：认为 MPC + RL 总比纯 MPC 或纯 RL 好**
> - 新手想法："两个方法结合一定比单个好"
> - 实际上：混合架构引入了新的失败模式——MPC 和 RL 不一致时，系统行为不可预测。例如：MPC 规划了足模式站立，但 RL 输出了轮模式的动作。如果没有一致性检查，这会导致力矩冲突和不稳定
> - 正确思维：混合架构需要显式的**一致性保证**——要么通过接口设计强制一致，要么通过安全过滤器在不一致时降级

### 练习 77.8

**[A] ⭐⭐⭐** 设计一个 MPC + RL 混合架构的 reward 函数。要求包含：质心跟踪、轮速匹配、力矩平滑、模式切换惩罚、存活奖励。给出每项的权重和物理意义。

**[B] ⭐⭐⭐** 讨论：MPC 参考信号的频率（30 Hz）和 RL 策略的频率（50 Hz）不同。RL 如何处理 MPC 参考信号的"过期"问题？设计一个插值策略。

**[C] ⭐⭐⭐⭐** 如果 RL 策略在部署时遇到训练中未见过的地形（如深雪），MPC 参考是否仍然有价值？分析 MPC 在 out-of-distribution 场景中的作用。

---

## 77.9 近年研究脉络：从 Keep Rollin' 到 kino-dynamic WB-MPC ⭐⭐⭐

### 这一节解决什么问题

77.7 精读了 Bjelonic 和 Swiss-Mile 两个代表架构。但轮足 MPC 是一个快速演进的领域，2018-2026 年间有一条清晰的技术演进主线。本节梳理这条主线，帮助读者建立"森林视角"——理解每篇工作在技术谱系中的位置，以及未解决的开放问题。这一节是 R6E（系统性分类）的集中体现：不是罗列论文，而是按"建模抽象层级"和"模式决策方式"两个维度对方法做穷举分类。

### 维度一：按建模抽象层级分类

轮足 MPC 的核心设计选择之一是**用多简化的模型做预测**。模型越简单，求解越快但精度越低。沿这个维度，已有工作可分为四档：

| 抽象层级 | 模型维度 | 代表工作 | 求解频率 | 适用场景 |
|---------|---------|---------|---------|---------|
| **全身刚体（whole-body）** | $n_v \sim 24$ | Bjelonic 2021 (IROS) | 30-100 Hz | 崎岖地形、需要精确足端力 |
| **kino-dynamic（运动学+质心）** | 质心 6 + 全身运动学 | WB-MPC kino-dynamic 2025 | 100+ Hz | 兼顾精度与速度，含 warm-start |
| **centroidal（质心）** | 6 + 接触点 | Swiss-Mile 2024、本章主线 | 30-50 Hz | 平地巡航、MPC+RL 混合 |
| **SRBD（单刚体）** | 12-16 | Di Carlo 2018（足式基础） | 20-50 Hz | 最快，腿惯量可忽略时 |

> **本质洞察**：这四档不是"越精确越好"的简单排序，而是**精度-实时性的 Pareto 前沿**上的不同工作点。kino-dynamic 模型（2025 年的新方向）的意义在于：它在质心动力学的基础上保留了**全身运动学**约束，从而能在 MPC 层就考虑关节限位和自碰撞，避免了 centroidal MPC 把不可达的足端轨迹交给 WBC 才发现不可行的问题。这是对"MPC 用质心、WBC 用全身"传统分工的一次重要修正。

### 维度二：按模式决策方式分类

77.4 节讨论了三种模式调度策略。把已有工作沿这个维度铺开，可以看到清晰的演进：

| 决策方式 | 模式序列来源 | 代表工作 | 优势 | 局限 |
|---------|------------|---------|------|------|
| **固定步态** | 人工预设 trot/walk | Keep Rollin' 2018 | 简单、可预测 | 无法地形自适应 |
| **运动学效用** | 在线评估腿工作空间余量 | Bjelonic 2021 | 自动发现接触序列 | 启发式、非全局最优 |
| **采样优化** | 模式空间随机采样+评估 | Jelavic 2021 (RSS) | 近似全局搜索 | 计算量大、需异步线程 |
| **学习策略** | RL/神经网络输出偏好 | Swiss-Mile 2024 | 端到端地形自适应 | 需大量训练、可解释性差 |

**演进主线**：从"人工设计模式序列"（2018）→ "在线启发式发现"（2021）→ "学习驱动"（2024-2026）。趋势是把模式决策的权力逐步从工程师手中交给优化器和学习算法，但**安全门控**（77.4 的滞回+最短驻留）始终是不可省略的兜底层。

### 几个关键工作的定位

**Keep Rollin' (Bjelonic et al., RAL 2018)**：轮足全身运动控制的早期奠基工作之一。它建立了"轮足四足机器人"的运动控制与规划框架，首次系统地处理了轮的非完整约束如何进入 WBC。本章的 WBC 轮速任务（77.5b）可以追溯到这一工作的思想。

**Rolling in the Deep (Bjelonic et al., RAL 2020)**：把轮足运动建模为**在线轨迹优化**问题，处理台阶、斜坡、间隙等地形。它是 Bjelonic 2021 全身 MPC 的前身——2020 年解决"能不能规划出可行轨迹"，2021 年解决"能不能实时全身 MPC + 在线步态生成"。

**kino-dynamic WB-MPC (2025)**：针对轮式双足（wheeled biped）提出新的 kino-dynamic 模型（线性倒立摆+飞轮 + 全身运动学）和 warm-start 方法，实现实时全身 MPC。它代表了 2025 年的一个重要方向——在双足轮式平台上把建模抽象层级从 SRBD/centroidal 提升到 kino-dynamic，同时保持实时性。warm-start 方法呼应了本章 77.5 的热启动讨论。

**disturbance-adaptive MPC (2025)**：针对轮式双足机器人，提出基于扰动估计的自适应 MPC 框架，在线估计外部扰动并补偿。这是对本章 77.5 安全降级思想的补充——不只是被动降级，而是主动估计扰动并调整 MPC 模型。

> **类比（标边界）**：轮足 MPC 这十年的演进与深度学习里"特征工程→端到端学习"的演进**路径相似**——都是把人工设计的环节逐步交给数据/优化驱动。相似之处：两者都经历了"专家知识主导→混合→学习主导"三阶段。**不同之处**：机器人控制有硬安全约束（摔倒不可逆、可能损坏硬件），所以即使在"学习主导"阶段，物理约束层（摩擦锥、ZMP、力矩限幅）和安全门控**永远不能被学习模块替代**。**不要把类比延伸到**：不要认为轮足控制会像图像分类那样最终完全端到端——可解释的物理模型在安全关键系统中有不可替代的地位，这是 Swiss-Mile 2024 仍然保留 MPC 层的根本原因。

### 开放问题与研究空白

梳理这条主线后，可以识别出几个尚未解决的开放问题（这些是潜在的研究方向）：

| 开放问题 | 现状 | 难点 |
|---------|------|------|
| 模式序列的**全局最优** | 启发式/采样近似 | $81^N$ 组合爆炸，无多项式算法 |
| 切换时机的**可微优化** | event-time 预设 | guard 触发时刻对状态不可微 |
| 真机**Sim2Real** 的模式切换 | 域随机化 | 切换瞬间的接触模型最难建模 |
| MPC 与 RL 的**理论保证** | 经验性混合 | 缺乏混合系统的稳定性证明 |

> **给研究者的提示**：上表第二行"切换时机的可微优化"是当前最活跃的方向之一。guard 触发时刻 $t_{\text{switch}}$ 由 $h_{\text{foot}}(\mathbf{x}(t))=0$ 隐式定义，对它求导需要隐函数定理，且在 guard 切平面处梯度可能病态。把切换时机变成可微的决策变量，是让 MPC 真正"自主优化何时切换"的关键技术瓶颈——目前多数系统仍把切换时机作为外部输入而非优化变量。

### 练习 77.9

**[A] ⭐⭐** 把本章 77.1 历史脉络表中的工作，按本节的"建模抽象层级"和"模式决策方式"两个维度重新分类，画一张二维散点图（横轴=抽象层级，纵轴=模式决策方式）。哪个象限的工作最多？哪个象限是空白？

**[B] ⭐⭐⭐** 研读 kino-dynamic WB-MPC（2025）的摘要。它的 kino-dynamic 模型相比本章的 centroidal 模型多保留了什么信息？这个信息在 MPC 层就考虑，相比留给 WBC 处理有什么好处？

**[C] ⭐⭐⭐⭐** 针对"切换时机的可微优化"这个开放问题，设计一个简化实验：对一个 1 维落地模型（质点从高度 $h_0$ 自由下落，落地时刻 $t_{\text{switch}}$ 满足 $h(t)=0$），推导 $t_{\text{switch}}$ 对初始高度 $h_0$ 的导数 $\partial t_{\text{switch}} / \partial h_0$。这个导数在什么情况下会发散？

---

## 本章小结

| 模块 | 核心问题 | 应掌握的能力 |
|------|---------|------------|
| 77.1 动机 | 为什么轮足更需要 MPC？ | 理解模式切换的三重数学困难 |
| 77.2 动力学建模 | 轮足的动力学方程？ | 写出 centroidal + 轮速的状态方程 |
| 77.2b 冲击动力学 | 切换瞬间发生什么？ | 用 guard/reset map 刻画速度跳变与能量损失 |
| 77.3 OCP 形式化 | 完整的 MPC 问题？ | 定义状态、输入、代价和约束 |
| 77.4 模式调度 | 谁决定模式序列？ | 设计带滞回的模式切换器 |
| 77.5 数值求解 | 如何实时求解？ | 理解 SQP-RTI 和 OCS2 接口 |
| 77.6 ZMP/DCM | 轮如何影响稳定性？ | 计算三类裕度指标 |
| 77.7 案例精读 | 工程如何落地？ | 分析 Swiss-Mile 和 Bjelonic 的架构 |
| 77.8 RL 混合 | MPC 和 RL 如何协作？ | 设计 MPC-RL 接口 |
| 77.9 研究脉络 | 领域如何演进？ | 按抽象层级/模式决策两维度定位已有工作 |

---

## 本章常见误解汇总

下表汇总全章散落的概念误区，集中对照，便于在复习时快速校准心智模型。

| # | 常见误解 | 正确理解 | 出处 |
|---|---------|---------|------|
| 1 | 轮足 MPC = 足式 MPC + 轮运动学 | 困难在最优控制层（模式切换使 OCP 成为切换系统/混合整数问题），不在运动学层 | 77.1 |
| 2 | MPC 应搜索全局最优模式序列 | $81^N$ 不可枚举；模式序列由启发式/学习给定，MPC 在固定序列下优化连续变量 | 77.1 |
| 3 | centroidal dynamics 不适用于轮足 | 其有效性取决于腿部惯量是否可忽略，与末端是足还是轮无关；轮改变的是接触约束 | 77.2 |
| 4 | 所有模式切换都要计算冲量 | 只有"切换激活新速度约束且当前速度不满足"时才有冲量；释放约束或提前匹配轮速无冲量 | 77.2b |
| 5 | reset map 会改变机器人位置 | 只改变速度（$\mathbf{q}^+=\mathbf{q}^-$），位置在碰撞瞬间连续 | 77.2b |
| 6 | 轮速参考应由速度命令前馈计算 | 应作为 MPC 决策变量优化（前馈忽略差速）；前馈值仅作热启动初值 | 77.3 |
| 7 | 所有模式用相同代价权重 | 权重是模式相关的分段二次型；wheel 强调速度匹配，foot 强调位置保持 | 77.3 |
| 8 | 追求模式切换"无缝"（力连续） | 力可行域本身离散，力连续不可能；目标是"力矩变化率有界" | 77.4 |
| 9 | SQP-RTI 一次迭代意味着解不精确 | 热启动使起点接近最优，一次迭代足以校正漂移，精度 $O(\Delta t^2)$ | 77.5 |
| 10 | 只检查 ZMP 即可保证稳定 | ZMP（几何/力矩）与摩擦裕度（力）独立，高速转弯可能 ZMP 安全但摩擦超限侧滑 | 77.6 |
| 11 | RL 可完全替代 MPC | 轮的连续转速增大动作空间、模式切换使 reward shaping 复杂；MPC 参考显著提升 RL 训练效率 | 77.7 |
| 12 | MPC+RL 总比单一方法好 | 混合引入新失败模式（两者不一致时行为不可预测），需显式一致性保证 | 77.8 |
| 13 | 能耗降低 85% 归因于 MPC 力矩优化 | 主要来自模式选择（轮的能效优势）；MPC 贡献是自动发现最优切换 | 77.7 |
| 14 | 摩擦锥多面体面数越多越好 | 面数应匹配 $\mu$ 估计精度；$k=8$ 的 7.6% 误差通常已被摩擦不确定性淹没 | 77.5c |
| 15 | wheel 模式可用各向同性圆锥 | 圆锥同时高估横向、低估纵向能力，导致转弯侧滑+加速无力；必须用各向异性椭圆锥 | 77.5c |
| 16 | 建模越精确（全身）越好 | 精度-实时性是 Pareto 前沿；kino-dynamic/centroidal 是不同工作点，按场景选择 | 77.9 |

> **如何使用本表**：这张表是全章概念误区的"体检清单"。建议在第二遍阅读或考前复习时，先遮住"正确理解"列，仅看"常见误解"列自测——如果你无法立即说出为什么某条是误解、正确理解是什么，就回到对应小节重读。能独立讲清全部 16 条，说明你已真正掌握轮足 MPC 的概念框架，而不只是记住了公式。

---

## 累积项目：本章新增模块——Mini Wheel-Legged MPC

| 阶段 | 任务 | 交付物 |
|------|------|--------|
| 1. 模式管理 | 实现三值模式状态机（含滞回和最短驻留时间） | 模式时间线绘图脚本 |
| 2. 约束接入 | 将复合/60 的 rolling_residual 接入 OCS2 风格的 StateInputConstraint | 残差与 slack 日志 |
| 3. WBC 合成 | 实现轮速度任务、基座稳定任务和接触力分配的优先级栈 | 命令曲线绘图脚本 |
| 4. 安全降级 | 实现求解超时降级策略（衰减上一解 → 低速 foot → 站立锁定） | 降级报告 |
| 5. 仿真验证 | 在简化仿真中测试平地直行 + 转弯 + 模式切换序列 | 完整仿真日志 |

---

## 延伸阅读

- **核心** ⭐⭐⭐ Bjelonic et al., "Whole-Body MPC and Online Gait Sequence Generation for Wheeled-Legged Robots" (IROS 2021): 轮足全身 MPC 的代表作，包含完整的问题形式化和实验验证
- **核心** ⭐⭐⭐ Bjelonic et al., "Rolling in the Deep: Hybrid Locomotion for Wheeled-Legged Robots Using Online Trajectory Optimization" (RAL 2020): 在线轨迹优化方法，展示了崎岖地形上的轮足运动
- **进阶** ⭐⭐⭐ Lee et al., "Learning Robust Autonomous Navigation and Locomotion for Wheeled-Legged Robots" (Science Robotics 2024): MPC + RL 混合架构，Swiss-Mile 的学术论文
- **进阶** ⭐⭐⭐ Grandia et al., "Perceptive Locomotion through NMPC" (RAL 2023): 感知约束进入 NMPC，展示了地形自适应
- **工具** ⭐⭐ OCS2 文档, Switched System Examples: OCS2 框架中切换系统的代码接口参考
- **基础** ⭐⭐ Di Carlo et al., "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control" (IROS 2018): SRBD + convex MPC 的经典论文，轮足 MPC 的足式基础
- **拓展** ⭐⭐⭐⭐ Jelavic et al., "Sampling-Based Motion Planning for Legged Robots" (RSS 2021): 采样 + 优化的模式序列规划，热启动策略
- **基础** ⭐⭐ Bjelonic et al., "Keep Rollin'—Whole-Body Motion Control and Planning for Wheeled Quadrupedal Robots" (RAL 2019): 轮足全身运动控制的早期奠基工作，WBC 轮速任务思想的源头
- **前沿** ⭐⭐⭐ "Real-time Whole-body MPC for Bipedal Locomotion with a Novel Kino-dynamic Model and Warm-start Method" (2025): 把建模抽象层级提升到 kino-dynamic，呼应本章热启动讨论
- **前沿** ⭐⭐⭐ "Estimation-based Disturbance Adaptive MPC for Wheeled Biped Robots" (2025): 在线扰动估计 + 自适应 MPC，对本章安全降级思想的主动式补充

---

## 术语速查表

本章新引入或重点使用的术语中英对照与一句话定义：

| 术语（中文） | English | 一句话定义 |
|------------|---------|----------|
| 切换系统 | Switched System | 动力学、约束、代价随离散模式分段变化的系统 |
| 模式 | Mode | 单个末端的接触状态：swing/foot/wheel 三值之一 |
| 质心动力学 | Centroidal Dynamics | 用质心角动量+线动量（6 维）描述整体动力学的简化模型 |
| 单刚体动力学 | SRBD | 把机器人近似为单个刚体的最简动力学模型 |
| kino-dynamic 模型 | Kino-dynamic Model | 质心动力学叠加全身运动学约束的中等抽象层级模型 |
| 滚动约束 | Rolling Constraint | 轮纯滚动的 Pfaffian（非完整）速度约束，三行：纵向/横向/法向 |
| guard（切换面） | Guard | 状态空间中触发模式切换的超曲面 $g(\mathbf{x})=0$ |
| 重置映射 | Reset Map | 切换瞬间状态跳变的映射 $\mathbf{x}^+=\Delta(\mathbf{x}^-)$ |
| 冲量 | Impulse | 接触力在零时间内的积分 $\boldsymbol{\Lambda}=\int\boldsymbol{\lambda}\,d\tau$，导致速度跳变 |
| 动力学一致投影 | Dynamically-consistent Projection | 度量 $M$ 下投影到约束流形的矩阵 $\mathbf{P}_M$ |
| 实时迭代 | SQP-RTI | 每控制周期只做一次 SQP 迭代的实时求解策略 |
| 热启动 | Warm-start | 用上一解平移作为本次优化初值，是实时性关键 |
| 模式驻留时间 | Dwell Time | 模式切换后必须维持的最短时间，防止抖动 |
| 滞回 | Hysteresis | 正向/反向切换阈值不同，防止边界抖动 |
| 发散分量 | DCM | $\xi=\mathbf{p}_G+\dot{\mathbf{p}}_G/\omega_0$，必须收敛到 ZMP |
| 各向异性摩擦锥 | Anisotropic Friction Cone | wheel 模式纵向（轮驱动限）与横向（摩擦限）能力不同的椭圆锥 |

---

## 数学符号速查表

术语表给的是"名词"，本表给的是"记号"——把全章散落的数学符号集中列出含义、维度与典型取值，便于在重读推导时随手回查。维度按本章 Go2+wheels 设定（4 腿、各 1 轮、12 关节 + 4 轮）给出。

| 符号 | 含义 | 维度 / 典型值 | 首次出处 |
|------|------|-------------|---------|
| $\mathbf{x}$ | MPC 状态向量（base 位姿+速度、关节角、轮角速度） | $n_x=29$ | 77.3 |
| $\mathbf{u}$ | MPC 输入向量（接触力、关节速度/力矩、轮速参考） | $n_u=28$ | 77.3 |
| $\sigma(t)$ | 模式信号，每个末端取 $\{\text{swing},\text{foot},\text{wheel}\}$ | $\{0,1,2\}^4$ | 77.2 |
| $\mathbf{v}$ | 广义速度（floating base + 关节 + 轮） | $24$（Go2+wheels） | 77.2 |
| $M(\mathbf{q})$ | 广义质量矩阵 | $24\times24$，对称正定 | 77.2 |
| $\mathbf{h}$ | 科氏/离心 + 重力项 $C\mathbf{v}+\mathbf{g}$ | $24$ | 77.2 |
| $S$ | 驱动选择矩阵（挑出可驱动自由度） | $n_\tau\times24$ | 77.2 |
| $J_c$ | 接触雅可比（堆叠所有激活接触） | $3n_c\times24$ | 77.2 |
| $\boldsymbol{\lambda}$ | 接触力（每接触 3 维：法向+两切向） | $3n_c$ | 77.2 |
| $\mu$ | 摩擦系数 | $0.3\sim0.8$（地面相关） | 77.5c |
| $\mathbf{P}_M$ | 动力学一致投影 $\mathbf{I}-M^{-1}J_c^T(J_cM^{-1}J_c^T)^{-1}J_c$ | $24\times24$ | 77.2b |
| $\mathbf{v}^-,\mathbf{v}^+$ | 切换前/后广义速度 | $24$ | 77.2b |
| $\boldsymbol{\Lambda}$ | 接触冲量 $\int\boldsymbol{\lambda}\,d\tau$ | $3n_c$ | 77.2b |
| $\Delta T$ | 切换能量损失 $\tfrac12\|J_c\mathbf{v}^-\|^2_{(\cdot)}$ | 标量，$\geq0$ | 77.2b |
| $\mathbf{Q}_m,\mathbf{R}_m$ | 模式 $m$ 相关的状态/输入权重矩阵 | $n_x\times n_x$ / $n_u\times n_u$ | 77.3 |
| $g(\mathbf{x})$ | guard 函数，$g=0$ 触发切换 | 标量 | 77.2b |
| $\boldsymbol{\xi}$ | 发散分量 DCM $\mathbf{p}_G+\dot{\mathbf{p}}_G/\omega_0$ | $\mathbb{R}^2$ 或 $\mathbb{R}^3$ | 77.6 |
| $\omega_0$ | DCM 自然频率 $\sqrt{g/z_c}$ | $\sim3.1\,\mathrm{s^{-1}}$（$z_c\!=\!1$ m） | 77.6 |
| $N$ | MPC 预测步数（horizon 节点数） | $10\sim30$ | 77.5 |
| $\Delta t$ | 离散化步长 | $20\sim50$ ms | 77.5 |
| $k$ | 摩擦锥多面体近似面数 | $4\sim8$ | 77.5c |

> **使用提示**：维度是排查 QP 组装错误的第一道防线。组装 Hessian 与约束矩阵时，先用上表核对每个块的行列数是否对齐——绝大多数"矩阵维度不匹配"的崩溃，根因都是把 $n_x=29$（含轮角速度状态）误用成足式的 $25$，或把 $J_c$ 的接触数 $n_c$ 写成了固定值而没随模式更新。

---

## 知识点总表

| 编号 | 知识点 | 核心要点 | 对应节 | 难度 |
|------|--------|---------|--------|------|
| K1 | 模式组合爆炸 | 4 腿、每腿 3 模式，共 $3^4=81$ 组合，不可枚举序列 | 77.1 | ⭐⭐ |
| K2 | 三重数学困难 | 组合爆炸 + 混合约束 + 新决策变量 | 77.1 | ⭐⭐ |
| K3 | 轮足统一动力学 | $M\dot{v}+h=S^T\tau+\sum J_c^T\lambda$，foot/wheel 力约束不同 | 77.2 | ⭐⭐⭐ |
| K4 | centroidal 简化 | 动力学降为 6 维，接触力可行域随模式变 | 77.2 | ⭐⭐⭐ |
| K5 | SRBD 轮足扩展 | 状态加轮角速度，输入加轮速参考 | 77.2 | ⭐⭐⭐ |
| K6 | guard/reset map | 切换瞬间速度投影到约束流形，位置不变 | 77.2b | ⭐⭐⭐⭐ |
| K7 | 切换能量损失 | $\Delta T\propto\|J_c\mathbf{v}^-\|^2$，落地速度越小冲击越柔和 | 77.2b | ⭐⭐⭐⭐ |
| K8 | 状态/输入向量 | $n_x=29$, $n_u=28$（比足式各多 4） | 77.3 | ⭐⭐⭐ |
| K9 | 模式相关代价 | 权重 $\mathbf{Q}_m,\mathbf{R}_m$ 随模式变化的分段二次型 | 77.3 | ⭐⭐⭐ |
| K10 | Q/R 调参原则 | 权重 = 偏差危险程度；安全量权重最高 | 77.3 | ⭐⭐⭐ |
| K11 | QP 组装五步 | 离散化→决策向量→二次型→等式→不等式 | 77.3 | ⭐⭐⭐⭐ |
| K12 | 模式调度三策略 | 启发式 / 采样优化 / 学习 | 77.4 | ⭐⭐⭐ |
| K13 | 切换工程约束 | 最短驻留 + 滞回 + 速度连续性 | 77.4 | ⭐⭐⭐ |
| K14 | SQP-RTI | 单次迭代 + 热启动，实时性来自时间连续性+反馈 | 77.5 | ⭐⭐⭐⭐ |
| K15 | OCS2 五组件 | Dynamics/Constraint/Cost/ModeSchedule/Solver | 77.5 | ⭐⭐⭐⭐ |
| K16 | 安全降级阶梯 | 衰减上一解→低速 foot→站立锁定 | 77.5 | ⭐⭐⭐ |
| K17 | WBC 轮速任务 | 加速度级软任务，优先级低于基座姿态 | 77.5b | ⭐⭐⭐ |
| K18 | 三类安全裕度 | 几何（ZMP）/ 摩擦 / 轮速，取最小值 | 77.6 | ⭐⭐⭐ |
| K19 | MPC+RL 接口 | MPC 给质心/力/模式参考，RL 软跟踪 | 77.8 | ⭐⭐⭐ |
| K20 | 抽象层级谱系 | SRBD→centroidal→kino-dynamic→全身的 Pareto 前沿 | 77.9 | ⭐⭐⭐ |

---

## 跨章综合练习

**综合题 1**（需要复合/20 + 复合/60 + 本章知识）：Go2+wheels 系统有 24 维广义速度。写出完整的 WBC QP：目标函数包含基座姿态跟踪、轮速匹配和力矩最小化；约束包含动力学方程 $M\dot{v}+h=S^T\tau+J_c^T\lambda$、摩擦锥 $\|\lambda_t\| \leq \mu\lambda_n$、Pfaffian 滚动约束和力矩限幅 $|\tau_i| \leq \tau_{\max}$。写出 QP 的标准形式 $\min \frac{1}{2}z^T H z + c^T z$ s.t. $Az \leq b, Cz = d$。

**综合题 2**（需要足式/110 + 本章知识）：比较 OCS2 中纯足式 MPC 和轮足 MPC 的 `ModeSchedule` 数据结构。纯足式的模式序列是 $\{0, 1\}^4$ 的时间线，轮足的模式序列是 $\{0, 1, 2\}^4$ 的时间线。在 OCS2 的 event-time 框架中，如何处理三值模式的切换事件？（提示：阅读 OCS2 的 `ModeScheduleManager` 源码）

**综合题 3**（需要足式/90 WBC + 复合/60 Pfaffian + 本章知识）：分析轮足 WBC 中从 foot 模式切换到 wheel 模式时，QP 结构如何变化。具体来说：（1）等式约束从 $J_c v = 0$ 变成 Pfaffian 约束，约束矩阵如何修改？（2）摩擦锥从圆锥变成椭圆锥，不等式约束矩阵如何修改？（3）代价权重矩阵中新增轮速匹配项，Hessian 矩阵如何变化？画出 foot 模式和 wheel 模式下 QP 的 Hessian 矩阵稀疏结构对比图。

**综合题 4**（需要复合/20 浮基动力学 + 77.2b + 足式/90 WBC）：把 77.2b 的落地 reset map 与 WBC 接触约束统一分析。（1）写出落地 reset map 的投影矩阵 $\mathbf{P}_M = \mathbf{I} - M^{-1}J_c^T(J_c M^{-1}J_c^T)^{-1}J_c$ 与足式/90 中 WBC 接触零空间投影矩阵，验证两者形式相同。（2）说明为什么"速度层投影（碰撞）"和"加速度层投影（WBC）"用的是同一个投影算子。（3）若一条腿从 swing 高速落地（接触点竖直速度 $-1.5$ m/s）后立即切换到 wheel 模式，分析落地冲量与轮速匹配应如何协调，才能让总的速度跳变最小。给出你的切换时序设计。

---

## 本章与后续章节的关系

本章在项目知识图谱中承上启下。下表说明本章为哪些后续章节铺垫了什么知识点，以及后续章节如何复用本章内容：

| 后续章节 | 关系 | 本章铺垫的知识点 |
|---------|------|---------------|
| 复合/80 Wheel-Legged-Gym RL | 强依赖 | 模式序列 $\sigma(t)$ 进入 RL observation（77.8）；滚动约束残差用于 reward shaping（77.5b）；ZMP/DCM 裕度作为稳定性 reward 项（77.6） |
| 复合/90 Swiss-Mile 商业化 | 案例延伸 | MPC+RL 混合架构（77.7）；商业部署的三大系统瓶颈（77 商业现状节）作为工程落地的现实约束 |
| 复合/130 OCS2 实践 | 实现承接 | OCS2 五组件接口与滚动约束实现（77.5）；SQP-RTI 配置与热启动（77.5）；API 速查表直接复用 |
| 复合/210 RAMBO 残差 RL | 前沿衔接 | MPC 模型误差的来源（77.2 简化假设、77.2b 冲击建模误差）正是残差 RL 要补偿的对象；MPC-RL 状态映射（77.8） |
| 足式/110 OCS2 双线程 MPC | 反向依赖 | 本章是足式 MPC 的三值扩展；理解本章后可回看足式如何作为轮足的二值特例 |

**知识树的位置**：如果把整个运动控制方向看作一棵树，足式 MPC（足式/110）是主干，本章是主干上长出的"轮足"分支。本章的根问题是"如何在统一优化框架中处理 swing/foot/wheel 三模式切换"，树干是 OCP 形式化 → 模式调度 → 数值求解三段递进，树枝是冲击动力学（77.2b）、摩擦锥近似（77.5c）、ZMP/DCM 扩展（77.6）等细化主题。读完本章后，读者应能把"轮足"这根分支完整挂回到"足式控制"主干上——理解轮足不是另起炉灶，而是足式框架的自然扩展。

> **本质洞察**：本章在整个项目中扮演"收束点"的角色——它把前面分散学习的浮基动力学（复合/20）、Pfaffian 约束（复合/60）、WBC 分层优化（足式/90）、OCS2 切换系统（足式/110）四条线索，在"轮足混合 MPC"这一个具体问题上汇聚成一个完整可运行的控制栈。这种"收束"正是体系化教学的价值——单独学每个工具时容易只见树木，只有在一个真实问题上把它们组装起来，才能看见森林。后续的 RL 章节（复合/80、210）则是从这个收束点再次发散，探索"学习"如何增强这个控制栈。

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MPC 输出 NaN | 初始猜测不可行或约束矛盾 | 1. 打印初始状态 2. 检查约束可行性 3. 降低 horizon 4. 检查模式序列一致性 | 77.3, 77.5 |
| KKT 残差持续升高 | 模式切换导致热启动失效 | 1. 检查切换时间戳 2. 增加切换过渡窗口 3. 切换时多做几次 SQP 迭代 | 77.4, 77.5 |
| 模式来回抖动 | 切换条件无滞回 | 1. 画模式时间线 2. 加入滞回阈值 3. 增加最短驻留时间 | 77.4 |
| 切换瞬间力矩尖峰 | 约束突变无平滑过渡 | 1. 画力矩时间线 2. 加入 $\alpha(t)$ 过渡函数 3. 检查 WBC 优先级 | 77.4, 77.6 |
| 轮速命令剧烈波动 | 轮速未加入状态向量 | 1. 检查状态定义 2. 确认轮速有积分动力学 3. 加入轮速变化率惩罚 | 77.2, 77.3 |
| 高速转弯侧滑 | 只检查 ZMP 不检查摩擦 | 1. 画摩擦裕度曲线 2. MPC 加入摩擦锥约束 3. 降低最大横向加速度 | 77.6 |
| 仿真成功但真机失败 | Sim2Real gap（延迟/摩擦/执行器模型） | 1. 比较仿真和真机的约束残差 2. 增加仿真中的延迟和噪声 3. 从低速开始 | 77.7 |
| 落地时足端弹跳/冲击噪声 | reset map 被忽略，触地竖直速度过大 | 1. 检查摆动轨迹末端竖直速度是否趋零 2. 代价函数加 $\|J_c\mathbf{v}^-\|^2$ 惩罚 3. 检查 guard 与网格对齐 | 77.2b |
| 切换时刻 KKT 残差突跳 | guard 触发时刻不在离散网格点上 | 1. 用 event-time 插入切换节点 2. guard 处加密网格 3. 检查事件检测逻辑 | 77.2b, 77.5 |

---

## API 速查表

本章涉及的核心 OCS2 / 求解器接口签名与说明（概念签名，具体以对应库版本为准）：

| 组件 | 接口签名（概念） | 说明 |
|------|---------------|------|
| 系统动力学 | `VectorXd SystemDynamics::computeFlowMap(t, x, u, pre)` | 返回 $\dot{\mathbf{x}}=f_\sigma(\mathbf{x},\mathbf{u})$；轮足需加轮速积分行 |
| 动力学线性化 | `VectorFunctionLinearApproximation SystemDynamics::linearApproximation(...)` | 返回 $\mathbf{A}_k,\mathbf{B}_k$；推荐 CppAD 自动微分 |
| 等式约束 | `VectorXd StateInputConstraint::getValue(t, x, u, pre)` | 滚动约束在此返回三维残差（纵/横/法向）|
| 约束线性化 | `VectorFunctionLinearApproximation StateInputConstraint::getLinearApproximation(...)` | 返回 $\mathbf{C}_k,\mathbf{D}_k$；先数值差分验证再迁移解析 |
| 代价 | `ScalarFunctionQuadraticApproximation StateInputCost::getQuadraticApproximation(...)` | 返回模式相关的 $\mathbf{Q}_m,\mathbf{R}_m$ Hessian |
| 模式调度 | `ModeSchedule ModeScheduleManager::getModeSchedule(t)` | 返回 `{eventTimes, modeSequence}`；轮足为三值模式编码 |
| 求解器设置 | `SqpSettings{maxNumIterations, timeHorizon, numPartitions, ...}` | RTI 设 `maxNumIterations=1` |
| MPC 主循环 | `MPC_MRT::updatePolicy(t, x)` / `getOptimizedInput(t)` | 更新策略并取第一步控制量 |
| 接触雅可比 | `pinocchio::computeJointJacobians(model, data, q)` | 计算 $J_c$，供约束与 reset map 使用 |
| QP 求解后端 | HPIPM（带状）/ qpOASES / ProxQP | 实时 MPC 须用利用块对角+带状结构的求解器 |

> **使用提示**：上表中"线性化"类接口是轮足扩展的主要工作量所在——滚动约束的解析导数 $\partial(J_c\mathbf{v})/\partial\mathbf{q}$ 需要 $\dot{J}_c$，手推易错。工程上的标准做法是先用中心差分（步长 $\sim10^{-7}$）实现并验证数值正确性，确认 SQP 能收敛后，再为热点路径迁移到 CppAD/CasADi 自动微分以提速。切忌一上来就手写解析导数——调试成本极高。

---

## 研究实践建议

针对不同目标的读者，给出分层次的实践路径建议：

**如果你是初学者（目标：跑通一个轮足 MPC demo）**

1. 不要从 81 模式的完整系统起步。先实现**纯 wheel 模式**的 centroidal MPC（无模式切换），验证平地直行和转弯。
2. 用本章 77.3 的 2D 简化算例打地基——先在 Python + scipy 里把 QP 组装逻辑跑通，建立对决策变量布局的直觉。
3. 再加入 **foot 与 wheel 二值切换**（先不要 swing），重点调通 77.4 的滞回+最短驻留逻辑。
4. 最后才引入 swing 和落地 reset map（77.2b）。这是最难的一步，放最后。

**如果你是研究者（目标：发表轮足 MPC 相关工作）**

1. 先把本章 77.9 的"开放问题"表读透——这是寻找 research gap 的起点。
2. "切换时机可微优化"和"MPC+RL 理论保证"是当前最有空间的方向，但难度也最高（⭐⭐⭐⭐）。
3. 复现基线是必经之路：建议从 OCS2 的官方轮足 example 起步，先复现 Bjelonic 2021 的结果，再做增量创新。
4. 真机实验的价值远高于纯仿真——审稿人对轮足工作几乎都要求真机验证（77.7 已说明 Sim2Real 的特殊难点）。

**如果你是工程师（目标：在产品上落地）**

1. 算法不是瓶颈，系统集成才是。把 80% 的时间预算留给 77（商业现状节）指出的三个系统问题：地形感知鲁棒性、状态估计漂移、机械磨损。
2. 安全降级（77.5）不是可选项——产品级系统必须有完整的降级阶梯，且要在真机上反复验证降级触发逻辑。
3. 优先 MPC+RL 混合架构（参考 Swiss-Mile 2024）而非纯 MPC——RL 提供的鲁棒性在户外非结构化环境中价值巨大。
4. 从低速（< 1 m/s）开始积累真机经验，逐步提速。高速轮足运动的失稳是不可逆的硬件风险。

> **本质洞察**：本章的学术核心（OCP 形式化、SQP-RTI、模式调度、冲击动力学）是"可迁移的知识"——学会后适用于任何轮足平台。但真正决定项目成败的"系统集成能力"是"经验性知识"——只能通过真机踩坑积累。建议的学习策略是：用本章建立扎实的理论框架，然后尽早接触真机，让理论指导你更快地理解真机暴露的问题。理论和实践不是先后关系，而是螺旋上升的互补关系。

---

## 轮足 MPC 在商业平台上的应用现状（2025-2026）

轮足混合 MPC 已从学术原型走向商业部署。Swiss-Mile 在 ANYmal-on-wheels 平台上验证了 MPC + RL 混合架构在城市配送场景中的可行性——轮模式用于平坦路面的高效巡航（速度可达 4 m/s），足模式用于台阶和路缘石的越障，模式切换由地形分类器触发。Unitree B2-W 采用了类似的轮足混合平台设计，2025 年的第三方研究已在 B2-W 上复现了基于 OCS2 的轮足 MPC 控制器，验证了本章介绍的 SQP-RTI 求解管线在商用硬件上的实时性（1 kHz 控制频率下 MPC 更新周期约 20 ms）。

**当前商业部署的主要瓶颈**不在算法层面，而在系统集成层面：（1）地形感知的鲁棒性——户外光照变化、雨雪天气下的深度估计退化直接导致模式切换错误；（2）长时间运行的状态估计漂移——轮足系统的里程计在模式切换时容易出现跳变；（3）机械磨损——轮模式下的高速运动对轮胎和轮轴的磨损显著高于实验室环境的预期。这些工程问题在学术论文中很少讨论，但对商业化落地至关重要。

> **给学习者的建议**：轮足 MPC 的学术核心（OCP 形式化、SQP-RTI 求解、模式调度）在本章已完整覆盖。如果你未来从事轮足相关的工业研发，上述三个系统集成瓶颈将消耗你大部分的工程时间。建议在仿真验证完成后，尽早在真机上做低速测试，积累对 Sim2Real gap 的第一手经验。

**展望**：轮足 MPC 的下一个重要方向是与 RL 的深度融合——不是简单的"MPC 做前馈 + RL 做残差"，而是让 RL 直接学习 MPC 的代价函数和约束参数，使整个优化管线自适应任务需求。复合/210 章（RAMBO）是这一方向的具体案例。

*掌握本章的 OCP 形式化和 SQP-RTI 求解管线后，即可进入复合/210 了解残差 RL 如何弥补 MPC 的模型误差。*

---

**—— 本章终 ——**
