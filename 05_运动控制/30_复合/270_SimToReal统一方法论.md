# 第 97 章：Sim-to-Real 统一方法论——从仿真到真机的系统化迁移框架

| 元信息 | 值 |
|--------|-----|
| 难度 | ⭐⭐⭐⭐（跨方法理论推导 + 工程部署闭环 + 方法选型决策） |
| 预计时间 | 2 周（40-50 小时） |
| 前置依赖 | 足式/190_腿足RL训练栈（PPO/奖励设计），足式/200_RL的CPP部署（ONNX/LibTorch），复合/240_ASAP_SimToReal（Delta-Action 实例），复合/110_轮足SimToReal与硬件（硬件接口） |
| 下游章节 | 复合/280_端到端学习（视觉策略部署），复合/290_大模型与具身智能（世界模型） |
| 核心关键词 | Domain Randomization、System Identification、Actuator Network、RMA、Delta-Action、Teacher-Student、在线适应 |

---

## 前置自测

📋 **前置自测（答不出 >= 2 题，先回复合/240 和足式/190 复习）**

| # | 问题 | 前置来源 | 合格关键词 |
|---|------|---------|-----------|
| 1 | PPO 的 clipped surrogate objective 中，clip 比率 $\epsilon$ 的作用是什么？ | 足式/190 | 限制策略更新步长，防止过大偏移 |
| 2 | Domain Randomization 的目标是让仿真更逼真，还是让策略更鲁棒？ | 足式/190 | 让策略更鲁棒，真实世界是随机化分布中的一个样本 |
| 3 | ONNX Runtime 部署策略时，观测归一化参数从何而来？不一致会导致什么？ | 足式/200 | 从训练端导出的 running mean/std；不一致导致动作幅值完全错误 |
| 4 | ASAP 的 delta-action 模型修正的是动力学还是动作？为什么？ | 复合/240 | 修正动作；因为修正动作只需修改 policy 输出，不需修改仿真器梯度 |
| 5 | Teacher-Student 架构中教师和学生的信息差是什么？ | 足式/190 | 教师访问特权信息（地面真值、摩擦系数等），学生只有可部署传感器 |

**自测标准**：
- 5/5 正确：直接阅读本章
- 3-4/5 正确：边读边查阅前置章节
- 0-2/5 正确：建议先完成足式/190 + 足式/200 + 复合/240

---

## 本章目标

学完本章后，你应该能够：

1. 系统分类 Sim-to-Real Gap 的六个维度，并判断真机失败属于哪一类
2. 完整推导 Domain Randomization 的鲁棒优化理论基础，理解其与分布鲁棒优化和 CVaR 的数学联系
3. 掌握 System Identification 的经典最小二乘推导和现代学习辨识方法
4. 理解 Actuator Network 的完整建模流程：数据采集、网络结构、训练和部署
5. 推导观测延迟的状态增广方法，理解 Action History 缓冲器的数学本质
6. 掌握 RMA 的完整教师-学生-适应模块三阶段训练流程
7. 理解 Delta-Action / 残差策略的数学形式化与工程优势
8. 运用 Teacher-Student 蒸馏将特权策略迁移为可部署策略
9. 面对新的 Sim-to-Real 问题时，用决策树选择合适的方法组合

---

## 97.1 动机与全景：为什么 Sim-to-Real 是机器人 RL 的瓶颈？ ⭐

### 动机：仿真中 10 分钟跑通，真机 10 秒摔倒

假设你在 IsaacGym 中训练了一个四足行走策略，奖励曲线漂亮地收敛了，仿真视频里机器人在各种地形上健步如飞。你满怀信心地把策略导出为 ONNX，加载到 Unitree Go2 上——结果机器人站起来就剧烈抖动，三步之内倒地不起。

这不是个例。几乎所有从零开始做 sim-to-real 的研究者都会经历这个"仿真天堂，真机地狱"的落差。问题的根源在于：**仿真器是对现实世界的近似模型，而不是现实世界本身**。仿真器中的每一个假设——刚体接触、理想执行器、零延迟传感——都会在部署时变成一个"漏洞"。

### 如果不系统解决 Sim-to-Real，会怎样？

许多初学者采用的做法是"头痛医头"：真机抖动就降增益，走不稳就加阻尼，滑倒就换地板。这种逐症状修补的方式有三个致命问题：

1. **无法迁移**：每换一台机器人、每换一个场景，都要从头调一遍
2. **无法诊断**：当策略失败时，不知道是传感器噪声的问题、执行器延迟的问题，还是摩擦建模的问题
3. **无法复现**：没有系统化流程，同一个人做两次可能得到完全不同的结果

### Sim-to-Real Gap 的六维分类学

> **本质洞察**：Sim-to-Real Gap 不是单一问题，而是六个正交维度的差异同时存在。像调 PID 一样逐参数试错是行不通的——必须先理解差异的结构，才能选择正确的工具。

为了系统化地思考 Sim-to-Real Gap，我们需要一个完备的分类框架。以下六个维度覆盖了文献中报告的几乎所有 Gap 来源：

| 维度 | Gap 来源 | 典型表现 | 影响量级 | 对策方向 |
|------|---------|---------|---------|---------|
| **D1: 动力学参数** | 质量/惯量/摩擦系数/弹性系数与标称值偏差 | 行走速度偏差、转弯漂移 | 中 | DR / SysID |
| **D2: 接触模型** | 刚性穿透 vs 柔性接触、摩擦锥离散化、接触检测阈值 | 滑倒、弹跳、脚步声异常 | 高 | DR + 接触参数辨识 |
| **D3: 执行器动力学** | 电机带宽有限、齿轮回程间隙、温度依赖的力矩常数、非线性摩擦 | 高频抖动、力矩跟踪偏差、温升后性能衰退 | 极高 | Actuator Network / SysID |
| **D4: 传感器与观测** | IMU 偏置漂移、编码器量化噪声、通信丢包、数据异步 | 状态估计漂移、速度估计跳变 | 高 | 观测噪声 DR + 滤波 |
| **D5: 时间延迟** | 通信延迟、计算延迟、传感器采样延迟 | 相位滞后、步态不同步、在快速运动中失稳 | 高 | 延迟随机化 + Action History |
| **D6: 软件与部署** | 观测归一化不一致、关节顺序错误、坐标系约定不同、浮点精度差异 | 策略输出完全错误（幅值差数量级）、关节混乱 | 致命 | 配置对齐检查 |

这个分类不是学术上的整理练习，而是工程上的**故障诊断框架**。当真机失败时，第一步是判断问题属于哪个维度（或哪几个维度的叠加），然后针对性地选择工具。

### 从分类到方法：全景知识地图

基于上述六维分类，Sim-to-Real 领域发展出了一系列互补的方法。它们之间不是"越新越好"的线性演进关系，而是**针对不同维度 Gap 的不同工具**：

```
                    Sim-to-Real 方法全景
                          │
          ┌───────────────┼───────────────┐
          │               │               │
    训练时方法          部署时方法        混合方法
          │               │               │
    ┌─────┼─────┐    ┌────┼────┐     ┌────┼────┐
    │     │     │    │    │    │     │    │    │
   DR   SysID  ActNet  RMA  Delta  T-S蒸馏 可微仿真
   │     │     │    │    │         │
   覆盖D1  精确D1   精确D3  在线     在线     训练-部署
   D2,D4  D2,D3        适应D1-D4  修正D1-D3  信息差
   D5
```

| 方法 | 核心思想 | 主要针对维度 | 需要真机数据？ | 本章节号 |
|------|---------|------------|-------------|---------|
| Domain Randomization | 在训练分布中覆盖真实参数 | D1, D2, D4, D5 | 否 | 97.2 |
| System Identification | 测量真机参数校准仿真器 | D1, D2, D3 | 是（离线） | 97.3 |
| Actuator Network | 神经网络学习执行器非线性 | D3 | 是（离线） | 97.4 |
| Action History / 延迟建模 | 状态增广包含历史信息 | D5 | 否 | 97.5 |
| RMA 在线适应 | 运行时估计环境参数 | D1, D2, D3 | 否（部署时自适应） | 97.6 |
| Delta-Action / 残差策略 | 学习对基线策略的修正 | D1, D2, D3 | 是（少量） | 97.7 |
| Teacher-Student 蒸馏 | 将特权信息压缩为可部署表征 | 信息差 | 否 | 97.8 |
| Physics Parameter Adaptation | 在线估计物理参数 | D1, D2 | 是（在线） | 97.9 |

> 回顾 足式/190_腿足RL训练栈：我们在那里学习了 PPO 训练的基本流程——奖励设计、课程学习、并行环境。但那里假设策略在仿真中训练好就能直接用。本章正是要回答："训练好之后，如何跨越仿真到真机的鸿沟？"

### 历史演进脉络

Sim-to-Real 方法的演进不是偶然的，每一代都是对上一代局限性的回应：

| 时间 | 里程碑 | 解决的问题 | 引入的新问题 |
|------|--------|-----------|------------|
| 2017 | Tobin et al., Domain Randomization (OpenAI) | 无需真机数据即可训练可迁移策略 | 范围选择靠经验，策略过于保守 |
| 2018 | Tan et al., Sim-to-Real 四足 (Google) | 结合 SysID + DR + Actuator Network | 需要大量真机辨识数据 |
| 2019 | OpenAI, Rubik's Cube | 大规模 DR 解决灵巧操作 | 需要海量并行环境，训练成本极高 |
| 2021 | Kumar et al., RMA (UC Berkeley) | 在线适应无需真机微调 | 适应模块训练需要精心设计 |
| 2023 | Cheng et al., Extreme Parkour | DR + Teacher-Student + 课程学习 | 训练流程极复杂 |
| 2024 | ICLR Workshop, DR 理论化 | DR 的分布鲁棒优化理论基础 | 理论与实践差距仍大 |
| 2025 | ASAP (He et al.) | Delta-Action 用少量真机数据高效修正 | 需要真机 rollout 数据 |
| 2025 | PACE (ETH) | 20 秒编码器数据即可辨识执行器 | 对复杂执行器建模能力有限 |
| 2025 | Newton 物理引擎 (NVIDIA/DeepMind/Disney) | GPU 加速可微仿真 | 接触微分仍不稳定 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：训练和部署的观测归一化不一致**
> 错误做法：训练时用 IsaacGym 内部的 running normalization，部署时重新初始化统计量
> 现象：策略输出的关节角幅值差 10-100 倍，机器人剧烈抖动或完全不动
> 根本原因：观测的 mean/std 与训练时不同，等价于输入了完全不同的状态
> 正确做法：导出训练结束时的 mean/std，与 ONNX 模型一起保存，部署时使用完全相同的归一化参数
> 自检方法：固定输入（全零向量）分别过训练端和部署端，对比输出是否一致

> 💡 **概念误区：认为 DR 范围越大越好**
> 新手想法："既然 DR 是让策略更鲁棒，那把所有参数的范围开到最大不就行了？"
> 实际上：DR 范围过大会导致策略过于保守——像一个只练过太极拳的人，动作虽然安全但速度和精度都很差。本质原因是过大的随机化引入了大量"不可能存在于真实世界"的环境配置，策略为了在这些极端情况下存活而牺牲了正常情况下的性能
> 正确做法：从物理合理的范围出发（如质量 $\pm 30\%$、摩擦系数 $[0.3, 1.2]$），逐步扩大并监控仿真性能

> 🧠 **思维陷阱：认为 Sim-to-Real 方法之间是"越新越好"的线性关系**
> 新手想法："RMA 是 2021 年的，ASAP 是 2025 年的，所以直接用 ASAP 就行了"
> 实际上：不同方法针对不同类型的 Gap。如果你的主要问题是摩擦系数不确定（D1），DR 可能就够了；如果是执行器非线性（D3），Actuator Network 更合适。方法选择取决于 Gap 的类型和严重程度，而非发表时间
> 正确思维：先用六维分类学诊断 Gap 的主要来源，再选择对应的工具

### 练习 97.1

**[A] ⭐** 列出你使用或了解的一个机器人平台（四足/人形/机械臂均可），分析其 Sim-to-Real Gap 在六个维度上各自的严重程度（用"低/中/高/极高"评级），并解释理由。

**[B] ⭐⭐** 阅读 Tan et al. 2018 "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots" 的方法部分，列出论文中使用了上表哪些方法的组合。为什么单一方法不够？

**[C] ⭐⭐⭐** （跨章综合题）结合 复合/20_浮动基座臂统一动力学 中的质量矩阵分块结构 $\mathbf{M}_{ba}$，分析为什么复合机器人（四足+臂）的 D1 维度 Gap 比纯四足更严重。提示：臂的构型变化会改变基座惯量。

---

## 97.2 Domain Randomization：从工程直觉到鲁棒优化理论 ⭐⭐

### 动机：为什么"随机一下"就能跨越 Gap？

在 足式/190_腿足RL训练栈 中，我们已经在训练配置中写过 `randomize_friction = True`、`mass_offset_range = [-0.5, 0.5]` 这样的设置。当时我们知道这叫"域随机化"，目的是让策略更鲁棒。但我们没有回答一个更深层的问题：**为什么在参数空间中"随机一下"就能让策略在未见过的真实环境中也有效？这背后有什么理论保障？**

要回答这个问题，我们需要从鲁棒优化的视角重新审视 Domain Randomization。

### 如果不做 Domain Randomization 会怎样？

假设你在仿真中精确设定了地面摩擦系数 $\mu = 0.6$，训练了一个完美的行走策略。这个策略学会了在 $\mu = 0.6$ 的地面上精确控制每一步的力矩和落脚点。

现在把这个策略部署到真机：
- 实验室地板的 $\mu \approx 0.4$（比训练时更滑）
- 策略施加的水平力超过了摩擦力上限
- 脚底打滑 → 状态估计跳变 → 策略输出更大的修正力矩 → 更严重的打滑
- 正反馈循环，3 步之内倒地

这就是**过拟合到单一仿真参数**的后果。策略成为了一个只能在 $\mu = 0.6$ 世界中生存的"特化物种"。

### 标准 RL 目标 vs DR 目标的数学对比

#### 标准 RL：在固定 MDP 上最大化期望回报

回顾标准 RL 的优化目标。一个 MDP 由参数 $\xi$（包括质量、摩擦、延迟等所有物理参数）完全确定，记为 $\mathcal{M}_\xi$。标准 RL 求解：

$$
\pi^*_\xi = \arg\max_\pi J(\pi, \xi) = \arg\max_\pi \mathbb{E}_{\tau \sim p_\xi(\cdot|\pi)} \left[ \sum_{t=0}^{T} \gamma^t r(s_t, a_t) \right]
$$

其中 $p_\xi(\cdot|\pi)$ 是在 MDP $\mathcal{M}_\xi$ 中由策略 $\pi$ 产生的轨迹分布。

问题在于：$\pi^*_\xi$ 是**关于 $\xi$ 的函数**。当真实世界参数 $\xi_{\text{real}} \neq \xi_{\text{sim}}$ 时，没有任何保证 $\pi^*_{\xi_{\text{sim}}}$ 在 $\mathcal{M}_{\xi_{\text{real}}}$ 中仍然表现良好。

#### Domain Randomization：在参数分布上最大化期望

DR 的核心思想是将优化目标从"在一个 MDP 上最大化"变为"在一族 MDP 上平均最大化"：

$$
\boxed{
\pi^*_{\text{DR}} = \arg\max_\pi \mathbb{E}_{\xi \sim p(\xi)} \left[ J(\pi, \xi) \right]
}
$$

其中 $p(\xi)$ 是环境参数的先验分布（例如 $\mu \sim \text{Uniform}(0.3, 1.0)$）。

这个公式的直觉含义是：我们不追求在任何单一环境中的最优性能，而是追求**在一族环境上的平均表现**。如果 $p(\xi)$ 的支撑集（support）覆盖了真实参数 $\xi_{\text{real}}$，那么最优策略在真实环境中至少有"不差于平均"的性能保障。

> **跨领域类比**：DR 类似于机器学习中的**数据增强**（Data Augmentation）。训练图像分类器时，我们对图像做随机裁剪、旋转、色彩抖动——不是为了让训练图像更"真实"，而是为了让模型对这些变换不敏感。类似地，DR 不是让仿真更逼真，而是让策略对参数变化不敏感。但关键区别在于：图像增强的变换空间是低维且直觉清晰的（旋转角度、亮度偏移），而 DR 的参数空间是高维且各参数之间可能存在复杂交互的。

### 鲁棒优化视角：DR 是 Minimax 的松弛

DR 的期望目标可以从另一个角度理解。考虑最坏情况鲁棒优化（minimax）：

$$
\pi^*_{\text{robust}} = \arg\max_\pi \min_{\xi \in \Xi} J(\pi, \xi)
$$

这要求策略在参数集 $\Xi$ 中**最差环境**下也表现良好。这个目标太悲观了——它会让策略极度保守，因为它需要应对可能永远不会出现的极端情况。

DR 的期望目标是 minimax 的一个**松弛**。数学上，对于任何分布 $p(\xi)$：

$$
\max_\pi \mathbb{E}_{\xi \sim p(\xi)} [J(\pi, \xi)] \geq \max_\pi \min_{\xi \in \text{supp}(p)} J(\pi, \xi)
$$

这意味着 DR 策略的平均性能至少不差于 minimax 策略的最差性能。DR 在鲁棒性和性能之间取了一个更合理的折中。

### 分布鲁棒优化（DRO）连接

更深层地，DR 可以被纳入**分布鲁棒优化**（Distributionally Robust Optimization, DRO）的框架。DRO 不固定参数分布 $p(\xi)$，而是在一个**分布集合** $\mathcal{P}$ 中寻找最差分布：

$$
\pi^*_{\text{DRO}} = \arg\max_\pi \min_{p \in \mathcal{P}} \mathbb{E}_{\xi \sim p} [J(\pi, \xi)]
$$

不确定性集合 $\mathcal{P}$ 的选择决定了 DRO 的保守程度：

| 不确定性集合 | 数学形式 | 等价于 | 保守程度 |
|-------------|---------|--------|---------|
| 所有分布 | $\mathcal{P} = \{\text{all distributions on } \Xi\}$ | Minimax（最差单点） | 极度保守 |
| KL 散度球 | $\mathcal{P} = \{p : D_{\text{KL}}(p \| p_0) \leq \delta\}$ | 指数倾斜的 soft-minimax | 中等 |
| $\chi^2$ 散度球 | $\mathcal{P} = \{p : D_{\chi^2}(p \| p_0) \leq \delta\}$ | CVaR 相关 | 中等 |
| 固定分布 | $\mathcal{P} = \{p_0\}$ | 标准 DR | 最不保守 |

#### CVaR 连接

当不确定性集合是 $\chi^2$ 散度球时，DRO 与**条件风险值**（CVaR）有精确的对偶关系。CVaR 定义为：

$$
\text{CVaR}_\alpha[X] = \mathbb{E}[X | X \leq \text{VaR}_\alpha[X]]
$$

即 $X$ 最差 $\alpha$ 分位数的期望。在 Sim-to-Real 语境下，这意味着优化策略在**最差 $\alpha$ 比例环境**中的平均表现。

Lim & Xu (2022) 证明了以下对偶关系：

$$
\min_{p \in \mathcal{P}_{\chi^2}} \mathbb{E}_p[J(\pi, \xi)] = \text{CVaR}_\alpha[J(\pi, \xi)]
$$

其中 $\alpha$ 与 $\chi^2$ 球的半径 $\delta$ 之间有显式关系。

**工程含义**：如果你用标准 DR 训练策略，但发现策略在某些极端环境中表现很差（尾部风险），可以把目标从 $\mathbb{E}[J]$ 切换为 $\text{CVaR}_\alpha[J]$。实现方式很简单——在 PPO 的 batch 中，按 episodic return 排序，只用最差 $\alpha$ 比例的 episode 来计算梯度。这等价于 LSDR (Mehta et al. 2020) 中的做法。

### 参数表设计方法论

理论告诉我们 DR 的优化目标是什么，但工程中最困难的部分是：**随机化哪些参数、范围设多大？**

#### 第一原则：物理合理性

每个随机化参数的范围必须有物理依据，不能凭感觉设。以下是从文献中整理的经验参数表：

| 参数类别 | 参数 | 标称值 | 随机化范围 | 来源 |
|---------|------|--------|-----------|------|
| **刚体参数** | 躯干质量 | $m_0$ | $[0.8 m_0, 1.2 m_0]$ | Rudin et al. 2022 |
| | 连杆质心偏移 | 0 | $[-2, 2]$ cm | Hwangbo et al. 2019 |
| | 连杆惯量 | $I_0$ | $[0.5 I_0, 1.5 I_0]$ | PACE 2025 |
| **接触参数** | 地面摩擦系数 | 0.7 | $[0.3, 1.2]$ | Walk These Ways 2023 |
| | 地面弹性 | 0.0 | $[0.0, 0.2]$ | Margolis et al. 2023 |
| | 接触刚度 | $k_0$ | $[0.5 k_0, 2.0 k_0]$ | Miki et al. 2022 |
| **执行器参数** | 电机力矩系数 | $K_\tau$ | $[0.85 K_\tau, 1.15 K_\tau]$ | Kumar et al. 2021 |
| | PD 增益 | $K_p, K_d$ | $\pm 20\%$ | Rudin et al. 2022 |
| **传感器噪声** | IMU 角速度偏置 | 0 | $[-0.05, 0.05]$ rad/s | Margolis et al. 2023 |
| | 关节编码器噪声 | 0 | $\sigma = 0.01$ rad | Hwangbo et al. 2019 |
| **延迟** | 观测延迟 | 0 | $[0, 40]$ ms | Miki et al. 2022 |
| | 动作延迟 | 0 | $[0, 20]$ ms | Walk These Ways 2023 |
| **外扰** | 随机推力 | 0 | $[0, 50]$ N, 间隔 8-15s | Rudin et al. 2022 |

#### 第二原则：逐步增加（课程式 DR）

不要一开始就把所有参数开到最大范围。推荐的策略是**课程式 Domain Randomization**：

1. **Phase 0**：零随机化，验证策略在标称参数下能学会任务
2. **Phase 1**：只开动力学参数（质量、摩擦），范围 $\pm 10\%$
3. **Phase 2**：加入传感器噪声和延迟
4. **Phase 3**：扩大范围到完整目标，加入外扰

每个 Phase 持续到策略收敛且性能不低于上一 Phase 的 90%。

#### 第三原则：消融验证

训练完成后，必须做消融实验验证每类随机化的贡献：

```python
# DR 消融验证框架
import numpy as np
from typing import Dict

def ablation_study(policy, env_factory, dr_config: Dict, n_eval: int = 100):
    """
    逐类关闭 DR 参数，评估策略性能变化。
    
    参数:
        policy: 训练好的策略
        env_factory: 环境工厂函数，接受 DR 配置
        dr_config: 完整的 DR 配置字典
        n_eval: 每个配置评估的 episode 数
    """
    results = {}
    
    # 基线：完整 DR 训练的策略在标称参数下的表现
    nominal_env = env_factory(dr_config={})  # 标称参数，无随机化
    baseline_return = evaluate(policy, nominal_env, n_eval)
    results['baseline'] = baseline_return
    
    # 逐类消融
    dr_categories = {
        'dynamics': ['mass_range', 'inertia_range', 'com_offset_range'],
        'contact':  ['friction_range', 'restitution_range'],
        'actuator': ['motor_strength_range', 'pd_gain_range'],
        'sensor':   ['imu_noise_std', 'encoder_noise_std'],
        'delay':    ['obs_delay_range', 'action_delay_range'],
        'external': ['push_force_range', 'push_interval'],
    }
    
    for category, params in dr_categories.items():
        # 关闭该类随机化，保留其他
        ablated_config = dr_config.copy()
        for p in params:
            ablated_config[p] = None  # 关闭该参数
        
        # 用消融后的 DR 重新训练（或在消融环境中评估）
        ablated_env = env_factory(dr_config=ablated_config)
        ablated_return = evaluate(policy, ablated_env, n_eval)
        results[f'without_{category}'] = ablated_return
        
        # 计算性能下降
        drop = (baseline_return - ablated_return) / baseline_return * 100
        print(f"关闭 {category}: 性能变化 {drop:+.1f}%")
    
    return results
```

### IsaacGym 中的 DR 实现

```python
# IsaacGym/IsaacLab 风格的 DR 配置（基于 Rudin et al. 2022）
class DomainRandomizationConfig:
    """域随机化参数配置。所有范围均有物理依据。"""
    
    # === 动力学参数（每个 episode 开始时采样一次） ===
    added_mass_range = [-1.0, 3.0]          # kg，躯干附加质量
    com_displacement_range = [-0.05, 0.05]  # m，质心偏移（三轴）
    
    # === 摩擦系数（按地形 tile 随机） ===
    friction_range = [0.3, 1.25]  # 无量纲
    
    # === 执行器（模拟电机个体差异） ===
    motor_strength_range = [0.85, 1.15]     # 力矩系数缩放因子
    kp_range = [0.8, 1.2]                   # P 增益缩放因子
    kd_range = [0.8, 1.2]                   # D 增益缩放因子
    
    # === 传感器噪声（每步叠加） ===
    noise_scales = {
        'dof_pos': 0.01,      # rad，关节位置噪声标准差
        'dof_vel': 1.5,       # rad/s，关节速度噪声标准差
        'ang_vel': 0.2,       # rad/s，角速度噪声标准差
        'gravity':  0.05,     # 重力方向噪声（模拟 IMU 偏置）
    }
    
    # === 延迟（模拟通信和计算延迟） ===
    action_delay_range = [0, 2]  # 步，动作延迟（1步=dt）
    
    # === 外扰（随机推力） ===
    push_robots = True
    push_interval_s = 10.0      # 平均推力间隔
    max_push_vel_xy = 0.5       # m/s，等效推力速度


def apply_domain_randomization(env, config):
    """
    在 IsaacGym 环境中应用域随机化。
    注意：每个并行环境独立采样参数。
    """
    num_envs = env.num_envs
    
    # --- 质量随机化 ---
    # 获取所有环境的躯干刚体索引
    base_idx = env.gym.find_actor_rigid_body_index(
        env.envs[0], env.actors[0], "base", gymapi.DOMAIN_ENV
    )
    for i in range(num_envs):
        props = env.gym.get_actor_rigid_body_properties(env.envs[i], env.actors[i])
        # 在标称质量基础上加随机偏移
        added_mass = np.random.uniform(*config.added_mass_range)
        props[base_idx].mass += added_mass
        env.gym.set_actor_rigid_body_properties(env.envs[i], env.actors[i], props)
    
    # --- 摩擦随机化 ---
    for i in range(num_envs):
        shape_props = env.gym.get_actor_rigid_shape_properties(env.envs[i], env.actors[i])
        for sp in shape_props:
            sp.friction = np.random.uniform(*config.friction_range)
        env.gym.set_actor_rigid_shape_properties(env.envs[i], env.actors[i], shape_props)
    
    # --- 观测噪声（在 step 中叠加） ---
    # 这部分在 env.step() 中实现，不在 reset 中
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：DR 参数在 episode 中间变化**
> 错误做法：每一步都重新采样摩擦系数
> 现象：策略根本无法学会稳定行走，因为"地面"在每一步都变化
> 根本原因：现实中摩擦系数在一个场景内是固定的，每步变化违反了物理规律
> 正确做法：动力学参数在 episode 开始时采样一次，episode 内保持不变；只有传感器噪声可以每步变化
> 自检方法：打印每步的环境参数，确认动力学参数只在 reset 时变化

> 💡 **概念误区：认为 DR 的随机化分布需要覆盖真实参数的精确值**
> 新手想法："真机摩擦是 0.5，那我随机化范围必须包含 0.5 这个精确值"
> 实际上：DR 的机制不是"碰巧在训练中遇到了真实参数"。正确的理解是：DR 让策略学会了对参数变化不敏感的行为模式（如更保守的步态、更低的重心）。即使真实参数在随机化范围的边缘甚至略微超出，策略仍可能有效迁移，因为它学到的是鲁棒性而非精确适配
> 关键区分：DR 不是让仿真"更像真实"，而是让策略"更抗变化"

> 🧠 **思维陷阱：所有参数用同一个随机化范围**
> 新手做法：所有参数统一 $\pm 30\%$
> 问题：不同参数的敏感度差异巨大。质量 $\pm 30\%$ 可能影响不大，但摩擦系数 $\pm 30\%$ 可能导致完全不同的接触行为
> 正确做法：对每个参数做敏感度分析——固定其他参数，只扫描该参数的范围，观察策略性能的变化曲线。对高敏感参数用更细的范围和更密的采样

### 练习 97.2

**[A] ⭐⭐** 实现上述消融验证框架，在 IsaacGym 的 Anymal 环境中逐类关闭 DR，绘制消融结果柱状图。哪类随机化对 sim-to-real 最关键？

**[B] ⭐⭐⭐** 在 PPO 训练中实现 CVaR 目标：将每个 batch 中的 episode 按总回报排序，只用最差 20% 的 episode 计算 policy gradient。对比标准 DR 和 CVaR-DR 的训练曲线和 sim-to-sim 迁移性能。

**[C] ⭐⭐⭐⭐** （推导题）从 $\chi^2$ 散度约束的 DRO 对偶出发，证明当不确定性集合为 $\{p : D_{\chi^2}(p \| p_0) \leq \delta\}$ 时，最差分布的解析形式为 $p^*(\xi) \propto p_0(\xi) \cdot [c - J(\pi, \xi)]^+$，其中 $c$ 是使得分布积分为 1 的常数。提示：用 Lagrange 乘子法。

---

## 97.3 System Identification：让仿真器逼近真实 ⭐⭐⭐

### 动机：DR 的互补方案

上一节我们学到 DR 通过**扩大训练分布**来获得鲁棒性。这引出一个自然的问题：如果我们能直接让仿真器的参数**精确匹配**真实世界，不是更好吗？这就是 System Identification（系统辨识）的思路。

DR 和 SysID 不是互斥的，而是互补的：

| 维度 | DR | SysID | 组合使用 |
|------|-----|-------|---------|
| 目标 | 让策略不依赖精确参数 | 让仿真参数接近真实 | 在精确仿真器上做小范围 DR |
| 数据需求 | 无需真机数据 | 需要真机测量 | 先 SysID 再 DR |
| 结果质量 | 策略偏保守 | 仿真更精确 | 策略既精确又鲁棒 |

> **跨领域类比**：DR 和 SysID 的关系类似于机器学习中的正则化和数据清洗。正则化（DR）让模型对噪声不敏感，数据清洗（SysID）让训练数据更干净。两者结合的效果最好。

### 如果不做 System Identification 会怎样？

纯 DR 方案的一个常见问题是"过度保守"。以四足行走为例，纯 DR 训练的策略往往会：
- 采用更低的步频（减小冲击）
- 保持更低的重心（增大稳定裕度）
- 使用更小的步幅（减少滑倒风险）

这些行为确实增加了鲁棒性，但也限制了机器人的运动能力。如果仿真器已经很接近真实（SysID 做到位），DR 的范围可以缩小，策略就能保留更多性能。

### 经典最小二乘辨识推导

System Identification 的核心数学工具是**最小二乘法**。这里完整推导其在机器人参数辨识中的应用。

#### 刚体动力学的线性参数化

机器人的逆动力学方程可以写成关于惯性参数的**线性形式**：

$$
\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{Y}(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}) \boldsymbol{\phi}
$$

其中：
- $\mathbf{Y} \in \mathbb{R}^{n \times p}$ 是**回归矩阵**（regressor matrix），由关节位置、速度、加速度计算得到
- $\boldsymbol{\phi} \in \mathbb{R}^{p}$ 是**惯性参数向量**，包含每个连杆的 10 个惯性参数（质量 $m_i$、质心位置 $m_i c_{x,i}, m_i c_{y,i}, m_i c_{z,i}$、惯量 $I_{xx,i}, I_{xy,i}, \ldots$）

每个连杆有 10 个参数，$N$ 个连杆共 $p = 10N$ 个参数。

> **为什么要写成线性形式？** 因为这使得参数辨识变成一个标准的线性回归问题，可以用最小二乘的全部理论工具（解析解、置信区间、可辨识性分析）。如果不写成线性形式，就只能用非线性优化（如 Levenberg-Marquardt），会面临局部最优和收敛困难。

#### 从数据到参数：最小二乘推导

采集 $K$ 个时刻的数据 $\{(\mathbf{q}_k, \dot{\mathbf{q}}_k, \ddot{\mathbf{q}}_k, \boldsymbol{\tau}_k)\}_{k=1}^{K}$，堆叠得到：

$$
\underbrace{\begin{pmatrix} \boldsymbol{\tau}_1 \\ \boldsymbol{\tau}_2 \\ \vdots \\ \boldsymbol{\tau}_K \end{pmatrix}}_{\mathbf{b} \in \mathbb{R}^{nK}} = \underbrace{\begin{pmatrix} \mathbf{Y}_1 \\ \mathbf{Y}_2 \\ \vdots \\ \mathbf{Y}_K \end{pmatrix}}_{\mathbf{A} \in \mathbb{R}^{nK \times p}} \boldsymbol{\phi}
$$

最小二乘目标：

$$
\hat{\boldsymbol{\phi}} = \arg\min_{\boldsymbol{\phi}} \| \mathbf{A}\boldsymbol{\phi} - \mathbf{b} \|^2
$$

对目标函数求梯度并令其为零：

$$
\frac{\partial}{\partial \boldsymbol{\phi}} \| \mathbf{A}\boldsymbol{\phi} - \mathbf{b} \|^2 = 2 \mathbf{A}^T(\mathbf{A}\boldsymbol{\phi} - \mathbf{b}) = 0
$$

得到**正规方程**（normal equations）：

$$
\boxed{
\hat{\boldsymbol{\phi}} = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T \mathbf{b}
}
$$

其中 $\mathbf{A}^+ = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T$ 是 $\mathbf{A}$ 的**伪逆**。

> 回顾 01_数学/线性代数：伪逆 $\mathbf{A}^+$ 给出的解 $\hat{\boldsymbol{\phi}}$ 位于 $\mathbf{A}$ 的行空间中——这意味着它是所有满足 $\|\mathbf{A}\boldsymbol{\phi} - \mathbf{b}\|$ 最小的解中，$\|\boldsymbol{\phi}\|$ 最小的那个。这个性质在参数辨识中很重要：它避免了不可辨识参数取极端值。

#### 可辨识性分析

并非所有 $10N$ 个参数都能被辨识出来。只有影响力矩的参数组合才是**可辨识的**。形式化地，如果 $\text{rank}(\mathbf{A}) < p$，则存在参数方向 $\Delta\boldsymbol{\phi} \in \text{null}(\mathbf{A})$ 使得 $\mathbf{A}(\boldsymbol{\phi} + \Delta\boldsymbol{\phi}) = \mathbf{A}\boldsymbol{\phi}$——这些方向上的参数变化不影响力矩，因此无法从力矩数据辨识。

解决方法：
1. 对 $\mathbf{A}$ 做 SVD 分解 $\mathbf{A} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^T$
2. 取 $\boldsymbol{\Sigma}$ 中大于阈值 $\epsilon$ 的奇异值对应的列
3. 得到**最小参数集**（base parameters），只辨识这些参数

```python
import numpy as np
from scipy.linalg import svd

def identify_base_parameters(A, b, tol=1e-6):
    """
    从回归矩阵 A 和力矩向量 b 辨识最小参数集。
    
    参数:
        A: 回归矩阵 (nK, p)
        b: 力矩向量 (nK,)
        tol: 奇异值阈值，低于此值的参数方向不可辨识
    返回:
        phi_base: 可辨识参数的最小二乘解
        rank: 回归矩阵的有效秩（=可辨识参数数量）
        condition_number: 条件数（越大越病态）
    """
    U, sigma, Vt = svd(A, full_matrices=False)
    
    # 确定有效秩
    rank = np.sum(sigma > tol * sigma[0])
    print(f"回归矩阵秩: {rank} / {A.shape[1]} (共 {A.shape[1]} 个参数)")
    print(f"条件数: {sigma[0] / sigma[rank-1]:.2e}")
    
    # 截断 SVD 求解（比直接伪逆更稳定）
    U_r = U[:, :rank]
    sigma_r = sigma[:rank]
    Vt_r = Vt[:rank, :]
    
    # phi = V_r * diag(1/sigma_r) * U_r^T * b
    phi_base = Vt_r.T @ (np.diag(1.0 / sigma_r) @ (U_r.T @ b))
    
    # 残差分析
    residual = np.linalg.norm(A @ phi_base - b) / np.linalg.norm(b)
    print(f"相对残差: {residual:.6f}")
    
    return phi_base, rank, sigma[0] / sigma[rank-1]
```

### 传统辨识 vs 学习辨识

#### 传统方法的局限

经典最小二乘辨识有三个关键假设：
1. **模型结构已知**：动力学方程的形式是确定的（刚体+摩擦）
2. **参数线性**：力矩关于惯性参数是线性的
3. **测量无偏**：力矩和状态测量是无偏的

在实际机器人中，这三个假设都可能被违反：
- 执行器的非线性（迟滞、饱和、温度效应）无法用线性参数化描述
- 齿轮传动的摩擦随温度和速度非线性变化
- 力矩传感器通常不可用（消费级腿足机器人几乎不配力矩传感器）

#### 学习辨识：用数据驱动的方式

现代方法用神经网络替代或补充物理模型：

| 方法 | 原理 | 优点 | 缺点 |
|------|------|------|------|
| **纯物理辨识** | 最小二乘拟合物理参数 | 可解释、外推能力强 | 不能捕捉未建模效应 |
| **纯数据驱动** | 端到端神经网络 $f_\theta(q, \dot{q}) \to \tau$ | 能拟合任意非线性 | 需要大量数据、泛化差 |
| **物理信息神经网络** | 物理模型 + 神经网络残差 | 兼具可解释性和灵活性 | 训练复杂度中等 |
| **PACE (2025)** | 进化优化物理参数 | 只需 20 秒编码器数据 | 不建模执行器非线性 |

PACE 框架特别值得关注：它在真机上用关节阻抗控制采集约 20 秒的空中摆腿数据（不需要力矩传感器！），然后用进化算法（CMA-ES）优化仿真器参数使得仿真轨迹与真机轨迹匹配。在 ANYmal 上实现了 32% 的能量效率提升和零样本迁移。

```python
import numpy as np
from scipy.optimize import minimize

def pace_identify(real_traj, sim_fn, param_bounds, n_iter=500):
    """
    PACE 风格的参数辨识：最小化仿真轨迹与真机轨迹的差异。
    
    参数:
        real_traj: 真机轨迹 {q, dq} 的时间序列，形状 (T, n_dof*2)
        sim_fn: 仿真器前向函数，输入参数向量 phi，输出仿真轨迹
        param_bounds: 参数搜索边界 [(lo, hi), ...]
        n_iter: 优化迭代次数
    """
    def objective(phi):
        sim_traj = sim_fn(phi)
        # 状态匹配损失（位置 + 速度）
        pos_err = np.mean((sim_traj[:, :n_dof] - real_traj[:, :n_dof])**2)
        vel_err = np.mean((sim_traj[:, n_dof:] - real_traj[:, n_dof:])**2)
        return pos_err + 0.1 * vel_err  # 速度误差权重较低（噪声大）
    
    # CMA-ES 优化（实际用 cmaes 库更合适）
    n_dof = real_traj.shape[1] // 2
    x0 = np.array([(lo + hi) / 2 for lo, hi in param_bounds])
    
    result = minimize(objective, x0, method='Nelder-Mead',
                      options={'maxiter': n_iter, 'disp': True})
    
    print(f"最终残差: {result.fun:.6f}")
    return result.x
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：辨识数据中加速度的数值微分不稳定**
> 错误做法：直接对关节位置做有限差分 $\ddot{q} \approx (q_{k+1} - 2q_k + q_{k-1})/\Delta t^2$ 来获得加速度
> 现象：辨识结果高度不稳定，重复实验差异巨大
> 根本原因：有限差分放大高频噪声，$\Delta t$ 越小放大越严重（$1/\Delta t^2$ 因子）
> 正确做法：用低通滤波后的数据做差分，或者用激励轨迹的已知解析形式直接计算加速度
> 自检方法：对比辨识后的仿真预测力矩与真实力矩的时间序列，残差应无系统性偏差

> 💡 **概念误区：认为辨识所有 10N 个参数是必要的**
> 新手想法："12 个连杆共 120 个参数，我需要辨识全部"
> 实际上：大部分参数是不可辨识的（质量矩阵中很多参数以组合形式出现）。对于典型的 12-DOF 四足，可辨识的最小参数集通常只有 40-60 个。辨识不可辨识的参数只会引入噪声
> 正确做法：先做 SVD 秩分析，确定最小参数集

> 🧠 **思维陷阱：先辨识后训练，不做 DR**
> 新手做法："既然仿真器已经通过 SysID 校准了，那 DR 就不需要了吧？"
> 问题：SysID 的结果本身有不确定性（测量噪声、模型结构误差）。在校准后的仿真器上训练，仍然可能过拟合到校准误差
> 正确做法：SysID 确定参数标称值，然后在标称值附近做**小范围 DR**（如 $\pm 10\%$），覆盖辨识误差

### 练习 97.3

**[A] ⭐⭐** 用 Pinocchio 的 `computeJointTorqueRegressor` 函数生成一个 3-DOF 平面机械臂的回归矩阵 $\mathbf{Y}$，验证 $\boldsymbol{\tau} = \mathbf{Y}\boldsymbol{\phi}$ 的正确性。

**[B] ⭐⭐⭐** 对上题的回归矩阵做 SVD 分析。$\text{rank}(\mathbf{A})$ 是多少？哪些参数组合是不可辨识的？给出物理解释。

**[C] ⭐⭐⭐** （跨章综合题）结合 复合/20_浮动基座臂统一动力学 中的质量矩阵分块结构，讨论为什么复合机器人的 SysID 比纯四足更困难。提示：考虑 $\mathbf{M}_{ba}$ 耦合项对辨识的影响。

---

## 97.4 Actuator Network：用神经网络学习执行器非线性 ⭐⭐⭐

### 动机：为什么物理模型不够？

在上一节中我们讨论了参数辨识，但有一类 Gap 是参数辨识无法解决的——**执行器的未建模非线性**。

考虑一个典型的无刷直流电机（BLDC）驱动关节。教科书上的电机模型是：

$$
\tau_{\text{motor}} = K_\tau \cdot i_{\text{motor}}
$$

其中 $K_\tau$ 是力矩常数，$i_{\text{motor}}$ 是电流。看起来很简单——但真实电机有大量教科书不讨论的非线性效应：

| 效应 | 原因 | 表现 | 能否用线性参数辨识？ |
|------|------|------|-------------------|
| 齿轮回程间隙 | 机械传动间隙 | 方向反转时有"死区" | 不能（非连续） |
| 库仑+粘性摩擦 | 传动系统摩擦 | 低速时力矩损失大 | 部分（需已知模型结构） |
| 温度依赖 | 电阻随温升增大 | 连续运行后力矩常数下降 5-15% | 不能（时变） |
| 电流限幅 | 驱动器硬件保护 | 高力矩需求被截断 | 不能（非光滑） |
| 谐波驱动柔顺 | 谐波减速器弹性 | 高频振荡 | 部分（需弹性模型） |

Hwangbo et al. (2019) 的关键洞察是：与其试图建立包含所有非线性效应的解析模型，不如**用神经网络直接从数据中学习**输入-输出关系。

### 如果不建模执行器非线性会怎样？

不建模执行器非线性的后果：
1. **高频抖动**：策略发送的期望力矩与实际输出力矩之间有 10-20% 的偏差，策略通过高增益反馈补偿，导致振荡
2. **温升加速**：策略不知道电机已经因为摩擦损耗而过热，继续发送大力矩请求，导致过热保护触发
3. **方向切换滞后**：齿轮回程间隙导致快速方向切换时的"粘滞"现象，步态不对称

### Actuator Network 的完整建模流程

Actuator Network 的核心思想是训练一个神经网络 $f_\theta$ 来替代物理执行器模型：

$$
\hat{\tau}_{\text{actual}} = f_\theta(\mathbf{x}_{\text{act}})
$$

其中 $\mathbf{x}_{\text{act}}$ 是执行器的输入特征向量。

#### 第一步：确定输入特征

输入特征的选择至关重要。根据电机物理学，影响实际输出力矩的因素包括：

$$
\mathbf{x}_{\text{act}} = \begin{pmatrix}
q_{\text{des}} - q_{\text{actual}} & \text{（位置误差，PD控制器输入）} \\
\dot{q}_{\text{actual}} & \text{（当前关节速度）} \\
\dot{q}_{\text{des}} & \text{（期望关节速度，如果有）} \\
q_{\text{des},t-1}, q_{\text{des},t-2}, \ldots & \text{（历史期望位置，捕捉延迟）}
\end{pmatrix}
$$

Hwangbo et al. (2019) 使用了过去 $H$ 步的期望位置和当前的实际位置/速度，共 $H + 2$ 维输入（每个关节独立）：

$$
\mathbf{x}_{\text{act}}^{(j)} = (q_{\text{des},t}^{(j)}, q_{\text{des},t-1}^{(j)}, \ldots, q_{\text{des},t-H+1}^{(j)}, q_{\text{actual},t}^{(j)}, \dot{q}_{\text{actual},t}^{(j)})
$$

> **为什么需要历史期望位置？** 因为电机的响应不是瞬时的——从接收到指令到实际产生力矩有通信延迟和电气响应时间。通过给网络提供历史指令序列，它可以学习"当前输出力矩是过去多少步指令的函数"，从而隐式学习延迟和带宽限制。

#### 第二步：网络结构设计

Actuator Network 通常使用简单的 MLP（多层感知器），因为：
- 输入维度较低（每关节 $H + 2 \approx 10$ 维）
- 映射关系是时不变的（给定相同输入，输出相同）
- 需要在仿真循环中实时推理（<0.1 ms / 关节）

```python
import torch
import torch.nn as nn

class ActuatorNetwork(nn.Module):
    """
    执行器神经网络模型。
    
    学习从 (历史期望位置, 当前实际位置, 当前实际速度) 到 实际输出力矩 的映射。
    每个关节独立建模（共享权重或独立权重）。
    """
    def __init__(self, history_len: int = 6, hidden_dims: list = [32, 32]):
        super().__init__()
        input_dim = history_len + 2  # H 个历史期望 + 实际位置 + 实际速度
        
        layers = []
        prev_dim = input_dim
        for dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, dim))
            layers.append(nn.ELU())  # ELU 比 ReLU 更平滑，适合力矩建模
            prev_dim = dim
        layers.append(nn.Linear(prev_dim, 1))  # 输出标量力矩
        
        self.mlp = nn.Sequential(*layers)
    
    def forward(self, x):
        """
        参数:
            x: (batch, n_joints, history_len + 2)
        返回:
            tau: (batch, n_joints) 预测的实际力矩
        """
        batch, n_joints, feat = x.shape
        x_flat = x.reshape(batch * n_joints, feat)
        tau_flat = self.mlp(x_flat)
        return tau_flat.reshape(batch, n_joints)
```

#### 第三步：训练数据采集

训练数据需要在**真机**上采集。关键是数据要覆盖足够多的操作条件。

| 采集方式 | 数据内容 | 优点 | 缺点 |
|---------|---------|------|------|
| **随机正弦激励** | 对每个关节施加不同频率的正弦位置指令 | 频率覆盖完整 | 非自然运动，不覆盖接触 |
| **行走数据** | 用初始策略在真机上行走并记录 | 覆盖实际工作范围 | 可能崩溃，数据有限 |
| **空中摆腿** | 机器人悬挂，关节自由摆动 | 安全、可重复 | 不包含负载和接触效应 |

推荐的数据采集方案：

```python
def collect_actuator_data(robot, duration_s=120, dt=0.002):
    """
    采集执行器辨识数据：多频率正弦扫描。
    
    每个关节施加不同频率的正弦位置指令，
    同时记录期望位置、实际位置、实际速度和（如果有）实际力矩。
    """
    n_joints = robot.n_dof
    n_steps = int(duration_s / dt)
    
    # 为每个关节生成不同频率的正弦（避免整数倍频率导致相干）
    freqs = np.linspace(0.5, 5.0, n_joints)  # Hz
    amplitudes = np.deg2rad(np.linspace(10, 30, n_joints))  # rad
    
    data = {
        'q_des_history': [],    # (T, n_joints, H)
        'q_actual': [],         # (T, n_joints)
        'dq_actual': [],        # (T, n_joints)
        'tau_measured': [],     # (T, n_joints) — 如果有力矩传感器
    }
    
    q_des_buffer = np.zeros((6, n_joints))  # 历史缓冲区，H=6
    
    for t in range(n_steps):
        time = t * dt
        # 生成期望位置
        q_des = robot.q_home + amplitudes * np.sin(2 * np.pi * freqs * time)
        
        # 发送指令并读取状态
        robot.send_position_command(q_des)
        q_act, dq_act = robot.read_state()
        tau_meas = robot.read_torque() if robot.has_torque_sensor else None
        
        # 更新缓冲区
        q_des_buffer = np.roll(q_des_buffer, 1, axis=0)
        q_des_buffer[0] = q_des
        
        # 记录
        data['q_des_history'].append(q_des_buffer.copy())
        data['q_actual'].append(q_act)
        data['dq_actual'].append(dq_act)
        if tau_meas is not None:
            data['tau_measured'].append(tau_meas)
    
    return {k: np.array(v) for k, v in data.items()}
```

#### 第四步：训练与集成到仿真器

训练完成后，Actuator Network 替代仿真器中的理想执行器模型：

```
原始仿真器流程：
  策略输出 q_des → 理想PD控制器 → tau = Kp*(q_des - q) + Kd*(0 - dq) → 仿真器

使用 Actuator Network 后：
  策略输出 q_des → ActuatorNetwork(q_des_history, q, dq) → tau_actual → 仿真器
```

### 无力矩传感器的辨识：UAN 方法

很多消费级机器人（如 Unitree Go2）没有力矩传感器。这种情况下，不能直接监督训练 Actuator Network。

Unsupervised Actuator Network（UAN）方法的核心思想是：用**状态转移匹配**代替力矩匹配。具体来说：

1. 在仿真器中执行相同的动作序列
2. 对比仿真和真机的**下一步状态** $(q_{t+1}, \dot{q}_{t+1})$
3. 反向传播状态误差到 Actuator Network 参数

$$
\mathcal{L}_{\text{UAN}} = \sum_{t} \| (q_{t+1}^{\text{sim}}, \dot{q}_{t+1}^{\text{sim}}) - (q_{t+1}^{\text{real}}, \dot{q}_{t+1}^{\text{real}}) \|^2
$$

这需要仿真器是**可微的**——梯度需要从状态误差穿过仿真器的积分步到达 Actuator Network 的参数。这正是可微仿真（如 MuJoCo MJX、Brax、Newton）的用武之地。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：Actuator Network 的输入顺序与训练时不一致**
> 错误做法：训练时输入是 [q_des_t, q_des_{t-1}, ..., q_act, dq_act]，部署时误写为 [q_act, dq_act, q_des_t, ...]
> 现象：仿真器中策略仍然不稳定，与不用 Actuator Network 没有区别
> 根本原因：神经网络对输入顺序敏感，错误的顺序等于完全错误的输入
> 正确做法：用 dataclass 或命名张量严格管理输入特征的顺序

> 💡 **概念误区：认为 Actuator Network 可以替代所有 DR**
> 新手想法："有了精确的执行器模型，就不需要 DR 了"
> 实际上：Actuator Network 只解决 D3（执行器）维度的 Gap。地面摩擦（D2）、传感器噪声（D4）、延迟（D5）仍然需要 DR 来覆盖。而且 Actuator Network 本身也有辨识误差——不同电机个体之间有差异，训练数据之外的工况（极端温度、高负载）下网络可能外推不准

### 练习 97.4

**[A] ⭐⭐** 实现上述 `ActuatorNetwork` 类，用合成数据（理想PD + 随机延迟 + 库仑摩擦）训练它。验证训练后的网络能否准确预测带摩擦的力矩。

**[B] ⭐⭐⭐** 在 MuJoCo 环境中，对比使用理想PD模型和 Actuator Network 训练的策略在"修改后仿真器"（增加摩擦和延迟）中的表现。量化 sim-to-sim 迁移的性能差异。

**[C] ⭐⭐⭐⭐** 讨论 Actuator Network 与 PACE 方法的优劣。在什么条件下你会选择 Actuator Network 而非 PACE？

---

## 97.5 Action History 与延迟建模：状态增广的数学 ⭐⭐

### 动机：看不见的"时间错位"

假设你的控制循环运行在 50 Hz（$\Delta t = 20$ ms）。策略在 $t$ 时刻读取传感器数据 $o_t$，计算动作 $a_t$，发送给执行器。但由于通信延迟（USB/CAN/EtherCAT）和计算延迟，执行器实际在 $t + \delta$ 时刻才开始执行 $a_t$。

| 延迟来源 | 典型值 | 可控？ |
|---------|--------|--------|
| 传感器采样到数据可用 | 1-5 ms | 硬件决定 |
| 策略推理（CPU/GPU） | 0.5-5 ms | 模型大小决定 |
| 通信延迟（USB/CAN） | 0.5-2 ms | 总线决定 |
| 执行器响应延迟 | 5-20 ms | 电机带宽决定 |
| **总延迟** | **7-32 ms** | |

在 50 Hz 控制频率下，$\Delta t = 20$ ms。如果总延迟为 25 ms，则策略基于**上一步甚至更早**的观测做决策——这等于在玩一个"延迟版"的游戏。

### 如果不处理延迟会怎样？

不处理延迟的后果在快速运动中尤为明显。以四足跑步为例：

- 目标步频 3 Hz → 步态周期 333 ms → 摆动相约 167 ms
- 25 ms 的延迟意味着"看到的世界"比"真实的世界"晚了 15% 个步态周期
- 策略认为脚还在空中（基于延迟的观测），但脚实际已经触地
- 触地检测延迟 → 错过支撑相切换时机 → 步态混乱 → 摔倒

### 状态增广：将 POMDP 近似为 MDP

延迟的本质效应是将 MDP 变成了 **POMDP**（部分可观测 MDP）。策略不能直接观测当前状态 $s_t$，只能观测到延迟后的状态 $s_{t-d}$。

标准的处理方法是**状态增广**（state augmentation）：将历史动作纳入状态空间，使得增广后的"状态"包含足够的信息来推断当前真实状态。

#### 数学推导

假设观测延迟为 $d$ 步（$d = 1$ 表示延迟一个控制周期）。原始的 MDP 状态转移为：

$$
s_{t+1} = f(s_t, a_t)
$$

由于延迟，策略在 $t$ 时刻只能看到 $o_t = s_{t-d}$。如果我们定义增广状态：

$$
\tilde{s}_t = (s_{t-d}, a_{t-d}, a_{t-d+1}, \ldots, a_{t-1})
$$

则从 $\tilde{s}_t$ 可以通过连续应用转移函数恢复 $s_t$：

$$
s_{t-d+1} = f(s_{t-d}, a_{t-d}), \quad s_{t-d+2} = f(s_{t-d+1}, a_{t-d+1}), \quad \ldots, \quad s_t = f(s_{t-1}, a_{t-1})
$$

因此 $\tilde{s}_t$ 与 $s_t$ 包含相同的信息量——增广后的系统重新变为**完全可观测**。

> **本质洞察**：状态增广不是"增加特征"，而是将 POMDP 重新变为 MDP。关键在于：动作历史 + 旧状态 = 当前状态的充分统计量。这与 Kalman 滤波的思想一致——过去的状态加上期间的输入可以预测当前状态。

#### 工程实现：Action History Buffer

在实践中不需要精确知道 $d$ 的值——直接维护一个足够长的动作历史缓冲区，让网络自己学习如何使用这些信息：

```python
import numpy as np
from collections import deque

class ActionHistoryBuffer:
    """
    动作历史缓冲区。将过去 H 步的动作拼接到观测中。
    
    在 IsaacGym 风格的训练环境中使用：
    - 每步接收当前观测 obs_t 和上一步动作 a_{t-1}
    - 输出增广观测 [obs_t, a_{t-1}, a_{t-2}, ..., a_{t-H}]
    """
    def __init__(self, history_len: int, action_dim: int, num_envs: int):
        self.H = history_len
        self.action_dim = action_dim
        self.num_envs = num_envs
        
        # 缓冲区：每个环境独立维护
        # 形状：(num_envs, H, action_dim)
        self.buffer = np.zeros((num_envs, history_len, action_dim))
    
    def push(self, action: np.ndarray):
        """记录新动作，弹出最旧的。"""
        self.buffer = np.roll(self.buffer, shift=1, axis=1)
        self.buffer[:, 0, :] = action  # 最新动作放在 index 0
    
    def get_augmented_obs(self, obs: np.ndarray) -> np.ndarray:
        """
        返回增广观测 = [原始观测, 动作历史展平]
        形状：(num_envs, obs_dim + H * action_dim)
        """
        history_flat = self.buffer.reshape(self.num_envs, -1)
        return np.concatenate([obs, history_flat], axis=-1)
    
    def reset(self, env_ids: np.ndarray):
        """环境 reset 时清空对应环境的历史。"""
        self.buffer[env_ids] = 0.0


# 使用示例（IsaacGym 风格训练循环）
history_buf = ActionHistoryBuffer(
    history_len=6,      # 保存最近 6 步动作
    action_dim=12,      # 12 个关节
    num_envs=4096       # 并行环境数
)

# 训练循环中
obs = env.reset()
for step in range(max_steps):
    augmented_obs = history_buf.get_augmented_obs(obs)
    action = policy(augmented_obs)
    obs, reward, done, info = env.step(action)
    history_buf.push(action)
    
    # 处理 done 的环境
    done_ids = np.where(done)[0]
    if len(done_ids) > 0:
        history_buf.reset(done_ids)
```

### 延迟随机化

除了 Action History，另一个重要的技巧是在训练时**随机化延迟本身**。这让策略学会应对不确定的延迟，而不是适配固定延迟：

```python
class DelayRandomization:
    """
    在训练环境中随机化观测和动作延迟。
    
    实现方式：维护一个 FIFO 缓冲区，每步以一定概率"跳过"更新，
    模拟延迟效果。
    """
    def __init__(self, max_obs_delay: int = 2, max_act_delay: int = 1, num_envs: int = 4096):
        self.max_obs_delay = max_obs_delay
        self.max_act_delay = max_act_delay
        
        # 每个环境独立采样延迟值（在 episode 开始时）
        self.obs_delays = np.random.randint(0, max_obs_delay + 1, size=num_envs)
        self.act_delays = np.random.randint(0, max_act_delay + 1, size=num_envs)
        
        # 观测缓冲区
        self.obs_buffer = None  # 延迟初始化
    
    def delay_observation(self, obs: np.ndarray) -> np.ndarray:
        """返回延迟后的观测。"""
        if self.obs_buffer is None:
            buffer_size = self.max_obs_delay + 1
            self.obs_buffer = np.zeros((buffer_size, *obs.shape))
        
        # FIFO 推入
        self.obs_buffer = np.roll(self.obs_buffer, 1, axis=0)
        self.obs_buffer[0] = obs
        
        # 每个环境取对应延迟步的观测
        delayed_obs = np.array([
            self.obs_buffer[d, i] for i, d in enumerate(self.obs_delays)
        ])
        return delayed_obs
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：reset 时忘记清空历史缓冲区**
> 错误做法：环境 reset 后，缓冲区中残留上一个 episode 的动作历史
> 现象：episode 开始的前几步策略行为异常（因为"看到了"不属于当前 episode 的动作）
> 根本原因：跨 episode 的动作历史破坏了 MDP 的 Markov 性质
> 正确做法：在 `env.reset()` 时同步清零缓冲区

> 💡 **概念误区：动作历史长度越长越好**
> 新手想法："延迟最多 2 步，那我存 20 步历史更安全"
> 实际上：过长的历史会增大观测空间维度（$12 \times 20 = 240$ 维），增加策略网络的学习难度，且大部分历史信息是冗余的
> 正确做法：$H$ 设为最大可能延迟的 1.5-2 倍即可。典型值 $H = 3 \sim 6$

### 练习 97.5

**[A] ⭐⭐** 在一个简单的 CartPole 环境中，对比有无 Action History 在人为加入 2 步延迟后的训练效果。绘制 reward 曲线。

**[B] ⭐⭐⭐** 推导：当延迟 $d$ 是**随机变量**（每步独立采样）时，Action History 方法的理论保证是否仍然成立？如果不成立，需要什么额外条件？

---

## 97.6 RMA 在线适应：运行时估计环境参数 ⭐⭐⭐⭐

### 动机：DR 的根本局限

Domain Randomization 的核心假设是：如果训练分布足够广，策略自然能应对真实环境。但这个假设在以下场景中会失败：

1. **真实参数在 DR 范围之外**：例如训练时摩擦范围是 $[0.3, 1.2]$，但真实地面是冰面 $\mu = 0.1$
2. **需要针对性适应**：DR 策略只能给出"对所有环境都还行"的通用行为，无法针对当前特定环境给出最优行为
3. **环境动态变化**：机器人从室内走到室外，从硬地走到草地，环境参数在 episode 内变化

RMA（Rapid Motor Adaptation, Kumar et al. 2021）的核心思想是：**不要让一个策略通用所有环境，而是让策略能在几秒钟内适应当前环境**。

> **跨领域类比**：DR 策略像一个"全能但平庸"的运动员——在所有场地上都能跑，但在任何一个场地上都不是最快的。RMA 策略像一个"能快速适应的职业选手"——到达新场地后花 30 秒热身，然后以接近最优的水平比赛。

### 如果不做在线适应会怎样？

考虑一个真实场景：机器人需要在工厂车间巡检。车间的不同区域有不同的地面材质：
- 钢板区域：$\mu \approx 0.4$
- 橡胶垫区域：$\mu \approx 0.9$  
- 油渍区域：$\mu \approx 0.15$

纯 DR 策略会采用一种"全场通用"的保守步态——步幅小、重心低、速度慢。在橡胶垫上浪费了摩擦裕度（本可以更快），在油渍上可能仍然不够保守（仍然打滑）。

RMA 策略则会在进入每个新区域后的几步内"感知"到地面变化，自动调整步态参数。

### RMA 的三阶段训练架构

RMA 的训练分为三个阶段，每个阶段的角色和训练方式不同：

```
阶段 1：训练教师策略（使用特权信息）
          ┌──────────────────────────┐
    e_t → │    环境编码器 μ(e_t)      │ → z_t（环境编码）
          └──────────────────────────┘
                    │
    o_t ──→ ┌──────┴──────┐
            │  基础策略 π  │ → a_t
    z_t ──→ │ (o_t, z_t)  │
            └─────────────┘

阶段 2：训练适应模块（学习从历史推断 z_t）
                    ┌──────────────────────┐
    o_{t-H:t} ──→   │  适应模块 φ(o 历史)    │ → ẑ_t ≈ z_t
                    └──────────────────────┘

阶段 3：部署（只用可部署传感器）
    o_{t-H:t} → 适应模块 → ẑ_t ─┐
                                │
    o_t ────────────────────→ 基础策略 → a_t
```

#### 阶段 1：教师策略训练

教师策略同时接收：
- 常规观测 $o_t$（关节角、角速度、IMU 等可部署传感器信息）
- 环境参数编码 $z_t = \mu(e_t)$（地面摩擦、质量、地形高度图等**特权信息**）

环境编码器 $\mu$ 将高维环境参数 $e_t$（可能是几十个标量参数或高维地形图）压缩为一个低维向量 $z_t \in \mathbb{R}^{d_z}$（典型 $d_z = 8 \sim 32$）。

教师策略和环境编码器一起用 PPO 端到端训练：

$$
\pi_{\text{teacher}}(a_t | o_t, z_t), \quad z_t = \mu(e_t)
$$

$$
\max_{\pi, \mu} \mathbb{E}_{\xi \sim p(\xi)} \left[ \sum_t \gamma^t r(s_t, a_t) \right]
$$

> **为什么需要编码器而不是直接拼接 $e_t$？** 因为环境参数 $e_t$ 可能是高维的（例如 $100 \times 100$ 的高度图），且不同参数之间有冗余（例如质量增大和摩擦减小可能需要类似的步态调整）。编码器将高维参数压缩为策略真正需要的"行为相关"低维表征。

#### 阶段 2：适应模块训练

教师训练好后，冻结教师策略和编码器。训练一个**适应模块** $\hat{z}_t = \phi(o_{t-H}, o_{t-H+1}, \ldots, o_t)$，使其从**只有可部署传感器的观测历史**中推断出环境编码 $z_t$。

适应模块的损失函数：

$$
\mathcal{L}_{\text{adapt}} = \mathbb{E} \left[ \| \phi(o_{t-H:t}) - \mu(e_t) \|^2 \right]
$$

为什么这是可能的？因为观测历史中**隐含了环境信息**。例如：
- 如果地面很滑，关节速度的波动模式与正常地面不同
- 如果质量增加，同样的力矩下加速度更小
- 如果有延迟，动作和状态变化之间的时间关系会移位

适应模块的典型结构是一个 **1D 卷积网络**或 **GRU**，因为它需要处理时间序列输入：

```python
import torch
import torch.nn as nn

class AdaptationModule(nn.Module):
    """
    RMA 适应模块：从观测历史推断环境编码。
    使用 1D 时间卷积处理变长历史。
    """
    def __init__(self, obs_dim: int, latent_dim: int, history_len: int = 50):
        super().__init__()
        self.history_len = history_len
        
        # 1D 卷积处理时间序列
        # 输入：(batch, obs_dim, history_len) — channels-first
        self.temporal_encoder = nn.Sequential(
            nn.Conv1d(obs_dim, 64, kernel_size=8, stride=4, padding=2),
            nn.ELU(),
            nn.Conv1d(64, 32, kernel_size=5, stride=2, padding=1),
            nn.ELU(),
            nn.Flatten(),
        )
        
        # 计算卷积输出维度（根据 history_len 自动推导）
        dummy = torch.zeros(1, obs_dim, history_len)
        conv_out_dim = self.temporal_encoder(dummy).shape[1]
        
        # MLP 将卷积特征映射为环境编码
        self.projector = nn.Sequential(
            nn.Linear(conv_out_dim, 64),
            nn.ELU(),
            nn.Linear(64, latent_dim),
        )
    
    def forward(self, obs_history: torch.Tensor) -> torch.Tensor:
        """
        参数:
            obs_history: (batch, history_len, obs_dim)
        返回:
            z_hat: (batch, latent_dim) — 估计的环境编码
        """
        # 转为 (batch, obs_dim, history_len) 给 Conv1d
        x = obs_history.permute(0, 2, 1)
        features = self.temporal_encoder(x)
        z_hat = self.projector(features)
        return z_hat
```

#### 阶段 3：部署

部署时只用基础策略 + 适应模块，不需要特权信息：

$$
a_t = \pi_{\text{teacher}}(o_t, \hat{z}_t), \quad \hat{z}_t = \phi(o_{t-H:t})
$$

适应模块在运行时持续更新 $\hat{z}_t$——每接收一个新观测，就用最近 $H$ 步的观测重新推断环境编码。这实现了**亚秒级的在线适应**，无需任何在线训练或梯度计算。

### 环境编码的可解释性

一个自然的问题是：$z_t$ 编码了什么？是否与物理参数有对应关系？

实验表明：$z_t$ 的不同维度确实会与特定物理参数**高度相关**，尽管它们不是通过显式监督学习的。例如 Kumar et al. (2021) 发现：
- $z_t$ 的某些维度与摩擦系数强相关（$|r| > 0.8$）
- 某些维度与附加质量强相关
- 某些维度编码了多个参数的组合效应

这意味着 RMA 不仅是一个黑盒适应器——它实际上在做**隐式系统辨识**，但在一个比物理参数更紧凑、更行为相关的空间中。

> **本质洞察**：RMA 的环境编码器不是在学"环境是什么样"（物理参数），而是在学"在这个环境中应该怎么走"（行为相关表征）。这比显式系统辨识更高效——两个物理上不同但行为上等价的环境会被编码到相近的 $z_t$。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：适应模块训练时用的观测与部署时不一致**
> 错误做法：阶段 2 训练适应模块时用的是无噪声观测，但部署时的观测有传感器噪声
> 现象：适应模块在仿真中完美推断 $z_t$，但在真机上完全失效
> 根本原因：分布偏移——训练数据和部署数据的分布不同
> 正确做法：阶段 2 训练时必须对观测施加与 DR 中相同的传感器噪声

> 💡 **概念误区：认为 RMA 不需要 DR**
> 新手想法："RMA 可以在线适应，那 DR 就不需要了"
> 实际上：RMA 的适应范围取决于教师策略训练时的参数范围。如果教师训练时没有 DR，它只学会了在标称参数下工作，适应模块也无法推断出有意义的环境编码。DR 为 RMA 提供了训练数据的多样性基础
> 正确关系：DR 提供训练多样性 → 教师学会在多种环境中工作 → 编码器学会区分不同环境 → 适应模块学会从观测推断环境

> 🧠 **思维陷阱：编码维度 $d_z$ 越大越好**
> 新手想法："环境参数有 20 个，那 $d_z$ 设 64 肯定够用"
> 问题：过高的 $d_z$ 让适应模块的回归任务变得更难，且容易过拟合到训练分布
> 正确做法：$d_z$ 通常设为 $8 \sim 16$。这远小于物理参数的维度，因为编码器会自动发现参数之间的冗余和组合关系

### 练习 97.6

**[A] ⭐⭐⭐** 在 IsaacGym 中实现 RMA 的完整三阶段训练流程。用 Anymal 环境，随机化质量和摩擦系数。对比 RMA 和纯 DR 在 sim-to-sim 迁移（训练参数范围之外）的表现。

**[B] ⭐⭐⭐** 训练完 RMA 后，将 $z_t$ 的每个维度与各物理参数做散点图和相关系数分析。验证是否存在可解释的对应关系。

**[C] ⭐⭐⭐⭐** （推导题）证明：当环境编码器 $\mu$ 是双射函数时，适应模块的最优解满足 $\phi^* = \mu \circ g$，其中 $g$ 是从观测历史到环境参数的后验估计。讨论这意味着什么。

---

## 97.7 Delta-Action / 残差策略：在基线上做修正 ⭐⭐⭐

### 动机：利用少量真机数据高效修正

前面的方法都试图在仿真中解决 Sim-to-Real 问题——要么通过 DR 增加鲁棒性，要么通过 SysID 提高仿真精度，要么通过 RMA 在线适应。但有一个更直接的思路：**在仿真中训练一个基线策略，然后用少量真机数据学习一个"修正项"**。

这就是 Delta-Action（残差策略）的核心思想。ASAP (He et al. 2025) 是这个方向最新的代表性工作。

> 回顾 复合/240_ASAP_SimToReal：我们在那里详细讨论了 ASAP 的两阶段流程。本节将 Delta-Action 从 ASAP 的具体实例抽象为通用方法论，给出更完整的数学分析。

### 如果直接在真机上微调策略会怎样？

一个自然的想法是：用 PPO 在真机上继续训练（fine-tuning）。但这有严重的实际困难：

1. **数据效率极低**：PPO 是 on-policy 算法，每次梯度更新需要大量 rollout 数据。真机 rollout 的速度是仿真的 $10^{-4}$（无法并行），且失败可能损坏硬件
2. **安全风险**：微调过程中策略可能探索到危险动作
3. **信用分配困难**：策略的哪一部分需要修改？哪些行为已经是好的？全参数微调无法利用已有的好行为

### 残差策略的数学形式化

Delta-Action 的核心思想是将策略分解为两部分：

$$
\boxed{
a_t = \underbrace{\pi_{\text{base}}(o_t)}_{\text{基线策略（仿真中训练，冻结）}} + \underbrace{\delta_\theta(o_t)}_{\text{残差项（用真机数据学习）}}
}
$$

其中 $\pi_{\text{base}}$ 是在仿真中用 DR 训练好的策略（参数冻结），$\delta_\theta$ 是一个需要学习的残差网络。

#### 与直接微调的数学对比

**直接微调**（修改整个策略参数 $\theta_{\text{all}}$）：

$$
\pi_{\theta_{\text{all}}}(o_t) = f_{\theta_{\text{all}}}(o_t)
$$

参数空间维度等于整个策略网络的参数数量（可能数十万）。在少量真机数据下，优化这么高维的参数空间极易过拟合。

**残差策略**（只学习残差 $\delta_\theta$）：

$$
a_t = \underbrace{f_{\theta_{\text{frozen}}}(o_t)}_{\text{已知好的基线}} + \underbrace{\delta_\theta(o_t)}_{\text{小修正}}
$$

优势：
1. **初始化良好**：$\delta_\theta = 0$ 时就退化为基线策略，保证起始行为安全
2. **参数空间小**：残差网络可以比基线策略小得多（因为只需学习修正）
3. **正则化自然**：对 $\|\delta_\theta(o_t)\|$ 施加惩罚，限制修正幅度

#### 为什么修正 Action 而不是修正 Dynamics？

一个替代方案是学习动力学残差 $\hat{s}_{t+1} = f_{\text{sim}}(s_t, a_t) + \Delta_\theta(s_t, a_t)$。ASAP 论文详细分析了这两种方案的区别：

| 方面 | Delta-Action（修正动作） | Delta-Dynamics（修正动力学） |
|------|------------------------|---------------------------|
| 修改对象 | 策略输出 | 仿真器状态转移 |
| 训练方式 | 可用 PPO 在修正后的仿真器中训练 | 需要可微仿真器做反向传播 |
| 梯度传播 | 只通过策略网络 | 需要穿过仿真器的积分步 |
| 接触处理 | 不需要接触微分 | 接触的不可微性是主要障碍 |
| 工程复杂度 | 低（只改策略输出） | 高（需要修改仿真器内核） |

Delta-Action 的工程优势非常明显：它只需要在策略输出上加一个小网络，完全不需要修改仿真器。

### ASAP 的两阶段流程

ASAP 的具体实现将 Delta-Action 方法分为两个阶段：

**阶段 1：仿真训练基线策略**

用标准 DR + PPO 在 IsaacGym 中训练基线策略 $\pi_{\text{base}}$。这一步与常规 RL 训练完全一致。

**阶段 2：真机 Rollout + 残差学习**

1. 在真机上用 $\pi_{\text{base}}$ 执行 rollout，记录 $(o_t, a_t^{\text{base}}, o_{t+1}^{\text{real}})$
2. 在仿真中用相同的 $a_t^{\text{base}}$ 前推，得到 $o_{t+1}^{\text{sim}}$
3. 计算状态偏差 $\Delta o_{t+1} = o_{t+1}^{\text{real}} - o_{t+1}^{\text{sim}}$
4. 训练残差网络 $\delta_\theta$ 使得：在仿真中执行 $a_t = \pi_{\text{base}}(o_t) + \delta_\theta(o_t)$ 后，仿真状态更接近真机状态

$$
\mathcal{L}_{\text{ASAP}} = \mathbb{E} \left[ \sum_t \| o_{t+1}^{\text{sim}}(\pi_{\text{base}} + \delta_\theta) - o_{t+1}^{\text{real}} \|^2 \right]
$$

这个损失函数的巧妙之处在于：它在**仿真器中**优化 $\delta_\theta$，但目标是匹配**真机**的状态转移。因此可以利用仿真器的并行性和安全性，同时逼近真机行为。

```python
import torch
import torch.nn as nn

class DeltaActionModel(nn.Module):
    """
    Delta-Action 残差模型。
    
    输出对基线策略动作的修正量。
    架构比基线策略更小（因为只学修正）。
    """
    def __init__(self, obs_dim: int, action_dim: int, max_delta: float = 0.1):
        super().__init__()
        self.max_delta = max_delta  # 限制最大修正幅度（弧度）
        
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.ELU(),
            nn.Linear(64, 32),
            nn.ELU(),
            nn.Linear(32, action_dim),
            nn.Tanh(),  # 输出范围 [-1, 1]
        )
    
    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """返回动作修正量，幅度被 max_delta 限制。"""
        raw = self.net(obs)
        return raw * self.max_delta  # 缩放到 [-max_delta, max_delta]

# 部署时的组合
def deploy_with_delta(obs, base_policy, delta_model):
    """
    组合基线策略和残差模型。
    base_policy 参数冻结，只有 delta_model 在训练中更新。
    """
    with torch.no_grad():
        a_base = base_policy(obs)        # 基线动作
    delta = delta_model(obs)              # 残差修正
    return a_base + delta                 # 组合动作
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：残差模型的输出范围未限制**
> 错误做法：残差网络最后一层用 ReLU 或无激活函数
> 现象：残差模型输出可能达到 $\pm 1$ rad 甚至更大，完全覆盖基线策略的贡献
> 根本原因：没有约束残差的幅度，"修正"变成了"替代"
> 正确做法：最后一层用 Tanh + 缩放因子（如 0.1 rad），确保修正幅度远小于基线动作幅度
> 自检方法：打印 $\|\delta_\theta\| / \|\pi_{\text{base}}\|$ 的比值，应保持在 $< 0.3$ 以下

> 💡 **概念误区：认为残差策略只能用于 sim-to-real**
> 实际上：残差策略是一种通用的迁移学习框架。它也可以用于 sim-to-sim 迁移（例如从 MuJoCo 到 IsaacGym）或 task-to-task 迁移（例如从平地行走到上坡行走）

### 练习 97.7

**[A] ⭐⭐⭐** 在 MuJoCo 中实现 Delta-Action 的 sim-to-sim 实验：在 MuJoCo-v4 中训练基线策略，然后在修改了摩擦和质量的 MuJoCo-v4' 中用 Delta-Action 修正。对比直接迁移 vs Delta-Action 修正的性能。

**[B] ⭐⭐⭐** 实验 $\text{max\_delta}$ 参数对性能的影响：分别设为 0.01, 0.05, 0.1, 0.2, 0.5 rad，绘制迁移性能曲线。是否存在最优值？

**[C] ⭐⭐⭐⭐** （跨章综合题）结合 复合/240_ASAP_SimToReal 中 ASAP 的具体实现，讨论为什么 ASAP 在人形机器人（G1）上用 Delta-Action 而不用 Delta-Dynamics。关键因素是什么？

---

## 97.8 Teacher-Student 蒸馏：从特权信息到可部署策略 ⭐⭐⭐

### 动机：特权信息在部署时不可用

在 97.6 节中，RMA 的教师策略使用了特权信息（环境参数 $e_t$）。更一般地，很多 RL 训练技巧都依赖仿真中才有的信息：

| 特权信息 | 仿真中 | 真机上 |
|---------|--------|--------|
| 地面摩擦系数 | 直接读取 | 不可知 |
| 精确接触力 | 仿真器输出 | 需要力传感器（精度有限） |
| 地形高度图 | 渲染引擎提供 | 需要深度相机+处理 |
| 对方机器人的精确位置 | 直接读取 | 需要视觉识别 |
| 未来参考轨迹 | 预生成 | 只知道当前目标 |

Teacher-Student 蒸馏（Knowledge Distillation for Sim-to-Real）的核心思想是：**先用特权信息训练一个强大的教师策略，然后训练一个只用可部署传感器的学生策略来模仿教师的行为**。

### 如果直接用受限观测训练策略会怎样？

为什么不直接用可部署传感器训练 RL 策略？因为受限观测下的 RL 训练面临严重的**部分可观测性**问题：

- 没有摩擦系数信息 → 策略需要从足底力反馈间接推断 → 信号延迟且含噪
- 没有地形高度图 → 策略只能依赖本体感觉"试探" → 探索效率极低
- 没有精确接触信息 → 无法准确判断支撑状态 → 步态切换不可靠

直接在 POMDP 中训练可能收敛极慢（数十倍 wall time），甚至收敛到次优策略。

Teacher-Student 框架解耦了这两个挑战：教师在 MDP 中解决任务（容易），学生从教师的行为中学习如何在 POMDP 中近似最优行为（相对容易）。

### 蒸馏损失函数推导

学生策略 $\pi_S(a | o_t)$ 的训练目标是模仿教师策略 $\pi_T(a | o_t, e_t)$ 的行为。最直接的方式是**行为克隆**（Behavior Cloning）：

$$
\mathcal{L}_{\text{BC}} = \mathbb{E}_{o_t, e_t \sim \mathcal{D}} \left[ \| \pi_S(o_t) - \pi_T(o_t, e_t) \|^2 \right]
$$

其中 $\mathcal{D}$ 是教师策略在仿真中产生的轨迹数据。

但纯行为克隆有**分布偏移**问题（DAgger, Ross et al. 2011）：学生犯了错误后进入教师从未访问过的状态，然后错误累积。解决方案是**在线蒸馏**：

$$
\text{for each step } t: \quad a_t^S = \pi_S(o_t), \quad a_t^T = \pi_T(o_t, e_t)
$$
$$
\mathcal{L}_{\text{online}} = \| a_t^S - a_t^T \|^2
$$

学生策略执行自己的动作 $a_t^S$（产生自己的状态分布），但用教师在相同状态下的动作 $a_t^T$ 作为监督信号。这消除了分布偏移——学生在自己的状态分布上受到训练。

更进一步，可以将行为克隆与 RL 目标**混合**：

$$
\mathcal{L}_{\text{student}} = \underbrace{-\lambda_{\text{RL}} J(\pi_S)}_{\text{RL 目标（最大化回报）}} + \underbrace{\lambda_{\text{BC}} \| \pi_S(o_t) - \pi_T(o_t, e_t) \|^2}_{\text{蒸馏目标（模仿教师）}}
$$

$\lambda_{\text{RL}}$ 和 $\lambda_{\text{BC}}$ 的比例控制学生在"自主探索"和"模仿教师"之间的平衡。典型做法是开始时 $\lambda_{\text{BC}}$ 大（主要模仿），逐渐衰减让学生有更多自主性。

### 实现：在 PPO 中加入蒸馏损失

```python
import torch
import torch.nn as nn

class TeacherStudentTrainer:
    """
    Teacher-Student 蒸馏训练器。
    
    在 PPO 训练循环中增加蒸馏损失项。
    """
    def __init__(self, student_policy, teacher_policy, 
                 bc_coeff=1.0, bc_decay=0.999):
        self.student = student_policy
        self.teacher = teacher_policy
        self.teacher.eval()  # 教师参数冻结
        
        self.bc_coeff = bc_coeff
        self.bc_decay = bc_decay
    
    def compute_loss(self, obs, privileged_obs, actions, advantages,
                     old_log_probs, clip_ratio=0.2):
        """
        组合 PPO 损失和蒸馏损失。
        
        参数:
            obs: 可部署观测（学生输入）
            privileged_obs: 特权观测（教师额外输入）
            actions: 学生采取的动作
            advantages: GAE 估计的优势函数
            old_log_probs: 旧策略的动作对数概率
            clip_ratio: PPO 裁剪参数
        """
        # --- PPO 损失 ---
        new_log_probs = self.student.log_prob(obs, actions)
        ratio = torch.exp(new_log_probs - old_log_probs)
        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1 - clip_ratio, 1 + clip_ratio) * advantages
        ppo_loss = -torch.min(surr1, surr2).mean()
        
        # --- 蒸馏损失 ---
        with torch.no_grad():
            teacher_actions = self.teacher.act(
                torch.cat([obs, privileged_obs], dim=-1)
            )
        student_actions = self.student.act(obs)
        bc_loss = nn.functional.mse_loss(student_actions, teacher_actions)
        
        # --- 组合 ---
        total_loss = ppo_loss + self.bc_coeff * bc_loss
        
        # 衰减蒸馏系数
        self.bc_coeff *= self.bc_decay
        
        return total_loss, {
            'ppo_loss': ppo_loss.item(),
            'bc_loss': bc_loss.item(),
            'bc_coeff': self.bc_coeff,
        }
```

### 与 RMA 的关系

Teacher-Student 蒸馏和 RMA 有紧密的联系，但也有关键区别：

| 方面 | Teacher-Student 蒸馏 | RMA |
|------|---------------------|-----|
| 教师的角色 | 提供行为监督信号 | 提供环境编码监督 |
| 学生学什么 | 模仿教师的**动作** | 预测教师的**环境编码** |
| 信息压缩方式 | 行为空间（动作维度） | 编码空间（$d_z$ 维度） |
| 适应能力 | 固定策略，不在线适应 | 在线估计环境编码，持续适应 |
| 适用场景 | 环境变化较小，策略一次训练长期使用 | 环境动态变化，需要持续适应 |

> **本质洞察**：Teacher-Student 蒸馏将"在有完整信息时怎么做"压缩为"在信息不完整时怎么近似"。RMA 更进一步——不只是近似教师的行为，而是恢复教师使用的信息（环境编码），然后让同一个基础策略做出相应调整。RMA 可以被视为"更智能的蒸馏"，因为它保留了适应性。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：教师和学生的观测空间部分重叠导致信息泄露**
> 错误做法：学生的输入中不小心包含了某些特权信息（如精确接触状态标志位）
> 现象：学生在仿真中完美模仿教师，但部署时特权通道为零或噪声，行为完全不同
> 根本原因：学生过度依赖了本应不可用的信息
> 正确做法：严格检查学生的输入维度列表，确保每个维度都对应真机上可获得的传感器

> 💡 **概念误区：认为蒸馏后学生一定不如教师**
> 新手想法："学生只有部分信息，性能一定差于教师"
> 实际上：在某些情况下，学生可能**超越**教师。原因有二：(1) 学生的混合 RL+BC 目标让它可以发现教师没探索到的好行为；(2) 教师可能在特权信息上过拟合（例如过度依赖精确摩擦值），学生被迫学习更鲁棒的替代策略

### 练习 97.8

**[A] ⭐⭐⭐** 在 IsaacGym 中训练一个使用地形高度图的教师策略，然后蒸馏为只用本体感觉的学生策略。对比两者在随机地形上的成功率。

**[B] ⭐⭐⭐** 实验蒸馏系数 $\lambda_{\text{BC}}$ 的衰减策略：(a) 固定不衰减，(b) 线性衰减到 0，(c) 指数衰减到 0.01。哪种策略产生最好的学生？

---

## 97.9 Physics Parameter Adaptation：在线参数估计 ⭐⭐⭐⭐

### 动机：将 RMA 的隐式辨识变为显式

RMA 在一个隐式的编码空间中做在线"系统辨识"。但有时我们需要**显式的参数估计**——例如当需要在 MPC 中使用估计的质量参数，或者当需要向用户报告"当前地面摩擦系数约为 0.3"。

Physics Parameter Adaptation 直接估计物理参数的值，而不仅仅是行为相关的编码。

### 在线参数估计的数学框架

将环境参数视为需要估计的隐变量。给定观测历史 $o_{1:t}$ 和动作历史 $a_{1:t-1}$，参数的后验分布为：

$$
p(\xi | o_{1:t}, a_{1:t-1}) \propto \underbrace{p(\xi)}_{\text{先验}} \prod_{k=1}^{t-1} \underbrace{p(o_{k+1} | o_k, a_k, \xi)}_{\text{似然（由物理模型决定）}}
$$

精确计算这个后验通常是 intractable 的（需要对高维参数空间积分）。实际方法分为两类：

| 方法 | 原理 | 优点 | 缺点 |
|------|------|------|------|
| **扩展 Kalman 滤波（EKF）** | 将参数视为"状态"的一部分，用 EKF 联合估计 | 有理论保证，计算量可控 | 需要线性化，对强非线性系统不准确 |
| **粒子滤波** | 用加权粒子表示后验分布 | 能处理任意非线性 | 计算量随参数维度指数增长 |
| **神经网络估计器** | 训练网络从观测历史直接输出参数 | 推理快、可 GPU 并行 | 需要大量训练数据，泛化性取决于训练分布 |

在 Sim-to-Real 场景中，**神经网络估计器**（与 RMA 的适应模块类似，但监督信号是物理参数而非行为编码）是最常用的选择：

$$
\hat{\xi}_t = g_\psi(o_{t-H:t})
$$

$$
\mathcal{L}_{\text{param}} = \mathbb{E} \left[ \| g_\psi(o_{t-H:t}) - \xi_{\text{true}} \|^2 \right]
$$

### 与 RMA 的对比

| 方面 | RMA 编码器 | Physics Parameter Estimator |
|------|-----------|---------------------------|
| 监督信号 | 行为编码 $z = \mu(e)$ | 物理参数 $\xi$ 本身 |
| 编码维度 | 低维（$d_z \sim 8$） | 与参数维度相同 |
| 可解释性 | 需要后验分析 | 直接物理含义 |
| 可否用于其他模块 | 只能用于对应的基础策略 | 可用于 MPC、安全约束等 |
| 估计精度 | 行为等价即可（不需精确） | 需要物理精确 |

> **不是X而是Y**：Physics Parameter Adaptation 不是 RMA 的替代品，而是互补工具。当你只需要策略鲁棒性时用 RMA（编码空间更紧凑、学习更容易）；当你需要物理参数的精确值时用参数估计（例如调整 MPC 的模型参数或触发安全约束）。

### ⚠️ 常见陷阱

> 💡 **概念误区：认为在线参数估计能估计所有参数**
> 新手想法："我有 20 个物理参数，训练一个网络把它们全估出来"
> 实际上：与线性辨识中的可辨识性分析类似，并非所有参数都能从观测中区分。例如，质量增大 10% 和摩擦增大 10% 可能产生非常相似的观测（步态都变慢了），网络无法将它们区分开
> 正确做法：先做可辨识性分析——在仿真中逐个改变参数，检查观测的变化是否可区分

### 练习 97.9

**[A] ⭐⭐⭐** 训练一个从观测历史预测摩擦系数的神经网络。在不同摩擦系数的环境中测试其准确性。

**[B] ⭐⭐⭐⭐** 对比 RMA 编码空间和显式参数估计在以下场景中的表现：(a) 环境参数在 episode 内变化（如从硬地走到草地），(b) 参数在训练分布之外。

---

## 97.10 综合比较与选型指南 ⭐⭐

### 方法选择决策树

面对一个新的 Sim-to-Real 问题，如何选择方法？以下决策流程图基于 Gap 的类型和可用资源：

```
                      开始
                        │
            ┌─────────── 有真机数据？ ────────────┐
            │                                    │
           否                                   是
            │                                    │
      ┌─── Gap 主要来自 ───┐          ┌── 数据量多少？ ──┐
      │                    │          │                  │
    参数不确定         执行器非线性    少量(<5min)      大量(>30min)
      │                    │          │                  │
    DR + RMA         DR + 观测噪声    Delta-Action   ActNet + SysID
      │                    │          + DR             + 小范围DR
      │                    │          │                  │
      └────── 性能够？ ─────┘          └──── 性能够？ ────┘
              │                               │
             是 → 完成                        是 → 完成
              │                               │
             否 → 加入 Teacher-Student         否 → 加入 RMA
                   蒸馏处理信息差                    + 在线适应
```

### 九种方法的完整对比

| 方法 | 实现难度 | 真机数据需求 | 针对 Gap 维度 | 最佳适用场景 | 典型组合搭档 |
|------|---------|------------|-------------|------------|------------|
| DR | 低 | 无 | D1,D2,D4,D5 | 参数不确定但范围已知 | 所有方法 |
| SysID | 中 | 中（离线采集） | D1,D2,D3 | 有力矩传感器的精密机器人 | DR |
| Actuator Net | 中 | 中（专项采集） | D3 | 执行器非线性严重 | DR, SysID |
| Action History | 低 | 无 | D5 | 延迟不确定 | DR |
| RMA | 高 | 无（部署时自适应） | D1-D4 | 环境动态变化 | DR, T-S |
| Delta-Action | 中 | 少（<5min rollout） | D1-D3 | 快速修正已有策略 | DR |
| Teacher-Student | 中 | 无 | 信息差 | 部署传感器受限 | DR, RMA |
| Param Adaptation | 中 | 无 | D1,D2 | 需显式参数估计 | MPC |
| 可微仿真 | 高 | 中 | D1-D3 | 梯度信息可用 | SysID |

### 不同平台的推荐方案

| 平台类型 | 典型 Gap | 推荐方案 | 理由 |
|---------|---------|---------|------|
| **四足（Go2 级）** | D3（电机）+ D1（质量） | DR + ActNet + Action History | 执行器非线性是主要瓶颈，无力矩传感器 |
| **人形（G1 级）** | D3 + D5 + D1 | DR + RMA + Delta-Action | 高维空间需要适应，接触模式复杂 |
| **四足+臂（复合）** | D1（耦合惯量）+ D3 | DR + SysID + RMA | 耦合效应需要精确参数 |
| **灵巧手** | D2（接触）+ D4（触觉） | 大规模 DR + Teacher-Student | 接触物理是最大瓶颈 |
| **无人机** | D1（空气动力学）+ D5 | DR + SysID | 空气动力学可精确辨识 |

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：用最新的方法就不需要基础方法**
> 新手想法："RMA + Delta-Action 这么强大，DR 就不用了吧？"
> 实际上：几乎所有成功的 Sim-to-Real 系统都以 DR 为基础。RMA 需要 DR 提供训练多样性，Delta-Action 需要 DR 训练的基线策略。DR 是地基，其他方法是上层建筑
> 经验法则：DR 覆盖 70% 的 Gap，其他方法各覆盖特定维度剩余的 30%

### 练习 97.10

**[A] ⭐⭐** 为以下三个场景设计 Sim-to-Real 方案，说明选择每种方法的理由：
  - (a) Unitree Go2 在室外草地上行走
  - (b) Franka Emika 灵巧操作任务
  - (c) 多旋翼无人机在强风中飞行

**[B] ⭐⭐⭐** （跨章综合题）结合 复合/20_浮动基座臂统一动力学 和 复合/110_轮足SimToReal与硬件，设计一个完整的 Go2+Z1 复合机器人的 Sim-to-Real 方案。需要考虑腿部和臂部的 Gap 差异、腿臂耦合效应、以及硬件安全约束。

---

## 97.11 工程部署流程：从训练完成到真机验证 ⭐⭐

### 动机：训练完成只是开始

很多研究者花 90% 的时间在训练上，但真正的挑战往往在部署的"最后一公里"。部署失败的原因大多不是算法问题，而是工程细节的疏忽。

本节给出从训练完成到真机验证的完整 pipeline，覆盖每一个可能出错的环节。

### 完整部署 Pipeline

```
Step 1: 模型导出
  训练环境 → ONNX/LibTorch 模型 + 归一化参数 + 配置文件
  ↓
Step 2: Sim-to-Sim 验证
  在独立仿真器（不同于训练仿真器）中加载模型，验证行为一致性
  ↓
Step 3: 静态对齐
  固定输入测试：相同输入在训练端和部署端产生相同输出
  ↓
Step 4: 低速闭环
  真机上以 50% 速度目标运行，工程师准备随时按急停
  ↓
Step 5: 标称闭环
  真机以正常速度运行，记录完整日志（观测、动作、中间变量）
  ↓
Step 6: 边界条件测试
  测试极端情况：推力扰动、地形变化、负载变化
  ↓
Step 7: 长时间运行
  连续运行 30-60 分钟，监控温升、漂移、性能衰退
```

### Step 1-3 的关键检查清单

```python
# 部署前对齐检查脚本
import numpy as np
import onnxruntime as ort

def deployment_alignment_check(
    onnx_path: str,
    obs_mean: np.ndarray,
    obs_std: np.ndarray,
    test_obs: np.ndarray,
    train_output: np.ndarray,
    joint_names_train: list,
    joint_names_deploy: list,
):
    """
    部署前的对齐检查。必须全部通过才能上真机。
    
    检查项：
    1. ONNX 模型推理结果与训练端一致
    2. 归一化参数正确
    3. 关节顺序一致
    4. 坐标系约定一致
    """
    issues = []
    
    # === 检查 1：ONNX 推理一致性 ===
    session = ort.InferenceSession(onnx_path)
    normalized_obs = (test_obs - obs_mean) / (obs_std + 1e-8)
    onnx_output = session.run(None, {"obs": normalized_obs.astype(np.float32)})[0]
    
    max_diff = np.max(np.abs(onnx_output - train_output))
    if max_diff > 1e-4:
        issues.append(f"ONNX 推理差异过大: max_diff = {max_diff:.6f}")
    print(f"[检查1] ONNX 推理差异: {max_diff:.6e} {'PASS' if max_diff < 1e-4 else 'FAIL'}")
    
    # === 检查 2：归一化参数非零 ===
    if np.any(obs_std < 1e-6):
        zero_dims = np.where(obs_std < 1e-6)[0]
        issues.append(f"归一化 std 中有零值: dims {zero_dims}")
    print(f"[检查2] 归一化参数: {'PASS' if np.all(obs_std >= 1e-6) else 'FAIL'}")
    
    # === 检查 3：关节顺序一致 ===
    if joint_names_train != joint_names_deploy:
        issues.append(f"关节顺序不一致!\n  训练: {joint_names_train}\n  部署: {joint_names_deploy}")
    print(f"[检查3] 关节顺序: {'PASS' if joint_names_train == joint_names_deploy else 'FAIL'}")
    
    # === 检查 4：输出范围合理 ===
    max_action = np.max(np.abs(onnx_output))
    if max_action > 1.0:
        issues.append(f"输出动作幅度过大: max = {max_action:.3f}")
    print(f"[检查4] 输出范围: max|a| = {max_action:.3f} {'PASS' if max_action <= 1.0 else 'WARN'}")
    
    if issues:
        print(f"\n{'='*50}")
        print(f"发现 {len(issues)} 个问题！禁止上真机。")
        for i, issue in enumerate(issues):
            print(f"  [{i+1}] {issue}")
    else:
        print(f"\n所有检查通过。可以进入 Step 4（低速闭环）。")
    
    return len(issues) == 0
```

### 安全降级策略

真机部署必须有安全保障。以下是推荐的三级降级策略：

| 级别 | 触发条件 | 动作 | 恢复条件 |
|------|---------|------|---------|
| **Level 1: 限幅** | 任意关节力矩 > 额定值 80% | 限制力矩到额定值 70% | 自动恢复 |
| **Level 2: 刹车** | 状态估计异常（IMU 角速度 > 阈值）或连续 3 步力矩限幅 | 关节锁定（高 Kp, Kd），停止策略 | 人工确认后重启 |
| **Level 3: 急停** | 硬件异常（过温、过流）或操作员按急停 | 电机断电 | 硬件检查后重启 |

```python
class SafetyMonitor:
    """
    三级安全监控。在策略输出和硬件之间作为最终安全层。
    """
    def __init__(self, torque_limit, ang_vel_limit=10.0, temp_limit=80.0):
        self.torque_limit = torque_limit          # Nm
        self.ang_vel_limit = ang_vel_limit        # rad/s
        self.temp_limit = temp_limit              # Celsius
        self.consecutive_clips = 0
        self.safety_level = 0  # 0=正常, 1=限幅, 2=刹车, 3=急停
    
    def check_and_clip(self, action_torque, imu_ang_vel, motor_temps):
        """
        检查安全约束并返回裁剪后的动作。
        
        返回:
            safe_action: 安全动作
            level: 当前安全等级
        """
        # === Level 3：硬件异常 ===
        if np.any(motor_temps > self.temp_limit):
            self.safety_level = 3
            return np.zeros_like(action_torque), 3
        
        # === Level 2：状态异常 ===
        if np.linalg.norm(imu_ang_vel) > self.ang_vel_limit:
            self.safety_level = 2
            # 刹车模式：返回阻尼力矩（关节速度 * 高Kd）
            return np.zeros_like(action_torque), 2
        
        if self.consecutive_clips >= 3:
            self.safety_level = 2
            return np.zeros_like(action_torque), 2
        
        # === Level 1：力矩限幅 ===
        clipped = np.clip(action_torque, 
                         -self.torque_limit * 0.7,
                          self.torque_limit * 0.7)
        if np.any(np.abs(action_torque) > self.torque_limit * 0.8):
            self.safety_level = 1
            self.consecutive_clips += 1
        else:
            self.consecutive_clips = 0
            self.safety_level = 0
        
        return clipped, self.safety_level
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：部署时忘记 clamp 动作到安全范围**
> 错误做法：策略输出直接发送到电机，没有限幅
> 现象：策略输出一个极端值 → 电机瞬间拉到位置极限 → 碰撞/过流保护 → 机器人断电倒地
> 根本原因：仿真中策略可能偶尔输出极端动作但被仿真器的积分步"平滑"掉了，真机上没有这种保护
> 正确做法：在策略输出和硬件之间加入 SafetyMonitor，始终限制力矩/位置范围

> 💡 **概念误区：认为 sim-to-sim 通过就可以直接上真机**
> 新手想法："在 MuJoCo 中验证了策略可以在不同参数下工作，那真机也应该没问题"
> 实际上：sim-to-sim 只验证了策略的参数鲁棒性，但没有覆盖软件部署层面（D6）的问题。真实部署中最常见的失败是：关节顺序错误、归一化参数不一致、坐标系约定不同——这些问题在 sim-to-sim 中根本不存在
> 正确做法：必须经过 Step 3（静态对齐检查）再上真机

### 练习 97.11

**[A] ⭐⭐** 实现上述 `deployment_alignment_check` 函数，用一个训练好的 IsaacGym 策略测试。刻意引入一个关节顺序错误，验证检查能否捕获。

**[B] ⭐⭐** 实现 `SafetyMonitor` 类，在仿真环境中测试三级降级策略的触发和恢复逻辑。

**[C] ⭐⭐⭐** 设计一个完整的部署日志格式（包含哪些字段、采样频率、存储格式），使得任何真机失败都能事后分析根因。

---

## 97.12 前沿：可微仿真与世界模型辅助 Sim-to-Real ⭐⭐⭐⭐

### 动机：从"黑盒仿真"到"可微仿真"

前面所有方法都将仿真器视为**黑盒**——我们可以在仿真器中执行策略、收集轨迹，但不能对仿真器的物理计算做反向传播。这限制了我们只能用无梯度方法（如 PPO 的策略梯度或进化算法）来优化。

**可微仿真**（Differentiable Simulation）打破了这个限制：它让物理仿真的每一步都支持梯度计算。这意味着可以直接对"策略 → 仿真器 → 轨迹回报"的整个链路做反向传播，获得解析梯度。

#### 可微仿真的价值

$$
\frac{\partial J}{\partial \theta} = \frac{\partial}{\partial \theta} \sum_t r(s_t, a_t) = \sum_t \frac{\partial r}{\partial s_t} \underbrace{\frac{\partial s_t}{\partial s_{t-1}} \cdot \frac{\partial s_{t-1}}{\partial s_{t-2}} \cdots}_{\text{需要仿真器可微}} \cdot \frac{\partial s_0}{\partial \theta}
$$

有了解析梯度，参数辨识和策略优化都可以用梯度下降，收敛速度远快于无梯度方法。

### 当前可微仿真引擎

| 引擎 | 开发者 | 特点 | 接触微分 | 成熟度 |
|------|--------|------|---------|--------|
| **Brax** | Google | JAX 原生，纯 Python | 弹簧接触（连续） | 中 |
| **MuJoCo MJX** | DeepMind | MuJoCo 的 JAX 后端 | 凸优化接触 | 高 |
| **Newton** | NVIDIA/DeepMind/Disney | GPU 加速，Linux Foundation 开源 | 多种求解器 | 发展中（2025 年 9 月开源） |
| **DiffTaichi** | MIT | 基于 Taichi 的可微仿真框架 | 自定义 | 中 |
| **Warp** | NVIDIA | 基于 CUDA 的可微物理 | 有限支持 | 高 |

#### 接触微分的挑战

可微仿真的最大挑战是**接触**。接触的物理本质是不连续的——一个物体要么接触要么不接触，摩擦力在滑动和粘着之间切换。这些不连续性导致梯度不存在或梯度为零。

当前的解决方案：

1. **软接触模型**：用弹簧-阻尼器替代硬接触，使力连续可微（但物理精度下降）
2. **接触平滑化**：在接触边界附近用 sigmoid 函数平滑，获得近似梯度
3. **随机化平滑**：对接触参数加微小噪声，用期望梯度替代不存在的精确梯度

### 世界模型辅助 Sim-to-Real

另一个前沿方向是使用**学习的世界模型**（World Model）来替代或补充物理仿真器。

核心思想：在真机上收集少量数据，训练一个神经网络世界模型 $\hat{s}_{t+1} = w_\psi(s_t, a_t)$，然后在这个学习的世界模型中训练或微调策略。

**TWIST (2025)** 是这个方向的代表工作：它用 Teacher-Student 架构训练世界模型——教师世界模型使用特权状态信息（仿真中），学生世界模型只用可部署传感器信息。蒸馏后的学生世界模型可以在真机上运行，为策略提供"想象中的未来状态"，加速在线适应。

**Real-is-Sim (2025)** 提出了"动态数字孪生"概念：在真机运行时，持续用传感器数据校正仿真器状态，使仿真器成为真实世界的实时镜像。策略可以在这个"纠正后的仿真器"中安全地做前瞻规划。

### 未来趋势

| 趋势 | 预期时间线 | 影响 |
|------|-----------|------|
| 可微仿真成为标准工具 | 2025-2027 | SysID 和策略优化效率提升 10 倍以上 |
| 世界模型替代物理仿真器 | 2026-2028 | 减少对精确物理建模的依赖 |
| Foundation Model for Robotics | 2026-2028 | 预训练的通用机器人策略，少样本适应 |
| 可微仿真+RL 统一框架 | 2027+ | 端到端可微的训练-部署-适应流程 |

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：认为可微仿真可以完全替代 RL**
> 新手想法："有了解析梯度就不需要 PPO 了，直接梯度下降优化策略"
> 实际上：可微仿真的梯度在长 horizon 上会消失或爆炸（与 RNN 的梯度消失问题类似）。对于 > 100 步的轨迹，解析梯度通常不可用。实践中，可微仿真更适合短 horizon 任务（如灵巧操作）或与 RL 结合使用（用解析梯度初始化，RL 做细调）

> 💡 **概念误区：认为世界模型可以无限外推**
> 世界模型只在训练数据覆盖的状态空间内可靠。一旦策略探索到分布外的状态，世界模型的预测就不可信。这与 Model-Based RL 的经典问题"模型利用"（model exploitation）是同一个问题

### 练习 97.12

**[A] ⭐⭐⭐** 用 MuJoCo MJX 实现一个简单的可微仿真实验：倒立摆控制。对比用解析梯度和 PPO 分别优化策略的收敛速度。

**[B] ⭐⭐⭐⭐** 调研 Newton 物理引擎的最新进展，总结其在接触微分上的解决方案，并讨论其对腿足机器人 Sim-to-Real 的潜在影响。

---

## 本章小结

本章系统地覆盖了 Sim-to-Real 迁移的完整方法论框架。以下是各方法的核心要点回顾：

| 节 | 方法 | 核心思想 | 关键公式/概念 | 适用场景 |
|-----|------|---------|-------------|---------|
| 97.1 | Gap 分类学 | 六维分类框架 | D1-D6 维度 | 问题诊断 |
| 97.2 | Domain Randomization | 参数分布上最大化期望回报 | $\max_\pi \mathbb{E}_{\xi \sim p(\xi)}[J(\pi,\xi)]$ | 参数不确定 |
| 97.3 | System Identification | 最小二乘参数辨识 | $\hat{\phi} = (\mathbf{A}^T\mathbf{A})^{-1}\mathbf{A}^T\mathbf{b}$ | 需精确仿真 |
| 97.4 | Actuator Network | 神经网络学习执行器非线性 | $\hat{\tau} = f_\theta(q_{\text{des history}}, q, \dot{q})$ | 执行器Gap主导 |
| 97.5 | Action History | 状态增广处理延迟 | $\tilde{s}_t = (s_{t-d}, a_{t-d:t-1})$ | 延迟不确定 |
| 97.6 | RMA | 在线隐式系统辨识 | 教师-编码器-适应模块三阶段 | 环境动态变化 |
| 97.7 | Delta-Action | 残差修正基线策略 | $a_t = \pi_{\text{base}}(o_t) + \delta_\theta(o_t)$ | 少量真机数据 |
| 97.8 | Teacher-Student | 蒸馏特权信息到可部署策略 | $\mathcal{L} = \|a_S - a_T\|^2$ | 部署传感器受限 |
| 97.9 | Param Adaptation | 显式在线参数估计 | $\hat{\xi}_t = g_\psi(o_{t-H:t})$ | 需显式参数值 |
| 97.10 | 选型指南 | 决策树 + 平台推荐 | — | 方案设计 |
| 97.11 | 部署流程 | 七步 Pipeline + 安全降级 | — | 工程部署 |
| 97.12 | 前沿方向 | 可微仿真 + 世界模型 | — | 研究前沿 |

> **全章本质洞察**：Sim-to-Real 不是单一问题，没有银弹。成功的迁移总是多种方法的**组合**——DR 提供鲁棒性基础，SysID/ActNet 提高仿真精度，RMA/Delta-Action 提供在线适应能力，Teacher-Student 解决信息差。关键能力不是掌握某一种方法，而是能够**诊断 Gap 的来源并选择正确的工具组合**。

---

## 累积项目：本章新增模块

**项目：从零构建 Sim-to-Real 迁移系统**

| 已完成（前序章节） | 本章新增 |
|-------------------|---------|
| 足式/190: PPO 训练基础策略 | DR 参数表设计与消融验证 |
| 足式/200: ONNX 导出与部署 | Action History 缓冲区实现 |
| 复合/240: ASAP Delta-Action 实例 | 完整部署 Pipeline + 安全监控 |

**本章累积项目任务**：在 IsaacGym 中训练一个 Anymal 行走策略，依次加入 DR、Action History、RMA，导出 ONNX 模型，通过部署对齐检查，在 MuJoCo 中做 sim-to-sim 验证。最终提交包含：

1. DR 配置文件 + 消融实验结果
2. Action History 实现代码
3. RMA 三阶段训练日志
4. 部署对齐检查报告
5. sim-to-sim 迁移性能表

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Tobin et al. "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017) | ⭐⭐ | DR 的开创性工作 |
| Tan et al. "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots" (RSS 2018) | ⭐⭐⭐ | 最早在四足上做完整 Sim-to-Real 的工作 |
| Hwangbo et al. "Learning Agile and Dynamic Motor Skills for Legged Robots" (Science Robotics 2019) | ⭐⭐⭐ | Actuator Network 的提出 |
| Kumar et al. "RMA: Rapid Motor Adaptation for Legged Robots" (2021) | ⭐⭐⭐ | RMA 原始论文 |
| He et al. "ASAP: Aligning Simulation and Real-World Physics" (2025) | ⭐⭐⭐⭐ | Delta-Action 在人形上的最新应用 |
| PACE: "Towards Bridging the Gap: Systematic Sim-to-Real Transfer for Diverse Legged Robots" (2025) | ⭐⭐⭐ | 20 秒数据完成系统辨识 |
| Aljalbout et al. "The Reality Gap in Robotics: Challenges, Solutions, and Best Practices" (2025) | ⭐⭐⭐ | 最全面的 Sim-to-Real 综述 |
| Lim & Xu "Distributionally Robust Reinforcement Learning" (2022) | ⭐⭐⭐⭐ | DR 的 DRO 理论基础 |
| Mehta et al. "Learning Domain Randomization Distributions for Training Robust Locomotion Policies" (2020) | ⭐⭐⭐ | CVaR + DR 的结合 |
| Newton Physics Engine Documentation (2025) | ⭐⭐⭐ | 可微仿真最新引擎 |

---

## 故障排查手册

🔧 **故障排查手册**

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| 真机站起来就剧烈抖动 | 观测归一化不一致（D6） | 1. 固定输入测试对比训练/部署端输出 2. 检查 mean/std 文件 3. 打印部署端输入范围 | 97.1, 97.11 |
| 仿真完美但真机打滑 | 摩擦系数未 DR 或范围太小（D1,D2） | 1. 检查 DR 配置中摩擦范围 2. 在仿真中手动设低摩擦测试 3. 扩大 DR 范围重训 | 97.2 |
| 快速运动时步态错乱 | 延迟未处理（D5） | 1. 测量真机端到端延迟 2. 检查是否有 Action History 3. 加延迟 DR | 97.5 |
| 行走一段时间后性能衰退 | 电机温升导致力矩常数下降（D3） | 1. 监控电机温度 2. 检查 Actuator Network 是否包含温度特征 3. 降低力矩上限 | 97.4 |
| 策略在新地形完全失败 | 训练 DR 范围未覆盖新地形参数 | 1. 判断新地形参数是否在 DR 范围内 2. 检查 RMA 适应模块是否激活 3. 扩大 DR 或用 Delta-Action 修正 | 97.2, 97.6, 97.7 |
| 关节动作方向反转 | 关节命名顺序或符号约定不一致（D6） | 1. 打印训练和部署的关节名列表 2. 逐关节发送单侧指令验证方向 3. 检查坐标系约定 | 97.11 |
| RMA 适应模块不收敛 | 观测历史中噪声过大或编码维度过高 | 1. 检查阶段2训练时是否加了传感器噪声 2. 降低编码维度 3. 增加历史长度 | 97.6 |
