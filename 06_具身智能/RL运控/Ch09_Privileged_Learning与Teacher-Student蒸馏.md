# 09. Privileged Learning 与 Teacher-Student 蒸馏

> **本章定位**：这是 RL 运动控制从仿真到部署链路中的核心枢纽章节。Ch05 讲了 observation/action 接口设计，Ch07 讲了 PPO 训练管线，Ch08 讲了 domain randomization——但有一个根本问题始终没有正面解决：仿真中可以看到的信息和真机上能获得的信息不一样。本章直面这条矛盾，系统讲解 privileged learning、asymmetric actor-critic、RMA adaptation module 和 teacher-student 蒸馏在 mjlab 与 Isaac Lab 双框架中的工程落地。重点不是推导蒸馏理论——而是讲清楚哪些信息该放在 actor、哪些该放在 critic、哪些该给 teacher、信息泄漏如何诊断、teacher-student 蒸馏的三阶段工程流程，以及 extreme-parkour 这样的工业级项目如何把上述所有技巧串联成一条完整管线。
>
> **前置依赖**：Ch05（Observation/Action 接口设计）、Ch06（Reward/Curriculum）、Ch07（PPO 训练管线）、Ch08（Domain Randomization）
>
> **关键文献**：Pinto et al., RSS 2018（Asymmetric Actor Critic for Image-Based Robot Learning）、Kumar et al. 2021（RMA: Rapid Motor Adaptation for Legged Robots）、Lee et al. 2020（Learning Quadrupedal Locomotion over Challenging Terrain）、Cheng et al. 2024（Extreme Parkour with Legged Robots, ICRA'24）
>
> **参考项目**：🔧 mjlab actor/critic group 配置 · 🔧 Isaac Lab observation group 配置 · ✅ `github.com/chengxuxin/extreme-parkour`（ICRA'24）

---

## 前置自测

📋 **答不出 ≥ 3 题 → 先回前置章节复习**

| # | 问题 | 检查目的 |
|---|------|----------|
| 1 | actor observation 和 critic observation 可以不同吗？在 PPO 中这如何影响 advantage 计算？ | 检查是否理解 asymmetric actor-critic 的基本结构 |
| 2 | 部署时只需要 actor 还是同时需要 actor 和 critic？为什么？ | 检查是否理解 critic 的训练辅助角色 |
| 3 | `enable_corruption=True` 在 actor group 和 critic group 上的含义有什么不同？ | 检查是否理解 noise 在两个 group 中的不同语义 |
| 4 | 一个信号"仿真中可获得但真机上不存在"，应该放在 actor_terms、critic_terms 还是都不放？ | 检查是否理解信息边界划分 |
| 5 | domain randomization 解决的是"参数不确定性"还是"信息不存在性"？两者有何区别？ | 检查是否区分 DR 和 privileged learning 的问题域（Ch08 知识） |
| 6 | 在 mjlab 中，`obs_groups` 配置的作用是什么？它如何将 env 的 observation 路由给 runner 的网络？ | 检查是否理解 runner 的 observation routing（Ch07 知识） |
| 7 | teacher-student 蒸馏的 student loss 与 RL reward 有什么本质区别？ | 检查是否区分监督学习和强化学习的训练信号 |
| 8 | RSL-RL 4.0 中 actor 和 critic 的配置方式与旧版有何不同？新版为什么要解耦？ | 检查是否了解 RSL-RL 4.0 的 breaking change（Ch07 知识） |
| 9 | observation normalization 的 running mean/std 在 checkpoint 恢复时需要注意什么？ | 检查是否理解 normalizer 状态管理 |

## 本章目标

学完本章后，你应该能够：

1. **分类** 所有常见 observation 信号到四个角色（actor / critic / teacher / student），理解每个角色的信息边界和部署状态
2. **配置** mjlab 和 Isaac Lab 中的 actor/critic observation group，包括 RSL-RL 4.0 的解耦配置、terms 选择、corruption 控制和 obs_groups routing
3. **理解** RMA adaptation module 的工程接线，并能对比三种在线适应范式（RMA / causal transformer / TTT）的适用场景
4. **设计** teacher-student 蒸馏的完整工程流程——从 teacher 训练、数据收集到 student 训练和双指标评估
5. **诊断** 信息泄漏和 normalization 问题，运用 15 项 debug checklist 系统排查 actor 中的不可部署信号
6. **精读** extreme-parkour 的三阶段管线（含 ROA 和 MTS 机制），并了解 HOVER 和 VIRAL 的前沿蒸馏方案
7. **导出** 训练好的 actor/student 为 ONNX 格式，正确烘焙 normalizer 并完成 sim-to-sim 验证
8. **选型** 面对新项目时，根据信息差类型和工程预算选择非对称 AC / RMA / 蒸馏的合适层次


---

## 9.1 算法回顾：Privileged Information 与蒸馏的三种形态 ⭐

> **这一节解决什么问题**：用 20% 的篇幅唤醒读者对 privileged learning 核心概念的记忆，建立三种形态的统一视角，为后续工程实现做铺垫。

### 信息不对称是部署的核心矛盾

在 MuJoCo 仿真中，你可以精确读取每个接触点的 3D 力向量、地形表面每个点的精确高度、机器人质心的精确位置和速度、所有物理参数（摩擦系数、质量分布、关节阻尼）。这些信息在仿真中是"免费"的——`mj_step()` 之后直接读取 `data` 结构体即可。但在真机上，你能获得的只有 IMU 角速度和加速度（有噪声和偏置）、编码器的关节位置（高精度但有量化）、编码器差分得到的关节速度（噪声较大）、可能的足底力传感器（如果有的话）、可能的深度相机（延迟 30-100ms，视野有限）。

这个差距的类比是：仿真就像开着上帝视角玩即时战略游戏——你能看到地图全貌、每个单位的血量和位置。真机就像正常玩——战争迷雾遮住了大部分地图，你只能看到侦察兵视野内的信息。如果你在上帝视角下训练出的策略依赖了全局地图信息来做战术决策，一旦切换到正常视角，策略就会失效——不是因为操作水平下降了，而是因为决策所依赖的信息根本不存在了。这个类比有一个重要的边界：游戏中的战争迷雾是明确的设计，你知道哪些区域看不到；而机器人部署中的信息缺失更隐蔽——仿真代码不会告诉你"这个 observation term 真机上没有"。

回顾 Ch05：我们在 observation 设计一章中提出了"部署可得性原则"——每个 actor observation term 必须在真机上有等价的传感器来源。本章在系统层面落地这个原则：整个训练-部署链路如何利用和隔离 privileged 信息。

> **本质洞察**：privileged learning 的本质不是"让策略作弊"，也不是"让训练更快"。它是在"训练时知道但部署时不知道"这条信息鸿沟上搭桥。critic 和 teacher 是桥的两端——critic 用特权信息提高训练信号质量，teacher 用特权信息生成高质量示范——但桥的目的地始终是一个只依赖部署可得信息的 actor 或 student。

### Privileged Information 概念（RMA, Kumar 2021）

Privileged learning 的概念在机器人 RL 中的系统化应用可以追溯到 Kumar et al. 2021 的 RMA（Rapid Motor Adaptation for Legged Robots）。RMA 的核心观察是：四足 locomotion 的控制困难很大程度上来自环境参数的不确定性——摩擦系数、负载质量、地面坡度等参数在仿真中可以精确读取，但在真机上无法直接测量。RMA 的解决方案是两阶段训练：第一阶段用一个 environment factor encoder 把仿真中的 privileged 环境参数编码成低维 latent，然后让 base policy 以 proprioception + latent 作为输入来学习控制；第二阶段用一个 adaptation module 从 proprioception history 中估计这个 latent——这样部署时就不需要真实的环境参数了。

RMA 的思想可以类比为"翻译器"：teacher 说的是仿真语言（精确的摩擦系数 0.7、质量偏移 +0.3kg），adaptation module 把这些翻译成部署可得语言（"最近几步的关节响应模式表明地面比较滑"）。翻译的损失（latent 近似误差）是不可避免的，但只要翻译质量足够好——latent 能捕捉对控制决策最重要的环境特征——策略就能在部署时正常工作。

### 如果不用 Privileged Learning 会怎样

**反面案例 A：直接把 privileged 信息喂给 actor。** 你在 mjlab 中训练 Go1 在 rough terrain 上行走，为了加速训练，把仿真接触力（`foot_contact_forces`）加入 actor observation。训练曲线漂亮——500 iteration 后 reward 接近饱和。但部署到真实 Go1 上，机器人迈出第一步就摇晃，三步后摔倒。原因是真实 Go1 没有与仿真 `ContactSensor` 等价的足底力传感器。这不是 sim-to-real gap 的常规误差（摩擦不匹配导致的打滑），而是信息边界的根本错误——actor 依赖了部署时不存在的信号。

**反面案例 B：完全不给 critic 特权信息。** 你谨慎地只让 actor 看 IMU + 关节编码器 + 命令（全部部署可得），但 critic 也只看同样的信息。这时 PPO 的 value function 估计会很差——因为 critic 要从噪声 IMU 数据中估计状态的"好坏"，方差极大。结果是 advantage 估计不准确，policy gradient 噪声大，训练需要更多 iteration 才能收敛（或者根本收敛不到好的策略）。回顾 Ch07：critic 不参与部署，它只在训练时产生 value 估计来计算 advantage。让 critic 看到 privileged 信息不会影响 actor 的部署输入，但会显著降低训练信号的方差。

**反面案例 C：只用 domain randomization 硬扛。** Ch08 讲了 DR 的强大能力——随机化摩擦、质量、延迟让策略对参数变化鲁棒。但 DR 解决的是"参数在某个范围内不确定"，而不是"某个信号根本不存在"。如果 actor 看不到地形高度但需要在崎岖地形上行走，再多的摩擦 randomization 也帮不了它——它需要从历史 proprioception 中推断地形信息，或者有一个 teacher 先用完美地形信息学会怎么走，然后蒸馏给只看 proprioception 的 student。

### 三种形态：非对称 AC → BC 蒸馏 → 并发 TS

Privileged learning 在机器人 RL 中有三种主要形态，它们解决的问题层次不同，工程复杂度递增。理解这三种形态的边界和适用场景，是本章的第一个核心任务。

| 维度 | 非对称 Actor-Critic | BC 蒸馏（Teacher-Student） | 并发 Teacher-Student |
|------|---------------------|--------------------------|---------------------|
| **核心思想** | critic 看 privileged，actor 看部署信息 | teacher 用 privileged 训练出最优策略，student 模仿 teacher | teacher 和 student 同时训练，teacher 持续产生示范 |
| **特权角色** | critic（估值器） | teacher（策略） | teacher（策略） |
| **特权角色输出** | value（标量） | action 或 latent（向量） | action 或 latent |
| **部署角色学习方式** | RL（policy gradient） | 监督学习（imitation loss） | RL + imitation loss |
| **训练阶段数** | 1（同时训练 actor 和 critic） | 2（先 teacher RL，再 student BC） | 1（同时但有两个网络） |
| **工程复杂度** | ⭐ 低（框架原生支持） | ⭐⭐⭐ 高（多阶段管线） | ⭐⭐⭐⭐ 很高 |
| **经典论文** | Pinto et al., RSS 2018 | Kumar et al. 2021, Lee et al. 2020 | 多种变体 |
| **框架支持** | mjlab/Isaac Lab 原生 | RSL-RL DistillationRunner | 需自定义 |

**形态一：非对称 Actor-Critic（Asymmetric AC）。** 这是最简单、最常用的 privileged learning 形态。critic 看到完整的 privileged observation（包括接触力、地形真值、环境参数），actor 只看部署可得信息。两者在 PPO 中同时训练。训练结束后 critic 被丢弃，只部署 actor。mjlab 和 Isaac Lab 都原生支持这种配置——只需要在 env config 中分别定义 actor group 和 critic group。

**形态二：BC 蒸馏（Teacher-Student Distillation）。** 当 actor 和部署输入之间的信息差太大（比如 actor 需要从深度图像中做决策，但训练时可以直接用低维 state），非对称 AC 不够用——因为 critic 只能帮助训练信号更干净，不能替代 actor 缺失的感知能力。这时需要先训练一个看 privileged 信息的 teacher（它可以是一个非对称 AC 中的 actor），然后用监督学习把 teacher 的行为蒸馏给只看部署信息的 student。

**形态三：并发 Teacher-Student。** teacher 和 student 同时在环境中运行和训练。teacher 用 RL + privileged 信息持续改进，student 同时模仿 teacher 的最新行为。这种方法避免了两阶段管线的复杂性，但工程上更难稳定——因为 teacher 的行为分布在不断变化，student 在追踪一个移动目标。

> **本质洞察**：三种形态的统一视角是"信息蒸馏链"的长度。非对称 AC 只有一级蒸馏（privileged → value → advantage → actor gradient），链很短。BC 蒸馏有两级（privileged → teacher action → student imitation loss → student gradient），链更长。并发 TS 试图把两级压缩到同一个训练循环中。链越长，信息损失越大，但能跨越的信息鸿沟也越宽。

### Estimator 网络设计概览

RMA 提出的 adaptation module（也叫 estimator 网络）是 privileged learning 工程化的关键组件。它的输入是部署可得的历史 observation（通常是 proprioception history），输出是对 privileged 环境信息的低维 latent 估计。这个 latent 被拼接到 base policy 的输入中，使 actor 在部署时能间接"感知"不可直接测量的环境参数。

Estimator 的训练有两种时机：与 policy 同步训练（online）和 policy 训练完后单独训练（offline）。同步训练的优势是 estimator 能适应 policy 诱导的状态分布，劣势是增加训练复杂度。离线训练更简单——先用 privileged teacher 的 rollout 收集 `(history, privileged_latent)` 对，然后用监督学习训练 estimator。我们将在 9.4 节深入讨论 estimator 的工程细节。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：混淆 asymmetric AC 和 teacher-student。** 新手经常把两者搞混——因为都涉及"一个角色看到更多信息"。关键区分：asymmetric AC 的 critic 输出 value（标量），teacher-student 的 teacher 输出 action（向量）。critic 帮助训练但不产生动作，teacher 直接产生可模仿的动作。

💡 **概念误区：认为"privileged learning 只用于 locomotion"。** 实际上 manipulation 任务（灵巧手操作、抓取）中同样存在大量 privileged 信息——物体精确位姿、接触法线、滑移检测。只是 locomotion 文献中 privileged learning 被更系统地研究和工程化了。

🧠 **思维陷阱：认为"teacher 越强 student 就越好"。** teacher 太强可能是因为过度依赖 privileged 信息做出了 student 根本无法复现的行为。最好的 teacher 是"在 privileged 信息帮助下找到一种 student 也能近似复现的高质量行为模式"——而不是"充分利用一切 privileged 信息达到极致表现"。这就像请一个数学天才教小学生——如果天才用高等数学解题，小学生无法模仿；如果天才用小学方法但解得又快又准，小学生才能学到东西。

### 四个角色的生命周期总结

为了彻底消除 actor / critic / teacher / student 四个角色的混淆，用生命周期的视角做一个完整总结。这个总结贯穿全章，后续每一节都会回到这张表。

| 角色 | 何时创建 | 何时活跃 | 何时销毁 | 部署保留？ | 信息来源 |
|------|---------|---------|---------|----------|---------|
| actor | PPO 训练开始 | 训练 + 评估 + 部署 | 永不销毁 | ✅ 是 | 部署可得信息 |
| critic | PPO 训练开始 | 仅训练期间（估计 value） | 训练结束后丢弃 | ❌ 否 | 部署信息 + privileged |
| teacher | teacher RL 训练开始 | teacher 训练 + rollout 收集 | student 训练完后丢弃 | ❌ 否 | 部署信息 + privileged |
| student | student 蒸馏训练开始 | 蒸馏训练 + 评估 + 部署 | 永不销毁 | ✅ 是 | 仅部署可得信息 |

关键观察：最终被部署的只有 actor 或 student——它们是整个 privileged learning 流水线的唯一产出物。critic 和 teacher 都是训练辅助角色，完成使命后即可丢弃。这意味着 critic 和 teacher 的计算成本只影响训练时间，不影响部署推理延迟。你可以给 critic 一个很大的网络（更准确的 value 估计），给 teacher 很丰富的 observation（更强的策略）——因为它们的大小和复杂度不会影响最终部署的模型。

另一个关键区分：actor 和 student 虽然都是部署角色，但学习方式不同。actor 通过 RL（policy gradient, trial and error）学习，student 通过监督学习（模仿 teacher 的 action）学习。actor 从 reward 信号中学习"什么是好的行为"，student 从 teacher 示范中学习"teacher 认为什么是好的行为"。两种学习方式的工程含义完全不同：actor 训练需要好的 reward 设计（Ch06 的主题），student 训练需要好的 teacher 和足够的蒸馏数据。

### 两种方法可以组合

Asymmetric actor-critic 和 teacher-student 不是互斥的——它们可以组合使用。一个常见的完整流水线是：

1. **用 asymmetric AC 训练 teacher**——teacher 的 critic 看 privileged，teacher 的 actor 也看 privileged
2. **用 teacher-student 蒸馏把 teacher 能力迁移到 student**——student 只看部署可得信息

在这个组合中，critic 帮助 teacher 训练得更好（第一层 privileged learning），teacher 再帮助 student 学会在有限信息下行动（第二层 privileged learning）。这是一个两级信息蒸馏链。

如果不做第一级（不用 asymmetric critic 帮 teacher）会怎样？teacher 的训练效率可能下降，但只要 teacher 最终性能足够好，第二级蒸馏仍然可以成功。第一级是效率优化，第二级才是信息边界的核心。

### Multi-Critic 架构：超越单 Critic 的设计

上面的讨论假设只有一个 critic。但 2025 年的前沿工作（HoST, RSS'25 Best Systems Paper Finalist）发现，当任务有多个相互冲突的目标时，**多个 critic 各自关注不同的 privileged 信息子集**可以显著改善训练。

HoST（Learning Humanoid Standing-up Control）在训练人形机器人起身任务时，使用了包含安全（safety）、探索（exploration）、精度（accuracy）三个维度的 multi-critic 架构。每个 critic 看到相同的 privileged 信息，但优化不同的 reward 子集：

| Critic | 关注的 Reward 子集 | 为什么需要独立 |
|--------|-------------------|--------------|
| Safety critic | 碰撞惩罚、关节限位、地面接触力 | 安全约束不应被 exploration bonus 稀释 |
| Exploration critic | 姿态多样性、运动幅度 | 探索信号通常比 task reward 弱，需要独立估值 |
| Accuracy critic | 目标姿态跟踪、质心位置 | 精确控制需要高精度 value 估计 |

如果只用一个 critic 同时估计所有 reward 项的 value，critic 的输出是一个标量——它被迫把安全、探索、精度三个维度的价值压缩成一个数字。这意味着在某些状态下，critic 可能因为探索 bonus 很高而给出高 value，掩盖了安全风险很大的事实。多 critic 架构让 PPO 的 advantage 估计更加精确——每个维度有自己的 baseline。

> **本质洞察：** Multi-critic 本质上是把 asymmetric actor-critic 从"信息维度"扩展到了"目标维度"。传统 asymmetric AC 解决的是"critic 比 actor 看到更多状态信息"的不对称；multi-critic 解决的是"不同训练目标之间的价值估计冲突"。两者可以叠加：每个 critic 都看 privileged 信息，但各自优化不同的 reward 子集。

在 mjlab/Isaac Lab 中实现 multi-critic 需要自定义 PPO 训练循环（RSL-RL 的标准 `OnPolicyRunner` 只支持单 critic），但核心修改并不复杂：为每个 critic 维护独立的 value network 和 advantage buffer，然后把多个 advantage 加权合并后用于 policy gradient 更新。具体的 multi-critic PPO 实现超出本章范围（属于 Ch07 训练管线的拓展），这里只需理解其与 privileged learning 的关系。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：混淆 asymmetric AC 和 teacher-student。** 新手经常把两者搞混——因为都涉及"一个角色看到更多信息"。关键区分：asymmetric AC 的 critic 输出 value（标量），teacher-student 的 teacher 输出 action（向量）。critic 帮助训练但不产生动作，teacher 直接产生可模仿的动作。

💡 **概念误区：认为"privileged learning 只用于 locomotion"。** 实际上 manipulation 任务（灵巧手操作、抓取）中同样存在大量 privileged 信息——物体精确位姿、接触法线、滑移检测。只是 locomotion 文献中 privileged learning 被更系统地研究和工程化了。

🧠 **思维陷阱：认为"teacher 越强 student 就越好"。** teacher 太强可能是因为过度依赖 privileged 信息做出了 student 根本无法复现的行为。最好的 teacher 是"在 privileged 信息帮助下找到一种 student 也能近似复现的高质量行为模式"——而不是"充分利用一切 privileged 信息达到极致表现"。这就像请一个数学天才教小学生——如果天才用高等数学解题，小学生无法模仿；如果天才用小学方法但解得又快又准，小学生才能学到东西。

⚠️ **编程陷阱：teacher checkpoint 被当作部署 actor。** 这是一个隐蔽但严重的错误。如果你把 teacher 的 checkpoint 直接导出 ONNX 部署，真机上会失败——因为 teacher 的输入中包含 privileged 信号。部署时应该加载 student（如果用了蒸馏）或 actor（如果只用了 asymmetric AC）的权重。自检方法：部署前打印模型输入维度，确认与部署传感器提供的维度一致。

### 练习

1. **[概念题]** 画出非对称 AC、BC 蒸馏和并发 TS 三种形态的数据流图。标注每种形态中 privileged 信息的流向（从 env 到哪个网络、经过什么变换、最终如何影响部署策略）。
2. **[分析题]** 解释为什么"critic 看到 privileged 信息不算作弊"需要一个前提：critic 不能直接影响 actor 的输入。如果训练框架中 critic 的 hidden state 被传给 actor（某些 shared network 架构），这个前提是否成立？
3. **[设计题]** 假设你要训练一个灵巧手抓取策略。列出仿真中可获得但真机上不存在的 privileged 信号（至少 5 个），并为每个信号判断应该放在 actor、critic 还是 teacher。

---

上一节回顾了三种 privileged learning 形态的算法原理和适用边界。在进入工程配置之前，有一个关键的跨章知识点需要明确：**privileged learning 不是孤立存在的——它和 Ch08 讨论的 Domain Randomization（DR）构成一个统一的训练 recipe**。

回顾 Ch08 的核心结论：DR 通过在训练时随机化环境参数（摩擦、质量、阻尼等），让策略学会在参数不确定性下保持鲁棒。但 DR 本身不解决"策略看不到这些参数"的问题——它只确保策略在参数变化时不崩溃。Privileged learning 补上了这块拼图：teacher/critic 直接看到这些参数的真值，student/actor 从行为历史中推断。

这个 "DR → privileged distillation" 的组合是当前工业级管线的**标准 recipe**：

1. 在**激进的 DR** 下训练 teacher（teacher 看 privileged 信息 + 环境参数的真值）
2. 把 teacher 的能力**蒸馏**到 student（student 只看 proprioception history）

DR 的范围直接决定了 student 能否从 history 中做 implicit system identification。如果 DR 太窄（比如摩擦系数只在 [0.95, 1.05] 范围内变化），proprioception history 中几乎看不出摩擦差异——student 的 adaptation module 学不到有意义的信号。如果 DR 太宽（比如摩擦从 0.01 到 10.0），teacher 在极端参数组合下训练失败——蒸馏的源头就坏了。

| DR 范围 | teacher 训练 | student 蒸馏 | 实际效果 |
|---------|-------------|-------------|---------|
| 太窄 | ✅ 容易收敛 | ❌ history 无法区分不同参数 | student 在 DR 外参数上失败 |
| 适中 | ✅ 稳定收敛 | ✅ history 可区分且可学 | 最优 |
| 太宽 | ❌ teacher 训练不稳定 | ❌ 蒸馏源头不可靠 | 全链路崩溃 |

这就是为什么 Ch08 和 Ch09 应该联合调参——DR 范围和 privileged learning 方案是同一个优化问题的两个面。当你在 Ch08 中选定了 DR 参数后，Ch09 的 privileged 设计应该确保 teacher 能在该 DR 范围下稳定训练，且 student 的 history 输入能编码出足够的环境参数信息。

有了这个全局视角，下一步是把"privileged 信息"这个模糊概念变成一张可操作的分类表——这正是下一节的主题。

## 9.2 Privileged 信息分类表 ⭐⭐

> **这一节解决什么问题**：建立 privileged 信息的系统分类框架，覆盖 locomotion 和 manipulation 中所有常见信号，为后续的 group 配置提供清晰的工程依据。

### 动机：为什么需要分类

"privileged 信息"不是一个二元标签（有或没有），而是一个光谱——不同信号的 privileged 程度、变化速率、可估计性差异很大。如果不做系统分类，你会面临两个工程陷阱：一是把所有非部署信号一刀切放到 critic，忽视了某些信号可以通过 estimator 间接恢复的可能性；二是不清楚哪些 privileged 信号值得给 teacher 看、哪些给了反而有害（比如未来信息会导致 teacher 学出因果违反的行为）。

### 如果不做分类会怎样

不分类的后果是配置时全靠直觉。你可能把摩擦系数和接触力都扔进 critic_terms，觉得"反正都是 privileged"。但这两类信号的性质完全不同：摩擦系数在整个 episode 内恒定，可以通过 5-10 步的 proprioception history 可靠地估计；接触力每步都在变，很难从 history 中推断。如果你用同样的 estimator 去估计这两类信号，要么 history 窗口太短（估不准摩擦），要么 history 窗口太长（对接触力的时效性没帮助）。分类的目的是让你对每类信号采取最合适的处理策略。

### 四类 Privileged 信息

按信息来源和时间特性，privileged 信息可以分成四类。每一类对应不同的工程处理策略。

**第一类：环境物理参数。** 摩擦系数、地面弹性（restitution）、质量分布、关节阻尼、关节刚度。这些参数在单个 episode 内通常不变（或变化极缓慢），但在不同 episode 间被 domain randomization 随机化。它们是"慢变量"——就像天气预报中的"气候"而非"天气"：你不需要每秒更新摩擦估计，几秒钟的 proprioception history 就足以推断当前在什么样的地面上行走。这正是 RMA 的核心思路——用 adaptation module 从 history 中估计这些慢变量的 latent encoding。

**第二类：动态接触信息。** 足底接触力、接触法线方向、接触滑移速度、接触状态（空中 / 触地）。这些信号在步态周期内快速变化——一个步态周期（约 0.3-0.5 秒）内，每条腿经历从触地到离地的完整转变。真机上可能有力传感器提供部分信息，但精度和语义通常与仿真不同。接触信息的特点是"当前值重要、历史值帮助不大"——上一步的接触力对当前步的控制决策参考价值有限。

**第三类：全局感知信息。** 地形高度图真值、物体精确 6DoF 位姿、路径规划全局信息。这些信号在仿真中通过 raycast 或直接读取 state 获得，在真机上需要外部感知系统（深度相机、LiDAR、motion capture）。全局感知信息的特点是"空间范围广、替代成本高"——你需要一个完整的感知管线来替代仿真中一行代码就能读到的信息。

**第四类：未来信息。** 下一步地形变化、即将到来的命令变化、未来接触时序。这些信息违反因果性——真机上不可能获得。它们不应该出现在任何角色的 observation 中（包括 teacher），因为 teacher 如果学会了依赖未来信息，蒸馏到 student 时会失败——student 永远无法复现基于未来信息做出的"提前反应"行为。

| 信息类别 | 典型信号 | 变化速率 | actor 可用？ | critic 可用？ | teacher 可用？ | student 可估计？ | 工程处理策略 |
|---------|---------|---------|-------------|-------------|--------------|---------------|------------|
| 环境物理参数 | 摩擦系数、质量偏移、关节阻尼 | 每 episode 恒定 | 否 | 是 | 是 | 是（通过 history） | RMA adaptation module |
| 动态接触信息 | 足底力、接触法线、滑移速度 | 每步变化 | 通常否 | 是 | 是 | 部分（通过力估计） | critic-only 或力传感器替代 |
| 全局感知信息 | 地形真值、物体位姿 | 连续变化 | 视传感器 | 是 | 是 | 视觉/LiDAR | teacher-student 蒸馏 |
| 未来信息 | 未来地形、未来命令 | N/A | 否 | 否 | 谨慎 | 否 | **禁止使用** |

### 信号到角色的详细映射表

下面这张表覆盖了 mjlab velocity 和 tracking 任务中所有常见 observation 信号。这是你在配置 env_cfg 时的核心参考。

| 信号 | 物理含义 | mjlab term 名 | actor | critic | teacher | 部署来源 |
|------|---------|-------------|-------|--------|---------|---------|
| base_lin_vel | 机体线速度 | `builtin_sensor("robot/imu_lin_vel")` | ✅ | ✅ | ✅ | IMU + 状态估计 |
| base_ang_vel | 机体角速度 | `builtin_sensor("robot/imu_ang_vel")` | ✅ | ✅ | ✅ | IMU |
| projected_gravity | 重力在机体系投影 | `projected_gravity` | ✅ | ✅ | ✅ | IMU 姿态估计 |
| joint_pos | 关节位置偏差 | `joint_pos_rel` | ✅ | ✅ | ✅ | 编码器 |
| joint_vel | 关节速度偏差 | `joint_vel_rel` | ✅ | ✅ | ✅ | 编码器差分 |
| last_action | 上一拍原始动作 | `last_action` | ✅ | ✅ | ✅ | 策略内部记忆 |
| command | 速度/轨迹命令 | `generated_commands("twist")` | ✅ | ✅ | ✅ | 上层规划 |
| height_scan (noisy) | 带噪地形扫描 | `height_scan` + noise | ✅ | — | — | 深度相机/LiDAR |
| height_scan (clean) | 无噪地形扫描 | `height_scan` (no noise) | — | ✅ | ✅ | 仿真 raycast |
| foot_height | 足端离地高度 | `foot_height` | — | ✅ | ✅ | 仿真计算 |
| foot_air_time | 足端滞空时间 | `foot_air_time` | — | ✅ | ✅ | 接触传感器推断 |
| foot_contact | 足端接触状态 | `foot_contact` | — | ✅ | ✅ | 仿真接触检测 |
| foot_contact_forces | 足端接触力 | `foot_contact_forces` | — | ✅ | ✅ | 仿真接触力 |
| body_pos | 全身关键点位置 | `robot_body_pos_b` | — | ✅ | ✅ | motion capture |
| body_ori | 全身关键点朝向 | `robot_body_ori_b` | — | ✅ | ✅ | motion capture |
| friction_coeff | 摩擦系数 | env 内部参数 | — | ✅ | ✅ | 不可测（RMA 估计） |
| body_mass | 机体质量 | env 内部参数 | — | ✅ | ✅ | 不可测（RMA 估计） |

注意 height_scan 出现了两次——actor 版本带噪声（模拟真实深度传感器），critic 版本无噪声（仿真真值）。这是 asymmetric AC 的典型配置模式：同一个物理量，actor 看到的是降质版本，critic 看到的是完美版本。

### 每类 Privileged 信息的最优处理策略

理解了四类分类之后，接下来的工程问题是：对每一类 privileged 信息，应该采取什么处理策略？这个问题的答案取决于两个因素：信息的变化速率和可替代性。

**第一类（环境物理参数）的处理策略：RMA + History。** 这类信息的特点是 episode 内恒定、跨 episode 变化。处理策略的核心是"用 adaptation module 从 history 推断"。工程上的关键参数是 history_length——摩擦系数这样的慢变量通常 5-10 步 history 就够了，但关节阻尼这种需要更长时间才能观察到效果的参数可能需要 15-20 步。如果 DR 的范围很大（如摩擦从 0.2 到 2.0），adaptation module 需要更多训练数据覆盖这个范围。

**第二类（动态接触信息）的处理策略：Critic-Only 或传感器替代。** 接触力这类快变信号难以从 history 推断（因为上一步的接触力对当前步的预测价值有限）。最简洁的处理是只放在 critic 中帮助 value 估计。如果任务对接触感知有强依赖（如灵巧手操作），考虑在真机上安装触觉传感器，然后在 actor 中使用传感器数据（注意需要在仿真中模拟传感器噪声特性）。

**第三类（全局感知信息）的处理策略：Teacher-Student 蒸馏。** 地形高度图和物体位姿这类信息通常需要视觉系统来替代。简单的 history 推断不够——你无法从关节位置历史中"看到"前方 3 米处的台阶。这正是 teacher-student 蒸馏最能发挥价值的场景：teacher 直接看到完美地形/位姿信息快速学会任务，然后蒸馏给使用 depth/RGB 的 student。extreme-parkour（9.5 节精读）就是这类策略的代表。

**第四类（未来信息）的处理策略：禁止使用。** 这是唯一一类即使给 teacher 也可能有害的信息。如果 teacher 看到了"未来 3 步的地形变化"，它可能学会"提前减速以应对即将出现的下坡"——但 student 只看当前信息，永远无法复现这种因果违反的行为。例外情况：如果 teacher 看到的"未来命令"在部署时由上层规划提前发送（比如 T+0.5 秒的目标速度），那么这不算真正的未来信息——它是一种 "lookahead command"，student 也可以接收。

| 信息类别 | 变化速率 | 最优处理策略 | 工程复杂度 | 关键参数 |
|---------|---------|------------|----------|---------|
| 环境物理参数 | 慢（episode 恒定） | RMA adaptation module | ⭐⭐ | history_length, latent_dim |
| 动态接触信息 | 快（每步变化） | critic-only 或传感器替代 | ⭐ | 传感器精度匹配 |
| 全局感知信息 | 中（连续变化） | teacher-student 蒸馏 | ⭐⭐⭐ | 蒸馏阶段数、数据多样性 |
| 未来信息 | N/A | **禁止使用** | — | — |

### 部署可用性评估：每类信号的替代方案

对 privileged 信号进行分类后，下一步是评估每类信号在部署时的替代方案。这个评估直接决定了 teacher-student 设计：如果某类 privileged 信号有可靠的部署替代，student 可以直接使用替代信号；如果没有，student 需要通过 history 或 estimator 间接恢复。

| privileged 信号 | 部署替代方案 | 替代质量 | 替代成本 | 推荐处理 |
|----------------|------------|---------|---------|---------|
| 摩擦系数 | proprioception history → adaptation module | 中等（可从滑移模式推断） | 低（只需软件） | RMA latent |
| 质量偏移 | proprioception history → adaptation module | 中等 | 低 | RMA latent |
| 地形高度图 | 深度相机 + elevation mapping | 高（但有延迟和噪声） | 中等（需传感器硬件） | depth teacher-student |
| 足底接触力 | 力传感器 / proprioception 推断 | 低-中等 | 中-高 | critic-only |
| 物体位姿 | RGB/depth + 6DoF 估计器 | 中等 | 高（需视觉管线） | teacher-student |
| 关节阻尼 | 系统辨识离线测量 | 高（但不实时） | 低 | DR 覆盖即可 |

**实战案例：Go1 Rough Terrain 的 Privileged 设计过程**

以四足 Unitree Go1 在崎岖地形上的 locomotion 任务为例，完整展示 privileged 信息的分类与处理决策过程。

第一步，列出仿真中所有可获取的信号：`joint_pos`(12)、`joint_vel`(12)、`base_ang_vel`(3)、`projected_gravity`(3)、`commands`(3)、`last_actions`(12)、`base_lin_vel`(3)、`friction_coeffs`(1)、`body_mass`(1)、`contact_forces`(12)、`terrain_heights`(187)。

第二步，逐信号分类：
- **部署可得**：joint_pos、joint_vel、base_ang_vel、projected_gravity（IMU）、commands（外部指令）、last_actions（自己发的）
- **环境参数（慢变）**：friction_coeffs、body_mass → 放入 critic + 考虑 RMA
- **接触信息（快变）**：contact_forces → 仅 critic（真机无力传感器）
- **全局感知**：terrain_heights → critic + 考虑 depth teacher-student
- **部署间接可得**：base_lin_vel → 可用 estimator 从 proprioception history 估计

第三步，确定角色分配：

| 角色 | Terms | 维度 |
|------|-------|------|
| **actor** | joint_pos + joint_vel + base_ang_vel + projected_gravity + commands + last_actions | **45** |
| **critic** | actor 全部 + base_lin_vel + friction_coeffs + body_mass + contact_forces + terrain_heights | **249** |
| **信息差** | critic 相对 actor 多出的维度 | **204** |

这个 204 维的信息差解释了为什么 asymmetric AC 对 locomotion 如此有效：critic 用 249 维的丰富信息估计 value，actor 用 45 维的部署信息生成动作。critic 的准确 value 估计让 PPO 的 policy gradient 更加精准，间接帮助 actor 在有限信息下做出更好的决策。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：把 `foot_contact` 放进 actor_terms。** 仿真中的 `foot_contact` 是通过 `ContactSensor` 直接读取的二值信号。在真机上，这个信号需要力传感器或阻抗估计器——两者的精度、延迟和语义都与仿真不同。如果你在 actor_terms 中使用了 `foot_contact`，即使在真机上有力传感器，阈值设置不当也会导致接触检测的 timing 和仿真不一致，策略表现下降。

💡 **概念误区：认为"所有 privileged 信息价值相等"。** 不同 privileged 信号对 value 估计的帮助差异很大。接触力直接影响步态切换决策，价值很高；而质量偏移在整个 episode 内恒定，一旦 policy 适应了就不需要再看。如果 critic 的输入维度有限制，应该优先加入变化快、对决策影响大的信号。

🧠 **思维陷阱：认为"未来信息只要不给 actor 就没问题"。** 即使只给 teacher，如果 teacher 学会了基于未来地形"提前 5 步调整步态"的行为，student 永远无法复现——因为触发这个行为的信息（未来地形）在 student 的输入中不存在。teacher 蒸馏的前提是 teacher 的行为可以被 student 的输入空间"近似解释"。

### 练习

1. **[分类题]** 对以下信号判断属于四类中的哪一类，并决定应该放在 actor、critic 还是 teacher：(a) IMU 角速度 (b) 仿真接触力 (c) 仿真地形高度真值 (d) 关节编码器位置 (e) 摩擦系数真值 (f) 下一步的地形变化 (g) 物体 6DoF 位姿。
2. **[设计题]** 为一个桌面抓取任务设计 privileged 信息分类表。列出至少 8 个 observation 信号，标注每个信号的类别、变化速率和推荐角色归属。
3. **[分析题]** 如果一个 teacher 的 privileged 输入包含"未来 3 步的目标轨迹"，蒸馏到 student（只看当前命令）时会出现什么问题？如何修改 teacher 的输入来解决？

---

有了 privileged 信息的分类框架，下一步就是把这些知识转化为框架配置——在 mjlab 和 Isaac Lab 中如何具体地定义 actor group 和 critic group。这正是下一节的工程重点。

## 9.3 双框架 Privileged Obs 配置 ⭐⭐⭐

> **这一节解决什么问题**：手把手展示如何在 mjlab 和 Isaac Lab 中配置 asymmetric actor-critic 的 observation group，包括 terms 选择、noise 配置、obs_groups routing 和常见错误排查。

### 动机：配置是 privileged learning 落地的第一步

回顾 Ch05（Observation/Action 设计）的关键设计原则：每个 observation term 在加入 actor 之前必须通过"部署可得性"检验——仿真中能读取的信号不等于真机上能获取的信号。Ch05 建立了这个原则但没有给出实施机制。本节正是把这个原则转化为 **framework-level 的配置工程**：通过 actor/critic group 的分组机制，在代码层面强制执行信息边界。

理解了 privileged 信息的分类之后，工程上的第一步是在 env config 中正确配置 actor 和 critic 的 observation group。这一步看似简单（只是配置文件），但错误的后果极其严重——信息泄漏可能在 sim 中完全无感（训练正常、play 正常），只有部署到真机时才暴露。这就像一个 bug 只在生产环境触发而在测试环境中永远正常——你需要一套系统化的配置方法和检查流程来防止它。

### 如果配置错误会怎样

考虑一个具体的失败场景。你在定义 `critic_terms` 时用了 Python 字典展开 `**actor_terms`，然后想覆盖其中的 `height_scan`（去掉噪声）。但你拼错了 key 名，写成了 `heigth_scan`（注意拼写错误）。Python 字典不会报错——它只是新增了一个 key，原来带噪声的 `height_scan` 仍然存在。结果是 critic 同时有两个 height_scan（一个带噪声、一个不带），维度比预期多了一倍。更糟的是，这不会导致训练崩溃——只是 critic 的输入里有冗余信息——你可能几百个 iteration 后才注意到维度不对。

### mjlab 配置详解

在 mjlab 中，observation group 的配置位于 task 的 `env_cfg.py` 文件中。以 velocity task 的 Go1 配置为例：

```python
# src/mjlab/tasks/velocity/velocity_env_cfg.py（简化展示）

# ——— actor 可见的 terms：全部部署可得 ———
actor_terms = {
    "base_lin_vel": ObservationTermCfg(
        func=mdp.builtin_sensor,
        params={"sensor_name": "robot/imu_lin_vel"},
        noise=Unoise(n_min=-0.5, n_max=0.5),     # actor 有噪声
    ),
    "base_ang_vel": ObservationTermCfg(
        func=mdp.builtin_sensor,
        params={"sensor_name": "robot/imu_ang_vel"},
        noise=Unoise(n_min=-0.2, n_max=0.2),     # actor 有噪声
    ),
    "projected_gravity": ObservationTermCfg(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
    ),
    "joint_pos": ObservationTermCfg(
        func=mdp.joint_pos_rel,
        noise=Unoise(n_min=-0.01, n_max=0.01),
    ),
    "joint_vel": ObservationTermCfg(
        func=mdp.joint_vel_rel,
        noise=Unoise(n_min=-1.5, n_max=1.5),
    ),
    "actions": ObservationTermCfg(func=mdp.last_action),
    "command": ObservationTermCfg(
        func=mdp.generated_commands,
        params={"command_name": "twist"},
    ),
    "height_scan": ObservationTermCfg(
        func=envs_mdp.height_scan,
        params={"sensor_name": "terrain_scan"},
        noise=Unoise(n_min=-0.1, n_max=0.1),     # 模拟深度传感器噪声
        scale=1 / terrain_scan.max_distance,
    ),
}

# ——— critic 可见的 terms：actor 全部 + privileged ———
critic_terms = {
    **actor_terms,                                  # 继承 actor 所有 terms
    "height_scan": ObservationTermCfg(              # 覆盖 height_scan（去掉噪声）
        func=envs_mdp.height_scan,
        params={"sensor_name": "terrain_scan"},
        scale=1 / terrain_scan.max_distance,        # 注意：没有 noise 参数
    ),
    "foot_height": ObservationTermCfg(              # critic 额外看到足端高度
        func=mdp.foot_height,
        params={"sensor_name": "foot_height_scan"},
    ),
    "foot_air_time": ObservationTermCfg(            # critic 额外看到滞空时间
        func=mdp.foot_air_time,
        params={"sensor_name": "feet_ground_contact"},
    ),
    "foot_contact": ObservationTermCfg(             # critic 额外看到接触状态
        func=mdp.foot_contact,
        params={"sensor_name": "feet_ground_contact"},
    ),
    "foot_contact_forces": ObservationTermCfg(      # critic 额外看到接触力
        func=mdp.foot_contact_forces,
        params={"sensor_name": "feet_ground_contact"},
    ),
}

# ——— 注册为两个 observation group ———
observations = {
    "actor": ObservationGroupCfg(
        terms=actor_terms,
        concatenate_terms=True,
        enable_corruption=True,       # actor group 打开噪声腐蚀
    ),
    "critic": ObservationGroupCfg(
        terms=critic_terms,
        concatenate_terms=True,
        enable_corruption=False,      # critic group 关闭噪声腐蚀
    ),
}
```

这段配置中有几个关键设计决策值得深入理解。

**`enable_corruption` 的不同语义。** actor group 的 `enable_corruption=True` 意味着所有 term 中定义的 `noise` 会在训练时被应用——这模拟了真实传感器的噪声特性。critic group 的 `enable_corruption=False` 意味着即使 term 中定义了 noise，也不会被应用——critic 看到的是"干净"的数据。如果不这样设置，critic 的 value 估计会因为噪声而方差增大，advantage 信号质量下降。

**`**actor_terms` 字典展开的陷阱。** Python 的 `{**dict1, key: value}` 语法会用后面的 key 覆盖前面的同名 key。这是 critic_terms 覆盖 height_scan 的机制。但如果你拼错了 key 名（如写成 `heigth_scan`），不会覆盖而是新增，导致 critic 看到两个 height_scan（一个有噪声一个没有）。**自检方法：打印 `len(critic_terms)` 和 `len(actor_terms)` 的差值，确认等于你预期添加的 privileged terms 数量。**

**obs_groups routing。** 在 mjlab 的 RSL-RL runner 配置中，`obs_groups` 参数告诉 runner 如何把 env 的 observation group 路由给网络：

```python
# RSL-RL runner 配置中的 obs_groups
obs_groups = {
    "actor": ("actor",),     # runner 的 actor 网络接收 env 的 "actor" group
    "critic": ("critic",),   # runner 的 critic 网络接收 env 的 "critic" group
}
```

这个映射看似多余（actor 对 actor、critic 对 critic），但它的存在是为了灵活性——你可以让 critic 同时接收多个 group 的拼接（如 `"critic": ("actor", "privileged")`），或者在 teacher-student 蒸馏中让 student 和 teacher 接收不同的 group。

### 实战演示：从对称配置升级到非对称配置

假设你已经有一个可工作的 velocity task，当前使用的是对称配置（actor 和 critic 看到相同的 observation）。以下是将其升级为 asymmetric AC 的完整工程步骤。

**Step 1：备份并复制原始配置。**

```bash
# 在 mjlab 项目中
cp env_cfg.py env_cfg_symmetric_backup.py  # 备份
```

**Step 2：识别当前 obs terms 中的 privileged 信号。** 逐个审查每个 term，问自己："真机上有传感器能提供这个信号吗？"

```python
# 审查脚本：为每个 term 标注部署可得性
current_terms = {
    "base_lin_vel":      "⚠️ 间接可得（需要状态估计器）",
    "base_ang_vel":      "✅ IMU 直接提供",
    "projected_gravity": "✅ IMU 可计算",
    "joint_pos":         "✅ 关节编码器",
    "joint_vel":         "✅ 关节编码器差分",
    "actions":           "✅ 自己发出的",
    "command":           "✅ 外部输入",
    "height_scan":       "⚠️ 需要深度相机或足端接触推断",
}
# ⚠️ 标记的 terms 需要决策：保留在 actor（有替代传感器）还是移到 critic-only
```

**Step 3：分拆 observation terms 为两组。** 这是核心修改——把原来的单一 terms dict 拆成 actor_terms 和 critic_terms：

> **工程提示：** Step 3 是整个升级中最容易出错的步骤。修改前后务必用 `len(actor_terms)` 和 `len(critic_terms)` 验证 term 数量，用维度检查验证总维度。

```python
# ——— 修改前（对称配置）———
observations = {
    "policy": ObservationGroupCfg(
        terms=all_terms,
        enable_corruption=True,
    ),
}

# ——— 修改后（非对称配置）———
# 1. actor_terms: 只保留部署可得的信号，添加噪声
actor_terms = {k: v for k, v in all_terms.items()
               if k not in PRIVILEGED_KEYS}
# 为 actor terms 添加噪声
for key in actor_terms:
    actor_terms[key].noise = Unoise(n_min=-0.1, n_max=0.1)

# 2. critic_terms: 包含所有信号，不添加噪声
critic_terms = {k: ObservationTermCfg(
    func=v.func, params=v.params,
    # 注意：不设置 noise（critic 看干净数据）
) for k, v in all_terms.items()}

# 3. 为 critic 额外添加 privileged terms
critic_terms["contact_forces"] = ObservationTermCfg(
    func=mdp.contact_forces, params={"sensor_cfg": contact_sensor_cfg}
)
critic_terms["friction_coeffs"] = ObservationTermCfg(
    func=mdp.friction_coefficients  # 仿真中可读取的地面摩擦系数
)

# 4. 注册两个 group
observations = {
    "actor": ObservationGroupCfg(
        terms=actor_terms, enable_corruption=True,
    ),
    "critic": ObservationGroupCfg(
        terms=critic_terms, enable_corruption=False,
    ),
}
```

**Step 4：修改 runner 配置以使用 obs_groups。** 告诉 RSL-RL runner 把 env 的两个 observation group 分别路由给 actor 和 critic 网络：

```python
# RSL-RL runner 配置
runner_cfg = OnPolicyRunnerCfg(
    ...
    obs_groups={
        "actor": ("actor",),
        "critic": ("critic",),
    },
)
```

**Step 5：运行维度验证。** 训练前必须确认配置正确：

```python
env = gym.make("your-task-v0")
obs = env.reset()
actor_dim = obs["actor"].shape[-1]
critic_dim = obs["critic"].shape[-1]
print(f"Actor: {actor_dim}, Critic: {critic_dim}, Diff: {critic_dim - actor_dim}")
assert critic_dim > actor_dim, "Critic must have more dims than actor!"
```

**Step 6：训练并对比。** 用相同的 seed 和超参数，分别训练对称和非对称版本，对比 value loss 和 reward 曲线。如果非对称版本的 value loss 显著更低，说明 privileged 信息确实帮助了 critic 估值。

> **本质洞察：** 从对称升级到非对称的全部工程量集中在 Step 2-4——识别 privileged terms、分拆配置、修改 routing。算法代码（PPO 训练循环）完全不需要改动。这就是 manager-based 框架的设计优势：observation 配置与训练算法解耦。

### Isaac Lab 配置详解

Isaac Lab 的 observation group 配置遵循类似的逻辑，但命名和 API 有所不同。在 Isaac Lab 中，observation group 通常命名为 `policy`（对应 mjlab 的 `actor`）和 `critic`。

```python
# Isaac Lab velocity task 的 observation 配置（简化展示）
# 位于 omni.isaac.lab_tasks 中

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group (deployed actor)."""

        # 部署可得的 terms
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )

        def __post_init__(self):
            self.enable_corruption = True   # policy group 打开噪声

    @configclass
    class CriticCfg(ObsGroup):
        """Observations for critic group (training only)."""

        # 继承 policy 所有 terms（手动重复或通过继承）
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)      # 注意：无噪声
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            # 无噪声——critic 看到 clean terrain
        )
        # critic 额外的 privileged terms
        contact_forces = ObsTerm(
            func=mdp.contact_forces,
            params={"sensor_cfg": SceneEntityCfg("contact_sensor")},
        )

        def __post_init__(self):
            self.enable_corruption = False   # critic group 关闭噪声

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
```

### 双框架配置对比

| 维度 | mjlab | Isaac Lab |
|------|-------|-----------|
| group 命名 | `"actor"` / `"critic"` | `"policy"` / `"critic"` |
| 配置方式 | dict of `ObservationTermCfg` | `@configclass` + `ObsTerm` 属性 |
| 继承机制 | `**actor_terms` 字典展开 | Python 类继承或手动重复 |
| corruption 控制 | `ObservationGroupCfg(enable_corruption=True/False)` | `__post_init__` 中设 `self.enable_corruption` |
| routing | `obs_groups` 在 runner config 中配 | RSL-RL runner 自动匹配 group 名 |
| 噪声定义位置 | 每个 term 的 `noise` 参数 | 每个 term 的 `noise` 参数 |
| 维度检查 | 手动打印对比 | `env.observation_manager` 可查 |

两个框架的核心逻辑完全一致：把 observation 分成两组，部署组（actor/policy）只包含可部署 terms 且加噪声，训练辅助组（critic）包含 privileged terms 且无噪声。区别主要在 API 形式上。

### RSL-RL 4.0 的 Actor/Critic 解耦配置（⚠️ 重要迁移）

如果你在 Ch07 中使用了 RSL-RL ≥ 4.0（arXiv 2509.10771, Schwarke, Mittal, Rudin, Hoeller, Hutter, 2025），那么 actor 和 critic 的网络配置已经**解耦**——这是一个 breaking change，直接影响本章的 asymmetric AC 配置。

回顾 Ch07 的核心结论：RSL-RL 4.0 用统一的 `RslRlMLPModelCfg` 替代了旧版的 `RslRlPpoActorCriticCfg`，actor 和 critic 可以使用完全不同的网络架构。

```python
# ❌ 旧版 RSL-RL ≤ 3.x（已废弃）
from rsl_rl.algorithms import RslRlPpoActorCriticCfg
rl_cfg = RslRlPpoActorCriticCfg(
    actor_hidden_dims=[256, 128, 64],
    critic_hidden_dims=[256, 128, 64],
    # actor/critic 共享相同网络结构——不灵活
)

# ✅ 新版 RSL-RL ≥ 4.0（推荐）
from rsl_rl.modules import RslRlMLPModelCfg
rl_cfg = RslRlPpoCfg(
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        obs_normalization=True,   # actor 的 running mean/std
    ),
    critic = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],  # critic 可以更大！
        obs_normalization=True,       # critic 独立的 normalizer
    ),
    obs_groups = {
        "actor": ("actor",),
        "critic": ("critic",),
    },
)
```

这个解耦对 privileged learning 的意义是：**critic 不仅可以看到更多 observation，还可以用更大的网络来处理这些额外信息。** 典型配置：actor 用 [256, 128, 64] 的小 MLP（因为要部署），critic 用 [512, 256, 128] 的大 MLP（反正不部署，且输入维度更高需要更大容量）。如果是视觉任务，critic 甚至可以用 `RslRlCNNModelCfg`——CNN 处理 privileged 的 depth/height 信息，而 actor 用纯 MLP 处理低维 proprioception。

⚠️ **迁移陷阱：旧版 checkpoint 无法直接加载到新版配置。** 如果你有旧版 RSL-RL 训练的 checkpoint，`state_dict` 的 key 名发生了变化（旧版 `actor_critic.actor.0.weight` → 新版 `actor.model.0.weight`）。迁移方法：写一个 key-mapping 脚本，或者重新训练。

⚠️ **迁移陷阱：`obs_normalization` 在新版中是 per-model 的。** 旧版的 `actor_obs_normalization` 和 `critic_obs_normalization` 合并成了每个 model 自己的 `obs_normalization` 参数。actor 和 critic 各自维护独立的 running mean/std buffer——这意味着 critic 的 normalizer 统计量包含 privileged 维度的信息，而 actor 的不包含。**导出 ONNX 部署时，必须使用 actor 的 normalizer，不能用 critic 的。**

### 维度一致性检查

配置完成后，第一件事是打印 actor 和 critic 的 observation 维度，确认 critic 维度 ≥ actor 维度：

```python
# mjlab 中的维度检查
env = gym.make("Mjlab-Velocity-Rough-Unitree-Go1-v0")
obs = env.reset()
print(f"Actor obs dim: {obs['actor'].shape[-1]}")
print(f"Critic obs dim: {obs['critic'].shape[-1]}")
# 预期输出示例：
#   Actor obs dim: 48    (proprio 36 + command 3 + height_scan 187 ≈ 226)
#   Critic obs dim: 274  (actor 226 + foot_height 4 + foot_air_time 4
#                         + foot_contact 4 + foot_contact_forces 12 ≈ 274)
# 关键：critic dim > actor dim

# Isaac Lab 中的维度检查
env = gym.make("Isaac-Velocity-Rough-Anymal-C-v0")
obs_dict = env.observation_manager.compute()
print(f"Policy obs dim: {obs_dict['policy'].shape[-1]}")
print(f"Critic obs dim: {obs_dict['critic'].shape[-1]}")
```

如果 actor 和 critic 维度相等，说明 privileged terms 没有生效——最常见的原因是 key 拼写错误（新增而非覆盖）或 obs_groups routing 配错了。

**逐 term 维度审计工具：** 上面的检查只验证了总维度，但不能告诉你"每个 term 贡献了多少维度"。以下工具可以逐个 term 打印维度，快速定位配置错误：

```python
# obs_audit.py — 逐 term 维度审计
def audit_obs_terms(env, framework="mjlab"):
    """
    打印每个 observation group 中每个 term 的维度和属性。
    用于在训练前系统化检查配置是否正确。
    """
    if framework == "mjlab":
        obs_manager = env.observation_manager
        groups = obs_manager.group_obs_terms
    else:  # isaac_lab
        obs_manager = env.observation_manager
        groups = {}
        for group_name in obs_manager.active_terms:
            groups[group_name] = obs_manager.active_terms[group_name]

    for group_name, terms in groups.items():
        total_dim = 0
        print(f"\n{'='*60}")
        print(f"Group: {group_name}")
        print(f"{'='*60}")
        print(f"{'Term':<30} {'Dim':>5} {'Noise':>8} {'Deployable':>12}")
        print(f"{'-'*60}")

        for term_name, term_cfg in terms.items():
            # 获取该 term 的输出维度
            sample = term_cfg.func(env, **term_cfg.params)
            dim = sample.shape[-1]
            total_dim += dim

            # 检查是否有噪声
            has_noise = hasattr(term_cfg, 'noise') and term_cfg.noise is not None
            noise_str = f"±{term_cfg.noise.n_max:.2f}" if has_noise else "None"

            # 简单的部署可得性启发检查
            priv_keywords = ["contact_force", "friction", "terrain_height",
                           "body_mass", "restitution"]
            is_priv = any(kw in term_name.lower() for kw in priv_keywords)
            deploy_str = "⚠️ PRIV" if is_priv else "✅ OK"

            print(f"{term_name:<30} {dim:>5} {noise_str:>8} {deploy_str:>12}")

        print(f"{'-'*60}")
        print(f"{'TOTAL':<30} {total_dim:>5}")

    # 交叉检查
    if "actor" in groups and "critic" in groups:
        actor_dim = sum(t.func(env, **t.params).shape[-1]
                       for t in groups["actor"].values())
        critic_dim = sum(t.func(env, **t.params).shape[-1]
                        for t in groups["critic"].values())
        diff = critic_dim - actor_dim
        print(f"\nPrivileged dims (critic - actor): {diff}")
        if diff <= 0:
            print("❌ WARNING: critic 没有额外的 privileged 信息！")
```

运行示例输出：

```text
============================================================
Group: actor
============================================================
Term                            Dim    Noise   Deployable
------------------------------------------------------------
base_lin_vel                      3    ±0.50       ✅ OK
base_ang_vel                      3    ±0.20       ✅ OK
projected_gravity                 3    ±0.05       ✅ OK
joint_pos                        12    ±0.01       ✅ OK
joint_vel                        12    ±1.50       ✅ OK
actions                          12     None       ✅ OK
command                           3     None       ✅ OK
height_scan                     187    ±0.10       ✅ OK
------------------------------------------------------------
TOTAL                           235

============================================================
Group: critic
============================================================
Term                            Dim    Noise   Deployable
------------------------------------------------------------
...（继承 actor 所有 terms，但无噪声）...
foot_contact_forces              12     None   ⚠️ PRIV
friction_coeffs                   1     None   ⚠️ PRIV
body_mass_offset                  1     None   ⚠️ PRIV
------------------------------------------------------------
TOTAL                           249

Privileged dims (critic - actor): 14
```

这个审计工具应该在每次修改 observation 配置后运行一次。把它放在项目的 `scripts/` 目录下，作为训练前的标准检查步骤。

### POMDP 视角：为什么 asymmetric AC 从理论上说是合理的

从理论角度看，asymmetric AC 处理的是 POMDP（Partially Observable MDP）问题。真实机器人的 actor 只看到 observation $o = h(s)$，而非完整状态 $s$。多个不同的真实状态可能产生相同的 observation——策略必须在这种信息不完整下决策。

critic 看到的 privileged information 相当于让 critic 在 MDP（而非 POMDP）上估值——因为 privileged info 让 critic 接近"完整状态"。这解释了为什么 critic 的 value 估计更准确：它在做一个"更简单"的估值问题（状态接近完全可观）。actor 仍然在 POMDP 上行动——它必须从不完整 observation 中做出好决策。

这个理论视角给出了一个重要的工程指导：**critic 的 privileged 输入应该尽可能接近完整状态。** 如果 critic 的输入仍然是 POMDP（只看了部分 privileged 信息），value 估计的改进会打折扣。当然，"完整状态"在实践中不存在——但把所有 4 类 privileged 信号中的前 3 类（环境参数 + 接触信息 + 全局感知）都给 critic，通常就足够好了。

另一个从 POMDP 理论得到的洞察是：如果 actor 的 observation function $h(s)$ 对于控制决策来说是"足够信息的"（即 $o$ 包含了选择最优行动所需的所有信息），那么 asymmetric AC 不会比 symmetric AC 好多少——因为 actor 已经有足够信息了。asymmetric AC 的真正价值体现在 actor 的 observation **不足以唯一确定最优行动**时——此时 critic 的额外信息帮助减少 value 估计噪声，让 policy gradient 方向更准确。

### Isaac Lab 中的 observation 拓展：如何添加自定义 privileged term

在 Isaac Lab 中，如果你想为 critic 添加一个框架没有内置的 privileged term（比如物体精确位姿），需要自定义一个 observation function：

```python
# Isaac Lab 中自定义 privileged observation term
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm

def object_pose_world(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """获取物体在世界系中的位姿（privileged，仿真直接读取）。"""
    asset = env.scene[asset_cfg.name]
    # 直接读取仿真状态——真机上无此接口
    pos = asset.data.root_pos_w  # (num_envs, 3)
    quat = asset.data.root_quat_w  # (num_envs, 4)
    return torch.cat([pos, quat], dim=-1)  # (num_envs, 7)

# 在 CriticCfg 中使用
class CriticCfg(ObsGroup):
    ...
    object_pose = ObsTerm(
        func=object_pose_world,
        params={"asset_cfg": SceneEntityCfg("target_object")},
    )
```

在 mjlab 中的对应写法类似——通过自定义 `ObservationTermCfg` 的 `func` 参数来定义新的 observation function。关键是确保这些自定义 term 只出现在 critic_terms 中，不泄漏到 actor_terms。

### 信息泄漏排查：15 项 Debug Checklist

信息泄漏不总是显式的。它可以以三种隐蔽形式出现：**显式泄漏**（actor_terms 直接包含不可部署 term）、**隐式泄漏**（自定义 term 的实现中暗中读取了仿真内部状态）、**评估泄漏**（play 模式中使用了 critic tensor 做判断）。以下 checklist 应在每次修改配置后执行：

```text
信息边界检查 Checklist
━━━━━━━━━━━━━━━━━━━━━━━

[ ] 1. actor_terms 逐项审查
      对每个 term 回答：部署时由什么传感器提供？
      如果答不出来 → 该 term 可能是 privileged，移到 critic

[ ] 2. critic_terms 检查 corruption 设置
      确认 critic group 的 enable_corruption=False

[ ] 3. obs_groups 映射检查
      打印 runner 的 obs_groups 配置
      确认 actor 指向 env 的 actor group、critic 指向 critic group

[ ] 4. 维度一致性检查
      打印 actor 和 critic observation 维度
      critic 维度应 >= actor 维度
      如果相等 → privileged terms 可能没有生效

[ ] 5. play 加载检查
      确认 play 模式只加载 actor 权重
      确认 play 的 env 配置关闭了 actor corruption

[ ] 6. 自定义 term 源码审查
      对每个自定义 term，检查其实现
      确认没有直接读取 env.sim.data 中的 privileged 字段

[ ] 7. contact 边界检查
      foot_contact_forces 是否只在 critic_terms 中？
      如果出现在 actor_terms → 除非有等价传感器，否则移除

[ ] 8. terrain 边界检查
      actor height_scan 是否有 noise？
      critic height_scan 是否更 clean？

[ ] 9. history 合理性检查（如使用 RMA）
      history 是否只加在了有意义的 terms 上？
      history_length 是否导致维度爆炸（< 500 维）？

[ ] 10. delay 合理性检查
       delay lag 换算成物理时间是否合理？

[ ] 11. teacher-student 边界检查（如适用）
       teacher 的 privileged input 是否存在 student 可观测代理？
       student 的训练 obs 是否严格等于部署 obs？

[ ] 12. 实验记录
       实验报告中是否显式记录了 actor 和 critic 的信息边界？

[ ] 13. observation normalization 状态检查
       actor 和 critic 的 running mean/std normalizer 是否分别初始化？
       从 checkpoint 恢复时是否确认 normalizer 状态一起加载？

[ ] 14. 新增 observation 维度后 normalizer 重置
       新增 term 后是否重新开始 normalization 统计？
       新维度的 mean/std 初始值是否合理（非零、非极大）？

[ ] 15. ONNX 导出时 normalizer 归属检查
       导出的是 actor 的 normalizer 而非 critic 的？
       normalizer 是否通过 `export_policy_as_onnx()` 烘焙进了模型？
```

### Observation Normalization：沉默的训练杀手

在 privileged learning 的工程实践中，observation normalization 是一个极容易被忽视但影响巨大的细节。RSL-RL 默认使用 **running mean/std 归一化**——在训练过程中持续追踪每个 observation 维度的均值和标准差，将输入归一化到近似零均值单位方差。这个看似简单的机制在 asymmetric AC 场景下有几个微妙但致命的工程影响。

**问题 1：Actor 和 Critic 的 normalizer 必须独立。** 因为 actor 和 critic 看到的 observation 维度不同（critic 多了 privileged 维度），它们的 running mean/std buffer 大小不同。RSL-RL 4.0 的解耦架构自然解决了这个问题（每个 model 有自己的 normalizer）。但在旧版或自定义实现中，如果 actor 和 critic 共享 normalizer，privileged 维度的统计量会污染 actor 维度的归一化。

**问题 2：新增 observation 维度后 normalizer buffer 未初始化。** 假设你在训练 5000 iterations 后决定给 critic 增加一个新的 privileged term（如 contact_forces）。如果不重新初始化 normalizer，新维度的 running mean 为 0、running std 为 1（或更糟，为 0）——而已有维度的统计量已经积累了 5000 iterations 的数据。这个统计量不一致会导致新维度的归一化不正确，critic 的 value 估计突然恶化。**解决方案：添加新 term 后必须重新开始训练，或至少重置 normalizer。**

**问题 3：Checkpoint 恢复时 normalizer 状态遗漏。** RSL-RL 的 `save_model()` 会保存 normalizer 状态，但如果你手动操作 state_dict（比如做 warm start 或跨框架迁移），很容易遗漏 normalizer 的 running mean/std。恢复后 normalizer 重新从零开始积累，导致前几百个 iteration 的归一化不准确——表现为 value loss 突然升高然后缓慢恢复。

**问题 4：AMP 判别器中的 normalization 对齐。** 如果你在 Ch10（模仿学习）中使用 AMP 判别器，判别器和 actor 必须共享同一个 observation normalizer。如果各自维护独立的 normalizer，判别器看到的"真实动作分布"和 actor 看到的"策略动作分布"会因归一化不一致而产生偏差——判别器可能仅凭归一化差异就能区分真假，而不是凭动作质量区分。这是 AMP 训练中最常见的沉默失败模式之一。

```python
# 检查 normalizer 状态的诊断代码
def check_normalizer_health(runner):
    """检查 actor 和 critic 的 normalizer 是否健康。"""
    actor_norm = runner.alg.actor.obs_normalizer
    critic_norm = runner.alg.critic.obs_normalizer

    # 检查 1：维度匹配
    assert actor_norm.running_mean.shape[0] == runner.env.obs_dict["actor"].shape[-1]
    assert critic_norm.running_mean.shape[0] == runner.env.obs_dict["critic"].shape[-1]

    # 检查 2：std 无零值（零 std 会导致除零 NaN）
    assert (actor_norm.running_var > 1e-8).all(), "Actor normalizer has zero-variance dims!"
    assert (critic_norm.running_var > 1e-8).all(), "Critic normalizer has zero-variance dims!"

    # 检查 3：count 一致（两者应该看到相同数量的 samples）
    print(f"Actor normalizer count: {actor_norm.count}")
    print(f"Critic normalizer count: {critic_norm.count}")

    # 检查 4：打印每个维度的 mean/std 范围
    print(f"Actor mean range: [{actor_norm.running_mean.min():.3f}, "
          f"{actor_norm.running_mean.max():.3f}]")
    print(f"Critic mean range: [{critic_norm.running_mean.min():.3f}, "
          f"{critic_norm.running_mean.max():.3f}]")
```

### 信息边界审计报告模板

在正式开始训练之前，养成为每个新项目撰写一份简短的"信息边界审计报告"的习惯。这份报告是你和合作者之间的共识文档——它明确记录了每个 observation term 的角色归属和部署方案，避免后续出现"谁把这个 term 加进 actor 的"这类责任不清的问题。

```text
信息边界审计报告
━━━━━━━━━━━━━━━━━

项目名称：Go1 Rough Terrain Velocity
审计日期：2026-05-20
审计人：[你的名字]

═══ Actor Observation Group ═══
1. base_lin_vel (3维) - 部署来源: IMU + 状态估计 → 通过
2. base_ang_vel (3维) - 部署来源: IMU → 通过
3. projected_gravity (3维) - 部署来源: IMU 姿态估计 → 通过
4. joint_pos (12维) - 部署来源: 关节编码器 → 通过
5. joint_vel (12维) - 部署来源: 编码器差分 → 通过
6. last_action (12维) - 部署来源: 策略内部记忆 → 通过
7. command (3维) - 部署来源: 上层规划 → 通过
8. height_scan (187维) - 部署来源: 深度相机 → [待确认传感器安装]
   Actor总维度: 235维

═══ Critic Extra Terms ═══
9. height_scan_clean (187维) - 仅训练用 → 通过
10. foot_height (4维) - 仅训练用 → 通过
11. foot_air_time (4维) - 仅训练用 → 通过
12. foot_contact (4维) - 仅训练用 → 通过
13. foot_contact_forces (12维) - 仅训练用 → 通过
   Critic总维度: 446维 (actor 235 + extra 211)

═══ 配置验证 ═══
obs_groups: actor→("actor",), critic→("critic",) → 通过
Corruption: actor=True, critic=False → 通过
维度检查: critic(446) > actor(235) → 通过
Play检查: 只加载actor, corruption=False → 通过

═══ 结论 ═══
信息边界正确。actor可安全部署，
前提条件：height_scan需要等价深度传感器。
未解决风险：深度传感器与仿真raycast的语义差异
（噪声模式、视野范围、更新频率）。
```

这份报告看起来简单，但它的价值在于**把隐式假设变成显式记录**。很多 sim-to-real 失败的根因是"某个 term 的部署可用性没人认真想过"——审计报告强制你为每个 term 写出部署来源，有效防止这类疏忽。

### 诊断矩阵

当训练或部署出现异常时，以下矩阵帮助你定位是否是信息边界问题：

| 现象 | 优先检查 | 允许情况 | 修复方向 |
|------|---------|---------|---------|
| sim 好但 play 差 | actor corruption / push event 配置 | play 关闭 corruption 是正常的 | 对比 train/play 配置 |
| sim 好但真机差 | actor_terms 中的 privileged 信号 | — | 移除不可部署 term |
| value loss 长期偏高 | critic_terms 是否缺少 privileged 信号 | — | 增强 critic 输入 |
| student loss 不降 | teacher 输入中是否有不可蒸馏信息 | — | 限制 teacher privilege |
| history 加大后无改善 | history terms 选择是否合理 | — | 只给 proprioception 加 history |
| critic 和 actor obs 维度相同 | obs_groups routing 是否正确 | — | 检查 key 拼写和路由配置 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Python 字典展开时 key 拼写错误导致覆盖失败。** `critic_terms = {**actor_terms, "heigth_scan": ...}` 不会覆盖 `height_scan`，而是新增一个 key。critic 维度比预期多一倍 height_scan 的维度。自检方法：`assert len(critic_terms) == len(actor_terms) + N_privileged`。

⚠️ **编程陷阱：Isaac Lab 中忘记在 CriticCfg 中设 `enable_corruption = False`。** 如果 critic 也启用了 corruption，它看到的 privileged 信号也会有噪声，导致 value 估计方差增大。自检方法：打印 critic group 的 corruption 状态。

💡 **概念误区：认为"critic 维度越大越好"。** critic 的 privileged 输入不是越多越好——如果 privileged 信号维度太高而对 value 估计帮助不大（如全身 100+ 个关键点的完整位置），反而会增加 critic 的拟合难度和训练时间。应该选择对 value 估计帮助最大的 privileged 信号（通常是接触信息和地形信息）。

🧠 **思维陷阱：认为"Isaac Lab 和 mjlab 的配置可以直接复制粘贴"。** 虽然两个框架的逻辑相同，但 term 名称、函数接口和 sensor 配置方式不同。把 mjlab 的 `mdp.foot_contact` 直接写进 Isaac Lab 配置会报错。始终参考各框架自己的 mdp 模块。

### 练习

1. **[编程题]** 在 mjlab 的 velocity_env_cfg.py 中故意给 actor_terms 添加 `foot_contact_forces`，训练 500 iteration，记录 reward 曲线。然后移除该 term 重新训练 500 iteration。对比两条 reward 曲线，解释差异的原因。
2. **[审查题]** 用上面的 15 项 checklist 审查你的 velocity task 配置。对每个检查项给出"通过/不通过"和理由。
3. **[跨框架题]** 分别在 mjlab 和 Isaac Lab 中打印 actor/critic 的 observation 维度，并画一张表格对比。维度差异主要来自哪些 privileged terms？

---

上一节解决了"如何在框架中正确配置 asymmetric actor-critic"的问题。但 asymmetric AC 有一个局限——它只能帮助训练信号更干净，无法让 actor 间接获取 privileged 信息。当 actor 需要从 history 中推断环境参数时，就需要 estimator 网络——这正是下一节的主题。

## 9.4 Estimator 网络训练 ⭐⭐⭐

> **这一节解决什么问题**：详解 RMA 风格的 adaptation module（estimator 网络）的工程实现——它的输入/输出设计、训练时机、损失函数选择，以及如何在 mjlab 和 Isaac Lab 中接线。

### 动机：为什么 Asymmetric AC 不够

回顾 Ch07（PPO 训练管线）的核心结论：PPO 的训练效率取决于 advantage 估计的质量。Advantage $\hat{A}_t = R_t - V(s_t)$ 中，$V(s_t)$ 由 critic 提供。Ch07 中我们讨论了 critic 网络大小和学习率对 value 估计精度的影响。本节在 Ch07 的基础上引入一个新的维度：critic 的输入信息量——即使 critic 网络足够大，如果它看到的信息不完整，value 估计仍然会有噪声。

Asymmetric AC（9.1 节）通过给 critic 更多信息解决了 value 估计的精度问题。但 actor 本身仍然只看部署可得信息——如果这些信息不足以支撑控制决策（例如在未知摩擦的粗糙地形上行走，actor 不知道当前地面有多滑），actor 的策略上限就会受到信息瓶颈的限制。

这个瓶颈的跨领域类比：考试时老师（critic）能看到标准答案帮你批改试卷（更准确的分数反馈），但你（actor）还是只能用课本上的知识来答题。如果考试内容超出了课本范围（比如需要知道当前地面的摩擦系数），老师的标准答案帮助你提高答题水平，但无法帮你获取考试中需要但课本中没有的知识。你需要一个额外的"参考资料本"——这就是 estimator 网络的角色：从你已有的信息（proprioception history）中推断出课本没有直接告诉你的知识（环境参数 latent）。

如果不用 estimator 会怎样？actor 只能学习一种"平均策略"——在所有可能的摩擦系数下都还行但在任何特定摩擦下都不是最优的策略。这就像一个不知道考试科目的学生只能做通识复习——考任何科目都能及格但都拿不到高分。estimator 让 actor 能在运行时"推断"当前环境参数，从而切换到针对该参数的最优行为。

### RMA 的两阶段架构

RMA（Rapid Motor Adaptation）的完整架构分为两个训练阶段：

**阶段一：训练 base policy + environment factor encoder。** Base policy 的输入是 proprioception $o_t^{\text{prop}}$ + 环境因子 latent $z_t$。Environment factor encoder 从仿真中读取 privileged 环境参数 $e_t$（摩擦、质量、阻尼等）并编码为低维 latent $z_t = f_\phi(e_t)$。两者在 PPO 中同时训练——encoder 作为 actor 网络的一部分参与 policy gradient 更新。

$$\pi_{\text{base}}(a_t \mid o_t^{\text{prop}}, z_t) \quad \text{where} \quad z_t = f_\phi(e_t)$$

**阶段二：训练 adaptation module。** 阶段一结束后，冻结 base policy 和 encoder。Adaptation module $g_\psi$ 从 proprioception history $\{o_{t-k}^{\text{prop}}, \ldots, o_t^{\text{prop}}\}$ 中估计 latent $\hat{z}_t$。训练目标是最小化估计 latent 和真实 latent 之间的 MSE：

$$\mathcal{L}_{\text{adapt}} = \mathbb{E}\left[\| g_\psi(o_{t-k:t}^{\text{prop}}) - f_\phi(e_t) \|^2\right]$$

部署时，环境参数 $e_t$ 不可得，adaptation module 用 proprioception history 产生 $\hat{z}_t$，代替真实 $z_t$ 输入 base policy。

### 为什么用 latent 而不是直接预测物理参数

一个自然的问题是：为什么 adaptation module 不直接预测摩擦系数、质量等物理参数的值，而是预测一个 latent encoding？

**原因一：物理参数的数值范围和单位不统一。** 摩擦系数在 [0.3, 1.5]，质量偏移在 [-2, +5] kg，关节阻尼在 [0.1, 5.0] Nm·s/rad。直接预测这些参数需要处理不同量纲、不同范围的多任务回归，损失函数的权重设置很棘手。

**原因二：并非所有物理参数对控制都同等重要。** latent encoding 是一种自适应的信息压缩——通过 policy gradient 训练的 encoder 会自动学习把对控制决策最重要的参数信息编码到 latent 中，忽略不重要的参数。如果你直接预测物理参数，adaptation module 会花等量精力在重要和不重要的参数上。

**原因三：latent 空间更容易学习。** encoder 输出的 latent 经过了 policy gradient 训练，其空间结构被优化为"对控制有用的表示"。adaptation module 只需要学会映射到这个已优化的空间，而不是从零学习一个从 history 到物理参数的复杂非线性映射。

> **本质洞察**：RMA 的 adaptation module 可以用 POMDP 的 belief state 来理解。真实机器人面对的是部分可观测的 MDP（POMDP）——环境参数是隐藏状态。从 observation history 中提取的 latent 就是对当前隐藏状态的 belief（信念状态）。理想的 belief 应该是充分统计量——包含 history 中对控制决策有用的所有信息。RMA 的 latent 是这个理想 belief 的参数化近似。

### Proprioception History 作为隐式环境估计器

为什么关节位置的历史序列包含环境参数信息？考虑一个四足机器人在光滑地面上行走：当脚底滑动时，PD 控制器的响应导致关节位置偏离期望值的方式与高摩擦地面不同——滑动会产生特征性的关节位置波动模式。如果策略能读取最近 5-10 步的关节位置序列，它可以从这些波动模式中推断出"当前地面摩擦较低"。

数学上，设关节状态序列为 $\{q_{t-k}, \ldots, q_t\}$，环境参数为 $\theta_{\text{env}}$。关节响应和环境参数之间存在函数关系：

$$q_t = f(q_{t-1}, a_{t-1}, \theta_{\text{env}}, \text{noise})$$

如果 $f$ 对 $\theta_{\text{env}}$ 的依赖足够强（即环境参数变化会显著影响关节响应），那么从 $\{q_{t-k}, \ldots, q_t\}$ 反推 $\theta_{\text{env}}$ 在理论上是可行的。MLP 或 1D CNN 能学会这种隐式的逆推关系。

但 proprioception history 有一个根本局限：它只能反映**已经发生过的**环境交互，不能预见**未来的**环境变化。对于慢变量（摩擦系数在整个 episode 内恒定），5-10 步 history 通常足够估计。对于快变量（前方突然出现的台阶），history 来不及反应——此时需要前向感知（深度相机）或 teacher-student 蒸馏。

### 三种在线适应范式：RMA 不是唯一选择

RMA 的显式 adaptation module 是当前最成熟的在线适应方案，但它不是唯一的。2021-2024 年间发展出了三种不同的技术路线，理解它们的差异对选型至关重要。

**范式 1：RMA（显式 Adaptation Module）**——Kumar et al. 2021。两阶段训练：Phase 1 训练 base policy + environment encoder，Phase 2 训练 adaptation module 从 proprioception history 预测 latent。部署时 adaptation module 以 10 Hz 运行（比 100 Hz 的 base policy 慢一个数量级），实现"分秒级适应"。优点：latent 空间可解释（对应摩擦、质量等物理参数），训练稳定。缺点：需要手动定义 latent 空间维度和目标参数。A-RMA（Kumar 2022）在此基础上增加了 Phase 3 fine-tuning，用于更复杂的双足 Cassie 机器人。

**范式 2：Causal Transformer（隐式 Attention 编码）**——Radosavovic et al., Science Robotics 2024。核心思路完全不同：不显式训练 adaptation module，而是用一个 causal transformer 直接把完整的观测历史编码成策略输入。transformer 的 attention 机制自动学会"关注历史中与当前环境参数相关的时间步"——相当于隐式完成了 system identification。优点：不需要预定义要适应什么参数，也不需要两阶段训练。缺点：transformer 的计算量远大于 MLP adaptation module，部署时推理延迟更高。

**范式 3：TTT（Test-Time Training）**——在线更新梯度。不同于前两者的"冻结权重部署"，TTT 在部署时持续用最新的观测数据更新网络权重。优点：能适应训练期间从未见过的环境变化。缺点：需要在边缘设备上运行反向传播，计算需求高且可能导致灾难性遗忘。

| 范式 | 代表工作 | 训练阶段 | 部署计算成本 | 适应速度 | 适用场景 |
|------|---------|---------|------------|---------|---------|
| RMA | Kumar 2021 | 两阶段 | 低（MLP 前向） | 0.1-1 秒 | 已知要适应的参数少（<10 维） |
| Causal Transformer | Radosavovic 2024 | 单阶段 | 中（attention） | 即时 | 不确定需要适应什么 |
| TTT | Sun et al. 2024 | 单阶段+在线 | 高（反向传播） | 持续 | 环境持续变化 |

**双重解读：System Identification vs Representation Learning**

这三种范式可以从两个完全不同的视角来理解——这是同一个问题的两种表述：

**视角 A（System Identification）：** 环境有一组未知参数 $\theta$（摩擦、质量、地形），策略需要先"辨识"这些参数，然后根据辨识结果调整行为。RMA 的 adaptation module 就是一个显式的 system identifier：从历史数据估计 $\hat{\theta}$，然后把 $\hat{\theta}$ 传给 policy。这个视角强调"先估计参数，再根据参数决策"的两步范式。

**视角 B（Representation Learning）：** 环境的真实参数 $\theta$ 不重要，重要的是学到一个"对控制有用的环境表示" $z$。$z$ 不需要对应任何物理参数——它只需要编码"哪些行为在当前环境中有效"的信息。Causal transformer 天然属于这个视角：attention 机制学到的不是"摩擦系数是 0.7"，而是"在这种历史模式下应该怎样迈步"。

这两个视角的工程含义不同：如果你走视角 A，latent 维度应该等于你想辨识的参数数量（4-16 维）；如果你走视角 B，latent 维度可以更大（32-128 维），因为它不受物理参数数量约束。RMA 原始论文实际上是混合了两种视角——它训练 encoder 把物理参数映射到 latent，但 latent 并不保证与物理参数一一对应。

> **本质洞察：** RMA 之所以在四足 locomotion 中效果好，不是因为它精确辨识了物理参数，而是因为 locomotion 任务中的"有用环境表示"恰好低维（摩擦和质量的变化可以用 4-8 维 latent 充分编码）。当任务更复杂（人形全身运动、loco-manipulation）时，causal transformer 的灵活性可能更适合——它不预设 latent 的结构，让 attention 自由发现有用的历史模式。

对于本教材的目标读者（PhD 学生做四足/人形 locomotion），**推荐从 RMA 入手**（工程成熟、可解释、计算轻量），在确认 RMA 不够用时再考虑 causal transformer。

### Adaptation Module 的工程参数

| 参数 | 典型值 | 选择依据 |
|------|--------|---------|
| history_length | 5-20 步 | 太短则估计不稳定，太长则维度爆炸 |
| latent 维度 | 4-16 维 | 从小开始实验，latent 太大训练不稳定 |
| MLP 结构 | [128, 64] 或 [256, 128] | 输入维度 = proprio_dim × history_length |
| 训练数据 | teacher rollout 100k-1M 帧 | 需要覆盖多种 DR 参数组合 |
| 损失函数 | MSE（latent 空间） | 可选加 cosine similarity 正则 |

**维度计算示例：** 假设 proprioception 维度为 36（12 joint_pos + 12 joint_vel + 12 last_action），history_length = 10，则 adaptation module 输入维度为 $36 \times 10 = 360$。如果 latent 维度为 8，MLP 结构为 [128, 64]，总参数量为 $360 \times 128 + 128 \times 64 + 64 \times 8 = 46080 + 8192 + 512 = 54784$，约 55K 参数。这是一个很小的网络——部署时推理延迟可以忽略。

**完整的 Adaptation Module 网络定义：**

以下是 RMA 中 adaptation module 和 environment encoder 的完整 PyTorch 实现。理解每一行对后续的训练和调试至关重要。

```python
import torch
import torch.nn as nn

class EnvironmentEncoder(nn.Module):
    """
    Phase 1 训练：把 privileged 物理参数编码为 latent。
    只在仿真训练时使用，部署时不需要。
    """
    def __init__(self, privileged_dim: int, latent_dim: int):
        super().__init__()
        # privileged_dim: 输入维度（如 friction + mass + restitution = 3）
        # latent_dim: 输出维度（如 8）
        self.encoder = nn.Sequential(
            nn.Linear(privileged_dim, 64),
            nn.ELU(),
            nn.Linear(64, 32),
            nn.ELU(),
            nn.Linear(32, latent_dim),
        )

    def forward(self, privileged_obs: torch.Tensor) -> torch.Tensor:
        """输入 privileged 物理参数，输出 latent 向量。"""
        return self.encoder(privileged_obs)


class AdaptationModule(nn.Module):
    """
    Phase 2 训练：从 proprioception history 预测 latent。
    部署时替代 EnvironmentEncoder，实现 online adaptation。
    """
    def __init__(self, proprio_dim: int, history_length: int, latent_dim: int):
        super().__init__()
        input_dim = proprio_dim * history_length  # 展平后的输入维度
        self.adapter = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, latent_dim),
        )
        self.history_length = history_length
        self.proprio_dim = proprio_dim

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        """
        输入 proprioception history，输出 latent 估计。

        Args:
            history: shape (batch, history_length, proprio_dim)
                     最新帧在 history[:, -1, :]
        Returns:
            latent: shape (batch, latent_dim)
        """
        # 展平 history: (batch, history_length * proprio_dim)
        flat = history.reshape(history.shape[0], -1)
        return self.adapter(flat)


class RMAPolicy(nn.Module):
    """
    完整的 RMA 策略：base policy + adaptation module。
    base policy 输入 = proprio + latent → action。
    """
    def __init__(self, proprio_dim, latent_dim, action_dim, hidden_dims=[256, 128]):
        super().__init__()
        layers = []
        input_dim = proprio_dim + latent_dim  # proprio 和 latent 拼接
        for h in hidden_dims:
            layers.extend([nn.Linear(input_dim, h), nn.ELU()])
            input_dim = h
        layers.append(nn.Linear(input_dim, action_dim))
        self.mlp = nn.Sequential(*layers)

    def forward(self, proprio: torch.Tensor, latent: torch.Tensor) -> torch.Tensor:
        """输入当前 proprio 和 latent 估计，输出 action。"""
        x = torch.cat([proprio, latent], dim=-1)
        return self.mlp(x)
```

**训练阶段与推理阶段的数据流对比：**

```text
训练 Phase 1（有 privileged info）:
  privileged_obs ─→ EnvironmentEncoder ─→ latent_gt
  proprio ─────────────────────────────┐
  latent_gt ───────────────────────────┼─→ RMAPolicy ─→ action
                                       │
训练 Phase 2（离线监督学习）:
  proprio_history ─→ AdaptationModule ─→ latent_pred
  MSE(latent_pred, latent_gt) ─→ 反向传播更新 AdaptationModule

部署（无 privileged info）:
  proprio_history ─→ AdaptationModule ─→ latent_pred
  proprio ─────────────────────────────┐
  latent_pred ─────────────────────────┼─→ RMAPolicy ─→ action
```

注意训练和部署时 RMAPolicy 的输入来源不同：训练时 latent 来自 EnvironmentEncoder（ground truth），部署时 latent 来自 AdaptationModule（估计值）。RMAPolicy 本身的权重在两个阶段是冻结的（Phase 2 只更新 AdaptationModule）。

### 训练时机：在线 vs 离线

| 维度 | 在线训练（与 policy 同步） | 离线训练（policy 冻结后） |
|------|-------------------------|------------------------|
| 数据分布 | 自动适应 policy 诱导的状态分布 | 固定在 teacher rollout 的分布 |
| 工程复杂度 | 高（需要修改训练循环） | 低（标准监督学习） |
| 训练稳定性 | 可能与 policy 更新竞争 | 稳定（固定目标） |
| 框架支持 | 需自定义 | RSL-RL DistillationRunner 支持 |
| 推荐场景 | 研究探索 | 生产部署 |

对于大多数项目，**离线训练**是更安全的选择：先训练好 teacher（asymmetric AC），收集 teacher rollout 数据，然后单独训练 adaptation module。这种方式的工程流程更清晰，不需要修改 PPO 训练循环。

### 离线训练 Adaptation Module 的完整代码

以下是在 mjlab/Isaac Lab 项目中离线训练 adaptation module 的典型代码结构。这个流程分为两步：（1）用已训练的 teacher 收集数据；（2）监督训练 adaptation module。

**Step 1：数据收集**

```python
# collect_adaptation_data.py
# 用已训练的 teacher 在多种 DR 参数下 rollout，保存 (history, latent) 对

import torch

def collect_adaptation_dataset(env, teacher_policy, encoder, cfg):
    """
    Args:
        env: 向量化环境（mjlab 或 Isaac Lab）
        teacher_policy: 已训练的 teacher actor
        encoder: teacher 的 environment encoder（冻结权重）
        cfg: 包含 history_length, num_steps 等
    Returns:
        dataset: dict with 'histories' [N, history_len, proprio_dim]
                 and 'latents' [N, latent_dim]
    """
    history_buf = torch.zeros(
        env.num_envs, cfg.history_length, cfg.proprio_dim,
        device=env.device
    )
    all_histories = []
    all_latents = []

    obs = env.reset()
    for step in range(cfg.num_steps):
        # 提取 proprioception（部署可得）
        proprio = obs[:, :cfg.proprio_dim]
        # 提取 privileged（仅仿真可得）
        privileged = obs[:, cfg.proprio_dim:]

        # 更新 history buffer（FIFO，最新帧在最后）
        history_buf = torch.roll(history_buf, shifts=-1, dims=1)
        history_buf[:, -1, :] = proprio

        # 用 encoder 计算 ground truth latent
        with torch.no_grad():
            gt_latent = encoder(privileged)

        # 只在 buffer 填满后才收集
        if step >= cfg.history_length:
            all_histories.append(history_buf.clone())
            all_latents.append(gt_latent.clone())

        # teacher 执行动作
        with torch.no_grad():
            action = teacher_policy(obs)
        obs, _, dones, _ = env.step(action)

        # 重置的环境清空 history
        history_buf[dones] = 0.0

    return {
        'histories': torch.cat(all_histories, dim=0),
        'latents': torch.cat(all_latents, dim=0),
    }
```

注意 history buffer 的 FIFO 管理：`torch.roll` 把所有帧向前移一位，最新帧放在最后。这个顺序约定必须与 adaptation module 推理时的约定一致——如果训练时最新帧在最后而推理时最新帧在最前，module 学到的时间模式将完全错乱。

**Step 2：监督训练**

```python
# train_adaptation_module.py
from torch.utils.data import TensorDataset, DataLoader

def train_adaptation_module(module, dataset, cfg):
    """标准监督训练循环。"""
    ds = TensorDataset(dataset['histories'], dataset['latents'])
    loader = DataLoader(ds, batch_size=cfg.batch_size, shuffle=True)
    optimizer = torch.optim.Adam(module.parameters(), lr=cfg.lr)

    for epoch in range(cfg.num_epochs):
        total_loss = 0.0
        for hist_batch, latent_batch in loader:
            # hist_batch: [B, history_len, proprio_dim]
            # 展平为 [B, history_len * proprio_dim] 输入 MLP
            hist_flat = hist_batch.reshape(hist_batch.shape[0], -1)
            pred_latent = module(hist_flat)
            loss = torch.nn.functional.mse_loss(pred_latent, latent_batch)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        avg_loss = total_loss / len(loader)
        if epoch % 10 == 0:
            print(f"Epoch {epoch}: MSE = {avg_loss:.6f}")
```

这个流程在 mjlab 和 Isaac Lab 中完全通用——区别仅在于 env 的 API 和 obs 维度的解析方式。在 mjlab 中 obs 来自 `obs_groups["critic"]`，在 Isaac Lab 中来自 `CriticCfg` 定义的 observation group。

> **本质洞察：** Adaptation module 的训练本质上是一个时序回归问题——从过去 k 步的低维信号（proprioception history）预测当前的高维隐状态（environment latent）。它的精度上限取决于 proprioception history 中是否真的包含了足够的环境信息。如果环境参数（如摩擦系数）完全不影响 proprioception 的时间演变模式，那么无论网络多大、数据多多，adaptation module 都无法估计这个参数。这就是为什么选择哪些参数放入 latent 空间比优化网络结构更重要。

### 损失函数选择与变体

标准的 adaptation module 损失是 MSE，但实践中有几种变体值得了解：

| 损失函数 | 公式 | 优点 | 缺点 | 推荐场景 |
|---------|------|------|------|---------|
| MSE | $\|g - f\|^2$ | 简单、稳定 | 被高方差维度主导 | latent 维度均匀时 |
| Normalized MSE | z-score 后 MSE | 各维度等权 | 需要在线统计量 | latent 方差不均匀时 |
| Cosine Similarity | $1 - \cos(g, f)$ | 只关注方向 | 忽略幅度信息 | 方向比幅度重要时 |
| 混合 | $\alpha \cdot \text{MSE} + (1-\alpha) \cdot \text{cos}$ | 平衡 | 多一个超参 | 不确定时 |

**MSE Loss（标准选择）：**

$$\mathcal{L}_{\text{MSE}} = \mathbb{E}\left[\| g_\psi(o_{t-k:t}^{\text{prop}}) - f_\phi(e_t) \|^2\right]$$

MSE 对所有 latent 维度一视同仁。如果某些维度的方差远大于其他维度，MSE 会被高方差维度主导。

**Normalized MSE：** 先对 encoder 输出做 z-score 归一化（减均值除标准差），然后计算 MSE。这确保所有 latent 维度被等权重优化。

**Cosine Similarity Loss：** 关注 latent 向量的方向而非幅度：

$$\mathcal{L}_{\text{cos}} = 1 - \frac{g_\psi(o_{t-k:t}^{\text{prop}}) \cdot f_\phi(e_t)}{\|g_\psi\| \cdot \|f_\phi\|}$$

适用于 latent 的方向比幅度更重要的情况。

**混合损失：** 同时使用 MSE 和 cosine similarity：$\mathcal{L} = \alpha \cdot \mathcal{L}_{\text{MSE}} + (1-\alpha) \cdot \mathcal{L}_{\text{cos}}$，典型 $\alpha = 0.5$。

在实践中，标准 MSE 加上 latent 空间的 z-score 归一化通常就足够了。更复杂的损失函数只在 latent 空间结构有特殊需求时才有价值。

### DAgger：解决 Compounding Error 的在线蒸馏

标准的 BC 蒸馏有一个根本性的理论缺陷：compounding error。Student 在训练时看到的是 teacher 诱导的状态分布 $d^{\pi_{\text{teacher}}}$，但部署时 student 自己的行为会诱导不同的状态分布 $d^{\pi_{\text{student}}}$。如果两个分布差异较大，student 在 $d^{\pi_{\text{student}}}$ 中的状态上可能表现很差——因为它从没在这些状态上被训练过。

这个问题的跨领域类比是驾校教学：如果你只在教练坐旁边时练习（teacher 分布），第一次独自上路（student 分布）时可能会慌张——因为教练在旁边时你不会犯的错误（如走错车道），在独自驾驶时可能发生，而你没有被训练过如何从这些错误中恢复。

DAgger（Dataset Aggregation, Ross et al. 2011）的解决方案是：

```text
DAgger 流程：
1. 用 teacher rollout 初始化数据集 D = {(s, a_teacher)}
2. for round i = 1, 2, ...:
   a. 训练 student 在 D 上做 BC
   b. 让 student 在环境中 rollout，收集状态序列 {s_1, s_2, ...}
   c. 对这些状态查询 teacher 的动作：{(s_t, a_teacher(s_t))}
   d. 把新数据加入 D（数据聚合）
3. 返回最终 student
```

DAgger 的关键在于步骤 2c：在 **student 诱导的状态分布**上收集 teacher 的标签。这弥合了训练分布和测试分布之间的差距。代价是需要多轮 teacher rollout 和 student 训练的交替执行。

在 mjlab 和 Isaac Lab 中实现 DAgger 需要修改标准训练循环。RSL-RL 的 `DistillationRunner` 原生支持离线 BC 蒸馏，但不直接支持 DAgger 的在线交替执行。不过，Isaac Lab 生态中已有现成参考：`iit-DLSLab/basic-locomotion-isaaclab` 项目提供了完整的 DAgger 脚本（`scripts/dagger/train_dagger.py`），用于 camera-conditioned student 的训练。核心逻辑如下：

```python
# DAgger 的概念实现
for dagger_round in range(num_rounds):
    # 1. 用当前 student 做 rollout
    student_rollouts = rollout(env, student_policy, num_steps=10000)

    # 2. 对 student 访问过的状态查询 teacher 的动作
    teacher_labels = teacher_policy(student_rollouts.observations)

    # 3. 把 (student_obs, teacher_action) 加入数据集
    dataset.extend(student_rollouts.observations, teacher_labels)

    # 4. 在扩展后的数据集上训练 student
    train_bc(student_policy, dataset, epochs=5)
```

以下是一个更完整的 DAgger 实现，展示了工程中需要关注的细节——混合比例退火、数据集大小管理和 early stopping：

```python
# dagger_trainer.py — 完整 DAgger 训练循环
import torch
from collections import deque

class DAggerTrainer:
    """
    DAgger 训练器，支持混合比例退火和数据集大小限制。
    """
    def __init__(self, env, teacher, student, cfg):
        self.env = env
        self.teacher = teacher   # 冻结的 teacher 网络
        self.student = student   # 可训练的 student 网络
        self.cfg = cfg

        # 数据集：使用 deque 限制最大大小（防止内存溢出）
        self.dataset_obs = deque(maxlen=cfg.max_dataset_size)
        self.dataset_act = deque(maxlen=cfg.max_dataset_size)

        self.optimizer = torch.optim.Adam(
            student.parameters(), lr=cfg.lr,
        )

    def collect_round(self, round_idx: int):
        """
        一轮 DAgger 数据收集。

        混合比例 beta 控制谁在执行动作：
        - beta=1.0: 完全用 teacher（第 0 轮，初始化数据集）
        - beta=0.0: 完全用 student（后期，收集 student 分布）
        """
        # 退火 beta：前 3 轮全 teacher，之后线性衰减
        if round_idx < self.cfg.warmup_rounds:
            beta = 1.0
        else:
            progress = (round_idx - self.cfg.warmup_rounds) / \
                       max(self.cfg.num_rounds - self.cfg.warmup_rounds, 1)
            beta = max(0.0, 1.0 - progress)

        obs = self.env.reset()
        for step in range(self.cfg.steps_per_round):
            student_obs = obs["policy"]  # 部署可得

            # teacher 需要 privileged obs
            with torch.no_grad():
                teacher_obs = torch.cat(
                    [obs["policy"], obs["critic"]], dim=-1,
                )
                teacher_action = self.teacher(teacher_obs)
                student_action = self.student(student_obs)

            # 混合执行策略
            if torch.rand(1).item() < beta:
                exec_action = teacher_action
            else:
                exec_action = student_action

            # 记录标签：无论谁执行，标签都是 teacher 的动作
            self.dataset_obs.append(student_obs.cpu())
            self.dataset_act.append(teacher_action.cpu())

            obs, _, dones, _ = self.env.step(exec_action)

        print(f"Round {round_idx}: beta={beta:.2f}, "
              f"dataset_size={len(self.dataset_obs)}")

    def train_round(self, num_epochs: int = 5):
        """在当前数据集上训练 student（标准 BC）。"""
        obs_tensor = torch.cat(list(self.dataset_obs), dim=0)
        act_tensor = torch.cat(list(self.dataset_act), dim=0)
        dataset = torch.utils.data.TensorDataset(obs_tensor, act_tensor)
        loader = torch.utils.data.DataLoader(
            dataset, batch_size=self.cfg.batch_size, shuffle=True,
        )

        for epoch in range(num_epochs):
            total_loss = 0
            for obs_b, act_b in loader:
                obs_b = obs_b.to(self.cfg.device)
                act_b = act_b.to(self.cfg.device)
                pred = self.student(obs_b)
                loss = torch.nn.functional.mse_loss(pred, act_b)
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                total_loss += loss.item()
        return total_loss / len(loader)

    def run(self):
        """运行完整的 DAgger 流程。"""
        for round_idx in range(self.cfg.num_rounds):
            self.collect_round(round_idx)
            loss = self.train_round(num_epochs=self.cfg.bc_epochs_per_round)
            print(f"  BC loss after round {round_idx}: {loss:.6f}")
```

这个实现中有几个值得注意的工程决策：

> **工程提示：** DAgger 的核心优势是弥合 student/teacher 的分布偏移。如果 rollout 评估显示纯 BC 已经达标（student ≥ 85% teacher），可以跳过 DAgger 节省训练时间。

- **`deque(maxlen=...)` 限制数据集大小**：DAgger 每轮新增数据，不限制的话会导致内存溢出。典型最大值为 500K-2M 帧。超出后自动丢弃最旧的数据——这也是合理的，因为早期 round 的数据来自更差的 student，价值更低
- **混合比例 beta 退火**：前几轮完全用 teacher 执行（初始化一个好的数据集），之后逐渐切换到 student 执行（让 student 探索自己的分布）。`warmup_rounds=3` 是常用起点
- **BC epochs_per_round=5**：每轮只训练少量 epoch（避免过拟合当前数据集），然后立即进入下一轮收集

DAgger 通常比纯 BC 蒸馏多花 2-3 倍训练时间，但在 student-teacher 信息差较大时效果显著更好。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：history 拼接顺序错误。** 如果 history buffer 中最旧的帧排在前面，但 MLP 期望最新的帧在前面（或反之），adaptation module 学到的时间模式是反的。自检方法：在 adaptation module 输入中人为设置一个已知的时间序列，检查输出是否符合预期。

⚠️ **编程陷阱：latent 维度过大导致训练不稳定。** latent 维度从 4 开始实验是一个好的经验法则。如果从 16 或 32 开始，encoder 可能在 latent 空间中产生高方差的表示，adaptation module 难以拟合。先确认 4 维 latent 能工作，再逐步增大。

💡 **概念误区：认为"adaptation module 的 MSE loss 越低越好"。** MSE loss 衡量的是"history 能多好地预测 latent"，但最终目标是"使用估计 latent 时 policy 的 rollout 表现"。有时 MSE loss 中等但 rollout 表现好（estimator 捕捉了对控制最重要的信息），MSE loss 很低但 rollout 表现差（estimator 过拟合了 training distribution）。始终以 rollout 表现为最终判据。

🧠 **思维陷阱：认为"DAgger 一定比纯 BC 好"。** DAgger 的优势在于弥合分布偏移，但如果 teacher 和 student 的信息差很小（student 输入几乎可以复现 teacher 的行为），纯 BC 就足够了——DAgger 的额外轮次反而是浪费。只有当 rollout performance 指标（而非 imitation loss）不合格时才需要考虑 DAgger。

### 练习

1. **[计算题]** 假设 proprioception 维度为 48（16 joint_pos + 16 joint_vel + 16 last_action），history_length = 15，latent 维度为 12，adaptation module 使用 [256, 128] 的 MLP。计算 adaptation module 的总参数量和单次推理的 FLOPs。
2. **[设计题]** 为 Go1 rough terrain 任务设计 RMA 的 latent 空间。列出你认为 latent 应该编码的环境特性（如摩擦水平、质量偏移方向、地面倾斜度），并解释为什么这些特性对控制有用。
3. **[跨章综合题]** 回顾 Ch08（Domain Randomization）：DR 随机化了摩擦、质量、阻尼等参数。RMA 的 adaptation module 试图从 history 中估计这些参数的 latent。两者之间是什么关系——互补还是冲突？如果 DR 的范围设得太窄（真实世界参数超出了训练范围），adaptation module 会怎样？

---

至此，我们已经掌握了 asymmetric AC 的配置和 estimator 网络的训练。但当 actor 的输入模态发生根本变化（比如从低维 state 变成高维图像）时，这些工具就不够用了——你需要一个完整的多阶段 teacher-student 蒸馏管线。下一节通过精读 extreme-parkour 项目，展示工业级的三阶段蒸馏流程。

## 9.5 精读：extreme-parkour 三阶段管线 ⭐⭐⭐⭐

> **这一节解决什么问题**：通过精读 extreme-parkour 项目（Cheng et al., ICRA'24, `github.com/chengxuxin/extreme-parkour`），展示一个完整的三阶段 teacher-student 蒸馏管线——从 blind teacher 到 depth teacher 到 depth student。这是本章的工程高潮，把前面所有概念串联成一条可执行的管线。

### 动机：为什么需要三阶段

回顾 Ch05（Observation 设计）的核心原则：observation 的选择必须满足"部署可得性"约束——任何放入 actor/student 的信号，在真机上都必须有对应的传感器或估计器。当部署传感器是一个**深度相机**时，observation 的模态发生了根本变化：从低维向量（proprioception，~48 维）变成了高维图像（depth image，如 64×64=4096 维）。这个模态跨越不是简单的"多了几维"——它需要 CNN 编码器来处理，而 CNN 的训练效率远低于 MLP。

考虑这个任务：让一个四足机器人在极端地形上做跑酷——跳上台阶、跃过沟壑、攀爬斜坡。最终部署时，机器人只有 IMU + 关节编码器 + 前向深度相机。

如果直接端到端训练（depth image → action），CNN 需要从稀疏的 RL reward 中学习视觉特征——这极其低效（Ch18 将详细讨论）。如果只用一阶段 teacher-student（privileged state → teacher action → depth student imitation），teacher 和 student 之间的输入模态差异太大（低维 state vs 高维 depth image），蒸馏质量难以保证。

extreme-parkour 的解决方案是把蒸馏过程分成三个渐进阶段，每一阶段只跨越一个"小鸿沟"：

```
Stage 1: blind teacher    输入: proprioception + privileged terrain + privileged env params
                          输出: action
                          训练方式: RL (PPO)
                          目标: 学会"如何运动"

Stage 2: depth teacher    输入: proprioception + depth image + privileged env params
                          输出: action
                          训练方式: RL (PPO), 用 Stage 1 的 blind teacher 做 warm start
                          目标: 学会"从深度图中提取地形信息并运动"

Stage 3: depth student    输入: proprioception + depth image (仅部署可得)
                          输出: action
                          训练方式: 监督学习 (imitation of Stage 2)
                          目标: 去掉最后的 privileged env params
```

这个三阶段设计的类比是语言学习中的渐进式教学：Stage 1 是母语教学（给学生最好的条件学习核心能力），Stage 2 是双语教学（在核心能力基础上加入第二语言，即深度视觉），Stage 3 是纯外语教学（只用部署可得的信息）。每个阶段的"新挑战"只有一个维度，学生不会因为同时面对多个全新挑战而崩溃。

### 如果只用一阶段蒸馏会怎样

一个自然的想法是：直接训练一个 state-based teacher（看完美地形 + 环境参数），然后一步蒸馏到 depth student。这种做法面临两个问题：

**问题一：模态差异过大。** teacher 的输入是低维向量（~100 维 state），student 的输入是高维图像（~58×87 的 depth image）。从向量到图像的映射不是一个简单的函数——student 的 CNN 需要同时学会"如何从图像中提取特征"和"如何用这些特征模仿 teacher 的行为"。两个任务耦合在一起，优化困难。

**问题二：teacher 的行为可能不适合视觉输入。** state-based teacher 看到的是精确的 height_scan（前方 N 个点的精确高度），它可能学会了一种依赖精确空间分辨率的行为模式（比如"在第 7 个 scan 点高度 > 0.3m 时开始跳跃"）。但 depth student 看到的是整幅图像——它无法精确复现 teacher 基于特定 scan 点的行为。如果 teacher 先学会"从 depth 图像中做决策"（Stage 2），它的行为模式天然就是 depth-compatible 的。

### Stage 1 详解：Blind Teacher（本体感知 + 特权信息）

Stage 1 的 teacher 使用 RL（PPO + asymmetric actor-critic）训练。Teacher actor 的输入包括：

```python
# Stage 1: blind teacher 的 observation 配置（概念展示）
teacher_actor_obs = {
    # ——— 部署可得的 proprioception ———
    "joint_pos": ...,           # 关节位置
    "joint_vel": ...,           # 关节速度
    "last_action": ...,         # 上一拍动作
    "base_ang_vel": ...,        # IMU 角速度
    "projected_gravity": ...,   # 重力投影
    "command": ...,             # 速度命令

    # ——— privileged terrain info ———
    "height_scan_clean": ...,   # 无噪声的 height scan
    "foot_contact": ...,        # 精确接触状态

    # ——— privileged env params ———
    "friction_coeff": ...,      # 摩擦系数
    "body_mass_offset": ...,    # 质量偏移
    "motor_strength": ...,      # 电机强度参数
}
```

这里的"blind"指的是不使用任何视觉输入（depth/RGB）——teacher 通过 privileged 低维信息"看到"地形。训练目标是让 teacher 达到尽可能高的任务表现，不需要考虑部署约束。

**训练要点：**
- 使用 asymmetric AC（teacher 的 critic 可以有更多 privileged 信息）
- DR 配置应该涵盖部署时可能遇到的所有参数范围
- 训练时间可以更长（teacher 不需要满足实时推理约束）
- checkpoint 管理：保存多个训练阶段的 checkpoint，选最好的

```bash
# Stage 1 训练命令（mjlab 风格的概念命令）
uv run train Mjlab-Velocity-Rough-Unitree-Go1-Teacher \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 10000 \
    --agent.logger wandb
```

**extreme-parkour 的 ROA 创新：把两阶段压缩为一阶段。** 标准的 RMA 需要两阶段：先训练 base policy + encoder，再训练 adaptation module。extreme-parkour 的 Stage 1 使用了 **Regularized Online Adaptation（ROA）**——在一个训练阶段内同时训练 base policy 和 adaptation module。ROA 在 PPO 更新中加入一个辅助正则损失，鼓励 adaptation module 的输出与 environment encoder 的输出保持一致，但不冻结 encoder——两者协同演化。这把传统的两阶段训练时间压缩了约 30-40%。ROA 的工程代价是训练循环更复杂，且如果正则权重设置不当，adaptation module 和 encoder 可能"互相拉扯"导致训练不稳定。

### Stage 2 详解：Depth Teacher（加入深度输入 + 特权参数）

Stage 2 在 Stage 1 的基础上引入深度图像作为额外输入。Depth teacher 的输入包括：

```python
# Stage 2: depth teacher 的 observation 配置
depth_teacher_obs = {
    # ——— 部署可得的 proprioception（与 Stage 1 相同）———
    "joint_pos": ...,
    "joint_vel": ...,
    "last_action": ...,
    "base_ang_vel": ...,
    "projected_gravity": ...,
    "command": ...,

    # ——— depth image（部署可得）———
    "depth_image": ...,         # 前向深度相机图像，通过 CNN 编码

    # ——— privileged env params（训练辅助）———
    "friction_coeff": ...,      # 仍然保留
    "body_mass_offset": ...,
    "motor_strength": ...,
    # 注意：不再需要 height_scan_clean 和 foot_contact
    # 因为 depth image 已经提供了地形信息的替代
}
```

**关键工程决策：用 Stage 1 的 teacher 做 warm start。** Depth teacher 的网络权重不是从零初始化的，而是从 Stage 1 的 blind teacher 初始化（proprioception → MLP 部分的权重继承，CNN 编码器随机初始化）。这大幅加速了 Stage 2 的训练——policy 不需要从零学习运动技能，只需要学习如何利用 depth image 来替代 height_scan。

这个 warm start 的效果可以用一个反事实来理解：如果 Stage 2 从零训练，CNN 编码器在训练初期输出的是随机特征，policy 需要同时学习"如何运动"和"如何看"——两个任务耦合，训练极其缓慢。有了 Stage 1 的 warm start，policy 已经知道"如何运动"，只需要让 CNN 学会输出有意义的特征——这是一个更简单的优化问题。

**Stage 2 的网络架构：**

```
depth_image (58×87) → CNN encoder → latent (32维)
                                        ↓
proprioception (36维) → concat → MLP [256, 256, 128] → action (12维)
                  ↑
env_params (8维) ─┘
```

**Stage 2 的 MTS（Mixture of Teacher and Student）训练技巧。** extreme-parkour 在 Stage 2 的蒸馏阶段使用了一个精巧的 yaw-command 处理机制：student 的 heading command 不完全由自身决定，而是部分来自 teacher 的预测。具体来说，在训练时 student 的 yaw command 是 teacher 预测和 student 自身预测的混合：

$$\text{yaw}_{\text{train}} = \beta \cdot \text{yaw}_{\text{teacher}} + (1 - \beta) \cdot \text{yaw}_{\text{student}}$$

随着训练推进，$\beta$ 从 1.0 逐渐退火到 0.0。这确保了训练早期 student 不会因为 heading 预测错误而"走错方向"导致 rollout 质量极差（这会让蒸馏的 state 分布严重偏离 teacher 的分布）。MTS 的思想类似于 DAgger——在 student 分布偏移最严重的维度上给予 teacher 的指导。

**何时需要 MTS：** 当 student 和 teacher 的输入空间差异导致某些 output 维度的预测质量极不均匀时。在 extreme-parkour 中，proprioception → action 的映射 student 可以快速学好（因为输入相似），但 heading 方向的预测完全依赖 depth image 的 CNN 解读（因为 heading 需要知道前方地形），所以这个维度学得最慢，需要 MTS 额外扶持。

### Stage 3 详解：Depth Student（去掉 Privileged，仅部署可得）

Stage 3 用监督学习把 Stage 2 的 depth teacher 蒸馏到只看部署可得信息的 depth student。核心的变化是**移除 privileged env params**，用一个 adaptation module 替代。

```python
# Stage 3: depth student 的 observation 配置
depth_student_obs = {
    # ——— 部署可得的 proprioception ———
    "joint_pos": ...,
    "joint_vel": ...,
    "last_action": ...,
    "base_ang_vel": ...,
    "projected_gravity": ...,
    "command": ...,

    # ——— depth image（部署可得）———
    "depth_image": ...,

    # ——— proprioception history（用于 adaptation module）———
    "proprio_history": ...,     # 最近 K 步的 proprioception 拼接
    # 注意：没有任何 privileged 信号！
}
```

**蒸馏数据收集：** 用 Stage 2 的 depth teacher 在大量不同的 DR 参数下运行 rollout，收集 `(student_obs, teacher_action)` 对。数据多样性至关重要——teacher 需要在不同摩擦、不同质量、不同地形配置下运行。

**蒸馏损失：**

$$\mathcal{L}_{\text{distill}} = \mathbb{E}\left[\| \pi_{\text{student}}(o_t^{\text{deploy}}) - a_t^{\text{teacher}} \|^2\right]$$

这是一个标准的行为克隆损失。RSL-RL 的 `DistillationRunner` 原生支持这种训练模式——它不计算 advantage，不做 PPO clip，只最小化 imitation loss。`DistillationRunner` 的数据流与 `OnPolicyRunner`（Ch07 中详细介绍的标准 PPO runner）完全不同：

| 维度 | OnPolicyRunner (PPO) | DistillationRunner (BC) |
|------|---------------------|------------------------|
| 数据来源 | student 自己在 env 中 rollout | teacher 预收集的 rollout 数据 |
| 训练信号 | reward → value → advantage → policy gradient | teacher action → MSE loss → supervised gradient |
| 需要 critic？ | 是 | 否 |
| 需要 GAE？ | 是 | 否 |
| 数据重用 | mini-batch epochs（PPO 的 K epochs） | 可以无限重用（标准监督学习） |
| 训练终止 | iteration 数或 reward 阈值 | loss 收敛或 rollout 评估通过 |

**蒸馏数据收集的工程细节：**

数据收集不是简单地让 teacher 跑几个 episode 就完事。数据多样性直接决定蒸馏质量，需要系统化地覆盖 DR 参数空间：

```python
# 蒸馏数据收集的概念代码（mjlab 风格）
import numpy as np

# 定义 DR 参数的采样网格
friction_values = np.linspace(0.3, 1.5, 10)   # 10 种摩擦
mass_offsets = np.linspace(-2.0, 3.0, 8)       # 8 种质量偏移
terrain_levels = range(0, 6)                    # 6 种地形难度

distill_data = []
for friction in friction_values:
    for mass in mass_offsets:
        for terrain in terrain_levels:
            # 配置环境参数
            env_cfg.events.friction = friction
            env_cfg.events.mass_offset = mass
            env_cfg.curriculum.terrain_level = terrain

            # 用 teacher 做 rollout
            rollout = collect_rollout(
                env=env, policy=teacher_policy,
                num_steps=1000,  # 每种配置 1000 步
            )

            # 记录 (student_obs, teacher_action) 对
            distill_data.append({
                "student_obs": rollout.student_observations,
                "teacher_action": rollout.actions,
            })

# 保存数据——总计 10×8×6×1000 = 480K 帧
save_distill_data(distill_data, "stage3_distill_data/")
```

一个关键的工程细节：收集数据时记录的应该是 **student 的 observation**（部署可得信息），而不是 teacher 的 observation（包含 privileged）。因为 student 训练时的输入必须严格等于部署时的输入。teacher 的 observation 只用于产生 teacher 的 action 标签。

**使用 DistillationRunner 训练 Student 的完整代码：**

收集完数据后，训练 student 就是标准的监督学习。以下展示如何用 RSL-RL 的 `DistillationRunner` 完成训练：

```python
# train_student.py — Stage 3 蒸馏训练
import torch
from rsl_rl.runners import DistillationRunner

def train_student(student_model, distill_data_dir, cfg):
    """
    用 DistillationRunner 训练 depth student。

    Args:
        student_model: student 网络（输入维度 = 部署 obs dim）
        distill_data_dir: 蒸馏数据目录（包含多个 .npz 文件）
        cfg: 训练配置（lr, batch_size, num_epochs 等）
    """
    # Step 1: 加载蒸馏数据
    all_obs = []
    all_actions = []
    for npz_file in sorted(Path(distill_data_dir).glob("*.npz")):
        data = np.load(npz_file)
        all_obs.append(torch.from_numpy(data["student_obs"]))
        all_actions.append(torch.from_numpy(data["teacher_action"]))

    obs_dataset = torch.cat(all_obs, dim=0).float().to(cfg.device)
    act_dataset = torch.cat(all_actions, dim=0).float().to(cfg.device)
    print(f"蒸馏数据集: {obs_dataset.shape[0]} 帧, "
          f"obs_dim={obs_dataset.shape[1]}, act_dim={act_dataset.shape[1]}")

    # Step 2: 构建 DataLoader
    dataset = torch.utils.data.TensorDataset(obs_dataset, act_dataset)
    loader = torch.utils.data.DataLoader(
        dataset, batch_size=cfg.batch_size, shuffle=True,
    )

    # Step 3: 训练循环
    optimizer = torch.optim.Adam(student_model.parameters(), lr=cfg.lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=cfg.num_epochs,
    )

    best_loss = float("inf")
    for epoch in range(cfg.num_epochs):
        epoch_loss = 0.0
        for obs_batch, act_batch in loader:
            pred_actions = student_model(obs_batch)
            loss = torch.nn.functional.mse_loss(pred_actions, act_batch)

            optimizer.zero_grad()
            loss.backward()
            # 梯度裁剪（防止 CNN 梯度爆炸）
            torch.nn.utils.clip_grad_norm_(
                student_model.parameters(), max_norm=1.0,
            )
            optimizer.step()
            epoch_loss += loss.item()

        avg_loss = epoch_loss / len(loader)
        scheduler.step()

        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(student_model.state_dict(), "student_best.pt")

        if epoch % 50 == 0:
            print(f"Epoch {epoch}/{cfg.num_epochs}: "
                  f"MSE={avg_loss:.6f}, LR={scheduler.get_last_lr()[0]:.6f}")

    print(f"训练完成. Best MSE: {best_loss:.6f}")
    return student_model
```

这段代码有几个关键的工程决策值得注意：

- **CosineAnnealingLR 学习率调度**：蒸馏训练与 RL 不同——数据集是固定的，所以可以用标准的 LR scheduler。Cosine annealing 在后期缓慢降低学习率，有助于 fine-tune 最后几个百分点的 loss
- **梯度裁剪 max_norm=1.0**：如果 student 有 CNN 编码器（处理 depth image），CNN 的梯度可能比 MLP 大一个数量级。不裁剪会导致 MLP 部分的权重被大梯度破坏
- **保存 best_model（而非 last_model）**：监督学习可能过拟合训练数据。best_model 是在验证集上 loss 最低的 checkpoint（这里简化为训练集 loss，实际应留 10% 做验证）

| 超参数 | 推荐值 | 说明 |
|--------|--------|------|
| batch_size | 512-2048 | 太小则梯度噪声大，太大则收敛慢 |
| lr | 1e-4 ~ 3e-4 | 配合 cosine annealing |
| num_epochs | 200-500 | 观察 loss 曲线决定 early stopping |
| grad_clip | 1.0 | CNN+MLP 混合网络必备 |
| val_split | 10% | 留出验证集检测过拟合 |

**Rollout 评估——蒸馏成功的必要检查：**

imitation loss 低不代表蒸馏成功——student 可能在 teacher 频繁访问的状态上拟合得好，但在边缘状态上完全失败（compounding error）。必须做 rollout 评估：

```python
def evaluate_student(env, student_model, teacher_model, num_episodes=100):
    """对比 student 和 teacher 的 rollout 表现。"""
    student_rewards = []
    teacher_rewards = []

    for ep in range(num_episodes):
        obs = env.reset()
        student_reward, teacher_reward = 0.0, 0.0

        for step in range(env.max_episode_length):
            # Student 只用部署可得的 obs
            student_obs = obs["policy"]  # 或 obs["actor"]
            with torch.no_grad():
                student_action = student_model(student_obs)

            # Teacher 用完整 obs（包含 privileged）
            teacher_obs = torch.cat([obs["policy"], obs["critic"]], dim=-1)
            with torch.no_grad():
                teacher_action = teacher_model(teacher_obs)

            # 用 student 的 action 推进环境
            obs, reward, done, info = env.step(student_action)
            student_reward += reward.mean().item()

        student_rewards.append(student_reward)

    print(f"Student mean reward: {np.mean(student_rewards):.2f} "
          f"± {np.std(student_rewards):.2f}")
    print(f"Teacher mean reward: {np.mean(teacher_rewards):.2f} "
          f"± {np.std(teacher_rewards):.2f}")
    ratio = np.mean(student_rewards) / max(np.mean(teacher_rewards), 1e-6)
    print(f"Student/Teacher ratio: {ratio:.2%}")

    # 蒸馏成功的标准：student ≥ 85% teacher performance
    if ratio >= 0.85:
        print("✅ 蒸馏成功: student 达到 teacher 85%+ 性能")
    else:
        print("❌ 蒸馏不足: 考虑增加数据多样性或改用 DAgger")
```

### 三阶段的 Checkpoint 管理

```text
project/
├── stage1_blind_teacher/
│   ├── checkpoints/
│   │   ├── model_5000.pt        # 训练中间状态
│   │   ├── model_10000.pt       # 最终 checkpoint
│   │   └── best_model.pt        # 最高 reward 的 checkpoint
│   └── config.yaml
├── stage2_depth_teacher/
│   ├── checkpoints/
│   │   ├── model_0.pt           # == stage1 best_model.pt (warm start)
│   │   ├── model_3000.pt
│   │   └── best_model.pt
│   ├── config.yaml
│   └── warmstart_from: "stage1_blind_teacher/best_model.pt"
├── stage3_depth_student/
│   ├── distill_data/
│   │   ├── rollout_friction_low.npz
│   │   ├── rollout_friction_high.npz
│   │   └── ...                  # 多种 DR 参数下的 rollout 数据
│   ├── checkpoints/
│   │   └── student_final.pt     # 最终可部署的 student
│   └── config.yaml
└── wandb/                        # 三个阶段的训练日志
```

### Teacher-Student 蒸馏的双指标评估

student 训练时看的是 imitation loss（action MSE）。但 loss 下降不代表 student 在环境中表现好——因为行为克隆（BC）的 compounding error 问题：student 的微小误差导致下一步状态偏移，偏移后的状态不在 teacher 数据分布内，student 输出进一步偏离，形成正反馈雪崩。

因此评估 student 需要两个指标同时通过：

| 指标 | 含义 | 计算方式 | 合格标准 |
|------|------|---------|---------|
| **Imitation Loss** | student 在 teacher 数据上复现 teacher 行为的能力 | held-out teacher rollout 上的 action MSE | MSE < 0.05（典型值） |
| **Rollout Performance** | student 独立运行时的实际任务表现 | student 在环境中的 reward 和 episode length | reward ≥ teacher reward × 0.8 |

只有两个指标同时合格才算蒸馏成功。如果 imitation loss 低但 rollout 差——compounding error 严重，需要增加蒸馏数据多样性或使用 DAgger（online distillation）。如果 imitation loss 高但 rollout 还行——student 可能找到了不同于 teacher 但同样有效的控制策略。

### Teacher 可蒸馏性检查

不是所有 teacher 都可以成功蒸馏。如果 teacher 的行为严重依赖 student 完全无法近似的信息，蒸馏就会失败。

| 检查项 | 通过条件 | 不通过的后果 |
|--------|---------|-----------|
| teacher 的每个 privileged input 是否有 student 可观测的代理？ | 每个 privileged 信号要么有部署传感器替代，要么可从 history 推断 | student action MSE 无法下降 |
| teacher 是否使用了未来信息？ | teacher 所有输入都是当前或过去的状态 | student 永远无法复现因果违反的行为 |
| teacher 的行为是否对 privileged 信息过度敏感？ | 略微扰动 privileged 输入，teacher 行为不剧变 | student 的近似误差被放大，rollout 不稳定 |
| teacher 和 student 的动作空间是否一致？ | 两者使用相同的 action space 和 scale | 蒸馏目标的物理含义不一致 |

### extreme-parkour 的工程启示

从 extreme-parkour 项目中可以提炼出几条通用的工程原则：

**原则一：渐进式信息降级。** 不要一步从最丰富的信息跳到最匮乏的信息。每个阶段只移除一类 privileged 信息或添加一种新的输入模态。这让每个阶段的优化问题都足够简单。

**原则二：warm start 而非冷启动。** 每个新阶段都从上一阶段的 checkpoint 初始化，保留已学到的运动技能。冷启动意味着从零开始，warm start 意味着只需要学习增量变化。

**原则三：数据多样性是蒸馏的生命线。** Stage 3 的蒸馏质量严重依赖 Stage 2 rollout 数据的多样性。如果 teacher 只在一种 DR 参数下运行，student 学到的策略就只在那种参数下有效。

**原则四：分离感知与控制。** Stage 1 解决"如何控制"（纯运动技能），Stage 2 解决"如何感知"（从 depth 中提取地形信息），Stage 3 解决"如何适应"（从 history 中估计环境参数）。每个阶段聚焦一个子问题。

> **本质洞察**：extreme-parkour 的三阶段管线揭示了一个深刻的工程原理：复杂系统的设计不是"一步到位"，而是"逐步剥离辅助信息"。每个阶段都在前一个阶段的基础上去掉一层"脚手架"（privileged 信息），直到最终的 student 只依赖部署可得信息。这和建筑施工中先搭脚手架、再浇混凝土、最后拆脚手架的过程完全一致——脚手架（privileged 信息）不会出现在最终建筑中，但没有它建筑无法成型。

### 在 Isaac Lab 中的对应实现思路

extreme-parkour 的原始代码基于 Isaac Gym（旧框架），但其三阶段管线的逻辑完全可以在 Isaac Lab 和 mjlab 中复现。在 Isaac Lab 中，关键的对应关系是：

| extreme-parkour 概念 | Isaac Lab 对应 | mjlab 对应 |
|---------------------|---------------|------------|
| teacher obs group | `CriticCfg` + 自定义 `TeacherCfg` | `critic` group + 自定义 `teacher` group |
| depth image obs | `TiledCamera` + `ObsTerm` | `DepthCamera` sensor + `ObsTerm` |
| warm start | `load_model_cfg.path` 指向上阶段 checkpoint | `agent.load_run` 指向上阶段目录 |
| distillation | RSL-RL `DistillationRunner` | RSL-RL `DistillationRunner` |
| DR 参数 | `EventTermCfg` | `EventTerm` |

在 Isaac Lab 中实现三阶段管线的工程流程是：

1. **定义三个 task 配置**：`TaskTeacherBlindCfg`、`TaskTeacherDepthCfg`、`TaskStudentDepthCfg`，每个配置有不同的 observation group
2. **Stage 1**：用 `rsl_rl/train.py --task TaskTeacherBlind` 训练，保存 checkpoint
3. **Stage 2**：用 `rsl_rl/train.py --task TaskTeacherDepth --load_run <stage1_dir>` 训练，warm start
4. **Stage 3**：先用 `rsl_rl/play.py --task TaskTeacherDepth` 收集 rollout 数据，再用 `rsl_rl/distill.py --task TaskStudentDepth` 训练 student

以下是三个 task 配置的骨架代码，展示了 observation group 如何逐阶段变化：

```python
# ---- Isaac Lab: 三阶段 task 配置骨架 ----

class TaskTeacherBlindCfg(LocomotionVelocityRoughCfg):
    """Stage 1: Blind Teacher（无视觉，有 privileged）"""
    class observations:
        class policy:
            # actor 输入：部署可得 + privileged 环境参数
            terms = ["joint_pos", "joint_vel", "base_ang_vel",
                     "commands", "last_actions",
                     "friction_coeffs", "body_mass_offset",
                     "contact_forces_z", "terrain_heights"]
        class critic:
            # critic 额外看 ground truth velocity
            terms = ["joint_pos", "joint_vel", "base_ang_vel",
                     "commands", "last_actions",
                     "friction_coeffs", "body_mass_offset",
                     "contact_forces_z", "terrain_heights",
                     "base_lin_vel"]  # critic 特有

class TaskTeacherDepthCfg(TaskTeacherBlindCfg):
    """Stage 2: Depth Teacher（加入深度图，保留 privileged）"""
    class observations:
        class policy:
            # 在 Stage 1 基础上加入深度图
            terms = ["joint_pos", "joint_vel", "base_ang_vel",
                     "commands", "last_actions",
                     "friction_coeffs", "body_mass_offset",
                     "contact_forces_z", "terrain_heights",
                     "depth_image"]  # 新增：来自 TiledCamera
        class critic:
            terms = TaskTeacherBlindCfg.observations.critic.terms
            # critic 不需要 depth（已经有 terrain heights）

class TaskStudentDepthCfg(LocomotionVelocityRoughCfg):
    """Stage 3: Depth Student（仅部署可得信息）"""
    class observations:
        class policy:
            # 去掉所有 privileged，只保留部署可得
            terms = ["joint_pos", "joint_vel", "base_ang_vel",
                     "commands", "last_actions",
                     "depth_image"]  # 部署时来自真实深度相机
        # Stage 3 不需要 critic（用 BC 蒸馏，不用 RL）
```

注意 Stage 2 到 Stage 3 的关键变化：`friction_coeffs`、`body_mass_offset`、`contact_forces_z` 和 `terrain_heights` 这些 privileged terms 全部被移除。这就是"信息降级"的工程含义——配置层面的差异清晰地定义了 teacher 和 student 之间的信息边界。

**Warm Start 的工程细节：**

Stage 2 的 warm start 需要特殊处理，因为网络输入维度发生了变化（新增 depth image 的 CNN 编码器）。正确的做法是部分加载：

```python
# Stage 2 warm start: 部分权重加载
checkpoint = torch.load(stage1_checkpoint_path)
stage1_state = checkpoint['model_state_dict']

# 只加载维度兼容的层
model_state = model.state_dict()
compatible_keys = []
for key in stage1_state:
    if key in model_state and stage1_state[key].shape == model_state[key].shape:
        compatible_keys.append(key)
        model_state[key] = stage1_state[key]

model.load_state_dict(model_state)
print(f"Loaded {len(compatible_keys)}/{len(stage1_state)} layers from Stage 1")
# 预期：CNN 相关层不会被加载（维度不同），MLP 的前几层会被加载
```

这个部分加载策略确保 proprioception → MLP 的知识被继承，而 CNN 编码器从随机初始化开始训练。如果错误地使用 `strict=True`，PyTorch 会抛出维度不匹配异常；如果使用 `strict=False` 但不检查哪些 key 被加载了，可能会遗漏关键层而不自知。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Stage 2 warm start 时网络维度不匹配。** Stage 1 的 actor 输入维度与 Stage 2 不同（Stage 2 多了 depth image 的 CNN latent 维度）。warm start 时需要只加载 proprioception → MLP 部分的权重，CNN 编码器随机初始化。如果直接 `load_state_dict(strict=True)`，会因维度不匹配而报错。

⚠️ **编程陷阱：Stage 3 蒸馏数据中 student obs 与部署 obs 不一致。** 如果蒸馏数据收集时 student obs 中混入了某些 env 特有的信号（如 reward 中间量），部署时这些信号不存在。student 的 obs 配置必须严格等于部署配置。

💡 **概念误区：认为"三阶段管线只适用于视觉任务"。** 三阶段的思想（渐进式信息降级）适用于任何存在大跨度信息鸿沟的任务。即使不涉及视觉，如果 actor 和部署之间有多类 privileged 信息需要逐步移除，多阶段管线也是值得考虑的。

💡 **概念误区：认为"Stage 3 必须用 BC 蒸馏"。** BC 是最常用的蒸馏方式，但不是唯一的。你也可以在 Stage 3 用 RL fine-tuning——student 先通过 BC 初始化，然后用 RL reward 做微调，减轻 compounding error。这相当于"先模仿老师打基础，然后自己练习改进"。

🧠 **思维陷阱：认为"teacher 的 rollout 数据量越多蒸馏越好"。** 数据量重要，但**数据多样性**更重要。10M 帧数据如果都来自同一种 DR 参数，不如 1M 帧数据覆盖 100 种 DR 参数组合。蒸馏数据的采集应该在 DR 参数空间上做均匀或分层采样。

### 练习

1. **[设计题]** 设计一个两阶段（而非三阶段）的 teacher-student 管线：Stage 1 训练 state-based teacher，Stage 2 直接蒸馏到 depth student。与三阶段方案相比，这种两阶段方案在什么情况下可行？什么情况下会失败？
2. **[分析题]** student 的 imitation loss 已经很低（< 0.01），但在 rollout 中 3 秒后总是摔倒。列出三个可能原因并给出对应的诊断方法。
3. **[跨章综合题]** 结合 Ch06（Reward 设计）和 Ch08（Domain Randomization）：如果 Stage 1 teacher 的 reward 中存在 reward hacking（如机器人找到了一种摆臂方式骗取 alive bonus 但实际没在走），Stage 3 的 student 会继承这个 bug 吗？为什么？如何在蒸馏前检测和修复？

### 前沿视角：HOVER 的 Mask-Conditioned Distillation

extreme-parkour 的三阶段管线是 2024 年的工业标准，但 2025 年的 HOVER（NVlabs/HOVER, ICRA 2025, He et al.）提出了一个更灵活的蒸馏框架——**mask-conditioned distillation**——它把 teacher-student 蒸馏的理念从"单一任务的信息降级"拓展到"多模式控制的统一蒸馏"。

HOVER 的核心思路：先训练一个 oracle teacher，它输入完整的 SMPL 参考姿态 + 机器人状态（全部 privileged）；然后蒸馏到一个 student，但蒸馏时**随机 mask 掉 teacher 命令的不同子集**。不同的 mask 配置对应不同的控制模式：

| Mask 配置 | 激活的命令通道 | 对应控制模式 |
|----------|--------------|------------|
| 全部激活 | 全身关键点 + 根速度 | 全身动作跟踪 |
| 只激活上半身 | 手臂/头部关键点 | 桌面操作 |
| 只激活根速度 | base_lin_vel + base_ang_vel | 速度导航 |
| 只激活头+手 | 头部/手部位置 | 遥操作 |

DAgger 蒸馏过程中，每个 mini-batch 随机采样不同的 mask，student 必须在所有 mask 配置下都产生合理行为。训练完成后，一个 student 网络支持 **>15 种控制模式**——切换控制模式只需要改变输入 mask，不需要切换网络。

HOVER 的工程实现是一个标准的 Isaac Lab extension。训练吞吐量参考值（RTX 4090, 1024 envs）：teacher ~0.84 s/iter（100k iter ≈ 23 小时），student ~0.097 s/iter（10k iter ≈ 16 分钟）。推荐 4096 envs 以获得更好的结果。

与 extreme-parkour 的对比：

| 维度 | extreme-parkour | HOVER |
|------|----------------|-------|
| 阶段数 | 3（blind → depth → student） | 2（oracle → masked student） |
| 蒸馏维度 | 信息类型（privileged → deployable） | 命令模式（full → masked subset） |
| 部署灵活性 | 一个 student 对应一种控制模式 | 一个 student 支持多种控制模式 |
| 适用对象 | 四足 + 深度相机 | 人形全身控制 |
| 框架依赖 | Isaac Gym（已废弃） | Isaac Lab（当前标准） |

### 前沿视角：VIRAL 的 64-GPU 视觉蒸馏

如果 extreme-parkour 代表"单 GPU 上的三阶段蒸馏"，那么 VIRAL（NVIDIA 2025, arXiv 2511.15200）代表"大规模集群上的视觉 teacher-student"。VIRAL 在 Unitree G1 上实现了 54/59 连续 loco-manipulation 循环，其管线的核心仍然是 privileged teacher → visual student，但规模和细节更加工业化：

1. **Privileged RL Teacher**：delta-action space + reference-state-initialization，privileged terrain/object 真值，训练 ~24-72 小时
2. **Visual Student DAgger**：大规模 tiled rendering（Isaac Lab `TiledCamera`，每 GPU 512 cameras）生成 depth/RGB 观测；student 在 teacher 诱导的状态分布上用 DAgger 训练
3. **视觉 Domain Randomization**：lighting、materials、camera intrinsics/extrinsics、image quality degradation、sensor delay——全部在 Isaac Lab 的 Replicator API 中配置

VIRAL 的关键工程发现：视觉 DR 的多样性比物理 DR 更重要——不做视觉 DR 时 sim-to-real 成功率从 >90% 降到 <30%。这呼应了 9.1 节讨论的 "DR-privileged 协同"主题：视觉管线中的 DR 不仅覆盖物理参数，还必须覆盖渲染参数。

### 从训练到部署：ONNX 导出的最后一公里

在 privileged learning 流水线中，最终要部署的是 actor（asymmetric AC 场景）或 student（蒸馏场景）。RSL-RL 提供了 `export_policy_as_onnx()` 函数，把 PyTorch 模型转换为 ONNX 格式用于边缘设备（如 Jetson Orin）上的实时推理。

```python
from rsl_rl.utils import export_policy_as_onnx

# 正确：导出 actor 的网络 + actor 的 normalizer
export_policy_as_onnx(
    actor_model=runner.alg.actor,
    path="./exported_models/",
    normalizer=runner.alg.actor.obs_normalizer,  # ← 必须是 actor 的
    filename="policy.onnx"
)
```

`export_policy_as_onnx()` 的关键机制是 **normalizer 烘焙**：它通过 `copy.deepcopy` 把 normalizer（running mean/std）作为 ONNX 模型的第一层嵌入，这样部署时不需要额外的 Python 归一化代码——ONNX 模型的 forward 第一步就是 `x = (x - mean) / std`。

⚠️ **部署陷阱：导出 critic 的 normalizer 而非 actor 的。** 如果你错误地使用了 critic 的 normalizer，它包含 privileged 维度的统计量，输入维度不匹配——ONNX 推理时会报 shape error 或产生垃圾输出。

⚠️ **部署陷阱：LSTM/RNN 模型的 ONNX 导出限制。** RSL-RL 4.0 的 ONNX exporter 目前对 RNN 模型硬编码了 LSTM 格式（已知 issue isaac-sim/IsaacLab #3008）。如果你的 actor 使用 GRU 或 Transformer，需要自定义导出逻辑。

部署前验证清单：

1. 打印 ONNX 模型的输入维度，确认与部署传感器提供的维度一致
2. 在仿真中用 ONNX 推理替代 PyTorch 推理（sim-to-sim 验证），确认行为一致
3. 在真机上先以 zero action 运行（验证通信正常），再切换到策略输出
4. 确认 `base_lin_vel` 的来源——真机 IMU 提供角速度但不直接提供线速度，需要状态估计器（如 EKF 融合 IMU + 腿部运动学）。如果训练时 actor 输入了 `base_lin_vel`，部署时必须有对应的估计器，否则需要将其移到 critic-only 并重新训练
5. 检查 observation 的物理单位——仿真中 `joint_pos` 的单位是 rad，`base_ang_vel` 是 rad/s。确认真机传感器的单位与此一致，不是 deg 或 deg/s

```python
# sim-to-sim ONNX 验证脚本
import onnxruntime as ort
import torch

def validate_onnx(onnx_path, pytorch_actor, test_obs, rtol=1e-4):
    """比较 ONNX 和 PyTorch 的输出是否一致。"""
    # PyTorch 前向
    with torch.no_grad():
        pt_output = pytorch_actor(test_obs).cpu().numpy()

    # ONNX 前向
    session = ort.InferenceSession(onnx_path)
    input_name = session.get_inputs()[0].name
    onnx_output = session.run(None, {input_name: test_obs.cpu().numpy()})[0]

    # 比较
    max_diff = abs(pt_output - onnx_output).max()
    print(f"Max diff between PyTorch and ONNX: {max_diff:.8f}")
    assert max_diff < rtol, f"ONNX mismatch! max_diff={max_diff} > rtol={rtol}"
    print("✅ ONNX validation passed")
```

⚠️ **关键安全提醒（来自 Isaac Lab 官方文档）：** "While real robot IMU sensors provide angular acceleration (which can be integrated to get angular velocity), they cannot directly measure linear velocity." 这是 privileged learning 最根本的工程动机之一——`base_lin_vel` 在仿真中可以直接读取但在真机上不可得。如果你的 actor 在训练时依赖了 `base_lin_vel`，这个信息必须在部署前移除或用估计器替代。

---

至此，我们完成了从算法概念到工程管线的完整旅程。最后一节把本章所有知识点串联成一个决策框架，帮助你在面对新项目时快速选择合适的 privileged learning 方案。

## 9.6 方案选型与工程决策树 ⭐⭐

> **这一节解决什么问题**：把前面五节的知识整合成一套可操作的选型框架，面对新的机器人 RL 项目时，快速判断应该用非对称 AC、RMA 还是多阶段蒸馏。

### 动机：不是所有项目都需要三阶段管线

回顾 Ch08（Domain Randomization）中讨论的"过度工程化"问题：不是所有任务都需要全范围 DR。同样的原则适用于 privileged learning——你不应该在一个传感器充足的平地行走任务上上三阶段蒸馏。选择最简单的能解决问题的方案，是区分工程新手和工程老手的关键标志。

理解这个问题的另一个角度来自 Ch06（Reward 设计）的"reward 复杂度与任务难度匹配"原则：更复杂的 privileged learning 管线意味着更多的超参数、更长的调试周期、更多的潜在故障点。如果任务本身不需要这种复杂度，额外的管线只会增加工程负担而不提供边际收益。

extreme-parkour 的三阶段管线很强大，但工程复杂度也很高——三个阶段的配置、checkpoint 管理、数据收集、多次训练。对于很多项目，简单的 asymmetric AC 就足够了。关键问题是：**你的项目需要什么层次的 privileged learning？** 以下是一个快速的工程成本估算，帮助你在项目初期做出选型决策：

| 方案 | 需要的额外配置文件 | 额外训练阶段 | 额外调参维度 | 典型调试时间 |
|------|------------------|------------|------------|------------|
| 直接 RL（无 privileged） | 0 | 0 | 0 | 1 天 |
| asymmetric AC | 1（critic obs） | 0 | obs group 选择 | 1-2 天 |
| RMA | 2（encoder + adapter） | +1（Phase 2） | latent dim, history_len | 3-5 天 |
| 两阶段蒸馏 | 3（teacher + student + distill） | +1（distillation） | BC loss, data amount | 1-2 周 |
| 三阶段蒸馏 | 5+（per-stage configs） | +2 | warm start, MTS, per-stage DR | 2-4 周 |

### 如果选错方案会怎样

**选太简单的方案（该用蒸馏但只用了 asymmetric AC）：** 你的四足机器人需要在极端地形上做视觉导航，但你只配了 asymmetric AC。actor 看 noisy depth image，critic 额外看 clean terrain。结果是 actor 需要同时从 RL reward 中学习"如何从图像中提取地形特征"和"如何在各种地形上运动"——两个任务耦合，训练极慢且不稳定。三阶段蒸馏可以把这两个任务解耦——Stage 1 学运动、Stage 2 学视觉、Stage 3 去特权。

**选太复杂的方案（该用 asymmetric AC 但上了三阶段蒸馏）：** 你的任务是平地行走，传感器充足（IMU + 编码器 + 足底力传感器），但你设计了一个完整的三阶段蒸馏管线。结果是你花了 3 倍的时间管理三个阶段的配置和 checkpoint，但最终 student 的表现和直接用 asymmetric AC 训练的 actor 差不多——因为 actor 已经有足够的信息做好决策了。

**选了对的方案但 DR 没跟上（有 RMA 但 DR 太窄）：** 你正确选择了 RMA 来处理未知摩擦和质量变化，但 Ch08 中配置的 DR 范围太窄——摩擦只在 [0.9, 1.1] 之间变化。RMA 的 adaptation module 在训练时从未见过显著不同的环境参数，proprioception history 中看不出有意义的差异——adaptation module 退化为一个常数输出。部署到真机时，真实摩擦可能是 0.4（光滑地面），adaptation module 无法响应——因为它从未学会"不同的摩擦长什么样"。这就是 9.1 节讨论的 DR-privileged 协同问题在选型层面的具体体现。**解决方案：先确认 DR 范围足够宽（Ch08 的 Phase 1-2），再决定是否需要 RMA。**

### 决策流程

```text
开始
│
├── Q1: actor 和部署 sensor 之间是否存在信息差？
│   ├── 否 → 不需要 privileged learning（直接 RL 训练）
│   │
│   └── 是 → Q2: 这些 privileged 信息只需要帮助训练信号吗？
│       │
│       ├── 是 → 非对称 AC（critic 看 privileged，actor 只看部署信息）
│       │         适用：大多数 locomotion 任务，接触力和地形真值只用于 value 估计
│       │
│       └── 否 → Q3: actor 是否需要在部署时间接获取环境参数？
│           │
│           ├── 是 → RMA / Adaptation Module
│           │         适用：需要实时适应摩擦/质量/地面倾斜变化
│           │
│           └── 否 → Q4: actor 的部署输入模态是否与训练模态完全不同？
│               │
│               ├── 是 → 多阶段 Teacher-Student 蒸馏
│               │         适用：state-based → depth/RGB 的模态跨越
│               │
│               └── 否 → 单阶段 Teacher-Student 蒸馏
│                         适用：信息类型相同但精度不同
```

### 四种方案的成本-收益对比

| 方案 | 工程成本 | 训练时间 | sim 表现 | 部署鲁棒性 | 适用场景 |
|------|---------|---------|---------|----------|---------|
| 直接 RL | ⭐ | 基准 | 基准 | 低 | 简单任务，信息充足 |
| 非对称 AC | ⭐⭐ | ~1.2× 基准 | +10-20% | 中 | 大多数 locomotion |
| RMA | ⭐⭐⭐ | ~2× 基准 | +15-30% | 高 | 环境参数自适应 |
| 多阶段 TS | ⭐⭐⭐⭐ | ~3-5× 基准 | 最高 | 最高 | 视觉部署、极端地形 |

### 技术演进脉络：从 Pinto 2018 到 VIRAL 2025

理解这四种方案的选型不能脱离历史语境——每一种方案都是前一代方案遇到瓶颈后的自然扩展。以下时间线梳理了 privileged learning 从概念提出到工业级部署的完整演化路径，每一代解决了上一代的什么问题：

| 年份 | 代表工作 | 会议 | 核心贡献 | 解决了上一代什么问题 |
|------|---------|------|---------|-------------------|
| 2018 | Pinto et al. | RSS | Asymmetric Actor-Critic | 首次提出 actor/critic 可以看到不同信息 |
| 2020 | Lee et al. | Science Robotics | Teacher-Student + terrain curriculum | 把 privileged learning 从视觉扩展到 locomotion |
| 2021 | Kumar et al. (RMA) | RSS | Adaptation module + online adaptation | 让 student 在部署时实时适应新环境 |
| 2022 | Miki et al. | Science Robotics | ANYmal depth + attention encoder | 从盲控制到感知控制；1700m 零跌倒 |
| 2022 | Rudin et al. | CoRL | 大规模并行 + terrain curriculum | 把训练时间从天降到分钟 |
| 2024 | Cheng et al. (extreme-parkour) | ICRA | 三阶段蒸馏 + ROA | 跨模态（state→depth）的渐进式蒸馏 |
| 2024 | Radosavovic et al. | Science Robotics | Causal transformer adaptation | 不需要显式 adaptation module |
| 2025 | He et al. (HOVER) | ICRA | Mask-conditioned distillation | 一个 student 多种控制模式 |
| 2025 | HoST | RSS | Multi-critic architecture | 解耦安全/探索/精度的 value 估计 |
| 2025 | He et al. (ASAP) | RSS | Delta action model | 用真机数据对齐仿真，替代手工 DR |
| 2025 | He et al. (VIRAL) | arXiv | 64-GPU 视觉 teacher-student | 视觉 DR + 大规模 DAgger 达到真机鲁棒 |

从这个演进脉络中可以看到两条清晰的技术趋势：

**趋势 1：信息边界的粒度越来越细。** 从 Pinto 2018 的"actor vs critic 二分法"，到 extreme-parkour 2024 的"三阶段渐进降级"，到 HOVER 2025 的"per-command-channel 的 mask"——信息边界从粗粒度二分走向了细粒度的连续谱。

**趋势 2：从手工设计走向数据驱动。** 传统的 DR + asymmetric AC 需要手动选择哪些参数随机化、哪些信号给 critic。ASAP 用真机数据自动学习 sim-to-real gap 的补偿；VIRAL 用大规模渲染自动覆盖视觉域偏移。这个趋势意味着未来的 privileged learning 可能不再需要工程师手动分类 privileged 信号——系统会自动发现"什么信息对训练有帮助但部署时不可得"。

### 四足 / 人形 / 操作的方案偏好

不同类型的机器人和任务对 privileged learning 的需求不同。

**四足 locomotion。** 典型配置是 asymmetric AC + 可选 RMA。actor 看 proprioception + noisy height scan，critic 额外看 clean terrain + contact forces。如果需要在未知摩擦地面上自适应，加 RMA adaptation module。如果需要视觉部署（depth camera 替代 height scan），升级到两阶段蒸馏。

**人形 locomotion。** 比四足复杂得多。状态维度更高（更多关节自由度），动量耦合更强（上半身运动影响下半身平衡）。人形的 teacher 设计有额外考虑：全身关键点信息（`body_pos`、`body_ori`）对 tracking 任务的 critic 极其有用，但在真机上需要 motion capture 系统。No-State-Estimation variant（移除依赖状态估计器的 terms）是 teacher-student 设计的天然起点。

人形 locomotion 中 privileged 信息的一个独特挑战是**角动量管理**。四足机器人有四个支撑点，静态稳定性强，角动量变化的影响相对较小。人形机器人只有两个支撑点（甚至单脚支撑期只有一个），角动量的管理至关重要——甩臂平衡、髋部扭转补偿都是关键行为。teacher 可以直接获得精确的角动量数值（仿真中直接从状态读取），student 需要从 IMU 和关节状态推断。

回顾 Ch05 中 observation 设计的讨论：mjlab 的 tracking task 提供了一个 `has_state_estimation=False` 的变体。这个变体从 actor 中移除了 `motion_anchor_pos_b` 和 `base_lin_vel`——模拟真机上没有精确状态估计器的场景。critic 仍然保留完整信息。这是 teacher-student 设计的天然起点：full-state actor 就是 teacher，No-State actor 就是 student 的目标输入空间。

**操作（Manipulation）。** privileged 信息的核心是物体位姿和接触状态。Teacher 可以直接看到物体的精确 6DoF 位姿，student 只看 RGB/depth 图像。蒸馏的挑战在于视觉特征提取——从图像中恢复 6DoF 位姿比从 proprioception 中推断摩擦系数困难得多。

操作任务中 privileged learning 还有一个独特的考量：**接触力的方向性**。locomotion 中接触主要发生在脚底，方向相对固定（向下）。manipulation 中接触可以发生在指尖、指侧、手掌的任何位置，方向也是任意的。这意味着操作任务的 privileged 接触信息维度更高、更难从 history 推断。一种常见做法是用触觉传感器作为接触力的部署替代，而不是完全依赖 estimator。

| 任务类型 | 核心 privileged 信号 | 推荐方案 | 典型蒸馏跨度 |
|---------|--------------------|---------|-----------| 
| 四足 velocity（平地） | 接触力、地面摩擦 | 非对称 AC | 信息质量差异（有噪 → 无噪） |
| 四足 velocity（粗糙地形） | 地形真值、接触力、环境参数 | 非对称 AC + RMA | 信息存在性差异（需要估计） |
| 四足 parkour（极端地形） | 完美地形 + 环境参数 | 三阶段蒸馏 | 模态跨越（state → depth） |
| 人形 tracking | 全身关键点、角动量 | 非对称 AC | 信息质量差异 |
| 桌面抓取 | 物体位姿、接触法线 | 两阶段蒸馏 | 模态跨越（state → depth/RGB） |
| 灵巧手操作 | 指尖接触力、物体滑移 | 非对称 AC + 力传感器 | 传感器精度差异 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：在简单任务上使用过于复杂的 privileged learning 方案。** 如果你的任务是平地行走且传感器充足，asymmetric AC 就够了——不需要上三阶段蒸馏。过度复杂的管线增加了调试难度和出错概率。遵循"足够用就好"的原则。

💡 **概念误区：认为"privileged learning 只是一种可选的优化技巧"。** 对于需要部署到真机的项目，privileged learning 不是优化——它是正确性保证。没有信息边界设计的策略在部署时几乎必然失败。

🧠 **思维陷阱：认为"选了方案就不需要回退"。** 工程实践中，你可能先选了 asymmetric AC，训练后发现 actor 在某些环境参数下表现差（说明 actor 的 observation 信息不够），需要升级到 RMA。也可能先选了三阶段蒸馏，但发现 teacher 在 Stage 1 就无法稳定训练（说明 reward 设计或 DR 范围有问题，不是 privileged learning 层面的问题），需要先退回去修 Ch06/Ch08 的内容。**方案选型是迭代的，不是一次性的。** 决策树给出的是起点，不是终点。

### 练习

1. **[选型题]** 你要训练一个双足机器人在室内平地上行走。机器人有 IMU + 关节编码器 + 足底力传感器。应该选择哪种 privileged learning 方案？说明理由。
2. **[选型题]** 你要训练一个四足机器人在户外草地上走。机器人有 IMU + 关节编码器 + 前向深度相机，但没有足底力传感器。应该选择哪种方案？如果地形包含台阶和沟壑呢？
3. **[设计题]** 为你自己的研究项目（或假设一个项目）设计完整的 privileged learning 方案。写出 teacher/student 的 observation 配置、蒸馏策略和评估计划。
4. **[跨章综合题]** 回顾本章 9.6 的工程成本表和 Ch08 的分阶段 DR 策略。假设你有 2 周时间训练一个 Go2 在粗糙地形上的 locomotion 策略并部署到真机。写出一份时间分配计划：第几天做 DR baseline、第几天加 asymmetric AC、什么条件下决定升级到 RMA、什么条件下回退。

---

本章从算法概念（三种 privileged learning 形态）出发，经过工程配置（双框架 obs group）、网络训练（estimator / adaptation module）、完整管线精读（extreme-parkour 三阶段 + HOVER/VIRAL 前沿方案）、到选型决策（成本-收益评估 + 技术演进脉络），构建了一套从"理解信息边界"到"部署 ONNX 模型"的完整工程路径。下面用一张表格总结所有知识点。

## 本章小结

| 知识点 | 核心结论 | 重要程度 |
|--------|---------|---------|
| 信息不对称是部署核心矛盾 | 仿真和真机的信息差不是小问题，是决定部署成败的第一道关卡 | ⭐ |
| privileged 信息四类分类 | 环境参数（慢变）、接触信息（快变）、全局感知、未来信息（禁用） | ⭐⭐ |
| 三种形态的统一视角 | 非对称 AC → BC 蒸馏 → 并发 TS，信息蒸馏链越长跨越的鸿沟越宽 | ⭐⭐ |
| multi-critic 架构 | HoST 的安全/探索/精度多 critic 分离，解耦冲突的训练目标 | ⭐⭐ |
| DR-privileged 协同 | DR 范围决定 student 的 implicit system-ID 能力，两者必须联合调参 | ⭐⭐⭐ |
| 双框架 obs group 配置 | mjlab: actor/critic dict + obs_groups routing；Isaac Lab: PolicyCfg/CriticCfg | ⭐⭐⭐ |
| RSL-RL 4.0 解耦配置 | actor/critic 用独立 `RslRlMLPModelCfg`，各自维护 normalizer | ⭐⭐⭐ |
| observation normalization | running mean/std 的沉默陷阱：buffer 未初始化、checkpoint 恢复遗漏 | ⭐⭐⭐ |
| RMA adaptation module | 从 proprioception history 估计环境参数 latent，而非直接预测物理量 | ⭐⭐⭐ |
| 三种在线适应范式 | RMA（显式）vs causal transformer（隐式）vs TTT（在线梯度） | ⭐⭐ |
| Teacher-Student 不是 Actor-Critic | teacher 产生动作（可模仿），critic 产生 value（辅助训练） | ⭐⭐ |
| 蒸馏双指标评估 | imitation loss 和 rollout performance 都要通过 | ⭐⭐⭐ |
| extreme-parkour 三阶段 | blind teacher → depth teacher → depth student，渐进式信息降级 | ⭐⭐⭐⭐ |
| HOVER mask-conditioned distillation | 一个 student 多种控制模式，比三阶段更灵活 | ⭐⭐⭐ |
| ONNX normalizer 烘焙 | 部署时必须导出 actor 的 normalizer，不能用 critic 的 | ⭐⭐ |
| 15 项 debug checklist | 含 normalization 检查，系统化防止信息泄漏和配置错误 | ⭐⭐ |
| 方案选型决策树 + 演进脉络 | 从 Pinto 2018 到 VIRAL 2025 的完整技术演化链 | ⭐⭐ |

## 累积项目：本章新增模块

在累积项目中，本章增加的模块是**"信息边界审查与蒸馏方案设计"**。你应该能够：

1. 对 velocity task 的 actor/critic 配置执行完整的 15 项 debug checklist，每项给出"通过/不通过"和理由
2. 在 mjlab 和 Isaac Lab 中分别打印 actor/critic observation 维度并确认 privileged terms 生效
3. 为项目中的机器人设计 RMA adaptation module 的 history 输入配置，包括 term 选择、history_length 和维度计算
4. 区分 asymmetric actor-critic 和 teacher-student 蒸馏，能用一句话说清两者的核心差异
5. 撰写一份信息边界审计报告，明确标注每个 observation term 的角色归属和部署方案

### 累积项目与前置章节的连接

本章的信息边界设计直接依赖 Ch05 的 observation/action 接口框架。Ch05 建立了"部署可得性原则"，本章在此基础上把这个原则具体化为 actor/critic group 配置和 debug checklist。如果你在执行 checklist 时发现对某个 term 的"部署可得性"判断不确定，回到 Ch05 的信号分类表重新审查。

本章的蒸馏设计也依赖 Ch06 的 reward 知识。teacher 的训练质量取决于 reward 设计——如果 teacher 的 reward 有漏洞（如 reward hacking），蒸馏出来的 student 会继承这些漏洞行为。蒸馏不能修复 reward 设计错误——它只能把 teacher 的行为忠实地迁移给 student。

本章的 obs_groups routing 机制在 Ch07（PPO 训练管线）中有详细介绍。如果你对 `OnPolicyRunner` 如何使用 obs_groups 从 env 获取 observation 仍有疑问，回到 Ch07 重新阅读 runner 的数据流部分。

本章讨论的 DR-privileged 协同关系（9.1 节过渡段）是理解 Ch08（Domain Randomization）工程价值的关键桥梁。DR 不仅仅是"让策略更鲁棒"，它同时也是"让 adaptation module 的隐式 system-ID 成为可能"。如果你发现 adaptation module 的 MSE 不降，第一个排查方向是回到 Ch08 检查 DR 范围是否覆盖了足够的参数变化。

本章的 teacher-student 蒸馏思想将在 Ch10（模仿学习）中进一步发展。Ch10 的 AMP 判别器训练和 BC/DAgger 管线与本章的蒸馏方法论高度互补——AMP 提供"风格约束"，而 teacher-student 提供"技能迁移"。两者可以组合：先用 AMP 训练一个风格自然的 teacher，再蒸馏到 deployable student。

本章的信息边界设计最终会在 Ch23（Sim2Real 部署）中接受真机验证。Ch23 将详细讨论 ONNX 导出后的真机部署流程——从通信延迟补偿到关节安全限位——这些都建立在本章"actor 只看部署可得信息"的基础之上。如果 Ch09 的信息边界有任何遗漏，Ch23 的真机测试会以最残酷的方式暴露它们。

项目代码保存在独立目录。请在实验日志中标注"累积项目：Ch09 新增信息边界审查模块"。

### 快速验证脚本

以下脚本可以一键验证你的 privileged obs 配置是否正确。把它放在项目目录下，每次修改 obs 配置后运行一次。

```python
# verify_privileged_obs.py
# 用法：python verify_privileged_obs.py --framework mjlab|isaac_lab

def verify_obs_groups(env, framework="mjlab"):
    """验证 actor/critic observation 配置的正确性。"""
    obs = env.reset()
    checks_passed = 0
    checks_total = 5

    if framework == "mjlab":
        actor_obs = obs["obs"]["actor"]
        critic_obs = obs["obs"]["critic"]
    else:  # isaac_lab
        actor_obs = obs["policy"]
        critic_obs = obs["critic"]

    # Check 1: critic 维度应该 > actor 维度
    actor_dim = actor_obs.shape[-1]
    critic_dim = critic_obs.shape[-1]
    if critic_dim > actor_dim:
        print(f"✅ Check 1 PASS: critic({critic_dim}) > actor({actor_dim})")
        checks_passed += 1
    else:
        print(f"❌ Check 1 FAIL: critic({critic_dim}) <= actor({actor_dim})")
        print("   → critic 应该包含额外的 privileged terms")

    # Check 2: actor obs 的前 N 维应与 critic 对齐
    shared_dim = min(actor_dim, critic_dim)
    if torch.allclose(actor_obs[:, :shared_dim], critic_obs[:, :shared_dim]):
        print(f"✅ Check 2 PASS: 共享维度 {shared_dim} 对齐")
        checks_passed += 1
    else:
        print(f"⚠️  Check 2 WARN: 共享维度数值不一致（可能是 noise 导致）")
        checks_passed += 1  # noise 差异是预期行为

    # Check 3: privileged 维度包含非零信号
    priv_dims = critic_obs[:, actor_dim:]
    if priv_dims.abs().max() > 1e-6:
        print(f"✅ Check 3 PASS: privileged dims 包含非零信号")
        checks_passed += 1
    else:
        print(f"❌ Check 3 FAIL: privileged dims 全为零")
        print("   → 检查 privileged terms 是否正确连接到环境数据")

    # Check 4: batch size 一致
    if actor_obs.shape[0] == critic_obs.shape[0]:
        print(f"✅ Check 4 PASS: batch size 一致 ({actor_obs.shape[0]})")
        checks_passed += 1
    else:
        print(f"❌ Check 4 FAIL: batch size 不一致")

    # Check 5: 无 NaN 或 Inf
    if not (torch.isnan(actor_obs).any() or torch.isnan(critic_obs).any()
            or torch.isinf(actor_obs).any() or torch.isinf(critic_obs).any()):
        print(f"✅ Check 5 PASS: 无 NaN/Inf")
        checks_passed += 1
    else:
        print(f"❌ Check 5 FAIL: 检测到 NaN 或 Inf")

    print(f"\n总计: {checks_passed}/{checks_total} 检查通过")
    return checks_passed == checks_total
```

这个脚本覆盖了最常见的 5 类配置错误。在实际项目中，建议把它集成到 CI/CD 流水线或 training 脚本的初始化阶段。

### 实验 Lab：Asymmetric AC 的 A/B 消融实验

以下是一个可直接运行的实验流程，验证 asymmetric AC 对 velocity tracking 任务的实际效果。通过 A/B 对比，你将直观看到 privileged critic 对训练速度和最终性能的影响。

**实验设计：**

| 组别 | Actor Obs | Critic Obs | 预期结果 |
|------|-----------|------------|---------|
| A（baseline） | proprio + noisy height | proprio + noisy height（与 actor 相同） | 收敛慢，value loss 高 |
| B（asymmetric） | proprio + noisy height | proprio + **clean** height + contact + friction | 收敛快，value loss 低 |

**Step 1：准备两份配置。** 在 mjlab 中，复制 velocity task 的配置文件，修改 Group B 的 critic terms：

```python
# 配置 A：symmetric（critic = actor）
# env_cfg_symmetric.py
critic_terms = {**actor_terms}  # critic 和 actor 完全相同

# 配置 B：asymmetric（critic > actor）
# env_cfg_asymmetric.py
critic_terms = {
    **actor_terms,  # 包含 actor 所有 terms
    # ——— 额外 privileged terms ———
    "clean_height_scan": ObservationTermCfg(
        func=mdp.height_scan,
        params={"sensor_cfg": SceneEntityCfg("height_scanner")},
        # 注意：无噪声
    ),
    "foot_contact_forces": ObservationTermCfg(
        func=mdp.contact_forces_z,
        params={"sensor_cfg": SceneEntityCfg("contact_sensor")},
    ),
    "friction_coeffs": ObservationTermCfg(
        func=mdp.friction_coefficients,
    ),
}
```

**Step 2：在相同条件下训练两组。** 控制变量：相同的 seed、num_envs、max_iterations、learning_rate、reward terms。唯一差异是 critic 的 observation。

```bash
# 组 A：symmetric baseline
uv run train Mjlab-Velocity-Rough-Go1-Symmetric \
    --env.scene.num-envs 4096 --agent.max-iterations 5000 \
    --agent.seed 42 --agent.logger tensorboard

# 组 B：asymmetric
uv run train Mjlab-Velocity-Rough-Go1-Asymmetric \
    --env.scene.num-envs 4096 --agent.max-iterations 5000 \
    --agent.seed 42 --agent.logger tensorboard
```

**Step 3：对比分析。** 训练完成后，对比以下指标：

```python
# 对比脚本
import tensorboard as tb
from tbparse import SummaryReader

def compare_runs(log_dir_a, log_dir_b):
    """对比 symmetric vs asymmetric 训练曲线。"""
    reader_a = SummaryReader(log_dir_a)
    reader_b = SummaryReader(log_dir_b)

    metrics = [
        "Loss/value_function",      # 预期 B 显著低于 A
        "Train/mean_reward",        # 预期 B 收敛更快
        "Train/mean_episode_length",# 预期 B 更长（机器人存活更久）
    ]

    for metric in metrics:
        df_a = reader_a.scalars[reader_a.scalars.tag == metric]
        df_b = reader_b.scalars[reader_b.scalars.tag == metric]
        print(f"\n{metric}:")
        print(f"  Symmetric (A): final = {df_a.value.iloc[-1]:.4f}")
        print(f"  Asymmetric (B): final = {df_b.value.iloc[-1]:.4f}")
```

**预期观察：**

- **Value loss**：组 B（asymmetric）的 value loss 从第 100 iteration 开始显著低于组 A——critic 有更多信息，value 估计更准确
- **Reward 收敛**：组 B 在 ~1000 iterations 达到组 A 在 ~3000 iterations 才能达到的 reward 水平——policy gradient 方向更精准
- **Episode length**：组 B 的 mean episode length 在训练中期比组 A 长 30-50%——机器人更少摔倒

**如果实验结果与预期不符：**

1. 如果两组差异很小（<5%）：说明 actor 的 observation 对此任务已经足够（可能是 flat terrain，信息差本身就小）。换成 rough terrain 重新实验
2. 如果组 A 反而更好：检查 critic 的 privileged terms 是否正确连接。最常见的错误是 term 名拼写错误导致 critic 实际没有获得额外信息
3. 如果两组都不收敛：问题不在 privileged learning，回到 Ch06 检查 reward 设计

这个 A/B 实验是理解 privileged learning 工程价值的最直接方式。它不需要 teacher-student 蒸馏或 adaptation module——只需要修改 critic 的 observation 配置。建议每个学生在开始更复杂的蒸馏实验前先完成这个基础实验。

### 实验记录模板

```text
实验：Asymmetric AC 消融
━━━━━━━━━━━━━━━━━━━━━
日期：
机器人：Go1 / Go2 / G1（选择一个）
框架：mjlab / Isaac Lab
地形：Flat / Rough（选择一个）
GPU：
num_envs：4096
max_iterations：5000
seed：42

组 A（symmetric）：
  actor obs dim: ___
  critic obs dim: ___（应 = actor）
  final reward: ___
  final value loss: ___
  final episode length: ___

组 B（asymmetric）：
  actor obs dim: ___
  critic obs dim: ___（应 > actor）
  额外 privileged terms: [列出]
  final reward: ___
  final value loss: ___
  final episode length: ___

结论：
  reward 提升比例: ___% 
  value loss 降低比例: ___%
  是否验证了 asymmetric AC 的价值：是/否
  下一步：是否需要升级到 RMA 或蒸馏？
```

## 延伸阅读

| 资料 | 难度 | 推荐原因 |
|------|------|---------|
| Pinto et al., RSS 2018, "Asymmetric Actor Critic for Image-Based Robot Learning" | ⭐⭐ | asymmetric actor-critic 的原始论文，建立了基本框架 |
| Kumar et al. 2021, "RMA: Rapid Motor Adaptation for Legged Robots" | ⭐⭐⭐ | RMA 的完整方法、adaptation module 设计和真机实验 |
| Lee et al. 2020, "Learning Quadrupedal Locomotion over Challenging Terrain" | ⭐⭐ | 四足 privileged learning 的经典工作，teacher-student 蒸馏的早期范例 |
| Cheng et al. 2024, "Extreme Parkour with Legged Robots" (ICRA'24) | ⭐⭐⭐ | 本章精读项目，三阶段蒸馏管线的工业级实现 |
| RSL-RL 文档：obs_groups 和 DistillationRunner | ⭐⭐ | mjlab 使用的训练框架的蒸馏支持，工程实现的直接参考 |
| Rudin et al. 2022, "Learning to Walk in Minutes" | ⭐⭐ | 大规模并行训练的工程细节，asymmetric AC 的实践经验 |
| Miki et al. 2022, "Learning robust perceptive locomotion for quadrupedal robots in the wild" (Science Robotics) | ⭐⭐⭐ | depth + attention encoder + privileged learning；ANYmal 1700m 零跌倒；后续视觉腿足工作的基础 |
| Radosavovic et al. 2024, "Humanoid Locomotion as Next Token Prediction" (Science Robotics) | ⭐⭐⭐ | Causal transformer 隐式适应，RMA 的替代范式；理解 9.4 节三种范式对比 |
| He et al. 2025, "HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots" (ICRA'25) | ⭐⭐⭐⭐ | Mask-conditioned distillation；一个 student 支持多种控制模式 |
| Huang et al. 2025, "HoST: Learning Humanoid Standing-up Control across Diverse Postures" (RSS'25) | ⭐⭐⭐ | Multi-critic 架构；理解 9.1 节 multi-critic 变体 |
| Schwarke et al. 2025, "RSL-RL: A Learning Library for Robotics Research" (arXiv 2509.10771) | ⭐⭐ | RSL-RL 4.0 的 actor/critic 解耦架构、DistillationRunner、ONNX exporter |

**阅读顺序建议**：先读 Pinto 2018（理解 asymmetric AC 的基本原理），再读 Kumar 2021（理解 RMA 的两阶段框架），然后读 Cheng 2024（理解多阶段蒸馏的完整管线）。在此基础上读 HOVER 2025（理解 mask-conditioned distillation 如何统一多种控制模式）。Miki 2022 和 Radosavovic 2024 作为"从盲控制到感知控制"和"显式 vs 隐式适应"的对比阅读材料。RSL-RL 4.0 论文中关于 obs_groups、DistillationRunner 和 ONNX exporter 的工程说明应作为持续参考。

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| sim 表现好但真机崩溃 | actor 信息泄漏 | 1. 逐项审查 actor_terms 2. 检查自定义 term 实现 3. 用 15 项 checklist 排查 | 本章 9.3 |
| value loss 长期偏高 | critic 缺少 privileged 信息 | 1. 比较 actor/critic 维度 2. 检查 obs_groups routing 3. 添加 clean contact/height | 本章 9.3 |
| student imitation loss 不降 | teacher 行为不可蒸馏 | 1. 检查 teacher 是否用了未来信息 2. 限制 teacher privilege 3. 增加 student history | 本章 9.5 |
| student loss 低但 rollout 差 | compounding error | 1. 检查训练/评估数据分布差异 2. 增加蒸馏数据多样性 3. 考虑 DAgger 或 RL fine-tuning | 本章 9.5 |
| history 加大后无改善且显存暴涨 | history 加在了无意义的 terms 上 | 1. 只给 proprioception 加 history 2. 减小 history_length 3. 评估收益/成本比 | 本章 9.4 |
| critic 和 actor obs 维度相同 | obs_groups routing 错误或 key 拼写错误 | 1. 打印两者维度 2. 检查 obs_groups 配置 3. 确认 critic group 有额外 terms | 本章 9.3 |
| Stage 2 warm start 后 reward 暴跌 | 网络权重加载维度不匹配 | 1. 检查 load_state_dict 的 strict 参数 2. 确认只加载兼容层的权重 3. CNN 层应随机初始化 | 本章 9.5 |
| RMA latent 维度过大训练不稳定 | adaptation module 拟合困难 | 1. 从 4 维开始实验 2. 检查 encoder latent 的方差 3. 降低 latent 维度 | 本章 9.4 |
| play 模式 actor 仍有噪声 | 自定义脚本未关闭 corruption | 1. 打印 actor group 的 enable_corruption 2. 检查 play.py 的 override 逻辑 3. 手动设为 False | 本章 9.3 |
| adaptation module MSE 低但 policy rollout 差 | latent 过拟合训练分布 | 1. 检查训练数据的 DR 覆盖度 2. 在新 DR 参数上测试 MSE 3. 增加训练数据多样性 | 本章 9.4 |
| 蒸馏数据收集时 reward 突然为零 | teacher checkpoint 加载失败 | 1. 打印 teacher 权重的 key 名 2. 检查 checkpoint path 是否指向正确阶段 3. 验证 teacher 在收集模式下的 rollout 表现 | 本章 9.5 |
| Isaac Lab 中 critic obs 维度与预期不符 | CriticCfg 的 terms 拼写错误 | 1. 检查 term 名是否与 ObservationManager 注册名一致 2. 打印 ObservationManager.group_obs_dim 3. 对比配置文件与实际加载的 terms | 本章 9.3 |
| mjlab 的 obs_groups 返回相同 obs | obs_groups 配置没有传给 env | 1. 检查 env_cfg.yaml 中 obs_groups 是否存在 2. 打印 env.obs_group_shapes 3. 确认 runner 版本支持 obs_groups | 本章 9.3 |
| 多阶段训练中后续 stage reward 持续低于前 stage | stage 间的 DR 或 reward 配置不一致 | 1. 对比各 stage 的 reward terms 和权重 2. 确认 DR 参数范围一致 3. 检查 warm start 是否成功加载 | 本章 9.5 |
| 训练前几百 iter value loss 异常高后缓慢恢复 | checkpoint 恢复时 normalizer 状态丢失 | 1. 确认 normalizer state_dict 被加载 2. 检查 running_mean/running_var shape 3. 如不可恢复则重置 normalizer 并预热 200 iter | 本章 9.3 |
| 新增 obs term 后 critic value 突然暴跌 | normalizer buffer 新维度的 mean/std 未初始化 | 1. 检查 normalizer 的 running_var 新维度是否为 0 或极小 2. 重新开始训练或手动初始化新维度统计量 3. 确认 actor normalizer 未受影响 | 本章 9.3 |
| ONNX 部署时 shape error 或垃圾输出 | 导出了 critic 的 normalizer 或 teacher 的网络 | 1. 打印 ONNX 模型输入 shape 2. 对比部署传感器维度 3. 确认 `export_policy_as_onnx` 使用的是 actor/student 的 normalizer | 本章 9.5 |
| AMP 判别器仅凭归一化差异区分真假 | actor 与 discriminator 使用了不同的 normalizer | 1. 检查两者是否共享同一个 normalizer 实例 2. 打印 discriminator 和 actor 的 running_mean 对比 3. 强制共享 normalizer | 本章 9.3 + Ch10 |
