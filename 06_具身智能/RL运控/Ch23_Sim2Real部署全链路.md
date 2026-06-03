# 第 23 章 Sim2Real 部署全链路

## 前置自测

📋 **答不出 $\ge$ 2 题 → 先回前置章节复习**

1. **[Ch18 视觉控制]** 视觉策略的 domain randomization 解决的是什么问题？它不能解决什么问题？
2. **[控制基础]** 50 Hz 控制频率下一帧延迟是多少 ms？两帧延迟对步态相位意味着什么？
3. **[导出]** ONNX 文件包含什么？不包含什么？为什么不包含状态估计器和安全限幅？
4. **[部署安全]** action clip 和急停（emergency stop）解决的是同一个问题吗？为什么不是？
5. **[系统辨识]** System Identification（SysID）和 Domain Randomization（DR）分别从哪个方向缩小 sim2real gap？

## 本章目标

学完本章后，你应该能够：

1. **分类** sim2real gap 的来源——按动力学、接触、执行器、传感器、时序、软件接口和安全七个层次穷举
2. **对比** SysID、DR、Fine-tuning 和 Adaptive Control 四种方法的理论基础、适用条件和局限性
3. **理解** mjlab 的 ONNX 导出链路——checkpoint 保存了什么、metadata 包含什么、部署端必须补齐什么
4. **设计**从 D0（静态核对）到 D6（完整任务）的分级验收流程，理解每个等级的风险边界
5. **实施**延迟补偿策略——区分 observation latency、policy latency、action transport latency 和 actuator latency

---

## 23.1 Sim2Real Gap 的系统性分类 ⭐⭐

> **这一节解决什么问题**：sim2real gap 应该怎样穷举分类？为什么"domain randomization 解决一切"是危险的简化？

### 为什么需要系统性分类 ⭐⭐

Sim2real 的失败很少是因为"仿真不够好"这样一个笼统的原因。每次失败都有具体的物理来源——可能是质量参数不对（动力学 gap），可能是地面摩擦系数不对（接触 gap），可能是电机响应慢了 5 ms（执行器 gap），可能是 IMU 有漂移（传感器 gap），可能是控制频率不稳定（时序 gap），可能是关节顺序写反了（软件接口 gap），也可能是没有急停保护（安全 gap）。

如果不做分类而把所有问题都叫"sim2real gap"，工程上会导致两个后果。第一，无法定位问题：当真机失败时，你不知道应该去调摩擦系数、还是去修通信延迟、还是去检查关节映射。第二，无法选择正确的解决工具：SysID 解决的是"参数不准"，DR 解决的是"参数不确定"，Fine-tuning 解决的是"模型结构不对"——用 DR 去解决关节顺序写反了的问题，就像用抗生素治骨折一样文不对题。

> **跨领域类比：Sim2Real gap 分类与医学诊断的鉴别诊断。** 病人说"头疼"，医生不会直接开止痛药——而是先按系统分类排查：是神经系统（偏头痛）、血管系统（高血压）、感染（脑膜炎）还是外伤？每种原因对应不同的治疗方案。Sim2real 的"头疼"（真机失败）也必须先分类诊断，再对症下药。

### 七层 Gap 分类体系 ⭐⭐

| Gap 层 | 具体内容 | mjlab 侧入口 | 部署侧入口 | 典型解决工具 |
|--------|---------|-------------|-----------|-------------|
| **动力学** | 质量、惯量、摩擦、关节阻尼 | DR events、SysID 参数 | 称重、辨识、日志拟合 | SysID + DR |
| **接触** | 足底摩擦、碰撞形状、软接触 | geom friction、contact cfg | 地面测试、脚底材料 | SysID + DR |
| **执行器** | PD gain、力矩限制、死区、饱和 | actuator cfg、action scale | SDK 控制模式、温度限制 | SysID |
| **传感器** | IMU bias、encoder noise、depth noise | noise cfg、camera DR | 标定、滤波、同步 | DR + 标定 |
| **时序** | 控制周期、通信延迟、图像延迟 | decimation、delay buffer | timestamp、实时线程 | 延迟注入 + 测量 |
| **软件接口** | joint order、单位、坐标系 | ONNX metadata | adapter 和 schema test | 单元测试 |
| **安全** | 限幅、急停、跌倒检测 | termination/reward 只训练倾向 | 硬件 watchdog | 外部安全系统 |

这七层之间不是独立的——它们会耦合。例如，执行器延迟（时序 gap）会放大摩擦模型误差（接触 gap）的影响：如果控制器对滑动的响应晚了 20 ms，即使摩擦模型完全准确，机器人也可能因为响应不及时而滑倒。这种耦合意味着 sim2real 不能只修一个 gap——必须把所有 gap 控制在策略能承受的联合范围内。

> **本质洞察**：Sim2real 不是缩小一个 gap。它是把多个误差源限制在策略能承受的联合分布内。如果只调摩擦，部署仍可能因为延迟失败。如果只调质量，部署仍可能因为相机外参失败。如果只导出 ONNX，部署仍可能因为 joint order 或 action scale 失败。

### 如果不做 Gap 分类会怎样 ⭐⭐

一个真实的场景：一个四足机器人的策略在仿真中能稳定行走，但真机上走了三步就摔倒。团队花了两周时间把 domain randomization 的范围从"摩擦 0.5-1.5"扩大到"摩擦 0.1-2.0"，重新训练了五轮——每轮训练成本数小时 GPU 时间。结果真机表现毫无改善。最终发现问题是关节顺序映射错了一位——左前腿的膝关节动作被发送到了左前腿的髋关节。这个 bug 用 DR 永远无法解决，因为它不是参数不确定性问题，而是确定性的软件错误。如果一开始就按七层分类逐层排查，用单关节小幅动作测试（软件接口层）就能在五分钟内发现问题。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：joint order 映射错位**

仿真中的关节顺序由 MJCF/URDF 定义，真实 SDK 的关节顺序由硬件协议定义——两者几乎不可能完全一致。如果不写显式的映射表并通过单关节测试验证，就会出现"动作数值正确但发给了错误的关节"的情况。这种 bug 在 reward 曲线上完全看不出来，因为训练发生在仿真中，关节顺序是正确的。正确做法：导出 ONNX metadata 中的 `joint_names`，在部署端与 SDK 的关节名称逐一比对，然后用单关节小幅正弦波驱动每个关节来目视确认。

💡 **概念误区：认为"仿真越逼真 sim2real gap 越小"**

直觉上这是对的，但在实践中有一个重要的例外。过于逼真的仿真可能导致策略过度利用仿真器的特定行为——比如某个接触求解器的数值特性、某种碰撞检测的边界情况。当真实世界的物理行为与仿真器的数值行为不同时（即使"更真实"），策略反而失效。一定程度的 domain randomization 实际上比极度逼真的仿真更有利于 sim2real——因为它迫使策略学到对物理参数鲁棒的行为，而不是利用特定参数设置下的"捷径"。

🧠 **思维陷阱：按 gap 大小排序解决**

初学者倾向于先修"最大的 gap"——比如如果摩擦 gap 导致了最多的失败，就先花大量时间做摩擦辨识。但实际上应该先修**最容易修的 gap**——软件接口 gap（关节顺序、单位转换）的修复成本极低（几分钟到几小时），但如果不修，其他所有优化都是白费的。正确的排查顺序是：软件接口 -> 时序 -> 执行器 -> 传感器 -> 接触 -> 动力学 -> 安全，从最确定性的问题到最不确定性的问题。

### 练习

1. **[分类题]** 一个机械臂策略在仿真中能抓取方块，但真机总是抓偏 2 cm 向左。列出至少三个可能的 gap 层次和对应的排查步骤。
2. **[设计题]** 为 Unitree Go2 四足机器人写一份 sim2real gap 排查清单。按七层分类，每层列出 2-3 个具体的检查项。
3. **[思考题]** 为什么安全 gap 不能通过 domain randomization 解决？用一个具体的例子说明。提示：考虑"急停按钮不工作"这个 gap——它是参数不确定性还是确定性的工程缺陷？
4. **[排查题]** 一个 Go2 四足机器人在仿真中能以 0.8 m/s 稳定行走，但真机上以 0.3 m/s 就摔倒。按七层分类写出排查计划：每层检查什么、用什么工具、预期需要多长时间。

理解了 gap 的分类之后，下一步是理解每种 gap 对应的最佳解决工具。不同工具解决不同层面的问题——选错工具不仅浪费时间，还可能让问题更难诊断。

---

## 23.2 四种 Sim2Real 方法的理论基础 ⭐⭐⭐

> **这一节解决什么问题**：SysID、Domain Randomization、Fine-tuning 和 Adaptive Control 各自的数学基础是什么？它们解决 gap 的哪个层面？什么时候应该用哪种方法？

### System Identification：让仿真均值对齐真实 ⭐⭐

System Identification（SysID）的目标是测量或估计真实系统的物理参数（质量、惯量、摩擦系数、PD gain、延迟等），然后把仿真器的参数设为这些测量值。数学上，SysID 试图找到参数 $\phi^*$ 使得仿真行为和真实行为的差异最小：

$$\phi^* = \arg\min_\phi \sum_t \| x_{\text{sim}}(t; \phi) - x_{\text{real}}(t) \|^2$$

SysID 的核心假设是：仿真器的模型结构是正确的，只是参数不准。如果这个假设成立，找到正确的参数就能消除 gap。SysID 的优点是直接减少仿真均值与真实值的偏差，使得 DR 可以用更小的范围覆盖残余不确定性——训练更高效，策略性能更好。SysID 的局限是：某些参数随时间变化（温度升高后摩擦系数改变、电池电压下降后电机力矩下降），辨识结果只代表辨识时刻的系统状态。

> **跨领域类比：SysID 与乐器调音。** 钢琴调音师测量每根弦的实际频率，然后调整张力使其与标准频率对齐。这不会让钢琴变成完美乐器（还有触键感、泛音等差异），但会消除最大的误差来源——音高偏差。SysID 做的是同样的事情：不追求完美匹配，但消除最大的参数偏差。

### Domain Randomization：让策略对参数不敏感 ⭐⭐⭐

回顾 Ch22 中 DR 的理论基础。在 sim2real 的语境下，DR 不仅随机化视觉参数，还随机化物理参数——质量、摩擦、PD gain、延迟等。其核心假设是：如果策略能在一个足够宽的参数分布上都表现良好，那么真实世界的参数（作为分布中的一个点）也应该被覆盖。

DR 与 SysID 是互补的，不是替代的。SysID 把仿真参数的均值对齐到真实值附近，DR 在这个均值周围添加不确定性范围。如果不做 SysID 就直接做 DR，需要用很宽的范围来覆盖"均值偏差 + 参数不确定性"——训练更困难，策略性能更差。如果只做 SysID 不做 DR，策略对参数的微小变化（温度、磨损、负载变化）毫无抵抗力——在辨识条件下完美，换个条件就失败。

> **本质洞察**：SysID 和 DR 的关系就像射击中的"归零"和"散布"。归零（SysID）让弹着点的中心对准靶心——消除系统性偏差。散布训练（DR）让射手在不同风速和距离下都能命中——增加鲁棒性。只归零不练散布，换个风就脱靶。只练散布不归零，平均弹着点偏了一个靶宽——再怎么鲁棒也够不着靶心。

### Fine-tuning：用真实数据修正残差 ⭐⭐⭐

当仿真器的模型结构本身存在系统性偏差时——比如刚体接触模型无法捕捉真实的软接触行为，或者简化的摩擦模型无法表达真实的各向异性摩擦——无论怎么调参数或扩大随机化范围都无法消除这种结构性 gap。Fine-tuning 的思路是：先在仿真中预训练一个接近可用的策略，然后在真实机器人上采集少量数据来修正策略中与模型结构偏差相关的部分。

$$\pi_{\text{fine}} = \arg\max_\pi \mathbb{E}_{\text{real}} \left[ \sum_t R(s_t, a_t) \right], \quad \pi_0 = \pi_{\text{sim}}$$

Fine-tuning 的优点是能处理 SysID 和 DR 都无法覆盖的结构性 gap。缺点是需要在真实机器人上采样——这既昂贵（机器人使用时间、人工监督）又危险（策略在早期可能做出不安全的动作）。因此 fine-tuning 通常是 sim2real 流程的最后一步，在 SysID + DR 已经把大部分 gap 消除之后，用少量真实数据修正残余偏差。

如果在没有充分 SysID 和 DR 的情况下直接做 fine-tuning 会怎样？策略的初始性能太差，真实采样中大部分轨迹都是失败的——有效梯度极少，fine-tuning 收敛极慢。更糟糕的是，不安全的动作可能损坏机器人。正确的流程是：SysID（消除均值偏差）-> DR（覆盖参数不确定性）-> 仿真验证 -> 分级真机测试 -> Fine-tuning（修正残余偏差）。

> **跨领域类比：Fine-tuning 与汽车赛道调校。** 赛车手不会拿一辆工厂设置的新车直接去比赛——先在工厂做基础调校（SysID：引擎参数、悬挂几何），再在多种路面条件下测试（DR：不同温度、不同轮胎），最后到实际赛道做精细调整（Fine-tuning：根据赛道的具体弯道和路面做微调）。跳过前两步直接上赛道调校，车手可能连一圈都跑不完——更不用说找到最优设置了。

Fine-tuning 还有一个实践中的重要考虑：**安全约束更严格**。在仿真中训练可以让策略自由探索——摔倒了就 reset，没有代价。在真机上 fine-tuning 时，每一步探索都有物理代价——摔倒可能损坏传感器，猛烈动作可能伤人。因此真机 fine-tuning 通常需要加入额外的安全约束：action 的变化率限制（防止突然猛动）、姿态保护阈值（接近摔倒就停止）、力矩上限（防止电机过载）。这些约束会降低 fine-tuning 的效率（探索空间变小），但它们是保护硬件和人员的前提。

### Adaptive Control：在线适应变化 ⭐⭐⭐

以上三种方法都是离线的——在部署前完成，部署时策略参数固定不变。但真实环境会变化：地面从瓷砖变成地毯（摩擦变化）、机器人抓了一个重物（负载变化）、电机温度升高（增益变化）、电池电量下降（力矩上限变化）。Adaptive control 在运行时在线估计环境参数并调整控制行为。

在 RL 框架中，adaptive control 通常通过 latent variable 实现：策略网络接收一个额外的低维"环境编码" $z$，这个编码由一个在线估计器从最近的 observation-action 历史中推断：

$$z_t = \text{Estimator}(o_{t-k:t}, a_{t-k:t-1})$$
$$a_t = \pi(o_t, z_t)$$

训练时，estimator 和 policy 在随机化环境中联合训练——estimator 学会从交互历史中推断当前环境参数（即使不直接观测），policy 学会根据推断结果调整行为。

Adaptive control 的优点是能应对部署后的环境变化，不需要每次变化都重新训练。局限是在线估计需要一段"热身期"（estimator 需要足够的历史数据才能准确推断），在热身期内策略的行为可能不稳定——需要保守的安全边界来覆盖这个过渡期。

### Distillation 在部署中的角色 ⭐⭐

回顾 Ch22 的 teacher-student 蒸馏。在 sim2real 的语境下，distillation 不仅是一种训练技术——它是一种 **deployment gap 的管理工具**。Teacher 使用仿真中的特权信息（精确物体位姿、ground-truth 接触状态、segmentation mask），这些信息在真机上不可获得。Student 只使用部署时可获得的输入（RGB/depth 图像、proprioception、IMU）。蒸馏过程确保 student 能从可部署的输入中重建足够的信息来完成任务——本质上是把"特权信息"编码到了 student 的网络权重中。

从 sim2real gap 分类的角度看，distillation 主要解决的是 **传感器 gap 中的"不可获得性"问题**——不是传感器噪声或延迟的问题，而是"某些量在真机上根本不存在"的问题。这是 SysID 和 DR 都无法解决的一类 gap：你不能通过辨识或随机化来让真机"长出" ground-truth segmentation。

Distillation 的部署边界：student 的输入必须完全由部署时可获得的传感器提供。如果 student 的输入中混入了任何仿真特权信息（即使是间接的，如通过 observation normalization 的 running mean 编码了特权信息的统计量），student 在真机上的行为就不可预测。验证方法：在 student 训练完成后，逐一检查其 observation schema 的每个维度，确认每个维度在真机上有明确的来源（哪个传感器、什么单位、什么采样率）。

### Classical Safety 不可或缺 ⭐⭐

在四种学习方法之外，还有一类不依赖学习的方法——经典安全保护（限幅、急停、监控）。这些方法不解决 sim2real gap，但它们保证在 gap 导致策略失效时机器人不会损坏或伤人。Ch23.5 会详细讨论安全链路，这里只强调一个关键原则：**安全系统不能依赖策略的正确性**。如果策略因为 sim2real gap 而输出不安全的动作，安全系统必须独立地检测并阻止这些动作——即使策略"认为"它在做正确的事情。

### 四种方法的决策流程 ⭐⭐

| 条件 | 推荐方法 | 理由 |
|------|---------|------|
| 参数可测量、不随时间变 | SysID | 直接消除偏差，最高效 |
| 参数不可测量或有不确定性 | DR | 覆盖不确定性范围 |
| 模型结构有系统偏差 | Fine-tuning | 参数调整无法消除结构差异 |
| 参数在部署中会变化 | Adaptive | 离线方法无法应对在线变化 |
| 日常部署 | SysID + DR + Adaptive | 组合使用，各解决一部分 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：DR 范围太宽导致训练不收敛**

把摩擦系数随机化到 $[0.01, 5.0]$ 的范围会让训练极其困难——因为 $\mu = 0.01$ 相当于冰面（几乎无摩擦），而 $\mu = 5.0$ 相当于粘合剂（几乎无滑动），策略需要同时处理两个完全不同的物理世界。正确的范围应该基于 SysID 的测量结果和工程判断——如果真实摩擦约 0.5-0.8，随机化到 $[0.3, 1.2]$ 通常足够。

💡 **概念误区：认为"Fine-tuning 可以替代 SysID 和 DR"**

这种想法忽略了 fine-tuning 的前提——策略的初始性能必须"接近可用"才能高效采样。如果初始策略在真机上完全不能走（因为没做 SysID 和 DR），fine-tuning 的采样全是失败轨迹，梯度无法指向正确的方向。Fine-tuning 是打磨工具，不是万能工具。

🧠 **思维陷阱：认为"所有 gap 都能通过训练解决"**

软件接口 gap（关节顺序错误）和安全 gap（缺少急停）不是训练问题——它们是确定性的工程错误，只能通过代码修正和硬件配置来解决。把这些问题交给 RL 训练去"适应"，就像让学生通过做更多习题来适应考卷印错了的情况一样荒谬。

> **反事实推理：如果只用 DR 不做 SysID 会怎样？** 假设真实摩擦系数是 0.6，但你不做 SysID，所以你不知道这个值。你猜测摩擦可能在 0.2-1.5 之间，用这个范围做 DR。训练出的策略确实对摩擦鲁棒——但代价是什么？(1) 策略在 $\mu=0.6$ 附近的性能比 SysID + 窄 DR 的策略差 15-30%（因为它要"兼顾"太宽的范围）；(2) 训练时间增加 2-3 倍（因为策略需要在更大的参数空间上收敛）。如果先做 SysID 测得 $\mu\approx0.6$，DR 范围设为 [0.4, 0.8]——训练更快，策略在真实值附近更优化。

> **反事实推理：如果在部署时不做 D0 检查直接上真机会怎样？** 一个真实案例的复盘：团队花了 3 天训练了一个优秀的人形策略（reward 持续上升、仿真中行走稳健），兴奋地接上真机 G1——第一步就摔倒并损坏了一个膝关节电机（维修费 ¥3000 + 等零件 2 周）。事后排查发现原因极其简单：ONNX 导出时 obs normalizer 的 mean/std 没有被包含在文件中，部署时 obs 未归一化，策略看到的是量级错误的输入，输出了最大力矩的动作。如果做了 D0 检查（5 分钟），会发现 "obs_mean not found in metadata" 的警告——避免了 ¥3000 和 2 周的损失。

### 练习

1. **[分析题]** 一个四足机器人在瓷砖地面训练时表现完美，但在地毯上摔倒。按四种方法分别提出解决方案，并分析每种方案的优劣。
2. **[设计题]** 你要部署一个手臂抓取策略。真实 gripper 的闭合力比仿真大 30%，导致每次都把物体捏碎。SysID、DR、Fine-tuning 哪种最适合？为什么？
3. **[跨章综合题]** 回顾 Ch18 的视觉 domain randomization。视觉 DR（光照、纹理）和物理 DR（质量、摩擦）在 sim2real 中分别覆盖了哪些 gap 层？画出两者的覆盖范围图。

### ASAP：学习残差动作模型 ⭐⭐⭐

前四种方法（SysID、DR、Fine-tuning、Adaptive）是经典的 sim2real 工具箱。2025 年出现了一种新的方法——**ASAP**（He et al., RSS 2025, arXiv:2502.01143）——它结合了 SysID 的思想和学习方法，通过**学习一个残差动作模型**来弥合 sim2real gap。

ASAP 的核心假设是：仿真器的物理模型大体正确，但在某些维度上存在系统性偏差——这种偏差可以用一个**可学习的残差函数** $\delta(s, a)$ 来补偿。具体来说，如果在仿真中执行动作 $a$ 得到的下一状态是 $s_{t+1}^{\text{sim}}$，而在真机上执行同样的动作得到的是 $s_{t+1}^{\text{real}}$，两者之间的差异可以通过修改动作来弥补：

$$s_{t+1}^{\text{sim}}(a + \delta(s, a)) \approx s_{t+1}^{\text{real}}(a)$$

**ASAP 的三阶段 pipeline**：

```
Phase 1: 仿真预训练（标准 PPO + DR）
──────────────────────────────
输出: 预训练策略 π_sim

Phase 2: Delta-A Open Loop（数据采集 + 残差学习）
──────────────────────────────────────────
步骤 2a: 用 π_sim 在真机上采集 rollout 数据
        → 记录 (s_t, a_t, s_{t+1}^real)
步骤 2b: 在仿真中重放相同的 (s_t, a_t)
        → 记录 s_{t+1}^sim
步骤 2c: 训练残差网络 δ(s, a)
        → minimize ‖s_{t+1}^sim(a + δ) - s_{t+1}^real‖²

Phase 3: Delta-A Closed Loop Finetune
──────────────────────────────────
步骤 3a: 冻结 $\delta$ 网络
步骤 3b: 在仿真中注入 δ：每步执行 a + δ(s, a) 而非 a
步骤 3c: 继续用 PPO 在修正后的仿真中训练
        → 策略学会在"真实化"的仿真中行动
```

```python
# ASAP delta-action 残差网络（简化实现）
class DeltaActionModel(nn.Module):
    """学习 sim-to-real 的 action 残差。"""
    def __init__(self, obs_dim, act_dim, hidden_dims=[256, 128]):
        super().__init__()
        layers = []
        prev_dim = obs_dim + act_dim  # 输入: obs 和 action 拼接
        for h in hidden_dims:
            layers.extend([nn.Linear(prev_dim, h), nn.ELU()])
            prev_dim = h
        layers.append(nn.Linear(prev_dim, act_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, obs, action):
        """预测 action 残差 δ(s, a)。"""
        x = torch.cat([obs, action], dim=-1)
        delta = self.net(x)
        return delta  # 修正后的动作 = action + delta

# Phase 2: 训练 delta 网络
def train_delta_model(
    sim_data: dict,    # {obs, action, next_obs_sim}
    real_data: dict,   # {obs, action, next_obs_real}
    epochs: int = 100,
):
    """用 real-sim 差异训练残差网络。"""
    model = DeltaActionModel(obs_dim, act_dim)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

    for epoch in range(epochs):
        delta = model(sim_data['obs'], sim_data['action'])
        corrected_action = sim_data['action'] + delta
        # 在仿真中用修正动作预测下一状态
        predicted_next = sim_step(sim_data['obs'], corrected_action)
        # 损失: 修正后的仿真下一状态 ≈ 真实下一状态
        loss = F.mse_loss(predicted_next, real_data['next_obs_real'])
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return model
```

**ASAP vs 传统方法的关键差异**：

| 维度 | SysID | DR | ASAP |
|------|-------|-----|------|
| 修正对象 | 仿真参数 | 策略鲁棒性 | 仿真动作 |
| 需要真机数据 | 是 | 否 | 是（少量） |
| 能处理模型结构偏差 | 否 | 部分 | 是 |
| 部署时的修改 | 无 | 无 | 需要携带 $\delta$ 网络 |
| 计算成本 | 低 | 训练成本增加 | 额外 $\delta$ 网络推理 |

ASAP 在 Unitree G1 上的实验表明，它在高动态动作（如跳跃、旋转）上的 sim2real 效果显著优于纯 DR——tracking error 降低 30-50%。但它需要在真机上采集 rollout 数据（Phase 2a），这意味着策略在 Phase 1 结束后必须"勉强能用"——如果 Phase 1 的策略在真机上完全不能走，Phase 2a 采集到的都是失败轨迹，$\delta$ 网络学不到有意义的残差。

> **反事实推理：如果不做 Phase 1 直接采集真机数据会怎样？** 没有预训练的策略在真机上输出随机动作——机器人立即摔倒。采集到的数据只反映了"从站立到摔倒"这个极短的过渡过程，$\delta$ 网络只能学到"在即将摔倒时如何修正动作"这个狭窄的场景。Phase 1 的预训练确保了策略在真机上能至少行走几秒钟，产生覆盖正常运动范围的数据——这是 $\delta$ 网络有效学习的前提。

---

## 23.3 ONNX 导出：策略文件包含什么、不包含什么 ⭐⭐

> **这一节解决什么问题**：mjlab 导出的 ONNX 文件边界在哪里？部署端必须自己补齐什么？

### ONNX 导出的精确边界 ⭐⭐

mjlab 的 runner 在训练过程中导出 ONNX 文件。理解这个文件的精确边界至关重要——因为大多数部署事故不是策略本身的问题，而是部署端对 ONNX 的使用方式与训练不一致。

ONNX 文件**包含**的内容：策略网络（actor）的完整计算图——从 observation 张量到 action 张量的映射。如果策略包含 CNN encoder，CNN 的权重也在其中。ONNX 还可以包含 metadata——joint_names、observation_names、action_scale、default_joint_pos 等，这些信息帮助部署端正确地映射输入输出。

ONNX 文件**不包含**的内容：状态估计器（把 IMU 读数变成 base velocity 的算法）、observation normalizer 的 running mean/std（如果训练时开启了 observation normalization）、安全限幅逻辑、PD 控制器参数、通信协议、相机采集线程、机器人 SDK adapter。

> **跨领域类比：ONNX 文件与汽车发动机。** ONNX 是发动机——它能把燃油（observation）变成动力（action）。但发动机不是汽车。你还需要传动系统（action scale 和关节映射）、刹车系统（安全限幅和急停）、油箱和油泵（传感器采集和预处理）、方向盘和仪表盘（状态估计和人机接口）。把发动机装上底盘之前，必须确认所有其他系统都已就位。

这个理解防止了一个常见错误：认为"训练好了就可以部署了"。训练产出的是一个函数 $a = \pi(o)$，部署需要的是一个闭环系统：

```text
真实闭环 = 传感器采集 -> 预处理 -> 状态估计 -> observation 拼接
         -> ONNX 推理 -> action 后处理 -> 安全过滤 -> SDK 发送
         -> 执行器响应 -> 物理世界变化 -> 传感器采集（回到起点）
```

ONNX 只覆盖了中间的 "ONNX 推理" 这一步。其他每一步都需要部署端自己实现和验证。

### mjlab 导出链路源码映射 ⭐⭐

| 组件 | 源码路径 | 关键内容 |
|------|---------|---------|
| ONNX 导出函数 | `src/mjlab/rl/runner.py` | `export_policy_to_onnx` 导出 actor 计算图 |
| Checkpoint 保存 | `src/mjlab/rl/runner.py` | 保存 `common_step_counter` 和 env state |
| Metadata 生成 | `src/mjlab/rl/exporter_utils.py` | joint_names、action_scale、observation_names |
| Metadata 写入 | `src/mjlab/rl/exporter_utils.py` | 写入 ONNX 的 metadata_props |
| Velocity runner | `src/mjlab/tasks/velocity/rl/runner.py` | 保存时自动导出 ONNX |
| Manipulation runner | `src/mjlab/tasks/manipulation/rl/runner.py` | 保存时自动导出 ONNX |

### 源码阅读路线 ⭐⭐

理解 mjlab 的导出链路需要按数据流方向阅读以下文件。

**第一站：VecEnv Wrapper**。读 `src/mjlab/rl/vecenv_wrapper.py` 第 72-89 行。理解训练接口和部署接口的关键差异：训练时 wrapper 会合并 done 信号并传递 `time_outs`（告诉 PPO 哪些 episode 是因为超时而非真正终止的），但部署时不需要 `time_outs`。这个差异意味着 `time_outs` 影响 PPO 的 value bootstrapping 但不影响部署——如果你在部署端看到 `time_outs` 字段不知道怎么处理，答案是忽略它。

**第二站：Runner 的 checkpoint 保存**。读 `src/mjlab/rl/runner.py` 第 67-81 行。注意 `common_step_counter` 被保存到 checkpoint 中——这不是策略权重，而是训练进度的计数器。如果你从 checkpoint 恢复训练但忘记恢复这个计数器，curriculum（难度递增）会从零开始，导致策略"忘记"已经学会的困难行为。

**第三站：ONNX 导出**。读同文件第 34-58 行。`export_policy_to_onnx` 函数使用 `torch.onnx.export` 导出 actor 网络。关键细节：导出使用 trace 模式（不是 script 模式），这意味着任何依赖于输入值的 control flow（如 if-else 分支）都会被"烤死"为 trace 时的路径。如果你的策略有条件分支（如"如果接触了就切换模式"），ONNX 导出可能不正确。

**第四站：Metadata 生成**。读 `src/mjlab/rl/exporter_utils.py` 第 22-62 行。`get_base_metadata` 函数从环境配置中提取 joint_names、default_joint_pos、command_names、observation_names 和 action_scale。这些信息被写入 ONNX 的 metadata_props——部署端可以通过 `onnx.load(path).metadata_props` 读取。

**第五站：任务特定的 runner**。读 `src/mjlab/tasks/velocity/rl/runner.py` 和 `src/mjlab/tasks/manipulation/rl/runner.py`。注意这些 runner 在保存 checkpoint 时自动调用 ONNX 导出——不需要手动触发。理解 velocity runner 和 manipulation runner 在 metadata 内容上的差异（velocity 包含 command_names 如 vx/vy/yaw，manipulation 包含 ee_target 相关信息）。

**第六站：DR events**。读 `src/mjlab/tasks/velocity/velocity_env_cfg.py` 中的 domain randomization 事件。按 gap 分类理解每个事件：`friction` 对应接触 gap、`encoder_bias` 对应传感器 gap、`base_com` 对应动力学 gap、`push_robot` 对应外部扰动鲁棒性。这些事件的范围和分布就是策略训练时"见过的世界"——部署时真实参数必须落在这个范围内。

### 最小 `uv run` 导出实验 ⭐

```bash
# 步骤 1：短训练触发 ONNX 导出
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
  --env.scene.num-envs 64 \
  --agent.max-iterations 2 \
  --agent.save-interval 1 \
  --agent.logger tensorboard \
  --agent.upload-model False
# 通过标准：日志目录中出现 model_1.pt 和对应的 .onnx 文件

# 步骤 2：读取 ONNX metadata
uv run python -c "
from pathlib import Path
import onnx
paths = sorted(Path('logs/rsl_rl').glob('**/*.onnx'))
if not paths:
    print('No ONNX found - check training logs for export warnings')
else:
    model = onnx.load(paths[-1])
    meta = {p.key: p.value for p in model.metadata_props}
    print(f'ONNX path: {paths[-1]}')
    for k, v in meta.items():
        print(f'  {k}: {v[:80]}...' if len(v) > 80 else f'  {k}: {v}')
"
# 通过标准：能看到 joint_names、observation_names、action_scale
```

如果 ONNX 没有生成，训练日志中可能有 `[WARN] ONNX export failed` 的提示——最常见的原因是自定义的网络层不支持 ONNX trace（比如某些自定义的 activation function 或 dynamic shape 操作）。

### ProtoMotions "Obs Baked-In" 导出模式 ⭐⭐⭐

传统的 ONNX 导出只包含策略网络——部署端需要自己重新实现 obs 计算函数（把传感器原始数据变成策略的输入向量）。这是一个高频错误源：obs 函数在 Python 训练代码和 C++ 部署代码中各实现一次，任何细微差异（如归一化系数、history buffer 长度、frame 转换）都会导致策略行为异常。

ProtoMotions（NVlabs）提出了一种更干净的解决方案——**把 obs 计算图也"烤"进 ONNX**。导出后的 ONNX 接收的输入不是"策略 obs 向量"而是"原始传感器信号"（关节角、IMU 读数等），obs 的拼接、归一化、history 堆叠等操作都在 ONNX 图内部完成。

ProtoMotions 用三个"计算层级"来定义哪些 obs 操作可以被 baked-in：

| 层级 | 定义 | ONNX 可导出 | 示例 |
|------|------|------------|------|
| Level 1 | 纯张量计算，无副作用 | ✅ | `quat_rotate_inverse(base_quat, gravity)` |
| Level 2 | 聚合多个 Level 1 的结果 | ✅ | `obs = concat([vel_b, ang_vel, proj_grav, ...])` |
| Level 3 | 有副作用（文件 I/O、随机性） | ❌ | `log_to_wandb(obs)` |

Level 1 和 Level 2 的计算被注册为 `MdpComponent`，导出时自动包含在 ONNX 图中。这意味着部署端的 C++ 代码只需要提供原始传感器数据——不需要重写任何 obs 函数：

```python
# ProtoMotions 的 baked-in obs 导出流程（简化）
class MdpComponent:
    """可导出的 obs 计算组件。"""
    def __init__(self, compute_func, dynamic_vars, static_params):
        self.compute_func = compute_func  # 纯张量函数
        self.dynamic_vars = dynamic_vars  # 运行时输入路径
        self.static_params = static_params  # 固化参数

    def is_exportable(self):
        """Level 1 和 2 可导出。"""
        return not self.has_side_effects

def export_baked_onnx(env, policy, path="policy_baked.onnx"):
    """导出包含 obs 计算图的 ONNX。"""
    # 收集所有可导出的 MdpComponent
    exportable_components = [
        c for c in env.observation_manager.components
        if c.is_exportable()
    ]

    # 构建完整计算图: raw_sensors → obs → action
    class BakedPolicy(nn.Module):
        def __init__(self, components, actor):
            super().__init__()
            self.components = nn.ModuleList(components)
            self.actor = actor

        def forward(self, raw_sensors):
            # obs 计算在 ONNX 图内完成
            obs_parts = [c.compute_func(raw_sensors) for c in self.components]
            obs = torch.cat(obs_parts, dim=-1)
            return self.actor(obs)

    baked = BakedPolicy(exportable_components, policy.actor)
    dummy_sensors = torch.randn(1, raw_sensor_dim)
    torch.onnx.export(baked, dummy_sensors, path, ...)
```

**Baked-in 导出的工程价值**：unitree_rl_lab 的 C++ 部署代码（`g1_ctrl`）使用这种模式——它只需要从 SDK 读取关节角和 IMU，然后直接喂给 ONNX Runtime，不需要 C++ 层面的 obs 计算。这消除了"Python 训练 obs 和 C++ 部署 obs 不一致"这个最高频的部署 bug。

**Baked-in 的局限**：如果 obs 中包含 Level 3 的计算（如需要从外部文件读取标定数据），这部分不能被 baked-in。此外，RNN 的 hidden state 管理（在 episode 边界重置）也不在 ONNX 图内——C++ 端需要自己管理 hidden state buffer。

> **双重解读**：ProtoMotions 的 baked-in obs 模式可以从两个角度理解。**从软件工程的角度**，它是一种"接口契约的简化"——把部署端需要遵守的契约从"重新实现 42 维 obs 的每一维计算"简化为"提供原始传感器读数"，大幅降低了出错概率。**从机器学习的角度**，它是一种"端到端导出"——ONNX 不仅包含"从 obs 到 action 的映射"，还包含"从传感器到 obs 的映射"，使得整个推理管线的数学行为在 Python 和 C++ 之间完全一致。

### 完整的 ONNX 导出与验证代码 ⭐⭐

以下代码展示了从训练到导出到验证的完整流程——确保 ONNX 输出和 PyTorch 输出完全一致：

```python
import torch
import torch.onnx
import onnx
import onnxruntime as ort
import numpy as np

def export_and_verify_onnx(
    policy,
    obs_dim: int,
    path: str = "policy.onnx",
    metadata: dict = None,
    normalizer=None,
):
    """导出 ONNX 并验证与 PyTorch 输出一致。"""

    # Step 1: 准备导出
    policy.eval()

    # 如果有 normalizer，创建包含 normalizer 的 wrapper
    class PolicyWithNorm(torch.nn.Module):
        def __init__(self, actor, norm):
            super().__init__()
            self.actor = actor
            self.norm = norm

        def forward(self, obs):
            if self.norm is not None:
                obs = (obs - self.norm.running_mean) / (
                    self.norm.running_var.sqrt() + 1e-8
                )
            return self.actor(obs)

    wrapped = PolicyWithNorm(policy.actor, normalizer)
    dummy_input = torch.randn(1, obs_dim)

    # Step 2: 导出
    torch.onnx.export(
        wrapped,
        dummy_input,
        path,
        input_names=["obs"],
        output_names=["action"],
        opset_version=11,
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
    )

    # Step 3: 写入 metadata
    if metadata:
        model = onnx.load(path)
        for key, value in metadata.items():
            meta = model.metadata_props.add()
            meta.key = key
            meta.value = str(value)
        onnx.save(model, path)

    # Step 4: 验证输出一致性
    session = ort.InferenceSession(path)
    test_inputs = [torch.randn(1, obs_dim) for _ in range(10)]

    max_diff = 0.0
    for test_obs in test_inputs:
        # PyTorch 输出
        with torch.no_grad():
            pt_action = wrapped(test_obs).numpy()
        # ONNX 输出
        ort_action = session.run(None, {"obs": test_obs.numpy()})[0]
        diff = np.abs(pt_action - ort_action).max()
        max_diff = max(max_diff, diff)

    print(f"Max output diff: {max_diff:.8f}")
    if max_diff > 1e-5:
        print(f"⚠️ ONNX 输出与 PyTorch 差异过大: {max_diff:.8f}")
        print("可能原因: 自定义 activation 在 ONNX 中行为不同")
    else:
        print("✅ ONNX 验证通过")

    # Step 5: 打印推理延迟
    import time
    latencies = []
    for _ in range(100):
        t0 = time.perf_counter()
        session.run(None, {"obs": dummy_input.numpy()})
        latencies.append((time.perf_counter() - t0) * 1000)
    print(f"ONNX 推理延迟: {np.mean(latencies):.2f} ± {np.std(latencies):.2f} ms")
    print(f"  p50: {np.percentile(latencies, 50):.2f} ms")
    print(f"  p99: {np.percentile(latencies, 99):.2f} ms")

    return path

# 使用示例
metadata = {
    "joint_names": '["FL_hip", "FL_thigh", "FL_calf", ...]',
    "action_scale": "0.25",
    "obs_dim": str(obs_dim),
    "control_dt": "0.02",  # 50 Hz
    "default_joint_pos": '[0.0, 0.8, -1.6, ...]',
}
export_and_verify_onnx(policy, obs_dim=48, metadata=metadata)
```

### Isaac Lab 的 ONNX 导出对比 ⭐

Isaac Lab 使用类似的 `torch.onnx.export` 流程，但通过 RSL-RL 的 runner 自动触发。关键差异在于 metadata 格式和 normalizer 处理方式：

| 维度 | mjlab | Isaac Lab (unitree_rl_lab) |
|------|-------|---------------------------|
| 触发时机 | 每次 checkpoint 保存时自动导出 | `play.py` 运行时导出 |
| Normalizer | 单独文件或 baked-in | 通常 baked-in |
| Metadata 格式 | ONNX metadata_props | `deploy.yaml` + ONNX |
| 关节顺序 | 与 MJCF 一致 | 与 USD 一致（可能和 SDK 不同） |
| 输出文件 | `policy.onnx` | `policy.onnx` + `policy.onnx.data`（大模型） |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：手写 joint order 而不读 metadata**

部署端最常见的错误是不读 ONNX 的 `joint_names` metadata，而是凭记忆或文档手写关节顺序映射。当 URDF/MJCF 的关节顺序发生变化时（比如添加了一个 gripper 关节），手写的映射不会自动更新——导致所有关节动作错位一位。正确做法：部署端启动时自动解析 metadata，与 SDK 的关节名称做自动化比对，任何不匹配都报错而不是静默继续。

💡 **概念误区：认为"observation normalizer 不重要"**

如果训练时开启了 observation normalization（running mean/std），那么策略看到的 observation 是经过归一化的——均值接近 0，标准差接近 1。部署时如果不应用同样的归一化（使用训练时保存的 mean/std），策略看到的输入尺度可能差几个数量级——输出的动作会接近饱和或接近零。Normalizer 的 state 必须和 ONNX 一起导出、一起部署。

### 练习

1. **[实验题]** 用短训练导出一个 ONNX 文件。读取其 metadata，列出所有 key-value 对。对比 metadata 中的 joint_names 和 MJCF 文件中的关节顺序是否一致。
2. **[设计题]** 为一个视觉策略的 ONNX 导出包设计 metadata schema——除了 joint_names 和 action_scale，还需要包含哪些视觉相关信息（提示：camera intrinsics/extrinsics、preprocessing pipeline、input tensor layout）？
3. **[概念题]** 为什么 ONNX 不包含安全限幅？如果把限幅逻辑"烤"进 ONNX 计算图中有什么问题？

---

## 23.4 部署调参参考 ⭐⭐

> **这一节解决什么问题**：当真机出现特定现象时，应该优先怀疑什么、检查什么、如何修复？

### 调参表 ⭐⭐

部署阶段的问题与训练阶段不同——训练问题通常是 reward 不涨或策略不稳定，部署问题通常是"仿真可以但真机不行"或"第一步就出事"。以下表格按部署现象分类。

| 部署现象 | 优先怀疑 | 第一检查项 | 调整方向 |
|---------|---------|-----------|---------|
| 动作方向反 | joint order 错 | metadata `joint_names` | 写 adapter 并用单关节测试验证 |
| 动作幅度过大 | action scale 错 | metadata `action_scale` | 部署端复用训练 scale |
| 低频摇摆 | 控制延迟 | timestamp 与 loop period | 训练注入 delay 或加状态预测 |
| 高频抖动 | PD gain 或动作噪声 | action diff 和电机温度 | 加 smoothing、限幅、调 gain |
| 站立偏一边 | COM 或默认姿态 gap | default joint pos 和 base COM | SysID 或 DR COM range |
| 转弯摔倒 | 摩擦和侧向速度 gap | foot slip 日志 | 扩展 friction 和 command curriculum |
| 视觉抓取固定偏移 | 外参误差 | 标定板或已知点投影 | 重标定或随机外参训练 |
| depth 近处饱和 | 深度尺度或 cutoff | 真实 depth histogram | 重设裁剪和归一化 |
| 策略偶发失控 | 安全链路缺 watchdog | 控制线程超时日志 | 低层停机和 last-action decay |
| ONNX 输出 NaN | normalizer 或输入异常 | preprocess 后观测范围 | 输入 clamp 和异常检测 |
| 长时间性能衰退 | 温度和电池变化 | 电机温度、电压趋势 | 降额控制和 adaptive 参数 |
| 移动操作撞桌 | base-arm 同步 gap | 末端轨迹和底盘速度 | 联合约束和速度门控 |
| 物体抓不稳 | 夹爪或接触 gap | gripper force 和物体质量 | SysID gripper、随机 payload |
| 回放可行在线失败 | 实时频率不稳定 | loop jitter (p50/p95/p99) | 固定 realtime thread 并监控 |
| checkpoint 恢复后差异 | 环境状态没恢复 | `common_step_counter` | 确认 env_state 完整恢复 |

### Debug 清单 ⭐

每次真机测试前逐项勾选——任何一项未确认都不启动电机。

- [ ] checkpoint 和 ONNX 来自同一 run directory
- [ ] 读取了 ONNX metadata 而不是手写 joint order
- [ ] observation_names 与部署端拼接顺序一致
- [ ] action_scale、default_joint_pos、PD gains 与训练一致
- [ ] 控制频率和训练 decimation 对应
- [ ] 所有传感器输入都有 timestamp
- [ ] 输入单位是 rad、m、m/s，不是 degree、cm 或 SDK 自定义单位
- [ ] 坐标系是 base、world、camera 还是 end-effector——已确认
- [ ] observation normalizer 的 state 在导出和推理中一致
- [ ] 视觉输入经过同样的 resize、crop、depth clamp 和 channel 排列
- [ ] 策略输出后还有限幅、速度限制和急停
- [ ] 通信丢包或超时会触发安全动作
- [ ] 第一次真机测试从悬空、低力矩、低速度开始
- [ ] 每次上线保存完整日志：obs、action、command、state、timestamp
- [ ] 现场人员有物理急停和明确口令

---

## 23.5 Sim2Sim 交叉验证 ⭐⭐⭐

> **这一节解决什么问题**：在真机部署之前，如何用双框架交叉验证来提前发现 sim2real 问题？

### 动机：为什么需要 Sim2Sim

Sim2Sim 是 sim2real 的"前置测试"——在两个不同的物理引擎（MuJoCo 和 PhysX）中分别验证策略。如果策略在训练框架（如 Isaac Lab / PhysX）中表现正常，但在另一个框架（如 MuJoCo）中性能大幅下降，说明策略依赖了特定引擎的数值行为——这种依赖在真机上几乎一定会导致失效。

反过来，如果策略在两个引擎中表现一致（tracking error 差异 < 15%），说明策略学到的行为是**物理鲁棒的**——不依赖于特定引擎的实现细节，在真机上成功的概率更高。

Sim2Sim 验证已经成为足式 RL 社区的标准实践。humanoid-gym 首先提出了这种范式，unitree_rl_lab 将其工程化为一条完整的管线：

```
Isaac Lab 训练 → ONNX 导出 → unitree_mujoco 加载 → MuJoCo 评估
      ↑                                                    ↓
      └──── 如果 MuJoCo 中失败，回到训练修复 ────────────────┘
```

### 完整的 Sim2Sim 验证代码 ⭐⭐

```python
# === Sim2Sim 交叉验证流程 ===

def sim2sim_cross_validation(
    policy_path: str,
    mjlab_task: str = "Mjlab-Velocity-Flat-Unitree-Go2",
    num_envs: int = 64,
    num_episodes: int = 100,
    commands: list = None,  # 固定的速度指令列表
):
    """在 mjlab (MuJoCo) 中验证 Isaac Lab 训练的策略。"""
    import onnxruntime as ort

    # 1. 加载 ONNX 策略
    session = ort.InferenceSession(policy_path)
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    # 2. 读取 metadata 中的 obs normalizer
    model = onnx.load(policy_path)
    meta = {p.key: p.value for p in model.metadata_props}
    obs_mean = np.array(eval(meta.get("obs_mean", "None")))
    obs_var = np.array(eval(meta.get("obs_var", "None")))

    # 3. 创建 mjlab 环境
    env = make(mjlab_task, num_envs=num_envs)
    obs, _ = env.reset()

    # 4. 运行评估
    metrics = {"tracking_error": [], "survival_time": [], "drift": []}
    ep_step = torch.zeros(num_envs)

    for step in range(5000):
        # 归一化 obs（如果训练时使用了归一化）
        obs_np = obs['policy'].cpu().numpy()
        if obs_mean is not None:
            obs_np = (obs_np - obs_mean) / (np.sqrt(obs_var) + 1e-8)

        # ONNX 推理
        action = session.run([output_name], {input_name: obs_np.astype(np.float32)})[0]
        action_tensor = torch.from_numpy(action).to(env.device)

        # Step
        obs, reward, done, truncated, info = env.step(action_tensor)
        ep_step += 1

        # 记录完成的 episode
        finished = done | truncated
        if finished.any():
            for idx in finished.nonzero(as_tuple=True)[0]:
                metrics["survival_time"].append(ep_step[idx].item() * env.dt)
            ep_step[finished] = 0

    # 5. 汇总
    avg_survival = np.mean(metrics["survival_time"]) if metrics["survival_time"] else 0
    print(f"=== Sim2Sim Results ===")
    print(f"Completed episodes: {len(metrics['survival_time'])}")
    print(f"Avg survival time: {avg_survival:.1f} s")
    print(f"Target: > 18 s (of 20 s episode)")

    if avg_survival < 15.0:
        print("⚠️ Sim2Sim 验证失败：策略在 MuJoCo 中存活时间不足")
        print("可能原因：")
        print("  1. 接触摩擦模型差异 → 调整 MJCF 摩擦参数")
        print("  2. Actuator 模型差异 → 对齐 PD gains")
        print("  3. Obs normalizer 未正确加载 → 检查 metadata")
    else:
        print("✅ Sim2Sim 验证通过")

    env.close()
    return metrics
```

### unitree_rl_lab 的完整部署管线 ⭐⭐⭐

unitree_rl_lab（Unitree 官方）提供了从训练到真机部署的工业级管线。这是当前 Unitree Go2/G1/H1 部署的标准实践。其五个阶段如下：

**Stage 1: Train（Isaac Lab + RSL-RL）**

```bash
# Go2 速度跟踪
./unitree_rl_lab.sh -p scripts/rsl_rl/train.py \
    --task Unitree-Go2-Velocity-Flat-v0 \
    --num_envs 4096 --headless

# G1 29-DOF 速度跟踪
./unitree_rl_lab.sh -p scripts/rsl_rl/train.py \
    --task Unitree-G1-29dof-Velocity-v0 \
    --num_envs 4096 --headless
```

**Stage 2: Play & Export**

```bash
# 可视化 + 自动导出 ONNX
./unitree_rl_lab.sh -p scripts/rsl_rl/play.py \
    --task Unitree-G1-29dof-Velocity-v0 \
    --load_run latest
# 输出:
#   logs/.../exported/policy.onnx        (策略计算图)
#   logs/.../exported/policy.onnx.data   (大模型的权重数据)
#   logs/.../exported/deploy.yaml        (关节映射、scale 等)
```

`deploy.yaml` 的关键内容：

```yaml
# deploy.yaml 示例（简化）
robot: g1_29dof
control_dt: 0.02  # 50 Hz
motor_kp: [60.0, 60.0, 60.0, ...]  # 每个关节的 PD kp
motor_kd: [1.5, 1.5, 1.5, ...]     # 每个关节的 PD kd
# 关节映射: URDF 顺序 → SDK 顺序
# G1 的 SDK 关节顺序和 URDF 不同！
joint_mapping:
  urdf_to_sdk: [0, 1, 2, 3, 4, 5, 9, 10, 11, 6, 7, 8, ...]
action_scale: 0.25
default_joint_pos: [0.0, 0.0, -0.4, 0.8, -0.4, ...]
obs_normalization:
  mean: [...]
  std: [...]
history_length: 1  # obs 不包含历史帧（如果有 LSTM 则需要设置）
```

**Stage 3: Sim2Sim（unitree_mujoco）**

```bash
# 在 MuJoCo 中验证
./unitree_mujoco \
    -i 0 \                    # 实例 ID
    -n eth0 \                 # 网络接口
    -r g1 \                   # 机器人类型
    -s scene_29dof.xml        # MuJoCo 场景
```

这个命令启动一个 MuJoCo 仿真，模拟 Unitree SDK 的 DDS 通信接口。同一个 C++ 控制器可以透明地连接到 MuJoCo 仿真或真实机器人——只需要改变网络接口地址。

**Stage 4: C++ Controller（build + run）**

```bash
# 构建 C++ 控制器
cd deploy/robots/g1_29dof
mkdir -p build && cd build
cmake .. && make -j4

# 依赖：unitree_sdk2, libyaml-cpp, libboost, libeigen3, CycloneDDS

# 运行（连接到 Sim2Sim）
./g1_ctrl --network 127.0.0.1

# 运行（连接到真机）
./g1_ctrl --network eth0
```

C++ 控制器的核心循环：

```cpp
// g1_ctrl 核心循环（简化）
class G1Controller {
    OrtSession* policy_;
    float obs_buffer_[OBS_DIM];
    float action_buffer_[ACT_DIM];
    float hidden_state_[HIDDEN_DIM];  // RNN hidden state

    void ControlLoop() {
        while (running_) {
            auto t_start = Now();

            // 1. 读取传感器 (from SDK DDS)
            auto state = sdk_.GetLowState();

            // 2. 构建 obs
            BuildObservation(state, obs_buffer_);

            // 3. 归一化 obs
            NormalizeObs(obs_buffer_, obs_mean_, obs_std_);

            // 4. ONNX 推理
            policy_->Run(obs_buffer_, action_buffer_);

            // 5. Action 后处理
            for (int i = 0; i < N_JOINTS; i++) {
                float target = default_pos_[i]
                    + action_buffer_[joint_map_[i]] * action_scale_;
                // 安全限幅
                target = Clamp(target, joint_min_[i], joint_max_[i]);
                // Rate limit
                target = RateLimit(target, last_target_[i], max_rate_);
                cmd_.joint_cmd[i].q = target;
                cmd_.joint_cmd[i].kp = motor_kp_[i];
                cmd_.joint_cmd[i].kd = motor_kd_[i];
            }

            // 6. 发送命令
            sdk_.SendLowCmd(cmd_);

            // 7. 等待到下一个控制周期
            auto dt = Now() - t_start;
            if (dt < control_dt_) {
                Sleep(control_dt_ - dt);
            } else {
                LOG_WARN("Control loop overrun: {} ms", dt * 1000);
            }
        }
    }
};
```

**Stage 5: Sim2Real**

```bash
# 真机部署前的安全流程：
# 1. 按 L2+R2 进入 debug/damping 模式
# 2. 启动控制器
./g1_ctrl --network eth0
# 3. 等待 1 分钟站立（低增益 PD 模式）
# 4. 逐步增加增益和速度指令范围
```

**unitree_rl_lab 中一个高频 bug 的修复**：unitree_rl_lab 的 GitHub Issues #82 报告了一个经典的关节映射错误——`play.py` 导出的 ONNX 中关节顺序是 URDF 顺序，但 C++ runtime 使用 SDK 顺序。修复方法是在 `deploy.yaml` 中显式写入 `joint_mapping` 排列表。

### 双框架 Sim2Sim 验证流程图 ⭐⭐

完整的 Sim2Sim 验证涉及三个环境（训练环境、MuJoCo 验证环境、真机环境）之间的策略传递：

```
Isaac Lab (PhysX) 训练
    │
    ├── export policy.onnx + deploy.yaml
    │
    ├── ① unitree_mujoco (MuJoCo)  ← 同一个 C++ 控制器
    │      └── 验证: tracking error < 15% 差异
    │
    ├── ② mjlab (MuJoCo Warp)       ← Python ONNX Runtime
    │      └── 验证: survival time > 90% of training
    │
    └── ③ 真机 (unitree_sdk2)       ← 同一个 C++ 控制器
           └── 分级验收 D0→D6

反向路径: 如果在 ① 或 ② 中失败：
    → 分析失败原因（接触?延迟?映射?）
    → 在训练环境中修复（调 DR/obs/reward）
    → 重新训练 → 重新验证
```

### mjlab 训练 → 真机的部署路径 ⭐⭐

如果训练在 mjlab（MuJoCo Warp）中完成，部署路径与 Isaac Lab 略有不同：

```
mjlab (MuJoCo Warp) 训练
    │
    ├── export policy.onnx + metadata
    │
    ├── ① mjlab play (MuJoCo Warp)  ← 训练同引擎验证
    │      └── 验证: train/play 一致性
    │
    ├── ② unitree_mujoco (MuJoCo C)  ← C++ 控制器
    │      └── 验证: MuJoCo Warp → MuJoCo C 一致性
    │      (MuJoCo Warp 和 MuJoCo C 引擎略有差异)
    │
    └── ③ 真机 (unitree_sdk2)
           └── 分级验收 D0→D6
```

mjlab 的优势是训练引擎（MuJoCo Warp）和 Sim2Sim 验证引擎（MuJoCo C）共享同一个物理模型——理论上 ① → ② 的差异应该比 Isaac Lab → MuJoCo 的差异更小。但注意 MuJoCo Warp（GPU 并行版本）和 MuJoCo C（CPU 单线程版本）在浮点精度和求解器实现上仍有细微差异——不能假设两者完全一致。

**unitree_rl_mjlab 的部署流程**与 unitree_rl_lab 完全对称：

```bash
# mjlab 训练
python scripts/train.py Unitree-G1-Flat \
    --env.scene.num-envs=4096 --headless

# Play + Export
python scripts/play.py Unitree-G1-Flat \
    --checkpoint_file=logs/.../model_xxx.pt

# Sim2Sim（同一个 C++ 控制器）
cd deploy/robots/g1 && mkdir -p build && cd build
cmake .. && make -j4
./g1_ctrl --network 127.0.0.1  # 连接到 unitree_mujoco

# 真机
./g1_ctrl --network eth0  # 连接到真实 G1
```

**两个管线的关键差异**：

| 维度 | Isaac Lab → 真机 | mjlab → 真机 |
|------|-----------------|-------------|
| 训练引擎 | PhysX (GPU) | MuJoCo Warp (GPU) |
| Sim2Sim 引擎 | MuJoCo C | MuJoCo C |
| 跨引擎差异 | PhysX → MuJoCo（较大） | MuJoCo Warp → MuJoCo C（较小） |
| C++ 控制器 | 同一个 | 同一个 |
| ONNX 格式 | 相同 | 相同 |
| deploy.yaml 格式 | 相同 | 相同 |
| 适用场景 | 需要 RTX 渲染/USD 资产 | 偏好 MuJoCo 生态/轻量级 |

### Sim2Sim 失败时的诊断决策树 ⭐⭐

当 Sim2Sim 验证失败时（策略在验证引擎中表现明显差于训练引擎），按以下决策树定位原因：

```
Sim2Sim survival < 90% of training
│
├── Zero agent 在两个引擎中的 base height 差异 > 3cm？
│   ├── 是 → 接触参数差异（摩擦、弹性、阻尼）
│   │   修复：对齐 MJCF 和 USD 的接触参数
│   └── 否 ↓
│
├── 同一 obs 输入在两个引擎中的 action 输出相同？
│   ├── 否 → ONNX 加载或 obs 预处理差异
│   │   修复：检查 normalizer、joint order、obs schema
│   └── 是 ↓
│
├── Action 相同但关节追踪误差不同？
│   ├── 是 → Actuator 模型差异（PD gains、力矩限制）
│   │   修复：对齐 PD 参数，检查 effort_limit
│   └── 否 ↓
│
└── 以上都一致但策略仍然失败？
    → 可能是 sim step 精度差异（dt 太大、求解器迭代次数不同）
    修复：减小 dt、增加求解器迭代次数
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Sim2Sim 时忘记加载 obs normalizer**
- 后果：策略在 MuJoCo 验证中表现极差（因为 obs 未归一化），你误以为是物理差异导致的
- 正确做法：验证脚本中首先检查 metadata 中是否包含 `obs_mean` 和 `obs_var`

⚠️ **思维陷阱：Sim2Sim 通过就认为 sim2real 一定成功**
- Sim2Sim 验证的是策略对物理引擎差异的鲁棒性，但它不能验证传感器噪声、通信延迟、电机温升等真机特有的问题
- 正确理解：Sim2Sim 是必要条件（通过了才可以尝试真机），不是充分条件（通过了不保证真机成功）

### 练习

1. **[实践题]** 在 mjlab 中训练一个 Go2 速度跟踪策略，导出 ONNX，然后在 Isaac Lab 中加载同一个 ONNX 进行评估。记录两个框架的 tracking error，分析差异来源。
2. **[分析题]** unitree_rl_lab 的 `deploy.yaml` 中 `joint_mapping.urdf_to_sdk` 为什么需要一个排列表？如果 G1 的 URDF 关节顺序和 SDK 关节顺序完全一致，这个排列表应该是什么？
3. **[设计题]** 为 sim2sim 验证设计一个"自动化 CI 流程"：每次训练完成后自动在 MuJoCo 中运行 100 个 episode 的评估，如果 survival time < 15s 自动标记为失败并邮件通知。写出 CI 脚本的伪代码。

---

## 23.6 延迟补偿：控制闭环中最隐蔽的 Gap ⭐⭐⭐

> **这一节解决什么问题**：延迟如何分解为四段？每段延迟对控制行为有什么具体影响？如何在训练阶段预补偿延迟？

### 延迟不是一个数字 ⭐⭐⭐

延迟（latency）是 sim2real 中最容易被低估的 gap。仿真中通常假设"观测立即可用、动作立即执行"，但真实系统中从传感器读数到电机响应的每一步都有时间消耗。更危险的是，延迟不是一个固定值——它会随负载、温度、通信状态和计算复杂度波动（jitter），这种波动比平均延迟更难处理。

延迟可以分解为四段，每段有不同的物理来源和影响模式：

| 延迟段 | 物理来源 | 典型范围 | 影响模式 |
|--------|---------|---------|---------|
| **Observation latency** | 传感器采样、A/D 转换、数据传输 | 1-20 ms | 策略看到的是"过去"的状态 |
| **Policy inference latency** | 神经网络前向传播 | 0.5-5 ms | 延长 obs 到 action 的间隔 |
| **Action transport latency** | 通信协议、串行传输、队列等待 | 1-10 ms | 命令到达执行器的延迟 |
| **Actuator response latency** | 电机电气时间常数、机械惯性 | 5-50 ms | 命令发出到力矩生效的延迟 |

对于一个 50 Hz 的控制循环（20 ms 周期），如果端到端延迟是 30 ms，策略的动作到达电机时，机器人已经运动了 1.5 帧——对于快速动态任务（步态、抓取），这个滞后可能导致相位错误或定位偏差。

考虑一个具体场景：四足机器人以 1 m/s 行走，足端触地时需要切换摆动/支撑相。如果延迟导致切换晚了 20 ms，足端在空中多飘了 2 cm 才着地——着地位置偏差可能导致滑动或绊倒。如果延迟导致切换早了 20 ms，足端在还没到达目标位置时就被锁定——步幅缩短，影响稳定性。

真实控制周期的时间预算可以用一个简单的公式表达：

```text
loop_budget = control_period
used = sensor + preprocess + policy + safety + transport
margin = loop_budget - used
```

如果 margin 经常小于 0，降低网络或传感器成本比调 PPO 更重要。如果 margin 偶发小于 0（jitter），偶发的延迟尖峰可能比平均延迟更危险——因为策略是为"大约 $k$ 帧延迟"训练的，偶发的 $3k$ 帧延迟完全超出训练分布。

### 延迟补偿的四种策略 ⭐⭐

**策略一：训练时注入延迟。** 在仿真环境中人为添加 delay buffer，让策略在训练时就学会在有延迟的条件下控制。这是最简单也最常用的方法。具体实现是在 observation buffer 中添加 $k$ 帧延迟——策略看到的是 $k$ 步之前的状态。缺点是延迟越大，训练越困难（因为策略需要学会"预测未来"），且策略只对训练时注入的延迟范围鲁棒。

**策略二：状态预测。** 用一个小型神经网络或物理模型预测当前状态，补偿 observation latency。策略接收的不是过去的状态，而是"预测的当前状态"。优点是不增加训练难度；缺点是预测本身有误差，尤其在接触切换等非线性事件附近。

**策略三：降低控制频率。** 如果端到端延迟是 15 ms，把控制频率从 100 Hz 降到 50 Hz（20 ms 周期），延迟占比从 150% 降到 75%——相对影响减小。缺点是低频控制对扰动的响应变慢。

**策略四：Action hold/repeat。** 当新的动作还没计算出来时，重复上一个动作。这避免了"空白期"但可能导致动作不连续。

如果完全不做延迟补偿会怎样？在低速任务（如慢速行走、静态抓取）中可能没有明显问题——因为一帧延迟造成的位移误差在机器人的容错范围内。但在高速任务（快速跑步、动态抓取）中，延迟是性能的主要瓶颈。一个经验法则是：如果 $\text{delay} / \text{task\_period} > 0.1$（延迟超过任务特征时间的 10%），延迟补偿就是必需的。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：只测平均延迟不测 jitter**

平均延迟 10 ms、偶发尖峰 50 ms 比稳定的 20 ms 延迟更危险——因为策略是为"大约 10 ms 延迟"训练的，偶发的 50 ms 延迟完全超出了训练分布。正确做法：测量延迟的 p50、p95 和 p99，策略的鲁棒性应该覆盖 p95 而非 p50。

💡 **概念误区：认为"action 延迟比 observation 延迟更危险"**

两者的影响模式不同，不能简单比较。Observation 延迟让策略做出"基于过时信息的决策"，action 延迟让"正确的决策晚执行"。对于慢变系统（行走），observation 延迟影响更大（因为决策质量下降）；对于快变系统（抓取的闭合瞬间），action 延迟影响更大（因为时机错过就失败）。

🧠 **思维陷阱：用仿真中的"完美零延迟"评估策略性能**

如果训练和评估都在零延迟条件下进行，那么评估出的性能是策略在理想条件下的上限——与真实部署性能完全无关。正确做法：评估时注入与真实系统相同的延迟（通过测量获得），这样评估结果才能预测真实部署的表现。

### 练习

1. **[计算题]** 一个控制系统的四段延迟分别是：observation 5 ms、policy 2 ms、transport 3 ms、actuator 10 ms。控制频率 50 Hz。计算端到端延迟占控制周期的比例。这个比例是否需要延迟补偿？
2. **[实验题]** 在 mjlab 中训练一个四足行走策略（0 帧延迟），然后在 play 时注入 1、2、4 帧 observation 延迟，记录不同延迟下的行走成功率和步态稳定性。
3. **[设计题]** 你的真实机器人的 policy inference 延迟有很大 jitter（p50=2 ms, p95=15 ms, p99=40 ms）。设计一个延迟补偿方案，说明如何在训练和部署两端分别处理。

### 延迟测量的工程实现 ⭐⭐

在部署前必须精确测量真实系统每段延迟。以下代码展示了如何在 C++ 控制器中嵌入延迟测量：

```cpp
// 延迟测量工具（嵌入控制器主循环）
struct LatencyProfiler {
    std::vector<double> obs_latencies;     // 传感器读取延迟
    std::vector<double> policy_latencies;  // 策略推理延迟
    std::vector<double> total_latencies;   // 端到端延迟

    void RecordObsLatency(double ms) { obs_latencies.push_back(ms); }
    void RecordPolicyLatency(double ms) { policy_latencies.push_back(ms); }
    void RecordTotalLatency(double ms) { total_latencies.push_back(ms); }

    void PrintStats() {
        auto stats = [](const std::vector<double>& v) {
            std::vector<double> sorted(v);
            std::sort(sorted.begin(), sorted.end());
            double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            double p50 = sorted[sorted.size() * 50 / 100];
            double p95 = sorted[sorted.size() * 95 / 100];
            double p99 = sorted[sorted.size() * 99 / 100];
            printf("  mean=%.2f  p50=%.2f  p95=%.2f  p99=%.2f ms\n",
                   mean, p50, p95, p99);
        };
        printf("Obs latency: "); stats(obs_latencies);
        printf("Policy latency: "); stats(policy_latencies);
        printf("Total latency: "); stats(total_latencies);
    }
};

// 在控制循环中使用
void ControlLoop() {
    LatencyProfiler profiler;
    while (running_) {
        auto t0 = Now();

        auto t_obs_start = Now();
        auto state = sdk_.GetLowState();
        BuildObservation(state, obs_buffer_);
        profiler.RecordObsLatency(ElapsedMs(t_obs_start));

        auto t_policy_start = Now();
        policy_->Run(obs_buffer_, action_buffer_);
        profiler.RecordPolicyLatency(ElapsedMs(t_policy_start));

        // ... action post-processing and sending ...

        profiler.RecordTotalLatency(ElapsedMs(t0));

        if (step_ % 1000 == 0) profiler.PrintStats();
    }
}
```

### 延迟注入训练 ⭐⭐

在 mjlab 中，通过 `EventTermCfg` 的 step mode 注入 action delay：

```python
# mjlab action delay 配置
class EventsCfg:
    # 模拟真实系统的 action 延迟
    action_delay = EventTermCfg(
        func=mdp.randomize_action_delay,
        mode="step",
        params={
            # 每个 env 的延迟在 reset 时采样，episode 内固定
            "delay_range": (1, 3),  # 1-3 个 sim step = 5-15 ms (dt=0.005)
        },
    )
    # 模拟观测延迟（滞后 1-2 帧 obs）
    obs_delay = EventTermCfg(
        func=mdp.randomize_obs_delay,
        mode="step",
        params={"delay_range": (0, 2)},
    )
```

**延迟注入的量纲映射**：训练中的延迟参数（以 sim step 为单位）必须和真机延迟对齐。如果 sim dt = 0.005 s，delay_range = (1, 3) 对应 5-15 ms 的延迟。如果真机测量的 p95 延迟是 20 ms，delay_range 应该设为 (1, 4)（覆盖 5-20 ms）。

> **跨领域类比**：延迟训练就像飞行员在飞行模拟器中练习在不同风速条件下着陆。真实着陆时的阵风不可预测（像延迟 jitter），但如果你在训练中经历过比真实更极端的阵风，你在温和条件下的表现会更稳定。同理，如果训练中的延迟范围覆盖了真实延迟的 p95，部署时策略在 p50 延迟下的表现会非常稳健。

---

## 23.7 安全链路：策略之外的保护层 ⭐⭐

> **这一节解决什么问题**：为什么安全不能依赖策略本身？安全链路应该包含哪些层？每一层保护什么？

### 安全 vs 控制：两个独立的系统 ⭐⭐

一个常见的误解是认为可以通过 reward shaping 让策略"学会安全"——给不安全行为加负 reward，策略自然会避免。这在仿真中确实有效：策略学会了避免大幅度动作和危险姿态。但在真实部署中，安全不能依赖策略的"意愿"——因为策略可能因为输入异常（传感器故障）、推理错误（数值溢出）或环境突变（被人推了一把）而产生不安全的输出。

> **本质洞察**：策略文件只是一层函数 $a = \pi(o)$。真实机器人需要完整的控制协议和安全协议。把策略看成完整系统是 sim2real 的常见错误。策略只负责从 observation 到 action 的映射。状态估计、单位转换、限幅、滤波、急停、回退和日志都在策略之外。

安全系统的设计原则是**独立性**和**可靠性优先于性能**。独立性意味着安全链路不能和控制链路共享同一个故障点。如果策略推理的 Python 进程崩溃，安全限幅不能因此失效。这意味着安全逻辑应该运行在独立的线程（或独立的硬件控制器）上，即使主控进程死掉也能独立执行急停。可靠性优先意味着安全链路的代码应该尽可能简单——越简单越不容易出错。一个 200 行的 watchdog 比一个 5000 行的"智能安全系统"更可靠。

### 安全链路的层次结构 ⭐⭐

| 层 | 监控内容 | 触发动作 | 实现位置 |
|----|---------|---------|---------|
| **L0 物理急停** | 人按下按钮 | 断电或制动 | 硬件按钮 |
| **L1 硬件 watchdog** | 通信超时 | 进入安全姿态 | 电机控制器内部 |
| **L2 姿态保护** | base pitch/roll 超限 | 锁定关节 | 独立安全线程 |
| **L3 动作限幅** | action 超出范围 | 裁剪到安全范围 | 部署端 post-process |
| **L4 温度保护** | 电机温度过高 | 降低动作幅度 | telemetry 监控 |
| **L5 状态估计保护** | 传感器数据过旧 | 冻结动作 | observation 时间戳检查 |

L0-L2 是**硬安全**——即使软件完全失效也能保护硬件和人员。L3-L5 是**软安全**——在软件正常运行的前提下保护策略不做危险动作。两层必须都有——只有软安全的系统在软件崩溃时毫无保护；只有硬安全的系统在正常运行时无法防止策略的"合法但危险"的输出（比如持续输出高力矩导致电机过热）。

### Unitree 部署边界详解 ⭐⭐

mjlab 可以训练 Unitree G1 和 Go1 相关任务，可以通过 runner 导出 ONNX 和 metadata。但真实 Unitree 部署还需要一个完整的 SDK adapter 层——这个 adapter 是仿真策略和真实硬件之间的翻译器。

**Adapter 必须处理的内容**：

| 功能 | 具体内容 | 如果缺失会怎样 |
|------|---------|---------------|
| 状态读取 | 从 SDK 获取关节角度、角速度、IMU 数据 | 策略没有输入，输出随机 |
| 坐标转换 | SDK 可能使用不同的坐标系约定 | 方向性动作全部反向 |
| Joint order 映射 | MJCF 和 SDK 的关节排列不同 | 关节动作错位 |
| 控制模式 | 确认 SDK 处于 position/velocity/torque 模式 | 策略输出的含义被误解 |
| PD gain 设置 | 训练时的 Kp/Kd 必须在真机上复现 | 关节追踪精度下降 |
| Action scale | 训练时的 action_scale 必须在真机上复现 | 动作幅度过大或过小 |
| 限幅 | 所有命令在发送前裁剪到安全范围 | 可能发送危险的关节命令 |
| 急停 | 独立于策略进程的停止机制 | 策略失控时无法制动 |

**上线前的四步验证**：

第一步，逐关节静态测试。给每个关节发送一个小幅正弦波命令（$\pm$0.05 rad, 0.5 Hz），目视确认：关节确实在运动？运动方向与命令方向一致？运动幅度与命令幅度成比例？没有其他关节联动？这一步可以在 5 分钟内发现 joint order 映射错误——这是最常见也最危险的部署 bug。

第二步，悬空低增益测试。把机器人悬挂起来（四脚离地），PD gain 设为训练值的 20%。启动策略，观察所有关节是否按预期的步态模式运动。低增益确保即使有问题，关节力矩也不足以损坏机器人。

第三步，低速 tethered 测试。用安全绳连接机器人，在平地上以最低速度（<0.3 m/s）行走。确认步态稳定、无抖动、制动有效。在这一步首次测试急停功能——主动触发急停按钮，确认机器人在 100 ms 内停止。

第四步，安全停止测试。主动触发每一个安全分支：watchdog 超时、姿态越限、通信中断、人工急停。确认每个分支的触发到停止延迟都在设计预算内。

**特别注意：Unitree SDK 的单位约定**。Unitree SDK 的关节角度单位是 rad，但某些接口的角速度单位可能是 rad/s 或 deg/s（取决于 SDK 版本）。如果训练时假设 rad/s 但真机发的是 deg/s，角速度相差 57 倍——策略看到的状态完全不对，但不会报任何错误。解决方法：在 adapter 中显式写明每个输入量的单位，并用已知的静态姿态做数值验证（关节角度应该与 MJCF 中的 default pose 一致）。

**Adapter 的测试方法**：Adapter 不应该只靠目视验证——应该写自动化的单元测试。最小测试集包括：

1. **静态一致性测试**：把机器人放在已知姿态（如 default pose），读取 SDK 关节角度，与 MJCF 的 default_joint_pos 对比。差异应该 < 0.01 rad。
2. **动态一致性测试**：让机器人执行一个已知的正弦波动作，记录 SDK 的关节角度反馈，与命令的正弦波对比。频率和幅度应该匹配（允许 PD 追踪误差）。
3. **单位验证测试**：发送 1.0 的 action，确认电机位移与 action_scale * 1.0 一致。如果 action_scale 是 0.25 rad，电机应该移动约 0.25 rad（$\pm$PD 追踪误差）。
4. **边界安全测试**：发送超出限幅范围的 action，确认 adapter 正确裁剪——电机命令不超过安全阈值。

这四个测试可以在 5 分钟内完成，但能避免大多数 adapter 相关的部署事故。每次修改 adapter 代码后都应该重新运行这些测试。

### 移动操作的额外安全要求 ⭐⭐

移动操作机器人（底盘 + 机械臂）比纯 locomotion 需要额外的安全保护：

- base 与 arm 的坐标树必须统一，尤其是 base yaw、arm base frame 和 camera frame
- 移动底盘会改变相机视角，视觉策略必须覆盖运动模糊和遮挡
- 机械臂动作会改变重心，locomotion 策略必须看到或承受 payload 变化
- 夹爪接触模型比足端接触更细，物体质量、摩擦和形状随机化要单独做
- 任务级状态机要决定何时移动、何时伸臂、何时冻结底盘
- 工作空间碰撞不能只靠 reward，部署端需要几何限位或规划保护
- base 和 arm 任一异常都必须能停全系统——不能出现 arm 继续动而 base 已急停

### ⚠️ 常见陷阱

⚠️ **编程陷阱：安全急停只放在 UI 层**

如果急停是通过 GUI 按钮发送一个"停止"命令到策略进程，那么当策略进程挂死（死循环、段错误）时，急停命令无法被接收——机器人继续执行最后一个动作。正确做法：急停由独立的硬件 watchdog 或独立进程执行，不经过策略进程。

💡 **概念误区：认为"action clip 等同于安全保护"**

Action clip 只限制动作数值的范围，不处理以下情况：通信丢包（没有 action 可 clip）、传感器故障（action 基于错误输入计算）、温度过高（在安全范围内的 action 长时间持续也会导致过热）、跌倒（action 不越界但姿态已经不可恢复）。安全保护需要覆盖所有这些场景。

### 安全系统的工程实现 ⭐⭐

以下代码展示了一个独立于策略的安全保护层——它应该在单独的线程或进程中运行，即使策略进程崩溃也能保护硬件：

```python
# === 独立安全监控线程 ===
import threading
import time

class SafetySupervisor:
    """独立于策略的安全保护层。
    
    在单独线程中运行，持续监控传感器状态，
    当检测到异常时直接发送安全命令（不经过策略）。
    """
    def __init__(self, sdk_interface, config):
        self.sdk = sdk_interface
        self.config = config
        self.is_safe = True
        self.safety_events = []
        self.lock = threading.Lock()

    def check_tilt(self, imu_data) -> bool:
        """姿态保护：roll/pitch 超限时锁关节。"""
        roll, pitch = imu_data.roll, imu_data.pitch
        if abs(roll) > self.config.max_roll or abs(pitch) > self.config.max_pitch:
            self.safety_events.append({
                "time": time.time(),
                "type": "tilt_exceeded",
                "roll": roll, "pitch": pitch,
            })
            return False
        return True

    def check_watchdog(self, last_cmd_time) -> bool:
        """通信超时保护：超过 200ms 未收到命令。"""
        if time.time() - last_cmd_time > self.config.watchdog_timeout:
            self.safety_events.append({
                "time": time.time(),
                "type": "watchdog_timeout",
            })
            return False
        return True

    def check_joint_limits(self, joint_pos) -> bool:
        """关节限位保护。"""
        for i, pos in enumerate(joint_pos):
            if pos < self.config.joint_min[i] or pos > self.config.joint_max[i]:
                self.safety_events.append({
                    "time": time.time(),
                    "type": "joint_limit",
                    "joint": i, "pos": pos,
                })
                return False
        return True

    def emergency_stop(self):
        """紧急停止：锁定所有关节到当前位置。"""
        with self.lock:
            self.is_safe = False
            current_pos = self.sdk.get_joint_positions()
            # 用高阻尼、低刚度锁定到当前位置
            self.sdk.send_damping_mode(
                positions=current_pos,
                kp=[5.0] * len(current_pos),  # 低刚度
                kd=[2.0] * len(current_pos),  # 高阻尼
            )
            print(f"🛑 EMERGENCY STOP triggered at {time.time()}")
            print(f"   Events: {self.safety_events[-1]}")

    def run(self):
        """安全监控主循环（在独立线程中运行）。"""
        while True:
            state = self.sdk.get_state()
            
            safe = True
            safe &= self.check_tilt(state.imu)
            safe &= self.check_watchdog(state.last_cmd_time)
            safe &= self.check_joint_limits(state.joint_pos)
            
            if not safe:
                self.emergency_stop()
                break  # 停止后不再恢复（需要人工重启）
            
            time.sleep(0.002)  # 500 Hz 安全检查频率

# 使用：在启动策略之前启动安全线程
safety = SafetySupervisor(sdk, safety_config)
safety_thread = threading.Thread(target=safety.run, daemon=True)
safety_thread.start()
```

**关键设计决策**：安全线程的检查频率（500 Hz）必须高于策略频率（50 Hz）——确保在策略输出下一个动作之前就能检测到异常。如果安全检查频率低于策略频率，可能存在一个"漏检窗口"——策略在安全检查之间输出了危险动作。

> **反事实推理：如果安全系统和策略在同一个线程中会怎样？** 策略的 ONNX 推理偶尔会卡顿（GC、内存分配），导致一帧的推理时间从 2ms 增大到 50ms。在这 50ms 内，安全检查也被阻塞——如果恰好在这段时间内发生了倾斜或碰撞，安全系统无法及时响应。独立线程确保了安全检查不受策略推理的影响。

🧠 **思维陷阱：只在"出了事"后才重视安全**

安全链路必须在第一次真机测试之前就验证完毕。正确的做法是：在无危险条件下主动触发每一个安全分支——让 watchdog 超时、让姿态越限、让通信中断——确认每个触发都能正确执行预期的保护动作。如果等到策略真的失控时才发现急停不工作，已经太晚了。

### 真实控制循环的性能监控 ⭐⭐

安全链路保护的是异常情况——但即使没有触发任何安全阈值，策略的性能仍然可能在缓慢退化。性能监控的目标是在退化到失败之前发出预警。

一次真实控制周期至少包括传感器读取、预处理、策略前向、后处理、安全过滤、SDK 发送和执行器响应。每一步都需要监控：

| 指标 | 记录方式 | 危险信号 | 处理 |
|------|---------|---------|------|
| policy forward time | 部署端 profiler | 接近 control period 的 80% | 减小网络或降低频率 |
| sensor age | timestamp 差值 | 图像或 IMU 数据超过 2 帧 | 同步线程和丢帧策略 |
| action norm | 每周期记录 | 持续饱和（接近 action_scale） | 检查 scale 和限幅 |
| action diff | 相邻动作差的范数 | 高频抖动（diff > 正常值 3 倍） | smoothing 或调 PD |
| motor temperature | 硬件 telemetry | 持续上升且不回落 | 降额和停止条件 |
| contact/slip | 足底力或状态估计 | 打滑或冲击力过大 | 摩擦 DR 和步态调参 |
| camera histogram | 图像统计（均值/方差） | 真实分布远超训练 | 视觉 DR 和预处理 |
| watchdog event | 安全日志 | 周期性超时 | 调度和通信链路 |

这些指标不需要实时分析——可以后处理。但数据必须在运行时记录完整，否则事后无法复盘。一个好的实践是在每次部署开始时用 10 秒的"基准数据"建立正常范围，然后在运行中持续对比——任何指标偏离基准 3 个标准差都应该触发警告。

### ⚠️ 常见陷阱（续）

⚠️ **编程陷阱：telemetry 线程抢占控制线程**

如果 telemetry 记录和策略推理在同一个 CPU 核上运行，磁盘写入可能导致策略推理的延迟尖峰。正确做法：telemetry 写入使用独立线程和缓冲队列，或者写到共享内存由另一个进程异步刷盘。

💡 **概念误区：认为"性能下降一定是策略问题"**

真机性能随时间下降可能不是策略退化——而是硬件状态变化。电机温度升高后 PD gain 改变、电池电压下降后最大力矩减小、地面灰尘积累后摩擦改变。在归因于策略之前，先检查硬件 telemetry 是否有系统性变化。

### 练习

1. **[设计题]** 为一个 Unitree G1 人形机器人设计安全链路。列出 L0-L5 每一层的具体实现（监控哪个传感器、阈值是多少、触发后执行什么动作）。
2. **[分析题]** 一个机器人的安全链路中，watchdog 和策略进程运行在同一个 Python 进程中。分析这个设计的风险，并提出改进方案。
3. **[思考题]** 移动操作机器人（底盘 + 机械臂）比纯 locomotion 机器人需要哪些额外的安全保护？为什么？

---

## 23.8 分级验收：从离线核对到完整任务 ⭐⭐

> **这一节解决什么问题**：真机测试为什么要分级？每个等级验证什么？何时可以升级到下一等级？

### 分级验收的动机 ⭐⭐

直接把训练好的策略放到真机上跑完整任务是危险的——如果策略有 bug（比如关节顺序错），机器人可能在第一帧就做出猛烈的不预期动作，损坏硬件或伤人。分级验收的核心思想是：**从最安全的测试开始，每次只增加一点风险，确认安全后再升级。**

> **跨领域类比：分级验收与飞行员训练。** 飞行员不会第一天就单飞。训练从地面理论开始（离线核对），然后是飞行模拟器（仿真回放），接着是副驾驶位伴飞（影子模式），再是有教官在场的简单飞行（低速测试），最后才是独立执行任务（完整部署）。每一步的风险都可控，前一步的验证为后一步提供信心。

### 七级验收体系 ⭐⭐

| 等级 | 执行环境 | 允许动作 | 主要风险 | 通过标准 |
|------|---------|---------|---------|---------|
| **D0** 静态核对 | 离线文件 | 无动作 | joint order、normalizer | schema 与训练配置一致 |
| **D1** 仿真回放 | mjlab play | 策略动作 | train/play 不一致 | 固定 seed replay 通过 |
| **D2** 影子模式 | 真机旁路 | 不下发电机 | 状态估计延迟 | policy 输出在安全范围内 |
| **D3** 悬空低增益 | 真机支撑 | 小幅动作 | PD gain 错位 | 关节方向和幅度正确 |
| **D4** 平地低速 | 真机平地 | 限速行走 | 延迟和摩擦 | 急停、watchdog 正常 |
| **D5** 任务片段 | 受控场景 | 限定范围 | 目标感知和接触 | 任务片段可重复 |
| **D6** 完整任务 | 真机任务区 | 完整策略 | 长时热稳定 | 达到验收率有失败归因 |

**D0-D1 是离线验证**——不接触真机硬件，零风险。D0 检查的是最基本的软件接口一致性：ONNX metadata 与训练配置是否一致、normalizer 是否存在且版本匹配、action scale 是否正确。D1 检查的是 train/play 一致性：同一个 checkpoint 在训练和回放中是否产生相同的行为。

**D2-D3 是低风险真机测试**。D2 的"影子模式"意味着策略运行并接收真实传感器数据，但输出的动作不下发给电机——只记录日志。这验证了状态估计和预处理链路的正确性，且完全安全。D3 把机器人悬挂或支撑起来，只给很低的 PD gain，让每个关节做小幅运动——这验证了关节方向映射的正确性，即使映射错了，低增益也不会产生危险的力。

**D4-D6 是逐步升级的真机任务**。每个等级都有明确的"通过标准"和"升级条件"——只有在当前等级稳定通过后才能升级。任何等级的失败都应该先回退到上一个等级诊断原因，而不是在当前等级反复尝试。

### D0 自动化验证脚本 ⭐⭐

D0 验证只需要 5 分钟但能发现 50% 以上的部署 bug。以下是一个完整的 D0 检查脚本：

```python
# === D0 静态核对脚本 ===
import onnx
import yaml
import json
import sys

def d0_verification(
    onnx_path: str,
    deploy_yaml_path: str = None,
    training_config_path: str = None,
):
    """D0 级别的静态核对。"""
    print("=" * 60)
    print("D0 Sim2Real 静态核对")
    print("=" * 60)
    passed = True

    # === Check 1: ONNX 文件完整性 ===
    print("\n[Check 1] ONNX 文件完整性...")
    try:
        model = onnx.load(onnx_path)
        onnx.checker.check_model(model)
        print(f"  ✅ ONNX 加载成功, opset={model.opset_import[0].version}")
    except Exception as e:
        print(f"  ❌ ONNX 加载失败: {e}")
        passed = False
        return passed

    # === Check 2: Metadata 完整性 ===
    print("\n[Check 2] Metadata 完整性...")
    meta = {p.key: p.value for p in model.metadata_props}
    required_keys = ["joint_names", "action_scale", "obs_dim"]
    for key in required_keys:
        if key in meta:
            print(f"  ✅ {key}: {meta[key][:60]}...")
        else:
            print(f"  ❌ 缺少 {key}")
            passed = False

    # === Check 3: 输入输出维度 ===
    print("\n[Check 3] 输入输出维度...")
    inputs = model.graph.input
    outputs = model.graph.output
    obs_dim = inputs[0].type.tensor_type.shape.dim[1].dim_value
    act_dim = outputs[0].type.tensor_type.shape.dim[1].dim_value
    print(f"  Obs dim: {obs_dim}")
    print(f"  Act dim: {act_dim}")
    if obs_dim == 0 or act_dim == 0:
        print(f"  ❌ 维度为 0，可能是动态维度未固化")
        passed = False
    else:
        print(f"  ✅ 维度合理")

    # === Check 4: Joint names 与 SDK 对比 ===
    if "joint_names" in meta:
        print("\n[Check 4] Joint names 对比...")
        onnx_joints = json.loads(meta["joint_names"])
        print(f"  ONNX joints ({len(onnx_joints)}): {onnx_joints[:5]}...")
        if deploy_yaml_path:
            with open(deploy_yaml_path) as f:
                deploy = yaml.safe_load(f)
            if "joint_mapping" in deploy:
                print(f"  ✅ deploy.yaml 包含 joint_mapping")
            else:
                print(f"  ⚠️ deploy.yaml 缺少 joint_mapping")

    # === Check 5: Obs normalizer 状态 ===
    print("\n[Check 5] Obs normalizer...")
    if "obs_mean" in meta and "obs_var" in meta:
        print(f"  ✅ Normalizer 已 baked-in (meta 包含 mean/var)")
    else:
        # 检查是否有单独的 normalizer 文件
        norm_path = onnx_path.replace(".onnx", "_normalizer.pt")
        import os
        if os.path.exists(norm_path):
            print(f"  ✅ 找到 normalizer 文件: {norm_path}")
        else:
            print(f"  ⚠️ 未找到 normalizer (如果训练未使用归一化则可忽略)")

    # === Check 6: ONNX Runtime 推理测试 ===
    print("\n[Check 6] ONNX Runtime 推理测试...")
    try:
        import onnxruntime as ort
        import numpy as np
        session = ort.InferenceSession(onnx_path)
        dummy = np.zeros((1, obs_dim), dtype=np.float32)
        result = session.run(None, {"obs": dummy})
        action = result[0]
        print(f"  ✅ 推理成功, action shape={action.shape}")
        print(f"  Action 范围: [{action.min():.3f}, {action.max():.3f}]")
        if np.isnan(action).any():
            print(f"  ❌ 推理结果包含 NaN!")
            passed = False
        if np.abs(action).max() > 10:
            print(f"  ⚠️ Action 绝对值 > 10，可能 normalizer 未正确加载")
    except Exception as e:
        print(f"  ❌ 推理失败: {e}")
        passed = False

    # === 总结 ===
    print("\n" + "=" * 60)
    if passed:
        print("✅ D0 验证通过 → 可以进入 D1 (仿真回放)")
    else:
        print("❌ D0 验证失败 → 修复后重新检查")
    print("=" * 60)
    return passed

# 使用
d0_verification(
    "logs/exported/policy.onnx",
    "logs/exported/deploy.yaml",
)
```

### D1 仿真回放验证代码 ⭐⭐

```python
# === D1 仿真回放验证 ===
def d1_replay_verification(
    task: str,
    checkpoint_path: str,
    num_envs: int = 4,
    seed: int = 42,
    num_steps: int = 1000,
):
    """D1: 固定 seed 回放，验证 train/play 一致性。"""
    import torch
    torch.manual_seed(seed)

    env = make(task, num_envs=num_envs)
    policy = torch.jit.load(checkpoint_path)

    obs, _ = env.reset()
    rewards = []
    for step in range(num_steps):
        with torch.no_grad():
            action = policy(obs['policy'])
        obs, reward, done, truncated, info = env.step(action)
        rewards.append(reward.mean().item())

    avg_reward = sum(rewards) / len(rewards)
    print(f"D1 Replay: avg_reward={avg_reward:.4f}")
    print(f"  Expected: 与训练时相同 seed 的 eval reward 一致 (±5%)")

    env.close()
    return avg_reward
```

### 长期运行与热稳定性 ⭐⭐

短时间测试通过不代表长时间运行也没问题。电机温度会随时间上升（改变 PD 行为和力矩上限），电池电压会下降（改变动作幅度），通信链路可能出现间歇性丢包。D6 的验收必须包含长时运行测试——设置保守的速度和动作幅度，运行固定时长的低风险任务，每隔固定时间执行同一测试片段，比较前后性能差异。如果温度、电压或丢包率呈现恶化趋势，必须在恶化到失败之前建立自动降级机制。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：跳过 D0 直接上真机**

"反正训练曲线很好，直接试试真机吧"——这种想法导致的事故比任何其他原因都多。D0 只需要 5 分钟：读取 ONNX metadata、比对 joint names、检查 normalizer 文件是否存在。这 5 分钟可以避免数小时的硬件维修。

💡 **概念误区：认为"D6 通过就可以无人值守运行"**

D6 的通过标准是"在有人监督的条件下完成任务"。无人值守运行需要更高等级的验证——包括长时间运行测试（电机温度、电池续航、通信稳定性）、异常恢复测试（物体丢失、被人推一把、地面突然变滑）、以及完善的远程监控和自动降级机制。

🧠 **思维陷阱：失败后在同一等级反复重试**

如果 D4 失败了（平地低速行走不稳定），正确做法是回退到 D3（悬空低增益）排查原因——是 PD gain 的问题还是延迟的问题还是摩擦的问题。在 D4 反复重试只会浪费时间和增加硬件损坏风险。每次失败都应该有明确的原因归因，修复后从前一个等级重新开始验证。

### 练习

1. **[设计题]** 为 mjlab 训练的 Go2 四足机器人设计一份完整的 D0-D6 验收清单。每个等级列出 3-5 个具体检查项。
2. **[分析题]** D2（影子模式）为什么不需要安全限幅？在什么条件下影子模式本身可能有风险？
3. **[跨章综合题]** 回顾 Ch22 的视觉策略。如果部署的是视觉抓取策略（而非低维状态策略），D0-D6 的每个等级需要增加哪些额外检查项？（提示：camera metadata、preprocessing parity、depth calibration）

---

## 23.9 Perception Gap：视觉部署的特殊挑战 ⭐⭐⭐

> **这一节解决什么问题**：视觉策略的 sim2real 比低维策略多了哪些 gap？如何系统性地处理 perception gap？

### Perception Gap 的四个来源 ⭐⭐

Ch22 讨论了视觉 domain randomization 如何在训练阶段覆盖外观差异。但部署时的 perception gap 不仅仅是"图像看起来不一样"——还包括更深层的系统性差异。

**外观差异（Appearance Gap）**。仿真器渲染的图像和真实相机拍摄的图像在统计性质上不同——即使物理参数完全一致。这是因为渲染引擎的光照模型（通常是简化的 Phong/PBR）和真实世界的光照行为（全局光照、焦散、次表面散射）有结构性差异。Domain randomization 可以部分覆盖这个 gap，但不能完全消除。

**几何差异（Geometric Gap）**。真实深度相机的噪声模式与仿真不同——ToF 相机有多路径反射造成的"飞点"，结构光相机在遮挡边缘有"阴影区"，stereo 相机在无纹理表面失效。这些噪声模式不能用简单的高斯噪声模拟。

**标定差异（Calibration Gap）**。相机的内参（焦距、主点、畸变系数）和外参（安装位姿）在仿真中是精确的，但在真机上有标定误差。内参误差导致像素到 3D 坐标的映射不准确，外参误差导致目标在图像中的位置偏移。

**时序差异（Temporal Gap）**。仿真中图像和状态是完美同步的，但真实系统中相机有采集延迟（曝光时间 + 传输时间），且与 IMU/关节编码器的时间戳不对齐。视觉策略看到的"当前图像"实际上是 20-50 ms 之前的画面。

### 处理 Perception Gap 的系统方法 ⭐⭐

对每种 perception gap，有不同的最佳处理策略：

| Gap | 训练侧处理 | 部署侧处理 |
|-----|-----------|-----------|
| 外观差异 | 光照/材质 DR | 部署场景尽可能简单和可控 |
| 几何差异 | 注入传感器特定的噪声模型 | 选择 sim2real 友好的传感器（depth > RGB） |
| 标定差异 | 相机外参 DR | 部署前重新标定 |
| 时序差异 | 注入帧延迟到 observation buffer | 测量真实延迟并写入部署配置 |

如果不处理 calibration gap 会怎样？一个典型的场景：仿真中相机完美安装在腕部法兰中心、光轴精确指向前方。真机上相机支架有 2 度的安装偏差和 5 mm 的位置偏差。对于距离 30 cm 的目标，2 度的角度偏差导致图像中目标位置偏移约 10 像素（在 64x64 图像中占 15% 视野）。如果策略没有在训练中见过这种偏差（没有做外参 DR），它对偏移后的图像会产生错误的空间推断——抓取持续偏向同一个方向。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：视觉导出包缺少 camera metadata**

ONNX 导出时只保存了策略权重和 joint metadata，但没有保存相机内参、外参和预处理参数。部署端只能猜测这些配置——猜错了策略就失效。正确做法：导出包必须包含完整的 camera schema（分辨率、FOV、near/far、cutoff_distance、CHW/HWC 格式、归一化方式）。

💡 **概念误区：认为"depth 不需要标定"**

Depth 图像确实不受光照影响，但深度值的准确性仍然依赖相机标定。ToF 相机的深度需要温度补偿和距离修正，结构光相机的深度精度随距离下降。如果仿真中假设深度完美准确而真机上的深度有 5% 的系统偏差，对于 30 cm 的目标距离，15 mm 的深度误差可能导致抓取失败。

🧠 **思维陷阱：认为"perception gap 只影响视觉策略"**

低维状态策略也有 perception gap——只是形式不同。Base velocity 来自 IMU+编码器的状态估计器，这个估计器有漂移和噪声。关节编码器有量化误差。Force/torque 传感器有温度漂移。这些都是 perception gap，只不过比视觉 gap 更容易处理（因为维度低、物理模型明确）。

### Perception Gap 的量化方法 ⭐⭐

光说"存在 perception gap"不够——必须量化 gap 的大小，才能判断它是否需要处理。量化的核心方法是：在仿真中模拟真实传感器的特性，然后测量策略性能的变化。

**外观 gap 量化**：用不同的光照和材质渲染同一场景，计算图像统计量（像素均值、方差、频谱）的变化范围。如果变化范围超过训练 DR 覆盖的范围，策略可能在真实图像上失效。具体做法：收集真实场景的 RGB 图像样本（10-20 张），计算其统计量，与仿真 DR 覆盖的统计量范围对比。

**几何 gap 量化**：收集真实深度相机在标准场景（如标定板、已知距离的平面）下的深度图，计算深度误差的分布（均值偏差、标准差、边缘飞点比例）。在仿真 depth 中注入相同分布的噪声，测量策略性能变化。如果性能下降 > 10%，说明策略对 depth 噪声敏感——需要在训练中注入更准确的噪声模型。

**标定 gap 量化**：用标定板测量真实相机的内外参误差。把误差注入仿真相机的参数中，测量策略性能变化。如果外参偏移 2 度导致抓取成功率下降 30%，说明策略对标定误差极其敏感——必须在训练中加入外参 DR 或在部署前精确标定。

**时序 gap 量化**：用硬件时间戳测量真实系统中图像到策略输出的端到端延迟。在仿真中注入相同的帧延迟，测量策略性能变化。这个量化结果直接告诉你："真实系统的延迟是否在策略的容忍范围内？"

每种 gap 的量化结果应该写入部署报告——这是判断"是否可以上真机"的关键依据。如果任何一种 gap 导致仿真性能下降超过 30%，就应该先在训练侧修复（加 DR、改噪声模型、加延迟注入），而不是直接上真机碰运气。

### Perception Gap 与 Physics Gap 的耦合 ⭐⭐

在实际部署中，perception gap 和 physics gap 不是独立的——它们会耦合放大。一个典型的例子：视觉策略依赖 depth 图来估计末端到目标的距离。如果 depth 相机有 5% 的系统性偏差（perception gap），策略会让末端移动到一个"看起来正确但物理上偏了"的位置。然后接触发生时的力不符合仿真预期（physics gap），策略的力控行为也出错。两个 gap 单独都不致命，但耦合后可能导致完全失败。

这种耦合意味着 perception gap 的验收标准不能只看"感知精度"——还要考虑感知误差在控制闭环中的放大效应。一个 5% 的 depth 偏差在开环场景中可能无关紧要，但在闭环抓取中可能被放大 3-5 倍——因为策略基于错误的距离估计做出连续的调整动作，每次调整都积累更多偏差。

> **跨领域类比：Perception-physics 耦合与飞行仪表误差。** 飞行员依赖高度表来保持飞行高度。如果高度表偏差 5%（perception gap），飞行员会让飞机飞到一个错误的高度。在平坦地形上这没什么问题——但在山区飞行时，5% 的高度偏差可能意味着撞山（physics gap 被 perception gap 放大）。同样，视觉策略在"宽容"任务（大物体、慢速度）中对感知偏差容忍度高，但在"精密"任务（小物体、快速度）中同样的偏差可能致命。

### 练习

1. **[分析题]** 一个视觉抓取策略在仿真中成功率 95%，但真机上只有 40%，且每次都偏向同一个方向。按 perception gap 的四个来源逐一排查，哪个最可能是主因？如何确认？
2. **[实验题]** 在 mjlab 中，分别训练两个 depth 策略：一个不做外参 DR，一个做 $\pm$2 度的外参 DR。在 play 时手动偏移相机外参，比较两个策略的鲁棒性。
3. **[设计题]** 为一个视觉操作策略设计完整的 perception gap 验收清单。对于外观、几何、标定、时序四个维度，各列出 2 个具体的测试项和通过标准。

---

## 23.10 System Identification 方法论详解 ⭐⭐⭐

> **这一节解决什么问题**：如何实际执行 System Identification？从哪些参数开始辨识？数据怎么采集？辨识结果怎么验证？

### SysID 的实操流程 ⭐⭐

23.2 节从理论层面介绍了 SysID 的数学基础。本节深入实操层面——因为理论上"最小化仿真与真实的差异"很简单，但实际执行中有大量的工程细节决定了辨识结果的质量。

**步骤一：确定辨识目标。** 不是所有参数都值得辨识。应该优先辨识那些对策略性能影响最大且不确定性最高的参数。对于四足机器人，高优先级参数通常是：足底摩擦系数（直接影响是否打滑）、电机 PD gain（影响关节追踪精度）、base COM 位置偏移（影响平衡）、通信延迟（影响闭环稳定性）。低优先级参数包括：连杆惯性张量（影响较小，且很难测量）、关节阻尼（通常被 PD 控制覆盖）。

**步骤二：设计辨识实验。** SysID 数据的质量取决于实验设计——如果数据中不包含参数的可辨识信号（exciting signal），无论用什么算法都辨识不出来。例如：要辨识摩擦系数，机器人必须在接近打滑的条件下运动（低速转弯、侧向推动）；要辨识延迟，必须有快速的状态变化（突然的速度指令切换）并精确记录时间戳。

一个常用的辨识实验序列：

```text
实验 1: 静态称重 -> 总质量和 COM 位置
实验 2: 单关节正弦扫频 -> PD gain 和关节摩擦
实验 3: 直线加减速 -> 纵向摩擦系数
实验 4: 原地旋转 -> 横向摩擦系数
实验 5: 阶跃指令响应 -> 端到端延迟
实验 6: 悬空自由振荡 -> 关节阻尼和惯性
```

**步骤三：参数拟合。** 用优化算法（如 CMA-ES、Bayesian Optimization 或梯度下降）调整仿真参数使仿真轨迹匹配真实轨迹。关键细节：拟合时不应该同时优化所有参数——参数之间的耦合可能导致不可辨识性（多组不同参数产生相同的轨迹）。正确做法是分组辨识：先用静态实验确定质量和 COM，然后固定这些参数再辨识摩擦，最后辨识延迟。

**步骤四：交叉验证。** 辨识结果必须在辨识实验之外的数据上验证——如果辨识用了直线行走数据，验证应该用转弯或斜坡数据。如果验证误差远大于辨识误差，说明模型结构不足（不是参数不准，而是方程形式不对）——此时需要考虑 Fine-tuning 而非继续辨识。

**SysID 实操代码——单关节正弦扫频**：

```python
# === SysID: 单关节正弦扫频辨识 PD gain ===
import numpy as np

def sine_sweep_sysid(
    sdk_interface,
    joint_idx: int = 0,
    freq_range: tuple = (0.5, 5.0),  # Hz
    amplitude: float = 0.1,          # rad
    duration: float = 30.0,          # seconds
    control_freq: float = 500.0,     # Hz
):
    """通过正弦扫频辨识单个关节的 PD gain 和延迟。

    原理：施加已知频率和幅度的正弦位置命令，
    记录实际关节角度响应，从频率响应中提取 PD 参数。
    """
    dt = 1.0 / control_freq
    t = np.arange(0, duration, dt)

    # 线性扫频信号（chirp）
    freqs = np.linspace(freq_range[0], freq_range[1], len(t))
    phases = np.cumsum(2 * np.pi * freqs * dt)
    cmd_positions = amplitude * np.sin(phases)

    # 采集数据
    actual_positions = []
    timestamps = []
    for i, cmd in enumerate(cmd_positions):
        sdk_interface.set_joint_position(joint_idx, cmd)
        state = sdk_interface.get_joint_state(joint_idx)
        actual_positions.append(state.position)
        timestamps.append(state.timestamp)
        sdk_interface.wait_next_cycle()

    actual_positions = np.array(actual_positions)
    timestamps = np.array(timestamps)

    # 分析：计算每个频率段的增益和相移
    segment_len = int(control_freq * 2)  # 2 秒一段
    gains = []
    phase_delays = []
    for start in range(0, len(t) - segment_len, segment_len):
        end = start + segment_len
        cmd_seg = cmd_positions[start:end]
        act_seg = actual_positions[start:end]
        freq = freqs[start + segment_len // 2]

        # 用 FFT 提取基频分量
        cmd_fft = np.fft.fft(cmd_seg)
        act_fft = np.fft.fft(act_seg)
        freq_idx = int(freq * segment_len / control_freq)

        gain = np.abs(act_fft[freq_idx]) / (np.abs(cmd_fft[freq_idx]) + 1e-10)
        phase = np.angle(act_fft[freq_idx]) - np.angle(cmd_fft[freq_idx])
        delay_ms = -phase / (2 * np.pi * freq) * 1000

        gains.append((freq, gain))
        phase_delays.append((freq, delay_ms))

    print(f"=== Joint {joint_idx} SysID Results ===")
    for freq, gain in gains:
        print(f"  {freq:.1f} Hz: gain={gain:.3f}")
    avg_delay = np.mean([d for _, d in phase_delays])
    print(f"Average delay: {avg_delay:.1f} ms")

    return {
        "gains": gains,
        "phase_delays": phase_delays,
        "avg_delay_ms": avg_delay,
        "raw_cmd": cmd_positions,
        "raw_actual": actual_positions,
    }
```

这段代码的关键工程细节：
- **扫频信号（chirp）而非单频正弦**：扫频在一次实验中覆盖多个频率，减少实验时间。但频率不能变化太快——每个频率至少需要 2-3 个完整周期才能获得稳定的频率响应
- **分段 FFT 分析**：把数据分成 2 秒一段，每段对应一个近似恒定的频率。用 FFT 提取该频率的增益和相移
- **延迟从相移中推算**：相移 $\phi$ 和延迟 $\tau$ 的关系是 $\tau = -\phi / (2\pi f)$。在低频段这个估计很准确，高频段会被 PD 控制器的滤波效应干扰

**SPI-Active（2025 新方法）**：LeCAR-Lab 的 SPI-Active（arXiv:2505.14266）提出了一种基于采样的主动探索 SysID 方法——策略在辨识阶段主动执行能最大化参数可辨识性的动作，而非被动地做预设实验。这比传统的扫频实验更高效，但需要在线优化辨识动作——实现复杂度更高。

### SysID 与 DR 的协同 ⭐⭐

辨识完成后，仿真参数的均值已经接近真实值。但辨识结果有不确定性——测量噪声、实验条件变化、参数时变性都会导致辨识值与真实值有偏差。DR 的范围应该基于辨识的不确定性来设置，而不是凭经验猜测。

具体做法：如果摩擦系数的辨识结果是 $\mu = 0.65 \pm 0.1$（通过多次实验的标准差估计），那么 DR 范围设为 $[0.45, 0.85]$（均值 $\pm$2 倍标准差）。这比盲猜的 $[0.3, 1.5]$ 窄得多——训练更高效，策略在辨识值附近更优化，同时仍覆盖合理的不确定性范围。

如果不做 SysID 而直接用很宽的 DR 范围，策略需要在一个大得多的参数空间上"面面俱到"——这意味着它在任何单个参数点上的性能都不如 SysID + 窄 DR 的策略。这就是为什么顶级的 sim2real 工作（如 ETH 的四足、MIT 的 Cheetah）都同时使用 SysID 和 DR——SysID 缩小"搜索空间"，DR 覆盖"残余不确定性"。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：辨识数据的时间戳不准**

SysID 的核心是对比仿真和真实的轨迹——时间对齐是前提。如果真实数据的时间戳有毫秒级的不确定性（比如 Python 的 `time.time()` 在 Windows 上精度只有 ~15 ms），辨识出的延迟参数就不可信。正确做法：使用硬件时间戳（如 SDK 提供的关节编码器时间戳），或者在数据中嵌入硬件同步信号。

💡 **概念误区：认为"辨识一次就够了"**

电机的 PD gain 随温度变化（热机后 gain 下降 5-15%），摩擦系数随地面状况变化，电池电压随使用时间下降。一个月前的辨识结果可能已经不准了。最佳实践是在每次重要测试前做一次简化版 SysID（只辨识最关键的 2-3 个参数），确认参数没有显著漂移。

### 练习

1. **[设计题]** 为 Unitree Go2 设计一套 SysID 实验序列。列出每个实验的目的、执行方式、记录的数据和期望辨识的参数。
2. **[计算题]** 摩擦系数辨识结果为 $\mu = 0.7 \pm 0.15$。如果 DR 范围设为均值 $\pm$2 倍标准差，范围是多少？与"凭经验猜测的 $[0.3, 1.5]$"相比，范围缩小了多少倍？
3. **[分析题]** 为什么不能同时辨识质量和摩擦系数？提示：考虑 $F = \mu m g$ 中的参数耦合。

---

## 23.11 真实部署的闭环迭代 ⭐⭐

> **这一节解决什么问题**：sim2real 不是一次性的——它是一个从仿真到真机再回到仿真的迭代循环。这个循环的每一步产出什么、反馈什么？

### 部署迭代的完整循环 ⭐⭐

成功的 sim2real 部署几乎不可能一次完成。更现实的模式是一个迭代循环：在仿真中训练 -> 在真机上测试 -> 发现问题 -> 回到仿真修正 -> 重新训练 -> 再次测试。关键是让每次迭代都有明确的信息流——每次真机测试不仅产出"成功/失败"的二元结果，更产出"失败的具体原因"和"应该在仿真中修改什么"的诊断信息。

```text
迭代循环：

仿真训练 ──────────────────────────────────────┐
    │                                           │
    ▼                                           │
D0-D1 离线验证 ───── 失败 ──→ 修配置/映射 ─────┘
    │ 通过                                      │
    ▼                                           │
D2-D3 低风险真机 ─── 失败 ──→ 修 PD/延迟/映射 ─┘
    │ 通过                                      │
    ▼                                           │
D4-D5 任务测试 ──── 失败 ──→ SysID/扩大 DR ────┘
    │ 通过                                      │
    ▼                                           │
D6 完整部署 ──────── 失败 ──→ Fine-tuning ──────┘
    │ 通过
    ▼
  部署成功
```

**每次迭代的信息反馈**：

D0-D1 失败反馈的是**软件接口问题**——关节映射、normalizer、action scale。修复成本最低，几分钟到几小时。

D2-D3 失败反馈的是**执行器和时序问题**——PD gain 不匹配、延迟未补偿、关节方向错误。修复需要在仿真中调整对应参数后重新训练，成本几小时到一天。

D4-D5 失败反馈的是**物理参数和感知问题**——摩擦不匹配、地形覆盖不足、视觉 gap。修复需要 SysID + 扩大 DR 范围后重新训练，成本一到几天。

D6 失败反馈的是**残余结构性差异**——模型结构不足以捕捉的物理现象。修复可能需要 fine-tuning 或更复杂的模型，成本几天到几周。

### 关键原则：每次真机测试都必须有诊断产出 ⭐⭐

"跑了一下，失败了，明天再试"——这是最浪费时间的做法。每次真机测试（即使只有 30 秒）都必须产出以下诊断信息：

1. **完整的 telemetry 日志**：obs、action、latency、safety events、视频时间戳
2. **失败的第一触发条件**：不是"摔倒了"，而是"pitch 超过 30 度时 action 仍在输出最大力矩"
3. **与仿真的对比**：同一个 checkpoint 在仿真中的行为 vs 真机上的行为，差异在哪个 observation 维度上最大
4. **下一步行动项**：基于诊断结果，明确应该修改仿真中的哪个参数/配置

如果真机测试没有产出诊断信息，这次测试就是白费的——因为你不知道为什么失败，也不知道下次应该改什么。

### 如果不做迭代而"一次搞定" ⭐

如果试图跳过迭代循环直接部署完整任务，最可能的结果是：花了几天训练了一个很好的仿真策略，上真机第一帧就失败（因为 joint order 错了），修完 joint order 又失败（因为延迟没补偿），修完延迟又失败（因为摩擦不匹配），修完摩擦又失败（因为视觉 gap）……每次都需要重新训练，总时间远超逐步迭代的方案。正确的做法是从最简单的测试开始，每次只推进一步，确保前面的问题都已解决后再增加复杂度。

> **跨领域类比：部署迭代与软件发布的 CI/CD。** 现代软件开发不会攒三个月的代码一次性发布——而是每天做小幅发布，每次发布都有自动化测试确认不破坏已有功能。Sim2real 的分级验收就是 sim2real 领域的 CI/CD——每个"等级"相当于一层测试，通过了才能"部署"到下一层。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：真机日志格式与仿真日志不兼容**

如果真机的 observation 记录格式（维度顺序、单位、采样率）与仿真不同，对比分析就变得极其困难。正确做法：真机端使用与仿真完全相同的 observation schema——维度名称、单位和顺序都一致，这样可以直接用同一套分析脚本处理两端的数据。

💡 **概念误区：认为"真机数据越多越好"**

真机数据的价值不在于量——而在于多样性和标注质量。100 次相同条件下的重复试验不如 10 次不同条件下的诊断性试验。每次真机试验都应该有明确的目的："验证摩擦辨识是否准确"、"测试延迟 2 帧下的行为"、"确认左前腿映射正确"。无目的的"看看行不行"式试验是对真机时间的浪费。

### 练习

1. **[流程题]** 你的四足机器人在 D4（平地低速）测试中，直线行走稳定但转弯时摔倒。按迭代循环，你应该回退到哪个等级？应该在仿真中修改什么？下一次真机测试的目标是什么？
2. **[设计题]** 设计一个"真机测试诊断模板"——列出每次真机测试后必须记录的 10 个字段，以及每个字段如何指导下一次迭代。
3. **[跨章综合题]** 回顾 Ch22 的视觉管线。如果部署的是视觉抓取策略，迭代循环中的每一步需要额外检查哪些视觉相关的内容？（提示：camera calibration、depth noise profile、preprocessing parity）

---

## 23.12 工程实战案例 ⭐⭐

> **这一节解决什么问题**：如何将前面的理论应用到具体的部署场景中？每个案例演示一个完整的 sim2real 排查和验收流程。

### 案例 23-1：Go1 平地速度策略部署预演 ⭐⭐

**背景**：平地 velocity 策略在仿真中稳定，但真机前需要检查动作尺度、关节顺序和延迟。这是最基础的 sim2real 流程——如果这个流程走不通，任何更复杂的任务都不可能成功。

**D0 静态核对**：

导出 ONNX 和 metadata。逐项检查：`joint_names` 是否与 SDK 的关节名一一对应？`action_scale` 是否与训练配置一致？`default_joint_pos` 是否与机器人的零位定义一致？observation_names 的顺序是否与部署端拼接的顺序一致？normalizer 文件是否存在且与 ONNX 来自同一个 checkpoint？

这些检查项看起来繁琐但极其重要。一个真实案例：某团队的 ONNX metadata 中 `joint_names` 按字母顺序排列（FR_hip, FR_knee, FR_thigh, ...），但 SDK 按拓扑顺序排列（FR_hip, FR_thigh, FR_knee, ...）——导致所有腿的 thigh 和 knee 动作互换。在仿真中这个问题不存在（因为两端都用同一个 MJCF），只有到了真机才暴露。

**D1 仿真回放**：

用同一个 checkpoint 做 `uv run play` 回放。记录 action 的 min/max/mean/std。确认 play 的行为与训练末期的行为一致（步态稳定、不抖动、能追踪速度指令）。如果 play 和 train 行为不一致，最常见的原因是 observation normalization 的 running mean/std 在 play 时没有正确加载。

```bash
uv run play Mjlab-Velocity-Flat-Unitree-Go1 \
  --checkpoint /path/to/model_xxx.pt \
  --viewer viser \
  --num-envs 4
# 观察：步态是否稳定？速度指令是否被追踪？动作是否平滑？
```

**D2 影子模式**：

策略运行在真机旁边的计算机上，接收真实 IMU 和关节编码器数据，输出 action——但 action 不下发给电机。只记录日志。检查：policy 输出的 action 是否在合理范围内（不超过 action_scale）？输出的 action 是否随关节状态合理变化（不是固定值或 NaN）？

**D3 悬空低增益**：

把机器人悬挂起来（四脚离地），PD gain 设为训练值的 20%。启动策略，观察每条腿的运动。逐个关节确认：发送正方向命令时关节向正方向运动？运动幅度与 action 数值成正比？没有不预期的关节运动（说明映射正确）？

**通过标准**：所有四条腿的 12 个关节方向正确、幅度合理、无抖动。任何一个关节方向错误都必须修正映射后重新开始 D3。

### 从训练完成到第一次真机行走的典型时间线 ⭐

以下是一个有 Ch22-Ch23 实战经验的读者，把 mjlab 训练的四足策略部署到 Unitree Go2 的典型时间投入。

| 时间 | 任务 | 预期产出 | 常见卡点 |
|------|------|---------|---------|
| Day 1 上午 | ONNX 导出 + D0 验证 | metadata 全匹配 | normalizer 遗漏 |
| Day 1 下午 | D1 仿真回放 + D2 影子模式 | action 在安全范围内 | train/play 不一致 |
| Day 2 上午 | C++ 控制器编译 + Sim2Sim | unitree_mujoco 中行走 | joint mapping 错 |
| Day 2 下午 | D3 悬空低增益测试 | 12 关节方向全正确 | SDK joint order $\ne$ URDF |
| Day 3 上午 | D4 平地低速行走 | 0.3 m/s 稳定行走 | 延迟未补偿 |
| Day 3 下午 | D5 速度指令范围测试 | 0.8 m/s 前进 + 转弯 | 摩擦 gap |
| Day 4 | D6 完整任务 + 长时测试 | 5 分钟稳定行走 | 电机温升 |

**如果你是第一次做 sim2real**，实际时间可能是上述的 3-5 倍。主要额外时间花在：理解 C++ 控制器的编译和配置（如果之前没接触过 CycloneDDS/CMake），排查 SDK 版本兼容性问题，以及反复在 D3-D4 之间迭代（关节映射→delay→摩擦→PD gain 的排查循环）。

### 部署报告模板 ⭐

每次真机测试都应该生成一份部署报告。以下是模板的核心字段：

```markdown
# 部署报告 [日期] [机器人ID] [任务名]

## 策略信息
- Checkpoint: model_xxxx.pt (hash: abc123)
- ONNX: policy.onnx (hash: def456)
- 训练 task: Mjlab-Velocity-Flat-Unitree-Go2
- 训练 iterations: 10,000
- 训练 WandB run: https://wandb.ai/...

## 验证等级
- D0: ✅ 通过 (时间: 2026-05-20 10:00)
- D1: ✅ 通过 (时间: 2026-05-20 10:30)
- D2: ✅ 通过 (时间: 2026-05-20 14:00)
- D3: ✅ 通过 (时间: 2026-05-21 09:00)
- D4: ⚠️ 部分通过 (时间: 2026-05-21 14:00)
  - 0.3 m/s 前进: ✅ 稳定
  - 0.5 m/s 前进: ✅ 稳定
  - 0.3 m/s 转弯: ❌ 左转时滑动

## 延迟预算
- Obs latency p50/p95: 1.2 / 2.8 ms
- Policy latency p50/p95: 1.5 / 3.2 ms
- Total latency p50/p95: 4.1 / 8.5 ms
- 训练 delay budget: 10 ms

## 安全配置
- Watchdog timeout: 200 ms
- Max tilt (roll/pitch): 30° / 30°
- Action rate limit: 0.5 rad/step
- Emergency stop: 硬件按钮 + 软件 watchdog

## 失败分析 (D4 转弯失败)
- 第一触发条件: 左前脚在转弯时滑动,
  base yaw rate 指令 0.5 rad/s,
  实际 yaw rate < 0.2 rad/s
- 可能原因: 地面摩擦 < 训练 DR 下限
- 计划修复: 
  1. SysID 测量真实地面摩擦
  2. 降低 DR 摩擦下限到 0.3
  3. 重新训练 2000 iterations
  4. 重新从 D3 开始验收

## Telemetry 文件
- 日志: logs/deploy/20260521_140000.jsonl
- 视频: videos/20260521_140000.mp4
- 时长: 45 秒 (在第 38 秒触发安全停止)
```

这份报告的关键原则：**记录失败案例比记录成功更重要**。成功的视频只能证明"策略在这个条件下工作"，但失败的分析告诉你"策略在什么条件下失败、为什么失败、下一步怎么修"——这些信息是迭代改进的基础。

### 案例 23-2：视觉策略的 train/play/export 一致性验证 ⭐⭐

**背景**：视觉策略导出后动作和仿真播放不一致。这是视觉部署中最常见也最隐蔽的问题——因为差异可能很小（几个百分点），但足以导致抓取失败。

**核心方法：固定输入比较**

保存一段固定 episode 的原始图像（raw depth/RGB tensor）和低维状态。用训练时的预处理管线和导出后的预处理管线分别处理同一帧输入。逐层比较：预处理后的张量是否完全一致？如果经过 normalizer，mean/std 是否一致？CNN 的输出 feature 是否一致？最终的 action 输出是否一致？

```python
# 一致性检查的核心逻辑
def check_parity(train_pipeline, export_pipeline, raw_input):
    """比较训练管线和导出管线对同一输入的处理是否一致。"""
    train_obs = train_pipeline.preprocess(raw_input)
    export_obs = export_pipeline.preprocess(raw_input)
    
    # 预处理后的张量必须完全一致
    preprocess_diff = (train_obs - export_obs).abs().max().item()
    assert preprocess_diff < 1e-6, f"预处理不一致: max diff = {preprocess_diff}"
    
    train_action = train_pipeline.forward(train_obs)
    export_action = export_pipeline.forward(export_obs)
    
    # action 输出必须完全一致（同一网络权重，同一输入）
    action_diff = (train_action - export_action).abs().max().item()
    assert action_diff < 1e-5, f"Action 不一致: max diff = {action_diff}"
    
    return True
```

**常见不一致来源**：

| 不一致来源 | 表现 | 修复方法 |
|-----------|------|---------|
| RGB 归一化常数不同 | action 有微小但系统性偏差 | 统一使用 /255.0 |
| depth cutoff 不同 | 目标距离映射不同 | 从配置文件读取而非硬编码 |
| normalizer 版本不匹配 | action 大幅偏差 | 确认 normalizer 与 checkpoint 同源 |
| CHW/HWC 格式不一致 | CNN 输出完全错误 | 统一在一处做 permute |
| camera 顺序不同 | 多相机时 feature 拼接错位 | 按名称而非索引排序 |

### 案例 23-3：移动底盘加机械臂联合验收 ⭐⭐

**背景**：底盘和机械臂单独可用，联合执行时底盘姿态扰动导致抓取失败。这个案例演示了移动操作的分层验收流程。

**分离验证**：

第一步，固定底盘（制动），只执行 arm reach/grasp。验证关节映射、gripper 方向和末端精度。如果固定基座下 reach 精度差，问题在 arm 控制而非底盘耦合——先修 arm 再联调。

第二步，固定 arm 姿态（安全位置），只执行底盘低速移动。验证行走稳定性、制动效果和姿态稳定（pitch/roll 不超过 $\pm$5 度）。

第三步，arm 夹持已知质量物体（如 200g 水瓶），底盘行走。验证 payload 对底盘稳定性的影响——如果 payload 导致底盘不稳，说明 domain randomization 中的 payload 范围不够。

**耦合验证**：

第四步，底盘移动到目标附近并停稳（速度降至 <0.05 m/s），然后 arm reach。这验证了"阶段切换"的时序——如果底盘还没稳定就开始 reach，相机画面仍在晃动，视觉定位不准。

第五步，在限速、限幅和观察员就位的条件下执行完整任务：移动到目标附近 -> 停稳 -> reach -> grasp -> lift -> 搬运。每个阶段单独记录成功率，失败时归因到 base、arm 或 perception。

**关键原则**：失败时回退到更简单的片段，不在复杂片段上反复尝试。如果联合执行失败但分离验证通过，问题在耦合点——检查坐标系统一性、动作空间分组和时序门控。

### 案例 23-4：Watchdog 与急停链路验证 ⭐⭐

**背景**：策略正常时看不到安全链路是否有效，异常时才发现已经太晚。这个案例的核心是"主动触发每个安全分支"。

**测试矩阵**：

| 触发条件 | 预期行为 | 实际测量 | 通过标准 |
|---------|---------|---------|---------|
| Policy timeout (不发送 action) | Watchdog 触发安全姿态 | 触发延迟 < 100 ms | 电机进入制动模式 |
| Observation stale (冻结传感器数据) | 检测到数据过旧并停止 | 检测延迟 < 50 ms | Action 输出冻结为安全值 |
| Action out-of-range (人为注入极大 action) | 限幅裁剪 | 电机命令 < 限值 | 无超限动作下发 |
| Tilt threshold (人为倾斜机器人) | 姿态保护触发 | 触发角度与设定一致 | 关节锁定 |
| 人工急停按钮 | 立即断电或制动 | 停止延迟 < 50 ms | 所有电机同时停止 |
| 物理急停（拔电源线） | 弹簧/摩擦制动 | 机器人不失控滑行 | 停在原地不倒 |

每个分支都必须有触发记录。如果任何一个分支的实际行为与预期不符，**不允许执行 D4 及以上等级的测试**。安全链路是部署的前提，不是部署的附加项。

### 案例 23-5：长期运行与热稳定性 ⭐⭐

**背景**：短时间测试通过，但长时间运行出现电机温度、电池电压和通信抖动问题。

**实验设计**：

设置保守速度（最大速度的 50%）和动作幅度（action_scale 的 70%）。运行固定时长（30 分钟）的低风险任务（平地来回行走）。每 5 分钟记录：电机温度（12 个关节）、电池电压、丢包率、policy latency（p50/p95）。每 10 分钟执行同一标准测试片段（直线行走 3 米 + 转弯 90 度 + 原地站立 10 秒），比较前后性能差异。

**预期退化模式**：

电机温度随时间上升，20 分钟后 PD gain 可能因为温度漂移而与训练值偏差 5-10%。电池电压随时间下降，30 分钟后可能降低 10-15%，影响最大力矩输出。通信 jitter 在 CPU 负载高时增大（比如同时运行视觉推理和 telemetry 记录）。

**通过标准**：30 分钟内电机温度不超过安全阈值的 80%，标准测试片段的成功率无系统性下降（允许 $\pm$10% 波动）。如果温度持续上升，必须降低 duty cycle 或添加温度相关的动作降额逻辑。

### 部署报告核心字段 ⭐

每次真机测试必须生成一份部署报告。报告不是汇报材料——而是事故复盘材料。以下字段是最小集合：

| 字段 | 记录内容 | 缺失风险 |
|------|---------|---------|
| **policy artifact** | checkpoint/ONNX/normalizer 文件名与 hash | 不能证明真机执行的是哪个策略 |
| **task identity** | task id、robot asset、control dt、训练 commit | 任务名相似时必须用完整 id |
| **observation schema** | 每个 obs 的名称、shape、单位、来源 | 来源不明的 obs 不能进入 actor |
| **action schema** | 每个 action 维度对应的关节和命令类型 | 混合 action space 必须分组 scale |
| **joint mapping** | 仿真 joint 与 SDK joint 的对应表 | 必须经过单关节小幅动作验证 |
| **latency budget** | 四段延迟的 p50/p95 | p95 超训练假设时回仿真重测 |
| **safety supervisor** | watchdog/tilt/stale 阈值和触发记录 | 所有阈值必须能在日志中看到 |
| **test level** | 当前处于 D0-D6 哪个等级 | 跳级测试必须写明理由 |
| **telemetry** | obs/action/latency/safety events/视频 | 日志不能对齐视频时结果不可复盘 |
| **failure trigger** | 失败时第一触发条件 | 不要只写"失败"，要写谁先越界 |
| **go/no-go** | 执行前检查结论 | 任一安全项未通过就是 no-go |

## 23.13 部署 Telemetry 与日志分析 ⭐⭐

> **这一节解决什么问题**：如何在真机部署中记录足够的诊断信息，使得每次测试——无论成功还是失败——都能产出可分析的数据？

### 动机：没有日志的测试等于浪费

一个不幸的现实是：大多数真机测试的日志不够详细——团队测试了 20 次，失败了 15 次，但只有"摔了"这个信息，没有记录摔倒前一秒的 obs/action/latency。两周后回顾时，没人记得具体情况，只能重新测试。

> **本质洞察**：Telemetry 的目标不是"记录一切"——而是**确保任何一次失败都能在离线复盘中定位到七层 gap 分类中的具体层**。如果你的日志能做到这一点，每次真机测试都是在积累诊断信息，最终收敛到部署成功。

### 最小 Telemetry Schema ⭐⭐

```python
# === 部署 Telemetry 记录器 ===

import time
import json
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import List, Optional

@dataclass
class TelemetryFrame:
    """单帧 telemetry 数据。"""
    timestamp: float              # 硬件时间戳 (seconds)
    step: int                     # 控制循环计数
    obs: List[float]              # 策略输入 (原始, 归一化前)
    obs_normalized: List[float]   # 策略输入 (归一化后)
    action: List[float]           # 策略输出 (后处理前)
    action_clipped: List[float]   # 策略输出 (限幅后, 发给电机)
    joint_pos_cmd: List[float]    # 关节位置命令
    joint_pos_actual: List[float] # 关节位置反馈
    joint_vel_actual: List[float] # 关节速度反馈
    imu_acc: List[float]          # IMU 加速度 [ax, ay, az]
    imu_gyro: List[float]        # IMU 角速度 [wx, wy, wz]
    base_rpy: List[float]        # 估计的 roll/pitch/yaw
    latency_obs_ms: float        # 传感器读取延迟
    latency_policy_ms: float     # 策略推理延迟
    latency_total_ms: float      # 端到端延迟
    sensor_age_ms: float         # 传感器数据新鲜度（当前时间 - 传感器时间戳）
    motor_temperature: Optional[List[float]] = None  # 电机温度（℃），硬件支持时填入
    safety_events: List[str]     # 安全事件 (如 "tilt_warning")

class TelemetryLogger:
    """部署日志记录器。"""
    def __init__(self, log_dir: str, robot_name: str):
        self.frames: List[TelemetryFrame] = []
        self.log_path = f"{log_dir}/{robot_name}_{int(time.time())}.jsonl"
        self.file = open(self.log_path, 'w')

    def log_frame(self, frame: TelemetryFrame):
        """实时记录一帧数据。"""
        self.frames.append(frame)
        # 实时写入文件（防止崩溃丢数据）
        self.file.write(json.dumps(asdict(frame)) + '\n')
        self.file.flush()

    def analyze(self):
        """离线分析日志。"""
        if not self.frames:
            print("No frames to analyze")
            return

        latencies = [f.latency_total_ms for f in self.frames]
        print(f"=== Telemetry Analysis ===")
        print(f"Total frames: {len(self.frames)}")
        print(f"Duration: {self.frames[-1].timestamp - self.frames[0].timestamp:.1f} s")
        print(f"Latency p50/p95/p99: "
              f"{np.percentile(latencies, 50):.1f}/"
              f"{np.percentile(latencies, 95):.1f}/"
              f"{np.percentile(latencies, 99):.1f} ms")

        # 检查安全事件
        safety_events = [e for f in self.frames for e in f.safety_events]
        if safety_events:
            from collections import Counter
            print(f"Safety events: {dict(Counter(safety_events))}")
        else:
            print("Safety events: None ✅")

        # 检查 action 限幅频率
        clipped_count = sum(
            1 for f in self.frames
            if f.action != f.action_clipped
        )
        clip_rate = clipped_count / len(self.frames)
        print(f"Action clip rate: {clip_rate:.1%}")
        if clip_rate > 0.1:
            print(f"  ⚠️ >10% 的帧被限幅——策略可能输出过大的动作")

        # 关节追踪误差
        tracking_errors = []
        for f in self.frames:
            err = np.abs(np.array(f.joint_pos_cmd) - np.array(f.joint_pos_actual))
            tracking_errors.append(err.max())
        print(f"Joint tracking error (max): "
              f"mean={np.mean(tracking_errors):.4f} rad, "
              f"max={np.max(tracking_errors):.4f} rad")

        # 传感器数据新鲜度
        sensor_ages = [f.sensor_age_ms for f in self.frames]
        print(f"Sensor age p50/p95: "
              f"{np.percentile(sensor_ages, 50):.1f}/"
              f"{np.percentile(sensor_ages, 95):.1f} ms")
        if np.percentile(sensor_ages, 95) > 20.0:
            print(f"  ⚠️ 传感器数据过旧（p95 > 20ms）——检查同步线程和丢帧策略")

        # 电机温度趋势（如果硬件支持）
        temps = [f.motor_temperature for f in self.frames if f.motor_temperature]
        if temps:
            max_temps = [max(t) for t in temps]
            print(f"Motor temperature max: "
                  f"start={max_temps[0]:.1f}°C, end={max_temps[-1]:.1f}°C, "
                  f"peak={max(max_temps):.1f}°C")
            if max_temps[-1] > max_temps[0] + 10:
                print(f"  ⚠️ 电机温度持续上升（+{max_temps[-1]-max_temps[0]:.1f}°C）"
                      f"——考虑降额或缩短测试时间")

    def find_failure_trigger(self):
        """找到导致失败的第一个异常帧。"""
        for i, f in enumerate(self.frames):
            # 检查 base tilt
            if abs(f.base_rpy[0]) > 0.5 or abs(f.base_rpy[1]) > 0.5:
                print(f"First tilt anomaly at step {f.step} "
                      f"(t={f.timestamp:.3f}s): "
                      f"roll={f.base_rpy[0]:.2f}, pitch={f.base_rpy[1]:.2f}")
                # 打印前后 5 帧的 action
                start = max(0, i - 5)
                for j in range(start, min(i + 5, len(self.frames))):
                    a = self.frames[j]
                    print(f"  step {a.step}: action_max={max(a.action):.3f}, "
                          f"latency={a.latency_total_ms:.1f}ms")
                return i
        return None
```

### Telemetry 与仿真日志的对齐 ⭐⭐

真机日志最有价值的用途是**与仿真日志对比**——找出两者的差异在哪些 obs 维度上最大。这个对比的前提是两端使用相同的 schema：

```python
# === 仿真-真机日志对比分析 ===
def compare_sim_real_logs(sim_log_path, real_log_path):
    """对比仿真和真机日志，找出最大差异维度。"""
    sim_frames = load_jsonl(sim_log_path)
    real_frames = load_jsonl(real_log_path)

    # 对齐时间（从 reset 时刻开始计算相对时间）
    sim_t = np.array([f['timestamp'] for f in sim_frames])
    real_t = np.array([f['timestamp'] for f in real_frames])
    sim_t -= sim_t[0]
    real_t -= real_t[0]

    # 对齐到相同的时间点
    common_len = min(len(sim_frames), len(real_frames))
    obs_names = ["base_vel_x", "base_vel_y", "base_vel_z",
                 "ang_vel_x", "ang_vel_y", "ang_vel_z",
                 "proj_grav_x", "proj_grav_y", "proj_grav_z",
                 # ... 更多 obs 名称 ...
                ]

    # 逐维度对比
    max_diffs = {}
    for dim_idx, name in enumerate(obs_names):
        sim_vals = [sim_frames[i]['obs'][dim_idx] for i in range(common_len)]
        real_vals = [real_frames[i]['obs'][dim_idx] for i in range(common_len)]
        diff = np.abs(np.array(sim_vals) - np.array(real_vals))
        max_diffs[name] = {
            'mean': diff.mean(),
            'max': diff.max(),
            'std': diff.std(),
        }

    # 按差异大小排序
    sorted_diffs = sorted(max_diffs.items(), key=lambda x: x[1]['mean'], reverse=True)
    print("=== Sim-Real Obs Difference (sorted by mean) ===")
    for name, stats in sorted_diffs[:10]:
        print(f"  {name:20s}: mean={stats['mean']:.4f}, "
              f"max={stats['max']:.4f}, std={stats['std']:.4f}")
    print("\n最大差异的维度最可能是 gap 的主要来源。")
```

这个分析的输出直接告诉你"gap 主要来自哪里"：如果 `base_vel_z` 差异最大，说明垂直方向的速度估计有问题（可能是 IMU 漂移或地面接触模型不同）；如果 `proj_grav_x/y` 差异大，说明 base 姿态估计有偏差；如果某个关节的 `joint_pos` 差异大，说明该关节的 PD 追踪有问题。

---

## 23.14 Sim2Real Checklist（20 项检查） ⭐⭐⭐

> **这一节解决什么问题**：给出一份可打印的、覆盖全部七层 gap 的部署前检查清单。

以下清单是 §23.1 七层分类的实操落地——每层至少 2-3 个具体检查项，每项有明确的"通过/不通过"标准。在第一次真机测试前逐项勾选，任何一项不通过都不应该上真机。

### 软件接口层（最先检查）

| # | 检查项 | 通过标准 | 验证方法 |
|---|--------|---------|---------|
| 1 | ONNX metadata 中的 `joint_names` 与 SDK 关节名逐一对应 | 全匹配，顺序明确 | D0 脚本自动对比 |
| 2 | `action_scale` 与训练配置一致 | 数值差异 = 0 | 读取 deploy.yaml 对比 |
| 3 | `default_joint_pos` 与 MJCF/USD 的初始关节角一致 | 差异 < 0.01 rad | 数值对比 |
| 4 | Obs 维度与策略网络输入维度一致 | 精确匹配 | ONNX input shape 检查 |

### 时序层

| # | 检查项 | 通过标准 | 验证方法 |
|---|--------|---------|---------|
| 5 | 端到端延迟 p95 < 控制周期的 50% | p95 < 10ms (50Hz) | LatencyProfiler 测量 |
| 6 | 训练 decimation $\times$ sim_dt = 真机 control_dt | 精确匹配 | 配置文件对比 |
| 7 | 训练中注入的 action delay $\ge$ 真机 p95 延迟 | delay_range 覆盖 | 训练配置检查 |

### 执行器层

| # | 检查项 | 通过标准 | 验证方法 |
|---|--------|---------|---------|
| 8 | 每个关节的 PD Kp/Kd 与训练一致 | 差异 < 5% | 单关节阶跃响应对比 |
| 9 | 关节力矩限制在安全范围内 | 不超过 datasheet 额定值 | SDK 配置检查 |
| 10 | 单关节正弦测试方向正确 | 所有关节方向匹配 | 目视确认 |

### 传感器层

| # | 检查项 | 通过标准 | 验证方法 |
|---|--------|---------|---------|
| 11 | IMU 静止时加速度读数 $\approx$ (0, 0, 9.81) m/s² | 误差 < 0.1 m/s² | 静态读数 |
| 12 | 关节编码器在 default pose 下读数与配置一致 | 差异 < 0.02 rad | 静态对比 |
| 13 | Obs normalizer (mean/std) 已正确加载 | ONNX metadata 包含或文件存在 | D0 脚本检查 |

### 动力学/接触层

| # | 检查项 | 通过标准 | 验证方法 |
|---|--------|---------|---------|
| 14 | 机器人总质量与仿真差异 < 10% | 称重对比 | 电子秤 |
| 15 | 训练 DR 的摩擦范围覆盖真实地面 | $\mu_{\text{real}} \in$ DR range | SysID 或经验估计 |
| 16 | Sim2Sim 交叉验证 survival time > 90% | MuJoCo 验证通过 | sim2sim 脚本 |

### 安全层

| # | 检查项 | 通过标准 | 验证方法 |
|---|--------|---------|---------|
| 17 | 硬件急停按钮已测试 | 按下后 100ms 内停止 | 主动触发测试 |
| 18 | Watchdog 超时保护已启用 | 通信中断后自动进入安全姿态 | 断开通信测试 |
| 19 | 姿态保护阈值已设置 | pitch/roll > 30° 时锁关节 | 手动倾斜测试 |
| 20 | Action rate limit 已启用 | 相邻帧 action 差异 < 阈值 | 检查 post-processing 代码 |

**使用方法**：打印此表，在每次新的真机测试前逐项勾选。任何一项标记为"不通过"就停止——修复后重新检查。这份清单看起来繁琐，但它能在 30 分钟内完成，而一次因为 joint order 错误导致的机器人摔倒可能需要两天来维修。

---

## 本章小结

| 知识点 | 核心结论 | 关键要素 |
|--------|---------|---------|
| Gap 七层分类 | 按动力学/接触/执行器/传感器/时序/接口/安全穷举 | 先修确定性错误，再修不确定性 |
| 四种方法+ASAP | SysID 对齐均值、DR 覆盖不确定性、FT 修正结构差、Adaptive 在线适应、ASAP 学习残差 | 组合使用，各解决一部分 |
| ONNX 导出边界 | 只包含 actor 计算图和 metadata | 不含 SDK、状态估计、安全 |
| ProtoMotions baked-in | obs 计算图烤入 ONNX，部署端只需原始传感器 | 消除 obs 重实现 bug |
| Sim2Sim 交叉验证 | Isaac Lab ↔ MuJoCo 双引擎验证策略鲁棒性 | survival 差异 < 15% |
| unitree_rl_lab 管线 | Train→Export→Sim2Sim→C++→真机五阶段 | deploy.yaml + joint_mapping |
| 延迟四段分解 | obs/policy/transport/actuator 各有不同影响 | delay/period > 0.1 就需补偿 |
| 安全链路 | 独立于策略的保护层（独立线程/进程） | 硬安全 + 软安全，缺一不可 |
| 分级验收 D0-D6 | 逐步升级风险，失败回退 | 每级有明确通过标准 |
| Perception Gap | 外观/几何/标定/时序四个来源 | 训练侧 DR + 部署侧标定 |
| SysID 实操 | 扫频辨识→参数拟合→交叉验证 | 辨识结果定义 DR 均值和范围 |
| 部署迭代循环 | D0 失败→修接口；D4 失败→修物理；D6 失败→Fine-tune | 每次测试必须有诊断产出 |
| Telemetry | 每帧记录 obs/action/latency/safety_events | 离线复盘定位 gap 层 |
| 20 项 Checklist | 覆盖七层 gap 的部署前检查 | 30 分钟完成 |

本章教授的核心工程方法论可以用一句话总结：**Sim2Real 不是一次性的"从仿真到真机"跳跃，而是一条由 D0-D6 分级验收、七层 Gap 诊断和 Telemetry 驱动的证据链——证据链越完整，真机风险越可控。**

## 章节收束

把本章内容压缩成一句工程判断：**先确认输入语义，再确认训练目标，最后确认部署边界。** 如果三者不一致，策略在仿真里越成功，迁移时越危险。

Sim2Real 不是最后一步——而是一条贯穿训练、导出、播放、部署和复盘的证据链。证据链越完整，真机风险越可控。本章定义了这条证据链的每一环：gap 分类告诉你应该关注什么（§23.1），四种经典方法+ASAP 残差方法告诉你用什么工具（§23.2），ONNX 导出边界和 ProtoMotions baked-in 告诉你策略文件包含什么和不包含什么（§23.3），Sim2Sim 和 unitree_rl_lab 管线告诉你如何在真机前验证（§23.5），延迟补偿告诉你闭环时序如何影响控制（§23.6），安全链路告诉你策略之外需要什么保护（§23.7），分级验收告诉你如何逐步升级风险（§23.8），perception gap 告诉你视觉部署的特殊挑战（§23.9），SysID 告诉你如何对齐仿真均值（§23.10），迭代循环告诉你如何从每次真机测试中获取诊断信息（§23.11），Telemetry 和 Checklist 是工程执行的基础设施（§23.13-14）。

回顾本章的知识树结构：

```
根节点: "Sim2Real = 把多个误差源限制在策略能承受的联合分布内"
│
├─ 方法论分支
│  ├─ §23.1 七层 Gap 分类（诊断框架）
│  ├─ §23.2 五种解决方法（SysID/DR/FT/Adaptive/ASAP）
│  └─ §23.10 SysID 实操（辨识→拟合→验证）
│
├─ 导出与验证分支
│  ├─ §23.3 ONNX 导出边界 + ProtoMotions baked-in obs
│  ├─ §23.4 部署调参参考
│  └─ §23.5 Sim2Sim 交叉验证 + unitree_rl_lab 管线
│
├─ 部署安全分支
│  ├─ §23.6 延迟四段分解 + 补偿
│  ├─ §23.7 安全链路分层（L0-L5）
│  └─ §23.8 分级验收 D0-D6
│
├─ 特殊场景分支
│  └─ §23.9 Perception Gap（视觉部署）
│
└─ 工程实践分支
   ├─ §23.11 部署迭代循环
   ├─ §23.12 工程实战案例
   ├─ §23.13 Telemetry 与日志分析
   └─ §23.14 20 项部署检查清单
```

一个学生读完本章后，面对任何 sim2real 失败场景，应该能够：（1）按七层分类定位 gap 来源，（2）选择正确的解决工具（SysID/DR/ASAP/Fine-tuning/Adaptive），（3）按分级验收流程安全地推进真机测试，（4）从每次测试的 Telemetry 中提取诊断信息反馈到仿真端。这四步能力构成了 sim2real 工程师的核心方法论。

> **下一章预告**：Ch24 将聚焦大规模训练——当你的环境搭建（Ch22）和 sim2real 管线（Ch23）都就绪后，下一个挑战是"如何高效地训练？"多 GPU 并行（mjlab 的 `--gpu-ids` + torchrunx，Isaac Lab 的 torchrun）、NaN 排查（`--enable-nan-guard True`）、性能 profiling（steps/s、VRAM、kernel profiling）、超参搜索——这些都是从"能跑"到"跑得快"的工程优化。AGILE 的四阶段工业级 workflow（Prepare→Train→Evaluate→Deploy）是 Ch24 的核心参考。Ch23 的 Sim2Sim 验证和 D0-D6 分级验收在 AGILE 的 Evaluate 和 Deploy 阶段会被直接复用。

## 累积项目：本章新增模块

在之前的训练和视觉管线上添加部署准备。以下 10 个模块按依赖顺序排列——前面的模块是后面的前置条件：

| # | 模块 | 描述 | 依赖 | 预计工作量 |
|---|------|------|------|-----------|
| 1 | D0 自动检查脚本 | 读取 ONNX metadata，比对训练配置 | §23.3, §23.8 | 2 小时 |
| 2 | ONNX 导出+验证 | 训练完自动导出 + PyTorch/ONNX 一致性检查 | §23.3 | 3 小时 |
| 3 | Sim2Sim 验证脚本 | 在 MuJoCo 中评估策略 + 自动报告 | §23.5, #2 | 4 小时 |
| 4 | 延迟注入训练 | EventsCfg 中加 action/obs delay | §23.6 | 1 小时 |
| 5 | C++ 延迟 Profiler | 控制循环中四段延迟测量 | §23.6 | 3 小时 |
| 6 | 安全监控线程 | 独立线程的 tilt/watchdog/limit 保护 | §23.7 | 4 小时 |
| 7 | SysID 扫频脚本 | 单关节正弦扫频数据采集+分析 | §23.10 | 半天 |
| 8 | Telemetry 记录器 | obs/action/latency/safety 实时记录 | §23.13 | 3 小时 |
| 9 | Sim-Real 日志对比 | 逐维度 obs 差异分析 | §23.13, #8 | 2 小时 |
| 10 | 部署报告模板 | metadata + 延迟 + 安全 + 测试结果 | §23.12, #1-9 | 1 小时 |

**从 Ch23 出发的后续路径**：

| 路径 | 本章贡献 | 后续章节 |
|------|---------|---------|
| 四足 sim2real | Sim2Sim + D0-D6 + Telemetry | Ch24 大规模训练优化 |
| 人形 sim2real | ASAP delta-action + 安全链路 + HoST 跌倒恢复 | Ch24-25 训练诊断 |
| 操作 sim2real | Perception Gap + 外参 DR + 力控标定 | Ch24-25 调参地图 |
| 网球 sim2real | 高速视觉延迟 + 球轨迹预测噪声 | Ch26-28 网球项目 |

## 延伸阅读

| 资料 | 难度 | 说明 |
|------|------|------|
| He et al., "ASAP: Aligning Simulation and Real-World Physics for Learning Agile Humanoid Whole-Body Skills", RSS 2025 (arXiv:2502.01143) | ⭐⭐⭐ | Delta-action 残差模型，最新 sim2real 方法 |
| ProtoMotions "Baked-in Obs" ONNX export (NVlabs) | ⭐⭐⭐ | MdpComponent 三层级导出架构 |
| unitree_rl_lab (Unitree Robotics, GitHub) | ⭐⭐ | Go2/G1/H1 完整五阶段部署管线 |
| unitree_rl_mjlab (Unitree Robotics, GitHub) | ⭐⭐ | mjlab 版本的等价部署管线 |
| Sobanbabu et al., "SPI-Active", arXiv:2505.14266, 2025 | ⭐⭐⭐ | 基于采样的主动探索 SysID |
| Tan et al., "Sim-to-Real: Learning Agile Locomotion", RSS 2018 | ⭐⭐ | SysID + DR 经典实践 |
| Kumar et al., "RMA: Rapid Motor Adaptation", RSS 2021 | ⭐⭐⭐ | Adaptive control + latent variable |
| Hwangbo et al., Science Robotics 2019 | ⭐⭐⭐ | ANYmal actuator network + sim2real |
| Lee et al., Science Robotics 2020 | ⭐⭐⭐ | Teacher-student + terrain adaptation |
| OpenAI, "Solving Rubik's Cube", 2019 | ⭐⭐⭐ | 大规模 DR 极致案例 |
| Di Carlo et al., IO-MPC 2018 | ⭐⭐ | MPC 频率和延迟预算分析 |
| mjlab 源码 `src/mjlab/rl/exporter_utils.py` | ⭐⭐ | ONNX 导出和 metadata 实现 |

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| ONNX 可运行但动作方向反 | joint order 映射错 | 1.读 metadata joint_names 2.与 SDK 逐一比对 3.单关节小幅测试 | 23.1, 23.3 |
| 站立几秒后摔倒 | 延迟未补偿 | 1.测量端到端延迟 2.对比训练 decimation 3.注入延迟重训 | 23.4 |
| 仿真稳定真机打滑 | 摩擦 gap + 延迟耦合 | 1.SysID 摩擦系数 2.检查足底材料 3.扩大 DR friction 范围 | 23.2 |
| 视觉抓取固定偏移 | 相机外参标定差异 | 1.已知点投影验证 2.重新标定 3.训练加外参 DR | 23.7 |
| 安全急停无效 | 急停只在 UI 层 | 1.确认硬件 watchdog 存在 2.主动触发测试 3.与策略进程分离 | 23.5 |
| 移动操作 base-arm 冲突 | 坐标系不统一或时序错 | 1.分离验证 base 和 arm 2.检查 frame 定义 3.加阶段门控 | 23.6 |
| 长时运行后性能衰退 | 电机温度或电池变化 | 1.记录温度电压趋势 2.对比冷热机性能 3.加降额逻辑 | 23.11 |
| SysID 后仍有系统偏差 | 模型结构不足 | 1.交叉验证辨识结果 2.检查非辨识数据误差 3.考虑 Fine-tuning | 23.9 |
| DR 范围太宽不收敛 | 未做 SysID 先做 DR | 1.先辨识关键参数 2.缩窄 DR 范围 3.阶段化增大 | 23.2, 23.9 |
| 真机日志无法复盘 | telemetry 格式不对齐 | 1.确认 schema 与仿真一致 2.检查视频时间戳 3.统一格式 | 23.10 |

### 真机前最终清单 ⭐

本清单是 D0-D6 分级验收的浓缩版——在第一次接触真机之前逐项勾选。

- policy 文件和 normalizer 文件成对保存，hash 可追溯
- obs schema 与真实状态估计字段逐项对应
- action schema 与 SDK 命令逐项对应，单位明确
- 所有 action 都有限幅和 rate limit
- joint order 通过单关节小幅动作验证
- control dt、policy dt 和真实循环频率一致
- latency p95 小于训练或压力测试预算
- watchdog、人工急停、物理急停都已触发测试
- telemetry 能对齐视频、obs、action 和安全事件
- 影子模式输出不越界
- 悬空低增益测试通过
- 平地低速测试通过后才进入任务片段
- 移动操作先单独验证 base 和 arm，再联合
- 视觉策略导出包含相机 metadata 和预处理 schema
- 部署报告记录失败案例，不只记录成功视频
- 任何安全链路未验证时，完整任务保持禁止状态
- 失败报告记录第一触发条件和回退等级
- 每次真机测试后更新部署报告，不遗漏任何诊断信息



## 🔧 故障排查手册（扩展版）

以下手册覆盖了本章所有 14 节涉及的故障场景，按频率排序。

| # | 症状 | 可能原因 | 排查步骤 | 相关节 |
|---|------|---------|---------|--------|
| 1 | ONNX 可运行但动作方向反 | joint order 映射错 | 1.读 metadata joint_names 2.与 SDK 逐一比对 3.单关节小幅测试 | §23.3, §23.8 |
| 2 | 站立几秒后摔倒 | 延迟未补偿 | 1.测量端到端延迟 2.对比训练 decimation 3.注入延迟重训 | §23.6 |
| 3 | 仿真稳定真机打滑 | 摩擦 gap + 延迟耦合 | 1.SysID 摩擦系数 2.检查足底材料 3.扩大 DR friction 范围 | §23.1, §23.10 |
| 4 | 视觉抓取固定偏移 | 相机外参标定差异 | 1.已知点投影验证 2.重新标定 3.训练加外参 DR | §23.9 |
| 5 | 安全急停无效 | 急停只在 UI 层 | 1.确认硬件 watchdog 存在 2.主动触发测试 3.与策略进程分离 | §23.7 |
| 6 | 移动操作 base-arm 冲突 | 坐标系不统一或时序错 | 1.分离验证 base 和 arm 2.检查 frame 定义 3.加阶段门控 | §23.7, §23.12 |
| 7 | 长时运行后性能衰退 | 电机温度或电池变化 | 1.记录温度电压趋势 2.对比冷热机性能 3.加降额逻辑 | §23.8 |
| 8 | SysID 后仍有系统偏差 | 模型结构不足 | 1.交叉验证辨识结果 2.检查非辨识数据误差 3.考虑 ASAP/Fine-tuning | §23.2, §23.10 |
| 9 | DR 范围太宽不收敛 | 未做 SysID 先做 DR | 1.先辨识关键参数 2.缩窄 DR 范围 3.阶段化增大 | §23.2, §23.10 |
| 10 | 真机日志无法复盘 | telemetry 格式不对齐 | 1.确认 schema 与仿真一致 2.检查视频时间戳 3.统一格式 | §23.13 |
| 11 | ONNX 推理结果与 PyTorch 不同 | normalizer 未 baked-in | 1.运行 §23.3 验证代码 2.检查 mean/std 文件 3.重新导出 | §23.3 |
| 12 | Sim2Sim 差异 > 30% | 接触/摩擦模型差异 | 1.对比 zero agent base height 2.调接触参数 3.确认 actuator 一致 | §23.5 |
| 13 | C++ 控制器推理延迟 jitter 大 | ONNX Runtime 未 warmup | 1.添加 100 次 warmup 推理 2.检查 GPU/CPU 绑定 3.固定线程 | §23.6 |
| 14 | deploy.yaml 关节映射不对 | SDK 版本更新改了关节枚举 | 1.检查 SDK 版本 2.重新生成 mapping 3.用单关节测试验证 | §23.5 |
| 15 | ASAP delta 模型使真机更差 | Phase 2 数据不足或不代表 | 1.增加真机 rollout 数据 2.检查 delta 网络输入是否正确 3.减小 delta 学习率 | §23.2 |

### 真机前最终清单（增强版） ⭐

本清单是 §23.14 的 20 项检查的浓缩版——在第一次接触真机之前逐项勾选。

**必须通过（阻断性）**：
- [ ] Policy 文件和 normalizer 文件成对保存，hash 可追溯
- [ ] Obs schema 与真实状态估计字段逐项对应
- [ ] Action schema 与 SDK 命令逐项对应，单位明确
- [ ] 所有 action 都有限幅和 rate limit
- [ ] Joint order 通过单关节小幅动作验证（D3）
- [ ] Control dt、policy dt 和真实循环频率一致
- [ ] Latency p95 小于训练延迟预算
- [ ] Watchdog、人工急停、物理急停都已触发测试
- [ ] Sim2Sim 交叉验证通过（survival > 90%）

**强烈建议（非阻断性）**：
- [ ] Telemetry 能对齐视频、obs、action 和安全事件
- [ ] 影子模式（D2）输出不越界
- [ ] 悬空低增益测试（D3）通过
- [ ] 平地低速测试（D4）通过后才进入任务片段
- [ ] 视觉策略导出包含相机 metadata 和预处理 schema
- [ ] 部署报告记录失败案例，不只记录成功视频
- [ ] 失败报告记录第一触发条件和回退等级
- [ ] 每次真机测试后更新部署报告
- [ ] SysID 的辨识结果在本周内有效（未过期）

> **反事实推理：如果跳过 Sim2Sim 验证直接上真机会怎样？** 你可能在真机上遇到一个问题——策略在第 5 秒摔倒。这时候你有两个假设：(a) 这是 sim-to-real gap 导致的（物理参数不对），需要调 DR 重新训练；(b) 这是策略本身的问题（reward 设计不好），需要修改 reward。如果你事先做了 Sim2Sim，你就知道策略在 MuJoCo 中是否也在第 5 秒摔倒——如果是，问题大概率在策略本身（两个仿真器表现一致说明不是物理引擎特定的问题）。如果 Sim2Sim 中策略稳定但真机不稳定，问题大概率在 sim-to-real gap。Sim2Sim 帮你把一个 2-假设问题缩减为 1-假设问题——排查效率提升一倍。

> **反事实推理：如果不记录 Telemetry 会怎样？** 你做了一次真机测试，机器人走了 8 秒然后摔倒。没有 Telemetry，你只知道"8 秒后摔了"。有 Telemetry，你知道"在第 7.2 秒，左前腿的关节追踪误差突然从 0.02 rad 增大到 0.15 rad，同时 IMU pitch 开始偏移，0.8 秒后 base 姿态越限触发 safety stop"。后者直接告诉你问题在左前腿的 PD 控制——可能是该关节的 Kp 不够或者延迟太大。没有 Telemetry 的测试等于白做。

---

> **下一章预告**：Ch24 将聚焦大规模训练——当你的环境搭建（Ch22）和 sim2real 管线（Ch23）都就绪后，下一个挑战是"如何高效地训练？"多 GPU 并行、NaN 排查、性能 profiling、超参搜索——这些都是从"能跑"到"跑得快"的工程优化。AGILE 的四阶段工业级 workflow（Prepare→Train→Evaluate→Deploy）是 Ch24 的核心参考。Ch23 的 Sim2Sim 验证和 D0-D6 分级验收在 AGILE 的 Evaluate 阶段会被直接复用。


### Sim2Real 排查的"五分钟法则" ⭐

当真机测试失败时，按以下优先级花 5 分钟做快速定位，然后再决定下一步行动：

**第 1 分钟：检查 ONNX 输出**。在部署端记录策略的原始输出（未限幅前）。如果输出全是 NaN 或固定值——大概率是 obs normalizer 问题。如果输出全在 $\pm$1 范围内但行为错误——大概率是 joint mapping 或 action scale 问题。

**第 2 分钟：检查 obs 数值**。打印一帧真实 obs 的各维度数值。和仿真中的 obs 对比——如果某些维度差异超过一个数量级，说明该维度的传感器读取或 frame 转换有问题。

**第 3 分钟：检查延迟**。看 Telemetry 中的 `latency_total_ms`。如果 p95 > 训练 delay budget，延迟是首要嫌疑——在仿真中注入同等延迟看策略是否还能工作。

**第 4 分钟：检查安全事件**。Telemetry 中有没有 `tilt_warning` 或 `joint_limit` 事件？这些事件往往发生在失败前 0.5-1 秒，指示了失败的根因。

**第 5 分钟：对比 Sim2Sim**。策略在 MuJoCo 验证中的 survival time 是多少？如果 Sim2Sim 也失败，问题在策略本身；如果 Sim2Sim 通过但真机失败，问题在 sim-to-real gap 的某个具体层。

这 5 分钟的快速定位通常能把问题缩小到七层分类中的 1-2 层——之后的修复就有了明确的方向。如果 5 分钟定位不了，说明 Telemetry 记录不够详细——回到 §23.13 加强日志记录后再测试。

> **跨领域类比**：这个五分钟法则就像急诊医生的"ABCDE"评估流程——Airway, Breathing, Circulation, Disability, Exposure。不是说每个病人只花 5 分钟，而是用 5 分钟快速判断"最紧急的问题在哪里"，然后集中资源处理那个问题。Sim2real 排查也是一样——不是 5 分钟就能修好，而是 5 分钟就能知道该往哪个方向使力。


### 从 Ch23 到后续章节的知识传递

本章建立的 sim2real 方法论不是"一次性使用"的——它是后续所有涉及真机部署的章节的基础：

| 后续章节 | 复用本章内容 | 特殊扩展 |
|---------|------------|---------|
| Ch24 大规模训练 | D0-D1 验证嵌入 CI | NaN 排查与 GPU 调试 |
| Ch25 训练诊断 | Telemetry 分析方法 | WandB 联合阅读 |
| Ch26-28 网球项目 | 完整 D0-D6 流程 | 高速视觉 + 球轨迹预测噪声 |

本章的 20 项 Checklist（§23.14）和五分钟排查法则是可以直接打印贴在实验室墙上的工程工具——它们的价值不在于"第一次读时的理解"，而在于"每次真机测试前的执行"。


---

> **全书定位**：Ch23 是 Part V（复合形态与自定义机器人）的最后一章，也是 Part I-V 中所有训练技能的最终验证——把仿真中学到的策略成功部署到真实机器人上。从 Ch24 开始进入 Part VI（大规模训练与调试），关注的不再是能不能跑，而是怎么跑得更快、更稳、更可复现。




