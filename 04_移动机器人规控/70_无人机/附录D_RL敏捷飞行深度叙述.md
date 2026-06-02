## 附录D：RL敏捷飞行深度叙述

> → 对应主大纲章节：D8-D9（敏捷飞行平台与 RL sim-to-real）

### 从零到冠军：四旋翼强化学习与 Sim-to-Real 全景地图（2017–2025+）

**四旋翼强化学习已经完成了从"勉强悬停"到"击败世界冠军"再到"2084 参数的跨构型基础策略"的十年跃迁。**核心驱动力并非算法创新（主流仍是 PPO 和 TD3），而是三件工程性事实：**CTBR 动作空间吸收了最难建模的电机动力学、非对称 actor-critic + 选择性域随机化让模拟器匹配硬件、以及大规模 GPU 并行模拟把训练时间压缩到分钟甚至秒级**。对已有足式机器人 RL 背景的读者而言，这意味着：你熟悉的"PPO + Isaac Gym + 教师-学生蒸馏"工具链几乎可以直接迁移，但必须把直觉从"12 维关节位置 + 接触动力学"切换到"4 维 CTBR + 连续气动"，且失败成本从"摔一跤"变成"炸机损毁"。本报告按年代分层梳理关键论文、仿真器与开源实现，最后与足式 RL 做系统对照。

### 早期阶段（2017–2020）：把控制器整体换成神经网络

这段时期的核心问题是：**四旋翼开环不稳定、姿态时间常数在几十毫秒量级、任何未建模效应（电机延迟、电池衰减、blade flapping）都会让 sim-trained 策略发散**。三篇奠基作确立了问题边界。

**Hwangbo 等 (RA-L 2017)** 是第一次证明单一神经网络可以直接输出 4 个单桨推力（SRT 动作空间）替代传统级联控制器，并在 Ascending Technologies Hummingbird 上成功从"5 m/s 倒置抛出"状态恢复悬停。观测为 18 维完整状态（位置、旋转矩阵、线/角速度），推理只需 7 µs，采用作者自研的基于 SVD 的自然梯度方法（DDPG 当时无法收敛）。该文**没有正式域随机化**，训练时加 PD 稳定辅助以便探索，推理时移除。这篇的意义是证明"端到端 RL 飞行可行"。

**Molchanov 等 (IROS 2019) "Sim-to-(Multi)-Real"** 第一次做到单一低层策略零样本迁移到**多个异构 Crazyflie 级别四旋翼**。关键配方是质量、惯量、推重比、电机时间常数、传感器噪声的域随机化 + 延迟建模 + 谨慎的奖励塑形，训练算法基于 PPO（garage 框架）。开源仓库 `amolchanov86/quad_sim2multireal` 及衍生的 `Zhehui-Huang/quad-swarm-rl`（QuadSwarm 模拟器）都在百星量级，是后续多机群 RL 工作的祖父级代码。

**Loquercio、Kaufmann 等的 "Deep Drone Racing" (CoRL 2018, T-RO 2020)** 严格说不是 RL 而是模仿学习，但它确立了 UZH RPG 的**模块化模式**——CNN 从单目 RGB 预测图像空间下一路径点，后端用经典规划+控制跟踪。通过纹理/光照/门几何域随机化完成零样本迁移。GitHub `uzh-rpg/sim2real_drone_racing` 约 90 星。这套"让学习只负责最难泛化的部分（感知），控制仍然经典"的思想，直接传承到后来的 Swift。

**为什么早期这么难？**（1）高频不稳定动力学让小误差指数放大；（2）DDPG/TRPO 在欠驱动系统上样本效率极差；（3）各家自己写模拟器（RAI、gym_art、custom Gazebo），无标准基准；（4）机载 CNN + 高速 IMU 的工程栈本身就是研究课题。对比足式机器人同期的 Hwangbo 2019 ANYmal 工作，后者通过**actuator network** 拟合真实 SEA 电机的力矩响应一举解决 sim-to-real，而四旋翼当时还没人找到类似的"统一修补点"。

### UZH RPG 竞速线（2020–2023）：从森林穿行到击败人类冠军

这四年 UZH RPG 的工作构成一条严密的演进链，每一步都解决前一步留下的最大 bug。

**Loquercio 等 "Learning High-Speed Flight in the Wild" (Science Robotics 2021)** 用 Flightmare 模拟器 + 特权模仿学习训练出能在森林、雪地、废墟中以 10 m/s 穿梭的策略。输入为 $224 \times 224$ 深度图 + IMU + 相对目标，输出是多假设的短时多项式轨迹，由经典 CTBR 控制器跟踪。核心是**"learning by cheating"**：特权 teacher 看到完整地图计算最优轨迹，student CNN 只从机载观测模仿。仓库 `uzh-rpg/agile_autonomy` 约 700+ 星。该文把"感知噪声建模"而非"视觉域随机化"确立为视觉策略 sim-to-real 的新范式。

**Kaufmann、Bauersfeld、Scaramuzza 的 ICRA 2022 基准论文**是整条线最关键的工程发现——它系统比较了 **LV（线速度+偏航率）、CTBR（总推力+机体角速度）、SRT（单桨推力）**三种动作空间。关键结论：在**名义模型**下三者性能相当，但一旦引入质量/惯量/气动扰动，**SRT 急剧崩溃、CTBR 保持稳定、LV 安全但敏捷性差**。原因是 CTBR 把电机时间常数、ESC 延迟、电池电压、blade flapping 等最难建模的高频动力学**外包给机载 2–8 kHz 运行的 PID rate 控制器**，留给策略的是慢得多的刚体姿态+推力问题。这解释了为什么此后几乎所有高性能四旋翼 RL（Swift、SimpleFlight、DATT、SOUS VIDE）都采用 CTBR。策略结构本身很简单：H=10 的状态窗口 + R=10 的参考窗口，分别经 3 层 FC(64) 编码后拼接进 $2 \times 128$ MLP；PPO 50 并行 agent，$\gamma=0.98$。该文还首次引入**非对称 actor-critic**——critic 看到质量、惯量、气动扰动等特权信息，actor 看不到——此后成为 UZH 线所有工作的固定结构。

**Song、Romero、Müller、Koltun、Scaramuzza (Science Robotics 2023) "Reaching the Limit in Autonomous Racing"** 首次把 RL 和时最优 MPC 放在同一赛道做 head-to-head 比较，RL 达到峰值加速度 >12 g、峰值速度 ~108 km/h，刷新纪录。作者的解释是**"RL 的优势不是把同一目标优化得更好，而是优化了一个更好的目标"**——MPC 必须先规划参考轨迹再跟踪，这一中间表示限制了行为空间；RL 直接端到端优化赛圈时间，可以榨干执行器边界并利用未建模效应。训练仅需在单工作站上几分钟。

**Kaufmann 等 "Champion-level Drone Racing Using Deep RL" (Nature 2023) Swift** 是这条线的终极成果——**首个仅靠机载感知就击败 FPV 无人机世界冠军的自主系统**，对 Alex Vanover 5:4、对 Thomas Bitmatta 4:3、对 Marvin Schaepper 6:3，最快圈速比最佳人类快约 0.5 秒。完整管线值得逐层拆解，因为它几乎把此前所有教训都融进去了：

- **模拟器**：TensorFlow Agents 实现 6-DoF 刚体 + 一阶电机动力学（Betaflight PID + BLHeli ESC，误差 <1%）+ Bauersfeld 灰盒多项式气动模型（2021 RSS NeuroBEM 谱系）+ 电池电压-功率模型。
- **观测**（31 维）：当前状态估计（位置 3 + 速度 3 + 旋转矩阵 9）+ 下一个门的 4 个角点 3D 相对坐标（12）+ 上一动作（4）。
- **动作**（4 维）：质量归一化总推力 + 3 轴机体角速度，即 CTBR，由 Betaflight + BLHeli 转为 PWM → 电机转速。
- **网络**：policy 和 value 均为 **2 层 128 神经元 MLP + LeakyReLU (0.2)**。Critic 额外看到仿真器的精确状态（非对称 AC）。
- **奖励**：`r = r_prog + r_perc + r_cmd − r_crash`。`r_prog` 沿向下一个门的进度；`r_perc = λ₂ exp(λ₃·δ_cam⁴)` 奖励把相机光轴对准下一个门——**这一项是 Swift 能脱离 mocap 的关键**，因为它让 VIO 和门检测保持工作。
- **训练**：PPO，100 并行 agent，每 episode 1500 步，Adam LR 3e-4，总 **$1 \times 10^8$ 环境步、在 i9-12900K + RTX 3090 上约 50 分钟**。残差微调再加 $2 \times 10^7$ 步。
- **感知**：CNN 在 30 Hz 灰度图上分割门的 4 个角点，经 PnP 反投影得到门 3D 位姿。
- **状态估计**：VIO（Intel RealSense T265 立体+IMU）+ 门位姿用 Kalman 滤波融合。
- **残差模型（sim-to-real 的真正秘密）**：采集**仅 ~50 秒真实飞行（3 次完整赛圈，~800–1000 样本）**，训练九路 1-D GP（RBF 核混合）建模 VIO 观测残差的随机分布，加上 k=5 的 kNN 回归建模动力学加速度残差（作者发现感知残差随机、动力学残差确定性）。策略在这个**残差增强的模拟器**中做少量微调。这是用"数据驱动的 real2sim2real"**替代传统粗暴的域随机化**。
- **部署**：全部机载，Jetson 级计算单元，**感知-运动延迟 40 ms（人类 ~220 ms）**。

**Swift 源码未公开**（Nature 论文明确说明以防误用），但周边基础设施开源：`uzh-rpg/agilicious` 约 **523 星**、`uzh-rpg/flightmare` 约 **1.3–1.4k 星**。

Swift 的成功可以压缩成一句话：**早期工作试图让策略对 sim-to-real 差距"鲁棒"，Swift 反过来让模拟器匹配真实（CTBR 抽象 + BEM 气动 + 电池模型 + 学到的残差），使 RL 问题退化为一个干净的仿真优化问题。**

### Sim-to-Real 的核心技术与 SimpleFlight 菜单

四旋翼 sim-to-real 比地面/足式机器人更棘手，因为系统**开环不稳定**、小型机（Crazyflie）**推重比只有 1.8** 几乎没有裕量、电池电压衰减直接决定可用推力、无线链路延迟 20–60 ms、**没有地面接触可以"纠偏"**——必须靠控制权限维持姿态。典型需要随机化的参数包括：质量 $\pm$10–30%、惯量对角元 $\pm$20–50%、推力系数 $k_f$、转矩系数 $k_m$、电机时间常数 $T_m$（典型 15–40 ms，Crazyflie 标定约 25 ms）、推重比（Crazyflie 1.7–2.5；竞速机 4–6）、线性/二次体阻尼、桨诱导阻力、地效、IMU 偏置噪声、状态估计延迟（10–30 ms）、指令-推力延迟（约 20 ms）。系统辨识方面，**叶素动量（BEM）模型** 由 UZH Bauersfeld 的 NeuroBEM（RSS 2021）定义了高保真上限，但因速度原因通常只用于评估；训练内部通常用一阶推力曲线 $k_f \cdot \Omega^2$ + 一阶电机滞后，配合拉力台静态测量。

**SimpleFlight (Chen, Yu 等, arXiv 2412.11764, RA-L 2025)** 是上海交大/清华 NICS-EFC 的工作，代码 `thu-uav/SimpleFlight` + `thu-uav/crazyswarm_SimpleFlight`（构建于 `btx0424/OmniDrones` $\approx$ 495 星之上），两仓库合计百星量级。它没有算法创新——就是朴素 PPO + MLP——但**系统地消融并归纳出"五个决定性因子"**，目前已成为新工作复现的基线菜单：

1. **Actor 输入 = 速度 + 旋转矩阵**（加未来 10 个参考点，间隔 50 ms）。用 9 维旋转矩阵而非四元数或欧拉角避免奇异性与双覆盖。
2. **Critic 输入 = Actor 输入 + 时间向量 f_t**（特权信息）。给 critic 一个"episode 进度感"极大改善价值估计。
3. **动作差分平滑奖励** `r_smooth = −‖u_t − u_{t-1}‖₂`。惩罚 CTBR 突变但不限制峰值——比直接 L2 惩罚动作幅度好，不牺牲敏捷性。
4. **SysID + 选择性域随机化**。先精确标定 m、I、k_f、T_m，然后只在**难以精确辨识的参数**（电机常数、推力不对称、延迟）上做随机化。盲目 DR 会过度正则化反而损害精度。
5. **大批量 PPO**。由 OmniDrones 的 GPU 并行（4096+ env）支持，显著稳定策略梯度。

Crazyflie 2.1 标定值：`m=0.0321 kg, Ixx=Iyy=1.4e-5, Izz=2.17e-5, k_f=2.35e-8, k_m=7.24e-10, T_m=0.025 s, Ω_max=2315 rad/s`。结果：在所有基准轨迹（包括尖锐 Z 字形"不可行"轨迹）上**跟踪误差比 Learning-to-Fly、DATT 等 SOTA 低 50%+**，且能从 Crazyflie 零样本迁移到更大自制四旋翼。SimpleFlight 的真正贡献是把一堆分散的工程窍门做成可复现的 recipe。

### 仿真器生态：速度-保真度的空间划分

主流四旋翼 RL 模拟器的定位可以按"物理保真度 $\times$ 渲染能力 $\times$ 并行度"三轴划分：

| 模拟器 | GitHub | 星数(2025) | 后端 | 物理保真度 | 渲染 | GPU 并行 | 最适场景 |
|---|---|---|---|---|---|---|---|
| Flightmare | uzh-rpg/flightmare | ~1.3k | Unity + C++ | 6-DoF + 可选 BEM | Unity 光真 | 部分（百级） | 视觉竞速、VIO |
| Aerial Gym Simulator | ntnu-arl/aerial_gym_simulator | ~691 | Isaac Gym | 6-DoF + 几何控制 | GPU 光线投射深度/分割 | 千级并行 | 杂乱场景导航 |
| gym-pybullet-drones | utiasDSL/gym-pybullet-drones | ~1.6–2.0k | PyBullet | 多模式 6-DoF | 低保真 OpenGL | 仅 CPU | 原型、多机群、Crazyflie SITL |
| AirSim | microsoft/AirSim | ~18k（2025-05 已归档） | Unreal | 6-DoF + simple_flight | UE 光真 | 单机/实例 | 感知数据、PX4 HITL |
| OmniDrones | btx0424/OmniDrones | ~495 | Isaac Sim | 6-DoF + 多机型 | Isaac Sim | 千级 | 高并行 RL、SimpleFlight 基础 |
| FiGS / SOUS VIDE | StanfordMSL/SousVide | 低几十 | 自研 10-DoF + 3DGS | 简化 | **3D 高斯溅射 130 fps** | 单 GPU | 真实场景视觉导航 |
| Learning-to-Fly | arplaboratory/learning-to-fly | 558 | 纯 C++ 头文件 | 最简 6-DoF | 无 | CPU 极快 | 秒级训练 |
| Pegasus Simulator | PegasusSimulator/PegasusSimulator | 百级 | Isaac Sim | PX4/ArduPilot 集成 | Isaac Sim | 受限 | PX4 研究 HITL |

**选择原则**：状态策略首选 OmniDrones/Aerial Gym（千级并行）；视觉策略若需真实场景首选 FiGS，若需通用首选 AirSim 或 Flightmare；对 Crazyflie SITL 首选 gym-pybullet-drones 或 CrazySim。一个经常被忽视的关键点是**"仿真侧低层控制器要和固件里的一致"**——SimpleFlight 特别强调了这一点，它让 sim 内部 PID rate 结构与 Crazyflie PIDrate 严格对齐。

**为什么四旋翼 sim-to-real 比地面机器人难**：（1）开环不稳定，几十毫秒不控就发散；（2）推重比紧，小误差直接打爆控制权限；（3）气动效应（桨涡、induced drag、地效、桨间干涉）是一阶效应且难建模；（4）电池电压随飞行直接按比例改变推力；（5）无线链路延迟 20–60 ms；（6）无地面接触作为"错误校正锚点"，只能靠控制权限维持；（7）高速下相机运动模糊、卷帘快门、光照剧变直接恶化 VIO——Swift 的 GP 残差观测模型就是为此而生。

### 秒级训练与微控制器部署：RLtools 路线

**Eschmann、Albani、Loianno 的 "Learning to Fly in Seconds" (RA-L 2024)** 在 2020 MacBook Pro M1 上**18 秒训练完成 Crazyflie RPM 级控制策略**，迁移到真实 Crazyflie 2.1 上 10/10 种子全部成功起飞。仓库 `arplaboratory/learning-to-fly` 558 星，底层 `rl-tools/rl-tools` 942 星。秒级训练的组合拳包括：

- **TD3（off-policy）** 而非 PPO，样本复杂度仅 ~30 万步。
- **非对称 actor-critic**：actor 看 $18 + 4 \cdot N_H$ 维（不含电机 RPM，含动作历史补偿部分可观测性）；critic 看 28 维含电机 RPM、随机力/力矩扰动等特权信息。
- **课程学习**：奖励权重每 10 万步指数增强，探索噪声同步衰减，回放缓冲奖励重算。消融显示课程 + AAC 是最大贡献项。
- **极致优化的 C++ 头文件模拟器**：Nvidia T2000 笔记本 GPU 达到 **12.84 亿步/秒**（8192 并行 env $\times$ 100 万步 $\times$ 6380 ms），**比 Flightmare 快约 6420$\times$**。
- **RLtools 纯 C++17 模板元编程库**：无动态分配、可编译进 STM32F405（Crazyflie）、ESP32、PX4 固件。策略直接作为 `actor_xxx.h` 头文件烘焙进固件。

动作空间是**直接 RPM**（4 维）——这是作者定义的"5 级分类"里最底层的第 5.1 级。电机建模为 $\tau=0.15$ s 的一阶低通，延迟 5–25 控制步（100 Hz）。奖励是状态跟踪、姿态、速度、角速度、动作差分的加权二次型加生存奖励。策略规模仅数千参数。

**RAPTOR (Eschmann 等, arXiv 2509.11481, 2025) 基础策略**把这条路线推到极致——**2084 参数、3 层 GRU（隐藏维度 16）的单一策略零样本控制 32 g 到 2.4 kg、推重比 1.75 到 12 之间的 10 种异构四旋翼**（brushed/brushless、2/3/4 叶桨、PX4/Betaflight/Crazyflie/M5StampFly 多种飞控），在目标 MCU 上 CPU 占用 <10%，训练 100 Hz 推理最高 500 Hz。训练配方是**两阶段元模仿学习**：阶段一从可分解参数分布祖先采样 **1000 架不同四旋翼**，每架用 TD3 + 特权观测（包含电机转速）训练专用 teacher；阶段二把 2084 参数 GRU student 用 MSE/KL 对 1000 个 teacher 做在线模仿，让 GRU 隐状态隐式学到系统辨识。线性探针证明 GRU 隐状态能预测未见推重比（$R^2=0.95$）。跟踪误差 RMSE 与专用 per-platform 控制器的 $R^2=0.95$。仓库 `rl-tools/raptor` 170 星，`pip install foundation-policy`。观测 22 维（$p+R+v+\omega+a_{t-1}$），动作 4 维归一化电机指令。

**与操作类基础模型对比**（Octo 93M 参数、RT-2 55B、OpenVLA 7B）：RAPTOR 仅 2084 参数、纯 GRU、纯低维本体感知、MCU 部署、100–500 Hz 硬实时、跨**动力学维度**（质量/几何/推力曲线）而非**语义维度**。RAPTOR 本质是"通过循环网络做隐式在线系统辨识"的低维连续控制基础策略，与 VLA 的多模态语义泛化是两个完全不同的问题类。

**部署栈对比**：Swift 在 Jetson TX2 上 CPU 评估策略（8 ms/次）+ GPU 跑门检测 CNN；SimpleFlight 是 host PC 上 PyTorch `deploy.pt` 通过 Crazyswarm2 radio 50–100 Hz 发 CTBR；RLtools/L2F/RAPTOR 则是纯 C++ header 烘焙进 STM32 固件跑直接 RPM 控制。MCU 级别不用 ONNX/TensorRT——权重就是 `constexpr` 数组，零堆、零框架依赖。

### 自适应 RL 与 MPC+RL 混合：理论保证 vs 端到端

当 sim-to-real 差距或外部扰动（风、载荷）无法仅靠 DR 消化时，出现两条路线。

**DATT (Huang, Rana, Spitzer, Shi, Boots, CoRL 2023 Oral)** 把 **$L_1$ 自适应控制与 RL 前馈组合**。训练时策略在仿真里条件于注入的 ground-truth 扰动 d̂；部署时把 ground truth 换成**实时运行的经典 $L_1$ 估计器**输出。策略收到（体坐标系误差、10 步未来参考点、d̂）输出 CTBR。推理 <3.2 ms、仅 L1 的 PPO 25M 步训练。在不可行轨迹（Z 字形、五角星）+ 风扰 + 纸板阻力板下，**误差比自适应 NMPC 低 34–54% 且计算量 ¼**。仓库 `KevinHuang8/DATT` 86 星。思路：RL 负责长时程轨迹级推理，$L_1$ 负责毫秒级扰动补偿，两者并行。

**Neural-Fly (O'Connell, Shi 等, Science Robotics 2022, Caltech Chung+Yue 组)** 走另一条极端——**完全不用 RL**。它把气动残差分解为 `f(x,w) ≈ φ(x)·a(w)`，其中 $\phi$ 是**风无关的共享神经基**、a(w) 是低维**风特定线性系数**。$\phi$ 通过 **DAIML（领域对抗不变元学习）** 离线从 12 分钟真实飞行数据（六种风速 0–22 km/h、36000 样本）训练；部署时只对 a 做 Kalman 风格的**在线最小二乘更新**，速率等于控制环带宽，毫秒级收敛。控制器本体是经典 SE(3) 几何跟踪，带 Lyapunov/contraction 稳定性证明。在 43.6 km/h 风下跟踪 RMS 约为 $L_1$/INDI 的 1/2。仓库 `aerorobotics/neural-fly` 211 星。它和纯 RL 的根本区别：**只学物理模型学不到的残差，保留所有经典控制的稳定性保证**。

**SOUS VIDE / FiGS (Low, Adang, Yu, Nagami, Schwager, Stanford MSL, arXiv 2412.16346, 2024–2025)** 代表第三条思路——**MPC 专家蒸馏成视觉学生**。FiGS（Flying in Gaussian Splats）用 2–3 分钟手持视频 + 一个 ArUco 标记通过 Nerfstudio/gsplat 重建场景，配 10 维简化动力学，**单 GPU 130 fps 光真渲染**。特权 MPC（acados 求解）在 FiGS 内对随机化动力学生成 10 万–30 万演示；SV-Net（含 RMA 式历史编码）用行为克隆蒸馏出 20 Hz 机载视觉策略（Orin Nano + PixRacer Pro）。实测在 $\pm$30% 质量、风扰、$\pm$60% 亮度、动态物体下鲁棒，105 次真实飞行。仓库 `StanfordMSL/SousVide` 数十星。

**MPC+RL 混合谱系**覆盖四种范式：(1) MPC 专家 → BC/RL 学生（GPS, SOUS VIDE, Tagliabue 2022）；(2) RL 学到的动力学放进 MPC（**Salzmann 等 "Real-time Neural-MPC" RA-L 2023**，`TUM-AAS/neural-mpc` 261 星——用 CasADi 兼容 autodiff 让 ResNet-18 级别模型能在 50 Hz MPC 里运行，跟踪误差降 82%）；(3) RL + MPC 安全滤波（Wabersich & Zeilinger PSF, Automatica 2021；Brunke 多步 MPSF）；(4) 可微 MPC 作为 actor 层（Amos 等 NeurIPS 2018；**Romero 等 "Actor-Critic MPC" T-RO 2025**, `uzh-rpg/acmpc_public`，21 m/s 真实竞速）。

**DATT vs Neural-Fly vs 纯 RL 对照**：Neural-Fly 适应最快（毫秒级，只更 3–12 维 a）、样本效率最高（12 分钟真实数据）、有稳定性保证，但限于可行平滑轨迹；DATT 能处理不可行轨迹+扰动，但需 25M 仿真步训练；纯 RL（Swift/SimpleFlight）任务性能最强但适应性最差且无保证。SimpleFlight 2024 的基准直接跑了 DATT 发布 checkpoint 对比，声称在不可行轨迹上误差降低 >50%——这说明**纯 RL 在精心 SysID+选择性 DR 下正在追赶混合方法**，是否需要混合方法正在重新评估。

### 足式 RL 与四旋翼 RL 的系统对照

对已有足式 RL 经验的读者，下表概括迁移时真正需要重建的直觉。

| 维度 | 足式 RL（四足/人形） | 四旋翼 RL |
|---|---|---|
| 动作空间 | 12 维（四足）/19–29 维（人形）关节位置目标，绕中性姿态 $\pm$0.25 rad，送给 200 Hz–1 kHz PD | **4 维** CTBR 或 4 维 RPM/PWM；维度低一个数量级 |
| 内环控制 | 策略 50 Hz；关节驱动器 PD 200 Hz–1 kHz | 策略 100–500 Hz；机体角速度 PID 1–8 kHz（Betaflight/PX4）；**内环余量更小** |
| 观测 | 关节位置/速度 + IMU + 投影重力 + 上动作 + 指令，全部**直接可测** | 姿态+角速度直测，但**位置/速度需 VIO 或动捕**——外部估计本身是噪声源 |
| 动力学 | **混合/不连续**：接触 make-break、Coulomb 摩擦、冲击脉冲 | **平滑连续**：气动 + 刚体旋转，单浮体无接触 |
| 仿真器 | Isaac Gym/Isaac Lab/MuJoCo MJX，~4k env/GPU | Isaac Lab/Aerial Gym/OmniDrones/Flightmare，**10k+ env/GPU 更容易**（无接触求解） |
| Sim-to-real 主要间隙 | 驱动器动力学（SEA 柔顺、齿轮摩擦）、地面摩擦几何、通信延迟 | 电机推力曲线+启动延迟、电池电压衰减、未建模气动、IMU 振动噪声、VIO 延迟 |
| 典型训练 | legged_gym：walking ~20 分钟；Extreme Parkour ~20 小时（3090） | SimpleFlight 分钟级；L2F **18 秒**；RAPTOR 元学习小时级 |
| 策略规模 | $3 \times 512$ MLP，100k–several M 参数 | $2 \times 128$ MLP（Swift）；L2F 数千参数；**RAPTOR 2084 参数** |
| 机载算力 | Jetson Orin / x86，50 Hz 轻松 | 从 Jetson 到 **STM32F405 (168 MHz Cortex-M4)** 全覆盖，硬实时要求严 |
| 主流算法 | PPO 近乎唯一 | PPO（UZH/SimpleFlight）+ TD3（RLtools/L2F/RAPTOR）并存 |
| 部署运行时 | LibTorch / ONNX Runtime + ROS 2（rl_sar ~1.2k 星、unitree_rl_lab ~937 星） | 多样：host-PC PyTorch over radio（SimpleFlight）、Jetson TensorRT（Swift）、**纯 C++ header 烘焙固件（RLtools）** |
| 失败后果 | 摔倒通常可恢复，体操垫+保护绳足够 | **撞机=机体损毁**，桨、机架、电调、电池刺穿都常见——实验设计被安全支配 |
| 适应机制主流 | **RMA**（Kumar RSS 2021）+ 教师-学生蒸馏 | RMA 变体 / $L_1$（DATT）/ 元学习残差（Neural-Fly）/ Swift 残差 GP |
| 代表奖励项 | 线/角速度跟踪 + 足腾空时间 + 对称/步态 + 平滑 + 存活 + 碰撞 | 位置/速度跟踪 + 姿态正则 + 动作差分 + 存活 + **感知感知（Swift 的 r_perc）** + 崩溃惩罚 |

**三条可直接迁移的经验**：(1) **非对称 actor-critic** 在两个领域都是核心——critic 看特权（足式看地形摩擦质量、四旋翼看风/真实速度/推力残差），actor 看部署时可得观测；(2) **教师-学生蒸馏**同样适用——足式是 scandots→proprioception（Miki 2022），四旋翼是 1000 teacher→GRU（RAPTOR），甚至 MPC→BC（SOUS VIDE）；(3) Isaac Gym/Lab 的大规模并行 PPO 直接能用。

**三条必须重建的直觉**：(1) **动作维度从 12/19 变成 4**——你熟悉的高维关节空间的奖励塑形（足腾空、对称、步态频率）全部不适用，四旋翼奖励空间简单得多，**感知感知项和存活项是新增的关键**；(2) **从离散接触到连续气动**——不再有"触地瞬间"这种事件，取而代之的是随速度连续变化的阻力/桨涡/地效，BEM 模型的精度决定极限性能；(3) **安全成本从 O(1) 跳到 O(100)**——足式你可以大胆地让策略在真机上探索，四旋翼必须先在高保真 BEM 模拟器里验证、再在 tethered/室内笼子里验证、最后才是自由飞行；因此**"off-policy 算法 + 少量真机数据 + 残差微调"（Swift 模式）在四旋翼上比在足式上更有吸引力**。

### 结论：工程胜过算法，收敛中的下一步

十年的四旋翼 RL 史几乎没有算法突破——**PPO 和 TD3 仍是两大主力**。真正改变游戏的是三件事：CTBR 动作空间把最难建模的动力学外包给固件 PID（Kaufmann 2022）、非对称 actor-critic 让 critic 用特权信息而 actor 用部署观测（从 Kaufmann 2022 贯穿到 Swift 和 RAPTOR）、大规模 GPU 并行模拟器（Isaac Gym 系 + Aerial Gym + OmniDrones + RLtools C++ 引擎）把训练时间从周压到秒。Sim-to-real 则从"粗暴域随机化"演化到"精确 SysID + 选择性 DR + 数据驱动残差模型"（SimpleFlight 五因子 + Swift GP/kNN）。

**对已有足式 RL 背景的入门者**，最高效的学习路径是：先读 Kaufmann 2022 ICRA 基准论文建立 CTBR 直觉；然后读 SimpleFlight（2412.11764）拿到可复现的五因子 recipe；再读 Swift（Nature 2023）看满血管线；想秒级训练就跑 `arplaboratory/learning-to-fly`；想跨平台泛化就看 RAPTOR。工具链侧，OmniDrones + Isaac Lab 是当前状态策略的最佳起点，FiGS 是视觉策略的新锐选择，Flightmare 仍适合经典竞速研究。

**下一个突破方向**，证据指向三处：(1) **基础策略从低维 proprioception 扩展到带视觉**——把 RAPTOR 的 GRU 隐式系统辨识与 SOUS VIDE 的 GSplat 视觉学生融合；(2) **纯 RL 追平并超过 MPC+RL 混合**——SimpleFlight 在不可行轨迹上已经击败 DATT 和 MPC，若加入自适应机制可能让 $L_1$/Neural-Fly 的优势面收窄；(3) **MCU 级别的在线学习**——RLtools 的 iOS/Teensy 训练能力预示了真机上持续微调的可能，这在足式机器人上仍未实现。如果这三条都兑现，四旋翼 RL 将在 2026–2027 年进入"单一小模型泛化所有构型、机载持续学习、视觉+动力学统一基础策略"的新阶段，**反过来给足式 RL 提供范式参考——尤其是它"让模拟器匹配真实"而非"让策略对抗差距"的根本方法论。**