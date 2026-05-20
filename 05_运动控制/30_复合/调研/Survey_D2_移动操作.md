## D2 移动操作：VLA 模型已取代 MPC 栈

### 开源生态系统 (2024-2026)

D2 是 **基础模型已取代经典模型堆栈**、成为前沿研究主导方向的子领域。最活跃的代码库包括：**`Physical-Intelligence/openpi`**（Apache-2.0 许可，2025 年前快速增长；提供 π0、π0-FAST 和 π0.5 检查点，2025 年 9 月新增 PyTorch 支持）； **`huggingface/lerobot`**（数万个星标，Apache-2.0；托管 π0 / π0-FAST 移植版本及 LeRobot 原生训练）； **`MarkFzp/mobile-aloha`** 和 **`tonyzhaozh/aloha`**（MIT，双臂移动操作的参考模仿学习栈，孕育了整个ALOHA生态系统）；**`facebookresearch/home-robot`**及其后续版本 **`stretch-ai`**（Meta + Hello Robot； 麻省理工学院；适用于 Stretch 类机器人的开放式基线，集成了 SLAM+抓取+VLM）；**`hello-robot/stretch_ros2`**（BSD-3，生产级）；**`leggedrobotics/ocs2`** `mobile_manipulator` 模块（适用于 Panda-on-Ridgeback 风格配置的标准 SQP+HPIPM NMPC）； **`moveit/moveit2`** 带伺服电机和任务构造器；**`ros-navigation/navigation2`** (Apache-2.0)；**`real-stanford/universal_manipulation_interface`** (UMI, MIT，用于便携式夹爪数据采集的模板，后来催生了 UMI-on-Legs)； **`NVIDIA-Omniverse/IsaacLab`** 移动操作环境；**斯坦福Robocasa / RoboCasa**（Apache-2.0）用于家庭规模的仿真。已投入生产：MoveIt2、Nav2、stretch_ros2、openpi。研究原型：其余大部分。

### 代表性硬件平台

| 平台 | 底盘 | 机械臂 / 自由度 | 负载 | 年份 | 开放性 |
|---|---|---|---|---|---|
| **Fetch** | 差动式 | 7自由度机械臂 + 躯干提升 | 6 kg | 2015 | 开放式 ROS，硬件已停产 |
| **PR2** (Willow) | 全向 | 2×7-DOF | 1.8 kg/臂 | 2010 | 完全开源，已弃用 |
| **丰田HSR** | 全向 | 5-DOF + 抓手 | 1.2 kg | 2017 | 仅限合作伙伴 |
| **PAL Tiago++** | 差动/全向 | 1 或 2 × 7-DOF | 3 kg | 2019 | ROS 开源 |
| **Stretch 3** (Hello Robot) | 差动 | 棱柱式 + 腕部（有效 5-DOF） | 1.5 kg | 2024 | **完全开源，约 $25,000** |
| **Mobile ALOHA** | AgileX Tracer（差动） | 2×ViperX 6-DOF | ~0.75 kg/臂 | 2024 | 完全开源，约$32k |
| **Galaxea R1 / R1 Pro** | 轮式躯干 + 腰部 | 2×7-DOF + 2-DOF 腰部 | 5 kg/臂 | 2024-25 | 部分 SDK |
| **AgileX Cobot Magic** | 差动式 | 2×6-DOF | 3 kg | 2024 | SDK 开源 |
| **Ridgeback + Panda** | 全向（麦克纳姆轮） | 7自由度 Franka | 3 kg | — | ROS 开源 |
| **Husky + UR5/UR10** | 滑移转向 | 6自由度 UR | 5-10 kg | — | ROS 开源 |

Spot+arm 属于边界情况（腿式而非轮式），归类于 D3 部分。

### 经典奠基性论文（2022 年前）

**Khatib，《运动与力控制的统一方法：操作空间建模》**（1987年，被引用超过8000次）仍是任务优先级WBC及所有现代分层QP求解器的根基。 **Stilman，《可移动障碍物间的导航》**（IJRR 2008，约500次引用）至今仍定义着当今TAMP论文中使用的NAMO分类法。**Berenson等人，《任务空间区域/CBiRRT》**（IJRR 2011）是典型的基于约束流形采样的规划器。 **Srinivasa等人，《HERB 2.0：移动机械手开发经验总结》**（PIEEE 2012）是系统集成的参考标准。**Chitta等人，《MoveIt!》** (ICRA-M 2012) 以及 **Quigley 等人的《ROS》** (ICRA 研讨会 2009) 界定了软件基础架构。**Sentis 与 Khatib 的《通过行为原语的分层控制合成全身行为》** (IJHR 2005) 奠定了 WBC 研究脉络。 **Brock &amp; Khatib 的《弹性带：人类环境中的运动生成框架》**（IJRR 2002）奠定了反应式重规划的基础。**Diankov 的 OpenRAVE 博士论文**（CMU 2010）提供了算法工具包。**Bohg 等人的《数据驱动抓取合成——综述》**（T-RO 2014，被引用超过1400次）**仍是抓取领域文献的权威参考。**Garrett 等人的 PDDLStream**（ICAPS 2020）是现代 TAMP 研究的重要切入点。

### 近期前沿论文（2023-2026）

2024-2026年的D2领域文献主要由基于学习的通用模型主导。**Fu、Zhao、Finn的《Mobile ALOHA：基于低成本全身遥操作的双臂移动操作学习》**（arXiv 2401.02117）点燃了双臂移动操作研究的热潮。 **Aloha 2**和**ALOHA Unleashed**（DeepMind 2024）实现了数据和扩散策略的规模化。 **Black等人，《π0：一种用于通用机器人控制的视觉-语言-动作流模型》**（arXiv 2410.24164，Physical Intelligence 2024）提出了基于7种具身形态和68项任务训练的流匹配VLA模型。 **《Physical Intelligence》期刊发表的“π0.5：具备开放世界泛化能力的VLA”**（arXiv 2504.16054，2025）展示了移动机械臂在**从未见过的住宅中完成10-15分钟的自主厨房/卧室清洁**——这在π0的基础上实现了质的飞跃。 **Pertsch等人，《FAST：VLA的高效动作分词》**（arXiv 2501.09747，2025）提出了基于DCT的动作分词方法。**Open X-Embodiment + RT-X**（ICRA 2024最佳论文）建立了跨具身预训练框架。 **Kim等人，《OpenVLA》**（CoRL 2024）发布了首个70亿参数的开源VLA模型。 **NVIDIA GR00T N1**（2025年3月）和**GR00T N1.5**（2025年5月，Computex）将双系统VLA（7-9 Hz的VLM系统2 + 动作专家系统1）引入类人机器人，并在**Fourier GR-1和1X NEO**上进行了演示； **GR00T N1.5采用流匹配+FLARE潜变量对齐+DreamGen合成数据**，仅需单个人类演示即可针对新任务进行微调。**Wu等人，《TidyBot：基于大型语言模型的个性化机器人辅助》**（AuRo 2023）普及了LLM→操作的架构。 **Liu等人，《VoxPoser》**（CoRL 2023）将语言映射到3D体素可操作性。**OK-Robot**（Meta/NYU 2024）、**HomeRobot OVMM挑战赛**以及**RoboCasa**（斯坦福大学 2024）共同构成了移动操作评估的格局。 **Chi等人提出的“Diffusion Policy”**（RSS 2023）和 **UMI**（Chi、Song，RSS 2024）是模仿学习的基石。 总体趋势：到2025年，前沿的D2架构将演变为**SLAM + nav2（导航）+ VLA（操作）+ 稀疏经典抓取备用方案**，而基于MPC的移动操作（OCS2）如今主要作为教育范例。

### 针对D2的C++库

**MoveIt 2**（任务构造器、Servo、Pilz运动规划器）；**Nav2**及其MPPI控制器、Smac规划器和TEB局部规划器；**OCS2 `mobile_manipulator`**（适用于Ridgeback+Panda风格的SQP+HPIPM NMPC）；**Isaac Sim移动操作**环境； 用于抓取候选项的 **contact-graspnet / AnyGrasp / GraspIt! / Dex-Net**；**PDDLStream / Caelan Garrett TAMP**；用于姿态-SLAM集成的 **`gtsam`**；通常采用Python实现的LLM编排（LangGraph、LlamaIndex），并通过ROS 2动作向C++暴露接口。 LeRobot 的运行时以 Python 为主，但社区正在将推理功能移植到 libtorch/ONNX-Runtime/TensorRT 以支持边缘部署。

### 数学增量

D2 引入：**SE(2) 移动底盘规划**（差分/全向/阿克曼运动学）与机械臂运动学耦合； **基于机械臂的雅可比矩阵耦合** J=[J_base | J_arm]∈ℝ^{6×(3+n)}；**带非全控约束的冗余消解**；**基于移动底盘的视觉伺服**（IBVS/PBVS，存在底盘诱导的可观测性问题）； **TAMP符号-连续积分**（PDDLStream风格）；以及至关重要的**VLA动作分段化**（FAST = DCT + 基于50步区段的码本量化），这是一个在机器人学研究生课程中尚属新颖的数学对象，值得单独成章。

### 博士研究开放性问题（2025-2026）

(1) **长时效杂波操控**，包含20多个子任务及优雅的故障恢复机制；(2) **基于移动底盘的不确定性感知抓取**——底盘漂移估计与抓取重新规划的紧密耦合； (3) **跨具身VLA迁移**：从轮式机械臂到腿足+机械臂系统（即物理智能领域所称的“跨具身迁移问题”）；(4) **欠驱动机械臂的实时全身MPC**（Stretch类棱柱机械臂需要非标准NMPC建模）； (5) **SLAM与操作的耦合**：当机械臂移动同时充当视觉地标的物体时，经典SLAM会失效；(6) **VLA + 显式安全**（围绕学习策略的正式安全包络）。

