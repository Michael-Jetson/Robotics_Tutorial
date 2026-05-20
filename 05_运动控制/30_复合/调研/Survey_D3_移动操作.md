## D3 移动-操作：最复杂、发展最快的前沿领域

D3 是用户既有背景（SLAM + 腿足机器人 + 强化学习）能发挥最大作用的领域。我将其分为 **D3a 四足机器人+机械臂**（以 MPC+WBC + 混合强化学习为主）和 **D3b 人形机器人**（以全身强化学习 + VLA 为主）。

### D3a 四足机器人+机械臂：开源生态系统

**`skywoodsz/qm_control`**（AGPL/MIT 许可，适用于 OCS2+ros-control 平台上的 Unitree B1+Z1 / Aliengo+Z1 的标准 NMPC+WBC 栈，被 IROS 2024 引用； 分支涵盖全身运动、力追踪、顺应性及硬件部署——这是D3a领域最具指导意义的开源代码库) 无疑是MPC路径读者的首选。**`danisotelo/qm_door`** 是一个活跃的AlienGo+Z1搜救分支，新增了视觉、规划和开门功能。 **`qiayuanl/legged_control`**（用户已知）是 qm_control 系列所扩展的 MPC 基线。**`Ericonaldo/visual_wholebody`**（麻省理工学院，Liu、Chen、Cheng、Ji、Yang、 王合著的《基于视觉的腿足式移动-操作全身控制》（CoRL 2024，arXiv 2403.16967）将低级统一强化学习策略（所有自由度追踪身体速度 + 能量最优位置）与高级视觉策略相结合。**`real-stanford/umi-on-legs`** (Ha, Gao, Fu, Tan, Song, CoRL 2024, arXiv 2407.10353) 是斯坦福+哥伦比亚大学系统：一种全身控制器，该控制器追踪由UMI训练的扩散策略生成的任务坐标系下末端执行器轨迹，从而实现将桌面操作策略零样本迁移至Unitree Go2 + ARX5机械臂； &gt;在抓取/非抓取/动态任务中成功率超过70%。**`MarkFzp/Deep-Whole-Body-Control`**（&quot;Deep WBC&quot;，Fu, Cheng, Pathak, CoRL 2022 口头报告 + 最佳系统决赛入围；arXiv 2210.10044）是统一强化学习（Unified-RL）领域的基准。 **`jin-cheng-me/rambo`** 实现了 **Cheng, Kang, Fadini, Shi, Coros, &quot;RAMBO: 基于强化学习的模型增强式全身控制方案&quot;**（arXiv 2504.06662, 2025）——该混合方案中，QP生成前馈扭矩，而强化学习策略提供残差反馈，已在四足推车、平衡板及双足行走任务中得到验证。 **`LocoMan`**（Lin 等，2024，arXiv 2403.18197）是一种轻量级的 6-DOF 行走-操纵器机身附加组件。**`RoboDuet`**（Pang 等，2024）采用两个具有零样本跨具体能力的协作策略。 其他活跃项目：LeCAR的Pedipulate（ETH）、Argmin/Argility的“Legs as Manipulator”（Cheng, Kumar, Pathak, ICRA 2023），以及针对B1Z1的Unitree示例。

### D3a硬件平台

| 平台 | 腿部自由度 | 手臂自由度 | 手臂负载 | 底盘速度 | 总质量 | 开源状态 |
|---|---|---|---|---|---|---|
| **Unitree Go2+Z1** | 12 | 6 | 5 kg | 3.7 m/s | 20 kg | SDK + URDF 开源 |
| **Unitree B1+Z1** | 12 | 6 | 5 kg | 1.5 m/s | 60 kg | SDK 开源 |
| **Unitree B2+Z1** | 12 | 6 | 7 kg | 5 m/s | 70 kg | SDK 开源 |
| **Aliengo+Z1** | 12 | 6 | 5 kg | 1.5 m/s | 30 kg | SDK 开放 |
| **Boston Dynamics Spot+Arm** | 12 | 6 | 11 kg | 1.6 m/s | 50+ kg | 仅 API，底层闭源 |
| **ANYmal-C/D + DynaArm** | 12 | 6 | ~5 kg | 1.2 m/s | 60 kg | 通过 RSL 支持 URDF |
| **ALMA (ANYmal-B + Kinova Jaco)** | 12 | 6-7 | ~2 kg | 1.0 m/s | 45 kg | 仅限研究 |
| **IIT HyQReal + 机械臂** | 12 | 7 | 5 kg | 1.5 m/s | ~130 kg | 部分支持 |
| **Jueying X30+机械臂** (DeepRobotics) | 12 | 6 | 5 kg | 4 m/s | 50 kg | SDK 部分支持 |

### D3a 经典论文

**Bellicoso 等人的《ALMA——适用于扭矩可控机器人的关节式行走与操作》**（ICRA 2019，约250次引用）是四足+机械臂系统中分层WBC方法的基础性参考文献。 **Sleiman、Farshidian、Minniti、Hutter 的《全身动态行走与操作的统一 MPC 框架》**（RA-L 2021，约 350 次引用）是所有基于 OCS2 的行走-操作栈（包括 qm_control）都引用的*那篇*论文。 **Khatib 1987年的操作空间**仍是WBC的根基。**Mistry、Buchli、Schaal的《基于正交分解的浮动基座系统逆动力学控制》**（ICRA 2010）将约束一致的WBC进行了形式化。**Sentis与Khatib的分层WBC（2005-2007）**。 **Righetti等人，《具有最优接触力分布的逆动力学二次规划》**（Humanoids 2011）是接触力二次规划（QP）的权威参考。 **Winkler、Bellicoso、Hutter、Buchli，《基于相位的末端执行器参数化方法下的腿足系统步态与轨迹优化》**（RA-L 2018, TOWR）仍是经典的立足点规划基准。 **Zucker等人提出的CHOMP**（IJRR 2013）是协变臂规划领域的经典参考。 **Fankhauser 等人的感知四足机器人**系列论文以及 **Hutter 的原始 ANYmal**（IROS 2016）构成了腿足基底的理论框架。**Hyon、Hale、Cheng 的《全身柔顺的人-类人形机器人交互》**（T-RO 2007）预示了当今的力自适应行走-操作技术。

### D3a 近期前沿（2023-2026）

除已列出的 Deep WBC、VBC、UMI-on-Legs、RoboDuet、LocoMan 和 RAMBO 之外： **Arm、Mittal、Kolvenbach、Hutter 的《Pedipulate：利用四足机器人腿部实现操作技能》（ICRA 2024，arXiv 2402.10837）将腿部本身作为操作器使用。**Ji、Margolis、Agrawal，《Dribblebot：野外环境中的动态腿足操纵》（ICRA 2023）确立了野外动态操纵的基准。**Portela、Margolis、Ji、Agrawal，《腿足操纵的力控制学习》（arXiv 2024）将基于腿足+手臂系统的强化学习力控制进行了形式化。 **Zhang、Lin、Peng、Xiong、Lou，《具有驱动饱和特性的四足操纵器的全身顺应性控制》**（IROS 2024）是 qm_control 的配套论文。 **《利用腿足式机械手学习开门与穿越门》**（CoRL 2024，苏黎世联邦理工学院）。**《HYPERmotion：自主行走-操作任务的混合行为规划学习》**（CoRL 2024）。**《面向鲁棒多接触行走-操作的引导式强化学习》**（CoRL 2024）。 **“HiLMa-Res：基于残差强化学习的分层框架，用于四足行走与操作”**（IROS 2024）。**“MLM：学习带手臂四足机器人的多任务行走-操作全身控制”**（arXiv 2508.10538，2025）。 **“Playful DoggyBot” / “Helpful DoggyBot”**（2024）展示了基于VLM的四足+机械臂在开放世界中的取物任务。**“Catch It! 利用移动灵巧手学习空中接物”**（ICRA 2025）推动了基于腿足底盘的动态操作研究。 **&quot;SPIN / 通过灵活的肢体间协调实现多功能移动-操作&quot;** (arXiv 2025)。达到《Science Robotics》水平的研究日益普遍；预计到2026年，每年将有多篇关于四足+机械臂的论文发表在《Science Robotics》上。

### D3b 人形机器人平台的开放性 (2025-2026) —— 决定性因素

| 平台 | 自由度 | 身高/体重 | 研究开放性 | 价格（2026年） |
|---|---|---|---|---|
| **Unitree G1** | 23-43 | 1.27 米 / 35 公斤 | **开放式 SDK、公开 URDF、Isaac Lab 支持、仅 2025 年就有超过 30 篇论文** | **基础版 1.35-1.79 万美元，教育版最高 7.39 万美元** |
| **Unitree H1** | 19-27岁 | 1.80 米 / 47 公斤 | 开放式 SDK、URDF、MuJoCo/Isaac Lab；吉尼斯世界纪录 3.3 米/秒 | **约 $90-150k** |
| **Unitree H2** | — | — | 已宣布将于2025年发布 | **2.99万美元**（据Unitree商店列表） |
| **Unitree R1** | — | 体型较小，运动型 | 开源 SDK | **$4.9-5.9k** |
| **波士顿动力 Atlas（电动版）** | ~28 | 1.5 米 | 闭源；可通过 TRI **大型行为模型**合作计划（2024-2025）获取研究访问权限 | 不对外销售 |
| **Fourier GR-1 / GR-2** | ~44 | 1.65 米 / 55 公斤 | 半开放式 SDK，用于 NVIDIA GR00T N1 演示 | ~$50-100k |
| **Apptronik Apollo** | ~40 | 1.73 米 / 73 公斤 | 商用（梅赛德斯、GXO）；研究访问受限 | 仅限企业 |
| **Figure 01/02/03** | 35+（上半身） | 1.70 米 | **封闭**； Helix 01/02 VLA论文仅以博客文章形式公开 | ~13万美元企业租赁（估算） |
| **1X NEO / NEO Gamma** | — | — | 闭源；公开1X World Model论文；采用NVIDIA GR00T N1 | 不对外销售 |
| **特斯拉Optimus第2/3代** | 28+ | 1.73米 | 闭源 | 不对外销售 |
| **Agility Digit** | ~30 | 1.75米 / 65公斤 | 半商业化，GXO/亚马逊试点项目 | 仅限企业用户 |
| **PAL Talos** | 32 | 1.75 米 / 95 公斤 | 经典开源研究机器人 | ~100 万美元 |
| **Booster T1** | ~23 | 1.2 米 | 开源 SDK，研究导向 | ~2-3 万美元 |
| **Kepler Forerunner K2** | 52 | 1.78 米 | 新兴，部分 SDK | 待定 |
| **UBTECH Walker S1** | 41 | 1.72 米 | 闭源 | 企业级 |
| **LimX CL-1 / P1** | ~30 | 1.65 米 | 部分 SDK | 待定 |
| **EngineAI PM01 / SE01** | 23+ | 1.38 米 | 新兴，计划开放 SDK | **起价 $13.7k** |
| **XPeng Iron** | — | — | 闭源 | 不对外销售 |
| **Galbot G1** | — | 轮式类人机器人 | 部分 SDK | 企业级 |
| **AGIBOT X2-N &quot;哪吒&quot;** | — | 双足↔轮式变形 | 闭源 | 企业级 |

**核心结论**：**Unitree G1**现已成为标准研究型人形机器人（价格、自由度范围、SDK成熟度、URDF可用性及社区生态均趋于一致）。 2025年大部分学术类人形机器人论文（ASAP、FALCON、HOMIE、HOVER、HomieBot）均采用G1或H1。Figure/1X/Tesla仍处于闭源状态；Atlas仅限TRI合作伙伴进行研究；Digit的研究访问权限受限。建议以G1/H1为核心规划课程。

### D3b人形机器人开源生态系统

**`unitreerobotics/unitree_rl_gym`** 和 **`unitree_sdk2`**（Apache-2.0许可，生产级，支持G1/H1强化学习）。 **`leggedrobotics/legged_gym`** 双足机器人分支。**`NVIDIA-Omniverse/IsaacLab`** 人形机器人环境（Apache-2.0）。**`MarkFzp/humanplus`**（斯坦福大学，HumanPlus 2024：基于人类视频的影子学习与自主全身技能）。 **`LeCAR-Lab/human2humanoid`** (H2O 和 OmniH2O)。**`LeCAR-Lab/ASAP`** (MIT 许可，RSS 2025，arXiv 2502.01143；G1 上的 real2sim delta-action-model 参考模型)。 **`LeCAR-Lab/FALCON`** (L4DC 2026, arXiv 2505.06776, 基于Unitree G1和Booster T1的力自适应类人机器人行走-操作任务，包含开门和拉车动作)。 **`LeCAR-Lab/BFM-Zero`**, **`SPI-Active`**。 **ExBody / ExBody2**（加州大学圣地亚哥分校 / 卡内基梅隆大学）。**PHC / PULSE**（卡内基梅隆大学，人形机器人持续控制，动作捕捉追踪）。**HoST**（上海人工智能实验室）。**OpenTeleVision**（卡内基梅隆大学/加州大学圣地亚哥分校，VR遥操作）。 **HOMIE**（RSS 2025，同构外骨骼驾驶舱遥操作）。**`stack-of-tasks/tsid`** 和 **`stack-of-tasks/sot-core`**（LAAS，TALOS级C++ WBC）。**`Rhoban/placo`**（人形机器人差分反向运动学 + QP）。 **`RobotLocomotionGroup/drake`** 用于Atlas规划（仍在TRI研究中使用）。**开放式人形机器人强化学习训练管道**（YanjieZe的`awesome-humanoid-robot-learning`列表涵盖了2025-2026年数百篇arXiv论文）。

### D3b经典人形机器人论文

**Kajita、Kanehiro、Kaneko等人合著的《利用零力矩点的预览控制生成双足行走模式》**（ICRA 2003，被引用超过3500次）奠定了LIPM+预览控制的基础。 **Wieber，《强扰动条件下实现稳定行走的无轨迹线性模型预测控制》**（Humanoids 2006，被引次数&gt;600）——线性MPC行走研究论文。 **Pratt、Carff、Drakunov、Goswami，《捕获点：迈向类人机器人推力恢复的一步》**（Humanoids 2006，被引次数&gt;900）。**Sentis &amp; Khatib WBC 2005-2007**。 **Hirukawa、Kanehiro 等，《HRP动力学与行走》**（IJRR 2006）。**Koolen 等，《IHMC团队虚拟机器人挑战赛参赛总结》/《基于动量的控制框架设计》**（Humanoids / IJHR 2012-2016）。 **Feng、Dai、Tedrake，《DARPA机器人挑战赛的基于优化的全身控制》**（《田野机器人学杂志》2015）。**Kuindersma等人，《Atlas的基于优化的运动规划、估计与控制设计》**（AuRo 2016，约1000次引用）。 **Escande、Mansard、Wieber，《分层二次规划》**（IJRR 2014）。**Englsberger、Ott、Albu-Schäffer，《基于运动发散分量的三维双足行走控制》**（T-RO 2015）——DCM。 **Herzog、Rotella等人，基于动量的双足行走控制**（IJHR 2016）。**Tedrake的直接配置Atlas**。

### D3b近期类人机器人论文（2023-2026）

**Fu、Zhao、Finn，《HumanPlus：基于人类的类人机器人影子跟随与模仿》** (CoRL 2024, 斯坦福大学)。**He、Luo、Wang、Shi，《H2O：学习人机实时全身遥操作》** (arXiv 2403.04436, ICRA 2025)。 **He等人，《OmniH2O：通用且灵巧的人类到类人机器人全身遥操作与学习》**（arXiv 2406.08858，2024年）。 **Cheng、Shi、Pathak，《ExBody：类人机器人的表现力全身控制》**（RSS 2024）及 **Ji 等，《ExBody2》**（2025）。**He、Gao、Xiao 等，《ASAP：通过协调模拟与真实世界物理规律学习敏捷类人机器人全身技能》** (RSS 2025, arXiv 2502.01143, CMU LeCAR + NVIDIA) — **两阶段模拟到现实：在模拟环境中预训练运动追踪 + 基于真实滚动模拟训练Delta-Action模型以弥合模拟到现实的动力学差距，在Unitree G1上经过验证**；2025年被引用次数最多的人形机器人论文。 **Mittal等人，《HOVER：适用于类人机器人的通用神经全身控制器》**（NVIDIA 2024）。 **Li、Zhang、Xiao等人，《SoFTA/Hold My Beer：柔和的人形机器人行走与能量效率稳定化》**（arXiv 2505.24198，2025年）。 **张、肖等人，《FALCON：力自适应类人机器人行走-操作系统》**（L4DC 2026，arXiv 2505.06776）。 **卢、肖等人，《Mobile-TeleVision：人形机器人全身控制的预测性运动先验》**（arXiv 2024，UCSD/MIT/NVIDIA；相较于H1，操作增益提升约40%）。 **&quot;HOMIE：基于同构外骨骼驾驶舱的人形机器人行走-操作系统&quot;**（RSS 2025）。**&quot;WoCoCo：基于序列接触的学习型人形机器人全身控制&quot;**（CoRL 2024）。**Luo、Kitani，&quot;PHC / PULSE&quot;**（CMU）。 **&quot;ACE：外骨骼式人形机器人遥操作的自适应控制&quot;**。**&quot;SkillBlender&quot;**（CoRL 2024）。**&quot;BFM-Zero：可提示的行为基础模型&quot;**（2025）。**&quot;BeyondMimic / OmniRetarget&quot;** ——将动作捕捉数据重映射至人形机器人。 **TRI + 波士顿动力 &quot;Atlas的大型行为模型&quot;**（2024-2025系列）。G1平台上的类人机器人跑酷（2025年多篇论文，例如清华大学、卡内基梅隆大学）。谷歌DeepMind为类人机器人开发的MuJoCo运动规划控制（MPC）。类人机器人足球（DeepMind 2024，2025年春季Booster会议）。 **EGM、CHIP、PvP、SENTINEL、VIRAL、HMC、HAFO、SafeFall**（2025年底YanjieZe列表中的arXiv预印本）。

### 跨领域VLA / 基础模型前沿

**Brohan 等，RT-1 / RT-2**（Google 2022-2023）和 **Open X-Embodiment / RT-X**（ICRA 2024 最佳论文）。 **Kim等人，OpenVLA**（CoRL 2024）。**Octo团队，“Octo”**（RSS 2024）。**π0 / π0-FAST / π0.5**（Physical Intelligence，2024-2025）——π0.5是当前在未见过的家庭环境中进行通用移动操作的最新最先进（SOTA）方法。 **Figure AI, Helix**（2025年2月）：一种“系统1、系统2”VLA，包含7-9 Hz的S2视觉运动模型（VLM）及200 Hz的S1视觉运动模块，可控制35自由度类人上肢； **Helix 02**（2026年1月）**扩展至全身自主控制，演示了从像素级端到端的4分钟洗碗机装载任务——首个实用的长时效人形机器人行走-操作VLA**。**1X World Model**（2024年）。 **NVIDIA GR00T N1**（2025年3月，arXiv）和 **GR00T N1.5**（2025年5月，基于Eagle-2.5的VLM + 流匹配 + FLARE + DreamGen合成数据，2025年7月与Newton物理引擎一同开源）。 **Pertsch等人，FAST**（arXiv 2501.09747，DCT动作分词）。 **Cheang等人，GR-3**（字节跳动，arXiv 2507.15493，2025年）。**RDT-1B**（ICLR 2025，用于双臂操作的扩散基础模型）。 **CogACT**、**X-VLA**、**InternVLA-A1**、**SpatialVLA**（2025年各类arXiv论文）。**Unitree UnifoLM-VLA-0**（2026年3月，已开源）。 **DeepMind AutoRT / RT-Trajectory / RT-H**。所有这些模型的共同特征是：**VLM 骨干网络（Eagle / Qwen2.5-VL / PaLI）+ 流匹配或 FAST 词法分析器动作头 + 跨具身协同训练 + 合成数据增强（DreamGen, Cosmos）**。 在复合机器人中的适用性：π0.5 已运行于移动机械臂；GR00T N1.5 面向类人机器人（Fourier GR-1、1X NEO）；Helix 运行于 Figure 02/03；UMI-on-Legs 表明“VLA→任务帧 EE → WBC”接口是将操作 VLA 移植到腿足底座上的主流模板。

### D3专用的C++库

**TSID** (TALOS)、**HierarchicalWbc**、**SoT (任务堆栈，LAAS)**、**placo (Rhoban)**、 **Crocoddyl**（含机械臂残差），**mink**（差分反向运动学），**OCS2 mobile_manipulator**（用于四足机器人+机械臂），**Pinocchio 3.x 约束关节**（用于闭链机械臂和手部），**raisim**（商业软件，但学术界免费使用）用于快速腿足-机械臂仿真，**Isaac Lab** 腿上机械臂，**TOWR**（含机械臂能量平衡目标），**contact-graspnet / AnyGrasp / GraspIt!** 用于抓取候选项，**LEAP Hand / Allegro Hand ROS2** 驱动程序用于灵巧手，**Shadow Hand** 堆栈，**Unitree Dex3-1** SDK（G1的3指手）。用于VLA推理：**libtorch**、**ONNX Runtime**、**TensorRT 10**、**NVIDIA Triton**；在Jetson Thor/Orin上进行边缘部署。 **LeRobot** Python + libtorch 运行时。**Newton 物理引擎**（NVIDIA + Google DeepMind + Disney，2025年7月开源）是 MuJoCo/PhysX 在可微分仿真领域中崭露头角的替代方案。

### 数学增量（D3）

超越24章节腿足机器人课程的核心增量：**统一的基座+手臂动力学**（6 + n_leg + m_arm 个自由度，并谨慎处理重力/科里奥利耦合）；**任务优先级冲突解决**，即行走任务（基座姿态、接触力、质心、角动量）与操作任务（末端执行器姿态、末端执行器力、抓取约束）之间的冲突； **动态接触集管理**（轨迹中途添加/移除手臂/手部接触点，这是一个组合优化问题，CITO/Aligator可提供帮助）；**手臂受扰动时的反作用力可行性**（手臂施力时摩擦锥收紧）；**摆动手臂下的质心动量**——对类人机器人至关重要； **负重手推力恢复过程中的角动量调节**；**双臂任务的双手接触扭矩锥**；**SMPL-X → 人形机器人重定位**（应用于 ASAP、H2O、OmniH2O）；**ASAP 风格的 delta-action 残差模型**（从真实推演中学习到的状态转换校正）； **VLA令牌化**（FAST = DCT + 码本）及作为一等数学对象的**流匹配动作头**；用于全身策略的**师-生蒸馏**（特权观测值 → 本体感觉）；基于移动底盘的**非抓握式操作动力学**。

### 博士研究课题（2025-2026）

基于研究者背景最具影响力的研究课题：(1) **行走时进行动态双手法操作**（速度&gt;2 m/s）——2025年的大部分研究属于准静态；(2) **借助触觉感知在移动过程中进行灵巧的手内操作**； (3) **步态干扰下的高接触力控制** —— FALCON 和 RAMBO 只是初步尝试，但泛化问题尚未解决；(4) **长时效 LLM/VLM 任务规划 + 低层 WBC**，且需保证可行性； (5) **G1、H1、Atlas、Digit之间**的**移动-操作策略跨具身迁移**（即π0.5论文中跨具身部分）；(6) **针对&gt;30自由度**的**实时全身多体控制**，响应时间控制在10毫秒内——当前SQP+HPIPM架构在25-30自由度左右会陷入停滞；(7) **高接触频率类人机器人的仿真到实机性能差距**——ASAP是首次尝试，但delta-action模型存在局限；(8) **负重手部推力恢复**及鲁棒的足部位置再生；(9) **具有实时保证的全身VLA**——Helix 02的200 Hz S1是当前标杆，但仍未达到形式安全； (10) **人形机器人的SLAM与操作耦合**，其中移动物体同时充当地标；(11) **基于世界模型的行走-操作**（针对全身任务的DreamerV3级预测控制）。

