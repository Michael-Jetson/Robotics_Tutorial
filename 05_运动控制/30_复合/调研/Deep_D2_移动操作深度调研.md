## 移动操作研究生教学大纲 复合/120_底盘臂联合规划–85 深度调研报告

> **范围**：轮式底盘 + 机械臂的复合机器人（Mobile Manipulation, MM）；排除纯自动驾驶与飞行器。
> **编写对象**：已完成 SLAM 46 章主线 + 腿足规控 24 章 (Pinocchio / OCS2 / Crocoddyl / TSID / WBC / DDP / IsaacLab+rsl_rl / LibTorch+ONNX / grid_map / Perceptive MPC) + 轮足 6 章的研究生。
> **技术偏好**：MPC + WBC 为主线，RL / VLA 作为扩展。
> **引用数说明**：均为 2026 年 4 月的区间估计，最终数值请以 Google Scholar 实时为准。

---

## 一、论文清单（27 篇，按三代划分）

### A. 经典奠基期（1987–2020）

#### 1. Khatib 1987 — 操作空间统一控制
- *A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation*, IEEE JRA 3(1):43–53, 1987.
- **引用**：≈ 5500–6500。
- **贡献**：定义任务空间惯量 Λ(q)=(JM⁻¹Jᵀ)⁻¹，将运动与力控制统一于末端执行器坐标；首次系统给出零空间投影 N=I−J⁺J 处理冗余。
- **代码**：无原官仓；Stanford SAI / Pinocchio-TSID / OpenSoT 为继承实现。
- **谱系位置**：所有 WBC / OSC / MPC-WBC（Sentis、Righetti、Minniti）的根节点；是 MM 控制分支起点。

#### 2. Stilman & Kuffner 2008 — NAMO
- *Planning Among Movable Obstacles with Artificial Constraints*, IJRR 27(11–12):1295–1307, 2008。
- **引用**：≈ 400–500。
- **贡献**：提出"人工约束"递归剪枝算法——反向规划扫略体作为未来操作的约束，LP1/LP2 单调性保证完备性；定义 Transit / Transfer 双图。
- **代码**：原型未公开；第三方 NAMOSIM (ki-robotics-lab/namosim)。
- **谱系位置**："环境可变"TAMP 分支奠基，启发 Dogar 2012 rearrangement、Garrett LIS TAMP。

#### 3. Berenson, Srinivasa & Kuffner 2011 — TSR / CBiRRT
- *Task Space Regions: A Framework for Pose-Constrained Manipulation Planning*, IJRR 30(12):1435–1460。
- **引用**：≈ 800–1000。
- **贡献**：6D 任务盒+TSR-Chain 表达位姿约束；CBiRRT2 在约束流形上具概率完备性，统一处理闭链、双臂搬大物、开门。
- **代码**：https://github.com/personalrobotics/comps (OpenRAVE 插件)。
- **谱系位置**：约束采样规划的事实标准；后续 OMPL Constrained、Atlas-RRT、MoveIt Constrained Planning 均基于此。

#### 4. Srinivasa et al. 2012 — HERB 2.0
- *HERB 2.0: Lessons Learned from Developing a Mobile Manipulator for the Home*, Proc. IEEE 100(8):2410–2428。
- **引用**：≈ 500–650。
- **贡献**：Segway 底盘+双 Barrett WAM 全栈系统——MOPED 视觉、CBiRRT 规划、push-grasping in clutter、caging grasp 开门。
- **代码**：PRL 组件 https://github.com/personalrobotics（prpy、or_cdchomp、comps）。
- **谱系位置**：家庭 MM 综合系统的教科书范本，TAMP+感知+操作全栈论述框架的原型。

#### 5. Chitta, Şucan & Cousins 2012 — MoveIt!
- *MoveIt! [ROS Topics]*, IEEE RAM 19(1):18–19。
- **引用**：≈ 1500–2000。
- **贡献**：把 OMPL + FCL + OctoMap + KDL/IKFast 封装为统一的 PlanningScene / MoveGroup / KinematicConstraints 抽象。
- **代码**：https://github.com/moveit/moveit , https://github.com/moveit/moveit2 。
- **谱系位置**：MM 的"基础设施层"——Fetch、TIAGo、Stretch、Spot+arm 均基于此。

#### 6. Sentis & Khatib 2005 — 分层 WBC
- *Synthesis of Whole-Body Behaviors through Hierarchical Control of Behavioral Primitives*, IJHR 2(4):505–518。
- **引用**：≈ 450–600。
- **贡献**：约束 > 操作 > 姿态的三级优先级投影算子，实现动力学解耦；ControlIt! / Stack-of-Tasks 的理论源头。
- **代码**：stack-of-tasks/sot-core（思想继承）。
- **谱系位置**：瞬时 QP-WBC 分支之父，与 Minniti 2019 的预测式 WBC 构成双源。

#### 7. Garrett, Lozano-Pérez & Kaelbling 2020 — PDDLStream
- *PDDLStream: Integrating Symbolic Planners and Blackbox Samplers via Optimistic Adaptive Planning*, ICAPS 2020:440–448。
- **引用**：≈ 350–500。
- **贡献**：引入 stream（条件生成器）声明式描述 IK/Grasp/collision 采样；optimistic adaptive planning 在假设-实证间自适应权衡。
- **代码**：https://github.com/caelan/pddlstream , https://github.com/caelan/pybullet-planning 。
- **谱系位置**：现代 TAMP 主流实现；与 LGP (Toussaint)、HPN (Kaelbling) 并列，MM 中作为长时序规划顶层。

#### 8. Bohg et al. 2014 — 数据驱动抓取综述
- *Data-Driven Grasp Synthesis—A Survey*, IEEE T-RO 30(2):289–309。
- **引用**：≈ 1800–2200。
- **贡献**：按物体熟悉度划分已知/相似/未知三类，梳理表示（点云、SQ、局部 patch）、采样与排序方法。
- **代码**：无（综述）。
- **谱系位置**：Dex-Net / GPD / Contact-GraspNet 的前置语汇；MM 抓取模块标准分类法。

#### 9. Minniti, Farshidian, Grandia & Hutter 2019 — 全身 MPC
- *Whole-Body MPC for a Dynamically Stable Mobile Manipulator*, RA-L 4(4):3687–3694。arXiv:1902.10415。
- **引用**：≈ 200–300。
- **贡献**：ballbot + 6DoF 臂的平衡、末端跟踪、接触力规划统一为单一 MPC；末端空间代价转录；SLQ/DDP 70 Hz 在线。
- **代码**：https://github.com/leggedrobotics/ocs2（核心求解器基础）。
- **谱系位置**：预测式 WBC 范式在 MM 中的奠基，前接 Sentis/Khatib，后开 Pankert 2020、Sleiman 2021、ALMA 2019。**本课程 复合/120_底盘臂联合规划 精读对象。**

#### 10. Pankert & Hutter 2020 — Perceptive MPC
- *Perceptive Model Predictive Control for Continuous Mobile Manipulation*, RA-L 5(4):6177–6184。
- **引用**：≈ 200–300。
- **贡献**：体素化 SDF/ESDF 视觉避障约束 + admittance 力控环同时嵌入 MPC；关节+ZMP 式支撑多级约束，>100 Hz 重规划。
- **代码**：https://github.com/leggedrobotics/perceptive_mpc （基于 OCS2）。
- **谱系位置**：感知-控制紧耦合 MPC 在 MM 的示范；Ridgeback+UR 与 ANYmal+Kinova 部署蓝本。**本课程 复合/120_底盘臂联合规划 精读对象。**

### B. 学习驱动期（2016–2023）

#### 11. Levine et al. 2016 — Deep Visuomotor Policies
- *End-to-End Training of Deep Visuomotor Policies*, JMLR 17(39):1–40。arXiv:1504.00702。
- **引用**：≈ 5500–6000。
- **贡献**：spatial-softmax CNN 端到端像素→力矩；Guided Policy Search (BADMM) 把轨迹优化作为监督信号。
- **代码**：https://github.com/cbfinn/gps 。
- **谱系位置**：视觉-运动联合训练奠基；与 MM 谱系相切于"低层端到端策略"。

#### 12. Kalashnikov et al. 2018 — QT-Opt
- *QT-Opt: Scalable Deep RL for Vision-Based Robotic Manipulation*, CoRL 2018。
- **引用**：≈ 2200–2500。
- **贡献**：闭环抓取 MDP + CEM on-Q 无 actor RL；7 台 Kuka × 4 月 × 58 万抓取，96% 未见物体成功率。
- **代码**：官方未开源；第三方 PyTorch 复现。
- **谱系位置**：真机大规模 RL 旗舰；与 MM 相切于"抓取"子模块。

#### 13. Brohan et al. 2023 — RT-1
- *RT-1: Robotics Transformer for Real-World Control at Scale*, RSS 2023。arXiv:2212.06817。
- **引用**：≈ 2300–2800。
- **贡献**：EfficientNet-B3 + FiLM + TokenLearner 离散动作 token；13 台 EDR × 17 月 × 130k 轨迹、700+ 任务。
- **代码**：https://github.com/google-research/robotics_transformer 。
- **谱系位置**：MM Transformer 范式起点；RT-2 / RT-X / OpenVLA 路线之源。

#### 14. Chi et al. 2023 — Diffusion Policy
- *Diffusion Policy: Visuomotor Policy Learning via Action Diffusion*, RSS 2023；IJRR 2024 扩展。
- **引用**：≈ 2800–3300。
- **贡献**：条件 DDPM 去噪生成动作 chunk；receding-horizon control + 视觉条件 U-Net/Transformer；处理多模态分布。
- **代码**：https://github.com/real-stanford/diffusion_policy 。
- **谱系位置**：生成式策略代表作；Mobile ALOHA、DP3、RDT、π0 的动作头先驱。**复合/140_VLA移动操作 精读。**

#### 15. Zhao et al. 2023 — ACT / ALOHA
- *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*, RSS 2023。
- **引用**：≈ 1800–2200。
- **贡献**：$20k 双臂遥操作硬件 + CVAE Transformer action chunking，10 分钟演示完成穿线等精细任务。
- **代码**：https://github.com/tonyzhaozh/act , https://github.com/tonyzhaozh/aloha 。
- **谱系位置**：低成本双臂 IL 起点，直接衍生 Mobile ALOHA。**复合/140_VLA移动操作 精读。**

#### 16. Wu et al. 2023 — TidyBot
- *TidyBot: Personalized Robot Assistance with LLMs*, Autonomous Robots 47(8):1087–1102；IROS 2023。
- **引用**：≈ 550–750。
- **贡献**：LLM few-shot 偏好摘要 + 开放词汇检测 + 移动基元执行；benchmark 91.2% 真机 85%。
- **代码**：https://github.com/jimmyyhwu/tidybot 。
- **谱系位置**：LLM 规划 + 模块化 MM 的典型。

#### 17. Huang et al. 2023 — VoxPoser（注：原任务写作 Liu，实为 Wenlong Huang）
- *VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models*, CoRL 2023。
- **引用**：≈ 1000–1300。
- **贡献**：LLM 代码调用 VLM 在 3D 体素合成 affordance/avoidance/rotation/velocity/gripper 五类值图作为 MPC 代价。
- **代码**：https://github.com/huangwl18/VoxPoser 。
- **谱系位置**：Code-as-Policies / SayCan 的 successor，连续轨迹输出。

#### 18. Ahn et al. 2022 — SayCan
- *Do As I Can, Not As I Say: Grounding Language in Robotic Affordances*, CoRL 2022。
- **引用**：≈ 3000–3600。
- **贡献**：P(skill|goal) · P(success|state,skill) 的 LLM + affordance 规划；PaLM-SayCan 厨房 101 任务 84%/74%。
- **代码**：https://github.com/google-research/google-research/tree/master/saycan 。
- **谱系位置**：基础模型驱动机器人里程碑；RT-1/RT-2 的高层互补。

### C. VLA 前沿期（2024–2026）

#### 19. Fu, Zhao & Finn 2024 — Mobile ALOHA
- *Mobile ALOHA: Learning Bimanual Mobile Manipulation with Low-Cost Whole-Body Teleoperation*, CoRL 2024。arXiv:2401.02117。
- **引用**：≈ 900–1100。
- **贡献**：$32k 双臂+移动底盘全身遥操作硬件；co-training 配方——50 条任务数据 + 静态 ALOHA/RT-X 混合训 ACT/Diffusion，成功率提升最多 90%。
- **代码**：https://github.com/MarkFzp/mobile-aloha , https://github.com/MarkFzp/act-plus-plus 。
- **谱系位置**：MM 低成本双臂数据采集的事实标杆，π0/π0.5 的上游数据来源。**复合/140_VLA移动操作 精读。**

#### 20. Black et al. 2024 — π0
- *π₀: A Vision-Language-Action Flow Model for General Robot Control*, Physical Intelligence. arXiv:2410.24164。
- **引用**：≈ 600–900。
- **贡献**：首个 flow matching action expert + PaliGemma-3B 的 VLA；50 Hz 高频 action chunk；7 机器人/68 任务/10k 小时。
- **代码**：https://github.com/Physical-Intelligence/openpi （权重开源）。
- **谱系位置**：连续动作 VLA 分支奠基，承接 OpenVLA + Diffusion Policy。**复合/150_Mobile_ALOHA与UMI 精读。**

#### 21. Pertsch et al. 2025 — FAST / π0-FAST
- *FAST: Efficient Action Tokenization for Vision-Language-Action Models*, RSS 2025. arXiv:2501.09747。
- **引用**：≈ 200–350。
- **贡献**：DCT + 量化 + BPE 压缩 action chunk 到 30–60 tokens（10× 压缩）；π0-FAST 训练计算省 5×，匹配扩散性能。
- **代码**：openpi 仓库内；FAST+ 通用 tokenizer。
- **谱系位置**：弥合自回归 VLA 与连续 VLA，被 π0.5 预训练阶段采用。**复合/150_Mobile_ALOHA与UMI 精读。**

#### 22. Physical Intelligence 2025 — π0.5
- *π₀.₅: a Vision-Language-Action Model with Open-World Generalization*, CoRL 2025. arXiv:2504.16054。
- **引用**：≈ 100–200。
- **贡献**：两阶段训练——FAST token 统一预训练（含 web 图文+子任务）→ flow matching action expert 微调；未见家居 10–15 分钟长时清洁。
- **代码**：openpi（KI 版本开源）。
- **谱系位置**：开放世界 MM 泛化最前沿；π0.6 (arXiv 2511.14759) 引入 RL。**复合/150_Mobile_ALOHA与UMI 精读。**

#### 23. Kim et al. 2024 — OpenVLA
- *OpenVLA: An Open-Source Vision-Language-Action Model*, CoRL 2024. arXiv:2406.09246。
- **引用**：≈ 1500–2000。
- **贡献**：7B VLA = Llama-2-7B + DINOv2+SigLIP + 256-bin 离散动作 token；OXE 970k 微调；跨 29 任务比 55B RT-2-X 高 16.5%；LoRA + INT4/8 量化消费级可跑。
- **代码**：https://github.com/openvla/openvla ；权重 openvla/openvla-7b。
- **谱系位置**：RT-2 开源版事实标准；OpenVLA-OFT 2025 加入 L1 连续回归。**复合/150_Mobile_ALOHA与UMI 精读。**

#### 24. Brohan/Zitkovich et al. 2023 — RT-2
- *RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control*, CoRL 2023. arXiv:2307.15818。
- **引用**：≈ 2500–3500。
- **贡献**：首次命名 "VLA"；PaLI-X/PaLM-E 12B/55B 上 co-fine-tune 离散动作 token 与网页 VQA；语义 CoT 选工具等涌现能力。
- **代码**：权重未公开；第三方 kyegomez/RT-2 复现。
- **谱系位置**：VLA 奠基作；催生 RT-2-X、OpenVLA、π0。

#### 25. Open X-Embodiment Collaboration 2024 — OXE / RT-X
- *Open X-Embodiment: Robotic Learning Datasets and RT-X Models*, ICRA 2024 Best Paper. arXiv:2310.08864。
- **引用**：≈ 2000–3000。
- **贡献**：22 机器人、21 机构、60 数据集、>1M 轨迹 RLDS 统一；RT-1-X/RT-2-X 跨本体 co-training 正迁移（小域+50%）。
- **代码**：https://github.com/google-deepmind/open_x_embodiment ；RT-1-X 开源权重。
- **谱系位置**：所有现代 VLA 预训练的数据基础设施。**复合/150_Mobile_ALOHA与UMI 精读。**

#### 26. NVIDIA GEAR 2025 — GR00T N1 / N1.5
- *GR00T N1: An Open Foundation Model for Generalist Humanoid Robots*, arXiv:2503.14734；N1.5 为 2025.5 NVIDIA Research 博客。
- **引用**：≈ 150–300 / N1.5 未确认。
- **贡献**：dual-system（S2 Eagle-2 VLM + S1 DiT flow matching）；数据金字塔（真机+人视频+Omniverse 合成）；2.2B 参数 L40 上 63.9 ms；N1.5 升级 Eagle 2.5 冻结 + FLARE（未来潜在对齐）+ DreamGen 世界模型合成轨迹，12 新动词零样本 13.1%→38.3%。
- **代码**：https://github.com/NVIDIA/Isaac-GR00T ；权重 huggingface.co/nvidia/GR00T-N1-2B（Apache-2.0）。
- **谱系位置**：Helix 后首个开源 dual-system VLA；人形 foundation model 锚点。

#### 27. Figure AI 2025–2026 — Helix 01 / 02
- 仅公司博客：figure.ai/news/helix, figure.ai/news/helix-02。**无 arXiv 论文**。
- **引用**：N/A（非学术出版）。
- **贡献**：Helix 01 = S2 (~7B VLM @ 7–9 Hz) + S1 (~80M visuomotor @ 200 Hz)，35 DoF 上身，500 h 遥操作；**Helix 02** 扩到 S0（1 kHz 全身运动先验，1000+ h 人类动作数据替代十万行 C++）+ pixels-to-whole-body + 掌心相机 + 指尖触觉，首次 humanoid 连续 4 分钟 61 步 loco-manipulation（装卸洗碗机）。
- **代码**：完全闭源。
- **谱系位置**：工业闭源双系统 VLA 标杆；早于 GR00T N1 一个月部署。

#### 28. Octo Model Team 2024 — Octo
- *Octo: An Open-Source Generalist Robot Policy*, RSS 2024. arXiv:2405.12213。
- **引用**：≈ 700–1000。
- **贡献**：Transformer + diffusion head，OXE 25 数据集 800k 轨迹预训练；灵活 tokenization 支持任意相机/本体/语言-图像条件；Octo-Small 27M、Octo-Base 93M。
- **代码**：https://github.com/octo-models/octo 。
- **谱系位置**：跨本体开源 generalist policy 奠基；π0/RDT/OpenVLA 对标基线。

#### 29. Liu et al. 2025 — RDT-1B
- *RDT-1B: a Diffusion Foundation Model for Bimanual Manipulation*, ICLR 2025. arXiv:2410.07864。
- **引用**：≈ 200–400。
- **贡献**：1.2B DiT 双臂扩散基础模型；Physically Interpretable Unified Action Space 统一跨机器人；46 数据集 1M+ episodes 预训练 + 6k 自建双臂微调；ALOHA 1–5 demo 学新技能相对 +56%。
- **代码**：https://github.com/thu-ml/RoboticsDiffusionTransformer 。
- **谱系位置**：Octo 双臂大规模版，中文学界 VLA 代表作；GR00T N1 的 DiT action head 直接参考。

#### 30. Liu et al. 2024 — OK-Robot
- *OK-Robot: What Really Matters in Integrating Open-Knowledge Models for Robotics*, NYU+Meta. arXiv:2401.12202. CoRL 2024 workshop。
- **引用**：≈ 150–250。
- **贡献**：零训练系统级整合——OWL-ViT/Detic + VoxelMap + AnyGrasp + Stretch 原语；10 家庭 58.5% pick-and-drop，清洁 82%。
- **代码**：https://github.com/ok-robot/ok-robot 。
- **谱系位置**：模块化 open-knowledge 路线，与 VLA 端到端形成对照。

#### 31. Yenamandra et al. 2023 — HomeRobot OVMM
- *HomeRobot: Open-Vocabulary Mobile Manipulation*, CoRL 2023 PMLR v229:1975–2011。arXiv:2306.11565。
- **引用**：≈ 200–350。
- **贡献**：定义 OVMM 任务（未见家中把任意物体从 start 到 goal receptacle）；Habitat 仿真 + Stretch 真机双栈；NeurIPS 2023 竞赛基准。
- **代码**：https://github.com/facebookresearch/home-robot 。
- **谱系位置**：开放词汇 MM 的奠基基准。

#### 32. Nasiriany et al. 2024 — RoboCasa
- *RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots*, RSS 2024. arXiv:2406.02523。
- **引用**：≈ 150–300。
- **贡献**：MuJoCo/RoboSuite 厨房程序化生成 120 场景×10×10；2509 物体（text-to-3D 增强）；LLM 代码化 100 任务；MimicGen 轨迹扩充；真机 +38%；2026 扩为 RoboCasa365（365 任务）。
- **代码**：https://github.com/robocasa/robocasa 。
- **谱系位置**：程序化+生成式仿真数据代表；GR00T 评测核心。

---

## 二、核心开源项目清单（11 个）

| # | 项目 | Star (2026.4) | 许可证 | ROS | 阅读时间 (概/精/完) |
|---|---|---|---|---|---|
| 1 | Physical-Intelligence/openpi | 10.2k | Apache-2.0 | — | 2 / 10 / 35 h |
| 2 | huggingface/lerobot | 22.6k | Apache-2.0 | — | 3 / 15 / 50 h |
| 3 | MarkFzp/mobile-aloha + tonyzhaozh/aloha | 4.4k + 2.2k | MIT | ROS 1 Noetic | 1.5 / 6 / 15 h |
| 4 | facebookresearch/home-robot | ~1.2k | MIT | ROS 1 Noetic (hw) | 2 / 10 / 30 h |
| 5 | hello-robot/stretch_ai + stretch_ros2 | 216 + 109 | Apache-2.0 混合 | ROS 2 Humble | 3.5 / 14 / 35 h |
| 6 | leggedrobotics/ocs2（mobile_manipulator） | ~1.1k | BSD-3 | ROS 1 Noetic | 4 / 20 / 60 h |
| 7 | moveit/moveit2 + moveit_task_constructor | ~2.2k + ~0.7k | BSD-3 | ROS 2 | 3 / 15 / 50 h |
| 8 | ros-navigation/navigation2（nav2_mppi） | ~2.7k | Apache-2.0 | ROS 2 | 2 / 10 / 30 h |
| 9 | real-stanford/universal_manipulation_interface (UMI) | ~1.4k | MIT | — | 2 / 8 / 25 h |
| 10 | robocasa/robocasa | ~900 | MIT | — | 2 / 8 / 20 h |

> Star 标"~"为搜索快照估计。

### 2.1 openpi（π0 / π0-FAST / π0.5 基座）
- 关键目录：`src/openpi/{models,models_pytorch,policies,training,serving}`, `scripts/{train,serve_policy,compute_norm_stats}.py`, `packages/openpi-client/`。
- 关键文件：`training/config.py`（TrainConfig 中心）、`policies/policy_config.py`（`create_trained_policy` 工厂）、`models/pi0.py`（flow matching VLA 网络）、`serve_policy.py`（WebSocket 推理服务）。
- 依赖：JAX/Flax（主）+ PyTorch + PaliGemma + LeRobot + transformers 4.53.2（需补丁）。

### 2.2 lerobot
- 关键目录：`src/lerobot/{datasets,policies,robots,teleoperators,envs,scripts}/`。
- 关键文件：`datasets/lerobot_dataset.py`（Parquet+MP4 统一格式）、`robots/robot.py`（connect/get_observation/send_action 基类）、`policies/act/modeling_act.py`、`policies/pi0/`、`policies/pi05/`、`scripts/train.py`、`scripts/eval.py`。
- 依赖：PyTorch + TorchCodec + HF Hub + gymnasium + feetech/dynamixel-sdk。
- ICLR 2026 接受；覆盖 SO-100/101、Koch、LeKiwi、Reachy2、Unitree G1、ALOHA。

### 2.3 Mobile ALOHA
- `aloha_scripts/{one_side_teleop,record_episodes,replay_episodes,visualize_episodes,constants}.py` + `launch/4arms_teleop.launch` + AgileX Tracer CAN 配置（`gs_usb` 内核模块 + pyagxrobots）。
- ROS 1 Noetic + Interbotix SDK + MuJoCo 2.3.7 + dm_control 1.0.14。
- 硬件 I/O 与采集栈；训练算法外迁到 act-plus-plus / LeRobot。

### 2.4 facebookresearch/home-robot
- `src/home_robot/{core,mapping,navigation,manipulation,perception}/` + `src/home_robot_hw/`（Stretch ROS 节点） + `src/home_robot_sim/`（Habitat 封装） + `projects/{habitat_ovmm,real_world_ovmm,slap_manipulation}/`。
- `core/interfaces.py`（Observations/Action）、`core/abstract_agent.py`、`home_robot_hw/nodes/simple_grasp_server.py`、SLAP 的 IPM/APM 两阶段策略。
- 依赖：habitat-lab/sim v0.2.5 + Detic + Grounded-SAM + Contact-GraspNet + Pinocchio。
- OVMM 官方挑战赛基线；2024 末节奏放缓，社区事实接棒者为 stretch_ai。

### 2.5 stretch_ai + stretch_ros2
- **stretch_ai**：`src/stretch/{agent,app,mapping/instance,perception,navigation,motion,llms}/` + `src/stretch_ros2_bridge/`。入口 `app/ai_pickup.py`、`app/dex_teleop/ros2_leader.py`；`agent/operations/pickup_executor.py`；`mapping/instance/core.py`（Instance/InstanceView 3D 开词汇记忆）。
- **stretch_ros2**：`stretch_core/stretch_core/stretch_driver.py`（核心驱动 + mode/action）、`stretch_nav2/launch/navigation.launch.py`、`stretch_funmap/stretch_funmap/funmap.py`、`stretch_description/urdf/stretch.urdf`。
- ROS 2 Humble + Nav2 + slam_toolbox + rerun.io + LeRobot fork + OpenAI/Qwen2.5-3B。
- OVMM 事实参考实现。

### 2.6 leggedrobotics/ocs2（MM 核心）
顶层：`ocs2_core`、`ocs2_oc`、`ocs2_ddp`、`ocs2_mpc`、`ocs2_sqp`、`ocs2_slp`、`ocs2_ipm`、`ocs2_pinocchio/*`、`ocs2_self_collision`、`ocs2_ros_interfaces`、`ocs2_python_interface`、`ocs2_mpcnet`、`ocs2_robotic_examples/`。

**重点：`ocs2_robotic_examples/ocs2_mobile_manipulator/`**
- `src/MobileManipulatorInterface.cpp` — OCP 构造中心：解析 URDF+`task.info`，注册 Cost/Constraint，创建 rollout 与 solver。
- `src/dynamics/WheelBasedMobileManipulatorDynamics.cpp` — 差速底盘运动学流 map $\dot x_b=v\cos\theta,\dot y_b=v\sin\theta,\dot\theta=\omega,\dot q_a=\dot q_a$。
- `src/dynamics/DefaultManipulatorDynamics.cpp` — 全向（SE(3) dummy）版本。
- `src/constraint/EndEffectorConstraint.cpp` — SE(3) 末端位姿约束（soft/ hard）。
- `src/constraint/JointVelocityLimits.cpp` — relaxed barrier 关节限位。
- `src/cost/QuadraticInputCost.cpp` + `cost/EndEffectorCost.cpp` — 代价组装（由 QuadraticPenalty 包装的末端误差）。
- `src/ManipulatorModelInfo.cpp` — `ManipulatorModelType` 枚举：0=DefaultManipulator、1=**WheelBasedMobileManipulator**、2=FloatingArm、3=FullyActuatedFloatingArmManipulator。
- `config/franka/task.info`、`config/ridgeback_ur5/task.info` — 权重与 horizon 配置。
- 依赖 `ocs2_self_collision`（HPP-FCL）、`ocs2_pinocchio_interface`、`ocs2_sqp` + HPIPM。
- **本课程 复合/120_底盘臂联合规划 核心精读对象。**

### 2.7 MoveIt2 + moveit_task_constructor
- **moveit2** 顶层：`moveit_core`、`moveit_ros`（含 `moveit_servo`）、`moveit_kinematics`、`moveit_planners`（OMPL/Chomp/Pilz）。
- **moveit_servo**：`moveit_ros/moveit_servo/src/{servo_node,servo_calcs}.cpp` — 实时伺服（低延迟 twist/joint jog → trajectory），适合视觉伺服 + MPC closed loop。
- **moveit_task_constructor**（ICRA 2019, Görner/Haschke）顶层：`core/{include,src}/moveit/task_constructor/`。
  - `core/include/moveit/task_constructor/stages/compute_ik.h` — `ComputeIK` wrapper stage（包装 pose generator 计算 IK）。
  - `core/src/stages/generate_grasp_pose.cpp` — `GenerateGraspPose`（围绕物体按 angle_delta 圆周生成抓取位姿）。
  - `core/src/stages/generate_place_pose.cpp` — `GeneratePlacePose`。
  - `core/src/stages/move_to.cpp` / `move_relative.cpp` — Cartesian 与 joint 空间运动。
  - `core/src/stages/connect.cpp` — 连接两个生成器解（由内向外规划）。
  - `core/src/stages/modify_planning_scene.cpp`、`fix_collision_objects.cpp` — 场景修改 / 修复。
  - `core/src/container.cpp` — `SerialContainer` / `ParallelAlternatives` 层级容器。
- Stage 三类：**generator**（CurrentState、GenerateGraspPose）、**propagator**（MoveRelative、MoveTo、ModifyPlanningScene）、**connector**（Connect）。从内向外规划。
- **本课程 复合/130_OCS2_mobile_manipulator 核心精读对象。**

### 2.8 ros-navigation/navigation2（nav2_mppi）
- 根目录：`nav2_behavior_tree`、`nav2_planner`、`nav2_controller`、`nav2_mppi_controller`、`nav2_smac_planner`、`nav2_collision_monitor`、`nav2_costmap_2d`、`nav2_amcl`、`nav2_behaviors`。
- **`nav2_mppi_controller/`** 关键源：
  - `src/controller.cpp`（`MPPIController` 实现 `nav2_core::Controller`）。
  - `src/optimizer.cpp`（核心 MPPI 优化器——时间步 56、batch_size 2000、temperature 0.3、gamma 0.015、iteration 1）。
  - `src/motion_models.cpp`（DiffDrive / Omni / Ackermann）。
  - `src/critics/*.cpp`（ConstraintCritic、CostCritic、ObstaclesCritic、GoalCritic、GoalAngleCritic、PathAlignCritic、PathFollowCritic、PathAngleCritic、PreferForwardCritic、TwirlingCritic）。
  - `src/trajectory_validator/default_optimal_trajectory_validator.cpp`。
  - `src/noise_generator.cpp`（Gaussian / Log-Normal 实验）。
- AVX2 + MFMA CPU 向量化，Intel i5 上 50–100 Hz；plugin 接入 `controller_server`。
- **本课程 复合/130_OCS2_mobile_manipulator 精读辅助。**

### 2.9 UMI（real-stanford/universal_manipulation_interface）
- 顶层：`umi/real_world/` + `umi/common/` + `diffusion_policy/`（submodule 复用）。
- 关键文件：
  - `umi/real_world/bimanual_umi_env.py` — 双臂 UMI 采集环境 (UR5/Franka + 多 UVC)。
  - `umi/real_world/franka_interpolation_controller.py`、`umi/real_world/rtde_interpolation_controller.py` — 机械臂插值控制。
  - `umi/real_world/multi_uvc_camera.py` — 多相机同步。
  - `umi/common/pose_trajectory_interpolator.py` — 相对轨迹插值。
  - `umi/common/usb_util.py`（reset_all_elgato_devices）。
  - `run_slam_pipeline.py` — ORB-SLAM3（Urban fork）处理 GoPro 视频。
  - `eval_real.py` — 真机推理（spacemouse + C/S 热键）。
- 依赖：ORB-SLAM3（GoPro SLAM pipeline）、diffusion_policy、zerorpc（Franka interface）、OpenImuCameraCalibrator、accelerate、GoPro + Mirrorless Gripper 硬件。
- **本课程 复合/140_VLA移动操作 精读。**

### 2.10 robocasa/robocasa
- 基于 MuJoCo/RoboSuite 的 120 厨房、2509 物体、100+ 任务仿真。2026 年 ICLR 版 RoboCasa365（365 任务，+ 600 h 人类 demo + 1600 h 合成）。
- GR00T N1 / N1.5 评测平台。
- 阅读时间：概览 2 h / 精读 8 h / 完整 20 h。

---

## 三、数学与算法深入

### 3.1 SE(2) 底盘运动学与臂耦合

**差速 (unicycle)**：$q_b=(x,y,\theta)$，$u_b=(v,\omega)^\top$，
$$\dot q_b = S(\theta)u_b = \begin{bmatrix}\cos\theta & 0\\ \sin\theta & 0\\ 0 & 1\end{bmatrix}u_b,\quad \dot x\sin\theta-\dot y\cos\theta=0\ \text{(Pfaffian)}$$

**全向 (mecanum)**：$u_b=(v_x^B,v_y^B,\omega)$，无 Pfaffian 约束，完整系统。麦克纳姆轮逆：$\dot\phi_i=\frac1r(v_x^B\mp v_y^B\mp(l_x+l_y)\omega)$。

**Ackermann**：$(x,y,\theta,\phi)$，$\dot\theta=\frac{v}{L}\tan\phi$，曲率约束 $|\kappa|\le\kappa_{\max}$。

**Pinocchio 建模**：全向用 `JointModelPlanar`；差速/Ackermann 用 planar + Pfaffian 约束注入 OCP；腿+臂用 `JointModelFreeFlyer`。OCS2 `ocs2_mobile_manipulator` 的 `ManipulatorModelType=1 (WheelBasedMobileManipulator)` 对应差速，`=3` 对应全向 SE(3) dummy。

### 3.2 臂-底盘联合雅可比

令 $V_{ee}^w\in\mathbb R^6$，$\nu=[u_b;\dot q_a]$。世界系 twist：
$$V_{ee}^w=V_{wb}^w+\mathrm{Ad}_{T_{wb}}V_{be}^b=J_b(q)u_b+J_a(q)\dot q_a=J(q)\nu$$

对差速底盘，令 $p_{be}^w=R_{wb}p_{be}^b$：
$$J_b=\begin{bmatrix}R_{wb}e_x & -[p_{be}^w]_\times e_z\\ \mathbf 0 & e_z\end{bmatrix}\in\mathbb R^{6\times2}$$

全向底盘多一列 $R_{wb}e_y$ 及对应耦合项，$J_b\in\mathbb R^{6\times3}$。

**冗余度** $r=m_b+n_a-6$，7-dof 臂+差速 r=3、+全向 r=4。阻尼伪逆 + 零空间：
$$\nu^\star=J^+\dot x_{ee}^d+(I-J^+J)\nu_0,\quad J^+=J^\top(JJ^\top+\lambda^2I)^{-1}$$

**非完整底盘处理**：直接用降维 $J_b\in\mathbb R^{6\times m_b}$；或选择矩阵 $H\in\mathbb R^{m_b\times 3}$ 投影 $u_b=H[v,\omega]^\top$。策略层面"手先动、基后动"通过零空间正则 $\nu_0=-k_b[\mathbf 1_{m_b},0]$ 实现。

### 3.3 OCS2 `mobile_manipulator` OCP 构造

**状态/输入**（见 `WheelBasedMobileManipulatorDynamics.cpp`）：
$$x=\begin{bmatrix}x_b\\ y_b\\ \theta_b\\ q_a\end{bmatrix}\in\mathbb R^{3+n_a},\quad u=\begin{bmatrix}v\\ \omega\\ \dot q_a\end{bmatrix}\in\mathbb R^{2+n_a},\quad \dot x=\begin{bmatrix}v\cos\theta_b\\ v\sin\theta_b\\ \omega\\ \dot q_a\end{bmatrix}$$

**代价**：末端误差 $e_{ee}(q)=\log_{SE(3)}(T_{we}^{d-1}T_{we}(q))^\vee$，
$$\ell=\tfrac12 e_{ee}^\top Q_{ee}e_{ee}+\tfrac12 u^\top Ru+\tfrac12(q_a-q_a^{reg})^\top Q_a(q_a-q_a^{reg})$$

典型 $Q_{ee}=\mathrm{diag}(100,100,100,20,20,20)$，$R_v=R_\omega=1$，$R_{\dot q_a}=0.1$。由 `EndEffectorConstraint + QuadraticPenalty` 实现 soft-constrained。

**约束**：
1. 关节限位 relaxed barrier；
2. 自碰撞 `ocs2_self_collision` + HPP-FCL，$h_{ij}(q)=d_{ij}(q)-d_{\min}\ge0$，$\nabla_q d_{ij}=\hat n_{ij}^\top(J_{p,i}-J_{p,j})$；
3. 外部障碍 SDF（Pankert 2020）：ESDF $\Phi(p)$，包络球 $h_k=\Phi(p_k(q))-r_k\ge0$，$\nabla_q h_k=\nabla\Phi^\top J_{p_k}$。ESDF 经 `SolverSynchronizedModule` 同步，避 race。

**求解器**：默认 SQP（`ocs2_sqp`），shooting 段 0.1 s、horizon 2 s；QP 用 HPIPM：Riccati 递推 $O(N(n+m)^3)$ 线性 horizon。mobile manipulator 无接触切换，SLQ/iLQR 特别高效：(i) 状态 $\le10$ 廉价线性化；(ii) 冗余度让 Gauss-Newton Hessian 良态；(iii) 末端二次代价满足收敛条件。

```python
while rclpy.ok():
    x0 = state_estimator.latest()
    refMgr.setTargetTrajectories(ee_goal_traj)
    solver.run(t_now, x0, t_now + T_horizon)   # SQP+HPIPM ~5-20Hz
    policy = solver.getPolicy()                # x*(·), u*(·), K(·)
    mrt_buffer.publish(policy)                 # → 500Hz WBC
```

### 3.4 视觉伺服 + 底盘漂移

**IBVS** 交互矩阵：
$$L(s,Z)=\begin{bmatrix}-f/Z & 0 & u/Z & uv/f & -(f^2+u^2)/f & v\\ 0 & -f/Z & v/Z & (f^2+v^2)/f & -uv/f & -u\end{bmatrix}$$
控制律 $v_c=-\lambda L^+e$。

**PBVS** 误差 $e=[t_c-t_c^d;\ \log(R_c^{d\top}R_c)^\vee]$，$\dot e=L_{PB}v_c$，$L_{PB}=\mathrm{blkdiag}(-I,-J_\omega(\phi))$。

**底盘漂移** 进入任务误差 $\tilde e=e(\hat q)+J_b(q_b^{odom})\delta q_b+O(\|\delta q_b\|^2)$。IBVS 天然闭环补偿图像误差；若目标为 world-frame，需把 SLAM pose $T_{wb}^{SLAM}$ 作为参考重置，或 world 任务转为相机系 image feature。

### 3.5 TAMP：PDDL + PDDLStream

```
(:action pick :parameters (?o ?p ?g ?q ?t)
  :precondition (and (AtPose ?o ?p) (HandEmpty)
                     (Kin ?o ?p ?g ?q ?t) (AtConf ?q))
  :effect (and (Holding ?o ?g) (not (HandEmpty)) (not (AtPose ?o ?p))))

(:stream inverse-kinematics :inputs (?o ?p ?g)
  :domain (and (Pose ?o ?p) (Grasp ?o ?g))
  :outputs (?q ?t) :certified (and (Conf ?q) (Traj ?t) (Kin ?o ?p ?g ?q ?t)))
```

Optimistic adaptive planning：先假设 stream 返回满足 certified 谓词的对象求乐观骨架，lazy 采样具体值，失败则提高采样等级 $\ell$。MM streams：`sample-base-conf`（IRM 查表）、`move-base`（Hybrid A\*）、`ik-arm`、`collision-free`(test)。

### 3.6 FAST 动作 tokenization（数学细节）

问题：50 Hz 下 naive per-dim per-step binning 产生 $H\cdot d_a=350$ tokens/chunk，相邻 token 高度相关，cross-entropy 梯度塌陷。

**DCT-II**：
$$C_i[k]=\sum_{n=0}^{H-1}a_i[n]\cos\!\Bigl[\tfrac{\pi}{H}(n+\tfrac12)k\Bigr],\quad C=DA$$

机器人轨迹能量集中在低频 $k\ll H$。**量化**：$\hat C_{ik}=\mathrm{round}(C_{ik}/s)$，$s$ 按训练集 95% 分位取，丢弃 $k>K$（$K\approx5$–10）。**Column-first 展平 + BPE**：贪心合并 bigram 到词表 $|V|\sim1024$。

```python
def fast_decode(tokens, bpe, K, H, d_a, s):
    ints = bpe.decode(tokens)
    C_hat = zeros(H, d_a)
    C_hat[:K,:] = unflatten_column_first(ints, K, d_a) * s
    return idct(C_hat, type=2, norm='ortho', axis=0)
```

| 方案 | tokens/chunk(50步7维) | 推理步数 |
|---|---|---|
| Naive per-step | 350 | 350 |
| DCT+quant 无BPE | ~80 | 80 |
| **FAST** | **30–60** | **30–60** |

RT-2/OpenVLA 的 per-step 方案 50 Hz → 350 tokens/s 自回归 ≈750 ms 无法实时；FAST <10 tokens/s。

### 3.7 Flow-matching 动作头（π0）

Conditional Flow Matching：OT path $A_\tau=\tau A_1+(1-\tau)A_0$，$A_0\sim\mathcal N(0,I)$，$A_1\sim p_{data}$。目标 $u_\tau=A_1-A_0$。损失：
$$\mathcal L_{CFM}=\mathbb E_{\tau,o,A_1,A_0}\|v_\theta(A_\tau,\tau,o)-(A_1-A_0)\|^2$$

推理 ODE Euler $N=10$ 步：
```python
def pi0_sample(o, N=10):
    A = torch.randn(H, d_a)
    for k in range(N):
        tau = k/N
        A = A + (1/N) * v_theta(A, tau, o)
    return A
```

与 DDPM 差别：学速度而非 score，OT path 训练 target 是常向量方差小；10 步 ODE vs DDIM 50 步。

**π0 架构**：PaliGemma-3B (SigLIP-400M + Gemma-2.6B) + 300M action expert；block-wise 因果掩码（VLM 块自注意、proprio 块注意 VLM+proprio、action 块注意全部）；推理 1 次 VLM 前向缓存 KV + 10 次 action expert 前向。

### 3.8 抓取候选生成

**力闭合 (Ferrari & Canny 1992)**：接触点 $c_i$ 法向 $n_i$，$k$ 边摩擦锥基 $f_{i,j}$，GWS：
$$\mathcal W=\mathrm{ConvHull}\{(f_{i,j},c_i\times f_{i,j})\}\subset\mathbb R^6$$
力闭合：原点 ∈ int($\mathcal W$)；质量 $Q_{FC}=\min_{w\in\partial\mathcal W}\|w\|$。

**Contact-GraspNet (ICRA 2021)**：把每观测点 $c_i\in P$ 作为 gripper contact，6-DoF grasp 降到 4-DoF ($a_i\in S^2$, $b_i\in S^1$, $w_i\in\mathbb R_+$)，$R_i=[b_i,a_i\times b_i,a_i]$。PointNet++ per-point 输出 $(s_i,a_i,b_i,w_i)$；ACRONYM 17M grasps 训练；未见物体 >90%。

**AnyGrasp (T-RO 2023)**：7-DoF + graspness-aware + CoM 感知 + 时序平滑，900+ MPPH bin picking 93.3%。

**Base placement 耦合**：grasp 候选 $\mathcal G=\{(T_g^w,s_g)\}$，底盘停车：
$$\max_{q_b,g}s_g\cdot\rho(T_g^w,q_b)\ \text{s.t.}\ \mathrm{IK}\ \text{可行},\ q_b\in\mathcal F_{nav}$$
$\rho$ 为 inverse reachability map（IRM，离线枚举臂末端位姿可达性），作为 `sample-base-conf` stream。

### 3.9 MPC-WBC 分层架构

| 层 | 频率 | 输出 | 求解器 |
|---|---|---|---|
| MPC（OCS2） | 5–50 Hz | $x^\star, u^\star, K$ | SQP+HPIPM / SLQ |
| WBC（TSID） | 500–1000 Hz | $\tau^\star, \ddot q^\star, f_c^\star$ | OSQP / proxQP |

TSID QP：
$$\min_{\ddot q,\tau,f_c}\sum_i w_i\|J_i\ddot q+\dot J_i\dot q-\ddot x_i^d\|^2\ \text{s.t.}\ M\ddot q+h=S^\top\tau+J_c^\top f_c,\ f_c\in\mathcal K_{fric},\ \tau^-\le\tau\le\tau^+$$

层级：Level1 动力学+接触 > Level2 末端 > Level3 底盘 > Level4 臂 regularization。严格级联 HQP (Kanoun 2011)：$\ddot q_{k+1}^\star\in\arg\min_{\ddot q\in\mathcal N_k}\|A_k\ddot q-b_k\|^2$。实际常用 weighted QP 近似。

```python
# 500 Hz WBC loop
while True:
    q, qd = robot.state()
    x_ref, u_ref, K = mrt.interpolate(t_now)
    T_ee_ref, V_ee_ref = fk_and_twist(x_ref, u_ref)
    A_ee_ref = feedback_pd(...) + A_ee_ff
    tasks = [Dynamics(q,qd), ContactConstr(q),
             EETask(A_ee_ref,w=1e2), BaseTask(base_ref,w=1e1),
             PostureTask(q_ref,w=1e0)]
    tau, qdd = tsid.solve(tasks)
    robot.send_torque(tau)
```

MPC 输出 $q_a^{MPC}$ 作为低优先级 regularization（不直接 PD，避与末端冲突）；**末端轨迹**才是 WBC 主任务，与 Minniti 2019 / Pankert 2020 / Sleiman 2021 一致。

---

## 四、硬件平台详解

| 平台 | 底盘 | 速度 | 臂 DOF | 每臂负载 | 价格 USD |
|---|---|---|---|---|---|
| **Stretch 3** | 2 轮差速 | ~0.6–1 m/s | 棱柱 6–7 + 夹爪 | 1.5 kg | **$24,950** |
| **Mobile ALOHA DIY** | Tracer 差速 | 1.6 m/s | 2 × 6 + 夹爪 | 0.75 kg | **~$32,000** |
| **TIAGo++ / Pro** | 差速 或 Mecanum | 1.5 m/s | 2 × 7 + 夹爪 | 3 kg | **$100–200 k+** |
| **Ridgeback + Panda** | Mecanum 全向 | 1.1 m/s | 1(-2) × 7 + 夹爪 | 3 kg | **$55–75 k** |
| **Husky + UR5/UR10** | 4 轮差速 | 2.0 m/s (A300) | 1 × 6 + 夹爪 | 5 / 10 kg | **$55–80 k** |
| **Fetch** (停产) | 2 轮差速 | 1.0 m/s | 1 × 7 + 夹爪 | 6 kg | $70–100 k |
| **Toyota HSR** | 3 全向轮 | 0.36–0.8 m/s | 1 × 4 + 升降+头 | 1.2 kg | 仅借用 |
| **Galaxea R1 / R1 Pro** | 3 轮转向 | 2.5 m/s² | 2 × 6 / 7 + 4 躯干 | 5–7 kg | **$27–44.5 k / $69,999** |
| **AgileX Cobot Magic** | Tracer 差速 | 1.6 m/s | 2 × 6 + 夹爪 | 0.75–1.5 kg | **~$30–50 k** |

**关键细节**：
- **Stretch 3**：24.5 kg 轻量，RealSense D435if+D405，RPLidar A1，Intel NUC 12 i5；ROS 2 Humble 原生；OK-Robot/HomeRobot 标配；棱柱臂无关节灵巧性但安全易用。
- **Mobile ALOHA**：Logitech C922 × 3、无 LiDAR、i7-12800H + RTX 3070Ti Mobile；ROS 1 Noetic。
- **TIAGo++/Pro**：伸缩躯干 35 cm（高 1.1–1.45 m）、Hey5 软手可选、可配 6 轴 FT；Robocup@Home 与 ERL-SR 事实标准。Pro 版 EtherCAT + 360° LiDAR。
- **Ridgeback+Panda**：ETH \"RoyalPanda\" 即此配置；Panda 关节力矩传感器 7 关节全。
- **Husky+UR5/10**：户外 IP54、30° 坡；A300 有 brushless 轮毂、2 m/s；UR 工业臂非力控但鲁棒。
- **HSR**：3 全向轮 + 升降 + 4 DoF 臂，8 DoF 操作；仅贷借给 HSR Developers Community 成员。
- **Galaxea R1**：ZED 2 + 2×ZED-Mini + IMU，3 轮转向独立转向+驱动；R1 Pro 26 DoF；Stanford BRS、Physical Intelligence 已采用；G0 VLA 开源。
- **Cobot Magic**：Stanford Mobile ALOHA 代码直跑；openpi-agilex 支持。

**趋势（2024–2026）**：双臂+全向+躯干躯成新标准（Mobile ALOHA、Galaxea R1、TIAGo Pro）；Stretch 3 + Galaxea R1 + Mobile ALOHA 是学术增长最快三平台。

---

## 五、博士研究开放问题（7 个）

1. **长时程杂乱操作的鲁棒恢复**：20+ 子任务连串中任一失败的检测、诊断、回滚与重规划。现状：π0.5 单次 10–15 min 清洁，但失败恢复靠隐式策略；缺显式的 belief-space POMDP + anomaly detector + replanner 三合一框架。可从 PDDLStream + VLA 低层 + LLM 反思形成分层闭环。

2. **移动底盘上的不确定性感知抓取**：底盘 odom 漂移 $\Sigma_b$、SLAM 地图协方差 $\Sigma_m$、grasp 点云噪声 $\Sigma_g$ 如何传入抓取质量 $Q_{FC}(\mu,\Sigma)$ 并与 base placement 联合优化。与 Chance-Constrained MPC 结合，是 OCS2 Constraint 层的自然扩展。

3. **跨具身 VLA 迁移（轮式臂↔足式臂）**：OXE 已证轮臂间正迁移，但动作空间维度、接触模态、动力学差异使足式臂迁移不稳。RDT-1B 的 Physically Interpretable Unified Action Space 是早期尝试；FLARE（N1.5）+ DreamGen 合成轨迹值得深挖。

4. **欠驱动棱柱臂的实时全身 MPC**：Stretch 3 的 lift+telescope+wrist 导致雅可比奇异面密集，传统 OCP 的 Hessian 病态；需 manifold-aware SQP 或 constraint-regularized iLQR。目前开源 OCS2 尚无 Stretch 配置。

5. **SLAM-操作耦合**：被操作物体同时是视觉地标时，操作前-操作中-操作后的地图更新与姿态修正需联合推断。当前 ORB-SLAM3 把动点视为 outlier 丢弃；需主动式 SLAM + 物体级 factor graph。

6. **VLA + 形式化安全包络**：flow matching/diffusion 生成的动作无可证约束。CBF（Control Barrier Function）+ VLA 的 runtime shielding 或 safety projection 是方向；需把 ODE 积分每步投影到 safe set，但这会破坏条件流场的概率解释。

7. **家庭场景长期部署的持续学习**：user-specific 偏好（TidyBot 思路）、环境分布漂移、灾难性遗忘。LoRA + replay buffer + EWC 在 MM 的系统性研究仍稀缺；π0.5 的 co-training 范式是工程路径而非理论保障。

---

## 六、复合/120_底盘臂联合规划–85 章节详细大纲

### **复合/120_底盘臂联合规划 OCS2 mobile_manipulator 深度精读 — SQP + HPIPM 的底盘 + 臂统一 NMPC**

**教学目标**：(1) 完整推导差速 / 全向底盘 + n-DoF 臂的联合 Jacobian 与动力学；(2) 阅读并讲解 `ocs2_mobile_manipulator` 所有 C++ 文件的 OCP 组装；(3) 独立修改 `task.info` 切换 Kinova / Ridgeback-UR5 / Franka 配置；(4) 把 `perceptive_mpc` 的 SDF 约束移植到当前项目；(5) 与已学腿足 OCS2 代码对比差异。

**子节**：
- 82.1 SE(2) 底盘运动学与 Pinocchio 建模（回顾）
- 82.2 联合 Jacobian 与冗余度、零空间任务优先级
- 82.3 `ManipulatorModelType` 四类枚举 + URDF 解析
- 82.4 `WheelBasedMobileManipulatorDynamics.cpp` 逐行
- 82.5 `EndEffectorConstraint` + QuadraticPenalty 的 SE(3) log 误差
- 82.6 `ocs2_self_collision` + HPP-FCL + Pinocchio Jacobian 链接
- 82.7 Pankert 2020 的 ESDF 约束（需移植 `grid_map` → `ocs2_esdf`）
- 82.8 SQP + HPIPM：shooting、Riccati block-tridiag、与 SLQ 对比
- 82.9 与 `MPCnet` 的结合：monte-carlo rollout → 模仿学习 policy net
- 82.10 Minniti 2019 ballbot 对比：动态 MPC vs 运动学 MPC

**指定论文**：Minniti 2019、Pankert & Hutter 2020、Sleiman 2021、Giftthaler 2017 (非完整约束 MPC)、Minniti 2021 (环境交互)。

**指定项目精读**：`leggedrobotics/ocs2` 的 `ocs2_robotic_examples/ocs2_mobile_manipulator/` 全部 + `perceptive_mpc` 的 `KinematicSimulation.cpp`。

**练习**：
- **A 编码**：(1) 为 Husky+UR5 写新的 `task.info` 与 URDF 资产；(2) 实现一个自定义 `EndEffectorPathConstraint`（圆弧跟踪）；(3) 用 ROS 2 port 把 `mobile_manipulator_ros` 从 ROS 1 迁移到 humble。
- **B 源码阅读**：(1) 画出 `MobileManipulatorInterface::createConstraints` 的调用图；(2) 写 1500 字报告分析 SQP 与 SLQ 在 mobile manipulator 上收敛差异；(3) 精读 HPIPM 的 `d_ocp_qp_ipm_solve`。
- **思考题**：(1) 若 $n_a=7$ 而任务只要求位置 3-D，如何设计零空间次级任务让底盘最小移动？(2) Ackermann 底盘下 Pfaffian 约束如何写入 OCP？(3) 为什么 mobile manipulator 的 Hessian 天然良态但 SLQ 有时仍震荡？

**学习时间**：讲授 6 h + 实验 14 h + 阅读 10 h = **30 h**。

---

### **复合/130_OCS2_mobile_manipulator MoveIt2 + Nav2 集成 — Task Constructor, Servo, MPPI, 抓取管线**

**教学目标**：(1) 掌握 `moveit_task_constructor` 的 generator/propagator/connector 三类 stage 与 SerialContainer 组装；(2) 用 `moveit_servo` 实现低延迟视觉伺服；(3) 配置 Nav2 `nav2_mppi_controller` 的 10+ critic 并在 DiffDrive / Omni / Ackermann 间切换；(4) 把 Contact-GraspNet / AnyGrasp 点云抓取接入 Task Constructor 的 `GenerateGraspPose`；(5) TAMP 顶层用 PDDLStream 调度 MTC 子任务。

**子节**：
- 83.1 MoveIt2 架构回顾（PlanningScene、MoveGroup、OMPL/Chomp/Pilz）
- 83.2 Task Constructor 三类 stage、InterfaceState、MonitoringGenerator
- 83.3 `ComputeIK` wrapper、`GenerateGraspPose`（angle_delta）、`GeneratePlacePose`、`Connect`
- 83.4 Pick-and-Place 完整示例（Panda/Stretch）源码阅读
- 83.5 `moveit_servo`：twist/joint jog、碰撞预警、collision-aware IK
- 83.6 Nav2 架构：`behavior_tree` + `planner_server`（Smac Hybrid A\*）+ `controller_server`
- 83.7 `nav2_mppi_controller`：MPPI 算法、Gaussian/Log-Normal sampling、10+ critic 调参
- 83.8 DiffDrive/Omni/Ackermann `motion_model` + `AckermannConstraints.min_turning_r`
- 83.9 抓取感知管线：点云 → Contact-GraspNet/AnyGrasp → Grasp Candidate → IRM 查表 → `GenerateGraspPose`
- 83.10 PDDLStream 调度 MoveIt Task Constructor：stream = IK + motion plan

**指定论文**：Chitta 2012、Görner 2019 (MTC)、Berenson 2011 (CBiRRT)、Sundermeyer 2021 (Contact-GraspNet)、Fang 2023 (AnyGrasp)、Garrett 2020 (PDDLStream)、Williams 2016 (MPPI 原论文)、Nav2 MPPI ROSCon 2023 Macenski talk。

**指定项目精读**：`moveit/moveit_task_constructor/core/src/stages/*.cpp` (8 个核心 stage)、`moveit2/moveit_ros/moveit_servo/src/servo_calcs.cpp`、`ros-navigation/navigation2/nav2_mppi_controller/src/{optimizer,critics/*}.cpp`、`caelan/pddlstream/examples/pybullet/` 的 kitchen demo。

**练习**：
- **A 编码**：(1) 写一个 Stretch 3 的开抽屉 MTC 任务（MoveRelative + AttachObject + MoveRelative）；(2) 为 Husky+UR5 的 Ackermann 配置写 `nav2_mppi` yaml 并调 PathAlign/PathAngle/PreferForward 三 critic；(3) 把 AnyGrasp 的 6-DoF grasp 输出改成 MTC `GenerateGraspPose` 的 `setMonitoredStage` 可用 pose。
- **B 源码阅读**：(1) 画 MTC `SerialContainer` 的 inside-out 规划顺序图；(2) 比较 `MPPIController` 与 TEB、DWB 在反向+转向性能；(3) 阅读 `moveit_servo` 的 400 Hz 控制循环与 Collision Monitor 交互。
- **思考题**：(1) 为何 MTC 从内向外规划？(2) MPPI 在 $N=1$ iteration 下为何仍有效？batch_size 与 temperature 如何 trade-off？(3) MTC 无法直接调用 nav2 移动底盘，如何设计 `NavigateToBase` stage？

**学习时间**：讲授 5 h + 实验 15 h + 阅读 8 h = **28 h**。

---

### **复合/140_VLA移动操作 Mobile ALOHA 与双臂模仿学习 — Diffusion Policy, UMI 数据采集范式**

**教学目标**：(1) 理解 ACT 的 CVAE + action chunking 数学与实现；(2) 精读 Diffusion Policy 的 conditional U-Net 与 receding horizon；(3) 搭建 Mobile ALOHA（或 Cobot Magic）数据采集流水线，co-training 配方复现；(4) 用 UMI 采集非机器人环境数据，训跨平台策略；(5) 对比 ACT / Diffusion Policy / RDT-1B / π0 在同一双臂任务上的成功率。

**子节**：
- 84.1 ALOHA 硬件回顾：ViperX follower + WidowX leader 主从遥操作
- 84.2 ACT：CVAE 编码 + Transformer 解码 + k 步 action chunk 与 exposure bias 缓解
- 84.3 Mobile ALOHA：16-DoF 动作向量（14 臂 + 2 底盘）、co-training 50/50 数据混合
- 84.4 Diffusion Policy：conditional DDPM、1D U-Net 去噪器、EMA、horizon 16/8/8 切片
- 84.5 UMI：手持夹爪 + GoPro + ARKit pose + ORB-SLAM3 映射、相对轨迹动作表示、延迟匹配
- 84.6 LeRobot 数据格式（Parquet + MP4 + LeRobotDataset）统一管线
- 84.7 双臂 VLA 扩展：RDT-1B (1.2B DiT) 的 Physically Interpretable Unified Action Space
- 84.8 评测：ALOHA Unleashed、RoboCasa365、LIBERO 的 cross-benchmark
- 84.9 UMI on Legs / UMI on Air：跨具身 diffusion guidance
- 84.10 工程陷阱：时钟同步、USB bandwidth、DMP vs diffusion 的实时性

**指定论文**：Zhao 2023 (ACT)、Fu 2024 (Mobile ALOHA)、Chi 2023/2024 (Diffusion Policy)、Chi 2024 (UMI RSS)、Liu 2025 (RDT-1B ICLR)、Ha 2024 (UMI on Legs)。

**指定项目精读**：`tonyzhaozh/aloha` + `tonyzhaozh/act`、`MarkFzp/mobile-aloha` + `MarkFzp/act-plus-plus`、`real-stanford/diffusion_policy`、`real-stanford/universal_manipulation_interface`（特别是 `bimanual_umi_env.py`、`franka_interpolation_controller.py`、`run_slam_pipeline.py`），`huggingface/lerobot/src/lerobot/policies/{act,diffusion,pi0}/`。

**练习**：
- **A 编码**：(1) 在 Cobot Magic / Mobile ALOHA 上采集 50 条搬杯子 episode，训 ACT 与 Diffusion Policy 对比；(2) 用 UMI 手持夹爪采集 30 次 in-the-wild 演示，迁移到 UR5e；(3) 把 RDT-1B 的 Unified Action Space 加入 LeRobot policy factory。
- **B 源码阅读**：(1) 画 ACT 的 CVAE 训练图（encoder 只在训练时用，推理丢掉）；(2) 对比 Diffusion Policy U-Net 与 Transformer 去噪器在双臂 ALOHA 上性能差异；(3) 阅读 UMI 的 ORB-SLAM3 pipeline 与 latency matching 实现。
- **思考题**：(1) 为何 co-training 50/50 比 fine-tuning 更稳？(2) Diffusion Policy 的 receding horizon 中 H/To/Ta 三参数（obs 2, action 16, execute 8）如何影响稳定性？(3) UMI 手持夹爪的 action 如何与机器人 base frame 解耦？

**学习时间**：讲授 5 h + 实验 18 h + 阅读 10 h = **33 h**。

---

### **复合/150_Mobile_ALOHA与UMI VLA 用于移动操作 — OpenVLA, π0/π0.5, FAST tokenization, 边缘部署**

**教学目标**：(1) 深入理解 VLA 的两大路线——离散 token 自回归 (RT-2/OpenVLA) vs 连续 flow matching (π0/π0.5)；(2) 推导 FAST 的 DCT+BPE 数学细节并复现 tokenizer；(3) 用 openpi 在 Mobile ALOHA / DROID 上微调 π0.5；(4) 把 OpenVLA-7B 用 LoRA + INT4 量化部署到 Jetson Orin；(5) 对比 GR00T N1.5 dual-system 与 Helix 02 三层架构。

**子节**：
- 85.1 VLA 分类：RT-1（ActionToken）→ RT-2（VLM→Action Token）→ OpenVLA（开源离散）→ Octo（diffusion）→ π0（flow）→ π0-FAST（DCT 离散）→ π0.5（两阶段）
- 85.2 OpenVLA 架构：Llama-2-7B + DINOv2+SigLIP + 256-bin 每维 token；co-fine-tune OXE 970k
- 85.3 FAST tokenization 完整推导（DCT-II + 量化 + column-first + BPE + FAST+）
- 85.4 Flow matching 数学：CFM 损失、OT path、Euler ODE 采样
- 85.5 π0 block-wise 因果掩码；PaliGemma 骨干；action expert 300M
- 85.6 π0.5 两阶段训练：FAST 统一预训练 + flow matching 微调；web 图文 co-training
- 85.7 GR00T N1/N1.5 dual-system（S1 DiT + S2 Eagle-2 VLM）+ FLARE + DreamGen
- 85.8 Figure Helix 02 三层（S0 1 kHz 运动先验 + S1 pixels-to-whole-body + S2 VLM）
- 85.9 Open X-Embodiment 数据混合策略与 RT-1-X / RT-2-X 正迁移
- 85.10 边缘部署：LoRA 微调、INT4/8 量化、ONNX/TensorRT、WebSocket 远程推理（openpi `serve_policy`）
- 85.11 评测基准：LIBERO、RoboCasa365、HomeRobot OVMM、DROID
- 85.12 安全与未来方向：flow VLA + CBF、持续学习、VLA + MPC shielding

**指定论文**：Brohan 2023 (RT-2)、Kim 2024 (OpenVLA)、O'Neill 2024 (OXE)、Octo Team 2024、Black 2024 (π0)、Pertsch 2025 (FAST)、PI 2025 (π0.5)、NVIDIA 2025 (GR00T N1)、Figure 2025/2026 (Helix blog)、Liu 2025 (RDT-1B)、Nasiriany 2024 (RoboCasa)、Yenamandra 2023 (HomeRobot OVMM)、Liu 2024 (OK-Robot)。

**指定项目精读**：`Physical-Intelligence/openpi`（`models/pi0.py`、`training/config.py`、`serve_policy.py`）、`openvla/openvla`（包括 OFT 连续版）、`NVIDIA/Isaac-GR00T`、`thu-ml/RoboticsDiffusionTransformer`、`octo-models/octo`、`google-deepmind/open_x_embodiment`、`robocasa/robocasa`、`facebookresearch/home-robot/projects/habitat_ovmm`。

**练习**：
- **A 编码**：(1) 实现 FAST tokenizer + 解码器，在 ALOHA 50 Hz 数据上验证 10× 压缩；(2) 用 openpi 的 LoRA 在自采 500 条家庭清洁任务上微调 π0.5-KI，并用 `serve_policy.py` 部署到 Stretch 3；(3) 在 Jetson Orin 上跑 OpenVLA-7B INT4 推理，测吞吐 + 延迟；(4) 用 Octo fine-tune 脚本在 RoboCasa365 的 20 任务上训练并对比 π0。
- **B 源码阅读**：(1) 画 π0 的 block-wise 因果掩码与 KV-cache 流；(2) 对比 `openpi/src/openpi/models/pi0.py` (JAX) 与 `models_pytorch/` 的实现差异；(3) 阅读 GR00T N1 的 DiT action head + Eagle-2 接口；(4) 阅读 OpenVLA 的 256-bin 动作离散化并对比 FAST 的 DCT 压缩。
- **思考题**：(1) flow matching 相对 DDPM 的 10× 步数减少，在什么条件下近似失效？(2) FAST 的 BPE 为何对机器人动作有效但对自然文本无效？(3) VLA + 形式化安全如何设计？CBF 每步投影 vs 端到端约束训练？(4) dual-system (Helix/GR00T) 的 S1/S2 频率差（200 Hz vs 7 Hz）如何同步？(5) π0.5 的 web 图文 co-training 是否会让 VLA 偏离精细操作？

**学习时间**：讲授 7 h + 实验 20 h + 阅读 14 h = **41 h**。

---

## 七、总结与闭环

**核心技术判断**：MM 在 2026 年呈现两条并行主线——(1) **MPC + WBC 严肃控制**：以 OCS2 `mobile_manipulator` + `perceptive_mpc` + MoveIt2 + Nav2-MPPI 为代表，保证安全、可解释、形式化可证；(2) **VLA 端到端学习**：以 π0.5 + GR00T N1.5 + Helix 02 为代表，追求开放世界泛化与自然语言接口。两者的融合点（VLA + CBF/MPC shielding、MPC + learned cost/value）是未来 3–5 年博士课题富矿。

**硬件推荐梯度**：预算<$30k 选 Stretch 3 或 Mobile ALOHA DIY；$30–70k 选 Galaxea R1 或 Husky+UR5；>$100k 选 TIAGo Pro 或 Cobot Magic 商业版。HSR 仅借用、Fetch 已停产。

**复合/120_底盘臂联合规划–85 累计 132 h**，覆盖从底层 NMPC 数学到前沿 VLA 部署的完整梯度，承接用户前 76 章 SLAM+腿足+轮足积累，可直接衔接博士级研究选题。下一步建议延展方向：(a) MM + 灵巧手（SvH/Ability Hand/Inspire Hand）；(b) MM + 视触融合（DIGIT/GelSight）；(c) MM + 持续学习（LoRA+replay）。

> **谨慎声明**：引用数均为 2026-04 估计，最终请以 Google Scholar 为准；Helix 01/02 仅有公司博客无 arXiv 论文；HSR 不公开销售；Fetch 2025 年底由 Zebra 关停支持到 2026-03；OCS2 主分支目前仅 ROS 1 Noetic，ROS 2 port 为社区 fork；部分 Star 数为搜索快照估计，可能与官方主页实时值有 ±20% 偏差。

---

> **D2→D3a 过渡提示**：D2 移动操作的底盘是轮式的（差动/全向/阿克曼），运动和操作是**松耦合**的——底盘负责导航，臂负责操作，两者通过联合雅可比 J=[J_base | J_arm] 连接但不存在动力学强耦合。接下来的 D3a 四足+臂报告则进入**紧耦合**领域：底盘本身是四足机器人，行走中的接触力和臂的操作力通过浮动基座动力学强耦合——臂施力时摩擦锥收紧、操作任务要求与行走稳定性产生优先级冲突。这一跃变使得 D2 中足够的"Nav2 导航 + MoveIt2 规划"分层架构在 D3a 中不再适用，取而代之的是 qm_control 式的统一 NMPC+WBC 架构或 Deep-WBC 式的端到端 RL 策略。D2 报告中 OCS2 mobile_manipulator 的 SQP+HPIPM 求解经验（复合/130_OCS2_mobile_manipulator）直接为 D3a 中 qm_control 的 OCS2 NMPC 精读（复合/170_qm_control精读）提供基础；D2 的 UMI 数据采集范式（复合/150_Mobile_ALOHA与UMI）在 D3a 中被 UMI-on-Legs（复合/200_UMI_on_Legs精读）扩展到四足平台上，实现了操作策略的跨平台迁移。

