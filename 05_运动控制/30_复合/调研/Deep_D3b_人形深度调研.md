## 人形机器人 Loco-Manipulation 深度调研（复合/220_经典人形全身控制–95 教学大纲）

本报告系统梳理**人形机器人 loco-manipulation（D3b）**的理论基础、RL 前沿、力敏感最新工作、开源生态、硬件平台与开放问题，为研究生级别 复合/220_经典人形全身控制–复合/250_力敏感人形LocoMani 四章提供结构化教学支撑。报告将用户已具备的 **SLAM + 腿足规控 + 轮足 + 移动操作 + 四足+臂** 基础作为先验，**只补充人形特有增量**（双足平衡、30+ DoF 全身动量、SMPL-X 重定向、双智能体 RL、sim-to-real delta action）。核心平台锁定 **Unitree G1 / H1**，理论主轴 MPC+WBC，技术主轴 RL。

---

## 第 1 部分　论文清单（28 篇，三个时代）

### 1.1　经典人形控制（2003–2020，10 篇）

**① Kajita et al. 2003 – ZMP Preview Control**
*"Biped walking pattern generation by using preview control of zero-moment point"*，ICRA 2003，DOI 10.1109/ROBOT.2003.1241826，Google Scholar 引用 **>3500**。把双足动力学抽象为 **cart-table 模型**，把 CoM 轨迹规划形式化为"未来 ZMP 参考的预见控制"最优跟踪伺服器，即离散 LQR with preview horizon。首次给出可实时计算的 CoM 轨迹，成为所有后续 LIPM 基线的母本。非作者官方开源，但社区复现极多（如 `rdesarz/biped-walking-controller`、JRL Stack-of-Tasks 的 LIPM demo）。**定位**：双足 pattern generator 的开山之作。

**② Wieber 2006 – Trajectory-free Linear MPC**
*"Trajectory free linear model predictive control for stable walking in the presence of strong perturbations"*，Humanoids 2006，Google Scholar **>900**。把 preview control 换成**带 ZMP 约束的 QP-MPC**，在每个采样周期重新优化 CoM jerk，允许在线应对强扰动且不需要预计算完整 ZMP 轨迹。奠定了 **ZMP-MPC** 范式（后来 Herdt 2010、Naveau 2017 的衍生）。无单一官方 repo，但 JRL/LAAS `walkgen-tools` 与 `pymanoid` 为参考实现。**定位**：从开环预见转向闭环 MPC 的关键转折。

**③ Pratt et al. 2006 – Capture Point**
*"Capture Point: A Step toward Humanoid Push Recovery"*，Humanoids 2006，Google Scholar **>1500**。引入 **Capture Point (CP) = x + ẋ/ω₀**：单足支撑下若将脚踏点设置在 CP 上，CoM 轨迹将渐近稳定到 CP。给出 push recovery 的三种策略（ankle / hip / stepping）。**定位**：从轨迹层抽象到"瞬时状态可达性"的几何化指标，是后续 DCM/ICP 的核心前体。

**④ Englsberger, Ott, Albu-Schäffer 2015 – 3D DCM**
*"Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion"*，T-RO 31(2)，引用 **>500**。把 CP 推广到 3D 并分解 LIPM 为**发散分量 (DCM, ξ = x + ẋ/ω) + 收敛分量**，控制目标转为跟踪 DCM 参考，收敛分量解析稳定。给出完整 **DCM–VRP (Virtual Repellent Point)** 控制律，用于 DLR TORO 和 Atlas 衍生平台。**定位**：把双足控制从 CoM 显式跟踪推进到 DCM reduced-order 稳定化——TSID/OCS2 人形配置的主流参考。

**⑤ Sentis & Khatib 2005 – Operational Space Hierarchical WBC**
*"Synthesis of Whole-Body Behaviors through Hierarchical Control of Behavioral Primitives"*，Int. J. Humanoid Robotics 2(4)，引用 **>700**。把 Khatib 的 operational-space framework 扩展到**浮动基 + 多接触 + 任务优先级零空间投影**，用 null-space projector Nₖ 实现严格任务优先级 τ = Σᵢ J̄ᵢᵀ Nᵢᵀ Fᵢ\*。**定位**：WBC 分层思想的理论起点。

**⑥ Escande, Mansard, Wieber 2014 – Hierarchical QP**
*"Hierarchical quadratic programming: Fast online humanoid-robot motion generation"*，IJRR 33(7)，引用 **>500**。**Lexicographic HQP** 算法：把任务级别转化为一连串 QP，每级在上一级解空间内求解，用 active-set 的 warm-start 达到 kHz 级求解。奠定了 TSID/TOWR-WB 的 QP 求解器框架。Stack-of-Tasks 的 `eiquadprog`/`soth` 为官方实现。**定位**：把 WBC 从概念推进到可实时的数值算法。

**⑦ Koolen et al. 2016 – Atlas Momentum-Based Control**
*"Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas"*，IJHR 13(1)，引用 **>500**。把**质心动量 hG = [ṗ; k] = AG(q) q̇**（Orin-Goswami centroidal momentum matrix）作为 top-priority task，通过 QP 求解接触力与关节力矩。IHMC 团队 DARPA 冠军之一的基础。开源：`IHMCRobotics/ihmc-open-robotics-software`。**定位**：动量守恒视角的 WBC，奠定了"角动量调控"作为稳定性度量。

**⑧ Kuindersma et al. 2016 – Atlas DRC Control Stack**
*"Optimization-based locomotion planning, estimation, and control design for the atlas humanoid robot"*，Auton. Robots 40(3)，引用 **>1100**。完整 DRC 栈：kino-dynamic footstep plan → Zero Moment Point MPC → Instantaneous inverse dynamics QP → atlas 电液执行器接口。引入**混合整数 footstep planning + contact-force-QP**。开源：MIT `RobotLocomotion/drake`。**定位**：工业级 Atlas 控制栈的文献蓝本。

**⑨ Feng, Xinjilefu, Atkeson 2015 – CMU DRC Atlas Walking**
*"Optimization-based full body control for the DARPA robotics challenge"*，J. Field Robotics 32(2)，引用 **>400**。CMU 队 Atlas 的 **CoM + swing-foot + pelvis-orientation + momentum 任务** 的耦合 QP 方案，强调在线 footstep 重规划与状态估计鲁棒性。配套 `ihmc-open-robotics-software` 的 CMU 分支。**定位**：全尺寸人形工程化 WBC 的教学样板。

**⑩ Herzog, Rotella, Mason, Grimminger, Schaal, Righetti 2016 – Momentum HLSP on Torque-Controlled Humanoid**
*"Momentum control with hierarchical inverse dynamics on a torque-controlled humanoid"*，Auton. Robots / IJHR 2016，引用 **>300**。在 MPI Sarcos 上把 momentum regulation 与 hierarchical QP 结合，首次在**真实扭矩控制人形**上实现显式角动量调节。开源：`machines-in-motion/kino_dynamic_opt`、`momentumopt`。**定位**：把角动量 + HLSP 从 Atlas（position-controlled）推进到 torque-controlled 平台，直接启发现今 TSID 的 torque-mode 配置。

---

### 1.2　人形 RL 浪潮（2023–2025，12 篇）

**⑪ Haarnoja et al. 2024 – DeepMind Bipedal Soccer**
*"Learning agile soccer skills for a bipedal robot with deep reinforcement learning"*，**Science Robotics 9 (89)**, eadi8022, 2024。DOI 10.1126/scirobotics.adi8022。Robotis OP3 20-DoF，MuJoCo 训练 → zero-shot real。**两阶段 RL**：先教师策略学原子技能（起立、行走、踢球），再 self-play 蒸馏。结果：行走速度快 181%、转身 302%、起立时间减 63%。**定位**：人形全身 RL 首次登上顶刊，证明 zero-shot sim-to-real 在 20-DoF 人形的可行性。

**⑫ Cheng, Ji, Chen, Yang, Yang, Wang 2024 – ExBody**
*"Expressive Whole-Body Control for Humanoid Robots"*，**RSS 2024**（DOI 10.15607/RSS.2024.XX.107），arXiv 2402.16796。核心思想：上半身 **精确跟踪** CMU MoCap 参考、下半身 **仅跟踪根速度命令**（解耦 keypoint tracking 与 root velocity）。验证平台 Unitree H1 19-DoF。开源 `chengxuxin/expressive-humanoid`（~465★）。**定位**：开创了"上下体任务解耦"范式（FALCON/SoFTA/JAEGER 之祖）。

**⑬ Fu, Zhao, Wu, Wetzstein, Finn 2024 – HumanPlus**
*"HumanPlus: Humanoid Shadowing and Imitation from Humans"*，**CoRL 2024**（openreview WnSl42M9Z4），arXiv 2406.10454。**HST (Humanoid Shadowing Transformer)** 在 AMASS 40h 数据上预训练 decoder-only Transformer 低层策略；**HIT (Humanoid Imitation Transformer)** 做 ACT 风格 behavior cloning。支持 RGB-only 单相机实时 shadow。33-DoF 定制人形。开源 `MarkFzp/humanplus`（~622★）。**定位**：首个把"人体动作→机器人"用 Transformer 统一起来并开源完整栈的工作。

**⑭ He et al. 2024 – H2O**
*"Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"*，**IROS 2024**，arXiv 2403.04436。提出 **"sim-to-data"** 流程：用 privileged imitator 在 AMASS 中筛选可行动作；再训 real-time imitator。Unitree H1 上实现 RGB 单相机 zero-shot 遥操作（走、踢、后跳、挥拳）。开源 `LeCAR-Lab/human2humanoid`（~856★，CC BY-NC 4.0）。**定位**：首个 learning-based real-time humanoid 全身遥操作。

**⑮ He et al. 2024 – OmniH2O**
*"OmniH2O: Universal and Dexterous Human-to-Humanoid Whole-Body Teleoperation and Learning"*，**CoRL 2024**，arXiv 2406.08858。在 H2O 基础上引入 **kinematic pose 为通用接口** + **privileged teacher → DAgger 蒸馏到稀疏 obs student**（仅需 VR 头 + 两手位姿），支持 VR / 语言 / GPT-4o 多模态驱动 + 灵巧手 IK。发布 OmniH2O-6 数据集。**定位**：把 H2O 从"RGB shadow"推进到"sparse-input versatile autonomy"，是 HOVER 的前身。

**⑯ Luo et al. 2023 – PHC / 2024 PULSE**
*"Perpetual Humanoid Control for Real-Time Simulated Avatars"*，**ICCV 2023**，arXiv 2305.06456。提出 **PNN (Progressive Neural Network) + MCP (Mixture of Closed-loop Policies)**，渐进训练新专家并和先前专家混合，实现 AMASS 全库 >98% 重建。PULSE（2024）进一步压缩为 latent 表示供下游 RL 复用。开源 `ZhengyiLuo/PHC`、`ZhengyiLuo/PULSE`。**定位**：**动捕→仿真双足 imitator 的黄金参考**，被 H2O、OmniH2O、ASAP、HOVER 全线引用作为 privileged teacher 来源。

**⑰ Zhang, Xiao, He, Shi 2024 – WoCoCo**
*"WoCoCo: Learning Whole-Body Humanoid Control with Sequential Contacts"*，**CoRL 2024**，arXiv 2406.06005。把 loco-manipulation 任务表示为一串 **contact goals 序列**（无需 reference motion），用通用 reward 处理跳箱、踢球、载物行走。H1 上真实部署。**定位**：反 reference-motion 路线——把 contact 作为唯一监督信号。

**⑱ Cheng, Li, Yang, Wang 2024 – OpenTeleVision**
*"Open-TeleVision: Teleoperation with Immersive Active Visual Feedback"*，**CoRL 2024**，arXiv 2407.01512。VR (Vision Pro / Quest) + WebXR + ZED 立体相机 + Pinocchio IK 的开源沉浸式遥操作栈，适配 H1 / GR-1。`OpenTeleVision/TeleVision`（~1.2k★）。**定位**：RGB/VR 遥操作数据采集的事实标准开源实现。

**⑲ Zhuang, Yao, Zhao 2024 – Humanoid Parkour**
*"Humanoid Parkour Learning"*，**CoRL 2024**，arXiv 2406.10759。端到端 depth → joint 的 vision-based WBC，无需任何 motion prior，分形噪声地形 + 自动课程。Unitree H1 上跳 0.42 m 平台、越 0.8 m 沟。2025 续作 **Perceptive Humanoid Parkour (PHP)** 在 G1 上爬 1.25 m 墙（arXiv 2602.15827）。**定位**：parkour 路线在人形上的对照组。

**⑳ Ji et al. 2024 – ExBody2**
*"Advanced Expressive Humanoid Whole-Body Control"*，arXiv 2412.13196。ExBody + privileged teacher 蒸馏 + keypoint/velocity 解耦 + AMASS 难度自适应数据筛选。H1 与 G1 双平台。项目主页 exbody2.github.io。**定位**：ExBody 的工业强化版，现为新方法的标准对比 baseline。

**㉑ He, Xiao, Lin et al. 2024 – HOVER**
*"HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots"*，arXiv 2410.21229；**ICRA 2025**（pp. 9989-9996）。NVIDIA + CMU + UT Austin 联合。把 motion imitation 作为 **通用预训练**，再通过 **observation masking** 蒸馏出一个可切换控制模式的单一网络（根速度 / 关节角 / 端效位姿）。G1/H1/H1-2 多平台。**定位**：从"多个任务专用策略"走向"单策略多模式"，为 VLA 接入铺路。

**㉒ He et al. 2025 – ASAP** ⭐
*"ASAP: Aligning Simulation and Real-World Physics for Learning Agile Humanoid Whole-Body Skills"*，arXiv 2502.01143，**RSS 2025**。CMU LeCAR-Lab + NVIDIA + UT Austin。**两阶段 sim-to-real**：阶段一，AMASS/TRAM 视频 → SMPL → 机器人重定向 → phase-conditioned motion tracking policy（IsaacGym）；阶段二，真实 rollout → **delta action residual model** $a_{\\text{real}}=\\pi(s)+\\Delta_\\phi(s)$ 最小化 simulation 与真实状态差 → 把 $\\Delta_\\phi$ 注回仿真器 fine-tune → 部署时丢弃 $\\Delta_\\phi$。IsaacGym→G1 实机 tracking error 降低 **52.7%**。开源 `LeCAR-Lab/ASAP`（~1.9k★ MIT）。**定位**：当前 sim-to-real agile humanoid 的 SOTA 基准，复合/240_ASAP_SimToReal 核心论文。

**㉓ Zhuang et al./Chen et al. 2024 – BeyondMimic / OmniRetarget / GMR 系列**（合并列出）。GMR = General Motion Retargeting，OmniRetarget（LeCAR 子项目）与 BeyondMimic（arXiv 2507.xxxxx 类）代表"retargeting-as-a-service"方向：把 SMPL→任意机器人关节的通用可微管线模块化。**定位**：给 复合/230_人形全身RL 提供"数据生成器"教学实操。

---

### 1.3　力敏感人形 Loco-Manipulation（2024–2026，8 篇） 

**㉔ Lu, Cheng, Li, Yang et al. 2024 – Mobile-TeleVision / PMP**
arXiv 2412.07773，**ICRA 2025**。上体 IK 精准操作 + 下体 RL locomotion + **CVAE-based Predictive Motion Prior** 作为下体 RL 的 observation condition。H1 主平台。主页 mobile-tv.github.io。**定位**：力敏感谱系的"上下体解耦 + 运动先验"奠基。

**㉕ Ben, Jia, Zeng et al. 2025 – HOMIE**
*"HOMIE: Humanoid Loco-Manipulation with Isomorphic Exoskeleton Cockpit"*，arXiv 2502.13013，**RSS 2025**。3D 打印 7-DoF 同构外骨骼（关节-关节一一映射，无 IK 误差）+ 15-DoF Hall 感应手套 + 脚踏板，硬件 ~$500。RL 下体三技术：**任意上体姿态课程 + 高度追踪 reward + 对称正则**。G1 与 GR-1。任务完成时间为 Open-TV 的一半。开源 `InternRobotics/OpenHomie`。**定位**：力敏感谱系的遥操作硬件分支代表。

**㉖ Zhang, He, Shi et al. 2025 – FALCON** ⭐⭐
*"FALCON: Learning Force-Adaptive Humanoid Loco-Manipulation"*，arXiv 2505.06776，**L4DC 2026**。**双智能体 RL（Dual-Agent PPO）**：下体 agent 维持行走稳定、上体 agent 精确 EE tracking + 隐式力补偿，共享 proprioception 与 commands，各自 reward 与 action head 联合训练。**3D 力课程**：渐进施加 0→100N 外力（~30% 体重），用逆动力学实时验证关节力矩可行性。上体 joint tracking 精度相较 monolithic WBC 和 IK-upper baseline 提升 **2×**；G1 29-DoF 与 Booster T1 跨平台验证；任务：运货 0-20N / 拉车 0-100N / 开门 0-40N。开源 `LeCAR-Lab/FALCON`（~311★ MIT，基于 HumanoidVerse）。**定位**：复合/250_力敏感人形LocoMani 核心论文，力敏感 loco-mani 当前 SOTA。

**㉗ Li, Zhang, Xiao et al. 2025 – SoFTA / Hold My Beer**
*"Hold My Beer: Learning Gentle Humanoid Locomotion and End-Effector Stabilization Control"*，arXiv 2505.24198。**Slow-Fast Two-Agent**：上体 100 Hz 精细 EE 稳定、下体 50 Hz 鲁棒 gait、仿真 200 Hz、`control_decimation` 调节；标准 PPO（on-policy，并非题目原描述的 off-policy）。EE 加速度降低 **50–80%**，端啤酒不洒、可做摄影稳定器。开源 `LeCAR-Lab/SoFTA`。**定位**：FALCON 的时间维（频率解耦）扩展，复合/250_力敏感人形LocoMani 对照案例。

**㉘ Kuang et al. 2025 – SkillBlender**
*"SkillBlender: Towards Versatile Humanoid Whole-Body Loco-Manipulation via Skill Blending"*，arXiv 2506.09366。**分层 RL**：先预训练 goal-conditioned primitive skills（走、伸手、蹲、跳）；再动态 blend 完成复合任务（仅需 1-2 个 reward）。发布 **SkillBench**（H1/G1/H1-2 × 4 技能 × 8 任务）。开源 `Humanoid-SkillBlender/SkillBlender`，主页 usc-gvl.github.io/SkillBlender-web。**定位**：技能库 / 可组合性路线，与 foundation-model 路线（BFM-Zero）正交。

**㉙ Li, Luo, Zhang et al. 2025 – BFM-Zero**
*"BFM-Zero: A Promptable Behavioral Foundation Model for Humanoid Control Using Unsupervised Reinforcement Learning"*，arXiv 2511.04131。基于 Meta Motivo 的 **Forward-Backward (FB) 表示 + FB-CPR**，**无监督 off-policy RL**。统一 latent z 同时编码 motions/goals/rewards，推理时 zero-shot / few-shot。G1 实机。开源 `LeCAR-Lab/BFM-Zero`（CC BY-NC 4.0）。**定位**：首个面向真实人形的 promptable behavioral foundation model。

**㉚ Yang et al. 2024 – ACE**
*"ACE: A Cross-Platform Visual-Exoskeletons System for Low-Cost Dexterous Teleoperation"*，arXiv 2408.11805。跨平台 6-DoF 外骨骼 + wrist + 相机，末端位姿映射（区别于 HOMIE 的关节同构），支持 humanoid/arm-hand/quadruped-gripper。主页 ace-teleop.github.io。**定位**：遥操作硬件的跨平台 baseline，HOMIE 对照组。

**㉛ Zhu et al. 2025 – ReLIC**（题目中标注的 "SPIN" 实为此缩写）
*"Versatile Loco-Manipulation through Flexible Interlimb Coordination"*，**CoRL 2025** (PMLR v305 pp. 610-632)，arXiv 2506.07876，RAI Institute。自适应控制器**动态分配 limb 角色**（走 vs. 操作），支持 3-leg walking、gait switch、foot-assisted manipulation；model-based（任务成功优先）+ RL gait policy 两模块协作；CMA-ES 做真机 torque 校准（Spot+arm；非双足人形但对照有价值）。**定位**：limb 角色分配路线，与上下体解耦正交。

> **注**：原题目中 SkillBlender "CoRL 2024"、SPIN 缩写、ACE "Adaptive Control" 三处有偏差，已在此纠正。

---

## 第 2 部分　开源项目精读清单（12 个）

| # | 项目 | URL | ★ | 许可 | 阅读时间（难度） |
|---|------|-----|---|------|----------------|
| 1 | **LeCAR-Lab/ASAP** | github.com/LeCAR-Lab/ASAP | ~1.9k | MIT | 25-35h（高）|
| 2 | **LeCAR-Lab/FALCON** | github.com/LeCAR-Lab/FALCON | ~311 | MIT | 15-20h（中高）|
| 3 | **MarkFzp/humanplus** | github.com/MarkFzp/humanplus | ~622 | MIT | 15-20h（中）|
| 4 | **LeCAR-Lab/human2humanoid** | github.com/LeCAR-Lab/human2humanoid | ~856 | CC BY-NC 4.0 | 20-25h（高）|
| 5 | **unitreerobotics/unitree_rl_gym** | github.com/unitreerobotics/unitree_rl_gym | ~3.1k | BSD-3 | 8-12h（低-中）|
| 6 | **stack-of-tasks/tsid** | github.com/stack-of-tasks/tsid | ~176 | BSD-2 | 20-30h（高）|
| 7 | **Rhoban/placo** | github.com/Rhoban/placo | ~274 | MIT | 12-18h（中）|
| 8 | **isaac-sim/IsaacLab** | github.com/isaac-sim/IsaacLab | ~6.8k | BSD-3 | 30-40h（高）|
| 9 | **ZhengyiLuo/PHC / PULSE** | github.com/ZhengyiLuo/PHC | ~1k+ | 研究用 | 20-25h（高）|
| 10 | **OpenTeleVision/TeleVision** | github.com/OpenTeleVision/TeleVision | ~1.2k | MIT | 10-15h（中）|
| 11 | **chengxuxin/expressive-humanoid** | github.com/chengxuxin/expressive-humanoid | ~465 | MIT | 12-18h（中）|
| 12 | **LeCAR-Lab/BFM-Zero** | github.com/LeCAR-Lab/BFM-Zero | ~100+ | CC BY-NC 4.0 | 20-25h（高）|

### 2.1　核心精读文件建议

**ASAP**（复合/240_ASAP_SimToReal 主项目）：
- `humanoidverse/envs/motion_tracking/motion_tracking.py` — `LeggedRobotMotionTracking` 基类
- `humanoidverse/config/env/delta_a_open_loop.yaml` / `delta_a_closed_loop.yaml` — delta action 训练配置
- `humanoidverse/train_agent.py` + `sim2real/rl_policy/listener_deltaa.py` — 真机数据采集与 finetune 入口
- 关键脚本：`scripts/data_process/` 下 SMPL→G1 的 shape fitting 与 motion retargeting 管线

**FALCON**（复合/250_力敏感人形LocoMani 主项目）：
- `humanoidverse/agents/ppo_ma/` — multi-agent PPO 两策略（上/下体独立 actor head、共享 critic）
- `humanoidverse/envs/decoupled_locomotion/` — 共享 proprioception 的解耦 env
- `humanoidverse/config/rewards/dec_loco/reward_dec_loco_stand_height_ma_diff_force.yaml` — 3D 力课程 reward 权重

**unitree_rl_gym**（复合/220_经典人形全身控制 入门项目）：
- `legged_gym/envs/g1/g1_env.py` + `g1_config.py`
- `deploy/deploy_mujoco/deploy_mujoco.py` + `deploy/deploy_real/deploy_real.py` — sim2sim/sim2real 对比
- `deploy/pre_train/g1/motion.pt` — 可直接部署的预训练权重

**tsid**（复合/220_经典人形全身控制 WBC 教学）：
- `include/tsid/formulations/inverse-dynamics-formulation-acc-force.hpp` — HLSP 构造
- `include/tsid/tasks/task-se3-equality.hpp`、`task-com-equality.hpp`
- `demo/demo_romeo.py` — 完整 WBC 例程

**human2humanoid**（复合/230_人形全身RL 主项目）：
- `legged_gym/legged_gym/envs/h1/h1_teleop.py`
- `scripts/data_process/grad_fit_h1.py` — 可微 SMPL→H1 重定向
- `legged_gym/scripts/train_hydra.py` + `config/config_teleop.yaml`

**IsaacLab**（环境框架精读）：
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py`
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/g1/rough_env_cfg.py` 与 `h1/rough_env_cfg.py`

> **注**：IsaacLab 在 2025 年架构重构后，旧路径 `source/extensions/omni.isaac.lab_tasks/...` 已废弃，改为 `source/isaaclab_tasks/...`。

---

## 第 3 部分　人形特有数学增量

用户已掌握 Pinocchio / OCS2 / Crocoddyl / TSID / DDP / grid_map / perceptive MPC 等一般腿足工具。以下只覆盖**人形独有**的数学内容。

### 3.1　LIPM 与 DCM/Capture Point 完整推导

**线性倒立摆模型（LIPM）**：假定 CoM 高度 $z_c$ 恒定，忽略角动量变化 $\\dot k \\approx 0$，得
$$\\ddot x = \\omega_0^2 (x - p_x), \\quad \\omega_0 = \\sqrt{g/z_c}$$
其中 $p_x$ 是 ZMP 的 x 坐标。此为 **cart-table 模型**。Kajita 2003 的 preview LQR 优化代价
$$J = \\sum_k \\big[ Q_e (p_x^{\\text{ref}} - p_x)^2 + R \\dddot x^2 \\big]$$
控制量为 CoM jerk $\\dddot x$，构造增广系统后用离散 Riccati + 未来参考前馈解出。

**发散/收敛分量分解（DCM）**：定义 $\\xi = x + \\dot x/\\omega_0$，则
$$\\dot \\xi = \\omega_0 (\\xi - p_x), \\quad \\dot x = -\\omega_0 (x - \\xi)$$
其中 $\\xi$ 是**发散**模态（不稳定极点 $+\\omega_0$），$x$ 对给定 $\\xi$ 是**收敛**的（极点 $-\\omega_0$）。控制目标简化为：**让 ZMP 跟踪 DCM 参考**以稳定 $\\xi$，$x$ 自动稳定。

**Capture Point 即零速瞬时 DCM**：$\\xi_{\\text{CP}} = x + \\dot x/\\omega_0$ 是"若立即停止扰动，脚应踏下的点"。Pratt 2006 给出 **踝关节策略 / 臀策略 / 迈步策略** 的代数触发条件。

### 3.2　质心动量矩阵与角动量调控

**Centroidal Momentum Matrix (CMM)**（Orin-Goswami 2008，在 Pinocchio 中可由 `ccrba()` 一行算出）：
$$h_G = \\begin{bmatrix} \\dot{p}_G \\\\ k_G \\end{bmatrix} = A_G(q)\\dot q \\in \\mathbb R^6$$
对 30+ DoF 人形，$A_G \\in \\mathbb R^{6\\times n}$。其时间导数：
$$\\dot h_G = \\sum_i \\begin{bmatrix} f_i \\\\ (p_i - p_G) \\times f_i + \\tau_i \\end{bmatrix} - \\begin{bmatrix} 0 \\\\ m g \\times (p_G - p_0) \\end{bmatrix}$$
即"质心处合力旋量 = 接触力旋量叠加"。**角动量 $k_G$ 本身可解释为绕 CoM 的 whole-body 角动量**——Pratt & Koolen 证明 push recovery 中若 $k_G$ 不主动调控则易失稳（"hip-strategy"本质上就是通过大幅摆臂摆腰产生反向 $\\dot k_G$）。

WBC 任务通常设
$$\\text{task}_{\\text{mom}}: \\quad A_G \\ddot q + \\dot A_G \\dot q = \\dot h_G^{\\text{ref}}$$
作为 top-priority 约束或高权重 cost，在 Koolen 2016 / Herzog 2016 的 HLSP 中出现。

### 3.3　SMPL-X → Humanoid 关节重定向

AMASS 中的人体用 **SMPL-X**（21 body joints + 15×2 hands + 3 face/eye，共 55 joints）+ 10 shape βs + 6890 顶点网格参数化。重定向到 Unitree G1 29-DoF 的数学步骤：

1. **Shape fitting** — 把 G1 URDF 的 marker（肩、肘、腕、髋、膝、踝关键点）和 SMPL marker 通过 β 参数优化对齐：
$$\\beta^\\star = \\arg\\min_\\beta \\sum_k \\| J_k^{\\text{SMPL}}(\\beta) - J_k^{\\text{G1}} \\|^2$$
（ASAP 的 `grad_fit_g1_shape.py`、H2O 的 `grad_fit_h1_shape.py`）

2. **Motion fitting** — 给定每帧 SMPL 姿态 $\\theta_t$，对 G1 关节 $q_t$ 做 IK：
$$q_t^\\star = \\arg\\min_{q_t} \\sum_k \\| T_k(q_t) - T_k^{\\text{SMPL}}(\\theta_t, \\beta^\\star) \\|^2 + \\lambda \\|\\dot q_t\\|^2$$

3. **物理可行性过滤** — 用 privileged imitator 在仿真中 rollout，删除 tracking error 超阈值的动作（H2O / ExBody2 的关键步骤）。

### 3.4　ASAP Delta-Action Residual Model 形式化

关键方程（He et al. 2025）：
$$a_{\\text{sim→real}}(s_t) = \\pi_\\theta(s_t) + \\Delta_\\phi(s_t, a_t, h_t)$$
训练目标：
$$\\min_\\phi \\mathbb E_{(s_t^r, a_t^r, s_{t+1}^r) \\sim \\mathcal D_{\\text{real}}} \\| s_{t+1}^r - f_{\\text{sim}}(s_t^r, a_t^r + \\Delta_\\phi) \\|^2$$
其中 $f_{\\text{sim}}$ 是 IsaacGym 的转移函数。**关键创新**：$\\Delta_\\phi$ 只修改 **action** 而非 **dynamics**，因此 finetune 时 $\\Delta_\\phi$ 冻结注入仿真器后，可直接用原 PPO 框架继续训练；**部署时丢弃 $\\Delta_\\phi$**——因为 finetune 后策略已把补偿学进 $\\pi_\\theta$。这是与传统 **delta dynamics learning**（修改 $f$）相比的计算与过拟合优势。

### 3.5　双足 Footstep Planning 作为独立子问题

四足的 footstep 通常与 body trajectory 一起优化（Kino-dynamic NLP）。双足由于**支撑多边形小、双支撑/单支撑交替**，footstep 常作为**独立的离散决策层**：
- **MIP/MICP**（Deits & Tedrake 2014, Kuindersma 2016）：footstep 位置 + 离散可行脚印区域作为 mixed-integer program
- **DCM-based footstep planner**（Englsberger 2015）：根据当前 DCM $\\xi$ 与预期 $\\xi^{\\text{ref}}$ 解析计算下一步落点 $p_x^{\\text{next}} = \\xi - (\\xi^{\\text{ref}} - \\xi)\\, e^{\\omega_0 T_s}$
- **Learning-based**：footstep 作为 policy 的 auxiliary output（HOMIE 的踏板映射 / WoCoCo 的 contact target）

### 3.6　30+ DoF 分层 QP 的可扩展性

朴素 HQP 每级 QP 的 active-set 求解复杂度 $O(n^3)$，30-DoF 双足 + 4 接触点 ×3D ≈ 60 变量，典型 3–5 级，耗时 1–3 ms。扩展技术：
- **Null-space basis recycling**（Escande 2014）：上级解的 null-space 基用 warm-start
- **HPIPM / PROXQP**（Stéphane Caron 2022 年后主流 TSID 后端）：内点法，对稀疏矩阵加速
- **GPU-based parallel QP**（近期尝试）：适合大批量 sim；真机仍以 CPU+warm-start 为主

人形特有难点：接触模式变化（单支撑↔双支撑）导致 active-set **不连续**，需平滑切换（soft contact switching）。

### 3.7　FALCON 双智能体 RL 数学

共享观测 $o_t$（proprio + command + 上体目标 + 下体目标），策略分解：
$$a_t^{\\text{upper}} = \\pi_\\theta^{\\text{u}}(o_t), \\quad a_t^{\\text{lower}} = \\pi_\\theta^{\\text{l}}(o_t)$$
损失（对称 PPO）：
$$L = \\mathbb E[L_{\\text{PPO}}(\\pi^{\\text{u}}, r^{\\text{u}}) + L_{\\text{PPO}}(\\pi^{\\text{l}}, r^{\\text{l}})]$$
其中 $r^{\\text{u}} = r^{\\text{EE-track}} + r^{\\text{reg}}$（末端位姿追踪）、$r^{\\text{l}} = r^{\\text{gait}} + r^{\\text{balance}} + r^{\\text{vel-track}}$。**力课程**：episode 中外力 $f_t^{\\text{ext}}$ 按课程线性增至 100N，每步用 Pinocchio RNEA 验证 $\\tau = H\\ddot q + C\\dot q + g - J^T f^{\\text{ext}}$ 是否在力矩上限内，不满足则截断 episode。

### 3.8　角动量在 Push Recovery 的显式利用

给定扰动冲量 $\\Delta p$，CoM 速度跳变 $\\Delta \\dot x = \\Delta p / m$。角动量策略：短时间内用臂腿反向摆动产生 $\\dot k_G^{\\text{rec}}$，其对 CoM 的反作用为 $\\dot h_G = [\\dot p_G; \\dot k_G]$ 中 $\\dot k_G$ 部分可通过接触位置杠杆臂折算为水平力：
$$f_x^{\\text{eff}} = \\dot k_G^y / (z_{\\text{CoP}} - z_{\\text{CoM}})$$
即"臀部反向摆动 → 有效脚底水平力"。这是 hip-strategy 的数学基础，也是 ExBody 上体模仿 + 下体解耦能稳定行走的原因。

---

## 第 4 部分　硬件平台详解

### 4.1　Unitree G1（主要平台，SKU 与价格 2026 年 4 月）

| SKU | DOF | 关键配置 | 参考价（USD） |
|-----|-----|---------|--------------|
| G1 EDU U1 | 23 | 腿 5×2 + 臂 4×2，无腰/手，Jetson Orin NX 100 TOPS | $42,435 |
| G1 EDU U2 Plus | 29 | +3-DoF 腰、7-DoF/臂，**whole-body loco-mani 起点** | $52,367 |
| G1 EDU U3 Ultimate A | 43 | +Dex3-1 3 指力控手（7 DoF/手） | $64,292 |
| G1 EDU U4 Ultimate B | 43 | Dex3-1 + 17 触觉传感器/手 | $66,277 |
| G1 EDU U5 Ultimate C | 41 | Inspire RH56DFQ 5 指手 | $66,277 |
| G1 EDU U5-1 Ultimate D | 41 | Inspire RH56DFTP 触觉 5 指手 | $73,900 |

通用：**127 cm / 35 kg，单臂 ~3 kg 负载，续航 ~2 h（9 Ah 热插拔），膝关节峰值 120 N·m**，传感器 Livox MID-360 LiDAR + RealSense D435i。SDK：`unitree_sdk2` C++/Python、URDF 开源、低层力矩 kp/kd 接口开放、Isaac Sim 与 MuJoCo 官方资产支持 sim2real。**消费版 Basic（$16K 起）不支持二次开发**。社区：2025 年出货 5500+ 台，GitHub `unitreerobotics/unitree_rl_gym`（~3.1k★）、`unitree_mujoco`；2025-2026 年百余篇 arXiv 以 G1 为主要平台；2026 年 3 月 Unitree 开源 `UnifoLM-VLA-0`（基于 Qwen2.5-VL-7B）。

### 4.2　Unitree H1 / H1-2 / H2

**H1**（$90-99.9K）：180 cm / 47 kg、19 DoF（5 腿 × 2、4 臂 × 2、1 腰），膝 360 N·m，**步速 3.3 m/s（世界纪录）**，864 Wh 电池。**H1-2**（$128.9K-$150K）：180 cm / 70 kg、27 DoF（6 腿 × 2、7 臂 × 2、1 腰），臂力矩 120 N·m（H1 为 75 N·m），可选 Inspire 5 指手 + Jetson Orin NX。**H2**（起 $30K，2025 末/2026 初发布）：31 DoF，统一计算平台。三者 SDK 同：torque-level、ROS2、EtherCAT、URDF 公开。H1 是 HumanPlus / H2O / OmniH2O / ExBody 的主验证平台。

### 4.3　PAL Talos（欧洲经典学术平台）

175 cm / 95 kg，**32 DoF**（6 腿 × 2、7 臂 × 2、1 夹爪 × 2、2 腰、2 颈），**单臂延伸 6 kg 负载**，1080 Wh 约 1.5 h 行走 / 3 h 待机。**全关节力矩反馈**（除头/腕/夹爪）、EtherCAT **5 kHz** 控制回路，Ubuntu LTS + Linux RT PREEMPT、ROS LTS、OROCOS、Gazebo、MoveIt、WBC 源码开放——**SDK 最开放的大型人形**。价格约 **€900K–€1M**。用户：Inria-LAAS（Pyrène）、欧洲 H2020 项目。**最适合 torque-controlled WBC 教学**，复合/220_经典人形全身控制 的 TSID + Talos 组合是经典教学场景。

### 4.4　Boston Dynamics Atlas Electric

2024 年 4 月液压 Atlas 退役，电动 Atlas 2026 年 CES 宣布量产。约 **150 cm / 89 kg**，关节 360° 旋转，3 指夹爪，**整机负载 50 kg**。**无公开研究 SDK**；Orbit 车队 API + 企业合作。研究获取三渠道：(1) Hyundai（首个商业客户）；(2) **TRI 合作**（2025 年 8 月 LBM 控制 Atlas 演示）；(3) **Google DeepMind 合作**（2026 CES 宣布 Gemini Robotics 部署）。研究团队**无法直接采购**。估价 $150K-$420K。作为教学对比使用。

### 4.5　Fourier GR-1 / GR-2（NVIDIA GR00T 首选平台）

**GR-1**（~$150-170K）：165 cm / 55 kg、**40 DoF**、髋峰值 300 N·m、步速 5 km/h。**NVIDIA GR00T N1** foundation model 主要在 GR-1 上预训练并实时部署（arXiv 2503.14734，HF 数据集 `PhysicalAI-Robotics-GR00T-GR1` 92 段 CC-BY-4.0）。**GR-2**（~$150K+）：175 cm / 63 kg、**53 DoF**（含 **12 DoF/手 + 6 tactile/指尖**）、FSA 2.0 执行器峰值 380 N·m、关节从并联改为**串联**以缩小 sim2real gap、NVIDIA Isaac Lab + MuJoCo 原生支持。用户：ETH Zurich、CMU、NVIDIA 合作伙伴。

### 4.6　Booster T1（FALCON 验证平台）

118 cm / ~30 kg，三配置：标准 **23 DOF / 夹爪 31 DOF / 灵巧手 41 DOF**。Jetson AGX Orin 200 TOPS + Intel i7，膝 130 N·m，>0.5 m/s，10.5 Ah 约 2 h/4 h。**全力矩控制 + 双编码器**、**完整开源 SDK、ROS2、Isaac Sim/MuJoCo/Webots**。售价 **$33,949**。社区：**2025 RoboCup AdultSize 冠军（清华 Hephaestus）**、CMU FALCON、UC Berkeley FastTD3。性价比之王。

### 4.7　次要对比平台

- **Agility Digit**（$250K RaaS）：30 DoF 鸟腿、16 kg 负载、**8 h 续航**、Amazon/GXO 商用部署。ROS 兼容但力矩不开放。
- **Figure 02**（未零售）：16 DoF/手、5 h、专有 Helix VLA，BMW 生产线部署。
- **Tesla Optimus Gen 2/3**：Gen3 手 22 DoF/50 执行器，2026 年 1 月 Fremont 量产。全闭源。
- **Kepler Forerunner K2**（$20-30K）：52 DoF、11 DoF tactile 手（96 触点/指尖）、8 h。
- **Unitree R1**（$4,900 AIR / $10K-35K EDU，2026 Q2 发货）：20-26 DoF 入门款，仅 EDU 开放 SDK。

---

## 第 5 部分　博士研究开放问题（8 个）

### Q1. 走动中的动态双臂协调
**问题**：当前 FALCON/SoFTA 在 EE 稳定/低力载荷下表现好，但**高速双臂协作任务**（如搬沙发、双手拧瓶盖时同步行走、打乒乓球）仍缺乏通用 policy。挑战：上体内部动量耦合（双臂反向摆动产生 $k_G$）与下体步态的实时协调。路线：扩展 FALCON 双 agent 为 **三 agent**（左臂 / 右臂 / 下体）？或把 CMM 约束显式写入 RL reward？**与用户背景匹配**：用户已掌握 WBC + RL 双栈，可从 TSID 的 momentum task 起步做 RL augmentation。

### Q2. 灵巧手指操作叠加行走（触觉+力控）
**问题**：G1 Dex3 / Inspire 5 指手已开源，但**行走中的指尖精细操作**（走路时开门把手、旋转螺丝、握持易碎物）尚无成熟方案。挑战：手指接触力在 10 N 量级，行走扰动易破坏接触。路线：手指 policy + 腕部 compliance + 下体 HOMIE-style loco policy 的三层解耦。

### Q3. FALCON 延伸：接触丰富力控 under 步态扰动
**问题**：FALCON 验证力范围 0-100 N 静力；**动态接触力、冲击、摩擦变化**（推门瞬间滑脱、搬起重物瞬间抬起力）尚未覆盖。路线：把 contact impedance 作为 policy 额外输出；引入 Fisher-Rao 信息几何度量接触模式切换。

### Q4. 长时程 VLM/LLM 任务规划 + 底层 WBC 可行性保证
**问题**：GR00T / HOVER 证明 VLA 可做高层指令解码，但**VLA 输出的 waypoint 序列可能违反下体 ZMP/DCM 可行性**。研究：把 DCM feasibility 作为 differentiable constraint 反传给 VLA；或用 MPC safety filter 过滤 VLA 输出（类似自动驾驶的 CBF-filter）。**与用户 MPC 背景强相关**。

### Q5. 跨具身迁移（G1 ↔ H1 ↔ Digit ↔ GR-1）
**问题**：当前 policy 高度 embodiment-specific，换机器人需重训。SkillBlender 与 HOVER 初步展示 **多平台单策略**，但真实部署仍有 gap。路线：把 kinematic tree 作为 graph network 输入；或用 morphology-conditioned transformer（类似 MetaMorph）。

### Q6. 实时 30+ DoF 全身 MPC（10 ms 预算）
**问题**：当前 SQP + HPIPM 对 30 DoF + 20 步预测只能做到 20-50 ms。**10 ms 以内 tightly-coupled kino-dynamic MPC** 仍未实现。路线：(a) 分层——base 层全身 MPC 低频 + WBC QP 高频；(b) **Learned MPC warm-start**（Neural ODE 预测好的 seed）；(c) GPU-parallel SQP。**与用户 OCS2/Crocoddyl/HPIPM 背景完美契合**。

### Q7. Sim-to-Real Gap for 接触丰富人形（ASAP 延伸）
**问题**：ASAP 的 delta-action 在 **non-contact** agile motion（C 罗 Siuuu、跳跃）上 work，但**接触触发**（抓握、撞击、脚底滑动）引入离散动力学，residual model 线性假设失效。路线：把 delta-action 扩展为 **contact-mode-conditioned residual**；或联合学习 contact event classifier。

### Q8. SLAM-操作耦合在人形上
**问题**：用户已有 46 章 SLAM 背景。人形独有挑战：**头部 IMU 受全身摆动严重扰动**（不同于四足 body IMU），LiDAR/RGB-D 在行走时 rolling-shutter 与 motion blur 严重。研究：主动 SLAM（用头部/腰部 gaze 控制优化 SLAM observability）；SLAM loop-closure 与 WBC 预测状态联合滤波。**SLAM + 人形 WBC 交叉的原创方向**。

---

## 第 6 部分　复合/220_经典人形全身控制–复合/250_力敏感人形LocoMani 章节详细大纲

### 复合/220_经典人形全身控制　经典人形全身控制（LIPM/DCM + TSID + 动量 WBC）

**教学目标**：补齐双足特有的平衡数学（ZMP/DCM/Capture Point）与 30+ DoF 的质心动量 WBC；让用户能用 TSID 在 PAL Talos 或 Unitree G1 仿真上复现 torque-level 行走。

**子节（9 个）**
1. 从四足到双足：为何 ZMP/CoP 成为必需——支撑多边形的时变性
2. LIPM 与 cart-table 模型：Kajita 2003 preview control 完整推导
3. DCM / Capture Point：Pratt 2006 + Englsberger 2015，3D 情形与 VRP
4. 双足 Footstep Planning：MIP/MICP + DCM-based 解析重规划
5. 质心动量矩阵 CMM 与 Pinocchio `ccrba()` 实操
6. 从 Operational Space 到 Hierarchical QP：Sentis 2005 → Escande 2014
7. Momentum-Based WBC：Koolen 2016 Atlas / Herzog 2016 torque-mode
8. 案例解剖：Kuindersma 2016 Atlas DRC 全栈 / Feng 2015 CMU Atlas
9. 用 TSID + Pinocchio 在 Talos/G1 上搭建双足行走 demo

**指定论文**：①②③④⑤⑥⑦⑧⑨⑩（10 篇经典）

**指定代码精读**：
- `stack-of-tasks/tsid` 的 `demo/demo_romeo.py`
- `stack-of-tasks/tsid/include/tsid/formulations/inverse-dynamics-formulation-acc-force.hpp`
- `Rhoban/placo` 的 `src/placo/kinematics/kinematics_solver.cpp`

**练习**（A 编码 + B 源码阅读 + C 思考）
- A1：用 `pinocchio.ccrba()` 验证 Koolen 2016 的 CMM 等式；用 G1 URDF 打印 $A_G(q)$ 的秩；
- A2：写一个 LIPM preview controller，用给定 ZMP 参考生成 CoM 轨迹，MATLAB/Python 皆可；
- B1：阅读 TSID `task-com-equality.hpp` 与 `task-angular-momentum-equality.hpp`，画出其 Jacobian 与期望加速度如何进入 HQP 矩阵；
- B2：阅读 Drake `atlas_walking` 示例，对比 Kuindersma 2016 论文 Fig 3 的 3-level QP 架构；
- C1：推导 Capture Point 的角动量扩展（$\\xi$ 中含 $k_G$ 项）；
- C2：为什么 RL 人形不需要显式 ZMP 约束也能学会走？（从 reward shaping + domain randomization 论证）。

**预计学习时间**：约 **45-55 小时**（10 论文 × 2-3h + TSID 精读 15h + 实操 15h）

---

### 复合/230_人形全身RL　人形全身 RL：动捕重定向与 Teacher-Student 蒸馏

**教学目标**：掌握 SMPL-X → humanoid 重定向管线、AMASS 过滤、privileged teacher-student 训练、RGB/VR 遥操作；在 H1 或 G1 仿真复现 ExBody 或 OmniH2O 风格策略。

**子节（10 个）**
1. 人形 RL 的动机：从 ZMP 约束到端到端学习 + 从四足 RL 到双足的增量
2. AMASS 与 SMPL-X：人体动捕数据的参数化与统计学基础
3. SMPL-X → 机器人关节的可微重定向（H2O 的 `grad_fit_h1.py` 解剖）
4. 物理可行性过滤：PHC/PULSE 作为 privileged imitator 的筛选器
5. ExBody 的核心思想：上体 keypoint 跟踪 + 下体 velocity 跟踪解耦
6. HumanPlus：HST 预训练 + HIT 模仿学习的两阶段
7. H2O / OmniH2O：Teacher-Student 蒸馏 + DAgger + 稀疏观测（VR sparse input）
8. ExBody2 与 HOVER：多模态命令空间统一 + observation masking
9. Haarnoja 2024 DeepMind Soccer：self-play + emergent skills
10. OpenTeleVision + Humanoid Parkour：VR 采集与端到端 depth-to-action

**指定论文**：⑪⑫⑬⑭⑮⑯⑲⑳㉑⑱⑰㉓

**指定代码精读**：
- `chengxuxin/expressive-humanoid/legged_gym/envs/h1/h1_mimic.py` 与 `h1_mimic_config.py`
- `MarkFzp/humanplus/HST/legged_gym/envs/h1/h1_mimic.py` + `HIT/detr/models/detr_vae.py`
- `LeCAR-Lab/human2humanoid/scripts/data_process/grad_fit_h1.py`
- `OpenTeleVision/TeleVision/teleop/TeleVision.py` + `teleop/robot_control/robot_arm_ik.py`

**练习**
- A1：在 IsaacGym 用 `unitree_rl_gym` baseline 训一个 G1 velocity tracking 策略，sim2real 到 MuJoCo；
- A2：从 AMASS 中选 10 段动作，用 ExBody 的 retargeting 脚本转换到 G1 URDF，可视化对比；
- B1：比较 ExBody vs. ExBody2 中 decouple keypoint/velocity 的 reward 权重；
- B2：阅读 OmniH2O 的 DAgger 蒸馏代码（`train_hydra.py` 中 `config_teleop.yaml`），解释 privileged teacher 观测比 student 多哪些维度；
- B3：阅读 HumanPlus HIT 的 ACT 实现，列出它与 Mobile-ALOHA 的 ACT 的差异；
- C1：ExBody 把下体从 keypoint 跟踪退化到 velocity 跟踪，这在数学上对应哪种"放松"——POMDP？reward hacking？请论证；
- C2：Teacher-Student 和 Behavior Cloning 有何本质区别？为什么在人形上 DAgger 能 work 而直接 BC 常失败？

**预计学习时间**：约 **55-65 小时**（12 论文 × 2h + 代码精读 20h + Isaac 实操 20h）

---

### 复合/240_ASAP_SimToReal　ASAP 与人形 Sim-to-Real：Delta-Action Residual Model

**教学目标**：系统掌握 ASAP 的两阶段 sim-to-real 管线与 delta-action residual model；能在 G1 上复现 C 罗 Siuuu 级敏捷动作；理解为何 delta-action 优于 domain randomization 与 SysID。

**子节（8 个）**
1. Sim-to-Real 历史回顾：DR、SysID、actuator modeling、latency matching
2. Why Delta Action？相比 delta dynamics 的过拟合与计算优势（数学分析）
3. ASAP 阶段一：TRAM 视频 → SMPL → 机器人 → phase-conditioned tracking policy
4. ASAP 阶段二：真机 rollout 收集 → delta-action model 训练 → finetune → 部署丢弃 $\\Delta_\\phi$
5. HumanoidVerse 多模拟器框架（IsaacGym / IsaacSim / Genesis / MuJoCo）设计思想
6. 跨仿真器实验：IsaacGym→IsaacSim、IsaacGym→Genesis、IsaacGym→G1 real
7. 失败案例分析：delta-action 在接触触发时的失效 + curriculum 干预
8. 从 ASAP 到 SONIC：scale up 的开放问题（模型/数据/算力）

**指定论文**：㉒（主）+ ⑯（PHC 作 teacher）+ ⑬⑳㉑（作 baseline 对照）

**指定代码精读**（ASAP 仓库 `LeCAR-Lab/ASAP`）：
- `humanoidverse/envs/motion_tracking/motion_tracking.py`（phase-conditioned env）
- `humanoidverse/config/env/delta_a_open_loop.yaml` 与 `delta_a_closed_loop.yaml`
- `humanoidverse/agents/ppo/ppo.py`（与 rsl_rl 的 diff）
- `sim2real/rl_policy/listener_deltaa.py`（真机 rollout 采集器）
- `scripts/data_process/` 下 SMPL→G1 shape fitting 与 motion retargeting

**练习**
- A1：用 ASAP 仓库训一个 G1 motion tracking policy，目标动作为 AMASS 中任意一段；报告 MPJPE；
- A2：按 `sim2real/rl_policy/listener_deltaa.py` 注释，实现一个"假装 MoCap"的 sim2sim delta-action 实验（IsaacGym→MuJoCo）；
- B1：阅读 `humanoidverse/agents/ppo_ma/` 与 `humanoidverse/agents/ppo/` 对比，找出 ASAP 与 FALCON 的 PPO 差异；
- B2：阅读 ASAP 论文附录 C 的 Algorithm 1，对照代码 `train_delta_action.py` 中的训练循环；
- C1：Delta-action 相比 delta-dynamics 的过拟合风险为何更小？用信息瓶颈论证；
- C2：若把 delta-action 应用到 FALCON（force-adaptive），需要对接触力做什么特殊处理？；
- C3：ASAP 部署时丢弃 $\\Delta_\\phi$ 的合理性——给出一个**可能失败**的场景。

**预计学习时间**：约 **40-50 小时**（核心论文与 2 篇对比 10h + ASAP 代码 30h + 仿真实验 10h）

---

### 复合/250_力敏感人形LocoMani　力敏感人形 Loco-Manipulation：FALCON / SoFTA / HOMIE

**教学目标**：掌握力敏感 loco-manipulation 的两大技术路线——**双智能体 RL（FALCON/SoFTA）**与**同构外骨骼遥操作（HOMIE）**；理解 BFM-Zero 代表的 foundation-model 路线；在 G1 或 Booster T1 上实现负载运输、开门、端啤酒等任务。

**子节（10 个）**
1. 力敏感任务的独特性：外力引入时变干扰与可行性约束
2. Mobile-TeleVision 的奠基：上体 IK + 下体 RL + CVAE 运动先验
3. FALCON 双智能体 RL：共享观测、独立 reward、联合训练的数学
4. FALCON 3D 力课程：Pinocchio RNEA 实时可行性检查
5. SoFTA 频率解耦：100 Hz 上体 + 50 Hz 下体的异步控制
6. HOMIE 同构外骨骼：硬件设计 + 任意上体姿态课程 RL
7. SkillBlender：分层 RL 与 SkillBench benchmark
8. BFM-Zero：Forward-Backward 表示 + 无监督 off-policy RL 做 promptable FM
9. ACE 与 OpenTeleVision 跨平台遥操作对比
10. 开放问题：动态双臂、灵巧手指 + 行走、VLM + WBC 可行性

**指定论文**：㉔㉕㉖㉗㉘㉙㉚㉛

**指定代码精读**：
- `LeCAR-Lab/FALCON/humanoidverse/agents/ppo_ma/` — multi-agent PPO
- `LeCAR-Lab/FALCON/humanoidverse/envs/decoupled_locomotion/`
- `LeCAR-Lab/FALCON/humanoidverse/config/rewards/dec_loco/reward_dec_loco_stand_height_ma_diff_force.yaml`
- `LeCAR-Lab/SoFTA/` 的双频率 `control_decimation` 实现
- `InternRobotics/OpenHomie/` 的 pose curriculum + height tracking reward
- `LeCAR-Lab/BFM-Zero/humanoidverse/agents/bfm/`（FB 表示）

**练习**
- A1：在 G1 仿真中训一个 FALCON-style 双 agent policy，只跟踪单手 0.3 m 水平偏移的 EE 目标 + velocity command，对比 monolithic PPO；
- A2：扩展 FALCON 的 force curriculum，把 3D 力加上低频正弦扰动 $f(t) = f_0 + A\\sin(\\omega t)$，观察 policy 学到的补偿行为；
- A3（大作业）：在 Booster T1（或 G1 仿真）上复现"端啤酒不洒"任务，评测 EE 垂直加速度 $<2$ m/s²；
- B1：阅读 FALCON 论文 Section 4 与代码 `ppo_ma/runner.py`，画出两个 actor 的参数更新流图；
- B2：阅读 HOMIE 论文 Table 2 与 `OpenHomie/legged_gym/envs/` 中的 pose curriculum；
- B3：阅读 BFM-Zero 论文 Appendix B（FB 数学）与 `agents/bfm/fb_model.py`，说明 latent z 如何同时编码 motion / goal / reward；
- C1：FALCON 的双 agent 相比 HOVER 的单网络+masking 的根本优势？给出一个 HOVER 退化但 FALCON 不退化的任务场景；
- C2：把 FALCON 与 SoFTA 合并为"空间+时间双重解耦"，理论上有何新挑战？（提示：Bellman 一致性、off-policy 评估）；
- C3：BFM-Zero 的 unsupervised off-policy RL 能否替代 FALCON 的 supervised reward？分析优缺点。

**预计学习时间**：约 **55-70 小时**（8 论文 × 2-3h + FALCON/SoFTA 代码 25h + 实验 25h）

**复合/220_经典人形全身控制-复合/250_力敏感人形LocoMani 累计**：约 **195-240 小时**（约 5-6 周全职 / 10-12 周半职学习）

---

## 结论：三条判断与路径建议

**判断一**：**RL 已成为人形 loco-manipulation 的事实默认范式，但 MPC+WBC 不可跳过**。ExBody/H2O/HOVER/ASAP/FALCON 全部用 RL，而它们的 reward 设计（CoM tracking、角动量正则、ZMP-proxy）本质仍是把经典 WBC 的 **task-priority 知识**编码为 reward。用户已有 MPC+WBC 基础，可直接进入 RL 阶段但不必放弃 MPC——事实上当前**最前沿的 sim-to-real**（ASAP delta-action）与 **real-time safety filter**（复合/250_力敏感人形LocoMani Q4 VLM+WBC）都需要两栈融合。

**判断二**：**Unitree G1 是 2026 年人形研究的性价比甜点**。$42K-74K 价格、开放 SDK、Isaac Lab / MuJoCo 官方资产、数千台社区规模、ASAP/FALCON/BFM-Zero/HumanoidVerse 全部以 G1 为主验证平台——形成了**数据-代码-硬件三位一体的正反馈**。H1 仍是高速 locomotion 研究的选择（步速纪录），但 loco-manipulation 应首选 G1 EDU U2 或 U3。

**判断三**：**力敏感 loco-manipulation 是 2025-2026 最活跃前沿**。FALCON（L4DC 2026）与 SoFTA（arXiv 2025 年 5 月）在 6 个月内由同一实验室（CMU LeCAR-Lab）推出，分别占据**空间解耦**与**时间解耦**两维；BFM-Zero 补上 foundation-model 维度。但现有工作仍停留在 **单手、低频、准静态力负载**——**高频动态接触、双臂协作、触觉精细操作**仍是开放沃土，与用户的 RL+SLAM+腿足三重背景正好匹配。

复合/220_经典人形全身控制-95 四章按 **经典 WBC → 人形 RL → sim-to-real → 力敏感** 递进，每章都有明确的指定论文、代码精读文件与 G1 实操路径，总学习时长约 200 小时，足以支撑一个研究生学期的人形方向核心课程。完成本序列后，学生应具备在 Unitree G1 上复现 ASAP + FALCON + HOMIE 三大主流工作的能力，并能独立开展第 5 部分 Q1-Q8 中任一开放问题的博士级研究。