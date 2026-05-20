## 四足+机械臂 Loco-Manipulation 研究生教学大纲（复合/160_四足臂动力学概览-91）

> 本报告面向 RL+SLAM+腿足三重背景的研究生，系统梳理 "Quadruped + Arm Loco-Manipulation" 方向的论文谱系、开源栈、数学增量、硬件、研究开放问题与六章教学大纲。论文引用数为 2026 年 4 月基于 Semantic Scholar / Google Scholar 的合理估计，代码 URL 均实际验证。

---

## 第一部分 论文清单（30 篇，三个时代）

### 1.1 经典奠基期（2017–2021，9 篇）

**[1] Khatib 1987 — Operational Space Formulation.** O. Khatib, *IEEE J. Robotics and Automation* 3(1):43–53, 1987. 提出操作空间动力学，将关节空间 `M(q)q̈+b+g=τ` 映射到末端得解耦方程 `Λ(x)ẍ+μ+p=F`，其中 `Λ=(JM⁻¹J^T)⁻¹` 为操作空间惯量，动力学一致伪逆 `J̄=M⁻¹J^T Λ`。冗余控制律 `τ=J^T F+(I−J^T J̄^T)τ₀` 使零空间任务与末端任务解耦。**被引≈6000+**。**无官方代码**（Stanford SAI 非开源；后由 Pinocchio/OpenSoT 实现）。**谱系**：全部 WBC/任务优先控制的理论根。

**[2] Sentis & Khatib 2005 — Whole-Body Behaviors.** L. Sentis, O. Khatib, *Int. J. Humanoid Robotics* 2(4):505–518. 首次提出"约束—操作任务—姿态"三级分层。动力学一致零空间投影 `N_{i|prev}=I−J̄_i^{prev T} J_i^{prev T}`，`τ=Σ_i N_{i|prev}^T J_i^T F_i`。**被引≈650**。**无官方代码**。**谱系**：操作空间→浮基人形 WBC 的中间环节。

**[3] Mistry, Buchli, Schaal 2010 — Orthogonal Decomposition.** *IEEE ICRA* 2010:3406–3412. 对约束雅可比 `J_c` 做 QR 分解 `J_c^T=Q[R;0]`，用 Q 的后 `n+6−k` 列 `S_u` 投影系统动力学 `S_u^T(Mq̈+h)=S_u^T S^T τ`，从而不需估计接触力即可得到解析正确的扭矩解。**被引≈450**。**无官方代码**。**谱系**：浮基 WBC 里程碑，被 Righetti 2011 统一推广。

**[4] Righetti, Buchli, Mistry, Schaal 2011 — Unified View.** *IEEE ICRA* 2011:1085–1090. 核心利用 `J_c` 的 QR 分解得投影 `P=I−J_c^+ J_c`，得 `P(Mq̈+h)=PS^T τ`，消去接触力 λ。导出最优扭矩闭式解，可利用扭矩冗余直接优化接触力分布。**被引≈500**。**无官方代码**（后 `mim_control` 继承）。**谱系**：MIT Cheetah WBIC、ANYmal WBC、qm_control 的直接技术来源。

**[5] Hutter et al. 2016 — ANYmal.** *IEEE/RSJ IROS* 2016:38–44 (IROS Best Paper). 机电贡献：SEA 关节 >70Hz 扭矩带宽，`τ=K(θ_m−θ_l)+D(θ̇_m−θ̇_l)` 实现类理想扭矩源。为分层 QP WBC 提供硬件基础。**被引≈1800**。**代码**：https://github.com/ANYbotics （仅驱动/msgs 公开）。**谱系**：ETH loco-manipulation 谱系（ALMA/Sleiman/Grandia/Ma）的平台节点。

**[6] Winkler et al. 2018 — TOWR.** *IEEE RA-L* 3(3):1560–1567. 提出相位化末端参数化 `p_i(t),f_i(t)`，相位时长 `ΔT_i^k` 作为连续变量。单一 NLP 同时决定步态、步时、落脚点、摆动轨迹、SRBD 基体运动，显式施加摩擦锥 `|f_t|≤μf_n`。**被引≈650**。**代码**：https://github.com/ethz-adrl/towr。**谱系**：直接被 Sleiman 2021/2023 作为接触编码范式借鉴。

**[7] Kim et al. 2019 — MIT Cheetah WBIC+MPC.** arXiv:1909.06586. 双层架构：上层 cMPC 基于 SRBD `Ω̇=Σ_i r_i×f_i/I, p̈=Σf_i/m−g` 在凸 QP 中 30Hz 规划 GRF；下层 WBIC 500Hz 求 `argmin ‖q̈_cmd−q̈‖²+‖δ_f‖²` 分层任务 + 动力学+摩擦锥。关键创新：引入浮基松弛变量，实现飞行相。**被引≈1100**。**代码**：https://github.com/mit-biomimetics/Cheetah-Software。**谱系**：MPC+WBC 双层范式里程碑，定义了此后 legged_control、qm_control 的标准架构。

**[8] Bellicoso et al. 2019 — ALMA.** *IEEE ICRA* 2019:8477–8483. ANYmal B + Kinova 6-DoF 的首次动态行走中操作。将系统建为 `M(q)q̈+h=S^Tτ+J_c^T λ`，ZMP-based 在线 motion planner 产生 CoM 参考与操作空间末端参考并行送入分层 QP。躯干姿态作为冗余自由度优化臂工作空间。**被引≈500**。**无官方代码**（ANYbotics 闭源）。**谱系**：Loco-manipulation 控制开山之作，直接衍生 Sleiman 2021 unified MPC。

**[9] Sleiman, Farshidian, Minniti, Hutter 2021 — Unified MPC.** *IEEE RA-L* 6(3):4688–4695, arXiv:2103.00946. 将 loco-manipulation 建模为 switched system，单一多接触 OCP 同时规划步态、接触力与操作任务。状态由 centroidal dynamics `ḣ_G=Σ_i(p_i−r)×f_i+mg` 与物体 6D 动力学增广组成。SLQ/DDP（OCS2）板载实时求解。**被引≈420**。**代码**：https://github.com/leggedrobotics/ocs2。**谱系**：从"规划+WBC 分层"推向"全身单级 MPC"，后续 Sleiman 2023、Dadiotis 2023 的直接基石。

### 1.2 MPC+WBC 深化期（2022–2024，7 篇）

**[10] Pankert & Hutter 2020 — Perceptive MPC for Mobile Manipulation.** *IEEE RA-L* 5(4):6177–6184. Receding-horizon OCP，成本含末端位姿 6D 误差 `log(T_d^{-1}T)^∨` 平方，约束融合视觉 ESDF 避障 `d(p(q))≥d_min`（用 relaxed barrier function）。Ridgeback+Panda、Husky+UR5 20–50Hz 运行。**被引≈260**。**代码**：https://github.com/leggedrobotics/perceptive_mpc。**谱系**：感知第一次纳入 whole-body MPC。

**[11] Grandia, Jenelten, Yang, Farshidian, Hutter 2023 — Perceptive Locomotion NMPC.** *IEEE T-RO* 39(5):3402–3421, arXiv:2208.08373. 完整感知-规划-控制流水线，ANYmal 100Hz NMPC。高程图→steppability+平面分割+SDF，foothold 可行性为局部凸不等式 `A_k p_f+b_k≤0`；求解采用 real-time iteration + filter line-search（HPIPM/OCS2 SQP）。**被引≈260**。**代码**：https://github.com/leggedrobotics/ocs2 + `elevation_mapping_cupy`。**谱系**：Pankert 2020 在四足上的深化。

**[12] Sleiman, Farshidian, Hutter 2023 — Versatile Multi-Contact.** *Science Robotics* 8(81):eadg5014, arXiv:2308.09179. 将 loco-manipulation 作为 TAMP 问题。外层 informed graph search 生成接触模式序列，内层对每段模式用多接触轨迹优化。实现 ANYmal 自主开洗碗机门、用腿卡弹簧门等无需手工 FSM 的行为。**被引≈280**。**无独立代码**（相关 OCS2 求解器公开）。**谱系**：Unified MPC 的自然延伸，当前 model-based loco-mani SOTA。

**[13] Dadiotis, Laurenzi, Tsagarakis 2023 — 37-DoF Centauro WB-MPC.** *IEEE-RAS Humanoids* 2023, arXiv:2310.02907. 在 CENTAURO（双 7-DoF 臂、4 腿、37 关节）上迄今最高 DoF 实时全身 MPC。状态 49 维（centroidal 动量+浮基+37 关节），输入 55 维。关键创新：绕过瞬时 WBC，MPC 直接对接关节阻抗层 `τ=K_p(q_d−q)+K_d(q̇_d−q̇)+τ_{ff}`。**被引≈45**。**代码**：https://github.com/ADVRHumanoids/wb_mpc_centauro。**谱系**：Sleiman 2021 在双臂四足上的规模化验证，"MPC-only"路线。

**[14] Ma, Farshidian, Hutter 2023 — Arm-Assisted Fall Damage Reduction.** *IEEE ICRA* 2023:12149–12155, arXiv:2303.05486. RL 跌倒保护+恢复策略，非对称 Actor-Critic。Reward 时变（跌落相最小化 `‖v_base‖²+impulse`，恢复相最大化末端高度）。仿真 98.9% 跌姿可成功恢复。**被引≈90**。**无官方代码**。**谱系**：把 arm 从"被补偿负载"变为主动安全执行器。

**[15] Zhang, Lin, Peng, Xiong, Lou 2024 — qm_control Compliance.** *IEEE/RSJ IROS* 2024. Unitree 四足+6-DoF 臂上构建 MPC(SRBD)+HO-QP WBC。关键贡献：电机扭矩饱和 `|τ|≤τ_max` 与地面摩擦锥 `|f_t|≤μf_n` 显式并行置入同一 QP，可行性回退到次优任务。**被引≈35**（代码 >400 星）。**代码**：https://github.com/skywoodsz/qm_control。**谱系**：WBIC + Sleiman 2021 开源中文社区主流实现。

**[16] Molnar et al. 2025 — Whole-Body Inverse Dynamics MPC.** arXiv:2511.19709。将 inverse dynamics 显式作为 MPC 决策变量 `(q̈,τ,λ)` 的一部分，统一加速度-扭矩-力 QP，避免 WBC 层丢失 MPC 的 horizon 优化。**被引≈10**。**预期 T-RO 2025/2026**。**谱系**：Dadiotis 2023 "MPC-only" 思路的 ID 变体。

### 1.3 RL 驱动期（2022–2026，14 篇）

**[17] Fu, Cheng, Pathak 2022 — Deep Whole-Body Control.** *CoRL* 2022 Oral, arXiv:2210.10044. A1+WidowX 250 端到端单策略（19 维动作，12 腿+6 臂+gripper）。两大创新：**Regularized Online Adaptation (ROA)** + **Advantage Mixing**（利用 arm/leg 因果解耦分别估计 advantage）。**被引≈550–650**。**代码**：https://github.com/MarkFzp/Deep-Whole-Body-Control。**谱系**：学习型 loco-mani canonical baseline。

**[18] Liu et al. 2024 — Visual Whole-Body Control (VBC).** *CoRL* 2024, arXiv:2403.16967. B1+Z1 三阶段分层：低层通用 goal-reaching RL → 特权教师规划器 → 深度图学生蒸馏。观测含 body 5D、arm 12D、leg 28D；14 个物体×3 种高度上涌现 retry 行为。**被引≈150–220**。**代码**：https://github.com/Ericonaldo/visual_wholebody。**谱系**：DeepWBC 的视觉+分层分支。

**[19] Ha, Gao, Fu, Tan, Song 2024 — UMI on Legs.** *CoRL* 2024, arXiv:2407.10353. Go2+ARX5。解耦为 UMI 手持采集+Diffusion Policy 高层 + RL 低层 WBC。**task-frame (非 body-frame) 追踪**——WBC 主动补偿 base 抖动。prehensile/non-prehensile/dynamic 三类任务成功率 >70%。**被引≈120–180**。**代码**：https://github.com/real-stanford/umi-on-legs。**谱系**：UMI 数据采集 + DeepWBC 的桥梁，定义"manipulation-centric WBC"。

**[20] Arm, Mittal, Kolvenbach, Hutter 2024 — Pedipulate.** *IEEE ICRA* 2024, arXiv:2402.10837. ANYmal 单策略追踪足端 3D 目标，**用一条腿当 manipulator**。奖励含足端跟踪、接触、tripod gait emergence、base 稳定。开门、采岩样、推障碍；足端承载 >2kg。**被引≈100–150**。**无官方代码**。**谱系**：与"腿+臂"主线正交。

**[21] Ji, Margolis, Agrawal 2023 — DribbleBot.** *IEEE ICRA* 2023, arXiv:2304.01159. Go1 野外运球——**无臂非抓取 loco-mani 先驱**。PPO+Isaac Gym 单策略，对变球动力学（沙、雪、砾石）大幅域随机化。**被引≈250–320**。**代码**：https://github.com/Improbable-AI/dribblebot。**谱系**：MIT 动态非抓取早期探索，奠定 Margolis/Ji 子谱系。

**[22] Portela, Margolis, Ji, Agrawal 2024 — Learning Force Control.** *IEEE ICRA* 2024:15366–15372, arXiv:2405.01402. **无力传感器的直接力控**——在仿真中对 EE 施加与期望力**等值反向**的外力隐式逼迫策略产生对应接触力。**被引≈80–130**。**无官方独立代码**（项目页 tif-twirl-13.github.io/learning-compliance）。**谱系**：DeepWBC 位置→力域扩展，RAMBO/FALCON 先驱。

**[23] Pan et al. 2024/2025 — RoboDuet.** *IEEE RA-L* 2025, arXiv:2403.17367. 清华/上海 AI Lab/UCSD。**双策略协作架构**（loco+arm），通过"指挥信号"交互；两阶段训练（固定 loco 训 arm 实现跨本体零样本）。成功率+23%。**被引≈70–100**。**代码**：https://github.com/locomanip-duet/RoboDuet。**谱系**：Fu 2022 的直接后继，"cooperative two-policy"替代 unified。

**[24] Lin et al. 2024 — LocoMan.** *IROS* 2024, arXiv:2403.18197. CMU/UW/Google DeepMind。**硬件创新**：Go1 小腿加装两个轻量 3-DoF 模块化 manipulator 形成双"calf-arm"。5 模式切换；**基于模型 WBC（非 RL）**。**被引≈60–90**。**代码**：https://github.com/linchangyi1/LocoMan。**谱系**：不用顶装 arm 思路。

**[25] Wang et al. 2024 — Arm-Constrained Curriculum.** *IROS* 2024 Oral, arXiv:2403.16535. 清华/HKUST。轮腿+臂的**CMDP + 游戏式课程学习**。完成开门、扇叶拨动、动态接力棒抓取。**被引≈30–50**。**代码**：https://github.com/aCodeDog/legged-robots-manipulation。**谱系**：首个 wheel-legged+arm 的 safe RL + curriculum。

**[26] Wu, Fu, Cheng, Wang, Finn 2024 — Helpful DoggyBot.** arXiv:2410.00231. Stanford/UCSD。**前腹 1-DoF "嘴式"夹爪 + RL whole-body + VLM**。第三视角 fisheye + egocentric RGB，经 SAM2/Florence2/GroundingDINO+GPT-4o 生成目标追踪；纯 sim 零样本到真实卧室爬床跳沙发取球。**被引≈70–110**。**代码**：https://github.com/WooQi57/Helpful-Doggybot。**谱系**：Extreme Parkour 的 manipulation 延伸。

**[27] Duan, Zhuang, Zhao, Schwertfeger 2024 — Playful DoggyBot.** arXiv:2409.19920. 上海 Qi Zhi/ShanghaiTech。**感知-控制解耦 RL**高动态抓取（跳跃 catch 飞球，仿真 1.05m/真机 0.8m，站立 0.3m 高）。被动机械夹爪。消融 MLP/RNN/Transformer。**被引≈30–50**。**代码**：https://github.com/playful-doggybot/playful-doggybot。**谱系**：Robot Parkour 延伸至动态 manipulation。

**[28] Portela, Cramariuc, Mittal, Hutter 2024/2025 — Whole-Body EE Pose Tracking.** ICRA 2025, arXiv:2409.16048. ETH/NVIDIA。ANYmal+6-DoF 臂。解决 workspace 限机身前方、只追 position 不追 orientation。核心：terrain-aware 采样 + game-based curriculum + 单 PPO 同时控 18 joints。软床垫/楼梯/不规则地面 cm 级 6D 精度。**被引≈50–80**。**未公开 repo**（依托 Isaac Lab）。**谱系**：成为 2025"whole-body 6D EE tracking"新标杆。

**[29] Cheng, Kang, Fadini, Shi, Coros 2025 — RAMBO.** *IEEE RA-L* 2025, arXiv:2504.06662. **MB+RL 混合**：QP 反作用力优化生成前馈扭矩，RL 输出反馈修正；两者相加至 torque 层。Go2 双足/四足模式：推购物车、平衡盘子、持软物。**被引≈30–60**。**代码**：https://github.com/catachiii/rambo。**谱系**：2025 年 model-based+learning 混合化的收敛节点。

**[30] Zhang et al. 2025 — Catch It!** *IEEE ICRA* 2025, arXiv:2409.10319. 清华/上海 Qi Zhi/Stanford。mobile base+6-DoF arm+12/16-DoF 灵巧手统一 catching。两阶段：base+arm 追抛物线 → 冻结 arm 训灵巧手抓取时机。**被引≈25–40**。**代码**：https://github.com/hang0610/Catch_It。**谱系**：首次灵巧手 catching 与 mobile base whole-body RL 耦合。

**附加：HYPERmotion (Wang et al. CoRL 2024, arXiv:2406.14655, IIT)**：CENTAURO 38-joint 的 **LLM+RL 分层**，motion library + behavior tree + morphology selector。被引≈40–60，无完整开源。**HiLMa-Res (Huang et al. IROS 2024, arXiv:2407.06584, Berkeley)**：**CPG+Bézier residual RL**，dribbling/障碍/推载三任务。被引≈25–40，未开源。**MLM (Liu 2025, arXiv:2508.10538)**：Fast-UMI 采集 real-world 轨迹 + 自适应 curriculum 多任务 whole-body。被引≈10–20。

---

## 第二部分 开源项目深度（8 个核心项目）

### 2.1 skywoodsz/qm_control — 唯一完整四足+臂 MPC+WBC
- **URL**：https://github.com/skywoodsz/qm_control **Star/Fork**：≈310 / 47
- **最近 commit**：2024 末 IROS 录用后停滞
- **目录**：`qm_common/` `qm_control/` `qm_controllers/`（ros_control 插件）`qm_description/`（A1+6DoF arm URDF）`qm_estimation/`（线性 KF）`qm_gazebo/` `qm_interface/`（**OCS2 NMPC 问题构造**：centroidal+arm）`qm_wbc/`（**分层 QP WBC**，qpOASES）`qpoases_catkin/`
- **依赖**：ROS1 Noetic + Ubuntu 20.04 + OCS2（需独立编译 ocs2+pinocchio+hpp-fcl+ocs2_robotic_assets）+ Gazebo Classic 11
- **四分支**：`main`（无末端外力）/ `feature-force`（受力稳定）/ `feature-compliance`（柔顺）/ `feature-real`（真机，未合并）
- **论文**：Zhang et al. IROS 2024 "Whole-body Compliance Control..."
- **语言**：C++ 93.8% / CMake 4.6% / Python 1.6%
- **局限**：奇异点未解；脚轨迹规划缺失；OCS2 依赖链繁重；仅 A1+6DoF；issues=0（作者不维护）
- **阅读时间**：快速浏览 3h；复现仿真 10–15h；深度修改 40–80h
- **Sim2Real**：仅 Gazebo；`feature-real` 未验证

### 2.2 MarkFzp/Deep-Whole-Body-Control
- **URL**：https://github.com/MarkFzp/Deep-Whole-Body-Control **Star/Fork**：≈365 / 33
- **最近 commit**：4 次（论文发布后一次性 dump，2022 末）
- **目录**：`legged_gym/`（IsaacGym，`envs/widowGo1/widowGo1_config.py` + `widowGo1.py`）`rsl_rl/`（定制 PPO+**ROA+Advantage Mixing**）`widowGo1/`（URDF+mesh，Go1+WidowX 250s）
- **观测/动作**：本体感受+base/EE 目标+历史；动作 19 维关节位置
- **依赖**：Python≥3.8，PyTorch CUDA，**IsaacGym Preview 3/4**（≥10GB VRAM），无 ROS，无 requirements.txt
- **论文**：Fu, Cheng, Pathak CoRL 2022
- **语言**：Python 99.9%
- **局限**：无真机部署代码；9 open issues；仅 Go1+WidowX；无视觉；IsaacGym 已 deprecate
- **阅读时间**：快速浏览 2h；复现训练 8–12h；深度修改 30–60h

### 2.3 Ericonaldo/visual_wholebody
- **URL**：https://github.com/Ericonaldo/visual_wholebody **Star/Fork**：≈389 / 37
- **最近 commit**：2024 下半年 CoRL 后零星修复
- **目录**：`high-level/`（teacher-student 深度图）`low-level/`（通用目标追踪，legged_gym 基础）`third_party/`（isaacgym/rsl_rl/skrl）
- **依赖**：conda Python 3.8（IsaacGym 硬性要求）+ PyTorch + IsaacGym Preview 4 + rsl_rl/skrl + pydelatin/imageio-ffmpeg/opencv/wandb
- **三阶段**：RL 低层 → RL 教师 → 在线模仿蒸馏深度图学生
- **论文**：Liu et al. CoRL 2024
- **语言**：Python 100%
- **局限**：IK 失败部分目标不可达；depth 对照明敏感；Z1 夹爪设计不佳；switch 机器人需改多处；15 open issues
- **阅读时间**：快速浏览 3h；复现低层 12–20h；完整高层 30–40h；深度修改 60–100h
- **Sim2Real**：真机 zero-shot B1+Z1 成功，但仓库不含真机代码

### 2.4 real-stanford/umi-on-legs
- **URL**：https://github.com/real-stanford/umi-on-legs **Star/Fork**：≈487 / 27
- **目录**：`mani-centric-wbc/`（核心 WBC 训练，基于 legged_gym + IsaacGym）`real-wbc/`（devcontainer/docker 真机）`arx5-sdk/`（submodule，yihuai-gao）`iPhoneVIO/`（submodule，iOS ARKit app）
- **依赖**：conda+PyTorch+CUDA+IsaacGym+Docker+VSCode Dev Containers；硬件 Go2+ARX5+GoPro+iPhone
- **论文**：Ha et al. CoRL 2024
- **语言**：Python 97.4% / Dockerfile 2.2% / CMake 0.4%
- **局限**：iPhone ARKit >100ms 延迟必须训练时建模；Go2 电机热保护触发；仅单末端追踪；ARX5 精度弱于谐波驱动；ORB-SLAM3 是 UMI 数据管线最脆弱环节
- **阅读时间**：快速浏览 3h；复现 rollout（下载 checkpoint 播放）2–4h；从零训练 WBC 15–25h；复现完整系统 80+h
- **Sim2Real**：**设计目标**——真机代码/3D 打印件/装配指南齐全

### 2.5 catachiii/rambo (RAMBO)
- **URL**：https://github.com/catachiii/rambo（非 jin-cheng-me/rambo，原任务 URL 错误）**Star/Fork**：≈112 / 5
- **目录**：`source/isaaclab/` `source/isaaclab_assets/` `source/crl2/`（in-house PPO）`scripts/reinforcement_learning/crl2/train.py, play.py`（任务 `Isaac-RAMBO-Quadruped-Go2-v0`, `Isaac-RAMBO-Biped-Go2-v0`）
- **依赖**：Python 3.10+conda+**Isaac Sim 4.5.0**+Isaac Lab（源码）+**qpth**（可微分 QP 用于 feedforward torque）+wandb+CUDA
- **论文**：Cheng et al. RA-L 2025
- **语言**：Python 98.8%
- **局限**：社区小；CC BY-NC 4.0 非商用；Isaac Sim 版本锁死；仅 Go2；真机代码未发布；0 issues
- **阅读时间**：快速浏览 2h；复现训练 10–15h；深度修改 40–60h
- **Sim2Real**：论文有 shopping cart/plate balancing 真机 demo，但仓库仅 sim

### 2.6 aCodeDog/legged-robots-manipulation
- **URL**：https://github.com/aCodeDog/legged-robots-manipulation **Star/Fork**：≈359 / 33
- **目录**：`loco_manipulation_gym/` 含 airbot/go2_arx/aliengo_z1/b2w_z1/b2w 多机 cfg；`resources/`（URDF/mesh）
- **依赖**：IsaacGym Preview 4 + rsl_rl + legged_gym + Python 3.8 + CUDA
- **训练**：`python train.py --task=b2w --rl_device=cuda:0`
- **论文**：Wang et al. IROS 2024 Oral
- **语言**：Python 100%
- **局限**：自承"partial implementation"；IsaacLab/vision 版计划中；8 open issues 涉 URDF 错误、curriculum 收敛
- **阅读时间**：单机训练 6–10h；4 机 20–30h；深度修改 40–60h

### 2.7 Pedipulate / LeCAR-Lab 相关
- **注意**：任务假设 LeCAR-Lab(CMU) 有 pedipulate 仓库**错误**。Pedipulate 为 ETH Philip Arm 工作，**官方未开源独立代码**
- **项目页**：https://sites.google.com/leggedrobotics.com/pedipulate
- **论文**：Arm et al. ICRA 2024（arXiv:2402.10837）+ 后续 Stolle et al. "Perceptive Pedipulate" Humanoids 2024
- **实现思路推测**：IsaacGym+rsl_rl，PPO 追踪足端 3D 目标，curriculum 采样+domain randomization
- **阅读时间**：论文 1–2h；从零 IsaacLab 复现 40–80h
- **LeCAR-Lab 替代**：`LeCAR-Lab/SoFTA`（force-adaptive humanoid loco-mani）或 `LeCAR-Lab/dial-mpc`（sampling-based MPC 支持四足）

### 2.8 leggedrobotics/ocs2
- **URL**：https://github.com/leggedrobotics/ocs2 **Star/Fork**：≈1.3k / 308
- **最近 commit**：长期活跃；最新 release `v12.0`（2023-07）；`ros2` 分支已 port
- **关键目录**：
  - `ocs2_core/`（数据类型+自动求导）
  - `ocs2_oc/`（OCP building blocks）
  - `ocs2_ddp/` `ocs2_mpc/`（SLQ/iLQR+MPC runtime）
  - `ocs2_sqp/` `ocs2_slp/` `ocs2_ipm/`（HPIPM/BLASFEO、PIPG、IPM）
  - `ocs2_pinocchio/`（URDF→centroidal/kinematics/self-collision HPP-FCL）
  - **`ocs2_robotic_examples/`**：
    - `ocs2_legged_robot/` + `ocs2_legged_robot_ros/`（ANYmal centroidal switched system + SwitchedModelReferenceManager）
    - `ocs2_mobile_manipulator/` + `ocs2_mobile_manipulator_ros/`（Franka/Kinova j2n6,j2n7/MABI-Mobile/PR2/Ridgeback-UR5，kinematic 模型+EE 6D 跟踪+self-collision）
  - `ocs2_mpcnet/`（MPC-Net 学习策略）
  - `ocs2_perceptive/` `ocs2_raisim/`
- **依赖**：ROS1 Noetic 或 ROS2；Pinocchio+HPP-FCL+Eigen+CppAD/CppADCodeGen+HPIPM/BLASFEO
- **语言**：C++ 94.9%
- **局限**：52 open issues；MPC-Net 扩展 mobile_manipulator 收敛难；编译复杂；文档陡；ROS2 port 完善中
- **阅读时间**：快速浏览 5h；复现仿真 demo 15–25h；深度修改 80–200h
- **Sim2Real**：无闭环（需接入 WBC+状态估计+硬件 SDK，参考 qm_control / qiayuanl/legged_control）

**Star 排序**：ocs2(1.3k) ≫ umi-on-legs(487) > visual_wholebody(389) > Deep-WBC(365) > legged-robots-manipulation(359) > qm_control(310) > rambo(112) > Pedipulate(无 repo)。

---

## 第三部分 数学深入（四足+臂特有增量）

### 3.1 浮动基座+腿+臂统一动力学

设广义坐标 `q = [q_b; q_ℓ; q_a] ∈ SE(3)×R^{n_ℓ}×R^{n_a}`，四足 `n_ℓ=12`，6-DoF 臂 `n_a=6`，总维度 `n=6+12+6=24`。全身约束动力学：

```
M(q) v̇ + h(q,v) = S^T τ + Σ_{i∈C_foot} J_{c_i}^T(q) λ_i + J_ee^T(q) f_ee
```

- `M(q)∈R^{n×n}`：Pinocchio `crba` O(n)；`h(q,v)=C(q,v)v+g(q)`：`rnea` 计算
- `S=[0_{(n_ℓ+n_a)×6}, I_{n_ℓ+n_a}]`：选择矩阵，前 6 维（浮基）不可驱动
- `λ_i∈R³`：支撑脚 GRF（point contact）
- `f_ee∈R^6`：末端 wrench

**三维扩展**：(1) 可驱动 DOF 12→18；(2) 雅可比堆叠增加 `J_ee∈R^{6×n}`（浮基列非零——基座移动即末端移动）；(3) `f_ee` 分两类：被动扰动（动量观测器估计）vs 主动接触（加入 `J_ee v=v_ref` + 摩擦锥）。

**Sleiman 2021 增广**：被操作物体动力学 `M_o v̇_o + h_o = −R f_ee`（R 为 EE→物体变换），从而物体是系统状态。

### 3.2 任务优先级冲突

**严格分层（Siciliano–Slotine）**：
```
v̇ = A_1^+ b_1 + N_1 A_2^+ (b_2 − A_2 A_1^+ b_1) + N_{1|2} A_3^+(⋯) + ⋯
```
其中 `A_1^+=A_1^T(A_1 A_1^T)^{-1}`，`N_1=I−A_1^+ A_1`，`N_{1|2}=N_1(I−(A_2 N_1)^+ A_2 N_1)`。

**层次化 QP（Escande–Mansard–Wieber）**：每一级在上一级最优解集合（松弛）中求解：
```
min_{v̇,f,s_k} ‖s_k‖²  s.t.  A_k v̇ = b_k + s_k,  {A_j v̇ = b_j + s_j*}_{j<k}
```

**软加权 WBC**：`min Σ_i w_i ‖A_i v̇ − b_i‖² s.t. 动力学/摩擦锥`。

**典型优先级**：0（硬，动力学+摩擦锥+扭矩限）→1（基座姿态 roll/pitch）→2（支撑脚 no-slip）→3（CoM/角动量）→4（EE pose tracking）→5（姿态 regularizer）。

**权重自适应**：`w_EE` vs `w_base` 冲突——接近目标时升 `w_EE`，行走相降。工程实践：MPC 软约束+WBC 硬约束分层（Pankert/Hutter）。

### 3.3 动态接触集管理

OCS2 `legged_robot` 把 gait 编码为 mode schedule `σ(t)∈{0,1}^4`。加入臂后扩展为 `σ(t)∈{0,1}^4×{0,1}^{n_ee}`。

**接触雅可比堆叠**：`J_c^(σ)(q) = vstack{J_i(q) : σ_i=1}`，约束 `J_c^(σ) v=0`（no-slip）与加速度级 `J̇_c v + J_c v̇ = 0`。

**摩擦锥**：`λ_i ∈ F_i = {λ : λ_n≥0, √(λ_t²+λ_o²) ≤ μ_i λ_n}`。EE 推门通常 `μ_door≈0.3`，远低于脚 `μ_foot≈0.7`。

**Switched System OCP**：
```
min_u Σ_{k=0}^{N-1} ℓ^(σ_k)(x_k,u_k) + Φ(x_N)
s.t. ẋ = f^(σ(t))(x,u),  每当 σ 切换状态需满足 reset map
```
OCS2 用 SLQ/DDP 对每个 mode 内光滑优化，event times 划分 mode。

**推门例**：mode 从 `[LF,RF,LH,RH]` 扩展为 `[LF,RF,LH,RH,EE]`；EE=1 相位内 `J_ee` 加入 `J_c^(σ)`；松手时 `λ_ee=0`。Sleiman 2023 实现了 versatile multi-contact，腿可当手用。

### 3.4 臂反力矩对基座稳定性

**摩擦锥收紧**：臂拉向上物反作用力向下压基座 → 摩擦锥扩大；臂顶重物向上（撑门顶）→ 法向载荷减小：
```
F_{n,foot,i} ≈ (m_total g − F_{z,arm,reaction}) / n_stance
μ_eff = μ · (F_{n,foot} − ΔF_arm) / F_{n,foot}
```

**ZMP 偏移公式（教学导出）**：
```
x_zmp = x_com − (z_com/g) ẍ_com + τ_{arm,y}/(mg) − z_arm F_{arm,x}/(mg)
```
其中 `τ_{arm,y}` 为臂绕 pitch 轴反力矩，`z_arm` 为臂质心相对脚面高度。

**支撑多边形变化**：臂外伸使全身 CoM 水平移动；操作姿态让 CoM 逼近多边形边界时动态裕度骤降。ALMA 在 WBC 加入 support-polygon 约束——把 CoM 水平投影约束在 Minkowski 收缩后的内多边形（等效 robust static stability）。

### 3.5 角动量调节与 push recovery

**CMM**：`h_com(q,v) = [m·ẋ_com; L_com] = A_G(q) v`，Pinocchio `computeCentroidalMomentumMatrix` 计算。加臂后 CMM 列数 18→24，臂速度对 `L_com` 贡献：`L_arm = A_{G,arm}(q) q̇_a`。

**Capture Point**：`x_cp = x_com + ẋ_com √(z_com/g)`。进入支撑多边形即可站住。

**臂惯量耦合**：扰动 `t_0` 时臂大幅挥舞可在不动脚的前提下改变 `L̇_com`——等效反惯量轮：
```
ΔL_com = ∫_0^T Σ_i (r_i × f_i) dt + ΔL_arm-swing
```
教学建议：Isaac Lab 训练臂-aware push recovery，对比 arm-frozen vs arm-swing 两种策略的 capture rate。

### 3.6 冗余度利用（18+DOF 零空间）

主任务 `J_1∈R^{m_1×n}`（EE 6D，m_1=6），24 DOF 系统冗余度 ≈ 24−6−6(浮基) = 12。

**零空间投影**：`v̇ = J_1^+ ẋ_1 + (I − J_1^+ J_1) v̇_0`，二级 `v̇_0` 可用于可操作度最大化/限位避免/碰撞避免。

**数值稳定性**（24 DOF 下关键）：
- **阻尼最小二乘**：`J^† = J^T(JJ^T + λ²I)^{-1}`，λ 随条件数变大
- **SVD 截断**：`J=UΣV^T`，σ<阈值（典型 10^-3）置零
- **Jacobian 正则化**：QP 加 `‖q̇‖²` 正则

**HQP vs 单层 QP（24 DOF 对比）**：单层加权 QP <1ms（OSQP/qpOASES）；HQP 3–10ms（严格优先级）；Cascaded QP+active-set 2–5ms。教学建议：单层加权起步；竞赛/论文用 HQP。

**Pinocchio 时变雅可比**：`computeJointJacobians(model,data,q)` 得 J；`computeJointJacobiansTimeVariation(model,data,q,v)` 得 dJ/dt，WBC 加速度级 `J v̇ + J̇ v = ẍ_ref` 所必需。

### 3.7 qm_control 的 OCS2 OCP 构造

qm_control 把 `ocs2_legged_robot` 扩展为 quadruped+arm unified OCP，采用 centroidal dynamics 模板（Sleiman 2021）。

**状态向量**（30 维）：`x = [h_com(6), q_b(6 浮基), q_ℓ(12), q_a(6)]`

**输入向量**（30 维）：`u = [λ_foot(12 脚力), q̇_ℓ(12 腿速), τ_a 或 q̇_a(6)]`

**Flow map**：
```
ḣ_com = Σ_i p_i × λ_i + (0,mg)^T + EE wrench terms
q̇_b = map(h_com, q, v)  (centroidal→基座速度)
q̇_ℓ = q̇_{ℓ,input}
q̇_a = q̇_{a,input}
```

**Cost**：`ℓ = ‖x_base − x_base,ref‖²_{Q_b} + ‖T_ee(q) ⊖ T_ee,ref‖²_{Q_ee} + ‖u‖²_R`，其中 `⊖` 是 SE(3) 左不变误差 `log(T_ref^{-1} T(q))∈se(3)`。

**约束**：
- 等式：`Σλ_i = ḣ_com,linear`；摆动相 `λ_i=0`；swing foot z 跟踪 ref
- 不等式：摩擦锥 pyramid 近似 `|λ_{i,x/y}| ≤ μλ_{i,z}/√2`；臂关节限位；扭矩/速度限
- self-collision：OCS2 HPP-FCL 距离梯度

**求解器**：SLQ(continuous DDP+AL) 或 SQP；MPC horizon 1.0s，50Hz 更新。

**WBC 下层**：OCS2 输出 `λ,q̇,q` reference；WBC（单层加权 QP，500Hz）转关节扭矩并硬约束摩擦锥+扭矩限。

---

## 第四部分 硬件平台详细规格

| 平台 | 机身质量 | 行走载荷 | 臂 payload | 总 DOF | IP | 开源度 | 参考价(USD) |
|---|---|---|---|---|---|---|---|
| Go2 + Z1 | 15 kg | 8 kg | 2–5 kg | 19 | 低 | 高 | 15–30k |
| B1 + Z1 | 50 kg | 20–40 kg | 3–5 kg | 19 | IP67/68 | 中 | ~120k |
| B2 + Z1 | 60 kg | >40 kg | 3–5 kg | 19 | IP67 | 中 | ~130k |
| ANYmal D+DynaArm | 50 kg | 10–15 kg | 6(–14) kg | 19 | IP67 | 中（会员） | ~200k+ |
| Spot + Spot Arm | 32.5 kg | 14 kg | 11 kg | 19 | IP54 | 低 | ~200k |
| HyQReal + dual arm | 130–140 kg | 数十 kg | 5 kg/臂 | 26 | 防尘水 | 无 | — |
| Aliengo + Z1 | 20 kg | 10 kg | 3 kg | 19 | — | 高（社区） | ~40k |
| X30 + 臂 | 56 kg | ~20 kg | 未公开 | 19 | IP67 | 中 | ~75k+ |
| Go2 + ARX5 | 18.4 kg | 1.5 kg | 1.5 kg | 19 | 低 | 极高 | <20k |

**关键细节**：

- **Unitree Go2+Z1**：Go2 EDU 15kg，尺寸 700×310×400mm，最高 3.7m/s，关节峰值 45N·m；Z1 4.1–4.3kg，reach 740mm，6 DOF，~0.1mm 重复精度，Ethernet/RS485，24V/>20A。总 DOF 19（不含浮基）。MPC 50–100Hz，WBC 200–500Hz，电机 1kHz。**用途**：教学首选（社区活跃、URDF 完善）。**局限**：Z1 4.3kg 对 Go2 8kg 载荷挤占；摆动强扰时 Go2 难稳定站立（UMI-on-Legs 文档记载需换 AlienGo/A1）。

- **Unitree B1+Z1**：B1 50kg，1126×467×636mm，站立 80kg/行走 20kg 载荷，IP67（可选 IP68 涉水 1.2m）；膝关节 210N·m；5×RealSense D430+3×Jetson Xavier NX。**用途**：Sleiman 类工作中国版对标。

- **Unitree B2+Z1**：B2 60kg，站立载荷 120kg/行走>40kg，45Ah/2250Wh 续航 4–5h（20kg 载），髋膝 360N·m，IP67 温度 -20~55°C，最大跳距 1.6m，最高速度 >6m/s。**用途**：重载 loco-mani、野外救援。

- **ANYmal D+DynaArm**：ANYmal D ~50kg，IP67，爬梯 45°踢面 230mm，~1m/s；DynaArm 6 DOF ~8–9kg，6kg（新版 14kg）payload，EtherCAT 原生，Quasi-Direct Drive 高透明度 backdrivable。EtherCAT 400Hz–1kHz，MPC 50–100Hz。ANYmal Research 会员制代码；Isaac Lab/Orbit 支持。**用途**：业内标杆（ALMA/Sleiman/Grandia 等大量论文）。

- **Boston Dynamics Spot+Spot Arm**：Spot 32.5kg，1.6m/s，90min 续航，IP54；Spot Arm 6 DOF 8kg，抬 11kg/0.5m 伸展持续 5kg/拖 25kg，末端 10m/s，夹爪集成 4K+LiDAR+IMU。**局限**：底层 API 封闭（10–50Hz 高层），**不能写 WBC/MPC**，不适合控制算法研究。

- **IIT HyQReal + IIT-INAIL dual arm**：130–140kg 液压（Moog ISA，钛 3D 打印本体），关节 300N·m；每臂 10kg 自重、5kg payload，头部双臂配置。总 DOF=12+2×6+2=26。**局限**：非商业；液压维护复杂；体积笨重。

- **Unitree Aliengo+Z1**：19–22kg，650×310×600mm，~1.5m/s，12600mAh 317.52Wh 续航 2.5–4.5h，关节 40N·m。qm_control 默认平台；UMI-on-Legs 原型用（现 Go2 接替）。**局限**：Aliengo（2019 款）电机易损返修慢，已被 B1/B2/Go2 接替。

- **DeepRobotics Jueying X30**：56kg，72V 可换电池续航 2.5–4h 里程≥10km，≥4m/s，IP67 -20~55°C，爬坡 45° 障碍≥200mm，多 LiDAR+RGB+红外热像+自主充电桩。X30 Pro ~74.8k USD。**用途**：国产替代、电力巡检最多；**局限**：学术支持弱；臂规格不透明。

- **Unitree Go2+ARX5（UMI-on-Legs 标准配置）**：ARX5 6 DOF 3.35kg reach 620mm payload 1.5kg，行星齿轮（精度低于 Z1 谐波但够用），USB/50Hz。iPhone ARKit 60Hz ~100ms 延迟作 VIO（挂 Go2 背），GoPro 腕视觉。整套 <20k USD。**用途**：diffusion policy+UMI 数据采集。

---

## 第五部分 博士研究开放问题（8 个方向）

### 5.1 感知–操作–运动闭环统一（Visual SLAM + Perceptive MPC + WBC 三层）

**核心问题**：当前 Perceptive Locomotion 把 LiDAR 高度图作为 MPC 地形约束，但**操作目标感知（物体 6D 位姿、语义边界）仍与 locomotion 感知解耦**。如何让一套统一的 SLAM+语义 pipeline 既提供地形 SDF 给 foothold，又提供物体姿态给 EE 优化，且在同一 WB-MPC 中消化？

**切入点（利用三重背景）**：SLAM 背景——把目标物体建模为带 SE(3) 状态的地标，做在线 BA 与轨迹优化共享 information matrix；RL 背景——训练 perception-policy 从原始点云输出可接触表面+抓取候选作为 MPC warm-start；腿足——OCS2 中加入物体 pose 不确定性的 chance constraint。

**可能突破**：**Differentiable perception-action pipeline**——SLAM uncertainty 作为 MPC cost 权重，逆向指导机器人 active sensing。

### 5.2 接触丰富的 loco-mani：Force-aware RL+MPC 混合

**核心问题**：纯 RL 策略在 contact-rich 任务（开门、转阀、插插头）力控精度差；纯 MPC 要求精确接触模型。如何让 MPC 处理已知接触（foot+主接触面），RL 处理剩余残差（模型不确定、动态摩擦）？RAMBO、Molnar 2025 已有雏形。

**切入点**：RL residual policy 加在 MPC 输出上，reward 含力跟踪；WBC 显式加接触力软约束与 RL 输出 δ 力加法融合；视觉+腕 F/T 做 online impedance 识别。

**可能突破**：**Hybrid force-position MPC**——QP 内同时优化 position 目标和 wrench ref，RL 残差只修正 impedance gain (K,D)，样本效率高。

### 5.3 长时程多阶段任务：LLM/VLM + WB-MPC

**核心问题**："把抽屉里的工具拿给我"需：导航→找抽屉→开抽屉→识别工具→抓取→返回→交付。LLM 可分解子任务，但每个子任务成功条件需可验证的物理反馈，而非语言。如何把 VLM 子目标与 WB-MPC cost/constraint 接口标准化？

**切入点**：SLAM 语义拓扑图作 LLM 可查世界模型；VLM 生成 dense reward 监督每 skill RL；为每类 skill 设计参数化 MPC（参数化 cost weights+mode schedule），LLM 只输出参数。

**可能突破**：**Behavior Tree + MPC as leaf action**；LLM 做 symbolic planner，叶节点是可配置 WB-MPC；前/后置条件用 VLM 验证。

### 5.4 动态物体操作：抓取运动物体+全身补偿

**核心问题**：抓滚动球/接抛袋/对接移动 AGV——目标物体在非零速度下接触。要求同时：目标速度预测+相对 EE 轨迹+全身动量补偿。四足底盘惯性小的劣势在此放大。Whole-Body Dynamic Throwing (arXiv:2410.05681) 只做单向抛出，抓取基本空白。

**切入点**：视觉伺服+运动物体 KF 作 EE ref；MPC 加"预测性动量补偿"——提前调整 CoM 使接触冲量后仍在支撑多边形；ANYmal+DynaArm 做实验。

**可能突破**：**Predictive Impulse Shaping**——接触瞬时冲量 `Δp=∫f dt` 作 MPC 约束，通过改变瞬时姿态将冲量方向对齐支撑多边形最长轴。

### 5.5 多机器人协作搬运

**核心问题**：多台 quad-manipulator 共抬长管/桌子。需同步 grasp 位姿+双机步态+物体柔性在线识别。分布式 vs 集中式控制？固定基座双臂协作研究较多，"移动基×接触闭环"理论不完整。

**切入点**：anchor SLAM 多机一致位姿估计（Kimera-Multi）；分布式多智能体 PPO+通信通道；双机联合 MPC 用 consensus ADMM 分解，每机解自己 OCP 只交换物体力矩。

**可能突破**：**Impedance Consensus**——双机通过物体建立 virtual coupling，阻抗参数 consensus 达成，允许异构协作（Spot+ANYmal）。

### 5.6 Sim-to-Real for 力敏感 WBC

**核心问题**：当前 RL-WBC 仿真 3cm 级跟踪，但接触任务下 sim2real gap 暴涨——仿真刚体 vs 真机柔顺地面、电机摩擦、Z1/DynaArm backlash。如何构建面向力敏感任务的 sim2real pipeline，不只是 state-action randomization？

**切入点**：可微分仿真（Brax/MJX/Genesis）+gradient-based SysID；保留 MPC 作 privileged oracle，residual RL 只学偏差；视觉+IMU 联合估计环境刚度（每种表面 LSTM model）。

**可能突破**：**Contact-conditioned Domain Adaptation**——分阶段"空载"→"载物"→"接触"数据，训练接触类型 condition 的 adaptation module，部署时前 0.5s 接触信号自动切换。

### 5.7 角动量驱动的敏捷操作

**核心问题**：多数 loco-mani 假设"准静态"。HyQReal 拉飞机、ANYmal 翻身展示了动态-to-manipulation 潜力。如何显式把 `L_com` 作为 MPC 决策变量，让机器人"甩臂助推"完成单脚站立插电、或"扭身投掷"？

**切入点**：OCS2 cost 加 `w_L ‖L_com − L_ref‖²`，`L_ref` 由任务层给；RL 学 `L_ref` 生成器避免人工设计；臂是最重可控惯量→摆臂贡献 `L_arm` 最大。

**可能突破**：**Momentum-Centric WBC**——顶层换成 `L̇_com` 跟踪，其他任务从属；使四足"半腾空"操作可能。

### 5.8 触觉/腕力反馈+RL 残差控制

**核心问题**：WBC 输出 joint torque，但手部接触物理状态（滑动、变形、摩擦变化）不通过本体感知进入控制回路。如何把腕 F/T+触觉阵列（SoftBubble/GelSight）高维信号编码为 low-dim latent，作 RL residual policy 输入以修正 MPC impedance？

**切入点**：MAE/DINO 把触觉压到 32-dim；MPC 输出 nominal impedance，RL 残差在触觉 latent 上输出 δK/δD/δforce_ref；视觉+触觉融合物体状态估计。

**可能突破**：**Tactile-in-the-Loop WBC**——触觉触发 event-based 重规划，MPC horizon 内某些接触力自动 soft→hard 切换实现"抓紧"。

### 博士路径建议

**Year 1**：选 Go2+Z1 / Go2+ARX5 搭 baseline，复现 qm_control / UMI-on-Legs；打通 OCS2+Pinocchio 教学链。
**Year 2**：锁定 5.2 或 5.6（力敏感 sim2real / Force-aware hybrid），目标 ICRA/IROS。
**Year 3–4**：扩展到 5.1 或 5.7，目标 RSS/Science Robotics。
**Year 5**：投产业（ANYbotics/Unitree/Boston Dynamics/DeepRobotics）。

---

## 第六部分 复合/160_四足臂动力学概览–复合/210_RAMBO混合MPC_RL 六章详细大纲

### 复合/160_四足臂动力学概览 四足+臂动力学与控制概览（ALMA → Sleiman → qm_control）

**教学目标**：建立 loco-manipulation 的问题表述，理解从纯腿足到腿+臂的三维扩展；掌握 centroidal+EE wrench 统一动力学；比较分层 WBC 与单级 MPC 的范式差异；辨识 ALMA/Sleiman 2021/qm_control 三条工程实线的承接关系。

**子节**：
1. 从腿足到 loco-manipulation 的问题升格：状态/输入/约束的维度变化
2. 浮动基+腿+臂统一动力学推导（Featherstone 框架，Pinocchio 实现）
3. 末端 wrench 作为系统输入的两种模型（被动扰动 vs 主动接触）
4. 任务优先级冲突：locomotion vs manipulation 的数学形式化
5. ALMA（Bellicoso 2019）：ZMP 规划+分层 QP WBC 解剖
6. Sleiman 2021 Unified MPC：centroidal+物体增广
7. Sleiman 2023 Versatile Multi-Contact：informed graph search 发现接触模式
8. qm_control 工程实现定位：从 MIT WBIC 谱系到 ETH Sleiman 谱系的融合
9. 硬件现状与仿真平台选型（Go2/B1/B2/ANYmal/Spot 对比）
10. 评估指标：EE 跟踪精度、base 稳定性、支撑裕度、推恢复时间

**指定论文**：Khatib 1987 [1]、Sentis-Khatib 2005 [2]、Bellicoso 2019 ALMA [8]、Sleiman 2021 Unified MPC [9]、Sleiman 2023 Sci. Robot. [12]。

**指定项目精读**：概念层（无 deep dive，准备 复合/170_qm_control精读）——OCS2 `ocs2_robotic_examples/ocs2_legged_robot` 与 `ocs2_mobile_manipulator` 的**顶层目录**对比阅读。

**练习**：
- **A 编码**：Pinocchio 加载 Go2+Z1 URDF，计算 M(q)、J_foot、J_ee，验证 rank 与 condition number。
- **A 编码**：实现纯加权 WBC（单层 QP），在 MuJoCo 中让 Go2+Z1 站立跟踪一个 EE 正弦目标。
- **B 源读**：阅读 `ocs2_legged_robot/src/LeggedRobotInterface.cpp` 与 `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`，写 300 字对比报告。
- **思考 1**：为什么 Sleiman 2021 选 centroidal dynamics 而非 full dynamics 作 MPC 模型？
- **思考 2**：ALMA 的 ZMP-based planner 在动态步态（trot/bound）下为何不够？Sleiman 2021 如何解决？

**预计学习时间**：20–25h（含练习）。

---

### 复合/170_qm_control精读 qm_control 源码精读（OCS2 NMPC + 混合 WBC 四分支）

**教学目标**：逐文件解析 qm_control 项目；理解 OCS2 如何把 legged_robot 的 switched system 扩展为含臂；掌握分层 QP WBC 的 task 实现（floating base/friction cone/swing leg/body tracking/EE tracking/torque limits）；区分 main/feature-force/feature-compliance/feature-real 四分支的控制差异。

**子节**：
1. qm_control 架构总览：ros_control 插件+OCS2 interface+WBC 的三层调用链
2. `qm_description`：A1+6DoF arm URDF 构建（base link 合并、fixed joint 处理）
3. `qm_interface`：OCS2 OCP 构造（状态/输入/cost/constraint/reference manager）
4. Switched system + arm mode schedule 的配置文件解析（`.info`）
5. `qm_wbc` 分层 QP：任务顺序（dynamics→friction→swing→body→EE→torque limits）+ qpOASES 调用
6. `qm_estimation` 线性 Kalman filter 基座估计（接触相触发更新）
7. main 分支：whole-body motion 基线
8. feature-force 分支：末端受扰下的 disturbance rejection 实现
9. feature-compliance 分支：阻抗控制 + 关节扭矩/摩擦锥饱和并行 QP（IROS 2024 核心）
10. feature-real 分支：硬件接口、时序、安全保护

**指定论文**：Zhang 2024 IROS qm_control [15]、Sleiman 2021 [9]、Kim 2019 WBIC [7]、Righetti 2011 [4]。

**指定项目精读**（文件级）：
- `qm_interface/src/QMInterface.cpp`——OCP 构造入口
- `qm_interface/src/constraint/FrictionConeConstraint.cpp`
- `qm_interface/src/cost/EndEffectorTrackingCost.cpp`
- `qm_wbc/src/WbcBase.cpp`、`HoQp.cpp`、`HierarchicalWbc.cpp`
- `qm_controllers/src/QMController.cpp`——ros_control 插件入口与 500Hz 回调
- `qm_estimation/src/FromTopicEstimate.cpp`、`LinearKalmanFilter.cpp`

**练习**：
- **A 编码**：把 qm_control 的 A1 换成 Go2 URDF，修改 interface 与 description，验证仿真。
- **A 编码**：在 WBC 中新增一个"角动量 regularizer"任务（优先级 4），观察 push recovery 变化。
- **B 源读**：比较 `HierarchicalWbc.cpp` 与 `WeightedWbc.cpp`，在 Gazebo 跑相同任务，记录 QP 时间与跟踪误差。
- **B 源读**：阅读 feature-compliance 分支 WBC 差异，写 500 字总结扭矩饱和+摩擦锥并行的实现技巧。
- **思考 1**：qm_control 的 mode schedule 如何编码 EE 接触？如果要做"腿+臂双推门"需改哪些文件？
- **思考 2**：OCS2 MPC 50Hz 输出 `λ,q̇,q` ref，WBC 500Hz 跟踪——中间插值策略对瞬时扭矩的影响？

**预计学习时间**：35–45h（OCS2 编译 10h+阅读 15h+练习 15h）。

---

### 复合/180_Deep_WBC精读 Deep Whole-Body Control 精读（端到端 RL 统一 18-DOF 策略）

**教学目标**：复刻 Fu 2022 CoRL 端到端统一策略；理解 RMA/ROA 教师-学生范式；掌握 Advantage Mixing 的数学直觉（arm/leg 因果解耦估计）；在 IsaacGym→IsaacLab 迁移。

**子节**：
1. Fu 2022 问题设定：A1+WidowX 250 的 19 维统一动作空间
2. PPO 基础回顾（用户已有）+ 四足+臂 MDP 特殊性（多时间尺度）
3. 观测设计：proprioception+base 目标+EE 目标+历史堆叠
4. 奖励设计：EE 位置跟踪+能耗+存活+关节正则
5. **Regularized Online Adaptation (ROA)**：RMA 基础+正则项数学
6. **Advantage Mixing**：arm/leg 因果解耦的 advantage 估计
7. Domain Randomization 设计（质量/摩擦/外扰/电机模型）
8. 从 IsaacGym Preview 4 迁移到 IsaacLab 的工程细节
9. 评估：EE 跟踪精度 vs 基线（DreamWaQ/RMA/Blind）
10. 局限：无视觉、无真机部署代码、IsaacGym deprecate

**指定论文**：Fu 2022 Deep-WBC [17]、Kumar 2021 RMA、Margolis 2022 Walk These Ways、Rudin 2022 Learning to Walk in Minutes。

**指定项目精读**（文件级）：
- `legged_gym/envs/widowGo1/widowGo1_config.py`——观测/动作/奖励/randomization 完整配置
- `legged_gym/envs/widowGo1/widowGo1.py`——step/reset/reward 实现
- `rsl_rl/algorithms/ppo.py`——ROA/Advantage Mixing 插入点
- `rsl_rl/modules/actor_critic.py`——双头网络
- `widowGo1/urdf/widowGo1.urdf`——关节连接与惯量标定

**练习**：
- **A 编码**：用 IsaacLab 重实现 Deep-WBC 最小版，对比训练曲线。
- **A 编码**：关闭 Advantage Mixing，观察 arm/leg 各自 reward 收敛速度。
- **B 源读**：阅读 `ppo.py` 中 ROA 正则项，写出数学形式并验证梯度。
- **B 源读**：对比 Deep-WBC 的 `widowGo1.py` 与 RoboDuet（`https://github.com/locomanip-duet/RoboDuet`）的 env 结构，写 500 字架构对比。
- **思考 1**：统一单策略 vs RoboDuet 双策略——样本效率 vs 跨本体迁移哪个更重要？
- **思考 2**：Deep-WBC 无视觉意味着"目标已知"——如何接入 Chapter 89 视觉层而不破坏原策略？

**预计学习时间**：25–35h。

---

### 复合/190_Visual_WBC精读 Visual Whole-Body Control 精读（两级 RL 架构）

**教学目标**：复刻 Liu 2024 CoRL VBC 三阶段分层；理解低层通用 goal-reaching 策略如何与高层视觉规划解耦；掌握特权教师-深度图学生的在线模仿蒸馏；分析 IK 失败与 depth 照明敏感的 sim2real 挑战。

**子节**：
1. 视觉 loco-manipulation 的三个层次：感知/规划/控制
2. VBC 阶段 1：低层通用 goal-reaching 策略（body 5D+arm 12D+leg 28D+last action 12D 观测）
3. 奖励工程：tracking+regularization+survival 三类
4. VBC 阶段 2：特权教师规划器（用 ground-truth 物体位姿）
5. VBC 阶段 3：深度图学生在线模仿蒸馏
6. Depth 处理 pipeline：分割+前景掩码+下采样
7. 14 物体×3 高度的泛化评估与 retry 行为分析
8. IK 不可达失败模式与缓解（课程采样、物体高度限制）
9. 与 Deep-WBC 统一策略对比：样本效率/性能/可扩展性
10. 与 RoboDuet 双策略对比：cooperative signal vs 特权蒸馏

**指定论文**：Liu 2024 VBC [18]、Pan 2024 RoboDuet [23]、Miki 2022 *Science Robotics* 鲁棒感知行走、Fu 2022 DeepWBC [17]。

**指定项目精读**（文件级）：
- `low-level/legged_gym/envs/b1z1/b1z1_config.py`——低层 env
- `low-level/rsl_rl/algorithms/ppo.py`
- `high-level/scripts/train.py`——教师训练入口
- `high-level/models/student.py`——深度图学生网络
- `high-level/utils/distillation.py`——在线模仿蒸馏
- `third_party/` 的 isaacgym/rsl_rl/skrl 三者协作

**练习**：
- **A 编码**：把 VBC 低层换到 Go2+ARX5 配置（需改 URDF+观测维度）。
- **A 编码**：实现一个 LiDAR→占据栅格→目标位姿的替代高层，对比 depth 版。
- **B 源读**：阅读 `distillation.py`，写出在线蒸馏 loss 的 PyTorch 实现并对比 DAgger。
- **B 源读**：追踪 `high-level` 与 `low-level` 之间的接口数据格式（单位、坐标系）。
- **思考 1**：为什么 VBC 要三阶段而非端到端？端到端需克服什么瓶颈？
- **思考 2**：VBC 在 ANYmal+DynaArm 上复现的主要工程障碍是什么（URDF/SDK/sensor calib）？

**预计学习时间**：30–40h。

---

### 复合/200_UMI_on_Legs精读 UMI on Legs 精读（任务帧 EE 接口+操作策略跨平台复用）

**教学目标**：理解 UMI 数据采集范式；掌握 task-frame（非 body-frame）EE 跟踪的关键动机；复刻 Ha 2024 CoRL 的 diffusion policy+RL WBC 解耦；分析 iPhone ARKit 延迟建模与 Go2 电机热保护等真机工程细节。

**子节**：
1. UMI（Universal Manipulation Interface）手持夹爪+GoPro 数据采集范式回顾
2. Diffusion Policy 基础：visuomotor 动作序列去噪
3. 从固定臂到移动基座的问题：base 抖动对高层策略的扰动
4. **Task-frame EE tracking** 的数学：把 base 抖动吸收到 WBC 层
5. RL WBC 训练：观测 proprioception+未来 EE 轨迹窗口；动作 17 维关节位置
6. 奖励设计：EE 位置+姿态 MSE+正则；horizon 窗长度权衡
7. iPhone ARKit 60Hz ~100ms 延迟的训练时建模
8. sim2real 管线：Isaac Gym 训练→Docker devcontainer→Unitree Jetson 部署
9. prehensile/non-prehensile/dynamic 三任务的成功率分析
10. 跨具身零样本：固定臂→Go2+ARX5 的策略复用原理

**指定论文**：Ha 2024 UMI on Legs [19]、Chi 2024 UMI (RSS)、Chi 2023 Diffusion Policy、Fu 2022 DeepWBC [17]。

**指定项目精读**（文件级）：
- `mani-centric-wbc/configs/*.yaml`——WBC 训练配置
- `mani-centric-wbc/envs/go2_arx5.py`——Go2+ARX5 env
- `mani-centric-wbc/rewards/ee_tracking.py`——task-frame reward
- `real-wbc/scripts/deploy.py`——真机部署入口
- `real-wbc/devcontainer/Dockerfile`——环境复现
- `arx5-sdk/` submodule 的 ARX5 Python SDK
- `iPhoneVIO/` submodule 的 iOS ARKit app（Swift）

**练习**：
- **A 编码**：在 Isaac Lab 中实现 task-frame vs body-frame reward，对比 EE 跟踪误差。
- **A 编码**：为 ARKit 延迟建模（~100ms 随机延迟 + 偶发丢包），训练延迟鲁棒的 WBC。
- **B 源读**：阅读 `deploy.py`，画出从 Diffusion Policy→task-frame EE→WBC→关节扭矩的完整数据流。
- **B 源读**：对比 UMI-on-Legs 与 Deep-WBC 的动作空间与观测设计，写 500 字说明 manipulation-centric 的意义。
- **思考 1**：为什么固定臂 diffusion policy 可以零样本迁移到 Go2？隐含哪些假设？
- **思考 2**：如果用 Livox MID-360 LiDAR-Inertial SLAM 替代 iPhone ARKit（用户的 SLAM 背景），哪些 reward/observation 需改？

**预计学习时间**：35–45h（UMI 数据管线部分耗时）。

---

### 复合/210_RAMBO混合MPC_RL RAMBO 与混合 MPC+RL（前馈 QP 力矩+残差 RL 反馈）

**教学目标**：掌握 RL-Augmented Model-Based Control 的架构动机；理解可微分 QP（qpth）作为前馈层的实现；复刻 Cheng 2025 RA-L RAMBO 在 Go2 双足/四足模式的切换；分析混合架构相比纯 RL 或纯 MPC 的样本效率与鲁棒性优势。

**子节**：
1. 混合 MB+RL 的动机：纯 RL 力控差、纯 MPC 模型误差；为什么"前馈 MB+反馈 RL"是 Pareto 最优区
2. QP-based 反作用力优化作为前馈（简化刚体模型+摩擦锥+力平衡）
3. 可微分 QP（qpth 库）：KKT 条件隐函数求导
4. RL 策略作为反馈残差：观测+MB 解→δτ 输出
5. 奖励工程：EE 跟踪+base 稳定+反馈扭矩正则（鼓励 MB 主导）
6. Isaac Lab 训练配置：`Isaac-RAMBO-Quadruped-Go2-v0` vs `Isaac-RAMBO-Biped-Go2-v0`
7. Go2 双足模式的特殊约束（两腿支撑→ZMP 约束更严）
8. 真机验证：推购物车、平衡盘子、持软物的接触模型分析
9. 与 Portela 2024 Force Control 的对比：显式 FF/FB 分离 vs 纯 RL 力控
10. 扩展方向：换 Pinocchio rnea 作 MB（精确 ID MPC）；换 MPC 替代单步 QP（走向 Molnar 2025）

**指定论文**：Cheng 2025 RAMBO [29]、Portela 2024 Force Control [22]、Molnar 2025 Whole-Body ID MPC [16]、Amos 2017 OptNet（可微分 QP 理论）。

**指定项目精读**（文件级）：
- `source/crl2/algorithms/ppo.py`——PPO+MB 残差
- `source/isaaclab_tasks/rambo/quadruped_env.py`
- `source/isaaclab_tasks/rambo/biped_env.py`
- `source/isaaclab_tasks/rambo/mb_solver.py`——qpth 可微分 QP 调用
- `source/isaaclab_tasks/rambo/rewards.py`——FF/FB 分离 reward
- `scripts/reinforcement_learning/crl2/train.py` 与 `play.py`

**练习**：
- **A 编码**：把 RAMBO 的 QP 反作用力优化换为 Pinocchio centroidal inverse dynamics，验证效果。
- **A 编码**：在 reward 中移除"FB 扭矩正则"项，观察 RL 是否会"接管"完全控制，失去 MB 好处。
- **B 源读**：阅读 `mb_solver.py`，画出 qpth 前向/反向传播计算图。
- **B 源读**：对比 RAMBO 与 qm_control（OCS2+WBC）——把同一任务（plate balancing）在两套架构上运行，记录扭矩、跟踪误差、CPU 时间。
- **思考 1**：为什么 RAMBO 用单步 QP 而非 horizon MPC？如果换成 MPC 需解决哪些可微分性问题？
- **思考 2**：利用用户的 RL+SLAM+腿足三重背景，如何把 Perceptive MPC（Grandia 2023）接入 RAMBO 作 MB 层，得到 Perceptive RAMBO？

**预计学习时间**：30–40h。

---

### 六章总览与学习序

| 章 | 主题 | 论文数 | 项目 | 学时 | 前置 |
|---|---|---|---|---|---|
| 86 | 概览 | 5 | OCS2 顶层 | 20–25h | 腿足 24 章 |
| 87 | qm_control 源读 | 4 | qm_control | 35–45h | 86 |
| 88 | Deep-WBC 精读 | 4 | Deep-WBC | 25–35h | 86+RL 基础 |
| 89 | VBC 精读 | 4 | visual_wholebody | 30–40h | 88 |
| 90 | UMI on Legs | 4 | umi-on-legs | 35–45h | 88 |
| 91 | RAMBO 混合 | 4 | rambo | 30–40h | 87+88 |

**总学时**：175–230h。建议配置 2 个学期（每学期 1.0–1.5 个学分），或集中暑期 12 周密集课程。

---

## 结论

"四足+机械臂"方向的 2022–2026 文献呈现三条清晰主线：**ETH Sleiman 谱系**（Unified MPC→Versatile Multi-Contact→Perceptive NMPC）代表 model-based 的深度，**MIT/UCSD/Stanford 谱系**（Deep-WBC→VBC→UMI on Legs）代表学习驱动的广度，**ETH/CMU RAMBO 谱系**代表两者的收敛。对用户而言，最具价值的战略定位是：**利用 SLAM 背景打通感知-操作-运动闭环**（方向 5.1），**利用 RL 背景攻克 force-aware 混合控制**（方向 5.2/5.6），**利用腿足规控背景深化 momentum-centric WBC**（方向 5.7）。六章大纲以 qm_control 为工程主轴、以 Deep-WBC/VBC/UMI/RAMBO 为范式四极，恰好覆盖 MPC+WBC+RL 的完整光谱，与用户前置知识（Pinocchio/OCS2/Crocoddyl/TSID/WBC/IsaacLab/rsl_rl/grid_map/Perceptive MPC）无缝衔接。建议实际教学时以 **复合/170_qm_control精读 qm_control 源读**（35–45h）作为主骨架练习，并把 **复合/210_RAMBO混合MPC_RL RAMBO** 作为博士入门课题的起点——因为它在一张图上同时要求 MPC、RL、全身动力学三种能力，正是用户三重背景的最佳检验场。

**数据可靠性提示**：论文被引数为 2026 年 4 月基于 Semantic Scholar/Google Scholar 的外推估计（通常 ±30%）。硬件价格随批次与地区波动 ±20–40%，采购应以厂商官方报价为准。部分项目（Pedipulate、HYPERmotion、HiLMa-Res、Whole-Body EE Pose Tracking）无官方开源代码——建议优先深读有代码的 qm_control/Deep-WBC/VBC/UMI-on-Legs/RAMBO 五个主轴。

---

> **D3a→D3b 过渡提示**：D3a 四足+臂系统（12 腿部 DoF + 6 臂部 DoF = 18 DoF）与 D3b 人形系统（23-43 DoF）的核心差异在于**双足平衡的动态复杂度**和**自由度规模的数量级跃变**。从技术架构看，D3a 的 qm_control NMPC+WBC 在 18 DoF 下仍可实时运行（复合/170_qm_control精读），但人形的 30+ DoF 使纯 MPC 的实时性触及极限——这正是 D3b 中 RL 全面接管控制的物理原因。D3a 中 RAMBO 的"前馈 QP + 残差 RL"混合思想（复合/210_RAMBO混合MPC_RL）在 D3b 的 FALCON 中被扩展为"双智能体 RL（下肢稳定 + 上肢跟踪，含隐式力补偿）"（复合/250_力敏感人形LocoMani），两者代表了混合架构从四足到人形的演进。D3a 中 UMI-on-Legs 的任务帧 EE 接口（复合/200_UMI_on_Legs精读）在 D3b 中同样适用——人形机器人的操作同样可以通过"VLA 生成 EE 轨迹 + WBC 追踪"的分层接口实现（参见全景综述中的"VLA→任务帧 EE→WBC"模板）。此外，D3a 中四足机器人的状态估计（浮基 EKF）经验可直接迁移到 D3b 的人形状态估计中，但需额外处理双足支撑相中的离散切换和手臂运动对质心位置的扰动。

