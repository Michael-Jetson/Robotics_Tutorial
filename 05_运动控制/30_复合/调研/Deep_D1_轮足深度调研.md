## 轮足机器人（D1）：研究生课程研究资料汇编

*范围：轮足混合型地面机器人。不包括汽车和无人机。目标受众：博士课程，第76–81章。2026年4月编纂。*

轮腿机器人（WL）是地面移动机器人领域发展最迅速的前沿方向。**它们兼具车轮的能效优势（与小跑相比，COT降低约80%）**与腿部的越障能力，并于2024–2026年间从苏黎世联邦理工学院（ETH）的实验室演示（ANYmal-on-Wheels、 Ascento、CENTAURO）的阶段，迈向了商业部署（Swiss-Mile/RIVR，于2026年初被亚马逊收购；Unitree Go2-W/B2-W已开始发货；LimX W1即将发布）。本资料汇编了研究级论文清单、开源技术栈、数学基础、硬件现状、博士级开放性问题，以及按章节划分的课程大纲。

---

## 1. 论文列表（30篇）

引用次数为近似值（Google Scholar/Semantic Scholar，2026年初）。谱系箭头显示了轮腿机器人（WL）文献中的影响路径。

### 1.1 经典基础（2022年前）

**C1. Grand, Ben Amar, Plumet, Bidaud (2004), &quot;混合轮足机器人Hylos的姿态与轨迹解耦控制。&quot;** *ICRA 2004, 第5卷, 第5111–5116页.* DOI 10.1109/ROBOT.2004.1302528。约170次引用。**数学模型：**利用主动冗余的速率级动力静态模型；解耦反向运动学将姿态（腿部）与轨迹（车轮）分离；纳入了力-角倾覆裕度。 **学术渊源：**混合式双轮控制的奠基性论文；Kashiri 2019 和 Medeiros 2020 的先决条件。

**C2. Klemm 等（2019），《Ascento：一款双轮跳跃机器人》。** *ICRA 2019，第 7515–7521 页。* arXiv:2005.11435。约250次引用。**数学模型：**线性化带腿WIP模型 + LQR平衡器 + 通过并联弹簧辅助腿部伸展实现的倾覆前馈跳跃轨迹；用于平衡/驱动/蹲姿/跳跃/恢复的状态机。 **代码：** https://github.com/ascento. **发展脉络：** 经典轮式双足基线模型；衍生出 Klemm T-RO 2024 非平滑 TO 模型。

**C3. Kashiri 等 (2019)，&quot;CENTAURO：一种混合式行走与高功率弹性操作平台。&quot;** *RA-L 4(2):1595–1602.* DOI 10.1109/LRA.2019.2896758. ~220 次引用。 **数学模型：**基于有效载荷/质量比&gt;1.5的执行器选型准则；采用串联弹性驱动模型实现抗冲击能力。**代码：**XBotCore中间件，详见 github.com/ADVRHumanoids。**发展脉络：**奠定了IIT CENTAURO系列（Laurenzi, Dadiotis）的基础。

**C4. Bjelonic, Bellicoso, de Viragh, Sako, Tresoldi, Jenelten, Hutter (2019)，&quot;Keep Rollin&#x27; — 轮式四足机器人的全身运动控制与规划。&quot;** *RA-L 4(2):2116–2123.* arXiv:1809.03557. ~300 次引用。**数学：**基于 ZMP 的运动优化，采用感知滚动方向的运动学轮模型；分层 WBC 机制在广义加速度下强制执行非全向滚动。 **首个实现4 m/s混合式行走的扭矩控制轮足机器人，COT减少83%。** **谱系：**首篇ANYmal-on-wheels论文；《Rolling-in-the-Deep》的直接前身。

**C5. Bjelonic, Sankar, Bellicoso, Vallery, Hutter (2020), &quot;Rolling in the Deep — 基于在线轨迹优化的轮足机器人混合运动.&quot;** *RA-L 5(2):3626–3633.* arXiv:1909.07193. ~250 次引用。**数学：**级联在线轨迹优化——车轮轨迹优化 → ZMP/COM 轨迹优化——均基于具有非全控滚动的 SRBD；分层 QP 基 WBC 跟踪参考文献。**谱系：**ETH 轮式 MPC 系列的基线；使用 OCS2。

**C6. de Viragh、Bjelonic 等（2019），《基于线性化ZMP的轮式四足机器人轨迹优化》** *RA-L.* 补充了《Keep Rollin&#x27;》的研究；引入了针对轮足接触的线性化ZMP支持区域——为Medeiros 2020奠定了基础。

**C7. Medeiros、Jelavic、Bjelonic、Siegwart、Meggiolaro、Hutter（2020），《在复杂地形中行驶的轮足四足机器人轨迹优化》。** *RA-L 5(3):4172–4179.* DOI 10.1109/LRA.2020.2990720。约120次引用。**数学模型：**基于基姿态的自然语言处理（NLP）+ 采用赫尔姆霍兹样条的轮位/力轨迹，2.5维地形图，力-角倾覆约束（SRBD）；步高可达腿长的40%，坡度65°。 **谱系：**Rolling-in-the-Deep在崎岖地形上的扩展。

**C8. Jelavic, Farshidian, Hutter (2021), &quot;基于采样与优化相结合的腿轮机器人规划.&quot;** *arXiv:2104.04247.* **数学：**一种RRT*-类接触模式采样器，用于为强制滚动约束的OCS2 SLQ-MPC提供热启动；演示了在纯TO算法会陷入局部极小值的崎岖地形中，该方法仍能成功穿越。**谱系：**为轮足机器人（WL）架起了采样与TO之间的桥梁。

**C9. Bjelonic, Grandia, Harley, Galliard, Zimmermann, Hutter (2021)，《轮足机器人的全身MPC与在线步态序列生成》** *IROS 2021，第8388–8395页。* arXiv:2010.06322. ~200 次引用。**数学：**基于运动动力学模型（SRBD + 完整运动学，车轮视为移动接触点）的单任务SLQ-MPC；步态序列由实时“腿部效用”启发式算法推导，而非预定义计划。 **代码：**通过 OCS2 发布。**COT 减少 85%，预测误差减少 71%。****谱系：**典型的现代 WL-MPC 公式化。

**C10. Sleiman, Farshidian, Minniti, Hutter (2021), &quot;A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation.&quot;** *RA-L 6(3):4688–4695.* arXiv:2103.00946. ~400 次引用。 **数学：**带质心+物体刚体动力学的开关系统最优控制问题（OCP）；接触调度作为模式条件下的等式/不等式约束；基于OCS2的SLQ-MPC；ANYmal-C + DynaArm。**发展脉络：**所有行走-操作MPC（包括CENTAURO后续研究）的基础。

### 1.2 现代MPC时代（2022–2023）

**M1. Bjelonic, Grandia, Geilinger, Harley, Medeiros, Pajovic, Jelavic, Coros, Hutter (2022), &quot;用于高级移动技能的离线运动库与在线MPC。&quot;** *IJRR 41(9-10):903–924.* DOI 10.1177/02783649221102473. 约60次引用。**数学：**离线地形感知TO通过Skaterbots接触方向模型构建运动库；在线运动动力学MPC将该库用作成本参考。 **在ANYmal-on-wheels平台上实现了1.5 m/s速度下0.20 m的感知步长。**

**M2. Dadiotis, Laurenzi, Tsagarakis (2022), &quot;重载四足移动机械手的轨迹优化。&quot;** *Humanoids 2022, 第291–298页。* arXiv:2210.06803。**数学模型：** SRBD + 载荷-机身轨迹优化，双臂抓取力矩，在CENTAURO上载荷质量超过15%时提升操纵能力。**代码：** https://github.com/ADVRHumanoids/casannis_walking (CasADi+IPOPT)。

**M3. Dadiotis, Laurenzi, Tsagarakis (2023), &quot;高冗余腿式机械人的全身MPC：37自由度双臂四足机器人。&quot;** *arXiv:2310.02907, RA-L/ICRA 2024.* **数学：**基于49个状态/55个输入的CENTAURO模型的OCS2 SLQ-MPC，包含自碰撞和摩擦锥；以数十赫兹的频率进行实时重规划。**代码：** https://github.com/ADVRHumanoids/wb_mpc_centauro.

**M4. Grandia, Jenelten, Yang, Farshidian, Hutter (2023), &quot;基于非线性模型预测控制的感知式行走.&quot;** *T-RO.* arXiv:2208.08373. **数学模型：**基于SRBD且采用带符号距离惩罚项的地形感知NMPC；通过Bjelonic 2021的扩展直接移植到轮式ANYmal上。**代码：**已集成到OCS2中。

**M5. Takasugi等人（2024），《基于全质心NMPC的Tachyon 3多功能伸缩式轮腿行走》。** *IFAC NMPC 2024.* **数学模型：** 基于CBF的解析平滑全质心NMPC；实时感知式6轮伸缩腿行走。

### 1.3 学习时代 (2023–2026)

**L1. Vollenweider, Bjelonic, Klemm, Rudin, Lee, Hutter (2023)，&quot;基于多重对抗运动先验的强化学习高级技能&quot;。** *ICRA 2023，第5120–5126页。* **数学模型：**针对ANYmal-on-wheels的多技能策略采用多AMP风格奖励；在WL-MPC演示数据集上进行预训练，并通过强化学习进行微调。

**L2. Chamorro, Klemm, de la Iglesia Valls, Pal, Siegwart (2024), &quot;基于腿式和轮腿式机器人的盲爬楼梯强化学习。&quot;** *ICRA 2024, 第8081–8087页.* arXiv:2402.06143. **数学部分：**基于位置（而非速度）的强化学习任务建模 + 仅在批评器中使用特权地形信息的非对称演员-批评器架构；布尔型楼梯模式观测；Ascento 机器人无需外感受即可攀爬 15 厘米高的台阶。**视频：** https://youtu.be/Ec6ar8BVJh4.

**L3. Wang, Zhang 等 (2024)，&quot;轮腿机器人行走-操作的臂约束课程学习&quot;。** *IROS 2024 口头报告。* arXiv:2403.16535。 **数学：**采用渐进式手臂-工作空间约束的课程，以防止基座-手臂梯度冲突；在B2W+Z1上进行PPO。**代码：** https://github.com/aCodeDog/legged-robots-manipulation. **项目：** wheel-legged-loco-manipulation.github.io。

**L4. Lee, Bjelonic, Reske, Wellhausen, Miki, Hutter (2024), &quot;学习轮足机器人的鲁棒自主导航与运动&quot;。** *《科学机器人》9(89)。* arXiv:2405.01792。 DOI 10.1126/scirobotics.adi9641. **数学模型：**分层强化学习（HRL）——高层次的HRL导航策略向无模型+特权强化学习（privileged RL）的运动策略发出速度指令，该运动策略会隐式选择步行或行驶模式。**已在苏黎世和塞维利亚的Swiss-Mile项目中展示了公里级别的城市配送能力。**

**L5. Klemm, de Viragh, Rohr, Siegwart, Tognon (2024)，&quot;带接触开关与碰撞的轮式平衡机器人非光滑轨迹优化&quot;。** *T-RO 2024（部分数据库将于2025年开放早期访问）。* DOI 10.1109/TRO.2024.... **数学背景：**平面非线性RBD非线性规划（NLP），通过弧长参数化将运动分解为接触阶段，使轨迹本质上具有接触一致性；处理碰撞、牵引力限制及驱动边界；针对Ascento提供包含运动学环路的闭式动力学方程。**渊源：**用户查询中的“T-RO 2025非光滑TO”参考文献。

**L6. Roth, Frey, Hutter 等 (2025)，&quot;用于安全且感知平台的机器人导航的自学习感知前向动力学模型&quot;。** *RSS 2025.* **数学：**基于深度与本体感觉的自学习前向模型 → 未来基座姿态；作为安全滤波器接入MPC。**代码：** https://github.com/leggedrobotics/fdm.

**L7. Dadiotis, Laurenzi, Patrizi, Tsagarakis (2024)，&quot;AugMPC：用于非步态式腿足及混合式运动的强化学习增强型MPC。&quot;** **数学模型：**残差强化学习策略修正标准OCS2 MPC；在CENTAURO上验证。

**L8. Zhang, Wang, Wang, Lai, Bing, Jiang, Zheng, Zhang (2022), &quot;一种针对轮式双足机器人Ollie的自适应全身平衡控制方法。&quot;** *IROS 2022.* 腾讯Robotics-X。**数学：**自适应动态规划（ADP）用于参数不确定性下的级联平衡控制。 随后由张等人于2023年在《Front. Neurorobotics》16:1102259发表的AOOR数据驱动扩展。

**L9. 王等人（2021），《一种新型轮足机器人（Ollie）的平衡控制》** *ICRA 2021.* 腾讯机器人-X。 Ollie系列的研究奠基论文（5连杆腿部、尾部、360°翻转演示）。

**L10. 崔、王、张、张、赖、郑、张、江（2021），《基于学习的轮足机器人平衡控制》。** *RA-L 6(4):7667–7674.* Ollie 上的早期强化学习平衡控制器。

**L11. Liu, Yang, Liao, Lyu (2024), &quot;DIABLO: 一个完全由直驱关节组成的 6-DoF 轮式双足机器人。&quot;** *arXiv:2407.21500.* 基于直驱平台的 2-D 简化动力学 + LQR。 广受欢迎的开源硬件参考。

**L12. CTBC 作者（LimX Dynamics + 学术合作伙伴，2025），《基于指令学习和强化学习的轮式双足机器人接触触发盲爬技术》。** *arXiv:2509.02986.* 仿生轮-障碍物接触触发抬腿反射；部署于 Tron1 机器人（轮半径 12.7 厘米，爬升高度远超轮半径）。

**L13. Chen, Wang, Hong, Shen, Wensing, Zhang (2023)，《轮式双足机器人的欠驱动跳跃运动规划与控制》** *RA-L.*

**L14. ATRos 作者 (2025)，&quot;ATRos：为轮腿机器人学习节能型敏捷行走&quot;。** *arXiv:2510.09980.* 考虑能耗的强化学习奖励；针对轮腿机器人（WL）的显式奖励中包含最优控制轨迹（COT）。

**L15. Adaptive MoE 作者 (2025)，&quot;基于稀疏专家混合深度强化学习的双足轮腿机器人自适应多模式行走&quot;。** *Frontiers/PMC 2025 (PMC12975443)。* Top-K 门控专家混合策略；滚动与抬腿专家；解耦冲突梯度。

**L16. Whleaper 作者 (2025)，&quot;Whleaper：一种 10 自由度柔性双足轮腿机器人。&quot;** *arXiv:2504.21767.* 采用机械结构+LQR+RL 设计的 10 自由度双足机器人，每条腿具有 3 个髋关节自由度。

**L17. Stand-Walk-Navigate 作者 (2025)，&quot;站立、行走、导航：基于低成本轮式四足机器人的恢复感知视觉导航&quot;。** *arXiv:2510.23902.* 在 Go2-W 类硬件上，将恢复感知策略与视觉导航集成。

**L18. Humphreys, Zhou (2025)，&quot;通过仿生步态策略学习适应以实现多功能四足行走&quot;。** *《自然·机器智能》*。具有步态记忆的步态转换强化学习——可直接迁移至轮式四足机器人模式选择。

**L19. Choi, Ji, Park, Kim, Mun, Lee, Hwangbo (2023), &quot;在可变形地形上学习四足行走.&quot;** *《科学机器人》8(74):eade2256.* 参数化颗粒介质模拟器 → Raibo在沙滩上以3.03 m/s的速度奔跑。 软质地形上轮式平台的交叉参考。

**L20. Hoeller, Rudin, Sako, Hutter (2024), &quot;ANYmal Parkour.&quot;** *《Science Robotics》9(88):eadi7566.* 无模型强化学习跑酷；轮式机器人社区力求达到的基准。

---

## 2. 开源项目（前10名）

| # | 项目 | 星标 (2026) | 许可证 | 语言 | 关键硬件 | 阅读时长 |
|---|---|---|---|---|---|---|
| 1 | leggedrobotics/ocs2 | ~1.3k | BSD-3 | C++85/Py10 | ANYmal, 通用 | 40–60 小时 |
| 2 | clearlab-sustech/Wheel-Legged-Gym | 566 | BSD-3 | Py100 | Diablo, 双轮双足 | 10–15 小时 |
| 3 | ADVRHumanoids/wb_mpc_centauro | ~30 | 研究 | Py60/C++30 | CENTAURO | 20–30 小时 |
| 4 | upkie/upkie | ~200 | Apache-2.0 | Py60/C++35 | Upkie 开源硬件 | 15–25 小时 |
| 5 | limxdynamics/pointfoot-legged-gym → tron1-rl-isaacgym | ~150 | BSD-3 | Py100 | Tron1 (PF/SF/WF), W1 | 10–15 小时 |
| 6 | fan-ziqi/robot_lab (+ rl_sar) | 1.5k / 1k | BSD-3 | Py95 / C++75 | Go2W, B2W, Tita, MagicDog-W | 20–30 小时 |
| 7 | unitreerobotics/unitree_sdk2 | ~805 | BSD-3 | C++80/Py15 | Go2/Go2-W, B2/B2-W | 15–20 小时 |
| 8 | isaac-sim/IsaacLab | 数千 | BSD-3 | Py92 | 生态系统平台 | 30–50 小时 |
| 9 | aCodeDog/legged-robots-manipulation | 中等 | BSD-3 | Py100 | B2W+Z1, Go2+ARX | 10–15 小时 |
| 10 | leggedrobotics/fdm | ~低百小时级 | BSD-3 | Py80/C++15 | ANYmal-D | 15–25 小时 |

**关键结构说明。**

**OCS2** 是基于参考模型的堆栈：包按求解器（`ocs2_ddp`、`ocs2_sqp`、`ocs2_slp`、`ocs2_ipm`）和示例（`ocs2_legged_robot`、`ocs2_mobile_manipulator`）划分。 入口点位于 `ocs2_legged_robot_ros/` 下的启动文件中。依赖项：Eigen、Pinocchio、hpp-fcl、CppAD/CppADCodeGen、HPIPM、raisim。

**Wheel-Legged-Gym (CLEAR Lab)** 是 legged_gym 的衍生版本，包含 `wheel_legged`、`wheel_legged_vmc`、`wheel_legged_vmc_flat` 环境以及标准的 `train.py`/`play.py`。依赖于 Isaac Gym Preview 4 + rsl_rl。 这是轮足机器人（WL）领域很好的入门强化学习参考，因为它展示了VMC闭环建模。

**wb_mpc_centauro**是典型的37自由度轮式机械手MPC。采用Horizon（IIT的CasADi+IPOPT TO库）+ Pinocchio + ROS1/xbot2编写。许可证未明确说明——讲师在重新分发前应予以核实。

**Upkie** 是最易于入手的研究级轮式机器人：物料清单成本约 3,000 美元，Apache-2.0 许可，以 Python 为核心，基于树莓派 4 搭载 mjbots qdd100/moteus 系统栈。 代理包括 `mpc_balancer` (ProxQP)、`pink_balancer`、`pid_balancer`、`ppo_balancer` (Stable-Baselines3)。支持 Bullet、Genesis、mock、pi3hat 的 Spines 框架。

**LimX 点足式健身器材**（该页面会重定向至 `tron1-rl-isaacgym` 以展示新 SKU）通过 `ROBOT_TYPE=WF_TRON1A` 支持 `WF_TRON1A`（轮足式）。 相关项目：`tron1-rl-deploy-ros`、`tron1-rl-deploy-python`、`tron1-mujoco-sim`、`limxsdk-lowlevel`。具备良好的仿真到实机的流程。

**robot_lab / rl_sar (Ziqi Fan)** —— IsaacLab 中最成熟的轮式机器人覆盖方案：`RobotLab-Isaac-Velocity-Rough-Unitree-Go2W-v0`、`...B2W-v0`、`...Tita-v0`、`...MagicDog-W-v0`、`...ZSL1W-v0`。采用 PPO/蒸馏/CusRL 并结合对称性增强。 `rl_sar` 增加了 Gazebo + ROS1/2 + libtorch/onnxruntime 的部署层。

**Unitree SDK2**命名空间 `b2`、`go2`、`g1`、`h1`：`ai-w`/`normal-w`运动切换器处理 Go2-W 和 B2-W。 `b2w`、`go2w` 的 MuJoCo 场景位于 `unitree_mujoco` 中。CycloneDDS 0.10.2 传输；扭矩模式仅在 EDU SKU 上提供。

**IsaacLab** 不提供原生的轮式任务——轮式功能通过扩展实现（robot_lab、`Isaac-RL-Two-wheel-Legged-Bot`、`LeggedLab`）。核心移动任务位于 `source/isaaclab_tasks/manager_based/locomotion/velocity/config/{anymal_c,anymal_d,spot,unitree_go2,...}/` 下。

**fdm（学习型感知前向动力学模型）**是与公开的“Swiss-Mile”代码库最接近的实现：基于 ROS1 + IsaacLab 扩展，采用 ANYmal-D 目标定位，并配合 RSS 2025 会议论文。Swiss-Mile/RIVR 本身没有公开的代码库。

**跨领域学习顺序建议：** Upkie → Wheel-Legged-Gym + robot_lab → Unitree SDK2 + rl_sar → OCS2 + wb_mpc_centauro → fdm + legged-robots-manipulation。

---

## 3. 数学深度解析

### 3.1 非全向约束 (帕夫式形式)

帕夫约束是一种速度线性等式 $A(q)\dot q=0$，它限制了允许的运动，但不限制配置流形。对于一个垂直滚动的圆盘，其 $q=[x,y,\varphi,\theta]$ 为：

$$A(q)=\begin{bmatrix}\cos\varphi & \sin\varphi & 0 & -r\\ \sin\varphi & -\cos\varphi & 0 & 0\end{bmatrix},\qquad A(q)\dot q=0.$$

弗罗贝尼乌斯定理证实了其不可积性（允许场之间的李括号会生成新的方向——即“停车”机动）。允许的速度位于 $\dot q=G(q)u$ 空间中，且满足 $G=\mathrm{null}(A)$。

对于具有雅可比矩阵 $J_c$ 的三维车轮接触点 $p_c(q)$，将其速度在与地形对齐的坐标系 $\{\hat e_\parallel,\hat e_\perp,\hat n\}$ 中分解，并施加
$$\hat e_\perp^\top J_c\dot q=0\ \text{(no-slip lateral)},\quad \hat e_\parallel^\top J_c\dot q-r\dot\theta=0\ \text{(pure rolling)},\quad \hat n^\top J_c\dot q=0\ \text{(stance)}.$$

**OCS2 编码。**车轮约束作为状态输入等式约束 $g_1(x,u,t)=0$ 进入开关系统 OCP。 在 `ocs2_legged_robot` 中，末端执行器类（例如 `EndEffectorLinearConstraint`）通过 Pinocchio 计算 $J_c\dot q$，并将其投影到滚动坐标系上。约束通过增广拉格朗日法或松弛对数障碍法处理，从而使 SLQ/SQP/IPM 求解器能够生成在每个插值点上满足滚动条件的状态-输入轨迹。 *参考文献：* Murray–Li–Sastry 第7章；Lynch–Park §13.3.1；Bloch (2003)；Giftthaler 等 ICRA 2017。

### 3.2 基于 Pinocchio 3.x 的闭链运动学 `ConstraintModel`

带Baumgarte稳定化环约束的受限拉格朗日动力学：
$$\begin{bmatrix}M & J_c^\top\\ J_c & 0\end{bmatrix}\begin{bmatrix}\ddot q\\ -\lambda\end{bmatrix}=\begin{bmatrix}\tau-h\\ -\dot J_c\dot q-2\alpha J_c\dot q-\alpha^2 c(q)\end{bmatrix}.$$

Pinocchio 3的`RigidConstraintModel`（类型`CONTACT_3D`、`CONTACT_6D`）声明了位于不同运动学分支上的两个坐标系之间的重合。 `pinocchio::initConstraintDynamics` + `constraintDynamics` 采用近点德拉苏斯求解器，通过 `computeConstraintDynamicsDerivatives` 利用解析导数以 O(n+m) 时间复杂度求解 KKT 方程组，从而实现可微分 MPC。

轮足机器人有两个应用场景：(i) Cassie/Digit/LimX Tron1/腾讯Ollie上的5连杆/平行腿传动机构，以及 (ii) 建模为铰接+滚动结构的驻车制动轮。 真正的5自由度回转环路可表示为：在单个分支上使用虚拟回转关节的CONTACT_6D，或由Baumgarte方法稳定化的CONTACT_3D（参见Pinocchio问题#2729）。*参考文献：* Carpentier等人 SII 2019；Sathya &amp; Carpentier T-RO 2025。

### 3.3 混合接触调度（车轮+足部模式）

一个切换动力学系统：$\dot x=f_{m(t)}(x,u)$，包含按模式划分的接触集 $\mathcal C_i$ 和碰撞跳转映射 $j_{m_i\to m_{i+1}}$。每次接触都会施加 $J_k\dot q=v_k^{\mathrm{ref}}$，其中 $v_k^{\mathrm{ref}}=0$ 用于足部站姿，$v_k^{\mathrm{ref}}=r\omega_k\hat e_\parallel$ 用于滚动车轮。

**扳手锥约束因接触类型而异。**
- 脚（点接触）：$\|f_t\|\le\mu f_n,\ f_n\ge 0$。
- 平足：加上鞋底的质心（CoP）。
- 滚动轮：$f_n\ge 0,\ |f_\parallel|\le\mu_{\mathrm{roll}} f_n,\ |f_\perp|\le\mu_s f_n,\ \tau_{\mathrm{spin}}\approx 0$（绕轮轴自由旋转）。

OCS2的切换质心（OCP）：
$$\min\sum_i\phi_i(x_{t_{i+1}})+\int_{t_i}^{t_{i+1}}\ell_i\,dt\ \text{s.t.}\ \dot x=f_i,\ g_{1,i}=0,\ h_i\ge 0.$$

Bjelonic 2021 的见解：将所有末端执行器统一视为移动地面接触点，仅在滚动约束的激活方面有所不同——这简化了数据管理。

### 3.4 车轮滑移模型（Pacejka 基础）

滑移比 $\kappa=(r\omega-v_x)/\max(|v_x|,v_{\mathrm{low}})$；滑移角 $\alpha=\arctan(v_y/|v_x|)$。

Pacejka &#x27;89 &quot;魔术公式&quot;：
$$F_y(\alpha)=D\sin\!\big(C\arctan(B\alpha-E(B\alpha-\arctan B\alpha))\big)+S_V.$$
$D\approx\mu F_z$ 为峰值，$BCD$ 为原点处的转弯刚度。组合滑移椭圆截断：$F_y^{\mathrm{comb}}=F_{y0}\sqrt{1-(F_x/F_{x,\max})^2}$。

实际应用中，轮腿式控制器采用线性小滑移近似 $F_y\approx -C_\alpha\alpha,\ F_x\approx C_\kappa\kappa$ 结合摩擦圆 $\sqrt{F_x^2+F_y^2}\le\mu F_z$ —— 完整的 Pacejka 模型仅在速度 &gt;2 m/s 或湿滑地形下才适用（这一失效模式在 *Science Robotics 2024* 的 Swiss-Mile 论文中已被指出）。 *参考文献：* Pacejka (2012)；Rajamani 第13章。

### 3.5 轮足双足机器人的ZMP

经典ZMP：$x_{\mathrm{ZMP}}=-M_y/F_z,\ y_{\mathrm{ZMP}}=M_x/F_z$。在LIP条件下且$z_c$恒定时：$x_{\mathrm{ZMP}}=x_c-(z_c/g)\ddot x_c$。可行性：ZMP位于支撑多边形内部。

对于轮式双足机器人（Upkie, Tron1-WF），支撑多边形退化为车轮接触点之间的线段——横向ZMP必须位于该线段上，但在滚动方向上，车轮扭矩作为平衡输入（捕获点/DCM控制取代了支撑多边形的包围条件）。 对于四轮ANYmal-on-wheels，该多边形是四个滚动接触点的凸包，但扭矩锥有所不同：滚动状态下禁止在不遵守$\mu$的情况下施加任意纵向力。Medeiros 2020在其NLP中实现了这种混合的ZMP+摩擦圆可行性检查；de Viragh 2019对其进行了线性化处理。 *参考文献：* Vukobratović–Borovac 2004；Sardain–Bessonnet 2004；Winkler RA-L 2018 顶点ZMP。

### 3.6 混合轮足接触下的全身逆动力学

浮动基座动力学：
$$M\ddot q+h=S^\top\tau+\sum_k J_k^\top\lambda_k.$$
堆叠足部与车轮雅可比矩阵 $J_c=[J^f_1;\dots;J^w_1;\dots]$。足部站立：$J^f_i\ddot q+\dot J^f_i\dot q=0$。滚动车轮：$J^w_j\ddot q+\dot J^w_j\dot q=\dot v_j^{\mathrm{ref}}$ 结合 $v_j^{\mathrm{ref}}=r_j\omega_j\hat e_\parallel^j$。

分层（或加权）QP：
$$\min_{\ddot q,\tau,\lambda}\sum_t w_t\|J_t\ddot q+\dot J_t\dot q-\ddot x_t^{\mathrm{des}}\|^2+\mathrm{reg}$$
s.t. $M\ddot q+h=S^\top\tau+J_c^\top\lambda,\ J_c\ddot q+\dot J_c\dot q=a_c^{\mathrm{ref}},\ \lambda\in\mathcal K_{\mathrm{fric}},\ \tau\in[\tau_{\min},\tau_{\max}]$。

Bjelonic 2020 将分层加权规划问题简化为单个加权规划问题（约30个变量，求解时间约0.5毫秒）。通过“移动接触”技巧，车轮与脚部共享同一代码路径。*参考文献：* Sentis–Khatib 2005；Herzog 等 2016；Del Prete 等 IJRR。

### 3.7 MPC中的模式切换建模

三种范式。

**(a) 混合整数MPC：** $x_{k+1}=\sum_i\sigma_k^i f_i(x_k,u_k),\ \mathbf{1}^\top\sigma_k=1$。精确但计算时间呈指数级增长；Aceituno-Cabezas/Mastalli RA-L 2018；Deits–Tedrake Humanoids 2014。

**(b) 隐式接触TO（Posa 2014, Manchester 2019）：** $0\le\phi(q_k)\perp\lambda_k^n\ge 0$ + 摩擦互补性，作为MPCC求解。Le Cleac&#x27;h在T-RO 2024上实现了实时求解。

**(c) 基于相位的参数化（Winkler RA-L 2018； Bjelonic IROS 2021)：**固定模式序列，将相位**持续时间** $\Delta T_i$ 设为连续变量，对末端执行器样条曲线进行参数化，从而自动满足接触约束。滚动 = 地面接触移动。纯平滑NLP，通过IPOPT/SQP在<50 ms. Mode sequence itself comes from the "leg-utility" heuristic.

For WL the phase-based approach dominates because the rolling constraint is state-input coupled (not complementarity); Klemm T-RO 2024 is the notable exception, using arc-length parameterization + nonsmooth NLP to *discover* contact switches (stairs, jumps) for a wheeled biped.

---

## 4. PhD-level open problems

### 4.1 Wheeled-legged + arm manipulation (D1 ∩ D3)

A WL base with an arm must simultaneously balance, drive/step, and execute contact-rich tasks. The non-holonomic rolling couples with the arm Jacobian inside the WBC QP; naive hierarchical separation yields infeasibilities. **SOTA 2024–2026:** ALMA follow-ups at ETH, CENTAURO/Dadiotis 37-DoF MPC, Swiss-Mile/RIVR commercial deployments, Wang IROS 2024 arm-curriculum RL, Du T-RO 2025 whole-body inverse dynamics MPC. **Challenges:** CoM shifts under heavy payload while rolling; real-time solve at ≥100 Hz for 30+ DoF; payload inertia ID for sim-to-real.

**PhD angles.** (i) Soft-task rolling constraint inside unified NMPC for dynamic pouring/pushing at >1 m/s下求解。 (ii) 在线贝叶斯有效载荷识别，将结果输入MPC终端成本以保持平衡裕度。

### 4.2 基于SLAM的感知驱动模式切换

何时滚动与何时步行的决策必须基于深度+高度+语义信息实时作出。**SOTA：**Lee *Sci. Robotics* 2024在HRL架构内隐式选择模式； Miki在IROS 2022上展示了GPU高程映射；Wild的视觉导航（基于RSL的自监督可通行性）；Humphreys–Zhou在《自然·机器智能》2025年发表了仿生步态转换。**挑战：**高速行驶时视野受限的感知；混合连续+离散动作空间在强化学习下不稳定；模式切换的滞后/抖动；语义地形类别的模糊性。

**博士研究方向。** (i) 基于流式多模态高度图的Transformer策略，用于**2–3秒的预判模式切换**。 (ii) 信息论模式选择准则（例如预期自由能），在感知噪声受限的情况下保证抗抖动。

### 4.3 基于InEKF的滑移感知控制

Hartley–Grizzle InEKF（IJRR 2020）假设足部接触是静态的。而车轮本质上违反了这一假设。 **SOTA：** Yoon 等 arXiv:2402.00366 NMN+InEKF；AttenNKF 2025；InEKFormer；Wang 2024 扰动观测器扩展。**挑战：**车轮接触点移动；偏航不可观测性；在学习校正下保持李群对数线性结构。

**博士研究方向。** (i) 通过保留对数线性性和全局收敛性的车轮级滑移-扭转李元，扩充状态空间。 (ii) 针对滑移的对称性保持等变神经观测器；在沙地/冰面/湿滑路面进行每公里漂移量的基准测试。

### 4.4 高速轮式跑酷

将5+ m/s的滚动速度与动态障碍物穿越相结合仍是未解课题。 **SOTA：**腾讯Ollie（ICRA 2021 + IROS 2022 ADP + 2023 AOOR）、Ascento/Klemm非光滑T-RO 2024、迪士尼BDX、Hoeller《Sci. Robotics》2024（腿式跑酷基准）、Cheng ICRA 2024 Extreme Parkour。 **挑战：**高速下摩擦锥缩小；<50 ms contact-mode transitions; perception latency; hardware durability.

**PhD angles.** (i) Depth-to-torque RL at >以4 m/s速度越过20 cm路缘石，达到Cheng的跑酷基准。（ii）5 m/s空中旋转恢复时尾部/反应轮惯性的理论下界。

### 4.5 软质地形（沙地、泥地、雪地）

车轮与双脚需要截然不同的策略。Bekker–Wong–Janosi压力-下沉模型 $p=(k_c/b+k_\varphi)z^n$ 虽是基础，但难以在线耦合。**SOTA：** Choi/Hwangbo *Sci. Robotics* 2023 参数化颗粒模型，KAIST Raibo 在沙地上达到 3.03 m/s；SUSTech 主动轮式贝瓦计； 代尔夫特理工大学带凸缘车轮沙地模拟（2025）；Ding RFT。**挑战：**MuJoCo/Isaac中缺乏支撑接触模型；空间变化的$c,\varphi,n$；打滑→100%失效模式；不可恢复的下陷。

**博士研究方向。** (i) 在线贝克尔参数识别 → 滚动与步进模式切换，在混合沙/泥/雪地形上实现&gt;50%的能耗降低。 (ii) 最小本体感觉信号集，足以在一轮旋转内识别 $(c,\varphi,n)$。

### 4.6 长距离配送自主性

Swiss-Mile → RIVR（亚马逊于2026年初收购）是典型的测试平台。**SOTA：**Lee *Sci. Robotics* 2024城市HRL、Deep Robotics Lynx M20 Pro、ANYbotics ANYmal X（巡检）。 **挑战：** 重载条件下2–4小时的能量预算；GNSS受限环境下的城市SLAM；法规合规；门前操作；车队级持续学习；电机故障时的平滑降级。

**博士研究方向。** (i) 在1000次送货的城市基准测试中，以步态切换激进度为自变量，研究成功率与能耗的帕累托前沿。 (ii) 基于遥控修正的持续强化学习，并在门前操作中具有可证明的无遗忘界限。

### 4.7 混合MPC+RL的正式稳定性

强劲的实证性能缺乏理论证明。**SOTA：** Grandia–Ames ICRA 2021 多层CBF+MPC；CBF-RL（arXiv:2510.14959）； 混合式腿足系统的HJ可达性（Choi等人，arXiv:2201.08538）；游戏玩法滤波器（Hsu–Fisac，arXiv:2405.00846）；一滤波器统领万象（arXiv:2412.09989）； Wu–Dai ICML 2024 李雅普诺夫稳定神经网络控制。**挑战：**离散接触重置破坏标准李雅普诺夫条件；30+自由度超出HJ求解器能力；神经策略难以通过SOS/MIP验证；滚动非霍诺莫尼约束下的CBF设计；仿真到实机的证书有效性。

**博士研究方向。** (i) 混合MPC+RL框架下全阶（30维）WL的深度HJ可达性。 (ii) 在滚动非全向约束下，确保不穿透地形边界的良好定义CBF类。

---

## 5. 硬件概况

### 5.1 Unitree Go2-W

16个电机（12个腿部电机 + 4个轮毂电机），7英寸充气轮胎，约15千克，最高滚动速度2.5米/秒，70厘米障碍物（混合模式），35°坡度，约8千克额定载荷，430瓦时电池（续航1.5–3小时）。 传感器：4D激光雷达 L1、Intel D435i、脚部力传感器、双关节编码器。计算单元：Jetson Orin NX 16 GB（EDU/Plus/Ultimate版本）。 **SDK：** `unitree_sdk2` DDS（约500 Hz），低级 `(q,dq,kp,kd,tau_ff)` 扭矩模式（**仅限EDU版本**）。 价格约 $14 k base, ~$22–33 千元（EDU版）。**已知问题：**脚部/车轮不可热插拔；不兼容 D1 臂；7 英寸车轮限制纯滚动爬楼梯能力；消费版 SKU 功能受限。

### 5.2 Unitree B2-W

16 个电机，12 英寸充气轮胎，75 公斤， 峰值腿部扭矩360 N·m，滚动速度5.56 m/s（20 km/h），40 cm台阶高度，&gt;45°坡度，最大静态载荷120 kg / 动态载荷≥40 kg，2,250 Wh可更换电池，续航4–5小时，IP67防护等级，工作温度−20至+55 °C。 传感器：RoboSense Helios 5515，2× D435i，双编码器。两台车载电脑（运动控制专用 + 用户可访问开发机）。通过DDS以~500 Hz实现**全扭矩控制**。价格约$89–100,000。 **已知问题：** 重量极大（需安全固定装置）；双PC架构的带宽限制了直接控制；车轮扭矩 < leg torque (slope climbing in rolling mode is weight-dependent); reports of gearbox wear under heavy sustained payload.

### 5.3 Upkie (Inria/Caron)

Open-hardware, Apache-2.0. 6 actuated joints — 2 legs × (hip pitch, knee, wheel). Actuators: mjbots qdd100 (hip+knee, 16 N·m peak, 6 N·m continuous) + moteus wheel drivers. ~5–6 kg, ~60 cm standing. Raspberry Pi 4 + pi3hat (5-CAN-FD). IMU ICM-20602-class. Magnetic 14-bit encoders. **Full torque control** 200–1000 Hz. Python-first API; Gymnasium envs; PPO/SB3 sim-to-real demonstrated; ProxQP MPC balancer. BoM ~$3 k + 60 h printing. **Known issues:** small payload, ~1.5 m/s top speed, 3-D-printed parts fragile (intentional), qdd100 beta 3 stock issues.

### 5.4 LimX Tron1

Multi-modal biped, 3 swappable feet: Point-Foot (PF), Sole (SF), Wheeled (WF = WF_TRON1A). 6 leg joints (3/leg) + 2 wheel motors on WF. Actuators: 48 V LimX harmonic-class, 30 N·m continuous / 80 N·m peak. ~20 kg. Battery 46.8 V / 4.5 Ah / 210.6 Wh, >2 h。标配RGB-D；可选配LiDAR + Jetson NX。Intel i3 16 GB / Ubuntu 20.04 + ROS Noetic。 **完整的位置/速度/扭矩 SDK**（Python+C++），支持 Isaac/MuJoCo/Gazebo 仿真到实机的管道。价格 1.34 万欧元 / 1.4–1.7 万美元（教育版）。**已知问题：**点状足部无踝关节扭矩（本质上不稳定）；更换足部需机械拆卸；高计算负载下电池续航有限；文档部分为中文。

### 5.5 LimX W1

带轮四足机器人，四条向后弯曲的腿部末端装有动力轮毂；可依靠后两腿站立并保持平衡（双足模式，152 厘米）。16 个自由度（12 个腿部 + 4 个轮毂）。自主研发的直驱轮毂执行器。 具备实时地形感知能力（LiDAR+深度传感器）、IMU及关节编码器。Motion-Intelligence技术栈整合了感知、强化学习、多体动力学及MPC。预计2024–2025年发布；价格需询价。**已知问题：**尚未与Tron1实现开放SDK功能对等；独立学术验证有限；轮腿组合结构的复杂性存在可靠性风险；完整数据手册尚未公开。

---

## 6. 章节大纲（第76–81章）

### 第76章 — 硬件格局与设计权衡

**教学目标。** 概述轮足混合设计空间；识别执行器、轮径及自由度（DoF）之间的权衡关系；分析成本、载荷、感知能力及SDK开放性。

**子章节。** (76.1) 分类：轮式双足机器人 vs. 轮式四足机器人 vs. 混合型机械手。 (76.2) 执行器拓扑结构（直驱 vs. 谐波 vs. QDD）。 (76.3) 车轮几何结构对爬楼梯和COT的影响。 (76.4) 感知套件（深度传感器、激光雷达、IMU、力/扭矩传感器）。 (76.5) SDK访问及扭矩模式可用性。(76.6) 平台深度解析：Go2-W、B2-W、Upkie、Tron1、W1、ANYmal-on-wheels、CENTAURO、Ascento、Ollie。

**论文。** Kashiri RA-L 2019 CENTAURO；Klemm ICRA 2019 Ascento；Wang ICRA 2021 Ollie；Liu 2024 DIABLO；Whleaper arXiv:2504.21767；Bjelonic–Klemm–Lee–Hutter《轮腿机器人综述》。

**代码库。** upkie/upkie; unitreerobotics/unitree_sdk2; limxdynamics/tron1-rl-isaacgym; leggedrobotics/anymal_c_simple_description（用于网格数据）。

**练习。** (i) 根据已发表的规格参数计算各平台的COT。 (ii) 构建URDF对比表（自由度、轮半径、质量）。 (iii) 在Bullet平衡演示中运行Upkie。 **时间：** 8小时讲座 + 10小时实验 = **18小时**。

### 第77章 — 轮足机器人（WL）的经典MPC（Bjelonic系列）

**教学目标。**掌握基于ZMP的运动优化→WBC管道（Bjelonic 2019）；理解Rolling-in-the-Deep的级联在线TO；构建IROS 2021的基于相位的运动动力学MPC；并扩展至Medeiros 2020所提出的崎岖地形场景。

**子章节。** (77.1) 单刚体 + 滚动约束。 (77.2) 基于分层QP的混合脚/轮接触WBC（§3.6）。 (77.3) 基于相位的末端执行器参数化（§3.7c）。 (77.4) 非平坦地形上的质心位置 (ZMP) (§3.5)。 (77.5) 采样辅助的轨迹优化 (TO) (Jelavic 2021)。 (77.6) 案例研究：37 自由度 CENTAURO 多相控制 (MPC) (Dadiotis 2023)。

**论文。** C4 Keep Rollin&#x27;; C5 Rolling in the Deep; C6 de Viragh 2019; C7 Medeiros 2020; C8 Jelavic 2021; C9 Bjelonic IROS 2021; M1 IJRR 2022; M3 Dadiotis 2023; M4 Grandia T-RO 2023.

**代码库。** leggedrobotics/ocs2 (SLQ-MPC); ADVRHumanoids/wb_mpc_centauro (Horizon+CasADi); ADVRHumanoids/casannis_walking。

**练习题。** (i) 在Pinocchio中推导4轮ANYmal的滚动约束帕夫菲安矩阵。 (ii) 在CasADi中为轮式双足机器人实现基于相位的双接触目标轨迹（TO）。(iii) 扩展OCS2中的`ocs2_legged_robot`，使其在单腿上暴露`wheel_rolling_constraint`。**时间：** 10小时讲座 + 20小时实验 = **30小时**。

### 第78章 — 轮式机器人感知

**教学目标。**理解轮式机器人的外感受管道（深度、激光雷达、RGB）；构建GPU高程图；学习感知MPC；评估可达性；通过SLAM闭环。

**子章节。** (78.1) 深度/激光雷达传感器及移动底盘上的标定。 (78.2) CuPY高程映射及其局限性。 (78.3) 感知型非线性多相位控制（Grandia T-RO 2023）。 (78.4) 野外视觉导航 + 自监督可通行性。 (78.5) 城市WL配送中的SLAM。 (78.6) 基于前向动力学模型的安全滤波器。

**参考文献。** Miki IROS 2022 elevation_mapping_cuPY；Grandia T-RO 2023；Lee *Sci. Robotics* 2024；Roth RSS 2025 fdm；Miki *Sci. Robotics* 2022 感知式行走；Stand-Walk-Navigate arXiv:2510.23902。

**代码库。** leggedrobotics/elevation_mapping_cuPY; leggedrobotics/fdm; leggedrobotics/wild_visual_navigation; isaac-sim/IsaacLab（深度传感器环境）。

**练习。** (i) 在 Upkie/Go2-W 包上运行 elevation_mapping_cuPY。 (ii) 在小型轮腿（WL）数据集上训练一个 fdm 风格的前向模型。(iii) 基于 ANYmal-on-wheels 日志构建一个 2D 可通行性分类器。**时间：** 8 小时讲座 + 16 小时实验 = **24 小时**。

### 第 79 章 — 轮腿机器人强化学习

**教学目标。**在IsaacLab中为轮腿机器人构建强化学习行走策略；理解领域随机化、非对称演员-批评家、特权学习；迁移至真实硬件。

**子章节。** (79.1) PPO回顾及Rudin“几分钟学会行走”管道。 (79.2) 带特权批评家的非对称演员-批评家 （Chamorro 2024）。(79.3) 基于位置的任务与基于速度的任务。(79.4) 轮腿机器人训练课程设计（手臂受限与基于速度）。(79.5) 针对轮子摩擦与打滑的领域随机化。(79.6) 模拟到真实：延迟、执行器模型、机载推理。

**论文。** L1 Vollenweider ICRA 2023 AMP；L2 Chamorro ICRA 2024；L3 Wang IROS 2024；L10 Cui RA-L 2021 基于学习的平衡；L14 ATRos arXiv:2510.09980；L15 MoE 2025。

**代码库。** clearlab-sustech/Wheel-Legged-Gym; fan-ziqi/robot_lab; fan-ziqi/rl_sar; limxdynamics/tron1-rl-isaacgym; aCodeDog/legged-robots-manipulation; isaac-sim/IsaacLab。

**习题。** (i) 在 robot_lab 中训练 Go2-W 策略以实现平地速度跟踪。 (ii) 添加车轮打滑域随机化；报告真实机器人迁移情况。 (iii) 在 Wheel-Legged-Gym 上复现 Chamorro 的布尔阶梯模式观察。 **时间：** 10 小时讲座 + 30 小时实验 = **40 小时**。

### 第 80 章 — 混合 MPC + RL 与模式切换

**教学目标。** 将基于模型的 MPC 与 RL 结合用于轮腿模式选择；理解 MPC-教师-RL-学生模型、残差 RL、学习型安全滤波器以及模式切换的 MoE。

**子章节。** (80.1) 模式切换建模 (§3.7)。 (80.2) MPC作为专家演示者（AugMPC）。 (80.3) 基于WBC的残差强化学习。 (80.4) 滚动/行走的专家混合模型（L15）。(80.5) 安全滤波器：CBFs、HJ可达性、游戏玩法滤波器。(80.6) 分层导航-运动（Lee 2024）。

**论文。** L4 Lee *《机器人科学》* 2024；L5 Klemm *《机器人技术》* 2024；L7 AugMPC；L12 CTBC 2025；L18 Humphreys–Zhou *《自然·机器智能》* 2025；Grandia ICRA 2021 多层安全； Hsu/Fisac 游戏玩法过滤器 2024；One-Filter-to-Deploy-Them-All 2024。

**代码库。** leggedrobotics/ocs2（MPC专家）；fan-ziqi/robot_lab（残差RL目标）；leggedrobotics/fdm（安全过滤器）；IsaacLab安全扩展。

**练习。** (i) 在模拟环境中，针对ANYmal-on-wheels模型，基于OCS2-MPC训练残差强化学习策略。 (ii) 在Wheel-Legged-Gym中，使用MoE实现布尔类型的“滚动 vs 步态”决策门。 (iii) 添加一个CBF安全过滤器，强制执行车轮不穿透约束，并测量干预频率。 **时间：** 12 小时讲座 + 24 小时实验 = **36 小时**。

### 第81章 — 轮腿底盘上的机械臂（D1 ∩ D3）

**教学目标。** 将机械臂集成到轮腿底盘上；设计统一的移动-操作MPC；处理底盘与机械臂的耦合；执行移动式拾取放置；理解Swiss-Mile/RIVR风格的物品递送。

**子章节。** (81.1) 轮腿+机械臂的运动学模型（浮动底盘 + 滚动 + 机械臂雅可比矩阵）。(81.2) 统一的移动-操纵多目标控制（Sleiman 2021, ALMA）。(81.3) 37自由度CENTAURO案例（Dadiotis）。 (81.4) 受手臂约束的课程强化学习（Wang 2024）。(81.5) 商用行走-操作系统：Swiss-Mile → RIVR。(81.6) 有效载荷识别与自适应控制。(81.7) 项目：门前配送模拟器。

**论文。** C10 Sleiman RA-L 2021；C3 Kashiri CENTAURO；M2 Dadiotis Humanoids 2022；M3 Dadiotis 2023；L3 Wang IROS 2024；Bellicoso ICRA 2019 ALMA；Du T-RO 2025 全身逆动力学行走-操作； Ma ICRA 2023 跌倒损伤减缓。

**代码库。** ADVRHumanoids/wb_mpc_centauro; aCodeDog/legged-robots-manipulation (B2W+Z1, Go2+ARX); leggedrobotics/ocs2 mobile_manipulator; wheel-legged-loco-manipulation.github.io。

**练习。** (i) 扩展 OCS2 mobile_manipulator 以在轮式底盘上添加滚动约束；解决 1 m/s 速度下的拾取放置任务。 (ii) 在 IsaacLab 上重现 B2W+Z1 上的臂约束课程强化学习。 (iii) 构建一个包含能量和成功率指标的模拟门前投递基准测试。 **时间：** 12 小时讲座 + 28 小时实验 = **40 小时**。

**课程总计：** ~188 小时（6 章，约一个 12 周的学期 × ~16 小时/周）。

---

## 7. 结论与收获

进入 2026 年，轮腿机器人技术将呈现三大转变。 **第一**，由 Bjelonic 和 Hutter 开创的基于模型的 MPC 流程（从 Keep Rollin&#x27; → Rolling-in-the-Deep → IROS 2021 全身 MPC）已发展为一套成熟的方案：在采用相位参数化且将车轮视为移动地面接触点的运动动力学模型上，实现单任务 SLQ-MPC，并基于 OCS2 进行实现。 **其次**，在鲁棒感知模式切换领域，学习方法已超越经典控制——Lee等人发表于《Science Robotics》2024年的论文，展示了在Swiss-Mile上实现公里级城市配送，这是最鲜明的例证；随后亚马逊收购RIVR更证实了其商业可行性。 **第三**，前沿已转向混合MPC+RL架构（AugMPC、MoE策略、残差RL）、可微分仿真以及形式化安全（Gameplay Filters、深度HJ可达性）——这些均尚未验证出全阶WL平台，因此提供了丰富的博士研究机会。

就课程设置而言，第77章和第79章构成技术核心； 第80章是杠杆作用最大的章节，因为它连接了MPC和RL两部分，并为第81章中探讨的D1∩D3交集奠定了基础。Upkie是理想的第一硬件平台（廉价、开源、可访问扭矩）；Tron1-WF或Go2-W EDU是理想的第二平台；B2-W或ANYmal-on-wheels则是理想的毕业设计项目。 **开源生态系统中最大的缺口在于IsaacLab核心中缺少一个参考级的轮腿环境**——目前大部分轮式机器人相关功能都存在于扩展模块中（如robot_lab、Wheel-Legged-Gym、tron1-rl-isaacgym），而一个BSD-3许可的“官方”轮式任务将大幅降低研究入门门槛。 在此之前，robot_lab（1.5k 星，支持 8 种以上轮式平台）是务实的默认选择。

七个未解问题——运动控制、感知模式切换、考虑滑移的InEKF、高速跑酷、软质地形、长时效配送以及形式化混合稳定性——每个问题都足以支撑一篇具体的博士论文。 其中研究人员最少但影响最大的是**基于全阶威廉斯算法的混合MPC+RL形式安全证明**：其他所有问题都将受益于此领域的进展，且目前尚无实验室针对30自由度轮式平台发表过相关安全证书。

---

> **D1→D2 过渡提示**：D1 轮足报告聚焦"轮+足"的混合运动控制。接下来的 D2 移动操作报告将视角从"底盘运动"转向"底盘+臂的协作"。两者的技术桥梁在于底盘运动学：D1 中 Pfaffian 约束下的 SE(2) 底盘规划（复合/60_轮式运动学与Pfaffian-77），直接为 D2 的底盘+臂联合运动学（复合/120_底盘臂联合规划）提供数学基础。此外，D1 中 Swiss-Mile 的导航感知栈（复合/90_Swiss_Mile商业化）与 D2 中 Nav2 的导航栈（复合/120_底盘臂联合规划）在地图表示和路径规划层面高度相通。对于有轮足背景的读者，D2 的增量主要在臂的运动规划（MoveIt2）和 VLA 端到端操作策略（openpi/LeRobot）两个维度。

