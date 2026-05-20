## 轮足机器人与足式机械臂开源项目全景调研

**轮足和loco-manipulation两大领域的开源生态已形成"RL训练+MPC规划+WBC执行"的三层技术栈，Pinocchio+OCS2+IsaacGym构成核心基础设施。** Swiss-Mile等标杆项目的核心代码并未开源，但以OCS2为代表的MPC框架和以Wheel-Legged-Gym为代表的RL训练平台提供了完整的算法研究入口。对于一名熟悉SLAM和C++规控的工程师，最高效的切入路径是：先通过OCS2的mobile_manipulator示例理解MPC架构，再用Wheel-Legged-Gym或robot_lab搭建RL训练环境，最后参考qm_control和legged_control实现完整的MPC+WBC分层控制。

---

## 第一部分：轮足式机器人开源项目

### 1.1 ETH RSL / Swiss-Mile生态——标杆级但开源有限

**OCS2（leggedrobotics/ocs2）** 是该领域最重要的MPC框架，**~1,300 stars**，BSD-3-Clause许可证，由ETH RSL的Farbod Farshidian和Marco Hutter团队开发。OCS2实现了SLQ（连续时间约束DDP）、iLQR、SQP（基于HPIPM/BLASFEO的多重射击）、SLP和IPM五类求解器，通过增广拉格朗日或松弛障碍法处理约束。其C++依赖包括**Pinocchio**（刚体动力学）、**HPIPM/BLASFEO**（结构化QP）、**CppAD/CppADCodeGen**（自动微分与代码生成）、Eigen、Boost、hpp-fcl（碰撞检测）。

OCS2的核心架构亮点在于其**模块化"收集器"设计**：`OptimalControlProblem`通过命名的cost/constraint term实现插件化组合，每个term可通过`isActive`在运行时启用/禁用。MPC-MRT（Model Reference Tracking）异步架构支持MPC在~50-100Hz运行、WBC在~400-1000Hz执行。OCS2包含六个机器人示例（double integrator、cartpole、ballbot、quadrotor、mobile manipulator、legged robot），其中**mobile_manipulator示例**支持Franka Panda、Kinova Jaco2、Ridgeback+UR5等平台，将移动底盘视为"带速度控制约束的虚拟轮基座"。

**关键发现：OCS2公开仓库中没有wheeled_biped或wheeled_quadruped示例。** Swiss-Mile/ETH RSL的轮足代码（滚动约束嵌入MPC）属于内部代码，未开源。但OCS2的约束框架天然支持轮足建模——**非完整滚动约束**可作为状态-输入等式约束（`g1`）编码，零滑移约束通过接触点速度约束实现。从论文"Rolling in the Deep"（RAL 2020）可知，ETH的轮足MPC方案为：基于ZMP的在线轨迹优化（级联QP，50Hz）+ 分层WBC（将滚动约束作为优先级任务），wheel-terrain接触点的x/z方向加速度约束为零。

**Swiss-Mile**（github.com/swiss-mile）仅有一个网站仓库（3 stars），所有控制代码均为商业机密。其Science Robotics 2024论文描述了分层RL框架（locomotion controller + navigation controller），实现行走/驾驶模式的平滑切换。**Ascento**（github.com/Ascento-AG）同样无公开控制代码，ascento-robotics组织于2024年10月存档。Ascento 2的核心算法为LQR平衡+分层WBC（RAL 2020），基于四连杆机构的闭链动力学解析解。

**IIT的wb_mpc_centauro**（ADVRHumanoids/wb_mpc_centauro）值得特别关注——这是目前唯一基于OCS2的公开轮足MPC实现，面向**CENTAURO**（37自由度双臂四足轮式机器人），使用OCS2 fork + Pinocchio + qpOASES，BSD-3-Clause许可证。

**配套论文**：Bjelonic et al. "Rolling in the Deep" (RAL 2020), "Keep Rollin'" (RAL 2019), "Whole-Body MPC and Online Gait Sequence Generation for Wheeled-Legged Robots" (IROS 2021), Sleiman et al. "A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation" (RAL 2021)。

**最小复现要求**：Ubuntu 20.04 + ROS Noetic，克隆OCS2 + Pinocchio + hpp-fcl，构建`ocs2_mobile_manipulator_ros`或`ocs2_legged_robot_ros`。

### 1.2 RL训练平台——轮足领域最活跃的开源赛道

**Wheel-Legged-Gym（clearlab-sustech/Wheel-Legged-Gym，566 stars）** 是轮足RL训练的标杆项目，由南方科技大学CLEAR Lab开发。其核心创新是**VMC（Virtual Model Control）**——将闭链机构（平行四连杆）的运动控制统一为开链虚拟力/力矩输出，使RL策略直接输出虚拟力和腿长目标（而非单独关节命令），极大简化了真实部署。基于IsaacGym Preview 3 + rsl_rl（PPO），提供`wheel_legged_vmc`（粗糙地形）和`wheel_legged_vmc_flat`（平坦地形，低VRAM需求~4GB+）两个任务环境。轮子在URDF中建模为连续旋转关节，**滚动约束通过RL奖励函数隐式学习**而非显式建模。BSD-3-Clause许可证。

**robot_lab（fan-ziqi/robot_lab）与rl_sar（fan-ziqi/rl_sar，1,200 stars）** 构成了目前最完整的"训练→仿真验证→真机部署"工具链。robot_lab是IsaacLab扩展，提供**Go2W、B2W、DDTRobot Tita、Deeprobotics M20、Zsibot ZSL1W**五款轮足机器人的RL训练环境，支持PPO+域随机化+Teacher-Student蒸馏+AMP。rl_sar提供**LibTorch C++部署框架**，支持ROS1/ROS2/纯CMake三种构建方式，通过JIT-traced `.pt`模型在真实机器人上推理，控制频率~50-200Hz。该组合覆盖了从训练到部署的全流程，是轮足RL研究的首选方案。

**Isaac-RL-Two-wheel-Legged-Bot（jaykorea，282 stars）** 是IsaacLab（而非IsaacGym）上的双轮足RL训练项目，使用PPO + 非对称Actor-Critic + observation stacking，适合希望基于更新平台（Isaac Lab/Isaac Sim）做研究的用户。

**LimX Dynamics的Tron1生态** 提供了从训练到部署的完整链路。**tron1-rl-isaacgym**（121 stars）支持三种足端类型：点足（PF）、球足（SF）、轮足（WF），轮足变体WF_TRON1A将轮子作为连续自由度关节。部署方面，**tron1-rl-deploy-ros/ros2**使用**ONNX Runtime C++** API进行策略推理（而非LibTorch），展示了另一种主流部署路径。注意：LimX的四足轮式产品W1并无开源训练代码，Tron1 WF是最接近的公开方案。

**Unitree生态**中，官方提供的unitree_ros包含Go2W和B2W的URDF描述，unitree_mujoco支持B2W的MuJoCo仿真。社区项目**go2w_rl_gym**（ShengqianChen，33 stars）基于unitree_rl_gym修改，专门针对Go2-W的RL训练+部署。**aCodeDog/legged-robots-manipulation**提供B2-W + Z1机械臂的loco-manipulation RL代码（IROS 2024 Oral）。

**Upkie（upkie/upkie，334 stars）** 是一个独特的全开源轮足双足机器人（硬件+软件），由Stéphane Caron开发。它提供MPC平衡器、PD平衡器和Gymnasium RL环境三种控制方案，支持PyBullet、Genesis、MuJoCo三种仿真器。Apache-2.0许可证，非常适合作为轮足算法的低成本验证平台。

其余值得关注的项目包括：**Col_DIABLO_Issac_RL**（DDTRobot，基于Wheel-Legged-Gym为DIABLO机器人训练RL策略）、**FLORES**（新型轮足设计，HIM-RL，Isaac Gym→MuJoCo→真机的sim-to-real流水线）、**foc-wheel-legged-robot**（Skythinker616，桌面级全开源轮足机器人，LQR控制，中文社区非常活跃）。

### 1.3 轮足MPC专用项目

除OCS2和wb_mpc_centauro外，轮足MPC领域的公开实现较为稀缺。**foc-wheel-legged-robot**使用简化LQR（倒立摆模型），适合入门理解但算法深度有限。基于Pinocchio+Crocoddyl的轮足轨迹优化目前**没有专门的公开项目**，但Crocoddyl的multi-contact框架和Aligator的约束处理能力理论上均可扩展至轮足场景。

**表1：轮足机器人开源项目汇总**

| 项目 | Stars | 团队/机构 | 核心算法 | 仿真器 | 关键C++库 | 许可证 |
|------|-------|-----------|----------|--------|-----------|--------|
| OCS2 | ~1,300 | ETH RSL | SLQ/SQP/IPM MPC | ROS+RViz | Pinocchio, HPIPM, CppAD | BSD-3 |
| Wheel-Legged-Gym | 566 | SUSTech ClearLab | PPO RL + VMC | IsaacGym | rsl_rl (Python) | BSD-3 |
| rl_sar | 1,200 | fan-ziqi | C++策略部署 | Gazebo | LibTorch, ROS | BSD-3 |
| robot_lab | 高 | fan-ziqi | PPO RL + DR | IsaacLab | rsl_rl (Python) | BSD-3 |
| Isaac-RL-2wheel | 282 | jaykorea | PPO RL | IsaacLab | rsl_rl (Python) | — |
| tron1-rl-isaacgym | 121 | LimX Dynamics | PPO RL | IsaacGym | ONNX Runtime (C++) | BSD-3 |
| Upkie | 334 | Stéphane Caron | MPC/PD/RL | PyBullet/Genesis/MuJoCo | Pinocchio, Pink | Apache-2.0 |
| wb_mpc_centauro | 低 | IIT HHCM | WB-MPC (OCS2) | ROS+RViz | OCS2, Pinocchio, qpOASES | BSD-3 |
| go2w_rl_gym | 33 | 社区 | PPO RL | IsaacGym | rsl_rl (Python) | BSD-3 |
| foc-wheel-legged | 高 | Skythinker616 | LQR | 嵌入式 | ESP32 (C) | 开源 |
| FLORES | 低 | 学术界 | HIM-RL PPO | IsaacGym+MuJoCo | rsl_rl (Python) | 开源 |

---

## 第二部分：足式机械臂（Loco-Manipulation）开源项目

### 2.1 MPC框架——OCS2与Crocoddyl双雄并立

**OCS2的mobile_manipulator示例**是loco-manipulation MPC研究的最佳起点。该示例将6DOF机械臂+移动底盘的2D位置+航向角统一为一个运动学优化问题，控制输入包括6个关节速度+前进/旋转底盘速度。支持Franka Panda、Kinova Jaco2（6/7DOF）、Ridgeback+UR5、PR2等平台，包含基于URDF碰撞体的自碰撞回避。其**legged_robot示例**则展示了四足切换系统的MPC，步态通过`SwitchedModelReferenceManager`管理，可在运行时通过`GaitReceiver`修改。两个示例的结合为"四足+机械臂"的统一MPC提供了完整参考。

核心论文：Sleiman et al. "A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation" (RAL 2021)，Minniti et al. "Whole-Body MPC for a Dynamically Stable Mobile Manipulator" (RAL 2019)，Pankert & Hutter "Perceptive Model Predictive Control for Continuous Mobile Manipulation" (RAL 2020)。ETH RSL还开源了**perceptive_mpc**（leggedrobotics/perceptive_mpc），结合感知信息的MPC用于连续移动操作。

**Crocoddyl（loco-3d/crocoddyl，~1,200 stars）** 由LAAS-CNRS和University of Edinburgh的Carlos Mastalli团队开发，实现了DDP、FDDP（Feasibility-driven DDP）、Box-FDDP等算法。其核心设计是**model-data分离**：model描述系统特性，data存储计算中间结果。Crocoddyl通过`DifferentialActionModelContactFwdDynamics`处理接触动力学，接触力在正向动力学中隐式计算。提供`whole_body_manipulation.ipynb`等Jupyter notebook示例。C++依赖仅Pinocchio+Eigen+Boost，轻量且易于集成。Crocoddyl**不使用显式任务层级**——所有代价项通过加权单目标组合，但Mastalli et al. (TRO 2023)提出了"Inverse-Dynamics MPC via Nullspace Resolution"方法实现零空间分解。BSD-3-Clause许可证。

**Aligator（Simple-Robotics/aligator，~260 stars）** 是Crocoddyl的"精神续作"，由INRIA Willow团队的Justin Carpentier和LAAS-CNRS的Nicolas Mansard开发。其核心算法**ProxDDP**（Proximal Constrained DDP）通过近端增广拉格朗日法原生支持**硬约束**（等式+不等式），这是相对Crocoddyl的关键改进——轮足的滚动约束和loco-manipulation的运动学约束可以作为硬约束直接嵌入。Aligator要求Pinocchio ≥ 3.4，提供Crocoddyl兼容接口。2024年RSS论文展示了在Solo四足机器人上的实时全身MPC。BSD-2-Clause许可证。

**FATROP（meco-group/fatrop，261 stars）** 是KU Leuven开发的快速约束非线性OCP求解器，基于结构化内点法+广义Riccati递归，通过CasADi接口使用。在四旋翼避障问题上比IPOPT快**5.2倍**（135ms vs 706ms），可用于复杂loco-manipulation轨迹规划。LGPL许可证。

### 2.2 全身控制（WBC）框架——从WBIC到分层QP

**MIT Cheetah-Software（mit-biomimetics/Cheetah-Software，~2,800 stars）** 是WBC的经典参考实现。其WBIC（Whole-Body Impulse Control）使用**分层零空间投影**确定关节位置/速度优先级，再通过QP优化接触力（跟踪MPC反力同时稳定姿态）。决策变量为接触力+关节加速度，输出前馈力矩+关节PD命令。MPC使用单刚体（SRB）凸QP模型，~30Hz运行；WBIC在500Hz执行。**代码完全自包含**——使用手写的浮动基座动力学引擎（非Pinocchio）、OSQP求解器、LCM通信、自研QT仿真器。模板化C++设计（`Quadruped<float>`/`Quadruped<double>`）。缺点是仅支持四足，无机械臂扩展。MIT许可证（隐含）。

**legged_control（qiayuanl/legged_control，~700 stars）** 是OCS2在真实四足机器人上部署最广泛的框架。架构为NMPC（OCS2 SLQ）+ 分层QP WBC + EKF状态估计，已在Unitree A1/Go1上验证。WBC使用分层零空间QP，决策变量包括关节加速度`ddq`、接触力`f`、关节力矩`τ`，严格遵守任务优先级。多个实验室报告可在数小时内完成部署。依赖OCS2、Pinocchio、hpp-fcl、ROS Noetic、ros-controls。BSD-3-Clause许可证。

**qm_control（skywoodsz/qm_control，~200 stars）** 是目前唯一**真正意义上的开源四足机械臂MPC+WBC完整实现**。基于OCS2开发，将浮动基座+4条腿+机械臂视为统一运动学链，支持游戏手柄同时控制底盘速度和末端执行器位姿。提供四个分支：main（全身运动）、feature-force（力扰动稳定性）、feature-compliance（全身柔顺）、feature-real（硬件部署）。配套论文：Zhang et al. "Whole-body Compliance Control for Quadruped Manipulator with Actuation Saturation" (IROS 2024)。对于想研究loco-manipulation MPC+WBC的工程师，**qm_control是最直接的参考项目**。

**TSID（stack-of-tasks/tsid，~550 stars）** 由LAAS-CNRS的Andrea Del Prete和Nicolas Mansard开发，实现了基于**逆动力学的分层最小二乘规划（HLSP）**。每个优先级层是一个标准最小二乘问题，通过级联QP求解。决策变量为关节加速度`ddq`+接触力`f`，运动方程`M·ddq + h = S^T·τ + J_c^T·f`作为等式约束。支持eiquadprog（默认，~0.6ms/人形机器人）、ProxQP、OSQP等QP后端。API设计优雅：`addMotionTask`、`addForceTask`、`addRigidContact`。BSD-2-Clause许可证。

**OpenSoT（ADVRHumanoids/OpenSoT，~100 stars）** 由IIT开发，提供**操作符化的分层QP**：`task1 / task2`（层级优先级）、`task1 + task2`（同级聚合），解耦了任务/约束描述与求解器。支持速度级IK、加速度级逆动力学、力矩级控制三种模式，多个QP后端（qpOASES默认、eiquadprog、OSQP、ProxQP）。LGPL-2.1许可证。

**ARC-OPT/wbc（~50 stars，DFKI）** 是最模块化的WBC库，将机器人模型、QP求解器和控制器完全解耦，支持闭链结构和浮动基座。Pinocchio+qpOASES，BSD-3-Clause许可证。配套论文发表在JOSS 2024。

**OpenLoong-Dyn-Control（~500 stars）** 是"青龙"人形机器人的MPC+WBC开源实现，使用SRB凸QP MPC + 分层WBC，任务优先级可配置。基于MuJoCo仿真，**不依赖Pinocchio/OCS2**（完全自包含），Apache-2.0许可证。

### 2.3 RL-based Loco-Manipulation——端到端与分层之争

**Deep Whole-Body Control（MarkFzp/Deep-Whole-Body-Control，365 stars）** 是loco-manipulation RL的里程碑项目，由CMU的Zipeng Fu和Deepak Pathak开发（CoRL 2022 Oral，最佳系统论文提名）。其核心是**端到端统一策略**——单个PPO策略同时控制Unitree Go1的12个腿关节+WidowX 250机械臂的6个关节，共18自由度。关键创新是**ROA（Regularized Online Adaptation）**——用单阶段在线估计替代RMA的两阶段方法，实现sim-to-real迁移。观测空间包含关节位置/速度、基座姿态、角速度、目标末端位姿和隐变量；动作空间为所有18DOF的关节位置目标。基于IsaacGym Preview 3 + rsl_rl。

**Visual Whole-Body Control（Ericonaldo/visual_wholebody，389 stars）** 采用**分层架构**——底层RL策略（PPO via rsl_rl）控制全身运动+末端跟踪（50Hz），高层视觉运动策略（模仿学习 via skrl）规划任务（10Hz）。目标平台为Unitree B1 + Z1臂（19DOF）。分层设计使高层策略可复用不同底层，更适合复杂操作任务。

**UMI on Legs（real-stanford/umi-on-legs，446 stars）** 来自Stanford的Shuran Song团队（CoRL 2024），其核心创新是**"操作为中心"的全身控制器**——底层RL策略跟踪任务空间的末端轨迹，使原本为固定基座机械臂训练的操作策略可以"即插即用"地迁移到移动平台。基于legged_gym + IsaacGym，目标平台为Go2 + ARX5臂。

**HumanPlus（MarkFzp/humanplus，Stanford）** 是人形loco-manipulation的代表项目（CoRL 2024），使用分层框架：底层PPO策略实现人体运动跟踪（HST: Humanoid Shadowing Transformer），高层模仿学习（HIT: Humanoid Imitation Transformer，基于ACT/Mobile ALOHA）完成具体操作任务。基于IsaacGym v4训练Unitree H1。

**RAMBO（ETH RSL，2025）** 代表了**混合MPC+RL的前沿方向**——模型化WBC生成前馈力矩（QP求解），RL策略提供反馈校正残差。这种"MPC为主、RL增强"的架构在精确力控+位置跟踪任务中表现优异（推购物车、托盘平衡等）。基于Isaac Lab，目标平台为Unitree Go2。代码状态待确认。

**legged-robots-manipulation（aCodeDog，IROS 2024 Oral）** 专门针对轮足loco-manipulation——Unitree B2-W + Z1机械臂，使用**臂约束课程学习**，基于IsaacGym + legged_gym + rsl_rl。

**表2：足式机械臂开源项目汇总**

| 项目 | Stars | 团队/机构 | 核心算法 | 仿真器 | 关键库 | 许可证 |
|------|-------|-----------|----------|--------|--------|--------|
| OCS2 (mobile_manip) | ~1,300 | ETH RSL | SLQ/SQP MPC | ROS+RViz | Pinocchio, HPIPM, CppAD | BSD-3 |
| Crocoddyl | ~1,200 | LAAS-CNRS | DDP/FDDP | Gepetto-viewer | Pinocchio, Eigen | BSD-3 |
| Cheetah-Software | ~2,800 | MIT | Convex MPC+WBIC | 自研QT仿真 | OSQP, LCM (自研动力学) | MIT |
| legged_control | ~700 | qiayuanl | NMPC+WBC (OCS2) | Gazebo | OCS2, Pinocchio | BSD-3 |
| TSID | ~550 | LAAS-CNRS | 分层QP逆动力学 | — | Pinocchio, eiquadprog | BSD-2 |
| UMI on Legs | 446 | Stanford | PPO RL | IsaacGym | rsl_rl (Python) | — |
| visual_wholebody | 389 | 多机构 | 分层RL(PPO+IL) | IsaacGym | rsl_rl, skrl | 自定义 |
| Deep WBC | 365 | CMU | 端到端PPO RL | IsaacGym | rsl_rl (Python) | — |
| Aligator | ~260 | INRIA+LAAS | ProxDDP | — | Pinocchio≥3.4 | BSD-2 |
| qm_control | ~200 | 学术界 | MPC+WBC (OCS2) | ROS+RViz | OCS2, Pinocchio | — |
| OpenLoong | ~500 | 上海人形中心 | SRB MPC+WBC | MuJoCo | 自包含 | Apache-2.0 |
| OpenSoT | ~100 | IIT | 分层QP (iHQP) | — | Pinocchio/XBot, qpOASES | LGPL-2.1 |
| Spot SDK | ~2,500 | Boston Dynamics | 高层API | Isaac Lab | gRPC, Python | 私有 |
| spot-rl-example | 33 | BD AI Institute | MPC+RL残差 | Isaac Lab | ONNX Runtime | MIT |
| rl-mpc-locomotion | 447 | 社区 | RL预测MPC权重 | IsaacGym | PyTorch (Python) | — |

---

## 第三部分：C++库生态与使用频次分析

### 3.1 Pinocchio生态占据绝对主导地位

**Pinocchio（~3,200 stars，BSD-2）** 是整个轮足和loco-manipulation领域的基础设施。它实现了RNEA、ABA、CRBA的**解析导数**（而非数值或自动微分），这是其相对RBDL和Drake的核心竞争力。Pinocchio通过`JointModelFreeFlyer`原生支持浮动基座，四元数表示旋转，构型空间`q`和速度空间`v`的维度不匹配通过李群操作处理。轮子关节建模为连续旋转关节。Pinocchio支持C++17/20/23，多线程友好，CasADi/CppAD自动微分集成，代码生成支持最大化性能。它被OCS2、Crocoddyl、Aligator、TSID、OpenSoT、Pink、legged_control、qm_control等几乎所有主流项目使用。

**Drake（~3,850 stars，BSD-3，MIT CSAIL+TRI）** 是另一个重量级选择，提供完整的多体动力学+数学规划+轨迹优化+控制系统工具箱。其hydroelastic接触模型、符号/自动微分标量类型、以及与SNOPT/IPOPT/Gurobi/Mosek的集成使其在接触隐式轨迹优化方面非常强大。但Drake的Bazel构建系统和"系统-端口-图"的架构范式学习曲线较陡，在轮足/loco-manipulation社区的采用率低于Pinocchio。

**RBDL（~161 stars，zlib许可证）** 是Featherstone算法的轻量实现，仅依赖Eigen3，但缺乏自动微分支持且维护活跃度低，已基本被Pinocchio取代。

### 3.2 QP求解器的选择直接影响实时性

在WBC场景中，**ProxQP**（ProxSuite的一部分，~800 stars）表现最优——典型WBC问题求解时间约**24±7µs**，比OSQP的167±93µs快约**7倍**，比qpOASES的142-238µs快约6-10倍。ProxQP使用原始-对偶增广拉格朗日方法，支持密集和稀疏后端，SIMD加速。

**HPIPM（~500-600 stars，BSD-2）** 专为MPC结构化QP设计，利用带状结构的Riccati分解，在OCS2的SQP求解器和acados中使用。其底层依赖BLASFEO（针对小矩阵优化的线性代数库）。对于轮足MPC中的级联QP问题，HPIPM是最高效的选择。

**OSQP（~2,000 stars，Apache-2.0）** 基于ADMM的稀疏QP求解器，在MIT Cheetah的MPC中使用，优势是纯C实现、可嵌入、支持代码生成。

**qpOASES（~300 stars，LGPL-2.1）** 作为参数化主动集方法在warm-starting场景（如机器人站立稳态）中有优势，但在接触切换场景下性能下降明显。

### 3.3 RL部署的C++方案

两种主流路径：**LibTorch**（rl_sar采用）通过`torch.jit.trace`导出`.pt`模型，C++端加载推理，优势是与PyTorch生态无缝对接，劣势是依赖较重。**ONNX Runtime**（LimX tron1-rl-deploy采用，MIT许可证，~16,000 stars）通过`.onnx`格式部署，更轻量、跨平台，支持CUDA/TensorRT/OpenVINO执行提供者。在NVIDIA Jetson平台上，**TensorRT**可作为ONNX Runtime的加速后端进一步提升推理速度。

典型部署流程：IsaacGym/IsaacLab训练（PPO，4096+并行环境）→ 导出模型（JIT/.pt 或 ONNX/.onnx）→ MuJoCo/Gazebo sim2sim验证 → C++部署（LibTorch/ONNX Runtime，50-200Hz控制频率）。

**表3：C++库使用频次统计**

| 库名称 | 类别 | 出现项目数 | 代表项目 | 许可证 |
|--------|------|-----------|----------|--------|
| Pinocchio | 刚体动力学 | **12+** | OCS2, Crocoddyl, TSID, Aligator, legged_control, qm_control, OpenSoT, wbc, Pink, Upkie | BSD-2 |
| Eigen | 线性代数 | **所有C++项目** | 全部 | MPL-2 |
| OSQP | QP求解 | 6+ | Cheetah-Software, TSID, OpenSoT, 多个WBC实现 | Apache-2.0 |
| qpOASES | QP求解 | 5+ | wb_mpc_centauro, OpenSoT, TSID, acados | LGPL-2.1 |
| HPIPM | 结构化QP | 4+ | OCS2 SQP, acados, FATROP关联 | BSD-2 |
| ProxQP | QP求解 | 4+ | TSID, Aligator, Pink, ProxSuite生态 | BSD-2 |
| CppAD/CppADCodeGen | 自动微分 | 3+ | OCS2, Pinocchio集成 | EPL-1.0 |
| hpp-fcl/Coal | 碰撞检测 | 4+ | OCS2, legged_control, Pinocchio生态 | BSD-2 |
| LibTorch | RL部署 | 3+ | rl_sar, dstx123/unitree_rl, OCS2 mpcnet | BSD-3 |
| ONNX Runtime | RL部署 | 3+ | LimX tron1-deploy, spot-rl-example | MIT |
| LCM | 实时通信 | 3+ | Cheetah-Software, Cassie研究, Unitree部署 | LGPL-2.1 |
| Boost | 通用 | 8+ | OCS2, Pinocchio, Crocoddyl, TSID等 | BSL-1.0 |
| MuJoCo | 仿真 | 6+ | OpenLoong, Upkie, sim2sim验证 | Apache-2.0 |

---

## 第四部分：关键问题解答

### Q1：轮足领域2024-2025年最值得研究的3-5个开源项目

1. **OCS2 + wb_mpc_centauro**：唯一公开的轮足MPC实现（CENTAURO），理解滚动约束如何嵌入SLQ-MPC的最佳参考
2. **robot_lab + rl_sar**（fan-ziqi）：覆盖Go2W/B2W/Tita/M20五款轮足机器人的完整训练→部署工具链，**1,200 stars**的rl_sar是C++部署标杆
3. **Wheel-Legged-Gym**（ClearLab）：VMC创新使闭链轮足机构的RL训练成为可能，566 stars，社区活跃
4. **Upkie**：全开源轮足双足机器人（硬件+软件），提供MPC/PD/RL三种控制方案，Apache-2.0许可证，最低成本的验证平台
5. **tron1-rl-isaacgym + tron1-rl-deploy**（LimX）：轮足变体WF_TRON1A + ONNX Runtime C++部署，展示完整的工业级sim-to-real流水线

### Q2：Loco-manipulation领域最值得研究的3-5个开源项目

1. **qm_control**：唯一完整的四足机械臂MPC+WBC开源实现，基于OCS2，含全身柔顺和力控分支
2. **OCS2**（mobile_manipulator + legged_robot示例）：MPC框架的"教科书级"参考，C++代码质量极高
3. **Deep Whole-Body Control**（CMU）：端到端RL loco-manipulation的里程碑，ROA sim-to-real方法影响深远
4. **Crocoddyl + Aligator**：欧洲学术界的轨迹优化标准，Aligator的ProxDDP支持硬约束是关键进步
5. **UMI on Legs**（Stanford）：操作为中心的全身控制器设计理念新颖，实现了操作策略的跨平台复用

### Q3：轮足相比普通四足在算法上的核心难点

**非完整约束（Nonholonomic Constraints）是核心差异。** 轮子的滚动约束要求接触点速度沿轮轴方向为零（无侧滑），沿滚动方向由轮速决定。这在代码中体现为：

- **MPC层面**：在OCS2中，滚动约束编码为状态-输入等式约束`g1(x,u)=0`。ETH的方案将接触点的x/z方向加速度约束为零，并将轮子转速与底盘速度通过运动学关系耦合。这使得QP问题多出2-4个等式约束（每个轮子），增加了求解复杂度。
- **RL层面**：Wheel-Legged-Gym等项目**不显式建模滚动约束**——轮子作为连续旋转关节在URDF中定义，RL策略通过奖励函数（平滑运动奖励、能量惩罚等）隐式学习合理的轮子使用策略。
- **模式切换**：轮足机器人需要在"滚动模式"（高效平地移动）和"行走模式"（越障）间切换。MPC方案通过接触模式序列（state machine或整数规划）实现，RL方案通过统一策略隐式学习模式切换时机。Swiss-Mile的Science Robotics 2024论文使用分层RL实现平滑模式过渡。
- **轮子作为额外自由度**：四足有12个关节自由度，轮足增加4个轮子自由度到16个。全身动力学模型的维度增加，WBC的QP规模相应增大。

### Q4：Loco-manipulation相比单独locomotion/manipulation的全身控制技术增量

**统一运动学链处理**是核心技术增量。单独的locomotion将腿视为接触执行器，单独的manipulation将臂视为任务执行器。Loco-manipulation需要将浮动基座+腿+臂视为一个**耦合的运动链**，核心挑战包括：

- **动力学耦合**：臂的运动产生对基座的反力矩，影响locomotion稳定性。在WBC中体现为：臂的加速度出现在基座动量方程的右端项中，必须在QP中联合考虑腿部接触力和臂部力矩。qm_control通过OCS2对完整链的MPC实现这一耦合。
- **任务优先级冲突**：locomotion需要基座稳定，manipulation需要末端精度，两者可能冲突。TSID和OpenSoT通过分层QP处理——locomotion约束（接触力、ZMP）在高优先级，manipulation任务在低优先级的零空间中执行。Deep WBC则用端到端RL隐式学习这种权衡。
- **接触状态扩展**：locomotion的接触在足端，manipulation可能在臂末端引入新接触（如推门、搬物），WBC需要动态管理接触集合。OCS2通过`SwitchedModelReferenceManager`支持运行时接触切换。
- **冗余度利用**：四足+臂的总自由度（如Go2+Z1为18DOF）远大于末端任务维度（6DOF），如何利用冗余度优化locomotion性能是关键。零空间投影方法将manipulation任务映射到低优先级空间，高优先级空间保留给locomotion稳定性。

### Q5：MPC派 vs RL派在轮足和loco-manipulation中的优劣与趋势

**MPC派（OCS2/Crocoddyl）的优势**：物理可解释性强、安全约束可显式编码（力矩限制、关节限位、摩擦锥、滚动约束）、不需要大规模GPU训练、在新任务/新约束上可快速调整。对于轮足的滚动约束和loco-manipulation的精确力控，MPC可以将物理约束作为硬约束直接嵌入优化问题。OCS2的自动微分+代码生成使其MPC可达**50-100Hz实时**。

**RL派（IsaacLab+legged_gym）的优势**：对模型不确定性和外部扰动的鲁棒性更强、不需要精确的动力学模型、可处理复杂地形和高度非线性行为（如跳跃、翻滚）、端到端训练简化了系统设计。轮足的模式切换和loco-manipulation的接触丰富场景中，RL避免了显式枚举所有接触模式的combinatorial explosion。

**2024-2025趋势：混合方法正在成为主流。** RAMBO（ETH RSL 2025）使用MPC生成前馈力矩+RL提供反馈校正；rl-mpc-locomotion（447 stars）使用RL预测MPC权重参数；Spot RL Research Kit使用MPC为基础+RL残差增强。这种"model-based为主、learning增强"的范式兼顾了安全性和鲁棒性，是最有前景的方向。

### Q6：熟悉SLAM和规控的工程师的推荐入门路径

**第一阶段（2-4周）——MPC基础**：
1. 构建OCS2的`ocs2_mobile_manipulator_ros`示例，理解SLQ-MPC的OptimalControlProblem组合机制
2. 阅读OCS2的legged_robot示例，理解切换系统的步态调度
3. 配套论文：Sleiman et al. (RAL 2021) "A Unified MPC Framework"

**第二阶段（2-4周）——WBC理解**：
1. 阅读MIT Cheetah-Software中WBIC的实现（`robot/MIT_Controller`），理解分层零空间投影
2. 部署legged_control在Gazebo中的Unitree A1，理解NMPC+WBC全流程
3. 配套论文：Kim et al. (2019) "Highly Dynamic Quadruped Locomotion via WBIC and MPC"

**第三阶段（2-4周）——轮足/loco-manipulation**：
1. 用Wheel-Legged-Gym或robot_lab训练第一个轮足RL策略
2. 部署qm_control的四足机械臂MPC+WBC（ROS+RViz仿真）
3. 用rl_sar理解C++ LibTorch部署流程

**第四阶段（持续）——深入研究**：
1. 在OCS2中实现自定义的轮足等式约束（非完整滚动约束）
2. 基于Aligator的ProxDDP尝试带硬约束的轮足轨迹优化
3. 实现混合MPC+RL架构（参考RAMBO的设计思路）

**SLAM背景的独特优势**：地形感知→模式切换决策（何时用轮/何时用腿），点云处理→可通行性分析→规划层输入。建议将SLAM能力与感知MPC（perceptive_mpc）结合。

### Q7：未提及的重要开源项目补充

- **rl-mpc-locomotion（silvery107，447 stars）**：RL预测MPC权重的分层控制，MIT Cheetah的Python移植+IsaacGym训练
- **Cafe-MPC（Notre Dame ROAM Lab）**：级联保真度MPC+基于值函数的WBC（VWBC），消除WBC调参需求，TRO 2024
- **Control Toolbox（ethz-adrl/control-toolbox）**：ETH ADRL的C++ iLQR/GNMS/NMPC库，含HyQ四足模型，概念上是OCS2的前身
- **Quadruped-PyMPC（IIT DLS Lab）**：Python四足MPC（SRB模型），支持acados梯度法和JAX采样法，MuJoCo集成
- **QpSolverCollection（AIST）**：统一C++接口封装qpOASES/OSQP/NASOQ/HPIPM/ProxQP/qpmad/LSSOL，极其适合QP求解器基准测试
- **WholebodyVLA（OpenDriveLab）**：视觉-语言-动作模型用于统一loco-manipulation（ICLR 2026）
- **SoFTA（CMU LECAR Lab）**：双智能体解耦框架——上半身高频精确+下半身低频鲁棒
- **Horizon（IIT）**：基于CasADi+IPOPT的轨迹优化框架，已在Boston Dynamics Spot上验证
- **LocoManipulationRL（2361098148）**：可重构机器人肢体的端到端loco-manipulation RL（MLP+GNN策略）
- **awesome-loco-manipulation（aCodeDog）**：最全面的loco-manipulation论文列表，含Go2-Arx/B1-Z1/Aliengo-Z1等URDF模型
- **awesome-wheeled-legged（XinLang2019）**：轮足领域的策展列表

### Q8：轮足和loco-manipulation的C++技术栈与普通四足/机械臂的差异

**与普通四足相比，轮足的专用技术需求**：
- **非完整约束处理**：普通四足的接触约束是完整的（点接触力），轮足需要处理非完整滚动约束。在OCS2中体现为额外的`StateInputEqualityConstraint`，在WBC中体现为优先级任务中的等式约束行。目前没有专门的"轮足约束库"，均在通用框架（OCS2/TSID）中手动实现。
- **混合整数规划需求**：模式切换（轮/足）在MPC框架中可能需要混合整数QP。OCS2的切换系统框架通过预定义模式序列回避了整数变量，但完全自主的模式选择仍是开放问题。
- **闭链机构处理**：许多轮足机器人（如Ascento的四连杆、DIABLO）使用闭链机构。Pinocchio 3.x开始支持约束关节（闭环），但多数项目仍使用VMC（Wheel-Legged-Gym）或解析解（Ascento论文）处理。

**与普通机械臂相比，loco-manipulation的专用技术需求**：
- **浮动基座动力学**：普通机械臂基座固定，动力学更简单。Loco-manipulation需要Pinocchio的`JointModelFreeFlyer`+全链路动力学。MIT Cheetah-Software选择手写浮动基座引擎以最大化性能，而不使用通用库。
- **接触力优化**：普通机械臂无需考虑地面反力。Loco-manipulation的WBC需要同时优化腿部接触力和关节力矩，QP的决策变量和约束数量显著增加（从6DOF臂的6变量到19DOF四足臂的25+变量）。
- **实时性要求更高**：locomotion控制需要500Hz-1kHz的更新率，远高于典型机械臂的100-200Hz。这推动了ProxQP（24µs/次）、HPIPM等高性能求解器的采用，以及LCM（替代ROS话题）用于低延迟通信。

**专用库方面**：目前不存在"轮足专用"或"loco-manipulation专用"的C++库。所有项目都基于通用库（Pinocchio+QP求解器+MPC框架）的组合，差异体现在约束配置和任务定义层面。这对研究者而言意味着：**掌握Pinocchio+一种QP求解器（推荐ProxQP或OSQP）+一种MPC框架（推荐OCS2）即可覆盖两个领域的大部分算法需求。**

---

## 结论：三个值得投入的技术方向

本次调研覆盖了轮足和loco-manipulation领域约**40个开源项目**和**16个核心C++库**。从研究投入产出比的角度，三个方向最值得关注：

**一是OCS2+Pinocchio的MPC+WBC分层架构**，这是学术界验证算法创新的标准平台。qm_control证明了在OCS2上扩展四足机械臂控制的可行性，下一步可类似地扩展轮足约束。对于有C++背景的工程师，这是最自然的切入点。

**二是robot_lab+rl_sar的RL训练+C++部署全栈**，这是工程落地最快的路径。**1,200 stars**的rl_sar已经支持Go2W/B2W的真机部署，LibTorch推理代码可直接复用。RL方法回避了轮足非完整约束的显式建模难题，适合快速出结果。

**三是混合MPC+RL的架构创新**，以RAMBO和rl-mpc-locomotion为代表。这一方向结合了MPC的安全可控性和RL的鲁棒性，是2025年最明确的研究趋势。对于同时掌握规控（MPC/WBC）和学习（RL部署）的工程师而言，这是最有差异化竞争力的研究方向。

---

## 四份子方向深度调研报告：总览

> **从项目调研到子方向深度钻研**：前面两份项目调研（第二批 C++ 项目全景 + 轮足与足式机械臂开源项目全景调研）从**工具和基础设施**的角度扫描了开源生态。接下来的四份独立深度调研报告则切换到**子方向纵深**视角，分别为续篇教学大纲中四段核心内容提供论文级数据支撑：
>
> | 深度调研报告 | 对应教学章节 | 核心技术主题 | 论文规模 |
> |---|---|---|---|
> | **D1 轮足机器人** | 复合/60_轮式运动学与Pfaffian-81（6 章） | Pfaffian 约束、Bjelonic NMPC、轮足 RL、模式切换 | ~30 篇 |
> | **D2 移动操作** | 复合/120_底盘臂联合规划-85（4 章） | 底盘+臂联合规划、OCS2 mobile_manipulator、VLA 端到端、ALOHA/UMI | ~27 篇 |
> | **D3a 四足+臂** | 复合/160_四足臂动力学概览-91（6 章） | qm_control MPC+WBC、Deep-WBC/VBC/UMI-on-Legs RL、RAMBO 混合架构 | ~30 篇 |
> | **D3b 人形** | 复合/220_经典人形全身控制-95（4 章） | 经典 LIPM/DCM WBC、人形全身 RL、ASAP sim-to-real、FALCON 力敏感 | ~28 篇 |
>
> **四份报告的共同结构**：每份报告均包含（1）按时代划分的论文清单、（2）核心开源项目精读（含文件级路径）、（3）该子方向特有的数学增量、（4）硬件平台详解、（5）博士研究开放问题、（6）对应章节的详细大纲。这一统一结构使读者可以在不同子方向间进行横向对比。
>
> **四份报告之间的技术关联**：虽然四个子方向各有侧重，但它们共享大量技术基础设施和方法论，在阅读时应注意以下交叉点：
> - **MPC 框架**：D1 的 Bjelonic NMPC（复合/70_轮足混合MPC）和 D3a 的 qm_control（复合/170_qm_control精读）都基于 OCS2，D2 的 mobile_manipulator（复合/130_OCS2_mobile_manipulator）同样复用 OCS2 的 SQP+HPIPM 后端。三份报告的 MPC 部分形成"同一框架、不同约束配置"的递进关系。
> - **WBC 与任务优先级**：D1 的轮足 WBC 需处理 Pfaffian 滚动约束，D3a 的四足+臂 WBC 需处理行走与操作的优先级冲突，D3b 的人形 WBC 需处理 30+ DoF 的角动量调节——三者都是 TSID/HQP 框架在不同规模系统上的扩展。
> - **RL 训练与部署**：D1 的 Wheel-Legged-Gym（复合/80_Wheel_Legged_Gym_RL）、D3a 的 Deep-WBC/VBC（复合/180_Deep_WBC精读-89）、D3b 的 ExBody/ASAP/FALCON（复合/230_人形全身RL-95）都使用 IsaacLab + rsl_rl 训练管线。D2 的 VLA（复合/140_VLA移动操作）则代表了从 RL 到基础模型的范式跃迁。
> - **底盘控制的复用**：D1 轮足报告中的 MPC 底盘控制框架也适用于 D2 移动操作中的轮式底盘控制——两者的底盘运动学（差动/全向/阿克曼）是共通的，区别在于 D1 的底盘同时承担运动和支撑功能，而 D2 的底盘仅提供 SE(2) 平移。
> - **操作策略的迁移**：D2 报告中的 VLA/扩散策略（openpi、UMI）与 D3a 报告中的 UMI-on-Legs（复合/200_UMI_on_Legs精读）直接相关——UMI-on-Legs 的核心贡献正是将 D2 的桌面操作策略零样本迁移到 D3a 的四足+臂平台上。D3b 的 FALCON 则将力敏感控制引入人形平台，构成"D2 策略→D3a 平台→D3b 扩展"的技术链条。
> - **Sim-to-Real**：D1 的轮足 sim-to-real（复合/110_轮足SimToReal与硬件）、D3a 的 RAMBO 残差学习（复合/210_RAMBO混合MPC_RL）、D3b 的 ASAP delta-action（复合/240_ASAP_SimToReal）代表了三种不同的 sim-to-real 策略，在 复合/270_SimToReal统一方法论（Sim-to-Real 统一方法论）中被横向对比。

