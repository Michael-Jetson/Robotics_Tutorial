## 第四部分：C++库生态全景调研

第三部分从工程技术维度剖析了八大横向主题。本部分换一个视角，从"库"的维度系统梳理规控工程师的工具箱：哪些库是必修的、哪些是选修的、它们之间如何协作。两部分互为补充——前者按技术主题组织，本部分按库类别组织，交叉阅读可获得最完整的技术选型视野。

**规控领域的C++库生态以Eigen、Pinocchio和OMPL三大不可替代的库为核心，围绕"动力学建模→轨迹优化→实时控制"这条技术主线展开。** 与SLAM领域共享Eigen/Sophus等数学基础设施不同，规控工程师需要额外掌握QP/NLP求解器（OSQP、Ipopt）、自动微分（CppAD）和最优控制框架（OCS2、Crocoddyl）等独有技术栈。当前趋势表明，GPU加速仿真（Isaac Lab、Genesis）正在重塑策略训练流程，而Pinocchio的模板化架构+CppADCodeGen的代码生成模式已成为现代规控C++开发的黄金范式。本报告系统覆盖8大库类别、50+核心库、7个代表性项目深度剖析，为C++教学大纲扩展和技术选型提供决策参考。

---

### 一、优化求解器：规控算法的计算引擎

优化求解器是规控系统中最核心的底层依赖——MPC需要在毫秒级求解QP，轨迹优化依赖NLP求解器，力控依赖实时QP。以下按问题类型分类梳理。

### QP求解器对比

| 求解器 | GitHub Stars | 核心算法 | 语言/接口 | 许可证 | 实时性 | 代表性用户 |
|--------|------------|---------|----------|--------|--------|-----------|
| **OSQP** | ~2,000 | ADMM | C（osqp-eigen封装） | Apache-2.0 | ✅优秀 | Drake、Autoware、OCS2 |
| **qpOASES** | ~520 | 在线活跃集 | 原生C++ | LGPL-2.1 | ✅良好 | MIT Cheetah、acados |
| **HPIPM** | ~650 | IPM+Riccati递推 | C | BSD-2 | ✅✅卓越 | acados（默认后端）、OCS2 |
| **ProxQP** | ~300 | 近端增广拉格朗日 | C++17/Eigen | BSD-2 | ✅✅卓越 | Crocoddyl、TSID、Pinocchio生态 |
| **qpSWIFT** | ~155 | IPM+Nesterov-Todd | ANSI C | GPL-3.0 | ✅良好 | Ghost Robotics Vision60 |
| **PIQP** | ~120 | IPM+PMM | C++14/header-only | BSD-2 | ✅良好 | EPFL最优控制研究 |

**OSQP**是最广泛使用的通用QP求解器，基于ADMM算法仅需一次矩阵分解，支持代码生成实现嵌入式部署，Drake和Autoware均默认集成。**qpOASES**以参数化活跃集策略著称，MIT Cheetah 3的凸MPC控制器正是基于qpOASES在**1ms内**求解10-16步时域的密集QP。**HPIPM**利用Riccati递推将OCP结构QP的每次迭代降至O(N)复杂度，是acados的默认后端，在嵌入式MPC场景中性能最优。**ProxQP**是INRIA为Pinocchio/Crocoddyl生态量身打造的新一代求解器，在密集逆运动学QP上比OSQP快**7倍**（24±7μs vs 167±93μs）。

### NLP求解器与MPC专用框架

**Ipopt**（~1,500⭐，EPL-2.0）是规控领域最重要的NLP求解器，采用原点-对偶内点法配合滤波线搜索，Drake默认集成。用户需实现`TNLP`接口的`eval_f()`/`eval_grad_f()`/`eval_g()`/`eval_jac_g()`/`eval_h()`五个虚函数。**SNOPT**（商业，SQP算法）是Drake的首选NLP求解器，在稀疏大规模轨迹优化中表现优异。二者均为离线规划设计，不适合高频实时MPC。

**acados**（~1,300⭐，BSD-2）是面向嵌入式实时NMPC的专用框架，核心采用RTI（Real-Time Iteration）策略——每个控制周期仅执行一步SQP迭代，以HPIPM为默认QP后端，通过CasADi生成动力学代码，最终输出自包含C代码可直接部署至嵌入式平台。comma.ai的openpilot自动驾驶辅助系统使用acados。**FATROP**（~300⭐，LGPL）是KU Leuven开发的新兴OCP求解器，利用广义Riccati递推将Ipopt算法加速**5倍**，2023年入围IROS最佳论文。

### SDP/SOCP与建模工具

**SCS**（~600⭐，MIT）是CVXPY的默认后端，支持LP/SOCP/SDP/指数锥等所有锥优化。**Mosek**（商业）是最成熟的商用锥优化求解器。二者均被Drake集成，用于旋转估计的SDP松弛、Graphs of Convex Sets运动规划等场景。**CVXGEN**是早期面向小规模QP（变量≤100）的代码生成器，生成无依赖的平面C代码，适用于嵌入式MPC。

**教学优先级**：OSQP（高）、Ipopt（高）、acados（高）、qpOASES（高）、HPIPM（中高）、ProxQP（中高）。

---

### 二、运动规划库：从采样到优化的全谱系

### 采样与搜索类规划器

**OMPL**（Open Motion Planning Library，~2,000⭐，BSD）是运动规划领域的事实标准，实现了RRT\*/PRM/BIT\*/AIT\*/KPIECE/EST/SBL等数十种算法。其架构围绕`StateSpace`（状态空间）、`SpaceInformation`（空间信息）、`Planner`（规划器）、`StateValidityChecker`（碰撞检测回调）四大虚接口展开。MoveIt2将OMPL作为默认规划后端，通过`OMPLPlannerManager`插件桥接。`ompl::control`命名空间扩展了动力学约束规划，用户实现`StatePropagator`虚接口定义系统动力学，支持SST/KPIECE等运动学规划算法。

**nav2_smac_planner**（Nav2子模块，Apache-2.0）是ROS2移动机器人导航的标准全局规划器。其核心特色是**模板化A\*搜索**——`AStarAlgorithm<NodeT>`模板类可接受`Node2D`（栅格A\*）、`NodeHybrid`（Hybrid-A\*，含Dubins/Reeds-Shepp曲线）、`NodeLattice`（状态格子）三种节点类型，实现了代码复用与性能优化的平衡。**SBPL**（~352⭐，BSD）提供ARA\*/AD\*等保证次优性的搜索算法，在ROS1中通过`sbpl_lattice_planner`使用。

### 基于轨迹优化的规划器

| 规划器 | 核心方法 | 碰撞处理 | 优化器 | 使用场景 |
|--------|---------|---------|--------|---------|
| **CHOMP** | 函数梯度下降 | 欧氏距离场(EDF) | 协变更新(A⁻¹为度量) | MoveIt2内置 |
| **STOMP** | PI²路径积分 | 任意代价函数(无需梯度) | 噪声轨迹加权组合 | MoveIt2插件 |
| **TrajOpt** | 序贯凸优化(SQP) | Bullet连续碰撞检测 | QP(BPMPD/Gurobi) | Tesseract(ROS-Industrial) |
| **KOMO** | 增广拉格朗日/SQP | 配置图碰撞特征 | Ipopt/NLopt/自研 | RAI框架(柏林工大教学) |

**TOPP-RA**（~600⭐，MIT）解决时间最优路径参数化问题：给定几何路径，在速度/加速度/力矩约束下求最快通过速度曲线。通过反向传播可达集（求解小规模LP）+正向提取时间最优速度，实现**100%成功率**。C++版依赖Eigen和qpOASES，已集成至MoveIt2的时间参数化适配器。

### 无人机专用规划器

**Fast-Planner**（~2,500⭐，GPL-3.0，HKUST）采用"Kinodynamic A\*前端搜索 + B样条后端优化"架构。`plan_env`模块通过深度图光线投射构建概率体素图并增量更新ESDF；`bspline_opt`模块用NLopt优化B样条控制点，代价包含平滑性+ESDF距离惩罚+动力学可行性。**EGO-Planner-v2**（ZJU FAST Lab，GPL-3.0）的核心创新是**无需ESDF**——直接从栅格地图计算碰撞梯度，优化器从B样条升级为MINCO（最小控制量）多项式表示，使用L-BFGS求解，规划时间降至**~1ms**。

**教学优先级**：OMPL（高）、nav2_smac_planner（高）、TOPP-RA（高）、CHOMP/STOMP（中）、Fast-Planner/EGO-Planner（中——无人机方向必学）。

---

### 三、机器人动力学库：从运动学到微分的模板化革命

### Pinocchio：现代规控的动力学基石

**Pinocchio**（~3,200⭐，BSD-2，INRIA/LAAS-CNRS）是当前规控领域最核心的动力学库。它实现了Featherstone的全部经典算法——RNEA（逆动力学）、ABA（正动力学）、CRBA（广义惯量矩阵计算），以及这些算法的**解析梯度**（这是相比RBDL和Bullet的关键区分点）。

Pinocchio的C++架构具有**教科书级别的模板化设计**：所有算法以`Scalar`类型参数化，`Model`存储不可变拓扑参数，`Data`存储可变计算缓冲区（严格的Model/Data分离确保线程安全）。这意味着只需将标量类型替换为`CppAD::AD<CppAD::cg::CG<double>>`，同一份动力学代码即可自动生成优化后的C导数代码——这正是OCS2实现千赫兹级MPC的技术核心。

Pinocchio支持URDF/SDF模型加载，通过HPP-FCL/Coal进行碰撞检测。其下游生态极其丰富：**Crocoddyl**（DDP轨迹优化）、**TSID**（任务空间逆动力学）、**OCS2**（切换系统最优控制）、**Aligator**（约束轨迹优化）均以Pinocchio为动力学后端。

### 其他动力学库对比

**RBDL**（~685⭐，zlib）更贴近Featherstone教材的伪代码实现，API简洁易懂，适合教学和生物力学研究，但缺乏解析导数支持。**Drake的MultibodyPlant**（~8,500⭐整体，BSD-3）是最完整的多体动力学引擎，独创**Hydroelastic接触模型**（体积压力场而非点接触），通过SceneGraph统一管理碰撞/可视化几何体，支持`AutoDiffXd`和`symbolic::Expression`标量类型。**MuJoCo**（~12,800⭐，Apache-2.0）是纯C库设计的物理仿真器，运行时零动态内存分配，单核人形机器人仿真步仅需~1ms，JAX-GPU版本（MJX）可实现百万级并行仿真。**DART**（~940⭐，BSD-2）为Gazebo提供物理引擎后端，支持URDF/SDF/MJCF/SKEL四种格式。

| 库 | Stars | 解析导数 | 模板化AD | URDF | 实时性 | 定位 |
|---|---|---|---|---|---|---|
| **Pinocchio** | 3,200 | ✅ | ✅ CppAD/CasADi | ✅ | ✅ | 算法库（需配合仿真器） |
| **RBDL** | 685 | ❌ | ❌ | ✅ | ✅ | 教学/生物力学 |
| **Drake** | 8,500 | ✅(AutoDiff) | ✅ | ✅ | ⚠️ | 全栈工具包 |
| **MuJoCo** | 12,800 | ✅(有限) | ❌(C API) | ⚠️(支持URDF加载,原生格式为MJCF) | ✅✅ | 仿真器+RL |
| **DART** | 940 | ❌ | ❌ | ✅ | ✅ | 多体动力学仿真库(Gazebo默认物理引擎、DartEnv RL环境) |

---

### 四、碰撞检测库：规划安全的守门人

**FCL**（Flexible Collision Library，~1,670⭐，BSD-3）是MoveIt2和Drake的默认碰撞检测库。窄相位使用libccd实现的GJK/EPA算法，宽相位提供SAP/区间树/AABB树等多种策略，支持基本几何体、三角网格和OctoMap八叉树。自v0.6起以标量类型模板化。

**HPP-FCL/Coal**（更名为Coal，~560⭐，BSD-2）是FCL的高性能分支，提供自研的GJK/EPA实现（比FCL快**2-15倍**），支持Nesterov加速碰撞检测、安全裕度、接触面片计算。**Pinocchio生态（Crocoddyl、TSID、OCS2）全部使用HPP-FCL/Coal**而非原版FCL。

**Bullet碰撞模块**（~14,400⭐整体，zlib）可独立于物理引擎使用，TrajOpt直接使用Bullet的连续碰撞检测实现凸-凸距离查询。**libccd**（~200⭐，BSD-3）是GJK/EPA/MPR的轻量C实现，FCL的窄相位内核即基于libccd。

**教学优先级**：FCL/HPP-FCL（高——理解GJK/EPA/BVH是规控工程师必备知识）。

---

### 五、数学与自动微分：规控的计算基础

**Eigen**是整个机器人C++生态的数学基石——header-only模板库，表达式模板实现零临时变量的惰性求值和SIMD向量化。规控中Eigen的使用与SLAM有显著差异：SLAM侧重稀疏矩阵（图优化/Bundle Adjustment），而规控侧重密集矩阵运算（质量矩阵、Riccati方程、轨迹矩阵）以及Eigen的**自定义标量类型**能力（支撑CppAD等自动微分框架）。

**Sophus**（~1,800⭐，MIT）和**manif**（~1,700⭐，MIT）均为基于Eigen的李群header-only库。Sophus 活跃度自 2024 年起明显下降（新功能开发趋缓），但在SLAM社区仍是事实标准。manif覆盖更多群（SE₂(3)、SGal(3)、Bundle<>），提供所有操作的**解析雅可比**，并兼容`ceres::Jet`自动微分类型，附带论文"A micro Lie theory for state estimation"是极好的入门教材。

**CppAD**（~553⭐，EPL/GPL）通过运算符重载记录计算图（tape），支持前向/反向模式自动微分。**CppADCodeGen**（~200⭐）在CppAD基础上进一步生成优化后的C源代码，编译为动态库后加载执行，导数求值速度比tape回放快**2-3个数量级**。OCS2的工作流程为：用`CppAD::AD<CppAD::cg::CG<double>>`标量类型记录Pinocchio动力学→CppADCodeGen生成C代码→编译为.so→运行时加载实现实时MPC。这一"模板化动力学+代码生成"范式是现代规控C++开发的**黄金实践**。

---

### 六、控制专用库与ROS2规控生态

### 最优控制框架三强

**OCS2**（~1,300⭐，BSD-3，ETH RSL）是面向切换系统的非线性最优控制C++工具箱，支持SLQ/DDP/iLQR/SQP求解器，通过增广拉格朗日处理路径约束。其MPC架构设计精良：`MPC_BASE`在后台线程异步运行求解器，`MPC_MRT_Interface`以线程安全方式向1kHz实时控制线程提供策略插值。深度集成Pinocchio（质心动力学/运动学）和CppADCodeGen（自动微分代码生成）。已在ANYmal四足等实体机器人上验证。

**Crocoddyl**（~1,200⭐，BSD-3，LAAS-CNRS）是面向接触序列的DDP/FDDP求解器。用户构造`ShootingProblem`（由一系列`ActionModel`组成），选择`SolverFDDP`或`SolverBoxDDP`求解。以Pinocchio为动力学后端，MathBase模板支持标量类型灵活替换。已实现100Hz机械臂MPC。

**Drake的系统框架**（~8,500⭐，BSD-3）采用Simulink启发的设计：`LeafSystem<T>`是最小构建单元（含输入/输出端口、状态、参数），`Diagram`通过`DiagramBuilder`组合子系统，`Context`将可变状态与系统定义分离。`MathematicalProgram`统一封装SNOPT/Ipopt/Gurobi/Mosek/OSQP/SCS等十余种求解器。

### ROS2控制栈

**ros2_control**（~860⭐，Apache-2.0）是ROS2硬件抽象控制框架。`ControllerManager`通过pluginlib加载控制器插件（继承`ControllerInterface`），`ResourceManager`管理硬件插件（继承`SystemInterface`）。控制器的`update()`在实时线程中执行，配合`realtime_tools`包实现RT安全的数据发布。**ros2_controllers**（~730⭐）提供JointTrajectoryController（三次/五次样条插值）、DiffDriveController、AdmittanceController等标准控制器。

### Nav2与MoveIt2的插件生态

**Nav2**（~2,500⭐，Apache-2.0）的全部算法均通过pluginlib插件加载。**MPPI控制器**是其最先进的本地控制器——采样~2000条高斯噪声扰动轨迹，通过插件化的Critic函数（CostCritic、GoalCritic、PathAlignCritic等）评分，软最大加权得到最优控制。支持差速/全向/Ackermann运动模型，CPU向量化优化后可在普通硬件上达到**50-100Hz**。整体任务编排通过BehaviorTree.CPP行为树实现。

**MoveIt2**（~1,000⭐，BSD）的规划管线经2024年重构后支持**链式规划**（如OMPL→STOMP：采样结果作为优化种子）和**并行规划**（多个管线同时运行，选择最优解）。运动学求解器通过`KinematicsBase`虚接口插件化，支持KDL（默认Jacobian数值IK）、TRAC-IK、BioIK。MoveIt Servo提供100Hz+实时笛卡尔/关节抖动控制。

---

### 七、仿真环境的代际更替

| 仿真平台 | Stars | 物理引擎 | GPU加速 | ROS2集成 | C++/Python | 定位 |
|---------|-------|---------|--------|---------|-----------|------|
| **Gazebo Sim** | — | DART(默认)/Bullet(插件) | 渲染(OGRE 2.x) | ros_gz_bridge | C++核心 | ROS标准仿真器 |
| **MuJoCo** | 12,800 | 自研凸优化接触 | MJX(JAX-GPU) | 社区维护 | C+Python绑定 | RL/控制研究金标准 |
| **Isaac Lab** | 3,500 | PhysX(GPU) | 全GPU物理+渲染 | 原生桥接 | Python为主 | 大规模RL训练 |
| **Genesis** | 20,000 | 自研多求解器统一 | 极致GPU(10-80x Isaac Gym) | 暂无 | 纯Python | 新兴全能仿真+生成式AI |

Gazebo Classic（Gazebo 11）已于2025年随ROS Noetic一起EOL。**Gazebo Sim**（原Ignition）采用模块化ECS架构，物理引擎通过插件加载。**MuJoCo MPC（MJPC）**（~2,000⭐）提供交互式实时MPC框架，内置iLQG/梯度下降/预测采样算法。**Genesis**（2024年12月发布，20,000⭐）号称比Isaac Gym快10-80倍，RTX 4090上可达4300万FPS，26秒训练出步态策略，但目前仍在早期阶段。

---

### 八、代表性开源项目技术栈深度剖析

### 机械臂：MoveIt2

MoveIt2的完整技术栈：OMPL（采样规划）+ FCL/Bullet（碰撞检测）+ KDL/TRAC-IK（运动学）+ Eigen3 + Boost + OctoMap + pluginlib + BehaviorTree.CPP（任务级）。C++17标准，核心设计模式为**Plugin/Strategy**（所有规划器/运动学求解器/碰撞检测器均为pluginlib插件）+ **Observer**（PlanningSceneMonitor监听ROS话题保持场景同步）+ **Facade**（MoveGroupInterface简化API）。

### 无人机：EGO-Planner-v2 vs Fast-Planner

二者对比揭示了无人机规划器的演进方向：Fast-Planner依赖NLopt求解器+ESDF地图，EGO-Planner-v2使用L-BFGS（LBFGS-Lite header-only库）+直接栅格碰撞梯度，省去ESDF构建的**O(N log N)**开销。轨迹表示从B样条演进为MINCO多项式。二者均为ROS1/catkin构建，C++11/14标准，非插件化的单体类设计。

### 四足机器人：OCS2 vs Cheetah-Software

**OCS2的legged_robot示例**展示了学术级MPC的完整技术栈：Pinocchio（质心动力学+末端执行器运动学）→ CppAD/CppADCodeGen（自动微分代码生成）→ SLQ/SQP求解器（HPIPM作为QP后端）→ HPP-FCL（自碰撞约束）→ MPC_MRT_Interface（异步求解+实时插值）。C++14标准，核心特色是**以标量类型模板化实现AD透明性**。

**MIT Cheetah-Software**则代表工程化路线：自研空间向量代数的`FloatingBaseModel<T>`（模板化float/double），凸MPC使用单刚体模型+qpOASES在<1ms内求解。**不依赖ROS**（使用LCM通信），不依赖外部动力学库（自实现Featherstone ABA/CRBA），体现了嵌入式部署的极简主义。

### 移动机器人/自动驾驶：Nav2 vs Autoware

**Nav2**的技术栈以pluginlib插件为核心架构，所有规划器/控制器/代价图层/行为均为运行时可替换的插件。MPPI控制器使用Eigen进行批量轨迹计算并利用CPU向量化（AVX2）优化。整体编排由BehaviorTree.CPP驱动。

**Autoware**的规控模块采用MPC横向控制器+PID纵向控制器架构。MPC控制器通过`VehicleModelInterface`虚基类支持运动学/动力学自行车模型切换，通过`QPSolverInterface`虚基类支持OSQP/Eigen最小二乘切换。Lanelet2提供高精地图的车道级路由和几何查询。行为速度规划器的场景模块（交叉口、人行横道、红绿灯）通过pluginlib插件化。

---

### 九、五大关键问题的系统回答

### 问题1：规控领域最不可替代的三个C++库

1. **Eigen**——100%的规控项目依赖Eigen进行矩阵运算，且其自定义标量类型机制使CppAD自动微分成为可能
2. **Pinocchio**——现代规控的动力学核心，Crocoddyl/TSID/OCS2/Aligator均以其为后端，模板化架构+解析导数的组合无可替代
3. **OMPL**——采样规划的事实标准，MoveIt2的默认后端，实现了最全面的规划算法集合

次席：OSQP/qpOASES（QP求解器至少需掌握一个）、FCL/HPP-FCL（碰撞检测不可或缺）。

### 问题2：规控 vs SLAM的C++库生态重叠与独特性

| 维度 | 共用库 | 规控独有 | SLAM独有 |
|------|--------|---------|---------|
| 数学基础 | **Eigen**、**Sophus/manif** | — | — |
| 优化求解 | Ceres（偶用） | **OSQP、Ipopt、qpOASES、HPIPM、acados** | g2o、GTSAM、iSAM2 |
| 运动/规划 | — | **OMPL、CHOMP、STOMP、TOPP-RA** | 回环检测、词袋 |
| 动力学 | — | **Pinocchio、RBDL、Drake** | — |
| 碰撞检测 | — | **FCL、HPP-FCL** | OctoMap |
| 自动微分 | Ceres::Jet | **CppAD、CppADCodeGen** | — |
| 控制框架 | — | **OCS2、Crocoddyl、TSID** | — |
| 物理仿真 | — | **MuJoCo、Bullet、DART** | — |

核心差异在于：SLAM优化是**图优化**（稀疏/结构化的最小二乘），规控优化是**最优控制**（QP/NLP/MPC）和**运动规划**（采样/搜索/轨迹优化）。

### 问题3：规控工程师相比SLAM工程师额外需要的C++技能

- **模板化标量类型与自动微分**：理解Pinocchio/OCS2的`Scalar`模板参数如何与CppAD交互，能编写可被自动微分的函数
- **QP/NLP问题建模能力**：将机器人约束（关节限制、碰撞避免、摩擦锥、力矩限制）转化为数学规划的等式/不等式约束
- **实时C++编程**：`EIGEN_RUNTIME_NO_MALLOC`断言、避免动态内存分配、实时线程与求解器线程的同步（mutex/lock-free）
- **pluginlib插件开发**：MoveIt2/Nav2/ros2_control的插件架构要求熟练使用`PLUGINLIB_EXPORT_CLASS`宏和工厂模式
- **Eigen高级用法**：固定大小矩阵（栈分配、循环展开）、对齐（`EIGEN_MAKE_ALIGNED_OPERATOR_NEW`）、表达式模板（避免过早求值）、Map（零拷贝包装原始内存）
- **空间向量代数**：6D空间力/速度、Plücker坐标、Featherstone算法的C++实现

### 问题4：Panda机械臂完整规控项目的最小库依赖

实现"运动规划+轨迹优化+力控"的最小依赖列表：

- **Eigen3**——所有数学运算
- **Pinocchio**——Panda的URDF加载、正/逆运动学、动力学（质量矩阵、科氏力、重力项）、雅可比计算
- **HPP-FCL/Coal**——碰撞检测（Pinocchio默认依赖）
- **OMPL**——采样规划器（RRT-Connect生成初始路径）
- **一个QP求解器**（OSQP或ProxQP）——力控QP、逆动力学QP
- **TOPP-RA**——轨迹时间参数化
- **ros2_control + franka_ros2**——硬件抽象和实时控制接口
- **MoveIt2**（可选但推荐）——规划管线编排、碰撞场景管理

如追求高级轨迹优化可加入**Crocoddyl**（DDP求解）或**OCS2**（MPC控制），如需力控可加入**TSID**。

### 问题5：规控领域C++代码的当前趋势

**趋势一：GPU仿真重塑策略训练。** Isaac Lab和Genesis将物理仿真从CPU单线程推向GPU万级并行。Genesis声称26秒训练步态策略。但经典C++规控栈（OMPL/Pinocchio/OCS2）仍主导部署侧——GPU训练C++部署是主流模式。

**趋势二：Pinocchio模板化+CppADCodeGen代码生成范式。** OCS2开创的"用模板化Pinocchio记录计算图→CppADCodeGen生成优化C代码→编译为.so→实时加载"已成为高性能MPC的标准范式。这对C++模板编程提出了很高要求。

**趋势三：Drake的系统化建模方法。** Drake用`LeafSystem<T>`+`Diagram`+`Context`的组合提供了对标Simulink的纯C++系统建模框架，`AutoDiffXd`和`symbolic::Expression`标量类型使同一模型可用于仿真/优化/验证。MIT本科课程（Underactuated Robotics、Manipulation）已全面转向Drake。

**趋势四：ProxSuite生态的崛起。** INRIA团队围绕Pinocchio构建了一整套高性能库——ProxQP（QP求解）、Coal（碰撞检测）、Crocoddyl（轨迹优化）、TSID（力控）、Aligator（约束优化），形成了紧密集成的法国学派生态。

**趋势五：ROS2的全面插件化与行为树编排。** Nav2的MPPI控制器、MoveIt2的链式规划管线、ros2_control的控制器管理器，均以pluginlib为核心实现运行时可配置。BehaviorTree.CPP成为任务级编排的标准。这使得**插件开发能力**成为规控ROS2工程师的必备技能。

---

### 十、教学大纲建议的库优先级总览

| 优先级 | 库名 | 类别 | 理由 |
|--------|------|------|------|
| **必修** | Eigen | 数学 | 一切C++机器人代码的基础 |
| **必修** | Pinocchio | 动力学 | 现代规控的动力学核心 |
| **必修** | OMPL | 运动规划 | 采样规划事实标准 |
| **必修** | OSQP或qpOASES | QP求解 | MPC/力控必需 |
| **必修** | FCL/HPP-FCL | 碰撞检测 | 规划安全性保障 |
| **必修** | ros2_control | ROS2控制 | 硬件控制标准框架 |
| **核心选修** | Ipopt | NLP求解 | 轨迹优化标准工具 |
| **核心选修** | CppAD/CppADCodeGen | 自动微分 | 高性能MPC的关键技术 |
| **核心选修** | Crocoddyl或OCS2 | 最优控制 | DDP/MPC框架二选一 |
| **核心选修** | Nav2 | 移动导航 | ROS2导航标准 |
| **核心选修** | MoveIt2 | 机械臂规划 | 操作规划标准 |
| **核心选修** | acados | 嵌入式MPC | 实时NMPC部署 |
| **核心选修** | Drake | 全栈工具 | 最完整的建模-优化-仿真平台 |
| **进阶选修** | MuJoCo | 仿真 | RL/控制研究必需 |
| **进阶选修** | Sophus/manif | 李群 | 与SLAM共用 |
| **进阶选修** | TOPP-RA | 时间参数化 | 轨迹后处理 |
| **进阶选修** | ProxQP | QP求解 | Pinocchio生态新一代 |
| **进阶选修** | HPIPM | 结构化QP | OCP专用高性能求解 |

### 结论

规控C++库生态呈现出清晰的**分层架构**：底层是Eigen数学基础，中间层是Pinocchio动力学+FCL碰撞检测+OMPL运动规划，上层是OCS2/Crocoddyl最优控制框架和MoveIt2/Nav2应用框架。与SLAM生态的核心差异不在于C++语言特性本身，而在于**问题建模方式**——SLAM是图优化的因子图世界，规控是最优控制的QP/NLP世界。

当前最值得关注的技术演进是Pinocchio模板化架构与CppADCodeGen的结合——它将"写一次动力学代码，自动获得高性能导数"变为现实，这需要规控工程师不仅理解算法，更要精通C++模板元编程。同时，GPU仿真平台（Isaac Lab、Genesis）正在快速降低策略训练门槛，但经典C++规控栈在部署侧的地位短期内不会动摇——"Python训练、C++部署"正成为行业的主流模式。


