## 第四至第八篇：深度调研报告

> **从教学路径到数据支撑**：前面两篇教学大纲（增量式 足式/70_腿足简化模型理论-56 + 续篇 复合/10_复合机器人全景-100）给出了"学什么、怎么学"的完整路径。接下来的深度调研报告则提供支撑这些教学设计的原始数据：项目源码级分析、论文谱系梳理、数学推导细节、硬件选型依据和博士开放问题论证。教学大纲中每个"指定论文"和"项目精读"条目的详细评述，均可在对应的调研报告中找到。

## 四足与人形规控 C++ 项目全景调研（第二批）

> **调研时间**：2026 年 4 月
> **范围界定**：本批次专注于**四足机器人**和**人形机器人**的**传统规控栈** C++ 开源项目。严格**排除**纯 Python/PyTorch 的 RL 训练仓库（即使机器人硬件相同，如果项目核心是 PPO/SAC 训练循环而非 MPC/WBC/轨迹优化 C++ 实现，不纳入）。允许纳入**纯 C++ 实现的 RL+MPC 混合范式**项目。
> **本批次**：T0 深度剖析 7 个核心项目 + T1 中度覆盖 15 个项目 + 横向分析，共 **22 个项目**。（原文附带的增量式教学大纲已合并至本文档其他章节，此处仅保留调研内容。）
> **配套文档**：本文档为第二批调研，与第一批《机械臂规控C++项目全景调研_第一批.md》配合使用。

---

## 摘要与核心发现

本批次覆盖了四足/人形规控生态中最具代表性的 22 个 C++ 项目。结合第一批机械臂调研，整个机械臂+腿足生态共 60 个项目。核心发现有四：

**发现一：四足/人形的 C++ 生态高度围绕 Pinocchio 集中**。OCS2（腿足 MPC）、Crocoddyl（DDP/FDDP）、TSID（逆动力学 WBC）、Aligator（ProxDDP）、bipedal_locomotion_framework（iCub）、legged_control 全部以 Pinocchio 为动力学后端——**Pinocchio 在腿足领域已达到 Eigen 在 SLAM 领域的"基础设施"地位**。唯二的例外是 MIT Cheetah-Software（自实现 `FloatingBaseModel<T>` 空间向量代数，为嵌入式部署做极端裁剪）和 OpenLoong（虽然用了 Pinocchio，但整个技术栈自成一派）——这两个例外恰好代表了"嵌入式极简主义"和"中国工业学派自主化"两条不同于法国 INRIA 学派的路线。

**发现二：腿足 MPC 的"两层控制"架构已成主导范式**。高层 10-100Hz 的 MPC（优化质心动力学 + 接触力 + 轨迹），底层 500Hz-1kHz 的 WBC（把 MPC 的力/加速度参考解算为关节扭矩）——**legged_control、MIT Cheetah、OpenLoong、IHMC、Bipedal-Locomotion-Framework 全部采用这个架构**。这与无人机的"单层 MPC 直接输出电机扭矩"范式有本质不同，原因是腿足有接触切换、欠驱动基座、高维状态空间，单层 MPC 无法同时照顾所有时间尺度。理解这个"MPC 生成参考 → WBC 追踪参考"的分工是理解所有腿足代码的钥匙。

**发现三：CppAD + CppADCodeGen 的"预编译 MPC"范式是腿足实时性的核心密码**。OCS2 的 `AutoDiff` 模式、Crocoddyl 的 code generation、Aligator 的 Pinocchio 模板化集成——这些项目的共同点是**用模板化 Pinocchio + CppAD 记录计算图，然后通过 CppADCodeGen 生成高度优化的 C 代码、编译为 .so、实时加载**。这种做法能把 7-DOF 机械臂的动力学 + 碰撞约束求导从数百微秒压到十几微秒，使得 500Hz+ 的实时 MPC 成为可能。这是无人机侧（用 ACADO/acados 自研代码生成）和 SLAM 侧（完全不需要）都没有的独特技术栈。

**发现四：C++ 实现的 RL+MPC 混合范式目前仍是少数**。调研发现，大部分 RL+MPC 项目的 RL 组件在 Python 侧（PyTorch），通过文件 I/O 或 Socket 传给 C++ MPC 使用——真正做到 "纯 C++ 集成" 的目前主要是 **OCS2-MPC-Net**（学到的价值函数被嵌入 OCS2 的代价函数）和**silvery107/rl-mpc-locomotion**（但 RL 侧仍为 PyTorch）。这说明 RL+MPC 的 C++ 工业化还没到来——这反过来是机器人工程师的**前沿机会**。

---

## 第一部分：T0 核心项目深度剖析（7 个）

### T0-1. OCS2（leggedrobotics/ocs2）—— 腿足/机械臂 MPC 的开源标杆

**元信息**：
- **GitHub**：https://github.com/leggedrobotics/ocs2
- **Stars**：约 1,100+（截至 2026 年 4 月）
- **License**：BSD-3-Clause
- **C++ 标准**：C++14/17
- **主要维护者**：Farbod Farshidian（ETH 腿足机器人组 RSL，现为 NVIDIA），其他核心贡献者 Ruben Grandia、Jean-Pierre Sleiman、Jan Carius、Sotaro Katayama 等
- **活跃度**：ROS1 主分支 + ROS2 分支并行维护

**项目定位与核心算法**：

OCS2（Optimal Control for Switched Systems）是 **ETH RSL 实验室开源的 MPC 通用框架**。它的核心创新是把"**腿足机器人的接触切换**"作为一等公民来处理——腿足机器人每步都在"脚着地→脚抬起→脚再落地"的模式间切换，每种模式有不同的动力学和约束。OCS2 的 "Switched Systems" 名称直接来源于此。

核心算法包括：**SLQ**（Sequential Linear Quadratic，连续时间约束 DDP）、**SQP**（Multiple-shooting 基于 HPIPM）、**SLP**（Sequential Linear Programming）、**iLQR**、**iPDDP** 等。约束处理支持 Augmented Lagrangian 和 relaxed barrier 两种策略。**ANYmal 四足的 perceptive locomotion**（Grandia et al., T-RO 2022）就是用 OCS2 做的，这个项目把 OCS2 的应用推到了工业级。

**C++ 工程亮点**：

1. **"Pinocchio + CppAD + CppADCodeGen"预编译 MPC 的开山范式**。OCS2 最大的技术贡献是把下面这条流水线做成产品化：(a) 用 `template<typename Scalar>` 的 Pinocchio 记录机器人动力学；(b) 把 `Scalar` 替换为 `CppAD::AD<CppAD::cg::CG<double>>` 运行一遍代码，自动生成计算图；(c) 用 CppADCodeGen 把计算图编译为高度优化的 C 代码，导出为 .so；(d) 运行时 dlopen 加载这个 .so 做推理。这条流水线让 ANYmal 的 MPC 能在嵌入式 Jetson 上以 100Hz 运行。关键代码：`ocs2_pinocchio/ocs2_pinocchio_interface/include/ocs2_pinocchio_interface/PinocchioInterface.h`、`ocs2_auto_diff/ocs2_cppad_code_gen/`。

2. **Centroidal Dynamics 抽象**。腿足机器人全身动力学超过 30 维（6 基座 + 12-24 关节），实时 MPC 直接优化非常困难。OCS2 提供了 Centroidal Model（质心动力学，6+N_leg*3 维）作为常用抽象——只保留"质心的线动量和角动量 + 接触力"，把关节层次下放给 WBC。核心类：`ocs2_centroidal_model/include/ocs2_centroidal_model/CentroidalModelPinocchioMapping.h`。

3. **MPC_MRT_Interface 异步求解 + 实时插值**。现实工程约束：MPC 求解需要 10-50ms，但控制器需要 1kHz 更新。OCS2 用 `MPC_MRT_Interface` 把 MPC 求解放在单独线程，主控线程以 1kHz 从最新解里**时间插值**获取当前指令——这是"高频控制器 + 低频规划器"经典架构的教科书级 C++ 实现。

4. **插件架构的约束与代价函数**。OCS2 里定义新的约束（例如"摩擦锥"、"自碰撞"、"足端位置"）只需继承 `StateInputConstraint` 或 `StateConstraint` 基类。这使得用户可以为新机器人快速组装 MPC 问题，而不需要改动求解器。

**关键文件路径**：
- `ocs2_core/include/ocs2_core/Types.h` —— 核心类型定义（state_vector_t、input_vector_t）
- `ocs2_ddp/` —— SLQ 和 iLQR 求解器
- `ocs2_sqp/` —— 基于 HPIPM 的 SQP 求解器
- `ocs2_oc/` —— Optimal Control 抽象层（Problem、Constraint、Cost）
- `ocs2_mpc/include/ocs2_mpc/MPC_MRT_Interface.h` —— MPC 主实时接口
- `ocs2_pinocchio/ocs2_centroidal_model/` —— 质心动力学 + Pinocchio 接口
- `ocs2_robotic_examples/ocs2_legged_robot/` —— ANYmal 腿足示例（完整可运行）

**教学价值评分**：★★★★★（5/5）

**评分理由**：(a) "Pinocchio + CppAD + CodeGen" 范式的开源标杆，这是腿足规控性能工程的核心技术栈；(b) 代码组织严格模块化（ocs2_core / ocs2_ddp / ocs2_sqp / ocs2_mpc / ocs2_pinocchio 分层清晰），适合精读；(c) MPC_MRT_Interface 是学习"异步实时控制"的典范；(d) 有完整的 ANYmal legged_robot 示例可运行，学习闭环完整。

**与其他项目的对比位置**：
- **vs Crocoddyl**：OCS2 偏工程应用（面向部署），Crocoddyl 偏学术研究（灵活性更高）；两者在算法上有重叠（都有 DDP 变体）
- **vs Drake**：OCS2 专注腿足 MPC，Drake 是全栈建模——但 Drake 也有 MultibodyPlant + DirectCollocation 可做轨迹优化
- **vs legged_control**：legged_control 是 OCS2 的**应用层**（在其上加 WBC、状态估计、硬件接口）

---

### T0-2. Crocoddyl（loco-3d/crocoddyl）—— DDP 约束轨迹优化的法国学派代表

**元信息**：
- **GitHub**：https://github.com/loco-3d/crocoddyl
- **Stars**：约 970+（截至 2026 年 4 月）
- **License**：BSD-3-Clause
- **C++ 标准**：**C++11/14/17/20** 全兼容
- **主要维护者**：Carlos Mastalli（赫瑞瓦特大学），核心作者包括 Justin Carpentier、Nicolas Mansard、Rohan Budhiraja、Wilson Jallet 等
- **活跃度**：2025 年仍在密集更新，算法迭代非常活跃

**项目定位与核心算法**：

Crocoddyl 是**法国 INRIA/LAAS 团队 + 爱丁堡联合开发的约束轨迹优化库**，核心是一系列 DDP 变体算法（FDDP、OdynSQP、Intro、Box-FDDP、Ipopt 集成等）。它和 OCS2 在同一时期出现，但设计哲学不同：OCS2 强调"工程就绪的 MPC 部署"，Crocoddyl 强调"灵活的算法原型 + 严格的数学正确性"。论文贡献：ICRA 2020 介绍 Crocoddyl 主架构、AutRob 2022 的 Feasibility-Driven DDP、T-RO 2023 的 Inverse-Dynamics MPC via Nullspace Resolution。

Crocoddyl 的应用覆盖：Talos 人形全身控制（Whole Body MPC on Talos）、ANYmal 四足 agile maneuvers、轻量级机器人协作（Solo8/12 开源四足）。

**C++ 工程亮点**：

1. **Virtualization + Model-Data 分离设计**——与 Pinocchio 的 CRTP 路线形成对比。Crocoddyl 官方 CONTRIBUTING.md 原话："Crocoddyl is designed using virtualization pattern for easy prototyping and yet efficient implementation. In early benchmark, we have shown that virtualization is as efficient as static polymorphism (i.e. CRTP design) for system dynamics higher than 16"——也就是说对于维度≥16 的系统（腿足机器人很容易超过），虚函数和 CRTP 性能差不多，但虚函数可读性和扩展性更好。**这是学习"何时选择虚多态、何时选择静态多态"的工业级决策案例**。

2. **ActionModel / ActionData 的时间步分离**。Crocoddyl 把一条轨迹分解为 N 个"时间步"（node），每步对应一个 `ActionModel`（描述动力学+代价+约束）和一个 `ActionData`（存该步的数据缓冲）。这是 Pinocchio Model-Data 分离范式的时间轴扩展——**避免了 Eigen 的临时矩阵分配**，使整个 DDP backward pass 里零堆分配。

3. **Contact Dynamics 作为一等公民**。腿足控制的核心是接触——Crocoddyl 提供 `DifferentialActionModelContactFwdDynamics` 类，内部用 `pinocchio::contactABA` 直接计算接触约束下的正动力学（KKT 方程求解）。关键文件 `include/crocoddyl/multibody/actions/contact-fwddyn.hxx`。

4. **完整的 CppAD 自动微分 + 代码生成支持**。Crocoddyl 和 OCS2 一样支持"**模板化 Scalar + CppADCodeGen**"预编译流水线，但 Crocoddyl 的更彻底——几乎每个类都有 `Tpl<Scalar>` 模板版本，用户可以自由实例化 `double` / `AD` / `codegen`。

5. **多线程支持通过 OpenMP**。Crocoddyl 的 Shooting Problem 有 `nthreads` 参数，backward pass 中每个时间步的 Jacobian 计算可以用 OpenMP 并行——这是腿足 MPC 中用多核 CPU 加速的直接范式。

**关键文件路径**：
- `include/crocoddyl/core/solver-base.hpp` —— 求解器基类
- `src/core/solvers/ddp.cpp`、`fddp.cpp`、`box-fddp.cpp` —— DDP 家族求解器
- `include/crocoddyl/core/actions/` —— 抽象 ActionModel（LQR、Impulse、Unicycle 等示例）
- `include/crocoddyl/multibody/actions/contact-fwddyn.hpp` —— 接触正动力学 ActionModel
- `include/crocoddyl/multibody/actions/impulse-fwddyn.hpp` —— 碰撞脉冲 ActionModel
- `examples/` —— 众多可运行示例（Talos arm、Solo quadruped、humanoid 等）

**教学价值评分**：★★★★★（5/5）

**评分理由**：(a) DDP 及其约束变体的最干净、最学术化的 C++ 实现；(b) Virtualization 设计的"为什么这样做"论证清晰，是学习 C++ 多态设计决策的极佳素材；(c) Contact Dynamics 的 ActionModel 抽象是"如何把复杂物理约束编码成优化问题"的教科书；(d) 学术活跃度极高，新算法（ProxDDP、Feasibility-Driven、Endpoint-Explicit DDP 等）不断加入。

**与其他项目的对比位置**：
- **vs OCS2**：算法家族相似（都含 DDP 变体），但 Crocoddyl 偏学术研究、虚多态风格；OCS2 偏工业部署、侧重代码生成
- **vs Aligator**（下面 T0-6）：Aligator 是 Crocoddyl 团队一些成员（Wilson Jallet、Justin Carpentier）在 2022+ 年推出的"下一代"——用 Proximal DDP 处理约束，性能和功能都超过 Crocoddyl，且可直接作为 Crocoddyl 的替代前端
- **vs Drake TrajectoryOptimization**：Drake 基于 MathematicalProgram 更通用但慢；Crocoddyl 专精 DDP 更快

---

### T0-3. TSID（stack-of-tasks/tsid）—— Pinocchio 生态的 WBC 基石

**元信息**：
- **GitHub**：https://github.com/stack-of-tasks/tsid
- **Stars**：约 360+
- **License**：BSD-2-Clause
- **C++ 标准**：C++17
- **主要维护者**：Andrea Del Prete（ex-LAAS，现特伦托大学），Justin Carpentier、Nicolas Mansard 等
- **活跃度**：持续活跃，支持 ProxQP + OSQP 多求解器后端

**项目定位与核心算法**：

TSID（Task Space Inverse Dynamics）是**法国学派的 Whole-Body Controller 标杆实现**。它解决"给定 MPC 或规划器的参考（目标末端位姿、质心轨迹、接触力），如何在每一个控制周期内算出 N 个关节的扭矩，让机器人既达成这些参考又不违反物理约束（关节限制、摩擦锥、扭矩限制）"。

核心思想是**分层二次规划（HQP / Hierarchical QP）**——优先级高的任务（如维持接触、避免自碰撞）作为硬约束，优先级低的任务（如末端跟踪）作为代价函数。唯一已实现的公式是 `InverseDynamicsFormulationAccForce`——决策变量是（基座加速度 + 关节加速度 + 接触力），关节扭矩不在决策变量里（但可以从加速度和接触力反推出来，减少变量数）。

**C++ 工程亮点**：

1. **`EIGEN_RUNTIME_NO_MALLOC` 运行时无分配断言**。TSID 的 CMakeLists.txt 默认开启 `EIGEN_RUNTIME_NO_MALLOC=ON`——如果 Eigen 在运行时做堆分配，直接 `assert` 失败。这迫使整个代码**在 startup 完成所有内存分配，运行时只做矩阵运算不分配**——这是实时控制代码的圣经。关键配置：`CMakeLists.txt` 中的 `option(EIGEN_RUNTIME_NO_MALLOC ... ON)`。

2. **Task + Constraint + Solver 三元分离**。TSID 把 WBC 问题建模为：**Task** 类描述"要追踪什么"（TaskSE3Equality、TaskCoMEquality、TaskJointPosture 等）；**Constraint** 类描述"必须满足什么"（等式/不等式/上下界）；**Solver** 类（EiquadprogFast、RtEiquadprog、ProxQP、OSQP）把上述组合求解为 QP。这是**"数学建模与求解器分离"**的优雅抽象。

3. **实时友好的多 QP 求解器后端**。TSID 支持 **EiquadprogFast**（动态矩阵但不在运行时分配）、**RtEiquadprog**（编译期矩阵大小，栈分配）、**ProxQP**（ProxSuite 后端）、**OSQP**（编译期可选）。用户可以根据场景选择——嵌入式用 RtEiquadprog，大维度用 ProxQP。

4. **Andrea Del Prete 的教学生态**。TSID 有完整的"配套课程"——Del Prete 教授在 YouTube 上有课程视频，仓库的 `exercizes/` 目录里有 ex_1 到 ex_5 完整的 Python 练习（LIPM 到 WBC 的桥接、humanoid 平衡、四足 walking 等）。这是少数**"论文 + 代码 + 视频 + 练习"**完整成一体的机器人开源项目。

**关键文件路径**：
- `include/tsid/tasks/` —— TaskSE3Equality、TaskCoMEquality、TaskJointPosture 等任务
- `include/tsid/contacts/` —— 接触约束（6D wrench、3D force 等）
- `include/tsid/solvers/` —— EiquadprogFast、RtEiquadprog、ProxQP 封装
- `include/tsid/formulations/inverse-dynamics-formulation-acc-force.hpp` —— HQP 建模
- `exercizes/ex_4_LIPM_to_TSID.py` —— LIPM + TSID 的双足行走

**教学价值评分**：★★★★★（5/5）

**评分理由**：(a) WBC 分层 QP 范式的最清晰 C++ 实现；(b) `EIGEN_RUNTIME_NO_MALLOC` 是学习实时 C++ 的第一课教材；(c) Task/Constraint/Solver 抽象具有很强的可扩展性（可直接迁移到你自己的机器人）；(d) 有完整教学配套。

**与其他项目的对比位置**：
- **vs OCS2 WBC**：OCS2 不自带 WBC，需要和 TSID 组合（或自研 WBC）
- **vs legged_control WBC**：legged_control 里有一个简化的自研 WBC，但 TSID 是工业级完整实现
- **vs Drake 的 InverseDynamicsController**：Drake 有同类组件但集成在 Diagram 框架里，独立使用不方便
- **vs MIT Cheetah QP**：MIT Cheetah 的 QP 是单纯的接触力分配（单层），TSID 是多层次 HQP——后者更通用但更复杂

---

### T0-4. legged_control（qiayuanl/legged_control）—— OCS2 在四足上的应用标杆

**元信息**：
- **GitHub**：https://github.com/qiayuanl/legged_control
- **Stars**：约 900+（这个数字对一个硕士生项目来说惊人地高）
- **License**：BSD-3
- **C++ 标准**：C++14/17
- **主要维护者**：Qiayuan Liao（UC Berkeley 博士生，2023 IROS 最佳应用论文候选）
- **活跃度**：2024 后由社区维护，作者本人转向 Berkeley Hybrid Robotics（Koushil Sreenath 组）

**项目定位与核心算法**：

legged_control 是 **2023 年整个四足开源社区最重要的作品之一**。它本身不是新算法，而是**"OCS2 + ros-control + WBC + 状态估计 + sim2real"的完整集成**，让学术界实验室和工业界早期创业公司**几小时内就能把 Unitree A1 或类似四足跑起来**。作者自称 "To the author's best knowledge, this framework is probably the best-performing open-source legged robot MPC control framework"——这句话在 2023-2024 年是基本成立的。

技术栈：**OCS2 SQP 求解器** → NMPC（10ms 级求解质心动力学） → 自研 **Whole-Body Controller**（基于 Pinocchio + qpOASES，约束接触力和扭矩） → **线性 Kalman 状态估计**（从 IMU + 关节 + 接触状态估计基座位置速度） → ros_control `RobotHW` → Unitree A1 / Aliengo / 自定义四足。

**C++ 工程亮点**：

1. **"OCS2 生成的 .so 如何在 ros_control 里实时加载"的工程打通**。legged_control 最大的价值是工程打通——OCS2 生成的高度优化 .so 库如何在 `ros_control::RobotHW` 的 1kHz 更新循环里正确调用而不破坏实时性。这涉及线程亲和性（thread affinity）、mlock 防止内存换页、`chrt -f` 的 SCHED_FIFO 实时调度、OCS2 MPC 线程和控制线程之间的 triple buffer 同步。

2. **自研 WBC 相对 TSID 的简化**。作者意识到 TSID 对新手门槛太高，于是**自己实现了一个轻量级 WBC**（`legged_wbc/` 目录）——只做"足端力 + 关节加速度"两层优化，用 qpOASES 直接求解。虽然功能不如 TSID，但读懂 legged_wbc 只需要半天时间，是 WBC 的绝佳入门实现。

3. **线性 KF 状态估计的教科书实现**。文件 `legged_estimation/src/LinearKalmanFilter.cpp` 是"四足如何从 IMU + 关节 + 接触开关估计基座 pose 和 velocity"的最清晰 C++ 实现——过程模型简单（基座匀速 + 噪声）、观测模型用足端接触点在世界系位置不变，Kalman 更新矩阵尺寸都是固定的（18x18）。

4. **gazebo_ros + hardware_interface 双后端**。同一套控制代码可以跑在 Gazebo 仿真和真实 Unitree A1 上——只换一个 `RobotHW` 实现。这是 sim2real 的最简洁框架。

**关键文件路径**：
- `legged_controllers/src/LeggedController.cpp` —— ros_control Controller 主类，包含 1kHz 更新循环
- `legged_wbc/src/WbcBase.cpp`、`HierarchicalWbc.cpp` —— 自研 WBC
- `legged_estimation/src/LinearKalmanFilter.cpp` —— 线性 Kalman 状态估计
- `legged_interface/` —— OCS2 Problem 定义（代价函数 + 约束 + 动力学）
- `legged_gazebo/` —— Gazebo 仿真对接
- `legged_unitree_hw/` —— Unitree 硬件接口

**教学价值评分**：★★★★★（5/5）

**评分理由**：(a) 是"读完一个仓库就能理解整个腿足控制栈"的罕见项目；(b) 代码规模适中（约 2 万行），一周可精读主干；(c) 作者 Qiayuan Liao 自己写了详细 README 解释每一层算法的论文依据；(d) 实战效果——2023 年无数创业公司和硕士论文基于它启动。

**与其他项目的对比位置**：
- **vs MIT Cheetah-Software**：MIT Cheetah 是自研动力学 + 凸 MPC（QP，不是 NMPC），legged_control 是 Pinocchio + OCS2 NMPC——后者算法更先进但计算开销更大
- **vs Quad-SDK**（T1 层）：Quad-SDK 是 CMU 的另一套四足全栈，用 Ipopt + 自研 global planner，更重视地形适应；legged_control 用 OCS2 更快
- **vs ocs2_legged_robot 官方示例**：legged_control 是"在官方示例基础上加了 WBC、状态估计、硬件接口的生产级版本"

---

### T0-5. MIT Cheetah-Software（mit-biomimetics/Cheetah-Software）—— 嵌入式四足控制的经典

**元信息**：
- **GitHub**：https://github.com/mit-biomimetics/Cheetah-Software
- **Stars**：约 3,100+
- **License**：MIT
- **C++ 标准**：C++11/14
- **主要维护者**：MIT Biomimetic Robotics Lab（Sangbae Kim 组），代表性论文作者 Jared Di Carlo、Gerardo Bledt、Benjamin Katz
- **活跃度**：2020 年后归档（MIT Cheetah 3 团队移动到其他项目），但社区仍然非常活跃——它是四足 MPC 教学的"永恒起点"

**项目定位与核心算法**：

MIT Cheetah-Software 是 **"凸 MPC 四足控制" 这一范式的开源原型**。Di Carlo 等人的经典论文 "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"（IROS 2018）提出的方法：**把四足动力学简化为单刚体模型 + 接触力，把摩擦锥线性化，整个 MPC 就变成凸 QP，可以用 qpOASES 在 1ms 内求解**——这直接打开了腿足机器人 "小于 100ms 级别 MPC"的实用门。

技术栈：自研 `FloatingBaseModel<T>` 空间向量代数（Featherstone 算法手实现） + 凸 MPC（400Hz，qpOASES） + 实时 QP 力控（500Hz，基于接触雅可比分配） + 基于 EKF 的状态估计（IMU + 关节 + 足端接触） + LCM 通信（非 ROS！）。

**C++ 工程亮点**：

1. **`FloatingBaseModel<T>` 模板化自实现动力学**——**避开 Pinocchio 的极致主义**。Kim 组的哲学是"控制器代码必须能在嵌入式微处理器上跑"，于是他们不用 Pinocchio（太重），而是基于 Roy Featherstone 的 Rigid Body Dynamics Algorithms 手写了一个极简的空间向量代数库。模板化 `T=float/double` 可以根据精度需求选择。代码在 `common/include/Dynamics/FloatingBaseModel.h`。

2. **`ConvexMPCLocomotion` 的步态设计器**。`user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp` 是**学习"如何把 QP MPC 和步态切换结合"的最清晰 C++ 案例**——整个 400Hz MPC + 步态 FSM（Trot、Bound、Pronk、Walk 等）全在一个文件里，约 500 行。

3. **不依赖 ROS——用 LCM + SharedMemory 通信**。MIT Cheetah 架构刻意**不依赖 ROS**，用 LCM（Lightweight Communications and Marshalling）做跨进程通信、用 POSIX shared memory 做仿真器-控制器之间的零拷贝共享。这是"去 ROS 化"的嵌入式实时机器人架构的标杆。

4. **无堆分配 + 固定大小 Eigen**。整个控制器代码严格使用 `Eigen::Matrix<float, N, M>` 固定大小矩阵——栈分配、编译期循环展开、启动后零堆分配。这是嵌入式实时的绝对要求。

**关键文件路径**：
- `common/include/Dynamics/FloatingBaseModel.h` —— 自研空间向量动力学
- `user/MIT_Controller/Controllers/convexMPC/` —— 凸 MPC 控制器
  - `ConvexMPCLocomotion.cpp` —— 步态 + MPC 主逻辑（核心教学文件）
  - `SolverMPC.cpp` —— QP 问题构造与 qpOASES 调用
- `user/MIT_Controller/FSM_States/` —— 高层 FSM 状态机（站立、行走、平衡等）
- `common/include/Controllers/StateEstimator.h` —— EKF 状态估计
- `lcm-types/` —— LCM 消息定义

**教学价值评分**：★★★★★（5/5）

**评分理由**：(a) 凸 MPC 的开山之作，理解它就理解了现代四足 MPC 的起源；(b) `FloatingBaseModel<T>` 的自实现是学习"如何不用 Pinocchio 自己写空间向量代数"的唯一开源工业级素材；(c) 嵌入式实时架构（无堆分配、LCM、shared memory、模板化 float/double）是教科书；(d) 代码虽然没有类型注解文档，但整体组织清晰——学员读完后会对"腿足机器人要解决哪些工程问题"有全景认知。

**与其他项目的对比位置**：
- **vs legged_control**：MIT Cheetah 是凸 MPC + 自实现动力学（嵌入式路线），legged_control 是 NMPC + Pinocchio（工控机路线）
- **vs A1-QP-MPC-Controller**（T1 层）：ShuoYangRobotics/A1-QP-MPC-Controller 是把 MIT Cheetah 算法移植到 Unitree A1 并加上 ROS 的版本
- **vs OpenLoong**：OpenLoong 的 MPC 算法直接引用 Di Carlo 的论文，本质是 MIT Cheetah 思想在人形的扩展

---

### T0-6. Aligator（Simple-Robotics/aligator）—— 下一代约束轨迹优化

**元信息**：
- **GitHub**：https://github.com/Simple-Robotics/aligator
- **Stars**：约 290+（相对较新，2023 年开始推广）
- **License**：BSD-2
- **C++ 标准**：**C++20**（最前沿）
- **主要维护者**：INRIA Willow team + LAAS Gepetto team，核心作者 Wilson Jallet、Antoine Bambade、Sarah El Kazdadi、Justin Carpentier、Nicolas Mansard
- **活跃度**：极其活跃，T-RO 2025 的 ProxDDP 论文刚发表

**项目定位与核心算法**：

Aligator 是 **Crocoddyl 团队核心成员在 2022+ 年启动的"下一代约束轨迹优化库"**。它的核心算法：**ProxDDP**（Proximal Differential Dynamic Programming）——把 Augmented Lagrangian 方法和 DDP 结合，**原生支持等式和不等式约束**（Crocoddyl 早期对约束的支持比较间接）。T-RO 2025 论文 "PROXDDP: Proximal Constrained Trajectory Optimization" 完整阐述。

Aligator 显式提供**与 Crocoddyl 的兼容接口**——你可以把 Crocoddyl 的 ShootingProblem 直接传给 Aligator 求解，**作为 Crocoddyl 的性能替代后端**。这种"承前启后"的设计让学界很快接受。

**C++ 工程亮点**：

1. **C++20 现代特性的积极应用**。Aligator 要求 C++20，使用 `std::span`、`std::optional`、concepts 等——是 Pinocchio 生态中迈向 C++20 最激进的项目。**学员可以通过 Aligator 看到现代 C++20 在机器人控制代码中的真实应用**。

2. **`ArenaMatrix` 分配器感知的 Eigen 容器**。Aligator 借鉴了 stan-dev/math 的 `arena_matrix` 设计，实现了 `ArenaMatrix` 模板类——一个"allocator-aware 的 Eigen 矩阵"，所有内存从预分配的内存池里取。这在 MPC 连续求解场景中避免了重复 malloc/free，极大提升性能。这是**学习"如何写分配器感知的容器"的前沿案例**。

3. **Parallel Riccati Solver——DDP backward pass 的并行化**。DDP 算法传统上被认为"时间步之间强依赖，无法并行"。Aligator 的 `ParallelRiccatiSolver` 通过一种特殊的分块 Riccati 递推，把 backward pass 并行到多核。这是**2024+ 年腿足 MPC 性能优化的前沿方向**。

4. **Pinocchio 3 深度集成**。Aligator 针对 Pinocchio 3（新版 API）设计，使用 `CRTPBase` 风格更多、约束分类更细（ExplicitIntegrator / ImplicitIntegrator）。

**关键文件路径**：
- `include/aligator/solvers/proxddp/` —— ProxDDP 求解器（核心）
- `include/aligator/solvers/fddp/` —— 兼容 Crocoddyl 的 FeasibleDDP
- `include/aligator/gar/` —— Parallel Riccati（gar = "generalized augmented riccati"）
- `include/aligator/modelling/` —— Problem 建模（Pinocchio 集成）
- `examples/croc-talos-arm.cpp` —— 与 Crocoddyl 对比的 Talos 手臂示例
- `bindings/` —— Python bindings 通过 nanobind

**教学价值评分**：★★★★☆（4.5/5）

**评分理由**：(a) 代表 2024+ 年腿足 MPC 的前沿，是学习**"下一代"技术的窗口**；(b) C++20 实战案例；(c) 相对较新，文档和教程不如 Crocoddyl 完善——扣 0.5 分。

**与其他项目的对比位置**：
- **vs Crocoddyl**：直接后继者，推荐"Crocoddyl 入门 → Aligator 进阶"的学习路径
- **vs OCS2**：OCS2 侧重应用部署（完整 MPC 框架），Aligator 侧重核心算法（约束轨迹优化）——两者实际可以组合使用

---

### T0-7. bipedal_locomotion_framework（ami-iit/bipedal-locomotion-framework）—— iCub 学派的全栈集成

**元信息**：
- **GitHub**：https://github.com/ami-iit/bipedal-locomotion-framework
- **Stars**：约 160+
- **License**：BSD-3
- **C++ 标准**：C++17
- **主要维护者**：IIT（Istituto Italiano di Tecnologia）的 Artificial and Mechanical Intelligence (AMI) 组，核心贡献者 Giulio Romualdi、Stefano Dafarra、Silvio Traversaro 等
- **活跃度**：持续活跃，2026 年仍月级别更新

**项目定位与核心算法**：

bipedal_locomotion_framework（简称 BLF）是 **IIT 为 iCub 人形机器人开发的全栈规控库**。它不是算法库，而是"把一大堆算法打包成可组合模块"的工程框架——**IK、TSID、MPC、步态生成器（UnicyclePlanner）、状态估计、接触检测、YARP middleware、Python bindings** 全部统一在一个 CMake 项目里。

算法覆盖：UnicyclePlanner（独轮车模型步态规划） → Centroidal MPC（质心 MPC） → QPTSID（基于 QP 的 TSID 变体） → IK（Stack-of-Tasks 风格分层 IK） → UKF/EKF 状态估计。核心应用是让 **ergoCub 人形机器人**走路——ami-iit 组的 paper_elobaid_2023_icra_walking_with_payloads 论文是典型应用。

**C++ 工程亮点**：

1. **Component-based 模块化 + CMake 选择性编译**。BLF 把所有功能分成独立 "components"（IK / TSID / MPC / Planners / Estimators / YarpImplementation 等），每个 component 有自己的 `find_package(...)` 依赖，用户可以**只编译需要的部分**。CMake 的 `FRAMEWORK_USE_<dep>:BOOL=OFF` 机制让它成为"按需裁剪"的范例。

2. **基于 `manif` 的李群运算**。BLF 严重依赖 manif 库（另一个 JR-L 学派的开源产物），所有基座姿态用 `manif::SE3` 而非 Eigen Transform——这带来了**雅可比、exp/log、Adjoint 等李群运算的类型安全**。这是学习"李群怎么在现代 C++ 里优雅使用"的绝佳案例。

3. **YARP 中间件集成而非 ROS**。与 ros2_control 不同，BLF 用 YARP（Yet Another Robot Platform）做中间件——iCub 整套生态都是 YARP。这让**学员看到"除了 ROS 还有什么"**。

4. **CentroidalMPC with CasADi**。BLF 的质心 MPC 不用 Pinocchio+CppAD，而是用 **CasADi** 做符号建模 + Ipopt 求解。这是与 OCS2/Crocoddyl 不同的另一条技术路线——**符号框架路线 vs AD+代码生成路线**。

**关键文件路径**：
- `src/ContactModels/` —— 接触模型
- `src/IK/` —— Stack-of-Tasks 风格分层 IK
- `src/TSID/` —— QPTSID 实现（注意这是 BLF 自己的 TSID 实现，不直接依赖 stack-of-tasks/tsid）
- `src/ReducedModelControllers/CentroidalMPC/` —— 质心 MPC（CasADi 实现）
- `src/Planners/UnicyclePlanner/` —— 独轮车模型步态生成器
- `src/Estimators/` —— UKF/EKF 状态估计

**教学价值评分**：★★★★☆（4/5）

**评分理由**：(a) Component-based 模块化 + CMake 选择性编译的工程范例；(b) manif 李群运算的优秀 C++ 示范；(c) CasADi MPC 与 Pinocchio+CppAD MPC 的**对照教材**；(d) 扣分原因：YARP 生态小众，学习投入有迁移成本。

**与其他项目的对比位置**：
- **vs OCS2 + TSID + legged_control** 组合：BLF 是"三合一"打包，但只对 iCub 支持最好
- **vs stack-of-tasks/tsid**：BLF 的 QPTSID 是自己的实现（轻量化），不是 stack-of-tasks/tsid 的封装

---

## 第二部分：T1 分领域项目清单（15 个）

### A. 四足/人形全栈控制框架（5 个）

#### A-1. Quad-SDK（robomechanics/quad-sdk）
- **URL**：https://github.com/robomechanics/quad-sdk
- **Stars**：约 500+；**License**：MIT；**C++ 标准**：C++14/17
- **定位**：CMU Robomechanics Lab 的全栈四足 SDK，侧重**地形适应 + 跳跃**
- **C++ 亮点**：Global Planner（Fast global motion planning for dynamic legged robots）+ Local Planner（基于 **Ipopt** 的 NMPC）+ Robot Driver，ROS1/ROS2 双支持，`LegController` 抽象类允许用户继承开发新控制器，EKF 状态估计（2023 更新）
- **教学评分**：★★★★☆（4/5）
- **典型使用**：CMU Spirit 四足（Ghost Robotics）、Unitree A1/Go1 移植版

#### A-2. OpenLoong Dyn-Control（loongOpen/OpenLoong-Dyn-Control）
- **URL**：https://github.com/loongOpen/OpenLoong-Dyn-Control
- **Stars**：约 700+（中国社区热度高）；**License**：Apache-2.0；**C++ 标准**：C++17
- **定位**：**上海人形机器人创新中心"青龙"机器人** 的全栈 MPC+WBC 控制框架
- **C++ 亮点**：MuJoCo 仿真后端（不是 Gazebo）、Pinocchio 动力学、quill 日志、**自实现凸 MPC（致敬 MIT Cheetah）+ 自实现 WBC**，提供 walking / jumping / blind stepping 三个 demo；代码可读性高、有中文文档和 API 文档
- **教学评分**：★★★★☆（4/5）
- **典型使用**：青龙人形、国内人形 demo 学习

#### A-3. Champ（chvmp/champ）
- **URL**：https://github.com/chvmp/champ
- **Stars**：约 1,400+；**License**：BSD-3；**C++ 标准**：C++14
- **定位**：轻量级四足通用控制框架，支持 MIT Cheetah 的步态库 + OpenQuadruped 方法
- **C++ 亮点**：模块化程度高、有完整的 URDF 生成器（任意四足尺寸参数化），ROS1/ROS2 兼容
- **教学评分**：★★★☆☆（3.5/5，偏入门）
- **典型使用**：业余 DIY 四足（MiniPupper、OpenQuadruped 等）

#### A-4. mrs_uav_system（ctu-mrs/mrs_uav_system）——无人机但架构思想可借鉴
- **URL**：https://github.com/ctu-mrs/mrs_uav_system
- **Stars**：约 400+；**License**：GPL-3；**C++ 标准**：C++17
- **定位**：CTU 布拉格捷克技术大学的多机无人机框架（无人机，不是腿足，但架构思想对腿足极有借鉴价值）
- **C++ 亮点**：极其模块化的 ROS 架构、Docker/Apptainer 部署自动化、pluginlib 控制器管理器
- **教学评分**：★★★★☆（4/5，作为架构参考）

#### A-5. PAL Robotics TALOS Stack（pal-robotics/talos_tutorials 等）
- **URL**：https://github.com/pal-robotics
- **定位**：PAL Robotics 的 TALOS 人形机器人全栈代码（开源部分），依赖 Pinocchio + TSID + ros_control
- **C++ 亮点**：和 Crocoddyl 团队深度合作，TALOS 是 Crocoddyl 最重要的应用平台之一
- **教学评分**：★★★☆☆（3/5，需要有 TALOS 硬件才能完整跑）

### B. 专项算法库（5 个）

#### B-1. TOWR（ethz-adrl/towr）
- **URL**：https://github.com/ethz-adrl/towr
- **Stars**：约 1,000+；**License**：BSD-3；**C++ 标准**：C++11/14
- **定位**：ETH Alexander W. Winkler 的**腿足轨迹优化**开源工具（"Trajectory Optimization for Walking Robots"）
- **C++ 亮点**：基于 **Ifopt**（Ipopt 的现代 C++ 接口）建模，完全脱离 ROS 独立编译，是学习"如何用 Ipopt 做 NLP 建模"的教科书
- **教学评分**：★★★★☆（4/5）
- **典型使用**：Winkler 博士论文、腿足 TO 教学

#### B-2. Ifopt（ethz-adrl/ifopt）
- **URL**：https://github.com/ethz-adrl/ifopt
- **Stars**：约 800+；**License**：BSD-3；**C++ 标准**：C++11
- **定位**：Ipopt 和 SNOPT 的现代 C++ 建模接口——对标 CasADi 但轻量得多
- **C++ 亮点**：`Variables`、`Constraints`、`CostTerms` 基类清晰，代码量小（不到 3000 行），TOWR 的底层
- **教学评分**：★★★★★（5/5，学习 NLP 建模的最佳素材）

#### B-3. ProxSuite / ProxQP（Simple-Robotics/proxsuite）—— 已在第一批列出
- 补充说明：ProxSuite 是 Aligator 和 Crocoddyl 的 QP 后端，在腿足 MPC 中使用极其频繁

#### B-4. HPIPM（giaf/hpipm）
- **URL**：https://github.com/giaf/hpipm
- **Stars**：约 500+；**License**：BSD-2；**C++ 标准**：**C** 为主（有 C++ 绑定）
- **定位**：Gianluca Frison 开发的**结构化稀疏 QP 求解器**，专为 MPC 优化（时间上的稀疏块状结构）
- **C++ 亮点**：BLASFEO（自研 BLAS，针对嵌入式）+ HPIPM，性能极致；OCS2 的 SQP 求解器默认后端
- **教学评分**：★★★★☆（4/5）

#### B-5. acados（acados/acados）
- **URL**：https://github.com/acados/acados
- **Stars**：约 1,400+；**License**：BSD-2；**C++ 标准**：**C** 为主（C++ + Python + MATLAB 接口）
- **定位**：嵌入式 MPC 代码生成器——给定非线性问题，生成可在嵌入式 ARM 上跑的 C 代码
- **C++ 亮点**：代码生成模式（编译期生成），静态内存分配，支持 SQP-RTI 实时迭代
- **教学评分**：★★★★☆（4/5）
- **典型使用**：四足/人形嵌入式 MPC 部署、无人机 MPC

### C. 仿真器和物理引擎（3 个）

#### C-1. MuJoCo（google-deepmind/mujoco）
- **URL**：https://github.com/google-deepmind/mujoco
- **Stars**：约 13,000+；**License**：Apache-2.0（2022 年被 DeepMind 收购并开源）；**C++ 标准**：C11 为主（C++ 绑定）
- **定位**：RL 和腿足研究的**金标准仿真器**，物理精度高、接触模型好
- **C++ 亮点**：凸优化接触求解器（ICC contact）、C 核心极其轻量（可嵌入），OpenLoong 直接用它、legged_control 也支持
- **教学评分**：★★★★★（5/5）

#### C-2. MuJoCo MPC / MJPC（google-deepmind/mujoco_mpc）
- **URL**：https://github.com/google-deepmind/mujoco_mpc
- **Stars**：约 1,700+；**License**：Apache-2.0；**C++ 标准**：C++17
- **定位**：DeepMind 发布的**实时交互式 MPC 框架**——基于 MuJoCo 作动力学后端，内置 iLQG / Gradient Descent / Predictive Sampling 三种求解器
- **C++ 亮点**：GUI 可实时调整代价函数权重观察效果，是 MPC 教学的最佳交互式工具
- **教学评分**：★★★★★（5/5）
- **典型使用**：RL+MPC 研究、MPC 教学

#### C-3. Gazebo Sim / Harmonic（gazebosim/gz-sim）
- **URL**：https://github.com/gazebosim/gz-sim
- **Stars**：约 870+；**License**：Apache-2.0；**C++ 标准**：C++17
- **定位**：Gazebo Classic 的继任者，2025 年 Gazebo Classic 已 EOL
- **C++ 亮点**：ECS（Entity-Component-System）架构，物理引擎可插拔（DART/Bullet/PhysX）
- **教学评分**：★★★★☆（4/5，ROS2 标配）

### D. RL+MPC 混合范式 C++ 实现（2 个）

#### D-1. OCS2 MPC-Net（ocs2/ocs2_mpcnet 子模块）
- **URL**：https://github.com/leggedrobotics/ocs2 内的 `ocs2_mpcnet/`
- **Stars**：包含在 OCS2 主仓库；**License**：BSD-3；**C++ 标准**：C++17
- **定位**：**Farshidian 等人 CoRL 2020 论文 "Deep Value Model Predictive Control" 的 C++ 实现**——用神经网络逼近 MPC 的价值函数，部署时 MPC 用 NN 预测的终值代价作为 terminal cost（缩短 horizon），大幅降低计算量
- **C++ 亮点**：**纯 C++ 的 TorchScript 推理** + OCS2 MPC 深度集成；学到的 NN 部署时作为 `OptimalControlProblem::finalCost`
- **教学评分**：★★★★☆（4/5，稀有的真正 C++ RL+MPC 例子）

#### D-2. silvery107/rl-mpc-locomotion（教学用）
- **URL**：https://github.com/silvery107/rl-mpc-locomotion
- **Stars**：约 500+；**License**：MIT；**C++ 标准**：C++11
- **定位**：把 **MIT Cheetah 凸 MPC 和 Isaac Gym RL 训练结合**——RL 策略学习 MPC 的 cost term 权重，在线自适应不同地形
- **C++ 亮点**：C++ 侧基于 MIT Cheetah，Python 侧用 Isaac Gym；两者通过 gRPC 通信
- **教学评分**：★★★☆☆（3/5，主要价值在架构思想，代码质量一般）

---

## 第三部分：横向分析

### 3.1 四足/人形 C++ 生态的三大关键发现

**发现一：Pinocchio + CppAD + CodeGen 的"预编译 MPC"技术栈已成为腿足性能工程的圣杯**。OCS2、Crocoddyl、Aligator 三大算法库共享同一技术栈：**模板化 Pinocchio 做动力学建模 → CppAD 做自动微分 → CppADCodeGen 生成优化 C 代码 → 编译为 .so → 运行时 dlopen**。这让 7-DOF 机械臂的动力学+碰撞求导从数百微秒压到十几微秒。如果你要做腿足 MPC 的 C++ 工程师，这条流水线**必须掌握**。

**发现二：MPC+WBC 两层架构是腿足控制的事实标准**。高层 MPC（10-100Hz，质心或简化动力学）→ 低层 WBC（500-1000Hz，全身逆动力学 + 分层 QP）。legged_control、MIT Cheetah、OpenLoong、bipedal_locomotion_framework 都采用这个架构。学员理解这个**时间尺度分工**是理解所有腿足代码的钥匙。

**发现三：RL+MPC 的 C++ 工业化还未到来**。目前绝大多数 RL+MPC 项目的 RL 在 Python 侧（PyTorch），C++ 侧只做 MPC；OCS2 MPC-Net 是少数例外（用 TorchScript 做 C++ 推理），但算力开销依然存在。这意味着**"把 RL 策略 TorchScript 化嵌入实时 C++ 控制器"目前是前沿空白**——对你（RL+embodied 背景）来说是个好机会。

### 3.2 四足/人形 vs 机械臂 的 C++ 生态差异

| 维度 | 机械臂（第一批） | 四足/人形（本批） | 分析 |
|------|------------------|-------------------|------|
| **主要应用场景** | 工业装配、抓取、打磨、协作 | Locomotion、平衡、多接触操作 | 根本差异：机械臂**固定基座**，腿足**浮动基座+接触切换** |
| **动力学维度** | 6-7 DOF | 12 DOF（四足） / 28-40 DOF（人形） | 腿足 MPC 维度高出 5-8 倍，对求解器压力大 |
| **控制频率** | 1kHz 实时（libfranka 典型） | 上层 MPC 10-100Hz + 下层 WBC 500-1000Hz 分层 | 腿足是"异步双层"架构，机械臂是"同步单层" |
| **核心库使用** | Pinocchio（可选）、MoveIt2 必选 | Pinocchio 必选、MoveIt2 **几乎不用** | 腿足的 Pinocchio 中心化程度比机械臂还高 |
| **碰撞检测** | FCL 的 mesh 碰撞（精确） | 主要检查足端接触（点接触简化） | 腿足碰撞检测更简化但接触力模型更复杂 |
| **规划器选型** | OMPL 采样规划 + TrajOpt 优化规划 + Ruckig 时间参数化 | NMPC（OCS2/Crocoddyl）主导，OMPL 几乎不用 | 腿足不做"配置空间采样"，直接做轨迹优化 |
| **求解器选型** | OSQP/ProxQP（QP） + Ipopt（NLP） | HPIPM（结构化 QP） + Ipopt（NLP） + 自研 SQP | 腿足重视**结构化稀疏** QP |
| **嵌入式部署** | 不强制（工控机为主） | 强制（板载计算） | 嵌入式 ARM 上跑 MPC 是腿足的日常 |
| **代表项目** | MoveIt2、Drake、Pinocchio、Ruckig | OCS2、Crocoddyl、TSID、legged_control、MIT Cheetah | 不同生态 |

**关键结论**：机械臂和腿足虽然都用 Pinocchio + Eigen + OSQP，但**整体架构、规划范式、控制频率分层都根本不同**——它们是机器人 C++ 生态中的**两个独立世界**。

### 3.3 机械臂+腿足 vs SLAM 生态的技能迁移

| 技能维度 | SLAM（主线 v8 重点） | 机械臂+腿足（本调研） | 迁移难度 |
|----------|---------------------|--------------------|----------|
| **Eigen 基础** | ✓ 核心技能 | ✓ 完全共享 | 零迁移 |
| **CRTP 模板元编程** | ✓ Sophus SO3Base、Eigen 内部 | ✓ Pinocchio JointModelBase | 低 |
| **智能指针 + RAII** | ✓ ORB-SLAM3 MapPoint/KeyFrame | ✓ Drake System 管理 | 零迁移 |
| **Lambda + TBB/OpenMP** | ✓ KISS-ICP 点云并行 | ✓ Crocoddyl OpenMP backward pass | 低 |
| **因子图优化（g2o/GTSAM）** | ✓ ORB-SLAM3 后端 | ✗ 基本不用 | —— |
| **模板标量类型**（AutoDiff） | 部分（Ceres Jet） | ✓ 核心（Pinocchio Scalar） | 中等 |
| **空间向量代数 / 6D 李群** | ✓ Sophus SE3 | ✓ Pinocchio SE3Tpl、manif | 低 |
| **CppAD + CodeGen** | ✗ | ✓ 核心（OCS2、Crocoddyl） | **高**（新知识） |
| **QP / NLP 建模** | ✗ | ✓ 核心（Ifopt、CasADi、HPIPM） | **高**（新知识） |
| **实时控制 1kHz 循环** | ✗ | ✓ 核心（libfranka、legged_control） | **高**（新知识） |
| **pluginlib 插件架构** | 部分（ROS2） | ✓ 深度（MoveIt2、Nav2、ros2_control） | 中等 |
| **行为树（BT.CPP）** | ✗ | ✓（MoveIt Pro） | 中等 |

**关键结论**：SLAM → 机械臂+腿足 的**迁移成本约 50-60%**——基础 C++ 技能（Eigen、模板、智能指针）完全共享，但**CppAD + CodeGen、QP/NLP 建模、实时控制** 这三个是全新领域。

> **注**：本调研原文第四部分包含一份 10 章增量式教学大纲（足式/30_Pinocchio深度精读-56），与本合并文档中《面向机械臂+腿足规控工程师的C++中高级进阶增量教学大纲（v1·扩展篇）》内容重复，已在合并时移除。完整大纲请参见本文档对应章节。

### 增量大纲总结

这 10 章（约 12 周）的增量内容，和主线 v8 的 46 章（48 周）加起来，构成一个**约 60 周的完整机器人 C++ 工程师培养路径**，横跨 SLAM + 机械臂 + 腿足三大方向。

**关键节点回顾**：
- **Week 1-12**（主线 v8 Ch1-11）：C++ 基础强化
- **Week 12-30**（主线 v8 Ch12-28）：模板、并发、SLAM 库（Eigen/GTSAM/Ceres）
- **Week 30-48**（主线 v8 Ch29-46）：架构、工程、CUDA、SLAM 精读、Mini-LIO 实战
- **Week 49-60**（本增量大纲）：**机械臂+腿足方向**——Pinocchio、CppAD、QP/NLP、WBC、MPC、实时 C++、BT.CPP、GPU 规控、综合项目


## 附录：本批次项目优先级总表（22 个，按教学价值 ★ 排序）

| 优先级 | 项目 | Stars | 核心学习价值 | 建议投入 |
|--------|------|-------|------------|---------|
| ★★★★★ | **OCS2** | 1,100+ | Pinocchio+CppAD+CodeGen 全流水线 + MPC 双线程实时架构 | 2 周 |
| ★★★★★ | **Crocoddyl** | 970+ | DDP 家族算法 + Virtualization + Contact Dynamics ActionModel | 1.5 周 |
| ★★★★★ | **TSID** | 360+ | WBC 分层 QP + EIGEN_RUNTIME_NO_MALLOC 实时实践 | 1 周 |
| ★★★★★ | **legged_control** | 900+ | OCS2 + WBC + 硬件对接完整栈，sim2real 最简路径 | 1.5 周 |
| ★★★★★ | **MIT Cheetah-Software** | 3,100+ | 凸 MPC 开山 + 嵌入式自实现动力学 + LCM/SharedMemory | 1 周 |
| ★★★★★ | **MuJoCo MPC (MJPC)** | 1,700+ | 交互式 MPC 教学神器 + iLQG/PredictiveSampling | 3-5 天 |
| ★★★★★ | **Ifopt** | 800+ | NLP 建模的最轻量 C++ 接口，学习 Ipopt 集成 | 3 天 |
| ★★★★★ | **MuJoCo** | 13,000+ | 物理仿真金标准，所有腿足 RL/MPC 仿真后端 | 1 周 |
| ★★★★☆ | **Aligator** | 290+ | ProxDDP 前沿算法 + C++20 实战 | 5 天 |
| ★★★★☆ | **bipedal_locomotion_framework** | 160+ | Component-based 模块化 + CasADi MPC 路线 | 1 周 |
| ★★★★☆ | **Quad-SDK** | 500+ | CMU 四足全栈，Ipopt NMPC 实现 | 5 天 |
| ★★★★☆ | **OpenLoong-Dyn-Control** | 700+ | 中国人形开源代表，MPC+WBC 国产实现 | 5 天 |
| ★★★★☆ | **TOWR** | 1,000+ | 腿足轨迹优化教学 + Ifopt 实战 | 3-5 天 |
| ★★★★☆ | **HPIPM** | 500+ | 结构化稀疏 QP 求解器，OCS2 后端 | 3 天 |
| ★★★★☆ | **acados** | 1,400+ | 嵌入式 MPC 代码生成器 | 3-5 天 |
| ★★★★☆ | **OCS2 MPC-Net** | 含在主库 | 真正 C++ 实现的 RL+MPC 范例 | 5 天 |
| ★★★★☆ | **Gazebo Sim** | 870+ | ROS2 标配仿真器 | 3 天 |
| ★★★☆☆ | **Champ** | 1,400+ | 入门级四足框架 | 2 天 |
| ★★★☆☆ | **silvery107/rl-mpc-locomotion** | 500+ | RL+MPC 架构思想（代码质量一般） | 2 天 |
| ★★★☆☆ | **PAL Robotics TALOS stack** | —— | Crocoddyl 的硬件应用（需硬件） | 3 天 |
| ★★★☆☆ | **mrs_uav_system** | 400+ | 多机协同架构参考（跨领域） | 2 天 |
| ★★★☆☆ | **A1-QP-MPC-Controller** | —— | MIT Cheetah 在 Unitree A1 的移植 | 2 天 |

## 全部两批调研汇总

结合第一批（机械臂，38 个项目）和第二批（四足/人形，22 个项目），一共覆盖 **60 个机器人规控 C++ 项目**。增量式教学大纲（足式/30_Pinocchio深度精读-56）的完整内容请参见本合并文档中《面向机械臂+腿足规控工程师的C++中高级进阶增量教学大纲（v1·扩展篇）》。

