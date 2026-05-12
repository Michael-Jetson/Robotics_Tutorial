> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

## 第 55 章:OCS2 完整栈 + 双线程 MPC 实时架构

> **难度**: ⭐⭐⭐ | **预计学时**: 40-50 小时(2 周) | **前置**: Ch47-50, Ch52-54

> **一句话概要**: OCS2 是 ETH RSL 打造的工业级 MPC 基础设施——把"切换系统最优控制"抽象为一等公民,用 SQP+HPIPM 求解,以双线程 Triple Buffer 架构保证实时性,是 ANYmal/Go2/HyQ 的生产控制栈。

---

## 前置自测

📋 前置自测（答不出 ≥ 2 题 → 先回 Ch50/Ch54 复习）

1. DDP 的 backward pass 计算什么？forward pass 做什么？
2. SQP 与 DDP 的核心区别是什么？各自的优势场景？
3. 什么是"切换系统(switched system)"？腿足机器人为什么是切换系统？
4. 实时控制循环要求 1 kHz 更新，但 MPC 求解需要 20 ms——如何解决这个矛盾？
5. ROS2 的 lifecycle node 有哪些状态？`on_activate` 和 `on_configure` 的区别？

### 本章目标

学完本章,学员应能:

1. **理解 OCS2 作为"工业级腿足 MPC 基础设施"的定位**——ANYmal / Go2 / HyQ 生产栈
2. **掌握 OCS2 的五层抽象**:RolloutBase -> OCP -> Solver -> MPC_Interface -> ROS Wrapper
3. **读懂 SQP 算法和 HPIPM 集成**——包括 SQP-RTI(Real-Time Iteration)为何只需 1 次 SQP 迭代
4. **精通双线程 MPC 架构**:MPC_Node(异步求解) + MRT_Node(实时查询) + Triple Buffer 无锁通信
5. **能为新四足机器人配置 OCS2 `legged_robot` 示例**——URDF + frame_declaration + task.info
6. **能做 Centroidal Model vs Kino-centroidal 模型的对比选择**
7. **能用 legged_control 框架在仿真和实物上部署 OCS2 MPC**

### 前置依赖

```
本章定位(依赖图):

Ch47 Pinocchio ──┐
Ch48 CppAD/CG ──┤
Ch49 Centroidal ──┼──> [Ch55 OCS2 完整栈]──> Ch56 步态管理
Ch50 HPIPM/QP ──┤                        ├──> Ch67 Perceptive MPC
Ch52 接触约束 ──┤                        ├──> Ch68 legged_control 精读
Ch54 DDP/Crocoddyl┘                      └──> Ch73 mobile_manipulator

v8 主线:
Ch17-20 并发 ──> 双线程 MPC 架构的基础
Ch29 设计模式 ──> OCS2 大量用 Strategy/Factory/Facade
Ch30 ROS2 ──────> MPC ROS Wrapper 是 Lifecycle Node
```

---

### 55.1 OCS2 的定位——ETH RSL 的 MPC 基础设施 ⭐

#### 55.1.1 动机:为什么需要一个"MPC 框架"?

> **自检问题**: 如果让你从零为四足机器人写一个 MPC 控制器,你需要实现哪些组件?

答案可能包括:动力学模型、代价函数、约束、QP 求解器、线性化、积分器、参考轨迹管理、步态调度、线程管理、ROS 封装......**每一个都是几千行代码**。一个个人或团队从头写,至少需要 1-2 年。

```
                    "从头写 MPC" 的痛苦分解
┌──────────────────────────────────────────────────────┐
│ 层级              工作量          可复用性             │
├──────────────────────────────────────────────────────┤
│ 动力学建模         高(Pinocchio)   高                │
│ 自动微分           中(CppAD)      高                │
│ QP 求解器          极高(HPIPM)    高                │
│ SQP/DDP 框架       高              中(问题相关)      │
│ 步态管理           中              低(腿足专用)      │
│ 双线程实时架构     高              中                 │
│ ROS 封装          中               高                │
│ 调参/调试工具      中               低                │
├──────────────────────────────────────────────────────┤
│ 总计:            ~5万行 C++       部分可复用         │
└──────────────────────────────────────────────────────┘
```

**OCS2 的价值**: 把上面这些"轮子"全部集成为一个**可插拔、可扩展、已验证的框架**。用户只需定义自己机器人的 OCP(动力学+代价+约束),其余全由框架处理。这就像游戏引擎(Unity/Unreal)之于游戏开发:引擎把渲染、物理、音频、网络等底层模块全部封装好,游戏开发者只需定义自己的游戏逻辑(角色、关卡、规则)。OCS2 对 MPC 的角色与之类似于——用户定义"我的机器人长什么样、要优化什么目标",框架负责"怎么高效求解、怎么实时部署"。

> **⚠️ 陷阱**: OCS2 的"价值"也是它的"代价"——抽象层次多,学习曲线陡。初学者容易迷失在继承层次中。本章的目标就是帮你建立清晰的心智模型。

#### 55.1.2 OCS2 的名字与历史

**OCS2** = **O**ptimal **C**ontrol for **S**witched **S**ystems.

为什么叫"Switched Systems(切换系统)"? 因为腿足运动的本质是**接触模式切换**:

```
步态 = 接触模式的时间序列

trot 步态:
 时间 ──────────────────────────────────────────>
 FR+RL: ████░░░░████░░░░████░░░░   (█=触地, ░=摆动)
 FL+RR: ░░░░████░░░░████░░░░████
        ^    ^    ^    ^    ^
        t0   t1   t2   t3   t4     (模式切换时刻)

每次 █↔░ 切换 = 系统的动力学方程改变
  - 触地腿: 零速度约束 + 可施加接触力
  - 摆动腿: 零力约束 + 自由运动

这就是"切换系统"!
```

OCS2 把这种"模式切换"抽象为框架的**一等公民**:
- `ModeSchedule`: 记录哪些时刻发生模式切换
- `jumpMap()`: 模式切换瞬间状态可能不连续(比如碰撞)
- 约束按 mode 启用/禁用: 触地腿有力约束,摆动腿没有

**历史脉络**:

```
2013-2016: ETH ADRL (Agile & Dexterous Robotics Lab)
           Farshidian 博士开始做 MPC for ANYmal
           早期用 Augmented Lagrangian Method (ALM)

2017:      ICRA 论文 "An efficient optimal planning and control
           framework for quadrupedal locomotion"
           ——OCS2 的前身论文,用 SLQ (Sequential Linear Quadratic)

2019:      切换到 SQP + HPIPM 作为主推求解器
           RA-L: "Frequency-aware MPC" (Grandia et al.)

2020-2021: 开源,加入 mobile_manipulator 示例
           RA-L: "Unified MPC for loco-manipulation" (Sleiman)

2022-2023: Perceptive Locomotion (T-RO, Grandia et al.)
           MPC + 感知集成,100Hz NMPC on ANYmal

2024-:     社区维护,legged_control (qiayuan) 广泛使用
           GPU 加速和 RL 混合是前沿方向
```

> **💡 洞察**: OCS2 的演化路线是 **ALM -> SLQ/DDP -> SQP+HPIPM**。选择 SQP 不是因为它"更好",而是因为腿足场景有大量硬约束(摩擦锥、关节限位),SQP 能直接处理,DDP 需要用罚函数近似。这是**需求驱动的技术选择**。

#### 55.1.3 四大 MPC 框架对比

| 维度 | **OCS2** | **Crocoddyl** | **acados** | **CasADi+IPOPT** |
|------|----------|---------------|------------|-------------------|
| **出品方** | ETH RSL | LAAS-CNRS | Uni. Freiburg | KU Leuven |
| **主攻场景** | 腿足+切换系统 | 机械臂+研究原型 | 嵌入式 MPC | 通用 NLP |
| **求解算法** | SQP(主推)+DDP | DDP/FDDP | SQP-RTI+HPIPM | IPOPT(内点法) |
| **约束处理** | 硬约束(QP 直接) | 软约束(ALM/ProxDDP) | 硬约束(QP) | 硬约束(NLP) |
| **切换系统** | **一等公民** | 需自己管理 | 需自己管理 | 需自己管理 |
| **自动微分** | CppAD/CppADCodeGen | Pinocchio analytical | CasADi | CasADi |
| **ROS 集成** | 完整 MPC/MRT Node | 简单 | 较弱 | 无 |
| **实时性设计** | Triple Buffer 双线程 | 无特殊设计 | RTI 内置 | 无 |
| **代码生成** | CppADCodeGen -> .so | 无 | CasADi -> C | CasADi -> C |
| **典型用户** | ANYmal, Go2, HyQ | Solo, Talos | 无人机, 汽车 | 学术原型 |
| **学习曲线** | 陡(抽象多) | 平缓 | 中等 | 低(Python) |
| **语言** | C++ | C++/Python | C/Python/MATLAB | Python/MATLAB |

```
选型决策树:

你要做什么?
├── 四足/腿足 MPC 生产部署 ──> OCS2 (推荐) 或 acados
├── 机械臂轨迹优化/研究 ──> Crocoddyl 或 CasADi
├── 无人机/汽车嵌入式 MPC ──> acados
├── 快速原型验证 ──> CasADi + IPOPT (Python)
└── 学术 NLP 研究 ──> CasADi 或 自写
```

> **🧠 深度理解**: acados 和 OCS2 的后端都是 HPIPM——它们在 QP 层是等价的。差异在上层抽象: OCS2 专注切换系统和 ROS 集成,acados 专注嵌入式部署和代码生成。如果你需要在 ARM 嵌入式平台上跑纯 C 代码,acados 更合适;如果你需要 ROS 生态和步态管理,OCS2 更合适。

**练习 55.1a**: 你的课题组已有 Crocoddyl 的使用经验,现在要给 Unitree Go2 做生产级 MPC。列出三条"为什么应该切换到 OCS2"的理由和一条"坚持用 Crocoddyl"的理由。

**练习 55.1b**: 如果要在 Jetson Orin NX (ARM) 上部署 MPC,OCS2 和 acados 哪个更适合?从编译工具链、实时性、代码生成三个维度对比。

#### 55.1.4 谁在用 OCS2?

| 项目/机器人 | 场景 | 参考 |
|------------|------|------|
| **ANYmal** (ETH RSL) | 全地形四足,感知+MPC | Grandia et al. T-RO 2023 |
| **Unitree Go1/Go2** (legged_control) | 开源四足 MPC+WBC | qiayuan/legged_control |
| **HyQReal** (IIT) | 液压四足 | Sleiman et al. RA-L 2021 |
| **Tachyon 3** (轮足) | 轮腿复合机器人 | Full-Centroidal NMPC |
| **Mobile Manipulator** (OCS2 示例) | 底盘+机械臂(Ch73) | ocs2_mobile_manipulator |
| **Ballbot** (OCS2 示例) | 平衡球机器人 | ocs2_ballbot |

> **自检**: 为什么以上项目都选择 OCS2 而非 Crocoddyl? 关键词: **硬约束处理** + **切换系统抽象** + **实时双线程架构**。

#### 55.1.5 线性 MPC 的预测方程基础——从离散模型到 QP

在深入 OCS2 的非线性 SQP 框架之前，有必要理解线性 MPC 的预测方程构造方法——这既是理解 SQP 中 QP 子问题的基础，也是 MIT Cheetah 凸 MPC（Ch51 SRBD）的直接实现思路。

**预测方程的递推构造**

给定离散线性系统 $\mathbf{x}_{k+1} = A\mathbf{x}_k + B\mathbf{u}_k$，$\mathbf{z}_k = C\mathbf{x}_k$，从时刻 $k$ 的状态 $\mathbf{x}_k$ 出发，逐步展开未来状态：

$$\mathbf{x}_{k+1} = A\mathbf{x}_k + B\mathbf{u}_k$$
$$\mathbf{x}_{k+2} = A^2\mathbf{x}_k + AB\mathbf{u}_k + B\mathbf{u}_{k+1}$$
$$\mathbf{x}_{k+v} = A^v\mathbf{x}_k + \sum_{j=0}^{v-1} A^{v-1-j}B\mathbf{u}_{k+j}$$

将预测时域（prediction horizon $f$）内的所有输出堆叠为向量，得到矩阵形式：

$$\underset{\bar{\mathbf{z}}}{\underbrace{\begin{pmatrix} \mathbf{z}_{k+1} \\ \mathbf{z}_{k+2} \\ \vdots \\ \mathbf{z}_{k+f} \end{pmatrix}}} = \underset{O}{\underbrace{\begin{pmatrix} CA \\ CA^2 \\ \vdots \\ CA^f \end{pmatrix}}} \mathbf{x}_k + \underset{M}{\underbrace{\begin{pmatrix} CB & 0 & \cdots & 0 \\ CAB & CB & \cdots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ CA^{f-1}B & CA^{f-2}B & \cdots & CB \end{pmatrix}}} \underset{\bar{\mathbf{u}}}{\underbrace{\begin{pmatrix} \mathbf{u}_k \\ \mathbf{u}_{k+1} \\ \vdots \\ \mathbf{u}_{k+f-1} \end{pmatrix}}}$$

即 $\bar{\mathbf{z}} = O\mathbf{x}_k + M\bar{\mathbf{u}}$，其中 $O$ 是可观性矩阵的变体，$M$ 是 Toeplitz 结构的控制-输出映射矩阵。

**代价函数的 QP 化**

定义跟踪误差代价和控制惩罚代价：

$$J = \underbrace{(\bar{\mathbf{z}}^d - \bar{\mathbf{z}})^T P (\bar{\mathbf{z}}^d - \bar{\mathbf{z}})}_{\text{跟踪误差}} + \underbrace{\bar{\mathbf{u}}^T W \bar{\mathbf{u}}}_{\text{控制惩罚}}$$

其中 $P$ 是跟踪权重矩阵（可为不同时刻设置不同权重），$W$ 可包含控制幅度和控制增量的惩罚。将系统方程代入，令 $\mathbf{s} = \bar{\mathbf{z}}^d - O\mathbf{x}_k$（已知量），代价函数化为关于 $\bar{\mathbf{u}}$ 的二次型：

$$J = (\mathbf{s} - M\bar{\mathbf{u}})^T P (\mathbf{s} - M\bar{\mathbf{u}}) + \bar{\mathbf{u}}^T W \bar{\mathbf{u}}$$

对 $\bar{\mathbf{u}}$ 求导令其为零，无约束最优解为：

$$\hat{\bar{\mathbf{u}}} = (M^T P M + W)^{-1} M^T P \mathbf{s}$$

加入不等式约束（如摩擦锥、力矩限幅）后，问题变为标准 QP，需用 QP 求解器（qpOASES、OSQP 等）数值求解。

> **从线性 MPC 到 OCS2 的跨越**：上述推导假设系统是 LTI（线性时不变）。但腿足机器人的 SRBD 模型是 LTV（$A_k$, $B_k$ 随时间变化），而 Centroidal 模型是完全非线性的。OCS2 使用 SQP 在每次迭代中将非线性问题线性化为 QP 子问题——该 QP 子问题的结构与上述预测方程完全同构，只是 $A$, $B$, $C$ 被替换为当前迭代点的线性化矩阵，且由 HPIPM 利用带状稀疏结构高效求解。理解了线性 MPC 的矩阵堆叠方法，就抓住了 SQP-MPC 的数学骨架。

---

### 55.2 OCS2 的五层架构 ⭐⭐

OCS2 的定位明确之后,下一个问题是:如此庞大的系统如何组织代码?OCS2 的答案是五层解耦架构——每层只依赖下层接口,上层可独立替换。

#### 55.2.1 架构全景图

```
┌─────────────────────────────────────────────────────────────────────┐
│ 第五层: ocs2_legged_robot_ros / ocs2_mobile_manipulator_ros        │
│ ┌─────────────────────┐ ┌──────────────┐ ┌───────────────────┐     │
│ │ LeggedRobotMpcNode  │ │ RViz 可视化   │ │ Launch / Config  │     │
│ └─────────┬───────────┘ └──────────────┘ └───────────────────┘     │
│           │ ROS2 话题/服务                                         │
├───────────▼─────────────────────────────────────────────────────────┤
│ 第四层: ocs2_mpc / ocs2_ros_interfaces                             │
│ ┌──────────────────────────┐  ┌──────────────────────────────────┐ │
│ │ MPC_MRT_Interface        │  │ MPC_ROS_Interface                │ │
│ │ (同进程双线程)           │  │ MRT_ROS_Interface                │ │
│ │ ┌──────┐ ┌──────────┐   │  │ (跨进程 ROS 通信)               │ │
│ │ │Buffer│ │PolicyData│   │  │                                  │ │
│ │ └──────┘ └──────────┘   │  └──────────────────────────────────┘ │
│ └──────────┬───────────────┘                                      │
├────────────▼────────────────────────────────────────────────────────┤
│ 第三层: ocs2_sqp / ocs2_ddp / ocs2_ipm                            │
│ ┌────────────┐  ┌────────────┐  ┌───────────┐  ┌────────────────┐ │
│ │ SqpSolver  │  │ SlqSolver  │  │ IpmSolver │  │ HpipmInterface │ │
│ │ (主推)     │  │            │  │           │  │ (QP 后端)      │ │
│ └─────┬──────┘  └────────────┘  └───────────┘  └────────────────┘ │
├───────▼─────────────────────────────────────────────────────────────┤
│ 第二层: ocs2_oc (Optimal Control Problem)                          │
│ ┌─────────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│ │SystemDynamicsBase│  │StateCost     │  │StateInputConstraint │   │
│ │ flowMap()        │  │StateInputCost│  │StateConstraint      │   │
│ │ jumpMap()        │  │              │  │                      │   │
│ └─────────────────┘  └──────────────┘  └──────────────────────┘   │
│  OptimalControlProblem struct (聚合以上组件)                        │
├─────────────────────────────────────────────────────────────────────┤
│ 第一层: ocs2_core / ocs2_robotic_tools                             │
│ ┌─────────────┐  ┌─────────────────────────┐  ┌────────────────┐  │
│ │ RolloutBase  │  │ ReferenceManagerInterface│  │ Types / Utils  │  │
│ │ (积分器)     │  │ ModeSchedule             │  │ CppAdInterface │  │
│ └─────────────┘  └─────────────────────────┘  └────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

#### 55.2.2 数据流:从用户命令到关节力矩

理解 OCS2 最好的方式是跟踪一个**完整的数据流**:

```
用户命令 (velocity cmd)
    │
    ▼
ReferenceManager: 生成参考轨迹 x_ref(t), u_ref(t)
    │              生成 ModeSchedule (步态切换时刻)
    ▼
OptimalControlProblem: 定义 OCP
    │   ├── SystemDynamics: Centroidal 方程 (调用 Pinocchio)
    │   ├── Cost: 跟踪代价 ||x - x_ref||^2_Q + ||u - u_ref||^2_R
    │   ├── EqualityConstraint: 触地脚零速度
    │   ├── InequalityConstraint: 摩擦锥, 关节限位
    │   └── ModeSchedule: 哪些脚触地,哪些摆动
    ▼
SqpSolver: 求解 NLP
    │   ├── 线性化 OCP -> 时变 (A_k, B_k, Q_k, R_k, C_k, D_k)
    │   ├── 构建 block-sparse QP
    │   ├── 调用 HPIPM 求解 QP -> delta_x, delta_u
    │   ├── Line search (Armijo 条件)
    │   └── 收敛判定: ||delta||< tol 或达到 max_iter
    ▼
PolicyData: MPC 输出
    │   ├── timeTrajectory: [t0, t1, ..., tN]
    │   ├── stateTrajectory: [x0, x1, ..., xN]
    │   ├── inputTrajectory: [u0, u1, ..., uN]
    │   └── controllerTrajectory: K(t), k(t) (反馈增益)
    ▼
MPC_MRT_Interface (Triple Buffer): 无锁传递
    │
    ▼
MRT: 按当前时间 t 插值查询
    │   ├── x_des = interpolate(stateTrajectory, t)
    │   ├── u_ff  = interpolate(inputTrajectory, t)
    │   └── u_fb  = K(t) * (x_meas - x_des)   (可选反馈)
    ▼
WBC (Ch53) 或 PD 控制器: 跟踪 MPC 输出
    │
    ▼
关节力矩 tau -> 电机驱动器
```

> **💡 洞察**: 注意数据流中**没有一步涉及"手动调 PID"**——MPC 直接输出最优前馈力和反馈增益。WBC 负责把 Centroidal 层面的力分配到关节。这就是"规划即控制"的思想。

> 💡 **为什么 WBC 不够——MPC 的动机**
>
> WBC 基于浮动基逆动力学，考虑了完整的机器人动力学模型，但它只求解**当前瞬时**的最优关节力矩。这意味着 WBC 无法为未来做准备：在支撑相的后半段，WBC 不会提前为即将到来的飞行相调整力分配；在飞行相后半程，WBC 也不会为着地冲击做预准备。MPC 通过在时间轴上前瞻，弥补了这一"短视"缺陷。

#### 55.2.3 各层关键文件与类

| 层级 | 关键文件 | 核心类 | 用户需要关心? |
|------|---------|--------|--------------|
| 第一层 | `ocs2_core/include/ocs2_core/Types.h` | `scalar_t`, `vector_t`, `matrix_t` | 是(类型基础) |
| 第一层 | `ocs2_core/include/ocs2_core/reference/` | `ReferenceManagerInterface` | 是(参考轨迹) |
| 第一层 | `ocs2_core/include/ocs2_core/integration/` | `RolloutBase`, `ODE45` | 否(框架内部) |
| 第二层 | `ocs2_oc/include/ocs2_oc/oc_problem/` | `OptimalControlProblem` | **核心**(定义 OCP) |
| 第二层 | `ocs2_oc/include/ocs2_oc/dynamics/` | `SystemDynamicsBase` | 是(继承实现) |
| 第三层 | `ocs2_sqp/src/SqpSolver.cpp` | `SqpSolver` | 了解(读源码) |
| 第三层 | `ocs2_sqp/include/ocs2_sqp/HpipmInterface.h` | `HpipmInterface` | 了解 |
| 第四层 | `ocs2_mpc/include/ocs2_mpc/MPC_MRT_Interface.h` | `MPC_MRT_Interface` | **核心**(部署) |
| 第五层 | `ocs2_legged_robot_ros/src/LeggedRobotMpcNode.cpp` | ROS Node | 是(启动入口) |

> **⚠️ 陷阱**: 初学者常犯的错误是试图"从第三层开始读"——直接看 SqpSolver 源码。正确的学习顺序是 **第二层(定义问题) -> 第四层(部署架构) -> 第三层(求解细节)**。先知道"OCS2 解什么问题",再知道"怎么部署",最后才看"怎么解"。

#### 55.2.4 与 nav2 架构的类比

ROS2 开发者可能熟悉 nav2(导航栈)。OCS2 的分层思想与 nav2 有相似之处:

```
nav2 架构                      OCS2 架构
─────────                      ─────────
Costmap2D(地图层)         <->  ReferenceManager(参考层)
Planner(全局规划)          <->  MPC Solver(最优控制)
Controller(局部跟踪)       <->  MRT + WBC(实时跟踪)
BehaviorTree(任务调度)     <->  GaitSchedule(步态调度)
Lifecycle Node(生命周期)    <->  MPC_ROS_Interface(ROS 封装)
```

**关键差异**: nav2 的 Planner 输出路径(几何),OCS2 的 Solver 输出轨迹(状态+控制+时间)——**多了"力"和"反馈增益"信息**,这是 MPC 比路径规划更强大的地方。类似于导航软件只告诉你"往哪走"(路径),而 MPC 相当于一位坐在副驾驶的教练,不仅告诉你路线,还告诉你每个弯道该踩多少油门、打多少方向盘(控制输入),以及遇到偏离时如何修正(反馈增益)。

#### 55.2.5 如何扩展每一层

用户为自己的机器人适配 OCS2 时,各层的工作量不同:

```
层级      需要做什么                         工作量
──────    ─────────────────────────          ──────
第一层    通常不动(复用 RolloutBase 等)       无
第二层    ★ 实现自己的 Dynamics/Cost/Constraint  高 (核心工作)
第三层    选择求解器(SQP vs DDP),调参数        低
第四层    选择部署模式(同进程 vs 跨进程)        低
第五层    写 launch 文件、配置 RViz              中
```

**练习 55.2a**: 画出 OCS2 五层架构中你"需要自己写代码"的部分和"直接复用"的部分。用红色标记需要写的,绿色标记复用的。

**练习 55.2b**: 如果你要把 OCS2 从四足扩展到轮足机器人(如 Tachyon 3),哪些层需要修改? 提示:轮子没有摆动腿,接触模式不同。

---

### 55.3 OCP 抽象——OptimalControlProblem ⭐⭐

#### 55.3.1 动机:为什么需要 OCP 抽象?

> **反面案例**: 如果不做抽象,每换一个机器人就要重写求解器接口。看过 Drake 的人知道——Drake 把问题和求解器分得很开,OCS2 也是。

OCS2 的核心设计理念: **用户定义 OptimalControlProblem,框架负责求解**。这一层是用户和框架之间的"契约"。

#### 55.3.2 OptimalControlProblem 完整结构

```cpp
// ocs2_oc/include/ocs2_oc/oc_problem/OptimalControlProblem.h
struct OptimalControlProblem {
  // === 动力学 ===
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  // flowMap(): dx/dt = f(x, u, t)  -- 连续相动力学
  // jumpMap(): x+ = g(x-, t)       -- 模式切换时的状态跳跃

  // === 运行代价 (intermediate) ===
  StateInputCostCollection cost;              // L(x, u, t)
  StateInputCostCollection softConstraint;    // 软约束 -> 代价

  // === 终端代价 ===
  StateCostCollection finalCost;              // Phi(x_T)
  StateCostCollection finalSoftConstraint;

  // === 跳跃代价 (prejump) ===
  StateCostCollection preJumpCost;
  StateCostCollection preJumpSoftConstraint;

  // === 等式约束 ===
  StateInputEqualityConstraintCollection equalityConstraint;
  // g(x, u, t) = 0
  // 例: 触地脚末端速度为零

  // === 不等式约束 ===
  StateInputInequalityConstraintCollection inequalityConstraint;
  // h(x, u, t) >= 0
  // 例: 摩擦锥 mu*Fn >= sqrt(Fx^2 + Fy^2)

  // === 等式 Lagrangian ===
  StateInputEqualityConstraintCollection equalityLagrangian;
  StateInputInequalityConstraintCollection inequalityLagrangian;

  // === 参考管理 ===
  std::shared_ptr<ReferenceManagerInterface> targetTrajectoriesPtr;
  ModeScheduleManagerPtr modeScheduleManagerPtr;
};
```

> **版本提示**：以上结构基于 OCS2 早期 API。字段名可能随版本变化，请以 `ocs2_oc/include/ocs2_oc/oc_problem/OptimalControlProblem.h` 为准。

**容器 + 命名查找** 模式:

```cpp
// Collection 内部是 std::map<std::string, std::unique_ptr<...>>
// 添加:
problem.costPtr->add("base_tracking",
    std::make_unique<BaseTrackingCost>(Q, x_ref));
problem.softConstraintPtr->add("friction_cone",
    std::make_unique<FrictionConeConstraint>(mu));

// 删除(动态移除约束):
problem.costPtr->erase("base_tracking");

// 查询:
auto& cost = problem.cost.get("base_tracking");
```

> **💡 为什么用 string 做 key?** 灵活性。用户可以在运行时动态添加/删除代价项。比如"地形感知模式"下加入"落脚点代价","平地模式"下移除它。代价是一次 `std::map::find`(O(log N),N 通常 < 10),对 MPC 求解时间(ms 级)来说可忽略。

> **⚠️ 陷阱**: 不要在 MPC 循环中频繁调用 `add()`/`erase()`——这涉及堆分配。正确做法是用 `isActive()` 方法动态启用/禁用已注册的约束。

#### 55.3.3 SystemDynamicsBase: flowMap() 和 jumpMap()

```cpp
class SystemDynamicsBase {
public:
  // 连续相动力学: dx/dt = flowMap(x, u, t)
  virtual vector_t flowMap(scalar_t t,
                           const vector_t& x,
                           const vector_t& u) = 0;

  // 模式切换时的状态跳跃: x_plus = jumpMap(t, x_minus)
  virtual vector_t jumpMap(scalar_t t,
                           const vector_t& x) {
    return x;  // 默认: 无跳跃 (连续)
  }

  // 线性化: 提供 A = df/dx, B = df/du
  virtual VectorFunctionLinearApproximation
  linearApproximation(scalar_t t,
                      const vector_t& x,
                      const vector_t& u) = 0;
};
```

**对腿足场景**:
- `flowMap()` 实现 Centroidal 动力学(Ch49):

$$\dot{\boldsymbol{h}}_G = \sum_{i \in \mathcal{C}} \begin{bmatrix} \boldsymbol{\lambda}_i \\ \boldsymbol{r}_i \times \boldsymbol{\lambda}_i \end{bmatrix} + \begin{bmatrix} m\boldsymbol{g} \\ \boldsymbol{0} \end{bmatrix}$$

- `jumpMap()` 默认恒等(常见 OCS2 腿足 MPC 实现中忽略或软化冲击效应,故常用 identity jump map。若需显式建模 touchdown impact,应使用 reset map)
- `linearApproximation()` 由 CppADCodeGen 自动生成(Ch48 流水线)

> **⚠️ 陷阱**: `jumpMap()` 在腿足中几乎不用,但在**弹跳机器人**(hopper)中很重要——着地瞬间速度突变。不要因为腿足不用就忽略这个接口。

#### 55.3.4 代价和约束的时间分类

OCS2 把代价和约束分为三个时间类别,这是与通用 NLP 框架的重要区别:

```
时间分类        何时激活             典型用途
──────────      ─────────            ──────
intermediate    t0 < t < tN          运行代价/约束(摩擦锥等)
preJump         模式切换时刻          切换代价(如碰撞能量)
final           t = tN               终端代价(稳定性保证)
```

```cpp
// 约束可以通过 isActive() 决定何时生效
class FrictionConeConstraint : public StateInputConstraint {
  bool isActive(scalar_t t) const override {
    // 只在触地相激活
    size_t mode = modeSchedulePtr_->getModeAtTime(t);
    return isContactActive(legIndex_, mode);
  }
};
```

#### 55.3.5 CppADCodeGen 集成流水线

OCS2 的自动微分流水线(Ch48 的直接应用):

```
用户写 flowMap()          CppAD 录制            CppADCodeGen 代码生成
(模板化 Scalar)            计算图                 C 源码 -> .so
─────────────────>  ─────────────────>  ─────────────────>
flowMap<ad_scalar_t>()    tape 记录              编译为共享库
                          所有运算                运行时 dlopen 加载

结果:
  - Jacobian A, B 自动计算,无需手推
  - 编译后的 .so 比解释执行快 10-100x
  - 首次运行需要编译(~30秒),之后直接加载
```

```cpp
// 用户代码示例: 实现模板化的 flowMap
template <typename SCALAR>
void LeggedRobotDynamics::flowMapImpl(
    SCALAR t,
    const Eigen::Matrix<SCALAR, -1, 1>& x,
    const Eigen::Matrix<SCALAR, -1, 1>& u,
    Eigen::Matrix<SCALAR, -1, 1>& dxdt) {
  // 用 Pinocchio 的模板化接口计算 Centroidal 动力学
  auto model = pinocchio::ModelTpl<SCALAR>(...);
  // ... Pinocchio centroidal dynamics computation
  dxdt = centroidalDynamics(model, x, u);
}
```

> **🧠 深度理解**: 为什么 OCS2 用 CppADCodeGen 而不是符号微分(SymPy)或有限差分?
> - **符号微分**: 对复杂的刚体动力学表达式会"爆炸"(表达式指数级膨胀)
> - **有限差分**: 精度差(O(h) 或 O(h^2)),且需要多次前向求值
> - **CppADCodeGen**: 精确到机器精度,且可以预编译为高效的 C 代码

**练习 55.3a**: 在 OCS2 的 `OptimalControlProblem` 中,如何表达以下约束?"摆动腿的末端 z 坐标必须高于 0.02m"。写出注册这个约束的伪代码,并指定 `isActive()` 的逻辑。

**练习 55.3b**: 解释为什么 OCS2 要求"state-input equality constraint 的 Jacobian w.r.t. input 必须满秩"。如果不满秩会发生什么?提示:和投影法(projection method)有关。

---

### 55.4 SQP 算法——为什么 OCS2 不用 DDP ⭐⭐⭐

#### 55.4.1 动机:Ch54 的续篇

> **前情回顾**: Ch54 详细讲了 DDP/FDDP/Crocoddyl。结论是——DDP 对无约束或软约束问题很高效,但腿足场景有大量硬约束。本节从 SQP 的角度重新审视。

**核心问题**: 为什么工业界(ANYmal, Go2, HyQ)普遍选择 SQP 而非 DDP?

如果不用 SQP 而坚持 DDP 处理硬约束会怎样?DDP 需要把不等式约束(摩擦锥、关节限位、自碰撞)转化为惩罚项或增广拉格朗日项——这引入了外循环(更新 $\lambda$ 和 $\rho$),每个外循环内部又要多次 DDP 迭代。在实时 MPC 中,这种"循环套循环"结构难以保证固定的计算预算。SQP 则在每次迭代中把所有约束线性化后交给 QP 一并处理,避免了嵌套循环。

```
DDP 处理约束的困境:

原始 DDP: 只有动力学约束,通过递推自动满足
FDDP:    加入可行性校正
AL-DDP:  用增广拉格朗日把不等式约束变成代价
         -> 问题:lambda 更新需要外循环,实时性差

SQP 处理约束的方式:

SQP:     每次迭代把约束线性化,在 QP 子问题中直接处理
         -> QP 求解器(HPIPM)原生支持等式/不等式约束
         -> 一次 QP 求解就处理了所有约束
         -> 无需外循环
```

#### 55.4.2 SQP 与 DDP 的对比总结

| 维度 | SQP | DDP |
|------|-----|-----|
| **线性化什么** | 动力学 + 代价 + 约束 | 只有代价和动力学 |
| **约束处理** | QP 直接处理(等式/不等式) | 硬嵌入动力学,其他用罚函数 |
| **时间结构** | QP 里有(带状稀疏) | 递推利用(backward pass) |
| **后端** | HPIPM/qpOASES | 自写 Riccati |
| **并行性** | 动力学线性化可并行 | backward pass 不能并行 |
| **每次迭代成本** | 高(需要 QP 求解) | 低(Riccati 递推) |
| **收敛速度** | 超线性(近最优时) | 二次(动力学可行时) |

#### 55.4.3 SQP 算法完整推导

**问题**: 求解如下 NLP(非线性规划):

$$\min_{\mathbf{x}, \mathbf{u}} \sum_{k=0}^{N-1} \ell_k(\mathbf{x}_k, \mathbf{u}_k) + \phi_N(\mathbf{x}_N)$$

subject to:
$$\mathbf{x}_{k+1} = f_k(\mathbf{x}_k, \mathbf{u}_k), \quad k = 0, \ldots, N-1$$
$$\mathbf{g}_k(\mathbf{x}_k, \mathbf{u}_k) = \mathbf{0}$$
$$\mathbf{h}_k(\mathbf{x}_k, \mathbf{u}_k) \geq \mathbf{0}$$

**SQP 思路**: 在当前迭代点 $(\bar{\mathbf{x}}, \bar{\mathbf{u}})$ 做二阶泰勒展开,得到 **QP 子问题**:

```
SQP 第 j 次迭代:

1. 线性化动力学:
   f_k(x_k, u_k) ~ f_k(x_bar_k, u_bar_k) + A_k * dx_k + B_k * du_k
   其中 A_k = df/dx|_{x_bar,u_bar}, B_k = df/du|_{x_bar,u_bar}

2. 二次化代价:
   l_k(x_k, u_k) ~ 1/2 [dx_k; du_k]^T [Q_k S_k; S_k^T R_k] [dx_k; du_k]
                     + [q_k; r_k]^T [dx_k; du_k] + const

3. 线性化约束:
   g_k(x_k, u_k) ~ g_k(x_bar, u_bar) + C_k * dx_k + D_k * du_k = 0
   h_k(x_k, u_k) ~ h_k(x_bar, u_bar) + E_k * dx_k + F_k * du_k >= 0

4. 求解 QP 子问题:
   min  1/2 sum [dx_k; du_k]^T H_k [dx_k; du_k] + g^T [dx_k; du_k]
   s.t. dx_{k+1} = A_k dx_k + B_k du_k + (f(x_bar,u_bar) - x_bar_{k+1})
        C_k dx_k + D_k du_k + g(x_bar,u_bar) = 0
        E_k dx_k + F_k du_k + h(x_bar,u_bar) >= 0

   ──> HPIPM 求解!

5. Line search:
   alpha = 1
   while merit(x_bar + alpha*dx, u_bar + alpha*du)
         > merit(x_bar, u_bar) + c * alpha * Dmerit:
     alpha *= beta  (Armijo 回缩, c=1e-4, beta=0.5)

6. 更新: x_bar <- x_bar + alpha*dx, u_bar <- u_bar + alpha*du

7. 收敛判定: ||dx||_inf < tol 且 ||du||_inf < tol
   ──> 若收敛,退出; 否则回到步骤 1
```

> **⚠️ 重要**: OCS2 的 SQP 用的是 **Multiple Shooting** 格式(Ch54 已介绍),不是 Single Shooting。这意味着 $\mathbf{x}_0, \ldots, \mathbf{x}_N$ 都是优化变量,而不是通过前向积分得到。Multiple Shooting 的好处是**QP 子问题有带状稀疏结构**,HPIPM 可以 O(N) 求解。

#### 55.4.4 SQP-RTI: Real-Time Iteration

**关键思想**: 在 MPC 场景中,不需要让 SQP 收敛到最优——**只做 1 次 SQP 迭代就返回**。

```
传统 SQP:                    SQP-RTI:
────────────────              ────────────────
MPC 周期开始                  MPC 周期开始
  线性化                        线性化
  求解 QP (iter 1)              求解 QP (iter 1)
  line search                   ──> 直接返回!
  线性化
  求解 QP (iter 2)
  line search
  ...
  收敛 (iter 3-5)
  ──> 返回
MPC 周期结束                  MPC 周期结束

时间: 30-50ms                 时间: 5-15ms
```

**为什么 RTI 可行?** 三个理论保证:

**1. 收缩性(Contraction)**

如果 SQP 的 Hessian 正定且 Jacobian 有界,则每次迭代都向最优解"收缩"。形式化地,存在 $\kappa < 1$ 使得:

$$\|\mathbf{z}^{j+1} - \mathbf{z}^*\| \leq \kappa \|\mathbf{z}^j - \mathbf{z}^*\|$$

其中 $\mathbf{z} = [\mathbf{x}; \mathbf{u}]$。这是 Diehl et al. (2005) 证明的核心定理(Theorem 4.1 on local contractivity)。

**2. 热启动(Warm Start)**

MPC 每次只前进一个采样周期($\Delta t \approx 20$ ms),上次的解经过时间 shift 后已经很接近新问题的最优解——所以"1 步就够了"。

**3. 连续跟踪**

RTI 的目标不是"求最优解",而是**跟踪最优解的轨迹**。当用 RTI 的初始猜测是参考轨迹时,第一步 Newton 方向恰好等于线性 MPC 的解。这提供了从线性 MPC 到非线性 MPC 的平滑桥接。

```
RTI 收缩性直觉图:

      最优解 z*(t)
         *
        /|
       / |  1 步收缩
      /  |
    *    |
    z^j  |
         |
   热启动点 = z*(t - dt) 经时间 shift
   (已经很接近 z*(t))
```

> **🧠 深度理解**: RTI 的思想来自 Moritz Diehl (Uni. Freiburg) 的经典论文。OCS2 的 `sqpIteration = 1` 参数就是 RTI 模式。设 `sqpIteration = 5` 就是传统 SQP。**实际部署中,RTI 几乎总是够用**——因为四足运动变化缓慢(相对于 MPC 频率)。

> **⚠️ 陷阱**: RTI 可行需要满足前提条件: (1) 采样时间足够小, (2) 预测时域足够长, (3) 积分器足够精确, (4) 使用了 shifting 策略。如果 MPC 频率太低(如 10Hz),RTI 可能不够——因为相邻两个问题差异太大,一步 SQP 无法收缩到足够近。

#### 55.4.5 OCS2 SqpSolver 源码导读

`ocs2_sqp/src/SqpSolver.cpp` 约 800 行。核心方法:

```cpp
// 简化的核心循环
void SqpSolver::runImpl(scalar_t initTime,
                        const vector_t& initState,
                        scalar_t finalTime) {
  // 1. 初始化轨迹 (热启动或线性插值)
  initializeTrajectories(initTime, initState, finalTime);

  for (int iter = 0; iter < settings_.sqpIteration; ++iter) {
    // 2. 线性化: 计算 A_k, B_k, Q_k, R_k, C_k, D_k
    //    (可并行: nThreads 个线程各负责一段 horizon)
    linearizeOcp(timeDiscretization_, ...);

    // 3. 构建 QP 子问题
    auto qpData = setupQuadraticSubproblem(
        linearization_, ...);

    // 4. 求解 QP (调用 HPIPM)
    auto qpSolution = hpipmInterface_.solve(qpData);

    // 5. Line search (Armijo)
    scalar_t stepSize = lineSearch(qpSolution, ...);

    // 6. 更新轨迹
    updateTrajectories(qpSolution, stepSize);

    // 7. 收敛判定
    if (converged(qpSolution, stepSize)) break;
  }
}
```

> **💡 关键设计**: 步骤 2 的线性化可以**多线程并行**。如果 horizon 有 N=15 个节点,`nThreads=3` 则每个线程处理 5 个节点。这是 SQP 相对于 DDP 的并行优势——DDP 的 backward pass 是串行的。ANYmal 的生产部署使用 4 核并行。

#### 55.4.6 Line Search: Armijo 条件

```
Armijo 条件:
  merit(z + alpha*dz) <= merit(z) + c * alpha * D_merit

其中:
  merit(z) = cost(z) + rho * ||constraints(z)||_1
  c = 1e-4 (sufficient decrease parameter)
  alpha 从 1 开始, 每次乘以 beta = 0.5 回缩

OCS2 实现:
  settings_.g_max = 1e-2   // merit 中约束违反的初始权重
  settings_.g_min = 1e-6   // 最小权重(避免数值问题)
```

> **⚠️ 陷阱**: 如果 line search 总是回缩到很小的 alpha(比如 0.001),说明 QP 子问题的搜索方向不好。可能原因: (1) Hessian 不正定(需要正则化), (2) 约束不一致(infeasible 问题), (3) 线性化点离最优太远(热启动没做好)。

**练习 55.4a**: 在 task.info 中把 `sqpIteration` 从 1 改到 5。记录每次迭代的 `||delta||` 和 `alpha`。画收敛曲线,验证 RTI(iter=1)是否已经"足够好"。

**练习 55.4b**: 解释为什么 OCS2 的 SQP 使用 Multiple Shooting 而非 Single Shooting。如果用 Single Shooting,HPIPM 还能用吗?

---

### 55.5 HPIPM 作为 SQP 后端 ⭐⭐⭐

#### 55.5.1 回顾与定位

> **Ch50 已详细介绍 HPIPM**。本节聚焦: OCS2 如何把 OCP 映射到 HPIPM 的 QP 格式。

HPIPM (High-Performance Interior Point Method) 是 Gianluca Frison 开发的高性能 QP 求解器,使用 BLASFEO 作为线性代数后端,专门优化了 OCP 结构的 block-sparse QP。

#### 55.5.2 Block-Sparse QP 结构

Multiple Shooting 产生的 QP 有特殊的**带状稀疏结构**:

```
QP 的 KKT 矩阵结构 (N=4 horizon 示例):

    dx0  du0  dx1  du1  dx2  du2  dx3  du3  dx4
    ┌───┬───┬────┬────┬────┬────┬────┬────┬───┐
dx0 │ Q0│   │ A0T│    │    │    │    │    │   │
du0 │   │ R0│ B0T│    │    │    │    │    │   │
    ├───┼───┼────┼────┼────┼────┼────┼────┼───┤
dx1 │A0 │B0 │ Q1 │    │ A1T│    │    │    │   │
du1 │   │   │    │ R1 │ B1T│    │    │    │   │
    ├───┼───┼────┼────┼────┼────┼────┼────┼───┤
dx2 │   │   │ A1 │ B1 │ Q2 │    │ A2T│    │   │
du2 │   │   │    │    │    │ R2 │ B2T│    │   │
    ├───┼───┼────┼────┼────┼────┼────┼────┼───┤
dx3 │   │   │    │    │ A2 │ B2 │ Q3 │    │A3T│
du3 │   │   │    │    │    │    │    │ R3 │B3T│
    ├───┼───┼────┼────┼────┼────┼────┼────┼───┤
dx4 │   │   │    │    │    │    │ A3 │ B3 │ QN│
    └───┴───┴────┴────┴────┴────┴────┴────┴───┘

非零块只出现在"对角线"和"次对角线" -> 带状稀疏!
HPIPM 利用这个结构: O(N * (nx+nu)^3) 而非 O((N*(nx+nu))^3)
```

#### 55.5.3 OCS2 HpipmInterface 的映射

```cpp
// ocs2_sqp/include/ocs2_sqp/HpipmInterface.h (简化)
class HpipmInterface {
public:
  // 设置 QP 维度
  void resize(int N, const std::vector<int>& nx,
              const std::vector<int>& nu,
              const std::vector<int>& ng);

  // 设置 QP 数据: 从 OCS2 的线性化结果映射
  void setDynamics(int k,
      const matrix_t& A, const matrix_t& B, const vector_t& b);
  void setCost(int k,
      const matrix_t& Q, const matrix_t& R, const matrix_t& S,
      const vector_t& q, const vector_t& r);
  void setConstraints(int k,
      const matrix_t& C, const matrix_t& D,
      const vector_t& lg, const vector_t& ug);

  // 求解
  HpipmStatus solve();

  // 获取结果
  const vector_t& getDeltaX(int k) const;
  const vector_t& getDeltaU(int k) const;
};
```

**映射流程**:

```
OCS2 OptimalControlProblem
     │
     ├── dynamicsPtr->linearApproximation()  ──>  setDynamics(k, A, B, b)
     ├── cost->getQuadraticApproximation()    ──>  setCost(k, Q, R, S, q, r)
     ├── equalityConstraint->linearApprox()   ──>  setConstraints(k, C, D, lg, ug)
     │                                              (lg = ug = -g(x_bar, u_bar))
     └── inequalityConstraint->linearApprox() ──>  setConstraints(k, E, F, lh, uh)
                                                    (lh = -h(x_bar, u_bar), uh = +inf)
     ▼
  hpipmInterface_.solve()
     ▼
  getDeltaX(k), getDeltaU(k)  ──>  SqpSolver line search
```

#### 55.5.4 性能基准

| 场景 | nx | nu | N | HPIPM 单次 | qpOASES | OSQP |
|------|----|----|---|-----------|---------|------|
| Centroidal 四足 | 24 | 24 | 15 | **1-2 ms** | 3-5 ms | 5-10 ms |
| Kino-centroidal | 36 | 24 | 15 | **2-4 ms** | 8-15 ms | 10-20 ms |
| 全身动力学 | 54 | 24 | 10 | **5-10 ms** | 30-50 ms | 不适用 |
| Mobile manipulator | 18 | 12 | 20 | **0.5-1 ms** | 1-2 ms | 2-3 ms |

> **💡 洞察**: HPIPM 的优势在**高维状态**和**长 horizon** 时更显著。因为 HPIPM 利用 OCP 的带状稀疏结构,复杂度是 O(N * n^3);而通用 QP 求解器(qpOASES/OSQP)把稀疏结构展平,复杂度是 O(N^3 * n^3)。最近的基准测试(Stark et al. 2024)在桌面 x86、LattePanda Alpha、Jetson Orin NX 三种平台上验证了这一点。

#### 55.5.5 内存预分配策略

```cpp
// OCS2 的 HPIPM 封装在初始化时一次性分配所有内存
// 避免运行时 malloc (实时系统禁忌)
void HpipmInterface::initialize() {
  // 1. 计算所需内存
  int memSize = d_ocp_qp_ipm_ws_memsize(&dim_, &arg_);

  // 2. 一次性 malloc
  memory_ = malloc(memSize);

  // 3. 创建工作空间(指针指向预分配内存)
  d_ocp_qp_ipm_ws_create(&dim_, &arg_,
                          &workspace_, memory_);

  // 运行时: solve() 只在预分配内存上操作,零 malloc
}
```

> **⚠️ 陷阱**: 如果你修改了 QP 的维度(比如添加了新约束),必须重新调用 `resize()` -> `initialize()`。运行时动态改变约束数量会导致内存不足 segfault。OCS2 的做法是**预分配最大可能的约束数量**,运行时用 `isActive()` 启用/禁用。

#### 55.5.6 Condensing vs Non-condensing

```
Non-condensing (OCS2 默认):
  保留所有 x_k 和 u_k 作为优化变量
  QP 维度: N*(nx+nu) + nx_terminal
  结构: block-sparse (HPIPM 擅长)

Condensing:
  消去所有 x_k (用动力学约束代入)
  QP 维度: N*nu (只剩输入)
  结构: dense (qpOASES 擅长)

选择:
  N 大或 nx 大 -> Non-condensing + HPIPM
  N 小且 nu 小 -> Condensing + qpOASES
  OCS2 腿足场景: N=15, nx=24, nu=24 -> Non-condensing
```

**练习 55.5**: 对于一个 6-DOF 机械臂 MPC(nx=12, nu=6, N=20),估算 HPIPM 和 qpOASES 的理论 flop 数量比值。HPIPM 还有优势吗?

---

### 55.6 模型选型——Centroidal vs Kino-centroidal ⭐⭐

SQP 框架和 HPIPM 后端确定了"怎么解",但 MPC 的性能上限取决于"解什么"——即动力学模型的选择。模型太简(如 LIPM)则无法表达复杂动作,模型太复杂(如全身动力学)则求解太慢。Centroidal 和 Kino-centroidal 是当前腿足 MPC 最主流的两种折中方案。

#### 55.6.1 Centroidal Model 完整定义

> **Ch49 的直接应用**。这里给出 OCS2 中的具体维度。

**状态向量** $\mathbf{x} \in \mathbb{R}^{24}$:

```
x = [h_G;  q_base;  q_joint]
     ───    ──────   ───────
     6      6        12 (四足)

h_G = [p_x, p_y, p_z, L_x, L_y, L_z]  (线动量 + 角动量)
q_base = [x, y, z, yaw, pitch, roll]    (基座位姿, ZYX 欧拉角)
q_joint = [q1, ..., q12]                (12 个关节角)

总维度: 6 + 6 + 12 = 24 (OCS2 默认)
```

**输入向量** $\mathbf{u} \in \mathbb{R}^{24}$:

```
u = [lambda_1; lambda_2; lambda_3; lambda_4; dq_joint]
     ────────  ────────  ────────  ────────  ────────
     3         3         3         3         12

lambda_i = [Fx, Fy, Fz]_i  (第 i 只脚的接触力)
dq_joint = [dq1, ..., dq12]  (关节速度)

总维度: 12 + 12 = 24
```

**动力学方程**(连续时间):

$$\dot{\mathbf{h}}_G = \sum_{i \in \mathcal{C}} \begin{bmatrix} \boldsymbol{\lambda}_i \\ \boldsymbol{r}_i \times \boldsymbol{\lambda}_i \end{bmatrix} + \begin{bmatrix} m\mathbf{g} \\ \mathbf{0} \end{bmatrix}$$

$$\dot{\mathbf{q}}_{\text{base}} = \mathbf{A}_b(\mathbf{q})^{-1} \left(\mathbf{h}_G - \mathbf{A}_j(\mathbf{q}) \dot{\mathbf{q}}_{\text{joint}}\right)$$

> **注意**: $\mathbf{A}_G$ 是 $6 \times n_v$ 的 Centroidal Momentum Matrix,不可逆。按基座/关节分块 $\mathbf{A}_G = [\mathbf{A}_b \;\; \mathbf{A}_j]$,其中 $\mathbf{A}_b$ 是 $6 \times 6$ 基座块(可逆),从而解出基座速度。

$$\dot{\mathbf{q}}_{\text{joint}} = \mathbf{u}_{\text{joint}}$$

> **💡 关键洞察**: 注意**接触力 $\boldsymbol{\lambda}_i$ 是输入变量**,不是动力学的输出。MPC 直接优化"应该施加什么力"。这和 WBC(Ch53)不同——WBC 是给定期望力后求关节力矩。

#### 55.6.2 Full Centroidal vs SRBD

OCS2 的 `centroidalModelType` 参数区分两种 Centroidal 变体:

```
centroidalModelType = 0: Full Centroidal Dynamics (FCD)
  - 使用完整的 Centroidal Momentum Matrix A_G(q)
  - A_G 随关节角变化 -> 更精确
  - 计算代价: 每步需要 Pinocchio 的 A_G 计算
  - 适用: ANYmal 等大型四足(腿质量不可忽略)

centroidalModelType = 1: Single Rigid Body Dynamics (SRBD)
  - 假设 A_G 恒定(不随关节角变化)
  - 等价于把机器人视为单刚体 + 附属肢体
  - 计算代价: 低(无需更新 A_G)
  - 适用: Go1/A1 等小型四足(腿质量 << 躯干质量)
```

#### 55.6.3 Kino-centroidal Model

**区别**: 输入变量改为**关节加速度** $\ddot{\mathbf{q}}_{\text{joint}}$,接触力通过动力学约束隐式确定。

```
Centroidal:                    Kino-centroidal:
  输入 = [接触力; 关节速度]       输入 = [关节加速度]
  接触力直接优化                  接触力由 KKT 系统隐式确定
  简单,维度低                    复杂,更物理
  MPC 输出力 -> WBC 分配力矩     MPC 输出加速度 -> 更直接
```

#### 55.6.4 选型决策树

```
你的机器人场景:
│
├── 标准四足步态(trot, walk, gallop)
│   ├── 小型四足(Go1, A1, 腿质量<15%总质量)
│   │   └── SRBD (centroidalModelType = 1)
│   └── 大型四足(ANYmal, HyQ, 腿质量>15%)
│       └── Full Centroidal (centroidalModelType = 0)
│
├── 需要精确关节力控制(搬运重物)
│   └── Kino-centroidal Model
│
├── 嵌入式部署(算力受限)
│   └── SRBD (必须)
│       理由: 求解时间少 40-60%
│
└── 学术研究(全身最优)
    └── 两者都试, 对比论文
```

> **⚠️ 陷阱**: `centroidalModelType = 1` (SRBD) 不是"Kino-centroidal"——它是更简化的"单刚体模型"。SRBD 假设惯量矩阵恒定(不随关节角变化),适用于小型四足。对于大型四足(如 ANYmal),SRBD 误差较大,应用 FCD。

**练习 55.6**: 用 Pinocchio 计算你的四足机器人在站立姿态和最大弯腿姿态下的 Centroidal Momentum Matrix $\mathbf{A}_G$。比较两者的差异大小(Frobenius 范数)。如果差异小于 5%,SRBD 就足够了。

---

### 55.7 双线程 MPC 架构——OCS2 的灵魂 ⭐⭐⭐

#### 55.7.1 问题背景:MPC 的实时困境

> **自检问题**: MPC 求解要 10-50 ms,但控制器需要每 1 ms 一次参考值。怎么办?

三种可能的方案:

```
方案 A: 同步执行 (最简单, 最差)
─────────────────────────────
控制循环: [ MPC 50ms ][ 等待 ][ MPC 50ms ][ 等待 ]...
          只有 MPC 完成后才能输出
          -> 控制频率 = MPC 频率 = 20 Hz (太慢!)

方案 B: 异步 + Mutex (常见, 有隐患)
─────────────────────────────
MPC 线程: [====求解====]      [====求解====]
MRT 线程: [读][读][读][读]...[读][读][读][读]
          mutex 保护共享数据
          -> 问题: MRT 读时 MPC 在写, mutex 导致 MRT 阻塞!
                   实时线程不能等待 (违反实时性)

方案 C: Triple Buffer (OCS2 的选择)
─────────────────────────────
MPC 线程: [====写 buf_A====]      [====写 buf_C====]
                      swap               swap
读端候选: [  buf_C (旧)  ] -> [  buf_A (新)  ]
MRT 线程: [读 buf_B][读 buf_B][读 buf_A][读 buf_A]...
          -> MRT 永远读最新的已完成 buffer
          -> 无锁! 无阻塞! 实时安全!
```

这好比一家报社的印刷流程:编辑(MPC 线程)在写新一期报纸时,读者(MRT 线程)可以随时阅读已印好的上一期——编辑写完后把新一期放到"待取区",读者下次来取时自动拿到最新版。三个缓冲区分别对应"正在编辑的稿件""读者手中的报纸""待取区的最新版"。这种设计保证了读者永远不会被迫等待编辑完成,也不会读到写了一半的内容。

> **本质洞察**:双线程 MPC 架构的核心不是"让 MPC 更快",而是**将求解质量与实时性解耦**——MPC 线程可以不受实时约束地追求更好的解(更多迭代、更长时域),MRT 线程则以固定 1kHz 频率从最新可用解中插值出当前控制量。这种解耦意味着 MPC 求解偶尔超时(如地形突变时)不会导致控制中断——MRT 继续用旧解 + 反馈增益维持稳定。

#### 55.7.2 Triple Buffer 详解

**Triple Buffer 的三个角色**:

```
┌─────────────────────────────────────────────────────────┐
│                Triple Buffer State Machine              │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐              │
│  │ Buffer A │  │ Buffer B │  │ Buffer C │              │
│  │  (写端)  │  │ (读端)   │  │(读端候选)│              │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘              │
│        │             │             │                    │
│  MPC 正在          MRT 正在       上次 MPC               │
│  写入新 policy     读取这个       完成的 policy          │
│                    policy         (等待被读取)           │
│                                                         │
│  当 MPC 写完时:                                          │
│    swap(写端, 读端候选)                                  │
│    Buffer A <-> Buffer C                                │
│    现在 A 是读端候选, C 是新的写端                        │
│                                                         │
│  当 MRT 准备读新数据时:                                   │
│    swap(读端, 读端候选)                                  │
│    Buffer B <-> Buffer A (如果 A 有新数据)               │
│    现在 B 有最新 policy                                  │
└─────────────────────────────────────────────────────────┘
```

**为什么需要 3 个 buffer 而不是 2 个?**

```
Double Buffer 的问题:
  MPC 写 A, MRT 读 B
  MPC 写完, swap: MPC 写 B, MRT 读 A
  -> 如果 MRT 还没读完 B, MPC 就 swap 了
     MPC 开始写 B, MRT 还在读 B -> 数据竞争!

Triple Buffer 的解决:
  MPC 写 A, MRT 读 B, 候选 C
  MPC 写完, swap(A, C): MPC 写 C, 候选 A
  MRT 还在读 B (安全! B 没人动)
  MRT 读完, swap(B, A): MRT 读 A (新数据!)
  -> 三方各读各写, 永远没有冲突
```

#### 55.7.3 C++ 实现

```cpp
// 简化的 Triple Buffer 实现
template <typename T>
class TripleBuffer {
  std::array<T, 3> buffers_;
  std::atomic<int> writeIdx_{0};
  std::atomic<int> readIdx_{1};
  std::atomic<int> spareIdx_{2};
  std::atomic<bool> newDataAvailable_{false};

public:
  // 生产者(MPC 线程)调用
  T& getWriteBuffer() {
    return buffers_[writeIdx_.load(std::memory_order_relaxed)];
  }

  void publishWrite() {
    // swap(writeIdx, spareIdx)
    int w = writeIdx_.load(std::memory_order_relaxed);
    int s = spareIdx_.exchange(w, std::memory_order_acq_rel);
    writeIdx_.store(s, std::memory_order_relaxed);
    newDataAvailable_.store(true, std::memory_order_release);
  }

  // 消费者(MRT 线程)调用
  bool updateRead() {
    if (!newDataAvailable_.load(std::memory_order_acquire)) {
      return false;  // no new data
    }
    // swap(readIdx, spareIdx)
    int r = readIdx_.load(std::memory_order_relaxed);
    int s = spareIdx_.exchange(r, std::memory_order_acq_rel);
    readIdx_.store(s, std::memory_order_relaxed);
    newDataAvailable_.store(false, std::memory_order_release);
    return true;
  }

  const T& getReadBuffer() const {
    return buffers_[readIdx_.load(std::memory_order_relaxed)];
  }
};
```

> **🧠 深度理解**: 注意 `memory_order` 的选择。`acq_rel` 用于 swap 操作(同步点),`relaxed` 用于同一线程内的读写(无需同步)。这比 `seq_cst` 更高效,但正确性依赖于"只有一个生产者和一个消费者"的假设(SPSC)。这正是 v8 Ch17-20 讲过的 SPSC 无锁数据结构在 MPC 场景的落地。

#### 55.7.4 OCS2 的 MPC_MRT_Interface

```cpp
// ocs2_mpc/include/ocs2_mpc/MPC_MRT_Interface.h (简化)
class MPC_MRT_Interface {
public:
  // ========== MPC 线程调用 ==========

  // 用最新观测推进 MPC 一步
  void setCurrentObservation(const SystemObservation& obs);

  // 执行一次 MPC 求解
  bool advanceMpc();

  // ========== MRT 线程调用 ==========

  // 更新读端(如果有新 policy 可用)
  void updatePolicy();

  // 按时间查询 policy (开环): time-based view
  void evaluatePolicy(scalar_t t,
                      const vector_t& x,
                      vector_t& xDesired,
                      vector_t& uDesired,
                      size_t& mode);

  // 按时间+状态查询 policy (闭环): state-based view
  // 需要 solver 设置中启用 feedback policy
  void evaluateFeedbackPolicy(scalar_t t,
                              const vector_t& x,
                              vector_t& uFeedback);

private:
  std::unique_ptr<MPC_BASE> mpcPtr_;        // MPC 求解器
  BufferedValue<PolicyData> policyBuffer_;   // 双重缓冲
  BufferedValue<CommandData> commandBuffer_; // 命令缓冲
};
```

**BufferedValue vs TripleBuffer**: OCS2 实际使用的是 `BufferedValue`(双重缓冲 + 原子标志),原理与 Triple Buffer 相同——**写端和读端不会访问同一块内存**。当 MPC 写完新 policy 时,BufferedValue 内部做一次原子 swap,MRT 下次读取时就能看到新数据。

#### 55.7.5 两种查询模式: Time-based vs State-based

```
Time-based view (开环):
  输入: 当前时间 t
  输出: x_des(t), u_des(t)   通过线性插值 stateTrajectory
  用途: 简单 PD 跟踪

State-based view (闭环):
  输入: 当前时间 t, 当前状态 x_meas
  输出: u = u_ff(t) + K(t) * (x_meas - x_des(t))
  用途: 含反馈增益的高级跟踪
  前提: solver 必须计算 feedback policy (controllerTrajectory)
```

> **💡 实用建议**: 对于 MPC + WBC 的架构(如 legged_control),用 time-based view 就够了——WBC 本身就是反馈控制器。只有当直接用 MPC 输出驱动电机(无 WBC)时,才需要 state-based view 的反馈增益。

#### 55.7.6 线程安全分析

```
关键问题: 为什么不需要 mutex?

分析:
1. MPC 线程: 只写 policyBuffer_ 的写端
2. MRT 线程: 只读 policyBuffer_ 的读端
3. swap 操作: 原子整数交换, lock-free

不变式 (Invariants):
- 写端和读端指向不同的物理 buffer
- swap 是原子操作, 不存在中间状态
- MRT 不关心 MPC 正在写什么, 只读"最新已完成"

对比 mutex 方案:
┌──────────────┬──────────────┬──────────────┐
│ 属性          │ Mutex        │ Triple Buffer│
├──────────────┼──────────────┼──────────────┤
│ 阻塞         │ 是           │ 否           │
│ 优先级反转   │ 可能         │ 不可能       │
│ 延迟可预测   │ 否(取决于锁) │ 是(常数时间)│
│ 数据新鲜度   │ 最新         │ 最新已完成   │
│ 实现复杂度   │ 低           │ 中           │
│ 实时安全     │ 否           │ 是           │
└──────────────┴──────────────┴──────────────┘
```

> **⚠️ 陷阱**: "实时安全"不仅仅是"快"——更重要的是**可预测性**。Mutex 在无竞争时很快(~20ns),但在竞争时可能导致几百微秒的阻塞,且不确定何时发生。Triple Buffer 的延迟始终是常数时间(一次 atomic exchange ~5ns)。

#### 55.7.7 当 MPC 求解慢时

```
正常情况:
MPC: [===solve===]      [===solve===]      [===solve===]
MRT: [r][r][r][r][r][r] [r][r][r][r][r][r] [r][r][r][r]
      读到 policy#1      读到 policy#2      读到 policy#3

MPC 慢了 (比如地形复杂,QP 迭代多):
MPC: [=======solve=======]               [===solve===]
MRT: [r][r][r][r][r][r][r][r][r][r][r] [r][r][r][r]
      读到 policy#1 ... 继续用 policy#1   读到 policy#2

MRT 会一直用最后一个有效的 policy!
这是安全的——因为 policy 包含完整的预测 horizon (1秒)
只要 MPC 不连续失败超过 horizon 时间, 系统就是安全的
```

**当 MPC 失败时**(QP infeasible 或发散):

```cpp
bool MPC_MRT_Interface::advanceMpc() {
  try {
    mpcPtr_->run(currentObservation_);
    // 成功: 写入新 policy
    policyBuffer_.write(extractPolicy());
    return true;
  } catch (const std::runtime_error& e) {
    // 失败: 不写入新 policy
    // MRT 继续用上次的 policy (安全降级)
    RCLCPP_WARN(logger_, "MPC solve failed: %s", e.what());
    return false;
  }
}
```

> **💡 洞察**: 这种"失败时用旧 policy"的策略叫做 **graceful degradation**。只要 MPC 偶尔失败(不是连续失败),系统可以继续运行。这在真实硬件上非常重要——QP solver 可能因为突发的大扰动而暂时无解。

#### 55.7.8 实时线程配置

```cpp
// 在真实机器人上,MRT 线程必须设置为实时优先级
#include <pthread.h>

void setRealtimePriority(int priority = 49) {
  struct sched_param param;
  param.sched_priority = priority;  // 1-99, higher = more priority
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  // SCHED_FIFO: 先入先出实时调度
  // 优先级 49: 低于系统关键线程, 高于普通线程
}

// 典型配置:
// MRT 线程: SCHED_FIFO, priority 49  (实时, 1kHz)
// MPC 线程: SCHED_OTHER, nice 0       (普通, 50-100Hz)
// ROS 话题: SCHED_OTHER, nice 10      (低优先级)
```

> **⚠️ 陷阱**: 在 Ubuntu 上设置 SCHED_FIFO 需要 root 权限或 `CAP_SYS_NICE`。可以用 `ulimit -r 99` 或在 `/etc/security/limits.conf` 中配置。忘记设置实时优先级是腿足机器人摔倒的常见原因之一——MRT 被其他线程抢占导致控制延迟。

#### 55.7.9 延迟分析: 从观测到力矩

```
从观测到力矩输出的完整延迟链:

传感器采样        0.1 ms
状态估计          0.2 ms
─── MPC 周期边界 ───
MPC 求解         10-50 ms (异步, 不阻塞 MRT)
─── MRT 周期边界 ───
MRT 查询          0.01 ms (插值)
WBC 求解          0.5-1 ms
力矩输出          0.1 ms
─── 控制周期结束 ───

MRT 到力矩的延迟 (确定性, 在 PREEMPT_RT + CPU affinity 配置下): < 2 ms
MPC 延迟 (观测到输出被使用): 1-2 个 MPC 周期 = 10-40 ms
```

```
延迟与稳定性的关系:

MPC 延迟 (ms)    效果
─────────────    ────
< 20             优秀: 快步态(跑步)稳定
20 - 50          良好: 慢步态(行走)稳定
50 - 100         勉强: 站立和慢走可以,快速运动不稳定
> 100            危险: 容易摔倒

ANYmal 部署参考 (Intel i7-8850H, 6 核):
  MPC @ 100Hz (4核并行): 单次 5-10ms -> 优秀
  MRT @ 400Hz: 确定性延迟 < 1ms
```

**练习 55.7a**: 在你的电脑上编译运行 OCS2 legged_robot 示例,记录 MPC 的单次求解时间。和 ANYmal 的基准(5-10ms on i7)比较。瓶颈在哪里?

**练习 55.7b**: 画出 Triple Buffer 在以下场景中的时序图: MPC 每次求解 30ms,MRT 每 2.5ms 查询一次。标注 MRT 何时读到新数据,何时读旧数据。

**练习 55.7c**: 如果 MPC 连续 3 次求解失败(共 60ms),但 policy 的 horizon 是 1.0 秒。系统会怎样?如果连续失败 1 秒呢?

---

### 55.8 ReferenceManager 与步态管理 ⭐⭐

#### 55.8.1 ReferenceManagerInterface

```cpp
class ReferenceManagerInterface {
public:
  // 在 MPC 求解前调用: 更新参考
  virtual void preSolverRun(scalar_t initTime,
                            scalar_t finalTime,
                            const vector_t& currentState) = 0;

  // 获取目标轨迹
  virtual const TargetTrajectories&
  getTargetTrajectories() const = 0;

  // 获取模式调度
  virtual const ModeSchedule& getModeSchedule() const = 0;
};
```

**腿足特化**: `SwitchedModelReferenceManager` 继承 `ReferenceManagerInterface`,持有 `GaitSchedule` 对象:

```cpp
class SwitchedModelReferenceManager
    : public ReferenceManagerInterface {
public:
  void preSolverRun(/*...*/) override {
    // 1. 更新步态调度
    gaitSchedule_->advanceTime(initTime);
    modeSchedule_ = gaitSchedule_->getModeSchedule(
        initTime, finalTime);

    // 2. 从用户命令生成参考轨迹
    targetTrajectories_ = generateTrajectory(
        currentState, userCommand_, initTime, finalTime);

    // 3. 更新摆动腿轨迹
    updateSwingTrajectories(modeSchedule_);
  }
};
```

#### 55.8.2 ModeSchedule 深度解析

```cpp
struct ModeSchedule {
  scalar_array_t eventTimes;    // 模式切换时刻
  size_array_t   modeSequence;  // 模式序列

  // 示例: trot 步态, 从 t=0 开始, 步态周期 0.4s
  // N 个 mode 需要 N-1 个 eventTime
  // eventTimes  = [0.2, 0.4, 0.6, 0.8, 1.0]
  // modeSequence = [6,   9,   6,   9,   6,   9]
};
```

**Mode 编码 (4-bit, 对应 4 条腿)**:

```
bit 3: LF (左前)    bit 2: RF (右前)
bit 1: LH (左后)    bit 0: RH (右后)

常见步态的 mode 编码:

步态         mode 序列            描述
─────        ──────────           ────
Stance       [15]                 0b1111 = 四脚触地
Trot         [6, 9]               0b0110, 0b1001 = 对角交替
Walk         [14, 13, 11, 7]      逐脚抬起
Pace         [5, 10]              0b0101, 0b1010 = 同侧交替
Bound        [3, 12]              0b0011, 0b1100 = 前后交替
Pronk        [0, 15]              全腾空 + 全触地
Flying trot  [6, 0, 9, 0]         对角 + 腾空相
```

> **💡 与 Ch56 的关系**: Ch56 会完全展开步态管理(GaitSchedule、步态切换、自适应步态)。本节只需理解 ModeSchedule 的数据结构和编码方式。

#### 55.8.3 按 mode 启用/禁用约束

OCS2 的约束可以按 mode 动态启用——这是"切换系统"抽象的精髓:

```cpp
// 零力约束: 摆动腿不施加力
class ZeroForceConstraint : public StateInputConstraint {
  bool isActive(scalar_t t) const override {
    size_t mode = referenceManager_->getModeSchedule()
                      .getModeAtTime(t);
    return !isContactActive(legIndex_, mode);
    // 腿不接触 -> 约束激活: 力必须为零
  }

  vector_t getValue(scalar_t t,
                    const vector_t& x,
                    const vector_t& u) const override {
    return getContactForce(u, legIndex_);  // lambda_i = 0
  }
};

// 零速度约束: 触地腿末端不滑动
class ZeroVelocityConstraint : public StateInputConstraint {
  bool isActive(scalar_t t) const override {
    size_t mode = referenceManager_->getModeSchedule()
                      .getModeAtTime(t);
    return isContactActive(legIndex_, mode);
    // 腿接触 -> 约束激活: 脚速度为零
  }

  vector_t getValue(scalar_t t, const vector_t& x,
                    const vector_t& u) const override {
    return footVelocity(x, u, legIndex_);  // v_foot = 0
  }
};
```

> **🧠 深度理解**: 同一条腿在不同时刻有完全不同的约束集——触地相有摩擦锥约束+零速度约束,摆动相有零力约束。这种"约束随时间切换"的模式就是 OCS2 作为"切换系统"框架的核心价值。

---

### 55.9 ROS Wrapper——MPC_Node 和 MRT_Node ⭐⭐

#### 55.9.1 两种部署模式

```
模式 A: 同进程 (推荐, 低延迟)
──────────────────────────────
┌─────────────────────────────┐
│ 单个 ROS2 Node              │
│                             │
│  MPC 线程 ─> BufferedValue  │
│  MRT 线程 <─ BufferedValue  │
│                             │
│  使用: MPC_MRT_Interface    │
│  延迟: < 0.1ms (函数调用)   │
└─────────────────────────────┘

模式 B: 跨进程 (灵活, 可分布式)
──────────────────────────────
┌──────────────┐    ROS2 话题    ┌──────────────┐
│ MPC Node     │  /mpc_policy   │ MRT Node     │
│              │ ─────────────> │              │
│ MPC_ROS_     │                │ MRT_ROS_     │
│ Interface    │  /observation  │ Interface    │
│              │ <───────────── │              │
└──────────────┘                └──────────────┘
  延迟: 1-5ms (序列化+传输)
```

**何时选择跨进程?** 当 MPC 和控制器跑在不同的计算机上(如 MPC 在高性能笔记本,控制器在机载嵌入式)。ANYmal 的早期版本用过这种架构,后来统一到同进程。

#### 55.9.2 序列化开销分析

```
PolicyData 的大小 (ANYmal, N=15, nx=24, nu=24):
  timeTrajectory:   15 * 8 bytes  =    120 bytes
  stateTrajectory:  15 * 24 * 8   =  2,880 bytes
  inputTrajectory:  15 * 24 * 8   =  2,880 bytes
  controllerData:   15 * (24*24 + 24) * 8 = ~69,120 bytes
  ────────────────────────────────────────────────
  总计:            ~75 KB per MPC update

  @ 50 Hz:         ~3.75 MB/s

  序列化+反序列化: ~0.5-1 ms (msgpack)
  ROS2 传输 (同机): ~0.2-0.5 ms (shared memory transport)
  ─────────────────────────────────
  跨进程总开销:    ~1-2 ms per update
```

> **⚠️ 陷阱**: 如果你用 `ros2 topic echo` 监控 `/mpc_policy`,可能看到延迟增加——因为 `echo` 也要反序列化。监控 MPC 性能应该用内部计时,不要依赖 ROS 工具。

#### 55.9.3 Dummy Node: 快速验证

OCS2 提供了 `LeggedRobotDummyNode`,它是一个**不带物理引擎的简单仿真器**:

```
Dummy Node 的工作方式:
1. 接收 MPC 的 policy
2. 用 policy 中的状态轨迹做开环前向积分
3. 发布积分后的状态作为"观测"反馈给 MPC
4. 在 RViz 中可视化预测轨迹和接触力

用途: 在没有 Gazebo/MuJoCo 的情况下快速验证 MPC 是否工作
限制: 无碰撞、无摩擦——只是"理想"仿真
```

---

### 55.10 task.info 完整配置教程 ⭐⭐

#### 55.10.1 task.info 关键参数详解

```ini
; ============================================================
; OCS2 legged_robot task.info 完整注释版
; ============================================================

; --- 接口设置 ---
[legged_robot_interface]
verbose = false                      ; 打印详细日志

; --- 模型设置 ---
[model_settings]
centroidalModelType = 0              ; 0=Full Centroidal, 1=SRBD
positionErrorGain = 0.0              ; 位置误差增益
phaseTransitionStanceTime = 0.4      ; 相位切换的站立时间 (秒)
verboseCppAd = true                  ; CppAD 编译详细输出
recompileLibrariesCppAd = true       ; 强制重新编译 AD 库
modelFolderCppAd = /tmp/ocs2         ; AD 编译输出目录

; --- SQP 求解器设置 ---
[sqp]
nThreads = 3                         ; 并行线性化线程数
dt = 0.015                           ; 离散化步长 (秒)
sqpIteration = 1                     ; SQP 迭代次数 (1 = RTI 模式!)
deltaTol = 1e-4                      ; 收敛容差
g_max = 1e-2                         ; 约束违反容忍上限
g_min = 1e-6                         ; 约束违反容忍下限
inequalityConstraintMu = 0.1         ; 不等式约束 relaxed barrier mu
inequalityConstraintDelta = 5.0      ; 不等式约束松弛 Delta
integratorType = RK2                 ; 积分器类型 (RK2 够用)
threadPriority = 50                  ; 线程优先级

; --- MPC 设置 ---
[mpc]
timeHorizon = 1.0                    ; 预测时域 (秒)
solutionTimeWindow = -1              ; 负值 = 使用整个时域
mpcDesiredFrequency = 50             ; MPC 期望频率 (Hz)
mrtDesiredFrequency = 400            ; MRT 期望频率 (Hz)

; --- 初始状态 (24 维) ---
initialState
{
  ; Centroidal momentum (6): 初始为零
  (0,0)  0.0  ; px (linear momentum x)
  (1,0)  0.0  ; py
  (2,0)  0.0  ; pz
  (3,0)  0.0  ; Lx (angular momentum x)
  (4,0)  0.0  ; Ly
  (5,0)  0.0  ; Lz
  ; Base pose (6)
  (6,0)  0.0    ; x position
  (7,0)  0.0    ; y position
  (8,0)  0.575  ; z position (ANYmal 站立高度)
  (9,0)  0.0    ; yaw
  (10,0) 0.0    ; pitch
  (11,0) 0.0    ; roll
  ; Joint angles (12)
  (12,0)  0.0   ; LF_HAA
  (13,0)  0.7   ; LF_HFE
  (14,0) -1.0   ; LF_KFE
  ; ... (其余 3 条腿类似)
}

; --- 跟踪代价权重 ---
[tracking_cost]
Q  ; 状态跟踪权重
{
  scaling = 1.0
  (0,0)  15.0   ; px tracking
  (1,0)  15.0   ; py tracking
  (2,0)  30.0   ; pz tracking (高度更重要!)
  (3,0)  10.0   ; Lx
  (4,0)  30.0   ; Ly (俯仰角动量, 防前倾)
  (5,0)  10.0   ; Lz
  (6,0)  500.0  ; x position (位置跟踪权重高)
  (7,0)  500.0  ; y position
  (8,0)  500.0  ; z position
  (9,0)  100.0  ; yaw
  (10,0) 200.0  ; pitch (防前倾, 权重高)
  (11,0) 200.0  ; roll  (防侧倾, 权重高)
  (12,0) 20.0   ; LF_HAA joint
  ; ... (其余关节, 20.0)
}
R  ; 输入代价权重
{
  scaling = 1e-3  ; 全局缩放因子
  (0,0)  1.0     ; LF_Fx (力的代价低 -> 鼓励施力)
  (1,0)  1.0     ; LF_Fy
  (2,0)  1.0     ; LF_Fz
  ; ... (其余脚力, 1.0)
  (12,0) 5000.0  ; LF_dq (关节速度代价高 -> 鼓励平滑!)
  ; ... (其余关节速度, 5000.0)
}

; --- 摆动腿轨迹配置 ---
[swing_trajectory_config]
liftOffVelocity = 0.2               ; 抬腿速度 (m/s)
touchDownVelocity = -0.4             ; 落脚速度 (m/s, 负号=向下)
swingHeight = 0.1                    ; 摆动高度 (m)
swingTimeScale = 0.15

; --- 摩擦锥约束 ---
[friction_cone]
frictionCoefficient = 0.5            ; 摩擦系数 mu
relaxedLogBarrierMu = 0.1            ; log barrier 参数
relaxedLogBarrierDelta = 5.0
```

> **⚠️ 关键调参提示**:
> 1. `Q(8,0) = 500.0` (z 位置) 远大于 `Q(12,0) = 20.0` (关节角)——保持站立高度比保持关节角更重要
> 2. `R(12,0) = 5000.0 * 1e-3 = 5.0` (关节速度) 远大于 `R(0,0) = 1.0 * 1e-3 = 0.001` (力)——鼓励用力来跟踪,而不是大幅运动
> 3. `swingHeight = 0.1` 是 10cm——平地足够,崎岖地形需增大到 0.15-0.2
> 4. `sqpIteration = 1` 是 RTI 模式——增大到 3-5 会更精确但可能超时

**练习 55.10**: 修改 task.info 中的 `frictionCoefficient` 从 0.5 改为 0.2(模拟冰面)。观察机器人行为变化——MPC 应该自动减小水平力并增大法向力。

---

### 55.11 OCS2 调试技巧 ⭐⭐

#### 55.11.1 常见问题与解决方案

| 问题 | 症状 | 原因 | 解决 |
|------|------|------|------|
| CppAD 编译失败 | `modelFolderCppAd` 下无 .so | URDF 路径错/Pinocchio 版本不匹配 | 检查 URDF, 设 `verboseCppAd = true` |
| QP Infeasible | MPC 返回失败 | 约束矛盾(如摩擦锥太紧+力太大) | 放松 `frictionCoefficient`, 检查 `mu` |
| MPC 求解超时 | 控制频率下降 | horizon 太长或约束太多 | 减小 `timeHorizon`, 减少约束 |
| 机器人漂移 | 位置误差累积 | 状态估计偏差 | 检查状态估计, 加位置误差积分 |
| 关节抖动 | 关节力矩震荡 | R 权重太小 | 增大 R 中关节速度权重 |
| 步态切换不稳 | 切换瞬间跌倒 | `phaseTransitionStanceTime` 太短 | 增大过渡时间到 0.5-0.6 |
| 首次运行慢 | 启动后等待 30s | CppAD 正在编译 .so | 正常,后续启动自动加载缓存 |

> **💡 调试顺序建议**: (1) 先看 MPC 是否求解成功(打印 return value), (2) 看求解时间是否在预算内, (3) 看轨迹是否合理(RViz 可视化), (4) 最后看力的分布。

#### 55.11.2 性能 profiling

```cpp
// 在 MPC 循环中添加计时
auto start = std::chrono::high_resolution_clock::now();
bool success = mpcMrtInterface_.advanceMpc();
auto end = std::chrono::high_resolution_clock::now();

double solveTime = std::chrono::duration<double, std::milli>(
    end - start).count();
RCLCPP_INFO(logger_, "MPC solve: %.2f ms, success: %d",
            solveTime, success);

// 进一步分解 (需要修改 SqpSolver):
// - 线性化时间: ~40% of total
// - HPIPM 求解: ~30% of total
// - Line search: ~20% of total
// - 其他 (内存拷贝等): ~10%
```

---

### 55.12 OCS2 与 legged_control 的关系 ⭐⭐

#### 55.12.1 legged_control 是什么?

legged_control(github.com/qiayuanl/legged_control)是**基于 OCS2 的开源四足控制框架**,被称为"可能是最好的开源四足 MPC 控制框架"。它在 OCS2 的基础上增加了:

```
OCS2 (ETH RSL)                    legged_control (qiayuan)
──────────────                    ─────────────────────────
OCP 定义                          继承 OCS2 的 OCP
SQP 求解器                        直接使用 OCS2 的 SqpSolver
双线程 MPC                        直接使用 MPC_MRT_Interface
ROS 封装 (示例级别)               + ros_control 硬件接口
                                  + WBC (Whole-Body Controller)
                                  + 状态估计 (KF + 接触检测)
                                  + Gazebo 仿真
                                  + Unitree Go1/A1 硬件驱动
                                  + sim2real 框架
```

#### 55.12.2 架构对比

```
OCS2 官方 legged_robot 示例:
┌──────────┐  ┌───────────┐
│ MPC Node │  │ Dummy Node│ (简单仿真,无物理引擎)
└──────────┘  └───────────┘
  纯 MPC 验证,无 WBC,无硬件接口

legged_control:
┌──────────┐  ┌──────────────────┐  ┌──────────────┐
│ MPC Node │->│ WBC Controller   │->│ 硬件接口     │
│ (OCS2)   │  │ (TSID, Ch53)     │  │ (ros_control)│
└──────────┘  └──────────────────┘  └──────────────┘
                     │
              ┌──────┴──────┐
              │ 状态估计     │
              │ (KF+接触)   │
              └─────────────┘
  完整的"感知-规划-控制-执行"闭环
```

#### 55.12.3 legged_control 的简化与增强

```
保留 OCS2 的:
  - Centroidal Model (FCD 和 SRBD)
  - SQP + HPIPM 求解器
  - 双线程 MPC 架构
  - ModeSchedule 步态管理

简化的:
  - 去掉了 Kino-centroidal 选项
  - 去掉了 SLQ/iLQR/IPM 求解器选项
  - 简化了配置文件结构 (单一 task.info)
  - ROS1 only (社区有 ROS2 移植版)

增加的:
  + WBC (加权全身控制器, TSID 风格)
  + 状态估计 (线性 KF + 接触检测)
  + Unitree SDK 硬件接口 (read()/write())
  + Gazebo 仿真环境 + 可视化
  + 社区衍生: legged_control_go2 等
```

> **💡 学习路径**: 先用 OCS2 官方示例理解 MPC 原理(本章),再用 legged_control 学习完整系统集成(Ch68)。OCS2 是"引擎",legged_control 是"整车"。

---

### 55.13 roboticsTemplateLibrary(RTL)与编译期优化 ⭐⭐

OCS2 内部有一个模板化的运动学层: `ocs2_robotic_tools` 的 RTL(roboticsTemplateLibrary)。

**核心思想**: 把运动学代码**完全模板化**,编译期展开:

```cpp
// EndEffectorKinematics<SCALAR_T>: 模板化的末端运动学
// 可实例化为:
EndEffectorKinematics<scalar_t>      // 运行时 double
EndEffectorKinematics<ad_scalar_t>   // CppAD 自动微分
EndEffectorKinematics<cg_scalar_t>   // CppADCodeGen 代码生成
```

这是 Ch47-48 模板化 Scalar 模式在 OCS2 层级的延续——从 Pinocchio 的 `SE3Tpl` 向上传播到整个运动学层。一份运动学代码,三种用途:运行时计算、自动微分、代码生成。

---

### 常见故障与排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| MPC 求解超时（单次求解耗时超过控制周期） | 预测 horizon 过长导致 OCP 规模膨胀；模型复杂度过高（如包含全身惯量矩阵的解析导数） | 1. 用 `solver.getBenchmarkingInfo()` 定位瓶颈在 rollout 还是 QP 子问题 2. 缩短 horizon 或降低离散化密度 3. 对导数启用 CppADCodeGen 代码生成替代在线 AD |
| 双线程 BufferedValue 数据竞争（MRT 读取到不一致的状态/力矩） | 自定义了 MPC-MRT 通信数据结构但未正确使用 `BufferedValue` 的 `updateFromBuffer()` / `setBuffer()` 接口；或在 buffer 读写间插入了非原子操作 | 1. 确认所有跨线程数据均通过 `BufferedValue` 传递 2. 检查 MRT 侧是否在每个控制周期起始处调用 `updateFromBuffer()` 3. 用 ThreadSanitizer (`-fsanitize=thread`) 编译并复现 |
| SQP 不收敛（迭代次数用满仍未满足 KKT 容差） | 初始轨迹猜测与当前状态偏差过大；约束不可行（如接触时序与实际步态不匹配导致运动学约束矛盾） | 1. 打印每次迭代的 KKT 残差，判断是发散还是振荡 2. 检查 `ReferenceManager` 的模式时序是否与实际步态对齐 3. 用上一次成功解做暖启动而非零初始化 |
| MRT 节点查询延迟过大（`evaluatePolicy()` 返回的力矩滞后于当前状态） | MPC 线程频率过低或求解时间波动大，导致 MRT 线性外推跨度过长；ROS 通信延迟叠加 | 1. 监控 `MPC_MRT_Interface` 的策略时间戳与当前时间的差值 2. 确认 MPC 线程频率 >= 50 Hz（Go2 典型配置 100 Hz） 3. 检查是否有其他高优先级线程抢占 MPC 线程的 CPU 核心 |
| 模型切换（mode transition）时力矩跳变 | 接触/离地切换瞬间，约束集突变导致 QP 解不连续；摆动腿轨迹的初始/终端条件与支撑相不匹配 | 1. 在 mode transition 前后各 50ms 录制力矩曲线，确认跳变幅度 2. 检查 `GaitSchedule` 的切换时刻是否有重叠或间隙 3. 对摆动腿轨迹添加起止速度平滑约束（`SwingTrajectoryPlanner` 的 liftoff/touchdown 速度配置） |

---

## 55.14 本章小结 ⭐

#### 核心概念回顾

```
本章知识图谱:

OCS2 定位
├── Optimal Control for Switched Systems
├── ETH RSL 出品, BSD 3-Clause
├── 对比: OCS2 / Crocoddyl / acados / CasADi
└── 用户: ANYmal / Go2 / HyQ / 轮足

五层架构
├── L1: Core (Types, Rollout, ReferenceManager)
├── L2: OCP (Dynamics, Cost, Constraint) [用户核心工作]
├── L3: Solver (SQP + HPIPM / DDP / IPM) [可插拔]
├── L4: MPC_MRT_Interface (双线程) [部署核心]
└── L5: ROS Wrapper (MPC_Node + MRT_Node)

SQP 算法
├── Multiple Shooting 格式 -> block-sparse QP
├── 线性化 + QP 子问题 -> HPIPM 求解
├── SQP-RTI: 只做 1 次迭代 (sqpIteration = 1)
├── Armijo Line Search (c=1e-4, beta=0.5)
├── 收缩性理论保证 (Diehl et al. 2005)
└── 多线程线性化 (nThreads 并行)

双线程架构
├── MPC 线程 (~50-100Hz): 异步求解, 非实时
├── MRT 线程 (~400-1000Hz): 实时查询, SCHED_FIFO
├── BufferedValue / Triple Buffer: 无锁通信
├── 失败降级: MRT 用旧 policy 继续
└── 延迟: MRT 到力矩 < 2ms (确定性)

模型选型
├── Full Centroidal (centroidalModelType = 0): 默认
├── SRBD (centroidalModelType = 1): 小型四足简化
└── Kino-centroidal: 进阶, 关节加速度为输入
```

#### 自检问题

1. **OCS2 的"Switched Systems"抽象具体体现在哪些代码组件中?** (ModeSchedule, jumpMap, isActive, ZeroForceConstraint/ZeroVelocityConstraint 按 mode 切换)

2. **SQP-RTI 为什么只需 1 次 SQP 迭代?** (热启动使得相邻问题的解很接近 + 收缩性保证每步都向最优收缩 + MPC 频率高所以问题变化小)

3. **Triple Buffer 和 Mutex 的本质差异是什么?** (无锁 vs 有锁; Triple Buffer 提供确定性延迟, Mutex 可能导致优先级反转和不确定阻塞)

4. **task.info 中 `sqpIteration = 1` 改成 5 会有什么影响?** (求解更精确但更慢;可能错过 MPC 周期导致 MRT 使用旧 policy;对稳定行走通常没有明显改善)

5. **为什么 R 矩阵中关节速度权重(5000)远大于力权重(1)?** (鼓励 MPC 通过调整力来跟踪参考,而不是通过大幅关节运动;减小关节速度可以减少抖动和能耗)

---

## 累积项目:OCS2 MPC 集成

> **项目目标**: 在你的四足控制器项目中集成 OCS2 MPC,替换之前的 PD/WBC 控制器,实现完整的 MPC+WBC 闭环。

#### 阶段 1: OCS2 环境搭建 (4-6 小时)

```bash
# === ROS1 路线（OCS2 main 分支，推荐 Ubuntu 20.04 + Noetic） ===
cd ~/catkin_ws/src
git clone https://github.com/leggedrobotics/ocs2.git
cd ocs2 && git checkout main
# Pinocchio/hpp-fcl 通过 robotpkg apt 源安装（非 ros-* 包）
# 参考: https://stack-of-tasks.github.io/pinocchio/download.html

# 安装 HPIPM 和 BLASFEO (如果未通过 apt 安装)
git clone https://github.com/giaf/blasfeo.git
cd blasfeo && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DTARGET=X64_AUTOMATIC && make -j$(nproc) && sudo make install
cd ../..
git clone https://github.com/giaf/hpipm.git
cd hpipm && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j$(nproc) && sudo make install

# 编译 OCS2 (ROS1 使用 catkin)
cd ~/catkin_ws
catkin build ocs2_legged_robot_ros

# === ROS2 路线（ocs2 ros2 分支，目标 Ubuntu 24.04 + Jazzy） ===
# mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
# git clone -b ros2 https://github.com/leggedrobotics/ocs2.git
# sudo apt install ros-jazzy-pinocchio ros-jazzy-hpp-fcl
# cd ~/colcon_ws && colcon build
```

#### 阶段 2: 运行官方示例 (2-3 小时)

```bash
# ROS1 main 分支：启动 MPC + Dummy 仿真
roslaunch ocs2_legged_robot_ros legged_robot_sqp.launch

# 在另一个终端发送速度命令
rostopic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}}"

# 验收: RViz 中能看到四足行走的预测轨迹和接触力箭头
```

如果你使用的是 `ros2` 分支，启动命令和消息类型需要切换到 ROS2 形式，例如 `ros2 launch ... .launch.py` 和 `geometry_msgs/msg/Twist`。不要把上面的 ROS1 main 分支安装步骤和 ROS2 运行命令混用。

#### 阶段 3: 适配你的机器人 (8-12 小时)

```
需要修改的文件:
1. URDF: 替换为你的机器人模型
2. frame_declaration.info: 指定基座链接名和 4 个末端链接名
3. task.info: 调整质量/高度/关节角初始值/代价权重
4. gait.info: 定义步态参数 (步态周期, stance/swing 比例)
5. reference.info: 默认参考位姿
```

#### 阶段 4: 接入 WBC (Ch53 的延续) (6-8 小时)

```cpp
// 控制器主循环 (伪代码)
void controlLoop() {
  // 1. MRT 查询 MPC 输出
  mrt_.updatePolicy();
  mrt_.evaluatePolicy(currentTime_, currentState_,
                      xDesired_, uDesired_, mode_);

  // 2. WBC: 把 MPC 的 Centroidal 输出分解为关节力矩
  auto tau = wbc_.solve(xDesired_, uDesired_,
                         currentState_, mode_);

  // 3. 发送关节力矩
  hardware_.sendTorques(tau);
}
```

---

### 实战练习

#### [A 型 - 基础] 练习 55.1: OCS2 环境搭建 ⭐

从头搭建 OCS2 + ocs2_legged_robot 的开发环境。编译并运行示例,在 RViz 中看到四足机器人行走。

**验收标准**: RViz 中能看到预测轨迹和接触力箭头,MPC 控制台无 error。

#### [A 型 - 中等] 练习 55.2: 改变 MPC Horizon ⭐⭐

修改 `task.info` 中 `timeHorizon` 为 0.3 / 1.0 / 2.0,记录:
1. 单次 MPC 求解时间
2. 行走稳定性 (是否摔倒)
3. 对扰动的鲁棒性

**提交**: 三组参数的对比表和分析。

#### [A 型 - 进阶] 练习 55.3: 添加自定义 Cost ⭐⭐⭐

在 OCP 中添加一个 cost: "让基座高度保持在 0.5m"。验证运行效果。

#### [B 型 - 源码阅读] 练习 55.4: 精读 SqpSolver ⭐⭐⭐

精读 `ocs2_sqp/src/SqpSolver.cpp`(约 800 行),回答:
1. `runImpl()` 的主循环结构是什么?
2. 线性化如何多线程并行?
3. Armijo line search 的参数值是多少?
4. 收敛判定的数学含义?

#### [B 型 - 双线程架构] 练习 55.5: 精读 MPC_MRT_Interface ⭐⭐⭐

精读 `ocs2_mpc/src/MPC_MRT_Interface.cpp`:
1. `advanceMpc()` 的调用链?
2. `evaluatePolicy()` 的时间插值实现?
3. BufferedValue 的 swap 是否真正无锁?
4. MPC 失败时 MRT 的行为?

#### [B 型 - 对比精读] 练习 55.6: OCS2 vs legged_control ⭐⭐

对比两个代码库的主循环:
- OCS2: `LeggedRobotMpcNode.cpp`
- legged_control: `LeggedController.cpp`

列出差异、简化和增强。

#### [思考题] 练习 55.7: 切换系统抽象的通用性 ⭐⭐

"切换系统"的抽象对以下场景是否有意义?
1. 机械臂(无离散切换)
2. 无人机(飞行模式切换)
3. 轮足复合机器人(轮 vs 腿模式)
4. 人形机器人(步态 + 抓取)

#### [思考题] 练习 55.8: SQP vs DDP 的工业选型 ⭐⭐⭐

列出支持 SQP 和支持 DDP 的论据。如果你要做新的腿足 MPC 框架,选哪个?为什么?

---

### 研究前沿与论文阅读

#### 必读论文

1. **Farshidian F., Neunert M., Winkler A. W., et al. (2017)** "An efficient optimal planning and control framework for quadrupedal locomotion" — ICRA. **OCS2 的前身**。

2. **Grandia R., Farshidian F., et al. (2019)** "Frequency-aware model predictive control" — RA-L. OCS2 应用于 ANYmal。

3. **Grandia R., Jenelten F., Yang S., Farshidian F., Hutter M. (2023)** "Perceptive locomotion through nonlinear model predictive control" — T-RO. **OCS2 现役生产版本**,100Hz NMPC + 4 核并行 + 实时感知。博士必读。

4. **Sleiman J.-P., Farshidian F., et al. (2021)** "A unified MPC framework for whole-body dynamic locomotion and manipulation" — RA-L. OCS2 扩展到 loco-manipulation(与 Ch73 关联)。

5. **Diehl M., Bock H. G., Schloder J. P. (2005)** "A real-time iteration scheme for nonlinear optimization in optimal feedback control" — SIAM J. Control Optim. **RTI 的原始论文**。

#### 近期进展

6. **Jenelten F., et al. (2022)** "Perceptive locomotion in rough terrain" — RA-L. OCS2 与感知集成。

7. **Stark S., et al. (2024)** "Benchmarking Different QP Formulations and Solvers for Dynamic Locomotion" — QP 求解器在腿足场景的最新基准。

8. **Frison G., Diehl M. (2020)** "HPIPM: a high-performance QP framework for model predictive control" — IFAC. HPIPM 论文。

#### 开放研究问题

- **OCS2 on GPU**: HPIPM 是 CPU-only。能否用 GPU 加速 MPC 求解? 参考 cuHPIPM。
- **分布式 OCS2**: 多机器人协同 MPC(如双四足抬重物)。
- **OCS2 + RL**: 用 NN 预测初始轨迹(热启动)、学习代价权重(逆 RL)、MPC-Net(OCS2 内置但实验性)。

---

## 延伸阅读

| 主题 | 资源 | 类型 |
|------|------|------|
| OCS2 官方文档 | leggedrobotics.github.io/ocs2 | 文档 |
| OCS2 GitHub | github.com/leggedrobotics/ocs2 | 代码 |
| legged_control | github.com/qiayuanl/legged_control | 代码 |
| quadruped_ros2_control | github.com/legubiao/quadruped_ros2_control | 代码(ROS2) |
| HPIPM 论文 | Frison & Diehl, IFAC 2020 | 论文 |
| RTI 理论 | Diehl et al., SIAM 2005 | 论文 |
| Quadruped-PyMPC | github.com/iit-DLSLab/Quadruped-PyMPC | 代码(Python) |

---

### 预计学习时间

**2 周(40-50 小时)**:

| 天 | 内容 | 时间 |
|----|------|------|
| 1-2 | 55.1-55.2: OCS2 定位与五层架构 | 6-8h |
| 3-4 | 55.3-55.4: OCP 抽象与 SQP 算法 (含 RTI 理论) | 6-8h |
| 5 | 55.5-55.6: HPIPM 集成与模型选型 | 4-5h |
| 6-7 | 55.7: 双线程 MPC 架构 (核心,含 Triple Buffer) | 6-8h |
| 8 | 55.8-55.9: ReferenceManager + ROS Wrapper | 4-5h |
| 9-10 | 55.10-55.12: task.info + 调试 + legged_control | 6-8h |
| 11-14 | 练习 + 累积项目 + 论文阅读 | 8-10h |

---
