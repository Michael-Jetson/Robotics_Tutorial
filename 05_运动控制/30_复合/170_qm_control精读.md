# 第87章：qm_control 源码精读——OCS2 NMPC + 混合 WBC 四分支

| 元信息 | 值 |
|---|---|
| 难度 | ⭐⭐⭐⭐（OCS2、Pinocchio、WBC、ros_control、状态估计联读） |
| 预计时间 | 2 周，35-45 小时 |
| 前置依赖 | 复合/160_四足臂动力学概览、复合/20_浮动基座臂统一动力学、复合/30_多模态MPC、足式/90_WBC分层优化与TSID、足式/110_OCS2完整栈与双线程MPC |
| 项目定位 | 开源四足+臂 MPC+WBC 工程主轴 |
| 阅读方式 | 先看数据流，再看 OCP，再看 WBC，最后看估计与分支差异 |

---

## 87.0 前置自测

开始读 qm_control 之前，请先回答下面问题。

如果答不出来，不建议直接跳源码。

| # | 问题 | 对应前置 |
|---|---|---|
| 1 | OCS2 中 `OptimalControlProblem` 的 cost、constraint、dynamics 分别在哪里注册？ | 足式/110_OCS2完整栈与双线程MPC |
| 2 | 四足臂统一动力学方程的右端有哪三类广义力？ | 复合/160_四足臂动力学概览 |
| 3 | WBC 中为什么接触不滑约束写成 $J_c\ddot{q}+\dot{J}_c\dot{q}=0$？ | 足式/90_WBC分层优化与TSID |
| 4 | SE(3) 末端误差如何由 $T_{ee}$ 和 $T_{ref}$ 构造？ | 复合/30_多模态MPC |
| 5 | MPC 50 Hz、WBC 500 Hz 时，中间为什么需要时间插值和线程安全数据交换？ | 足式/90_WBC分层优化与TSID |

**自测标准**：

| 正确题数 | 建议 |
|---|---|
| 5 | 可以按本章顺序精读源码 |
| 3-4 | 先读架构，遇到公式再回查 |
| 0-2 | 先补 OCS2、WBC、SE(3) 误差和统一动力学 |

---

## 87.1 本章目标

本章要把 qm_control 从"一个能跑的仓库"拆解成可迁移的工程模式。

学完后你应该能完成六件事。

1. 画出 `ros_control` 控制器、OCS2 interface、WBC、估计器和仿真之间的数据流。
2. 说清楚 qm_control 如何从 `legged_robot` 扩展到四足+臂。
3. 写出 OCS2 状态、输入、代价、约束和 mode schedule 的布局。
4. 手动组装 WBC 的核心任务：floating base、摩擦锥、摆动腿、base tracking、EE tracking、力矩限幅。
5. 区分 `main`、`feature-force`、`feature-compliance`、`feature-real` 四条分支的技术重点。
6. 设计从 A1+臂迁移到 Go2+Z1 或 Go2+ARX5 的修改清单。

---

## 87.2 阅读路线：先数据流，后文件名 ⭐

这一节解决源码阅读顺序问题。

新手读开源控制仓库常犯一个错误：一上来按文件树逐个打开。

结果很快被类名、模板、配置和 launch 文件淹没。

读 qm_control 应先抓住运行时数据流。

```text
Gazebo / Hardware
    │ joint state, imu, contact
    ▼
qm_estimation
    │ estimated base state
    ▼
qm_controllers::QMController
    │ current state + command
    ├───────────────► qm_interface / OCS2 MPC
    │                 │ optimized state/input trajectory
    │                 ▼
    └──────────────► qm_wbc
                      │ joint torque / position command
                      ▼
                 ros_control hardware interface
```

这个数据流比文件树更重要。

文件树只是这个数据流的静态展开。

### 87.2.1 模块角色总表

| 模块 | 主要职责 | 读源码时要找什么 |
|---|---|---|
| `qm_description` | 统一 URDF、mesh、关节命名 | base、leg、arm 的连接方式 |
| `qm_interface` | OCS2 OCP 构造 | state/input layout、cost、constraint、reference |
| `qm_wbc` | 全身 QP | 任务矩阵、优先级、求解器调用 |
| `qm_estimation` | 基座状态估计 | 接触触发更新、IMU/腿运动学融合 |
| `qm_controllers` | ros_control 插件入口 | update 回调、MPC/WBC 调用频率 |
| `qm_common` | 公共数据结构 | 关节索引、常量、消息类型 |
| `qm_gazebo` | 仿真环境 | 插件、场景、控制接口 |

### 87.2.2 推荐阅读顺序

| 顺序 | 文件/目录 | 目标 |
|---:|---|---|
| 1 | `qm_description` | 确认机器人拓扑和 frame 命名 |
| 2 | `qm_controllers/src/QMController.cpp` | 找主循环入口 |
| 3 | `qm_interface/src/QMInterface.cpp` | 找 OCP 注册入口 |
| 4 | `qm_interface/src/cost/EndEffectorTrackingCost.cpp` | 看末端代价如何计算 |
| 5 | `qm_interface/src/constraint/FrictionConeConstraint.cpp` | 看摩擦锥如何写成约束 |
| 6 | `qm_wbc/src/WbcBase.cpp` | 看 WBC 任务矩阵如何生成 |
| 7 | `qm_wbc/src/HierarchicalWbc.cpp` | 看层级 QP 如何求解 |
| 8 | `qm_estimation/src/LinearKalmanFilter.cpp` | 看状态估计如何为控制提供基座状态 |
| 9 | 四个分支差异 | 看力控、柔顺、实机保护怎么扩展 |

### 87.2.3 本章不把源码当黑盒

本章每节都按四步讲。

第一步说明这个模块解决什么问题。

第二步写出数学或数据接口。

第三步给出接近真实 C++ 的代码框架。

第四步列出常见失败模式。

这样读完后，你不只是能复现仓库。

你还能把同样架构迁移到新的四足臂平台。

### ⚠️ 常见陷阱

> ⚠️ **源码阅读陷阱：从 launch 文件一路追到所有依赖**
>
> 结果通常是读了很多启动配置，却没理解控制器每个周期做什么。
>
> 正确做法：先找到 `QMController::update()` 之类的周期入口，再向上追参数，向下追 MPC/WBC。

> 💡 **概念误区：认为 qm_control 等于 OCS2 示例的简单合并**
>
> 它不是把 `legged_robot` 和 `mobile_manipulator` 两个示例粘起来。
>
> 四足臂需要重新定义状态、输入、接触模式、末端代价、WBC 任务和估计接口。

> 🧠 **思维陷阱：只记文件名，不记接口**
>
> 文件名会随项目变化。
>
> 真正可迁移的是：状态布局、输入布局、参考管理、任务优先级和控制频率。

### 练习 87.2

| # | 练习 | 难度 |
|---|---|---|
| 1 | 根据本节数据流，画出 qm_control 的运行时调用图，并标出每条边传递的数据。 | ⭐ |
| 2 | 阅读 `QMController.cpp`，找出 MPC、WBC、低层命令分别在什么条件下更新。 | ⭐⭐ |
| 3 | 不看文件树，用自己的话解释 `qm_interface` 和 `qm_wbc` 的边界。 | ⭐⭐ |

---

## 87.3 URDF 与统一模型：`qm_description` 的真正作用 ⭐⭐

这一节解释为什么描述文件不是普通资产。

在四足臂控制中，URDF 的拓扑、惯量和 frame 命名会直接决定 OCP 和 WBC 的矩阵维度。

如果 URDF 错了，后面调权重没有意义。

### 87.3.1 统一拓扑

四足臂 URDF 的核心拓扑是：

```text
world
  └─ floating base
      └─ base_link
          ├─ FL_hip → FL_thigh → FL_calf → FL_foot
          ├─ FR_hip → FR_thigh → FR_calf → FR_foot
          ├─ RL_hip → RL_thigh → RL_calf → RL_foot
          ├─ RR_hip → RR_thigh → RR_calf → RR_foot
          └─ arm_mount → arm_joint1 → ... → arm_joint6 → ee_link
```

臂通常通过 fixed joint 安装在机身上。

这看似只是几何连接。

但在动力学中，它意味着臂的质量、惯量和末端雅可比都属于同一个多体系统。

### 87.3.2 关节索引和 frame 命名

控制代码最怕"名字差一点"。

例如：

| 期望名字 | 实际名字差异 | 后果 |
|---|---|---|
| `FL_foot` | `FL_foot_link` | frame id 查找失败 |
| `ee_link` | `tool0` | 末端代价计算的是错误 frame |
| `arm_joint_1` | `joint1` | 关节索引错位 |
| `base_link` | `trunk` | 状态估计坐标系不一致 |

迁移平台时，第一步不是改控制参数。

第一步是建立命名映射表。

### 87.3.3 惯量参数比视觉 mesh 更重要

视觉 mesh 影响显示。

碰撞 mesh 影响避障。

惯量参数影响动力学和 WBC。

| URDF 字段 | 影响 |
|---|---|
| `mass` | CoM、重力补偿、接触力分配 |
| `origin` in inertial | 质心位置，影响角动量 |
| `ixx, iyy, izz` | 姿态加速度和臂惯性耦合 |
| joint limit | OCP 和 WBC 的可行域 |
| transmission | ros_control 与硬件接口 |

如果臂惯量过小，控制器会低估臂摆动对基座的扰动。

如果臂惯量过大，控制器会过度保守，末端响应变慢。

### 87.3.4 模型加载检查代码

```cpp
struct ModelAudit {
  pinocchio::Model model;
  pinocchio::Data data;

  explicit ModelAudit(const std::string& urdf) : model(), data(model) {
    pinocchio::urdf::buildModel(
        urdf, pinocchio::JointModelFreeFlyer(), model);
    data = pinocchio::Data(model);
  }

  void printBasicInfo() const {
    std::cout << "nq = " << model.nq << "\n";
    std::cout << "nv = " << model.nv << "\n";
    std::cout << "njoints = " << model.njoints << "\n";
    std::cout << "nframes = " << model.nframes << "\n";
  }

  void checkFrame(const std::string& name) const {
    if (!model.existFrame(name)) {
      throw std::runtime_error("缺少 frame: " + name);
    }
  }

  void checkAllFrames() const {
    checkFrame("FL_foot");
    checkFrame("FR_foot");
    checkFrame("RL_foot");
    checkFrame("RR_foot");
    checkFrame("ee_link");
  }
};
```

### 87.3.5 从 A1 换到 Go2 的修改清单

| 类别 | 要改什么 | 验证方法 |
|---|---|---|
| URDF | base、leg、arm 连接和惯量 | `check_urdf` + Pinocchio 加载 |
| frame 名 | 四足端和末端 frame | 打印 frame id |
| 关节顺序 | 腿和臂关节索引 | 与硬件 SDK/仿真 qpos 对齐 |
| 限位 | joint position/velocity/torque | 打印配置表 |
| 控制增益 | PD/WBC 权重 | 先静态站立 |
| 初始姿态 | default joint pose | CoM 投影在支撑多边形内 |
| 碰撞模型 | 自碰撞 pair | 可视化最小距离 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：仿真 qpos 顺序与 Pinocchio 关节顺序不一致**
>
> 现象：机器人一启动就扭曲，或者某条腿响应另一个关节命令。
>
> 根本原因：仿真器、URDF、控制数组使用不同顺序。
>
> 正确做法：建立显式索引映射，并写单关节脉冲测试。

> 💡 **概念误区：fixed joint 不影响动力学**
>
> fixed joint 没有自由度，但它连接的 link 质量和惯量仍然进入整机动力学。
>
> 臂安装座的位姿也会改变全身 CoM 和 CMM。

> 🧠 **思维陷阱：先调控制器再查模型**
>
> 模型错时，控制器调参只会掩盖问题。
>
> 正确顺序是：模型加载、质量检查、CoM 检查、Jacobian 检查、动力学检查，然后才是控制。

### 练习 87.3

| # | 练习 | 难度 |
|---|---|---|
| 1 | 写脚本打印所有 joint 和 frame，建立 qm_control 到 Go2+Z1 的命名映射表。 | ⭐ |
| 2 | 修改臂第 2 link 的质量为原来的 2 倍，比较 CoM 和 $\mathbf{A}_{G,a}$ 的变化。 | ⭐⭐ |
| 3 | 设计一个模型审计清单，要求在 WBC 启动前自动检查 frame、质量、限位和四元数归一化。 | ⭐⭐⭐ |

---

## 87.4 OCS2 接口：`qm_interface` 如何构造 OCP ⭐⭐⭐

这一节进入 qm_control 的规划层。

`qm_interface` 的核心任务是把机器人、任务和约束翻译成 OCS2 能求解的最优控制问题。

回顾复合/30：MPC 的本质不是一个求解器，而是一组状态、输入、动力学、代价和约束。

### 87.4.1 状态布局

qm_control 风格的 centroidal 状态通常可写成：

$$
\mathbf{x}
=
\begin{bmatrix}
\mathbf{h}_G \\
\mathbf{q}_b \\
\mathbf{q}_{\ell} \\
\mathbf{q}_a
\end{bmatrix}
$$

一个常见维度为：

| 子块 | 维度 | 说明 |
|---|---:|---|
| $\mathbf{h}_G$ | 6 | 质心角动量 + 线动量 |
| $\mathbf{q}_b$ | 6 或 7 | OCS2 内部常用最小坐标表示基座位姿 |
| $\mathbf{q}_{\ell}$ | 12 | 腿关节 |
| $\mathbf{q}_a$ | 6 | 臂关节 |
| 合计 | 30 左右 | 依具体表示略有差异 |

为什么不直接用 $(q,v)$？

因为 MPC 层更关心质心动量和广义坐标。

速度可以通过 centroidal mapping 或输入近似恢复。

这与纯四足 OCS2 `legged_robot` 的思想一致，但多了臂关节和末端任务。

### 87.4.2 输入布局

输入可写成：

$$
\mathbf{u}
=
\begin{bmatrix}
\boldsymbol{\lambda}_{foot} \\
\dot{\mathbf{q}}_{\ell}^{cmd} \\
\dot{\mathbf{q}}_a^{cmd}
\end{bmatrix}
$$

常见维度：

| 子块 | 维度 | 说明 |
|---|---:|---|
| 足端接触力 | 12 | 4 足 × 3D |
| 腿关节速度 | 12 | 运动学参考 |
| 臂关节速度 | 6 | 末端任务相关 |
| 合计 | 30 | 与调研中的布局一致 |

有些实现会把臂输入解释为速度、力矩或低层参考。

读源码时必须看清楚每个输入子块在 flow map 和 WBC 中如何使用。

为什么输入是接触力和关节速度，而不是关节力矩？这个设计选择反映了 centroidal MPC 的核心思想：MPC 层在"质心动量空间"规划，而不是在"关节空间"规划。接触力 $\lambda_{foot}$ 是质心动量变化的直接原因（回顾本章 86.3：$\dot{\mathbf{h}}_G$ 由外力决定）。如果 MPC 输入是关节力矩，它需要先经过全身逆动力学才能知道对质心的影响——这等于要求 MPC 在每一步内部求解一个 WBC，计算量会膨胀一个数量级。选择接触力作为输入让 MPC 可以直接推演质心轨迹，关节力矩的计算则留给下游的 WBC。

如果末端与环境主动接触，还要先约定 $f_{ee}$ 的身份。

一种做法是把 $f_{ee}$ 当作优化输入，它与足端接触力一起进入 OCP。

另一种做法是把 $f_{ee}$ 当作已知外部扰动，它不属于 $\mathbf{u}$，而是由力传感器、阻抗层或任务规划器给定。

两种做法不能在同一个公式里混用，否则 MPC 会一边把末端力当决策变量，一边又把它当外部常量。

### 87.4.3 Flow map

质心动力学部分：

$$
\dot{\mathbf{h}}_G
=
\begin{bmatrix}
\sum_i(\mathbf{p}_{c_i}-\mathbf{p}_G)\times\lambda_i
+(\mathbf{p}_{ee}-\mathbf{p}_G)\times f_{ee}
+\tau_{ee} \\
m\mathbf{g}+\sum_i\lambda_i+f_{ee}
\end{bmatrix}
$$

这里的 $f_{ee}$ 表示"环境施加在机器人末端上的线力"，$\tau_{ee}$ 表示环境施加在末端上的纯力矩或等效扭矩。

如果代码中保存的是"机器人施加给环境的力"，代入上式前必须取反。

角动量项不能只写 $\tau_{ee}$，因为末端线力只要不穿过质心，就会通过力臂 $(\mathbf{p}_{ee}-\mathbf{p}_G)$ 产生绕质心的力矩。

这对推、拉、托举的影响不同：

| 操作 | 对角动量的典型影响 | 工程含义 |
|---|---|---|
| 高处推墙或推门 | 水平反作用力与末端高度形成俯仰/横滚力矩 | 机身可能后仰、前倾或侧倾，需要足底力矩和步态补偿 |
| 拉把手或拖物体 | 力方向与推相反，角动量变化符号也相反 | 同一末端位置下，推和拉会要求不同的支撑力分配 |
| 托举物体 | 竖直力若不穿过质心，会产生横滚/俯仰力矩；若穿过质心，角动量贡献接近零 | 搬运时不仅要看重量，还要看物体相对质心的水平偏置 |

因此检查 flow map 时，要同时看线动量和角动量。

线动量只关心合力，角动量还关心每个力的作用点。

配置积分部分：

$$
\dot{\mathbf{q}}_{\ell}
=
\dot{\mathbf{q}}_{\ell}^{cmd}
$$

$$
\dot{\mathbf{q}}_{a}
=
\dot{\mathbf{q}}_{a}^{cmd}
$$

基座速度则由 centroidal 模型和关节速度关系确定。

### 87.4.4 代价项

总代价通常分成：

$$
\ell
=
\ell_{base}
+
\ell_{momentum}
+
\ell_{ee}
+
\ell_{joint}
+
\ell_{input}
$$

| 代价 | 数学形式 | 作用 |
|---|---|---|
| base tracking | $\|q_b-q_b^{ref}\|_{Q_b}^{2}$ | 保持机身高度和姿态 |
| momentum tracking | $\|h_G-h_G^{ref}\|_{Q_h}^{2}$ | 跟踪速度和角动量 |
| EE tracking | $\|\log(T_{ee}^{-1}T_{ref})\|_{Q_{ee}}^{2}$ | 当前末端局部坐标中的 6D 任务 |
| joint regularization | $\|q_j-q_j^{nom}\|_{Q_j}^{2}$ | 避免奇异和限位 |
| input regularization | $\|u-u^{ref}\|_{R}^{2}$ | 平滑接触力和速度 |

### 87.4.5 约束项

| 约束 | 类型 | 物理含义 |
|---|---|---|
| 摆动脚力为零 | 等式 | 非接触脚不能施加地面反力 |
| 支撑脚摩擦锥 | 不等式 | 接触力必须可实现 |
| 关节限位 | 不等式 | 避免越界 |
| 速度限位 | 不等式 | 符合执行器能力 |
| 自碰撞距离 | 不等式/软约束 | 避免臂撞腿和身体 |
| EE 任务 | 代价或约束 | 按任务阶段切换 |

### 87.4.6 OCP 构造伪代码

```cpp
class QMInterface {
 public:
  void setupOptimalControlProblem() {
    // 1. 加载模型和配置。
    loadRobotModel();
    loadTaskInfo();
    loadReferenceManager();

    // 2. 注册动力学。
    problem_.dynamicsPtr = std::make_unique<QuadArmCentroidalDynamics>(
        modelInfo_, pinocchioInterface_);

    // 3. 注册代价项。
    problem_.costPtr->add("baseTracking", makeBaseTrackingCost());
    problem_.costPtr->add("momentumTracking", makeMomentumCost());
    problem_.costPtr->add("endEffectorTracking", makeEndEffectorCost());
    problem_.costPtr->add("jointRegularization", makeJointRegularization());
    problem_.costPtr->add("inputRegularization", makeInputRegularization());

    // 4. 注册约束项。
    problem_.inequalityConstraintPtr->add(
        "frictionCone", makeFrictionConeConstraint());
    problem_.inequalityConstraintPtr->add(
        "jointLimits", makeJointLimitConstraint());
    problem_.equalityConstraintPtr->add(
        "zeroForceSwingFoot", makeSwingFootForceConstraint());

    // 5. 准备 MPC 求解器。
    setupMpcSolver(problem_);
  }
};
```

这段伪代码的关键不是函数名。

关键是每个 term 都必须知道状态和输入布局。

如果状态布局改了，所有 cost/constraint 的索引函数都必须一起改。

### 87.4.7 索引函数是工程稳定性的核心

> **本质洞察**：源码精读的核心不是记住 `QMInterface.cpp` 或 `WbcBase.cpp` 这些文件名，而是找出跨模块不变量：状态布局、输入布局、坐标系约定、接触模式和频率边界。文件可以重命名，分支可以重构，但这些不变量一旦错位，MPC、WBC、估计器和硬件接口会同时失真。

```cpp
struct StateInputLayout {
  int h_start = 0;
  int h_dim = 6;
  int base_start = 6;
  int base_dim = 6;
  int leg_start = 12;
  int leg_dim = 12;
  int arm_start = 24;
  int arm_dim = 6;

  Eigen::VectorXd getMomentum(const Eigen::VectorXd& x) const {
    return x.segment(h_start, h_dim);
  }

  Eigen::VectorXd getArmJoints(const Eigen::VectorXd& x) const {
    return x.segment(arm_start, arm_dim);
  }
};
```

不要在 cost 里到处写魔法数字。

四足臂项目维度一改，魔法数字会让调试时间成倍增加。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：状态布局和输入布局散落在多个文件**
>
> 现象：改臂自由度后编译通过，但运行时矩阵维度错或行为异常。
>
> 根本原因：多个 cost/constraint 手写了相同索引。
>
> 正确做法：集中维护 layout，并为每个子块写单元测试。

> 💡 **概念误区：MPC 输入就是最终电机力矩**
>
> 在 centroidal MPC 中，输入常是接触力和关节速度参考。
>
> 最终电机力矩由 WBC 根据全身动力学求解。
>
> 这就是 MPC 和 WBC 分层的接口。

> 🧠 **思维陷阱：把末端任务全部放到 MPC 中**
>
> 低频 MPC 不适合处理所有高频接触细节。
>
> 精细阻抗、接触冲击和力矩限幅通常需要 WBC 或低层控制处理。

### 练习 87.4

| # | 练习 | 难度 |
|---|---|---|
| 1 | 在纸上写出 qm_control 的状态和输入布局，并标注每个 cost 会访问哪些子块。 | ⭐⭐ |
| 2 | 实现 `StateInputLayout`，写测试验证随机向量切片后能正确还原。 | ⭐⭐ |
| 3 | 设计一个新增 EE wrench tracking cost，说明它需要访问哪些状态、输入和 Pinocchio 量。 | ⭐⭐⭐ |

---

## 87.5 末端代价：`EndEffectorTrackingCost` 的数学与实现 ⭐⭐⭐

这一节讲 qm_control 中最能体现四足臂增量的代价项。

纯四足没有 EE tracking。

底盘臂移动操作有 EE tracking，但底盘通常不是浮动基座接触系统。

四足臂的 EE tracking 同时依赖基座、腿和臂。

### 87.5.1 SE(3) 误差

末端当前位姿：

$$
T_{ee}(q)\in SE(3)
$$

参考位姿：

$$
T_{ref}(t)\in SE(3)
$$

误差可写为：

$$
\xi
=
\log(T_{ee}^{-1}T_{ref})^{\vee}
\in\mathbb{R}^{6}
$$

本章统一采用这个约定：$T_{ee}$ 和 $T_{ref}$ 都先表示在 world frame 中，然后构造 $T_{ee}^{-1}T_{ref}$。

这是一个左不变的局部误差，也就是"从当前末端坐标系看，参考位姿还差多少"。

因此 $\xi$ 的 6 个分量表达在当前 EE 的局部坐标中，而不是 world frame 中。

代码里对应的是：

```cpp
pinocchio::log6(T_ee.actInv(T_ref)).toVector()
```

如果项目选择反向相对变换 $T_{ref}^{-1}T_{ee}$，误差符号和局部坐标含义都会改变。

这不是简单把公式倒过来，还必须同步修改残差、雅可比线性化方向和单轴测试的符号判断。

代价：

$$
\ell_{ee}
=
\frac{1}{2}\xi^{T}Q_{ee}\xi
$$

如果只做位置跟踪：

$$
\ell_{pos}
=
\frac{1}{2}(p_{ee}-p_{ref})^{T}Q_p(p_{ee}-p_{ref})
$$

位置-only 更简单，但不适合需要姿态的任务，如插入、抓握、拧阀。

### 87.5.2 雅可比和梯度

对小扰动 $\delta q$：

$$
\delta \xi
\approx
\mathbf{J}_{log}\mathbf{J}_{ee}\delta q
$$

所以代价梯度近似为：

$$
\frac{\partial \ell_{ee}}{\partial q}
=
\mathbf{J}_{ee}^{T}\mathbf{J}_{log}^{T}Q_{ee}\xi
$$

OCS2 中通常通过自动微分或预计算接口获得二次近似。

但你需要理解梯度背后的物理意义。

末端误差会通过 $\mathbf{J}_{ee}^{T}$ 分配到基座、腿和臂的坐标上。

这个分配不是均匀的——它取决于当前构型下 $\mathbf{J}_{ee}$ 各列的大小。当臂接近伸展极限时，臂列的雅可比分量变小（接近奇异），优化器会更多地利用基座运动来减小误差。当臂在工作空间中心时，臂列分量大，优化器优先动臂。这种自适应行为不是工程师编程出来的，而是 $\mathbf{J}_{ee}^{T}$ 的数学结构自然产生的。理解这一点可以解释很多看似奇怪的 MPC 行为——比如"为什么目标在前方但底盘先后退"（因为后退可以让臂离开奇异区域，总体减小代价）。

回顾复合/130（OCS2 mobile manipulator）中的 Gauss-Newton 近似：那里用 $\ell_{xx} \approx J_e^T Q J_e$ 近似末端代价 Hessian。qm_control 的 EE cost 使用同样的近似策略，但 $\mathbf{J}_{ee}$ 的维度更大（包含浮动基座的 6 列），因此 Hessian 矩阵也更大、结构更复杂。OCS2 的 SQP 求解器（详见 130 章 83.22.1 节关于 KKT 系统的讨论）正是利用了这个 Hessian 的稀疏结构来高效求解。

由于本章的 $\xi$ 是当前 EE 局部坐标中的误差，线性化时使用的 $\mathbf{J}_{ee}$ 也应采用同一坐标约定。

若上游使用 world 对齐雅可比，则要先用伴随变换把误差和速度统一到同一坐标系，再进入代价近似。

如果不限制基座任务，优化器可能用机身运动来追末端。

### 87.5.3 代价权重调度

EE 权重不应固定不变。

| 阶段 | 建议 EE 权重 | 理由 |
|---|---:|---|
| 行走接近 | 低 | 优先保持步态和稳定 |
| 静态预对准 | 中 | 允许身体配合 |
| 接触操作 | 中到高 | 需要末端精度和力 |
| 推拉强接触 | 方向选择性 | 法向力重要，切向滑动需限制 |
| 失稳恢复 | 低 | 生存任务优先 |

一个简单调度公式：

$$
Q_{ee}(t)
=
\alpha_{support}(t)\alpha_{phase}(t)Q_{ee}^{nom}
$$

其中 $\alpha_{support}$ 根据支撑裕度降低权重。

当摩擦裕度小或 ZMP 接近边界时，末端任务让位。

### 87.5.4 实现框架

```cpp
class EndEffectorTrackingCost {
 public:
  double getValue(double time,
                  const Eigen::VectorXd& state,
                  const TargetTrajectories& target) const {
    Eigen::VectorXd q = layout_.getGeneralizedCoordinates(state);
    pinocchio::forwardKinematics(model_, data_, q);
    // Pinocchio 3.x: updateFramePlacement 更新单个 frame 的位姿。
    // 如果需要所有 frame，使用 updateFramePlacements（复数形式）。
    pinocchio::updateFramePlacement(model_, data_, eeFrame_);

    const pinocchio::SE3 T_ee = data_.oMf[eeFrame_];
    const pinocchio::SE3 T_ref = interpolateEeReference(target, time);

    // 本章统一使用 T_ee^{-1} T_ref，误差表达在当前 EE 局部坐标。
    Eigen::Matrix<double, 6, 1> error =
        pinocchio::log6(T_ee.actInv(T_ref)).toVector();

    Eigen::Matrix<double, 6, 6> Q = scheduledWeight(time, state);
    return 0.5 * error.transpose() * Q * error;
  }

 private:
  StateInputLayout layout_;
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  pinocchio::FrameIndex eeFrame_;
};
```

### 87.5.5 坐标系一致性

EE reference 可能来自：

| 来源 | 坐标系 |
|---|---|
| 手柄 | base frame 或世界 frame |
| 视觉系统 | camera frame |
| 任务规划器 | map/world frame |
| 示教轨迹 | task frame |

进入 cost 前必须统一到同一坐标系。

如果 reference 在 base frame，而实际 $T_{ee}$ 在 world frame，末端会出现随基座运动而变化的假误差。

这里的"统一"包含两层：

| 层次 | 必须统一的量 | 不统一的后果 |
|---|---|---|
| 位姿来源坐标系 | $T_{ee}$、$T_{ref}$ 都转换到 world 或同一个 task frame | 平移和姿态残差混入基座运动 |
| 误差表达坐标系 | $\xi$、$\mathbf{J}_{ee}$、末端速度残差使用同一种 LOCAL / world 对齐约定 | 梯度方向与残差方向不匹配 |

本章后续代码默认第一层使用 world frame 位姿，第二层使用当前 EE 局部坐标残差。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：SE(3) 误差左右乘顺序写反**
>
> 本章约定使用 $T_{ee}^{-1}T_{ref}$，即 `T_ee.actInv(T_ref)`。
>
> $T_{ee}^{-1}T_{ref}$ 和 $T_{ref}^{-1}T_{ee}$ 的误差方向相反，表达坐标也不同。
>
> 位置误差小时不容易发现，姿态误差大时会导致控制方向错误。
>
> 自检方法：只给末端一个 +x 平移目标，并确认公式、代码、雅可比和权重矩阵使用同一坐标约定。

> 💡 **概念误区：EE tracking 只需要臂关节**
>
> 在浮动基座系统中，基座移动和腿姿态都会影响末端位姿。
>
> 如果任务允许，身体可以配合扩展工作空间。
>
> 如果任务不允许，就必须用更高优先级限制 base。

> 🧠 **思维陷阱：固定 EE 权重适合所有阶段**
>
> 行走、接近、接触、恢复阶段对末端精度的需求不同。
>
> 权重调度不是调参技巧，而是任务阶段的数学表达。

### 练习 87.5

| # | 练习 | 难度 |
|---|---|---|
| 1 | 写一个函数比较 $T_{ee}^{-1}T_{ref}$ 和 $T_{ref}^{-1}T_{ee}$ 的误差符号。 | ⭐⭐ |
| 2 | 设计基于支撑裕度的 $Q_{ee}$ 调度函数，并在仿真中观察末端误差变化。 | ⭐⭐⭐ |
| 3 | 将 EE reference 从 camera frame 转到 world frame，写出完整变换链。 | ⭐⭐⭐ |

---

## 87.6 摩擦锥与接触约束：`FrictionConeConstraint` ⭐⭐⭐

这一节讲 WBC 和 MPC 的安全底线。

如果摩擦锥写错，机器人可以在优化里"站得住"，但在仿真或真机中会滑倒。

### 87.6.1 点接触摩擦锥

对单个足端接触力：

$$
\lambda
=
\begin{bmatrix}
f_x \\ f_y \\ f_z
\end{bmatrix}
$$

库仑摩擦锥为：

$$
\sqrt{f_x^2+f_y^2}
\le
\mu f_z
$$

且：

$$
f_z\ge 0
$$

为了写成 QP，常使用多面体近似。

四边近似可写为：

$$
\begin{aligned}
 f_x &\le \mu f_z / \sqrt{2}\\
-f_x &\le \mu f_z / \sqrt{2}\\
 f_y &\le \mu f_z / \sqrt{2}\\
-f_y &\le \mu f_z / \sqrt{2}\\
-f_z &\le 0
\end{aligned}
$$

这里采用的是**内接四边形近似**：把真实圆锥内部的一部分作为可行域，因此比真实摩擦锥更保守。它的好处是不会让优化器使用真实接触无法提供的切向力；代价是可能把一些本来可行的接触力排除掉。另一种常见写法是 $|f_x|\le\mu f_z,\ |f_y|\le\mu f_z$，那是外接方形近似，计算同样简单但会放大可行域。教学实现优先选内接近似，是为了让失败更早出现在仿真中，而不是推迟到真机打滑时才暴露。

### 87.6.2 摆动相零力约束

如果某只脚处于摆动相，它不能施加地面反力。

写成：

$$
\lambda_i=0
\quad
\text{if}
\quad
\sigma_i=0
$$

如果不加这个约束，优化器可能用"空中脚"生成虚假力矩。

### 87.6.3 约束矩阵构造

```cpp
class FrictionConeConstraint {
 public:
  void addFoot(Eigen::MatrixXd& A,
               Eigen::VectorXd& b,
               int row,
               int force_index,
               double mu,
               bool in_contact) const {
    if (!in_contact) {
      // 摆动脚：fx=fy=fz=0，可作为等式处理。
      return;
    }

    const double c = mu / std::sqrt(2.0);

    // fx - c*fz <= 0
    A(row + 0, force_index + 0) = 1.0;
    A(row + 0, force_index + 2) = -c;

    // -fx - c*fz <= 0
    A(row + 1, force_index + 0) = -1.0;
    A(row + 1, force_index + 2) = -c;

    // fy - c*fz <= 0
    A(row + 2, force_index + 1) = 1.0;
    A(row + 2, force_index + 2) = -c;

    // -fy - c*fz <= 0
    A(row + 3, force_index + 1) = -1.0;
    A(row + 3, force_index + 2) = -c;

    // -fz <= 0
    A(row + 4, force_index + 2) = -1.0;
    b.segment<5>(row).setZero();
  }
};
```

### 87.6.4 与末端接触的区别

脚底接触常是点接触。

末端接触可能是面接触、抓取接触或单向推压。

| 接触 | 约束 |
|---|---|
| 支撑脚 | $f_z\ge0$，摩擦锥 |
| 手推墙 | 法向力只能推，切向摩擦有限 |
| 夹爪抓取 | 接触可双向，取决于夹持力 |
| 托举物体 | 法向方向由物体接触面决定 |

把末端约束直接套用脚底摩擦锥通常不正确。

末端接触方向应由接触面法向决定。

### 87.6.5 与 feature-compliance 的关系

`feature-compliance` 的核心思想之一是把电机扭矩饱和和摩擦锥限制放进同一个 QP 可行性问题。

这比单独做力控更稳。

因为末端柔顺不是只调臂阻抗。

末端柔顺会改变足底反力和关节扭矩分配。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：摩擦锥坐标系错**
>
> 如果 $z$ 轴不是接触法向，$f_z$ 就不是法向力。
>
> 在坡面或墙面接触中，必须把力转换到接触局部坐标。

> 💡 **概念误区：只要 $f_z>0$ 就不会滑**
>
> 法向力为正只说明接触没有拉地面。
>
> 是否滑动由切向力和 $\mu f_z$ 的比例决定。

> 🧠 **思维陷阱：摩擦系数设大一点就更容易成功**
>
> 高摩擦系数会让优化器使用真实世界做不到的水平力。
>
> 训练和仿真可以用摩擦随机化，但安全验证必须测试低摩擦极端。

### 练习 87.6

| # | 练习 | 难度 |
|---|---|---|
| 1 | 手写单脚四边摩擦锥矩阵，验证随机力是否满足约束。 | ⭐ |
| 2 | 将脚底摩擦锥扩展到斜坡接触，写出世界系到接触局部系的变换。 | ⭐⭐⭐ |
| 3 | 设计一个末端推墙约束，明确法向方向、单侧力和切向摩擦限制。 | ⭐⭐⭐ |

---

## 87.7 WBC 总览：`qm_wbc` 的任务图 ⭐⭐⭐⭐

这一节进入控制层。

MPC 给出的是参考轨迹和接触力趋势。

WBC 要在当前时刻把它变成关节力矩。

回顾足式/90：WBC 的核心是带约束的全身逆动力学。

四足臂比纯四足多了末端任务、臂姿态任务和更强的自碰撞/限位需求。

### 87.7.1 决策变量

一种直观写法：

$$
\mathbf{z}
=
\begin{bmatrix}
\dot{\mathbf{v}} \\
\boldsymbol{\lambda} \\
\boldsymbol{\tau}
\end{bmatrix}
$$

其中：

| 子块 | 维度示例 | 含义 |
|---|---:|---|
| $\dot{v}$ | 24 | 全身广义加速度 |
| $\lambda$ | 12 或更多 | 支撑脚接触力，若 EE 接触则增加 |
| $\tau$ | 18 | 腿+臂关节力矩 |

### 87.7.2 任务层级

qm_control 风格的 WBC 可理解为：

```text
Level 0: 硬约束
  ├─ Floating base dynamics
  ├─ Contact acceleration
  ├─ Friction cone
  └─ Torque limits

Level 1: 生存与支撑
  ├─ Base orientation
  ├─ CoM / momentum
  └─ Stance foot stability

Level 2: 运动与操作
  ├─ Swing foot trajectory
  ├─ Body velocity tracking
  └─ End-effector tracking

Level 3: 正则
  ├─ Joint posture
  ├─ Arm nominal pose
  └─ Torque smoothness
```

不同分支和实现细节可能不同。

但这个任务图是读源码时的导航。

> **跨领域类比**：这个层级结构类似于操作系统的进程调度。Level 0 硬约束相当于内核态保护——不可越界，否则系统崩溃（机器人摔倒）。Level 1 生存任务相当于实时进程——必须在 deadline 内完成，否则系统性能严重下降（机身失稳）。Level 2 运动与操作相当于普通用户进程——尽力执行，但当资源不足（可行力矩/接触力不够）时可以被降级。Level 3 正则相当于后台进程——只在有空闲资源时运行。与操作系统不同的是，WBC 的"调度"不是时间片轮转，而是零空间投影或 QP 权重——它在一个时刻内同时处理所有层级，但保证高层级不被低层级干扰。

### 87.7.3 Floating base dynamics task

全身动力学：

$$
\mathbf{M}\dot{\mathbf{v}}+\mathbf{h}
=
\mathbf{S}^{T}\tau+\mathbf{J}_{c}^{T}\lambda
$$

末端主动接触时，$f_{ee}$ 必须明确属于哪一类变量。

**建模方式 A：$f_{ee}$ 是优化变量。**

这种做法适合推门、托举、接触力分配等任务。

把末端力并入接触力向量：

$$
\lambda_{all}
=
\begin{bmatrix}
\lambda_{foot}\\
f_{ee}
\end{bmatrix},
\qquad
\mathbf{J}_{all}
=
\begin{bmatrix}
\mathbf{J}_{c}\\
\mathbf{J}_{ee}
\end{bmatrix}
$$

动力学写成：

$$
\mathbf{M}\dot{\mathbf{v}}+\mathbf{h}
=
\mathbf{S}^{T}\tau+\mathbf{J}_{all}^{T}\lambda_{all}
$$

写成 QP 等式：

$$
\begin{bmatrix}
\mathbf{M} & -\mathbf{J}_{c}^{T} & -\mathbf{J}_{ee}^{T} & -\mathbf{S}^{T}
\end{bmatrix}
\begin{bmatrix}
\dot{\mathbf{v}}\\
\lambda_{foot}\\
f_{ee}\\
\tau
\end{bmatrix}
=
-\mathbf{h}
$$

此时末端力还需要自己的约束或代价。

例如单向推压要限制法向力符号，抓取要允许双向力并加入夹持能力约束，托举要考虑接触法向和摩擦锥。

如果末端已经固定在环境上，还应把 $\mathbf{J}_{ee}\dot{\mathbf{v}}+\dot{\mathbf{J}}_{ee}\mathbf{v}=0$ 或期望末端加速度并入接触加速度任务。

**建模方式 B：$f_{ee}$ 是已知扰动。**

这种做法适合力传感器测到外力、上层阻抗已经给出外力估计，或只想在 WBC 中补偿已知末端负载。

此时 $f_{ee}$ 不进入 QP 决策变量，而是移动到等式右端：

$$
\begin{bmatrix}
\mathbf{M} & -\mathbf{J}_{c}^{T} & -\mathbf{S}^{T}
\end{bmatrix}
\begin{bmatrix}
\dot{\mathbf{v}}\\
\lambda_{foot}\\
\tau
\end{bmatrix}
=
-\mathbf{h}+\mathbf{J}_{ee}^{T}f_{ee}^{meas}
$$

这里的 $f_{ee}^{meas}$ 仍按"环境施加在机器人末端上的力"取号。

如果传感器输出的是机器人施加给环境的力，右端应使用 $-\mathbf{J}_{ee}^{T}f_{ee}^{meas}$。

读代码时可以用一个规则判断：只要 $f_{ee}$ 出现在优化变量向量里，就必须同时看到末端接触雅可比、力约束和代价；如果它没有出现在优化变量里，就只能作为右端已知项或外部补偿项出现。

### 87.7.4 Contact acceleration task

支撑脚不滑：

$$
\mathbf{J}_{c}\dot{\mathbf{v}}+\dot{\mathbf{J}}_{c}\mathbf{v}=0
$$

QP 等式：

$$
\begin{bmatrix}
\mathbf{J}_{c} & 0 & 0
\end{bmatrix}
\mathbf{z}
=
-\dot{\mathbf{J}}_{c}\mathbf{v}
$$

摆动脚不是接触约束。

摆动脚应作为运动任务跟踪期望轨迹。

### 87.7.5 Base tracking task

基座姿态任务常写成加速度级 PD：

$$
\dot{\omega}_{b}^{des}
=
K_p e_R
+
K_d(\omega_{ref}-\omega_b)
+
\dot{\omega}_{ref}
$$

映射到广义加速度：

$$
\mathbf{J}_{base}\dot{v}+\dot{\mathbf{J}}_{base}v
=
\ddot{x}_{base}^{des}
$$

### 87.7.6 End-effector task

末端任务：

$$
\mathbf{J}_{ee}\dot{v}+\dot{\mathbf{J}}_{ee}v
=
\ddot{x}_{ee}^{des}
$$

其中：

$$
\ddot{x}_{ee}^{des}
=
\ddot{x}_{ref}
+
K_p \xi
+
K_d(\dot{x}_{ref}-\dot{x}_{ee})
$$

注意：这个任务作用在全身加速度上，不只是臂加速度。

如果 base 任务优先级不够，WBC 可能通过移动机身来追末端。

### 87.7.7 任务组装代码框架

```cpp
struct Task {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::MatrixXd D;
  Eigen::VectorXd f;
  double weight = 1.0;
};

class WbcBase {
 public:
  Task formulateFloatingBaseEomTask(const DynamicsCache& cache) {
    Task task;
    const int nv = cache.model.nv;
    const int nf = cache.contactForceDim;
    const int na = cache.actuatedDof;
    task.A.resize(nv, nv + nf + na);
    task.A.setZero();
    task.A.block(0, 0, nv, nv) = cache.M;
    task.A.block(0, nv, nv, nf) = -cache.Jc.transpose();
    task.A.block(0, nv + nf, nv, na) = -cache.S.transpose();
    task.b = -cache.h;
    return task;
  }

  Task formulateEndEffectorTask(const DynamicsCache& cache,
                                const EeReference& ref) {
    Task task;
    const int nv = cache.model.nv;
    const int dim = 6;
    task.A.resize(dim, cache.totalDecisionDim());
    task.A.setZero();
    task.A.block(0, 0, dim, nv) = cache.Jee;
    task.b = ref.acceleration - cache.dJee_v;
    task.weight = ref.weight;
    return task;
  }
};
```

### 87.7.8 Hierarchical vs Weighted

如果源码中同时存在 `HierarchicalWbc` 和 `WeightedWbc`，可以按下面方式理解。

| 实现 | 思路 | 优势 | 风险 |
|---|---|---|---|
| Weighted | 所有软任务加权合成一个 QP | 快，代码短 | 优先级靠权重，可能互相污染 |
| Hierarchical | 高层先解，低层在上层可行集内求解 | 优先级更清晰 | 求解次数更多，调试复杂 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：`dJ * v` 没有计算或符号错误**
>
> 现象：静态任务能跟踪，快速运动时足端或末端出现系统误差。
>
> 根本原因：加速度级任务缺少 $\dot{J}v$ 项。
>
> 正确做法：使用 Pinocchio 的 Jacobian time variation，并做有限差分验证。

> 💡 **概念误区：WBC 只是在 MPC 后面加 PD**
>
> WBC 解的是受动力学和接触约束限制的逆动力学问题。
>
> PD 只提供期望加速度，不能替代约束求解。

> 🧠 **思维陷阱：把 EE task 放得越高越好**
>
> EE task 太高会破坏 base 和 contact 任务。
>
> 对移动操作而言，末端任务通常应低于生存和支撑任务。

### 练习 87.7

| # | 练习 | 难度 |
|---|---|---|
| 1 | 手推 `formulateFloatingBaseEomTask()` 的矩阵维度，确认每个块能相乘。 | ⭐⭐ |
| 2 | 给最小 WBC 添加 EE task，比较只用臂雅可比和用全身雅可比的差异。 | ⭐⭐⭐ |
| 3 | 对同一组任务实现 Weighted 与 Hierarchical 两版，记录求解时间和 base/EE 误差。 | ⭐⭐⭐⭐ |

---

## 87.8 `HoQp` 与层级求解：把优先级变成矩阵 ⭐⭐⭐⭐

这一节解释 `HoQp` 这类文件的意义。

名字里常出现 HoQP、HQP、Hierarchical QP。

它们都围绕同一个问题：如何让低优先级任务不破坏高优先级任务。

### 87.8.1 两层问题

Level 1：

$$
\min_z \|A_1 z-b_1\|^2
\quad
\text{s.t.}
\quad
D_1 z\le f_1
$$

得到最优解 $z_1^{*}$ 和残差 $r_1^{*}$。

Level 2：

$$
\min_z \|A_2 z-b_2\|^2
$$

同时保持：

$$
A_1z-b_1=r_1^{*}
$$

或者在数值上允许小松弛：

$$
\|A_1z-b_1\|^2\le \|r_1^{*}\|^2+\epsilon
$$

### 87.8.2 零空间形式

如果 Level 1 是等式任务，可写：

$$
z
=
z_1^{*}
+
N_1 y
$$

其中 $N_1$ 是 $A_1$ 的零空间基。

Level 2 在 $y$ 上求解：

$$
\min_y
\|A_2(z_1^{*}+N_1y)-b_2\|^2
$$

工程实现通常不会显式构造大规模零空间基。

会通过级联 QP 或约束保持上层残差。

### 87.8.3 为什么四足臂更需要层级

纯四足中，主要冲突是 base、foot、contact。

四足臂中新增：

| 冲突 | 例子 |
|---|---|
| base vs EE | 为追末端导致机身倾斜 |
| foot vs EE force | 推门导致脚底摩擦不足 |
| arm posture vs collision | 关节居中可能让臂靠近腿 |
| torque limit vs compliance | 柔顺力控要求力矩，但电机已饱和 |
| swing foot vs arm motion | 摆腿空间被臂占据 |

这些冲突不是简单权重能稳健解决的。

### 87.8.4 层级 WBC 代码骨架

```cpp
class HierarchicalWbc {
 public:
  Eigen::VectorXd solve(const std::vector<Task>& levels) {
    Eigen::VectorXd z = Eigen::VectorXd::Zero(decisionDim_);
    ConstraintSet locked;

    for (const Task& level : levels) {
      QpProblem qp = buildQpForLevel(level, locked);
      QpSolution sol = solver_.solve(qp);
      if (!sol.ok()) {
        return fallback(z, level);
      }
      z = sol.primal;
      locked.addOptimalResidual(level, z);
    }
    return z;
  }

 private:
  QpProblem buildQpForLevel(const Task& level,
                            const ConstraintSet& locked) {
    QpProblem qp;
    // 当前层目标。
    qp.H = level.A.transpose() * level.A;
    qp.g = -level.A.transpose() * level.b;
    // 已锁定的上层残差作为等式或窄松弛约束。
    qp.addLockedConstraints(locked);
    qp.addInequality(level.D, level.f);
    return qp;
  }
};
```

### 87.8.5 可行性回退

四足臂 WBC 必须有回退策略。

| 失败 | 可能回退 |
|---|---|
| 低层 EE task 不可行 | 降低 EE 权重或冻结 EE 参考 |
| 摩擦锥不可行 | 放松末端力，降低推拉任务 |
| 力矩限幅不可行 | 降低加速度目标，切换站立保护 |
| 接触状态异常 | 使用上一次接触模式，等待估计稳定 |
| 求解器超时 | 使用上一次力矩并增加阻尼 |

注意：硬约束不是随便放松的。

最先放松的应是操作精度、姿态正则、速度命令。

最后才考虑接触和安全约束。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：层级约束没有带上不等式**
>
> 如果 Level 2 只保留 Level 1 的等式残差，却丢失摩擦锥和力矩限幅，解可能重新违反硬约束。
>
> 层级求解时硬约束必须贯穿所有层。

> 💡 **概念误区：HQP 不需要权重**
>
> HQP 解决优先级，不消除同层任务的尺度问题。
>
> 同一层内部仍需要合理权重和单位归一化。

> 🧠 **思维陷阱：不可行就是求解器不好**
>
> 很多不可行来自任务本身冲突。
>
> 例如低摩擦地面上要求大末端水平推力。
>
> 调求解器之前，应先检查物理可行性。

### 练习 87.8

| # | 练习 | 难度 |
|---|---|---|
| 1 | 用二维 toy problem 实现两层 HQP，验证低层不破坏高层。 | ⭐⭐ |
| 2 | 在四足臂 WBC 中把 EE task 从 Level 2 移到 Level 1，观察 base 稳定变化。 | ⭐⭐⭐ |
| 3 | 设计一个不可行检测表，区分摩擦锥不可行、力矩不可行和任务冲突不可行。 | ⭐⭐⭐⭐ |

---

## 87.9 `QMController`：500 Hz 回调中的时间结构 ⭐⭐⭐

这一节讲控制主循环。

源码中最重要的不是某个类的继承关系。

最重要的是每个周期发生什么。

### 87.9.1 控制周期分层

| 模块 | 典型频率 | 说明 |
|---|---:|---|
| 硬件/仿真 step | 1 kHz 或更高 | 关节状态和命令 |
| WBC | 200-500 Hz | 当前时刻力矩求解 |
| MPC | 20-100 Hz | horizon 优化 |
| 估计器 | 200-1000 Hz | IMU/关节/接触融合 |
| 手柄/任务命令 | 10-50 Hz | 高层目标变化 |

MPC 和 WBC 通常异步。

WBC 不能等待 MPC 每次求解完成。

否则 MPC 一次延迟就会阻塞控制链。

### 87.9.2 主循环伪代码

```cpp
void QMController::update(const ros::Time& time,
                          const ros::Duration& period) {
  // 1. 从硬件接口读取关节状态和 IMU。
  RobotState state = readStateFromHardware();

  // 2. 状态估计更新基座位置、速度和接触状态。
  estimator_.update(state, period.toSec());

  // 3. 低频触发 MPC，或从异步 MPC buffer 读取最新轨迹。
  if (mpcTimer_.shouldRun(time)) {
    mpcInterface_.setObservation(estimator_.observation());
    mpcInterface_.advanceMpc();
  }
  MpcReference ref = mpcInterface_.getInterpolatedReference(time);

  // 4. 高频 WBC 组装 QP 并求解当前关节命令。
  WbcOutput wbc = wbc_.solve(estimator_.observation(), ref);

  // 5. 安全检查和命令限幅。
  JointCommand cmd = safety_.filter(wbc.toJointCommand(), state);

  // 6. 写入硬件接口。
  writeCommandToHardware(cmd);
}
```

### 87.9.3 插值为什么重要

MPC 输出的是离散时间轨迹。

WBC 需要任意当前时刻的参考。

如果直接使用最近节点，会出现阶跃。

阶跃参考会导致加速度和力矩尖峰。

常见插值：

| 量 | 建议插值 |
|---|---|
| CoM / base position | 线性或三次样条 |
| 姿态 | SLERP 或 Lie group 插值 |
| 接触力 | 分段线性 + 接触切换平滑 |
| 关节参考 | 三次样条 |
| mode flag | 离散切换，但前后加过渡窗口 |

### 87.9.4 安全过滤

```cpp
class SafetyFilter {
 public:
  JointCommand filter(const JointCommand& raw,
                      const RobotState& state) {
    JointCommand out = raw;
    limitTorque(out.tau);
    limitTorqueRate(out.tau);
    limitPositionTarget(out.q_des, state.q);
    if (!state.contactsReliable) {
      increaseDamping(out);
    }
    if (estop_) {
      out = makeDampedHoldCommand(state);
    }
    return out;
  }
};
```

真机分支通常会增加更多保护。

例如通信超时、关节温度、电机错误码、姿态倾倒阈值和急停。

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：WBC 等待 MPC**
>
> WBC 高频循环不能阻塞等待 MPC。
>
> 正确做法是使用最新可用轨迹；MPC 过期时外推或降级。

> 💡 **概念误区：插值只是让轨迹更好看**
>
> 插值影响力矩连续性。
>
> 不连续参考会直接变成 WBC 加速度尖峰。

> 🧠 **思维陷阱：仿真里不需要安全过滤**
>
> 仿真也需要限幅和异常处理。
>
> 否则仿真策略可能依赖真实硬件做不到的力矩尖峰。

### 练习 87.9

| # | 练习 | 难度 |
|---|---|---|
| 1 | 在主循环中记录 MPC/WBC/估计器耗时，绘制 p50/p90/p99。 | ⭐⭐ |
| 2 | 实现接触力线性插值和平滑切换，对比切换瞬间扭矩尖峰。 | ⭐⭐⭐ |
| 3 | 设计真机安全过滤器，列出至少 10 个保护条件和对应动作。 | ⭐⭐⭐ |

---

## 87.10 状态估计：`qm_estimation` 为什么不能省 ⭐⭐

这一节讲估计器。

四足臂控制里，估计器不是外围功能。

WBC 和 MPC 都依赖当前基座状态、速度、接触模式和关节状态。

### 87.10.1 浮动基座估计的难点

浮动基座没有编码器。

它的位置和速度需要通过 IMU、腿运动学、接触假设和外部定位估计。

加入机械臂后，多了两个影响。

第一，臂运动改变整机 CoM 和 IMU 感受到的加速度。

第二，末端接触会引入外力，让仅基于腿接触的估计更容易偏。

### 87.10.2 线性 Kalman filter 的典型结构

状态可简化为：

$$
x
=
\begin{bmatrix}
p_b\\
v_b\\
p_{foot,1}\\
\cdots\\
p_{foot,4}
\end{bmatrix}
$$

预测：

$$
p_{b,k+1}
=
p_{b,k}+v_{b,k}\Delta t+\frac{1}{2}a_{imu}\Delta t^2
$$

$$
v_{b,k+1}
=
v_{b,k}+a_{imu}\Delta t
$$

观测来自支撑脚运动学：

$$
p_{foot}^{world}
\approx
p_b+R_b p_{foot}^{base}(q)
$$

当脚处于支撑相，足端世界位置近似不变。

当脚处于摆动相，不能用它约束基座。

### 87.10.3 接触置信度

接触不是绝对真值。

| 信号 | 可用于接触判断 | 风险 |
|---|---|---|
| 足端力传感 | 直接 | 许多平台没有 |
| 电机电流 | 间接 | 摩擦和温漂影响 |
| 足端高度 | 简单 | 地形不平时误判 |
| 速度突变 | 有用 | 噪声敏感 |
| 计划 mode | 稳定 | 实际可能提前/滞后 |

工程上常结合计划 mode 和测量信号，得到接触置信度。

估计器可根据置信度调节观测噪声。

### 87.10.4 状态估计对 WBC 的影响

| 估计错误 | WBC 后果 |
|---|---|
| base roll 偏差 | 姿态任务施加错误补偿 |
| base velocity 偏差 | 阻尼项错误，导致振荡 |
| contact mode 错误 | 把摆动脚当支撑脚或相反 |
| CoM 偏差 | 接触力分配不合理 |
| 延迟过大 | WBC 追踪过期参考 |

### 87.10.5 估计器调试代码框架

```cpp
struct EstimatorDiagnostics {
  Eigen::Vector3d base_pos;
  Eigen::Vector3d base_vel;
  Eigen::Vector3d imu_acc;
  std::array<double, 4> contact_prob;
  double innovation_norm = 0.0;
  double covariance_trace = 0.0;

  void printIfAbnormal() const {
    if (innovation_norm > 1.0) {
      std::cerr << "估计创新过大，检查接触状态或 IMU 标定\n";
    }
    if (covariance_trace > 10.0) {
      std::cerr << "估计不确定性过高，考虑降级控制\n";
    }
  }
};
```

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：完全相信计划接触模式**
>
> 计划中某脚支撑，不代表真实已经接触。
>
> 早触地、晚触地和打滑都会让估计器偏移。
>
> 正确做法是用接触概率调节观测权重。

> 💡 **概念误区：臂运动不影响基座估计**
>
> 快速臂运动会改变 IMU 读数和全身 CoM。
>
> 如果估计器仍按纯四足模型解释加速度，会出现系统偏差。

> 🧠 **思维陷阱：估计误差只能靠滤波器调参解决**
>
> 很多估计问题来自接触、模型和坐标系。
>
> 调噪声前应先检查 IMU 坐标、足端 frame、关节方向和接触判断。

### 练习 87.10

| # | 练习 | 难度 |
|---|---|---|
| 1 | 打印每条腿接触概率和估计创新，观察走路和站立时的差异。 | ⭐⭐ |
| 2 | 在仿真中让臂快速摆动，比较开启和关闭臂惯量建模时的 base velocity 估计。 | ⭐⭐⭐ |
| 3 | 设计一个估计器异常时的 WBC 降级策略：哪些任务保留，哪些任务降低权重。 | ⭐⭐⭐ |

---

## 87.11 四分支差异：main、force、compliance、real ⭐⭐⭐

qm_control 的四条分支代表不同工程关注点。

读分支时不要只看代码差异。

要看它试图解决哪类任务。

### 87.11.1 main：全身运动基线

`main` 可理解为基本全身运动框架。

它的目标是打通：

1. 统一 URDF。
2. OCS2 NMPC。
3. WBC。
4. Gazebo 仿真。
5. 手柄或高层命令。

学习时应先让 main 跑通。

不要先从力控或实机分支开始。

### 87.11.2 feature-force：末端受力稳定

这个分支关注末端受到外部力时，系统如何保持稳定。

核心问题：

$$
\mathbf{J}_{ee}^{T}f_{dist}
$$

如何进入 WBC 或补偿项。

典型策略包括：

| 方法 | 含义 |
|---|---|
| 估计末端扰动 | 从力传感或动量误差推断 |
| 调整 base 任务 | 增强支撑稳定 |
| 调整足底力分配 | 对抗外部力矩 |
| 降低 EE tracking | 避免硬抗扰动 |

### 87.11.3 feature-compliance：柔顺与饱和

这个分支更接近 IROS 2024 的核心贡献。

它关注末端柔顺控制，同时显式考虑：

1. 电机扭矩饱和。
2. 地面摩擦锥。
3. 任务可行性回退。

柔顺不是让臂变软这么简单。

末端柔顺会重新分配全身力。

如果扭矩已饱和，继续提高阻抗会造成不可行。

### 87.11.4 feature-real：实机链路

实机分支通常关注：

| 内容 | 为什么重要 |
|---|---|
| 硬件 SDK | 关节命令和状态读取 |
| 安全限幅 | 防止电机过流、过热和越限 |
| 通信延迟 | 影响 WBC 参考同步 |
| 急停 | 实机必须具备 |
| 状态初始化 | 上电姿态不一定等于仿真初值 |
| 日志 | 真机调试依赖完整记录 |

实机代码不一定比算法代码更复杂。

但它的容错要求更高。

### 87.11.5 分支阅读对照

| 问题 | 优先读哪个分支 |
|---|---|
| 想理解基本架构 | main |
| 想处理末端外力扰动 | feature-force |
| 想研究柔顺和饱和 | feature-compliance |
| 想迁移硬件 | feature-real |
| 想改平台 | main + feature-real |
| 想改末端任务 | main + feature-compliance |

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：直接在实机分支改算法**
>
> 实机分支里有大量安全和硬件细节。
>
> 算法修改应先在仿真 main 分支验证，再迁移到实机链路。

> 💡 **概念误区：柔顺控制只属于臂**
>
> 四足臂的末端柔顺会影响足底反力、基座姿态和关节力矩。
>
> 所以柔顺必须是全身问题。

> 🧠 **思维陷阱：把分支差异当成互斥路线**
>
> 四个分支是同一架构的不同侧面。
>
> 真正成熟的系统需要运动、受力、柔顺和安全保护同时存在。

### 练习 87.11

| # | 练习 | 难度 |
|---|---|---|
| 1 | 对比 main 和 feature-force，列出末端扰动相关的新增数据流。 | ⭐⭐ |
| 2 | 对 feature-compliance 画出扭矩限幅和摩擦锥如何同时进入 QP。 | ⭐⭐⭐ |
| 3 | 为 Go2+Z1 实机部署设计 feature-real 修改清单，覆盖硬件接口、限幅和日志。 | ⭐⭐⭐ |

---

## 87.12 从 qm_control 到自己的四足臂控制器 ⭐⭐⭐

这一节把源码阅读转成工程迁移路线。

如果你只会运行原项目，收获有限。

真正掌握是能把架构迁移到新机器人或新任务。

### 87.12.1 最小迁移步骤

| 步骤 | 任务 | 验证 |
|---:|---|---|
| 1 | 替换 URDF 和 mesh | Pinocchio 加载，frame 全部存在 |
| 2 | 更新关节和 frame 名 | 单关节动作方向正确 |
| 3 | 更新状态/输入维度 | layout 测试通过 |
| 4 | 更新 OCS2 task.info 权重 | MPC 能初始化 |
| 5 | 更新 WBC 接触和 EE frame | 站立 QP 可解 |
| 6 | 更新估计器参数 | base 状态稳定 |
| 7 | 运行静态站立 | 无扭矩尖峰 |
| 8 | 运行 EE 小幅轨迹 | base 姿态和末端误差达标 |
| 9 | 加入扰动测试 | 支撑裕度可恢复 |
| 10 | 才考虑真机 | 安全过滤和日志完备 |

### 87.12.2 平台迁移中的三个硬点

第一，关节顺序。

第二，惯量参数。

第三，控制接口。

| 硬点 | 表现 | 处理 |
|---|---|---|
| 关节顺序 | 命令错位 | 建索引映射 + 单关节测试 |
| 惯量参数 | CoM 和力矩不准 | 用 CAD/称重/辨识修正 |
| 控制接口 | torque/position 模式不一致 | 明确低层模式，WBC 输出适配 |

### 87.12.3 新增角动量 regularizer

目标：让 WBC 在不破坏高优先级任务的前提下限制角动量突变。

任务形式：

$$
\dot{k}_G^{des}
=
-K_k k_G
$$

由 CMM 得：

$$
\dot{h}_G
=
\dot{A}_G v
+
A_G\dot{v}
$$

取角动量前三行：

$$
A_{G,ang}\dot{v}
=
\dot{k}_G^{des}
-
\dot{A}_{G,ang}v
$$

WBC 任务矩阵：

$$
\begin{bmatrix}
A_{G,ang} & 0 & 0
\end{bmatrix}z
=
\dot{k}_G^{des}-\dot{A}_{G,ang}v
$$

### 87.12.4 角动量任务代码框架

```cpp
Task formulateAngularMomentumTask(const DynamicsCache& cache,
                                  const Eigen::Vector3d& k_des_dot) {
  Task task;
  const int nv = cache.model.nv;
  task.A.resize(3, cache.totalDecisionDim());
  task.A.setZero();
  task.A.block(0, 0, 3, nv) = cache.Ag.topRows<3>();
  task.b = k_des_dot - cache.dAg.topRows<3>() * cache.v;
  task.weight = cache.weights.angularMomentum;
  return task;
}
```

这个任务不应放在最高层。

它通常低于接触和 base 稳定，高于普通姿态正则。

### 87.12.5 任务测试

| 测试 | 预期 |
|---|---|
| 站立臂静止 | 角动量任务几乎不改变输出 |
| 臂快速摆动 | 角动量峰值下降 |
| 外部推扰 | 恢复时姿态更平滑 |
| EE 精度 | 可能略有下降 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：使用 $A_G$ 却忘了 $\dot{A}_Gv$**
>
> 动量任务是加速度级任务。
>
> 如果缺少 $\dot{A}_Gv$，高速运动时会有系统偏差。

> 💡 **概念误区：角动量越小越好**
>
> 某些动态动作需要角动量。
>
> regularizer 的目标是限制危险突变，而不是禁止全部角动量。

> 🧠 **思维陷阱：新增任务后只看跟踪误差**
>
> 新任务可能改善稳定性但牺牲末端精度。
>
> 需要同时看角动量、基座姿态、末端误差、接触力和求解时间。

### 练习 87.12

| # | 练习 | 难度 |
|---|---|---|
| 1 | 实现角动量 regularizer，并在站立臂摆动任务中比较峰值角动量。 | ⭐⭐⭐ |
| 2 | 把 A1+臂迁移到 Go2+Z1，列出所有必须修改的配置和源码位置。 | ⭐⭐⭐ |
| 3 | 设计一个自动化 smoke test：加载模型、启动估计器、解一次 MPC、解一次 WBC。 | ⭐⭐⭐⭐ |

---

## 87.13 本章小结

### 87.13.1 核心数据流

| 段 | 输入 | 输出 |
|---|---|---|
| `qm_description` | URDF、mesh、惯量 | Pinocchio/仿真模型 |
| `qm_estimation` | IMU、关节、接触 | base 状态 |
| `qm_interface` | state、command、reference | OCS2 MPC 解 |
| `qm_wbc` | 当前状态、MPC 参考、接触模式 | 关节力矩或目标 |
| `qm_controllers` | 所有模块 | 硬件/仿真命令 |

### 87.13.2 关键公式速查

| 公式 | 作用 |
|---|---|
| $x=[h_G,q_b,q_\ell,q_a]$ | MPC 状态布局 |
| $u=[\lambda_{foot},\dot{q}_\ell,\dot{q}_a]$ | MPC 输入布局 |
| $\ell_{ee}=\frac{1}{2}\|\log(T_{ee}^{-1}T_{ref})\|_{Q}^{2}$ | 末端代价 |
| $M\dot{v}+h=S^T\tau+J_c^T\lambda$ | WBC 动力学约束 |
| $J_c\dot{v}+\dot{J}_cv=0$ | 支撑脚不滑 |
| $|f_x|,|f_y|\le\mu f_z/\sqrt{2}$ | 摩擦锥线性近似 |
| $A_{G,ang}\dot{v}=\dot{k}^{des}-\dot{A}_{G,ang}v$ | 角动量 regularizer |

### 87.13.3 知识点掌握矩阵

| 知识点 | 入门 | 工程 | 研究 |
|---|---|---|---|
| 架构数据流 | 能画模块图 | 能定位每个接口 | 能重构为新平台 |
| URDF 审计 | 能加载模型 | 能查惯量和 frame | 能做系统辨识 |
| OCS2 OCP | 能说状态输入 | 能添加 cost/constraint | 能改动力学模型 |
| EE cost | 能写 SE(3) 误差 | 能做权重调度 | 能做接触阶段切换 |
| WBC | 能写 QP 块 | 能实现 task | 能做 HQP/回退 |
| 分支差异 | 能说作用 | 能移植改动 | 能融合柔顺和实机保护 |

---

## 累积项目：qm_control 到 Go2+Z1 的迁移练习

本章项目目标是完成一个可运行的迁移方案。

不要求一次上真机。

要求每个环节都有可验证输出。

### 阶段 1：模型替换

1. 准备 Go2+Z1 URDF。
2. 对齐 foot 和 EE frame。
3. 检查 `nq/nv` 和 actuated DoF。
4. 打印质量、CoM、惯量。
5. 对比 A1+臂和 Go2+Z1 的支撑多边形。

### 阶段 2：OCS2 接口更新

1. 更新状态输入维度。
2. 更新关节索引。
3. 更新 base、leg、arm 权重。
4. 更新 EE frame。
5. 保持初始任务简单：站立 + 小幅 EE tracking。

### 阶段 3：WBC 更新

1. 更新 contact frame。
2. 更新 torque limit。
3. 更新 friction coefficient。
4. 确认 floating base dynamics task 维度。
5. 确认 EE task 使用全身 Jacobian。
6. 增加角动量 regularizer。

### 阶段 4：估计和安全

1. 确认 IMU 坐标系。
2. 确认关节方向。
3. 建立接触概率。
4. 加入命令限幅。
5. 加入求解超时降级。

### 阶段 5：实验

| 实验 | 目标 |
|---|---|
| 静态站立 | WBC 可解，无扭矩尖峰 |
| 小幅 EE 正弦 | 末端误差和 base 姿态达标 |
| 摩擦降低 | 观察安全裕度 |
| 臂快速摆动 | 验证角动量任务 |
| 外部推扰 | 验证回退和恢复 |

---

## 延伸阅读

| 材料 | 类型 | 难度 | 阅读目标 |
|---|---|---|---|
| qm_control 仓库 | 代码 | ⭐⭐⭐⭐ | 本章主轴 |
| Zhang et al. 2024 Whole-body Compliance Control | 论文 | ⭐⭐⭐ | 柔顺与饱和 |
| OCS2 legged_robot | 代码 | ⭐⭐⭐ | 纯四足 switched MPC |
| OCS2 mobile_manipulator | 代码 | ⭐⭐⭐ | 末端代价和自碰撞 |
| legged_control | 代码 | ⭐⭐⭐ | OCS2+WBC 四足基线 |
| TSID | 代码/库 | ⭐⭐⭐⭐ | 分层逆动力学 |
| Pinocchio centroidal examples | 文档 | ⭐⭐ | CMM 和动量任务 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|---|---|---|---|
| 启动后机器人姿态异常 | 关节顺序或初始姿态错误 | 1. 单关节测试 2. 打印 qpos 映射 3. 对比 URDF joint order | 87.3 |
| OCS2 初始化失败 | 状态/输入维度与 cost/constraint 不一致 | 1. 打印 layout 2. 检查每个 term 的维度 3. 先关闭新增 term | 87.4 |
| EE tracking 方向反了 | SE(3) 误差左右乘顺序或坐标系错误 | 1. +x 目标测试 2. 打印 world/base/camera 变换 3. 检查 frame | 87.5 |
| 站立时脚底力为负 | 摩擦锥或接触模式配置错误 | 1. 打印每脚 $f_z$ 2. 检查 stance flag 3. 检查重力方向 | 87.6 |
| WBC QP 不可行 | 摩擦锥、力矩限与任务冲突 | 1. 逐层关闭低优先级任务 2. 检查力矩限 3. 降低 EE 权重 | 87.7, 87.8 |
| 切换接触时扭矩尖峰 | MPC 参考插值不连续或 mode 切换太硬 | 1. 平滑接触力 2. 检查时间戳 3. 加过渡窗口 | 87.9 |
| 行走中 base 估计漂移 | 接触判断错误或 IMU 坐标不一致 | 1. 打印接触概率 2. 检查 IMU frame 3. 调整观测噪声 | 87.10 |
| 柔顺任务中末端发抖 | 阻抗增益过高或扭矩饱和 | 1. 降低 Kp 2. 打印 torque saturation 3. 增加 damping | 87.11 |
| 迁移到 Go2 后 WBC 可解但行为差 | 惯量、限位、增益仍是 A1 参数 | 1. 重算质量和 CoM 2. 更新 torque limit 3. 重新调权重 | 87.12 |

---

## 综合项目：为 qm_control 新增"角动量感知推门"模式

目标：在现有架构上增加一个推门任务阶段，使系统在末端推力和角动量之间做可解释折中。

### 项目要求

1. 新增一个 EE push mode。
2. 在 OCS2 reference 中加入 EE force/pose target。
3. 在 WBC 中加入角动量 regularizer。
4. 在 support monitor 中记录摩擦裕度和 ZMP margin。
5. 当支撑裕度低于阈值时降低 EE force target。
6. 记录末端误差、末端力、base pitch、角动量和足底力。

### 推荐实现顺序

| 顺序 | 内容 |
|---:|---|
| 1 | 在仿真中固定四脚站立 |
| 2 | 加入 EE 位置跟踪 |
| 3 | 加入末端推力参考 |
| 4 | 加入角动量 regularizer |
| 5 | 加入支撑裕度调度 |
| 6 | 再考虑行走接近和接触切换 |

### 交付物

| 交付 | 要求 |
|---|---|
| 数据流图 | 标出新增 reference 和 WBC task |
| 数学说明 | 写出 EE force、角动量和支撑裕度公式 |
| 实验曲线 | 至少 5 条时间曲线 |
| 失败归因 | 至少分析 3 种失败模式 |
| 迁移说明 | 说明从 A1+臂到 Go2+Z1 的改动 |


## 章末统一练习与故障排查

⚠️ **易错点一：只看单个指标。** 170_qm_control精读 中的任何结论都应同时检查任务指标、物理约束和软件接口。只看总误差或总奖励，容易把模型错误误判为参数问题。

💡 **易错点二：忽略坐标系和时间戳。** 复合机器人控制链很长，坐标系、采样频率和延迟一旦没有显式记录，后续所有优化和学习结果都会失去解释力。

🧠 **易错点三：把演示成功当成系统可靠。** 教学实验应至少包含一次扰动、一次异常输入和一次日志复盘，才能说明方法的边界。

### 练习

1. 选择本章一个核心公式，写出每一项的单位、坐标系和数据来源。
2. 选择本章一个代码片段，说明它依赖哪些配置项；如果配置错一个符号，会出现什么日志现象？
3. 设计一个只改变单个因素的实验，用来验证本章最关键的工程判断。

> **本质洞察**：复合机器人文档中的公式、代码和项目不是三块孤立内容。公式定义可行边界，代码实现边界，项目用日志证明边界是否真实存在。

### 故障排查

| 症状 | 优先怀疑 | 验证动作 |
| --- | --- | --- |
| 仿真正常但部署异常 | 观测、坐标系或时间戳不一致 | 用同一段日志离线回放训练端和部署端 |
| 指标突然变差 | 模式切换、限幅或安全壳触发 | 画出模式、保护标志和控制命令 |
| 调参没有效果 | 根因不是权重而是模型假设错误 | 回到最小实验，关闭无关模块 |
| 结果难以复现 | 配置没有版本化 | 保存模型哈希、配置哈希和随机种子 |
