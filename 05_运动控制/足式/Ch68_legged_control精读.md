> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 68 章 legged_control 完整项目精读——从 OCS2 到真机部署

> **难度**: ⭐⭐⭐ | **预计学时**: 40-50 小时(2 周) | **前置**: Ch47-49, Ch53-57, Ch61-62
>
> **一句话概要**: legged_control 是连接 OCS2 理论与真机部署的桥梁——它用约 15,000 行 C++ 将 NMPC、WBC、状态估计和硬件接口粘合为一个可运行的四足控制栈,是学完前面所有章节后的"毕业集成项目"。

---

## 前置自测

📋 **前置自测**（答不出 >= 2 题 -> 先回对应章节复习）

1. OCS2 的 `MPC_MRT_Interface` 中,MPC 线程和 MRT 线程通过什么机制通信?Triple Buffer 的三个槽分别扮演什么角色?（Ch55）
2. WBC 的 HQP（Hierarchical QP）如何保证高优先级任务不被低优先级任务干扰?零空间投影的数学含义是什么?（Ch53）
3. `ros_control` 的 `ControllerInterface::update()` 方法由谁调用?调用频率由什么决定?（Ch62）
4. Pinocchio 的 `crba()` 计算的是什么矩阵?它的计算复杂度是多少?（Ch47）
5. Unitree A1 的电机接口使用什么通信协议?`LowCmd` 结构体包含哪些字段?（Ch62）

---

## 本章目标

学完本章，你应能：

1. **完整读懂 legged_control 的每一个模块**——从 LeggedController 主循环到硬件接口的全链路
2. **说清楚 legged_control 与 OCS2 legged_robot 示例的本质差异**——legged_control 加了什么、改了什么、简化了什么
3. **独立将 legged_control 部署到 Gazebo 仿真**——让 A1 站立、trot、bound
4. **为新机器人适配 legged_control**——以 Go2 为例，完成 URDF 替换、SDK 升级、参数调整
5. **修改核心模块**——替换 WBC 策略、更换状态估计器、添加新约束
6. **识别 legged_control 的"教学级简化"**——对比工业级方案，知道差距在哪里

---

## 68.1 legged_control 的定位——为什么需要这个项目 ⭐

### 68.1.1 动机：OCS2 的"最后一公里"问题

我们在 Ch55 精读了 OCS2 的完整架构——五层抽象、SQP 求解器、双线程 MPC。OCS2 本身提供了一个 `ocs2_legged_robot` 示例,演示了如何用 Centroidal Model 做四足 MPC。

**但 OCS2 legged_robot 示例缺了什么？** 它只是一个"算法演示",不是一个"可部署的控制器"。

| 缺失环节 | 为什么不可或缺 | OCS2 示例的状态 |
|----------|---------------|----------------|
| **WBC（全身控制器）** | MPC 输出质心轨迹和接触力参考，需要 WBC 翻译为关节扭矩 | 无——MPC 直接输出关节前馈扭矩，无反馈补偿 |
| **状态估计** | 真机没有 ground truth，需要从 IMU + 编码器 + 触觉融合出基座状态 | 使用 Gazebo ground truth |
| **硬件接口** | 需要通过 SDK 与电机通信，处理通信延迟和安全限制 | 仅 Gazebo ROS 接口 |
| **Sim2Real 框架** | 仿真和真机使用同一控制器代码，只切换底层硬件接口 | 与 Gazebo 强耦合 |
| **安全检查** | 防止关节超限、扭矩过大、姿态异常 | 无 |

> **跨领域类比**：如果 OCS2 是汽车的"发动机"，legged_control 就是把发动机装进车身、接上变速箱和方向盘的工作。发动机再好，不完成这些集成，车就跑不起来。这与软件工程中"库 vs 框架"的区别类似——OCS2 是库(你调用它的 API),legged_control 是框架(它调用你的代码)。框架的价值在于**定义了模块间的协作协议**——数据格式、调用时序、错误处理,这些"胶水"代码往往比算法本身更难写对。

> **本质洞察**：legged_control 的核心贡献**不是算法创新,而是接口设计**。MPC、WBC、状态估计这些算法在论文中都已经定义好了,真正困难的是让它们在同一个实时循环中以正确的时序、正确的数据格式、正确的线程安全方式协同工作。这解释了为什么 legged_control 的 ~15,000 行代码中,纯算法代码不到 30%,其余都是接口、转换、配置和安全逻辑。

### 68.1.2 项目背景

**legged_control**（`qiayuanl/legged_control`）由 **Qiayuan Liao**（UC Berkeley 博士生，导师 Koushil Sreenath）开发，2022 年开源。截至 2025 年：

| 指标 | 数值 |
|------|------|
| GitHub Stars | ~1,700 |
| Forks | ~350 |
| 核心代码量 | ~15,000 行 C++ |
| 依赖的核心框架 | OCS2 + Pinocchio + ros_control + qpOASES |
| 支持的机器人 | Unitree A1（主要）、可扩展到 Go1/Go2 |
| ROS 版本 | ROS1（Noetic，ros_control）|
| 维护状态 | 作者已标注"不再主动维护"，但社区仍活跃 |

**Qiayuan Liao 的研究方向**：安全关键运动控制。代表论文包括 "Walking in Narrow Spaces: Safety-critical Locomotion Control for Quadrupedal Robots with Duality-based Optimization"（IROS 2023），以及参与 Berkeley Humanoid 项目。legged_control 是他博士研究的实验平台基础。

### 68.1.3 legged_control 对社区的意义

在 legged_control 出现之前，想要用 OCS2 做四足控制的研究者面临的困境：

```
                  2021 年的困境
┌──────────────────────────────────────────────┐
│ 我有一个 Unitree A1，想用 OCS2 MPC 控制它    │
│                                              │
│ Step 1: 读 OCS2 文档 ──────→ 2 周            │
│ Step 2: 理解 ocs2_legged_robot ──→ 1 周      │
│ Step 3: 写 WBC ──────────────→ 3 周          │
│ Step 4: 写状态估计 ──────────→ 2 周          │
│ Step 5: 写硬件接口 ──────────→ 2 周          │
│ Step 6: Sim2Real 调试 ───────→ 2 周          │
│ Step 7: 集成 + 调参 ────────→ 2 周          │
│                                              │
│ 总计: 14 周 ≈ 3.5 个月                       │
│ 前提: 你已经懂 MPC + WBC + ROS 控制 + C++    │
└──────────────────────────────────────────────┘

                  有 legged_control 之后
┌──────────────────────────────────────────────┐
│ Step 1: 读 legged_control 代码 ──→ 1 周      │
│ Step 2: 编译 + Gazebo 验证 ──────→ 2 天      │
│ Step 3: 适配到自己的机器人 ──────→ 1 周      │
│ Step 4: 改研究模块(如加 CBF) ──→ 按需      │
│                                              │
│ 总计: 2-3 周即可开始研究                     │
└──────────────────────────────────────────────┘
```

**社区衍生项目**（证明其影响力）：

| 项目 | 作者/团队 | 特点 |
|------|----------|------|
| **hexapod_control** | MasterYip | 六足适配，从 4 足扩展到 6 足 |
| **NMPC4dog** | STAN-32 | legged_control 的简化教学版 |
| **legged_control_go2** | Feng1909 / WeixianLin-cc | Go2 适配版 |
| **Legged_Control (ylo2)** | elpimous | 自制四足机器人适配 |
| **quad_mpc** | zhangOSK | 重构版本，简化依赖 |
| **legged_perceptive** | qiayuanl 本人 | 加入感知的扩展版（Ch67 相关） |

### 68.1.4 如果不用 legged_control 会怎样

假设你决定"从零写一个 MPC+WBC 四足控制器"，你需要：

1. **自己实现 MPC-WBC 的通信接口**——MPC 输出的格式、WBC 输入的格式、时间同步
2. **自己写 ros_control 的 Controller 插件**——理解 ControllerInterface 的生命周期、如何注册 HardwareInterface
3. **自己处理 MPC 的异步性**——MPC 求解需要 10-50 ms，但控制循环只有 1 ms 的预算
4. **自己做状态估计**——从 IMU + 编码器估计基座位姿和速度
5. **自己写安全层**——防止关节飞车、姿态翻转

这些都是"脏活累活"，但缺一个整个系统就跑不起来。legged_control 的价值恰恰在于：**把这些脏活做好，让研究者专注于算法创新**。

> ⚠️ **概念误区**：不要认为 legged_control"只是 OCS2 的 ROS 封装"
>
> **新手想法**："legged_control 就是把 OCS2 的 legged_robot 示例包了一层 ros_control 接口"
>
> **实际上**：legged_control **新增了 WBC、状态估计、安全检查和硬件接口四个完整模块**，这些模块的代码量和复杂度与 MPC 层相当。"封装"这个词严重低估了集成工程的难度——真正的工程价值往往在"粘合"而非"算法"本身。

> 💡 **入门推荐：A1-QP-MPC-Controller 代码阅读路径**
>
> 如果 legged_control 的完整架构初看过于复杂，推荐先阅读 [A1-QP-MPC-Controller](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller)（YY硕的简化版实现）。该项目用更少的代码展示了 MPC+QP 控制链路的核心逻辑。
>
> **核心入口 `MainGazebo.cpp` 的双线程架构**：
>
> ```
> 线程 1：力控线程（高频 ~500 Hz）
>   └─ 读取关节状态 → 计算 Jacobian → 执行 QP 力分配 → 输出关节力矩
>
> 线程 2：主更新线程（低频 ~30 Hz）
>   └─ 读取 IMU/状态估计 → 运行 MPC → 更新期望接触力 → 更新步态相位
> ```
>
> 阅读顺序建议：
> 1. `MainGazebo.cpp`：理解双线程启动和数据流
> 2. `ConvexMPCLocomotion.cpp`：MPC 求解核心（对应 Ch55 理论）
> 3. `BalanceController.cpp`：QP 力分配（对应 Ch51 SRBD + Ch53 WBC 简化版）
> 4. `GaitScheduler.cpp`：步态管理（对应 Ch56）
>
> 建立整体理解后，再回到 legged_control 精读本章内容。

**练习 68.1.A** ⭐：列出 legged_control 的所有 ROS 包,为每个包用一句话说明其职责。画出包之间的依赖关系图（谁 `find_package` 了谁）。

**练习 68.1.B** ⭐：如果你要为一个全新的六足机器人使用 legged_control，哪些包需要修改？哪些可以原封不动地复用？

---

## 68.2 系统架构总览 ⭐⭐

### 68.2.1 全局架构图

上一节说明了 legged_control 的定位,本节展开它的完整系统架构——搞清楚哪些模块存在、它们如何协作。

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        legged_control 系统架构                         │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    ROS Launch + Config                            │   │
│  │  controllers.yaml / task.info / reference.info / gait.info       │   │
│  └──────────────────────────────┬───────────────────────────────────┘   │
│                                 │                                       │
│  ┌──────────────────────────────▼───────────────────────────────────┐   │
│  │              LeggedController (ros_control plugin)               │   │
│  │              ┌─────────────────────────────────────┐             │   │
│  │              │          update() @ 500-1kHz        │             │   │
│  │              │                                     │             │   │
│  │              │  ┌─────────────┐  ┌──────────────┐  │             │   │
│  │              │  │   State     │  │    MPC_MRT   │  │             │   │
│  │              │  │  Estimate   │  │  Interface   │  │             │   │
│  │              │  │ (KalmanF.)  │  │ (query MPC)  │  │             │   │
│  │              │  └──────┬──────┘  └──────┬───────┘  │             │   │
│  │              │         │                │          │             │   │
│  │              │         ▼                ▼          │             │   │
│  │              │  ┌──────────────────────────────┐   │             │   │
│  │              │  │      WBC (HierarchicalWbc)   │   │             │   │
│  │              │  │  -> joint pos/vel/torque      │   │             │   │
│  │              │  └──────────────────────────────┘   │             │   │
│  │              │                │                    │             │   │
│  │              │         ┌──────▼───────┐            │             │   │
│  │              │         │ SafetyChecker│            │             │   │
│  │              │         └──────┬───────┘            │             │   │
│  │              └────────────────┼────────────────────┘             │   │
│  └───────────────────────────────┼──────────────────────────────────┘   │
│                                  │ command_interfaces                    │
│                                  ▼                                       │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │              HardwareInterface (UnitreeHW / GazeboHW)            │   │
│  │              read() -> joint_pos, joint_vel, IMU                  │   │
│  │              write() -> pos_cmd, vel_cmd, kp, kd, torque_cmd     │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌────────────────────────────────────────┐                             │
│  │          MPC Thread (异步)             │                             │
│  │  ┌───────────────────────────────────┐ │                             │
│  │  │ SqpMpc::run() @ 50-100 Hz        │ │                             │
│  │  │ OCS2 求解 -> Triple Buffer 写入   │ │                             │
│  │  └───────────────────────────────────┘ │                             │
│  └────────────────────────────────────────┘                             │
│                                                                         │
│  ┌────────────────────────────────────────┐                             │
│  │     LeggedRobotVisualizer (RViz)       │                             │
│  │  MarkerArray: 接触力, 足端轨迹, CoM    │                             │
│  └────────────────────────────────────────┘                             │
└─────────────────────────────────────────────────────────────────────────┘
```

### 68.2.2 数据流：从传感器到关节扭矩

理解 legged_control 的关键在于追踪**数据流**——一个控制周期内，数据如何从传感器流到执行器。

```
时间线（一个 1ms 控制周期内）
──────────────────────────────────────────────────────────────>
  t=0          t=0.1ms       t=0.3ms        t=0.7ms     t=1ms
  │             │              │               │           │
  ▼             ▼              ▼               ▼           ▼
read()     State         MPC query        WBC solve    write()
硬件读取    Estimate      evaluatePolicy   QP 求解      硬件发送
           KF update     获取参考轨迹     关节扭矩
```

**详细数据流表**：

| 阶段 | 输入 | 处理 | 输出 | 耗时 |
|------|------|------|------|------|
| **HW::read()** | UDP/CAN 原始字节 | 解码电机状态 + IMU | joint_pos[12], joint_vel[12], IMU quat/gyro/accel | ~0.05 ms |
| **State Estimate** | joint + IMU + contact flags | Kalman Filter 预测+更新 | base_pos(3), base_quat(4), base_twist(6), foot_pos(12) | ~0.05 ms |
| **MPC Query** | currentObservation | `evaluatePolicy()` 插值 MPC 轨迹 | optimizedState, optimizedInput, plannedMode | ~0.02 ms |
| **WBC** | desired + measured + mode | HQP 三层任务求解 | joint_pos_cmd[12], joint_vel_cmd[12], torque_ff[12] | ~0.2-0.5 ms |
| **Safety Check** | WBC 输出 + 当前状态 | 关节限位/扭矩限制/姿态检查 | pass/fail + clipped commands | ~0.01 ms |
| **HW::write()** | pos/vel/kp/kd/torque | 编码为电机命令 | UDP/CAN 发送 | ~0.05 ms |
| **总计** | | | | ~0.4-0.7 ms |

> 💡 **设计洞察**：为什么 WBC 占了大部分时间？因为 WBC 需要调用 Pinocchio 计算前向运动学 + 质量矩阵 + 非线性项（~0.1 ms），然后组装并求解 QP（~0.1-0.4 ms）。这也是为什么 WBC 的实时优化如此关键——Ch53 §6.2-6.4 利用 `EIGEN_RUNTIME_NO_MALLOC` 宏在开发期拦截 Eigen 的运行时堆分配:当设置 `set_is_malloc_allowed(false)` 后任何堆分配请求都会触发 assertion 失败,从而将"隐性延迟"转化为"显性崩溃",帮助开发者在上线前消灭所有非确定性内存操作。

### 68.2.3 ROS 包结构

```
legged_control/
├── legged_common/              # 通用工具：类型别名、接触标志解析
│   ├── include/legged_common/
│   │   ├── hardware_interface/ # HybridJointInterface 定义
│   │   └── utils.h             # 工具函数
│   └── src/
├── legged_interface/           # OCS2 问题定义封装
│   ├── src/
│   │   ├── LeggedInterface.cpp # setupOptimalControlProblem()
│   │   ├── LeggedRobotPreComputation.cpp
│   │   ├── constraint/         # 各类约束实现
│   │   └── initialization/     # 初始轨迹生成
│   └── config/                 # task.info, reference.info
├── legged_estimation/          # 状态估计
│   ├── src/
│   │   ├── StateEstimateBase.cpp
│   │   ├── LinearKalmanFilter.cpp  # 核心：线性 KF
│   │   └── FromTopicStateEstimate.cpp  # 仿真用
│   └── include/
├── legged_wbc/                 # 全身控制器
│   ├── src/
│   │   ├── WbcBase.cpp         # 任务公式化
│   │   ├── HierarchicalWbc.cpp # HQP 求解
│   │   ├── WeightedWbc.cpp     # 加权 QP（备选）
│   │   └── HoQp.cpp           # 分层 QP 数据结构
│   └── include/
├── legged_controllers/         # ros_control Controller 插件
│   ├── src/
│   │   └── LeggedController.cpp  # ★ 核心：主循环
│   ├── config/
│   │   └── a1/                 # A1 专用配置
│   │       ├── task.info       # MPC 参数
│   │       ├── reference.info  # 参考轨迹参数
│   │       └── gait.info       # 步态参数
│   └── launch/
├── legged_gazebo/              # Gazebo 仿真集成
├── legged_hw/                  # 硬件接口基类
├── legged_examples/
│   └── legged_unitree/
│       ├── legged_unitree_description/  # URDF + xacro
│       └── legged_unitree_hw/           # Unitree SDK 硬件接口
└── qpoases_catkin/             # qpOASES 的 catkin 封装
```

> ⚠️ **编程陷阱**：legged_control 使用 `catkin build`（ROS1），不是 `catkin_make`
>
> **错误做法**：`catkin_make` 编译整个工作空间
>
> **现象**：OCS2 的 CppAD 代码生成依赖特殊的 CMake 目标顺序，`catkin_make` 的并行编译会导致 "找不到自动生成的 .so 文件" 错误
>
> **根本原因**：`catkin_make` 把所有包放在一个 CMake 项目中并行编译，无法保证包间的编译顺序。`catkin build` 则逐包编译，自动处理依赖
>
> **正确做法**：
> ```bash
> cd ~/catkin_ws
> catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
> # 不是 catkin_make!
> ```

**练习 68.2.A** ⭐⭐：画出 legged_control 所有 ROS 话题的图（`rqt_graph` 风格）。标注每个话题的消息类型和频率。

**练习 68.2.B** ⭐⭐：如果把 WBC 的 QP 求解时间从 0.3 ms 增加到 1.2 ms（比如六足机器人），会发生什么？提出两种解决方案。

---

## 68.3 LeggedController 主循环精读 ⭐⭐

### 68.3.1 为什么从这里开始

上面看到了全局架构，现在我们深入代码。阅读一个大型项目的最佳起点是**找到"调度中心"**——调用其他所有模块的那个类。在 legged_control 中，这个调度中心是 `LeggedController`。

**阅读策略**：先读 `update()` 的控制流（不关心细节），建立"调用关系图"，然后逐个深入被调用的模块。

### 68.3.2 类声明与成员变量

```cpp
// legged_controllers/include/legged_controllers/LeggedController.h

class LeggedController : public controller_interface::ControllerBase {
 public:
  // === ros_control 生命周期 ===
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time,
              const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

 private:
  // === 核心模块（四大金刚）===
  std::shared_ptr<LeggedInterface> leggedInterface_;  // OCS2 问题定义
  std::unique_ptr<MPC_BASE> mpc_;                     // MPC 求解器(SqpMpc)
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_; // MPC 查询接口
  std::shared_ptr<StateEstimateBase> stateEstimate_;   // 状态估计
  std::shared_ptr<WbcBase> wbc_;                       // 全身控制器

  // === 硬件句柄 ===
  std::vector<HybridJointHandle> hybridJointHandles_;  // 12 个关节
  std::vector<ContactSensorHandle> contactHandles_;     // 4 个触觉
  hardware_interface::ImuSensorHandle imuSensorHandle_; // 1 个 IMU

  // === 状态变量 ===
  SystemObservation currentObservation_;  // 当前观测（MPC 格式）
  vector_t measuredRbdState_;             // 全身状态（WBC 格式）

  // === 可视化与安全 ===
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // === MPC 线程 ===
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{false};
};
```

**为什么用 `shared_ptr` 管理核心模块？** 因为 LeggedController 的生命周期由 ros_control 的 Controller Manager 管理，而某些模块（如 `leggedInterface_`）需要被多个对象引用（MPC 和 WBC 都需要访问 Pinocchio 接口）。`shared_ptr` 确保对象在最后一个引用释放后才析构。

### 68.3.3 init() 方法——系统引导

`init()` 是 ros_control 插件的初始化入口，Controller Manager 在加载插件时调用一次。

```cpp
bool LeggedController::init(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle& controller_nh) {
  // ---- Step 1: 加载配置文件路径 ----
  std::string taskFile, urdfFile, referenceFile;
  controller_nh.getParam("taskFile", taskFile);
  controller_nh.getParam("urdfFile", urdfFile);
  controller_nh.getParam("referenceFile", referenceFile);

  // ---- Step 2: 创建 LeggedInterface（OCS2 问题定义）----
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile,
                                                        referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile,
                                                referenceFile, true);

  // ---- Step 3: 创建 MPC 求解器 ----
  // 根据配置选择 SQP 或 DDP
  mpc_ = std::make_unique<SqpMpc>(
      leggedInterface_->mpcSettings(),
      leggedInterface_->sqpSettings(),
      leggedInterface_->getOptimalControlProblem(),
      leggedInterface_->getInitializer());

  // ---- Step 4: 创建 MPC-MRT 接口 ----
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(
      leggedInterface_->mpcSettings(),
      leggedInterface_->getRolloutPtr());

  // ---- Step 5: 获取硬件句柄 ----
  auto* hybridJointInterface =
      robot_hw->get<HybridJointInterface>();
  // 按照配置中的关节名获取 12 个关节句柄
  for (const auto& jointName : jointNames) {
    hybridJointHandles_.push_back(
        hybridJointInterface->getHandle(jointName));
  }

  // ---- Step 6: 创建状态估计器 ----
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
      leggedInterface_->getPinocchioInterface(),
      leggedInterface_->getCentroidalModelInfo(), ...);

  // ---- Step 7: 创建 WBC ----
  wbc_ = std::make_shared<HierarchicalWbc>(
      leggedInterface_->getPinocchioInterface(),
      leggedInterface_->getCentroidalModelInfo(), ...);

  // ---- Step 8: 创建可视化和安全检查 ----
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(...);
  safetyChecker_ = std::make_shared<SafetyChecker>(
      leggedInterface_->getCentroidalModelInfo());

  return true;
}
```

**创建顺序为什么重要？**

```
LeggedInterface ──> MPC ──> MPC_MRT_Interface
      │                          │
      ├──> StateEstimate         │
      │                          │
      └──> WBC <─────────────────┘
                 (需要 MPC 输出的格式信息)
```

`LeggedInterface` 必须最先创建，因为它持有 Pinocchio 接口和 Centroidal Model 信息,后续所有模块都依赖这些数据结构。

### 68.3.4 update() 方法——1 kHz 心跳

这是整个项目最核心的 50 行代码。每个控制周期都会执行一次。

```cpp
void LeggedController::update(const ros::Time& time,
                                const ros::Duration& period) {
  // ==== Phase 1: 状态估计 ====
  // 从硬件句柄读取传感器数据,融合为全身状态
  vector_t jointPos(12), jointVel(12);
  for (int i = 0; i < 12; ++i) {
    jointPos[i] = hybridJointHandles_[i].getPosition();
    jointVel[i] = hybridJointHandles_[i].getVelocity();
  }
  Eigen::Quaternion<scalar_t> quat(
      imuSensorHandle_.getOrientation()[3],  // w
      imuSensorHandle_.getOrientation()[0],  // x
      imuSensorHandle_.getOrientation()[1],  // y
      imuSensorHandle_.getOrientation()[2]); // z
  contact_flag_t contactFlag;
  for (int i = 0; i < 4; ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateImu(quat, angularVel, linearAccel);
  stateEstimate_->updateContact(contactFlag);
  measuredRbdState_ = stateEstimate_->update(time, period);

  // ==== Phase 2: 构造 MPC 观测 ====
  currentObservation_.time = time.toSec();
  // 把 measuredRbdState_ 映射为 MPC 的 centroidal 状态表示
  currentObservation_.state =
      rbdConversions_.computeCentroidalStateFromRbdModel(
          measuredRbdState_);
  currentObservation_.input = vector_t::Zero(inputDim_);
  currentObservation_.mode = cycleContactFlag2ModeNumber(contactFlag);

  // 告知 MPC 当前观测
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // ==== Phase 3: 查询 MPC 策略 ====
  // 从 MPC 的 Triple Buffer 读取最新策略
  mpcMrtInterface_->updatePolicy();
  // 在当前时刻插值，获取参考轨迹
  vector_t optimizedState, optimizedInput;
  size_t plannedMode;
  mpcMrtInterface_->evaluatePolicy(
      currentObservation_.time,
      currentObservation_.state,
      optimizedState, optimizedInput, plannedMode);

  // ==== Phase 4: WBC 求解 ====
  vector_t wbcCmd = wbc_->update(
      optimizedState,       // MPC 期望状态
      optimizedInput,       // MPC 期望输入(力 + 速度)
      measuredRbdState_,    // 当前测量状态
      plannedMode,          // 接触模式
      period.toSec());      // dt

  // ==== Phase 5: 安全检查 + 发送命令 ====
  // 从 wbcCmd 提取关节位置、速度、扭矩
  vector_t posDes = centroidal_model::getJointAngles(
      optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(
      optimizedInput, leggedInterface_->getCentroidalModelInfo());
  vector_t torque = wbcCmd.tail(12);  // 最后 12 维是关节扭矩

  for (int j = 0; j < 12; ++j) {
    hybridJointHandles_[j].setCommand(
        posDes[j],   // 期望位置
        velDes[j],   // 期望速度
        0,           // Kp (由硬件接口的默认值覆盖)
        3,           // Kd (阻尼)
        torque[j]);  // 前馈扭矩
  }

  // ==== Phase 6: 可视化 ====
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(),
                            mpcMrtInterface_->getCommand());
}
```

### 68.3.5 关键设计决策分析

**决策 1：为什么 WBC 的输出包含位置和速度，而不只是扭矩？**

纯扭矩控制在理论上是最"干净"的——WBC 计算出精确的关节扭矩，直接发给电机。但实际中有两个问题：

| 问题 | 原因 | 后果 |
|------|------|------|
| 模型不确定性 | Pinocchio 的惯性参数与真机有误差 | 纯前馈扭矩会有稳态误差 |
| 通信延迟 | WBC 计算的扭矩经过 UDP 传到电机需要 ~0.5 ms | 在这 0.5 ms 内状态已变化 |

**解决方案**：使用 **混合位置-力矩控制模式**（Unitree 电机原生支持）：

$$\tau_{cmd} = K_p (q_{des} - q_{meas}) + K_d (\dot{q}_{des} - \dot{q}_{meas}) + \tau_{ff} \tag{68.1}$$

其中 $\tau_{ff}$ 来自 WBC，$q_{des}$ 和 $\dot{q}_{des}$ 来自 MPC 参考轨迹。PD 反馈补偿了模型误差和通信延迟。

> 🧠 **思维陷阱**：不要认为"加了 PD 就是在退化"
>
> **新手想法**："WBC 算出了扭矩，再加 PD 不是多此一举吗？"
>
> **实际上**：WBC 的扭矩是基于"此刻"的状态计算的。但从计算完成到电机执行之间有 0.5-1 ms 的延迟,在这段时间内机器人已经移动了。PD 反馈在电机内部以 10-40 kHz 运行，**比 WBC 快 10-40 倍**，能实时补偿这个"算了但还没执行"的间隙。
>
> 这是控制中经典的"前馈+反馈"范式：前馈提供主要驱动力（WBC 扭矩），反馈消除残差（PD 补偿）。

**决策 2：为什么 MPC 是异步的，而 WBC 是同步的？**

```
MPC 线程 (50 Hz):     |===solve===|     |===solve===|     |===solve===|
                       ↓ 写 buffer  ↓     ↓ 写 buffer  ↓
Controller 线程 (1kHz): |u|u|u|u|u|u|u|u|u|u|u|u|u|u|u|u|u|u|u|u|
                         ↑ 读 buffer                     ↑ 读 buffer
```

- **MPC 异步**：因为 MPC 的求解时间（10-50 ms）远超 1 ms 周期预算。让 MPC 在独立线程中异步运行,Controller 线程只查询最新结果,不等待。
- **WBC 同步**：因为 WBC 的求解时间（< 1 ms）可以在周期预算内完成。同步执行保证了 WBC 使用的状态是最新的。

如果 WBC 也做成异步的会怎样？

```
WBC 异步的问题：
1. WBC 用的状态是"过去的"（延迟 1-2 个周期）
2. 机器人在快速运动时，1-2 ms 的状态延迟 → 足端位置误差可达 mm 级
3. 在 stance-swing 切换瞬间，延迟可能导致"以为着地但实际在空中" → 摔倒
```

> ⚠️ **编程陷阱**：MPC 线程的异常处理
>
> **错误做法**：MPC 线程抛出异常后直接终止，Controller 线程不知道
>
> **现象**：MPC 求解失败后，Controller 继续用旧策略运行，但策略越来越过时——机器人开始发抖，最终摔倒
>
> **根本原因**：MPC 线程和 Controller 线程通过 Triple Buffer 松耦合，没有异常传播机制
>
> **正确做法**：legged_control 在 MPC 线程外包了 `try-catch`，异常时设置 `controllerRunning_ = false`，Controller 检测到后安全停机

**练习 68.3.A** ⭐⭐：在 `update()` 中，如果 `mpcMrtInterface_->updatePolicy()` 发现 Triple Buffer 中没有新策略（MPC 还没算完第一次），会发生什么？阅读 OCS2 源码找答案。

**练习 68.3.B** ⭐⭐：修改 `update()` 方法，增加一个计时统计功能：记录每个 Phase 的耗时，每 1000 次循环打印一次平均值。讨论这个计时本身会带来多少开销。

---

## 68.4 MPC 集成——如何包装 OCS2 ⭐⭐

### 68.4.1 从 OCS2 示例到 legged_control 的 MPC

上节看到 `update()` 如何查询 MPC 策略,本节深入看 MPC 如何被集成。理解这部分需要回顾 Ch55 的双线程架构。

OCS2 的 `ocs2_legged_robot` 示例使用**跨进程**架构（MPC_ROS_Interface + MRT_ROS_Interface），MPC 和 Controller 分别是两个 ROS 节点，通过 ROS 话题通信。

legged_control 使用**同进程**架构（MPC_MRT_Interface），MPC 和 Controller 在同一个进程的不同线程中，通过 Triple Buffer 通信。

| 架构 | OCS2 示例 | legged_control |
|------|----------|---------------|
| 通信方式 | ROS 话题（序列化+反序列化） | Triple Buffer（零拷贝内存共享） |
| 延迟 | ~1-5 ms（网络栈+序列化） | ~0.01 ms（原子指针交换） |
| 部署复杂度 | 两个节点要分别启动 | 一个节点内部管理 |
| 可扩展性 | 可以分布在不同机器上 | 必须在同一机器上 |
| 实时性 | 受 ROS 话题调度影响 | 不经过 ROS 调度，延迟确定 |

**为什么 legged_control 选择同进程？** 因为真机控制对延迟极度敏感。ROS 话题的 1-5 ms 延迟在 1 kHz 控制循环中是不可接受的——相当于每次都用 1-5 个周期前的策略。

> **跨领域类比**：同进程 vs 跨进程的架构选择,就像计算机体系结构中"L1 缓存 vs 主存"的权衡。ROS 话题通信如同访问主存——通用、灵活、但延迟大（~1 ms）; Triple Buffer 如同访问 L1 缓存——局限于同一进程、但延迟极小（~0.01 ms）。在 1 kHz 控制循环中,每个周期只有 1 ms 的预算,如果通信就吃掉了全部预算,就像 CPU 每次操作都要等主存一样——系统会被 I/O 而非计算拖慢。legged_control 选择"缓存路线",牺牲了分布式部署能力,换取了确定性低延迟。

### 68.4.2 MPC 线程的启动与运行

```cpp
// legged_controllers/src/LeggedController.cpp

void LeggedController::setupMpc() {
  // 设置 MPC-MRT 接口
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(
      *mpc_, leggedInterface_->getRolloutPtr());

  // 启动 MPC 线程
  controllerRunning_ = true;
  mpcThread_ = std::thread([this]() {
    // 设置线程优先级（SCHED_FIFO, 优先级低于 Controller 线程）
    setThreadPriority(mpcThreadPriority_);

    while (controllerRunning_) {
      try {
        executeAndSleep(
            [this]() { mpcMrtInterface_->advanceMpc(); },
            mpcDesiredFrequency_);  // 典型值: 50-100 Hz
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[MPC Thread] " << e.what());
      }
    }
  });
}
```

**`executeAndSleep()` 的作用**：OCS2 提供的工具函数,执行 lambda 后 sleep 到目标周期。例如 MPC 求解花了 15 ms，目标频率 50 Hz（20 ms 周期），则 sleep 5 ms。如果求解花了 25 ms（超时），不 sleep，立即开始下一次求解——这就是"尽力而为"的调度策略。

### 68.4.3 MPC 观测与策略的交互

Controller 线程和 MPC 线程之间的数据交互通过 `MPC_MRT_Interface` 完成（Ch55 详细讲过 Triple Buffer 机制）：

```
Controller 线程（1 kHz）        MPC 线程（50 Hz）
─────────────────               ─────────────────
setCurrentObservation() ──→     读取 observation
       │                              │
       │                        advanceMpc()
       │                         ├─ 线性化
       │                         ├─ SQP 求解
       │                         └─ 写入 Triple Buffer
       │                              │
updatePolicy() ←────────────    (写完后原子交换)
evaluatePolicy() ←──────        (在当前时刻插值)
       │
       ▼
  optimizedState
  optimizedInput
  plannedMode
```

**`evaluatePolicy()` 做了什么？** MPC 输出的是一条从当前到未来 1 秒的轨迹（离散时间序列）。Controller 需要的是"此刻"的参考值。`evaluatePolicy()` 在轨迹上做线性插值：

$$x_{ref}(t) = x_k + \frac{t - t_k}{t_{k+1} - t_k} (x_{k+1} - x_k) \tag{68.2}$$

其中 $t_k \le t < t_{k+1}$ 是 MPC 轨迹的相邻两个时间点。

> 💡 **设计洞察**：为什么线性插值就够了？因为 MPC 的采样间隔通常是 10-20 ms，在这么短的时间内，质心轨迹近似线性。如果采样间隔更长（如 50 ms），可能需要样条插值——但那意味着 MPC 预测步数太少，应该增加步数而非改善插值。

### 68.4.4 LeggedInterface——OCS2 问题定义

`LeggedInterface` 是 legged_control 与 OCS2 之间的"翻译层"——它把"四足机器人的控制任务"翻译成 OCS2 能理解的"最优控制问题(OCP)"。

```cpp
void LeggedInterface::setupOptimalControlProblem(
    const std::string& taskFile, const std::string& urdfFile,
    const std::string& referenceFile, bool verbose) {

  // ===== 动力学模型 =====
  // Centroidal Model: 状态 = [CoM位置, 欧拉角, CoM速度, 角动量, 关节角]
  // 输入 = [接触力(3x4), 关节速度(12)]
  problemPtr_->dynamicsPtr = std::make_unique<CentroidalModelRbdConversions>(
      pinocchioInterface_, centroidalModelInfo_);

  // ===== 代价函数 =====
  // 追踪参考轨迹 + 输入正则化
  problemPtr_->costPtr->add("base_tracking", ...);
  problemPtr_->costPtr->add("input_regularization", ...);

  // ===== 等式约束 =====
  // Stance 腿: 足端速度 = 0（不滑）
  problemPtr_->equalityConstraintPtr->add("zero_velocity", ...);
  // Swing 腿: 接触力 = 0（不受力）
  problemPtr_->equalityConstraintPtr->add("zero_force", ...);

  // ===== 不等式约束 =====
  // 摩擦锥: 接触力在锥内
  problemPtr_->inequalityConstraintPtr->add("friction_cone", ...);
  // 关节限位
  problemPtr_->inequalityConstraintPtr->add("joint_limits", ...);
}
```

**关键理解**：这个函数定义了 MPC 要解的问题——"在满足动力学、不滑、摩擦锥等约束下，找到使代价最小的质心轨迹和接触力"。这正是 Ch55 讲的 OCS2 OCP 在四足场景的具体实例化。

> ⚠️ **概念误区**：MPC 的决策变量不包含关节扭矩
>
> **新手想法**："MPC 直接算出关节扭矩不是更好吗？"
>
> **实际上**：Centroidal Model 的决策变量是 [接触力(12维) + 关节速度(12维)] = 24 维。关节扭矩由 WBC 层计算。如果 MPC 直接优化关节扭矩，状态维度要包含全身 $q, \dot{q}$（36 维），加上约束（摩擦锥 + 关节限位 + 接触不滑），QP 维度翻倍。在 50 Hz 频率下，这会导致求解超时。
>
> 这就是 Ch53 讲的"分层的价值"——MPC 用简化模型快速规划，WBC 用全身模型精确执行。

**练习 68.4.A** ⭐⭐：打开 legged_control 的 `config/a1/task.info` 文件，找到 MPC 的预测时间长度和采样步数。计算每步的时间间隔。讨论增大预测步数的利弊。

**练习 68.4.B** ⭐⭐：如果要把 MPC 的求解频率从 50 Hz 提高到 100 Hz，需要修改哪些配置？提高频率后 MPC 的求解窗口变短,有什么影响？

---

## 68.5 WBC 集成——HierarchicalWbc 精读 ⭐⭐

### 68.5.1 从 Ch53 到 legged_control 的 WBC

Ch53 已经详细推导了 WBC 的数学形式——全身动力学方程、QP 公式化、HQP 分层优化（详见 53.2-53.4 节）。本节聚焦于 legged_control 如何**工程实现**这些数学。

legged_control 的 WBC 有两个实现——`HierarchicalWbc`（默认）和 `WeightedWbc`（备选）。我们精读默认的 `HierarchicalWbc`。

### 68.5.2 WbcBase——任务公式化

> **跨领域类比**：WbcBase 的角色就像律师起草合同——它把客户的意图（"机器人要站稳、腿要到指定位置、不能打滑"）翻译成严格的法律条款（QP 的等式/不等式约束矩阵 $A, b, D, f$）。律师不负责执行合同（不求解 QP），只确保条款准确表达了意图。如果律师（WbcBase）把某个约束的符号写反了，法官（QP 求解器）会忠实地执行一个错误的裁决——机器人可能朝相反方向移动，而 QP 求解器不会报错，因为它只知道"矩阵运算正确"，不知道"物理意义错误"。

`WbcBase` 负责把"控制目标"翻译成 QP 的矩阵形式。它**不求解** QP,只负责组装矩阵。

```cpp
// legged_wbc/src/WbcBase.cpp

// 六个任务公式化方法
Task WbcBase::formulateFloatingBaseEomTask();    // 浮动基座动力学
Task WbcBase::formulateTorqueLimitsTask();       // 扭矩限制
Task WbcBase::formulateNoContactMotionTask();     // 非接触脚运动
Task WbcBase::formulateFrictionConeTask();        // 摩擦锥
Task WbcBase::formulateBaseAccelTask();           // 基座加速度追踪
Task WbcBase::formulateSwingLegTask();            // 摆动腿追踪
Task WbcBase::formulateContactForceTask();        // 接触力追踪
```

**每个 Task 返回什么？** 一个 `Task` 结构包含等式约束 $(A, b)$ 和不等式约束 $(D, f)$：

$$Ax = b \quad \text{(等式)} \qquad Dx \le f \quad \text{(不等式)} \tag{68.3}$$

其中决策变量 $x = [\ddot{q}_{18}, \lambda_{12}, \tau_{12}]^T$ 共 42 维。

**核心任务详解**：

**任务 1: 浮动基座动力学（等式约束）**

这是 WBC 最重要的约束——全身动力学方程的前 6 行：

$$M_b \ddot{q} + h_b = J_c^T \lambda \tag{68.4}$$

在 WbcBase 中的实现：

```cpp
Task WbcBase::formulateFloatingBaseEomTask() {
  // A = [M, -J_c^T, -S^T]，b = -h
  // 其中 M 来自 Pinocchio 的 crba()
  // h 来自 nonLinearEffects()
  // J_c 来自 getFrameJacobian()

  matrix_t a(info_.generalizedCoordinatesNum,
             numDecisionVars_);
  // 填入质量矩阵
  a.leftCols(info_.generalizedCoordinatesNum) = data.M;
  // 填入 -J_c^T
  a.middleCols(info_.generalizedCoordinatesNum,
               3 * info_.numThreeDofContacts) = -j_.transpose();
  // 填入 -S^T
  a.rightCols(info_.actuatedDofNum) = -s_.transpose();

  vector_t b = -data.nle;  // 非线性效应(科氏力+重力)的负值

  return {a, b, matrix_t(), vector_t()};
}
```

**任务 2: 摩擦锥（不等式约束）**

每个接触脚有 5 个不等式约束（线性化的摩擦金字塔,而非圆锥）：

$$\begin{cases}
-f_z \le 0 & \text{(法向力非负)} \\
f_x - \mu f_z \le 0 & \\
-f_x - \mu f_z \le 0 & \\
f_y - \mu f_z \le 0 & \\
-f_y - \mu f_z \le 0 &
\end{cases} \tag{68.5}$$

```cpp
Task WbcBase::formulateFrictionConeTask() {
  matrix_t frictionPyramid(5, 3);
  frictionPyramid << 0,  0, -1,          // -f_z <= 0
                      1,  0, -frictionCoeff_,  // f_x - mu*f_z <= 0
                     -1,  0, -frictionCoeff_,  // ...
                      0,  1, -frictionCoeff_,
                      0, -1, -frictionCoeff_;
  // 对每个接触脚,把 frictionPyramid 放到对应位置
  // d 矩阵的非零块在 lambda 对应的列
  return {matrix_t(), vector_t(), d, f};
}
```

> 💡 **对照 Ch53**：Ch53 的 53.3.3 节详细推导了为什么用线性金字塔而非 SOC 锥。简单回顾：线性化后可以用普通 QP 求解器（qpOASES），不需要 SOCP 求解器。5 面金字塔近似的误差约为 3%，对控制来说足够精确。

### 68.5.3 HierarchicalWbc——三层优先级求解

`HierarchicalWbc` 在 `WbcBase` 组装的任务矩阵基础上,用 HQP 分层求解。

```cpp
// legged_wbc/src/HierarchicalWbc.cpp

vector_t HierarchicalWbc::update(
    const vector_t& stateDesired,
    const vector_t& inputDesired,
    const vector_t& rbdStateMeasured,
    size_t mode, scalar_t period) {

  // 更新 Pinocchio 状态(计算 M, h, J 等)
  WbcBase::update(stateDesired, inputDesired,
                   rbdStateMeasured, mode, period);

  // ===== 第 0 层(硬约束): 物理定律 =====
  Task task0 = formulateFloatingBaseEomTask()   // 动力学
             + formulateTorqueLimitsTask()       // 扭矩限制
             + formulateFrictionConeTask()       // 摩擦锥
             + formulateNoContactMotionTask();    // 非接触约束

  // ===== 第 1 层(运动追踪): 轨迹跟踪 =====
  Task task1 = formulateBaseAccelTask()     // 基座追踪
             + formulateSwingLegTask();      // 摆动腿追踪

  // ===== 第 2 层(力分配): 接触力参考 =====
  Task task2 = formulateContactForceTask(); // MPC 期望力

  // ===== 分层 QP 求解 =====
  // 第 0 层: 求解满足硬约束的最优 x0
  HoQp hoQp0(task0, nullptr);
  // 第 1 层: 在 task0 的零空间内优化 task1
  HoQp hoQp1(task1, &hoQp0);
  // 第 2 层: 在 task0+task1 的零空间内优化 task2
  HoQp hoQp2(task2, &hoQp1);

  // 提取最终解
  vector_t x = hoQp2.getSolution();
  // x = [ddq(18), lambda(12), tau(12)]

  // 返回 WBC 命令: 关节前馈扭矩(最后 12 维)
  return x;
}
```

**三层优先级的物理意义**：

| 层级 | 任务 | 物理意义 | 违反后果 |
|------|------|---------|---------|
| **Level 0**（硬约束） | 动力学 + 扭矩限制 + 摩擦锥 | 物理定律不可违反 | 电机烧毁 / 足底打滑 |
| **Level 1**（运动追踪） | 基座 + 摆动腿追踪 | 跟踪 MPC 参考轨迹 | 偏离期望姿态,但仍安全 |
| **Level 2**（力分配） | 接触力参考 | 尽量按 MPC 建议分配力 | 力分配不均,但运动不受影响 |

**为什么接触力追踪放在最低优先级？** 因为 MPC 用简化模型(Centroidal)计算的接触力只是"大致参考"——实际的接触几何、地面刚度等在简化模型中被忽略了。WBC 用全身模型重新计算的力更准确。所以 MPC 的力参考只作为"柔性目标",不作为硬约束。

### 68.5.4 HoQp——分层 QP 数据结构

`HoQp`（Hierarchical Optimization QP）是实现零空间投影的核心类。

```cpp
// legged_wbc/src/HoQp.cpp

class HoQp {
 public:
  HoQp(const Task& task, HoQp* prevHoQp);
  vector_t getSolution() const;

 private:
  // 零空间投影矩阵
  matrix_t stackedZPrev_;   // 上层所有任务的零空间
  matrix_t stackedTaskPrev_; // 上层所有约束的堆叠

  // 当前层的 QP
  // min ||A * Z_prev * x_slack - (b - A * x_prev)||^2
  // s.t. D * Z_prev * x_slack <= f - D * x_prev
};
```

**零空间投影的直觉理解**（回顾 Ch53）：

假设第 0 层的最优解为 $x_0^*$,其等式约束 $A_0 x = b_0$ 的零空间为 $Z_0$（$A_0 Z_0 = 0$）。第 1 层的优化变量不是 $x$,而是 $\delta x \in \text{range}(Z_0)$：

$$x_1 = x_0^* + Z_0 \delta x_1 \tag{68.6}$$

这保证了第 1 层的修正 **不影响** 第 0 层已满足的约束——因为 $A_0 x_1 = A_0 x_0^* + A_0 Z_0 \delta x_1 = b_0 + 0 = b_0$。

> ⚠️ **编程陷阱**：HoQp 的零空间计算使用 SVD,时间复杂度 $O(n^3)$
>
> **现象**：在决策变量较多(如人形机器人 ~100 维)的场景下,WBC 求解时间可能超过 1 ms
>
> **根本原因**：SVD 是 $O(n^3)$ 操作,零空间矩阵 $Z$ 的列数随层数增加而减少,但 SVD 本身的开销不变
>
> **替代方案**：
> 1. 使用完全正交分解(Complete Orthogonal Decomposition)代替 SVD
> 2. 使用 ProxQP 的分层求解模式（不需要显式零空间）
> 3. 使用加权 QP + 极大权重近似分层（`WeightedWbc` 的做法）

**练习 68.5.A** ⭐⭐：在 HierarchicalWbc 中,把 Level 1 的 `formulateSwingLegTask()` 移到 Level 0（和硬约束放一起）。预测会发生什么,然后在仿真中验证。

**练习 68.5.B** ⭐⭐⭐：比较 `HierarchicalWbc` 和 `WeightedWbc` 在 trot 步态下的表现差异。画出基座高度和俯仰角的时间曲线。讨论在什么场景下一个优于另一个。

---

## 68.6 步态管理器 ⭐⭐

### 68.6.1 步态在 legged_control 中的角色

在前面几节中,我们多次看到 `plannedMode` 这个变量——它编码了"哪些脚在地上"。这个信息来自**步态管理器**。步态管理器是 MPC 和 WBC 的上游模块,它告诉控制器"何时抬腿、何时着地"。

### 68.6.2 ModeSchedule 与接触编码

OCS2 用一个整数编码接触模式：4 个脚各有 2 种状态(stance/swing),共 $2^4 = 16$ 种模式。

```
接触模式编码(二进制 → 十进制):
  LF RF LH RH        (OCS2/legged_control 惯例,注意与 Unitree FL/FR/RL/RR 命名不同)
  1  1  1  1  = 15 (全着地 = stance)
  0  1  0  1  = 5  (LF+LH 摆动 = trot 的一半)
  1  0  1  0  = 10 (RF+RH 摆动 = trot 的另一半)
  0  0  0  0  = 0  (全摆动 = 腾空)
  1  1  0  0  = 12 (前两腿着地,后两腿摆动 = bound)
```

`ModeSchedule` 是一个时间序列,记录了未来一段时间内的模式切换：

```cpp
struct ModeSchedule {
  scalar_array_t eventTimes;  // 切换时刻: [t1, t2, t3, ...]
  size_array_t modeSequence;  // 模式序列: [6, 9, 6, 9, ...]  (trot: 对角交替)
  //                             t<t1: mode=6  (RF+LH stance)
  //                             t1<=t<t2: mode=9  (LF+RH stance)
  //                             t2<=t<t3: mode=6 ...
};
```

### 68.6.3 GaitSchedule 实现

legged_control 复用了 OCS2 的 `GaitSchedule` 类,通过 `SwitchedModelReferenceManager` 与 MPC 集成。

```cpp
// 步态定义在 config/a1/gait.info 中
// 典型配置:
{
  list
  {
    [0] stance    // 站立
    [1] trot      // 对角步态
    [2] flying_trot  // 带腾空的快 trot
    [3] pace      // 同侧步态
    [4] bounding  // 前后腿交替
  }
  trot
  {
    cycleDuration      0.5   // 步态周期(秒)
    switchingTimes     [0.0, 0.5]  // 一个周期内的切换时刻
    modeSequence       [6, 9]      // 模式序列(6=RF+LH对角, 9=LF+RH对角)
  }
}
```

**Trot 步态的时序详解**：

```
周期 = 0.5 秒
                   时间(秒)
0.0    0.125   0.25    0.375   0.5     0.625   0.75
│      │       │       │       │       │       │
├──────┼───────┼───────┼───────┤       │       │
│   mode=6     │   mode=9      │ mode=6 │mode=9 │
│  RF+LH stance│  LF+RH stance│ ...    │  ...  │
│  LF+RH swing │  RF+LH swing │        │       │
│              │               │        │       │

LF: ░░░░░░░░░░░████████████████░░░░░░░░░░░█████
RF: ████████████░░░░░░░░░░░░░░░████████████░░░░░
LH: ████████████░░░░░░░░░░░░░░░████████████░░░░░
RH: ░░░░░░░░░░░████████████████░░░░░░░░░░░█████
    █ = stance(着地)  ░ = swing(摆动)
```

### 68.6.4 步态切换的命令接口

用户通过 ROS 话题发送步态切换命令：

```bash
# 切换到 trot 步态
rostopic pub /gait_command std_msgs/String "data: 'trot'"

# 切换到 stance(站立)
rostopic pub /gait_command std_msgs/String "data: 'stance'"
```

在代码中,`GaitReceiver` 订阅这个话题,更新 `SwitchedModelReferenceManager` 中的步态计划：

```cpp
class GaitReceiver {
  void gaitCallback(const std_msgs::String::ConstPtr& msg) {
    // 查找步态名 → ModeSchedule
    const auto& gait = gaitList_.at(msg->data);
    // 从当前时刻开始,用新步态生成未来的 ModeSchedule
    referenceManager_->setModeSchedule(
        gait.toModeSchedule(currentTime_));
  }
};
```

> ⚠️ **编程陷阱**：步态切换不是"立即生效"
>
> **错误做法**：在 trot 过程中突然切换到 stance，期望机器人立刻停下来
>
> **现象**：如果切换发生在摆动相中期，某条腿正在空中——突然要求它 stance（着地），但腿还没落地
>
> **根本原因**：步态切换需要等当前步态周期的合适时机才能生效（通常是下一次全着地时刻）
>
> **正确做法**：legged_control 的 `GaitSchedule` 会把新步态追加到当前 ModeSchedule 的末尾，而不是替换。MPC 在下一次求解时自然过渡

> 💡 **概念澄清**：legged_control 的步态管理是"固定模式序列"型的,不是"自适应选脚"型的。也就是说,步态的时间表是预定义的（trot 每 0.25s 切换一次），不会根据地形或扰动自动调整。自适应选脚属于研究前沿（如 Raibert 式启发选脚、RL 选脚），需要额外模块。

**练习 68.6.A** ⭐⭐：修改 `gait.info`，创建一个新步态 "gallop"（对角线不同步的跑步），定义其 `cycleDuration`、`switchingTimes` 和 `modeSequence`。在 Gazebo 中测试。

**练习 68.6.B** ⭐⭐：分析 trot 步态中如果 `cycleDuration` 从 0.5 秒减小到 0.2 秒会发生什么。从 MPC 预测步数、WBC 追踪带宽、机械限制三个角度讨论。

---

## 68.7 硬件接口——从抽象到真机 ⭐⭐

### 68.7.1 ros_control 的 HardwareInterface 回顾

在 Ch62 中我们学习了 ros_control 的架构——Controller 通过 HardwareInterface 与硬件解耦。legged_control 的硬件层继承这个架构，提供了两种 HardwareInterface 实现。

> **设计决策**：ros_control 框架的控制循环不使用 ROS 话题通信，而是通过内存指针直接读写硬件接口数据。这是因为 ROS 话题通信涉及序列化、网络传输和反序列化，会引入不确定延迟和抖动，对于 500-1000 Hz 的控制循环不可接受。legged_control 中整个控制环路（`read() → update() → write()`）内部没有任何 ROS 话题通信，只有对外部的非实时接口（如接收用户目标点、步态切换命令）才使用 ROS 话题。这种设计严格地将"实时"与"非实时"任务分离开来。

以下是两种 HardwareInterface 实现：

| 实现 | 用途 | 底层 |
|------|------|------|
| `LeggedHW` (基类) | 定义接口规范 | 抽象类 |
| `UnitreeHW` | Unitree A1 真机 | Unitree Legged SDK (UDP) |
| Gazebo plugin | Gazebo 仿真 | ros_control gazebo 插件 |

### 68.7.2 HybridJointInterface——混合控制模式

legged_control 定义了一个自定义的硬件接口——`HybridJointInterface`，支持同时下发位置、速度、Kp、Kd 和扭矩五个量。

```cpp
// legged_common/include/legged_common/hardware_interface/HybridJointInterface.h

class HybridJointHandle {
 public:
  // 读取
  double getPosition() const;
  double getVelocity() const;
  double getEffort() const;

  // 写入: 五自由度命令
  void setCommand(double posDes, double velDes,
                   double kp, double kd, double ff) {
    // 电机端执行: tau = kp*(posDes - pos) + kd*(velDes - vel) + ff
    *posDesPtr_ = posDes;
    *velDesPtr_ = velDes;
    *kpPtr_ = kp;
    *kdPtr_ = kd;
    *ffPtr_ = ff;
  }
};
```

**为什么需要自定义接口？** 标准的 `EffortJointInterface` 只支持单一扭矩命令，`PositionJointInterface` 只支持位置命令。而 Unitree 电机的混合模式需要同时下发 5 个量。

**五参数的物理意义**（对应公式 68.1）：

| 参数 | 来源 | 物理意义 |
|------|------|---------|
| `posDes` | MPC 参考轨迹 | 期望关节角度,PD 反馈的位置目标 |
| `velDes` | MPC 参考轨迹 | 期望关节速度,PD 反馈的速度目标 |
| `kp` | 配置文件 / WBC | 位置增益,控制刚度 |
| `kd` | 配置文件 / WBC | 速度增益,控制阻尼 |
| `ff` | WBC 前馈扭矩 | 补偿重力/科氏力/接触力的主要贡献 |

### 68.7.3 UnitreeHW 精读

```cpp
// legged_examples/legged_unitree/legged_unitree_hw/src/UnitreeHW.cpp

class UnitreeHW : public LeggedHW {
 public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override {
    // Step 1: 初始化 Unitree SDK
    udp_ = std::make_unique<UNITREE_LEGGED_SDK::UDP>(
        UNITREE_LEGGED_SDK::LOWLEVEL);
    // Step 2: 设置安全级别
    safety_ = std::make_unique<UNITREE_LEGGED_SDK::Safety>(
        UNITREE_LEGGED_SDK::LeggedType::A1);
    // Step 3: 注册关节到 ros_control
    for (int i = 0; i < 12; ++i) {
      hardware_interface::JointStateHandle stateHandle(
          jointNames[i], &jointData_[i].pos_,
          &jointData_[i].vel_, &jointData_[i].tau_);
      jointStateInterface_.registerHandle(stateHandle);
      // 注册 HybridJointHandle ...
    }
  }

  void read(const ros::Time& time, const ros::Duration& period) override {
    // 从 UDP 缓冲区读取电机状态
    udp_->Recv();
    udp_->GetRecv(lowState_);

    for (int i = 0; i < 12; ++i) {
      jointData_[i].pos_ = lowState_.motorState[i].q;
      jointData_[i].vel_ = lowState_.motorState[i].dq;
      jointData_[i].tau_ = lowState_.motorState[i].tauEst;
    }
    // IMU 数据
    imuData_.ori_[0] = lowState_.imu.quaternion[1]; // x
    imuData_.ori_[1] = lowState_.imu.quaternion[2]; // y
    imuData_.ori_[2] = lowState_.imu.quaternion[3]; // z
    imuData_.ori_[3] = lowState_.imu.quaternion[0]; // w
    imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
    imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
    imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
    imuData_.linearAccel_[0] = lowState_.imu.accelerometer[0];
    imuData_.linearAccel_[1] = lowState_.imu.accelerometer[1];
    imuData_.linearAccel_[2] = lowState_.imu.accelerometer[2];
  }

  void write(const ros::Time& time, const ros::Duration& period) override {
    for (int i = 0; i < 12; ++i) {
      lowCmd_.motorCmd[i].q   = jointData_[i].posDes_;
      lowCmd_.motorCmd[i].dq  = jointData_[i].velDes_;
      lowCmd_.motorCmd[i].Kp  = jointData_[i].kp_;
      lowCmd_.motorCmd[i].Kd  = jointData_[i].kd_;
      lowCmd_.motorCmd[i].tau = jointData_[i].ff_;
    }
    // 安全检查: 限制扭矩和速度
    safety_->PositionLimit(lowCmd_);
    safety_->PowerProtect(lowCmd_, lowState_, 1);

    udp_->SetSend(lowCmd_);
    udp_->Send();
  }
};
```

> ⚠️ **编程陷阱**：Unitree SDK 的四元数顺序与 Eigen 不同
>
> **错误做法**：直接把 `lowState_.imu.quaternion[0-3]` 按顺序赋给 `Eigen::Quaterniond`
>
> **现象**：机器人在 yaw 方向疯狂旋转,或者 pitch 反向
>
> **根本原因**：Unitree SDK 的四元数顺序是 `[w, x, y, z]`,而 Eigen 构造函数的参数顺序是 `Quaterniond(w, x, y, z)`,但 `coeffs()` 返回的顺序是 `[x, y, z, w]`。ROS 的 `geometry_msgs::Quaternion` 也是 `[x, y, z, w]`
>
> **自检方法**：打印四元数的 norm,如果不等于 1.0 说明顺序搞反了

### 68.7.4 Sim2Real 的架构设计

Controller 对硬件完全透明——这是 Sim2Real 的关键：

```
┌─────────────────────────┐
│     LeggedController     │
│   (不知道底层是什么)     │
│                         │
│   read() → 传感器数据    │
│   write() → 关节命令     │
└────────────┬────────────┘
             │ HybridJointInterface (抽象)
  ┌──────────┴──────────┐
  │                     │
  ▼                     ▼
┌──────────┐    ┌────────────┐
│ UnitreeHW│    │ Gazebo     │
│ (真机)   │    │ Plugin     │
│ UDP SDK  │    │ (仿真)     │
└──────────┘    └────────────┘
```

只要仿真和真机提供相同的 `HybridJointInterface`，Controller 代码不需要任何修改。这就是 ros_control 的"硬件抽象"设计的价值。

切换仿真和真机的方式很简单——只需要在 launch 文件中指定不同的 HardwareInterface：

```xml
<!-- 仿真模式: 使用 Gazebo 插件 -->
<param name="hardware_interface/type"
       value="gazebo_ros_control/DefaultRobotHWSim"/>

<!-- 真机模式: 使用 UnitreeHW -->
<param name="hardware_interface/type"
       value="legged_unitree_hw/UnitreeHW"/>
```

> 🧠 **思维陷阱**：不要认为 Sim2Real 只是"换一个 HardwareInterface"
>
> **新手想法**："仿真跑通了,换上 UnitreeHW 就能在真机上跑了吧？"
>
> **实际上**：仿真到真机的 gap 还包括：
> 1. **传感器噪声**——仿真的 IMU 是完美的,真机有 bias 和噪声
> 2. **通信延迟**——仿真是零延迟,真机 UDP 有 ~0.5 ms 延迟
> 3. **模型误差**——Pinocchio 用的惯性参数与真机有差距
> 4. **执行器动力学**——仿真电机是理想力源,真机有力矩带宽限制
> 5. **初始化流程**——真机需要"先锁关节、再慢慢解锁"的安全启动序列

**练习 68.7.A** ⭐⭐：Unitree SDK 的 `safety_->PowerProtect()` 做了什么？阅读 SDK 源码，找到它的限制逻辑。讨论在什么场景下这个安全保护会被触发。

**练习 68.7.B** ⭐⭐：如果要支持一个新的电机品牌（比如 CubeMars），需要实现哪些函数？写出 `CubeMarsHW` 类的框架代码。

### 68.7.5 仿真硬件抽象链——LeggedHWSim 精读

68.7.4 节展示了 Controller 如何通过 `HybridJointInterface` 与真机/仿真解耦。本节深入仿真侧的实现——当 Controller 以为自己在和真机对话时，仿真端到底发生了什么？

#### 整体架构

在仿真模式下，`LeggedHWSim` 类充当硬件的"替身"——它继承自 `gazebo_ros_control::DefaultRobotHWSim`，在 Gazebo 物理引擎和 ros_control 之间架起桥梁。整条数据链路如下：

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    仿真硬件抽象链（一个控制周期）                         │
│                                                                          │
│  Gazebo 物理引擎                                                         │
│  ┌─────────────────────────────────────────────────────────┐             │
│  │  Joint 物理状态   │  Link 姿态/角速度  │  碰撞检测结果   │             │
│  └────────┬──────────┴────────┬───────────┴───────┬────────┘             │
│           │                   │                   │                       │
│           ▼                   ▼                   ▼                       │
│  ┌────────────────────────────────────────────────────────────┐          │
│  │              LeggedHWSim::readSim()                        │          │
│  │  ┌──────────────┐  ┌─────────────────┐  ┌──────────────┐  │          │
│  │  │ 关节位置/速度 │  │ IMU 姿态/角速度  │  │ 足端接触标志 │  │          │
│  │  │ → jointData  │  │ → imuData       │  │ → contactFlag│  │          │
│  │  └──────┬───────┘  └───────┬─────────┘  └──────┬───────┘  │          │
│  └─────────┼──────────────────┼────────────────────┼──────────┘          │
│            │ (C++ 指针)       │ (C++ 指针)         │ (C++ 指针)          │
│            ▼                  ▼                    ▼                      │
│  ┌────────────────────────────────────────────────────────────┐          │
│  │  HybridJointInterface  │  ImuSensorInterface  │  Contact  │          │
│  │     (句柄层 — 零拷贝指针访问)                              │          │
│  └────────────────────────────┬───────────────────────────────┘          │
│                               │                                          │
│                               ▼                                          │
│  ┌────────────────────────────────────────────────────────────┐          │
│  │         LeggedController::update() @ 500-1000 Hz          │          │
│  │  状态估计 → MPC查询 → WBC求解 → 安全检查 → 写入命令       │          │
│  └────────────────────────────┬───────────────────────────────┘          │
│                               │                                          │
│                               ▼                                          │
│  ┌────────────────────────────────────────────────────────────┐          │
│  │              LeggedHWSim::writeSim()                       │          │
│  │  从 HybridJointHandle 读取 posDes/velDes/kp/kd/ff         │          │
│  │  执行 PD 混合控制: tau = kp*(posDes-pos) + kd*(velDes-vel) + ff      │
│  │  通过 EffortJointInterface 将 tau 写入 Gazebo 关节         │          │
│  └────────────────────────────────────────────────────────────┘          │
└──────────────────────────────────────────────────────────────────────────┘
```

**关键洞察**：Controller 只面对 `HybridJointInterface`、`ImuSensorInterface` 和 `ContactSensorInterface` 这三个标准接口，它完全不知道数据来自 Gazebo 还是真实传感器。这正是硬件抽象的核心价值。

#### transmission 与 EffortJointInterface

仿真中关节驱动的"底座"是 URDF 中的 `<transmission>` 标签。它建立了 Gazebo 物理关节与 ros_control 关节句柄之间的映射：

```xml
<!-- transmission.xacro: 为每个关节创建 EffortJointInterface -->
<transmission name="${prefix}_HAA_tran">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${prefix}_HAA">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="${prefix}_HAA_motor">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

`gazebo_ros_control` 在加载时扫描所有 `<transmission>` 标签，为每个声明了 `EffortJointInterface` 的关节创建一个 `JointHandle`。这些 Handle 只支持单一力矩写入——但四足机器人需要混合 PD+力矩控制。`LeggedHWSim` 的核心工作就是在此基础上"包装"出 `HybridJointInterface`。

#### LeggedHWSim 的包装逻辑

`LeggedHWSim::initSim()` 在初始化时完成从 `EffortJointInterface` 到 `HybridJointInterface` 的升级：

```cpp
// 简化的初始化逻辑
bool LeggedHWSim::initSim(...) {
  // 父类已经通过 transmission 创建了 EffortJointInterface
  DefaultRobotHWSim::initSim(...);

  // 获取所有已注册的 EffortJoint 名称
  auto jointNames = ej_interface_.getNames();

  for (const auto& name : jointNames) {
    // 为每个关节创建 HybridJointHandle
    // 内部用指针指向 jointData 的 pos/vel/effort 和 command 缓冲区
    HybridJointHandle handle(
        jointStateInterface_.getHandle(name),
        &jointData_[name].posDes_,
        &jointData_[name].velDes_,
        &jointData_[name].kp_,
        &jointData_[name].kd_,
        &jointData_[name].ff_);
    hybridJointInterface_.registerHandle(handle);
  }
  // 注册 IMU 和接触传感器接口(类似)
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);
}
```

#### 指针式零拷贝设计

硬件句柄的底层实现值得特别关注——它们使用**指针**而非值传递来交换数据：

```cpp
// HybridJointHandle 内部结构（简化）
class HybridJointHandle {
  // 状态读取：指向硬件层的状态缓冲区
  const double* posPtr_;   // 指向 jointData.pos_
  const double* velPtr_;   // 指向 jointData.vel_
  const double* effPtr_;   // 指向 jointData.effort_

  // 命令写入：指向硬件层的命令缓冲区
  double* posDesPtr_;      // 指向 jointData.posDes_
  double* velDesPtr_;      // 指向 jointData.velDes_
  double* kpPtr_;          // 指向 jointData.kp_
  double* kdPtr_;          // 指向 jointData.kd_
  double* ffPtr_;          // 指向 jointData.ff_
};
```

当 Controller 调用 `hybridJointHandles_[i].getPosition()` 时，它直接解引用一个指针读取数据——没有序列化、没有拷贝、没有 ROS 消息传递。同理，`setCommand()` 也只是写入内存地址。整个 `read() → update() → write()` 循环中的数据流都是**零拷贝**的指针操作。

#### IMU 与接触数据的同步读取

> 💡 **工程要点：为什么 IMU 数据不走 ROS 话题？**
>
> 仿真中完全可以为 IMU 挂一个 Gazebo 传感器插件（如 `libgazebo_ros_imu.so`），让它以 ROS 话题形式发布 IMU 数据。但 legged_control **没有这样做**。原因在于：
>
> ROS 话题通信会经历序列化、网络栈传输、反序列化的完整流程，这意味着数据有**不确定延迟和抖动**，并且是**异步**到达控制器的。对于 500-1000 Hz 的高频控制循环来说，这是不可接受的——控制器在 `update()` 时需要的 IMU 数据必须与关节角度、接触状态**精确对齐在同一时间戳**。
>
> 实际数据流是：`Gazebo 物理引擎 → LeggedHWSim::readSim()（直接读取连杆状态）→ ImuSensorInterface（C++ 共享内存）→ LeggedController::update()（直接内存访问）`。数据在 ros_control 的同一个 `read/update/write` 循环中被**同步读取**，不经过任何 ROS 通信层。

`readSim()` 中 IMU 数据的读取方式体现了这一设计：

```cpp
void LeggedHWSim::readSim(ros::Time time, ros::Duration period) {
  // 关节状态：从 Gazebo 关节直接读取
  for (int i = 0; i < nJoints; ++i) {
    jointData_[i].pos_ = sim_joints_[i]->Position(0);
    jointData_[i].vel_ = sim_joints_[i]->GetVelocity(0);
  }

  // IMU：直接从 Gazebo 连杆读取无噪声真值
  auto bodyLink = parent_model_->GetLink(imuLinkName_);
  auto pose = bodyLink->WorldPose();
  auto angVel = bodyLink->RelativeAngularVel();
  auto linAccel = bodyLink->RelativeLinearAccel();

  imuData_.ori_[0] = pose.Rot().X();  // 直接内存写入
  imuData_.ori_[1] = pose.Rot().Y();
  imuData_.ori_[2] = pose.Rot().Z();
  imuData_.ori_[3] = pose.Rot().W();
  // angularVel, linearAccel 同理...

  // 接触：直接查询 Gazebo 碰撞检测
  for (int i = 0; i < 4; ++i) {
    contactData_[i].isContact_ =
        !footLinks_[i]->GetContacts().empty();
  }
}
```

**注意**：仿真中 IMU 读取的是完全理想化的数据——无噪声、无 bias、无延迟。这是仿真到真机差距（Sim2Real Gap）的重要来源之一。真机的 IMU 需要经过滤波和 bias 校准后才能使用。

#### 插件注册与加载机制

`LeggedHWSim` 不是被直接实例化的，而是作为 Gazebo 插件被动态加载。整个过程分为四步：

**Step 1：编译时注册**（C++ 宏声明）

```cpp
// LeggedHWSim.cpp 末尾
PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros_control::RobotHWSim)
```

**Step 2：包级声明**（XML 描述文件）

```xml
<!-- legged_gazebo/legged_hw_sim_plugins.xml -->
<library path="lib/liblegged_hw_sim">
    <class name="legged_gazebo/LeggedHWSim"
           type="legged::LeggedHWSim"
           base_class_type="gazebo_ros_control::RobotHWSim">
    </class>
</library>
```

**Step 3：URDF 中指定**（在 `gazebo.xacro` 中）

```xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="liblegged_hw_sim.so">
        <robotSimType>legged_gazebo/LeggedHWSim</robotSimType>
    </plugin>
</gazebo>
```

**Step 4：运行时加载**——Gazebo 启动时解析 URDF，发现 `robotSimType` 指向 `LeggedHWSim`，通过 pluginlib 动态加载该类，调用 `initSim()`、`readSim()`、`writeSim()` 驱动仿真。

> ⚠️ **工程陷阱**：legged_control 默认配置假设关节名为 `LF_HAA`、`LF_HFE`、`LF_KFE` 等固定格式。如果自定义机器人使用不同的关节命名，需要同时修改配置文件和控制器代码中的名称映射，否则控制器无法正确输出力矩。具体来说，`LeggedHWSim::initSim()` 通过 `ej_interface_.getNames()` 获取所有注册了 `EffortJointInterface` 的关节列表——关节名来自 URDF 的 `<transmission>` 标签，必须与 `task.info` 中的 `jointNames` 完全一致。

---

## 68.8 可视化与调试 ⭐⭐

### 68.8.1 LeggedRobotVisualizer 的作用

调试腿足机器人是出了名的困难——12 个关节同时运动,接触力看不见摸不着,状态估计的偏差需要对比 ground truth。可视化工具是调试的生命线。

legged_control 的可视化由 `LeggedRobotVisualizer`（来自 OCS2）和自定义的 ROS 话题组成。

### 68.8.2 关键可视化话题

| 话题 | 消息类型 | 内容 | 用途 |
|------|---------|------|------|
| `/legged_robot/currentState` | `ocs2_msgs/MpcObservation` | 当前观测（状态+输入+模式） | 监控状态估计 |
| `/legged_robot/optimizedTrajectory` | `ocs2_msgs/MpcFlattenedController` | MPC 优化轨迹 | 可视化规划路径 |
| `/legged_robot/desiredTrajectory` | `ocs2_msgs/MpcTargetTrajectories` | 用户命令的目标轨迹 | 确认命令正确接收 |
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | 接触力箭头、足端轨迹、CoM 球 | RViz 3D 可视化 |
| `/tf` | `tf2_msgs/TFMessage` | 各关节和基座的坐标变换 | RViz 模型渲染 |

### 68.8.3 RViz 中的调试技巧

**技巧 1：观察接触力箭头**

OCS2 的 Visualizer 在 RViz 中为每个接触点画一个力箭头（MarkerArray 的 Arrow 类型），箭头长度正比于力大小，方向为力方向。

```
调试信号:
- 箭头突然消失 → 该脚失去接触,检查步态调度
- 箭头剧烈抖动 → 接触力不稳定,检查摩擦系数或 WBC 权重
- 箭头方向不垂直 → 可能有侧向滑动,检查摩擦锥约束
- 所有箭头长度相等 → MPC 的力分配太均匀,可能缺少真实地形信息
```

**技巧 2：对比 MPC 轨迹与实际轨迹**

同时可视化 MPC 规划的 CoM 轨迹（`optimizedTrajectory`）和状态估计的实际 CoM 位置（`currentState`）。如果两者偏差持续增大,说明：

1. 状态估计有偏差（最常见：yaw 漂移）
2. MPC 模型与实际不匹配（惯性参数错误）
3. WBC 无法跟踪 MPC 参考（QP 约束太紧）

**技巧 3：使用 `rqt_plot` 监控关键信号**

```bash
# 监控基座高度(应该约 0.3m)
rqt_plot /legged_robot/currentState/state[2]

# 监控 MPC 求解时间
rqt_plot /legged_robot/mpc_timer/data

# 监控关节扭矩(检查是否饱和)
rqt_plot /joint_states/effort[0] /joint_states/effort[3]
```

**技巧 4：系统性排障流程**

当机器人在仿真中"抖动"时，按以下顺序排查：

| 步骤 | 检查内容 | 方法 | 常见原因 |
|------|---------|------|---------|
| 1 | WBC QP 是否有解 | 打印 qpOASES 返回状态 | 约束矛盾(如摩擦系数设太小) |
| 2 | 状态估计是否准确 | 对比 Gazebo ground truth 和 KF 输出 | IMU 四元数顺序错误 |
| 3 | MPC 是否在运行 | 打印 MPC 求解频率 | MPC 线程崩溃但 Controller 不知道 |
| 4 | 步态调度是否正确 | 打印 plannedMode | 接触标志与实际不一致 |
| 5 | PD 增益是否合理 | 降低 Kp 到 0,只用扭矩前馈 | Kp 过大导致高频震荡 |

> 💡 **调试经验**："先排除基础设施问题,再排查算法问题"。90% 的 bug 来自坐标系错误、关节编号映射错误、四元数顺序错误——这些与 MPC/WBC 算法无关,但会导致整个系统看起来"算法不工作"。

**练习 68.8.A** ⭐：在 RViz 中配置 legged_control 的完整可视化界面。保存 `.rviz` 配置文件,包括机器人模型、接触力箭头、MPC 轨迹和 TF 树。

**练习 68.8.B** ⭐⭐：写一个 ROS 节点,订阅 `currentState` 和 `optimizedTrajectory`,计算实际 CoM 与 MPC 参考之间的追踪误差,以 Float64 话题发布。用 `rqt_plot` 画出误差曲线。

---

## 68.9 从 OCS2 example 到 legged_control——关键差异 ⭐⭐⭐

### 68.9.1 为什么要专门对比

很多学习者在读完 Ch55（OCS2）后直接跳到 legged_control,会困惑"legged_control 到底改了什么？还是只是 OCS2 的 wrapper？"。本节系统回答这个问题。

### 68.9.2 OCS2 legged_robot 示例的能力边界

OCS2 的 `ocs2_legged_robot` 是一个**算法演示**,它能做的是：

1. 在 Gazebo 中运行 ANYmal 的 MPC
2. 通过 ROS 话题发送速度命令
3. 用 OCS2 的可视化工具看 MPC 轨迹
4. 使用 Gazebo 的 ground truth 作为状态反馈

**它不能做的**（需要 legged_control 补齐的）：

| 能力 | OCS2 legged_robot | legged_control |
|------|-------------------|----------------|
| WBC 全身控制 | 无,MPC 直接输出关节前馈 | HierarchicalWbc + WeightedWbc |
| 状态估计 | 使用 Gazebo ground truth | LinearKalmanFilter |
| 真机部署 | 不支持 | UnitreeHW + Sim2Real |
| 安全检查 | 无 | SafetyChecker |
| 混合位置-力矩控制 | 无（纯扭矩前馈） | HybridJointInterface |
| 关节柔顺控制 | 无 | Kp/Kd 可配置 |
| 自定义步态命令 | 有（基础） | 增强的 GaitReceiver |

### 68.9.3 legged_control 的核心增量

**增量 1：WBC 层（~2000 行）**

这是最大的增量。OCS2 legged_robot 把 MPC 的输出（centroidal 状态 + 接触力参考）直接通过逆动力学转换为关节扭矩，没有考虑：
- 任务优先级（摩擦锥 vs 轨迹追踪）
- 关节扭矩限制的严格满足
- 摩擦锥约束的运动学级别满足

legged_control 的 WBC 用分层 QP 同时处理这些约束，保证安全性的同时最大化追踪精度。

**增量 2：状态估计（~500 行）**

`LinearKalmanFilter` 融合 IMU 和腿部运动学：

| 状态量 | 维度 | 来源 |
|-------|------|------|
| 基座位置 | 3 | KF 估计（IMU 加速度积分 + 腿部运动学校正） |
| 基座线速度 | 3 | KF 估计 |
| 四脚位置 | 12 | 正运动学（base + joint_angles） |
| **总计** | 18 | |

预测模型：基座做匀速运动（一阶近似），脚不动（stance 假设）。
观测模型：脚的位置 = 基座位置 + 正运动学。

这个 KF 的简洁性正是它的优势——300 行代码，足够用于教学和原型验证。但精度不如 InEKF（Ch57）。

**增量 3：硬件抽象（~800 行）**

`LeggedHW` 基类 + `UnitreeHW` 实现了完整的 Sim2Real 框架。HybridJointInterface 是关键创新——把 5 参数控制模式抽象为标准 ros_control 接口。

**增量 4：安全层（~200 行）**

`SafetyChecker` 检查三类安全条件：

```cpp
class SafetyChecker {
  bool check(const SystemObservation& obs,
             const vector_t& optimizedState,
             const vector_t& command) {
    // 1. 关节位置是否在限位内
    bool positionSafe = checkJointPositionLimits(command);
    // 2. 关节速度是否过快
    bool velocitySafe = checkJointVelocityLimits(command);
    // 3. 基座姿态是否合理(没有翻倒)
    bool orientationSafe = checkBaseOrientation(obs);
    return positionSafe && velocitySafe && orientationSafe;
  }
};
```

### 68.9.4 架构对比图

```
OCS2 legged_robot 示例:
┌──────────┐     ┌──────────────┐     ┌────────────┐
│ ROS 命令  │────>│ MPC (SQP)    │────>│ 逆动力学   │──> Gazebo
│ (话题)   │     │ (独立节点)   │     │ (直接扭矩) │
└──────────┘     └──────────────┘     └────────────┘
                  使用 ground truth

legged_control:
┌──────────┐     ┌────────────────────────────────────────┐
│ ROS 命令  │────>│          LeggedController               │
│ (话题)   │     │  ┌──────┐  ┌─────┐  ┌─────┐  ┌──────┐  │
└──────────┘     │  │State │─>│ MPC │─>│ WBC │─>│Safety│  │
                 │  │ Est  │  │query│  │(HQP)│  │Check │  │
                 │  └──────┘  └─────┘  └─────┘  └──────┘  │
                 └───────────────────────┬────────────────┘
                                         │ HybridJointInterface
                        ┌────────────────┴───────────────┐
                        │                                │
                        ▼                                ▼
                 ┌─────────────┐                ┌──────────────┐
                 │  UnitreeHW  │                │ Gazebo Plugin│
                 │  (真机)     │                │  (仿真)      │
                 └─────────────┘                └──────────────┘
```

### 68.9.5 什么时候用 OCS2 示例、什么时候用 legged_control

| 场景 | 推荐 | 理由 |
|------|------|------|
| 学习 OCS2 MPC 算法 | OCS2 示例 | 没有 WBC/估计的干扰,更容易理解 MPC 行为 |
| 修改 MPC 算法本身 | OCS2 示例 → 验证 → 移植到 legged_control | 先在简单环境中验证,再集成 |
| 真机实验 | legged_control | 必须有 WBC + 估计 + 硬件接口 |
| 添加新约束到 OCP | 两者都行 | OCS2 示例验证数学,legged_control 验证工程 |
| 做研究(发论文) | legged_control | 需要完整系统才能做有意义的实验 |

> 🧠 **思维陷阱**：不要认为 OCS2 示例"没用了"
>
> **新手想法**："既然 legged_control 更完整，OCS2 示例就没价值了吧？"
>
> **实际上**：OCS2 示例的价值在于**算法纯净性**。如果你想理解 MPC 的求解行为（不被 WBC 和状态估计"污染"），OCS2 示例是最好的起点。如果你想改 MPC 算法本身（换求解器、改约束、加新代价），先在 OCS2 示例中验证，再移植到 legged_control,是最高效的工作流。

**练习 68.9.A** ⭐⭐⭐：在 OCS2 的 `ocs2_legged_robot` 示例中关闭 ground truth,改用 legged_control 的 `LinearKalmanFilter`。比较有无状态估计噪声时的 MPC 表现差异。

**练习 68.9.B** ⭐⭐⭐：列出 legged_control 的 `LeggedInterface::setupOptimalControlProblem()` 与 OCS2 `ocs2_legged_robot` 中对应函数的逐行差异。每个差异解释其工程动机。

---

## 68.10 扩展与移植——添加新机器人、新步态、新模块 ⭐⭐⭐

### 68.10.1 为什么要学扩展

阅读代码只是第一步。legged_control 的真正价值在于**作为你研究的起点**。无论是换一个机器人、加一种步态、还是替换某个模块，都需要理解如何扩展 legged_control。

### 68.10.2 添加新机器人（以 Go2 为例）

社区已有多个 Go2 适配项目（如 `Feng1909/legged_control_go2`、`WeixianLin-cc/leggedcontrol_go2`），以下是核心步骤：

**Step 1: 准备 URDF**

```bash
# Go2 的 URDF 来自 unitree_ros
# 需要确保关节名和链接名与 legged_control 的命名规范一致
# A1 的关节命名: LF_HAA, LF_HFE, LF_KFE, RF_HAA, ...
# Go2 的关节命名可能不同, 需要在 xacro 中重映射
```

关键的 URDF 适配步骤在于**模仿 `legged_examples/legged_unitree/legged_unitree_description`** 的结构，编写 Go2 的 xacro 文件并生成 URDF。关节名和链接名必须与 `task.info` 中的配置完全一致。

**Step 2: 修改 task.info**

```ini
; Go2 与 A1 的关键差异
model_settings
{
  jointNames
  {
    ; Go2 的关节名序列(与 URDF 一致)
    [0] FL_hip_joint    ; 对应 A1 的 LF_HAA
    [1] FL_thigh_joint  ; 对应 A1 的 LF_HFE
    [2] FL_calf_joint   ; 对应 A1 的 LF_KFE
    ; ... 12 个关节
  }

  contactNames3DoF
  {
    [0] FL_foot  ; 足端 link 名
    [1] FR_foot
    [2] RL_foot
    [3] RR_foot
  }
}

; Go2 的几何尺寸与 A1 不同, 影响 MPC 的质心高度参考
centroidal_model_settings
{
  defaultJointState  ; Go2 的默认站立姿势
  {
    (0,0) 0.0   ; FL_hip
    (1,0) 0.8   ; FL_thigh (与 A1 不同!)
    (2,0) -1.6  ; FL_calf
    ; ...
  }
}
```

**Step 3: 升级 Unitree SDK**

```
A1 使用 unitree_legged_sdk v3.2 (LowCmd/LowState 结构)
Go2 使用 unitree_sdk2 (完全不同的 API!)

关键差异:
- 通信方式: A1 用 UDP 直连, Go2 用 CycloneDDS
- 命令结构: A1 的 LowCmd 是 C struct, Go2 的是 IDL 消息
- 电机编号: A1 和 Go2 的关节编号映射不同
- 安全机制: Go2 有额外的 SDK 级安全层
```

**Step 4: 实现 Go2HW**

> **注意**：以下为**伪代码示意**，展示 Go2 硬件接口的设计思路，不可直接编译运行。实际实现需参考 unitree_sdk2 官方示例。

```cpp
class Go2HW : public LeggedHW {
  // 不再使用 UNITREE_LEGGED_SDK::UDP
  // 改用 unitree_sdk2 的 DDS 通信
  unitree::robot::go2::LowLevel lowLevel_;

  void read(...) override {
    auto state = lowLevel_.getState();
    // 解析 state 到 jointData_ ...
    // 注意: Go2 的关节编号映射与 A1 不同
    static const int go2ToLegged[] = {
        3, 4, 5,   // FL: hip, thigh, calf
        0, 1, 2,   // FR
        9, 10, 11, // RL
        6, 7, 8    // RR
    };
  }
};
```

**Step 5: 调参**

| 参数 | A1 值 | Go2 建议初值 | 调整理由 |
|------|-------|-------------|---------|
| swingKp | 350 | 300 | Go2 腿更长,同样增益 → 过冲更大 |
| swingKd | 8 | 10 | 更长的腿 → 更大的惯量 → 需要更多阻尼 |
| frictionCoeff | 0.5 | 0.5 | 取决于脚底材料,Go2 橡胶脚底摩擦系数相当 |
| baseHeightTarget | 0.30 | 0.33 | Go2 腿更长,站更高 |

> ⚠️ **编程陷阱**：Go2 SDK 与 OCS2 的依赖冲突
>
> **现象**：编译时 CycloneDDS 和 OCS2 的 Boost 版本冲突
>
> **根本原因**：Go2 的 unitree_sdk2 依赖 CycloneDDS,而 CycloneDDS 可能与 ROS1 的 Boost 版本不兼容
>
> **正确做法**：使用 Docker 隔离编译环境,或升级到 ROS2 版本的 legged_control（社区有非官方的 ROS2 移植）

> **ROS2 迁移清单**（超出 DDS/Boost 冲突）：
> - `ros_control` → `ros2_control`：HardwareInterface 生命周期（configure/activate/deactivate）完全重写
> - OCS2 分支切换：main (ROS1) → ros2 (ROS2)，API 变动较多
> - 硬件接口：`SystemInterface` 的 `on_init/read/write` 签名不同
> - DDS 初始化：unitree_sdk2 的 DDS 与 ROS2 的 rmw 可能冲突，需隔离或统一 DDS 实现

### 68.10.3 添加新步态

**场景**：你想加一个 "pronk"（四腿同时跳）步态。

**Step 1: 在 gait.info 中定义**

```ini
pronk
{
  cycleDuration      0.4   ; 0.4 秒一跳
  switchingTimes     [0.0, 0.2]  ; 前半周期 stance, 后半周期 swing
  modeSequence       [15, 0]     ; 15=全着地, 0=全腾空
}
```

**Step 2: 验证 MPC 能处理腾空相**

Pronk 有一个完全腾空的阶段（mode=0,没有任何接触力）。这意味着：

```
pronk 步态的挑战:
1. 腾空时无接触力 → MPC 的输入维度从 24 降为 12(只有关节速度)
2. 着地瞬间冲击力 → 需要摩擦锥约束足够宽松
3. 高度控制困难 → MPC 预测步数要覆盖整个跳跃周期
4. 重力加速度不可忽略 → 0.2 秒腾空, 下降距离 ~0.5*9.8*0.2^2 ≈ 0.2m
```

**Step 3: 调整 WBC 参数**

在腾空阶段,WBC 的 Level 0 约束发生变化——没有接触力决策变量,动力学方程退化为"自由下落"。需要确保 WBC 不会在这个阶段输出错误的关节扭矩（应该只输出关节位置保持命令,把腿收到着地姿势）。

### 68.10.4 替换状态估计器

**场景**：你想把 `LinearKalmanFilter` 替换为 Ch57 学过的 InEKF。

legged_control 的模块化设计使替换很直接——只需继承 `StateEstimateBase` 并实现 `update()` 方法：

```cpp
// 1. 创建新类,继承 StateEstimateBase
class InEKFEstimate : public StateEstimateBase {
 public:
  InEKFEstimate(const PinocchioInterface& pinocchioInterface,
                 const CentroidalModelInfo& info,
                 const PinocchioEndEffectorKinematics& eeKinematics);

  vector_t update(const ros::Time& time,
                   const ros::Duration& period) override {
    // Step 1: IMU 前向传播(用 InEKF 的 Lie 群形式)
    // X_pred = X * exp(u * dt)  (SE_2(3) 上的指数映射)

    // Step 2: 腿部运动学观测更新
    // 对每个 stance 脚:
    //   y = base_R^T * (foot_pos - base_pos)
    //   K = P * H^T * (H*P*H^T + R)^{-1}
    //   X = K * (y - y_pred) * X  (右不变更新)

    // Step 3: 返回全身状态
    return rbdState;
  }
};

// 2. 在 LeggedController::init() 中切换
std::string estimatorType;
nh.getParam("estimator_type", estimatorType);
if (estimatorType == "kalman") {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(...);
} else if (estimatorType == "inekf") {
  stateEstimate_ = std::make_shared<InEKFEstimate>(...);
}
```

**InEKF vs LinearKF 的预期差异**：

| 指标 | LinearKF | InEKF |
|------|----------|-------|
| 位置误差(平坦地面) | ~2 cm | ~1 cm |
| 位置误差(粗糙地面) | ~5 cm | ~2 cm |
| yaw 漂移(1 min) | ~3 度 | ~1 度 |
| 计算时间 | ~0.02 ms | ~0.05 ms |
| 代码复杂度 | ~300 行 | ~600 行 |

> 💡 **设计权衡**：legged_control 选择 LinearKF 而非 InEKF 的原因是**教学优先**——300 行代码更容易读懂和修改。对于在平坦地面上的 trot 行走,LinearKF 的精度已经足够。当你需要在复杂地形上运行时,切换到 InEKF 是值得的投入。

### 68.10.5 常见移植陷阱清单

| 陷阱 | 现象 | 根本原因 | 解决方案 |
|------|------|---------|---------|
| 关节顺序错误 | 机器人腿交叉或反向运动 | URDF 中的关节顺序与 SDK 不一致 | 打印关节名和位置,逐一对应 |
| 坐标系不一致 | 机器人向后走 | 机器人 base_link 的 x 轴方向与 legged_control 假设不同 | 在 URDF 中旋转 base_link |
| 惯性参数错误 | MPC 计算的力与实际差距大 | URDF 的惯性张量不准确 | 用 CAD 重新计算,或用系统辨识 |
| PD 增益过大 | 关节剧烈抖动 | 不同电机的带宽不同 | 从小增益开始,逐步增大 |
| 通信超时 | 机器人偶尔"卡顿" | UDP/CAN 丢包 | 加超时检测和上一次命令保持 |
| 重力方向错误 | 机器人无法站立 | URDF 的 world 坐标系 z 轴不朝上 | 检查 URDF 的 world link 定义 |

**练习 68.10.A** ⭐⭐⭐：完成 Go2 的 URDF 适配（可以从 `unitree_ros` 获取 Go2 URDF）。修改 `task.info` 中的关节名和默认姿态。在 RViz 中验证模型加载正确。

**练习 68.10.B** ⭐⭐⭐：实现一个 `InEKFEstimate` 类的框架（不需要完整的 InEKF 数学,但要有正确的接口和数据流）。能编译通过,能在 LeggedController 中切换使用。

---

## 常见故障与排查

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| 控制器切换步态时机器人剧烈抖动或摔倒 | 步态切换发生在摆动相中期,某条腿正在空中却被要求 stance;或 MPC 的 ModeSchedule 出现不连续跳变 | 1. 检查步态切换是否等到当前周期的全着地时刻 2. 用 `rqt_plot` 绘制 `plannedMode` 的时间序列,确认无突变 3. 在 `gait.info` 中加入过渡步态（如 stance → trot 中间插入 0.5 秒的慢速 trot） | 68.6 |
| WBC 的 QP 报 infeasible（无解）,关节扭矩输出为零或 NaN | 摩擦锥约束与运动追踪约束在当前状态下互相矛盾（如 MPC 要求的接触力超出了摩擦锥范围）,或 Pinocchio 的质量矩阵 $M$ 因数值问题变得奇异 | 1. 打印 QP 的约束矩阵条件数 2. 检查 MPC 输出的接触力是否在摩擦锥内 3. 临时放松摩擦系数（增大 $\mu$）看是否恢复可行 4. 检查 Pinocchio 的 URDF 惯性参数是否合理 | 68.5 |
| MPC 预测轨迹持续漂移——机器人慢慢偏离期望位置 | 状态估计的里程计累积漂移;或 MPC 的参考轨迹生成器没有用闭环反馈（开环积分命令速度） | 1. 用外部 mocap 对比状态估计的位置漂移量 2. 检查 `currentObservation_.state` 中基座位置是否逐步偏离真实值 3. 在参考轨迹中加入位置反馈项（而非纯速度积分） | 68.4, 68.9 |
| 仿真中工作正常但真机上关节"反向运动" | URDF 中的关节轴方向与 Unitree SDK 的正方向约定不一致,或关节顺序映射表错误 | 1. 逐个关节发送小幅正位置命令,观察实际运动方向 2. 对照 URDF 的 `<axis>` 标签和 SDK 的关节定义 3. 修改 `joint_reorder` 映射表 | 68.7, 68.10 |

---

> **本质洞察**：精读 legged_control 之后最重要的认知是——控制系统的**稳定性不取决于单个模块的正确性,而取决于模块间接口的一致性**。MPC 可以用完美的算法求解,WBC 可以输出精确的扭矩,状态估计可以给出准确的姿态——但如果 MPC 的坐标系约定与 WBC 的不一致,或者状态估计的更新时序与控制循环不同步,系统照样会崩溃。legged_control 中超过 70% 的 bug（根据 GitHub issues 统计）都是接口层面的问题——坐标系错误、关节顺序错位、四元数约定不同。这也是为什么系统集成能力（而非算法能力）才是从仿真走向真机的核心瓶颈。

## 68.11 本章小结 + 累积项目 + 延伸阅读

### 68.11.1 知识点总结

| 小节 | 核心知识点 | 难度 | 一句话总结 |
|------|-----------|------|-----------|
| 68.1 | legged_control 定位 | ⭐ | OCS2 到真机的桥梁,填补 WBC+估计+硬件的空白 |
| 68.2 | 系统架构 | ⭐⭐ | 双线程(MPC异步+Controller同步),数据流从传感器到执行器 |
| 68.3 | LeggedController | ⭐⭐ | update() 六阶段:读 → 估计 → MPC查询 → WBC → 安全 → 写 |
| 68.4 | MPC 集成 | ⭐⭐ | 同进程 Triple Buffer 通信,LeggedInterface 定义 OCP |
| 68.5 | WBC 集成 | ⭐⭐ | 三层 HQP:物理约束 → 运动追踪 → 力分配 |
| 68.6 | 步态管理 | ⭐⭐ | ModeSchedule 编码接触序列,GaitSchedule 生成步态 |
| 68.7 | 硬件接口 | ⭐⭐ | HybridJointInterface 5参数,UnitreeHW 封装 SDK |
| 68.8 | 可视化调试 | ⭐⭐ | RViz 接触力箭头 + rqt_plot 信号监控 + 系统排障 |
| 68.9 | OCS2 对比 | ⭐⭐⭐ | legged_control 增加了 WBC+估计+硬件+安全 四大模块 |
| 68.10 | 扩展移植 | ⭐⭐⭐ | 新机器人(Go2)、新步态(pronk)、新估计器(InEKF) |

### 68.11.2 累积项目：本章新增模块

本章是足式方向累积项目"从零构建四足控制器"的第五站。在前面章节你已经有了：

```
Ch47: 加载 URDF + Pinocchio 接口
Ch49: 正/逆运动学 + Centroidal 模型
Ch53: WBC QP 公式化 + qpOASES 求解
Ch55: OCS2 MPC 集成 + 双线程架构
────────────────────────────────
Ch68 (本章新增):
  ├─ 将 MPC + WBC 集成为一个 ros_control Controller
  ├─ 添加 LinearKalmanFilter 状态估计
  ├─ 实现 HybridJointInterface 混合控制接口
  ├─ Gazebo 仿真验证: A1 trot 行走
  └─ (可选) Go2 适配 / InEKF 替换
```

**里程碑检查**：

- [ ] 能在 Gazebo 中编译运行 legged_control，让 A1 站立
- [ ] 能发送 trot 步态命令，A1 在 Gazebo 中行走
- [ ] 能修改 gait.info 中的步态参数，观察行为变化
- [ ] 能在 RViz 中看到接触力箭头和 MPC 轨迹
- [ ] 能说清楚 legged_control 每个包的职责
- [ ] 能画出 update() 的完整数据流图

### 68.11.3 延伸阅读

**核心参考**：

| 资源 | 类型 | 难度 | 推荐理由 |
|------|------|------|---------|
| [legged_control 源码](https://github.com/qiayuanl/legged_control) | 代码 | ⭐⭐ | 本章精读对象，需要逐文件阅读 |
| [OCS2 官方文档](https://leggedrobotics.github.io/ocs2/) | 文档 | ⭐⭐⭐ | 理解 legged_control 依赖的框架 |
| [BIT-XLJ/legged_control_explanation](https://github.com/BIT-XLJ/legged_control_explanation) | 注释代码 | ⭐⭐ | 中文注释版,适合初读 |

**论文**：

| 论文 | 年份/会议 | 难度 | 内容 |
|------|----------|------|------|
| Farshidian et al., "An efficient optimal planning and control framework for quadrupedal locomotion" | ICRA 2017 | ⭐⭐⭐ | OCS2 的前身论文 |
| Di Carlo et al., "Dynamic locomotion in the MIT Cheetah 3 through convex model-predictive control" | IROS 2018 | ⭐⭐ | 另一种 MPC 方案(凸 MPC),与 OCS2 对比 |
| Liao et al., "Walking in Narrow Spaces: Safety-critical Locomotion Control" | IROS 2023 | ⭐⭐⭐ | legged_control 作者的研究,NMPC+DCBF |
| Grandia et al., "Perceptive Locomotion Through Nonlinear Model-Predictive Control" | T-RO 2023 | ⭐⭐⭐⭐ | OCS2 + 感知 MPC,legged_control 的研究前沿 |
| Kim et al., "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control" | IROS 2019 | ⭐⭐⭐ | MIT Mini Cheetah 的 MPC+WBC,经典对照 |

**社区资源**：

| 资源 | 类型 | 难度 | 内容 |
|------|------|------|------|
| [Feng1909/legged_control_go2](https://github.com/Feng1909/legged_control_go2) | 代码 | ⭐⭐ | Go2 适配参考 |
| [MasterYip/hexapod_control](https://github.com/MasterYip/hexapod_control) | 代码 | ⭐⭐⭐ | 六足适配,看如何扩展到非四足 |
| [legged_perceptive](https://github.com/qiayuanl/legged_perceptive) | 代码 | ⭐⭐⭐⭐ | 作者的感知扩展,Ch67 的代码参考 |

---

### 与其他章节衔接

**向前承接**：

| 前置章节 | 在本章中的体现 |
|---------|--------------|
| Ch47-49 Pinocchio + Centroidal | WBC 的动力学计算、MPC 的模型基础 |
| Ch53 WBC/TSID | HierarchicalWbc 的数学基础,本章不重复推导 |
| Ch55 OCS2 | MPC 集成的框架基础,Triple Buffer 通信 |
| Ch57 状态估计 | LinearKalmanFilter 的理论基础 |
| Ch61-62 实时C++ + ros_control | Controller 插件、HardwareInterface |

**向后指向**：

| 后续章节 | 本章提供的基础 |
|---------|--------------|
| Ch69 Mini-Legged 综合项目 | legged_control 作为"参考实现",你的项目对标它 |
| Ch70 研究方向综述 | legged_control 是大部分研究方向的实验平台 |
| Ch67 Perceptive MPC(回溯) | legged_perceptive 基于 legged_control 构建 |

---

> 走完 legged_control 的完整精读，你已经从"理解理论"跨越到"理解系统"。控制理论(MPC、WBC)只是大厦的几块砖,而 legged_control 展示了如何把砖砌成墙、把墙围成屋。下一章 Ch69 是你的"毕业设计"——**从零搭建一个简化四足控制器**。legged_control 是你的参考答案,但你需要独立走一遍从无到有的过程。
