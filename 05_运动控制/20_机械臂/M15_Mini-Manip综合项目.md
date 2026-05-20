> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M15 Mini-Manip 综合项目

## 前置自测

📋 **前置自测**（答不出 ≥ 3 题 → 先回对应章节复习，本章是全栈综合）

1. URDF 的 `<ros2_control>` 标签如何声明硬件接口？（P01, M12）
2. MoveIt2 的 PlanningScene 如何管理 attached objects？（M14）
3. BT.CPP 的 Fallback 节点如何实现错误恢复？（M13）
4. ros2_control 的 JointTrajectoryController 如何执行 MoveIt2 的规划轨迹？（M12）
5. MTC 的 Stage 如何通过 InterfaceState 传播约束？（M14）
6. OMPL 规划器中 RRT-Connect 与 BIT* 的适用场景分别是什么？（M07）
7. ForwardCommandController 与 JTC 的区别是什么？为什么 RL 部署用前者？（M12）

## 本章目标

学完本章后，你能够：
1. **端到端贯通** M01-M14 的全部技能，搭建完整的 pick-and-place 系统
2. 在 Gazebo Harmonic 中完成**完整的 bin-picking 流程**：检测 → 规划 → 抓取 → 搬运 → 放置 → 返回
3. 用 BT.CPP **编排错误恢复**：规划失败重试、抓取失败退回、物体掉落重新检测
4. 体验 **sim-to-real swap**：同一份代码从 Gazebo 切换到 mock hardware 只改一行参数
5. 对 IK/规划器/时间参数化做**量化对比**，产出选型报告
6. 产出一个**可展示的 GitHub 仓库**——作为求职和项目展示的作品

---

## M01-M14 知识桥接回顾

在进入综合项目之前，我们先回顾本项目所依赖的所有前置模块及其核心贡献。这不是简单的"见 M01-M14"，而是从系统集成的视角重新审视每个模块在完整 pick-and-place 栈中扮演的角色。

| 模块 | 核心能力 | 在 Mini-Manip 中的角色 | 关键接口 |
|------|---------|----------------------|---------|
| **M01 Pinocchio** | FK/IK/Jacobian 计算 | MoveIt2 IK 插件的底层引擎 | `pinocchio::forwardKinematics()` |
| **M02 动力学库** | 动力学模型对比 | 理解 Pinocchio 为何被 MoveIt2 选中 | RNEA、CRBA |
| **M03 IK 求解器** | 多种 IK 解法及性能光谱 | MTC 的 ComputeIK Stage 背后的 IK 插件选型 | KDL/TRAC-IK/pick-ik |
| **M04 碰撞检测** | FCL/Bullet 碰撞引擎 | PlanningScene 中障碍物检测的底层实现 | `FCL::collide()` |
| **M05 QP/NLP** | 优化问题建模 | 理解 CHOMP/STOMP 等优化规划器的数学基础 | ProxQP/OSQP |
| **M07 OMPL** | 采样规划算法 | MoveIt2 默认规划器后端 | RRT-Connect/BIT* |
| **M08 轨迹优化** | 轨迹平滑与优化 | 链式规划管线中的后处理 | STOMP/TrajOpt |
| **M10 时间参数化** | 路径→轨迹转换 | MoveIt2 规划管线的最后一步 | TOTG/Ruckig |
| **M11 实时 C++** | RT 安全编程 | ros2_control 控制循环的安全保证 | SCHED_FIFO + mlockall |
| **M12 ros2_control** | 控制器-硬件解耦 | JTC 执行 MoveIt2 规划的轨迹 | JTC/Forward/Admittance |
| **M13 BehaviorTree** | 任务编排与错误恢复 | 顶层 pick-and-place 流程控制 | BT.CPP + Groot2 |
| **M14 MoveIt2/MTC** | 规划框架与多阶段任务 | 核心规划能力 | MoveGroupInterface/MTC Task |

> **本质洞察**：Mini-Manip 项目的真正价值不是"用 MoveIt2 做 pick-and-place"——这一点几行 Python 就能演示。它的价值在于让你**亲手体验 14 个模块如何通过清晰的接口边界组装成一个完整系统**。每个模块的设计决策（如 M12 的 command interface 独占机制、M14 的 pluginlib 插件化、M13 的 Blackboard 数据隔离）在单独学习时可能显得"过度设计"，但在系统集成时你会发现：正是这些抽象层让"换一个 IK 求解器"变成了改一行 YAML，而不是改一千行代码。

---

## M15.1 系统架构设计 ⭐⭐

### 动机 ⭐⭐

从 M01 到 M14，我们分别学习了运动学、动力学、碰撞检测、规划、控制、ros2_control、行为树、MoveIt2/MTC 等模块。但在真实系统中，这些模块不是孤立的——它们需要**端到端集成**，数据流必须在各层之间无缝传递。

本章的目标不是学新知识——而是把已有知识**组装**成一个完整的工作系统，并在这个过程中深化对各模块交互方式的理解。

> **跨领域类比**：这就像从学习单个乐器（运动学、规划、控制...）到组建乐队演奏完整曲目——每个乐手（模块）都需要听其他乐手（通过接口通信），指挥（BT 编排）协调所有人的节奏。乐队的水平不取决于最好的乐手，而取决于最差的配合。

### 四层架构 ⭐⭐

```
┌─────────────────────────────────────────────────────────┐
│ Layer 1: 任务编排 (BT.CPP, 10 Hz tick)                   │
│                                                          │
│  Sequence:                                               │
│   ├─ ScanTable (Condition: 物体检测)                     │
│   ├─ PlanPick  (Action: 调用 MTC)                       │
│   ├─ ExecutePick (Action: 执行轨迹)                     │
│   ├─ PlanPlace (Action: 调用 MTC)                       │
│   └─ ExecutePlace (Action: 执行轨迹)                    │
│  Fallback (错误恢复):                                    │
│   ├─ RetryPlan (重新规划, 最多 3 次)                    │
│   └─ ReturnHome (回到安全位姿)                          │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│ Layer 2: 运动规划 (MoveIt2 + MTC)                        │
│                                                          │
│  ├─ OMPL (RRT-Connect, M07) → 路径                      │
│  ├─ STOMP (可选链式平滑, M08)                           │
│  ├─ GPU 规划器/加速后端 (可选, M09)                     │
│  ├─ Ruckig → 时间参数化 (M10)                           │
│  ├─ FCL → 碰撞检测 (M04)                                │
│  └─ TRAC-IK / pick-ik → 运动学 (M03)                   │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│ Layer 3: 执行控制 (ros2_control, 500 Hz)                 │
│                                                          │
│  ├─ JointTrajectoryController                           │
│  ├─ JointStateBroadcaster                               │
│  └─ GripperActionController                              │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│ Layer 4: 硬件/仿真 (Xacro sim-to-real swap)              │
│                                                          │
│  ├─ Gazebo Harmonic: gz_ros2_control                    │
│  ├─ Mock: mock_components/GenericSystem                  │
│  └─ 真机 (预留): franka_hardware                        │
└─────────────────────────────────────────────────────────┘
```

### 数据流详解 ⭐⭐

```
感知模块 ──物体位姿──► BT 条件节点
                         │
                    BT Action 节点
                         │
                    MTC Task ──InterfaceState──► 多阶段规划
                         │
                    JointTrajectory
                         │
              FollowJointTrajectory Action
                         │
                    JTC update() ──cmd──► Hardware write()
                         │
                    /joint_states ──► RViz / 感知反馈
```

**各层之间的接口**：

| 层间接口 | 通信方式 | 消息类型 | 延迟 |
|---------|---------|---------|------|
| BT → MTC | C++ 函数调用 | mtc::Task API | <1ms |
| MTC → JTC | ROS2 Action | FollowJointTrajectory | 1-5ms |
| JTC → Hardware | 进程内 (ros2_control) | command interface | <1us |
| Hardware → RViz | ROS2 Topic | /joint_states | 1-5ms |
| 感知 → BT | ROS2 Topic/Service | PoseStamped | 5-50ms |

> **本质洞察**：系统架构的关键不是每个组件多么复杂，而是**组件之间的接口是否清晰**。每一层只知道上下层的接口，不知道对方的内部实现——这就是 M12 中「ros2_control 解耦控制器与硬件」和 M14 中「MoveIt2 pluginlib 解耦规划器与框架」在系统层面的体现。

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为"先把所有模块写完再集成"
   新手想法："先写完感知模块，再写完规划模块，最后集成"
   实际上：先用最简化的 mock 版本跑通端到端流程，然后逐步替换：
          1. 第一天：硬编码物体位姿 + MTC 固定规划 + Gazebo 执行
          2. 第二天：换成真实感知
          3. 第三天：加入 BT 错误恢复
          4. 第四天：加入 IK/规划器对比
          这种「vertical slice first」的开发方式能更早发现集成问题。
```

```
💡 概念误区：认为"模块测试通过了就不会有集成问题"
   新手想法："每个模块单独测试都通过了，集成应该没问题"
   实际上：集成问题往往出在接口边界：
          - 坐标系不一致（MTC 用 world frame，感知用 camera frame）
          - 时间戳不同步（感知数据滞后于机器人状态）
          - 消息格式不匹配（PoseStamped vs Pose）
          - 启动顺序依赖（move_group 必须先于 BT 节点启动）
```

### 练习

1. **[A 型]** 画出完整系统的进程拓扑图：哪些组件在同一进程中？哪些通过 ROS2 通信？标注每条通信链路的延迟。

2. **[思考题]** 如果将系统从 Franka Panda 移植到 UR5e，需要修改哪些文件？哪些代码完全不改？

---

## M15.2 分阶段开发计划 ⭐⭐

### 第一阶段：环境搭建（2 天） ⭐⭐

**目标**：Gazebo 中启动 Franka Panda，验证 ros2_control + RViz 基本功能。

**核心：sim-to-real swap 的 Xacro 实现**

```xml
<!-- panda.ros2_control.xacro -->
<xacro:macro name="panda_ros2_control"
             params="hardware_plugin:=mock robot_ip:=172.16.0.2">
  <xacro:if value="${hardware_plugin == 'gazebo'}">
    <gazebo>
      <plugin filename="gz_ros2_control-system"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin"/>
    </gazebo>
  </xacro:if>

  <ros2_control name="PandaSystem" type="system">
    <hardware>
      <!-- hardware_plugin 必须三选一：gazebo / mock / franka。
           不要同时用 use_sim/use_mock 两个 bool，否则可能生成两个 <plugin>。 -->
      <xacro:if value="${hardware_plugin == 'gazebo'}">
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </xacro:if>
      <xacro:if value="${hardware_plugin == 'mock'}">
        <plugin>mock_components/GenericSystem</plugin>
        <param name="mock_sensor_commands">true</param>
      </xacro:if>
      <xacro:if value="${hardware_plugin == 'franka'}">
        <plugin>franka_hardware/FrankaHardwareInterface</plugin>
        <param name="robot_ip">${robot_ip}</param>
      </xacro:if>
    </hardware>
    <joint name="panda_joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <!-- ... joint2 到 joint7 ... -->
  </ros2_control>
</xacro:macro>
```

Gazebo 分支有两层含义：上面的 `<gazebo>` system plugin 把 Gazebo 和 controller manager 接起来；`<ros2_control>` 里的 `gz_ros2_control/GazeboSimSystem` 则是 ros2_control 看到的 hardware plugin。缺少任意一层，仿真中的控制链路都不完整。

> **跨领域类比**：Xacro 的条件编译类似于 C++ 的 `#ifdef`——同一份代码根据参数生成不同输出。sim-to-real swap 的核心是**不是三份代码，而是一份代码三种配置**。

**验收标准**：
```bash
# Gazebo 启动
ros2 launch mini_manip_bringup gazebo.launch.py
# 验证
ros2 topic echo /joint_states  # 有数据
ros2 control list_controllers   # JTC + Broadcaster active
# RViz 中可交互式规划并执行
```

### 第二阶段：MTC Pick-and-Place（3 天） ⭐⭐

**目标**：用 MTC 实现完整的抓取-放置流程。

回顾 M14.4：MTC 的 Stage 体系通过 InterfaceState 自动传播约束。本阶段要把 M14 中学的 MTC API 应用到实际系统中。

**关键步骤**：

1. 添加碰撞对象（桌面 + 目标物体）
2. 定义 MTC Task（8+ 个 Stage）
3. 在 RViz 中可视化 MTC 方案
4. 在 Gazebo 中执行

**MTC Task 结构**：
```
CurrentState → OpenGripper → Connect(to pre-grasp)
→ ComputeIK(GenerateGraspPose)
  → Approach → CloseGripper → Attach → Lift
→ Connect(to place) → PlaceObject → OpenGripper → Detach → Retreat
```

**常见调试问题**：

| 问题 | 症状 | 解决 |
|------|------|------|
| approach 碰撞 | MTC 报 0 解 | 调整 pre-grasp 偏移量 |
| IK 无解 | ComputeIK 0 解 | 增大 angle_delta、max_ik_solutions |
| 抓取滑落 | Gazebo 中物体掉落 | 增大摩擦系数或用 attach plugin |
| 放置偏差 | 物体位置不准 | 检查 attach 时的相对位姿 |

### 第三阶段：BT.CPP 编排（2 天） ⭐⭐

**目标**：用 BT.CPP 包装 MTC 调用，加入错误恢复。

**BT XML 设计**：

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MiniManip">
    <Sequence>
      <!-- 检测 (带重试) -->
      <RetryUntilSuccessful num_attempts="3">
        <DetectObject object_name="target_cube"
                      object_pose="{cube_pose}"/>
      </RetryUntilSuccessful>

      <!-- 抓取 (带 Fallback 恢复) -->
      <Fallback>
        <PlanAndExecutePick object_pose="{cube_pose}"/>
        <Sequence>
          <OpenGripper/>
          <MoveToHome/>
          <DetectObject object_name="target_cube"
                        object_pose="{cube_pose}"/>
          <PlanAndExecutePick object_pose="{cube_pose}"/>
        </Sequence>
      </Fallback>

      <!-- 放置 -->
      <PlanAndExecutePlace place_pose="{place_target}"/>

      <!-- 返回 -->
      <MoveToHome/>
    </Sequence>
  </BehaviorTree>
</root>
```

**BT Action 节点实现**：

```cpp
// 推荐做法：把 MTC plan/execute 放在 ROS2 Action Server 内部，
// BT 节点只负责发送 goal、接收 feedback/result、在 halt 时取消 goal。
// BehaviorTree.ROS2 的 RosActionNode 基类会在 halt/destruct 路径处理取消；
// Action Server 端必须在长耗时 plan/execute 循环中检查 is_canceling()。
#include "behaviortree_ros2/bt_action_node.hpp"
#include "mini_manip_msgs/action/pick.hpp"

class PlanAndExecutePick
  : public BT::RosActionNode<mini_manip_msgs::action::Pick>
{
public:
  using ActionT = mini_manip_msgs::action::Pick;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = ActionT::Goal;
  using WrappedResult =
    rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

  PlanAndExecutePick(const std::string& name,
                     const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
    : Base(name, config, params)
  {}

  static BT::PortsList providedPorts() {
    return Base::providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "object_pose", "目标物体位姿")
    });
  }

  bool setGoal(Goal& goal) override {
    auto pose = getInput<geometry_msgs::msg::PoseStamped>(
      "object_pose");
    if (!pose) return false;
    goal.object_pose = pose.value();
    return true;
  }

  BT::NodeStatus onResultReceived(
    const WrappedResult& result) override
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
      return BT::NodeStatus::FAILURE;
    return result.result->success
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
  }
};
```

如果项目暂时不拆 ROS2 Action Server，而是在 BT 节点内部启动 worker 线程，也不要用裸 `std::async([this])`：`std::future` 没有标准取消语义，`onHalted()` 只调用 `cancel_execution()` 会和析构、下一次 tick 产生竞态。完整 worker 模式至少要包含 `std::jthread`/`stop_token` 或原子取消标志、MTC 执行取消、互斥保护共享状态，并在 `onHalted()` 和析构函数中请求停止后 `join()`，保证 worker 不再访问已析构的 `this`。

**集成 Groot2 日志**：

```cpp
auto tree = factory.createTreeFromFile("mini_manip.xml");
BT::SqliteLogger sql_log(tree, "log.db3");
BT::Groot2Publisher groot_pub(tree);

while (tree.tickOnce() == BT::NodeStatus::RUNNING) {
  std::this_thread::sleep_for(100ms);
}
```

### 第四阶段：优化与对比（2 天） ⭐⭐⭐

**目标**：量化比较不同组件配置的影响。

**IK 求解器对比**：

对 1000 个随机目标位姿分别测试：

| IK 求解器 | 成功率 | 平均耗时 |
|-----------|--------|---------|
| KDL | ~60-70% | ~2ms |
| TRAC-IK | ~90-95% | ~0.5ms |
| pick-ik | ~85-90% | 配置超时常设为 50ms；实际耗时取决于 local/global 模式和目标难度，需在本机实测 |

**规划器对比**：

| 规划器 | 规划时间 | 路径长度 | 加速度 RMS |
|--------|---------|---------|-----------|
| RRT-Connect | ~100ms | 中等 | 高 |
| BIT* | ~500ms | 短 | 中 |
| OMPL → STOMP | ~300ms | 中等 | 低 |

**端到端性能**：

| 指标 | 测量方法 | 目标值 |
|------|---------|-------|
| 端到端周期 | BT 时间戳差 | <15s |
| 规划时间占比 | MTC plan() 耗时 / 总时间 | <30% |
| 成功率 | 100 次中成功次数 | >95% |
| 错误恢复率 | 注入故障后恢复率 | >80% |

### 第五阶段：sim-to-real 验证（1 天） ⭐⭐⭐

**目标**：验证同一份代码在不同硬件后端运行。

```bash
# Mock Hardware (RViz 纯可视化)
ros2 launch mini_manip_bringup bringup.launch.py \
  hardware_plugin:=mock

# Gazebo 仿真
ros2 launch mini_manip_bringup bringup.launch.py \
  hardware_plugin:=gazebo

# 真机 (如果有)
ros2 launch mini_manip_bringup bringup.launch.py \
  hardware_plugin:=franka robot_ip:=172.16.0.2
```

**Sim-to-Real 部署清单**：

**必须修改**：
- [ ] launch 参数: `hardware_plugin:=franka`
- [ ] robot_ip: 改为真机 IP
- [ ] 夹爪力参数: 仿真和真机力单位可能不同

**可能需要调整**：
- [ ] 轨迹速度缩放: 真机初次运行建议设 0.1
- [ ] PlanningScene 障碍物: 真实桌面尺寸和位置
- [ ] 碰撞安全间距: 真机需要更大间距

**不需要修改**：
- [ ] BT XML（任务逻辑不变）
- [ ] MTC Task 代码（规划逻辑不变）
- [ ] 控制器 YAML（JTC 配置不变）

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Gazebo 中夹爪抓取物体总是滑落
   错误做法：使用默认接触参数
   现象：夹爪闭合后物体从手中滑落
   根本原因：默认摩擦系数和接触刚度不足
   正确做法：
     1. 增大摩擦系数 (mu1, mu2)
     2. 增大接触刚度 (kp, kd)
     3. 或使用 Gazebo attach plugin 直接绑定物体
   自检方法：先用 attach plugin 验证流程，再调接触参数
```

```
🧠 思维陷阱：认为"仿真成功了真机就能成功"
   新手想法："Gazebo 里跑通了，真机应该也没问题"
   实际上：sim-to-real gap 包括：
          1. 惯性参数误差
          2. 摩擦模型差异
          3. 传感器噪声
          4. 通信延迟
          5. 电机动力学
   正确思维：仿真验证算法逻辑和系统集成的正确性，
            不是物理参数的准确性。真机需要额外的参数调整。
```

---

## M15.3 感知集成 ⭐⭐

### 动机 ⭐⭐

完整的感知系统（相机标定、物体检测、6D 位姿估计）是独立的大课题。本项目使用简化版感知。

### 三种感知模式 ⭐⭐

| 模式 | 复杂度 | 实现方式 |
|------|--------|---------|
| **硬编码** | 最低 | 代码中固定物体位姿 |
| **仿真 ground truth** | 低 | 从 Gazebo 获取模型位姿 |
| **视觉感知** | 高 | 相机 + 检测算法 |

**推荐开发顺序**：硬编码 → 仿真 ground truth → 视觉感知。

**仿真 ground truth 实现**：

```cpp
class SimObjectDetector : public rclcpp::Node {
public:
  SimObjectDetector() : Node("sim_detector") {
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/model/target_cube/pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
        detected_ = true;
      });
  }

  std::optional<geometry_msgs::msg::Pose> detect() {
    if (detected_) return current_pose_;
    return std::nullopt;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr
    pose_sub_;
  geometry_msgs::msg::Pose current_pose_;
  bool detected_{false};
};
```

### 手眼标定完整推导与代码 ⭐⭐⭐

如果使用真实相机，需要**手眼标定**（Hand-Eye Calibration）确定相机坐标系与机器人基座坐标系的关系。

**两种安装方式**：
- **Eye-in-Hand**：相机装在末端执行器上，随末端运动。标定 $T_{gripper\_camera}$
- **Eye-to-Hand**：相机固定在外部（如工作台上方）。标定 $T_{base\_camera}$

**Eye-in-Hand 标定方程推导**：

本节统一记号：$T_{a\_b}$ 表示**把 b 坐标系中的点变换到 a 坐标系**，也就是 frame b 在 frame a 中的位姿。例如 $T_{base\_gripper}$ 是 gripper→base，$T_{camera\_target}$ 是 target→camera。

考虑机器人末端从位姿 $i$ 移动到位姿 $j$。标定板固定在世界中，相机跟随末端移动。

在位姿 $i$：
$$T_{base\_gripper_i} \cdot T_{gripper\_camera} \cdot T_{camera_i\_target} = T_{base\_target}$$

在位姿 $j$：
$$T_{base\_gripper_j} \cdot T_{gripper\_camera} \cdot T_{camera_j\_target} = T_{base\_target}$$

由于标定板不动，两式右边相等。令 $A = (T_{base\_gripper_i})^{-1} \cdot T_{base\_gripper_j}$（两个末端位姿之间的相对变换，从机器人正运动学得到），$B = T_{camera_i\_target} \cdot (T_{camera_j\_target})^{-1}$（两次观测之间的相对变换，从标定板检测得到），$X = T_{gripper\_camera}$（待求），得到：

$$AX = XB$$

这就是经典的手眼标定方程。已知多组 $(A_i, B_i)$，求 $X$。

**求解方法**：

| 方法 | 论文 | 特点 | OpenCV 常量 |
|------|------|------|------------|
| Tsai-Lenz | Tsai & Lenz, 1989 | 分步求旋转再求平移 | `CALIB_HAND_EYE_TSAI` |
| Park-Martin | Park & Martin, 1994 | 李群方法，更鲁棒 | `CALIB_HAND_EYE_PARK` |
| Horaud | Horaud & Dornaika, 1995 | 四元数方法 | `CALIB_HAND_EYE_HORAUD` |
| Daniilidis | Daniilidis, 1999 | 对偶四元数，一步求解 | `CALIB_HAND_EYE_DANIILIDIS` |
| Andreff | Andreff et al., 2001 | 非线性优化 | `CALIB_HAND_EYE_ANDREFF` |

**Tsai-Lenz 方法的核心步骤**（最经典，理解其他方法的基础）：

Step 1：从 $AX = XB$ 中分离旋转部分 $R_A R_X = R_X R_B$。

Step 2：利用 Rodrigues 参数（旋转的轴角表示），令 $P_{R_A} = 2\sin(\theta_A/2)\hat{n}_A$（修正 Rodrigues 参数），将旋转方程转化为线性方程：

$$(P_{R_A} + P_{R_B}) \times P_{R_X} = P_{R_B} - P_{R_A}$$

多组数据叠加后用最小二乘求 $P_{R_X}$，还原得到 $R_X$。

Step 3：从 $R_A R_X t_X + t_A = R_X t_B + t_X$ 中，整理为：

$$(R_A - I) t_X = R_X t_B - t_A$$

多组数据叠加后用最小二乘求 $t_X$。

**完整标定代码**：

```python
import cv2
import numpy as np

def invert_transform(T):
    T_inv = np.eye(4)
    T_inv[:3, :3] = T[:3, :3].T
    T_inv[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return T_inv


def hand_eye_calibrate(T_base_gripper_list,
                       T_camera_target_list):
    """
    Eye-in-hand 手眼标定。

    记号: T_a_b 表示 b -> a。
    OpenCV calibrateHandEye 需要:
      - gripper -> base:  T_base_gripper
      - target  -> camera: T_camera_target
    并返回:
      - camera -> gripper: T_gripper_camera

    若你的机器人 API 返回 T_gripper_base，先取逆得到 T_base_gripper。
    若你的视觉/PnP API 返回 T_target_camera，先取逆得到 T_camera_target。
    """
    n = len(T_base_gripper_list)
    assert n >= 3, "至少需要 3 组数据，推荐 15-20 组"
    assert n == len(T_camera_target_list)

    # OpenCV 参数名沿用官方约定:
    # R_gripper2base/t_gripper2base == T_base_gripper
    # R_target2cam/t_target2cam       == T_camera_target
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for i in range(n):
        T_bg = T_base_gripper_list[i]
        T_ct = T_camera_target_list[i]
        R_gripper2base.append(T_bg[:3, :3])
        t_gripper2base.append(T_bg[:3, 3].reshape(3, 1))
        R_target2cam.append(T_ct[:3, :3])
        t_target2cam.append(T_ct[:3, 3].reshape(3, 1))

    # 调用 OpenCV 求解（五种方法对比）
    methods = {
        'Tsai':       cv2.CALIB_HAND_EYE_TSAI,
        'Park':       cv2.CALIB_HAND_EYE_PARK,
        'Horaud':     cv2.CALIB_HAND_EYE_HORAUD,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
        'Andreff':    cv2.CALIB_HAND_EYE_ANDREFF,
    }

    results = {}
    for name, method in methods.items():
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=method
        )
        T_gripper_camera = np.eye(4)
        T_gripper_camera[:3, :3] = R_cam2gripper
        T_gripper_camera[:3, 3] = t_cam2gripper.flatten()
        results[name] = T_gripper_camera

        # 验证旋转矩阵正交性
        det = np.linalg.det(R_cam2gripper)
        orth_err = np.linalg.norm(
            R_cam2gripper @ R_cam2gripper.T - np.eye(3))
        print(f"{name}: det(R)={det:.6f}, "
              f"orth_err={orth_err:.2e}, "
              f"t={t_cam2gripper.flatten()}")

    # 推荐：使用 Daniilidis 或 Park 方法
    return results['Park']


def validate_calibration(T_gripper_camera,
                         T_base_gripper_list,
                         T_camera_target_list):
    """验证 AX=XB 一致性。"""
    errors = []
    n = len(T_base_gripper_list)
    for i in range(n):
        for j in range(i + 1, n):
            # 从机器人运动学计算的相对变换
            A = (np.linalg.inv(T_base_gripper_list[i]) @
                 T_base_gripper_list[j])
            # 从标定结果推算的相对变换
            B = (T_camera_target_list[i] @
                 np.linalg.inv(T_camera_target_list[j]))
            # AX 应该等于 XB
            AX = A @ T_gripper_camera
            XB = T_gripper_camera @ B
            # 位移误差
            t_err = np.linalg.norm(AX[:3, 3] - XB[:3, 3])
            errors.append(t_err)

    mean_err = np.mean(errors)
    max_err = np.max(errors)
    print(f"标定验证: 平均误差={mean_err*1000:.2f}mm, "
          f"最大误差={max_err*1000:.2f}mm")
    return mean_err
```

**标定数据采集的工程要求**：

| 要求 | 说明 | 不满足的后果 |
|------|------|------------|
| 位姿数量 >= 15 | 越多越好，推荐 15-25 组 | <10 组精度显著下降 |
| 旋转覆盖 >= 60 度 | 三个轴都要有旋转变化 | 纯平移无法约束旋转 |
| 标定板充满视野 60%+ | 提高角点检测精度 | 标定板太小则检测噪声大 |
| 避免极端角度 | 标定板法线与光轴夹角 < 45 度 | 角点检测失败率增大 |
| 避免运动模糊 | 每次拍照前静止 0.5s | 运动模糊降低检测精度 |

> **反事实推理**：如果不做手眼标定直接使用相机数据会怎样？相机检测到物体在 camera frame 的位置为 (0.3, 0.1, 0.5)m，但你不知道这个坐标在 robot base frame 中是哪里——规划的抓取位姿会完全偏离实际物体位置。手眼标定的本质就是建立这两个坐标系之间的桥梁。

> **本质洞察**：手眼标定方程 $AX = XB$ 的本质是一个**齐次 Sylvester 方程**。它之所以可解，依赖于一个关键条件：多组 $(A_i, B_i)$ 对应的旋转轴不能全部平行——否则方程欠约束。这就是为什么标定数据采集必须包含多个方向的旋转。

### ⚠️ 常见陷阱

```
💡 概念误区：认为手眼标定做一次就永远不变
   新手想法："标定完就固定了"
   实际上：相机安装可能松动、温度变化导致微小偏移。
          工业系统通常定期自动重标定。
```

```
⚠️ 编程陷阱：标定数据中旋转变化不足
   错误做法：只在平面上移动机器人采集标定数据
   现象：标定结果的平移精度可以，旋转误差很大
   根本原因：纯平移运动使 AX=XB 中旋转部分欠约束
   正确做法：确保数据包含绕三个轴的显著旋转（各 >= 20 度）
   自检方法：画出所有 A_i 的旋转轴，应分散在球面上而非集中在一条线上
```

### 练习

1. **[A 型]** 实现仿真 ground truth 感知：从 Gazebo 获取物体位姿，发布到 BT 可读取的 topic。

2. **[思考题]** bin-picking 场景中相机通常装在上方（Eye-to-Hand）而非末端（Eye-in-Hand）。为什么？考虑抓取时的遮挡问题。

3. **[A 型]** 在仿真中实现手眼标定：(a) 在 Gazebo 中放置 ArUco 标定板；(b) 控制机器人到 15 个不同位姿拍照；(c) 用上述代码标定；(d) 验证标定精度。

---

## M15.4 抓取规划基础 ⭐⭐⭐

### 动机 ⭐⭐⭐

知道物体在哪里（感知），接下来要决定**从哪个方向、以什么姿态抓取**。

### 简单立方体的抓取策略 ⭐⭐

对于 5cm 立方体，从顶部向下抓取：

```
抓取方向: world Z 轴的负方向 (从上向下)
预抓取偏移: 物体上方 10-15cm
approach: 沿 Z 轴向下 5-10cm
lift: 沿 Z 轴向上 10-20cm
```

MTC 的 `GenerateGraspPose` 自动在物体周围采样多个抓取角度（通过 `setAngleDelta`），`ComputeIK` 为每个角度计算 IK 解，选择最优方案。

### 力闭合与形封闭 ⭐⭐⭐

抓取规划的理论基础：

- **力闭合**（Force Closure）：抓取力能抵抗任意方向的外力和力矩（通过摩擦）
- **形封闭**（Form Closure）：几何约束能阻止物体任何方向运动（不依赖摩擦）

对于平行夹爪抓取立方体，当摩擦系数 mu > 0.5 时，从两侧夹持即可实现力闭合。

> **反事实推理**：如果不考虑力闭合直接抓取会怎样？(1) 搬运时物体旋转滑落——抓取力矩不够；(2) 加速时物体滑出——摩擦力不够。仿真中可能不明显（完美摩擦），真机频繁发生。

### GraspNet / AnyGrasp 集成详解 ⭐⭐⭐

对于复杂形状物体（非规则几何体），手工设计抓取策略不可行。GraspNet（Fang et al., CVPR 2020）和 AnyGrasp（Fang et al., T-RO 2023）等方法从点云直接预测 6-DOF 抓取位姿和质量分数。

**GraspNet 推理 Pipeline**：

```
点云 → GraspNet → 多个抓取候选 (位姿 + 质量分数)
                     │
                     ├── 排序: 按质量分数降序
                     ├── 碰撞过滤: PlanningScene 检查
                     └── 送入 MTC: 替代 GenerateGraspPose
```

**GraspNet ROS2 集成代码**：

```python
# grasp_detector_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np
import open3d as o3d

# GraspNet 推理（简化接口）
from graspnetAPI import GraspGroup
from gsnet import AnyGrasp  # AnyGrasp SDK

class GraspDetector(Node):
    def __init__(self):
        super().__init__('grasp_detector')

        # AnyGrasp 模型加载
        self.anygrasp = AnyGrasp(cfgs)
        self.anygrasp.load_net()

        # 订阅点云
        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', 
            self.pc_callback, 10)

        # 发布抓取候选
        self.grasp_pub = self.create_publisher(
            PoseArray, '/grasp_candidates', 10)

        # 工作空间限制（只检测桌面区域）
        self.workspace = {
            'x': [0.2, 0.8],
            'y': [-0.3, 0.3],
            'z': [0.0, 0.3]
        }

    def pc_callback(self, msg):
        # 1. 点云预处理
        points = pointcloud2_to_numpy(msg)  # (N, 3)
        colors = extract_colors(msg)         # (N, 3)

        # 工作空间裁剪
        mask = (
            (points[:, 0] > self.workspace['x'][0]) &
            (points[:, 0] < self.workspace['x'][1]) &
            (points[:, 1] > self.workspace['y'][0]) &
            (points[:, 1] < self.workspace['y'][1]) &
            (points[:, 2] > self.workspace['z'][0]) &
            (points[:, 2] < self.workspace['z'][1])
        )
        points_cropped = points[mask]
        colors_cropped = colors[mask]

        # 2. AnyGrasp 推理
        gg, _ = self.anygrasp.get_grasp(
            points_cropped, colors_cropped,
            lims=list(self.workspace.values()))

        if len(gg) == 0:
            self.get_logger().warn("No grasps detected")
            return

        # 3. 按质量分数排序，取 top-K
        gg.sort_by_score()
        top_k = min(10, len(gg))
        grasps = gg[:top_k]

        # 4. 转换为 ROS PoseArray
        pose_array = PoseArray()
        pose_array.header = msg.header
        for g in grasps:
            pose = grasp_to_pose(
                g.translation, g.rotation_matrix, g.score)
            pose_array.poses.append(pose)

        self.grasp_pub.publish(pose_array)
        self.get_logger().info(
            f"Published {top_k} grasp candidates "
            f"(best score: {grasps[0].score:.3f})")


def grasp_to_pose(translation, rotation, score):
    """将 GraspNet 格式转换为 ROS Pose"""
    from geometry_msgs.msg import Pose
    from scipy.spatial.transform import Rotation as R

    pose = Pose()
    pose.position.x = float(translation[0])
    pose.position.y = float(translation[1])
    pose.position.z = float(translation[2])

    quat = R.from_matrix(rotation).as_quat()  # [x,y,z,w]
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose
```

**将 GraspNet 候选送入 MTC 的集成方式**：

```cpp
// 教学骨架：自定义 MTC Generator Stage，从 GraspNet 候选生成
// 带 target_pose 属性的 InterfaceState，再交给 ComputeIK。
//
// 不建议继承 GeneratePose 后直接调用 spawn(PoseStamped, cost)：
// 当前 MTC 的生成器数据流以 InterfaceState + Solution/SubTrajectory
// 为核心，不同 MoveIt2 版本的 spawn() 签名也会变化。
class GraspNetGraspPose
    : public mtc::Generator
{
public:
    GraspNetGraspPose(const std::string& name,
                      rclcpp::Node::SharedPtr node)
        : Generator(name), node_(node)
    {
        grasp_sub_ = node_->create_subscription<PoseArray>(
            "/grasp_candidates", 10,
            [this](PoseArray::SharedPtr msg) {
                latest_grasps_ = *msg;
            });
    }

    void init(const moveit::core::RobotModelConstPtr& model) override {
        Generator::init(model);
        robot_model_ = model;
    }

    bool canCompute() const override {
        return !generated_ && !latest_grasps_.poses.empty();
    }

    void compute() override {
        if (latest_grasps_.poses.empty()) {
            RCLCPP_WARN(node_->get_logger(),
                "No grasp candidates available");
            return;
        }

        for (size_t i = 0; i < latest_grasps_.poses.size();
             ++i)
        {
            geometry_msgs::msg::PoseStamped grasp_pose;
            grasp_pose.header = latest_grasps_.header;
            grasp_pose.pose = latest_grasps_.poses[i];

            auto scene =
                std::make_shared<planning_scene::PlanningScene>(
                    robot_model_);
            mtc::InterfaceState state(scene);
            state.properties().set("target_pose", grasp_pose);
            state.properties().set("grasp_rank", static_cast<int>(i));

            // 排名靠前的候选 cost 更低。SubTrajectory 这里只作为
            // 生成器输出的解载体；具体构造 API 以目标 MTC 版本为准。
            mtc::SubTrajectory solution;
            double cost = static_cast<double>(i);
            solution.setCost(cost);
            spawn(std::move(state), std::move(solution));
        }
        generated_ = true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<PoseArray>::SharedPtr grasp_sub_;
    moveit::core::RobotModelConstPtr robot_model_;
    PoseArray latest_grasps_;
    bool generated_{false};
};

// 在 MTC Task 中替换标准 GenerateGraspPose，但仍沿用 ComputeIK。
// grasp_frame_transform 描述 TCP/夹爪坐标系相对候选抓取位姿的偏置；
// 例如 GraspNet 输出的是两指中心，而 IK frame 是 panda_hand。
Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
grasp_frame_transform.translation().z() = 0.10;

auto grasp_gen = std::make_unique<GraspNetGraspPose>(
    "anygrasp_candidates", node);
auto compute_ik = std::make_unique<mtc::stages::ComputeIK>(
    "compute_ik", std::move(grasp_gen));
compute_ik->setGroup("manipulator");
compute_ik->setIKFrame(grasp_frame_transform, "panda_hand");
compute_ik->setMaxIKSolutions(8);
// ... 后续 Stage 不变 ...
```

### 完整 Bin-Picking Pipeline ⭐⭐⭐

Bin-picking（料箱抓取）是工业机器人最有价值的应用之一。下面给出从感知到执行的完整 pipeline。

```
┌─────────────────────────────────────────────────────────────┐
│                  Bin-Picking Pipeline                         │
│                                                              │
│  ┌─────────┐  ┌───────────┐  ┌──────────┐  ┌─────────────┐ │
│  │ 点云获取 │→│ 桌面/箱体  │→│ AnyGrasp │→│ 碰撞过滤    │ │
│  │ (D435)   │ │ 分割       │ │ 推理     │ │ +IK 可达检查 │ │
│  └─────────┘  └───────────┘  └──────────┘  └──────┬──────┘ │
│                                                     │        │
│                                              抓取候选列表    │
│                                                     │        │
│  ┌─────────┐  ┌───────────┐  ┌──────────┐  ┌──────▼──────┐ │
│  │ 放置确认 │←│ 搬运执行  │←│ 抓取执行 │←│ MTC 规划    │ │
│  │ (视觉)   │ │ (MTC)     │ │ (MTC)    │ │ (多阶段)    │ │
│  └─────────┘  └───────────┘  └──────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

**完整 BT 编排**：

```xml
<root BTCPP_format="4" main_tree_to_execute="BinPicking">
  <BehaviorTree ID="BinPicking">
    <ReactiveSequence>
      <!-- 全局安全监控 -->
      <IsSystemHealthy/>

      <Sequence name="main_loop">
        <!-- 循环直到料箱为空或目标数量达成 -->
        <SetBlackboard output_key="pick_count" value="0"/>
        <SetBlackboard output_key="target_count" value="10"/>

        <WhileDoElse name="pick_loop">
          <!-- 条件：还需要继续抓取 -->
          <Sequence>
            <IsLessThan key_A="{pick_count}"
                        key_B="{target_count}"/>
            <IsBinNotEmpty/>
          </Sequence>

          <!-- 主体：单次 pick-place -->
          <Sequence>
            <!-- 感知 -->
            <TriggerPointCloud/>
            <WaitForGraspCandidates timeout="3000"
                candidates="{grasp_list}"/>

            <!-- 选择最优抓取 -->
            <SelectBestGrasp candidates="{grasp_list}"
                             selected="{best_grasp}"
                             planning_scene="{ps}"/>

            <!-- 执行抓取 -->
            <Fallback name="grasp_with_recovery">
              <SubTree ID="ExecuteGrasp"
                       grasp_pose="{best_grasp}"/>
              <Sequence>
                <!-- 恢复：换一个抓取候选 -->
                <SelectNextGrasp candidates="{grasp_list}"
                                 index="1"
                                 selected="{alt_grasp}"/>
                <SubTree ID="ExecuteGrasp"
                         grasp_pose="{alt_grasp}"/>
              </Sequence>
            </Fallback>

            <!-- 放置 -->
            <ComputePlacePosition pick_count="{pick_count}"
                                  place_pose="{place_pos}"/>
            <SubTree ID="ExecutePlace"
                     place_pose="{place_pos}"/>

            <!-- 计数器递增 -->
            <IncrementCounter key="{pick_count}"/>
          </Sequence>

          <!-- else：完成 -->
          <Sequence>
            <MoveToHome/>
            <ReportCompletion total="{pick_count}"/>
          </Sequence>
        </WhileDoElse>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>

  <!-- 可复用的抓取子树 -->
  <BehaviorTree ID="ExecuteGrasp">
    <Sequence>
      <OpenGripper/>
      <PlanMTCPick grasp_pose="{grasp_pose}"
                   approach_dist="0.12"/>
      <ExecuteMTCPlan/>
      <VerifyGraspForce min_force="2.0" result="{ok}"/>
      <Precondition if="{ok}" else="FAILURE">
        <AlwaysSuccess/>
      </Precondition>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="ExecutePlace">
    <Sequence>
      <PlanMTCPlace place_pose="{place_pose}"/>
      <ExecuteMTCPlan/>
      <OpenGripper/>
      <MoveToSafe/>
    </Sequence>
  </BehaviorTree>
</root>
```

**碰撞过滤与 IK 可达检查**：

```cpp
// 对 GraspNet 候选进行工程级过滤
std::vector<GraspCandidate> filterGrasps(
    const std::vector<GraspCandidate>& candidates,
    const planning_scene::PlanningScenePtr& scene,
    const moveit::core::JointModelGroup* jmg,
    const std::string& ee_link)
{
    std::vector<GraspCandidate> filtered;

    for (const auto& grasp : candidates) {
        // 1. 质量分数过滤
        if (grasp.score < 0.3) continue;  // 低质量跳过

        // 2. 工作空间过滤
        if (!isInWorkspace(grasp.pose)) continue;

        // 3. IK 可达性检查
        moveit::core::RobotState state(scene->getRobotModel());
        bool ik_ok = state.setFromIK(jmg, grasp.pose, ee_link,
                                      0.05);  // 50ms 超时
        if (!ik_ok) continue;

        // 4. 碰撞检查（包括 approach 路径）
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        scene->checkCollision(req, res, state);
        if (res.collision) continue;

        // 5. approach 路径可行性
        Eigen::Vector3d approach_dir =
            grasp.rotation * Eigen::Vector3d(0, 0, -1);
        bool approach_ok = checkApproachPath(
            scene, state, approach_dir, 0.10);
        if (!approach_ok) continue;

        filtered.push_back(grasp);
    }

    // 按（分数 - 碰撞风险）综合排序
    std::sort(filtered.begin(), filtered.end(),
        [](const auto& a, const auto& b) {
            return a.score > b.score;
        });

    return filtered;
}
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MTC 的 approach 距离设太短
   错误做法：approach distance = 2cm
   现象：approach 阶段碰撞，MTC 返回 0 解
   根本原因：2cm 不够安全间距
   正确做法：approach 设 5-15cm，宁多勿少
```

### 抓取质量评估

在选择抓取方案时，需要量化评估每个候选抓取的质量。常用的评估指标包括：

**1. Grasp Wrench Space (GWS) 分析**

给定接触点和接触法向量，GWS 分析计算抓取能抵抗的力/力矩空间。GWS 体积越大，抓取越稳定。

**2. 力封闭判据**

力封闭要求：对于任意方向的外力 $f_{ext}$，都存在接触力 $f_c$ 使得：

$$\sum f_c + f_{ext} = 0, \quad f_c \in \text{friction cone}$$

对于平行夹爪，这简化为检查夹爪两侧的摩擦锥是否覆盖所有方向。当摩擦系数 $\mu > \tan(\alpha)$（其中 $\alpha$ 是物体表面法线与夹爪力方向的夹角）时，力封闭成立。

**3. 工程近似评估**

在实际系统中，完整的 GWS 分析计算量较大。工程上常用以下近似指标：

| 指标 | 计算方式 | 含义 |
|------|---------|------|
| 接触面积 | 接触 patch 的面积估计 | 面积越大越稳定 |
| 力臂比 | 抓取力到物体重心的力臂 | 力臂越小越不容易翻转 |
| 对称性 | 接触点是否关于重心对称 | 对称抓取更稳定 |
| 碰撞间距 | 抓取路径到障碍物的最小距离 | 间距越大越安全 |

MTC 的 `GenerateGraspPose` 不做力封闭分析——它只在几何上采样抓取角度。如果需要更高质量的抓取评估，可以实现自定义 Stage 或在 BT 层面加入抓取质量过滤。

### 从仿真到真机的抓取调整

在仿真中成功的抓取策略，在真机上经常需要调整：

| 差异源 | 仿真 | 真机 | 调整方式 |
|--------|------|------|---------|
| 接触模型 | 完美摩擦、刚性接触 | 有限摩擦、弹性接触 | 增大夹爪力 |
| 物体位姿精度 | Ground truth | 感知误差 +-5mm | 增大接近距离 |
| 夹爪响应时间 | 即时 | 50-200ms 延迟 | 闭合后等待确认 |
| 力传感器 | 完美 | 有噪声和漂移 | 加滤波器和阈值 |

> **跨领域类比**：仿真中的抓取调试类似于在白板上设计算法——逻辑正确但忽略了实际约束。真机调试类似于工程实现——必须处理噪声、延迟、精度等现实因素。两个阶段都不可缺少：先在仿真中验证逻辑正确性，再在真机上调整物理参数。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MTC 的 approach 距离设太短
   错误做法：approach distance = 2cm
   现象：approach 阶段碰撞，MTC 返回 0 解
   根本原因：2cm 不够安全间距
   正确做法：approach 设 5-15cm，宁多勿少
```

```
💡 概念误区：认为"一种抓取策略适用所有物体"
   新手想法："从上方向下抓取对所有物体都有效"
   实际上：不同形状需要不同策略：
          - 扁平物体（如书）→ 从侧面夹持
          - 高圆柱体（如瓶子）→ 从侧面夹持中段
          - 不规则物体 → 需要多个候选抓取点的评估
          MTC 的 Alternatives Stage 可以同时尝试多种抓取策略
```

```
🧠 思维陷阱：忽视抓取的动态稳定性
   新手想法："静态抓取稳定就够了"
   实际上：搬运过程中机器人加速/减速产生惯性力，
          可能破坏静态稳定的抓取。需要考虑：
          1. 搬运轨迹的加速度上限
          2. 物体质量和重心位置
          3. 夹爪力的安全裕度（通常设为静态所需力的 2-3 倍）
```

### 练习

1. **[A 型]** 修改 MTC 的 `angle_delta`（30度 → 15度 → 5度），观察抓取候选数量和成功率变化。

2. **[A 型]** 在 Gazebo 中改变物体位置（靠近桌边、靠近障碍物），观察 MTC 自动选择不同的抓取角度。记录哪些位置导致规划失败。

3. **[思考题]** 圆柱体（如瓶子）的抓取策略与立方体有什么区别？考虑旋转对称性如何影响 MTC 的 `GenerateGraspPose` 采样。

---

## M15.5 完整项目结构与配置 ⭐⭐

### 项目目录结构

```
mini_manip_ws/
├── src/
│   ├── mini_manip_description/         # URDF + Xacro
│   │   ├── urdf/
│   │   │   ├── panda.urdf.xacro
│   │   │   ├── panda.ros2_control.xacro
│   │   │   └── panda_hand.urdf.xacro
│   │   ├── meshes/
│   │   └── CMakeLists.txt
│   │
│   ├── mini_manip_moveit_config/       # MoveIt2 配置
│   │   ├── config/
│   │   │   ├── panda.srdf
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   ├── controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   └── pipeline_configs.yaml
│   │   ├── launch/
│   │   │   └── move_group.launch.py
│   │   └── CMakeLists.txt
│   │
│   ├── mini_manip_bringup/             # Launch + 控制器
│   │   ├── config/
│   │   │   └── ros2_controllers.yaml
│   │   ├── launch/
│   │   │   ├── bringup.launch.py
│   │   │   └── gazebo.launch.py
│   │   ├── worlds/
│   │   │   └── table_scene.sdf
│   │   └── CMakeLists.txt
│   │
│   ├── mini_manip_mtc/                 # MTC 任务
│   │   ├── src/pick_place_task.cpp
│   │   ├── include/.../task.hpp
│   │   └── CMakeLists.txt
│   │
│   ├── mini_manip_bt/                  # BT 编排
│   │   ├── src/
│   │   │   ├── bt_nodes/
│   │   │   │   ├── detect_object.cpp
│   │   │   │   ├── plan_pick.cpp
│   │   │   │   ├── plan_place.cpp
│   │   │   │   └── move_to_home.cpp
│   │   │   └── bt_main.cpp
│   │   ├── trees/
│   │   │   └── mini_manip.xml
│   │   ├── plugin.xml
│   │   └── CMakeLists.txt
│   │
│   └── mini_manip_perception/          # 感知 (简化)
│       ├── src/sim_detector.cpp
│       └── CMakeLists.txt
│
└── README.md
```

### 关键配置文件

**ros2_controllers.yaml**：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    gripper_controller:
      type: gripper_controllers/GripperActionController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - panda_joint1
      - panda_joint2
      - panda_joint3
      - panda_joint4
      - panda_joint5
      - panda_joint6
      - panda_joint7
    command_interfaces: [position]
    state_interfaces: [position, velocity]

gripper_controller:
  ros__parameters:
    joint: panda_finger_joint1
    action_monitor_rate: 20.0
    goal_tolerance: 0.002
    max_effort: 50.0
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：launch 文件中组件启动顺序不对
   错误做法：controller_manager 启动前就 spawn 控制器
   现象：spawner 报 "controller_manager not available"
   正确做法：用 launch 事件机制确保顺序：
            robot_state_publisher → controller_manager →
            spawner → move_group → bt_node
```

### 练习

1. **[A 型]** 从零创建上述项目结构，实现第一阶段（Gazebo 中启动 Panda + 可交互式规划）。

---

## M15.6 调试方法论与性能指标 ⭐⭐

### 调试层级

系统不工作时，**从底层向上**排查：

```
Layer 4 (硬件) → Layer 3 (控制) → Layer 2 (规划) → Layer 1 (编排)

1. 硬件层: /joint_states 有数据吗? 频率对吗?
2. 控制层: 控制器是 active 状态吗? 命令被接受了吗?
3. 规划层: MTC 规划成功了吗? 碰撞检测正确吗?
4. 编排层: BT 节点返回了什么状态? 数据流对吗?
```

### 关键调试命令

```bash
# Layer 4
ros2 topic echo /joint_states --once
ros2 control list_hardware_interfaces

# Layer 3
ros2 control list_controllers
ros2 action list

# Layer 2
# RViz: PlanningScene + MotionPlanning 面板

# Layer 1
# Groot2 连接实时监控
# SqliteLogger 日志回放
```

### 性能指标框架

| 指标类别 | 具体指标 | 目标值 |
|---------|---------|-------|
| **规划** | MTC plan 耗时 | <2s |
| | 规划成功率 | >95% |
| | IK 求解时间 | <50ms |
| **执行** | 轨迹执行时间 | <10s |
| | 关节跟踪误差 | <0.01 rad |
| | 端到端周期 | <15s |
| **鲁棒性** | 连续成功次数 | >10 |
| | 错误恢复成功率 | >80% |
| **资源** | CPU 使用率 | <50% |
| | RT 循环 jitter | <100us |

### ⚠️ 常见陷阱

```
🧠 思维陷阱：只测量成功场景
   新手想法："成功执行一次 8 秒，性能不错"
   实际上：需要连续运行 100 次统计成功率、随机改变物体位置、
          注入故障测试错误恢复。工业标准是「多大概率能做到」
          而非「能不能做到」。
```

### 常见集成问题诊断表

| 问题类别 | 典型症状 | 诊断步骤 | 根本原因 |
|---------|---------|---------|---------|
| **坐标系** | 规划的目标位置偏差很大 | 1. `ros2 topic echo /tf` 检查坐标系树 2. 确认目标位姿的 frame_id 正确 | 目标位姿在错误的坐标系中 |
| **时间同步** | 感知数据与机器人状态不同步 | 1. 检查 use_sim_time 参数 2. 比较消息时间戳 | 仿真时间与系统时间不一致 |
| **启动顺序** | spawner 报错或 Action Server 不可用 | 1. 检查 launch 日志 2. 手动按顺序启动 | 组件间依赖未正确处理 |
| **参数冲突** | 同一参数在多处定义不一致 | 1. `ros2 param list` 2. 搜索所有 YAML 中的参数 | YAML 文件间参数不一致 |
| **内存泄漏** | 长时间运行后系统变慢 | 1. `htop` 监控内存 2. Valgrind 检查 | MTC Task 对象未正确释放 |

### 性能瓶颈分析方法

当端到端性能不达标时，需要**分层计时**找到瓶颈：

```cpp
// 在异步 worker 或离线 benchmark 中加入计时；不要阻塞 BT tick 线程。
auto t0 = std::chrono::steady_clock::now();

// MTC 规划
task_->plan(5);
const bool has_solution = !task_->solutions().empty();
auto t1 = std::chrono::steady_clock::now();

// 执行
if (!has_solution) {
  throw std::runtime_error("MTC planning failed: no solution");
}
auto exec_result = task_->execute(*task_->solutions().front());
const bool exec_ok =
  exec_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
auto t2 = std::chrono::steady_clock::now();

// 输出
double plan_ms = std::chrono::duration<double, std::milli>(
  t1 - t0).count();
double exec_ms = std::chrono::duration<double, std::milli>(
  t2 - t1).count();

RCLCPP_INFO(logger, "Plan: %.1fms, Execute: %.1fms, success: %s",
  plan_ms, exec_ms, exec_ok ? "true" : "false");
```

**典型瓶颈分布**（Franka Panda 桌面 pick-and-place）：

```
端到端 ~12s 的典型分解:
├── 感知检测:      ~200ms   (  2%)
├── MTC pick 规划:  ~1.5s   ( 13%)
├── pick 执行:      ~4.0s   ( 33%)
├── MTC place 规划: ~1.0s   (  8%)
├── place 执行:     ~3.5s   ( 29%)
├── 返回 home:      ~1.5s   ( 13%)
└── BT 开销:        ~0.3s   (  2%)
```

**优化方向**：
- 规划瓶颈 → 换更快的规划器、减少 MTC 搜索数
- 执行瓶颈 → 提高速度缩放因子（velocity_scaling）
- 感知瓶颈 → 预计算、缓存上次结果

### RViz 调试技巧

| RViz 面板 | 用途 | 何时使用 |
|-----------|------|---------|
| **MotionPlanning** | 交互式设目标、可视化规划路径 | 单步调试规划 |
| **PlanningScene** | 显示碰撞对象和 ACM | 调试碰撞问题 |
| **TF** | 显示坐标系树 | 调试坐标系问题 |
| **MarkerArray** | 自定义可视化（如抓取位姿候选） | 调试抓取规划 |
| **MTC Tasks** | MTC 多阶段方案可视化 | 调试 MTC Stage |

### ⚠️ 常见陷阱

```
🧠 思维陷阱：只测量成功场景
   新手想法："成功执行一次 8 秒，性能不错"
   实际上：需要连续运行 100 次统计成功率、随机改变物体位置、
          注入故障测试错误恢复。工业标准是「多大概率能做到」。
```

```
⚠️ 编程陷阱：性能优化时降低安全参数
   错误做法：为了加快速度把碰撞检测关掉或降低安全间距
   现象：偶发碰撞事故
   正确做法：先通过分层计时找到真正的瓶颈，
            然后针对性优化（换规划器、调参数），
            不要减少安全检查
```

```
💡 概念误区：认为调试只需要 print/log
   新手想法："加够 RCLCPP_INFO 就能找到问题"
   实际上：机器人系统是多进程、多话题、空间三维的——
          纯文本日志很难展示空间关系。
          RViz 可视化（碰撞对象、轨迹、坐标系）和
          Groot2 日志回放是更有效的调试工具。
   推荐顺序：RViz 可视化 → Groot2 回放 → 文本日志
```

### 练习

1. **[A 型]** 连续运行 20 次 pick-and-place，记录每次的规划时间、执行时间和成功/失败。画出时间分布直方图。

2. **[A 型]** 注入故障测试：(a) 执行过程中删除 PlanningScene 中的物体，(b) 规划过程中添加新障碍物。观察错误恢复行为。

3. **[A 型]** 用分层计时找到系统的性能瓶颈。尝试通过换规划器或调参数将端到端时间缩短 20%。

---

## M15.6B 性能评估指标体系 ⭐⭐⭐

### 动机

"系统能工作"和"系统能可靠工作"之间有巨大鸿沟。工业场景要求的不是"演示一次成功"，而是"每小时 X 次循环、Y% 成功率、Z mm 精度"。建立量化的性能评估体系是从研究原型走向工程部署的必经之路。

### 四维评估框架

| 维度 | 关键指标 | 目标值（桌面 pick-place） | 工业标准 |
|------|---------|-------------------------|---------|
| **效率** | 单次周期时间（Cycle Time） | <15s | 6-12s |
| | 每小时处理量（Throughput） | >200 次/h | 300-600 次/h |
| | 空闲时间占比 | <10% | <5% |
| **可靠性** | 单次成功率（First-pass Yield） | >90% | >99% |
| | 含恢复成功率（Overall Yield） | >95% | >99.9% |
| | 平均无故障时间（MTBF） | >1h | >8h |
| | 平均恢复时间（MTTR） | <30s | <10s |
| **精度** | 放置位置精度 | <5mm | <1mm |
| | 放置姿态精度 | <3 度 | <0.5 度 |
| | 抓取位姿偏差 | <3mm | <1mm |
| **安全** | 碰撞事件数 | 0 次/100 次 | 0 次/10000 次 |
| | 急停触发率 | <5% | <0.1% |
| | 最大关节偏差 | <0.05 rad | <0.01 rad |

### 自动化性能评估脚本

```python
#!/usr/bin/env python3
"""
bin-picking 性能评估脚本
连续执行 N 次 pick-place，记录所有指标
"""
import rclpy
from rclpy.node import Node
import json
import time
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class TrialResult:
    """单次试验结果"""
    trial_id: int
    success: bool
    recovery_needed: bool = False
    recovery_success: bool = False

    # 时间指标 (秒)
    perception_time: float = 0.0
    plan_pick_time: float = 0.0
    execute_pick_time: float = 0.0
    plan_place_time: float = 0.0
    execute_place_time: float = 0.0
    return_home_time: float = 0.0
    total_time: float = 0.0

    # 精度指标 (米/弧度)
    place_position_error: float = 0.0
    place_orientation_error: float = 0.0

    # 错误信息
    failure_stage: Optional[str] = None
    failure_reason: Optional[str] = None


@dataclass
class EvalReport:
    """评估报告"""
    num_trials: int = 0
    trials: List[TrialResult] = field(default_factory=list)

    def compute_metrics(self):
        """计算汇总指标"""
        successes = [t for t in self.trials if t.success]
        failures = [t for t in self.trials if not t.success]
        recoveries = [t for t in self.trials
                      if t.recovery_needed]

        metrics = {
            # 可靠性
            'first_pass_yield': (
                len([t for t in self.trials
                     if t.success and not t.recovery_needed])
                / self.num_trials * 100),
            'overall_yield': (
                len(successes) / self.num_trials * 100),
            'recovery_rate': (
                len([t for t in recoveries
                     if t.recovery_success])
                / max(len(recoveries), 1) * 100),

            # 效率
            'mean_cycle_time': np.mean(
                [t.total_time for t in successes]),
            'std_cycle_time': np.std(
                [t.total_time for t in successes]),
            'max_cycle_time': np.max(
                [t.total_time for t in successes]),
            'throughput_per_hour': (
                3600.0 / np.mean(
                    [t.total_time for t in successes])),

            # 时间分布
            'mean_perception': np.mean(
                [t.perception_time for t in successes]),
            'mean_plan_pick': np.mean(
                [t.plan_pick_time for t in successes]),
            'mean_exec_pick': np.mean(
                [t.execute_pick_time for t in successes]),
            'mean_plan_place': np.mean(
                [t.plan_place_time for t in successes]),
            'mean_exec_place': np.mean(
                [t.execute_place_time for t in successes]),

            # 精度
            'mean_place_error_mm': np.mean(
                [t.place_position_error * 1000
                 for t in successes]),
            'max_place_error_mm': np.max(
                [t.place_position_error * 1000
                 for t in successes]),

            # 失败分析
            'failure_stages': {},
        }

        # 按失败阶段统计
        for t in failures:
            stage = t.failure_stage or 'unknown'
            metrics['failure_stages'][stage] = (
                metrics['failure_stages'].get(stage, 0) + 1)

        return metrics

    def print_report(self):
        m = self.compute_metrics()
        print("=" * 60)
        print("  Bin-Picking Performance Report")
        print("=" * 60)
        print(f"  Trials: {self.num_trials}")
        print(f"  First-pass Yield: {m['first_pass_yield']:.1f}%")
        print(f"  Overall Yield:    {m['overall_yield']:.1f}%")
        print(f"  Recovery Rate:    {m['recovery_rate']:.1f}%")
        print("-" * 60)
        print(f"  Mean Cycle Time:  "
              f"{m['mean_cycle_time']:.2f} +/- "
              f"{m['std_cycle_time']:.2f} s")
        print(f"  Max Cycle Time:   {m['max_cycle_time']:.2f} s")
        print(f"  Throughput:       "
              f"{m['throughput_per_hour']:.0f} picks/h")
        print("-" * 60)
        print("  Time Breakdown (mean):")
        print(f"    Perception:   "
              f"{m['mean_perception']*1000:.0f} ms")
        print(f"    Plan Pick:    "
              f"{m['mean_plan_pick']*1000:.0f} ms")
        print(f"    Exec Pick:    "
              f"{m['mean_exec_pick']*1000:.0f} ms")
        print(f"    Plan Place:   "
              f"{m['mean_plan_place']*1000:.0f} ms")
        print(f"    Exec Place:   "
              f"{m['mean_exec_place']*1000:.0f} ms")
        print("-" * 60)
        print(f"  Place Error:  "
              f"{m['mean_place_error_mm']:.2f} mm (mean), "
              f"{m['max_place_error_mm']:.2f} mm (max)")
        if m['failure_stages']:
            print("-" * 60)
            print("  Failure Analysis:")
            for stage, count in m['failure_stages'].items():
                print(f"    {stage}: {count}")
        print("=" * 60)

    def save_json(self, path):
        data = {
            'num_trials': self.num_trials,
            'metrics': self.compute_metrics(),
            'trials': [asdict(t) for t in self.trials],
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
```

**性能评估的最佳实践**：

| 实践 | 说明 | 不做的后果 |
|------|------|-----------|
| 连续运行 N >= 50 次 | 统计显著性需要足够样本 | N=5 的成功率 80% 可能是偶然 |
| 随机化物体位姿 | 每次试验随机放置物体 | 固定位姿只证明"一个点"可行 |
| 注入故障测试 | 人为制造感知失败、夹爪故障 | 不测试恢复 = 不知道恢复能否工作 |
| 记录失败原因 | 每次失败标注阶段和原因 | 无法针对性改进 |
| 对比基线 | 改进前后用相同测试集对比 | 无法量化改进幅度 |
| 时间分解 | 分层记录各阶段耗时 | 无法找到瓶颈 |

> **本质洞察**：性能评估不是项目的最后一步——它是贯穿整个开发过程的反馈环。每次修改（换规划器、调参数、改恢复策略）后都应该重新评估，用数据证明改进是真实的。这和软件工程中的"regression testing"是同一个思想。

---

## M15.7 交付物与评估标准 ⭐

### 交付物

1. **GitHub 仓库**：完整 ROS2 workspace
2. **README.md**：架构图、安装步骤、运行方法、演示 GIF
3. **技术报告**（1-2 页）：IK/规划器/时间参数化对比数据 + 选型结论
4. **Groot2 日志**：可回放的行为树执行记录

### 评估标准

| 维度 | 及格 | 优秀 |
|------|------|------|
| 功能完整性 | pick 1 个物体成功 | pick 3+ 物体全部成功 |
| 错误恢复 | 有 ReturnHome | Fallback 自动重试+重检测 |
| sim-to-real | Gazebo + mock 两目标 | 三目标 + 详细部署指南 |
| 代码质量 | 能运行 | clang-format + clang-tidy |
| 性能对比 | 默认配置 | 量化对比报告 |
| 文档 | 有 README | README + 架构图 + 视频 |
| BT 调试 | 有日志 | Groot2 可回放 |

---

## M15.8 从仿真到实机的部署流程 ⭐⭐⭐

### 动机

sim-to-real swap 是本项目的核心设计理念——同一份代码在仿真和真机上运行，只通过 launch 参数切换。但「参数一样」不代表「行为一样」——仿真和真机之间存在系统性差异，需要一套标准化的部署流程来管理。

### 三阶段部署流程

```
阶段 1: Mock Hardware (RViz 纯可视化)
  目的: 验证控制链路和 BT 逻辑
  特点: 无物理仿真，关节位置直接跟随命令
  验收: BT 执行完整流程，Groot2 日志正常

        │ 通过
        ▼

阶段 2: Gazebo 仿真
  目的: 验证运动规划和碰撞避免
  特点: 有物理仿真，但物理参数可能不准
  验收: 物体成功被抓取、搬运、放置

        │ 通过
        ▼

阶段 3: 真机部署
  目的: 在真实硬件上执行
  特点: 必须处理通信延迟、传感器噪声、摩擦差异
  验收: 10 次连续成功
```

### 真机首次运行安全规程

真机首次运行时**必须遵循以下安全规程**——这不是建议，而是强制要求：

1. **降低速度**：`velocity_scaling_factor` 设为 0.1（正常值的 10%）
2. **单步执行**：不用 BT 自动编排，手动一步一步触发每个动作
3. **急停就绪**：操作员手持急停按钮，随时准备按下
4. **无人区域**：机器人工作范围内不得有人
5. **先不抓取**：先只做空间运动（不闭合夹爪），确认运动范围正确
6. **逐步加速**：确认安全后，逐步提高 `velocity_scaling_factor`（0.1 → 0.3 → 0.5 → 1.0）

> **反事实推理**：如果不遵循安全规程直接以全速运行会怎样？仿真中的碰撞对象位置可能与真实环境有偏差——如果桌子的实际位置比 PlanningScene 中的位置高 2cm，机器人可能以全速撞上桌面。2cm 的偏差在仿真中无关紧要，但全速碰撞可能损坏机器人末端或桌上的设备。

### 参数调整清单

| 参数类别 | 仿真中的值 | 真机建议值 | 原因 |
|---------|-----------|-----------|------|
| velocity_scaling | 1.0 | 0.3-0.5 | 真机需要更保守的速度 |
| 碰撞安全间距 | 0.01m | 0.03-0.05m | 补偿感知误差 |
| approach 距离 | 0.10m | 0.12-0.15m | 补偿位姿估计误差 |
| 夹爪力 | 默认 | 增大 50-100% | 补偿真实摩擦差异 |
| 夹爪闭合后等待 | 0ms | 200-500ms | 等待夹爪完全闭合 |
| MTC 规划超时 | 5s | 10s | 真机环境更复杂 |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：真机上使用仿真时间
   错误做法：launch 参数中 use_sim_time:=true
   现象：TF 变换失败（时间戳不匹配），规划超时
   根本原因：use_sim_time=true 让所有节点使用 /clock topic 的时间，
            但真机没有 Gazebo 发布 /clock
   正确做法：真机部署时 use_sim_time:=false
   自检方法：ros2 param get /move_group use_sim_time
```

```
🧠 思维陷阱：在真机上频繁修改代码
   新手想法："真机上发现问题就直接改代码重新编译"
   实际上：真机时间宝贵（可能需要预约实验室、多人协调）。
          所有逻辑问题应该在仿真中解决，真机上只做参数调整。
   正确流程：
     仿真中修复 bug → 仿真中验证 → 部署到真机 → 只调参数
```

### 练习

1. **[A 型]** 按照三阶段部署流程，先在 Mock Hardware 中验证 BT 逻辑完整性，然后切换到 Gazebo 验证物理交互。记录每个阶段发现的问题和解决方案。

2. **[思考题]** 如果你有一台 UR5e 但没有 Franka Panda，需要修改哪些文件才能将本项目移植到 UR5e？列出所有需要修改的文件，并标注修改的难度（只改配置 / 需要改代码 / 需要重写）。

---

## 本章小结

| 知识点 | 核心内容 | 难度 |
|--------|---------|------|
| M15.1 系统架构 | 四层架构、数据流、接口设计 | ⭐⭐ |
| M15.2 分阶段开发 | 五阶段计划、vertical slice first | ⭐⭐ |
| M15.3 感知集成 | 三种模式、手眼标定概述 | ⭐⭐ |
| M15.4 抓取规划 | 力闭合、MTC 抓取采样 | ⭐⭐⭐ |
| M15.5 项目结构 | 目录结构、配置文件、launch | ⭐⭐ |
| M15.6 调试方法论 | 层级调试、性能指标 | ⭐⭐ |
| M15.7 交付物 | 评估标准、仓库要求 | ⭐ |
| M15.8 sim-to-real 部署 | 三阶段部署、安全规程、参数调整 | ⭐⭐⭐ |

## 跨章综合练习 ⭐⭐⭐

**题目**：综合 M03（IK 求解器）+ M07（OMPL）+ M12（ros2_control）+ M13（BT.CPP）+ M14（MTC），完成以下全栈挑战：

1. **IK+规划器量化对比**（M03+M07+M14）：在 Mini-Manip 系统中，分别使用 KDL/TRAC-IK/pick-ik 三种 IK 求解器和 RRT-Connect/BIT* 两种规划器的所有组合（3x2=6 种），对 100 个随机 pick-and-place 目标运行 MTC Task。记录每种组合的规划时间、成功率和路径长度。分析哪种组合在什么指标上最优，并解释原因。

2. **错误注入测试**（M13+M14）：在 BT 编排的 pick-and-place 流程中，用 Gazebo 插件在以下三个时刻注入故障：(a) 规划阶段移动障碍物导致碰撞检测失败；(b) 执行阶段改变物体位置导致抓取落空；(c) 搬运阶段旋转物体导致 place 失败。验证 BT 的 Fallback/Retry 恢复机制能否在每种故障下自动恢复。统计恢复成功率。

3. **RT 性能审计**（M11+M12）：用 ftrace 跟踪 mini_manip 系统运行时 ros2_control 的 RT 循环。测量 read()/update()/write() 各阶段耗时。确认没有隐藏的堆分配（用 `EIGEN_RUNTIME_NO_MALLOC` 或 LD_PRELOAD malloc 拦截）。

4. **一页纸选型报告**：基于上述实验数据，为一个假设的工业 bin-picking 场景（吞吐量要求: 12 cycles/min，成功率要求: >98%）撰写 IK+规划器+时间参数化的选型推荐报告。

**评分标准**：实验数据完整（6 种组合 x 100 目标）、故障注入覆盖三种场景、RT 审计确认零堆分配、选型报告有数据支撑。

---

## 累积项目：最终交付

本章是 Mini-Manip 累积项目的最终阶段。

```
项目进度回顾:
P01:     URDF 模型           ← 基础
M01-M03: 运动学/动力学        ← 理论
M04:     碰撞检测             ← 环境约束
M05-M06: 优化建模/自动微分     ← 数值工具
M07-M09: 采样/优化/GPU 规划    ← 规划算法
M10:     时间参数化            ← 轨迹成形
M11:     实时 C++              ← 工程能力
M12:     ros2_control          ← 执行层
M13:     BT.CPP               ← 编排层
M14:     MoveIt2 + MTC         ← 规划层
M15:     端到端集成 + 交付      ← 你在这里
```

**本章不设独立练习——整个章节就是一个大项目。**

五个阶段就是你的开发计划：
1. 环境搭建（2天）→ 2. MTC pick-and-place（3天）→ 3. BT 编排（2天）→ 4. 对比优化（2天）→ 5. sim-to-real 验证（1天）

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| MoveIt2 pick-and-place 教程 | ⭐ | 官方入门 |
| franka_ros2 示例仓库 | ⭐⭐ | Franka 完整示例 |
| Fang et al. (2020) "GraspNet-1Billion" CVPR | ⭐⭐⭐ | 学习型抓取 |
| Fang et al. (2023) "AnyGrasp" T-RO | ⭐⭐⭐ | 通用抓取 |
| Tsai & Lenz (1989) Hand-Eye Calibration | ⭐⭐⭐ | 手眼标定经典 |
| Tedrake (2023) "Robotic Manipulation" MIT | ⭐⭐⭐ | 操作全景 |
| BenchBot (2021) "Benchmarking Robot Manipulation" | ⭐⭐⭐ | 性能评估 |

---

## M15.9 前沿展望：AnyGrasp 2.0、Foundation Grasp Model 与未来方向 ⭐⭐⭐⭐

完成 Mini-Manip 综合项目后，你已经具备了从感知到执行的完整抓取管线搭建能力。以下简要介绍抓取感知领域正在发生的重要进展，帮助你定位下一步的研究或工程方向。

### AnyGrasp 最新进展

AnyGrasp（Fang et al., T-RO 2023）是 GraspNet 系列的最新工作，相比原始 GraspNet-1Billion 有显著改进：

| 维度 | GraspNet (2020) | AnyGrasp (2023) | 改进原因 |
|------|-----------------|-----------------|---------|
| 推理速度 | ~200ms | ~50ms | 更轻量的骨干网络 + TensorRT 优化 |
| 抓取成功率 | ~80% (已知物体) | ~90%+ (未知物体) | 更大规模训练数据 + 改进的点云编码 |
| 泛化能力 | 依赖 GraspNet-1Billion 数据集 | 跨域泛化（训练域外物体） | 对比学习 + 几何先验 |
| 部署方式 | Python + PyTorch | SDK 封装 + ROS2 集成 | 工业化部署 |

**工程意义**：对于 Mini-Manip 项目，AnyGrasp 可以替代 MTC 的 `GenerateGraspPose` Stage——不再需要手动设计抓取策略（顶部/侧面/角度采样），而是让模型从点云直接输出多个候选抓取位姿和质量评分。这把"抓取规划"从"工程师经验驱动"变成了"数据驱动"。

### Foundation Grasp Model 概念

2025 年的一个学术前沿方向是"Foundation Grasp Model"——训练一个大规模、跨域泛化的抓取基础模型，类似于 NLP 领域的 GPT 或视觉领域的 SAM。

**核心思路**：

1. **大规模预训练**：在数百万个仿真场景中生成海量抓取数据（利用物理引擎验证抓取成功/失败），预训练一个通用的"点云→抓取位姿"模型
2. **少样本适配**：在真实场景中只需要少量标注数据即可适配特定物体类别或夹爪类型
3. **多模态输入**：除了点云，还融合语义信息（"抓住杯子的把手"）和物理属性（物体重量、摩擦系数）

**代表工作**：

| 工作 | 年份 | 特点 |
|------|------|------|
| UniGrasp (Shao et al.) | 2020 | 跨夹爪泛化 |
| FoundationGrasp | 2024 | 大规模预训练 + 零样本泛化 |
| GraspGPT (Tang et al.) | 2023 | 语言引导的语义抓取 |
| SparseDFF (Wang et al.) | 2024 | 稀疏扩散特征场用于 6-DOF 抓取 |

> **反事实推理**：如果 Foundation Grasp Model 成熟了会怎样？当前 Mini-Manip 项目中，更换物体类别（从立方体到水瓶）需要调整 GenerateGraspPose 的参数（angle_delta、approach 方向）。有了 Foundation Grasp Model，模型直接从点云推断抓取策略，换物体只需要换点云输入——工程师不再需要理解"力闭合"或手动设计抓取策略。但这也意味着：当模型失败时（如极端形状的物体），工程师可能缺乏手动干预的能力——这正是我们在 M15.4 中教授力闭合理论的原因。

> **本质洞察**：Foundation Grasp Model 的演进路径与自动驾驶中的感知模型类似——从规则驱动（手工设计特征）到学习驱动（端到端神经网络），再到基础模型驱动（预训练+微调）。每一步都降低了工程师的手动工作量，但也增加了对训练数据质量和分布覆盖的依赖。机器人工程师的角色从"设计抓取策略"转变为"评估和验证模型输出的安全性"。

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| Gazebo 中 Panda 不动 | 控制器未激活 | 1. list_controllers 2. 检查 spawner | M12 |
| MTC 规划 0 解 | 碰撞或 IK 无解 | 1. RViz 可视化 2. 增大 IK 采样 3. 检查方向 | M14 |
| BT 一直 RUNNING | Action Server 未启动 | 1. action list 2. 检查 move_group | M13 |
| 夹爪抓取滑落 | 摩擦参数不足 | 1. 增大 mu 2. 增大力 3. attach plugin | M15.2 |
| sim-to-real 行为差异 | 参数不匹配 | 1. 对比参数 2. 降低速度 3. 逐步验证 | M15.2 |
| launch 启动失败 | 组件依赖未满足 | 1. 检查日志 2. 加 TimerAction | M15.5 |
| 端到端性能差 | 规划耗时过长 | 1. 分层计时 2. 换规划器 3. 减少搜索数 | M15.6 |
