> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M14 MoveIt2 + MTC 工业集成

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回 M07/M12/`02_基础/30_软件工程` 复习）

1. `pluginlib` 的 ClassLoader 如何通过字符串动态创建 C++ 对象？（`02_基础/30_软件工程/10_设计模式与高级惯用法`）
2. OMPL 中 RRT-Connect 的双树扩展策略是什么？它为什么比单树 RRT 更快？（M07）
3. ros2_control 的 JointTrajectoryController 如何接收和执行轨迹？（M12）
4. IK 求解器的性能光谱是什么？从 opw (~0.1us) 到 KDL (~2ms) 差了多少量级？（M03）
5. Composite Pattern 如何用统一接口处理叶子节点和容器节点？（`02_基础/30_软件工程/10_设计模式与高级惯用法`）

## 本章目标

学完本章后，你能够：
1. **理解** MoveIt2 的 pluginlib 三层工厂架构，知道每个组件如何被运行时替换
2. **熟练使用** MoveGroupInterface 和 MoveItCpp 两套 API 进行运动规划
3. **掌握** PlanningScene 的 diff-based 同步机制和碰撞对象管理
4. **使用 MTC** 编排多阶段操作任务（pick-and-place 完整流程）
5. **配置** 链式/并行规划管线，选择最优规划器组合
6. **独立搭建** 从 MoveIt Setup Assistant 到 Gazebo 执行的完整系统

---

## M14.1 MoveIt2 架构全景 ⭐⭐

### 动机 ⭐⭐

当你需要让一台 7-DOF 机械臂从当前位姿移动到目标位姿，同时避开桌子上的障碍物——这个问题涉及运动学（IK 求解）、碰撞检测（环境感知）、路径搜索（OMPL 规划器）、轨迹平滑（时间参数化）、命令执行（ros2_control）等多个子系统。

MoveIt2 不是一个规划算法——它是一个**规划框架**，通过 pluginlib 插件架构统一了这些子系统。

### 如果不用 MoveIt2 会怎样 ⭐⭐

假设你要自己组装一个运动规划系统：
1. 用 Pinocchio 做 FK/IK → 需要自己管理 URDF 加载和关节限位
2. 用 FCL 做碰撞检测 → 需要自己维护碰撞对象列表和更新机制
3. 用 OMPL 做路径规划 → 需要自己定义状态空间、有效性检查器、采样器
4. 用 Ruckig 做时间参数化 → 需要自己连接路径输出和时间参数化输入
5. 用 ros2_control 执行 → 需要自己将轨迹转换为 FollowJointTrajectory action

每个步骤都需要大量胶水代码，而且更换任何一个组件都需要修改多处代码。

> **本质洞察**：MoveIt2 的核心价值不是任何一个算法，而是**接口标准化**——它定义了规划器、IK 求解器、碰撞检测器、时间参数化器的标准接口，让所有实现都可以通过 pluginlib 运行时替换。这类似于 JDBC 之于数据库、OpenGL 之于图形渲染——抽象层的价值在于解耦和可替换性。

### 架构分层 ⭐⭐

```
用户 API 层:
  MoveGroupInterface (ROS Action 封装，简单易用)
  MoveItCpp (进程内直调，跳过 ROS 通信，适合高频)
        │
        ▼
规划管线层:
  PlanningPipeline
  (可链式: OMPL → STOMP, 或并行: OMPL ∥ TrajOpt)
        │
        ▼
插件层 (全部通过 pluginlib 运行时加载):
  ├── 规划器:    OMPL / CHOMP / STOMP / Pilz / cuMotion
  ├── IK 求解器: KDL / TRAC-IK / IKFast / pick-ik / BioIK
  ├── 碰撞检测:  FCL / Bullet
  └── 时间参数化: TOTG / Ruckig / IPTP
        │
        ▼
执行层:
  ros2_control (JointTrajectoryController → 硬件驱动)
```

**每一层都通过接口解耦**，替换任一组件只需修改 YAML 配置文件：

```yaml
# 替换 IK 求解器: 只改一行配置
kinematics:
  manipulator:
    kinematics_solver: pick_ik/PickIkPlugin
    # 原来是 kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
```

### pluginlib 三层工厂架构 ⭐⭐

MoveIt2 里，规划器、IK 求解器、碰撞检测器**全都是 pluginlib 插件**。

回顾 `02_基础/30_软件工程/10_设计模式与高级惯用法` 和 M12：我们在 `02_基础/30_软件工程/10_设计模式与高级惯用法` 中学习了工厂模式和 pluginlib 动态加载，在 M12 中看到 ros2_control 如何用 pluginlib 加载硬件驱动。MoveIt2 把这个模式推到了极致——几乎所有可变组件都是插件。

```cpp
// MoveIt2 内部加载规划器的核心逻辑
// (moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp)
std::unique_ptr<pluginlib::ClassLoader<
    planning_interface::PlannerManager>>
  planner_plugin_loader;

planner_plugin_loader.reset(
  new pluginlib::ClassLoader<
    planning_interface::PlannerManager>(
    "moveit_core",
    "planning_interface::PlannerManager"));

// 根据 YAML 配置的字符串名动态加载
planner_instance.reset(
  planner_plugin_loader->createUnmanagedInstance(
    planner_name));
```

这是**工厂模式 + 动态链接 + 抽象接口**三大模式的完整组合：
- **抽象接口**：`PlannerManager` / `KinematicsBase` / `CollisionPlugin`
- **动态链接**：pluginlib 通过 `dlopen` 加载 .so 文件
- **工厂模式**：`ClassLoader::createUnmanagedInstance()` 根据字符串创建对象

> **反事实推理**：如果 MoveIt2 不用 pluginlib 而是硬编码所有实现会怎样？每次 OMPL 发布新的规划器（如 FCIT*），都需要修改 MoveIt2 核心代码、重新编译整个框架。有了 pluginlib，新规划器只需发布独立 ROS 包，用户修改一行 YAML 即可使用——MoveIt2 核心零修改。

> **跨领域类比**：MoveIt2 的插件架构类似于 Web 浏览器的扩展系统——Chrome 定义了扩展 API 接口，任何开发者都可以发布扩展，用户安装后即可使用，不需要重新编译浏览器。区别在于 MoveIt2 的插件运行在同一进程内（通过 dlopen），而浏览器扩展运行在沙箱中。

### ⚠️ 常见陷阱

```
💡 概念误区：认为 MoveIt2 只是"OMPL 的封装"
   新手想法："MoveIt2 就是用 OMPL 做运动规划的工具"
   实际上：OMPL 只是 MoveIt2 的规划器插件之一。MoveIt2 还包括：
          - PlanningScene（碰撞世界管理）
          - IK 求解器框架（5+ 种 IK 插件）
          - 轨迹处理管线（平滑、时间参数化）
          - 执行管理（ros2_control 集成）
          - MTC（多阶段任务规划）
          把 MoveIt2 等同于 OMPL 就像把 Linux 等同于 kernel。
```

```
🧠 思维陷阱：认为"pluginlib 灵活性没有代价"
   新手想法："插件化这么好，为什么不是所有系统都用 pluginlib？"
   实际上：pluginlib 的灵活性有代价：
          1. dlopen 延迟：首次加载插件需 10-100ms
          2. 虚函数调用：接口调用通过 vtable，比直接调用慢 2-5ns
          3. 配置复杂度：plugin.xml + YAML + CMake 多处需一致
          4. 调试困难：断点不容易设在动态加载的代码中
   与 Pinocchio CRTP 的对比：Pinocchio 用编译期多态消除虚函数开销，
          但代价是不能运行时替换。MoveIt2 选择灵活性，Pinocchio 选择性能。
```

```
⚠️ 编程陷阱：pluginlib 声明文件路径错误
   错误做法：plugin.xml 中 library path 与 CMake target 名不匹配
   现象：MoveIt2 启动时报 "Failed to load plugin"
   根本原因：pluginlib 通过 plugin.xml 中的 path 属性找 .so 文件
   正确做法：确保 plugin.xml 的 path 与 CMakeLists.txt 的 target 名一致
   自检方法：ros2 plugin list 列出已注册插件
```

### 练习

1. **[A 型]** 用 MoveIt Setup Assistant 为 Franka Panda 生成完整配置包。检查 `kinematics.yaml`、`ompl_planning.yaml`、`controllers.yaml`，标注每个配置项对应架构图中哪一层。

2. **[B 型]** 精读 `planning_pipeline.cpp` 中规划器加载的代码路径。画出调用链：YAML → pluginlib ClassLoader → dlopen → 工厂创建 → 虚函数调用。

3. **[思考题]** MoveIt2 的 pluginlib 架构让任何组件都可运行时替换。这种灵活性的代价是什么？与 Pinocchio CRTP 编译期分派对比。

---

## M14.2 MoveGroupInterface vs MoveItCpp ⭐⭐

### 动机 ⭐⭐

MoveIt2 提供两套用户 API，适合不同场景。理解两者的区别是正确使用 MoveIt2 的第一步。

### MoveGroupInterface（简单，有通信开销）

MoveGroupInterface 通过 ROS2 Action 与 move_group 节点通信：

```cpp
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

rclcpp::NodeOptions options;
options.automatically_declare_parameters_from_overrides(true);
auto node = rclcpp::Node::make_shared("my_node", options);

rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread spinner([&executor]() { executor.spin(); });

auto move_group =
  moveit::planning_interface::MoveGroupInterface(
    node, "manipulator");

if (move_group.getCurrentState(2.0)) {
  move_group.setStartStateToCurrentState();

  // 设置目标位姿
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.4;
  target_pose.orientation.w = 1.0;
  move_group.setPoseTarget(target_pose);

  // 规划
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
    (move_group.plan(plan) ==
      moveit::core::MoveItErrorCode::SUCCESS);

  // 执行
  if (success) {
    move_group.execute(plan);
  }
}
executor.cancel();
spinner.join();
```

示例把失败路径写成普通分支而不是直接 `throw`，是为了保证 executor 线程总能 `join()`。工程代码可以用 RAII guard 管理 spinner 生命周期后再抛异常。

**数据流**：
```
你的代码 → ROS2 Action Goal → move_group 节点
                                    │
                                    ├── 调用规划器
                                    ├── 碰撞检测
                                    ├── 轨迹平滑
                                    │
move_group 节点 → ROS2 Action Result → 你的代码
                                    │
                                    └── 通过 ros2_control 执行
```

### MoveItCpp（快速，进程内直调）

MoveItCpp 跳过了 ROS Action 通信，直接在进程内调用规划逻辑：

```cpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

auto node = rclcpp::Node::make_shared("my_node");
auto moveit_cpp =
  std::make_shared<moveit_cpp::MoveItCpp>(node);
auto planning_component =
  std::make_shared<moveit_cpp::PlanningComponent>(
    "manipulator", moveit_cpp);

// 设置目标
geometry_msgs::msg::PoseStamped target;
target.header.frame_id = "panda_link0";
target.pose.position.x = 0.5;
target.pose.position.z = 0.4;
target.pose.orientation.w = 1.0;
planning_component->setGoal(target, "panda_hand");  // 目标 link / tip frame

// 规划 + 执行
auto solution = planning_component->plan();
if (solution) {
  // execute() 的封装在 MoveIt2 版本间略有差异。常见用法是
  // 通过 MoveItCpp 执行 plan() 返回的 RobotTrajectory。
  moveit_cpp->execute("manipulator", solution.trajectory, true);
}
```

### 详细对比

| 维度 | MoveGroupInterface | MoveItCpp |
|------|-------------------|-----------|
| 通信方式 | ROS2 Action（跨进程） | 进程内直调 |
| 延迟 | 1-10ms 通信开销 | ~0 通信开销 |
| 部署 | 需要单独启动 move_group 节点 | 所有组件在同一进程内 |
| API 复杂度 | 简单（高层封装） | 中等（更多控制） |
| 适合场景 | 标准 pick-and-place、教学 | 高频重规划、visual servoing |
| 多语言 | 有 Python 绑定 | 仅 C++ |

> **跨领域类比**：MoveGroupInterface 类似于通过 REST API 调用微服务，MoveItCpp 类似于直接调用库函数。前者有网络开销但架构清晰，后者零开销但耦合更紧。

**选型建议**：
- **90% 的场景**用 MoveGroupInterface——简单、稳定、有 Python 支持
- **需要 >10 Hz 重规划**时用 MoveItCpp——如 visual servoing、实时避障

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MoveGroupInterface 规划前未设置 start state
   错误做法：直接 setPoseTarget + plan，不设置起始状态
   现象：规划从错误位置开始或失败
   根本原因：默认 start state 可能是过时的
   正确做法：
     move_group.setStartStateToCurrentState();
     move_group.setPoseTarget(target_pose);
     move_group.plan(my_plan);
```

```
💡 概念误区：认为 MoveItCpp 一定比 MoveGroupInterface 快
   新手想法："进程内调用一定更快"
   实际上：规划本身耗时（OMPL 搜索 100ms-2s）远大于通信开销（1-10ms）。
          MoveItCpp 的优势只有在高频重规划（如 visual servoing 每 100ms
          重规划）时才显著。单次规划时两者差距可忽略。
```

### MoveGroupInterface PIMPL 封装

MoveGroupInterface 内部使用了 PIMPL（Pointer to Implementation）惯用法——所有实现细节隐藏在 `MoveGroupInterfaceImpl` 中：

```cpp
// move_group_interface.h (公开头文件)
class MoveGroupInterface {
public:
  MoveGroupInterface(const rclcpp::Node::SharedPtr& node,
                     const std::string& group_name);

  bool setPoseTarget(const geometry_msgs::msg::Pose& pose);
  MoveItErrorCode plan(Plan& plan);
  MoveItErrorCode execute(const Plan& plan);
  // ... 其他公开 API ...

private:
  class MoveGroupInterfaceImpl;  // 前向声明
  std::unique_ptr<MoveGroupInterfaceImpl> impl_;  // PIMPL
};
```

PIMPL 的工程价值（回顾 `02_基础/30_软件工程/10_设计模式与高级惯用法`）：
1. **ABI 稳定**：修改内部实现不需要重新编译用户代码
2. **编译隔离**：用户头文件不包含内部依赖（如 OMPL、FCL 的头文件）
3. **封装性**：内部状态（ROS2 Action Client、规划器实例等）完全隐藏

> **反事实推理**：如果 MoveGroupInterface 不用 PIMPL 而是直接暴露所有成员会怎样？每次 MoveIt2 内部重构（如更换 Action 接口版本），所有用户代码都需要重新编译——这在大型工程中意味着数小时的编译时间和潜在的兼容性问题。

### 使用 Python API

MoveIt2 的 Python API 通过 `moveit_py` 包提供，基于 pybind11 绑定：

```python
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import numpy as np

# 初始化
moveit = MoveItPy(node_name="my_moveit_py")
panda = moveit.get_planning_component("panda_arm")

# 设置目标
panda.set_goal_state(
    pose_stamped_msg=target_pose,
    pose_link="panda_link8"
)

# 规划
plan_result = panda.plan()

# 执行
if plan_result:
    robot_trajectory = plan_result.trajectory
    moveit.execute(robot_trajectory, controllers=[])
```

**Python vs C++ 的选型**：

| 维度 | Python API | C++ API |
|------|-----------|---------|
| 开发速度 | 快（无编译） | 慢（编译+链接） |
| 运行性能 | 规划本身 C++ 执行，Python 开销仅在 API 调用 | 最优 |
| 适用场景 | 原型开发、研究、80% 的实际部署 | 高频重规划、嵌入式 |
| 调试 | Python debugger 更方便 | gdb/lldb |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Python API 中忘记处理 plan() 返回 None
   错误做法：直接使用 plan_result.trajectory 不检查 None
   现象：AttributeError: 'NoneType' has no attribute 'trajectory'
   正确做法：
     plan_result = panda.plan()
     if plan_result:
       moveit.execute(plan_result.trajectory)
     else:
       print("Planning failed!")
```

### 练习

1. **[A 型]** 分别用 MoveGroupInterface 和 MoveItCpp 实现同一 pose target 规划。对比规划耗时（包含通信开销）。

2. **[A 型]** 用 Python API 实现相同的规划任务。对比开发时间和代码行数。

3. **[思考题]** MoveIt2 的 Python API 近年占新部署的 80%。为什么 Python 在实际部署中更受欢迎？

4. **[B 型]** 精读 `move_group_interface.h` 和 `move_group_interface.cpp`。找到 `Impl` 类，列出 PIMPL 隐藏了哪些内部细节。

---

## M14.3 PlanningScene 管理 ⭐⭐

### 动机 ⭐⭐

运动规划不是在真空中进行的——机械臂必须避开桌子、物体、人和自身。PlanningScene 是 MoveIt2 中管理「机器人周围有什么」的核心组件。

### PlanningScene 的内容

PlanningScene 包含三类信息：
1. **机器人状态**（RobotState）：当前关节角度、连杆位姿
2. **环境物体**（CollisionObject）：桌子、障碍物、待抓取物体
3. **允许碰撞矩阵**（ACM）：哪些连杆对不需要碰撞检查

### 碰撞对象管理

```cpp
// Rolling/Kilted 优先使用 .hpp；Humble 仍可用 .h。
#if __has_include(<moveit/planning_scene_interface/planning_scene_interface.hpp>)
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#else
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#endif

moveit::planning_interface::PlanningSceneInterface psi;

// 添加桌面碰撞对象
moveit_msgs::msg::CollisionObject table;
table.id = "table";
table.header.frame_id = "world";
table.operation = moveit_msgs::msg::CollisionObject::ADD;

shape_msgs::msg::SolidPrimitive box;
box.type = shape_msgs::msg::SolidPrimitive::BOX;
box.dimensions = {1.0, 1.0, 0.02};  // 1m x 1m x 2cm

geometry_msgs::msg::Pose table_pose;
table_pose.position.x = 0.5;
table_pose.position.z = -0.01;
table_pose.orientation.w = 1.0;

table.primitives.push_back(box);
table.primitive_poses.push_back(table_pose);

if (!psi.applyCollisionObject(table)) {
  throw std::runtime_error("failed to apply table collision object");
}
```

### Attached Objects（附着对象）

当机器人抓取物体后，物体从环境中「附着」到末端——碰撞检测需要知道这个变化：

```cpp
// 抓取后：将物体附着到末端
moveit_msgs::msg::AttachedCollisionObject attached;
attached.link_name = "panda_hand";
attached.touch_links = {
  "panda_hand", "panda_leftfinger", "panda_rightfinger"
};
attached.object.id = "target_object";
attached.object.header.frame_id = "panda_hand";
attached.object.operation =
  moveit_msgs::msg::CollisionObject::ADD;

shape_msgs::msg::SolidPrimitive object_shape;
object_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
object_shape.dimensions = {0.04, 0.04, 0.12};

geometry_msgs::msg::Pose object_pose;
object_pose.orientation.w = 1.0;
object_pose.position.z = 0.08;  // 相对 panda_hand 的抓取位姿

attached.object.primitives.push_back(object_shape);
attached.object.primitive_poses.push_back(object_pose);

// 附着后，碰撞检测自动：
// 1. 不检查 target_object 与 touch_links 中夹爪 link 的碰撞
// 2. 检查 target_object 与环境的碰撞（搬运时不撞桌子）
// 3. 检查 target_object 与机器人其他部分的碰撞

psi.applyAttachedCollisionObject(attached);
```

> **反事实推理**：如果抓取后不 attach 物体会怎样？PlanningScene 仍然认为物体在桌上，机器人搬运时的规划会试图避开物体原来的位置（因为那里「还有」障碍物），但不会避开物体当前跟着末端移动的位置——可能导致物体撞到障碍物。

### diff-based 同步机制

PlanningScene 在多节点系统中使用 diff-based 同步——只传输变化部分：

```
move_group 节点（维护完整 PlanningScene）
        │
        │ /planning_scene topic (PlanningSceneMsg)
        │ 只包含 diff（变化的物体）
        ▼
你的节点（PlanningSceneMonitor）
  └── 应用 diff 重建完整场景
```

> **跨领域类比**：diff-based 同步类似于 Git 的 diff/patch——只传输变化部分，接收端通过应用变化重建完整状态。也类似于 React Virtual DOM diff——只更新变化的节点。

### ACM（Allowed Collision Matrix）

ACM 定义了哪些碰撞对不需检查——对性能至关重要：

```
7-DOF 臂有 ~10 个碰撞体
不加 ACM: 需要检查 C(10,2) = 45 对碰撞
加 ACM 后: 相邻连杆（必然接触）不检查，只检查 ~20 对
→ 碰撞检测时间减少 ~55%
```

MoveIt Setup Assistant 自动生成 ACM（通过在多个随机配置下采样碰撞），保存在 SRDF 文件中。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：添加碰撞对象后立即规划
   错误做法：addCollisionObjects({table}); plan();
   现象：规划结果偶尔穿过桌子
   根本原因：addCollisionObjects 走 topic 发布，调用返回不等于所有
            PlanningSceneMonitor 都已经处理了 diff
   正确做法：
     if (!psi.applyCollisionObject(table)) { fail_fast(); }
     // applyCollisionObject 通过服务把 diff 应用到 move_group，语义上
     // 比 addCollisionObjects 更适合“更新后立刻规划”的路径。
     // 如果你的节点还维护本地 PlanningSceneMonitor，再等待本地场景
     // 确认看到 table；短 sleep 只能做调试缓冲，不是同步保证。
     move_group.plan(plan);
```

```
💡 概念误区：认为 PlanningScene 只包含静态障碍物
   新手想法："PlanningScene 就是一堆固定的碰撞盒子"
   实际上：PlanningScene 是动态的——运行时可添加/删除/移动碰撞对象：
          1. 相机检测到新物体 → 添加碰撞对象
          2. 机器人抓取物体 → 从环境转为 attached
          3. 物体放下 → 从 attached 转为环境
          4. 障碍物被移走 → 删除碰撞对象
```

```
🧠 思维陷阱：认为 ACM 越多越好
   新手想法："disable 更多碰撞对可以加快碰撞检测"
   实际上：ACM 中跳过不该跳过的碰撞对比检查多余的碰撞对危险得多。
          多余的碰撞检查只浪费 CPU 时间（微秒级），
          缺失的碰撞检查可能导致物理碰撞（破坏设备）。
          Setup Assistant 自动生成的 ACM 已经足够——不要手动修改。
```

### 练习

1. **[A 型]** 动态添加、移动、删除碰撞对象。用 RViz PlanningScene 显示观察变化。

2. **[A 型]** 实现完整的 attach/detach 流程：添加桌面和物体 → 规划到物体附近 → attach → 规划搬运 → detach 到目标位置。

3. **[思考题]** ACM 中如果漏了一对不应检查的碰撞对，会怎样？多了一对不应跳过的碰撞对，又会怎样？哪种错误更危险？

---

## M14.4 MoveIt Task Constructor (MTC) 深度 ⭐⭐

### 动机 ⭐⭐

单次运动规划（从 A 到 B）用 MoveGroupInterface 就够了。但 pick-and-place 任务是**多阶段**的：打开夹爪 → 移动到预抓取位 → 接近物体 → 关闭夹爪 → 提起物体 → 移动到放置位 → 放下物体 → 后退。

这 8 个阶段不是独立的——后一个阶段的起始状态是前一个阶段的结束状态，中间还有碰撞场景的变化（物体从桌上 attach 到末端）。MTC 专门解决这种**多阶段运动规划的约束传播和多解搜索**。

### 核心概念 ⭐⭐

**Stage（阶段）**是 MTC 的基本单元。每个 Stage 按功能分为三类：

| Stage 类型 | 功能 | 数据流 | 典型 Stage |
|-----------|------|--------|-----------|
| **Generator** | 生成新的机器人状态 | 无输入，输出状态 | CurrentState, GenerateGraspPose |
| **Propagator** | 从已知状态推导新状态 | 前向或后向传播 | MoveTo, MoveRelative |
| **Connector** | 连接两个已知状态 | 双向输入 | Connect (自由空间规划) |

**InterfaceState** 在 Stage 之间传递，不仅包含机器人关节状态，还包含 PlanningScene 快照（物体附着信息等）。

```
CurrentState → OpenGripper → Connect → ComputeIK
     │              │           │          │
     ▼              ▼           ▼          ▼
  [state0]  →  [state1]  → [state2]  → [state3]
  (当前)      (夹爪开)   (预抓取位)  (抓取IK解)
                                          │
     ← Approach ← CloseGripper ← Attach  │
          │            │           │      ▼
          ▼            ▼           ▼   [state4]
       [state5]     [state6]    [state7]
       (接近后)     (夹爪闭)    (已attach)
```

### Composite Pattern 的应用

MTC 的 Stage 体系直接应用了 Composite Pattern（回顾 `02_基础/30_软件工程/10_设计模式与高级惯用法`）：

```
Stage (基类 = Component)
├── Wrapper (Decorator) — 包装单个子 Stage
├── SerialContainer (Composite) — 顺序执行多个子 Stage
├── Alternatives (Composite) — 尝试多种方案取最优
├── Merger (Composite) — 合并多个子 Stage 的解
└── 具体 Stage (Leaf):
    ├── CurrentState (Generator)
    ├── MoveTo (Propagator)
    ├── MoveRelative (Propagator)
    ├── ModifyPlanningScene (Generator)
    ├── GenerateGraspPose (Generator)
    ├── ComputeIK (Wrapper)
    └── Connect (Connector)
```

SerialContainer 和单个 Stage 共享相同接口——可以任意深度嵌套。这与 BT.CPP 中 Sequence 嵌套 SubTree 的思想完全一致。

### 完整 Pick-and-Place 示例

```cpp
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

namespace mtc = moveit::task_constructor;

auto task = std::make_unique<mtc::Task>();
task->stages()->setName("pick_place");
task->loadRobotModel(node);  // node 为当前 ROS2 节点；也可显式 setRobotModel(...)

// ---- 规划器配置 ----
auto pipeline =
  std::make_shared<mtc::solvers::PipelinePlanner>(
    node, "ompl");
pipeline->setPlannerId("RRTConnectkConfigDefault");
auto cartesian =
  std::make_shared<mtc::solvers::CartesianPath>();
cartesian->setMaxVelocityScaling(0.1);
auto joint_interp =
  std::make_shared<mtc::solvers::JointInterpolationPlanner>();

// ---- Stage 1: 当前状态 ----
auto current_state =
  std::make_unique<mtc::stages::CurrentState>("current");
auto current_state_ptr = current_state.get();
task->add(std::move(current_state));

// ---- Stage 2: 打开夹爪 ----
auto open_gripper =
  std::make_unique<mtc::stages::MoveTo>(
    "open_gripper", joint_interp);
open_gripper->setGroup("hand");
open_gripper->setGoal("open");
task->add(std::move(open_gripper));

// ---- Stage 3: 移动到预抓取位 (自由空间) ----
auto move_to_pregrasp =
  std::make_unique<mtc::stages::Connect>(
    "move_to_pregrasp",
    mtc::stages::Connect::GroupPlannerVector{
      {"manipulator", pipeline}});
task->add(std::move(move_to_pregrasp));

// ---- Stage 4: 生成抓取位姿 (IK 采样) ----
auto grasp_gen =
  std::make_unique<mtc::stages::GenerateGraspPose>(
    "generate_grasp");
grasp_gen->setObject("target_object");
grasp_gen->setAngleDelta(M_PI / 12);   // 每 15 度采样
grasp_gen->setPreGraspPose("open");
grasp_gen->setMonitoredStage(current_state_ptr);

// ---- Stage 5: 包装 IK 求解 ----
Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
grasp_frame_transform.translation().z() = 0.10;  // TCP 到手爪参考点偏置
auto compute_ik =
  std::make_unique<mtc::stages::ComputeIK>(
    "compute_ik", std::move(grasp_gen));
compute_ik->setGroup("manipulator");
compute_ik->setIKFrame(grasp_frame_transform, "panda_hand");
compute_ik->setMaxIKSolutions(8);

// ---- Stage 6: 接近物体 (笛卡尔直线) ----
auto approach =
  std::make_unique<mtc::stages::MoveRelative>(
    "approach", cartesian);
approach->setGroup("manipulator");
approach->setMinMaxDistance(0.05, 0.15);
geometry_msgs::msg::Vector3Stamped dir;
dir.header.frame_id = "world";
dir.vector.z = -1.0;    // 沿 Z 轴向下
approach->setDirection(dir);

// ---- Stage 7: 允许夹爪与目标物体接触 ----
// 关闭夹爪前需要临时允许 hand links 与 object 接触，否则规划场景
// 会把正常抓取接触判为碰撞。
std::vector<std::string> hand_links = {
  "panda_hand", "panda_leftfinger", "panda_rightfinger"};
auto allow_touch =
  std::make_unique<mtc::stages::ModifyPlanningScene>(
    "allow_touch");
allow_touch->allowCollisions("target_object", hand_links, true);

// ---- Stage 8: 关闭夹爪 ----
auto close_gripper =
  std::make_unique<mtc::stages::MoveTo>(
    "close_gripper", joint_interp);
close_gripper->setGroup("hand");
close_gripper->setGoal("close");

// ---- Stage 9: 附着物体 ----
auto attach =
  std::make_unique<mtc::stages::ModifyPlanningScene>(
    "attach");
attach->attachObject("target_object", "panda_hand");

// ---- Stage 10: 提起物体 (笛卡尔直线) ----
auto lift =
  std::make_unique<mtc::stages::MoveRelative>(
    "lift", cartesian);
lift->setGroup("manipulator");
lift->setMinMaxDistance(0.05, 0.2);
dir.vector.z = 1.0;     // 向上
lift->setDirection(dir);

// ---- 组合抓取序列 ----
// ComputeIK 是 WrapperStage，只包裹单个 GenerateGraspPose。
// approach/close/attach/lift 应与 compute_ik 作为同级子 Stage
// 放入外层 SerialContainer，不能把 SerialContainer 再塞进 ComputeIK。
auto grasp_container =
  std::make_unique<mtc::SerialContainer>("grasp");
grasp_container->insert(std::move(approach));
grasp_container->insert(std::move(compute_ik));
grasp_container->insert(std::move(allow_touch));
grasp_container->insert(std::move(close_gripper));
grasp_container->insert(std::move(attach));
grasp_container->insert(std::move(lift));

task->add(std::move(grasp_container));

// ---- Stage 10: 移动到放置位 ----
auto move_to_place =
  std::make_unique<mtc::stages::Connect>(
    "move_to_place",
    mtc::stages::Connect::GroupPlannerVector{
      {"manipulator", pipeline}});
task->add(std::move(move_to_place));

// ---- Stage 11-14: 放置 (类似 grasp 的结构) ----
// ... approach → open → detach → retreat ...

// ---- 规划 ----
task->plan(10);   // 搜索 10 个方案

// ---- 执行最优方案 ----
if (task->solutions().empty()) {
  throw std::runtime_error("MTC planning failed: no solution");
}
auto result = task->execute(*task->solutions().front());
if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
  throw std::runtime_error("MTC execution failed");
}
```

### 自定义 MTC Stage 实现 ⭐⭐⭐

MTC 的 Stage 系统可以通过继承具体语义的基类实现扩展。下面是一个工业场景常用的教学示例——「视觉扫描」Generator Stage：它主动生成一个“已扫描”的输出状态，并在该状态中加入检测到的物体。

> 注意：MTC 的内部 Stage 接口会随 MoveIt2 版本演进。下面代码展示符合 Generator/Wrapper 数据流语义的结构，属于教学骨架；函数签名、`SubTrajectory` 构造、属性声明等细节应以目标 MoveIt2 版本的官方示例和头文件为准。

```cpp
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>

namespace mtc = moveit::task_constructor;

// 自定义 Generator：不消费上游解，主动生成一个 InterfaceState。
// 若需要“先移动到扫描位再扫描”，更常见做法是用 MoveTo/MoveRelative
// 负责运动规划，再在后续 Generator/Wrapper 中更新场景。
class ScanTable : public mtc::Generator {
public:
    ScanTable(const std::string& name,
              const std::string& group)
        : Generator(name), group_(group)
    {
        properties().declare<std::string>("object_id",
            "detected_object", "detected object id");
    }

    void init(const moveit::core::RobotModelConstPtr& model)
        override
    {
        Generator::init(model);
        robot_model_ = model;
        scan_pose_ = {0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8};
    }

    bool canCompute() const override { return !done_; }

    void compute() override
    {
        auto scene = std::make_shared<planning_scene::PlanningScene>(
            robot_model_);

        auto& state = scene->getCurrentStateNonConst();
        const auto* jmg =
            state.getJointModelGroup(group_);
        state.setJointGroupPositions(jmg, scan_pose_);

        moveit_msgs::msg::CollisionObject obj;
        obj.id = properties().get<std::string>("object_id");
        obj.header.frame_id = "world";
        // ... 设置形状和位姿 ...
        scene->processCollisionObjectMsg(obj);

        mtc::InterfaceState output(scene);

        // SubTrajectory 可为空，表示该 Stage 只生成/更新状态；
        // 如果 Stage 同时负责运动，需要填入 RobotTrajectory。
        mtc::SubTrajectory result;
        result.setCost(0.0);
        spawn(std::move(output), std::move(result));
        done_ = true;
    }

private:
    std::string group_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::vector<double> scan_pose_;
    bool done_{false};
};
```

如果把该 Stage 做成 pluginlib 插件，就可以在 MTC Task 中像内置 Stage 一样使用；教学代码中也可以直接链接并构造对象：

```cpp
auto scan_stage = std::make_unique<ScanTable>(
    "scan_table", "manipulator");
task->add(std::move(scan_stage));
```

### MTC Alternatives 与 Merger 高级组合

**Alternatives** 尝试多种方案，保留所有可行解供后续评分选优：

```cpp
// Alternatives：同时尝试三种抓取策略
auto alternatives =
    std::make_unique<mtc::Alternatives>("grasp_strategies");

// 策略 A：从顶部抓取
{
    auto grasp_gen =
        std::make_unique<mtc::stages::GenerateGraspPose>(
            "top_grasp");
    grasp_gen->setObject("target");
    grasp_gen->setAngleDelta(M_PI / 6);  // 30 度采样

    auto ik = std::make_unique<mtc::stages::ComputeIK>(
        "top_ik", std::move(grasp_gen));
    ik->setGroup("manipulator");
    ik->setMaxIKSolutions(4);

    auto container =
        std::make_unique<mtc::SerialContainer>("top_approach");
    container->insert(std::move(ik));
    // ... 添加 approach, close_gripper 等 ...
    alternatives->insert(std::move(container));
}

// 策略 B：从侧面抓取
{
    auto side_gen =
        std::make_unique<mtc::stages::GenerateGraspPose>(
            "side_grasp");
    side_gen->setObject("target");
    side_gen->setAngleDelta(M_PI / 4);
    // ... 设置侧面 approach 方向 ...
    auto container =
        std::make_unique<mtc::SerialContainer>("side_approach");
    // ... 添加 stages ...
    alternatives->insert(std::move(container));
}

task->add(std::move(alternatives));
// MTC 会搜索所有策略的所有可行解，按总 cost 排序
```

**Merger** 合并多个 Stage 的解（用于双臂同步规划）：

```cpp
auto merger = std::make_unique<mtc::Merger>("dual_arm_grasp");

// 左臂抓取
auto left_grasp = std::make_unique<mtc::SerialContainer>(
    "left_arm_grasp");
// ... 左臂 stages ...
merger->insert(std::move(left_grasp));

// 右臂抓取
auto right_grasp = std::make_unique<mtc::SerialContainer>(
    "right_arm_grasp");
// ... 右臂 stages ...
merger->insert(std::move(right_grasp));

// Merger 确保两臂的解在同一 PlanningScene 中无碰撞
task->add(std::move(merger));
```

### MTC vs BT.CPP 的关系

| 维度 | MTC | BT.CPP |
|------|-----|--------|
| 层次 | **规划层**编排 | **执行层**编排 |
| 输出 | 运动轨迹 | 触发 Action/Service |
| 多解搜索 | 搜索多个方案取最优 | 按顺序尝试 |
| 错误处理 | 换规划器/换参数/换 IK 解 | Retry/Fallback/Recovery |
| 约束传播 | InterfaceState 自动传播 | Blackboard 手动传递 |
| 时间尺度 | 毫秒到秒（规划时间） | 秒到分钟（任务执行） |

**不是 X 而是 Y**：MTC 和 BT.CPP 不是替代关系，而是互补关系。BT 负责「什么时候做什么」（任务级决策），MTC 负责「怎么做」（运动级规划）。把 MTC 的工作放到 BT 中做是可行但不推荐的——因为 MTC 的 Stage 抽象专门为运动规划设计，比 BT 的通用节点更适合处理约束传播和多解搜索。

**典型组合**：
```
BT.CPP (执行层)
├── Condition: IsObjectDetected
├── Action: CallMTCPickAndPlace ──► MTC (规划层)
│                                    ├── CurrentState
│                                    ├── OpenGripper
│                                    ├── Approach + Grasp
│                                    ├── Lift + MoveToPlace
│                                    └── Place + Retreat
├── Condition: VerifyGrasp
└── Action: ReturnHome
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MTC 中忘记 attach/detach 物体
   错误做法：关闭夹爪后直接规划搬运，不 attach
   现象：搬运规划会避开物体原来的位置（PlanningScene 中物体还在桌上）
   根本原因：不 attach 的话 PlanningScene 认为物体是环境障碍物
   正确做法：close_gripper 后立即 ModifyPlanningScene attach
```

```
💡 概念误区：认为 MTC 只能做 pick-and-place
   新手想法："MTC 就是做抓取放置的工具"
   实际上：MTC 的 Stage 抽象足够通用，可以编排任何多阶段运动：
          装配、焊接、打磨、工具切换、多臂协调...
```

```
⚠️ 编程陷阱：approach 方向设错导致 MTC 无解
   错误做法：approach 方向设为 world 坐标系的 +Z（向上）
   现象：MTC plan() 返回 0 个方案
   根本原因：如果物体在桌面上，approach 应该是 -Z（向下）才能接近物体
   正确做法：根据物体位置和抓取策略设置合理的 approach 方向
   调试方法：在 RViz 中可视化 MTC 的 Stage 结果，逐 Stage 检查
```

### 练习

1. **[A 型]** 用 MTC 实现 Franka Panda 的桌面 pick-and-place：物体用 CollisionObject 表示，在 Gazebo 中执行。

2. **[A 型]** 为 MTC 写一个自定义 Stage——「ScanTable」（移动到扫描位姿，模拟点云采集），注册为 pluginlib 插件。

3. **[B 型]** 精读 `stage.h` 的 Stage 基类和 `serial_container.h`。画出 Composite Pattern 三要素（Component/Leaf/Composite）对应关系。

---

## M14.5 规划管线配置 ⭐⭐⭐

### 动机

MoveIt2 2024 年重构后支持**链式规划**和**并行规划**——不再局限于单一规划器。OMPL、RRT-Connect、BIT* 的算法原理见 M07，本节只讨论它们在 MoveIt2 管线中的工程配置位置。

### 链式规划

```
OMPL (采样规划) → STOMP (优化平滑)
│                    │
├── 快速找到可行路径  ├── 以 OMPL 结果为初始解
│   (可能不够平滑)   │   优化平滑度和避障间距
│                    │
▼                    ▼
粗糙但可行的路径  →  平滑且高质量的轨迹
```

链式规划取长补短——OMPL 擅长快速搜索，STOMP 擅长优化平滑。

### 并行规划

```
├── OMPL RRT-Connect  ──┐
├── OMPL BIT*          ──┤─→ 选择最优解
├── Pilz PTP            ──┘
```

不同规划器在不同场景下各有优势，并行运行取最优。

### 配置方式（概念示意）

Kilted 及更新版本的 MoveIt2 文档开始介绍多 planning pipeline、并行规划，以及部分场景下的 planner chaining 思路。但具体字段名、插件名和“链式”组织方式与 MoveIt2 发行版、安装的 planner 插件有关。下面 YAML 只表达配置意图，精确字段请以对应版本的官方教程和示例配置为准；不要假设把多个 `planning_plugins` 写在同一个列表里就会在所有版本自动串联执行。

```yaml
planning_pipelines:
  pipeline_names:
    - ompl
    - stomp
    - ompl_stomp_chain
    - pilz

ompl:
  planning_plugins:
    - ompl_interface/OMPLPlanner
  request_adapters:
    - default_planning_request_adapters/ResolveConstraintFrames
    - default_planning_request_adapters/ValidateWorkspaceBounds
    - default_planning_request_adapters/CheckStartStateBounds
    - default_planning_request_adapters/CheckStartStateCollision

stomp:
  planning_plugins:
    - stomp_moveit/StompPlanner

# 概念示意: OMPL → STOMP
# 精确字段以目标 MoveIt2 版本的官方 planner chaining 示例为准。
ompl_stomp_chain:
  planning_plugins:
    - ompl_interface/OMPLPlanner
    - stomp_moveit/StompPlanner

pilz:
  planning_plugins:
    - pilz_industrial_motion_planner/CommandPlanner
```

### 规划器选型指南

| 规划器 | 适用场景 | 优势 | 劣势 |
|--------|---------|------|------|
| OMPL RRT-Connect | 通用场景 | 快速、概率完备 | 路径不平滑 |
| OMPL BIT* | 高质量路径 | 渐近最优 | 较慢 |
| STOMP | 平滑路径 | 路径质量高 | 需好的初始解 |
| CHOMP | 避障间距要求高 | 考虑梯度信息 | 可能局部最优 |
| Pilz PTP/LIN/CIRC | 工业确定性运动 | 确定性、可预测 | 不避障 |
| cuMotion | 实时重规划 | GPU 加速 60x | 需要 NVIDIA GPU |

```
你需要什么样的规划？
    │
    ├── 通用避障路径？
    │   └── OMPL RRT-Connect (默认首选)
    │
    ├── 高质量平滑路径？
    │   └── OMPL → STOMP 链式
    │
    ├── 确定性直线/圆弧运动？
    │   └── Pilz PTP/LIN/CIRC
    │
    ├── 实时重规划 (>10 Hz)？
    │   ├── 有 GPU？→ cuMotion
    │   └── 无 GPU？→ MoveIt Servo
    │
    └── 最优路径（不急）？
        └── OMPL BIT* / Informed-RRT*
```

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为"最新/最快的规划器一定最好"
   新手想法："cuMotion 比 OMPL 快 60x，应该总是用 cuMotion"
   实际上：cuMotion 需要 GPU，且对碰撞场景有特定要求。
          对于大多数桌面 pick-and-place，OMPL RRT-Connect 100ms
          内就能给出可行路径——对人类操作员来说是瞬间的。
          选择规划器应基于场景需求，不是性能排行。
```

### Request Adapters（请求适配器）

规划管线中不仅有规划器，还有**请求适配器**——它们在规划前后对请求和结果进行预处理/后处理：

```yaml
ompl:
  planning_plugins:
    - ompl_interface/OMPLPlanner
  request_adapters:
    # 预处理（规划前）
    - default_planning_request_adapters/ResolveConstraintFrames
    - default_planning_request_adapters/ValidateWorkspaceBounds
    - default_planning_request_adapters/CheckStartStateBounds
    - default_planning_request_adapters/CheckStartStateCollision
  response_adapters:
    # 后处理（规划后）
    - default_planning_response_adapters/AddTimeOptimalParameterization
    - default_planning_response_adapters/ValidateSolution
    - default_planning_response_adapters/DisplayMotionPath
```

**预处理适配器**确保规划请求是有效的（起始状态不在碰撞中、不超出关节限制等）。**后处理适配器**对规划结果进行时间参数化、验证和可视化。

这种管线设计类似于 Web 框架的 middleware——每个适配器负责一个小功能，通过链式组合实现完整的处理流程。

### 时间参数化对比

OMPL 输出的路径只有路径点的关节位置，没有时间信息。时间参数化（Time Parameterization）为路径点分配时间戳，使轨迹满足速度、加速度、jerk 约束。

| 参数化器 | 算法 | 输出约束 | 适用场景 |
|---------|------|---------|---------|
| **TOTG** | Time-Optimal Time Parameterization | 速度+加速度 | 默认，够用 |
| **Ruckig** | Jerk-limited, 任意目标状态 | 速度+加速度+jerk | 平滑性要求高 |
| **IPTP** | Iterative Parabolic Time Parameterization | 速度+加速度 | 旧版默认 |

Ruckig 相比 TOTG 的关键优势是**jerk 限制**——这意味着加速度的变化是连续的，不会出现突变，对真实硬件更友好（减少机械冲击和振动）。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：时间参数化失败导致轨迹无法执行
   错误做法：忽略时间参数化的 response adapter 输出
   现象：JTC 拒绝执行轨迹，报 "invalid trajectory"
   根本原因：路径点之间距离太大，在给定约束下无法参数化
   正确做法：
     1. 降低 max_velocity_scaling_factor (如 0.5 → 0.3)
     2. 在路径中插入更多中间点
     3. 检查 joint_limits.yaml 中的限制是否过严
```

### 练习

1. **[A 型]** 配置 OMPL → STOMP 链式管线。对比纯 OMPL 和链式的路径质量（用关节加加速度 RMS 衡量平滑度）。

2. **[A 型]** 配置 RRT-Connect、BIT*、PRM* 三种规划器，同一目标分别规划，对比路径长度和规划时间。

3. **[A 型]** 分别用 TOTG 和 Ruckig 对同一路径做时间参数化。对比轨迹时长和关节 jerk 特性。

4. **[思考题]** 并行规划消耗更多 CPU。在 ARM 平台（如 Jetson）上是否合适？有什么替代策略？

---

## M14.6 MoveIt Servo 实时控制 ⭐⭐⭐

### 动机

标准的规划-执行流程有 100ms-2s 规划延迟——对**遥操作**和**视觉伺服**太慢了。MoveIt Servo 接收笛卡尔/关节速度命令，实时计算 IK 并发送给 ros2_control——延迟 <10ms。

### 工作原理

```
输入:
  TwistStamped (笛卡尔速度) 或 JointJog (关节速度)
        │
        ▼
MoveIt Servo:
  1. 计算雅可比 J
  2. J^+ 将笛卡尔速度转为关节速度
  3. 碰撞检测（每 N 步检查一次）
  4. 关节限制裁剪
  5. 积分得到关节位置增量
        │
        ▼
输出: ros2_control command interface
```

### 配置（版本敏感示意）

MoveIt Servo 的参数名随 MoveIt2 发行版和 `moveit_servo` 重构变化较明显。下面 YAML 只表达调参维度；部署时应从目标版本安装包里的 `servo_parameters.yaml` 或官方教程复制字段名，再填入本节解释的数值含义。

```yaml
servo:
  ros__parameters:
    publish_period: 0.01          # 100 Hz
    ee_frame_name: "panda_hand"
    planning_frame: "panda_link0"
    move_group_name: "manipulator"

    # 速度缩放/限制：字段名可能是 scale.linear/rotational，
    # 也可能由上层输入设备节点限幅，按目标版本配置。
    scale:
      linear: 0.5
      rotational: 1.0
    joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    check_collisions: true
    collision_check_rate: 10.0    # Hz

    # 奇异性阈值/停止策略的字段名也随版本变化；
    # 核心语义是接近奇异性时减速，超过硬阈值时停止输出。
    hard_stop_singularity_threshold: 30.0
    lower_singularity_threshold: 17.0
```

### 典型应用

1. **遥操作**：SpaceMouse 输入笛卡尔速度，Servo 实时控制
2. **Visual Servoing**：视觉反馈计算误差速度，Servo 实时跟踪
3. **力控协作**：力传感器输入转换为速度命令

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Servo 在奇异性附近失控
   错误做法：不设奇异性管理参数
   现象：关节速度暴增，机器人疯转
   根本原因：雅可比在奇异性附近条件数极大
   正确做法：按目标版本配置奇异性减速阈值和硬停止阈值
```

```
💡 概念误区：认为 Servo 的碰撞检测和规划器一样可靠
   新手想法："Servo 有碰撞检测，所以不会撞东西"
   实际上：Servo 碰撞检测频率（10 Hz）远低于命令频率（100 Hz）。
          两次检测之间有 10 步运动未被检查。
          Servo 的碰撞检测是「尽力而为」的安全网，
          不是规划器级别的保证。操作员需要保持视觉监控。
```

### Servo 参数调优详解 ⭐⭐⭐

Servo 的性能和安全性高度依赖参数配置。以下是关键参数族的深度解析；字段名仍以目标版本 `servo_parameters.yaml` 为准，不要逐字复制这段示意配置。

```yaml
servo:
  ros__parameters:
    # === 频率与延迟参数 ===
    publish_period: 0.01          # 命令发布周期 (s)
    # 0.01 = 100Hz: 标准遥操作
    # 0.005 = 200Hz: 高精度视觉伺服
    # 0.02 = 50Hz: 低性能平台 (如 Jetson Nano)

    # === 速度缩放/限制 ===
    scale:
      linear: 0.5                 # 笛卡尔线速度缩放/上限语义
      rotational: 1.0             # 笛卡尔角速度缩放/上限语义
    # 工业安全推荐: 首次部署设 0.1/0.3，逐步放宽

    # === 关节限位安全裕度 ===
    joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    # 单位: rad。距离关节限位 margin 内时减速停止
    # 0.1 rad ≈ 5.7°: 标准
    # 0.2 rad: 更保守（初次部署推荐）

    # === 碰撞检测 ===
    check_collisions: true        # 必须开启
    collision_check_rate: 10.0    # 碰撞检测频率 (Hz)
    # 命令 100Hz / 检测 10Hz = 每 10 步检查一次
    # 安全间距 >= speed_limit * (1/collision_check_rate)
    # = 0.5 * 0.1 = 0.05m = 5cm

    self_collision_proximity_threshold: 0.02  # 自碰撞阈值 (m)
    scene_collision_proximity_threshold: 0.03 # 环境碰撞阈值 (m)

    # === 奇异性管理 ===
    hard_stop_singularity_threshold: 30.0
    # 条件数 > 30 时硬停止
    # Franka 典型工作范围条件数: 5-15
    # 接近奇异性时条件数 > 100
    leaving_singularity_threshold_multiplier: 2.0
    # 离开奇异性的阈值放松系数，防止抖动

    # === 输入类型选择 ===
    command_in_type: "speed_units"
    # "speed_units": 输入直接是速度 (m/s, rad/s)
    # "unitless": 输入是 [-1, 1] 的归一化值
    # SpaceMouse 通常输出 unitless，需设为 "unitless"，
    # 并把上面的 scale.linear / scale.rotational 调到保守值。
```

**Servo 与 Visual Servoing 的集成模式**：

```cpp
// 视觉伺服控制循环
class VisualServoController : public rclcpp::Node {
public:
    VisualServoController() : Node("visual_servo") {
        // 订阅相机检测结果
        detection_sub_ = create_subscription<PoseStamped>(
            "/object_pose", 10,
            [this](PoseStamped::SharedPtr msg) {
                current_detection_ = *msg;
            });

        // 发布 Servo 速度命令
        twist_pub_ = create_publisher<TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        // 控制定时器 (50 Hz)
        timer_ = create_wall_timer(20ms,
            [this]() { controlLoop(); });
    }

private:
    void controlLoop() {
        // 计算视觉误差
        auto error = computePoseError(
            current_detection_, target_pose_);

        // 比例控制（P 控制）
        TwistStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "panda_link0";
        cmd.twist.linear.x = kp_linear_ * error.position.x;
        cmd.twist.linear.y = kp_linear_ * error.position.y;
        cmd.twist.linear.z = kp_linear_ * error.position.z;
        cmd.twist.angular.x = kp_angular_ * error.angular.x;
        cmd.twist.angular.y = kp_angular_ * error.angular.y;
        cmd.twist.angular.z = kp_angular_ * error.angular.z;

        // 速度限幅
        clampTwist(cmd.twist, max_linear_, max_angular_);

        // 收敛检查
        if (poseErrorNorm(error) < convergence_threshold_) {
            // 发送零速度停止
            cmd.twist = geometry_msgs::msg::Twist();
        }

        twist_pub_->publish(cmd);
    }
};
```

### 练习

1. **[A 型]** 配置 Servo + 键盘输入遥操作 Franka Panda。

2. **[思考题]** Servo 碰撞检测 10 Hz vs 命令 100 Hz——中间 10 步未被检查。如何设计安全间距弥补检测间隙？

3. **[A 型]** 实现简化的 Visual Servoing：用 ArUco marker 检测目标位姿，通过 Servo 实时跟踪。测量跟踪精度和响应延迟。

---

## M14.7 多机器人 PlanningScene 管理 ⭐⭐⭐⭐

### 动机

当工作站有多台机械臂（双臂系统、人机协作），需要在同一 PlanningScene 中协调规划。

### 多机器人 PlanningScene 的三种架构

**架构 A：共享 PlanningScene（推荐）**

所有机器人共享一个 move_group 节点和一个 PlanningScene：

```
┌─────────────────────────────────────────┐
│        共享 PlanningScene                │
│  ├── left_arm (7 DOF)                   │
│  ├── right_arm (7 DOF)                  │
│  ├── both_arms (14 DOF 联合组)          │
│  ├── 环境碰撞对象                       │
│  └── ACM (含跨臂碰撞检查)              │
└─────────────────────────────────────────┘
         │                    │
    left_arm 规划        right_arm 规划
    (自动检查与右臂碰撞)  (自动检查与左臂碰撞)
```

优势：跨臂碰撞检测自动完成。劣势：14-DOF 联合规划慢。

**架构 B：独立 PlanningScene + 互斥锁**

每臂独立的 move_group 节点，通过软件互斥避免冲突：

```
┌──────────────┐    ┌──────────────┐
│  left_arm     │    │  right_arm    │
│  move_group   │    │  move_group   │
│  (独立 PS)    │    │  (独立 PS)    │
└──────┬───────┘    └──────┬───────┘
       │                    │
       └──── workspace_lock ────┘
             (ROS2 Service)
```

优势：各自独立，规划互不阻塞。劣势：需要手动管理工作空间分配。

**架构 C：时间分片（最简单）**

两臂交替工作，不需要碰撞检查：

```
时间线: ─left_pick──left_place──right_pick──right_place──
        │                      │
        左臂工作时右臂静止     右臂工作时左臂静止
```

优势：零碰撞风险。劣势：效率低（只有 50% 利用率）。

### 双臂 SRDF 配置详解

```xml
<!-- dual_panda.srdf -->
<robot name="dual_panda">
  <group name="left_arm">
    <chain base_link="world" tip_link="left_panda_hand"/>
  </group>
  <group name="right_arm">
    <chain base_link="world" tip_link="right_panda_hand"/>
  </group>
  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>
  <group name="left_hand">
    <link name="left_panda_leftfinger"/>
    <link name="left_panda_rightfinger"/>
  </group>
  <group name="right_hand">
    <link name="right_panda_leftfinger"/>
    <link name="right_panda_rightfinger"/>
  </group>

  <!-- 关键：ACM 中跨臂碰撞必须保留！ -->
  <!-- 同臂相邻连杆可以 disable -->
  <disable_collisions link1="left_panda_link0"
                      link2="left_panda_link1"
                      reason="Adjacent"/>
  <!-- ... 同臂其他相邻对 ... -->

  <!-- 跨臂碰撞不要 disable：
       left_panda_link5 和 right_panda_link5 可能碰撞。 -->
</robot>
```

### 多机器人碰撞对象的动态管理

双臂系统中，一臂抓取的物体可能进入另一臂的工作空间：

```cpp
// 左臂抓取物体后，右臂的 PlanningScene 需要更新
void updateCrossArmCollision(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& object_id,
    const std::string& attach_link)
{
    // 1. 将物体 attach 到左臂末端
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = attach_link;  // "left_panda_hand"
    aco.object.id = object_id;
    aco.object.operation =
        moveit_msgs::msg::CollisionObject::ADD;
    psi.applyAttachedCollisionObject(aco);

    // 2. 共享 PlanningScene 架构下，右臂规划时自动检查
    //    与 attached 物体的碰撞——无需额外代码
    // 3. 独立 PlanningScene 架构下，需要手动同步：
    //    将 attached 物体的包围盒发布给右臂的 PS
}
```

### 工业产线集成案例：CNC 上下料工作站 ⭐⭐⭐

```
┌─────────────────────────────────────────────────────┐
│                  CNC 上下料工作站                      │
│                                                      │
│  ┌───────┐  传送带  ┌───────┐  CNC 加工  ┌───────┐  │
│  │ 来料   │ ──────► │ 机械臂 │ ──────────►│ 出料   │  │
│  │ 缓冲区 │         │ (Panda)│           │ 检测区 │  │
│  └───────┘         └───┬───┘           └───┬───┘  │
│                        │                    │       │
│                    CNC 门控              视觉检测    │
│                    (I/O 信号)            (合格/不合格) │
└─────────────────────────────────────────────────────┘
```

**BT + MTC 集成的完整工作流**：

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="CNCLoadUnload">
    <Sequence>
      <!-- 等待 CNC 完成加工 -->
      <WaitForSignal signal="cnc_done" timeout="120000"/>

      <!-- 打开 CNC 门 -->
      <SetDigitalOutput pin="cnc_door" value="open"/>
      <WaitForSignal signal="door_open" timeout="5000"/>

      <!-- 取出已加工工件 -->
      <Fallback>
        <SubTree ID="PickFromCNC"
                 object="finished_part"
                 place_target="inspection_zone"/>
        <Sequence>
          <RetractFromCNC/>
          <RetryUntilSuccessful num_attempts="2">
            <SubTree ID="PickFromCNC"
                     object="finished_part"
                     place_target="inspection_zone"/>
          </RetryUntilSuccessful>
        </Sequence>
      </Fallback>

      <!-- 放入新毛坯 -->
      <SubTree ID="PickFromConveyor"
               object="raw_part"
               place_target="cnc_fixture"/>

      <!-- 关闭 CNC 门，启动加工 -->
      <SetDigitalOutput pin="cnc_door" value="close"/>
      <WaitForSignal signal="door_closed" timeout="5000"/>
      <SetDigitalOutput pin="cnc_start" value="pulse"/>

      <!-- 对已加工工件做视觉检测 -->
      <SubTree ID="InspectPart"
               result="{inspection_result}"/>
      <Fallback>
        <IsPartOK result="{inspection_result}"/>
        <MoveToRejectBin/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
```

**这个案例的工程要点**：
1. **I/O 集成**：BT 节点通过 `SetDigitalOutput` / `WaitForSignal` 与 PLC 通信（通过 Modbus/TCP 或 EtherCAT）
2. **定位精度**：CNC 夹具定位精度 <0.5mm，MTC 的 approach 需要补偿
3. **节拍时间**：整个上下料周期要在 CNC 加工时间内完成（通常 30-60 秒）
4. **安全联锁**：CNC 门打开时机械臂才能进入，门关闭时机械臂必须退出

### 动机

当工作站有多台机械臂（双臂系统、人机协作），需要在同一 PlanningScene 中协调规划。

### 配置要点

双臂 SRDF 需要定义联合规划组：

```xml
<robot name="dual_arm_system">
  <group name="left_arm">
    <chain base_link="world" tip_link="left_hand"/>
  </group>
  <group name="right_arm">
    <chain base_link="world" tip_link="right_hand"/>
  </group>
  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>
</robot>
```

**三种规划策略**：

| 策略 | 方法 | 适用场景 |
|------|------|---------|
| 独立规划 | 分别为每臂规划，检查碰撞 | 工作空间不重叠 |
| 联合规划 | 14-DOF 联合规划 | 需要同步配合 |
| 优先级规划 | 先规划一臂，另一臂视其为障碍 | 一主一辅 |

### 工业集成案例

**码垛案例**：
```
BT.CPP (顶层)
├── ScanPallet (视觉检测托盘状态)
├── ForEachBox:
│   ├── MTC PickBox (从传送带抓取)
│   ├── MTC PlaceBox (放到托盘指定位置)
│   └── VerifyPlacement (视觉确认)
└── SignalConveyor (通知传送带继续)
```

**焊接案例**：
```
BT.CPP (顶层)
├── LoadWeldProgram (读取焊接路径)
├── MTC ApproachWeldStart (移动到焊接起点)
├── ReactiveSequence:
│   ├── IsArcStable (监控焊接电弧)
│   └── Servo FollowWeldPath (沿焊缝实时跟踪)
└── MTC Retreat (焊完退回)
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：双臂 ACM 错误跳过跨臂碰撞
   错误做法：SRDF 的 disable_collisions 包含跨臂连杆对
   现象：两臂规划时互相穿过——危险
   正确做法：ACM 只 disable 同一臂内相邻连杆，跨臂碰撞必须检查
```

### 练习

1. **[思考题]** 14-DOF 联合规划的 C-space 是独立 7-DOF 的两倍。OMPL 采样效率随维度急剧下降。替代策略？（考虑优先级规划、时间参数化协调）

2. **[跨章综合题]** 结合 M12（ros2_control）、M13（BT.CPP）、M14（MoveIt2+MTC），设计一个完整的 pick-and-place 系统架构：(a) BT.CPP 顶层编排包含错误恢复，(b) BT Action 节点内部调用 MTC 做多阶段规划，(c) MTC 输出轨迹通过 JTC 执行，(d) PlanningScene 动态管理碰撞对象。画出完整的系统架构图和数据流。

---

## M14.8 前沿展望：MoveIt2 2026 路线图、MoveIt Servo 2.0 与 Foundation Model 集成 ⭐⭐⭐⭐

前七节覆盖了 MoveIt2 和 MTC 从架构到工业部署的核心内容。MoveIt 生态仍在快速迭代，以下梳理近期路线图中与教学和工程选型直接相关的发展方向。

### MoveIt2 2025-2026 路线图

MoveIt2 社区（主要由 PickNik Robotics 驱动）在 2025-2026 年聚焦以下方向：

| 方向 | 进展 | 对用户的影响 |
|------|------|------------|
| **Python-first API** | `moveit_py` 已成为推荐入口 | 80%+ 的新部署使用 Python，C++ 仅用于性能关键路径 |
| **cuMotion 集成** | NVIDIA cuMotion（GPU 加速碰撞检测+规划）作为 pluginlib 插件接入 | 碰撞检测速度提升 10-100x，使 1000+ FPS 的实时避障成为可能 |
| **行为树原生集成** | MoveIt Pro 的 Objective 节点模式向开源版渗透 | BT 节点可以直接调用 MoveIt2 API，无需自定义 Action Server 中转 |
| **多机器人原生支持** | 双臂/多臂的 PlanningScene 和 SRDF 支持标准化 | 不再需要为双臂场景手动 hack SRDF |

### MoveIt Servo 2.0

MoveIt Servo 是 MoveIt2 中用于实时笛卡尔遥操作和 visual servoing 的组件。Servo 2.0（随 ROS2 Jazzy/Rolling 发布）相比 1.0 有重大架构变化：

**关键改进**：
1. **Composable Node 架构**：Servo 2.0 可以作为 composable node 加载到 move_group 进程中，消除了跨进程通信延迟——这对 visual servoing 的 100+Hz 重规划至关重要
2. **改进的奇异性管理**：新增"奇异性缩放"模式——接近奇异构型时自动降低笛卡尔速度（而非硬停止），提供更平滑的操作体验
3. **碰撞距离场**（Collision Distance Field）：用 3D 距离场替代逐对碰撞检查，实现亚毫秒级的碰撞距离计算

> **本质洞察**：MoveIt Servo 的设计哲学与 MoveIt2 主规划管线截然不同。主规划管线是"离线"的——给定起点和终点，搜索一条完整路径。Servo 是"在线"的——每个控制周期根据当前状态和用户输入增量更新目标。这两种模式覆盖了机器人运动控制的两个极端：离线规划适合已知环境中的点到点运动；在线 Servo 适合未知/动态环境中的人机交互。

### Foundation Model 集成概念

2024-2025 年的一个趋势是将视觉-语言基础模型（Foundation Models）与 MoveIt2 集成，形成"感知→推理→规划→执行"的完整闭环：

```
用户指令 ("把红色杯子放到架子上")
    │
    ▼
VLM (视觉-语言模型, 如 GPT-4V / Gemini)
    ├── 场景理解: 识别杯子位置、架子位置、障碍物
    ├── 任务分解: pick(红色杯子) → place(架子)
    └── 抓取策略: 从侧面抓取（杯子有把手）
    │
    ▼
MTC Task (MoveIt2)
    ├── GenerateGraspPose: VLM 提供的抓取方向
    ├── Connect + ComputeIK: MoveIt2 标准流程
    └── 执行: ros2_control
```

**当前局限**：
- VLM 的推理延迟（100ms-2s）与 MoveIt2 的规划延迟（100ms-1s）叠加，端到端延迟可能达到 3-5s——不适合需要快速反应的场景
- VLM 的空间推理能力仍然有限——对于精确的 6-DOF 位姿估计，专用的位姿估计网络（如 FoundationPose, GraspNet）仍然优于通用 VLM
- 安全保证：VLM 的输出是概率性的，不能保证物理安全——必须经过 MoveIt2 的碰撞检测和约束验证才能执行

> **跨领域类比**：Foundation Model 在机器人中的角色类似于自动驾驶中的"感知-预测"模块——它提供高层语义理解（"这是一个杯子，应该从侧面抓"），但不直接控制执行器。MoveIt2/ros2_control 类似于"规划-控制"模块——负责把高层意图转化为安全的、物理可行的运动。两者的分工边界正在随着 Foundation Model 能力的增强而移动，但短期内"安全保证"仍然是传统规划框架的不可替代优势。

---

## 本章小结

| 知识点 | 核心内容 | 难度 |
|--------|---------|------|
| M14.1 架构全景 | pluginlib 三层工厂、分层解耦 | ⭐⭐ |
| M14.2 两套 API | MoveGroupInterface vs MoveItCpp | ⭐⭐ |
| M14.3 PlanningScene | 碰撞对象、ACM、attached、diff 同步 | ⭐⭐ |
| M14.4 MTC 深度 | Stage 体系、Composite Pattern、pick-and-place | ⭐⭐ |
| M14.5 规划管线 | 链式/并行、规划器选型决策流程 | ⭐⭐⭐ |
| M14.6 Servo | 实时笛卡尔控制、奇异性管理 | ⭐⭐⭐ |
| M14.7 多机器人 | 双臂 ACM、三种协调策略、工业案例 | ⭐⭐⭐⭐ |

## 累积项目：本章新增模块

```
mini_manip_ws/
├── src/
│   ├── mini_manip_moveit_config/    # Setup Assistant 生成
│   │   ├── config/
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   ├── controllers.yaml
│   │   │   └── pipeline_configs.yaml
│   │   └── launch/
│   ├── mini_manip_mtc/             # 本章新增
│   │   ├── src/pick_place_task.cpp
│   │   └── CMakeLists.txt
│   └── ...
```

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| MoveIt2 官方文档 (`moveit.picknik.ai`) | ⭐ | 教程和 API |
| MoveIt2 Tutorials 仓库 | ⭐⭐ | 完整教程代码 |
| MTC 文档 | ⭐⭐ | MTC 教程 |
| Tesseract (`tesseract-robotics/tesseract`) | ⭐⭐⭐ | 替代框架对比 |
| Sucan et al. (2013) "MoveIt!" IEEE RAM | ⭐⭐ | 原始论文 |
| PickNik MoveIt Pro | ⭐⭐⭐ | 商业版参考 |

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| "no solution found" | IK 无解或碰撞 | 1. 检查目标可达性 2. RViz 可视化碰撞 3. 放宽容差 | M14.2 |
| 轨迹穿过障碍物 | PlanningScene 未同步 | 1. 检查碰撞对象 2. 等待同步后规划 | M14.3 |
| MTC 返回 0 解 | Stage 约束过紧 | 1. 逐 Stage 调试 2. 增大 IK 采样数 3. 检查方向 | M14.4 |
| Servo 奇异性失控 | 未设奇异性管理 | 1. 设置奇异性减速/硬停止阈值 2. 降低速度缩放 | M14.6 |
| 双臂互穿 | ACM 错误跳过跨臂碰撞 | 1. 检查 SRDF 2. 确认跨臂碰撞未 disable | M14.7 |
| pluginlib 加载失败 | 插件未安装或路径错误 | 1. `ros2 pkg list` 2. 检查 plugin.xml | M14.1 |
