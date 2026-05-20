> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# D10 综合实战——Mini-DualArm 从零搭建

> **本章定位**：本章是双臂协调与遥操作系列（D01-D10，24 周）的总收尾章。从一个空白工作区开始，逐步搭建一个完整的双臂遥操作 + 自主操作系统。本章不引入新理论——它的价值在于**将 D01-D09 的所有知识在一个统一系统中落地**，让你体验从"单一控制律"到"双臂 + 遥操作完整系统"的工程鸿沟。
>
> **适用范围**：本章的系统架构适用于任何双臂桌面操作平台。代码示例基于 MuJoCo + ROS2 + MoveIt2，可替换为 Gazebo 或 Isaac Sim。
>
> **前置依赖**：D01-D09（全部双臂/遥操作章节）——本章是所有前置知识的综合应用
>
> **建议用时**：3 周（30-45 小时）

---

## 前置自测 ⭐

> 📋 **答不出 >= 3 题 → 先回前置章节复习，本章是综合实战，需要全面的前置知识**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **双臂 URDF**：如何用 Xacro prefix 参数化创建双臂 URDF？`both_arms` planning group 在 SRDF 中如何定义？ | D09.1-D09.2 |
| 2 | **MoveIt2 规划**：同步规划（both_arms 14D）和异步规划（两个 7D）的选型依据是什么？ | D09.3 |
| 3 | **Object Impedance**：双臂共持物体时，Object Impedance 控制的数学形式是什么？内力回路的作用？ | D03 双臂协调力控 |
| 4 | **遥操作映射**：ALOHA 的关节映射和 OpenTeleVision 的笛卡尔映射分别适用于什么场景？ | D08.2 |
| 5 | **数据采集**：LeRobot 数据格式的结构是什么？ACT 训练需要哪些字段？ | D08.6-D08.7 |
| 6 | **ros2_control**：双臂合并轨迹方案和并行执行器方案的区别？ | D09.6 |

---

## 本章目标

学完本章后，你应该能够：

1. **从零搭建**完整的双臂遥操作 + 自主操作系统（MuJoCo 仿真环境 + ROS2 + MoveIt2）
2. **实现三个核心任务**：双臂协同搬运、GELLO 式遥操作数据采集、ACT 策略部署
3. **对比分析**三种控制方式（MoveIt2 规划 / 手工阻抗控制 / ACT 策略）的性能差异
4. **体验**从"单一控制律"到"完整系统"的工程集成挑战
5. **形成**可复用的双臂系统开发模板，为后续研究项目打下基础

---

## D10.1 系统架构设计 ⭐⭐

### 动机——为什么需要统一架构？

过去 9 章中，我们学了很多独立的技术：运动学、规划、力控、遥操作、数据采集。但一个实际的双臂系统需要这些技术**同时工作**。系统架构定义了这些模块之间的接口、通信、时序——它决定了系统能否正确、高效、可靠地运行。

### 如果不做架构设计会怎样

> 新手的典型做法：每个功能写一个独立的 Python 脚本。`moveit_plan.py`、`teleop.py`、`train_act.py`、`run_policy.py`——各自直接读写关节。
>
> 后果：(1) 多个脚本同时运行时抢占关节控制权，机械臂在多个目标间跳动；(2) 遥操作脚本的数据格式和 ACT 训练脚本不兼容——需要写转换脚本；(3) 切换模式需要杀掉一个脚本、启动另一个——中间有几秒的"无控制"空窗期，机械臂可能下落；(4) 添加新功能（如力控）需要修改所有脚本。
>
> 统一架构解决了所有这些问题：模块通过标准 ROS2 接口通信，模式切换通过 topic 消息，数据格式全局统一。

### 完整系统架构

```
┌──── MuJoCo 仿真 ────────────────────────────────────────┐
│  双 Franka Panda + 桌面 + 物体（箱子、杆、瓶子）          │
│  物理引擎: MuJoCo 3.x (接触稳定, 1kHz)                   │
└──────────────── mujoco_ros2_control ─────────────────────┘
         ↕ ros2_control HardwareInterface × 2
┌──── 控制层 ──────────────────────────────────────────────┐
│  方案 A: both_arms_controller (14D JTC, 1 kHz)           │
│  方案 B: left/right_arm_controller (各 7D JTC, 1 kHz)    │
│  方案 C: left/right_impedance_controller (阻抗, 1 kHz)   │
│  协调层: Object Impedance + 内力回路 (方案 C 时启用)       │
└──────────────────────────────────────────────────────────┘
         ↕ ROS2 话题/服务/Action
┌──── 任务规划层 ──────────────────────────────────────────┐
│  MoveIt2 MoveGroup (both_arms / left_arm / right_arm)    │
│  MTC 任务编排 (pick → lift → transport → place)          │
│  FSM/Mode Manager (模式切换: 自主/遥操作/策略)            │
└──────────────────────────────────────────────────────────┘
         ↕ ROS2 话题/服务
┌──── 遥操作/策略层 ───────────────────────────────────────┐
│  模式 A: GELLO 式遥操作 (关节镜像, 100 Hz)               │
│  模式 B: ACT 策略推理 (50 Hz, GPU)                       │
│  模式 C: MoveIt2 自主规划 (both_arms 组)                  │
└──────────────────────────────────────────────────────────┘
         ↕
┌──── 数据层 ──────────────────────────────────────────────┐
│  LeRobot 数据采集 (parquet + mp4)                        │
│  → ACT 训练 (GPU) → 策略部署                              │
│  数据质量评估 (Jerk/效率/覆盖度)                          │
└──────────────────────────────────────────────────────────┘
```

### 模块间接口定义

| 接口 | 话题/服务 | 消息类型 | 频率 | 说明 |
|------|---------|---------|------|------|
| 关节状态 | `/joint_states` | `sensor_msgs/JointState` | 100 Hz pub | 14 关节位置/速度/力矩 |
| 关节指令 | `/both_arms_controller/...` | `trajectory_msgs/JointTrajectory` | Action | MoveIt2 → 控制器 |
| 相机图像 | `/camera_{high,left,right}/image_raw` | `sensor_msgs/Image` | 30 Hz | 3 个相机 |
| 遥操作指令 | `/teleop/joint_target` | `std_msgs/Float64MultiArray` | 50-100 Hz | leader 关节位置 |
| ACT 动作 | `/policy/action` | `std_msgs/Float64MultiArray` | 50 Hz | 策略输出 14D 关节目标 |
| 模式请求 | `/mode_switch` | `std_msgs/String` | 事件触发 | "teleop" / "policy" / "moveit"，只作为请求，不直接抢占控制器 |
| 关节指令仲裁 | `/joint_commands` | `std_msgs/Float64MultiArray` | 50-100 Hz | Command Arbiter 统一发布到底层控制器 |

### 三种操作模式

| 模式 | 控制流 | 适用 | 优势 | 劣势 |
|------|--------|------|------|------|
| **MoveIt2 自主** | 目标位姿 → OMPL → JTC | 简单几何任务 | 全局避碰 | 不能力控 |
| **遥操作采集** | leader → follower + 录制 | 数据采集 | 人类直觉 | 需操作者 |
| **ACT 策略** | 图像+状态 → Transformer → 动作 | 部署阶段 | 无需人类 | 需训练数据 |

> **本质洞察**：Mini-DualArm 的三种模式不是三个独立系统，而是**同一个系统的三种运行方式**。它们共享同一个 URDF、同一个 ros2_control 栈、同一个仿真环境——区别只在于"谁生成关节指令"。这种设计的好处是：遥操作采集的数据可以直接用于训练 ACT 策略，训练好的策略可以直接在同一个系统上部署——**零迁移成本**。

### 设计原则

**跨领域类比——微服务架构**：Mini-DualArm 的设计与互联网的微服务架构有相似之处。每个 ROS2 节点是一个"微服务"（遥操作、策略推理、MoveIt2 规划），它们通过标准话题（相当于 REST API）通信，模式切换相当于"服务路由"。微服务的核心原则——松耦合、接口标准化、独立部署——在这里同样适用。

**四条设计规则**：

1. **单一数据源**：所有模式使用同一个 URDF 和同一个 ros2_control 配置
2. **模块化切换**：通过 ROS2 topic 请求模式切换，由 Command Arbiter 或 ros2_control 控制器生命周期统一仲裁
3. **统一数据格式**：无论哪种模式，录制的数据格式相同（LeRobot）
4. **渐进式开发**：先让单臂工作 → 双臂独立 → 双臂协调 → 遥操作 → 策略

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：多个模式同时发送关节指令
   错误做法：遥操作节点和 MoveIt2 同时 active，各自发送 joint command
   现象：关节在两个目标之间高频跳动，机械臂剧烈振荡
   根本原因：ros2_control 的 JTC 接受最新的指令，两个来源交替覆盖
   正确做法：不要让业务节点直接写同一个控制器。增加 Command Arbiter：
            teleop/policy/moveit 只发布候选指令，arbiter 根据当前模式
            只转发一个来源，并在切换时先发零速/保持当前位置、等待确认。
            真机或 ros2_control 场景下，也可用 controller_manager/lifecycle
            只 activate 一个占用关节资源的控制器，切换前 deactivate 旧控制器，
            activate 新控制器，避免多个控制器同时 claim 同一批 joint。

🧠 思维陷阱：认为"先搭完整系统再调试"
   新手想法："把所有模块装好，然后一起调试"
   实际上：系统越复杂，同时调试越困难。bug 可能在任何层。
   正确做法：自底向上逐层验证——
   1. 先验证 MuJoCo 关节读写
   2. 再验证 ros2_control 连接
   3. 再验证 MoveIt2 规划
   4. 再加遥操作
   5. 最后加策略推理
   每层验证通过后再往上叠加。

💡 概念误区：认为"综合实战就是把代码拼起来"
   新手想法："D01-D09 的代码各复制一份，放到一个文件夹就行"
   实际上：系统集成的难度不在于代码本身，而在于接口匹配、时序协调、
   资源管理、错误处理。一个 1000 行的集成系统比 10 个 100 行的独立脚本
   难 10 倍以上——因为 bug 来源从"代码逻辑错误"扩展到"模块间交互错误"。
```

### 练习

1. **[设计]** 画出完整的 ROS2 节点图（node graph），标注每个节点的输入输出话题。用 `rqt_graph` 验证实际运行时的图与设计一致。
2. **[思考题]** 如果要在真机上部署（从 MuJoCo 换到真实 Franka），哪些模块需要修改？哪些可以直接复用？提示：ros2_control 的 HardwareInterface 需要替换，但控制器、MoveIt2 配置、遥操作逻辑不变。

---

## D10.2 阶段式开发路线 ⭐⭐

### 路线 Alpha（基础，2 周）

**目标**：搭建可运行的双臂系统，实现三个核心功能。

```
Day 1-2: MuJoCo 双 Franka 环境搭建
  ├─ URDF xacro 组合（D09.1 知识）
  ├─ MuJoCo 导入验证（14 关节读写正常）
  └─ ros2_control HardwareInterface 连接

Day 3-4: MoveIt2 双臂配置
  ├─ SRDF + both_arms 组（D09.2 知识）
  ├─ kinematics.yaml + controllers.yaml
  ├─ ACM 优化（D09.4 知识）
  └─ RViz 中验证 both_arms 规划

Day 5-6: Object Impedance 双臂搬运
  ├─ 阻抗控制器配置（D03/F04 知识）
  ├─ 30cm 杆的共持搬运
  ├─ 参数调节: K_o=[500,500,500,20,20,20], f_int=40N
  └─ 验证: 搬运 50cm 距离不掉落

Day 7-8: GELLO 式遥操作 + 数据记录
  ├─ 关节镜像实现（D08.2 知识）
  ├─ ROS2 多模态同步录制（D08.6 知识）
  ├─ LeRobot/HDF5 格式输出
  └─ 采集 50 episodes

Day 9-10: ACT 策略训练 + 部署
  ├─ LeRobot 数据加载（D08.6 知识）
  ├─ ACT 训练 2000 epochs（D04 知识）
  ├─ 策略部署到双 Franka 仿真
  └─ 记录 20 次测试成功率

Day 11-12: 性能对比 + 报告
  ├─ MoveIt2 vs Object Impedance vs ACT 三方法对比
  ├─ 指标: 成功率 / 完成时间 / 力矩峰值
  └─ 技术报告撰写
```

### 路线 Beta（进阶，3 周）

**目标**：在 Alpha 基础上添加高级功能和全面对比。

```
Week 1: 路线 Alpha 全部内容

Week 2:
  Day 1-2: OMPL ConstrainedStateSpace 约束规划
    ├─ 两末端保持固定相对位姿约束（D02 理论落地）
    ├─ 自定义 ompl::base::Constraint
    └─ 验证共持物体搬运路径

  Day 3-4: Handover 实现
    ├─ 左手拿物 → lift → 右手接近 → 交接
    ├─ MTC handover 管线（D09.5 知识）
    └─ 验证 10 次交接成功率

  Day 5: TDPA 安全层集成
    ├─ 遥操作模式加入 TDPA 能量监控（D07 知识）
    └─ 验证: 人为制造延迟(100ms)时系统仍稳定

Week 3:
  Day 1-2: 双臂 RL（robosuite TwoArmLift PPO）
    ├─ robosuite 环境配置
    ├─ PPO 训练 1M steps
    └─ 部署到 MuJoCo 环境

  Day 3-4: 全方法对比
    ├─ 手工 Object Impedance vs ACT vs RL vs MoveIt2
    ├─ 四种方法在 5 个任务上的对比
    └─ 可视化对比表和图

  Day 5: 完整演示 + 技术报告
    ├─ 录制演示视频（三种模式切换）
    └─ 撰写完整技术报告
```

### 阶段式验收标准

每个阶段必须通过验收标准后才能进入下一阶段。**跳过验收是系统集成失败的第一大原因**。

| 阶段 | 验收标准 | 量化指标 | 验证方法 | 不通过的后果 |
|------|---------|---------|---------|-------------|
| **Day 1-2: 环境** | 14 关节可独立读写 | 跟踪误差 < 0.05 rad | `test_env.py` 全绿 | 后续所有步骤无法进行 |
| **Day 3-4: MoveIt2** | 三种 group 规划成功 | 规划时间 < 10s，100% 成功 | `test_moveit.py` 全绿 | 无法自主规划 |
| **Day 5-6: 力控** | 杆搬运 50cm 不脱落 | 姿态偏差 < 5 度，成功率 > 80% | 20 次搬运实验 | 力控不稳定 |
| **Day 7-8: 遥操作** | 50 episodes 数据可读 | HDF5 字段完整，Jerk < 200 | 数据加载 + 统计 | 训练数据不足 |
| **Day 9-10: ACT** | 策略可运行 | 成功率 > 50%（20 次） | 策略部署测试 | 需要更多数据或调参 |
| **Day 11-12: 报告** | 三方法对比表完成 | 5 指标 × 3 方法 | — | — |

**关键验收脚本**：

```python
# tests/test_env.py — 环境验收
def test_joint_count():
    """验收 1: 14 关节可读"""
    env = DualFrankaEnv("config/dual_panda.urdf")
    q = env.get_joint_positions()
    assert len(q) == 14, f"Expected 14, got {len(q)}"

def test_joint_tracking():
    """验收 2: 关节跟踪精度"""
    env = DualFrankaEnv("config/dual_panda.urdf")
    env.reset()
    target = env.get_joint_positions()
    target[0] += 0.3  # 左臂 J1
    target[7] -= 0.3  # 右臂 J1
    for _ in range(2000):
        env.set_joint_targets(target)
        env.step()
    error = np.linalg.norm(env.get_joint_positions() - target)
    assert error < 0.05, f"Tracking error {error:.4f} > 0.05"

def test_no_self_collision_at_home():
    """验收 3: home 构型无自碰撞"""
    env = DualFrankaEnv("config/dual_panda.urdf")
    env.reset()
    contacts = env.get_contact_count()
    assert contacts == 0, f"Self-collision at home: {contacts} contacts"
```

### 每日检查点

每天结束时，用以下清单自检当日进度：

| 检查项 | 验证方法 | 通过标准 |
|--------|---------|---------|
| 关节读写 | 打印 14 关节位置 | 数值合理（在关节限位内）且实时更新 |
| 碰撞检测 | 手动拖两臂到中间 | MoveIt2 在 RViz 中标红碰撞 |
| 规划执行 | both_arms 到 home | 两臂同步到达目标，无碰撞 |
| 遥操作 | 移动 leader（键盘模拟） | follower 实时跟随，延迟 < 100ms |
| 数据录制 | 检查输出文件 | HDF5 包含 qpos/action/image 字段，维度正确 |
| 策略推理 | 加载模型运行 | 输出 14D 动作向量，数值在合理范围 |

---

## D10.3 环境搭建实战（Day 1-2）⭐⭐

### 项目目录结构

```
mini_dualarm/
├── CMakeLists.txt                  # colcon/ament 构建配置
├── package.xml                     # ROS2 包描述
├── config/
│   ├── dual_panda.urdf.xacro      # 双臂 URDF（D09.1）
│   ├── dual_panda.srdf             # SRDF（D09.2）
│   ├── kinematics.yaml             # IK 配置
│   ├── ompl_planning.yaml          # OMPL 配置
│   ├── ros2_controllers.yaml       # ros2_control 配置（仿真）
│   ├── ros2_controllers_real.yaml  # ros2_control 配置（真机）
│   ├── moveit_controllers.yaml     # MoveIt2 执行器映射
│   ├── impedance_params.yaml       # Object Impedance 参数
│   └── moveit.rviz                 # RViz 布局
├── launch/
│   ├── sim.launch.py               # MuJoCo + ros2_control
│   ├── gazebo.launch.py            # Gazebo Harmonic 方案
│   ├── moveit.launch.py            # MoveIt2 + RViz
│   ├── teleop.launch.py            # 遥操作 + 数据采集
│   └── full_system.launch.py       # 完整系统启动
├── src/
│   ├── dual_franka_env.py          # MuJoCo 仿真环境
│   ├── mujoco_bridge.py            # ROS2 桥接节点
│   ├── teleop_node.py              # 遥操作节点
│   ├── data_collector.py           # 数据采集节点
│   ├── object_impedance.py         # 双臂阻抗控制器
│   ├── policy_inference.py         # ACT 策略推理节点
│   └── mode_manager.py             # 模式切换管理
├── scripts/
│   ├── train_act.py                # ACT 训练脚本
│   ├── evaluate.py                 # 性能评估脚本
│   ├── benchmark.py                # 性能指标采集
│   └── plot_metrics.py             # 可视化脚本
├── tests/
│   ├── test_env.py                 # 仿真环境测试
│   ├── test_moveit.py              # MoveIt2 集成测试
│   └── test_teleop.py              # 遥操作回环测试
├── data/
│   └── episodes/                   # 采集的数据存放
└── models/
    └── act_dual_arm/               # 训练好的模型
```

### CMakeLists.txt 核心配置

```cmake
cmake_minimum_required(VERSION 3.16)
project(mini_dualarm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# C++ 节点
add_executable(test_moveit_dual src/test_moveit_dual.cpp)
ament_target_dependencies(test_moveit_dual
  rclcpp moveit_ros_planning_interface)

add_executable(mtc_dual_arm src/mtc_dual_arm.cpp)
ament_target_dependencies(mtc_dual_arm
  rclcpp moveit_task_constructor_core moveit_ros_planning_interface)

# 安装
install(TARGETS test_moveit_dual mtc_dual_arm
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts src
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  src/dual_franka_env.py
  src/mujoco_bridge.py
  src/teleop_node.py
  src/data_collector.py
  src/object_impedance.py
  src/policy_inference.py
  src/mode_manager.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### MuJoCo 双 Franka 仿真环境

```python
# mini_dualarm/src/dual_franka_env.py

import mujoco
import mujoco.viewer
import numpy as np

class DualFrankaEnv:
    """双 Franka Panda MuJoCo 仿真环境"""
    
    def __init__(self, model_path: str, render: bool = True):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        if self.model.nu == 0:
            raise RuntimeError(
                "MuJoCo model has no actuators (model.nu == 0). "
                "URDF joints do not automatically create data.ctrl entries; "
                "add MJCF actuators or run the converter with actuator generation.")
        
        # 建立关节名 → MuJoCo 地址映射。
        # joint_id 只是关节表编号，不能直接当 qpos/qvel/ctrl 索引；
        # actuator 名也不一定等于 joint 名。用 actuator transmission
        # 反查“哪个 actuator 驱动哪个 joint”，再得到 ctrl 索引。
        self.qpos_map = {}
        self.qvel_map = {}
        self.ctrl_map_by_joint = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.qpos_map[name] = self.model.jnt_qposadr[i]
                self.qvel_map[name] = self.model.jnt_dofadr[i]
        for i in range(self.model.nu):
            if self.model.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_JOINT:
                continue
            joint_id = self.model.actuator_trnid[i, 0]
            joint_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_name:
                self.ctrl_map_by_joint[joint_name] = i
        
        # 分组索引
        self.left_qpos_ids = [
            self._require(self.qpos_map, f'left_panda_joint{j+1}', 'qpos')
            for j in range(7)]
        self.right_qpos_ids = [
            self._require(self.qpos_map, f'right_panda_joint{j+1}', 'qpos')
            for j in range(7)]
        self.left_qvel_ids = [
            self._require(self.qvel_map, f'left_panda_joint{j+1}', 'qvel')
            for j in range(7)]
        self.right_qvel_ids = [
            self._require(self.qvel_map, f'right_panda_joint{j+1}', 'qvel')
            for j in range(7)]
        self.left_ctrl_ids = [
            self._require(self.ctrl_map_by_joint, f'left_panda_joint{j+1}', 'ctrl')
            for j in range(7)]
        self.right_ctrl_ids = [
            self._require(self.ctrl_map_by_joint, f'right_panda_joint{j+1}', 'ctrl')
            for j in range(7)]
        
        assert len(self.left_qpos_ids) == 7, "Left arm needs 7 joints"
        assert len(self.right_qpos_ids) == 7, "Right arm needs 7 joints"
        self._expect_torque_ctrl(self.left_ctrl_ids + self.right_ctrl_ids)
        
        # 渲染
        self.viewer = None
        if render:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data)
        
        self.dt = self.model.opt.timestep

    def _require(self, mapping: dict, joint_name: str, kind: str) -> int:
        """取 MuJoCo 地址/ctrl 索引；缺失时给出可操作错误，而不是裸 KeyError。"""
        if joint_name not in mapping:
            available = ", ".join(sorted(mapping.keys())[:8])
            raise KeyError(
                f"Missing {kind} mapping for {joint_name!r}. "
                f"First available names: {available}. Check prefix and actuator names.")
        return mapping[joint_name]

    def _expect_torque_ctrl(self, ctrl_ids):
        """本示例 set_joint_targets 内部计算 tau，因此要求 ctrl 是力/力矩语义。"""
        for ctrl_id in ctrl_ids:
            if self.model.actuator_biastype[ctrl_id] != mujoco.mjtBias.mjBIAS_NONE:
                act_name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, ctrl_id)
                raise RuntimeError(
                    f"Actuator {act_name!r} does not look like a direct torque motor. "
                    "data.ctrl semantics depend on actuator type: position actuators expect q_des, "
                    "motor/general torque actuators expect force/torque-like commands.")
        
    def get_joint_positions(self) -> np.ndarray:
        """返回 14D 关节位置 [left_7, right_7]"""
        left_q = np.array([self.data.qpos[i] for i in self.left_qpos_ids])
        right_q = np.array([self.data.qpos[i] for i in self.right_qpos_ids])
        return np.concatenate([left_q, right_q])
    
    def get_joint_velocities(self) -> np.ndarray:
        """返回 14D 关节速度"""
        left_qd = np.array([self.data.qvel[i] for i in self.left_qvel_ids])
        right_qd = np.array([self.data.qvel[i] for i in self.right_qvel_ids])
        return np.concatenate([left_qd, right_qd])
    
    def set_joint_targets(self, targets: np.ndarray,
                          kp: float = 100.0, kd: float = 10.0):
        """外部 PD 位置控制：计算 tau 并写入 torque/motor actuator 的 data.ctrl。"""
        assert len(targets) == 14
        q = self.get_joint_positions()
        qd = self.get_joint_velocities()
        tau = kp * (targets - q) - kd * qd
        
        # 写入控制力矩。注意 data.ctrl 是 actuator 命令，不是 qpos；
        # 本函数只适用于 motor/general torque actuator。
        all_ctrl_ids = self.left_ctrl_ids + self.right_ctrl_ids
        for i, ctrl_id in enumerate(all_ctrl_ids):
            self.data.ctrl[ctrl_id] = tau[i]
    
    def step(self, n_substeps: int = 1):
        for _ in range(n_substeps):
            mujoco.mj_step(self.model, self.data)
        if self.viewer:
            self.viewer.sync()
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        for i, qpos_id in enumerate(self.left_qpos_ids):
            self.data.qpos[qpos_id] = home[i]
        for i, qpos_id in enumerate(self.right_qpos_ids):
            self.data.qpos[qpos_id] = home[i]
        mujoco.mj_forward(self.model, self.data)
```

### 验证脚本

```python
# scripts/test_env.py
from dual_franka_env import DualFrankaEnv
import numpy as np

env = DualFrankaEnv("config/dual_panda.urdf", render=True)
env.reset()

# 测试 1: 关节读取
q = env.get_joint_positions()
print(f"Initial positions (14D): {np.round(q, 3)}")
assert len(q) == 14, f"Expected 14 joints, got {len(q)}"

# 测试 2: 关节控制——左臂第一关节转动
target = q.copy()
target[0] += 0.5
for _ in range(1000):
    env.set_joint_targets(target)
    env.step()

q_new = env.get_joint_positions()
error = abs(q_new[0] - target[0])
print(f"Tracking error: {error:.4f} rad")
assert error < 0.05, f"Tracking failed: error={error:.4f}"

# 测试 3: 两臂同时运动
target2 = q.copy()
target2[0] += 0.3   # 左臂 joint1
target2[7] -= 0.3   # 右臂 joint1
for _ in range(1000):
    env.set_joint_targets(target2)
    env.step()

print("All environment tests PASSED")
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MuJoCo 的 ctrl 索引与 qpos 索引不一致
   错误做法：直接 self.data.ctrl[joint_id] = tau
   现象：错误的关节被控制，或 ctrl 越界
   根本原因：MuJoCo 的 actuator 和 joint 可能有不同的索引和顺序
   进阶陷阱：直接假设 actuator 名称等于 joint 名称
   反例：MJCF 中 actuator 可能叫 left_j1_motor，但 joint 叫 left_panda_joint1
   正确做法：优先通过 actuator_trntype/actuator_trnid 建立 joint_name → ctrl_id；
            如果必须靠名称查找，也要在启动时 assert 完整映射并打印映射表
   自检方法：对每个关节单独施加力矩，观察是否正确的关节运动

⚠️ 编程陷阱：URDF 导入后 model.nu == 0
   错误做法：以为 URDF 的 revolute joint 会自动变成 MuJoCo actuator
   现象：data.ctrl 为空，set_joint_targets 什么也控制不了
   正确做法：在 MJCF 中显式添加 actuator，或在 URDF→MJCF 转换阶段生成 actuator

⚠️ 编程陷阱：actuator 名称映射裸 KeyError
   错误做法：self.ctrl_map_by_joint["left_panda_joint1"] 失败时只看到 KeyError
   现象：不知道是 prefix 错、actuator 缺失，还是 transmission 没连到 joint
   正确做法：启动时打印 joint/qpos/dof/actuator 表，缺失时抛出带可用名称列表的错误

⚠️ 编程陷阱：把 data.ctrl 当作统一的关节目标
   错误做法：无论 actuator 类型都写 data.ctrl[i] = q_des 或 tau
   现象：position actuator 正常，motor actuator 却爆冲；或相反
   正确做法：先确认 actuator 的 gain/bias/type。position actuator 写位置目标；
            torque/motor actuator 才写外部 PD 算出的力矩

💡 概念误区：认为"PD 增益随便设就行"
   新手想法："kp=100, kd=10 应该差不多"
   实际上：PD 增益依赖于关节惯量。Franka 肩关节（joint1-2）的惯量
   比腕关节（joint6-7）大 10 倍以上。统一增益导致大关节欠阻尼
   （响应慢、跟踪误差大）、小关节过阻尼（抖动、能量浪费）。
   正确做法：参考 Franka 官方推荐增益，或用关节惯量矩阵对角元素
   按比例设计各关节增益。
```

### 练习

1. **[编程]** 完成 MuJoCo 双 Franka 环境搭建。运行验证脚本确认 14 关节读写正常。让两臂同时做正弦跟踪（0.5 Hz、0.3 rad 振幅），录制 10 秒视频。
2. **[编程]** 实现 ROS2 桥接节点：在一个终端运行 MuJoCo 仿真，另一个终端用 `ros2 topic echo /joint_states` 确认数据流通。

---

## D10.4 MoveIt2 双臂集成（Day 3-4）⭐⭐

### 配置验证

将 D09 的全部配置落地到 Mini-DualArm 项目中。关键验证步骤：

```cpp
// test_moveit_dual.cpp: 验证双臂规划
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_dual_arm");
    
    // 测试 1: 左臂独立规划
    auto left_arm = moveit::planning_interface::MoveGroupInterface(node, "left_arm");
    left_arm.setNamedTarget("left_home");
    moveit::planning_interface::MoveGroupInterface::Plan p1;
    auto s1 = left_arm.plan(p1);
    RCLCPP_INFO(node->get_logger(), 
                "Left arm plan: %s (%.2f s)", 
                s1 == moveit::core::MoveItErrorCode::SUCCESS ? "OK" : "FAIL",
                p1.planning_time);
    
    // 测试 2: 双臂同步规划
    auto both_arms = moveit::planning_interface::MoveGroupInterface(node, "both_arms");
    both_arms.setNamedTarget("both_home");
    both_arms.setPlanningTime(10.0);
    moveit::planning_interface::MoveGroupInterface::Plan p2;
    auto s2 = both_arms.plan(p2);
    RCLCPP_INFO(node->get_logger(), 
                "Both arms plan: %s (%.2f s)", 
                s2 == moveit::core::MoveItErrorCode::SUCCESS ? "OK" : "FAIL",
                p2.planning_time);
    
    // 测试 3: 执行
    if (s2) {
        both_arms.execute(p2);
        RCLCPP_INFO(node->get_logger(), "Execution complete");
    }
    
    rclcpp::shutdown();
}
```

### 练习

1. **[编程]** 在 RViz 中验证 left_arm、right_arm、both_arms 三种 Planning Group 都能正常规划。记录各自的平均规划时间。
2. **[编程]** 用 MTC 实现双臂 pick-and-place 管线（D09.5 知识的落地）。

---

## D10.5 Object Impedance 双臂搬运（Day 5-6）⭐⭐⭐

### 任务描述

两臂共持一根 30cm 铝制杆，从桌面左侧搬运到右侧（50cm 距离）。

**验收标准**：
- 杆不脱落（全程抓持力 > 20N）
- 搬运平稳（杆的姿态偏差 < 5 度）
- 完成时间 < 10 秒

### Object Impedance 控制实现

回顾 D03（双臂协调力控）：Object Impedance 控制将双臂末端的控制分解为**物体运动**和**内力**两个正交分量。物体运动由阻抗控制器驱动，内力由独立的内力闭环维持。

关键点：双臂接触力不是直接写成 `0.5 * F_obj ± F_int`。教学上正确的做法是先建立 Grasp Matrix：

```text
w_obj = G f_c
f_c = G^+ w_obj + (I - G^+ G) f_int_ref
```

其中 `w_obj` 是物体空间期望 wrench，`f_c = [f_L; f_R]` 是左右接触 wrench 堆叠，`G^+ w_obj` 负责实现物体运动，`(I - G^+ G) f_int_ref` 落在 grasp matrix 零空间内，只改变内力，不改变物体合 wrench。对称抓取、接触点完全对称、内力方向理想时，“物体力均分”可以作为直觉近似，但它只是这个公式的特例，不应作为通用实现。

所有接触 wrench、物体 wrench、接触点位置必须表达在同一个 world-aligned frame。最常见的错误是左手 wrench 用左手局部坐标、右手 wrench 用右手局部坐标，却直接堆进同一个 Grasp Matrix；这样得到的内力和物体力都会错。

```python
# mini_dualarm/src/object_impedance.py

import numpy as np
import pinocchio as pin

def skew(p):
    """返回 p 的叉乘矩阵，使 skew(p) @ f = p × f"""
    return np.array([
        [0.0, -p[2], p[1]],
        [p[2], 0.0, -p[0]],
        [-p[1], p[0], 0.0],
    ])

def se3_log_error(T_des, T_cur):
    """用 SE(3) log 计算当前物体到期望物体的 6D 误差 [linear, angular]。"""
    T_err = T_des * T_cur.inverse()
    return pin.log6(T_err).vector

class ObjectImpedanceController:
    """双臂 Object Impedance 控制器"""
    
    def __init__(self, robot_model_left, robot_model_right):
        # 两个 model 必须共享同一世界原点（例如从同一 dual-arm URDF 加载），
        # 否则 LOCAL_WORLD_ALIGNED 雅可比在不同坐标系下无法直接组合进 Grasp Matrix。
        self.model_L = robot_model_left
        self.model_R = robot_model_right
        self.data_L = robot_model_left.createData()
        self.data_R = robot_model_right.createData()
        
        # 物体空间阻抗参数
        self.K_obj = np.diag([500, 500, 500, 20, 20, 20])  # 物体刚度
        self.D_obj = np.diag([50, 50, 50, 5, 5, 5])        # 物体阻尼
        
        # 内力参数
        self.f_int_des = 40.0   # N，挤压力目标
        self.K_int = 0.5        # 内力反馈增益：输出为 N 级修正量，不直接替代设定点
        self.f_int_feedback_limit = 15.0  # N，单步反馈修正限幅
        self.f_int_max = 80.0   # N，夹持力安全上限

        # 物体动力学前馈（示例：30cm 轻杆）
        self.object_mass = 0.2
        self.object_inertia_world = np.diag([0.0015, 0.0015, 0.0002])
        self.gravity_world = np.array([0.0, 0.0, -9.81])
        
    def compute(self, T_obj_des, T_obj, dx_obj,
                q_left, q_right, dq_left, dq_right,
                dx_obj_des=None,
                ddx_obj_des=None,
                measured_internal_force=None):
        """
        计算双臂关节力矩
        T_obj_des / T_obj 为 pin.SE3；dx_obj / dx_obj_des / ddx_obj_des 均用 world-aligned 坐标表达。
        返回 (tau_left_7D, tau_right_7D)
        """
        if dx_obj_des is None:
            dx_obj_des = np.zeros(6)
        if ddx_obj_des is None:
            ddx_obj_des = np.zeros(6)

        # 计算两臂雅可比
        pin.forwardKinematics(self.model_L, self.data_L, q_left)
        pin.updateFramePlacements(self.model_L, self.data_L)
        pin.computeJointJacobians(self.model_L, self.data_L, q_left)
        J_L = pin.getFrameJacobian(
            self.model_L, self.data_L,
            self.model_L.getFrameId("left_panda_hand_tcp"),
            pin.LOCAL_WORLD_ALIGNED)
        
        pin.forwardKinematics(self.model_R, self.data_R, q_right)
        pin.updateFramePlacements(self.model_R, self.data_R)
        pin.computeJointJacobians(self.model_R, self.data_R, q_right)
        J_R = pin.getFrameJacobian(
            self.model_R, self.data_R,
            self.model_R.getFrameId("right_panda_hand_tcp"),
            pin.LOCAL_WORLD_ALIGNED)
        
        # 1. 物体运动控制（笛卡尔阻抗）
        # 位姿误差必须用 SE(3) log，不能把 6D pose 向量直接相减。
        e_obj = se3_log_error(T_obj_des, T_obj)
        dx_err = dx_obj - dx_obj_des
        w_imp = self.K_obj @ e_obj - self.D_obj @ dx_err

        # 物体重力/惯性前馈：没有这项时，共持物体的重量会被误当成位置误差来补偿。
        w_ff = np.zeros(6)
        w_ff[0:3] = self.object_mass * (ddx_obj_des[0:3] - self.gravity_world)
        w_ff[3:6] = self.object_inertia_world @ ddx_obj_des[3:6]
        w_obj = w_imp + w_ff
        
        # 2. 内力控制
        # 真实闭环应来自腕部 F/T、仿真 contact wrench 或夹爪传感器。
        # 仅靠两 TCP 距离无法估计刚性物体内力：距离不变时接触力仍可大幅变化。
        if measured_internal_force is None:
            f_int_feedback = 0.0  # 无力测量时只给开环安全夹持设定点
        else:
            f_int_error = self.f_int_des - measured_internal_force
            f_int_feedback = np.clip(
                self.K_int * f_int_error,
                -self.f_int_feedback_limit,
                self.f_int_feedback_limit)
        F_int_scalar = np.clip(
            self.f_int_des + f_int_feedback,
            0.0,
            self.f_int_max)
        
        # 3. Grasp Matrix 力分配
        p_L = self.data_L.oMf[
            self.model_L.getFrameId("left_panda_hand_tcp")
        ].translation
        p_R = self.data_R.oMf[
            self.model_R.getFrameId("right_panda_hand_tcp")
        ].translation
        p_obj = T_obj.translation
        r_L = p_L - p_obj
        r_R = p_R - p_obj

        # 接触 wrench f_i = [force_i, torque_i]，均在 world-aligned 坐标表达。
        # 单个接触对物体的贡献为 [force_i, torque_i + r_i × force_i]。
        G_L = np.block([
            [np.eye(3), np.zeros((3, 3))],
            [skew(r_L), np.eye(3)],
        ])
        G_R = np.block([
            [np.eye(3), np.zeros((3, 3))],
            [skew(r_R), np.eye(3)],
        ])
        G = np.hstack([G_L, G_R])      # 6x12
        G_pinv = np.linalg.pinv(G)     # 12x6
        N_G = np.eye(12) - G_pinv @ G  # grasp 内力零空间

        # 内力参考：沿两接触点连线相向挤压，左右力大小相等、方向相反。
        n_LR = p_R - p_L
        n_LR = n_LR / (np.linalg.norm(n_LR) + 1e-9)
        f_int_ref = np.zeros(12)
        f_int_ref[0:3] = F_int_scalar * n_LR
        f_int_ref[6:9] = -F_int_scalar * n_LR

        f_c = G_pinv @ w_obj + N_G @ f_int_ref
        F_left = f_c[0:6]
        F_right = f_c[6:12]
        
        # 4. 笛卡尔接触 wrench → 关节力矩
        # F_left/F_right 是 hand-on-object；机器人侧 object-on-hand = -F。
        # 与 D03 保持一致：tau_i = -J_i^T f_i + g_i。
        tau_left = -J_L.T @ F_left
        tau_right = -J_R.T @ F_right
        
        # 加机械臂自身重力补偿；高加速度搬运还应加入完整 inverse dynamics 前馈。
        tau_left += pin.computeGeneralizedGravity(
            self.model_L, self.data_L, q_left)
        tau_right += pin.computeGeneralizedGravity(
            self.model_R, self.data_R, q_right)
        
        return tau_left, tau_right
    
    def _estimate_internal_force(self, q_left, q_right):
        """教学占位：只能估计几何压缩量，不能作为刚性物体内力闭环。"""
        pin.forwardKinematics(self.model_L, self.data_L, q_left)
        pin.updateFramePlacements(self.model_L, self.data_L)
        pin.forwardKinematics(self.model_R, self.data_R, q_right)
        pin.updateFramePlacements(self.model_R, self.data_R)
        
        p_L = self.data_L.oMf[
            self.model_L.getFrameId("left_panda_hand_tcp")
        ].translation
        p_R = self.data_R.oMf[
            self.model_R.getFrameId("right_panda_hand_tcp")
        ].translation
        
        # 两末端距离偏差 × 虚拟弹簧 = 几何压缩量近似。
        # 对刚性杆/刚性盒，该值不能替代 F/T 传感器或仿真接触 wrench。
        d_actual = np.linalg.norm(p_R - p_L)
        d_nominal = 0.30  # 杆长 30cm
        return 500.0 * (d_nominal - d_actual)  # 虚拟弹簧
```

上面的 `w_ff` 是教学版前馈：平移部分补偿物体重量和期望加速度，转动部分补偿期望角加速度。若物体姿态变化明显，应把物体坐标系惯量 $I_o$ 旋到 world frame：$I_w = R_{wo} I_o R_{wo}^T$；高速转动还要加入 $\omega \times I\omega$ 项。机械臂自身的重力补偿由 `computeGeneralizedGravity()` 处理，两者不能互相替代。

### 参数调节指南

| 参数 | 推荐范围 | 过大后果 | 过小后果 |
|------|---------|---------|---------|
| K_obj (平移) | 300-800 N/m | 物体运动过冲/振荡 | 跟踪误差大 |
| K_obj (旋转) | 10-50 Nm/rad | 夹爪扭转过冲 | 物体旋转松弛 |
| D_obj | 2 ζ √(K·M_eff) | 过阻尼（慢） | 欠阻尼（振荡） |
| f_int_des | 20-60 N | 物体变形/夹爪过载 | 物体滑落 |
| K_int | 0.2-1.0 | 内力振荡、修正频繁打到限幅 | 内力收敛慢 |
| f_int_feedback_limit | 5-20 N | 初始夹持冲击大 | 抗扰能力弱 |

### Object Impedance 参数调优实战流程

**三阶段调参法**（从安全到性能）：

```
Phase 1: 安全验证（低增益，不会损坏硬件）
  K_obj = [100, 100, 100, 5, 5, 5]    # 软弹簧
  D_obj = [30, 30, 30, 2, 2, 2]       # 高阻尼
  f_int_des = 10 N                      # 低内力
  → 验证：两臂能跟踪目标但慢，杆可能滑落——正常，不要紧

Phase 2: 内力调节（保证抓持稳定）
  保持 K_obj / D_obj 不变
  逐步增加 f_int_des: 10 → 20 → 30 → 40 N
  每次增 10 N，跑 5 次搬运，记录成功率
  → 目标：找到"刚好不掉"的最小内力 + 安全裕度 ×1.5
  → 典型值：铝杆(μ≈0.5) f_int_des=25-40 N

Phase 3: 运动性能调优（提速 + 减振）
  内力固定在 Phase 2 的结果
  逐步增加 K_obj 平移: 100 → 200 → 300 → 500
  每次增加后检查：是否振荡？
    是 → 增加 D_obj（D = 2ζ√(K·M_eff)，ζ=0.7-1.0）
    否 → 继续增加 K
  → 目标：跟踪误差 < 1cm 且无明显振荡
```

**参数与物体属性的关系**：

| 物体 | 质量 | 摩擦系数 | 推荐 f_int | 推荐 K_obj(平移) | 备注 |
|------|------|---------|-----------|-----------------|------|
| 铝杆 30cm | 0.2 kg | 0.5 | 25-40 N | 300-500 | 轻且滑 |
| 木板 40cm | 1.0 kg | 0.7 | 20-30 N | 400-600 | 重但粗糙 |
| 塑料箱 | 0.5 kg | 0.3 | 40-60 N | 300-500 | 滑，需大内力 |
| 玻璃杯 | 0.15 kg | 0.4 | 10-15 N | 100-200 | 易碎，低增益 |

**调参可视化脚本**：

```python
# scripts/plot_impedance_metrics.py
import matplotlib.pyplot as plt
import numpy as np

def plot_impedance_tuning(log_data):
    """可视化阻抗控制调参过程"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    t = log_data['time']
    
    # 1. 物体位姿跟踪误差
    axes[0, 0].plot(t, log_data['pos_error'], 'b-', label='Position (m)')
    axes[0, 0].plot(t, log_data['ori_error'], 'r-', label='Orientation (rad)')
    axes[0, 0].axhline(y=0.01, color='b', linestyle='--', alpha=0.5)
    axes[0, 0].axhline(y=np.deg2rad(5), color='r', linestyle='--', alpha=0.5)
    axes[0, 0].set_ylabel('Error')
    axes[0, 0].set_title('Object Tracking Error')
    axes[0, 0].legend()
    
    # 2. 内力跟踪
    axes[0, 1].plot(t, log_data['f_int_actual'], 'g-', label='Actual')
    axes[0, 1].axhline(y=log_data['f_int_des'], color='k', 
                        linestyle='--', label='Desired')
    axes[0, 1].set_ylabel('Force (N)')
    axes[0, 1].set_title('Internal Force')
    axes[0, 1].legend()
    
    # 3. 关节力矩
    for i in range(7):
        axes[1, 0].plot(t, log_data['tau_left'][:, i], alpha=0.7)
    axes[1, 0].set_ylabel('Torque (Nm)')
    axes[1, 0].set_title('Left Arm Joint Torques')
    
    # 4. 杆姿态
    axes[1, 1].plot(t, np.rad2deg(log_data['rod_angle']), 'm-')
    axes[1, 1].axhline(y=5, color='k', linestyle='--', alpha=0.5)
    axes[1, 1].axhline(y=-5, color='k', linestyle='--', alpha=0.5)
    axes[1, 1].set_ylabel('Angle (deg)')
    axes[1, 1].set_title('Rod Tilt Angle')
    
    for ax in axes.flat:
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('impedance_tuning.png', dpi=150)
    plt.show()
```

### 反事实推理——不做内力控制

> 如果只控物体运动，不控内力会怎样？
>
> 两臂各自用阻抗控制跟踪自己的末端目标。但目标只规定了"物体去哪"，没有规定"两臂之间挤多紧"。结果：(1) 如果目标位姿让两末端距离稍大于杆长→内力为零→杆滑落；(2) 如果目标位姿让两末端距离稍小于杆长→内力被动产生（由杆的刚性约束力决定）→不可控、不可预测。
>
> 内力控制的价值正是**主动控制抓持力**，使其不依赖于被动约束力，而是由控制器显式维持。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：内力方向在物体旋转后不更新
   错误做法：内力始终沿世界坐标系 x 轴
   现象：物体旋转 90 度后，内力方向不再沿抓持方向 → 物体滑落
   正确做法：内力方向应定义在物体坐标系中，随物体姿态实时旋转

💡 概念误区：认为"内力越大越安全"
   新手想法："设 f_int = 200N 就绝对不会掉"
   实际上：过大的内力导致：(1) 夹爪电机过载；(2) 软物体变形或损坏；
   (3) 关节力矩接近限位，留给运动控制的余量不足。
   正确做法：内力 = 物体重量 × g / (2 × 摩擦系数) × 安全系数(1.5-2.0)
```

### 练习

1. **[编程]** 实现 Object Impedance 双臂搬运。在 MuJoCo 中共持 30cm 杆搬运 50cm。调参使杆姿态偏差 < 5 度。
2. **[编程]** 对比不同内力水平（20N、40N、80N）对搬运稳定性的影响。记录杆的姿态偏差和关节力矩峰值。

---

## D10.6 遥操作数据采集（Day 7-8）⭐⭐

### 遥操作节点实现

```python
# mini_dualarm/src/teleop_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np

class TeleopNode(Node):
    """GELLO 式关节镜像遥操作 + 数据录制"""
    
    def __init__(self):
        super().__init__('teleop_node')
        
        # 订阅 leader 关节位置
        self.leader_sub = self.create_subscription(
            JointState, '/leader/joint_states',
            self.leader_callback, 10)
        
        # 发布候选 follower 目标；Command Arbiter 唯一发布 /joint_commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/teleop/joint_target', 10)
        
        # 模式控制
        self.mode_sub = self.create_subscription(
            String, '/mode_switch', self.mode_callback, 10)
        self.active = False
        
        # 标定偏置（初始化时测量）
        self.joint_offset = np.zeros(14)
        
        self.get_logger().info('Teleop node initialized (inactive)')
        
    def leader_callback(self, msg):
        if not self.active:
            return
        
        leader_q = np.array(msg.position[:14])
        target = leader_q + self.joint_offset
        
        cmd = Float64MultiArray()
        cmd.data = target.tolist()
        self.cmd_pub.publish(cmd)
        
    def mode_callback(self, msg):
        if msg.data == 'teleop':
            self.active = True
            self.get_logger().info('Teleop ACTIVATED')
        else:
            self.active = False
            self.get_logger().info(f'Teleop deactivated (mode: {msg.data})')
```

### 数据采集与存储

Mini-DualArm 录制端先写 ALOHA-HDF5 legacy 文件，训练端通过 D08 的 canonical schema adapter 读取。统一语义如下：

| 语义 | HDF5 写入字段 | 训练/LeRobot canonical key |
|------|---------------|----------------------------|
| 关节状态 | `observations/qpos` | `observation.state` |
| 动作目标 | `action` | `action` |
| 图像 | `observations/images/cam_high` | `observation.images.cam_high` |
| 时间戳 | `timestamps` | `timestamp` |

不要在同一项目中混用 `actions/`、`action`、`observations/qpos`、`observation.state` 作为业务层字段；这些差异应只存在于 adapter 中。

```python
# mini_dualarm/src/data_collector.py

import h5py
import numpy as np
from pathlib import Path

class DataCollector:
    """ALOHA-HDF5 格式数据采集；通过 adapter 转为 LeRobot canonical schema。"""
    
    def __init__(self, output_dir='data/episodes'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.episode_buffer = []
        self.episode_count = len(list(self.output_dir.glob('*.hdf5')))
        
    def add_frame(self, obs_qpos, obs_images, action, timestamp=None):
        """添加一帧数据。timestamp 应为采集时刻的单调时钟值（秒），用于离线时间对齐。"""
        import time
        self.episode_buffer.append({
            'qpos': obs_qpos.copy(),        # (14,)
            'action': action.copy(),          # (14,)
            'cam_high': obs_images.copy(),    # (H, W, 3)
            'timestamp': timestamp if timestamp is not None else time.monotonic(),
        })
        
    def save_episode(self, success=True):
        """保存 episode 到 HDF5"""
        if not self.episode_buffer:
            return None
        
        T = len(self.episode_buffer)
        path = self.output_dir / f'episode_{self.episode_count:04d}.hdf5'
        
        with h5py.File(path, 'w') as f:
            qpos = np.stack([fr['qpos'] for fr in self.episode_buffer])
            actions = np.stack([fr['action'] for fr in self.episode_buffer])
            images = np.stack([fr['cam_high'] for fr in self.episode_buffer])
            
            timestamps = np.array([fr['timestamp'] for fr in self.episode_buffer])
            f.create_dataset('timestamps', data=timestamps)
            f.create_dataset('observations/qpos', data=qpos)
            f.create_dataset('action', data=actions)
            f.create_dataset('observations/images/cam_high',
                           data=images,
                           chunks=(1,) + images.shape[1:],
                           compression='gzip')
            
            f.attrs['num_timesteps'] = T
            f.attrs['success'] = success
            f.attrs['sim'] = True
        
        self.episode_count += 1
        self.episode_buffer = []
        return path

def hdf5_frame_to_lerobot(h5, t):
    """训练/导出时使用的 adapter：HDF5 legacy -> canonical frame。"""
    return {
        "observation.images.cam_high": h5["observations/images/cam_high"][t],
        "observation.state": h5["observations/qpos"][t].astype("float32"),
        "action": h5["action"][t].astype("float32"),
        "timestamp": h5["timestamps"][t],
    }
```

### 练习

1. **[编程]** 实现遥操作数据采集。用键盘模拟 leader 输入（WASD 控制关节 1-2），采集 50 episodes。
2. **[编程]** 对采集的 50 episodes 计算 RMS Jerk，过滤掉 Jerk > 200 的 episodes。统计过滤前后的数量。

---

## D10.7 ACT 策略训练与部署（Day 9-10）⭐⭐⭐

### 训练流程

```python
# scripts/train_act.py
# 使用 LeRobot 框架训练 ACT 策略

from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig

# 配置
config = ACTConfig(
    input_features={
        "observation.images.cam_high": PolicyFeature(
            type=FeatureType.VISUAL, shape=(3, 480, 640)),
        "observation.state": PolicyFeature(
            type=FeatureType.STATE, shape=(14,)),
    },
    output_features={
        "action": PolicyFeature(type=FeatureType.ACTION, shape=(14,)),
    },
    chunk_size=100,
    n_action_steps=100,
    dim_model=512,
    n_heads=8,
    n_encoder_layers=4,
    n_decoder_layers=1,
)

policy = ACTPolicy(config)

# 训练 2000 epochs...
```

### 策略部署节点

```python
# mini_dualarm/src/policy_inference.py

class PolicyInferenceNode(Node):
    """ACT 策略推理 (50 Hz)"""
    
    def __init__(self, model_path):
        super().__init__('policy_inference')
        
        # 加载模型
        self.policy = self._load_model(model_path)
        
        # 订阅观测
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera_high/image_raw', self.image_cb, 10)
        
        # 发布候选动作；Command Arbiter 唯一发布 /joint_commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/policy/action', 10)
        self.mode_sub = self.create_subscription(
            String, '/mode_switch', self.mode_callback, 10)
        
        # 50 Hz 推理循环
        self.timer = self.create_timer(0.02, self.inference_step)
        
        self.action_queue = []
        self.active = False
        self.latest_state = None
        self.latest_image = None

    def mode_callback(self, msg):
        # 策略节点只在 policy 模式发布候选动作；最终控制权仍由 Command Arbiter 决定。
        self.active = (msg.data == 'policy')
    
    def inference_step(self):
        if not self.active or self.latest_state is None:
            return
        
        if self.action_queue:
            action = self.action_queue.pop(0)
        else:
            # 新推理
            obs = self._prepare_observation()
            with torch.no_grad():
                chunk = self.policy.select_action(obs)
            self.action_queue = chunk.squeeze(0).cpu().numpy().tolist()
            action = self.action_queue.pop(0)
        
        cmd = Float64MultiArray()
        cmd.data = list(action)
        self.cmd_pub.publish(cmd)
```

### 练习

1. **[编程]** 用采集的数据训练 ACT（2000 epochs）。部署到仿真，记录 20 次成功率。
2. **[跨章综合题]** 对比三种方法（MoveIt2 / Object Impedance / ACT）在"双臂搬箱到桌对面"任务上的成功率、完成时间、力矩峰值。撰写对比表格和分析。

---

## D10.8 性能对比与评价指标 ⭐⭐

### 评估指标

| 指标 | 计算方法 | 优秀标准 |
|------|---------|---------|
| **成功率** | 成功次数 / 20 × 100% | > 80% |
| **完成时间** | 从启动到物体到达目标 | < 10s |
| **力矩峰值** | max(\|tau\|) 所有关节 | < 80% 限位 |
| **轨迹平滑度** | RMS Jerk (14D) | < 100 rad/s^3 |
| **物体姿态偏差** | max 角度偏差 | < 5 度 |

### 预期结果对比

| 方法 | 成功率 | 完成时间 | 力矩峰值 | 泛化能力 | 适用场景 |
|------|--------|---------|---------|---------|---------|
| MoveIt2 both_arms | ~95% | 5-15s | 低 | 差（需精确模型） | 几何搬运 |
| Object Impedance | ~85% | 3-8s | 中 | 中（参数需调） | 力敏感操作 |
| ACT (50 demos) | ~70-85% | 3-6s | 中 | 好（视觉泛化） | 多变环境 |
| RL (1M steps) | ~60-80% | 2-5s | 高 | 好（域随机化） | 仿真到真机 |

**跨领域类比——自动驾驶三范式**：MoveIt2 类似于基于高精地图的路径规划（全局最优但依赖精确环境模型），Object Impedance 类似于反应式避障（实时响应但可能局部最优），ACT 策略类似于端到端学习（数据驱动、泛化强但需要大量高质量数据）。三种方法在自动驾驶和机器人操作中有相同的优劣势分布——这不是巧合，而是**规划 vs 反应 vs 学习**三种范式的本质特征。

### 性能指标采集与可视化

```python
# scripts/benchmark.py — 自动化性能评估

import numpy as np
import time
import json
from dataclasses import dataclass, asdict
from typing import List, Optional

@dataclass
class TrialResult:
    """单次试验结果"""
    method: str               # "moveit" | "impedance" | "act"
    success: bool
    completion_time: float    # 秒
    max_torque: float         # 所有关节最大力矩 (Nm)
    rms_jerk: float           # 14D RMS Jerk (rad/s^3)
    rod_max_tilt: float       # 杆最大倾斜角 (deg)
    path_length: float        # 末端路径长度 (m)
    
class Benchmark:
    """三方法性能对比评估框架"""
    
    def __init__(self, env, n_trials=20):
        self.env = env
        self.n_trials = n_trials
        self.results: List[TrialResult] = []
    
    def run_trial(self, method, controller):
        """运行单次试验并记录指标"""
        self.env.reset()
        
        log = {'q': [], 'tau': [], 'rod_angle': [], 'ee_pos': []}
        t_start = time.time()
        success = False
        
        try:
            while time.time() - t_start < 15.0:  # 15s 超时
                q = self.env.get_joint_positions()
                tau = controller.step(self.env)
                rod = self.env.get_rod_angle()
                ee = self.env.get_ee_positions()
                
                log['q'].append(q.copy())
                log['tau'].append(tau.copy())
                log['rod_angle'].append(rod)
                log['ee_pos'].append(ee.copy())
                
                self.env.step()
                
                if self.env.check_task_success():
                    success = True
                    break
        except Exception as e:
            print(f"Trial error: {e}")
        
        t_elapsed = time.time() - t_start
        q_arr = np.array(log['q'])
        tau_arr = np.array(log['tau'])
        
        result = TrialResult(
            method=method,
            success=success,
            completion_time=t_elapsed,
            max_torque=np.max(np.abs(tau_arr)),
            rms_jerk=self._compute_rms_jerk(q_arr, dt=0.001),
            rod_max_tilt=np.max(np.abs(log['rod_angle'])),
            path_length=self._compute_path_length(log['ee_pos']),
        )
        self.results.append(result)
        return result
    
    def run_all(self, controllers_dict):
        """对所有方法运行 n_trials 次"""
        for method, ctrl in controllers_dict.items():
            print(f"\n=== Running {method} ({self.n_trials} trials) ===")
            for i in range(self.n_trials):
                r = self.run_trial(method, ctrl)
                status = "OK" if r.success else "FAIL"
                print(f"  Trial {i+1}: {status} ({r.completion_time:.1f}s)")
    
    def summary(self):
        """生成对比表格"""
        methods = set(r.method for r in self.results)
        print("\n" + "="*80)
        print(f"{'Method':<15} {'Success%':>10} {'Time(s)':>10} "
              f"{'MaxTau':>10} {'Jerk':>10} {'Tilt(deg)':>10}")
        print("-"*80)
        for m in sorted(methods):
            trials = [r for r in self.results if r.method == m]
            sr = sum(1 for r in trials if r.success) / len(trials) * 100
            t_avg = np.mean([r.completion_time for r in trials if r.success])
            tau_max = np.mean([r.max_torque for r in trials])
            jerk_avg = np.mean([r.rms_jerk for r in trials])
            tilt_max = np.mean([r.rod_max_tilt for r in trials])
            print(f"{m:<15} {sr:>9.0f}% {t_avg:>10.1f} "
                  f"{tau_max:>10.1f} {jerk_avg:>10.0f} {tilt_max:>10.1f}")
    
    def save(self, path='benchmark_results.json'):
        with open(path, 'w') as f:
            json.dump([asdict(r) for r in self.results], f, indent=2)
    
    @staticmethod
    def _compute_rms_jerk(q, dt):
        if len(q) < 4:
            return 0.0
        vel = np.diff(q, axis=0) / dt
        acc = np.diff(vel, axis=0) / dt
        jerk = np.diff(acc, axis=0) / dt
        return float(np.sqrt(np.mean(np.sum(jerk**2, axis=1))))
    
    @staticmethod
    def _compute_path_length(ee_pos_list):
        if len(ee_pos_list) < 2:
            return 0.0
        positions = np.array(ee_pos_list)
        diffs = np.diff(positions, axis=0)
        return float(np.sum(np.linalg.norm(diffs, axis=1)))
```

```python
# scripts/plot_metrics.py — 性能可视化
import matplotlib.pyplot as plt
import json
import numpy as np

def plot_comparison(results_path='benchmark_results.json'):
    with open(results_path) as f:
        data = json.load(f)
    
    methods = sorted(set(r['method'] for r in data))
    colors = {'moveit': '#2196F3', 'impedance': '#FF9800', 'act': '#4CAF50'}
    
    fig, axes = plt.subplots(1, 4, figsize=(20, 5))
    
    # 1. 成功率柱状图
    success_rates = []
    for m in methods:
        trials = [r for r in data if r['method'] == m]
        sr = sum(1 for r in trials if r['success']) / len(trials) * 100
        success_rates.append(sr)
    axes[0].bar(methods, success_rates, color=[colors[m] for m in methods])
    axes[0].set_ylabel('Success Rate (%)')
    axes[0].set_ylim(0, 105)
    axes[0].set_title('Task Success Rate')
    
    # 2. 完成时间箱线图
    times = [[r['completion_time'] for r in data 
              if r['method'] == m and r['success']] for m in methods]
    bp = axes[1].boxplot(times, labels=methods, patch_artist=True)
    for patch, m in zip(bp['boxes'], methods):
        patch.set_facecolor(colors[m])
    axes[1].set_ylabel('Time (s)')
    axes[1].set_title('Completion Time')
    
    # 3. 力矩峰值
    torques = [[r['max_torque'] for r in data 
                if r['method'] == m] for m in methods]
    bp2 = axes[2].boxplot(torques, labels=methods, patch_artist=True)
    for patch, m in zip(bp2['boxes'], methods):
        patch.set_facecolor(colors[m])
    axes[2].set_ylabel('Max Torque (Nm)')
    axes[2].set_title('Peak Joint Torque')
    
    # 4. 轨迹平滑度
    jerks = [[r['rms_jerk'] for r in data 
              if r['method'] == m] for m in methods]
    bp3 = axes[3].boxplot(jerks, labels=methods, patch_artist=True)
    for patch, m in zip(bp3['boxes'], methods):
        patch.set_facecolor(colors[m])
    axes[3].set_ylabel('RMS Jerk (rad/s^3)')
    axes[3].set_title('Trajectory Smoothness')
    
    plt.tight_layout()
    plt.savefig('method_comparison.png', dpi=150)
    plt.show()
```

### 从仿真到实机的详细迁移清单

| # | 检查项 | 仿真状态 | 真机变更 | 验证方法 | 风险等级 |
|---|--------|---------|---------|---------|---------|
| 1 | URDF hardware plugin | `mujoco_ros2_control` | `franka_hardware/FrankaHardwareInterface` | `ros2 control list_hardware_interfaces` | 高 |
| 2 | 控制器配置 | `ros2_controllers.yaml` | `ros2_controllers_real.yaml` | `ros2 control list_controllers` | 高 |
| 3 | 关节限位 | URDF `<limit>` | 确认与真机一致 | 手动逐关节测试极限 | 高 |
| 4 | 通信延迟 | ~0 ms (同进程) | 5-20 ms (EtherCAT/USB) | `ros2 topic hz` + 时间戳差 | 中 |
| 5 | 重力补偿 | 仿真自动 | 真机驱动内置 | 释放关节看是否下垂 | 中 |
| 6 | PD 增益 | 仿真标称值 | 可能需降低 20-30% | 阶跃响应测试 | 中 |
| 7 | 碰撞检测 | 几何精确 | mesh 可能简化 | RViz 对比 collision mesh | 低 |
| 8 | 相机标定 | 理想内参 | 需实际标定 | checkerboard 标定 | 中 |
| 9 | 夹爪控制 | 理想抓取 | 需调整力/宽度 | 实物抓取测试 | 中 |
| 10 | 速度限制 | 无 | 初次 20% 速度 | `setMaxVelocityScalingFactor(0.2)` | 高 |
| 11 | 急停 | 无需 | 确认 E-stop 可用 | 手动触发急停 | 安全 |
| 12 | ACT 归一化 | stats.json (仿真) | 需用真机数据重新统计 | 对比均值/方差 | 高 |

**迁移五步法**：

```
Step 1: 低速单臂 (10% 速度, 只启动一臂)
  → 验证: 关节跟踪, 重力补偿, E-stop
  
Step 2: 低速双臂 (10% 速度, 两臂同时)
  → 验证: 同步精度, 无碰撞, 控制器切换
  
Step 3: 中速双臂 (30% 速度)
  → 验证: 动态跟踪误差, 力矩未饱和
  
Step 4: 遥操作测试 (手动操作, 不采集数据)
  → 验证: 延迟可接受, 工作空间足够
  
Step 5: 全速 + 数据采集 + 策略部署
  → 验证: 全部指标达到仿真 80%+
```

### 常见集成问题 Troubleshooting（50+ 条）

**A. 环境与硬件类（15 条）**

| # | 问题 | 症状 | 根因 | 解决 |
|---|------|------|------|------|
| 1 | MuJoCo 加载 URDF 失败 | "unknown mesh extension" | DAE mesh | 转换为 STL/OBJ |
| 2 | MuJoCo actuator 为空 | `model.nu == 0` | URDF 无 actuator 等价 | `--add-actuators` |
| 3 | 仿真爆炸 | NaN / 物体飞出 | 惯性参数错误 / dt 过大 | 检查惯性 + 减小 dt |
| 4 | 关节顺序错误 | 左臂指令控制右臂 | MuJoCo 重排关节 | `mj_id2name` 确认映射 |
| 5 | 仿真速度太慢 | < 0.1x 实时 | 碰撞 mesh 过精细 | 简化 collision mesh |
| 6 | 物体穿模 | 杆穿过桌面 | 接触刚度不足 | 增大 `solimp` / `solref` |
| 7 | USB 设备丢失 | leader 断连 | USB 供电不足 | 使用有源 USB Hub |
| 8 | EtherCAT 超时 | 真机无响应 | 网线/网卡问题 | 检查 RT 内核 + 网卡配置 |
| 9 | GPU 显存不足 | ACT 推理 OOM | batch size / 图像分辨率 | 降低分辨率到 240x320 |
| 10 | ROS2 DDS 通信丢帧 | 话题延迟增大 | QoS 配置不匹配 | 统一 QoS profile |
| 11 | MuJoCo viewer 不显示 | 黑屏 / 段错误 | OpenGL 驱动 | 安装 NVIDIA/Mesa 驱动 |
| 12 | Gazebo 加载超慢 | > 30s 启动 | mesh 文件过大 | 简化 visual mesh LOD |
| 13 | 真机关节抖动 | 高频振荡 | PD 增益过高 | 降低 Kp 20-30% |
| 14 | 编码器分辨率 | 关节位置跳变 | 低分辨率编码器 | 加卡尔曼滤波 |
| 15 | 电源不足 | 关节力矩饱和 | PSU 容量不够 | 升级电源 + 限制加速度 |

**B. MoveIt2 / 规划类（15 条）**

| # | 问题 | 症状 | 根因 | 解决 |
|---|------|------|------|------|
| 16 | both_arms 规划超时 | 10s 内无解 | 14D 空间太大 | BIT* + ACM 优化 + fallback |
| 17 | 找不到 Planning Group | "Group not found" | SRDF 未加载 | 检查 launch 参数路径 |
| 18 | IK 无解 | "No IK solution" | 目标不可达 | 可视化确认工作空间 |
| 19 | 碰撞误报 | "In collision" 但看不到 | ACM 缺失 | `disable_collisions` 相邻对 |
| 20 | 碰撞漏报 | 两臂碰撞但规划通过 | ACM 过度优化 | 恢复跨臂碰撞检查 |
| 21 | 轨迹执行失败 | "Invalid trajectory" | 关节名不匹配 | 对比 controller joints vs URDF |
| 22 | 执行速度过快 | 真机急停 | velocity scaling 太高 | `setMaxVelocityScalingFactor(0.3)` |
| 23 | MTC Merger 失败 | "No solutions" | 子 stage 合并后碰撞 | 增大 approach 距离 |
| 24 | 规划结果不稳定 | 同目标不同路径 | 随机采样 | 增加 planning attempts |
| 25 | Controller 冲突 | "Controller already active" | 多控制器竞争 | 先 deactivate 再 activate |
| 26 | TF 超时 | "Transform timeout" | 时钟不同步 | 检查 use_sim_time 参数 |
| 27 | OMPL 崩溃 | Segfault | 状态空间配置错误 | 检查 projection_evaluator |
| 28 | 路径不平滑 | 关节角跳变 | 时间参数化失败 | 加 TOTG / ISP 后处理 |
| 29 | Attached object 漏跟 | 物体不随手移动 | 未 attach 到正确 link | 检查 attach link 名 |
| 30 | Planning Scene 过时 | 碰撞检测用旧状态 | Monitor 未启动 | 启动 PlanningSceneMonitor |

**C. 遥操作 / 数据采集类（10 条）**

| # | 问题 | 症状 | 根因 | 解决 |
|---|------|------|------|------|
| 31 | leader-follower 偏移 | 初始不对齐 | 未标定 offset | 启动时记录差值 |
| 32 | 遥操作延迟大 | > 200ms | Python GIL / 队列堆积 | QoS depth=1 + C++ |
| 33 | 数据时间不同步 | 图像与关节偏差 > 50ms | sync slop 过小 | 增大 slop 到 20ms |
| 34 | HDF5 文件损坏 | 读取报错 | 异常退出未 close | try/finally 确保 close |
| 35 | LeRobot 格式错误 | 训练时字段缺失 | features 定义不匹配 | 对照 LeRobot schema |
| 36 | 录制数据过大 | 50 ep > 100 GB | 原始图像未压缩 | 用 mp4 + parquet |
| 37 | 操作者疲劳 | 后期数据质量差 | 长时间无休息 | 每 20 ep 休息 5min |
| 38 | 夹爪状态不录 | action 缺 gripper | subscriber 缺失 | 加 gripper 话题 |
| 39 | action 定义错误 | 策略学到"保持不动" | action[t]=qpos[t] | action[t]=qpos[t+1] |
| 40 | 数据归一化不一致 | 策略输出异常 | stats.json 版本不同 | 训练和部署用同一 stats |

**D. ACT 策略 / 学习类（12 条）**

| # | 问题 | 症状 | 根因 | 解决 |
|---|------|------|------|------|
| 41 | ACT 输出全零 | 机器人不动 | 模型未训练 / 路径错误 | 检查模型文件 + 权重 |
| 42 | ACT 输出超限 | 关节超极限 | 归一化错误 | 加 clamp 到关节限位 |
| 43 | 策略犹豫不决 | 在两动作间切换 | 多模态混淆 | 增加 chunk_size + KL loss |
| 44 | 训练 loss 不降 | > 1000 epoch 不收敛 | 学习率过高/数据问题 | lr=1e-5 + 检查数据 |
| 45 | 策略只在起始位有效 | 偏离后失败 | 数据分布太窄 | 更多样的初始条件 |
| 46 | GPU 推理太慢 | > 50ms / step | 模型太大 | 减小 dim_model + ONNX |
| 47 | action chunk 过期 | 动作滞后 | queue 未清空 | 每次新推理清空旧 chunk |
| 48 | 图像预处理不一致 | 策略行为异常 | 训练/推理 resize 不同 | 统一 transforms |
| 49 | co-training 无效 | 加入仿真数据后反而变差 | 仿真数据质量太低 | 提升仿真保真度 |
| 50 | 模式切换后跳变 | 切到 ACT 后臂跳动 | 未从当前位置开始 | 切换时用当前 qpos 初始化 |
| 51 | 多相机图像混淆 | 策略行为随机 | 相机顺序不一致 | 固定相机名→索引映射 |
| 52 | 训练过拟合 | 训练成功率高但测试低 | 数据太少 | 数据增强 + dropout |

---

## D10.9 开源参考项目详细对比 ⭐⭐

| 项目 | Stars | 与本项目关系 | 精读文件 | 优势 | 局限 |
|------|-------|------------|---------|------|------|
| tonyzhaozh/aloha | ~1200 | 遥操作 + ACT 原始实现 | `robot_utils.py`, `record_episodes.py` | 端到端验证，社区活跃 | Dynamixel 限定，6-DOF |
| huggingface/lerobot | ~8000+ | 数据格式 + 训练框架 | `policies/act/`, `datasets/` | 标准化数据格式，多策略 | 实时部署支持弱 |
| moveit/moveit2 | ~2500 | 运动规划框架 | `dual_arm_panda_moveit_config/` | 工业标准，MTC 编排 | 力控集成弱 |
| ros-controls/ros2_control | ~4500 | 硬件抽象层 | `joint_trajectory_controller/` | 硬件抽象完善 | 多臂同步文档少 |
| google-deepmind/mujoco | ~8000+ | 物理仿真核心 | `python/mujoco/viewer.py` | 极快接触仿真 | ROS2 集成需自行开发 |
| ARISE-Initiative/robosuite | ~1500 | 双臂 RL 基准 | `environments/two_arm_lift.py` | 标准化 RL 环境 | 不支持 ROS2 |
| wuphilipp/gello_software | ~300 | 低成本 leader 方案 | `agents/gello_agent.py` | $300 成本 | 无力反馈 |
| real-stanford/umi | ~1000+ | 无机器人数据采集 | `franka_interpolation_controller.py` | 突破性采集范式 | 需后处理标定 |

**选型决策矩阵**：

| 你的需求 | 推荐方案 | 理由 |
|---------|---------|------|
| 快速验证 ACT | ALOHA + LeRobot | 端到端最快 |
| 低成本硬件 | GELLO leader + ViperX follower | $1000 以内 |
| 工业精度双臂 | MoveIt2 + Franka | 成熟稳定 |
| 大规模 RL 训练 | MuJoCo + Isaac Lab | GPU 并行 |
| 通用数据格式 | LeRobot parquet + mp4 | 社区标准 |

**项目成熟度评估**：

| 项目 | 文档质量 | 社区活跃度 | 复现难度 | 维护状态 (2025) |
|------|---------|-----------|---------|----------------|
| ALOHA | 中（README + 论文） | 高（Issues 活跃） | 中（需特定硬件） | 活跃 |
| LeRobot | 高（完整教程） | 极高（HuggingFace 推动） | 低（pip install） | 非常活跃 |
| MoveIt2 | 高（官方教程） | 高 | 中（ROS2 环境配置） | 活跃 |
| robosuite | 中 | 中 | 低（pip install） | 维护但更新慢 |
| GELLO | 低（README 简短） | 低 | 高（需 3D 打印 + 标定） | 低频更新 |
| UMI | 中 | 中 | 高（需 GoPro + ORB-SLAM3） | 活跃 |

### 练习

1. **[编程]** 运行 `Benchmark` 脚本对比三种方法。生成 `method_comparison.png` 并撰写 500 字的对比分析。
2. **[编程]** 按迁移清单的前三步，将 Mini-DualArm 从 MuJoCo 迁移到 Gazebo Harmonic。记录每步遇到的问题和解决方案。
3. **[跨章综合题]** 结合 D03（Object Impedance 理论）、D08（数据采集）、D10（系统集成）：设计一个实验方案来验证"Object Impedance 控制 vs ACT 策略在**未见过的物体**上的泛化能力"。你需要定义：(a) 训练物体集和测试物体集；(b) 评估指标；(c) 对照实验设计（控制变量）。预测哪种方法泛化更好并给出理由。

---

## D10.10 系统调试经验总结 ⭐⭐

### 黄金法则

经过 D01-D10 的完整学习和实践，以下是双臂系统集成的五条黄金法则：

1. **先仿真后真机**：没有通过仿真验证的代码不应该在真机上运行——这不是建议，是安全规则
2. **先单臂后双臂**：单臂 bug 在双臂中会被放大两倍，而调试难度增加四倍
3. **先位控后力控**：阻抗控制中的 bug 可能导致大力矩输出，先确认位置控制安全
4. **先低速后高速**：从 10% 速度开始，每次提速不超过 2 倍
5. **先手动后自主**：遥操作验证系统可用性后，再引入策略自主执行

**反事实推理——如果违反黄金法则**：一个团队直接在真机上调试双臂 ACT 策略（同时违反法则 1、2、5）。策略输出了一个超限关节角，两臂高速相撞——左臂第六关节齿轮箱损坏，维修费 $3000，停机两周。如果先在仿真中跑，这个 bug 会在零成本环境中被发现——**1 小时的仿真验证能避免 3000 美元的损失**。

### 调试的正确顺序

系统集成的调试**必须自底向上**——违反这个顺序是浪费时间的根源。

```
Level 0: 硬件/仿真环境
  → 关节能读能写？通信稳定？
  → 不过 → 不要动上层代码

Level 1: 单臂控制
  → PD 跟踪正常？重力补偿正确？
  → 不过 → 不要尝试双臂

Level 2: 双臂协调
  → 同步精度满足？无碰撞？
  → 不过 → 不要加遥操作

Level 3: 遥操作
  → 延迟可接受？映射正确？
  → 不过 → 不要采集训练数据

Level 4: 数据管线
  → 格式正确？时间同步？
  → 不过 → 不要训练策略

Level 5: 策略训练 + 部署
  → loss 下降？推理正常？
  → 最后才调这里
```

> **本质洞察**：系统集成中 90% 的 bug 在 Level 0-2，但新手 90% 的调试时间花在 Level 4-5。原因是 Level 0-2 的 bug 症状（延迟、抖动、偶发失败）容易被误诊为 Level 4-5 的问题（"策略没训好"、"数据不够"）。**先确保底层完美，再向上排查**。

### 常见误诊案例

| 表面症状 | 新手诊断 | 实际根因 | 正确排查 |
|---------|---------|---------|---------|
| ACT 成功率低 | "需要更多数据" | MuJoCo 接触参数错误 | 先手动控制验证环境 |
| 双臂搬运失败 | "阻抗增益不对" | 关节 5-6 PD 增益不匹配 | 先单关节阶跃响应 |
| 遥操作卡顿 | "网络延迟" | Python GIL 阻塞 | `htop` 看 CPU 占用 |
| 规划总超时 | "OMPL 太慢" | ACM 未优化，检查 120 对 | `ros2 topic echo /planning_scene` |
| 模式切换崩溃 | "lifecycle bug" | 残余指令竞争 | 切换前发零速 + 等待 |

### 性能基线参考

完成 Mini-DualArm 后，你的系统应达到以下基线（基于 MuJoCo 仿真，双 Franka Panda）：

| 指标 | 基线值 | 优秀值 | 说明 |
|------|--------|--------|------|
| MoveIt2 both_arms 规划时间 | < 5s | < 2s | BIT* + ACM 优化 |
| 单臂 7D 规划时间 | < 0.5s | < 0.2s | RRT-Connect |
| Object Impedance 搬运成功率 | > 80% | > 95% | 50cm 距离 |
| 遥操作端到端延迟 | < 100ms | < 30ms | 从 leader → follower 运动 |
| ACT 50-demo 成功率 | > 60% | > 85% | pick-and-place 任务 |
| 数据采集速度 | > 3 ep/min | > 5 ep/min | 含操作 + 保存 |
| 控制循环频率 | 1 kHz | 1 kHz | ros2_control |
| ACT 推理频率 | > 30 Hz | > 50 Hz | RTX 3090 |

如果你的系统显著低于基线值，优先检查底层（环境/硬件），而非调上层参数。**基线值的意义不是"目标"，而是"健康指标"**——低于基线说明某处有系统性问题。

> **本质洞察**：Mini-DualArm 项目的核心价值不是最终的成功率数字——而是你在集成过程中获得的**系统思维**：理解模块间如何交互、时序如何协调、错误如何传播。这种思维方式比任何单一算法的知识都更有长期价值。

> **跨领域类比——集成调试与医学诊断**：Mini-DualArm 的分层调试过程（Level 0 → Level 5）与临床医学的诊断流程惊人地相似。当患者（系统）表现出症状（ACT 成功率低），新手医生（工程师）倾向于直接考虑复杂疾病（"策略需要更多数据"），而有经验的医生会先检查生命体征（底层健康）：体温/血压/心率（关节通信/控制频率/延迟）。90% 的"疑难杂症"最终被追溯到基础问题（感染→抗生素 vs 环境参数错误→修正参数）。**先排除常见病因再考虑罕见病因**——这在系统调试和医学诊断中是同一条黄金法则。

---

## D10.11 Docker 部署与 ROS2 版本兼容性 ⭐⭐

### ROS2 Iron/Jazzy/Rolling 兼容性 ⭐⭐

Mini-DualArm 项目默认基于 ROS2 Humble（LTS，2027 年 EOL）。但随着 ROS2 版本演进，以下兼容性问题需要关注：

| 组件 | Humble | Iron | Jazzy | 注意事项 |
|------|--------|------|-------|---------|
| MoveIt2 | 2.5.x | 2.7.x | 2.9.x | Jazzy 引入 Parallel Planning API |
| ros2_control | 2.x | 3.x | 4.x | 3.x 重构了 HardwareInterface 生命周期 |
| MuJoCo plugin | mujoco_ros2_control 0.3 | 0.4 | 0.5 | 0.4+ 支持多实例 |
| Gazebo | Fortress | Harmonic | Harmonic | Iron 开始切换到 Gazebo Harmonic |
| nav2 (移动底盘) | 1.1.x | 1.2.x | 1.3.x | 仅 Mobile ALOHA 需要 |

**关键迁移注意事项**：

1. **ros2_control 3.x 的 Breaking Change**：`HardwareInterface` 的 `read()` 和 `write()` 方法签名从无参变为接受 `rclcpp::Time` 和 `rclcpp::Duration` 参数。所有自定义硬件接口需要修改
2. **MoveIt2 Jazzy 的新 API**：`MoveGroupInterface` 的 `plan()` 返回类型从 `MoveItErrorCode` 改为 `std::pair<MoveItErrorCode, RobotTrajectory>`
3. **TF2 API 调整**：Jazzy 中部分 TF2 API 头文件和接口有调整，详见 ROS 2 迁移指南

### Docker 部署最佳实践 ⭐⭐

Docker 容器化是确保 Mini-DualArm 环境可复现的最可靠方式——避免"在我的机器上能跑"问题。

**推荐的 Dockerfile 结构**：

```dockerfile
# 基础镜像：ROS2 Humble + CUDA (ACT 推理需要 GPU)
FROM ros:humble-perception-jammy AS base

# 系统依赖
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo
RUN pip3 install mujoco==3.1.0 mujoco-python-viewer

# ACT 依赖
RUN pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# 工作空间
WORKDIR /ros2_ws
COPY src/ src/
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# GPU 支持入口
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics
```

**Docker Compose 双服务架构**（推荐将仿真和控制分离）：

```yaml
# docker-compose.yml
services:
  sim:
    build: .
    command: ros2 launch mini_dualarm sim.launch.py
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix  # GUI 支持
    environment:
      - DISPLAY=${DISPLAY}
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
  
  control:
    build: .
    command: ros2 launch mini_dualarm control.launch.py
    depends_on:
      - sim
    network_mode: "host"  # 低延迟 ROS2 通信
```

**工程建议**：(1) 将 `colcon build` 的结果缓存为 Docker layer（build 层和 runtime 层分离）；(2) 使用 `--network=host` 避免 DDS 在容器间发现节点的问题；(3) GPU 渲染（MuJoCo/Gazebo）需要 `nvidia-container-toolkit`。

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 综合的前置章节 |
|--------|---------|------|-------------|
| 系统架构 | 三模式统一设计、微服务类比 | ⭐⭐ | D01-D09 全部 |
| 环境搭建 | MuJoCo + ros2_control 桥接 | ⭐⭐ | D09.7, P02 |
| MoveIt2 集成 | 双臂配置验证与规划测试 | ⭐⭐ | D09.1-D09.6 |
| Object Impedance | 双臂共持搬运、内力控制 | ⭐⭐⭐ | D03 力控 |
| 遥操作采集 | GELLO 式镜像 + LeRobot 录制 | ⭐⭐ | D08 |
| ACT 部署 | 训练 → 推理节点 → 部署 | ⭐⭐⭐ | D04, D08 |
| 性能对比 | 三方法定量评估框架 | ⭐⭐ | — |
| Docker/版本兼容 | Humble→Jazzy 迁移、Docker Compose 部署 | ⭐⭐ | — |

## 累积项目：本章是终点

**Mini-DualArm 项目完成**：

```
D01-D04: 双臂理论（运动学/规划/力控/学习） ✓
D05-D07: 遥操作理论（无源性/波变量/TDPA） ✓
D08: 运动映射与数据采集 ✓
D09: MoveIt2 双臂系统集成 ✓
D10: 综合实战 ✓
  ├─ MuJoCo 双 Franka 仿真环境 ✓
  ├─ ros2_control + MoveIt2 全套配置 ✓
  ├─ Object Impedance 双臂搬运 ✓
  ├─ GELLO 式遥操作 + 数据采集 (50 eps) ✓
  ├─ ACT 策略训练 + 部署 ✓
  └─ 三方法性能对比报告 ✓
```

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Zhao et al. (2023) "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" | ⭐⭐ | ALOHA/ACT 原论文——本项目的直接灵感来源 |
| Fu et al. (2024) "Mobile ALOHA" | ⭐⭐ | co-training 方法扩展 |
| Cadena et al. (2025) "LeRobot: State-of-the-art Machine Learning for Real-World Robotics" | ⭐⭐ | 数据 + 训练框架 |
| robosuite TwoArmLift documentation | ⭐⭐⭐ | 双臂 RL 环境的标准参考 |
| Siciliano & Khatib (2016) "Springer Handbook of Robotics", Ch. 30 Dual-Arm Manipulation | ⭐⭐⭐⭐ | 双臂协作的经典理论 |

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MuJoCo 仿真爆炸 | 惯性参数错误 / timestep 过大 | 1.检查 URDF 惯性 2.减小 timestep 3.检查初始构型无自碰撞 | D10.3 |
| 遥操作延迟大 | USB 带宽 / Python GIL / 话题队列 | 1.测量 round-trip 延迟 2.减少发布频率 3.设 QoS depth=1 | D10.6, D08 |
| ACT 策略不动 | 输出全零 / 归一化错误 | 1.打印策略原始输出 2.检查 stats.json 3.检查输入图像预处理 | D10.7, D04 |
| 双臂搬运物体掉落 | 内力不足 / 摩擦低 | 1.增加 f_int 2.检查 MuJoCo friction 3.检查夹爪闭合 | D10.5, D03 |
| 模式切换后异常 | 指令残留 / 控制器冲突 | 1.切换时发送零速指令 2.等待 500ms 过渡 3.检查节点 lifecycle | D10.1 |

---
