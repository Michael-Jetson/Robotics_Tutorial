> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# D09 双臂 MoveIt2 / ros2_control 系统集成

> **本章定位**：本章是双臂协调与遥操作系列（D01-D10，24 周）Part 3 的第一章。从单臂 MoveIt2 配置（M14）出发，系统讲解双臂 URDF/SRDF 建模、MoveIt2 双臂 Planning Group 配置、同步/异步规划执行、碰撞矩阵优化、MTC 双臂任务流、Gazebo/Isaac Sim 双臂仿真环境搭建，以及 ros2_control 双臂硬件驱动集成。本章是 D10（综合实战）的直接前置——D10 的所有系统功能都建立在本章的配置之上。
>
> **适用范围**：双臂 MoveIt2 配置对任何双臂系统均适用（双 Franka、双 UR5、异构双臂）。ros2_control 双臂同步策略也适用于多臂系统（如四臂协作）。
>
> **前置依赖**：M14（MoveIt2 + MTC 单臂配置）——单臂 MoveIt2 全流程；D01-D02（双臂运动学/规划理论）——理解协调运动约束；P01（URDF/Xacro 建模）——Xacro 宏系统和参数化设计
>
> **下游章节**：D10（综合实战 Mini-DualArm）、D08（遥操作数据采集的 ROS2 集成）
>
> **建议用时**：3 周（25-35 小时）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **URDF 结构**：`<link>` 的三个子元素（visual/collision/inertial）各自的作用是什么？为什么需要分别定义 visual 和 collision mesh？ | P01 URDF/Xacro 建模 |
| 2 | **Xacro 宏系统**：如何用 `xacro:macro` 参数化一个机械臂，使得同一个 macro 可以通过 `prefix` 参数生成左臂和右臂？ | P01 Xacro 参数化 |
| 3 | **MoveIt2 基础**：什么是 Planning Group？什么是 SRDF？`MoveGroupInterface` 的核心 API（plan/execute/move）如何使用？ | M14 MoveIt2 配置 |
| 4 | **ros2_control 框架**：`HardwareInterface` 和 `ControllerInterface` 的关系是什么？`JointTrajectoryController` 如何接收和执行轨迹？ | M12 ros2_control |
| 5 | **双臂协调**：什么是双臂的"相对运动约束"？为什么共持物体时两末端必须保持固定相对位姿？ | D02 双臂协调规划 |

---

## 本章目标

学完本章后，你应该能够：

1. **从零配置**双臂 MoveIt2 系统：URDF xacro 组合、SRDF 双臂 planning group、kinematics.yaml、controllers.yaml
2. **理解并实践**同步规划（`both_arms` 14D C-space）与异步规划（两个独立 7D group）的选择与权衡
3. **优化**双臂碰撞矩阵（ACM），将跨臂碰撞检查从 49 对压缩到 ~15 对，规划速度提升 3-5 倍
4. **编写** MTC 双臂任务流（Merger stage 并行、both_arms 协同搬运）
5. **搭建** Gazebo Harmonic 或 MuJoCo 中的双臂仿真环境，并与 MoveIt2 联调
6. **配置** ros2_control 双臂控制器，实现合并轨迹/并行执行/同步触发三种同步策略

---

## D9.1 双臂 URDF/Xacro 建模 ⭐⭐

### 动机——为什么双臂建模不是"复制粘贴两次"？

你可能想：双臂 URDF 就是把单臂 URDF 复制两份，改改名字就行了。如果你这样做，会遇到以下问题：

1. **名称冲突**：两个 `panda_link0` 在同一个 URDF 中——TF2 无法区分
2. **固定基座**：两臂的 `base_link` 如何连接到世界坐标系？相对位置如何确定？
3. **参数冲突**：`ros2_control` 的 `<hardware>` 标签中，两臂的关节名必须不同，否则控制器无法区分
4. **维护噩梦**：如果单臂 URDF 更新了一个关节限位，你需要手动修改两份——容易遗漏

### 如果不用 Xacro 参数化会怎样

手动复制粘贴 URDF 的后果：

一个 Franka Panda 的 URDF 约 500 行。双臂就是 1000 行。你手动把所有 `panda_` 改成 `left_panda_` 和 `right_panda_`。

一周后，上游 `franka_description` 包发布了更新——修正了一个惯性张量错误。你需要：(1) 下载新 URDF；(2) 重新手动改名 ×2；(3) 合并到你的双臂 URDF；(4) 祈祷没有遗漏。

两周后，你需要加第三臂做多臂协作实验。再手动改名 ×1？

**正确做法**：用 Xacro 宏参数化，单臂只写一次，通过 `prefix` 参数实例化任意多个。

### 历史——双臂 URDF 的演进

早期 ROS1 时代（2012-2016），双臂配置通常是手动维护两份 URDF 文件。MoveIt1 的 `dual_arm_panda_moveit_config` 示例首次展示了 Xacro 参数化方案（2018）。ROS2 时代，`moveit_resources` 包中的 `dual_panda` 配置成为事实标准，被广泛参考。

### Xacro 双臂组合——完整实现

**Step 1：单臂 Xacro 宏定义**

回顾 P01（URDF/Xacro 建模）：Xacro 宏通过 `xacro:macro` 定义可复用的参数化模板。现在我们利用这个机制创建可实例化的单臂模块。

```xml
<!-- panda_arm.urdf.xacro: 参数化单臂宏 -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 参数 -->
  <xacro:arg name="arm_id" default="panda"/>
  <xacro:arg name="connected_to" default=""/>
  <xacro:arg name="xyz" default="0 0 0"/>
  <xacro:arg name="rpy" default="0 0 0"/>
  
  <xacro:macro name="panda_arm" params="prefix connected_to xyz rpy">
    
    <!-- 基座连接 -->
    <xacro:if value="${connected_to != ''}">
      <joint name="${prefix}base_joint" type="fixed">
        <parent link="${connected_to}"/>
        <child link="${prefix}panda_link0"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
      </joint>
    </xacro:if>
    
    <!-- Link 0: 基座 -->
    <link name="${prefix}panda_link0">
      <visual>
        <geometry>
          <mesh filename="package://franka_description/meshes/visual/link0.dae"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://franka_description/meshes/collision/link0.stl"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.629769"/>
        <origin xyz="-0.041018 -0.00014 0.049974"/>
        <inertia ixx="0.00315" ixy="-6.69e-05" ixz="6.04e-04"
                 iyy="0.00388" iyz="1.02e-05" izz="0.00412"/>
      </inertial>
    </link>
    
    <!-- Joint 1 -->
    <joint name="${prefix}panda_joint1" type="revolute">
      <parent link="${prefix}panda_link0"/>
      <child link="${prefix}panda_link1"/>
      <origin xyz="0 0 0.333" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-2.8973" upper="2.8973" effort="87" velocity="2.175"/>
    </joint>
    
    <!-- Link 1 -->
    <link name="${prefix}panda_link1">
      <visual>
        <geometry>
          <mesh filename="package://franka_description/meshes/visual/link1.dae"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://franka_description/meshes/collision/link1.stl"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="4.97068"/>
        <origin xyz="0.003875 0.002081 -0.04762"/>
        <inertia ixx="0.70337" ixy="-0.000139" ixz="0.006772"
                 iyy="0.70661" iyz="0.019169" izz="0.009117"/>
      </inertial>
    </link>
    
    <!-- Joint 2 - Joint 7 同理，所有名称都用 ${prefix} 前缀 -->
    <!-- 此处省略 Joint 2-7 和 Link 2-7 的重复定义 -->
    <!-- 完整文件参见 moveit_resources/dual_arm_panda_moveit_config -->
    
    <!-- 末端：手爪 TCP -->
    <link name="${prefix}panda_hand_tcp"/>
    <joint name="${prefix}panda_hand_tcp_joint" type="fixed">
      <parent link="${prefix}panda_link8"/>
      <child link="${prefix}panda_hand_tcp"/>
      <origin xyz="0 0 0.1034" rpy="0 0 0"/>
    </joint>
    
    <!-- ros2_control 硬件接口 -->
    <ros2_control name="${prefix}panda_hardware" type="system">
      <hardware>
        <plugin>franka_hardware/FrankaHardwareInterface</plugin>
        <param name="robot_ip">PLACEHOLDER</param>
      </hardware>
      <joint name="${prefix}panda_joint1">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      <!-- ... Joint 2-7 类似 ... -->
    </ros2_control>
    
  </xacro:macro>
</robot>
```

**Step 2：双臂组合 URDF**

```xml
<!-- dual_panda.urdf.xacro: 组合两臂 -->
<robot name="dual_panda" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 引入单臂宏 -->
  <xacro:include filename="$(find franka_description)/urdf/panda_arm.urdf.xacro"/>
  
  <!-- 世界坐标系 -->
  <link name="world"/>
  
  <!-- 桌面（可选：提供固定参考和碰撞体） -->
  <link name="table_top">
    <visual>
      <geometry><box size="1.2 0.8 0.02"/></geometry>
      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><box size="1.2 0.8 0.02"/></geometry>
    </collision>
    <inertial>
      <mass value="50.0"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="table_joint" type="fixed">
    <parent link="world"/>
    <child link="table_top"/>
    <origin xyz="0.5 0 0.75" rpy="0 0 0"/>
  </joint>
  
  <!-- 左臂：位于桌面左侧 -->
  <xacro:panda_arm prefix="left_" 
                    connected_to="world"
                    xyz="0.0 0.5 0.75" 
                    rpy="0 0 0"/>
  
  <!-- 右臂：位于桌面右侧，绕 Z 轴旋转 180 度面对左臂 -->
  <xacro:panda_arm prefix="right_" 
                    connected_to="world"
                    xyz="1.0 -0.5 0.75" 
                    rpy="0 0 ${pi}"/>
  
</robot>
```

**为什么右臂要旋转 180 度？** 这是一个常见的双臂布局——两臂面对面站在桌面两侧。$\text{rpy} = (0, 0, \pi)$ 使右臂的 x 轴指向左臂方向，两臂的工作空间在桌面中心重叠——这是双臂协作（如共持物体）的前提。

**布局设计考量**：

| 布局 | 两臂间距 | 适用场景 | 注意 |
|------|---------|---------|------|
| 面对面 (180 度) | 0.8-1.2m | 共持搬运、handover | 工作空间重叠区域大 |
| 并排 (0 度) | 0.3-0.5m | 独立操作、ALOHA 风格 | 工作空间重叠小 |
| L 形 (90 度) | 0.5-0.8m | 流水线协作 | 一臂送料一臂操作 |
| 自定义角度 | 任意 | 特殊任务 | 需仿真验证可达性 |

**跨领域类比——PCB 元件布局**：双臂布局设计与 PCB 元件布局有相似之处：都需要在有限空间内最大化"互连性"（工作空间重叠），同时避免"干扰"（碰撞）。PCB 设计师用 DRC（Design Rule Check）验证间距，机器人工程师用碰撞检测验证可达性——工具不同但设计思想一致。

### 验证双臂 URDF

```bash
# 1. 检查 Xacro 语法
xacro dual_panda.urdf.xacro > dual_panda.urdf

# 2. 验证 URDF 完整性
check_urdf dual_panda.urdf
# 应输出: robot name is: dual_panda
#         ---------- Successfully Parsed XML ---------------
#         root Link: world has 2 child(ren)

# 3. 在 RViz2 中可视化
ros2 launch dual_panda_description display.launch.py
# 确认：两臂位置正确、TF 树完整、关节滑块可用

# 4. 检查 TF 树
ros2 run tf2_tools view_frames
# 应看到以 world 为根的树，左右臂各自的 link chain
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Xacro prefix 末尾缺少下划线
   错误做法：prefix="left" → 生成 "leftpanda_joint1"（无分隔符）
   正确做法：prefix="left_" → 生成 "left_panda_joint1"（清晰可读）
   根本原因：Xacro 的字符串拼接是直接连接，不会自动加分隔符
   自检方法：xacro 输出的 URDF 中搜索 joint 名称，确认格式正确

💡 概念误区：认为"两臂 URDF 可以是两个独立文件"
   新手想法："左臂一个 URDF，右臂一个 URDF，分别加载"
   实际上：MoveIt2 的碰撞检测和 both_arms 规划要求两臂在同一个 URDF 中。
   如果分开加载，MoveIt2 无法检测跨臂碰撞，导致规划的轨迹可能让两臂相撞。
   ros2_control 也需要一个统一的 robot_description 来注册所有关节。

🧠 思维陷阱：认为"双臂间距随便设"
   新手想法："先随便放，不行再调"
   实际上：双臂间距直接决定工作空间重叠区域大小。如果间距太大，
   两臂在桌面中心无法同时够到——共持物体成为不可能。
   如果间距太小，两臂的 link 在初始构型就碰撞——MoveIt2 无法规划。
   正确做法：先用正运动学计算两臂各自的可达工作空间（球形近似），
   确保重叠区域覆盖任务所需的操作空间。典型：Franka 臂展 855mm，
   面对面间距 1.0m 时重叠区域约 710mm 深——足够大多数桌面操作。
```

### 练习

1. **[编程]** 用 Xacro 创建双 Franka Panda URDF。要求：(a) 支持通过 launch 参数切换面对面/并排布局；(b) 桌面作为固定链接；(c) 在 RViz2 中验证 TF 树和关节滑块。
2. **[手推]** 计算面对面布局（间距 1.0m）中两臂工作空间重叠区域的近似体积。Franka Panda 的有效可达半径约 855mm。提示：两个球的交集体积公式。
3. **[思考题]** 如果你需要构建一个"异构双臂"系统（左臂 Franka 7-DOF + 右臂 UR5 6-DOF），Xacro 参数化方案该如何修改？两个不同的 macro 共享同一个 `connected_to` 参数是否可行？

---

## D9.2 SRDF 双臂 Planning Group 配置 ⭐⭐

### 动机——为什么需要 SRDF？

URDF 定义了机器人的物理模型（连杆、关节、几何）。但 MoveIt2 还需要额外的语义信息：

- **哪些关节构成一个"组"**（Planning Group）？
- **哪些碰撞对可以忽略**（ACM, Allowed Collision Matrix）？
- **什么是"初始构型"**（Named States）？
- **末端执行器怎么定义**？

这些信息存储在 **SRDF**（Semantic Robot Description Format）中。URDF 是"物理的"——描述机器人长什么样；SRDF 是"语义的"——描述机器人怎么被使用。

> **跨领域类比——SRDF 与数据库 Schema**：SRDF 之于 URDF，类似于数据库 Schema（表结构定义、索引、约束）之于原始数据文件。原始数据文件只包含数据本身（对应 URDF 的物理几何），而 Schema 定义了数据的"使用方式"——哪些字段构成一个查询组（对应 Planning Group）、哪些字段对有唯一性约束（对应 Named States）、哪些外键关系可以忽略（对应 ACM 中禁用的碰撞对）。没有 Schema 的数据库可以运行但效率极低；没有 SRDF 的 MoveIt2 也可以规划但必须手动指定所有信息。

### 双臂 SRDF 完整配置

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot name="dual_panda">
  
  <!-- ============== Planning Groups ============== -->
  
  <!-- 左臂组：7 个关节 -->
  <group name="left_arm">
    <chain base_link="left_panda_link0" 
           tip_link="left_panda_hand_tcp"/>
  </group>
  
  <!-- 右臂组：7 个关节 -->
  <group name="right_arm">
    <chain base_link="right_panda_link0" 
           tip_link="right_panda_hand_tcp"/>
  </group>
  
  <!-- 双臂联合组：14 个关节 -->
  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>
  
  <!-- 左手组（夹爪） -->
  <group name="left_hand">
    <joint name="left_panda_finger_joint1"/>
    <joint name="left_panda_finger_joint2"/>
  </group>
  
  <!-- 右手组 -->
  <group name="right_hand">
    <joint name="right_panda_finger_joint1"/>
    <joint name="right_panda_finger_joint2"/>
  </group>
  
  <!-- ============== 末端执行器 ============== -->
  
  <end_effector name="left_hand_ee" parent_link="left_panda_link8"
                group="left_hand" parent_group="left_arm"/>
  <end_effector name="right_hand_ee" parent_link="right_panda_link8"
                group="right_hand" parent_group="right_arm"/>
  
  <!-- ============== Named States ============== -->
  
  <group_state name="left_home" group="left_arm">
    <joint name="left_panda_joint1" value="0"/>
    <joint name="left_panda_joint2" value="-0.785"/>
    <joint name="left_panda_joint3" value="0"/>
    <joint name="left_panda_joint4" value="-2.356"/>
    <joint name="left_panda_joint5" value="0"/>
    <joint name="left_panda_joint6" value="1.571"/>
    <joint name="left_panda_joint7" value="0.785"/>
  </group_state>
  
  <group_state name="right_home" group="right_arm">
    <joint name="right_panda_joint1" value="0"/>
    <joint name="right_panda_joint2" value="-0.785"/>
    <joint name="right_panda_joint3" value="0"/>
    <joint name="right_panda_joint4" value="-2.356"/>
    <joint name="right_panda_joint5" value="0"/>
    <joint name="right_panda_joint6" value="1.571"/>
    <joint name="right_panda_joint7" value="0.785"/>
  </group_state>
  
  <group_state name="both_home" group="both_arms">
    <joint name="left_panda_joint1" value="0"/>
    <joint name="left_panda_joint2" value="-0.785"/>
    <joint name="left_panda_joint3" value="0"/>
    <joint name="left_panda_joint4" value="-2.356"/>
    <joint name="left_panda_joint5" value="0"/>
    <joint name="left_panda_joint6" value="1.571"/>
    <joint name="left_panda_joint7" value="0.785"/>
    <joint name="right_panda_joint1" value="0"/>
    <joint name="right_panda_joint2" value="-0.785"/>
    <joint name="right_panda_joint3" value="0"/>
    <joint name="right_panda_joint4" value="-2.356"/>
    <joint name="right_panda_joint5" value="0"/>
    <joint name="right_panda_joint6" value="1.571"/>
    <joint name="right_panda_joint7" value="0.785"/>
  </group_state>
  
  <!-- ============== Allowed Collision Matrix ============== -->
  
  <!-- 同臂相邻连杆：永远相邻，无需碰撞检查 -->
  <disable_collisions link1="left_panda_link0" link2="left_panda_link1" reason="Adjacent"/>
  <disable_collisions link1="left_panda_link1" link2="left_panda_link2" reason="Adjacent"/>
  <disable_collisions link1="left_panda_link2" link2="left_panda_link3" reason="Adjacent"/>
  <disable_collisions link1="left_panda_link3" link2="left_panda_link4" reason="Adjacent"/>
  <disable_collisions link1="left_panda_link4" link2="left_panda_link5" reason="Adjacent"/>
  <disable_collisions link1="left_panda_link5" link2="left_panda_link6" reason="Adjacent"/>
  <disable_collisions link1="left_panda_link6" link2="left_panda_link7" reason="Adjacent"/>
  <!-- 右臂同理 -->
  <disable_collisions link1="right_panda_link0" link2="right_panda_link1" reason="Adjacent"/>
  <!-- ... -->
  
  <!-- 跨臂底座连杆：固定不动，永不碰撞 -->
  <disable_collisions link1="left_panda_link0" link2="right_panda_link0" reason="Never"/>
  <disable_collisions link1="left_panda_link0" link2="right_panda_link1" reason="Never"/>
  <disable_collisions link1="left_panda_link1" link2="right_panda_link0" reason="Never"/>
  <disable_collisions link1="left_panda_link1" link2="right_panda_link1" reason="Never"/>
  
  <!-- 注意：跨臂远端连杆和两夹爪之间 *不* 禁用碰撞检测！ -->
  
</robot>
```

### Planning Group 的三种用法

| 用法 | API 调用 | C-space 维度 | 碰撞检查范围 | 适用场景 |
|------|---------|-------------|-------------|---------|
| 单臂独立 | `moveit::planning_interface::MoveGroupInterface(node, "left_arm")` | 7D | 单臂自碰撞 + 环境 | 独立操作 |
| 双臂同步 | `moveit::planning_interface::MoveGroupInterface(node, "both_arms")` | 14D | 双臂互碰 + 自碰撞 + 环境 | 协同搬运 |
| 两臂顺序 | 先 `left_arm.plan()` 再 `right_arm.plan()` | 7D + 7D | 各自独立（不检跨臂） | 先后操作 |

### kinematics.yaml 双臂配置

```yaml
# kinematics.yaml
left_arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
  position_only_ik: false
  
right_arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
  position_only_ik: false

both_arms:
  # 不建议为合成组声明“自动拼接 IK”。
  # both_arms 可靠用于 14D joint-space planning；
  # 双末端 pose 目标应分别对 left_arm/right_arm 求 IK 后拼接。
  kinematics_solver_timeout: 0.1
```

> **本质洞察**：`both_arms` 组的强项是 14D 联合关节空间规划和双臂碰撞检测，不要假设 MoveIt2 会自动为合成组调用左右子组 IK 并可靠拼接。工程上处理双末端笛卡尔目标的稳妥流程是：分别用 `left_arm`、`right_arm` 的 IK 求解两个末端目标，检查左右解和跨臂碰撞，再把 14D 关节目标交给 `both_arms` 做联合规划。若目标包含闭链、相对位姿或“共持物体距离固定”等协调约束，应转到 D02 的约束 IK/约束规划，而不是依赖 `both_arms` 的普通 IK。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：SRDF 中 both_arms 组重复定义关节
   错误做法：在 both_arms 组中逐个列出 14 个关节
   现象：可能遗漏关节，或与子组定义不一致
   正确做法：用 <group name="left_arm"/> 引用子组
   MoveIt2 会自动展开子组中的所有关节

💡 概念误区：认为"MoveIt Setup Assistant 能自动完美处理双臂"
   新手想法："跑一次 Setup Assistant 就能自动生成双臂配置"
   实际上：Setup Assistant 可以生成基本配置，但 both_arms 组和跨臂
   ACM 需要手动调整。特别是跨臂碰撞对的 enable/disable 策略
   需要人工判断（哪些跨臂 link 对真的可能碰撞）。
   推荐流程：用 Setup Assistant 生成初始配置 → 手动添加 both_arms 组
   → 手动优化跨臂 ACM → 仿真验证。

🧠 思维陷阱：认为"只要 both_arms 组定义了就够了"
   新手想法："有了 both_arms 组，所有双臂操作都用它规划"
   实际上：both_arms 的 14D 规划速度远低于单臂 7D。如果两臂当前操作
   不需要协调（各自独立操作），使用 both_arms 是浪费计算资源。
   正确做法：根据任务阶段动态选择 planning group：
   独立操作 → left_arm/right_arm；协调运动 → both_arms。
```

### 练习

1. **[编程]** 用 MoveIt2 Setup Assistant 为双 Franka 生成初始配置。然后手动修改 SRDF：添加 `both_arms` 组和跨臂 ACM 优化。在 RViz 中验证三种 Planning Group 都能正常规划。
2. **[思考题]** 如果你的双臂系统有一个"共享末端执行器"（如两臂共持的一个大夹爪），它应该属于哪个 Planning Group？需要如何修改 SRDF？

---

## D9.3 同步 vs 异步双臂规划 ⭐⭐

### 动机——14D 规划的代价

回顾 D02（双臂协调规划）：双臂协调运动需要在 14D 联合构型空间中搜索无碰撞路径。但 14D 空间的体积是 7D 空间的平方——采样规划器（RRT、PRM）的效率急剧下降。

具体来说，RRT-Connect 在 7D 中找到一条路径平均需要 0.1-0.5 秒。但在 14D 中，同样的环境复杂度下，平均需要 2-10 秒——因为随机采样在 14D 中击中"狭窄通道"的概率远低于 7D。

这就产生了一个实际工程问题：**是否总需要 14D 同步规划？能不能两臂各自独立 7D 规划？**

### 同步规划（both_arms 组）

```cpp
// 同步规划：14D 联合空间
#include <moveit/move_group_interface/move_group_interface.h>

auto both_arms = moveit::planning_interface::MoveGroupInterface(node, "both_arms");

// 设置双臂联合目标——方法 1：关节空间目标
both_arms.setJointValueTarget({
    // 左臂 7 关节 + 右臂 7 关节 = 14D
    0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,  // left
    0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785   // right
});

// 方法 2：双末端笛卡尔目标
// 不依赖 both_arms 自动 IK；先分别对左右臂 IK，再拼接成 14D joint target。
geometry_msgs::msg::Pose left_target, right_target;
left_target.position.x = 0.4; left_target.position.y = 0.2;
left_target.position.z = 0.9;
left_target.orientation.w = 1.0;

right_target.position.x = 0.6; right_target.position.y = -0.2;
right_target.position.z = 0.9;
right_target.orientation.w = 1.0;

auto state = both_arms.getCurrentState(1.0);
auto* left_jmg = state->getJointModelGroup("left_arm");
auto* right_jmg = state->getJointModelGroup("right_arm");
auto* both_jmg = state->getJointModelGroup("both_arms");

bool ok_left = state->setFromIK(left_jmg, left_target, "left_panda_hand_tcp", 0.05);
bool ok_right = state->setFromIK(right_jmg, right_target, "right_panda_hand_tcp", 0.05);
if (!ok_left || !ok_right) {
    throw std::runtime_error("left/right IK failed; do not plan both_arms");
}

std::vector<double> both_target;
state->copyJointGroupPositions(both_jmg, both_target);
both_arms.setJointValueTarget(both_target);

// 规划配置
both_arms.setPlanningTime(10.0);       // 14D 需要更长规划时间
both_arms.setNumPlanningAttempts(5);   // 多次尝试
both_arms.setMaxVelocityScalingFactor(0.5);

// 规划并执行
moveit::planning_interface::MoveGroupInterface::Plan plan;
auto code = both_arms.plan(plan);
if (code == moveit::core::MoveItErrorCode::SUCCESS) {
    both_arms.execute(plan);
}
```

**同步规划的优势**：
- 自动避双臂互碰——碰撞检测在 14D 联合空间中进行
- 路径全局最优（在时间限制内）
- 轨迹使用同一个时间参数——一条 14D 轨迹包含两臂的对应采样点；执行同步仍取决于控制器时间基准

**同步规划的劣势**：
- 规划时间长（5-20 倍于单臂）
- 可能找不到解（14D 空间中的概率完整性收敛更慢）
- 对于独立操作任务是不必要的浪费

### 异步规划（两组独立）

```cpp
// 异步规划：两个独立 7D
auto left_arm = moveit::planning_interface::MoveGroupInterface(node, "left_arm");
auto right_arm = moveit::planning_interface::MoveGroupInterface(node, "right_arm");

// 各自规划
left_arm.setPoseTarget(left_target, "left_panda_hand_tcp");
right_arm.setPoseTarget(right_target, "right_panda_hand_tcp");

moveit::planning_interface::MoveGroupInterface::Plan plan_L, plan_R;
auto code_L = left_arm.plan(plan_L);
auto code_R = right_arm.plan(plan_R);

if (code_L == moveit::core::MoveItErrorCode::SUCCESS &&
    code_R == moveit::core::MoveItErrorCode::SUCCESS) {
    // 方法 1: 顺序执行（安全但慢）
    left_arm.execute(plan_L);
    // 等待左臂完成
    right_arm.execute(plan_R);
    
    // 方法 2: 并行执行（快但可能碰撞！需额外检查）
    auto fut_L = std::async(std::launch::async, 
                            [&]{ left_arm.execute(plan_L); });
    auto fut_R = std::async(std::launch::async, 
                            [&]{ right_arm.execute(plan_R); });
    fut_L.get();
    fut_R.get();
}
```

### 同步与异步执行的详细代码

**完整的同步规划 + 执行框架**（含错误处理和 fallback 逻辑）：

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <future>

class DualArmPlanner {
public:
    DualArmPlanner(rclcpp::Node::SharedPtr node)
        : node_(node),
          both_arms_(node, "both_arms"),
          left_arm_(node, "left_arm"),
          right_arm_(node, "right_arm") {
        
        // 全局配置
        both_arms_.setPlanningTime(10.0);
        both_arms_.setNumPlanningAttempts(3);
        both_arms_.setMaxVelocityScalingFactor(0.5);
        both_arms_.setMaxAccelerationScalingFactor(0.3);
        
        left_arm_.setPlanningTime(3.0);
        right_arm_.setPlanningTime(3.0);
    }
    
    /// 同步规划：14D 联合空间，保证无碰撞
    bool plan_sync(const geometry_msgs::msg::Pose& left_target,
                   const geometry_msgs::msg::Pose& right_target) {
        auto state = both_arms_.getCurrentState(1.0);
        auto* left_jmg = state->getJointModelGroup("left_arm");
        auto* right_jmg = state->getJointModelGroup("right_arm");
        auto* both_jmg = state->getJointModelGroup("both_arms");

        bool ok_left = state->setFromIK(
            left_jmg, left_target, "left_panda_hand_tcp", 0.05);
        bool ok_right = state->setFromIK(
            right_jmg, right_target, "right_panda_hand_tcp", 0.05);
        if (!ok_left || !ok_right) {
            RCLCPP_WARN(node_->get_logger(), "Left/right IK failed");
            return false;
        }

        std::vector<double> both_target;
        state->copyJointGroupPositions(both_jmg, both_target);
        both_arms_.setJointValueTarget(both_target);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto code = both_arms_.plan(plan);
        if (code == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), 
                "Sync plan OK: %.2fs, %zu points",
                plan.planning_time,
                plan.trajectory.joint_trajectory.points.size());
            return both_arms_.execute(plan) == 
                   moveit::core::MoveItErrorCode::SUCCESS;
        }
        RCLCPP_WARN(node_->get_logger(), "Sync plan FAILED, trying fallback");
        return false;
    }
    
    /// 异步规划 + 碰撞验证：两个 7D 独立规划，合并后验证
    bool plan_async_verified(
            const geometry_msgs::msg::Pose& left_target,
            const geometry_msgs::msg::Pose& right_target) {
        left_arm_.setPoseTarget(left_target, "left_panda_hand_tcp");
        right_arm_.setPoseTarget(right_target, "right_panda_hand_tcp");
        
        // 并行规划
        moveit::planning_interface::MoveGroupInterface::Plan plan_L, plan_R;
        auto fut_L = std::async(std::launch::async, [&]{
            return left_arm_.plan(plan_L);
        });
        auto fut_R = std::async(std::launch::async, [&]{
            return right_arm_.plan(plan_R);
        });
        
        auto res_L = fut_L.get();
        auto res_R = fut_R.get();
        
        if (res_L != moveit::core::MoveItErrorCode::SUCCESS ||
            res_R != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Async plan failed");
            return false;
        }
        
        // 碰撞验证：合并两条轨迹检查 14D 碰撞
        if (check_dual_collision(plan_L, plan_R)) {
            RCLCPP_WARN(node_->get_logger(), "Async plans collide!");
            return false;  // 碰撞则放弃，上层应调 plan_sync
        }
        
        // 并行执行
        auto exec_L = std::async(std::launch::async, [&]{
            return left_arm_.execute(plan_L);
        });
        auto exec_R = std::async(std::launch::async, [&]{
            return right_arm_.execute(plan_R);
        });
        exec_L.get();
        exec_R.get();
        return true;
    }
    
    /// 自适应选择：先尝试异步（快），碰撞则切同步（安全）
    bool plan_adaptive(const geometry_msgs::msg::Pose& left_target,
                       const geometry_msgs::msg::Pose& right_target) {
        if (plan_async_verified(left_target, right_target)) {
            return true;
        }
        RCLCPP_INFO(node_->get_logger(), "Falling back to sync planning");
        return plan_sync(left_target, right_target);
    }

private:
    bool check_dual_collision(
            const moveit::planning_interface::MoveGroupInterface::Plan& pL,
            const moveit::planning_interface::MoveGroupInterface::Plan& pR) {
        // 教学骨架：生产代码必须对每个时间步合并左右臂关节状态，
        // 再调用 PlanningScene::checkCollision() 检查完整双臂模型。
        // 这里不能 return false 假装安全；未实现时应保守失败，
        // 让上层回退到同步规划或直接拒绝执行。
        auto& traj_L = pL.trajectory.joint_trajectory;
        auto& traj_R = pR.trajectory.joint_trajectory;
        size_t n = std::max(traj_L.points.size(), traj_R.points.size());
        (void)traj_L;
        (void)traj_R;
        (void)n;
        RCLCPP_WARN(node_->get_logger(),
                    "Dual-arm collision check is a skeleton; "
                    "treating trajectory as unsafe.");
        return true;
    }
    
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface both_arms_, left_arm_, right_arm_;
};
```

### 同步规划的加速策略

14D 规划慢的根因是采样效率低。以下策略可显著加速：

| 策略 | 方法 | 加速比 | 适用条件 |
|------|------|--------|---------|
| **更好的规划器** | BIT* 替代 RRT-Connect | 2-5x | 通用 |
| **子空间约束** | 限制 both_arms 只在子空间移动 | 3-10x | 对称操作 |
| **规划时间+重试** | `setPlanningTime(5.0)` + 3 次重试 | — | 实时需求 |
| **镜像初始猜测** | 利用对称构型作 seed | 2-3x | 对称双臂 |
| **ACM 优化** | 减少碰撞检查对数 | 1.5-3x | 通用 |

```yaml
# ompl_planning.yaml: BIT* 配置
both_arms:
  planner_configs:
    - BITstar
    - RRTConnect
  default_planner_config: BITstar
  # 投影评估器：只投影到 4 个"关键关节"降低距离估计维度
  projection_evaluator: joints(left_panda_joint1, left_panda_joint4, 
                               right_panda_joint1, right_panda_joint4)
```

### 选型决策树

```
                    两臂是否需要同时运动？
                           │
              ┌────────────┴────────────┐
              │ 否                       │ 是
              ▼                          ▼
        ┌──────────┐           两臂是否可能碰撞？
        │ 异步规划  │              │
        │ (更快)   │    ┌─────────┴─────────┐
        └──────────┘    │ 是                 │ 否(已确认)
                        ▼                    ▼
                 ┌──────────┐         ┌──────────┐
                 │ 同步规划  │         │ 异步规划  │
                 │(both_arms)│        │ +碰撞监控 │
                 └──────────┘         └──────────┘
```

### 反事实推理——异步规划碰撞的后果

如果两臂各自独立规划，然后并行执行，可能发生什么？

设左臂从 A 到 B，右臂从 C 到 D。各自的路径在 7D 中无碰撞（不与环境碰撞）。但当两条路径在时间上叠加时，可能存在某个时刻 $t$ 使得左臂在位置 $p_L(t)$ 和右臂在位置 $p_R(t)$ 发生碰撞——**这在各自的 7D 规划中是检测不到的**。

工程案例：一个双臂系统执行"两臂末端交换位置"的任务。左臂从左移到右，右臂从右移到左。独立规划时，两条路径都经过中间区域——在 $t \approx T/2$ 时两臂在中间相撞。同步规划会自动避开这个碰撞，生成一条"绕行"路径（如一臂先上升、另一臂先通过、然后第一臂下降）。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：both_arms 规划超时后不做 fallback
   错误做法：规划失败就报错退出
   现象：14D 规划经常超时（特别是狭窄空间），系统频繁停机
   正确做法：超时后 fallback 到异步规划 + 离散碰撞检查
   伪代码：
     Plan plan; auto code = both_arms.plan(plan);
     if (code != SUCCESS) {
         Plan plan_L, plan_R;
         auto ok_L = left_arm.plan(plan_L);
         auto ok_R = right_arm.plan(plan_R);
         if (ok_L == SUCCESS && ok_R == SUCCESS && 
             !check_dual_arm_collision(plan_L, plan_R)) {
             execute_parallel(plan_L, plan_R);
         } else {
             // 需人工介入或重新规划
         }
     }

💡 概念误区：认为"同步规划总是比异步好"
   新手想法："同步规划更安全，应该总是用 both_arms"
   实际上：如果两臂操作的工作空间完全不重叠（如左臂在左半桌面，
   右臂在右半桌面），同步规划是浪费——14D 搜索中大量采样落在
   "两臂各自可行但联合冗余"的区域。
   正确做法：分析任务的空间重叠度。重叠 > 50% → 同步；
   重叠 < 20% → 异步；中间地带 → 异步 + 碰撞监控。
```

### 练习

1. **[编程]** 分别用 `both_arms` 和两个独立组规划"两臂末端交换位置"。记录规划时间和碰撞检测结果——同步规划应避免碰撞但时间 5-20 倍；异步规划快但可能碰撞。
2. **[编程]** 实现一个 `check_dual_arm_collision` 函数：输入两条 7D 轨迹，对每个时间点合并为 14D 构型，用 `PlanningScene::checkCollision` 检查碰撞。
3. **[跨章综合题]** 结合 D02（双臂协调规划）和 D09：D02 中讨论了"约束规划"（如两末端保持固定相对位姿）。在 MoveIt2 中如何实现？提示：使用 `ompl_interface::ConstrainedPlanningStateSpace` 和自定义 `ompl::base::Constraint`。

---

## D9.4 碰撞矩阵（ACM）优化 ⭐⭐

### 动机——碰撞检查是规划的瓶颈

MoveIt2 规划的每一步（采样 → 碰撞检查 → 连接）中，碰撞检查占 60-80% 的计算时间。双臂系统的碰撞检查比单臂复杂得多。

单臂 Franka（8 link）的碰撞对数量：$\binom{8}{2} = 28$ 对。双臂（16 link）：$\binom{16}{2} = 120$ 对。其中跨臂碰撞对：$8 \times 8 = 64$ 对。

但绝大多数跨臂碰撞对在实际中不可能碰撞（如左臂底座和右臂底座，它们都固定在桌面上）。**ACM 优化就是识别并禁用这些"不可能碰撞"的对，减少碰撞检查的计算量**。

> **跨领域类比——ACM 与防火墙白名单**：ACM 的工作方式与网络防火墙的白名单规则高度类似。防火墙默认检查所有流量（对应默认检查所有碰撞对），但管理员可以将已知安全的 IP 对加入白名单跳过检查（对应 ACM 中 disable 已知不碰撞的 link 对）。过度白名单（放行太多流量）会引入安全风险（对应过度 disable ACM 导致碰撞漏检）；白名单太保守（几乎不放行）则防火墙成为性能瓶颈（对应 ACM 未优化导致规划超时）。两者的调优原则相同：**在安全的前提下最大化效率**。

### ACM 优化方法

**方法一：MoveIt2 Setup Assistant 自动生成**

Setup Assistant 的 Self-Collision 页面通过随机采样 10000 个构型，统计每对 link 的碰撞频率：

```
默认结果（双 Franka，10000 随机构型）：
  - 同臂相邻对：28 对 → disable（Adjacent）
  - 跨臂从不碰撞对：~40 对 → disable（Never）
  - 跨臂可能碰撞对：~24 对 → enable（保留检查）
  - 总碰撞检查对：从 120 → ~52（减少 57%）
```

**方法二：手动基于几何分析**

比随机采样更精确——基于两臂几何约束的确定性分析：

| 跨臂连杆对 | 是否可能碰撞？ | 理由 |
|-----------|---------------|------|
| link0 ↔ link0 | 不可能 | 都固定在桌面 |
| link0 ↔ link1 | 不可能 | 距离远且运动范围小 |
| link1 ↔ link1 | 不可能 | 近端关节运动范围限制 |
| link3 ↔ link3 | 不可能 | 中段关节几何上不可达 |
| link5 ↔ link5 | **可能** | 远端连杆可达范围重叠 |
| link6 ↔ link6 | **可能** | 末端工作空间重叠 |
| link7 ↔ link7 | **很可能** | 两末端执行器经常接近 |
| hand ↔ hand | **必须检查** | 两夹爪在操作中频繁接近 |

**方法三：动态 ACM（共持物体场景）**

```cpp
// 共持物体时动态修改 ACM
auto planning_scene = planning_scene_monitor->getPlanningScene();
auto& acm = planning_scene->getAllowedCollisionMatrixNonConst();

// 物体被左手抓取 → 允许物体与左手碰撞（attached object）
acm.setEntry("box", "left_panda_hand", true);  // true = 允许碰撞

// 共持时 → 也允许物体与右手碰撞
acm.setEntry("box", "right_panda_hand", true);

// 但两夹爪之间仍需碰撞检查（防止夹爪互碰导致物体掉落）
acm.setEntry("left_panda_hand", "right_panda_hand", false);  // false = 检查碰撞
```

### 反事实推理——过度优化 ACM 的后果

如果为了加速规划，禁用了所有跨臂碰撞检测，会发生什么？

规划器"看不到"两臂之间的碰撞。它生成的路径可能让左臂直接穿过右臂——在仿真中表现为物体穿模，在真机上表现为两臂相撞，可能损坏硬件。更隐蔽的情况：大部分时间没问题，但在某些特殊构型下偶尔碰撞——这种间歇性 bug 极难调试。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Setup Assistant 的采样数不够
   错误做法：用默认 10000 采样生成 ACM，直接使用
   现象：某些低概率碰撞对被误标为 "Never"
   正确做法：增加采样数到 100000 或更多（需要更长生成时间）
   或手动基于几何分析确认关键碰撞对

💡 概念误区：认为"ACM 只影响规划速度"
   新手想法："ACM 优化只是让规划快一点，不优化也没关系"
   实际上：ACM 同时影响规划的正确性。错误地禁用了一个实际会碰撞的对，
   规划器会生成碰撞路径——在仿真中物体穿模，在真机上直接撞坏。
   ACM 优化的原则：宁可多检查（false negative 比 false positive 安全）。
```

### 练习

1. **[编程]** 用 MoveIt2 Setup Assistant 重新生成双臂 ACM。对比默认 ACM（全量检查）和优化 ACM（Auto-Disable 10000 采样）的规划速度差异。做 20 次 `both_arms` 规划，记录平均规划时间。
2. **[思考题]** 如果两臂共持一个大箱子，箱子同时 attached 到左手和右手。此时箱子与两臂的碰撞检查应该如何设置？箱子本身是否应该从碰撞场景中移除？

---

## D9.5 MoveIt Task Constructor (MTC) 双臂扩展 ⭐⭐⭐

### 动机——复杂双臂任务的模块化编排

回顾 M14（MoveIt2 + MTC）：MTC 把复杂任务拆分为有序的 Stages，每个 Stage 负责一小段运动。单臂 pick-and-place 的 MTC 管线是：approach → grasp → lift → move → place → release。

双臂任务更复杂——需要两臂的 stages 同步或顺序执行。例如，双臂协同搬运的管线：

```
两臂各自 approach → 两臂同时 grasp → 双臂协同 lift → 
双臂协同 transport → 双臂协同 place → 两臂同时 release
```

MTC 的 **Merger stage** 是处理双臂并行动作的关键——它把两个独立 stage 的轨迹合并为一条同步轨迹。

> **跨领域类比——MTC 与 CI/CD Pipeline**：MTC 的 stage 编排与软件工程中的 CI/CD Pipeline（如 GitHub Actions、GitLab CI）有惊人的相似性。每个 MTC Stage 对应 CI/CD 中的一个 Job（build/test/deploy），Stage 之间的依赖关系对应 Job 之间的 `needs` 声明。Merger Stage 对应 CI/CD 中的"并行 Job 组"——两个独立 Job 同时运行，完成后合并结果进入下一阶段。MTC 的 fallback 机制（一个 solver 失败时尝试另一个）对应 CI/CD 的 retry 策略。两者的设计哲学都是**将复杂流程拆解为可组合、可独立测试的原子单元**。

### Merger Stage 详解

```cpp
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <set>

using namespace moveit::task_constructor;

// 创建双臂协同搬运任务
auto task = std::make_unique<Task>();
task->setRobotModel(robot_model);

// 规划求解器
auto pipeline = std::make_shared<solvers::PipelinePlanner>(node, "ompl");
pipeline->setPlannerId("RRTConnectkConfigDefault");
auto joint_interp = std::make_shared<solvers::JointInterpolationPlanner>();
// PipelinePlanner 描述的是 planning pipeline / planner id，不描述规划组。
// left_arm/right_arm/both_arms 由每个 stage 的 setGroup() 决定。

// ============ Stage 1: 两臂各自 approach ============
{
    auto approach_both = std::make_unique<stages::Merger>("approach_both");
    
    // 左臂 approach
    auto approach_L = std::make_unique<stages::MoveRelative>("approach_L", pipeline);
    approach_L->setGroup("left_arm");
    approach_L->setIKFrame("left_panda_hand_tcp");
    geometry_msgs::msg::Vector3Stamped dir_L;
    dir_L.header.frame_id = "world";
    dir_L.vector.x = 0.1;  // 向前 10cm
    approach_L->setDirection(dir_L);
    
    // 右臂 approach
    auto approach_R = std::make_unique<stages::MoveRelative>("approach_R", pipeline);
    approach_R->setGroup("right_arm");
    approach_R->setIKFrame("right_panda_hand_tcp");
    geometry_msgs::msg::Vector3Stamped dir_R;
    dir_R.header.frame_id = "world";
    dir_R.vector.x = -0.1;  // 右臂旋转了180度，x反向
    approach_R->setDirection(dir_R);
    
    // Merger 合并两条轨迹
    approach_both->insert(std::move(approach_L));
    approach_both->insert(std::move(approach_R));
    task->add(std::move(approach_both));
}

// ============ Stage 2: 同步 grasp ============
{
    auto grasp = std::make_unique<stages::ModifyPlanningScene>("attach_box");
    // MoveIt attached object 只能有一个 parent link；共持物体时选一个主挂载 link，
    // 再把两只手的接触 link 都放进 touch_links，避免“只允许左手碰箱子”。
    std::set<std::string> touch_links = {
        "left_panda_hand", "left_panda_hand_tcp",
        "left_panda_leftfinger", "left_panda_rightfinger",
        "right_panda_hand", "right_panda_hand_tcp",
        "right_panda_leftfinger", "right_panda_rightfinger",
    };
    grasp->allowCollisions("box", touch_links, true);
    grasp->attachObject("box", "left_panda_hand_tcp");
    task->add(std::move(grasp));
    
    // 关闭两手夹爪
    auto close_L = std::make_unique<stages::MoveTo>("close_left", joint_interp);
    close_L->setGroup("left_hand");
    close_L->setGoal("close");
    task->add(std::move(close_L));
    
    auto close_R = std::make_unique<stages::MoveTo>("close_right", joint_interp);
    close_R->setGroup("right_hand");
    close_R->setGoal("close");
    task->add(std::move(close_R));
}

// ============ Stage 3: 协同 lift ============
{
    auto lift = std::make_unique<stages::MoveRelative>("lift", pipeline);
    // 教学版几何 lift：both_arms 提供 14D 联合规划和碰撞检查，
    // 但 MoveRelative 本身不保证共持物体的闭链相对位姿约束。
    lift->setGroup("both_arms");
    geometry_msgs::msg::Vector3Stamped up;
    up.header.frame_id = "world";
    up.vector.z = 0.15;  // 上升 15cm
    lift->setDirection(up);
    task->add(std::move(lift));
}

// ============ Stage 4: transport ============
{
    auto transport = std::make_unique<stages::MoveTo>("transport", pipeline);
    transport->setGroup("both_arms");
    // 共持物体时，这个 named state 应来自双 TCP IK / 约束规划结果，
    // 不能只手写一个看起来合理的 14D 关节姿态。
    transport->setGoal("dual_arm_place_pose");
    task->add(std::move(transport));
}

// ============ Stage 5: place (下降) ============
{
    auto place = std::make_unique<stages::MoveRelative>("place", pipeline);
    place->setGroup("both_arms");
    geometry_msgs::msg::Vector3Stamped down;
    down.header.frame_id = "world";
    down.vector.z = -0.15;
    place->setDirection(down);
    task->add(std::move(place));
}

// ============ Stage 6: release ============
{
    auto release = std::make_unique<stages::ModifyPlanningScene>("detach_box");
    release->detachObject("box", "left_panda_hand_tcp");
    task->add(std::move(release));
}

// 规划并执行。不同 MTC 版本的 plan() 返回值语义略有差异，
// 真正取 front() 前必须检查解集非空。
if (task->plan(10) && !task->solutions().empty()) {  // 最多规划 10 个方案
    task->introspection().publishSolution(*task->solutions().front());
    // execute...
}
```

这里不要把 `attachObject("box", "left_panda_hand_tcp")` 理解成“箱子只由左手抓住”。MoveIt 的 AttachedCollisionObject 在运动学树里只能挂到一个 link；右手的接触许可要通过 `allowCollisions("box", touch_links, true)` 和 ACM 表达。真正闭链刚性共持的接触力分配不由 MTC attach 解决，而应交给 D03/D10 的 Object Impedance 或专门的闭链约束控制器。

同样，不要把 `both_arms` + `MoveRelative` 理解成“自动保持共持物体刚性闭链”。它只在 14D 关节空间里生成一条联合轨迹并做碰撞检查，不会保证 $T_{\text{left tcp}}^{-1}T_{\text{right tcp}}$ 沿轨迹恒定。工程上应采用三种更严格的写法之一：用 D02 的受限 14D 规划和自定义相对位姿约束；逐 waypoint 生成双 TCP 目标后拼接成 14D 目标；或至少在 MTC 解出来后逐点检查左右 TCP 相对位姿误差，超出阈值就拒绝执行。

**Merger stage 的内部机制**：

1. 分别规划两个子 stage（各自 7D）
2. 对两条轨迹做**时间对齐**：找最长的一条，短的用 hold（保持末尾状态）补齐
3. **碰撞检查**：在合并后的 14D 空间中检查每个时间点的碰撞
4. 如果碰撞 → 重新规划子 stage（随机化初始构型）→ 重试

> **本质洞察**：Merger 不是简单的"拼接"——它是一个**后验碰撞检查器**。它允许两臂各自独立高效规划（7D），然后在合并时检查 14D 碰撞。如果碰撞则重试。这种"投机执行 + 回滚"的策略在大多数情况下比直接 14D 规划更快，因为大部分独立规划的路径不会碰撞。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MTC Merger 中子 stage 使用 both_arms 组
   错误做法：approach_L->setGroup("both_arms")
   现象：Merger 尝试在两个子 stage 中各做 14D 规划，极慢且冲突
   正确做法：子 stage 各自用 left_arm / right_arm，
            Merger 负责合并和碰撞检查。
            只在需要联合运动的 stage（如 lift）用 both_arms

💡 概念误区：认为"MTC 等于顺序执行"
   新手想法："MTC 的 stages 是一个接一个执行的"
   实际上：Merger stage 内部的子 stage 是并行规划的。
   MTC 的 stage 流是逻辑顺序，不是时间顺序。
   MTC 的真正强大之处在于逆向推理（backward reasoning）：
   每个 stage 可以向后传播约束，自动确定中间状态。
```

### 练习

1. **[编程]** 用 MTC 编写双臂 pick-lift-transport-place 管线。使用 Merger stage 实现两臂同步 approach。验证完整管线在 RViz 中执行。
2. **[编程]** 扩展管线添加 handover 功能：左臂拿起物体 → lift → 右臂接近 → 左手打开 → 右手关闭 → 右臂搬运。
3. **[思考题]** MTC 的 Merger 通过时间对齐处理两臂速度不同的情况。但如果两臂需要**速度同步**（如共持物体搬运时两末端必须同速同向），Merger 的时间对齐是否足够？需要什么额外约束？

---

## D9.6 ros2_control 双臂同步执行 ⭐⭐

### 动机——规划完成后的"最后一公里"

MoveIt2 规划生成了轨迹，但轨迹执行是由 ros2_control 负责的。双臂系统中，两个 `JointTrajectoryController`（JTC）各自接收一条 7D 轨迹。问题是：**如何保证两个 JTC 精确同步启动？**

如果左臂 JTC 比右臂 JTC 早启动 50ms，在 50ms 内只有左臂在运动——对于共持物体来说，这 50ms 的单臂运动可能导致物体掉落或关节过载。

### 三种同步策略

#### 策略一：合并轨迹（推荐）

把 left_traj + right_traj 合并为一条 14D 轨迹，交给一个 14 关节的 JTC。

```yaml
# controllers.yaml — 合并方案
controller_manager:
  ros__parameters:
    update_rate: 1000  # 1 kHz
    
    both_arms_controller:
      type: joint_trajectory_controller/JointTrajectoryController

both_arms_controller:
  ros__parameters:
    joints:
      - left_panda_joint1
      - left_panda_joint2
      - left_panda_joint3
      - left_panda_joint4
      - left_panda_joint5
      - left_panda_joint6
      - left_panda_joint7
      - right_panda_joint1
      - right_panda_joint2
      - right_panda_joint3
      - right_panda_joint4
      - right_panda_joint5
      - right_panda_joint6
      - right_panda_joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
```

**优势**：一个控制器 → 一个 action server → 共享同一个控制器时间基准。它能避免两个 JTC action 分别启动造成的大偏差，但真机上仍要测量总线调度、驱动插补和时钟同步带来的 skew。

#### 策略二：并行执行器

两个独立 JTC，通过 MoveIt2 的 `TrajectoryExecutionManager` 并行调度。

```yaml
# controllers.yaml — 并行方案
controller_manager:
  ros__parameters:
    update_rate: 1000
    
    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

left_arm_controller:
  ros__parameters:
    joints: [left_panda_joint1, left_panda_joint2, left_panda_joint3,
             left_panda_joint4, left_panda_joint5, left_panda_joint6,
             left_panda_joint7]
    command_interfaces: [position]
    state_interfaces: [position, velocity]

right_arm_controller:
  ros__parameters:
    joints: [right_panda_joint1, right_panda_joint2, right_panda_joint3,
             right_panda_joint4, right_panda_joint5, right_panda_joint6,
             right_panda_joint7]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

```yaml
# moveit_controllers.yaml — 告诉 MoveIt2 如何调度
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
moveit_simple_controller_manager:
  controller_names:
    - left_arm_controller
    - right_arm_controller
  left_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints: [left_panda_joint1, left_panda_joint2, left_panda_joint3,
             left_panda_joint4, left_panda_joint5, left_panda_joint6,
             left_panda_joint7]
  right_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints: [right_panda_joint1, right_panda_joint2, right_panda_joint3,
             right_panda_joint4, right_panda_joint5, right_panda_joint6,
             right_panda_joint7]
```

**同步精度**：±5-10 ms（取决于 DDS 通信和内核调度）。

#### 策略三：双臂导纳控制

为需要力控的场景，配置双臂各自的 `admittance_controller`：

```yaml
left_admittance_controller:
  ros__parameters:
    joints: [left_panda_joint1, left_panda_joint2, left_panda_joint3,
             left_panda_joint4, left_panda_joint5, left_panda_joint6,
             left_panda_joint7]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    ft_sensor:
      name: left_ft_sensor
      frame:
        id: left_panda_link8
    admittance:
      selected_axes: [true, true, true, true, true, true]
      mass: [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
      damping_ratio: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
      stiffness: [200.0, 200.0, 200.0, 50.0, 50.0, 50.0]

right_admittance_controller:
  ros__parameters:
    # 类似配置，连接 right_ft_sensor
    joints: [right_panda_joint1, right_panda_joint2, right_panda_joint3,
             right_panda_joint4, right_panda_joint5, right_panda_joint6,
             right_panda_joint7]
    # ...
```

### 同步策略选型

| 策略 | 同步精度 | 实现复杂度 | 适用场景 |
|------|---------|-----------|---------|
| 合并轨迹 | 控制器时间基准同步（总线/驱动 skew 需实测） | 低 | 通用，推荐默认方案 |
| 并行执行器 | ±5-10 ms | 中 | 独立操作，需要各自 admittance |
| 同步触发 | ±1 ms | 高（需自定义） | 精密同步（工业装配） |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：合并轨迹方案下用两个独立控制器
   错误做法：both_arms 规划生成 14D 轨迹，但控制器是两个独立 7D JTC
   现象：MoveIt2 找不到匹配的控制器，执行失败
   根本原因：MoveIt2 的 TrajectoryExecutionManager 按关节名匹配控制器，
            14D 轨迹需要一个包含全部 14 关节的控制器
   正确做法：要么用一个 14 关节 JTC（合并方案），
            要么在 moveit_controllers.yaml 中配两个控制器并行调度

🧠 思维陷阱：认为"仿真里同步没问题就行"
   新手想法："MuJoCo 里两臂同步完美，上真机也一样"
   实际上：仿真是确定性的——两个控制器共享同一个仿真时钟。
   真机中两个控制器通过 EtherCAT/CAN 与不同硬件通信，
   通信延迟不同 → 真实同步精度可能差 5-10ms。
   对共持物体等精密协调任务，需要在真机上测量实际同步精度。
```

### 练习

1. **[编程]** 配置合并轨迹方案和并行执行器方案，对比两臂末端位姿同步误差。在 MuJoCo 仿真中测量执行期间最大末端位姿偏差。
2. **[编程]** 为双 Franka 各配一个 admittance_controller，设置 z 轴柔顺（K_z=200，其余 K=1000）。用 rqt 发布 wrench 话题模拟外力，验证两臂独立响应。
3. **[思考题]** 如果两臂共持物体移动，你应该用 `both_arms` 组规划还是用 Object Impedance 力控？提示：几何简单 + 无外力 → 规划；接触丰富 + 力敏感 → 力控；长距离搬运 → 先规划路径再力控跟踪。

---

## D9.7 仿真环境搭建 ⭐⭐

### 动机——没有仿真就没有调试

双臂系统的复杂度远超单臂——14 个关节、跨臂碰撞、协调运动——在真机上调试效率极低且有碰撞风险。仿真是必不可少的开发前置。

### MuJoCo + mujoco_ros2_control

对于快速原型和 RL 训练，MuJoCo 是首选（更快的物理仿真、更稳定的接触模型）。

```python
# MuJoCo 双 Franka 环境
import mujoco
import mujoco.viewer

# 从 URDF 加载（MuJoCo 3.x 原生支持 URDF 导入）
model = mujoco.MjModel.from_xml_path("dual_panda.urdf")
data = mujoco.MjData(model)

if model.nu == 0:
    raise RuntimeError(
        "MuJoCo URDF import produced no actuators (model.nu == 0). "
        "URDF joints are kinematic structure only; add MJCF actuators or "
        "use your conversion script's --add-actuators option.")

# 验证关节数和名称。注意 model.nq 是 qpos 维度，不等于 actuator 数。
print(f"qpos 维度: {model.nq}")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    qadr = model.jnt_qposadr[i]
    dadr = model.jnt_dofadr[i]
    print(f"  Joint {i}: {name}, qpos={qadr}, dof={dadr}")

# 不要假设 ctrl[0:7] 一定对应左臂。actuator 顺序由 MJCF <actuator> 定义决定。
def actuator_id(name: str) -> int:
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if act_id < 0:
        raise KeyError(f"Actuator {name!r} not found; print mjOBJ_ACTUATOR names and fix the map")
    return act_id

# 示例命名，实际项目必须以 mj_id2name 打印结果为准。
left_actuator_names = [f"left_panda_joint{i+1}_actuator" for i in range(7)]
right_actuator_names = [f"right_panda_joint{i+1}_actuator" for i in range(7)]
left_act_ids = [actuator_id(name) for name in left_actuator_names]
right_act_ids = [actuator_id(name) for name in right_actuator_names]

# position actuator 示例；如果是 motor/general actuator，应换成外部 PD 计算出的 tau。
target_left_ctrl = target_left
target_right_ctrl = target_right

# 简单位置控制测试
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # data.ctrl 是 actuator 控制输入，不等价于 qpos。
        # position actuator: ctrl 是位置目标；motor/general actuator: ctrl 是力/力矩类输入。
        data.ctrl[left_act_ids] = target_left_ctrl
        data.ctrl[right_act_ids] = target_right_ctrl
        
        mujoco.mj_step(model, data)
        viewer.sync()
```

### Gazebo Harmonic + gz_ros2_control

与 MoveIt2 联调时，Gazebo Harmonic 是最成熟的选择：

```python
# gazebo_dual_panda.launch.py（简化版）
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    # 2. Spawn 双臂模型
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'dual_panda'],
    )
    
    # 3. 时钟桥
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    
    return LaunchDescription([gazebo, spawn, bridge])
```

### 仿真选型决策

| 仿真器 | 优势 | 劣势 | 适用 |
|--------|------|------|------|
| MuJoCo | 极快、接触稳定、RL 生态好 | ROS2 集成需额外工作 | 快速原型、RL 训练 |
| Gazebo Harmonic | 原生 ROS2 集成、传感器仿真丰富 | 接触不如 MuJoCo | MoveIt2 联调、完整系统测试 |
| Isaac Sim/Lab | GPU 并行、渲染真实 | 重量级、需 NVIDIA GPU | 大规模 RL、视觉策略训练 |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MuJoCo 导入 URDF 时关节顺序变化
   错误做法：假设 MuJoCo 的关节顺序与 URDF 中定义一致
   现象：发送到左臂关节 1 的指令实际控制了右臂关节
   根本原因：MuJoCo 在解析 URDF 时可能重排关节顺序
   正确做法：导入后用 mj_id2name 打印确认关节名和索引的对应关系

⚠️ 编程陷阱：URDF 导入后没有 actuator
   错误做法：以为 URDF 的 joint 会自动生成可写 data.ctrl
   现象：model.nu == 0，data.ctrl 为空，控制指令完全无效
   根本原因：URDF 只描述关节和连杆，MuJoCo actuator 是执行器模型，必须显式添加
   正确做法：在 MJCF 中添加 <actuator>，或用转换脚本生成 actuator，并核对 ctrl 语义

⚠️ 编程陷阱：把 ctrl 当成关节位置
   错误做法：data.ctrl[i] = q_des，默认认为所有 actuator 都是位置伺服
   现象：如果 actuator 是 motor/general，机器人收到的是力/力矩类输入而不是位置目标
   正确做法：先确认 actuator 类型和 gain/bias 配置，再决定 ctrl 写 q_des、tau 还是归一化命令

💡 概念误区：认为"仿真配置好了就可以直接上真机"
   新手想法："Gazebo 里跑通了，真机也能跑"
   实际上：仿真与真机之间存在 sim-to-real gap（P02 详述）。
   关节摩擦、编码器分辨率、通信延迟、重力补偿误差——
   这些在仿真中不存在或被简化的因素，在真机上可能导致完全不同的行为。
   正确做法：仿真验证逻辑正确性 → 真机先低速测试 → 逐步提速。
```

### Gazebo 双臂仿真环境——完整 Launch 配置

```python
# launch/dual_panda_gazebo.launch.py
# 完整的双臂 Gazebo Harmonic + MoveIt2 联调 launch 文件

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg_share = get_package_share_directory('mini_dualarm')
    
    # ============ 参数 ============
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ============ 1. Robot Description ============
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'config', 'dual_panda.urdf.xacro'),
        ' use_gazebo:=true',
        ' use_sim_time:=', use_sim_time,
    ])
    robot_description = {'robot_description': robot_description_content}
    
    # ============ 2. Gazebo Harmonic ============
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v 3 empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    
    # Spawn 模型
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dual_panda',
            '-z', '0.0',
        ],
        output='screen',
    )
    
    # ============ 3. ros2_control ============
    # robot_state_publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Controller Manager（Gazebo 启动后自动加载 gz_ros2_control）
    # 加载并激活控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )
    
    load_both_arms_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'both_arms_controller'],
        output='screen',
    )
    
    # 控制器需要在 spawn 后加载
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )
    delay_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_both_arms_controller],
        )
    )
    
    # ============ 4. 时钟桥 ============
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    
    # ============ 5. MoveIt2 ============
    moveit_config = {
        'robot_description': robot_description_content,
        'robot_description_semantic': Command([
            'cat ', os.path.join(pkg_share, 'config', 'dual_panda.srdf')
        ]),
        'robot_description_kinematics': load_yaml(
            os.path.join(pkg_share, 'config', 'kinematics.yaml')
        ),  # 必须用 load_yaml() 解析为字典，PathJoinSubstitution 只返回路径字符串
        'use_sim_time': use_sim_time,
    }
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            moveit_config,
            os.path.join(pkg_share, 'config', 'ompl_planning.yaml'),
            load_yaml(os.path.join(pkg_share, 'config',
                                   'moveit_controllers.yaml')),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    
    # ============ 6. RViz ============
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'moveit.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_entity,
        bridge,
        delay_jsb,
        delay_ctrl,
        move_group_node,
        rviz_node,
    ])
```

### 硬件驱动集成——双 ros2_control 实例管理

从仿真迁移到真机时，核心变更点是 `ros2_control` 的 `<hardware>` 插件。双臂系统有两种硬件集成策略：

**策略 A：单 HardwareInterface 管理 14 关节**

```yaml
# ros2_controllers_real.yaml — 单实例管理两臂
controller_manager:
  ros__parameters:
    update_rate: 1000
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    both_arms_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# URDF 中：
# <ros2_control name="dual_panda_hw" type="system">
#   <hardware>
#     <plugin>franka_hardware/FrankaDualArmInterface</plugin>
#     <param name="left_robot_ip">192.168.1.10</param>
#     <param name="right_robot_ip">192.168.1.11</param>
#   </hardware>
#   <joint name="left_panda_joint1">...</joint>
#   ...14 joints...
# </ros2_control>
```

**策略 B：双 HardwareInterface 各管 7 关节（更常见）**

```yaml
# ros2_controllers_real.yaml — 双实例各管一臂
controller_manager:
  ros__parameters:
    update_rate: 1000
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# URDF 中定义两个独立 ros2_control 块：
# <ros2_control name="left_panda_hw" type="system">
#   <hardware>
#     <plugin>franka_hardware/FrankaHardwareInterface</plugin>
#     <param name="robot_ip">192.168.1.10</param>
#   </hardware>
#   <joint name="left_panda_joint1">...</joint>
#   ...7 joints (left)...
# </ros2_control>
# <ros2_control name="right_panda_hw" type="system">
#   <hardware>
#     <plugin>franka_hardware/FrankaHardwareInterface</plugin>
#     <param name="robot_ip">192.168.1.11</param>
#   </hardware>
#   <joint name="right_panda_joint1">...</joint>
#   ...7 joints (right)...
# </ros2_control>
```

**两种策略对比**：

| 维度 | 策略 A (单实例) | 策略 B (双实例) |
|------|---------------|---------------|
| 同步精度 | 控制器时间基准同步；总线/驱动 skew 需实测 | ±0.5-1 ms（各自 update） |
| 驱动复杂度 | 需定制双臂驱动 | 直接复用厂商单臂驱动 |
| 故障隔离 | 一臂错误影响另一臂 | 独立故障隔离 |
| 适用场景 | 精密协调（工业装配） | 通用双臂操作 |
| 厂商支持 | 少（需 fork 驱动） | 多（官方驱动直接可用） |

### 调试方法论

**Planning Scene 可视化**：

```cpp
// 打印 Planning Scene 中的碰撞信息——调试规划失败的利器
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

void debug_planning_scene(
        const planning_scene::PlanningScenePtr& scene,
        const moveit::core::RobotState& state) {
    // 检查自碰撞
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.contacts = true;
    req.max_contacts = 100;
    scene->checkSelfCollision(req, res, state);
    
    if (res.collision) {
        RCLCPP_ERROR(rclcpp::get_logger("debug"), 
            "Self-collision detected! %zu contacts:", res.contact_count);
        for (const auto& [pair, contacts] : res.contacts) {
            RCLCPP_ERROR(rclcpp::get_logger("debug"),
                "  %s <-> %s (depth: %.4f m)",
                pair.first.c_str(), pair.second.c_str(),
                contacts[0].depth);
        }
    }
    
    // 打印 ACM 状态
    const auto& acm = scene->getAllowedCollisionMatrix();
    RCLCPP_INFO(rclcpp::get_logger("debug"), "ACM entries: %zu", 
                acm.getSize());
}
```

**轨迹回放验证**：

```python
# 将 MoveIt2 规划的轨迹回放到 MuJoCo 中验证
import numpy as np
from trajectory_msgs.msg import JointTrajectory

def replay_trajectory_in_mujoco(env, trajectory_msg):
    """将 ROS2 JointTrajectory 消息回放到 MuJoCo"""
    joint_names = trajectory_msg.joint_names
    
    results = {'positions': [], 'errors': [], 'times': []}

    sim_dt = getattr(env, "dt", None)
    if sim_dt is None:
        sim_dt = env.model.opt.timestep

    prev_time = 0.0
    prev_target = env.get_joint_positions().copy()

    for point in trajectory_msg.points:
        target = np.array(point.positions)
        t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        segment_dt = max(t - prev_time, 0.0)

        # 按 waypoint 时间差和仿真 dt 积分；段内线性插值，避免固定 substeps
        # 导致快慢轨迹都用同一仿真时长。
        if segment_dt <= 1e-12:
            env.set_joint_targets(target)
            env.step()
        else:
            n_steps = max(1, int(np.ceil(segment_dt / sim_dt)))
            for k in range(1, n_steps + 1):
                alpha = min(k * sim_dt / segment_dt, 1.0)
                cmd = (1.0 - alpha) * prev_target + alpha * target
                env.set_joint_targets(cmd)
                env.step()
        
        actual = env.get_joint_positions()
        error = np.linalg.norm(target - actual)
        
        results['positions'].append(actual.copy())
        results['errors'].append(error)
        results['times'].append(t)

        prev_target = target
        prev_time = t
    
    max_error = max(results['errors'])
    mean_error = np.mean(results['errors'])
    print(f"Replay: max_err={max_error:.4f} rad, "
          f"mean_err={mean_error:.4f} rad")
    
    return results
```

### 练习

1. **[编程]** 搭建 MuJoCo 双 Franka 环境：从 URDF 导入，验证 14 个关节读写正常。实现关节 PD 控制，让两臂同时做正弦跟踪。
2. **[编程]** 搭建 Gazebo Harmonic 双 Franka + ros2_control 环境。配置 both_arms_controller，用 RViz MotionPlanning 插件验证 MoveIt2 规划和执行。
3. **[编程]** 实现 `replay_trajectory_in_mujoco` 函数：将 MoveIt2 规划的 `both_arms` 轨迹录制为 rosbag，提取 `JointTrajectory` 消息，在 MuJoCo 中回放并记录跟踪误差。

---

## D9.8 MoveIt2 2026 新特性与 Servo 2.0 双臂联动 ⭐⭐⭐

### MoveIt2 Jazzy/Rolling 新特性对双臂的影响 ⭐⭐⭐

MoveIt2 在 ROS2 Jazzy (2024) 和 Rolling (2025-2026) 版本中引入了多项对双臂系统有重大影响的改进：

**Multi-Robot Support 重构**：MoveIt2 Rolling 引入了 `robot_model_loader` 的多实例支持——允许在同一个 `move_group` 节点中加载多个独立的 URDF/SRDF 模型。这对双臂系统的意义在于：不再需要把两臂合并到一个 URDF 中，而是可以保持各臂 URDF 独立，在运行时动态组合。优势是维护更简单（上游 URDF 更新不需要重新合并），劣势是跨臂碰撞检查需要额外配置。

**OMPL 2.0(规划中)集成**：OMPL 2.0 引入了 Informed RRT* 的改进变体和 Experience-based Planning——规划器可以利用历史成功路径加速后续规划。对 14D 双臂规划的提速效果显著：D9.3 中讨论的 both_arms 规划超时问题（14D 空间太大）可以通过 experience database 缓解——首次规划可能需要 5 秒，但相似构型下的后续规划可降至 <0.5 秒。

**Parallel Planning API**：MoveIt2 2025+ 提供了 `planMultipleGoals()` API，允许对同一请求并行运行多个规划器（RRT*, BIT*, PRM*），取最先成功的结果。这直接解决了 D9.3 中"同步规划选型困难"的问题——不必在规划器之间做选择，而是让它们赛跑。

### Servo 2.0 双臂联动 ⭐⭐⭐

MoveIt Servo（实时关节速度/笛卡尔速度命令接口）在 2.0 版本中增加了多组联动支持——可以同时对 `left_arm` 和 `right_arm` 发送笛卡尔速度命令，Servo 内部保证两组的速度指令在同一控制周期内下发。这对双臂遥操作（D08）和 Object Impedance 实时控制（D03.4）至关重要。

Servo 2.0 的双臂模式支持两种协调方式：

| 模式 | 输入 | 适用场景 |
|------|------|---------|
| **独立模式** | 两组独立的 twist 命令 | D02 独立任务 |
| **协调模式** | 物体 twist + 相对 twist | D03 Object Impedance |

协调模式下，Servo 内部将物体速度分解为两臂速度（利用 D01 的增广 Jacobian），并执行实时碰撞检查——如果预测到碰撞，自动降速或停止，比 D9.3 的离线规划更适合动态场景。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Servo 2.0 的 twist 命令坐标系不一致
   错误做法：左臂和右臂的 twist 都在 base_link 坐标系下给出
   现象：两臂运动方向相反（如都向 x+ 运动，但右臂实际向 x- 运动）
   根本原因：Servo 默认将 twist 解释为 end-effector 坐标系下的命令，
            而左右臂的 end-effector 坐标系朝向可能相反
   正确做法：统一使用 base_link 坐标系，并在 Servo 配置中设置
            frame_id: "base_link"
```

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关联章节 |
|--------|---------|------|---------|
| 双臂 URDF/Xacro | prefix 参数化、布局设计、维护策略 | ⭐⭐ | P01 URDF |
| SRDF 配置 | both_arms 组、ACM、named states、末端执行器 | ⭐⭐ | M14 MoveIt2 |
| 同步 vs 异步规划 | 14D 联合 vs 两个 7D、加速策略、选型决策 | ⭐⭐ | D02 双臂规划 |
| ACM 优化 | 碰撞对削减方法、动态 ACM、安全原则 | ⭐⭐ | M04 碰撞检测 |
| MTC 双臂 | Merger stage、协同搬运管线、逆向推理 | ⭐⭐⭐ | M14 MTC |
| ros2_control 同步 | 合并轨迹/并行/导纳三策略、选型 | ⭐⭐ | M12 ros2_control |
| 仿真环境 | MuJoCo/Gazebo/Isaac Sim 选型与搭建 | ⭐⭐ | P02 sim-to-real |
| MoveIt2 2026 新特性 | Parallel Planning、Experience-based、Servo 2.0 双臂联动 | ⭐⭐⭐ | — |

## 累积项目：本章新增模块

**Mini-DualArm 项目进度**：

```
D01-D07: 理论基础 ✓
D08: 遥操作数据采集 ✓
D09 新增:
  ├─ 双臂 URDF/Xacro (dual_panda.urdf.xacro)
  ├─ SRDF + ACM 优化 (dual_panda.srdf)
  ├─ MoveIt2 配置全套 (kinematics/controllers/ompl_planning)
  ├─ MTC 双臂搬运管线 (pick-lift-transport-place)
  ├─ MuJoCo/Gazebo 仿真环境
  └─ [下一步] D10: 综合实战集成
```

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| MoveIt2 官方文档 "Dual Arm Panda Tutorial" | ⭐⭐ | 官方双臂配置教程 |
| ros2_control 官方文档 "Multi-interface Robot" | ⭐⭐ | 多接口/多控制器配置 |
| Coleman et al. (2014) "Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study" | ⭐⭐⭐ | MoveIt 的设计哲学 |
| MoveIt Task Constructor GitHub Wiki | ⭐⭐⭐ | MTC 的 stage 类型和用法 |
| Aertbelien et al. (2026) "Simplifying ROS2 Controllers" arXiv 2601.08514 | ⭐⭐⭐⭐ | 模块化参考生成器 |
| PickNik MoveIt Pro 文档 | ⭐⭐⭐ | 双臂 MTC 工业化参考 |

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MoveIt2 找不到 both_arms 组 | SRDF 未定义或未加载 | 1.检查 SRDF 文件 2.确认 launch 加载路径 3.`ros2 param get` | D9.2 |
| both_arms 规划超时 | 14D 空间太大 / ACM 未优化 | 1.增加 planning_time 2.换 BIT* 3.优化 ACM 4.fallback 异步 | D9.3, D9.4 |
| 两臂执行不同步 | 控制器配置不匹配 | 1.确认是一个 14D JTC 还是两个 7D 2.检查 action topic 3.打印时间戳 | D9.6 |
| MTC Merger 失败 | 子 stage 合并后碰撞 | 1.检查 approach 方向 2.增大 max_solutions 3.检查 ACM 4.调整起始构型 | D9.5 |
| Gazebo 中关节不响应 | gz_ros2_control 配置错误 | 1.检查 hardware plugin 2.确认 joint 名称匹配 3.检查 update_rate | D9.7 |

---
