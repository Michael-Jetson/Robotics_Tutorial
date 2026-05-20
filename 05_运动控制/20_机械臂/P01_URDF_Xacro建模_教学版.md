> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# P01 URDF / Xacro 机器人建模

> **本章定位**：URDF 是 ROS2 / MoveIt2 / Gazebo / MuJoCo / Drake / Isaac Sim 共用的机器人描述标准。本章从"为什么不能把机器人尺寸硬编码在代码里"出发，逐层构建 URDF 的 link/joint/transmission 语义，引入 Xacro 宏系统消除重复，完成一个 7-DOF 机械臂的完整描述文件，并实现 URDF → SDF → MJCF 多格式转换。
>
> **性质**：✅ **全方向共享**——机械臂、腿足、无人机、RL 均以本章为建模起点。

| 属性 | 值 |
|------|----|
| **难度** | ⭐ |
| **周数** | 1.0 周 |
| **前置依赖** | 02_基础/ROS2 Launch + TF2 基础 |
| **共享标记** | ✅ 全方向共享 |

---

## 前置自测

> 答不出 >= 2 题 → 先回 ROS2 基础章节复习

1. **TF2 是什么**：ROS2 中 TF2 的作用是什么？`/tf` 和 `/tf_static` 话题的区别是什么？（答不出 → 回顾 ROS2 TF2 基础）
2. **Launch 文件**：ROS2 launch 文件的核心作用是什么？如何通过 launch 参数在运行时切换配置？（答不出 → 回顾 ROS2 Launch 基础）
3. **XML 基础**：XML 的标签(tag)、属性(attribute)、嵌套(nesting)各是什么？给出一个包含这三者的最小例子。（答不出 → 任意 XML 教程 10 分钟即可）
4. **关节概念**：机械臂的"旋转关节"和"平移关节"在物理上有什么区别？一个 7-DOF 臂有几个独立运动自由度？（答不出 → 机器人学导论第 2 章）

---

## 本章目标

学完本章后，你应该能够：

1. **从零手写** 一个 7-DOF 机械臂的 URDF/Xacro 描述文件，包含 link/joint/transmission 全语义
2. **正确计算** 并嵌入惯性参数——理解为什么错误的惯量是仿真爆炸的头号原因
3. **使用 Xacro 宏系统** 做参数化模板，用同一个 macro 生成 6-DOF / 7-DOF 变体
4. **完成多格式转换**：URDF → SDF（Gazebo）→ MJCF（MuJoCo），并理解各格式的差异
5. **用 `robot_state_publisher` + `joint_state_publisher_gui`** 在 RViz 中验证模型正确性

---

## P01.1 URDF XML Schema——机器人的统一描述语言 ⭐

### 1.1 动机：为什么不能把机器人尺寸硬编码在代码里

假设你正在写一个机械臂控制程序。你可能会这样做：

```cpp
// 反面示例：硬编码机器人参数
const double link1_length = 0.3;
const double link2_length = 0.25;
const double link1_mass = 2.5;
// ... 几十个参数散落在代码各处
```

这会带来三个严重问题：

**问题 1：修改一个参数要改 N 个文件**。控制器里写了一份长度，仿真器里又写了一份，可视化代码里再写一份。改了一个忘改另一个，机器人在仿真器里和控制器里是两个不同的机器人——你调试三天都不会发现 bug 根源是参数不一致。

**问题 2：无法跨工具复用**。你的 C++ 代码能读这些常量，但 MoveIt2 规划器不知道你的机器人什么样；Gazebo 仿真器也不知道；MuJoCo 也不知道。每个工具都需要你用它自己的格式重新描述一遍机器人。

**问题 3：无法描述拓扑结构**。硬编码只能描述数值参数，无法表达"link2 通过一个旋转关节连接在 link1 上"这种结构信息。

URDF（Unified Robot Description Format）正是为解决这些问题而生。它由 Willow Garage 为 PR2 机器人创建，至今是 ROS1/2 生态的标准机器人描述格式。一份 URDF 文件就是一个 single source of truth——ROS2、MoveIt2、Gazebo、MuJoCo（通过转换）、Drake、Isaac Sim 都能读取同一份文件。坐标约定遵循 **REP-103**：右手系，X 前、Y 左、Z 上，SI 单位（米/千克/弧度/秒）。

> **跨领域类比**：URDF 之于机器人工具链，就像 HTML 之于 Web 浏览器。HTML 是一种声明式标记语言，描述"页面长什么样"，不同浏览器（Chrome / Firefox / Safari）各自渲染。URDF 描述"机器人长什么样"，不同工具链（RViz / Gazebo / MuJoCo）各自解析。相似之处在于：两者都不描述行为（HTML 不写业务逻辑，URDF 不写控制算法），都有限的表达能力需要扩展机制（HTML 有 CSS/JS，URDF 有 Xacro/Gazebo 插件）。不同之处在于：URDF 描述的对象有物理含义（惯量、关节限位），一个惯性参数错误就导致仿真爆炸，而 HTML 的 CSS 错误最多导致显示不好看。

> **反事实推理**：如果 ROS 生态没有 URDF 这样的统一格式会怎样？每个工具都定义自己的格式——MoveIt 用 JSON、Gazebo 用 SDF、MuJoCo 用 MJCF——你的机器人描述会有 N 份，改一处必须同步 N-1 处。这不是假设——这正是 2009 年 URDF 诞生前的状况，也是至今非 ROS 生态（如纯 MuJoCo 项目）仍面临的痛点。

> **本质洞察**：URDF 的核心价值不在于它的 XML 语法有多优雅（实际上很啰嗦），而在于它建立了一个**跨工具链的契约**——所有工具约定好"机器人描述长这样"，于是每个工具只需实现一个解析器，而不是 N 个格式互转器。

### 1.2 `<link>` 元素：视觉、碰撞、惯性三子树

一个 `<link>` 描述一个刚体，包含三个语义不同的子元素：

```xml
<link name="link1">
  <!-- 1. visual：渲染用几何体，影响你在 RViz/Gazebo 里看到的外观 -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/visual/link1.dae"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
  </visual>

  <!-- 2. collision：碰撞检测用几何体，通常是简化版本 -->
  <collision>
    <geometry>
      <mesh filename="package://my_robot/meshes/collision/link1.stl"/>
    </geometry>
  </collision>

  <!-- 3. inertial：质量和惯性张量，决定动力学行为 -->
  <inertial>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <mass value="2.5"/>
    <inertia ixx="0.01" ixy="0" ixz="0"
             iyy="0.01" iyz="0"
             izz="0.005"/>
  </inertial>
</link>
```

**设计决策**：visual 用高保真网格（DAE，含颜色纹理）获得逼真外观，collision 用简化凸包（STL）获得高效碰撞检测。两者分离是性能与视觉效果的工程权衡——碰撞检测算法的开销与面片数近似成正比，用原始的几万面 CAD 模型做碰撞检测会拖垮实时仿真。

### 1.3 `<joint>` 元素：六种关节类型

`<joint>` 连接两个 link，定义它们之间的运动关系。URDF 支持六种关节类型：

| 类型 | 运动 | 典型用途 | 是否需要 `<limit>` |
|------|------|---------|-------------------|
| `revolute` | 带限位旋转 | 机械臂关节 | 必须 |
| `continuous` | 无限位旋转 | 轮子 | 不需要 |
| `prismatic` | 带限位平移 | 线性导轨、升降台 | 必须 |
| `fixed` | 无运动 | 传感器安装、法兰 | 不需要 |
| `floating` | 6-DOF 自由 | 移动底盘（极少用） | 不需要 |
| `planar` | 平面 2-DOF | 几乎不用 | 不需要 |

```xml
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>   <!-- 旋转轴方向，在关节帧中定义 -->
  <limit lower="-2.97" upper="2.97"
         effort="87.0" velocity="2.175"/>
  <dynamics damping="0.01" friction="0.005"/>
</joint>
```

`<limit>` 子元素的关键属性：

- `effort`：最大力/力矩（N 或 N-m）。**设为 0 会阻止所有运动**——这是新手高频犯的错误，机器人在 Gazebo 里完全不动，日志里没有任何报错
- `velocity`：最大速度（m/s 或 rad/s）
- `lower`/`upper`：关节范围，**单位是弧度，不是度**

`<axis xyz="0 0 1"/>` 指定的旋转轴是在**关节帧**中定义的，不是世界帧。这个区分非常重要——如果 joint 的 `<origin>` 带有旋转分量，axis 方向在世界坐标系中看起来会不同于你直觉写下的方向。

### 1.4 实例：最小 2-link 机械臂 URDF

下面是一个最小但完整的 2-link 旋转臂——所有机械臂 URDF 的骨架：

```xml
<?xml version="1.0"?>
<robot name="two_link_arm">
  <link name="world"/>
  <joint name="fixed_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.05" length="0.1"/></geometry>
    </visual>
    <collision>
      <geometry><cylinder radius="0.05" length="0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>

  <link name="upper_arm">
    <visual>
      <geometry><box size="0.04 0.04 0.3"/></geometry>
      <origin xyz="0 0 0.15"/>
    </visual>
    <collision>
      <geometry><box size="0.04 0.04 0.3"/></geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15"/>
      <mass value="2.0"/>
      <inertia ixx="0.015" ixy="0" ixz="0"
               iyy="0.015" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.35" upper="2.35" effort="40" velocity="2.0"/>
  </joint>

  <link name="forearm">
    <visual>
      <geometry><box size="0.035 0.035 0.25"/></geometry>
      <origin xyz="0 0 0.125"/>
    </visual>
    <collision>
      <geometry><box size="0.035 0.035 0.25"/></geometry>
      <origin xyz="0 0 0.125"/>
    </collision>
    <inertial>
      <origin xyz="0 0 0.125"/>
      <mass value="1.5"/>
      <inertia ixx="0.008" ixy="0" ixz="0"
               iyy="0.008" iyz="0" izz="0.0006"/>
    </inertial>
  </link>
</robot>
```

验证：

```bash
sudo apt install liburdfdom-tools
check_urdf two_link_arm.urdf        # 输出: robot name is: two_link_arm  ...  4 links, 3 joints
urdf_to_graphviz two_link_arm.urdf   # 生成运动链 PDF 图
```

> **⚠️ Pitfall：惯性张量必须正定**
>
> 惯性张量 $I$ 的三个主轴惯量必须满足三角不等式：$I_{xx} + I_{yy} \geq I_{zz}$（对所有排列成立）。违反此条件意味着物理上不可能的质量分布——没有任何实心物体能产生这样的惯量。Gazebo 会在加载时警告或拒绝模型；即使加载成功，仿真也会表现为剧烈振荡或"爆炸"。
>
> ```xml
> <!-- 错误：违反三角不等式，ixx + iyy = 0.002 < izz = 0.01 -->
> <inertia ixx="0.001" ixy="0" ixz="0"
>          iyy="0.001" iyz="0" izz="0.01"/>
>
> <!-- 正确：满足所有三角不等式 -->
> <inertia ixx="0.01" ixy="0" ixz="0"
>          iyy="0.01" iyz="0" izz="0.005"/>
> ```

---

## P01.2 Xacro 宏系统——让 URDF 可维护 ⭐⭐

### 2.1 问题：原始 URDF 的重复灾难

一个 7-DOF 机械臂的纯 URDF 文件通常超过 500 行，其中 70% 以上是结构几乎相同的 link/joint 定义——只有名字、尺寸、质量不同。每次修改一个 link 模板（比如碰撞几何的包络策略），你要手动改 7 处。这不是"不方便"的问题，而是"一定会出错"的问题。

Xacro（XML macro）是 ROS 的 URDF 预处理器，提供四大机制消除重复。

> **跨领域类比**：Xacro 之于 URDF，就像 C 预处理器（`#define`/`#include`/`#ifdef`）之于 C 源码。`xacro:property` 对应 `#define`（常量替换），`xacro:macro` 对应函数宏（参数化代码生成），`xacro:include` 对应 `#include`（文件包含），`xacro:if` 对应 `#ifdef`（条件编译）。不同的是，Xacro 内嵌 Python 表达式求值，比 C 预处理器的纯文本替换更强大——你可以在 `${}` 内写 `sin(pi/4)` 这样的数学运算。

> **反事实推理**：如果不使用 Xacro 而直接手写纯 URDF 会怎样？一个 7-DOF 臂的纯 URDF 通常超过 500 行。假设你需要把所有 link 的碰撞几何从 mesh 改为简化凸包——你需要手动修改 7 个 link 的 `<collision>` 块。改到第 5 个时漏了一个属性，仿真中该 link 碰撞检测异常但日志里没有报错，你调试三天才发现。用 Xacro 的 macro，改一处模板即可全局生效。

### 2.2 `xacro:property`——变量与数学表达式

```xml
<xacro:property name="link_radius" value="0.04"/>
<xacro:property name="link1_length" value="0.3"/>

<!-- ${...} 内使用 Python 语法 -->
<origin xyz="0 0 ${link1_length / 2}" rpy="0 ${pi/2} 0"/>

<!-- 支持: sin, cos, atan2, sqrt, radians, degrees, min, max, round -->
<origin rpy="0 ${radians(45)} ${atan2(1, 2)}"/>

<!-- 从 YAML 文件加载参数——UR ROS2 Description 的核心模式 -->
<xacro:property name="params"
                value="${load_yaml('config/robot_params.yaml')}"/>
<cylinder radius="${params['link']['radius']}"/>
```

### 2.3 `xacro:macro`——参数化模板

```xml
<!-- 定义：一个通用的机械臂 link 宏 -->
<xacro:macro name="arm_link" params="name length radius mass">
  <link name="${name}">
    <visual>
      <geometry><cylinder radius="${radius}" length="${length}"/></geometry>
      <origin xyz="0 0 ${length/2}"/>
    </visual>
    <collision>
      <geometry><cylinder radius="${radius}" length="${length}"/></geometry>
      <origin xyz="0 0 ${length/2}"/>
    </collision>
    <inertial>
      <origin xyz="0 0 ${length/2}"/>
      <mass value="${mass}"/>
      <!-- 圆柱体惯量公式，沿 Z 轴 -->
      <inertia ixx="${(1.0/12)*mass*(3*radius*radius + length*length)}"
               ixy="0" ixz="0"
               iyy="${(1.0/12)*mass*(3*radius*radius + length*length)}"
               iyz="0"
               izz="${0.5*mass*radius*radius}"/>
    </inertial>
  </link>
</xacro:macro>

<!-- 调用：一行生成一个完整 link，包含正确的惯量计算 -->
<xacro:arm_link name="link1" length="0.3"  radius="0.04"  mass="2.0"/>
<xacro:arm_link name="link2" length="0.25" radius="0.035" mass="1.5"/>
<xacro:arm_link name="link3" length="0.25" radius="0.035" mass="1.5"/>
```

**块参数**用于传递 XML 子树——当不同 link 需要不同的 `<origin>` 时：

```xml
<xacro:macro name="inertial_cylinder" params="mass radius length *origin">
  <inertial>
    <xacro:insert_block name="origin"/>
    <mass value="${mass}"/>
    <inertia ixx="${(1.0/12)*mass*(3*radius*radius+length*length)}"
             ixy="0" ixz="0"
             iyy="${(1.0/12)*mass*(3*radius*radius+length*length)}"
             iyz="0"
             izz="${0.5*mass*radius*radius}"/>
  </inertial>
</xacro:macro>
```

### 2.4 `xacro:include` 与条件分支

```xml
<!-- 文件包含 -->
<xacro:include filename="$(find my_robot_description)/urdf/inertial_macros.xacro"/>

<!-- 条件分支：根据 launch 参数切换配置 -->
<xacro:arg name="use_camera" default="true"/>
<xacro:arg name="dof" default="7"/>

<xacro:if value="$(arg use_camera)">
  <xacro:include filename="camera.xacro"/>
</xacro:if>

<xacro:unless value="${dof == 6}">
  <!-- 仅在 7-DOF 模式下添加冗余关节 -->
  <xacro:arm_link name="link7" length="0.1" radius="0.03" mass="0.5"/>
</xacro:unless>
```

注意 `$(arg ...)` 读命令行参数，`${...}` 对 xacro property 求值。这两种语法的混淆是 Xacro 调试中的高频错误源。

### 2.5 设计模式：同一 macro 生成 6/7-DOF 变体

Universal Robots ROS2 Description 的核心思路——用 `load_yaml()` 加载型号参数，一个宏覆盖全系列：

```xml
<xacro:arg name="robot_model" default="ur5e"/>
<xacro:property name="cfg"
    value="${load_yaml('$(find my_robot)/config/'
                       + robot_model + '.yaml')}"/>

<xacro:macro name="arm_chain" params="config">
  <xacro:arm_link name="link1"
      length="${config['link_lengths'][0]}"
      radius="${config['link_radii'][0]}"
      mass="${config['link_masses'][0]}"/>
  <!-- ... 后续 link 同理，每个 link 一行调用 ... -->
</xacro:macro>
```

展开 Xacro 到纯 URDF：

```bash
xacro robot.urdf.xacro robot_model:=ur5e > /tmp/robot.urdf
```

---

## P01.3 惯性参数与 Mesh 管理 ⭐⭐

### 3.1 惯性参数的三条获取路径

惯性参数（质量、质心位置、惯性张量）的准确性直接决定仿真行为是否接近真实物理。三条路径，精度递增、难度递增：

**路径 1：几何近似（开发早期，快速迭代）**

将每个 link 近似为基本几何体，用公式直接计算：

```
长方体 (x, y, z, m):
  Ixx = (1/12) * m * (y^2 + z^2)
  Iyy = (1/12) * m * (x^2 + z^2)
  Izz = (1/12) * m * (x^2 + y^2)

圆柱 (r, h, m, 沿 Z 轴):
  Ixx = Iyy = (1/12) * m * (3*r^2 + h^2)
  Izz = (1/2) * m * r^2

球 (r, m):
  Ixx = Iyy = Izz = (2/5) * m * r^2
```

当质心不在 link 原点时，用**平行轴定理**偏移：$I'_{xx} = I_{xx} + m(d_y^2 + d_z^2)$。

注意 Xacro 中 `${1/12 * mass * ...}` 的写法——`1/12` 在 Python 3 中是浮点除法（返回 0.0833...），但如果你用的 xacro 版本底层是 Python 2 风格求值，`1/12` 可能得到整数 0。安全做法始终写 `1.0/12`。

**路径 2：CAD 导出 / Mesh 估算（开发中期）**

- **CAD 导出**：SolidWorks / Fusion 360 / Onshape 直接输出质量属性。准确性取决于 CAD 模型中的材料属性设置
- **MeshLab**：Filters → Quality Measure → Compute Geometric Measures，获取体积后乘以密度
- **trimesh（Python）**：`mesh = trimesh.load('link.stl'); mesh.density = 2700; print(mesh.moment_inertia)`

**路径 3：系统辨识 SysId（部署前精标定）**

在真实机器人上施加已知激励轨迹，通过最小二乘拟合辨识惯性参数。精度最高但需要真机。典型方法参见 Swevers et al. (1997) 和 Khalil & Dombre (2004)。

### 3.2 Mesh 文件的 LOD 策略

| 用途 | 格式 | 面片数 | 说明 |
|------|------|--------|------|
| Visual（渲染） | DAE / OBJ | 5k-50k | 含颜色、纹理、法线贴图 |
| Collision（碰撞） | STL | 500-2k | 简化凸包，追求碰撞检测速度 |

**生产级文件组织**（对齐 ros2_control_demos 和 UR ROS2 Description 的目录结构）：

```
my_robot_description/
├── urdf/
│   ├── robot.urdf.xacro         # 顶层入口：include 一切，声明 args
│   ├── inertial_macros.xacro    # 可复用惯量宏（box/cylinder/sphere）
│   └── ros2_control.xacro       # 硬件接口（sim/real 切换点）
├── meshes/
│   ├── visual/                  # DAE 文件（高保真渲染）
│   └── collision/               # STL 文件（简化碰撞）
├── config/
│   └── robot_params.yaml        # 按型号的参数
└── launch/
    ├── display.launch.py        # RViz 可视化验证
    └── sim.launch.py            # Gazebo 仿真
```

碰撞 mesh 简化工具：MeshLab 的 Quadric Edge Collapse Decimation 滤波器，或 CoACD（学习增强凸分解），将复杂形状分解为少量凸包的并集。

> **⚠️ Pitfall：mesh scale 单位 m vs mm**
>
> CAD 软件（SolidWorks、Fusion 360）默认使用毫米，URDF 的 SI 单位是米。忘记转换的后果：
>
> ```xml
> <!-- 错误：CAD 导出单位是 mm，机器人在 RViz 中有几百米大 -->
> <mesh filename="package://my_robot/meshes/link.stl"/>
>
> <!-- 正确：添加 scale 因子将 mm 转为 m -->
> <mesh filename="package://my_robot/meshes/link.stl"
>       scale="0.001 0.001 0.001"/>
> ```
>
> **诊断方法**：在 MeshLab 中打开 STL，查看 Bounding Box 尺寸。一个本应 0.3m 的 link 若显示 300 单位长，说明源文件单位是 mm。

---

## P01.4 多格式转换——URDF 不是终点 ⭐⭐

### 4.1 为什么需要多格式

URDF 是 ROS 生态的标准，但不同仿真器有各自的原生格式：

| 仿真器 | 原生格式 | URDF 支持方式 |
|--------|---------|--------------|
| Gazebo Harmonic | SDF | 自动转换或 `gz sdf -p` 显式转换 |
| MuJoCo 3.x | MJCF | Python API (`mujoco.MjModel.from_xml_path()`) 转换（推荐方式） |
| Drake | URDF / SDF | 直接原生支持 |
| Isaac Sim | USD | URDF Importer 插件转换 |

### 4.2 URDF → SDF（Gazebo）

```bash
# 显式转换：查看 Gazebo 实际使用的 SDF 表示
gz sdf -p /tmp/robot.urdf > /tmp/robot.sdf

# 通常无需手动——Gazebo launch 时自动完成
# 但显式转换便于调试：查看 Gazebo 对你的 URDF 到底"理解"成了什么
```

### 4.3 URDF → MJCF（MuJoCo）

```python
# MuJoCo 3.x Python API 可直接加载 URDF 并导出 MJCF
import mujoco

model = mujoco.MjModel.from_xml_path('/tmp/robot.urdf')
mujoco.mj_saveLastXML('/tmp/robot.mjcf', model)
# 检查转换结果——尤其关注 MuJoCo 对 inertia 和 joint limit 的解释
```

MuJoCo 的编译器选项 `balanceinertia="true"` 会自动修正不满足三角不等式的惯量，但这意味着 MuJoCo 使用的惯量和你 URDF 中写的不一致——仿真行为可能偏离预期。最佳实践是在 URDF 中就保证惯量正确。

### 4.4 `robot_state_publisher` + `joint_state_publisher_gui` 验证

在做格式转换之前，先用 ROS2 原生工具验证 URDF 正确性：

```python
# display.launch.py —— 标准验证 launch 文件
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_description = ParameterValue(
        Command(['xacro ', '/path/to/robot.urdf.xacro']),
        value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2'),
    ])
```

`robot_state_publisher` 读取 `robot_description` 参数，订阅 `/joint_states`，用 KDL 运动学树计算并发布所有 link 的 TF 变换：fixed joints 发布到 `/tf_static`（transient-local QoS），movable joints 发布到 `/tf`。`joint_state_publisher_gui` 提供滑块界面让你手动拖动每个关节检查运动方向和范围。

**验证清单**（每个 URDF 必须过）：
1. 所有关节旋转轴方向正确（拖动滑块时 link 运动方向符合预期）
2. 关节限位范围合理（滑块极值对应物理极限）
3. 各 link 相对位置正确（无 link 悬空、穿透、或错位）
4. RViz 中开启 Mass Properties 可视化，惯量椭球形状合理（不会出现极度扁平或极度细长的椭球）

> **💡 URDF 不支持闭链——MJCF 的 `<equality><connect>` 可以**
>
> URDF 强制树状结构：每个 link 只有一个 parent joint。这意味着平行四连杆、达芬奇手术机器人等闭链机构无法直接表达。URDF 的 `<mimic>` 标签只是近似——让一个关节角度跟随另一个关节，但不是真正的运动学闭环约束。
>
> MuJoCo 的 MJCF 通过 `<equality>` 元素原生支持闭链：
>
> ```xml
> <equality>
>   <connect body1="link_a" body2="link_b" anchor="0 0 0.1"/>
> </equality>
> ```
>
> 这告诉物理引擎："link_a 和 link_b 在锚点处必须保持连接"，通过约束力维持闭链。MuJoCo 的底层是约束满足求解器，天然支持这类拓扑约束。如果你的机器人包含平行机构（如 ABB IRB 系列的平行四连杆传动），需要在 MJCF 层面处理。

---

## P01.5 `<transmission>` 与 ros2_control ⭐⭐

### 5.1 `<ros2_control>` 标签：连接描述与硬件

URDF 描述的是机器人的几何和物理属性。控制器还需要知道：每个关节通过什么接口接收命令？能读到什么状态？`<ros2_control>` 标签正是这个桥梁。

```xml
<ros2_control name="my_arm" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-2.97</param>
      <param name="max">2.97</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... 其余关节同结构 ... -->
</ros2_control>
```

### 5.2 Xacro 条件分支实现 sim-to-real 切换

通过 Xacro 条件分支切换 `<hardware><plugin>`，同一份描述文件服务多个部署目标。注意 Gazebo 仿真需要两层配置同时出现：`<gazebo>` 中加载 Gazebo system plugin，`<ros2_control>` 中把 hardware plugin 切到 `gz_ros2_control/GazeboSimSystem`。

```xml
<xacro:macro name="arm_ros2_control"
             params="use_mock:=false sim_gazebo:=false">
  <xacro:if value="${sim_gazebo}">
    <gazebo>
      <plugin filename="gz_ros2_control-system"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin"/>
    </gazebo>
  </xacro:if>

  <ros2_control name="arm" type="system">
    <hardware>
      <xacro:if value="${sim_gazebo}">
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </xacro:if>
      <xacro:if value="${use_mock}">
        <plugin>mock_components/GenericSystem</plugin>
      </xacro:if>
      <xacro:unless value="${use_mock or sim_gazebo}">
        <plugin>ur_robot_driver/URPositionHardwareInterface</plugin>
      </xacro:unless>
    </hardware>
    <!-- joint 接口定义在所有后端中保持一致 -->
  </ros2_control>
</xacro:macro>
```

这个模式的工程价值在于：**控制器代码零修改**，只通过 launch 参数切换底层硬件。本章只需理解这个结构；Controller Manager、硬件接口生命周期、实时线程等深度内容在 **M12 ros2_control 章节**展开。

---

## 练习

### 练习 1（A 型 -- 参数化 Xacro）⭐

**任务**：用 Xacro 为 Franka Panda 写参数化 URDF，支持通过 launch 参数切换三种配置：

- `config:=7dof`——标准 7-DOF 臂
- `config:=gripper`——7-DOF 臂 + 平行夹爪（用 `<mimic>` 实现联动）
- `config:=dual`——双臂配置（两个 7-DOF 臂固定在同一底座）

**要求**：
1. 所有 link 的惯性参数用 `inertial_macros.xacro` 中的宏计算，禁止硬编码惯量值
2. `check_urdf` 对三种配置均验证通过
3. 在 RViz 中开启 Mass Properties 可视化，确认惯量椭球合理

**验收标准**：`xacro robot.urdf.xacro config:=7dof | check_urdf /dev/stdin` 输出正确的 link/joint 数量。

### 练习 2（B 型 -- 多格式转换与仿真对比）⭐⭐

**任务**：将练习 1 的 URDF 转换为 SDF 和 MJCF，在 Gazebo Harmonic 和 MuJoCo 中各运行 10 秒自由落体仿真（移除固定底座约束），记录末端执行器轨迹。

**要求**：
1. 用 `gz sdf -p` 生成 SDF，用 MuJoCo Python API 加载 URDF 并导出 MJCF
2. 对比两个仿真器中关节位置时间曲线的差异，分析原因（积分器差异？默认接触参数？阻尼处理？）
3. 用 matplotlib 将对比结果绘制为图表

**验收标准**：两个仿真器的关节轨迹在同一张图上展示，附 200 字以上差异分析。

### 练习 3（思考题）

**问题**：为什么 URDF 不支持闭链？MJCF 的 `<equality><connect>` 如何绕过这一限制？

**提示方向**：
- 树结构中每个节点恰好有一个父节点。闭链意味着某个节点有两个父节点——这违反了树的定义
- `robot_state_publisher` 使用 KDL 树做正运动学。KDL 的递归算法假设输入是树——如果输入包含环，递归会怎样？
- MuJoCo 的约束满足求解器不依赖树结构——它把闭链约束当作约束方程求解，这和 KDL 的递归遍历是根本不同的计算范式
- URDF 的 `<mimic>` 只是关节角度的代数关系（$q_2 = m \cdot q_1 + o$），不是位置级的闭环约束。对于精密平行机构这种近似误差可能不可接受

---

## 本章小结

| 知识点 | 核心要点 |
|--------|---------|
| URDF 定位 | ROS 生态的 single source of truth，所有工具链的起点 |
| `<link>` | visual / collision / inertial 三子树，分离渲染与碰撞几何 |
| `<joint>` | 6 种类型；`revolute` 最常用；effort/velocity 设为 0 会阻止运动 |
| Xacro | property / macro / include / conditional 四大机制消除重复 |
| 惯性参数 | 几何近似 → CAD 导出 → SysId 三级精度；三角不等式是合法性底线 |
| Mesh 管理 | visual(DAE) vs collision(STL) 分离；注意 m/mm 单位转换 |
| 多格式转换 | URDF → SDF(gz sdf) / MJCF(mujoco)；URDF 不支持闭链 |
| ros2_control | `<hardware><plugin>` 决定后端；Xacro 条件分支实现 sim/real 切换 |

**下一章预告**：P02 sim-to-real 资产管道——从 CAD 到仿真到真机的完整管线，Domain Randomization，Docker 多阶段构建。

---

## 累积项目：本章新增模块

从本章开始构建一个贯穿机械臂方向的累积项目 **mini-manip**。每学完一章，给项目加一个模块。

### 项目概述

```
mini-manip/
├── urdf/
│   ├── panda.urdf.xacro         ← P01 本章核心产出
│   ├── macros/
│   │   ├── inertial_macros.xacro  ← 惯性参数计算宏
│   │   └── link_macros.xacro      ← link 模板宏
│   └── config/
│       ├── sim.yaml               ← 仿真模式 ros2_control 配置
│       └── real.yaml              ← 真机模式配置
├── meshes/
│   ├── visual/                    ← DAE 高精度渲染网格
│   └── collision/                 ← STL 简化碰撞网格
├── launch/
│   └── display.launch.py          ← robot_state_publisher + RViz
├── scripts/
│   ├── validate_urdf.sh           ← check_urdf + 三角不等式验证
│   └── convert_formats.py         ← URDF → SDF/MJCF 自动转换
└── CMakeLists.txt
```

### 本章新增任务

1. 用 Xacro 编写完整的 7-DOF 机械臂描述文件（参数化）
2. 实现 `validate_urdf.sh`：自动检查 URDF 语法 + 惯性参数三角不等式
3. 实现 `convert_formats.py`：一键从 URDF 生成 SDF 和 MJCF
4. 在 RViz 中用 `joint_state_publisher_gui` 验证所有关节运动正常

### 与后续章节的连接

| 后续章节 | 累积项目新增 |
|---------|------------|
| P02 sim-to-real | 添加域随机化配置、CI/CD 格式转换管线 |
| M01 Pinocchio | 用 Pinocchio 加载同一 URDF，验证 FK/ID |
| M04 碰撞检测 | 为 URDF 添加碰撞对优化、生成 SRDF |

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|:---:|------|
| [ROS 官方 URDF 规范](http://wiki.ros.org/urdf/XML) | ⭐ | 权威参考，所有标签和属性的完整定义 |
| [Xacro 官方文档](http://wiki.ros.org/xacro) | ⭐ | Xacro 语法和 Python 表达式支持 |
| [REP-103 标准坐标系约定](https://www.ros.org/reps/rep-0103.html) | ⭐ | ROS 坐标系约定（右手系、SI 单位） |
| [MuJoCo MJCF 文档](https://mujoco.readthedocs.io/en/latest/XMLreference.html) | ⭐⭐ | MJCF 格式完整参考，对比 URDF 差异 |
| [SDF 格式规范](http://sdformat.org/spec) | ⭐⭐ | Gazebo 原生格式，比 URDF 更强的表达能力 |
| [robot_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py) | ⭐ | 175+ 种机器人 URDF 即取即用，学习优秀建模范例 |
| Lynch & Park (2017) "Modern Robotics" Ch4 | ⭐⭐ | 运动链的数学基础（DH 参数 vs Product of Exponentials） |
| Coumans & Bai (2021) "MuJoCo Physics Engine" | ⭐⭐⭐ | MuJoCo 物理引擎设计，理解 MJCF 的设计动机 |
| [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) | ⭐⭐ | 从 Onshape CAD 自动导出 URDF 的工具 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 机器人在 Gazebo 中完全不动 | `<limit effort="0">` 阻止了所有力矩输出 | 1. 检查 URDF 中所有 revolute/prismatic 关节的 effort 值 2. 确保 effort > 0 3. 同时检查 velocity 限制是否合理 | P01.1 `<joint>` |
| 仿真中机器人"爆炸"（link 飞散） | 惯性参数错误（违反三角不等式或数量级错误） | 1. 用脚本检查所有 link 的三角不等式 $I_{xx}+I_{yy} \geq I_{zz}$ 等 2. 检查质量数量级（kg 不是 g） 3. 检查惯量单位（$\text{kg}\cdot\text{m}^2$） | P01.3 惯性参数 |
| RViz 中 mesh 不显示（白色或缺失） | mesh 路径错误或使用了绝对路径 | 1. 确认使用 `package://` 协议 2. 运行 `ros2 pkg prefix` 确认包路径 3. 检查 mesh 文件格式（visual 用 DAE，collision 用 STL） | P01.1 `<link>` |
| 模型尺寸异常（巨大或微小） | STL 文件单位是 mm 但 URDF 期望 m | 1. 在 MeshLab 检查 mesh bounding box 2. 添加 `scale="0.001 0.001 0.001"` 3. 或在 CAD 软件中以 m 为单位导出 | P01.3 Mesh 管理 |
| `check_urdf` 报告 link 数量不对 | Xacro 条件分支逻辑错误或宏参数未传递 | 1. 先 `xacro model.urdf.xacro > /tmp/expanded.urdf` 展开 2. 检查展开后的 URDF 3. 用 `urdf_to_graphviz` 可视化拓扑树 | P01.2 Xacro |
| URDF 转 MJCF 后关节方向反了 | URDF 和 MJCF 的关节轴约定不同 | 1. 对比转换前后零位 FK 结果 2. 检查 axis 方向是否被翻转 3. 在 MJCF 中手动修正 axis 或添加 post-processing 脚本 | P01.4 多格式转换 |

---

## P01.6 URDF 的来龙去脉与设计哲学 ⭐

### 历史溯源

URDF（Unified Robot Description Format）诞生于2009年前后，由Willow Garage团队为PR2双臂移动机器人开发。当时ROS1生态正在快速成长，团队需要一种**统一的机器人描述格式**，让rviz可视化、MoveIt运动规划、Gazebo仿真三个核心工具共享同一份模型文件。在此之前，每个工具都有自己的模型格式——这造成了大量重复劳动和不一致Bug。

URDF选择了XML作为载体，原因很实际：2009年的ROS生态以C++和Python为主，两者都有成熟的XML解析库（tinyxml/lxml），而JSON/YAML解析在当时还不够普及。XML的schema验证能力也让URDF可以在解析阶段就捕获结构错误。

### 核心设计选择

URDF做了三个关键的设计决策，理解它们有助于理解后续遇到的各种限制：

**决策一：仅支持树结构（tree-only）**。URDF中的link和joint构成一棵有根树，根节点通常是`base_link`。每个link最多有一个parent joint。这意味着**不支持闭链机构**（如平行四边形连杆、Stewart平台）。

为什么选择tree-only？因为树结构让正向运动学（FK）变成单次递归遍历——从根节点出发，逐级累乘齐次变换矩阵即可。闭链机构需要引入约束方程，用迭代求解器（如Newton-Raphson）才能计算位姿，计算复杂度和实现难度都大幅上升。MJCF格式支持闭链，代价是其FK求解器要复杂得多。

**决策二：单一坐标约定**。右手坐标系，长度单位米，角度单位弧度。没有单位转换选项，没有左手坐标系支持。这消除了一大类单位混乱Bug——CAD软件常用毫米，导入URDF时必须显式转换。

**决策三：纯运动学/几何描述**。URDF的核心职责是描述**link的几何与惯性属性**和**joint的运动约束**。它不负责描述执行器、传感器、控制器、接触参数等动态特性。这些在ROS生态中由其他配置文件补充（如ros2_control的YAML、Gazebo插件的SDF overlay）。

### URDF的局限性清单

以下是URDF无法原生表达的关键特性——在实际项目中，这些缺失常常成为痛点：

| 缺失特性 | 实际影响 | ROS生态的替代方案 |
|-----------|----------|-------------------|
| 闭链机构 | 无法描述平行连杆、Stewart平台 | Gazebo SDF的`<joint type="ball">` + constraint |
| 执行器模型 | 无法描述电机惯量、减速比、摩擦 | `<transmission>`标签（已半废弃）/ ros2_control YAML |
| 接触参数 | 无法描述摩擦系数、刚度、阻尼 | Gazebo SDF `<surface>` / MJCF `<geom>` |
| 腱/肌肉 | 无法描述欠驱动手指、仿生手 | MJCF `<tendon>` |
| 传感器规格 | 无法描述IMU噪声、相机内参 | Gazebo SDF `<sensor>` 插件 |
| 默认值继承 | 每个link/joint必须完整定义 | Xacro宏参数化 |
| 场景描述 | 无法描述光源、地面、物体 | SDF `<world>` / USD Stage |
| 柔性体 | 无法描述柔性关节、软体机器人 | MJCF `<flexcomp>` / SOFA |

### 格式生态对比

下表从八个维度对比当前主流机器人描述格式：

| 维度 | URDF | SDF | MJCF | USD |
|------|------|-----|------|-----|
| **闭链支持** | 不支持 | 支持 | 支持 | 支持（PhysX约束） |
| **执行器模型** | 仅transmission | 基本支持 | 丰富（电机、腱） | 通过Schema扩展 |
| **接触参数** | 不支持 | 支持 | 非常丰富 | 通过PhysX材质 |
| **传感器规格** | 不支持 | 插件式支持 | 基本支持 | 通过Sensor Schema |
| **主要仿真器** | - | Gazebo | MuJoCo | Isaac Sim / Omniverse |
| **ROS2支持** | 原生 | Gazebo桥接 | mujoco_ros2 | Isaac ROS |
| **工具链** | xacro/check_urdf | gz sdf | MuJoCo编译器 | USD Composer |
| **学习曲线** | 低 | 中 | 中-高 | 高 |

**当前趋势**：USD（Universal Scene Description）正在获得越来越大的关注度，NVIDIA Isaac Sim以USD为原生格式，支持从URDF/MJCF导入。但在ROS2生态中，URDF仍然是事实标准——robot_state_publisher、MoveIt2、Nav2都以URDF为输入。短期内URDF不会被替代，但长期来看，可能会出现一种"URDF-next"格式来弥补当前的不足。

---

## P01.7 Xacro 高级技巧与设计模式 ⭐⭐

### 条件分支实战

Xacro的`<xacro:if>`和`<xacro:unless>`标签允许根据参数值条件性地包含XML片段。这在**仿真/真机切换**场景中极为实用：

```xml
<!-- robot.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- 声明参数，可通过launch文件传入 -->
  <xacro:arg name="use_sim" default="true"/>
  <xacro:arg name="use_mock" default="false"/>

  <!-- 将arg转为property，后续在表达式中使用 -->
  <xacro:property name="use_sim" value="$(arg use_sim)"/>
  <xacro:property name="use_mock" value="$(arg use_mock)"/>

  <!-- 仿真模式：Gazebo system plugin + ros2_control hardware plugin 必须同时生成 -->
  <xacro:if value="${use_sim}">
    <gazebo>
      <plugin filename="gz_ros2_control-system"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin"/>
    </gazebo>
    <ros2_control name="GazeboSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>
      <!-- joints... -->
    </ros2_control>
  </xacro:if>

  <!-- Mock模式：用于无硬件的集成测试 -->
  <xacro:if value="${use_mock}">
    <ros2_control name="MockSystem" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
      </hardware>
      <!-- joints... -->
    </ros2_control>
  </xacro:if>

  <!-- 真机模式：两个都为false时 -->
  <xacro:unless value="${use_sim or use_mock}">
    <ros2_control name="RealHardware" type="system">
      <hardware>
        <plugin>my_robot_driver/MyRobotSystemInterface</plugin>
        <param name="serial_port">/dev/ttyUSB0</param>
      </hardware>
      <!-- joints... -->
    </ros2_control>
  </xacro:unless>

</robot>
```

在launch文件中通过参数控制：

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')

    robot_description = Command([
        'xacro ',
        '/path/to/robot.urdf.xacro',
        ' use_sim:=', LaunchConfiguration('use_sim'),
    ])

    return LaunchDescription([use_sim_arg, ...])
```

### 数学表达式

Xacro支持Python风格的数学表达式，在`${...}`中使用。预定义常量`pi`可直接引用：

```xml
<xacro:property name="arm_length" value="0.4"/>
<xacro:property name="offset" value="0.02"/>

<!-- 数学运算 -->
<origin xyz="0 0 ${arm_length * 0.5 + offset}" rpy="0 ${pi/4} 0"/>

<!-- 三角函数 -->
<origin xyz="${arm_length * cos(pi/6)} ${arm_length * sin(pi/6)} 0"/>

<!-- 条件表达式（Python三元运算） -->
<xacro:property name="radius" value="${0.05 if arm_length > 0.3 else 0.03}"/>
```

注意：表达式中可以使用Python的math模块函数（`sin`, `cos`, `sqrt`, `atan2`等），但不能import其他模块。

### 跨文件include模式

大型机器人项目的Xacro通常拆分为多个文件，通过`<xacro:include>`组合：

```
my_robot_description/
  urdf/
    robot.urdf.xacro          # 主文件，include所有子模块
    base/
      base.urdf.xacro         # 底盘
    arm/
      arm.urdf.xacro          # 机械臂（macro定义）
      arm_params.yaml          # 臂参数（DH参数等）
    gripper/
      gripper.urdf.xacro      # 末端执行器
    sensors/
      camera.urdf.xacro       # 相机
      lidar.urdf.xacro        # 激光雷达
      imu.urdf.xacro          # IMU
    ros2_control/
      hardware.urdf.xacro     # ros2_control标签
```

主文件的典型结构：

```xml
<!-- robot.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <xacro:include filename="$(find my_robot_description)/urdf/base/base.urdf.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/arm/arm.urdf.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/gripper/gripper.urdf.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/sensors/camera.urdf.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/ros2_control/hardware.urdf.xacro"/>

  <!-- 实例化 -->
  <xacro:arm prefix="left_" parent="base_link" reflect="1"/>
  <xacro:arm prefix="right_" parent="base_link" reflect="-1"/>
  <xacro:gripper prefix="left_" parent="left_link_7"/>
  <xacro:gripper prefix="right_" parent="right_link_7"/>

</robot>
```

### 参数化关节限位

关节限位是安全运行的关键参数，应当通过macro参数显式传递，而非硬编码：

```xml
<xacro:macro name="revolute_joint"
             params="name parent child
                     axis_xyz
                     limit_lower limit_upper
                     velocity_limit effort_limit
                     origin_xyz origin_rpy">
  <joint name="${name}" type="revolute">
    <parent link="${parent}"/>
    <child link="${child}"/>
    <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
    <axis xyz="${axis_xyz}"/>
    <limit lower="${limit_lower}" upper="${limit_upper}"
           velocity="${velocity_limit}" effort="${effort_limit}"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>
</xacro:macro>
```

### 完整示例：参数化7-DOF机械臂

以下是一个完整的参数化7-DOF机械臂Xacro，展示了"基座+N关节"的设计模式——用一个macro生成单个关节-连杆单元，再在顶层串联成整条臂：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm_7dof">

  <!-- ======================== 全局参数 ======================== -->
  <!-- xacro 已内置 pi 常量（${pi}），无需重定义 -->
  <xacro:property name="default_damping" value="0.1"/>
  <xacro:property name="default_friction" value="0.05"/>

  <!-- ======================== 惯性计算宏 ======================== -->
  <xacro:macro name="cylinder_inertia" params="mass radius length com_z">
    <inertial>
      <origin xyz="0 0 ${com_z}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${mass * (3*radius*radius + length*length) / 12.0}"
        iyy="${mass * (3*radius*radius + length*length) / 12.0}"
        izz="${mass * radius * radius / 2.0}"
        ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </xacro:macro>

  <!--
    坐标系约定：
    - link_i 坐标系放在 joint_i 轴线上，+Z 指向下一段的名义方向；
    - joint_z 是 parent_link 坐标系到 joint_i/link_i 坐标系的固定偏移；
    - body_length 是 link_i 自身的简化圆柱几何长度，必须 > 0。
    因此可以表达"两个关节轴共点"（joint_z=0），但不会生成退化圆柱。
  -->
  <!-- ======================== 单关节-连杆单元 ======================== -->
  <xacro:macro name="arm_segment"
               params="index parent_link
                       joint_z body_length radius mass
                       axis_xyz
                       limit_lower limit_upper
                       velocity_limit effort_limit
                       prefix">
    <!-- link -->
    <link name="${prefix}link_${index}">
      <visual>
        <geometry>
          <cylinder radius="${radius}" length="${body_length}"/>
        </geometry>
        <origin xyz="0 0 ${body_length/2.0}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${radius}" length="${body_length}"/>
        </geometry>
        <origin xyz="0 0 ${body_length/2.0}" rpy="0 0 0"/>
      </collision>
      <xacro:cylinder_inertia mass="${mass}"
                               radius="${radius}"
                               length="${body_length}"
                               com_z="${body_length/2.0}"/>
    </link>

    <!-- joint -->
    <joint name="${prefix}joint_${index}" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${prefix}link_${index}"/>
      <origin xyz="0 0 ${joint_z}" rpy="0 0 0"/>
      <axis xyz="${axis_xyz}"/>
      <limit lower="${limit_lower}" upper="${limit_upper}"
             velocity="${velocity_limit}" effort="${effort_limit}"/>
      <dynamics damping="${default_damping}"
                friction="${default_friction}"/>
    </joint>

    <!-- ⚠️ 以下 `<transmission>` 标签为 ROS1 遗留语法，在 ROS2 中已被 `<ros2_control>` 标签替代（见 P01.5 节）。保留仅为兼容性参考。 -->
    <transmission name="${prefix}trans_${index}">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}joint_${index}">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}motor_${index}">
        <mechanicalReduction>100</mechanicalReduction>
      </actuator>
    </transmission>
  </xacro:macro>

  <!-- ======================== 基座 ======================== -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.08" length="0.05"/></geometry>
    </visual>
    <collision>
      <geometry><cylinder radius="0.08" length="0.05"/></geometry>
    </collision>
    <xacro:cylinder_inertia mass="2.0" radius="0.08" length="0.05" com_z="0.0"/>
  </link>

  <!-- ======================== 7个关节实例化 ======================== -->
  <!-- J1: 基座旋转 (yaw) -->
  <xacro:arm_segment index="1" parent_link="base_link"
    joint_z="0.15" body_length="0.15" radius="0.06" mass="3.0"
    axis_xyz="0 0 1"
    limit_lower="${-pi*170/180}" limit_upper="${pi*170/180}"
    velocity_limit="2.0" effort_limit="87.0" prefix=""/>

  <!-- J2: 肩部俯仰 (pitch)，joint_z=0 表示与 J1 共点，但 body_length 仍为非零几何 -->
  <xacro:arm_segment index="2" parent_link="link_1"
    joint_z="0.00" body_length="0.08" radius="0.05" mass="2.5"
    axis_xyz="0 1 0"
    limit_lower="${-pi*120/180}" limit_upper="${pi*120/180}"
    velocity_limit="2.0" effort_limit="87.0" prefix=""/>

  <!-- J3: 肩部旋转 (yaw) -->
  <xacro:arm_segment index="3" parent_link="link_2"
    joint_z="0.35" body_length="0.35" radius="0.05" mass="2.0"
    axis_xyz="0 0 1"
    limit_lower="${-pi*170/180}" limit_upper="${pi*170/180}"
    velocity_limit="2.0" effort_limit="87.0" prefix=""/>

  <!-- J4: 肘部俯仰 (pitch)，共点关节用 joint_z=0 表达，不用零长度圆柱表达 -->
  <xacro:arm_segment index="4" parent_link="link_3"
    joint_z="0.00" body_length="0.07" radius="0.04" mass="1.8"
    axis_xyz="0 1 0"
    limit_lower="${-pi*120/180}" limit_upper="${pi*120/180}"
    velocity_limit="2.0" effort_limit="50.0" prefix=""/>

  <!-- J5: 前臂旋转 (yaw) -->
  <xacro:arm_segment index="5" parent_link="link_4"
    joint_z="0.35" body_length="0.35" radius="0.04" mass="1.5"
    axis_xyz="0 0 1"
    limit_lower="${-pi*170/180}" limit_upper="${pi*170/180}"
    velocity_limit="3.0" effort_limit="50.0" prefix=""/>

  <!-- J6: 腕部俯仰 (pitch)，共点关节仍保留非零简化碰撞几何 -->
  <xacro:arm_segment index="6" parent_link="link_5"
    joint_z="0.00" body_length="0.06" radius="0.03" mass="1.0"
    axis_xyz="0 1 0"
    limit_lower="${-pi*120/180}" limit_upper="${pi*120/180}"
    velocity_limit="3.0" effort_limit="20.0" prefix=""/>

  <!-- J7: 腕部旋转 (roll) -->
  <xacro:arm_segment index="7" parent_link="link_6"
    joint_z="0.10" body_length="0.10" radius="0.03" mass="0.5"
    axis_xyz="1 0 0"
    limit_lower="${-pi*175/180}" limit_upper="${pi*175/180}"
    velocity_limit="3.0" effort_limit="20.0" prefix=""/>

</robot>
```

> **⚠️ Pitfall：Xacro变量作用域**
>
> `<xacro:property>`在macro内部定义时是**局部变量**，仅在该macro调用范围内可见。在macro外部（文件顶层）定义的property是**全局变量**。一个常见错误是在macro A内部定义了一个property，然后试图在macro B中读取它——这会得到一个空值或解析错误，且错误信息往往不直观（显示为"undefined"而非指出作用域问题）。
>
> 正确做法：跨macro共享的参数应当定义在文件顶层，或通过macro参数显式传递。

---

## P01.8 惯性参数详解与系统辨识 ⭐⭐⭐

### 惯性张量的物理含义

惯性张量（Inertia Tensor）是一个3x3对称矩阵，描述刚体对旋转运动的"抵抗程度"。它在机器人动力学中的地位等同于平动中的质量——质量越大，加速越难；惯性矩越大，角加速越难。

$$
\mathbf{I} = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix}
$$

各分量的物理含义：
- **$I_{xx}$**：绕x轴旋转的惯性矩。数值越大，使刚体绕x轴转动需要的力矩越大。
- **$I_{yy}$, $I_{zz}$**：类似，分别对应y轴和z轴。
- **$I_{xy}$, $I_{xz}$, $I_{yz}$**：惯性积（products of inertia）。当刚体的质量分布关于某坐标平面不对称时，惯性积非零。对于URDF中常见的规则几何体（圆柱、长方体），如果坐标原点在质心且坐标轴与几何轴对齐，惯性积为零。

以均匀实心圆柱体（质量$m$、半径$r$、高度$h$，z轴为圆柱轴线）为例，精确解析解为：

$$
I_{xx} = I_{yy} = \frac{m}{12}(3r^2 + h^2), \quad I_{zz} = \frac{mr^2}{2}, \quad I_{xy} = I_{xz} = I_{yz} = 0
$$

$I_{zz} < I_{xx}$这一关系是符合直觉的：绕自身轴线旋转（z轴）比绕直径方向旋转（x/y轴）更容易，因为质量更集中于旋转轴附近。

### 主轴与惯性椭球

对惯性张量$\mathbf{I}$做特征值分解：

$$
\mathbf{I} = \mathbf{R} \begin{bmatrix} I_1 & 0 & 0 \\ 0 & I_2 & 0 \\ 0 & 0 & I_3 \end{bmatrix} \mathbf{R}^T
$$

其中$I_1, I_2, I_3$是**主惯量**（principal moments），$\mathbf{R}$的列向量是**主轴方向**。主惯量的几何意义是：在主轴坐标系下，惯性积全部为零，惯性张量变成对角矩阵。

惯性椭球以主轴为坐标轴，半轴长度与主惯量的平方根成反比。椭球越"扁"的方向，对应惯性矩越大（越难转动）。在URDF中，如果link的坐标系恰好与主轴对齐，则只需填写$I_{xx}, I_{yy}, I_{zz}$三个对角元素，交叉项为零。

### 平行轴定理

当惯性张量的参考点不在质心时，必须使用平行轴定理（Parallel Axis Theorem）进行转换：

$$
\mathbf{I}_O = \mathbf{I}_{cm} + m \begin{bmatrix} d_y^2 + d_z^2 & -d_x d_y & -d_x d_z \\ -d_x d_y & d_x^2 + d_z^2 & -d_y d_z \\ -d_x d_z & -d_y d_z & d_x^2 + d_y^2 \end{bmatrix}
$$

其中$(d_x, d_y, d_z)$是质心到新参考点$O$的位移向量。

在URDF中，`<inertial>`标签的`<origin>`指定的是质心相对于link坐标系原点的位移。如果`<origin>`非零（即质心不在link原点），URDF解析器会自动处理。但如果你在外部计算惯性（如从CAD导出），需要确认导出的惯性是相对于**质心**还是相对于**某个参考点**。如果是后者，必须先用平行轴定理反向转换回质心，再填入URDF。

### 负惯性检测——三角不等式

一个物理上合法的惯性张量必须是**正定的**，且主惯量之间满足三角不等式：

$$
I_1 + I_2 \geq I_3, \quad I_1 + I_3 \geq I_2, \quad I_2 + I_3 \geq I_1
$$

这三个不等式的物理根源是：任何刚体的质量分布，绕任意两个正交轴的惯性矩之和，必定不小于绕第三个正交轴的惯性矩。

违反这些不等式的惯性参数，意味着该"刚体"不对应任何真实的质量分布。常见的违反场景是：
- 手动调参时为了"让仿真稳定"随意修改了某个分量
- CAD导出时单位错误（如$I_{zz}$少了几个数量级）
- 简化模型时将某个惯性分量设为零

后果是仿真器在积分过程中产生非物理的角加速度，最终导致**仿真爆炸**（数值发散，link飞到无穷远处）。

检测代码：

```python
def check_inertia_validity(ixx, iyy, izz, ixy, ixz, iyz):
    """检测惯性张量的物理合法性"""
    import numpy as np
    I = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz]
    ])
    eigenvalues = np.linalg.eigvalsh(I)
    I1, I2, I3 = sorted(eigenvalues)

    # 检查正定性
    if I1 <= 0:
        print(f"[ERROR] 惯性张量非正定: 最小特征值 = {I1:.6e}")
        return False

    # 检查三角不等式
    if I1 + I2 < I3 * (1 - 1e-10):  # 带数值容差
        print(f"[ERROR] 三角不等式违反: {I1:.6e} + {I2:.6e} < {I3:.6e}")
        return False

    print(f"[OK] 主惯量: {I1:.6e}, {I2:.6e}, {I3:.6e}")
    return True

# 示例：圆柱体 (m=2kg, r=0.05m, h=0.3m)
m, r, h = 2.0, 0.05, 0.30
ixx = m * (3*r**2 + h**2) / 12  # 0.01625
izz = m * r**2 / 2               # 0.0025
check_inertia_validity(ixx, ixx, izz, 0, 0, 0)
# [OK] 主惯量: 2.500000e-03, 1.625000e-02, 1.625000e-02
```

### 系统辨识（System Identification）概述

在实际机器人上，CAD模型的惯性参数与真实值往往有较大偏差（线缆、紧固件、润滑脂的质量未计入）。系统辨识（SysId）通过实验数据来估计真实的惯性参数。

经典的Swevers方法流程如下：

1. **参数化动力学模型**：将机器人逆动力学方程写成关于"基参数"（base parameters）的线性形式 $\boldsymbol{\tau} = \mathbf{Y}(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}) \boldsymbol{\theta}$
2. **设计激励轨迹**：找到使观测矩阵$\mathbf{Y}$条件数最小的关节轨迹（通常用傅里叶级数参数化，优化傅里叶系数）
3. **执行实验**：让机器人跟踪激励轨迹，记录关节角度、速度和力矩数据
4. **最小二乘求解**：$\hat{\boldsymbol{\theta}} = (\mathbf{Y}^T \mathbf{Y})^{-1} \mathbf{Y}^T \boldsymbol{\tau}$

辨识出的参数可以回填到URDF中，显著改善基于模型的控制器（如计算力矩控制）的性能。

### 从mesh顶点近似计算惯性

当没有CAD模型只有mesh文件时，可以用mesh顶点近似计算惯性：

```python
import numpy as np
import trimesh

def inertia_from_mesh(mesh_path, density=1000.0):
    """
    从mesh文件近似计算惯性参数
    假设均匀密度，使用trimesh的体积积分

    Args:
        mesh_path: STL/OBJ/DAE文件路径
        density: 材料密度 (kg/m^3), 铝约2700, 钢约7800, ABS约1050
    Returns:
        mass, center_of_mass, inertia_tensor
    """
    mesh = trimesh.load(mesh_path)

    if not mesh.is_watertight:
        print("[WARN] Mesh不是封闭的，体积计算可能不准确")
        print("       建议用MeshLab修复后再计算")

    # trimesh假设密度=1时的质量等于体积
    volume = mesh.volume  # m^3 (确保mesh单位是米)
    mass = density * volume

    # 质心
    com = mesh.center_mass  # (3,) array

    # 惯性张量（相对于质心，density=1时）
    I_unit = mesh.moment_inertia  # (3,3) array
    I = I_unit * density  # 缩放到实际密度

    print(f"体积: {volume:.6e} m^3")
    print(f"质量: {mass:.4f} kg")
    print(f"质心: [{com[0]:.4f}, {com[1]:.4f}, {com[2]:.4f}] m")
    print(f"惯性张量 (kg*m^2):")
    print(f"  Ixx={I[0,0]:.6e}  Ixy={I[0,1]:.6e}  Ixz={I[0,2]:.6e}")
    print(f"  Iyy={I[1,1]:.6e}  Iyz={I[1,2]:.6e}")
    print(f"  Izz={I[2,2]:.6e}")

    return mass, com, I
```

> **⚠️ Pitfall：CAD导出的惯性单位陷阱**
>
> 许多CAD软件（SolidWorks、Fusion360）默认使用$\text{g} \cdot \text{mm}^2$作为惯性单位，而URDF要求$\text{kg} \cdot \text{m}^2$。两者之间差**6个数量级**（$1 \text{ g} \cdot \text{mm}^2 = 10^{-9} \text{ kg} \cdot \text{m}^2$）。
>
> 如果不转换就直接填入URDF，link的惯性会被低估一百万倍。Gazebo仿真中的表现是：该link对任何外力都几乎没有惯性响应，像"纸片"一样被轻易推飞，或者关节处出现极大的角加速度振荡。
>
> 验证方法：对于一个典型的机械臂link（铝合金，长0.3m，截面直径0.06m），合理的$I_{xx}$量级应该在$10^{-3}$到$10^{-2}$ $\text{kg} \cdot \text{m}^2$范围内。如果你看到$10^{-9}$或$10^3$量级的数值，几乎可以确定是单位错误。

---

## P01.9 ros2_control 标签详解 ⭐⭐

### `<ros2_control>`标签完整解剖

`<ros2_control>`标签是URDF与ros2_control框架之间的桥梁。它定义了硬件抽象层需要的全部信息：用什么硬件插件、有哪些关节、每个关节提供什么命令和状态接口。

标签的完整结构：

```xml
<ros2_control name="system_name" type="system">

  <!-- 1. 硬件插件声明 -->
  <hardware>
    <plugin>package_name/PluginClassName</plugin>
    <param name="param1">value1</param>
    <param name="param2">value2</param>
  </hardware>

  <!-- 2. 关节接口声明（每个受控关节一个） -->
  <joint name="joint_1">
    <!-- 命令接口：控制器向硬件发送的指令类型 -->
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <command_interface name="velocity">
      <param name="min">-2.0</param>
      <param name="max">2.0</param>
    </command_interface>

    <!-- 状态接口：硬件向控制器反馈的数据类型 -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <!-- 3. 传感器接口声明（可选） -->
  <sensor name="tcp_force_torque">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
    <param name="frame_id">tool0</param>
  </sensor>

  <sensor name="base_imu">
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
    <state_interface name="linear_acceleration.x"/>
    <state_interface name="linear_acceleration.y"/>
    <state_interface name="linear_acceleration.z"/>
  </sensor>

</ros2_control>
```

各部分的角色：
- **`<hardware>`**：声明具体的硬件驱动插件。ros2_control的controller_manager在启动时加载这个插件，调用其`on_configure()`和`on_activate()`生命周期方法。
- **`<joint>`**：每个`<command_interface>`声明一种控制模式，每个`<state_interface>`声明一种反馈信号。关节的name必须与URDF中`<joint name="...">`完全匹配。
- **`<sensor>`**：声明非关节型传感器的接口。力/力矩传感器和IMU是最常见的两种。

### 三种部署模式切换

ros2_control的一个核心优势是**硬件抽象**——同一套控制器代码，通过切换`<hardware>`插件就能在三种环境中运行：

**模式一：Mock Hardware（无硬件测试）**

```xml
<hardware>
  <plugin>mock_components/GenericSystem</plugin>
  <param name="mock_sensor_commands">true</param>
  <param name="calculate_dynamics">true</param>
</hardware>
```

Mock硬件会直接将command值镜像为state值（发送position=1.0，状态立刻反馈position=1.0）。用于控制器逻辑的单元测试和CI流水线。不需要Gazebo或真实硬件。

**模式二：Gazebo仿真**

```xml
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
```

Gazebo仿真中的物理引擎提供关节的动力学响应。Command通过Gazebo的关节API施加，State从仿真状态中读取。延迟、惯性、摩擦等都由物理引擎模拟。

**模式三：真实硬件**

```xml
<hardware>
  <plugin>my_robot_driver/MyRobotHardwareInterface</plugin>
  <param name="serial_port">/dev/ttyUSB0</param>
  <param name="baudrate">1000000</param>
</hardware>
```

真实硬件插件负责与物理设备通信——通过串口、EtherCAT、CAN总线等协议读写关节命令和状态。

### 完整示例：7-DOF臂的ros2_control配置

```xml
<ros2_control name="arm_7dof_system" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>

  <joint name="joint_1">
    <command_interface name="effort">
      <param name="min">-87.0</param>
      <param name="max">87.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint_2">
    <command_interface name="effort">
      <param name="min">-87.0</param>
      <param name="max">87.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint_3">
    <command_interface name="effort">
      <param name="min">-87.0</param>
      <param name="max">87.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint_4">
    <command_interface name="effort">
      <param name="min">-50.0</param>
      <param name="max">50.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint_5">
    <command_interface name="effort">
      <param name="min">-50.0</param>
      <param name="max">50.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint_6">
    <command_interface name="effort">
      <param name="min">-20.0</param>
      <param name="max">20.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint_7">
    <command_interface name="effort">
      <param name="min">-20.0</param>
      <param name="max">20.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <sensor name="tcp_fts">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
    <param name="frame_id">tool0</param>
  </sensor>
</ros2_control>
```

### 与M12（ros2_control）章节的衔接

P01中`<ros2_control>`标签的作用是**声明接口契约**：这个机器人有哪些关节、每个关节支持什么类型的命令和反馈。但标签本身并不实现任何逻辑。

真正的实现在M12章节中：
- `SystemInterface::read()`：从硬件读取state_interface的值
- `SystemInterface::write()`：向硬件写入command_interface的值
- `on_configure()`中解析`<param>`标签的参数
- 生命周期管理：inactive -> active -> inactive

P01和M12的关系可以类比为：P01是"接口定义"（header file），M12是"接口实现"（source file）。两者必须严格一致——如果P01声明了`effort` command_interface，但M12的`write()`方法只处理了`position`，那么effort命令会被静默忽略。

> **⚠️ Pitfall：command_interface类型与控制器不匹配**
>
> ros2_control中最常见也最难调试的错误之一：`<command_interface>`声明的类型与控制器期望的类型不匹配。
>
> 例如，你在URDF中声明了`<command_interface name="position"/>`，但在controller YAML中配置了`effort_controllers/JointGroupEffortController`。这个控制器需要写入effort接口，但硬件只暴露了position接口。
>
> 后果：controller_manager会成功加载控制器，控制器也能切换到active状态，但实际的力矩命令**不会被执行**——因为硬件根本没有effort的command_interface可供写入。机器人看起来像是"没有响应"，但不会有任何错误日志（只有在debug级别才能看到warning）。
>
> 诊断方法：使用`ros2 control list_hardware_interfaces`命令检查实际暴露的接口列表，与控制器的要求逐一比对。

---

## P01.10 补充练习

### 练习4：手动计算圆柱体惯性张量并验证三角不等式

**题目**：一个均匀实心圆柱体link，质量$m = 2 \text{ kg}$，半径$r = 0.05 \text{ m}$，高度$h = 0.3 \text{ m}$，圆柱轴线沿z轴方向。

**任务**：
1. 手动计算$I_{xx}$、$I_{yy}$、$I_{zz}$
2. 验证三角不等式是否成立
3. 将计算结果填入URDF的`<inertial>`标签
4. 用Python脚本验证你的手算结果

**解答过程**：

第一步：代入公式。圆柱体（轴线沿z）的惯性矩解析解：

$$
I_{xx} = I_{yy} = \frac{m}{12}(3r^2 + h^2) = \frac{2}{12}(3 \times 0.05^2 + 0.3^2) = \frac{2}{12}(0.0075 + 0.09) = \frac{2 \times 0.0975}{12} = 0.01625 \text{ kg} \cdot \text{m}^2
$$

$$
I_{zz} = \frac{mr^2}{2} = \frac{2 \times 0.05^2}{2} = \frac{2 \times 0.0025}{2} = 0.0025 \text{ kg} \cdot \text{m}^2
$$

第二步：验证三角不等式。

$$
I_{xx} + I_{yy} = 0.01625 + 0.01625 = 0.0325 \geq I_{zz} = 0.0025 \quad \checkmark
$$

$$
I_{xx} + I_{zz} = 0.01625 + 0.0025 = 0.01875 \geq I_{yy} = 0.01625 \quad \checkmark
$$

$$
I_{yy} + I_{zz} = 0.01625 + 0.0025 = 0.01875 \geq I_{xx} = 0.01625 \quad \checkmark
$$

三个不等式全部成立，惯性张量物理合法。

第三步：填入URDF。

```xml
<link name="cylinder_link">
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- 质心在半高处 -->
    <inertia ixx="0.01625" iyy="0.01625" izz="0.0025"
             ixy="0" ixz="0" iyz="0"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </visual>
</link>
```

第四步：Python验证脚本。

```python
import numpy as np

m, r, h = 2.0, 0.05, 0.3
Ixx = m * (3*r**2 + h**2) / 12
Iyy = Ixx  # 圆柱对称
Izz = m * r**2 / 2

print(f"Ixx = Iyy = {Ixx:.5f} kg*m^2")
print(f"Izz = {Izz:.5f} kg*m^2")

# 三角不等式
assert Ixx + Iyy >= Izz, "违反: Ixx+Iyy < Izz"
assert Ixx + Izz >= Iyy, "违反: Ixx+Izz < Iyy"
assert Iyy + Izz >= Ixx, "违反: Iyy+Izz < Ixx"
print("三角不等式: 全部通过")
```

---

### 练习5：Xacro条件分支——三种模式切换

**题目**：编写一个完整的Xacro文件和对应的launch文件，使同一个URDF可以在三种模式下运行：
1. `mode:=sim` -- Gazebo仿真
2. `mode:=mock` -- Mock hardware（CI测试）
3. `mode:=real` -- 真实硬件

**要求**：
- 使用`<xacro:if>`条件分支
- 三种模式的`<hardware>`插件不同
- 真机模式需要额外的`<param>`（串口地址）

**参考实现**：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="switchable_robot">

  <xacro:arg name="mode" default="mock"/>
  <xacro:property name="mode" value="$(arg mode)"/>

  <!-- 共享的link/joint定义 -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.08" length="0.05"/></geometry>
    </visual>
  </link>
  <link name="link_1">
    <visual>
      <geometry><cylinder radius="0.04" length="0.3"/></geometry>
      <origin xyz="0 0 0.15"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.012" iyy="0.012" izz="0.0012"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="50.0"/>
  </joint>

  <!-- Gazebo仿真模式 -->
  <xacro:if value="${mode == 'sim'}">
    <gazebo>
      <plugin filename="gz_ros2_control-system"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin"/>
    </gazebo>
  </xacro:if>

  <!-- ============ ros2_control: 模式切换 ============ -->
  <ros2_control name="robot_system" type="system">

    <!-- Mock模式 -->
    <xacro:if value="${mode == 'mock'}">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
        <param name="calculate_dynamics">true</param>
      </hardware>
    </xacro:if>

    <!-- Gazebo仿真模式 -->
    <xacro:if value="${mode == 'sim'}">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>
    </xacro:if>

    <!-- 真机模式 -->
    <xacro:if value="${mode == 'real'}">
      <hardware>
        <plugin>my_robot_driver/HardwareInterface</plugin>
        <param name="serial_port">/dev/ttyUSB0</param>
        <param name="baudrate">1000000</param>
        <param name="timeout_ms">10</param>
      </hardware>
    </xacro:if>

    <!-- 关节接口（三种模式共享） -->
    <joint name="joint_1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

  </ros2_control>
</robot>
```

对应的launch文件：

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mock',
        choices=['sim', 'mock', 'real'],
        description='Hardware mode: sim/mock/real'
    )

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf', 'robot.urdf.xacro'
        ]),
        ' mode:=', LaunchConfiguration('mode'),
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([mode_arg, rsp_node])
```

启动命令：
```bash
# Mock模式（默认）
ros2 launch my_robot_description robot.launch.py

# Gazebo仿真模式
ros2 launch my_robot_description robot.launch.py mode:=sim

# 真机模式
ros2 launch my_robot_description robot.launch.py mode:=real
```

---

### 练习6：从Franka Panda URDF提取关节限位并可视化工作空间

**题目**：下载Franka Panda的官方URDF，用Python脚本提取所有revolute关节的`<limit>`属性，生成两张图表：
1. 各关节的位置/速度/力矩限位的柱状对比图
2. 用蒙特卡洛采样法近似绘制末端执行器的可达工作空间边界（xy平面投影）

**提示代码**：

```python
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt

def extract_joint_limits(urdf_path):
    """从URDF提取所有revolute关节的限位"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = []
    for joint in root.findall('joint'):
        if joint.get('type') in ['revolute', 'prismatic']:
            limit = joint.find('limit')
            if limit is not None:
                joints.append({
                    'name': joint.get('name'),
                    'type': joint.get('type'),
                    'lower': float(limit.get('lower', 0)),
                    'upper': float(limit.get('upper', 0)),
                    'velocity': float(limit.get('velocity', 0)),
                    'effort': float(limit.get('effort', 0)),
                })
    return joints

def plot_joint_limits(joints):
    """绘制关节限位柱状图"""
    names = [j['name'] for j in joints]
    n = len(names)
    x = np.arange(n)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 位置范围
    ranges = [j['upper'] - j['lower'] for j in joints]
    axes[0].bar(x, ranges, color='steelblue')
    axes[0].set_ylabel('Position Range (rad)')
    axes[0].set_title('Joint Limits from URDF')

    # 速度限位
    velocities = [j['velocity'] for j in joints]
    axes[1].bar(x, velocities, color='coral')
    axes[1].set_ylabel('Max Velocity (rad/s)')

    # 力矩限位
    efforts = [j['effort'] for j in joints]
    axes[2].bar(x, efforts, color='mediumseagreen')
    axes[2].set_ylabel('Max Effort (Nm)')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(names, rotation=45, ha='right')

    plt.tight_layout()
    plt.savefig('joint_limits.png', dpi=150)
    plt.show()

# 使用示例
# joints = extract_joint_limits('panda.urdf')
# plot_joint_limits(joints)
```

工作空间采样的核心逻辑（需要配合正向运动学）：

```python
def sample_workspace(n_samples=50000):
    """蒙特卡洛采样末端执行器可达位置"""
    # 需要 roboticstoolbox 或自行实现FK
    # pip install roboticstoolbox-python
    import roboticstoolbox as rtb

    robot = rtb.models.Panda()
    points = []

    for _ in range(n_samples):
        q = robot.random_q()  # 在关节限位内均匀采样
        T = robot.fkine(q)    # 正向运动学
        points.append(T.t[:2])  # 取xy平面投影

    points = np.array(points)
    plt.figure(figsize=(8, 8))
    plt.scatter(points[:, 0], points[:, 1], s=0.5, alpha=0.3)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Panda Workspace (XY Projection, Monte Carlo)')
    plt.axis('equal')
    plt.grid(True)
    plt.savefig('workspace.png', dpi=150)
    plt.show()
```

---

### 练习7：URDF vs MJCF -- 找出5个差异

**题目**：选择一个你熟悉的机器人（如Franka Panda或UR5），分别用URDF和MJCF描述它，找出至少5个MJCF能表达但URDF不能的特性。

**参考对比列表**（供验证你的发现）：

| 特性 | URDF中的表达 | MJCF中的表达 |
|------|-------------|-------------|
| **1. 关节摩擦模型** | 仅`<dynamics friction="..."/>`（库仑摩擦系数） | `<joint frictionloss="..." damping="..." armature="..."/>`（分离库仑摩擦、粘性阻尼、电机惯量） |
| **2. 接触参数** | 不支持 | `<geom friction="slide spin roll" solimp="..." solref="..."/>`（滑动/自旋/滚动摩擦 + 接触刚度阻尼） |
| **3. 执行器模型** | `<transmission>`（已半废弃，仅减速比） | `<actuator><motor joint="j1" gear="100" ctrlrange="-1 1"/></actuator>`（电机模型、力矩范围、控制增益） |
| **4. 肌腱/耦合** | 不支持 | `<tendon><spatial><site site="s1"/><geom geom="g1"/><site site="s2"/></spatial></tendon>` |
| **5. 默认值继承** | 每个元素必须完整定义 | `<default class="arm"><geom rgba="0.8 0.2 0.2 1"/><joint damping="0.5"/></default>`（层级默认值系统） |
| 6. 闭链机构 | 不支持 | `<equality><connect body1="b1" body2="b2" anchor="0 0 0"/></equality>` |
| 7. 场景元素 | 不支持 | `<worldbody><light.../><geom type="plane".../></worldbody>` |

**动手建议**：
1. 先用`mujoco`包的`mujoco.MjModel.from_xml_path()`加载一个现有的MJCF模型
2. 尝试将其转换为URDF，记录每一处信息丢失
3. 反思哪些丢失的信息对你的仿真任务影响最大
