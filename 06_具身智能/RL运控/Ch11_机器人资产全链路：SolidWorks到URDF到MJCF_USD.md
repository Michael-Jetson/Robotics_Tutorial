# Ch11 | 机器人资产全链路：SolidWorks → URDF → MJCF/USD

> **本章定位**：Ch01-Ch10 讨论了如何训练策略——从 PPO 到 reward 设计到 motion imitation。但所有训练都有一个前置假设：你已经有了一个可用的机器人模型。本章解决这个前置问题：如何从 CAD 设计图纸出发，经过一系列格式转换和参数调优，得到一个可以在 mjlab 和 Isaac Lab 中训练的机器人资产。
>
> **前置依赖**：Ch03（MuJoCo Warp 基础，MjSpec/MjModel/MjData 生命周期）
>
> **参考项目**：✅ `ros/solidworks_urdf_exporter` · MuJoCo Menagerie · Isaac Lab Asset Zoo · ✅ `aCodeDog/awesome-loco-manipulation`

---

## 前置自测

📋 **答不出 $\ge$ 3 题 → 先回前置章节复习**

| # | 问题 | 检查目的 |
|---|------|----------|
| 1 | MjSpec、MjModel、MjData 的生命周期分别是什么？在哪个阶段可以修改模型结构？ | Ch03 核心概念 |
| 2 | URDF 的 `<link>` 和 `<joint>` 元素分别描述什么？一个 6-DOF 机械臂有几个 link 和几个 joint？ | URDF 基础 |
| 3 | 为什么仿真中的碰撞检测通常不使用三角形 mesh 而是凸包？ | 物理仿真基础 |
| 4 | 刚体的惯性张量 $I$ 是一个 $3\times3$ 矩阵。它必须满足什么物理约束才是"合法"的？ | 刚体力学基础 |
| 5 | MuJoCo 的 `<actuator>` 元素中，`<position>` 和 `<motor>` 类型有什么区别？ | Ch03 |
| 6 | Isaac Lab 加载机器人资产时需要 USD 格式。USD 是什么？它和 URDF 有什么本质区别？ | Isaac Lab 基础 |

## 本章目标

学完本章后，你应该能够：

1. **导出** 从 SolidWorks 装配体到 URDF 的完整流程，理解 sw2urdf 的装配体要求和常见错误
2. **转换** URDF 到 MJCF（MuJoCo 格式），掌握 MuJoCo Menagerie 的标准转换流程和手动调优步骤
3. **转换** URDF 到 USD（Isaac Lab 格式），使用 Isaac Lab 的 `convert_urdf.py` 工具
4. **简化** collision mesh，使用 V-HACD 和 CoACD 工具将复杂几何体分解为凸包集合
5. **验证** 惯性参数的物理一致性，用自由落体和静止平衡测试检测不合理参数
6. **使用** MuJoCo Menagerie 和 Isaac Lab Asset Zoo 中的现成模型，理解复合机器人 URDF 的组织方式
7. **调试** 模型在双框架（mjlab/Isaac Lab）中行为不一致的根本原因

---

## 11.1 从 CAD 到仿真：全链路概览 ⭐

> **这一节解决什么问题**：建立从 SolidWorks CAD 文件到可训练的仿真模型的完整心智模型——每一步做什么、会丢失什么信息、会引入什么错误。

### 动机：为什么不能直接把 CAD 文件拖进仿真器

SolidWorks 等 CAD 软件描述的是精确几何——每个零件的形状、材料、装配关系都有毫米级精度。但物理仿真器（MuJoCo、PhysX）需要的不是精确几何，而是**物理模型**——质量、惯量、关节约束、碰撞几何、执行器参数。从 CAD 到物理模型的转换不是简单的格式转换，而是一个**信息选择和参数重建**的过程。

这个类比：CAD 到仿真模型，类似于建筑设计图纸到结构力学模型。设计图纸包含所有细节（门窗位置、装修材料、水电线路），但结构力学模型只关心承重结构——梁、柱、板的截面尺寸和材料强度。你不能直接用设计图纸做结构分析，需要**提取**力学相关的信息并**补充**设计图纸中没有的力学参数。

### 如果跳过某些步骤会怎样

| 跳过的步骤 | 后果 | 症状 |
|-----------|------|------|
| Collision mesh 简化 | 碰撞检测极慢，仿真帧率 <1 fps | 训练无法启动或极慢 |
| 惯性参数验证 | 不物理的惯量导致数值不稳定 | 机器人在仿真中"抖动"或"飞走" |
| 坐标系对齐 | 关节旋转方向错误 | 机器人运动与预期镜像或交叉 |
| Actuator 配置 | 没有驱动力 | 机器人在重力下直接瘫倒 |
| Mesh 路径修复 | 模型加载失败 | FileNotFoundError |

### 全链路数据流

```text
SolidWorks 装配体 (.sldasm)
    │
    ├─→ sw2urdf 插件 ──→ URDF (.urdf) + STL meshes
    │                         │
    │   ┌─────────────────────┼─────────────────────┐
    │   │                     │                     │
    │   ▼                     ▼                     ▼
    │  MuJoCo 加载            Isaac Lab              直接使用
    │  + 手动调优              convert_urdf.py       （ROS/Gazebo）
    │   │                     │
    │   ▼                     ▼
    │  MJCF (.xml)           USD (.usd)
    │   │                     │
    │   ▼                     ▼
    │  mjlab                  Isaac Lab
    │  (MuJoCo Warp)          (PhysX)
    │   │                     │
    └───┴──→ RL 训练 ←────────┘
```

每个箭头都会丢失或引入信息：

| 转换步骤 | 丢失的信息 | 引入的信息 | 常见错误 |
|---------|----------|----------|---------|
| SolidWorks → URDF | 装饰细节、线缆 | joint 类型推断 | 关节方向错误 |
| URDF → MJCF | URDF 特有的 `<gazebo>` 标签 | actuator、default、contact | 坐标系不对齐 |
| URDF → USD | URDF 特有标签 | PhysX material、articulation | collision 类型默认值 |
| Mesh 简化 | 几何细节 | 凸包近似 | 孔洞被填充 |

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为"URDF 是通用格式，两个仿真器应该行为一致"。** URDF 只定义了运动学结构（link/joint 树）和基本物理参数（mass/inertia）。它**不定义**接触参数、执行器模型、求解器设置——这些由仿真器各自填充默认值。同一个 URDF 在 MuJoCo 和 PhysX 中的行为可能完全不同，因为默认的接触刚度、阻尼和摩擦模型不同。

### 练习

1. **[概念题]** 画出从 SolidWorks 到 mjlab 训练的完整数据流图，标注每个步骤需要的工具和可能引入的错误。
2. **[思考题]** 为什么 MuJoCo 选择 MJCF（自己的格式）而不是直接使用 URDF？MJCF 相对 URDF 多了哪些信息？

---

上一节建立了全链路的全局视角。接下来逐段讲解每个转换步骤的工程实现，从 SolidWorks → URDF 开始。

## 11.2 SolidWorks → URDF（sw2urdf 插件） ⭐⭐

> **这一节解决什么问题**：从 CAD 装配体导出 URDF 是全链路的第一步，也是错误最容易引入的步骤。讲解 sw2urdf 的安装、装配体要求和常见错误排查。

### 动机：sw2urdf 是 ROS 生态中最广泛使用的 CAD → URDF 工具

sw2urdf（`ros/solidworks_urdf_exporter`）是一个 SolidWorks 插件，它通过分析装配体中的配合关系（Mates）自动推断关节类型和旋转轴。它的输出是标准的 URDF 文件 + STL mesh 文件。

### 如果不用 sw2urdf 会怎样

手动写 URDF 对于简单的机器人（<10 个 link）是可行的，但对于人形机器人（20-30 个 link + 复杂 mesh）几乎不可能——手动指定每个 link 的惯性张量、关节位置和 mesh 文件路径的工作量巨大且极易出错。sw2urdf 自动化了这个过程，但它的输出通常需要手动修正。

### sw2urdf 安装与版本匹配

```text
sw2urdf 安装要求：
- SolidWorks ≥ 2018 SP5
- Visual Studio 2017 + .NET Desktop Development 工作负载
- SolidWorks API SDK（在 SolidWorks 安装时勾选）
- sw2urdf 当前版本：1.6.1

安装步骤：
1. 从 GitHub releases 下载 sw2urdf.dll
2. 在 SolidWorks 中：工具 → 插件管理 → 添加 → 选择 sw2urdf.dll
3. 重启 SolidWorks
4. 验证：工具菜单中应出现 "Export as URDF" 选项
```

### 装配体要求：每个 link 一个子装配体

sw2urdf 的核心假设是：**URDF 的每个 link 对应 SolidWorks 中的一个子装配体或零件。** 如果一个 link 由多个零件组成（如大腿 link 由骨架 + 外壳 + 电机组成），这些零件必须放在同一个子装配体中。

```text
正确的装配体结构：
robot_assembly.sldasm
  ├── base_link.sldasm           → URDF: base_link
  │     ├── base_frame.sldprt
  │     └── base_cover.sldprt
  ├── hip_link.sldasm            → URDF: hip_link
  │     ├── hip_motor.sldprt
  │     └── hip_bracket.sldprt
  ├── thigh_link.sldasm          → URDF: thigh_link
  │     ├── thigh_bone.sldprt
  │     ├── thigh_cover.sldprt
  │     └── knee_motor.sldprt
  └── ...

错误的装配体结构（零件散放）：
robot_assembly.sldasm
  ├── base_frame.sldprt          → sw2urdf 不知道这属于哪个 link
  ├── base_cover.sldprt
  ├── hip_motor.sldprt
  └── ...
```

### 坐标系定义：Z 轴对齐旋转轴

sw2urdf 使用 SolidWorks 中的坐标系来定义关节轴。**每个关节的旋转轴必须与某个坐标系的 Z 轴对齐。** 这是 URDF 的约定——revolute joint 的旋转轴由 `<axis xyz="0 0 1"/>` 指定（默认 Z 轴）。

```python
# URDF 中的关节定义
"""
<joint name="hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="hip_link"/>
  <origin xyz="0.0 0.05 0.0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  ← 旋转轴 = Z 轴
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
</joint>
"""
```

⚠️ **关键：如果 SolidWorks 中的装配坐标系 Z 轴没有对齐旋转轴，导出的 URDF 中关节方向会是错误的。** 自检方法：在 SolidWorks 中显示坐标系，确认每个关节处的 Z 轴（蓝色箭头）指向旋转方向。

### sw2urdf 导出的常见错误

```python
# sw2urdf_check.py — 检查 sw2urdf 导出的 URDF 常见错误
import xml.etree.ElementTree as ET
import os

def check_urdf(urdf_path):
    """检查 sw2urdf 导出的 URDF 的常见问题。"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    issues = []

    # 检查 1：mesh 路径是否使用了 package:// URI
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            issues.append(
                f"Mesh 使用 package:// URI: {filename}\n"
                f"  修复：替换为相对路径（如 meshes/xxx.stl）"
            )

    # 检查 2：惯性参数是否为零
    for inertial in root.iter('inertial'):
        mass_elem = inertial.find('mass')
        if mass_elem is not None:
            mass = float(mass_elem.get('value', '0'))
            if mass < 1e-6:
                parent = inertial.getparent() if hasattr(inertial, 'getparent') else 'unknown'
                issues.append(f"Link 的质量为零: mass={mass}")

        inertia = inertial.find('inertia')
        if inertia is not None:
            ixx = float(inertia.get('ixx', '0'))
            iyy = float(inertia.get('iyy', '0'))
            izz = float(inertia.get('izz', '0'))
            # 三角不等式检查
            if ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
                issues.append(
                    f"惯性张量违反三角不等式: "
                    f"Ixx={ixx}, Iyy={iyy}, Izz={izz}"
                )

    # 检查 3：关节限位是否合理
    for joint in root.iter('joint'):
        jtype = joint.get('type', '')
        if jtype == 'revolute':
            limit = joint.find('limit')
            if limit is not None:
                lower = float(limit.get('lower', '0'))
                upper = float(limit.get('upper', '0'))
                if lower >= upper:
                    issues.append(
                        f"Joint '{joint.get('name')}' 限位错误: "
                        f"lower={lower} >= upper={upper}"
                    )
                if upper - lower > 6.28:
                    issues.append(
                        f"Joint '{joint.get('name')}' 限位异常大: "
                        f"range={upper-lower:.2f} rad (>2π)"
                    )

    # 检查 4：STL 文件是否存在
    urdf_dir = os.path.dirname(urdf_path)
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if not filename.startswith('package://'):
            full_path = os.path.join(urdf_dir, filename)
            if not os.path.exists(full_path):
                issues.append(f"Mesh 文件不存在: {full_path}")

    print(f"检查完成: {len(issues)} 个问题")
    for i, issue in enumerate(issues):
        print(f"  [{i+1}] {issue}")
    return len(issues) == 0
```

### Mesh 路径修复：从 package:// 到相对路径

sw2urdf 默认生成的 mesh 路径使用 ROS 的 `package://` URI 格式。MuJoCo 和 Isaac Lab 都不认识这种格式——必须替换为相对路径。

```python
# fix_mesh_paths.py — 修复 sw2urdf 输出的 mesh 路径
import re

def fix_mesh_paths(urdf_path, output_path=None):
    """将 package:// URI 替换为相对路径。"""
    with open(urdf_path, 'r') as f:
        content = f.read()

    # 替换 package://xxx/meshes/yyy.stl → meshes/yyy.stl
    fixed = re.sub(
        r'package://[^/]+/meshes/',
        'meshes/',
        content
    )

    # 也处理 package://xxx/xxx/meshes/ 的情况
    fixed = re.sub(
        r'package://[^"]+/',
        'meshes/',
        fixed
    )

    output = output_path or urdf_path
    with open(output, 'w') as f:
        f.write(fixed)

    print(f"已修复 mesh 路径: {output}")
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：STL 文件是二进制格式。** sw2urdf 默认导出二进制 STL。MuJoCo 可以加载二进制 STL，但 Isaac Lab 的某些版本的 URDF importer 需要 ASCII STL 或 OBJ。推荐统一转换为 OBJ 格式（用 Blender 或 trimesh 库）。

⚠️ **编程陷阱：惯性参数在 SolidWorks 中按零件的质心计算，但 URDF 要求在 link 坐标系下表示。** sw2urdf 应该自动处理这个转换，但在某些版本中存在 bug——惯性参数可能没有正确变换。自检方法：在 MuJoCo 中加载后，打开 "Inertia" 可视化选项（红色椭球），检查每个 link 的惯性椭球是否合理。

💡 **概念误区：认为 sw2urdf 导出的 URDF 可以直接使用。** sw2urdf 输出的 URDF 通常是"基本正确但需要手动修正"的。常见的手动修正包括：mesh 路径修复、关节限位调整、惯性参数验证、添加碰撞几何（sw2urdf 可能只导出 visual mesh 而遗漏 collision mesh）。

### 练习

1. **[动手题]** 如果你有 SolidWorks，使用 sw2urdf 导出一个简单的 2-DOF 机械臂。如果没有，从 MuJoCo Menagerie 下载 UR5e 的 URDF 作为起点。用上面的 `check_urdf()` 脚本检查导出结果。
2. **[分析题]** sw2urdf 通过 SolidWorks 的配合关系（Mates）推断关节类型。同轴配合 → revolute，平面配合 → prismatic。如果一个关节有多个配合约束（如同轴 + 距离限制），sw2urdf 如何处理？
3. **[设计题]** 如果你要从 Onshape（在线 CAD 工具）导出 URDF，需要使用 `onshape-to-robot` 工具。与 sw2urdf 的流程有什么异同？

---

## 11.3 URDF → MJCF（MuJoCo 格式转换） ⭐⭐⭐

> **这一节解决什么问题**：URDF 是 ROS 生态的标准格式，但 MuJoCo 的原生格式是 MJCF。本节讲解从 URDF 到 MJCF 的转换流程，以 MuJoCo Menagerie 的标准工作流为范本。

### 动机：为什么需要 MJCF

URDF 定义了机器人的运动学结构（link/joint 树），但它**缺少**以下 MuJoCo 需要的关键信息：

| 缺失的信息 | MJCF 中的对应元素 | 默认行为（如果不指定） |
|-----------|-----------------|-------------------|
| 接触参数 | `<default><geom condim="3" friction="1 0.005 0.0001"/>` | MuJoCo 使用内部默认值 |
| 执行器 | `<actuator><position joint="hip" kp="100"/>` | **无驱动力——机器人瘫倒** |
| 求解器设置 | `<option solver="Newton" iterations="4"/>` | 默认 PGS 求解器 |
| 关节阻尼 | `<joint damping="0.5"/>` | 0（无阻尼） |
| 关键帧 | `<keyframe><key name="home" qpos="..."/>` | 无预设姿态 |
| 碰撞过滤 | `<contact><exclude body1="..." body2="..."/>` | 所有 body 之间检测碰撞 |

> **本质洞察：** URDF 只是机器人的"骨架"。MJCF 是"骨架 + 肌肉 + 皮肤 + 神经"——它不仅定义了结构，还定义了机器人如何与物理世界交互。从 URDF 到 MJCF 的转换不是格式翻译，而是**信息补全**。

### MuJoCo Menagerie 的标准转换流程

MuJoCo Menagerie（google-deepmind/mujoco_menagerie，50+ 模型）为 URDF → MJCF 转换建立了事实标准。以下是以 UR10e 为例的完整流程：

```bash
# ============================================
# Step 1: 准备 URDF 和 mesh 文件
# ============================================
# 下载 URDF（通常来自机器人厂商或 ROS 包）
git clone https://github.com/ros-industrial/universal_robot.git
cp -r universal_robot/ur10e_description ./

# ============================================
# Step 2: 将 DAE/STL mesh 转换为 OBJ 格式
# ============================================
# MuJoCo 推荐使用 OBJ 格式（而非 DAE 或 STL）
# 原因：OBJ 支持材质颜色，且 MuJoCo 的 mesh 加载器对 OBJ 最稳定

# 方法 A：使用 Blender 命令行批量转换
blender --background --python convert_dae_to_obj.py

# 方法 B：使用 trimesh（Python 库）
python -c "
import trimesh
mesh = trimesh.load('meshes/shoulder.dae')
mesh.export('meshes/shoulder.obj')
"

# ============================================
# Step 3: 使用 obj2mjcf 处理 OBJ 文件
# ============================================
# obj2mjcf 是 MuJoCo 官方工具，它：
# - 按材质分组拆分 OBJ
# - 生成 MuJoCo 兼容的 mesh 文件
# - 创建对应的纹理/材质配置
pip install obj2mjcf
obj2mjcf --obj-dir meshes/ --output-dir assets/

# ============================================
# Step 4: 在 URDF 中添加 MuJoCo 提示标签
# ============================================
# 在 URDF 的 <robot> 元素中添加：
# <mujoco>
#   <compiler discardvisual="false"/>
# </mujoco>
# 这告诉 MuJoCo 保留 visual mesh（否则默认丢弃）

# ============================================
# Step 5: 用 MuJoCo 加载 URDF 并保存 MJCF
# ============================================
python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('ur10e.urdf')
mujoco.mj_saveLastXML('ur10e.xml', model)
print('MJCF 已保存')
"

# ============================================
# Step 6: 手动调优 MJCF（最重要的步骤）
# ============================================
# 见下方详细说明
```

### Step 6 详解：MJCF 手动调优

MuJoCo 自动转换的 MJCF 只是起点——需要大量手动调优才能得到仿真可用的模型。以下是 Menagerie 项目标准化的调优步骤：

**6a. 提取公共属性到 `<default>` 块**

```xml
<!-- 调优前（每个 geom 重复写参数） -->
<body name="link1">
  <geom type="mesh" mesh="link1" rgba="0.7 0.7 0.7 1"
        condim="3" friction="1 0.005 0.0001" solref="0.02 1"/>
</body>
<body name="link2">
  <geom type="mesh" mesh="link2" rgba="0.7 0.7 0.7 1"
        condim="3" friction="1 0.005 0.0001" solref="0.02 1"/>
</body>

<!-- 调优后（用 default 避免重复） -->
<default>
  <default class="visual">
    <geom type="mesh" contype="0" conaffinity="0" group="2"/>
  </default>
  <default class="collision">
    <geom type="mesh" condim="3" friction="1 0.005 0.0001"
          solref="0.02 1" group="3"/>
  </default>
  <joint damping="0.5" armature="0.1"/>
</default>
```

**6b. 添加 actuator**

```xml
<!-- URDF 不定义 actuator——必须在 MJCF 中手动添加 -->
<actuator>
  <!-- 位置控制器（PD 控制） -->
  <position name="shoulder_pan" joint="shoulder_pan_joint"
            kp="100" ctrlrange="-6.28 6.28"/>
  <position name="shoulder_lift" joint="shoulder_lift_joint"
            kp="100" ctrlrange="-6.28 6.28"/>
  <position name="elbow" joint="elbow_joint"
            kp="100" ctrlrange="-3.14 3.14"/>
  <!-- ... 其余关节 -->
</actuator>
```

**actuator 类型选择指南：**

| actuator 类型 | MJCF 标签 | 适用场景 | 关键参数 |
|-------------|----------|---------|---------|
| `<position>` | PD 位置控制 | 高刚度关节（机械臂） | kp, kd |
| `<motor>` | 力矩控制 | RL 策略直接输出力矩 | gear |
| `<velocity>` | 速度控制 | 传送带、轮式底盘 | kv |
| `<general>` | 通用（可自定义） | 自定义 actuator 模型 | dyntype, gaintype, biastype |

`<general>` actuator 是 MuJoCo 中最灵活的 actuator 类型——通过组合 `gaintype`、`biastype` 和 `dyntype`，可以实现从 Ideal PD 到带宽限制 DC Motor 的任何线性 actuator 模型。详细的参数组合和工程用法在 Ch12（Actuator 建模与系统辨识）中深入讲解。

对于 RL 训练，最常用的是 `<position>` 类型——策略输出目标关节角度，PD 控制器计算对应力矩。这和 Ch05 讨论的 action space 设计一致：position action 比 torque action 更稳定、更容易训练。

**6c. 设计碰撞对过滤**

> **碰撞过滤规则**：MuJoCo 默认对同一 body 内的 geom 和 parent-child body 对禁用碰撞检测。但祖父-孙子或更远的 body 对需要手动 `<exclude>`。

相邻的 link 之间不应该检测碰撞——否则它们在任何姿态下都"碰撞"：

```xml
<contact>
  <!-- 排除相邻 link 的碰撞检测 -->
  <exclude body1="base_link" body2="shoulder_link"/>
  <exclude body1="shoulder_link" body2="upper_arm_link"/>
  <exclude body1="upper_arm_link" body2="forearm_link"/>
  <!-- 对于人形机器人，还需要排除大腿-躯干、小腿-大腿等 -->
</contact>
```

**6d. 添加关键帧**

```xml
<keyframe>
  <key name="home" qpos="0 -1.57 1.57 -1.57 -1.57 0"/>
  <key name="ready" qpos="0 -1.0 1.0 -1.0 -1.57 0"/>
</keyframe>
```

关键帧的工程作用：(1) 在 MuJoCo viewer 中快速切换到预设姿态检查模型，(2) 作为 RL 训练的初始状态（Ch04-Ch06 的 reset 姿态），(3) 定义 observation 中的 "default pose" 参考。

**6e. 为 MJX/Warp 创建简化版场景**

回顾 Ch03：MuJoCo Warp 在 GPU 上运行 batched simulation。GPU 碰撞检测对 mesh 复杂度非常敏感——CPU 能处理的精细 collision 在 GPU 上可能慢 10 倍。MuJoCo Menagerie 为每个模型提供两个场景文件：

```xml
<!-- scene.xml — CPU 仿真（精细 collision） -->
<include file="go1.xml"/>
<worldbody>
  <light pos="0 0 3"/>
  <geom name="floor" type="plane" size="10 10 0.1"
        material="grid"/>
</worldbody>

<!-- scene_mjx.xml — GPU 仿真（简化 collision） -->
<include file="go1.xml"/>
<!-- 替换精细的 mesh collision 为 primitive shapes -->
<compiler meshdir="assets_mjx/"/>
<worldbody>
  <light pos="0 0 3"/>
  <geom name="floor" type="plane" size="10 10 0.1"/>
</worldbody>
<option solver="Newton" iterations="4" ls_iterations="8"/>
```

**6f. 完整的 MJCF 调优示例（Go1 四足）**

> **6 步调优清单速查：**

| 步骤 | 做什么 | 关键参数 |
|------|--------|---------|
| 6a | default 块 | condim, friction, solref |
| 6b | actuator | kp, kd, forcerange |
| 6c | 碰撞排除 | `<exclude body1 body2>` |
| 6d | 关键帧 | home, crouch qpos |
| 6e | MJX 场景 | solver=Newton, primitive collision |
| 6f | 完整验证 | viewer 可视化检查 |

以下是一个经过调优的四足机器人 MJCF 的关键片段，展示了 default、actuator、contact 和 keyframe 的完整配置：

```xml
<?xml version="1.0"?>
<mujoco model="unitree_go1">
  <!-- 编译器设置 -->
  <compiler angle="radian" meshdir="assets/" autolimits="true"/>

  <!-- 全局选项 -->
  <option timestep="0.002" iterations="4" solver="Newton"
          gravity="0 0 -9.81"/>

  <!-- 默认属性（避免每个元素重复写） -->
  <default>
    <default class="go1">
      <joint damping="0.5" armature="0.01"
             limited="true" frictionloss="0.1"/>
      <geom condim="3" friction="0.8 0.02 0.01"
            solref="0.005 1"/>
      <position kp="80" kv="4" forcelimited="true"
                forcerange="-33.5 33.5"/>
    </default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2"
            material="go1_material"/>
    </default>
    <default class="collision">
      <geom type="mesh" group="3"/>
    </default>
  </default>

  <!-- Mesh 资产 -->
  <asset>
    <mesh name="trunk" file="trunk.obj"/>
    <mesh name="hip" file="hip.obj"/>
    <mesh name="thigh" file="thigh.obj"/>
    <mesh name="calf" file="calf.obj"/>
    <mesh name="foot" file="foot.obj"/>
    <material name="go1_material" rgba="0.2 0.2 0.2 1"/>
  </asset>

  <!-- 机器人结构（嵌套 body 树） -->
  <worldbody>
    <body name="trunk" pos="0 0 0.35">
      <!-- 自由关节（6-DOF base） -->
      <freejoint name="root"/>
      <geom type="mesh" mesh="trunk" class="visual"/>
      <geom type="box" size="0.2 0.05 0.05" class="collision"/>

      <!-- 右前腿 -->
      <body name="FR_hip" pos="0.1881 -0.04675 0">
        <joint name="FR_hip_joint" axis="1 0 0"
               range="-0.86 0.86" class="go1"/>
        <geom type="mesh" mesh="hip" class="visual"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 -0.08 0"
              class="collision"/>

        <body name="FR_thigh" pos="0 -0.08505 0">
          <joint name="FR_thigh_joint" axis="0 1 0"
                 range="-0.69 4.50" class="go1"/>
          <geom type="mesh" mesh="thigh" class="visual"/>
          <geom type="capsule" size="0.015" fromto="0 0 0 0 0 -0.213"
                class="collision"/>

          <body name="FR_calf" pos="0 0 -0.213">
            <joint name="FR_calf_joint" axis="0 1 0"
                   range="-2.72 -0.92" class="go1"/>
            <geom type="mesh" mesh="calf" class="visual"/>
            <geom type="capsule" size="0.01" fromto="0 0 0 0 0 -0.213"
                  class="collision"/>

            <!-- 足端 -->
            <body name="FR_foot" pos="0 0 -0.213">
              <geom type="sphere" size="0.02" class="collision"/>
              <site name="FR_foot_site" size="0.01"/>
            </body>
          </body>
        </body>
      </body>

      <!-- ... FL_hip, RL_hip, RR_hip 结构类似 ... -->
    </body>
  </worldbody>

  <!-- 执行器：12 个位置控制器（每条腿 3 个关节） -->
  <actuator>
    <position name="FR_hip_act" joint="FR_hip_joint" class="go1"/>
    <position name="FR_thigh_act" joint="FR_thigh_joint" class="go1"/>
    <position name="FR_calf_act" joint="FR_calf_joint" class="go1"/>
    <!-- ... 其余 9 个 actuator ... -->
  </actuator>

  <!-- 碰撞排除 -->
  <contact>
    <exclude body1="trunk" body2="FR_hip"/>
    <exclude body1="trunk" body2="FL_hip"/>
    <exclude body1="trunk" body2="RL_hip"/>
    <exclude body1="trunk" body2="RR_hip"/>
    <!-- 相邻 link 之间排除碰撞 -->
    <exclude body1="FR_hip" body2="FR_thigh"/>
    <exclude body1="FR_thigh" body2="FR_calf"/>
    <!-- ... 其余腿类似 ... -->
  </contact>

  <!-- 关键帧 -->
  <keyframe>
    <key name="home"
         qpos="0 0 0.35 1 0 0 0
               0 0.8 -1.6  0 0.8 -1.6
               0 0.8 -1.6  0 0.8 -1.6"/>
    <key name="crouch"
         qpos="0 0 0.25 1 0 0 0
               0 1.2 -2.4  0 1.2 -2.4
               0 1.2 -2.4  0 1.2 -2.4"/>
  </keyframe>
</mujoco>
```

这个 MJCF 文件有几个值得深入理解的工程决策：

- **`<freejoint>` 在 trunk 上**：让底座有 6-DOF 自由度（平移 3 + 旋转 3）。如果不加 freejoint，底座固定在世界坐标系——机器人不能移动
- **`armature="0.01"`**：给每个关节增加虚拟转子惯量。这不是物理精度的需要，而是**数值稳定性**的需要——没有 armature 时，轻量关节的加速度可能极大，导致积分不稳定
- **`forcerange="-33.5 33.5"`**：限制 actuator 的最大输出力矩（Go1 的电机额定力矩 33.5 Nm）。这在 RL 训练中很重要——如果不限制，策略可能学到"用极大力矩瞬间恢复"的行为，真机上做不到
- **capsule 作为 collision**：用 capsule 和 sphere 替代 mesh 作为碰撞几何。每个 link 只有 1 个碰撞基元——这让 GPU 碰撞检测极快
- **`condim="3"`**：三维摩擦锥（切向两维 + 旋转一维）。`condim="1"` 是无摩擦，`condim="3"` 是标准摩擦，`condim="6"` 加滚动和旋转摩擦

### Actuator 参数调优：从 viewer 演示到 RL 训练

Menagerie 模型的 actuator 参数是为 viewer 中的手动控制优化的——用户通过 slider 控制关节角度，kp 的设置让动作看起来"平滑"。但 RL 训练对 actuator 参数有不同的需求：

```python
# actuator_tuning.py — actuator 参数对 RL 训练的影响分析
"""
kp 对 RL 训练的影响：

kp 太小（如 kp=20）：
  - 策略输出 Δq=0.1 rad → 力矩 = kp * Δq = 2 Nm → 很小
  - 机器人在 DR 下（摩擦↓）无法保持站立
  - 症状：reward 不收敛，episode length 很短

kp 太大（如 kp=1000）：
  - 策略输出 Δq=0.01 rad → 力矩 = kp * Δq = 10 Nm → 很大
  - action space 的"有效分辨率"变低
  - 症状：关节抖动，action_rate penalty 很大

推荐范围（四足 locomotion）：
  hip:   kp = 80-200,  kv = 2-8
  thigh: kp = 80-200,  kv = 2-8
  calf:  kp = 80-200,  kv = 2-8

推荐范围（人形全身）：
  hip:      kp = 150-300, kv = 5-15
  knee:     kp = 150-300, kv = 5-15
  ankle:    kp = 40-80,   kv = 2-5
  shoulder: kp = 40-100,  kv = 2-5
  elbow:    kp = 40-100,  kv = 2-5
"""
```

### 双重解读：声明式 vs 命令式模型定义

URDF 和 MJCF 代表了两种不同的模型定义哲学，理解这个区别对调试至关重要：

**视角 A（URDF = 声明式）：** URDF 只声明"机器人有什么"——link、joint、mesh。它不声明"机器人怎么运动"——这由仿真器的内部模型决定。好处是通用性——同一个 URDF 可以在 MuJoCo、PhysX、ODE 等不同引擎中使用。坏处是缺乏控制——你无法在 URDF 中精确控制接触行为。

**视角 B（MJCF = 命令式）：** MJCF 不仅声明结构，还**命令**仿真器如何处理——接触参数、求解器设置、actuator 模型都可以精确指定。好处是精确控制——你可以调整每一个物理参数。坏处是丧失通用性——MJCF 模型只能在 MuJoCo 中使用。

这两个视角的工程含义：如果你的目标是"快速在两个框架中都能用"，从 URDF 出发是正确的选择。如果你的目标是"在 MuJoCo 中获得最好的仿真质量"，直接写 MJCF 或深度调优转换后的 MJCF 是必要的。

### 在 mjlab 中加载 MJCF 模型

mjlab 使用 `MjSpec` 加载模型（回顾 Ch03）：

```python
# mjlab 中加载自定义 MJCF 模型
import mujoco

# 方法 1：从 MJCF XML 文件加载
spec = mujoco.MjSpec.from_file("my_robot.xml")

# 方法 2：从 URDF 直接加载（MuJoCo 自动转换）
spec = mujoco.MjSpec.from_file("my_robot.urdf")

# 在 mjlab 的 EntityCfg 中使用
class MyRobotEntityCfg:
    @staticmethod
    def spec_fn() -> mujoco.MjSpec:
        spec = mujoco.MjSpec.from_file(
            str(Path(__file__).parent / "assets" / "my_robot.xml")
        )
        # 可以在 MjSpec 阶段做程序化修改
        # 例如：修改 actuator 增益
        for actuator in spec.actuators:
            if "hip" in actuator.name:
                actuator.kp = 200.0  # 增大髋关节的 PD 增益
        return spec
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：MuJoCo 加载 URDF 时默认丢弃 visual mesh。** 如果不在 URDF 中添加 `<mujoco><compiler discardvisual="false"/></mujoco>`，MuJoCo 只保留 collision geometry——模型在 viewer 中是一堆简陋的几何体而非精细的 mesh。

⚠️ **编程陷阱：MJCF 的 body 树结构与 URDF 不同。** URDF 是扁平的 link/joint 列表（joint 引用 parent/child link），MJCF 是嵌套的 body 树（child body 直接嵌套在 parent body 内）。转换时的坐标系变换容易出错——在 MuJoCo viewer 中打开 "Frame" 可视化选项检查每个 body 的坐标系方向。

🧠 **思维陷阱：Menagerie 模型可以直接用于 RL 训练。** Menagerie 模型是为 viewer 演示优化的，不一定适合 RL 训练。例如：actuator 的 kp/kd 可能太大（导致策略输出的 action 变化幅度过小）或太小（导致机器人在 DR 下不稳定）。RL 项目通常需要根据 Ch05 的 action space 设计原则重新调整 actuator 参数。

### 练习

1. **[动手题]** 从 MuJoCo Menagerie 下载 Unitree Go1 的模型。在 MuJoCo viewer 中打开，分别可视化 (a) visual mesh (b) collision mesh (c) inertia ellipsoid (d) contact points。记录你观察到的差异。
2. **[编码题]** 写一个 Python 脚本，读取 MJCF 文件，打印每个 body 的名称、质量和惯性张量主轴。用 Go1 模型测试。
3. **[跨章综合题]** 结合 Ch05（Observation 设计）：如果 MJCF 模型中的关节名称与 mjlab 的 observation term 期望的名称不一致（如 MJCF 用 "FR_hip_joint" 但 mjlab 期望 "front_right_hip"），会发生什么？如何修复？

---

## 11.4 URDF → USD（Isaac Lab 格式转换） ⭐⭐⭐

> **这一节解决什么问题**：Isaac Lab 使用 NVIDIA 的 USD（Universal Scene Description）格式。本节讲解从 URDF 到 USD 的转换流程和 PhysX 特有的参数配置。

### 动机：Isaac Lab 为什么选择 USD 而非 URDF

USD（Universal Scene Description）是 Pixar 开发的场景描述格式，被 NVIDIA 用作 Omniverse 生态的基础格式。Isaac Lab 选择 USD 的原因：

1. **可组合性**：USD 支持 "reference" 和 "payload" 机制——多个机器人可以引用同一个 mesh 而非复制，大幅减少内存占用
2. **Instanceable**：4096 个相同的机器人共享同一份 mesh 数据——这对大规模并行 RL 训练至关重要
3. **材质系统**：USD 的 MDL 材质比 URDF 的简单 rgba 颜色丰富得多，支持 PBR 渲染
4. **NVIDIA 生态一致性**：与 Isaac Sim、Omniverse 工具链无缝集成

### Isaac Lab 的 URDF → USD 转换工具

```bash
# 使用 Isaac Lab 的命令行工具转换 URDF → USD
./isaaclab.sh -p source/standalone/tools/convert_urdf.py \
    /path/to/my_robot.urdf \
    /path/to/output/my_robot.usd \
    --merge-joints \
    --make-instanceable
```

也可以在 Python 中使用 `UrdfConverter`：

```python
# Isaac Lab 中的 URDF → USD 转换
from isaaclab.sim.converters import UrdfConverterCfg, UrdfConverter

cfg = UrdfConverterCfg(
    asset_path="/path/to/my_robot.urdf",
    usd_dir="/path/to/output/",
    usd_file_name="my_robot.usd",
    # 关键参数
    fix_base=False,                  # 底座是否固定
    merge_fixed_joints=True,         # 合并固定关节（减少 link 数）
    make_instanceable=True,          # 生成 instanceable 资产
    # 碰撞近似
    default_drive_type="position",   # 默认驱动类型
    default_drive_stiffness=100.0,   # PD kp
    default_drive_damping=10.0,      # PD kd
)
converter = UrdfConverter(cfg)
converter.convert()
print(f"USD 文件已生成: {cfg.usd_dir}/{cfg.usd_file_name}")
```

### 在 Isaac Lab 中加载 USD 模型

```python
# Isaac Lab 中加载自定义机器人
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

MY_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/path/to/my_robot.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),  # 初始位置
        joint_pos={".*": 0.0},  # 所有关节归零
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*hip.*", ".*knee.*", ".*ankle.*"],
            stiffness=80.0,
            damping=4.0,
        ),
    },
)
```

注意 Isaac Lab 的 `ArticulationCfg` 中包含了 PhysX 特有的参数——这些在 MJCF 中没有对应物：

```python
# PhysX 特有参数详解
rigid_props = sim_utils.RigidBodyPropertiesCfg(
    # 最大速度限制（防止数值爆炸）
    max_linear_velocity=1000.0,     # m/s
    max_angular_velocity=1000.0,    # rad/s
    # 穿透修正速度（物体穿透时的分离速度）
    max_depenetration_velocity=100.0,  # m/s
    # 如果设得太小，穿透的物体分离太慢，看起来"粘在一起"
    # 如果设得太大，分离时会"弹飞"
)

articulation_props = sim_utils.ArticulationRootPropertiesCfg(
    # 自碰撞检测
    enabled_self_collisions=False,  # RL 训练中通常关闭（用 reward 惩罚代替）
    # PhysX 求解器迭代次数
    solver_position_iteration_count=4,   # 位置修正迭代
    solver_velocity_iteration_count=0,   # 速度修正迭代（0=只做位置修正）
    # 对于 RL 训练，4+0 是常用配置——足够稳定且不影响吞吐量
)
```

### UrdfConverterCfg 参数详解

Isaac Lab 的 URDF → USD 转换有大量可配置参数。以下逐项解释最重要的选项：

```python
from isaaclab.sim.converters import UrdfConverterCfg

cfg = UrdfConverterCfg(
    # ---- 输入/输出路径 ----
    asset_path="/path/to/robot.urdf",
    usd_dir="/path/to/output/",
    usd_file_name="robot.usd",

    # ---- 结构处理 ----
    fix_base=False,
    # True: 底座固定在世界坐标系（机械臂）
    # False: 底座自由（locomotion 机器人）

    merge_fixed_joints=True,
    # True: 合并所有 fixed joint 连接的 link（减少 link 数量）
    # False: 保留所有 link（如果你需要在 fixed link 上放传感器）
    # ⚠️ 对于有 gripper 的复合机器人，gripper 内部的 fixed link
    #    可能不应该被合并——合并会导致无法在指尖放 contact sensor

    # ---- Instanceable 资产 ----
    make_instanceable=True,
    # True: 分离 mesh 数据到独立文件（4096 envs 共享一份 mesh）
    # False: mesh 嵌入主 USD（不推荐用于 RL 训练）

    # ---- 默认驱动参数 ----
    default_drive_type="position",
    # "position": PD 位置控制（最常用于 RL）
    # "velocity": 速度控制
    # "effort":   力矩控制
    default_drive_stiffness=100.0,    # PD kp
    default_drive_damping=10.0,       # PD kd

    # ---- 碰撞近似 ----
    # collision_approximation 在转换时设置，不易后期修改
    # "convexHull": 默认，会填充凹面
    # "convexDecomposition": 保留凹面（推荐 manipulation）
    # "none": 不生成 collision（纯视觉）
    # "meshSimplification": 简化三角形数量
)
```

### Isaac Lab 的 MJCF → USD 转换

除了 URDF，Isaac Lab 也支持从 MJCF 直接转换。这在你已经有一个调优好的 MuJoCo 模型时非常有用：

```bash
# MJCF → USD 转换
./isaaclab.sh -p source/standalone/tools/convert_mjcf.py \
    /path/to/robot.xml \
    /path/to/output/robot.usd \
    --make-instanceable
```

MJCF → USD 转换时会保留 MJCF 中的 actuator 信息，但 PhysX 的 actuator 模型与 MuJoCo 不完全相同——需要在 Isaac Lab 的 `ArticulationCfg` 中重新配置 actuator 参数。

### URDF → USD 转换的完整验证脚本

```python
# verify_usd_conversion.py — 验证 URDF → USD 转换的正确性
"""
在 Isaac Lab 环境中运行此脚本。
检查转换后的 USD 模型是否正确。
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg

def verify_conversion(usd_path, urdf_path):
    """对比 USD 和原始 URDF 的关键属性。"""

    # 加载 USD 模型
    robot_cfg = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, 1)),
    )

    # 检查项
    checks = []

    # 1. 关节数量
    # 从 URDF 中解析预期的关节数
    import xml.etree.ElementTree as ET
    urdf_tree = ET.parse(urdf_path)
    urdf_joints = [j for j in urdf_tree.iter('joint')
                   if j.get('type') in ('revolute', 'prismatic', 'continuous')]
    expected_joints = len(urdf_joints)
    print(f"URDF 中的活动关节数: {expected_joints}")

    # 2. 总质量
    urdf_mass = sum(
        float(m.get('value', 0))
        for m in urdf_tree.iter('mass')
    )
    print(f"URDF 总质量: {urdf_mass:.3f} kg")

    # 3. 关节名称列表
    urdf_joint_names = sorted([j.get('name') for j in urdf_joints])
    print(f"URDF 关节名:")
    for name in urdf_joint_names:
        print(f"  - {name}")

    # 4. 提示用户在 Isaac Lab 中检查
    print(f"\n请在 Isaac Lab GUI 中打开 {usd_path} 验证:")
    print(f"  - 关节数量是否为 {expected_joints}")
    print(f"  - 总质量是否接近 {urdf_mass:.1f} kg")
    print(f"  - 所有 mesh 是否正确显示")
    print(f"  - collision mesh 是否合理")
```

### URDF → USD 转换的常见问题

| 问题 | 症状 | 原因 | 解决方案 |
|------|------|------|---------|
| 关节轴警告 | "articulation parse warning" | 关节轴与 body 主轴完全平行 | 微小扰动关节轴（加 1e-6） |
| Collision 填充孔洞 | 夹具无法夹持 | 默认 convexHull 填充凹面 | 改用 convexDecomposition |
| 超过 64 link 限制 | PhysX articulation 错误 | PhysX 历史限制 | 合并 fixed joint 或拆分 |
| Mesh 路径丢失 | 白色无纹理模型 | USD 引用相对路径与工作目录不一致 | 使用绝对路径或修正引用 |

### MuJoCo vs PhysX：同一 URDF 的行为差异

同一个 URDF 在 mjlab（MuJoCo）和 Isaac Lab（PhysX）中加载后，行为可能显著不同。以下代码帮助诊断差异来源：

```python
# cross_sim_compare.py — 对比同一模型在两个仿真器中的行为
def compare_free_fall(urdf_path):
    """
    在两个仿真器中做自由落体测试，比较 root 轨迹。
    如果轨迹差异 > 1cm，说明物理参数有显著差异。
    """
    # MuJoCo 自由落体
    mj_model = mujoco.MjModel.from_xml_path(urdf_path)
    mj_data = mujoco.MjData(mj_model)
    mj_data.qpos[2] = 1.0  # 从 1m 高度落下

    mj_trajectory = []
    for _ in range(100):
        mujoco.mj_step(mj_model, mj_data)
        mj_trajectory.append(mj_data.qpos[2])

    # Isaac Lab 自由落体（需要在 Isaac Sim 环境中运行）
    # ... PhysX 部分的代码

    # 比较
    # 如果 MuJoCo 和 PhysX 的轨迹差异 > 阈值，
    # 最可能的原因是接触参数（摩擦、弹性）不同
    print(f"MuJoCo final height: {mj_trajectory[-1]:.4f}")
    # PhysX 比较需要在 Isaac Lab 环境中执行
```

> **本质洞察：** MuJoCo 和 PhysX 使用不同的接触力模型。MuJoCo 使用"软接触"（complementarity-based），PhysX 使用"刚性接触"（impulse-based）。这意味着即使所有显式参数（质量、摩擦系数）完全相同，两个引擎的接触行为仍然不同。在 RL 训练中，这种差异通常被 Domain Randomization（Ch08）吸收——只要 DR 的范围足够宽，策略对接触模型的差异是鲁棒的。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Isaac Lab 的 `make_instanceable` 参数会把 mesh 数据分离到独立文件。** 如果只复制了主 USD 文件而没有复制 `Props/instanceable_assets.usd`，Isaac Lab 加载时 mesh 丢失——模型变成无形状的"骨架"。

⚠️ **编程陷阱：PhysX 的 articulation 限制 64 个 link。** 带有灵巧手的人形机器人（body 30 + hand 20 = 50 link）接近这个限制。解决方案：合并 fixed joint 以减少 link 数量（Isaac Lab 的 `merge_fixed_joints=True`）。

### 练习

1. **[动手题]** 用 Isaac Lab 的 `convert_urdf.py` 将 Menagerie 的 Go1 URDF 转换为 USD。在 Isaac Sim 的 GUI 中打开，对比 MuJoCo viewer 中的视觉效果差异。
2. **[分析题]** Isaac Lab 的 `UrdfConverterCfg` 有 `default_drive_type` 和 `default_drive_stiffness` 参数。这些对应 MJCF 中的什么？如果两个框架的默认值不同，RL 策略的行为会如何变化？
3. **[跨章综合题]** 结合 Ch08（Domain Randomization）：如果 MuJoCo 和 PhysX 对同一个 URDF 的接触行为不同，DR 的范围应该如何设置才能让策略在两个引擎中都能工作？

---

## 11.5 Collision Mesh 简化 ⭐⭐⭐

> **这一节解决什么问题**：CAD 导出的 mesh 通常有数万个三角形面——这对物理仿真的碰撞检测来说太复杂了。本节讲解为什么需要简化、如何简化、以及精度-速度的权衡。

### 动机：为什么仿真需要凸包而非三角形 mesh

物理仿真中的碰撞检测分为两步：(1) broad phase（快速排除明显不碰的对）和 (2) narrow phase（精确计算碰撞点和法向量）。Narrow phase 对凸包的计算复杂度是 $O(n)$，但对凹 mesh 是 $O(n^2)$ 或更高。一个有 10,000 个三角形面的 mesh 在 narrow phase 中极其缓慢——尤其是 MuJoCo 要在 GPU 上对 4096 个环境同时做碰撞检测时。

解决方案：把复杂的原始 mesh 分解为多个凸包的组合——这就是 **convex decomposition**。这类似于用乐高积木搭建复杂形状——每个乐高块是一个简单的凸几何体，但组合在一起可以近似任意形状。凸包数越多近似越精确，但碰撞检测的计算量也越大。

```text
原始 mesh（凹面、复杂几何）
    │
    ├─→ V-HACD / CoACD 分解
    │
    ▼
多个凸包的并集（每个凸包面数 <100）
    │
    ├─→ MuJoCo: <geom type="mesh" mesh="part_0"/>
    │           <geom type="mesh" mesh="part_1"/>
    │           ...
    │
    └─→ Isaac Lab: convexDecomposition 碰撞近似
```

### V-HACD vs CoACD：两种分解工具

```python
# 使用 CoACD 进行 convex decomposition（推荐用于操作任务）
import coacd
import trimesh

def decompose_mesh(input_mesh_path, output_dir, max_parts=16):
    """
    使用 CoACD 将复杂 mesh 分解为凸包集合。

    CoACD（Collision-Aware Convex Decomposition）相比 V-HACD：
    - 保留孔洞和凹面细节（夹具手柄不会被填充）
    - 生成更少的凸包（减少碰撞检测负载）
    - 推荐用于操作任务（抓取、插入）
    """
    mesh = trimesh.load(input_mesh_path)
    vertices = mesh.vertices.astype('float64')
    faces = mesh.faces.astype('int32')

    # CoACD 分解
    parts = coacd.run_coacd(
        vertices, faces,
        threshold=0.05,        # 误差阈值（越小越精确，越多凸包）
        max_convex_hull=max_parts,
        preprocess_mode="auto",
    )

    # 保存每个凸包
    for i, (verts, tris) in enumerate(parts):
        part_mesh = trimesh.Trimesh(vertices=verts, faces=tris)
        output_path = f"{output_dir}/collision_part_{i:03d}.obj"
        part_mesh.export(output_path)
        print(f"  Part {i}: {len(verts)} vertices, {len(tris)} faces")

    print(f"分解完成: {len(parts)} 个凸包")
    return len(parts)
```

| 工具 | 算法 | 优点 | 缺点 | 推荐场景 |
|------|------|------|------|---------|
| V-HACD | 体素层次化 | 快速、稳定 | 填充孔洞 | Locomotion（不需要抓取） |
| CoACD | 碰撞感知分解 | 保留孔洞、精度高 | 较慢 | Manipulation（抓取、插入） |

> **本质洞察：** V-HACD 和 CoACD 的核心区别是对"凹面"的处理方式。V-HACD 在分解时会填充凹面（因为凸包定义就是"没有凹面"），CoACD 则试图保留凹面特征——它不是把凹面填满，而是用多个小凸包来"围出"凹面的形状。对于夹具手柄、杯子把手等需要被抓取的零件，CoACD 是唯一正确的选择。

### 在 MJCF 中使用分解后的 collision mesh

```xml
<!-- 使用多个凸包作为一个 body 的碰撞几何 -->
<body name="gripper_finger">
  <!-- Visual mesh（精细，不参与碰撞） -->
  <geom type="mesh" mesh="finger_visual"
        class="visual"/>  <!-- contype="0" conaffinity="0" -->

  <!-- Collision mesh（多个凸包） -->
  <geom type="mesh" mesh="finger_collision_000"
        class="collision"/>
  <geom type="mesh" mesh="finger_collision_001"
        class="collision"/>
  <geom type="mesh" mesh="finger_collision_002"
        class="collision"/>
</body>
```

### 在 Isaac Lab 中配置 collision approximation

Isaac Lab 在 USD 转换时设置碰撞近似类型。但转换后可以在 Python 中动态修改：

```python
# 在 Isaac Lab 中修改 collision approximation
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.physx.scripts.utils as physx_utils

def set_collision_type(prim_path, collision_type="convexDecomposition"):
    """
    修改一个 prim 的碰撞近似类型。

    collision_type 选项：
    - "convexHull": 单个凸包（最快，但填充孔洞）
    - "convexDecomposition": 多个凸包（保留凹面，推荐 manipulation）
    - "meshSimplification": 简化三角形（保留形状但减少面数）
    - "none": 不做碰撞检测
    - "boundingCube" / "boundingSphere": 包围体
    """
    prim = get_prim_at_path(prim_path)
    collision_api = physx_utils.setCollider(prim, collision_type)
    print(f"Set {prim_path} collision to {collision_type}")
```

### V-HACD 的命令行使用

V-HACD 可以通过 Python 的 trimesh 库或命令行工具使用：

```python
# 使用 trimesh 调用 V-HACD
import trimesh

def vhacd_decompose(input_path, output_dir, max_hulls=16, resolution=100000):
    """
    使用 V-HACD 将 mesh 分解为凸包集合。

    Args:
        input_path: 输入 mesh 文件路径（OBJ/STL）
        output_dir: 输出目录
        max_hulls: 最大凸包数量
        resolution: 体素分辨率（越高越精确但越慢）
    """
    mesh = trimesh.load(input_path)

    # V-HACD 分解
    convex_parts = mesh.convex_decomposition(
        maxhulls=max_hulls,
        resolution=resolution,
    )

    if not isinstance(convex_parts, list):
        convex_parts = [convex_parts]

    # 保存每个凸包
    import os
    os.makedirs(output_dir, exist_ok=True)
    for i, part in enumerate(convex_parts):
        out_path = os.path.join(output_dir, f"hull_{i:03d}.obj")
        part.export(out_path)
        print(f"  Hull {i}: {len(part.vertices)} verts, "
              f"{len(part.faces)} faces")

    print(f"\n总共 {len(convex_parts)} 个凸包")
    return convex_parts
```

### 自动生成 MJCF 的 collision geom 声明

分解完成后，需要在 MJCF 文件中声明这些凸包。以下脚本自动生成对应的 XML 片段：

```python
# generate_collision_xml.py — 自动生成 collision geom 的 MJCF 片段
import os
import glob

def generate_collision_xml(hull_dir, body_name, default_class="collision"):
    """
    读取一个目录下的所有 hull_*.obj 文件，
    生成对应的 MJCF <geom> 和 <mesh> 声明。
    """
    hull_files = sorted(glob.glob(os.path.join(hull_dir, "hull_*.obj")))

    # 生成 <asset> 中的 mesh 声明
    asset_xml = []
    for f in hull_files:
        name = os.path.splitext(os.path.basename(f))[0]
        asset_xml.append(
            f'    <mesh name="{body_name}_{name}" file="{f}"/>'
        )

    # 生成 <body> 中的 geom 声明
    geom_xml = []
    for f in hull_files:
        name = os.path.splitext(os.path.basename(f))[0]
        geom_xml.append(
            f'      <geom type="mesh" mesh="{body_name}_{name}" '
            f'class="{default_class}"/>'
        )

    print("<!-- 添加到 <asset> 部分 -->")
    print("\n".join(asset_xml))
    print()
    print(f"<!-- 添加到 <body name=\"{body_name}\"> 部分 -->")
    print("\n".join(geom_xml))
```

### 精度 vs 速度实验

```python
# collision_benchmark.py — 不同 collision 精度的性能对比
def benchmark_collision_levels(mjcf_path, num_steps=1000):
    """
    对比不同 collision mesh 精度下的仿真速度。

    实验设计：
    - Level 0: 原始 mesh（baseline，最慢）
    - Level 1: V-HACD max_parts=32
    - Level 2: V-HACD max_parts=8
    - Level 3: primitive shapes（box/sphere/capsule）
    """
    import time

    levels = {
        "原始 mesh": "robot_original.xml",
        "V-HACD 32 parts": "robot_vhacd32.xml",
        "V-HACD 8 parts": "robot_vhacd8.xml",
        "Primitive shapes": "robot_primitive.xml",
    }

    for name, xml_path in levels.items():
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        data.qpos[2] = 0.5  # 从 0.5m 落下

        start = time.perf_counter()
        for _ in range(num_steps):
            mujoco.mj_step(model, data)
        elapsed = time.perf_counter() - start

        fps = num_steps / elapsed
        print(f"{name:25s}: {fps:8.1f} fps, "
              f"contacts/step={data.ncon:3d}")
```

典型结果（单 CPU, 单环境）：

| Collision 级别 | FPS | 接触点数/步 | 碰撞精度 |
|-------------|-----|----------|---------|
| 原始 mesh（50K 面） | ~200 | ~50 | 最高 |
| V-HACD 32 parts | ~2,000 | ~20 | 高 |
| V-HACD 8 parts | ~5,000 | ~8 | 中 |
| Primitive shapes | ~20,000 | ~4 | 低 |

对于 RL 训练（4096 并行环境），collision mesh 的精度等级应该选择 "V-HACD 8-16 parts" 或 "Primitive shapes"——因为 RL 训练需要的是吞吐量（每秒采样量）而非碰撞精度。

### MJX/Warp 的特殊要求

MuJoCo Warp（mjlab 的 GPU 后端）对 collision mesh 有额外要求：

```xml
<!-- scene_mjx.xml — 为 MJX/Warp 优化的场景配置 -->
<mujoco>
  <option solver="Newton" iterations="4" ls_iterations="8"/>

  <!-- MJX/Warp 需要更简化的碰撞几何 -->
  <!-- 用 primitive shapes 替代 mesh collision -->
  <worldbody>
    <body name="base_link">
      <!-- Visual: 精细 mesh -->
      <geom type="mesh" mesh="base_visual" class="visual"/>
      <!-- Collision: 简单 box -->
      <geom type="box" size="0.15 0.1 0.05" class="collision"
            pos="0 0 0.05"/>
    </body>
  </worldbody>
</mujoco>
```

MuJoCo Menagerie 为每个模型提供两个场景文件：`scene.xml`（CPU 仿真，精细 collision）和 `scene_mjx.xml`（GPU 仿真，简化 collision）。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：V-HACD 默认参数生成过多凸包。** 默认 `max_convex_hull=64` 对大多数机器人零件过于精细。从 `max_convex_hull=8` 开始，如果碰撞行为不满意再逐步增加。

⚠️ **编程陷阱：Isaac Lab 的 collision approximation 类型影响行为。** `convexHull`（默认）会填充所有凹面——对于手柄、杯子等需要抓取的物体是错误的。需要手动设置为 `convexDecomposition`。

💡 **概念误区：认为"collision mesh 越精细仿真越准确"。** 对于 RL 训练，碰撞检测的精度远不如吞吐量重要。一个用 8 个凸包近似的 collision mesh 在物理行为上与原始 mesh 几乎无差异，但仿真速度快 10 倍。

> **前沿替代工具（2025）：** 除了 V-HACD 和 CoACD，近期出现了新的碰撞分解工具。**foam**（arXiv 2503.13704, 2025）使用球体分解（sphere decomposition）替代凸包，在相同误差下凸包数量减少 ~70%。**Empart**（arXiv 2509.22847, 2025）提供交互式分解界面，适合需要精细控制的场景。目前 CoACD 仍然是最成熟和广泛支持的选择。

### 练习

1. **[动手题]** 下载一个复杂 mesh（如 Franka 机械臂的 link5），用 V-HACD 和 CoACD 分别分解。比较 (a) 生成的凸包数量 (b) 视觉质量 (c) 是否保留了凹面特征。
2. **[实验题]** 用 `benchmark_collision_levels` 脚本对比四种碰撞精度级别在 Go1 四足机器人上的性能。绘制 FPS vs 碰撞精度的 trade-off 曲线。

---

## 11.6 惯性参数估计与验证 ⭐⭐

> **这一节解决什么问题**：惯性参数（质量、质心位置、惯性张量）是仿真物理行为的基础。本节讲解如何获取、验证和修正这些参数。

### 动机：错误的惯性参数是仿真不稳定的首要原因

CAD 软件可以从 3D 模型自动计算惯性参数——但前提是每个零件的材料密度正确。如果 SolidWorks 中的零件没有指定材料（默认密度为 1000 kg/m³），导出的惯性参数可能与实际差一个数量级。

### 惯性张量的物理约束

一个合法的惯性张量 $I$ 必须满足以下条件：

```python
# inertia_check.py — 检查惯性张量的物理一致性
import numpy as np

def check_inertia(ixx, ixy, ixz, iyy, iyz, izz, mass):
    """
    检查惯性张量是否物理一致。

    物理约束：
    1. 所有主惯量 > 0
    2. 三角不等式：Ix + Iy >= Iz（循环）
    3. 质量 > 0
    4. pseudo-inertia 矩阵 J 正定
    """
    I = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz]
    ])

    issues = []

    # 检查 1：质量
    if mass <= 0:
        issues.append(f"质量非正: {mass}")

    # 检查 2：主惯量
    eigenvalues = np.linalg.eigvalsh(I)
    if any(e <= 0 for e in eigenvalues):
        issues.append(f"主惯量非正: {eigenvalues}")

    # 检查 3：三角不等式
    Ix, Iy, Iz = sorted(eigenvalues)
    if Ix + Iy < Iz:
        issues.append(
            f"违反三角不等式: {Ix:.6f} + {Iy:.6f} < {Iz:.6f}"
        )

    # 检查 4：惯量与质量的比值
    # 对于半径 r 的均匀球体，I = 0.4 * m * r²
    # 如果 I/m > r² 且 r 不合理，说明参数有问题
    max_I = max(eigenvalues)
    equiv_radius = np.sqrt(max_I / (0.4 * mass)) if mass > 0 else 0
    if equiv_radius > 1.0:  # 等效半径 > 1m 对于大多数机器人零件不合理
        issues.append(
            f"惯量/质量比异常: 等效半径 = {equiv_radius:.2f}m"
        )

    if issues:
        print(f"❌ 惯性参数不合法:")
        for issue in issues:
            print(f"   {issue}")
    else:
        print(f"✅ 惯性参数合法 (主惯量: {eigenvalues})")

    return len(issues) == 0
```

### 验证方法：三个简单测试

**测试 1：自由落体**

```python
# 从 1m 高度自由落体，检查落地时间
# 理论值：t = sqrt(2h/g) = sqrt(2/9.81) ≈ 0.452 秒
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)
data.qpos[2] = 1.0  # root z = 1m

t = 0
while data.qpos[2] > 0.05:
    mujoco.mj_step(model, data)
    t += model.opt.timestep

print(f"落地时间: {t:.3f}s (理论值 0.452s)")
# 如果差异 > 10%，检查质量参数
```

**测试 2：静止平衡**

```python
# 设置 home 姿态，检查是否能静止（没有 actuator 力的情况下）
data.qpos[:] = model.key_qpos[0]  # home 关键帧
for _ in range(1000):
    mujoco.mj_step(model, data)

drift = np.linalg.norm(data.qpos[:3] - model.key_qpos[0][:3])
print(f"位置漂移: {drift:.4f}m")
# 如果漂移 > 0.01m（且没有 actuator），
# 说明 home 姿态下的 gravity torque 不为零
# 常见原因：质心位置不正确
```

**测试 3：质心可视化**

```python
# 在 MuJoCo viewer 中可视化质心
# viewer 快捷键：Ctrl+I 显示 inertia ellipsoid
# 红色椭球的中心 = body 质心
# 椭球的形状 = 惯性张量的主轴
# 如果椭球严重偏离 body 几何中心 → 质心/惯量可能有误
```

### MuJoCo 的自动惯性计算

如果 URDF 中没有指定惯性参数（或者你不信任 CAD 导出的值），MuJoCo 可以从 collision geometry 自动计算：

```xml
<!-- MJCF 中的自动惯性计算 -->
<compiler inertiafromgeom="true"/>

<!-- 或者对单个 body 覆盖 -->
<body name="link1">
  <inertial pos="0 0 0" mass="1.5"
            diaginertia="0.01 0.01 0.005"/>
  <!-- 如果不写 inertial，MuJoCo 从 geom 推算 -->
</body>
```

⚠️ **注意：MuJoCo 的自动计算假设 geom 是均匀密度的。** 如果实际零件的质量分布不均匀（如电机在一端），自动计算的质心和惯量会偏差较大。

### Pseudo-Inertia 与 DR 中的物理一致性

Ch08 讨论了 Domain Randomization 对质量和惯量的随机化。但随机化惯性参数时有一个微妙的陷阱：**独立随机化质量、质心和惯性张量的对角元素可能产生物理不一致的参数。**

Wensing et al. (RA-L 2018) 定义了 **pseudo-inertia 矩阵**——一个 $4\times4$ 矩阵 $J(\pi)$，它是 10 维惯性参数向量 $\pi = (m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz})$ 的线性函数。物理一致性等价于 $J(\pi) \succ 0$（正定）。

```python
# pseudo_inertia.py — 检查惯性参数的物理一致性（Wensing LMI 方法）
import numpy as np

def build_pseudo_inertia(m, com, inertia_at_com):
    """
    构建 4x4 pseudo-inertia 矩阵。

    Args:
        m: 质量 (kg)
        com: 质心位置 (3,) relative to link frame
        inertia_at_com: 惯性张量 (3,3) at CoM
    Returns:
        J: 4x4 pseudo-inertia 矩阵
    """
    cx, cy, cz = com
    I = inertia_at_com

    # 平行轴定理：把惯性张量从 CoM 变换到 link frame
    # I_link = I_com + m * (c^T c I3 - c c^T)
    c_outer = np.outer(com, com)
    I_link = I + m * (np.dot(com, com) * np.eye(3) - c_outer)

    # 构建 pseudo-inertia 矩阵
    # J = [[0.5*tr(I_link)*I3 - I_link,  m*com],
    #      [m*com^T,                       m    ]]
    J = np.zeros((4, 4))
    J[:3, :3] = 0.5 * np.trace(I_link) * np.eye(3) - I_link
    J[:3, 3] = m * com
    J[3, :3] = m * com
    J[3, 3] = m

    return J

def check_physical_consistency(m, com, inertia_at_com):
    """检查惯性参数是否物理一致。"""
    J = build_pseudo_inertia(m, com, inertia_at_com)
    eigenvalues = np.linalg.eigvalsh(J)

    if all(e > 0 for e in eigenvalues):
        print(f"✅ 物理一致 (min eigenvalue: {min(eigenvalues):.6f})")
        return True
    else:
        print(f"❌ 物理不一致 (eigenvalues: {eigenvalues})")
        return False
```

**DR 中的正确做法**：不是独立随机化 mass 和 inertia，而是：

1. 采样一个密度缩放因子 $\rho \sim U(0.8, 1.2)$
2. mass → $\rho \cdot m_{\text{nominal}}$
3. inertia → $\rho \cdot I_{\text{nominal}}$（等比缩放保持物理一致性）
4. 可选：额外随机化 CoM 位置 $\Delta c \sim U(-0.01, 0.01)$

```python
# 物理一致的惯性参数随机化
def randomize_inertia_consistent(nominal_mass, nominal_inertia, nominal_com):
    """
    Ch08 Domain Randomization 中的物理一致惯性随机化。

    关键：mass 和 inertia 必须等比缩放，
    不能独立随机化（否则可能违反物理约束）。
    """
    # 密度缩放因子
    rho = np.random.uniform(0.8, 1.2)

    # 等比缩放
    new_mass = rho * nominal_mass
    new_inertia = rho * nominal_inertia  # 惯性 ∝ 密度 × 几何

    # CoM 扰动（可选，幅度要小）
    com_noise = np.random.uniform(-0.01, 0.01, size=3)
    new_com = nominal_com + com_noise

    # 验证
    assert check_physical_consistency(new_mass, new_com, new_inertia)
    return new_mass, new_inertia, new_com
```

### 完整的模型诊断工具

以下是一个综合诊断工具，覆盖惯性、关节、actuator 和 collision 的全方位检查：

```python
# model_diagnostics.py — 机器人模型综合诊断工具
import mujoco
import numpy as np

def full_diagnostics(mjcf_path):
    """
    对一个 MJCF 模型执行全方位诊断。
    适用于从 URDF 转换后的首次检查。
    """
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)

    report = {
        "basic": {},
        "inertia_issues": [],
        "joint_issues": [],
        "actuator_issues": [],
        "collision_info": {},
    }

    # ========== 1. 基本信息 ==========
    report["basic"] = {
        "nbody": model.nbody,
        "njnt": model.njnt,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "ngeom": model.ngeom,
        "total_mass": float(sum(model.body_mass)),
    }

    # ========== 2. 惯性诊断 ==========
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        mass = model.body_mass[i]
        inertia = model.body_inertia[i]

        if mass < 1e-8 and i > 0:
            report["inertia_issues"].append(
                f"Body '{name}': 质量为零 (mass={mass})"
            )
            continue

        if mass > 0:
            # 三角不等式
            Ix, Iy, Iz = sorted(inertia)
            if Ix + Iy < Iz * 0.99:
                report["inertia_issues"].append(
                    f"Body '{name}': 三角不等式违反 "
                    f"({Ix:.6f}+{Iy:.6f} < {Iz:.6f})"
                )

            # 等效半径检查
            max_I = max(inertia)
            equiv_r = np.sqrt(max_I / (0.4 * mass))
            if equiv_r > 0.5:
                report["inertia_issues"].append(
                    f"Body '{name}': 等效半径偏大 ({equiv_r:.3f}m)，"
                    f"检查 CAD 密度设置"
                )

    # ========== 3. 关节诊断 ==========
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if model.jnt_limited[i]:
            lo, hi = model.jnt_range[i]
            if lo >= hi:
                report["joint_issues"].append(
                    f"Joint '{name}': 限位错误 (lo={lo:.3f} >= hi={hi:.3f})"
                )
            if hi - lo > 2 * np.pi:
                report["joint_issues"].append(
                    f"Joint '{name}': 范围异常大 ({np.degrees(hi-lo):.0f}°)"
                )

        # 检查阻尼
        damping = model.jnt_stiffness[i]
        if damping == 0 and model.njnt > 1:
            # 对于多关节模型，无阻尼可能导致震荡
            pass  # 不一定是错误，但值得注意

    # ========== 4. Actuator 诊断 ==========
    if model.nu == 0:
        report["actuator_issues"].append(
            "⚠️ 没有 actuator！机器人将在重力下瘫倒。"
        )
    else:
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            # 检查 ctrlrange
            if model.actuator_ctrllimited[i]:
                lo, hi = model.actuator_ctrlrange[i]
                if lo >= hi:
                    report["actuator_issues"].append(
                        f"Actuator '{name}': ctrlrange 错误"
                    )

    # ========== 5. 碰撞信息 ==========
    n_visual = sum(1 for i in range(model.ngeom)
                   if model.geom_contype[i] == 0)
    n_collision = model.ngeom - n_visual
    report["collision_info"] = {
        "total_geoms": model.ngeom,
        "visual_geoms": n_visual,
        "collision_geoms": n_collision,
    }

    # ========== 打印报告 ==========
    print("=" * 60)
    print(f"模型诊断报告: {mjcf_path}")
    print("=" * 60)

    print(f"\n--- 基本信息 ---")
    for k, v in report["basic"].items():
        print(f"  {k}: {v}")

    print(f"\n--- 惯性诊断 ({len(report['inertia_issues'])} 个问题) ---")
    for issue in report["inertia_issues"]:
        print(f"  {issue}")

    print(f"\n--- 关节诊断 ({len(report['joint_issues'])} 个问题) ---")
    for issue in report["joint_issues"]:
        print(f"  {issue}")

    print(f"\n--- Actuator 诊断 ({len(report['actuator_issues'])} 个问题) ---")
    for issue in report["actuator_issues"]:
        print(f"  {issue}")

    print(f"\n--- 碰撞信息 ---")
    for k, v in report["collision_info"].items():
        print(f"  {k}: {v}")

    total_issues = (len(report["inertia_issues"]) +
                   len(report["joint_issues"]) +
                   len(report["actuator_issues"]))
    print(f"\n{'=' * 60}")
    if total_issues == 0:
        print("✅ 模型诊断通过")
    else:
        print(f"⚠️ 发现 {total_issues} 个问题需要修复")

    return report
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：PhysX 会"静默修正"不合法的惯性参数。** 如果你的惯性张量违反三角不等式，MuJoCo 会在加载时报错，但 PhysX 会自动修正并继续——这意味着你在 Isaac Lab 中不会收到任何警告，但仿真行为可能不正确。始终在 MuJoCo 中先加载检查。

💡 **概念误区：认为"惯性参数精确就好"。** 对于 RL 训练，惯性参数的精确度远不如"物理一致性"重要。一个精确但略有偏差的惯量可以被 DR 覆盖，但一个违反三角不等式的惯量会导致数值不稳定，DR 无法修复。

---

## 11.7 机器人模型库 ⭐

> **这一节解决什么问题**：大多数研究项目不需要从 SolidWorks 开始——可以直接使用现成的高质量模型。本节介绍三个主要的模型库。

### MuJoCo Menagerie

MuJoCo Menagerie（google-deepmind/mujoco_menagerie）是当前最大的开源 MuJoCo 模型集合，包含 50+ 个机器人模型，每个都经过手动调优。

```text
mujoco_menagerie/
├── unitree_go1/          # 四足
│   ├── go1.xml           # 主 MJCF 文件
│   ├── scene.xml         # CPU 仿真场景
│   ├── scene_mjx.xml     # GPU (MJX/Warp) 场景
│   ├── assets/           # OBJ mesh 文件
│   └── README.md         # 转换流程文档
├── unitree_g1/           # 人形
├── franka_emika_panda/   # 机械臂
├── anymal_c/             # 四足
├── robotiq_2f85/         # 夹具
└── ...
```

**Menagerie 模型的质量保证**：每个模型的 README 详细记录了从 URDF 到 MJCF 的转换步骤。这些文档本身就是最好的学习材料——它们展示了实际的手动调优决策（为什么选择特定的摩擦系数、为什么排除某些碰撞对）。

**按任务选择 Menagerie 模型的指南：**

| 任务类型 | 推荐模型 | 特点 | 注意事项 |
|---------|---------|------|---------|
| 四足 locomotion | unitree_go1, unitree_go2, anymal_c | 12 DoF, 轻量 | go1/go2 有真机对应 |
| 人形 locomotion | unitree_g1, unitree_h1 | 23-29 DoF | g1 是当前 sim2real 主流 |
| 机械臂操作 | franka_emika_panda, kuka_iiwa | 7 DoF | panda 是操作研究标准 |
| 灵巧手操作 | shadow_hand, allegro_hand | 16-24 DoF | 需要 collision 简化 |
| 双臂操作 | aloha | $2\times7$ DoF | 包含夹具 |

**快速开始代码——在 mjlab 中使用 Menagerie 模型：**

```python
# 在 mjlab 中加载 Menagerie 模型
# 方法 1：直接从本地路径加载
import mujoco
from pathlib import Path

menagerie_path = Path("mujoco_menagerie")
spec = mujoco.MjSpec.from_file(
    str(menagerie_path / "unitree_go1" / "scene.xml")
)
model = spec.compile()
data = mujoco.MjData(model)

# 打印基本信息
print(f"Bodies: {model.nbody}")
print(f"Joints: {model.njnt} (nq={model.nq}, nv={model.nv})")
print(f"Actuators: {model.nu}")
print(f"Total mass: {sum(model.body_mass):.2f} kg")
print(f"Timestep: {model.opt.timestep}")

# 设置 home 姿态并可视化
if model.nkey > 0:
    data.qpos[:] = model.key_qpos[0]
    mujoco.mj_forward(model, data)
    print(f"Root position at home: {data.qpos[:3]}")

# 方法 2：在 mjlab EntityCfg 中引用
# （需要将 Menagerie 模型放在项目的 assets 目录下）
```

### Isaac Lab 内置模型

Isaac Lab 在 `isaaclab_assets` 中内置了 16+ 个机器人模型，已经转换为 USD 格式并配置好 actuator 和 collision：

```python
# Isaac Lab 中直接使用内置模型
# 注意：Isaac Lab v2.0+ 使用 isaaclab_assets，v1.x 使用 omni.isaac.lab_assets
from isaaclab_assets import UNITREE_GO1_CFG, ANYMAL_D_CFG
from isaaclab_assets import UNITREE_G1_CFG, FRANKA_PANDA_CFG

# 内置模型的完整列表（截至 Isaac Lab 2.1）
AVAILABLE_ROBOTS = {
    # 四足
    "go1": UNITREE_GO1_CFG,
    "go2": UNITREE_GO2_CFG,
    "anymal_c": ANYMAL_C_CFG,
    "anymal_d": ANYMAL_D_CFG,
    "a1": UNITREE_A1_CFG,
    # 人形
    "g1": UNITREE_G1_CFG,
    "h1": UNITREE_H1_CFG,
    # 机械臂
    "panda": FRANKA_PANDA_CFG,
    "ur10e": UR10E_CFG,
    "kuka_iiwa": KUKA_IIWA_CFG,
    # 移动操作
    "ridgeback_ur5": RIDGEBACK_UR5_CFG,
}

# 使用内置模型
env_cfg.robot = UNITREE_GO1_CFG.replace(
    prim_path="/World/envs/env_.*/Robot",
)
```

### Isaac Lab 和 mjlab 内置模型的对应关系

两个框架的内置模型有大量重叠——同一个机器人在两个框架中都有现成模型。但参数（actuator kp/kd、collision 设置）可能不同：

| 机器人 | Menagerie (mjlab) | Isaac Lab Assets | 参数差异 |
|--------|-------------------|------------------|---------|
| Go1 | `unitree_go1/` | `UNITREE_GO1_CFG` | kp: 80 vs 100 |
| Go2 | `unitree_go2/` | `UNITREE_GO2_CFG` | collision 简化程度不同 |
| G1 | `unitree_g1/` | `UNITREE_G1_CFG` | 手部 DoF 处理不同 |
| ANYmal C | `anymal_c/` | `ANYMAL_C_CFG` | 足端接触模型不同 |
| Franka | `franka_emika_panda/` | `FRANKA_PANDA_CFG` | gripper 碰撞设置不同 |

**工程建议**：如果你的项目需要在两个框架中都运行（如论文中要对比 MuJoCo 和 PhysX 的 sim2real 效果），建议以 URDF 为中心——从同一个 URDF 分别生成 MJCF 和 USD，然后手动对齐 actuator 参数。不要分别使用两个框架的内置模型——因为参数可能已经在各自的调优过程中产生了偏差。

### awesome-loco-manipulation 的复合机器人

awesome-loco-manipulation（aCodeDog/awesome-loco-manipulation）整理了一系列**复合机器人**的 URDF——这些是在四足底盘上挂载机械臂的系统，如 Go2+Arx、B1+Z1、Aliengo+Z1：

```text
awesome-loco-manipulation/
├── go2_arx/
│   ├── go2_arx.urdf      # Go2 四足 + Arx 机械臂
│   ├── meshes/
│   └── README.md
├── b1_z1/
│   ├── b1_z1.urdf         # B1 四足 + Z1 机械臂
│   └── ...
├── b2w_z1/
│   ├── b2w_z1.urdf        # B2-W 轮式 + Z1 机械臂
│   └── ...
└── aliengo_z1/
    └── ...
```

复合机器人 URDF 的组织方式是把两个独立的 URDF（底盘 + 手臂）通过一个 fixed joint 连接：

```xml
<!-- 复合机器人 URDF 的结构 -->
<robot name="go2_arx">
  <!-- Go2 底盘部分（12 个 revolute joints） -->
  <link name="base_link">...</link>
  <joint name="FR_hip_joint" type="revolute">...</joint>
  <!-- ... 四足部分 ... -->

  <!-- 安装框架（fixed joint 连接底盘和手臂） -->
  <joint name="arm_mount" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Arx 机械臂部分（6 个 revolute joints） -->
  <link name="arm_base">...</link>
  <joint name="arm_joint_1" type="revolute">...</joint>
  <!-- ... 手臂部分 ... -->
</robot>
```

### 在 mjlab 中动态组合机器人

mjlab 的 `MjSpec.attach()` 允许在运行时动态组合多个模型——不需要预先生成复合 URDF：

```python
# mjlab 中动态组合机器人
import mujoco

def compose_loco_manipulation():
    """
    动态组合：Go2 底盘 + Arx5 机械臂。
    比预先生成复合 URDF 更灵活——可以在运行时切换手臂。
    """
    # 加载底盘
    chassis = mujoco.MjSpec.from_file("unitree_go2/go2.xml")

    # 加载机械臂
    arm = mujoco.MjSpec.from_file("arx5/arx5.xml")

    # 在底盘的 base_link 上挂载机械臂
    # prefix="arm/" 避免命名冲突
    # frame="base_link" 指定挂载点
    chassis.attach(
        arm,
        prefix="arm/",
        frame="base_link",
        pos=[0.15, 0, 0.1],  # 挂载偏移
        quat=[1, 0, 0, 0],   # 挂载朝向
    )

    # 编译
    model = chassis.compile()
    data = mujoco.MjData(model)

    # 验证
    print(f"组合后 Bodies: {model.nbody}")
    print(f"组合后 Joints: {model.njnt}")
    print(f"组合后 Actuators: {model.nu}")

    # 打印所有关节名
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  Joint {i}: {name}")

    return model, data
```

`MjSpec.attach()` 的工程优势：

1. **模块化**：底盘和手臂的 MJCF 各自独立维护和调优
2. **可复用**：同一个底盘可以挂载不同的手臂
3. **命名空间隔离**：`prefix` 参数避免两个模型中同名元素冲突
4. **与 Isaac Lab 的对比**：Isaac Lab 中组合机器人通常需要在 USD 层面做 reference——不如 `MjSpec.attach()` 灵活

### 在 Isaac Lab 中组合机器人

Isaac Lab 中组合机器人通常通过 USD reference 实现：

```python
# Isaac Lab 中的复合机器人配置
# 方法 1：使用预先生成的复合 USD
COMPOSITE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="go2_arx.usd",  # 预先从复合 URDF 转换
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*hip.*", ".*thigh.*", ".*calf.*"],
            stiffness=80.0, damping=4.0,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["arm_joint_.*"],
            stiffness=100.0, damping=10.0,
        ),
    },
)

# 方法 2：在代码中动态组合（需要 USD composition API）
# 这比 MjSpec.attach() 复杂得多，通常不推荐
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Menagerie 模型的 actuator 参数是为 viewer 演示优化的，不一定适合 RL。** 例如 Go1 的 `kp=100` 在 viewer 中看起来"刚好能站住"，但在 RL 训练中 DR 摩擦变小时可能不够。RL 项目通常需要调高 kp 到 200-400。

⚠️ **编程陷阱：复合机器人的碰撞排除不完整。** `MjSpec.attach()` 不会自动排除底盘和手臂之间的碰撞。手臂安装座处的碰撞几何可能与底盘重叠——需要手动添加 `<exclude>` 标签。

💡 **概念误区：认为"两个框架的内置模型参数一致"。** Menagerie 的 Go1 和 Isaac Lab Assets 的 Go1 是独立维护的——actuator kp、collision 设置、solver 参数可能不同。在做双框架对比实验时，以 URDF 为基准重新转换两端是更安全的做法。

### 练习

1. **[动手题]** 从 Menagerie 下载 3 个不同类型的模型（一个四足、一个机械臂、一个夹具），在 MuJoCo viewer 中分别可视化。记录每个模型的 (a) 总 body 数 (b) 总 joint 数 (c) actuator 类型和数量。
2. **[编码题]** 写一个脚本，读取一个 MJCF 文件，生成一份"模型信息卡"：总质量、质心位置、关节名列表、actuator 配置、collision geom 数量。

---

## 11.8 实验：端到端模型验证 ⭐⭐

> **这一节解决什么问题**：把前面所有步骤串联起来，完成一个机器人模型从 URDF 到双框架训练的完整验证。

### 动机：没有验证的模型是定时炸弹

模型中的错误可能在训练几千个 iteration 后才暴露——当你花了 12 小时训练完发现"原来关节 3 的旋转方向反了"时，一切都要重来。端到端验证确保你在开始训练前就发现了所有问题。

这类似于软件开发中的"冒烟测试"——不是完整的功能测试，而是快速检查"系统能不能基本运行"。模型验证是 RL 训练的冒烟测试。

### 如果不做验证会怎样

一个真实案例：某研究团队花了 2 周训练一个人形机器人的 locomotion 策略，reward 始终很低但缓慢上升。他们以为需要更多 iteration。最终发现：膝关节的旋转方向反了——策略学到的是"用反向弯曲的膝盖走路"，这当然很难学好。如果在训练前花 10 分钟做可视化检查，这个问题立即可以发现。

模型验证就像飞行前的检查清单——飞行员不会因为"上次飞没问题"就跳过检查。每次使用模型前的快速验证可以节省数天的调试时间。

### 实验设计

```text
实验目标：把同一个机器人模型分别加载到 mjlab 和 Isaac Lab，
验证两者的基本物理行为一致。

实验步骤：
1. 选择一个 URDF（如 Menagerie 的 Go1）
2. URDF → MJCF（手动调优）
3. URDF → USD（Isaac Lab converter）
4. 在两个框架中分别执行：
   a. 自由落体测试（比较落地时间）
   b. 关节摆动测试（给单个关节施加正弦力矩）
   c. velocity task 短训练（200 iterations，比较 reward 曲线）
5. 记录差异并分析原因
```

### 完整验证脚本

```python
# model_validation.py — 机器人模型双框架验证
import mujoco
import numpy as np

def validate_model(mjcf_path):
    """在 MuJoCo 中执行模型验证套件。"""
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)

    print("=" * 60)
    print(f"模型验证: {mjcf_path}")
    print("=" * 60)

    # ---- 基本信息 ----
    print(f"\n--- 基本信息 ---")
    print(f"Bodies: {model.nbody}")
    print(f"Joints: {model.njnt} (nq={model.nq}, nv={model.nv})")
    print(f"Actuators: {model.nu}")
    print(f"Geoms: {model.ngeom}")
    total_mass = sum(model.body_mass)
    print(f"总质量: {total_mass:.3f} kg")
    print(f"Timestep: {model.opt.timestep}")
    print(f"Solver: {['PGS', 'CG', 'Newton'][model.opt.solver]}")

    # ---- 惯性检查 ----
    print(f"\n--- 惯性检查 ---")
    inertia_issues = 0
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        mass = model.body_mass[i]
        inertia = model.body_inertia[i]
        if mass > 0.001:
            Ix, Iy, Iz = sorted(inertia)
            if Ix + Iy < Iz * 0.99:
                print(f"  ⚠️ Body '{name}': 三角不等式违反")
                inertia_issues += 1
    if inertia_issues == 0:
        print(f"  ✅ 所有 body 惯性参数合法")

    # ---- 自由落体测试 ----
    print(f"\n--- 自由落体测试 ---")
    data.qpos[:] = 0
    data.qvel[:] = 0
    if model.nq >= 7:  # 有 freejoint
        data.qpos[2] = 1.0  # root z = 1m
        data.qpos[3] = 1.0  # quat w = 1（单位四元数）
    else:
        print("  跳过（固定底座机器人）")

    steps = 0
    max_steps = int(2.0 / model.opt.timestep)  # 最多 2 秒
    while steps < max_steps:
        mujoco.mj_step(model, data)
        steps += 1
        if model.nq >= 7 and data.qpos[2] < 0.05:
            break

    if model.nq >= 7:
        fall_time = steps * model.opt.timestep
        theoretical = np.sqrt(2 * 1.0 / 9.81)
        error = abs(fall_time - theoretical) / theoretical * 100
        print(f"  落地时间: {fall_time:.3f}s (理论值: {theoretical:.3f}s)")
        print(f"  误差: {error:.1f}%")
        if error > 10:
            print(f"  ⚠️ 误差较大，检查接触参数或质量设置")

    # ---- 关节范围可视化 ----
    print(f"\n--- 关节范围 ---")
    for i in range(min(model.njnt, 20)):  # 只打印前 20 个
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if model.jnt_limited[i]:
            lo, hi = model.jnt_range[i]
            range_deg = np.degrees(hi - lo)
            print(f"  {name:30s}: [{np.degrees(lo):7.1f}°, "
                  f"{np.degrees(hi):7.1f}°] (range={range_deg:.1f}°)")

    # ---- Actuator 检查 ----
    print(f"\n--- Actuator 检查 ---")
    if model.nu == 0:
        print(f"  ⚠️ 没有 actuator！")
    else:
        for i in range(min(model.nu, 20)):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f"  {name:30s}: "
                  f"gain={model.actuator_gainprm[i, 0]:.1f}, "
                  f"bias={model.actuator_biasprm[i, 1]:.1f}")

    # ---- 关节摆动测试 ----
    print(f"\n--- 关节摆动测试（前 3 个关节）---")
    if model.nq >= 7:
        data.qpos[:] = model.key_qpos[0] if model.nkey > 0 else 0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    for j_idx in range(min(3, model.nu)):
        j_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j_idx)
        # 给关节施加正弦控制
        data.ctrl[:] = 0
        positions = []
        for t in range(200):
            data.ctrl[j_idx] = 0.3 * np.sin(2 * np.pi * t / 100)
            mujoco.mj_step(model, data)
            qpos_addr = model.jnt_qposadr[
                model.actuator_trnid[j_idx, 0]
            ]
            positions.append(data.qpos[qpos_addr])

        amp = max(positions) - min(positions)
        print(f"  {j_name}: 响应幅度 = {np.degrees(amp):.1f}°")
        if amp < 0.01:
            print(f"    ⚠️ 几乎无响应——检查 actuator 增益或 joint 类型")

    print(f"\n{'=' * 60}")
    print("验证完成")
```

### 双框架对比实验的结果分析

运行完验证后，对比两个框架的结果。以下是典型的差异及其根本原因：

| 测试项 | MuJoCo 结果 | PhysX 结果 | 差异原因 | 是否影响 RL |
|--------|------------|-----------|---------|-----------|
| 自由落体时间 | 0.452s | 0.450s | 数值积分差异 | ❌ 忽略 |
| 接触弹跳高度 | 0.02m | 0.05m | 接触模型不同 | ✅ DR 覆盖 |
| 关节摆动幅度 | 15° | 12° | 阻尼默认值不同 | ✅ 对齐参数 |
| 摩擦滑动距离 | 0.5cm | 2.0cm | 摩擦模型不同 | ✅ DR 覆盖 |

**结论**：两个框架在自由运动（无接触）时行为几乎一致，差异主要来自接触相关的物理模型。对于 RL 训练，这些差异通过 Domain Randomization（Ch08）处理——只要 DR 的范围足够宽，策略对接触模型差异是鲁棒的。

### 实验记录模板

```text
Ch11 模型验证实验记录
━━━━━━━━━━━━━━━━━━━
日期：
源 URDF：
目标 MJCF：
目标 USD：

基本信息：
  Bodies: ___
  Joints: ___ (nq=___, nv=___)
  Actuators: ___
  总质量: ___ kg

惯性检查：
  三角不等式违反数: ___
  质心可视化检查：通过 / 有异常

自由落体测试：
  mjlab 落地时间: ___ s  (理论 0.452s)
  Isaac Lab 落地时间: ___ s
  差异: ___% → 原因分析: ___

关节摆动测试：
  Joint 1 响应: mjlab ___° / Isaac Lab ___°
  Joint 2 响应: mjlab ___° / Isaac Lab ___°
  差异分析: ___

velocity task 短训练（200 iter）：
  mjlab reward@200: ___
  Isaac Lab reward@200: ___
  收敛趋势是否一致：是/否
  差异分析: ___

结论：
  模型是否可用于 RL 训练：是/否
  需要修正的问题：___
  下一步：开始 Ch04 velocity task 训练 / 修正模型
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：验证时忘记设置 home 关键帧。** 如果模型的初始 qpos 全为零，四足机器人的腿会完全伸直——这不是一个有意义的测试姿态。始终从 home 关键帧开始验证。

🧠 **思维陷阱：认为"两个框架的行为完全一致才能开始训练"。** 完全一致是不可能的（不同的接触模型）。目标是"基本行为一致、差异可被 DR 覆盖"。如果自由落体时间差异 <5%、关节响应幅度差异 <20%，通常就足够了。

### 练习

1. **[动手题]** 选择 Menagerie 的 Go1 模型，执行完整的验证流程。记录所有测试结果，标注发现的问题。
2. **[编码题]** 修改验证脚本，添加一个"静态平衡测试"：在 home 姿态下设置 actuator ctrl 为 home 的 joint pos，运行 1000 步，检查 root 位置是否漂移。
3. **[跨章综合题]** 结合 Ch07（PPO 训练）和 Ch08（DR）：如果模型验证发现 MuJoCo 和 PhysX 的接触弹跳差异为 3x，Ch08 的 DR 参数应该如何设置以覆盖这个差异？哪些 DR 参数最相关？

---

## 11.9 跨仿真器参数对齐 ⭐⭐

> **这一节解决什么问题**：当你需要在 mjlab 和 Isaac Lab 中使用同一个机器人时，如何确保两端的物理参数尽可能一致，减少因仿真器差异导致的策略迁移问题。

### 动机：为什么需要对齐

典型的研究场景是：在 mjlab 中快速实验（MuJoCo Warp 编译快、macOS 可预览），然后在 Isaac Lab 中大规模训练（PhysX GPU 吞吐量更高）。如果两端的物理参数不一致，在 mjlab 中调好的 reward 权重和超参数搬到 Isaac Lab 后可能完全不工作。

### 参数对齐清单

以下是从同一个 URDF 出发时，两端需要手动对齐的关键参数：

```python
# parameter_alignment.py — 跨仿真器参数对齐检查工具
"""
URDF 只定义了运动学和基本物理参数（mass/inertia）。
以下参数是仿真器各自填充的——需要手动确保一致。
"""
ALIGNMENT_CHECKLIST = {
    # ====== 接触参数 ======
    "friction_coefficient": {
        "mjcf_default": "friction='1 0.005 0.0001'",
        "physx_default": "static_friction=0.5, dynamic_friction=0.5",
        "action": "在 MJCF 和 Isaac Lab cfg 中统一设置",
        "priority": "HIGH",
    },

    # ====== Actuator 参数 ======
    "position_kp": {
        "mjcf_location": "<position kp='100'/>",
        "isaac_lab_location": "ImplicitActuatorCfg(stiffness=100)",
        "action": "确保两端数值相同",
        "priority": "CRITICAL",
    },
    "position_kd": {
        "mjcf_location": "<position kv='4'/> 或 joint damping",
        "isaac_lab_location": "ImplicitActuatorCfg(damping=4)",
        "action": "⚠️ MuJoCo 的 kv 和 joint damping 效果叠加",
        "priority": "CRITICAL",
    },
    "force_limit": {
        "mjcf_location": "<position forcerange='-33.5 33.5'/>",
        "isaac_lab_location": "ImplicitActuatorCfg(effort_limit=33.5)",
        "action": "确保两端的力矩限制一致",
        "priority": "HIGH",
    },

    # ====== 求解器参数 ======
    "timestep": {
        "mjcf_location": "<option timestep='0.005'/>",
        "isaac_lab_location": "SimCfg(dt=0.005)",
        "action": "必须完全一致",
        "priority": "CRITICAL",
    },
    "gravity": {
        "mjcf_location": "<option gravity='0 0 -9.81'/>",
        "isaac_lab_location": "SimCfg(gravity=(0, 0, -9.81))",
        "action": "通常不需要修改，但要确认方向一致",
        "priority": "LOW",
    },

    # ====== Collision 参数 ======
    "self_collision": {
        "mjcf_location": "<contact><exclude .../>",
        "isaac_lab_location": "enabled_self_collisions=False",
        "action": "MJCF 用精细排除，Isaac Lab 通常全局关闭",
        "priority": "MEDIUM",
    },
}


def check_alignment(mjcf_path, isaac_cfg):
    """对比 MJCF 和 Isaac Lab 配置的关键参数。"""
    import mujoco

    model = mujoco.MjModel.from_xml_path(mjcf_path)

    print("=" * 60)
    print("跨仿真器参数对齐检查")
    print("=" * 60)

    # 1. Timestep
    mj_dt = model.opt.timestep
    isaac_dt = isaac_cfg.get("dt", 0.005)
    match = abs(mj_dt - isaac_dt) < 1e-6
    print(f"\n[{'✅' if match else '❌'}] Timestep: "
          f"MuJoCo={mj_dt}, Isaac={isaac_dt}")

    # 2. Gravity
    mj_g = model.opt.gravity[2]
    isaac_g = isaac_cfg.get("gravity_z", -9.81)
    match = abs(mj_g - isaac_g) < 0.01
    print(f"[{'✅' if match else '❌'}] Gravity Z: "
          f"MuJoCo={mj_g}, Isaac={isaac_g}")

    # 3. Actuator gains
    if model.nu > 0:
        mj_kp = model.actuator_gainprm[0, 0]
        isaac_kp = isaac_cfg.get("kp", 100.0)
        match = abs(mj_kp - isaac_kp) < 1.0
        print(f"[{'✅' if match else '❌'}] Actuator kp (first): "
              f"MuJoCo={mj_kp}, Isaac={isaac_kp}")

    # 4. Joint damping
    if model.njnt > 0:
        mj_damp = model.dof_damping[6] if model.nv > 6 else 0
        isaac_damp = isaac_cfg.get("damping", 0.5)
        print(f"[{'⚠️' if abs(mj_damp - isaac_damp) > 0.5 else '✅'}] "
              f"Joint damping (first DOF): "
              f"MuJoCo={mj_damp:.2f}, Isaac={isaac_damp:.2f}")

    print(f"\n{'=' * 60}")
```

> **本质洞察：** MuJoCo 和 PhysX 的根本差异在于**接触力模型**。MuJoCo 使用"互补性约束"（soft contact），接触力通过求解 LCP 得到，有 `solref` 和 `solimp` 两组参数控制接触刚度和阻尼。PhysX 使用"基于脉冲的约束"（impulse-based），通过迭代求解器处理穿透。这意味着即使所有显式参数（质量、摩擦）完全一致，接触行为仍然不同。对于 RL 训练，这个差异通过 Domain Randomization（Ch08）处理——只要 DR 范围足够宽，策略可以同时适应两种接触模型。

### MuJoCo 特有的关键参数（Isaac Lab 无对应物）

| MuJoCo 参数 | 作用 | Isaac Lab 近似替代 |
|-------------|------|------------------|
| `solref` | 接触弹簧的参考阻尼/时间常数 | PhysX solver iterations |
| `solimp` | 接触阻抗的深度和宽度 | 无直接对应 |
| `armature` | 关节的虚拟转子惯量 | 通过增大 link inertia 近似 |
| `frictionloss` | 关节的库仑摩擦力矩 | 无直接对应 |
| `condim` | 接触维度（1/3/4/6） | PhysX 固定为 3D friction |

**实践建议**：不要试图让两个仿真器在接触层面完全一致——这是不可能的。目标是让**非接触行为**（关节响应、自由运动）一致，接触差异留给 DR 处理。具体做法：

1. **timestep、gravity** 必须完全一致
2. **actuator kp/kd、force limit** 必须完全一致
3. **mass/inertia** 必须完全一致（来自同一 URDF）
4. **摩擦系数** 设为相近值，DR 覆盖残差
5. **接触刚度/阻尼** 不做对齐（模型不同），DR 覆盖

### ⚠️ 常见陷阱

⚠️ **编程陷阱：MuJoCo 的 kv 和 joint damping 效果叠加。** 在 MJCF 中，`<position kv="4"/>` 和 `<joint damping="0.5"/>` **同时生效**——总阻尼是两者之和。Isaac Lab 的 `ImplicitActuatorCfg(damping=4)` 对应的是 **总阻尼**。如果你在 MJCF 中设了 kv=4 + joint_damping=0.5，Isaac Lab 应设 damping=4.5。

---

## 本章小结

| 知识点 | 核心结论 | 重要程度 |
|--------|---------|---------|
| 全链路数据流 | SolidWorks → URDF → MJCF/USD → RL 训练 | ⭐ |
| sw2urdf 装配体要求 | 每个 link 一个子装配体，Z 轴对齐旋转轴 | ⭐⭐ |
| Mesh 路径修复 | package:// → 相对路径，STL → OBJ | ⭐⭐ |
| URDF → MJCF 六步流程 | mesh 转换 → obj2mjcf → MuJoCo 加载 → 手动调优 | ⭐⭐⭐ |
| MJCF 手动调优六步 | default 块 + actuator + 碰撞过滤 + 关键帧 + MJX 场景 + 验证 | ⭐⭐⭐⭐ |
| 声明式 vs 命令式 | URDF = 骨架（通用），MJCF = 骨架+肌肉+神经（精确） | ⭐⭐ |
| Actuator 参数调优 | RL 训练的 kp 需要比 viewer 演示大 2-3 倍 | ⭐⭐⭐ |
| URDF → USD | Isaac Lab convert_urdf.py + instanceable + PhysX 参数 | ⭐⭐⭐ |
| UrdfConverterCfg | merge_fixed_joints, make_instanceable, default_drive_type | ⭐⭐⭐ |
| MuJoCo vs PhysX 行为差异 | 接触模型不同→DR 覆盖差异 | ⭐⭐⭐ |
| V-HACD vs CoACD | 填充孔洞 vs 保留凹面，操作任务必须用 CoACD | ⭐⭐⭐ |
| Collision 精度-速度权衡 | RL 训练优先吞吐量→简化 collision→8-16 parts | ⭐⭐ |
| MJX/Warp collision 要求 | GPU 后端需要 primitive shapes 或极简化 mesh | ⭐⭐ |
| 惯性张量三角不等式 | $I_x+I_y\ge I_z$（循环），MuJoCo 会拒绝不合法参数 | ⭐⭐⭐ |
| Pseudo-inertia LMI | 物理一致的 DR 随机化必须等比缩放 mass/inertia | ⭐⭐⭐ |
| 惯性验证三测试 | 自由落体 + 静止平衡 + 质心可视化 | ⭐⭐ |
| MuJoCo Menagerie | 50+ 模型，每个有详细的转换文档，是最佳学习材料 | ⭐⭐⭐ |
| Isaac Lab Asset Zoo | 16+ 内置 USD 模型，直接引用 CFG | ⭐⭐ |
| awesome-loco-manipulation | 复合机器人 URDF（Go2+Arx, B1+Z1） | ⭐⭐ |
| MjSpec.attach() 动态组合 | 运行时组合多个机器人/物体，prefix 避免命名冲突 | ⭐⭐⭐ |
| 跨仿真器参数对齐 | timestep/kp/kd/mass 必须一致，接触差异留给 DR | ⭐⭐⭐ |

本章建立了从 CAD 到仿真的完整工程链路。这条链路上的每一步都可能引入错误，而错误会在 RL 训练中被放大——一个关节方向反了、一个惯性参数不合法、一个 collision mesh 太精细，都可能导致训练失败或策略不收敛。验证不是可选的——它是训练前的必修步骤。

下一章（Ch12）将聚焦链路中的一个关键组件——actuator 模型。Ch11 只为 actuator 设置了基本的 kp/kd 值，Ch12 将深入讨论如何从真机的频率响应数据辨识出更精确的 actuator 模型，以及如何在 mjlab 和 Isaac Lab 中实现自定义 actuator。准确的 actuator 模型是 sim-to-real 成功的关键因素之一——如果仿真中的电机响应与真机差 2 倍，再好的 DR 也很难弥补。

---

> **Ch11 全章知识图谱**：本章覆盖了机器人资产管线的 9 个主要节（从全链路概览到跨仿真器对齐），涉及 4 个工具（sw2urdf、obj2mjcf、V-HACD/CoACD、Isaac Lab converter），3 个模型库（Menagerie、Isaac Lab Assets、awesome-loco-manipulation），以及 2 套验证方法（物理验证三测试 + 跨仿真器参数对齐）。如果你只有时间做一件事，请做"从 URDF 到 MJCF 的完整六步转换 + validate_model() 验证"——这是后续所有训练章节的基础。

## 累积项目

本章需要在你的累积项目中完成以下工作：

1. 选择一个机器人 URDF（推荐从 Menagerie 或 awesome-loco-manipulation 下载），完成 URDF → MJCF 的完整六步转换流程
2. 用 Isaac Lab 的 convert_urdf.py 从同一 URDF 生成 USD 版本
3. 在两个框架中分别加载，执行 `model_diagnostics.py` 和 `validate_model()` 全套验证
4. 运行 `check_alignment()` 脚本，确保关键参数对齐
5. 撰写模型信息卡和验证报告

### 实验 Lab：双框架一致性 A/B 对比

```python
# ch11_experiment_lab.py — Ch11 累积项目的实验框架
"""
实验设计：
  A 组: mjlab 中使用 MJCF 模型
  B 组: Isaac Lab 中使用 USD 模型（同一 URDF 来源）

对比指标：
  1. 自由落体落地时间
  2. 单关节正弦响应幅度
  3. 200 iteration velocity tracking reward
"""

experiment_configs = {
    "A_mjlab": {
        "framework": "mjlab",
        "model_path": "go1.xml",
        "num_envs": 4096,
        "max_iterations": 200,
    },
    "B_isaac_lab": {
        "framework": "isaac_lab",
        "model_path": "go1.usd",
        "num_envs": 4096,
        "max_iterations": 200,
    },
}

# 预期结果
expected_results = {
    "free_fall_time_diff": "< 5%（非接触行为应高度一致）",
    "joint_response_diff": "< 20%（取决于阻尼对齐质量）",
    "reward_200iter_diff": "< 30%（接触差异会影响早期训练）",
}
```

### 快速验证脚本

```python
# verify_ch11_completion.py — 检查 Ch11 累积项目完成度
def verify():
    import os

    checks = {
        "MJCF 文件存在": os.path.exists("my_robot.xml"),
        "USD 文件存在": os.path.exists("my_robot.usd"),
        "诊断报告存在": os.path.exists("model_diagnostics_report.txt"),
        "对齐检查存在": os.path.exists("alignment_check.txt"),
    }

    for name, passed in checks.items():
        status = "✅" if passed else "❌"
        print(f"{status} {name}")

    # 读取诊断报告检查是否有未修复的问题
    if os.path.exists("model_diagnostics_report.txt"):
        with open("model_diagnostics_report.txt") as f:
            content = f.read()
        if "⚠️" in content:
            print("⚠️ 诊断报告中有未修复的警告")
        else:
            print("✅ 诊断报告无警告")

verify()
```

### 与其他章节的连接

本章的模型是后续所有训练章节的基础——Ch04-Ch10 的 velocity tracking、motion imitation、AMP 训练都假设你有一个正确的机器人模型。如果模型有问题（关节方向错误、惯量不合理、collision 过精细），后续章节的所有训练都会受到影响。

本章的 actuator 配置直接影响 Ch05（Action Space 设计）——position actuator 的 kp 值决定了策略输出 action 到实际力矩的映射比例。Ch12 会深入讨论更精确的 actuator 模型。

本章的 collision mesh 简化影响 Ch08（Domain Randomization）的训练速度——更简化的 collision 意味着更高的仿真吞吐量，DR 实验可以更快完成。

本章的 pseudo-inertia 讨论为 Ch08 的惯性参数随机化提供了正确的方法——等比缩放而非独立随机化。

本章的 `MjSpec.attach()` 是 Ch20（loco-manipulation）的前置——复合机器人的动态组合是 loco-manipulation 任务的标准做法。

本章的跨仿真器对齐经验将在 Ch23（Sim-to-Real）中再次用到——从仿真到真机的差异比从 MuJoCo 到 PhysX 的差异更大，但对齐的方法论是相同的。

### 实验记录模板

```text
Ch11 累积项目实验记录
━━━━━━━━━━━━━━━━━━━
日期：
源 URDF 来源：Menagerie / awesome-loco-manipulation / 自有
机器人名称：
框架版本：mjlab ___ / Isaac Lab ___

URDF → MJCF 转换：
  Step 1 (mesh 转换): ✅/❌  注意事项：___
  Step 2 (obj2mjcf): ✅/❌
  Step 3 (MuJoCo 加载): ✅/❌
  Step 4 (手动调优):
    - default 块: ✅/❌
    - actuator (kp=___, kd=___): ✅/❌
    - 碰撞排除 (排除了 ___ 对): ✅/❌
    - 关键帧 (home: ___): ✅/❌
  Step 5 (MJX 场景): ✅/❌

URDF → USD 转换：
  merge_fixed_joints: True/False
  make_instanceable: True/False
  default_drive_type: position/velocity/effort
  default_drive_stiffness: ___
  问题：___

模型诊断：
  惯性问题数: ___
  关节问题数: ___
  actuator 问题数: ___
  collision geom 数: ___

参数对齐：
  timestep: MuJoCo=___ / Isaac=___  [✅/❌]
  kp: MuJoCo=___ / Isaac=___  [✅/❌]
  kd: MuJoCo=___ / Isaac=___  [✅/❌]
  friction: MuJoCo=___ / Isaac=___  [✅/❌]

验证测试：
  自由落体: MuJoCo=___s / Isaac=___s / 差异=___%
  关节响应: MuJoCo=___° / Isaac=___° / 差异=___%

结论：
  模型可用于 RL 训练：是/否
  下一步：___
```

## 延伸阅读

| 资料 | 难度 | 推荐原因 |
|------|------|---------|
| MuJoCo 建模文档 (mujoco.readthedocs.io/en/stable/modeling.html) | ⭐⭐ | MJCF 格式的权威参考，包含所有元素和属性 |
| MuJoCo Menagerie (google-deepmind/mujoco_menagerie) | ⭐⭐ | 50+ 模型的转换 README 是最好的实战学习材料 |
| sw2urdf 官方文档 (wiki.ros.org/sw_urdf_exporter) | ⭐ | SolidWorks 导出 URDF 的参考 |
| Isaac Lab 资产导入文档 (isaac-sim.github.io/IsaacLab) | ⭐⭐ | URDF/MJCF → USD 的官方工作流和参数说明 |
| Wei et al. 2022, "CoACD: Collision-Aware Convex Decomposition" (SIGGRAPH) | ⭐⭐⭐ | 理解 CoACD 为什么优于 V-HACD，操作任务必读 |
| Wensing et al. 2018, "Linear Matrix Inequalities for Physically-Consistent Inertial Parameter Identification" (RA-L) | ⭐⭐⭐ | 惯性参数物理一致性的数学基础，DR 随机化必读 |
| obj2mjcf 官方文档 (github.com/kevinzakka/obj2mjcf) | ⭐ | mesh 格式转换工具 |
| awesome-loco-manipulation (github.com/aCodeDog/awesome-loco-manipulation) | ⭐⭐ | 复合机器人 URDF 参考 |

**阅读顺序建议**：先读 MuJoCo Menagerie 中你关注的机器人的 README（理解转换流程），再读 MuJoCo 建模文档中 MJCF 的关键元素（default、actuator、contact），然后读 Isaac Lab 资产导入文档（理解 USD 工作流）。CoACD 和 Wensing 论文在需要 collision 简化或惯性 DR 时精读。

**论文精读优先级**：如果时间有限，最推荐精读的两个资源是 MuJoCo Menagerie 的 UR10e README（展示完整的六步转换流程）和 Isaac Lab 的 "Importing a New Asset" 文档（展示 URDF → USD 的完整工作流）。这两个资源覆盖了本章 90% 的工程知识。

**对于没有 CAD 经验的读者**：跳过 11.2 节的 sw2urdf 部分，直接从 Menagerie 下载现成模型开始学习 11.3-11.9 的转换和验证流程。当你有自己的机器人设计时再回来学 sw2urdf。

**对于只使用一个框架的读者**：如果你只用 mjlab，可以跳过 11.4（URDF→USD）和 11.9 的 Isaac Lab 部分。如果你只用 Isaac Lab，可以跳过 11.3 的 MJCF 手动调优细节——但建议至少了解 MJCF 的 default 和 actuator 机制，因为很多论文和开源项目使用 MJCF 格式。

**对于有 sim-to-real 需求的读者**：重点关注 11.6（惯性参数验证）和 11.9（跨仿真器对齐）——这两节的方法论直接适用于 Ch23 的 sim-to-real 流程。

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MuJoCo 加载 URDF 报 "file not found" | mesh 路径使用 package:// URI | 1. 运行 fix_mesh_paths.py 2. 检查 mesh 文件是否存在 | 本章 11.2 |
| 模型在 viewer 中是一堆白色方块 | discardvisual 未设置为 false | 1. 在 URDF 中添加 `<mujoco><compiler discardvisual="false"/>` 2. 重新转换 | 本章 11.3 |
| 机器人在重力下瘫倒 | 没有添加 actuator | 1. 检查 MJCF 是否有 `<actuator>` 部分 2. 添加 position 控制器 3. 检查 kp 值 | 本章 11.3 |
| 关节旋转方向相反 | sw2urdf 导出的关节轴方向错误 | 1. 检查 SolidWorks 坐标系 Z 轴 2. 在 URDF 中修改 `<axis xyz>` | 本章 11.2 |
| 仿真极慢（<10 fps） | collision mesh 太精细 | 1. 用 V-HACD/CoACD 简化 2. 减少凸包数到 8-16 3. 用 primitive shapes | 本章 11.5 |
| MuJoCo 报 "inertia too small" | 惯性参数不合法 | 1. 运行 inertia_check.py 2. 修正 CAD 材料密度 3. 设 inertiafromgeom="true" | 本章 11.6 |
| Isaac Lab 转换后 mesh 丢失 | instanceable 文件未复制 | 1. 检查 Props/ 目录是否存在 2. 复制 instanceable_assets.usd | 本章 11.4 |
| MuJoCo 和 PhysX 行为差异大 | 接触模型/默认参数不同 | 1. 运行 check_alignment.py 2. 对齐 kp/kd/timestep 3. 增大 DR 范围 | 本章 11.9 |
| 复合机器人 attach 后碰撞异常 | 底盘和手臂之间缺少碰撞排除 | 1. 添加 `<exclude>` 标签 2. 检查 contype/conaffinity | 本章 11.7 |
| Isaac Lab 报 "64 link limit" | PhysX articulation 链太长 | 1. 开启 merge_fixed_joints 2. 简化末端执行器 | 本章 11.4 |
| sw2urdf 导出的关节限位全为 0 | SolidWorks 中未定义 mate limit | 1. 在 SolidWorks 中设置角度限制 2. 手动修改 URDF limit | 本章 11.2 |
| Menagerie 模型 RL 训练不收敛 | actuator kp 值不适合 RL | 1. 检查 kp 值（viewer 默认偏低）2. 增大到 200-400 3. 参考 Ch05 action space | 本章 11.3 |
| MuJoCo 的 kv 和 Isaac Lab 的 damping 不一致 | MuJoCo 的 kv 和 joint damping 叠加 | 1. 检查 MJCF 中 actuator kv + joint damping 的总和 2. 在 Isaac Lab 设 damping = 总和 | 本章 11.9 |
| 惯性 DR 后 MuJoCo 报错 | 独立随机化违反三角不等式 | 1. 改用等比缩放（密度$\times$factor） 2. 验证 pseudo-inertia PSD | 本章 11.6 |
| CoACD 分解后凸包数过多 | threshold 太小 | 1. 增大 threshold 到 0.1 2. 减小 max_convex_hull | 本章 11.5 |
| V-HACD 填充了夹具的抓取空间 | V-HACD 对凹面做凸包填充 | 1. 改用 CoACD 2. 或手动设计 primitive collision | 本章 11.5 |

---

> **本章完。** 机器人资产管线是 RL 训练的基础设施。正确的模型让你把精力集中在算法和 reward 设计上，错误的模型让你在无尽的调试中迷失。投入时间做好 11.3-11.9 的全套验证——这是对后续所有章节最好的投资。

