> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# P02 sim-to-real 资产管道与多目标部署

> **本章定位**：本章是跨方向共享基础（P01-P02）的第二章。建立从 CAD 到仿真到真机的完整资产管线，掌握 URDF/MJCF/USD 多格式互转、物理参数辨识、域随机化配置、sim-to-real gap 的系统性分析、数字孪生同步，以及 RL + ros2_control 的混合部署架构。本章的内容对机械臂、足式、人形三个方向都是通用基础。
>
> **适用范围**：资产管道对任何需要仿真到真机迁移的机器人系统均适用。域随机化和 sim-to-real 方法论对 RL 训练尤其关键。
>
> **前置依赖**：P01（URDF/Xacro 建模）——理解 URDF 结构和 Xacro 参数化；基本 RL 概念（PPO、observation/action space）
>
> **下游章节**：M01-M15（机械臂核心系列全部使用本章的资产管道）、F09-F10（学习型力控需要域随机化）、D04/D10（双臂学习需要仿真环境）
>
> **建议用时**：1.5 周（10-15 小时）

---

## 知识导航

```
本章知识结构全景：

P02 sim-to-real 资产管道与多目标部署
│
├─ P02.1 URDF Single Source of Truth ⭐⭐       ← 管道设计：5 目标转换图
│        （CAD → URDF → {Gazebo, MuJoCo, Isaac, RViz, 真机}）
├─ P02.2 格式转换工具与陷阱 ⭐⭐                ← 工程实操：各目标的转换代码和后处理
│        （URDFtoMJCFConverter 完整实现）
├─ P02.3 域随机化 DR ⭐⭐⭐                      ← 理论+工程：DRO 数学框架 + Isaac Lab EventCfg
│        （标准套餐 8 项 + 课程式 DR + 视觉 DR）
├─ P02.4 物理参数辨识 SysId ⭐⭐⭐               ← 标定方法：回归形式推导 + L-BFGS-B 优化
│        （Swevers 回归 + 激励轨迹设计）
├─ P02.5 sim-to-real gap 四维分析 ⭐⭐⭐         ← 诊断框架：动力学/物理参数/感知/执行
│        （Reality Gap Metric 六维量化）
├─ P02.6 CRISP 混合架构 ⭐⭐                     ← 部署架构：GPU 策略 + C++ 合规控制
│        （5-10 Hz + 1 kHz 解耦）
├─ P02.7 数字孪生三层次 ⭐⭐⭐⭐                 ← 高级应用：ROS2 桥接 + 工业部署
│        （DigitalTwinBridge 完整实现）
├─ P02.8 MuJoCo Playground / MJX ⭐⭐⭐          ← 前沿工具：GPU 后端 + 零样本迁移
│        （mujoco-usd-converter 新工具）
└─ P02.9 跨仿真器一致性验证 ⭐⭐                 ← 质量保障：静态/运动学/动力学三层
         （CrossSimValidator 完整实现）
```

**推荐阅读路径**：
- **首次学习（推荐）**：P02.1 → P02.2 → P02.3 → P02.4 → P02.5 → P02.6（核心管道 + 标定 + 部署）
- **RL 训练方向**：P02.3 → P02.5 → P02.8（域随机化 + gap 分析 + MJX 加速）
- **真机部署方向**：P02.4 → P02.6 → P02.7（SysId + CRISP + 数字孪生）
- **工程参考/调试**：P02.2 → P02.9（格式转换 + 一致性验证）

**预计阅读时间**：
| 模式 | 时间 | 说明 |
|------|------|------|
| 精读（含练习） | 10-14 小时 | 逐节阅读 + 完成编程和推导练习 |
| 速读（仅理论） | 4-5 小时 | 阅读正文和代码注释，跳过推导细节 |
| 速查（查表参考） | 20-30 分钟 | 仅查阅 DR 标准套餐表、gap 四维分类表、故障排查手册 |

## 版本兼容表

| 工具/库 | 推荐版本 | 兼容范围 | 说明 |
|---------|---------|---------|------|
| MuJoCo | 3.1+ | 2.3 ~ 3.x | `from_xml_path` URDF 导入需 2.3+；`MjModel` copy 语义需 3.0+；MJX GPU 后端需 3.0+ |
| MuJoCo Python | 3.1+ | 3.0+ | `mujoco` PyPI 包；旧版 `mujoco-py` (OpenAI) 已废弃 |
| Isaac Sim | 4.2+ | 4.0 ~ 4.x | URDF Importer API 随版本频繁变化，`URDFParseAndImportFile` 命令需确认当前版本 |
| Isaac Lab | 1.0+ | 0.6 ~ 1.x | `EventCfg` DR 配置 API 自 0.6 起可用；`ManagerTermCfg` 自 1.0 起 |
| Gazebo | Harmonic (gz-sim 8) | Harmonic / Rolling | Gazebo Classic 已于 2025-01 EOL |
| gz_ros2_control | 1.0+ | Humble ~ Jazzy | `GazeboSimROS2ControlPlugin`(system plugin) vs `GazeboSimSystem`(hardware plugin) |
| Pinocchio | 3.0+ | 2.6 ~ 3.x | SysId 回归形式 `computeJointTorqueRegressor` 需 2.6+ |
| Docker | 24.0+ | 20.10+ | 多阶段构建需 17.05+；buildx 推荐 24.0+ |
| mujoco-usd-converter | 0.1+ | 0.1+ | 2025 年发布的 MJCF-USD 互转工具；USD → MJCF 方向仍为实验性 |

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **URDF 基础**：URDF 的 `<link>` 和 `<joint>` 分别描述什么？`<inertial>` 子元素中哪些参数最容易出错？ | P01 URDF/Xacro |
| 2 | **Xacro 宏**：如何用 `xacro:property` 和 `xacro:if` 实现同一个 URDF 根据参数切换仿真/真机模式？ | P01 Xacro 参数化 |
| 3 | **ROS2 基础**：什么是 `ros2_control` 的 `HardwareInterface`？它如何连接仿真和真机？ | 02_C++基础与进阶/ROS2 基础 |
| 4 | **RL 基础**：PPO 算法的 observation 和 action 分别是什么？什么是 reward shaping？ | RL 基础概念 |
| 5 | **Docker 基础**：多阶段构建（multi-stage build）的目的是什么？如何减小部署镜像体积？ | 02_C++基础与进阶/Docker 容器化部署 |

---

## 本章目标

学完本章后，你应该能够：

1. **理解**完整的 CAD → URDF → {Isaac Lab, MuJoCo, Gazebo, RViz, 真机} 资产管道，知道每个转换节点的工具和陷阱
2. **执行** URDF → MJCF 和 URDF → USD 的格式转换，识别并解决常见转换问题
3. **配置**运行时域随机化（质量/摩擦/延迟/噪声），理解"不改 URDF、运行时扰动"的正确做法
4. **进行**物理参数辨识（System Identification），用真机数据反向校准仿真参数
5. **分析** sim-to-real gap 的四大来源（动力学/物理参数/感知/执行），并采取对应弥补措施
6. **搭建** RL + ros2_control 的混合架构（CRISP 模式：GPU 策略 5-10 Hz + C++ 合规控制 1 kHz）

---

## P02.1 URDF 作为 Single Source of Truth 的资产管道 ⭐⭐

### 动机——为什么需要统一的资产管道？

一个机械臂项目通常需要在 5 个不同环境中使用同一个机器人模型：

1. **RViz**：可视化和调试
2. **Gazebo**：ROS2 系统集成测试
3. **MuJoCo**：快速物理仿真和 RL 训练
4. **Isaac Sim/Lab**：GPU 并行 RL 训练
5. **真机**：最终部署

如果为每个环境单独维护一份模型文件，当机器人硬件修改（如换了夹爪、改了连杆长度）时，你需要同步更新 5 份文件——**这是工程灾难的温床**。

### 如果不做统一管道会怎样

团队成员 A 在 Gazebo 里修正了一个惯性张量错误。成员 B 在 MuJoCo 里还用着旧参数。成员 C 在 Isaac Sim 里的 USD 文件是一个月前的版本。

结果：三个仿真器的行为不一致。在 Gazebo 里调好的控制参数，到 MuJoCo 上表现完全不同。最终部署到真机时，没人确定哪个仿真器的参数是对的。

这不是假设场景——这是大多数机器人实验室的日常。

### Single Source of Truth 架构

```
              CAD (SolidWorks / Fusion 360 / Onshape)
                     │
                     ▼  [solidworks_urdf_exporter / onshape-to-robot]
           URDF / Xacro (+ meshes)  ← 唯一版本控制的源文件
                     │
      ┌──────────────┼──────────────┬──────────────┬─────────────┐
      ▼              ▼              ▼              ▼             ▼
┌──────────┐  ┌───────────┐  ┌──────────┐  ┌────────────┐  ┌──────────┐
│ RViz     │  │ Gazebo    │  │ MuJoCo   │  │ Isaac Sim/ │  │ 真机     │
│(vis only)│  │(Harmonic) │  │ (MJCF)   │  │ Isaac Lab  │  │(ros2_   │
│          │  │ 原生 URDF │  │urdf→mjcf │  │(URDF → USD)│  │ control)│
└──────────┘  └───────────┘  └──────────┘  └────────────┘  └──────────┘
```

**关键不变量**：
- URDF 是**唯一**被版本控制的建模源文件
- 每个下游目标**不持有独立副本**，而是通过转换工具**即时生成**
- `ros2_control` 的 `<hardware><plugin>` 标签是连接五个目标的**唯一变量点**——仿真时加载仿真插件，真机时加载厂商驱动

> **本质洞察**：资产管道的核心不是"格式转换"——它是**版本控制策略**。URDF 是 source code（源码），MJCF/USD 是 build artifact（构建产物）。就像你不应该把编译后的 `.o` 文件提交到 Git 一样，你也不应该把 MJCF/USD 文件作为独立源来维护。它们应该由 CI/CD 管线从 URDF 自动生成。

**跨领域类比——编译器工具链**：URDF → MJCF/USD 的关系就像 C++ → x86/ARM 的关系。C++ 是高级源码（URDF），x86 和 ARM 是不同平台的目标代码（MJCF 和 USD）。你不会手动编辑 `.o` 文件——你修改 C++ 源码然后重新编译。同理，你不应该手动编辑 MJCF/USD——你修改 URDF 然后重新转换。

### 与 P01 的接口

回顾 P01（URDF/Xacro 建模）：在 P01 中我们学了如何从零编写 URDF，包括 link/joint/transmission 的完整语义、Xacro 宏系统、惯性参数计算。P01 的产出——一个高质量的参数化 URDF——正是本章资产管道的输入。

**P01 → P02 的检查清单**：

| 检查项 | P01 保证 | P02 需要 |
|--------|---------|---------|
| mesh 格式 | STL(碰撞) + DAE(可视化) | MuJoCo 只支持 STL/OBJ（不支持 DAE） |
| 惯性参数 | CAD 导出或估算 | SysId 精确标定（本章 P02.4） |
| 关节限位 | URDF `<limit>` 标签 | 仿真和真机共享同一限位 |
| ros2_control | `<hardware>` 标签 | 仿真/真机切换 plugin |
| 单位 | SI（米、千克、弧度） | 所有仿真器统一 SI |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：URDF 中用了绝对路径引用 mesh
   错误做法：<mesh filename="/home/user/robot/meshes/link0.stl"/>
   现象：在另一台电脑上无法加载 mesh（路径不存在），RViz 显示空白
   根本原因：绝对路径不可移植
   正确做法：<mesh filename="package://robot_description/meshes/link0.stl"/>
   使用 ROS2 package:// 协议，通过 ament_index 自动定位包路径

💡 概念误区：认为"MJCF 和 USD 可以手动编辑然后长期维护"
   新手想法："从 URDF 转出 MJCF 后，在 MJCF 里手动调了些参数，以后就用 MJCF 了"
   实际上：一旦 URDF 更新（如换了夹爪），你的手动修改全部丢失。
   更危险的是：你的 MJCF 参数和 URDF 参数可能已经不一致但没人发现。
   正确做法：所有修改在 URDF/Xacro 中进行，MJCF/USD 始终自动生成。
   如果 MJCF 需要额外参数（如 actuator），用 post-processing 脚本追加。

🧠 思维陷阱：认为"资产管道是一次性工作"
   新手想法："搭好管道后就不用管了"
   实际上：机器人硬件会迭代——换传感器、改夹爪、调连杆长度。
   资产管道必须是自动化的（CI/CD），每次 URDF 变更自动触发
   所有下游格式的重新生成和仿真回归测试。
   工业实践：用 GitHub Actions 在每次 URDF PR 时自动运行转换 + 仿真测试。
```

### 练习

1. **[编程]** 选 Franka Panda 或 UR5。写一套脚本：从 URDF 导出 MJCF（用 `mujoco.MjModel.from_xml_path`）、检查 Isaac Lab import、在 Gazebo Harmonic launch、在 RViz 可视化——同一份 URDF 支持所有 5 个目标。
2. **[思考题]** 为什么 Isaac Sim 选择 USD 而非 URDF 作为原生格式？USD 的 composition arcs（sublayers、references、inherits）解决了什么问题？提示：考虑大型场景中多个机器人 + 环境的组合管理。
3. **[编程]** 写一个 GitHub Actions workflow：每次 `robot_description/` 目录有变更时，自动 (a) 验证 URDF 语法 (b) 转换为 MJCF 并验证关节数 (c) 在 Gazebo 中运行 5 秒仿真确认不崩溃。

---

## P02.2 各目标的转换工具与陷阱 ⭐⭐

### Gazebo Harmonic

**转换方式**：原生读取 URDF，通过 `<gazebo>` 扩展标签配置物理属性。

```xml
<!-- Gazebo 特定配置（在 URDF 的 Xacro 中条件加载） -->
<xacro:if value="$(arg use_gazebo)">
  <gazebo reference="panda_link0">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1e6</kp>
    <kd>1e2</kd>
  </gazebo>
  
  <!-- gz_ros2_control 插件 -->
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find robot_description)/config/ros2_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</xacro:if>
```

**关键注意**：Gazebo Classic 已于 2025-01 EOL（End of Life）。所有新项目应使用 Gazebo Harmonic 或更新版本。Classic 和 Harmonic 的 API 和配置有显著差异——旧教程中的 `<gazebo>` 插件配置**不能直接用于 Harmonic**。在 URDF/SDF 的 `<gazebo>` 标签里加载的是 Gazebo system plugin，应写 `gz_ros2_control::GazeboSimROS2ControlPlugin`；`gz_ros2_control::GazeboSimSystem` 是 ros2_control 的 hardware plugin 概念，出现在 `ros2_control` 硬件配置链路中，不应作为 Gazebo 系统插件名放在这里。

### MuJoCo 转换与三大陷阱

**转换方式**：用官方 Python API `mujoco.MjModel.from_xml_path('robot.urdf')` 直接加载，再用 `mujoco.mj_saveLastXML()` 保存 MuJoCo 编译后的 MJCF。MuJoCo Python 包没有官方的一键"URDF 转 MJCF 并自动加 actuator"命令；actuator 通常需要保存后用 XML 后处理脚本补上，或使用项目中明确锁定版本的第三方转换工具。

```python
import mujoco
import mujoco.viewer

# 方法 1: 直接从 URDF 加载（MuJoCo 3.x 原生支持）
model = mujoco.MjModel.from_xml_path('robot.urdf')

# 方法 2: 保存为 MJCF（可手动或脚本化编辑后使用）
mujoco.mj_saveLastXML('robot.xml', model)

# 验证加载结果
print(f"Bodies: {model.nbody}")
print(f"Joints: {model.njnt}")
print(f"Actuators: {model.nu}")   # 如果为 0，需要手动添加 actuator
print(f"DOF: {model.nv}")

# 打印关节名称和顺序
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(f"  Joint {i}: {name}")
```

**三大陷阱详解**：

```
⚠️ MuJoCo 陷阱一：只导入 collision mesh，丢弃 visual mesh
   现象：MuJoCo viewer 中机器人显示为简单几何体（圆柱/盒子），没有精细外观
   原因：MuJoCo 的渲染默认使用 collision geometry
   解决方案：
   方法 A: 在转换后的 MJCF 中手动添加 visual-only geom
           <geom type="mesh" file="link0_visual.obj" contype="0" conaffinity="0"/>
   方法 B: 用 mujoco_menagerie 的高质量 MJCF 模型（已包含 visual）

⚠️ MuJoCo 陷阱二：不支持 DAE（Collada）格式 mesh
   现象：加载含 DAE mesh 的 URDF 时报错 "unknown mesh extension"
   原因：MuJoCo 只接受 STL 和 OBJ 格式
   解决方案：
   方法 A: 用 Blender 批量转换 DAE → OBJ
           blender --background --python convert_dae_to_obj.py
   方法 B: 确保 URDF 中 collision 始终用 STL，visual 用 DAE（但 MuJoCo 不用 visual）

   ⚠️ MuJoCo 陷阱三：URDF 导入后 actuator 为空
   现象：model.nu == 0，data.ctrl 数组为空，关节无法控制
   原因：URDF 的 <transmission> 标签没有 MuJoCo actuator 的等价物
   解决方案：
   方法 A: 用 MjModel.from_xml_path() 加载 URDF，再用 mj_saveLastXML() 保存 MJCF，
           通过下面的后处理脚本为 hinge/slide 关节添加 actuator
   方法 B: 手动在 MJCF 中添加:
           <actuator>
             <position joint="panda_joint1" kp="100"/>
             <position joint="panda_joint2" kp="100"/>
             ...
           </actuator>
```

### URDF → MJCF → USD 格式互转完整代码

**URDF → MJCF 自动转换 + 后处理脚本**：

```python
# scripts/urdf_to_mjcf.py — 自动化 URDF → MJCF 转换
import mujoco
import mujoco.viewer
import xml.etree.ElementTree as ET
import shutil
import os
from pathlib import Path

class URDFtoMJCFConverter:
    """URDF → MJCF 转换器（含后处理）"""
    
    def __init__(self, urdf_path: str, output_dir: str = "mjcf_output"):
        self.urdf_path = Path(urdf_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    def convert(self, add_actuators=True, fix_meshes=True):
        """完整转换流程"""
        # Step 1: MuJoCo 原生加载 URDF
        model = mujoco.MjModel.from_xml_path(str(self.urdf_path))
        
        # 验证加载
        print(f"Loaded: {model.nbody} bodies, {model.njnt} joints, "
              f"{model.nu} actuators, {model.nv} DOF")
        
        # Step 2: 如果 actuator 为空，生成 MJCF 并手动添加
        if model.nu == 0 and add_actuators:
            print("WARNING: No actuators found, adding position actuators")
            mjcf_path = self._add_actuators_to_mjcf(model)
        else:
            mjcf_path = self.output_dir / "robot.xml"
            mujoco.mj_saveLastXML(str(mjcf_path), model)
        
        # Step 3: 修复 mesh 路径
        if fix_meshes:
            self._fix_mesh_paths(mjcf_path)
        
        # Step 4: 验证最终 MJCF
        final_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        print(f"Final MJCF: {final_model.njnt} joints, "
              f"{final_model.nu} actuators")
        
        # Step 5: 打印关节映射（关键！）
        print("\nJoint mapping:")
        for i in range(final_model.njnt):
            name = mujoco.mj_id2name(
                final_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"  [{i}] {name}")
        
        return str(mjcf_path)
    
    def _add_actuators_to_mjcf(self, model):
        """为无 actuator 的 MJCF 添加位置执行器"""
        # 先保存原始 MJCF
        temp_path = self.output_dir / "robot_no_actuator.xml"
        mujoco.mj_saveLastXML(str(temp_path), model)
        
        tree = ET.parse(temp_path)
        root = tree.getroot()
        
        # 创建 actuator 块。position actuator 的 ctrlrange 应来自关节
        # range/limit：hinge 使用 rad，slide 使用 m；不要硬编码 -3.14 3.14。
        actuator_elem = ET.SubElement(root, 'actuator')
        for i in range(model.njnt):
            name = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_type = model.jnt_type[i]
            if name and joint_type in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                attrs = {
                    'name': f'act_{name}',
                    'joint': name,
                    'kp': '100' if joint_type == mujoco.mjtJoint.mjJNT_HINGE else '1000',
                }

                if model.jnt_limited[i]:
                    lower, upper = model.jnt_range[i]
                    attrs['ctrllimited'] = 'true'
                    attrs['ctrlrange'] = f'{lower:.12g} {upper:.12g}'
                else:
                    attrs['ctrllimited'] = 'false'
                    print(f"WARNING: joint {name} has no limit; "
                          "position actuator ctrlrange left unlimited")

                ET.SubElement(actuator_elem, 'position', attrs)
        
        output_path = self.output_dir / "robot.xml"
        tree.write(str(output_path), encoding='unicode')
        return output_path
    
    def _fix_mesh_paths(self, mjcf_path):
        """修复 MJCF 中的 mesh 路径"""
        tree = ET.parse(mjcf_path)
        root = tree.getroot()
        
        # 查找所有 mesh 引用
        for mesh in root.iter('mesh'):
            if 'file' in mesh.attrib:
                old_path = mesh.attrib['file']
                # 如果是绝对路径，转为相对路径
                if os.path.isabs(old_path):
                    rel_path = os.path.relpath(old_path, self.output_dir)
                    mesh.attrib['file'] = rel_path
                    
                # 如果是 DAE 格式，警告
                if old_path.endswith('.dae'):
                    print(f"WARNING: DAE mesh not supported: {old_path}")
                    print(f"  Convert to STL/OBJ first")
        
        tree.write(str(mjcf_path), encoding='unicode')

# 使用示例
# converter = URDFtoMJCFConverter("robot.urdf")
# mjcf_path = converter.convert()
```

**URDF → USD 转换脚本**（Isaac Sim / Isaac Lab 用）：

```python
# scripts/urdf_to_usd.py — URDF → USD 转换（Isaac Lab 环境下）
# 注意：此脚本需在 Isaac Sim Python 环境中运行

def convert_urdf_to_usd(
    urdf_path: str,
    usd_output_path: str,
    fix_base: bool = True,
    merge_fixed: bool = True,
):
    """
    URDF → USD 转换（Isaac Lab 风格）
    需要在 Isaac Sim 的 Python 环境中执行
    """
    import omni.kit.commands
    from pxr import UsdPhysics, Usd

    status, import_config = omni.kit.commands.execute(
        "URDFCreateImportConfig")
    if not status:
        raise RuntimeError("Failed to create URDF import config")

    import_config.merge_fixed_joints = merge_fixed
    import_config.fix_base = fix_base
    import_config.make_default_prim = True
    import_config.create_physics_scene = True

    # Isaac Sim URDF importer 的命令 API 会随版本演进；当前官方路径是
    # URDFParseAndImportFile / URDFImportRobot，而不是旧的 _urdf.import_robot()。
    result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=usd_output_path,
    )[0]
    
    if result:
        print(f"USD saved to: {usd_output_path}")
        
        # 验证 USD
        stage = Usd.Stage.Open(usd_output_path)
        joint_count = 0
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                joint_count += 1
        print(f"  Joints in USD: {joint_count}")
    else:
        print("ERROR: URDF import failed")
    
    return result

# CI/CD 脚本：自动从 URDF 生成所有格式
# scripts/build_assets.sh:
# #!/bin/bash
# set -e
# echo "=== Building assets from URDF ==="
# python scripts/urdf_to_mjcf.py --urdf config/robot.urdf --out assets/mjcf/
# python scripts/urdf_to_usd.py --urdf config/robot.urdf --out assets/usd/
# echo "=== Validating ==="
# python scripts/validate_assets.py
# echo "=== All assets built successfully ==="
```

### Isaac Sim / Isaac Lab 转换

**转换方式**：通过 Omniverse URDF Importer 将 URDF 转为 USD（Universal Scene Description）。

```python
# Isaac Sim / Isaac Lab URDF 导入（简化示例）
import omni.kit.commands

_, urdf_config = omni.kit.commands.execute("URDFCreateImportConfig")
urdf_config.merge_fixed_joints = True     # 合并固定关节（减少 body 数量）
urdf_config.fix_base = True               # 固定基座

ok, robot_prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/path/to/robot.urdf",
    import_config=urdf_config,
    dest_path="/World/Robot",
)
```

**关键陷阱**：改 URDF 后需要重新 import。保持 URDF 为源文件、**USD 当 build artifact 不提交到 Git**。

### 转换工具对比

| 目标仿真器 | 转换工具 | 自动化程度 | 主要陷阱 |
|-----------|---------|-----------|---------|
| RViz | 无需转换 | 完全 | 无 |
| Gazebo Harmonic | 原生 URDF + `<gazebo>` | 需配置 | Classic API 不兼容 |
| MuJoCo | `from_xml_path` + `mj_saveLastXML` + XML 后处理 | 半自动 | DAE、actuator、关节顺序 |
| Isaac Sim/Lab | URDF Importer → USD | 半自动 | 需重新 import |
| 真机 | ros2_control plugin 切换 | 自动 | 参数需标定 |

### 反事实推理——如果混用多个源文件

如果你在 Gazebo 中用 URDF，在 MuJoCo 中手动编辑了 MJCF（添加了 actuator 和调整了摩擦），在 Isaac Sim 中手动编辑了 USD（改了材质），会发生什么？

三个月后，你发现仿真和真机的行为差异越来越大。你想排查是哪个参数有问题——但你面对的是三个不同格式、不同参数值的文件，没人记得谁改了什么。这就是"多源文件"的噩梦。Single Source of Truth 的价值正是**消除这种不一致**。

### ⚠️ 常见陷阱

```
💡 概念误区：认为"MuJoCo 和 Gazebo 的物理引擎是一样的"
   新手想法："同一个 URDF，两个仿真器应该给出相同结果"
   实际上：MuJoCo 和 Gazebo 用完全不同的接触求解器。MuJoCo 使用软约束接触模型，
   并将接触步求解成凸优化问题，常用 Newton / CG / PGS 等数值求解器；它不是严格
   LCP 互补性模型。Gazebo 侧则取决于 DART/Bullet/ODE 后端。同样的摩擦系数，
   两者的接触行为可能显著不同。
   正确做法：不要假设跨仿真器参数可以直接复用。
   为每个仿真器单独标定接触参数（摩擦、刚度、阻尼）。
```

### 练习

1. **[编程]** 把同一个 Panda URDF 分别加载到 MuJoCo 和 Gazebo。对比：(a) 关节名称和顺序是否一致？(b) 同样的关节指令序列，两个仿真器的轨迹差异有多大？量化误差。
2. **[编程]** 用 `mujoco.viewer` 可视化 MuJoCo 导入的 Panda。如果 visual mesh 缺失，编写后处理脚本将 URDF 的 visual mesh 以 `contype="0" conaffinity="0"` 添加到 MJCF。

---

## P02.3 域随机化——运行时 API，不改 URDF ⭐⭐

### 动机——为什么需要域随机化？

RL 策略在仿真中训练，但需要在真机上部署。仿真与真实之间的差异（sim-to-real gap）会导致策略在真机上失效。域随机化（Domain Randomization, DR）的核心思想是：**不追求仿真精确匹配真实，而是让策略对参数变化鲁棒**。

### 历史——从 Tobin 2017 到现代实践

Domain Randomization 的概念由 Tobin et al.（2017, "Domain Randomization for Transferring DNNs from Simulation to Real World"）首次系统提出，最初用于视觉 sim-to-real（随机化纹理、光照、背景），后被推广到物理参数（质量、摩擦、执行器延迟）。到 2024-2025 年，DR 已经是 RL sim-to-real 的标准配置——几乎每个成功的 sim-to-real RL 工作都使用了某种形式的 DR。

> **本质洞察**：域随机化**不是让仿真更接近真实**——它是让策略对参数变化**更鲁棒**。真实世界只是随机化覆盖的参数分布中的一个采样点。DR 的核心假设是：如果策略能在大量不同参数设置下都成功，那么真实世界的特定参数也在成功范围内。这是一个**充分但非必要**的条件——过窄的随机化可能不覆盖真实，过宽的随机化使训练困难。

### 反面——新手的错误做法

```
❌ 错误做法：批量生成带扰动的 URDF 文件
   新手写 Python 脚本：
   for mass_scale in [0.85, 0.90, 0.95, 1.0, 1.05, 1.10, 1.15]:
       modify_urdf_mass('robot.urdf', mass_scale)
       save_as(f'robot_{mass_scale}.urdf')
   
   四大问题：
   1. 生成 7^5 = 16807 个 URDF（5 个参数各 7 档）→ 磁盘空间浪费
   2. 破坏 single-source-of-truth 原则
   3. 训练时每个 iter 加载不同 URDF → 磁盘 IO 成为瓶颈，极慢
   4. 参数空间离散化 → 策略只在离散档位上鲁棒，档位之间无法泛化
```

**正确做法**：Isaac Lab、MuJoCo、PyBullet 都提供**运行时 randomization API**——在仿真重置时随机采样参数并应用到内存中的模型，不修改文件。

### 域随机化的数学框架

域随机化可以从**分布鲁棒优化**（Distributionally Robust Optimization, DRO）的视角理解。设仿真参数为 $\xi$（包含质量、摩擦、延迟等），策略为 $\pi_\theta$，则：

**标准 RL 的目标**：在固定参数 $\xi_0$ 下最大化期望回报

$$\max_\theta \mathbb{E}_{\tau \sim p(\tau|\pi_\theta, \xi_0)} \left[ \sum_t r(s_t, a_t) \right]$$

**域随机化 RL 的目标**：在参数分布 $\Xi$ 上最大化期望回报

$$\max_\theta \mathbb{E}_{\xi \sim \Xi} \left[ \mathbb{E}_{\tau \sim p(\tau|\pi_\theta, \xi)} \left[ \sum_t r(s_t, a_t) \right] \right]$$

**DRO 视角**（更严格的鲁棒性保证）：

$$\max_\theta \min_{\xi \in \Xi} \mathbb{E}_{\tau \sim p(\tau|\pi_\theta, \xi)} \left[ \sum_t r(s_t, a_t) \right]$$

DR 实际是上述目标的蒙特卡洛近似——每个 episode 随机采样 $\xi$，隐式地在参数分布上做平均。DRO 版本要求最坏情况鲁棒，计算上更昂贵但理论保证更强。

**随机化分布的选择**是关键的工程决策：

| 分布类型 | 适用参数 | 优点 | 缺点 |
|---------|---------|------|------|
| 均匀分布 $U[a, b]$ | 质量、摩擦系数 | 简单、无偏 | 边界值训练频率与中间值相同 |
| 对数均匀 $\exp(U[\log a, \log b])$ | PD 增益、延迟 | 覆盖多数量级 | 大值稀疏采样 |
| 高斯 $\mathcal{N}(\mu, \sigma^2)$ | 传感器噪声 | 符合真实噪声分布 | 可能采到不合理值 |
| 截断高斯 | 关节阻尼 | 避免极端值 | 需设置截断边界 |

> **反事实推理**：如果随机化范围设得太窄（如质量只在 $\pm 5\%$ 内变化）会怎样？策略只对 5% 的参数变化鲁棒。真机的质量误差可能达 $\pm 15\%$（加了夹爪、负载变化），策略直接失效。反之，如果设得太宽（如质量在 $\pm 50\%$），训练变得极其困难——策略需要在差异巨大的环境中都表现良好，收敛缓慢甚至无法收敛。这就是为什么课程式随机化（Curriculum DR）很重要——从窄范围开始，逐步扩大。

### Isaac Lab 域随机化配置

```python
# Isaac Lab 域随机化——运行时 API
from isaaclab.envs.mdp.events import (
    randomize_rigid_body_mass,
    randomize_rigid_body_material,
    randomize_actuator_gains,
)
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

@configclass
class EventCfg:
    """域随机化配置"""
    
    # 质量随机化：±15%（每次 episode 重置时采样）
    randomize_mass = EventTerm(
        func=randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mass_distribution_params": (0.85, 1.15),
            "operation": "scale",
        },
    )
    
    # 摩擦系数随机化：0.1-2.0
    randomize_friction = EventTerm(
        func=randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "static_friction_range": (0.1, 2.0),
            "dynamic_friction_range": (0.1, 2.0),
            "restitution_range": (0.0, 0.2),
            "num_buckets": 64,
        },
    )
    
    # PD 增益随机化：±30%
    randomize_gains = EventTerm(
        func=randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "stiffness_distribution_params": (0.7, 1.3),
            "damping_distribution_params": (0.7, 1.3),
            "operation": "scale",
        },
    )
```

### 标准 Randomization 套餐

以下是 RL locomotion 和机械臂操作的标准域随机化配置（来自 Isaac Lab / legged_gym 社区约定）：

| 参数 | 范围 | 分布 | 重要性 | mode |
|------|------|------|--------|------|
| Base mass | $\pm$15% | Uniform | 高 | reset |
| Link mass | $\pm$10% | Uniform | 中 | reset |
| Friction coefficients | 0.1-2.0 | Uniform | 高 | reset |
| PD gains (Kp/Kd) | $\pm$30% | Uniform | 高 | reset |
| Motor strength | $\pm$20% | Uniform | 中 | reset |
| **Action delay** | **10-30 ms** | **Uniform** | **极高** | **reset** |
| Joint position noise | 0.01-0.05 rad | Gaussian | 中 | step |
| Joint velocity noise | 0.05-0.2 rad/s | Gaussian | 中 | step |

**为什么 action delay 是最关键的？**

真机的控制链存在不可忽视的延迟：传感器采集（~1 ms）→ 通信（~1-5 ms）→ 策略推理（~5-20 ms）→ 通信（~1-5 ms）→ 执行器响应（~5-10 ms）。总延迟约 15-40 ms。如果训练时假设零延迟（仿真的默认状态），策略学到的是"立即反应"的行为。部署到真机时，15-40 ms 的延迟使策略的反应"滞后"——对于快速动态任务（如接触/碰撞回避），这个滞后足以导致完全失败。

**跨领域类比——音乐演奏延迟**：音乐家对延迟极其敏感——耳机中 20ms 的音频延迟就足以打乱节奏感。RL 策略也是如此：它在"零延迟"环境中学到了精确的时序关系，20ms 的部署延迟就足以打乱学到的控制节奏。

### 课程式 Randomization

直接从最大范围开始训练往往导致学习困难。**课程式随机化（Curriculum DR）**从小范围开始，随训练进展逐步扩大：

```python
class CurriculumDR:
    def __init__(self):
        self.level = 0  # 0=clean, 1=mild, 2=moderate, 3=full
        
    def get_ranges(self):
        levels = {
            0: {"mass": (0.98, 1.02), "friction": (0.7, 1.3), "delay": (0, 0.005)},
            1: {"mass": (0.93, 1.07), "friction": (0.4, 1.6), "delay": (0.005, 0.015)},
            2: {"mass": (0.88, 1.12), "friction": (0.2, 1.8), "delay": (0.010, 0.025)},
            3: {"mass": (0.85, 1.15), "friction": (0.1, 2.0), "delay": (0.010, 0.030)},
        }
        return levels[self.level]
    
    def maybe_advance(self, success_rate):
        """当成功率 > 80% 时提升 DR 等级"""
        if success_rate > 0.8 and self.level < 3:
            self.level += 1
            return True
        return False
```

### Isaac Sim 域随机化 YAML 配置

Isaac Lab 官方主线是 Python config：用 `EventTermCfg` / `SceneEntityCfg` 把 `isaaclab.envs.mdp.events` 中的事件函数挂到 reset、step 等 mode 上。下面的 YAML 是**项目自定义 loader 的声明式外壳**，不是 Isaac Lab 官方主 schema；工程里可以把它解析成上一节的 Python `EventCfg`，但不要假设 Isaac Lab 会原生读取这些字段名。

```yaml
# config/domain_randomization.yaml
# 项目自定义 loader 的域随机化配置模板

physics_randomization:
  # ======== 质量 ========
  body_mass:
    mode: reset          # episode 开始时随机化
    distribution: uniform
    operation: scale     # 相对缩放
    range: [0.85, 1.15]  # ±15%
    body_names: ["*"]    # 所有 body
    
  # ======== 摩擦 ========
  surface_friction:
    mode: reset
    distribution: uniform
    static_friction_range: [0.3, 1.5]
    dynamic_friction_range: [0.2, 1.2]
    restitution_range: [0.0, 0.3]
    geom_names: ["*"]
    
  # ======== 执行器 ========
  actuator_gains:
    mode: reset
    distribution: uniform
    kp_range: [0.7, 1.3]   # Kp ±30%
    kd_range: [0.7, 1.3]   # Kd ±30%
    operation: scale
    
  # ======== 动作延迟 ========
  action_delay:
    mode: reset
    distribution: uniform
    delay_range: [0.010, 0.030]  # 10-30 ms
    unit: seconds

observation_noise:
  # ======== 关节位置噪声 ========
  joint_position:
    mode: step           # 每步添加噪声
    distribution: gaussian
    mean: 0.0
    std: 0.02            # 0.02 rad ≈ 1.1 度
    
  # ======== 关节速度噪声 ========
  joint_velocity:
    mode: step
    distribution: gaussian
    mean: 0.0
    std: 0.1             # 0.1 rad/s
    
  # ======== IMU 噪声 ========
  imu_angular_velocity:
    mode: step
    distribution: gaussian
    mean: 0.0
    std: 0.05            # rad/s
  
  imu_linear_acceleration:
    mode: step
    distribution: gaussian
    mean: 0.0
    std: 0.2             # m/s^2
```

### 视觉域随机化配置

对于使用图像输入的策略（ACT、Diffusion Policy），视觉域随机化至关重要：

```python
# config/visual_domain_randomization.py
# Isaac Sim 视觉域随机化配置

class VisualDRConfig:
    """视觉域随机化配置"""
    
    # ======== 纹理随机化 ========
    texture_randomization = {
        'enabled': True,
        'targets': ['table', 'wall', 'floor'],  # 随机化哪些物体的纹理
        'methods': {
            'random_color': {
                'hue_range': [0.0, 1.0],
                'saturation_range': [0.3, 1.0],
                'value_range': [0.3, 1.0],
            },
            'random_texture': {
                'texture_dir': 'assets/textures/dtd/',  # Describable Textures Dataset
                'uv_scale_range': [0.5, 2.0],
            },
        },
        'mode': 'reset',  # 每 episode 随机化一次
    }
    
    # ======== 光照随机化 ========
    lighting_randomization = {
        'enabled': True,
        'ambient_intensity_range': [0.1, 0.6],  # 环境光强度
        'directional_lights': {
            'count_range': [1, 3],               # 1-3 个方向光
            'intensity_range': [500, 3000],      # 流明
            'color_temperature_range': [3000, 7000],  # K
            'position_noise_m': 0.5,             # 位置扰动
        },
        'shadows': True,
        'mode': 'reset',
    }
    
    # ======== 相机随机化 ========
    camera_randomization = {
        'enabled': True,
        'extrinsic': {
            'position_noise_m': 0.01,    # ±1cm
            'orientation_noise_deg': 2.0, # ±2度
        },
        'intrinsic': {
            'fov_noise_percent': 5.0,     # FOV ±5%
            'distortion_noise': 0.1,      # 畸变系数扰动
        },
        'mode': 'reset',
    }
    
    # ======== 物体外观随机化 ========
    object_appearance = {
        'enabled': True,
        'color_noise': {
            'rgb_std': 0.1,               # RGB 各通道 ±10%
        },
        'scale_noise': {
            'range': [0.9, 1.1],          # 尺寸 ±10%
        },
        'mode': 'reset',
    }
```

**视觉 DR 的三层递进**：

| 层级 | 随机化内容 | 目标 | 训练时间影响 |
|------|----------|------|------------|
| L1: 基础 | 颜色抖动、亮度变化 | 光照鲁棒 | +10% |
| L2: 中级 | 纹理替换、光源数量/位置 | 背景鲁棒 | +30% |
| L3: 完整 | 相机位姿、物体外观、distractor 物体 | 全面鲁棒 | +60% |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：在 step 而非 reset 时随机化物理参数
   错误做法：每一步都重新采样质量和摩擦
   现象：策略无法学到任何稳定行为——环境每步都在变
   根本原因：物理参数应在 episode 开始时固定，让策略在一组固定参数下
   学习完整行为；噪声参数（如传感器噪声）才在每步添加
   正确做法：物理参数 mode="reset"，观测噪声 mode="step"

💡 概念误区：认为"随机化范围越大越好"
   新手想法："mass ±50% 比 ±15% 更鲁棒"
   实际上：过大的随机化使训练极度困难——策略需要在完全不同的动力学下
   都表现良好，对神经网络容量要求极高。而且有些参数组合物理上不合理
   （如质量增加 50% 但电机力矩不变 → 根本无法完成任务）。
   正确做法：随机化范围 = 真实世界不确定性估计 × 安全系数(1.5-2.0)

🧠 思维陷阱：认为"域随机化可以替代精确建模"
   新手想法："反正都要随机化，URDF 参数大概对就行"
   实际上：域随机化是在精确建模基础上的补充，不是替代。
   如果基础模型错误过大（如质量差 3 倍），再怎么随机化也覆盖不到真实。
   正确做法：先做 System Identification 精确标定基准，
   然后在精确值附近做适度域随机化。SysId 在 P02.4 详述。
```

### 练习

1. **[编程]** 在 Isaac Lab 或 MuJoCo 中为机械臂配置 5 项域随机化（mass、friction、damping、action delay、observation noise）。运行短训练（100K steps），对比有无 DR 的 reward 曲线差异。
2. **[思考题]** 为什么 action delay 是 domain randomization 中最关键的项之一？如果训练时没有 action delay，策略迁移到真机的典型失败模式是什么？用具体的控制理论语言描述（提示：相位裕度）。

---

## P02.4 物理参数辨识（System Identification）⭐⭐⭐

### 动机——精确的基础模型

域随机化需要一个"基准"——围绕基准做随机化。如果基准本身偏差很大，随机化再怎么扩大也可能不覆盖真实参数。**物理参数辨识（System Identification, SysId）**就是通过真机实验数据反向估计精确的物理参数。

### SysId 的数学框架

机械臂的逆动力学方程 $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$ 对动力学参数是**线性的**——这是 SysId 的关键数学性质。每个 link $i$ 有 10 个标准惯性参数：

$$\phi_i = [m_i, \; m_i c_{x,i}, \; m_i c_{y,i}, \; m_i c_{z,i}, \; I_{xx,i}, \; I_{xy,i}, \; I_{xz,i}, \; I_{yy,i}, \; I_{yz,i}, \; I_{zz,i}]^T$$

其中 $m_i$ 是质量，$c_{x,i}, c_{y,i}, c_{z,i}$ 是质心位置（注意使用 $m_i c_x$ 而非 $c_x$ 本身，以保证线性性），$I_{xx,i}$ 等是惯性张量分量。对 $n$ 自由度机械臂，总参数向量 $\phi \in \mathbb{R}^{10n}$。

逆动力学方程可重写为**回归形式**：

$$\tau = Y(q, \dot{q}, \ddot{q}) \cdot \phi$$

其中 $Y(q, \dot{q}, \ddot{q}) \in \mathbb{R}^{n \times 10n}$ 称为**回归矩阵**（regressor matrix），仅依赖于关节运动学量（位置、速度、加速度），不依赖惯性参数。这个线性性意味着：给定 $K$ 组关节轨迹数据 $(q_k, \dot{q}_k, \ddot{q}_k, \tau_k)$，可以堆叠成**超定线性系统**：

$$\begin{bmatrix} \tau_1 \\ \tau_2 \\ \vdots \\ \tau_K \end{bmatrix} = \begin{bmatrix} Y_1 \\ Y_2 \\ \vdots \\ Y_K \end{bmatrix} \phi \quad \Rightarrow \quad W = Y_{\text{stack}} \cdot \phi$$

用最小二乘求解：$\hat{\phi} = (Y_{\text{stack}}^T Y_{\text{stack}})^{-1} Y_{\text{stack}}^T W$

**激励轨迹设计**是 SysId 成功的前提。并非所有轨迹都能辨识出所有参数——回归矩阵 $Y_{\text{stack}}$ 必须有足够的数值秩。Fourier 基激励轨迹（由多个正弦分量叠加而成）通过优化频率和振幅来最大化 $Y_{\text{stack}}$ 的条件数，这是 Swevers et al. (1997) 提出的经典方法。

> **跨领域类比**：SysId 的回归方法本质上与 SLAM 中的 Bundle Adjustment 相同——都是从观测数据反推模型参数。BA 从像素观测反推 3D 点和相机位姿，SysId 从力矩观测反推惯性参数。两者都使用最小二乘框架，都需要"激励"（BA 需要足够的视角变化，SysId 需要足够丰富的运动模式），都面临可观性问题（BA 中的 gauge freedom，SysId 中的不可辨识参数组合）。

### SysId 的核心流程

```
┌─────────────────┐     ┌─────────────────┐
│ 真机实验         │     │ 仿真 replay     │
│ 施加已知激励    │     │ 同样的激励      │
│ 记录:           │     │ 记录:           │
│  - 关节位置 q   │     │  - 仿真关节位置  │
│  - 关节速度 qd  │     │  - 仿真关节速度  │
│  - 关节力矩 tau │     │  - 仿真关节力矩  │
└────────┬────────┘     └────────┬────────┘
         │                       │
         └───────────┬───────────┘
                     ▼
         ┌───────────────────────────┐
         │ 参数优化                   │
         │ min_θ Σ ||q_real - q_sim(θ)||² │
         │ θ = {mass, inertia,       │
         │      friction, damping}    │
         └───────────────────────────┘
```

### 各参数的辨识方法

| 参数 | 辨识方法 | 典型精度 | 实验要求 |
|------|---------|---------|---------|
| 质量/惯量 | Swevers 法（激励轨迹 + LS） | $\pm$5% | 特殊设计的周期性激励 |
| 关节摩擦 | 低速正反转 → 记录力矩差 | $\pm$10% | 库仑 + 粘滞摩擦模型 |
| 关节阻尼 | 自由衰减振荡 → 拟合包络 | $\pm$15% | 释放关节后记录振荡 |
| 接触刚度 | 已知物体碰撞 + 力传感器 | $\pm$20% | 需要 F/T 传感器 |
| 执行器延迟 | 阶跃响应时间测量 | $\pm$2 ms | 最容易测量 |

### sim-to-real 调试的三大观察指标

**按优先级排序**：

1. **关节位置跟踪误差**：开环回放同样的力矩序列到仿真和真机，比较关节位置偏差。偏差 > 0.05 rad → actuator 模型参数（增益/摩擦）有误
2. **基座轨迹漂移**（适用于移动机器人）：同样的目标速度，真机 5 秒内漂移量 → 整体动力学参数（质量/摩擦）有误
3. **接触力对比**：力传感器读数 vs 仿真预测 → 接触模型（摩擦系数/刚度/阻尼）有误

### 物理参数辨识——最小二乘法实现

经典的 Swevers 法基于**线性回归**：机器人动力学方程可以写成关于惯性参数的线性形式：

$$\tau = Y(q, \dot{q}, \ddot{q}) \cdot \pi$$

其中 $Y$ 是回归矩阵（由运动学量计算），$\pi$ 是待辨识的惯性参数向量（质量、质心、惯性张量组合）。

```python
# sysid/least_squares_sysid.py — 最小二乘物理参数辨识
import numpy as np
from scipy.optimize import least_squares
import pinocchio as pin

class LeastSquaresSysId:
    """最小二乘法物理参数辨识"""
    
    def __init__(self, model_path: str):
        self.model = pin.buildModelFromUrdf(model_path)
        self.data = self.model.createData()
        self.nq = self.model.nq
    
    def collect_data(self, q_traj, dq_traj, ddq_traj, tau_traj):
        """
        收集辨识数据
        q_traj: (N, nq)  关节位置
        dq_traj: (N, nq) 关节速度
        ddq_traj: (N, nq) 关节加速度
        tau_traj: (N, nq) 关节力矩（真机测量）
        """
        self.q_data = q_traj
        self.dq_data = dq_traj
        self.ddq_data = ddq_traj
        self.tau_data = tau_traj
        self.N = len(q_traj)
    
    def compute_regressor(self):
        """计算完整回归矩阵 Y"""
        Y_full = []
        tau_full = []
        
        for i in range(self.N):
            q = self.q_data[i]
            dq = self.dq_data[i]
            ddq = self.ddq_data[i]
            
            # Pinocchio 计算回归矩阵
            Y_i = pin.computeJointTorqueRegressor(
                self.model, self.data, q, dq, ddq)
            Y_full.append(Y_i)
            tau_full.append(self.tau_data[i])
        
        self.Y = np.vstack(Y_full)      # (N*nq, n_params)
        self.tau = np.concatenate(tau_full)  # (N*nq,)
        
        print(f"Regressor: Y shape={self.Y.shape}, "
              f"tau shape={self.tau.shape}")
        return self.Y, self.tau
    
    def solve(self, regularization=1e-4):
        """求解: pi* = (Y^T Y + lambda I)^{-1} Y^T tau"""
        YtY = self.Y.T @ self.Y
        Ytt = self.Y.T @ self.tau
        
        # Tikhonov 正则化（防止过拟合 + 数值稳定）
        n_params = YtY.shape[0]
        pi_star = np.linalg.solve(
            YtY + regularization * np.eye(n_params), Ytt)
        
        # 计算拟合残差
        tau_pred = self.Y @ pi_star
        residual = np.sqrt(np.mean((self.tau - tau_pred)**2))
        print(f"SysId residual: {residual:.4f} Nm (RMS)")
        
        return pi_star, residual
```

### 物理参数辨识——黑盒参数优化 / L-BFGS-B SysID

当参数只能通过"改 MuJoCo 模型 → 回放控制 → 比较真机轨迹"这种黑盒仿真闭环评估时，可以先用 SciPy 的 L-BFGS-B 做有边界参数优化。下面代码不是 surrogate/acquisition 式 BO；若要走 BO 路线，需要显式引入高斯过程或随机森林 surrogate，以及 EI/UCB 等 acquisition function。

```python
# sysid/lbfgsb_sysid.py — L-BFGS-B 物理参数辨识
import numpy as np
from scipy.optimize import minimize
import mujoco

class LBFGSBSystemID:
    """有边界黑盒参数优化（用仿真-真实轨迹误差做目标函数）"""
    
    def __init__(self, mjcf_path: str, real_data: dict):
        """
        mjcf_path: MuJoCo 模型路径
        real_data: {
            'ctrl': (T, nu) 控制输入（真机施加的力矩序列）
            'qpos': (T, nq) 真机关节位置响应
        }
        """
        self.base_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.real_ctrl = real_data['ctrl']
        self.real_qpos = real_data['qpos']
        self.T = len(self.real_ctrl)
    
    def objective(self, params):
        """
        目标函数：仿真轨迹与真机轨迹的差异
        params: 待优化参数 [mass_scale_1, ..., friction_1, ...]
        """
        model = self._apply_params(params)
        data = mujoco.MjData(model)
        
        # 初始化仿真。这里假设 real_qpos 已由采集/预处理脚本转换为 MuJoCo qpos 地址顺序；
        # 如果来源是 ROS JointState，必须先按 name -> jnt_qposadr 建表后再写入。
        data.qpos[:model.nq] = self.real_qpos[0]
        
        # 回放控制输入
        sim_qpos = []
        for t in range(self.T):
            data.ctrl[:] = self.real_ctrl[t]
            mujoco.mj_step(model, data)
            sim_qpos.append(data.qpos[:model.nq].copy())
        
        sim_qpos = np.array(sim_qpos)
        
        # 计算误差
        error = np.mean((sim_qpos - self.real_qpos)**2)
        return error
    
    def optimize(self, n_iterations=100):
        """运行 L-BFGS-B 有边界优化"""
        # 参数边界：质量 ±30%，摩擦 0.01-2.0
        n_bodies = self.base_model.nbody
        bounds_mass = [(0.7, 1.3)] * n_bodies
        bounds_friction = [(0.01, 2.0)] * 3  # 滑动/滚动/自旋
        bounds = bounds_mass + bounds_friction
        
        # 初始点
        x0 = [1.0] * n_bodies + [0.5, 0.01, 0.01]
        
        result = minimize(
            self.objective, x0, method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': n_iterations, 'disp': True})
        
        print(f"Optimal params: {result.x}")
        print(f"Final error: {result.fun:.6f}")
        return result
    
    def _apply_params(self, params):
        """将参数应用到 MuJoCo 模型（内存中修改，不改文件）"""
        # MuJoCo >= 3.0 的 MjModel 支持 copy.copy()，返回独立的模型副本。
        # 注意：这是 shallow copy，但 MjModel 内部数组是独立分配的，
        # 因此修改副本不会影响原模型。旧版（< 3.0）不支持此操作，
        # 需改用 mujoco.MjModel.from_xml_string(xml_string) 重新构建。
        model = copy.copy(self.base_model)
        n_bodies = model.nbody
        
        # 质量缩放
        for i in range(n_bodies):
            model.body_mass[i] *= params[i]
        
        # 摩擦系数
        for i in range(model.ngeom):
            model.geom_friction[i, 0] = params[n_bodies]      # slide
            model.geom_friction[i, 1] = params[n_bodies + 1]  # torsional
            model.geom_friction[i, 2] = params[n_bodies + 2]  # rolling
        
        return model
```

### Action 滤波——真机部署的标准做法

策略的神经网络输出往往在相邻帧之间不连续（高频抖动）。直接发送到真机会导致电机噪音和机械磨损。EMA（指数移动平均）滤波是标准做法：

$$a_{\text{filtered}}(t) = \alpha \cdot a_{\text{raw}}(t) + (1 - \alpha) \cdot a_{\text{filtered}}(t-1)$$

典型 $\alpha = 0.2 \text{-} 0.5$。$\alpha$ 越小越平滑但延迟越大。

```python
class ActionFilter:
    """EMA 动作滤波器"""
    def __init__(self, alpha=0.3, dim=7):
        self.alpha = alpha
        self.prev = np.zeros(dim)
        self.initialized = False
        
    def filter(self, raw_action):
        if not self.initialized:
            self.prev = raw_action.copy()
            self.initialized = True
            return raw_action
        filtered = self.alpha * raw_action + (1 - self.alpha) * self.prev
        self.prev = filtered.copy()
        return filtered
    
    def reset(self):
        self.initialized = False
```

### ⚠️ 常见陷阱

```
💡 概念误区：认为"仿真参数调到完美就不需要域随机化"
   新手想法："SysId 标定到误差 < 1% 就是完美仿真了"
   实际上：即使标定非常精确，真实世界仍有不可建模的时变因素——
   温度变化导致摩擦漂移、物体表面磨损、电机老化、负载变化。
   SysId 给出的是"今天在这个温度下的参数"，不是永恒常数。
   正确做法：SysId 标定精确基准 + 在基准周围做适度域随机化。

⚠️ 编程陷阱：action filter 的 alpha 在训练和部署时不一致
   错误做法：训练时不加 filter，部署时加 filter(alpha=0.2)
   现象：策略训练时学到了高频动作（因为仿真可以精确执行），
        部署时 filter 把高频滤掉 → 行为变化
   正确做法：训练时也加相同的 filter（作为 wrapper）
```

### 练习

1. **[编程]** 对 MuJoCo 中的 Franka Panda 做简易 SysId：在一个 MuJoCo 实例（作为"真机"）中用特定增益运行轨迹，在另一个 MuJoCo 实例（作为"仿真"）中用默认参数 replay 同样力矩，比较轨迹误差。调整质量和摩擦使误差最小化。
2. **[思考题]** Action filter 的 $\alpha$ 如何选择？如果策略训练时已做了 action delay 随机化（10-30ms），部署时 filter 引入的额外延迟是否还在训练覆盖的范围内？定量分析。

---

## P02.5 sim-to-real gap 的系统性分析 ⭐⭐⭐

### 四维分类框架

sim-to-real gap 不是一个单一问题，而是**四个独立维度的差异叠加**。穷举式分类：

| 维度 | 具体来源 | 典型症状 | 弥补手段 |
|------|---------|---------|---------|
| **动力学** | 质量分布/惯性张量/关节摩擦/接触模型/变形 | 轨迹偏差、力不匹配 | SysId + DR |
| **物理参数** | 重力精度/弹性系数/阻尼系数/空气阻力 | 长时间漂移 | 精确标定 |
| **感知** | 传感器噪声/视觉差异/延迟/数据丢失/标定误差 | 状态估计错误 | 噪声 DR + 视觉 DR |
| **执行** | 电机延迟/量化误差/饱和/通信带宽/齿轮间隙 | 动作不跟踪 | Delay DR + filter |

**视觉域随机化**（针对使用图像的策略）：

| 参数 | 范围 | 方法 |
|------|------|------|
| 纹理 | 随机纹理/颜色 | Isaac Sim 材质替换 |
| 光照 | 位置/强度/颜色随机 | 随机点光源 + 环境光 |
| 相机内参 | FOV $\pm$5%, 畸变 $\pm$10% | 运行时修改相机参数 |
| 相机外参 | 位姿 $\pm$1cm / $\pm$2度 | 随机扰动相机 transform |
| 背景 | 随机图片/颜色/纹理 | 绿幕替换或随机化 |

### 仿真保真度评估方法

在做 sim-to-real 之前，应该定量评估仿真的保真度：

```python
def evaluate_sim_fidelity(sim_trajectory, real_trajectory):
    """
    定量评估仿真保真度
    sim_trajectory, real_trajectory: (T, n_joints) 关节位置时间序列
    """
    import numpy as np
    
    # 指标 1: 平均关节位置误差 (rad)
    mean_error = np.mean(np.abs(sim_trajectory - real_trajectory))
    
    # 指标 2: 相关系数（运动趋势一致性）
    correlations = []
    for j in range(sim_trajectory.shape[1]):
        corr = np.corrcoef(sim_trajectory[:, j], real_trajectory[:, j])[0, 1]
        correlations.append(corr)
    mean_corr = np.mean(correlations)
    
    # 指标 3: 最大瞬时误差
    max_error = np.max(np.abs(sim_trajectory - real_trajectory))
    
    return {
        'mean_pos_error_rad': mean_error,
        'max_pos_error_rad': max_error,
        'mean_correlation': mean_corr,
    }
```

**保真度评级标准**：

| 指标 | 高保真 | 中保真 | 低保真 |
|------|--------|--------|--------|
| 平均关节误差 | < 0.02 rad | 0.02-0.1 rad | > 0.1 rad |
| 相关系数 | > 0.98 | 0.90-0.98 | < 0.90 |
| 所需 DR 强度 | 轻度 | 中度 | 强烈（或需重新标定） |

### 仿真保真度评估——Reality Gap Metric（RGM）

保真度评估不应只看关节位置误差——需要一个多维度的综合指标：

```python
# sysid/reality_gap_metric.py — 多维度仿真保真度量化

import numpy as np
from dataclasses import dataclass

@dataclass
class RealityGapReport:
    """Reality Gap 分析报告"""
    position_rmse: float       # 关节位置 RMSE (rad)
    velocity_rmse: float       # 关节速度 RMSE (rad/s)
    torque_rmse: float         # 关节力矩 RMSE (Nm)
    correlation_mean: float    # 平均相关系数
    spectral_divergence: float # 频谱差异
    contact_f1: float          # 接触事件 F1 分数
    overall_gap_score: float   # 综合评分 (0=完美, 1=极差)
    recommendation: str        # 行动建议

def compute_reality_gap(
    sim_data: dict, real_data: dict, dt: float = 0.001
) -> RealityGapReport:
    """
    定量分析 sim-to-real gap
    
    sim_data / real_data: {
        'qpos': (T, nq), 'qvel': (T, nv), 'tau': (T, nv),
        'contacts': (T,) bool  # 是否有接触
    }
    """
    # 1. 关节位置 RMSE
    pos_err = sim_data['qpos'] - real_data['qpos']
    pos_rmse = np.sqrt(np.mean(pos_err**2))
    
    # 2. 关节速度 RMSE
    vel_err = sim_data['qvel'] - real_data['qvel']
    vel_rmse = np.sqrt(np.mean(vel_err**2))
    
    # 3. 力矩 RMSE
    tau_err = sim_data['tau'] - real_data['tau']
    tau_rmse = np.sqrt(np.mean(tau_err**2))
    
    # 4. 时序相关性（运动趋势是否一致）
    nq = sim_data['qpos'].shape[1]
    correlations = []
    for j in range(nq):
        corr = np.corrcoef(
            sim_data['qpos'][:, j], real_data['qpos'][:, j])[0, 1]
        correlations.append(corr)
    corr_mean = np.mean(correlations)
    
    # 5. 频谱差异（检测动态特性匹配）
    spectral_divs = []
    for j in range(min(nq, 7)):  # 前 7 个关节
        sim_fft = np.abs(np.fft.rfft(sim_data['qpos'][:, j]))
        real_fft = np.abs(np.fft.rfft(real_data['qpos'][:, j]))
        # KL 散度近似
        sim_fft = sim_fft / (sim_fft.sum() + 1e-10)
        real_fft = real_fft / (real_fft.sum() + 1e-10)
        kl = np.sum(real_fft * np.log(
            (real_fft + 1e-10) / (sim_fft + 1e-10)))
        spectral_divs.append(kl)
    spectral_div = np.mean(spectral_divs)
    
    # 6. 接触事件 F1（如果有接触数据）
    contact_f1 = 0.0
    if 'contacts' in sim_data and 'contacts' in real_data:
        sim_c = sim_data['contacts'].astype(bool)
        real_c = real_data['contacts'].astype(bool)
        tp = np.sum(sim_c & real_c)
        fp = np.sum(sim_c & ~real_c)
        fn = np.sum(~sim_c & real_c)
        precision = tp / (tp + fp + 1e-10)
        recall = tp / (tp + fn + 1e-10)
        contact_f1 = 2 * precision * recall / (precision + recall + 1e-10)
    
    # 综合评分（加权）
    gap_score = (
        0.3 * min(pos_rmse / 0.1, 1.0) +     # 位置权重最高
        0.2 * min(vel_rmse / 1.0, 1.0) +
        0.15 * min(tau_rmse / 5.0, 1.0) +
        0.15 * (1 - corr_mean) +
        0.1 * min(spectral_div / 1.0, 1.0) +
        0.1 * (1 - contact_f1)
    )
    
    # 行动建议
    if gap_score < 0.15:
        rec = "低 gap——可直接迁移，轻度 DR 即可"
    elif gap_score < 0.35:
        rec = "中 gap——需要 SysId 标定 + 中度 DR"
    elif gap_score < 0.6:
        rec = "高 gap——需要重新标定物理参数，检查接触模型"
    else:
        rec = "极高 gap——模型可能有根本性错误，重新检查 URDF"
    
    return RealityGapReport(
        position_rmse=pos_rmse,
        velocity_rmse=vel_rmse,
        torque_rmse=tau_rmse,
        correlation_mean=corr_mean,
        spectral_divergence=spectral_div,
        contact_f1=contact_f1,
        overall_gap_score=gap_score,
        recommendation=rec,
    )
```

### sim-to-real gap 定量分析方法

**系统性分析四步法**：

```
Step 1: 开环回放测试
  → 真机执行一段力矩序列，记录 qpos/qvel
  → 仿真回放同样力矩序列
  → 计算 RGM
  → gap_score > 0.35 → 进入 Step 2

Step 2: 按维度分解
  → 逐关节计算误差，找到贡献最大的关节
  → 对该关节做单独的 SysId（摩擦/惯量/增益）
  → 更新参数 → 重新测 RGM
  → gap_score > 0.35 → 进入 Step 3

Step 3: 接触场景测试
  → 设计包含接触的测试轨迹
  → 对比接触时刻、力大小、后续运动
  → 调整接触参数（stiffness/damping/friction）
  → gap_score > 0.35 → 进入 Step 4

Step 4: 残差分析
  → 如果 Step 1-3 后 gap 仍高
  → 检查：执行器延迟、传感器偏差、通信丢帧
  → 这些无法通过参数调整解决——需要在 DR 中覆盖
```

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为"sim-to-real gap 只存在于 RL"
   新手想法："我不做 RL，只做 MoveIt2 规划，不需要考虑 sim-to-real"
   实际上：MoveIt2 的规划也依赖准确的机器人模型（碰撞检测需要精确的几何）。
   默认 TOTG/Ruckig 时间参数化主要依赖 joint_limits 中的速度、加速度、jerk 等限制，
   URDF 惯性偏差不会直接改变这类默认时间参数化结果。
   但惯性会显著影响动力学仿真、力控、基于力矩约束的时间参数化，以及最终 sim-to-real 跟踪表现。
   正确做法：几何、限位、惯性和执行器参数分别校准，按规划/仿真/力控/RL 的需求使用。
```

### 练习

1. **[编程]** 对 Franka Panda 做仿真保真度评估：在 MuJoCo 和 Gazebo 中 replay 同一段力矩序列，计算两个仿真器之间的保真度指标。
2. **[穷举分类]** 列出你的双臂操作项目（D10）中所有可能的 sim-to-real gap 来源，按四维框架分类。对每个来源评估严重程度（高/中/低）和对应弥补手段。

---

## P02.6 RL + ros2_control 混合架构——CRISP 模式 ⭐⭐

### 动机——策略推理和实时控制的矛盾

RL 策略推理需要 GPU（5-10 Hz），但机器人底层控制需要 1 kHz 实时响应。一个 Python 进程无法同时满足两者。解决方案是**解耦**：高频合规控制在 C++ 实时线程，低频策略在 GPU Python 进程。

### CRISP 架构（TUM, 2025）

TUM 的 CRISP 框架展示了 RL + 机械臂的最干净的混合架构：

```
┌──────────────────────┐           ┌──────────────────────┐
│ GPU Workstation      │   ROS 2   │ RT Linux Workstation │
│                      │ topic/srv │                      │
│  RL / VLA Policy     │◄─────────►│  ros2_control (1kHz) │
│  (5-10 Hz inference) │  network  │  Cartesian Impedance │
│  libtorch / ONNX     │           │  Joint Compliance    │
│                      │           │         ▼            │
└──────────────────────┘           │  Franka FR3 EtherCAT │
                                   └──────────────────────┘
```

**核心设计原则**：

| 原则 | 说明 | 好处 |
|------|------|------|
| 高频合规在 C++ | 1 kHz ros2_control，RT-safe，无堆分配 | 安全、响应快 |
| 低频决策在 GPU | 5-10 Hz 策略推理，libtorch/ONNX | 可用深度学习库 |
| 标准 ROS2 通信 | topic 解耦，无紧耦合 | 独立部署、易替换 |
| 合规保障安全 | 策略卡顿时底层仍在控制 | 安全冗余 |

**跨领域类比——操作系统内核/用户空间**：CRISP 的设计与 OS 的内核态/用户态分离完全同构。内核（1 kHz 合规控制）处理硬件中断和实时响应，保证系统不崩溃；用户空间（GPU 策略）处理高层决策，不需要实时保证但可以使用复杂库。两者通过系统调用（ROS2 topic）通信。如果用户空间程序崩溃（策略推理卡死），内核仍在运行——机器人不会失控。

### RL 部署的 C++ 推理栈

| 方案 | 工具 | 优势 | 劣势 | 代表项目 |
|------|------|------|------|---------|
| **LibTorch** | `torch.jit.trace` → .pt → C++ 加载 | PyTorch 原生、调试方便 | 体积大 (~300MB) | rl_sar |
| **ONNX Runtime** | export → .onnx → C++/Python 推理 | 跨平台、多后端 | 不支持所有算子 | LimX tron1 |
| **TensorRT** | .onnx → .engine → NVIDIA GPU 加速 | Jetson 极速推理 | 仅 NVIDIA、编译慢 | Unitree 部署 |

```cpp
// LibTorch C++ 策略推理（rl_sar 风格）
#include <torch/script.h>

class PolicyRunner {
public:
    PolicyRunner(const std::string& model_path) {
        model_ = torch::jit::load(model_path);
        model_.eval();
        // 预热：第一次推理较慢
        auto dummy = torch::zeros({1, obs_dim_});
        model_.forward({dummy});
    }
    
    std::vector<float> inference(const std::vector<float>& obs) {
        auto input = torch::from_blob(
            const_cast<float*>(obs.data()),
            {1, static_cast<long>(obs.size())},
            torch::kFloat32
        ).clone();
        
        auto output = model_.forward({input}).toTensor();
        
        std::vector<float> action(
            output.data_ptr<float>(),
            output.data_ptr<float>() + output.numel());
        return action;
    }
    
private:
    torch::jit::script::Module model_;
    int obs_dim_ = 48;  // 典型 observation 维度
};
```

### 项目精读清单

> Stars 数据截至 2026-01，实际数字随时间增长，仅供量级参考。

| 项目 | Stars（截至 2026-01） | 说明 |
|------|-------|------|
| fan-ziqi/rl_sar (~1.2k) | LibTorch + ONNX 双栈 | 最完整的 RL locomotion 部署框架 |
| unitreerobotics/unitree_rl_lab | ONNX → C++ | Unitree 官方部署 |
| CRISP (TUM, 2025) | ros2_control + RL | 解耦架构参考 |
| Isaac Lab (~3.5k) | GPU 并行训练 | 训练端参考 |
| robot_descriptions.py | 175+ 描述 | 资产管道中央枢纽 |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：在 RT 控制循环中调用 GPU 推理
   错误做法：在 ros2_control 的 update() 函数中直接调 libtorch
   现象：GPU 推理耗时 5-20ms，控制循环从 1 kHz 降到 50-200 Hz
   根本原因：GPU 推理不是 RT-safe——包含内存分配、CUDA 同步等操作
   正确做法：策略推理在独立线程/节点中运行，通过 topic 发送目标；
            控制循环只读取最新目标，用阻抗/PD 控制跟踪

🧠 思维陷阱：认为"策略频率越高越好"
   新手想法："10 Hz 不如 100 Hz，应该把策略做到 1 kHz"
   实际上：策略频率受限于推理速度和信息更新速度。
   (1) 勉强提高频率导致推理延迟不稳定 → 控制不平滑
   (2) GPU 利用率 100% → 其他任务被抢占
   (3) 大多数策略的决策带宽远低于 100 Hz——视觉信息只有 30 Hz
   正确做法：策略 5-10 Hz + 底层合规控制 1 kHz。底层在策略更新间平滑插值。
```

### 练习

1. **[编程]** 搭建最小化 CRISP 原型：Python 侧以 10 Hz 发布 target pose 到 ROS2 topic，C++ ros2_control 侧以 1 kHz 订阅并用 PD 控制跟踪。验证两侧解耦——Python 侧挂起 1 秒，C++ 侧仍在平滑控制（保持最后目标）。
2. **[代码精读]** 精读 `rl_sar` 主循环（`rl_real.cpp`）：理解 observation 组装（关节位置/速度/基座姿态/上一步动作）、policy inference 调用、action scale（乘以 action_scale 参数）、action filter（EMA）的完整实现。对比 `LimX tron1-rl-deploy`（ONNX Runtime 后端）的设计差异。
3. **[思考题]** 如果你的策略需要图像输入（如 ACT），图像预处理（resize + normalize）应该在 GPU 侧还是 CPU 侧做？延迟如何？提示：考虑 GPU 显存复制 vs CPU 图像处理的延迟权衡。

---

## P02.7 数字孪生与工业部署 ⭐⭐⭐⭐

### 数字孪生的三个层次

**数字孪生（Digital Twin）**不只是"仿真跑一个模型"——它是真机与仿真的**实时双向同步**：

| 层次 | 内容 | 延迟要求 | 用途 |
|------|------|---------|------|
| Level 1: 可视化镜像 | 真机 → RViz 显示 | ~50 ms | 远程监控 |
| Level 2: 物理镜像 | 真机 → 仿真同步 | ~100 ms | 碰撞预测、故障诊断 |
| Level 3: 预测孪生 | 仿真**超前**真机运行 | N/A | 预测性维护、在线优化 |

### 工业案例：从 Isaac Sim 到产线部署

一个典型的工业 sim-to-real 项目时间线：

| 阶段 | 工具 | 时间 | 输出 |
|------|------|------|------|
| 建模 | SolidWorks + URDF exporter | 1-2 周 | 参数化 URDF |
| 仿真验证 | Isaac Sim + MoveIt2 | 2-4 周 | 可达性/碰撞/节拍验证 |
| RL 训练 | Isaac Lab (GPU $\times$8) | 1-2 周 | 策略模型 (.onnx) |
| SysId 标定 | 真机实验 + 参数优化 | 1 周 | 标定后参数 |
| 部署 | ros2_control + ONNX Runtime | 1-2 周 | 产线运行 |
| 监控 | RViz + 数字孪生 Level 1 | 持续 | 实时状态监控 |
| **总计** | | **7-12 周** | 从零到产线 |

### 数字孪生架构——ROS2 Bridge + 状态同步

```python
# digital_twin/ros2_bridge.py — 真机→仿真状态同步
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import mujoco
import mujoco.viewer
import numpy as np
import time

class DigitalTwinBridge(Node):
    """
    Level 2 数字孪生：真机关节状态 → MuJoCo 仿真同步
    支持双向：仿真可反向预测并发出碰撞警告
    """
    
    def __init__(self, mjcf_path: str):
        super().__init__('digital_twin')
        
        # MuJoCo 孪生实例
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data)

        # JointState.name 与 MuJoCo 内部地址不保证同序，必须显式建表。
        self.joint_qposadr, self.joint_dofadr = self._build_joint_address_maps()
        
        # 订阅真机关节状态
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_callback, 10)
        
        # 发布孪生状态（用于碰撞预测）
        self.twin_pub = self.create_publisher(
            JointState, '/digital_twin/joint_states', 10)
        
        # 碰撞预警发布
        self.collision_pub = self.create_publisher(
            WrenchStamped, '/digital_twin/collision_warning', 10)
        
        # 同步状态
        self.latest_joint_state = None
        self.sync_latency_ms = []
        
        # 同步循环 (100 Hz)
        self.timer = self.create_timer(0.01, self.sync_step)
        
        self.get_logger().info('Digital Twin bridge started')

    def _build_joint_address_maps(self):
        """建立 ROS JointState.name -> MuJoCo qpos/qvel 地址映射。"""
        qposadr = {}
        dofadr = {}
        supported_types = {
            mujoco.mjtJoint.mjJNT_HINGE,
            mujoco.mjtJoint.mjJNT_SLIDE,
        }

        for jnt_id in range(self.model.njnt):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
            if not name or self.model.jnt_type[jnt_id] not in supported_types:
                continue
            qposadr[name] = int(self.model.jnt_qposadr[jnt_id])
            dofadr[name] = int(self.model.jnt_dofadr[jnt_id])

        return qposadr, dofadr
    
    def joint_callback(self, msg):
        """接收真机状态"""
        t_recv = time.monotonic()
        state = {}
        for i, name in enumerate(msg.name):
            if name not in self.joint_qposadr:
                continue
            if i >= len(msg.position):
                continue
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            state[name] = (msg.position[i], vel)
        self.latest_joint_state = state
        
        # 计算同步延迟
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        latency = (t_recv - stamp) * 1000  # ms
        self.sync_latency_ms.append(latency)
        if len(self.sync_latency_ms) % 100 == 0:
            avg = np.mean(self.sync_latency_ms[-100:])
            self.get_logger().info(f'Sync latency: {avg:.1f} ms')
    
    def sync_step(self):
        """同步仿真状态到真机状态"""
        if self.latest_joint_state is None:
            return

        # 按关节名写入 MuJoCo 地址。qpos 和 qvel 地址空间不同，不能用 nq 截断 qvel。
        for name, (pos, vel) in self.latest_joint_state.items():
            qadr = self.joint_qposadr.get(name)
            dadr = self.joint_dofadr.get(name)
            if qadr is None or dadr is None:
                continue
            self.data.qpos[qadr] = pos
            self.data.qvel[dadr] = vel
        
        # 前向动力学（计算接触力等）
        mujoco.mj_forward(self.model, self.data)
        
        # 检查碰撞
        if self.data.ncon > 0:
            self._publish_collision_warning()
        
        # 更新 viewer
        if self.viewer.is_running():
            self.viewer.sync()
    
    def predict_future(self, horizon_steps=100):
        """
        Level 3: 预测未来状态（仿真快于实时）
        用当前状态 + 当前速度趋势，预测 horizon_steps 后的状态
        """
        # 复制当前状态
        data_pred = mujoco.MjData(self.model)
        data_pred.qpos[:] = self.data.qpos[:]
        data_pred.qvel[:] = self.data.qvel[:]
        data_pred.ctrl[:] = self.data.ctrl[:]
        
        predicted_contacts = []
        for t in range(horizon_steps):
            mujoco.mj_step(self.model, data_pred)
            if data_pred.ncon > 0:
                predicted_contacts.append(t * self.model.opt.timestep)
        
        return predicted_contacts
    
    def _publish_collision_warning(self):
        """发布碰撞预警"""
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            
            if np.linalg.norm(force[:3]) > 10.0:  # > 10N
                msg = WrenchStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.wrench.force.x = float(force[0])
                msg.wrench.force.y = float(force[1])
                msg.wrench.force.z = float(force[2])
                self.collision_pub.publish(msg)
```

### 从 Isaac Sim 到产线的完整部署 Checklist

| 阶段 | # | 检查项 | 负责人 | 状态 |
|------|---|--------|--------|------|
| **建模** | 1 | CAD 模型导出为 URDF（SI 单位） | ME | [ ] |
| | 2 | 惯性参数从 CAD 导出并验证 | ME | [ ] |
| | 3 | mesh 格式统一（STL碰撞 + OBJ可视化） | ME | [ ] |
| | 4 | URDF 通过 `check_urdf` 验证 | SW | [ ] |
| **仿真** | 5 | Isaac Lab 环境搭建并运行 | RL | [ ] |
| | 6 | 仿真保真度评估 RGM < 0.3 | RL | [ ] |
| | 7 | 域随机化配置（5 项标准套餐） | RL | [ ] |
| | 8 | RL 策略训练收敛（reward 稳定） | RL | [ ] |
| | 9 | 仿真中策略成功率 > 90% | RL | [ ] |
| **标定** | 10 | 激励轨迹设计并在真机执行 | SW | [ ] |
| | 11 | SysId 参数辨识完成 | SW | [ ] |
| | 12 | 标定后 RGM 改善 > 30% | SW | [ ] |
| **部署** | 13 | C++ 推理栈（LibTorch/ONNX）编译通过 | SW | [ ] |
| | 14 | 推理延迟 < 10ms @ batch=1 | SW | [ ] |
| | 15 | ros2_control 配置切换到真机驱动 | SW | [ ] |
| | 16 | 安全限制生效（速度/力矩/工作空间） | Safety | [ ] |
| | 17 | E-stop 测试通过 | Safety | [ ] |
| | 18 | 低速测试（10%）通过 | SW | [ ] |
| | 19 | 中速测试（30%）通过 | SW | [ ] |
| | 20 | 全速测试（100%）50 次成功率 > 85% | SW+QA | [ ] |
| **监控** | 21 | Level 1 数字孪生上线 | SW | [ ] |
| | 22 | 碰撞预警系统激活 | Safety | [ ] |
| | 23 | 性能指标 dashboard 搭建 | Ops | [ ] |
| | 24 | 异常自动报警配置 | Ops | [ ] |

### 练习

1. **[编程]** 实现 Level 1 数字孪生：一个 MuJoCo 实例模拟"真机"发布 `/joint_states`，另一个 RViz 实例实时显示。测量端到端延迟（从关节运动到 RViz 更新）。
2. **[编程]** 实现 `DigitalTwinBridge` 节点。用两个 MuJoCo 实例模拟真机和孪生，验证状态同步和碰撞预警功能。
3. **[思考题]** Level 3 预测孪生需要仿真"跑得比真机快"。MuJoCo 可以做到 10-100 倍实时速度。如果仿真 10 倍速运行，可以预测未来 0.1-1 秒的状态。这对碰撞避免有什么价值？对预测性维护呢？

### 跨章综合练习

**[跨章综合]** 构建一个完整的 sim-to-real 迷你管线：

1. 使用 P01 中构建的 7-DOF 机械臂 URDF
2. 实现 URDF → MJCF 的自动转换脚本，并对比转换前后的零位 FK 结果
3. 在 MuJoCo 中配置 3 项域随机化（质量 $\pm 15\%$、关节摩擦 $\pm 30\%$、观测噪声 $\sigma = 0.01$）
4. 编写一个简单的 reach 任务 reward 函数，训练 PPO 策略 500 epoch
5. 导出策略为 ONNX 格式，用 C++ 加载并测量推理延迟
6. 实现 action filter（$\alpha = 0.3$），对比有无 filter 时策略输出的平滑度

**验收标准**：
- 转换后 FK 位置误差 $< 1\text{ mm}$
- DR 训练的策略在 5 组不同参数下的成功率 $> 70\%$
- C++ ONNX 推理延迟 $< 5\text{ ms}$（CPU）

---

## P02.7+ sim-to-real 部署安全规程 ⭐⭐

### 为什么 sim-to-real 部署需要安全分级

将仿真中训练的策略部署到真实机器人是整个管线中风险最高的环节。仿真中一个"不太好"的策略最多浪费算力；真机上一个不安全的策略可能损坏机器人硬件或伤害操作人员。因此，真机部署必须遵循严格的安全分级流程。

> **跨领域类比**：sim-to-real 部署类似于航空领域的飞行测试分级。新飞机不会直接满载乘客试飞——先在风洞中测试（仿真），再做无人低速滑跑（mock hardware），再做无人低速飞行（10% 速度），逐步升级到有人全速飞行（100% 部署）。每个阶段都有明确的通过标准和安全约束，不满足就不进入下一阶段。

### 五级部署阶梯

| 级别 | 环境 | 速度 | 安全约束 | 通过标准 |
|------|------|------|---------|---------|
| L0 | 纯仿真 | 100% | 无 | 仿真成功率 $> 90\%$ |
| L1 | Mock Hardware | 100% | 虚拟限位 | 关节轨迹在安全范围内 |
| L2 | 真机 + 围栏 | 10% | 硬件急停 + 速度限制 | 50 次无碰撞 |
| L3 | 真机 + 围栏 | 30% | 硬件急停 + 力矩限制 | 100 次成功率 $> 80\%$ |
| L4 | 真机 | 100% | 全部安全系统 | 200 次成功率 $> 85\%$ |

**每级升级前必须满足的前提条件**：

- L0 → L1：策略在 DR 覆盖范围内的 5 组参数上均表现稳定
- L1 → L2：action filter 参数调优完成，输出轨迹无突变
- L2 → L3：L2 阶段无任何触发急停的事件
- L3 → L4：L3 阶段的力矩峰值始终低于安全阈值的 80%

### 安全层实现

真机部署中的安全保障是多层的——不依赖单一机制：

```
策略网络输出
    │
    ▼
[Action Filter]  ←── 输出平滑 + 加速度限制
    │
    ▼
[关节限位裁剪]   ←── 硬性关节角度范围
    │
    ▼
[速度限制]       ←── 最大关节速度/笛卡尔速度
    │
    ▼
[力矩限制]       ←── 最大关节力矩
    │
    ▼
[碰撞检测]       ←── 自碰撞 + 环境碰撞（MoveIt2 或 FCL）
    │
    ▼
[硬件急停]       ←── 物理按钮 + 软件触发
    │
    ▼
  执行器
```

每一层都是独立的安全守卫——即使上层失效，下层仍然能阻止危险动作。这种"纵深防御"（defense in depth）策略借鉴了工业安全标准 ISO 10218（机器人安全）的分层保护理念。

```python
class SafetyLayer:
    """多层安全守卫——sim-to-real 部署的关键中间件"""

    def __init__(self, joint_limits, max_velocity, max_effort,
                 alpha_filter=0.3):
        self.joint_limits = joint_limits  # [(lower, upper), ...]
        self.max_velocity = max_velocity  # rad/s per joint
        self.max_effort = max_effort      # Nm per joint
        self.alpha = alpha_filter
        self.prev_action = None
        self.dt = 0.01  # 100 Hz control loop

    def apply(self, raw_action, current_pos, current_vel):
        """
        将策略原始输出经过多层安全过滤后输出。
        
        Args:
            raw_action: 策略网络输出（目标关节位置）
            current_pos: 当前关节位置
            current_vel: 当前关节速度
        Returns:
            safe_action: 安全过滤后的目标关节位置
        """
        import numpy as np

        action = np.array(raw_action, dtype=np.float64)

        # Layer 1: Action filter（指数平滑）
        if self.prev_action is not None:
            action = (self.alpha * action
                      + (1 - self.alpha) * self.prev_action)
        self.prev_action = action.copy()

        # Layer 2: 关节限位裁剪
        for i, (lo, hi) in enumerate(self.joint_limits):
            action[i] = np.clip(action[i], lo, hi)

        # Layer 3: 速度限制（限制单步位移）
        if current_pos is not None:
            max_delta = self.max_velocity * self.dt
            delta = action - current_pos
            delta = np.clip(delta, -max_delta, max_delta)
            action = current_pos + delta

        # Layer 4: 力矩估计与限制
        # 简化版：检查加速度是否会超过力矩限制
        # 完整版应使用逆动力学计算
        if current_vel is not None:
            accel = (action - current_pos) / (self.dt ** 2)
            # 粗估力矩 = 惯量 * 加速度（简化为常数惯量）
            est_effort = np.abs(accel) * 0.1  # 简化惯量估计
            scale = np.minimum(
                1.0, self.max_effort / (est_effort + 1e-8))
            action = current_pos + (action - current_pos) * scale

        return action
```

> **⚠️ 陷阱：安全层引入的延迟**
>
> 多层安全过滤会引入额外延迟：action filter 平滑 1-2 个控制周期、速度限制裁剪可能"刹车"突变指令。这些延迟在绝大多数场景下是可接受的（机械臂的机械响应本身就有 10-50 ms 延迟），但在高速接触切换场景（如快速抓取释放）中可能导致响应不及时。
>
> 解决方案：对不同任务阶段使用不同的 $\alpha$ 值——自由运动阶段 $\alpha = 0.5$（快响应），接近目标阶段 $\alpha = 0.2$（高精度），接触阶段 $\alpha = 0.1$（极平滑）。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "域随机化就是让仿真更逼真" | 域随机化的目的不是让仿真更逼真，而是让策略对参数变化更鲁棒。真实世界只是随机化覆盖的分布中的一个点。如果仿真已经非常精确，DR 的价值在于覆盖仿真无法精确捕捉的残差不确定性 |
| "URDF 转 MJCF 是无损的" | 转换必然丢失 ROS 特有标签（`<gazebo>` 插件、`<ros2_control>`），同时 MuJoCo 会用自己的默认值填充 URDF 中未定义的参数（接触刚度 `solref`、阻尼 `solimp`）。转换后必须逐项审查物理参数 |
| "Isaac Sim 比 MuJoCo 好因为渲染更真" | 两者定位不同：MuJoCo 擅长快速接触丰富的操作仿真（CPU 单线程可达数十倍实时），Isaac Sim 擅长 GPU 并行和光线追踪渲染。对于纯策略训练（不涉及视觉感知），MuJoCo + DR 往往比 Isaac Sim 无 DR 更有效 |
| "SysId 一次标定永久有效" | 机器人的物理参数会随时间漂移（关节磨损、润滑脂老化、负载变化）。生产环境中应定期重新标定，或在线自适应辨识 |
| "仿真参数尽量精确就不需要 DR" | 即使仿真参数非常精确，仍然存在不可建模的残差（如柔性线缆、装配公差、环境温度变化）。DR 覆盖的是这些**模型化困难的长尾不确定性** |
| "action filter 只是低通滤波器" | action filter 不仅平滑输出，还承担安全限速功能。一阶指数平滑（$a_t = \alpha \cdot a_{policy} + (1-\alpha) \cdot a_{t-1}$）的 $\alpha$ 参数同时控制响应速度和输出平滑度——$\alpha$ 越小越平滑但响应越慢 |
| "CI/CD 管线只检查代码" | 生产级机器人项目的 CI/CD 应该同时验证**代码**和**资产**：URDF 语法检查、惯性参数合法性、格式转换一致性、仿真 smoke test。资产错误的破坏力不亚于代码 bug |
| "数字孪生就是仿真" | 数字孪生不是独立的仿真——它是一个**与真机状态实时同步的仿真实例**。关键区别在于数据流方向：仿真是"从初始条件向前积分"，数字孪生是"从真机状态实时跟踪" |

---

## P02.8 资产管道的 CI/CD 集成 ⭐⭐

### 为什么需要自动化资产验证

在团队协作中，模型文件的修改（URDF 参数调整、mesh 替换、Xacro 宏修改）和代码修改一样需要自动化验证。一个未经检查的惯性参数错误可能不会导致编译失败，但会让仿真行为完全偏离预期——这类"静默错误"比编译错误更难发现。

### 完整的 CI/CD 管线

```yaml
# .github/workflows/asset_pipeline.yml
name: Asset Pipeline Validation
on:
  pull_request:
    paths:
      - 'urdf/**'
      - 'meshes/**'
      - 'config/**'

jobs:
  urdf-validation:
    runs-on: ubuntu-latest
    container: ros:jazzy
    steps:
      - uses: actions/checkout@v4
      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y liburdfdom-tools python3-pip
          pip3 install mujoco trimesh numpy

      - name: Xacro expand + check_urdf
        run: |
          for config in sim mock real; do
            xacro urdf/robot.urdf.xacro mode:=$config > /tmp/robot_${config}.urdf
            check_urdf /tmp/robot_${config}.urdf
          done

      - name: Inertia validation
        run: python3 scripts/validate_inertia.py /tmp/robot_sim.urdf

      - name: MuJoCo conversion test
        run: |
          python3 -c "
          import mujoco
          model = mujoco.MjModel.from_xml_path('/tmp/robot_sim.urdf')
          print(f'MuJoCo: {model.nq} DoF, {model.nbody} bodies')
          mujoco.mj_saveLastXML('/tmp/robot.mjcf', model)
          "

      - name: Smoke test (10-step simulation)
        run: |
          python3 -c "
          import mujoco, numpy as np
          model = mujoco.MjModel.from_xml_path('/tmp/robot.mjcf')
          data = mujoco.MjData(model)
          for _ in range(10):
              mujoco.mj_step(model, data)
          # 检查无 NaN
          assert not np.any(np.isnan(data.qpos)), 'qpos 出现 NaN!'
          assert not np.any(np.isnan(data.qvel)), 'qvel 出现 NaN!'
          print('Smoke test passed')
          "
```

### 资产版本控制策略

| 文件类型 | 版本控制方式 | 说明 |
|---------|-------------|------|
| `.urdf.xacro` | Git 直接追踪 | 文本文件，diff 友好 |
| `.yaml`（参数文件） | Git 直接追踪 | 参数变更需要 review |
| `.stl` / `.obj`（mesh） | Git LFS | 二进制文件，用 LFS 避免仓库膨胀 |
| `.dae`（可视化 mesh） | Git LFS | 同上 |
| `.mjcf` / `.usd`（转换产物） | **不纳入版本控制** | CI/CD 管线自动生成，加入 `.gitignore` |

```bash
# .gitignore 中排除转换产物
*.mjcf
*.usd
*.usda
*.usdc
# 但保留 URDF 源文件和 mesh
!urdf/**
!meshes/**
```

> **本质洞察**：资产管道 CI/CD 的核心理念是"模型即代码"（Model as Code）。就像软件工程中的基础设施即代码（Infrastructure as Code）一样，机器人模型文件应该接受与源代码相同的严格度：Pull Request 审查、自动化测试、持续集成。一个未经审查的惯性参数修改和一个未经审查的控制器参数修改一样危险。

### 仿真器选择决策树

在实际项目中，选择哪个仿真器常常令人纠结。以下决策树基于 2025-2026 年主流方案：

```
你的任务需要什么？
│
├── 纯策略训练（无视觉）
│   ├── 接触密集（操作、抓取）→ MuJoCo（CPU 快速迭代）
│   ├── 需要 GPU 并行（数千环境）→ Isaac Lab / MuJoCo MJX
│   └── 简单任务（reach, push）→ MuJoCo 或 Gymnasium-Robotics
│
├── 视觉策略训练
│   ├── 需要光线追踪级渲染 → Isaac Sim + Replicator
│   ├── 简单 RGB/深度 → MuJoCo + 离线渲染增强
│   └── 真实图像 → 真机数据 + sim 策略微调
│
├── ROS2 系统集成测试
│   └── Gazebo Harmonic（原生 ROS2 集成）
│
└── 数字孪生 / 在线监控
    ├── 低延迟碰撞检测 → MuJoCo（< 1ms 单步）
    └── 可视化展示 → Isaac Sim / RViz
```

> **反事实推理**：如果只使用一个仿真器会怎样？假设你只用 Isaac Sim——它渲染能力强但启动慢、单步延迟高，不适合快速原型迭代。假设你只用 MuJoCo——它物理仿真快但渲染能力有限，无法训练视觉策略。假设你只用 Gazebo——它与 ROS2 集成最好但物理精度和速度都不如专用仿真器。**多仿真器并用是当前机器人项目的常态**，而资产管道正是让多仿真器协同成为可能的基础设施。

---

## P02.8 MuJoCo Playground 与现代 Sim-to-Real 工具链 ⭐⭐⭐

### 动机——2025 年的 Sim-to-Real 生态

2024-2025 年间，sim-to-real 生态发生了重大变化。MuJoCo Playground（Google DeepMind，RSS 2025）和 Isaac Lab 2.0 分别代表了两种路线：MuJoCo 路线追求轻量级、快速迭代，在单 GPU 上几分钟内完成训练；Isaac Lab 路线追求大规模 GPU 并行，在多 GPU 上数小时完成复杂任务训练。两者不是竞争关系而是互补——选择取决于任务复杂度和硬件资源。

### MuJoCo Playground 架构

MuJoCo Playground 的核心创新是将 MuJoCo 的 C 引擎替换为 MJX（MuJoCo 的 JAX 后端），实现了**全 GPU 加速的物理仿真 + 训练一体化**：

```
传统流程:                     MuJoCo Playground:
CPU 物理仿真 (MuJoCo C)       GPU 物理仿真 (MJX/JAX)
     ↓ 数据拷贝                    ↓ 零拷贝
CPU→GPU 传输                  GPU 上训练 (Brax/JAX PPO)
     ↓                            ↓ 零拷贝
GPU 训练 (PyTorch)            GPU 渲染 (Madrona)
     ↓ 数据拷贝
GPU→CPU 传输
     ↓
CPU 渲染
```

关键优势：消除了 CPU-GPU 数据拷贝瓶颈。传统流程中每个时间步需要 CPU→GPU 和 GPU→CPU 两次数据传输，当并行环境数量达到数千个时，这个开销成为主要瓶颈。MJX 将整个 pipeline 保持在 GPU 上，实现了 10-100 倍的训练加速。

### MuJoCo-USD 互转——2025 年新工具

`mujoco-usd-converter` 于 2025 年发布（由 Google DeepMind MuJoCo 团队和 NVIDIA 合作推进；具体发布形式请参阅 [项目 GitHub](https://github.com/google-deepmind/mujoco) 获取最新信息），这是首个支持 MJCF → USD 双向转换的工具：

```bash
# 安装
pip install mujoco-usd-converter

# MJCF → USD
python -m mujoco_usd_converter --input robot.xml --output robot.usd

# USD → MJCF (实验性)
python -m mujoco_usd_converter --input robot.usd --output robot.xml --reverse
```

这个工具的意义在于：原来 URDF → MJCF 和 URDF → USD 是两条独立管线，参数可能不一致。现在 MJCF ↔ USD 直接互转，确保 MuJoCo 训练和 Isaac Sim 渲染使用完全相同的物理参数。

### 资产管道的 2025 推荐架构

结合新工具，更新后的资产管道架构为：

```
           CAD (SolidWorks / Onshape)
                    │
                    ▼  [solidworks_urdf_exporter / onshape-to-robot]
          URDF / Xacro  ← 唯一版本控制源
                    │
     ┌──────────────┼──────────────┐
     ▼              ▼              ▼
┌──────────┐  ┌──────────┐  ┌──────────┐
│ MuJoCo   │  │ Gazebo   │  │ Isaac    │
│ (MJCF)   │  │Harmonic  │  │ Sim/Lab  │
│          │  │原生 URDF │  │ (USD)    │
└────┬─────┘  └──────────┘  └─────┬────┘
     │                            │
     └─────── mujoco-usd ────────┘
              双向互转
```

新增的 MJCF ↔ USD 互转通道意味着：在 MuJoCo 中标定好的接触参数可以直接导入 Isaac Sim 进行大规模视觉渲染训练，无需手动同步参数。

### 选择 MuJoCo Playground vs Isaac Lab

| 维度 | MuJoCo Playground | Isaac Lab |
|------|-------------------|-----------|
| **硬件要求** | 单 GPU (RTX 3060+) | 多 GPU 推荐 (A100/H100) |
| **训练时间** | 分钟级 (简单任务) | 小时级 (复杂任务) |
| **并行环境数** | 数百-数千 | 数千-数万 |
| **视觉渲染** | Madrona (基础) | Omniverse (光线追踪) |
| **接触模型** | 软约束凸优化 | PhysX (互补性/TGS) |
| **生态** | 学术开源 | NVIDIA 商业支持 |
| **适用** | 快速原型、单任务 | 大规模 DR、视觉策略 |

**选择指南**：

```
你的任务是否需要视觉输入？
  │
  ├─ 否（纯状态输入）→ MuJoCo Playground（更快）
  │
  └─ 是
       │
       ├─ 需要光线追踪级别渲染？→ Isaac Lab（Omniverse）
       │
       └─ 基础视觉就够？→ 都可以，MuJoCo Playground + Madrona 更轻量
```

### MuJoCo Playground 的零样本 Sim-to-Real 实践

MuJoCo Playground 在 2025 年的 RSS 论文中展示了六种机器人平台的零样本（zero-shot）sim-to-real 迁移，成功率 84%-93%。其核心方法论可以提炼为三步：

**Step 1：精确建模 + 最小 DR**

与传统"大范围 DR 暴力覆盖"不同，Playground 提倡**先精确建模再适度 DR**。具体做法是：先用 SysId 标定基准参数，然后在基准 $\pm$10-15% 范围内做 DR——范围比传统做法（$\pm$30-50%）小得多，但因为基准更准，策略质量更高。

**Step 2：阶跃响应验证**

在每个关节上做阶跃响应测试（突然施加目标角度变化），对比仿真和真机的阶跃响应曲线。关注三个指标：上升时间（rise time）、超调量（overshoot）、稳态误差（steady-state error）。如果三者匹配良好（误差 < 10%），说明 actuator 模型足够精确。

**Step 3：在线适应**

部署时可选择在线参数适应——用真机运行数据持续更新仿真参数（类似于在线 SysId）。这弥补了 DR 无法覆盖的时变因素（温度漂移、磨损）。

> **本质洞察**：MuJoCo Playground 的成功不在于某个单一技术突破，而在于**整合了正确的工具链**——JAX 后端消除数据拷贝、精确建模减少 DR 需求、阶跃响应验证提供定量保真度指标。这三者的组合使得"分钟级训练 + 零样本迁移"成为可能。

### 反事实推理——不用 MJX 后端

如果仍然使用传统的 CPU MuJoCo 后端训练 RL：

- 单个环境仿真速度约 10-50 倍实时，但无法 GPU 并行
- 训练 1M 步需要数小时（vs MJX 的数分钟）
- 开发者倾向于减少训练步数来"省时间"→ 策略欠训练→ 迁移失败
- 被迫使用更大的 DR 范围来"补偿"策略质量不足→ 训练更难收敛→ 恶性循环

MJX 打破了这个循环：快速训练 → 充分训练 → 高质量策略 → 减少 DR 需求 → 更容易收敛。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：MJX 后端与 C 后端的数值差异
   错误做法：在 MJX 上训练策略，在 C 后端 MuJoCo 上验证
   现象：训练成功率 95%，C 后端验证只有 70%
   根本原因：MJX 使用 float32 而 C 后端默认 float64，
   加上 JAX 的 XLA 编译器可能重排浮点运算顺序，
   导致长 horizon 轨迹出现不可忽视的数值漂移
   正确做法：验证也使用 MJX 后端；或在训练时加入
   float32 量化噪声作为隐式 DR

💡 概念误区：认为"mujoco-usd-converter 可以完美双向转换"
   新手想法："MJCF 和 USD 可以无损互转"
   实际上：MJCF 有些概念在 USD 中没有直接对应（如 tendon、equality constraint），
   反之 USD 的 composition arcs 在 MJCF 中无等价。
   转换工具会尽力保留物理参数，但语义不匹配的部分会丢失或近似。
   正确做法：转换后始终做对比验证——对同一关节轨迹，
   比较两个格式加载后的物理响应差异。

🧠 思维陷阱：认为"MuJoCo Playground 可以替代 Isaac Lab"
   新手想法："Playground 更快更简单，不需要 Isaac Lab 了"
   实际上：两者适用场景不同。Playground 在单 GPU、状态输入、
   快速迭代场景中优势明显。但需要光线追踪级视觉渲染、
   大规模场景管理、或 NVIDIA 硬件生态集成时，Isaac Lab 不可替代。
   正确做法：原型阶段用 Playground 快速验证，
   视觉策略和大规模训练切换到 Isaac Lab。
```

### 练习

1. **[编程]** 安装 `mujoco-usd-converter`，将 Franka Panda 的 MJCF 转换为 USD。对比转换前后的关节数量、限位范围、惯性参数是否一致。记录任何丢失或近似的参数。
2. **[思考题]** MuJoCo Playground 使用 MJX（JAX 后端）实现全 GPU 加速。但 JAX 要求所有计算必须是**纯函数**（无副作用、无可变状态）。这对物理仿真引擎的设计意味着什么？提示：传统 MuJoCo C 引擎大量使用可变的 `MjData` 结构。
3. **[编程]** 按 MuJoCo Playground 的方法论，对你的机械臂做阶跃响应验证：在 MuJoCo 中对每个关节施加 0.5 rad 阶跃输入，记录上升时间、超调量、稳态误差。对比 PD 增益 $K_p = 50, 100, 200$ 三组参数的阶跃响应差异。

---

## P02.9 多仿真器一致性验证 ⭐⭐

### 动机——"同一个机器人在不同仿真器中行为不同"

当项目需要同时使用多个仿真器（如 MuJoCo 用于训练、Gazebo 用于 ROS2 集成测试、Isaac Sim 用于视觉渲染）时，一个关键问题是：**三个仿真器中的"同一个机器人"是否真的表现一致？**

### 一致性验证框架

多仿真器一致性验证应系统化地覆盖三个层面：

| 层面 | 验证内容 | 指标 | 可接受阈值 |
|------|---------|------|-----------|
| **静态一致性** | 零位 FK、关节限位、link 质量 | 精确匹配 | 误差 = 0 |
| **运动学一致性** | 相同关节角的末端位姿 | FK 误差 | < 1e-6 m |
| **动力学一致性** | 相同力矩序列的关节轨迹 | 轨迹 RMSE | < 0.05 rad |

**静态一致性**应该精确匹配——如果零位 FK 都不一致，说明转换过程中有参数丢失或错误。运动学一致性取决于浮点精度。动力学一致性取决于物理引擎差异，允许更大的误差。

```python
# scripts/cross_sim_validation.py — 跨仿真器一致性验证

import numpy as np
import mujoco
import json

class CrossSimValidator:
    """跨仿真器一致性验证"""
    
    def __init__(self, mjcf_path: str):
        """加载 MuJoCo 模型作为参考"""
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
    
    def static_check(self):
        """静态属性对比报告"""
        report = {
            'n_joints': int(self.model.njnt),
            'n_bodies': int(self.model.nbody),
            'n_actuators': int(self.model.nu),
            'joint_names': [],
            'joint_ranges': [],
            'body_masses': [],
        }
        
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            report['joint_names'].append(name)
            if self.model.jnt_limited[i]:
                lo, hi = self.model.jnt_range[i]
                report['joint_ranges'].append(
                    [name, float(lo), float(hi)])
        
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            report['body_masses'].append(
                [name, float(self.model.body_mass[i])])
        
        return report
    
    def kinematic_check(self, joint_angles_list):
        """
        运动学一致性检查：对一组关节角计算末端位姿
        返回 FK 结果，与其他仿真器对比
        """
        results = []
        for q in joint_angles_list:
            mujoco.mj_resetData(self.model, self.data)
            
            # 写入关节角
            idx = 0
            for i in range(self.model.njnt):
                jtype = self.model.jnt_type[i]
                qadr = self.model.jnt_qposadr[i]
                if jtype in (mujoco.mjtJoint.mjJNT_HINGE,
                             mujoco.mjtJoint.mjJNT_SLIDE):
                    if idx < len(q):
                        self.data.qpos[qadr] = q[idx]
                        idx += 1
            
            mujoco.mj_forward(self.model, self.data)
            
            # 记录所有 body 位姿
            poses = {}
            for i in range(self.model.nbody):
                name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, i)
                if name:
                    pos = self.data.xpos[i].tolist()
                    quat = self.data.xquat[i].tolist()
                    poses[name] = {'pos': pos, 'quat': quat}
            
            results.append(poses)
        
        return results
    
    def dynamic_check(self, ctrl_sequence, dt=0.001):
        """
        动力学一致性检查：回放控制序列，记录关节轨迹
        """
        mujoco.mj_resetData(self.model, self.data)
        trajectory = []
        
        for ctrl in ctrl_sequence:
            self.data.ctrl[:len(ctrl)] = ctrl
            mujoco.mj_step(self.model, self.data)
            trajectory.append(self.data.qpos[:self.model.nq].copy())
        
        return np.array(trajectory)
    
    def save_reference(self, path='cross_sim_reference.json'):
        """保存参考数据，供其他仿真器对比"""
        # 生成标准测试关节角
        test_angles = [
            [0.0] * 7,                              # 零位
            [0.5, -0.3, 0.2, -1.0, 0.1, 0.8, 0.3],  # 随机位置 1
            [-0.5, 0.3, -0.2, -2.0, -0.1, 1.5, 0.0], # 随机位置 2
        ]
        
        reference = {
            'static': self.static_check(),
            'kinematic': {
                'test_angles': test_angles,
                'fk_results': self.kinematic_check(test_angles),
            },
        }
        
        with open(path, 'w') as f:
            json.dump(reference, f, indent=2)
        
        print(f"Reference saved to {path}")
        return reference
```

### 动力学差异的根因分析

当两个仿真器对相同输入产生不同轨迹时，按以下顺序排查差异来源：

1. **积分器差异**：MuJoCo 默认使用 Euler 积分（可选 RK4），Gazebo/DART 使用半隐式 Euler。不同积分器在相同时间步下的数值误差累积不同。
2. **接触求解器差异**：MuJoCo 使用软约束凸优化求解器，Gazebo/ODE 使用 LCP 互补性求解器。同一接触场景的法向力和摩擦力可能显著不同。
3. **关节摩擦模型差异**：MuJoCo 的 `frictionloss` 是库仑摩擦模型；Gazebo 的 `<dynamics friction>` 也是库仑模型但参数化方式不同。
4. **时间步差异**：MuJoCo 默认 $dt = 0.002$ s，Gazebo 默认 $dt = 0.001$ s。不同时间步的积分误差不同。

> **反事实推理**：如果你发现 MuJoCo 和 Gazebo 对同一力矩序列的轨迹差异 RMSE > 0.1 rad，应该怎么做？不应该尝试"调参使两者一致"——因为两个物理引擎的数值方法根本不同，强行对齐一个参数可能导致其他参数偏离真实。正确做法是：(1) 确认两者的静态和运动学一致性；(2) 选择一个作为"训练仿真器"，另一个作为"验证仿真器"；(3) 用 DR 覆盖两者之间的动力学差异。

### ⚠️ 常见陷阱

```
💡 概念误区：认为"两个仿真器参数相同就应该给出相同结果"
   新手想法："URDF 是同一份，参数一模一样，为什么轨迹不同？"
   实际上：物理引擎不是数学公式的精确求解器——它们是数值近似器。
   不同的积分方案、接触模型、约束求解策略会导致即使参数相同，
   长 horizon 轨迹也会发散。这不是 bug，而是数值方法的固有特性。
   正确做法：接受动力学差异的存在，用 DR 覆盖，用 SysId 校准。
```

---

## 本章小结

### 术语速查表

| 术语 | 英文 | 一句话定义 |
|------|------|-----------|
| 资产管道 | Asset Pipeline | 从 CAD 源文件到各仿真器/真机的自动化转换流程 |
| Single Source of Truth | Single Source of Truth | URDF 作为唯一的版本控制源文件，其他格式通过转换生成 |
| 域随机化 | Domain Randomization (DR) | 训练时随机扰动仿真参数，使策略对参数不确定性更鲁棒 |
| 课程式 DR | Curriculum DR | 随训练进度逐步扩大 DR 范围，先在窄范围内学会再扩展 |
| SysId | System Identification | 用真机实验数据估计动力学参数（质量、摩擦、阻尼等） |
| sim-to-real gap | Sim-to-Real Gap | 仿真与真实之间的行为差异，来源于动力学/参数/感知/执行四维度 |
| RGM | Reality Gap Metric | 衡量仿真保真度的归一化指标 |
| CRISP | CRISP Architecture | GPU 策略网络（5-10 Hz）+ C++ 实时合规控制（1 kHz）的解耦架构 |
| action filter | Action Filter | 对策略输出进行平滑和限速的后处理模块 |
| 数字孪生 | Digital Twin | 与真机状态实时同步的仿真实例，用于监控/预警/预测 |

### 知识点总表

| 编号 | 知识点 | 核心要点 | 对应节 | 难度 |
|------|--------|---------|--------|------|
| 1 | 资产管道架构 | URDF 单源、5 目标转换、CI/CD 自动化 | P02.1 | ⭐⭐ |
| 2 | 格式转换实操 | MuJoCo/Gazebo/Isaac Sim 各自陷阱与最佳实践 | P02.2 | ⭐⭐ |
| 3 | 域随机化 | 运行时 API、5 项标准套餐、课程式 DR | P02.3 | ⭐⭐ |
| 4 | 物理参数辨识 | SysId 流程、激励轨迹设计、最小二乘标定 | P02.4 | ⭐⭐⭐ |
| 5 | sim-to-real gap 分析 | 四维分类（动力学/参数/感知/执行）、保真度评估 | P02.5 | ⭐⭐⭐ |
| 6 | CRISP 部署架构 | GPU 策略 + C++ 合规控制解耦、LibTorch/ONNX 推理 | P02.6 | ⭐⭐ |
| 7 | 数字孪生 | 三层次架构（镜像/碰撞预警/预测）、工业部署流程 | P02.7 | ⭐⭐⭐⭐ |
| 8 | CI/CD 资产验证 | 自动化 URDF 检查、惯性验证、格式转换测试、烟雾测试 | P02.8 | ⭐⭐ |
| 9 | 仿真器选择 | MuJoCo/Isaac Sim/Gazebo 的定位差异与选型决策树 | P02.8 | ⭐⭐ |

## 本章与后续章节的关系

| 后续章节 | 关系 | 铺垫知识点 |
|---------|------|-----------|
| M01 Pinocchio | M01 用 Pinocchio 加载本章管道生成的 URDF | 资产管道、格式一致性 |
| M12 ros2_control | M12 深入实现本章 CRISP 架构的 C++ 实时控制层 | CRISP 架构、硬件接口切换 |
| F09 学习型力控 | F09 的 RL 训练依赖本章的 DR 和 sim-to-real 方法论 | 域随机化、action filter |
| D04/D10 双臂学习 | D04/D10 的仿真环境基于本章的资产管道 | 多格式转换、仿真环境配置 |

## 累积项目：本章新增模块

**跨方向共享基础进度**：

```
P01: URDF/Xacro 建模 ✓
P02 新增:
  ├─ 资产管道脚本（URDF → MuJoCo/Gazebo/Isaac Sim）
  ├─ 域随机化配置模板（5 项标准套餐）
  ├─ SysId 标定脚本框架
  ├─ 仿真保真度评估工具
  ├─ Action filter 实现
  └─ CRISP 最小化原型（Python 10Hz + C++ 1kHz）
```

## 常见误解汇总

| # | 误解 | 正确理解 |
|---|------|---------|
| 1 | "域随机化是让仿真更精确" | DR 的目的不是让仿真更像真实世界，而是让策略对仿真与真实之间的差异**更鲁棒**。DR 通过在参数空间中训练分布，使策略学会应对参数变化，从而在未见过的真实参数下也能工作 |
| 2 | "SysId 和 DR 二选一即可" | SysId 和 DR 是**互补**而非替代关系。SysId 标定出精确的基准参数（均值），DR 在基准周围做适度随机化（方差）。只做 SysId 不做 DR，策略对未标定误差没有鲁棒性；只做 DR 不做 SysId，随机化中心偏离真实值太远，训练效率极低 |
| 3 | "DR 范围越大越好——能覆盖所有可能" | DR 范围过大会导致策略变得过于保守（最坏情况优化），性能下降。推荐做法是从窄范围开始，用课程式 DR 逐步扩大，同时监控训练 reward 曲线 |
| 4 | "不同仿真器之间参数一致就能得到相同结果" | 即使所有物理参数完全相同，不同仿真器由于**积分方案**（半隐式 Euler vs Runge-Kutta）、**接触模型**（LCP vs 软约束凸优化）、**约束求解策略**的差异，长 horizon 轨迹仍会发散。这不是 bug，而是数值方法的固有特性 |
| 5 | "mujoco-usd-converter 可以完美双向转换" | MJCF → USD 方向相对成熟，但 USD → MJCF（反向）仍为实验性，可能丢失 MuJoCo 特有的 actuator 语义、tendon 约束等。建议以 URDF 为 single source of truth，转换后做跨仿真器一致性验证 |
| 6 | "Action filter 会引入不可接受的延迟" | EMA 滤波（$a_f(t) = \alpha a_r(t) + (1-\alpha) a_f(t-1)$）在 $\alpha=0.2\text{-}0.5$ 范围内引入的群延迟约为 1-3 个控制周期（1-3 ms @ 1 kHz），远小于真机执行器本身的 10-30 ms 响应延迟。滤波带来的平滑性收益远大于延迟代价 |

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Tobin et al. (2017) "Domain Randomization for Transferring DNNs from Simulation to Real World" | ⭐⭐ | DR 原论文——视觉域随机化的开创性工作 |
| Muratore et al. (2022) "Robot Learning from Randomized Simulations: A Review" | ⭐⭐⭐ | DR 综述——覆盖物理和视觉两类 DR |
| fan-ziqi/rl_sar (~1.2k Stars) | ⭐⭐ | 最完整的 RL locomotion 部署框架 |
| CRISP 框架 (TUM, 2025) | ⭐⭐⭐ | ros2_control + RL 解耦架构参考 |
| Isaac Lab 文档 "Domain Randomization" | ⭐⭐ | Isaac Lab 官方 DR 教程 |
| MuJoCo Playground (RSS 2025) | ⭐⭐ | MuJoCo MJX GPU 后端演示 |
| robot_descriptions.py | ⭐⭐ | 175+ 机器人描述即取即用 |
| Peng et al. (2018) "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization" | ⭐⭐⭐ | 物理 DR 在机械臂操作中的经典应用 |
| Mittal et al. (2025) "Isaac Lab: A Unified and Modular Framework for Robot Learning" | ⭐⭐ | Isaac Lab 的系统设计，包括 USD 资产管道 |
| ISO 10218-1:2011 "Robots and robotic devices -- Safety requirements" | ⭐⭐⭐⭐ | 工业机器人安全标准，真机部署必读 |
| Swevers et al. (1997) "Optimal Robot Excitation and Identification" | ⭐⭐⭐ | SysId 激励轨迹设计的经典方法 |
| Rigyd.com — SimReady Assets for Robotics | ⭐⭐ | 商业级 3D 资产管道参考，OpenUSD 驱动 |

### 研究实践建议

- **初学者**（第 1-2 周）：先完成 URDF → MuJoCo 的手动转换，熟悉格式差异。在单一仿真器中完成一个 reach 任务的 RL 训练
- **进阶**（第 3-4 周）：搭建 CI/CD 资产管线，配置域随机化标准套餐，训练 DR 策略并对比无 DR 的迁移效果
- **高级**（第 5-6 周）：实现 CRISP 架构原型，完成 SysId 标定流程，搭建 Level 1 数字孪生

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MuJoCo 导入 URDF 报错 | DAE mesh 不支持 | 1.检查 mesh 格式 2.转换 DAE→STL/OBJ 3.重新导入 | P02.2 |
| 仿真中机器人"飘走" | 惯性参数错误 / 基座未固定 | 1.检查 URDF 基座 joint 2.检查惯性张量正定性 3.加 `fix_base` | P02.2 |
| DR 训练 reward 不涨 | 随机化范围过大 | 1.减小 DR 范围 2.用课程式 DR 3.检查 reward scale 4.增加网络容量 | P02.3 |
| 真机部署策略抖动 | 输出不平滑 | 1.加 action filter ($\alpha$=0.3) 2.检查 obs 噪声 3.降低策略频率 | P02.4 |
| 策略推理太慢 | Python / CUDA 同步 | 1.用 LibTorch/ONNX C++ 2.batch=1 3.profile CUDA 调用 4.检查 GPU 利用率 | P02.6 |

---
