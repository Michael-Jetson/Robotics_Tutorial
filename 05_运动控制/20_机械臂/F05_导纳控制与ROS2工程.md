> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# F05 导纳控制与 ROS2 工程——ros2_controllers / FZI FDCC 完整实战

> **本章定位**：本章与 F04 形成对偶——F04 讲的是力矩接口型机器人的阻抗控制（输出力矩），本章讲的是位置/速度接口型机器人的导纳控制（输出位置修正）。导纳控制是**工业界最广泛使用**的力柔顺方案——因为绝大多数工业臂（UR、Fanuc、ABB、Yaskawa）只提供位置/速度接口，无法直接发送力矩命令。我们将精读 ros2_controllers 官方 `admittance_controller` 源码和 FZI FDCC（Forward Dynamics Compliance Controller），理解两种导纳实现的数学原理、工程架构和调参策略。
>
> **适用范围**：所有 ros2_control 兼容的位置/速度接口机器人（UR3/5/10/16/20/30、Fanuc、ABB、Kinova、Franka 导纳模式）。
>
> **前置依赖**：F01（阻抗/导纳概念）、F03（力控算法基础）、F04（笛卡尔阻抗——作为对比参考）、M12（ros2_control 框架——ControllerInterface/HardwareInterface）
>
> **下游章节**：F06（变阻抗与无源性）、F07（WBC 力控）、D05（遥操作中的导纳控制）
>
> **建议用时**：3 周（导纳原理 1 周 + ros2_controllers 源码精读 1 周 + FZI FDCC + 实战 1 周）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | 导纳控制与阻抗控制的因果方向有何不同？导纳控制器的输入和输出分别是什么？ | F01 第 2.4 节 |
| 2 | ros2_control 的 `ControllerInterface::update()` 方法在什么线程中执行？有什么实时约束？ | M12 ros2_control |
| 3 | `ChainableControllerInterface` 是什么？它如何实现控制器级联？ | M12 ros2_control |
| 4 | 导纳方程 $M_d \ddot{x} + D_d \dot{x} + K_d x = f_{ext}$ 中，当 $K_d = 0$ 时系统行为是什么？对应什么物理场景？ | F01 第 2.3 节 |
| 5 | 什么是 F/T（力/力矩）传感器？它测量哪 6 个分量？安装在哪里？ | 传感器基础 |

---

## 本章目标

学完本章后，你应该能够：

1. **解释**导纳控制的因果链——从力传感器读数到关节位置命令的完整信号流
2. **配置** ros2_controllers `admittance_controller` 的 YAML 参数，实现零力引导、恒力跟踪和选择性柔顺
3. **精读** `admittance_rule_impl.hpp` 中的核心导纳律积分代码，区分理论离散化与源码中的显式加速度、关节空间速度先行积分
4. **配置** FZI FDCC 的三个控制器（motion/force/compliance），理解虚拟正向动力学如何绕开 IK 奇异
5. **分析**导纳控制的稳定性约束——采样时间、环境刚度和内环带宽之间的关系
6. **实现**基于力传感器的碰撞检测与安全停止，以及人机协作手引导教（lead-through）

---

## 1. 导纳控制原理——力到运动的因果链 ⭐

### 1.1 动机——当你的机器人不接受力矩命令

> 回顾 F04 第 10 节：我们用一棵决策树展示了阻抗/导纳的选型——如果你的机器人只提供位置/速度接口（绝大多数工业臂），你**只能**用导纳控制。

这不是一个理论偏好问题，而是**硬件约束**。让我们看看为什么：

| 机器人 | 控制接口 | 内部架构 | 力矩级控制可能性 |
|--------|---------|---------|----------------|
| UR5e | 位置/速度 (500 Hz) | 工业伺服驱动器，位置环闭合在驱动器内部 | 否——驱动器不开放力矩接口 |
| Fanuc LR Mate | 位置 (250 Hz) | 专有控制器，通过 FANUC Karel 编程 | 否——完全封闭 |
| ABB IRB 120 | 位置/速度 (250 Hz) | ABB OmniCore 控制器 | 否——需要 ABB RWS 中间件 |
| Franka Panda | **力矩** (1 kHz) | FCI 开放力矩接口 | **是**——这就是 F04 的主题 |
| KUKA iiwa | **力矩** (1 kHz) | Sunrise 开放力矩接口 | **是** |

**关键观察**：全球安装量前 5 的工业机器人品牌（Fanuc、ABB、KUKA、Yaskawa、安川）中，只有 KUKA iiwa 系列提供力矩接口。其余 95% 以上的工业臂**只能**通过导纳控制实现力柔顺。

### 1.2 导纳控制的完整因果链

导纳控制的信号流是阻抗控制的"倒置"：

```
阻抗控制：运动状态(q, dq) → 控制律 → 力矩命令(τ)
导纳控制：外力测量(f_ext) → 导纳律 → 位置/速度命令(Δx, Δq)
```

完整的导纳控制因果链：

```
Step 1: 力传感器采集
  F/T 传感器 → 原始 wrench [fx, fy, fz, τx, τy, τz]
         ↓
Step 2: 力信号预处理
  低通滤波 → 重力补偿（减去工具重力）→ 坐标变换
         ↓
Step 3: 导纳律积分
  M_d · Δẍ + D_d · Δẋ + K_d · Δx = f_processed
  源码实现：显式计算笛卡尔加速度 → IK 得到关节加速度 → 关节速度先行积分
         ↓
Step 4: 逆运动学
  x_current + Δx → IK → Δq（关节修正量）
         ↓
Step 5: 关节命令输出
  q_reference + q_adm → 硬件 command interface（或目标控制器 reference interface）→ 关节伺服
```

> **类比**：导纳控制就像一个"力到运动翻译器"——它把外界施加的力"翻译"成机器人应该做的位移。如果有人推你的手，你的手臂不是直接感受到力矩然后反应（那是阻抗），而是你的大脑感知到力，然后命令你的手臂移动（那是导纳）。导纳控制需要一个中间环节（位置环/IK），就像你的大脑需要"想一下"再让手臂动。

### 1.3 导纳控制与阻抗控制的互补性分析

两种方法不是竞争关系，而是**互补的必然选择**：

| 维度 | 阻抗控制 | 导纳控制 |
|------|---------|---------|
| **因果方向** | 运动 → 力矩 | 力 → 运动 |
| **控制器输出** | 关节力矩 $\tau$ | 位置/速度修正 $\Delta x$, $\Delta q$ |
| **硬件要求** | 力矩级控制接口 | 位置/速度接口 + F/T 传感器 |
| **力分辨率** | 高（直接控制力矩） | 受限于 F/T 传感器精度和位置环带宽 |
| **带宽** | 高（单环：力矩→动力学） | 低（双环：导纳→IK→位置→动力学） |
| **对刚性环境的稳定性** | 好（阻抗直接控制力-位关系） | 差（位置环与环境刚度耦合可能失稳） |
| **对柔性环境的性能** | 好 | 好 |
| **实现复杂度** | 高（需精确动力学模型） | 中（导纳律简单，但需 IK） |
| **工业普及度** | 低（仅 Franka/iiwa） | **高**（所有 ros2_control 机器人） |

> **反事实推理**：如果所有工业臂都提供力矩接口（像 Franka 一样），导纳控制是否还有存在的必要？答案是**有**。即使硬件支持力矩控制，导纳控制在某些场景下仍然有优势：(1) 不需要精确的动力学模型——导纳律只需要 $M_d, D_d, K_d$（设计参数），不需要真实的 $M(q), C(q,\dot{q}), g(q)$；(2) 利用已有的位置环——如果机器人的位置环已经调得很好（如工业伺服），叠加导纳修正比从头实现力矩控制更稳定。

> **本质洞察**：阻抗控制和导纳控制的选择不是"哪个更好"的问题，而是**硬件接口决定的必然选择**。如果你的机器人只接受位置/速度命令（绝大多数工业臂），你只能用导纳控制——不管阻抗控制理论上多优越。

### 1.4 力传感器接口与信号预处理

导纳控制的输入质量**完全取决于**力传感器的信号质量。

**力/力矩传感器（F/T Sensor）基础**：

| 传感器 | 量程（力/力矩） | 分辨率 | 采样率 | 价格 | 典型应用 |
|--------|---------------|--------|--------|------|---------|
| ATI Mini45 | 145 N / 5 Nm | 0.025 N / 0.5 mNm | 7 kHz | ~$5000 | 研究级精密力控 |
| ATI Gamma | 400 N / 20 Nm | 0.05 N / 1 mNm | 7 kHz | ~$4000 | 工业打磨/装配 |
| Robotiq FT 300 | 300 N / 30 Nm | 0.2 N / 10 mNm | 100 Hz | ~$2000 | 协作臂柔顺 |
| OnRobot HEX-E | 200 N / 10 Nm | 0.2 N / 3 mNm | 1 kHz | ~$3000 | UR 集成 |
| 内置传感器（UR） | ~150 N | ~1 N | 500 Hz | 随臂附带 | 基础碰撞检测 |

**信号预处理的三个必需步骤**：

**Step 1：低通滤波**

力传感器的原始信号包含高频噪声（电气干扰、机械振动）。必须滤波后才能用于控制。

```
滤波方法：一阶指数低通（IIR）
  f_filtered(k) = α · f_raw(k) + (1-α) · f_filtered(k-1)
  α = filter_coefficient（ros2_controllers 中的参数）

  与采样周期 dt 的换算:
  α = 1 - exp(-2π f_c dt)
  f_c = -ln(1-α) / (2π dt) ≈ α / (2π dt)  (α 很小时)

  若控制周期 dt = 0.001 s（1 kHz）:
    α = 0.005 → 截止频率 ≈ 0.8 Hz（非常平滑，响应慢）
    α = 0.05  → 截止频率 ≈ 8 Hz（中等，推荐）
    α = 0.5   → 截止频率 ≈ 110 Hz（快速，但可能有噪声）
```

> **反事实推理**：如果不滤波会怎样？高频噪声通过导纳律积分会产生位置抖动（因为力噪声→加速度噪声→积分得到速度噪声→再积分得到位置噪声）。对于二阶导纳系统，力噪声被**二次积分**放大——1 N 的力噪声在 $M_d = 5$ kg 下产生 0.2 m/s$^2$ 的加速度噪声，积分 1 秒得到 0.2 m/s 的速度噪声。这显然不可接受。

**Step 2：工具重力补偿**

F/T 传感器测量的是传感器以下（含工具）的所有外力——包括**工具本身的重力**。要得到真正的环境外力，必须减去工具重力。

```
约定:
  W: fixed_world_frame.frame.id
  S: gravity_compensation.frame.id（通常为传感器/工具系）
  R_WS: S -> W 的旋转
  g^W = [0, 0, -9.81] m/s^2

工具重力在传感器系中的 wrench:
  f_g^S = R_WS^T * (m_tool * g^W)
  τ_g^S = r_CoG^S × f_g^S

环境外力:
  w_external^S = w_raw^S - [f_g^S; τ_g^S]

需要的参数（ros2_controllers 的 admittance_controller 配置）：
  gravity_compensation.frame.id: CoG 定义所在坐标系，通常是力传感器/工具坐标系
  gravity_compensation.CoG.force: 工具重量标量 [N]，即 mass × g
  gravity_compensation.CoG.pos: 工具质心在 gravity_compensation.frame.id 中的位置 [m]
  control.frame.id: 导纳律投影、选择性轴和位姿修正所使用的控制坐标系
  fixed_world_frame.frame.id: 重力方向参考世界系，通常是 base_link/world/map

说明：不同 ROS 2 发行版的精确字段会随 ros2_controllers 的 generate_parameter_library schema 调整；
实际工程以目标发行版的 admittance_controller 官方 schema 和示例 YAML 为准。
```

**Step 3：坐标变换**

力传感器测量的 wrench 在传感器坐标系中。导纳律通常在**基坐标系**或**柔顺参考坐标系**中定义。需要通过当前的传感器-基座变换矩阵进行坐标变换。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：忘记工具重力补偿**
>
> 错误做法：直接把 F/T 传感器的原始读数送给导纳律
>
> 现象：机器人在无外力时缓慢向下运动（因为工具重力被当作"外力向下推"）
>
> 根本原因：F/T 传感器测量的是传感器以下所有力，包括工具重力。一个 0.5 kg 的抓手在 z 轴产生约 4.9 N 的持续"外力"
>
> 正确做法：在 ros2_controllers 中正确配置 `gravity_compensation.CoG.force`

> 💡 **概念误区：认为导纳控制不需要力传感器**
>
> 新手想法："导纳控制的输入是外力，但外力可以用动量观测器估计，不需要传感器"
>
> 实际上：对于位置接口型机器人，动量观测器需要精确的动力学模型和关节力矩测量——而这些信息在封闭式工业臂中通常不可获取。FZI FDCC 是一个例外——它用虚拟正向动力学绕开了这个问题，但仍然受限于模型精度。对于精密力控，**硬件 F/T 传感器仍然是必需的**

### 练习

1. ⭐ **因果链对比**：画出阻抗控制和导纳控制的信号流图（从传感器输入到关节命令输出），标注每个环节的延迟和带宽瓶颈。
2. ⭐ **重力补偿计算**：一个 ATI Gamma 传感器安装在 UR5e 末端法兰上，工具是 0.3 kg 的真空吸盘，质心在传感器坐标系的 [0, 0, 0.05] m。当机器人末端朝下时，传感器读数应如何补偿？当末端水平时呢？
3. ⭐⭐ **滤波参数选择**：在 Python 中模拟一个含噪声的力信号（真实信号 5 N + 高斯噪声 $\sigma$=1 N），分别用 $\alpha = 0.005, 0.05, 0.5$ 进行低通滤波。绘制三组滤波后的信号，分析平滑度和延迟的 trade-off。

---

## 2. ros2_controllers admittance_controller 架构精读 ⭐⭐

### 2.1 动机——为什么要精读官方实现

ros2_controllers 的 `admittance_controller` 是 ROS2 生态中**官方的导纳控制器**（隶属于 ros-controls 组织，~4.5k★ 总仓库）。它被设计为通用的、可配置的导纳控制器，适用于所有 ros2_control 兼容的机器人。精读它的源码有三个价值：

1. **理解标准实现**：这是"工业界事实标准"——大多数新的 ROS2 力控项目都基于或参考它
2. **学习架构设计**：`ChainableControllerInterface` 的级联设计是 ros2_control 的核心范式
3. **识别局限**：理解它做了什么和**没做什么**，才能知道何时需要 FZI FDCC 或自定义实现

### 2.2 整体架构——控制器级联

```
F/T state interfaces（ForceTorqueSensor semantic component，前缀为 ft_sensor.name）
        ↓
参考源（非 chained mode: ~/joint_references；chained mode: 上游控制器写入 reference interfaces）
        ↓
admittance_controller（导纳控制器）
  ├── 1. 读取 F/T 传感器数据
  ├── 2. 低通滤波（filter_coefficient）
  ├── 3. 工具重力补偿（CoG.force = mass × 9.81）
  ├── 4. 导纳律计算：M·Δẍ + D·Δẋ + K·Δx = f_ext
  │      （源码先显式算 Δẍ）
  ├── 5. 通过 kinematics_interface 做微分 IK
  │      （KDL 或 Pinocchio 插件）
  └── 6. 关节空间速度先行积分，写出位置/速度命令
        ↓
hardware_interface 或目标控制器 reference interfaces（按 command_interfaces/command_joints 配置）
```

**ChainableControllerInterface 的核心概念**：

普通的控制器直接从 `state_interfaces_` 读状态、向 `command_interfaces_` 写命令。`ChainableControllerInterface` 额外导出一组可写的 **reference interfaces**，让其他控制器或控制器管理器把参考值写进来。

在导纳控制中：
- `chainable_command_interfaces` 定义 `admittance_controller/<joint>/position|velocity` 这类 reference interfaces，用作 **chained mode 的输入参考**。
- 非 chained mode 下，参考值来自 `~/joint_references` 话题。
- `command_interfaces` 是控制器最终要写的命令接口；它可以直接写硬件，也可以在目标发行版支持的级联配置中写另一个控制器导出的 reference interfaces。

资源配置的关键原则：同一组硬件 `joint/position` 或 `joint/velocity` command interfaces 不能被 `admittance_controller` 和 `JointTrajectoryController` 同时声明。若要让 JTC 参与轨迹生成，通常把 JTC 放在导纳控制器前面作为参考源；若只是测试导纳控制器，则直接向 `~/joint_references` 发送参考，避免两个控制器抢同一硬件资源。

> **类比**：ChainableControllerInterface 像一组标准化插座。reference interfaces 是给上游写参考的插座，command interfaces 是控制器最终写命令的插座。工程配置时先确认谁写参考、谁写硬件命令，再启动控制器。

### 2.3 YAML 配置完整解析

```yaml
admittance_controller:
  ros__parameters:
    # === 基础配置 ===
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    command_interfaces: [position]     # 输出类型：position 或 velocity
    state_interfaces: [position, velocity]
    chainable_command_interfaces: [position, velocity]  # 导出的 reference interfaces；不是硬件资源声明
    # command_joints 仅在命令目标关节名不同（如写另一个控制器的 reference interfaces）时设置；直连硬件时通常省略

    # === F/T 传感器配置 ===
    ft_sensor:
      name: ft_sensor                  # F/T semantic component 的接口前缀
      frame:
        id: tool0                      # 传感器测量帧
        external: false                # false = frame 来自机器人 URDF/运动链
      filter_coefficient: 0.005        # 指数低通滤波系数（越小越平滑）
      # 计算公式：f_filtered = α·f_raw + (1-α)·f_filtered_prev
      # 换算：α = 1 - exp(-2π*f_c*dt)
      # dt=0.001s 时 α=0.005 → 截止频率 ≈ 0.8 Hz

    # === 导纳控制坐标系 ===
    control:
      frame:
        id: tool0                      # 导纳律计算、选择性轴和位姿修正所在坐标系
        external: false

    # === 固定世界系 ===
    fixed_world_frame:
      frame:
        id: base_link                  # 重力方向参考系；重力沿该系 -Z 方向
        external: false

    # === IK 配置 ===
    kinematics:
      plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
      plugin_package: kinematics_interface
      base: base_link
      tip: tool0
      alpha: 0.0005                    # Jacobian 阻尼伪逆正则化参数

    # === 导纳律参数 ===
    admittance:
      selected_axes: [true, true, true, true, true, true]
      # 选择哪些轴启用导纳（false = 该轴刚性跟踪参考位姿）

      # 虚拟质量矩阵 M_d（6个对角元素）
      mass: [5.5, 6.6, 7.7, 8.8, 9.9, 10.1]
      # 单位：kg（平移）、kgm²（旋转）
      # 物理含义：外力作用下末端的"惯性感"
      # 越大→加速越慢→响应越缓→越稳定

      # 阻尼比 ζ
      damping_ratio: [2.828, 2.828, 2.828, 2.828, 2.828, 2.828]
      # 无量纲。实际阻尼 D = 2ζ√(MK)
      # ζ = 1.0：临界阻尼
      # ζ = 2.828 ≈ 2√2：过阻尼（推荐用于工业导纳——更稳定）

      # 刚度矩阵 K_d（6个对角元素）
      stiffness: [214.1, 214.2, 214.3, 214.4, 214.5, 214.6]
      # 单位：N/m（平移）、Nm/rad（旋转）
      # K = 0 → 纯导纳（无弹簧回复力）→ 零力引导模式

    # === 工具重力补偿 ===
    # 按 ros2_control admittance_controller 官方 schema 格式:
    # 参考: https://control.ros.org/rolling/doc/ros2_controllers/admittance_controller
    gravity_compensation:
      frame:
        id: tool0                         # CoG 定义所在坐标系，通常取力传感器/工具坐标系
        external: false
      CoG:
        pos: [0.01, 0.01, 0.08]       # 工具质心在传感器坐标系中的位置 [m]
        force: 4.905                  # 工具重量标量 [N]，0.5 kg * 9.81
    # 注意:
    # - 'force' 是 gravity_compensation.CoG 的子项，且是正的重量标量，不是 3D 重力向量。
    # - control.frame.id 定义导纳律所在的控制帧，不等同于 CoG 定义帧。
    # - fixed_world_frame.frame.id 定义重力方向参考世界系，控制器内部会处理姿态变换。
    # - 精确字段以目标 ROS 2 发行版的 ros2_controllers admittance_controller schema 为准。
```

**参数之间的数学关系**：

阻尼 $D$ 由 mass、damping_ratio 和 stiffness 共同决定：

$$D_i = 2 \cdot \zeta_i \cdot \sqrt{M_i \cdot K_i}$$

以平移 x 方向为例：$D_x = 2 \times 2.828 \times \sqrt{5.5 \times 214.1} = 5.656 \times 34.3 = 194.0$ Ns/m

**为什么默认 $\zeta = 2.828$（过阻尼）？** 工业导纳控制需要额外的稳定性裕度——因为双环结构引入的延迟会降低有效阻尼。过阻尼确保即使在内环延迟和环境刚度耦合下，系统仍然稳定。

### 2.4 导纳律积分核心代码精读

导纳律的核心实现在 `admittance_rule_impl.hpp` 的 `calculate_admittance_rule()` 函数中。

**连续时间导纳方程**：

$$M_d \ddot{\Delta x} + D_d \dot{\Delta x} + K_d \Delta x = f_{ext}$$

**源码实际离散流程**：

`ros2_controllers` 的 `calculate_admittance_rule()` 并不是把阻尼项写成 $D\dot{\Delta x}_{k+1}$ 的 1D 阻尼隐式格式。它先用当前步的位姿偏差、导纳速度和 wrench 显式计算笛卡尔加速度：

$$\ddot{\Delta x}_k = M_d^{-1}(f_{ext,k} - D_d \dot{\Delta x}_k - K_d \Delta x_k)$$

然后通过 `kinematics_interface` 把这个笛卡尔加速度增量转换为关节加速度，再在关节空间做速度先行积分：

$$\dot{q}_{adm,k+1} = \dot{q}_{adm,k} + \ddot{q}_{adm,k}\Delta t$$

$$q_{adm,k+1} = q_{adm,k} + \dot{q}_{adm,k+1}\Delta t$$

所以更准确的说法是：**笛卡尔导纳加速度显式计算，关节偏置用速度先行（symplectic/半隐式位置更新）积分**。这比把整段实现称为“阻尼隐式半隐式 Euler”更贴近源码。

**几种离散写法的区别**：

| 写法 | 速度/加速度更新 | 位移更新 | 备注 |
|---------|---------|---------|--------|
| 显式 Euler | $a_k=(F_k-Dv_k-Kx_k)/M$，$v_{k+1}=v_k+a_k\Delta t$ | $x_{k+1}=x_k+v_k\Delta t$ | 最简单，数值能量容易增长 |
| 速度先行积分 | $a_k=(F_k-Dv_k-Kx_k)/M$，$v_{k+1}=v_k+a_k\Delta t$ | $x_{k+1}=x_k+v_{k+1}\Delta t$ | `admittance_controller` 的关节偏置积分属于这一类 |
| 阻尼隐式半隐式 | $v_{k+1}=\frac{Mv_k+(F_k-Kx_k)\Delta t}{M+D\Delta t}$ | $x_{k+1}=x_k+v_{k+1}\Delta t$ | 阻尼项更稳健，但不是当前源码的写法 |
| 完整隐式 Euler | $v_{k+1}=v_k+a(x_{k+1},v_{k+1})\Delta t$ | $x_{k+1}=x_k+v_{k+1}\Delta t$ | 线性系统数值稳定性最强，但有数值耗散 |

> **本质洞察**：看源码时要分清“方程在哪里离散化”。官方实现的弹簧、阻尼和外力项在笛卡尔空间用当前状态显式求加速度；真正积分保存的状态是关节空间的 admittance offset。稳定性不能只套一个 1D 阻尼隐式公式，仍要结合采样周期、IK、内环位置/速度控制和环境刚度一起分析。

**对应的代码逻辑**（按源码语义简化）：

```cpp
// admittance_rule_impl.hpp: calculate_admittance_rule()

// 1. 读取并滤波 F/T 传感器数据
wrench_filtered = alpha * wrench_raw + (1 - alpha) * wrench_filtered_prev;

// 2. 工具重力补偿
wrench_compensated = wrench_filtered - gravity_wrench_in_sensor_frame;

// 3. 坐标变换到控制帧
wrench_in_control_frame = transform_wrench(wrench_compensated,
                                           sensor_frame, control_frame);

// 4. 选择性轴过滤
for (int i = 0; i < 6; ++i) {
    if (!selected_axes[i]) {
        wrench_in_control_frame[i] = 0.0;  // 非选中轴不做导纳
    }
}

// 5. 当前导纳偏差 X 与速度 X_dot
X = current_ft_pose_in_base - reference_ft_pose_in_base;
X_dot = admittance_state.admittance_velocity;

// 6. 显式计算笛卡尔加速度: F = M*a + D*v + K*x
X_ddot = mass_inv.cwiseProduct(F_base - D * X_dot - K * X);

// 7. 微分 IK：笛卡尔加速度增量 -> 关节加速度增量
joint_acc = kinematics_interface.convert_cartesian_deltas_to_joint_deltas(
                current_joint_positions, X_ddot, ft_sensor_frame);

// 8. 额外关节阻尼，然后在关节空间速度先行积分
joint_acc -= joint_damping * joint_vel;
joint_vel += joint_acc * dt;
joint_pos += joint_vel * dt;

// 9. 输出关节命令
joint_command.position = reference_joint_positions + joint_pos;
joint_command.velocity = reference_joint_velocities + joint_vel;
```

### 2.5 参数调节指南——三种典型场景

**场景 A：零力引导（手动拖动 / lead-through）**

```yaml
admittance:
  selected_axes: [true, true, true, true, true, true]
  mass: [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
  damping_ratio: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]   # 临界阻尼
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]         # K=0 → 无回复力
```

**物理含义**：$K_d = 0$ → 无弹簧 → 末端不会回到原始位置。外力推动末端移动，松手后末端停在当前位置（阻尼使其减速至零）。$M_d$ 决定了"推动的手感"——越大越"重"，需要更大的力才能加速。

**调参要点**：
- $M_d$ 太小 → 稍有噪声就漂移
- $M_d$ 太大 → 推动费力，操作者体验差
- 推荐 $M_d = 3\text{-}10$ kg（平移），$1\text{-}5$ kgm$^2$（旋转）

**场景 B：恒力跟踪（打磨/擦拭）**

```yaml
admittance:
  selected_axes: [true, true, true, false, false, false]
  # 只在平移方向启用导纳，旋转锁定
  mass: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
  damping_ratio: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]    # 过阻尼→更稳定
  stiffness: [200.0, 200.0, 50.0, 50.0, 50.0, 50.0]
  # z 轴刚度低→法向柔顺；xy 轴刚度中→切向跟踪
```

**关键技巧**：要实现恒力 $F_{ref} = 10$ N，不是在导纳参数中设置，而是在**力信号处理**中叠加力偏置：

$$f_{input} = f_{measured} - F_{ref}$$

当实际力等于 $F_{ref}$ 时，$f_{input} = 0$，导纳律不产生修正——达到稳态。

**场景 C：高精度位置+选择性力柔顺**

```yaml
admittance:
  selected_axes: [false, false, true, false, false, false]
  # 只有 z 轴启用导纳，其余轴位置锁定
  mass: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
  damping_ratio: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
  stiffness: [500.0, 500.0, 100.0, 100.0, 100.0, 100.0]
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：selected_axes 设置不匹配任务**
>
> 错误做法：对打磨任务设置 `selected_axes: [true, true, true, true, true, true]`（全轴导纳）
>
> 现象：打磨力不稳定，机器人末端在切向和旋转方向上随外力漂移
>
> 正确做法：只在法向（通常 z）启用导纳。切向和旋转方向保持位置跟踪

> 🧠 **思维陷阱：认为导纳控制的 M_d 应该等于机器人的真实质量**
>
> 新手想法："$M_d$ 是虚拟质量，应该设为机器人末端的实际惯量才'物理正确'"
>
> 实际上：$M_d$ 是**设计参数**，不是物理参数。它决定了"外力作用下末端的响应速度"。真实末端惯量可能是 5-10 kg，但你可以设 $M_d = 1$ kg 让末端对力更敏感，或设 $M_d = 50$ kg 让末端更"迟钝"（更稳定）。选择取决于任务需求

> ⚠️ **编程陷阱：filter_coefficient 设置不当导致抖动或延迟**
>
> `filter_coefficient = 0.5`（太大）→ 力信号噪声通过 → 位置抖动
>
> `filter_coefficient = 0.001`（太小）→ 在 1 kHz 控制周期下等效截止频率约 0.16 Hz，力信号延迟可达数百 ms → 响应太慢
>
> 推荐范围：0.005-0.05。先用 0.01 开始，如果抖动则减小，如果响应慢则增大

### 练习

1. ⭐ **YAML 编写**：为 UR5e + ATI Gamma F/T 传感器编写 `admittance_controller` 的完整 YAML 配置文件。任务：沿桌面擦拭，法向力 5 N，切向跟踪 CAD 路径。
2. ⭐⭐ **源码精读**：找到 `admittance_rule_impl.hpp` 中 `calculate_admittance_rule()` 函数。标注：(a) 低通滤波的位置 (b) 重力补偿的位置 (c) 显式笛卡尔加速度计算的位置 (d) 关节空间速度先行积分的位置 (e) IK 调用的位置。
3. ⭐⭐ **数值积分对比**：在 Python 中实现 1D 导纳方程 $M\ddot{x} + D\dot{x} + Kx = F$ 的三种积分方法（显式/速度先行/隐式 Euler）。让 $F$ 为阶跃输入，绘制三种方法的位移响应。在 $\Delta t = 0.01$ s 和 $\Delta t = 0.002$ s 下比较稳定性。

---

## 3. FZI FDCC——虚拟正向动力学绕开 IK 奇异 ⭐⭐

### 3.1 动机——传统导纳的奇异问题

> 回顾第 2 节：ros2_controllers 的导纳流程是 $f_{ext} \to \Delta x \to \text{IK}(J^{-1}) \to \Delta q$。这个流程在奇异附近有**致命问题**——IK 需要 $J^{-1}$（或 $J^+$），而 $J$ 在奇异处降秩，$J^{-1}$ 爆炸。

**问题的严重性**：工业任务中机器人经常工作在或经过奇异附近的构型。例如：
- UR 系列在完全伸展时（"肘部奇异"）
- 大多数 6-DOF 臂在工作空间边界附近

传统导纳控制器在这些构型附近会产生关节速度跳变，甚至导致控制器崩溃。

### 3.2 Scherzinger 的虚拟正向动力学思想

FZI 的 FDCC（Forward Dynamics Compliance Controller，Scherzinger 2017 IROS）用一个简单但精妙的思路解决了这个问题。

**传统导纳**：

```
f_ext → 导纳律 → Δx（笛卡尔修正）→ IK(J⁻¹) → Δq（关节修正）
                                        ↑ 奇异时 J⁻¹ 爆炸！
```

**FDCC**：

```
f_ext → J^T → τ_virtual（虚拟关节力矩）
                ↓
        虚拟正向动力学：M_v · q̈ + D_v · q̇ = τ_virtual
                ↓
        积分 → q̇ → 积分 → q（直接得到关节角）
                ↓
        位置控制：q → 关节伺服
```

**核心洞察**：FDCC 用 $J^T$（转置）代替了 $J^{-1}$（逆）来完成笛卡尔→关节的映射。

| 映射 | 数学含义 | 奇异行为 |
|------|---------|---------|
| $J^{-1}$：速度映射 | 笛卡尔速度 → 关节速度 | 奇异处爆炸（降秩） |
| $J^T$：力映射 | 笛卡尔力 → 关节力矩 | **奇异处良态**（某些方向力为零而已） |

> **本质洞察**：$J^T$ 在奇异处为什么不会爆炸？因为 $J^T$ 做的是**力的映射**而非运动的映射。在奇异处，某些笛卡尔方向的**运动**消失（机器人无法在该方向移动），但力仍然可以传递——只是传递后的关节力矩在某些方向为零。零力矩意味着虚拟机器人在该方向不受力、不运动——这正是物理上正确的行为。

### 3.3 FDCC 的虚拟动力学方程——完整推导

FDCC 在关节空间中模拟一个"虚拟机器人"，其动力学方程为：

$$M_{virtual} \ddot{q} + D_{virtual} \dot{q} = J^T F_{cart}$$

其中：
- $M_{virtual}$ 是虚拟惯量矩阵（可以设为对角阵，甚至单位阵——不需要等于真实惯量）
- $D_{virtual}$ 是虚拟阻尼矩阵
- $F_{cart}$ 是笛卡尔空间的力（来自 PD 控制或 F/T 传感器）

**推导过程——从笛卡尔空间到关节空间的映射**：

**Step 1**：定义期望的笛卡尔空间柔顺行为

$$M_d \ddot{\tilde{x}} + D_d \dot{\tilde{x}} + K_d \tilde{x} = f_{ext}$$

其中 $\tilde{x} = x - x_d$ 是位姿偏差。这是标准的导纳方程——与 ros2_controllers 相同。

**Step 2**：将笛卡尔力映射到关节空间

利用虚功原理 $\tau = J^T F$，将等式两边映射到关节空间：

$$J^T M_d \ddot{\tilde{x}} + J^T D_d \dot{\tilde{x}} + J^T K_d \tilde{x} = J^T f_{ext}$$

**Step 3**：用关节空间变量替换

利用 $\dot{\tilde{x}} = J\dot{q}$（忽略 $\dot{J}$ 项——这是 FDCC 的一个近似），代入得：

$$J^T M_d J \ddot{q} + J^T D_d J \dot{q} + J^T K_d \tilde{x} = J^T f_{ext}$$

**Step 4**：Scherzinger 的简化

Scherzinger 做了一个关键的工程简化——将 $J^T M_d J$ 替换为可任意设定的对角矩阵 $M_{virtual}$，将 $J^T D_d J$ 替换为 $D_{virtual}$。物理上这意味着虚拟机器人的惯量/阻尼不再精确对应笛卡尔空间的期望参数，而是一组"近似等效"的关节空间参数。

同时，将弹簧项 $J^T K_d \tilde{x}$ 和外力项 $J^T f_{ext}$ 合并为一个笛卡尔力的关节投影：

$$M_{virtual} \ddot{q} + D_{virtual} \dot{q} = J^T F_{cart}$$

其中 $F_{cart}$ 包含了弹簧回复力和外力：$F_{cart} = K_d(x_d - x) + f_{ext}$。

**这个简化牺牲了什么？** 精确的笛卡尔空间阻抗行为——因为 $M_{virtual} \neq J^T M_d J$，所以末端在不同方向的等效惯量并非精确等于 $M_d$。但它获得了：(1) 计算效率——不需要每周期计算 $J^T M_d J$；(2) 数值鲁棒性——$M_{virtual}$ 是对角正定的，永远良态；(3) 设计自由度——可以为每个关节独立设定虚拟惯量。

**关键优势**：$M_{virtual}$ 和 $D_{virtual}$ 可以**任意设定**——它们不代表真实物理，只是控制参数。这给了工程师极大的设计自由度：
- 不同关节可以有不同的虚拟惯量→不同的响应速度
- 接近关节限位时可以增大虚拟阻尼→自然减速

### 3.4 FDCC 的三个控制器

FZI `cartesian_controllers` 提供了三个控制器，覆盖不同任务场景：

| 控制器 | 输入 | 内部流程 | 用途 |
|--------|------|---------|------|
| `cartesian_motion_controller` | 位姿参考 $x_d$ | $x_d \to \text{PD wrench} \to J^T \to \text{FD} \to q$ | 笛卡尔位置跟踪（带柔顺） |
| `cartesian_force_controller` | wrench 参考 $F_d$ | $F_d \to J^T \to \text{FD} \to q$ | 力跟踪/力限制 |
| `cartesian_compliance_controller` | **位姿 + wrench 同时** | $(x_d, F_d) \to \text{PD+FF} \to J^T \to \text{FD} \to q$ | 打磨（路径+法向力） |

**cartesian_compliance_controller 的 YAML 配置**：

```yaml
cartesian_compliance_controller:
  ros__parameters:
    end_effector_link: tool0
    robot_base_link: base_link
    ft_sensor_ref_link: sensor_link
    compliance_ref_link: tool0      # 柔顺参考帧
    joints: [joint_1, ..., joint_6]

    stiffness:
      trans_x: 500.0
      trans_y: 500.0
      trans_z: 200.0               # z 向软——适合法向力跟踪
      rot_x: 20.0
      rot_y: 20.0
      rot_z: 20.0

    solver:
      error_scale: 0.5            # 误差缩放因子（PD 增益）
      iterations: 1               # 每控制周期的 FD 迭代次数
```

### 3.5 FDCC vs. ros2_controllers admittance_controller

| 维度 | ros2_controllers admittance | FZI FDCC |
|------|---------------------------|----------|
| 笛卡尔→关节映射 | IK（$J^{-1}$ 或 $J^+$） | 虚拟正向动力学（$J^T$） |
| 奇异行为 | 需要阻尼伪逆正则化 | **天然良态** |
| 物理一致性 | 导纳律在笛卡尔空间 | 虚拟动力学在关节空间 |
| 参数含义 | $M_d, D_d, K_d$（笛卡尔空间） | 虚拟质量/阻尼（关节空间） |
| 位姿+力同时输入 | 不原生支持 | **cartesian_compliance_controller** |
| ROS 版本 | ROS2 官方维护 | ROS1/ROS2（FZI 维护） |
| 文档质量 | 高（control.ros.org） | 中（GitHub README + ROSCon 2019） |

> **反事实推理**：如果 ros2_controllers 的 admittance_controller 也使用 $J^T$ 而不是 IK，它是否就不需要 FZI FDCC 了？理论上是的——$J^T$ 方法可以集成到 admittance_controller 中。但架构上有一个区别：admittance_controller 的输出是**笛卡尔位姿修正**，通过 IK 转换为关节修正。如果改用 $J^T$，输出就直接是**关节修正**，不再需要 IK——但这改变了控制器的接口语义，需要重新设计级联架构。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：FDCC 的 solver.error_scale 设置过大**
>
> 错误做法：设置 `error_scale: 1.0`
>
> 现象：位姿跟踪出现振荡——虚拟机器人对误差的反应过于激进
>
> 根本原因：error_scale 控制了笛卡尔误差到虚拟 wrench 的增益。过大等效于过高的 PD 增益
>
> 正确做法：从 `error_scale: 0.3` 开始，逐步增大。如果振荡则减小

> 💡 **概念误区：认为 FDCC 不需要 F/T 传感器**
>
> 新手想法："FDCC 用虚拟动力学绕开了 IK，是不是也不需要传感器了？"
>
> 实际上：FDCC 的 `cartesian_force_controller` 和 `cartesian_compliance_controller` **仍然需要** F/T 传感器来测量外力。FDCC 绕开的是 IK 奇异问题，不是传感器需求。只有 `cartesian_motion_controller`（纯位置跟踪+柔顺）可以不用传感器——但此时的"柔顺"来自虚拟弹簧-阻尼器，不是力反馈闭环

### 练习

1. ⭐ **FDCC 安装**：在 UR5e Gazebo 仿真中安装 FZI `cartesian_controllers`。分别运行 `cartesian_motion_controller` 和 `cartesian_compliance_controller`，感受两者的行为差异。
2. ⭐⭐ **奇异对比实验**：在同一 UR5e 上分别配置 `admittance_controller` 和 FZI `cartesian_compliance_controller`。让末端经过接近奇异的构型（如完全伸展），记录关节速度信号。哪个控制器在奇异附近更平滑？
3. ⭐⭐ **$J^T$ vs. $J^{-1}$ 分析**：在 Python 中计算 UR5e 在某个接近奇异的构型下的 $J$, $J^+$（伪逆）和 $J^T$。比较 $J^+ F$ 和 $J^T F$ 对同一笛卡尔力 $F$ 的响应。哪个更"温和"？

---

## 4. 导纳控制的稳定性分析 ⭐⭐⭐

### 4.1 动机——导纳控制为什么比阻抗控制更容易失稳

导纳控制有一个阻抗控制不具有的结构性弱点：**双环耦合**。

```
阻抗控制（单环）：
  控制律 → τ → 机器人动力学 → (q, dq)
  └─────────────────────────────┘ 一个闭环

导纳控制（双环）：
  导纳律 → Δx → IK → q_ref → 位置环 → τ → 动力学 → (q, dq)
  └──────── 外环（导纳）──────┘└── 内环（位置）──┘
  两个嵌套的闭环
```

内环（位置环）引入了额外的**相位延迟**——它不能瞬间跟踪参考。这个延迟在与环境交互时可能导致不稳定。

### 4.2 接触稳定性的关键约束

考虑导纳控制器与刚性环境（$K_e$）的耦合系统。更实用的经验法则（Ott 2008）：

$$K_e < \frac{D_d}{2 \Delta t}$$

**数值示例**：

```
设 D_d = 100 Ns/m，Δt = 0.002 s（500 Hz 控制频率）
→ K_e_max = 100 / (2 × 0.002) = 25000 N/m

这意味着导纳控制器能稳定交互的最大环境刚度为 25000 N/m。
钢铁表面的刚度约 10⁶ N/m → 远超限制 → 不稳定！
```

**解决方案的穷举式分类**：

| 方法 | 效果 | 代价 | 适用性 |
|------|------|------|--------|
| 增大 $D_d$ | 提高 $K_e$ 上限 | 导纳响应变慢 | 通用 |
| 减小 $\Delta t$（提高频率） | 提高 $K_e$ 上限 | 硬件限制 | 受限于硬件 |
| 增加柔性垫 | 直接降低 $K_e$ | 需额外硬件 | 可行时最有效 |
| 增大 $M_d$ | 间接提高稳定性 | 响应更慢 | 通用 |
| 使用阻抗控制 | 消除双环延迟 | 需要力矩接口 | 仅 Franka/iiwa |

> **反事实推理**：如果工业臂的位置环带宽是无穷大（瞬间跟踪），导纳控制的稳定性问题是否消失？是的——如果内环是理想的（零延迟），导纳控制退化为等效的阻抗控制（力→瞬间位移→瞬间力矩），不存在双环耦合问题。所以导纳控制的稳定性瓶颈**本质上是内环的非理想性**。

### 4.3 内环带宽要求

经验法则：**内环位置控制器的带宽必须至少是导纳控制器带宽的 5 倍**。

导纳控制器的等效带宽为：

$$\omega_{adm} = \sqrt{K_d / M_d}$$

内环位置控制器的带宽 $\omega_{inner}$ 必须满足：

$$\omega_{inner} \geq 5 \omega_{adm}$$

**数值示例**：

```
设 K_d = 200 N/m，M_d = 5 kg
→ ω_adm = √(200/5) = 6.32 rad/s ≈ 1 Hz
→ ω_inner ≥ 5 × 6.32 = 31.6 rad/s ≈ 5 Hz
→ 典型工业臂位置环带宽 10-50 Hz → 满足条件

但如果 K_d = 2000, M_d = 2：
→ ω_adm = √(2000/2) = 31.6 rad/s ≈ 5 Hz
→ ω_inner ≥ 158 rad/s ≈ 25 Hz → 许多工业臂做不到！
```

> **本质洞察**：导纳控制的参数空间不是任意的——$K_d$ 和 $M_d$ 的比值受限于内环带宽。这就是为什么工业导纳控制通常使用**低刚度 + 大虚拟质量**的组合——它压低了导纳带宽，留出足够的裕度给位置环。

### 4.4 采样时间的影响——离散化带来的额外约束

除了内环带宽约束外，离散时间积分本身也引入稳定性约束。

`ros2_controllers` 风格的实现是"显式计算加速度，再用新速度更新位置"。对 1D 接触系统 $M\ddot{x}+D\dot{x}+(K+K_e)x=0$，记 $K_t=K+K_e$，实际离散格式为：

$$v_{k+1}=v_k+\frac{\Delta t}{M}(-Dv_k-K_t x_k)$$

$$x_{k+1}=x_k+\Delta t\,v_{k+1}$$

对这个速度先行格式，Schur 稳定边界为：

$$\boxed{\Delta t < \frac{\sqrt{D^2+4M(K+K_e)}-D}{K+K_e}}$$

其中 $K_e$ 是环境刚度。工程上还要为内环延迟、IK、滤波和位置伺服留裕度，通常把控制频率再提高到理论下限的 5-10 倍。

**数值示例**：

```
D = 100 Ns/m, M = 5 kg, K_e = 10^5 N/m（铝）
→ Δt < 0.013 s → 理论下限约 76 Hz；工程建议 400-800 Hz

D = 100 Ns/m, M = 5 kg, K_e = 10^6 N/m（钢）
→ Δt < 0.0044 s → 理论下限约 230 Hz；工程建议 1-2 kHz
```

这个公式只覆盖导纳离散环节；官方 `admittance_controller` 还包含 IK、关节阻尼和内环伺服，实际稳定边界必须以整条控制链验证。这就是为什么导纳控制器接触钢铁表面时仍然经常需要柔性垫或更高阻尼。

### 4.5 离散化稳定性分析深化——采样时间 vs 环境刚度的量化关系 ⭐⭐⭐

上述公式给出了与本章伪代码一致的主稳定边界。本节用 z 域形式说明它来自哪里，并把表格、经验频率和柔性垫估算统一到同一个离散格式。

**连续系统状态空间形式**：

令 $x_1 = \Delta x$，$x_2 = \dot{\Delta x}$，导纳方程 $M\ddot{\Delta x} + D\dot{\Delta x} + (K+K_e)\Delta x = 0$（接触环境 $K_e$ 后等效刚度变为 $K+K_e$）写成：

$$\dot{X} = AX, \quad A = \begin{bmatrix} 0 & 1 \\ -(K+K_e)/M & -D/M \end{bmatrix}$$

连续系统的特征值为 $s_{1,2} = \frac{-D \pm \sqrt{D^2 - 4M(K+K_e)}}{2M}$。稳定条件：$D > 0$，$K+K_e > 0$。

**实际速度先行积分的 z 域稳定边界**：

速度先行积分先更新速度，再用新速度更新位移：

$$v_{k+1} = v_k + \frac{\Delta t}{M}(-D v_k - (K+K_e) x_k)$$
$$x_{k+1} = x_k + \Delta t \cdot v_{k+1}$$

这里的速度式仍把阻尼按旧速度显式处理；它对应 §2.4 中“显式加速度 + 速度先行位置更新”的离散思想，而不是阻尼隐式写法。若要采用更稳健的阻尼隐式格式，应使用 $v_{next}=(M v+(F-Kx)\Delta t)/(M+D\Delta t)$，再用 $x_{next}=x+v_{next}\Delta t$。

将第一式代入第二式，写成矩阵形式：

$$\begin{bmatrix} x_{k+1} \\ v_{k+1} \end{bmatrix} = \underbrace{\begin{bmatrix} 1 - (K+K_e)\Delta t^2/M & \Delta t(1 - D\Delta t/M) \\ -(K+K_e)\Delta t/M & 1 - D\Delta t/M \end{bmatrix}}_{A_{semi}} \begin{bmatrix} x_k \\ v_k \end{bmatrix}$$

对 $A_{semi}$ 的特征多项式应用 Schur 稳定性判据（所有特征值模 < 1），经完整推导得到稳定条件：

$$K_t\Delta t^2+2D\Delta t<4M,\quad K_t=K+K_e$$

等价地：

$$\boxed{\Delta t < \frac{\sqrt{D^2+4MK_t}-D}{K_t}}$$

速度先行积分对无阻尼保守系统有更好的长期能量行为；加入阻尼、接触和离散采样后，它不再是纯面积保持映射，仍需要按这个实际离散方程做稳定性分析。

**隐式 Euler 的稳定性**：

隐式 Euler 需要求解 $(I - A\Delta t)X_{k+1} = X_k$。其 z 域特征值始终在单位圆内（无条件稳定），但计算成本更高——需要每周期求解一个 $2 \times 2$ 线性方程组。

**按实际格式统一后的对比表**（取 $M=5$ kg, $K=0$）：

| 参数组合 | 速度先行 $\Delta t_{max}$ | 理论最低频率 | 工程建议频率（5-10x） |
|---------|:---:|:---:|:---:|
| $D=100, K_e=10^3$ | 0.0732 s | 13.7 Hz | 70-140 Hz |
| $D=100, K_e=10^4$ | 0.0358 s | 27.9 Hz | 140-280 Hz |
| $D=100, K_e=10^5$ (铝) | 0.0132 s | 75.9 Hz | 400-800 Hz |
| $D=100, K_e=10^6$ (钢) | 0.00437 s | 229 Hz | 1.1-2.3 kHz |
| $D=500, K_e=10^5$ | 0.0100 s | 100 Hz | 500-1000 Hz |

**关键工程结论**：

1. 真正的瓶颈是环境刚度 $K_e$ 和整条内环链路的延迟，而不是单个积分公式。
2. 接触铝（$K_e \approx 10^5$）通常需要 400-800 Hz 量级；接触钢（$K_e \approx 10^6$）通常需要 1 kHz 以上并配合柔性垫。
3. 增大阻尼 $D$ 不会无限放宽频率要求；在速度先行格式下还受 $M$ 与 $K_t$ 的二阶项约束。
4. 隐式 Euler 虽然对线性导纳方程更稳健，但引入**数值阻尼**，还会改变期望的柔顺动态。

> **类比**：采样时间与环境刚度的关系类似于弹簧振子的采样定理。弹簧-质量系统的固有频率 $\omega_n = \sqrt{(K+K_e)/M}$。Nyquist 采样定理要求采样率 $>2\omega_n$。对于钢表面 $K_e = 10^6$ N/m，$M = 5$ kg：$\omega_n = \sqrt{10^6/5} = 447$ rad/s $= 71$ Hz，Nyquist 要求 $>142$ Hz。但控制系统的稳定性比 Nyquist 更严格，通常需要数倍到 10 倍裕度；这与上表给出的 kHz 量级工程建议一致。

**工程解决方案——柔性垫的量化效果**：

在末端执行器上安装柔性垫（硅胶、橡胶）是工业界最常用的"物理滤波器"。它将等效环境刚度从 $K_e$ 降低到 $K_{eq} = \frac{K_e K_{pad}}{K_e + K_{pad}}$（串联弹簧）。

| 柔性垫 | $K_{pad}$ [N/m] | 钢表面 $K_{eq}$ [N/m] | 所需频率 |
|--------|:---:|:---:|:---:|
| 无垫 | $\infty$ | $10^6$ | 1.1-2.3 kHz |
| 硬硅胶 | $10^5$ | $9.1 \times 10^4$ | 380-760 Hz |
| 软硅胶 | $10^4$ | $9.9 \times 10^3$ | 140-280 Hz |
| 橡胶发泡 | $10^3$ | $999$ | 70-140 Hz |

**安装 $10^4$ N/m 的软硅胶垫后，接触钢表面的建议频率从 kHz 量级降到百 Hz 量级。** 这就是为什么 UR 系列协作臂在做力控接触任务时几乎总是配合柔性垫使用。

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：认为导纳控制参数可以任意设置**
>
> 新手想法："我想要高精度跟踪，就把 $K_d$ 设大一点（2000 N/m）、$M_d$ 设小一点（1 kg）"
>
> 实际上：$\omega_{adm} = \sqrt{2000/1} \approx 45$ rad/s $\approx 7$ Hz。需要内环带宽 $\geq$ 35 Hz。如果你的 UR5e 位置环带宽只有 20 Hz → 不稳定
>
> 正确做法：先确认内环带宽 $\omega_{inner}$，然后约束 $K_d/M_d < (\omega_{inner}/5)^2$

> ⚠️ **编程陷阱：导纳控制器的采样时间与内环不一致**
>
> 错误做法：导纳律以 100 Hz 运行，内环位置环以 500 Hz 运行，但导纳律的 $\Delta t$ 错误地设为 0.002 s（500 Hz 的周期）而非 0.01 s（100 Hz）
>
> 后果：积分步长错误 → 导纳响应异常 → 可能失稳
>
> 正确做法：确保 `period` 参数从 `update()` 的 `rclcpp::Duration` 中正确获取

### 练习

1. ⭐⭐ **稳定性边界计算**：给定 UR5e（位置环带宽约 25 Hz，控制频率 500 Hz）和导纳参数 $M_d = 5$ kg, $D_d = 100$ Ns/m, $K_d = 200$ N/m。计算：(a) 导纳带宽 $\omega_{adm}$；(b) 带宽比 $\omega_{inner}/\omega_{adm}$；(c) 最大可稳定环境刚度 $K_e^{max}$。该配置能否稳定接触钢铁工件？
2. ⭐⭐ **参数极限实验**：在 MuJoCo UR5e 仿真中，逐步增大 $K_d$（从 50 到 5000 N/m），保持 $M_d = 5$ kg。让末端接触刚性桌面，记录每组 $K_d$ 下的接触力波形。在什么 $K_d$ 值时出现振荡？这与理论预测的 $K_d^{max}$ 一致吗？
3. ⭐⭐⭐ **跨章综合题**：一个工厂需要 UR10e + ATI Gamma 在铝工件上打磨（法向力 15 N），控制频率 500 Hz。铝的接触刚度约 $5 \times 10^4$ N/m。(a) 使用 $K_e < D_d/(2\Delta t)$ 计算所需的最小 $D_d$；(b) 用临界阻尼关系 $D = 2\sqrt{MK}$ 确定 $M_d$；(c) 检验导纳带宽是否低于内环带宽/5。完成 YAML 配置。

---

## 5. 碰撞检测与安全停止 ⭐⭐

### 5.1 动机——力控机器人的安全底线

无论使用阻抗还是导纳控制，机器人在与人或环境交互时都可能发生**非预期碰撞**。碰撞检测是力控系统的**安全底线**——即使上层控制器失效，碰撞检测仍然保护人和设备。

### 5.2 基于力传感器的碰撞检测

最简单且最可靠的碰撞检测方法：**监控力传感器读数是否超过阈值**。

```cpp
// 简单阈值碰撞检测
struct CollisionDetector {
    double force_threshold = 30.0;    // N
    double torque_threshold = 5.0;    // Nm
    int consecutive_count = 0;
    int count_threshold = 5;          // 连续 5 个周期超标才触发

    bool detect(const Eigen::Matrix<double, 6, 1>& wrench) {
        bool force_exceeded = wrench.head(3).norm() > force_threshold;
        bool torque_exceeded = wrench.tail(3).norm() > torque_threshold;

        if (force_exceeded || torque_exceeded) {
            consecutive_count++;
        } else {
            consecutive_count = 0;
        }
        return consecutive_count >= count_threshold;
    }
};
```

**为什么需要连续计数而不是单次触发？** 力传感器可能有瞬态噪声尖峰（如机械振动、电磁干扰），单次超标不一定是真碰撞。连续 5 个周期（5 ms @ 1 kHz）超标才是可靠的碰撞信号。

#### 完整碰撞检测系统——多层级架构

工业级碰撞检测不是一个简单的阈值判断，而是一个**多层级检测系统**，融合力阈值、力变化率、能量指标和动量观测器：

```cpp
// 完整碰撞检测系统（C++，ros2_control 兼容）
#include <Eigen/Dense>
#include <deque>
#include <algorithm>

class CollisionDetectionSystem {
public:
    struct Config {
        double force_threshold = 30.0;       // 静态力阈值 [N]
        double torque_threshold = 5.0;       // 静态力矩阈值 [Nm]
        double force_rate_threshold = 500.0; // 力变化率阈值 [N/s]
        double energy_threshold = 2.0;       // 能量阈值 [J]
        int consecutive_count = 5;           // 连续触发计数
        double dt = 0.001;                   // 采样周期 [s]
        int history_size = 50;               // 历史窗口（50 ms @ 1 kHz）
    };

    enum class DetectionLevel {
        NONE,       // 无碰撞
        WARNING,    // 预警（力接近阈值）
        SOFT,       // 软碰撞（缓慢接触）
        HARD        // 硬碰撞（高速冲击）
    };

    struct DetectionResult {
        DetectionLevel level;
        Eigen::Vector3d collision_direction;  // 碰撞方向（单位向量）
        double collision_force;               // 碰撞力大小 [N]
        double force_rate;                    // 力变化率 [N/s]
        double collision_energy;              // 碰撞能量 [J]
    };

private:
    Config config_;
    int force_count_ = 0;
    int rate_count_ = 0;
    std::deque<Eigen::Matrix<double,6,1>> wrench_history_;
    double accumulated_energy_ = 0.0;
    Eigen::Matrix<double,6,1> prev_wrench_;
    bool initialized_ = false;

public:
    explicit CollisionDetectionSystem(const Config& cfg = Config())
        : config_(cfg) {}

    DetectionResult detect(const Eigen::Matrix<double,6,1>& wrench,
                           const Eigen::Matrix<double,6,1>& velocity) {
        DetectionResult result;
        result.level = DetectionLevel::NONE;

        // === Layer 1: 静态力阈值检测 ===
        double force_norm = wrench.head(3).norm();
        double torque_norm = wrench.tail(3).norm();

        bool force_exceeded = (force_norm > config_.force_threshold) ||
                              (torque_norm > config_.torque_threshold);

        if (force_exceeded) {
            force_count_++;
        } else {
            force_count_ = std::max(0, force_count_ - 1);  // 缓慢衰减
        }

        // === Layer 2: 力变化率检测（微分碰撞）===
        double force_rate = 0.0;
        if (initialized_) {
            Eigen::Vector3d df = wrench.head(3) - prev_wrench_.head(3);
            force_rate = df.norm() / config_.dt;

            if (force_rate > config_.force_rate_threshold) {
                rate_count_++;
            } else {
                rate_count_ = 0;
            }
        }

        // === Layer 3: 碰撞能量估计 ===
        // E_collision = ∫ F · v dt ≈ Σ F_k · v_k · Δt
        if (force_norm > config_.force_threshold * 0.5) {
            double power = wrench.head(3).dot(velocity.head(3));
            accumulated_energy_ += std::abs(power) * config_.dt;
        } else {
            accumulated_energy_ *= 0.95;  // 无碰撞时能量缓慢衰减
        }

        // === 综合判断 ===
        result.collision_force = force_norm;
        result.force_rate = force_rate;
        result.collision_energy = accumulated_energy_;
        if (force_norm > 1e-3) {
            result.collision_direction = wrench.head(3).normalized();
        } else {
            result.collision_direction = Eigen::Vector3d::Zero();
        }

        // 硬碰撞：力变化率超标（高速冲击特征）
        if (rate_count_ >= 2) {
            result.level = DetectionLevel::HARD;
        }
        // 软碰撞：力持续超标
        else if (force_count_ >= config_.consecutive_count) {
            result.level = DetectionLevel::SOFT;
        }
        // 预警：力接近阈值或能量积累
        else if (force_norm > config_.force_threshold * 0.7 ||
                 accumulated_energy_ > config_.energy_threshold * 0.5) {
            result.level = DetectionLevel::WARNING;
        }

        // 更新历史
        prev_wrench_ = wrench;
        initialized_ = true;
        wrench_history_.push_back(wrench);
        if (static_cast<int>(wrench_history_.size()) > config_.history_size) {
            wrench_history_.pop_front();
        }

        return result;
    }

    void reset() {
        force_count_ = 0;
        rate_count_ = 0;
        accumulated_energy_ = 0.0;
        wrench_history_.clear();
        initialized_ = false;
    }
};
```

**四层检测的设计逻辑**：

| 层级 | 检测指标 | 检测目标 | 响应延迟 |
|------|---------|---------|---------|
| Layer 1 | 力绝对值 | 准静态碰撞（慢速夹持） | 5 ms |
| Layer 2 | 力变化率 | 瞬态碰撞（高速冲击） | 2 ms |
| Layer 3 | 碰撞能量 | 累积伤害评估 | 10+ ms |
| 综合 | 多指标融合 | 分级响应 | 2-5 ms |

> **反事实推理**：如果只用力阈值检测（Layer 1），会遗漏什么？高速碰撞的特征是力变化率极高但持续时间极短（<5 ms）。如果力的峰值恰好略低于阈值但变化率超过 1000 N/s，单靠 Layer 1 会**漏检**——而这种碰撞的能量可能足以造成伤害。Layer 2 通过检测力的微分特征弥补了这个盲区。

### 5.3 基于动量观测器的碰撞检测（无 F/T 传感器）

> 回顾 F06 预告：De Luca 2005 的动量观测器可以在不需要力传感器的情况下估计外部力。Franka 的 `tau_ext_hat_filtered` 就是这个观测器的输出。

对于没有 F/T 传感器的机器人，可以用动量观测器进行碰撞检测：

```
广义动量：p = M(q)·dq
残差：    r = K_O·[p - ∫(τ + C^T·dq - g + r)dt - p(0)]
         ṙ = K_O·(τ_ext - r)    ← 一阶低通估计

碰撞检测：如果 ||r|| > threshold → 碰撞
笛卡尔力估计：F̂_ext = (J^T)^+ · r
```

**动量观测器的优缺点**：

| 优点 | 缺点 |
|------|------|
| 不需要额外传感器 | 需要精确的 $M(q), C(q,\dot{q}), g(q)$ |
| 整个机体的碰撞都能检测 | 模型误差导致假阳性/假阴性 |
| 实时性好 | 高速碰撞时延迟 20-50 ms |

### 5.4 三种安全停止策略

碰撞检测触发后，机器人应如何反应？ISO/TS 15066 和工程实践定义了三种策略：

**策略 1：Reflex（反射停止）**

```
检测到碰撞 → 立即停止所有关节电机 → 保持当前位置
```

最简单但可能不是最安全——持续施加碰撞力（准静态力）。

**策略 2：Retract（回缩）**

```
检测到碰撞 → 沿碰撞力方向后退预设距离（50 mm）→ 停止
```

比 reflex 更安全——主动减小接触力。但需要知道碰撞方向。

**策略 3：Comply（柔顺）**

```
检测到碰撞 → 切换到零力引导模式（K_d = 0, D_d 增大）→ 等待恢复
```

最安全——机器人变成完全被动的柔顺体。

```
安全策略选择决策树：
  ├── 准静态碰撞（低速，<0.25 m/s）
  │   → Comply（柔顺模式）
  ├── 瞬态碰撞（高速，>0.25 m/s）
  │   → Reflex + Retract（先停再退）
  └── 夹持碰撞（人被夹在机器人和环境之间）
      → 立即 Retract（远离碰撞方向）
      → 这是最危险的场景
```

### 5.5 人机协作手引导教（Lead-through）

手引导教（lead-through programming）是协作机器人最直观的编程方式——操作者直接用手推动机器人到目标位姿，机器人记录轨迹。

**导纳实现**：

```yaml
# lead-through 模式的 admittance_controller 配置
admittance:
  selected_axes: [true, true, true, true, true, true]
  mass: [3.0, 3.0, 3.0, 1.0, 1.0, 1.0]      # 小质量→轻盈手感
  damping_ratio: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 临界阻尼→松手即停
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # K=0→无回复力

gravity_compensation:
  frame:
    id: tool0
    external: false
  CoG:
    pos: [0.0, 0.0, 0.08]
    force: 9.81  # 1 kg 工具重量标量 [N]

# 完整 admittance_controller 参数中还需要保留坐标系配置：
control:
  frame:
    id: tool0        # 导纳律控制帧
    external: false
fixed_world_frame:
  frame:
    id: base_link    # 重力方向参考世界系
    external: false
```

**lead-through 的三个关键要求**：

1. **轻盈手感**：$M_d$ 要足够小（3-5 kg），推动不费力
2. **精确停止**：临界阻尼 $\zeta = 1.0$，松手后立即减速至零
3. **精确重力补偿**：工具重力补偿偏差 >0.5 N 就会导致机器人在垂直方向漂移

> **类比**：好的 lead-through 体验就像推一辆高端的超市手推车——轻盈、灵活、松手后平稳停止。差的 lead-through 像推一辆生锈的手推车——沉重、不灵活、方向偏移。重力补偿精度决定了"是否偏移"，$M_d$ 决定了"是否沉重"，$\zeta$ 决定了"是否平稳停止"。

#### Lead-through 轨迹记录与回放系统

Lead-through 不只是"推着机器人动"——工程上需要**记录**操作者的示教轨迹，然后让机器人**自主回放**。这涉及轨迹采集、预处理、存储和回放四个环节。

**轨迹记录节点**（ROS2 Python）：

```python
# lead_through_recorder.py — 轨迹记录节点
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import json
import numpy as np
from scipy.interpolate import CubicSpline

class LeadThroughRecorder(Node):
    def __init__(self):
        super().__init__('lead_through_recorder')
        # 订阅末端位姿和关节状态
        self.pose_sub = self.create_subscription(
            PoseStamped, '/admittance_controller/current_pose',
            self.pose_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # 录制控制服务
        self.start_srv = self.create_service(
            Trigger, '~/start_recording', self.start_recording)
        self.stop_srv = self.create_service(
            Trigger, '~/stop_recording', self.stop_recording)

        self.recording = False
        self.trajectory = {
            'timestamps': [],    # 时间戳 [s]
            'poses': [],         # [x,y,z, qx,qy,qz,qw]
            'joint_positions': [],  # [q1,...,q6]
            'velocities': []     # [vx,vy,vz]
        }
        self.t0 = None

    def start_recording(self, request, response):
        self.recording = True
        self.t0 = self.get_clock().now()
        self.trajectory = {k: [] for k in self.trajectory}
        self.get_logger().info('Recording started')
        response.success = True
        return response

    def stop_recording(self, request, response):
        self.recording = False
        # 预处理轨迹
        processed = self.preprocess_trajectory()
        # 保存为 JSON
        filename = f'trajectory_{self.get_clock().now().seconds_nanoseconds()[0]}.json'
        with open(filename, 'w') as f:
            json.dump(processed, f, indent=2)
        self.get_logger().info(f'Saved trajectory: {filename}, '
                               f'{len(processed["timestamps"])} points')
        response.success = True
        response.message = filename
        return response

    def pose_callback(self, msg):
        if not self.recording:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self.trajectory['timestamps'].append(t)
        self.trajectory['poses'].append(pose)

    def preprocess_trajectory(self):
        """预处理：降采样 + 平滑 + 去除静止段"""
        ts = np.array(self.trajectory['timestamps'])
        poses = np.array(self.trajectory['poses'])

        if len(ts) < 10:
            return self.trajectory

        # Step 1: 计算末端速度，去除静止段（v < 0.5 mm/s）
        dp = np.diff(poses[:, :3], axis=0)
        dt = np.diff(ts)
        vel = np.linalg.norm(dp / dt[:, None], axis=1)
        # 找到运动段的起止
        moving = vel > 0.0005  # 0.5 mm/s 阈值
        if not np.any(moving):
            return self.trajectory
        start_idx = np.argmax(moving)
        end_idx = len(moving) - np.argmax(moving[::-1])

        # Step 2: 截取运动段
        ts = ts[start_idx:end_idx+1]
        poses = poses[start_idx:end_idx+1]
        ts = ts - ts[0]  # 时间归零

        # Step 3: 降采样到固定间隔（10 Hz 足够回放）
        t_uniform = np.arange(0, ts[-1], 0.1)  # 100 ms 间隔
        poses_interp = np.zeros((len(t_uniform), 7))
        for i in range(3):  # 位置用三次样条插值
            cs = CubicSpline(ts, poses[:, i])
            poses_interp[:, i] = cs(t_uniform)
        for i in range(3, 7):  # 四元数线性插值（slerp 更好但简化）
            poses_interp[:, i] = np.interp(t_uniform, ts, poses[:, i])
        # 四元数归一化
        qnorm = np.linalg.norm(poses_interp[:, 3:7], axis=1, keepdims=True)
        poses_interp[:, 3:7] /= qnorm

        return {
            'timestamps': t_uniform.tolist(),
            'poses': poses_interp.tolist(),
            'sample_rate_hz': 10,
            'duration_s': float(ts[-1])
        }
```

**轨迹回放节点**：

```python
# lead_through_player.py — 轨迹回放节点
class LeadThroughPlayer(Node):
    def __init__(self):
        super().__init__('lead_through_player')
        self.pose_pub = self.create_publisher(
            PoseStamped, '/admittance_controller/target_pose', 10)
        self.timer = None
        self.trajectory = None
        self.play_idx = 0
        self.play_srv = self.create_service(
            Trigger, '~/play', self.play_callback)

    def load_trajectory(self, filename):
        with open(filename) as f:
            self.trajectory = json.load(f)
        self.get_logger().info(
            f'Loaded: {len(self.trajectory["timestamps"])} points, '
            f'{self.trajectory["duration_s"]:.1f}s')

    def play_callback(self, request, response):
        if self.trajectory is None:
            response.success = False
            response.message = 'No trajectory loaded'
            return response
        self.play_idx = 0
        dt = 1.0 / self.trajectory['sample_rate_hz']
        self.timer = self.create_timer(dt, self.publish_next_pose)
        response.success = True
        return response

    def publish_next_pose(self):
        if self.play_idx >= len(self.trajectory['timestamps']):
            self.timer.cancel()
            self.get_logger().info('Playback complete')
            return
        pose_data = self.trajectory['poses'][self.play_idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = pose_data[0]
        msg.pose.position.y = pose_data[1]
        msg.pose.position.z = pose_data[2]
        msg.pose.orientation.x = pose_data[3]
        msg.pose.orientation.y = pose_data[4]
        msg.pose.orientation.z = pose_data[5]
        msg.pose.orientation.w = pose_data[6]
        self.pose_pub.publish(msg)
        self.play_idx += 1
```

**录制→回放的完整工作流**：

```bash
# 1. 启动导纳控制器（lead-through 模式）
ros2 launch ur_bringup ur5e_bringup.launch.py \
  admittance_config:=lead_through_config.yaml

# 2. 启动录制节点
ros2 run force_control lead_through_recorder

# 3. 开始录制
ros2 service call /lead_through_recorder/start_recording std_srvs/srv/Trigger

# 4. 操作者手动示教（拖动机器人走完整轨迹）

# 5. 停止录制
ros2 service call /lead_through_recorder/stop_recording std_srvs/srv/Trigger

# 6. 切换导纳控制器到跟踪模式（恢复刚度）
# 修改 admittance 参数：stiffness 从 0 恢复到工作值

# 7. 回放轨迹
ros2 run force_control lead_through_player --ros-args -p trajectory_file:=xxx.json
ros2 service call /lead_through_player/play std_srvs/srv/Trigger
```

#### 导纳控制参数在线调节——GUI 架构设计

生产环境中，导纳参数需要根据任务动态调节，而非每次重启节点。在 ROS2 中，`dynamic_reconfigure`（ROS1 工具）已被**原生参数机制**取代——每个节点的参数可通过 `ros2 param set` 命令或 DDS 参数服务动态修改，并通过 `on_set_parameters_callback` 在节点内响应。本节描述一个基于 ROS2 参数机制的在线调参架构。

**架构层次**：

```
┌── rqt GUI（操作者界面）────────────────────────────┐
│  Slider: M_d [0.5-50 kg]                          │
│  Slider: damping_ratio [0.5-5.0]                  │
│  Slider: K_d [0-5000 N/m]                         │
│  选择框: 预设模式 (lead-through / polish / track)  │
│  紧急按钮: 切换到零力引导                           │
│  实时曲线: F_ext, delta_x, tank_energy             │
└────────────────── DDS 参数服务 ──────────────────┘
                     ↕
┌── admittance_controller（1 kHz 实时循环）──────────┐
│  on_parameter_event_callback():                    │
│    读取新参数 → 指数平滑过渡 → 更新控制律           │
│    过渡时间 τ_smooth = 0.2 s（防止参数突变）        │
└──────────────────────────────────────────────────┘
```

**参数过渡的指数平滑**：

直接跳变导纳参数会产生力矩突变。必须用指数平滑：

$$K_{d,actual}(k) = \alpha K_{d,target} + (1-\alpha) K_{d,actual}(k-1)$$

$$\alpha = 1 - e^{-\Delta t / \tau_{smooth}}$$

其中 $\tau_{smooth} = 0.2$ s 是平滑时间常数。这保证了从任意旧参数到新参数的过渡在约 $5\tau_{smooth} = 1$ s 内完成，且过渡期间力矩连续。

**预设模式的快速切换**：

| 模式 | $M_d$ [kg] | $\zeta$ | $K_d$ [N/m] | 典型场景 |
|------|:---:|:---:|:---:|---------|
| lead-through | 3 | 1.0 | 0 | 手动示教 |
| polish | 5 | 1.5 | 200 (z: 50) | 打磨/擦拭 |
| track | 8 | 2.0 | 500 | 轨迹跟踪+柔顺 |
| emergency | 3 | 2.0 | 0 | 碰撞后零力 |

#### ros2_control 导纳配置实战——完整 launch + YAML

**UR5e + ATI Gamma 导纳控制 launch 文件（直接写硬件命令模式）**：

```python
# ur5e_admittance.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    admittance_params = PathJoinSubstitution([
        FindPackageShare('force_control'), 'config', 'admittance_params.yaml'])

    # ros2_control 控制器不作为独立 Node 启动，而是通过 controller_manager 的 spawner 加载
    # controller_manager 本身随 robot_state_publisher / hardware_interface 一起启动
    # 注意: 控制器的参数（joints、command_interfaces 等）应在 YAML 配置文件中定义，
    # 通过 --param-file 传入。spawner 会通过 controller_manager 的服务接口
    # 加载、配置和激活控制器，确保正确的资源分配和生命周期管理。
    return LaunchDescription([
        # 1. 可选：加载力传感器广播用于监控；admittance_controller 本身读取 F/T state interfaces
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ft_sensor_broadcaster',
                       '--param-file', admittance_params],
        ),
        # 2. 通过 spawner 加载导纳控制器
        # 本模板让 admittance_controller 直接声明并写入硬件 command_interfaces。
        # 不在这里同时激活 JointTrajectoryController，否则两者会抢同一组关节命令资源。
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['admittance_controller',
                       '--param-file', admittance_params],
        ),
    ])
```

若需要 `JointTrajectoryController` 生成路径参考，应把它配置成导纳控制器的上游参考源或使用目标发行版支持的 controller chaining 方案；不要让 JTC 和 `admittance_controller` 同时 claim 同一批硬件 `joint/position` command interfaces。

**YAML 配置——三个工业场景模板**：

```yaml
# admittance_params.yaml — 场景切换模板
admittance_controller:
  ros__parameters:
    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
             wrist_1_joint, wrist_2_joint, wrist_3_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    chainable_command_interfaces: [position, velocity]
    # command_joints 仅在命令目标不是同名硬件关节时设置

    ft_sensor:
      name: ft_sensor                 # F/T semantic component 的接口前缀
      frame:
        id: tool0
        external: false
      filter_coefficient: 0.01     # 中等滤波

    control:
      frame:
        id: tool0                  # 导纳律计算、选择性轴和位姿修正所在坐标系
        external: false

    fixed_world_frame:
      frame:
        id: base_link              # 重力方向参考世界系
        external: false

    kinematics:
      plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
      plugin_package: kinematics_interface
      base: base_link
      tip: tool0
      alpha: 0.0005

    # --- 默认：打磨模式 ---
    admittance:
      selected_axes: [false, false, true, false, false, false]  # 仅 z 轴
      mass: [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
      damping_ratio: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
      stiffness: [500.0, 500.0, 100.0, 50.0, 50.0, 50.0]

    gravity_compensation:
      frame:
        id: tool0
        external: false
      CoG:
        pos: [0.0, 0.0, 0.06]
        force: 7.36                 # 0.75 kg 打磨工具重量 [N]
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：碰撞检测阈值设置过低导致误触发**
>
> 错误做法：设置 `force_threshold = 5.0` N
>
> 现象：导纳控制器在正常接触操作（如轻触工件）时频繁触发碰撞停止
>
> 正确做法：碰撞阈值应大于正常操作力的 2-3 倍。如果正常操作力 10 N，阈值应设为 25-30 N

> 💡 **概念误区：认为 lead-through 只需要"零刚度"**
>
> 新手想法："把 K 设为 0 就实现了手引导"
>
> 实际上：K=0 只是第一步。还需要：(1) 精确的工具重力补偿（否则垂直漂移）；(2) 合适的虚拟质量（太大则推动费力）；(3) 临界阻尼（否则松手后末端继续运动）；(4) F/T 传感器噪声足够低（否则末端在静止时抖动）

### 练习

1. ⭐ **碰撞检测实现**：在 MuJoCo UR5e 中实现基于力阈值的碰撞检测。让机器人以 0.1 m/s 接近桌面，检测接触瞬间。记录碰撞检测延迟（从接触到检测触发的时间）。
2. ⭐⭐ **三种停止策略对比**：在 MuJoCo 中实现 Reflex/Retract/Comply 三种停止策略。让机器人以 0.2 m/s 碰撞弹性物体（$K_e = 500$ N/m），记录三种策略下的最大接触力和碰后行为。
3. ⭐⭐ **lead-through 调参**：在 UR5e（仿真或真机）上配置 admittance_controller 实现 lead-through。尝试不同的 $M_d$（1, 5, 20 kg）和 $\zeta$（0.5, 1.0, 2.0），评价轻盈度/停止性/稳定性。

---

## 6. 柔顺控制统一视角与工业案例 ⭐⭐

### 6.1 柔顺设计空间

**柔顺控制不是一个"算法"，而是一个"设计维度"**。每个机器人系统的柔顺行为由多个层级共同决定：

```
┌────────────────────────────────────────────────────────┐
│         柔顺来源谱系（从被动到主动到学习）                │
├──────────┬──────────────┬──────────────┬───────────────┤
│ 机械被动  │ 控制主动      │ 任务自适应    │ 学习自主       │
│          │              │              │               │
│ RCC 装置  │ 阻抗控制      │ 变阻抗（FSM） │ RL+阻抗       │
│ SEA 弹簧  │ 导纳控制      │ 变阻抗（KB）  │ Diffusion+柔顺│
│ 软体材料  │ FDCC         │ 能量罐       │ 残差 RL+基层   │
│ 气动阻尼  │ 操作空间控制  │ 自适应阻抗    │ 模仿学习+柔顺  │
│          │              │              │               │
│ 延迟: 0  │ 延迟: ~1ms   │ 延迟: ~1ms   │ 延迟: 20-100ms│
│ 可调: 否 │ 可调: 参数级  │ 可调: 在线   │ 可调: 任务级   │
│ 安全: 天然│ 安全: 需保证  │ 安全: 需验证  │ 安全: 待研究   │
└──────────┴──────────────┴──────────────┴───────────────┘
```

### 6.2 工程柔顺决策树

```
你的机器人有力矩接口吗？
├── 是（Franka/iiwa/DLR）→ 用阻抗控制（F04）
│   ├── 需要学习策略集成？→ CRISP
│   ├── 需要 rqt 在线调参？→ matthias-mayr CIC
│   └── 纯位置跟踪+柔顺？→ 标准阻抗
│
├── 否（UR/Fanuc/ABB）→ 用导纳控制（本章）
│   ├── 有 F/T 传感器？→ ros2_controllers admittance
│   ├── 无传感器但要柔顺？→ FZI FDCC
│   └── 需要奇异安全？→ FZI FDCC（J^T 天然良态）
│
└── 浮动基座（腿足/人形）→ 用 WBC + 阻抗任务
    ├── mc_rtc ImpedanceTask
    ├── TSID + ContactForce
    └── WBIC + MPC 力参考
```

### 6.3 工业场景中的柔顺配方

| 任务 | $K_{trans}$ [N/m] | $K_{rot}$ [Nm/rad] | $\zeta$ | 力控轴 | 备注 |
|------|:---------:|:-----------:|:---:|:------:|------|
| 零力引导 | 0-10 | 0-5 | 1.0 | 全轴 | $M_d = 5$-10 kg |
| 精密擦拭 | 500-1000 | 20-50 | 0.8 | z | 法向 5-10 N |
| 工业打磨 | 1000-2000 | 50-100 | 1.0 | z | 法向 10-30 N |
| PCB 插件 | 200-500 | 10-30 | 0.7-1.0 | z | SERL: K=300 |
| Peg-in-hole | 50-2000 | 5-200 | 0.8-1.0 | 四阶段 | F06 详解 |
| 螺丝拧紧 | 1000 | 5-20 (Rz 软) | 1.0 | Rz 力矩 | 扭矩监控 |
| 包装码垛 | 2000-5000 | 100-200 | 1.0 | z 柔顺 | 高刚度+碰撞检测 |

### 6.4 工业案例：协作装配力控

**案例：UR10e + ATI Gamma 协作装配汽车车门密封条**

```
任务描述：
  - 将柔性密封条压入车门沟槽
  - 沟槽位置有 ±2 mm 偏差（不同车辆个体差异）
  - 需要 20-30 N 法向力保证密封条完全压入
  - 操作者在旁边监督和辅助

控制架构：
  JointTrajectoryController 或轨迹节点（路径参考源）
    → admittance_controller（z 轴导纳，叠加法向柔顺）
    + 碰撞检测（安全层）

参数配置：
  selected_axes: [false, false, true, false, false, false]
  mass: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
  damping_ratio: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
  stiffness: [500.0, 500.0, 100.0, 50.0, 50.0, 50.0]

力参考：f_ref_z = -25 N（向下压入）
碰撞阈值：50 N（正常操作力的 2 倍）
安全策略：Comply（碰撞后切换到零力引导）
```

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：认为一种柔顺方案能解决所有问题**
>
> 新手想法："学了导纳控制就够了，所有力控任务都用导纳"
>
> 实际上：不同任务需要不同层次的柔顺。简单的零力引导用导纳足够；精密装配可能需要变阻抗+状态机；学习型操作任务需要双层架构。关键是理解**柔顺设计空间**，然后根据任务约束选择合适的组合

### 练习

1. ⭐ **场景选型**：为以下五个工业场景各选择一种柔顺方案，说明理由：(a) 汽车焊接机器人防碰撞 (b) 协作臂 PCB 检测 (c) 未知形状物体抓取 (d) 手术机器人力反馈 (e) 包装线码垛
2. ⭐⭐ **柔顺椭球可视化**：在 MuJoCo Franka 中设置三组刚度矩阵——各向同性、轴选择性和耦合椭球。从 x/y/z 方向施力，用 matplotlib 画出柔顺椭球。
3. ⭐⭐⭐ **跨章综合题**：结合 F01-F05 的知识，为一个完整的协作装配场景设计力控系统。场景：UR5e + ATI Gamma，将 M8 螺栓拧入铝板。需要：(1) 接近阶段位置控制；(2) 接触检测；(3) 导纳下的螺旋搜索对准；(4) 恒力矩拧紧。画出状态机和每阶段的 YAML 参数。

---

### 导纳控制在协作机器人上的工业实践

导纳控制是工业协作机器人力柔顺的**主流实现方式**。以下整理了主要协作臂平台的导纳控制工程实践，帮助读者理解"教科书理论"在真实产品中的具体形态。

**Universal Robots（UR3e/5e/10e/16e/20e/30e）的力控实现**：

UR 的 `force_mode` 是一个内置的 6 轴选择性导纳控制器。用户通过 URScript 或 RTDE 接口配置每个轴的柔顺/刚性模式。其内部实现是：F/T 传感器读数（TCP 处的 6D 力/力矩）→ 低通滤波 → 导纳积分 → 叠加到关节位置参考。UR 的独特设计是**传感器集成在工具端**（不是关节处），因此工具重力补偿的精度直接影响零力引导的质量。UR 提供了内置的 payload 辨识程序——安装新工具后**必须运行**，否则手引导时会有明显的重力偏差。工程中常见的经验值：$M_d = 2$-$10$ kg，$D_d = 50$-$200$ Ns/m，$K_d = 0$（零力引导）或 $K_d = 100$-$500$ N/m（弹性回中心）。UR 的导纳环工作在 500 Hz，因此对于 $K_e > 5 \times 10^4$ N/m 的刚性环境（如金属-金属接触），需要降低 $K_d$ 并增大 $D_d$ 以维持稳定。

**Franka Emika 的导纳模式**：

虽然 Franka 以力矩接口著称（F04 的主题），但它也支持导纳模式——通过 `franka_ros2` 的 `admittance_controller` 或自定义的导纳控制器。Franka 的独特优势是**关节力矩传感器提供的高精度外力估计**（`tau_ext_hat_filtered`），可以替代外部 F/T 传感器。但需注意：`tau_ext_hat_filtered` 的精度依赖于出厂标定的动力学模型，安装未标定的工具后精度会显著下降。Franka 在导纳模式下的带宽受限于 `cartesian_impedance_controller` 的内环（~200 Hz），但对大多数协作任务已足够。

**工业导纳调参的"黄金法则"**：

经过数百个工业部署案例的经验积累，导纳参数调整有以下经验法则：

| 场景 | $M_d$ [kg] | $\zeta$ | $K_d$ [N/m] | 关键考量 |
|------|:---:|:---:|:---:|---------|
| 零力引导（手引导教） | 1-5 | 2-3 | 0 | $M_d$ 小→轻盈；$K_d=0$→松手停 |
| 恒力抛光 | 5-10 | 2-3 | 50-200 | 低 $K_d$ 适应表面起伏 |
| 弹性装配 | 3-8 | 1-2 | 200-1000 | $K_d$ 提供回中心力 |
| 碰撞后退让 | 1-3 | 1 | 0-50 | 快速响应，低惯量感 |

> **本质洞察**：工业导纳调参不是"计算出最优参数"，而是"从保守参数开始，逐步放宽"。先用大 $M_d$（稳定但迟钝）、大 $\zeta$（过阻尼）、小 $K_d$（柔顺），确认稳定后再逐步降低 $M_d$（提高响应速度）、降低 $\zeta$（加快收敛）、提高 $K_d$（提高位置精度）。每次只调一个参数，观察 30 秒后再动下一个。

---

## 本章小结

| 知识点 | 难度 | 核心结论 | 工程应用 |
|--------|------|---------|---------|
| 导纳因果链 | ⭐ | 力→导纳律→IK→位置命令 | 理解控制架构 |
| ros2_controllers 配置 | ⭐ | YAML 参数的物理含义 | 日常导纳配置 |
| 导纳离散实现 | ⭐⭐ | 源码为显式笛卡尔加速度 + 关节速度先行积分；接触/阻尼下仍需稳定性分析 | 数值稳定性 |
| FZI FDCC | ⭐⭐ | $J^T$ 绕开 IK 奇异 | 奇异附近柔顺 |
| 导纳稳定性 | ⭐⭐⭐ | $K_e < D_d/(2\Delta t)$ | 参数选择约束 |
| 碰撞检测 | ⭐⭐ | 阈值+连续计数+三种停止策略 | 安全底线 |
| Lead-through | ⭐ | K=0 + 精确重力补偿 + 临界阻尼 | 协作编程 |
| 柔顺统一视角 | ⭐⭐ | 被动→主动→自适应→学习 | 系统设计 |

---

## 累积项目：本章新增模块

**项目：从零构建力控机械臂系统**

| 章节 | 新增模块 | 功能 |
|------|---------|------|
| F01 | 概念模型 | 阻抗/导纳二分法 |
| F03 | 算法库 | 四大经典力控算法 |
| F04 | 阻抗控制器 | libfranka 笛卡尔阻抗 |
| **F05** | **导纳控制器** | **ros2_controllers 导纳 + FZI FDCC + 碰撞检测 + lead-through** |

本章实现的模块：
1. ros2_controllers admittance_controller 配置与调参
2. FZI cartesian_compliance_controller 配置
3. F/T 传感器数据预处理（滤波 + 重力补偿）
4. 碰撞检测与三种安全停止策略
5. Lead-through 零力引导模式

下一章（F06）将添加：变阻抗模块（能量罐 + KB 条件）+ 碰撞安全计算。

---

## 延伸阅读

| 材料 | 类型 | 难度 | 核心内容 |
|------|------|------|---------|
| ros2_controllers admittance_controller 官方文档 (control.ros.org) | 文档 | ⭐ | 参数说明最全 |
| Scherzinger et al. 2017, "Forward Dynamics Compliance Controller" (IROS) | 论文 | ⭐⭐ | FZI FDCC 原始论文 |
| Scherzinger 2019, ROSCon Macau 演讲 | 演讲 | ⭐ | FDCC 的 ROS 实现和工程经验 |
| Aertbelien et al. 2025, "Simplifying ROS2 Controllers" (arXiv 2601.08514) | 论文 | ⭐⭐ | 模块化参考生成架构 |
| Ott 2008, "Cartesian Impedance Control of Redundant and Flexible-Joint Robots" | 专著 | ⭐⭐⭐ | 导纳/阻抗控制的稳定性理论 |
| Haddadin & Croft 2016, "Physical Human-Robot Interaction" (Springer Handbook Ch.69) | 教材 | ⭐⭐ | ISO 15066 权威解读 |
| FZI cartesian_controllers GitHub 仓库 | 代码 | ⭐⭐ | FDCC 完整实现 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 末端持续向下漂移 | 工具重力补偿不准 | 1. 检查 CoG.force 值 2. 水平姿态下测 z 分量 3. 称重法精确测工具质量 | F05§1.4 |
| 接触刚性环境时振荡 | $K_d/M_d$ 过大超出带宽 | 1. 降低 $K_d$ 2. 增大 $M_d$ 3. 增大 $D_d$ 4. 检查内环带宽 | F05§4 |
| 力传感器读数抖动 | filter_coefficient 过大 | 1. 减小 filter_coefficient 到 0.005 2. 检查电缆屏蔽 3. 增大 $M_d$ | F05§1.4 |
| FZI FDCC 位姿跟踪振荡 | error_scale 过大 | 1. 减小 error_scale 到 0.3 2. 增大 solver.iterations | F05§3.4 |
| lead-through 末端飘不停 | 阻尼不足或 K 未设 0 | 1. 确认 K=0 2. 确认 $\zeta$=1.0 3. 增大 $D_d$ | F05§5.5 |
| admittance 与 JTC 级联不工作 | reference interfaces / command_interfaces 方向配置错误，或两个控制器抢同一硬件资源 | 1. 确认 `chainable_command_interfaces` 是导纳控制器导出的参考输入 2. 确认只有一个控制器 claim 硬件 command interfaces 3. 检查 chained mode 与加载顺序 | F05§2.2 |
| IK 在奇异附近跳变 | admittance 使用 KDL IK | 1. 增大 alpha 正则化 2. 切换到 FZI FDCC 3. 约束工作空间 | F05§3 |
