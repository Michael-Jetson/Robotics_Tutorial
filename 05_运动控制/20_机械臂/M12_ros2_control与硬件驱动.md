> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M12 ros2_control 与硬件驱动

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回 M11 / `02_基础/50_ROS2工程化` 复习）

1. 什么是 `SCHED_FIFO` 调度策略？它与 `SCHED_OTHER` 的延迟差异量级是多少？（M11）
2. `pluginlib` 的动态加载机制是什么？它与 `dlopen` 的关系是什么？（`02_基础/50_ROS2工程化/40_硬件集成与RL部署`）
3. ROS2 Lifecycle Node 的四个主要状态是什么？状态转换回调函数的命名规则是什么？（`02_基础/50_ROS2工程化/40_硬件集成与RL部署`）
4. 什么是 RAII？在实时控制循环中为什么禁止使用 `new/delete`？（`02_基础/10_C++语言核心/40_RAII与智能指针`、M11）
5. URDF 的 `<ros2_control>` 标签中 `<hardware>` 和 `<joint>` 的语义是什么？（P01）

## 本章目标

学完本章后，你能够：
1. **深入理解** ros2_control 的四大核心组件及其协作机制，能画出完整的数据流图
2. **独立编写** 一个符合实时约束的 `SystemInterface` 硬件驱动插件
3. **熟练配置** JointTrajectoryController / ForwardCommandController / AdmittanceController 三大机械臂控制器
4. **掌握** 从 RL 策略导出到 ros2_control 执行的完整部署流水线

---

## M12.1 ros2_control 架构全景 ⭐⭐

### 动机 ⭐⭐

假设你有一个 MoveIt2 规划好的关节轨迹，需要在真实的 Franka Panda 上执行。问题来了：MoveIt2 的规划算法不应该知道「这是 Franka」还是「这是 UR5e」还是「这是 Gazebo 仿真」——如果每换一种硬件就要改规划代码，维护成本将指数爆炸。

同时，你可能还想在同一台机器人上切换控制模式：MoveIt2 规划执行时用轨迹跟踪控制器，RL 策略推理时用直接位置转发控制器，力控交互时用导纳控制器。如果每种控制器都要自己实现硬件通信，代码重复将不可收拾。

### 如果不用 ros2_control 会怎样 ⭐⭐

在 ros2_control 出现之前（ROS1 早期），每个硬件厂商自己实现 ROS 驱动节点：
- Franka 有自己的 `franka_ros`，发布 `/joint_states`，订阅 `/joint_commands`
- UR 有自己的 `ur_modern_driver`，发布 `/joint_states`，但命令格式不同
- Gazebo 有自己的 `gazebo_ros_control`，接口又不一样

结果是：
1. **控制器代码不可复用**——为 Franka 写的 PD 控制器无法直接用在 UR 上
2. **通信延迟不可控**——所有数据经过 ROS topic，延迟 1-5ms，在 1kHz 控制循环中占了一半以上
3. **安全性无保障**——多个节点可以同时发送命令到同一关节，产生冲突
4. **接口不统一**——每换一种硬件就要改控制器代码中的消息类型和话题名

> **本质洞察**：ros2_control 的核心贡献不是某个具体算法，而是一个**抽象层**——它把「控制算法想读/写什么」和「硬件怎么读/写」彻底解耦。这类似于操作系统中设备驱动的角色：应用程序调用 `read()/write()`，不需要知道底层是 SSD 还是 HDD。

### 历史背景 ⭐

ros2_control 的前身是 ROS1 的 `ros_control`（2013 年由 Adolfo Rodriguez Tsouroukdissian 在 PAL Robotics 发起）。ROS1 版本已经引入了 Controller Manager 和 Hardware Interface 的分离，但存在几个关键问题：
- 线程模型不灵活（所有控制器在同一线程）
- 缺乏生命周期管理（硬件不能安全地从错误中恢复）
- 不支持链式控制器（一个控制器的输出不能直接作为另一个控制器的输入）

ROS2 版本（2020 年由 PickNik Robotics / Bence Magyar 等人主导重写）解决了这些问题，并引入了 Chainable Controllers、generate_parameter_library、更精细的错误处理等现代化设计。

### 四大核心组件 ⭐⭐

ros2_control 的架构由四个核心组件构成，它们通过明确的接口协作：

```
┌─────────────────────────────────────────────────────────┐
│                   Controller Manager                     │
│  (编排一切: 加载组件、管理生命周期、执行 RT 主循环)        │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │ Controller A  │  │ Controller B  │  │ Controller C  │  │
│  │ (JTC)        │  │ (Forward)    │  │ (Admittance) │   │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘   │
│         │read              │read              │read       │
│         │state             │state             │state      │
│         │                  │                  │           │
│         │write             │write             │write      │
│         │command           │command           │command    │
│  ┌──────▼──────────────────▼──────────────────▼───────┐  │
│  │              Resource Manager                       │  │
│  │  (管理接口的独占/共享, 调用硬件 read/write)          │  │
│  │                                                     │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │  │
│  │  │  HW Iface A  │  │  HW Iface B  │  │ HW Iface C │  │  │
│  │  │  (Franka)    │  │  (Gripper)   │  │ (FT Sensor)│  │  │
│  │  └─────────────┘  └─────────────┘  └────────────┘  │  │
│  └─────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

**1. Controller Manager**（编排者）

Controller Manager 是整个系统的入口点。它的职责包括：
- **加载**：通过 pluginlib 动态加载控制器和硬件插件（dlopen .so 文件）
- **生命周期管理**：驱动所有组件经过 Unconfigured → Inactive → Active 状态转换
- **RT 主循环执行**：在实时线程中以固定频率（通常 500-1000 Hz）执行 `read() → update() → write()` 循环
- **控制器切换**：原子地激活/停用控制器，保证切换过程不会导致不一致的命令

回顾 M11：我们在 M11 中学习了实时线程的创建（`SCHED_FIFO`、`mlockall`、CPU 绑定）。Controller Manager 的 RT 主循环正是运行在这样的实时线程上。`ros2_control_node` 的 `main()` 函数中，主循环调用 `controller_manager->read(time, period)`、对每个 active controller 调用 `update(time, period)`、最后调用 `controller_manager->write(time, period)`——整个过程必须在一个周期内完成（1ms @1kHz）。

**2. Resource Manager**（资源调度者）

Resource Manager 是 Controller Manager 和 Hardware Interface 之间的中间层。它的核心职责是**管理接口的独占与共享**：

- **State interfaces**（只读）：**共享的**——多个控制器可以同时读取同一个关节的位置/速度/力矩。例如 JointStateBroadcaster 和 JointTrajectoryController 都需要读取关节位置
- **Command interfaces**（可写）：**独占的**——同一时刻只有一个控制器可以拥有某个关节的 command interface。这防止了两个控制器同时给同一关节发送矛盾的命令

这个独占/共享设计是跨领域类比中**读写锁（shared_mutex）**的完美映射：多读者单写者。但与 `shared_mutex` 不同的是，ros2_control 的独占是在控制器激活时一次性声明的，而非每次访问都加锁——这消除了锁的运行时开销，满足实时约束。

**3. Hardware Interfaces**（你的驱动代码）

Hardware Interface 是你与物理硬件通信的代码。它实现三个关键方法：
- `on_init()` / `on_configure()` / `on_activate()`：生命周期回调，`on_init()` / `on_configure()` 可做内存预分配和资源初始化，`on_activate()` 使能电机
- `read()`：从硬件读取传感器数据（编码器位置、速度、力矩），写入内部 state 缓冲区
- `write()`：将 command 缓冲区的数据发送给硬件

`read()`、controller `update()` 和 `write()` 都运行在 RT 热路径上，必须满足 M11 中讲的所有实时约束：无 malloc、无 mutex 竞争、无系统调用阻塞、无异常抛出。

**4. Controllers**（算法代码）

Controller 实现控制算法。它的 `update()` 方法在每个 RT 循环中被调用：
- 从 state interfaces 读取当前状态（关节位置、速度等）
- 根据目标（轨迹点、力矩指令、导纳目标等）计算控制量
- 将控制量写入 command interfaces

### 主循环时序 ⭐⭐

```
时间 ────────────────────────────────────────────────────────►
     │◄──── 一个周期 T (1ms @1kHz) ────►│
     │                                   │
     │ read()    update()     write()    │
     │ ┌─────┐  ┌────────┐  ┌──────┐   │
     │ │ HW  │  │ Ctrl A │  │ HW   │   │
     │ │read │→│ update │→│write │   │
     │ └─────┘  │ Ctrl B │  └──────┘   │
     │          │ update │              │
     │          └────────┘              │
     │                                   │
     ▼                                   ▼
  t=0                                 t=T
```

关键约束：`read() + Σ update() + write()` 的总耗时必须小于周期 T。在 1kHz 控制频率下，T = 1ms。典型的 read/write 各需 50-200μs（取决于通信协议），控制器 update 通常 10-100μs。

| 组件 | 典型耗时 | 瓶颈来源 |
|------|---------|---------|
| `read()` | 50-200μs | EtherCAT/USB 通信延迟 |
| `update()` (JTC) | 10-50μs | 样条插值 + PID 计算 |
| `update()` (Admittance) | 50-100μs | 质量-弹簧-阻尼器模型 + FK |
| `write()` | 50-200μs | 命令帧发送 |
| **总计** | 160-550μs | 留约 450-840μs 余量 |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：在 read()/write() 中使用 ROS 日志
   错误做法：RCLCPP_INFO(get_logger(), "Position: %f", hw_positions_[0]);
   现象：第一次调用时分配日志缓冲区(malloc)，偶发延迟 50-500μs
   根本原因：RCLCPP_INFO 内部可能触发堆分配和线程同步
   正确做法：使用 RT-safe 日志工具（如 rclcpp_lifecycle 的 throttled 日志）
            或仅在非 RT 路径（on_configure 等）中打印
   自检方法：用 cyclictest 对比加日志前后的最大延迟
```

```
💡 概念误区：认为 state interface 和 command interface 是 ROS topic
   新手想法："state interface 就是 /joint_states topic 的数据源"
   实际上：State/command interfaces 是进程内的共享内存指针，
          不经过 DDS 通信。JointStateBroadcaster 是一个控制器，
          它从 state interfaces 读数据，然后发布到 /joint_states topic。
          直接从 state interface 读取比订阅 topic 快 1000 倍以上。
   为什么重要：理解这一点才能理解 ros2_control 的实时性能——
             RT 循环内部完全是进程内数据传递，零序列化开销。
```

```
🧠 思维陷阱：认为"控制器越多越好，每种行为写一个"
   新手想法："关节控制用一个控制器，力矩限制用一个，安全检查用一个"
   实际上：每个 active 控制器的 update() 都在 RT 循环中串行执行。
          10 个控制器 × 50μs/个 = 500μs，已经占满 1kHz 周期的一半。
          而且 command interface 独占意味着同一关节不能被两个控制器同时写入。
   正确思维：控制器数量要精简。复杂逻辑放在一个控制器内部实现，
            或使用 Chainable Controllers 将多个控制器的输入输出串联。
```

### 练习

1. **[A 型·概念图]** 画出 ros2_control 的完整数据流图：从 URDF 中 `<ros2_control>` 标签的解析，到 Resource Manager 创建 state/command interfaces，到控制器通过 handle 读写数据，到硬件执行物理 I/O。标注每个步骤是在哪个线程中执行的。

2. **[A 型·延迟预算]** 对一个 7-DOF 机械臂系统，列出 RT 循环中每个阶段（read、各控制器 update、write）的延迟预算。假设目标频率是 1kHz，计算最多能同时运行多少个控制器。

3. **[思考题·对比操作系统]** ros2_control 的 Resource Manager 类似于操作系统的哪个组件？Command interface 的独占锁类似于操作系统的哪种同步原语？两者的设计出发点有什么异同？

---

## M12.2 三种硬件组件类型 ⭐⭐

### 动机 ⭐⭐

在 M12.1 中我们知道 Hardware Interface 是与物理硬件通信的代码。但不同硬件的通信模式差异很大：
- 一台 6-DOF 机械臂的 6 个关节通过同一个 EtherCAT 总线通信——读一次就得到所有关节的状态
- 一个模块化机器人的每个关节有独立的驱动板和通信通道——每个关节可以独立读写
- 一个外部力矩传感器只需要读取，不需要写入命令

如果用同一个接口来覆盖这三种场景，要么过于复杂（简单传感器也要实现命令写入），要么过于简单（无法表达总线共享的语义）。

### 三种接口类型 ⭐⭐

ros2_control 定义了三种硬件组件类型来覆盖这些场景：

| 类型 | 语义 | 典型场景 | URDF 标签 |
|------|------|---------|-----------|
| **SystemInterface** | 多 DOF 共享一个通信总线 | 6-DOF 机械臂 (EtherCAT), 移动底盘 (CAN bus) | `<hardware><plugin>my_robot/SystemHW</plugin></hardware>` |
| **ActuatorInterface** | 单个执行器独立通信 | 模块化关节 (每关节独立 USB/CAN), Dynamixel 舵机 | `<hardware><plugin>my_motor/ActuatorHW</plugin></hardware>` |
| **SensorInterface** | 纯传感器，只读 | 外部 F/T 传感器, IMU, 激光传感器 | `<hardware><plugin>my_sensor/SensorHW</plugin></hardware>` |

**SystemInterface** 是最常用的类型。一个 SystemInterface 实例管理多个关节——例如 Franka Panda 的 7 个关节通过同一个 UDP 连接通信，`read()` 一次调用就读回 7 个关节的位置/速度/力矩，`write()` 一次调用就发送 7 个关节的命令。这种「批量 I/O」的模式比逐关节通信高效得多（一个 EtherCAT 帧可以携带所有从站的数据）。

**ActuatorInterface** 适用于每个执行器有独立通信通道的情况。例如用 Dynamixel 舵机组装的机械臂，每个舵机通过 USB-TTL 独立寻址。ActuatorInterface 只管理一个关节，Resource Manager 会为每个关节分别加载一个 ActuatorInterface 实例。

**SensorInterface** 是最简单的——只有 `read()`，不需要 `write()`。典型用例是外接的 6 轴力矩传感器（ATI、OnRobot），它只需要把力矩数据读进来，不需要发送命令。

> **本质洞察**：三种类型的划分不是按「硬件品牌」分的，而是按「通信拓扑」分的。同一台机器人的不同部分可以使用不同类型——例如 Franka 的 7 个关节用一个 SystemInterface（共享 UDP 总线），末端的夹爪用一个 ActuatorInterface（独立 USB），外接的力矩传感器用一个 SensorInterface。

### URDF 中的声明

硬件类型在 URDF 的 `<ros2_control>` 标签中声明：

```xml
<!-- SystemInterface: 一个 <ros2_control> 块管理多个 joint -->
<ros2_control name="FrankaSystem" type="system">
  <hardware>
    <plugin>franka_hardware/FrankaHardwareInterface</plugin>
    <param name="robot_ip">172.16.0.2</param>
  </hardware>
  <joint name="joint1">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
  </joint>
  <joint name="joint2">
    <!-- 同上，7 个关节都在同一个 <ros2_control> 块内 -->
  </joint>
  <!-- ... joint3 到 joint7 ... -->
</ros2_control>

<!-- SensorInterface: 独立的力矩传感器 -->
<ros2_control name="FTSensor" type="sensor">
  <hardware>
    <plugin>my_sensors/ATIFTSensorInterface</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
  </hardware>
  <sensor name="ft_sensor">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
  </sensor>
</ros2_control>
```

Resource Manager 从 URDF 解析这些标签，通过 pluginlib 加载对应的硬件插件，并根据 `<state_interface>` 和 `<command_interface>` 声明管理可用接口。注意这里有版本差异：Humble 时代的常见写法是硬件插件覆写 `export_state_interfaces()` / `export_command_interfaces()`，把接口显式绑定到自己的内部缓冲区；Jazzy/Kilted 引入了按 XML 自动创建和导出的默认路径，接口存储由框架管理。后一种路径不会自动绑定你随手声明的 `std::vector<double>`，驱动必须通过框架提供的接口访问方式更新状态、读取命令，或继续采用显式导出写法。

### 反事实推理：如果只有 SystemInterface 会怎样

假设 ros2_control 只提供 SystemInterface，不区分三种类型：
- 外接 F/T 传感器也要实现 `write()`——返回 `OK` 但不做任何事。这不仅浪费 RT 循环时间，而且语义不清（调用者不知道 `write()` 实际上是空操作）
- 模块化关节也要塞进同一个 SystemInterface——但它们的通信是独立的，强行共享一个 `read()` 调用会导致代码里充斥 `if-else` 来区分不同关节的通信方式
- Resource Manager 无法判断哪些组件是只读的，无法优化 RT 循环的执行顺序

分成三种类型后，每种都有最小必要接口，既清晰又高效。

### ⚠️ 常见陷阱

```
💡 概念误区：认为 SystemInterface 只能用于工业机械臂
   新手想法："我做的是移动机器人，应该用 ActuatorInterface"
   实际上：如果你的移动底盘的多个电机通过同一个 CAN bus 通信，
          应该用 SystemInterface（共享总线语义）。
          ActuatorInterface 适用于每个电机有独立通信通道的场景。
   判断标准：
     - 一次通信能读/写多个执行器？→ SystemInterface
     - 每个执行器独立通信？→ ActuatorInterface
     - 只读传感器？→ SensorInterface
```

```
⚠️ 编程陷阱：在 SensorInterface 中声明 command_interface
   错误做法：在 URDF 的 sensor 类型 <ros2_control> 块中声明 command_interface
   现象：Resource Manager 报错或静默忽略，控制器无法获取命令接口
   根本原因：SensorInterface 的 export_command_interfaces() 返回空向量，
            即使 URDF 中声明了也无效
   正确做法：如果需要向传感器发送配置命令（如改采样率），
            在 on_configure() 中实现，不通过 command_interface
```

### 练习

1. **[A 型·分类]** 对以下硬件，判断应使用哪种接口类型并说明理由：(a) UR5e 6-DOF 臂 (EtherCAT)，(b) Robotis Dynamixel 串联舵机臂 (U2D2 USB 总线)，(c) Intel RealSense 深度相机，(d) OnRobot HEX 6-axis F/T 传感器，(e) 差速驱动移动底盘 (CAN bus 连两个电机)

2. **[A 型·URDF 编写]** 为一个包含 7-DOF 臂 + 1-DOF 夹爪 + 6-axis F/T 传感器的系统编写完整的 `<ros2_control>` URDF 标签。臂和夹爪用不同的 `<ros2_control>` 块（因为夹爪有独立的驱动板）。

3. **[思考题]** Dynamixel 舵机可以通过同一条 TTL 总线寻址所有舵机（广播模式），也可以逐个寻址。在这种情况下，应该用 SystemInterface 还是 ActuatorInterface？分别讨论两种选择的优缺点。

---

## M12.3 编写自定义 SystemInterface ⭐⭐

### 动机 ⭐⭐

理解了架构之后，最重要的实践能力是**自己写一个硬件驱动**。无论你用的是哪种机器人——工业臂、协作臂、自定义关节——你都需要实现一个 Hardware Interface 来桥接 ros2_control 和你的硬件通信协议。

本节以 SystemInterface 为例（因为它覆盖最常见的场景），完整演示一个从零开始的硬件驱动。

### 生命周期状态机 ⭐⭐

在写代码之前，必须理解 Hardware Interface 的生命周期。它遵循 ROS2 Lifecycle Node 的状态机模型：

```
                     on_init()
    ┌─────────────────────────────────┐
    │           UNCONFIGURED          │
    │  (刚加载, 什么都没做)            │
    └────────────┬────────────────────┘
                 │ on_configure()
                 │ (打开通信通道, 解析参数)
                 ▼
    ┌─────────────────────────────────┐
    │            INACTIVE             │
    │  (通信已建立, 但电机未使能)       │
    └────────────┬────────────────────┘
                 │ on_activate()
                 │ (使能电机, 释放制动)
                 ▼
    ┌─────────────────────────────────┐
    │             ACTIVE              │
    │  (正常运行: read/write 被调用)   │
    └────────────┬────────────────────┘
                 │ on_deactivate()
                 │ (锁定制动, 停止电机)
                 ▼
    ┌─────────────────────────────────┐
    │            INACTIVE             │
    └─────────────────────────────────┘
```

每个状态转换都有对应的回调函数。关键原则：
- `on_init()`：解析接口声明，预分配与接口数量相关的固定缓冲区
- `on_configure()`：读取参数，预分配配置期确定的资源，打开串口 / 初始化 CAN 总线 / 建立 UDP 连接
- `on_activate()`：使能电机、释放制动——从这一刻起 `read()/write()` 开始被调用
- `on_deactivate()`：锁定制动、停止命令发送
- `on_cleanup()`：关闭通信通道、释放资源
- `on_error()`：错误恢复逻辑

进入 active 状态后，`read()` / controller `update()` / `write()` 禁止堆分配和阻塞 I/O；需要在线刷新的参数应在非实时回调或 timer 中处理，再通过实时安全缓冲区传给 `update()`。

这种分层设计的智慧在于**资源获取与使能分离**。`on_configure()` 只负责建立通信通道（资源获取），`on_activate()` 才真正使能硬件（开始控制）。这样在调试阶段可以反复 activate/deactivate 而不用每次都重新建立通信连接——这在串口设备上尤其重要（频繁开关串口可能导致驱动崩溃）。

> **跨领域类比**：这种分层设计类似于数据库连接池——`on_configure()` 是「建立连接」，`on_activate()` 是「从池中取出连接开始使用」，`on_deactivate()` 是「归还连接」，`on_cleanup()` 是「关闭连接」。连接的生命周期和使用的生命周期独立管理。

### 完整代码实现 ⭐⭐

下面我们分步构建一个完整的自定义 SystemInterface 实现。在深入代码之前，先理解每个文件的设计角色和它们之间的关系。我们分三个文件来组织：头文件、源文件、pluginlib 声明。

**Step 1: 头文件——声明类结构**

先看为什么要这样声明：Hardware Interface 必须继承 `hardware_interface::SystemInterface` 并覆写生命周期回调和 `read()/write()` 方法。这个示例采用**显式导出接口**的写法：`export_state_interfaces()` / `export_command_interfaces()` 把 ros2_control 的接口句柄绑定到成员 `std::vector<double>` 的元素地址上，因此后续 `read()` 更新 `hw_positions_`、`write()` 读取 `hw_commands_position_` 才会被 controller 看到。不要把内部 vector 当成框架会自动识别的特殊变量。

```cpp
// include/my_robot_hardware/my_robot_system.hpp
#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <vector>
#include <string>

namespace my_robot_hardware {

using CallbackReturn =
  hardware_interface::CallbackReturn;  // ros2_control 对 lifecycle CallbackReturn 的别名

class MyRobotSystem : public hardware_interface::SystemInterface {
public:
  // on_init/on_configure 是非 RT 配置路径，可分配/阻塞；
  // read/write 和 controller update 是 RT 热路径，禁止分配/阻塞。
  CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

  // RT 热路径——必须满足实时约束
  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  // 本示例使用显式导出：把 ros2_control 接口绑定到下面的 vector 元素
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

private:
  bool open_and_configure_serial();
  bool read_current_positions_once();

  // 状态缓冲区（在 on_init 中预分配）
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // 命令缓冲区
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;

  // 硬件通信句柄
  int serial_fd_{-1};
  std::string serial_port_;
  int baud_rate_{115200};
};

}  // namespace my_robot_hardware
```

**Step 2: 源文件——实现各回调**

```cpp
// src/my_robot_system.cpp
#include "my_robot_hardware/my_robot_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cerrno>        // errno
#include <chrono>        // steady_clock
#include <cstring>       // memcpy
#include <fcntl.h>       // open
#include <termios.h>     // termios
#include <thread>        // sleep_for
#include <unistd.h>      // close, read, write

namespace my_robot_hardware {

namespace {
bool baud_rate_to_termios(int baud_rate, speed_t& speed)
{
  switch (baud_rate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
#ifdef B230400
    case 230400: speed = B230400; return true;
#endif
#ifdef B460800
    case 460800: speed = B460800; return true;
#endif
#ifdef B921600
    case 921600: speed = B921600; return true;
#endif
    default: return false;
  }
}
}  // namespace

// ============================================================
// on_init(): 第一个被调用的回调。
// 目的：解析参数 + 预分配接口数量决定的固定缓冲区
// 约束：允许配置期 malloc/日志/异常；不要把分配留到 read/write/update
// ============================================================
CallbackReturn MyRobotSystem::on_init(
    const hardware_interface::HardwareInfo& info)
{
  // 调用基类的 on_init —— 基类解析 URDF 中的 joint/interface 声明
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 从 <param> 标签读取硬件参数
  serial_port_ = info_.hardware_parameters.at("serial_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));

  // !! 关键：在这里预分配接口相关缓冲区 !!
  // info_.joints 包含 URDF 中声明的所有关节
  const size_t n_joints = info_.joints.size();
  hw_positions_.resize(n_joints, 0.0);
  hw_velocities_.resize(n_joints, 0.0);
  hw_efforts_.resize(n_joints, 0.0);
  hw_commands_position_.resize(n_joints, 0.0);
  hw_commands_velocity_.resize(n_joints, 0.0);

  // 验证 URDF 中声明的接口与代码匹配
  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() < 1) {
      RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
        "Joint '%s' has no command interfaces",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystem"),
    "Initialized with %zu joints on %s @%d baud",
    n_joints, serial_port_.c_str(), baud_rate_);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyRobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    for (const auto& interface : joint.state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(joint.name, interface.name, &hw_positions_[i]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(joint.name, interface.name, &hw_velocities_[i]);
      } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(joint.name, interface.name, &hw_efforts_[i]);
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyRobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    for (const auto& interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(joint.name, interface.name, &hw_commands_position_[i]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(joint.name, interface.name, &hw_commands_velocity_[i]);
      }
    }
  }
  return command_interfaces;
}

bool MyRobotSystem::open_and_configure_serial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }

  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
      "Failed to open serial port: %s", serial_port_.c_str());
    return false;
  }

  speed_t termios_speed;
  if (!baud_rate_to_termios(baud_rate_, termios_speed)) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
      "Unsupported baud rate: %d", baud_rate_);
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 配置串口参数: 使用 URDF 参数解析出的 termios_speed，而不是写死 B115200。
  // 教学示例默认 baud_rate_=115200，但真实驱动应支持参数化并验证取值。
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
      "Failed to read termios attributes: %s", std::strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  cfsetospeed(&tty, termios_speed);
  cfsetispeed(&tty, termios_speed);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_lflag = 0;
  tty.c_iflag = 0;
  tty.c_oflag = 0;

  // !! 关键: 设置非阻塞读取 !!
  // O_NONBLOCK + VMIN=0, VTIME=0 → read() 立即返回已有数据，不等待
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
      "Failed to apply termios attributes: %s", std::strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystem"),
    "Serial port configured successfully @%d baud", baud_rate_);
  return true;
}

bool MyRobotSystem::read_current_positions_once()
{
  // 真实驱动通常先发送一次 query_position_frame(serial_fd_)，
  // 再解析返回帧刷新 hw_positions_/hw_velocities_/hw_efforts_。
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  uint8_t rx_buf[64];

  while (std::chrono::steady_clock::now() < deadline) {
    ssize_t n = ::read(serial_fd_, rx_buf, sizeof(rx_buf));
    if (n > 0) {
      // parse_current_position_frame(rx_buf, n, hw_positions_,
      //                              hw_velocities_, hw_efforts_);
      for (size_t i = 0; i < hw_positions_.size(); ++i) {
        hw_commands_position_[i] = hw_positions_[i];
        hw_commands_velocity_[i] = 0.0;
      }
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
    "Failed to read current joint positions during recovery");
  return false;
}

// ============================================================
// on_configure(): 打开通信通道
// 目的：建立与硬件的连接，预分配配置期才能确定的通信资源
// 约束：允许配置期阻塞 I/O；active 后 read/write/update 禁止阻塞
// ============================================================
CallbackReturn MyRobotSystem::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  return open_and_configure_serial()
    ? CallbackReturn::SUCCESS
    : CallbackReturn::ERROR;
}

// ============================================================
// on_activate(): 使能电机
// 目的：从 INACTIVE 进入 ACTIVE，开始接受 read/write 调用
// ============================================================
CallbackReturn MyRobotSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // !! 极其重要：同步当前位置到命令缓冲区 !!
  // 如果 command 初始化为 0 而关节当前在 1.5rad，
  // 激活瞬间会产生巨大的命令跳变，可能损坏硬件
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_commands_position_[i] = hw_positions_[i];
    hw_commands_velocity_[i] = 0.0;
  }

  // 发送使能命令到电机驱动器
  // send_enable_command(serial_fd_);

  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystem"),
    "Activated — initial positions synced to commands");
  return CallbackReturn::SUCCESS;
}

// ============================================================
// read(): RT 热路径——从硬件读取状态
// 约束：无 malloc / 无阻塞 / 无异常 / 无 ROS 日志
// ============================================================
hardware_interface::return_type MyRobotSystem::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
  // 栈分配固定大小缓冲区——不触发 malloc
  uint8_t rx_buf[64];
  ssize_t n = ::read(serial_fd_, rx_buf, sizeof(rx_buf));

  if (n > 0) {
    // 解析帧数据到关节状态
    // parse_frame(rx_buf, n, hw_positions_,
    //             hw_velocities_, hw_efforts_);
    //
    // 下面用模拟逻辑示意：
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      hw_velocities_[i] =
        (hw_commands_position_[i] - hw_positions_[i]) * 500.0;
      hw_positions_[i] = hw_commands_position_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write(): RT 热路径——向硬件发送命令
// 约束：同 read()
// ============================================================
hardware_interface::return_type MyRobotSystem::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
  uint8_t tx_buf[64];  // 栈分配
  // pack_command_frame(tx_buf, hw_commands_position_);
  // ssize_t n = ::write(serial_fd_, tx_buf, frame_len);

  return hardware_interface::return_type::OK;
}

// ============================================================
// on_deactivate(): 停止电机
// ============================================================
CallbackReturn MyRobotSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // send_disable_command(serial_fd_);
  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystem"),
    "Deactivated");
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_cleanup(): 关闭通信通道
// ============================================================
CallbackReturn MyRobotSystem::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_error(): 错误恢复
// ============================================================
CallbackReturn MyRobotSystem::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystem"),
    "Error detected, attempting recovery...");

  // 恢复时必须复用完整串口配置：termios、非阻塞读取、波特率等。
  if (!open_and_configure_serial()) {
    return CallbackReturn::FAILURE;  // 无法恢复
  }

  // 重新读取硬件当前位置，并把 command 同步到 state，避免恢复后命令跳变。
  if (!read_current_positions_once()) {
    close(serial_fd_);
    serial_fd_ = -1;
    return CallbackReturn::FAILURE;  // 无法恢复
  }

  return CallbackReturn::SUCCESS;    // 恢复成功
}

}  // namespace my_robot_hardware

// 注册为 pluginlib 插件
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MyRobotSystem,
  hardware_interface::SystemInterface)
```

**Step 3: pluginlib 声明文件**

```xml
<!-- my_robot_hardware_plugin.xml -->
<library path="my_robot_hardware">
  <class name="my_robot_hardware/MyRobotSystem"
         type="my_robot_hardware::MyRobotSystem"
         base_class_type="hardware_interface::SystemInterface">
    <description>Custom hardware interface for MyRobot</description>
  </class>
</library>
```

**Step 4: CMakeLists.txt 关键部分**

```cmake
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(my_robot_hardware SHARED
  src/my_robot_system.cpp)

target_include_directories(my_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(my_robot_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface my_robot_hardware_plugin.xml)

install(TARGETS my_robot_hardware
  LIBRARY DESTINATION lib)
```

### 错误写法对比

```cpp
// ❌ 错误 1：在 read() 中分配内存
hardware_interface::return_type read(...) override {
  auto data = std::make_unique<double[]>(7);  // malloc!
  // RT 违规: 堆分配延迟不确定(1-100μs)
}

// ❌ 错误 2：在 read() 中抛出异常
hardware_interface::return_type read(...) override {
  if (serial_fd_ < 0) {
    throw std::runtime_error("Serial port not open");
    // RT 违规: 异常展开涉及 malloc 和栈回退
  }
}
// ✅ 正确做法：返回错误码
hardware_interface::return_type read(...) override {
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
}

// ❌ 错误 3：on_activate 不同步初始命令
CallbackReturn on_activate(...) override {
  // 忘记 hw_commands_position_[i] = hw_positions_[i];
  // 激活瞬间 command 可能是 0，而关节在 1.5rad
  // 结果：电机疯转到 0 位置 —— 极其危险!
  return CallbackReturn::SUCCESS;
}
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：on_activate 不同步初始命令值
   错误做法：激活时 hw_commands_ 保持默认值（0.0 或上次的值）
   现象：电机在激活瞬间突然跳到命令值对应的位置
   根本原因：控制器假设 command 初始值是当前位置
   正确做法：
     hw_commands_position_[i] = hw_positions_[i];
   自检方法：在仿真中将机器人拖到非零位姿再激活，观察是否有跳变
```

```
💡 概念误区：认为内部 vector 会被 ros2_control 自动绑定
   新手想法："URDF 里写了 <state_interface>，所以 hw_positions_ 会自动
             成为 joint/position 的数据源"
   实际上：内部 vector 只是普通成员变量。它能被 controller 读写，必须满足
          下面两种模式之一：
          1. 旧式/显式导出：覆写 export_state_interfaces() 和
             export_command_interfaces()，把接口句柄绑定到 vector 元素地址。
             上面的示例采用这种模式，所以 Humble 风格代码迁到 Jazzy/Kilted
             仍然容易看懂。
          2. Jazzy/Kilted 推荐模式：不覆写旧式 export_*，由框架根据
             ros2_control XML 自动创建接口，驱动代码通过框架管理的接口访问
             API 或预缓存的接口句柄读写接口存储；RT 路径里要避开会查找、
             阻塞或抛异常的访问形式。这时应删除自管的 hw_commands_ /
             hw_states_ 存储，或只把它们当作通信协议的临时缓存，不能假设
             它们和接口句柄自动同步。
   关键判断：如果你在 read()/write() 中直接操作 hw_positions_、hw_commands_，
             就必须像本节一样显式导出并绑定；如果依赖框架自动导出，就操作
             框架提供的接口访问 API，而不是私有 vector。
```

```
⚠️ 编程陷阱：串口 read() 阻塞导致 RT 循环超时
   错误做法：用默认的阻塞模式打开串口
   现象：如果硬件没有发送数据，read() 无限等待，RT 循环卡住
   根本原因：默认串口 read() 是阻塞的
   正确做法：
     1. 设置 VMIN=0, VTIME=0（立即返回）
     2. 或打开时加 O_NONBLOCK 标志
   自检方法：拔掉串口线，观察 RT 循环是否继续运行
```

### 练习

1. **[A 型·Mock 驱动]** 修改上述代码，实现一个 Mock 硬件驱动：`read()` 返回正弦波位置（模拟关节运动），`write()` 只打印命令值。配合 `JointStateBroadcaster` 在 RViz 中可视化。

2. **[A 型·错误注入]** 在 `read()` 中加入随机延迟（模拟通信故障），用 `cyclictest` 测量对 RT 循环最大延迟的影响。

3. **[跨章综合题]** 结合 M11（实时 C++）和本章，设计一个完整的实时硬件驱动测试方案：(a) 用 `mlockall` + `SCHED_FIFO` 配置 RT 线程，(b) 在 `read()` 中使用 RT-safe 的环形缓冲区替代串口直接读取，(c) 用 `clock_gettime(CLOCK_MONOTONIC)` 测量每个循环的 jitter。画出整个系统的线程模型图。

---

## M12.4 机械臂常用控制器 ⭐⭐

### 动机

有了硬件驱动（M12.3），数据已经能从硬件读进来并写回去了。但「读什么、算什么、写什么」的控制逻辑还需要控制器来实现。ros2_control 提供了一系列开箱即用的标准控制器，覆盖了机械臂的主要控制模式。

理解这些控制器不仅是使用它们的前提，更是理解 MoveIt2 执行层的关键——MoveIt2 规划好的轨迹最终通过 JointTrajectoryController 发送给硬件。

### JointTrajectoryController (JTC) ⭐⭐

**定位**：JTC 是 MoveIt2 的默认执行后端，也是机械臂最常用的控制器。它接收一个关节轨迹（`FollowJointTrajectory` action），在 RT 循环中用样条插值生成平滑的中间点，并通过 PID 反馈跟踪这些点。

**工作原理**：

```
MoveIt2 规划输出:
  [t=0: q1, t=0.5: q2, t=1.0: q3, ...]  (稀疏的轨迹点)
                    │
                    ▼
JointTrajectoryController 内部:
  1. 收到 FollowJointTrajectory goal
  2. 用三次/五次样条插值生成 1kHz 的密集轨迹
  3. 每个 RT 周期:
     a. 查表得到当前时刻的目标 (q_d, dq_d, ddq_d)
     b. 读取 state interface 的实际值 (q, dq)
     c. 计算命令:
        position 模式: cmd = q_d (直接前馈)
        velocity 模式: cmd = dq_d + Kp*(q_d - q)
        effort  模式: cmd = Kp*(q_d-q) + Kd*(dq_d-dq) + Ki*∫
     d. 写入 command interface
```

JTC 支持三种命令模式，取决于硬件提供的 command interface：

| Command Interface | JTC 行为 | 适用场景 |
|-------------------|---------|---------|
| `position` | 直接发送目标位置，硬件内部做位置闭环 | 大多数工业臂（UR、Franka 位置模式） |
| `velocity` | 发送目标速度 + 位置 PID 反馈 | 需要速度控制的场景 |
| `effort` | 发送力矩 + 完整 PID 反馈 | 力矩控制模式（Franka effort 模式） |

**配置示例**：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    # 轨迹容差
    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.01
      joint1:
        goal: 0.01       # 终点误差容差 (rad)
        trajectory: 0.05  # 过程中误差容差

    # PID 增益（仅 velocity/effort 模式需要）
    gains:
      joint1: {p: 100.0, i: 0.1, d: 10.0, i_clamp: 1.0}

    allow_integration_in_goal_trajectories: true
```

**跨领域类比**：JTC 类似于 CNC 加工中的 G-code 执行器——接收离散的路径点（G01 X10 Y20），内部用插值生成连续的电机命令。两者的共同点是「稀疏目标 → 密集命令」的转换，区别在于 CNC 用线性/圆弧插值，JTC 用多项式样条插值。

### ForwardCommandController ⭐⭐

**定位**：最简单的控制器——直接将接收到的命令转发给硬件，不做任何插值或反馈。

**为什么需要这么「简陋」的控制器？** 因为在 RL 策略部署场景中，策略网络已经输出了每个时间步的关节命令，不需要 JTC 再做插值。如果用 JTC，它会在策略输出的两个时间步之间做样条插值——这反而破坏了策略原始的命令意图。

```yaml
forward_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
    interface_name: position
```

> **反事实推理**：如果 RL 部署也用 JTC 会怎样？JTC 会在策略输出的两个点之间做样条插值，产生「平滑但不受策略控制」的中间命令。对于力控/阻抗控制的 RL 策略，这种插值可能引入非预期的力矩，导致接触行为不稳定。所以 RL 部署首选 ForwardCommandController。

### AdmittanceController ⭐⭐⭐

**定位**：实现力感知的柔顺控制——当外力作用于机械臂时，机械臂像弹簧-阻尼系统一样柔顺地响应。

回顾力控子课程 F01/F03-F05：阻抗/导纳控制关注「力与运动之间的动态关系」。阻抗控制器通常直接输出力矩或等效 effort 命令（需要硬件 effort 接口或底层力矩控制能力），而导纳控制器将外力转换为位置/速度修正，再交给位置或速度控制链路执行。AdmittanceController 属于后者。

**力控交互的数学模型**（质量-弹簧-阻尼器）：

$$M\ddot{x} + D\dot{x} + K(x - x_d) = F_{ext}$$

其中：
- $M$：虚拟惯量矩阵——决定对外力的响应「惯性」
- $D$：虚拟阻尼矩阵——决定运动的「粘滞感」
- $K$：虚拟刚度矩阵——决定恢复到目标位置的「弹性」
- $x_d$：期望位置/姿态
- $F_{ext}$：外部力/力矩（从 F/T 传感器读取）

AdmittanceController 在每个 RT 周期中：
1. 读取 F/T 传感器数据 $F_{ext}$
2. 用上述动力学模型积分一步，得到位置修正 $\Delta x$
3. 将 $x_d + \Delta x$ 通过逆运动学转换为关节位置命令
4. 写入 position command interface

**配置示例**（Humble/Jazzy/Rolling 的 `admittance_controller` schema 形态，具体字段以目标发行版安装的参数定义文件为准）：

```yaml
controller_manager:
  ros__parameters:
    admittance_controller:
      type: admittance_controller/AdmittanceController

admittance_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7

    # 写硬件 command interface；非 chained 模式下直接输出到硬件。
    command_interfaces:
      - position

    # 读取硬件 state interface。部分版本也支持 acceleration；
    # 若硬件不提供 velocity/acceleration，控制器可能退回到 last command。
    state_interfaces:
      - position
      - velocity

    # ChainedControllerInterface 导出的 reference interfaces。
    # 上游 JTC/限幅器可以写这些虚拟接口，再由本控制器写硬件。
    chainable_command_interfaces:
      - position
      - velocity

    ft_sensor:
      name: ft_sensor
      frame:
        id: tool0
        external: false
      filter_coefficient: 0.005

    control:
      frame:
        id: tool0
        external: false

    fixed_world_frame:
      frame:
        id: base_link
        external: false

    gravity_compensation:
      frame:
        id: tool0
        external: false
      CoG:
        pos: [0.0, 0.0, 0.10]
        force: 9.81  # end-effector mass * 9.81

    admittance:
      selected_axes: [true, true, true, false, false, false]
      mass:          [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
      damping_ratio: [1.0,  1.0,  1.0,  1.0, 1.0, 1.0]
      stiffness:     [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
      joint_damping: 5.0

    kinematics:
      plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
      plugin_package: kinematics_interface
      base: base_link
      tip: tool0
      group_name: manipulator
      alpha: 0.0005
```

版本差异要点：
- `chainable_command_interfaces` 是链式控制器场景的关键字段，常见默认是 `position`/`velocity`，不要只写 `command_interfaces`。
- `kinematics.base/tip/group_name/alpha`、`control.frame`、`fixed_world_frame`、`gravity_compensation` 和 `admittance.selected_axes` 在新版本参数库中都是一等参数；缺失时控制器可能配置失败或坐标系/重力补偿错误。
- 旧发行版或下游 vendor fork 可能把 `group_name` 写成 `group`，或缺少某些 `external`/动态参数字段。产品工程应以本机 `/opt/ros/<distro>/share/admittance_controller` 中的 GPL 参数定义和官方示例为准。

### GripperActionController ⭐

专门控制平行夹爪的开合。接收 `GripperCommand` action（目标位置 + 最大力）。

```yaml
gripper_controller:
  ros__parameters:
    joint: finger_joint
    action_monitor_rate: 20.0
    goal_tolerance: 0.002
    max_effort: 50.0
    stall_velocity_threshold: 0.001
    stall_timeout: 1.0
```

### JointStateBroadcaster ⭐

不是控制器，而是广播器——从 state interfaces 读取关节状态，发布到 `/joint_states` topic。几乎所有系统都需要它。

### 控制器选型决策流程

```
你的场景是什么？
    │
    ├── MoveIt2 规划轨迹执行？
    │   └── JointTrajectoryController
    │
    ├── RL 策略直接输出关节命令？
    │   └── ForwardCommandController
    │
    ├── 需要力感知柔顺控制？
    │   ├── 硬件支持 effort 接口？
    │   │   └── 考虑自定义阻抗控制器(F03/F04)
    │   └── 只支持 position 接口？
    │       └── AdmittanceController
    │
    ├── 夹爪开合？
    │   └── GripperActionController
    │
    └── 自定义控制逻辑？
        └── 写 custom controller (继承 ControllerInterface)
```

### 运行时控制器切换

ros2_control 支持在运行时原子地切换控制器：

```bash
# 原子切换（同时停用旧控制器、激活新控制器）
ros2 control switch_controllers \
  --activate forward_position_controller \
  --deactivate joint_trajectory_controller \
  --strict

# 查看当前活跃的控制器
ros2 control list_controllers

# 查看硬件接口状态
ros2 control list_hardware_interfaces
```

`--strict` 标志确保如果切换失败（例如两个控制器争夺同一个 command interface），整个操作回滚。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：忘记启动 JointStateBroadcaster
   错误做法：只配置了 JTC，没有加 JointStateBroadcaster
   现象：RViz 中机器人不动，/joint_states topic 无数据
   根本原因：JointStateBroadcaster 是 /joint_states 的唯一数据源
   正确做法：永远在 controller_manager 配置中加上 JointStateBroadcaster
   自检方法：ros2 topic echo /joint_states 应有数据
```

```
💡 概念误区：认为 JTC 的 PID 参数在位置命令模式下很重要
   新手想法："我必须仔细调 JTC 的 PID 增益"
   实际上：如果用 position command interface，JTC 直接发送
          目标位置给硬件——硬件内部有自己的位置闭环。JTC 的 PID
          只在 velocity/effort command interface 时才生效。
   何时需要 PID：当 command_interfaces 是 velocity 或 effort 时
```

```
🧠 思维陷阱：认为"控制器切换是无缝的"
   新手想法："switch_controllers 是原子操作，所以切换平滑过渡"
   实际上：原子只保证「不会出现两个控制器同时写同一接口」，
          不保证切换时命令值连续。如果旧控制器最后命令是 q1，
          新控制器初始命令是 q2 != q1，切换瞬间会有命令跳变。
   正确思维：在新控制器的 on_activate() 中同步当前位置
```

### 练习

1. **[A 型·JTC 配置]** 为 Franka Panda 配置完整的 JTC + JointStateBroadcaster。在 RViz 中用 `ros2 action send_goal` 发送一个简单的轨迹目标。

2. **[A 型·控制器切换]** 在运行时从 JTC 切换到 ForwardCommandController，再切换回来。用 `rqt_plot` 观察切换瞬间的关节位置是否有跳变。

3. **[思考题]** AdmittanceController 的虚拟质量 M 越大，对外力的响应越慢；M 越小，响应越灵敏但容易振荡。在人机协作场景中如何选择 M/D/K 参数？

---

## M12.5 generate_parameter_library ⭐⭐⭐

### 动机

控制器有大量 YAML 参数需要配置。传统做法是在代码中手动用 `get_parameter()` 读取每个参数、手动检查类型和范围、手动设置默认值。这不仅冗长，而且容易出错——参数名拼写错误只有在运行时才发现。

**generate_parameter_library (GPL)** 由 PickNik Robotics 开发，解决这个问题：你写一个 YAML 格式的**参数声明文件**，GPL 自动生成类型安全的 C++ 结构体和参数验证代码。

### 工作原理

```
参数声明文件 (*.yaml)               GPL 自动生成的代码
┌────────────────────────┐         ┌────────────────────────┐
│ my_controller:          │   ──►  │ struct Params {         │
│   joints: {              │        │   std::vector<string>   │
│     type: string_array  │        │     joints;             │
│   }                     │        │   double kp = 100.0;    │
│   kp: {                 │        │   // 自动范围检查       │
│     type: double        │        │ };                      │
│     default_value: 100  │        │                         │
│     validation:         │        │ ParamListener listener; │
│       gt: 0.0           │        │ // 自动订阅参数更新     │
│   }                     │        └────────────────────────┘
└────────────────────────┘
```

**优势**：
1. **编译期类型安全**——参数名拼写错误在编译时就报错
2. **自动验证**——范围检查、类型检查在参数加载时自动执行
3. **运行时可调**——ParamListener 自动订阅参数更新，支持在线调参
4. **文档自动生成**——从声明文件生成参数说明文档

### 示例

```yaml
# my_controller_params.yaml
my_controller:
  joints: {
    type: string_array,
    description: "Controlled joint names",
    default_value: [],
    validation: { not_empty<>: null }
  }
  gains:
    kp: {
      type: double,
      default_value: 100.0,
      description: "Proportional gain",
      validation: { gt<>: [0.0] }
    }
    kd: {
      type: double,
      default_value: 10.0,
      description: "Derivative gain",
      validation: { gt_eq<>: [0.0] }
    }
  update_rate: {
    type: int,
    default_value: 500,
    description: "Controller update rate (Hz)",
    validation: { gt<>: [0], lt<>: [10000] },
    read_only: true
  }
```

```cpp
// 使用自动生成的参数结构体
#include "my_controller_parameters.hpp"  // GPL 生成
#include <realtime_tools/realtime_buffer.hpp>

class MyController : public controller_interface::ControllerInterface {
  std::shared_ptr<my_controller::ParamListener> param_listener_;
  my_controller::Params params_;
  realtime_tools::RealtimeBuffer<my_controller::Params> params_buffer_;
  rclcpp::TimerBase::SharedPtr param_timer_;

  CallbackReturn on_configure(...) override {
    param_listener_ = std::make_shared<my_controller::ParamListener>(
      get_node());
    params_ = param_listener_->get_params();
    params_buffer_.writeFromNonRT(params_);

    // 参数刷新在非实时 executor 回调中完成；update() 只读 RealtimeBuffer。
    param_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (param_listener_->is_old(params_)) {
          params_ = param_listener_->get_params();
          params_buffer_.writeFromNonRT(params_);
        }
      });

    // 直接用结构体成员——类型安全，有自动补全
    auto joints = params_.joints;       // std::vector<std::string>
    double kp = params_.gains.kp;       // double, 保证 > 0
    int rate = params_.update_rate;     // int, 保证 0 < rate < 10000
    return CallbackReturn::SUCCESS;
  }

  return_type update(...) override {
    const auto* rt_params = params_buffer_.readFromRT();
    const double kp = rt_params->gains.kp;
    // 使用 kp 等实时侧快照；不要在这里调用 get_params() 或分配内存
  }
};
```

> **跨领域类比**：GPL 之于 ros2_control 参数，就像 Protocol Buffers 之于网络协议——都是用声明式的 schema 自动生成类型安全的代码，消除手动序列化/反序列化的 boilerplate。两者的共同设计哲学是「schema 即文档，schema 即代码」。

### ⚠️ 常见陷阱

```
💡 概念误区：认为 GPL 只是"代码生成器"
   新手想法："我直接用 get_parameter() 也能做同样的事"
   实际上：GPL 提供的不仅是类型安全，还有运行时参数动态更新、
          参数验证、参数文档自动生成。手动实现这些需要几百行 boilerplate。
   关键价值：在控制器代码中消除所有手动参数解析代码
```

### 练习

1. **[A 型]** 为 ForwardCommandController 编写 GPL 声明文件，包含关节名、接口名、低通滤波器时间常数（带范围验证）。

2. **[思考题]** GPL 的参数声明文件本质上是一种 DSL。它与 URDF 的 `<ros2_control>` 标签有什么关系？两者有没有信息重复（例如关节名）？

---

## M12.6 Chainable Controllers 与错误恢复 ⭐⭐⭐

### 动机

考虑一个常见场景：你想在 JTC 的输出上加一层安全限制（关节速度限幅），或者在 AdmittanceController 的输出前加一层重力补偿。传统做法是把所有逻辑塞进一个巨大的控制器——但这违反了单一职责原则，代码不可复用。

Chainable Controllers 解决了这个问题：一个控制器的输出可以作为另一个控制器的输入，形成控制链。

### 工作原理

```
普通控制器:
  state_interfaces → [Controller] → command_interfaces

链式控制器:
  state_interfaces → [Controller A] → reference_interfaces
                                           │
                                           ▼
                     [Controller B] ← reference_interfaces
                           │
                           ▼
                     command_interfaces (写硬件)
```

链式控制器通过 `reference_interfaces`（引用接口）将输出暴露给下游控制器。Reference interfaces 类似于虚拟的 command interfaces——上游控制器写入，下游控制器读取。

**典型链式结构**：

```
MoveIt2 轨迹
     │
     ▼
┌─────────────────┐
│ JTC             │ → reference: position/velocity
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ JointLimiter    │ → reference: position/velocity (限幅后)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ PIDController   │ → command: effort (力矩)
└────────┬────────┘
         │
         ▼
    Hardware write()
```

### 错误恢复机制

ros2_control 的错误恢复不仅体现在 Hardware Interface 的 `on_error()` 回调中，还涉及整个系统层面的策略。

**错误传播路径**：

```
Hardware read() 返回 ERROR
        │
        ▼
Resource Manager 检测到硬件错误
        │
        ▼
Controller Manager 通知所有相关控制器
        │
        ├── 控制器进入 INACTIVE 状态
        │
        ▼
Hardware Interface 进入 ERROR 状态
        │
        ├── on_error() 尝试恢复
        │   ├── 成功 → 回到 UNCONFIGURED → 重新 configure/activate
        │   └── 失败 → 保持 FINALIZED（需要人工干预）
```

**典型错误场景与恢复策略**：

| 错误场景 | 检测方式 | 恢复策略 |
|---------|---------|---------|
| 通信超时 | read() 超时 N 次 | 重新建立连接 |
| 编码器跳变 | 相邻帧位置差 > 阈值 | 忽略当前帧，用上一帧数据 |
| 电机过温 | 驱动器报警标志 | deactivate → 等待冷却 → reactivate |
| EtherCAT 从站掉线 | 从站状态不是 OP | 重新初始化从站状态机 |
| 安全限位触发 | 硬件 safety flag | 停止所有运动，等待人工复位 |

**Franka 的错误恢复是一个好的参考**：

```cpp
// franka_ros2 中的错误恢复逻辑
// 当 libfranka 报告 reflex error 时：
try {
  robot_->automaticErrorRecovery();  // libfranka API
  // 恢复成功后重新同步关节位置
  auto state = robot_->readOnce();
  for (size_t i = 0; i < 7; i++) {
    hw_commands_position_[i] = state.q[i];
  }
} catch (const franka::Exception& e) {
  // 无法自动恢复，需要人工按下 Franka 面板按钮
  return CallbackReturn::FAILURE;
}
```

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为链式控制器会增加 RT 延迟
   新手想法："三个控制器串联执行，延迟是三倍"
   实际上：链式控制器在同一个 RT 周期内依次执行 update()。
          总延迟是三个 update() 的时间之和（通常 10-50μs * 3），
          远小于一个 RT 周期（1ms）。所有数据传递是进程内指针操作。
   类比：就像函数调用 f(g(h(x)))，同一线程内依次执行。
```

```
⚠️ 编程陷阱：错误恢复后忘记重新同步命令
   错误做法：on_error() 恢复连接后直接回到 ACTIVE
   现象：恢复后电机跳到错误恢复前的命令值，可能已经过时
   正确做法：恢复后必须重新读取当前位置并同步到命令缓冲区
```

### 练习

1. **[A 型]** 配置一个链式控制器：JTC → JointLimiter → 硬件。设置速度限制为 1.0 rad/s，验证即使 JTC 输出目标速度超限，实际命令也被裁剪。

2. **[A 型]** 在 Mock 驱动中模拟通信故障（每 100 次 read 随机返回 ERROR），实现自动恢复逻辑并验证系统能否自动恢复正常运行。

3. **[思考题]** 链式控制器的依赖关系需要 Controller Manager 按正确顺序调用 update()。Controller Manager 如何确定调用顺序？（提示：拓扑排序）

---

## M12.7 机械臂参考驱动精读 ⭐⭐

### 动机

学习 ros2_control 硬件驱动的最佳方式是精读工业级参考实现。本节精读两个最成熟的开源驱动：`franka_ros2`（Franka Panda/FR3）和 `ur_robot_driver`（UR3-UR30）。

### libfranka + franka_ros2

**libfranka**（`frankaemika/libfranka`，~250★）是 Franka 机械臂的底层 C++ 客户端库。

**通信架构**：
```
Franka 控制柜                    你的工作站
┌─────────────┐   1kHz UDP    ┌─────────────┐
│ FCI 固件     │◄────────────►│ libfranka   │
│ (ARM 核心)   │  RobotState  │             │
│              │  ◄───────    │ read(): 解析 │
│              │  Command     │ write(): 打包│
│              │  ───────►    │             │
└─────────────┘              └─────────────┘
```

**关键设计特点**：

1. **1 kHz 同步 UDP 通信**：每毫秒一个 RobotState（编码器、力矩、外力估计等），你必须在同一毫秒内返回 Command。超时 = 安全停止。这是最严格的实时约束——回顾 M11 中的 `SCHED_FIFO` + `mlockall`。

2. **PIMPL 惯用法**：`franka::Robot` 的所有内部字段都隐藏在 `Robot::Impl` 中。这让 libfranka 可以在不破坏 ABI 的情况下修改内部实现——对需要跨版本二进制兼容的生态非常重要。

3. **回调风格 vs 读写循环风格**：
   - 传统回调风格（v0.x）：`robot.control([](const RobotState& state, Duration) { return command; });`
   - 新的 Active Control API（v0.14+）：显式的 `read()`/`write()` 循环，更适合 ros2_control

4. **动力学模型接口**（v0.14+）：通过 `franka::Model model = robot.loadModel()` 获取模型对象，再调用 `model.mass(state)`、`model.coriolis(state)`、`model.gravity(state)` 计算 M/C/G（详见 F04 §2 代码精读）。

**franka_ros2 中的 SystemInterface 核心逻辑**：

```cpp
// 简化版
return_type FrankaHardwareInterface::read(...) {
  robot_state_ = robot_->readOnce();
  for (size_t i = 0; i < 7; i++) {
    hw_positions_[i]    = robot_state_.q[i];
    hw_velocities_[i]   = robot_state_.dq[i];
    hw_efforts_[i]      = robot_state_.tau_J[i];
    hw_ext_torques_[i]  = robot_state_.tau_ext_hat_filtered[i];
  }
  return return_type::OK;
}

return_type FrankaHardwareInterface::write(...) {
  if (control_mode_ == ControlMode::kPosition) {
    franka::JointPositions cmd(hw_commands_position_.data());
    robot_->writeOnce(cmd);
  } else if (control_mode_ == ControlMode::kEffort) {
    franka::Torques cmd(hw_commands_effort_.data());
    robot_->writeOnce(cmd);
  }
  return return_type::OK;
}
```

### ur_robot_driver

**ur_robot_driver**（`UniversalRobots/Universal_Robots_ROS2_Driver`，~370★）是 UR 机械臂的 ROS2 驱动。

**双通道通信架构**：

```
UR 控制箱                           你的工作站
┌──────────────┐                  ┌──────────────────┐
│ UR Controller │                  │ ur_robot_driver   │
│              │                  │                    │
│ TCP 30001    │◄── URScript 注入 │ 发送 URScript 脚本 │
│ (Primary)    │                  │ 用于实时控制       │
│              │                  │                    │
│ TCP 30004    │◄►  RTDE          │ 可定制化字段       │
│ (RTDE)       │   (500Hz)       │ 读/写分离          │
└──────────────┘                  └──────────────────┘
```

**双通道设计的原因**：
1. **状态读取**（RTDE 30004 端口，500Hz）：UR 控制器推送关节状态
2. **命令发送**（URScript 注入 30001/30002 端口）：发送 URScript 程序片段如 `servoj(q, t, lookahead, gain)`

这种设计与 Franka 的对称 UDP 不同——UR 的命令是「一段 URScript 程序」而非直接的关节命令。

### 两种驱动的对比

| 维度 | libfranka / franka_ros2 | ur_robot_driver |
|------|------------------------|-----------------|
| 通信协议 | 同步 1kHz UDP | 异步 TCP (RTDE 500Hz) |
| 命令模式 | 直接关节命令 | URScript 函数注入 |
| 实时性 | 硬实时（超时=安全停止） | 软实时（TCP 无硬延迟保证） |
| 控制频率 | 1 kHz（固定） | 125-500 Hz（可配置） |
| 力矩控制 | 原生支持 (effort interface) | 需要 External Control URCap |
| 外力估计 | 内置 | 需要额外 F/T 传感器 |
| 代码复杂度 | ~2000 行 | ~5000 行 |

**UR driver 是精读 ros2_control 驱动的"金标准"**——代码质量高、注释充分、CI 覆盖完整。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：在 Franka 驱动中超时
   错误做法：在 read() 到 write() 之间做耗时计算
   现象：libfranka 抛出 communication_error，机器人安全锁定
   根本原因：Franka FCI 要求每 1ms 必须收到命令回复
   正确做法：
     1. 确保控制器 update() 逻辑极简
     2. 重计算在单独线程中做
     3. 用 automaticErrorRecovery() 从锁定中恢复
```

```
💡 概念误区：认为 UR 的 RTDE 和 Primary 接口功能相同
   新手想法："都是 TCP 连接，用哪个都行"
   实际上：
     - RTDE (30004)：可定制字段、500Hz、推荐用于实时控制
     - Primary (30001)：125Hz、固定数据包、用于 URScript 注入
     - 两者必须配合使用
```

### 练习

1. **[B 型·UR 精读]** 精读 `ur_robot_driver/src/hardware_interface.cpp`。标注 RTDE 连接建立、RobotState 解析、URScript 命令生成、超时处理。

2. **[B 型·Franka 精读]** 精读 `franka_hardware/src/franka_hardware_interface.cpp`。对比 libfranka 的回调风格和 Active Control API。

3. **[思考题]** Franka 用同步 UDP，UR 用异步 TCP。哪种模式更适合力控场景？为什么？

---

## M12.8 EtherCAT 驱动集成 ⭐⭐⭐⭐

### 动机

在工业机械臂领域，EtherCAT 是最主流的实时通信总线协议（市场份额 >30%）。理解 EtherCAT 与 ros2_control 的集成，是从实验室走向工厂的关键一步。

### EtherCAT 基础

EtherCAT（Ethernet for Control Automation Technology）由 Beckhoff 2003 年提出，核心特征是**飞行式帧处理**：以太网帧不在从站停留——每个从站在帧经过时直接读取/写入自己的数据，然后让帧继续传递。整条总线的通信延迟与从站数量几乎无关（每个从站增加约 1μs）。

```
Master (你的工作站)
   │
   │  ┌─── 一个以太网帧 ───────────────────────────────┐
   │  │ [Header][Slave1 Data][Slave2 Data]...[SlaveN]    │
   │  └──────────────────────────────────────────────────┘
   │         │              │              │
   ▼         ▼              ▼              ▼
┌──────┐  ┌──────┐      ┌──────┐      ┌──────┐
│Slave1│─→│Slave2│─→ ···│SlaveN│─→ 返回 Master
└──────┘  └──────┘      └──────┘
```

> **跨领域类比**：EtherCAT 的飞行式帧处理类似于工厂流水线上的传送带——每个工位（从站）在产品（帧）经过时取出自己的零件、放入自己的产品，传送带不停。而传统 Ethernet 更像快递配送——每个包裹（帧）必须送到目的地、拆包、回复，延迟随路径增长。

**与 Franka UDP / UR TCP 的对比**：

| 维度 | EtherCAT | Franka UDP | UR TCP/RTDE |
|------|----------|------------|-------------|
| 实时性 | 硬实时（<1μs jitter） | 硬实时（~100μs jitter） | 软实时（~1ms jitter） |
| 拓扑 | 菊花链/星型/树型 | 点对点 | 点对点 |
| 从站数 | 理论 65535 | 1 | 1 |
| 协议层 | 数据链路层（L2） | 网络层（L3） | 传输层（L4） |

### ros2_control + EtherCAT 集成

```
ros2_control RT Loop
     │
     ▼
┌─────────────────────────────────┐
│ EtherCAT SystemInterface        │
│  read():  master->receive()     │
│           解析 PDO → hw_states_ │
│  write(): hw_cmds_ → 打包 PDO   │
│           master->send()        │
└────────────────┬────────────────┘
                 │
                 ▼
┌─────────────────────────────────┐
│ EtherCAT Master Library          │
│ (SOEM / IgH EtherLab / Acontis) │
│  - DC 分布式时钟同步             │
│  - PDO 映射配置                  │
│  - 状态机 INIT→PREOP→SAFEOP→OP  │
└─────────────────────────────────┘
         │ raw Ethernet (L2)
         ▼
    物理 EtherCAT 从站
```

**开源 EtherCAT Master 对比**：

| 库 | 许可证 | 特点 | ros2_control 集成 |
|----|--------|------|-------------------|
| **SOEM** | GPLv2 | 轻量、用户态、易于集成 | ethercat_driver_ros2 (ICube) |
| **IgH EtherLab** | GPLv2 | 内核态、DC 精度最高 | 需要自行封装 |
| **Acontis** | 商业 | 跨平台、认证 | 厂商提供 |

**ethercat_driver_ros2**（ICube-Robotics，~200★）是目前最成熟的 ros2_control + EtherCAT 开源集成：

```xml
<ros2_control name="EtherCATSystem" type="system">
  <hardware>
    <plugin>ethercat_driver/EthercatDriver</plugin>
    <param name="master_id">0</param>
    <param name="control_frequency">1000</param>
  </hardware>
  <joint name="joint1">
    <state_interface name="position"/>
    <command_interface name="position"/>
    <ec_module name="Elmo_Gold">
      <plugin>ethercat_generic_plugins/EcCiA402Drive</plugin>
      <param name="slave_position">1</param>
    </ec_module>
  </joint>
</ros2_control>
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：EtherCAT 需要 root 权限或 CAP_NET_RAW
   错误做法：以普通用户运行 ros2_control_node
   现象：EtherCAT master 无法打开原始套接字
   正确做法：
     sudo setcap cap_net_raw+ep $(which ros2_control_node)
```

```
💡 概念误区：认为 EtherCAT 就是"更快的以太网"
   新手想法："用以太网线缆，所以和 TCP/IP 差不多"
   实际上：EtherCAT 工作在 OSI 第 2 层，完全绕过 TCP/IP 协议栈。
          不使用 IP 地址、不经过路由器、不走操作系统网络栈——
          这就是亚微秒级确定性延迟的来源。
```

### 练习

1. **[A 型]** 用 SOEM 的 `slaveinfo` 工具扫描 EtherCAT 从站（虚拟机练习 master 初始化）。

2. **[思考题]** EtherCAT 的分布式时钟（DC）如何确保多个从站的采样时刻一致？这对 6 轴同步运动有什么意义？

---

## M12.9 RL 策略部署：LibTorch vs ONNX Runtime ⭐⭐

### 动机

ForwardCommandController 可以直接转发 RL 策略的输出。但策略模型如何从 Python 训练环境导出、在 C++ 中加载和推理、最终通过 ros2_control 发送给硬件——这个完整的部署流水线需要详细讲解。

### 部署架构（CRISP 模式）

CRISP（Control at High Rate, Inference at Slow Pace）是 RL 部署的标准架构：

```
┌────────────────────────────────────────────────────────┐
│               GPU 工作站 (5-50 Hz)                      │
│                                                         │
│  1. 组装观测向量 (obs)                                  │
│     - 关节位置/速度 (从 /joint_states)                  │
│     - 基座姿态 (从 IMU/EKF)                            │
│     - 视觉特征 (从相机)                                 │
│                                                         │
│  2. 前向推理: action = policy.forward(obs)              │
│                                                         │
│  3. 动作后处理                                          │
│     - Action scale: target = default + a * scale        │
│     - EMA 滤波: a_t = alpha*a_t + (1-alpha)*a_prev     │
│     - 关节限位裁剪                                      │
│                                                         │
│  4. 发布到 ForwardCommandController                     │
└──────────────────────┬──────────────────────────────────┘
                       │ ROS2 topic (50 Hz)
                       ▼
┌────────────────────────────────────────────────────────┐
│              RT 工作站 (500-1000 Hz)                    │
│                                                         │
│  ros2_control ForwardCommandController                  │
│  (重复上次 action 直到新 action 到达 = decimation)      │
│            │                                            │
│            ▼                                            │
│      Hardware write()                                   │
└────────────────────────────────────────────────────────┘
```

**Decimation 的含义**：策略以 50 Hz 输出 action，但硬件以 500 Hz 运行。在两次策略推理之间的 10 个 RT 周期里，ForwardCommandController 重复使用上一次的 action。这种「重复保持」在 IsaacGym/IsaacLab 训练时已经内置——训练时 `decimation=10`。

### LibTorch C++ 部署

**导出流程**：

```python
# 训练侧 (Python)
import torch

class Policy(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.mlp = torch.nn.Sequential(
            torch.nn.Linear(48, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 12),
        )

    def forward(self, obs):
        return self.mlp(obs)

policy = Policy()
policy.load_state_dict(torch.load("policy_best.pth"))
policy.eval()

# trace 导出
dummy_input = torch.zeros(1, 48)
traced = torch.jit.trace(policy, dummy_input)
traced.save("policy.pt")
```

**C++ 推理侧**：

```cpp
#include <torch/script.h>
#include <vector>

class RLPolicyNode {
public:
  RLPolicyNode(const std::string& model_path) {
    // 加载模型（初始化时，不在 RT 循环中）
    policy_ = torch::jit::load(model_path);
    policy_.eval();
    // 预分配 tensor
    obs_tensor_ = torch::zeros({1, 48});
    action_prev_ = torch::zeros({1, 12});
  }

  std::vector<double> infer(const std::vector<double>& obs) {
    // !! 非 RT 线程 !! ——LibTorch 内部有 malloc
    auto obs_data = obs_tensor_.accessor<float, 2>();
    for (size_t i = 0; i < 48; i++) {
      obs_data[0][i] = static_cast<float>(obs[i]);
    }

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_tensor_);
    auto action_tensor = policy_.forward(inputs).toTensor();

    // EMA 滤波
    constexpr float alpha = 0.3f;
    action_tensor =
      alpha * action_tensor + (1.0f - alpha) * action_prev_;
    action_prev_ = action_tensor.clone();

    auto action_data = action_tensor.accessor<float, 2>();
    std::vector<double> action(12);
    for (size_t i = 0; i < 12; i++) {
      action[i] = static_cast<double>(action_data[0][i]);
    }
    return action;
  }

private:
  torch::jit::script::Module policy_;
  torch::Tensor obs_tensor_;
  torch::Tensor action_prev_;
};
```

### ONNX Runtime C++ 部署

**导出流程**：

```python
dummy_input = torch.zeros(1, 48)
torch.onnx.export(
    policy, dummy_input, "policy.onnx",
    input_names=["obs"],
    output_names=["action"],
    dynamic_axes={
      "obs": {0: "batch"},
      "action": {0: "batch"}
    },
    opset_version=17,
)
```

**C++ 推理侧**：

```cpp
#include <onnxruntime_cxx_api.h>
#include <vector>

class RLPolicyONNX {
public:
  RLPolicyONNX(const std::string& model_path) {
    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "rl_policy");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(
      env_, model_path.c_str(), opts);
    obs_data_.resize(48, 0.0f);
    action_prev_.resize(12, 0.0f);
  }

  std::vector<double> infer(const std::vector<double>& obs) {
    for (size_t i = 0; i < 48; i++) {
      obs_data_[i] = static_cast<float>(obs[i]);
    }

    auto memory_info = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 2> shape{1, 48};
    auto input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, obs_data_.data(), obs_data_.size(),
      shape.data(), shape.size());

    const char* input_names[] = {"obs"};
    const char* output_names[] = {"action"};
    auto outputs = session_->Run(
      Ort::RunOptions{nullptr},
      input_names, &input_tensor, 1,
      output_names, 1);

    float* out = outputs[0].GetTensorMutableData<float>();
    constexpr float alpha = 0.3f;
    std::vector<double> action(12);
    for (size_t i = 0; i < 12; i++) {
      float filtered =
        alpha * out[i] + (1.0f - alpha) * action_prev_[i];
      action_prev_[i] = filtered;
      action[i] = static_cast<double>(filtered);
    }
    return action;
  }

private:
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<float> obs_data_;
  std::vector<float> action_prev_;
};
```

### LibTorch vs ONNX Runtime 详细对比

| 维度 | LibTorch (.pt) | ONNX Runtime (.onnx) |
|------|---------------|---------------------|
| **依赖大小** | ~500 MB | ~50 MB |
| **GPU 加速** | CUDA 原生 | CUDA / TensorRT / OpenVINO |
| **Jetson 部署** | 支持但较重 | TensorRT EP 最优 |
| **PyTorch 兼容** | 完美 | 需要 onnx.export，复杂模型可能需调整 |
| **动态控制流** | TorchScript 支持 | ONNX opset 有限支持 |
| **推理延迟** (MLP 48→12) | ~0.5-1.0 ms (CPU) | ~0.3-0.5 ms (CPU) |
| **代表项目** | rl_sar (~1.2k★) | LimX tron1-rl-deploy |

**选型指南**：
- **原型开发 / 研究**：LibTorch
- **产品部署 / Jetson**：ONNX Runtime
- **复杂模型（Transformer/VLA）**：LibTorch

### Action Scale 与 EMA 滤波

RL 策略的输出通常是**关节位置偏移**：

$$q_{target} = q_{default} + action \times scale$$

**为什么用偏移而非绝对位置？**
1. 策略输出接近零均值，梯度更稳定
2. scale 控制了每步最大关节移动量
3. 默认角度提供安全基线

**EMA 动作滤波**平滑输出：

$$a_{filtered}(t) = \alpha \cdot a_{raw}(t) + (1 - \alpha) \cdot a_{filtered}(t-1)$$

| alpha 值 | 效果 | 适用场景 |
|----------|------|---------|
| 0.2 | 强平滑，延迟大 | 低速精密任务 |
| 0.5 | 中等平滑 | Locomotion |
| 1.0 | 无滤波 | 快速响应力控 |

> **反事实推理**：如果不做 EMA 滤波会怎样？在仿真中可能看不出问题，但在真机上策略输出的高频抖动会导致电机异响、机械振动、甚至触发安全限制。EMA 是 sim-to-real 部署中最简单有效的平滑手段。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：在 RT 线程中运行推理
   错误做法：把 policy.forward() 放在 ros2_control 的 update() 中
   现象：推理耗时不确定（0.3-2ms），导致 RT 循环偶发超时
   根本原因：LibTorch/ONNX Runtime 内部有 malloc 和线程池调度
   正确做法：
     - 推理在单独的非 RT 线程中运行
     - 用 lock-free 环形缓冲区传递 action 给 RT 线程
     - RT 线程的 ForwardCommandController 读取最新 action
```

```
💡 概念误区：认为 decimation 是性能妥协
   新手想法："策略 50Hz 而硬件 500Hz，说明策略不够快"
   实际上：Decimation 是训练时就内置的设计选择。策略学到的是
          「在 10 步重复条件下的最优行为」。如果部署时改成每步都
          推理，反而可能破坏策略的预期行为。
```

```
⚠️ 编程陷阱：训练和部署的观测归一化不一致
   错误做法：训练时用 RunningMeanStd 归一化，部署时用原始观测
   现象：策略输出完全无意义的动作
   正确做法：
     1. 训练时保存归一化参数（均值、标准差）
     2. 部署时用相同参数归一化观测
     3. 或在导出模型时把归一化层包含在模型内
```

### 练习

1. **[A 型·RL 部署]** 训练一个简单 RL 策略（如 Isaac Lab 的 Franka reach task），导出 .pt 和 .onnx。写 C++ 推理节点，对比两种格式的推理延迟。

2. **[A 型·滤波对比]** 用不同 EMA alpha 值（0.1, 0.3, 0.5, 0.8, 1.0）滤波同一策略输出。绘制关节加速度曲线，分析 alpha 对平滑性和响应速度的影响。

3. **[跨章综合题]** 结合 M11（实时 C++）、M12.3（SystemInterface）、M12.9（RL 部署），设计完整的 RL 部署系统：(a) RT 线程运行 ros2_control（1kHz），(b) 非 RT 线程运行推理（50Hz），(c) 用 lock-free 环形缓冲区传递 action。画出线程模型和数据流图。

---

## M12.10 Chainable Controllers 实战与 gz_ros2_control 最新集成 ⭐⭐⭐

### Chainable Controllers 深入理解

Chainable Controllers 是 ros2_control 在 ROS2 Humble/Iron 之后引入的重要架构演进。它解决了一个长期痛点：**如何让一个控制器的输出直接作为另一个控制器的输入，而不经过硬件层**。

**为什么需要链式控制器**：考虑一个典型的力控场景——外环是导纳控制器（根据力矩传感器输出位置偏差），内环是关节轨迹跟踪控制器（跟踪位置命令）。如果没有链式控制器，导纳控制器必须通过 ROS2 Action 把轨迹发给 JTC——这引入了毫秒级通信延迟，在 1kHz 的力控回路中不可接受。

Chainable Controllers 允许导纳控制器直接把位置命令写入 JTC 的 reference interfaces（参考接口），JTC 在同一个 RT 循环中立刻读取并执行——零通信延迟，全部在进程内完成。

**链式控制器的数据流**：

```
Admittance Controller (外环)
  ├── 读取: F/T 传感器 state interfaces
  ├── 计算: 导纳模型 → 位置偏差
  └── 写入: JTC 的 reference interfaces (不是硬件 command interfaces!)
        │
        ▼
JTC (内环)
  ├── 读取: 关节位置 state interfaces + reference interfaces
  ├── 计算: PID 跟踪
  └── 写入: 硬件 command interfaces
        │
        ▼
Hardware Interface → 电机
```

> **本质洞察**：Chainable Controllers 的核心设计理念不是"让控制器串联"——而是**让控制器在 RT 循环内共享数据路径，消除任何通信中间层**。Reference interfaces 本质上是进程内的共享内存指针，和 state/command interfaces 一样——只是它们的生产者和消费者都是控制器，而非硬件。这是 ros2_control 架构中"抽象层"理念的自然延伸：不仅硬件可以被抽象，控制器之间的数据传递也可以被抽象为统一的 interface 机制。

**链式控制器的配置示例**：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # 1kHz

    admittance_controller:
      type: admittance_controller/AdmittanceController

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
    # 声明 reference interfaces，供上游链式控制器写入
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

admittance_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7
    # 指定链式下游：输出到 JTC 的 reference interfaces
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    ft_sensor:
      name: ft_sensor
      frame:
        id: tool0
    # 链式配置：admittance → JTC
    chainable_command_interface_name: position
```

**反事实推理**：如果不用 Chainable Controllers 而是用 ROS2 topic/action 连接两个控制器会怎样？外环输出的位置命令经过 DDS 序列化/反序列化、传输、反序列化到内环——延迟约 1-5ms。对于 1kHz 的力控回路，每个控制周期只有 1ms 预算，光通信就占满了。结果是力控带宽严重受限（外环频率被迫降到 100-200Hz），系统对接触力的响应变慢，安全性降低。

### gz_ros2_control 最新集成（Gazebo Harmonic + Ionic）

随着 Gazebo（原 Ignition Gazebo）升级到 Harmonic（2024）和 Ionic（2025）系列，`gz_ros2_control` 的集成方式也在演进。

**当前推荐架构**（Gazebo Harmonic / ROS2 Rolling/Jazzy/Kilted）：

| 组件 | 版本 | 说明 |
|------|------|------|
| Gazebo | Harmonic (gz-sim 8.x) 或 Ionic | 物理仿真引擎 |
| gz_ros2_control | 1.x (Humble) / 2.x (Rolling) | ros2_control 与 Gazebo 桥梁 |
| ros2_control | Humble/Iron/Jazzy/Kilted | 控制框架 |

**关键变化**：
1. **插件声明统一**：Harmonic 使用 `<plugin filename="gz_ros2_control-system">`（注意：不再是旧的 `libgazebo_ros2_control.so`）
2. **SDF 优先**：Harmonic 推荐 SDF 而非 URDF 直接使用；但通过 `ros_gz_sim` 的 `create` 服务，URDF/Xacro 仍然可以自动转换为 SDF 加载
3. **传感器插件集成**：gz_ros2_control 现已支持力矩传感器、IMU 等 sensor interface 的仿真后端
4. **Mimic joints**：Harmonic 的 gz_ros2_control 支持 `mimic` 关节（如夹爪的左右手指镜像运动），无需额外 workaround

> **跨领域类比**：gz_ros2_control 的角色类似于 Docker 的虚拟网络——它让 ros2_control 控制器"以为"自己在和真实硬件通信，实际上对面是 Gazebo 的物理引擎。这种"虚拟硬件"模式是 sim-to-real 的基础——M15 的 Mini-Manip 项目正是通过切换这一层来实现 Gazebo/Mock/真机三态切换。

---

## 本章小结

| 知识点 | 核心内容 | 难度 |
|--------|---------|------|
| M12.1 架构全景 | 四大组件、RT 主循环、独占/共享接口 | ⭐⭐ |
| M12.2 硬件组件类型 | System/Actuator/Sensor 三种类型及选型 | ⭐⭐ |
| M12.3 自定义 SystemInterface | 生命周期回调、RT-safe read/write | ⭐⭐ |
| M12.4 常用控制器 | JTC/Forward/Admittance/Gripper/Broadcaster | ⭐⭐ |
| M12.5 generate_parameter_library | 类型安全参数生成 | ⭐⭐⭐ |
| M12.6 Chainable Controllers | 链式控制器、错误恢复 | ⭐⭐⭐ |
| M12.7 参考驱动精读 | libfranka vs ur_robot_driver | ⭐⭐ |
| M12.8 EtherCAT 集成 | 飞行式帧处理、SOEM/IgH | ⭐⭐⭐⭐ |
| M12.9 RL 部署 | LibTorch vs ONNX、CRISP 架构 | ⭐⭐ |

## 累积项目：本章新增模块

**Mini-Manip 项目进度**：本章新增 ros2_control 配置层。

```
mini_manip_ws/
├── src/
│   ├── mini_manip_hardware/        # 本章新增
│   │   ├── include/.../mock_system.hpp
│   │   ├── src/mock_system.cpp
│   │   ├── plugin.xml
│   │   └── CMakeLists.txt
│   ├── mini_manip_bringup/         # 本章新增
│   │   ├── config/controllers.yaml
│   │   ├── config/ros2_control.xacro
│   │   └── launch/robot_bringup.launch.py
│   └── ...
```

新增内容：
1. Mock 硬件驱动 + JTC + JointStateBroadcaster 配置
2. ForwardCommandController（为 M12.9 RL 部署预留）
3. Launch 文件支持 `use_mock_hardware` 参数切换

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| ros2_control 官方文档 (`control.ros.org`) | ⭐ | 架构概览、教程 |
| ros2_control_demos 仓库 | ⭐⭐ | 15+ 示例 |
| UR ROS2 Driver 源码 | ⭐⭐ | 生产级驱动金标准 |
| franka_ros2 源码 | ⭐⭐ | Franka 驱动 |
| rl_sar 仓库 (fan-ziqi) | ⭐⭐ | RL 部署框架 |
| ethercat_driver_ros2 (ICube) | ⭐⭐⭐ | EtherCAT 集成 |
| Bence Magyar, ROSCon 2022 talk | ⭐ | 架构设计官方演讲 |

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| CM 启动报 "plugin not found" | pluginlib 路径错误 | 1. 检查 plugin.xml 路径 2. `ros2 plugin list` 列出已注册插件 | M12.3 |
| 激活报 "interface already claimed" | 两个控制器争夺同一 command interface | 1. `ros2 control list_controllers` 2. 先 deactivate 旧控制器 | M12.4 |
| JTC 执行 abort | 跟踪误差超容差 | 1. 检查 constraints 参数 2. rqt_plot 观察偏差 3. 检查 PID | M12.4 |
| 控制器切换时跳变 | 新控制器初始命令不匹配 | 1. on_activate() 中同步位置 2. mock hw 先验证 | M12.3 |
| RL 真机行为异常 | 观测归一化不一致 | 1. 打印对比仿真/真机观测 2. 检查归一化参数 3. 检查 action_scale | M12.9 |
| EtherCAT "no slaves" | 权限不足或网卡配置错误 | 1. `sudo ethercat slaves` 2. 检查网卡名 3. 确认 CAP_NET_RAW | M12.8 |
| RT 循环 jitter > 1ms | read/write 中有阻塞或推理在 RT 线程 | 1. cyclictest 测基线 2. ftrace 追踪 3. 检查 malloc/mutex | M12.1 |
