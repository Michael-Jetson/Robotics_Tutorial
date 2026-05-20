> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M13 BehaviorTree.CPP 深度

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回 `02_基础/10_C++语言核心` / `02_基础/30_软件工程` / `02_基础/50_ROS2工程化` 复习）

1. 什么是 Composite Pattern？它如何用统一接口处理叶子节点和容器节点？（`02_基础/30_软件工程/10_设计模式与高级惯用法`）
2. C++ 共享库如何通过 `dlopen`/符号导出在运行时加载？ROS `pluginlib` 在此基础上又增加了什么？（`02_基础/50_ROS2工程化/40_硬件集成与RL部署`）
3. ROS2 Action 的三个组成部分（Goal/Feedback/Result）分别表达什么语义？（`02_基础/50_ROS2工程化/40_硬件集成与RL部署`）
4. `std::any` 是什么？它如何在运行时实现类型安全的多态存储？（`02_基础/10_C++语言核心/10_类型系统与值类别推导`）
5. 有限状态机（FSM）的状态数与转移数在复杂系统中如何增长？（`02_基础/30_软件工程/10_设计模式与高级惯用法`）

## 本章目标

学完本章后，你能够：
1. **深刻理解** BehaviorTree 与 FSM 的本质差异，能在实际项目中做出正确的选型决策
2. **掌握** BT.CPP v4 的异步 ticking 执行模型和四种节点类型
3. **独立编写** 自定义的 Action/Condition 节点，并用 BT.CPP 原生插件机制或 ROS 集成方式注册
4. **设计** 包含错误恢复的完整 pick-and-place 行为树
5. **理解** BT.CPP 与 ROS2 Action Server 的集成模式
6. **使用** Groot2 进行可视化编辑和日志回放调试

---

## M13.1 行为树 vs 有限状态机 ⭐⭐

### 动机 ⭐⭐

机器人操作任务不是一条直线——它是一棵充满分支的决策树。考虑一个看似简单的 pick-and-place 任务：

```
1. 检测物体位姿
2. 规划抓取路径 → 如果规划失败？重新检测？换抓取点？
3. 移动到抓取位姿 → 如果运动规划失败？避障后重试？
4. 闭合夹爪 → 如果抓取失败（物体滑落）？重新检测？
5. 搬运到目标位置 → 如果中途检测到新障碍物？
6. 放置物体 → 如果放置失败？
7. 返回初始位姿
```

每一步都可能失败，每种失败都需要不同的恢复策略。如何组织这种复杂的控制逻辑？

### 如果用 FSM 会怎样 ⭐⭐

最直觉的方案是有限状态机（FSM）：每个步骤是一个状态，步骤之间的转换是边。

```
[检测] ─成功→ [规划] ─成功→ [移动] ─成功→ [抓取] ─成功→ [搬运] ─成功→ [放置]
  │            │           │           │            │
  失败         失败        失败         失败         失败
  │            │           │           │            │
  ▼            ▼           ▼           ▼            ▼
 [报错]      [重检测]    [重规划]    [重检测]     [重规划]
```

问题立刻暴露：

1. **状态数爆炸**：5 个动作 × 3 种错误恢复 × 2 种感知条件 = 30+ 状态、100+ 转移边。每增加一个新的错误处理逻辑，都需要增加状态和转移。

2. **修改困难**：要在「搬运」和「放置」之间插入一个「视觉验证」步骤，需要修改所有指向「放置」的转移边。在大型 FSM 中，这种修改极易引入 bug。

3. **复用困难**：如果另一个任务也需要「检测 → 抓取」的子流程，你无法直接复用——因为 FSM 的状态转移是全局的。

4. **反应性差**：如果在「搬运」途中传感器检测到危险（人靠近），你需要为每个状态都加上「danger → 暂停」的转移边——这意味着修改所有状态。

> **本质洞察**：FSM 的根本问题不是「不能表达」复杂逻辑——任何图灵完备的系统都能。问题在于**可维护性**：FSM 的转移是全局耦合的（任何状态都可以转移到任何状态），而行为树的控制是局部组合的（每个子树独立封装，通过控制节点组合）。这类似于面向对象编程中「全局变量」vs「封装」的区别。

### 行为树如何解决 ⭐⭐

BT 用**树状结构 + 异步 tick** 替代 FSM 的**图状结构 + 状态转移**：

```xml
<BehaviorTree ID="PickAndPlace">
  <Sequence>
    <RetryUntilSuccessful num_attempts="3">
      <Sequence>
        <DetectObject/>
        <PlanGrasp/>
      </Sequence>
    </RetryUntilSuccessful>
    <Fallback>
      <ExecuteGrasp/>
      <Sequence>
        <DetectObject/>
        <PlanGrasp/>
        <ExecuteGrasp/>
      </Sequence>
    </Fallback>
    <MoveToPlace/>
    <PlaceObject/>
  </Sequence>
</BehaviorTree>
```

**关键优势**：

| 维度 | FSM | Behavior Tree |
|------|-----|---------------|
| 添加错误恢复 | 修改全局转移图 | 在局部包一层 Fallback |
| 插入新步骤 | 修改所有指向后续状态的转移 | 在 Sequence 中插入子节点 |
| 复用子流程 | 复制粘贴状态子图 | 将子树抽取为 SubTree |
| 反应性（实时条件检查） | 为每个状态加转移边 | 用 ReactiveSequence 自动检查 |
| 可视化 | 状态转移图（边多时难以理解） | 树状图（层次清晰） |
| 新增一个动作 | O(N) 修改（N=已有状态数） | O(1) 修改（只改局部子树） |

**不是 X 而是 Y**：BT 的价值不是「功能比 FSM 更强」——两者在计算能力上等价。BT 的价值是「在复杂任务中更容易维护和扩展」。如果你的任务只有 3-5 个状态且不需要错误恢复，FSM 可能更简单直接。

### 历史背景 ⭐

行为树最初由游戏 AI 领域发展而来——2004 年 Halo 2 的 AI 系统使用了与 BT 结构相似的层次化决策架构。2005 年 Damian Isla 在 GDC（游戏开发者大会）上介绍了该系统的设计思路，推动了 BT 概念的普及。此后 BT 被 Unreal Engine（2012）等主流游戏引擎采纳为标准 AI 架构。

机器人领域的采用较晚。2014 年 Michele Colledanchise 和 Petter Ögren 将 BT 引入机器人控制，发表了理论分析论文并出版了专著 "Behavior Trees in Robotics and AI"（2018）。2018 年 Davide Faconti 在 Eurecat 启动了 BT.CPP 库的开发（与 IIT 的 Colledanchise 合作），2019 年后逐步成熟并被 ROS2 的 Nav2 导航栈采用为顶层任务编排框架，从此在 ROS 生态中广泛普及。

### 选型决策流程 ⭐⭐

```
你的任务有多复杂？
    │
    ├── ≤5 个状态，无错误恢复，逻辑线性
    │   └── FSM（简单直接，维护成本低）
    │
    ├── 5-15 个状态，需要部分错误恢复
    │   └── BT（可维护性开始超过 FSM）
    │
    ├── >15 个状态，多种错误恢复，需要反应性
    │   └── BT（FSM 在此规模下几乎不可维护）
    │
    └── 实时安全关键系统（汽车/航空）
        └── FSM（可形式化验证，有成熟的安全认证框架）
        注意：BT 的形式化验证工具正在发展中（CONVINCE 项目）
```

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为"BT 可以完全替代 FSM"
   新手想法："BT 更先进，所有场景都应该用 BT"
   实际上：FSM 在以下场景仍然更合适：
          1. 简单线性流程（3-5 步，无分支）
          2. 需要形式化验证的安全关键系统
          3. 硬件状态机（如 EtherCAT 从站 INIT→PREOP→SAFEOP→OP）
          4. 底层通信协议（如 TCP 握手状态）
   正确思维：FSM 适合底层/简单/安全关键，BT 适合上层/复杂/任务编排。
            两者经常共存——BT 在顶层编排任务，底层硬件用 FSM 管理状态。
```

```
💡 概念误区：认为"BT 只是 FSM 的树状画法"
   新手想法："BT 就是把 FSM 的状态画成树，本质一样"
   实际上：BT 的 tick 机制与 FSM 的状态转移是完全不同的执行模型。
          FSM 在当前状态等待事件触发转移；
          BT 每次 tick 从根节点重新评估整棵树。
          这意味着 BT 天生支持反应性（每次 tick 都重新检查条件），
          而 FSM 需要为每个状态手动添加条件检查的转移边。
```

```
⚠️ 编程陷阱：用全局变量在 BT 节点间传递数据
   错误做法：定义全局 struct SharedData 让所有节点读写
   现象：节点之间高度耦合，无法独立测试，SubTree 无法复用
   根本原因：绕过了 Blackboard 的隔离机制
   正确做法：使用 Blackboard + Port 机制传递数据（M13.3 详细讲解）
```

### 练习

1. **[A 型]** 为一个「巡逻 + 充电」任务分别画出 FSM 和 BT 的设计。FSM 需要处理：巡逻中电池低 → 去充电 → 充满回来继续巡逻。BT 用 ReactiveSequence 实现条件检查。对比两种设计的节点/转移数量。

2. **[思考题]** Nav2（ROS2 导航栈）从 ROS1 的 FSM 架构切换到了 BT 架构。这次切换解决了什么问题？提示：考虑 recovery behaviors 的可扩展性。

3. **[思考题]** 如果需要在安全关键的工业焊接场景中使用 BT，你需要额外做什么来保证安全性？（提示：CONVINCE 项目的形式化验证方法）

---

## M13.2 BT.CPP v4 异步 Ticking 执行模型 ⭐⭐

### 动机 ⭐⭐

理解了 BT vs FSM 的宏观差异后，我们深入 BT.CPP 的执行模型——这是理解所有后续内容的基础。

### 三态语义 ⭐⭐

BT.CPP 中，每个 `TreeNode::tick()` 返回三种状态之一：

| 状态 | 含义 | 类比 |
|------|------|------|
| `SUCCESS` | 本动作完成 | 函数返回 true |
| `FAILURE` | 本动作失败 | 函数返回 false |
| `RUNNING` | 本动作还在执行 | `std::future` 的 `wait_for` 返回 not_ready |

**RUNNING 是 BT 与普通函数调用的关键区别**。在同步编程中，函数调用要么成功要么失败，没有「正在执行」的概念。但机器人动作（如「移动到目标位姿」）可能需要数秒完成——你不能让整棵树阻塞等待。RUNNING 状态允许节点说「我还没做完，下次 tick 再来问我」。

> **跨领域类比**：RUNNING 状态类似于操作系统的进程调度——一个进程可以处于「运行中」但被时间片打断，下次调度时继续执行。BT 的 tick 循环就像操作系统的调度器，每次 tick 轮询所有 RUNNING 节点的进度。区别在于 OS 调度是抢占式的（任何时候可以打断），BT tick 是协作式的（节点在 tick 返回时才让出控制权）。

### Tick 循环 ⭐⭐

顶层代码以固定频率（通常 10-100 Hz）调用 BT.CPP v4 的公开 tick API，例如 `tree.tickOnce()`。如果希望库内部循环到非 RUNNING 状态，可用 `tree.tickWhileRunning()`；需要严格单次 tick 时可用 `tree.tickExactlyOnce()`。

```cpp
BT::BehaviorTreeFactory factory;
// 注册所有节点类型
factory.registerNodeType<DetectObject>("DetectObject");
factory.registerNodeType<PlanGrasp>("PlanGrasp");
// ... 更多注册 ...

auto tree = factory.createTreeFromFile("my_tree.xml");

// 主循环: 10 Hz tick
BT::NodeStatus status = BT::NodeStatus::RUNNING;
while (status == BT::NodeStatus::RUNNING) {
  status = tree.tickOnce();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
// 到这里: status 是 SUCCESS 或 FAILURE
```

每次 tick 时，整棵树自顶向下遍历。控制节点根据子节点的返回值决定下一步的行为。

### 四种节点类型 ⭐⭐

BT.CPP 的节点分为四类：

```
         TreeNode (基类)
         ┌────┴────┐
    ControlNode   LeafNode
    ┌────┴────┐   ┌────┴────┐
Sequence   ...  ActionNode  ConditionNode
Fallback        (异步动作)  (同步条件检查)
Parallel
Reactive...
         │
    DecoratorNode
    (修饰单个子节点)
```

**1. Control Nodes**（控制节点）——定义子节点的执行策略

| 控制节点 | 行为 | C++ 类比 |
|---------|------|---------|
| **Sequence** | 依次执行，遇 FAILURE 立即返回 FAILURE | `if (a && b && c)` |
| **Fallback** (Selector) | 依次尝试，遇 SUCCESS 立即返回 SUCCESS | `if (a \|\| b \|\| c)` |
| **Parallel** | 同一轮 tick 中按顺序 tick 所有子节点，按阈值决定结果 | 逻辑并行状态机 |
| **ReactiveSequence** | 每次 tick 从头重新评估 | `while (condition)` |
| **ReactiveFallback** | 每次 tick 从头重新评估 | 持续监控型 fallback |

**Sequence vs ReactiveSequence 的关键区别**：

```
Sequence:
  第 1 次 tick: A=SUCCESS → B=RUNNING (停在 B)
  第 2 次 tick: 跳过 A → B=RUNNING (继续等 B)
  第 3 次 tick: 跳过 A → B=SUCCESS → C 开始执行
  → A 只评估一次

ReactiveSequence:
  第 1 次 tick: A=SUCCESS → B=RUNNING (停在 B)
  第 2 次 tick: A=SUCCESS → B=RUNNING (重新检查 A!)
  第 3 次 tick: A=FAILURE → 立即返回 FAILURE (中断 B!)
  → A 每次 tick 都重新评估
```

ReactiveSequence 适合「持续检查安全条件」：

```xml
<ReactiveSequence>
  <IsBatteryOK/>        <!-- 每次 tick 都检查电池 -->
  <IsPathClear/>        <!-- 每次 tick 都检查路径 -->
  <NavigateToGoal/>     <!-- 长时间运行的导航任务 -->
</ReactiveSequence>
<!-- 如果电池不足或路径被阻挡，NavigateToGoal 立即中断 -->
```

> **反事实推理**：如果用普通 Sequence 代替 ReactiveSequence 会怎样？电池检查和路径检查只在 NavigateToGoal 开始前执行一次。如果导航途中电池耗尽或出现新障碍物，系统无法及时响应——在真实机器人中可能导致碰撞或搁浅。

**2. Action Nodes**（动作节点）——执行具体行为

BT.CPP v4 提供两种 Action 基类：

```cpp
// 同步 Action（tick 内完成，不返回 RUNNING）
class SimpleAction : public BT::SyncActionNode {
  BT::NodeStatus tick() override {
    // 做一些快速操作（如设置 Blackboard 值）
    return BT::NodeStatus::SUCCESS;
  }
};

// 异步 Action（可能需要多次 tick 才能完成）
class LongRunningAction : public BT::StatefulActionNode {
  // 第一次 tick 时调用
  BT::NodeStatus onStart() override {
    // 开始异步操作（如发送 ROS2 Action Goal）
    return BT::NodeStatus::RUNNING;
  }

  // 后续每次 tick 时调用（前提是上次返回 RUNNING）
  BT::NodeStatus onRunning() override {
    if (action_done_) return BT::NodeStatus::SUCCESS;
    if (action_failed_) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
  }

  // 被父节点中断时调用
  void onHalted() override {
    cancel_action();  // 取消正在执行的操作
  }
};
```

`StatefulActionNode` 的三个回调与 ROS2 Action 的三个阶段完美对应：
- `onStart()` → 发送 Goal
- `onRunning()` → 等待 Feedback / 检查 Result
- `onHalted()` → 取消 Goal

**3. Condition Nodes**（条件节点）——检查世界状态

Condition 节点是同步的——每次 tick 立即返回 SUCCESS 或 FAILURE，绝不返回 RUNNING。

```cpp
class IsObjectDetected : public BT::ConditionNode {
  BT::NodeStatus tick() override {
    auto pose = getInput<geometry_msgs::msg::Pose>("object_pose");
    if (pose && is_valid(pose.value())) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};
```

**4. Decorator Nodes**（装饰器节点）——修饰单个子节点的行为

| Decorator | 行为 |
|-----------|------|
| `Inverter` | 反转子节点结果（SUCCESS↔FAILURE） |
| `Repeat` | 重复执行子节点 N 次 |
| `RetryUntilSuccessful` | 失败后重试，最多 N 次 |
| `Timeout` | 超时后返回 FAILURE |
| `ForceSuccess` | 无论子节点结果都返回 SUCCESS |
| `Delay` | 等待一段时间后执行子节点 |

> **跨领域类比**：Decorator 模式在 BT 中的应用与 `02_基础/30_软件工程/10_设计模式与高级惯用法` 中学过的 Decorator 设计模式完全一致——不修改原始节点，通过包装增强行为。`RetryUntilSuccessful` 就像网络请求的重试中间件，`Timeout` 就像 gRPC 的 deadline。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：在 ConditionNode 中返回 RUNNING
   错误做法：条件节点的 tick() 返回 RUNNING
   现象：控制节点行为异常（Sequence 卡住不往下走）
   根本原因：BT.CPP 约定条件节点必须同步返回 SUCCESS/FAILURE。
            返回 RUNNING 违反约定，导致未定义行为。
   正确做法：如果需要异步条件检查，用 ActionNode + Blackboard 组合
```

```
💡 概念误区：认为 Parallel 是"多线程并发执行"
   新手想法："Parallel 节点在多个线程中同时运行子节点"
   实际上：BT.CPP 的 Parallel 是逻辑并行：同一轮 tick 中按顺序
          tick 子节点，并允许多个子节点同时保持 RUNNING。
          只有异步 Action 节点内部自己启动线程、ROS Action 或 I/O
          回调时，才会产生真正并发。
   工程影响：ControlNode 本身不是多核加速工具；线程安全问题来自
          异步 Action 的内部实现和共享 Blackboard 数据。
```

```
🧠 思维陷阱：认为 tick 频率越高越好
   新手想法："100 Hz tick 肯定比 10 Hz 好"
   实际上：tick 频率决定了反应速度和 CPU 开销的平衡。
          10 Hz: 100ms 反应延迟，CPU 开销极低——适合大多数操作任务
          50 Hz: 20ms 反应延迟——适合需要快速反应的移动机器人
          100 Hz: 10ms 反应延迟——只有安全关键场景才需要
          tick 的大部分时间花在轮询 RUNNING 节点上，
          如果节点逻辑简单（只检查一个标志），高频率没问题；
          如果节点逻辑复杂（如感知计算），高频率会成为瓶颈。
```

### 练习

1. **[A 型]** 实现一个简单的 BT：`Sequence(PrintHello → Wait2Seconds → PrintWorld)`。用 `StdCoutLogger` 观察 tick 过程。特别观察 Wait2Seconds 在多次 tick 中如何返回 RUNNING。

2. **[A 型]** 用 ReactiveSequence 实现「如果检测到人靠近就暂停机械臂运动」。条件节点检查距离 topic，动作节点执行 MoveIt plan。验证条件变化时动作是否被正确中断。

3. **[思考题]** BT.CPP 的单线程 tick 模型在 10 Hz 频率下，每次 tick 需要遍历整棵树。如果树中有 100 个节点，每个节点 tick 需要 0.1ms，一次完整遍历需要 10ms——已经占满 100ms 的 tick 周期的 10%。如何优化大型行为树的 tick 性能？（提示：考虑哪些节点实际上需要被 tick）

---

## M13.3 Blackboard 与 Port 类型安全数据流 ⭐⭐

### 动机

BT 节点之间需要传递数据（检测节点输出物体位姿，规划节点读取位姿并输出轨迹）。但 BT 的设计原则是**节点之间不直接调用方法**——每个节点只知道自己的输入和输出，不知道数据来自哪个节点。这种解耦让节点可以独立开发、测试和复用。

### Blackboard 机制

BT.CPP 使用 **Blackboard**（黑板）作为共享键值存储：

```
┌──────────────┐     write("pose", p)    ┌──────────────┐
│ DetectObject  │ ──────────────────────► │  Blackboard   │
│ (Producer)    │                         │               │
└──────────────┘     read("pose")        │ "pose" → Pose │
                   ◄──────────────────── │ "traj" → Traj │
┌──────────────┐                         │               │
│ PlanGrasp     │                         └──────────────┘
│ (Consumer)    │
└──────────────┘
```

> **跨领域类比**：Blackboard + Port 机制类似于 ROS2 的 topic 通信——Publisher 不知道谁在 Subscribe，中间通过命名空间解耦。但 Blackboard 是进程内同步访问，比 ROS topic 快得多且无序列化开销。另一个类比是 Redux 状态管理——全局 store 存储状态，组件通过 selector 读取、通过 action 写入。

### Port 声明与数据绑定

**Port 机制**使数据流在 XML 中可见、在编译时可检查：

```cpp
class DetectObject : public BT::SyncActionNode {
public:
  // 声明输入输出 Port
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("target_name",
        "要检测的物体名"),
      BT::OutputPort<geometry_msgs::msg::Pose>("detected_pose",
        "检测到的位姿"),
    };
  }

  BT::NodeStatus tick() override {
    auto target = getInput<std::string>("target_name");
    if (!target) {
      throw BT::RuntimeError("Missing input: target_name");
    }

    // 执行检测逻辑...
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.3;
    pose.orientation.w = 1.0;

    // 写入 Blackboard
    setOutput("detected_pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

class PlanGrasp : public BT::SyncActionNode {
public:
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("target_pose",
        "目标位姿"),
      BT::OutputPort<std::string>("grasp_plan",
        "抓取规划方案 ID"),
    };
  }

  BT::NodeStatus tick() override {
    auto pose = getInput<geometry_msgs::msg::Pose>(
      "target_pose");
    if (!pose) {
      return BT::NodeStatus::FAILURE;
    }
    // 使用 pose.value() 进行规划...
    setOutput("grasp_plan", "plan_001");
    return BT::NodeStatus::SUCCESS;
  }
};
```

**XML 中的数据绑定**使用 `{variable_name}` 语法：

```xml
<Sequence>
  <DetectObject target_name="red_cube"
                detected_pose="{object_pose}"/>
  <!-- DetectObject 将结果写入 Blackboard 的 "object_pose" 键 -->

  <PlanGrasp target_pose="{object_pose}"
             grasp_plan="{current_plan}"/>
  <!-- PlanGrasp 从 Blackboard 读 "object_pose"，
       写入结果到 "current_plan" -->

  <ExecuteGrasp plan_id="{current_plan}"/>
</Sequence>
```

### 类型安全机制

BT.CPP 使用 `std::any` 存储 Blackboard 中的值，并在 `getInput<T>()` 时做运行时类型检查：

```cpp
// Blackboard 内部简化实现
class Blackboard {
  std::unordered_map<std::string, Entry> storage_;

  struct Entry {
    std::any value;          // 类型擦除存储
    TypeInfo type_info;      // 编译时捕获的类型信息
    uint64_t sequence_id;   // 每次写入递增
  };
};

// getInput<T>() 的简化逻辑
template<typename T>
Expected<T> TreeNode::getInput(const std::string& key) {
  auto entry = config_.blackboard->get(key);
  if (!entry) return nonstd::make_unexpected("key not found");

  try {
    return std::any_cast<T>(entry->value);
  } catch (std::bad_any_cast&) {
    // 尝试字符串转换（BT.CPP 支持从字符串反序列化）
    return convertFromString<T>(entry->string_value);
  }
}
```

这比 `void*` 安全得多——`std::any_cast` 在类型不匹配时抛出异常而非产生未定义行为。同时，`convertFromString<T>` 机制允许在 XML 中直接写字面值（如 `target_name="red_cube"`），BT.CPP 会自动转换为对应的 C++ 类型。

### Stamped API（v4 新增）

BT.CPP v4 为 Blackboard entry 加入了时间戳和序列号：

```cpp
// 写入（自动更新 sequence_id）
setOutput("detected_pose", pose);

// 读取（带时间戳/序列号检查）。不同 BT.CPP v4 小版本中
// StampedValue 的字段名略有差异，先检查 Expected，再访问 value/seq/stamp。
auto stamped = getInputStamped<Pose>("detected_pose");
if (!stamped) {
  return BT::NodeStatus::FAILURE;
}
const auto& entry = stamped.value();
if (entry.seq > last_seen_seq_) {
  // 数据比上次新——使用新数据
  last_seen_seq_ = entry.seq;
  process(entry.value);
} else {
  // 数据未更新——跳过或使用缓存
}
```

这支持了**reactive programming**模式——节点可以知道「数据是不是比上次新了」。

### SubTree 的 Blackboard 隔离

SubTree 有独立的 Blackboard，需要通过 port remapping 显式暴露数据：

```xml
<BehaviorTree ID="PickObject">
  <!-- 这个 SubTree 内部的 Blackboard 是隔离的 -->
  <Sequence>
    <DetectObject detected_pose="{pose}"/>
    <PlanGrasp target_pose="{pose}"/>
  </Sequence>
</BehaviorTree>

<BehaviorTree ID="MainTree">
  <Sequence>
    <!-- 通过 port remapping 传递数据 -->
    <SubTree ID="PickObject"
             pose="{main_object_pose}"/>
    <!-- "main_object_pose" 是 MainTree 的 Blackboard 键，
         映射到 PickObject 内部的 "pose" 键 -->
  </Sequence>
</BehaviorTree>
```

这种隔离防止了不同 SubTree 之间的键名冲突——类似于编程语言中的命名空间或模块作用域。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Port 名称拼写不一致
   错误做法：C++ 中写 "detected_pose"，XML 中写 "detect_pose"
   现象：运行时 getInput 返回空值，节点莫名 FAILURE
   根本原因：BT.CPP 的 Port 匹配是字符串比较，拼写不一致就匹配不上
   正确做法：在 C++ 侧定义常量字符串，XML 中严格对照
   自检方法：BT.CPP 在 createTree 时会打印 Port 不匹配的 warning
```

```
💡 概念误区：认为 Blackboard 是全局变量
   新手想法："Blackboard 就是全局变量的集合，随便读写"
   实际上：每个 TreeNode 只能访问它声明的 Port 对应的 Blackboard 键。
          未声明的键无法通过 getInput/setOutput 访问。
          这是「声明式的访问控制」——比全局变量安全得多。
```

```
⚠️ 编程陷阱：忘记自定义类型的 convertFromString
   错误做法：在 Port 中使用自定义类型但不定义转换函数
   现象：XML 中写的字面值无法解析，getInput 返回空
   正确做法：为自定义类型特化 BT::convertFromString<T>
```

### 练习

1. **[A 型]** 实现三个节点的数据流：`DetectObject`（输出 pose）→ `PlanMotion`（读 pose，输出 trajectory）→ `ExecuteMotion`（读 trajectory）。验证数据通过 Blackboard 正确传递。

2. **[B 型]** 精读 `blackboard.h` 中 `std::any` 的使用。理解 `getInput<T>()` 如何做运行时类型检查。对比 `void*` 的不安全做法——写出 `void*` 版本的等价代码，说明哪些错误在 `std::any` 版本中会被捕获。

3. **[思考题]** Blackboard 的键是字符串，两个不相关节点如果使用同名键会产生冲突。BT.CPP 如何通过 SubTree 的 Blackboard 隔离来解决？如果确实需要跨 SubTree 共享数据呢？

---

## M13.4 XML DSL 与工厂注册 ⭐⭐

### 动机

BT 的核心设计原则是**行为与实现分离**——行为设计师在 XML 中定义树的拓扑，C++ 开发者实现每个节点的逻辑。这种分离让非程序员也能通过 Groot2 GUI 设计行为树。

### 工厂注册三种方式

BT.CPP 使用 `BehaviorTreeFactory` 作为节点的注册中心：

```cpp
BT::BehaviorTreeFactory factory;

// 方式 1: 静态注册（编译时确定）
factory.registerNodeType<DetectObject>("DetectObject");
factory.registerNodeType<PlanGrasp>("PlanGrasp");
factory.registerNodeType<ExecuteGrasp>("ExecuteGrasp");

// 方式 2: 动态插件加载（运行时从 .so 文件加载）
factory.registerFromPlugin("libmy_bt_nodes.so");

// 方式 3: 简单函数注册（适合简单节点）
factory.registerSimpleAction("SayHello",
  [](BT::TreeNode& node) -> BT::NodeStatus {
    std::cout << "Hello!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  });

// 从 XML 创建树
auto tree = factory.createTreeFromFile("pick_and_place.xml");
```

BT.CPP 原生插件不是 ROS `pluginlib` 插件。共享库中需要导出 `BT_REGISTER_NODES` 符号，`registerFromPlugin()` 通过 BT.CPP 的插件加载器找到这个符号并调用注册函数：

```cpp
#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DetectObject>("DetectObject");
  factory.registerNodeType<PlanGrasp>("PlanGrasp");
  factory.registerNodeType<ExecuteGrasp>("ExecuteGrasp");
}
```

只有当你把 BT 节点再包装成 ROS 包级插件、让其他 ROS 组件通过 XML class description 发现它时，才涉及 `pluginlib` 的 `plugin.xml` / export 机制。Nav2 等 ROS 集成可能同时使用 BT.CPP 插件和 ROS 包导出，但两层机制不要混为一谈。

**三种方式的适用场景**：

| 方式 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| 静态注册 | 简单直接，编译时检查 | 添加新节点需重新编译 | 开发阶段 |
| BT.CPP 插件加载 | 运行时可扩展 | 需要导出 `BT_REGISTER_NODES`，并管理 .so 搜索路径/ABI | 产品部署 |
| Lambda | 极简，无需定义类 | 不支持 Port/Blackboard | 原型验证 |

### XML 格式（BT.CPP v4）

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <DetectObject target_name="red_cube"
                    detected_pose="{object_pose}"/>
      <PlanGrasp target_pose="{object_pose}"
                 grasp_plan="{plan}"/>
      <ExecuteGrasp plan_id="{plan}"/>
    </Sequence>
  </BehaviorTree>

  <!-- 可以定义多棵树 -->
  <BehaviorTree ID="RecoveryTree">
    <Fallback>
      <ReturnToHome/>
      <EmergencyStop/>
    </Fallback>
  </BehaviorTree>
</root>
```

### SubTree 复用

SubTree 是 BT 最重要的复用机制：

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="PickObject">
    <Sequence>
      <DetectObject detected_pose="{pose}"/>
      <PlanGrasp target_pose="{pose}" grasp_plan="{plan}"/>
      <ExecuteGrasp plan_id="{plan}"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="PickObject"/>
      <MoveToPlace/>
      <PlaceObject/>
      <!-- 复用：连续抓取第二个物体 -->
      <SubTree ID="PickObject"/>
      <MoveToPlace target="{second_location}"/>
      <PlaceObject/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 热加载

由于行为树的拓扑定义在 XML 中，你可以在不重新编译 C++ 代码的情况下修改行为树结构：

```cpp
// 运行时重新加载 XML
auto new_tree = factory.createTreeFromFile("updated_tree.xml");
// 旧树析构，新树开始执行
```

这种「热加载」能力在工业场景中非常有价值——系统集成工程师可以在现场调整行为树，而不需要软件工程师重新编译部署。

### SubTree 复用的四种工业模式 ⭐⭐⭐

SubTree 是 BT 最重要的代码复用机制。下面总结四种在工业项目中反复出现的 SubTree 复用模式，每种模式都有明确的适用场景和实现要点。

**模式 1：参数化 SubTree（最常用）**

同一个操作逻辑用于不同物体/位置，通过 Port 参数化：

```xml
<root BTCPP_format="4">
  <!-- 通用的"抓取某个物体"子树 -->
  <BehaviorTree ID="PickAny">
    <Sequence>
      <DetectObject target_name="{object_name}"
                    detected_pose="{obj_pose}"/>
      <PlanGrasp target_pose="{obj_pose}"
                 approach_dir="{approach_direction}"
                 grasp_plan="{plan}"/>
      <ExecuteGrasp plan_id="{plan}"/>
    </Sequence>
  </BehaviorTree>

  <!-- 主树多次调用，传入不同参数 -->
  <BehaviorTree ID="SortObjects">
    <Sequence>
      <SubTree ID="PickAny"
               object_name="red_cube"
               approach_direction="top"
               obj_pose="{red_pose}"
               plan="{red_plan}"/>
      <PlaceAtBin bin_id="red_bin"/>

      <SubTree ID="PickAny"
               object_name="blue_cylinder"
               approach_direction="side"
               obj_pose="{blue_pose}"
               plan="{blue_plan}"/>
      <PlaceAtBin bin_id="blue_bin"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**关键要点**：SubTree 的每个 Port 都需要在调用处绑定到父树的 Blackboard 键。不同调用使用不同键名避免冲突（`red_pose` vs `blue_pose`）。

**模式 2：带错误恢复的可复用操作模块**

将"操作 + 恢复"作为完整单元封装：

```xml
<BehaviorTree ID="RobustGrasp">
  <Fallback>
    <!-- 正常路径 -->
    <Sequence>
      <PlanGrasp target_pose="{pose}" grasp_plan="{plan}"/>
      <ExecuteGrasp plan_id="{plan}"/>
      <VerifyGrasp result="{grasp_ok}"/>
    </Sequence>

    <!-- 恢复路径 1：换抓取角度 -->
    <Sequence>
      <OpenGripper/>
      <MoveToSafe/>
      <PlanGrasp target_pose="{pose}"
                 angle_offset="90"
                 grasp_plan="{plan}"/>
      <ExecuteGrasp plan_id="{plan}"/>
    </Sequence>

    <!-- 恢复路径 2：换抓取策略 -->
    <Sequence>
      <OpenGripper/>
      <MoveToSafe/>
      <PlanGraspSideApproach target_pose="{pose}"
                             grasp_plan="{plan}"/>
      <ExecuteGrasp plan_id="{plan}"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
```

调用方只需 `<SubTree ID="RobustGrasp" pose="{target}"/>`，所有恢复逻辑被封装在子树内部，调用方完全不需要关心。

**模式 3：条件选择 SubTree（运行时多态）**

根据运行时条件选择不同的操作子树：

```xml
<BehaviorTree ID="AdaptiveGrasp">
  <Fallback>
    <!-- 小物体用精密抓取 -->
    <Sequence>
      <IsSmallObject object_name="{obj}"/>
      <SubTree ID="PrecisionGrasp" object="{obj}"/>
    </Sequence>

    <!-- 大物体用力控抓取 -->
    <Sequence>
      <IsLargeObject object_name="{obj}"/>
      <SubTree ID="ForceControlGrasp" object="{obj}"/>
    </Sequence>

    <!-- 默认：标准抓取 -->
    <SubTree ID="StandardGrasp" object="{obj}"/>
  </Fallback>
</BehaviorTree>
```

**模式 4：跨文件 SubTree 库**

大型项目将 SubTree 分布在多个 XML 文件中，通过 `factory.registerBehaviorTreeFromFile()` 统一注册：

```cpp
BT::BehaviorTreeFactory factory;

// 注册 C++ 节点
factory.registerNodeType<DetectObject>("DetectObject");
factory.registerNodeType<PlanGrasp>("PlanGrasp");
// ...

// 从多个文件加载 SubTree 定义
factory.registerBehaviorTreeFromFile(
    "subtrees/pick_subtree.xml");
factory.registerBehaviorTreeFromFile(
    "subtrees/place_subtree.xml");
factory.registerBehaviorTreeFromFile(
    "subtrees/recovery_subtree.xml");

// 主树引用这些 SubTree
auto tree = factory.createTreeFromFile("main_tree.xml");
```

这种方式实现了"SubTree 库"——每个子树独立维护、独立测试、独立版本控制。工业项目中通常有 10-30 个 SubTree 文件。

**四种模式的适用场景对比**：

| 模式 | 适用场景 | 复杂度 | 典型子树数量 |
|------|---------|--------|------------|
| 参数化 | 同操作不同参数 | 低 | 1-3 个 |
| 带恢复 | 需要封装恢复逻辑 | 中 | 3-5 个 |
| 条件选择 | 多策略动态切换 | 中高 | 5-10 个 |
| 跨文件库 | 大型产品系统 | 高 | 10-30 个 |

### Groot2 调试实战流程 ⭐⭐⭐

Groot2 不仅仅是可视化工具——它是 BT 调试的核心工作流。下面给出从「行为树不工作」到「找到并修复问题」的完整调试流程。

**调试四步法**：

**Step 1：录制执行日志**

```cpp
auto tree = factory.createTreeFromFile("my_tree.xml");

// 同时启用三种日志（不要省略任何一种）
BT::StdCoutLogger console_log(tree);           // 控制台实时输出
BT::SqliteLogger sqlite_log(tree, "debug.db3"); // 可回放日志
BT::Groot2Publisher groot_pub(tree);            // 实时监控连接

// 执行
while (tree.tickOnce() == BT::NodeStatus::RUNNING) {
    std::this_thread::sleep_for(100ms);
}
```

**Step 2：Groot2 实时监控——定位失败区域**

连接到 `localhost:1666`，观察节点颜色变化：
- 绿色闪烁 = 该节点正在被 tick 且返回 SUCCESS
- 红色闪烁 = 该节点返回 FAILURE
- 蓝色持续 = 该节点处于 RUNNING（长时间蓝色说明卡住了）
- 灰色 = 该节点未被 tick（被控制节点跳过）

**常见模式识别**：
```
模式 A：Sequence 第一个子节点红色
  → 条件不满足，检查 Blackboard 输入数据

模式 B：Action 节点持续蓝色超过预期时间
  → Action Server 无响应或超时，检查 ROS2 Action

模式 C：Fallback 所有子节点依次红色
  → 所有恢复策略都失败，需要新增恢复方案

模式 D：ReactiveSequence 中的条件突然变红
  → 外部条件变化导致任务中断，检查传感器数据
```

**Step 3：SqliteLogger 回放——逐 tick 分析**

```bash
# 在 Groot2 中 File → Load Log → 选择 debug.db3
# 使用滑块或箭头按钮逐 tick 前进/后退
# 重点关注：
#   - FAILURE 首次出现的 tick 编号
#   - FAILURE 之前一个 tick 的状态（是从 RUNNING 变 FAILURE？还是从 IDLE 直接 FAILURE？）
#   - 同一 tick 中其他节点的状态（有无关联失败？）
```

**Step 4：Blackboard 数据追踪**

```cpp
// 在可疑节点中添加 Blackboard 打印
BT::NodeStatus tick() override {
    auto pose = getInput<geometry_msgs::msg::Pose>("target_pose");
    if (!pose) {
        RCLCPP_ERROR(logger_,
            "Blackboard key 'target_pose' is empty! "
            "Upstream node may not have set it.");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(logger_,
        "target_pose: [%.3f, %.3f, %.3f]",
        pose->position.x, pose->position.y, pose->position.z);

    // ... 正常逻辑 ...
}
```

**Groot2 与 Chrome Tracing 结合**：

对于性能调试，`MinitraceLogger` 输出 Chrome trace 格式，在浏览器中打开 `chrome://tracing` 可以看到每个节点的 tick 时间：

```cpp
BT::MinitraceLogger trace_log(tree, "bt_perf.json");
// 执行后在 Chrome 中加载 bt_perf.json
// 可以看到：
//   - 每个 tick 的总耗时
//   - 每个节点在 tick 中的占比
//   - tick 之间的间隔是否均匀
```

> **本质洞察**：BT 调试的困难不在于代码 bug，而在于**组合爆炸**——50 个节点的树有数百种可能的执行路径，每条路径取决于运行时条件。Groot2 的日志回放本质上是把不可重复的运行时行为变成了可重复、可检查的静态数据。这和 SLAM 中用 rosbag 回放替代实时跑车是同一个思想——用录制回放将非确定性问题转化为确定性分析。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：XML 中的节点名与 C++ 注册名不一致
   错误做法：C++ 中 registerNodeType<MyNode>("my_node")，
            XML 中写 <MyNode/>
   现象：createTreeFromFile 抛出异常 "node not found"
   根本原因：注册名是 "my_node"（小写下划线），XML 中用 "MyNode"（驼峰）
   正确做法：统一命名规范。推荐 PascalCase（大驼峰），因为 XML 标签
            习惯用 PascalCase
```

```
💡 概念误区：认为 XML 热加载可以改变节点实现
   新手想法："修改 XML 后行为树会用新的节点逻辑"
   实际上：XML 只定义拓扑结构（哪些节点、什么顺序、什么参数）。
          节点的 C++ 实现在编译时就确定了。
          热加载可以改变节点的组合方式和参数，不能改变节点的内部逻辑。
```

### 练习

1. **[A 型]** 用 Groot2 编辑一个 BT XML，运行时修改节点顺序（不重新编译）——验证热加载。

2. **[A 型]** 将「检测 + 规划 + 抓取」封装为 SubTree，在主树中复用两次。

3. **[B 型]** 精读 `BehaviorTreeFactory` 的 `registerNodeType()`、`registerFromPlugin()`、`createTreeFromFile()` 三个方法。分析静态注册、动态插件、XML 驱动如何协作。

---

## M13.5 与 ROS2 Action Server 的集成 ⭐⭐

### 动机

BT Action 节点通常需要调用 ROS2 Action Server（如 MoveIt2 的 `MoveGroup`、Nav2 的 `NavigateToPose`）。BT 的 RUNNING 状态与 ROS2 Action 的 Feedback 阶段天然对应。

### 集成模式

```
BT Action Node                    ROS2 Action Server
   │                                   │
   │ onStart():                        │
   │   创建 ActionClient               │
   │   发送 Goal ─────────────────────►│
   │   return RUNNING                  │
   │                                   │
   │ onRunning():                      │
   │   检查 result_future_             │
   │   ├── result ready?               │
   │   │   ├── succeeded → SUCCESS     │
   │   │   └── aborted → FAILURE       │
   │   └── not ready → RUNNING         │
   │                                   │
   │ onHalted():                       │
   │   cancel_goal_async() ───────────►│ cancel
   │                                   │
```

### BehaviorTree.ROS2 版本敏感骨架

BT.CPP 的 ROS2 集成通常来自 `behaviortree_ros2`（BehaviorTree.ROS2）包；`behaviortree_cpp` 是核心库。不要把示例工程里的接口包名误写成 Action 节点基类所在包。不同发行版打包的 BehaviorTree.ROS2 版本会影响回调签名，下面代码只展示稳定的数据流骨架：用目标系统里的 `behaviortree_ros2/bt_action_node.hpp` 和 `providedPorts()` 作为最终准绳。

**实现骨架**（MoveIt2 MoveGroup Action）：

```cpp
#include "behaviortree_ros2/bt_action_node.hpp"
#include "moveit_msgs/action/move_group.hpp"

class MoveToTarget
  : public BT::RosActionNode<moveit_msgs::action::MoveGroup>
{
public:
  using ActionT = moveit_msgs::action::MoveGroup;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = ActionT::Goal;
  using Feedback = ActionT::Feedback;
  using WrappedResult =
    rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

  MoveToTarget(const std::string& name,
               const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : Base(name, config, params)
  {}

  static BT::PortsList providedPorts() {
    // providedBasicPorts() 会补齐 action server 名称/超时等 ROS2 基础 port；
    // 如果你的 BehaviorTree.ROS2 版本没有这个 helper，就按该版本文档显式列出。
    return Base::providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "target_pose", "目标位姿"),
      BT::InputPort<std::string>(
        "planning_group", "manipulator", "规划组名"),
    });
  }

  // 构造 Goal
  bool setGoal(Goal& goal) override {
    auto target = getInput<geometry_msgs::msg::PoseStamped>(
      "target_pose");
    if (!target) return false;

    auto group = getInput<std::string>("planning_group");
    goal.request.group_name = group.value();
    goal.request.goal_constraints.push_back(
      make_pose_constraint(target.value()));
    return true;
  }

  // 处理 Feedback（可选）。新版本通常要求返回 NodeStatus；
  // 若旧版本头文件仍是 void，以本机头文件为准。
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const Feedback> fb)
    override
  {
    // 可以更新进度到 Blackboard
    return BT::NodeStatus::RUNNING;
  }

  // 处理 Result
  BT::NodeStatus onResultReceived(
    const WrappedResult& result) override
  {
    if (result.code ==
        rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  // 处理 Server 不可用
  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(),
      "MoveToTarget failed: %d",
      static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }
};

// 注册时把 ROS2 node 和默认 action 名称传给 BehaviorTree.ROS2。
BT::RosNodeParams params;
params.nh = node;
params.default_port_value = "/move_action";
factory.registerNodeType<MoveToTarget>("MoveToTarget", params);
```

**对比手动实现和使用 RosActionNode 基类**：

| 维度 | 手动实现 | RosActionNode 基类 |
|------|---------|-------------------|
| 代码量 | ~100 行（ActionClient 管理 + 状态检查） | ~30 行（只需实现 setGoal/onResult） |
| Goal 取消 | 需要手动在 onHalted 中 cancel | 基类自动处理 |
| 超时处理 | 需要手动计时 | 基类提供 server_timeout 参数 |
| Server 发现 | 需要手动 wait_for_action_server | 基类自动处理 |

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：忘记在 onHalted 中取消 Action Goal
   错误做法：自己写 StatefulActionNode 但 onHalted() 中什么都不做
   现象：BT 认为任务已中断，但 Action Server 仍在执行，
        机器人继续运动到旧目标——可能发生碰撞
   正确做法：在 onHalted() 中调用 cancel_goal_async()
   推荐：直接用 RosActionNode 基类，它自动处理取消逻辑
```

```
🧠 思维陷阱：认为 BT tick 频率应该与 Action 频率匹配
   新手想法："MoveGroup 规划需要 2 秒，BT 应该每 2 秒 tick 一次"
   实际上：BT tick 频率决定了系统的反应速度。
          tick 频率与 Action 执行时间无关。10 Hz tick 意味着
          每 100ms 检查一次「Action 完成了吗？安全条件还满足吗？」
          Action 可能需要几秒才完成，但 BT 每 100ms 就检查一次状态。
```

### 练习

1. **[A 型]** 实现一个 BT Action 节点调用 MoveIt2 MoveGroup Action。将目标位姿通过 Blackboard 传入。

2. **[思考题]** BT tick 10 Hz 而 Action Feedback 100 Hz——大部分 Feedback 被错过。这有什么影响？如何在节点内缓存最新 Feedback？

3. **[思考题]** BT.CPP 的 RUNNING 状态和 ROS2 Action 的 feedback 是什么关系？一个 BT Action 节点内部通常会创建一个 ROS2 ActionClient，RUNNING 对应 action 的 feedback 阶段。如果 action 超时，BT 节点如何处理？

---

## M13.6 错误恢复策略设计 ⭐⭐

### 动机

错误恢复是 BT 最大的价值所在。一个没有错误恢复的 BT 和一个简单的顺序脚本没有区别。本节系统性地讲解三种错误恢复模式。

### 三种错误恢复模式

**模式 1: Retry（重试）** —— 对偶发性失败

```xml
<RetryUntilSuccessful num_attempts="3">
  <PlanGrasp/>
</RetryUntilSuccessful>
```

适用于：规划超时、网络抖动、感知偶发失败。根本原因没变但随机因素可能导致成功。

**模式 2: Fallback（备选方案）** —— 对策略性失败

```xml
<Fallback>
  <PlanGraspApproachA/>    <!-- 方案 A: 从正面抓取 -->
  <PlanGraspApproachB/>    <!-- 方案 B: 从侧面抓取 -->
  <PlanGraspApproachC/>    <!-- 方案 C: 从顶部抓取 -->
</Fallback>
```

适用于：可能需要不同策略的场景。Fallback 会依次尝试，直到有一个成功。

**模式 3: Recovery SubTree（恢复子树）** —— 对需要多步恢复的失败

```xml
<Sequence>
  <Fallback>
    <ExecuteGrasp/>
    <!-- 抓取失败 → 执行恢复子树 -->
    <Sequence name="grasp_recovery">
      <OpenGripper/>           <!-- 先松开 -->
      <MoveToSafePosition/>    <!-- 退回安全位置 -->
      <DetectObject/>          <!-- 重新检测 -->
      <PlanGrasp/>             <!-- 重新规划 -->
      <ExecuteGrasp/>          <!-- 重新执行 -->
    </Sequence>
  </Fallback>
</Sequence>
```

适用于：需要先撤销当前操作、恢复到安全状态、再重新尝试的复杂场景。

### 模式组合：完整的 Pick-and-Place

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="RobustPickAndPlace">
    <Sequence name="main_sequence">
      <!-- Phase 1: 检测与规划 (带重试) -->
      <RetryUntilSuccessful num_attempts="3">
        <Sequence>
          <DetectObject target_name="{object_name}"
                        detected_pose="{object_pose}"/>
          <PlanGrasp target_pose="{object_pose}"
                     grasp_plan="{plan}"/>
        </Sequence>
      </RetryUntilSuccessful>

      <!-- Phase 2: 抓取 (带 fallback 恢复) -->
      <Fallback name="grasp_with_recovery">
        <ExecuteGrasp plan_id="{plan}"/>
        <Sequence name="grasp_recovery">
          <OpenGripper/>
          <MoveToSafePosition/>
          <RetryUntilSuccessful num_attempts="2">
            <Sequence>
              <DetectObject target_name="{object_name}"
                            detected_pose="{object_pose}"/>
              <PlanGrasp target_pose="{object_pose}"
                         grasp_plan="{plan}"/>
              <ExecuteGrasp plan_id="{plan}"/>
            </Sequence>
          </RetryUntilSuccessful>
        </Sequence>
      </Fallback>

      <!-- Phase 3: 搬运 (带安全条件) -->
      <ReactiveSequence name="safe_transport">
        <IsPathClear/>
        <MoveToPlace target="{place_location}"/>
      </ReactiveSequence>

      <!-- Phase 4: 放置并返回 -->
      <PlaceObject/>
      <OpenGripper/>
      <MoveToHome/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 穷举式错误分类

设计错误恢复之前，必须先系统性地分类所有可能的错误。按四个维度穷举：

| 维度 | 错误类型 | 恢复策略 |
|------|---------|---------|
| **感知** | 物体未检测到 / 位姿不准 / 遮挡 | Retry + 换视角检测 |
| **规划** | 规划超时 / 无解 / 碰撞 | Retry + 换起始位姿 |
| **执行** | 轨迹跟踪偏差大 / 电机过载 / 通信断开 | 安全停止 + 重连 |
| **交互** | 抓取滑落 / 物体变形 / 外力干扰 | 退回 + 重新检测 + 换策略 |

### ⚠️ 常见陷阱

```
🧠 思维陷阱：过度使用 RetryUntilSuccessful
   新手想法："失败了就重试，最终总会成功"
   实际上：如果失败的根本原因没有改变（如物体不在视野中），
          重试 100 次也不会成功。
   正确思维：Retry 适合偶发性失败。持续性失败应该用 Fallback 切换策略，
            或上升到更高层级处理。
   经验法则：RetryUntilSuccessful 的 num_attempts 不超过 3-5 次。
            如果超过 5 次仍然失败，说明需要换策略而非继续重试。
```

```
⚠️ 编程陷阱：Fallback 中的子节点有副作用但无回滚
   错误做法：Fallback 中的第一个方案失败后留下了中间状态（如夹爪闭合），
            第二个方案假设初始状态是干净的
   现象：第二个方案在不正确的初始状态上执行，行为异常
   正确做法：每个 Fallback 子节点在失败时要清理自己的副作用，
            或者在每个方案前加一个「状态重置」步骤
```

### 练习

1. **[A 型]** 为上述 pick-and-place BT 添加全局安全监控：用 ReactiveSequence 包裹整棵树，加入 `IsEmergencyStop` 条件。急停触发时所有动作中断。

2. **[A 型]** 用 Groot2 + SqliteLogger 记录一次执行。人为制造失败（删除物体使检测失败），用 Groot2 回放找到失败节点和恢复路径。

3. **[跨章综合题]** 结合 M12（ros2_control 控制器切换）和 M13（BT 编排），设计一个行为树：正常时用 JTC + MoveIt2 做规划执行，切换到「RL 模式」时触发 `switch_controllers` 切换到 ForwardCommandController 并启动 RL 策略。

---

## M13.7 Groot2 可视化与日志回放 ⭐⭐⭐

### 动机

调试行为树最大的挑战是「为什么第 37 秒那个节点失败了」。BT 的执行路径取决于运行时条件——你无法通过静态阅读 XML 来预测执行路径。

### 日志基础设施

BT.CPP 自带四种 Logger，每种记录每次 tick 中每个节点的状态转换：

```cpp
auto tree = factory.createTreeFromFile("my_tree.xml");

// 1. 控制台日志（开发调试）
BT::StdCoutLogger cout_logger(tree);

// 2. SQLite 日志（可回放）
BT::SqliteLogger sqlite_logger(tree, "bt_log.db3");

// 3. Minitrace 日志（Chrome trace 格式）
BT::MinitraceLogger minitrace_logger(tree, "bt_trace.json");

// 4. Groot2 实时监控（通过 ZMQ 通信）
BT::Groot2Publisher groot_publisher(tree);
```

**推荐的日志策略**：
- **开发阶段**：StdCoutLogger + SqliteLogger（前者实时观察，后者事后回放）
- **测试阶段**：SqliteLogger + Groot2Publisher（事后回放 + 实时监控）
- **产品部署**：仅 SqliteLogger（最低开销，保留回放能力）

### Groot2 的三种模式

1. **编辑模式**：拖拽节点、连接、设置 Port 参数。生成 XML 文件。
2. **监控模式**：连接运行中的 BT（通过 ZMQ），实时显示每个节点的状态（颜色编码：绿色=SUCCESS，红色=FAILURE，蓝色=RUNNING，灰色=IDLE）。
3. **回放模式**：加载 SqliteLogger 的 `.db3` 日志文件，逐 tick 回放执行历史。可以前进、后退、跳到指定 tick。

> **跨领域类比**：这种「从一开始就把可观测性内建到框架里」的思想类似于 Kubernetes 的 Prometheus metrics + Grafana dashboard——不是事后才加日志，而是框架本身就提供了结构化的观测能力。在 BT.CPP 中，这意味着你永远不需要手动在每个节点里加 `cout` 调试——Logger 自动记录所有状态转换。

### ⚠️ 常见陷阱

```
💡 概念误区：认为 Groot2 只是"画图工具"
   新手想法："我手写 XML 就够了，不需要 GUI"
   实际上：Groot2 的最大价值不是编辑 XML，而是日志回放。
          当 BT 有 50+ 节点和多层错误恢复时，
          通过日志回放逐 tick 观察状态变化，比读文本日志高效 10 倍。
```

### 练习

1. **[A 型]** 用 SqliteLogger 记录一次执行，Groot2 回放找到每个节点第一次返回 RUNNING 的 tick 编号。

2. **[思考题]** Groot2 通过 ZMQ 与运行中的 BT 通信。这种架构有什么延迟和带宽开销？在 100 Hz tick 频率、100 节点的树中，每次 tick 需要发送多少状态数据？

---

## M13.8 MTC 集成与工业案例 ⭐⭐⭐

### 动机

在实际系统中，BT.CPP 和 MoveIt Task Constructor (MTC) 经常配合使用，但它们在不同层次工作。

### BT.CPP vs MTC 的关系

| 维度 | BT.CPP | MTC |
|------|--------|-----|
| 层次 | **执行层**编排 | **规划层**编排 |
| 关注点 | 感知→规划→执行→错误恢复 | 多阶段运动规划（approach→grasp→lift→...） |
| 输出 | 触发各种 ROS2 Action/Service | 输出一系列运动轨迹 |
| 错误处理 | Retry/Fallback/Recovery | 尝试多个 planning pipeline |
| 时间尺度 | 秒到分钟 | 毫秒到秒 |

**典型组合**：BT 顶层编排 → 某个 BT Action 节点内部调用 MTC → MTC 输出轨迹 → 通过 ros2_control 执行。

```
BT.CPP (顶层)
├── DetectObject (感知)
├── [BT Action Node] ────────► MTC Task (规划层)
│                                ├── CurrentState
│                                ├── OpenGripper
│                                ├── MoveToPreGrasp
│                                ├── ApproachObject
│                                ├── CloseGripper
│                                ├── LiftObject
│                                └── MoveToPlace
├── VerifyGrasp (感知)
└── ReturnHome (执行)
```

**不是 X 而是 Y**：BT 和 MTC 不是替代关系，而是互补关系。BT 负责「什么时候做什么」（任务级决策），MTC 负责「怎么做」（运动级规划）。把 MTC 的工作放到 BT 中做是可以的但不推荐——因为 MTC 的 Stage 抽象专门为运动规划设计，比 BT 的通用节点更适合处理运动规划的约束传播和多解搜索。

### 工业案例：多步装配流程

**案例：PCB 连接器装配**

```
任务：将 3 个不同型号的连接器插入 PCB 板
约束：
  - 3 种连接器需要 2 种夹爪（2-pin 用小夹爪，4-pin 用大夹爪）
  - 每次插入后需要视觉检查
  - 插入力过大（>5N）需要退回重试
  - 整个流程不超过 60 秒
```

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="PCBAssembly">
    <Timeout msec="60000">
      <Sequence>
        <MoveToHome/>
        <SetBlackboard output_key="connectors"
          value="conn_2pin_A;conn_2pin_B;conn_4pin_C"/>

        <!-- 逐个装配 -->
        <ForEachConnector connector_list="{connectors}"
                          current="{current_conn}">
          <SubTree ID="AssembleSingleConnector"
                   connector="{current_conn}"/>
        </ForEachConnector>

        <FinalInspection/>
        <MoveToHome/>
      </Sequence>
    </Timeout>
  </BehaviorTree>

  <BehaviorTree ID="AssembleSingleConnector">
    <Sequence>
      <!-- 选择正确的夹爪 -->
      <SelectGripper connector_type="{connector}"
                     gripper_id="{gripper}"/>
      <Fallback>
        <IsCorrectGripperAttached gripper_id="{gripper}"/>
        <ChangeGripper target_gripper="{gripper}"/>
      </Fallback>

      <!-- 抓取 -->
      <RetryUntilSuccessful num_attempts="3">
        <SubTree ID="PickObject"
                 object_name="{connector}"/>
      </RetryUntilSuccessful>

      <!-- 插入（带力控监控） -->
      <Fallback name="insert_with_force_check">
        <ReactiveSequence>
          <IsInsertionForceOK max_force="5.0"/>
          <InsertConnector target_slot="{connector}"/>
        </ReactiveSequence>
        <Sequence>
          <RetractFromSlot distance="0.02"/>
          <AdjustAlignment/>
          <InsertConnector target_slot="{connector}"/>
        </Sequence>
      </Fallback>

      <!-- 视觉检查 -->
      <VerifyInsertion connector="{connector}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

这个案例体现了 BT 在工业场景中的关键优势：
- **工具切换**：Fallback + Condition 实现「如果当前夹爪不对就换」
- **力控安全**：ReactiveSequence + ForceCheck 实时监控插入力
- **模块化**：每个连接器的装配逻辑封装在 SubTree 中
- **超时保护**：全局 Timeout 确保不会无限执行

### 工业错误恢复的五级体系 ⭐⭐⭐

工业场景中的错误恢复不是简单的"重试"——需要按错误严重程度分级处理。以下五级体系来自工业机器人系统集成的最佳实践：

| 级别 | 名称 | 触发条件 | BT 实现 | 恢复时间 |
|------|------|---------|---------|---------|
| L1 | **本地重试** | 偶发性失败（规划超时、感知抖动） | `RetryUntilSuccessful(3)` | <5s |
| L2 | **策略切换** | 当前策略不可行（IK 无解、碰撞） | `Fallback(方案A, 方案B, 方案C)` | 5-15s |
| L3 | **状态重置** | 中间状态不正确（物体掉落、夹爪故障） | Recovery SubTree（松开→安全位→重检测→重试） | 15-30s |
| L4 | **任务降级** | 当前任务无法完成 | 跳过当前物体，记录失败，继续下一个 | 0s（跳过） |
| L5 | **人工介入** | 系统级故障（硬件故障、通信断开） | 通知操作员，等待人工确认后恢复 | 不确定 |

**L4 任务降级的 BT 实现**：

```cpp
// 自定义 Decorator：失败时降级为 SUCCESS 并记录
class DegradeOnFailure : public BT::DecoratorNode {
public:
    DegradeOnFailure(const std::string& name,
                     const BT::NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("task_id"),
            BT::OutputPort<std::string>("failed_tasks"),
        };
    }

    BT::NodeStatus tick() override {
        auto status = child_node_->executeTick();
        if (status == BT::NodeStatus::FAILURE) {
            auto task_id = getInput<std::string>("task_id");
            RCLCPP_WARN(logger_,
                "Task '%s' degraded: marking as skipped",
                task_id.value().c_str());

            // 将失败任务追加到失败列表
            auto prev = getInput<std::string>("failed_tasks")
                            .value_or("");
            setOutput("failed_tasks",
                      prev + ";" + task_id.value());

            return BT::NodeStatus::SUCCESS;  // 降级为成功
        }
        return status;
    }
};
```

```xml
<!-- 使用降级装饰器：某个物体抓取失败不影响整体流程 -->
<ForEachObject objects="{object_list}" current="{obj}">
  <DegradeOnFailure task_id="{obj}"
                     failed_tasks="{failures}">
    <SubTree ID="RobustGrasp" object="{obj}"/>
  </DegradeOnFailure>
</ForEachObject>

<!-- 最后报告失败的任务 -->
<ReportFailures failed_list="{failures}"/>
```

**L5 人工介入的 BT 实现**：

```cpp
class WaitForOperator : public BT::StatefulActionNode {
    BT::NodeStatus onStart() override {
        RCLCPP_ERROR(logger_,
            "SYSTEM FAULT: Waiting for operator intervention. "
            "Send 'resume' to /operator_cmd topic to continue.");
        // 发送告警到 HMI
        pub_->publish(make_alert("hardware_fault"));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (received_resume_) {
            received_resume_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        // 取消告警
        pub_->publish(make_alert("fault_cleared"));
    }
};
```

**完整五级恢复的 BT 骨架**：

```xml
<BehaviorTree ID="IndustrialPickPlace">
  <ReactiveSequence name="safety_monitor">
    <!-- 全局安全条件 -->
    <Fallback>
      <IsSystemHealthy/>
      <!-- L5: 人工介入 -->
      <WaitForOperator/>
    </Fallback>

    <Timeout msec="120000">
      <Sequence name="main_flow">
        <!-- L1+L2: 感知带重试和策略切换 -->
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DetectWithCamera camera="overhead"/>
            <DetectWithCamera camera="wrist"/>
          </Fallback>
        </RetryUntilSuccessful>

        <!-- L3+L4: 抓取带状态重置和降级 -->
        <DegradeOnFailure task_id="{current_obj}">
          <Fallback>
            <SubTree ID="GraspNormal"/>
            <SubTree ID="GraspRecovery"/>
          </Fallback>
        </DegradeOnFailure>
      </Sequence>
    </Timeout>
  </ReactiveSequence>
</BehaviorTree>
```

> **跨领域类比**：五级错误恢复体系类似于操作系统的异常处理层级——用户态异常先在本函数处理（L1-L2），处理不了向上抛到调用者（L3-L4），最终到达内核（L5 人工介入）。区别在于机器人系统的最后一道防线是人类操作员而非内核 panic。

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为 BT 可以替代 MTC
   新手想法："BT 能编排多步任务，MTC 也能，选一个就行"
   实际上：两者层次不同。BT 负责任务级决策，MTC 负责运动级规划。
          典型组合：BT Action 节点内部调用 MTC Task。
          把运动规划的细节（approach 方向、grasp 朝向）放在 BT 节点中
          是反模式——这些应该由 MTC 的 Stage 抽象来处理。
```

### 练习

1. **[A 型]** 为 PCB 装配案例添加「工具校准」子树：每次换夹爪后，移动到校准位，用视觉确认偏移量写入 Blackboard。

2. **[思考题]** 如果第 2 个连接器装配失败且无法恢复：(a) 跳过继续装第 3 个？(b) 整个任务 abort？(c) 通知操作员？不同选择的 BT 设计有什么区别？

3. **[思考题]** MoveIt Pro（PickNik 商业版）深度集成了 BT.CPP。与开源 MoveIt2 + BT.CPP 手动集成相比，MoveIt Pro 的 Objective 节点提供了什么额外价值？

---

## M13.9 前沿展望：BT.CPP v5、LLM 驱动的 BT 生成与形式化等价性 ⭐⭐⭐⭐

前八节建立了行为树从原理到工程的完整知识链。本节将目光投向行为树生态的下一步演进方向——BT.CPP v5 的架构升级、LLM 与行为树的融合、以及行为树与状态机的形式化等价关系。

### BT.CPP v5 预览

BT.CPP v5 是 Davide Faconti 规划中的下一个大版本。虽然截至 2026 年 5 月尚未正式发布稳定版，但已公开的设计意图和早期 RFC 讨论揭示了几个重要方向：

**预期特性**：

| 特性 | 说明 | 影响 |
|------|------|------|
| **原生协程支持** | 用 C++20 coroutines 替代当前的 StatefulActionNode 模型 | 异步节点的写法将更简洁，不再需要手动管理 onStart/onRunning/onHalted 三个回调 |
| **类型化 Blackboard** | 编译期检查 Port 类型匹配（当前是运行时 `std::any_cast`） | 把"Port 类型不匹配导致运行时崩溃"提前到编译期发现 |
| **多线程 tick** | 支持在不同线程池中 tick 树的不同分支 | Parallel 节点可以真正并行执行，而非当前的逻辑串行 |
| **改进的 XML 模式** | 更丰富的 XML 验证和代码生成工具 | 减少 XML 与 C++ 之间的不一致错误 |

> **反事实推理**：如果 BT.CPP v5 实现了原生协程支持会怎样？当前写一个异步 BT Action（如"移动到目标位姿"）需要实现三个回调函数（onStart/onRunning/onHalted），开发者必须手动管理内部状态（goal 发送了吗？结果收到了吗？被中断了吗？）。有了协程，同一个逻辑可以写成线性代码：`co_await send_goal(); co_await wait_result(); return result;`——`co_await` 自动处理挂起和恢复，BT 框架自动处理 halt 时的协程取消。代码量可能减少 40-60%，逻辑也更清晰。

### LLM 驱动的行为树生成（LLM-driven BT Generation）

2024-2025 年学术前沿的一个活跃方向是用大语言模型（LLM）自动生成行为树。核心思路是：给 LLM 一段自然语言任务描述（如"从桌子上拿起红色杯子放到架子上"），LLM 输出对应的 BT XML 文件。

**代表工作**：

| 工作 | 年份 | 方法 |
|------|------|------|
| SayCan (Ahn et al.) | 2022 | LLM 生成 skill 序列，不是 BT 结构 |
| ProgPrompt (Singh et al.) | 2023 | LLM 生成 Python 程序，包含 if/while 控制流 |
| LLM+BT (Lykov et al.) | 2024 | LLM 直接生成 BT XML，带 Retry/Fallback |
| RoboTree | 2025 | LLM 生成 BT + 自动验证安全约束 |

**设计动机**：手动编写 BT XML 需要机器人工程师同时理解任务语义和 BT 语法——这是一个瓶颈。如果非技术人员（工厂操作员）能用自然语言描述任务，LLM 自动生成 BT，将大幅降低机器人编程的门槛。

**关键挑战**：

1. **安全性**：LLM 生成的 BT 可能遗漏关键的安全检查（如"在闭合夹爪前检查手指间没有障碍物"）。当前解决方案是让 LLM 生成后经过形式化验证工具检查
2. **鲁棒性**：LLM 对 BT 语义的理解依赖训练数据——如果训练数据中 Fallback 和 Sequence 的使用场景不够丰富，生成的错误恢复逻辑可能不正确
3. **可解释性**：LLM 生成的 BT 比人工编写的更难理解——调试时工程师需要同时理解 LLM 的意图和 BT 的语义

> **本质洞察**：LLM 驱动的 BT 生成不是要"替代"人类工程师编写 BT——而是改变了编程的抽象层级。正如 BT 把任务编排从"写代码"提升到"画树"，LLM 把它进一步提升到"说话"。但每一层抽象都带来新的失败模式：代码级 bug → BT 结构 bug → 自然语言歧义 bug。工程师的角色从"编写 BT"变成了"验证和修正 LLM 生成的 BT"。

### BT 与 FSM 的形式化等价性

从计算理论的角度，BT 和 FSM 在表达能力上是**等价的**——任何 BT 都可以转换为等效的 FSM，反之亦然（Colledanchise & Ogren, 2018, Theorem 2）。但这种等价是理论层面的，类似于"任何高级语言程序都可以用图灵机实现"——正确但不实用。

**形式化等价的意义**：

1. **安全关键系统**：FSM 有成熟的模型检测工具（如 SPIN、NuSMV），可以形式化验证"系统是否会进入危险状态"。BT 的形式化验证工具较少——但由于 BT→FSM 的等价转换存在，理论上可以先将 BT 转换为 FSM，再用 FSM 工具验证
2. **CONVINCE 项目**（EU Horizon 2020 资助）正在开发 BT 专用的形式化验证工具链，目标是直接在 BT 上做性质验证（而非转换为 FSM）
3. **实践意义**：等价性告诉我们"选择 BT 还是 FSM 不是能力问题，而是工程效率问题"——对于复杂任务，BT 的模块化结构让维护成本从 O(N^2) 降到 O(N)

**BT→FSM 转换的复杂度爆炸**：

虽然理论上可转换，但实际操作中一棵深度为 D、分支因子为 B 的 BT 转换为 FSM 后，状态数可能达到 O(B^D)。例如一棵包含 5 层 Sequence/Fallback 嵌套、每层 3 个子节点的 BT，等效 FSM 可能有数百个状态——完全失去可读性。这正是 BT 的工程价值所在：它用树状结构紧凑地表达了 FSM 需要指数级状态数才能表达的逻辑。

---

## 本章小结

| 知识点 | 核心内容 | 难度 |
|--------|---------|------|
| M13.1 BT vs FSM | 本质差异、选型决策、可维护性对比 | ⭐⭐ |
| M13.2 异步 Ticking | 三态语义、四种节点类型、Reactive 模式 | ⭐⭐ |
| M13.3 Blackboard + Port | 类型安全数据流、Stamped API、SubTree 隔离 | ⭐⭐ |
| M13.4 XML DSL 与工厂 | 行为与实现分离、SubTree 复用、热加载 | ⭐⭐ |
| M13.5 ROS2 Action 集成 | RosActionNode 基类、RUNNING↔Feedback 映射 | ⭐⭐ |
| M13.6 错误恢复策略 | Retry/Fallback/Recovery SubTree 三种模式 | ⭐⭐ |
| M13.7 Groot2 可视化 | 编辑/监控/日志回放三种模式 | ⭐⭐⭐ |
| M13.8 MTC 集成与工业案例 | BT+MTC 层次关系、PCB 装配案例 | ⭐⭐⭐ |

## 累积项目：本章新增模块

Mini-Manip 项目新增 BT 编排层：

```
mini_manip_ws/
├── src/
│   ├── mini_manip_bt/              # 本章新增
│   │   ├── include/.../bt_nodes.hpp
│   │   ├── src/
│   │   │   ├── detect_object_node.cpp
│   │   │   ├── plan_grasp_node.cpp
│   │   │   ├── execute_grasp_node.cpp
│   │   │   └── move_to_node.cpp
│   │   ├── trees/
│   │   │   ├── pick_and_place.xml
│   │   │   └── recovery.xml
│   │   ├── plugin.xml
│   │   └── CMakeLists.txt
│   └── ...
```

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| BT.CPP 官方文档 (`behaviortree.dev`) | ⭐ | 入门教程和 API 参考 |
| Groot2 GUI | ⭐⭐ | 可视化编辑器和日志回放 |
| Davide Faconti 讲座 | ⭐⭐ | BT.CPP 作者的设计理念 |
| Nav2 BT 应用 (`navigation.ros.org/behavior_trees/`) | ⭐⭐ | BT 在导航中的大规模应用 |
| Colledanchise & Ogren (2018) "Behavior Trees in Robotics and AI" | ⭐⭐⭐ | 理论基础专著 |
| CONVINCE 项目 | ⭐⭐⭐⭐ | BT 形式化验证研究前沿 |
| Iovino et al. (2022) "A Survey of BT in Robotics and AI" | ⭐⭐⭐ | 综述论文 |

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| createTree 报 "node not found" | 注册名与 XML 不一致 | 1. 检查 registerNodeType 字符串 2. 对比 XML 节点名大小写 | M13.4 |
| getInput 返回空值 | Port 名拼写不一致 | 1. 检查 providedPorts 2. 检查上游节点 setOutput 3. 检查 XML `{}` 语法 | M13.3 |
| Action 节点一直 RUNNING | Action Server 未启动 | 1. 检查 Server 是否 alive 2. 加 Timeout decorator 3. 检查网络连接 | M13.5 |
| ReactiveSequence 不中断 | 条件节点始终 SUCCESS | 1. StdCoutLogger 打印状态 2. 检查条件逻辑 3. 检查 Blackboard 数据 | M13.2 |
| Groot2 无法连接 | ZMQ 端口占用 | 1. 检查端口 1666 2. 确认 Groot2Publisher 已创建 3. 检查防火墙 | M13.7 |
| SubTree 读不到父树数据 | Blackboard 隔离 | 1. 检查 SubTree 的 port remapping 2. 使用 `_autoremap` | M13.3 |
