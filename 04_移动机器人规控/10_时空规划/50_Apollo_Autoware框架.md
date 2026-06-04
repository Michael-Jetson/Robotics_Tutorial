# T4 Apollo 与 Autoware：工业规划栈如何组织时空规划

## 前置自测

开始前,先回答下面 5 个问题。
这一章是 T1-T3 的"工程落地篇"——把前三章的算法放进真实的工业框架里看。
答不出 2 题以上,建议先回 T1-T3 补一补。

1. T1 的 path-speed 解耦把规划拆成"先定路径、再排速度"两步。
   它为什么在工业界如此流行?(答不出 → 回 T1 §T1.1)
2. T1 的 SL 图和 ST 图分别表示什么?路径优化在哪个图上、速度优化在哪个图上?(答不出 → 回 T1)
3. T3 的 piecewise-jerk QP / OSQP 是用来解什么问题的?为什么工业界偏爱它?(答不出 → 回 T3、T1)
4. T3 的 OBCA + Hybrid A\* 组合用在什么场景?为什么泊车能用"真正时空联合"而高速行车不能?(答不出 → 回 T3 §T3.3)
5. 什么是有限状态机(FSM)?为什么用 FSM 管理"驾驶场景的切换"是自然的?(没接触过 FSM → 这一章会讲)

**参考答案要点**(先自己答,再对照):
1. 解耦把一个难的联合问题拆成两个凸子问题,各自能用成熟的 QP 高效求解;且 SL/ST 两图可分别可视化、分别调参,工程上可调试、可验证——这正是 T4 要展开的"工业为什么坚持解耦"。
2. SL 图(纵向位置 s × 横向偏移 l)表示**路径**的空间形状,路径优化在 SL 图;ST 图(纵向位置 s × 时间 t)表示**速度**沿路径的分布,速度优化在 ST 图。
3. piecewise-jerk QP 把路径/速度优化建模成"决策变量是位置/速度/加速度序列、以 jerk 平方为主要代价、约束线性"的二次规划,用 OSQP 求解;工业界偏爱它因为快(毫秒级)、凸(全局最优)、易调参。
4. OBCA + Hybrid A\* 用于泊车/开放空间:Hybrid A\* 出可行粗解 warm-start、OBCA 精确避障精修。
   泊车空间小、速度低、计算预算充裕,能负担"真正时空联合"的非凸优化;高速行车要求几十毫秒实时,3D 非凸联合算不动,只能解耦。
5. FSM 是"一组状态 + 状态间转移条件"的模型。
   驾驶天然分场景(车道保持、泊车、路口、靠边停车……),每个场景有自己的规划逻辑、场景间按条件切换——用 FSM 管理这种"离散场景切换"既自然又清晰。

---

## 本章目标

学完本章,你应该能够:

1. 理解 **Apollo 的 Scenario / Stage / Task 三层架构**:驾驶场景(Scenario)→ 顺序阶段(Stage)→ 原子操作(Task,即 Decider 和 Optimizer)如何组织成一个可扩展的规划系统。
2. 掌握 **Apollo EM Planner 的 SL+ST 双图迭代**:E-step 把障碍投影到 SL/ST 图、M-step 用 DP+QP 在图上优化,path 与 speed 如何交替迭代。
3. 理解 **Apollo 的 piecewise-jerk QP 在框架中的位置**:它如何作为一个 Task(Optimizer)嵌入 Task 链,以及 Task 之间的数据如何传递。
4. 理解 **Apollo Open Space 的 Hybrid A\* + OBCA**:为什么这是 Apollo 主链路中唯一"真正时空联合"的模块,以及它的工程理由。
5. 掌握 **Autoware Universe 的严格分层流水线**:Mission → Behavior Path → Behavior Velocity → Path Optimizer → Motion Velocity → Velocity Smoother,以及 pluginlib 如何实现模块热插拔。
6. 对比 **Apollo 与 Autoware 的时空规划策略**:半耦合 EM 迭代 vs 严格解耦 + 规则叠加,理解两种工业取舍。
7. 理解 **为什么工业栈仍坚持解耦**,以及 2025+ 端到端 + 安全过滤的新范式正在如何改变这个局面。

这一章和 T1-T3 不同:前三章讲**算法**(怎么把时间变成决策变量、怎么求解),这一章讲**工程**——把那些算法组织成一个能在真车上 10 Hz 稳定运行、能被几十个工程师协作维护、能安全验证的系统,需要怎样的架构。
读完你会明白:工业规划栈的难点,一半在算法,另一半在**架构**——如何把 T1-T3 的 QP、搜索、OBCA 拼成一个可调试、可扩展、可验证的整体。

---

## 知识导航

| 小节 | 主题 | 难度 | 一句话 |
|------|------|------|--------|
| §T4.1 | Apollo Planning 架构 | ⭐⭐ | Scenario/Stage/Task 三层 FSM,场景化组织规划 |
| §T4.2 | Apollo SL+ST 双图管线 | ⭐⭐⭐ | EM 迭代:E-step 投影、M-step DP+QP,path↔speed 交替 |
| §T4.3 | Apollo Open Space | ⭐⭐⭐ | Hybrid A\*+OBCA,主链路唯一的真正时空联合 |
| §T4.4 | Autoware Universe 流水线 | ⭐⭐⭐ | 严格分层 BPP→BVP→PathOpt→VelSmoother,pluginlib 热插拔 |
| §T4.5 | Apollo vs Autoware | ⭐⭐ | 半耦合 EM vs 严格解耦,两种工业取舍对比 |
| §T4.6 | 解耦的坚持与新范式 | ⭐⭐ | 为什么仍解耦,以及端到端+安全过滤 |

**两条阅读线**:想读懂 Apollo 源码的,重点看 §T4.1–§T4.3;想读懂 Autoware 源码的,重点看 §T4.4。
但 §T4.5(对比)、§T4.6(取舍与趋势)建议都读——它们回答的"为什么工业这样设计"的问题,比记住某个框架的目录结构更重要、也更不易过时。

---

## 前置知识桥接

**回顾 T1-T3(本章把它们落地)**:T1 给了 path-speed 解耦 + Frenet/ST 图这套工业基线;T2 讲了走廊与搜索(怎么定 homotopy、怎么把非凸避障变凸);T3 讲了连续优化(CILQR/TEB/OBCA/MINCO/CPC,怎么把时间变成决策变量)。
这一章要回答的是:**工业框架怎么用这些?** 答案会让你有点意外——主流量产栈(Apollo on-lane、Autoware)其实**主要用 T1 的解耦 + T1/T3 的 QP**,而把 T3 的"真正时空联合"(OBCA)只用在泊车这一个角落。
为什么?这正是本章要讲透的工程取舍。

**具体对应**:Apollo 的 `PiecewiseJerkPathOptimizer`(SL 路径 QP)和 `PiecewiseJerkSpeedOptimizer`(ST 速度 QP)就是 T1 解耦 + T1/T3 的 piecewise-jerk QP 的工业实现;Apollo Open Space 的 `OpenSpaceTrajectoryOptimizer` 就是 T3 §T3.3 OBCA 的落地;Autoware 的 Path Optimizer(MPT)是 T1 Frenet 横向 QP,Velocity Smoother 是 jerk-limited 速度 QP。
换句话说,**T4 = T1-T3 的算法 + 工业级的架构组织**。

**前向预告**:这一章会出现大量框架专有名词(Scenario/Stage/Task、BPP/BVP、pluginlib、CyberRT……)。
别被名词吓到——它们背后是同一套朴素的工程思想:**把复杂系统拆成可独立开发、可热插拔、可单独测试的模块**。
读的时候,重点抓"为什么这样拆"而非"叫什么名字"。
本章末尾的对比表和故障排查会把关键名词都串起来。

---

## 如果跳过本章会怎样

跳过 T4,你会在两个地方吃亏。
**场景一:读不懂量产代码,空有算法知识。** 你学完 T1-T3,理解了 Frenet、QP、OBCA,信心满满地打开 Apollo 的 `modules/planning` 想看看真实代码——结果迎面撞上 `ScenarioManager`、`Stage`、几十个 `Decider` 和 `Optimizer`、`ReferenceLineInfo` 在 Task 间传递的数据流,完全找不到"我学的那个 QP 在哪"。
算法知识是散点,工业框架是把这些散点组织起来的骨架;没有 T4 这层骨架,你的算法知识落不了地、读不懂真实工程。
**场景二:做架构决策时没有参照。** 你要为一个新的机器人/自动驾驶项目设计规划模块,面临一堆架构选择:用状态机管理场景还是别的?路径和速度解耦还是联合?模块怎么拆才能让团队协作、让功能可热插拔、让安全可验证?Apollo 和 Autoware 是两个经过大规模实战检验的工业答案——不研究它们,你只能从零摸索,重复别人踩过的坑。
T4 给你的正是这两套成熟架构的设计思想和取舍依据。
这两个场景的共同点:T1-T3 让你**懂算法**,T4 让你**懂系统**。
算法能在论文和 demo 里跑,系统才能在真车上跑、被团队维护、通过安全审查。
跳过 T4,你会停在"会写规划算法但读不懂、设计不出量产规划栈"的状态。

## 预计阅读时间

| 模式 | 时长 | 适合 |
|------|------|------|
| **精读** | 6-10 小时 | 第一次系统学工业规划架构:逐节读、对照 Apollo/Autoware 真实源码目录、搭一个仿真环境跑通。建议配合实际 clone 两个仓库浏览。 |
| **速读** | 2-3 小时 | 有自驾基础、想建立架构全景:读各节的架构图和"为什么这样设计"、§T4.5 对比表、§T4.6 取舍,跳过代码细节。 |
| **速查** | 20-40 分钟 | 已学过、回来查特定模块:定位到对应 §T4.x 看架构图 + 关键类职责;或查章末故障排查、API 速查、对比表。 |

本章不要求你能从零实现 Apollo/Autoware(那是几十万行代码的工程),目标是**读懂架构、理解取舍、能在其上做二次开发或借鉴其设计**。
所以即便速读,§T4.1(Apollo 三层架构)和 §T4.5(对比)也建议读——它们是理解工业规划栈的纲。

---

## 本章符号约定

本章是框架剖析,符号较少,主要是模块/类名(用等宽字体)和少量沿用前几章的数学记号。

| 记号 | 含义 | 首见 |
|------|------|------|
| `Scenario` / `Stage` / `Task` | Apollo 三层架构:场景 / 阶段 / 原子操作 | §T4.1 |
| `Decider` / `Optimizer` | Apollo Task 的两类:决策器 / 优化器 | §T4.1 |
| `ReferenceLineInfo` | Apollo 中承载一条参考线上所有规划数据的核心容器 | §T4.2 |
| SL 图 / ST 图 | 纵向 s × 横向 l(路径)/ 纵向 s × 时间 t(速度) | §T4.2(沿用 T1) |
| E-step / M-step | Apollo EM Planner:投影步 / 优化步 | §T4.2 |
| BPP / BVP | Autoware:Behavior Path Planner / Behavior Velocity Planner | §T4.4 |
| MPT | Autoware Path Optimizer 的 Model Predictive Trajectory | §T4.4 |
| pluginlib | ROS 的插件机制,运行时动态加载模块 | §T4.4 |

**怎么读这章**:§T4.1 先建立 Apollo 的整体骨架(三层架构),§T4.2 深入它的核心(SL+ST 迭代),§T4.3 看它的特例(Open Space 联合);§T4.4 转向 Autoware(另一套分层哲学);§T4.5、§T4.6 横向对比与展望。
建议按顺序读,因为后面的对比依赖前面建立的两个框架图景。

---

## §T4.1 Apollo Planning 模块架构 ⭐⭐

### 动机:一个规划器怎么应付那么多种驾驶情况?

T1-T3 教的算法,大多是在回答一个具体问题:给定一条参考线、一堆障碍,怎么算出一条好轨迹。
但真实驾驶不是单一问题。
车道内正常跟车、要变道超车、前方红灯要停、路口要让行、到了目的地要靠边停车、停车场里要泊入车位、走错了要掉头……每一种情况,**规划的逻辑、约束、甚至用的算法都不一样**。
车道内跟车用 SL+ST 解耦 QP 就够了;泊车却要用 Hybrid A\*+OBCA 的联合优化;靠边停车要先判断什么时候开始减速靠边、再规划停靠轨迹。
如果把所有这些逻辑塞进一个巨大的 `plan()` 函数,里面全是 `if (是泊车) {...} else if (是路口) {...} else if (要变道) {...}`,这个函数会迅速膨胀成几千行无人敢动的"巨石"——加一个新场景要改它、修一个 bug 要读懂全部、两个工程师同时改必然冲突。
于是工业框架的第一个核心问题就是:**怎么组织规划代码,让它能优雅地应对几十种驾驶场景,还能让一个大团队协作开发、持续加新场景?** Apollo 的答案是 Scenario / Stage / Task 三层架构。

### 如果不这样做会怎样(反面)

不做分层、用一个大 `plan()` 函数,会同时撞上几堵墙。
其一是**不可维护**:几千行的条件分支,改一处怕影响另一处,没人敢重构,技术债越滚越大。
其二是**不可协作**:所有人改同一个函数,合并冲突不断;新人要读懂整个函数才能动手。
其三是**不可复用**:泊车和靠边停车都需要"减速到停止"的逻辑,但塞在大函数里的代码无法复用,只能复制粘贴,改一处忘改另一处。
其四是**不可测试**:无法单独测试"路口让行逻辑"——它和别的逻辑纠缠在一个函数里,要测就得把整个规划器跑起来。
其五是**不可扩展**:加一个新场景(比如"环岛"),要在那个巨型函数里见缝插针,极易破坏已有逻辑。
设想这个巨型函数已经有三千行、嵌套了几十层 if-else,现在产品要支持"环岛通行"——你得在这堆纠缠的逻辑里找到合适的位置插入环岛处理,还要确保不破坏已有的车道保持、路口、泊车逻辑。
这几乎是不可能完成的任务:你既读不懂全部上下文,又无法保证改动的局部影响,每加一个场景,整个函数就更难维护一分,最终没人敢碰它——这就是所谓的"巨石"(monolith)技术债。
这些痛点的根源是一样的:**把"场景判断""阶段推进""具体计算"三件不同粒度的事,糊在了一个层次上**。
Apollo 的三层架构正是把这三件事分到三个层次——场景判断归 Scenario 层、阶段推进归 Stage 层、具体计算归 Task 层。

### 历史:从 DARPA 手写状态机到 Apollo 工业化

自动驾驶规划的架构演进,有一条清晰的脉络。
2005-2007 年的 DARPA 挑战赛时代(斯坦福 Stanley、CMU Boss),规划普遍是**手写的状态机** + 基础的 lattice planner——能跑通比赛,但代码组织粗糙、难以扩展到量产的复杂场景。
2017-2018 年,百度 Apollo 把规划工业化:Haoyang Fan 等人的《Baidu Apollo EM Motion Planner》(arXiv 1807.08048,2018)提出了基于 Frenet 帧的 path-speed 迭代框架,并配套了 Scenario/Stage/Task 的工程架构——自 Apollo v1.5(2017 年 9 月)起部署到数十辆车,到 2018 年 5 月已测试 3380 小时、约 6.8 万公里。
此后 Apollo 持续演进:5.0-8.0(2020-2023)把 piecewise-jerk QP 标准化、加入 Open Space 的 OBCA、引入 Plugin 机制;8.0 引入面向学习场景的包管理;9.0(2024)把包管理升级到 2.0、支持便捷的二次开发;10.0 面向大规模场景应用做全面性能优化;到 **11.0**(当前最新),面向配送、清扫、巡逻、园区接驳等高价值场景,支持端到端的运营闭环。
Apollo 以 `ApolloAuto/apollo`(约 26k★,Apache-2.0,C++)开源,planning 模块约 10 万行 C++,是研究工业级规划架构最好的公开样本之一。
这条脉络的关键是:**从"能跑"到"能量产、能协作、能持续演进",靠的不只是更好的算法,更是更好的架构**——这正是 Apollo 三层架构的价值。

### 理论:Scenario / Stage / Task 三层架构

**整体骨架。**
Apollo 的 Planning 模块(运行在 CyberRT 这个实时中间件上的一个节点)核心由几部分组成:

```text
PlanningComponent (CyberRT Node)            ← 规划模块的总入口
    │
    ├── ScenarioManager                     ← 场景切换 FSM(决定当前用哪个 Scenario)
    │     ├── LaneFollowScenario            ← 车道保持/跟车(最常用)
    │     ├── PullOverScenario              ← 靠边停车
    │     ├── ValetParkingScenario          ← 代客泊车
    │     ├── BareIntersectionScenario      ← 无信号灯路口
    │     └── TrafficLightScenario ...      ← 信号灯路口等
    │
    ├── ReferenceLineProvider               ← 参考线生成与平滑(DiscretePointsSmoother)
    │
    └── 每个 Scenario 内部:
          Stage (顺序阶段,一个场景分若干阶段)
            └── Task (原子操作,一个阶段执行一串 Task)
                  ├── Decider 类(做决策):
                  │     PathLaneBorrowDecider   ← 是否借道
                  │     PathBoundsDecider        ← 算 SL 上下界
                  │     STBoundsDecider           ← 算 ST 上下界(yield/overtake)
                  │     SpeedBoundsDecider        ← 算速度上下界
                  │     ...
                  └── Optimizer 类(做优化):
                        PiecewiseJerkPathOptimizer    ← SL 路径 QP
                        PathTimeHeuristicOptimizer    ← DP-ST 搜索
                        PiecewiseJerkSpeedOptimizer   ← ST 速度 QP
                        OpenSpaceTrajectoryOptimizer  ← OBCA(泊车)
```

**三层各管什么。**
**Scenario(场景)层**——回答"现在是什么驾驶情况"。
ScenarioManager 是一个有限状态机,根据车辆位置、地图信息、周围环境,判断当前应该处于哪个 Scenario(车道保持?路口?泊车?),并在条件满足时切换。
比如车开到停车场入口,ScenarioManager 从 LaneFollowScenario 切到 ValetParkingScenario。
每个 Scenario 封装了"这种情况下该怎么规划"的完整逻辑。
**Stage(阶段)层**——回答"这个场景分几步走"。
一个 Scenario 往往不是一步到位,而是分若干顺序阶段。
比如靠边停车(PullOver)可能分"接近(Approach)→ 调整(Adjust)→ 停靠(Park)"几个 Stage,按顺序推进。
Stage 之间也是一个小状态机:当前 Stage 完成,转入下一个 Stage。
**Task(任务)层**——回答"这一步具体算什么"。
这是最细的粒度,每个 Task 是一个**原子操作**,只做一件事。
Task 分两大类:**Decider**(决策器,做离散决策,如"是否借道""对这个障碍是 yield 还是 overtake")和 **Optimizer**(优化器,做连续优化,如路径 QP、速度 QP)。
一个 Stage 会按配置顺序执行一串 Task。

**这个分层为什么好?**
对照前面那五堵墙,三层架构逐一拆解了它们。
**可维护**:每层、每个 Task 职责单一、代码量小,改一个 Decider 不影响别的。
**可协作**:不同工程师负责不同 Scenario / Task,边界清晰、少冲突。
**可复用**:同一个 Task(如 `SpeedBoundsDecider`)可以被多个 Scenario / Stage 复用——配一下就行,不用复制代码。
**可测试**:每个 Task 是独立单元,可以单独喂输入、验输出地单元测试。
**可扩展**:加新场景 = 加一个新 Scenario 类 + 配置它的 Stage/Task 链,不用动别的代码;加新功能 = 写一个新 Task 注册进去。
这就是 Apollo Plugin 机制的基础——新 Task 能以插件形式接入。
一句话:**三层架构把"一个巨型函数"拆成了"一棵可组合、可插拔的树",每个节点小而专一,整体却能应对任意复杂的场景组合**。
这是工业规划栈区别于课程 demo 的根本一步。

> **多视角对照**:Scenario/Stage/Task 三层架构,从三个工程角度看是三件熟悉的事,合起来才看全它的价值。
> **从"操作系统的进程调度"看**:Scenario 像"当前运行的程序"、Stage 像"程序的执行阶段"、Task 像"原子的系统调用"——调度器(Stage)按序执行 Task、Task 之间通过共享内存(PlanningData)通信。
> 这个视角告诉你它**为什么能统一调度**:像 OS 调度进程一样,用统一接口(Task 基类)调度异构的操作。
> **从"流水线工厂"看**:Task 链像装配线上的工位,每个工位(Task)只做一道工序、把半成品(数据)往下传。
> 这个视角告诉你它**为什么数据要逐步积累**:像产品在装配线上逐步成型,ReferenceLineInfo 在 Task 间逐步填充完整。
> **从"插件化软件(如浏览器扩展)"看**:Task 像可安装/卸载的插件,通过注册接入、通过配置启用。
> 这个视角告诉你它**为什么可扩展**:像给浏览器装扩展一样,加新功能 = 写新 Task + 注册 + 配置,不动内核。
> 三个视角不矛盾:OS 调度视角解释"统一调度异构操作"、流水线视角解释"数据逐步积累"、插件化视角解释"可扩展"。
> 只盯着"三层"这个词会觉得抽象,换这三个工程类比看,它的设计意图就清楚了——这也是为什么有 OS、有流水线、有插件系统经验的工程师,理解 Apollo 架构会特别快。

**Planning 模块跑在 CyberRT 之上。**
既然前面把 Planning 称作"CyberRT 上的一个节点",就有必要交代 CyberRT 是什么——读 Apollo 代码绕不开它。
CyberRT 是 Apollo 自研的运行时中间件(替代早期版本用的 ROS),专为自动驾驶的高频、确定性、低延迟通信设计。
它和 ROS 的核心区别在两点:**共享内存通信**(同机上的模块间传数据走共享内存,避免序列化/拷贝开销,延迟极低)、**确定性调度**(任务调度可配置、可预测,满足自动驾驶对实时确定性的要求——不能让规划任务被随机延迟)。
对理解 Planning 架构,你只需知道:Planning 作为一个 CyberRT 组件(Component),通过 CyberRT 的通道(channel)订阅上游(感知、预测、定位、地图)的数据、发布规划好的轨迹给下游(控制)。
Scenario/Stage/Task 三层架构是 Planning **组件内部**的组织方式,而 CyberRT 是承载这个组件、并让它和其他模块通信的**外部运行时**。
这是一个常见的分层:CyberRT 管"模块间怎么通信和调度"(系统级),三层架构管"Planning 模块内部怎么组织"(模块级)——两者正交,各司其职。
Autoware 对应的是 ROS 2 + DDS 管系统级通信、六层流水线管模块级组织。
读任一框架,先分清"系统级中间件"和"模块级架构"这两层,就不会混乱。

### 代码实现(为什么 → 正确 → 错误 → 对比)

**Step 1 — 为什么这样写。**
真实 Apollo 的 Task/Stage/Scenario 用了工厂注册、配置驱动、CyberRT 通信等大量工程设施,代码量大。
下面给一份**可直接编译运行**的最简骨架,把三层架构的**组织方式和执行流**讲清:Task 是原子操作、Stage 顺序执行一串 Task、Scenario 含若干 Stage、Task 之间通过一个数据容器(模拟 Apollo 的 `ReferenceLineInfo`)逐步积累数据。
它用 C++ 的虚函数 + 智能指针实现多态的 Task,贴合 Apollo 的真实设计(Apollo 的 Task 也是继承自一个基类的多态对象)。
只用标准库,`g++ -std=c++17` 即编。

```cpp
// 最简演示:Apollo 风格的 Scenario/Stage/Task 三层架构如何组织与执行
// Task 是原子操作(Decider/Optimizer),Stage 顺序执行一串 Task,
// Scenario 含若干 Stage。Task 间通过数据容器逐步积累。纯标准库,g++ 即编。
#include <vector>
#include <string>
#include <memory>
#include <cstdio>

// 规划数据容器(模拟 Apollo 的 ReferenceLineInfo:Task 间传递、逐步积累)
struct PlanningData {
  bool path_bounds_ready = false;   // PathBoundsDecider 产出
  bool path_ready        = false;   // PiecewiseJerkPathOptimizer 产出
  bool st_bounds_ready   = false;   // STBoundsDecider 产出
  bool speed_ready       = false;   // PiecewiseJerkSpeedOptimizer 产出
  std::string log;
};

// Task:原子操作基类(Decider 和 Optimizer 都继承它)
class Task {
 public:
  explicit Task(std::string name) : name_(std::move(name)) {}
  virtual ~Task() = default;
  virtual bool Process(PlanningData& data) = 0;   // 每个 Task 只做一件事
  const std::string& name() const { return name_; }
 protected:
  std::string name_;
};

class PathBoundsDecider : public Task {           // Decider:做决策
 public:
  PathBoundsDecider() : Task("PathBoundsDecider") {}
  bool Process(PlanningData& d) override {
    d.path_bounds_ready = true;                   // 算出 SL 上下界
    d.log += "  [Decider] PathBoundsDecider: 算出 SL 边界\n";
    return true;
  }
};
class PiecewiseJerkPathOptimizer : public Task {  // Optimizer:做优化
 public:
  PiecewiseJerkPathOptimizer() : Task("PiecewiseJerkPathOptimizer") {}
  bool Process(PlanningData& d) override {
    if (!d.path_bounds_ready) {                   // 依赖前一个 Task 的产出!
      d.log += "  [Optim. ] PathOptimizer: 缺 SL 边界,失败\n";
      return false;
    }
    d.path_ready = true;                          // SL 路径 QP
    d.log += "  [Optim. ] PiecewiseJerkPathOptimizer: SL 路径 QP 完成\n";
    return true;
  }
};
class STBoundsDecider : public Task {
 public:
  STBoundsDecider() : Task("STBoundsDecider") {}
  bool Process(PlanningData& d) override {
    if (!d.path_ready) { d.log += "  STBounds: 缺路径,失败\n"; return false; }
    d.st_bounds_ready = true;                     // 算 ST 边界 + yield/overtake
    d.log += "  [Decider] STBoundsDecider: 算出 ST 边界(yield/overtake)\n";
    return true;
  }
};
class PiecewiseJerkSpeedOptimizer : public Task {
 public:
  PiecewiseJerkSpeedOptimizer() : Task("PiecewiseJerkSpeedOptimizer") {}
  bool Process(PlanningData& d) override {
    if (!d.st_bounds_ready) { d.log += "  SpeedOpt: 缺 ST 边界,失败\n"; return false; }
    d.speed_ready = true;                         // ST 速度 QP
    d.log += "  [Optim. ] PiecewiseJerkSpeedOptimizer: ST 速度 QP 完成\n";
    return true;
  }
};

// Stage:顺序执行一串 Task
class Stage {
 public:
  explicit Stage(std::string name) : name_(std::move(name)) {}
  void AddTask(std::unique_ptr<Task> t) { tasks_.push_back(std::move(t)); }
  bool Process(PlanningData& d) {
    d.log += "Stage[" + name_ + "] 执行 " + std::to_string(tasks_.size()) + " 个 Task:\n";
    for (auto& t : tasks_)
      if (!t->Process(d)) {                       // 任一 Task 失败则 Stage 中止
        d.log += "Stage[" + name_ + "] 在 " + t->name() + " 处中止\n";
        return false;
      }
    return true;
  }
 private:
  std::string name_;
  std::vector<std::unique_ptr<Task>> tasks_;
};

// Scenario:含若干 Stage
class Scenario {
 public:
  explicit Scenario(std::string name) : name_(std::move(name)) {}
  void AddStage(std::unique_ptr<Stage> s) { stages_.push_back(std::move(s)); }
  bool Process(PlanningData& d) {
    d.log += "Scenario[" + name_ + "] 激活\n";
    for (auto& s : stages_) if (!s->Process(d)) return false;
    return true;
  }
 private:
  std::string name_;
  std::vector<std::unique_ptr<Stage>> stages_;
};

int main() {
  // 组装 LaneFollow 场景的 Task 链(对应 §T4.2 的典型管线)
  auto stage = std::make_unique<Stage>("LaneFollowStage");
  stage->AddTask(std::make_unique<PathBoundsDecider>());          // 1. 算 SL 边界
  stage->AddTask(std::make_unique<PiecewiseJerkPathOptimizer>()); // 2. SL 路径 QP
  stage->AddTask(std::make_unique<STBoundsDecider>());            // 3. 算 ST 边界
  stage->AddTask(std::make_unique<PiecewiseJerkSpeedOptimizer>());// 4. ST 速度 QP

  Scenario lane_follow("LaneFollow");
  lane_follow.AddStage(std::move(stage));

  PlanningData data;
  bool ok = lane_follow.Process(data);            // ScenarioManager 选中后执行
  std::printf("%s", data.log.c_str());
  std::printf("规划结果: %s (path=%d speed=%d)\n",
              ok ? "成功" : "失败", data.path_ready, data.speed_ready);
  return 0;
}
// 运行输出:
//   Scenario[LaneFollow] 激活
//   Stage[LaneFollowStage] 执行 4 个 Task:
//     [Decider] PathBoundsDecider: 算出 SL 边界
//     [Optim. ] PiecewiseJerkPathOptimizer: SL 路径 QP 完成
//     [Decider] STBoundsDecider: 算出 ST 边界(yield/overtake)
//     [Optim. ] PiecewiseJerkSpeedOptimizer: ST 速度 QP 完成
//   规划结果: 成功 (path=1 speed=1)
```

**Step 2 — 正确要点。**
这份骨架抓住了 Apollo 架构的三个本质。
其一,**Task 是多态的原子操作**:`Task` 是基类,`PathBoundsDecider`、`PiecewiseJerkPathOptimizer` 等是子类,各自只 override 一个 `Process`——这正是 Apollo 的设计(所有 Task 继承自统一基类),让 Stage 能用一个 `vector<Task*>` 统一调度任意 Task。
其二,**数据在 Task 间逐步积累**:`PlanningData`(模拟 `ReferenceLineInfo`)被引用传递给每个 Task,前一个 Task 写入的结果(如 `path_bounds_ready`)是后一个 Task 的输入——这是理解 Apollo Task 链的关键:**数据不是各 Task 各算各的,而是在一个共享容器上接力式地填充**。
其三,**顺序与依赖**:Task 按配置顺序执行,后面的 Task 依赖前面的产出(`PiecewiseJerkPathOptimizer` 检查 `path_bounds_ready`)——所以 Task 链的**顺序不能乱**,这也是 Apollo 用配置文件固定 Task 顺序的原因。
真实 Apollo 还有工厂注册(按名字创建 Task)、配置驱动(Task 链写在配置文件里)、Stage 间的 FSM 转移等,但"多态 Task + 共享数据容器 + 顺序执行"这个骨架是一致的。

**Step 3 — 一个常见的错误写法。**
新手照搬这个架构时,最容易犯的错是**让 Task 之间直接互相调用、而非通过共享数据容器解耦**:

```cpp
// 反例:Task 内部直接 new 并调用下一个 Task,把执行顺序硬编码进 Task
class PathBoundsDeciderBad {
 public:
  void Process(PlanningData& d) {
    d.path_bounds_ready = true;
    PiecewiseJerkPathOptimizer next;      // 错:Task 内部直接创建并调用下一个 Task
    next.Process(d);                       // 把"谁是下一个"硬编码进了当前 Task
  }
};
// 问题:(1) 执行顺序被写死在 Task 内部 —— 想调整 Task 链顺序,得改 Task 代码;
//       (2) Task 之间强耦合 —— PathBoundsDecider 必须知道 PathOptimizer 的存在,
//           无法单独复用、单独测试 PathBoundsDecider;
//       (3) 加 Task / 改顺序 = 改一串 Task 的代码,失去了"配置驱动"的灵活性。
//       正确做法是 Task 只管自己那步,由 Stage 统一按配置顺序调度。
```

这个错误把"调度逻辑"混进了"业务逻辑",让 Task 不再是可自由编排的积木,退回到了硬编码的调用链。

**Step 4 — 对比。**
正确版(Apollo 架构)里,Task 只负责自己那一步、对"下一个是谁"一无所知,调度权归 Stage——于是 Task 链的顺序、增删都通过配置/编排完成,Task 本身可复用、可单测。
错误版把调度硬编码进 Task,Task 之间强耦合,失去了架构的全部好处(可维护/可协作/可复用/可测试/可扩展)。
这个对比是所有"管线式"架构的通用要点——**让每个处理单元只管自己、由一个调度器统一编排,而非让单元之间互相直接调用**。
Apollo 的 Stage 调度 Task、Autoware 的 pluginlib 加载模块,本质都是这个思想。

### 带数字走一遍

把架构骨架的运行摊开看,能把"三层架构怎么运转"从抽象变具体。
场景是组装一个 LaneFollow 的 Task 链(4 个 Task)并执行。
运行输出依次是:`Scenario[LaneFollow] 激活` → `Stage[LaneFollowStage] 执行 4 个 Task` → 四个 Task 依次打印(PathBoundsDecider 算 SL 边界 → PiecewiseJerkPathOptimizer 路径 QP → STBoundsDecider 算 ST 边界 → PiecewiseJerkSpeedOptimizer 速度 QP)→ `规划结果: 成功 (path=1 speed=1)`。
逐步读这个执行流的含义。
**为什么 Scenario→Stage→Task 是逐层下沉的?** 因为执行从最粗的粒度(选中 LaneFollow 这个场景)开始,下沉到这个场景的 Stage,再下沉到 Stage 里的一串 Task——这正是三层架构"情况→阶段→原子操作"的执行体现。
`Scenario::Process` 调 `Stage::Process`、后者循环调每个 `Task::Process`,层层下沉。
**为什么四个 Task 必须按这个顺序?** 因为数据在 `PlanningData` 上接力积累:`PathBoundsDecider` 先写入 `path_bounds_ready`,`PiecewiseJerkPathOptimizer` 才能读它并产出 `path_ready`,`STBoundsDecider` 才能基于路径算 ST 边界……每个 Task 依赖前面 Task 的产出。
把顺序打乱(如 STBounds 放最前),它会因 `path_ready` 为假而失败——这就是练习 2 要验证的。
**为什么最后 `path=1 speed=1` 才算成功?** 因为这表示路径和速度都成功产出(两个 bool 都被置真)。
任一 Task 失败(返回 false),Stage 立即中止、`ok` 为假——这是工业系统"任一环节失败就降级"的雏形。
**把某个 Task 的 `Process` 改成返回 false 再跑**,你会看到 Stage 在该 Task 处中止、后续 Task 不再执行、结果为失败——这演示了 Task 链的"短路"行为,真实 Apollo 也是任一 Task 失败则该 Stage 失败、触发降级。

### ⚠️ 常见陷阱

💡 **编程陷阱:把调度逻辑硬编码进 Task,让 Task 互相直接调用**
- **错误想法**:一个 Task 算完,自然该由它来调用下一个 Task,简单直接。
- **现象 / 后果**:Task 之间强耦合、执行顺序写死在代码里,想调整 Task 链顺序或增删 Task 都要改一串 Task 的源码,无法单独复用或测试某个 Task。
- **根本原因**:把"业务逻辑(这步算什么)"和"调度逻辑(下一步是谁)"混在了一个 Task 里,违背了三层架构"Task 只管自己、调度归 Stage"的分工。
- **正确做法**:Task 只负责自己那一步、对下游一无所知;由 Stage 按配置顺序统一调度。
  增删、重排 Task 通过编排/配置完成,不动 Task 代码。

⚠️ **概念陷阱:以为 Scenario/Stage/Task 三层是按"算法复杂度"分的**
- **错误想法**:Scenario 是大算法、Task 是小算法,三层是算法的粗细之分。
- **现象 / 后果**:理解不了为什么一个简单的 Decider(如"是否借道",可能就几行判断)和一个复杂的 Optimizer(QP 求解)处于**同一层**(都是 Task)。
- **根本原因**:三层不是按算法复杂度分,而是按**职责粒度**分——Scenario 管"什么情况",Stage 管"分几步",Task 管"每步做一件原子操作"。
  一个 Task 不论内部多简单或多复杂,它在架构里都是"一个原子操作"这个角色。
- **正确做法**:理解三层是"情况 → 阶段 → 原子操作"的职责递进,而非算法大小;Decider 和 Optimizer 同为 Task,只是"做决策"和"做优化"两类原子操作。

🧠 **思维陷阱:以为有了好架构就不需要好算法(或反之)**
- **错误想法**:Apollo 的厉害全在那套三层架构(或:全在 EM Planner 算法),抓住一个就够了。
- **现象 / 后果**:只学架构不学算法,做出空有骨架、规划质量差的系统;只学算法不学架构,写出跑得通 demo 但无法量产、团队没法维护的代码。
- **根本原因**:工业规划栈是"算法 + 架构"的乘积,缺一不可——好算法给出好轨迹,好架构让算法能被组织、维护、扩展、安全验证。
- **正确做法**:两手都要硬。
  T1-T3 给你算法,T4 给你架构;真正的工业能力是把好算法装进好架构。
  读 Apollo 时,既看 `PiecewiseJerkSpeedOptimizer` 的算法(T1/T3),也看它怎么作为 Task 嵌入 Task 链(T4)。

### 练习

1. **[实现]** 给上面的骨架加一个新场景 `PullOverScenario`(靠边停车):它含两个 Stage(`ApproachStage` 接近、`ParkStage` 停靠),每个 Stage 配不同的 Task 链。
   体会"加新场景 = 加新类 + 配置 Task 链,不动已有代码"的可扩展性。
2. **[调试]** 故意把骨架里 Task 链的顺序写错(把 `STBoundsDecider` 放到 `PiecewiseJerkPathOptimizer` 前面),运行观察会发生什么(`STBoundsDecider` 因 `path_ready` 为假而失败)。
   这说明 Task 链的什么性质?Apollo 用什么机制保证 Task 顺序正确?
3. **[设计]** 真实 Apollo 用工厂模式 + 配置文件来创建 Task(按名字字符串创建对应 Task 对象,Task 链写在配置里)。
   请设计一个简单的 `TaskFactory`:输入 Task 名字字符串,返回对应的 `unique_ptr<Task>`。
   为什么这种"按名字创建"对"配置驱动的 Task 链"是必需的?(提示:配置文件里只有字符串)

---

## §T4.2 Apollo SL+ST 双图管线:EM 迭代详解 ⭐⭐⭐

> 上一节搭好了 Apollo 的骨架(Scenario/Stage/Task 三层),但骨架里最核心的那个 LaneFollow 场景到底怎么规划?这一节钻进去看——它用的是 EM Planner,一套让 path 和 speed 交替迭代的"半耦合"方法。
> 这是 Apollo on-lane 规划的灵魂,也是理解"工业为什么不用纯解耦、也不用纯联合"的关键一步。

### 动机:解耦了路径和速度,可它们明明相互影响

T1 讲过 path-speed 解耦:先在 SL 图上优化路径,再在 ST 图上优化速度。
这个拆分的好处(两个凸子问题、各自快速可解)我们清楚,但它有个绕不开的问题:**路径和速度其实相互影响**。
举个例子:前方有辆车以中速行驶,你要超它。
"怎么超"(路径:从左边绕还是右边绕、绕多远)和"何时超"(速度:加速抢到它前面、还是减速跟在后面等机会)是耦合的——选了激进的路径就得配激进的速度,反之亦然。
纯解耦"先定死路径、再排速度",可能先选了一条贴着前车的路径,结果速度规划发现这条路径下根本排不出安全的速度(要么追尾要么急刹)。
反过来,如果路径规划时能"预见"到速度规划的难处,它本可以选一条更靠外侧、给速度留更多余地的路径。
这种"路径该参考速度、速度该参考路径"的相互依赖,正是纯解耦丢掉的东西,也正是 EM 迭代要找回的——让 path 和 speed 在迭代中互相"看见"对方,逼近那个二者协调的联合解。
那怎么办?完全联合优化(T3 的 OBCA)又太慢,满足不了高速行车的实时要求。
Apollo EM Planner 的答案是一个聪明的折中:**不完全联合,而是让 path 和 speed 交替迭代、互相参考对方的上一轮结果**——这就是它名字里 "EM"(借自 EM 算法的交替思想)的由来。

### 如果不这样做会怎样(反面)

如果坚持"纯单向解耦"(path 算一次、speed 算一次,互不参考),在强耦合场景会失效。
**单向解耦的失效**:path 优化时不知道 speed 会怎么排,可能选一条 speed 无法配合的路径;speed 拿到这条死路径,只能在它的约束下勉强排速度,得到次优甚至不可行的结果。
前面那个超车例子就是典型——路径定死了,速度没有回旋余地。
**完全联合的代价**:那把 path 和 speed 放进一个大优化问题一起解?这就回到 T3 的非凸联合优化,3D(s,l,t)空间的非凸问题在几十毫秒内根本算不动,满足不了 on-lane 高速行车 100 ms 级的实时预算。
EM 迭代正是在这两个极端之间:它**保留解耦的高效**(每轮仍是 path、speed 各自的凸子问题,快),又**通过迭代引入耦合**(path 参考上一轮的 speed、speed 参考这一轮的 path),逼近联合优化的质量。
代价是要迭代几轮(增加一点计算),但每轮都快,总体仍在实时预算内。
这是工业界"在质量和实时之间找平衡"的经典手法。

### 历史:EM Planner 的迭代思想

EM Planner 的核心思想来自 Fan 等人 2018 年的论文。
论文标题里的 "EM" 借自机器学习的 EM(期望最大化)算法——那是一种"固定一部分、优化另一部分,交替进行"的迭代框架。
EM Planner 把这个思想用到规划:把"路径"和"速度"看成两组要交替优化的变量,固定一个、优化另一个,反复迭代直到收敛。
论文把每轮迭代分成两步:**E-step(投影步)**——把障碍物投影到 SL 图和 ST 图上(用上一轮的结果来估计动态障碍的交互);**M-step(优化步)**——在投影好的图上,用动态规划(DP)+ 样条二次规划(spline QP)优化路径和速度。
论文强调 EM Planner 是 **light-decision-based(轻决策)** 的:它不预先做大量硬性决策,而是让 DP 步在优化中**顺带**确定障碍决策(nudge 绕行 / yield 让行 / overtake 超车),再让 QP 在 DP 给出的凸域内求精——这比"重决策"(预先决定一切再优化)更能应对复杂场景。

轻决策和重决策的对立,是规划设计里一个重要的分野,这里把它说透。
**重决策(heavy-decision)**:先用一套(往往是规则化的)逻辑,把所有离散决策都定死——对每个障碍明确"让行还是超车"、对每条车道明确"走还是不走",然后在这些定死的决策下做优化。
优点是决策清晰、易理解易解释;缺点是规则难以覆盖所有复杂场景,预先定死的决策一旦不当,后面的优化也救不回来,且场景一复杂,规则就爆炸(组合太多)。
**轻决策(light-decision)**:不预先定死,而是把决策**融入优化**——让 DP 搜索在探索解空间时,顺带确定哪个决策更优(DP 的每条边对应一个决策选择,搜索自然选出代价最低的决策组合),再让 QP 在选定决策的凸域内求精。
优点是能在优化中权衡决策(而非拍脑袋定死)、更能应对复杂场景的组合;缺点是决策隐含在优化里,不如重决策那样一目了然。
**EM Planner 选轻决策**,正是因为它要应对城市道路的复杂多变(几十种障碍交互组合,规则化的重决策覆盖不过来)。
它用 DP 的搜索来"软性"地探索和选择决策,比硬编码规则更有弹性。
**这也呼应 §T4.6 的可解释性话题**:轻决策虽然决策融入优化、不如重决策直白,但 Apollo 通过把决策结果显式输出到 ST 图(yield/overtake 可视化),仍保住了可解释性——这是"轻决策的弹性"和"决策可解释"的一个巧妙平衡。
理解轻/重决策的取舍,你会在很多规划系统里看到类似选择:是把决策前置定死(简单可控但僵硬),还是融入优化(灵活但需额外手段保证可解释)。

这套方法已随 Apollo 量产(v1.5 起部署,2018 年已测试数万公里),在 Apollo 5.0 后,DP+spline QP 中的 QP 部分逐步标准化为 **piecewise-jerk QP**(分段 jerk 二次规划,用 OSQP 求解),也就是我们 T1/T3 见过的那个形式。
所以读 Apollo 代码时,你会看到论文的 E/M-step 思想,落地为一串具体的 Task:投影和边界由 Decider 完成,DP 和 QP 由 Optimizer 完成。

### 理论:E-step 投影、M-step 优化,path↔speed 交替

**整体迭代结构。**
EM Planner 的一轮迭代,概念上是:

```text
                  ┌─────────────────────────────────────┐
                  │  用上一轮的 speed profile 作为参考     │
                  ▼                                       │
   E-step(投影): 把障碍投影到 SL 图                        │
   M-step(优化): DP path → 凸域 + nudge 决策               │
                 spline/piecewise-jerk QP path → 平滑路径   │
                  │                                       │
                  ▼                                       │
   E-step(投影): 沿新路径,把障碍投影到 ST 图               │
   M-step(优化): DP speed → 凸域 + yield/overtake 决策      │
                 piecewise-jerk QP speed → 平滑速度         │
                  │                                       │
                  └───────── 下一轮(若未收敛)─────────────┘
```

**E-step:投影(SL 和 ST mapping)。**
E-step 的任务是把三维的"障碍 + 自车"交互,**投影**到二维的 SL 图和 ST 图上,从而把难处理的高维问题降到两个二维图。
**SL 投影**:把障碍投影到 SL 图(纵向 s × 横向 l)。
论文指出,出于安全,SL 投影主要考虑**静态、低速、对向**障碍——这些障碍的横向占据相对确定,适合在表达"路径空间形状"的 SL 图上处理。
**ST 投影**:沿当前路径,把障碍(静态 + 动态)投影到 ST 图(纵向 s × 时间 t)。
动态障碍与自车的交互,关键在"什么时刻、自车走到路径上哪个位置时会和障碍相遇"——这正是 ST 图表达的。
投影需要估计自车沿路径的时间-位置关系,而这依赖速度——所以要用**上一轮的 speed profile** 来估计,这就是迭代耦合的来源。

**M-step:DP + QP 优化。**
M-step 在投影好的图上做两阶段优化(对 path 和 speed 都是如此)。
**DP 阶段**:在图上做动态规划搜索,得到一个粗糙但可行的解,并**顺带确定障碍决策**——DP path 阶段在 SL 图上决定对静态障碍 nudge(向左/向右绕)、DP speed 阶段在 ST 图上决定对动态障碍 yield(让行,从它后面过)还是 overtake(超车,从它前面过)。
DP 的产出是一个**凸的可行域(feasible tunnel)**,把非凸的避障问题(绕左还是绕右是不同 homotopy)收敛到一个确定的凸通道里。
**QP 阶段**:在 DP 给出的凸域内,用 piecewise-jerk QP(OSQP 求解)求出平滑的最优解。
因为 DP 已经把问题限制在凸域内,QP 是个凸问题、能快速求全局最优。
这个 "DP 定 homotopy + QP 求精" 的分工,正是 T2/T3 那条主线在 Apollo 里的体现——DP 是离散搜索(定绕行方向),QP 是连续优化(在选定方向内求精)。

**为什么这是"半耦合"。**
对照两个极端:纯解耦是 path、speed 各算一次互不参考;完全联合是 path、speed 一起优化。
EM Planner 处在中间——它**形式上仍是解耦**(每步只优化 path 或 speed 之一,是凸子问题),但**通过迭代引入了耦合**(speed 投影参考 path、下一轮 path 参考 speed)。
所以称它"半耦合"或"准时空联合":它没有真正在一个优化问题里联合 path 和 speed(那是 Open Space 的 OBCA 才做的),但通过交替迭代,获得了接近联合优化的协调性,同时保住了解耦的实时性。

> **多视角对照**:EM 的"半耦合",从三个角度看,各自照亮它的一面。
> **从"数值方法"看**:它是块坐标下降(block coordinate descent)——把变量分成 path 块和 speed 块,固定一块优化另一块,交替进行。
> 这个视角告诉你它**为什么收敛**:块坐标下降在每块凸时会收敛到协调解,这是成熟的数值优化框架。
> **从"博弈/协商"看**:path 和 speed 像两个协商者,轮流出价、参考对方最新立场调整自己,直到达成一致(不动点)。
> 这个视角告诉你它**为什么能协调**:迭代就是协商过程,不动点就是双方都满意的协调态。
> **从"工程折中"看**:它是"纯解耦"(快但不协调)和"完全联合"(协调但慢)之间的滑块,通过迭代轮数调节——迭代越多越接近联合(慢)、越少越接近解耦(快)。
> 这个视角告诉你它**为什么务实**:它不是非此即彼,而是在质量和实时的连续谱上取一个工程平衡点。
> 三个视角不矛盾:数值方法视角给收敛保证、博弈视角给协调直觉、工程视角给折中定位。
> 理解了这三层,你就明白 EM 既不是"假联合"也不是"次优解耦",而是一个有理论支撑(块坐标下降)、有协调能力(迭代协商)、又务实(可调轮数)的工业方案。

**三种策略并排看。**
把"纯解耦、半耦合(EM)、完全联合"三种处理 path-speed 的策略并排,EM 的定位一目了然:

| 维度 | 纯解耦 | 半耦合(EM) | 完全联合 |
|------|--------|-------------|----------|
| path/speed 关系 | 各算一次,互不参考 | 交替迭代,互相参考 | 同一优化问题里一起解 |
| 子问题 | 两个独立凸 QP | 每轮仍是凸 QP,迭代多轮 | 一个非凸 NLP |
| 协调能力 | 无(强耦合场景失效) | 中等(迭代逼近联合) | 强(真正联合最优) |
| 计算速度 | 最快 | 较快(几轮 QP) | 慢(非凸 NLP) |
| 实时性 | 最好 | 好(on-lane 够用) | 差(只够 Open Space) |
| Apollo 用在 | —(不单用) | on-lane 主链路 | Open Space 泊车 |

读这张表,EM 的位置很清楚:它在协调能力上比纯解耦强(能应对 on-lane 的中等耦合)、在速度上比完全联合快(满足 on-lane 实时),正好卡在中间这个"on-lane 够用"的甜点。
而纯解耦在强耦合场景会失效(协调能力无)、完全联合在 on-lane 算不动(实时性差)——两端都不适合 on-lane,只有半耦合合适。
这也再次印证了 Apollo 的因地制宜:on-lane 用半耦合(EM)、Open Space 用完全联合(OBCA)、从不单用纯解耦(协调能力太弱)。

**典型性能**:LaneFollow 场景的一条完整 Task 链(PathBoundsDecider → PiecewiseJerkPathOptimizer → STBoundsDecider → SpeedBoundsDecider → PathTimeHeuristicOptimizer → PiecewiseJerkSpeedOptimizer)总延迟约 30-50 ms,落在 10 Hz 规划频率的预算内。
路径 QP 约 5 ms、速度 QP 约 10 ms,DP 搜索和各 Decider 占其余。

**LaneFollow 的完整 Task 链:逐个看职责。**
理解了 EM 的迭代结构,再把 Apollo LaneFollow 场景的真实 Task 链摊开,你就能把"论文里的 E/M-step"和"代码里的一串 Task"对上号。
典型的 Task 链(按执行顺序)大致是:
**1. PathLaneBorrowDecider**——决定是否借道(本车道被堵时,是否借用相邻车道绕行)。
这是个 Decider,输出一个布尔决策影响后续路径边界。
**2. PathBoundsDecider**——计算 SL 图的上下界(路径的横向可行范围),其中包含对静态障碍的 nudge(向左/向右绕)决策。
这是 path 优化的"凸域准备"。
**3. PiecewiseJerkPathOptimizer**——在 PathBoundsDecider 给的 SL 边界内,做路径的 piecewise-jerk QP(OSQP,约 5 ms),产出平滑路径。
这是 M-step 的 path QP。
**4. PathAssessmentDecider**——评估产出路径的质量(是否碰撞、是否过于激进),给路径打分。
**5. PathDecider**——从候选路径里选最优的(多车道时有多条候选)。
**6. RuleBasedStopDecider**——基于规则的停车决策(如前方有不可逾越的障碍,该停)。
**7. STBoundsDecider**——构建 ST 图的上下界,包含对动态障碍的 yield(让行)/overtake(超车)决策。
这是 speed 优化的"凸域准备",也是 ST 投影的体现。
**8. SpeedBoundsDecider**——计算速度的上下界(含道路限速、弯道的向心加速度折减等)。
**9. PathTimeHeuristicOptimizer**——在 ST 图上做 DP 搜索(DP-ST),产出粗速度 + 确定动态障碍决策,给 QP 提供 warm-start 和凸域。
这是 M-step speed 的 DP。
**10. PiecewiseJerkSpeedOptimizer**——在 DP 给的凸域内做速度的 piecewise-jerk QP(OSQP,约 10 ms),产出平滑速度。
这是 M-step 的 speed QP。
读这条链,几件事变清晰了:**Decider 和 Optimizer 交替出现**(先 Decider 准备边界/决策,再 Optimizer 在边界内优化);**path 相关的 Task(1-6)在前、speed 相关的(7-10)在后**,体现 EM 先 path 后 speed 的顺序;每个 DP(9)都在对应 QP(10)之前,体现"DP 定凸域 + QP 求精"。
这就是论文的 E/M-step 思想落地成可执行 Task 链的样子——读 Apollo 源码时,你会在配置文件里看到这一串 Task 的名字,在代码里看到它们各自继承 Task 基类。

### 代码实现(为什么 → 正确 → 错误 → 对比)

**Step 1 — 为什么这样写。**
真实 EM Planner 的 path/speed 优化各自是完整的 DP+QP(几百行),耦合通过 ST 投影里的速度估计实现,难以在几十行里完整重现。
但 EM 的**核心机制——交替迭代逼近联合最优**——可以用一个玩具耦合模型清楚演示。
下面这份**可直接编译运行**的代码,用两个相互影响的简单函数模拟 path 和 speed 的耦合(path 的激进程度影响最优 speed、反之亦然),做交替迭代,看它如何收敛到"联合最优"。
只用标准库,`g++ -std=c++17` 即编。

```cpp
// 最简演示:EM Planner 的 path<->speed 交替迭代如何收敛
// 玩具耦合模型:给定 speed=v,最优 path 是 p*=0.5v+1(速度越快路径越激进)
//              给定 path=p,最优 speed 是 v*=0.4p+0.5(路径越激进速度配合)
// 真正联合最优是两式交点。EM 迭代:固定 v 解 p → 固定 p 解 v → 重复。纯标准库。
#include <cmath>
#include <cstdio>

double f_path (double v) { return 0.5 * v + 1.0; }   // M-step path:用上一轮 speed
double f_speed(double p) { return 0.4 * p + 0.5; }   // M-step speed:用这一轮 path

int main() {
  // 解析解(联合最优):p=0.5v+1 与 v=0.4p+0.5 联立 => p=1.5625, v=1.125
  double p_star = 1.25 / 0.8, v_star = 0.4 * p_star + 0.5;
  double v = 0.0, p = 0.0;                    // 初始

  std::printf("EM 迭代(path<->speed 交替),联合最优 p*=%.4f v*=%.4f\n", p_star, v_star);
  std::printf("轮次 | path p   | speed v  | 距最优\n");
  for (int iter = 1; iter <= 8; ++iter) {
    p = f_path(v);                            // M-step path:固定 speed 优化 path
    v = f_speed(p);                           // M-step speed:固定 path 优化 speed
    double err = std::hypot(p - p_star, v - v_star);
    std::printf("  %d  | %.5f | %.5f | %.2e\n", iter, p, v, err);
    if (err < 1e-6) { std::printf("第 %d 轮收敛!\n", iter); break; }
  }
  return 0;
}
// 运行输出:
//   联合最优 p*=1.5625 v*=1.1250
//   轮1: p=1.00000 v=0.90000 距最优 6.06e-01
//   轮3: p=1.54000 v=1.11600 距最优 2.42e-02
//   轮8: p=1.56249 v=1.12500 距最优 7.75e-06  → 收敛
```

**Step 2 — 正确要点。**
这份演示抓住了 EM 迭代的本质。
其一,**每步是简单子问题**:`f_path` 和 `f_speed` 各自是简单(这里是线性,真实是凸 QP)的优化——这对应 EM 每步只优化 path 或 speed 之一、是个能快速求解的凸子问题。
其二,**耦合通过"参考对方上一轮"实现**:`f_path(v)` 用当前的 `v`、`f_speed(p)` 用刚算的 `p`——这正是 EM 的耦合机制(ST 投影用上一轮 speed 估计、path 用之)。
其三,**交替迭代收敛到联合最优**:误差几何级数下降,几轮就逼近两式交点(联合最优)——这说明 EM 用"解耦的子问题 + 迭代"逼近了"联合优化"的解,这就是"半耦合"的价值。
真实 EM Planner 的收敛性比线性玩具复杂(非线性、有决策切换),通常迭代固定轮数(而非等收敛)以保证实时性,但"交替逼近联合最优"的本质一致。

**Step 3 — 一个常见的错误写法。**
新手实现交替迭代时,最容易犯的错是 **path 和 speed 都用"上一轮"的对方值(Jacobi 式),而非 speed 用"这一轮刚算的"path(Gauss-Seidel 式)**:

```cpp
// 反例:path 和 speed 都用上一轮的旧值(Jacobi 迭代),收敛慢甚至震荡
double p_new = f_path(v_old);     // path 用旧 v
double v_new = f_speed(p_old);    // 错:speed 也用旧 p,而非刚算出的 p_new!
p_old = p_new; v_old = v_new;     // 一起更新
// 问题:speed 没用上 path 刚算出的最新结果,信息利用不充分,
//       收敛更慢;在强耦合时甚至可能在两个值之间来回震荡不收敛。
//       正确做法(Gauss-Seidel)是 speed 立即用 path 刚算的新值,信息传递更快。
```

这个错误对应数值迭代里 Jacobi(都用旧值)和 Gauss-Seidel(立即用新值)的区别——EM Planner 用的是后者(speed 立即用刚优化的 path),收敛更快更稳。

**Step 4 — 对比。**
正确版(Gauss-Seidel,EM 用的)里,`f_speed(p)` 立即用 `f_path` 刚算出的 `p`,最新信息马上传递,收敛快。
错误版(Jacobi)里,speed 用的是上一轮的旧 path,信息滞后一轮,收敛慢、易震荡。
这个对比是所有交替优化的通用要点——**交替迭代时,后优化的变量应立即使用先优化变量的最新结果,而非都用上一轮的旧值**。
EM Planner 让 speed 立即用刚算的 path,正是这个道理。

### 带数字走一遍

把 EM 迭代 demo 的输出摊开,能看清"交替迭代逼近联合最优"到底是怎么发生的。
玩具模型:path 的激进程度 p 和 speed 的激进程度 v 相互影响(p\*=0.5v+1、v\*=0.4p+0.5),联合最优是两式交点 (1.5625, 1.125)。
从 (0,0) 出发交替迭代。
输出每轮的 (p, v) 和距最优的误差:轮 1 (1.0, 0.9) 误差 0.61 → 轮 2 (1.45, 1.08) 误差 0.12 → 轮 3 (1.54, 1.116) 误差 0.024 → … → 轮 8 (1.5625, 1.125) 误差 7.75e-6,收敛。
逐个数字读它的含义。
**为什么误差每轮大约缩小到 1/5?** 因为这个玩具的迭代映射是压缩的——每轮迭代把 (p,v) 往最优拉近一个固定比例(这里约 0.2,即两个耦合系数 0.5×0.4 的量级)。
误差几何级数下降(0.61→0.12→0.024→…),这是压缩映射的典型行为。
真实 EM 的收敛更复杂(非线性、有决策切换),但"每轮拉近、逐步收敛"的本质一致。
**为什么从 (0,0) 这么差的初值也能收敛?** 因为压缩映射不依赖初值——不管从哪开始,都会收敛到唯一不动点(联合最优)。
这解释了 EM 为什么对初值不敏感、鲁棒。
**为什么收敛到的正好是两式交点?** 因为交点是"同时满足 p=0.5v+1 和 v=0.4p+0.5"的点,即迭代不再变化的不动点——而不动点正是 path 和 speed 都不想再调整的"协调态",对应联合最优。
**把耦合系数调大(如 p\*=0.9v+1、v\*=0.9p+0.5,乘积 0.81 接近 1)再跑**,你会看到收敛明显变慢(每轮只缩小到 0.81);若乘积 >1(如都用 1.2),迭代发散——这说明 EM 收敛与耦合的"压缩性"直接相关,这正是练习 1 要探究的。
真实 EM 用固定少数轮数(而非等收敛),正是因为知道几轮内已足够接近。

### ⚠️ 常见陷阱

💡 **编程陷阱:交替迭代时双方都用上一轮旧值(Jacobi 而非 Gauss-Seidel)**
- **错误想法**:一轮迭代里,path 和 speed 都基于"上一轮"的对方值来算,逻辑上更"对称"。
- **现象 / 后果**:收敛明显变慢,强耦合时甚至在两个值之间来回震荡、不收敛。
- **根本原因**:后优化的 speed 没用上 path 刚算出的最新结果,信息利用滞后一轮(Jacobi 迭代的通病)。
- **正确做法**:用 Gauss-Seidel 式——speed 立即用这一轮刚优化好的 path。
  EM Planner 正是先算 path、再用新 path 算 speed。

⚠️ **概念陷阱:以为 EM Planner 是"真正的时空联合优化"**
- **错误想法**:EM 迭代让 path 和 speed 协调了,那它就是把两者联合优化了。
- **现象 / 后果**:把 EM Planner 和 OBCA 那种真正的联合优化混为一谈,理解不了为什么 Apollo 还要单独搞 Open Space。
- **根本原因**:EM 是**半耦合**——每步仍是 path 或 speed 的**独立**凸子问题,只是通过迭代互相参考;它没有在一个优化问题里同时含 path 和 speed 的决策变量。
  真正的联合优化(OBCA)才是把 (x,y) 和 (v,a) 放进同一个 NLP。
- **正确做法**:明确 EM 是"解耦 + 迭代耦合"的折中,用接近实时的代价换接近联合的质量;真正的时空联合(非凸 NLP)只用在 Open Space(§T4.3)。

🧠 **思维陷阱:以为迭代轮数越多越好**
- **错误想法**:EM 迭代既然收敛,那多迭代几轮总能得到更优的解。
- **现象 / 后果**:盲目增加迭代轮数,规划延迟超出 10 Hz 预算,反而拖垮实时性。
- **根本原因**:EM 是实时系统的一环,有严格的时间预算(整条 Task 链 30-50 ms);多迭代一轮带来的质量提升边际递减,但时间成本是实打实的。
- **正确做法**:在实时预算内固定一个够用的迭代轮数(工业实践常是很少几轮),而非追求完全收敛。
  工程上"够好且实时"胜过"最优但超时"。

### 练习

1. **[实现]** 把玩具模型改成更强的耦合(如 `f_path(v)=0.9v+1`、`f_speed(p)=0.9p+0.5`),观察收敛是否变慢;再试 `f_path(v)=1.2v+1`(耦合系数 >1),看是否发散。
   这说明 EM 迭代的收敛与什么有关?(提示:迭代映射的"压缩性")
2. **[对比]** 分别用 Gauss-Seidel(speed 用新 path)和 Jacobi(都用旧值)实现同一个玩具迭代,对比两者的收敛速度(达到同样误差所需轮数)。
   验证 Gauss-Seidel 更快。
3. **[跨章]** EM 的 M-step 是 "DP 定凸域 + QP 求精"。
   回忆 T2:这正是"搜索定 homotopy + 连续优化求精"。
   请说明 Apollo 的 DP path/speed 对应 T2 的什么、QP 对应 T3 的什么,并解释为什么 DP 一定要在 QP 之前(提示:QP 需要一个凸的可行域)。

---

## §T4.3 Apollo Open Space:主链路唯一的真正时空联合 ⭐⭐⭐

> 前两节讲的都是 Apollo 的 on-lane 主链路——解耦 + EM 半耦合。
> 但有一类场景这套方法完全失效:泊车。
> 这一节看 Apollo 怎么处理它——用 T3 学过的 Hybrid A\*+OBCA,做主链路里唯一一处"真正的时空联合优化"。
> 看完你会理解 Apollo 最务实的一面:不是全用一种方法,而是按场景在解耦和联合之间切换。

### 动机:泊车为什么不能用 SL+ST 解耦?

§T4.1、§T4.2 讲的 on-lane 主链路(车道内行驶),用 SL+ST 解耦 + EM 迭代,跑得又快又稳。
但有一类场景,这套方法彻底失效:**泊车、掉头、停车场内行驶**——Apollo 把它们统称 Open Space(开放空间)。
为什么失效?因为 SL+ST 解耦的前提是"有一条参考线(车道中心线),车基本沿它走,只在横向小幅偏移"。
泊车没有这个前提:车要倒进一个垂直车位,得先前进、再打满方向盘倒车、可能还要揉一把方向——轨迹是大幅度的、前进后退混合的、没有"沿参考线"这回事。
想象一个最常见的侧方位停车:车先开过车位、停下、打方向往后倒、车尾摆进车位、再回正方向往前调整——这个过程里,车的位置在前后移动(s 不单调)、朝向在大幅旋转(从平行车道到斜着倒入再回正)、速度在前进倒车间切换、转向角在打满和回正间变化,这些量全部紧密耦合、缺一不可。
你没法说"先定一条路径、再排速度"——倒车的每一刻,位置、朝向、速度、转向角是作为一个整体被规划的。
这就是泊车必须联合、不能解耦的直观图景。
在这种场景里,"路径"和"速度"的耦合极强(倒车入库时,每一点的位置、朝向、速度、转向角都紧密关联),无法解耦;而且车要在窄小的空间里精确避开两侧车辆和墙,需要**硬约束**保证不刮蹭——这正是 T3 §T3.3 OBCA 解决的问题。
于是 Apollo 在 Open Space 用了完全不同的方法:**Hybrid A\* 搜索 + OBCA 优化的真正时空联合**——这是 Apollo 主链路里唯一一处不用解耦、而用 T3 那种联合优化的地方。

### 如果不这样做会怎样(反面)

如果硬把 on-lane 的解耦方法套到泊车上,会全面崩坏。
**没有参考线,SL 图无从建立**:SL 图的横轴是"沿参考线的纵向位置 s",泊车没有一条可以沿着走的参考线,s 这个坐标失去意义,SL 图根本建不起来。
**解耦表达不了前进后退**:SL+ST 假设车单调地沿参考线前进(s 递增),而泊车要前进再倒车(s 非单调),解耦框架无法表达。
**软约束不够安全**:泊车是厘米级的精度要求(差几厘米就刮蹭),on-lane 那种软约束(留点裕度、允许微小越界)不可接受,必须硬约束保证严格无碰撞。
反过来,为什么 on-lane 不能用 Open Space 的联合优化?因为联合优化是非凸 NLP,慢——泊车能容忍(空间小、速度低、计算预算充裕,几百毫秒甚至秒级可接受),但 on-lane 高速行车要求几十毫秒级实时,非凸联合算不动。
所以 Apollo 的选择是**因地制宜**:on-lane 用解耦(快、够用),Open Space 用联合(慢、但泊车能容忍且必须)。
这种"主链路解耦 + 特例联合"的设计,正是工业框架务实的体现。

### 历史:OBCA 进入 Apollo

Open Space 的核心算法,正是 T3 §T3.3 讲的 OBCA。
OBCA(Optimization-Based Collision Avoidance)由 Xiaojing Zhang、Alexander Liniger、Francesco Borrelli(UC Berkeley MPC Lab + ETH)提出(arXiv 1711.03449,后发表于 IEEE Transactions on Control Systems Technology, 2020, 29(3):972-983)——用凸优化的强对偶,把非凸、不可微的避障约束,精确重构成光滑可微的约束,使标准 NLP 求解器能解。
其面向泊车的版本 **H-OBCA**(Hierarchical OBCA,Zhang-Liniger-Sakai-Borrelli,CDC 2018)确立了"Hybrid A\* 出粗解 warm-start → OBCA 精修"的两段式管线。
Apollo 把这套方法工程化进了 Open Space 模块:`OpenSpaceTrajectoryOptimizer` 这个 Task,内部用 Hybrid A\* 做搜索、用 OBCA 思路(在 Apollo 里叫 `DistanceApproachProblem`,基于 CasADi + Ipopt)做优化。
Apollo 还做了一个变体 **TDR-OBCA**(Trust-region & Dual-relaxation OBCA),用信赖域和对偶松弛改善 OBCA 的求解稳定性和速度——这是 Apollo 在 Berkeley 原始 OBCA 基础上的工程改进。

**为什么需要 TDR-OBCA 这样的改进?**
理解这个改进的动机,能让你体会"论文算法"到"量产模块"的工程距离。
原始 OBCA 是一个非凸 NLP,用 Ipopt 这类内点法求解。
它在论文的实验场景下工作良好,但放到量产泊车的千变万化场景里,会暴露两个问题。
**收敛稳定性**:非凸 NLP 的求解对初值、对问题条件敏感,某些刁钻的泊车场景(极窄车位、复杂障碍布局)下,Ipopt 可能收敛慢、或陷入数值困难、甚至不收敛。
量产系统不能接受"偶尔算不出来"——它需要在**所有**场景下都能在限定时间内给出可用结果。
**信赖域(Trust-region)**的作用:它限制每步优化的步长在一个"可信的邻域"内,避免优化跳到远处的坏区域,从而提升收敛的稳定性和可预测性——这是处理非凸问题的经典稳健化手段。
**对偶松弛(Dual-relaxation)**的作用:OBCA 的对偶变量(那些表达避障的 λ)在某些情况下会让问题变得病态;适当松弛对偶约束,能改善数值条件、加快求解。
**这个例子的普遍意义**:它说明工业落地一篇算法论文,核心工作往往不是"实现论文的主算法"(那相对直接),而是"让它在所有边界场景下都稳定、快速、可靠"——这需要信赖域、松弛、warm-start、降级等一系列工程加固。
TDR-OBCA 之于 OBCA,正如许多工业实现之于其学术原型:主体思想来自论文,但能上车的关键在那层工程加固。
读 Apollo 的 Open Space 代码时,你看到的大量"论文里没有"的细节,正是这层加固。

所以 Open Space 模块是一个很好的例子,展示工业框架如何把一篇学术论文(OBCA)落地为可部署的模块:不是照搬,而是配上 warm-start 前端、做求解稳定性改进、嵌入到 Task 架构里。

### 理论:Hybrid A\* + OBCA 的两段式联合

**整体流程。**
Open Space 的轨迹生成是一个两段式管线:

```text
   起点、目标车位、障碍(其他车、墙)
              │
              ▼
   Hybrid A* 搜索  (状态空间 (x, y, θ),考虑车辆运动学/Reeds-Shepp 曲线)
              │  产出:一条粗糙但运动学可行、无碰撞的轨迹(含前进/倒车切换)
              ▼  (作为 warm-start)
   TDR-OBCA 优化  (同时优化 x, y, θ, v, a, δ + 对偶变量 λ)
              │  OBCA 把避障约束精确光滑化,Ipopt 求解非凸 NLP
              ▼  产出:平滑、动力学可行、严格无碰撞的精确轨迹
   DistanceApproachProblem (CasADi + Ipopt 的具体实现)
              │
              ▼
   Open Space 轨迹输出
```

**第一段:Hybrid A\* 搜索。**
Hybrid A\* 是一种在连续状态空间(x, y, θ)上做的 A\* 变体:它不像普通 A\* 那样在离散网格上走,而是用车辆运动学模型(或 Reeds-Shepp 曲线,允许前进+倒车)生成可行的状态转移,在连续空间里搜索。
它的产出是一条**运动学可行、无碰撞、但粗糙**的轨迹——包含了"先前进到某点、再打方向倒车"这样的前进后退切换。
关键作用:它**定下了 homotopy**(怎么倒、往哪个方向揉方向),并给出一个可行的初值。
这正是 T2/T3 反复出现的"离散搜索定 homotopy"那一步——泊车的"绕行方式"是离散的(从车位左侧切入还是右侧、倒一把还是两把),Hybrid A\* 负责选定它。

**第二段:OBCA 优化。**
拿到 Hybrid A\* 的粗解 warm-start,OBCA 做精修。
它同时优化轨迹的所有量:位置 (x, y)、朝向 θ、速度 v、加速度 a、转向角 δ——这是**真正的时空联合**(不像 on-lane 那样 path/speed 分开)。
避障约束用 OBCA 的对偶重构(T3 §T3.3):每个障碍引入对偶变量 λ,把"车不在障碍内"精确表达成光滑约束,保证严格无碰撞。
求解用 Ipopt(内点法 NLP 求解器)。
因为有 Hybrid A\* 的 warm-start 定好了 homotopy,这个非凸 NLP 能稳定收敛到该 homotopy 下的高质量解。

**为什么这是主链路唯一的"真正联合"。**
回顾 Apollo 的全貌:on-lane 主链路(LaneFollow 等)用 SL+ST 解耦 + EM 半耦合迭代,**没有**把 path 和 speed 放进一个优化问题;只有 Open Space 才把位置、朝向、速度、转向角**全部**放进同一个 OBCA NLP 里联合优化。
这是 Fan 等人 2018 论文里那个工程判断的体现:on-lane 的 3D(s,l,t)非凸联合无法满足 100 ms 实时,所以解耦;Open Space 空间小、速度低、预算充裕,可以联合。

**为什么两个场景的计算预算差这么多?**
这个"预算差异"是 Apollo 因地制宜的根本依据,值得算一笔账。
**on-lane 的预算极紧**:高速行车时,车速可能 30 m/s(108 km/h),0.1 秒就走 3 米。
规划必须高频(如 10 Hz,每 100 ms 一帧)才能及时响应路况变化;每帧的计算预算就是这 100 ms,还要分给感知后处理、预测、规划、控制等多个模块,留给规划的可能只有几十毫秒。
在这点时间里,3D 非凸联合优化(变量多、非凸、要迭代到收敛)根本算不完——所以必须用解耦(每个凸子问题几毫秒)+ 少数轮 EM 迭代。
**Open Space 的预算宽松**:泊车时车速通常 <2 m/s(慢速挪动),环境基本静止(停好的车、固定的墙)。
这意味着:规划不需要那么高频(慢速下,几百毫秒甚至一两秒规划一次也来得及响应),且因为环境静态,一条规划好的泊车轨迹可以用比较久、不用频繁重算。
于是计算预算从 on-lane 的"几十毫秒"放宽到"几百毫秒到秒级"——这就足够跑一个非凸的 OBCA NLP(Hybrid A\* 搜索 + Ipopt 优化)了。
**一句话**:预算差异源于**速度和环境动态性的差异**——高速 + 动态环境逼出"高频 + 紧预算"(只能解耦),低速 + 静态环境允许"低频 + 松预算"(可以联合)。
这笔账算清,你就彻底理解了 Apollo 为什么敢在泊车用联合、却不敢在 on-lane 用——不是技术不行,是预算不允许。
这也提示你做任何规划系统设计时,先算清"每帧有多少计算预算",再据此选方法,而非反过来。

所以理解 Apollo 的关键是:**它不是"全用解耦"或"全用联合",而是按场景的实时性预算和耦合强度,在两者间务实选择**——on-lane 选解耦(快),Open Space 选联合(精)。
这种"看菜下饭"的工程智慧,比任何单一方法都更值得学。

> **本质洞察**:Apollo "on-lane 解耦、Open Space 联合"的设计,本质是"**让方法的复杂度匹配问题的约束,而非一刀切**"。
> 新手常犯的错是找到一个"最好的方法"就到处用——但工业系统里,不存在放之四海皆准的最优方法,只存在"在这个场景的实时预算、耦合强度、精度要求下最合适的方法"。
> on-lane 的约束是"高速、几十毫秒实时、耦合中等",解耦+迭代最合适;Open Space 的约束是"低速、预算充裕、耦合极强、精度要求高",联合优化最合适。
> 同一个系统里并存两种截然不同的方法,不是设计不统一,而是**对"问题的约束决定方法的选择"这一原则的忠实贯彻**。
> 理解了这一点,你看任何成熟工业系统都会发现类似的"分而治之"——不是它们不够优雅,而是真实世界的问题本就异质,强行用一种方法统一,反而会在某些场景上付出惨痛代价(on-lane 硬上联合会超时、Open Space 硬上解耦会失效)。
> 这是从"追求最优方法"到"追求最优匹配"的认知跃迁,也是工程成熟度的标志。

### 代码实现(为什么 → 正确 → 错误 → 对比)

**Step 1 — 为什么这样写。**
完整的 Open Space 是 Hybrid A\* + OBCA NLP(后者依赖 Ipopt/CasADi),难以在几十行里完整重现。
但 Open Space 第一段 **Hybrid A\* 的核心——在连续状态空间按车辆运动学扩展节点**——可以独立演示,而且它正是 Hybrid A\* 区别于普通 A\* 的精髓。
下面这份**可直接编译运行**的代码,用自行车模型在 (x, y, θ) 连续空间搜索,展示"运动学可行的搜索"。
只用标准库,`g++ -std=c++17` 即编。

```cpp
// 最简演示:Hybrid A* 的核心——在连续状态(x,y,θ)上按车辆运动学扩展节点搜索
// 区别于普通 A*(离散网格 4/8 邻居):用自行车模型生成连续状态转移,
// 搜出的路径天然运动学可行(车真能开)。这是 Apollo Open Space 的第一段。纯标准库。
#include <vector>
#include <queue>
#include <cmath>
#include <cstdio>
#include <unordered_set>

struct State { double x, y, theta; };

State Move(const State& s, double steer, double ds) {   // 自行车模型:演化到下一状态
  const double L = 2.5;                         // 轴距
  State n;
  n.x = s.x + ds * std::cos(s.theta);
  n.y = s.y + ds * std::sin(s.theta);
  n.theta = s.theta + ds / L * std::tan(steer); // 朝向按运动学变化
  return n;
}

long Key(const State& s) {                      // 连续状态离散化为栅格 key(去重防爆炸)
  long ix = std::lround(s.x / 0.5), iy = std::lround(s.y / 0.5);
  long it = std::lround(s.theta / (M_PI / 12));
  return (ix * 1000 + iy) * 100 + (it % 100 + 100) % 100;
}

int main() {
  State start{0, 0, 0}, goal{6, 3, 0};
  double goal_tol = 0.8;
  struct Node { State s; double g, f; };
  auto cmp = [](const Node& a, const Node& b){ return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
  std::unordered_set<long> closed;
  auto h = [&](const State& s){ return std::hypot(goal.x - s.x, goal.y - s.y); };
  open.push({start, 0, h(start)});

  double steers[3] = {-0.4, 0.0, 0.4};          // 左转/直行/右转:运动学邻居
  double ds = 0.8;
  int expanded = 0; bool found = false; State reach{};
  while (!open.empty()) {
    Node cur = open.top(); open.pop();
    long k = Key(cur.s);
    if (closed.count(k)) continue;
    closed.insert(k); ++expanded;
    if (h(cur.s) < goal_tol) { found = true; reach = cur.s; break; }
    if (expanded > 20000) break;
    for (double st : steers) {                   // 按运动学扩展子节点
      State ns = Move(cur.s, st, ds);
      if (ns.x < -2 || ns.x > 12 || ns.y < -5 || ns.y > 8) continue;
      if (closed.count(Key(ns))) continue;
      double ng = cur.g + ds;
      open.push({ns, ng, ng + h(ns)});
    }
  }
  std::printf("Hybrid A*: 起点(0,0,0) → 目标(6,3), 扩展 %d 节点\n", expanded);
  if (found)
    std::printf("找到可行粗解 (%.2f,%.2f,θ=%.2f), 距目标 %.2f ✓\n",
                reach.x, reach.y, reach.theta, h(reach));
  return 0;
}
// 运行输出:
//   Hybrid A*: 起点(0,0,0) → 目标(6,3), 扩展 37 节点
//   找到可行粗解 (6.38,2.74,θ=0.81), 距目标 0.46 ✓
```

**Step 2 — 正确要点。**
这份演示抓住了 Hybrid A\* 的本质。
其一,**用运动学模型扩展、而非网格邻居**:`Move` 用自行车模型生成子节点(`steers` 是转向角)——这是 Hybrid A\* 区别于普通 A\* 的核心。
普通 A\* 在网格上走上下左右,产出的路径有尖角、车开不了;Hybrid A\* 每步都是车辆运动学允许的转移,产出的路径天然可行。
其二,**连续状态 + 栅格去重**:状态 (x,y,θ) 是连续的,但用 `Key` 离散化做去重——既保留连续搜索的可行性,又防止节点无限膨胀。
这是 Hybrid A\* 的工程关键(纯连续会爆炸,纯离散不可行,折中)。
其三,**它产出的是粗解、要交给 OBCA 精修**:搜到的解"距目标 0.46m"——可行但不精确,这正是 warm-start 的角色(定 homotopy + 给可行初值),精确轨迹靠第二段 OBCA。
真实 Apollo 的 Hybrid A\* 还用 Reeds-Shepp 曲线(支持倒车)、更复杂的启发式和碰撞检测,但"运动学扩展 + 离散去重 + 产出粗解"的核心一致。

**Step 3 — 一个常见的错误写法。**
新手做泊车搜索时,最容易犯的错是**用普通栅格 A\*(上下左右邻居)代替运动学扩展**:

```cpp
// 反例:用普通栅格 A* 的 4/8 邻居扩展(忽略车辆运动学)
int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
for (int i = 0; i < 4; ++i) {                  // 错:上下左右移动,不考虑朝向和运动学
  State ns = {cur.x + dx[i]*step, cur.y + dy[i]*step, /* θ ??? */};
  // ...
}
// 问题:(1) 产出的路径是栅格折线,有 90° 尖角 —— 车的转弯半径有限,根本开不出来;
//       (2) 没有朝向 θ 的概念 —— 但泊车必须管朝向(车头朝哪、能不能倒进去);
//       (3) 完全忽略"车不能横向平移、不能原地转向"等非完整约束。
//       结果:搜出的"路径"在物理上不可执行,泊车失败。
```

这个错误把"几何最短路"当成了"车能开的路",忽略了车辆是**非完整系统**(不能横移、转弯半径有限),产出的路径中看不中用。

**Step 4 — 对比。**
正确版(Hybrid A\*)用自行车模型扩展、带朝向 θ、每步都满足运动学,产出的路径车真能开。
错误版(栅格 A\*)上下左右走、无朝向、忽略运动学,产出的路径有尖角、车开不了。
这个对比是所有"为非完整系统做搜索"的通用要点——**搜索的状态转移必须用系统的运动学模型生成,而非几何上的网格邻居**,否则搜出的路径不可执行。
Apollo Open Space 用 Hybrid A\* 正是这个道理。

### 带数字走一遍

把 Hybrid A\* demo 的输出摊开,能看清"运动学可行搜索"的运转。
场景:从起点 (0,0,θ=0)(朝东)搜到目标 (6,3),用三个转向角(左转-0.4/直行0/右转0.4)、每步前进 0.8m 扩展。
输出:扩展 37 个节点,找到可行粗解 (6.38,2.74,θ=0.81),距目标 0.46m(<0.8 容差)。
逐个数字读它的含义。
**为什么只扩展了 37 个节点就找到?** 因为 A\* 的启发式(到目标的欧氏距离 `h`)引导搜索朝目标走,不盲目扩展;加上 `Key` 把连续状态离散化去重,避免重复扩展邻近状态。
37 这个小数字说明启发式 + 去重让搜索很高效——真实 Hybrid A\* 在复杂泊车场景会扩展更多,但同样靠启发式控制规模。
**为什么终点是 (6.38,2.74,θ=0.81) 而非正好 (6,3,?)?** 因为 Hybrid A\* 按固定步长(0.8m)和固定转向角扩展,落点是离散转移的结果,不会正好命中目标——只要进入容差(0.46<0.8)就算到达。
这正是它产出"粗解"的体现:可行、接近,但不精确。
精确轨迹靠第二段 OBCA 精修。
**为什么终点朝向 θ=0.81(约 46°)?** 因为目标在右前方,车要左转朝向它,朝向自然从 0 变到约 46°——这个朝向是运动学扩展("每步朝向按 ds/L·tan(steer) 变化")累积的结果,保证了路径全程车头朝向连续、可执行。
**把目标改到正后方(如 (-3,0))再跑**,纯前进的 Hybrid A\* 会很难到达(车不能直接倒车)——这正暴露了练习 1 的点:要泊车必须加倒车能力(ds 取负 / Reeds-Shepp 曲线),否则车头朝前的车到不了正后方的车位。

### ⚠️ 常见陷阱

💡 **编程陷阱:用普通栅格 A\* 代替 Hybrid A\* 做泊车/开放空间搜索**
- **错误想法**:找路径不就是 A\* 吗,网格上跑个 A\* 就行。
- **现象 / 后果**:搜出的路径是带 90° 尖角的栅格折线,且没有朝向信息——车的转弯半径有限、不能横移,这种路径根本开不出来,泊车失败。
- **根本原因**:普通栅格 A\* 用几何邻居(上下左右)扩展,忽略了车辆是非完整系统(转弯半径有限、不能横向平移、朝向重要)。
- **正确做法**:用 Hybrid A\*——状态含朝向 (x,y,θ),按车辆运动学模型(自行车模型/Reeds-Shepp 曲线)扩展节点,保证每步转移车真能执行。

⚠️ **概念陷阱:以为 Open Space 也是 path-speed 解耦的**
- **错误想法**:Apollo 既然主链路用解耦,Open Space 应该也是先定路径再排速度。
- **现象 / 后果**:理解不了为什么 Open Space 单独用 OBCA、为什么它被称为"唯一的真正时空联合"。
- **根本原因**:泊车没有参考线(SL 图建不起来)、要前进后退(s 非单调)、要厘米级硬约束——解耦的前提全不成立。
  Open Space 必须把位置/朝向/速度/转向角放进一个 OBCA NLP 联合优化。
- **正确做法**:明确 Apollo 是"主链路解耦 + Open Space 联合"的混合设计;Open Space 是主链路里唯一不解耦、用 T3 那种联合优化的模块。

🧠 **思维陷阱:以为"联合优化更先进,所以应该全用联合"**
- **错误想法**:Open Space 的联合优化比 on-lane 的解耦"高级",工业栈应该全面转向联合。
- **现象 / 后果**:在 on-lane 高速场景硬上非凸联合优化,算不动、满足不了几十毫秒实时,系统失效。
- **根本原因**:联合优化(非凸 NLP)和解耦(凸 QP)各有适用边界——联合更精确但慢,解耦更快但需迭代弥补耦合。
  选哪个取决于**场景的实时预算和耦合强度**,不是"谁更先进"。
- **正确做法**:学 Apollo 的务实——按场景选方法:on-lane(高速、实时紧)用解耦,Open Space(低速、预算松、耦合强)用联合。
  工业的智慧在"因地制宜",不在"用最先进的"。

### 练习

1. **[实现]** 给上面的 Hybrid A\* 加入倒车能力:在 `steers` 之外,允许 `ds` 取负值(倒车),观察搜索能否产出"先前进再倒车"的路径。
   这对应 Reeds-Shepp 曲线的前进+倒车能力,是泊车必需的。
2. **[分析]** Hybrid A\* 用 `Key` 把连续状态离散化去重。
   如果栅格取得太粗(如 2m),会有什么问题?太细(如 0.05m)呢?这个粒度的选择体现了什么权衡?(提示:可行性 vs 节点数/内存)
3. **[跨章]** Open Space 的 "Hybrid A\* → OBCA" 和 on-lane 的 "DP → QP"、T3 的 "搜索/走廊 → 连续优化"是同一个模式。
   请说明这三者的共同结构(谁定 homotopy、谁求精),以及为什么连续优化(OBCA/QP)前面都需要一个离散搜索(Hybrid A\*/DP)做 warm-start。

---

## §T4.4 Autoware Universe:严格分层流水线 ⭐⭐⭐

> 前三节看的是 Apollo 一种工业架构。
> 这一节转向 Autoware——一套同样经过量产验证、但做了不同选择的架构。
> 它不搞 EM 迭代,而是把规划拆成一条严格单向的流水线。
> 对照着 Apollo 看,你会第一次清楚:"怎么组织规划栈"不止一个正确答案,存在一个取舍空间——这正是下一节(§T4.5)要展开的。

### 动机:另一种工业哲学——把规划拆成一条严格的流水线

Apollo 用 Scenario/Stage/Task 三层 + EM 半耦合迭代。
Autoware Universe(基金会维护的开源 L4 全栈,ROS 2 原生)给出了**另一种**工业架构哲学:**严格分层的流水线**。
它不像 Apollo 那样在 SL/ST 之间迭代,而是把规划拆成一串**单向流动**的层:全局路由 → 行为路径 → 行为速度 → 路径优化 → 运动速度 → 速度平滑,数据像流水线一样从上一层流到下一层,每层只做一件事、只往后传。
为什么要研究这第二种架构?因为它代表了和 Apollo 不同的取舍:Apollo 追求"准联合"(EM 迭代弥补解耦),Autoware 追求"彻底解耦 + 规则叠加"(每层独立、靠后面的层叠加修正)。
两种都是经过大规模实战的工业方案,对比着看,你才能理解工业架构设计的**取舍空间**——不是只有一种"正确答案",而是不同的权衡导向不同的架构。
而且 Autoware 是 ROS 2 原生、用 pluginlib 做模块热插拔,它的工程化方式(尤其插件机制)对所有 ROS 机器人系统都有借鉴价值。
换句话说,即便你不做自动驾驶,只要你用 ROS 2 做任何机器人系统,Autoware 的模块化设计(怎么用 pluginlib 把功能拆成可热插拔的插件、怎么用配置驱动不同的部署)都是值得借鉴的工程范本——这也是把它和 Apollo 并列研究的额外价值。

### 如果不这样做会怎样(反面)

如果不用严格分层、而是把所有规划逻辑混在一起,会回到 §T4.1 那个"巨型函数"的困境(不可维护、不可协作、不可测试)。
那为什么 Autoware 选"严格单向流水线"而非 Apollo 的"迭代"?这背后是一个取舍。
**严格分层的代价**:它放弃了层间的迭代反馈——路径层(BPP)定了路径后,速度层(MVP)只能在这条路径上排速度,不能反过来让路径层重新调整。
所以强耦合场景(路径和速度需要协同),Autoware 不如 Apollo 的 EM 迭代灵活。
**严格分层的好处**:换来了**极致的清晰和可验证性**——每层职责单一、输入输出明确、可独立替换和测试;数据单向流动,没有迭代带来的收敛性、稳定性顾虑;每层叠加的约束(尤其 BVP 那些交通规则模块)可追溯、可解释。
所以 Autoware 的选择是:**用"严格解耦 + 规则叠加"换取最大的工程清晰度和可验证性,代价是强耦合场景的协调能力**。
这和 Apollo"用 EM 迭代换协调能力,代价是收敛性管理"是两种不同的工业取舍——没有绝对优劣,看你更看重什么。

**严格分层的可验证性,具体好在哪?**
"可验证"听起来抽象,落到实处有几个具体好处。
**每层可独立测试**:因为每层输入输出明确、不依赖后续层的反馈,可以单独给一层喂测试输入、验证它的输出对不对——比如单独测 BVP 在某路口场景是否正确插入了停止点,不用跑整个流水线。
**问题可定位到层**:出了问题(如某场景规划不当),因为数据单向流动,可以顺着流水线逐层检查"数据在哪一层变得不对",快速定位是 BPP、BVP 还是哪层的问题。
迭代式架构(如 Apollo EM)因为 path/speed 反复交互,定位问题相对难一些。
**无收敛性顾虑**:单向流水线一遍走完就出结果,不像迭代式架构要担心"会不会不收敛、收敛到哪"。
这让系统行为更可预测、更易形式化分析。
这些正是 Autoware 用"放弃层间反馈"换来的——对一个要通过严格安全审查、要多方协作开发的开源项目,这种可验证性和可预测性极有价值。
这也再次说明:架构取舍没有绝对优劣,Autoware 的严格分层在协调能力上让步,但在可验证性上得分——是否值得,取决于项目最看重什么。

### 历史:Autoware.AI 到 Universe

Autoware 也有清晰的演进。
**Autoware.AI**(2015-2021,基于 ROS 1)是首个完整的开源 L4 自动驾驶栈,由名古屋大学和 Tier IV 发起——它证明了"开源也能做全栈自动驾驶",但 ROS 1 的架构和代码组织逐渐跟不上需求。
**Autoware Universe**(2022 起,基于 ROS 2)是重构后的现代版本,由 Autoware 基金会维护(`autowarefoundation/autoware_universe`,Apache-2.0,C++/ROS 2)——它确立了严格分层的规划流水线,用 pluginlib 做模块热插拔,文档用 mkdocs 规范完整。
**2025+ 的新方向**:Autoware 开始接纳端到端——`autoware_diffusion_planner`(2025 年合并入 universe)把扩散模型规划器引入这个 ROS 2 工业栈,Tier IV 在 IEEE IV 2025 报告了"面向 Autoware 的模块化端到端扩散规划器";配套有从候选轨迹里筛选安全可行解的安全过滤思路。
这让 Autoware 成为工业栈中较早正式接纳扩散规划的代表。
所以 Autoware 的脉络是:开源全栈先行(.AI)→ ROS 2 重构 + 严格分层(Universe)→ 端到端 + 安全过滤(2025+)。
这条线和 Apollo 的演进并行,共同勾勒了工业规划栈的发展轨迹。

**为什么 Autoware 选 ROS 2,而 Apollo 自研 CyberRT?**
这是两个框架一个有意思的分歧,反映了不同的取舍。
Apollo 早期也用过 ROS,但后来自研 CyberRT——因为百度作为一家公司,有资源和动机打造一个为自动驾驶深度优化的中间件(共享内存、确定性调度),换取极致的性能和对整个技术栈的掌控。
代价是:它是 Apollo 专属的,生态、工具、社区都要自己建。
Autoware 作为一个**开源基金会项目**,选择拥抱标准 ROS 2——因为它的目标是让全球开发者、研究机构、公司都能方便地参与和使用。
ROS 2 有庞大的生态(现成的驱动、工具、可视化、社区),站在这个标准之上,Autoware 能借力整个 ROS 生态,降低参与门槛。
代价是:ROS 2 + DDS 的通信开销比定制的共享内存方案大一些,对极致延迟的场景不如 CyberRT。
**这个分歧的本质**:Apollo 走"垂直整合、定制优化"路线(像苹果自研芯片),Autoware 走"拥抱标准、生态借力"路线(像安卓用通用硬件)。
前者性能和掌控更强但封闭,后者开放通用但有标准化的开销。
两条路都对——取决于你是"一家公司追求极致性能",还是"一个开放社区追求广泛参与"。
这也提示你:中间件的选择不只是技术问题,还和项目的组织形态、目标人群深度相关。

### 理论:六层流水线 + pluginlib 热插拔

**整体流水线。**
Autoware Universe 的规划是一条严格单向的流水线:

```text
   MissionPlanner            ← 全局路由:从当前位置到目的地的车道级路径
        │  (route)
        ▼
   ScenarioSelector          ← 选场景:LaneDriving 还是 Parking
        │
   ┌────┴─────────────────────────────── LaneDriving ───────────────────────────┐
   │                                                                             │
   │  BehaviorPathPlanner (BPP)   ← 行为层·路径:生成要走的路径(含变道/避障)      │
   │    模块(pluginlib): lane_change / goal_planner(靠边停)/ start_planner       │
   │                    / static_obstacle_avoidance / dynamic_obstacle_avoidance  │
   │    输出: PathWithLaneId(带车道 id 的路径)                                    │
   │        │                                                                    │
   │        ▼                                                                    │
   │  BehaviorVelocityPlanner (BVP)  ← 行为层·速度:按交通规则在路径上插停车/减速点 │
   │    模块(pluginlib): intersection / crosswalk / traffic_light / stop_line     │
   │                    / run_out / occlusion_spot / ...                         │
   │    输出: 在 path 上插入了 stop/减速点                                         │
   │        │                                                                    │
   │        ▼                                                                    │
   │  PathOptimizer (MPT)     ← 路径优化:Model Predictive Trajectory,Frenet 横向 QP│
   │    输出: 平滑的路径(速度从输入 path 取,零阶保持——这一层不优化速度!)         │
   │        │                                                                    │
   │        ▼                                                                    │
   │  MotionVelocityPlanner   ← 运动速度:按障碍调速度(巡航/停车/减速)             │
   │    模块: obstacle_cruise / obstacle_stop / obstacle_slow_down / dynamic_stop │
   │    输出: 带速度限制的轨迹                                                     │
   │        │                                                                    │
   │        ▼                                                                    │
   │  VelocitySmoother        ← 速度平滑:jerk-limited QP,产出最终速度 profile     │
   │    输出: 最终轨迹(位置+速度+加速度)                                          │
   └─────────────────────────────────────────────────────────────────────────────┘
                                                          Parking → FreespacePlanner
                                                          (Hybrid A* / Reeds-Shepp)
```

**六层各管什么。**
**MissionPlanner(任务规划)**:全局路由,算出从当前位置到目的地的车道级路径(走哪些车道),不管具体轨迹。
**BehaviorPathPlanner(BPP,行为路径)**:决定"要走的路径"——包括变道(lane_change)、靠边停车(goal_planner)、起步(start_planner)、静态/动态避障。
它用 pluginlib 加载这些行为模块,输出带车道 id 的路径 `PathWithLaneId`。
路径的横向变形(如避障 shift)用 constant-jerk profile 保证平滑。
**BehaviorVelocityPlanner(BVP,行为速度)**:按**交通规则**在路径上插入停车/减速点——路口(intersection)、人行横道(crosswalk)、红绿灯(traffic_light)、停止线(stop_line)、突然出现的障碍(run_out)、遮挡盲区(occlusion_spot)等,每个是一个 pluginlib 模块。
它不改路径形状,只在路径上"贴"速度约束(在哪要停、在哪要慢)。
**PathOptimizer(MPT,路径优化)**:用 Model Predictive Trajectory——一个 Frenet 横向的 QP(OSQP 求解,自行车模型线性化)——把路径优化得平滑、动力学可行。
关键:**这一层只优化路径(横向),速度直接从输入 path 取、零阶保持**——这是 Autoware 严格解耦的鲜明体现(路径优化和速度优化彻底分开)。
**MotionVelocityPlanner(运动速度)**:根据障碍调整速度——跟车巡航(obstacle_cruise)、遇障停车(obstacle_stop)、减速(obstacle_slow_down)、动态障碍停车。
输出带速度限制的轨迹。
**VelocitySmoother(速度平滑)**:最后一层,用 jerk-limited QP(OSQP,可选 JerkFiltered/L2/Linf 范数)把速度平滑成满足速度/加速度/jerk 约束的最终 profile。
**Parking 分支**:停车场景走 FreespacePlanner——用 Hybrid A\*(支持 Reeds-Shepp 倒车)/ RRT\* 等,和 Apollo Open Space 的搜索段类似,但 Autoware 的 freespace 通常只到搜索(无 OBCA 那样的 NLP 精修)。

**pluginlib:热插拔的关键。**
Autoware 的 BPP 和 BVP 用 ROS 的 **pluginlib** 机制加载模块——模块是编译成共享库的插件,在 `default_preset.yaml` 里配 `true`/`false` 就能启用/禁用,下次启动生效,不用改主程序代码、不用重新编译核心。
比如不需要路口处理,就把 `intersection` 模块设 `false`;要加一个自定义模块,写成插件、注册、配置启用即可。
这和 Apollo 的 Plugin 机制目标一致(模块化、可扩展),实现不同(Apollo 是 Task 注册 + 配置驱动,Autoware 是 ROS pluginlib 动态加载共享库)。
**为什么这对工程重要**:它让"增删功能模块"变成配置操作而非代码改动——不同车型、不同场景可以用不同的模块组合,同一套核心代码支持多种部署。
这是工业框架可维护、可定制的关键设施。

**一帧 Autoware 规划的数据流:看数据怎么变形。**
理解了六层和 pluginlib,再跟一帧规划的数据流走一遍,你就能把"分层流水线"和"代码里的 topic/message"对上号。
数据在六层间逐步变形:
**输入**:MissionPlanner 拿到目的地,产出 `route`(车道级路径——走哪些 lanelet)。
**BPP 后**:BehaviorPathPlanner 把 route 变成 `PathWithLaneId`——一条带车道 id 的具体路径(已含变道、避障的横向 shift),但还没有速度约束。
**BVP 后**:BehaviorVelocityPlanner 在这条路径上**插入速度约束点**——路口前减速、红灯前停止线、人行横道前停车等。
路径形状没变,但路径上多了"在哪要停、在哪要慢"的标记。
**PathOptimizer(MPT)后**:把路径优化成平滑、动力学可行的 `Trajectory`(轨迹点);注意此时**速度是从输入路径零阶保持取来的**,MPT 没动速度。
**MotionVelocityPlanner 后**:根据障碍(前车、横穿物)调整速度——跟车巡航、遇障减速/停车,产出带速度限制的轨迹。
**VelocitySmoother 后**:最后把速度平滑成满足速度/加速度/jerk 约束的最终 profile,产出 Control 模块能跟踪的 `Trajectory`(每点含位置、速度、加速度,约 0.1s 间隔)。
读这个数据流,几件事变清晰了:**数据严格单向流动**(route → PathWithLaneId → 带速度标记的 path → Trajectory → 带速度的 Trajectory → 平滑 Trajectory),没有回头、没有迭代——这正是 Autoware"严格分层"的体现。
**路径和速度彻底分开处理**:BPP/MPT 管路径(横向),BVP/MotionVelocity/VelocitySmoother 管速度(纵向),中间 MPT 那一步速度只是零阶保持透传。
对比 Apollo 的 EM 迭代(path/speed 交替协调),Autoware 这种"一条道走到黑"的单向流水线,清晰可验证,但放弃了层间反馈——这就是 §T4.5 要对比的两种哲学的微观体现。
读 Autoware 源码时,你会看到这些层是独立的 ROS 2 节点,数据通过 topic 在节点间传递,每个 message 类型(PathWithLaneId、Trajectory)对应数据流的一个阶段。

**BPP / BVP 主要模块速览。**
BPP 和 BVP 是两个 pluginlib 模块最多的层,读 Autoware 源码会频繁遇到这些模块名。
把它们的职责列出来,便于对照代码:

| 层 | 模块 | 职责 |
|----|------|------|
| BPP | lane_change | 变道:生成安全的变道路径 |
| BPP | goal_planner | 靠边停车(pull-over):规划停到路边目标点的路径 |
| BPP | start_planner | 起步(pull-out):从路边起步汇入车道 |
| BPP | static_obstacle_avoidance | 静态避障:对静止障碍做横向 shift 绕行 |
| BPP | dynamic_obstacle_avoidance | 动态避障:对移动障碍的路径规避 |
| BVP | intersection | 路口:无信号/有信号路口的让行与通过决策 |
| BVP | crosswalk | 人行横道:检测行人、必要时停车 |
| BVP | traffic_light | 红绿灯:按灯色插入停止/通过 |
| BVP | stop_line | 停止线:在停止线处插入停车点 |
| BVP | run_out | 突现障碍:对可能突然冲出的物体减速 |
| BVP | occlusion_spot | 遮挡盲区:对视野被遮挡处预防性减速 |

读这张表,几件事变清晰了:**BPP 的模块都在"改路径形状"**(变道、避障是横向的路径调整),**BVP 的模块都在"按规则贴速度约束"**(路口、红灯、人行横道是在路径上插停车/减速点,不改路径形状)——这再次印证 Autoware 的路径/速度严格分层。
而每个模块都是 pluginlib 插件,在 `default_preset.yaml` 里配 `true`/`false`——比如做高速公路场景,可以关掉 crosswalk、traffic_light 这些城区模块,只留 lane_change 等。
这就是 §T4.4 反复强调的"配置即增删功能"的实际样子:同一套 Autoware,通过不同的模块组合,适配高速、城区、园区等不同场景。

### 代码实现(为什么 → 正确 → 错误 → 对比)

**Step 1 — 为什么这样写。**
真实 pluginlib 依赖 ROS 的类加载器、共享库(.so)、`PLUGINLIB_EXPORT_CLASS` 宏等设施,无法脱离 ROS 运行。
但 pluginlib 的**核心机制——按名字字符串从工厂创建模块、按配置决定加载哪些**——可以用纯 C++ 模拟清楚。
下面这份**可直接编译运行**的代码,演示 Autoware BVP 的模块如何注册到工厂、如何按配置(模拟 `default_preset.yaml`)热插拔。
它用 `std::function` 工厂 + 配置 map,贴合 pluginlib 的设计思想。
只用标准库,`g++ -std=c++17` 即编。

```cpp
// 最简演示:Autoware 风格的 pluginlib 模块热插拔机制
// 核心:模块注册到工厂,运行时按配置(default_preset.yaml 的 true/false)
// 决定加载哪些 → 无需改核心代码、无需重编译就能增删功能模块。纯标准库模拟。
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <functional>
#include <cstdio>

// 模块基类(BVP 各交通规则模块都继承它)
class VelocityModule {
 public:
  virtual ~VelocityModule() = default;
  virtual std::string Name() const = 0;
  virtual void PlanVelocity(std::string& log) = 0;   // 在路径上插速度约束
};

class IntersectionModule : public VelocityModule {
 public:
  std::string Name() const override { return "intersection"; }
  void PlanVelocity(std::string& log) override { log += "  [intersection] 路口前插减速点\n"; }
};
class CrosswalkModule : public VelocityModule {
 public:
  std::string Name() const override { return "crosswalk"; }
  void PlanVelocity(std::string& log) override { log += "  [crosswalk] 人行横道前插停车点\n"; }
};
class TrafficLightModule : public VelocityModule {
 public:
  std::string Name() const override { return "traffic_light"; }
  void PlanVelocity(std::string& log) override { log += "  [traffic_light] 红灯前插停止线\n"; }
};

// 插件工厂:按名字字符串创建模块(pluginlib 的核心)
class ModuleFactory {
 public:
  using Creator = std::function<std::unique_ptr<VelocityModule>()>;
  void Register(const std::string& name, Creator c) { creators_[name] = std::move(c); }
  std::unique_ptr<VelocityModule> Create(const std::string& name) {
    auto it = creators_.find(name);
    return it == creators_.end() ? nullptr : it->second();
  }
 private:
  std::map<std::string, Creator> creators_;
};

int main() {
  ModuleFactory factory;
  // 注册可用模块(真实 pluginlib 用宏 PLUGINLIB_EXPORT_CLASS 在库里自动注册)
  factory.Register("intersection",  []{ return std::make_unique<IntersectionModule>(); });
  factory.Register("crosswalk",     []{ return std::make_unique<CrosswalkModule>(); });
  factory.Register("traffic_light", []{ return std::make_unique<TrafficLightModule>(); });

  // 模拟 default_preset.yaml:哪些模块启用
  std::map<std::string, bool> preset = {
    {"intersection", true}, {"crosswalk", false}, {"traffic_light", true},
  };  // crosswalk=false:这个场景不需要 → 关掉

  std::vector<std::unique_ptr<VelocityModule>> loaded;
  std::printf("按 preset 加载 BVP 模块:\n");
  for (auto& [name, enabled] : preset) {
    if (!enabled) { std::printf("  %s: 禁用(跳过)\n", name.c_str()); continue; }
    if (auto m = factory.Create(name)) {
      std::printf("  %s: 已加载\n", name.c_str()); loaded.push_back(std::move(m));
    }
  }
  std::string log;
  for (auto& m : loaded) m->PlanVelocity(log);
  std::printf("\nBVP 速度规划(%zu 个模块):\n%s", loaded.size(), log.c_str());
  return 0;
}
// 运行输出:
//   crosswalk: 禁用(跳过)
//   intersection: 已加载
//   traffic_light: 已加载
//   BVP 速度规划(2 个模块):
//     [intersection] 路口前插减速点
//     [traffic_light] 红灯前插停止线
```

**Step 2 — 正确要点。**
这份演示抓住了 pluginlib 的本质。
其一,**按名字字符串创建模块**:`factory.Create("intersection")` 用字符串拿到对象——这是热插拔的关键。
因为配置文件里只有字符串(模块名),必须能"按名字造对象"才能配置驱动。
真实 pluginlib 用 `PLUGINLIB_EXPORT_CLASS` 宏让模块在 .so 库里自动注册,效果一样。
其二,**配置决定加载哪些**:`preset` 这个 map(模拟 `default_preset.yaml`)的 `true`/`false` 决定加载哪些模块——`crosswalk=false` 就跳过。
这就是"改配置增删功能"。
其三,**模块多态、统一调度**:所有模块继承 `VelocityModule`,主程序用 `vector<VelocityModule*>` 统一执行——加新模块不用改调度代码。
真实 pluginlib 还跨越编译边界(模块在独立 .so 里,主程序运行时动态加载),实现了"加模块不重编译核心",但"按名字创建 + 配置启用 + 多态调度"的核心一致。

**Step 3 — 一个常见的错误写法。**
新手实现模块化时,最容易犯的错是**用一串 `if-else` 硬编码判断加载哪些模块**:

```cpp
// 反例:用 if-else 硬编码模块的创建,而非工厂 + 配置
std::vector<std::unique_ptr<VelocityModule>> loaded;
if (enable_intersection)  loaded.push_back(std::make_unique<IntersectionModule>());
if (enable_crosswalk)     loaded.push_back(std::make_unique<CrosswalkModule>());
if (enable_traffic_light) loaded.push_back(std::make_unique<TrafficLightModule>());
// ... 每加一个模块,这里就要加一行 if
// 问题:(1) 加新模块必须改这段核心代码、重新编译 —— 失去"热插拔"的意义;
//       (2) 主程序必须 #include 所有模块的头文件、知道所有模块类 —— 强耦合;
//       (3) 第三方无法在不改你代码的前提下加自己的模块。
//       正确做法是工厂按名字创建 + 配置驱动,主程序对具体模块类一无所知。
```

这个错误把"有哪些模块"硬编码进了主程序,每次增删都要改核心、重编译,完全失去了 pluginlib"配置即增删、核心不动"的价值。

**Step 4 — 对比。**
正确版(工厂 + 配置)里,主程序只认 `VelocityModule` 基类和配置里的字符串,对具体模块类一无所知;加模块 = 注册 + 配置,核心代码不动。
错误版(if-else 硬编码)里,主程序 #include 并显式 new 每个模块类,加模块要改核心、重编译,且和所有模块强耦合。
这个对比是所有"插件化"设计的通用要点——**用工厂按名字创建 + 配置驱动,让主程序与具体实现解耦**,从而实现"增删功能不改核心代码"。
Autoware pluginlib、Apollo Task 注册,本质都是这个思想。

### 带数字走一遍

把 pluginlib demo 的输出摊开,能看清"配置驱动的热插拔"怎么运转。
场景:工厂注册了 3 个 BVP 模块(intersection/crosswalk/traffic_light),preset 配置 intersection=true、crosswalk=false、traffic_light=true。
输出:`crosswalk: 禁用(跳过)` → `intersection: 已加载` → `traffic_light: 已加载` → 执行 2 个模块(路口插减速点、红灯插停止线)。
逐个数字读它的含义。
**为什么 crosswalk 被跳过、最终只加载 2 个?** 因为 preset 里 crosswalk=false——主程序读配置,遇到 false 就不创建该模块。
这就是"改配置增删功能":想去掉人行横道处理,改一个 false 即可,不动任何代码。
最终加载数(2)= 配置里为 true 的模块数。
**为什么主程序能"按字符串"造出对象?** 因为工厂里注册了"名字→创建函数"的映射(`factory.Register("intersection", []{...})`)。
`factory.Create("intersection")` 查这个映射、调对应的创建函数。
这是热插拔的关键——配置文件里只有字符串,必须能按字符串造对象。
真实 pluginlib 用 `PLUGINLIB_EXPORT_CLASS` 宏让模块在 .so 库里自动注册进类加载器,效果一样。
**为什么主程序对具体模块类"一无所知"也能调度?** 因为所有模块继承 `VelocityModule` 基类,主程序只持有基类指针 `vector<VelocityModule*>`、只调虚函数 `PlanVelocity`——多态让它不需要知道具体是 intersection 还是 traffic_light。
所以加新模块不用改调度代码。
**把 crosswalk 改成 true、再加一个没注册的模块名(如 "foobar")到 preset**,你会看到 crosswalk 被加载,而 "foobar" 因工厂里没注册返回 nullptr 被跳过——这演示了配置和注册的解耦:配置说"要什么",工厂决定"能造什么",这正是练习 1(加新模块)和真实系统模块管理的基础。

### ⚠️ 常见陷阱

💡 **编程陷阱:用 if-else 硬编码模块加载,而非工厂 + 配置**
- **错误想法**:加载哪些模块,用几个 `if (enable_xxx)` 判断一下就行,简单明了。
- **现象 / 后果**:每加一个新模块都要改这段核心代码并重新编译,主程序和所有模块强耦合,第三方无法在不改你代码的前提下扩展。
- **根本原因**:把"有哪些模块"这个易变的信息硬编码进了主程序,违背了插件化"核心稳定、模块可插拔"的目标。
- **正确做法**:工厂按名字字符串创建模块 + 配置文件驱动启用;主程序只认基类和配置,对具体模块类一无所知。
  增删模块 = 注册 + 改配置。

⚠️ **概念陷阱:以为 Autoware 的 PathOptimizer 也优化速度**
- **错误想法**:既然叫 Path"Optimizer"又在 trajectory 上工作,它应该把路径和速度都优化了。
- **现象 / 后果**:理解不了为什么 Autoware 后面还要单独的 MotionVelocityPlanner 和 VelocitySmoother。
- **根本原因**:Autoware 严格解耦——PathOptimizer(MPT)只优化路径(横向 QP),**速度直接从输入 path 取、零阶保持**,完全不动;速度由后面的 MotionVelocityPlanner 和 VelocitySmoother 单独处理。
- **正确做法**:明确 Autoware 的路径优化和速度优化是**彻底分开的两层**(MPT 管路径、VelocitySmoother 管速度),这是它"严格解耦"哲学的鲜明体现,和 Apollo 的 EM 半耦合形成对比。

🧠 **思维陷阱:以为分层越多越好**
- **错误想法**:Autoware 分了六层,层次越细、模块越多,架构就越好。
- **现象 / 后果**:盲目增加层数和模块,层间数据传递开销、调试复杂度、延迟都上升,反而降低系统性能和可维护性。
- **根本原因**:分层是手段不是目的——每多一层就多一次数据转换和延迟,层数要匹配问题的自然结构,而非越多越好。
  Autoware 的六层是其严格解耦哲学的结果,不是"层多就好"。
- **正确做法**:分层粒度要恰当——每层有清晰单一的职责、层间接口稳定。
  Apollo 的三层和 Autoware 的六层都是合理的,关键是匹配各自的设计哲学(半耦合 vs 严格解耦),而非比谁层多。

### 练习

1. **[实现]** 给上面的 pluginlib 骨架加一个新的 BVP 模块 `StopLineModule`(停止线):注册它、在 `preset` 里启用它,运行验证它被加载执行。
   体会"加新模块 = 注册 + 配置,不改调度代码"。
2. **[分析]** Autoware 的 PathOptimizer 速度零阶保持、VelocitySmoother 才优化速度。
   对比 Apollo 的 EM 迭代(path/speed 交替协调)。
   在"前方有车需要变道超车"这种强耦合场景,哪种架构更可能产出协调的轨迹?为什么?这体现了两种架构的什么取舍?
3. **[设计]** 真实 pluginlib 让模块编译成独立的 .so 库、主程序运行时动态加载——这样加模块连核心都不用重新编译。
   我们的 demo 没做到这点(所有模块在一个文件里)。
   请说明:要实现"加模块不重编译核心",模块和主程序之间需要怎样的边界?(提示:基类头文件 + 动态库加载)

---

## §T4.5 Apollo vs Autoware:两种工业取舍 ⭐⭐

讲完两个框架,这一节横向对比——不是评判谁好谁坏,而是理解两种工业取舍各自的逻辑。

### 动机:为什么要对比两个框架?

你可能会问:学一个框架不就够了,为什么要对比?
因为**对比才能看清"设计空间"**。
单看 Apollo,你会以为"规划栈就该这么设计";单看 Autoware,你会以为"规划栈就该那么设计"。
只有把两个都经过大规模量产验证、却做了不同选择的框架放在一起,你才能看清:在"怎么组织规划栈"这个问题上,**存在一个取舍空间**,Apollo 和 Autoware 是这个空间里的两个不同点,各自的选择都有清晰的理由和代价。
理解了这个取舍空间,你将来设计自己的规划栈时,就不是"抄某一个",而是"知道有哪些选项、各自的代价,据此为你的场景选择"。
这比记住任何一个框架的细节都重要。

### 理论:逐维度对比

**核心对比表。**

| 维度 | Apollo | Autoware Universe |
|------|--------|------------------|
| 时空策略 | **半耦合**:EM 迭代在 SL+ST 间交替 | **严格解耦**:各层单向叠加约束 |
| ST 表达 | 显式 ST 图 + DP+QP | 无显式 ST 图 |
| 动态避障 | ST 图中 nudge/yield/overtake 统一决策 | 纵向 obstacle_cruise + 横向 shift 分离处理 |
| 路径/速度优化 | EM 迭代协调 | MPT 优化路径、VelSmoother 优化速度,彻底分开 |
| 联合优化 | Open Space 用 OBCA + Ipopt(真联合) | freespace 仅 Hybrid A\*(无 NLP 精修) |
| 中间件 | CyberRT(共享内存,低延迟) | ROS 2 + DDS(标准、生态丰富) |
| 扩展机制 | Task/Plugin 注册 + 配置驱动 | pluginlib 动态加载共享库 |
| 端到端 | 学习模式实验性(有限) | diffusion_planner + 安全过滤(2025+) |
| 代码规模 | planning 模块 ~10 万行 C++ | planning 相关 ~5 万行 C++ |
| 文档/社区 | 中文文档 + 社区丰富 | 英文 mkdocs 规范完整 |

**怎么用这张表。**
对照表不是用来背的,而是用来**做选型决策**的。
用法是:先确定你的项目在哪些维度上有硬约束或强偏好,再看哪个框架在这些维度上更契合。
比如,你的项目**对泊车精度要求高**——看"联合优化"那行:Apollo 有 OBCA NLP 精修、Autoware 的 freespace 只到搜索,那 Apollo 更契合(或你要在 Autoware 上自己补 OBCA)。
你的项目**团队熟悉 ROS 2、要做开源贡献**——看"中间件"和"文档"行:Autoware 是 ROS 2 原生、英文文档规范,更契合。
你的项目**要快速迭代、频繁 A/B 测试不同模块**——看"扩展机制"行:两者都支持(Apollo Task 注册、Autoware pluginlib),但 Autoware 的 pluginlib 在"运行时热插拔"上更彻底。
你的项目**对延迟极度敏感**——看"中间件"行:Apollo 的 CyberRT 共享内存延迟更低。
关键是:**没有哪个框架在所有维度都赢**——Apollo 功能更全、联合更强、延迟更低,Autoware 更标准、更清晰、更易协作。
选型就是看你的项目把哪些维度当硬约束。
这张表的价值,正是把"两个框架的差异"整理成可逐维度比对的决策依据。
下面对几个最关键的维度再深入解读。

**几个关键维度的深入解读。**
**时空策略(最核心的差异)**:Apollo 半耦合——形式上解耦(SL/ST 各自凸子问题)、但通过 EM 迭代引入协调;Autoware 严格解耦——路径层、速度层单向流动,后层只能在前层结果上叠加约束、不能反馈。
这导致:强耦合场景(如需路径速度协同的变道超车),Apollo 的迭代更灵活,Autoware 靠"路径层先做避障 shift、速度层再调速"的分离处理应对,协调性弱一些但更清晰。
**ST 表达**:Apollo 有显式的 ST 图,动态障碍的 yield/overtake 决策在 ST 图上统一、可视化、可解释;Autoware 没有显式 ST 图,纵向交互由 MotionVelocityPlanner 的 obstacle_cruise/stop 处理。
Apollo 的显式 ST 图在调试动态场景时更直观。
**联合优化的范围**:两者都在泊车用 Hybrid A\* 搜索,但 Apollo 还接了 OBCA NLP 做精修(真联合),Autoware 的 freespace 通常止于搜索。
所以 Apollo 的泊车轨迹更平滑、更精确。
**中间件**:Apollo 的 CyberRT 用共享内存,延迟低、为自动驾驶定制;Autoware 用标准 ROS 2 + DDS,生态丰富、通用。
这是"定制高性能"vs"标准通用"的取舍。
**代码与文档**:Apollo planning 约 10 万行(功能多、含完整 Open Space)、中文社区活跃;Autoware 约 5 万行(更精简)、英文文档规范。

> **多视角对照**:Apollo 和 Autoware 的差异,用几个日常类比能一下抓住。
> **像"集成套件 vs Unix 小工具"**:Apollo 像一个功能齐全的集成套件(自带 EM、Open Space、CyberRT,开箱即用、功能多),Autoware 像 Unix 哲学的小工具组合(每层一个小而专的工具、用管道串起来、可自由替换)。
> 这个视角解释了代码规模(10 万 vs 5 万)和扩展方式的差异。
> **像"紧密协作 vs 流水线分工"**:Apollo 的 EM 像一个小团队反复开会协调(path 和 speed 反复商量),Autoware 像工厂流水线分工(每个工位做完往下传、不回头)。
> 这个视角解释了"半耦合 vs 严格解耦"在协调能力和清晰度上的取舍。
> **像"定制赛车 vs 标准量产车"**:Apollo 的 CyberRT 是为自动驾驶定制的高性能中间件(像定制赛车,性能极致但专用),Autoware 用标准 ROS 2(像量产车,通用、生态丰富、易维护)。
> 这个视角解释了中间件选择背后"定制性能 vs 标准通用"的权衡。
> 这些类比不是要分高下,而是帮你快速把握两个框架的"性格":Apollo 偏"功能完整、性能定制、务实协调",Autoware 偏"模块清晰、标准通用、严格分层"。
> 性格不同源于取舍不同,取舍不同源于目标不同——没有更好,只有更适合谁。

**两种哲学的本质。**
把所有维度归纳,两个框架体现两种工业哲学。
**Apollo:务实的"准联合"**——在解耦的高效骨架上,用 EM 迭代尽量找回耦合协调,用 Open Space 的真联合处理必须联合的场景。
它的取向是"在实时预算内尽量逼近联合优化的质量"。
**Autoware:极致的"清晰解耦"**——把规划拆成职责单一、单向流动、可独立验证的层,用规则叠加和后层修正应对复杂性。
它的取向是"用最大的工程清晰度和可验证性,换强耦合场景的一点协调能力"。
没有谁对谁错:Apollo 的取向适合追求规划质量、有定制中间件能力的量产团队;Autoware 的取向适合追求开源标准、模块清晰、易于多方协作的场景。
理解这两种哲学,比记住任何细节都重要。

**如果让你从头设计一个规划栈,会怎么选?**
这正是本节练习要你思考的,这里给个思考框架。
不必非此即彼地"抄 Apollo"或"抄 Autoware",而是按你的项目特点,在几个关键决策上各自取舍。
**决策一:用三层架构(Scenario/Stage/Task)还是分层流水线?** 如果你的场景多样、需要灵活的场景切换(车道/泊车/路口/特殊场景),三层架构的 Scenario FSM 更自然;如果你的规划流程相对固定、追求每步清晰可验证,分层流水线更直接。
多数复杂自驾倾向三层(场景多),简单场景的机器人可用流水线。
**决策二:on-lane 用半耦合还是严格解耦?** 如果你的场景常有强耦合(频繁变道、密集交互),半耦合(EM)的协调能力更值得;如果耦合大多较弱、更看重清晰可验证,严格解耦够用且更省心。
**决策三:中间件用定制还是标准?** 看你是有资源做垂直整合的公司(可考虑定制,极致性能),还是要快速开发、借力生态(用 ROS 2)。
多数团队没有自研中间件的资源,ROS 2 是务实之选。
**决策四:怎么为端到端预留接口?** 不管选什么,都该为未来的端到端 + 安全过滤预留位置(§T4.6)——把规划做成"可以替换/叠加一个轨迹生成器 + 一个安全过滤器"的结构。
**核心原则**:不存在"最好的架构",只有"最适合你的场景、团队、目标的架构"。
Apollo 和 Autoware 给了你两个成熟参照系和一整套可借鉴的设计模式(三层/流水线、半耦合/解耦、定制/标准中间件、模块热插拔),你的工作是理解这些模式各自的代价,据此组合出适合自己的方案。
这种"理解取舍、按需组合"的能力,正是 T4 想给你的——它比记住任何一个框架的细节都珍贵、也都持久。

### 差异之外:两个框架的共识

讲了一堆差异,容易忽略一个同样重要的事实:**Apollo 和 Autoware 在很多根本问题上是一致的**——这些"共识"往往比差异更能告诉你"工业规划栈的不变规律"。
**共识一:都以解耦为骨架。** 无论 Apollo 的半耦合还是 Autoware 的严格解耦,核心都是 path-speed 解耦(T1),都没有把 on-lane 规划做成完全的时空联合。
差异只在"解耦后要不要迭代找回一点耦合"。
这印证了 §T4.6 的判断:解耦的工程可控性优势,是两个框架共同的选择基础。
**共识二:都用"离散搜索 + 连续优化"。** Apollo 的 DP→QP、Apollo/Autoware 泊车的 Hybrid A\*→(OBCA)、Autoware BPP 的行为决策→MPT 优化——都是"先用离散方法定 homotopy/决策,再用连续优化求精"。
这正是 T2/T3 那条贯穿主线,两个框架都遵循。
**共识三:都用 QP/OSQP 做核心优化。** Apollo 的 piecewise-jerk QP、Autoware 的 MPT QP 和 VelocitySmoother QP——都用二次规划(多数是 OSQP)做核心的轨迹优化。
因为 QP 凸、快、成熟,是工业实时优化的事实标准(T1/T3)。
**共识四:都做模块化 + 配置驱动 + 失败兜底。** Apollo 的 Task 注册 + Autoware 的 pluginlib——都让功能模块可插拔、可配置;都有规划失败时的降级机制。
这是工业系统可维护、可靠的共同要求。
看清这些共识很重要:它们说明**工业规划栈有一套相当稳定的"最佳实践"**(解耦骨架、离散+连续、QP 核心、模块化+兜底),不因框架而异;Apollo 和 Autoware 的差异,是在这套共同实践之上的不同取舍,而非根本分歧。
学规划栈,既要学这些跨框架的不变共识(更重要、更持久),也要理解具体框架的特定取舍。

**共识五:都把实时性当硬约束。** 无论 Apollo 还是 Autoware,规划都必须在严格的周期预算内完成(如 10 Hz / 100 ms)——这是行车安全的底线,慢一拍就可能酿成事故。
所以两个框架的所有设计(解耦、QP、固定迭代轮数、降级兜底)都服从于"必须实时"这个铁律:宁可用次优但快的方法,不用最优但超时的方法。
这也解释了为什么真正的时空联合(慢)只敢用在低速泊车——实时性预算这条红线,是悬在所有规划设计头上的。
理解了"实时是硬约束"这条共识,你就明白工业规划栈的很多"保守"选择,本质都是在实时红线下的不得已而为之——这和算法课上"追求最优"的思路截然不同,是工业和学术的一个根本差异。

### ⚠️ 常见陷阱

⚠️ **概念陷阱:以为对比的目的是选出"更好的框架"**
- **错误想法**:对比 Apollo 和 Autoware,就是要得出"哪个更好"的结论。
- **现象 / 后果**:执着于"Apollo 党 vs Autoware 党"的无谓争论,错过了对比真正的价值。
- **根本原因**:两个框架都经过大规模量产验证、各自的选择都有清晰理由——它们是"设计空间里的不同点",不是"好与坏"。
  脱离场景问"哪个更好"没有意义。
- **正确做法**:对比的目的是理解**取舍空间**——存在哪些设计选项、各自的代价。
  据此为你的具体场景(团队、中间件、场景复杂度、对质量/清晰度的偏好)做选择。

🧠 **思维陷阱:以为工业框架的架构是"一次设计定终身"**
- **错误想法**:Apollo/Autoware 的架构是某个天才一次设计出来的、固定不变的。
- **现象 / 后果**:把现有架构当教条照搬,不理解它们为什么持续演进(Apollo 1.5→11.0、Autoware.AI→Universe→端到端)。
- **根本原因**:工业架构是**随需求和技术持续演进**的——从 DARPA 手写状态机到 EM 工业化到端到端,每一步都是对当时痛点的回应。
  架构是活的,不是死的。
- **正确做法**:理解架构演进的**驱动力**(实时性、可维护性、新算法如扩散模型的出现),而非死记某个版本的目录结构。
  这样你能预判趋势、跟上演进。

### 练习

1. **[分析]** 针对"高速公路上跟车 + 偶尔变道"这个场景,分别分析 Apollo(EM 半耦合)和 Autoware(严格解耦)会如何处理,各自的优劣是什么?哪种架构在这个场景更合适?
2. **[设计]** 假设你要为一个**低速园区配送机器人**设计规划栈(场景简单、速度低、要求快速开发和易维护)。
   你会更多借鉴 Apollo 还是 Autoware 的设计?为什么?(提示:考虑场景复杂度、对联合优化的需求、团队规模)
3. **[思考]** 两个框架都在泊车用 Hybrid A\*,但 Apollo 接了 OBCA 精修而 Autoware 没有。
   如果你的产品对泊车精度要求很高(窄车位),这个差异意味着什么?你会怎么补上 Autoware 这块?

---

## §T4.6 为什么工业栈坚持解耦——以及新范式 ⭐⭐

### 动机:一个贯穿全章的问题——为什么不直接联合?

学到这里,一个问题应该越来越清晰也越来越尖锐:T3 教了那么多"真正时空联合"的方法(CILQR/MINCO/OBCA),为什么主流工业栈(Apollo on-lane、Autoware)**偏偏主要用解耦**,只在泊车这个角落用联合?
这不是工业界"不够先进"或"没跟上学术"——恰恰相反,这是经过大规模实战检验的、深思熟虑的工程选择。
这一节把这个贯穿全章的问题彻底讲清:解耦在工业上到底好在哪,以及 2025+ 端到端的新范式正在如何挑战(但还没推翻)这个选择。

### 理论:解耦的三大工程优势

为什么工业坚持解耦?不只是"快",还有三个更深的工程优势。
**优势一:可调试性。**
解耦后,SL 图(路径)和 ST 图(速度)可以**分别可视化、分别调参**。
车开得不对,工程师能快速定位:是路径问题(看 SL 图,是不是绕障绕得不好)还是速度问题(看 ST 图,是不是该让行没让)?路径调参和速度调参互不干扰。
联合优化是个黑盒大问题,出了问题难以定位是哪部分、调一个权重影响全局——可调试性差得多。
对一个要持续迭代、快速定位线上问题的量产系统,可调试性是生命线。
**优势二:可独立替换与 A/B 测试。**
解耦后,路径优化器和速度优化器是独立模块,可以**单独替换、单独 A/B 测试**。
想试一个新的速度优化算法?只换速度模块、路径模块不动,对比新旧速度模块的效果。
这种"单变量"的迭代方式,是工程上快速、可控地改进系统的基础。
联合优化是一个整体,换任何一部分都牵动全局,无法做这种干净的单变量实验。

举个具体例子:假设你想验证"一个新的速度优化算法能不能让乘坐更舒适"。
解耦架构下,你只需把速度模块从旧算法换成新算法、路径模块完全不动,在仿真或路测里跑两组对比——舒适度的差异就能干净地归因到"速度算法的改变",因为其他变量都没动。
这种单变量实验是工程改进的黄金标准:每次只改一个东西、观察效果、确认有益再保留。
而如果是联合优化,路径和速度在一个问题里耦合求解,你没法"只换速度部分而路径不变"——任何改动都会同时影响路径和速度,实验结果无法干净归因。
这种"能不能做单变量实验"的差异,直接决定了一个系统能不能被科学、可控地持续改进——这正是解耦在工程迭代上的隐形价值。
**优势三:安全可验证性。**
这是最关键的。
解耦后,ST 图上的 yield/overtake 决策是**显式、可追溯、可解释**的——"为什么这里减速?因为 ST 图上决定对这个行人 yield(让行)"。
对自动驾驶这种安全攸关系统,每个决策都要能被审查、被验证、出事后能复盘"当时为什么这么决策"。
解耦 + 显式决策提供了这种可解释性。
联合优化(尤其端到端)是个黑盒,"为什么这么开"难以解释——这在安全验证和事故归因时是大问题。

**可解释性为什么对自动驾驶生死攸关?**
对很多软件系统,黑盒不是大问题(推荐错了商品、识别错了图片,代价有限)。
但自动驾驶不同——它的决策关乎生命,可解释性是刚需,原因有三。
**其一,安全审查。** 量产前,自动驾驶系统要通过严格的安全审查(功能安全 ISO 26262、预期功能安全 SOTIF 等)。
审查要求能说清"系统在各种场景下会怎么决策、为什么安全"——一个黑盒"开起来还行但说不清为什么"的系统,过不了这种审查。
**其二,事故归因。** 一旦出事故,必须能复盘"当时系统为什么这么决策、是哪里出了问题",才能定责、才能改进、才能避免同类事故。
解耦 + 显式决策(ST 图上的 yield/overtake)让这种复盘可行;黑盒端到端出了事,"它当时为什么这么开"可能无法回答,这在法律和工程上都是灾难。
**其三,持续改进的可控性。** 发现一个问题(如某场景下决策不当),可解释的系统能定位到具体模块/规则去修;黑盒只能"重新训练、祈祷修好、还可能引入新问题"——改进不可控。
**这就是为什么** §T4.6 反复强调:可解释性不是"锦上添花",而是安全攸关系统的生命线。
这也是端到端进工业栈必须配安全过滤 + 传统兜底的根本原因——用可解释、可验证的传统方法,为不可解释的黑盒兜底,把可解释性这条生命线守住。
理解了这一点,你就理解了工业对黑盒方法那种"既想用、又必须套上安全外壳"的审慎态度从何而来。

**三个优势的共同点**:它们都不是"算法质量"层面的,而是"工程可控性"层面的——可调试、可迭代、可验证。
**工业系统的成败,一半在算法质量,另一半在工程可控性**;解耦在工程可控性上的优势,正是它在工业界长盛不衰的根本原因。
这也呼应了 §T4.1 那个判断:工业规划栈的难点,一半在算法,一半在架构。

> **本质洞察**:工业坚持解耦,本质揭示了一个常被初学者忽略的真理——"**可控性是和性能同等重要的工程属性,有时甚至更重要**"。
> 学术界评价一个方法,主要看性能指标(轨迹多优、多快);但工业界部署一个方法,还要问:出了问题能不能定位(可调试)?能不能安全地一点点改进(可迭代)?决策能不能解释、能不能通过安全审查(可验证)?一个性能高 10% 但完全黑盒、无法调试和验证的方法,在安全攸关的量产系统里可能根本不能用——因为你无法保证它在长尾场景不出灾难性错误,也无法在出事后复盘归因。
> 解耦"牺牲一点性能(靠迭代弥补)换取彻底的可控性",正是因为工业深知:**一个能被理解、调试、验证的次优系统,胜过一个无法掌控的最优系统**。
> 这个洞察远超规划本身——它解释了为什么工业界对任何黑盒方法(包括端到端)都保持审慎,为什么"可解释 AI""可验证 AI"是安全攸关领域的刚需。
> 理解了可控性的分量,你才能理解工业的许多"保守"选择背后,其实是对系统安全和可维护性的深刻负责。

### 新范式(2025+):端到端 + 安全过滤

但解耦的统治地位,正在被一个新范式挑战:**端到端 + 安全过滤**。
**端到端规划器**:用一个神经网络(扩散模型、Transformer 等)直接从感知输入生成时空联合的轨迹候选——不再有 SL/ST 解耦,不再有显式的 DP/QP,而是数据驱动地"一步到位"产出轨迹。
它的优势是能学到人类驾驶的复杂、流畅的行为(解耦+规则难以手工编码的),尤其在复杂交互场景。
**安全过滤**:端到端的致命弱点是黑盒、不保证安全(可能产出碰撞或违规轨迹)。
所以新范式在端到端后面接一个**安全过滤器**——从网络产出的候选轨迹里,筛掉碰撞的、违反交通规则的、动力学不可行的、不舒适的,只保留安全可行的。
**兜底**:如果所有候选都被滤掉(端到端这一帧全不靠谱),fallback 到传统的 EM/QP 规划器——保证系统始终有一个安全的输出。
整个架构是:

```text
   扩散/Transformer 端到端规划器  → 生成多条时空联合轨迹候选
              │
              ▼
   安全过滤器  → 检查每条候选:碰撞?违反交通规则?动力学可行?舒适?
              │  保留通过的,滤掉不安全的
              ▼
   若有候选通过 → 输出最优的
   若全部被滤掉 → fallback 到传统 EM/QP 规划器(兜底)
```

**这正是 Autoware 2025+ 的方向**:`autoware_diffusion_planner`(扩散端到端)+ 安全过滤思路,让 ROS 2 工业栈较早地正式接纳了扩散规划;Apollo 也有 Plugin 2.0 允许以 Task 形式接入学习模块。

**端到端为什么是现在(2025+)才进工业栈?**
端到端规划的想法不新(感知到控制端到端学习,十年前就有研究),但直到近一两年才真正进入 Apollo/Autoware 这样的量产栈,背后有几个条件最近才成熟。
**条件一:生成模型的进步。** 扩散模型、大规模 Transformer 这几年才成熟到能稳定生成高质量、多模态的轨迹候选——早期的端到端(直接回归一条轨迹)质量和多样性都不够,产出常常平庸或不安全。
扩散模型能产出"多条不同的合理候选",正好配合"安全过滤器选一条"的范式。
**条件二:安全过滤范式的成型。** 让端到端可用的关键不是端到端本身,而是想清楚了"怎么给黑盒套安全外壳"——产出多候选 + 安全过滤 + 传统兜底,这个范式让黑盒的不可靠变得可控。
这个工程范式近年才清晰。
**条件三:数据与算力。** 端到端要海量驾驶数据训练、要车载算力推理。
大规模路测数据的积累、车载 AI 芯片的普及,这两年才到位。
**条件四:成熟的传统栈做兜底。** 端到端进工业栈的前提,是有一个可靠的传统栈(本章的 EM/QP/OBCA)做安全网——正因为 Apollo/Autoware 这些传统栈已经成熟稳定,才"敢"在它们之上叠加端到端、用它们兜底。
**这个时间线的启示**:新技术进入工业,往往不是"技术一出现就替换",而是"等配套条件(模型质量、安全范式、数据算力、兜底基础)都成熟,才以受控的方式引入"。
这也是为什么学好传统方法(本章)不会过时——它们既是当下的主力,又是新范式的安全基础。

**新范式没有推翻解耦,而是"包住"它**:端到端负责"在大多数情况下产出更像人、更流畅的轨迹",解耦的 EM/QP 退居"安全兜底"——把传统方法的可验证性当作端到端黑盒的安全网。
这是工业界引入新技术的典型方式:不激进替换,而是用成熟方法兜底、让新技术在安全边界内发挥。

**安全过滤器具体检查什么。**
这个安全过滤器是新范式的关键一环——它到底检查哪些条件,正是本节练习要你设计的,这里先给出框架。
一个合格的安全过滤器,对端到端产出的每条候选轨迹,至少要查四类条件。
**碰撞检查**:轨迹密采样后,逐点检查车辆占据(考虑车身形状,不只是质点)是否与障碍(静态 + 动态,动态要沿预测轨迹检查时空碰撞)相交。
这是最硬的底线——任何会碰撞的候选直接淘汰。
**交通规则检查**:轨迹是否违反交规——闯红灯、越实线、逆行、超速、不让行人等。
这些规则是显式的、可枚举的,逐条检查。
端到端学到的行为可能"开得像老司机"但偶尔擦边违规,这一关把它拉回合规。
**动力学可行性检查**:轨迹是否满足车辆动力学——速度/加速度/jerk 在界内、曲率不超过最小转弯半径、转向角速率可执行。
端到端可能产出数学上漂亮但车实际跟不了的轨迹,这一关筛掉。
**舒适度检查**:横向/纵向加速度、jerk 是否在舒适范围(过山车式的急加急减虽不违规不碰撞,但乘客难受)。
这一关是软性的,通常用于在多条通过的候选里排序,而非硬性淘汰。
**过滤器的两难**:这里有个关键工程权衡——过滤器太松,放过危险轨迹(失去安全意义);太严,把大部分候选都拒掉(端到端形同虚设,频繁 fallback 到传统兜底,新范式的价值发挥不出来)。
所以安全过滤器的设计核心,是**在"足够安全"和"不过度保守"之间找平衡**:碰撞和硬性交规必须严格(宁可错杀),舒适度等软约束可以宽松(只用于排序)。
这正是练习 1 要你思考的"如果 filter 过于严格会怎样"——答案是端到端被架空、系统退化回纯传统,新技术白引入了。

**开放问题**:如何**验证**端到端规划器的安全性(它是黑盒,传统的形式化验证难以适用)?如何在解耦和联合、传统和端到端之间做**自适应切换**(弱交互用解耦/传统、强交互用联合/端到端)?这些是当前工业和学术共同攻关的前沿。

### ⚠️ 常见陷阱

⚠️ **概念陷阱:以为工业坚持解耦是因为"落后"或"没跟上学术"**
- **错误想法**:学术都在搞联合优化和端到端了,工业还用解耦,是不是落后保守?
- **现象 / 后果**:轻视解耦,盲目推崇联合/端到端,忽略了工业系统的真实约束(实时、可调试、可验证)。
- **根本原因**:工业坚持解耦不是落后,而是因为解耦在**工程可控性**(可调试、可独立迭代、可安全验证)上有联合/端到端给不了的优势——这些对安全攸关的量产系统至关重要。
- **正确做法**:理解工业选择背后的工程理由——质量不是唯一标准,可控性同等重要。
  解耦是"质量够用 + 可控性强"的务实选择,不是技术落后。

🧠 **思维陷阱:以为端到端会立刻全面取代传统规划**
- **错误想法**:端到端/扩散规划这么强,马上就会淘汰 EM/QP 这些传统方法。
- **现象 / 后果**:盲目全面转向端到端,忽视它黑盒、不可验证、不保证安全的致命弱点,埋下安全隐患。
- **根本原因**:端到端的黑盒性使它难以满足安全攸关系统的可验证要求;当前的务实路径是"端到端 + 安全过滤 + 传统兜底",而非全面替换。
- **正确做法**:理解新范式是"端到端 + 传统兜底"的共存,而非替换;传统方法(本章的 EM/QP/OBCA)作为安全网仍不可或缺。
  学好传统方法,才能理解和构建这个安全网。

### 练习

1. **[设计]** 为新范式里的"安全过滤器"设计检查清单:它需要检查哪些条件(碰撞、交通规则、动力学可行、舒适度)?每个条件具体怎么检查?如果 filter 过于严格(拒绝太多候选),会发生什么?如何平衡安全性和通过率?
2. **[分析]** 端到端规划器是黑盒,难以解释"为什么这么开"。
   这在安全验证和事故归因时是大问题。
   对比解耦方法(ST 图上的显式 yield/overtake 决策),说明可解释性在量产自动驾驶中为什么重要,以及端到端如何弥补这个短板。
3. **[思考]** 新范式用"传统 EM/QP 兜底"为端到端兜底。
   这个兜底机制要解决一个难题:如何快速判断"端到端这一帧的输出不靠谱、该切兜底"?请设计这个判断逻辑(提示:安全过滤器全部拒绝、或端到端置信度过低、或输出明显异常)。

### 补充:工业部署的额外考量

本章聚焦架构和算法,但一个真正能上路的规划栈,还有一圈"算法之外"的工程考量——它们不在论文里,却决定系统能否量产。
简要点出,帮你建立完整图景。
**监控与可观测。** 量产规划栈必须能实时监控:每帧规划耗时(是否超预算)、规划成功率、降级触发率、各 Task/层的中间结果。
Apollo 的 Dreamview、Autoware 的 rviz/topic 监控都提供这种可观测性。
没有监控,线上出问题你根本不知道发生了什么。
**日志与可复现。** 每帧规划的输入(感知、预测、定位)和输出(轨迹)都要记录,以便事后复盘——尤其出了异常或事故,要能回放"当时为什么这么规划"。
这也是 §T4.6 强调可解释性的延伸:可解释 + 可复现,才能归因和改进。
**降级与安全兜底。** 规划失败(不收敛、超时、输入异常)时,系统不能直接崩溃或输出乱七八糟的轨迹,必须有分级降级:轻则沿用上一帧、减速;重则触发最小风险机动(MRM,如安全靠边停车)。
这是安全攸关系统的底线。
**版本管理与灰度。** 规划算法的每次更新都要能灰度发布(先小范围、再全量)、能快速回滚。
这就是 Apollo 8.0+ 强调包管理、Autoware 用 pluginlib 配置的部署价值——让"换一个模块/版本"成为可控的运维操作。
**标定与适配。** 同一套规划栈部署到不同车型,车辆参数(轴距、转弯半径、动力学)不同,要能通过配置适配,而非改代码。
这也是模块化 + 配置驱动架构的实际收益。
这些考量的共同点:它们都是为了让规划栈在**真实、长期、大规模**的运营中可靠、可维护、可演进——这正是"工程可控性"(§T4.6)在部署层面的体现。
学算法和架构是基础,但要让系统真正落地,这一圈工程实践同样不可或缺。
理解它们,你对"工业规划栈"的认识才完整:它不只是 EM/QP/OBCA 这些算法,也不只是 Scenario/Task 这些架构,而是算法 + 架构 + 这一整圈工程实践的总和。

---

## 本章常见误解汇总

| 误解 | 实情 | 出处 |
|------|------|------|
| Scenario/Stage/Task 三层按算法复杂度分 | 按职责粒度分:情况→阶段→原子操作;Decider 和 Optimizer 同为 Task | §T4.1 |
| 让 Task 互相直接调用更简单 | 那把调度硬编码进 Task、强耦合;应由 Stage 统一调度 | §T4.1 |
| EM Planner 是真正的时空联合优化 | 是半耦合:每步仍是 path/speed 独立凸子问题,靠迭代协调 | §T4.2 |
| EM 迭代越多轮越好 | 有实时预算,固定少数几轮;质量提升边际递减但时间是实打实的 | §T4.2 |
| 交替迭代双方都用上一轮旧值 | 应 Gauss-Seidel:后算的用先算的最新值,收敛更快 | §T4.2 |
| Open Space 也是 path-speed 解耦 | 泊车无参考线、要前进后退、需硬约束,必须 OBCA 联合 | §T4.3 |
| 泊车用普通栅格 A\* 就行 | 车是非完整系统,必须 Hybrid A\*(运动学扩展) | §T4.3 |
| 联合优化更先进,应全用联合 | 按场景选:on-lane 解耦(快)、Open Space 联合(精) | §T4.3 |
| 用 if-else 硬编码模块加载 | 应工厂按名字创建 + 配置驱动,核心代码不动 | §T4.4 |
| Autoware 的 PathOptimizer 也优化速度 | 只优化路径,速度零阶保持;速度由后面两层单独处理 | §T4.4 |
| 对比两框架是为选出"更好的" | 是为理解取舍空间,据场景选择;两者各有适用 | §T4.5 |
| 工业坚持解耦是落后保守 | 是为工程可控性(可调试/可迭代/可验证)的务实选择 | §T4.6 |
| 端到端会立刻全面取代传统规划 | 当前是"端到端+安全过滤+传统兜底"共存,非替换 | §T4.6 |
| EM 的 "EM" 就是机器学习的 EM 算法 | 借了交替优化思想,但 E-step 是投影、M-step 是优化,非概率 EM | §T4.2 |
| 三层架构和六层流水线哪个层多哪个好 | 层数匹配设计哲学(半耦合/严格解耦),非越多越好 | §T4.4 |
| 工业能直接调 OBCA 库,不用自己改 | 论文算法到量产要大量工程加固(TDR-OBCA 的信赖域/松弛) | §T4.3 |

---

## 本章小结

本章是 T 线的工业落地篇——讲 Apollo 和 Autoware 两个量产规划栈如何把 T1-T3 的算法组织成可部署、可维护、可验证的系统。
**Apollo** 用 Scenario/Stage/Task 三层架构(§T4.1)组织规划,用 EM 半耦合迭代(§T4.2,E-step 投影、M-step DP+QP,path↔speed 交替)做 on-lane 规划,只在 Open Space 泊车(§T4.3)用 Hybrid A\*+OBCA 的真正时空联合。
**Autoware Universe** 走严格分层流水线(§T4.4,Mission→BPP→BVP→PathOpt→MotionVel→VelSmoother),用 pluginlib 热插拔模块。
两者对比(§T4.5)体现两种工业哲学:Apollo 的务实"准联合" vs Autoware 的极致"清晰解耦"。
而工业坚持解耦(§T4.6)的根本是工程可控性(可调试/可独立迭代/可安全验证),2025+ 的端到端+安全过滤新范式正在"包住"而非推翻解耦。
贯穿全章的主线:**工业规划栈的难点一半在算法、一半在架构;好的架构让 T1-T3 的算法能被组织、维护、扩展、安全验证**。

### 术语速查

| 术语 | 英文/全称 | 一句话定义 |
|------|----------|-----------|
| 场景/阶段/任务 | Scenario / Stage / Task | Apollo 三层:驾驶情况 / 顺序阶段 / 原子操作 |
| 决策器/优化器 | Decider / Optimizer | Apollo Task 两类:做离散决策 / 做连续优化 |
| EM 规划器 | EM Motion Planner | Apollo on-lane:path↔speed 交替迭代的半耦合规划 |
| E-step / M-step | Expectation / Maximization step | 投影步(障碍投到 SL/ST 图)/ 优化步(DP+QP) |
| 分段 jerk QP | piecewise-jerk QP | 以 jerk 平方为代价的路径/速度二次规划,OSQP 求解 |
| 开放空间 | Open Space | 泊车/掉头,用 Hybrid A\*+OBCA 真联合 |
| 混合 A\* | Hybrid A\* | 在连续状态(x,y,θ)按车辆运动学扩展的 A\* |
| 行为路径规划器 | BehaviorPathPlanner (BPP) | Autoware:生成要走的路径(变道/避障) |
| 行为速度规划器 | BehaviorVelocityPlanner (BVP) | Autoware:按交通规则在路径上插停车/减速点 |
| 模型预测轨迹 | Model Predictive Trajectory (MPT) | Autoware 路径优化:Frenet 横向 QP |
| 插件库 | pluginlib | ROS 插件机制,运行时动态加载共享库模块 |
| 半耦合 | semi-coupled | 形式解耦但通过迭代引入协调(EM) |
| CyberRT | Cyber Real-Time | Apollo 自研中间件,共享内存 + 确定性调度 |
| TDR-OBCA | Trust-region & Dual-relaxation OBCA | Apollo 对 OBCA 的工程加固(信赖域+对偶松弛) |

### 知识点总表

| 知识点 | 核心结论 | 关联 |
|--------|---------|------|
| 三层架构 | Scenario/Stage/Task 把巨型函数拆成可插拔的树 | §T4.1 |
| EM 半耦合 | 解耦的高效 + 迭代的协调,逼近联合质量 | §T4.2 |
| DP+QP 分工 | DP 定 homotopy/凸域,QP 在凸域求精 | §T4.2 |
| Open Space 联合 | 主链路唯一的真联合,Hybrid A\*+OBCA | §T4.3 |
| Hybrid A\* | 运动学扩展保证路径车能开 | §T4.3 |
| 严格分层 | Autoware 六层单向流水线,清晰可验证 | §T4.4 |
| pluginlib | 配置驱动的模块热插拔 | §T4.4 |
| 两种哲学 | 准联合(Apollo)vs 清晰解耦(Autoware) | §T4.5 |
| 解耦的根本 | 工程可控性:可调试/可迭代/可验证 | §T4.6 |
| 新范式 | 端到端 + 安全过滤 + 传统兜底,共存非替换 | §T4.6 |
| 计算预算 | on-lane 紧(高速动态)→解耦,Open Space 松(低速静态)→联合 | §T4.3 |
| 论文到量产 | 主算法来自论文,关键在工程加固(稳定/快/接口/降级) | §T4.3 |
| 工程实践 | 监控/日志/降级/版本管理/标定——算法之外的可控性 | §T4.6 |

---

## 累积项目:给规划器套上工业架构

延续 T1-T3 的累积项目(T 线规划器),本章把它从"一组算法"升级为"一个有工业架构的规划系统"。

### 本章新增模块

```cpp
// T 线累积项目:工业架构外壳(承接 T1-T3 的算法,本章用架构组织它们)
// 把 T1-T3 的算法(QP/搜索/OBCA)封装成 Task,用 Scenario/Stage 组织

// 1. 所有算法封装成统一的 Task 接口(Apollo 风格)
class PlanningTask {
 public:
  virtual ~PlanningTask() = default;
  virtual bool Process(PlanningContext& ctx) = 0;  // ctx 承载 T1-T3 的数据
  virtual std::string Name() const = 0;
};

// 2. T1-T3 的算法各自封装成 Task
//    PathBoundsDecider      → 算 SL 边界(T1)
//    PiecewiseJerkPathTask  → SL 路径 QP(T1/T3)
//    STBoundsDecider        → 算 ST 边界 + yield/overtake(T1/T2)
//    PiecewiseJerkSpeedTask → ST 速度 QP(T1/T3)
//    OpenSpaceTask          → Hybrid A* + OBCA(T3,泊车场景)

// 3. Scenario 组织 Task 链 + pluginlib 风格的模块热插拔
class Scenario {
 public:
  bool Process(PlanningContext& ctx) {
    for (auto& task : task_chain_)      // 按配置顺序执行 Task 链
      if (!task->Process(ctx)) return Fallback(ctx);  // 失败则降级兜底
    return true;
  }
 private:
  std::vector<std::unique_ptr<PlanningTask>> task_chain_;  // 配置驱动
  bool Fallback(PlanningContext&);      // 兜底:减速/沿用上一帧
};
// 关键:T1-T3 是"算法",本章把它们装进"架构"——
//   统一 Task 接口 + Scenario 组织 + 配置驱动 + 失败兜底 = 工业规划系统
```

### 验收标准

1. **架构正确**:T1-T3 的算法都封装成统一 Task 接口,由 Scenario/Stage 按配置组织,数据通过共享 context 在 Task 间传递。
2. **可扩展**:加新场景 = 加 Scenario + 配 Task 链,不动已有代码;加新算法 = 写新 Task 注册进去。
3. **场景切换**:实现一个简单的 ScenarioManager,能根据条件在 LaneFollow 和 Parking(Open Space)间切换。
4. **降级兜底**:任一 Task 失败时,Scenario 降级到安全兜底(减速/沿用上一帧),不输出劣质或无效轨迹。
5. **可观测**:每个 Task 的输入输出可日志化、可视化(SL 图/ST 图分别可视化),便于调试——这是工业可控性的体现。

---

## 评价指标

工业规划栈的质量,除了 T1-T3 的轨迹质量指标,还要看**工程指标**。

| 维度 | 指标 | 含义 |
|------|------|------|
| 实时性 | 端到端规划延迟、各 Task 耗时 | 整条 Task 链是否在规划周期(如 100 ms)内;哪个 Task 是瓶颈 |
| 稳定性 | 规划成功率、降级触发率 | 多少帧成功输出有效轨迹;多少帧触发兜底 |
| 可调试性 | 决策可追溯性、中间结果可视化 | yield/overtake 等决策能否追溯;SL/ST 图能否可视化 |
| 可扩展性 | 加新场景/模块的代码改动量 | 加功能要改多少核心代码(越少越好) |
| 一致性 | 跨帧轨迹连贯性 | 相邻帧的规划是否连贯(无突变/抖动) |

**工程指标和算法指标同等重要**:一个轨迹质量极高但延迟超时、或无法调试、或加个功能要重写一半的规划栈,在工业上是失败的。
本章讲的架构,正是为了在保证算法质量的同时,把这些工程指标做好。

---

## FAQ

**Q:学完 T4,我能直接上手改 Apollo/Autoware 的代码吗?**
能读懂架构、能在其上做有针对性的修改(如调一个 Optimizer 的权重、加一个 Decider、配置模块),但要从零实现整个栈不现实(几十万行)。
T4 的目标是让你**读懂架构、理解取舍、能二次开发或借鉴设计**。
建议配合实际 clone 仓库、搭仿真环境、跟着官方文档动手——读代码和搭仿真是把本章知识坐实的必经之路。

**Q:Apollo 和 Autoware,初学者该先学哪个?**
看你的方向和基础。
想进国内自动驾驶公司、中文资料更顺手 → Apollo(社区活跃、中文文档多、功能完整含 Open Space)。
想做开源贡献、ROS 2 生态、英文文档无障碍 → Autoware(架构清晰、文档规范、pluginlib 设计优雅)。
两者架构思想相通(模块化、可扩展),学透一个再看另一个会很快。

**Q:为什么不直接学最新的端到端,还要学这些"传统"框架?**
三个理由。
其一,端到端目前还需要传统方法做安全兜底(§T4.6),不懂传统就构建不了这个安全网。
其二,端到端的"安全过滤器"要检查碰撞、动力学、交通规则——这些检查逻辑正是传统方法的知识。
其三,绝大多数量产车现在跑的仍是传统或半传统栈,工程岗位需要这些技能。
端到端是未来方向,但传统框架是当下的基础设施,且短期内不会消失。

**Q:EM Planner 的 "EM" 和机器学习的 EM 算法是一回事吗?**
借用了思想,不是一回事。
机器学习的 EM(期望最大化)是"E 步算期望、M 步最大化",用于含隐变量的概率模型参数估计。
Apollo EM Planner 借的是它"固定一部分、优化另一部分,交替迭代"的框架思想:把 path 和 speed 看成两组交替优化的变量。
两者都是"交替优化"的体现,但 Apollo 的 E-step 是"投影"(把障碍投到 SL/ST 图)、M-step 是"优化"(DP+QP),与概率意义的 EM 不同。

**Q:CyberRT 和 ROS 2 我该用哪个?**
看场景。
CyberRT 是 Apollo 为自动驾驶定制的中间件,共享内存、低延迟、为高频确定性通信优化——适合对延迟和确定性要求极高的量产自动驾驶。
ROS 2 是通用机器人中间件,生态丰富、标准化、社区大——适合通用机器人、研究、需要丰富现成功能包的场景。
自动驾驶量产偏 CyberRT(或自研),通用机器人和研究偏 ROS 2。

**Q:这章和 T1-T3、和后续 Part 什么关系?**
T1-T3 给算法(解耦、走廊、联合优化),T4 给架构(怎么把算法组织成系统)。
T4 是 T 线"从算法到系统"的关键一跃。
往后:Part-U(不确定性)、Part-G(博弈)会讲这些工业栈还没很好解决的问题(预测不确定、强交互博弈),而它们的方法最终也要装进 T4 这样的架构里才能落地。
所以 T4 的架构知识是后续所有"把新方法工程化"的基础。

**Q:我在 T1-T3 学的那些算法(CILQR、TEB、MINCO、CPC),在 Apollo/Autoware 里都能找到吗?**
部分能,部分不能——这正反映了工业的选择。
能找到的:T1 的 path-speed 解耦(两个框架的骨架)、T1/T3 的 piecewise-jerk QP(Apollo 的 Path/SpeedOptimizer、Autoware 的 MPT/VelocitySmoother)、T3 的 OBCA(Apollo Open Space)、T2 的搜索思想(DP-ST、Hybrid A\*)。
不太能直接找到的:CILQR(自驾界用,但 Apollo/Autoware 主链路没用,它更多在其他研究型栈或学术实现里)、MINCO/CPC(主要在无人机方向,不在这两个车用框架里)。
这说明:工业框架不是"把所有先进算法都塞进去",而是选**经过验证、够用、可控**的少数几个(主要是解耦 + QP),把它们工程化做扎实。
学 T1-T3 是为了懂原理和选择空间,不是每个都要在量产栈里见到。

**Q:Apollo Open Space 既然用 OBCA,为什么不直接调一个现成的 OBCA 库,还要自己实现 TDR-OBCA?**
因为学术论文的算法和工业可部署的模块之间,有相当大的工程距离。
原始 OBCA(Berkeley 的实现)是研究原型,在某些场景下求解可能不够稳定、不够快。
Apollo 做 TDR-OBCA(信赖域 + 对偶松弛)是为了**改善求解的稳定性和速度**,让它能在量产车上可靠运行——这包括处理数值边界情况、保证求解时间可控、和上游 Hybrid A\* 及下游控制的接口适配等。
这是工业落地的常态:很少能直接"调库",更多是拿论文的核心思想,做大量工程加固(稳定性、性能、接口、降级)才能上车。
这也是为什么读 Apollo 的 Open Space 代码,你会看到比论文复杂得多的工程细节——那些细节正是"可部署"和"能跑通 demo"的区别。

---

## 延伸阅读

**核心论文**:Fan et al., *Baidu Apollo EM Motion Planner*, arXiv 1807.08048 (2018)——Apollo on-lane 规划的奠基论文,EM 迭代 + DP+QP 的来源;Zhang, Liniger, Borrelli, *Optimization-Based Collision Avoidance*, T-CST 2020 29(3):972-983(OBCA,Apollo Open Space 的算法核心);Zhang et al., *Autonomous Parking Using Optimization-Based Collision Avoidance*, CDC 2018(H-OBCA,泊车两段式)。

**官方资源**:Apollo 仓库 `ApolloAuto/apollo`(Apache-2.0,中文文档 + Dreamview 仿真)、其 `modules/planning` 是研究工业规划架构最好的公开代码;Autoware Universe `autowarefoundation/autoware_universe`(Apache-2.0)及其 mkdocs 文档(规范完整,各 planning 模块都有详细说明)、AWSIM/Planning Simulator 仿真。

**前沿**:`autoware_diffusion_planner`(2025,Autoware 的扩散端到端规划器);Tier IV 在 IEEE IV 2025 关于"面向 Autoware 的模块化端到端扩散规划器"的报告;扩散规划 + 安全过滤的相关工作(如 RSTP 等)。

**与本书其他部分**:T1-T3 是本章算法基础(解耦/走廊/QP/OBCA);Part-U、Part-G 处理本章工业栈尚未很好解决的不确定性与博弈;ROS 2 相关章节是理解 Autoware 工程化的基础。

---

## 本章与后续的关系

| 后续主题 | 本章提供的基础 | 衔接点 |
|---------|--------------|--------|
| Part-U 不确定性规划 | 工业栈的确定性规划骨架 | 预测不确定性如何嵌入 EM/QP 框架(机会约束等) |
| Part-G 博弈规划 | 工业栈对交互的现有处理(yield/overtake) | 强交互场景:从规则化决策到博弈均衡 |
| 多智能体协调 | 单车规划栈 | 多车各自跑规划栈 + 协调 |
| 学习驱动前沿 | 端到端 + 安全过滤的架构 | 扩散/Transformer 规划器如何接入工业栈、被安全兜底 |
| 系统集成与部署 | 中间件(CyberRT/ROS 2)、模块化架构 | 完整自动驾驶系统的集成 |

---

## 🔧 故障排查手册

| 现象 | 可能原因 | 排查 / 修复 |
|------|---------|-----------|
| 加新场景/模块要改一大片核心代码 | Task/模块的调度被硬编码(if-else),没用工厂+配置 | 改用工厂按名字创建 + 配置驱动(§T4.1、§T4.4 编程陷阱) |
| 某个 Optimizer Task 报错"缺少输入" | Task 链顺序错,依赖的前置 Decider 没先执行 | 检查 Task 链配置顺序,前置 Decider 必须在前(§T4.1 练习) |
| EM 迭代收敛慢或在两个解间震荡 | 交替迭代用了 Jacobi(都用旧值)而非 Gauss-Seidel | 让 speed 立即用刚算的 path(§T4.2 编程陷阱);或耦合太强,检查迭代映射压缩性 |
| 规划延迟超出周期预算 | EM 迭代轮数过多,或某 Task 太慢 | 固定少数迭代轮数;profile 各 Task 耗时定位瓶颈(§T4.2 思维陷阱) |
| 泊车轨迹有尖角、车开不出来 | 用了普通栅格 A\* 而非 Hybrid A\* | 改用 Hybrid A\*,按车辆运动学扩展(§T4.3 编程陷阱) |
| 泊车 OBCA 优化不收敛 | 没有 Hybrid A\* warm-start,或初值差 | 先用 Hybrid A\* 出可行粗解定 homotopy 再 warm-start OBCA(§T4.3、T3 §T3.3) |
| Autoware 路径优化后速度不对 | 误以为 PathOptimizer 优化速度(它零阶保持) | 速度由 MotionVelocityPlanner/VelocitySmoother 处理(§T4.4 概念陷阱) |
| 端到端规划器偶尔输出危险轨迹 | 缺少安全过滤 / 兜底机制 | 加安全过滤器筛候选 + 传统 EM/QP 兜底(§T4.6) |

---

## API 速查表

> 以教学示意接口为主,实际 Apollo/Autoware 的命名与签名以各自代码为准。

| 功能 | 示意接口 | 说明 |
|------|---------|------|
| Apollo Task 基类 | `Task::Process(ReferenceLineInfo&)` | 所有 Decider/Optimizer 继承,只做一件事 |
| Apollo 路径 QP | `PiecewiseJerkPathOptimizer` | SL 路径二次规划,OSQP |
| Apollo 速度 QP | `PiecewiseJerkSpeedOptimizer` | ST 速度二次规划,OSQP |
| Apollo Open Space | `OpenSpaceTrajectoryOptimizer` | Hybrid A\* + OBCA(DistanceApproachProblem) |
| Apollo 场景管理 | `ScenarioManager` | 场景切换 FSM |
| Autoware 行为路径 | `BehaviorPathPlanner` | 生成路径,pluginlib 加载行为模块 |
| Autoware 行为速度 | `BehaviorVelocityPlanner` | 交通规则插速度约束,pluginlib |
| Autoware 路径优化 | `PathOptimizer` (MPT) | Frenet 横向 QP(速度零阶保持) |
| Autoware 速度平滑 | `VelocitySmoother` | jerk-limited 速度 QP,OSQP |
| 插件注册 | `PLUGINLIB_EXPORT_CLASS(模块, 基类)` | 把模块注册为可动态加载的插件 |

---

## 研究实践建议

按目标分三层。

**入门(读懂架构、能跑仿真)。**
先选一个框架(Apollo 或 Autoware)深入。
Clone 仓库、按官方文档搭仿真环境(Apollo Dreamview / Autoware Planning Simulator),跑通默认场景,用可视化工具观察规划过程(Apollo 看 SL/ST 图、Autoware 看 topic 输出)。
然后对照本章的架构图读代码:找到 Scenario/Task(Apollo)或六层流水线(Autoware)的对应实现,把"架构图"和"真实代码"对上号。
这一阶段别想读懂全部,抓住主干流程(一帧规划怎么从输入到输出)即可。

**进阶(能二次开发、能对比)。**
在仿真里做有针对性的修改:调一个 Optimizer 的代价权重看效果(如增大 jerk 惩罚看速度是否更平滑)、加一个简单的 Decider 或 BVP 模块、配置增删模块。
这一阶段重点练两个能力:一是**读懂模块间的数据流**(数据如何在 Task 间/层间传递、积累),二是**理解每个设计选择的取舍**(为什么这样拆、为什么这个顺序)。
建议把 Apollo 和 Autoware 都浏览一遍,对照着理解两种架构哲学(§T4.5)。

**研究(能改进架构、能融合新范式)。**
盯住 §T4.6 的开放问题:端到端规划器的安全验证、解耦与联合/传统与端到端的自适应切换。
前沿方向是"端到端 + 安全过滤 + 传统兜底"的融合架构——这需要同时懂传统方法(本章 + T1-T3)和学习方法(扩散/Transformer)。
建议从复现 `autoware_diffusion_planner` 这类工作起步,理解端到端如何接入工业栈、如何被安全兜底;在复现中找到现有方案的局限(如安全过滤的保守性、兜底切换的及时性),那往往是研究起点。
架构层面的创新,要建立在对现有工业架构(本章)的深刻理解之上。

---

## 版本信息速查

| 项目 | 版本 / 状态(截至本章撰写) | 备注 |
|------|--------------------------|------|
| Apollo | 最新 11.0(8.0 包管理 / 9.0 包管理 2.0 / 10.0 大规模场景 / 11.0 端到端运营闭环) | Apache-2.0,CyberRT,planning ~10 万行 C++ |
| Autoware Universe | 持续迭代,ROS 2 | Apache-2.0,严格分层 + pluginlib |
| autoware_diffusion_planner | 2025 合并入 universe | 扩散端到端,ROS 2 工业栈 |
| EM Planner 论文 | arXiv 1807.08048(2018) | Apollo on-lane 规划奠基 |
| OBCA / H-OBCA | T-CST 2020 / CDC 2018 | Apollo Open Space 算法核心 |
| OSQP / Ipopt / CasADi | 稳定 | 各 QP/NLP 的求解底座 |

> 版本随时间变化,落地前请以各项目官方仓库为准。

---

## 前沿工作与开放问题

**端到端进入工业栈。** `autoware_diffusion_planner`(2025 合并入 Autoware Universe)是 ROS 2 工业栈较早正式接纳扩散规划的代表;Apollo 的 Plugin 2.0 允许以 Task 形式接入学习模块。
趋势是端到端不再只是研究,而开始进入量产栈——但都配着安全过滤和传统兜底,而非裸用。

**安全过滤与兜底。** 端到端的黑盒性催生了"安全过滤 + 传统兜底"这一配套范式:从神经网络产出的候选里筛安全可行的,全被筛掉则 fallback 到 EM/QP。
如何设计高效又不过度保守的安全过滤器、如何及时判断该切兜底,是工程攻关重点。

**自适应架构切换。** 一个有吸引力的方向:在解耦/联合、传统/端到端之间**自适应切换**——弱交互场景用解耦/传统(快、可验证),强交互场景切联合/端到端(更像人、更灵活)。
如何鲁棒地判断"当前是弱交互还是强交互"、如何平滑切换,是开放问题。

**开放问题。** 如何**形式化验证**端到端规划器的安全性(黑盒难以用传统方法验证)?如何统一传统规划的可解释性和端到端的灵活性?如何让工业栈在不牺牲可控性的前提下,吸收端到端的能力?这些是工业和学术共同攻关的前沿,也是把本章内容往研究推进的入口。

**把这些前沿串起来看。** 它们围绕同一个张力:**端到端的能力 vs 工业的可控性要求**。
端到端(扩散/Transformer)能学到传统方法难以手工编码的、流畅拟人的驾驶行为,这是它的吸引力;但它黑盒、不保证安全、难以验证,这与工业对可控性的刚需(§T4.6)直接冲突。
当前所有前沿方向,本质都是在调和这个张力:**安全过滤**是给黑盒套一层可验证的外壳、**传统兜底**是用可控的旧方法接住黑盒的失误、**自适应切换**是让黑盒只在它擅长且安全的场景上场、**形式化验证**是试图直接攻克黑盒的可验证性。
理解了这个统一张力,你就能判断任何一篇新论文/新方案"在解决张力的哪一面、用什么招式"——这比追逐具体技术名词更重要。
而能调和好这个张力的方案,既要懂端到端(学习方法),也要懂传统规划和工业架构(本章 + T1-T3)——这正是为什么本章的知识是拥抱未来新范式的、而非会被淘汰的基础。

---

## 拓展练习(综合 / 项目级)

1. **[仿真 + 精读]** 搭建 Apollo Dreamview 仿真,在某地图上跑 LaneFollow 场景,用 Planning 面板观察 SL 图和 ST 图的实时更新;手动加动态障碍,观察 yield/overtake 决策如何在 ST 图上体现。
   再精读 LaneFollow 的 Task 链配置 + 对应代码,画出数据如何在 ReferenceLineInfo 上从 PathBoundsDecider 逐步积累到 PiecewiseJerkSpeedOptimizer。
2. **[仿真 + 精读]** 用 Autoware Planning Simulator 跑默认场景,观察 BPP→BVP→PathOpt→VelSmoother 各模块的 topic 输出,用 rqt_graph 画规划模块的 topic 依赖图。
   再精读 `autoware_path_optimizer` 的 MPT 实现,标注 Frenet 横向状态定义、自行车模型线性化、OSQP QP 构建。
3. **[实现 + 实验]** 在 Apollo 的 `PiecewiseJerkSpeedOptimizer` 中修改代价权重(增大 jerk 惩罚),对比修改前后的速度 profile 光滑度,理解代价权重如何影响驾驶舒适度。
4. **[设计]** 假设你要设计一个新的 L4 规划栈。
   综合本章,回答:(a) 用 Scenario/Stage/Task 还是严格分层流水线?(b) on-lane 用 EM 半耦合还是严格解耦?(c) 模块怎么做热插拔?(d) 如何为未来的端到端预留接口和安全兜底?给出你的架构设计和理由。
5. **[思考·跨 T 线]** 把 T1-T4 串起来:T1 给解耦+Frenet/ST、T2 给走廊+搜索、T3 给联合优化、T4 给工业架构。
   请说明一帧 on-lane 规划(如跟车带变道)在 Apollo 里如何从感知输入走到轨迹输出,沿途用到 T1-T4 的哪些知识(参考线→SL/ST→DP+QP→Task 链→输出),以及哪里体现了"DP 定 homotopy + QP 求精"这条贯穿主线。
6. **[源码精读·对比数据流]** 分别在 Apollo 和 Autoware 里追踪一帧 on-lane 规划的数据流,画两张数据流图:Apollo 的(数据如何在 ReferenceLineInfo 上被各 Task 逐步填充)和 Autoware 的(数据如何经 route→PathWithLaneId→Trajectory 逐层变形)。
   对比两张图,指出:(a) 哪里体现 Apollo 的"共享容器接力" vs Autoware 的"单向流水线变形";(b) 速度信息在两个框架里分别是何时、如何确定的(Apollo 的 EM speed QP vs Autoware 的 MotionVelocity+VelocitySmoother);(c) 两者各自的优劣。
7. **[架构改造·综合]** 假设你要给 Autoware 加上 Apollo 式的"准联合"能力(让路径和速度能迭代协调,而非严格单向)。
   请设计:(a) 改造方案——在哪两层之间加迭代反馈?反馈什么信息?(b) 这会破坏 Autoware 的什么优点(提示:严格单向带来的清晰性、可验证性)?(c) 权衡——这个改造值得吗?什么场景下值得?这道题考验你对两种架构取舍的深层理解,以及"架构改造必有代价"的工程意识。

---

## 速记卡:一句话记住每个要点

复习时用这几张卡片快速唤起本章核心。

**Apollo 三层架构——"情况→阶段→原子操作"。**
记忆锚点:Scenario/Stage/Task。
把巨型 plan() 拆成可插拔的树:Scenario 管"什么情况"、Stage 管"分几步"、Task 管"每步一个原子操作"(Decider 决策 / Optimizer 优化)。
一句话:**用分层把复杂场景拆成可组合、可热插拔的积木。**

**Apollo EM 迭代——"path 和 speed 轮流商量"。**
记忆锚点:半耦合。
E-step 把障碍投到 SL/ST 图、M-step 用 DP+QP 优化,path 和 speed 交替迭代、互相参考——形式解耦(快)、迭代协调(逼近联合)。
一句话:**不完全联合,而让路径和速度轮流优化、互相迁就,在实时预算内逼近联合质量。**

**Apollo Open Space——"泊车才用真联合"。**
记忆锚点:Hybrid A\*+OBCA。
主链路唯一不解耦的地方:泊车没参考线、要前进后退、需硬约束,用 Hybrid A\* 搜粗解(运动学可行)+ OBCA 精修(精确避障)。
一句话:**on-lane 解耦图快,泊车联合图精——因地制宜。**

**Autoware 流水线——"一条单向的装配线"。**
记忆锚点:严格分层 + pluginlib。
六层单向流动(Mission→BPP→BVP→PathOpt→MotionVel→VelSmoother),每层只做一件事、只往后传;模块用 pluginlib 配置热插拔。
一句话:**把规划拆成职责单一、单向流动、可独立验证的装配线。**

**为什么坚持解耦——"工程可控性"。**
记忆锚点:可调试/可迭代/可验证。
解耦让 SL/ST 分别可视化调参、模块可独立 A/B 测试、决策可追溯可解释——这些工程可控性,是安全攸关量产系统的生命线,联合/端到端给不了。
一句话:**质量不是唯一标准,可控性同等重要;解耦是务实之选。**

把五张卡片连起来:Apollo 用"三层架构 + EM 半耦合 + Open Space 真联合"务实地逼近质量,Autoware 用"严格分层 + pluginlib"追求极致清晰,两者都把解耦当骨架——因为工业系统的成败,一半在算法,一半在工程可控性。
记住这两套架构和那个"算法+架构"的判断,就记住了这一章。

---

## 三个核心 takeaway

如果这一章只能记住三件事,应该是这三个——它们比任何框架的目录结构都更持久、更通用。
**第一,工业规划栈 = 算法 + 架构,缺一不可。** T1-T3 给你算法,T4 给你架构。
一个能产出好轨迹但无法组织、维护、扩展、验证的系统,在工业上是失败的;反之,空有漂亮架构但算法质量差,也跑不出好车。
Apollo 和 Autoware 的价值,正在于展示了"怎么把好算法装进好架构"。
读任何工业系统,都要同时看这两层。
**第二,没有最好的方法/架构,只有最适合场景的选择。** Apollo on-lane 用解耦、Open Space 用联合;Apollo 用半耦合、Autoware 用严格解耦;Apollo 自研中间件、Autoware 用标准 ROS 2——这些都不是"谁对谁错",而是按场景的实时预算、耦合强度、团队形态、目标人群做的取舍。
工程成熟度的标志,是理解取舍空间、按需选择,而非迷信某个"最优方法"。
**第三,工程可控性和算法质量同等重要,有时更重要。** 工业坚持解耦,不是技术落后,而是因为可调试、可独立迭代、可安全验证这些工程可控性,对安全攸关的量产系统是生命线。
这个洞察解释了工业对黑盒方法(包括端到端)的审慎,也解释了为什么"端到端 + 安全过滤 + 传统兜底"是当前的务实路径。
理解了可控性的分量,你才真正理解了工业思维和学术思维的根本差异。
把这三点内化,你看任何规划系统(乃至任何复杂工程系统)的眼光都会不一样——不再是"它用了什么炫酷算法",而是"它在算法和架构上做了什么取舍、为什么、代价是什么"。
这种眼光,是 T4 最想留给你的。

---

## 结语:你现在站在哪里

本章带你从"算法"走到了"系统"。
T1-T3 让你理解了时空规划的算法内核(解耦、走廊、联合优化),而本章——**Apollo 和 Autoware 如何把这些算法组织成能在真车上跑、能被团队维护、能通过安全验证的系统**——让你看清了工业规划栈的另一半:架构。

你现在手里有两套经过大规模量产检验的架构范本:Apollo 的"三层架构 + EM 半耦合 + Open Space 真联合"、Autoware 的"严格分层流水线 + pluginlib 热插拔"。
更重要的是,你理解了它们背后的取舍——Apollo 的务实准联合 vs Autoware 的极致清晰解耦,以及二者共同坚持解耦的根本原因:**工程可控性**(可调试、可独立迭代、可安全验证)。
你也看清了 2025+ 的新范式(端到端 + 安全过滤 + 传统兜底)如何"包住"而非推翻这套成熟架构。

但你也该看清这套工业架构的**边界**。
它把 T1-T3 的算法组织得很好,却仍站在两个假设上:它处理的交互大多是**规则化**的(ST 图上的 yield/overtake 是预设规则,不是真正的博弈),它假设障碍的**预测是给定的**(没有显式建模预测的不确定性)。
规则化决策应对不了强交互(对方会因你的动作改变意图),给定预测应对不了真实世界的不确定。
这两个边界,正是 Part-U(不确定性)和 Part-G(博弈)要突破的——而它们突破后的新方法,最终也要装回 T4 这样的工业架构里,才能从论文走向量产。

T1-T4 为你建立了"算法 + 架构"的完整自动驾驶规划能力:你既懂怎么算一条好轨迹,也懂怎么把算法组织成一个可部署、可维护、可验证的系统。
带着这套能力和它的边界,你已经准备好去直面真实世界更难的挑战——不确定与博弈。
