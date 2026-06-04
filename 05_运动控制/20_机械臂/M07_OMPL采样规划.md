> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M07 OMPL采样运动规划深度——从构型空间理论到工业调参

> **性质**: ❌ 纯机械臂 | **时长**: 1.5 周 (15-20 学时)
> **前置**: M03(IK求解器), M04(碰撞检测 FCL/Coal); C++ 基础（继承多态、设计模式相关章节）
> **下游**: M08(轨迹优化规划器) → M09(GPU加速规划) → M10(时间参数化) → M14(MoveIt2集成)

---

## M07.0 前置自测

开始本章之前，请独立回答以下 5 题。若答不出 3 题以上，建议先回顾 M03/M04 和相关基础章节。

| # | 问题 | 期望答案要点 |
|---|------|-------------|
| 1 | 给定 7-DOF 机械臂，其关节角 $q \in \mathbb{R}^7$。$q$ 所在的空间称为什么？该空间与工作空间 $\mathcal{W} \subseteq SE(3)$ 有何关系？ | 构型空间(C-space) $\mathcal{C} = \mathbb{R}^7$（受关节限位约束时为超矩形子集）。FK 是 $\mathcal{C} \to \mathcal{W}$ 的映射；工作空间中的"近"不等于构型空间中的"近" |
| 2 | 什么是概率完备性(Probabilistic Completeness)？与确定性完备(Complete)有何区别？ | 概率完备：若解存在，则随着采样数 $n \to \infty$，找到解的概率趋于 1。确定性完备：有限步内必然找到解或报告无解。RRT/PRM 是概率完备但不确定性完备 |
| 3 | C++ 中 `virtual void solve() = 0;` 声明的函数叫什么？其派生类必须做什么？ | 纯虚函数。派生类必须 `override` 实现，否则自身也是抽象类无法实例化 |
| 4 | FCL 的 `BroadPhaseCollisionManager` 和 `NarrowPhaseCollision` 分别做什么？哪个更耗时？ | BroadPhase(宽相)用 AABB/SAP 快速排除不可能碰撞的物体对；NarrowPhase(窄相)用 GJK/EPA 精确计算具体碰撞。窄相更耗时（10-100x） |
| 5 | RAII 的核心思想是什么？`std::unique_ptr<T>` 如何体现这个思想？ | Resource Acquisition Is Initialization——资源在构造时获取、析构时释放。`unique_ptr` 在构造时持有裸指针，析构时自动 `delete`，避免内存泄漏 |

---

## 本章目标

学完本章后，你将能够：
1. 用构型空间语言精确形式化机械臂运动规划问题，理解 C-space 与工作空间的本质差异
2. 掌握 PRM/RRT/RRT*/Informed-RRT*/BIT* 五大核心算法的原理、完备性/最优性/收敛率
3. 独立使用 OMPL C++ API（StateSpace/SpaceInformation/Planner 三层抽象）为 7-DOF 臂搭建完整规划系统
4. 理解碰撞检测回调在 OMPL 中的注入机制及其对规划性能的决定性影响
5. 在 MoveIt2 中配置和调优 OMPL 规划器参数，针对不同场景选型
6. 用形式化语言陈述概率完备性与渐近最优性，并理解二者的证明骨架
7. 针对约束流形任务（倒水/焊接）选择正确的 OMPL 约束状态空间并排查投影失败

---

## 本章知识导航

本章的知识结构可以画成一棵"问题驱动"的树：根是"机械臂如何从 A 构型安全运动到 B 构型"，树干是采样规划的理论与算法，树枝是 OMPL 的工程落地与调参。下面这张地图标注了每个小节在树上的位置，以及它解决的核心问题——读之前先扫一遍，建立"我现在学到哪、为什么要学这个"的方位感。

```
                    机械臂运动规划：从 q_start 到 q_goal 的无碰撞路径
                                        │
        ┌───────────────────────────────┼───────────────────────────────┐
        ▼                               ▼                               ▼
   【为什么难】                     【怎么解】                      【怎么落地】
   M07.1 构型空间理论              M07.2 采样策略                  M07.5 OMPL C++ API
   ─ C-space vs 工作空间           ─ 均匀/桥/高斯/信息增益          ─ StateSpace 三层抽象
   ─ 维度灾难                       │                              ─ 碰撞检测注入
   ─ 不可显式构造                   ▼                              M07.6 MoveIt2 集成
        │                       M07.3 PRM 族                      ─ ompl_planning.yaml
        │  (理论基础)            ─ 多次查询路线图                   ─ longest_valid_segment
        │                       ─ PRM* 渐近最优半径                M07.7 约束规划
        └──────────────►        ─ Lazy-PRM                        ─ 约束流形/投影/Atlas
                                M07.4 RRT 族                       M07.8 工业调参
                                ─ RRT / RRT-Connect (可行性)       ─ 场景选型决策树
                                ─ RRT* / Informed (最优性)         ─ 三大真实案例
                                ─ FMT* / BIT* / AIT*               M07.8b 前沿 BIT*/AIT*/VAMP
```

**两条主线贯穿全章**，理解它们就抓住了本章的骨架：

| 主线 | 含义 | 体现在哪些小节 |
|------|------|--------------|
| **可行性 vs 最优性** | "先找到一条能用的路" 还是 "找到最短的路"——这是采样规划最根本的取舍 | M07.3（PRM vs PRM*）、M07.4（RRT-Connect vs RRT*/BIT*）、M07.8（选型） |
| **算法逻辑 vs 碰撞检测** | 规划时间 80-95% 花在碰撞检测而非算法本身——这决定了所有性能优化的方向 | M07.5（开销分析）、M07.7（约束放大）、M07.8b（VAMP/GPU）、M09（预告） |

**§号与正文的一一对应**（本导航中出现的每个 §号在正文都有对应小节，反之亦然）：

| § | 标题 | 难度 | 一句话定位 |
|---|------|------|----------|
| M07.0 | 前置自测 | — | 5 题筛查 M03/M04 与 C++ 基础 |
| M07.1 | 构型空间理论基础 | ⭐ | 为什么不能在工作空间规划 |
| M07.2 | 采样策略深度 | ⭐⭐ | 随机撒点为什么有效、怎么撒更高效 |
| M07.3 | 核心算法：PRM | ⭐⭐ | 预计算路线图供多次查询 |
| M07.4 | 核心算法：RRT 族 | ⭐⭐ | 一次搜索找路径，最优性如何获得 |
| M07.5 | OMPL C++ API 精读 | ⭐⭐ | 三层抽象与碰撞检测注入 |
| M07.6 | MoveIt2 集成配置 | ⭐⭐ | 从独立 OMPL 到生产系统 |
| M07.7 | 规划约束 | ⭐⭐⭐ | 约束流形上的采样与规划 |
| M07.8 | 工业场景调参 | ⭐⭐ | 场景驱动的选型与系统调参 |
| M07.8b | 前沿进展 | ⭐⭐⭐⭐ | BIT*/AIT*/VAMP 最新动向 |

### 前置知识桥接

本章是机械臂全栈管线的"规划层"，强依赖前两章打下的地基。这里用 2-3 行重新激活每个前置概念，让你不必翻回原章也能跟上：

- **M03（IK 求解器）**：在那里我们学会了把末端目标位姿 $T_{\text{goal}} \in SE(3)$ 反解为关节角 $q_{\text{goal}} \in \mathbb{R}^n$，并理解了同一位姿对应多组解（elbow-up/elbow-down，OPW 的 8 组解）。本章默认规划的目标是**已经求好的关节角** $q_{\text{goal}}$；当目标只给位姿时，需要先用 M03 的 IK 转成构型（M07.4 练习 3 与 M07.8 练习 3 正是这个衔接）。
- **M04（碰撞检测 FCL/Coal）**：在那里我们学会了"给定一个构型 $q$，判断机器人是否与环境或自身碰撞"，并区分了宽相（BroadPhase，AABB/SAP 快速排除）与窄相（NarrowPhase，GJK/EPA 精确计算）。本章把这个能力封装成 OMPL 的 `StateValidityChecker` 回调——**采样规划本身不知道任何几何，它只反复问 M04 "这个点行不行"**。
- **C++ 基础（继承多态、设计模式）**：本章的 OMPL API 精读（M07.5）大量用到纯虚函数、策略模式（Strategy）、门面模式（Facade）与 RAII。如果对 `virtual ... = 0` 的分派机制或 `unique_ptr` 的自动释放不熟，建议先回顾对应章节——M07.0 前置自测的第 3、5 题就是这两个点的筛查。

### 如果跳过本章会怎样

| 场景 | 跳过的后果 |
|------|-----------|
| 直接用 MoveIt2 的 `move_group` 而不懂 OMPL | 规划频繁失败时只会无脑调超时时间，不知道真正的瓶颈是碰撞检测、采样策略还是目标不可达，调参变成碰运气 |
| 不理解可行性 vs 最优性 | 在只需"能用"的场景误用 RRT* 等十几秒收敛，或在需要"最短"的场景用 RRT-Connect 得到锯齿绕路，选型全凭直觉 |
| 不懂约束流形 | 遇到倒水/焊接这类带姿态约束的任务，写不出能让末端保持水平的规划，只能退回手工示教 |
| 不理解碰撞检测是瓶颈 | 看到 M09 的 GPU/SIMD 加速会觉得"莫名其妙"——不知道为什么加速的是碰撞检测而非算法逻辑 |

### 预计阅读时间

| 档位 | 时长 | 路径 |
|------|------|------|
| **精读**（推荐） | 15-20 学时（约 1.5 周） | 全章顺序读 + 完成 M07.4/M07.5/M07.8 的编程练习 + 跑一次 OMPL Benchmark |
| **速读** | 4-6 小时 | M07.1 理论直觉 + M07.4 算法对比总表 + M07.5 完整示例 + M07.8 选型决策树，跳过证明细节与约束规划 |
| **速查** | 30 分钟 | 直接看 M07.4「算法对比总表」、M07.6「ompl_planning.yaml」、章末「API 速查表」与「故障排查手册」 |

---

## M07.1 构型空间理论基础 ⭐

### 动机：为什么不能在工作空间里规划？ ⭐

初学者的第一直觉：末端从点 A 移到点 B，在 $\mathbb{R}^3$ 里画条直线绕过障碍不就行了？

这个直觉在移动机器人（Nav2 导航）中是对的——底盘是一个"点"（或带朝向的 SE(2) 刚体），工作空间就是 2D 地图，A* 或 Dijkstra 直接搜。但机械臂不行：

1. **碰撞不只是末端**——机械臂的每一段 link 都可能撞到障碍物或自身。末端走直线时，肘部可能扫到桌子
2. **奇异性**——工作空间中一条短直线可能要求关节角发生剧烈变化（穿越奇异构型）
3. **多解性**——同一个末端位姿对应多组关节角，工作空间路径不唯一映射到关节空间
4. **关节限位**——工作空间路径可能要求某关节超出物理极限

### 如果不这样做会怎样 ⭐

如果强行在工作空间规划并通过 IK 逐点映射到关节空间：

| 问题 | 后果 |
|------|------|
| IK 在路径中段失败（无解） | 路径中断，机器人停在半途 |
| 相邻路径点的 IK 解选择不一致 | 关节角跳变，机械臂"抽搐" |
| 某段路径肘部碰撞但末端无碰撞 | 工作空间检查通过，实际碰撞 |
| 路径穿越奇异构型 | IK 数值不稳定，Jacobian 退化 |

### 形式化定义 ⭐

**定义**: 机器人的构型空间 $\mathcal{C}$ 是所有可能构型 $q$ 组成的空间。

| 机器人类型 | 构型空间 $\mathcal{C}$ | 维度 |
|-----------|----------------------|------|
| 全向移动底盘 | $\mathbb{R}^2$ 或 $SE(2)$ | 2 或 3 |
| 6-DOF 工业臂 | $\mathbb{R}^6$（受关节限位约束） | 6 |
| 7-DOF 冗余臂 (Panda) | $\mathbb{R}^7$ | 7 |
| 移动操作平台 (底盘+臂) | $SE(2) \times \mathbb{R}^7$ | 10 |
| 人形机器人 (浮动基座+全身) | $SE(3) \times \mathbb{R}^{n}$ | 6+n (可达 30+) |

**关键概念**:

$$
\mathcal{C}_{\text{obs}} = \{q \in \mathcal{C} \mid \text{机器人在构型 } q \text{ 时与障碍物碰撞}\}
$$

$$
\mathcal{C}_{\text{free}} = \mathcal{C} \setminus \mathcal{C}_{\text{obs}}
$$

**运动规划问题**: 给定 $q_{\text{start}}, q_{\text{goal}} \in \mathcal{C}_{\text{free}}$，求连续路径 $\sigma: [0,1] \to \mathcal{C}_{\text{free}}$，使得 $\sigma(0) = q_{\text{start}}$，$\sigma(1) = q_{\text{goal}}$。

### 历史脉络 ⭐

| 年份 | 里程碑 | 意义 |
|------|--------|------|
| 1979 | Lozano-Perez (MIT) 提出 C-space 概念 | 将运动规划从工作空间转移到构型空间 |
| 1988 | Schwartz & Sharir 证明运动规划是 PSPACE-hard | 精确算法在高维不可行 |
| 1994 | Kavraki & Latombe (Stanford) 在 ICRA 1994 首次提出 PRM 概念；1996 Kavraki et al. 在 IEEE T-RA 发表完整版 | 第一个实用的随机采样规划器 |
| 1998 | LaValle 提出 RRT | 快速扩展随机树 |
| 2000 | Kuffner & LaValle 提出 RRT-Connect | 双向 RRT + 贪心连接，至今最常用 |
| 2011 | Karaman & Frazzoli 提出 RRT*/PRM* | 渐近最优采样规划 |
| 2012 | Sucan, Moll & Kavraki 发布 OMPL | 统一框架，40+ 算法 |
| 2015 | Gammell et al. 提出 BIT* | 批量信息化搜索，收敛最快 |
| 2024+ | OMPL 2.0 与 VAMP 相关工作并行推进 | SIMD 加速碰撞检测开始通过可选桥接进入 OMPL 风格工作流 |

### C-space 的几何直觉 ⭐

构型空间的核心洞察是 Lozano-Perez (1979) 提出的**C-space 障碍物映射**：将工作空间中的物理障碍"膨胀"到构型空间中。

回顾 M04（碰撞检测）：在那里我们用 FCL/Coal 检测"给定 $q$ 时机器人是否碰撞"。现在换个视角——如果我们能对**所有** $q$ 做碰撞检测，就能画出 $\mathcal{C}_{\text{obs}}$ 的完整形状。

> **本质洞察**: 工作空间中一个简单的长方体障碍物，映射到 7 维构型空间后会变成**极其复杂的高维几何体**。这就是为什么构型空间无法精确计算——只能通过采样去探索。

**跨领域类比**: C-space 障碍物映射类似于概率论中的**坐标变换**。工作空间到 C-space 的障碍映射，就像把简单分布通过非线性变换映射到高维空间——变换后的形状可能极度复杂，正如正态分布通过 $y = x^3$ 变换后不再对称。两者的共同点是：变换前简单、变换后复杂，且没有解析逆。

### 为什么构型空间"不可显式构造" ⭐⭐

7-DOF 臂的 $\mathcal{C} = \mathbb{R}^7$。如果每个关节离散化为 100 个点，构型空间有 $100^7 = 10^{14}$ 个格点——即使每个格点只存一个 bit（碰撞/不碰撞），也需要约 12.5 TB 存储。而且每个格点都需要一次完整的碰撞检测（回顾 M04：一次 FCL 检测 ~50 us），全部检测需要 $10^{14} \times 50 \times 10^{-6} \text{s} \approx 160$ 年。

> **反事实推理**: 如果构型空间可以预计算为栅格地图，我们就能直接用 A* 搜索——像 Nav2 搜 2D 地图一样。但 7 维空间的"维度灾难"(Curse of Dimensionality) 让这条路在计算和存储上都不可行。这正是采样规划器（PRM/RRT）存在的根本原因——它们用**随机采样**代替穷举，只探索"有用的"构型空间区域。

### 度量与拓扑 ⭐⭐

不同构型空间有不同的**距离度量**和**拓扑结构**：

| 空间 | 度量 | 拓扑 |
|------|------|------|
| $\mathbb{R}^n$（关节角） | 欧氏距离 $\|q_1 - q_2\|_2$ | 开集，有界（关节限位） |
| $SO(3)$（三维旋转） | 测地距离 $\|\text{Log}(R_1^T R_2)\|$ | 紧致流形 |
| $SE(3)$（刚体位姿） | 加权距离 $\alpha\cdot d_{\text{rot}} + \beta\cdot d_{\text{trans}}$ | 非紧致 |
| $\mathbb{T}^n$（连续旋转关节） | 角度距离，考虑 $2\pi$ 周期 | 环面拓扑 |

**为什么度量很重要？** 采样规划器的核心操作是"找最近邻"——这依赖距离度量。如果度量选错，算法效率会大幅下降。OMPL 的 `StateSpace` 抽象正是为了让每种空间自定义正确的度量。

### ⚠️ 常见陷阱

**编程陷阱: 关节角的周期性**

```
⚠️ 对连续旋转关节 (continuous joint)，角度 0 和 2π 是同一构型。
   错误做法: 直接用欧氏距离 |θ₁ - θ₂|
   现象: θ₁ = 0.1, θ₂ = 6.18 (≈ 2π) 的距离被计算为 6.08，
         实际应为 0.1。规划器会认为两点很远，生成不必要的绕路。
   正确做法: 使用 atan2(sin(θ₁-θ₂), cos(θ₁-θ₂)) 计算角度差，
            或使用 OMPL 的 SO2StateSpace 自动处理。
```

**概念误区: 混淆 C-space 和工作空间距离**

```
💡 两个构型 q₁ 和 q₂ 在关节空间中很近（欧氏距离小），
   但对应的末端位姿 FK(q₁) 和 FK(q₂) 可能很远（例如穿越奇异构型）。
   反之，末端位姿接近的两组构型，关节角可能差距巨大
   （例如 elbow-up vs elbow-down, 回顾 M03.2 中 OPW 的 8 组解）。
   这是 C-space 规划区别于工作空间规划的根本所在。
```

### 练习

1. **[手推]** 对一个 2-DOF 平面臂（两根连杆，长度各 1m），在工作空间中放一个半径 0.3m 的圆形障碍（圆心 (1.0, 0.5)）。手画 $\mathcal{C}_{\text{obs}}$ 在 $(\theta_1, \theta_2)$ 平面上的大致形状。提示：固定 $\theta_1$，遍历 $\theta_2$ 看哪些构型碰撞。
2. **[编程]** 用 Pinocchio 加载 Panda URDF，在 $\mathcal{C}$ 中均匀采样 10000 个构型，对每个构型用 FCL 做碰撞检测（回顾 M04 的 `computeCollisions` API），统计 $\mathcal{C}_{\text{free}}$ 的体积占比。讨论：这个比例如何随障碍物增多而变化？
3. **[思考]** SLAM 中 Nav2 的全局规划为什么可以用 A* 在 2D 栅格地图上搜索？如果要对 Panda 也用 A* 栅格搜索，需要多大的栅格？这说明了什么？

---

## M07.2 采样策略深度 ⭐⭐

### 动机：随机采样为什么有效？ ⭐

既然不能穷举构型空间，采样规划的核心策略是：**随机撒点，逐步探索**。但"怎么撒点"直接决定规划效率。本节深入分析四种主要采样策略及其适用场景。

### 均匀采样 (Uniform Sampling) ⭐

最基础的策略：在 $[q_{\min}, q_{\max}]$ 超矩形内均匀随机采样。

```cpp
// OMPL 默认采样器 (教学简化)
class RealVectorUniformSampler : public StateSampler {
    void sampleUniform(State* state) override {
        auto* rv = state->as<RealVectorStateSpace::StateType>();
        for (unsigned i = 0; i < dim_; ++i) {
            // 在关节限位范围内均匀采样
            rv->values[i] = rng_.uniformReal(bounds_.low[i],
                                              bounds_.high[i]);
        }
    }
};
```

**优点**: 实现简单，对开阔空间效率尚可。

**缺点**: 在**窄通道**（narrow passage）场景中效率极低——自由空间体积占比极小，绝大多数采样点落在 $\mathcal{C}_{\text{obs}}$ 中被丢弃。

### 桥采样 (Bridge Sampling) ⭐⭐

Hsu et al. (2003) 提出，专门解决窄通道问题。核心思想：窄通道中的自由构型，其"两端"（沿某方向延伸）往往在障碍物内。

```
算法:
1. 在 C 中均匀采样 q₁
2. 如果 q₁ ∈ C_obs (碰撞):
   a. 在 q₁ 附近高斯采样 q₂ ~ N(q₁, σ²I)
   b. 如果 q₂ ∈ C_obs:
      c. 取中点 q_mid = (q₁ + q₂) / 2
      d. 如果 q_mid ∈ C_free:
         → q_mid 很可能在窄通道中! 加入路标
```

**为什么有效？** 窄通道中的点 $q_{\text{mid}}$ 恰好被两侧障碍"夹"住。如果 $q_1$ 和 $q_2$ 都在障碍中且间距适当，中点很可能落在通道内。

**窄通道为什么是采样规划的死穴——一个定量估计**：设窄通道在 $d$ 维构型空间中近似为一个"厚度"为 $w$、其余方向尺度为 $L$ 的薄板，则均匀采样**单次命中**通道的概率约为

$$
p_{\text{hit}} \approx \frac{\mu(\text{通道})}{\mu(\mathcal{C})} \sim \frac{w \cdot L^{d-1}}{L^d} = \frac{w}{L}.
$$

要让树穿过通道，需要在通道内采到点。命中之前的期望采样次数约 $1/p_{\text{hit}} = L/w$——对单一窄通道尚可，但若一条路径要连续穿过 $k$ 个独立窄通道，全部命中的概率退化为 $(w/L)^k$，期望采样次数 $(L/w)^k$ **指数爆炸**。这正是上一节（M07.1）维度灾难在采样层面的具体表现，也精确对应 M07.4 概率完备性证明里那个"单步成功下界 $p_0$"——窄通道让 $p_0 \approx w/L$ 趋于零，达到同样成功概率所需的迭代数随之指数增长。桥采样的全部意义，就是把"盲目均匀采样"换成"专挑被夹住的中点"，人为把通道内的有效命中率 $p_0$ 抬高一两个数量级。

**跨领域类比**: 桥采样类似于蒙特卡洛积分中的**重要性采样**——在关键区域（窄通道）集中采样点，而非浪费在开阔空间。就像估计尾部概率时，在尾部多采样比在均值附近多采样效率高得多。两者相似在"按重要性重新分配采样密度"；不同在于重要性采样需要显式的提议分布并做密度加权校正，而桥采样靠"两端碰撞、中点自由"这个几何判据隐式定位重要区域，不做加权——所以桥采样会改变采样分布的统计性质，用它构建的 PRM 不再保持 PRM* 的渐近最优半径理论，这是它的代价。

### 高斯采样 (Gaussian Sampling) ⭐⭐

Boor et al. (1999) 提出：只保留**靠近障碍物边界**的采样点。

```
算法:
1. 在 C 中均匀采样 q₁
2. 在 q₁ 附近高斯采样 q₂ ~ N(q₁, σ²I)
3. 如果恰好一个碰撞、一个不碰撞:
   → 保留不碰撞的那个 (它靠近 C_free 边界)
4. 否则丢弃
```

**直觉**: 在 $\mathcal{C}_{\text{free}}$ 的边界附近密集采样，有助于刻画窄通道的形状。

### 信息增益采样 (Informed Sampling) ⭐⭐⭐

Gammell et al. (2014) 的关键洞察：**找到初始解后，只在可能改进解的区域采样**。

当已有一条路径、长度为 $c_{\text{best}}$ 时，更优路径的所有中间构型必然满足：

$$
\|q - q_{\text{start}}\| + \|q - q_{\text{goal}}\| \leq c_{\text{best}}
$$

这定义了一个**椭球**（以 $q_{\text{start}}$ 和 $q_{\text{goal}}$ 为焦点），只在椭球内采样即可。

$$
\text{椭球参数}: \quad a = \frac{c_{\text{best}}}{2}, \quad b = \frac{\sqrt{c_{\text{best}}^2 - c_{\min}^2}}{2}
$$

其中 $c_{\min} = \|q_{\text{goal}} - q_{\text{start}}\|$ 是直线距离。

**椭球采样的实现**: 在单位超球中均匀采样，然后线性变换到椭球：

$$
q_{\text{sample}} = C \cdot L \cdot q_{\text{ball}} + q_{\text{center}}
$$

其中 $C$ 是从起点-终点方向到坐标轴的旋转矩阵，$L = \text{diag}(a, b, b, \ldots, b)$。

> **反事实推理**: 如果不用 Informed 采样会怎样？RRT* 找到初始解后，继续在整个 $\mathcal{C}$ 中均匀采样——绝大多数新采样点无法改进当前解，被白白浪费。在 7 维空间中，椭球体积可能只占全空间的 1%——Informed 采样等于把有效采样率提升了 100 倍。

### 采样策略选型表 ⭐⭐

| 策略 | 适用场景 | 不适用场景 | OMPL 实现 |
|------|---------|-----------|----------|
| 均匀 | 开阔空间、低维 | 窄通道 | `UniformValidStateSampler` |
| 桥 | 窄通道、狭缝 | 开阔空间（浪费采样） | `BridgeTestValidStateSampler` |
| 高斯 | 障碍密集 | 远离障碍的目标 | `GaussianValidStateSampler` |
| 信息增益 | 找到初始解后优化 | 初始解搜索 | 内置于 Informed-RRT*/BIT* |
| Halton/低差异序列 | 需要均匀覆盖 | 需要纯随机性 | `HaltonSequence` (VAMP 内部使用) |

### ⚠️ 常见陷阱

**概念误区: 认为更多采样一定更好**

```
💡 在窄通道场景中，增加均匀采样的数量几乎无效——
   如果窄通道在 C-space 中的体积占比为 10⁻⁶，
   即使采样 10⁸ 个点，期望只有 100 个落在通道中。
   切换到桥采样可能 1000 个点就够了。
   这不是数量问题，而是策略问题。
```

**思维陷阱: 总想找"最好的"采样策略**

```
🧠 没有全局最优的采样策略。每种策略都对某类场景有优势：
   - 你不知道场景特征 → 均匀采样（鲁棒基线）
   - 你知道有窄通道 → 桥采样
   - 你已经有初始解要优化 → 信息增益采样
   MoveIt2 默认用均匀采样，因为它对未知场景最鲁棒。
   正确思维: 先看场景特征，再选采样策略。
```

### 练习

1. **[编程]** 在 2D 环境中（$\mathcal{C} = [0,10]^2$，一个 L 形障碍物制造窄通道），分别用均匀采样和桥采样各撒 500 个点。可视化采样点分布，观察桥采样是否在窄通道中集中。
2. **[手推]** 对 $\mathbb{R}^2$ 中的 Informed 椭球采样，给定 $q_{\text{start}} = (0,0)$、$q_{\text{goal}} = (4,0)$、$c_{\text{best}} = 6$，计算椭球长半轴 $a$ 和短半轴 $b$。画出椭球，标注采样区域面积相比全空间缩小了多少倍。
3. **[思考]** 为什么 VAMP (M09 将详细讲) 使用 Halton 低差异序列而非伪随机数？低差异序列在高维空间中覆盖均匀性优于伪随机的理论依据是什么？

---

## M07.3 核心算法深度: PRM ⭐⭐

### 动机：能否预计算一张"路线图"供反复查询？ ⭐

Kavraki et al. (1996, Stanford) 的洞察：如果同一个工作环境不变，但需要频繁规划不同的起点-终点对（如工厂产线多个工位之间的搬运），反复从零建树太浪费。

> **跨领域类比**: PRM 之于运动规划，如同 Google Maps 的路网之于导航——预先建好路网（路标图），每次导航只做图搜索。区别在于：Google Maps 的路网是确定的（实际道路），PRM 的路网是随机采样构建的（概率路标）。

### PRM 算法 ⭐⭐

**阶段 1: 预处理（构建路标图）**

```
输入: C_free (通过碰撞检测隐式定义)
参数: N (采样点数), K (邻居数)

roadmap = 空图
for i = 1 to N:
    q_rand = 在 C 中随机采样
    if isValid(q_rand):          // 碰撞检测 (M04)
        roadmap.addVertex(q_rand)
        neighbors = nearestK(roadmap, q_rand, K)   // K-近邻
        for each q_near in neighbors:
            if canConnect(q_rand, q_near):  // 路径碰撞检测
                roadmap.addEdge(q_rand, q_near)
```

**阶段 2: 查询（图搜索）**

```
输入: q_start, q_goal

将 q_start, q_goal 连接到 roadmap 的最近节点
在 roadmap 上运行 A* / Dijkstra 求最短路径
```

**完备性**: PRM 是概率完备的——随着 $N \to \infty$，若路径存在则一定能找到。但对于固定 $N$，不保证成功（可能恰好没采样到窄通道中的关键构型）。

### PRM* 的渐近最优性 ⭐⭐⭐

原始 PRM 常用固定连接半径或固定 K 近邻——这不能保证最优性。Karaman & Frazzoli (2011, MIT) 证明了 PRM* 的最优性条件：

连接半径需随采样数 $n$ 按以下公式缩小：

$$
r(n) = \gamma \cdot \left(\frac{\log n}{n}\right)^{1/d}
$$

其中 $d$ 是构型空间维度，$\gamma > \gamma^* = 2 \left(1+\frac{1}{d}\right)^{1/d} \left(\frac{\mu(\mathcal{C}_{\text{free}})}{\zeta_d}\right)^{1/d}$，$\zeta_d$ 是 $d$ 维单位球体积。注意这是半径型（r-disc）PRM* 的条件；k-nearest PRM* 另用 $k(n)=k_0 \log n$，且 $k_0 > e(1+1/d)$，不能把固定 K 近邻直接套进上面的半径公式。

**推导直觉**: 这个半径保证了两件事：
1. $r(n) \to 0$（随着 $n$ 增大，连接越来越局部，避免图过于稠密）
2. $r(n)$ 不能缩得太快——$\log n$ 因子保证图始终连通

> **反事实推理**: 如果连接半径不随 $n$ 变化会怎样？固定半径太大 → 图过于稠密，查询时 A* 太慢（$O(n^2)$ 条边）；固定半径太小 → 图不连通，找不到路径。PRM* 的自适应半径在连通性和效率之间取得最优平衡。

### r-disc 与 k-nearest：PRM* 的两种连接策略 ⭐⭐⭐

上一段给出的是**半径型（r-disc）** PRM*——每个新节点与半径 $r(n)$ 内的所有邻居相连。但 OMPL 的 `PRMstar` 与 `RRTstar` 默认走的其实是另一条等价路线：**k-近邻型（k-nearest）**。两者都能达到渐近最优，但工程行为不同，初学者最容易在这里把固定 K 近邻（原始 PRM 的做法）误当成 k-nearest PRM*。把这两条策略并排讲清楚，正好为 M07.4 的 RRT*/BIT* 的 `use_k_nearest` 配置项（M07.6 的 YAML 里出现过）打下地基。

**两种策略的渐近最优条件并排对照**：

| 策略 | 连接规则 | 渐近最优阈值 | OMPL 实现 |
|------|---------|------------|----------|
| r-disc | 连接 $r(n)$ 半径内全部邻居 | $r(n)=\gamma\left(\frac{\log n}{n}\right)^{1/d}$，$\gamma>\gamma^*$ | `setRange` 间接控制 |
| k-nearest | 连接最近的 $k(n)$ 个邻居 | $k(n)=k_0\log n$，$k_0>e\left(1+\frac{1}{d}\right)$ | `setMaxNearestNeighbors`，`use_k_nearest:1` |

关键洞察是：两者的"自适应"形式不同，但量级一致。r-disc 让半径随 $n$ **收缩**（$r(n)\to 0$），k-nearest 让邻居数随 $n$ **缓慢增长**（$k(n)=k_0\log n\to\infty$）。这并不矛盾——在 $n$ 个均匀点中，半径 $r(n)$ 的球内**期望**恰好包含 $n\cdot\frac{\zeta_d r(n)^d}{\mu(\mathcal{C}_{\text{free}})}\sim\log n$ 个点。换言之，"半径内点数"和"近邻个数"在期望意义上都被钉死在 $\Theta(\log n)$ 这个"刚好够连通又不至于稠密"的临界量级上——这正是 M07.4 会用到的随机几何图（RGG）连通相变的同一个 $\log n$。

> **对比性思维（不是 X 而是 Y）**：固定 K 近邻（每个点永远连最近的 10 个，原始 PRM 的常见写法）**不是** k-nearest PRM*。差别在于 $k$ 是否随 $n$ 增长：固定 $k$ 时，随采样加密，每个点的邻居在空间上越挨越近，图退化成一堆局部小团，跨区域的"长边"永远不会出现——于是图可能整体连通但最优路径所需的边缺失，渐近最优**失效**。只有让 $k(n)=k_0\log n$ 随 $n$ 增长，才能在加密的同时保留足够的连接广度。这一字之差（$k$ 常数还是 $k\propto\log n$），就是"看着像最优"和"真能最优"的分水岭，和 M07.4 里 RRT 与 RRT* "能否改父节点"的分野是同一类陷阱。

**工程上该选哪个？** k-nearest 在实践中通常更受青睐，原因是它对 $\mu(\mathcal{C}_{\text{free}})$ 的估计不敏感——r-disc 的 $\gamma^*$ 阈值里含 $\mu(\mathcal{C}_{\text{free}})$，而自由空间体积在有障碍场景里根本无法精确知道，只能保守高估，导致半径偏大、图偏稠密；k-nearest 的阈值 $k_0>e(1+1/d)$ 只依赖维度 $d$，与自由空间体积无关，因此更稳健、更易调参。这也是 OMPL 把 `use_k_nearest` 默认设为 1 的原因。

> **本质洞察**：r-disc 和 k-nearest 不是两个"不同算法"，而是同一个渐近最优条件在"以距离为锚"和"以个数为锚"两种坐标下的写法。理解了这一点，你就能在 M07.6 的 YAML 里看懂 `use_k_nearest` 和 `rewire_factor`——后者正是把理论阈值 $k_0$（或 $\gamma$）乘上一个 $\geq 1$ 的安全系数，让实际连接比理论下界更密一点以换取经验上更高的成功率，代价是更多的碰撞检测。

### Lazy-PRM: 延迟碰撞检查 ⭐⭐

碰撞检测是 PRM 预处理的最大开销（80-95% 时间）。Lazy-PRM 的思路：

```
预处理阶段: 只检查节点是否碰撞，不检查边
查询阶段:
  1. 在路标图上搜最短路径
  2. 沿路径逐边做碰撞检测
  3. 如果某边碰撞 → 删除该边，重新搜索
  4. 重复直到找到无碰撞路径或报告失败
```

**优势**: 只检查"有用的"边，预处理时间大幅缩短。

### PRM 的适用场景与局限 ⭐⭐

| 优势 | 局限 |
|------|------|
| 多次查询高效（预计算一次） | 预处理时间长（分钟级） |
| 环境不变时最优 | 环境变化需重建 |
| 可增量更新 | 高维空间路标数需指数增长 |
| 并行化简单（各节点独立） | 窄通道需大量采样 |

**最佳适用**: 固定环境、多次查询（工厂产线、重复抓取任务）。

### ⚠️ 常见陷阱

**编程陷阱: Lazy-PRM 的重试上限**

```
⚠️ Lazy-PRM 先建图、不做碰撞检测，查询时才沿路径检查。
   如果路径上某条边碰撞，删除该边后重新搜索。
   错误做法: 不设重试上限，在密集障碍场景中陷入无限循环。
   正确做法: 设置最大重试次数（OMPL 默认 100），超过则报告失败。
```

### 练习

1. **[编程]** 用 OMPL 的 PRM* 在 $\mathbb{R}^2$ 中构建路标图（100 个障碍矩形）。可视化路标图结构。然后查询 10 对随机起点-终点，记录查询时间。与单次 RRT-Connect 对比总耗时。
2. **[思考]** 在动态环境中（障碍物移动），PRM 的路标图可能包含已失效的边。如何高效处理？提示：考虑 Lazy 检查 vs 增量删除/重建。
3. **[对比]** 用 OMPL 的 `PRMstar` 在同一 $\mathbb{R}^7$ 场景上分别设 `use_k_nearest:1` 和 `use_k_nearest:0`（r-disc），各跑 50 次，记录路标图边数、平均度数与查询成功率。验证 r-disc 因需保守高估 $\mu(\mathcal{C}_{\text{free}})$ 而边数偏多的预期；再把固定 K 近邻（不随 $n$ 增长）与之对比，观察固定 $k$ 在采样加密后成功率为何不再提升。

---

## M07.4 核心算法深度: RRT 族 ⭐⭐

### 动机：能否一次搜索就找到路径，不需要预计算？ ⭐

PRM 适合多次查询，但预处理成本高。LaValle (1998, Iowa State) 提出 RRT(Rapidly-exploring Random Tree)：从起点**生长一棵树**向自由空间扩展，一次搜索一次使用。

### RRT 基本算法 ⭐⭐

```
RRT(q_start, q_goal, max_iter):
    tree = {q_start}
    for i = 1 to max_iter:
        q_rand = 随机采样()
        q_near = 在 tree 中找 q_rand 的最近邻
        q_new  = 从 q_near 向 q_rand 方向前进步长 ε
        if canConnect(q_near, q_new):   // 碰撞检测 (M04)
            tree.addNode(q_new, parent=q_near)
            if distance(q_new, q_goal) < tolerance:
                return extractPath(tree, q_start, q_new)
    return FAILURE
```

**几何直觉**: 树像章鱼的触手一样，向未探索区域伸展。均匀采样保证树"遍历"整个 $\mathcal{C}_{\text{free}}$——这就是**Voronoi 偏置**(Voronoi bias)：距离当前树节点最远的区域被采样到的概率最高，因为其 Voronoi 区域最大。

**完备性证明思路**: 当 $n \to \infty$，树的节点在 $\mathcal{C}_{\text{free}}$ 中稠密。如果存在路径 $\sigma$，则 $\sigma$ 的每个 $\epsilon$-邻域内最终都有树节点。因此连接相邻节点可复原 $\sigma$。

### RRT-Connect: 双向生长 + 贪心连接 (Kuffner & LaValle, ICRA 2000) ⭐⭐

RRT 的两个关键改进：

1. **双向搜索**: 从 $q_{\text{start}}$ 和 $q_{\text{goal}}$ 同时生长两棵树
2. **Connect 启发**: 不是只前进一步（Extend），而是**贪心地多步前进**直到碰撞或到达目标

```
RRT-Connect(q_start, q_goal, max_iter):
    tree_a = {q_start}
    tree_b = {q_goal}
    for i = 1 to max_iter:
        q_rand = 随机采样()
        if EXTEND(tree_a, q_rand) ≠ TRAPPED:
            if CONNECT(tree_b, q_new_a) == REACHED:
                return mergePaths(tree_a, tree_b)
        swap(tree_a, tree_b)    // 交替生长
    return FAILURE

EXTEND(tree, q):
    q_near = 最近邻(tree, q)
    q_new  = q_near + ε * (q - q_near) / ||q - q_near||
    if isValid(q_new) and canConnect(q_near, q_new):
        tree.add(q_new)
        return (q_new == q) ? REACHED : ADVANCED
    return TRAPPED

CONNECT(tree, q):
    while EXTEND(tree, q) == ADVANCED:
        continue       // 贪心: 一直延伸到碰撞或到达
    return last_status
```

**为什么 MoveIt2 默认用 RRT-Connect？**

- 双向搜索比单向平均快 ~2 倍（两棵树在中间汇合）
- Connect 贪心策略在开阔空间中极其高效
- 不追求最优——只找可行路径，再交给 M08（轨迹优化）改善质量

> **本质洞察**: RRT-Connect 的哲学是"先找到一条能用的路径，质量交给后续优化"。这种"可行性优先+后续优化"的两阶段范式，贯穿了从 MoveIt2 的链式规划管线（M08）到 GPU 加速运动生成（M09）的整个运动规划技术栈。不是 A or B（采样 or 优化），而是 A then B（采样找可行路径，优化提升质量）。

### 插曲：概率完备性的形式化与证明骨架 ⭐⭐⭐

前面在 M07.0 自测和 PRM 小节里反复用到"概率完备"这个词，但只给了直觉描述。这一节把它讲透——因为它是理解"采样规划凭什么靠得住"的理论支柱，也是区分 RRT-Connect 与"完整图搜索（如 A*）"的根本所在。

**动机：我们到底在担心什么？** 采样规划是随机算法——每次运行采样序列都不同，结果可能不同。一个自然的恐惧是："万一它永远找不到那条明明存在的路怎么办？"概率完备性就是用来打消（或限定）这个恐惧的数学保证。

**形式化定义**：设规划问题存在一条解路径。记 $A_n$ 为"算法在前 $n$ 次迭代内找到解"这一事件。算法是**概率完备的（Probabilistically Complete）**，当且仅当

$$
\lim_{n \to \infty} \Pr[A_n] = 1.
$$

注意三个细节，初学者最容易在这里建错心智模型：

1. **它是关于"解存在"的条件保证**。如果解根本不存在（$q_{\text{start}}$ 和 $q_{\text{goal}}$ 在 $\mathcal{C}_{\text{free}}$ 的不同连通分量），概率完备性什么也不承诺——采样规划器会一直跑到超时，既不会找到解，也无法证明"无解"。这是它与确定性完备算法的本质差别。
2. **它是渐进保证，不是有限步保证**。"$n \to \infty$ 时概率趋于 1"不等于"某个有限 $n$ 必然成功"。对任意固定的 $n$，失败概率 $\Pr[\bar A_n] > 0$ 始终为正。这就是为什么工程上要设超时与重试。
3. **极限是 1 而非"等于 1"**。这是一个收敛过程：失败概率指数衰减但永不真正归零。

**为什么需要一个"鲁棒性"前提？** 上述极限不会凭空成立——如果解通道是一条"零宽度"的缝（测度为零），随机采样命中它的概率就是零，极限也到不了 1。因此概率完备性的标准证明都建立在一个几何前提上：

> **$\delta$-鲁棒可解（$\delta$-robustly feasible）**：存在一条解路径 $\sigma$，使得其上每一点周围半径 $\delta > 0$ 的球都完全落在 $\mathcal{C}_{\text{free}}$ 内。直观地说，存在一条"有一定宽度的安全走廊"连接起点和终点，而不只是一条无限细的线。

**RRT 概率完备性的证明骨架**（LaValle & Kuffner 2001，这里给出可复述的三步逻辑，而非完整 $\epsilon$-$\delta$ 推导）：

1. **覆盖走廊**：把 $\delta$-鲁棒走廊 $\sigma$ 切成 $m$ 段，每段长度小于步长 $\epsilon$，于是沿走廊形成一串相互重叠、半径 $\delta$ 的球 $B_1, B_2, \ldots, B_m$，$B_1$ 含起点、$B_m$ 含终点。
2. **单步成功概率有正下界**：假设树已经在 $B_i$ 中有节点。由于均匀采样在 $B_{i+1}$ 内采到点的概率 $p \ge \mu(B_{i+1})/\mu(\mathcal{C}) > 0$（$\mu$ 是体积测度），且 Voronoi 偏置使靠近边界的树节点更可能被选中扩展，因此"在一次迭代里把树从 $B_i$ 推进到 $B_{i+1}$"的概率有一个与 $n$ 无关的正下界 $p_0 > 0$。
3. **几何级数收敛**：要走完整条走廊需要 $m$ 次成功推进。每次失败的概率至多 $(1-p_0)$，因此在 $n$ 次迭代内**没能**走完走廊的概率被一个随 $n$ 指数衰减的量控制——形式上 $\Pr[\bar A_n] \le c\, e^{-b n}$（$b, c > 0$ 是与问题相关的常数）。令 $n \to \infty$ 即得 $\Pr[A_n] \to 1$。$\blacksquare$

> **本质洞察**：概率完备性的证明本质是"几何级数把指数衰减压到零"。关键不在于某一步多聪明，而在于**每一步成功概率都有一个不随时间退化的正下界**——只要这个下界存在，无限多次尝试就几乎必然成功。这也解释了为什么窄通道（$\delta \to 0$）是采样规划的死穴：$\delta$ 越小，单步下界 $p_0$ 越小，达到同样成功概率所需的 $n$ 就指数级增长（回顾 M07.2 桥采样正是为了人为抬高窄通道内的 $p_0$）。

**反事实推理：为什么需要这套理论，而不是直接相信"撒够多点总能找到"？** 如果没有 $\delta$-鲁棒前提，"撒够多点"是错的——存在解但采样规划几乎必然找不到的反例（零测度通道）。理论的价值正在于**划清边界**：它告诉你采样规划在什么条件下可靠（走廊有正宽度）、在什么条件下无能为力（零宽度缝隙、无解判定），让你在工程上知道何时该信任规划器、何时该怀疑是建模问题（回顾 M07.8 的"把规划失败归咎于算法"思维陷阱）。

### RRT*: 渐近最优 (Karaman & Frazzoli, IJRR 2011) ⭐⭐⭐

RRT 找到的路径通常很"锯齿"——远非最优。RRT* 在 RRT 基础上加入两个关键操作：

**操作 1: 选择最优父节点 (Choose Parent)**

新节点 $q_{\text{new}}$ 不一定选 $q_{\text{near}}$ 做父节点。在半径 $r(n)$ 内搜索所有邻居，选使"从起点到 $q_{\text{new}}$ 的总代价最小"的那个做父节点。

$$
q_{\text{parent}}^* = \arg\min_{q \in \text{Near}(q_{\text{new}}, r(n))} \left[\text{cost}(q) + c(q, q_{\text{new}})\right]
$$

**操作 2: 重连 (Rewire)**

$q_{\text{new}}$ 加入树后，检查其邻域内的节点：如果经过 $q_{\text{new}}$ 到达某邻居的代价**更低**，重新连接（改变父节点）。

```
for each q_nb in Near(q_new, r(n)):
    new_cost = cost(q_new) + dist(q_new, q_nb)
    if new_cost < cost(q_nb) and canConnect(q_new, q_nb):
        tree.changeParent(q_nb, q_new)
        propagateCostUpdate(q_nb)   // 递归更新子树代价
```

**收敛率**: RRT* 的路径代价以 $O(n^{-1/d})$ 的速率收敛到最优值（$d$ 是维度，$n$ 是采样数）。7-DOF 时 $d=7$，收敛非常慢——需要约 $10^5$ 次采样才能接近最优。

**收敛率的维度诅咒详解**:

为什么 $O(n^{-1/d})$ 是灾难性的？以下是不同维度下达到 5% 最优间隙所需的采样数估算：

| 维度 $d$ | 收敛率 $n^{-1/d}$ | 达到 5% 最优间隙所需 $n$ |
|----------|------------------|------------------------|
| 2 | $n^{-0.5}$ | ~400 |
| 3 | $n^{-0.33}$ | ~8,000 |
| 6 | $n^{-0.17}$ | ~$10^5$ |
| 7 | $n^{-0.14}$ | ~$3 \times 10^5$ |
| 14 | $n^{-0.07}$ | ~$10^{10}$ |

7-DOF 已经需要十万级采样（数十秒），14-DOF（双臂）在实际中不可能用 RRT* 收敛到最优。这正是 FMT*/BIT*/AIT* 存在的动机——通过更好的搜索策略降低等效维度。

### 渐近最优性的形式化：RRT 与 RRT* 的本质分野 ⭐⭐⭐

"RRT* 渐近最优、RRT 不最优"是本章被引用最多的结论之一，但它的精确含义远比口语化的"采样够多就最优"微妙。这里把它讲清楚，因为它直接决定你在 M07.8 的选型——什么时候非 RRT* 不可、什么时候用 RRT-Connect 反而更明智。

**形式化定义**：记 $Y_n$ 为算法在第 $n$ 次迭代时返回的最优路径代价（随机变量），$c^*$ 为真实最优代价。算法是**渐近最优的（Asymptotically Optimal, AO）**，当且仅当返回代价**几乎必然（almost surely, a.s.）**收敛到最优：

$$
\Pr\left[\lim_{n \to \infty} Y_n = c^*\right] = 1.
$$

"几乎必然收敛"是比"概率收敛"更强的随机收敛模式——它要求**几乎每一条采样序列（样本路径）**自身都收敛到 $c^*$，而不只是"在每个固定时刻接近 $c^*$ 的概率高"。这个强度是必要的：我们要的是"让它一直跑，路径质量单调逼近最优"，而不是"某一时刻碰巧好"。

**Karaman & Frazzoli (2011) 的两个对偶结论**：

1. **RRT 几乎必然收敛到一个次优值**。标准 RRT 一旦把某节点接入树，其父节点就**永不改变**——树的结构被早期的随机选择"锁死"。Karaman & Frazzoli 证明：RRT 返回代价几乎必然收敛到某个随机的极限值 $Y_\infty$，而

$$
\Pr[Y_\infty = c^*] = 0.
$$

   也就是说，**RRT 找到真正最优解的概率恰好是零**。这不是"收敛慢"，而是"根本不收敛到最优"——再多采样也无济于事。直觉：最优路径要求一组特定的边恰好都被选中且顺序正确，这在连续空间里是一个零测度事件。

2. **RRT\* 通过 Rewire 修复了这个缺陷**。M07.4 前面讲的两个操作（选择最优父节点 + 重连）让树结构**可以随新样本不断改善**——任何一条边只要发现更短的替代连接就会被替换。配合随 $n$ 收缩的连接半径 $r(n) = \gamma(\log n / n)^{1/d}$，RRT* 达到上面的 AO 定义。

> **对比性思维（不是 X 而是 Y）**：RRT 与 RRT* 的差别**不是"算法更先进"，而是"树结构是否可改写"**。RRT 是一棵"只生长不修剪"的树（append-only），早期错误永久保留；RRT* 是一棵"边长边修"的树（mutable），用 Rewire 把局部错误不断纠正。这一字之差（能否改父节点）就是次优与渐近最优的分水岭。

**连接半径里的 $\log n$ 因子从哪来？** 这是渐近最优性证明的技术核心，值得展开（回顾 M07.3 PRM* 用的是同一个 $r(n)$）。半径必须满足两个相互拉扯的要求：

- **下界（连通性）**：随机几何图（Random Geometric Graph, RGG）理论指出，$n$ 个点在 $d$ 维空间中要"几乎必然连通"，连接半径不能比 $(\log n / n)^{1/d}$ 衰减得更快。$\log n$ 因子正是连通性相变（connectivity threshold）的临界量级——少了它图会碎成不连通的块，最优路径所需的边可能根本不存在。
- **上界（效率/稀疏性）**：半径又必须随 $n$ 趋于零（$r(n) \to 0$），否则每个节点的邻居数无界增长，图变稠密，每次 Rewire 的代价和总边数都爆炸（$O(n^2)$）。$(1/n)^{1/d}$ 这部分保证邻居数维持在 $O(\log n)$ 这个"刚好够用"的水平。

$\gamma$ 的常数门槛 $\gamma > \gamma^* = 2(1+1/d)^{1/d}(\mu(\mathcal{C}_{\text{free}})/\zeta_d)^{1/d}$ 就是把"恰好越过连通相变"这件事写成了显式公式（$\zeta_d$ 是 $d$ 维单位球体积）。

> **前沿勘误（研究级 ⭐⭐⭐⭐）**：Solovey、Janson、Schmerling、Frazzoli 与 Pavone 等人在 *Revisiting the Asymptotic Optimality of RRT\** (ICRA 2020) 中指出，Karaman & Frazzoli 原始证明在处理"样本到达顺序"时存在一个逻辑缺口——RRT* 的边代价依赖样本被加入的**时间顺序**，而原证明把它当作了顺序无关的 RGG。他们给出了修正后的严格证明，并指出为吃掉"时间这一额外维度"，RRT* 的连接半径指数应从 $(\log n/n)^{1/d}$ 收紧到 $(\log n/n)^{1/(d+1)}$。结论（RRT* 是 AO）不变，但条件更严。这说明：**即便是被引用上万次的经典结果，其证明细节也可能在近十年后被修补**——读论文时对"显然成立"的引理保持警惕，是研究者的基本素养。

**工程含义：渐近最优 $\neq$ 实践最优**。AO 是 $n \to \infty$ 的极限性质；在有限时间预算里，"渐近最优规划器 + 短超时"未必比"可行规划器 + 轨迹优化"质量更好。回顾上面的维度诅咒表：7-DOF 下 RRT* 要十万级采样才接近最优。所以 MoveIt2 默认 RRT-Connect 而非 RRT*——把"找最优"这件事交给 M08 的局部优化器在一条好的可行路径上做，往往比硬等 RRT* 全局收敛更快得到可执行的高质量轨迹（这正是 M07.4 末尾"RRT* 一定比 RRT-Connect 好"那条概念误区的理论根据）。

### Informed-RRT* (Gammell et al., IROS 2014) ⭐⭐⭐

在 RRT* 基础上加入 **M07.2 中的信息增益采样**：找到初始解后，只在椭球内采样。

$$
\text{收敛加速}: O(n^{-1/d}) \to O(n^{-1/d_{\text{eff}}})
$$

其中 $d_{\text{eff}} < d$ 是椭球的"有效维度"（椭球越扁，有效维度越低，收敛越快）。

### FMT*: 快速行进树 (Janson et al., IJRR 2015) ⭐⭐⭐

FMT* (Fast Marching Tree) 是介于 PRM* 和 RRT* 之间的"一次性"渐近最优规划器。其核心思想借鉴了 Fast Marching Method（水平集方法中的数值波前传播）。

**算法流程**:

```
FMT*(q_start, q_goal, N):
    1. 预采样 N 个随机构型 + q_start + q_goal
    2. 计算 r(N) 半径内的邻居图 (同 PRM*)
    3. 从 q_start 开始, 按"最低代价"顺序激活节点:
       - 对当前最低代价的活跃节点 z:
         - 遍历 z 的未激活邻居 x
         - 如果 cost(z) + c(z,x) < cost(x) 且 canConnect(z,x):
           x.parent = z, x.cost = cost(z) + c(z,x)
           标记 x 为活跃
    4. 当 q_goal 被激活时, 回溯提取路径
```

**与 RRT* 的关键区别**:
- RRT* 每次采样一个点，然后重连邻域。FMT* 先采样所有点，然后按代价顺序"激活"
- FMT* 的搜索过程类似于 Dijkstra 在预采样图上的运行——但不需要检查所有边，只检查"有希望"的边

**FMT* 的收敛优势**: 在相同采样数下，FMT* 的路径代价通常低于 RRT*，因为它利用了全局最佳优先顺序（而非 RRT* 的局部重连）。但 FMT* 需要预采样——不像 RRT 那样可以"边采样边搜索"。

> **跨领域类比**: FMT* 之于 RRT*，类似于 A* 之于贪心搜索——都是用全局最佳优先策略替代局部贪心策略。A* 用启发函数排序 open list，FMT* 用当前最低代价排序活跃节点。代价更高、结构更严谨，但路径质量显著提升。

### AIT*: 自适应信息化搜索 (Strub & Gammell, IJRR 2022) ⭐⭐⭐⭐

AIT* (Adaptively Informed Trees) 是 BIT* 的进化版本，由同一团队（Gammell Lab, Oxford）提出。

**AIT* 解决的问题**: BIT* 在每个批次中对所有边使用相同的启发函数，没有利用 RGG（随机几何图）中的**渐近最优启发**。AIT* 引入了**自适应启发**——随着搜索进展，启发函数自身也在改进。

**两层交替搜索**:

```
AIT*:
    维护两棵树: 正向搜索树 T_f (从 start), 反向搜索树 T_r (从 goal)

    while not converged:
        // 反向搜索: 从 goal 回溯, 改进启发函数
        在 T_r 上执行 RRT-style 扩展
        → 更新 h(v) = T_r 中从 v 到 goal 的最佳已知代价

        // 正向搜索: 用改进的启发函数指导 BIT*-style 批量搜索
        在 T_f 上执行 BIT* 的批量最佳优先搜索
        → 使用 f(v) = g(v) + h_improved(v) 排序

        // 关键: 反向树越大, 启发越准, 正向搜索越高效
```

**与 BIT* 的对比**:

| 维度 | BIT* | AIT* |
|------|------|------|
| 启发函数 | 固定（欧氏距离） | **自适应**（反向树不断改进） |
| 搜索方向 | 单向 | **双向**（正向搜索 + 反向启发改进） |
| 碰撞检测效率 | 中 | 更高（更好的启发 → 更少无用边检查） |
| 收敛速度 | 快 | **更快**（实验中快 2-5 倍） |
| 实现复杂度 | 中 | 高 |

AIT* 在 OMPL 中已有实现：`ompl::geometric::AITstar`。对于 7-DOF 臂的复杂场景，AIT* 通常比 BIT* 更快收敛到接近最优解。

> **反事实推理**: 如果启发函数始终是欧氏距离（BIT* 的做法）会怎样？当障碍物遮挡直线路径时，欧氏启发严重低估实际代价——搜索浪费大量时间探索被障碍阻断的方向。AIT* 的反向树逐步揭示"绕过障碍的实际代价"，让正向搜索直接跳过无望区域。这种"启发改进"的思路在 A* 研究中也有——ALT 算法用 landmark 预计算改进启发。

### BIT*: 批量信息化搜索 (Gammell et al., IJRR 2020) ⭐⭐⭐

BIT* (Batch Informed Trees) 是目前渐近最优采样规划器中收敛最快的之一（AIT* 是其直接后继，收敛更快但实现更复杂）。

**核心思想**: 将采样和搜索**交替批量执行**，而非像 RRT* 那样每次只加一个点。

```
BIT*:
    1. 初始批量采样 N₀ 个点
    2. 在当前采样点集上执行最佳优先搜索 (类似 A*)
       - 启发函数: h(q) = ||q - q_goal|| (距离到目标)
       - 搜索优先级: f(edge) = g(source) + c(edge) + h(target)
    3. 如果搜索完毕仍未找到（更优）解 → 增加一批新采样点
    4. 用 Informed 椭球约束新采样区域
    5. 重复 2-4
```

**为什么比 RRT* 快？**

| 特性 | RRT* | BIT* |
|------|------|------|
| 每次增加 | 1 个点 | 一批点 (100+) |
| 搜索方式 | 局部重连 | 全局最佳优先搜索 |
| 信息利用 | 低（只看邻域） | 高（启发式全局排序） |
| 冗余碰撞检测 | 多 | 少（启发式剪枝无望的边） |

> **跨领域类比**: RRT* vs BIT* 类似于**随机梯度下降 vs 小批量梯度下降**——单样本更新虽然每步快，但方向噪声大；小批量每步略慢但方向更准确，总收敛时间更短。

**Anytime（随时）性质——为什么这对工业落地至关重要**：RRT*、Informed-RRT*、BIT*、AIT* 都是 **anytime 算法**——它们一旦找到第一条可行解，之后每多给一点时间，返回的解就**单调不变差**（代价只减不增）。这个性质让规划器与 M07.8 的硬实时约束天然契合：

- **可被随时打断**：给 100 ms 就拿 100 ms 内的最优解，给 1 s 就拿更优的——不存在"必须跑完才有结果"。这与 RRT-Connect（找到即停、无优化）和精确算法（不跑完无解）形成对比。
- **首解时间 vs 收敛时间是两回事**：BIT* 的首条可行解出现时间通常略晚于 RRT-Connect（因为要先攒一批样本），但首解之后质量提升极快。工程上据此分流：实时回路（协作避人，250 ms 内必须动）优先 RRT-Connect 拿首解；离线/半离线（产线节拍允许 1-2 s）用 BIT*/AIT* 换更短路径。
- **代价单调性的来源**：anytime 性质本质上由 M07.2 的 Informed 椭球保证——每次刷新 $c_{\text{best}}$ 后，新采样被限制在更小的椭球内，搜索只可能发现更短的解，不可能"找到更长的解又接受它"。

> **本质洞察**：anytime 性质把"规划"从一个"要么成功要么失败的离散事件"变成了"质量随时间连续爬升的过程"。这让上层调度器可以做**时间-质量权衡**：在时间预算内随时取走当前最优，而不必赌一个固定的求解时间。这是采样规划相对精确算法（A* 在完整图上、必须跑完）在机器人实时场景中的一个隐性但关键的优势。

### 算法对比总表 ⭐⭐

| 算法 | 完备性 | 最优性 | 收敛率 | 单/双向 | 典型时间(7-DOF) | 路径质量 |
|------|--------|--------|--------|---------|----------------|---------|
| **RRT** | 概率完备 | ❌ | — | 单向 | ~50 ms | 差（锯齿） |
| **RRT-Connect** | 概率完备 | ❌ | — | 双向+贪心 | **~20 ms** | 差（锯齿） |
| **RRT*** | 概率完备 | **渐近最优** | $O(n^{-1/7})$ | 单向 | ~200 ms | 好（随时间改善） |
| **Informed-RRT*** | 概率完备 | **渐近最优** | 快于 RRT* | 单向 | ~150 ms | 好 |
| **FMT*** | 概率完备 | **渐近最优** | 快于 RRT* | 单向 | ~120 ms | 好（预采样排序） |
| **BIT*** | 概率完备 | **渐近最优** | 很快 | 单向 | ~80 ms | 很好 |
| **AIT*** | 概率完备 | **渐近最优** | **最快** | 双向(自适应) | ~60 ms | **最好** |

**选择指南速查**:
- 只需可行路径（后续有轨迹优化 M08）→ **RRT-Connect**（最快）
- 需要渐近最优 + 时间预算充足（>500ms）→ **AIT*** 或 **BIT***
- 需要渐近最优 + 时间预算紧张（<200ms）→ **Informed-RRT***
- 预采样可接受 + 需要高质量路径 → **FMT***

### ⚠️ 常见陷阱

**概念误区: 认为 RRT* 一定比 RRT-Connect 好**

```
💡 RRT* 是渐近最优的，但"渐近"意味着需要大量采样才能接近最优。
   在 7-DOF 空间中，RRT* 收敛到最优解可能需要数十万次采样（数十秒）。
   而 RRT-Connect 在 20ms 内就能给出可行路径。
   工程原则: 如果后续有轨迹优化（M08），RRT-Connect + 优化
   通常比等 RRT* 收敛更快得到高质量轨迹。
   不是 "RRT* 更先进所以更好"——是 "任务目标决定算法选择"。
```

**编程陷阱: RRT 步长 $\epsilon$ 的选择**

```
⚠️ 步长 ε 太大: 树的节点粗疏，可能跳过窄通道
   步长 ε 太小: 树的节点太密，近邻搜索开销大，探索速度慢
   OMPL 默认: 根据空间的 getMaximumExtent() 自动设定
   经验值: ε ≈ 0.05 * getMaximumExtent() 是合理起点
   自检: 观察树的节点数——如果规划用了 10000+ 节点仍未找到路径，
         ε 可能太小（探索太慢）或场景确实很难。
```

### 练习

1. **[编程]** 用 OMPL 在 $\mathbb{R}^2$（带 L 形窄通道障碍）中分别运行 RRT、RRT-Connect、RRT*、BIT*。对每种算法记录：求解时间、路径长度、采样次数。用箱线图展示 50 次运行的统计分布。
2. **[精读]** 阅读 OMPL 源码 `src/ompl/geometric/planners/rrt/RRTConnect.h` 和 `.cpp` 中的 `solve()` 方法（约 200 行）。标注：双树初始化、Extend 操作、Connect 操作、双树交替、路径提取——对应论文 Algorithm 1 的哪一步？
3. **[跨章综合]** 结合 M03（IK）和本章：如果目标不是关节角 $q_{\text{goal}}$ 而是末端位姿 $T_{\text{goal}} \in SE(3)$，如何用 RRT-Connect 规划？提示：OMPL 的 `GoalLazySamples` 可以在搜索过程中按需用 IK 生成目标构型。

---

## M07.5 OMPL C++ API 精读 ⭐⭐

### 动机：理解三层抽象的设计动机 ⭐⭐

OMPL 由 Rice University Kavraki Lab 开发（Sucan, Moll & Kavraki, RAM 2012），其核心设计哲学是**分离关注点**：

- **StateSpace**: 定义"构型是什么"（度量、拓扑、插值）
- **SpaceInformation**: 定义"构型是否合法"（碰撞检测、运动验证）
- **Planner**: 定义"怎么搜索"（算法）

这三层对应设计模式中的**策略模式(Strategy)**和**门面模式(Facade)**（参见 C++ 基础中的设计模式章节），让算法与碰撞检测完全解耦。

### 层 1: StateSpace 继承体系 ⭐⭐

```
                    StateSpace (虚基类)
                    ├── getDimension()
                    ├── distance(s1, s2) = 0        // 纯虚: 距离度量
                    ├── interpolate(from, to, t) = 0 // 纯虚: 插值
                    ├── enforceBounds(state) = 0     // 纯虚: 限位
                    ├── getMaximumExtent() = 0       // 纯虚: 空间直径
                    └── allocState() / freeState()   // 内存池
                          │
          ┌───────────────┼────────────────┐
          │               │                │
   RealVectorStateSpace  SO3StateSpace  CompoundStateSpace
   (R^n, 欧氏距离)       (四元数距离)    (多空间笛卡尔积)
          │
          └── 7-DOF 机械臂通常用这个
```

**StateSpace 的虚函数**要求每个派生类重写的核心方法：
- `distance(s1, s2)`: 两个状态间的度量
- `interpolate(from, to, t, result)`: 沿测地线插值
- `copyState(dest, src)`: 深拷贝状态
- `equalStates(s1, s2)`: 状态相等判断
- `getMaximumExtent()`: 空间直径（归一化用）
- `enforceBounds(state)`: 把状态裁剪到合法范围（关节限位）

**为什么不用 `std::vector<double>`？**

| 操作 | `std::vector<double>` | OMPL StateSpace |
|------|----------------------|-----------------|
| 距离 | 只能欧氏 | 虚函数分派到正确度量 |
| 插值 | 只能线性 | SO3 用 SLERP，SE3 用加权 |
| 边界 | 手动检查 | `enforceBounds` 自动 |
| 组合 | 手动索引偏移 | `CompoundStateSpace` 自动 |

### 完整 7-DOF 规划示例 ⭐⭐

```cpp
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <iostream>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// === 碰撞检测回调 (策略模式注入) ===
// 实际工程中: 此处调用 FCL/Coal + Pinocchio FK (参见 M04)
// 此处简化为构型空间中的球形障碍
bool isStateValid(const ob::State* state) {
    const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
    // 简化示意: 检查前两个关节是否在"禁区"
    double q0 = rv->values[0];
    double q1 = rv->values[1];
    double cx = 0.5, cy = 0.5, r = 0.3;
    if ((q0 - cx) * (q0 - cx) + (q1 - cy) * (q1 - cy) < r * r)
        return false;
    return true;
}

int main() {
    // === 1. 定义 StateSpace ===
    auto space = std::make_shared<ob::RealVectorStateSpace>(7);

    // 设置关节限位 (Franka Panda 实际值)
    ob::RealVectorBounds bounds(7);
    bounds.setLow(0, -2.8973);  bounds.setHigh(0, 2.8973);
    bounds.setLow(1, -1.7628);  bounds.setHigh(1, 1.7628);
    bounds.setLow(2, -2.8973);  bounds.setHigh(2, 2.8973);
    bounds.setLow(3, -3.0718);  bounds.setHigh(3, -0.0698);
    bounds.setLow(4, -2.8973);  bounds.setHigh(4, 2.8973);
    bounds.setLow(5, -0.0175);  bounds.setHigh(5, 3.7525);
    bounds.setLow(6, -2.8973);  bounds.setHigh(6, 2.8973);
    space->setBounds(bounds);

    // === 2. SimpleSetup (门面模式, 封装三层抽象) ===
    og::SimpleSetup ss(space);

    // 注入碰撞检测回调 (策略模式)
    ss.setStateValidityChecker(isStateValid);

    // === 3. 起点和终点 ===
    ob::ScopedState<> start(space);  // RAII: 自动释放
    start[0] = 0; start[1] = 0; start[2] = 0; start[3] = -1.5;
    start[4] = 0; start[5] = 1.8; start[6] = 0;

    ob::ScopedState<> goal(space);
    goal[0] = 1.0; goal[1] = 1.0; goal[2] = 0.5; goal[3] = -1.5;
    goal[4] = 0.5; goal[5] = 2.0; goal[6] = 0.3;

    ss.setStartAndGoalStates(start, goal);

    // === 4. 选择规划器 (一行切换! 策略模式的威力) ===
    // 切换算法: 改这一行即可, 其余代码完全不变
    auto planner = std::make_shared<og::RRTConnect>(
        ss.getSpaceInformation());
    // auto planner = std::make_shared<og::BITstar>(
    //     ss.getSpaceInformation());
    ss.setPlanner(planner);

    // === 5. 求解 ===
    ob::PlannerStatus status = ss.solve(5.0);  // 5 秒超时

    if (status == ob::PlannerStatus::EXACT_SOLUTION) {
        std::cout << "找到解! 路径节点数: "
                  << ss.getSolutionPath().getStateCount() << "\n";
        std::cout << "路径长度: "
                  << ss.getSolutionPath().length() << "\n";

        // 路径简化 (去除冗余节点, 平滑折线)
        ss.simplifySolution();
        std::cout << "简化后路径长度: "
                  << ss.getSolutionPath().length() << "\n";

        // 打印路径
        ss.getSolutionPath().print(std::cout);
    } else {
        std::cout << "规划失败: " << status << "\n";
    }
    return 0;
}
// 编译 (ROS2 环境):
// g++ -std=c++17 -O2 $(pkg-config --cflags ompl) \
//     ompl_panda.cpp -o ompl_panda $(pkg-config --libs ompl)
```

### 层 2: SpaceInformation 与碰撞检测注入 ⭐⭐

SpaceInformation 是连接 StateSpace 和 Planner 的**门面**：

```cpp
// SpaceInformation 的核心接口 (教学简化)
class SpaceInformation {
public:
    // 碰撞检测 (用户通过 StateValidityChecker 虚函数提供)
    bool isValid(const State* state) const {
        return stateValidityChecker_->isValid(state);
    }

    // 运动可行性检查 (两点之间离散化碰撞检测)
    bool checkMotion(const State* s1, const State* s2) const {
        return motionValidator_->checkMotion(s1, s2);
    }

    // 状态采样 (可替换为自定义采样策略)
    StateSamplerPtr allocStateSampler() const;
};
```

**碰撞检测的开销分析**:

回顾 M04（碰撞检测）：一次 FCL 碰撞检测对 7-DOF 臂约 50-500 us（取决于环境复杂度和碰撞模型精度）。

| 规划器 | 典型碰撞检测次数 | 碰撞检测总时间 | 占总规划时间比例 |
|--------|----------------|--------------|----------------|
| RRT-Connect | 1,000-5,000 | 50-250 ms | **80-95%** |
| RRT* | 5,000-50,000 | 250 ms-25 s | **90-98%** |
| PRM* (预处理) | 50,000-500,000 | 2.5-250 s | **95-99%** |

> **本质洞察**: **碰撞检测是运动规划的性能瓶颈**，而非规划算法本身。RRT-Connect 的树操作（最近邻搜索、节点插入）只占规划总时间的 5-20%。这就是为什么 M09（GPU加速规划）的核心不是加速算法逻辑，而是加速碰撞检测——用 GPU 并行（cuRobo）或 SIMD 向量化（VAMP）把每次碰撞检查从 50 us 降到 0.1 us。

#### 碰撞检测接口的两个层次：StateValidityChecker 与 MotionValidator ⭐⭐⭐

前面的完整示例里我们用 `ss.setStateValidityChecker(isStateValid)` 注入了一个**自由函数**，这是教学便利写法。生产代码几乎总是注入一个**继承 `ob::StateValidityChecker` 的对象**——因为它需要持有机器人模型、碰撞场景等状态，自由函数装不下。理解这一点是把"独立 OMPL 示例"升级为"对接 M04 碰撞管线"的关键。

OMPL 的合法性检查分为**两个互补的层次**，初学者最常见的误解是以为只有点检查：

| 层次 | 接口 | 回答的问题 | 调用频率 |
|------|------|-----------|---------|
| **点合法性** | `StateValidityChecker::isValid(state)` | 单个构型 $q$ 是否无碰撞、是否在限位内 | 每个采样点 1 次 |
| **运动合法性** | `MotionValidator::checkMotion(s1, s2)` | 从 $q_1$ 到 $q_2$ 的**整段直线运动**是否全程无碰撞 | 每条候选边 1 次（内部展开为多次点检查） |

```cpp
// === 生产级 StateValidityChecker: 对接 M04 碰撞管线 ===
// 对比教学示例的自由函数 isStateValid——这里持有机器人模型与碰撞场景
class ArmStateValidityChecker : public ob::StateValidityChecker {
public:
    ArmStateValidityChecker(const ob::SpaceInformationPtr& si,
                            std::shared_ptr<RobotModel> model,   // 来自 M01 URDF
                            std::shared_ptr<CollisionScene> scene) // 来自 M04
        : ob::StateValidityChecker(si), model_(model), scene_(scene) {}

    bool isValid(const ob::State* state) const override {
        const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Map<const Eigen::VectorXd> q(rv->values, 7);

        // 1) 限位检查由 SpaceInformation 的 bounds 负责, 这里聚焦碰撞
        // 2) FK: 关节角 → 各 link 位姿 (回顾 M01 Pinocchio FK)
        model_->computeForwardKinematics(q);
        // 3) 碰撞查询: 环境碰撞 + 自碰撞 (回顾 M04 的 BroadPhase→NarrowPhase)
        return !scene_->inCollision(*model_);   // 无碰撞返回 true
    }
private:
    std::shared_ptr<RobotModel> model_;
    std::shared_ptr<CollisionScene> scene_;
};

// 注入方式: 用 std::make_shared 构造对象, 而非传自由函数
auto si = ss.getSpaceInformation();
si->setStateValidityChecker(
    std::make_shared<ArmStateValidityChecker>(si, model, scene));
```

**MotionValidator 是 `longest_valid_segment_fraction` 的真正执行者**。M07.6 会讲这个参数，但它的机制就在这里：默认的 `DiscreteMotionValidator` 把一段运动 $q_1 \to q_2$ 按固定分辨率离散成 $k$ 个中间点，逐点调用 `isValid`：

```cpp
// DiscreteMotionValidator::checkMotion 的核心逻辑 (教学简化)
bool checkMotion(const ob::State* s1, const ob::State* s2) const {
    int nd = si_->getStateSpace()->validSegmentCount(s1, s2); // = ceil(dist/步长)
    ob::State* test = si_->allocState();
    for (int j = 1; j < nd; ++j) {
        // 在 s1→s2 上等距插值出第 j 个中间构型
        si_->getStateSpace()->interpolate(s1, s2, (double)j / nd, test);
        if (!si_->isValid(test)) {        // 任一中间点碰撞 → 整段运动非法
            si_->freeState(test);
            return false;
        }
    }
    si_->freeState(test);
    return true;
}
```

> **本质洞察**：采样规划器**从不"看见"障碍物的几何形状**——它对环境的全部认知，就是 `isValid` 返回的一串 true/false。规划算法（RRT/PRM）与碰撞几何（FCL/Coal）之间，靠 `StateValidityChecker` 这一个布尔接口彻底解耦。这就是策略模式（Strategy）的威力：换碰撞后端（FCL→Bullet→GPU）不动算法一行，换算法（RRT→BIT*）不动碰撞一行。M09 的全部加速工作，本质上都是在优化这个布尔函数的吞吐量。

> **反事实推理**：如果 OMPL 把碰撞几何直接写进规划算法（不做这层抽象）会怎样？每新增一种碰撞后端（如 GPU 版）就要重写 40+ 个规划器；每新增一个算法就要适配所有碰撞库——组合爆炸。正是这层布尔接口让 OMPL 的 40+ 规划器与任意碰撞实现自由组合，也让 VAMP/cuRobo 能以"替换 isValid 实现"的方式无侵入地接入。

**离散检查的固有风险——隧穿（tunneling）**：因为 `checkMotion` 只检查有限个中间点，如果障碍物比离散步长还薄，两个采样点恰好骑跨它两侧，中间所有采样点都落在障碍外——碰撞被漏检（M07.6 的反事实推理与故障排查手册"路径穿越障碍物"讲的就是这个）。彻底的解法是连续碰撞检测（Continuous Collision Detection, CCD），用扫掠体（swept volume）判断而非离散采样，OMPL 允许通过自定义 `MotionValidator` 接入——代价是更高的单次检查开销。

### 层 3: Planner 基类与策略模式 ⭐⭐

```cpp
// Planner 基类 (教学简化)
class Planner {
public:
    // 唯一需要 override 的核心方法
    virtual PlannerStatus solve(
        const PlannerTerminationCondition& ptc) = 0;

    // 通用接口
    void setProblemDefinition(ProblemDefinitionPtr pdef);
    PlannerData getPlannerData() const;  // 导出规划器内部数据
    void clear();   // 重置规划器状态
    void setup();   // 初始化
};
```

**开闭原则**: 新增算法只需继承 `Planner`，重写 `solve()` 即可——OMPL 核心代码不需要任何修改。OMPL 目前有 40+ 种规划器，全部通过这个接口统一。

### ScopedState RAII ⭐⭐

OMPL 的 State 用内存池分配，原始指针需要手动释放。`ScopedState` 是 RAII 封装：

```cpp
// ❌ 不安全: 异常路径内存泄漏
ob::State* s = space->allocState();
// ... 中间代码可能 throw 异常 ...
space->freeState(s);  // 异常时永远不会执行

// ✅ RAII (ScopedState): 析构时自动释放
ob::ScopedState<ob::RealVectorStateSpace> s(space);
s[0] = 1.0;  // operator[] 访问关节值
// 离开作用域时自动释放, 即使有异常也安全
```

这是 C++ 基础（RAII 相关章节）在机器人库中的直接应用。

### OMPL Benchmark 基础设施 ⭐⭐

OMPL 自带的 benchmark 工具允许系统对比不同规划器：

```cpp
#include <ompl/tools/benchmark/Benchmark.h>

ompl::tools::Benchmark b(ss, "Panda planning benchmark");
b.addPlanner(std::make_shared<og::RRTConnect>(si));
b.addPlanner(std::make_shared<og::RRTstar>(si));
b.addPlanner(std::make_shared<og::BITstar>(si));

ompl::tools::Benchmark::Request request;
request.maxTime = 10.0;    // 每次规划最大 10 秒
request.maxMem = 1024.0;   // 最大 1GB 内存
request.runCount = 50;     // 每个规划器跑 50 次

b.benchmark(request);
b.saveResultsToFile("benchmark.log");
// 用 ompl_benchmark_statistics.py 生成 PDF 报告
// (包含箱线图、成功率、路径长度、求解时间统计)
```

### ⚠️ 常见陷阱

**编程陷阱: StateValidityChecker 的线程安全**

```
⚠️ OMPL 的 CForest (并行 RRT*) 会在多线程中调用 isValid()。
   如果你的碰撞检测函数不是线程安全的（如使用 FCL 的共享
   BroadPhaseManager），会出现数据竞争 → undefined behavior。
   现象: 间歇性崩溃或错误的碰撞结果。
   正确做法: 每个线程使用独立的碰撞检测实例，
            或使用 thread_local 变量。
   MoveIt2 的做法: PlanningScene::clone() 为每个线程创建独立副本。
```

**概念误区: 认为 SimpleSetup 是 OMPL 的全部**

```
💡 SimpleSetup 是便捷封装（门面模式），但隐藏了很多可控参数。
   生产代码应直接操作 SpaceInformation + ProblemDefinition：
   - 自定义 MotionValidator（控制路径离散化精度）
   - 自定义 StateSampler（非均匀采样策略）
   - 多目标 GoalRegion（而非单点目标）
   SimpleSetup 适合教学和快速原型，生产环境需要更精细的控制。
```

### 练习

1. **[编程]** 为移动操作机器人（底盘 SE(2) + 7-DOF 臂）配置 `CompoundStateSpace = SE2StateSpace + RealVectorStateSpace(7)`。注意 SE(2) 中的角度需要正确的距离度量。用 RRT-Connect 规划一条底盘移动+臂运动的联合路径。
2. **[精读]** 阅读 OMPL `src/ompl/base/StateSpace.h` 中的纯虚函数声明。列出所有必须 override 的方法。如果你要为一个新型关节（如谐波关节，角度范围 $[0, 4\pi]$）实现自定义 StateSpace，需要 override 哪些方法？
3. **[Benchmark]** 用 OMPL Benchmark 框架，在 Panda 规划问题上对比 RRT-Connect / RRT* / BIT* / AIT* 四种算法，生成 PDF 报告。期望观察：RRT-Connect 最快但路径非最优，BIT* 路径最短但更慢。

---

## M07.6 MoveIt2 中的 OMPL 集成配置 ⭐⭐

### 动机：从独立 OMPL 到 MoveIt2 生产系统 ⭐⭐

独立使用 OMPL 需要手写碰撞检测、FK、关节限位。MoveIt2 通过 `ompl_interface` 包将这些全部封装，用户只需写 YAML 配置文件。

```
MoveIt2 OMPL 集成架构:

  MoveGroupInterface (用户 API)
       │
       ▼
  PlanningPipeline
       │
       ▼
  OMPLPlannerManager (pluginlib 插件)
       │
       ├── PlanningContext → OMPL SimpleSetup
       │    ├── StateSpace ← JointModelGroup 自动配置
       │    ├── StateValidityChecker ← PlanningScene 碰撞检测
       │    └── Planner ← ompl_planning.yaml 指定
       │
       └── PlanningScene
            ├── RobotState (FK, IK from M03)
            ├── WorldGeometry (环境模型)
            └── CollisionDetector (FCL/Bullet from M04)
```

**关键代码路径**: `moveit_planners/ompl/ompl_interface/src/ompl_interface.cpp`。

### ompl_planning.yaml 配置详解 ⭐⭐

```yaml
# panda_moveit_config/config/ompl_planning.yaml
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0           # 步长 (0 = 自动)

  RRTstarkConfigDefault:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05      # 向目标采样的概率
    delay_collision_checking: 1  # 延迟碰撞检查

  BITstarkConfigDefault:
    type: geometric::BITstar
    use_k_nearest: 1
    rewire_factor: 1.1
    samples_per_batch: 100  # 每批采样数

  PRMstarkConfigDefault:
    type: geometric::PRMstar
    max_nearest_neighbors: 10

panda_arm:
  # === 默认规划器 ===
  default_planner_config: RRTConnectkConfigDefault

  # === 全局参数 ===
  projection_evaluator: joints(panda_joint1, panda_joint2)
  longest_valid_segment_fraction: 0.005  # 关键! 见下文

  # === 本 planning group 可选的 planner config ID ===
  planner_configs:
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - BITstarkConfigDefault
    - PRMstarkConfigDefault
```

### longest_valid_segment_fraction 的关键性 ⭐⭐

这是 OMPL 中**最影响性能和安全的参数**：

$$
\text{离散化步长} = \text{longest\_valid\_segment\_fraction} \times \text{space.getMaximumExtent()}
$$

**含义**: 检查两个构型之间的运动是否碰撞时，按此步长离散化，逐点检测。

| 值 | 效果 | 碰撞检测次数 | 安全性 |
|----|------|-------------|--------|
| 0.05（太大） | 快但不安全 | 少 | 可能遗漏碰撞 |
| 0.005（默认） | 均衡 | 中 | 足够安全 |
| 0.001（太小） | 慢但非常安全 | 多 | 浪费计算 |

> **反事实推理**: 如果把 `longest_valid_segment_fraction` 设为 0.1（太大），两个无碰撞的构型之间的路径可能穿过一个薄壁障碍物而不被检测到——这在生产环境中是灾难性的。但设太小（0.0001）则碰撞检测次数暴增 50 倍，规划时间从 20ms 变成 1 秒。

### MoveIt2 链式规划管线 ⭐⭐

MoveIt2 Kilted (2025+) 支持多规划管线、请求/响应适配器和多 pipeline 规划。下面的 YAML 是**自定义调度器的合法伪配置**，只表达"OMPL 先给可行路径，再由优化器后处理"和"并行跑多个候选再选优"的意图；`chain` / `parallel_candidates` / `selection` 不是 MoveIt 原生稳定字段。落地时需要用目标发行版支持的 planning pipeline 配置、MoveItCpp 代码或自定义 adapter/orchestrator 实现。

```yaml
# pipeline_configs.yaml
planning_pipelines:
  pipeline_names: [ompl, stomp, chomp]

custom_planning_orchestrator:
  # 链式: OMPL 找路径 → STOMP 优化
  chains:
    ompl_then_stomp:
      stages:
        - pipeline: ompl
          planner_id: RRTConnectkConfigDefault
        - pipeline: stomp

  # 并行: 同时跑多个候选, 选最好的
  parallel_candidates:
    - name: fast_rrtconnect
      pipeline: ompl
      planner_id: RRTConnectkConfigDefault
    - name: optimizing_bitstar
      pipeline: ompl
      planner_id: BITstarkConfigDefault
    - name: chomp_refinement
      pipeline: chomp
  timeout: 5.0
  selection: shortest_path
```

这正是 M07→M08 的桥接点——M07 找到可行路径后，M08 提升路径质量。

### ⚠️ 常见陷阱

**编程陷阱: PlanningScene 的线程安全**

```
⚠️ MoveIt2 的 PlanningScene 不是线程安全的。
   如果使用 CForest（多线程 RRT*），MoveIt2 会自动
   为每个线程 clone PlanningScene。但如果你自定义
   StateValidityChecker 直接访问共享资源，必须加锁。
   正确做法: 遵循 MoveIt2 的 PlanningScene::clone() 机制，
            不要跨线程共享可变状态。
```

**思维陷阱: 过度调参**

```
🧠 新手常见行为: 对每个场景精调 OMPL 参数，追求 5ms 的提升。
   实际上: RRT-Connect 的默认参数在 90% 场景中足够好。
   真正影响性能的是:
   1. 碰撞检测效率 (M04/M09 的范畴)
   2. 规划器选择 (窄通道用 PRM*, 开阔用 RRT-Connect)
   3. longest_valid_segment_fraction (安全性与速度的权衡)
   先把这三个搞对，再考虑其他参数。
```

### 练习

1. **[工程]** 在 MoveIt2 中配置 Panda 的 `ompl_planning.yaml`，分别测试 RRT-Connect、BIT*、PRM* 在同一场景下的性能。用 `MoveGroupInterface::plan()` 记录规划时间和路径长度。
2. **[调参]** 对同一场景，逐步调大 `longest_valid_segment_fraction`（0.001 → 0.005 → 0.01 → 0.05）。记录规划时间和是否出现碰撞遗漏。画出安全性-效率曲线。
3. **[跨章综合]** 配置 MoveIt2 链式管线（OMPL → STOMP）。对比纯 RRT-Connect 和链式管线的路径质量（长度、光滑度）。这是 M07→M08 的实际桥接。

---

## M07.7 规划约束 ⭐⭐⭐

### 动机：路径不能只避碰，还要满足任务约束 ⭐⭐⭐

很多实际任务对路径有额外约束：

| 约束类型 | 示例 | 数学描述 |
|---------|------|---------|
| 关节限位 | $q_i \in [q_{\min,i}, q_{\max,i}]$ | Box constraint |
| 笛卡尔约束 | 末端保持水平（端水杯） | $R_z(q) = [0,0,1]^T$ |
| 定向约束 | 工具朝向固定（焊接） | $R(q) = R_{\text{target}}$ |
| 路径约束 | 末端沿直线移动 | $p(q) = p_{\text{start}} + t \cdot \Delta p$ |

### 如果不做约束规划会怎样 ⭐⭐⭐

例如倒水任务：如果不加"杯口朝上"约束，RRT-Connect 会找到一条虽然无碰撞但杯子翻转的路径——水洒了。

### OMPL 约束规划机制 ⭐⭐⭐

OMPL 通过**约束流形**（Constraint Manifold）处理等式约束：

$$
F(q) = 0, \quad F: \mathbb{R}^n \to \mathbb{R}^k
$$

约束后的自由空间是 $(n-k)$ 维流形。OMPL 提供三种方法在流形上采样和规划：

**方法 1: 投影法 (ProjectedStateSpace)**

```
采样 q_rand ∈ C
投影 q_proj = Project(q_rand) 使 F(q_proj) ≈ 0
    (用 Newton-Raphson: q ← q - J_F^† F(q), 类似 M03 的 IK 迭代)
```

**方法 2: Atlas 法 (AtlasStateSpace)**

用多个局部坐标卡(chart)覆盖约束流形。每个 chart 是流形的切平面近似。适用于高曲率流形。这里讨论的是 OMPL 的约束规划状态空间类型本身，具体可用 API 以你安装的 OMPL 版本为准，不把 AtlasStateSpace 绑定到某个“2.0”版本断言。

**方法 3: Tangent Bundle 法 (TangentBundleStateSpace)**

在切空间中做采样和规划，然后投影回流形。适用于约束维度较低的情况。

### 实现约束规划 ⭐⭐⭐

```cpp
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <Eigen/Dense>

// 自定义约束: 末端保持水平 (z 轴朝上)
class KeepLevelConstraint : public ob::Constraint {
public:
    KeepLevelConstraint(/* robot model */)
        : ob::Constraint(7, 2)  // 7 DOF, 2 个约束
        // (z_ee_x = 0 和 z_ee_y = 0, 即 z 轴的 x,y 分量为零)
    {}

    // 约束函数: F(q) = 0
    void function(const Eigen::Ref<const Eigen::VectorXd>& q,
                  Eigen::Ref<Eigen::VectorXd> out) const override {
        // FK 计算末端 z 轴方向
        Eigen::Vector3d z_ee = computeEndEffectorZAxis(q);
        out[0] = z_ee[0];  // x 分量应为 0
        out[1] = z_ee[1];  // y 分量应为 0
    }

    // 约束 Jacobian: dF/dq
    void jacobian(const Eigen::Ref<const Eigen::VectorXd>& q,
                  Eigen::Ref<Eigen::MatrixXd> out) const override {
        out = computeConstraintJacobian(q);  // 2×7 矩阵
    }
};

// 使用约束规划
auto space = std::make_shared<ob::RealVectorStateSpace>(7);
auto constraint = std::make_shared<KeepLevelConstraint>();
auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

// 其余与普通规划相同
og::SimpleSetup ss(csi);
ss.setStateValidityChecker(isStateValid);
// ...
```

### 三种约束规划方法的深入对比 ⭐⭐⭐

三种方法适用于不同的约束复杂度和流形曲率：

| 维度 | ProjectedStateSpace | AtlasStateSpace | TangentBundleStateSpace |
|------|--------------------|-----------------|-----------------------|
| **原理** | Newton-Raphson 投影到约束面 | 多个局部切平面(chart)覆盖流形 | 在切空间采样，投影回流形 |
| **适用约束维度** | 低 (1-3 个约束) | 中-高 (3-6 个约束) | 低 (1-2 个约束) |
| **流形曲率** | 低-中 | **高**（自动管理 chart 边界） | 低 |
| **实现复杂度** | 低（只需 `function` + `jacobian`） | 高（chart 管理、边界检测） | 中 |
| **碰撞检测开销** | 中（投影迭代中额外检查） | 高（chart 边界处需额外验证） | 中 |
| **OMPL 成熟度** | 最成熟（推荐首选） | 成熟（OMPL 1.6+） | 实验性 |
| **典型用例** | 保持末端水平（倒水） | 复杂笛卡尔路径约束（焊缝跟踪） | 简单定向约束 |

**ProjectedStateSpace 的详细配置**:

```cpp
// 约束规划完整配置 (OMPL 1.6+)
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// Step 1: 定义约束 (继承 ob::Constraint)
auto constraint = std::make_shared<KeepLevelConstraint>(robot_model);

// Step 2: 配置投影参数
constraint->setTolerance(1e-4);      // 投影容差 (默认 1e-3)
// 更紧的容差 → 更精确但投影更慢
// 焊接场景需要 1e-5; 倒水场景 1e-3 足够

constraint->setMaxIterations(50);    // Newton-Raphson 最大迭代 (默认 50)
// 如果流形曲率高, 可能需要增加到 100

// Step 3: 构建约束状态空间
auto css = std::make_shared<ob::ProjectedStateSpace>(
    ambient_space, constraint);

// Step 4: 配置离散化步长 (约束流形上更短)
auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
csi->setStateValidityCheckingResolution(0.001);
// 约束规划中步长应比无约束更小 (0.001 vs 0.005)
// 因为流形上的测地线可能偏离直线, 较大步长会离开流形

// Step 5: 选择规划器 (约束规划中 RRT-Connect 仍是首选)
auto planner = std::make_shared<og::RRTConnect>(csi);
planner->setRange(0.5);  // 约束规划中步长通常需要减小
// 原因: 大步长可能跨越流形的高曲率区域, 投影失败
```

**AtlasStateSpace 的使用场景**:

当约束流形曲率很高时（例如沿曲面焊缝规划），ProjectedStateSpace 的单次 Newton 投影可能发散。AtlasStateSpace 通过维护多个局部 chart 解决此问题：

```cpp
// AtlasStateSpace 示意 (高曲率约束流形)
auto atlas_css = std::make_shared<ob::AtlasStateSpace>(
    ambient_space, constraint);

// Atlas 特有参数
atlas_css->setExploration(0.5);     // 探索 vs 利用 chart 的平衡
atlas_css->setRho(1.0);            // chart 有效半径
atlas_css->setAlpha(M_PI / 8.0);   // chart 之间最大角度差
atlas_css->setEpsilon(0.05);       // chart 边界重叠宽度

// 每个 chart 是流形的一个局部切平面近似
// 当采样点离开当前 chart 的有效范围时, 自动创建新 chart
// chart 之间通过边界重叠保证连通性
```

### 投影失败的系统诊断 ⭐⭐⭐

约束规划在工程上最常见的报错就是 `constraint projection failed` 或规划长时间无解。它的根因往往不是"约束写错"，而是几个隐蔽的数值/几何问题。这里给一个可照着走的诊断顺序，对应回 M03 的 IK 数值经验——投影的 Newton-Raphson 迭代 $q \leftarrow q - J_F^\dagger F(q)$ 与 IK 的雅可比迭代是同一套数学。

| 检查顺序 | 症状 | 根因 | 处理 |
|---------|------|------|------|
| 1 | 投影几乎总失败，连起点都投不上 | 起点本身不满足约束（$F(q_{\text{start}}) \not\approx 0$） | 先把起点/终点投影到流形再传入；OMPL 要求端点已在流形上 |
| 2 | 投影偶发失败，集中在某些区域 | 约束雅可比 $J_F$ 在该处接近奇异（约束冗余或冲突） | 改用阻尼伪逆 $J_F^T(J_F J_F^T+\lambda^2 I)^{-1}$（同 M03.3 DLS），抑制奇异点发散 |
| 3 | 投影成功但路径频繁"掉出"流形 | 离散步长太大，跨越了流形高曲率区 | 减小 `setStateValidityCheckingResolution`（0.005→0.001）与规划器 `setRange` |
| 4 | 投影迭代不收敛（达到 maxIterations） | 容差过紧或流形曲率过高 | 放宽 `setTolerance`，或从 ProjectedStateSpace 换 AtlasStateSpace |
| 5 | 一切正常但成功率仍低 | 约束把 $\mathcal{C}_{\text{free}}$ 切成不连通的流形片 | 检查约束是否过强；考虑放松为不等式容差带（软约束） |

**编程陷阱（约束维度声明）**：

```
⚠️ ob::Constraint(ambientDim, coDim) 的第二参数 coDim 是约束方程的个数 k,
   不是流形维数 (n-k)。写反会导致 function()/jacobian() 的输出维度
   与 OMPL 期望不符。
   现象: 段错误, 或投影立即返回失败。
   正确做法: "末端保持水平"是 z 轴的 x,y 分量为零 → 2 个方程 → coDim=2,
            流形维数 = 7-2 = 5。构造时传 ob::Constraint(7, 2)。
```

> **本质洞察**：约束规划的投影迭代和 M03 的 IK 迭代是同一枚硬币的两面——IK 把"末端到达某位姿"写成 $F(q)=0$ 在一个点上求解，约束投影把"末端始终满足某关系"写成 $F(q)=0$ 在整条路径的每个采样点上反复求解。所以 M03 学到的一切数值技巧（阻尼、奇异规避、初值选择）在这里直接复用——这也是为什么约束规划被放在 M03/M04 之后讲。

### 与碰撞检测 (M04) 的交互: 性能影响分析 ⭐⭐

约束规划中碰撞检测的开销被放大：

| 因素 | 无约束 | 有约束 | 放大倍数 |
|------|--------|--------|---------|
| 每次采样的碰撞检测 | 1 次 | 1 + 投影迭代中每步 1 次 | 3-10x |
| 投影成功率 | 100% | 50-80% | 1.2-2x |
| 路径验证 | 标准步长 | 更短步长（流形曲率） | 2-5x |
| **总碰撞检测次数** | N | 6N-100N | **6-100x** |

> **本质洞察**: 约束规划的性能瓶颈不是流形投影的数学计算（~1 us），而是**投影过程中需要额外碰撞检测**来验证投影后的构型仍然无碰撞。这也是 M09（GPU加速）在约束规划场景中价值更大的原因。

### ⚠️ 常见陷阱

**编程陷阱: 约束 Jacobian 的数值稳定性**

```
⚠️ 约束的 Jacobian 如果接近奇异（约束冲突或冗余），
   投影的 Newton-Raphson 迭代会发散。
   现象: 规划器报告 "constraint projection failed"。
   根本原因: 约束之间线性相关，或当前构型恰好在约束奇异点。
   正确做法: 使用带阻尼的伪逆进行投影
   (类似 M03.3 中 IK 的阻尼最小二乘: J^T(JJ^T + λ²I)^{-1})。
```

### 练习

1. **[编程]** 实现一个"末端保持水平"的约束，用 OMPL 的 `ProjectedStateSpace` 进行约束规划。可视化路径中末端朝向的变化——应始终保持水平。
2. **[思考]** 在焊接任务中，工具需要沿焊缝做匀速直线运动。这需要什么样的约束？OMPL 的状态约束（每个构型独立满足）能否表达路径约束（相邻构型之间的关系）？

---

## M07.8 工业场景调参指南 ⭐⭐

### 动机：从论文基准到真实产线 ⭐⭐

学术论文中的 OMPL benchmark 通常在简单场景（几个 box 障碍）中进行。工业场景的挑战完全不同：

| 学术 vs 工业 | 学术基准 | 工业现场 |
|-------------|---------|---------|
| 障碍物数量 | 3-10 个 | 50-500+ 个 |
| 环境变化 | 静态 | 动态（传送带、人员） |
| 成功率要求 | 90% | **99.9%+** |
| 规划时间限制 | 无限制 | **< 200 ms** |
| 碰撞检测精度 | 粗略 | 严格（安全裕度 5mm+） |
| 安全认证 | 无 | CE/ISO 13849 |

### 场景分类与规划器选型决策流程 ⭐⭐

```
                ┌───────────────────────────────────────────┐
                │            场景诊断                        │
                └──────────────────┬────────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    ▼                              ▼
              环境静态?                         环境动态?
                    │                              │
          ┌────────┴────────┐              ┌──────┴──────┐
          ▼                  ▼              ▼              ▼
     多次查询?          单次查询?      需要实时?      可离线?
          │                  │              │              │
          ▼                  ▼              ▼              ▼
    PRM* (预计算)     RRT-Connect    cuRobo(M09)    RRT-Connect
    + A* 查询         + STOMP优化     GPU重规划       + STOMP
```

### 关键调参参数一览 ⭐⭐

| 参数 | 功能 | 默认值 | 调整方向 |
|------|------|--------|---------|
| `range` (RRT*) | 树扩展步长 | 自动 | 增大→快但粗糙；减小→慢但精细 |
| `goal_bias` (RRT) | 朝目标采样概率 | 0.05 | 开阔空间可增到 0.1-0.2 |
| `longest_valid_segment_fraction` | 路径碰撞检测精度 | 0.005 | 安全关键场景降到 0.001 |
| `samples_per_batch` (BIT*) | 每批采样数 | 100 | 增大→每批更全但更慢 |
| `rewire_factor` (RRT*) | 重连半径倍率 | 1.1 | 增大→更优路径但更慢 |
| 超时时间 | 最大规划时间 | 5.0 s | 工业: 0.1-0.5 s |

### 性能优化检查清单 ⭐⭐

| 优化项 | 预期提速 | 方法 |
|--------|---------|------|
| 碰撞检测加速 | 2-10x | 用 Bullet 替代 FCL（BroadPhase 更快）；简化碰撞模型 |
| 碰撞模型简化 | 2-5x | 用球体/胶囊体近似代替 mesh |
| 规划器选择 | 1.5-3x | 开阔空间用 RRT-Connect；窄通道用 PRM* |
| 安全裕度调优 | 1.5-2x | 适当调整 padding 减少碰撞检测的假阳性 |
| 硬件加速 (M09) | 视模型和场景而定 | cuRobo 可替代部分全栈运动生成；VAMP 需显式桥接碰撞/FK，不能假设自动加速所有 OMPL 规划器 |

### 调参案例: 焊接机器人 ⭐⭐

```
场景: UR10 焊接机器人, 在汽车车身上执行 20 个焊点
挑战: 窄通道(车身内部), 定向约束(焊枪朝向), 99.9% 成功率

初始配置:
  规划器: RRT-Connect (默认)
  结果: 成功率 85%, 平均 350ms

优化 Step 1: 切换到 PRM* (环境静态, 多次查询)
  结果: 预计算 30s, 查询 5ms, 成功率 95%

优化 Step 2: 加入桥采样 (窄通道, 参见 M07.2)
  结果: 成功率 99.2%

优化 Step 3: 碰撞模型简化 (mesh → 凸包, 参见 M04)
  结果: 碰撞检测 5x 加速, 查询 1ms

优化 Step 4: 增加路标数 (10000 → 50000)
  结果: 成功率 99.95%

最终: 预计算 2min, 查询 1ms, 成功率 99.95%
```

### 调参案例: 电子装配 (SMT 贴片) ⭐⭐

```
场景: SCARA + 7-DOF 辅助臂, PCB 板上精密贴装
挑战: 极高精度(±0.05mm), 高速节拍(0.8s/件), 密集零件间避碰
安全要求: 零碰撞 (ESD 敏感元件)

初始配置:
  规划器: RRT-Connect (默认)
  结果: 成功率 92%, 平均 180ms, 精度满足

优化 Step 1: 碰撞模型用球体+胶囊体混合 (回顾 M04)
  - 已贴装元件用球体近似 (快速)
  - 工具头用胶囊体 (更紧密)
  结果: 碰撞检测 3x 加速, 规划 60ms

优化 Step 2: 利用任务对称性预计算
  - PCB 上的贴装位置固定 → 预计算 PRM* 路标图
  - 每种元件类型一张路标图 (不同工具头形状)
  结果: 预计算 45s/元件类型, 查询 2ms

优化 Step 3: goal_bias 调高到 0.15
  - 贴装任务的起点-终点通常无大障碍遮挡
  - 更高 goal_bias 加速收敛
  结果: 查询 0.8ms

最终: 预计算 45s, 查询 0.8ms, 成功率 99.8%
```

### 调参案例: 协作机器人避人 ⭐⭐

```
场景: UR10e 协作机器人, 与人共享工作空间
挑战: 人员位置动态变化, ISO/TS 15066 要求 250ms 避让反应
安全要求: 实时避障, 速度/力矩限制

初始配置:
  规划器: RRT-Connect + STOMP (链式)
  碰撞检测: FCL + OctoMap (深度相机输入)
  结果: 总延迟 350ms (超出 250ms 要求)

优化 Step 1: 碰撞检测换 Bullet (BroadPhase 更快)
  结果: 碰撞检测 2x 加速, 总延迟 250ms (刚好及格)

优化 Step 2: 缩短 OMPL 规划时间上限
  - 从 5s → 0.1s, 配合 goal_bias=0.1
  - 路径质量下降可接受 (后续有 STOMP)
  结果: OMPL 阶段 30ms, 总延迟 180ms

优化 Step 3: 降低 longest_valid_segment_fraction 到 0.003
  - 提高安全性 (人体碰撞必须零容忍)
  - 碰撞检测增加 50%, 但 Bullet 够快
  结果: 总延迟 200ms, 安全性更高

最终: 总延迟 200ms < 250ms, 满足 ISO/TS 15066

注意: 如果需要进一步压缩到 <50ms,
      考虑 GPU 加速 (M09 cuRobo + nvblox)
```

### 工业规划器参数调优的系统方法论 ⭐⭐

调参不应是随机尝试。以下是系统方法：

**Step 1: 诊断瓶颈**

```
用 OMPL Benchmark 跑 50 次, 收集:
  - 平均/中位/P99 规划时间
  - 成功率
  - 碰撞检测调用次数
  - 路径长度 (与直线距离的比值)

关注: 如果成功率 < 95% → 采样策略问题
     如果时间 > 要求 → 碰撞检测效率问题
     如果路径太长 → 需要渐近最优规划器或后续优化
```

**Step 2: 按优先级调参**

| 优先级 | 参数 | 何时调 |
|--------|------|--------|
| 1 | 规划器选择 | 成功率不足 / 路径质量不满足 |
| 2 | `longest_valid_segment_fraction` | 安全性 vs 速度权衡 |
| 3 | 碰撞检测后端 (FCL→Bullet→GPU) | 碰撞检测是瓶颈 |
| 4 | `goal_bias` | 开阔空间可适当增加 |
| 5 | `range`（步长） | 探索效率 vs 精细度 |
| 6 | 采样策略 (均匀→桥→高斯) | 窄通道导致成功率低 |

**Step 3: 验证**

```
每次只调一个参数, 跑 50 次 Benchmark:
  - 性能提升 → 保留
  - 无变化或下降 → 回退
  - 成功率下降 → 立即回退 (安全优先)
```

### ⚠️ 常见陷阱

**思维陷阱: 把规划失败归咎于算法**

```
🧠 规划失败的最常见原因不是算法不好，而是:
   1. 目标构型不可达 (在 C_obs 中) → 先检查 IK 可行性 (M03)
   2. 碰撞模型过大 (过度 padding) → 导致 C_free 中无路径存在
   3. 关节限位过紧 → 实际有路径但被限位排除
   4. 起始构型已碰撞 → 规划器拒绝启动
   诊断顺序: 检查起点/终点可行性 → 检查碰撞模型 → 检查限位 → 换算法
```

### 练习

1. **[工程]** 对一个含 10 个障碍 box 的桌面抓取场景，分别用 RRT-Connect 和 PRM* 规划 100 次不同的起点-终点对。记录预计算时间、查询时间、成功率。分析在多少次查询后 PRM* 的总时间优于 RRT-Connect。
2. **[调参]** 在窄通道场景中，逐步增大 `goal_bias`（0.01 → 0.05 → 0.1 → 0.2 → 0.5）。记录成功率和规划时间。观察 goal_bias 过大时反而降低成功率的现象（目标偏置太强，探索性不足）。

3. **[跨章综合]** 综合 M03（IK 求解器）+ M04（碰撞检测）+ M07（本章），设计一个完整的"目标位姿 → IK → 碰撞检测 → 采样规划"管线：(1) 给定末端目标位姿，用 M03 的 IK 求解器计算多个关节角解；(2) 用 M04 的碰撞检测管线筛选无碰撞解；(3) 用 M07 的 OMPL RRT-Connect 规划从当前构型到目标构型的路径。画出数据流图并标注每个环节的典型延迟。讨论：如果所有 IK 解都碰撞怎么办？

---

## M07.8b 前沿进展：BIT*、AIT* 与 VAMP 最新进展 ⭐⭐⭐⭐

### BIT* 的工程成熟化

BIT*（Batch Informed Trees, Gammell et al., IJRR 2020）在 OMPL 1.6+ 中已完全集成，成为渐近最优规划的推荐默认选项。BIT* 的核心洞察是将图搜索和采样结合——在每一批次中，BIT* 在当前最优解的启发式椭球内批量采样，然后用类似 A* 的边展开顺序处理这些样本。这使得 BIT* 同时拥有采样规划的全局探索能力和图搜索的高效边展开策略。

截至 2026 年，BIT* 已在 MoveIt2 中可通过 `ompl_planning.yaml` 直接配置，是工业场景中路径质量要求高时的首选。

### AIT*：自适应启发式搜索

AIT*（Adaptively Informed Trees, Strub & Gammell, IJRR 2022）是 BIT* 的演化版本，引入了**自适应启发函数**。BIT* 使用固定的椭球启发函数（基于起点到终点的直线距离），而 AIT* 在搜索过程中动态更新启发函数——用已发现的"反向搜索树"提供更精确的 cost-to-go 估计。这使得 AIT* 在复杂环境（多个窄通道、障碍物密集）中比 BIT* 收敛更快。

> **本质洞察**：从 RRT* → Informed-RRT* → BIT* → AIT* 的演进反映了一个核心趋势——采样规划正在吸收图搜索的优点。RRT* 是纯采样的，BIT* 加入了批量图搜索，AIT* 进一步加入了自适应启发函数。最终目标是在构型空间中实现"用采样处理高维、用搜索处理最优性"的最佳组合。

### VAMP：GPU/SIMD 级别的采样加速（M09 预读）

VAMP（Vectorized Accelerated Motion Planning, Thomason et al., ICRA 2024, RSS 2024）代表了采样规划加速的全新维度——不是改进算法本身，而是用 SIMD 指令（AVX2/NEON）将 FK 和碰撞检测加速 100-1000 倍。VAMP 在 Panda 7-DOF 上实现了 35 微秒的 PRM* 规划——这意味着可以在每个控制周期（1 ms）内完成多次重规划。

VAMP 的 2024 年后续工作 VAMP-MR（RSS 2024）进一步扩展到多臂协同规划，使用 CAPT（Conflict-Aware Parallel Trees）数据结构在复合构型空间中高效搜索。详细内容见 M09。

---

## 本章常见误解汇总

下表汇总全章散落的高频误区，集中纠正。左列是初学者脑中常见的错误心智模型，右列是经过本章理论支撑的正确理解——建议读完全章后回头逐条自检。

| # | 常见误解 | 正确理解 | 出处 |
|---|---------|---------|------|
| 1 | "末端走直线绕过障碍就行，何必去构型空间" | 机械臂每段 link 都会碰撞，工作空间的"近"不等于 C-space 的"近"；必须在关节空间规划 | M07.1 |
| 2 | "构型空间可以像 2D 地图一样预计算成栅格" | 7-DOF 栅格化需 12.5 TB、160 年碰撞检测——维度灾难使其不可行，只能采样 | M07.1 |
| 3 | "采样越多结果一定越好" | 窄通道中均匀采样近乎无效（命中概率随通道宽度指数衰减），是策略问题而非数量问题 | M07.2 |
| 4 | "概率完备就是采样够多必能找到解" | 仅在"解存在且 $\delta$-鲁棒"时成立；无解时永远找不到也无法判定无解 | M07.4 插曲 |
| 5 | "RRT 收敛慢但终究会到最优" | RRT 找到最优解的概率**恰为零**——树只增不改，早期错误永久锁死 | M07.4 渐近最优 |
| 6 | "RRT* 比 RRT-Connect 先进所以更好" | AO 是 $n\to\infty$ 极限；有限时间内 RRT-Connect + 轨迹优化常更快得到高质量轨迹 | M07.4 |
| 7 | "OMPL 规划器自己知道障碍物在哪" | 规划器只通过 `isValid` 的布尔返回认知环境，与碰撞几何彻底解耦 | M07.5 |
| 8 | "碰撞检测只是检查采样点" | 还有 `MotionValidator` 检查整段运动；离散步长过大会漏检薄壁（隧穿） | M07.5 |
| 9 | "SimpleSetup 就是 OMPL 的全部" | 它是门面便利封装；生产需直接操作 SpaceInformation/ProblemDefinition 控制采样器、MotionValidator、多目标 | M07.5 |
| 10 | "规划失败 = 算法不行，换个算法" | 最常见原因是目标不可达/碰撞模型过大/限位过紧/起点已碰撞——先查可行性 | M07.8 |
| 11 | "约束规划慢是因为流形投影的数学计算贵" | 投影本身约 1 us，瓶颈是投影迭代中**反复的碰撞检测**（放大 6-100x） | M07.7 |
| 12 | "调参就是把每个参数都试一遍" | 90% 场景 RRT-Connect 默认值够好；优先级是规划器选型 > 安全步长 > 碰撞后端 | M07.8 |

---

## M07.9 本章小结

| 知识点 | 核心要点 | 工程价值 |
|--------|---------|---------|
| 构型空间 (M07.1) | 在关节角空间而非工作空间规划 | 正确形式化运动规划问题 |
| 采样策略 (M07.2) | 均匀/桥/高斯/信息增益各有适用场景 | 针对窄通道选桥采样 |
| PRM/PRM* (M07.3) | 预计算路标图，多次查询 | 固定环境产线 |
| RRT/RRT-Connect (M07.4) | 单次快速可行路径 | MoveIt2 默认，最常用 |
| RRT*/Informed-RRT*/BIT* (M07.4) | 渐近最优，需更多时间 | 路径质量要求高的场景 |
| OMPL 三层抽象 (M07.5) | StateSpace/SpaceInformation/Planner | 策略模式一行切换算法 |
| 碰撞检测瓶颈 (M07.5) | 占规划时间 80-95% | GPU 加速的根本动机 (M09) |
| MoveIt2 集成 (M07.6) | ompl_planning.yaml 配置 | 生产环境标准做法 |
| 规划约束 (M07.7) | 约束流形上的采样和规划 | 焊接/装配等任务约束 |
| 工业调参 (M07.8) | 场景驱动的规划器选型与参数调优 | 从学术到产线的最后一步 |

### 术语速查表

本章新引入术语的中英对照与一句话定义（按首次出现顺序）：

| 术语 | 英文 | 一句话定义 |
|------|------|----------|
| 构型空间 | Configuration Space (C-space) | 所有关节角组合构成的空间，运动规划在此进行而非工作空间 |
| 自由空间 | $\mathcal{C}_{\text{free}}$ | 构型空间中无碰撞构型的集合，$\mathcal{C} \setminus \mathcal{C}_{\text{obs}}$ |
| 维度灾难 | Curse of Dimensionality | 空间体积随维度指数增长，使栅格化与穷举不可行 |
| 概率完备性 | Probabilistic Completeness | 若解存在，采样数趋于无穷时找到解的概率趋于 1 |
| $\delta$-鲁棒可解 | $\delta$-robustly feasible | 存在有正宽度（半径 $\delta$）安全走廊的解，概率完备性的前提 |
| 渐近最优 | Asymptotic Optimality (AO) | 返回路径代价几乎必然收敛到全局最优代价 |
| 几乎必然收敛 | almost sure convergence | 几乎每条采样序列自身都收敛，强于概率收敛 |
| Voronoi 偏置 | Voronoi bias | RRT 中未探索区域因 Voronoi 区域大而更易被采样，驱动树向外扩展 |
| 重连 | Rewire | RRT* 中用更短连接替换已有边，使树结构可改写，是 AO 的关键 |
| 信息增益采样 | Informed Sampling | 找到初始解后只在以起终点为焦点的椭球内采样 |
| 随机几何图 | Random Geometric Graph (RGG) | 随机点按距离阈值连边的图，PRM*/RRT* 半径理论的基础 |
| 约束流形 | Constraint Manifold | 等式约束 $F(q)=0$ 定义的 $(n-k)$ 维子流形 |
| 隧穿 | Tunneling | 离散运动检查中薄障碍被两侧采样点骑跨而漏检的现象 |
| 路标图 | Roadmap | PRM 预计算的图，节点为无碰撞构型、边为可连接关系 |

### 知识点总表

| 编号 | 知识点 | 核心要点 | 对应节 | 难度 |
|------|--------|---------|--------|------|
| K1 | 构型空间 | 在关节空间规划；C-space 障碍是工作空间障碍的复杂高维映射 | M07.1 | ⭐ |
| K2 | 不可显式构造 | 维度灾难使栅格化不可行，必须随机采样探索 | M07.1 | ⭐⭐ |
| K3 | 采样策略 | 均匀/桥/高斯/信息增益各有适用场景，按场景特征选 | M07.2 | ⭐⭐ |
| K4 | PRM/PRM* | 预计算路标图供多次查询；PRM* 自适应半径达渐近最优；r-disc 与 k-nearest 两种连接策略量级一致（$\Theta(\log n)$） | M07.3 | ⭐⭐ |
| K5 | RRT/RRT-Connect | 单次搜索找可行路径，双向+贪心连接是 MoveIt2 默认 | M07.4 | ⭐⭐ |
| K6 | 概率完备性 | 形式化定义 + $\delta$-鲁棒前提 + 几何级数证明骨架 | M07.4 | ⭐⭐⭐ |
| K7 | 渐近最优性 | RRT 几乎必然次优（$\Pr=0$ 最优），RRT* 靠 Rewire 修复 | M07.4 | ⭐⭐⭐ |
| K8 | Informed/FMT*/BIT*/AIT* | 用搜索策略降低等效维度，加速收敛 | M07.4 | ⭐⭐⭐ |
| K9 | OMPL 三层抽象 | StateSpace/SpaceInformation/Planner，策略模式一行切换 | M07.5 | ⭐⭐ |
| K10 | 碰撞检测接口 | StateValidityChecker（点）+ MotionValidator（运动）双层 | M07.5 | ⭐⭐⭐ |
| K11 | 碰撞检测瓶颈 | 占规划时间 80-95%，是所有加速工作的核心 | M07.5 | ⭐⭐ |
| K12 | MoveIt2 集成 | ompl_planning.yaml 配置；longest_valid_segment 控安全 | M07.6 | ⭐⭐ |
| K13 | 约束规划 | 投影/Atlas/切丛三法在约束流形上采样 | M07.7 | ⭐⭐⭐ |
| K14 | 工业调参 | 场景驱动选型 + 系统化三步调参方法论 | M07.8 | ⭐⭐ |

---

## 累积项目：本章新增模块

**机械臂全栈项目进度**:
- M01: 加载 URDF → Pinocchio Model
- M03: IK 求解器 (opw/TRAC-IK/pick-ik)
- M04: 碰撞检测管线 (FCL/Coal)
- **M07 (本章新增): OMPL 运动规划模块**
  - 配置 `RealVectorStateSpace(7)` + Panda 关节限位
  - 实现 `StateValidityChecker`（对接 M04 碰撞检测管线）
  - 集成 RRT-Connect + BIT* 两种规划器
  - 输出：关节空间路径 $[q_0, q_1, \ldots, q_T]$

**下一步 (M08)**: 将 M07 的锯齿路径输入轨迹优化器（CHOMP/TrajOpt/STOMP），获得平滑可执行轨迹。

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| LaValle, *Planning Algorithms* (2006) | ⭐⭐ | 运动规划圣经，免费在线 lavalle.pl/planning/ |
| Sucan et al., "The Open Motion Planning Library", RAM 2012 | ⭐⭐ | OMPL 架构论文 |
| Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning", IJRR 2011 | ⭐⭐⭐ | RRT*/PRM* 理论基础 |
| Gammell et al., "Batch Informed Trees (BIT*)", IJRR 2020 | ⭐⭐⭐ | BIT* 完整理论与实验 |
| Strub & Gammell, "Adaptively Informed Trees (AIT*)", IJRR 2022 | ⭐⭐⭐ | AIT* 自适应启发搜索 |
| Janson et al., "Fast Marching Tree (FMT*)", IJRR 2015 | ⭐⭐⭐ | 预采样最佳优先规划 |
| Kuffner & LaValle, "RRT-Connect", ICRA 2000 | ⭐⭐ | 双向 RRT 原始论文 |
| OMPL 官网 ompl.kavrakilab.org | ⭐ | 教程、API 文档、示例 |
| Thomason et al., "Motions in Microseconds via Vectorized Sampling-Based Planning", ICRA 2024 | ⭐⭐⭐ | VAMP: SIMD 加速采样规划 (M09 预读) |
| MoveIt2 文档 moveit.picknik.ai | ⭐⭐ | OMPL 集成配置指南 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 规划返回 FAILURE | 起点/终点在 $\mathcal{C}_{\text{obs}}$ 中 | 1. 打印 `isValid(start)` 和 `isValid(goal)` 2. 检查碰撞模型 padding 3. 检查关节限位 | M04 |
| 路径穿越障碍物 | `longest_valid_segment_fraction` 过大 | 1. 降低到 0.001 2. 增加碰撞模型 padding 3. 启用连续碰撞检测 (CCD, 参见 M04) | M04 |
| 规划时间 > 5 秒 | 碰撞检测过慢 / 场景过复杂 | 1. 简化碰撞模型 (mesh→凸包→球体) 2. 换 Bullet 后端 3. 考虑 GPU 加速 (M09) | M04, M09 |
| PRM 预计算极慢 | 采样点太多 / 碰撞检测太慢 | 1. 减少路标数 2. 增大连接半径 3. 用 Lazy-PRM | — |
| 约束规划频繁失败 | 约束 Jacobian 奇异 | 1. 检查约束是否冗余 2. 用阻尼投影 3. 增大约束容差 | M07.7 |
| MoveIt2 规划器不加载 | pluginlib 配置错误 | 1. 检查 `ompl_planning.yaml` 中 `type` 拼写 2. 确认 `moveit_planners_ompl` 包已安装 3. 查看 ROS2 日志 | M14 |
| CForest 崩溃/竞态 | StateValidityChecker 非线程安全 | 1. 检查碰撞检测是否使用共享可变状态 2. 改用 PlanningScene::clone() 3. 使用 thread_local | M07.5 |

---

**向后指向**: 本章讲的采样规划器找到的是**几何路径**——一系列关节角的离散序列。这条路径通常质量很差（锯齿、绕路、不光滑）。M08（轨迹优化规划器）将接手这条粗糙路径，用 CHOMP/TrajOpt/STOMP 等优化方法将其变成**平滑、无碰撞、符合动力学约束**的可执行轨迹。采样规划与轨迹优化不是替代关系，而是互补关系：采样规划的价值在于**找到拓扑上正确的可行解**（绕过障碍物的哪一侧），为优化器提供一个好的初始猜测——没有这个初始猜测，优化器可能陷入错误的局部最优（绕障碍物错误的一侧）甚至发散。

---

## API 速查表

本章涉及的 OMPL 核心 API 签名与说明（命名空间 `ob = ompl::base`，`og = ompl::geometric`，`ot = ompl::tools`）。实际签名以你安装的 OMPL 版本头文件为准。

**状态空间与边界**

| API | 说明 |
|-----|------|
| `ob::RealVectorStateSpace(dim)` | $\mathbb{R}^n$ 关节空间，最常用于机械臂 |
| `ob::RealVectorBounds(dim)` + `setLow/setHigh(i, v)` | 设置每个关节的限位 |
| `space->setBounds(bounds)` | 把限位绑定到状态空间 |
| `ob::SO2StateSpace` / `ob::SO3StateSpace` | 连续旋转关节（自动处理周期）/ 三维旋转（四元数 SLERP 插值） |
| `ob::CompoundStateSpace` | 多空间笛卡尔积，如底盘 SE(2) + 7-DOF 臂 |
| `space->distance(s1,s2)` / `interpolate(from,to,t,out)` / `enforceBounds(s)` | 度量 / 插值 / 限位裁剪（派生类须 override） |
| `space->getMaximumExtent()` | 空间直径，用于归一化与默认步长 |

**空间信息与合法性检查**

| API | 说明 |
|-----|------|
| `ss.getSpaceInformation()` | 获取 `SpaceInformationPtr` |
| `si->setStateValidityChecker(fn 或 对象)` | 注入点碰撞检测（自由函数或继承 `ob::StateValidityChecker` 的对象） |
| `si->isValid(state)` | 查询单个构型是否合法 |
| `si->checkMotion(s1,s2)` | 查询整段运动是否合法（内部走 `MotionValidator`） |
| `si->setStateValidityCheckingResolution(r)` | 设置运动离散化分辨率（相对值） |
| `si->setMotionValidator(...)` | 替换为自定义运动检查器（如 CCD 连续检测） |

**问题定义与求解**

| API | 说明 |
|-----|------|
| `og::SimpleSetup ss(space)` | 门面封装三层抽象，适合教学/原型 |
| `ob::ScopedState<> s(space)` + `s[i]` | RAII 状态封装，`operator[]` 访问关节值 |
| `ss.setStartAndGoalStates(start, goal)` | 设置起点终点 |
| `ss.setPlanner(planner)` | 注入规划器（策略模式，一行切换算法） |
| `ss.solve(timeout)` → `ob::PlannerStatus` | 求解，返回 `EXACT_SOLUTION` / `APPROXIMATE_SOLUTION` / 失败 |
| `ss.getSolutionPath()` + `.length()` / `.getStateCount()` / `.print()` | 取解路径及其长度、节点数 |
| `ss.simplifySolution()` | 路径简化（去冗余、平滑折线） |
| `ob::GoalLazySamples` | 搜索过程中按需用 IK 生成目标构型（位姿目标，见 M07.4 练习 3） |

**规划器**

| API | 说明 |
|-----|------|
| `og::RRTConnect(si)` | 双向+贪心，MoveIt2 默认，最快可行解 |
| `og::RRTstar(si)` / `og::InformedRRTstar(si)` | 渐近最优 / + 椭球采样加速 |
| `og::PRMstar(si)` / `og::LazyPRM(si)` | 多次查询路标图 / 延迟边检查 |
| `og::FMT(si)` / `og::BITstar(si)` / `og::AITstar(si)` | 预采样最佳优先 / 批量信息化 / 自适应启发 |
| `planner->setRange(r)` | 设置树扩展步长（0 = 自动） |

**约束规划**

| API | 说明 |
|-----|------|
| `ob::Constraint(ambientDim, coDim)` | 约束基类，须 override `function` 与 `jacobian` |
| `constraint->setTolerance(t)` / `setMaxIterations(n)` | 投影容差 / Newton-Raphson 最大迭代 |
| `ob::ProjectedStateSpace(space, constraint)` | Newton 投影法（低曲率，首选） |
| `ob::AtlasStateSpace(space, constraint)` | 多 chart 覆盖（高曲率焊缝） |
| `ob::TangentBundleStateSpace(space, constraint)` | 切空间采样（实验性） |
| `ob::ConstrainedSpaceInformation(css)` | 约束空间的 SpaceInformation |

**Benchmark**

| API | 说明 |
|-----|------|
| `ot::Benchmark b(ss, name)` + `b.addPlanner(...)` | 配置多规划器对比 |
| `ot::Benchmark::Request{maxTime, maxMem, runCount}` | 每次时限 / 内存上限 / 重复次数 |
| `b.benchmark(request)` + `b.saveResultsToFile("...")` | 运行并保存，用 `ompl_benchmark_statistics.py` 出 PDF |

---

## 本章与后续章节的关系

| 后续章节 | 关系 | 本章铺垫的知识点 |
|---------|------|----------------|
| **M08 轨迹优化规划器** | 直接下游：接收本章输出的锯齿几何路径作为初始猜测 | "可行性优先+后续优化"范式（M07.4）、链式管线桥接点（M07.6）、路径简化（M07.5） |
| **M09 GPU 加速规划** | 性能延伸：把本章识别的碰撞检测瓶颈用 GPU/SIMD 攻克 | 碰撞检测占 80-95%（M07.5）、VAMP/cuRobo 预读（M07.8b）、布尔接口解耦使加速无侵入（M07.5） |
| **M10 时间参数化** | 串行下游：给 M08 平滑后的几何/路径加上时间律 | 本章产出的是无时间信息的纯几何路径 $[q_0,\ldots,q_T]$（累积项目） |
| **M14 MoveIt2 集成** | 系统整合：本章的 OMPL 在 MoveIt2 中的完整插件化部署 | ompl_planning.yaml（M07.6）、PlanningScene 线程安全（M07.5）、pluginlib 加载（故障排查手册） |
| **M04 碰撞检测**（前置回访） | 反向复用：本章把 M04 封装为 StateValidityChecker | 宽相/窄相（M07.0）、FCL/Bullet 后端选择（M07.8）、CCD（M07.5） |

---

## 研究实践建议

按掌握程度分层，给出从入门到研究的进阶路径：

**入门级（先跑起来）**：装好 OMPL（推荐随 ROS2 + MoveIt2 一起装），把 M07.5 的完整 7-DOF 示例编译运行，切换 `RRTConnect` ↔ `BITstar` 观察求解时间与路径长度差异。这一步的目标是建立"算法是可替换插件"的直觉。

**核心级（理解为什么）**：用 OMPL Benchmark 在同一 Panda 场景上对比 RRT-Connect / RRT* / BIT* / AIT*，亲手画出"求解时间 vs 路径长度"的 Pareto 前沿——你会直观看到 M07.4 算法对比总表里的取舍。再精读 `RRTConnect.cpp` 的 `solve()`，把代码与 Kuffner-LaValle 论文 Algorithm 1 逐行对应。

**进阶级（吃透理论）**：读 Karaman & Frazzoli (2011) 的渐近最优性证明，对照本章 M07.4 的证明骨架；再读 Solovey et al. (2020) 的勘误，理解"样本顺序引入额外维度"这个微妙之处。这能训练你对论文引理保持批判性——经典结果也可能有缺口。

**研究级（站到前沿）**：复现 Informed-RRT* 的椭球采样（M07.2 公式），或基于 OMPL 实现一个自定义采样器/MotionValidator。若对加速感兴趣，提前读 VAMP（ICRA 2024）论文，思考"为什么加速碰撞检测比改进算法逻辑更有杠杆"——这是 M09 的核心命题。

**通用纪律**：每改一个参数只改一个、跑 50 次取统计分布（而非单次运行），安全相关参数（`longest_valid_segment_fraction`）下降立即回退——这是 M07.8 系统调参方法论的精髓，也是工业落地与学术 demo 的分水岭。

---

## 符号表

本章引入的核心数学符号汇总如下。首次出现的小节标注在"出处"列，方便回查上下文。

| 符号 | 含义 | 出处 |
|------|------|------|
| $\mathcal{C}$ | 构型空间（Configuration Space），机械臂所有关节角构成的高维空间 | M07.1 |
| $\mathcal{C}_{\text{free}}$ | 自由构型空间，$\mathcal{C}$ 中不与障碍物碰撞的子集 | M07.1 |
| $\mathcal{C}_{\text{obs}}$ | 障碍物构型空间，$\mathcal{C} \setminus \mathcal{C}_{\text{free}}$ | M07.1 |
| $\mathcal{W}$ | 工作空间（Workspace），通常为 $\mathbb{R}^3$ 或 $SE(3)$ | M07.1 |
| $d$ | 构型空间维度（$d = n_{\text{DOF}}$），如 7-DOF 臂 $d=7$ | M07.1 |
| $n$ | 采样点数量 | M07.2 |
| $q_{\text{start}}, q_{\text{goal}}$ | 起始构型与目标构型 | M07.1 |
| $r(n)$ | 连接半径，PRM*/RRT* 中随采样数衰减的邻域半径 | M07.3/M07.4 |
| $\gamma$ | 连接半径公式中的常数因子，需满足 $\gamma > \gamma^*$ 以保证渐近最优 | M07.4 |
| $\gamma^*$ | 渐近最优的临界常数门槛，$\gamma^* = 2(1+1/d)^{1/d}(\mu(\mathcal{C}_{\text{free}})/\zeta_d)^{1/d}$ | M07.4 |
| $\zeta_d$ | $d$ 维单位球的体积，$\zeta_d = \pi^{d/2}/\Gamma(d/2+1)$ | M07.4 |
| $\mu(\mathcal{C}_{\text{free}})$ | 自由构型空间的 Lebesgue 测度（"体积"） | M07.4 |
| $\epsilon$ | RRT 步长（extend 距离），控制树每次生长的最大长度 | M07.4 |
| $d_{\text{eff}}$ | Informed-RRT* 椭球采样的有效维度，$d_{\text{eff}} < d$ | M07.4 |
| $c_{\text{best}}$ | 当前已知最优路径代价，用于 Informed-RRT* 的椭球约束 | M07.4 |

---

## 版本信息速查

| 项 | 版本/取值 | 说明 |
|----|----------|------|
| OMPL | 1.6+ | 约束规划（ProjectedStateSpace/AtlasStateSpace）、BIT*/AIT* 已完整集成；具体 API 以安装版本为准 |
| MoveIt2 | Humble / Kilted (2025+) | `ompl_planning.yaml` 配置；Kilted 支持多 pipeline 与请求/响应适配器 |
| ROS2 | Humble 及以上 | OMPL 通常随 MoveIt2 一并安装 |
| C++ 标准 | C++17 | 示例编译用 `-std=c++17 -O2` |
| RRTConnect 默认 | `range=0.0`（自动）、`goal_bias≈0.05` | 自动步长在 `setup()` 时由空间直径确定 |
| `longest_valid_segment_fraction` | 默认 0.005 | 安全关键场景降至 0.001-0.003 |
| 编写规范 | v5.0 / 算法工程教学 | text:code ≥ 60:40，三类陷阱全覆盖 |
