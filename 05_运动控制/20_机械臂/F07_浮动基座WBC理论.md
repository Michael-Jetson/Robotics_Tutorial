> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# F07 浮动基座 WBC 理论——从固定基座力控到全身控制的范式跃迁

> **本章定位**：本章是力控系列从固定基座（F01-F06）跨入浮动基座的关键转折点。从"为什么固定基座阻抗控制不够"出发，建立浮动基座全身控制（Whole-Body Control, WBC）的第一性原理——Sentis & Khatib 2005 的多优先级零空间投影框架，然后展开加权 QP 与层次化 QP（HQP）两大实现范式，最终通过 TSID 和 mc_rtc 两大开源框架的实战演练，让读者掌握从理论到代码的完整链路。
>
> **前置依赖**：F01（阻抗/导纳二分法）、F02（操作空间动力学/无源性理论）、F03（经典力控算法/笛卡尔阻抗控制推导）、M01（Pinocchio 深度精读/CRTP/动力学 API）、M05（QP 求解器/凸优化基础）
>
> **下游章节**：F08（MPC+WBC 联合力控）、F09（学习型力控）、F10（综合实战）
>
> **建议用时**：4 周（浮动基座动力学回顾 1 周 + WBC-QP 理论 1 周 + TSID/mc_rtc 实战 1 周 + 综合练习 1 周）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | 写出固定基座机械臂的操作空间动力学方程 $\Lambda\ddot{x} + \mu + p = F$。$\Lambda$ 是什么？它与 $M, J$ 的关系是什么？ | F02 第 4 节 |
| 2 | 零空间投影矩阵 $N = I - J^+ J$ 的几何意义是什么？为什么次要任务必须投影到零空间？ | F02 第 5 节 |
| 3 | 若取 $e_x=x_d-x$，笛卡尔阻抗控制律 $\tau = J^T(K_d e_x + D_d \dot{e}_x) + g(q)$ 在接触刚性环境时，阻抗参数 $K_d, D_d$ 的选择如何影响稳定性？ | F03 第 3 节 |
| 4 | Pinocchio 中 `computeAllTerms(model, data, q, v)` 计算了哪些量？`data.M`, `data.nle` 分别是什么？ | M01 第 6 节 |
| 5 | QP（二次规划）的标准形式是什么？等式约束和不等式约束分别对应什么物理含义？ | M05 第 2 节 |

---

## 本章目标

学完本章后，你应该能够：

1. **解释**为什么固定基座力控框架不能直接用于浮动基座机器人，识别"6 个无驱动自由度"这一根本区别
2. **推导** WBC-QP 的完整数学形式，包括决策变量 $[\dot{v}, \tau, f_c]$ 的选择理由和所有约束的物理来源
3. **区分**加权 QP（soft priority）与层次化 QP（strict priority）的数学结构和工程适用场景
4. **掌握** TSID 框架的 Contact6d 12 维力表示及其与摩擦锥约束的关系
5. **使用** mc_rtc 的四种力控任务（AdmittanceTask / ImpedanceTask / CoPTask / DampingTask）并在仿真中运行
6. **建立**固定基座阻抗控制（F03-F04）与浮动基座 WBC 的统一视角，理解 WBC 是阻抗控制的多任务推广

### 本章知识导航

本章包含 9 个核心知识模块，沿"理论→框架→实战"三层递进：

```
理论层（为什么 + 数学基础）
  §1 为什么需要 WBC → §2 浮动基座动力学回顾

框架层（怎么做 + 两大实现范式）
  §3 WBC-QP 标准形式 → §4 TSID Contact6d → §5 mc_rtc 力控任务 → §6 统一视角

实战层（代码 + 工程集成）
  §7 TSID API 实战 → §8 移动操作接口 → §9 前沿展望
```

知识点之间的依赖关系：§1 和 §2 是全章基础。§3 的 QP 组装是核心技能，§4 和 §5 分别展示 TSID 和 mc_rtc 两个框架如何实现 §3 的理论。§6 将 WBC 与 F03 阻抗控制统一起来。§7-§9 是工程应用。

推荐阅读路径：初学者应按顺序阅读 §1-§3-§6（核心路径，约 4 小时）。使用 TSID 的读者加读 §4-§7；使用 mc_rtc 的读者加读 §5。

### 前置知识桥接

> 回顾 F02 第 4-5 节：操作空间动力学将关节空间控制问题映射到任务空间，$\Lambda = (JM^{-1}J^T)^{-1}$ 是操作空间惯量矩阵，零空间投影 $N = I - J^+J$ 保证次要任务不干扰主任务。在固定基座场景下，这些工具已经能处理单末端跟踪任务。但当基座不再固定时——例如人形站在地面上、四足搭载机械臂——操作空间动力学的所有公式都需要扩展到浮动基座维度 $(6+n)$，并且引入接触力 $f_c$ 作为新的决策变量。这就是本章的核心任务。
>
> 回顾 M05 第 2-3 节：QP 的标准形式 $\min \frac{1}{2}z^THz + g^Tz$ s.t. $A_{eq}z = b_{eq}$, $A_{ineq}z \leq b_{ineq}$，以及 KKT 条件的物理含义。WBC 的核心计算就是在每个控制周期求解一个这样的 QP。

### 如果跳过本章会怎样

- **场景 1**：你在开发四足+臂的 loco-manipulation 系统（如 ANYmal + DynaArm）。如果不了解浮动基座 WBC 理论，你无法正确处理"6 个无驱动自由度"——直接把固定基座的阻抗控制移植过来，机器人会因为缺少基座力矩补偿而倾倒。
- **场景 2**：你想在 Franka 上实现同时跟踪末端位姿 + 避奇异 + 关节限位的多任务控制。如果不了解 WBC-QP 框架，你只能用 F02 的零空间投影——它无法处理不等式约束（如关节限位、力矩限），也无法在任务冲突时给出最优折中。

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含练习） | 12 小时 | 需要深入理解 WBC 理论并动手实现的读者 |
| 速读（跳过代码细节） | 5 小时 | 有控制理论基础、只需了解框架结构的读者 |
| 速查（只看表格和速查卡） | 40 分钟 | 遇到具体 QP 组装问题时回来查 |

---

### 知识导航

> 本章知识结构详见章首知识导航图（略——各节之间的逻辑递进关系见本章目标中的编号顺序）。

### 如果跳过本章会怎样

1. **直接学 F08（MPC+WBC 联合力控）**：你会看到 MPC 输出期望加速度/接触力，但不理解 WBC-QP 如何将这些期望转化为关节力矩——QP 的约束从何而来、为什么需要优先级
2. **直接做移动操作（loco-manipulation）**：你会需要同时控制足端力和手端力，但不理解浮动基座的欠驱动约束如何影响力分配

### 预计阅读时间

| 模式 | 时间 | 适用人群 |
|------|------|---------|
| 精读（含推导和练习） | 10-15 小时 | 首次系统学习该方向 |
| 速读（关注结论） | 4-6 小时 | 有基础，补充特定知识 |
| 速查（仅看总结表格） | 1 小时 | 工程实现时查阅 |


## 1. 为什么机械臂也需要 WBC ⭐

### 动机——固定基座力控的天花板

> 回顾 F03：若取 $e_x=x_d-x$，固定基座 7-DOF 机械臂上的笛卡尔阻抗控制律 $\tau = J^T(K_d e_x + D_d \dot{e}_x) + g(q)$ 已经能够实现柔顺的力/位混合控制。那为什么还需要 WBC？

这个问题的答案取决于你的机器人是否满足两个前提假设：

| 假设 | 固定基座 7-DOF 臂 | 移动机械臂 | 人形上肢 |
|------|-------------------|-----------|---------|
| 基座固定不动 | 螺栓固定在桌面 | 基座是移动平台 | 基座是躯干/骨盆 |
| 所有关节有执行器 | 每个关节一个电机 | 平台可能只有轮/腿 | 浮动基座无执行器 |

当这两个假设被打破时，固定基座的力控框架面临三个根本性困难：

**困难 1：6 个无驱动自由度**

浮动基座机器人（如人形、四足+臂、移动平台+臂）的基座有 6 个自由度（3 平移 + 3 旋转），但没有直接的执行器。这意味着基座的运动只能通过地面接触力**间接控制**。

$$\underbrace{M(q)\dot{v} + h(q,v)}_{\text{动力学}} = \underbrace{\begin{bmatrix} 0_{6\times1} \\ \tau \end{bmatrix}}_{S^T\tau: \text{关节力矩（前6行为0）}} + \underbrace{J_c^T f_c}_{\text{接触力}}$$

注意 $S^T = [0_{6\times n}; I_{n\times n}]$ 是选择矩阵——它反映了一个物理事实：**你无法直接给浮动基座施加力矩**。前 6 行（浮动基座对应的动力学方程）只有接触力 $J_c^T f_c$ 这一项，没有 $\tau$。

> **类比**：固定基座机械臂像一个站在地板上的人用手推墙——脚牢牢钉在地上，手的力直接由手臂肌肉产生。浮动基座机器人像一个站在冰面上的人推墙——你必须通过脚与地面的摩擦力**间接**产生推力，否则你自己会被推走。WBC 就是解决"冰面推墙"问题的控制框架。

**困难 2：多任务冲突**

在固定基座上，阻抗控制只需要跟踪一个末端位姿。但浮动基座机器人通常有多个同时需要跟踪的任务：

```
人形机器人的典型任务列表（按优先级降序）：
  Priority 0: 动力学可行性（等式约束，不可违反）
  Priority 1: 摩擦锥约束（不等式约束，不可违反）
  Priority 2: 质心位置跟踪（维持平衡）
  Priority 3: 末端位姿跟踪（操作任务）
  Priority 4: 躯干姿态跟踪（美观/视觉稳定）
  Priority 5: 关节正则化（避免奇异/关节限位）
```

这些任务可能相互冲突——例如，末端想向前伸得更远（Priority 3），但这会让质心偏离支撑多边形（违反 Priority 2）。固定基座的阻抗控制没有处理多任务优先级的机制。

**困难 3：接触力必须被优化**

固定基座力控不需要关心接触力——基座螺栓承受所有反力。浮动基座则不同：地面接触力是控制输出的一部分，必须满足摩擦锥约束（否则脚滑），还要满足法向力约束（否则脚离地）。

> **反事实推理**：如果浮动基座机器人不优化接触力会怎样？
> - 如果接触力的切向分量超过摩擦锥 → 脚底打滑 → 机器人摔倒
> - 如果法向力为负 → 脚离地 → 单脚支撑变成零支撑 → 自由落体
> - 如果接触力分布不均 → ZMP 偏离支撑多边形中心 → 倾翻力矩 → 不稳定
>
> 所以接触力不是"可选的优化目标"，而是"必须满足的安全约束"。

### 三类需要 WBC 的机械臂场景

**场景 A：移动操作（Mobile Manipulation）**

```
全向移动平台 + 7-DOF 臂（如 TIAGo, HSR）：
  平台 DOF: 3（x, y, theta）——轮驱动
  臂 DOF: 7
  总 DOF: 10，末端 6D → 4 维零空间

WBC 任务栈：
  Task 1: 末端位姿跟踪（操作任务）
  Task 2: 平台姿态稳定（防止倾翻）
  Task 3: 关节避奇异

为什么需要 WBC：平台和臂的自由度耦合——末端的力会通过臂传递到平台，
                可能导致平台侧翻（特别是臂伸到极限位置时）。
```

**场景 B：人形上肢操作（Humanoid Manipulation）**

```
人形上半身（如 iCub, JVRC-1）：
  浮动基座: 6 DOF（无执行器）
  躯干: 3 DOF
  双臂: 7+7 = 14 DOF
  总 DOF: 23，双臂末端 12D → 11 维零空间

WBC 任务栈：
  Task 1: 双脚接触约束（维持站立）
  Task 2: 质心位置/ZMP（维持平衡）
  Task 3: 右臂末端位姿（操作任务）
  Task 4: 左臂末端位姿（辅助/抓握）
  Task 5: 躯干姿态（保持直立）
  Task 6: 关节正则化
```

**场景 C：四足+臂（Quadruped + Arm, Loco-Manipulation）**

```
四足 + 6-DOF 臂（如 ANYmal + DynaArm, AlienGo + Z1）：
  浮动基座: 6 DOF（无执行器）
  四足: 3x4 = 12 DOF
  臂: 6 DOF
  总 DOF: 24

WBC 任务栈：
  Task 1: 四足接触约束 + 摩擦锥
  Task 2: 质心位置（四足平衡）
  Task 3: 臂末端位姿（操作任务）
  Task 4: 臂末端力跟踪（力控操作）
  Task 5: 体姿态（保持水平）
  Task 6: 关节正则化
```

> **本质洞察**：WBC 不是"另一种力控算法"，而是**多任务约束优化框架**，它可以把固定基座阻抗控制（F03）作为其中一个任务嵌入到更大的优化问题中。从这个角度看，固定基座阻抗控制与 WBC 的单任务、无接触、固定基座情形有相同的控制结构；但二者是否给出同一力矩，还取决于代价权重、动力学度量和正则化选择。

### 历史演进

| 年代 | 里程碑 | 核心思想 |
|------|--------|---------|
| 1987 | Khatib, "A Unified Approach for Motion and Force Control" | 操作空间控制——WBC 的理论起点 |
| 2005 | Sentis & Khatib, "Synthesis of Whole-Body Behaviors", IJHR | 多优先级零空间投影——WBC 概念化 |
| 2012 | De Luca et al., "Sensorless Estimation of Interaction Forces", ICRA | 动量观测器扩展到浮动基座 |
| 2015 | Del Prete et al., "TSID (Task Space Inverse Dynamics)" | 加权 QP 全身控制，Pinocchio 后端 |
| 2019 | Bouyarmane et al., "mc_rtc: Real-Time Control Framework" | 最完整的人形 WBC 框架，4 种力控任务 |
| 2022 | Romualdi et al., "BipedLocomotionFramework (BLF)" | iCub/ergoCub 力控，QPTSID + CentroidalMPC |
| 2022 | qiayuanl, "legged_control" | OCS2+WBC 教学友好实现，Unitree 适配 |
| 2024 | Dantec et al., "mc_rtc Impedance & Admittance Tasks" | mc_rtc v2.x 几何一致的 SE(3) 阻抗任务 |

### 如果不用 WBC 会怎样

假设你在人形机器人上直接用 F03 的笛卡尔阻抗控制（忽略浮动基座）：

```
tau_arm = J_arm^T(K_d e_x - D_d x_dot) + g_arm(q)    <-- F03 的控制律，e_x = x_d - x 且 x_d 静止

问题 1: g_arm(q) 只补偿了臂的重力，没有补偿浮动基座和躯干的重力
  -> 机器人会因为自身重力而倾倒（躯干没有力矩支撑）

问题 2: 没有摩擦锥约束
  -> 臂施加大力时，反作用力可能让脚底打滑

问题 3: 没有 ZMP 约束
  -> 臂伸出去操作时，质心可能偏离支撑多边形 -> 倾翻

问题 4: 关节力矩没有考虑浮动基座动力学
  -> tau_arm 中缺少 J_c^T f_c 项（接触力的影响）
  -> 力矩计算不准确，实际执行时力控精度很差
```

所以，**WBC 不是锦上添花，而是浮动基座力控的基本需求**。

### ⚠️ 常见陷阱

```
💡 概念误区：认为"WBC 是腿足机器人专用的"
   新手想法："我做机械臂不需要 WBC，那是做四足/人形的人才学的"
   实际上：只要你的机械臂安装在移动平台上、协作机器人需要全身避障、
          或者你需要同时控制多个末端——你就需要 WBC 框架。
          WBC 的本质是"多任务约束优化"，与基座是否浮动无关。
          即使固定基座 7-DOF 臂，WBC 也能帮你优雅地处理：
          末端跟踪 + 关节避奇异 + 关节限位 + 自碰撞避免的多目标冲突。
   正确认识：WBC 是通用的多任务控制框架，浮动基座只是它最常见的应用场景。
```

```
⚠️ 编程陷阱：直接对浮动基座用 g(q) 补偿
   错误做法：把 Pinocchio 计算的 g(q) 直接发送给关节电机
   现象：关节力矩指令中包含了浮动基座前 6 行对应的"虚拟力矩"——
        但浮动基座没有电机，这些力矩无处可施。
   根本原因：g(q) 属于 R^(6+n) 是全维度的广义力，前 6 个元素对应浮动基座，
            只有后 n 个元素才是关节力矩。
   正确做法：tau = g(q)[6:end]（只取关节部分），或使用 S^T tau 的选择矩阵形式。
   自检方法：打印 g(q).size()，确认等于 6+n 而非 n。
```

```
🧠 思维陷阱：认为 WBC 比阻抗控制"更好"
   新手想法："WBC 这么强大，是不是应该所有场景都用 WBC 替代阻抗控制？"
   实际上：WBC 和阻抗控制不是替代关系，而是包含关系。
          WBC 框架中的每个任务本身可以是阻抗控制任务。
          对于固定基座单任务，直接用阻抗控制更简单、更直观、更容易调参。
          WBC 引入了 QP 求解器、多任务权重、约束组装等额外复杂度。
   正确思维：问三个问题来决定是否需要 WBC：
          1. 基座是否浮动？（是 -> WBC）
          2. 是否有多个需要优先级排序的任务？（是 -> WBC）
          3. 是否需要优化接触力（摩擦锥/ZMP）？（是 -> WBC）
          如果三个都是"否"，F03 的阻抗控制就够了。
```

### 练习

1. ⭐ **约束计数**：对于一个双足人形机器人（浮动基座 6 DOF + 两条腿各 6 DOF + 两只手臂各 7 DOF = 32 DOF），站立时双脚着地（2 个 6D 接触），WBC-QP 有多少个决策变量？多少个等式约束？（提示：$z = [\dot{v}; \tau; f_c]$，其中 $\dot{v} \in \mathbb{R}^{38}$, $\tau \in \mathbb{R}^{26}$, $f_c = ?$）
2. ⭐ **关系分析**：分析当浮动基座固定（$\dot{v}_{base} = 0$, $\ddot{v}_{base} = 0$）、只有一个末端跟踪任务、无接触力约束时，WBC-QP 与 F03 笛卡尔阻抗控制律在什么度量和正则化选择下具有相同形式。（提示：从 QP 的 KKT 条件出发）
3. ⭐⭐ **场景设计**：为一个 TIAGo 移动操作平台（全向轮底盘 3 DOF + 升降柱 1 DOF + 7-DOF 臂）设计 WBC 任务栈：列出所有任务、优先级、约束类型。解释为什么升降柱的存在改变了任务优先级设计。

---

## 2. 浮动基座动力学回顾 ⭐⭐

### 动机——WBC 的数学基础

> 回顾 M01：Pinocchio 中的浮动基座模型使用 $q = [q_{base}; q_{joint}] \in \mathbb{R}^{7+n}$（基座用四元数 7D 表示配置），$v = [v_{base}; \dot{q}_{joint}] \in \mathbb{R}^{6+n}$（基座用 6D 空间速度表示速度）。注意 $\dim(q) \neq \dim(v)$，因为 SO(3) 用 4D 四元数表示但切空间是 3D。

WBC 的所有数学推导建立在浮动基座动力学方程之上。在进入 QP 组装之前，我们必须清楚每个矩阵、每个向量的物理含义和计算方式。

### 标准形式推导

浮动基座刚体系统的运动方程（Euler-Lagrange 形式）：

$$M(q)\dot{v} + h(q,v) = S^T\tau + J_c^T f_c$$

各项含义：

| 符号 | 维度 | 物理含义 | Pinocchio API |
|------|------|---------|---------------|
| $M(q)$ | $(6+n) \times (6+n)$ | 广义质量矩阵（正定对称） | `data.M` after `crba()` |
| $\dot{v}$ | $(6+n) \times 1$ | 广义加速度：$[\dot{v}_{base}; \ddot{q}_{joint}]$ | 决策变量 |
| $h(q,v)$ | $(6+n) \times 1$ | 非线性效应：$C(q,v)v + g(q)$（科氏力+重力） | `data.nle` after `nonLinearEffects()` |
| $S$ | $n \times (6+n)$ | 选择矩阵：$S = [0_{n\times 6}, I_{n\times n}]$ | 手动构造 |
| $\tau$ | $n \times 1$ | 关节力矩（控制输入） | 决策变量 |
| $J_c$ | $3k \times (6+n)$ | 接触雅可比（$k$ 个接触点，每个 3D） | `getFrameJacobian()` |
| $f_c$ | $3k \times 1$ | 接触力 | 决策变量 |

**为什么 $S^T\tau$ 而不是直接 $\tau$？**

这是 WBC 推导中最容易混淆的一点。在固定基座中，$\tau \in \mathbb{R}^n$ 直接出现在 $n$ 维动力学方程中。但在浮动基座中，动力学方程是 $(6+n)$ 维的，而关节力矩只作用在后 $n$ 个自由度上。$S^T$ 把 $n$ 维力矩"嵌入"到 $(6+n)$ 维空间中：

$$S^T\tau = \begin{bmatrix} 0_{6\times n} \\ I_{n\times n} \end{bmatrix} \tau = \begin{bmatrix} 0_{6\times 1} \\ \tau \end{bmatrix}$$

前 6 行全为零——这就是"浮动基座没有执行器"的数学表达。

**将动力学方程分块**：

把 $(6+n)$ 维方程按浮动基座（前 6 行）和关节（后 $n$ 行）分块：

$$\begin{bmatrix} M_{bb} & M_{bj} \\ M_{jb} & M_{jj} \end{bmatrix} \begin{bmatrix} \dot{v}_b \\ \ddot{q}_j \end{bmatrix} + \begin{bmatrix} h_b \\ h_j \end{bmatrix} = \begin{bmatrix} 0 \\ \tau \end{bmatrix} + \begin{bmatrix} J_{c,b}^T \\ J_{c,j}^T \end{bmatrix} f_c$$

前 6 行（浮动基座方程）：

$$M_{bb}\dot{v}_b + M_{bj}\ddot{q}_j + h_b = J_{c,b}^T f_c$$

这个方程没有 $\tau$——浮动基座的加速度 $\dot{v}_b$ **完全由接触力 $f_c$ 和耦合动力学决定**。

后 $n$ 行（关节方程）：

$$M_{jb}\dot{v}_b + M_{jj}\ddot{q}_j + h_j = \tau + J_{c,j}^T f_c$$

这里可以显式求解关节力矩：

$$\tau = M_{jb}\dot{v}_b + M_{jj}\ddot{q}_j + h_j - J_{c,j}^T f_c$$

> **本质洞察**：浮动基座动力学的核心矛盾在于——你想控制基座运动（如维持质心位置），但你没有基座执行器。唯一的途径是通过接触力 $f_c$ **间接控制**基座。这就是为什么 $f_c$ 必须作为 QP 的决策变量，而不像固定基座那样被忽略。

### 接触约束方程

接触点的运动学约束：**接触点的速度为零**（假设无滑动、非弹性接触）。

接触点的速度：

$$\dot{x}_c = J_c v$$

对时间求导得接触点的加速度约束：

$$\ddot{x}_c = J_c \dot{v} + \dot{J}_c v = 0$$

这个方程是 WBC-QP 的**等式约束**，确保接触点不滑动、不弹跳。

> **反事实推理**：如果不加接触约束会怎样？QP 可能给出让脚底"穿过地面"或"脱离地面"的加速度解——这在物理上不可能发生，但数学上是允许的。接触约束就是在告诉优化器："接触点必须保持静止，不准穿模也不准飞起来。"

**Pinocchio 计算接触约束项**：

```cpp
// 计算接触雅可比
pinocchio::Data::Matrix6x J_c(6, model.nv);
J_c.setZero();
pinocchio::getFrameJacobian(model, data, contact_frame_id,
                             pinocchio::LOCAL_WORLD_ALIGNED, J_c);
// 注意: LOCAL_WORLD_ALIGNED 帧——原点在接触点，轴与世界帧平行
// 对于点接触，只取平移部分 J_c.topRows(3)

// 计算 dJ * v（加速度层面的偏置项）
pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
pinocchio::Data::Matrix6x dJ_c(6, model.nv);
dJ_c.setZero();
pinocchio::getFrameJacobianTimeVariation(model, data, contact_frame_id,
                                          pinocchio::LOCAL_WORLD_ALIGNED, dJ_c);
// dJ_c * v 就是接触约束等式的偏置项
```

### 摩擦锥约束

库仑摩擦定律要求接触力满足：

$$\sqrt{f_{c,x}^2 + f_{c,y}^2} \leq \mu f_{c,z}, \quad f_{c,z} \geq 0$$

其中 $\mu$ 是摩擦系数。这是一个**二阶锥约束**（SOCP），不能直接用 QP 求解。

**线性化近似——摩擦金字塔**：

将圆锥用**外接**正方形（4 面）逼近（金字塔外切于圆锥，即金字塔包含圆锥）：

```
4 面线性化（金字塔）:
  +f_{c,x} <= mu * f_{c,z}
  -f_{c,x} <= mu * f_{c,z}
  +f_{c,y} <= mu * f_{c,z}
  -f_{c,y} <= mu * f_{c,z}
  f_{c,z} >= f_min           <-- 法向力下界（防脱离）

写成矩阵形式: A_fric * f_c <= b_fric

其中 A_fric = [ 1  0  -mu ]     b_fric = [0]
              [-1  0  -mu ]              [0]
              [ 0  1  -mu ]              [0]
              [ 0 -1  -mu ]              [0]
              [ 0  0  -1  ]              [-f_min]
```

> **双重解读**：
> 角度 1（物理）：摩擦金字塔是在说"脚底的水平力不能超过法向力的 $\mu$ 倍"。如果超过了，脚就会打滑。$f_{min}$ 是说"脚必须踩在地上"。
> 角度 2（优化）：把二阶锥约束替换为线性不等式约束，将 SOCP 降级为 QP，换取了更快的求解速度。此处 4 面金字塔 $|f_x| \leq \mu f_z$、$|f_y| \leq \mu f_z$ 是圆锥 $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$ 的**外逼近**（可行域更大，非保守），允许的最大切向力为 $\mu\sqrt{2} f_z$（对角方向超出圆锥约束）。它不能作为"保证不滑"的保守约束来解释。
>
> 若需要**内逼近**（保守，保证线性约束内的力一定落在真实圆锥内），工程上常用 $|f_x| + |f_y| \leq \mu f_z$，写成 4 个线性面：
> $$
> \begin{bmatrix}
>  1 &  1 & -\mu \\
>  1 & -1 & -\mu \\
> -1 &  1 & -\mu \\
> -1 & -1 & -\mu
> \end{bmatrix} f_c \leq 0, \quad f_{min} \leq f_z \leq f_{max}
> $$
> 等价的简化做法是在外逼近公式里使用 $\mu_{eff}=\mu/\sqrt{2}$。工程中常见"外逼近 + 降低 $\mu$"是经验安全裕度，不是数学上的保守近似。

### Pinocchio 计算流完整示例

```cpp
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

// Step 1: 前向运动学 + 所有项
pinocchio::forwardKinematics(model, data, q, v);
pinocchio::updateFramePlacements(model, data);
pinocchio::crba(model, data, q);       // 计算 M
data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();  // 对称化
pinocchio::nonLinearEffects(model, data, q, v);  // 计算 h = C*v + g

// Step 2: 雅可比
pinocchio::computeJointJacobians(model, data, q);
pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);

// Step 3: 对每个接触点计算 J_c 和 dJ_c*v
for (auto& contact : contact_frames) {
    pinocchio::getFrameJacobian(model, data, contact.id,
                                 pinocchio::LOCAL_WORLD_ALIGNED, contact.J);
    pinocchio::getFrameJacobianTimeVariation(model, data, contact.id,
                                              pinocchio::LOCAL_WORLD_ALIGNED, contact.dJ);
    contact.drift = contact.dJ * v;  // dJ*v 项
}

// 现在 data.M, data.nle, 以及各 contact.J, contact.drift 都准备好了
// 可以组装 QP
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：忘记对称化 CRBA 结果
   错误做法：直接使用 data.M，不做 triangularView 赋值
   现象：data.M 只有上三角有值，下三角是旧数据或垃圾值。
        QP 求解器可能读取下三角元素，导致不对称的质量矩阵。
   根本原因：Pinocchio 的 crba() 只计算上三角（性能优化），
            不自动填充下三角。
   正确做法：调用 crba() 后立即对称化。
   自检方法：assert((data.M - data.M.transpose()).norm() < 1e-10);
```

```
💡 概念误区：混淆 q 的维度和 v 的维度
   新手想法："q 和 v 维度应该一样吧？一个是位置，一个是速度"
   实际上：浮动基座的 q 用四元数表示旋转（7D），而 v 用角速度表示旋转（6D）。
          所以 dim(q) = 7 + n，但 dim(v) = 6 + n。
          M(q) 的维度是 (6+n)x(6+n)，与 v 匹配，不与 q 匹配。
          Pinocchio 用 model.nq (= 7+n) 和 model.nv (= 6+n) 区分。
   为什么重要：组装 QP 矩阵时，所有维度都按 nv 计算，不是 nq。
             用错维度会导致矩阵维度不匹配的运行时崩溃。
```

### 练习

1. ⭐ **维度验证**：对 Unitree A1 四足机器人（浮动基座 + 12 关节），写出 $M$, $h$, $S^T$, $J_c$, $f_c$ 的维度（假设 4 脚着地，每个接触点 3D 力）。
2. ⭐⭐ **分块推导**：从浮动基座动力学方程 $M\dot{v} + h = S^T\tau + J_c^T f_c$ 出发，把 $M$ 按 $6 \times 6$, $6 \times n$, $n \times 6$, $n \times n$ 分块，显式写出关节力矩 $\tau$ 关于 $\dot{v}_b$, $\ddot{q}_j$, $f_c$ 的表达式。
3. ⭐⭐ **摩擦锥外逼近误差**：计算 4 面外接金字塔相对于真实圆锥的截面积比（在 $f_{c,z} = const$ 平面上）。8 面外接近似呢？$k$ 面呢？写出 $k \to \infty$ 时面积比趋近 1 的速度，并说明为什么这不是保守约束。

---

## 3. WBC-QP 标准形式——从物理到优化 ⭐⭐

### 动机——把力控变成优化问题

> 回顾 F03 第 3 节：笛卡尔阻抗控制律 $\tau = J^T F + g(q)$ 是一个**解析公式**——给定状态，直接算出力矩。它的优点是计算快（微秒级），缺点是不能处理约束（关节限位、摩擦锥、多任务优先级）。

WBC 的核心思想：把力控制律的**求解**从解析公式变成**约束优化问题**（QP）。代价是计算量增加（毫秒级），换来的是处理任意约束和多任务优先级的能力。

**为什么用 QP 而不是一般 NLP？**

| 问题类型 | 求解时间 | 全局最优 | 求解器成熟度 |
|---------|---------|---------|-------------|
| LP（线性规划） | ~0.01 ms | 保证 | 非常成熟 |
| QP（二次规划） | ~0.1-1 ms | 保证（凸 QP） | 成熟（OSQP, ProxQP, qpOASES） |
| SOCP（二阶锥） | ~1-10 ms | 保证 | 较成熟（ECOS, Mosek） |
| SDP（半定规划） | ~10-100 ms | 保证 | 中等 |
| NLP（非线性规划） | ~10-1000 ms | 不保证（可能局部最优） | 复杂（Ipopt, SNOPT） |

WBC 需要在 500-1000 Hz 控制循环中求解，所以 QP 是自然选择：足够快、保证全局最优、约束处理能力满足需求。摩擦锥的线性化（第 2 节）正是为了把 SOCP 降级为 QP。

### 决策变量的选择

WBC-QP 的决策变量 $z$ 有三种常见选择：

| 形式 | 决策变量 | 维度 | 优缺点 |
|------|---------|------|--------|
| 形式 A | $z = [\dot{v}; \tau; f_c]$ | $(6+n)+n+3k$ | 最通用，动力学作为等式约束 |
| 形式 B | $z = [\dot{v}; f_c]$ | $(6+n)+3k$ | $\tau$ 由动力学方程显式算出，维度更小 |
| 形式 C | $z = [\tau; f_c]$ | $n+3k$ | $\dot{v}$ 由正向动力学算出，适合低 DOF |

**TSID 和 mc_rtc 都使用形式 A**——虽然维度最大，但约束组装最自然。形式 B 是 legged_control 中 WeightedWbc 的选择（通过 $\tau = M_{jb}\dot{v}_b + M_{jj}\ddot{q}_j + h_j - J_{c,j}^T f_c$ 消去 $\tau$）。

我们以形式 A 为主进行推导。

### 完整 QP 组装

**决策变量**：

$$z = \begin{bmatrix} \dot{v} \\ \tau \\ f_c \end{bmatrix} \in \mathbb{R}^{(6+n) + n + 3k}$$

**等式约束**：

约束 1——动力学约束（物理定律不可违反）：

$$M\dot{v} + h = S^T\tau + J_c^T f_c$$

写成标准形式 $A_{eq} z = b_{eq}$：

$$\begin{bmatrix} M & -S^T & -J_c^T \end{bmatrix} \begin{bmatrix} \dot{v} \\ \tau \\ f_c \end{bmatrix} = -h$$

约束 2——接触无滑约束（接触点保持静止）：

$$J_c \dot{v} + \dot{J}_c v = 0$$

写成标准形式：

$$\begin{bmatrix} J_c & 0 & 0 \end{bmatrix} \begin{bmatrix} \dot{v} \\ \tau \\ f_c \end{bmatrix} = -\dot{J}_c v$$

合并所有等式约束：

$$\underbrace{\begin{bmatrix} M & -S^T & -J_c^T \\ J_c & 0 & 0 \end{bmatrix}}_{A_{eq}} z = \underbrace{\begin{bmatrix} -h \\ -\dot{J}_c v \end{bmatrix}}_{b_{eq}}$$

**不等式约束**：

约束 3——摩擦锥（每个接触点 5 个不等式）：

$$A_{fric} f_{c,i} \leq b_{fric}, \quad i = 1, ..., k$$

约束 4——关节力矩限：

$$\tau_{min} \leq \tau \leq \tau_{max}$$

约束 5（可选）——关节位置/速度限：

$$q_{min} \leq q + v\Delta t + \frac{1}{2}\dot{v}\Delta t^2 \leq q_{max}$$

$$\dot{q}_{min} \leq v + \dot{v}\Delta t \leq \dot{q}_{max}$$

合并写成标准形式 $A_{ineq} z \leq b_{ineq}$。

**代价函数**（加权 QP 形式）：

$$\min_z \frac{1}{2} z^T H z + g^T z$$

其中代价函数由多个任务的加权和组成：

$$\text{cost} = \sum_{i=1}^{N_{tasks}} w_i \|J_i \dot{v} + \dot{J}_i v - \ddot{x}_{ref,i}\|^2 + w_\tau \|\tau\|^2 + w_f \|f_c - f_{c,ref}\|^2$$

每个任务 $i$ 的含义：
- $J_i$ 是任务的雅可比矩阵
- $\ddot{x}_{ref,i}$ 是期望的任务空间加速度（由 PD 控制律生成）
- $w_i$ 是任务权重

将每项展开并匹配标准形式 $\frac{1}{2}z^THz+g^Tz$，叠加得到 $H = \sum H_i$, $g = \sum g_i$。

**展开示例——单任务代价项**：

对任务 $i$，其代价 $w_i \|J_i \dot{v} + \dot{J}_i v - \ddot{x}_{ref,i}\|^2$ 可以写为：

$$w_i \|J_i \dot{v} - a_i\|^2$$

其中 $a_i = \ddot{x}_{ref,i} - \dot{J}_i v$（已知量）。

展开：

$$w_i (J_i \dot{v} - a_i)^T (J_i \dot{v} - a_i) = w_i (\dot{v}^T J_i^T J_i \dot{v} - 2 a_i^T J_i \dot{v} + a_i^T a_i)$$

由于 $\dot{v}$ 只是 $z$ 的前 $(6+n)$ 个元素，用提取矩阵 $E_v = [I_{(6+n)}, 0, 0]$ 表示 $\dot{v} = E_v z$：

匹配 $\frac{1}{2}z^THz+g^Tz$ 时，二次项和一次项都要带上 2 倍因子：

$$H_i = 2w_i E_v^T J_i^T J_i E_v, \quad g_i = -2w_i E_v^T J_i^T a_i$$

如果你的求解器接口使用的是 $\min \ z^THz + 2g^Tz$ 或 $\min \ \|Az-b\|^2$ 形式，则因子会被接口吸收；关键是同一个项目里必须统一约定。

**关节位置和速度限约束的离散化细节**：

约束 5 中关节位置限的离散化公式 $q_{min} \leq q + v\Delta t + \frac{1}{2}\dot{v}\Delta t^2 \leq q_{max}$ 需要特别注意。这个公式用 $\dot{v}$（QP 决策变量）预测下一时刻的关节位置，但它基于匀加速度假设——在 1 kHz 控制频率下，$\Delta t = 1$ ms 内的加速度变化确实可以忽略，所以这个近似是合理的。

但工程实践中有两个重要的注意事项：

1. **安全裕度**：不应直接使用物理关节限作为 $q_{min}, q_{max}$。建议使用 $q_{limit} \pm \epsilon$（如 $\epsilon = 5^\circ$），留出制动距离。否则 QP 可能给出让关节"恰好不超限"的加速度——但模型误差和延迟可能导致实际超限。

2. **速度限的处理**：$\dot{q}_{min} \leq v + \dot{v}\Delta t \leq \dot{q}_{max}$ 约束加速度，使得下一时刻速度不超限。但这假设电机能在 $\Delta t$ 内产生所需的加速度——实际上电机的力矩响应有延迟。更保守的做法是将速度限乘以安全系数（如 0.8）。

**QP 可行域的可视化理解**：

将所有约束叠加后，QP 的可行域是一个**凸多面体**（等式约束定义了一个线性子空间，不等式约束在其上切出一个凸区域）。代价函数 $\frac{1}{2}z^THz + g^Tz$ 定义了一个二次曲面——QP 的最优解就是这个二次曲面在凸多面体内的最低点。

对于 WBC，这个几何图像的物理含义是：
- **等式约束**（动力学+接触）限定了"物理上可能的运动"
- **不等式约束**（摩擦锥+力矩限）进一步限定了"安全的运动"
- **代价函数**（任务跟踪）在安全运动中选择"最好的运动"

当可行域为空（infeasible）时，意味着**没有任何安全且物理可行的运动**——通常是接触状态错误或任务目标不可达导致的。

**任务空间加速度参考的生成**：

每个任务 $i$ 的加速度参考由 PD 控制律生成（这是 WBC 内部的"小控制器"）：

$$\ddot{x}_{ref,i} = \ddot{x}_{ff,i} + K_{p,i}(x_{d,i} - x_i) + K_{d,i}(\dot{x}_{d,i} - \dot{x}_i)$$

其中 $\ddot{x}_{ff,i}$ 是前馈加速度（通常来自 MPC 或轨迹规划器），$K_{p,i}, K_{d,i}$ 是任务空间的刚度和阻尼。

> **理论到工程衔接**：注意到这个 PD 控制律与 F03 的笛卡尔阻抗控制在**形式上完全一样**——都是"$K_p$ 位置误差 + $K_d$ 速度误差"。区别在于：F03 直接把这个控制律转换为关节力矩 $\tau = J^T F$；WBC 则把它作为 QP 代价函数的一项，由 QP 求解器在满足所有约束的前提下**尽力跟踪**这个参考。

### 加权 QP vs 层次化 QP（HQP）

**加权 QP（Weighted QP / Soft Priority）**：

$$\min_z \sum_{i=1}^{N} w_i \|J_i \dot{v} - b_i\|^2 + \text{regularization}$$
$$\text{s.t. } A_{eq} z = b_{eq}, \quad A_{ineq} z \leq b_{ineq}$$

所有任务通过权重 $w_i$ "软竞争"。$w_i$ 越大，任务 $i$ 越优先。

| 优点 | 缺点 |
|------|------|
| 一个 QP 求解，计算快 | 权重难调——$w_1 / w_2$ 的比值对结果影响很大 |
| 实现简单 | 高优先级任务不能保证完美满足 |
| 连续可微（利于基于梯度的 MPC） | 权重缺乏物理直觉（$w = 100$ 什么意思？） |

**层次化 QP（Hierarchical QP / Strict Priority / HQP）**：

```
Level 0（最高优先级）：
  求解: min ||e_0||^2 s.t. 等式约束 + 不等式约束
  记录最优值: ||e_0||^2* = epsilon_0

Level 1（次高优先级）：
  求解: min ||e_1||^2 s.t. 等式约束 + 不等式约束
                     + ||e_0||^2 <= epsilon_0 + delta（不破坏上层）
  记录最优值: ||e_1||^2* = epsilon_1

Level k（第 k 优先级）：
  求解: min ||e_k||^2 s.t. 等式约束 + 不等式约束
                     + ||e_i||^2 <= epsilon_i + delta, i = 0,...,k-1
```

等价实现方式——零空间投影法（Sentis & Khatib 2005）：

$$\dot{v}_k^* = \dot{v}_{k-1}^* + N_{k-1}(\dot{v}_k^{raw} - \dot{v}_{k-1}^*)$$

其中 $N_{k-1}$ 是前 $k-1$ 个任务的**叠加零空间投影矩阵**：

$$N_{k-1} = I - J_{1:k-1}^+ J_{1:k-1}$$

> **跨领域类比**：加权 QP 像公司里的**矩阵管理**——所有部门的需求同时提交给老板，老板按"重要性系数"（权重）分配资源，每个部门都能拿到一些但谁也拿不到 100%。HQP 像**军队的命令链**——将军的命令绝对执行，上校只能在不违反将军命令的前提下自由发挥，以此类推。

| 对比维度 | 加权 QP | HQP |
|---------|--------|-----|
| 高优先级保证 | 不保证（但 $w_i \gg w_j$ 时近似保证） | 严格保证 |
| 计算量 | 1 个 QP | $N$ 个 QP（$N$ = 优先级层数） |
| 调参 | 调 $N$ 个权重 $w_i$（调参地狱） | 只需设定优先级顺序 |
| 适用场景 | MPC 内嵌 WBC（需要可微） | 独立 WBC（安全关键） |
| 代表实现 | TSID | legged_control 的 HoQp |

> **反事实推理**：如果用加权 QP 且权重设置不当（如平衡任务权重 $w_1 = 10$ 和操作任务权重 $w_2 = 8$，差距不够大），可能出现：操作任务把质心"拉"出支撑多边形——机器人为了够到一个远处的物体而摔倒。HQP 不会出现这个问题，因为平衡任务的约束是**不可违反**的。这也是为什么 legged_control 在四足场景下选择 HQP——脚底不打滑比手够得更远重要得多。

### 任务空间加速度参考的设计——从 PD 到高级控制律

§3 前面提到任务空间加速度参考由 PD 控制律生成：$\ddot{x}_{ref,i} = \ddot{x}_{ff,i} + K_{p,i}(x_{d,i} - x_i) + K_{d,i}(\dot{x}_{d,i} - \dot{x}_i)$。这是最基本的形式。在实际系统中，加速度参考的设计有三个层次，每个层次引入更多的信息和更好的性能。

**层次 1——纯 PD 反馈**（无前馈）：

$$\ddot{x}_{ref} = K_p(x_d - x) + K_d(\dot{x}_d - \dot{x})$$

适用于静态目标跟踪。$K_p$ 和 $K_d$ 的选择遵循临界阻尼条件 $K_d = 2\sqrt{K_p}$（标量情况）或 $K_d = 2\sqrt{K_p M}$（考虑惯量时）。但对于运动目标（如跟踪 MPC 输出的时变轨迹），纯 PD 反馈会产生稳态跟踪误差——因为没有前馈项来"告诉"控制器目标在加速。

**层次 2——PD + 前馈加速度**：

$$\ddot{x}_{ref} = \ddot{x}_d + K_p(x_d - x) + K_d(\dot{x}_d - \dot{x})$$

$\ddot{x}_d$ 来自轨迹规划器或 MPC。前馈项消除了运动目标的稳态跟踪误差。这是 TSID 和 mc_rtc 的默认模式。

**层次 3——PD + 前馈 + 积分抗偏移**：

$$\ddot{x}_{ref} = \ddot{x}_d + K_p(x_d - x) + K_d(\dot{x}_d - \dot{x}) + K_i \int_0^t (x_d - x) d\tau'$$

积分项 $K_i$ 消除因模型误差或摩擦导致的持续位姿偏移。但积分项可能引入 wind-up 问题——当约束限制了实际运动时，积分状态持续累积，解除约束后产生过冲。工程实现时必须加 anti-wind-up 机制（如积分饱和限幅或条件积分）。

| 层次 | 稳态误差 | 瞬态性能 | 复杂度 | 适用场景 |
|------|---------|---------|--------|---------|
| 纯 PD | 有（运动目标） | 好 | 低 | 静态目标、调试阶段 |
| PD + 前馈 | 无 | 好 | 中 | **通用推荐** |
| PD + 前馈 + 积分 | 无 | 取决于 $K_i$ | 高 | 有持续偏差的精密任务 |

> **理论到工程衔接**：$K_p$ 和 $K_d$ 的选择直接影响 QP 代价函数的形状——$K_p$ 过大会让 QP 中该任务的"引力"过强，可能挤占其他任务的"空间"。工程上建议从 $K_p = 50, K_d = 14$（阻尼比 $\zeta = 1$）开始，逐步调整。注意 $K_p$ 的物理含义是"加速度/位移"（单位 $1/s^2$），不是力/位移——它是加速度空间的增益，不要和阻抗控制的 $K_d$（力/位移，单位 N/m）混淆。

### QP 求解器选型

| 求解器 | 算法 | 语言 | Warm-start | 稀疏支持 | 推荐场景 |
|--------|------|------|-----------|---------|---------|
| qpOASES | Active-set | C++ | 优秀 | 稠密 | 小规模 QP（DOF < 30） |
| OSQP | ADMM | C | 良好 | 稀疏 | 中大规模，嵌入式 |
| ProxQP | Prox-Newton | C++ | 优秀 | 稀疏+稠密 | Pinocchio 生态首选 |
| ECOS | Interior-point | C | 无 | 稀疏 | 需要 SOCP 时 |

**ProxQP（INRIA 2022）** 是 Pinocchio 团队维护的新 QP 求解器，针对 WBC 场景优化：

```cpp
// ProxQP 使用示例
#include <proxsuite/proxqp/dense/dense.hpp>
using namespace proxsuite::proxqp;

dense::QP<double> qp(n_var, n_eq, n_ineq);
qp.init(H, g, A_eq, b_eq, A_ineq, b_ineq_lower, b_ineq_upper);
qp.solve();

// warm-start: 下一个控制周期用上一步的解作为初始猜测
qp.update(H_new, g_new, A_eq_new, b_eq_new, A_ineq_new, lb_new, ub_new);
qp.solve();  // 从上一步解开始迭代，通常 2-5 步收敛
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：QP 求解失败（infeasible）但不处理
   错误做法：sol = solver.solve(HQPData) 后不检查 sol.status
   现象：QP 返回 infeasible，但代码继续使用上一步的 dv/tau，
        导致状态渐渐偏离可行域，最终机器人崩溃。
   根本原因：当接触状态突变（如一只脚突然离地但约束还在）或
            参考轨迹不可达时，QP 约束不兼容。
   正确做法：
     if (sol.status != 0) {
       // 方案 A: 使用上一步的 tau，但降低增益
       // 方案 B: 移除最低优先级任务，重新求解
       // 方案 C: 切换到安全模式（阻尼控制）
     }
   自检方法：在仿真中故意制造接触状态不一致，验证异常处理逻辑。
```

```
💡 概念误区：认为 QP 的权重越大，任务跟踪越好
   新手想法："把末端跟踪权重设成 10000 不就行了？"
   实际上：权重过大会导致 QP 的条件数恶化（H 矩阵的特征值范围过大），
          求解器精度下降或迭代次数暴增。极端情况下 QP 数值不稳定。
          OSQP 的官方建议是权重比不超过 1000:1。
   正确做法：
     - 权重从 1 开始，按 10 倍递增调试
     - 如果某个任务必须优先，考虑用 HQP 而非加大权重
     - 使用 w_regularization ~1e-4 正则化项改善条件数
```

```
🧠 思维陷阱：直接抄别人的权重参数
   新手想法："论文里用了 w_com=100, w_ee=50, w_posture=1，我也这样设"
   实际上：权重必须根据你的任务重新调。不同机器人的动力学量级不同：
          - 100 kg 的人形和 5 kg 的桌面臂，加速度量级差 20 倍
          - 不同末端执行器的惯量不同，力矩正则化的量级也不同
   正确思维：权重调参的系统方法：
          1. 先只开一个任务（如 com），调 Kp/Kd 让 PD 响应合理
          2. 加第二个任务（如 ee），调权重比让两者都有合理跟踪精度
          3. 逐个加任务，每次只调一个权重
          4. 最后加力矩正则化，从 1e-6 开始逐步增大
```

### 练习

1. ⭐ **QP 维度推导**：对 ANYmal 四足机器人（浮动基座 + 12 关节，4 脚着地，每脚 3D 接触力），使用形式 A 决策变量，推导 QP 的维度：$\dim(z) = ?$, $\dim(A_{eq}) = ?$, 不等式约束总数 = ?（假设 4 面摩擦锥 + 关节力矩限）。
2. ⭐⭐ **KKT 条件分析**：写出加权 QP（单任务：末端跟踪，无不等式约束）的 KKT 条件。证明 KKT 条件的解等价于 $\tau = J^T \Lambda(\ddot{x}_{ref} - \dot{J}v) + h - J_c^T f_c^*$（即阻抗控制律 + 动力学补偿 + 接触力）。
3. ⭐⭐ **权重敏感性实验**：在 TSID Python 示例中，固定 $K_p, K_d$，将末端跟踪权重 $w_{ee}$ 从 1 变到 10000，记录末端跟踪误差和力矩范数。画出 $w_{ee}$ vs 跟踪误差的曲线——是否存在"权重饱和"现象？

---

## 4. TSID 框架详解——Contact6d 与 12 维力表示 ⭐⭐⭐

### 动机——为什么 6D wrench 不适合做决策变量

> 回顾第 2 节：摩擦锥约束要求 $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$，这是一个二阶锥约束。我们用金字塔近似将其线性化，使 QP 可解。但如果接触面是有面积的（如足底），情况更复杂。

考虑一个矩形足底接触（如人形的脚掌）。足底传递的是一个 6D wrench $w = [f_x, f_y, f_z, \tau_x, \tau_y, \tau_z]^T$。如果直接把 6D wrench 作为决策变量，摩擦锥约束仍然是 SOCP。更麻烦的是，还需要约束 ZMP（Zero Moment Point）在足底矩形内：

$$-l_x \leq -\frac{\tau_y}{f_z} \leq l_x, \quad -l_y \leq \frac{\tau_x}{f_z} \leq l_y$$

这涉及 $\tau / f$ 的**除法**，导致约束变成**非线性**的。QP 无法处理。

### TSID 的解决方案——12 维力表示

TSID 用 4 个顶点（矩形足底的 4 个角）的力来**参数化** 6D wrench：

```
矩形足底 4 个顶点:
  p_1 = (+lx, +ly, 0)   顶点 1 的力: f_1 属于 R^3
  p_2 = (+lx, -ly, 0)   顶点 2 的力: f_2 属于 R^3
  p_3 = (-lx, +ly, 0)   顶点 3 的力: f_3 属于 R^3
  p_4 = (-lx, -ly, 0)   顶点 4 的力: f_4 属于 R^3

12 维决策变量: f_12 = [f_1; f_2; f_3; f_4] 属于 R^12

从 12D 到 6D wrench 的映射:
  w = G * f_12

其中 G 属于 R^(6x12) 是力生成矩阵:
  G = [I_3    I_3    I_3    I_3   ]    <-- 力叠加
      [p_1x   p_2x   p_3x   p_4x ]    <-- 力矩 = sum(p_i x f_i)

  p_ix 是 p_i 的反对称矩阵（叉积矩阵）
```

**为什么这个转换解决了问题？**

每个顶点的摩擦锥约束是独立的。若使用下面这种轴向 4 面写法，它是外逼近；安全关键硬约束应替换为 第 2 节中的内逼近或使用更保守的 $\mu_{eff}=\mu/\sqrt{2}$：

$$|f_{i,x}| \leq \mu f_{i,z}, \quad |f_{i,y}| \leq \mu f_{i,z}, \quad f_{i,z} \geq 0 \quad (i = 1,...,4)$$

这是 4 组线性不等式（每组 5 个，共 20 个）。ZMP 约束呢？**自动满足了**。

> **本质洞察**：当所有 4 个顶点的法向力 $f_{i,z} \geq 0$ 时，ZMP 自动落在 4 个顶点的凸包（即足底矩形）内。这是因为 ZMP 是 4 个顶点按法向力加权的重心：
> $$\text{ZMP}_x = \frac{\sum_i p_{i,x} f_{i,z}}{\sum_i f_{i,z}}, \quad \text{ZMP}_y = \frac{\sum_i p_{i,y} f_{i,z}}{\sum_i f_{i,z}}$$
> 凸组合的结果必然在凸包内。所以 $f_{i,z} \geq 0$ **隐式地保证了 ZMP 约束**。

**代价是什么？** 决策变量从 6D 增加到 12D。但线性约束比非线性约束快得多，这个 trade-off 是划算的。

### Contact6d 的 TSID 实现

```python
import tsid
import numpy as np

# 创建面积接触应使用 Contact6d；单点接触才使用 ContactPoint。
# TSID Contact6d 构造函数签名 (tsid v1.7+ 常见形式):
#   Contact6d(name, robot, frame_name, contact_points, contact_normal,
#             mu, f_min, f_max)
# 注意: contact_points 在构造时传入（部分版本也提供 setContactPoints）

lx, ly = 0.1, 0.05  # 足底半长、半宽
contact_points = np.array([
    [+lx, +ly, 0],  # 左前
    [+lx, -ly, 0],  # 右前
    [-lx, +ly, 0],  # 左后
    [-lx, -ly, 0],  # 右后
]).T  # TSID 期望 3xN 矩阵（每列一个顶点）

contact_normal = np.array([0, 0, 1])  # 法向量

try:
    contact_rf = tsid.Contact6d(
        "contact-rf",           # 名称
        robot,                   # RobotWrapper
        "right_foot_frame",      # 接触帧
        contact_points,          # 3x4, 每列一个足底顶点
        contact_normal,          # 法向量
        0.7,                     # 摩擦系数 mu
        1.0,                     # 最小法向力 f_min
        1000.0                   # 最大法向力 f_max
    )
except TypeError:
    # TSID 版本敏感骨架：有些 Python binding 构造函数不接收 contact_points，
    # 需要先构造 Contact6d 再 setContactPoints(contact_points)。
    contact_rf = tsid.Contact6d(
        "contact-rf",
        robot,
        "right_foot_frame",
        contact_normal,
        0.7,
        1.0,
        1000.0
    )
    contact_rf.setContactPoints(contact_points)

# 对比：如果你只有一个点接触，才写 ContactPoint，且不传四角 contact_points。
# 不要把 ContactPoint 和四角 contact_points 混用。
# 注意: TSID 中 Contact6d 和 ContactPoint 的 API 在不同版本间有变化
# 建议查看实际安装版本的 Python bindings：help(tsid.Contact6d)
# contact_points 一般为 3xN numpy 数组；若示例或版本文档写 4x3，需要确认
# binding 是否在内部转置，避免四角顺序和坐标轴被悄悄换掉。

# 设置接触 PD 增益（6D: [wx, wy, wz, vx, vy, vz]）
Kp = np.array([0, 0, 0, 0, 0, 100])  # 只在 z 方向有刚度
Kd = 2.0 * np.sqrt(Kp)
contact_rf.setKp(Kp)
contact_rf.setKd(Kd)

# 设置参考位姿
contact_ref = robot.framePosition(data, robot.model().getFrameId("right_foot_frame"))
contact_rf.setReference(contact_ref)

# 添加到 formulation
formulation.addRigidContact(contact_rf, w_force_reg=1e-5)
# w_force_reg = 1e-5: 力正则化权重 -> 最小化 ||f_c||^2 * 1e-5
```

**为什么力正则化权重很重要？**

没有力正则化时，QP 对接触力的分布是**不确定**的——只要满足约束，任意分布都是最优的（因为代价函数里没有 $f_c$ 项）。这意味着：

- 可能所有力集中在一个顶点 → 实际脚底应力集中 → 磨损不均
- 不同时间步的力分布跳变 → 力矩指令抖动
- 数值问题：QP 有无穷多等优解，求解器可能在解之间"跳跃"

加了 $w_f \|f_c\|^2$ 后，QP 倾向于让力均匀分布在所有顶点上——这在物理上也更合理。

### TSID Contact6d 的力正则化策略详解

力正则化不仅关乎数值稳定性，还直接影响力分布的物理合理性。以下深入分析三种力正则化策略的效果差异。

**策略 A——均匀正则化**（最常用）：

$$\min \|f_{12}\|^2 = \sum_{i=1}^{4} \|f_i\|^2$$

效果：法向力均匀分布在 4 个顶点上（每个顶点承担约 $1/4$ 的总法向力）。ZMP 趋向足底几何中心。

**策略 B——加权正则化**：

$$\min \sum_{i=1}^{4} w_i \|f_i\|^2$$

通过对不同顶点设置不同权重，可以将 ZMP 引向足底的特定位置。例如，前脚掌权重高于后脚跟（$w_{front} > w_{heel}$），ZMP 倾向前移——这在上坡行走中有利于防止后倾。

**策略 C——最小法向力方差正则化**：

$$\min \sum_{i=1}^{4} (f_{i,z} - \bar{f}_z)^2, \quad \bar{f}_z = \frac{1}{4}\sum_i f_{i,z}$$

效果：法向力分布最均匀（最小化方差）。这在柔软地面上特别有用——法向力分布越均匀，足底压强越均匀，地面变形越小。

> **本质洞察**：力正则化的本质是在 QP 可行域内选择一个"偏好解"。不同的正则化策略对应不同的物理偏好——策略 A 最小化接触力总范数（能效优先），策略 B 引导 ZMP 位置（平衡优先），策略 C 最小化地面应力集中（地面保护优先）。选择哪种策略取决于任务需求，没有"最好"的策略。

### 与 6D wrench 决策变量的工程对比

| 对比项 | 6D wrench 变量 | 12D 顶点力变量 |
|--------|--------------|---------------|
| 决策变量维度 | 6 / 接触 | 12 / 接触 |
| 摩擦锥约束 | SOCP（二阶锥） | 线性（金字塔） |
| ZMP 约束 | 非线性（tau/f） | 隐式满足（f_z >= 0） |
| 求解器 | 需要 SOCP 求解器 | QP 即可 |
| 求解速度 | 较慢 | 较快 |
| 力分辨率 | 直接得到 wrench | 需要 w = G * f_12 转换 |

> **不是 X 而是 Y**：12 维力表示**不是**更精确的力模型，**而是**一种让非线性约束变成线性约束的**参数化技巧**。它描述的物理现实完全相同，只是数学表述不同。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Contact6d 的顶点坐标使用了错误的坐标系
   错误做法：用世界坐标系定义 4 个顶点坐标
   现象：当脚的朝向变化时，顶点位置不随脚转动，
        摩擦锥约束方向错误。
   根本原因：TSID 的 Contact6d 期望顶点坐标在接触帧（足底局部坐标系）中定义。
   正确做法：
     contact_points = np.array([
         [+lx, +ly, 0],   # 相对于足底中心的局部坐标
         [+lx, -ly, 0],
         [-lx, +ly, 0],
         [-lx, -ly, 0],
     ]).T  # 3x4：每列一个顶点
```

```
💡 概念误区：认为 12D 力表示意味着足底有 12 个自由度
   新手想法："12 个变量 = 12 个独立的力自由度？"
   实际上：12D 力通过 G 矩阵映射到 6D wrench，所以 rank(G) = 6。
          12 - 6 = 6 个"自由度"是力分布的冗余——
          相同的 wrench 可以由不同的顶点力组合实现。
          力正则化 ||f_12||^2 的作用就是在这 6D 冗余空间中选择最小范数解。
```

### 练习

1. ⭐ **G 矩阵构建**：对一个正方形足底（$l_x = l_y = 0.05$ m），手算力生成矩阵 $G \in \mathbb{R}^{6 \times 12}$。验证 $\text{rank}(G) = 6$。
2. ⭐⭐ **ZMP 隐式保证证明**：给定 4 个顶点力 $f_i = (f_{i,x}, f_{i,y}, f_{i,z})^T$ 且 $f_{i,z} \geq 0$，证明 ZMP 坐标 $(x_{ZMP}, y_{ZMP})$ 落在 4 个顶点的凸包内。
3. ⭐⭐ **SOCP vs QP 性能对比**：在 Python 中用 CVXPY 分别实现 6D wrench SOCP 和 12D 顶点力 QP 两种接触力优化，随机生成 1000 个 wrench 参考值，对比求解时间。

---

## 5. mc_rtc 的四种力控任务 ⭐⭐

### 动机——从"力作为优化变量"到"力作为任务描述"

> 回顾第 3 节：TSID 把力控制嵌入 QP 的代价函数或约束中——力是**优化变量**。mc_rtc 提供了另一种视角：力控制是一种**任务描述**——你告诉框架"我想在这个面上施加 20N 的法向力"，框架自动将其转换为合适的 QP 形式。

mc_rtc（Multi-Contact Real-Time Control）是 CNRS-AIST 联合实验室开发的人形机器人控制框架，被用于 HRP-2, HRP-4, HRP-5P, JVRC-1 等机器人。它的设计哲学是**任务驱动**：用户通过声明式 API 定义任务，框架自动组装 QP。

### 四种力控任务的统一视角

mc_rtc 提供四种力控任务，覆盖了力控场景的完整谱系：

```
力控任务谱系（按控制因果性排列）：

  纯力跟踪 <----------------------------> 纯位置跟踪
  DampingTask  AdmittanceTask  ImpedanceTask   (普通 SurfaceTask)
       |            |              |
       |            |              |
    零刚度        高阻尼+力跟踪    弹簧阻尼器
    纯阻尼        导纳因果性       阻抗因果性
```

| 任务 | 因果性 | 输入 | 输出（传给 QP） | 本质 |
|------|--------|------|----------------|------|
| `DampingTask` | — | 无 | 速度趋0 的约束 | 碰撞后的"刹车" |
| `AdmittanceTask` | 导纳 | 力参考 $F_d$ | 位置参考（由力误差积分） | F01 的导纳控制 |
| `ImpedanceTask` | 阻抗 | 位姿 + 力参考 | 力参考（由位置误差+阻抗） | F03 的阻抗控制 |
| `CoPTask` | — | CoP 目标 | 接触力分布约束 | ZMP/CoP 跟踪 |

**关键理解**：这四种任务**不是独立的 4 种控制算法**，它们都是 F01 二端口网络上不同工作点的实例化。DampingTask 是阻抗控制 $K_d = 0$ 的特例；AdmittanceTask 是 F05 导纳控制的 WBC 版；ImpedanceTask 是 F03 阻抗控制的 WBC 版。

### AdmittanceTask 详解

> 回顾 F05：导纳控制的因果性是"力输入 -> 位移输出"。导纳控制器测量外力，通过导纳传函计算期望末端运动。

mc_rtc 的 `AdmittanceTask` 核心逻辑（简化）：

```cpp
void AdmittanceTask::update(mc_solver::QPSolver & solver) {
    // 1. 获取当前力传感器测量
    sva::ForceVecd w_measured = robot.surfaceWrench(surface_name);
    
    // 2. 计算力误差
    sva::ForceVecd w_error = w_target - w_measured;
    
    // 3. 导纳积分: 力误差 -> 速度参考
    //    v_ref = admittance * w_error
    sva::MotionVecd v_admittance;
    v_admittance.linear() = admittance_force.cwiseProduct(w_error.force());
    v_admittance.angular() = admittance_couple.cwiseProduct(w_error.couple());
    
    // 4. 用速度参考更新目标位姿
    //    x_target += v_admittance * dt
    target_pose = target_pose * sva::PTransformd(v_admittance * dt);
    
    // 5. 将目标位姿传给底层 SurfaceTask（位姿跟踪任务）
    surface_task.target(target_pose);
}
```

**数据流**：

```
力传感器 -> w_measured
           |
力误差: w_error = w_target - w_measured
           |
导纳积分: v_admittance = admittance * w_error
           |
位姿更新: x_target += v_admittance * dt
           |
SurfaceTask（位姿跟踪）-> QP 代价项
           |
QP 求解 -> 关节力矩 tau
```

> **跨领域类比**：AdmittanceTask 像一个"力控制翻译器"——你告诉它"我想施加 20N"，它把这个力需求翻译成位置控制器能理解的语言（"把末端再往前推 0.1mm"）。每个控制周期翻译一次，逐步逼近目标力。这与 F05 中独立的导纳控制器完全相同，只不过底层的位置跟踪从"PD控制"变成了"WBC QP"。

### ImpedanceTask 详解

mc_rtc v2.x（2024 更新）的 `ImpedanceTask` 实现了 SE(3) 几何一致的阻抗控制：

```yaml
# ImpedanceTask YAML 配置
ImpedanceTask:
  surface: RightHand
  # 期望阻抗参数
  stiffness:
    linear: [200, 200, 200]     # N/m
    angular: [20, 20, 20]       # Nm/rad
  damping:
    linear: [28, 28, 28]        # N*s/m (临界阻尼: 2*sqrt(K*m))
    angular: [9, 9, 9]          # Nm*s/rad
  # 期望 wrench（前馈力）
  wrench:
    force: [0, 0, -10]          # 10N 向下推力
    couple: [0, 0, 0]
  # 位姿参考
  target:
    translation: [0.5, 0.0, 0.3]
    rotation: [1, 0, 0, 0]      # 四元数 w,x,y,z
```

**SE(3) 几何一致性的含义**：

旋转误差不是简单的欧拉角差值（会有万向节锁），而是用 SO(3)/SE(3) 对数映射计算。下面代码采用 **current-to-target** 约定，即小角度下对应 $\text{Log}(R^T R_d)$；因此后续可以写成 $+K e$。如果改用 target-to-current 约定 $\text{Log}(R_d^T R)$，恢复项就必须同步改成 $-K e$。

```cpp
// SE(3) 阻抗控制律（mc_rtc ImpedanceTask 核心）
sva::ForceVecd F_impedance;

// 位置误差: Log map on SE(3)
// 这里定义误差为 current -> target，因此正的 K*error 产生恢复方向的 wrench。
sva::PTransformd X_error = X_current.inv() * X_target;
Eigen::Matrix<double, 6, 1> pose_error =
    sva::transformVelocity(X_error);

// 速度误差
Eigen::Matrix<double, 6, 1> vel_error = v_target - v_current;

// 阻抗力 = K * pose_error + D * vel_error + F_feedforward
F_impedance = K.cwiseProduct(pose_error) + D.cwiseProduct(vel_error) + F_ff;
```

如果项目中采用相反定义 `X_target.inv() * X_current`（target -> current），恢复项必须同步改成 `-K * pose_error`。SE(3) 误差的方向和控制律符号必须成对出现，否则自由空间姿态偏差会被越推越远，表现为正反馈。

### ImpedanceTask 的 SE(3) 误差计算深入分析

SE(3) 阻抗控制的核心难点在于旋转误差的计算。与平移误差（简单的向量差 $\Delta p = p_d - p$）不同，旋转误差必须在李群 SO(3) 上定义——否则会遇到万向节锁（Gimbal Lock）或角度不连续的问题。

**三种旋转误差定义及其等价性分析**：

| 定义方式 | 数学表达 | 优点 | 缺点 |
|---------|---------|------|------|
| 欧拉角差值 | $\Delta\phi = \phi_d - \phi$ | 直觉简单 | 万向节锁、$360^\circ$ 不连续 |
| 四元数误差 | $q_e = q_d \otimes q^{-1}$ | 无奇异 | 双覆盖问题（$q$ 和 $-q$ 表示同一旋转） |
| 对数映射 | $\omega_e = \text{Log}(R^T R_d)$ | SO(3) 上唯一、光滑 | 在 $\theta = \pi$ 处需要特殊处理 |

mc_rtc 的 ImpedanceTask 使用**对数映射**方式，这也是 Pinocchio 的内建选择。对数映射将旋转误差从 SO(3) 映射到其李代数 $\mathfrak{so}(3) \cong \mathbb{R}^3$——一个线性空间，可以直接乘以增益矩阵 $K_p$。

**对数映射的物理含义**：$\omega_e = \text{Log}(R^T R_d) = \theta \hat{n}$，其中 $\theta$ 是从当前姿态到目标姿态的最短旋转角度，$\hat{n}$ 是旋转轴。$K_p \omega_e$ 产生的力矩方向就是沿这个旋转轴——这是物理上最自然的恢复力方向。

> **反事实推理**：如果使用欧拉角差值作为旋转误差会怎样？当 $pitch \approx \pm 90^\circ$ 时（万向节锁），欧拉角的 $roll$ 和 $yaw$ 产生歧义——同一个旋转对应无穷多组欧拉角。阻抗控制律会产生不确定的力矩输出，表现为末端在该姿态附近剧烈抖动。对数映射不存在这个问题（除了 $\theta = \pi$ 的边界情况）。

### CoPTask——压力中心跟踪

`CoPTask` 用于控制足底压力中心（Center of Pressure, CoP）的位置。这在步态控制中至关重要——CoP 必须在支撑多边形内才能维持平衡。

```
CoP 的物理定义：
  CoP = 地面反力合力的作用点
  如果 CoP 在支撑多边形内 -> 静平衡（ZMP 条件满足）
  如果 CoP 在支撑多边形边界 -> 即将倾翻
  如果 CoP 在支撑多边形外 -> 不可能（定义上 CoP 必须在接触区域内）

CoPTask 的控制目标：
  让 CoP 跟踪一个参考轨迹（通常来自步态规划器）
  
实现方式：
  CoP = (-tau_y/f_z, tau_x/f_z) <-- 由 6D wrench 计算
  通过调整接触力分布（12D 顶点力），使 CoP 趋向目标
```

### DampingTask——碰撞后安全缓冲

`DampingTask` 是最简单的力控任务——它只做一件事：阻尼末端速度，让末端慢慢停下来。

```
使用场景：
  1. 碰撞检测触发后：立即切换到 DampingTask，吸收动能
  2. 接触过渡：从自由空间进入接触前，降低速度
  3. 紧急停止：作为安全策略的最后一道防线

数学形式：
  F_damping = -D * v_ee    <-- 纯阻尼，无刚度
  
等价于阻抗控制 K=0 的特例
```

### Admittance 样例——JVRC-1 推墙

```yaml
# mc_rtc 教程: JVRC-1 推墙到 -20 N
# 文件: sample-admittance.yaml

MainRobot: JVRC1
Enabled: [Posture, CoM, Admittance]

Posture:
  stiffness: 5.0
  weight: 10.0

CoM:
  stiffness: 20.0
  weight: 1000.0
  above:
    contacts: [LeftFoot, RightFoot]

Admittance:
  type: admittance
  surface: RightGripper
  wrench:
    force: [0, 0, -20]    # 目标法向力 -20 N
    couple: [0, 0, 0]
  admittance:
    force: [0, 0, 0.001]  # z 轴导纳增益
    couple: [0, 0, 0]
  stiffness: 5.0
  damping: 300.0
  weight: 500.0
```

**参数解读**：

| 参数 | 值 | 物理含义 | 调参指南 |
|------|-----|---------|---------|
| `wrench.force[2]` | -20 N | 目标法向力 | 根据任务需求设定 |
| `admittance.force[2]` | 0.001 | 导纳增益 $a_z$（m/s/N） | 越大响应越快但可能振荡 |
| `stiffness` | 5.0 | 位姿跟踪 $K_p$（底层 PD） | 越大跟踪越紧但力过渡越硬 |
| `damping` | 300.0 | 位姿跟踪 $K_d$（底层 PD） | 高阻尼防振荡 |
| `weight` | 500.0 | QP 权重 | 相对于 Posture(10) 和 CoM(1000) |

**导纳增益 0.001 的含义**：当力误差为 20 N 时，末端速度参考 $v_{ref} = 0.001 \times 20 = 0.02$ m/s（2 cm/s）。末端以 2 cm/s 的速度向墙壁移动，直到接触力达到 20N。

> **反事实推理**：如果导纳增益设得太大（如 0.01），末端会以 20 cm/s 的速度冲向墙壁——接触瞬间的力冲击很大。如果设得太小（如 0.0001），末端以 0.2 cm/s 的速度移动——达到目标力需要很长时间。所以导纳增益本质上是**速度-安全性的 trade-off**。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：AdmittanceTask 和 ImpedanceTask 混用
   错误做法：对同一个表面同时添加 AdmittanceTask 和 ImpedanceTask
   现象：两个任务的目标位姿冲突，QP 在两者之间"拉锯"，末端抖动。
   根本原因：两种任务都会修改同一个表面的位姿参考，互相覆盖。
   正确做法：一个表面只能有一种力控任务。
            需要切换时，先移除旧任务再添加新任务。
```

```
💡 概念误区：认为 AdmittanceTask 的导纳增益 = F05 的导纳参数
   新手想法："F05 的导纳传函是 Y(s) = 1/(Ms^2 + Ds + K)，
            所以 mc_rtc 的 admittance 参数就是 Y(s) 的某个系数"
   实际上：mc_rtc 的 admittance 是一个简化的比例增益：v = a * Delta_f，
          不是完整的二阶导纳模型。它更像 P 控制器（比例导纳），
          而 F05 的导纳是完整的 MDK 二阶模型。
          mc_rtc 选择简化模型是因为 WBC 的 QP 已经提供了动力学补偿。
   正确理解：mc_rtc 的 admittance 是力误差到表面速度参考的外层比例增益，
          再经过目标位姿积分交给底层任务。暂态由外层比例/积分、底层任务 PD、
          QP 权重和机器人闭环共同决定，
          不是 F05 那种显式 MDK 二阶导纳暂态。
```

```
🧠 思维陷阱：认为 weight 越大力控精度越高
   新手想法："我想要精确的 20N 力控，把 Admittance 的 weight 设到最大"
   实际上：力控精度主要取决于导纳增益和力传感器精度，不取决于 QP 权重。
          权重只决定位姿跟踪的优先级——太高会让力控任务"绑架"其他任务
          （如 CoM 平衡），导致机器人为了追力而失去平衡。
   正确思维：weight 应该在 CoM（安全）和 Posture（舒适）之间：
          CoM > Admittance > Posture，即 1000 > 500 > 10。
```

### 练习

1. ⭐ **任务选型**：对以下场景分别选择 mc_rtc 的哪种力控任务：(a) 人形右手擦桌子（恒力打磨），(b) 站立时平衡控制，(c) 碰撞检测后安全响应，(d) 右手拧螺丝（位姿+力矩）。
2. ⭐ **导纳增益调参**：如果力传感器有 2N 的噪声（$\sigma = 2$N），导纳增益 $a = 0.001$ m/s/N，计算由噪声引起的末端位置抖动幅度（1kHz 控制频率）。如果抖动超过 0.5mm 的容忍值，应如何调整？
3. ⭐⭐ **mc_rtc 安装与运行**：在 Ubuntu 22.04 上安装 mc_rtc，运行 Admittance 推墙样例。修改力参考从 -20N 到 -50N，记录力跟踪的稳态误差。

---

## 6. 与固定基座阻抗控制的统一视角 ⭐⭐⭐

### 动机——WBC 真的是全新的东西吗？

学到这里，你可能觉得 WBC 和 F03 的阻抗控制是完全不同的两个世界。但实际上，它们有深刻的数学联系。理解这种联系不仅能帮你更好地理解 WBC，还能让你在设计控制器时知道"什么时候用什么"。

### 统一推导

**F03 的笛卡尔阻抗控制**（固定基座，单任务）：

$$\tau = J^T\left(\Lambda\ddot{x}_d + \mu + p + K_d e_x + D_d \dot{e}_x\right), \quad e_x=x_d-x$$

其中 $\Lambda = (JM^{-1}J^T)^{-1}$, $\mu = \Lambda J M^{-1} C\dot{q} - \Lambda \dot{J}\dot{q}$, $p = \Lambda J M^{-1} g$。注意 $\mu$ 已经包含 $-\Lambda\dot{J}\dot{q}$，因此 $\Lambda$ 前馈项只写 $\Lambda\ddot{x}_d$；若改用 $\mu_C=\Lambda J M^{-1}C\dot{q}$，才等价写成 $J^T[\Lambda(\ddot{x}_d-\dot{J}\dot{q})+\mu_C+p+K_d e_x+D_d\dot{e}_x]$。

**WBC 加权 QP**（浮动基座、多任务；下面取固定基座单任务情形做对比）：

$$\min_{\dot{v}, \tau, f_c} \|J\dot{v} + \dot{J}v - \ddot{x}_{ref}\|^2 + w_\tau \|\tau\|^2$$
$$\text{s.t. } M\dot{v} + h = S^T\tau + J_c^T f_c$$

其中 $\ddot{x}_{ref} = K_p e_x + K_d \dot{e}_x + \ddot{x}_d$（PD 加速度参考）。

**关系分析**——在强约束条件下与阻抗控制对比：

条件 1：固定基座 → $\dot{v}_b = 0$, $M$ 退化为 $M_{jj}$, $S = I$

条件 2：无接触力 → $f_c = 0$, $J_c^T f_c$ 项消失

条件 3：单任务 → 只有一个末端跟踪任务

条件 4：$w_\tau \to 0$ → 不惩罚力矩

在这四个条件下，如果代价只惩罚任务空间加速度误差，QP 的拉格朗日函数为：

$$L = \|J\ddot{q} + \dot{J}\dot{q} - \ddot{x}_{ref}\|^2 + \lambda^T(M\ddot{q} + h - \tau)$$

对 $\ddot{q}$ 和 $\tau$ 分别求导，令梯度为零：

$$\nabla_{\ddot{q}} L = 2J^T(J\ddot{q} + \dot{J}\dot{q} - \ddot{x}_{ref}) + M^T\lambda = 0$$
$$\nabla_{\tau} L = -\lambda = 0 \implies \lambda = 0$$

代入第一个方程，$J^T(J\ddot{q} + \dot{J}\dot{q} - \ddot{x}_{ref}) = 0$。这意味着 $\ddot{q}$ 使得任务误差在 $J$ 列空间上被消除——即 $\ddot{q} = J^+(\ddot{x}_{ref} - \dot{J}\dot{q})$。

代入动力学约束：

$$\tau = M J^+(\ddot{x}_{ref} - \dot{J}\dot{q}) + h$$

展开 $\ddot{x}_{ref} = K_p e_x + K_d \dot{e}_x$：

$$\tau = M J^+(K_p e_x + K_d \dot{e}_x - \dot{J}\dot{q}) + C\dot{q} + g$$

这与 F03 的阻抗控制律具有相同的"PD 加速度参考 + 逆动力学"结构，但不能简单说二者总是等价。上式使用的是 Moore-Penrose 伪逆 $J^+$；若代价采用动力学一致度量，才会出现 $\bar{J}=M^{-1}J^T\Lambda$，并进一步接近操作空间控制中的 $J^T\Lambda(\ddot{x}_{ref}-\dot{J}\dot{q})$ 形式。实际 WBC 还会受到力矩正则化、约束活动集和接触力变量的影响。

> **本质洞察**：WBC 不是"新发明"的力控方法，而是阻抗控制在以下三个方向上的**自然推广**：
> 1. 从固定基座推广到浮动基座 → 引入 $S^T\tau$ 和 $J_c^T f_c$
> 2. 从单任务推广到多任务 → 引入权重/优先级
> 3. 从无约束推广到有约束 → 引入 QP 求解器
>
> 理解了这种推广关系，你就知道 WBC 的每个组件"为什么在那里"。

### 统一视角表

| 特性 | F03 阻抗控制 | WBC 加权 QP | WBC HQP |
|------|-------------|------------|---------|
| 基座 | 固定 | 浮动 | 浮动 |
| 任务数 | 1 | N（加权） | N（严格优先级） |
| 接触力 | 忽略 | 优化变量 | 优化变量 |
| 约束 | 无 | 摩擦锥+力矩限 | 摩擦锥+力矩限 |
| 求解 | 解析公式 | 1 个 QP | N 个 QP |
| 计算量 | ~10 us | ~0.1-1 ms | ~0.5-5 ms |
| 实时性 | 10 kHz+ | 500-1000 Hz | 200-500 Hz |

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为 WBC 完全取代了阻抗控制
   新手想法："既然 WBC 是阻抗控制的超集，那以后只学 WBC 就行了"
   实际上：在固定基座、单任务、无约束的场景下，
          阻抗控制比 WBC 简单 10 倍、快 100 倍、更容易调参。
          WBC 的额外能力（多任务、约束）伴随着额外的复杂度。
          很多工业场景（Franka 力控、协作机器人力导引）
          直接用 F04 的阻抗控制就完美解决了。
   正确思维：工具选择应匹配问题复杂度。
          固定基座单任务 -> F03/F04 阻抗控制
          固定基座多任务 -> 零空间投影（F02 第 5 节）
          浮动基座 -> WBC
```

### 练习

1. ⭐⭐ **跨章综合题**：用 F02 的操作空间动力学 + F03 的阻抗控制律 + 第 3 节的 WBC-QP 框架，手动组装一个固定基座 7-DOF 臂的简化 WBC-QP（单任务、无接触力、无不等式约束）。写出 QP 的 $H$, $g$, $A_{eq}$, $b_{eq}$ 矩阵，并证明其 KKT 解与 F03 的闭式阻抗控制律等价。
2. ⭐⭐⭐ **WBC 作为统一框架**：设计一个同时包含位控任务（末端到达目标点）和力控任务（沿法向维持 10N）的 WBC-QP。画出 QP 的完整数学形式。与 F03 的 Raibert-Craig 力位混合控制对比——两者的选择矩阵 $S$ 有什么区别？

---

## 7. Pinocchio TSID API 实战 ⭐⭐

### 动机——从理论到可运行代码

前面第 1-6 节建立了 WBC 的完整理论框架。本节的目标是**将理论变成代码**——用 TSID Python API 实现一个完整的 WBC 示例。

### 环境搭建

```bash
# 安装 Pinocchio + TSID（推荐 conda）
conda install -c conda-forge pinocchio tsid meshcat-python example-robot-data

# 验证安装
python3 -c "import pinocchio; import tsid; print('OK')"
```

### 完整示例：7-DOF 臂的 WBC

```python
"""
TSID 示例: 7-DOF Franka Panda 的加权 QP WBC
任务:
  1. 末端位姿跟踪 (权重 100)
  2. 关节正则化 (权重 1)
  3. 力矩限 (硬约束)
"""
import numpy as np
import pinocchio as pin
from example_robot_data import load

# ===== 1. 加载模型 =====
robot_wrapper = load("panda")
model = robot_wrapper.model
data = model.createData()
q0 = robot_wrapper.q0.copy()
v0 = np.zeros(model.nv)

# ===== 2. 创建 TSID =====
import tsid
robot = tsid.RobotWrapper(
    robot_wrapper.urdf,
    [robot_wrapper.model_path],
    False                         # False = 固定基座
)

formulation = tsid.InverseDynamicsFormulationAccForce(
    "panda_wbc", robot, False
)
formulation.computeProblemData(0.0, q0, v0)

# ===== 3. 末端位姿跟踪任务 =====
ee_frame = "panda_hand"
ee_task = tsid.TaskSE3Equality("ee_task", robot, ee_frame)
ee_task.setKp(100.0 * np.ones(6))
ee_task.setKd(2.0 * np.sqrt(100.0) * np.ones(6))  # 临界阻尼
ee_task.useLocalFrame(False)

# 目标: 末端向前移动 10cm
pin.forwardKinematics(model, data, q0)
pin.updateFramePlacements(model, data)
ee_frame_id = model.getFrameId(ee_frame)
H_init = data.oMf[ee_frame_id]
H_target = H_init.copy()
H_target.translation[0] += 0.1

ref = tsid.TrajectorySE3Constant("ee_ref", H_target)
ee_task.setReference(ref.computeNext())
formulation.addMotionTask(ee_task, w=100.0, level=1, transition_duration=0.0)

# ===== 4. 关节正则化任务 =====
posture_task = tsid.TaskJointPosture("posture_task", robot)
posture_task.setKp(10.0 * np.ones(robot.nv))
posture_task.setKd(2.0 * np.sqrt(10.0) * np.ones(robot.nv))
q_ref = tsid.TrajectoryEuclidianConstant("q_ref", q0)
posture_task.setReference(q_ref.computeNext())
formulation.addMotionTask(posture_task, w=1.0, level=1, transition_duration=0.0)

# ===== 5. 力矩限约束 =====
tau_max = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)  # Panda: J1-4=87Nm, J5-7=12Nm
tau_min = -tau_max
actuator_bounds = tsid.TaskActuationBounds("tau_bounds", robot)
actuator_bounds.setBounds(tau_min, tau_max)
formulation.addActuationTask(actuator_bounds, w=1.0, level=0, transition_duration=0.0)

# ===== 6. 求解 =====
solver = tsid.SolverHQuadProg("qp_solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

dt = 0.001       # 1 kHz
N_steps = 3000   # 3 秒

q = q0.copy()
v = v0.copy()
ee_pos_log = []
tau_log = []

for i in range(N_steps):
    t = i * dt
    HQPData = formulation.computeProblemData(t, q, v)
    sol = solver.solve(HQPData)
    if sol.status != 0:
        print(f"QP infeasible at t={t:.3f}!")
        break
    
    dv = formulation.getAccelerations(sol)
    tau = formulation.getActuatorForces(sol)
    
    # formulation.computeProblemData() 更新的是 TSID/RobotWrapper 内部 data；
    # 本地 pinocchio data 也必须同步，否则 data.oMf 仍停留在初始化姿态。
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    ee_pos_log.append(data.oMf[ee_frame_id].translation.copy())
    tau_log.append(tau.copy())
    
    # Lie 群积分
    v += dv * dt
    q = pin.integrate(model, q, v * dt)

print(f"Final EE pos: {ee_pos_log[-1]}")
print(f"Target pos: {H_target.translation}")
print(f"Error: {np.linalg.norm(ee_pos_log[-1] - H_target.translation):.4f} m")
```

**为什么用 `pin.integrate()` 而不是 `q += v * dt`？**

因为浮动基座的 $q$ 包含四元数（SO(3) 上的元素）。简单的加法会破坏四元数单位范数约束。`pin.integrate()` 使用 Lie 群积分，保证 $q$ 始终在流形上。对固定基座虽然 $q \in \mathbb{R}^n$ 无此问题，但用 `pin.integrate()` 是好习惯。

### TSID 任务切换与接触过渡

在动态任务中，接触状态会随时间变化（如步态切换、抬起/放下物体）。TSID 通过 `addRigidContact` 和 `removeRigidContact` 管理接触状态。但接触切换时有一个重要的工程细节：**不能在同一控制周期内同时移除旧接触和添加新接触**。

原因是移除接触后，该接触点的力变量从 QP 中消失，而新接触的力变量尚未初始化——QP 的维度突变可能导致 warm-start 失效，求解器从"冷启动"开始，耗时显著增加甚至 infeasible。

**推荐的接触切换流程**：

1. 在接触切换前 $T_{transition}$（典型 50-100 ms），开始降低该接触的力权重（或增大力正则化权重），让力平滑趋近零
2. 当接触力降至阈值以下（如 $\|f_c\| < 5$ N）时，移除接触约束
3. 等待 1-2 个控制周期让 QP 稳定
4. 添加新接触，初始力权重为零（或很小），然后在 $T_{transition}$ 内逐步增大到正常值

> **类比**：接触切换就像飞机换跑道降落——不能在空中突然从一条跑道"跳"到另一条，必须先从当前跑道拉起（减小力），在空中调整方向（过渡期），然后对准新跑道降落（建立新接触）。

### 扩展到浮动基座

将固定基座示例改为浮动基座只需要 3 处修改：

```python
# 修改 1: 加载浮动基座模型
robot = tsid.RobotWrapper(
    urdf_path,
    package_dirs,
    pin.JointModelFreeFlyer()    # 使用 FreeFlyer 基座
)

# 修改 2: q 和 v 的维度
# q 属于 R^(7+n): 前 7 个是 [position(3), quaternion(4)]
# v 属于 R^(6+n): 前 6 个是 [linear_vel(3), angular_vel(3)]

# 修改 3: 添加接触约束
lx, ly = 0.1, 0.05
foot_vertices = np.array([
    [+lx, +ly, 0],
    [+lx, -ly, 0],
    [-lx, +ly, 0],
    [-lx, -ly, 0],
]).T  # 3x4, 每列一个足底顶点（接触帧局部坐标）
try:
    contact_rf = tsid.Contact6d("rf_contact", robot, "right_foot",
                                foot_vertices,
                                np.array([0, 0, 1]),  # 法向
                                0.7, 1.0, 1000.0)     # mu, f_min, f_max
except TypeError:
    contact_rf = tsid.Contact6d("rf_contact", robot, "right_foot",
                                np.array([0, 0, 1]),
                                0.7, 1.0, 1000.0)
    contact_rf.setContactPoints(foot_vertices)
contact_rf.setKp(np.zeros(6))
contact_rf.setKd(np.zeros(6))
formulation.addRigidContact(contact_rf, 1e-5)
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：TSID TaskSE3Equality 默认使用 LOCAL 帧
   错误做法：不调 useLocalFrame(False)
   现象：旋转误差在末端局部帧中计算，
        当末端旋转较大时，误差方向不直观。
   正确做法：
     ee_task.useLocalFrame(False)  # 世界帧误差
```

```
⚠️ 编程陷阱：TrajectoryConstant 不会动态更新
   错误做法：使用 TrajectoryConstant 后想动态改变目标但忘了重新设置
   现象：目标始终是初始值
   正确做法：动态目标使用 TrajectorySE3Cubic 或手动每步更新参考
```

### 练习

1. ⭐ **TSID 入门**：运行上述 7-DOF 臂示例。修改目标位姿（y 方向移动 20cm），记录末端轨迹和力矩曲线。阻尼比从 1.0 变到 0.5 时有什么变化？
2. ⭐⭐ **权重实验**：将末端任务权重从 10 变到 1000，记录末端跟踪精度、关节运动幅度、QP 求解时间。
3. ⭐⭐ **多任务冲突**：添加第二个任务——某关节角度跟踪。当末端任务和关节角度任务冲突时，观察加权 QP 的折中行为。

---

## 8. 移动操作（Loco-Manipulation）接口设计 ⭐⭐⭐

### 动机——当底盘和手臂必须协同

移动操作是 WBC 在机械臂领域最直接的应用场景：一个移动平台搭载机械臂执行操作任务。关键挑战在于底盘运动和手臂运动必须协调。

### 任务分配策略

```
策略 A: 时序分离（先移后操）
  Phase 1: 底盘移动到目标附近
  Phase 2: 底盘锁定，手臂执行操作
  优点: 简单，控制器独立
  缺点: 不能边走边操作

策略 B: 空间分离（频率解耦）
  底盘控制: 低频（10-50 Hz）导航栈
  手臂控制: 高频（500-1000 Hz）阻抗/WBC
  优点: 可同时运动
  缺点: 底盘急转弯时手臂跟踪延迟

策略 C: 全身统一 WBC
  底盘 DOF + 手臂 DOF 统一优化
  优点: 最优协调
  缺点: 计算量大，需要底盘动力学模型
```

| 策略 | 适用场景 | 代表系统 |
|------|---------|---------|
| A 时序分离 | 仓库 pick-and-place | 多数工业 AMR |
| B 空间分离 | 服务机器人 | TIAGo, HSR |
| C 全身 WBC | 研究前沿 | ANYmal+DynaArm, CENTAURO |

### 策略 C 的 WBC 框架

```
移动操作 WBC-QP:

决策变量: z = [v_dot_base(3 or 6); q_ddot_arm(n); tau_arm(n); f_c(3k)]

等式约束:
  M [v_dot_base; q_ddot_arm] + h = [tau_base; tau_arm] + J_c^T f_c
  J_c [v_dot_base; q_ddot_arm] + J_dot_c v = 0  (如果有接触)

代价函数:
  w_ee ||J_ee [v_dot_base; q_ddot_arm] - x_ddot_ee_ref||^2   <-- 末端跟踪
  + w_base ||v_base - v_base_ref||^2                          <-- 底盘速度跟踪
  + w_posture ||q_ddot_arm - q_ddot_arm_reg||^2               <-- 关节正则化
  + w_tau ||tau_arm||^2                                        <-- 力矩正则化

特殊之处:
  - 末端雅可比 J_ee 同时包含底盘和手臂的贡献:
    J_ee = [J_ee_base, J_ee_arm]
    底盘运动也能改变末端位置
```

### 底盘-手臂耦合动力学分析

移动操作中一个容易被忽视的关键问题是**底盘运动对手臂末端力控精度的影响**。当底盘产生加速度 $\ddot{p}_{base}$ 时，手臂末端会受到惯性力：

$$F_{inertial,ee} \approx m_{arm} \cdot \ddot{p}_{base}$$

对于 7 kg 的机械臂（如 Franka Panda），底盘以 $0.5 \text{ m/s}^2$ 加速时，惯性力约为 3.5 N。如果此时末端正在执行 1 N 精度的力控任务，惯性力将完全淹没力控信号。

更精确的分析需要考虑手臂的动力学耦合效应。在完整动力学方程中，底盘加速度通过质量矩阵的耦合块 $M_{bj}$ 影响手臂关节：

$$M_{jb}\ddot{p}_{base} + M_{jj}\ddot{q}_{arm} + h_j = \tau_{arm}$$

$M_{jb}\ddot{p}_{base}$ 是底盘对手臂的惯性耦合力矩——如果 WBC 的动力学模型正确计算了这一项，它可以通过力矩补偿来抵消。但如果使用策略 B（空间分离），手臂控制器**看不到底盘的加速度**，无法补偿这个耦合——这就是策略 C（全身 WBC）在精密力控场景下优于策略 B 的根本原因。

**工程权衡**：精密力控（<5 N 精度）时使用策略 C 或限制底盘加速度上限（如 $\|\ddot{p}_{base}\| \leq 0.1 \text{ m/s}^2$）。粗力控或纯位控时策略 B 足够。

> **类比**：底盘-手臂耦合就像在行驶的汽车上做精密手工——如果汽车匀速行驶（$\ddot{p} = 0$），你可以正常操作；如果汽车急刹车或急转弯（$\ddot{p}$ 大），你的手会被惯性力带偏。WBC 策略 C 相当于"考虑了汽车运动的精密机器人手"——它预知底盘会加速，提前调整手臂力矩来补偿。

### ROS2 接口设计

```cpp
// 移动操作 ROS2 接口（概念性）
class LocoManipulationController : public controller_interface::ControllerInterface {
public:
    // 输入接口: base_position(3), base_orientation(4), arm_joints(n)
    std::vector<hardware_interface::LoanedStateInterface> joint_state;
    
    // 输出接口: base_velocity(3), arm_torques(n)
    std::vector<hardware_interface::LoanedCommandInterface> joint_command;
    
    // 订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_target_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sensor_sub;
    
    // WBC 求解器
    std::unique_ptr<WBCSolver> wbc_solver;
    
    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override {
        
        auto [q, v] = readState();
        auto ee_target = getEETarget();
        
        wbc_solver->setEETarget(ee_target);
        auto [base_vel_cmd, arm_tau] = wbc_solver->solve(q, v);
        
        writeBaseVel(base_vel_cmd);
        writeArmTorque(arm_tau);
        return controller_interface::return_type::OK;
    }
};
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：末端雅可比忘记包含底盘贡献
   错误做法：J_ee = getArmJacobian()  // 只有手臂部分
   现象：底盘移动时末端位置变化，但控制器不知道——
        末端跟踪出现与底盘运动成正比的稳态误差。
   正确做法：J_ee = getFullJacobian()  // 包含 base + arm
```

```
💡 概念误区：认为底盘和手臂可以完全解耦
   新手想法："底盘负责走，手臂负责抓，互不干扰"
   实际上：底盘加速度产生惯性力，通过手臂结构传递到末端。
          如果手臂正在做精细力控（如 1N 精度），底盘突然加速 ->
          手臂末端受到数牛顿惯性力 -> 力控精度崩溃。
   正确做法：精细力控阶段应限制底盘加速度。
```

### 练习

1. ⭐ **雅可比分析**：对 3-DOF 全向底盘 + 7-DOF 臂系统，推导末端雅可比 $J_{ee} \in \mathbb{R}^{6 \times 10}$ 的结构。哪些列对应底盘？哪些对应手臂？
2. ⭐⭐ **策略对比**：在 MuJoCo 中搭建简化底盘+臂模型，分别实现策略 A 和策略 C，对比末端到达时间和力矩消耗。
3. ⭐⭐⭐ **跨章综合题**：结合 F01（阻抗/导纳选型）、F03（阻抗控制律）、F07（WBC-QP），为 ANYmal 四足+6-DOF 臂设计"推门"任务的完整控制架构。画出控制框图，标注每个模块的输入/输出、频率和算法选择。

---

## 9. 前沿展望——Centroidal MPC + WBC 联合优化与 Whole-Body MPC ⭐⭐⭐⭐

前八节建立了从操作空间视角出发的浮动基座 WBC 理论。本节将视野拓展到 MPC 与 WBC 的联合优化前沿——这一方向同时汇聚了机械臂与足式两个领域的进展。

> **跨方向视角提示**：足式方向（足式/90_WBC分层优化与TSID）侧重 Centroidal 动力学视角——以质心动量和接触力为核心变量，关注行走平衡与步态切换；本章则从操作空间视角出发——以末端位姿和交互力为核心变量，关注精确力控与多任务协调。两者共享相同的浮动基座动力学方程 $M\dot{v} + h = S^T\tau + J_c^T f_c$，但任务优先级和代价函数的设计哲学不同。在四足+臂等复合平台上（F08），两种视角需要统一。

### 传统分层架构的局限

传统的 MPC+WBC 架构（F08 将详细展开）采用严格分层：MPC 使用简化模型（如单刚体 SRBD）生成质心参考轨迹和接触力，WBC 在全身动力学下跟踪这些参考。这种分层的优势是计算高效（MPC 只需求解低维 QP），但代价是**层间最优性损失**——MPC 生成的参考可能在全身动力学下不可行或远非最优。

> **反事实推理**：如果 MPC 的简化模型预测质心在 0.5 秒后到达某位置，但全身动力学下该位置需要的关节速度超出限制，会怎样？WBC 被迫在力矩限约束下"尽力而为"，导致跟踪误差累积。极端情况下，MPC 参考与 WBC 能力之间的 gap 会导致系统不稳定。

### Centroidal MPC + WBC 联合优化

近年来，Centroidal MPC（基于质心动力学的 MPC）逐渐成为弥补这一 gap 的标准方案。与 SRBD 不同，Centroidal 模型保留了质心的动量方程和角动量方程，同时通过 centroidal 动力学（Orin & Goswami 2008）将全身惯性参数映射到质心空间。

**Centroidal 动力学方程**：

$$\dot{h} = \begin{bmatrix} \dot{l} \\ \dot{k} \end{bmatrix} = \begin{bmatrix} m\ddot{c} \\ \dot{k} \end{bmatrix} = \sum_{i=1}^{n_c} \begin{bmatrix} f_i \\ (p_i - c) \times f_i \end{bmatrix} + \begin{bmatrix} mg \\ 0 \end{bmatrix}$$

其中 $h = [l; k]$ 是质心线动量和角动量，$c$ 是质心位置，$p_i$ 是第 $i$ 个接触点位置，$f_i$ 是接触力。与 SRBD 相比，Centroidal 模型的关键改进在于**角动量 $k$ 不再假设躯干绕质心的惯性矩不变**——它捕捉了腿部摆动引起的角动量变化。

**联合优化的核心思想**：将 Centroidal MPC 的预测与 WBC 的约束在同一优化问题中耦合。具体做法有两种：

| 方法 | 代表工作 | 核心思路 | 计算开销 |
|------|---------|---------|---------|
| 迭代耦合 | Romualdi et al. 2022 (BLF/iCub) | Centroidal MPC 迭代更新 WBC 参考，WBC 反馈可行性修正给 MPC | 中等（两个 QP 交替求解） |
| 单层全身 MPC | Dantec et al. 2024 (Whole-Body MPC, mc_rtc) | 直接在全身动力学上做 MPC，消除分层 gap | 高（需要高效 NLP 求解器，如 ProxDDP） |

### Whole-Body MPC——消除分层的终极目标

Whole-Body MPC (WB-MPC) 的核心理念是**把 WBC 的全身动力学约束直接嵌入 MPC 的预测模型中**，从而消除 MPC 与 WBC 之间的模型不一致问题。这相当于在预测时域的每个时间步都求解一个 WBC 级别的优化，形成一个嵌套优化或单层大规模 NLP。

**WB-MPC 的数学形式**（简化表示）：

$$\min_{\mathbf{x}_{0:N}, \mathbf{u}_{0:N-1}} \sum_{k=0}^{N} \ell_k(x_k, u_k) \quad \text{s.t.} \quad M(q_k)\dot{v}_k + h_k = S^T\tau_k + J_c^T f_{c,k}, \quad \text{摩擦锥、力矩限、接触约束}$$

其中状态 $x_k = [q_k; v_k]$ 是全身状态（维度可达 50-80），而非简化的 13 维质心状态。

近年来的关键使能技术包括：Carpentier et al. 2024 的 ProxDDP 算法（基于增广 Lagrangian 的 DDP 变体，支持不等式约束）、Jallet et al. 2024 的 aligator 库（Pinocchio 生态中的高效全身 OCP 求解器），以及 GPU 加速的采样 MPC（如 DeepMind 的 mujoco_mpc）。这些工具正在使 WB-MPC 从理论走向工程可行。

> **类比**：传统 MPC+WBC 分层就像一个将军（MPC）在沙盘上用简化地形图规划路线，然后让士兵（WBC）在实际地形上执行。Whole-Body MPC 则是将军直接在实际地形的高精度 3D 模型上规划——路线更优，但地图处理的计算量也大得多。当前的工程挑战正是如何让这张"高精度地图"的计算速度跟上实时控制的需求。

### Differentiable WBC

另一个前沿方向是**可微分 WBC**（Differentiable WBC）：将 QP 求解器的前向过程和反向梯度嵌入学习管线中，使得 RL 策略可以直接优化 WBC 的任务权重和参考轨迹。Leziart et al. 2024 (ICRA) 展示了在 iCub 上通过可微分 TSID 端到端学习行走策略的可行性。这个方向将经典 WBC 的结构化优势与学习方法的自适应能力结合，是 F09（学习型力控）与本章内容的自然交汇点。

### 碰撞感知 WBC（Collision-Aware WBC）

2024 年，清华大学在专利中提出了碰撞感知 WBC（CA-WBC），核心思想是将碰撞避免约束从离散的"碰到就停"升级为连续的"安全距离惩罚"。具体做法是在 WBC-QP 的代价函数中添加基于距离场（Signed Distance Field）的势场项：

$$\ell_{collision} = w_{col} \sum_{i} \max(0, d_{safe} - d_i(q))^2$$

其中 $d_i(q)$ 是机器人第 $i$ 个碰撞体到最近障碍物的有符号距离，$d_{safe}$ 是安全距离阈值。当 $d_i > d_{safe}$ 时代价为零（无干预），当 $d_i < d_{safe}$ 时代价二次增长（渐进式排斥）。

> **反事实推理**：如果不用碰撞感知 WBC，而是用传统的碰撞检测+紧急停止会怎样？传统方法检测到碰撞后立即停止所有运动，中断当前任务。在人形上，紧急停止可能破坏平衡（某条腿正在摆动中被停止），导致更严重的后果。CA-WBC 通过在 QP 优化中**连续地权衡碰撞避免与任务执行**，实现"避障的同时继续完成任务"。

### 安全约束 WBC 的统一范式（Safe Task Space Inverse Dynamics）

NSF 资助的一项 2024 研究将 Control Barrier Functions（CBF，参见 F06 第 6 节）与任务空间逆动力学（TSID）统一在同一个 QP 框架中。核心贡献是定义了**任务空间 CBF**：

$$h_i(x) = d_{safe,i} - \|x_i - x_{obs}\|$$

CBF 约束 $\dot{h}_i + \gamma h_i \geq 0$ 被添加为 WBC-QP 的**不等式约束**（而非代价项），从而提供**硬安全保证**——即使所有任务权重都很高，QP 也不会给出违反安全距离的解。

这种方法在人形机器人上的实验表明，CBF 约束相比纯势场方法将最小安全距离的违反率从 ~5% 降低到 <0.1%，同时对任务跟踪精度的影响仅为 2-3%。这一结果再次验证了 F06 中 CBF 方法的"最小干预原则"在 WBC 层面的适用性。

### 2024-2025 WBC 工程实践趋势总结

| 趋势 | 代表工作 | 工程意义 |
|------|---------|---------|
| 从分层到单层 MPC | ProxDDP (Carpentier 2024), aligator 库 | 消除 MPC-WBC 模型不一致，但计算量大 |
| 碰撞感知 WBC | CA-WBC (清华 2024) | 连续碰撞避免，适合人机共融场景 |
| CBF 硬安全约束 | Safe TSID (NSF 2024) | 在 QP 中嵌入硬安全保证，不依赖代价权重 |
| 可微分 WBC | Leziart et al. 2024 | 端到端学习 WBC 权重和参考，降低调参成本 |
| 37 DOF 全身 MPC | Sleiman et al. 2023 | 双臂四足实时全身优化，展示 WB-MPC 工程可行性 |
| GPU 加速采样 MPC | mujoco_mpc (DeepMind 2023) | 500 Hz 采样 MPC 消除 MPC-WBC 频率 gap |

> **本质洞察**：WBC 领域 2024-2025 的核心趋势是**模糊分层边界**——传统的"MPC 规划 → WBC 执行"严格分层正在被"MPC 与 WBC 融合"所取代。这一趋势的驱动力是更强的计算硬件（GPU/NPU）和更高效的算法（ProxDDP、ADMM），使得在预测时域的每个时间步求解全身动力学约束变得可行。但这不意味着分层架构过时了——对于计算资源有限的嵌入式平台（如 ARM Cortex），传统分层架构仍然是最实际的选择。

---

## 本章常见误解汇总

| 编号 | 误解 | 正确理解 |
|:----:|------|---------|
| 1 | "WBC 是腿足机器人专用的" | WBC 的本质是"多任务约束优化"，与基座是否浮动无关。即使固定基座 7-DOF 臂，WBC 也能处理末端跟踪 + 关节避奇异 + 关节限位 + 自碰撞避免的多目标冲突 |
| 2 | "WBC 比阻抗控制更好" | WBC 和阻抗控制不是替代关系，而是包含关系。WBC 框架中的每个任务本身可以是阻抗控制任务。固定基座单任务场景下，直接用阻抗控制更简单、更快、更容易调参 |
| 3 | "QP 权重越大，任务跟踪越好" | 权重过大会导致 QP 条件数恶化（$H$ 的特征值范围过大），求解器精度下降或迭代暴增。OSQP 建议权重比不超过 1000:1 |
| 4 | "HQP 在所有场景都优于加权 QP" | HQP 需要求解 $N$ 个 QP，计算量更大。在嵌入式平台上加权 QP 的单个 QP 可能更实际。有时"所有任务都做到 90%"比"高优先级 100% + 低优先级 0%"更好 |
| 5 | "12D 顶点力表示意味着足底有 12 个独立力自由度" | 12D 力通过 $G$ 矩阵映射到 6D wrench，$\text{rank}(G) = 6$。12 - 6 = 6 个自由度是力分布的冗余——相同的 wrench 可由不同顶点力组合实现 |
| 6 | "q 和 v 的维度一样" | 浮动基座的 $q$ 用四元数表示旋转（7D），而 $v$ 用角速度（6D）。$\dim(q) = 7+n$，$\dim(v) = 6+n$。$M(q)$ 的维度是 $(6+n) \times (6+n)$，与 $v$ 匹配 |
| 7 | "mc_rtc 的导纳增益等于 F05 的导纳参数" | mc_rtc 的 admittance 是简化的比例增益 $v = a \cdot \Delta f$，不是完整的 MDK 二阶导纳模型。mc_rtc 选择简化模型是因为 WBC 的 QP 已经提供了动力学补偿 |
| 8 | "底盘和手臂可以完全解耦控制" | 底盘加速度产生惯性力，通过手臂结构传递到末端。精细力控阶段（如 1N 精度）时底盘突然加速会产生数牛顿惯性力，破坏力控精度 |

---


## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "WBC 只用于足式机器人" | WBC 的核心框架（多优先级 + QP）同样适用于浮动基座+机械臂的移动操作场景 |
| "WBC-QP 和阻抗控制是完全不同的方法" | WBC-QP 和阻抗控制在数学本质上解决同一类问题；区别在于约束复杂度 |
| "TSID 的 Contact6d 就是 6D 力约束" | Contact6d 实际上是 12 维表示（4 个角点各 3D 力），用于精确描述摩擦锥和 ZMP 约束 |
| "mc_rtc 的力控任务和 Pinocchio TSID 是同一个东西" | mc_rtc 和 TSID 有不同的 QP 建模方式和 API 设计，但底层物理约束相同 |

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关联章节 |
|--------|---------|------|---------|
| 第 1 节 为什么需要 WBC | 浮动基座的 3 大困难，3 类需要 WBC 的场景 | ⭐ | F01, F03 |
| 第 2 节 浮动基座动力学 | $M\dot{v} + h = S^T\tau + J_c^T f_c$，接触约束，摩擦锥 | ⭐⭐ | M01, M05 |
| 第 3 节 WBC-QP 标准形式 | 决策变量、等式/不等式约束、代价函数，加权 QP vs HQP | ⭐⭐ | F02, F03 |
| 第 4 节 TSID Contact6d | 12 维力表示，力生成矩阵 G，隐式 ZMP 保证 | ⭐⭐⭐ | 第 3 节 |
| 第 5 节 mc_rtc 力控任务 | 4 种任务（Admittance/Impedance/CoP/Damping），推墙样例 | ⭐⭐ | F05, F03 |
| 第 6 节 统一视角 | WBC 是阻抗控制的多任务+浮动基座推广 | ⭐⭐⭐ | F03 |
| 第 7 节 TSID API 实战 | 完整 Python 代码，固定到浮动基座扩展 | ⭐⭐ | M01 |
| 第 8 节 移动操作接口 | 3 种任务分配策略，ROS2 接口设计 | ⭐⭐⭐ | F08 |

### 术语速查表

| 术语 | 英文 | 一句话定义 |
|------|------|----------|
| WBC | Whole-Body Control | 考虑全身动力学约束的多优先级控制框架 |
| TSID | Task Space Inverse Dynamics | 基于 QP 的任务空间逆动力学求解器 |
| 浮动基座 | Floating Base | 基座不固定于地面的机器人（如四足、人形） |
| 欠驱动 | Underactuated | 驱动器数少于自由度数的系统 |
| Contact6d | Contact6d | TSID 中的 6D 接触约束（12 维力表示） |
| mc_rtc | mc_rtc | 基于 QP 的多接触全身控制框架（CNRS/LIRMM） |
| 移动操作 | Loco-Manipulation | 同时进行运动和操作的任务 |


---

## WBC 的实时性能与调优 ⭐⭐

### QP 求解时间与控制频率的 trade-off

WBC-QP 的核心挑战是**在毫秒级内求解 QP**。典型的 QP 规模和求解时间：

| 场景 | 决策变量数 | 约束数 | 典型求解时间 | 推荐求解器 |
|------|:---------:|:-----:|:----------:|----------|
| 固定基座 7-DOF 臂 | 13 ($\ddot{q}$ + $\lambda$) | 20-30 | 0.05-0.2 ms | qpOASES |
| 四足 12-DOF | 30 ($\ddot{q}$ + 4$\times$3D $\lambda$) | 40-60 | 0.2-0.5 ms | ProxQP |
| 人形 30-DOF | 60+ ($\ddot{q}$ + 2$\times$6D $\lambda$) | 80-120 | 0.5-2 ms | ProxQP/OSQP |
| 人形+双臂操作 | 80+ | 120-160 | 1-5 ms | ProxQP |

**求解器选择指南**：

| 求解器 | 方法 | warm-start | 实时安全 | 典型场景 |
|--------|------|:---------:|:-------:|---------|
| qpOASES | active-set | 是 | 是 | 小规模 QP（$< 50$ 变量） |
| ProxQP | proximal-point | 是 | 是 | 中大规模 QP（50-200 变量） |
| OSQP | ADMM | 是 | 是（有界迭代） | 大规模稀疏 QP |
| ECOS | interior-point | 否 | 否 | 锥规划（SOC 摩擦锥） |

> **本质洞察**：WBC-QP 的求解时间决定了控制频率的上限。如果 QP 求解需要 2 ms，控制频率最多 500 Hz——对于需要 1 kHz 力控的接触任务可能不够。解决方案：(1) 减少约束数（简化摩擦锥），(2) 使用 warm-start（利用上一步的解作为初始值），(3) 使用更快的求解器（ProxQP 比 qpOASES 在大规模问题上快 2-5 倍）。

### WBC 参数调优的系统方法

WBC-QP 的性能高度依赖任务权重的选择。以下是系统化的调参方法：

**Step 1: 确定任务优先级**

| 优先级 | 任务 | 权重量级 | 理由 |
|:------:|------|:-------:|------|
| 1（最高） | 动力学可行性 | 硬约束 | 物理定律不可违反 |
| 2 | 接触力约束 | 硬约束 | 摩擦锥/ZMP 约束 |
| 3 | 质心/末端跟踪 | $w = 100$-$1000$ | 任务核心目标 |
| 4 | 零空间正则化 | $w = 0.1$-$10$ | 舒适构型/操作度 |
| 5 | 力矩正则化 | $w = 0.001$-$0.1$ | 能量最小化 |

**Step 2: 从保守参数开始，逐步提高**

1. 所有任务权重设为 1.0，检查 QP 是否可行
2. 增大最高优先级任务权重（$\times 10$），检查跟踪精度
3. 如果低优先级任务干扰高优先级，增大高优先级权重或加入严格优先级

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：认为 WBC-QP 的权重可以随意选择**
>
> 新手想法："权重不就是相对大小吗？随便设一下就行"
>
> 实际上：WBC-QP 的权重直接决定了 QP 的条件数。如果最大权重和最小权重之比超过 $10^6$，QP 求解器可能因为数值 conditioning 问题给出错误的解——表现为力矩跳变或任务跟踪突然发散。正确做法：权重比控制在 $10^3$-$10^4$ 以内，通过严格优先级（而非极端权重差）来实现任务分层
>
> 正确做法：使用分层 QP（HoQp）或 null-space 投影实现严格优先级，而非用 $10^{10}$ 的权重模拟优先级

### 练习

1. ⭐⭐ **QP 规模分析**：对一个 7-DOF Franka Panda 固定在四足背上的移动操作场景（12 + 7 = 19 个受驱关节），列出 WBC-QP 的所有决策变量和约束。QP 的规模是多少？
2. ⭐⭐ **求解器对比**：在 Pinocchio + ProxQP 中实现一个简单的 WBC-QP（Franka 7-DOF，末端位置任务 + 关节正则化）。测量 100 个不同构型下的求解时间。最慢的构型是什么？
3. ⭐⭐⭐ **权重调优**：在 MuJoCo 中实现四足+臂的 WBC，变化末端位置任务权重从 1 到 10000。观察末端跟踪精度和关节力矩幅值的变化。是否存在"甜点"权重范围？

---

## 累积项目：本章新增模块

```
Mini-ForceControl 项目进度:
  F01: 力控概念框架
  F02: 操作空间数学工具
  F03: 阻抗/混合力控算法
  F04: libfranka 笛卡尔阻抗
  F05: ros2_control 导纳控制
  F06: 变阻抗+无源性+碰撞安全
  F07: WBC-QP 框架 <-- 本章新增
       - TSID Python 绑定的 WBC 示例
       - mc_rtc AdmittanceTask 推墙样例
       - 固定基座 WBC 与阻抗控制关系验证实验
```

---


## 本章与后续章节的关系

| 后续章节 | 关系 | 本章铺垫的关键知识 |
|---------|------|------------------|
| F08 MPC+WBC 联合力控 | 系统集成 | WBC 作为 MPC 的底层执行器 |
| F09 学习型力控 | 前沿融合 | WBC 作为学习策略的底层安全保证 |

---

## 研究实践建议

### 初学者

1. 从本章最核心的算法/概念出发，在 MuJoCo 中实现最简版本
2. 对比不同参数配置下的行为差异，建立直觉
3. 精读本章引用的 1-2 篇核心论文

### 进阶者

1. 在真机上验证仿真中调好的参数，记录仿真与真实的差异
2. 尝试将本章方法与其他章节的方法组合
3. 复现本章引用的前沿工作

## WBC 的常见故障模式与诊断 ⭐⭐

### QP 不可行的诊断

WBC-QP 最常见的运行时问题是**QP 不可行**（infeasible）——没有解能同时满足所有约束。

**不可行的常见原因**：

| 原因 | 表现 | 诊断方法 | 解决方案 |
|------|------|---------|---------|
| 摩擦锥约束过紧 | 需要的切向力超过 $\mu f_n$ | 打印 $f_t / f_n$ 比值 | 增大 $\mu$ 或降低切向力需求 |
| 力矩限制过紧 | 某关节饱和 | 打印各关节力矩利用率 | 降低任务加速度或放松限制 |
| 任务冲突 | 两个任务的期望加速度不相容 | 分别求解单任务 QP | 调整权重或优先级 |
| 接触状态错误 | 实际脱离接触但 QP 仍约束该脚 | 检查接触力的法向分量 | 修正步态检测 |
| 奇异构型 | Jacobian 秩下降 | 检查 $\sigma_{min}(J)$ | 远离奇异或加正则化 |

**QP 不可行时的应急策略**：

1. **放松约束**：逐步放大力矩限制（$\times 1.1, 1.2, ...$）直到 QP 可行，然后截断实际力矩
2. **降低优先级**：暂时去掉最低优先级任务，只保留安全关键任务
3. **冻结输出**：使用上一步的解（零阶保持），等待 QP 恢复可行
4. **安全停机**：如果连续 $> 10$ 步不可行，触发安全模式

```
QP 不可行应急流程:
  QP 不可行?
  |-- 否 -> 正常运行
  +-- 是
      |-- 连续不可行次数 < 3?
      |     |-- 是 -> 使用上一步的解（零阶保持）
      |     +-- 否 -> 放松约束重试
      |           |-- 成功 -> 截断力矩输出
      |           +-- 失败 -> 连续不可行 > 10?
      |                 |-- 否 -> 降低任务优先级
      |                 +-- 是 -> 安全停机
```

### WBC 力矩输出跳变的诊断

力矩输出在相邻控制周期间突然跳变是 WBC 最常见的"可工作但不好用"的问题。

**跳变原因分析**：

| 跳变类型 | 原因 | 频率特征 | 解决方案 |
|---------|------|---------|---------|
| 每步都跳 | 权重不合理导致 QP conditioning 差 | 全频段 | 减小权重比 |
| 周期性跳变 | 接触状态切换 | 步态频率 | 加力平滑约束 |
| 偶发跳变 | 奇异构型附近 | 不规则 | DLS 正则化 |
| 单次跳变 | 任务切换/参数重配置 | 一次性 | 参数平滑过渡 |

---

## 从固定基座到浮动基座——概念迁移指南 ⭐⭐

### 固定基座力控知识在浮动基座中的复用

对于已经掌握 F01-F06 固定基座力控知识的读者，以下对照表帮助你将概念迁移到浮动基座场景：

| 概念 | 固定基座 | 浮动基座 | 主要差异 |
|------|---------|---------|---------|
| **动力学方程** | $M\ddot{q}+h=\tau+J^Tf$ | $M\ddot{q}+h=S^T\tau+J_c^T\lambda$ | 多了选择矩阵 $S$ 和接触力 $\lambda$ |
| **阻抗控制** | 显式公式 | QP 求解 | 约束复杂度从 $O(1)$ 到 $O(n_{contact})$ |
| **零空间** | $N = I - \bar{J}J$ | 多层级累积 $N_k = \prod_{i=1}^{k} (I - \bar{J}_i J_i)$ | 多任务优先级 |
| **无源性** | $\dot{V} \leq f^T v$ | 需考虑各接触点的能量 | 多端口无源性 |
| **力传感器** | 末端 F/T | 关节力矩 + 足端力传感器 | 传感器冗余度更高 |
| **奇异性** | 单一 $J$ 的奇异 | $J$ + 接触 Jacobian 的联合奇异 | 更复杂的奇异分析 |
| **接触约束** | 单一接触点 | 多接触点 + 摩擦锥 + ZMP | 约束数量级增加 |
| **欠驱动** | 无（全驱动） | 浮动基座前 6 维无执行器 | 必须用接触力控制基座 |

**迁移中的常见困惑及解答**：

> **Q**："浮动基座的阻抗控制器长什么样？"
>
> **A**：浮动基座的阻抗控制不再是一个显式公式，而是 QP 的一个**任务项**。在 QP 中，末端位姿跟踪误差的加速度目标被设定为阻抗行为：$\ddot{x}_{des} = \Lambda^{-1}(-K_d \tilde{x} - D_d \dot{\tilde{x}} + f_{ext})$。QP 在满足所有约束（动力学、摩擦锥、力矩限制）的前提下，找到最接近这个期望加速度的解。

> **Q**："无源性在浮动基座中如何保证？"
>
> **A**：浮动基座的无源性分析比固定基座复杂得多——需要考虑多个接触点的能量流。一般通过两种方式保证：(1) 分别保证每个接触点的力控是无源的（局部无源性），(2) 用全局 Lyapunov 函数证明整体系统的无源性（全局方法，较难但更完整）。

> **Q**："我能不能在四足+臂上对手臂直接用 F03 的阻抗控制？"
>
> **A**：可以，但有条件——手臂的 Jacobian 和动力学需要在**浮动基座坐标系**中计算，而非固定世界坐标系。如果基座在移动（四足行走），世界坐标系中的手臂末端位置 = 基座位姿 + 手臂正运动学。忽略基座运动会导致手臂阻抗控制的参考位置"飘"。

### WBC 在不同机器人平台上的部署

| 平台 | DOF | WBC 框架 | 接触约束 | 控制频率 | 核心挑战 |
|------|:---:|---------|:-------:|:-------:|---------|
| Franka Panda（固定） | 7 | 简单 QP / 显式 | 0-1 | 1 kHz | 模型精度 |
| Talos（人形） | 32 | TSID / mc_rtc | 2-4 | 200-500 Hz | QP 求解速度 |
| ANYmal（四足） | 12+1 | WBC / legged_control | 2-4 | 400 Hz | 步态切换 |
| Atlas（人形） | 30 | MIT WBIC | 2-4 | 300 Hz | 高动态运动 |
| Franka on ANYmal | 12+7 | qm_control | 2-4+0-1 | 200-400 Hz | MPC+WBC 耦合 |

---

## 延伸阅读

| 资源 | 类型 | 难度 | 内容 |
|------|------|------|------|
| Sentis & Khatib 2005 "Whole-Body Behaviors" | 论文 | ⭐⭐⭐ | WBC 奠基论文 |
| Del Prete 2015 "TSID" | 代码+文档 | ⭐⭐ | TSID 开源框架 |
| mc_rtc 教程 `jrl.cnrs.fr/mc_rtc/tutorials/` | 教程 | ⭐⭐ | mc_rtc 完整教程 |
| Bouyarmane et al. 2019 "mc_rtc Framework" | 论文 | ⭐⭐⭐ | mc_rtc 架构论文 |
| Romualdi et al. 2022 "BLF" | 代码 | ⭐⭐⭐⭐ | iCub WBC 框架 |
| Pinocchio 官方文档 | 文档 | ⭐⭐ | Pinocchio API 参考 |
| ProxQP (Bambade et al. 2022) | 论文 | ⭐⭐⭐⭐ | INRIA 新 QP 求解器 |
| Escande et al. 2014 "HQP" | 论文 | ⭐⭐⭐⭐ | 层次化 QP 高效算法 |

---

## API 速查表

| API / 函数 | 所属库 | 功能 | 本章相关 |
|-----------|-------|------|---------|
| `tsid::InverseDynamicsFormulationAccForce` | TSID | 加速度-力 QP 建模 | WBC-QP 标准形式 |
| `tsid::TaskSE3Equality` | TSID | 6D 末端位姿跟踪任务 | 操作任务 |
| `tsid::Contact6d` | TSID | 6D 接触约束（12 维力） | 接触力约束 |
| `mc_rtc::mc_control::MCController` | mc_rtc | 多接触控制器基类 | mc_rtc 控制器 |
| `mc_rtc::tasks::force::AdmittanceTask` | mc_rtc | mc_rtc 导纳力控任务 | 末端力控 |
| `pinocchio::forwardDynamics()` | Pinocchio | 正向动力学 $\ddot{q} = M^{-1}(\tau-h)$ | WBC 验证 |
| `proxsuite::proxqp::dense::QP` | ProxQP | 稠密 QP 求解 | WBC-QP 求解 |
| `qpOASES::QProblem` | qpOASES | active-set QP 求解 | 小规模 WBC |
| `pinocchio::computeAllTerms()` | Pinocchio | 一次调用计算所有动力学量 | WBC 数据准备 |

---

## 版本信息速查

| 库/平台 | 推荐版本 | 备注 |
|---------|---------|------|
| Pinocchio | $\geq$ 3.x | WBC 动力学计算核心 |
| TSID | $\geq$ 1.7 | Pinocchio 3.x 兼容 |
| mc_rtc | $\geq$ 2.0 | CNRS/LIRMM 多接触控制 |
| ProxQP | $\geq$ 0.5 | 推荐替代 qpOASES |
| qpOASES | 3.2.1 | 经典 active-set QP |
| OSQP | $\geq$ 0.6 | 大规模稀疏 QP |

---


## WBC-QP 的数值稳定性优化 ⭐⭐⭐


### 常见 QP 求解器的数值精度对比

| 求解器 | 浮点精度 | 约束违反容差 | warm-start 鲁棒性 | 适合的 conditioning |
|--------|:-------:|:----------:|:----------------:|:-----------------:|
| qpOASES | 双精度 | $10^{-12}$ | 高 | $\kappa < 10^{10}$ |
| ProxQP | 双精度 | $10^{-9}$ | 高 | $\kappa < 10^{12}$ |
| OSQP | 双精度 | $10^{-6}$ | 中 | $\kappa < 10^{8}$ |
| ECOS | 双精度 | $10^{-8}$ | 低 | $\kappa < 10^{10}$ |

> **工程建议**：WBC 的 QP 求解器选择应基于实测而非理论——不同求解器在不同问题规模和 conditioning 下表现差异很大。推荐做法：在目标机器人的典型构型上，用所有候选求解器求解 1000 个 QP 实例，对比求解时间的**中位数和最坏情况**（不是均值）。实时系统关心的是最坏情况，不是平均情况。

### WBC 在 ROS2 中的标准架构

```
ros2_control 架构中的 WBC 位置:

  +-------------------+
  | 任务管理器 (BT/FSM)|  10-50 Hz
  +--------+----------+
           | 期望任务
  +--------v----------+
  | MPC 规划器         |  30-200 Hz
  | (OCS2/Crocoddyl)  |
  +--------+----------+
           | 期望加速度/力
  +--------v----------+
  | WBC-QP 控制器      |  200-1000 Hz
  | (TSID/mc_rtc/HoQp)|
  +--------+----------+
           | 关节力矩
  +--------v----------+
  | ros2_control       |
  | hardware_interface |
  +--------+----------+
           | 底层通信
  +--------v----------+
  | 机器人硬件         |
  +-------------------+
```


### QP conditioning 的改善方法

WBC-QP 的数值 conditioning 直接影响求解器的精度和稳定性。以下是常见的 conditioning 问题及解决方案：

| 问题 | 原因 | QP 条件数 | 解决方案 |
|------|------|:---------:|---------|
| 权重差异过大 | $w_{max}/w_{min} > 10^6$ | $> 10^{12}$ | 用严格优先级替代极端权重 |
| 约束矩阵病态 | $J$ 接近奇异 | $> 10^8$ | DLS 正则化 |
| 决策变量量级差异 | $\ddot{q}$ (rad/s$^2$) vs $\lambda$ (N) | $> 10^4$ | 变量缩放（归一化） |
| Hessian 不正定 | 权重矩阵有零特征值 | $\infty$ | 加微小正则化 $\epsilon I$ |

**变量缩放的标准方法**：

将 QP 的决策变量 $z = [\ddot{q}^T, \lambda^T, \tau^T]^T$ 缩放为无量纲变量：

$$\bar{z} = D^{-1} z, \quad D = \text{diag}(\ddot{q}_{max}, ..., \lambda_{max}, ..., \tau_{max}, ...)$$

缩放后的 QP 条件数通常降低 2-3 个数量级。

**ProxQP vs qpOASES 在 WBC 中的对比**：

| 维度 | qpOASES | ProxQP |
|------|---------|--------|
| 方法 | active-set | proximal-point |
| 小规模（$< 50$ 变量） | 最快 | 稍慢 |
| 中规模（50-200 变量） | 可接受 | 更快 |
| 大规模（$> 200$ 变量） | 慢 | 快 2-5 倍 |
| warm-start 效果 | 好 | 好 |
| 数值稳定性 | 良好 | 更好（proximal 正则化） |
| C++ 集成 | 简单 | 简单（header-only） |

推荐策略：小规模 WBC（$< 50$ 变量，如固定基座 7-DOF）用 qpOASES；中大规模（人形/移动操作）用 ProxQP。

### QP 求解失败时的安全降级策略

```
QP 求解状态判断:
  |
  v
  STATUS_OPTIMAL -> 正常使用 QP 解
  |
  STATUS_PRIMAL_INFEASIBLE -> 约束不可行
  |  -> 放松约束重试（力矩限制 x1.2）
  |  |-- 成功 -> 截断力矩到原始限制
  |  +-- 失败 -> 使用上一步解 + 低增益阻尼
  |
  STATUS_MAX_ITER -> 未收敛
  |  -> 使用中间解（sub-optimal but feasible）
  |
  STATUS_NUMERICAL_ERROR -> 数值问题
  |  -> 重置 warm-start
  |  -> 加大正则化 epsilon
  |  -> 重试
```


## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| QP 频繁不可行 | 任务/约束冲突 | 1. 分别求解单任务 QP 2. 逐步加入约束检查可行性 3. 放松约束或降低任务 | §3, §4 |
| 力矩输出跳变 | 接触状态切换/权重不合理 | 1. 加力矩平滑约束 2. 检查接触检测 3. 减小权重比 | §3 |
| 末端位姿跟踪不准 | 权重太低/约束太紧 | 1. 增大位姿任务权重 2. 检查力矩限制是否饱和 3. 检查 Jacobian 计算 | §3, §4 |
| 零空间行为干扰主任务 | 零空间投影不正确 | 1. 验证 $JN \approx 0$ 2. 检查是否用了动力学一致伪逆 3. 降低零空间增益 | §6, F02 |
| TSID Contact6d 力约束方向错误 | 接触法向定义不一致 | 1. 可视化接触法向 2. 检查 world vs local 坐标系 3. 与 Pinocchio 帧定义对齐 | §4 |
| mc_rtc 导纳任务力跟踪差 | 导纳参数不合理/更新频率低 | 1. 检查 $K_d, D_d$ 值 2. 确认 WBC 频率 $\geq$ 200 Hz 3. 检查 F/T 数据延迟 | §5 |
| QP 求解时间超过控制周期 | 问题规模太大 | 1. 减少约束数 2. 使用 warm-start 3. 切换到更快的求解器（ProxQP） | §3 |
| 移动操作时手臂力控不稳定 | 基座运动导致手臂参考系变化 | 1. 检查手臂 Jacobian 的参考系 2. 在基座系中做阻抗 3. 加基座运动前馈补偿 | §8 |
| 全身控制启动时机器人"弹跳" | 初始解不连续 | 1. 启动时用低增益过渡 2. 初始 QP 解用当前状态 warm-start 3. 渐增任务权重 | §3 |

**WBC 调试的黄金法则**：

1. **先单任务后多任务**：先确保每个任务单独运行正常，再组合
2. **先低增益后高增益**：从 50% 的期望增益开始，确认稳定后逐步增加
3. **先仿真后真机**：MuJoCo 中验证所有任务组合后再上真机
4. **实时监控 QP 状态**：打印 QP 的可行性、求解时间、active constraints 数量
5. **版本控制参数**：每次调参后 commit 参数文件，方便回滚


| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| QP 返回 infeasible | 接触约束与任务目标不兼容 | 1. 打印等式约束维度和秩 2. 检查接触帧 3. 降低 Kp | 第 3 节 |
| 关节力矩跳变/抖动 | 力正则化权重过小 | 1. 增大 w_tau 到 1e-3 2. 检查接触力正则化 3. 打印条件数 | 第 3、4 节 |
| 末端跟踪精度差 | 任务权重过低 | 1. 打印各任务残差 2. 逐个关闭低优先级任务 3. 提高权重 | 第 3 节 |
| 接触力分布不对称 | 足底顶点坐标错误 | 1. 可视化顶点 2. 检查坐标系 3. 验证 G 矩阵 | 第 4 节 |
| mc_rtc Admittance 不收敛 | 导纳增益过大 | 1. 减小 admittance 2. 增大 damping 3. 检查力传感器噪声 | 第 5 节 |
| Pinocchio crba() 后矩阵不对称 | 未对称化 | 1. 加 triangularView 赋值 | 第 2 节 |
| 浮动基座积分后四元数非单位 | 使用了 q += v*dt | 1. 改用 pin.integrate() 2. 检查 q 范数 | 第 7 节 |
| 接触切换瞬间力矩跳变 | 接触力变量突然出现/消失 | 1. 在切换前平滑降低力权重 2. 使用 50-100 ms 过渡窗口 3. 新接触从零力权重渐增 | 第 7 节 |
| 多任务时力矩抖动 | 多个任务之间竞争导致活动约束频繁切换 | 1. 降低任务权重比 2. 增大力矩正则化 $w_\tau$ 3. 检查任务之间是否冲突 | 第 3 节 |
| QP 求解时间突然暴增 | 约束活动集频繁切换 | 1. 检查接触状态是否在两个周期间反复切换 2. 增大摩擦锥约束的安全裕度 3. 使用 warm-start 减少迭代 | 第 3 节 |
| 末端姿态振荡（旋转方向） | SE(3) 误差符号约定不匹配 | 1. 确认 current-to-target 或 target-to-current 2. 检查增益符号是否一致 3. 在小角度下验证收敛性 | 第 5 节 |
| 多任务时某个任务完全不生效 | 该任务在其他任务零空间中的投影为零 | 1. 检查任务 Jacobian 的秩 2. 打印零空间维度 3. 降低高优先级任务的 Jacobian 秩 | 第 3 节 |
| ImpedanceTask 姿态反馈不稳定 | SE(3) 误差方向定义与增益符号不匹配 | 1. 确认 current-to-target 还是 target-to-current 约定 2. 检查 $K \cdot e$ 的符号是否产生恢复力矩 | 第 5 节 |

---

## QP 调试系统化方法论

WBC-QP 的调试是初学者面临的最大工程挑战。以下给出系统化的调试流程，帮助快速定位问题。

**快速自检清单**（在报告"QP 不工作"之前先逐条核对）：

| 序号 | 检查项 | 命令/方法 | 通过标准 |
|:----:|--------|---------|---------|
| 1 | $M$ 是否对称 | `assert((M - M.T).norm() < 1e-10)` | 是 |
| 2 | $M$ 是否正定 | `assert(min(eig(M)) > 0)` | 最小特征值 > 0 |
| 3 | $\dim(z)$ 是否正确 | 打印 `z.size()` | $= (6+n) + n + 3k$ |
| 4 | $A_{eq}$ 行数 $\leq$ $z$ 维度 | 打印 `A_eq.rows(), z.size()` | $A_{eq}$ 行数 $<$ $z$ 维度 |
| 5 | $A_{eq}$ 是否满行秩 | `rank(A_eq)` | $= A_{eq}$ 行数 |
| 6 | $H$ 条件数是否合理 | $\kappa(H) = \lambda_{max}/\lambda_{min}$ | $< 10^6$ |
| 7 | 接触帧 ID 是否正确 | `model.getFrameId("foot_name")` | 返回值非 -1 且非 0 |
| 8 | 雅可比帧类型一致 | 检查 `LOCAL_WORLD_ALIGNED` 或 `LOCAL` | 约束和代价函数使用同一帧类型 |
| 9 | crba 后是否对称化 | `M.triangularView<Lower>() = M.T` | 下三角有值 |
| 10 | dJ*v 是否更新 | 调用 `computeJointJacobiansTimeVariation` | drift 项非零（运动中） |

> **最终提醒**：QP 调试的黄金法则——**先确保矩阵组装正确，再调参数**。90% 的"QP 不收敛"问题是矩阵维度、帧类型或索引错误，不是权重或增益不对。建议在项目初期就搭建维度检查和条件数监控的自动化工具。

**Step 1——维度验证**（最常见的错误源）：

在 QP 组装之前，打印并验证所有矩阵的维度。对于 $n$ 个关节、$k$ 个接触点的系统：

| 矩阵 | 期望维度 | 常见错误 |
|------|---------|---------|
| $M$ | $(6+n) \times (6+n)$ | 用了 `model.nq` 而非 `model.nv` |
| $h$ | $(6+n) \times 1$ | 忘记调用 `nonLinearEffects()` |
| $S^T$ | $(6+n) \times n$ | 手动构造时行列搞反 |
| $J_c$（每个接触点） | $6 \times (6+n)$ 或 $3 \times (6+n)$ | 点接触取 3 行但实际取了 6 行 |
| $z$ | $(6+n) + n + 3k$ | 忘记接触力变量 |
| $A_{eq}$ | $(6+n+3k) \times \dim(z)$ | 接触约束维度与接触点数不匹配 |

**Step 2——可行性检查**：

如果 QP 返回 infeasible，按以下顺序排查：

1. 只保留等式约束（动力学+接触），删除所有不等式约束，看是否有解
2. 如果仍 infeasible → 等式约束过定——检查 $A_{eq}$ 的秩是否等于行数
3. 如果有解 → 逐个添加不等式约束，定位导致不可行的那条约束
4. 常见原因：接触状态与实际不一致（如脚已离地但接触约束仍在）

**Step 3——数值稳定性检查**：

打印 $H$ 矩阵的条件数 $\kappa(H) = \lambda_{max}/\lambda_{min}$。当 $\kappa > 10^6$ 时，求解器精度显著下降。常见原因：权重比过大（如 $w_1 = 10000$, $w_2 = 0.001$）。解决方法：缩小权重比，或添加力矩正则化项 $w_\tau \|\tau\|^2$（即使很小，如 $w_\tau = 10^{-6}$，也能显著改善条件数）。

> **类比**：QP 调试就像医生看病——Step 1 是量体温（最基础的检查），Step 2 是做 CT（定位问题），Step 3 是看血液化验（量化问题的严重程度）。跳过 Step 1 直接看 Step 3 是新手最常犯的错误——99% 的 QP 问题都是维度或约束组装错误，不是数值问题。

**Step 4——运行时监控指标**：

在部署阶段，建议持续记录以下诊断量以便事后分析：

| 监控指标 | 正常范围 | 异常含义 |
|---------|---------|---------|
| QP 求解时间 | < 0.5 ms（1 kHz 循环） | 超时说明约束过多或求解器未 warm-start |
| 活跃不等式约束数 | < 总约束数的 30% | 过多说明可行域过小，系统接近物理极限 |
| $\|\tau - \tau_{prev}\|_\infty$ | < 5 Nm/step | 力矩跳变说明 QP 解不连续，需检查约束切换逻辑 |
| 接触力法向分量 $f_{c,z}$ | $[f_{min}, f_{max}]$ 范围内 | 贴近边界说明接触即将破裂或摩擦锥约束将激活 |
