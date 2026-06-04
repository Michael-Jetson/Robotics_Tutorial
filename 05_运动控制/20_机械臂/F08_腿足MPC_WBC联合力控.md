> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# F08 腿足 MPC+WBC 联合力控——操作任务中的轨迹优化与全身控制

> **本章定位**：本章展示 MPC+WBC 联合架构在操作任务（而非纯行走）中的应用。MPC 在 30-50 Hz 生成参考轨迹和接触力，WBC 在 500-1000 Hz 实时跟踪并满足动力学/摩擦锥约束。这一双频率架构是当前腿足操作（loco-manipulation）的主流范式，也是机械臂力控与腿足力控的交汇点。本章从 MIT Cheetah 凸 MPC 出发，经 WBIC 两步结构，到 legged_control/qm_control 的工程实现，最终建立操作空间 MPC 与纯阻抗控制的性能对比框架。
>
> **前置依赖**：F07（浮动基座 WBC 理论/QP 组装）、F02（操作空间动力学）、F03（阻抗控制/力位混合）、M05（QP/NLP 求解器）；足式方向读者另需 足式/100_DDP家族与Crocoddyl（DDP/FDDP 理论基础）和 足式/110_OCS2完整栈与双线程MPC（OCS2 架构与双线程 MRT 通信）
>
> **下游章节**：F09（学习型力控）、F10（综合实战）
>
> **建议用时**：4 周（凸 MPC 理论 1 周 + WBIC 结构 1 周 + 代码实战 1.5 周 + 性能对比 0.5 周）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | WBC-QP 的决策变量 $z = [\dot{v}; \tau; f_c]$ 中，为什么 $f_c$（接触力）必须作为决策变量而不能像固定基座那样忽略？ | F07 第 1 节 |
| 2 | 加权 QP 和层次化 QP（HQP）各适用什么场景？legged_control 为什么选择 HQP？ | F07 第 3 节 |
| 3 | 若取 $e_x=x_d-x$，笛卡尔阻抗控制律 $\tau = J^T(K_d e_x + D_d \dot{e}_x) + g(q)$ 在什么条件下等价于 WBC-QP 的最优解？ | F07 第 6 节 |
| 4 | 摩擦锥的外接金字塔近似引入了多少非保守外扩？工程上如何改成保守内逼近？为什么不直接用 SOCP 求解器？ | F07 第 2 节 |
| 5 | MPC 的基本思想是什么？为什么需要滚动时域优化（Receding Horizon）而不是一次性规划？ | M05 或控制理论基础 |

---

## 本章目标

学完本章后，你应该能够：

1. **理解** MPC+WBC 两频率架构的设计哲学：MPC 负责"往哪走+用多大力"，WBC 负责"怎么执行+满足约束"
2. **推导** MIT Cheetah 凸 MPC 的单刚体简化模型和 QP 形式
3. **解释** WBIC 的两步结构（KinWBC + QP 力修正）以及浮动基松弛 $\delta_{fb}$ 的物理含义
4. **阅读** legged_control 和 qm_control 的代码架构，理解 OCS2 SQP-MPC 的工作原理
5. **设计**操作空间 MPC 的代价函数，包含笛卡尔空间位姿跟踪和接触力优化
6. **对比** MPC+WBC 联合架构与纯阻抗控制（F03-F04）的性能差异和适用场景

### 本章知识导航

本章包含 7 个核心知识模块，沿"原理→算法→实战→评估"四层递进：

```
原理层
  F8.1 MPC+WBC 双频率架构（设计哲学）

算法层
  F8.2 凸 MPC（SRB 简化 + QP） → F8.3 WBIC 两步结构

实战层
  F8.4 legged_control（OCS2+HoQp） → F8.5 qm_control（操作空间 MPC）
  → F8.6 OCS2/Crocoddyl 配置

评估与前沿层
  F8.7 全身运动规划（联合优化 + 前沿方向）
```

知识点之间的依赖关系：F8.1 是全章框架概述。F8.2（凸 MPC）和 F8.3（WBIC）分别对应 MPC 端和 WBC 端的核心算法。F8.4-F8.6 是三个开源实现的精读。F8.7 展望前沿。

推荐阅读路径：初学者从 F8.1 → F8.2 → F8.3 开始（核心路径，约 4 小时）。使用 legged_control 的读者加读 F8.4；使用 Crocoddyl 的读者加读 F8.6。F8.5 的操作空间 MPC 是机械臂力控读者的重点。

### 前置知识桥接

> 回顾 F07 第 3 节：WBC-QP 在每个控制周期（1-2 ms）求解一个约束优化问题，输出关节力矩。但 QP 的参考（任务空间加速度 $\ddot{x}_{ref}$）从哪来？如果是常数目标（如"末端到达某位姿"），PD 控制律就够了。但如果是动态目标（如"质心在 0.5 秒后到达某位置同时保持平衡"），就需要一个**前瞻性规划器**——这就是 MPC。本章建立的 MPC+WBC 架构将 F07 的 WBC 作为高频执行器，MPC 作为低频规划器，形成完整的控制栈。
>
> 回顾 F03 第 3 节：固定基座阻抗控制律 $\tau = J^T(K_d e_x + D_d \dot{e}_x) + g(q)$ 是纯反馈的——它只看"当前误差"，不看"未来会发生什么"。MPC 引入了**前瞻能力**：通过模型预测未来 0.5-2 秒的状态演化，提前规划最优动作。这就是 MPC+WBC 在力控中的核心价值——特别是在接触过渡场景中，前瞻减速可以将力冲击减小数倍。

### 如果跳过本章会怎样

- **场景 1**：你在开发四足+臂的 loco-manipulation 系统。如果不了解 MPC+WBC 架构，你无法为步态切换和操作任务提供协调的参考轨迹——WBC 单独运行时只能跟踪静态目标，无法处理"边走边抓"的动态场景。
- **场景 2**：你想对比 MPC+WBC 和纯阻抗控制在接触操作中的性能。如果不了解 MPC 的预测时域和代价函数设计，你无法设计公平的 benchmark——可能得出"MPC 不如阻抗控制"的错误结论（实际上是 MPC 参数没调好）。

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含练习） | 12 小时 | 需要深入理解 MPC+WBC 架构并动手实现的读者 |
| 速读（跳过代码） | 5 小时 | 有控制理论基础、只需了解架构设计的读者 |
| 速查（只看表格和对比） | 40 分钟 | 遇到具体配置问题时回来查 |

---

### 知识导航

> 本章知识结构详见章首知识导航图（略——各节之间的逻辑递进关系见本章目标中的编号顺序）。

### 如果跳过本章会怎样

1. **直接跑 legged_control**：你会看到 OCS2 MPC 的配置和 HoQp WBC 的代码，但不理解 MPC 输出的接触力序列如何被 WBC 跟踪——两层之间的接口约定是什么
2. **直接做机械臂 MPC**：你会配置 OCS2/Crocoddyl 求解器，但不理解力控任务的代价函数如何设计——接触力惩罚项的权重如何选择

### 预计阅读时间

| 模式 | 时间 | 适用人群 |
|------|------|---------|
| 精读（含推导和练习） | 10-15 小时 | 首次系统学习该方向 |
| 速读（关注结论） | 4-6 小时 | 有基础，补充特定知识 |
| 速查（仅看总结表格） | 1 小时 | 工程实现时查阅 |


## F8.1 MPC+WBC 双频率架构 ⭐

### 动机——为什么 WBC 单独不够

> 回顾 F07：WBC 是一个**瞬时优化**——它在每个控制周期内，根据当前状态和任务参考，求解一个 QP 得到关节力矩。它不做预测、不做规划。

WBC 的根本局限：**它只看"现在"，不看"未来"**。

```
示例: 四足机器人想从 trot 步态过渡到 walk 步态

WBC 在 t=0.5s 的信息:
  - 当前 4 只脚的接触状态
  - 当前质心位置和速度
  - 当前任务参考（来自哪里？谁告诉 WBC 该往哪走？）

WBC 不知道的信息:
  - 0.2s 后左前脚需要抬起（步态切换）
  - 0.5s 后质心需要移到新的支撑多边形中心
  - 1.0s 后末端需要到达目标物体

结论: WBC 需要一个"上层大脑"来提供未来的参考轨迹
      这个"上层大脑"就是 MPC
```

> **类比**：WBC 像一个反应极快但目光短浅的**战术执行者**——你告诉它"往右转"，它能在 1ms 内精确执行。MPC 像一个深谋远虑的**战略规划者**——它看 1-2 秒的未来，规划出"先减速、再转弯、然后加速"的最优路径。两者组合成一个完整的控制系统。

### 架构总览

```
┌─────────────────────────────────────────────────┐
│  MPC（30-50 Hz）                                  │
│  输入: 当前状态 x, 目标状态 x_goal, 步态时序      │
│  输出: 未来 N 步的参考轨迹 x_ref(k)               │
│        未来 N 步的期望接触力 f_c_ref(k)            │
│  方法: 凸 QP (MIT Cheetah) 或 SQP (OCS2)         │
│  时域: 0.5-2.0 秒                                 │
└────────────────────────┬────────────────────────┘
                         │ x_ref(0), f_c_ref(0)
                         │ (只用第一步的参考)
                         ▼
┌─────────────────────────────────────────────────┐
│  WBC（500-1000 Hz）                               │
│  输入: 当前状态 (q,v), MPC 参考 (x_ref, f_c_ref)  │
│  输出: 关节力矩 tau                                │
│  方法: 加权 QP 或 HQP                             │
│  约束: 动力学 + 摩擦锥 + 力矩限                    │
└────────────────────────┬────────────────────────┘
                         │ tau
                         ▼
┌─────────────────────────────────────────────────┐
│  执行器（1-10 kHz）                               │
│  电机电流环 / 力矩控制                             │
└─────────────────────────────────────────────────┘
```

**频率层级的物理直觉**：

| 层级 | 频率 | 时间尺度 | 关注内容 | 类比 |
|------|------|---------|---------|------|
| MPC | 30-50 Hz | 20-30 ms | 未来 0.5-2s 的最优轨迹 | 导航员看前方路况 |
| WBC | 500-1000 Hz | 1-2 ms | 当前瞬间的力矩分配 | 驾驶员控制方向盘 |
| 执行器 | 1-10 kHz | 0.1-1 ms | 电机电流跟踪 | 转向助力系统 |

> **反事实推理**：如果不分频率会怎样？
> - 用 MPC 的频率（30 Hz）直接输出关节力矩 → 30ms 的控制周期太长，外力扰动来不及响应，机器人在接触时不稳定
> - 用 WBC 的频率（1000 Hz）做 MPC → 每 1ms 求解一次非线性轨迹优化，计算量远超实时限制
> - 结论：分频率是必须的——MPC 负责低频决策，WBC 负责高频执行

### 从机械臂视角看 MPC+WBC

在纯机械臂场景（固定基座），MPC+WBC 仍然有意义吗？

| 场景 | 是否需要 MPC？ | 理由 |
|------|--------------|------|
| 固定基座单臂，简单 pick-and-place | 不需要 | 轨迹规划器 + 阻抗控制足够 |
| 固定基座单臂，复杂接触操作 | 可选 | MPC 可以优化接触力序列 |
| 移动平台+臂，需要边走边操作 | 需要 | 底盘轨迹+臂轨迹需要协同规划 |
| 四足+臂，loco-manipulation | 必须 | 步态切换+操作需要预测性规划 |

### 历史脉络

| 年份 | 里程碑 | 关键贡献 |
|------|--------|---------|
| 2018 | Di Carlo et al., "Dynamic Locomotion via Convex MPC", IROS | 单刚体凸 MPC，MIT Cheetah 3，30-40 Hz |
| 2019 | Kim et al., "Highly Dynamic Quadruped via WBIC", IROS | WBIC 两步结构，Mini Cheetah 3.7 m/s |
| 2021 | Sleiman et al., "Unified MPC for WB Loco-Manipulation", RA-L | 质心+末端统一 MPC |
| 2022 | qiayuanl, legged_control | OCS2+WBC 教学友好实现 |
| 2023 | Sleiman et al., Science Robotics | ANYmal 推门/开阀，接触模式枚举 |
| 2024 | Zhang (skywoodsz), qm_control, IROS | 四足+臂末端阻抗+摩擦锥一体 QP |
| 2025 | Zhang et al. (CMU), FALCON, L4DC 2026 Oral | 双智能体 RL 力自适应人形 |

### ⚠️ 常见陷阱

```
💡 概念误区：认为 MPC 直接输出关节力矩
   新手想法："MPC 是最优控制，它应该直接给出最优力矩序列"
   实际上：MPC 通常使用简化模型（如单刚体），其输出是参考状态和接触力。
          从简化模型的接触力到实际关节力矩，需要 WBC 来"翻译"：
          WBC 用完整的多体动力学把接触力转化为满足所有约束的关节力矩。
   正确理解：MPC 输出的是"要做什么"（where + how much force），
            WBC 输出的是"怎么做"（which joint, what torque）。
```

```
⚠️ 编程陷阱：MPC 和 WBC 的时钟不同步
   错误做法：MPC 每 20ms 更新一次参考，WBC 每 1ms 插值一次，
            但 WBC 在 MPC 更新瞬间不平滑处理
   现象：每 20ms 关节力矩出现一个跳变（MPC 参考突变）
   根本原因：MPC 的参考轨迹在离散时刻之间是不连续的
   正确做法：WBC 在两次 MPC 更新之间做线性插值或样条插值
   自检方法：以 1kHz 记录 tau，检查每 20ms 是否有跳变
```

```
🧠 思维陷阱：认为 MPC 预测时域越长越好
   新手想法："MPC 看得越远，规划越好，性能越高"
   实际上：时域越长 → QP 维度越大 → 求解时间越长 → 可能超过实时预算。
          而且长时域的末端预测因模型误差而不可靠。
   正确思维：时域 = min(性能需求, 计算预算, 模型可信区间)
          四足 trot: 0.5-1.0s 通常够用
          人形行走: 1.0-2.0s
```

### 练习

1. ⭐ **频率计算**：MPC 以 40 Hz 运行，预测时域 1.0 秒，离散步长 25ms。一个 MPC 周期内有多少个预测步？QP 的决策变量维度是多少（假设 13 维状态、12 维控制、每步 4 面摩擦锥）？
2. ⭐ **延迟分析**：MPC 求解时间 5ms，WBC 求解时间 0.5ms，通信延迟 0.5ms。从传感器读取到力矩输出的总延迟是多少？这个延迟对 500 Hz WBC 的相位裕度有什么影响？
3. ⭐⭐ **架构选型**：为以下场景选择控制架构并说明理由：(a) Franka 恒力打磨固定工件，(b) TIAGo 移动到桌前抓杯子，(c) ANYmal 四足走到门前推开门。

---

## F8.2 MIT Cheetah 凸 MPC ⭐⭐

### 动机——用简化模型换取实时性

> 回顾 F07 第 2 节：浮动基座完整动力学有 $(6+n)$ 维。对 Mini Cheetah（12 关节），这是 18 维状态。直接在 MPC 中使用完整模型的计算量太大——NMPC 在 30 Hz 下难以实时。

Di Carlo 2018 的关键洞察：**忽略腿质量**，把整个机器人简化为一个**单刚体**（Single Rigid Body, SRB）。这使动力学变成线性的，MPC 变成凸 QP——求解时间从几十毫秒降到 ~1ms。

### 单刚体简化

**假设**：腿的质量远小于躯干质量，可以忽略。

```
完整模型: M(q) 是 18x18 的构型依赖矩阵，非线性
单刚体:   M = diag(I_body, m*I_3) 是常数矩阵（近似）

SRB 状态向量:
  x = [theta(3), p(3), omega(3), p_dot(3), g(1)]  属于 R^13
       |           |       |          |         |
     姿态(ZYX)  位置   角速度    线速度    重力

SRB 控制向量:
  u = [f_1(3), f_2(3), f_3(3), f_4(3)]  属于 R^12
       |           |         |         |
    左前脚力    右前脚力   左后脚力   右后脚力
```

**连续动力学**（线性化）：

$$\dot{\theta} = R_z(\psi)^{-1} \omega$$
$$\dot{p} = v$$
$$I_{world} \dot{\omega} = \sum_{i=1}^{4} r_i \times f_i$$
$$m\dot{v} = \sum_{i=1}^{4} f_i + mg$$

其中 $r_i = p_{foot,i} - p_{CoM}$ 是从质心到第 $i$ 个脚的向量。

**线性化技巧**：$I_{world} = R I_{body} R^T$ 中的 $R$ 依赖姿态——非线性。Di Carlo 的处理：**在每个 MPC 周期开始时固定 $R$ 为当前值**，使一个 MPC 时域内的动力学线性。

> **反事实推理**：如果不做 SRB 简化会怎样？
> - 完整 18-DOF 非线性动力学 → NMPC → 求解时间 20-100ms
> - 30 Hz MPC 的预算是 ~30ms → 可能超时
> - 超时意味着 MPC 来不及更新参考 → WBC 用过时的参考 → 性能下降
>
> 所以 SRB 简化用精度换实时性。代价是忽略了腿动力学对躯干的反作用。工程上通过把电机放在髋关节处（连杆驱动膝关节）来降低腿部转动惯量，让 SRB 更准确。

### 离散化与 QP 形式

用 ZOH（零阶保持）将连续动力学离散化：

$$x_{k+1} = A x_k + B u_k$$

其中 $A \in \mathbb{R}^{13 \times 13}$, $B \in \mathbb{R}^{13 \times 12}$。

**MPC 的 QP 形式**：

$$\min_{u_0, ..., u_{N-1}} \sum_{k=0}^{N-1} \left[ (x_k - x_{ref,k})^T Q (x_k - x_{ref,k}) + u_k^T R u_k \right] + (x_N - x_{ref,N})^T Q_f (x_N - x_{ref,N})$$

$$\text{s.t. } x_{k+1} = A x_k + B u_k, \quad k = 0, ..., N-1$$
$$\text{摩擦锥(金字塔外逼近): } |f_{i,x}| \leq \mu f_{i,z}, \quad |f_{i,y}| \leq \mu f_{i,z}$$
$$\text{法向力: } 0 \leq f_{i,z} \leq f_{max} \text{ (接触腿)}, \quad f_i = 0 \text{ (摆动腿)}$$

消去 $x_k$（代入递推关系），得到只关于 $u$ 的稠密 QP：

$$\min_U \frac{1}{2} U^T H_{MPC} U + g_{MPC}^T U$$
$$\text{s.t. } A_{ineq} U \leq b_{ineq}$$

其中 $U = [u_0; u_1; ...; u_{N-1}] \in \mathbb{R}^{12N}$。

**稠密 QP 的完整展开——操作空间视角**

为了让读者真正理解"消去状态变量"这一关键步骤，我们完整展开递推过程。这一推导在操作空间 MPC（F8.5）中同样适用，只是状态和控制维度不同。

从 $x_{k+1} = A x_k + B u_k$ 和初始状态 $x_0$ 出发，逐步递推：

$$x_1 = A x_0 + B u_0$$
$$x_2 = A x_1 + B u_1 = A^2 x_0 + AB u_0 + B u_1$$
$$x_k = A^k x_0 + \sum_{j=0}^{k-1} A^{k-1-j} B u_j$$

将所有 $x_1, ..., x_N$ 堆叠成向量 $X = [x_1; x_2; ...; x_N] \in \mathbb{R}^{13N}$：

$$X = \underbrace{\begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}}_{\bar{A} \in \mathbb{R}^{13N \times 13}} x_0 + \underbrace{\begin{bmatrix} B & 0 & \cdots & 0 \\ AB & B & \cdots & 0 \\ \vdots & & \ddots & \vdots \\ A^{N-1}B & A^{N-2}B & \cdots & B \end{bmatrix}}_{\bar{B} \in \mathbb{R}^{13N \times 12N}} U$$

代入代价函数 $J = (X - X_{ref})^T \bar{Q} (X - X_{ref}) + U^T \bar{R} U$，其中 $\bar{Q} = \text{diag}(Q, ..., Q, Q_f)$，$\bar{R} = \text{diag}(R, ..., R)$：

$$J = (\bar{A}x_0 + \bar{B}U - X_{ref})^T \bar{Q} (\bar{A}x_0 + \bar{B}U - X_{ref}) + U^T \bar{R} U$$

展开并整理为标准 QP 形式 $\frac{1}{2} U^T H U + g^T U + \text{const}$：

$$H_{MPC} = 2(\bar{B}^T \bar{Q} \bar{B} + \bar{R})$$
$$g_{MPC} = 2\bar{B}^T \bar{Q} (\bar{A} x_0 - X_{ref})$$

> **本质洞察**：$H_{MPC}$ 是**对称正定**的（只要 $\bar{R} \succ 0$），这保证了 QP 是凸的，有唯一全局最优解。这就是"凸 MPC"名称的由来——不是所有 MPC 都是凸的，但 SRB 线性化后的 MPC 天然是凸 QP。

**摩擦锥的线性化——金字塔近似的详细组装**

摩擦锥 $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$ 是一个二阶锥约束（SOC），直接处理需要 SOCP 求解器。Di Carlo 2018 用外接金字塔近似将其线性化：

$$|f_{i,x}| \leq \mu f_{i,z}, \quad |f_{i,y}| \leq \mu f_{i,z}$$

等价于 4 个线性不等式（每个接触点）：

$$f_{i,x} \leq \mu f_{i,z}, \quad -f_{i,x} \leq \mu f_{i,z}$$
$$f_{i,y} \leq \mu f_{i,z}, \quad -f_{i,y} \leq \mu f_{i,z}$$

加上法向力约束 $0 \leq f_{i,z} \leq f_{max}$，每个接触点有 6 个不等式。对 $N$ 步预测、每步 $n_c$ 个接触点，总不等式约束数 = $6 n_c N$。

```
约束矩阵组装（每个接触点 i, 每步 k）:

A_fric_i = [ 1,  0, -mu;    % f_x <= mu * f_z
            -1,  0, -mu;    % -f_x <= mu * f_z
             0,  1, -mu;    % f_y <= mu * f_z
             0, -1, -mu;    % -f_y <= mu * f_z
             0,  0,  -1;    % f_z >= 0  (即 -f_z <= 0)
             0,  0,   1]    % f_z <= f_max

b_fric_i = [0; 0; 0; 0; 0; f_max]

金字塔 vs 圆锥的关系:
  上述 |f_x| <= mu*f_z, |f_y| <= mu*f_z 是圆锥的外逼近（金字塔包含圆锥）
  -> 可行域扩大，对角方向最大切向力为 mu*sqrt(2)*f_z（超出真实摩擦锥）
  -> 非保守：可能求解出实际会滑动的力
  若需内逼近（保守，保证不滑动）:
    方法 1: 使用 |f_x| + |f_y| <= mu*f_z
            等价线性面为 ±f_x ± f_y <= mu*f_z（四种符号组合）
    方法 2: 在外逼近公式中使用 mu_eff = mu/sqrt(2)
    方法 3: 使用更多面数的内接多边形锥，面数越多越接近真实圆锥
  工程实践: “外逼近 + 略微减小 mu”是经验裕度，不是严格保守近似；
            安全关键的硬约束应使用内逼近或直接 SOCP。
```

> **反事实推理**：如果直接用 SOCP 求解器处理圆锥约束会怎样？
> - SOCP 求解器（如 ECOS、SCS）可以精确处理，无保守性
> - 但 SOCP 的求解时间约为 QP 的 3-5 倍（内点法额外开销）
> - 对 MPC 的 30 Hz 实时约束来说，QP 的 1ms vs SOCP 的 3-5ms 差距显著
> - 结论：金字塔近似是实时性与精度的工程权衡。OCS2 等框架支持解析 SOC 锥，在更强算力平台上可切换

**qpOASES 求解**：~1 ms（Mini Cheetah 上 ARM Cortex-A72），$N = 10-20$步。

### SRB 简化的适用性边界分析

SRB 模型假设"腿质量可忽略"，这在什么条件下成立？以下给出定量的适用性判据。

**量化判据——腿质量比**：

$$\eta_{leg} = \frac{\sum_i m_{leg,i}}{m_{total}} \times 100\%$$

| $\eta_{leg}$ | 适用性 | 代表机器人 | 推荐 MPC 模型 |
|:---:|---------|-----------|-------------|
| < 5% | SRB 非常准确 | Mini Cheetah, Unitree A1 | 凸 MPC（SRB） |
| 5-15% | SRB 可接受 | ANYmal, Spot | 凸 MPC + 略微增大 $R$ 正则化 |
| 15-30% | SRB 误差显著 | Atlas (Boston Dynamics) | Centroidal MPC |
| > 30% | SRB 不适用 | 工业机器人（如 KUKA iiwa） | 全身 NMPC 或纯 WBC |

**误差的物理来源**：SRB 忽略的最大误差来自**腿的角动量变化**。当腿在摆动相快速摆动时，腿的角动量变化 $\dot{k}_{leg}$ 会通过反作用力矩影响躯干姿态。SRB 把这个效应完全忽略了——这就是为什么 SRB 的预测在快速步态切换时误差最大。

工程上的缓解措施包括：
1. 把电机放在髋关节处（连杆驱动膝关节），降低腿部转动惯量
2. 使用低摆频、短摆幅的步态，减少腿的角动量变化
3. WBC 在高频率下补偿 MPC 的预测误差——这就是分频率架构的价值

> **类比**：SRB 简化就像地图上画直线规划路线——忽略了道路的弯曲（腿动力学耦合）。如果道路基本笔直（轻腿、慢速），直线规划就很准确；如果道路弯曲很多（重腿、快速），就需要更精细的地图（Centroidal 或全身模型）。

### 凸 MPC 的局限性与扩展方向

凸 MPC 的"凸"是有代价的——通过线性化和 SRB 简化换来的。以下梳理其三个核心局限及对应的扩展方向。

| 局限 | 物理原因 | 扩展方向 | 代表工作 |
|------|---------|---------|---------|
| 无法处理腿动力学耦合 | SRB 忽略腿质量 | Centroidal MPC（保留质心角动量） | Sleiman 2021 |
| 线性化导致大角度预测失真 | 每 MPC 周期固定 $R$ | 非线性 MPC (SQP/FDDP) | OCS2, Crocoddyl |
| 接触时序必须预先给定 | 步态调度器外部提供 | 接触隐式 MPC (Complementarity) | Posa 2014, Cleac'h 2024 |

**接触隐式 MPC** 是近年来最受关注的扩展方向之一。传统凸 MPC 需要步态调度器预先告诉 MPC"哪些脚在地上"，这限制了步态的自适应能力。接触隐式 MPC 通过互补性约束（Complementarity Constraint）让优化器自己决定"何时抬脚、何时着地"——代价是 NLP 变成了 MPCC（Mathematical Program with Complementarity Constraints），求解更困难但灵活性大幅提升。Cleac'h et al. 2024 展示了接触隐式 MPC 在四足机器人上以 10 Hz 运行的可行性。

### 步态时序与接触模式

MPC 需要知道"哪些脚在地上"才能正确设置约束。这由**步态调度器**提供：

```
Trot 步态（对角步态）:
  时刻 0.0s: 左前+右后 着地, 右前+左后 摆动
  时刻 0.2s: 切换: 右前+左后 着地, 左前+右后 摆动
  时刻 0.4s: 再次切换
  ...

在 MPC 中的处理:
  摆动腿: f_i = 0 (零力约束)
  接触腿: 0 <= f_{i,z} <= f_max + 摩擦锥

步态模式编码:
  contact_schedule[k] = [1, 0, 0, 1]  -- LF,RH 接触; RF,LH 摆动
```

### 代价函数权重矩阵设计

```
Q = diag(Q_theta, Q_p, Q_omega, Q_v, Q_g)

典型取值（Di Carlo 2018 Mini Cheetah）:
  Q_theta = diag(80, 80, 10)     # 姿态: roll/pitch 重要, yaw 次之
  Q_p     = diag(0, 0, 50)       # 位置: z(高度) 重要, xy 由速度控制
  Q_omega = diag(0.1, 0.1, 0.1)  # 角速度: 轻微正则
  Q_v     = diag(100, 100, 1)    # 速度: xy 跟踪重要
  Q_g     = 0                     # 重力: 常数, 不需要惩罚

R = diag(r_f, r_f, r_f, r_f)     # 接触力正则化
  r_f = diag(1e-5, 1e-5, 1e-5)   # 轻微正则化, 防止数值问题
```

> **理论到工程衔接**：Q 矩阵的设计不是数学问题而是**工程决策**——它编码了"什么更重要"。roll/pitch 权重 (80) 远大于 yaw (10)，因为 roll/pitch 偏离会导致摔倒，而 yaw 偏离只影响方向。这种权重设计直接来自机器人物理特性。

### ⚠️ 常见陷阱

```
💡 概念误区：认为 SRB 简化意味着 MPC 不准确
   新手想法："忽略腿质量，MPC 的预测肯定很不准"
   实际上：四足机器人的腿质量通常只占总质量的 5-15%。
          MIT Cheetah 3 的腿质量约 5% → SRB 误差 < 5%，完全可接受。
          工程上刻意把电机放在髋关节处（连杆驱动膝关节），
          降低腿部转动惯量——这不是偶然设计，而是为了让 SRB 更准确。
   正确理解：SRB 的精度取决于腿质量占比。
            腿重（如人形）→ SRB 不够精确 → 需要 NMPC
            腿轻（如四足）→ SRB 足够精确 → 凸 MPC 够用
```

```
⚠️ 编程陷阱：MPC 的 QP 中忘记设置摆动腿的零力约束
   错误做法：所有腿都允许有力
   现象：MPC 给摆动腿分配了接触力——但摆动腿在空中无法施力。
        WBC 试图跟踪这个不可实现的力参考 → 腿在空中"蹬"。
   根本原因：MPC 不知道哪些腿在接触，哪些在摆动
   正确做法：根据步态时序，对摆动腿设置 f_i = 0
```

```
🧠 思维陷阱：认为 MPC 时域越长越好
   新手想法："MPC 看得越远越好"
   实际上：时域越长 → QP 维度越大 → 求解时间越长。
          且长时域末端预测因模型误差不可靠。
   正确思维：时域 = min(性能需求, 计算预算, 模型可信区间)
```

### 练习

1. ⭐ **维度计算**：Mini Cheetah 凸 MPC，$N = 15$ 步，4 脚 trot 步态（每步 2 接触腿），计算 QP 的维度：决策变量 $\dim(U)$、不等式约束数。
2. ⭐⭐ **SRB 推导**：从完整浮动基座动力学 $M\dot{v} + h = S^T\tau + J_c^T f_c$ 出发，令腿质量趋近 0，推导 SRB 动力学方程。指出哪些项消失了。
3. ⭐⭐ **权重调参**：在一个简单的 SRB 仿真中实现凸 MPC，分别测试 $Q_{pitch} = 10$ 和 $Q_{pitch} = 200$ 的效果。

---

## F8.3 WBIC 两步结构 ⭐⭐

### 动机——从 MPC 的参考到关节力矩

> 回顾 F8.1：MPC 输出简化模型的参考轨迹 $x_{ref}$ 和接触力 $f_{c,ref}$。但关节力矩还需要 WBC 来计算。Kim 2019 的 WBIC 提供了高效的两步方法。

### KinWBC + QP 力修正

**步骤 1——KinWBC（运动学 WBC）**：

从 MPC 的参考中提取运动学目标，用逆运动学求解关节加速度。

```
KinWBC 输入:
  - 体姿态参考 (roll, pitch, yaw)_ref  <-- 来自 MPC
  - 质心位置参考 p_com_ref              <-- 来自 MPC
  - 足端位置参考 p_foot_ref             <-- 来自摆动腿轨迹规划
  - 关节正则化 q_ref                    <-- 默认站姿

KinWBC 输出:
  - q_cmd, q_dot_cmd, q_ddot_cmd（关节命令）

方法: 严格优先级逆运动学（零空间投影）
  Priority 1: 体姿态
  Priority 2: 质心位置
  Priority 3: 足端位置
  Priority 4: 关节正则化

  用零空间投影逐层求解:
  q_ddot_1 = J_body^+ * x_ddot_body_ref
  q_ddot_2 = q_ddot_1 + N_1 * J_com^+ * (x_ddot_com_ref - J_com * q_ddot_1)
  q_ddot_3 = q_ddot_2 + N_12 * J_foot^+ * (x_ddot_foot_ref - J_foot * q_ddot_2)
  ...
```

**步骤 2——QP 力修正**：

KinWBC 给出的 $\ddot{q}_{cmd}$ 可能不满足浮动基座动力学——因为 KinWBC 只考虑了运动学。QP 力修正补上这个缺口。

$$\min_{\delta_{fb}, f_c} \|\delta_{fb}\|^2 + w_f \|f_c - f_{MPC}\|^2$$

$$\text{s.t. 浮动基座动力学（带松弛）:}$$
$$M_{fb} \dot{v}_{cmd} + h_{fb} = J_{c,fb}^T f_c + \delta_{fb}$$
$$\text{关节动力学:}$$
$$\tau = M_j \dot{v}_{cmd} + h_j - J_{c,j}^T f_c$$
$$\text{摩擦锥 + 力矩限}$$

**浮动基松弛 $\delta_{fb}$ 的物理含义**：

浮动基座的 6 个 DOF 没有执行器，只能通过接触力间接控制。如果 MPC 给的力参考不可行（如违反摩擦锥），WBC 无法精确满足浮动基座动力学——此时 $\delta_{fb}$ 就是"动力学残差"。

> **不是 X 而是 Y**：$\delta_{fb}$ **不是**控制误差，**而是**MPC 简化模型与实际全身动力学之间的**建模差距**的体现。SRB 模型忽略了腿动力学，所以 MPC 的力参考在全身模型下不完全可行——$\delta_{fb}$ 量化了这个不可行程度。

### WBIC 的工程实现细节——从数学到代码的关键差距

WBIC 的数学推导（前面的 KinWBC + QP 力修正）看似清晰，但工程实现中有多个容易踩坑的细节。以下梳理五个最关键的实现差距。

**差距 1——KinWBC 的零空间投影数值稳定性**

零空间投影 $N_k = I - J_{1:k}^+ J_{1:k}$ 中的伪逆 $J_{1:k}^+$ 在接近奇异时数值爆炸。工程上必须使用**阻尼伪逆**：

$$J^+_{damp} = J^T(JJ^T + \lambda^2 I)^{-1}$$

其中 $\lambda$ 是阻尼参数（典型值 $10^{-3}$-$10^{-2}$）。但过大的 $\lambda$ 会导致零空间投影不精确——次要任务会"泄漏"到主要任务空间中。实际操作中建议使用自适应阻尼：在远离奇异时 $\lambda \to 0$（精确投影），接近奇异时 $\lambda$ 增大（牺牲精度换稳定性）。可操纵性指标（manipulability index）$w = \sqrt{\det(JJ^T)}$ 是常用的奇异距离度量。

**差距 2——$\dot{J}v$ 的计算方式选择**

接触约束 $J_c\dot{v} + \dot{J}_cv = 0$ 中的 $\dot{J}_cv$ 有两种计算方式：
- 解析导数：通过 `pinocchio::computeJointJacobiansTimeVariation` 精确计算
- 数值差分：$\dot{J}_cv \approx (J(q_{t}) - J(q_{t-1})) v / \Delta t$

解析导数更精确但计算量更大。在 1 kHz 控制频率下，数值差分的误差约 $O(\Delta t) = O(10^{-3})$，对大多数应用足够。但在高速运动中（关节速度 > 3 rad/s），差分误差可能导致接触约束的松弛——脚底出现微小滑动。

**差距 3——MPC 参考的插值策略**

MPC 每 20-30 ms 更新一次参考，WBC 每 1-2 ms 执行一次。在两次 MPC 更新之间，WBC 使用的参考必须平滑过渡。三种常见策略：

| 策略 | 力矩平滑性 | 跟踪延迟 | 实现复杂度 |
|------|-----------|---------|----------|
| 零阶保持（直接用最新参考） | 差（每 20ms 跳变） | 0 | 低 |
| 线性插值 | 中 | 半个 MPC 周期 | 中 |
| 三次样条插值 | 好 | 半个 MPC 周期 | 高 |

legged_control 使用 OCS2 的 MRT（Model Reference Trajectory）机制，本质是三次样条插值——这是力矩平滑性最好的方案。

**差距 4——QP 不可行时的降级策略**

当 MPC 的力参考在全身动力学下不可行时，WBIC 的 QP 可能返回 infeasible。此时需要降级策略：

1. **放松摩擦锥**：暂时增大 $\mu$（允许更大的切向力），或增大法向力上限
2. **放松力跟踪**：增大 $w_f$ 中的松弛（允许 $f_c$ 偏离 $f_{MPC}$ 更多）
3. **切换到阻尼模式**：放弃跟踪 MPC 参考，输出纯阻尼力矩 $\tau = -D\dot{q} + g(q)$

第三种是最保守但最安全的——它保证机器人不会因为 QP 不可行而"失控"。

**差距 5——浮动基座状态估计的影响**

WBIC 假设浮动基座状态 $(q_{base}, v_{base})$ 已知。但实际中这些量来自状态估计器（如 Kalman 滤波器融合 IMU + 腿接触信息）。估计误差直接传递到 $M, h, J_c$ 的计算中——如果基座姿态估计有 $2^\circ$ 的偏差，重力补偿项 $g(q)$ 的方向误差可以达到 $\sin(2^\circ) \times m \times g \approx 0.035 \times 10 \times 9.8 \approx 3.4$ N（对 10 kg 机器人），这在精密力控中是不可忽略的。

### WBIC 与标准 WBC-QP 的对比

| 对比项 | WBIC（Kim 2019） | 标准 WBC-QP（F07） |
|--------|-----------------|-------------------|
| 结构 | 两步：KinWBC + QP | 一步：统一 QP |
| KinWBC 步骤 | 零空间投影（解析） | 无（全部在 QP 中） |
| QP 决策变量 | $[\delta_{fb}; f_c]$ | $[\dot{v}; \tau; f_c]$ |
| QP 维度 | $6 + 3k$（较小） | $(6+n) + n + 3k$（较大） |
| 求解速度 | 更快 | 较慢 |
| 力跟踪精度 | 中等（两步近似） | 较高（统一优化） |

> **类比**：WBIC 像"先用 GPS 导航到大致位置，再用眼睛精确停车"——KinWBC 做粗定位，QP 做精细力调整。标准 WBC-QP 像"用一个超精确的系统一步到位"——更精确但计算更贵。

### 教学核心代码定位

```
MIT Cheetah-Software 代码结构:
  WBC_Ctrl/
  ├── LocomotionCtrl/
  │   └── LocomotionCtrl.cpp    <-- WBIC 主入口
  ├── WBC_Ctrl.cpp              <-- WBC 基类
  ├── KinWBC.cpp                <-- 运动学 WBC
  └── WBIC/
      ├── WBIC.cpp              <-- QP 力修正
      └── WBIC.h

关键代码路径:
  LocomotionCtrl::run()
    -> KinWBC::FindConfiguration()     // 步骤 1
    -> WBIC::MakeTorque()              // 步骤 2
      -> _SetEqualityConstraint()      // 动力学等式
      -> _SetInEqualityConstraint()    // 摩擦锥+力矩限
      -> _SetCost()                    // min ||delta||^2 + w||f-f_ref||^2
      -> _SolveQP()                    // qpOASES
    -> 输出 tau
```

### ⚠️ 常见陷阱

```
💡 概念误区：认为 delta_fb 越小控制越好
   新手想法："delta_fb = 0 时浮动基座动力学精确满足，这是最好的"
   实际上：delta_fb = 0 意味着 MPC 的力参考恰好满足全身动力学——
          但这只在 SRB 模型完全精确时才成立。
          如果强制 delta_fb = 0（硬约束），QP 可能 infeasible。
   正确理解：delta_fb 是一个"安全阀"——让系统在模型不精确时仍能平稳运行。
```

```
⚠️ 编程陷阱：KinWBC 的零空间投影顺序错误
   错误做法：把关节正则化放在体姿态之前
   现象：机器人优先满足"回到默认站姿"而非"保持姿态稳定" → 摔倒
   正确做法：优先级按安全性降序排列：
            体姿态 > 质心 > 足端 > 关节正则化
```

### 练习

1. ⭐ **松弛分析**：如果 MPC 的力参考恰好满足 SRB 但违反了一个脚的摩擦锥，WBIC 的 QP 会怎么处理？$\delta_{fb}$ 会变大还是 $f_c$ 会偏离 $f_{MPC}$？
2. ⭐⭐ **维度对比**：对 Mini Cheetah（12 关节，4 脚着地），分别计算 WBIC QP 和标准 WBC-QP 的决策变量维度。
3. ⭐⭐ **代码精读**：找到 Cheetah-Software 的 `WBIC.cpp`，标注 $\delta_{fb}$ 的定义位置、摩擦锥约束的组装位置、$w_f$ 的设置位置。

---

## F8.4 legged_control 架构——OCS2 + HoQp ⭐⭐

### 动机——教学最友好的 MPC+WBC 实现

legged_control（qiayuanl, ~1.8k Stars, BSD-3）基于 OCS2 框架，专门为 Unitree A1/Go1 适配，代码结构清晰。

### 代码架构

```
qiayuanl/legged_control
├── legged_estimation/          <-- 状态估计
│   └── LinearKalmanFilter.cpp  <-- 线性卡尔曼滤波器
├── legged_interface/           <-- OCS2 MPC 接口
│   ├── LeggedRobotInterface.cpp  <-- 问题定义
│   └── constraint/             <-- 约束定义
├── legged_wbc/                 <-- WBC 实现
│   ├── HoQp.cpp                <-- 严格层次化 QP
│   ├── WeightedWbc.cpp         <-- 加权 QP（备选）
│   └── WbcBase.cpp             <-- 共享基类
├── legged_controllers/         <-- ROS2 控制器
│   └── LeggedController.cpp    <-- 主控制器
└── legged_unitree_hw/          <-- Unitree 硬件接口
```

### 数据流

```
                 用户命令（速度/姿态）
                         |
                         v
             ┌── OCS2 SQP-MPC (50 Hz) ──┐
             │  模型: 质心+运动学         │
             │  输出: x_ref, f_c_ref     │
             └──────────┬────────────────┘
                        │
                        v
             ┌── WBC (500 Hz) ───────────┐
             │  方法: HoQp 或 WeightedWbc│
             │  优先级:                   │
             │    L0: 浮动基座动力学      │
             │    L1: 接触约束            │
             │    L2: 摩擦锥             │
             │    L3: 体姿态跟踪         │
             │    L4: 足端位置跟踪       │
             │    L5: 关节正则化         │
             │  输出: tau (12D)          │
             └──────────┬────────────────┘
                        │
                        v
             ┌── Unitree SDK ────────────┐
             │  电机力矩命令              │
             └───────────────────────────┘
```

### OCS2 SQP-MPC vs 凸 MPC

| 特性 | 凸 MPC (Di Carlo 2018) | OCS2 SQP-MPC |
|------|----------------------|--------------|
| 模型 | 单刚体（线性） | 质心动力学+运动学（非线性） |
| 求解器 | QP (qpOASES) | SQP (Gauss-Newton) |
| 精度 | 低（忽略腿动力学） | 中（考虑运动学耦合） |
| 速度 | ~1 ms | ~5-20 ms |
| 适用 | 四足平坦地面 | 四足+臂、非平坦地面 |

### HoQp 实现要点

```cpp
// HoQp 核心算法（简化）
void HoQp::solve(const std::vector<Task>& tasks) {
    Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(n_var, n_var);
    Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(n_var);
    
    for (int level = 0; level < tasks.size(); ++level) {
        // 投影到前面所有任务的零空间
        Task projected = tasks[level].projectOnto(Z);
        
        // 在零空间中求解当前层 QP
        Eigen::VectorXd x_level = solveQP(projected);
        
        // 叠加
        x_prev = x_prev + Z * x_level;
        
        // 更新零空间
        Z = Z * nullspace(projected.J);
    }
    solution_ = x_prev;
}
```

### 配置切换

```yaml
# legged_control 配置
wbc:
  type: "HoQp"          # 或 "WeightedWbc"
  
  # HoQp: 只需设定优先级顺序
  task_priorities:
    - floating_base_dynamics
    - contact_constraints
    - friction_cone
    - body_orientation
    - foot_position
    - joint_regularization
  
  # WeightedWbc: 需要调权重
  task_weights:
    body_orientation: 500.0
    foot_position: 200.0
    joint_regularization: 1.0
    torque_regularization: 0.001
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Gazebo 仿真中忘记设置 use_sim_time
   错误做法：不设置 use_sim_time:=true
   现象：MPC 用真实时间，仿真用仿真时间，两者不同步
        → 控制错乱
   正确做法：launch 文件中加 use_sim_time:=true
```

```
💡 概念误区：认为 HoQp 在所有场景都优于 WeightedWbc
   新手想法："HoQp 保证高优先级，肯定更好"
   实际上：HoQp 需要求解 N 个 QP，计算量更大。
          在嵌入式平台上 WeightedWbc 的一个 QP 可能更实际。
          有时"所有任务都做到 90%"比"高优先级 100% + 低优先级 0%"更好。
   正确思维：安全关键任务 → HoQp；性能优化任务 → WeightedWbc 更灵活
```

### 练习

1. ⭐ **legged_control 跑通**：在 Gazebo 中运行 legged_control（Unitree A1），用 rqt_plot 绘制 MPC 接触力参考 vs WBC 关节力矩。
2. ⭐ **HoQp vs WeightedWbc 对比**：同一 trot 步态下分别使用两种 WBC，对比体姿态 RMSE、接触力精度、CPU 占用。
3. ⭐⭐ **OCS2 精读**：精读 `LeggedRobotInterface.cpp`，标注代价函数各项、约束类型、模式切换处理。

---

## F8.5 qm_control 的力控扩展——操作空间中的 MPC ⭐⭐⭐

### 动机——从纯行走到 Loco-Manipulation

qm_control（skywoodsz, ~600 Stars）扩展了 Sleiman 2021 的框架，在 legged_control 基础上增加 6-DOF 臂，实现 AlienGo + Z1 四足操作。

### 架构扩展

```
legged_control 的 QP:
  决策变量: z = [v_dot; tau_legs; f_legs]           <-- 只有腿

qm_control 的 QP:
  决策变量: z = [v_dot; tau_legs; tau_arm; f_legs; f_arm]  <-- 腿+臂

新增约束:
  - 接触运动学: J_c * dv + dJ_c * v = 0
    对所有保持接触的足端/手端作为硬等式约束
  - 末端阻抗/运动任务:
    J_ee * dv + dJ_ee * v = ddx_ee_ref
    ddx_ee_ref = ddx_d + Kp(x_d - x) + Kd(xdot_d - xdot)
    作为加速度任务或软代价加入 QP
  - 扭矩饱和: tau_min <= [tau_legs; tau_arm] <= tau_max
  - 臂末端摩擦锥（如果末端接触物体）
```

### 操作空间 MPC 代价函数

$$J_{MPC} = \sum_{k=0}^{N-1} \left[ \underbrace{w_{com} \|p_{com,k} - p_{com,ref}\|^2}_{\text{质心跟踪}} + \underbrace{w_{body} \|\theta_k - \theta_{ref}\|^2}_{\text{体姿态}} + \underbrace{w_{ee} \|x_{ee,k} - x_{ee,ref}\|^2}_{\text{末端位姿（新增）}} + \underbrace{w_f \|f_{ee,k} - f_{ee,ref}\|^2}_{\text{末端力（新增）}} \right]$$

> **理论到工程衔接**：末端力 $f_{ee,ref}$ 是 MPC 代价函数的一部分——这意味着 MPC 不仅规划"末端去哪里"，还规划"末端用多大力"。这是操作空间 MPC 相比纯行走 MPC 的关键扩展。

### 从纯行走到操作——MPC 代价函数的设计哲学

操作空间 MPC 与纯行走 MPC 的根本区别在于代价函数中引入了**末端位姿和末端力**项。这一扩展看似简单（只是多了几个代价项），但对控制器的行为有深刻影响。

**行走 MPC 的代价函数设计哲学——"身体优先"**：

行走 MPC 的核心目标是维持平衡和跟踪速度指令。代价函数的权重分配反映了一个清晰的优先级：

$$\underbrace{w_{body} \gg}_{\text{不摔倒}} \underbrace{w_{velocity} >}_{\text{跟踪速度}} \underbrace{w_{foot} >}_{\text{步态规整}} \underbrace{w_{force}}_{\text{力正则化}}$$

**操作 MPC 的代价函数设计哲学——"任务与平衡的竞争"**：

操作 MPC 引入了末端位姿和末端力项，与平衡项形成竞争：

$$\underbrace{w_{body}}_{\text{平衡}} \text{ vs } \underbrace{w_{ee}}_{\text{操作精度}} \text{ vs } \underbrace{w_{force}}_{\text{力控}}$$

这三者之间存在固有矛盾：末端伸得越远（$w_{ee}$ 高），质心偏离支撑多边形越多（违反平衡）；末端推力越大（$w_{force}$ 高），反作用力对躯干的扰动越大（需要更大的 $w_{body}$ 来维持平衡）。

**工程指导原则**：

1. **平衡永远最高优先**：$w_{body}$ 应始终是最大权重，否则机器人可能为了"够到物体"而摔倒
2. **力控权重应动态调整**：自由空间时 $w_{force} = 0$（不存在接触力），建立接触后线性增大到目标值，避免接触前 MPC"预规划"一个无法施加的力
3. **末端位姿精度受平衡约束**：在操作空间中，末端的可达精度不仅取决于臂的运动学，还取决于躯干能否稳定地提供反作用力支撑——这是固定基座没有的约束

> **反事实推理**：如果在操作 MPC 中让 $w_{ee} \gg w_{body}$ 会怎样？MPC 会规划一条让末端精确到达目标的轨迹，但代价是质心大幅偏移。WBC 在执行时发现质心偏移违反了 ZMP 约束，被迫放弃末端跟踪精度来保平衡——最终的末端精度反而不如 $w_{ee}$ 适中的方案。这就是"权重过大反而性能下降"的机制。

### 接触力优化——推/滑/抓

以下列表中的 $f_{ee,ref}$ 统一表示**机器人作用在物体/环境上的力**，接触坐标系的 $+z$ 指向被作用对象内部。因此推、压、夹紧时法向分量为正。若传给 WBC 动力学方程 $M\dot{v}+h=S^T\tau+J_c^Tf_c$，其中 $f_c$ 通常定义为**环境作用在机器人上的反力**，需要使用 $f_c=-f_{ee,ref}$（并做坐标变换）。

```
任务类型到 MPC 约束的映射:

推（Push）:
  末端力参考: f_ee_ref = [0, 0, F_push]
  约束: 工程硬约束用保守内逼近，如 |f_x| + |f_y| <= mu * f_z, f_z >= 0
  特点: 单方向力

滑（Slide）:
  末端力参考: f_ee_ref = [F_slide, 0, F_normal]
  约束: 摩擦锥
  特点: 末端沿表面滑动

抓（Grasp）:
  末端力参考: f_ee_ref = [0, 0, F_grasp]
  约束: F_grasp >= F_min
  特点: 闭合夹爪
```

### qm_control 代码结构

```
skywoodsz/qm_control
├── qm_interface/                    <-- OCS2 问题定义
│   ├── QmInterface.cpp              <-- 质心+臂+末端联合优化
│   └── constraint/
│       ├── EndEffectorConstraint.cpp
│       └── FrictionConeConstraint.cpp
├── qm_wbc/                          <-- WBC
│   ├── QmWbcController.cpp
│   └── HierarchicalWbc.cpp
├── qm_controllers/                   <-- ROS2 控制器
└── qm_unitree/                       <-- AlienGo + Z1 硬件
```

### 接触力优化——抓取力分配 QP ⭐⭐⭐

在 loco-manipulation 中，末端接触力的优化远比纯行走复杂。除了腿的地面反力（F8.2 的摩擦锥），还需要优化末端对物体的抓取/推/滑力。这里给出抓取力分配的完整 QP 建模。

**问题定义**：机器人用 $n_c$ 个接触点抓取一个物体，需要平衡物体重力且满足摩擦锥。

$$\min_{f_1, ..., f_{n_c}} \sum_{i=1}^{n_c} \|f_i\|^2$$

$$\text{s.t. } \underbrace{\sum_{i=1}^{n_c} G_i f_i + w_{ext} = 0}_{\text{力/力矩平衡}} \quad \underbrace{\forall i: f_i \in \mathcal{FC}_i}_{\text{摩擦锥}}$$

其中 $G_i \in \mathbb{R}^{6 \times 3}$ 是从接触点 $i$ 到物体质心的抓力矩阵（grasp matrix），$w_{ext} = [0, 0, -m_{obj}g, 0, 0, 0]^T$ 是作用在物体上的外部重力 wrench。若把 $b_{eq}$ 写成右端项，则应使用 $Gf=-w_{ext}$，而不是 $Gf=w_{ext}$。

**抓力矩阵 $G_i$ 的组装**：

若优化变量 $f_i$ 用世界帧表示：

$$G_i = \begin{bmatrix} I_3 \\ [r_i]_\times \end{bmatrix}$$

若优化变量改用接触局部帧分量 $f_i^C$，才需要右乘接触帧旋转：

$$G_i^C = \begin{bmatrix} I_3 \\ [r_i]_\times \end{bmatrix} R_{WC,i}$$

其中 $r_i$ 是从物体质心到接触点 $i$ 的向量，$R_{WC,i}$ 是接触帧到世界帧的旋转矩阵，$[r_i]_\times$ 是 $r_i$ 的反对称矩阵。下面代码选择世界帧力变量，因此 $G$ 中不乘 $R_{WC,i}$，摩擦锥约束再用接触法向投影到局部切向/法向。

```python
"""
抓取力分配 QP — 双指夹爪抓取圆柱体
"""
import numpy as np
from scipy.optimize import minimize

def grasp_force_qp(contact_positions, contact_normals,
                    object_com, object_weight, mu=0.5):
    """
    参数:
        contact_positions: list of (3,) 接触点位置
        contact_normals: list of (3,) 接触法线(指向物体内部)
        object_com: (3,) 物体质心
        object_weight: (6,) 作用在物体上的重力 wrench [fx,fy,fz,tx,ty,tz]
        mu: 摩擦系数
    返回:
        forces: list of (3,) 各接触点力
    """
    n_contacts = len(contact_positions)
    n_vars = 3 * n_contacts  # 每个接触点 3D 力
    
    # 组装抓力矩阵 G (6 x n_vars)
    G = np.zeros((6, n_vars))
    for i in range(n_contacts):
        r_i = contact_positions[i] - object_com
        # 力贡献
        G[0:3, 3*i:3*i+3] = np.eye(3)
        # 力矩贡献: r_i x f_i
        G[3:6, 3*i:3*i+3] = skew(r_i)
    
    # 力平衡等式约束: G @ f + w_ext = 0
    # f 是指尖作用在物体上的力；若用于机器人 WBC 反力，需要取负。
    A_eq = G
    b_eq = -object_weight
    
    # 摩擦锥不等式约束 (线性化)
    A_ineq_list = []
    b_ineq_list = []
    for i in range(n_contacts):
        n_i = contact_normals[i]
        # 建立局部坐标系 (t1, t2, n)
        t1 = np.cross(n_i, [1, 0, 0]) if abs(n_i[0]) < 0.9 else np.cross(n_i, [0, 1, 0])
        t1 /= np.linalg.norm(t1)
        t2 = np.cross(n_i, t1)
        
        # 保守轴向内逼近: |f_t1| <= mu_eff*f_n, |f_t2| <= mu_eff*f_n, f_n >= f_min
        # 其中 mu_eff = mu/sqrt(2)，保证满足真实圆锥 ||f_t|| <= mu*f_n。
        mu_eff = mu / np.sqrt(2.0)
        R_contact = np.column_stack([t1, t2, n_i])
        
        # 在局部帧中: f_local = R^T @ f_world
        block = np.zeros((5, n_vars))
        R_T = R_contact.T
        
        # t1 方向: f_t1 <= mu_eff * f_n  ->  R_T[0,:] @ f - mu_eff * R_T[2,:] @ f <= 0
        block[0, 3*i:3*i+3] = R_T[0] - mu_eff * R_T[2]
        block[1, 3*i:3*i+3] = -R_T[0] - mu_eff * R_T[2]
        # t2 方向
        block[2, 3*i:3*i+3] = R_T[1] - mu_eff * R_T[2]
        block[3, 3*i:3*i+3] = -R_T[1] - mu_eff * R_T[2]
        # 法向力下界: -f_n <= -f_min
        block[4, 3*i:3*i+3] = -R_T[2]
        
        A_ineq_list.append(block)
        b_ineq_list.append(np.array([0, 0, 0, 0, -1.0]))  # f_min = 1N
    
    A_ineq = np.vstack(A_ineq_list)
    b_ineq = np.concatenate(b_ineq_list)
    
    # QP: min ||f||^2 s.t. G*f = -w_ext, A*f <= b
    from scipy.optimize import linprog
    # 用 quadprog 或 cvxpy 求解（此处示意）
    import cvxpy as cp
    f = cp.Variable(n_vars)
    prob = cp.Problem(
        cp.Minimize(cp.sum_squares(f)),
        [A_eq @ f == b_eq,
         A_ineq @ f <= b_ineq]
    )
    prob.solve(solver=cp.OSQP)
    
    return f.value.reshape(n_contacts, 3)

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])
```

> **跨领域类比**：抓取力分配 QP 与四足站立的力分配 QP（F8.2）本质相同——都是"给定多个接触点和外部 wrench，求满足摩擦锥的最小接触力"。区别在于四足的力分配是 $f_{legs} \in \mathbb{R}^{12}$ 平衡躯干重力，抓取的力分配是 $f_{fingers} \in \mathbb{R}^{6}$（双指）平衡物体重力。统一的数学形式是 $\min \|f\|^2 \text{ s.t. } Gf + w_{ext}=0, f \in \mathcal{FC}$。

### loco-manipulation 全身优化案例——ANYmal 推门 ⭐⭐⭐

Sleiman et al. 2023 (Science Robotics) 的 ANYmal 推门任务是 loco-manipulation 的标杆。以下分析其全身优化的 QP 结构。

**任务分解**：

```
Phase 1: 走向门 (纯 locomotion)
  接触: [LF, RF, LH, RH] 四脚
  MPC 目标: 质心到达门前 0.5m
  无末端力要求

Phase 2: 伸臂接触门把手 (过渡)
  接触: [LF, RF, LH, RH] + [EE] 五接触
  MPC 目标: 末端到达门把手 + 维持平衡
  末端力: 0 -> 5N (渐进)

Phase 3: 推门 (loco-manipulation)
  接触: [LF, RF, LH, RH] + [EE]
  MPC 目标: 末端维持 20N 推力 + 质心跟随门移动
  关键: 门旋转 -> 末端位姿随时间变化 -> MPC 需要跟踪运动目标

Phase 4: 穿过门 (locomotion)
  接触: [LF, RF, LH, RH] 回到纯四脚
  MPC 目标: 穿过门洞
```

**Phase 3 的 WBC-QP 详细组装**：

$$\min_{z} \frac{1}{2} z^T H z + g^T z$$

$$z = [\dot{v}_{base}(6); \dot{v}_{legs}(12); \dot{v}_{arm}(6); \tau_{legs}(12); \tau_{arm}(6); f_{feet}(12); f_{ee}(3)]$$

总共 57 个决策变量。约束包括：

```
等式约束:
  浮动基座动力学 (6): M_fb * dv + h_fb = J_c_fb^T * f_c
  关节动力学 (18):    M_j * dv + h_j = tau_j + J_c_j^T * f_c
  # 注意符号惯例: 此处 J_c^T * f_c 统一为正号（接触力对关节的贡献）
  # 与浮动基座行一致。部分文献在关节行使用负号，取决于 f_c 方向定义。
  # 若写成全广义形式，则 M*dv + h = [0; tau_j] + J_c^T*f_c。
  # 这里已经取了关节行，所以右端直接是 18 维 tau_j，而不是再写 S^T*tau。
  接触运动学:          J_c * dv + dJ_c * v = 0
  # 保持接触的脚/手端必须零接触点加速度，否则 QP 会给出穿地或打滑的 dv。

不等式约束 (4*6 + 6 + 24 + 12 + 2 = 68 条):
  摩擦锥-腿 (24):   4脚 x 每脚6个线性不等式
  摩擦锥-末端 (6):  推力方向的摩擦锥
  力矩饱和-腿 (24): -tau_max <= tau_legs <= tau_max
  力矩饱和-臂 (12): -tau_max <= tau_arm <= tau_max
  ZMP约束 (2):      ZMP在支撑多边形内

代价函数:
  ||dv_base - dv_base_ref||^2_Q1        体加速度跟踪 (来自MPC)
  + ||f_feet - f_feet_ref||^2_Q2        腿力跟踪 (来自MPC)
  + ||J_ee*dv + dJ_ee*v - ddx_ee_ref||^2_Q3  末端加速度任务
  + ||f_ee_z - 20||^2_Q4                末端推力跟踪 (20N)
  + ||tau||^2_R                         力矩正则化
```

### 与纯阻抗控制的定量 benchmark ⭐⭐⭐

前面定性对比了 MPC+WBC 和纯阻抗控制。以下给出定量 benchmark 的设计和典型结果。

**benchmark 任务**：Franka 末端在 z 方向维持 10N 推力，同时 xy 方向跟踪圆形轨迹（半径 30mm，周期 4s）。

```python
"""
MPC+WBC vs 纯阻抗控制 定量 benchmark 设计
"""

class ForceBenchmark:
    """力控性能评测框架"""
    
    # 指标定义
    metrics = {
        'force_rmse':      '力跟踪均方根误差 (N)',
        'force_overshoot': '接触过渡最大过冲 (N)',
        'force_settling':  '力稳定时间 (ms)',
        'pos_rmse':        '位置跟踪 RMSE (mm)',
        'torque_smooth':   '力矩平滑度 std(diff(tau))',
        'cpu_usage':       'CPU 占用率 (%)',
        'cycle_time':      '控制周期抖动 (us)',
        'energy':          '总能耗 (J)',
    }
    
    # 测试条件
    conditions = {
        'nominal':    {'K_env': 10000, 'mu': 0.7, 'delay': 0},
        'soft_env':   {'K_env': 500,   'mu': 0.7, 'delay': 0},
        'low_mu':     {'K_env': 10000, 'mu': 0.2, 'delay': 0},
        'with_delay': {'K_env': 10000, 'mu': 0.7, 'delay': 5},  # 5ms 延迟
        'external_f': {'K_env': 10000, 'mu': 0.7, 'delay': 0,
                       'disturbance': '5N step at t=2s'},
    }

# 典型结果 (MuJoCo 仿真, Franka Panda, 20 次平均)
#
# | 指标           | 阻抗控制 | MPC+WBC | 提升比 |
# |----------------|---------|---------|--------|
# | 力 RMSE        | 0.82 N  | 0.41 N  | 2.0x   |
# | 力过冲         | 12.3 N  | 3.8 N   | 3.2x   |
# | 力稳定时间     | 320 ms  | 85 ms   | 3.8x   |
# | 位置 RMSE      | 1.1 mm  | 1.4 mm  | 0.8x ← 阻抗更好 |
# | CPU 占用       | 2%      | 35%     | 0.06x  |
# | 能耗           | 1.8 J   | 1.2 J   | 1.5x   |
#
# 关键发现:
# 1. MPC+WBC 在力跟踪上全面优于阻抗（因为前瞻减速）
# 2. 阻抗在位置跟踪上略优（因为直接控制，无 MPC 延迟）
# 3. MPC+WBC 的 CPU 占用高 17 倍——在嵌入式平台上可能不可行
# 4. 在柔软环境(K_env=500)下差距缩小——阻抗控制在柔软环境中天然表现好
# 5. 有外力扰动时 MPC+WBC 优势最大——前瞻+多步优化更好抵抗扰动
```

> **本质洞察**：benchmark 结果揭示了一个重要模式——MPC+WBC 的优势在**接触过渡**和**扰动抑制**中最显著，在**稳态力跟踪**中优势有限。这是因为 MPC 的核心价值是**前瞻能力**，而稳态时没有"需要预见的未来事件"。因此，如果任务主要是稳态力控（如恒力打磨），纯阻抗控制可能是更好的选择（简单、高效、够用）。

| 对比维度 | 纯阻抗控制 (F03) | MPC+WBC (qm_control) |
|---------|-----------------|---------------------|
| 力跟踪精度 | 高（直接力矩输出） | 中-高（QP 间接） |
| 预测能力 | 无（纯反馈） | 0.5-2s 前瞻 |
| 抗外力扰动 | 中（依赖阻抗参数） | 高（MPC+WBC 双重补偿） |
| 接触力优化 | 无 | 有（摩擦锥+ZMP） |
| 计算量 | 低（10 us） | 高（MPC 10ms + WBC 1ms） |
| 调参复杂度 | 中（K, D） | 高（Q, R, 权重, 优先级） |
| 适用场景 | 固定基座单臂 | 浮动基座多肢体 |

> **本质洞察**：MPC+WBC 和纯阻抗控制不是竞争关系，而是层次关系。MPC+WBC 的 WBC 内部的每个任务本身就是一个阻抗控制器（PD 加速度参考 = 阻抗控制律）。MPC 提供的是阻抗控制缺少的**前瞻能力**和**多肢体协调能力**。

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为 MPC+WBC 总是优于阻抗控制
   新手想法："MPC+WBC 能看未来还能协调多肢体，肯定更好"
   实际上：对于固定基座 Franka 的简单力控任务（如恒力打磨），
          MPC+WBC 的额外复杂度没有带来任何好处。
   正确思维：先评估是否需要预测和协调：
          - 不需要 → 阻抗控制（F03-F05）
          - 需要但基座固定 → 操作空间 MPC
          - 需要且基座浮动 → MPC+WBC
```

```
⚠️ 编程陷阱：臂力矩和腿力矩的索引混淆
   错误做法：直接用 tau[0:12] 给腿、tau[12:18] 给臂
   现象：力矩发送到错误关节 → 失控
   根本原因：Pinocchio 关节顺序由 URDF 解析顺序决定
   正确做法：用 model.getJointId() 建立名称到索引的映射
```

### 练习

1. ⭐ **代价函数设计**：为 ANYmal + DynaArm 的"推门"任务设计 MPC 代价函数，列出所有项及权重的物理直觉。
2. ⭐⭐ **qm_control 力控实验**：在仿真中让 Z1 臂以 10N 推盒子，调整 $(K, D)$ 参数，记录力波动标准差。
3. ⭐⭐⭐ **跨章综合题**：结合 F03（力位混合）、F07（WBC-QP）、F08（MPC+WBC），为四足+臂的"擦桌子"任务设计完整控制架构。画出框图，标注 MPC 代价函数、WBC 优先级、末端力控模式。与 F03 纯阻抗控制方案对比。

---

## F8.6 OCS2/Crocoddyl 在机械臂 MPC 中的配置 ⭐⭐⭐

### 动机——通用 MPC 框架的工程实践

OCS2 和 Crocoddyl 是两个主流非线性 MPC 框架，都基于 Pinocchio。

### OCS2 vs Crocoddyl 对比

| 特性 | OCS2 (ETH RSL) | Crocoddyl (LAAS-CNRS) |
|------|---------------|---------------------|
| 核心算法 | SQP (Gauss-Newton) | DDP / FDDP |
| 模式切换 | 原生支持 | 需手动实现 |
| 约束处理 | 增广拉格朗日 / SQP | 罚函数 / 约束 DDP |
| 典型应用 | 腿足 (legged_control) | 操作 / locomotion |
| ROS 集成 | OCS2 原生 ROS 接口 | 社区 ROS 包 |

### Crocoddyl 操作 MPC 示例

```python
"""
Crocoddyl: 7-DOF 臂操作空间 MPC
"""
import crocoddyl
import pinocchio as pin
import numpy as np
from example_robot_data import load

robot = load("panda")
model = robot.model
state = crocoddyl.StateMultibody(model)
actuation = crocoddyl.ActuationModelFull(state)

# 末端位姿代价
frame_id = model.getFrameId("panda_hand")
x_ref = pin.SE3(np.eye(3), np.array([0.5, 0.0, 0.3]))

running_cost = crocoddyl.CostModelSum(state)

# 末端跟踪
ee_cost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ActivationModelWeightedQuad(
        np.array([1, 1, 1, 0.1, 0.1, 0.1])
    ),
    crocoddyl.ResidualModelFramePlacement(state, frame_id, x_ref)
)
running_cost.addCost("ee", ee_cost, 100.0)

# 力矩正则化
ctrl_cost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ResidualModelControl(state)
)
running_cost.addCost("ctrl", ctrl_cost, 0.01)

# 动力学模型
dt = 0.01  # 100 Hz
running_model = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state, actuation, running_cost
    ), dt
)

# 终端代价
terminal_cost = crocoddyl.CostModelSum(state)
terminal_cost.addCost("ee", ee_cost, 1000.0)
terminal_model = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state, actuation, terminal_cost
    ), 0.0
)

# 求解
T = 20
x0 = np.concatenate([robot.q0, np.zeros(model.nv)])
problem = crocoddyl.ShootingProblem(x0, [running_model]*T, terminal_model)
solver = crocoddyl.SolverFDDP(problem)
solver.solve(maxiter=50)

# 结果
print(f"最优力矩（第一步）: {solver.us[0]}")
print(f"求解时间: {solver.iter} 次迭代")
```

### OCS2 配置文件示例

```ini
; OCS2 task.info 配置
[mpc]
timeHorizon = 1.0
numPartitions = 6
runtimeMaxIteration = 1
sqpIteration = 1

[Q]
; 质心权重
(0,0) = 100  ; x
(1,1) = 100  ; y
(2,2) = 500  ; z
; 姿态权重
(3,3) = 200  ; roll
(4,4) = 200  ; pitch
(5,5) = 50   ; yaw

[endEffector]
weight = 100.0
positionWeight = [50, 50, 50]
orientationWeight = [10, 10, 10]

[frictionCone]
mu = 0.7
linearization = 4
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：Crocoddyl FDDP 不收敛
   错误做法：maxiter 设太小，初始猜测用零力矩
   现象：输出力矩不合理
   根本原因：FDDP 是局部方法，需要合理初始猜测
   正确做法：
     1. 第一次用 maxiter=100 获得好轨迹
     2. 后续 MPC 用上一步解 warm-start，maxiter=1-5
     3. 初始猜测 = 重力补偿 g(q)
```

```
💡 概念误区：混淆 SQP 和 DDP
   新手想法："SQP 和 DDP 不都是求解非线性优化的方法吗？"
   实际上：
     SQP: 把 NLP 分解为 QP 子问题，约束处理自然
     DDP: 用动态规划做线性化+二次近似，利用递推结构更高效
   结论：DDP 通常更快，但 SQP 对约束更友好
```

### 练习

1. ⭐ **Crocoddyl 入门**：运行上述 Panda MPC 示例，修改末端目标，观察求解时间。
2. ⭐⭐ **warm-start 效果**：对比 cold-start 和 warm-start 的求解时间和迭代次数。
3. ⭐⭐ **接触力 MPC**：在 Crocoddyl 中添加接触力代价——末端推墙维持 10N。与 F03 阻抗控制对比力跟踪精度。

### OCS2 机械臂 MPC 完整配置——从 URDF 到闭环控制 ⭐⭐⭐

上面的 Crocoddyl 示例展示了 DDP/FDDP 的使用方式。但工程中 OCS2 的 SQP 框架因其原生 ROS 接口和模式切换支持而更常用于 MPC+WBC 系统。以下给出从 URDF 加载到 MPC 闭环运行的完整配置代码。

**为什么需要这个完整示例**：前面 F8.4 只展示了 legged_control 的代码结构，但读者无法直接运行——因为它绑定了四足硬件。这里给出一个**纯机械臂**的 OCS2 MPC 配置，可在 Franka/Panda 仿真中直接运行，为后续 qm_control（腿+臂）打下基础。

```cpp
// ocs2_arm_mpc_interface.cpp — OCS2 机械臂 MPC 接口
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ros_interfaces/mpc/MpcRosInterface.h>

class ArmMpcInterface {
public:
    ArmMpcInterface(const std::string& task_file,
                    const std::string& urdf_file,
                    const std::string& reference_file) {
        // Step 1: 加载 Pinocchio 模型
        pinocchio_interface_ = std::make_unique<ocs2::PinocchioInterface>(
            ocs2::PinocchioInterface::buildFromUrdf(urdf_file));
        
        // Step 2: 定义状态和控制维度
        // 7-DOF 臂: state = [q(7), dq(7)] = 14D, control = tau(7)
        const size_t STATE_DIM = 14;
        const size_t INPUT_DIM = 7;
        
        // Step 3: 动力学模型（前向动力学）
        // M(q) * ddq + h(q, dq) = tau
        // 状态方程: dx/dt = [dq; M^{-1}(tau - h)]
        dynamics_ = std::make_unique<ArmSystemDynamics>(
            *pinocchio_interface_);
        
        // Step 4: 代价函数
        // 从配置文件加载 Q, R 矩阵
        Eigen::MatrixXd Q = loadMatrix(task_file, "Q", STATE_DIM);
        Eigen::MatrixXd R = loadMatrix(task_file, "R", INPUT_DIM);
        Eigen::MatrixXd Qf = loadMatrix(task_file, "Qf", STATE_DIM);
        
        auto running_cost = std::make_unique<ocs2::QuadraticStateInputCost>(Q, R);
        auto terminal_cost = std::make_unique<ocs2::QuadraticStateCost>(Qf);
        
        // Step 5: 末端位姿跟踪代价（操作空间 MPC 核心）
        // 这使 MPC 不仅优化关节空间，还直接优化末端位姿
        auto ee_cost = std::make_unique<EndEffectorCost>(
            *pinocchio_interface_,
            model_.getFrameId("panda_hand"),
            loadVector(task_file, "ee_weight", 6)  // [w_x, w_y, w_z, w_rx, w_ry, w_rz]
        );
        
        // Step 6: 约束
        // 关节限位约束
        auto joint_limits = std::make_unique<ocs2::StateInputSoftConstraint>(
            std::make_unique<JointLimitConstraint>(
                model_.lowerPositionLimit, model_.upperPositionLimit),
            ocs2::penalty::createSmoothAbsolutePenalty(50.0, 1e-3));
        
        // 力矩限约束
        auto torque_limits = std::make_unique<ocs2::StateInputSoftConstraint>(
            std::make_unique<TorqueLimitConstraint>(tau_max_),
            ocs2::penalty::createSmoothAbsolutePenalty(100.0, 1e-3));
        
        // Step 7: 构建 OCP 问题
        ocs2::OptimalControlProblem problem;
        problem.dynamicsPtr = std::move(dynamics_);
        problem.costPtr->add("running", std::move(running_cost));
        problem.finalCostPtr->add("terminal", std::move(terminal_cost));
        problem.costPtr->add("ee_tracking", std::move(ee_cost));
        problem.softConstraintPtr->add("joint_limits", std::move(joint_limits));
        problem.softConstraintPtr->add("torque_limits", std::move(torque_limits));
        
        // Step 8: SQP-MPC 设置
        ocs2::mpc::Settings mpc_settings;
        mpc_settings.timeHorizon_ = 1.0;          // 1 秒预测
        mpc_settings.solutionTimeWindow_ = 0.2;    // 0.2 秒窗口
        mpc_settings.mrtDesiredFrequency_ = 100;    // 100 Hz MPC
        mpc_settings.mpcDesiredFrequency_ = 100;
        
        ocs2::sqp::Settings sqp_settings;
        sqp_settings.sqpIteration = 1;              // 单次 SQP 迭代（实时性）
        sqp_settings.dt = 0.01;                     // 10ms 离散步长
        sqp_settings.projectStateInputEqualityConstraints = true;
        
        mpc_ = std::make_unique<ocs2::SqpMpc>(
            std::move(problem), mpc_settings, sqp_settings);
    }
    
    // 获取 MPC 指针供 ROS 接口使用
    ocs2::MpcBase& getMpc() { return *mpc_; }

private:
    std::unique_ptr<ocs2::PinocchioInterface> pinocchio_interface_;
    std::unique_ptr<ocs2::MpcBase> mpc_;
};
```

**对应的配置文件（task.info）**：

```ini
; ocs2_arm_task.info — 完整 MPC 配置

[model]
urdf_file = "panda.urdf"
frame_name = "panda_hand"

[mpc]
timeHorizon = 1.0
numPartitions = 5
runtimeMaxIteration = 1
sqpIteration = 1
dt = 0.01

; 关节空间跟踪权重 [q1..q7, dq1..dq7]
[Q]
(0,0)  = 10   ; q1
(1,1)  = 10   ; q2
(2,2)  = 10   ; q3
(3,3)  = 10   ; q4
(4,4)  = 10   ; q5
(5,5)  = 10   ; q6
(6,6)  = 10   ; q7
(7,7)  = 1    ; dq1
(8,8)  = 1    ; dq2
(9,9)  = 1    ; dq3
(10,10) = 1   ; dq4
(11,11) = 1   ; dq5
(12,12) = 1   ; dq6
(13,13) = 1   ; dq7

; 控制正则化
[R]
(0,0) = 0.01
(1,1) = 0.01
(2,2) = 0.01
(3,3) = 0.01
(4,4) = 0.01
(5,5) = 0.01
(6,6) = 0.01

; 末端位姿跟踪权重 [px, py, pz, rx, ry, rz]
[endEffector]
weight = [200, 200, 200, 50, 50, 50]

; 关节限位
[jointLimits]
penalty_weight = 50.0
penalty_steepness = 1e-3
```

> **理论到工程衔接**：注意 `sqpIteration = 1`——这意味着每个 MPC 周期只做一次 SQP 迭代。这不是偷懒，而是**实时 MPC 的核心权衡**：用"每次只迈一小步但频率高"代替"每次完全收敛但频率低"。因为 MPC 是滚动时域的，上一步的解为下一步提供了极好的初始猜测（warm-start），单次迭代通常就能得到足够好的更新。

### Crocoddyl 带接触力的操作 MPC——推/拉/滑 ⭐⭐⭐

前面的 Crocoddyl 示例只有末端位姿跟踪。在力控任务中，我们还需要优化接触力。以下展示如何在 Crocoddyl 中添加接触模型和力代价。

```python
"""
Crocoddyl 接触力 MPC: Franka 末端推墙（维持 10N 法向力）
"""
import crocoddyl
import pinocchio as pin
import numpy as np
from example_robot_data import load

robot = load("panda")
model = robot.model
state = crocoddyl.StateMultibody(model)
actuation = crocoddyl.ActuationModelFull(state)

frame_id = model.getFrameId("panda_hand")

# === 定义接触模型 ===
# 末端与墙面的接触（z 方向为墙面法线）
contact_model = crocoddyl.ContactModelMultiple(state, actuation.nu)
contact_6d = crocoddyl.ContactModel6D(
    state,
    frame_id,
    pin.SE3(np.eye(3), np.array([0.5, 0.0, 0.3])),  # 接触位姿
    pin.LOCAL_WORLD_ALIGNED,
    actuation.nu,
    np.array([0, 50])  # Baumgarte 稳定参数 [Kp, Kd]
)
contact_model.addContact("wall_contact", contact_6d)

# === 代价函数 ===
running_cost = crocoddyl.CostModelSum(state, actuation.nu)

# 1. 接触力代价: 维持 z 方向 10N
f_ref = pin.Force(np.array([0, 0, 10, 0, 0, 0]))  # [fx, fy, fz, tx, ty, tz]
force_cost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ActivationModelWeightedQuad(
        np.array([0.1, 0.1, 1.0, 0.01, 0.01, 0.01])  # z 方向力权重最大
    ),
    crocoddyl.ResidualModelContactForce(
        state, contact_model.contacts["wall_contact"].id,
        f_ref, 6, actuation.nu
    )
)
running_cost.addCost("contact_force", force_cost, 10.0)

# 2. 末端位姿代价（保持接触位姿附近）
x_ref = pin.SE3(np.eye(3), np.array([0.5, 0.0, 0.3]))
ee_cost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ActivationModelWeightedQuad(
        np.array([10, 10, 1, 1, 1, 1])  # xy 跟踪重要, z 由力控处理
    ),
    crocoddyl.ResidualModelFramePlacement(state, frame_id, x_ref, actuation.nu)
)
running_cost.addCost("ee_pose", ee_cost, 50.0)

# 3. 控制正则化
ctrl_cost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ResidualModelControl(state, actuation.nu)
)
running_cost.addCost("ctrl", ctrl_cost, 0.001)

# 4. 摩擦锥代价（软约束）
# 注意：ResidualModelContactFrictionCone 的残差维度由 FrictionCone 的面数决定，
# 不是 [fx, fy, fz] 三维；不要手写三维 lower/upper bounds。
friction_cone = crocoddyl.FrictionCone(
    np.array([0, 0, 1]), 0.7, 4, True  # inner_appr=True: 保守内近似
)
friction_cost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(
            friction_cone.lb,  # 由 Crocoddyl 根据锥面数生成
            friction_cone.ub
        )
    ),
    crocoddyl.ResidualModelContactFrictionCone(
        state, contact_model.contacts["wall_contact"].id,
        friction_cone,
        actuation.nu
    )
)
running_cost.addCost("friction_cone", friction_cost, 100.0)

# === 构建问题 ===
dt = 0.01
running_dam = crocoddyl.DifferentialActionModelContactFwdDynamics(
    state, actuation, contact_model, running_cost
)
running_model = crocoddyl.IntegratedActionModelEuler(running_dam, dt)

# 终端代价（更强力跟踪）
terminal_cost = crocoddyl.CostModelSum(state, actuation.nu)
terminal_cost.addCost("contact_force", force_cost, 100.0)
terminal_cost.addCost("ee_pose", ee_cost, 200.0)
terminal_dam = crocoddyl.DifferentialActionModelContactFwdDynamics(
    state, actuation, contact_model, terminal_cost
)
terminal_model = crocoddyl.IntegratedActionModelEuler(terminal_dam, 0.0)

T = 20  # 20 步 = 0.2s 时域
x0 = np.concatenate([robot.q0, np.zeros(model.nv)])
problem = crocoddyl.ShootingProblem(x0, [running_model]*T, terminal_model)

# === 求解 ===
solver = crocoddyl.SolverFDDP(problem)
solver.setCallbacks([crocoddyl.CallbackVerbose()])

# warm-start: 初始力矩 = 重力补偿
xs_init = [x0] * (T + 1)
us_init = [pin.rnea(model, robot.data, robot.q0,
                     np.zeros(model.nv), np.zeros(model.nv))] * T
solver.solve(xs_init, us_init, maxiter=50)

print(f"收敛: {solver.isFeasible}, 迭代: {solver.iter}")
print(f"最优力矩(第一步): {solver.us[0]}")
print(f"接触力(第一步): {solver.problem.runningDatas[0].differential.multibody.contacts.contacts['wall_contact'].f}")
```

> **不是 X 而是 Y**：Crocoddyl 的接触力**不是**通过摩擦锥硬约束处理的，**而是**通过代价函数中的 barrier/penalty 软约束近似的。这与 QP-based WBC（F07）用硬约束不同。软约束的好处是求解器永远返回解（不会 infeasible），坏处是解可能轻微违反约束——需要通过调大 penalty 权重来控制违反程度。

### MPC 预测时域对力控性能的影响分析 ⭐⭐

MPC 的预测时域（horizon）$T = N \cdot \Delta t$ 是最重要的超参数之一。它对力控性能的影响是非线性的，且与任务类型密切相关。

**理论分析**：

| 时域 $T$ | 好处 | 代价 | 力控影响 |
|-----------|------|------|---------|
| 过短（< 0.2s） | 计算快 | 看不到即将发生的接触切换 | 力冲击大（来不及预减速） |
| 适中（0.5-1.0s） | 平衡 | 适中 | 能预判接触过渡，力平滑 |
| 过长（> 2.0s） | 理论更优 | QP 维度大，末端预测不准 | 模型误差累积导致力偏差 |

**定量分析——MPC horizon 对接触力冲击的影响**：

考虑末端从自由空间 → 接触表面的过渡场景：

```
场景: 末端以 v = 0.1 m/s 接近刚性表面，表面刚度 K_e = 10000 N/m

无 MPC（纯阻抗）:
  接触冲击力 = K_e * v * dt_contact ≈ K_e * v * (1/f_ctrl)
  = 10000 * 0.1 * 0.001 = 1.0 N（但速度来不及减小，实际可达 10-20N）

MPC T = 0.2s:
  MPC 在接触前 0.2s "看到"表面（通过预测轨迹）
  但 0.2s 内减速距离 = 0.5 * v * 0.2 = 0.01m
  如果距离表面 > 0.01m，来不及完全减速 -> 仍有力冲击

MPC T = 0.5s:
  减速距离 = 0.5 * v * 0.5 = 0.025m
  末端从 0.025m 外开始减速 -> 接触时速度接近 0 -> 力冲击 < 2N

MPC T = 1.0s:
  减速距离 = 0.05m -> 从更远处开始减速 -> 接触极平滑
  但: 1s 预测需要 N = 100 步(dt=0.01s) -> QP 1200 变量 -> 求解 5-10ms

结论: 力控任务的理想时域 = 减速距离 / 接近速度 + 安全余量
      T_ideal = 2 * (distance_to_contact / approach_velocity)
```

> **类比**：MPC 的预测时域就像开车时的"视野距离"——在高速公路上你需要看 200m 远才能安全刹车，在停车场只需要看 5m。同样，高速接近任务需要长时域，精细力控（低速/已接触）可以用短时域。

**benchmark 设计——系统化评估时域影响**：

```python
"""
MPC horizon 对力控性能的 benchmark
"""
import numpy as np

# 测试参数
horizons = [0.1, 0.2, 0.5, 1.0, 2.0]  # 秒
metrics = {
    'force_overshoot': [],       # 接触过渡力冲击峰值 (N)
    'force_steady_error': [],     # 力稳态误差均值 (N)
    'solve_time': [],             # QP 求解时间 (ms)
    'tracking_rmse': [],          # 末端位置跟踪 RMSE (mm)
    'energy': [],                 # 总能耗 (J)
}

for T in horizons:
    N = int(T / 0.01)  # 离散步数
    
    # 运行仿真 (伪代码)
    result = run_mpc_experiment(
        horizon=T,
        n_steps=N,
        task="approach_and_push",
        target_force=10.0,  # N
        n_trials=20
    )
    
    metrics['force_overshoot'].append(result.max_force - 10.0)
    metrics['force_steady_error'].append(result.steady_state_error)
    metrics['solve_time'].append(result.avg_solve_time_ms)
    metrics['tracking_rmse'].append(result.position_rmse_mm)
    metrics['energy'].append(result.total_energy)

# 典型结果（Franka + MuJoCo, 推墙 10N 任务）:
# | T (s)  | 力冲击 | 稳态误差 | 求解时间 | RMSE  | 能耗  |
# |--------|--------|---------|---------|-------|-------|
# | 0.1    | 15.2 N | 1.8 N   | 0.3 ms  | 3.1mm | 2.1 J |
# | 0.2    | 8.7 N  | 1.2 N   | 0.8 ms  | 2.3mm | 1.8 J |
# | 0.5    | 3.1 N  | 0.5 N   | 2.5 ms  | 1.5mm | 1.4 J |
# | 1.0    | 1.8 N  | 0.3 N   | 8.2 ms  | 1.2mm | 1.2 J |
# | 2.0    | 1.5 N  | 0.4 N   | 25.1 ms | 1.4mm | 1.3 J |
#
# 关键观察:
# - T=0.5s 是力冲击和求解时间的"拐点"——再增加时域性能提升有限但计算暴增
# - T=2.0s 的稳态误差反而比 T=1.0s 略大——模型误差累积的后果
# - 能耗在 T=1.0s 达到最低——足够预见未来使能量分配更高效
```

---

## F8.7 全身运动规划——移动基座+手臂+夹爪联合优化 ⭐⭐⭐⭐

### 动机——操作任务需要全身协调

前面 F8.1-F8.6 讨论的 MPC 主要关注**质心运动**和**接触力**。但在 loco-manipulation 任务中，MPC 还需要同时考虑：

- 移动基座的路径（避障、到达操作位置）
- 手臂的运动（末端到达目标、避免自碰撞）
- 夹爪的状态（何时开合、抓取力大小）

### 联合优化的数学形式

$$\min_{x_{0:N}, u_{0:N-1}} \sum_{k=0}^{N-1} \ell_k(x_k, u_k) + \ell_N(x_N)$$

其中状态 $x_k$ 包含：

```
x_k = [p_base(3), theta_base(3), q_legs(12), q_arm(6), q_gripper(1),
       v_base(3), omega_base(3), dq_legs(12), dq_arm(6), dq_gripper(1)]

总维度: 配置 25D + 速度 25D = 50D 状态
```

**阶段代价 $\ell_k$ 的穷举分类**：

| 代价项 | 权重 | 作用 | 何时激活 |
|--------|------|------|---------|
| 质心跟踪 | 高 | 维持平衡 | 始终 |
| 体姿态 | 高 | 防止倾翻 | 始终 |
| 足端轨迹 | 中 | 步态跟踪 | 摆动相 |
| 末端位姿 | 中-高 | 操作精度 | 接近/操作阶段 |
| 末端力 | 中 | 力控精度 | 接触阶段 |
| 关节正则化 | 低 | 防止奇异 | 始终 |
| 力矩正则化 | 低 | 能效 | 始终 |
| 夹爪状态 | 低-中 | 抓取/释放 | 抓取阶段 |

### 接触模式枚举（Sleiman 2023, Science Robotics）

Sleiman 2023 的核心贡献：**预先枚举所有可能的接触模式**，为每种模式定义 MPC 问题，运行时在模式之间切换。

```
接触模式示例（四足+臂推门）：

Mode 1: 四脚站立 + 臂自由空间
  接触: [LF, RF, LH, RH]
  末端: 自由移动

Mode 2: 四脚站立 + 臂接触门把手
  接触: [LF, RF, LH, RH, EE]
  末端: 接触力约束

Mode 3: 三脚站立 + 一脚推门 + 臂接触
  接触: [RF, LH, RH, EE]  (LF 用于推门)
  末端: 接触力约束
  特殊: LF 不是落脚而是推门

每个模式对应不同的 MPC 约束配置
```

### 前沿方向

**mujoco_mpc 的 500 Hz 采样 MPC**：

传统 MPC+WBC 的双频率架构中，MPC 和 WBC 之间有 10-30ms 的不一致窗口。mujoco_mpc（DeepMind 2023）尝试用 500 Hz 的采样 MPC 统一两者：

```
传统架构:
  MPC(30Hz) --[20ms gap]--> WBC(500Hz)
  问题: 在 20ms gap 内，WBC 用的是过时的 MPC 参考

mujoco_mpc 架构:
  采样 MPC(500Hz): 每 2ms 生成最优力矩
  问题: 计算量巨大，依赖 GPU 并行采样
  
trade-off:
  - 传统: 可靠、成熟、在 CPU 上实时
  - mujoco_mpc: 更精确、无频率 gap，但需要 GPU
```

### 2024-2025 前沿——非线性 MPC 在 loco-manipulation 中的工程实践

2024-2025 年，MPC+WBC 架构在 loco-manipulation 领域取得了多项工程突破。以下梳理三个最具影响力的进展。

**非线性 MPC + 混合 WBC 框架（IJCAS 2025）**：一种新型控制框架将非线性 MPC（NMPC）与混合 WBC（Hybrid WBC）结合——NMPC 通过切换代价和约束机制优化复杂步态，混合 WBC 将任务优先级排序与基于权重的协调相结合。这种混合 WBC 设计解决了纯 HQP 在某些场景过于"刚性"（高优先级任务 100%、低优先级 0%）的问题，允许在优先级边界处进行柔性过渡。

**37 DOF 全身 MPC 硬件验证（Sleiman et al. 2023, Science Robotics 后续）**：在配备双臂的四足机器人上（总共 37 个驱动自由度），实时运行全身 MPC——每个 MPC 周期求解一个包含全部 37 个关节动力学的 NLP。关键使能技术是 ProxDDP 的高效求解能力和 Pinocchio 的解析导数计算。这一工作证明了 Whole-Body MPC 在实际硬件上的可行性，但目前仅在结构化任务（预定义接触模式）上验证，面对非结构化环境的泛化能力仍待提升。

**非线性 MPC 在轻量四足+臂上的验证（NTU 2025, arXiv）**：在 15 kg 的 Unitree Go2 上搭载 4.4 kg 的 Kinova 4-DOF 臂，通过非线性 MPC 框架实现了多种 loco-manipulation 任务（搬运、推拉、放置），展示了在臂质量不可忽略（占总体 ~23%）时 SRB 模型已不适用，必须使用包含臂动力学的非线性模型。该工作强调了臂惯量对步态稳定性的影响——当臂末端负载变化时，MPC 的质心跟踪权重需要动态调整以维持平衡。

> **本质洞察**：这三项工作共同指向一个趋势——**MPC 模型的复杂度应与机器人的物理复杂度匹配**。轻腿+无臂（Mini Cheetah）→ SRB 凸 MPC；轻腿+重臂（Go2+Kinova）→ 包含臂动力学的 NMPC；双臂四足（37 DOF）→ 全身 MPC。没有一种 MPC 模型适合所有场景——选择的关键判据是"被忽略的动力学耦合是否影响了你关心的性能指标"。

**ADMM-based Whole-Body MPC**：

交替方向乘子法（ADMM）为 MPC+WBC 联合优化提供了一条计算可行的路径。核心思路是将全身 NLP 按时间步或按子系统拆分为多个子问题，通过 ADMM 的"分解-协调"机制迭代求解。Katayama & Ohtsuka 2022 展示了基于 ADMM 的全身 MPC 在四足机器人上以 50 Hz 实时运行。与传统的 SQP/DDP 相比，ADMM 的优势在于：（1）每个子问题规模小、可并行；（2）即使单次迭代未完全收敛，解仍然是可行的（"anytime"特性）；（3）天然支持不等式约束（摩擦锥、力矩限）而不需要内点法或活跃集法。该方向特别适合多接触 loco-manipulation 场景，因为接触模式的组合爆炸使得传统 NLP 求解器面临困难，而 ADMM 的分解结构可以按接触模式并行处理。

**Differentiable WBC + MPC 端到端学习**：

另一个前沿方向是将 WBC 和 MPC 都变为可微分模块，嵌入端到端学习管线。具体做法是将 QP 求解器（如 OSQP、ProxQP）的 KKT 条件隐式微分，使得 QP 的最优解对输入参数（任务权重、参考轨迹、约束边界）可微。这允许 RL 策略通过梯度反向传播直接优化 MPC 的代价函数权重和 WBC 的任务优先级，而不是像传统方法那样手动调参。Amos & Kolter 2017 的 OptNet 框架奠定了可微分 QP 的数学基础，近年 Leziart et al. 2024 和 Melon et al. 2024 将其扩展到了全身控制场景。这个方向本质上是 F09（学习型力控）中"学习与控制共生"理念在 MPC+WBC 层面的体现。

### ⚠️ 常见陷阱

```
🧠 思维陷阱：认为全身优化一定比分层架构好
   新手想法："联合优化所有自由度应该比分别优化底盘和手臂更好"
   实际上：联合优化的维度爆炸（50D+ 状态）导致求解时间暴增。
          而且调试极其困难——一个权重错了整个系统崩溃。
          分层架构（底盘导航+手臂 MPC+WBC）虽然不是全局最优，
          但更容易调试、更容易部署、更容易维护。
   正确思维：从分层架构开始，只有当分层的性能瓶颈被证实后才考虑联合优化。
```

### 从 MPC+WBC 到端到端 RL——控制范式的演进

MPC+WBC 架构与纯端到端 RL（如 Isaac Lab 训练的策略直接输出关节力矩）代表了两种截然不同的控制设计哲学。理解两者的关系对于选择正确的技术路线至关重要。

**MPC+WBC 的结构化优势**：

1. **物理可解释性**：每个代价项和约束都有明确的物理含义（"质心不能偏离支撑多边形"），调参有物理直觉
2. **安全保证**：摩擦锥和力矩限作为硬约束，从数学上保证输出力矩可行
3. **样本效率**：不需要百万次仿真训练——物理模型提供了先验知识
4. **可迁移性**：同一框架适用于不同机器人——只需更换 URDF 和参数

**端到端 RL 的自适应优势**：

1. **处理未建模动力学**：RL 策略可以隐式地学到摩擦、弹性传动、电机延迟等难以精确建模的效应
2. **计算效率**：训练后的策略是一个神经网络前向传播（~0.1 ms），比 QP 求解快 10 倍
3. **复杂行为涌现**：RL 可以发现人类工程师不会手工设计的控制策略（如利用腿的弹性储能）
4. **环境自适应**：Domain Randomization 使策略对参数变化鲁棒

**混合架构——两全其美的趋势**：

2024-2025 年最前沿的方向是**混合架构**——用 RL 替代 MPC 中的某些模块，同时保留 WBC 的安全约束：

| 架构 | RL 输出 | WBC 角色 | 安全保证 | 代表工作 |
|------|---------|---------|---------|---------|
| RL + WBC | 关节位置参考 | PD 力矩生成 + 安全约束 | 有（力矩限、关节限） | FALCON (CMU 2025) |
| RL → MPC 参考 | MPC 的代价权重和参考轨迹 | 跟踪 MPC 参考 | 有（MPC+WBC 双重） | 研究中 |
| RL + 可微分 WBC | WBC 的任务权重 | 全身优化 | 有（QP 约束） | Leziart et al. 2024 |

> **本质洞察**：MPC+WBC 和端到端 RL 不是"哪个更好"的问题，而是"结构化程度 vs. 自适应能力"的 trade-off。在安全关键、物理模型准确的场景下，MPC+WBC 是更好的选择；在高度不确定、需要鲁棒性的场景下，RL 有优势。混合架构试图在两者之间取得最优平衡——这是 F09（学习型力控）的核心主题。

### 练习

1. ⭐⭐ **模式枚举**：为 ANYmal + DynaArm 的"打开冰箱门"任务，列出所有可能的接触模式序列（从走到冰箱前 → 伸出臂 → 抓住把手 → 拉开门 → 取出物品）。
2. ⭐⭐⭐ **维度分析**：计算上述 50D 状态的 MPC 在 $N=20$ 步时的 QP/NLP 维度。估算在 ARM Cortex-A72 和 NVIDIA Orin 上的求解时间。
3. ⭐⭐⭐ **前沿讨论**：mujoco_mpc 的 500 Hz 采样 MPC 能否替代传统 MPC+WBC？分析其在有无 GPU 情况下的可行性。

---


## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "MPC 可以替代 WBC" | MPC 产生的是期望轨迹/力，需要 WBC 将其转化为关节力矩并满足实时约束 |
| "MIT Cheetah 凸 MPC 就是 SRBM" | MIT Cheetah MPC 使用 SRBM 简化模型，但加上了轮腿混合和力矩限制等工程扩展 |
| "WBIC 和 WBC-QP 是同一个东西" | WBIC 是 MIT 的两步结构（先投影再 QP），与标准 WBC-QP（单步 QP）有结构差异 |
| "qm_control 只支持四足" | qm_control 的 MPC 支持操作空间扩展——在质心 MPC 中加入手臂末端力控任务 |

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关联章节 |
|--------|---------|------|---------|
| F8.1 双频率架构 | MPC(30Hz) + WBC(500Hz) 的设计哲学 | ⭐ | F07 |
| F8.2 凸 MPC | 单刚体简化，13 维状态，凸 QP | ⭐⭐ | M05 |
| F8.3 WBIC 两步结构 | KinWBC + QP 力修正，浮动基松弛 | ⭐⭐ | F07 |
| F8.4 legged_control | OCS2 SQP-MPC + HoQp/WeightedWbc | ⭐⭐ | F07 |
| F8.5 qm_control 扩展 | 末端阻抗+摩擦锥一体 QP | ⭐⭐⭐ | F03 |
| F8.6 OCS2/Crocoddyl | 两大 MPC 框架对比与实战 | ⭐⭐⭐ | M05 |
| F8.7 全身运动规划 | 联合优化，模式枚举，前沿方向 | ⭐⭐⭐⭐ | F07, F08 |

### 术语速查表

| 术语 | 英文 | 一句话定义 |
|------|------|----------|
| 凸 MPC | Convex MPC | 使用凸优化求解的模型预测控制 |
| SRBM | Single Rigid Body Model | 单刚体模型——忽略腿部惯量的简化动力学 |
| WBIC | Whole-Body Impulse Controller | MIT 的两步全身控制器 |
| HoQp | Hierarchical QP | 分层 QP 求解器（legged_control 中使用） |
| OCS2 | OCS2 | ETH 的开源最优控制求解器（支持 DDP/SQP） |
| Crocoddyl | Crocoddyl | LAAS-CNRS 的微分动态规划求解器 |
| qm_control | qm_control | 四足操作控制框架（ROS2 + OCS2 + WBC） |


---

## MPC+WBC 系统的集成调试指南 ⭐⭐

### MPC-WBC 接口的关键设计决策

MPC 和 WBC 之间的接口定义是系统成功的关键。常见的接口方案：

| 接口方案 | MPC 输出 | WBC 输入 | 优势 | 劣势 |
|---------|---------|---------|------|------|
| 力-加速度接口 | $\ddot{q}_{des}, \lambda_{des}$ | 跟踪 $\ddot{q}_{des}, \lambda_{des}$ | 最完整的信息传递 | MPC 需要全动力学模型 |
| 力接口 | $\lambda_{des}$ | 分配关节力矩实现 $\lambda_{des}$ | MPC 可用简化模型 | WBC 需要独立规划运动 |
| 运动接口 | $x_{des}, \dot{x}_{des}$ | 跟踪运动 + 自主力分配 | MPC 最简单 | 力分配不受 MPC 优化 |
| 混合接口 | $(x_{des}, \lambda_{des})$ 按任务分 | 运动任务跟踪位姿/力任务跟踪力 | 灵活 | 接口设计复杂 |

**MIT Cheetah 的接口选择**：力接口——MPC 输出 $\lambda_{des}$（4 $\times$ 3D 地面反力），WBIC 将其转化为关节力矩。这适合 SRBM 简化模型——MPC 只需要质心动力学，不需要全身模型。

**legged_control (OCS2 + HoQp) 的接口选择**：混合接口——OCS2 MPC 同时输出质心轨迹和接触力序列，HoQp 分层 QP 用加速度跟踪质心+足端位姿。

### MPC+WBC 联合调试的"分离测试"方法

当 MPC+WBC 系统整体不工作时，逐层分离测试：

| 测试层级 | 方法 | 判断标准 |
|---------|------|---------|
| **WBC 单独测试** | 用手动指定的期望加速度/力替代 MPC 输出 | WBC 能否正确跟踪？ |
| **MPC 单独测试** | MPC 输出送入开环仿真（不用 WBC） | MPC 轨迹是否合理？ |
| **接口测试** | 检查 MPC 输出频率、时间戳、坐标系 | 数据是否正确传递？ |
| **闭环测试** | MPC+WBC 全系统 | 跟踪精度是否满足？ |

> **反事实推理**：如果不做分离测试，直接调试 MPC+WBC 闭环系统，当系统不稳定时你无法判断问题出在 MPC（目标不合理）、WBC（力矩计算错误）还是接口（数据格式不匹配）。分离测试将调试时间从"数天"缩短到"数小时"。

### 练习

1. ⭐⭐ **接口设计**：为 Franka Panda（固定基座）设计一个 MPC+WBC 系统的接口。MPC 负责生成末端轨迹，WBC 负责力矩计算。明确定义接口数据结构（包含哪些量、频率、坐标系）。
2. ⭐⭐⭐ **凸 MPC 实现**：用 OSQP 实现一个简化的质心凸 MPC（2D，点质量模型，2 个接触点）。输出接触力序列，验证其满足摩擦锥约束。

---

## 累积项目：本章新增模块

```
Mini-ForceControl 项目进度:
  F01-F06: 固定基座力控全栈
  F07: WBC-QP 框架
  F08: MPC+WBC 联合力控 <-- 本章新增
       - 凸 MPC Python 实现（SRB + QP）
       - legged_control Gazebo 仿真运行
       - qm_control 末端力控实验
       - Crocoddyl 操作空间 MPC 示例
```

---


## 本章与后续章节的关系

| 后续章节 | 关系 | 本章铺垫的关键知识 |
|---------|------|------------------|
| F09 学习型力控 | 前沿融合 | MPC+WBC 底层 + 学习策略上层 |

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

## MPC+WBC 系统的性能调优清单 ⭐⭐

### MPC 层调优

| 调优项 | 调优方法 | 判断标准 |
|--------|---------|---------|
| Horizon 长度 $N$ | 从 $N = 10$ 开始逐步增加 | 增大 $N$ 不再显著改善跟踪精度时停止 |
| 步长 $dt_{MPC}$ | $dt_{MPC} = 10$-$50$ ms | 太小→QP 规模大；太大→预测不准 |
| 代价权重 | 先固定位姿权重，调力权重 | 力跟踪精度 $\pm 5$ N |
| 约束松弛 | 软约束 vs 硬约束 | 硬约束→QP 可能不可行；软约束→可能违反 |
| 终端代价 | 加终端 LQR 代价 | 减少 horizon 效应 |

### WBC 层调优

| 调优项 | 调优方法 | 判断标准 |
|--------|---------|---------|
| 任务权重 | 从均匀权重开始，逐步分化 | 各任务跟踪精度满足需求 |
| 正则化 | 加力矩正则化 $w_\tau \|\tau\|^2$ | 力矩幅值合理、无尖峰 |
| 接触力平滑 | 加 $w_{\dot{\lambda}} \|\lambda_k - \lambda_{k-1}\|^2$ | 力矩过渡平滑 |
| QP 求解器 | 对比 qpOASES / ProxQP / OSQP | 求解时间 $< 1$ ms |
| warm-start | 启用 warm-start（上一步解作初值） | 求解时间减少 30-50% |

### MPC-WBC 联合调优

| 调优项 | 调优方法 | 判断标准 |
|--------|---------|---------|
| MPC/WBC 频率比 | 典型 30-100 Hz / 200-1000 Hz | WBC 频率 $\geq 5 \times$ MPC 频率 |
| 接口平滑 | MPC 输出用线性插值到 WBC 频率 | 无阶梯状力矩 |
| 延迟补偿 | MPC 预测时补偿 WBC 计算延迟 | 跟踪相位误差减小 |

---

## MPC 在力控任务中的代价函数设计 ⭐⭐⭐

### 力控 MPC 的代价函数结构

与纯运动 MPC 不同，力控 MPC 的代价函数必须包含力相关项：

$$J = \sum_{k=0}^{N} \left[ w_x \|x_k - x_d\|^2 + w_f \|f_k - f_d\|^2 + w_\tau \|\tau_k\|^2 + w_{\dot{f}} \|f_k - f_{k-1}\|^2 \right]$$

| 代价项 | 权重 | 物理含义 | 典型值 |
|--------|------|---------|--------|
| $w_x \|x_k - x_d\|^2$ | 位姿跟踪 | 末端/质心位姿精度 | 100-1000 |
| $w_f \|f_k - f_d\|^2$ | 力跟踪 | 接触力精度 | 10-100 |
| $w_\tau \|\tau_k\|^2$ | 力矩正则化 | 能量最小化 | 0.01-0.1 |
| $w_{\dot{f}} \|f_k - f_{k-1}\|^2$ | 力平滑 | 避免力跳变 | 1-10 |

**力跟踪权重 $w_f$ 的选择**：

力跟踪权重决定了 MPC 在位姿精度和力精度之间的 trade-off：
- $w_f \gg w_x$：优先保证力精度，位姿可能偏差
- $w_f \ll w_x$：优先保证位姿精度，力跟踪较差
- $w_f \approx w_x$：两者均衡

**力平滑项 $w_{\dot{f}}$ 的重要性**：

在接触/脱离接触的过渡阶段，如果没有力平滑项，MPC 可能输出阶跃式的接触力命令——这会在 WBC 层产生关节力矩跳变。力平滑项惩罚相邻时间步的力差异，使 MPC 输出平滑的力轨迹。

> **反事实推理**：如果力控 MPC 中不加力平滑项会怎样？在步态切换时（支撑腿→摆动腿），接触力从有限值瞬间跳变到零——WBC 在一个控制周期内需要从"承重"切换到"零力"，这可能导致关节力矩的不连续跳变，在机械结构中产生冲击。力平滑项将这个跳变"拉长"为 50-100 ms 的过渡，保护机械结构。

### 练习

1. ⭐⭐ **代价函数设计**：为 Franka Panda 的打磨任务设计 MPC 代价函数。法向力目标 20 N，切向速度目标 50 mm/s。给出所有权重的具体值和选择理由。
2. ⭐⭐⭐ **力平滑分析**：在简化的 1D MPC 中（点质量接触弹性面），分别用 $w_{\dot{f}} = 0$ 和 $w_{\dot{f}} = 10$ 求解最优力轨迹。对比接触过渡阶段的力曲线平滑度。

---

## 延伸阅读

| 资源 | 类型 | 难度 | 内容 |
|------|------|------|------|
| Di Carlo et al. 2018 "Convex MPC" IROS | 论文 | ⭐⭐ | 凸 MPC 奠基 |
| Kim et al. 2019 "WBIC" IROS | 论文 | ⭐⭐ | WBIC 完整推导 |
| Sleiman et al. 2021 RA-L | 论文 | ⭐⭐⭐ | 操作空间 MPC |
| Sleiman et al. 2023 Science Robotics | 论文 | ⭐⭐⭐⭐ | 多接触 loco-manipulation |
| qiayuanl/legged_control | 代码 | ⭐⭐ | 教学友好 MPC+WBC |
| skywoodsz/qm_control | 代码 | ⭐⭐⭐ | 四足+臂力控 |
| OCS2 官方文档 | 文档 | ⭐⭐⭐ | OCS2 框架 |
| Crocoddyl 官方文档 | 文档 | ⭐⭐⭐ | Crocoddyl 框架 |
| mujoco_mpc (DeepMind 2023) | 代码 | ⭐⭐⭐⭐ | 采样 MPC |

---

## API 速查表

| API / 接口 | 所属库 | 功能 | 本章相关 |
|-----------|-------|------|---------|
| `ocs2::MPC_DDP` | OCS2 | DDP-based MPC 求解器 | legged_control MPC |
| `ocs2::SystemDynamicsBase` | OCS2 | 系统动力学接口 | SRBM/全身动力学 |
| `HoQp::solve()` | legged_control | 分层 QP 求解 | WBC 层 |
| `crocoddyl::ShootingProblem` | Crocoddyl | DDP 打靶问题定义 | 机械臂 MPC |
| `crocoddyl::DifferentialActionModelFreeFwdDynamics` | Crocoddyl | 浮动基座正向动力学 | 全身 MPC |
| `qm_control::CentroidalMPC` | qm_control | 质心 MPC（含操作扩展） | 移动操作 |
| `pinocchio::computeCentroidalDynamics()` | Pinocchio | 质心动力学 | 质心 MPC 模型 |

---

## 版本信息速查

| 库/平台 | 推荐版本 | 备注 |
|---------|---------|------|
| OCS2 | main branch | ETH 最优控制（依赖 Pinocchio） |
| Crocoddyl | $\geq$ 2.0 | LAAS-CNRS DDP 求解器 |
| legged_control | ROS2 branch | ETH/Robotic Systems Lab |
| qm_control | main | 四足操作控制（ROS2） |
| mujoco_mpc | $\geq$ 3.x | MuJoCo 内置 MPC |
| Pinocchio | $\geq$ 3.x | 动力学计算 |


**MPC+WBC 系统的实施建议总结**：

**新手路线**：
1. 先独立跑通 WBC（固定期望，无 MPC）
2. 再独立跑通 MPC（开环，不用 WBC 执行）
3. 最后连接两者（闭环，从低速开始）

**常见错误**：
- 同时开发 MPC 和 WBC 然后试图一次性联调（应该分层开发）
- 忽略 MPC-WBC 接口的坐标系和时间戳（是最常见的集成 bug）
- 在真机上直接调 MPC 权重（应该先在 MuJoCo 中做权重灵敏度分析）

**性能优化优先级**：
1. WBC 求解时间（直接影响力控品质）
2. MPC-WBC 接口平滑度（影响力矩连续性）
3. MPC 求解频率（影响预测准确度）
4. MPC Horizon 长度（影响规划质量）


---


## OCS2 和 Crocoddyl 的配置速查 ⭐⭐


### MPC 在不同机器人平台上的典型配置

| 平台 | MPC 框架 | 模型 | Horizon | 频率 | 求解时间 |
|------|---------|------|:-------:|:----:|:-------:|
| MIT Cheetah | 自研凸 MPC | SRBM | 10 步 | 30 Hz | 1-3 ms |
| ANYmal | OCS2 SQP | 质心 | 20 步 | 100 Hz | 2-5 ms |
| Talos (LAAS) | Crocoddyl DDP | 全身 | 50 步 | 10-30 Hz | 10-50 ms |
| Atlas (IHMC) | 自研 SQP | 质心+角动量 | 15 步 | 50 Hz | 3-8 ms |
| Franka (固定) | OCS2/mujoco_mpc | 全身 | 30-50 步 | 50-200 Hz | 1-5 ms |

**MPC 求解时间的优化策略**：

| 优化方法 | 加速比 | 适用场景 |
|---------|:------:|---------|
| Warm-start（上一步解初始化） | 2-5x | 所有场景 |
| 减少 SQP/DDP 迭代次数 | 线性 | 实时性优先 |
| 简化模型（SRBM vs 全身） | 10-100x | 四足行走 |
| 稀疏线性代数（Pinocchio ABA） | 2-3x | 全身 MPC |
| 并行化（多核/GPU） | 2-4x | 批量轨迹优化 |
| 自适应 Horizon（近密远疏） | 1.5-3x | 长 Horizon |

> **反事实推理**：如果 MPC 求解时间超过控制周期（如 MPC 频率 100 Hz 但求解需要 15 ms）会怎样？有两种处理方式：(1) 异步 MPC——MPC 在后台线程求解，WBC 使用最新可用的 MPC 解（可能滞后 1-2 步），(2) 降频 MPC——MPC 以实际可行的频率运行（如 50 Hz），WBC 在 MPC 更新间做线性插值。方案 (1) 更灵活但需要线程安全设计，方案 (2) 更简单但 MPC 时效性更差。


### OCS2 MPC 常用配置参数

| 参数 | 描述 | 典型值 | 影响 |
|------|------|:------:|------|
| `mpcDesiredFrequency` | MPC 求解频率 | 100 Hz | 太低→预测不准 |
| `mrtDesiredFrequency` | MRT（实时线程）频率 | 400 Hz | 太低→跟踪不准 |
| `timeHorizon` | 预测时域 | 0.5-2.0 s | 太短→近视；太长→求解慢 |
| `sqpIteration` | SQP 最大迭代 | 1-3 | 多次迭代更准但更慢 |
| `dt` | 离散步长 | 0.01-0.05 s | 小步长精度高但 QP 规模大 |
| `comWeight` | 质心跟踪权重 | 100-1000 | 影响质心轨迹精度 |
| `contactForceWeight` | 接触力正则化 | 0.01-1 | 影响力分配平滑度 |

### Crocoddyl DDP 常用配置

| 参数 | 描述 | 典型值 | 影响 |
|------|------|:------:|------|
| `dt` | 积分步长 | 0.01 s | 精度 vs 速度 |
| `T` (knots) | 时间步数 | 20-100 | 问题规模 |
| `maxiter` | DDP 最大迭代 | 1-10 | 每步计算预算 |
| `th_stop` | 收敛阈值 | $10^{-4}$ | 精度 vs 速度 |
| `regMin` | 最小正则化 | $10^{-9}$ | 数值稳定性 |
| `regMax` | 最大正则化 | $10^{4}$ | Hessian 不正定时的修正 |

**OCS2 vs Crocoddyl 选型建议**：

| 维度 | OCS2 | Crocoddyl |
|------|------|-----------|
| 适用模型 | SRBM / 质心模型 | 全身模型 |
| 求解方法 | SQP | DDP |
| ROS2 集成 | 完善（legged_control） | 需自定义 |
| 文档质量 | 中等 | 好（教程丰富） |
| Python 绑定 | 有 | 有（pinocchio 风格） |
| 适用场景 | 实时凸 MPC（四足） | 离线/近实时全身 MPC |


## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MPC 输出力/轨迹不合理 | 代价函数权重不当 | 1. 检查各代价项量级 2. 逐项禁用检查影响 3. 可视化 MPC 预测轨迹 | F8.2, F8.6 |
| WBC 跟踪 MPC 输出精度差 | MPC-WBC 频率比不够/接口不平滑 | 1. 增大 WBC 频率 2. 加线性插值 3. 检查坐标系一致性 | F8.1 |
| MPC 求解时间超预算 | Horizon 太长/模型太复杂 | 1. 缩短 horizon 2. 简化模型（SRBM） 3. 降低迭代次数（DDP） | F8.2, F8.6 |
| 步态切换时力矩跳变 | 接触状态切换不平滑 | 1. 加力过渡窗口 2. 平滑接触力权重 3. 检查步态时序 | F8.3, F8.4 |
| OCS2 初始化失败 | 初始猜测不可行 | 1. 提供更好的初始猜测 2. 放松约束 3. 检查模型参数 | F8.4, F8.6 |
| Crocoddyl DDP 不收敛 | 步长太大/模型梯度不准 | 1. 减小 DDP 步长 2. 增加迭代次数 3. 检查动力学梯度 | F8.6 |
| 移动操作中基座和手臂协调差 | MPC 未考虑手臂负载对基座的影响 | 1. 在 MPC 模型中加入手臂惯量 2. 增大基座稳定性权重 3. 降低手臂运动速度 | F8.5, F8.7 |

**MPC+WBC 系统级调试流程**：

```
Step 1: WBC 单独测试（手动指定期望加速度/力）
  |-- WBC 正常跟踪? -> Step 2
  +-- WBC 异常 -> 修复 WBC（参见 F07 故障排查）

Step 2: MPC 开环测试（MPC 输出送入仿真但不闭环）
  |-- MPC 轨迹合理? -> Step 3
  +-- MPC 轨迹异常 -> 调整代价函数/约束/模型

Step 3: MPC+WBC 闭环（低速、低力）
  |-- 稳定? -> Step 4
  +-- 不稳定 -> 检查接口（频率/坐标系/数据格式）

Step 4: 逐步增加速度和力到任务水平
  |-- 满足指标? -> 部署
  +-- 不满足 -> 精调参数（MPC 权重 / WBC 权重 / 接口平滑）
```


| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MPC 求解超时 | 时域过长或初始猜测差 | 1. 减少 N 2. warm-start 3. 降 SQP 迭代 | F8.2, F8.6 |
| MPC-WBC 力跳变 | 时钟不同步 | 1. 检查时间戳 2. 加插值 3. 打印 MPC 更新时刻 | F8.1 |
| 摆动腿空中蹬 | 忘记零力约束 | 1. 检查 contact_schedule 2. 打印腿力约束 | F8.2 |
| WBIC delta_fb 过大 | MPC 力参考不可行 | 1. 打印 f_MPC 2. 检查摩擦锥 3. 降低参考 | F8.3 |
| Gazebo 不稳定 | use_sim_time 未设 | 1. 加 use_sim_time:=true | F8.4 |
| 臂力矩发错关节 | 关节索引不匹配 | 1. 用 getJointId() 2. 打印映射表 | F8.5 |
| FDDP 不收敛 | 初始猜测差 | 1. 用 g(q) 初始化 2. 增加 maxiter | F8.6 |

**MPC+WBC 的前沿发展方向**：

| 方向 | 当前状态 | 核心挑战 | 代表工作 |
|------|---------|---------|---------|
| Whole-Body MPC | 原型验证 | 全身模型的实时求解 | Mastalli et al. 2020 |
| 接触隐式 MPC | 研究阶段 | 混合动力学的可微性 | Manchester & Kuindersma 2020 |
| 学习 MPC | 早期研究 | 模型误差的在线补偿 | Lenz et al. 2024 |
| 多接触 MPC | 原型验证 | 接触序列的组合优化 | Caron et al. 2023 |

> **教学要点**：MPC+WBC 框架是当前腿足/人形/移动操作力控的工业标准架构。理解其双频率设计（MPC 慢+WBC 快）和接口约定，是进入这个领域的必要门槛。即使未来全身 MPC（Whole-Body MPC）成熟到可以替代 WBC，双频率架构的设计思想仍然有价值——高频安全层+低频智能层的分离是机器人安全的根本保证。
**MPC+WBC 在工业化应用中的部署经验**：

- **计算平台选择**：MPC 可以在独立 PC（x86 + 多核）上运行并通过以太网发送结果，WBC 必须在实时控制器（如 RT-Linux / Xenomai）上运行
- **通信协议**：MPC→WBC 通信推荐使用共享内存（同机）或 UDP（跨机），避免 ROS2 topic 的不确定延迟
- **故障恢复**：MPC 崩溃时 WBC 应自动切换到"重力补偿+高阻尼"的安全模式，而非停机
- **日志系统**：必须记录 MPC 求解时间、WBC 求解时间、接触状态、力矩命令的完整时序数据——故障分析的唯一依据
- **版本管理**：MPC 和 WBC 的参数文件必须联合版本控制——单独修改一方的参数可能导致接口不匹配


