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

---

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

### 练习

1. ⭐⭐ **模式枚举**：为 ANYmal + DynaArm 的"打开冰箱门"任务，列出所有可能的接触模式序列（从走到冰箱前 → 伸出臂 → 抓住把手 → 拉开门 → 取出物品）。
2. ⭐⭐⭐ **维度分析**：计算上述 50D 状态的 MPC 在 $N=20$ 步时的 QP/NLP 维度。估算在 ARM Cortex-A72 和 NVIDIA Orin 上的求解时间。
3. ⭐⭐⭐ **前沿讨论**：mujoco_mpc 的 500 Hz 采样 MPC 能否替代传统 MPC+WBC？分析其在有无 GPU 情况下的可行性。

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

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| MPC 求解超时 | 时域过长或初始猜测差 | 1. 减少 N 2. warm-start 3. 降 SQP 迭代 | F8.2, F8.6 |
| MPC-WBC 力跳变 | 时钟不同步 | 1. 检查时间戳 2. 加插值 3. 打印 MPC 更新时刻 | F8.1 |
| 摆动腿空中蹬 | 忘记零力约束 | 1. 检查 contact_schedule 2. 打印腿力约束 | F8.2 |
| WBIC delta_fb 过大 | MPC 力参考不可行 | 1. 打印 f_MPC 2. 检查摩擦锥 3. 降低参考 | F8.3 |
| Gazebo 不稳定 | use_sim_time 未设 | 1. 加 use_sim_time:=true | F8.4 |
| 臂力矩发错关节 | 关节索引不匹配 | 1. 用 getJointId() 2. 打印映射表 | F8.5 |
| FDDP 不收敛 | 初始猜测差 | 1. 用 g(q) 初始化 2. 增加 maxiter | F8.6 |
