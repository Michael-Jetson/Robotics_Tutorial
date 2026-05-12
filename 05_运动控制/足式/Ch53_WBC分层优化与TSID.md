> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 53 章 WBC 分层优化——TSID 精读 + 轻量 WBC 对照

### 前置自测

在开始阅读之前,请先尝试回答以下问题。如果有两个以上不确定,建议先复习 Ch47(Pinocchio)、Ch50(QP 求解器)和 Ch52(摩擦锥):

1. MPC 输出的是"未来 1 秒的质心轨迹",WBC 输出的是"当前时刻的关节扭矩"——这两层之间的**信息接口**是什么?WBC 需要哪些量作为输入?
2. 给定全身动力学方程 $M\ddot{q} + h = S^T\tau + J_c^T\lambda$,如果决策变量选 $(\ddot{q}, \lambda)$ 而不是 $(\tau, \ddot{q}, \lambda)$,**减少了多少维**?为什么能这样做?
3. 加权 QP 中设 $w_1 = 100, w_2 = 1$,为什么**不能保证** Task 1 绝对优先于 Task 2?
4. `EIGEN_RUNTIME_NO_MALLOC` 开启后,哪些 Eigen 操作会触发 assert 失败?请至少列出 3 种。
5. legged_control 的 WBC 只有约 300 行代码,而 TSID 有数千行——它做了哪些**简化**来实现这一点?

---

### 本章目标

学完本章,学员应能:

1. **说清楚 WBC 在控制层级中的定位**——MPC 的接力棒、关节扭矩的源头
2. **手推 WBC 的 QP 公式**——从全身动力学到矩阵组装,每一步都写得出来
3. **理解 HQP 的数学思想**——零空间投影的推导、为什么分层优于加权
4. **读懂 TSID 源码**并能为新机器人添加自定义 Task
5. **对比 TSID 和 legged_control 的轻量 WBC**——理解设计取舍
6. **掌握实时控制的内存约束**——`EIGEN_RUNTIME_NO_MALLOC` 的原理和实践
7. **能独立实现一个四足 WBC 并在仿真中跑通平衡控制**
8. **了解前沿进展**——VWBC、学习 WBC、Contact-Implicit WBC

### 前置依赖

**v8 主线前置**:

| 章节 | 关联知识 | 在本章中的用途 |
|------|---------|---------------|
| Ch11 + Ch22 | Eigen 表达式模板 / 对齐 / SIMD | WBC 是高频矩阵运算,必须理解 Eigen 的内存行为 |
| Ch17-20 | 并发与线程安全 | WBC 在 SCHED_FIFO 实时线程中运行 |
| Ch29 | 设计模式 - Strategy | TSID 的 Task/Constraint/Solver 是 Strategy 模式的教科书案例 |
| Ch35 | pmr 分配器 | WBC 的无堆分配替代方案 |

**本大纲前置**:

| 章节 | 关联知识 | 在本章中的用途 |
|------|---------|---------------|
| Ch47 | Pinocchio 动力学计算 | WBC 依赖 `aba()`, `crba()`, `computeJointJacobians()` |
| Ch49 | 接触 Jacobian | WBC 的接触约束 $J_c \ddot{q} = -\dot{J}_c \dot{q}$ |
| Ch50 | QP 求解器(OSQP, ProxQP, qpOASES) | WBC 的核心计算引擎 |
| Ch52 | 摩擦锥约束 | WBC 的接触力约束 |

---

### 53.1 控制层级——规划到关节的四级火箭 ⭐

#### 53.1.1 为什么需要分层?

**动机**:假设你要让一只四足机器人从 A 走到 B。最"简单"的方案是:把整个问题写成一个巨大的优化——同时决定步态、接触序列、质心轨迹、关节角度、关节扭矩。

这个"巨型优化"的问题维度有多大?以 Unitree Go2 为例:

| 量 | 维度 | 说明 |
|----|------|------|
| 关节位置 $q$ | 18 (6+12) | 浮动基座 6 + 12 关节 |
| 关节速度 $\dot{q}$ | 18 | |
| 关节扭矩 $\tau$ | 12 | 只有关节被驱动 |
| 接触力 $\lambda$ | 12 (4 足 $\times$ 3) | |
| **单时步决策变量** | **~60** | |
| MPC 预测步数 | 20-50 | 预测未来 0.5-1 秒 |
| **全时域决策变量** | **1200-3000** | 60 $\times$ 20~50 |

> 💡 **概念澄清**:全维优化不是不可能——Cafe-MPC (Li & Wensing, T-RO 2024) 就用全身动力学做 MPC。但它需要极其高效的求解器(定制 iLQR),而且预测步数受限。**分层是工程上更可扩展的方案**。

**历史视角**:控制分层不是新想法。Honda ASIMO (2000) 就已经使用了"步态规划 -> ZMP 控制 -> 关节伺服"的三层架构。但那时的 WBC 很简单——只是逆动力学,没有 QP 优化。现代 WBC 从 2010 年代开始引入 QP,代表工作包括:

- **Sentis & Khatib (2005)**:首次将操作空间控制(Operational Space Control)与优先级控制结合
- **Escande, Mansard & Wieber (2014)**:HQP 理论的完整数学框架
- **Kim et al. (2019)**:MIT Mini Cheetah 上的 MPC+WBC 实现,推动了小型四足机器人高动态 MPC+WBC 控制的广泛研究

#### 53.1.2 时间尺度金字塔

```
┌──────────────────────────────────────────────────────────┐
│ 任务层(行为树、FSM)                                      │
│ "这是一个 pick-and-place 任务"                            │
│ 频率:1-10 Hz    状态维度:离散(任务 ID、阶段)            │
│ 输出:目标位姿、步态类型                                   │
└─────────────────────┬────────────────────────────────────┘
                      │ 目标位姿 / 步态模式
┌─────────────────────▼────────────────────────────────────┐
│ 全局规划层(A*、RRT*、Graph Search)                       │
│ "从 A 点走到 B 点,避开障碍物"                             │
│ 频率:0.1-1 Hz   状态维度:SE(2) 或 SE(3)                 │
│ 输出:路径点序列                                           │
└─────────────────────┬────────────────────────────────────┘
                      │ 路径点 + 速度命令
┌─────────────────────▼────────────────────────────────────┐
│ MPC 层(Crocoddyl、OCS2、Cafe-MPC)                       │
│ "未来 1 秒的 CoM 轨迹 + 接触力 + 步态序列"                │
│ 频率:10-100 Hz  状态维度:~30(质心 + 动量 + 足端)        │
│ 输出:质心轨迹、足端轨迹、接触力参考                       │
│ 模型:简化模型(LIPM / Centroidal / SRBD)                 │
└─────────────────────┬────────────────────────────────────┘
                      │ x_com_ref, f_ref, p_foot_ref
┌─────────────────────▼────────────────────────────────────┐
│ WBC 层(TSID、legged_wbc)  <-- 本章                       │
│ "把 MPC 参考翻译为关节扭矩"                               │
│ 频率:500-1000 Hz  状态维度:~60(全身 q, dq)              │
│ 输出:关节扭矩 tau                                        │
│ 模型:全身刚体动力学(Pinocchio)                           │
└─────────────────────┬────────────────────────────────────┘
                      │ 关节扭矩 tau
┌─────────────────────▼────────────────────────────────────┐
│ 关节伺服层(电机驱动器内部 FOC)                           │
│ "tau -> 电流 -> PWM"                                     │
│ 频率:10-40 kHz   状态维度:电流/位置                      │
│ 输出:电机电压                                             │
└──────────────────────────────────────────────────────────┘
```

#### 53.1.3 频率分离原则

**为什么不同层运行在不同频率?** 这不是随意选择,而是由物理和计算共同决定的:

| 层 | 频率 | 决定因素 |
|----|------|---------|
| MPC | 10-100 Hz | 求解时间 10-100 ms;模型变化时间尺度 ~0.1 s |
| WBC | 500-1000 Hz | 关节动力学时间常数 ~1 ms;扭矩必须"紧跟"当前状态 |
| 伺服 | 10-40 kHz | 电机电气时间常数 ~0.1 ms |

> ⚠️ **编程陷阱**:WBC 的 1 kHz 频率意味着**每次调用必须在 1 ms 内完成**。这包括 Pinocchio 动力学计算(~0.1 ms)+ QP 求解(~0.1-0.5 ms)+ 数据交换开销。**任何堆分配、mutex 等待、缓存未命中都可能导致超时**。

**如果不分层会怎样?**

```
┌─────────────────────────────────────────┐
│ 方案 A:巨型 MPC(不分层)              │
│                                         │
│ 优点:理论上最优(全局一致性)           │
│ 缺点:                                  │
│   1. 维度爆炸 -> 求解太慢               │
│   2. 全身模型非线性强 -> 收敛困难       │
│   3. 频率只能到 10-50 Hz               │
│      -> 关节响应延迟 20-100 ms          │
│      -> 走路时足底打滑                  │
│                                         │
│ 方案 B:分层 MPC + WBC                  │
│                                         │
│ MPC(简化模型,50 Hz)-> 参考轨迹       │
│ WBC（全身模型,1 kHz）-> 关节扭矩      │
│                                         │
│ 优点:                                  │
│   1. 各层独立优化,易于调试             │
│   2. WBC 高频 -> 即时响应扰动           │
│   3. MPC 用简化模型 -> 可预测更长时间   │
│ 缺点:                                  │
│   MPC 与 WBC 的模型不一致 -> 性能折损   │
└─────────────────────────────────────────┘
```

> 🧠 **思维陷阱**:不要认为"分层一定不如全局优化"。实际上分层的**鲁棒性**往往更好:MPC 求解失败时,WBC 仍能用上一次的参考维持平衡。全局方案一旦求解失败,整个控制链断裂。

**练习 53.1.A** ⭐:画出你所熟悉的一个机器人系统的控制层级图。标注每层的频率、输入输出、使用的模型。如果某些层被合并了(例如没有独立的 WBC 层),分析这种设计的优劣。

**练习 53.1.B** ⭐:假设你有一个超强求解器能在 0.5 ms 内求解全身 MPC。你还需要 WBC 层吗?讨论取消 WBC 层的利弊(提示:考虑模型不确定性和传感器噪声)。

---

### 53.2 WBC 的数学形态——带约束的全身逆动力学 ⭐⭐

#### 53.2.1 从动力学到 QP:完整推导

**起点**:我们有全身刚体动力学方程(Ch47 Pinocchio 计算的核心):

$$M(q) \ddot{q} + h(q, \dot{q}) = S^T \tau + \sum_{c} J_c^T \lambda_c \tag{53.1}$$

其中:
- $M(q) \in \mathbb{R}^{n_v \times n_v}$:广义质量矩阵(对称正定)
- $h(q, \dot{q}) \in \mathbb{R}^{n_v}$:科氏力 + 重力项
- $S \in \mathbb{R}^{n_a \times n_v}$:选择矩阵,$S = [0_{n_a \times 6} \;\; I_{n_a}]$,选出驱动关节
- $\tau \in \mathbb{R}^{n_a}$:关节扭矩(决策变量)
- $J_c \in \mathbb{R}^{3n_c \times n_v}$:接触 Jacobian(所有接触点堆叠)
- $\lambda_c \in \mathbb{R}^{3n_c}$:接触力(决策变量)

**Go2 的具体维度**:

| 符号 | 含义 | Go2 数值 |
|------|------|---------|
| $n_v$ | 广义速度维度 | 18 (6 基座 + 12 关节) |
| $n_a$ | 驱动关节数 | 12 |
| $n_c$ | 接触点数 | 4 (四足着地) |
| $3n_c$ | 接触力维度 | 12 |

#### 53.2.2 欠驱动基座的处理

动力学方程 (53.1) 的**前 6 行**对应浮动基座(无驱动器):

$$M_{b}(q) \ddot{q} + h_{b}(q, \dot{q}) = J_{c,b}^T \lambda_c \tag{53.2}$$

注意**没有 $\tau$**——基座不受电机驱动,只通过接触力间接控制。这是 WBC 的核心约束之一:基座的加速度完全由接触力决定。

**后 $n_a$ 行**对应驱动关节:

$$M_{j}(q) \ddot{q} + h_{j}(q, \dot{q}) = \tau + J_{c,j}^T \lambda_c \tag{53.3}$$

> 💡 **概念澄清**:方程 (53.2) 不是"约束",而是物理规律——浮动基座必须通过地面反力来加速。这意味着 WBC 的解必须同时满足"足底力产生正确的基座加速度"和"摩擦锥限制足底力"——这就是 WBC 问题的核心张力。这相当于一位木偶师(WBC)只能通过拉绳子(接触力)来控制木偶身体(浮动基座)的运动,但每根绳子的拉力不能超过上限(摩擦锥),且绳子只能拉不能推(法向非负)。

#### 53.2.3 约束集合

**等式约束 (a)——动力学方程**:

将 (53.1) 改写为关于决策变量 $(\ddot{q}, \lambda_c, \tau)$ 的线性约束:

$$\underbrace{[M \;\; -J_c^T \;\; -S^T]}_{A_{dyn}} \begin{bmatrix} \ddot{q} \\ \lambda_c \\ \tau \end{bmatrix} = \underbrace{-h}_{b_{dyn}} \tag{53.4}$$

**等式约束 (b)——接触不滑动**:

接触点的加速度为零(保持静止接触):

$$J_c \ddot{q} + \dot{J}_c \dot{q} = 0 \quad \Rightarrow \quad J_c \ddot{q} = -\dot{J}_c \dot{q} \tag{53.5}$$

写成标准形式:

$$\underbrace{[J_c \;\; 0 \;\; 0]}_{A_{contact}} \begin{bmatrix} \ddot{q} \\ \lambda_c \\ \tau \end{bmatrix} = \underbrace{-\dot{J}_c \dot{q}}_{b_{contact}} \tag{53.6}$$

**不等式约束 (c)——摩擦锥**:

回顾 Ch52:库仑摩擦锥 $\|\lambda^t\| \le \mu \lambda^n$ 是二阶锥约束(SOC),为了在 QP 中使用,需要将其线性化为 $k$ 条边的多面体近似。这里我们采用 $k=4$ 的外切近似(即 box cone),误差约 27% 但计算最快。Ch52 还分析了 $k=8$ 和内切近似的权衡——在 WBC 的 1kHz 频率需求下,$k=4$ 或 $k=8$ 是工程上的主流选择。

线性化后的摩擦锥约束(每个接触点 5 个不等式):

$$D_i \lambda_{c,i} \le 0, \quad i = 1, \ldots, n_c \tag{53.7}$$

其中 $D_i \in \mathbb{R}^{5 \times 3}$ 是摩擦锥的线性近似矩阵(Ch52 详细推导了该矩阵的构造):

$$D = \begin{bmatrix} 1 & 0 & -\mu \\ -1 & 0 & -\mu \\ 0 & 1 & -\mu \\ 0 & -1 & -\mu \\ 0 & 0 & -1 \end{bmatrix}$$

第 1-4 行是摩擦锥的四棱锥近似:$|f_x| \le \mu f_z$ 和 $|f_y| \le \mu f_z$;第 5 行保证法向力非负:$f_z \ge 0$。

**不等式约束 (d)——扭矩限制**:

$$-\tau_{\max} \le \tau \le \tau_{\max} \tag{53.8}$$

**代价函数 (e)——追踪 MPC 参考**:

$$\min \sum_k w_k \| A_k \begin{bmatrix} \ddot{q} \\ \lambda_c \\ \tau \end{bmatrix} - b_k^{\text{ref}} \|^2 \tag{53.9}$$

典型追踪任务包括:
- 质心加速度追踪:$\ddot{x}_G \approx \ddot{x}_G^{\text{ref}}$
- 足端追踪(摆动腿):$\ddot{p}_{foot} \approx \ddot{p}_{foot}^{\text{ref}}$
- 基座姿态追踪:$\ddot{\theta}_{base} \approx \ddot{\theta}_{base}^{\text{ref}}$
- 关节正则化:$\ddot{q}_{joint} \approx K_p(q^{ref} - q) + K_d(\dot{q}^{ref} - \dot{q})$

#### 53.2.4 组装完整的 QP

将所有约束和代价函数组装为标准 QP:

$$\min_{x} \frac{1}{2} x^T H x + g^T x$$
$$\text{s.t.} \quad A_{eq} x = b_{eq}, \quad A_{ineq} x \le b_{ineq}$$

其中 $x = [\ddot{q}^T, \lambda_c^T, \tau^T]^T \in \mathbb{R}^{n_v + 3n_c + n_a}$。

**Go2 四足的 QP 维度**:

| 矩阵 | 维度 | Go2 数值 |
|-------|------|---------|
| 决策变量 $x$ | $n_v + 3n_c + n_a$ | $18+12+12=42$ |
| 等式约束(动力学) | $n_v$ | 18 |
| 等式约束(接触不滑) | $3n_c$ | 12 |
| 不等式约束(摩擦锥) | $5n_c$ | 20 |
| 不等式约束(扭矩限制) | $2n_a$ | 24 |
| **QP 总规模** | | **42 变量, 30 等式, 44 不等式** |

> ⚠️ **编程陷阱**:这个 QP 只有 42 维,用 ProxQP/eiquadprog 求解只需 0.1-0.3 ms。**千万不要用 Ipopt/SNOPT 等非线性求解器**——它们的启动开销就超过 1 ms。Ch50 详细讨论了各 QP 求解器的性能特征。

如果不用摩擦锥约束会怎样?求解器会给出物理上不可能的接触力——例如水平摩擦力远超 $\mu \lambda_z$,机器人在仿真中"一步迈出"后瞬间打滑倒地。实物上更危险:电机会按照不可实现的力指令输出最大电流,导致关节过热甚至损坏减速器。Ch52 的三大铁律正是 WBC 约束矩阵中不等式行的物理来源。

**不同机器人的 QP 规模对比**:

| 机器人 | $n_v$ | $n_a$ | $n_c$(最大） | 决策变量 | 求解时间(典型) |
|--------|-------|-------|-------------|---------|---------------|
| Go2 四足 | 18 | 12 | 4 | 42 | 0.1-0.3 ms |
| A1 四足 | 18 | 12 | 4 | 42 | 0.1-0.3 ms |
| Atlas 人形 | 36 | 30 | 2 | 72 | 0.3-0.8 ms |
| H1 人形 | 25 | 19 | 2 | 50 | 0.2-0.5 ms |
| Panda 臂 | 7 | 7 | 0 | 7 | <0.05 ms |

**练习 53.2.A** ⭐⭐:为平面三连杆机器人(2D,3 个关节,无浮动基座)手写 WBC 的 QP 矩阵。决策变量是 $(\ddot{q}, \tau)$(没有接触力),约束只有动力学方程和关节扭矩限制。用 Eigen + osqp-eigen 求解,验证扭矩是否满足限制。

**练习 53.2.B** ⭐⭐⭐:在 53.2.A 的基础上,加入一个"足尖接触地面"的约束。将机器人变为浮动基座,增加接触力 $\lambda$ 和摩擦锥约束。对比有无摩擦锥约束时,解的差异。

---

### 53.3 "为什么要分层而不是加权"——HQP 核心洞察 ⭐⭐⭐

#### 53.3.1 加权 QP 的根本缺陷

**朴素做法**:把多个目标加权求和做成单个 QP:

$$\min \; w_1 \|\ddot{x}_G - \ddot{x}_G^{\text{ref}}\|^2 + w_2 \|\tau\|^2 + w_3 \|\ddot{p}_{ee} - \ddot{p}_{ee}^{\text{ref}}\|^2 \tag{53.10}$$

**问题 1:权重与物理量级耦合**

设 $w_1 = 10, w_2 = 1$,看起来"追踪重要、扭矩次要"。但实际行为取决于各项的数值量级:

| 项 | 典型量级 | 加权后 |
|----|---------|--------|
| $\|\ddot{x}_G\|^2$ | $(1 \text{ m/s}^2)^2 = 1$ | $10 \times 1 = 10$ |
| $\|\tau\|^2$ | $(50 \text{ Nm})^2 = 2500$ | $1 \times 2500 = 2500$ |

**结果**:扭矩项主导代价函数,与直觉完全相反!要修正这个问题,你需要把权重设为 $w_1 = 2500, w_2 = 1$——但这个"2500"**完全取决于当前运动状态**,没有泛化能力。

**问题 2:不能表达"绝对优先级"**

"保持平衡"和"末端追踪"之间的关系不是"重要性差 10 倍",而是**质的差别**:平衡丢了,机器人摔倒,末端追踪毫无意义。用权重表达这种"绝对优先"需要 $w_1 / w_2 \to \infty$——但这会导致 QP 的 Hessian 矩阵条件数退化,求解器不收敛。

> ⚠️ **工程陷阱：加权 QP 的"污染"问题**
>
> 在 WQP（Weighted QP）中，不同任务通过权重区分优先级。但一个**低权重任务**如果其雅可比矩阵范数较大或瞬时误差较大，仍可能在优化过程中"污染"高优先级任务的执行——因为 QP 求解器在最小化加权代价时会折中所有任务。
>
> 这是 WQP 相对于严格分层 HQP 的本质缺陷：**WQP 没有严格的优先级保证**。如果高优先级任务的执行质量不可妥协（如支撑腿约束），应使用 HQP 或 NSP（零空间投影）。

> 🧠 **思维陷阱**:很多人认为"把权重设得很大就能保证优先级"。数学上不成立。设 $w_1 = 10^6$,那么 Hessian 矩阵的条件数 $\kappa(H) \ge 10^6$,在 64 位浮点下只剩 ~10 位有效数字,QP 求解可能给出垃圾结果。

#### 53.3.2 HQP 的数学框架

**HQP (Hierarchical Quadratic Programming)** 的核心思想:将任务按优先级排列,**逐层求解**,低层不破坏高层的最优性。

**关键论文**:Escande A., Mansard N., Wieber P.-B. (2014) *"Hierarchical quadratic programming: Fast online humanoid-robot motion generation"*, IJRR, 33(7):1006-1028。该论文提出了一种完整的方法来求解包含等式和不等式约束的多层最小二乘问题,比传统的迭代投影方法快 10 倍以上。

**形式化定义**:给定 $p$ 层优先级,每层有任务 $(A_k, b_k)$ 和约束 $(C_k, d_k)$:

$$\text{lexmin}_{x} \left( \|A_1 x - b_1\|^2, \|A_2 x - b_2\|^2, \ldots, \|A_p x - b_p\|^2 \right)$$

$$\text{s.t.} \quad C_k x \le d_k, \quad k = 1, \ldots, p$$

**"lexmin"(字典序最小化)**:先最小化第 1 层,在所有使第 1 层最优的解中最小化第 2 层,以此类推。这好比医院急诊的分级诊疗:危重病人(Level 1)无条件优先,普通病人(Level 2)只能在不影响危重病人的前提下得到治疗。不同于加权方案(给每个病人打一个"紧急程度分数"然后排序),分级制度保证了危重病人的绝对优先权,不会因为普通病人数量多而被"稀释"。

#### 53.3.3 零空间投影——HQP 的数学引擎

**Level 1**:求解最高优先级任务:

$$x_1^* = \arg\min_{x} \|A_1 x - b_1\|^2 \quad \text{s.t.} \; C_1 x \le d_1$$

如果无不等式约束,解为最小二乘解 $x_1^* = A_1^+ b_1$,其中 $A_1^+$ 是 Moore-Penrose 伪逆。

**零空间投影矩阵**:

$$N_1 = I - A_1^+ A_1 \tag{53.11}$$

$N_1$ 将任何向量投影到 $A_1$ 的零空间。**在这个子空间里改变 $x$ 不会影响 $A_1 x$ 的值**——这是整个 HQP 理论的基石。

> **本质洞察**:零空间投影的意义不是让次要任务完美执行,而是在**完美保护主任务的前提下,尽最大努力去近似次要任务**。当零空间维度不足时,低优先级任务会被部分甚至完全牺牲——这不是缺陷,而是"优先级控制"的本质:有限的自由度必须分配给最重要的目标。

**直觉理解**:假设 $A_1$ 是一个 $3 \times 5$ 矩阵(3 个方程,5 个未知数)。最小二乘解确定了解空间中的一个 2 维平面(5 - 3 = 2 维零空间)。Level 2 只能在这个 2 维平面上优化,不能"跳出"平面。

**Level 2**:在不破坏 Level 1 的前提下优化 Level 2:

$$x_2 = x_1^* + N_1 \delta_2 \tag{53.12}$$

代入 Level 2 的目标:

$$\|A_2 x_2 - b_2\|^2 = \|A_2(x_1^* + N_1 \delta_2) - b_2\|^2 = \|\underbrace{A_2 N_1}_{\tilde{A}_2} \delta_2 - \underbrace{(b_2 - A_2 x_1^*)}_{\tilde{b}_2}\|^2$$

这是一个**关于 $\delta_2$ 的新最小二乘问题**:

$$\delta_2^* = \tilde{A}_2^+ \tilde{b}_2 = (A_2 N_1)^+ (b_2 - A_2 x_1^*) \tag{53.13}$$

**Level 3**:用累积零空间 $N_2 = N_1 (I - \tilde{A}_2^+ \tilde{A}_2)$ 继续投影,以此类推。

**关键性质的证明**:Level 2 不破坏 Level 1:

$$A_1 x_2 = A_1 (x_1^* + N_1 \delta_2) = A_1 x_1^* + A_1 N_1 \delta_2$$

$$= A_1 x_1^* + A_1 (I - A_1^+ A_1) \delta_2 = A_1 x_1^* + (A_1 - A_1) \delta_2 = A_1 x_1^* \quad \square$$

> 💡 **几何直觉**:想象 3D 空间中,Level 1 约束 $x$ 在一条线上(1D 子空间)。Level 2 的自由度只有沿这条线滑动的方向。每增加一层任务,可用自由度减少。当自由度耗尽时,更低层的任务完全被忽略——这是设计优先级时必须考虑的。

#### 53.3.3b 为什么 Moore-Penrose 伪逆是最优的——拉格朗日乘子法证明

上文直接使用了伪逆 $A^+ = A^T(AA^T)^{-1}$ 来获取最小二乘解，但一个自然的问题是：对于欠定方程 $Ax = b$（$A$ 是 $m \times n$ 矩阵，$m < n$），存在无穷多个右逆矩阵 $G$ 满足 $AG = I$，为什么偏偏选 $A^+$？

答案来自**能量最优性**：在满足任务约束的前提下，我们希望找到**范数最小**的解——即用最小的"动作量"完成任务，避免不必要的剧烈运动。这可以形式化为一个带等式约束的优化问题：

$$\min_{x} \frac{1}{2} \|x\|^2 \quad \text{s.t.} \quad Ax = b$$

引入 $m \times 1$ 的拉格朗日乘子向量 $\boldsymbol{\lambda}$，构造拉格朗日函数：

$$\mathcal{L}(x, \boldsymbol{\lambda}) = \frac{1}{2} x^T x - \boldsymbol{\lambda}^T (Ax - b)$$

对 $x$ 和 $\boldsymbol{\lambda}$ 分别求偏导并令其为零：

$$\nabla_x \mathcal{L} = x - A^T \boldsymbol{\lambda} = 0 \quad \Longrightarrow \quad x = A^T \boldsymbol{\lambda}$$

$$\nabla_{\boldsymbol{\lambda}} \mathcal{L} = -(Ax - b) = 0 \quad \Longrightarrow \quad Ax = b$$

将第一式代入第二式：$A(A^T \boldsymbol{\lambda}) = b$，即 $(AA^T)\boldsymbol{\lambda} = b$。当 $A$ 行满秩时（非奇异构型下冗余机器人的雅可比矩阵满足此条件），$AA^T$ 可逆，解得：

$$\boldsymbol{\lambda} = (AA^T)^{-1} b$$

代回第一式：

$$x^* = A^T (AA^T)^{-1} b = A^+ b$$

这就证明了：**满足 $Ax = b$ 约束的唯一最小范数解，正是由 Moore-Penrose 伪逆给出的**。

> **更深刻的数学性质**：$A^+ b$ 位于 $A$ 的行空间 $\mathcal{R}(A^T)$ 中——而行空间与零空间 $\mathcal{N}(A)$ 正交互补（秩-零度定理）。因此最小范数解天然地不包含任何零空间分量，是"纯粹完成任务"的那部分运动。这也是零空间投影 $N = I - A^+ A$ 能将任意向量正交投影到 $\mathcal{N}(A)$ 的根本原因——投影后的向量与原始向量之间的欧氏距离最小。

> ⚠️ **工程注意**：当 $A$ 接近奇异（接近工作空间边界）时，$AA^T$ 的行列式趋近于零，直接求逆会导致数值爆炸。工程中常用**阻尼最小二乘法**（DLS）替代：$A^+ \approx A^T(AA^T + \lambda^2 I)^{-1}$。微小的阻尼项 $\lambda^2$ 保证了矩阵始终可逆，但代价是解不再精确满足 $Ax = b$——这是"精度 vs 数值稳定性"的经典权衡。

#### 53.3.4 级联 QP 实现

实际的 HQP 实现并不直接计算零空间投影(伪逆的数值稳定性差)。Escande et al. (2014) 提出了**级联 QP**方法:

```
算法:级联 QP (Cascaded QP)
输入:p 层任务 {(A_k, b_k, C_k, d_k)}

1. 解 Level 1 QP:
   x1* = argmin ||A1*x - b1||^2  s.t. C1*x <= d1
   w1* = ||A1*x1* - b1||^2  (最优代价)

2. for k = 2, ..., p:
   解 Level k QP:
   xk* = argmin ||Ak*x - bk||^2
   s.t. Ck*x <= dk                     (本层约束)
        ||Aj*x - bj||^2 <= wj* + eps   (保持上层最优, j < k)

   wk* = ||Ak*xk* - bk||^2

3. 输出: xp*
```

> ⚠️ **编程陷阱**:约束 $\|A_j x - b_j\|^2 \le w_j^* + \varepsilon$ 是**二次约束**,不能直接放进标准 QP。TSID 的实现将其转化为线性等式约束:固定上层的残差方向,只允许在零空间中移动。这种线性化是 `solver-HQuadProg-fast.cpp` 的核心技巧。

#### 53.3.5 三种 QP 策略对比

| 策略 | 优先级保证 | 计算复杂度 | 调参难度 | 典型实现 |
|------|-----------|-----------|---------|---------|
| **加权 QP** | 无(软优先级) | 1 次 QP | 高(权重敏感) | 简单手写 |
| **严格 HQP** | 数学保证 | $p$ 次 QP | 低(只排优先级) | TSID HQuadProg |
| **加权 + 优先级约束** | 近似保证 | 1 次 QP + 约束 | 中等 | legged_control |

**四足典型优先级设置**(3 层):

```
Level 1 (硬约束):
  - 动力学方程:M*ddq + h = S^T*tau + Jc^T*lambda
  - 接触不滑:Jc*ddq = -dJc*dq
  - 摩擦锥:D*lambda <= 0
  - 扭矩限制:-tau_max <= tau <= tau_max

Level 2 (高优先级软目标):
  - 基座姿态追踪(保持水平)
  - 质心位置追踪(防摔倒)
  - 摆动腿足端追踪(步态执行)

Level 3 (低优先级软目标):
  - 关节姿态正则化(接近 nominal pose)
  - 接触力正则化(||lambda - lambda_ref||^2)
```

> 🧠 **思维陷阱**:有人认为"HQP 一定比加权 QP 好"。**不一定**。HQP 需要解 $p$ 次 QP,计算量更大。对于实时性要求极高的场景(如 1 kHz WBC),有时用加权 QP + 合理权重比 HQP 更实用。legged_control 就是这种思路——用 2 层优先级 + 加权的混合方案。

#### 53.3.6 四足机器人的经典四任务分层与 NSP 递推公式

在基于零空间投影（NSP）的经典 WBC 实现中，四足机器人的控制任务通常分为以下四个优先级。这种分层的物理直觉是：四足依靠支撑腿与地面的稳定接触来实现身体控制，支撑腿的约束是一切运动的前提；机身姿态（保持水平）对平衡至关重要；机身位置（高度控制）次之；摆动腿轨迹只要大致正确即可。

| 优先级 | 任务 | 物理含义 |
|--------|------|---------|
| P1 (最高) | 支撑腿足端静止 | 触地脚在世界系中保持不动（$\dot{\mathbf{x}}_{\text{sup}} = \mathbf{0}$），防止打滑导致摔倒 |
| P2 | 机身转动控制 | 控制 Roll/Pitch/Yaw 角跟随期望姿态 |
| P3 | 机身平动控制 | 控制质心位置（尤其是高度）跟随期望轨迹 |
| P4 (最低) | 摆动腿足端追踪 | 控制空中腿沿规划轨迹运动至下一个落脚点 |

对应的 NSP 递推公式在加速度级别的完整形式为：

$$\ddot{\mathbf{q}}_1^{\text{cmd}} = J_1^+ (-\dot{J}_1 \dot{\mathbf{q}})$$

$$\ddot{\mathbf{q}}_i^{\text{cmd}} = \ddot{\mathbf{q}}_{i-1}^{\text{cmd}} + (J_i N_{i-1}^A)^+ \left( \ddot{\mathbf{x}}_i^{\text{cmd}} - \dot{J}_i \dot{\mathbf{q}} - J_i \ddot{\mathbf{q}}_{i-1}^{\text{cmd}} \right), \quad i = 2, 3, 4$$

其中 $\ddot{\mathbf{x}}_i^{\text{cmd}} = \ddot{\mathbf{x}}_i^d + K_p^i (\mathbf{x}_i^d - \mathbf{x}_i) + K_d^i (\dot{\mathbf{x}}_i^d - \dot{\mathbf{x}}_i)$ 是带 PD 反馈的期望加速度，$N_{i-1}^A = I - (J_{i-1}^A)^+ J_{i-1}^A$ 是前 $i-1$ 个任务的累积零空间投影矩阵，$J_{i-1}^A = [J_1^T, \ldots, J_{i-1}^T]^T$ 是堆叠雅可比矩阵。最终输出 $\ddot{\mathbf{q}}_4^{\text{cmd}}$ 即为融合了全部四个优先级任务的关节加速度指令。

> **NSP vs 优化 WBC 的取舍**：上述 NSP 递推公式有解析形式，计算极快，但无法处理不等式约束（如摩擦锥、力矩限幅）。工程中更常见的做法是将上述优先级思想转化为 HQP 或加权 QP 形式（如 TSID），在保留分层精神的同时获得约束处理能力。

**练习 53.3.A** ⭐⭐:用 Python + numpy 实现零空间投影法。构造一个 3 层优先级问题(2D 空间):Level 1 约束 $x_1 + x_2 = 1$,Level 2 最小化 $\|x - [3,3]^T\|^2$,Level 3 最小化 $\|x\|^2$。验证每层是否不破坏上层。

**练习 53.3.B** ⭐⭐⭐:对同一个问题,用加权 QP 方法,尝试不同的权重比($w_1:w_2:w_3 = 100:10:1$ 和 $10000:100:1$),观察解如何变化。与 HQP 的解对比,讨论在什么情况下加权 QP 的解"足够接近" HQP。

---

### 53.4 TSID 的 Task / Constraint / Solver 三元分离 ⭐⭐

上一节从数学层面回答了"为什么分层优于加权"。但数学上的优美并不自动转化为工程上的可用性——如何将 HQP 的零空间投影、多层 QP 封装成一个可扩展、可维护的软件框架?这正是 TSID 要解决的问题。

#### 53.4.1 设计哲学

TSID (Task Space Inverse Dynamics) 是 Pinocchio 生态的**工业级 WBC 实现**,由 LAAS-CNRS 的 Andrea Del Prete 等人开发。当前版本为 1.9.x,同时提供 C++ 和 Python 接口(通过 EigenPy 和 Boost.Python 绑定)。

TSID 的核心设计思想是 **Strategy Pattern** (Ch29) 的教科书级应用:将"描述要做什么"(Task)、"描述必须满足什么"(Constraint)、"如何求解"(Solver) 三者完全分离:

```
┌─────────────────────────────────────────────────────┐
│              InverseDynamicsFormulation              │
│                                                     │
│  ┌───────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ TaskBase  │  │ConstraintBase│  │ SolverBase   │  │
│  │           │  │              │  │              │  │
│  │ ┌───────┐ │  │ ┌──────────┐ │  │ ┌──────────┐ │  │
│  │ │TaskSE3│ │  │ │Equality  │ │  │ │HQuadProg │ │  │
│  │ ├───────┤ │  │ ├──────────┤ │  │ ├──────────┤ │  │
│  │ │TaskCoM│ │  │ │Inequality│ │  │ │ProxQP    │ │  │
│  │ ├───────┤ │  │ ├──────────┤ │  │ ├──────────┤ │  │
│  │ │Posture│ │  │ │Bound     │ │  │ │eiquadprog│ │  │
│  │ └───────┘ │  │ └──────────┘ │  │ └──────────┘ │  │
│  └───────────┘  └──────────────┘  └──────────────┘  │
│                                                     │
│  addMotionTask(task, weight, priority_level)         │
│  addContactTask(contact)                             │
│  computeProblemData(t, q, v) -> HQPData              │
└─────────────────────────────────────────────────────┘
```

**为什么这个分离很重要**:这好比乐高积木的标准化接口——每块积木(Task/Constraint/Solver)的内部结构可以完全不同,但只要接口形状一致就能自由组合。TSID 的 Strategy Pattern 正是让 WBC 的各个"积木"可以独立替换而不影响整体:换一个 QP 后端就像换一块积木,不需要重建整座城堡。

| 场景 | 需要修改的部分 | 不需要改的部分 |
|------|--------------|--------------|
| 添加新任务(如"保持末端水平") | 继承 TaskBase | Solver, Formulation |
| 换 QP 后端(如 qpOASES -> ProxQP) | 继承 SolverBase | Task, Formulation |
| 换机器人(如 Go2 -> H1) | 换 URDF + 调参 | 所有代码 |
| 增加约束(如关节速度限制) | 继承 ConstraintBase | Task, Solver |

#### 53.4.2 TaskBase 接口详解

```cpp
// tsid/include/tsid/tasks/task-base.hpp (simplified)
class TaskBase {
 public:
  // Compute task residual and Jacobian at current time
  virtual const ConstraintBase& compute(
      double t,
      const Eigen::VectorXd& q,   // joint positions
      const Eigen::VectorXd& v,   // joint velocities
      pinocchio::Data& data       // Pinocchio cache
  ) = 0;

  // Return the constraint object (equality/inequality/bound)
  virtual const ConstraintBase& getConstraint() const = 0;

  // Task name (for debug output)
  const std::string& name() const;

  // Task type determines which block of decision variables
  // TaskMotion:    target is J*ddq = desired_acc
  // TaskForce:     target is lambda = desired_force
  // TaskActuation: target is tau = desired_torque
  enum TaskType { MOTION, FORCE, ACTUATION };
};
```

**关键方法 `compute()` 的职责**(以 `TaskSE3Equality` 为例):

1. 调用 Pinocchio 的 `getFrameJacobian()` 获取末端 Jacobian $J_{ee}$
2. 调用 `getFrameAcceleration()` 获取 $\dot{J}_{ee}\dot{q}$(Jacobian 时间导数项)
3. 计算位姿误差:$e = \text{log}(R_{ref}^T R_{cur})$（旋转部分使用 SE3 对数映射）
4. 计算期望加速度:$\ddot{x}_{des} = \ddot{x}_{ref} + K_d \dot{e} + K_p e$
5. 填入约束矩阵:$J_{ee} \ddot{q} = \ddot{x}_{des} - \dot{J}_{ee}\dot{q}$

> ⚠️ **编程陷阱**:`compute()` 内部**不能有堆分配**,因为 TSID 默认开启 `EIGEN_RUNTIME_NO_MALLOC`(53.6 节详述)。所有临时矩阵必须预分配为成员变量。如果你继承 TaskBase 实现自定义 Task,这一点至关重要。

#### 53.4.3 主要 Task 类型

| Task 类 | 追踪目标 | Jacobian 来源 | 典型应用 |
|---------|---------|-------------|---------|
| `TaskSE3Equality` | 末端 6D 位姿 | `getFrameJacobian()` | 机械臂末端追踪、四足足端 |
| `TaskComEquality` | 质心位置 | `jacobianCenterOfMass()` | 四足/人形平衡 |
| `TaskJointPosture` | 关节角度 | $I_{n_a}$ (单位矩阵) | 关节正则化 |
| `TaskAMEquality` | 角动量 | 质心动量矩阵 | 人形步行保持角动量 |
| `TaskJointBounds` | 关节限位 | 边界约束 | 安全 |
| `TaskCopEquality` | 压力中心(CoP) | 接触力 Jacobian | 人形全脚掌平衡 |

#### 53.4.4 ContactModel6d vs ContactPoint

TSID 提供两种接触模型:

| 接触模型 | 力维度 | 适用场景 | 约束类型 |
|---------|--------|---------|---------|
| `ContactPoint` | 3D 力 | 点接触(四足足尖) | 线性摩擦锥($5 \times 3$ 矩阵) |
| `Contact6d` | 6D 力矩 | 面接触(人形全脚掌) | 摩擦锥 + CoP 在脚掌内 + 力矩约束 |

> 💡 **概念澄清**:四足机器人通常用 `ContactPoint`(每足 3 维力);人形机器人用 `Contact6d`(每脚 6 维力+力矩),因为脚掌是面接触,需要约束 CoP(压力中心)不超出脚掌边缘,否则脚会"翻倒"。`Contact6d` 的约束矩阵更大(17 行 vs 5 行),QP 也更大。

**如何添加自定义 Task(分步指南)**:

```cpp
// Step 1: Inherit TaskBase (or TaskMotion for motion tasks)
class TaskEndEffectorVertical : public tsid::tasks::TaskMotion {
 public:
  TaskEndEffectorVertical(const std::string& name,
                          tsid::robots::RobotWrapper& robot,
                          const std::string& frame_name);

  // Step 2: Implement compute()
  const ConstraintBase& compute(double t,
                                const Eigen::VectorXd& q,
                                const Eigen::VectorXd& v,
                                pinocchio::Data& data) override {
    // Get end-effector rotation matrix from Pinocchio
    auto frame_id = m_robot.model().getFrameId(m_frame_name);
    const auto& oMf = data.oMf[frame_id];
    Eigen::Vector3d z_axis = oMf.rotation().col(2);

    // Error: z_axis should be [0, 0, 1] (vertical)
    Eigen::Vector3d e = z_axis - Eigen::Vector3d::UnitZ();

    // Desired angular acceleration (PD control)
    // J_angular * ddq = -Kp * e - Kd * de
    m_constraint.setMatrix(m_J_angular);    // pre-allocated
    m_constraint.setVector(m_desired_acc);  // pre-allocated

    return m_constraint;
  }

 private:
  std::string m_frame_name;
  // Pre-allocated matrices (NO runtime malloc!)
  Eigen::MatrixXd m_J_angular;
  Eigen::VectorXd m_desired_acc;
  tsid::math::ConstraintEquality m_constraint;
};
```

#### 53.4.5 典型使用代码

```cpp
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/solvers/solver-proxqp.hpp>

// 1. Load Go2 model
tsid::robots::RobotWrapper robot(
    "go2.urdf",
    std::vector<std::string>(),  // package search paths
    pinocchio::JointModelFreeFlyer()  // floating base
);

// 2. Create Formulation
tsid::InverseDynamicsFormulationAccForce formulation(
    "go2_wbc", robot, false  // false = no debug print
);
formulation.computeProblemData(0.0, q0, v0);

// 3. Add 4 foot contacts
const std::array<std::string, 4> foot_names =
    {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
for (int i = 0; i < 4; ++i) {
    auto contact = std::make_shared<tsid::contacts::ContactPoint>(
        "foot_" + std::to_string(i), robot,
        foot_names[i],
        Eigen::Vector3d::UnitZ(),  // contact normal
        0.6,                       // friction coefficient mu
        1.0, 400.0                 // min/max normal force [N]
    );
    contact->Kp(1000 * Eigen::Vector3d::Ones());
    contact->Kd(100 * Eigen::Vector3d::Ones());
    formulation.addRigidContact(*contact, 1e-5);
}

// 4. Add CoM tracking task (Level 1, high priority)
auto com_task = std::make_shared<tsid::tasks::TaskComEquality>(
    "com_task", robot
);
com_task->Kp(300 * Eigen::Vector3d::Ones());
com_task->Kd(50 * Eigen::Vector3d::Ones());
formulation.addMotionTask(*com_task, 1.0, 1);

// 5. Add joint regularization task (Level 2, low priority)
auto posture_task = std::make_shared<tsid::tasks::TaskJointPosture>(
    "posture_task", robot
);
posture_task->Kp(10 * Eigen::VectorXd::Ones(robot.nv() - 6));
posture_task->Kd(2 * Eigen::VectorXd::Ones(robot.nv() - 6));
formulation.addMotionTask(*posture_task, 0.1, 2);

// 6. Create Solver
tsid::solvers::SolverProxQP solver("solver");

// 7. Main loop (1 kHz)
while (running) {
    // Update reference trajectories
    com_task->setReference(com_ref_trajectory.computeNext());
    posture_task->setReference(q_nominal);

    // Assemble QP and solve
    const auto& qp_data = formulation.computeProblemData(t, q, v);
    const auto& sol = solver.solve(qp_data);

    if (sol.status != tsid::solvers::HQP_STATUS_OPTIMAL) {
        std::cerr << "WBC solver failed!" << std::endl;
        // Fallback: use last valid torque or zero torque
    }

    // Extract joint torques
    Eigen::VectorXd tau = formulation.getActuatorForces(sol);

    // Send to robot
    robot.sendTorque(tau);
    t += dt;
}
```

> ⚠️ **编程陷阱**:`addMotionTask()` 的第三个参数是**优先级 level**,不是"权重比例因子"。Level 0 是最高优先级(硬约束级别),Level 1 和 Level 2 是软约束的不同优先级。如果你把所有任务都放在 Level 1,那退化为加权 QP,HQP 的优先级保证消失。

**练习 53.4.A** ⭐⭐:编译 TSID + example-robot-data,运行 Python 示例 `exercizes/ex_1_forward_dynamics.py`。修改 PD 增益(Kp, Kd),观察末端追踪精度和振荡行为的变化。记录至少 3 组增益的效果。

**练习 53.4.B** ⭐⭐⭐:继承 `TaskBase` 实现一个自定义 Task——"末端竖直任务"(强制末端 z 轴与世界 z 轴对齐)。提示:误差 $e = R_{ee}[:,2] - [0,0,1]^T$,Jacobian 需要用 Pinocchio 的角速度 Jacobian。添加到已有 TSID 问题中,观察行为变化。

---

### 53.5 `InverseDynamicsFormulationAccForce`——TSID 的核心公式 ⭐⭐⭐

#### 53.5.1 决策变量的选择:为什么不用 tau?

TSID 的决策变量是 $(\ddot{q}, \lambda)$ 而**不是** $(\tau, \ddot{q}, \lambda)$。这是一个精妙的工程选择。

**推导**:从动力学方程 (53.1),一旦 $(\ddot{q}, \lambda)$ 确定,$\tau$ 可以**直接反推**:

$$\tau = S (M \ddot{q} + h - J_c^T \lambda) \tag{53.14}$$

其中 $S = [0_{n_a \times 6} \;\; I_{n_a}]$ 选出驱动关节的行。

**为什么这是合法的?** 因为 $S$ 选出的是方程 (53.1) 的后 $n_a$ 行,这些行的左边就是 $M_j \ddot{q} + h_j$,右边就是 $\tau + J_{c,j}^T \lambda$。一旦 $\ddot{q}$ 和 $\lambda$ 确定,左右两边都确定了,$\tau$ 就是**唯一确定的**。

**维度节省**:

| 方案 | 决策变量维度 | Go2 数值 |
|------|------------|---------|
| $(\tau, \ddot{q}, \lambda)$ | $n_a + n_v + 3n_c$ | $12+18+12=42$ |
| $(\ddot{q}, \lambda)$ | $n_v + 3n_c$ | $18+12=30$ |
| **节省** | $n_a$ | **12 维 (28.6%)** |

QP 求解时间大致与 $n^{2}$ 到 $n^{3}$ 成正比(取决于求解算法和稀疏性),30 维 vs 42 维可以**快 30-50%**——在 1 kHz 的约束下,这就是"能不能跑起来"的差别。

#### 53.5.2 完整 QP 矩阵构建

**等式约束——欠驱动基座的动力学方程**:

消去 $\tau$ 后,只剩基座的 6 行方程(因为基座没有驱动器):

$$M_{b} \ddot{q} + h_{b} = J_{c,b}^T \lambda \tag{53.15}$$

改写为标准形式:

$$\underbrace{[M_b \;\; -J_{c,b}^T]}_{A_{eq,1}} \begin{bmatrix} \ddot{q} \\ \lambda \end{bmatrix} = \underbrace{-h_b}_{b_{eq,1}} \tag{53.16}$$

维度:$6 \times (n_v + 3n_c) = 6 \times 30$。

**等式约束——接触不滑**:

$$\underbrace{[J_c \;\; 0]}_{A_{eq,2}} \begin{bmatrix} \ddot{q} \\ \lambda \end{bmatrix} = \underbrace{-\dot{J}_c \dot{q}}_{b_{eq,2}} \tag{53.17}$$

维度:$3n_c \times (n_v + 3n_c) = 12 \times 30$。

**不等式约束——摩擦锥**:

$$\underbrace{[0 \;\; D_{stack}]}_{A_{ineq,1}} \begin{bmatrix} \ddot{q} \\ \lambda \end{bmatrix} \le 0 \tag{53.18}$$

维度:$5n_c \times 30 = 20 \times 30$。

**不等式约束——扭矩限制**(通过 (53.14) 反推):

$$-\tau_{max} \le S(M\ddot{q} + h - J_c^T \lambda) \le \tau_{max}$$

展开为两组不等式:

$$SM \ddot{q} - SJ_c^T \lambda \le \tau_{max} - Sh \tag{53.19}$$

$$-SM \ddot{q} + SJ_c^T \lambda \le \tau_{max} + Sh \tag{53.20}$$

维度:$2n_a \times 30 = 24 \times 30$。

**代价函数——任务追踪**:

对每个 Motion Task $k$(权重 $w_k$,Jacobian $J_k$,期望加速度 $\ddot{x}_k^{des}$):

$$\text{err}_k = J_k \ddot{q} - (\ddot{x}_k^{des} - \dot{J}_k \dot{q})$$

堆叠所有任务后的 Hessian 和梯度:

$$H = \sum_k w_k \begin{bmatrix} J_k^T J_k & 0 \\ 0 & 0 \end{bmatrix}, \quad g = -\sum_k w_k \begin{bmatrix} J_k^T \\ 0 \end{bmatrix} (\ddot{x}_k^{des} - \dot{J}_k \dot{q}) \tag{53.21}$$

如果还有 Force Task(如接触力正则化 $\|\lambda - \lambda^{ref}\|^2$),则 $H$ 的右下角也会有对应项。

**完整 QP 汇总(Go2 四足,4 足着地)**:

$$\min_{\ddot{q}, \lambda \in \mathbb{R}^{30}} \frac{1}{2} x^T H x + g^T x$$

| 约束类型 | 矩阵维度 | 行数 |
|---------|---------|------|
| 基座动力学(等式) | $6 \times 30$ | 6 |
| 接触不滑(等式) | $12 \times 30$ | 12 |
| 摩擦锥(不等式) | $20 \times 30$ | 20 |
| 扭矩限制(不等式) | $24 \times 30$ | 24 |
| **总计** | | **18 等式 + 44 不等式** |

> 💡 **与 53.2 的对比**:53.2 节用 $(\tau, \ddot{q}, \lambda)$ 时是 42 维 + 30 等式 + 44 不等式。消去 $\tau$ 后变为 30 维 + 18 等式 + 44 不等式。等式约束也减少了(不需要显式写驱动关节的动力学),QP 更紧凑。**这就是"好的数学建模直接影响工程性能"的范例**。

#### 53.5.3 QP 求解器选择

基于近期的 QP 求解器基准测试(arXiv:2510.21773, 2025),对 WBC 这类密集小规模 QP 的推荐:

| 求解器 | 算法 | WBC 典型求解时间 | 鲁棒性 | 推荐场景 |
|--------|------|----------------|--------|---------|
| **eiquadprog** | Active-set | <0.1 ms | 良好 | 默认首选,最快 |
| **qpmad** | Active-set | <0.1 ms | 良好 | 低依赖,嵌入式 |
| **ProxQP** | Augmented Lagrangian | 0.1-0.3 ms | 优秀 | 接触丰富/鲁棒性优先 |
| **qpOASES** | Active-set | 0.1-0.3 ms | 良好 | 小问题,经典选择 |
| **OSQP** | ADMM | 0.2-0.5 ms | 中等 | 稀疏 MPC,不推荐 WBC |

> 💡 **概念澄清**:OSQP 虽然流行,但它是**一阶方法**(ADMM),对 WBC 这种小密集 QP 不如**二阶方法**(active-set)快。OSQP 更适合 MPC 中的大规模稀疏 QP。Ch50 有详细的算法对比。

**练习 53.5.A** ⭐⭐⭐:用 Eigen 手动构建上述 QP 矩阵(使用 Pinocchio 计算 M, h, J_c),然后用 ProxQP 和 eiquadprog 分别求解。对比求解时间和精度差异。

**练习 53.5.B** ⭐⭐:在 TSID 中切换不同的 Solver 后端(`SolverHQuadProgFast` vs `SolverProxQP`),跑同一个追踪任务,记录求解时间和追踪精度。你能复现基准测试中的性能排名吗?

---

### 53.6 `EIGEN_RUNTIME_NO_MALLOC`——实时控制的硬约束 ⭐⭐

前两节解剖了 TSID 的架构设计和 QP 公式构造。然而,即使数学公式完全正确、软件架构清晰优雅,如果 QP 求解过程中触发了一次堆内存分配,整个 1kHz 控制循环就可能超时。内存管理是 WBC 从"能跑"到"能上机器人"之间最容易被忽视的鸿沟。

#### 53.6.1 为什么 malloc 是实时控制的"死刑"

**背景**:WBC 在实时线程(SCHED_FIFO)中运行,控制周期是 1 ms。假如没有 `EIGEN_RUNTIME_NO_MALLOC` 保护,一个看似无害的 Eigen 表达式(如 `MatrixXd C = A * B` 而非预分配的 `C.noalias() = A * B`)就会在 QP 组装的热路径中触发 `malloc`。在实验室环境下可能 999 次都没事,但第 1000 次恰好碰到内核页面回收,控制周期从 0.8 ms 跳到 5 ms——机器人在那一瞬间失去平衡控制,轻则踉跄、重则倒地。一次 `malloc` 调用可能带来以下延迟:

| 情况 | 耗时 | 原因 |
|------|------|------|
| 最好情况(free list 命中) | 0.1-1 us | 只操作用户态链表 |
| 一般情况(sbrk) | 1-10 us | 系统调用扩展堆 |
| 最坏情况(mmap) | 10-1000 us | 内核分配虚拟内存页 |
| 极端情况(页面回收) | 1-10 ms | 内核 OOM killer 或 swap |

**1 ms 的控制周期,10-1000 us 的 malloc 延迟,最坏情况直接超时**。更糟的是,malloc 的延迟**不可预测**——这就是实时系统最忌讳的"非确定性延迟"。

#### 53.6.2 Linux 实时调度基础

为了理解 WBC 的运行环境,需要了解 Linux 的实时调度:

```
┌─────────────────────────────────────────────┐
│ SCHED_FIFO / SCHED_RR (实时策略)            │
│ 优先级 1-99,抢占式调度                      │
│ WBC 线程: SCHED_FIFO, priority=49           │
│ -> 不被普通进程抢占                         │
│ -> 但 malloc 的内核路径可能阻塞!            │
├─────────────────────────────────────────────┤
│ SCHED_OTHER (普通策略,CFS 调度器)           │
│ 优先级 0,时间片轮转                         │
│ 非实时进程运行在这里                         │
└─────────────────────────────────────────────┘
```

**SCHED_FIFO 的关键规则**:
- 实时线程**不会被普通进程抢占**
- 同优先级的实时线程按 FIFO 顺序运行(先到先服务)
- 更高优先级的实时线程可以抢占低优先级的

**但 SCHED_FIFO 不能保护你免受 malloc 的伤害**:当你调用 `malloc`,内核需要操作页表——这发生在内核态,实时线程必须等待。更糟的是,如果触发了**优先级反转**(priority inversion):

```
时间 ->
┌──────────────────────────────────────────┐
│ 高优先级(WBC):   运行...调 malloc...等   │
│ 低优先级(日志):   持有 malloc 内部锁     │
│ 中优先级(网络):   抢占了低优先级         │
│                                          │
│ 结果:WBC 等待低优先级释放锁             │
│       但低优先级被中优先级抢占           │
│       -> WBC 间接被中优先级阻塞!        │
└──────────────────────────────────────────┘
```

PREEMPT_RT 内核通过优先级继承(priority inheritance)可以缓解这个问题,但**最根本的解决方案是:在实时路径上不调用 malloc**。

#### 53.6.3 TSID 的内存安全机制

TSID 的 `CMakeLists.txt` 默认开启两个 Eigen 宏:

```cmake
option(EIGEN_RUNTIME_NO_MALLOC
       "If ON, Eigen will assert on runtime malloc" ON)
option(EIGEN_NO_AUTOMATIC_RESIZING
       "If ON, Eigen will not auto-resize matrices" ON)
```

**`EIGEN_RUNTIME_NO_MALLOC`** 的工作机制:

```cpp
// Eigen/src/Core/util/Memory.h (simplified)
#ifdef EIGEN_RUNTIME_NO_MALLOC
static bool is_malloc_allowed_flag = true;

inline void set_is_malloc_allowed(bool allowed) {
    is_malloc_allowed_flag = allowed;
}

inline void check_that_malloc_is_allowed() {
    eigen_assert(is_malloc_allowed_flag &&
        "Heap allocation in real-time code path!");
}
#endif
```

使用方式:

```cpp
// During initialization (before real-time loop)
Eigen::internal::set_is_malloc_allowed(true);
// ... allocate all matrices here ...

// Enter real-time loop
Eigen::internal::set_is_malloc_allowed(false);
while (running) {
    // Any Eigen allocation here will trigger assert failure
    wbc.solve(q, v);  // must be malloc-free
}
```

#### 53.6.4 Eigen 何时会偷偷 malloc?

这是最容易犯错的地方。以下操作在 `EIGEN_RUNTIME_NO_MALLOC` 下会触发 assert:

| 操作 | 原因 | 修复方法 |
|------|------|---------|
| `MatrixXd A; A = B * C;` | 矩阵乘法创建临时变量 | `A.noalias() = B * C;` |
| `MatrixXd A(n, m);` 在循环内 | 每次构造都分配 | 移到循环外作为成员变量 |
| `M.resize(10, 10)` | 改变尺寸 = 重分配 | 预分配正确尺寸 |
| `A * B * C` | 链式乘法的中间结果 | 分步计算或 `.noalias()` |
| `A.inverse()` | 需要临时矩阵 | 用 `A.llt().solve(I)` |
| `svd.compute(A)` | SVD 需要工作空间 | 预分配 SVD 对象 |
| `v.conservativeResize(n+1)` | 保守调大 = malloc + copy | 预分配最大尺寸 |

**正确示范**:

```cpp
class WBC {
  // Pre-allocate ALL matrices as member variables
  Eigen::MatrixXd A_;       // constraint matrix
  Eigen::VectorXd b_;       // constraint vector
  Eigen::VectorXd x_;       // solution
  Eigen::MatrixXd temp_;    // temporary for A*B

 public:
  WBC(int n, int m) : A_(n, m), b_(n), x_(m), temp_(n, m) {}

  void solve(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    // GOOD: use pre-allocated matrices, noalias avoids temporaries
    temp_.noalias() = M_ * J_.transpose();
    A_.block(0, 0, 6, nv_) = M_.topRows(6);

    // BAD examples (would crash with EIGEN_RUNTIME_NO_MALLOC):
    // Eigen::MatrixXd temp = M_ * J_.transpose();  // malloc!
    // A_.resize(new_n, new_m);                      // malloc!
  }
};
```

> ⚠️ **编程陷阱**:`noalias()` 并非万能。当**目标矩阵出现在表达式右侧**时,使用 `noalias()` 会产生**错误结果**:
> ```cpp
> // WRONG: C appears on both sides
> C.noalias() += A * C;  // overwrites C mid-computation!
>
> // CORRECT: use pre-allocated temporary
> temp.noalias() = A * C;
> C += temp;
> ```

#### 53.6.5 pmr 分配器替代方案

如果你确实需要在实时路径上分配内存(例如接触点数量动态变化),可以使用 Ch35 的 `std::pmr` 方案:

```cpp
#include <memory_resource>

class RealtimeWBC {
    // Stack-based memory pool: 64 KB on stack, no malloc
    alignas(64) std::byte buffer_[65536];
    std::pmr::monotonic_buffer_resource pool_{
        buffer_, sizeof(buffer_)};

    // Containers use pool instead of malloc
    std::pmr::vector<Eigen::Vector3d> contact_forces_{&pool_};

 public:
    void update(int num_contacts) {
        pool_.release();  // O(1) "free all" - no malloc
        contact_forces_.reserve(num_contacts);
        // ... use contact_forces_ ...
    }
};
```

#### 53.6.6 调试 malloc 违规

当你在 TSID 中添加新 Task 后遇到 assert 失败时,如何定位?

```bash
# Method 1: GDB backtrace
gdb ./wbc_node
(gdb) run
# ... assert failure ...
(gdb) bt
# Shows the exact line that triggered malloc

# Method 2: Valgrind + massif (offline profiling)
valgrind --tool=massif ./wbc_node
ms_print massif.out.12345

# Method 3: LD_PRELOAD malloc interception
LD_PRELOAD=./libmalloc_tracker.so ./wbc_node
```

> 💡 **实践技巧**:在开发阶段,用 `EIGEN_RUNTIME_NO_MALLOC` + Debug 模式运行所有单元测试。所有 malloc 违规都会在测试中暴露,而不是在机器人上暴露(那时已经来不及了)。

**练习 53.6.A** ⭐⭐:写一个小程序,开启 `EIGEN_RUNTIME_NO_MALLOC`,然后故意触发 5 种不同的 Eigen 堆分配。逐一修复,使程序在 `set_is_malloc_allowed(false)` 下不触发 assert。

**练习 53.6.B** ⭐⭐⭐:在你的 WBC 实现中加入 `EIGEN_RUNTIME_NO_MALLOC` 保护。用 `std::chrono` 测量加保护前后的求解时间分布(均值、最大值、p99)。预分配是否消除了时间分布的长尾?

---

### 53.7 legged_control 的轻量 WBC——相对 TSID 的简化 ⭐⭐

TSID 提供了一个功能完备、可扩展的 WBC 框架,但它的通用性也带来了代码量和接口复杂度。在实际四足机器人开发中,许多团队选择了一条更务实的路径:只保留最核心的 QP 组装逻辑,砍掉不需要的抽象层。legged_control 的 WBC 就是这种工程取舍的典型代表。

#### 53.7.1 legged_control 概述

**legged_control** 由 Qiayuan Liao 开发(GitHub: `qiayuanl/legged_control`),是基于 OCS2 + ros-control 的四足机器人控制框架。它的 WBC 模块(`legged_wbc/`)是一个**极简但完整的 WBC 实现**,非常适合入门学习。

**代码规模对比**:

| 项目 | 核心 WBC 代码量 | 支持的机器人 | 求解器 |
|------|---------------|-------------|--------|
| TSID | ~5000 行(多文件) | 任意(通用框架) | 多后端 |
| legged_wbc | ~600 行(3 文件) | 四足(专用) | qpOASES |

#### 53.7.2 核心类结构

```
legged_wbc/
├── include/legged_wbc/
│   ├── WbcBase.h          // Base class: dynamics computation
│   ├── HierarchicalWbc.h  // 2-level HQP implementation
│   └── WeightedWbc.h      // Weighted QP alternative
└── src/
    ├── WbcBase.cpp         // ~200 lines: task formulation
    ├── HierarchicalWbc.cpp // ~150 lines: cascaded QP
    └── WeightedWbc.cpp     // ~100 lines: single QP
```

#### 53.7.3 WbcBase 的 6 个任务公式

`WbcBase` 实现了 6 个任务公式化方法,每个返回一个 `(A, b)` 对(等式约束)或 `(D, f)` 对(不等式约束):

| 方法 | 功能 | 约束类型 |
|------|------|---------|
| `formulateFloatingBaseEomTask()` | 浮动基座动力学方程 | 等式 |
| `formulateNoContactMotionTask()` | 接触足零加速度 | 等式 |
| `formulateFrictionConeTask()` | 摩擦锥约束 | 不等式 |
| `formulateBaseAccelTask()` | 基座加速度追踪 | 等式(软) |
| `formulateSwingLegTask()` | 摆动腿足端追踪 | 等式(软) |
| `formulateTorqueLimitsTask()` | 关节扭矩限制 | 不等式 |

**决策变量结构**(与 TSID 相同的消元思路):

```
x = [ddq (18), lambda (12), tau (12)]^T    // 42 维
```

注意 legged_wbc 没有消去 $\tau$——它保留了全部三组决策变量。这使代码更直观(直接读出 $\tau$),但 QP 维度更大(42 vs 30)。

#### 53.7.4 HierarchicalWbc 的两层结构

`HierarchicalWbc` 实现了一个简单的两层 HQP:

```
Level 1 (硬约束 -> QP 1):
  等式约束:
    - formulateFloatingBaseEomTask()   // 动力学
    - formulateNoContactMotionTask()   // 接触不滑
  不等式约束:
    - formulateFrictionConeTask()      // 摩擦锥
    - formulateTorqueLimitsTask()      // 扭矩限制
  代价:min ||x||^2 (正则化)

Level 2 (软目标 -> QP 2, 在 Level 1 的零空间内):
  代价:
    - formulateBaseAccelTask()         // 基座追踪
    - formulateSwingLegTask()          // 足端追踪
  约束:不破坏 Level 1
```

**核心代码片段**(简化):

```cpp
// HierarchicalWbc.cpp (simplified)
Eigen::VectorXd HierarchicalWbc::solve(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v) {

  // Level 1: dynamics + contact constraints
  auto [A1, b1] = formulateFloatingBaseEomTask();
  auto [A2, b2] = formulateNoContactMotionTask();

  Eigen::MatrixXd A_eq;      // stack A1, A2
  Eigen::VectorXd b_eq;      // stack b1, b2
  stackTasks(A1, b1, A2, b2, A_eq, b_eq);

  auto [D, f] = formulateFrictionConeTask();
  auto [D2, f2] = formulateTorqueLimitsTask();
  stackTasks(D, f, D2, f2, D_ineq, f_ineq);

  // Solve QP 1: min ||x||^2 s.t. A_eq*x=b_eq, D*x<=f
  solveQP(H_identity, g_zero, A_eq, b_eq, D_ineq, f_ineq, x1);

  // Compute null-space of Level 1
  // N1 = I - A_eq^+ * A_eq
  Eigen::MatrixXd N1 = computeNullSpace(A_eq);

  // Level 2: tracking tasks in null-space of Level 1
  auto [A3, b3] = formulateBaseAccelTask();
  auto [A4, b4] = formulateSwingLegTask();
  stackTasks(A3, b3, A4, b4, A_track, b_track);

  // Project into null-space: A_track_proj = A_track * N1
  Eigen::MatrixXd A_proj = A_track * N1;
  Eigen::VectorXd b_proj = b_track - A_track * x1;

  // Solve QP 2: min ||A_proj*delta - b_proj||^2
  solveQP(A_proj.transpose() * A_proj,
          -A_proj.transpose() * b_proj,
          ..., delta);

  // Final solution
  return x1 + N1 * delta;
}
```

#### 53.7.5 TSID vs legged_wbc 对比

| 维度 | TSID | legged_wbc |
|------|------|------------|
| **代码量** | ~5000 行 | ~600 行 |
| **优先级层数** | 任意 | 2 层 |
| **Task 类型** | 10+ 种(SE3, CoM, AM, ...) | 6 种(硬编码) |
| **约束抽象** | 通用(Equality/Inequality/Bound) | 硬编码 |
| **求解器** | 多后端(ProxQP, eiquadprog, ...) | 只用 qpOASES |
| **决策变量** | $(\ddot{q}, \lambda)$ 消元 | $(\ddot{q}, \lambda, \tau)$ 全保留 |
| **机器人类型** | 任意 | 四足专用 |
| **实时安全** | `EIGEN_RUNTIME_NO_MALLOC` | 无 |
| **学习曲线** | 1-2 周 | 半天-1 天 |
| **适合场景** | 产品级/研究级 | 教学/快速原型 |

> 🧠 **思维陷阱**:不要因为 legged_wbc 代码少就认为它"不好"。它的简化是**有目的的**——去掉了通用性,换取了可读性和开发速度。如果你只做四足控制,legged_wbc 完全够用;如果你要做人形或多平台产品,则需要 TSID 级别的框架。

**推荐学习路径**:

```
第 1 天:读 legged_wbc (600 行,建立直觉)
  -> 理解 WBC 的基本结构
  -> 理解"任务公式化"的套路

第 2-3 天:读 TSID 的 TaskSE3Equality (核心 Task)
  -> 对比 legged_wbc 的 formulateSwingLegTask()
  -> 理解"通用框架 vs 专用实现"的设计差异

第 4-5 天:读 TSID 的 InverseDynamicsFormulationAccForce
  -> 理解 QP 组装流程
  -> 理解决策变量消元技巧

第 6-7 天:自己实现一个极简 WBC
  -> 融合两者的优点
  -> 在 MuJoCo/Gazebo 中验证
```

**练习 53.7.A** ⭐⭐:精读 `legged_wbc/src/WbcBase.cpp` 的 `formulateFloatingBaseEomTask()`。用文字描述:它如何用 Pinocchio 计算 $M$、$h$、$J_c$?动力学方程如何转化为 $Ax = b$ 的形式?

**练习 53.7.B** ⭐⭐⭐:精读 `HierarchicalWbc.cpp`,画出它的两层 QP 求解流程图。标注每步的矩阵维度(以 Go2 为例)。

---

### 53.8 MPC + WBC 集成——腿足控制的完整图景 ⭐⭐⭐

> **本质洞察**:MPC 和 WBC 的分工不是"粗糙规划 + 精细执行"那么简单。MPC 用简化模型在长时域上搜索全局一致的运动策略(解决"做什么"),WBC 用全身模型在当前时刻满足所有物理约束(解决"怎么做")。两者的模型不一致恰恰是设计意图——简化模型让 MPC 跑得快,全身模型让 WBC 跟得准。这种"有意的模型不一致 + 高频修正"是分层控制鲁棒性的来源,而非缺陷。

#### 53.8.1 数据流

```
┌──────────────────┐     triple buffer / atomic ptr     ┌──────────────────┐
│   MPC Thread     │ ─────────────────────────────────> │   WBC Thread     │
│                  │                                     │                  │
│ freq: 50 Hz      │  data: {com_traj, foot_traj,       │ freq: 1000 Hz    │
│ model: SRBD/     │         contact_forces,             │ model: full-body │
│   Centroidal     │         contact_schedule,           │   dynamics       │
│ solver: iLQR/    │         timestamp}                  │ solver: QP       │
│   DDP            │                                     │                  │
│ time: 10-20 ms   │                                     │ time: 0.3-0.8 ms │
└──────────────────┘                                     └──────────────────┘
```

**关键数据接口**:

| 数据 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `com_trajectory` | 分段多项式 | $3 \times N_{steps}$ | 质心位置轨迹 |
| `foot_trajectory` | 分段多项式 | $3 \times 4 \times N_{steps}$ | 四足位置轨迹 |
| `contact_forces` | 离散序列 | $12 \times N_{steps}$ | 接触力参考 |
| `contact_schedule` | 布尔序列 | $4 \times N_{steps}$ | 每足是否着地 |
| `timestamp` | double | 1 | MPC 解对应的起始时间 |

#### 53.8.2 Triple Buffer 机制

**为什么不用 mutex?** 因为 mutex 会导致 WBC 线程阻塞(等待 MPC 线程释放锁),这在 1 kHz 控制中不可接受。

**Triple buffer 的工作原理**:

```
┌─────────────────────────────────────────────────────┐
│ Buffer A: [MPC 正在写入新解]                         │
│ Buffer B: [WBC 正在读取当前解]                       │
│ Buffer C: [空闲,等待交换]                           │
│                                                     │
│ MPC 写完 -> atomic swap(A, C) -> MPC 开始写 C       │
│ WBC 需要新解 -> atomic swap(B, C) -> WBC 读 C       │
│                                                     │
│ 关键:所有 swap 操作都是原子的,无锁                 │
│ 代价:WBC 可能读到"前一次"而非"最新"的 MPC 解      │
│       (延迟最多一个 MPC 周期)                       │
└─────────────────────────────────────────────────────┘
```

```cpp
// Simplified triple buffer implementation
template <typename T>
class TripleBuffer {
    std::array<T, 3> buffers_;
    std::atomic<int> write_idx_{0};  // MPC writes here
    std::atomic<int> read_idx_{1};   // WBC reads here
    std::atomic<int> free_idx_{2};   // ready for swap

 public:
    // MPC thread calls this after finishing a solve
    T& getWriteBuffer() { return buffers_[write_idx_.load()]; }

    void publishWrite() {
        int old_free = free_idx_.exchange(write_idx_.load());
        write_idx_.store(old_free);
    }

    // WBC thread calls this to get latest MPC solution
    const T& getReadBuffer() {
        int old_free = free_idx_.exchange(read_idx_.load());
        read_idx_.store(old_free);
        return buffers_[read_idx_.load()];
    }
};
```

#### 53.8.3 时间插值算法

MPC 输出的是离散时间步的轨迹(例如 20 个点,每点间隔 50 ms)。WBC 运行在 1 kHz,需要在**任意时刻**查询参考值。

**线性插值**(最简单):

$$x_{ref}(t) = x_k + \frac{t - t_k}{t_{k+1} - t_k} (x_{k+1} - x_k), \quad t_k \le t < t_{k+1}$$

**三次样条插值**(更平滑,用于足端轨迹):

$$x_{ref}(t) = a_0 + a_1(t - t_k) + a_2(t - t_k)^2 + a_3(t - t_k)^3$$

系数由位置和速度的边界条件确定(Hermite 插值）。

> ⚠️ **编程陷阱**:时间戳同步是 MPC+WBC 集成中最容易出错的地方。MPC 解的时间戳是"MPC 开始求解时的时间",但 WBC 使用时已经过了 10-20 ms。如果 WBC 不补偿这个延迟,追踪会有滞后。正确做法:WBC 查询 $t_{current}$ 对应的参考值,而不是 $t_{MPC\_start}$。

#### 53.8.4 失败模式与恢复

| 失败模式 | 检测方法 | 恢复策略 |
|---------|---------|---------|
| MPC 求解超时 | 超过预设时间阈值 | WBC 用上一次 MPC 解 + 外推 |
| MPC 求解失败 | 求解器返回 INFEASIBLE | 切换到安全站立姿态 |
| WBC 求解失败 | QP 返回 INFEASIBLE | 降级:放松摩擦锥约束或切到 PD 控制 |
| 通信丢包 | 时间戳不更新 | 用上一次有效解,计数告警 |

```cpp
// Robust MPC+WBC integration (pseudocode)
void wbc_loop() {
    while (running) {
        // Try to get latest MPC solution
        auto& mpc_sol = triple_buffer.getReadBuffer();

        if (mpc_sol.timestamp < t_current - mpc_timeout) {
            // MPC solution is stale
            stale_count++;
            if (stale_count > max_stale) {
                // Switch to safe standing mode
                tau = pd_controller(q, v, q_safe, v_zero);
                continue;
            }
            // Use stale solution with time extrapolation
        }

        // Normal WBC solve
        auto status = wbc.solve(q, v, mpc_sol);
        if (status != QP_OPTIMAL) {
            // WBC failed: relax friction cone or use last tau
            tau = last_valid_tau;
            continue;
        }

        tau = wbc.getTorques();
        last_valid_tau = tau;
        stale_count = 0;
    }
}
```

> 💡 **工程洞察**:Ch55 OCS2 框架的双线程架构(`MPC_MRT_Interface`)就是上述 MPC+WBC 集成的工业级实现。它使用 `std::condition_variable` + triple buffer 实现线程间通信,并包含完整的失败恢复逻辑。

**练习 53.8.A** ⭐⭐:实现一个 triple buffer 模板类,并用两个线程测试:生产者每 20 ms 写入一个递增计数器,消费者每 1 ms 读取。验证消费者是否从不阻塞,且读到的值单调递增(允许重复但不允许跳回)。

**练习 53.8.B** ⭐⭐⭐:在 MPC+WBC 集成中,如果 MPC 求解时间不确定(有时 10 ms,有时 50 ms),WBC 应如何处理?设计一个自适应插值方案:当 MPC 解新鲜时用线性插值,当 MPC 解陈旧时用外推 + 衰减。

---

### 53.9 WBC 调参实用指南 ⭐⭐

#### 53.9.1 PD 增益调参

WBC 中每个 Task 都有 PD 增益 $(K_p, K_d)$,它们决定了任务追踪的**刚度**和**阻尼**。

**基本原则**:

$$\ddot{x}_{des} = \ddot{x}_{ref} + K_d(\dot{x}_{ref} - \dot{x}) + K_p(x_{ref} - x)$$

这等效于一个二阶系统:

$$\ddot{e} + K_d \dot{e} + K_p e = 0$$

特征方程:$s^2 + K_d s + K_p = 0$

**临界阻尼条件**:$K_d = 2\sqrt{K_p}$（不振荡、最快收敛）

| 场景 | 推荐 $K_p$ | 推荐 $K_d$ | 理由 |
|------|-----------|-----------|------|
| 质心追踪 | 200-500 | 30-50 | 中等刚度,防止过冲导致失稳 |
| 摆动腿足端 | 300-1000 | 50-100 | 高刚度,精确落脚 |
| 关节正则化 | 5-20 | 1-5 | 低刚度,不干扰主要任务 |
| 基座姿态 | 100-300 | 20-50 | 中等,保持水平但允许倾斜 |

> ⚠️ **编程陷阱**:$K_p$ 设太大(如 $>2000$)会导致 QP 的 Hessian 矩阵条件数恶化,求解器可能不收敛或给出振荡解。如果你发现机器人"抖动",第一件事是降低 $K_p$。

#### 53.9.2 权重与优先级配置

**权重的选择原则**:

1. **量纲归一化**:不同任务的误差量纲不同(位置 m、角度 rad、力 N）。先将各任务误差归一化到相近量级,再用权重调节相对重要性。
2. **对角阵权重**:对 SE3 任务,通常位置分量和旋转分量的权重不同:

```cpp
Eigen::VectorXd weight(6);
weight << 1.0, 1.0, 1.0,    // position (x, y, z)
          0.5, 0.5, 0.5;    // orientation (roll, pitch, yaw)
task.setWeight(weight);
```

3. **渐进调参法**:

```
Step 1: 只开 Level 1 硬约束,确认机器人能站稳
Step 2: 加 CoM 追踪(Level 2),调 Kp/Kd 直到平衡良好
Step 3: 加足端追踪(Level 2),确认步态正常
Step 4: 加关节正则化(Level 3),防止关节漂移
Step 5: 微调权重比,优化能量效率
```

#### 53.9.3 常见故障诊断

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| 机器人抖动 | $K_p$ 过大 / 控制延迟 | 降低 $K_p$,检查通信延迟 |
| 机器人"软" | $K_p$ 过小 / 权重不当 | 提高追踪任务权重 |
| QP 不可行 | 约束冲突(如摩擦锥太紧) | 放松摩擦系数或检查接触模式 |
| 关节扭矩饱和 | $\tau_{max}$ 设置不当 | 检查电机参数,降低追踪要求 |
| 足底打滑 | 摩擦系数过高 / 力分配不均 | 降低 $\mu$,加力正则化 |

**练习 53.9.A** ⭐⭐:在四足仿真中,从"全部默认参数"开始,按上述渐进法调参。记录每步的调参过程和效果。

---

### 53.10 WBC 在不同机器人形态的适配 ⭐⭐

#### 53.10.1 四足 vs 双足 vs 人形

| 维度 | 四足 | 双足 | 人形(上肢+腿） |
|------|------|------|--------------|
| 自由度 | 12-18 | 10-12 | 20-40+ |
| 接触模型 | 4 点接触 | 2 面接触 | 2 面接触 + 手 |
| 稳定性 | 高(4 点支撑) | 低(2 点支撑) | 低 + 操作需求 |
| 关键 Task | CoM + 足端 | CoM + ZMP + 角动量 | 全部 + 操作 |
| 计算负担 | 中(42 维 QP) | 中(50 维 QP) | 高(80+ 维 QP) |

**双足特有的挑战**:

- **角动量管理**:双足只有 2 个接触点,稳定性裕度小。通过 `TaskAMEquality` 控制角动量变化率,防止侧翻。
- **ZMP 约束**:ZMP(Zero Moment Point)必须在支撑多边形内——这是一个依赖接触力的非线性约束,需要用 `Contact6d` 处理。

**人形特有的挑战**:

- **操作与平衡的冲突**:上肢操作物体会改变质心位置和角动量,WBC 必须在操作和平衡之间分配优先级。
- **自碰撞避免**:关节数多,需要额外的 Task 或 Constraint 防止身体各部分碰撞。

#### 53.10.2 bipedal_locomotion_framework 的 TSID 变体

IIT 的 `robotology/bipedal-locomotion-framework` (BLF) 为 iCub 人形机器人实现了一个 TSID 变体。与 stack-of-tasks/TSID 的主要区别:

| 维度 | TSID (stack-of-tasks) | BLF QPTSID |
|------|----------------------|------------|
| 动力学库 | Pinocchio | iDynTree (转向 Pinocchio) |
| 接触模型 | ContactPoint / Contact6d | ContactWrench |
| QP 求解器 | 多后端 | OSQP / qpOASES |
| 角动量 Task | TaskAMEquality | CentroidalMomentumTask |
| 目标平台 | 通用 | iCub / ergoCub |

> 💡 **趋势观察**:随着 Pinocchio 3.x 的发展和 TSID 的成熟,越来越多的人形机器人项目开始统一使用 Pinocchio + TSID 生态,而非各自开发 WBC 框架。

---

### 53.11 本章小结

#### 核心概念回顾

| 概念 | 一句话总结 |
|------|-----------|
| WBC | 把 MPC 的宏观参考翻译为关节扭矩的 QP 优化器 |
| HQP | 按优先级分层求解,数学保证低层不破坏高层 |
| 零空间投影 | $N = I - A^+A$,在不影响高优先级任务的子空间中优化 |
| AccForce 消元 | 用 $(\ddot{q}, \lambda)$ 代替 $(\tau, \ddot{q}, \lambda)$,减少 QP 维度 |
| EIGEN_RUNTIME_NO_MALLOC | 实时循环中禁止堆分配,开发期 assert 捕获违规 |
| Triple buffer | 无锁线程间数据交换,MPC/WBC 异步运行 |
| VWBC | 用 MPC 的 Q 函数代替手调 WBC 权重 |

#### 关键公式速查

| 公式 | 编号 | 用途 |
|------|------|------|
| $M\ddot{q} + h = S^T\tau + J_c^T\lambda$ | (53.1) | 全身动力学 |
| $J_c\ddot{q} = -\dot{J}_c\dot{q}$ | (53.5) | 接触不滑约束 |
| $N_1 = I - A_1^+A_1$ | (53.11) | 零空间投影 |
| $\tau = S(M\ddot{q} + h - J_c^T\lambda)$ | (53.14) | 扭矩反推(消元) |

---

### 累积项目:四足控制器——WBC 扭矩生成模块 ⭐⭐⭐

在前几章中,你已经实现了:
- Ch47: Pinocchio 模型加载与动力学计算
- Ch49: 接触 Jacobian 计算
- Ch50: QP 求解器封装
- Ch52: 摩擦锥约束构建

本章的累积项目目标:**实现完整的 WBC 模块,接收 MPC 参考,输出关节扭矩**。

```cpp
// Target interface for cumulative project
class QuadrupedWBC {
 public:
  QuadrupedWBC(const pinocchio::Model& model, const WBCConfig& config);

  // Main solve function: called at 1 kHz
  // Input: current state (q, v), MPC reference
  // Output: joint torques (12-dim for Go2)
  Eigen::VectorXd solve(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& v,
      const MPCReference& mpc_ref,
      const ContactSchedule& contacts);

 private:
  // Task formulation (following legged_wbc pattern)
  Task formulateFloatingBaseEomTask();
  Task formulateContactNoSlipTask();
  Task formulateFrictionConeTask();
  Task formulateComTrackingTask();
  Task formulateSwingLegTask();
  Task formulatePostureTask();
  Task formulateTorqueLimitsTask();

  // QP solver (from Ch50)
  QPSolver solver_;

  // Pre-allocated matrices (EIGEN_RUNTIME_NO_MALLOC safe)
  Eigen::MatrixXd M_, Jc_, H_, A_eq_, A_ineq_;
  Eigen::VectorXd h_, b_eq_, b_ineq_, g_, x_;
};
```

**实现步骤**:

1. **Step 1** (1-2 小时):实现 `formulateFloatingBaseEomTask()` 和 `formulateContactNoSlipTask()`,用 Pinocchio 计算 $M$, $h$, $J_c$。
2. **Step 2** (1-2 小时):实现 `formulateFrictionConeTask()` 和 `formulateTorqueLimitsTask()`(复用 Ch52 代码)。
3. **Step 3** (2-3 小时):实现追踪任务(CoM, 足端, 姿态),组装 QP 并求解。
4. **Step 4** (2-3 小时):在 MuJoCo 仿真中测试四足站立平衡。
5. **Step 5** (进阶):添加 triple buffer,与 MPC 模块集成(Ch55 衔接)。

---

### 项目精读点

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **TSID** | `include/tsid/tasks/task-base.hpp` | `TaskBase` 抽象基类 | 代码阅读 |
| **TSID** | `include/tsid/tasks/task-se3-equality.hpp` + `.cpp` | 最常用的末端 Task 实现 | 代码阅读(核心) |
| **TSID** | `include/tsid/tasks/task-com-equality.hpp` | 质心追踪 Task | 代码阅读 |
| **TSID** | `include/tsid/tasks/task-joint-posture.hpp` | 关节姿态 Task(正则化) | 代码阅读 |
| **TSID** | `formulations/inverse-dynamics-formulation-acc-force.hpp` | **HQP 构建核心** | 代码阅读(核心) |
| **TSID** | `include/tsid/solvers/solver-HQP-eiquadprog-rt.hpp` | 实时 HQP 求解器 | 代码阅读 |
| **TSID** | `include/tsid/solvers/solver-proxqp.hpp` | ProxQP 后端集成 | 代码阅读 |
| **TSID** | `CMakeLists.txt` | `EIGEN_RUNTIME_NO_MALLOC` 配置 | 代码阅读 |
| **TSID** | `exercizes/ex_4_LIPM_to_TSID.py` | **LIPM -> WBC 双足行走** | 代码实战 |
| **legged_control** | `legged_wbc/src/WbcBase.cpp` | 6 个任务公式化方法 | 代码阅读(核心) |
| **legged_control** | `legged_wbc/src/HierarchicalWbc.cpp` | **轻量 HQP 实现** | 代码阅读(对比) |
| **legged_control** | `legged_controllers/src/LeggedController.cpp` | WBC 在控制主循环中的调用 | 代码阅读 |
| **BLF** | `src/TSID/` | iCub 生态的 TSID 变体 | 代码阅读 |

---

### 研究前沿与论文阅读(博士预备视角)

**必读经典**:

1. **Escande A., Mansard N., Wieber P.-B. (2014)** "Hierarchical quadratic programming: Fast online humanoid-robot motion generation" — IJRR, 33(7):1006-1028. HQP 理论的奠基论文。提出了同时处理等式和不等式约束的完整层级求解框架。
2. **Kim D., Jorgensen S. J., Lee J., et al. (2020)** "Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control" — IJRR. WBC 应用于高动态腿足运动的标杆工作。

**近期重要进展(2023-2025)**:

3. **Li H. & Wensing P. M., Cafe-MPC (T-RO, 2024)** "A Cascaded-Fidelity Model Predictive Control Framework with Tuning-Free Whole-Body Control". 提出了**VWBC (Value-function-based WBC)**:用 MPC 反向扫描得到的 action-value 函数 $Q(\delta x, \delta u)$ 作为 WBC 的代价函数,消除了 WBC 层的手动调参需求。$Q$ 函数编码了长时域代价-到-走(cost-to-go)信息,WBC 直接最小化 $Q$ 函数展开,在全身动力学和不等式约束下求解。实验在 MIT Mini Cheetah 上实现了跑步空翻(barrel roll)。

4. **Herzog A., Rotella N., Mason S., et al. (2016)** "Momentum control with hierarchical inverse dynamics on a torque-controlled humanoid" — Autonomous Robots. Momentum-based HQP 变体,通过控制质心动量而非质心位置来实现更鲁棒的人形平衡。

5. **QP 求解器基准(2025)** arXiv:2510.21773 "Real-Time QP Solvers: A Concise Review and Practical Guide Towards Legged Robots". 系统比较了 ProxQP、qpOASES、eiquadprog、qpmad、OSQP 在 WBC 和 MPC 场景下的性能。结论:eiquadprog/qpmad 在 WBC 场景最快(sub-ms),ProxQP 最鲁棒。

**开放研究问题**:

- **学习 WBC**:能否用强化学习自动调 WBC 权重?最新工作通过 RL 策略网络预测最优权重组合,根据地形和步态自适应调整。
- **Contact-Implicit WBC**:当前 WBC 假设接触模式(哪些脚着地)由 MPC 预先决定。能否让接触模式也成为 WBC 的决策变量?这会将 QP 变成混合整数规划(MIQP),计算量大增,但能处理意外接触。
- **GPU WBC**:单个 WBC 的 QP 只有 30-80 维,GPU 加速收益不大。但**并行环境训练**(1000+ 个机器人同时仿真)需要批量 WBC——这是 RL sim-to-real 训练的瓶颈之一。

---

### 延伸阅读

| 资源 | 类型 | 推荐理由 |
|------|------|---------|
| [TSID GitHub](https://github.com/stack-of-tasks/tsid) | 代码 | 官方仓库,含 Python 教程 |
| [legged_control GitHub](https://github.com/qiayuanl/legged_control) | 代码 | 最好的四足 WBC 入门代码 |
| [Pinocchio 文档](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/) | 文档 | WBC 依赖的动力学库 |
| Escande et al. (2014) HQP [PDF](https://gepettoweb.laas.fr/uploads/Publications/2014_escande_ijrr.pdf) | 论文 | HQP 理论原文 |
| Cafe-MPC (2024) [arXiv](https://arxiv.org/abs/2403.03995) | 论文 | VWBC 方法,消除调参 |
| eigenrealtime [GitHub](https://github.com/stulp/eigenrealtime) | 代码 | Eigen 实时安全的最佳实践 |
| QP Solver Review (2025) [arXiv](https://arxiv.org/abs/2510.21773) | 论文 | QP 求解器基准测试 |

---

### 预计学习时间

**1 周(20-25 小时)**:

| 内容 | 时间 | 输出 |
|------|------|------|
| 概念理解(分层优化的数学) | 4-5 小时 | 能手推 HQP 零空间投影 |
| TSID 代码阅读 | 8-10 小时 | 能读懂 TaskSE3Equality 和 Formulation |
| legged_control WBC 对比阅读 | 3-4 小时 | 能对比两套实现的优劣 |
| 实战练习(极简 WBC / Go2) | 6-8 小时 | 四足仿真中跑通站立平衡 |
| 思考题 + 前沿论文 | 3-4 小时 | 理解 VWBC 的基本思想 |

---
