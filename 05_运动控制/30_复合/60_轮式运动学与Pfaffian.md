# 第76章：轮式运动学与 Pfaffian 约束——非完整几何的完整推导

| 元信息 | 值 |
|--------|-----|
| 难度 | ⭐⭐⭐（非完整约束完整推导 + 轮接触建模 + 可控性分析） |
| 预计时间 | 1.5 周（30-35 小时） |
| 前置依赖 | 复合/10\_复合机器人全景、复合/20\_浮动基座臂统一动力学、足式/70\_腿足简化模型理论 |
| 下游连接 | 复合/70\_轮足混合MPC、复合/80\_Wheel\_Legged\_Gym\_RL、复合/90\_Swiss\_Mile商业化 |

> **本章定位**：轮足机器人比纯四足多出的第一个数学对象不是电机，而是速度层面的**非完整滚动约束**。本章从最基本的"为什么轮子不能横移"出发，完整推导 Pfaffian 约束矩阵，经过 Frobenius 可积性判据和 Chow 定理的可控性分析，延伸到实际的滑移建模和轮足混合运动学，最终落地到 ROS2 导航栈的运动学插件接口。全部推导不省略任何步骤，每个公式都给出物理动机和工程意义。

---

## 前置自测

📋 **前置自测**（答不出 2 题以上请先回顾对应章节）

| # | 问题 | 来源 |
|---|------|------|
| 1 | 浮动基座系统中 `model.nq` 与 `model.nv` 为什么不同？两者的差异来自哪个数学结构？ | 复合/20 |
| 2 | 足端静止接触约束如何写成 $J_c(q)\dot{q}=0$ 的形式？$J_c$ 的物理含义是什么？ | 足式/50 |
| 3 | SE(2) 群上的运动有几个自由度？用齐次坐标写出 SE(2) 变换矩阵。 | 足式/50 |
| 4 | 什么是"约束"在数学上的含义？位置约束和速度约束有什么本质区别？ | 基础力学 |
| 5 | 差速底盘为什么不能瞬时侧向平移？这个限制是永久性的还是瞬时性的？ | 本科机器人学 |

**自测标准**：5/5 直接阅读；3-4/5 边读边查；0-2/5 先回顾 复合/20 + 足式/50。

---

## 本章目标

学完本章后，你应当能够：

1. 从物理约束出发，独立推导差速、阿克曼、全向和麦克纳姆四种底盘的 Pfaffian 约束矩阵 $A(q)\dot{q}=0$。
2. 用 Frobenius 定理判断一个 Pfaffian 约束是否可积（完整），用 Lie bracket 给出判据。
3. 用 Chow 定理解释"差速车不能横移但能到达任意位置"的数学本质。
4. 定义滑移比和滑移角，解释 Pacejka 模型的简化形式及其工程用途。
5. 写出轮足混合系统中足+轮共存的统一 Pfaffian 约束矩阵。
6. 理解 ROS2 nav2 中运动学插件的接口设计及其与 Pfaffian 矩阵的对应关系。

---

## 76.1 动机：为什么轮式机器人不能随意横移？⭐

### 这一节解决什么问题

每个学过机器人学的人都知道"差速车不能横移"，但很少有人能精确回答：**这个限制的数学本质是什么？它是位置限制还是速度限制？它能被克服吗？**本节通过一个日常体验引入非完整约束的核心概念，为后续严格推导铺路。

### 从停车场说起

想象你正在驾驶一辆普通轿车（后轮驱动，前轮转向）。你需要从当前车道横向平移到相邻车道。直觉告诉你：**不能直接横移**——车子只能前进或后退，同时可以转弯。但经过一系列"前进-打方向-后退-回正"的操作，你确实可以到达相邻车道的目标位置。

这个日常经验揭示了一个深刻的数学性质：

| 特征 | 描述 |
|------|------|
| **瞬时限制** | 在任意时刻，车辆的速度方向受到约束——不能垂直于车轮平面运动 |
| **全局自由** | 给定足够时间和空间，车辆可以到达平面上的任意位置和朝向 |
| **矛盾的表象** | 速度受限，但位置不受限——这怎么可能？ |

这种"速度受限但位置不受限"的约束，就是**非完整约束**（nonholonomic constraint）。它与"位置受限"的**完整约束**（holonomic constraint）有本质不同。

### 完整约束 vs 非完整约束：第一次区分

在经典力学中，约束分为两大类：

**完整约束**（holonomic constraint）：可以写成位置（和时间）的函数：

$$
\phi(q, t) = 0
$$

例如，一个质点被约束在球面上：$x^2 + y^2 + z^2 - R^2 = 0$。这个约束限制了质点的**可达位置**——质点永远只能在球面上。

**非完整约束**（nonholonomic constraint）：只能写成速度的函数，**不能积分**为位置约束：

$$
A(q)\dot{q} = 0
$$

例如，轮子的纯滚动约束。这个约束限制了**瞬时速度方向**，但不限制可达位置。

> **本质洞察**：完整约束减少了系统的**配置空间维度**（质点从三维空间降到二维球面），而非完整约束减少了系统的**瞬时速度自由度**但不减少配置空间维度。这个区别正是轮式机器人运动规划复杂性的根源。

### 如果不区分这两种约束会怎样？

假设你在设计轮式机器人的路径规划器。如果你错误地把非完整约束当作完整约束处理：

1. **规划器会尝试生成横向平移轨迹**——因为在完整约束视角下，轮子的约束只限制了某些位置，而不限制速度方向
2. **底层控制器无法执行**——因为生成的轨迹在物理上不可行（要求车辆横移）
3. **实际表现**：机器人原地打转或完全不动，轮胎发出刺耳的摩擦声

反过来，如果你错误地把非完整约束当作"不可克服的限制"——认为差速车只能走直线：

1. **规划器过于保守**——很多可达位置被错误地标记为不可达
2. **机器人只能在狭窄走廊中直行**——丧失了通过多次机动到达任意位置的能力

正确的理解方式是：非完整约束限制的是**瞬时运动方向**（微分约束），而非**最终可达空间**（积分约束）。

### 历史脉络

非完整约束的研究始于 19 世纪末的经典力学：

| 年代 | 人物 | 贡献 |
|------|------|------|
| 1894 | Hertz | 首次系统讨论非完整约束在力学中的角色 |
| 1897 | Appell | 提出处理非完整系统的 Appell 方程 |
| 1900s | Chaplygin | 研究滚动圆盘等非完整系统的精确解 |
| 1939 | Chow | 证明了 Chow 定理——非完整系统的可控性判据 |
| 1970s | Brockett | 将非完整约束与控制论联系，开创非线性控制理论 |
| 1993 | Murray, Li, Sastry | 《A Mathematical Introduction to Robotic Manipulation》系统化了非完整运动规划 |
| 2000s | LaValle | 将采样方法（RRT 等）推广到非完整系统 |

> **类比**：非完整约束之于运动规划，就像不可压缩条件之于流体力学——都是对"速度场"的微分约束，而非对"位置"的代数约束。在流体力学中，$\nabla \cdot \mathbf{v} = 0$ 限制了速度场的散度为零，但流体仍可到达任意位置。在轮式机器人中，$A(q)\dot{q} = 0$ 限制了速度方向，但机器人仍可到达任意配置。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：直接用 A* 或 Dijkstra 规划轮式机器人路径**
> - 错误做法：在栅格地图上用标准 A* 搜索最短路径，然后直接让轮式机器人跟踪
> - 典型现象：规划出的路径包含急转弯甚至 90 度直角转弯，底层控制器无法跟踪，机器人频繁偏离路径
> - 根本原因：A*/Dijkstra 是对**完整系统**的路径搜索——它假设机器人可以沿任意方向移动。非完整系统需要使用 state-lattice planner 或 Hybrid A* 等考虑运动学约束的规划器
> - 正确做法：使用 Hybrid A*（在 $(x, y, \theta)$ 空间搜索）或 state-lattice planner（预计算满足运动学约束的运动原语）

> 💡 **概念误区：认为"非完整 = 不可控"**
> - 新手想法："轮子不能横移，所以差速车的横向位置不可控"
> - 实际上：非完整约束限制的是**瞬时速度方向**，不是**可达位置集合**。差速车通过"前进-转弯-前进"的序列可以到达平面上任意 $(x, y, \theta)$ 配置。非完整系统的可控性由 Chow 定理保证（本章 76.4 节详述）
> - 关键区别：不可控是指存在永远无法到达的状态；非完整是指到达某些状态需要"绕路"

> 🧠 **思维陷阱：混淆"不能横移"和"不能到达侧面位置"**
> - 新手想法："差速车不能横移，所以它不能到达正侧方 1 米处的位置"
> - 实际上：差速车完全可以到达侧方任意位置——只需要先前进、转 90 度、再前进。"不能横移"只是说不能**瞬时**沿侧向运动，不是说侧向位置不可达
> - 正确思维：区分"瞬时运动方向"（微分层面）和"可达配置集合"（积分层面）

### 练习 76.1

**[A] ⭐** 列举三个日常生活中的非完整约束和三个完整约束的例子。对每个例子，说明它限制的是位置还是速度。

**[B] ⭐⭐** 一个冰球在冰面上自由滑动。它受到的约束（如果有）是完整的还是非完整的？如果你给冰球加上一个"只能沿当前朝向滑动"的约束，情况如何变化？

**[C] ⭐⭐** 考虑一个在平面上运动的刚体，其配置空间是 $SE(2)$，维度为 3。如果施加一个非完整约束减少了一个瞬时速度自由度，剩余几个瞬时速度自由度？配置空间维度是否改变？

---

## 76.2 SE(2) 上的运动学与速度约束 ⭐⭐

### 这一节解决什么问题

轮式机器人在平面上运动，其配置空间是 $SE(2)$——平面上的刚体位姿空间。本节建立 $SE(2)$ 上的运动学框架，作为后续推导 Pfaffian 约束的数学基础。

### 从复合/20 桥接

回顾复合/20\_浮动基座臂统一动力学：浮动基座机器人的配置空间是 $SE(3)$（六维），广义坐标 $q_b$ 用四元数表示方位（$n_q = 7$），广义速度 $v_b$ 用 twist 表示（$n_v = 6$），因此 $n_q \neq n_v$。这是 Lie 群的基本特征。

对于平面运动，情况简化到 $SE(2)$——三维 Lie 群。一个在平面上运动的刚体，其配置完全由 **位置** $(x, y)$ 和 **朝向** $\theta$ 确定：

$$
q = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix} \in SE(2) \cong \mathbb{R}^2 \times S^1
$$

$SE(2)$ 的齐次坐标表示为：

$$
T = \begin{pmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{pmatrix} \in SE(2)
$$

**与 $SE(3)$ 的区别**：$SE(2)$ 上朝向只有一个角度 $\theta$，不需要四元数，因此 $n_q = n_v = 3$。这是 $S^1$（圆）相比 $SO(3)$（球面上的旋转）的简化。

### body frame 速度与 world frame 速度

刚体的速度可以在两个坐标系中表示：

**World frame 速度**（空间速度）：

$$
\dot{q} = \begin{pmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{pmatrix}
$$

**Body frame 速度**（体速度）：

$$
v^b = \begin{pmatrix} v_x^b \\ v_y^b \\ \omega \end{pmatrix}
$$

其中 $v_x^b$ 是沿车体前向的速度，$v_y^b$ 是沿车体侧向的速度，$\omega = \dot{\theta}$ 是偏航角速度。

两者通过旋转矩阵联系：

$$
\begin{pmatrix} \dot{x} \\ \dot{y} \end{pmatrix} = R(\theta) \begin{pmatrix} v_x^b \\ v_y^b \end{pmatrix} = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} v_x^b \\ v_y^b \end{pmatrix}
$$

展开后：

$$
\dot{x} = v_x^b \cos\theta - v_y^b \sin\theta
$$

$$
\dot{y} = v_x^b \sin\theta + v_y^b \cos\theta
$$

$$
\dot{\theta} = \omega
$$

> **反事实推理**：如果我们不区分 body frame 和 world frame 速度会怎样？考虑一个朝向 $\theta = 90°$ 的车辆以 body frame 前向速度 $v_x^b = 1$ m/s 行驶。在 world frame 中，它实际上是沿 $y$ 方向运动（$\dot{x}=0, \dot{y}=1$）。如果我们错误地把 body frame 速度直接当作 world frame 速度，就会计算出车辆沿 $x$ 方向运动——方向完全错误。这种错误在仿真中表现为机器人"螃蟹走路"。

### 约束的 Pfaffian 形式

现在我们可以精确定义：**Pfaffian 约束**是对广义速度的线性约束，其一般形式为：

$$
A(q)\dot{q} = 0
$$

其中 $A(q) \in \mathbb{R}^{k \times n}$，$k$ 是约束个数，$n$ 是广义坐标维度。

Pfaffian 约束矩阵 $A(q)$ 的每一行定义了一个**禁止的速度方向**——系统的速度不能有沿该方向的分量。所有满足约束的速度构成一个**允许速度分布**（distribution）：

$$
\mathcal{D}(q) = \{v \in \mathbb{R}^n \mid A(q)v = 0\} = \ker A(q)
$$

$\mathcal{D}(q)$ 的维度为 $n - \text{rank}(A(q))$，即系统在任意时刻的**瞬时运动自由度**。

| 概念 | 数学表示 | 物理含义 |
|------|---------|---------|
| 配置空间维度 | $n = \dim(q)$ | 系统需要几个坐标描述位置 |
| 约束个数 | $k = \text{rank}(A)$ | 几个独立的速度方向被禁止 |
| 瞬时自由度 | $n - k$ | 系统此刻能沿几个方向运动 |
| 允许速度分布 | $\mathcal{D}(q) = \ker A(q)$ | 此刻允许的所有速度的集合 |

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：在 world frame 中写约束矩阵时忘记旋转**
> - 错误做法：直接写 $A = [0, 1, 0]$（禁止 $y$ 方向速度）作为"不能横移"的约束
> - 典型现象：约束只在 $\theta = 0$ 时正确，其他朝向下约束方向错误
> - 根本原因：$A(q)$ 必须是配置 $q$ 的函数——特别是 $\theta$ 的函数。"禁止横向速度"在 body frame 是常数约束 $v_y^b = 0$，但转换到 world frame 后变成 $-\dot{x}\sin\theta + \dot{y}\cos\theta = 0$，矩阵 $A$ 依赖 $\theta$
> - 正确做法：先在 body frame 写约束，再通过旋转矩阵变换到 world frame

> 💡 **概念误区：认为 Pfaffian 约束 $A(q)\dot{q}=0$ 的形式与足端接触 $J_c(q)v=0$ 不同**
> - 新手想法："轮式约束和足端约束是两个完全不同的东西"
> - 实际上：两者在数学形式上完全相同！足端静止接触 $J_c(q)v = 0$ 就是一个 Pfaffian 约束，其中 $A(q) = J_c(q)$。区别仅在于**物理含义**：足端约束要求接触点速度为零（完整约束，可积分为固定位置），轮式约束要求接触点不横滑（非完整约束，不可积分为位置关系）
> - 延伸：这种统一视角正是轮足混合系统（76.7 节）的数学基础

### 练习 76.2

**[A] ⭐⭐** 写出 $SE(2)$ 上从 body frame 速度到 world frame 速度的完整变换矩阵。验证其行列式等于 1（即变换保体积）。

**[B] ⭐⭐** 如果一个平面机器人受到约束 $v_y^b = 0$（body frame 侧向速度为零），将其转换为 Pfaffian 形式 $A(q)\dot{q} = 0$。写出 $A(q)$ 并验证它在 $\theta = 0, \pi/4, \pi/2$ 时的具体形式。

**[C] ⭐⭐⭐** 考虑一个受到约束 $v_x^b = v_y^b = 0$（body frame 线速度为零，只能原地旋转）的机器人。这是一个什么类型的约束（完整/非完整）？它的允许速度分布维度是多少？

---

## 76.3 Pfaffian 约束矩阵的完整推导：四种底盘 ⭐⭐⭐

### 这一节解决什么问题

本节是全章的核心。我们将从物理约束出发，逐一推导差速、阿克曼、全向（Swedish wheel）和麦克纳姆四种主流底盘的 Pfaffian 约束矩阵。每种推导都遵循同一个三步框架：（1）明确物理约束（纯滚动 + 不横滑），（2）将 body frame 约束转换为 world frame，（3）组装 Pfaffian 矩阵 $A(q)$。

### 76.3.1 单轮圆盘：最小非完整系统 ⭐⭐

#### 动机

单轮圆盘（upright rolling disk）是非完整约束的**最小模型**——只有两个约束和四个自由度。理解它是理解所有轮式系统的基础。

#### 配置空间

一个竖直圆盘在平面上滚动，其配置由四个变量描述：

$$
q = \begin{pmatrix} x \\ y \\ \varphi \\ \theta \end{pmatrix} \in \mathbb{R}^2 \times S^1 \times S^1
$$

| 变量 | 含义 | 范围 |
|------|------|------|
| $(x, y)$ | 轮心在世界系中的位置 | $\mathbb{R}^2$ |
| $\varphi$ | 轮平面的朝向角（heading） | $[0, 2\pi)$ |
| $\theta$ | 轮绕自身轴的转角（rolling） | $\mathbb{R}$（无界） |

配置空间维度 $n = 4$。

#### 推导：从物理到数学

**Step 1：建立局部坐标系**

在轮的 body frame 中定义三个单位向量：

- 前向单位向量：$e_\parallel = (\cos\varphi, \sin\varphi)^T$（轮平面朝向）
- 横向单位向量：$e_\perp = (-\sin\varphi, \cos\varphi)^T$（垂直于轮平面）
- 法向单位向量：$e_n = (0, 0, 1)^T$（垂直于地面向上）

**Step 2：写出物理约束**

纯滚动条件包含两个独立约束：

**约束 1：纵向纯滚动**——轮心沿前向的速度等于轮缘线速度 $r\dot{\theta}$

这个约束的物理含义是：轮子向前滚动时，轮心走过的距离等于轮缘展开的弧长。如果不满足这个约束，要么轮子在地面上打滑（轮缘速度大于轮心速度），要么轮子被拖着走（轮心速度大于轮缘速度）。

数学表达：

$$
e_\parallel^T \dot{p} - r\dot{\theta} = 0
$$

展开：

$$
\dot{x}\cos\varphi + \dot{y}\sin\varphi - r\dot{\theta} = 0
$$

**约束 2：横向不滑**——轮心沿横向的速度为零

这个约束的物理含义是：轮子不会沿着轮轴方向滑动。如果违反这个约束，轮子就是在"侧滑"——就像汽车在冰面上打横一样。

数学表达：

$$
e_\perp^T \dot{p} = 0
$$

展开：

$$
-\dot{x}\sin\varphi + \dot{y}\cos\varphi = 0
$$

**Step 3：组装 Pfaffian 矩阵**

将两个约束写成矩阵形式 $A(q)\dot{q} = 0$：

$$
\underbrace{\begin{pmatrix} \cos\varphi & \sin\varphi & 0 & -r \\ -\sin\varphi & \cos\varphi & 0 & 0 \end{pmatrix}}_{A(q)} \begin{pmatrix} \dot{x} \\ \dot{y} \\ \dot{\varphi} \\ \dot{\theta} \end{pmatrix} = \begin{pmatrix} 0 \\ 0 \end{pmatrix}
$$

**验证维度**：$A \in \mathbb{R}^{2 \times 4}$，$\text{rank}(A) = 2$（两行线性无关，因为 $\cos^2\varphi + \sin^2\varphi = 1 \neq 0$），瞬时自由度 = $4 - 2 = 2$。

**瞬时自由度的物理含义**：圆盘在任意时刻只能做两种运动——向前/后滚动（改变 $\theta$ 和 $(x,y)$）和改变朝向（改变 $\varphi$）。不能横移，也不能在不转轮的情况下平移。

#### 允许速度的参数化

从 $A(q)\dot{q} = 0$ 的零空间求出允许速度：

$$
\dot{q} = u_1 \underbrace{\begin{pmatrix} r\cos\varphi \\ r\sin\varphi \\ 0 \\ 1 \end{pmatrix}}_{g_1(q)} + u_2 \underbrace{\begin{pmatrix} 0 \\ 0 \\ 1 \\ 0 \end{pmatrix}}_{g_2(q)}
$$

其中 $u_1 = \dot{\theta}$（轮转速）和 $u_2 = \dot{\varphi}$（转向速率）是两个输入。$g_1$ 和 $g_2$ 是两个**控制向量场**，它们生成了允许速度分布 $\mathcal{D}(q) = \text{span}\{g_1(q), g_2(q)\}$。

> **本质洞察**：Pfaffian 约束 $A(q)\dot{q} = 0$ 的零空间定义了一组控制向量场 $\{g_1, g_2, \ldots\}$。系统的运动方程变成 $\dot{q} = \sum_i u_i g_i(q)$——这是**漂移-free 仿射控制系统**的标准形式。非完整运动规划的所有理论工具（Chow 定理、RRT、steering function）都建立在这个形式之上。

### 76.3.2 差速底盘 ⭐⭐

#### 动机

差速底盘（differential drive）是最简单的实用移动机器人底盘，由两个独立驱动的轮子和一个或多个被动轮组成。TurtleBot、iRobot Create 和大量教学机器人都使用这种构型。

#### 配置与约束推导

差速底盘的配置只需三个变量（轮转角通过运动学约束隐含）：

$$
q = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix} \in SE(2)
$$

其中 $(x, y)$ 是两轮中点（或底盘质心）在世界系中的位置，$\theta$ 是底盘朝向。

两个轮子的转速 $\omega_L, \omega_R$ 通过以下运动学关系映射到底盘速度：

$$
v_x^b = \frac{r(\omega_R + \omega_L)}{2}, \quad \omega = \dot{\theta} = \frac{r(\omega_R - \omega_L)}{L}
$$

其中 $r$ 是轮半径，$L$ 是两轮间距。

差速底盘的**唯一约束**是横向不滑（body frame 侧向速度为零）：

$$
v_y^b = 0
$$

转换到 world frame：

$$
-\dot{x}\sin\theta + \dot{y}\cos\theta = 0
$$

Pfaffian 矩阵：

$$
A(q) = \begin{pmatrix} -\sin\theta & \cos\theta & 0 \end{pmatrix}
$$

**维度分析**：$n = 3$，$k = \text{rank}(A) = 1$，瞬时自由度 = $3 - 1 = 2$（前进/后退 + 原地旋转）。

注意差速底盘和单轮圆盘的关键区别：差速底盘有两个独立电机输入（$\omega_L, \omega_R$），恰好等于两个瞬时自由度——系统是**完全驱动的**（在允许速度子空间内）。

#### 运动方程

$$
\dot{q} = \begin{pmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{pmatrix} = \underbrace{\begin{pmatrix} \cos\theta \\ \sin\theta \\ 0 \end{pmatrix}}_{g_1} v + \underbrace{\begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}}_{g_2} \omega
$$

其中 $v = v_x^b$ 是前向线速度，$\omega = \dot{\theta}$ 是角速度。

### 76.3.3 阿克曼转向 ⭐⭐⭐

#### 动机

阿克曼转向（Ackermann steering）是汽车和类车机器人的标准转向方式。与差速底盘不同，阿克曼系统的转弯是通过前轮偏转实现的，而非两轮差速。

#### 配置与约束推导

阿克曼系统的配置需要四个变量：

$$
q = \begin{pmatrix} x \\ y \\ \theta \\ \delta \end{pmatrix}
$$

其中 $(x, y)$ 是后轴中点的位置，$\theta$ 是车体朝向，$\delta$ 是前轮转向角。

约束同样是后轮不横滑：

$$
v_y^b = 0 \implies -\dot{x}\sin\theta + \dot{y}\cos\theta = 0
$$

但阿克曼运动学引入了额外关系——前轮转向角 $\delta$ 与转弯半径 $R$ 的关系：

$$
\tan\delta = \frac{L_{\text{wb}}}{R}
$$

其中 $L_{\text{wb}}$ 是轴距。这给出了偏航角速度与前向速度的关系：

$$
\dot{\theta} = \frac{v}{R} = \frac{v \tan\delta}{L_{\text{wb}}}
$$

Pfaffian 矩阵：

$$
A(q) = \begin{pmatrix} -\sin\theta & \cos\theta & 0 & 0 \end{pmatrix}
$$

维度分析与差速底盘相同：$n = 4$，$k = 1$，瞬时自由度 = 3。但实际输入只有 2 个（前向速度 $v$ 和转向角速率 $\dot{\delta}$），因此系统是**欠驱动的**——$\delta$ 的变化不受 Pfaffian 约束限制，但受到机械转向角限制 $|\delta| \leq \delta_{\max}$。

**阿克曼运动方程**：

$$
\dot{q} = \begin{pmatrix} \cos\theta \\ \sin\theta \\ \frac{\tan\delta}{L_{\text{wb}}} \\ 0 \end{pmatrix} v + \begin{pmatrix} 0 \\ 0 \\ 0 \\ 1 \end{pmatrix} \dot{\delta}
$$

> **反事实推理**：如果阿克曼系统没有最大转向角限制（$\delta_{\max} = 90°$），会发生什么？当 $\delta \to 90°$ 时，$\tan\delta \to \infty$，意味着即使前向速度 $v$ 很小，偏航角速度 $\dot{\theta}$ 也趋向无穷——车辆绕后轴做极小半径旋转。物理上这等价于差速底盘的原地旋转。但实际汽车的 $\delta_{\max}$ 通常在 30-40 度，这进一步限制了最小转弯半径，使得停车入库等操作需要多次前进后退。

### 76.3.4 全向轮（Swedish/Omni Wheel） ⭐⭐

#### 动机

全向底盘（如 Kuka youBot 的底座、RoboCup 小型组机器人）使用特殊设计的轮子消除了横向不滑约束，使机器人能够沿任意方向瞬时运动。

#### 约束分析

全向轮（Swedish wheel / omni wheel）的关键设计：轮缘上安装了自由滚动的小辊子，使得轮子可以沿轴向自由滑动。因此，**横向不滑约束被消除**。

三轮全向底盘（120 度对称布置）的运动方程：

$$
\begin{pmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{pmatrix} = J^{-1}(\theta) \begin{pmatrix} r\omega_1 \\ r\omega_2 \\ r\omega_3 \end{pmatrix}
$$

**Pfaffian 约束矩阵**：$A(q) = \emptyset$——**无约束**。全向底盘是**完整系统**。

这意味着：全向底盘可以瞬时沿任意方向运动（前后、左右、旋转的任意组合）。配置空间维度 = 瞬时自由度 = 3。

| 对比项 | 差速底盘 | 阿克曼 | 全向 |
|--------|---------|--------|------|
| Pfaffian 约束数 | 1 | 1 | 0 |
| 瞬时自由度 | 2 | 2（实际输入）| 3 |
| 能否瞬时横移 | 否 | 否 | 能 |
| 运动规划复杂度 | 中等 | 高（最小转弯半径）| 低 |
| 机械复杂度 | 低 | 中 | 高 |

### 76.3.5 麦克纳姆轮 ⭐⭐⭐

#### 动机

麦克纳姆轮（Mecanum wheel）是全向运动的另一种实现方案，常用于仓库搬运机器人和 RoboCup 中型组。与 Swedish wheel 的 90 度辊子不同，麦克纳姆轮的辊子以 45 度角安装。

#### 运动学推导

四轮麦克纳姆底盘（矩形布置）的逆运动学：

$$
\begin{pmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{pmatrix} = \frac{1}{r} \begin{pmatrix} 1 & -1 & -(L_x + L_y) \\ 1 & 1 & (L_x + L_y) \\ 1 & 1 & -(L_x + L_y) \\ 1 & -1 & (L_x + L_y) \end{pmatrix} \begin{pmatrix} v_x^b \\ v_y^b \\ \omega \end{pmatrix}
$$

其中 $L_x, L_y$ 是轮子到底盘中心的前后和左右距离。

**Pfaffian 约束**：与全向轮类似，麦克纳姆底盘**没有非完整约束**——是完整系统。但实际工程中，辊子与地面的接触力学使得麦克纳姆轮的**横向力传递效率低**，因此侧向运动的最大加速度远小于前向。这不是运动学约束，而是动力学限制。

> **类比**：麦克纳姆轮之于 Swedish 轮，类似于锥齿轮之于正齿轮——都能实现同样的运动学功能，但通过不同的机械几何。45 度辊子分解力为纵向和横向分量，使得四个轮子的合力可以产生任意方向的平面运动。

### 代码验证：四种底盘运动学的统一仿真

以下代码将四种底盘的运动学方程封装为统一接口，便于对比仿真：

```python
import numpy as np
import matplotlib.pyplot as plt

class DifferentialDrive:
    """差速底盘运动学。"""
    def __init__(self, wheel_radius=0.05, track_width=0.3):
        self.r = wheel_radius
        self.L = track_width
    
    def forward_kinematics(self, q, u, dt):
        """
        q = (x, y, theta), u = (v, omega)
        返回: q_next
        """
        x, y, theta = q
        v, omega = u
        # Euler 积分
        x_next = x + v * np.cos(theta) * dt
        y_next = y + v * np.sin(theta) * dt
        theta_next = theta + omega * dt
        return np.array([x_next, y_next, theta_next])
    
    def pfaffian_residual(self, q, q_dot):
        """计算 Pfaffian 约束残差 A(q) @ q_dot"""
        theta = q[2]
        A = np.array([[-np.sin(theta), np.cos(theta), 0]])
        return A @ q_dot


class AckermannDrive:
    """阿克曼底盘运动学。"""
    def __init__(self, wheel_radius=0.05, wheelbase=0.5):
        self.r = wheel_radius
        self.L_wb = wheelbase
    
    def forward_kinematics(self, q, u, dt):
        """
        q = (x, y, theta, delta), u = (v, delta_dot)
        返回: q_next
        """
        x, y, theta, delta = q
        v, delta_dot = u
        x_next = x + v * np.cos(theta) * dt
        y_next = y + v * np.sin(theta) * dt
        theta_next = theta + v * np.tan(delta) / self.L_wb * dt
        delta_next = delta + delta_dot * dt
        return np.array([x_next, y_next, theta_next, delta_next])


# 仿真对比：差速车平行泊车 vs 全向底盘直接横移
def simulate_parking():
    diff = DifferentialDrive()
    
    # 差速车平行泊车轨迹：前进-左转-后退-右转
    q = np.array([0.0, 0.0, 0.0])
    trajectory_diff = [q.copy()]
    
    maneuvers = [
        (0.3, 0.5, 2.0),    # 前进 + 左转
        (-0.3, -0.5, 2.0),  # 后退 + 右转
        (0.3, 0.5, 2.0),    # 前进 + 左转
        (-0.3, -0.5, 2.0),  # 后退 + 右转
    ]
    
    dt = 0.01
    for v, omega, duration in maneuvers:
        for _ in range(int(duration / dt)):
            q = diff.forward_kinematics(q, [v, omega], dt)
            trajectory_diff.append(q.copy())
    
    trajectory_diff = np.array(trajectory_diff)
    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.plot(trajectory_diff[:, 0], trajectory_diff[:, 1], 'b-', label='差速车')
    ax.plot([0, 0], [0, 1], 'r--', label='全向底盘（直线）')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('平行泊车：差速车 vs 全向底盘')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True)
    plt.show()

# simulate_parking()  # 取消注释运行
```

### 四种底盘的 Pfaffian 矩阵总结

| 底盘类型 | $A(q)$ | $\text{rank}(A)$ | 瞬时自由度 | 非完整？ |
|---------|--------|-----------------|-----------|---------|
| 单轮圆盘 | $\begin{pmatrix} \cos\varphi & \sin\varphi & 0 & -r \\ -\sin\varphi & \cos\varphi & 0 & 0 \end{pmatrix}$ | 2 | 2 | 是 |
| 差速 | $\begin{pmatrix} -\sin\theta & \cos\theta & 0 \end{pmatrix}$ | 1 | 2 | 是 |
| 阿克曼 | $\begin{pmatrix} -\sin\theta & \cos\theta & 0 & 0 \end{pmatrix}$ | 1 | 3 | 是 |
| 全向/麦轮 | $\emptyset$ | 0 | 3 | 否 |

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：阿克曼模型中混用前轴和后轴参考点**
> - 错误做法：阿克曼运动学公式中 $(x, y)$ 取前轴中点，但 $\dot{\theta} = v\tan\delta/L$ 公式是以后轴为参考推导的
> - 典型现象：仿真中车辆走的弧线半径与预期不符，低速时误差小但高速时误差显著
> - 根本原因：$\dot{\theta} = v\tan\delta/L$ 中的 $v$ 是后轴中点的速度。如果 $(x,y)$ 取前轴，需要额外的运动学变换
> - 正确做法：统一选择后轴中点作为参考点，或者使用完整的双轮模型

> 💡 **概念误区：认为"全向底盘没有约束，所以运动规划更简单"**
> - 新手想法："全向底盘可以随意横移，路径规划就是直线连接起点和终点"
> - 实际上：全向底盘虽然没有运动学约束，但仍有**动力学约束**（最大加速度、轮子打滑限制）和**环境约束**（避障）。而且全向底盘的横向力传递效率低，实际最大横向加速度通常只有前向的 50-70%
> - 延伸：运动规划的复杂度不仅来自运动学约束，还来自动力学约束和环境约束

> 🧠 **思维陷阱：从单轮推导直接推广到多轮时忽略轮间耦合**
> - 新手想法："四个轮子分别满足各自的纯滚动约束就行了"
> - 实际上：多轮系统的约束是**冗余的**——四个轮子各有两个约束共 8 个，但 $SE(2)$ 只有 3 个自由度。因此 8 个约束中只有 1-3 个是独立的，其余是线性相关的。轮间几何关系（轮距、轴距）决定了哪些约束是独立的
> - 正确思维：先写出所有轮的约束，然后分析 $A(q)$ 的秩，只保留独立约束

### 练习 76.3

**[A] ⭐⭐** 为差速底盘推导正运动学：给定 $\omega_L, \omega_R$，计算 $(\dot{x}, \dot{y}, \dot{\theta})$。验证当 $\omega_L = \omega_R$ 时机器人直行，当 $\omega_L = -\omega_R$ 时原地旋转。

**[B] ⭐⭐⭐** 推导自行车模型（阿克曼的简化）的 Pfaffian 约束矩阵。自行车模型用 $q = (x, y, \theta, \delta)$ 描述，其中 $(x, y)$ 是后轮接地点。写出完整的 $A(q)$ 并与汽车的阿克曼矩阵对比。

**[C] ⭐⭐⭐** 用 Python + SymPy 符号计算四轮麦克纳姆底盘的正运动学矩阵。验证当四轮同速同向转时机器人前进，当对角轮反向时机器人原地旋转。

---

## 76.4 可积性与 Frobenius 定理 ⭐⭐⭐

### 这一节解决什么问题

上一节推导了四种底盘的 Pfaffian 约束。但一个关键问题悬而未决：**如何判断一个 Pfaffian 约束是否是非完整的？**换句话说，$A(q)\dot{q} = 0$ 能否积分为 $\phi(q) = 0$？Frobenius 定理给出了精确判据。

### 可积性的含义

如果 Pfaffian 约束 $A(q)\dot{q} = 0$ 可以积分为位置约束 $\phi(q) = 0$，则称该约束是**可积的**（integrable）或**完整的**。否则称为**不可积的**（non-integrable）或**非完整的**。

> **类比**：一个向量场是否是某个标量函数的梯度？在微积分中，$\mathbf{F} = \nabla \phi$ 当且仅当 $\nabla \times \mathbf{F} = 0$（无旋条件）。Frobenius 定理是这个条件的高维推广——将"无旋"推广为"对合性"（involutivity）。

### Frobenius 定理的陈述

**定理**（Frobenius）：设分布 $\mathcal{D}(q) = \text{span}\{g_1(q), \ldots, g_m(q)\}$ 是一个光滑的 $m$ 维分布。则 $\mathcal{D}$ 是可积的（即存在 $n - m$ 个独立函数 $\phi_1, \ldots, \phi_{n-m}$ 使得 $\nabla\phi_i \perp \mathcal{D}$），当且仅当 $\mathcal{D}$ 是**对合的**（involutive）：

$$
\forall\, i, j: \quad [g_i, g_j](q) \in \mathcal{D}(q)
$$

其中 $[g_i, g_j]$ 是 Lie bracket（李括号）。

### Lie bracket 的定义与计算

两个向量场 $g_i, g_j$ 的 Lie bracket 定义为：

$$
[g_i, g_j](q) = \frac{\partial g_j}{\partial q} g_i(q) - \frac{\partial g_i}{\partial q} g_j(q)
$$

**直觉理解**：Lie bracket $[g_i, g_j]$ 衡量了"先沿 $g_i$ 走一小步、再沿 $g_j$ 走一小步"与"先沿 $g_j$ 走一小步、再沿 $g_i$ 走一小步"之间的差异。如果两条路径的终点一致（bracket 为零或在 $\mathcal{D}$ 内），则约束是可积的；如果终点不同（bracket 不在 $\mathcal{D}$ 内），则说明系统可以通过交替使用两个输入来**生成新的运动方向**——这正是非完整系统"绕路到达"的数学本质。

### 差速底盘的可积性验证

让我们用 Frobenius 定理验证差速底盘的约束确实是非完整的。

差速底盘的两个控制向量场（76.3.2 节）：

$$
g_1 = \begin{pmatrix} \cos\theta \\ \sin\theta \\ 0 \end{pmatrix}, \quad g_2 = \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}
$$

**计算 Lie bracket $[g_1, g_2]$**：

首先计算两个雅可比矩阵：

$$
\frac{\partial g_1}{\partial q} = \begin{pmatrix} 0 & 0 & -\sin\theta \\ 0 & 0 & \cos\theta \\ 0 & 0 & 0 \end{pmatrix}, \quad \frac{\partial g_2}{\partial q} = \begin{pmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix}
$$

$$
[g_1, g_2] = \frac{\partial g_2}{\partial q} g_1 - \frac{\partial g_1}{\partial q} g_2 = \begin{pmatrix} 0 \\ 0 \\ 0 \end{pmatrix} - \begin{pmatrix} -\sin\theta \\ \cos\theta \\ 0 \end{pmatrix} = \begin{pmatrix} \sin\theta \\ -\cos\theta \\ 0 \end{pmatrix}
$$

**关键检验**：$[g_1, g_2] = (\sin\theta, -\cos\theta, 0)^T$。这个向量是否在 $\mathcal{D} = \text{span}\{g_1, g_2\}$ 内？

如果是，则需要存在 $\alpha, \beta$ 使得：

$$
\begin{pmatrix} \sin\theta \\ -\cos\theta \\ 0 \end{pmatrix} = \alpha \begin{pmatrix} \cos\theta \\ \sin\theta \\ 0 \end{pmatrix} + \beta \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}
$$

从第三行：$\beta = 0$。从前两行：$\alpha\cos\theta = \sin\theta$ 且 $\alpha\sin\theta = -\cos\theta$。两式相除得 $\tan\theta = -1/\tan\theta$，即 $\tan^2\theta = -1$——**无实数解**。

**结论**：$[g_1, g_2] \notin \mathcal{D}$，分布不是对合的，约束**不可积**，系统是**非完整的**。

> **本质洞察**：$[g_1, g_2] = (\sin\theta, -\cos\theta, 0)^T$ 恰好是 body frame 的**侧向方向**。这意味着：通过交替使用前进（$g_1$）和转向（$g_2$），差速车可以生成侧向运动——这正是平行泊车的数学本质！Lie bracket 产生的新方向，就是系统通过"绕路"可以到达的方向。

### Frobenius 定理的实用判据

对于 $k$ 个约束（$A \in \mathbb{R}^{k \times n}$）、$m = n - k$ 个控制向量场的情况，Frobenius 定理的等价条件可以用约束矩阵 $A(q)$ 直接判断：

**约束形式的 Frobenius 条件**：设 $\omega^i = A^i(q)$ 是 $A$ 的第 $i$ 行（约束一形式）。约束可积当且仅当：

$$
d\omega^i \wedge \omega^1 \wedge \cdots \wedge \omega^k = 0, \quad \forall\, i
$$

其中 $d\omega^i$ 是 $\omega^i$ 的外微分，$\wedge$ 是楔积。

对于**单个约束**（$k = 1$），条件简化为：

$$
d\omega \wedge \omega = 0
$$

对于差速底盘（$\omega = -\sin\theta\, dx + \cos\theta\, dy$）：

$$
d\omega = -\cos\theta\, d\theta \wedge dx - \sin\theta\, d\theta \wedge dy
$$

$$
d\omega \wedge \omega = (-\cos\theta\, d\theta \wedge dx - \sin\theta\, d\theta \wedge dy) \wedge (-\sin\theta\, dx + \cos\theta\, dy)
$$

$$
= \cos^2\theta\, d\theta \wedge dx \wedge dy + \sin^2\theta\, d\theta \wedge dy \wedge dx
$$

$$
= (\cos^2\theta - \sin^2\theta + 2\sin\theta\cos\theta)? \text{...}
$$

实际上注意到 $d\theta \wedge dy \wedge dx = -d\theta \wedge dx \wedge dy$，所以：

$$
d\omega \wedge \omega = (\cos^2\theta + \sin^2\theta)\, d\theta \wedge dx \wedge dy = d\theta \wedge dx \wedge dy \neq 0
$$

**结论一致**：$d\omega \wedge \omega \neq 0$，约束不可积。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：用数值方法计算 Lie bracket 时步长选取不当**
> - 错误做法：用有限差分近似 $\partial g / \partial q$，步长取 $h = 0.1$（过大）
> - 典型现象：数值 Lie bracket 与解析结果差异大，特别是在 $\theta$ 接近 $0$ 或 $\pi$ 时
> - 根本原因：$g_1 = (\cos\theta, \sin\theta, 0)^T$ 对 $\theta$ 的导数是三角函数，有限差分需要小步长才能精确近似
> - 正确做法：使用 SymPy 进行符号计算，或使用自动微分（如 JAX 的 `jacfwd`）

> 💡 **概念误区：认为"Frobenius 定理只是一个理论工具，工程中用不到"**
> - 新手想法："我知道差速车不能横移，不需要 Frobenius 定理来验证"
> - 实际上：Frobenius 定理在工程中有直接应用——当你设计新型轮系（如球形轮、万向节连接的轮组）时，需要判断新约束是否是非完整的，才能选择正确的运动规划算法。完整系统可以用简单的路径搜索 + 逆运动学，非完整系统必须使用 steering function 或采样规划
> - 工程应用：ROS2 nav2 的运动规划器选择（DWA vs TEB vs Lattice planner）正是基于底盘是否满足非完整约束

### 练习 76.4

**[A] ⭐⭐⭐** 用 SymPy 计算单轮圆盘的 Lie bracket $[g_1, g_2]$（76.3.1 的两个控制向量场），验证它不在分布 $\mathcal{D} = \text{span}\{g_1, g_2\}$ 内。

**[B] ⭐⭐⭐** 考虑一个配置为 $q = (x, y, \theta)$ 的系统，受约束 $\dot{x}\sin\theta - \dot{y}\cos\theta + \ell\dot{\theta} = 0$（$\ell$ 为常数）。用 Frobenius 定理判断这个约束是否可积。（提示：这是拖车约束）

**[C] ⭐⭐⭐⭐** 证明全向底盘的约束是可积的（trivially，因为没有约束）。然后考虑一个全向底盘加上一个拖车：全向底盘的 $(x, y, \theta)$ 无约束，但拖车的角度 $\phi$ 受到约束。写出完整系统的 Pfaffian 矩阵并判断可积性。

---

## 76.5 非完整运动规划：Chow 定理与可控性 ⭐⭐⭐

### 这一节解决什么问题

Frobenius 定理告诉我们差速车的约束是非完整的——瞬时不能横移。但"不能瞬时横移"不等于"不能到达侧方位置"。**Chow 定理**回答了最关键的问题：非完整系统到底能到达哪些配置？

### Lie Algebra Rank Condition (LARC)

回忆差速底盘的两个控制向量场 $g_1, g_2$ 和它们的 Lie bracket $g_3 = [g_1, g_2]$：

$$
g_1 = \begin{pmatrix} \cos\theta \\ \sin\theta \\ 0 \end{pmatrix}, \quad g_2 = \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}, \quad g_3 = [g_1, g_2] = \begin{pmatrix} \sin\theta \\ -\cos\theta \\ 0 \end{pmatrix}
$$

检查 $\{g_1, g_2, g_3\}$ 的秩：

$$
\text{rank}\begin{pmatrix} \cos\theta & 0 & \sin\theta \\ \sin\theta & 0 & -\cos\theta \\ 0 & 1 & 0 \end{pmatrix} = 3 \quad (\text{处处满秩})
$$

**验证**：行列式 = $0 \cdot (\cdots) - 1 \cdot (\cos\theta \cdot (-\cos\theta) - \sin\theta \cdot \sin\theta) + 0 = \cos^2\theta + \sin^2\theta = 1 \neq 0$。

三个向量场（原始 2 个 + Lie bracket 生成的 1 个）张成了完整的 $\mathbb{R}^3$ 空间。这就是 **Lie Algebra Rank Condition (LARC)** 满足的标志。

### Chow 定理

**定理**（Chow, 1939; Rashevsky, 1938）：设控制系统 $\dot{q} = \sum_{i=1}^m u_i g_i(q)$，其中 $g_1, \ldots, g_m$ 是光滑向量场。如果在每一点 $q$ 处，$g_1, \ldots, g_m$ 及其所有阶 Lie bracket 生成的 Lie algebra 的维度等于配置空间维度 $n$（即 LARC 成立），则系统是**小时间局部可控的**（small-time locally controllable, STLC）——从任意配置出发，可以在任意短时间内到达其邻域内的任意配置。

**对差速底盘的意义**：

- $g_1, g_2$ 只生成 2 维分布（不含横向）
- $g_1, g_2, [g_1, g_2]$ 生成 3 维分布（包含横向）——填满整个 $SE(2)$
- LARC 满足 $\Rightarrow$ 差速车可以从**任意**配置到达**任意**配置

> **本质洞察**：Chow 定理是"绕路到达"的数学保证。$[g_1, g_2]$ 代表的横向运动方向虽然不能通过单个输入直接产生，但可以通过**交替使用** $g_1$ 和 $g_2$ 来逼近。这就像停车时的前进-转弯-后退-回正序列——每一步都在允许速度空间内，但序列的净效果是横向位移。

### 可控性秩条件的层次

对于更复杂的系统，LARC 可能需要多阶 Lie bracket：

| 阶数 | 向量场 | 生成的新方向 |
|------|--------|------------|
| 0 阶 | $g_1, g_2$ | 前进/后退, 旋转 |
| 1 阶 | $[g_1, g_2]$ | 横向平移 |
| 2 阶 | $[g_1, [g_1, g_2]], [g_2, [g_1, g_2]]$ | 更高阶运动 |
| $k$ 阶 | $[\cdots]$ | 更精细的运动方向 |

**需要 1 阶 Lie bracket** 的系统（差速车）比**需要多阶 Lie bracket** 的系统（如带拖车的卡车）在运动规划上更容易——因为"绕路"的路径更短。

> **反事实推理**：如果 Lie bracket 不产生新方向（即分布是对合的/完整的），会怎样？那就意味着系统被约束在一个低维子流形上——类似于质点被约束在球面上。约束减少了可达配置集合的维度。这正是完整约束和非完整约束的根本区别：完整约束让配置空间"塌缩"，非完整约束保持配置空间维度不变，只限制到达某些配置的路径。

### 运动规划的实际影响

Chow 定理的理论结果直接影响运动规划器的设计：

| 系统类型 | LARC | 可控性 | 规划方法 |
|---------|------|--------|---------|
| 完整系统（全向底盘） | 自然满足 | 沿任意方向瞬时可控 | A*, RRT, 任意路径搜索 |
| 非完整 + LARC 满足 | 1-2 阶 | 可控但需要绕路 | Hybrid A*, RRT + steering, lattice |
| 非完整 + LARC 不满足 | 不满足 | 不完全可控 | 需要分析可达集 |

### ⚠️ 常见陷阱与误区

> 💡 **概念误区：认为 Chow 定理保证了"快速到达"**
> - 新手想法："差速车满足 LARC，所以可以快速到达任意位置"
> - 实际上：Chow 定理只保证**存在**到达的路径，不保证路径的长度或时间。通过 Lie bracket 生成的运动方向（横移）需要多次来回机动才能实现，效率远低于直接运动的方向。差速车横移 1 米需要的路径长度远大于前进 1 米
> - 延伸：路径效率与 Lie bracket 的阶数有关——阶数越高，"绕路"越长，运动规划越困难

> 🧠 **思维陷阱：把 LARC 当作设计指标**
> - 新手想法："只要 LARC 满足，底盘设计就没问题"
> - 实际上：LARC 是可控性的**必要条件的充分条件**。实际底盘设计还需要考虑到达效率、能耗、机械复杂度等因素。例如，全向底盘不需要 Lie bracket 就能横移，但机械更复杂、负载能力更低
> - 正确思维：LARC 是理论保证，工程选择需要综合权衡运动学灵活性、动力学性能和机械可靠性

### SymPy 符号验证

以下代码用 SymPy 自动化地验证差速底盘的 LARC。这不仅是一个验证工具，也是读者将来分析新型底盘可控性时的模板：

```python
import sympy as sp

# 定义符号
x, y, theta = sp.symbols('x y theta')
q = sp.Matrix([x, y, theta])

# 定义控制向量场
g1 = sp.Matrix([sp.cos(theta), sp.sin(theta), 0])  # 前进
g2 = sp.Matrix([0, 0, 1])                            # 旋转

def lie_bracket(f, g, q_vars):
    """
    计算两个向量场的 Lie bracket [f, g]。
    [f, g] = dg/dq * f - df/dq * g
    """
    n = len(q_vars)
    df_dq = sp.Matrix([[sp.diff(f[i], q_vars[j]) for j in range(n)] for i in range(n)])
    dg_dq = sp.Matrix([[sp.diff(g[i], q_vars[j]) for j in range(n)] for i in range(n)])
    return sp.simplify(dg_dq * f - df_dq * g)

# 计算 [g1, g2]
g3 = lie_bracket(g1, g2, q)
print("g1 =", g1.T)
print("g2 =", g2.T)
print("[g1, g2] =", g3.T)
# 输出: [g1, g2] = [sin(theta), -cos(theta), 0]

# 检查 LARC：{g1, g2, [g1,g2]} 的秩
rank_matrix = sp.Matrix([g1.T, g2.T, g3.T]).T
rank = rank_matrix.rank()
print(f"LARC 矩阵的秩 = {rank}, 配置空间维度 = 3")
print(f"LARC 满足: {rank == 3}")
# 输出: LARC 满足: True

# 验证行列式
det = rank_matrix.det()
print(f"行列式 = {sp.simplify(det)}")
# 输出: 行列式 = 1（处处非零！）
```

这段代码的输出确认了我们手工推导的结论：$[g_1, g_2] = (\sin\theta, -\cos\theta, 0)^T$ 与 $g_1, g_2$ 一起生成完整的 $\mathbb{R}^3$。

**读者练习**：将上述代码修改为分析单轮圆盘（4 维配置空间，2 个控制向量场），验证 LARC 满足需要几阶 Lie bracket。

### 可控性等级与运动规划效率

LARC 满足只保证可控性存在，但不同系统"绕路"的效率差异巨大。Sub-Riemannian 几何中的 **Ball-Box 定理** 给出了量化关系：

如果系统需要 $k$ 阶 Lie bracket 才能生成某个方向 $d$，那么在该方向上移动距离 $\epsilon$ 所需的最短路径长度约为 $O(\epsilon^{1/k})$。

| 系统 | LARC 阶数 | 横移 $\epsilon$ 米所需路径长度 | 规划难度 |
|------|----------|--------------------------|---------|
| 全向底盘 | 0（无需 Lie bracket）| $\epsilon$（直线）| 低 |
| 差速底盘 | 1 | $O(\sqrt{\epsilon})$ | 中 |
| 阿克曼底盘 | 1（有最小转弯半径）| $O(\sqrt{\epsilon})$ 但下界更大 | 高 |
| 拖车系统 | 2 | $O(\epsilon^{1/3})$ | 很高 |

这解释了一个直觉事实：差速车倒车入库比全向底盘难得多（需要多次前进后退），而拖着拖车倒车入库更是极其困难（需要更多次的精细机动）。

### 练习 76.5

**[A] ⭐⭐⭐** 对阿克曼底盘计算所有需要的 Lie bracket，验证 LARC 是否满足。如果满足，需要几阶 Lie bracket？

**[B] ⭐⭐⭐** 考虑一辆差速车拖着一个被动拖车（tractor-trailer system）。配置 $q = (x, y, \theta_1, \theta_2)$，其中 $\theta_1$ 是拖车朝向。写出控制向量场并验证 LARC 是否满足。需要几阶 Lie bracket？

**[C] ⭐⭐⭐⭐** （开放题）如果一个系统的 LARC 需要 $k$ 阶 Lie bracket 才能满足，直觉上"绕路"到达侧向位置需要多少倍于直线距离的路径长度？（提示：研究 sub-Riemannian geometry 中的 Ball-Box 定理）

---

## 76.6 滑移建模：当纯滚动假设失效 ⭐⭐⭐

### 这一节解决什么问题

前面五节的所有推导都基于一个关键假设：**纯滚动 + 不横滑**。但在低摩擦地面（如湿滑地砖、松软沙土）、高加速度转弯或紧急制动时，这个假设会失效。本节介绍如何量化"滑移程度"，并简要介绍 Pacejka 轮胎力模型。

### 两个滑移参数

滑移用两个无量纲参数描述：

**纵向滑移比**（longitudinal slip ratio）$\kappa$：

$$
\kappa = \frac{r\omega - v_x}{\max(|v_x|, v_{\text{low}})}
$$

| $\kappa$ 值 | 物理含义 |
|-------------|---------|
| $\kappa = 0$ | 纯滚动，$r\omega = v_x$ |
| $\kappa > 0$ | 驱动打滑（轮缘速度 > 地面速度），如起步时轮胎空转 |
| $\kappa < 0$ | 制动打滑（地面速度 > 轮缘速度），如急刹车 |
| $\kappa = 1$ | 完全打滑，轮子空转（$v_x = 0$ 但 $\omega \neq 0$） |
| $\kappa = -1$ | 完全抱死，轮子锁死（$\omega = 0$ 但 $v_x \neq 0$） |

> **注意**：分母中的 $v_{\text{low}}$ 是一个小正数（典型值 0.05-0.1 m/s），用于避免低速时除以零。这个低速阈值在工程中非常重要——低速时滑移比会因为分母接近零而剧烈波动，失去物理意义。

**侧向滑移角**（lateral slip angle）$\alpha$：

$$
\alpha = \arctan\left(\frac{v_y}{|v_x| + v_{\text{low}}}\right)
$$

| $\alpha$ 值 | 物理含义 |
|-------------|---------|
| $\alpha = 0$ | 无侧滑，轮子沿前向行驶 |
| $\alpha > 0$ | 轮子向左侧滑 |
| $\alpha < 0$ | 轮子向右侧滑 |

### Pacejka 魔术公式简介

Hans Pacejka（1934-2017）提出的"Magic Formula"是汽车工业中最广泛使用的轮胎力模型。它被称为"魔术"公式是因为**没有直接的物理推导**——它是一个纯经验拟合公式，但惊人地适用于各种轮胎和工况。

**简化 Pacejka 公式**（纯纵向或纯侧向）：

$$
F = F_z \cdot D \cdot \sin\left(C \cdot \arctan\left(B \cdot s - E \cdot (B \cdot s - \arctan(B \cdot s))\right)\right)
$$

其中 $s$ 是滑移参数（纵向为 $\kappa$，侧向为 $\alpha$），$B, C, D, E$ 是四个经验参数：

| 参数 | 名称 | 物理含义 |
|------|------|---------|
| $B$ | 刚度因子 | 小滑移时的力-滑移斜率 |
| $C$ | 形状因子 | 曲线形状（通常 $C \approx 1.3$ 纵向，$C \approx 1.5$ 侧向）|
| $D$ | 峰值因子 | 最大摩擦系数 $\mu_{\max}$ |
| $E$ | 曲率因子 | 峰值后的下降速率 |

**简化摩擦锥约束**：

无论使用多复杂的轮胎模型，接触力始终受到摩擦锥限制：

$$
\sqrt{F_x^2 + F_y^2} \leq \mu F_z
$$

这是所有轮式接触约束的最终边界。

### Pacejka 曲线的物理理解

Pacejka 曲线的典型形状是一条"S 形"曲线，具有以下特征区域：

| 区域 | 滑移范围 | 物理行为 | 控制意义 |
|------|---------|---------|---------|
| **线性区** | $|s| < 0.03$ | 力与滑移近似线性 $F \approx C_s \cdot s$ | 正常操作区，PD 控制有效 |
| **峰值区** | $|s| \approx 0.05$-$0.1$ | 摩擦力达到最大值 $F_{\max} = D \cdot F_z$ | 最大制动/驱动力，ABS/TCS 目标 |
| **滑动区** | $|s| > 0.15$ | 摩擦力下降（$F < F_{\max}$） | 失控区，必须减速或切换模式 |

对于轮足机器人，通常不需要精确拟合 Pacejka 参数——因为轮子很小（半径 $\sim 0.05$ m），接触面积小，轮胎力学与汽车差异大。但**力-滑移曲线的定性形状**仍然适用：小滑移时力线性增长，大滑移时力饱和甚至下降。

**线性轮胎模型的适用范围**：

对于轮足机器人的控制器设计，一个简化的线性模型通常足够：

$$
F_x = C_\kappa \cdot \kappa, \quad F_y = C_\alpha \cdot \alpha
$$

其中 $C_\kappa$ 是纵向刚度（典型值 $\sim 500$-$2000$ N/rad），$C_\alpha$ 是侧偏刚度（典型值 $\sim 300$-$1500$ N/rad）。这个模型在 $|\kappa| < 0.05$ 和 $|\alpha| < 3°$ 的范围内精度足够。

### 滑移在轮足控制中的角色

对于轮足机器人，滑移不是简单的"异常状态"，而是**需要在控制回路中主动管理的物理量**。具体来说：

| 模块 | 滑移的用途 |
|------|-----------|
| **状态估计** | 滑移大时增大轮速观测的协方差——减小对轮里程计的信任 |
| **MPC** | 滑移大时软化纯滚动约束——从等式约束变为惩罚项 |
| **WBC** | 滑移大时减小分配给该轮的法向力——防止进一步打滑 |
| **安全监控** | 滑移超阈值时限制最大速度——触发降级策略 |

> **反事实推理**：如果控制器完全忽略滑移，假设轮子永远满足纯滚动，会怎样？在高摩擦干燥地面上也许没问题。但一旦进入低摩擦表面（湿地、草地、碎石），纯滚动假设给出的轮速观测与实际地面速度不符，状态估计会产生漂移。估计漂移传给 MPC，MPC 产生错误的力/速度参考，WBC 执行错误的力矩——形成**正反馈环路**，最终可能导致失控。这正是为什么滑移检测和闭环使用如此重要。

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：低速时直接使用滑移比公式**
> - 错误做法：$\kappa = (r\omega - v_x) / v_x$，不加低速保护
> - 典型现象：机器人低速启动时滑移比突然飙到 $\pm 100$，触发安全保护或导致控制器震荡
> - 根本原因：当 $v_x \to 0$ 时分母趋零，数值爆炸
> - 正确做法：使用 $\max(|v_x|, v_{\text{low}})$ 作为分母，并在 $|v_x| < v_{\text{low}}$ 时切换为直接使用速度残差 $r\omega - v_x$ 而非比值

> 💡 **概念误区：认为 Pacejka 模型是轮足机器人必须使用的轮胎模型**
> - 新手想法："Pacejka 是最好的轮胎模型，所以轮足机器人应该用它"
> - 实际上：Pacejka 模型有 6-20 个经验参数，需要大量轮胎测试数据来标定。轮足机器人的"轮胎"通常是橡胶包覆的小轮，与汽车轮胎差异巨大。实际工程中，轮足机器人更常用**线性力-滑移模型**（$F = k \cdot s$，$s$ 为滑移参数）或**简单摩擦锥**（$|F_t| \leq \mu F_n$），因为参数少、可标定、计算快
> - 正确选择：根据应用精度需求选择模型复杂度。户外高速场景可考虑简化 Pacejka，室内低速场景用线性模型足够

### 练习 76.6

**[A] ⭐⭐** 实现一个 Python 函数 `slip_ratio(v_wheel, v_ground, v_low=0.05)`，计算带低速保护的纵向滑移比。测试 $v_{\text{ground}} = 0.01, 0.1, 1.0$ m/s 和 $v_{\text{wheel}} = 0, 0.5, 1.0, 2.0$ m/s 的各种组合，画出滑移比热力图。

**[B] ⭐⭐⭐** 用 Pacejka 简化公式（$B=10, C=1.5, D=0.8, E=-0.5$）绘制侧向力 vs 滑移角曲线。标出峰值摩擦系数对应的滑移角。讨论：为什么峰值不在最大滑移角处？

**[C] ⭐⭐⭐** 设计一个"滑移感知权重调度器"：输入为四轮的滑移比，输出为四个权重 $w_i \in [0, 1]$，用于调节状态估计器对每个轮速观测的信任度。要求 $w_i$ 在 $|\kappa_i| < 0.05$ 时为 1，在 $|\kappa_i| > 0.3$ 时为 0，中间平滑过渡。

---

## 76.7 轮足混合运动学：足+轮统一 Pfaffian ⭐⭐⭐

### 这一节解决什么问题

前面所有推导都是针对纯轮式系统的。但轮足机器人（如 ANYmal-on-wheels、Swiss-Mile 的产品）的每个末端可以在**足模式**（foot）、**轮模式**（wheel）和**摆动模式**（swing）之间切换。本节把这三种接触模式统一到同一个 Pfaffian 约束框架中，为下一章（复合/70\_轮足混合MPC）打下基础。

### 从复合/20 桥接

回顾复合/20\_浮动基座臂统一动力学：浮动基座系统的动力学方程为 $M(q)\dot{v} + h(q,v) = S^T\tau + J_c^T\lambda$。其中接触雅可比 $J_c$ 描述了接触点速度与广义速度的关系：$v_c = J_c(q)v$。

在纯足式系统中，接触约束是 $J_c(q)v = 0$——接触点世界系速度为零。这是一个**完整约束**（可积分为"接触点位置固定"）。

在轮足系统中，轮模式的接触约束不再是速度为零，而是速度方向受限：

$$
\text{足模式}: \quad v_c = J_c(q)v = 0 \quad (\text{三维速度为零，完整约束})
$$

$$
\text{轮模式}: \quad \begin{cases} e_\perp^T J_c(q) v = 0 & (\text{横向不滑}) \\ e_\parallel^T J_c(q) v - r\dot{\theta} = 0 & (\text{纵向纯滚动}) \\ n^T J_c(q) v = 0 & (\text{法向不穿透}) \end{cases}
$$

$$
\text{摆动模式}: \quad \text{无接触约束}
$$

### 模式相关的统一约束矩阵

设第 $i$ 个末端的接触点速度为 $v_{c,i} = J_{c,i}(q)v$，模式为 $m_i \in \{\text{swing}, \text{foot}, \text{wheel}\}$。定义模式相关的约束矩阵 $A_i^{m_i}$：

$$
A_i^{\text{swing}} = \emptyset \quad (\text{无约束行})
$$

$$
A_i^{\text{foot}} = J_{c,i}(q) \quad (\text{3 行约束})
$$

$$
A_i^{\text{wheel}} = \begin{pmatrix} e_{\perp,i}^T J_{c,i}(q) \\ e_{\parallel,i}^T J_{c,i}(q) - r_i [0 \cdots 1_{\theta_i} \cdots 0] \\ n_i^T J_{c,i}(q) \end{pmatrix} \quad (\text{3 行约束，但结构不同})
$$

其中 $[0 \cdots 1_{\theta_i} \cdots 0]$ 是一个行向量，在轮转角对应的速度索引位置为 1，其余为 0。

**统一 Pfaffian 矩阵**：将所有末端的约束垂直堆叠：

$$
A_{\text{unified}}(q, \sigma) = \begin{pmatrix} A_1^{m_1} \\ A_2^{m_2} \\ A_3^{m_3} \\ A_4^{m_4} \end{pmatrix}
$$

其中 $\sigma = (m_1, m_2, m_3, m_4)$ 是当前模式向量。

**关键观察**：统一 Pfaffian 矩阵的**结构随模式切换而变化**——行数、非零模式都不同。这种结构变化正是轮足 MPC（复合/70）面临的核心数学困难。

| 模式组合 | 约束行数 | 瞬时自由度（$n_v = 24$ 的 Go2+wheels） |
|---------|---------|--------------------------------------|
| 4 foot | 12 | 12 |
| 4 wheel | 12 | 12（但约束结构不同） |
| 2 foot + 2 wheel | 12 | 12 |
| 2 foot + 2 swing | 6 | 18 |
| 4 swing（空中） | 0 | 24 |

### 局部坐标系的构造

轮模式约束需要在每个接触点定义局部坐标系 $(e_\parallel, e_\perp, n)$。这三个方向的来源是：

- **法向** $n$：由地形表面法向决定。平地上 $n = (0, 0, 1)^T$，坡面上由地形估计给出
- **前向** $e_\parallel$：轮平面朝向在地面切平面上的投影。必须先投影到切平面再归一化
- **横向** $e_\perp$：$e_\perp = n \times e_\parallel$

**注意**：在非平坦地面上，$e_\parallel$ 不能直接取车体前向——必须先投影到地形切平面。否则约束方程中会出现法向分量的泄漏，导致残差偏置。

```python
import numpy as np

def build_contact_frame(body_forward, terrain_normal):
    """
    从车体前向和地形法向构造接触点局部坐标系。
    
    参数:
        body_forward: 车体前向方向 (3,)，世界系
        terrain_normal: 地形表面法向 (3,)，世界系，朝上
    返回:
        e_par: 滚动方向（前向投影到切平面）
        e_perp: 横向方向
        n: 法向方向
    """
    n = terrain_normal / np.linalg.norm(terrain_normal)
    
    # 将车体前向投影到地形切平面
    e_par = body_forward - np.dot(body_forward, n) * n
    norm_par = np.linalg.norm(e_par)
    if norm_par < 1e-6:
        raise ValueError("车体前向与地形法向平行，无法构造坐标系")
    e_par = e_par / norm_par
    
    # 横向 = 法向 x 前向
    e_perp = np.cross(n, e_par)
    e_perp = e_perp / np.linalg.norm(e_perp)
    
    return e_par, e_perp, n
```

### 完整的残差计算代码

以下代码给出可复用的滚动残差模块——它是本章理论落地到代码的核心产物，也是后续 MPC（复合/70）和 RL（复合/80）都会复用的基础组件：

```python
import numpy as np

def rolling_residual(jac_contact, v_generalized, wheel_speed, radius, 
                     forward, lateral, normal):
    """
    计算轮接触点的三维滚动残差。
    
    参数:
        jac_contact: 接触点雅可比 (3, nv) 或 (6, nv)，取前 3 行
        v_generalized: 广义速度 (nv,)
        wheel_speed: 轮角速度 (rad/s)，标量
        radius: 轮半径 (m)，标量
        forward: 滚动方向（前向），已投影到切平面 (3,)
        lateral: 横向方向 (3,)
        normal: 地面法向 (3,)
    返回:
        residual: (3,) = [r_roll, r_lat, r_normal]
        - r_roll: 纵向纯滚动残差（应为 0）
        - r_lat:  横向侧滑残差（应为 0）
        - r_normal: 法向穿透残差（应为 0）
    """
    # 取雅可比的线性部分（如果是 6xnv，取前 3 行）
    if jac_contact.shape[0] == 6:
        jac_contact = jac_contact[:3, :]
    
    # 接触点线速度 (3,)
    v_contact = jac_contact @ v_generalized
    
    # 归一化方向向量
    def safe_normalize(v):
        n = np.linalg.norm(v)
        if n < 1e-9:
            raise ValueError(f"方向向量长度过小: {n}")
        return v / n
    
    e_fwd = safe_normalize(forward)
    e_lat = safe_normalize(lateral)
    e_nrm = safe_normalize(normal)
    
    # 三维残差
    r_roll = e_fwd @ v_contact - radius * wheel_speed  # 纵向匹配
    r_lat = e_lat @ v_contact                           # 横向不滑
    r_normal = e_nrm @ v_contact                        # 法向不穿透
    
    return np.array([r_roll, r_lat, r_normal])


def mode_aware_constraint(jac_contact, v_generalized, wheel_speed, radius,
                          forward, lateral, normal, mode):
    """
    根据模式返回对应的约束残差。
    
    参数:
        mode: 'swing' | 'foot' | 'wheel'
    返回:
        residual: 约束残差向量（维度取决于模式）
        - swing: 空向量 (0,)
        - foot: (3,) = J_c @ v（三维速度为零）
        - wheel: (3,) = [r_roll, r_lat, r_normal]
    """
    if mode == 'swing':
        return np.array([])
    
    if jac_contact.shape[0] == 6:
        jac_contact = jac_contact[:3, :]
    
    if mode == 'foot':
        # 足模式：接触点三维速度为零
        return jac_contact @ v_generalized
    
    elif mode == 'wheel':
        # 轮模式：Pfaffian 约束
        return rolling_residual(
            jac_contact, v_generalized, wheel_speed, radius,
            forward, lateral, normal
        )
    else:
        raise ValueError(f"未知模式: {mode}")
```

**单元测试思路**：

1. 构造一个满足纯滚动约束的广义速度（即 $v_x^b = r\omega$，$v_y^b = 0$），验证残差为零
2. 故意修改轮半径（减小 5%），验证 $r_{\text{roll}}$ 出现系统偏置
3. 故意反转轮轴方向，验证 $r_{\text{roll}}$ 符号反转
4. 在坡面上不做切平面投影，验证 $r_n$ 出现偏置

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：轮模式仍使用足模式的 $J_c v = 0$ 约束**
> - 错误做法：模式切换为 wheel 后，约束矩阵不更新
> - 典型现象：机器人在轮模式下无法滚动——优化器试图让轮接触点速度为零，与电机驱动的轮转速矛盾，产生力矩尖峰
> - 根本原因：足模式要求 $v_c = 0$（三维速度为零），轮模式允许纵向速度 $e_\parallel^T v_c = r\dot{\theta}$
> - 正确做法：模式切换时同步更新约束矩阵的行数和结构

> 💡 **概念误区：认为轮足切换只是"约束矩阵换一行"**
> - 新手想法："足到轮的切换只需要把 $J_c v = 0$ 改成 $e_\perp^T J_c v = 0$"
> - 实际上：切换涉及多个层面的变化——约束矩阵结构、MPC 代价函数权重、WBC 任务优先级、状态估计器的观测模型。任何一层不同步都会导致切换瞬间的力矩尖峰或状态跳变
> - 正确做法：模式切换消息必须带时间戳，所有模块（MPC、WBC、估计器）必须在同一控制周期切换

> 🧠 **思维陷阱：在坡面上直接用世界系 forward 作为滚动方向**
> - 新手想法："滚动方向就是车头方向，取车体前向就行"
> - 实际上：在 8 度坡面上，如果不做切平面投影，法向残差 $r_n = n^T J_c v$ 会出现系统性偏置（因为"前向"有法向分量）。这个偏置被估计器当作法向穿透，触发虚假的接触力
> - 正确做法：所有方向向量必须先投影到地形切平面，再归一化

### 练习 76.7

**[A] ⭐⭐** 写出一个四轮轮足机器人在"前两轮 wheel + 后两腿 foot"模式下的统一 Pfaffian 矩阵的维度和结构（用分块矩阵表示）。

**[B] ⭐⭐⭐** 实现 `rolling_residual` 函数：输入 Pinocchio 的接触雅可比、广义速度、轮速、半径和局部坐标系，输出三维残差向量 $(r_{\text{roll}}, r_{\text{lat}}, r_n)$。用单位测试验证：当广义速度满足纯滚动约束时，残差为零。

**[C] ⭐⭐⭐** 讨论：当模式从 foot 切换到 wheel 时，约束矩阵的秩是否改变？如果秩不变，约束空间的几何结构有何变化？用零空间的基底来解释。

---

## 76.8 工程实现：ROS2 nav2 运动学插件接口 ⭐⭐

### 这一节解决什么问题

前七节建立了完整的理论框架。本节将理论落地到最常用的移动机器人软件栈——ROS2 Navigation2（nav2），展示 Pfaffian 约束如何映射到 nav2 的运动学插件接口。

### nav2 的运动学抽象

ROS2 nav2 在 Galactic/Humble+ 版本中引入了**运动学插件**（kinematics plugin）的概念。核心接口包括：

| 接口方法 | 功能 | 与 Pfaffian 的关系 |
|---------|------|-------------------|
| `isPositionReachable(from, to)` | 判断目标是否可达 | Chow 定理的工程实现 |
| `getMinTurningRadius()` | 最小转弯半径 | 阿克曼约束的参数化 |
| `isHolonomic()` | 底盘是否全向 | $A(q) = \emptyset$ 的判断 |
| `projectVelocity(cmd_vel)` | 将命令速度投影到可行空间 | $\mathcal{D}(q) = \ker A(q)$ 上的投影 |

`projectVelocity` 是最核心的方法。它将一个可能违反运动学约束的速度命令投影到允许速度分布上：

$$
v_{\text{projected}} = (I - A^{\dagger}A) \, v_{\text{cmd}}
$$

这恰好是 $A(q)$ 零空间的正交投影。

### 运动学参数与 Pfaffian 矩阵的对应

| nav2 参数 | Pfaffian 对应 | 差速底盘值 | 阿克曼值 |
|----------|--------------|-----------|---------|
| `holonomic` | $\text{rank}(A) = 0?$ | `false` | `false` |
| `min_turning_radius` | $L/\tan(\delta_{\max})$ | 0（可原地转）| 1-5 m |
| `max_velocity_x` | 不在 $A$ 中，是动力学限制 | 1.0 m/s | 3.0 m/s |
| `max_velocity_y` | $A$ 决定是否为零 | 0（非完整）| 0（非完整）|
| `max_angular_velocity` | 不在 $A$ 中，是动力学限制 | 2.0 rad/s | 0.5 rad/s |

### 轮足机器人的 nav2 适配

对于轮足机器人，nav2 的标准运动学插件需要扩展——因为底盘类型在运行时可以切换：

1. **wheel 模式**：等效于差速底盘（非完整），`isHolonomic() = false`
2. **foot 模式**：等效于全向（在平面投影上），`isHolonomic() = true`
3. **混合模式**：需要自定义插件

```python
# 轮足运动学插件的概念代码
class WheelLeggedKinematicsPlugin:
    def __init__(self, robot_config):
        self.wheel_radius = robot_config['wheel_radius']
        self.track_width = robot_config['track_width']
        self.current_mode = 'wheel'  # 初始模式
    
    def is_holonomic(self):
        """根据当前模式返回是否全向"""
        if self.current_mode == 'wheel':
            return False  # 差速/阿克曼约束
        elif self.current_mode == 'foot':
            return True   # 足式步态可横移
        else:
            return False  # 混合模式保守处理
    
    def project_velocity(self, cmd_vel, current_state):
        """将速度命令投影到当前模式的可行空间"""
        if self.current_mode == 'wheel':
            # 差速约束：v_y = 0
            return [cmd_vel[0], 0.0, cmd_vel[2]]
        elif self.current_mode == 'foot':
            # 无约束
            return cmd_vel
        else:
            # 混合模式：根据接触状态决定
            return self._mixed_projection(cmd_vel, current_state)
```

### ⚠️ 常见陷阱与误区

> ⚠️ **编程陷阱：nav2 插件中硬编码底盘类型**
> - 错误做法：在 nav2 配置中固定 `robot_model_type: differential`，不支持运行时切换
> - 典型现象：轮足机器人在足模式下使用差速规划器，生成弧线路径去到达旁边 1 米处的目标（实际可以直接横移）
> - 修正思路：编写自定义运动学插件，根据模式消息动态切换 `isHolonomic()` 返回值

> 💡 **概念误区：认为 nav2 的 `cmd_vel` 就是机器人的实际速度**
> - 新手想法："nav2 发布的 `cmd_vel` 直接发给底盘就行"
> - 实际上：`cmd_vel` 是**期望速度**，底盘控制器需要将其转换为**轮速命令**（通过逆运动学），再由电机控制器执行。中间有延迟、饱和和执行误差。nav2 的 velocity smoother 插件部分处理了这些问题，但轮足机器人的模式切换需要额外的平滑逻辑

### 练习 76.8

**[A] ⭐⭐** 为差速底盘实现完整的 `projectVelocity` 函数：输入 $(v_x, v_y, \omega)$，输出投影后的速度。验证投影后 $v_y = 0$。

**[B] ⭐⭐⭐** 设计一个模式切换状态机：定义 wheel/foot/swing 三种模式的切换条件和最短驻留时间。画出状态转移图并解释每个转移条件的物理含义。

**[C] ⭐⭐⭐** 阅读 ROS2 nav2 的 `nav2_costmap_2d` 文档，理解 costmap 如何处理机器人的足印（footprint）。讨论：轮足机器人在 wheel 模式和 foot 模式下的等效足印是否相同？对避障有什么影响？

---

## 本章小结

| 模块 | 核心问题 | 应掌握的能力 |
|------|---------|------------|
| 76.1 动机 | 为什么轮子不能横移？ | 区分完整/非完整约束 |
| 76.2 SE(2) 运动学 | 速度约束的数学形式？ | 写出 $A(q)\dot{q}=0$ 形式 |
| 76.3 Pfaffian 推导 | 四种底盘的约束矩阵？ | 独立推导差速/阿克曼/全向/麦轮的 $A(q)$ |
| 76.4 Frobenius 定理 | 约束是否可积？ | 用 Lie bracket 判断可积性 |
| 76.5 Chow 定理 | 非完整系统能到达哪里？ | 验证 LARC 并解释可控性 |
| 76.6 滑移建模 | 纯滚动失效时怎么办？ | 计算滑移比/角，理解 Pacejka 模型 |
| 76.7 轮足混合 | 足+轮约束如何统一？ | 写出模式相关的统一 Pfaffian |
| 76.8 工程实现 | 如何落地到 ROS2？ | 理解 nav2 运动学插件接口 |

---

## 累积项目：本章新增模块——Rolling Constraint Checker

本章为轮足复合机器人的累积项目新增以下模块：

| 阶段 | 任务 | 交付物 |
|------|------|--------|
| 1. 几何检查 | 读取 URDF 中的轮关节配置（轴向、半径、速度索引） | 轮配置报告脚本 |
| 2. 约束构造 | 实现 `rolling_residual()` 函数，计算三维残差 | 可复用的 Python 模块 |
| 3. 可积性验证 | 用 SymPy 计算 Lie bracket，验证非完整性 | 符号计算脚本 |
| 4. 滑移检测 | 实现滑移比/角计算和低速保护 | 滑移时间线绘图脚本 |
| 5. 异常注入 | 故意修改半径/轴向/索引，验证残差是否正确定位异常 | 故障定位报告 |

---

## Pfaffian 约束在轮足混合平台上的扩展讨论

### 从固定底盘到浮动基座：约束结构的本质变化

前文推导的 Pfaffian 约束 $A(q)\dot{q}=0$ 隐含了一个关键假设——底盘是刚体，轮子与底盘的几何关系固定。当我们从纯轮式平台过渡到轮足混合平台（如 ANYmal-on-wheels、Unitree B2-W），这个假设不再成立：轮子安装在腿末端，而腿的构型会改变轮子相对于基座的位置和姿态。

这意味着 Pfaffian 约束矩阵 $A(q)$ 不再只是底盘构型 $q_b \in SE(2)$ 的函数，而是整个系统广义坐标 $q = (q_b, q_\ell)$ 的函数，其中 $q_\ell$ 包含所有腿关节角度。具体来说，每个轮子的接地点位置和滚动方向都要通过正运动学从基座经由腿链计算得到：

$$
p_{\text{contact},i} = \text{FK}(q_b, q_{\ell,i}), \quad e_{\parallel,i} = R_{\text{wheel},i}(q_b, q_{\ell,i}) \cdot e_x
$$

因此，统一 Pfaffian 约束变为：

$$
A(q_b, q_\ell) \begin{pmatrix} \dot{q}_b \\ \dot{q}_\ell \end{pmatrix} = 0
$$

这个扩展后的约束矩阵有两个重要特性：（1）它的维度随着系统模式（哪些腿处于轮模式、哪些处于足模式）动态变化；（2）它对腿关节角度的依赖使得约束的 Lie bracket 结构更加复杂——这直接影响 Chow 定理的可控性分析。

### Chow 定理的直觉解释

Chow 定理（也称 Chow-Rashevskii 定理）回答了一个根本性问题：**受非完整约束限制的系统，能否到达构型空间中的任意点？** 其精确陈述为：如果约束分布 $\Delta$ 及其所有 Lie bracket 生成的分布 $\text{Lie}(\Delta)$ 在每一点都张成整个切空间，则系统是完全可控的——即任意两点之间存在满足约束的路径。

**直觉理解**：想象一辆汽车（阿克曼转向）。它在任何瞬间只能沿前进方向移动或转弯——不能横移。这是非完整约束的效果。但是通过"前进-转弯-前进-转弯"的组合（类似平行泊车），汽车可以到达平面上的任意位置和朝向。Chow 定理告诉我们：**反复执行允许的运动并取它们的"组合效应"（Lie bracket），就能产生原本被禁止的运动方向**。

用数学语言说，如果 $f_1$ 和 $f_2$ 是两个允许的速度方向，它们的 Lie bracket $[f_1, f_2]$ 表示"先沿 $f_1$ 走一小段，再沿 $f_2$ 走一小段，再沿 $-f_1$ 走一小段，再沿 $-f_2$ 走一小段"的净效应——这个净效应可能是一个全新的运动方向（如汽车的侧移）。

**对轮足混合的意义**：纯轮式底盘的可控性取决于底盘类型——差速可控、阿克曼可控、全向可控。但轮足混合系统有一个纯轮式没有的优势：**腿的运动提供了额外的速度方向**。即使轮子受非完整约束限制，腿的摆动可以直接在约束被限制的方向上产生位移。这意味着轮足混合系统的可控性通常优于同等轮式配置的纯轮系统——Chow 条件更容易满足。

### LARC 条件的实际验证

**Lie Algebra Rank Condition (LARC)** 是 Chow 定理的可操作形式：在某个构型点 $q_0$ 处，计算约束分布的所有阶 Lie bracket，检查它们是否张成整个切空间。对于轮足混合系统，这个验证可以用 SymPy 符号计算自动化：

1. 写出约束向量场 $f_1, f_2, \ldots, f_k$（约束零空间的基底）
2. 计算一阶 Lie bracket $[f_i, f_j]$
3. 检查 $\{f_1, \ldots, f_k, [f_1,f_2], [f_1,f_3], \ldots\}$ 的秩
4. 如果秩等于构型空间维度，则系统可控

**工程价值**：LARC 验证不仅是理论工具，它直接指导运动规划器的设计。如果某个模式组合下 LARC 条件不满足（比如所有腿都锁定在某个特殊构型），规划器应当避免这个构型——或者切换到其他模式。

> **学习建议**：初学者可以先在最简单的差速底盘上手动验证 LARC（只需 2 个向量场，1 个 Lie bracket），建立对"Lie bracket 产生新方向"的直觉。然后用 SymPy 处理四轮差速和轮足混合的更复杂情况。本章 76.5 节的练习提供了完整的符号计算框架。

---

## 延伸阅读

- **基础** ⭐⭐ Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Chapter 7: 非完整约束与可控性的经典教科书推导
- **基础** ⭐⭐ Lynch & Park, *Modern Robotics*, Section 13.3: 移动机器人速度约束的几何推导，配有优秀的可视化
- **进阶** ⭐⭐⭐ Bjelonic et al., "Whole-Body MPC and Online Gait Sequence Generation for Wheeled-Legged Robots" (IROS 2021): ANYmal-on-wheels 的 WBC 和步态序列优化
- **进阶** ⭐⭐⭐ Bjelonic et al., "Rolling in the Deep: Hybrid Locomotion for Wheeled-Legged Robots Using Online Trajectory Optimization" (RAL 2020): 轮足在线轨迹优化
- **拓展** ⭐⭐⭐⭐ Pacejka, *Tire and Vehicle Dynamics*: 轮胎力模型的权威参考，工业级 Pacejka 公式的完整推导
- **拓展** ⭐⭐⭐ LaValle, *Planning Algorithms*, Chapter 15: 非完整系统的采样运动规划
- **工程** ⭐⭐ ROS2 nav2 文档, Kinematics Plugin Interface: nav2 运动学插件的 API 参考

---

## 跨章综合练习

**综合题 1**（需要复合/20 + 本章知识）：Go2+wheels 系统有 24 维广义速度。四轮全部在 wheel 模式下，写出统一 Pfaffian 约束矩阵 $A_{\text{unified}}$ 的维度和秩。计算约束后的瞬时自由度。与纯足式（4 foot）模式的自由度对比，解释为什么轮模式允许更多运动方向。

**综合题 2**（需要足式/50 + 本章知识）：复合/20 中质量矩阵的分块结构 $M_{bb}, M_{b\ell}, \ldots$ 是在关节空间定义的。现在加入轮约束 $A(q)\dot{q} = 0$，约束后的有效动力学方程是什么？（提示：使用约束空间动力学 $M_c = P^T M P$，其中 $P$ 是 $A$ 零空间的基底矩阵）

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 残差长期偏置（$r_{\text{roll}}$ 非零均值） | 轮半径标定错误或坐标系不一致 | 1. 固定速度直行 2. 打印 $r\omega$ 和 $e_\parallel^T J_c v$ 3. 比较单位 | 76.3, 76.6 |
| 模式切换瞬间力矩尖峰 | 约束矩阵在切换瞬间不连续跳变 | 1. 画模式时间线 2. 检查约束行数变化 3. 添加平滑过渡窗口 | 76.7 |
| 仿真中轮子"拖拽"（不转但机器人前进） | 轮模式仍使用 $J_c v = 0$（足模式约束） | 1. 打印约束类型 2. 验证模式消息传递 3. 检查约束矩阵构造逻辑 | 76.7 |
| 低速时滑移比异常震荡 | 滑移比公式分母接近零 | 1. 打印 $v_x$ 和 $v_{\text{low}}$ 2. 确认低速阈值合理 3. 低速段切换为速度残差 | 76.6 |
| 坡面上法向残差 $r_n$ 持续非零 | 滚动方向未投影到地形切平面 | 1. 打印 $e_\parallel$ 与 $n$ 的内积 2. 确认 `build_contact_frame` 正确 3. 检查地形法向估计 | 76.7 |

---

## 本章核心公式速查表

以下汇总本章最重要的公式及其物理含义，供复习和快速查阅。

| 公式 | 含义 | 适用场景 |
|------|------|---------|
| $A(q)\dot{q} = 0$ | Pfaffian 约束的标准形式：速度必须在约束矩阵零空间内 | 所有非完整轮式/轮足系统 |
| $\dot{x}\sin\theta - \dot{y}\cos\theta = 0$ | 差速底盘的侧向速度为零约束 | 差速/阿克曼底盘 |
| $[f_1, f_2] = \frac{\partial f_2}{\partial q}f_1 - \frac{\partial f_1}{\partial q}f_2$ | Lie bracket：两个向量场的"交换子" | Frobenius 可积性判断、LARC 验证 |
| $\text{rank}(\text{Lie}(\Delta)) = n$ | Chow 定理条件：Lie bracket 生成的分布张成全空间 | 非完整系统可控性分析 |
| $s_x = \frac{v_x - r\omega}{\max(|v_x|, |r\omega|, v_{\text{low}})}$ | 纵向滑移比：衡量轮胎纵向滑移程度 | 滑移检测与建模 |
| $\alpha = \arctan(v_y / v_x)$ | 侧偏角：衡量轮胎侧向滑移 | 高速转弯、侧滑检测 |
| $M_c = P^T M P$ | 约束空间有效质量矩阵（$P$ 为约束零空间基） | 约束后动力学分析 |

> **使用建议**：遇到轮足约束问题时，先确定当前模式（foot/wheel/mixed），写出对应的 $A(q)$，再检查零空间维度是否与预期自由度一致。如果不一致，通常是约束矩阵构造有误。

---

## 本章与后续章节的衔接

本章建立的 Pfaffian 约束框架是后续多个章节的基础：

- **复合/70 轮足混合 MPC**：将本章的约束矩阵 $A(q)$ 作为 MPC 优化问题的等式约束，约束残差成为 MPC 的可行性指标
- **复合/80 轮足 WBC**：将滚动约束作为 WBC 的最高优先级任务，确保 QP 解满足非完整约束
- **复合/90 模式切换**：本章 76.7 节的模式相关约束矩阵是模式切换策略设计的数学基础
- **足式/50 空间向量代数**：本章的雅可比矩阵 $J_c$ 与空间向量的关系在足式/50 中有详细推导

学习者在完成本章后，建议直接进入复合/70（轮足混合 MPC），在 MPC 的优化框架中看到 Pfaffian 约束如何变成"活的工程模块"——从纸面公式到实时求解的全过程。

> **核心收获回顾**：本章的核心不是某一个具体公式，而是一种思维方式——**任何轮式或轮足系统的运动分析，都从写出 Pfaffian 约束开始**。约束矩阵 $A(q)$ 决定了系统的瞬时自由度、可控性和规划空间的维度。掌握了这个起点，后续的运动学控制、MPC 和 WBC 都是在这个约束框架上的自然延伸。
>
> 如果你在后续章节中遇到约束相关的困惑，
> 随时回到本章的 76.3 节（Pfaffian 推导）和 76.5 节（Chow 定理）复习核心概念。

---

**—— 本章终 ——**
