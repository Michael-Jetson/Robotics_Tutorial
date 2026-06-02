# D2 MPC 与自适应控制——从 OCP 公式到 acados 亚毫秒求解

**性质**：算法工程教学 | **难度跨度**：⭐⭐ ~ ⭐⭐⭐⭐ | **预计精读**：12-16 小时

> **一句话定位**：从 OCP 标准形式到 SQP-RTI 的"准备-反馈"流水线，从 rpg_mpc 的 Eigen::Map 零拷贝到 acados/HPIPM/BLASFEO 三层栈的亚毫秒反馈延迟，再到 L1 自适应、GP-MPC、Neural-MPC 三条鲁棒性增强路线——本章完整讲透**四旋翼 NMPC 的问题结构、求解器工程与扰动补偿**，建立从"理解 MPC 是什么"到"在嵌入式 CPU 上实时运行 NMPC"的完整知识链。

---

## 前置自测

开始前先回答下面 5 个问题。答不出 2 题以上，建议先回前置章节补齐——D2 的每一步推导和代码都建立在这些基础之上，欠了账会在 D2.3 卡住。

1. **MPC 的"滚动时域"（Receding Horizon）是什么意思？为什么不直接求解整条轨迹的最优控制？**
   （答不出 → 回 Part 0 最优控制基础）

2. **SQP（Sequential Quadratic Programming）每次迭代做什么？它与无约束 Newton 法的关系是什么？**
   （答不出 → 回 Part 0 非线性优化）

3. **四旋翼的状态向量包含哪些物理量？控制输入通常选什么？为什么选 CTBR（Collective Thrust + Body Rate）而非单电机转速？**
   （答不出 → 回 D1 微分平坦与 SE(3) 控制，§D1.2）

4. **`Eigen::Map` 是什么？它和 `Eigen::Matrix` 的核心区别是什么？在 1 kHz 控制循环中为什么要用 Map 而非 Matrix？**
   （答不出 → 回 v8 Ch11 Eigen 深度，§Ch11.4）

5. **什么是"代码生成"（Code Generation）求解器？与"解释型"（Runtime）求解器有什么不同？为什么嵌入式 MPC 倾向于使用代码生成？**
   （答不出 → 回 v8 Ch17 Ceres/GTSAM 中的自动微分与代码生成讨论）

**参考答案要点**（先自己答，再对照）：

1. 滚动时域：每个控制周期只优化未来 $N$ 步的有限窗口，执行第一步控制 $u_0^*$ 后窗口前移一步，下一个测量到达后重新优化。直接求解整条轨迹的 NLP 规模太大（$N_{\text{total}} \gg N$），且对模型误差没有反馈修正机制。滚动时域兼顾了前瞻性（看 $N$ 步以后）和反馈性（每步用最新测量纠偏）。

2. SQP 在每次迭代中，将非线性约束优化线性化为一个 QP 子问题并求解，得到搜索方向 $\delta z^*$。它相当于 Newton 法在约束优化下的推广——Hessian 由 Lagrangian 的二阶信息近似（Gauss-Newton 时用 $J^\top J$ 近似 $\nabla^2 L$），约束被线性化为 QP 的线性等式/不等式约束。

3. 状态 $x \in \mathbb{R}^{10}$：位置 $p$(3) + 速度 $v$(3) + 四元数姿态 $q$(4)。控制 $u \in \mathbb{R}^4$：质量归一化总推力 $f$(1) + 体轴角速度 $\omega$(3)。选 CTBR 因为它与 PX4/Betaflight 的内环 rate controller 直接匹配，将高带宽内环（~2 kHz）隐藏在飞控固件内，MPC 只需 50-200 Hz。单电机转速需额外建模电机一阶时延（$\tau_m \sim 10$-$50$ ms），增加 4 个状态维度。

4. `Eigen::Map` 不分配内存，直接包装已有的原始内存指针（raw pointer）；`Eigen::Matrix` 拥有自己的内存（固定大小在栈上，动态大小在堆上）。Map 的 `sizeof` 只有 ~16 字节（一个指针 + 维度信息）。在 1 kHz 循环中，MPC 求解器输出的 C 数组需要频繁转换为 Eigen 矩阵——Map 实现零拷贝，避免每次迭代 memcpy 4-5 KB 数据的缓存污染。

5. 代码生成求解器在离线阶段针对特定问题结构编译为定制 C 代码——无分支预测失败、无动态内存分配、循环完全展开。解释型求解器是通用运行时，需要在线解析问题结构。代码生成消除了运行时开销但牺牲了灵活性（改模型需重新生成编译）。嵌入式 CPU（如 Pixhawk 的 STM32F7）缓存小、分支预测简单，代码生成的零间接调用对性能至关重要。

---

## 本章目标

学完本章后，你应该能够：

1. **建模**：写出四旋翼 MPC 的标准 OCP 公式——10 状态（$p, v, q$） $\times$ 4 输入（$f, \omega$） $\times$ $N=20$ 射击节点 $\times$ 0.05-0.1 s 步长，并理解每个选择背后的物理约束
2. **求解器**：解释 SQP-RTI 的"准备-反馈"两阶段流水线，以及 acados/HPIPM/BLASFEO 三层栈如何实现亚毫秒反馈延迟
3. **工程**：理解 rpg_mpc 的三层 C++ 架构（mpc_solver $\to$ mpc_wrapper $\to$ mpc_controller），能复用 Eigen::Map 零拷贝模式
4. **自适应**：掌握 L1 自适应控制如何**叠加在**（而非替代）MPC 之上，在线估计未建模扰动并补偿
5. **数据驱动**：理解 GP-MPC 和 Neural-MPC 两条将数据驱动残差放入 MPC 的技术路线及其各自权衡
6. **对比**：能系统对比基于优化的 MPC（SQP-RTI）与基于采样的 MPC（MPPI）在四旋翼与足式机器人两种场景下的适用条件

---

### 本章知识导航

本章的知识结构是一棵以"如何在四旋翼上实时运行 NMPC"为根的树。树干是三个递进的核心问题——怎么建模、怎么解、怎么让它鲁棒——树枝是每个问题的技术细节和工程实现。

```
为什么四旋翼需要 MPC？ (§D2.1 问题结构)
    │
    ├─→ 怎么写成数学问题？ (§D2.1 OCP 公式)
    │     ├─ 状态/输入维度选择
    │     ├─ 代价函数设计
    │     └─ 动力学离散化
    │
    ├─→ 怎么实时求解？ (§D2.2-D2.4)
    │     ├─ rpg_mpc 三层架构 (§D2.2) ─→ Eigen::Map 零拷贝
    │     ├─ acados 三层栈 (§D2.3) ──→ HPIPM + BLASFEO
    │     └─ SQP-RTI 算法 (§D2.4) ──→ 准备-反馈流水线
    │
    ├─→ 怎么让它鲁棒？ (§D2.5-D2.7)
    │     ├─ L1 自适应 (§D2.5) ──→ 零训练在线补偿
    │     ├─ GP-MPC (§D2.6) ────→ 少量数据 + 不确定性
    │     └─ Neural-MPC (§D2.7) ─→ l4casadi + 最高精度
    │
    └─→ 还有别的方法吗？ (§D2.8-D2.9)
          ├─ MPPI 采样式 MPC (§D2.8) ─→ GPU 并行、无导数
          └─ 前沿融合 (§D2.9) ────────→ AC-MPC、MPCC++
```

| 小节 | 主题 | 难度 | 一句话 |
|------|------|------|--------|
| §D2.1 | 四旋翼 OCP 问题结构 | ⭐⭐ | 10 状态 4 输入 N=20——最简单的真实机器人 MPC |
| §D2.2 | rpg_mpc：三层 C++ 架构 | ⭐⭐ | ACADO 代码生成 + Eigen::Map 零拷贝的教科书 |
| §D2.3 | acados：现代求解器栈 | ⭐⭐⭐ | HPIPM 块三对角 Riccati + BLASFEO 小矩阵优化 |
| §D2.4 | SQP-RTI 算法深入 | ⭐⭐⭐ | 准备-反馈流水线将延迟压缩到 0.1 ms |
| §D2.5 | L1 自适应 + NMPC | ⭐⭐⭐ | 零训练、0.01 ms、叠加补偿——首选鲁棒方案 |
| §D2.6 | GP-MPC | ⭐⭐⭐ | 稀疏高斯过程残差增强 + 预测不确定性 |
| §D2.7 | Neural-MPC + l4casadi | ⭐⭐⭐⭐ | PyTorch $\to$ CasADi 桥接，82% 误差降低 |
| §D2.8 | MPPI 对比 | ⭐⭐ | 无导数采样式 MPC 的适用场景与局限 |
| §D2.9 | 前沿：AC-MPC / MPCC++ | ⭐⭐⭐⭐ | 可微 MPC 作为 RL actor，21 m/s 真机竞速 |

**两条阅读线**：

- **理论线**（理解算法）：§D2.1 $\to$ §D2.4 $\to$ §D2.5 $\to$ §D2.7 $\to$ §D2.9，重点是 OCP 公式、SQP-RTI 数学、自适应理论
- **工程线**（会用就行）：§D2.1 $\to$ §D2.2 $\to$ §D2.3 $\to$ §D2.5，重点是 acados Python 接口、rpg_mpc 架构、L1 补偿实现

无论哪条线，§D2.1 都是必读——它是所有后续内容的地基。

---

### 前置知识桥接

**回顾 D1（微分平坦与 SE(3) 控制）**：D1 建立了四旋翼的核心数学性质——给定平坦输出 $\sigma(t) = (x, y, z, \psi)^\top$ 及其前四阶导数，12 维状态和 4 维控制输入可通过**纯代数运算**恢复。这意味着：规划问题从"12 维状态空间的最优控制"坍缩为"4 维平坦输出空间的曲线设计"。但 D1 的 SE(3) 几何控制器是**逆动力学控制器**——假设参考轨迹已知、无输入约束、无前瞻性。本章的 MPC 正是为了弥补这三个缺口：它在**有约束、有前瞻**的框架下做最优控制。

**回顾 v8 Ch11（Eigen 深度）**：rpg_mpc 的核心 C++ 技巧——`Eigen::Map` 零拷贝——直接来自 Ch11 学过的"固定大小 vs 动态大小矩阵"选型原则。四旋翼 MPC 的矩阵尺寸只有 10$\times$10、10$\times$4、14$\times$14，全部用固定大小矩阵在栈上分配，编译器自动 SIMD 向量化。如果在 Ch11 学的 `Eigen::Map` 这里会直接用上。

**回顾 v8 Ch17-18（Ceres/GTSAM）**：SLAM 中 Ceres 用 Gauss-Newton 迭代求解非线性最小二乘，GTSAM 用 Bayes Tree 做增量变量消元。MPC 的 SQP 在数学上与 Gauss-Newton 同构，HPIPM 的 Riccati 递推与 Bayes Tree 在线性链拓扑上的消元等价。**学过 SLAM 的非线性优化，你已经懂了 MPC 求解器的 80%。**

**前向预告**：本章的 MPC 负责"跟踪给定参考轨迹"。但参考轨迹从何而来？下一章 D3 的多项式轨迹生成（Mellinger/Richter 方法）和 D5 的 MINCO 将解答这个问题——它们在微分平坦空间中生成最优轨迹，然后交给本章的 MPC 跟踪。现在只需要知道：**MPC 的输入是一条时间参数化的参考轨迹 $x_{\text{ref}}(t)$，MPC 的输出是每步的最优控制 $u_0^*$。**

---

### 如果跳过本章会怎样

跳过 D2，你会卡在三个具体的地方。

**场景一："PID 够用吗？"** 你用 D1 学的 SE(3) 几何控制器跟踪一条 8 m/s 的竞速轨迹。悬停和低速时完美——但高速急转弯时，控制器只看当前误差、不看未来，无法提前减速入弯。结果要么冲出赛道，要么不得不把整条轨迹放慢到 3 m/s。根本原因是几何控制器没有**前瞻性**（prediction horizon），不知道 0.5 秒后要做急转弯。MPC 的滚动时域恰好补上这个缺口。

**场景二："为什么 acados 这么快？"** 你用 CasADi + Ipopt 实现了四旋翼 NMPC——数学上完全正确，但每步求解要 15 ms，50 Hz 控制频率都勉强。你把 N 从 20 减到 10，牺牲了预测精度。你不知道 acados 的 SQP-RTI + HPIPM + BLASFEO 三层栈可以把同一个问题的反馈延迟压缩到 0.1-1 ms，比 Ipopt 快 10-100 倍——因为 Ipopt 是通用 NLP 求解器，完全没有利用 OCP 的块三对角 KKT 结构。没有 D2.3-D2.4，你会一直用大炮打蚊子。

**场景三："挂了载荷就飘了"** 你的四旋翼 MPC 在无扰动条件下跟踪精度 5 mm。然后挂上一个 30% 体重的载荷——位置误差暴增到 30 mm，侧风一吹直接漂移 50 cm。你知道"模型不准了"但不知道怎么在线修正。L1 自适应只需 0.01 ms 额外计算就能将误差降低 80-90%，但如果没学过 D2.5，你可能花三个月去折腾 GP 训练或重新辨识模型参数。

---

### 预计阅读时间

| 模式 | 时长 | 适合 |
|------|------|------|
| **精读** | 12-16 小时 | 第一次系统学四旋翼 MPC：逐节读动机 $\to$ 推导 $\to$ 代码，运行 acados Python 示例，做完每节练习。建议分 4-5 次。 |
| **速读** | 4-5 小时 | 有优化基础、想建立全局图景：读每节的"动机"和关键公式、§D2.3 acados 栈概览、§D2.5 L1-NMPC 核心思想，跳过代码细节和数值实验。 |
| **速查** | 30-60 分钟 | 已学过、回来查特定内容：直接定位到对应小节，看框住的公式 + 符号表 + API 速查。 |

---

### 科研发展脉络

在钻进具体方法前，先把这条研究线的来龙去脉理清——知道每个方法"从哪来、解决了前人什么痛点、又留下什么给后人"，比孤立地记名字有用得多。

| 年份 | 论文 | Venue | 核心贡献 |
|------|------|-------|---------|
| 2017 | Kamel, Stastny, Alexis, Siegwart (ETH ASL) | 书章 | **范式定义**：ACADO + qpOASES 做成 ROS 包，定义了 mav_control_rw 范式 |
| 2018 | Falanga, Foehn, Lu, Scaramuzza (UZH RPG) | IROS | **感知代价入 MPC**：惩罚 POI 投影偏离光轴 + 像素速度 |
| 2021 | Torrente, Kaufmann, Foehn, Scaramuzza | RA-L | **GP 残差增强**：稀疏 GP 学体速度残差，~70% 跟踪误差降低 |
| 2022 | Hanover, Foehn, Sun, Kaufmann, Scaramuzza | RA-L | **L1 + NMPC**：未知载荷 60% 体重仍可跟踪，0.82 ms |
| 2022 | O'Connell, Shi 等 (Caltech) | Sci. Rob. | **元学习风基**：12 分钟训练，43.6 km/h 风下鲁棒 |
| 2022 | Verschueren, Frison 等 | MPC | **acados 系统论文**：BLASFEO + HPIPM + SQP-RTI |
| 2023 | Salzmann 等 (TUM/UZH) | RA-L | **Neural-MPC**：l4casadi 桥接，82% 误差降低 |
| 2024 | Frey 等 | arXiv | **AS-RTI**：高级步实时迭代 + 多层级 SQP，反馈延迟 ~0.1 ms |
| 2025 | Romero, Aljalbout, Song, Scaramuzza | T-RO | **Actor-Critic MPC**：可微 MPC 作为 RL actor，21 m/s 竞速 |
| 2025 | Frey 等 | OCAM | **多相 OCP**：acados 多阶段 OCP，结合不同动力学模型 |

**关键实验室脉络**：ETH ASL（Kamel/Siegwart，ACADO 范式奠基）$\to$ UZH RPG（Scaramuzza 团队，PAMPC/L1-NMPC/AC-MPC）$\to$ TUM AAS（Salzmann，Neural-MPC/l4casadi）$\to$ Caltech AMBER（O'Connell，Neural-Fly 元学习）$\to$ Freiburg（Frison/Verschueren，acados/HPIPM/BLASFEO 底层栈）。

看这条线，有一条清晰的主线：**模型的精细程度不断提高，求解器的延迟不断降低**。早期用刚体名义模型 + 通用 QP（ACADO + qpOASES），中期用结构化 QP + 实时迭代（acados + HPIPM + SQP-RTI），近期用数据驱动残差模型 + 自适应补偿（L1/GP/NN）。2024 年以后的最新趋势是 RL 优化 MPC 的代价函数参数（AC-MPC），将 MPC 的约束满足能力与 RL 的长期目标优化能力融合。

---

### 本章符号约定

| 符号 | 含义 | 首见 |
|------|------|------|
| $x \in \mathbb{R}^{10}$ | 状态向量 $(p, v, q)$ | §D2.1 |
| $u \in \mathbb{R}^4$ | 控制输入 $(f, \omega_x, \omega_y, \omega_z)$ | §D2.1 |
| $N$ | 射击节点数（预测步数） | §D2.1 |
| $\Delta t$ | MPC 步长 [s] | §D2.1 |
| $Q, R, P$ | 状态/输入/终端权重矩阵 | §D2.1 |
| $f_d(x, u)$ | 离散化动力学 | §D2.1 |
| $\sigma$ | 等效扰动（L1 估计） | §D2.5 |
| $\Gamma$ | L1 适应增益 | §D2.5 |
| $\omega_c$ | L1 低通滤波器截止频率 | §D2.5 |
| $\mu_{\text{GP}}$ | GP 预测均值 | §D2.6 |

---

## §D2.1 四旋翼 OCP 问题结构 ⭐⭐

上一章 D1 给了我们四旋翼的动力学模型和 SE(3) 几何控制器——一个"给参考姿态和推力，直接算电机指令"的反馈控制器。它在悬停和慢速飞行时表现完美。但当你要求无人机高速穿越复杂航路点时，SE(3) 控制器暴露了三个根本缺陷：没有前瞻性、不处理约束、不考虑最优性。MPC 正是为了弥补这三个缺口而生的——这就是本节要建立的数学框架。

### 动机：为什么 PID/几何控制器不够

考虑一个具体场景：无人机需要以 8 m/s 沿一条 S 形赛道飞行。赛道上有一个半径 2 m 的急转弯。

用 D1 学的 SE(3) 几何控制器，你会看到什么？控制器在直线段表现完美——位置误差 < 1 cm。但进入急转弯的瞬间，误差猛增到 0.5 m——因为控制器只看**当前**误差，完全不知道 0.25 秒后要做急转弯。它没有提前减速，而是冲进弯道再被动拉回。

如果你手动在急转弯前加一个减速段呢？可以缓解问题——但这等于把**控制器本该做的优化**交给了人类。如果赛道变了、风况变了、载荷变了，你又要重新手调。

> **本质洞察**：PID/几何控制器是**反应式**（reactive）的——基于当前误差生成控制信号。MPC 是**预测式**（predictive）的——基于未来 $N$ 步的模型预测生成最优控制序列。反应式控制像"看路面开车"，预测式控制像"看远处路况提前调整"。这就是为什么人类驾驶员在弯道前提前刹车——大脑在做隐式的"MPC"。

### 如果不用 MPC 会怎样

不用 MPC，你有三条替代路径，每条都有致命缺陷：

| 替代方案 | 缺陷 | 具体表现 |
|---------|------|---------|
| PID + 前馈 | 无前瞻、无约束处理 | 急转弯冲出赛道；推力饱和无法处理 |
| SE(3) 几何控制 | 无前瞻、假设完美跟踪 | 高速时跟踪误差大；不处理电机饱和 |
| LQR（无穷时域） | 线性假设、无约束 | 大姿态角时线性化失效；推力可以为负（不物理） |

MPC 同时解决了这三个问题：**有限时域前瞻** $\to$ 提前规划弯道减速；**显式约束** $\to$ 推力和角速度不超过物理极限；**非线性模型** $\to$ 大姿态角下仍准确。代价是计算量——每步要解一个 NLP。但 acados 的 SQP-RTI 已经把这个代价压缩到了亚毫秒级。

### OCP 的标准形式

MPC 的核心是在每个控制周期反复求解一个有限时域最优控制问题（Optimal Control Problem, OCP）。对于四旋翼，标准 OCP 如下：

$$
\min_{u_0, \ldots, u_{N-1}} \quad J = \sum_{k=0}^{N-1} \left[ \|x_k - x_{\text{ref},k}\|_Q^2 + \|u_k - u_{\text{ref},k}\|_R^2 \right] + \|x_N - x_{\text{ref},N}\|_P^2
$$

$$
\text{s.t.} \quad x_{k+1} = f_d(x_k, u_k), \quad k = 0, \ldots, N-1 \quad \text{（动力学约束）}
$$

$$
u_{\min} \leq u_k \leq u_{\max}, \quad k = 0, \ldots, N-1 \quad \text{（输入约束）}
$$

$$
x_0 = x_{\text{measured}} \quad \text{（初始状态约束）}
$$

$$
\text{（可选）} \quad h(x_k, u_k) \leq 0 \quad \text{（路径约束：障碍物/走廊）}
$$

每个符号的物理含义和设计理由：

- **$J$**——代价函数。二次跟踪代价 + 终端代价。为什么是二次的？因为二次代价让 QP 子问题有闭式解（线性 KKT 系统），这是 SQP 高效的前提。
- **$Q \in \mathbb{R}^{n \times n}$**——状态权重矩阵。对角矩阵，各维度权重不同。位置权重 > 姿态权重 > 速度权重，因为最终关心的是位置精度。
- **$R \in \mathbb{R}^{m \times m}$**——输入权重矩阵。惩罚控制能量，防止电机饱和。$R$ 过小 $\to$ 控制激进、振荡；$R$ 过大 $\to$ 响应迟钝。
- **$P \in \mathbb{R}^{n \times n}$**——终端权重矩阵。近似无穷时域代价，补偿有限时域的短视性。通常取 $Q$ 的 5-20 倍，或用离散 Riccati 方程（DARE）精确计算。
- **$f_d$**——离散化动力学。连续动力学 $\dot{x} = f(x, u)$ 的 RK2/RK4 离散化。
- **$u_{\min}, u_{\max}$**——输入约束。推力下限 > 0（不能产生负推力！），上限 $\approx$ 4$g$；角速度 $\pm$6 rad/s（$\approx$ 340 deg/s）。

> **关键理解**：MPC 不是一次性求解整条轨迹，而是**滚动时域**（receding horizon）——每一步只优化 $N$ 步窗口，执行第一步控制 $u_0^*$ 后窗口前移一步，下一个测量到达后再重新优化。"模型预测控制"名称的三个关键词：**模型**（$f_d$）做**预测**（$N$ 步前看），然后做**控制**（$u_0^*$）。

> **阶段小结**：到这里我们写出了 OCP 的数学形式。接下来要填入两个关键细节：状态/输入维度选择（四旋翼特有）和代价函数的工程调参。

### 状态与控制维度选择

**状态向量** $x \in \mathbb{R}^{10}$（rpg_mpc 使用的标准定义）：

```
x = [p_x, p_y, p_z,           ← 世界坐标位置 (3)
     v_x, v_y, v_z,           ← 世界坐标速度 (3)
     q_w, q_x, q_y, q_z]      ← 姿态四元数 (4)
                               ——合计 10 维
```

**控制输入** $u \in \mathbb{R}^4$：

```
u = [f,                        ← 质量归一化总推力 (1)  [m/s^2]
     omega_x, omega_y, omega_z]← 体轴角速度 (3)       [rad/s]
                               ——合计 4 维
```

**为什么选 CTBR 而非 4 个电机转速？** 这是一个重要的设计决策，初学者常常困惑。两种选择的对比：

| 维度 | CTBR $(f, \omega)$ | 单旋翼推力 SRT $(T_1, T_2, T_3, T_4)$ |
|------|-------------------|---------------------------------------|
| 与飞控接口 | 直接匹配 PX4/Betaflight rate controller | 需要绕过内环，直接驱动电机 |
| 模型复杂度 | 无需建模电机动力学 | 需要额外 4 个电机一阶时延状态 |
| 状态维度 | 10 | 14（+4 电机状态） |
| Sim-to-Real | 鲁棒性更好（Kaufmann 等 ICRA 2022 实测） | 对电机参数敏感 |
| MPC 频率需求 | 50-200 Hz（内环由飞控 ~2 kHz 闭合） | 500+ Hz（MPC 直接闭合电机环） |

> 如果不选 CTBR 而选 SRT 会怎样？状态维度从 10 增加到 14，QP 子问题的规模增大 $\sim$40%。更关键的是，你必须精确辨识每个电机的一阶时延常数 $\tau_m$（10-50 ms），否则模型失配导致 SQP 收敛变慢甚至发散。CTBR 把这些复杂性隐藏在飞控的内环中，MPC 只需要在 50-200 Hz 的低频率下工作。

**四元数动力学**——连续时间形式：

$$
\dot{p} = v
$$

$$
\dot{v} = R(q) \begin{pmatrix} 0 \\ 0 \\ f \end{pmatrix} + \begin{pmatrix} 0 \\ 0 \\ -g \end{pmatrix}
$$

$$
\dot{q} = \frac{1}{2} q \otimes \begin{pmatrix} 0 \\ \omega_x \\ \omega_y \\ \omega_z \end{pmatrix}
$$

其中 $R(q) \cdot [0, 0, f]^\top$ 将体坐标系推力旋转到世界坐标系。四元数乘法避免了三角函数计算，这对 CasADi 的自动微分和代码生成都很友好。

读到这里你可能会问："10 维状态中有四元数的 4 个分量，但四元数有单位约束 $\|q\| = 1$，实际自由度只有 3。这不是浪费了一个维度吗？" 的确，状态空间实际维度是 9（位置 3 + 速度 3 + 姿态 SO(3) 的 3 个自由度）。使用四元数的 10 维表示是为了**避免万向锁**（gimbal lock）和**保持 ODE 的简洁性**。代价是需要在每步积分后归一化 $q \leftarrow q / \|q\|$，或在 acados 中将 $\|q\|^2 = 1$ 作为等式约束加入 OCP。

### 问题规模分析

```
射击节点 N = 20
步长 dt = 0.05 ~ 0.1 s
滚动时域 T = N * dt = 1 ~ 2 s

决策变量数：
  状态：10 × (N+1) = 210
  控制：4  × N     = 80
  合计：290

等式约束：
  动力学：10 × N = 200
  初始状态：10
  合计：210

不等式约束：
  输入上下限：2 × 4 × N = 160  (上限+下限)
  (可选) 路径约束
  合计：160+

→ 问题规模：~290 变量、~210 等式、~160 不等式
```

> **对比其他机器人 MPC**（帮助你定位四旋翼 MPC 的难度水平）：
>
> | 系统 | 状态维度 | 控制维度 | 典型 N | 总变量 | 特殊难点 |
> |------|---------|---------|--------|--------|---------|
> | **四旋翼** | 10 | 4 | 20 | ~290 | 四元数归一化 |
> | 四足（OCS2） | ~24 | ~12 | 20-30 | ~1080 | 接触切换、非光滑 |
> | 人形（MIT） | ~40+ | ~20+ | 10-15 | ~900+ | 高维、平衡约束 |
> | 自动驾驶 | ~6 | ~2 | 40-80 | ~640 | 非凸障碍物约束 |
>
> **结论**：四旋翼 MPC 是最简单的真实机器人 MPC 问题之一——非常适合入门学习 MPC 的完整技术栈。

### 代价函数的工程设计

$Q$ 和 $R$ 矩阵的选择对性能有决定性影响。这不是简单的"调参"——每个权重背后都有物理含义。

```
Q = diag(Q_pos, Q_pos, Q_pos,           ← 位置权重 [100~500]
         Q_vel, Q_vel, Q_vel,           ← 速度权重 [10~50]
         Q_att, Q_att, Q_att, Q_att)    ← 姿态权重 [50~200]

R = diag(R_thrust,                       ← 推力权重 [1~10]
         R_rate, R_rate, R_rate)         ← 角速度权重 [1~10]
```

**调参经验法则与物理直觉**：

| 规则 | 物理直觉 | 典型数值 |
|------|---------|---------|
| $Q_{\text{pos}} \gg Q_{\text{vel}}$ | 位置精度是最终目标，速度误差通过积分影响位置 | $Q_{\text{pos}} / Q_{\text{vel}} \approx 10$ |
| $Q_{\text{pos,z}} > Q_{\text{pos,xy}}$ | 垂直方向的碰撞风险更大（地面/天花板） | $Q_z / Q_{xy} \approx 2$ |
| $R$ 过小 $\to$ 激进 | 不惩罚控制能量，电机可能饱和后失控 | $R < 1$ 时危险 |
| $R$ 过大 $\to$ 迟钝 | 过度惩罚控制能量，响应像"飞在糖浆里" | $R > 50$ 时过保守 |
| $P \approx 5\text{-}20 \times Q$ | 终端代价补偿有限时域，近似无穷时域 LQR 值函数 | 或用 DARE 精确计算 |

> 如果不设终端代价 $P$ 会怎样？MPC 只看 $N$ 步以内的代价，对 $N$ 步以后的后果完全无视。在时域较短时，MPC 会出现"短视行为"——比如在时域末尾全力加速（因为之后的代价看不到了），导致实际轨迹振荡。$P$ 的物理含义是"$N$ 步以后所有未来代价的近似"，它让 MPC 在有限时域内模拟无穷时域 LQR 的行为。

### 动力学离散化

连续动力学 $\dot{x} = f(x, u)$ 需要离散化为 $x_{k+1} = f_d(x_k, u_k)$ 以供 MPC 使用。三种常用方法的精度和计算量对比：

| 方法 | 精度 | 计算量 | rpg_mpc/acados 默认 |
|------|------|--------|-------------------|
| 前向 Euler | $O(\Delta t)$ | 最低 | 不推荐 |
| RK2（Heun） | $O(\Delta t^2)$ | 中 | rpg_mpc 默认 |
| RK4 | $O(\Delta t^4)$ | 较高 | acados 默认（ERK4） |

**RK4 离散化**（acados 默认）：

$$
k_1 = f(x_k, u_k)
$$
$$
k_2 = f(x_k + \tfrac{\Delta t}{2} k_1, u_k)
$$
$$
k_3 = f(x_k + \tfrac{\Delta t}{2} k_2, u_k)
$$
$$
k_4 = f(x_k + \Delta t \cdot k_3, u_k)
$$
$$
x_{k+1} = x_k + \tfrac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)
$$

**四元数归一化**：每步积分后需要重新归一化 $q \leftarrow q / \|q\|$，否则单位四元数约束漂移。acados 支持两种处理方式：

1. **投影法**：积分后手动归一化（简单但不精确）
2. **约束法**：将 $\|q\|^2 = 1$ 作为等式约束加入 OCP（严格但增加 $N$ 个约束）

> 如果不归一化四元数会怎样？$\|q\|$ 会随积分步数缓慢漂移（每步误差 $\sim O(\Delta t^2)$）。50 步后 $\|q\| \approx 1.01$——旋转矩阵 $R(q)$ 不再正交，推力方向计算出现 ~1% 误差。看起来不大，但在 200 Hz 控制循环中，误差累积 1 秒就可能导致 5-10 cm 的位置偏差。

### acados Python 前端实现四旋翼 OCP

以下是使用 acados Python 接口定义四旋翼 OCP 的**完整可运行代码**。这段代码的作用是**验证上面所有理论**——你应该在理解了 OCP 公式、状态/输入维度、代价设计之后再读代码，而不是先读代码。

```python
"""
四旋翼 MPC 的 acados OCP 定义
状态 x = [px, py, pz, vx, vy, vz, qw, qx, qy, qz] ∈ R^10
输入 u = [thrust, wx, wy, wz] ∈ R^4

依赖：pip install acados_template casadi numpy
"""

import numpy as np
from casadi import SX, vertcat
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# ──────────────────────────────────────────────────────────
# 1. 定义 CasADi 符号动力学模型
# ──────────────────────────────────────────────────────────
def quadrotor_model() -> AcadosModel:
    """
    四旋翼刚体动力学（CTBR 动作空间）

    动力学:
      dp/dt = v
      dv/dt = R(q) @ [0, 0, f]^T + [0, 0, -g]^T
      dq/dt = 0.5 * q ⊗ [0, omega]
    """
    model = AcadosModel()
    model.name = "quadrotor"

    # 符号状态变量
    px, py, pz = SX.sym("px"), SX.sym("py"), SX.sym("pz")
    vx, vy, vz = SX.sym("vx"), SX.sym("vy"), SX.sym("vz")
    qw, qx, qy, qz = SX.sym("qw"), SX.sym("qx"), SX.sym("qy"), SX.sym("qz")
    x = vertcat(px, py, pz, vx, vy, vz, qw, qx, qy, qz)

    # 符号控制输入
    f_thrust = SX.sym("f_thrust")       # 质量归一化总推力 [m/s^2]
    wx, wy, wz = SX.sym("wx"), SX.sym("wy"), SX.sym("wz")
    u = vertcat(f_thrust, wx, wy, wz)

    g = 9.8066  # 重力加速度

    # 四元数旋转：R(q) @ v_body
    def quat_rotate(q_w, q_x, q_y, q_z, vec):
        """Rodriguez 旋转公式的四元数形式——避免显式构造 R 矩阵"""
        t = 2.0 * vertcat(
            q_y * vec[2] - q_z * vec[1],
            q_z * vec[0] - q_x * vec[2],
            q_x * vec[1] - q_y * vec[0])
        return vec + q_w * t + vertcat(
            q_y * t[2] - q_z * t[1],
            q_z * t[0] - q_x * t[2],
            q_x * t[1] - q_y * t[0])

    # 推力在世界坐标系的投影
    thrust_world = quat_rotate(qw, qx, qy, qz, vertcat(0, 0, f_thrust))

    # 四元数动力学：dq/dt = 0.5 * q ⊗ [0, omega]
    dqw = 0.5 * (-qx * wx - qy * wy - qz * wz)
    dqx = 0.5 * ( qw * wx + qy * wz - qz * wy)
    dqy = 0.5 * ( qw * wy + qz * wx - qx * wz)
    dqz = 0.5 * ( qw * wz + qx * wy - qy * wx)

    # 显式 ODE: dx/dt = f_expl(x, u)
    f_expl = vertcat(
        vx, vy, vz,                             # dp/dt = v
        thrust_world[0],                         # dvx/dt
        thrust_world[1],                         # dvy/dt
        thrust_world[2] - g,                     # dvz/dt
        dqw, dqx, dqy, dqz)                     # dq/dt

    model.x = x
    model.u = u
    model.f_expl_expr = f_expl
    model.x_dot = SX.sym("x_dot", 10)
    model.f_impl_expr = model.x_dot - f_expl

    return model


# ──────────────────────────────────────────────────────────
# 2. 配置 OCP
# ──────────────────────────────────────────────────────────
def create_ocp_solver(N: int = 20, dt: float = 0.05) -> AcadosOcpSolver:
    ocp = AcadosOcp()
    ocp.model = quadrotor_model()
    nx, nu = 10, 4

    # ── 代价函数（LINEAR_LS 类型）──
    Q_diag = [200., 200., 400., 20., 20., 40., 100., 100., 100., 50.]
    R_diag = [5., 5., 5., 5.]
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    Vx = np.zeros((nx + nu, nx)); Vx[:nx, :nx] = np.eye(nx)
    Vu = np.zeros((nx + nu, nu)); Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vx = Vx; ocp.cost.Vu = Vu
    ocp.cost.Vx_e = np.eye(nx)

    W = np.block([[Q, np.zeros((nx, nu))],
                  [np.zeros((nu, nx)), R]])
    ocp.cost.W = W
    ocp.cost.W_e = Q * 10.0  # 终端权重 = 10×Q

    hover = np.array([0., 0., 1., 0., 0., 0., 1., 0., 0., 0.])
    hover_u = np.array([9.8066, 0., 0., 0.])
    ocp.cost.yref = np.concatenate([hover, hover_u])
    ocp.cost.yref_e = hover

    # ── 输入约束 ──
    ocp.constraints.lbu = np.array([0.5, -6.0, -6.0, -6.0])
    ocp.constraints.ubu = np.array([4 * 9.8066, 6.0, 6.0, 6.0])
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.x0 = hover

    # ── 求解器配置 ──
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt
    ocp.solver_options.integrator_type = "ERK"   # 显式 Runge-Kutta
    ocp.solver_options.num_stages = 4            # RK4
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # 实时迭代
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hpipm_mode = "SPEED"

    return AcadosOcpSolver(ocp, json_file="quadrotor_ocp.json")
```

> **代码要点解析**：
>
> 1. **CasADi 符号表达式**：模型用 `SX.sym` 定义符号变量，`vertcat` 拼接。CasADi 会自动推导雅可比矩阵 $\partial f / \partial x$ 和 $\partial f / \partial u$，供 SQP 线性化使用——你无需手写雅可比。
> 2. **`SQP_RTI`**：指定实时迭代模式——每次只做一步 SQP 迭代，大幅降低延迟。后面 §D2.4 会详细解释为什么一步就够。
> 3. **`PARTIAL_CONDENSING_HPIPM`**：利用 OCP 的块三对角 KKT 结构做部分凝聚后用 HPIPM 求解——后面 §D2.3 会详细解释。
> 4. **推力下限 0.5**（而非 0）：实际电机有最低转速，归一化推力不能降到 0。

### ⚠️ 常见陷阱

**陷阱 1（编程陷阱）：四元数未归一化导致漂移**

```
错误做法：积分后不归一化四元数
现象：飞行 5-10 秒后，姿态估计开始"飘"，位置误差逐渐增大
根本原因：RK4 积分不保持 ||q|| = 1 的不变量，每步 ||q|| 漂移 O(dt^2)
正确做法：
  方案 A：每步后 q = q / ||q||（简单，推荐）
  方案 B：在 acados 中加等式约束 ||q||^2 = 1（严格但增加约束）
```

**陷阱 2（概念误区）：认为 N 越大越好**

```
错误理解：预测步数 N 越多、MPC 看得越远，性能就越好
现象：N=40 的 MPC 求解时间 3 ms，超出控制周期，反而不如 N=20 的 0.5 ms
根本原因：N 增大 → QP 规模线性增长 → 求解时间增加。但 MPC 的实时性要求
         求解时间 << 控制周期。N 应该让时域覆盖"关键动态"即可——四旋翼
         的主要动态时间常数 ~0.3-0.5 s，N=20 × dt=0.05 = 1s 已经足够
正确做法：选 N 使 T = N*dt 覆盖 2-3 倍系统时间常数，同时确保求解时间 < dt/2
```

**陷阱 3（思维陷阱）：忽视终端代价 P 的重要性**

```
错误理解："终端代价只是最后一步的权重，影响不大"
现象：去掉 P 后，MPC 在时域末尾出现"加速-急刹"振荡
根本原因：无终端代价时 MPC 对 N 步之后的后果完全无视——它会在最后几步全力
         加速（因为 N 步之后的代价不计入）。P 的本质是"未来所有代价的近似"
正确做法：P = DARE(Q, R, A, B) 精确计算，或取 P = 10*Q 作为简单近似
```

### 练习

**练习 1（实现题）**：运行上面的 acados 代码，实现一个从 $(0,0,0)$ 飞到 $(2,2,2)$ 的点到点 MPC 闭环仿真。记录并绘制：(a) 位置轨迹 3D 图；(b) 推力 $f(t)$ 和角速度 $\omega(t)$；(c) 每步求解时间。对比 $N=10$ 和 $N=40$ 的差异——你会发现 $N=10$ 在目标点附近有振荡（短视），$N=40$ 更平滑但每步求解时间增加 ~3 倍。

**练习 2（设计题）**：在 OCP 中加入一个路径约束 $p_z \geq 0.5$（不允许飞低于 0.5 m）。提示：使用 `ocp.constraints.lh` 和 `ocp.constraints.uh` 定义非线性路径约束，或使用 `lbx`/`ubx` 对状态的第 3 个分量加下界。观察 MPC 如何在接近地面时主动爬升。

---

上一节建立了四旋翼 MPC 的数学框架——OCP 的标准形式、状态/输入维度、代价函数设计。但写出 OCP 只是第一步。关键问题是：**怎么在 5 ms 内解出来？** 这正是下面两节的主题——先看一个教科书级的工程实现（rpg_mpc），再看现代求解器栈（acados）如何把延迟压缩到亚毫秒。

## §D2.2 rpg_mpc：三层 C++ 架构 ⭐⭐

### 动机

你用 acados Python 接口实现了四旋翼 MPC——它跑在你的笔记本上，每步 0.5 ms，完美。但现在要部署到真机：ROS 节点需要订阅状态估计、发布控制指令、与飞控通信。你需要一个 C++ 的 MPC 封装层，它既要高性能（零拷贝），又要与 ROS 接口整洁分离。

rpg_mpc（UZH RPG 实验室）是这个问题的教科书级答案。它的三层架构将**代码生成的 C 求解器**与**ROS 接口**完全解耦，中间用 `Eigen::Map` 零拷贝桥接。这个架构模式直接适用于所有 C 语言求解器的 C++ 集成。

### 如果不学这个会怎样

你可能把所有东西写在一个 ROS 节点里——ACADO 的 C 数组操作、Eigen 矩阵运算、ROS 消息转换混在同一个文件中。结果是：(a) 换求解器时要改遍整个节点；(b) C 数组的列优先索引和 Eigen 的行列访问混用，导致隐蔽的数据布局 bug；(c) 每次 MPC 迭代做 2-3 次不必要的 memcpy，在 200 Hz 时浪费 5-10 $\mu$s。

### 架构总览

```
┌──────────────────────────────────────────────────────┐
│ 层 3: mpc_controller.cpp                             │
│   ROS 接口层                                          │
│   - 订阅：/state_estimate (nav_msgs/Odometry)        │
│   - 订阅：/reference_trajectory                       │
│   - 发布：/control_command (推力 + 体率)               │
│   - 定时器回调 → 调用层 2                              │
├──────────────────────────────────────────────────────┤
│ 层 2: mpc_wrapper.h                                  │
│   Eigen::Map 零拷贝封装                                │
│   - 将 ACADO 的 C 数组 ↔ Eigen 矩阵双向映射            │
│   - 填充参考轨迹 → ACADO 数组                          │
│   - 读取求解结果 → Eigen 向量                          │
├──────────────────────────────────────────────────────┤
│ 层 1: mpc_solver/ (ACADO 自动生成)                    │
│   纯 C 文件——acado_solver.c / acado_integrator.c     │
│   所有数据存储在列优先 C 数组中                         │
└──────────────────────────────────────────────────────┘
```

> **本质洞察**：三层架构的设计原则是"每一层只知道相邻层的接口"。层 1（C 求解器）不知道 Eigen 的存在；层 2（Eigen::Map 封装）不知道 ROS 的存在；层 3（ROS 节点）不知道 ACADO 数组的内存布局。这让你可以独立替换任意一层——比如把层 1 从 ACADO 换成 acados，只需改层 2 的 Map 映射，层 3 完全不动。

### Eigen::Map 零拷贝——rpg_mpc 的核心 C++ 技巧

这是整个仓库**最值得学习的工程模式**。ACADO 代码生成器输出列优先的 C 数组，直接操作这些数组非常不便且容易出错。`mpc_wrapper.h` 用 `Eigen::Map` 将它们包装为 Eigen 矩阵，**不做任何内存拷贝**：

```cpp
// ─── ACADO 生成的全局变量（acado_common.h）─────────────
static constexpr int kStateSize = 10;     // p(3) + v(3) + q(4)
static constexpr int kInputSize = 4;      // f(1) + omega(3)
static constexpr int kSamples   = 20;     // 射击节点数 N

// ─── Eigen::Map 零拷贝封装（mpc_wrapper.h）─────────────

// 包装状态轨迹——零拷贝！
Eigen::Map<Eigen::Matrix<double, kStateSize, kSamples + 1,
                         Eigen::ColMajor>>
    predicted_states(acadoVariables.x);
    // predicted_states 与 acadoVariables.x 共享同一块内存
    // 修改 predicted_states 等于直接修改 acadoVariables.x

// 此后可用 Eigen 惯用法操作：
Eigen::Vector3d position_k = predicted_states.col(k).segment<3>(0);
Eigen::Quaterniond attitude_k(
    predicted_states(6, k), predicted_states(7, k),
    predicted_states(8, k), predicted_states(9, k));
```

**为什么零拷贝如此重要？** 这不是微优化——在嵌入式 MPC 场景下，这是必需品。

```
传统做法（有拷贝）：
  C数组 → memcpy → Eigen::Matrix → 运算 → memcpy → C数组
  每次 MPC 迭代：2 × 290 × 8 = 4.6 KB 拷贝
  200 Hz 控制频率：920 KB/s 无意义的内存搬运 → 缓存污染

Eigen::Map 做法（零拷贝）：
  C数组 ← Eigen::Map 直接操作 → 零额外分配
  Map 只是一个指针 + 维度信息，sizeof(Map) ≈ 16 bytes
  没有 new/delete，没有 memcpy，没有缓存污染
```

这个类比有助于理解：`Eigen::Map` 像一副"眼镜"——戴上眼镜后你看到的是 Eigen 矩阵的接口，但底下的数据纹丝未动。这个类比**不像**的地方是：Map 不是只读的，通过 Map 的写入会直接修改底层 C 数组。

> **可迁移性**：这个模式直接适用于所有代码生成求解器的 C++ 集成：
> - acados C 接口 $\to$ `Eigen::Map` 包装 `ocp_nlp_out_get()`
> - OSQP C 接口 $\to$ `Eigen::Map` 包装 `work->solution->x`
> - Ceres Solver $\to$ `Eigen::Map` 在 cost function 中包装参数块指针

### mpc_wrapper 的关键方法

```cpp
class MpcWrapper {
public:
    // ── 设置参考轨迹（Eigen → ACADO C 数组，零拷贝）──
    bool setReferencePose(const Eigen::Vector3d& position,
                          const Eigen::Quaterniond& orientation,
                          int node_index) {
        Eigen::Map<Eigen::Matrix<double, kRefSize, 1>>
            ref_node(acadoVariables.y + node_index * kRefSize);
        ref_node.segment<3>(0) = position;
        ref_node.segment<3>(3) = Eigen::Vector3d::Zero();  // 速度参考=0
        ref_node(6) = orientation.w();
        // ... 其余姿态和输入参考
        return true;
    }

    // ── 求解并获取结果 ──
    bool solve(const Eigen::Matrix<double, kStateSize, 1>& x0) {
        // 设置初始状态（零拷贝）
        Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>
            initial_state(acadoVariables.x0);
        initial_state = x0;  // Eigen 赋值 → 直接写入 C 数组

        // ACADO 求解（纯 C 调用）
        acado_preparationStep();   // 准备阶段：线性化 + 凝聚
        acado_feedbackStep();      // 反馈阶段：QP 求解

        return acado_getKKT() < 1e-3;  // KKT 残差检查
    }

    // ── 获取最优控制 u_0*（零拷贝读取）──
    Eigen::Vector4d getOptimalInput() const {
        return Eigen::Map<const Eigen::Matrix<double, kInputSize, 1>>(
            acadoVariables.u);
    }
};
```

### ⚠️ 常见陷阱

**陷阱 1（编程陷阱）：Eigen::Map 的生命周期陷阱**

```
错误做法：Map 指向栈上临时变量
  Eigen::Map<Eigen::Vector4d> get_u() {
      double temp[4] = {1,2,3,4};
      return Eigen::Map<Eigen::Vector4d>(temp);  // 危险！
  }   // temp 在栈上，函数返回后 Map 指向无效内存
现象：有时结果正确（内存还没被覆写），有时 segfault 或垃圾值
根本原因：Map 不拥有内存，底层数据的生命周期必须 >= Map 的使用周期
正确做法：确保 Map 指向的内存是全局/堆/足够长命的栈上数据
```

**陷阱 2（编程陷阱）：行优先 vs 列优先不匹配**

```
错误做法：ACADO 数组是列优先，但 Map 指定了 RowMajor
现象：数据看起来"转置"了——position_k 取到的是第 k 个状态维度而非第 k 步
根本原因：C/Fortran 风格数组通常是列优先（column-major），Eigen 默认也是
         但如果底层数据来自 row-major 的系统（如 Python NumPy C-order），
         必须在 Map 模板参数中指定 Eigen::RowMajor
正确做法：先确认底层数据的存储顺序，再在 Map 中匹配
```

**陷阱 3（概念误区）：以为 Map 会复制数据**

```
错误理解："Eigen::Map 构造时会复制一份数据"
现象：通过 Map 修改数据后，原始 C 数组没变（实际上这不会发生，但误解会
     导致你写出多余的 memcpy）
根本原因：Map 的全部意义就是"不复制"。如果你需要独立副本，用 Matrix = Map
正确做法：理解 Map 是引用语义（像 C++ 引用），Matrix 是值语义（像 C++ 值）
```

### 练习

**练习 1（精读题）**：克隆 `uzh-rpg/rpg_mpc`，精读 `mpc_wrapper.h`（约 300 行）。标注：(a) 哪些 ACADO 数组被 Map 包装；(b) 参考轨迹如何从 Eigen 写入 ACADO；(c) 求解结果如何从 ACADO 读出为 Eigen。

**练习 2（实现题）**：写一个最小化示例——模拟 ACADO 风格的 C 数组，用 `Eigen::Map` 包装，进行矩阵运算，然后验证原始 C 数组被修改了（证明零拷贝）。用 `std::chrono` 对比 Map 操作和 Matrix 拷贝操作的时间差。

---

rpg_mpc 展示了如何把代码生成求解器封装成干净的 C++ 接口。但它的底层 QP 求解器（qpOASES）是通用活性集方法，没有利用 OCP 的特殊结构。下一节介绍的 acados 用结构化求解器（HPIPM）和小矩阵优化（BLASFEO）将性能提升了一个数量级。

## §D2.3 acados：现代无人机 MPC 的默认求解器 ⭐⭐⭐

### 动机

你用 rpg_mpc（ACADO + qpOASES）实现了四旋翼 MPC，每步求解 ~2 ms。够快了吗？50 Hz 控制频率下够了。但如果你要做 200 Hz 高速竞速、或在计算能力有限的 Pixhawk STM32 上运行呢？你需要把延迟压缩到 < 0.5 ms。

acados 比 ACADO + qpOASES 快的原因不是"算法更好"——SQP-RTI 的基本框架相同。关键区别在**底层实现**：HPIPM 利用了 OCP 的块三对角 KKT 结构做 Riccati 递推（而非通用 QP），BLASFEO 对 10$\times$10 量级的小矩阵做了硬件级优化（而非调用 OpenBLAS 的通用 dgemm）。

### ACADO vs acados：本质差异

```
┌──────────────────────────────────────────────────────┐
│              ACADO Toolkit (2011-2020)                 │
│                                                       │
│  C++ 前端 → 代码生成器 → 纯 C 代码                      │
│                         ↓                             │
│                    qpOASES（活性集，通用 QP）            │
│                                                       │
│  局限：                                                │
│  - 改模型需重新生成编译（无热更新）                       │
│  - qpOASES 不利用 OCP 的块三对角结构                    │
│  - 矩阵运算用标准 BLAS（对小矩阵低效）                   │
└──────────────────────────────────────────────────────┘
        ↓ 演进
┌──────────────────────────────────────────────────────┐
│              acados (2020+)                            │
│                                                       │
│  Python/MATLAB 前端 → CasADi 符号模型                  │
│          ↓               ↓              ↓             │
│    C 运行时        SQP/SQP-RTI      QP 后端            │
│    (热更新参数)                     ├─ HPIPM (默认)    │
│                                    ├─ qpOASES         │
│                                    ├─ DAQP            │
│                                    └─ OSQP            │
│          ↓                                            │
│    BLASFEO（小矩阵优化线性代数）                         │
│                                                       │
│  优势：                                                │
│  - 改参数不需重编译（热更新 Q, R, 约束边界）              │
│  - HPIPM 利用块三对角做 Riccati 递推                    │
│  - BLASFEO 对 4-64 维小矩阵比 LAPACK 快 2-5 倍         │
│  - Python 前端用 CasADi 自动微分——无需手写雅可比         │
└──────────────────────────────────────────────────────┘
```

如果不从 ACADO 升级到 acados 会怎样？你失去了三个关键能力：(a) 在线调参——ACADO 的 C 代码是"一次性"的，改任何参数都要重新代码生成 + 编译，在真机调试中意味着每次调 Q/R 都要等 30-60 秒的编译；(b) 结构化 QP——qpOASES 把 OCP 的 QP 当作通用稠密 QP 求解，完全浪费了块三对角结构，变量多于 ~100 时效率急剧下降；(c) 小矩阵优化——标准 BLAS 的 dgemm 对 10$\times$10 矩阵的利用率只有 ~20% 峰值 FLOPS。

### BLASFEO——为什么小矩阵需要专门的线性代数库

这是一个容易被忽略但影响巨大的底层优化。为什么标准 BLAS 对小矩阵不行？

标准 BLAS/LAPACK（如 OpenBLAS、MKL）针对大矩阵（$n > 100$）优化：寄存器分块、L2/L3 缓存利用、多线程。但四旋翼 MPC 的矩阵尺寸只有 10$\times$10、10$\times$4、14$\times$14——**太小了，BLAS 的调用开销比实际计算还大**。

```
传统 BLAS（对 10×10 矩阵）：
  调用 dgemm() → 函数指针间接调用 → 维度检查 → 分块策略选择 → 实际计算
  → 开销可能占 50% 的总时间，实际计算只用了一半

BLASFEO（对 10×10 矩阵）：
  面板格式存储 → 直接跳转到预编译的固定大小内核
  → 利用率 ~80% 峰值 FLOPS（对比 OpenBLAS 的 ~20%）
  → 快 2-5 倍

性能数据（Frison 等，ACM TOMS 2018）：
  n=12 dgemm: BLASFEO ~80% 峰值 vs OpenBLAS ~20% 峰值
```

BLASFEO 的核心思想是**面板格式**（panel-major）存储：将矩阵按 4-8 行分块，每块内列优先，对齐到 SIMD 宽度（AVX: 4 double = 32 bytes）。这种布局让 SIMD 指令可以直接加载连续的数据，避免了散列访问。

> 这和 SLAM 中的优化有什么关系？Ceres Solver 中对小型 `Jet<double, N>` 自动微分也有类似优化——固定大小的 Jet 利用模板元编程在编译期展开循环。思路相同：**当问题规模已知且小时，通用库的开销不可忽略**。

### HPIPM——利用 OCP 结构的 Riccati 递推

HPIPM（High-Performance Interior Point Method）是 acados 的默认 QP 求解器。它的关键优势是利用了 OCP 特有的**块三对角 KKT 结构**。

MPC 的 QP 子问题的 KKT 矩阵具有以下结构：

```
┌                                               ┐
│ H_0  A_0^T                                    │
│ A_0  -H_1  A_1^T                              │
│       A_1  -H_2  A_2^T                        │
│             ...   ...   ...                   │
│                   A_{N-2}  -H_{N-1}  A_{N-1}^T│
│                             A_{N-1}  -H_N     │
└                                               ┘

其中：
  H_k = blkdiag(Q_k, R_k)     — 代价 Hessian（10×10 + 4×4）
  A_k = [∂f/∂x_k | ∂f/∂u_k]  — 动力学雅可比（10×14）
```

这是一个**块三对角结构**。通用 QP 求解器（如 qpOASES）看不到这个结构，按稠密矩阵处理——对 290 个变量做 $O(n^3) = O(290^3) \approx 2.4 \times 10^7$ 的运算。HPIPM 的 Riccati 递推利用了块三对角结构，复杂度降为 $O(N \cdot (n+m)^3) = O(20 \times 14^3) \approx 55000$——快了 **400 倍**。

**Riccati 递推**（后向传播 + 前向恢复）：

```
后向传播（backward pass）：从 k=N 到 k=0
  P_N = Q_N
  for k = N-1, ..., 0:
    K_k = (R_k + B_k^T P_{k+1} B_k)^{-1} B_k^T P_{k+1} A_k
    P_k = Q_k + A_k^T P_{k+1} A_k - A_k^T P_{k+1} B_k K_k

前向恢复（forward pass）：从 k=0 到 k=N-1
  for k = 0, ..., N-1:
    δu_k = -K_k δx_k + ...
    δx_{k+1} = A_k δx_k + B_k δu_k + ...
```

> **本质洞察：HPIPM Riccati 与 SLAM 信息滤波是同一件事**。HPIPM 的后向 Riccati 递推在数学上等价于因子图的消息传递——从最后一个节点向第一个节点传播信息矩阵。iSAM2 的 Bayes Tree 变量消元在稀疏因子图上做同样的运算。区别在于拓扑：OCP 是线性链（每个状态只连接前后两个），因子图可以有更复杂的拓扑。**学过 SLAM 非线性优化的工程师已经懂了 HPIPM 80% 的数学**。

### 部分凝聚（Partial Condensing）

OCP 的 QP 有两种极端形式：

| 形式 | 变量数 | KKT 结构 | 求解方法 | 适合 |
|------|--------|---------|---------|------|
| 完全稀疏（Multi-shooting） | $N(n+m) \approx 290$ | 块三对角，大但稀疏 | Riccati 递推 $O(Nn^3)$ | $n$ 较小时 |
| 完全凝聚（Single-shooting） | $Nm = 80$ | 稠密 $80 \times 80$ | 稠密 QP $O((Nm)^3)$ | $m$ 较小时 |
| 部分凝聚 | $N_{\text{pc}}(n+m)$ | 中间形态 | 平衡两者优势 | 一般情况 |

部分凝聚将 $N$ 个射击节点合并为 $N_{\text{pc}} < N$ 个超节点（例如 $N=20 \to N_{\text{pc}}=5$），经验最优在 $N_{\text{pc}} \approx \sqrt{N}$ 附近。acados 的配置极其简洁：

```python
ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # 部分凝聚
# 或完全凝聚：
ocp.solver_options.qp_solver = "FULL_CONDENSING_DAQP"
```

### ⚠️ 常见陷阱

**陷阱 1（编程陷阱）：acados 代码生成目录冲突**

```
错误做法：在同一目录下运行多个不同模型名的 acados 代码生成
现象：第二个模型生成后覆盖了第一个的 C 代码，导致链接错误
根本原因：acados 默认将生成的代码放在 model.name 对应的子目录，
         但如果 json_file 路径相同会冲突
正确做法：每个模型使用不同的 json_file 路径和 code_export_directory
```

**陷阱 2（概念误区）：以为 HPIPM 只能用于线性 MPC**

```
错误理解：HPIPM 是 QP 求解器，所以只能用于线性系统的 MPC
现象：不敢在非线性四旋翼 MPC 中使用 HPIPM
根本原因：SQP 在每次迭代中将非线性 OCP 线性化为 QP 子问题——HPIPM 解的
         是这个线性化后的 QP，不是原始非线性问题。HPIPM 完全适用于 NMPC
正确做法：acados 的 SQP/SQP-RTI + HPIPM 组合是四旋翼 NMPC 的标准配置
```

### 练习

**练习 1（对比实验）**：在同一个四旋翼 OCP 上分别使用 `PARTIAL_CONDENSING_HPIPM` 和 `FULL_CONDENSING_DAQP`，记录求解时间。预期 HPIPM 在 $N \geq 15$ 时更快。

**练习 2（思考题）**：画出 HPIPM 的 Riccati 递推和 iSAM2 的 Bayes Tree 变量消元的对应关系图。提示：OCP 的线性链拓扑对应 Bayes Tree 退化为线性链的特殊情况。

---

acados 的三层栈（CasADi 自动微分 + HPIPM 结构化 QP + BLASFEO 小矩阵）解释了"硬件层面为什么快"。但还有一个算法层面的问题：标准 SQP 需要多次迭代（3-10 次）才能收敛，即使每次迭代 0.1 ms，总延迟仍达 0.3-1 ms。SQP-RTI 用一步迭代就够——下一节解释为什么。

## §D2.4 SQP-RTI 算法深入 ⭐⭐⭐

### 动机

你的 acados 求解器每次 SQP 迭代 0.1 ms，但标准 SQP 需要 5 次迭代才能收敛——总延迟 0.5 ms。在 200 Hz 控制频率下，0.5 ms 的求解时间占了控制周期的 10%，勉强可以接受。但测量到达后从"读到状态"到"发出控制"的**反馈延迟**也是 0.5 ms——这意味着控制器基于 0.5 ms 前的状态做决策。在 20 m/s 的飞行速度下，0.5 ms 对应 1 cm 的位置预测误差。

能不能把反馈延迟压缩到 0.1 ms？这就是 SQP-RTI 的核心目标。

### 从 SQP 到 Real-Time Iteration

**SQP 的基本思想**：对于 NLP $\min_z f(z) \text{ s.t. } g(z) = 0$，在每次迭代中将其线性化为 QP：

$$
\min_{\delta z} \quad \tfrac{1}{2} \delta z^\top H_i \delta z + \nabla f(z_i)^\top \delta z
$$

$$
\text{s.t.} \quad \nabla g(z_i)^\top \delta z + g(z_i) = 0
$$

求解得 $\delta z^*$，然后更新 $z_{i+1} = z_i + \alpha \delta z^*$。标准 SQP 重复迭代直到 KKT 残差 < 阈值。

**RTI 的关键洞察**：在 MPC 的滚动时域场景下，连续两步的 OCP 只是初始状态 $x_0$ 变了一点（因为系统只演化了 $\Delta t$）。这意味着上一步的解 $z^*_{k-1}$ 已经是本步 OCP 的**极好的初始猜测**。在这个初始猜测附近，**一步 SQP 迭代就足以产生满意的控制精度**。

```
标准 SQP 每步迭代数：3-10 次  → 总延迟 0.3-1.0 ms
RTI 每步迭代数：       1 次   → 总延迟 ~0.1 ms

为什么 1 次就够？因为暖启动——上一步的解只差 O(dt)
```

### RTI 的"准备-反馈"流水线

RTI 的精髓在于将单次 SQP 迭代分为两个阶段，**流水线化**以最小化反馈延迟：

```
时间线：
  ─────────┬────────────────────────┬───────────┬─────────
           t_{k-1}                              t_k
           │                                    │
           │   ┌──────────────────────┐         │
           │   │  准备阶段 (Preparation)│       │
           │   │  - 积分动力学 f(x,u)  │       │
           │   │  - 计算雅可比 A_k,B_k │       │
           │   │  - 凝聚 QP 矩阵      │       │
           │   └──────────────────────┘         │
           │                ↓                   │
           │        [准备完毕，等待测量]          │
           │                                    │
           │                            ┌───────┴─────┐
           │                            │ 反馈阶段     │
           │                            │ - 更新 x_0   │
           │                            │ - 求解 QP    │
           │                            │ - 读取 u_0*  │
           │                            └───────┬─────┘
           │                                    │
           │                               [发布 u_0*]
           └────────────────────────────────────┘
              反馈延迟 = 反馈阶段耗时 ≈ 0.1-0.5 ms
              （而非整个 SQP 迭代的 0.5-1.0 ms）
```

> **本质洞察**：准备阶段的计算量占 SQP 迭代的 70-90%（积分 + 线性化 + 凝聚），但它不依赖最新测量——可以基于上一步预测提前做。反馈阶段只做 QP 求解 + 读结果，利用**已经准备好的 QP 数据** + 最新 $x_0$。这就像餐厅的"预炒+出锅"模式：食材提前切好配好（准备），客人到了只需翻炒 30 秒出锅（反馈）。这个类比**不像**的地方是：RTI 的准备阶段是基于预测状态的近似，而餐厅的预处理是精确的。

**RTI 的数学保证**（Diehl 等, Automatica 2005）：

如果系统轨迹变化足够平滑（OCP 的解连续依赖于初始状态），RTI 的单次迭代产生的控制误差是 $O(\Delta t^2)$——因为初始猜测与最优解的差距只有 $O(\Delta t)$，一步 Newton/SQP 迭代的收敛速率是二次的。

### Advanced-Step RTI（AS-RTI）

2024 年 Frey 等提出的 AS-RTI（arXiv:2403.07101）进一步优化了 RTI：在准备阶段不仅做线性化和凝聚，还做一次基于预测状态的 QP 预求解。反馈阶段只需要做增量修正（因为 $x_0 \approx x_{\text{pred}}$），进一步将反馈延迟从 ~0.5 ms 压缩到 ~0.1 ms。

```
标准 RTI：准备 = 线性化 + 凝聚
         反馈 = 更新 x_0 + 求解 QP（~0.5 ms）

AS-RTI：  准备 = 线性化 + 凝聚 + QP 预求解（基于 x_pred）
         反馈 = x_0 修正 + QP 增量修正（~0.1 ms）
```

### ⚠️ 常见陷阱

**陷阱 1（概念误区）：认为 RTI 精度比标准 SQP 差很多**

```
错误理解：RTI 只做一步迭代，精度肯定比完全收敛的 SQP 差很多
现象：在标称条件下 RTI 和 SQP 的控制性能几乎相同
根本原因：MPC 的滚动时域本身就在不断"修正"——即使单步 OCP 的解不完全最优，
         下一步的重新优化会补偿。RTI 的控制误差是 O(dt^2) ≈ 2.5×10^{-3}
         （dt=0.05），对位置跟踪的影响 < 0.1 mm
正确做法：默认使用 SQP_RTI；仅在动力学强非线性（如侧翻恢复）时考虑 SQP
```

**陷阱 2（编程陷阱）：忘记暖启动导致 RTI 发散**

```
错误做法：每步 MPC 不用上一步的解初始化，从零开始
现象：RTI 的一步迭代不够收敛，控制输出跳变，系统振荡
根本原因：RTI 依赖"上一步的解是好的初始猜测"这一前提。冷启动时初始猜测
         可能离最优解很远，一步 SQP 不够
正确做法：
  solver.set(k, "x", x_prev[k])  # 用上一步的预测轨迹暖启动
  solver.set(k, "u", u_prev[k])
```

### 练习

**练习 1（对比实验）**：在同一个四旋翼 OCP 上分别使用 `SQP_RTI`（1 次迭代）和 `SQP`（max_iter=10），记录：(a) 每步求解时间；(b) 位置跟踪 RMSE。预期 SQP_RTI 快 5-10 倍，但跟踪精度差 < 1%。

**练习 2（思考题）**：在什么情况下 RTI 的"一步就够"假设会失效？提示：初始状态突变（碰撞后状态跳变）、参考轨迹急变（急刹车）、约束突然激活（进入禁飞区）。

---

到这里我们已经建立了四旋翼 NMPC 的完整技术栈：OCP 公式（§D2.1）$\to$ rpg_mpc 工程架构（§D2.2）$\to$ acados 求解器栈（§D2.3）$\to$ SQP-RTI 算法（§D2.4）。MPC 在名义条件下表现完美。但真实世界不是名义条件——有风、有载荷变化、有模型误差。接下来三节讨论三种让 MPC 在非名义条件下保持鲁棒的方法。

## §D2.5 L1 自适应控制——叠加在 MPC 之上的扰动补偿 ⭐⭐⭐

### 动机

你的四旋翼 MPC 在室内无风条件下跟踪精度 5 mm。然后发生了以下任何一件事：

| 场景 | 扰动来源 | 误差影响 |
|------|---------|---------|
| 挂载物资 | 质量 $m$ 和惯量 $J$ 变化 | $\Delta m \sim 60\% m_0$ |
| 室外侧风 | 外力 $f_{\text{wind}}$ 未建模 | ~5 N 持续力 |
| 近地飞行 | 地面效应使推力增大 ~30% | ~3 N 推力偏差 |
| 电池消耗 | 有效推力随电压线性下降 | ~10-15% 一个架次内 |
| 高速飞行 | 寄生阻力、诱导阻力 | ~2-5 N @ 10 m/s |

这些扰动让 MPC 的名义模型 $f_{\text{nominal}}(x, u)$ 与真实动力学之间产生了**模型失配**。MPC 不知道模型错了——它基于错误的模型做预测，得到错误的最优控制，跟踪误差急剧增大。

### 如果不做自适应补偿会怎样

你有三条替代路径，每条都有明显缺陷：

| 方案 | 原理 | 缺陷 |
|------|------|------|
| 鲁棒 MPC | 假设扰动在已知集合内，最小化最坏情况代价 | 在线计算重（min-max 问题）、过度保守 |
| 系统辨识 | 在线估计模型参数（$m$、$J$、$k_f$） | 参数化模型可能不够灵活（风力无法用质量变化描述） |
| 重新训练 | 收集扰动条件下的数据，重新训练残差模型 | 每种新扰动都要重新收集数据——不可扩展 |

L1 自适应控制提供了第四条路径：**不改模型、不学残差、不重新训练——直接在线估计并补偿等效扰动**。它的核心优势是"零训练、强泛化、极低计算开销"。

### L1 自适应的三个组件

L1 自适应控制（Hovakimyan & Cao, 2010）由三个模块组成，每个模块有明确的职责：

```
┌─────────────────────────────────────────────────────────┐
│  真实系统: ẋ = f(x, u + σ_true)  ← σ_true 是未知扰动   │
│       ↓ x (测量)                                        │
│  ┌──────────────┐                                      │
│  │  状态预测器   │ x̂̇ = f(x̂, u) + σ̂                     │
│  │  (State      │ ← 用名义模型 + 估计扰动预测状态        │
│  │   Predictor) │                                      │
│  └──────┬───────┘                                      │
│         │ ẽ = x̂ - x  (预测误差)                         │
│  ┌──────▼───────┐                                      │
│  │  自适应律     │ σ̂ = −Γ · ẽ_relevant                   │
│  │  (Adaptation │ ← Γ 是适应增益（越大适应越快）         │
│  │   Law)       │                                      │
│  └──────┬───────┘                                      │
│         │ σ̂ (估计的扰动)                                │
│  ┌──────▼───────┐                                      │
│  │  低通滤波器   │ u_L1(s) = C(s) · σ̂(s)               │
│  │  (Low-pass   │ ← C(s) = ω_c / (s + ω_c)            │
│  │   Filter)    │ ← 限制补偿带宽，滤除噪声              │
│  └──────┬───────┘                                      │
│         ↓                                               │
│  u_total = u_MPC + u_L1  ← 叠加在 MPC 输出之上！         │
└─────────────────────────────────────────────────────────┘
```

**为什么需要低通滤波器？** 这是 L1 架构中最精妙的设计。如果没有低通滤波器，自适应增益 $\Gamma$ 越大、适应越快——但也越容易放大传感器噪声。低通滤波器将适应速率（$\Gamma$，可以很大）和鲁棒性（$\omega_c$，限制带宽）**解耦**了。$\Gamma$ 决定"多快估计扰动"，$\omega_c$ 决定"补偿多宽频带的扰动"——两个旋钮独立可调。

> 如果去掉低通滤波器会怎样？$\Gamma = 1000$ 时，传感器噪声（~1 mm 量级的位置噪声）被放大 1000 倍注入控制信号，导致 ~1 m 量级的推力振荡——无人机会剧烈抖动。低通滤波器以 $\omega_c = 20$ rad/s 截断高频噪声，只让低频扰动（风、载荷）的补偿通过。

### L1 叠加在 MPC 上的完整架构

```
                    ┌──────────────┐
  x_ref ──────────→ │   NMPC       │──→ u_MPC ──┐
                    │  (acados)    │             │
                    └──────────────┘             │
                                                 │  + （加法叠加）
                    ┌──────────────┐             │
  x_measured ────→  │   L1 自适应   │──→ u_L1 ───┘
                    │  控制器       │             │
                    └──────────────┘             │
                                                 ↓
                                            u_total = u_MPC + u_L1
                                                 │
                                                 ↓
                                          [飞控执行器]
```

**三个关键设计原则**：

1. **L1 不替代 MPC**，而是叠加补偿。MPC 负责前瞻性规划（约束、轨迹跟踪）；L1 负责实时扰动补偿。两者职责正交。
2. **MPC 基于名义模型规划**——不需要知道扰动。保持 MPC 的 OCP 问题结构不变，不增加在线计算量。
3. **L1 在控制输出空间工作**——补偿的是推力和力矩偏差，而非状态或参数。

### C++ 实现——L1 自适应控制器

```cpp
/**
 * L1 自适应控制器——叠加在 NMPC 之上
 * 基于 Hanover 等 RA-L 2022 的简化实现
 */
class L1AdaptiveController {
public:
    struct Config {
        double adaptation_gain = 1000.0;   // Γ: 适应增益
        double filter_cutoff   = 20.0;     // ω_c: 低通截止频率 [rad/s]
        double dt              = 0.002;    // 采样周期 [s]（500 Hz）
        double mass            = 0.73;     // 无人机质量 [kg]
    };

    L1AdaptiveController(const Config& cfg) : cfg_(cfg) {
        sigma_hat_.setZero();   // 估计扰动
        u_l1_.setZero();        // L1 补偿信号
        x_hat_.setZero();       // 预测状态
        // 一阶 IIR 低通滤波器系数
        alpha_ = 1.0 - std::exp(-cfg_.filter_cutoff * cfg_.dt);
    }

    Eigen::Vector4d compute(
        const Eigen::Matrix<double, 10, 1>& x_measured,
        const Eigen::Vector4d& u_mpc)
    {
        // 1. 预测误差
        Eigen::Matrix<double, 10, 1> e = x_hat_ - x_measured;

        // 2. 自适应律：用速度误差估计力扰动
        Eigen::Vector3d f_dist = -cfg_.adaptation_gain * e.segment<3>(3);
        sigma_hat_.head<3>() = f_dist / cfg_.mass;

        // 3. 低通滤波——关键步骤！
        u_l1_ = (1.0 - alpha_) * u_l1_ + alpha_ * sigma_hat_;

        // 4. 更新状态预测器
        updatePredictor(u_mpc + u_l1_);

        return u_l1_;
    }

private:
    void updatePredictor(const Eigen::Vector4d& u_total) {
        // 名义动力学 Euler 积分（简化）
        Eigen::Quaterniond q(x_hat_(6), x_hat_(7), x_hat_(8), x_hat_(9));
        Eigen::Vector3d thrust_body(0, 0, u_total(0));
        Eigen::Vector3d acc = q * thrust_body + Eigen::Vector3d(0, 0, -9.8066)
                              + sigma_hat_.head<3>();  // 加入估计扰动
        x_hat_.segment<3>(0) += x_hat_.segment<3>(3) * cfg_.dt;
        x_hat_.segment<3>(3) += acc * cfg_.dt;
        // ... 四元数积分（省略）
    }

    Config cfg_;
    Eigen::Vector4d sigma_hat_, u_l1_;
    Eigen::Matrix<double, 10, 1> x_hat_;
    double alpha_;
};

// 使用示例：MPC + L1 集成
void mpcLoopWithL1() {
    AcadosMpcSolver mpc;
    L1AdaptiveController l1({.adaptation_gain=800, .filter_cutoff=15});

    while (running) {
        auto x = getStateEstimate();
        Eigen::Vector4d u_mpc = mpc.solve(x, getReferenceTrajectory());
        Eigen::Vector4d u_l1 = l1.compute(x, u_mpc);
        Eigen::Vector4d u = u_mpc + u_l1;  // 叠加！

        // 饱和限幅
        u(0) = std::clamp(u(0), 0.5, 4.0 * 9.8066);
        for (int i = 1; i < 4; ++i) u(i) = std::clamp(u(i), -6.0, 6.0);

        sendToFlightController(u);
    }
}
```

### 实验数据（Hanover RA-L 2022）

| 实验条件 | NMPC 单独 | L1-NMPC | 改善 |
|---------|----------|---------|------|
| 无扰动悬停 | 3.2 mm | 3.1 mm | ~0% |
| 圆轨迹 4 m/s | 12.1 mm | 8.4 mm | 31% |
| 载荷 30% 体重 | 28.4 mm | 4.8 mm | 83% |
| 载荷 60% 体重 | 51.2 mm | 5.1 mm | 90% |
| 70 km/h 竞速 | 发散 | 成功 | --- |
| 侧风 3 m/s | 35.7 mm | 6.2 mm | 83% |
| **L1 额外计算** | — | **0.01 ms** | — |

> **本质洞察**：L1 补偿几乎"免费"（0.01 ms），但在大扰动下将误差降低 80-90%。在名义条件下性能不退化。这是**性价比最高的鲁棒性增强方案**——在你尝试 GP-MPC 或 Neural-MPC 之前，先加 L1。

### ⚠️ 常见陷阱

**陷阱 1（编程陷阱）：L1 采样频率太低**

```
错误做法：L1 和 MPC 以相同频率运行（如 50 Hz）
现象：扰动估计有明显延迟，补偿效果差
根本原因：L1 的状态预测器需要高频更新才能准确估计快速扰动。
         50 Hz 的 Nyquist 频率只有 25 Hz，无法跟踪 > 25 Hz 的扰动
正确做法：L1 以 200-500 Hz 运行（dt = 2-5 ms），MPC 可以更慢（50-100 Hz）
```

**陷阱 2（思维陷阱）：以为 L1 可以替代 MPC**

```
错误理解："既然 L1 能在线补偿扰动，那用 PID + L1 不就够了？"
现象：PID + L1 在约束场景下失效——推力饱和、急转弯超调
根本原因：L1 补偿的是"等效扰动"——它把模型误差投影到控制空间，但不做前瞻
         规划。MPC 的前瞻性（看 N 步以后）和约束处理能力是 L1 无法替代的
正确做法：L1 是 MPC 的补充，不是替代。MPC 负责约束和前瞻，L1 负责扰动
```

**陷阱 3（编程陷阱）：忘记对 L1 输出做饱和限幅**

```
错误做法：u_total = u_mpc + u_l1 后直接发送，不检查是否超限
现象：大扰动下 u_l1 可能很大，u_total 超出执行器限制
根本原因：L1 不知道执行器的物理限制（它没有约束模型）
正确做法：叠加后必须做饱和限幅 u_total = clamp(u_mpc + u_l1, u_min, u_max)
```

### 练习

**练习 1（实现题）**：在 §D2.1 的 acados 仿真基础上，实现一个最简 L1 自适应补偿器。在仿真环境中加入 0.5 N 恒力扰动（模拟侧风），对比有/无 L1 的位置跟踪误差。

**练习 2（调参题）**：固定 $\Gamma = 800$，将 $\omega_c$ 从 5 调到 50 rad/s，绘制跟踪误差 vs $\omega_c$ 的曲线。你会发现存在一个最优 $\omega_c$——太低补偿不够，太高噪声放大。

---

L1 自适应的核心优势是"零训练、强泛化"——但如果你有训练数据，能不能做得更精确？下面两节讨论用数据驱动的方式增强 MPC 的模型精度。

## §D2.6 GP-MPC——高斯过程残差增强 ⭐⭐⭐

### 动机

L1 自适应把所有模型误差投影到一个"等效扰动"——它不区分风力、气动效应和质量变化。这在大多数场景下够用，但在**重复飞行已知赛道**时，你可以做得更好：用数据**学习**模型误差的空间分布，在 MPC 中直接使用更精确的模型。

### 核心思想

GP-MPC（Torrente 等, RA-L 2021）的思路很直观：

$$
\dot{x} = f_{\text{nominal}}(x, u) + \underbrace{\mu_{\text{GP}}(x, u)}_{\text{GP 学到的残差}}
$$

三个独立 GP 分别学 $x/y/z$ 轴的体速度残差，输入特征为 $(v_{\text{body}}, \omega_{\text{body}}, f_{\text{thrust}})$。

**为什么用 GP 而非简单多项式拟合？** GP 提供两个多项式没有的关键能力：(a) **预测不确定性**——GP 不仅给出预测均值 $\mu(x^*)$，还给出方差 $\sigma^2(x^*)$，可以用于鲁棒 MPC 的约束收紧；(b) **训练数据少时表现好**——GP 的贝叶斯框架天然处理数据稀疏性。

### 稀疏 GP 与实时性

全 GP 推理复杂度 $O(N^3)$（$N$ = 训练点数），不适合实时 MPC。Torrente 等使用稀疏 GP（诱导点法），将推理复杂度降为 $O(M^2)$（$M \ll N$ 为诱导点数）。

```
全 GP：N=1000 训练点 → 推理 ~10 ms → 不实时
稀疏 GP：M=20 诱导点 → 推理 ~0.1 ms × 3 轴 ≈ 0.3 ms
加上雅可比近似（有限差分，20 次额外 GP 评估）→ 总 ~4 ms
```

### GP-MPC 的局限

| 优势 | 局限 |
|------|------|
| 少量数据（10-100 条轨迹） | 推理慢 ~4 ms（vs L1 的 0.01 ms） |
| 提供预测不确定性 $\sigma^2$ | 分布外（OOD）泛化差 |
| 物理可解释（核函数长度尺度） | 雅可比需有限差分近似 |

> 如果不了解 GP-MPC 的局限直接使用会怎样？你在室内 5 m/s 训练了 GP 模型，然后到室外 15 m/s 飞行——GP 回退到先验均值（零残差），MPC 退化为名义模型，和没加 GP 一样。这就是 OOD 泛化问题——GP 只在"见过的"状态-控制空间区域有效。

### ⚠️ 常见陷阱

**陷阱 1（概念误区）：以为 GP 能泛化到任意条件**

```
错误理解："GP 学了模型残差，新环境下也能用"
现象：训练时 5 m/s，部署时 15 m/s——GP 预测退化为先验均值
根本原因：GP 是非参数模型，本质是训练数据的插值/外推。远离训练分布时，
         GP 的预测方差 σ² → 先验方差，均值 → 0
正确做法：部署条件必须在训练分布内。用 σ² 检测 OOD 并切换到 L1 补偿
```

**陷阱 2（编程陷阱）：诱导点选择不当**

```
错误做法：随机选 M=20 个诱导点
现象：GP 精度在某些速度区间好、某些区间差
根本原因：诱导点应覆盖状态-控制空间的关键区域。随机选择可能在
         高速区域（气动效应最大）没有诱导点
正确做法：用 k-means 或边界感知采样选择诱导点
```

### 练习

**练习 1（思考题）**：GP-MPC 和 L1-NMPC 在什么场景下各自胜出？设计一个表格，列出 5 个具体场景和推荐方案。

**练习 2（设计题）**：设计一个 GP + L1 的混合方案——GP 在训练分布内提供精确残差，L1 在 OOD 时提供兜底补偿。什么信号可以触发从 GP 切换到 L1？提示：GP 的预测方差 $\sigma^2$ 超过阈值时切换。

---

GP-MPC 的核心瓶颈是推理速度（~4 ms）和雅可比近似。Neural-MPC 用神经网络替代 GP，通过 l4casadi 获得精确雅可比，推理速度也提升到 ~1 ms。

## §D2.7 Neural-MPC——神经网络动力学进入 acados ⭐⭐⭐⭐

### 动机

Neural-MPC（Salzmann 等, RA-L 2023）解决了 GP-MPC 的两个瓶颈：推理速度和雅可比计算。核心思想：用 PyTorch 训练的 MLP 替代 GP，通过 **l4casadi** 桥接框架将其嵌入 acados OCP——CasADi 自动微分提供精确雅可比，无需有限差分。

### l4casadi 桥接

l4casadi（Learning for CasADi）是 Neural-MPC 的关键使能技术。它将 PyTorch 模型转换为 CasADi 可用的符号函数：

```python
import l4casadi as l4c
import casadi as cs

# 加载训练好的 PyTorch 模型
model = ResidualDynamics()
model.load_state_dict(torch.load("trained.pt", weights_only=True))

# l4casadi 包装
l4c_model = l4c.L4CasADi(model, model_expects_batch_dim=True, device="cpu")

# 在 CasADi 中使用——精确雅可比！
x_sym = cs.MX.sym("x", 10)
u_sym = cs.MX.sym("u", 4)
residual = l4c_model(cs.vertcat(x_sym, u_sym))
J_x = cs.jacobian(residual, x_sym)  # 自动微分，精确雅可比
```

### 三种鲁棒方案对比

| 维度 | L1-NMPC | GP-MPC | Neural-MPC |
|------|---------|--------|-----------|
| 残差模型 | 无（在线估计） | 稀疏 GP | 深度 NN |
| 训练数据需求 | **零** | 少（10-100 条） | 中（1000+ 条） |
| 在线计算量 | **0.01 ms** | ~4 ms | ~1-2 ms |
| 泛化到新环境 | **强** | 差（OOD） | 中 |
| 精度提升 | ~90%（大扰动） | ~70% | **~82%** |
| 理论保证 | **Lyapunov** | 概率边界 | 无 |
| 部署复杂度 | **低** | 中 | 高（需 PyTorch） |
| 适用场景 | 通用/原型验证 | 数据稀缺 + 不确定性 | 已知赛道/重复飞行 |

**选择建议**：

- **原型验证/通用场景** $\to$ L1-NMPC（零训练、强泛化、极低开销——先试这个）
- **已知赛道/重复飞行** $\to$ Neural-MPC（最高精度）
- **数据稀缺 + 需要不确定性** $\to$ GP-MPC
- **实际部署最佳方案** $\to$ L1-NMPC + Neural-MPC 组合——NN 提供基础残差补偿，L1 处理 NN 训练分布外的意外扰动

### Neural-Fly——元学习路线

Neural-Fly（O'Connell 等, Science Robotics 2022）走了不同路线：不是把大网络放进 MPC，而是用元学习提取**风条件无关**的共享表示：

$$
f_{\text{aero}}(x) = A(w) \cdot \Phi(x)
$$

其中 $\Phi(x)$ 是固定的共享基函数（大网络，但只前向传播一次），$A(w)$ 是小系数矩阵（$h \times 3$, $h \sim 10$-$20$），用 Kalman 风格的在线最小二乘更新。

Neural-Fly 的独特优势：有 Lyapunov/contraction 稳定性证明；12 分钟真实飞行数据即可训练；在线适应只需 ~0.05 ms。

### ⚠️ 常见陷阱

**陷阱 1（概念误区）：以为 Neural-MPC 不需要名义模型**

```
错误理解："神经网络能学全部动力学，不需要物理模型"
现象：纯 NN 动力学 + MPC 在训练分布内很好，分布外灾难性失败
根本原因：NN 学的是 f_nominal + f_residual 的总和。将 f_nominal 分离出来后，
         NN 只需学残差——残差幅度远小于总动力学，更容易泛化
正确做法：始终保留名义物理模型，NN 只学残差。物理优先、数据补充
```

**陷阱 2（编程陷阱）：l4casadi 的 batch_dim 设置错误**

```
错误做法：PyTorch 模型期望 batch 维度但 l4casadi 没设置
现象：CasADi 调用 NN 时维度不匹配报错
根本原因：PyTorch 模型通常期望输入 shape 为 (B, D)，但 CasADi 传入的是 (D,)
正确做法：l4c.L4CasADi(model, model_expects_batch_dim=True)
```

### 练习

**练习 1（对比题）**：在同一个四旋翼仿真环境中，分别实现 L1-NMPC 和名义 NMPC，加入 2 m/s 恒风扰动。记录 100 秒跟踪的 RMSE。预期 L1-NMPC 误差降低 > 50%。

**练习 2（思考题）**：Neural-MPC 把 PyTorch 模型放进 acados；足式机器人的 OCS2 用 CppADCodeGen 预编译 Pinocchio 动力学。两者都是"让 NLP 求解器使用复杂动力学"——但技术路线完全不同。分析各自的优劣和适用场景。

---

到这里我们覆盖了四旋翼 NMPC 的三种鲁棒增强路线。但上面所有方法都基于**基于优化的 MPC**（SQP-RTI）。还有一种完全不同的方法论：**基于采样的 MPC**（MPPI）。了解它的适用场景对于在不同机器人系统中选择正确的 MPC 方法至关重要。

## §D2.8 采样式 MPC（MPPI）对比 ⭐⭐

### 动机

MPPI（Model Predictive Path Integral）是一种无导数、基于采样的 MPC 方法。它不做线性化、不解 QP——而是采样大量候选轨迹，按代价加权平均。为什么需要了解它？因为在足式机器人和 GPU 密集场景中，MPPI 已经成为主流选择（如 NVIDIA Isaac Lab 的默认 MPC），而在四旋翼场景中 SQP-RTI 仍然是首选。理解两者的适用边界，你才能在实际项目中做出正确选择。

### SQP-RTI vs MPPI 系统对比

| 维度 | SQP-RTI (acados) | MPPI |
|------|-----------------|------|
| 需要模型可微？ | 是（雅可比） | **否**（黑盒前向模拟） |
| 约束处理 | **精确**（等式/不等式） | 弱（代价惩罚或投影） |
| 非凸代价 | 可能局部最优 | **全局搜索能力更强** |
| 计算硬件 | CPU（串行） | **GPU**（并行） |
| 四旋翼典型延迟 | **0.1-1 ms** | 5-50 ms |
| 高维控制空间 | 可行 | 维度诅咒（>10D 效率骤降） |
| 代表实现 | acados, HPIPM | mppi-isaac, STORM |

**四旋翼场景的结论**：SQP-RTI + acados 是**默认选择**——问题规模小（4 维控制）、约束重要（推力/体率上下限）、需要低延迟（<1 ms）、嵌入式部署（无 GPU）。MPPI 在四旋翼上的优势场景有限。

**足式机器人场景则相反**：接触动力学非光滑（不可微）、控制维度高（12+ 维关节力矩）、NVIDIA Isaac GPU 仿真已成标配。MPPI 在这个场景中更有竞争力。

### MPPI 算法核心

```python
def mppi_step(x0, U_nominal, dynamics_fn, cost_fn,
              K=1024, sigma=0.5, lam=1.0, N=20):
    """
    K: 采样数（1024-8192 典型）; sigma: 噪声标准差; lam: 温度参数
    """
    m = U_nominal.shape[1]
    epsilon = np.random.randn(K, N, m) * sigma        # 采样扰动
    U_candidates = U_nominal[None, :, :] + epsilon     # 候选控制

    # 并行前向模拟（GPU 加速的关键）
    costs = np.zeros(K)
    for k in range(K):
        x = x0.copy()
        for t in range(N):
            costs[k] += cost_fn(x, U_candidates[k, t])
            x = dynamics_fn(x, U_candidates[k, t])

    # 重要性加权（softmin）
    beta = np.min(costs)
    weights = np.exp(-(costs - beta) / lam)
    weights /= np.sum(weights)

    # 加权平均
    U_optimal = np.sum(weights[:, None, None] * U_candidates, axis=0)
    return U_optimal
```

### ⚠️ 常见陷阱

**陷阱 1（思维陷阱）：以为 MPPI 总是比 SQP-RTI 差**

```
错误理解："MPPI 是'暴力搜索'，肯定不如 SQP-RTI 精确"
现象：在某些非凸代价景观中，MPPI 找到了比 SQP-RTI 更好的解
根本原因：SQP-RTI 是局部方法（梯度下降），对非凸问题可能陷入局部最优。
         MPPI 是全局搜索（蒙特卡洛采样），有概率找到全局最优
正确做法：根据问题性质选择。凸/接近凸 → SQP-RTI；非凸 → MPPI
```

**陷阱 2（概念误区）：以为 MPPI 不需要好的动力学模型**

```
错误理解："MPPI 不做线性化，所以对模型精度不敏感"
现象：MPPI 用粗糙的模型前向模拟，采样的轨迹与真实偏差大，控制效果差
根本原因：MPPI 不需要模型可微，但需要模型足够精确——它的控制质量完全取决于
         前向模拟的精度。模型越准，采样的轨迹越接近真实，加权平均越有意义
正确做法：MPPI 对模型精度的要求不低于 SQP-RTI，只是不要求可微
```

### 练习

**练习 1（思考题）**：设计一个四旋翼场景，其中 MPPI 比 SQP-RTI 更合适。提示：考虑代价函数高度非凸的情况，如避障代价。

**练习 2（跨章综合题）**：结合 D1 的微分平坦、D2 的 MPC、D3 的轨迹生成，描述一个完整的无人机自主飞行管线。从航路点输入到电机指令输出，每个模块做什么？模块间的数据流是什么？

---

## §D2.9 前沿：Actor-Critic MPC 与 MPCC++ ⭐⭐⭐⭐

### 动机

MPC 的代价函数 $Q, R$ 通常由工程师手工设计——这在标准跟踪场景下足够，但在竞速等极端场景下，手工设计的代价函数未必是时间最优的。2025 年 UZH RPG 的 AC-MPC 提出了一个优雅的解决方案：**用 RL 自动学习 MPC 的代价函数**。

### Actor-Critic MPC

AC-MPC（Romero 等, T-RO 2025）将可微 MPC 作为 RL 的 actor 网络：

```
传统 RL：  Actor = 神经网络 π_θ(a|s)     ← RL 优化 θ
AC-MPC：  Actor = MPC(x, cost_θ)         ← RL 优化 cost 参数 θ

好处：
  - MPC 提供短期约束满足和动力学一致性
  - RL 提供长期目标优化（时间最优）
  - cost_θ 是小型 NN → MPC 的 Q/R 由 RL 学到
```

**实验结果**：在无人机竞速赛道上达到 21 m/s（~76 km/h）。相比纯 MPC 圈速提升 ~15%，相比纯 RL 样本效率提升 ~5 倍。

### MPCC++

MPCC++（轮廓控制）将轨迹跟踪和时间最优飞行统一：

$$
\min \sum_k \left[ \|e_{\text{lag},k}\|^2 + \|e_{\text{con},k}\|^2 - w_\theta \Delta \theta_k \right]
$$

其中 $e_{\text{lag}}$ 是沿路径切线方向的滞后误差，$e_{\text{con}}$ 是垂直于路径的轮廓误差，$\Delta \theta_k$ 是路径参数进度（要最大化 $\to$ 最短时间）。100 Hz 实时，赛道 > 80 km/h 无碰撞。

### ⚠️ 常见陷阱

**陷阱 1（思维陷阱）：以为 AC-MPC 可以替代手工调参**

```
错误理解："有了 RL 自动调参，就不需要理解 MPC 的代价函数设计了"
现象：RL 训练不收敛，或收敛到奇怪的策略
根本原因：RL 优化的搜索空间是 MPC 参数空间——如果 MPC 的问题结构本身有缺陷
         （如约束设置不合理），RL 无法修复。AC-MPC 是"MPC + RL"，不是"RL 替代 MPC"
正确做法：先手工调一个合理的 baseline MPC，再用 RL fine-tune 代价参数
```

### 练习

**练习 1（论文精读）**：精读 Romero 等的 AC-MPC 论文（T-RO 2025），画出训练管线的数据流图：仿真环境 $\to$ 可微 MPC actor $\to$ PPO critic $\to$ cost 参数更新。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "MPC 就是 LQR 加了约束" | MPC 是**非线性**最优控制的在线求解（SQP），LQR 只是 MPC 在线性无约束特殊情况下的退化 |
| "RTI 精度差因为只做一步" | RTI 的控制误差 $O(\Delta t^2)$，在滚动时域框架下被持续修正，实际性能几乎等同完全收敛 SQP |
| "acados 比 ACADO 好因为算法更好" | 算法框架（SQP-RTI）相同。acados 快是因为**底层实现**更优：HPIPM 利用块三对角结构，BLASFEO 优化小矩阵 |
| "L1 自适应会降低名义性能" | L1 在名义条件下补偿为零（$\hat{\sigma} \approx 0$），性能不退化 |
| "GP-MPC 比 L1 更先进所以更好" | 不同场景适用不同方案。通用场景首选 L1（零训练），已知赛道首选 Neural-MPC（最高精度） |
| "MPPI 是 MPC 的低端版本" | MPPI 在非凸代价、不可微模型、高维 GPU 场景下有独特优势 |

---

## 本章小结

### 术语速查表

| 术语 | 英文 | 一句话定义 |
|------|------|-----------|
| 最优控制问题 | OCP (Optimal Control Problem) | 在动力学约束下最小化代价函数的数学问题 |
| 滚动时域 | Receding Horizon | MPC 每步只优化 $N$ 步窗口然后前移——兼顾前瞻性和反馈性 |
| 序列二次规划 | SQP (Sequential Quadratic Programming) | 将非线性优化逐步线性化为 QP 子问题并求解 |
| 实时迭代 | RTI (Real-Time Iteration) | SQP 只做一步迭代——暖启动保证精度，流水线最小化延迟 |
| 部分凝聚 | Partial Condensing | 平衡稀疏和稠密 QP 形式的中间策略 |
| 集合推力+体率 | CTBR (Collective Thrust + Body Rate) | 四旋翼 MPC 的标准动作空间 |
| 零拷贝 | Zero-Copy | `Eigen::Map` 直接包装 C 数组，不分配新内存 |
| L1 自适应 | L1 Adaptive Control | 在线估计等效扰动并叠加补偿——零训练、强泛化 |
| 诱导点 | Inducing Points | 稀疏 GP 用少量代表点近似全数据——$O(M^2)$ 推理 |

### 知识点总表

| # | 知识点 | 核心要点 | 对应节 | 难度 |
|---|--------|---------|--------|------|
| 1 | 四旋翼 OCP 结构 | 10 状态 4 输入 N=20，最简真实机器人 MPC | §D2.1 | ⭐⭐ |
| 2 | rpg_mpc 三层架构 | C 求解器 $\to$ Eigen::Map $\to$ ROS | §D2.2 | ⭐⭐ |
| 3 | Eigen::Map 零拷贝 | 包装 C 数组为 Eigen 矩阵，零内存分配 | §D2.2 | ⭐⭐ |
| 4 | acados 三层栈 | CasADi + HPIPM + BLASFEO | §D2.3 | ⭐⭐⭐ |
| 5 | HPIPM Riccati | 块三对角 KKT 的结构化求解，比通用 QP 快 400 倍 | §D2.3 | ⭐⭐⭐ |
| 6 | SQP-RTI | 准备-反馈流水线，反馈延迟 0.1-0.5 ms | §D2.4 | ⭐⭐⭐ |
| 7 | L1 自适应三组件 | 预测器 + 自适应律 + 低通滤波器，0.01 ms 额外开销 | §D2.5 | ⭐⭐⭐ |
| 8 | GP-MPC | 稀疏 GP 残差增强，少量数据 + 不确定性 | §D2.6 | ⭐⭐⭐ |
| 9 | Neural-MPC + l4casadi | PyTorch $\to$ CasADi 桥接，82% 误差降低 | §D2.7 | ⭐⭐⭐⭐ |
| 10 | SQP-RTI vs MPPI | 凸/低维/嵌入式 $\to$ SQP；非凸/GPU $\to$ MPPI | §D2.8 | ⭐⭐ |
| 11 | AC-MPC | RL 优化 MPC 代价参数，21 m/s 竞速 | §D2.9 | ⭐⭐⭐⭐ |

---

## 累积项目：本章新增模块

**项目主线**：贯穿 D1-D5 的"从动力学建模到自主飞行"完整管线。

**D2 新增**：在 D1 的几何控制器基础上，添加 acados NMPC 模块和 L1 自适应补偿层。

```
D1 模块：四旋翼动力学模型 + SE(3) 几何控制器
    ↓
D2 新增：
    ├─ acados OCP 定义（quadrotor_model.py）
    ├─ MPC 闭环仿真（mpc_simulation.py）
    ├─ L1 自适应补偿（l1_adaptive.py 或 l1_adaptive.cpp）
    └─ 对比实验：SE(3) vs MPC vs MPC+L1
    ↓
D3 将新增：多项式轨迹生成模块（参考轨迹来源）
```

---

## 延伸阅读

### 教材与综述

| 资源 | 定位 |
|------|------|
| Rawlings, Mayne, Diehl, *Model Predictive Control* (2nd ed., 2020) | MPC 理论标准教材——OCP、稳定性、鲁棒 MPC。⭐⭐⭐⭐ |
| Hovakimyan & Cao, *L1 Adaptive Control Theory* (2010) | L1 自适应控制系统论述。⭐⭐⭐ |
| Verschueren 等, "acados" (MPC 2022) | acados 系统论文——必读。⭐⭐⭐⭐⭐ |
| Frison & Diehl, "HPIPM" (IFAC 2020) | HPIPM 算法细节。⭐⭐⭐ |

### 关键论文

| 论文 | 为什么要读 |
|------|-----------|
| Hanover 等, "L1-NMPC" (RA-L 2022) | L1+MPC 的实验验证标杆——表格数据直接可引用 |
| Salzmann 等, "Neural-MPC" (RA-L 2023) | l4casadi 桥接的完整技术路线 |
| Torrente 等, "GP-MPC" (RA-L 2021) | 稀疏 GP 残差增强 MPC 的先驱工作 |
| O'Connell 等, "Neural-Fly" (Sci. Rob. 2022) | 元学习 + 在线适应的优雅方案 |
| Romero 等, "AC-MPC" (T-RO 2025) | 可微 MPC + RL 融合的最新进展 |
| Frey 等, "AS-RTI" (arXiv 2024) | 高级步实时迭代的理论和 acados 实现 |
| Frey 等, "Multi-Phase OCP in acados" (OCAM 2025) | acados 多阶段 OCP 最新功能 |

### 开源项目

| 项目 | GitHub | Stars | 定位 |
|------|--------|-------|------|
| acados | `acados/acados` | ~1.3k | 现代 MPC 求解器（⭐⭐⭐⭐⭐ 首选） |
| rpg_mpc | `uzh-rpg/rpg_mpc` | ~486 | Eigen::Map 零拷贝教科书 |
| data_driven_mpc | `uzh-rpg/data_driven_mpc` | ~347 | GP-MPC 参考实现 |
| neural-mpc | `TUM-AAS/neural-mpc` | ~261 | Neural-MPC 参考实现 |
| l4casadi | `Tim-Salzmann/l4casadi` | ~800+ | PyTorch $\to$ CasADi 桥接 |
| neural-fly | `aerorobotics/neural-fly` | ~211 | 元学习风力补偿 |
| mav_control_rw | `ethz-asl/mav_control_rw` | ~430 | ETH ASL 线性/非线性 MPC |

---

## 本章与后续章节的关系

| 后续章节 | 关系 | 本章铺垫的知识点 |
|---------|------|----------------|
| D3 多项式轨迹生成 | MPC 的参考轨迹从何而来 | OCP 的 $x_{\text{ref}}(t)$ 由 D3 的 min-snap QP 生成 |
| D4 B 样条轨迹 | 时间也变成优化变量 | 本章固定 $T_i$，D4 优化 $T_i$ |
| D5 MINCO | 端到端时空轨迹优化 | 本章的 MPC 负责跟踪，D5 负责生成 |
| 足式 M5 | 相同的 SQP/QP 求解器栈 | acados/HPIPM/BLASFEO 直接复用 |

---

## 故障排查手册

### F1. acados 求解器返回非零 status

```
症状：solver.solve() 返回 status ≠ 0
     常见 status: 1 = 达到最大迭代, 2 = 不可行, 4 = QP 失败

诊断步骤：
  1. 检查 KKT 残差：solver.get_stats("res_stat") — 应 < 1e-3
  2. 检查 QP 迭代：solver.get_stats("qp_iter") — 过多说明 QP 困难
  3. 打印约束违反：检查参考轨迹是否违反物理限制

常见原因与修复：
  ├─ 原因 A: 冷启动——没有用上一步的解暖启动
  │  修复: solver.set(k, "x", x_prev[k]); solver.set(k, "u", u_prev[k])
  │
  ├─ 原因 B: 约束不可行——参考加速度 > 4g
  │  修复: 检查参考轨迹的激进程度；放松约束临时调试
  │
  ├─ 原因 C: Q/R 条件数太大——Q_pos=1e6, Q_vel=1
  │  修复: 保持 cond(Q) < 1e6；各维度权重差异不超过 3-4 个数量级
  │
  └─ 原因 D: 步长 dt 太大导致积分发散
     修复: 减小 dt 或换 RK4；检查四元数归一化
```

### F2. MPC 跟踪误差突然增大

```
症状：误差从 ~5 mm 跳到 ~50 mm+

诊断步骤：
  1. 记录 u_mpc——是否持续在 u_max 或 u_min（饱和）？
  2. 记录 x_predicted vs x_measured——模型预测是否准确？
  3. 检查参考轨迹——加速度/jerk 是否超过物理极限？

常见原因与修复：
  ├─ 原因 A: 执行器饱和
  │  修复: 降低参考轨迹攻击性；增大 R 惩罚大控制输入
  │
  ├─ 原因 B: 模型失配（风/载荷变化）
  │  修复: 加 L1 自适应层（§D2.5）
  │
  ├─ 原因 C: 状态估计延迟
  │  修复: x0_compensated = x0 + v * t_delay
  │
  └─ 原因 D: 预测时域太短——急转弯前没提前减速
     修复: 增大 N 或 dt
```

### F3. L1 自适应控制器振荡

```
症状：加入 L1 后控制信号出现高频振荡

诊断步骤：
  1. 绘制 sigma_hat 时间序列——是否高频振荡？
  2. 绘制 u_l1 频谱——是否有高于 ω_c 的能量泄漏？
  3. 检查 Γ 和 ω_c 的组合

常见原因与修复：
  ├─ 原因 A: ω_c 太高（> 系统带宽）
  │  修复: 降低 ω_c 到 10-30 rad/s
  │
  ├─ 原因 B: Γ 太大 + 传感器噪声
  │  修复: 降低 Γ 或增加传感器预滤波
  │
  └─ 原因 C: L1 采样频率太低
     修复: L1 以 > 200 Hz 运行
```

### F4. Eigen::Map 使用常见错误

```
症状：Segfault、结果错误、或 Eigen assert 失败

常见错误与修复：
  ├─ 错误 A: Map 指向栈上临时变量（生命周期问题）
  │  修复: 确保底层内存生命周期 >= Map 使用周期
  │
  ├─ 错误 B: 行优先 vs 列优先不匹配
  │  修复: 确认底层数据布局，在 Map 模板参数中匹配
  │
  └─ 错误 C: SIMD 对齐问题
     修复: 使用 Eigen::Map<..., Eigen::Aligned16>(ptr)
```

### F5. acados 代码生成编译失败

```
症状：Python 端生成代码后 C 编译报错

常见原因与修复：
  ├─ 原因 A: acados 安装路径未设置
  │  修复: export ACADOS_SOURCE_DIR=/path/to/acados
  │
  ├─ 原因 B: CasADi 版本与 acados 不兼容
  │  修复: pip install casadi==3.6.x（匹配 acados 文档版本）
  │
  └─ 原因 C: 模型名冲突（同目录下多个模型）
     修复: 每个模型使用不同的 json_file 和 code_export_directory
```

---

## API 速查表

### acados Python 接口核心 API

| API | 用途 | 示例 |
|-----|------|------|
| `AcadosModel` | 定义动力学模型 | `model.f_expl_expr = f_expl` |
| `AcadosOcp` | 配置 OCP 问题 | `ocp.cost.W = W` |
| `AcadosOcpSolver` | 创建求解器 | `solver = AcadosOcpSolver(ocp)` |
| `solver.set(k, "x", x)` | 设置第 k 步初始猜测 | 暖启动 |
| `solver.set(k, "yref", y)` | 设置第 k 步参考 | 在线更新参考轨迹 |
| `solver.solve()` | 求解 OCP | 返回 0 表示成功 |
| `solver.get(k, "u")` | 获取第 k 步最优控制 | `u_opt = solver.get(0, "u")` |
| `solver.get_stats("time_tot")` | 获取求解时间 | 单位：秒 |

### rpg_mpc Eigen::Map 模式

| 模式 | 代码 | 用途 |
|------|------|------|
| 包装 C 数组 | `Eigen::Map<Matrix<double,10,21>> m(ptr)` | 零拷贝读写 |
| 只读包装 | `Eigen::Map<const Vector4d> u(ptr)` | 防止意外修改 |
| 列提取 | `m.col(k).segment<3>(0)` | 取第 k 步位置 |

---

## 研究实践建议

| 层次 | 建议 |
|------|------|
| **入门**（1-2 周） | 跑通 acados Python 四旋翼示例；理解 OCP 公式和 SQP-RTI |
| **进阶**（2-4 周） | 精读 rpg_mpc 源码；实现 L1 自适应补偿；在 Gazebo 中对比 |
| **研究级**（1-3 月） | 复现 Neural-MPC/l4casadi；尝试 AC-MPC 的可微 MPC + RL 训练 |

---

## 从 SLAM 到 MPC 的概念映射

这张对照表是为已经学过 SLAM 的读者准备的——如果你学过 v8 的 Ceres/GTSAM 章节，这里的 MPC 概念会非常亲切。

| SLAM 概念 | MPC 概念 | 共通点 |
|-----------|---------|--------|
| 因子图 | OCP 的 KKT 系统 | 都是稀疏结构化优化问题 |
| `BetweenFactor<Pose3>` | 动力学等式约束 | 连接相邻节点的约束 |
| Bayes Tree 变量消元 | Riccati 递推 | 块结构的 Gaussian elimination |
| iSAM2 增量更新 | RTI 暖启动 | 用上一步解加速本步 |
| 信息矩阵 $\Lambda$ | KKT 的块三对角 Hessian | 都是正定稀疏矩阵 |
| 边缘化先验 | 终端代价 $P$ | 总结"看不到的"信息 |
| `Eigen::Map` 在 Ceres cost | `Eigen::Map` 在 mpc_wrapper | 零拷贝包装 raw pointer |

---

## 版本信息速查

| 组件 | 推荐版本 | 说明 |
|------|---------|------|
| acados | $\ge$ 0.3.0 | 支持 AS-RTI 和多相 OCP |
| CasADi | $\ge$ 3.6.0 | 与 acados 匹配 |
| l4casadi | $\ge$ 0.3.0 | PyTorch $\to$ CasADi 桥接 |
| Eigen | $\ge$ 3.4 | Map 的 `Aligned16` 支持 |
| PyTorch | $\ge$ 2.0 | l4casadi 依赖 |
| PX4 | $\ge$ 1.14 | CTBR offboard 接口稳定 |

---

## 参数选择手册：从零调出一个能飞的 MPC ⭐⭐

前面各节反复出现"$Q$ 条件数别太大""$N$ 别太短""$\omega_c$ 别太高"这类零散告诫。但读者真正坐到仿真器前面时，面对的是一组空白的数值——**第一组数该填什么，错了往哪个方向改**。这一节把散落在全章的调参经验收拢成一张可执行的流程图。它不引入新理论，只是把"为什么"翻译成"怎么做"。

> **本质洞察**：MPC 调参不是在十几个旋钮里盲目搜索，而是沿着一条几乎固定的因果链逐级排查——先让求解器**收敛**，再让闭环**稳定**，最后才追求**精度**。颠倒这个顺序（一上来就调精度）是初学者最大的时间黑洞。

### 三阶段调参法

调参必须分三个阶段，每个阶段有明确的"通过判据"，不达标不进入下一阶段：

| 阶段 | 目标 | 通过判据 | 主要旋钮 |
|------|------|---------|---------|
| **阶段一：能解** | 求解器每步返回 status=0 | 连续 1000 步无求解失败 | $N$、$\Delta t$、积分器、约束松紧 |
| **阶段二：能稳** | 闭环不发散、悬停不抖 | 悬停位置漂移 < 2 cm，无可见振荡 | $Q/R$ 量级、$N$ 长度 |
| **阶段三：能准** | 跟踪误差达标 | 激进轨迹下误差 < 5 cm | $Q$ 各维比例、终端代价 $P$、L1 层 |

这个分阶段不是教条，而是有因果依据的：一个解不出来的 OCP 谈不上稳定，一个会发散的闭环谈不上精度。**每次只调一个阶段的旋钮**，把其余固定，否则你无法判断是哪个改动起了作用——这正是 §D2.1 反复强调的"控制变量"思想在调参上的体现。

### 阶段一：让求解器先收敛

第一步永远是把 $Q$、$R$ 设成各维全 1 的单位阵，关掉所有软约束，只保留物理硬约束（推力上下限）。此时你不关心飞得好不好，只关心 `solver.solve()` 是否返回 0。这一阶段的旋钮和判断逻辑如下：

```text
# 阶段一伪代码：逐步收紧，定位求解失败的根因
for dt in [0.10, 0.05, 0.02]:          # 步长由大到小
    for N in [10, 20, 30]:              # 时域由短到长
        fails = run_closed_loop(dt, N, Q=I, R=I, steps=1000)
        if fails == 0:
            print(f"可行配置: dt={dt}, N={N}")  # 记录最宽松的可行点
            break
```

为什么按这个顺序扫？因为求解失败的根因有明确的优先级：

- **积分发散（status$\ne$0 且 res_eq 爆炸）**：$\Delta t$ 太大，显式 RK 在四旋翼快速旋转动力学上不稳定。先减小 $\Delta t$，再考虑换隐式积分器（acados 的 `IRK`）。这与 §D2.1 离散化小节的结论一致——四元数动力学对步长敏感。
- **QP 不可行（status=2）**：约束矛盾。最常见是参考轨迹本身要求 > 4g 的加速度，而推力上限做不到。此时**先放松约束**（把推力上限临时设到 10g）确认问题确实出在约束，再回头修参考轨迹。
- **达到最大迭代（status=1）**：暖启动没生效。检查是否每步都把上一步的解 `set` 进了求解器——冷启动是 RTI 框架下最隐蔽的性能杀手（§D2.4）。

> **反事实**：如果不做阶段一就直接调 $Q/R$，你会把"求解器数值发散"误判成"权重没调好"，于是越调越乱——因为问题根本不在权重层。先收敛，是一切的前提。

### 阶段二：让闭环稳下来

求解器稳定返回 0 之后，才开始动 $Q$ 和 $R$ 的**整体量级**（注意：此阶段只调量级，不调各维之间的比例）。核心矛盾是一对此消彼长的力：

- $Q$ 整体偏大 → 控制器"急功近利"，为了压位置误差而猛打控制量 → 抖动、甚至激发未建模的机架弹性模态。
- $R$ 整体偏大 → 控制器"畏手畏脚"，输出软绵绵 → 跟踪迟钝、悬停漂移。

调法是固定 $R=I$，让 $Q$ 的整体缩放因子 $\alpha$ 从 1 开始，每次乘 10 往上加，直到悬停刚好出现可见抖动，然后退回一档。这给出 $Q$ 量级的上界。一个经验起点见下表（位置/速度/姿态分组给值，组内比例阶段三再细调）：

| 状态分量 | 起始权重量级 | 经验依据 |
|---------|------------|---------|
| 位置 $p$ | $10^1 \sim 10^2$ | 主要跟踪目标，权重最高 |
| 速度 $v$ | $10^0 \sim 10^1$ | 提供阻尼，抑制超调 |
| 姿态 $q$ | $10^0$ | 姿态是内环职责，MPC 不必抢 |
| 推力 $f$ | $R \sim 10^0$ | 太大则迟钝 |
| 体率 $\omega$ | $R \sim 10^{-1}$ | 通常给小值，留出机动余地 |

这里要警惕 §D2.1 与故障排查 F1 反复点名的陷阱：**各维权重的数量级跨度不要超过 3-4 个**。若位置给 $10^6$、体率给 $10^{-2}$，$Q$ 的条件数就爆到 $10^8$，QP 在数值上就病态了——这不是"飞得不好"，而是"算得不准"。

时域长度 $N$ 在本阶段的作用是"看多远"。$N \cdot \Delta t$ 就是预测时域的物理时长，它必须**长于系统最慢的动态时间常数**。四旋翼悬停附近约 0.3-0.5 s 量级，所以 $N \cdot \Delta t \approx 1$ s 是常见安全值。太短（比如 0.2 s）会导致急转弯前来不及减速——这正是 F2 里"原因 D"描述的现象。

### 阶段三：把精度榨出来

闭环稳了，才轮到精度。这一阶段调的是 $Q$ **各维之间的比例**和**终端代价 $P$**，外加按需引入 §D2.5 的 L1 层。判断标准从"悬停漂移"升级为"激进轨迹下的跟踪误差"。典型操作顺序：

```text
# 阶段三：诊断驱动的精度调优
err = run_aggressive_trajectory()      # 跑一条 lemniscate / 高速绕圈
diagnose(err):
    if 误差集中在转弯处:               # 弯道甩出去
        ↑ 提高位置 Q，或 ↑ N（看更远）
    if 误差集中在直线段末端:            # 终点冲过头
        ↑ 终端代价 P（用 LQR 的 Riccati 解作为 P 起点）
    if 误差随时间整体漂移:             # 系统性偏置
        引入 L1 自适应层（§D2.5），它专治模型失配
    if 误差是高频抖动:                # 控制器自激
        ↓ Q 或 ↑ R，回退一点激进度
```

终端代价 $P$ 值得单独一提。它的物理意义是"时域窗口之外那段未来的代价摘要"（§D2.4 提过它和 SLAM 边缘化先验同源）。一个有理论依据的好起点是：用线性化系统在工作点解一次离散 LQR，把得到的 Riccati 矩阵 $P_{\text{LQR}}$ 直接当终端代价。这能显著改善短时域 MPC 的稳定裕度——相当于"用 LQR 的无穷时域智慧补上 MPC 有限时域的近视"。

> **类比（标注边界）**：三阶段调参像调一台旧收音机。阶段一是**接通电源**（没电什么都免谈）；阶段二是**对准频段**（先听到声音，哪怕有杂音）；阶段三是**微调旋钮去噪**（追求音质）。像的地方：严格的先后依赖，跳步必乱。不像的地方：收音机各旋钮基本独立，而 MPC 的 $Q/R/N/P$ 互相耦合——所以才更要靠"每次只动一个"来解耦。

### 调参速查卡

把上面三阶段压缩成一张随手可查的"症状→动作"卡片：

| 你观察到的现象 | 最可能的根因 | 第一个该拧的旋钮 |
|--------------|------------|----------------|
| 求解器频繁 status$\ne$0 | 阶段一未过：$\Delta t$ 太大 / 约束矛盾 | 减小 $\Delta t$；放松约束定位 |
| 悬停就抖 | $Q$ 整体太大 | $Q$ 整体 ÷10 |
| 悬停慢慢漂走 | $R$ 太大 / 有恒定扰动 | $R$ ÷10；加 L1 层 |
| 直线还行，转弯甩出去 | 时域太短或位置权重不足 | ↑ $N$；↑ 位置 $Q$ |
| 终点总冲过头 | 终端代价太弱 | $P \leftarrow P_{\text{LQR}}$ |
| 误差整体有固定偏置 | 模型失配（载荷/风） | 引入 L1（§D2.5）|
| QP 数值告警、解不稳定 | $Q$ 条件数过大 | 压缩各维量级跨度到 $\le 10^4$ |

---

## 故障排查手册（续）：部署与实时性场景

前面 F1-F5 覆盖的是"算法本身"的故障——求解失败、跟踪变差、L1 振荡。但当 MPC 从仿真器搬到真机、从单机跑到嵌入式板子上，会冒出另一类故障：它们与控制理论无关，纯粹是**实时系统工程**的坑。这一组 F6-F9 专门收录这类"代码没错、理论没错，但就是飞不起来"的情形。诊断思路和 F1-F5 一致：症状先行，再沿可能原因逐条排除。

### F6. 仿真飞得好，上真机就飘

```text
症状：同一套 MPC 在仿真里误差 5 mm，换到真机上误差 50 mm+ 甚至炸机

诊断步骤：
  1. 对比仿真与真机的"控制延迟"：从测量时刻到控制生效的总时延
  2. 检查 IMU/位置估计的更新率是否真的达到设定频率
  3. 录一段真机数据回放进仿真——误差是否复现？

常见原因与修复：
  ├─ 原因 A: 通信/计算延迟未建模——仿真假设零延迟
  │  现象: 控制总慢半拍，高速时表现为相位滞后、绕圈外扩
  │  修复: 测出端到端延迟 t_d，用 x0_comp = f_d(x0, u_last, t_d) 把
  │        初始状态前推一个延迟周期再喂给 MPC（延迟补偿）
  │
  ├─ 原因 B: 状态估计噪声远大于仿真——真机 VIO/光流抖
  │  现象: 控制量跟着估计噪声一起高频抖
  │  修复: 估计端加滤波；MPC 端适当 ↑R 以平滑控制
  │
  └─ 原因 C: 推力-油门映射不准——仿真用理想 f，真机油门非线性
     现象: 悬停油门点不对，整体上飘或下坠
     修复: 实测电机推力曲线，标定 thrust = a·throttle² + b·throttle
```

> **本质洞察**：仿真到真机的鸿沟，九成不是"控制器不行"，而是"仿真里那些被默认为零的东西在真机上不为零"——延迟、噪声、执行器非线性。调真机的第一步永远是**把这三样测出来补进模型**，而不是回去重调 $Q/R$。

### F7. MPC 求解时间偶发性超时

```text
症状：平均求解 0.3 ms，但每隔几秒突然有一次 5 ms+，控制周期被打爆

诊断步骤：
  1. 直方图统计 solver.get_stats("time_tot")——是否长尾分布？
  2. 检查这些尖峰是否与 qp_iter 暴增同时发生
  3. 排查是否有内存分配、日志 I/O、GC 混在控制线程里

常见原因与修复：
  ├─ 原因 A: 某些时刻 QP 变难（接近约束边界）→ 迭代数暴涨
  │  修复: 固定最大 QP 迭代数 + RTI 单步——宁可这一步次优，
  │        也不让它拖垮实时性（这正是 RTI 的设计哲学，§D2.4）
  │
  ├─ 原因 B: 控制线程里偷偷做了动态内存分配
  │  现象: 尖峰与代码里 std::vector 扩容、new 同步出现
  │  修复: 控制循环内零分配——预分配缓冲，用 Eigen::Map 包装（§D2.2）
  │
  └─ 原因 C: 进程未设实时优先级，被操作系统调度抢占
     现象: 尖峰随机出现，与算法无关
     修复: chrt -f 设 SCHED_FIFO；隔离 CPU 核；关掉该核的节能降频
```

### F8. 加了 GP/神经网络后求解变慢且偶发不收敛

```text
症状：纯 MPC 0.3 ms 收敛，接入 GP-MPC（§D2.6）或 Neural-MPC（§D2.7）后
     求解时间涨到几 ms，且偶尔 status≠0

诊断步骤：
  1. 单独 profile 残差模型的前向 + 雅可比求值耗时
  2. 检查学习模型的雅可比是否处处良态（神经网络可能给出剧烈梯度）
  3. 关掉残差项，确认是否回到正常——隔离问题来源

常见原因与修复：
  ├─ 原因 A: GP 用了全数据而非稀疏近似——推理 O(n³)
  │  修复: 改稀疏 GP，诱导点 M 取 20-50，推理降到 O(M²)（§D2.6）
  │
  ├─ 原因 B: 神经网络雅可比剧烈变化，破坏 SQP 的局部凸近似
  │  现象: SQP 步长被迫极小，迭代数飙升甚至发散
  │  修复: 训练时加梯度/谱归一化约束；用 l4casadi 的精确雅可比而非
  │        有限差分（§D2.7）；网络规模够用即可，别过大
  │
  └─ 原因 C: 残差模型外推到训练分布之外，输出无意义
     现象: 飞到没见过的工况时残差给出离谱补偿
     修复: 用 GP 的方差或网络不确定性做置信门控，低置信时退回名义模型
```

### F9. 体率指令发出去了，飞机响应不对

```text
症状：MPC 输出的 CTBR（集合推力 + 体率）看起来合理，但飞机实际动作不符

诊断步骤：
  1. 核对坐标系约定：MPC 的 body 系与飞控期望的是否一致（FLU vs FRD）
  2. 核对单位：体率是 rad/s 还是 deg/s？推力是 N 还是归一化 [0,1]？
  3. 用地面台架隔离测试：发固定 CTBR，看姿态环响应方向

常见原因与修复：
  ├─ 原因 A: 坐标系不匹配——MPC 用 FLU，PX4 期望 FRD
  │  现象: 某些轴动作反向，绕一个轴转却另一个轴响应
  │  修复: 在 wrapper 层做一次明确的坐标变换，并写注释固化约定
  │
  ├─ 原因 B: 推力归一化方式不一致
  │  现象: 整体上飘或砸地——悬停推力对不上
  │  修复: 明确飞控期望的是绝对推力还是 [0,1] 油门，做对应换算
  │
  └─ 原因 C: 体率符号/顺序约定不同（(p,q,r) 的轴定义）
     现象: 滚转俯仰耦合错乱
     修复: 严格按飞控文档的轴顺序与正方向重排 MPC 输出
```

> **反事实**：如果不把坐标系/单位约定在 wrapper 层显式写死并加注释，这类 bug 会在每次换飞控、换坐标系时复发，而且极难调试——因为代码"看起来全对"，错的是两个模块之间没说出口的隐含约定。接口处的一行注释，省下的是几个通宵。

### 故障定位总览

把 F1-F9 按"故障归属层"重新组织，便于快速定位该往哪个方向查：

| 故障层 | 典型症状 | 涉及场景 | 首查方向 |
|-------|---------|---------|---------|
| 求解器层 | status$\ne$0、求解超时 | F1, F7 | KKT 残差、QP 迭代数、暖启动 |
| 控制性能层 | 跟踪误差大、振荡 | F2, F3 | 饱和、模型失配、$\omega_c/\Gamma$ |
| 实现/内存层 | Segfault、偶发尖峰 | F4, F7 | 生命周期、零分配、实时优先级 |
| 工具链层 | 编译失败、版本冲突 | F5, F8 | 路径、版本匹配、雅可比良态 |
| 部署接口层 | 仿真好真机飘、动作不对 | F6, F9 | 延迟、坐标系、单位、推力标定 |
