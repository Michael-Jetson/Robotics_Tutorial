> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 54 章 DDP 家族——Crocoddyl 精读 + Aligator/ProxDDP 进阶

> **难度**: ⭐⭐⭐ | **建议时间**: 1.5 周(25-30 小时) | **前置**: 足式/30_Pinocchio深度精读-48 Pinocchio/CppAD, 足式/60_QP_NLP建模 NLP, 足式/80_接触力学与约束优化 接触力学

```
本章定位
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
足式/60_QP_NLP建模 NLP基础 ──→ [足式/100_DDP家族与Crocoddyl DDP家族] ──→ 足式/110_OCS2完整栈与双线程MPC OCS2/SQP
足式/30_Pinocchio深度精读 Pinocchio ──→ [Crocoddyl架构] ──→ 足式/110_OCS2完整栈与双线程MPC 工业级MPC
足式/80_接触力学与约束优化 接触力学 ──→ [接触动力学OC] ──→ 足式/120_步态管理与接触序列 步态管理
02_C++基础与进阶/10_Eigen CRTP    ──→ [虚函数反例]   ──→ 设计决策案例
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## 前置自测

📋 前置自测（答不出 ≥ 2 题 → 先回 足式/60_QP_NLP建模 QP/NLP 和 足式/30_Pinocchio深度精读 Pinocchio 复习）

1. NLP 的 KKT 条件包含哪些方程？拉格朗日乘子的物理意义是什么？
2. Newton 法求解非线性方程组时，为什么需要 Hessian 矩阵？Gauss-Newton 用什么近似？
3. Pinocchio 的 RNEA 和 ABA 分别计算什么？计算复杂度各是多少？
4. 什么是"shooting method"？它与"collocation method"的核心区别是什么？
5. 为什么控制问题的时间离散化会引入"动力学间隙(gap)"？

## 本章目标

学完本章,学员应能:

1. **写出 DDP 的 backward/forward pass 完整伪代码**——理解每一步数学推导
2. **区分 DDP / iLQR / FDDP 三者的数学差异**——知道何时选哪种
3. **说清控制限位为什么必须进 backward pass**——理解 box-DDP 的 projected-Newton 与 Box-FDDP 双模式
4. **解释 Crocoddyl 为什么选虚函数而不是 CRTP**——02_C++基础与进阶/10_Eigen 的精彩反例
5. **用 Crocoddyl 搭建四足 trot 轨迹优化**——从 ActionModel 到 Solver 全流程
6. **理解 ProxDDP 的增广拉格朗日框架**——约束 DDP 的前沿方法
7. **复述 ParallelRiccati 的 parallel scan 思想**——打破"backward pass 不可并行"的教条

## 前置依赖

```
必修前置(v8 主线)              本课程前置
┌─────────────────────┐    ┌──────────────────────┐
│ 02_C++基础与进阶/10_Eigen 多态+CRTP│    │ 足式/30_Pinocchio深度精读 Pinocchio    │
│ 02_C++基础与进阶/10_Eigen         │    │ 足式/60_QP_NLP建模 NLP/QP 基础     │
│ 02_C++基础与进阶/30_并发与实时     │    │ 足式/70_腿足简化模型理论 简化模型+接触 │
│ 02_C++基础与进阶/10_Eigen Concepts│    │ 足式/90_WBC分层优化与TSID WBC/TSID  │
└─────────────────────┘    └──────────────────────┘
```

## 本章知识导航

本章沿"通用 NLP → 利用结构的 DDP → 算法变体 → 工业级框架 → 前沿并行/约束"这条主线展开,可分为五块。下表给出全部主干小节与它们解决的核心问题,便于按需检索。

| 块 | 小节 | 解决的问题 | 难度 |
|----|------|-----------|------|
| **一、为什么是 DDP** | 54.1 从通用 NLP 到 DDP | Ipopt 为何慢?Markov 结构如何把 $O(N^3)$ 降到 $O(N)$ | ⭐⭐ |
| **二、DDP 的数学内核** | 54.2 Bellman 方程的二次近似 | Q 函数系数推导、控制律、Riccati 递推、正则化(含 54.2.8B 两种正则模式)、2D 小车手算 | ⭐⭐⭐ |
| | 54.3 DDP vs iLQR | 唯一差异(动力学 Hessian 项)、收敛速率、何时重要 | ⭐⭐ |
| **三、面向工程的变体** | 54.4 FDDP | 用 gap 容忍不可行初值,MPC warm-start 的标准选择 | ⭐⭐⭐ |
| | 54.4B Box-DDP / Box-FDDP | 把控制限位塞进 backward pass(projected-Newton + 双模式切换) | ⭐⭐⭐⭐ |
| **四、Crocoddyl 框架精读** | 54.5 ActionModel/ActionData 架构 | Model-Data 分离、接口、内存布局、线程安全、自定义模型 | ⭐⭐ |
| | 54.6 常见 ActionModel | Differential/Integrated 两层、自由/接触动力学、残差代价、积分器 | ⭐⭐ |
| | 54.7 虚函数 vs CRTP | 为什么应用层框架选虚函数(性能瓶颈不在调度) | ⭐⭐⭐ |
| | 54.8 OpenMP 并行化 | 哪些可并行、加速比、false sharing | ⭐⭐ |
| **五、前沿:约束与并行** | 54.9 Aligator / ProxDDP / ParallelRiccati | 增广拉格朗日约束、$O(\log N)$ 并行 backward pass | ⭐⭐⭐⭐ |
| **实战与对比** | 54.10 四足 trot 实战 | 从 Pinocchio 模型到 FDDP 求解的全流程 Python 代码 | ⭐⭐⭐ |
| | 54.11 DDP 在 MPC 中的使用 | warm-start、不等收敛策略、反馈增益复用 | ⭐⭐⭐ |
| | 54.12 DDP vs SQP | 两大流派的设计哲学分歧与选型 | ⭐⭐⭐ |

**阅读路径建议**:

- **精读(博士/算法工程,25-30 小时)**:按 54.1 → 54.12 顺序通读,完成 A/B/C 三型练习与跨章综合题。54.2.3 的 Q 函数推导、54.4B 的 box-QP、54.9 的 ParallelRiccati 是三个必须吃透的硬骨头。
- **速读(已懂 LQR/DDP,只补 Crocoddyl 工程,8-10 小时)**:略过 54.2 的逐项推导,重点读 54.4(FDDP)、54.4B(控制限位)、54.5-54.8(框架与并行)、54.10(实战代码)。
- **速查(写代码时定位 API)**:直接跳 54.10 实战代码 + 章末"API 速查表"+"故障排查手册",遇到不懂的概念再回溯对应小节。

### 如果跳过本章会怎样

- 不理解 DDP 的 Markov 结构,就会把足式 MPC 直接丢给 Ipopt,在 1kHz 控制频率下根本算不过来(54.1)。
- 不懂 FDDP 的 gap 机制和 Box-FDDP 的控制限位处理,MPC warm-start 会频繁发散、电机扭矩越界却无人察觉(54.4、54.4B)。
- 看不懂 Crocoddyl 的 ActionModel/ActionData 分离,就无法为新机器人或新任务自定义动力学与代价(54.5)。

---

## 54.1 从通用 NLP 到 DDP——利用时间结构的特殊解法 ⭐⭐

### 54.1.1 动机:为什么不直接用 Ipopt? ⭐⭐

假设你有一只四足机器人,需要规划未来 0.5 秒的全身运动。最直觉的做法是:把它写成一个大的 NLP(Non-Linear Program),然后扔给通用求解器 Ipopt。

**MPC 的 NLP 形式化**:

$$\min_{\mathbf{x}_0, \mathbf{u}_0, \mathbf{x}_1, \mathbf{u}_1, \dots, \mathbf{x}_N} \sum_{k=0}^{N-1} l_k(\mathbf{x}_k, \mathbf{u}_k) + l_N(\mathbf{x}_N)$$

$$\text{s.t.} \quad \mathbf{x}_{k+1} = f_k(\mathbf{x}_k, \mathbf{u}_k), \quad k = 0, \dots, N-1$$
$$\quad\quad\quad \mathbf{x}_0 = \mathbf{x}_{\text{init}}$$
$$\quad\quad\quad g_k(\mathbf{x}_k, \mathbf{u}_k) \leq 0 \quad \text{(路径约束)}$$

对于 ANYmal 四足机器人:
- 状态维度 $n_x = 37$(浮动基座 7 + 12 关节位置 + 18 速度维度）
- 控制维度 $n_u = 12$（关节扭矩）
- 时间步 $N = 50$

**NLP 决策变量总数**: $(n_x + n_u) \times N + n_x = (37 + 12) \times 50 + 37 = 2487$ 维。

```
NLP 求解器眼中的问题:
┌───────────────────────────────────────────────────────┐
│  2487 个决策变量,全部耦合                              │
│  50 组动力学等式约束(每组 37 维)                       │
│  路径约束(关节限位、摩擦锥等)                          │
│  Ipopt: "我看到一个大矩阵,我用内点法迭代"              │
│  → KKT 矩阵: 2487×2487                               │
│  → 每次迭代需要分解这个矩阵                            │
│  → 对于 1kHz 的 MPC? 太慢了!                          │
└───────────────────────────────────────────────────────┘
```

> ⚠️ **陷阱**: 初学者常问"为什么不直接用 Ipopt?"答案是:Ipopt 不利用问题的时间结构。它把所有时间步的变量混在一起求解,KKT 矩阵虽然是稀疏的,但分解代价仍然是 $O(N^3 \cdot n^3)$ 量级(带状结构只能降到 $O(N \cdot n^6)$)。

### 54.1.2 DDP 的 Markov 洞察 ⭐⭐

DDP 看到了一个 Ipopt 没看到的结构:**Markov 性**。

**关键观察**:在动力学约束 $\mathbf{x}_{k+1} = f_k(\mathbf{x}_k, \mathbf{u}_k)$ 下,时间步 $k+1$ 的状态**只取决于**时间步 $k$ 的状态和控制。时间步 $k+1$ 的最优决策**不需要知道** $k-1$ 发生了什么——这就是 Markov 性。这好比一场接力赛:每位选手只需要知道接棒时自己的位置和速度(当前状态),不需要知道前面的选手是如何跑到这里的(历史轨迹)。Ipopt 则相当于让所有选手同时上场商量最优策略,忽略了"信息只沿时间方向传递"这一天然结构。

```
Ipopt 眼中的结构         DDP 眼中的结构
┌─────────────┐         ┌───┐ ┌───┐ ┌───┐     ┌───┐
│ 一个大矩阵   │         │ 0 │→│ 1 │→│ 2 │→...→│ N │
│ 全部耦合     │         └───┘ └───┘ └───┘     └───┘
│ O(N³·n³)    │         │链式│链式│链式│       │
└─────────────┘         └───┘ 每步独立的小问题
                        O(N · n³) ← 线性于 N!
```

**形式化**:利用 Bellman 最优性原理,定义价值函数:

$$V_k(\mathbf{x}) = \min_{\mathbf{u}_k, \dots, \mathbf{u}_{N-1}} \left[ \sum_{j=k}^{N-1} l_j(\mathbf{x}_j, \mathbf{u}_j) + l_N(\mathbf{x}_N) \right]$$

满足递归关系:

$$V_k(\mathbf{x}) = \min_{\mathbf{u}} \left[ l_k(\mathbf{x}, \mathbf{u}) + V_{k+1}(f_k(\mathbf{x}, \mathbf{u})) \right]$$

这把一个 2487 维的优化分解成了 50 个**局部子问题**,每个只涉及 $(n_x + n_u) = 49$ 维。

### 54.1.3 复杂度对比 ⭐⭐

| 方法 | 单次迭代复杂度 | N=50, $n_x$=37 时间估计 |
|------|-------------|----------------------|
| Ipopt (稠密) | $O(N^3 \cdot n_x^3)$ | ~100 ms |
| Ipopt (利用带状稀疏) | $O(N \cdot n_x^6)$ | ~10 ms |
| **DDP** | $O(N \cdot (n_x^3 + n_u^3))$ | **~2 ms** |

> 💡 **洞察**: DDP 的复杂度对 N 是**线性**的。这意味着延长规划时域(增大 N)的代价是可控的。而 Ipopt 的代价随 N 超线性增长。

### 54.1.4 历史脉络 ⭐

```
时间线:
1966 ─ Mayne ─ 提出 DDP(连续时间); 1970 ─ Jacobson & Mayne 出版专著
        │
2002 ─ Todorov ─ 把最优控制用于生物运动控制(Nature Neuroscience)
        │
2004 ─ Li & Todorov ─ 提出 iLQR(DDP 的简化版)
        │
2014 ─ Tassa et al. ─ 带控制限位的 box-DDP(ICRA 2014; MuJoCo)
        │
2020 ─ Mastalli et al. ─ Crocoddyl(ICRA 2020,arXiv 1909.04947)
        │
2022 ─ Mastalli et al. ─ Box-FDDP(Autonomous Robots 46(8):985-1005)
        │
2025 ─ Jallet et al. ─ ProxDDP(T-RO 41:2605-2624)+ ParallelRiccati(RSS 2024)+ Aligator 库
```

> 🧠 **深入理解**: Jacobson 1970 年的 DDP 论文是一本书,不是一篇文章!那时候的"论文"可以是 200 页。现代 DDP 的实用化是 2000 年代 Todorov 的工作推动的——他在 MuJoCo 里把 iLQR 变成了实用工具。

**练习 54.1a** (⭐): 给定 $n_x = 12, n_u = 6, N = 30$,分别计算 DDP 和稠密 Ipopt 的单次迭代 FLOP 估计(用大 O 量级比较即可)。

**练习 54.1b** (⭐⭐): 假设 Ipopt 每次迭代需要 5ms,需要 20 次迭代收敛;DDP 每次迭代需要 1ms,需要 10 次迭代。MPC 要求 5ms 内返回,哪种方案可行?如果允许"warm start"让迭代次数从 10 降到 3,结论是否改变?

---

## 54.2 DDP 的数学结构——Bellman 方程的二次近似 ⭐⭐⭐

### 54.2.1 问题设定 ⭐⭐

**无约束离散时间最优控制**:

$$\min_{\mathbf{u}_0, \dots, \mathbf{u}_{N-1}} J = \sum_{k=0}^{N-1} l_k(\mathbf{x}_k, \mathbf{u}_k) + l_N(\mathbf{x}_N)$$

$$\text{s.t.} \quad \mathbf{x}_{k+1} = f_k(\mathbf{x}_k, \mathbf{u}_k)$$

其中 $l_k$ 是运行代价(running cost),$l_N$ 是终端代价(terminal cost),$f_k$ 是离散动力学。

DDP 通过迭代求解:从一条初始轨迹 $(\bar{\mathbf{x}}_0, \bar{\mathbf{u}}_0, \bar{\mathbf{x}}_1, \dots, \bar{\mathbf{x}}_N)$ 出发,反复改进。

### 54.2.2 Q 函数与二次近似 ⭐⭐⭐

在当前轨迹 $(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k)$ 附近,定义扰动 $\delta \mathbf{x} = \mathbf{x} - \bar{\mathbf{x}}_k$, $\delta \mathbf{u} = \mathbf{u} - \bar{\mathbf{u}}_k$。

**行动-价值函数(Q 函数)**:

$$Q_k(\delta \mathbf{x}, \delta \mathbf{u}) = l_k(\bar{\mathbf{x}}_k + \delta \mathbf{x}, \bar{\mathbf{u}}_k + \delta \mathbf{u}) + V_{k+1}(f_k(\bar{\mathbf{x}}_k + \delta \mathbf{x}, \bar{\mathbf{u}}_k + \delta \mathbf{u}))$$

对 $Q_k$ 做二阶 Taylor 展开:

$$Q_k(\delta \mathbf{x}, \delta \mathbf{u}) \approx Q_0 + \begin{bmatrix} Q_\mathbf{x} \\ Q_\mathbf{u} \end{bmatrix}^T \begin{bmatrix} \delta \mathbf{x} \\ \delta \mathbf{u} \end{bmatrix} + \frac{1}{2} \begin{bmatrix} \delta \mathbf{x} \\ \delta \mathbf{u} \end{bmatrix}^T \begin{bmatrix} Q_{\mathbf{xx}} & Q_{\mathbf{xu}} \\ Q_{\mathbf{ux}} & Q_{\mathbf{uu}} \end{bmatrix} \begin{bmatrix} \delta \mathbf{x} \\ \delta \mathbf{u} \end{bmatrix}$$

### 54.2.3 Q 函数展开系数的完整推导 ⭐⭐⭐

这是本章最核心的推导,**一步不跳**。

设动力学的 Jacobian 和 Hessian 为:

$$\mathbf{f}_\mathbf{x} = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k}, \quad \mathbf{f}_\mathbf{u} = \frac{\partial f}{\partial \mathbf{u}}\bigg|_{\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k}$$

动力学二阶导(Tensor):$\mathbf{f}_{\mathbf{xx}}, \mathbf{f}_{\mathbf{xu}}, \mathbf{f}_{\mathbf{uu}}$ 都是三阶张量。

**推导过程**:

$$Q_\mathbf{x} = l_\mathbf{x} + \mathbf{f}_\mathbf{x}^T V_{\mathbf{x},k+1}$$

> 解读:Q 对 x 的梯度 = 当步代价对 x 的梯度 + 未来价值通过动力学传回的梯度。

$$Q_\mathbf{u} = l_\mathbf{u} + \mathbf{f}_\mathbf{u}^T V_{\mathbf{x},k+1}$$

> 解读:Q 对 u 的梯度 = 当步代价对 u 的梯度 + 未来价值通过动力学传回的梯度。

$$Q_{\mathbf{xx}} = l_{\mathbf{xx}} + \mathbf{f}_\mathbf{x}^T V_{\mathbf{xx},k+1} \mathbf{f}_\mathbf{x} + V_{\mathbf{x},k+1} \cdot \mathbf{f}_{\mathbf{xx}}$$

$$Q_{\mathbf{uu}} = l_{\mathbf{uu}} + \mathbf{f}_\mathbf{u}^T V_{\mathbf{xx},k+1} \mathbf{f}_\mathbf{u} + V_{\mathbf{x},k+1} \cdot \mathbf{f}_{\mathbf{uu}}$$

$$Q_{\mathbf{ux}} = l_{\mathbf{ux}} + \mathbf{f}_\mathbf{u}^T V_{\mathbf{xx},k+1} \mathbf{f}_\mathbf{x} + V_{\mathbf{x},k+1} \cdot \mathbf{f}_{\mathbf{ux}}$$

> ⚠️ **陷阱**: $V_{\mathbf{x},k+1} \cdot \mathbf{f}_{\mathbf{xx}}$ 是一个向量与三阶张量的缩并,结果是一个矩阵。这一项在 iLQR 中被**丢弃**——这正是 DDP 和 iLQR 的核心区别!

```
Q 函数系数结构图:

Q_x  = l_x  + f_x^T · V_x(k+1)          ← 一阶链式
Q_u  = l_u  + f_u^T · V_x(k+1)          ← 一阶链式

Q_xx = l_xx + f_x^T·V_xx·f_x + V_x·f_xx  ← 二阶 = GN项 + 动力学Hessian项
Q_uu = l_uu + f_u^T·V_xx·f_u + V_x·f_uu  ← 二阶 = GN项 + 动力学Hessian项
Q_ux = l_ux + f_u^T·V_xx·f_x + V_x·f_ux  ← 二阶 = GN项 + 动力学Hessian项

       ↑         ↑              ↑
    代价Hessian  Gauss-Newton项  DDP独有项(iLQR丢弃)
```

### 54.2.4 最优控制律 ⭐⭐

对 $\delta \mathbf{u}$ 求极值:$\frac{\partial Q}{\partial \delta \mathbf{u}} = 0$

$$Q_\mathbf{u} + Q_{\mathbf{uu}} \delta \mathbf{u} + Q_{\mathbf{ux}} \delta \mathbf{x} = 0$$

$$\boxed{\delta \mathbf{u}^* = \underbrace{-Q_{\mathbf{uu}}^{-1} Q_\mathbf{u}}_{\mathbf{k}_k \text{ (前馈项)}} + \underbrace{(-Q_{\mathbf{uu}}^{-1} Q_{\mathbf{ux}})}_{\mathbf{K}_k \text{ (反馈增益)}} \delta \mathbf{x}}$$

> 💡 **洞察**: DDP 的 backward pass 不仅给出了最优轨迹修正,还给出了**反馈增益** $\mathbf{K}_k$。这是 DDP 相对于 SQP 的天然优势——SQP 只给出开环轨迹,而 DDP 给出闭环控制策略。

### 54.2.5 价值函数递推(Riccati-like) ⭐⭐⭐

把最优 $\delta \mathbf{u}^*$ 代回 Q 函数,得到 $V_k$ 的二次近似系数:

$$V_{\mathbf{x},k} = Q_\mathbf{x} + \mathbf{K}_k^T Q_{\mathbf{uu}} \mathbf{k}_k + \mathbf{K}_k^T Q_\mathbf{u} + Q_{\mathbf{ux}}^T \mathbf{k}_k$$

简化后:

$$\boxed{V_{\mathbf{x},k} = Q_\mathbf{x} - Q_{\mathbf{ux}}^T Q_{\mathbf{uu}}^{-1} Q_\mathbf{u}}$$

$$\boxed{V_{\mathbf{xx},k} = Q_{\mathbf{xx}} - Q_{\mathbf{ux}}^T Q_{\mathbf{uu}}^{-1} Q_{\mathbf{ux}}}$$

> 🧠 **深入理解**: 这与离散时间 Riccati 方程**形式相同**!当代价是二次的、动力学是线性的时,DDP 退化为精确的 LQR。这就是为什么 DDP 也叫"迭代 LQR"的原因。

### 54.2.6 Backward Pass 完整伪代码 ⭐⭐

```
算法: DDP Backward Pass
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入: 当前轨迹 {x̄₀, ū₀, ..., x̄_N}
输出: 控制律 {(k₀,K₀), ..., (k_{N-1},K_{N-1})}
      价值函数 {V_x,k, V_xx,k}

1. 初始化:
   V_x,N = l_N,x(x̄_N)           // 终端代价梯度
   V_xx,N = l_N,xx(x̄_N)         // 终端代价 Hessian

2. For k = N-1 downto 0:
   a. 计算动力学 Jacobian: f_x, f_u
   b. 计算动力学 Hessian: f_xx, f_uu, f_ux  ← DDP独有
   c. 计算代价梯度/Hessian: l_x, l_u, l_xx, l_uu, l_ux

   d. 组装 Q 函数系数:
      Q_x  = l_x  + f_x^T · V_x,k+1
      Q_u  = l_u  + f_u^T · V_x,k+1
      Q_xx = l_xx + f_x^T · V_xx,k+1 · f_x + V_x,k+1 · f_xx
      Q_uu = l_uu + f_u^T · V_xx,k+1 · f_u + V_x,k+1 · f_uu
      Q_ux = l_ux + f_u^T · V_xx,k+1 · f_x + V_x,k+1 · f_ux

   e. [可选] 正则化: Q_uu += μI  (Levenberg-Marquardt)

   f. 计算控制律:
      k_k = -Q_uu⁻¹ · Q_u
      K_k = -Q_uu⁻¹ · Q_ux

   g. 更新价值函数:
      V_x,k  = Q_x  - Q_ux^T · Q_uu⁻¹ · Q_u
      V_xx,k = Q_xx - Q_ux^T · Q_uu⁻¹ · Q_ux

3. 返回 {(k_k, K_k)}, {V_x,k, V_xx,k}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 54.2.7 Forward Pass 与 Line Search ⭐⭐

Backward pass 给出了修正方向,forward pass 执行修正:

$$\mathbf{u}_k^{\text{new}} = \bar{\mathbf{u}}_k + \alpha \mathbf{k}_k + \mathbf{K}_k (\mathbf{x}_k^{\text{new}} - \bar{\mathbf{x}}_k)$$

$$\mathbf{x}_{k+1}^{\text{new}} = f_k(\mathbf{x}_k^{\text{new}}, \mathbf{u}_k^{\text{new}})$$

其中 $\alpha \in (0, 1]$ 是 line search 步长。

**Armijo 条件**(判断是否接受这一步):

$$J(\text{new}) \leq J(\text{old}) + c_1 \alpha \Delta J_{\text{expected}}$$

其中预期代价下降为:

$$\Delta J_{\text{expected}} = \sum_{k=0}^{N-1} \left( \alpha \mathbf{k}_k^T Q_{\mathbf{u},k} + \frac{\alpha^2}{2} \mathbf{k}_k^T Q_{\mathbf{uu},k} \mathbf{k}_k \right)$$

如果不做 line search 会怎样?直接取 $\alpha = 1$(全步长)在强非线性问题上极容易发散——backward pass 给出的搜索方向是基于**二次近似**的,离当前点越远近似越差。没有 line search 保护,一次糟糕的全步长更新就可能让状态轨迹飞到物理不可能的区域(例如关节角超出限位 10 倍),之后的动力学计算产生 NaN,整个优化崩溃。

**典型的 line search 策略**:$\alpha$ 从 1 开始,如果 Armijo 条件不满足就乘以 0.5,最多缩减 10 次。

```
Forward Pass 流程:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
α = 1.0
while α > α_min:
  x₀_new = x₀  (初始状态不变)
  for k = 0 to N-1:
    u_k_new = ū_k + α·k_k + K_k·(x_k_new - x̄_k)
    x_{k+1}_new = f(x_k_new, u_k_new)
  end
  if J_new ≤ J_old + c₁·α·ΔJ_expected:
    接受新轨迹
    break
  else:
    α *= 0.5  (回退)
end
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 54.2.8 Levenberg-Marquardt 正则化 ⭐⭐⭐

当 $Q_{\mathbf{uu}}$ 不正定时(可能在远离最优解时发生),需要正则化:

$$Q_{\mathbf{uu}}^{\text{reg}} = Q_{\mathbf{uu}} + \mu \mathbf{I}$$

$\mu$ 的调节策略:
- 如果 forward pass 成功(代价下降):$\mu \leftarrow \mu / \nu$,其中 $\nu > 1$
- 如果 forward pass 失败:$\mu \leftarrow \mu \cdot \nu$

> ⚠️ **陷阱**: 正则化参数 $\mu$ 如果太大,DDP 退化为梯度下降(每步只走一小步);如果太小,可能 $Q_{\mathbf{uu}}$ 不正定导致数值崩溃。Crocoddyl 的默认策略是 $\nu = 10$,初始 $\mu = 10^{-9}$。

如果不做正则化会怎样?当 $Q_{uu}$ 有负特征值时,Riccati 回退的控制增益 $\mathbf{k} = -Q_{uu}^{-1} Q_u$ 会指向代价**增大**的方向——forward pass 不仅不降低代价,反而让轨迹发散。更糟糕的是,$Q_{uu}$ 的 Cholesky 分解会直接失败(负对角元素),整个算法崩溃。正则化 $\mu I$ 的作用就是在 $Q_{uu}$ 的特征值上加一个正偏移,把所有特征值推到正数区域,代价是步长变保守——但至少保证了算法不崩溃。

### 54.2.8B 两种正则化模式:control-reg vs state-reg ⭐⭐⭐

上面给的 $Q_{\mathbf{uu}} + \mu\mathbf{I}$ 只是**最基础**的一种正则化(control regularization,控制正则)。Crocoddyl/Box-FDDP 实际实现了**两种**正则化模式,它们加在不同的地方,几何含义也不同。理解这个区别,是读懂 `solver.reg_min`、`solver.ureg`、`solver.xreg` 这些字段的前提。

**模式一:control regularization(ureg)。** 直接在控制 Hessian 上加偏移:

$$\tilde{Q}_{\mathbf{uu}} = Q_{\mathbf{uu}} + \mu\mathbf{I}_{n_u}$$

这等价于在代价里隐式加了一项 $\frac{\mu}{2}\|\delta\mathbf{u}\|^2$——惩罚控制修正过大。它只改 $Q_{\mathbf{uu}}$,实现最简单,但当**动力学本身**(而非代价)导致 $Q_{\mathbf{uu}}$ 病态时,可能需要很大的 $\mu$ 才压得住。

**模式二:state regularization(xreg)。** Tassa(2014)和 Crocoddyl 的默认做法是把正则化加在**下一步价值函数的 Hessian** 上,即在计算 $Q$ 系数时用 $V_{\mathbf{xx},k+1} + \mu\mathbf{I}_{n_x}$ 替换 $V_{\mathbf{xx},k+1}$:

$$\tilde{Q}_{\mathbf{uu}} = l_{\mathbf{uu}} + \mathbf{f}_\mathbf{u}^T (V_{\mathbf{xx},k+1} + \mu\mathbf{I})\mathbf{f}_\mathbf{u} + V_{\mathbf{x},k+1}\cdot\mathbf{f}_{\mathbf{uu}}$$

$$\tilde{Q}_{\mathbf{ux}} = l_{\mathbf{ux}} + \mathbf{f}_\mathbf{u}^T (V_{\mathbf{xx},k+1} + \mu\mathbf{I})\mathbf{f}_\mathbf{x} + V_{\mathbf{x},k+1}\cdot\mathbf{f}_{\mathbf{ux}}$$

注意此时正则化**同时影响** $Q_{\mathbf{uu}}$ 和 $Q_{\mathbf{ux}}$,因此也影响反馈增益 $\mathbf{K}=-\tilde Q_{\mathbf{uu}}^{-1}\tilde Q_{\mathbf{ux}}$。

> 💡 **洞察**:control-reg 只让**前馈** $\mathbf{k}$ 变保守,反馈结构 $\mathbf{K}$ 几乎不变;state-reg 同时让前馈和反馈都变保守,相当于"假装离最优解更近的轨迹也有不确定性"。Tassa 发现 state-reg 在强非线性问题上更鲁棒——因为它正则化的是"未来代价对状态的敏感度",这正是 trust-region 思想:不信任远处的二次近似,就把那里的曲率信息打个折扣。

```
两种正则化的几何对比:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
control-reg:  在控制空间画一个信任域
   → δu 不能太大,但 δu 怎样响应 δx 不变
   → 适合:代价 Hessian 病态

state-reg:    在状态空间画一个信任域
   → 不信任 V_xx(k+1) 的远处曲率
   → 同时收缩前馈和反馈
   → 适合:动力学强非线性(接触、碰撞)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**自适应调节(基于 Cholesky 失败)。** 实际代码并不去算特征值(太贵),而是**直接尝试 Cholesky 分解 $\tilde Q_{\mathbf{uu}} = LL^T$**:若成功则 $\tilde Q_{\mathbf{uu}}$ 正定,继续;若失败(出现非正对角元),立刻按 $\mu\leftarrow\mu\cdot\nu$ 放大正则化并重试当前时间步。这是 backward pass 内部的一个小循环:

```
backward pass 内的正则化自适应(每个时间步):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
loop:
  组装 Q_uu(含 μ 正则)
  L = cholesky(Q_uu)
  if L 成功:  break        # Q_uu 正定,放行
  else:       μ *= ν       # 放大正则,重试
              if μ > μ_max: 报错"无法正定化"
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
整次迭代结束后:
  forward pass 成功 → μ /= ν (下次更激进)
  forward pass 失败 → μ *= ν (下次更保守)
```

> ⚠️ **数值陷阱**:正则化下界 `reg_min` 不能设成 0。即便理论上 $Q_{\mathbf{uu}}$ 正定,浮点运算下接近奇异时 Cholesky 仍可能产生巨大的 $L^{-1}$,放大舍入误差。Crocoddyl 默认 `reg_min` $=10^{-9}$,给数值留一个"地板"。把它强行设为 0 在接触切换的尖锐时刻几乎必然触发 NaN。

> ⚠️ **思维陷阱**:正则化让代价下降"变慢"不等于"变差"。新手看到加了 $\mu$ 后单步下降变小,常误以为正则化有害而把它关掉,结果在难题上直接发散。正确心态:正则化是**保险**,平时几乎不花钱($\mu$ 自适应缩到很小),关键时刻($Q_{\mathbf{uu}}$ 失正定)救命。这与 足式/60_QP_NLP建模 里 trust-region 与 line-search 互补的论述一致——正则化调"方向是否可信",line search 调"沿该方向走多远"。

### 54.2.9 数值例子:2D 小车 (手算可追踪) ⭐⭐

**系统**:$\mathbf{x} = [p, v]^T$,$u$ 是加速度。

$$f(\mathbf{x}, u) = \begin{bmatrix} p + v \cdot dt \\ v + u \cdot dt \end{bmatrix}, \quad dt = 0.1$$

**代价**: $l_k = \frac{1}{2} u^2 \cdot R$, $l_N = \frac{1}{2} \|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\|^2_{\mathbf{Q}_f}$

其中 $R = 0.01$, $\mathbf{Q}_f = \text{diag}(100, 10)$, $\mathbf{x}_{\text{goal}} = [1, 0]^T$。

**Backward pass 第 N-1 步的手算**:

$$\mathbf{f}_\mathbf{x} = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix}, \quad \mathbf{f}_\mathbf{u} = \begin{bmatrix} 0 \\ dt \end{bmatrix} = \begin{bmatrix} 0 \\ 0.1 \end{bmatrix}$$

此系统动力学是线性的,所以 $\mathbf{f}_{\mathbf{xx}} = \mathbf{f}_{\mathbf{uu}} = \mathbf{f}_{\mathbf{ux}} = 0$(DDP 和 iLQR 在此例中完全等价)。

```cpp
// C++ implementation of the 2D car DDP example
#include <Eigen/Dense>
#include <iostream>
#include <vector>

struct DDPResult {
  Eigen::VectorXd k;  // feedforward
  Eigen::MatrixXd K;  // feedback gain
};

int main() {
  const int N = 20;
  const double dt = 0.1;
  const double R = 0.01;
  Eigen::Matrix2d Qf;
  Qf << 100, 0, 0, 10;
  Eigen::Vector2d x_goal(1.0, 0.0);
  
  // Dynamics Jacobians (constant for linear system)
  Eigen::Matrix2d fx;
  fx << 1, dt, 0, 1;
  Eigen::Vector2d fu(0, dt);
  
  // Initialize trajectory: zero controls, propagate from x0 = [0,0]
  std::vector<Eigen::Vector2d> xs(N + 1, Eigen::Vector2d::Zero());
  std::vector<double> us(N, 0.0);
  
  // Backward pass
  Eigen::Vector2d Vx = Qf * (xs[N] - x_goal);  // terminal gradient
  Eigen::Matrix2d Vxx = Qf;                      // terminal Hessian
  
  std::vector<DDPResult> gains(N);
  for (int k = N - 1; k >= 0; --k) {
    // Q-function coefficients
    double Qu = R * us[k] + fu.dot(Vx);
    double Quu = R + fu.transpose() * Vxx * fu;
    Eigen::RowVector2d Qux = fu.transpose() * Vxx * fx;
    Eigen::Vector2d Qx = fx.transpose() * Vx;
    Eigen::Matrix2d Qxx = fx.transpose() * Vxx * fx;
    
    // Optimal gains
    gains[k].k = Eigen::VectorXd::Constant(1, -Qu / Quu);
    gains[k].K = -Qux / Quu;
    
    // Value function update
    Vx = Qx - Qux.transpose() * Qu / Quu;
    Vxx = Qxx - Qux.transpose() * Qux / Quu;
  }
  
  std::cout << "Backward pass complete. Feedback gain K_0:\n"
            << gains[0].K << std::endl;
  return 0;
}
```

> 💡 **洞察**: 对于这个线性-二次问题,DDP 一次迭代就能找到全局最优解——因为它退化为精确的 LQR。非线性问题则需要多次迭代。

**练习 54.2a** (⭐⭐): 扩展上面的代码,加入 forward pass 和 line search,实现完整的 DDP 迭代。让小车从 $[0,0]$ 到达 $[1,0]$,绘制状态轨迹和控制序列。

**练习 54.2b** (⭐⭐⭐): 把动力学改为非线性的(例如加入空气阻力 $\dot{v} = u - 0.1 v^2$),观察 DDP 和 iLQR 的迭代次数差异。

---

## 54.3 DDP vs iLQR——微妙但重要的区别 ⭐⭐

### 54.3.1 精确的数学差异 ⭐⭐⭐

**iLQR (iterative LQR)**,也叫 iLQG (iterative Linear-Quadratic-Gaussian),是 Li & Todorov (2004) 提出的 DDP 简化版。

**唯一的差异**:iLQR **丢弃**了 Q 函数 Hessian 中的**动力学二阶导项**:

```
                    DDP                              iLQR
Q_xx = l_xx + f_x^T·V_xx·f_x + V_x·f_xx    Q_xx = l_xx + f_x^T·V_xx·f_x
Q_uu = l_uu + f_u^T·V_xx·f_u + V_x·f_uu    Q_uu = l_uu + f_u^T·V_xx·f_u
Q_ux = l_ux + f_u^T·V_xx·f_x + V_x·f_ux    Q_ux = l_ux + f_u^T·V_xx·f_x
                                  ↑                                 ↑
                           保留动力学Hessian                 丢弃(=0)
```

**代码级别的差异**(仅这几行不同):

```cpp
// DDP version
Eigen::MatrixXd Qxx = Lxx + fx.transpose() * Vxx_next * fx;
for (int i = 0; i < nx; ++i) {
  Qxx += Vx_next(i) * fxx[i];  // <-- DDP only: tensor contraction
}

// iLQR version (simply omit the tensor contraction)
Eigen::MatrixXd Qxx = Lxx + fx.transpose() * Vxx_next * fx;
// No f_xx term!
```

### 54.3.2 收敛速率分析 ⭐⭐⭐

| 属性 | DDP | iLQR |
|------|-----|------|
| 动力学展开阶数 | 二阶 (Newton) | 一阶 (Gauss-Newton) |
| 理论收敛速率 | **二次** (quadratic) | **超线性** (superlinear) |
| 单步计算代价 | 需要动力学 Hessian | 不需要 |
| 数值稳定性 | 动力学 Hessian 可能导致 $Q_{uu}$ 不正定 | 更稳定(GN 近似天然半正定）|

> 🧠 **深入理解**: 理论上 iLQR 只有超线性收敛(Roulet et al., ICML 2019 证明了 iLQR/iDDP 的局部线性收敛保证),但实践中观察到的收敛速度接近 DDP,因为对大多数机器人问题,动力学 Hessian 项的贡献相对较小。

> **本质洞察**:DDP 和 iLQR 的差异本质上是**Newton 法 vs Gauss-Newton 法**在最优控制中的投影。Gauss-Newton 用 $J^TJ$ 近似 Hessian,天然半正定且无需二阶导数;Newton 法用精确 Hessian,收敛更快但可能不正定。这与 足式/60_QP_NLP建模 中非线性最小二乘的讨论完全对应——只是这里的"残差"是沿时间轴展开的代价函数。

### 54.3.3 什么时候差异重要? ⭐⭐

```
差异影响评估:
┌─────────────────────────────────────────────────────┐
│ 动力学强非线性(如柔性体、接触碰撞)                    │
│   → DDP 的动力学 Hessian 项有意义                     │
│   → DDP 收敛更快(少 2-5 次迭代)                      │
│                                                       │
│ 动力学弱非线性(如刚体、匀速运动附近)                  │
│   → 动力学 Hessian 项接近零                           │
│   → DDP 和 iLQR 几乎一样                             │
│   → iLQR 更值得(省了计算 Hessian 的开销)             │
│                                                       │
│ Pinocchio 提供动力学解析导数(包括 Hessian)            │
│   → DDP 的额外代价可控                                │
│   → Crocoddyl 默认用 FDDP (基于 DDP,保留 Hessian)   │
└─────────────────────────────────────────────────────┘
```

> ⚠️ **陷阱**: 很多教程和博客混用"DDP"和"iLQR"这两个术语。严格来说它们是不同的算法。当有人说"我们用 DDP 做 MPC",你需要确认他们是否真的计算了动力学 Hessian。

**练习 54.3** (⭐⭐): 在 54.2 的 2D 小车例子上,把动力学改为 $\dot{v} = u - 0.5\sin(p)$(单摆模型)。分别用 DDP 和 iLQR 求解,记录每次迭代的代价下降。需要多少次迭代收敛?差异是否显著?

---

## 54.4 FDDP——Feasibility-Driven DDP ⭐⭐⭐

### 54.4.1 经典 DDP 的 Achilles' Heel ⭐⭐

经典 DDP 是 **single shooting** 方法:forward pass 必须从 $\mathbf{x}_0$ 开始完整地"rollout"动力学。

> **本质洞察**:经典 DDP 的脆弱性根源在于 single shooting 把"满足动力学约束"的职责完全压在了 forward pass 上——每一步的误差都会通过动力学传播到后续所有时间步,形成误差雪崩。FDDP 的突破在于引入 gap(动力学间隙)作为优化变量,将"轨迹可行性"从硬性前提降级为逐步收敛的软目标,从而打破了"必须先有好的初始猜测才能开始优化"的鸡生蛋困局。

**问题 1:初始轨迹必须可行**

如果初始控制序列 $\{\bar{\mathbf{u}}_k\}$ 导致状态发散(数值爆炸),forward pass 失败——DDP 无法启动。如果不引入 FDDP 的 gap 机制,用户就必须手工设计一条物理可行的初始轨迹——对于四足机器人跳跃、翻越障碍等复杂动作,这本身就是一个比优化更难的问题。

**问题 2:MPC warm-start 产生不可行猜测**

```
MPC warm-start 的问题:
时间步:  0  1  2  3  ...  N-1  N
旧解:   [u0 u1 u2 u3 ... u_{N-1}]
移位后: [u1 u2 u3 ... u_{N-1} ???]  ← 最后一步没有解!
                                  ↑
                            通常填零或重复
                            → 轨迹末端不可行
```

### 54.4.2 FDDP 的核心思想:接受"Gaps" ⭐⭐⭐

FDDP (Mastalli et al., Autonomous Robots 2022;论文中完整算法名为 **Box-FDDP**,包含 feasibility-driven 与 control-bounded 两种模式) 的关键创新:**允许轨迹中存在"动力学间隙"(gaps)**。如果说经典 DDP 的 forward pass 相当于一列火车——每节车厢(时间步)必须严格连接,那么 FDDP 相当于允许车厢之间有弹性连接:初始时车厢可以有间隙,随着优化迭代,间隙逐步收紧至零。这种"先宽松后收紧"的策略大幅提升了对初始猜测的容忍度。

**Gap 的定义**:

$$\bar{\mathbf{f}}_k = f_k(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k) - \bar{\mathbf{x}}_{k+1}$$

如果 $\bar{\mathbf{f}}_k = 0$,则第 $k$ 步满足动力学(可行)。如果 $\bar{\mathbf{f}}_k \neq 0$,则存在 gap。（此定义与 Mastalli et al., 2022 一致：gap = 动力学预测 − 名义下一状态。$\bar{\mathbf{f}}_k = 0$ 表示满足动力学（可行）。）

```
Single-shooting (经典 DDP):           Multiple-shooting (FDDP 允许):
x₀ ──f──→ x₁ ──f──→ x₂ ──f──→ x₃   x₀ ──f──→ x₁' x₁ ──f──→ x₂' x₂ ──f──→ x₃
          │          │          │               ↕gap      ↕gap
       必须连续    必须连续    必须连续      允许间隙    允许间隙
                                          逐步收敛到零
```

### 54.4.3 FDDP 的修改:Backward Pass with Gaps ⭐⭐⭐

在存在 gaps 的情况下,backward pass 的 Q 函数系数需要修改:

$$Q_\mathbf{x} = l_\mathbf{x} + \mathbf{f}_\mathbf{x}^T (V_{\mathbf{x},k+1} + V_{\mathbf{xx},k+1} \bar{\mathbf{f}}_k)$$

$$Q_\mathbf{u} = l_\mathbf{u} + \mathbf{f}_\mathbf{u}^T (V_{\mathbf{x},k+1} + V_{\mathbf{xx},k+1} \bar{\mathbf{f}}_k)$$

> 💡 **洞察**: Gap $\bar{\mathbf{f}}_k$ 出现在 $V_\mathbf{x}$ 的修正中——相当于把"消除 gap"也作为代价的一部分,让优化器自动去消除它。

### 54.4.4 FDDP Forward Pass:带 Gap 的 Rollout ⭐⭐⭐

$$\hat{\mathbf{x}}_{k+1} = f_k(\hat{\mathbf{x}}_k, \hat{\mathbf{u}}_k) + (1 - \alpha) \bar{\mathbf{f}}_k$$

当 $\alpha = 1$ 时,$\hat{\mathbf{x}}_{k+1} = f_k(\hat{\mathbf{x}}_k, \hat{\mathbf{u}}_k)$——gaps 完全消除。
当 $\alpha = 0$ 时,$\hat{\mathbf{x}}_{k+1} = f_k(\hat{\mathbf{x}}_k, \hat{\mathbf{u}}_k) + \bar{\mathbf{f}}_k$——保持旧的 gaps。

**在迭代过程中,FDDP 从保持 gaps ($\alpha$ 小) 逐步过渡到消除 gaps ($\alpha \to 1$)**。

### 54.4.5 FDDP 完整算法 ⭐⭐⭐

```
算法: FDDP (Feasibility-Driven DDP)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入: 初始轨迹 {x̄₀, ū₀, ..., x̄_N}(可以不满足动力学!)
输出: 收敛的可行轨迹

1. 计算初始 gaps: f̄_k = x̄_{k+1} - f(x̄_k, ū_k), ∀k

2. Repeat until convergence:
   a. [Backward Pass]
      - V_x,N = l_N,x(x̄_N), V_xx,N = l_N,xx(x̄_N)
      - For k = N-1 downto 0:
        Q_x = l_x + f_x^T · (V_x,k+1 + V_xx,k+1 · f̄_k)
        Q_u = l_u + f_u^T · (V_x,k+1 + V_xx,k+1 · f̄_k)
        Q_xx, Q_uu, Q_ux: 同标准 DDP
        k_k = -Q_uu⁻¹ · Q_u
        K_k = -Q_uu⁻¹ · Q_ux
        更新 V_x,k, V_xx,k

   b. [Forward Pass with Line Search]
      α = 1.0
      while α > α_min:
        x̂₀ = x̄₀
        For k = 0 to N-1:
          û_k = ū_k + α·k_k + K_k·(x̂_k - x̄_k)
          x̂_{k+1} = f(x̂_k, û_k) + (1-α)·f̄_k  ← gap 项!
        end
        if cost decreased (Armijo):
          接受, 更新 f̄_k
          break
        α *= 0.5

   c. [收敛判断]
      if ‖∇J‖ < ε AND ‖f̄‖ < ε_feas:
        converged, break
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 54.4.6 FDDP 为什么是 MPC 的标准选择 ⭐⭐

| 特性 | 经典 DDP | FDDP |
|------|---------|------|
| 初始轨迹要求 | 必须可行(能 rollout) | **可以不可行** |
| MPC warm-start | 需要小心处理末端 | **自然处理 gaps** |
| 收敛行为 | gap 始终为零 | **gap 单调减小** |
| 实际收敛速度 | 快(如果初始可行) | 更鲁棒,略慢 |

> ⚠️ **陷阱**: FDDP 的"gaps 单调减小"不是自动保证的——需要 Armijo line search 配合。如果 line search 参数设置不当,gaps 可能振荡。Crocoddyl 的默认参数经过仔细调优。

**练习 54.4a** (⭐⭐): 在 54.2 的 2D 小车例子上,给一个"不可行"的初始轨迹(比如所有状态都设为零,但控制设为随机值)。经典 DDP 是否能收敛?FDDP 呢?

**练习 54.4b** (⭐⭐⭐): 实现 FDDP 的 forward pass(带 gap 项),并绘制每次迭代中 $\|\bar{\mathbf{f}}\|$ 的变化,验证 gap 单调减小。

---

## 54.4B Control-Limited DDP(box-DDP)——把控制限位塞进 backward pass ⭐⭐⭐⭐

上一节的 FDDP 解决了"初始轨迹不可行"的问题,但还遗留一个 DDP 家族的老大难:**控制限位**(control limits)。Crocoddyl 默认的求解器叫 `SolverBoxFDDP`,这个 "Box" 前缀正是本节的主题。我们先讲清楚为什么控制限位这么棘手,再一步步推导 Tassa et al.(ICRA 2014)提出的 box-DDP,最后说明它如何与 FDDP 的 gap 机制合体成 Box-FDDP。

### 54.4B.1 动机:为什么控制限位是 DDP 的硬骨头? ⭐⭐⭐

回顾 足式/60_QP_NLP建模:在通用 NLP 里,控制限位 $\underline{\mathbf{u}} \leq \mathbf{u} \leq \overline{\mathbf{u}}$ 只是又一组不等式约束,内点法或 active-set 法统一处理即可。但 DDP 是一种**间接法**(indirect method)——它不直接在约束集上优化,而是通过 Bellman 递推在**无约束的控制空间**里解析地求出最优修正 $\delta\mathbf{u}^* = \mathbf{k} + \mathbf{K}\delta\mathbf{x}$。这个解析解是把 $\partial Q/\partial\delta\mathbf{u}=0$ 直接求逆得到的,它**根本不知道**控制有上下界。

> **本质洞察**:DDP 的高效率恰恰来自"只在无约束控制空间里求解"——它用 $-Q_{\mathbf{uu}}^{-1}Q_\mathbf{u}$ 一步算出最优控制,省掉了通用 NLP 反复探测约束激活集的代价。而控制限位破坏了这个前提:一旦最优解落在边界外,那个漂亮的闭式解就失效了。box-DDP 的全部技巧,就是在"保住闭式解的高效"和"尊重控制边界"之间找平衡。

那么,不专门处理控制限位会怎样?工程上有两种朴素做法,Tassa 的论文证明它们都不行:

**朴素做法一:Clamping(事后裁剪)。** 先用无约束 DDP 算出 $\mathbf{u}^* = \bar{\mathbf{u}} + \alpha\mathbf{k} + \mathbf{K}\delta\mathbf{x}$,然后在 forward pass 里把超界的分量裁剪回边界:$\mathbf{u} \leftarrow \mathrm{clamp}(\mathbf{u}, \underline{\mathbf{u}}, \overline{\mathbf{u}})$。

```
为什么 Clamping 不行?
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
问题在于:反馈增益 K 是在"无约束假设"下算出来的。
当某个控制分量被裁剪到边界,它就不再随 δx 变化了
(它的有效增益应该是 0),但 K 矩阵仍然让其它分量
按"它还在自由变化"的假设去补偿——这是错误的耦合。

后果:被裁剪分量与自由分量之间的反馈协调被破坏,
     line search 经常失败,收敛退化为线性甚至发散。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**朴素做法二:Penalty(惩罚函数)。** 在代价里加一项 $\rho\cdot\max(0, \mathbf{u}-\overline{\mathbf{u}})^2$ 软性惩罚越界。问题是:$\rho$ 小则约束形同虚设,$\rho$ 大则 $Q_{\mathbf{uu}}$ 病态(condition number 爆炸),Riccati 回退数值崩溃。这与 足式/80_接触力学与约束优化 中讨论的"软约束 vs 硬约束"权衡完全同构——惩罚法永远在"约束不够硬"和"数值不稳定"之间走钢丝。

> ⚠️ **概念误区**:很多人以为"控制限位嘛,clamp 一下不就行了"。Tassa et al.(2014)用实验证明:naive clamping 会让 DDP 在带限位的人形机器人问题上**丧失二次收敛**,迭代次数翻几倍甚至发散。控制限位不是 forward pass 的事后修补,而必须在 **backward pass 求控制律时**就纳入考虑。

### 54.4B.2 核心思想:把每一步的 $\delta\mathbf{u}$ 求解变成一个 box-QP ⭐⭐⭐⭐

box-DDP 的洞察是:backward pass 里求 $\delta\mathbf{u}^*$ 本质上是在最小化一个**二次型**

$$\delta\mathbf{u}^* = \arg\min_{\delta\mathbf{u}} \; \frac{1}{2}\delta\mathbf{u}^T Q_{\mathbf{uu}}\delta\mathbf{u} + (Q_\mathbf{u} + Q_{\mathbf{ux}}\delta\mathbf{x})^T\delta\mathbf{u}$$

无约束时,令梯度为零就得到 $\delta\mathbf{u}^* = -Q_{\mathbf{uu}}^{-1}(Q_\mathbf{u} + Q_{\mathbf{ux}}\delta\mathbf{x})$。现在我们**直接给这个二次最小化加上箱型约束**:

$$\boxed{\delta\mathbf{u}^* = \arg\min_{\delta\mathbf{u}} \; \frac{1}{2}\delta\mathbf{u}^T Q_{\mathbf{uu}}\delta\mathbf{u} + Q_\mathbf{u}^T\delta\mathbf{u} \quad \text{s.t.} \quad \underline{\mathbf{u}} - \bar{\mathbf{u}} \leq \delta\mathbf{u} \leq \overline{\mathbf{u}} - \bar{\mathbf{u}}}$$

这是一个**带箱型约束的二次规划(box-constrained QP,简称 box-QP)**。注意约束边界是 $\underline{\mathbf{u}}-\bar{\mathbf{u}}$ 到 $\overline{\mathbf{u}}-\bar{\mathbf{u}}$,因为 $\delta\mathbf{u}$ 是相对当前 $\bar{\mathbf{u}}$ 的扰动。

> 💡 **洞察**:这一步是"把约束从 forward pass 搬到 backward pass"的关键。无约束 DDP 在每个时间步解一个**线性方程组**($Q_{\mathbf{uu}}\delta\mathbf{u} = -Q_\mathbf{u}$);box-DDP 在每个时间步解一个**小规模 box-QP**($n_u$ 维,$n_u$ 通常只有 12)。$n_u$ 维的 box-QP 用 projected-Newton 几次迭代就收敛,代价远小于一次 Pinocchio 动力学求导。

### 54.4B.3 Projected-Newton 求解 box-QP ⭐⭐⭐⭐

Tassa 用 **projected-Newton 法**(投影牛顿法)解这个 box-QP。核心是维护一个**激活集(active set)**——记录哪些控制分量当前顶在边界上。

把控制分量分成两类:
- **自由集(free set)** $\mathcal{F}$:$\delta u_i$ 严格在边界内,可自由优化。
- **钳制集(clamped set)** $\mathcal{C}$:$\delta u_i$ 顶在上界或下界,暂时固定。

对自由集里的分量,我们解一个**降维的无约束 Newton 子问题**——只用 $Q_{\mathbf{uu}}$ 中对应自由分量的子块:

$$\delta\mathbf{u}_{\mathcal{F}}^* = -\left(Q_{\mathbf{uu}}^{\mathcal{F}\mathcal{F}}\right)^{-1}\left(Q_{\mathbf{u}}^{\mathcal{F}} + Q_{\mathbf{uu}}^{\mathcal{F}\mathcal{C}}\delta\mathbf{u}_{\mathcal{C}}\right)$$

其中 $Q_{\mathbf{uu}}^{\mathcal{F}\mathcal{F}}$ 是 $Q_{\mathbf{uu}}$ 取自由集行列的子矩阵,$\delta\mathbf{u}_{\mathcal{C}}$ 是钳制分量(固定在边界)。算完后投影回箱内、更新激活集,重复直到激活集稳定。

```
Projected-Newton 求解 box-QP (单个时间步):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入: Q_uu (n_u×n_u), Q_u (n_u), 边界 [lb, ub]
1. 初始化 δu = clamp(无约束解, lb, ub)
2. Repeat:
   a. 判定激活集:
      clamped = { i : (δu_i==lb_i 且 grad_i>0) 或
                      (δu_i==ub_i 且 grad_i<0) }
      free    = 其余分量
   b. 在 free 子空间解 Newton 步:
      δu_free = -(Q_uu[free,free])⁻¹ · grad[free]
   c. 沿 Newton 方向做投影 line search:
      δu = clamp(δu + t·Δ, lb, ub)
   d. if 激活集不再变化 且 ‖free 梯度‖<ε: break
3. 返回 δu* 及自由集上的反馈增益 K_free
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

> 🧠 **深入理解**:为什么是"projected-**Newton**"而不是"projected-**gradient**"?因为在自由子空间里我们用了完整的二阶信息 $(Q_{\mathbf{uu}}^{\mathcal{F}\mathcal{F}})^{-1}$,这保住了 DDP 的二次收敛特性——Tassa 论文的标题结论正是"box-DDP exhibits quadratic convergence"。如果只用投影梯度,会退化成一阶方法,收敛慢得多。这与 足式/60_QP_NLP建模 中"梯度法 vs 牛顿法"的对比一脉相承,只是这里的牛顿步被限制在激活集补集上。

### 54.4B.4 钳制分量的反馈增益必须置零 ⭐⭐⭐

box-DDP 还有一个极易被忽略的细节,正是 naive clamping 翻车的根因:**钳制集分量的反馈增益行必须置零**。

直觉是这样的:如果某个控制分量 $u_i$ 已经顶在上界(比如电机已经满扭矩),那么当状态发生扰动 $\delta\mathbf{x}$ 时,$u_i$ **不能再增加**去响应——它已经饱和了。因此它对 $\delta\mathbf{x}$ 的反馈增益就应该是 0:

$$\mathbf{K}_{i,:} = \mathbf{0}, \quad \forall i \in \mathcal{C}\ (\text{钳制集})$$

只有自由集分量才保留非零反馈增益,且这些增益由 $-(Q_{\mathbf{uu}}^{\mathcal{F}\mathcal{F}})^{-1}Q_{\mathbf{ux}}^{\mathcal{F}}$ 给出。

> **本质洞察**:naive clamping 的致命错误,就是用了**完整**的 $\mathbf{K}=-Q_{\mathbf{uu}}^{-1}Q_{\mathbf{ux}}$ 但又在 forward pass 裁剪控制——增益矩阵假设所有分量都自由,而现实里一部分分量已饱和。box-DDP 通过"钳制分量增益置零"让反馈律与饱和事实自洽,这才是它能保持收敛的根本原因。一句话:**饱和的执行器不该再有反馈权限**。

### 54.4B.5 Box-FDDP:gap 机制 + 控制限位合体 ⭐⭐⭐

现在把两条线索接起来。Crocoddyl 的旗舰求解器 `SolverBoxFDDP` 同时具备:
- **FDDP 的 gap 机制**(§54.4):容忍不可行初值,gap 单调收敛;
- **box-DDP 的 projected-Newton**(本节):每步解 box-QP,尊重控制边界。

Mastalli et al.(2022)的论文把它表述为**两种模式的自动切换**:

| 模式 | 触发条件 | 行为 |
|------|---------|------|
| **feasibility-driven 模式** | 当前轨迹不可行($\|\bar{\mathbf{f}}\|$ 大) | 优先收紧 gap,$\alpha$ 主要用于消除动力学间隙 |
| **control-bounded 模式** | 轨迹接近可行 | 启用 box-QP,精细处理控制限位,逼近最优 |

```
Box-FDDP 双模式自动切换:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                  ‖gap‖ 大?
                 ╱        ╲
              是╱          ╲否
   feasibility-driven    control-bounded
   (先把轨迹拉回可行)    (box-QP 精修控制)
        │                    │
        └────── 迭代推进 ─────┘
              gap → 0, 控制满足边界
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

> ⚠️ **思维陷阱**:不要把 box-DDP 和 ProxDDP(§54.9.2)搞混。box-DDP 只处理**控制限位**(箱型约束,可在每步局部 QP 解决);ProxDDP 用增广拉格朗日处理**任意约束**(含状态约束、摩擦锥这类把不同时间步耦合起来的约束)。控制限位之所以能用 box-DDP 廉价解决,正因为它是**逐时间步解耦**的——每个 $\mathbf{u}_k$ 的边界只约束自己,不牵连别的时间步。一旦约束跨时间步耦合(如"整条轨迹的足端不能踩进禁区"),box-QP 就无能为力,必须上 ProxDDP/SQP。

**练习 54.4Ba** (⭐⭐⭐): 在 54.2 的 2D 小车例子上加控制限位 $|u|\leq 2$。先用 naive clamping(无约束 DDP + forward pass 裁剪),再用 projected-Newton box-QP。对比两者的收敛迭代数和最终代价。

**练习 54.4Bb** (⭐⭐⭐⭐): 实现一个 $n_u=3$ 的 box-QP projected-Newton 求解器(Eigen),输入随机正定 $Q_{\mathbf{uu}}$、随机 $Q_\mathbf{u}$ 和边界,验证:(a) 解满足 KKT 条件;(b) 钳制分量的梯度方向都指向边界外。与 `quadprog` 或 OSQP 的结果对照。

---

## 54.5 Crocoddyl 的 ActionModel/ActionData 架构 ⭐⭐

### 54.5.1 设计动机:为什么分 Model 和 Data? ⭐⭐

回忆 足式/30_Pinocchio深度精读 的 Pinocchio:

```
Pinocchio 的 Model-Data 分离:
┌────────────────────┐    ┌────────────────────┐
│     Model          │    │     Data           │
│ (机器人参数,不变)   │    │ (计算中间结果)       │
│ - 连杆质量/惯量     │    │ - 关节位置/速度      │
│ - 运动链拓扑       │    │ - Jacobian 缓冲      │
│ - 关节限位         │    │ - 动力学计算缓冲      │
│ 线程安全:只读       │    │ 线程安全:每线程一份   │
└────────────────────┘    └────────────────────┘
```

Crocoddyl 把这个模式**扩展到时间轴**:

```
Crocoddyl 的 ActionModel-ActionData 分离:
┌─────────────────────────────────────────────────┐
│ ShootingProblem                                   │
│                                                   │
│ 时间步 0    时间步 1    时间步 2    ...  时间步 N  │
│ ┌───────┐  ┌───────┐  ┌───────┐       ┌───────┐ │
│ │Model₀ │  │Model₁ │  │Model₂ │       │ModelN │ │
│ │(代价+  │  │(代价+  │  │(代价+  │       │(终端  │ │
│ │ 动力学)│  │ 动力学)│  │ 动力学)│       │ 代价) │ │
│ └───┬───┘  └───┬───┘  └───┬───┘       └───┬───┘ │
│     │          │          │                │     │
│ ┌───▼───┐  ┌───▼───┐  ┌───▼───┐       ┌───▼───┐ │
│ │Data₀  │  │Data₁  │  │Data₂  │       │DataN  │ │
│ │(xnext,│  │(xnext,│  │(xnext,│       │(cost, │ │
│ │ Fx,Fu,│  │ Fx,Fu,│  │ Fx,Fu,│       │ Lx,..)│ │
│ │ cost) │  │ cost) │  │ cost) │       │       │ │
│ └───────┘  └───────┘  └───────┘       └───────┘ │
└─────────────────────────────────────────────────┘
```

### 54.5.2 ActionModelAbstract 接口 ⭐⭐

```cpp
// Crocoddyl core interface (simplified from action-base.hpp)
template <typename _Scalar>
class ActionModelAbstractTpl {
 public:
  typedef _Scalar Scalar;
  typedef ActionDataAbstractTpl<Scalar> ActionDataAbstract;
  typedef MathBaseTpl<Scalar> MathBase;

  // Core computation: cost + dynamics
  virtual void calc(
      const boost::shared_ptr<ActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x,
      const Eigen::Ref<const VectorXs>& u) = 0;

  // Derivatives: Jacobians + Hessians (for DDP backward pass)
  virtual void calcDiff(
      const boost::shared_ptr<ActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x,
      const Eigen::Ref<const VectorXs>& u) = 0;

  // Factory: create matching Data object
  virtual boost::shared_ptr<ActionDataAbstract> createData() = 0;

  std::size_t get_nx() const;  // state dimension
  std::size_t get_nu() const;  // control dimension
};
```

### 54.5.3 ActionDataAbstract 的内存布局 ⭐⭐

```cpp
// ActionData: pre-allocated buffers for one time step
template <typename _Scalar>
struct ActionDataAbstractTpl {
  // Dynamics output
  VectorXs xnext;   // f(x, u): next state, size nx
  
  // Dynamics Jacobians
  MatrixXs Fx;      // df/dx, size nx x nx
  MatrixXs Fu;      // df/du, size nx x nu
  
  // Cost value + derivatives
  Scalar cost;       // l(x, u)
  VectorXs Lx;      // dl/dx, size nx
  VectorXs Lu;      // dl/du, size nu
  MatrixXs Lxx;     // d2l/dx2, size nx x nx
  MatrixXs Luu;     // d2l/du2, size nu x nu
  MatrixXs Lxu;     // d2l/dxdu, size nx x nu
  
  // Constraint data (Crocoddyl 2.0+)
  VectorXs g;       // constraint residual g(x,u)
  MatrixXs Gx;      // dg/dx
  MatrixXs Gu;      // dg/du
};
```

> 💡 **洞察**: 所有矩阵在 `createData()` 时**一次性分配**,之后 `calc()` / `calcDiff()` 只是**填充已有内存**,不做任何堆分配。这对实时性至关重要——02_C++基础与进阶/40_内存管理 中深入讨论了 pmr(Polymorphic Memory Resource)分配器的原理:通过在启动时预分配一块连续内存池,运行时的所有"分配"都变成指针偏移操作(O(1),无系统调用),从而消除 `malloc` 在实时线程中引发的不确定延迟。Crocoddyl 的 `createData()` 预分配策略与 pmr 的思路一致,只是实现层面更简单——直接在构造函数中 resize 所有 Eigen 矩阵。

### 54.5.4 线程安全性分析 ⭐⭐⭐

**Model 是只读的**: 多个线程可以共享同一个 Model 对象(例如 trot 步态中多个时间步使用相同的接触配置)。

**Data 是可写的**: 每个线程**必须有自己的 Data**。这就是为什么 `ShootingProblem` 持有 N 个独立的 Data 对象。

```
OpenMP 并行安全模型:
┌──────────────────────────────────────────┐
│ Thread 0          Thread 1          ...  │
│ ┌─────────┐      ┌─────────┐            │
│ │ Data[0] │      │ Data[1] │            │
│ │ Data[4] │      │ Data[5] │            │
│ │ Data[8] │      │ Data[9] │            │
│ └────┬────┘      └────┬────┘            │
│      │                │                 │
│      ▼                ▼                 │
│ Model[0]──────→ Model[1]  (只读共享)    │
└──────────────────────────────────────────┘
```

> ⚠️ **陷阱**: 如果两个时间步共享同一个 ActionModel **和同一个 ActionData**(例如误用指针),OpenMP 并行化会产生 data race。Crocoddyl 通过为每个时间步创建独立的 Data 来避免这个问题。

### 54.5.5 如何实现自定义 ActionModel(分步教程) ⭐⭐

假设你想实现一个简单的"弹簧-质量-阻尼"系统:

```cpp
#include <crocoddyl/core/action-base.hpp>

// Step 1: Define your ActionModel
class ActionModelSpringMass
    : public crocoddyl::ActionModelAbstractTpl<double> {
 public:
  ActionModelSpringMass(double k_spring, double b_damp, double dt)
      : ActionModelAbstractTpl(
            boost::make_shared<crocoddyl::StateVector>(2),  // nx=2: [pos, vel]
            1),  // nu=1: force
        k_(k_spring), b_(b_damp), dt_(dt) {}

  void calc(const boost::shared_ptr<ActionDataAbstract>& data,
            const Eigen::Ref<const Eigen::VectorXd>& x,
            const Eigen::Ref<const Eigen::VectorXd>& u) override {
    // Dynamics: x_next = [p + v*dt, v + (u - k*p - b*v)*dt]
    data->xnext(0) = x(0) + x(1) * dt_;
    data->xnext(1) = x(1) + (u(0) - k_ * x(0) - b_ * x(1)) * dt_;
    // Cost: 0.5 * u^2 * R
    data->cost = 0.5 * u(0) * u(0) * 0.01;
  }

  void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                const Eigen::Ref<const Eigen::VectorXd>& x,
                const Eigen::Ref<const Eigen::VectorXd>& u) override {
    // Dynamics Jacobians
    data->Fx << 1.0, dt_, -k_ * dt_, 1.0 - b_ * dt_;
    data->Fu << 0.0, dt_;
    // Cost derivatives
    data->Lx.setZero();
    data->Lu(0) = 0.01 * u(0);
    data->Lxx.setZero();
    data->Luu(0, 0) = 0.01;
    data->Lxu.setZero();
  }

 private:
  double k_, b_, dt_;
};
```

**练习 54.5a** (⭐⭐): 用上面的 `ActionModelSpringMass` 搭建一个 `ShootingProblem`,用 `SolverFDDP` 求解。目标:让弹簧系统从 $[1, 0]$ 到达 $[0, 0]$。

**练习 54.5b** (⭐⭐⭐): 给自定义 ActionModel 添加约束:$|u| \leq 5$(控制限位)。使用 Crocoddyl 2.0 的 `ConstraintManager` 接口。

---

## 54.6 常见 ActionModel 详解 ⭐⭐

### 54.6.1 Differential vs Integrated:两层架构 ⭐⭐

Crocoddyl 把一个时间步的计算分为两层:

```
┌──────────────────────────────────────────┐
│        IntegratedActionModel             │
│  (离散时间:给 DDP 用的接口)              │
│                                          │
│  ┌────────────────────────────────────┐  │
│  │    DifferentialActionModel         │  │
│  │  (连续时间:返回 ẍ, 不做积分)       │  │
│  │                                    │  │
│  │  calc() → 计算 xdot, cost         │  │
│  │  calcDiff() → 计算 Fx_cont, Fu... │  │
│  └────────────────────────────────────┘  │
│                                          │
│  IntegratedActionModel.calc():           │
│    xdot = DiffModel.calc(x, u)          │
│    x_next = x + xdot * dt  (Euler)      │
│    或 RK4(x, xdot, dt)                  │
└──────────────────────────────────────────┘
```

**好处**:同一个 DifferentialActionModel 可以被不同的积分器包裹——Euler、RK4、隐式 Euler 等。

### 54.6.2 DifferentialActionModelFreeFwdDynamics ⭐⭐

用于**无接触**的机械臂:

$$\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{h}(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{S}^T \boldsymbol{\tau}$$

其中 $\mathbf{S}$ 是选择矩阵(浮动基座无驱动的行为零)。

`calc()` 内部调用 Pinocchio 的 ABA (Articulated Body Algorithm):

$$\ddot{\mathbf{q}} = \text{ABA}(\mathbf{q}, \dot{\mathbf{q}}, \boldsymbol{\tau})$$

`calcDiff()` 内部调用 Pinocchio 的解析导数:

$$\frac{\partial \ddot{\mathbf{q}}}{\partial \mathbf{q}}, \quad \frac{\partial \ddot{\mathbf{q}}}{\partial \dot{\mathbf{q}}}, \quad \frac{\partial \ddot{\mathbf{q}}}{\partial \boldsymbol{\tau}}$$

### 54.6.3 DifferentialActionModelContactFwdDynamics(腿足核心) ⭐⭐⭐

**这是四足机器人最常用的 ActionModel**。它求解含接触约束的 KKT 系统:

$$\begin{bmatrix} \mathbf{M} & -\mathbf{J}_c^T \\ \mathbf{J}_c & \mathbf{0} \end{bmatrix} \begin{bmatrix} \ddot{\mathbf{q}} \\ \boldsymbol{\lambda} \end{bmatrix} = \begin{bmatrix} \mathbf{S}^T \boldsymbol{\tau} - \mathbf{h} \\ -\dot{\mathbf{J}}_c \dot{\mathbf{q}} \end{bmatrix}$$

**KKT 系统解读**:

- 第一行:牛顿方程 + 接触力 $\mathbf{J}_c^T \boldsymbol{\lambda}$
- 第二行:接触约束(接触点加速度为零,即不滑)
- $\mathbf{J}_c$:接触 Jacobian(各接触点的速度 Jacobian 堆叠)
- $\boldsymbol{\lambda}$:接触力(拉格朗日乘子）

**KKT 解法**(Pinocchio `forwardDynamics`):

1. 计算 $\mathbf{M}^{-1}$(利用 Cholesky 分解）
2. $\boldsymbol{\lambda} = (\mathbf{J}_c \mathbf{M}^{-1} \mathbf{J}_c^T)^{-1} (\mathbf{J}_c \mathbf{M}^{-1}(\mathbf{S}^T\boldsymbol{\tau} - \mathbf{h}) + \dot{\mathbf{J}}_c \dot{\mathbf{q}})$
3. $\ddot{\mathbf{q}} = \mathbf{M}^{-1} (\mathbf{S}^T\boldsymbol{\tau} - \mathbf{h} + \mathbf{J}_c^T \boldsymbol{\lambda})$

> 🧠 **深入理解**: KKT 系统的 Jacobian(即 `calcDiff()` 的输出)比无接触情况复杂得多——需要对 $\mathbf{M}, \mathbf{J}_c, \mathbf{h}$ 同时求导。Pinocchio 的 `computeConstraintDynamicsDerivatives()` 用高效的解析方法完成这个计算,复杂度 $O(n_{\text{dof}}^2 \cdot n_c)$。回顾 足式/80_接触力学与约束优化:接触 Jacobian $J_c$ 把关节速度映射到接触点速度,其转置 $J_c^T$ 把接触力映射回关节空间广义力,两者构成虚功原理要求的对偶映射。这里的 KKT 系统正是将这一对偶关系与动力学方程 $M\ddot{q} + h = S^T\tau + J_c^T\lambda$ 联立求解,使得接触约束 $J_c\ddot{q} = -\dot{J}_c\dot{q}$ 被隐式满足。

### 54.6.4 CostModelResidual:残差式代价 ⭐⭐

Crocoddyl v1.8.0（2021 年 7 月）引入了 **ResidualModel** 抽象:

```
代价 = 激活函数 ∘ 残差

CostModelResidual(state, activation, residual)
  │
  ├── activation: ActivationModelQuad  → ½‖r‖²
  │                ActivationModelWeightedQuad → ½r^T W r
  │                ActivationModelSmooth1Norm → smooth L1
  │
  └── residual: ResidualModelState → r = x - x_ref
                ResidualModelControl → r = u - u_ref
                ResidualModelFrameTranslation → r = p(q) - p_ref
                ResidualModelCoMPosition → r = CoM(q) - CoM_ref
                ResidualModelContactForce → r = λ - λ_ref
```

> 💡 **洞察**: 这种"激活函数 + 残差"的分离让代价函数的构建非常灵活。例如,Huber 损失可以通过 `ActivationModelSmooth1Norm` + 任何残差来实现,无需修改残差代码。

### 54.6.5 IntegratedActionModel:Euler vs RK4 ⭐⭐

| 积分方法 | 精度 | 计算代价 | 适用场景 |
|---------|------|---------|---------|
| Euler | $O(dt)$ | 1x `calc()` | MPC (dt 小) |
| RK4 | $O(dt^4)$ | 4x `calc()` | 离线规划 (dt 大) |

**MPC 中通常用 Euler**:$dt = 10\text{ms}$ 时 Euler 误差足够小,且快 4 倍。

> ⚠️ **陷阱**: RK4 的 `calcDiff()` 需要对 4 个中间点求导数,比 Euler 贵得多。在 MPC 中用 RK4 往往不划算——不如把省下的时间用来多跑几次 DDP 迭代。

**练习 54.6** (⭐⭐): 用 Crocoddyl Python 接口搭建一个 Panda 机械臂的轨迹优化:使用 `DifferentialActionModelFreeFwdDynamics` + `IntegratedActionModelEuler`。代价:末端位姿误差 + 控制正则。N=100 步,FDDP 求解。

---

## 54.7 虚函数 vs CRTP:Crocoddyl 的设计决策 ⭐⭐⭐

### 54.7.1 回顾 CRTP 教义（02_C++基础与进阶/10_Eigen） ⭐⭐

02_C++基础与进阶/10_Eigen 中讲了 CRTP 的核心思想:**编译时多态,避免虚函数表(vtable)的间接调用开销**。

Pinocchio、Sophus、Eigen 都用 CRTP:

```cpp
// Pinocchio CRTP pattern (from 足式/30_Pinocchio深度精读)
template <typename Derived>
struct ModelTpl : CRTPBase<Derived> {
  auto algorithm() { return derived().algorithm_impl(); }
};
```

**CRTP 的好处**:编译器可以内联虚函数调用,节省 5-10ns 的 vtable 查找。

### 54.7.2 Crocoddyl 的反例:虚函数够用 ⭐⭐

Crocoddyl 的 `ActionModelAbstract` 是**经典的虚基类**:

```cpp
class ActionModelAbstract {
 public:
  virtual void calc(...) = 0;      // <-- virtual function
  virtual void calcDiff(...) = 0;  // <-- virtual function
};
```

为什么不用 CRTP?

### 54.7.3 性能分析:瓶颈不在调用开销 ⭐⭐⭐

```
性能瓶颈分析:
┌──────────────────────────────────────────────────┐
│ 虚函数调用开销: ~5-10 ns (vtable lookup + jump)  │
│                                                    │
│ ActionModel::calcDiff() 内部:                      │
│   Pinocchio RNEA + 解析导数: ~10-20 μs            │
│   KKT 系统求解 (含接触):    ~5-10 μs              │
│   代价函数求值+求导:        ~1-2 μs               │
│                                                    │
│ 虚函数开销占比: 10ns / 20μs = 0.05%              │
│                                                    │
│ 结论: 优化虚函数调度等于"优化 0.05%"→ 无意义     │
└──────────────────────────────────────────────────┘
```

### 54.7.4 灵活性和 Python 绑定 ⭐⭐

**灵活性**: Crocoddyl 用户经常在运行时根据配置选择不同的 ActionModel(自由飞行 vs 接触动力学 vs 自定义)。虚函数天然支持运行时多态,CRTP 需要类型擦除(复杂且脆弱)。

**Python 绑定**: Crocoddyl 使用 Boost.Python(后期版本用 eigenpy)暴露给 Python。虚函数可以直接被 Python 继承重写:

```python
# Python side: inherit ActionModel
class MyPythonModel(crocoddyl.ActionModelAbstract):
    def calc(self, data, x, u):
        # Python implementation
        data.cost = 0.5 * np.dot(u, u)
        data.xnext = my_dynamics(x, u)
    
    def calcDiff(self, data, x, u):
        # Python implementation
        data.Lu = u
        data.Fx = my_jacobian_x(x, u)
        data.Fu = my_jacobian_u(x, u)
```

CRTP 无法做到这一点——Python 不能"模板化继承" C++ 类。

### 54.7.5 设计决策总结 ⭐⭐

```
设计决策矩阵:
┌────────────────┬──────────────────┬──────────────────┐
│    维度         │ 选 CRTP          │ 选 Virtual       │
├────────────────┼──────────────────┼──────────────────┤
│ 调用频率       │ 超高频(矩阵运算) │ 低频(每步一次)  │
│ 计算量/次调用  │ 极小(几ns)       │ 大(几十μs)      │
│ 类型确定时机   │ 编译时            │ 运行时           │
│ Python 绑定    │ 困难              │ 简单             │
│ 代表库         │ Eigen, Pinocchio │ Crocoddyl        │
└────────────────┴──────────────────┴──────────────────┘
```

> 🧠 **深入理解**: 这个案例说明**性能优化要看瓶颈**。如果你在写高频矩阵库(Eigen),每次调用只有几纳秒,虚函数的 5ns 开销是 100% 的性能损失——必须 CRTP。如果你在写应用层框架(Crocoddyl),每次调用要做 20 微秒的 Pinocchio 运算,虚函数的 5ns 是噪音中的噪音——用虚函数省事。

**练习 54.7** (⭐⭐⭐): 写一个 benchmark:分别用虚函数和 CRTP 实现一个简单的 `calc()` 函数(内部做一次矩阵乘法)。在 $36 \times 36$ 矩阵上测量两种方式的耗时差异。结论是什么?

---

## 54.8 Crocoddyl 的 OpenMP 并行化 ⭐⭐

### 54.8.1 可并行的部分与不可并行的部分 ⭐⭐⭐

```
DDP 一次迭代的并行结构:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[Forward Pass]  串行 ← x_{k+1} = f(x_k, u_k) 依赖前一步
    └→ 但 calc() 可以跨时间步并行(不同轨迹猜测)

[calc + calcDiff]  可并行!
    └→ 每个时间步独立:Data[k] 只依赖 (x_k, u_k)
    └→ OpenMP: #pragma omp parallel for

[Backward Pass]  不可并行 (经典方法)
    └→ V_k 依赖 V_{k+1},严格顺序
    └→ 除非用 ParallelRiccati (→ 54.9)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 54.8.2 Crocoddyl 的 OpenMP 实现 ⭐⭐⭐

```cpp
// From crocoddyl/core/optctrl/shooting.cpp (simplified)
void ShootingProblem::calc(const std::vector<VectorXd>& xs,
                           const std::vector<VectorXd>& us) {
  #pragma omp parallel for schedule(static)
  for (std::size_t t = 0; t < T_; ++t) {
    running_models_[t]->calc(running_datas_[t], xs[t], us[t]);
  }
  terminal_model_->calc(terminal_data_, xs.back());
}

void ShootingProblem::calcDiff(const std::vector<VectorXd>& xs,
                               const std::vector<VectorXd>& us) {
  #pragma omp parallel for schedule(static)
  for (std::size_t t = 0; t < T_; ++t) {
    running_models_[t]->calcDiff(running_datas_[t], xs[t], us[t]);
  }
  terminal_model_->calcDiff(terminal_data_, xs.back());
}
```

### 54.8.3 加速比分析 ⭐⭐⭐

```
理论加速比 vs 实际加速比 (N=50, ANYmal 全身动力学):
┌────────────────────────────────────────────┐
│ 核数    理论加速    实际加速    效率         │
│  1       1.0x       1.0x      100%         │
│  2       2.0x       1.8x       90%         │
│  4       4.0x       3.2x       80%         │
│  8       8.0x       5.5x       69%         │
│ 16      16.0x       7.0x       44%         │
└────────────────────────────────────────────┘

瓶颈分析:
- OpenMP 线程调度开销: ~5 μs/次
- 内存带宽争抢: 多线程同时访问 Data
- 串行部分(backward pass): Amdahl 定律限制
- False sharing: 相邻 Data 对象可能在同一缓存行
```

> ⚠️ **陷阱**: False sharing 是 OpenMP 的经典陷阱。如果 `Data[0]` 和 `Data[1]` 的某些成员恰好在同一个 64 字节缓存行,一个线程写 `Data[0]` 会导致另一个线程的缓存行失效,造成不必要的缓存同步。Crocoddyl 通过让每个 Data 对象足够大(几 KB)来自然避免这个问题。

> 💡 **洞察**: 实际中 4 核是 Crocoddyl OpenMP 的"甜蜜点"——80% 效率,3.2 倍加速。超过 8 核后收益递减严重,因为 backward pass 是串行瓶颈(Amdahl 定律)。这正是 ParallelRiccati 要解决的问题。

**练习 54.8** (⭐⭐): 编译 Crocoddyl 并用 `OMP_NUM_THREADS=1,2,4,8` 运行 `examples/cpp/quadrupedal_walking.cpp`。记录不同核数下的求解时间,绘制加速比曲线。观察到什么?

---

## 54.9 Aligator / ProxDDP / ParallelRiccati ⭐⭐⭐⭐

### 54.9.1 Aligator:Crocoddyl 的"下一代" ⭐⭐⭐

**Aligator** (LAAS-CNRS / Inria, Jallet, Carpentier 等) 是 Pinocchio/Crocoddyl 团队的新一代轨迹优化库,它综合了两篇论文的成果:ProxDDP 的约束处理(Jallet et al., *PROXDDP: Proximal Constrained Trajectory Optimization*, T-RO 41:2605-2624, 2025)与 ParallelRiccati 的并行 backward pass(Jallet et al., *Parallel and Proximal Constrained Linear-Quadratic Methods for Real-Time Nonlinear MPC*, RSS 2024, Delft):

```
Crocoddyl 与 Aligator 的定位:
┌───────────────────────────────────────────────────┐
│ Crocoddyl (2019-)          Aligator (2024-)       │
│ ├─ DDP / FDDP              ├─ ProxDDP (核心)     │
│ ├─ 虚函数多态               ├─ C++17/20 Concepts │
│ ├─ Boost.Python             ├─ eigenpy 绑定       │
│ ├─ 无约束处理(除了嵌入动力学)├─ 增广拉格朗日约束   │
│ ├─ OpenMP (calc/calcDiff)   ├─ ParallelRiccati    │
│ └─ 成熟稳定                 └─ 前沿研究级         │
└───────────────────────────────────────────────────┘

依赖关系:
Eigen ──→ Pinocchio ──→ Aligator
                    └──→ Crocoddyl
fmtlib (>=10) ──→ Aligator
mimalloc (>=2.1) ──→ Aligator (高效内存分配)
```

### 54.9.2 ProxDDP:增广拉格朗日约束处理 ⭐⭐⭐⭐

**问题**:经典 DDP/FDDP 处理约束(如关节限位、摩擦锥)很困难。通常的做法是:

1. 把约束写成惩罚项加到代价里(不精确)
2. 用投影方法(如 Box-FDDP 对控制限位做投影)
3. 放弃 DDP,改用 SQP(OCS2 的选择)

ProxDDP 提供第四条路:**增广拉格朗日方法(ALM)嵌入 DDP**。

**约束 OCP 形式**:

$$\min \sum_{k=0}^{N-1} l_k(\mathbf{x}_k, \mathbf{u}_k) + l_N(\mathbf{x}_N)$$
$$\text{s.t.} \quad \mathbf{x}_{k+1} = f_k(\mathbf{x}_k, \mathbf{u}_k) \quad \text{(动力学)}$$
$$\quad\quad\quad g_k(\mathbf{x}_k, \mathbf{u}_k) = 0 \quad \text{(等式约束)}$$
$$\quad\quad\quad h_k(\mathbf{x}_k, \mathbf{u}_k) \leq 0 \quad \text{(不等式约束)}$$

**增广拉格朗日函数**:

$$\mathcal{L}_{\mu}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}) = l(\mathbf{x}, \mathbf{u}) + \boldsymbol{\lambda}^T g(\mathbf{x}, \mathbf{u}) + \frac{\mu}{2} \|g(\mathbf{x}, \mathbf{u})\|^2$$

其中 $\boldsymbol{\lambda}$ 是对偶变量(Lagrange 乘子),$\mu$ 是增广参数(惩罚系数)。

### 54.9.3 ProxDDP 的双层循环结构 ⭐⭐⭐⭐

```
算法: ProxDDP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
外层循环 (更新对偶变量):
  For j = 1, 2, ...:
    ┌──────────────────────────────────────┐
    │ 内层循环 (求解增广拉格朗日子问题):   │
    │   用 DDP 求解:                       │
    │   min Σ l_k(x,u) + λ^T·g(x,u)       │
    │       + (μ/2)‖g(x,u)‖²              │
    │                                       │
    │   这个子问题是**无约束的**!           │
    │   → 标准 DDP backward/forward pass   │
    │   → 直到内层收敛                     │
    └──────────────────────────────────────┘
    
    更新对偶变量:
      λ ← λ + μ · g(x*, u*)    (梯度上升)
    
    更新增广参数:
      if ‖g(x*,u*)‖ 减小不够:
        μ ← μ · 10             (增大惩罚)
    
    if ‖g(x*,u*)‖ < ε:
      break (外层收敛)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

> 💡 **洞察**: ProxDDP 的精妙之处在于:**内层循环用的是标准 DDP,不需要任何修改**。约束信息被"编码"进了增广代价函数。这意味着 ProxDDP 可以复用所有的 DDP 基础设施(backward pass、forward pass、line search)。

### 54.9.4 ProxDDP vs FDDP:何时用哪个? ⭐⭐⭐

| 特性 | FDDP | ProxDDP |
|------|------|---------|
| 等式约束(接触不滑) | 嵌入动力学 | ALM 显式处理 |
| 不等式约束(关节限位) | 不直接支持(需 Box-FDDP) | **ALM 原生支持** |
| 摩擦锥约束 | 不直接支持 | **ALM 原生支持** |
| 收敛速度(无约束） | 快 | 略慢(ALM 外层开销) |
| 初始猜测鲁棒性 | FDDP 比 DDP 好 | **更好**(ALM 平滑化) |
| 实时性 | 成熟(Crocoddyl 调优) | 需要更多调参 |
| 论文发表 | Box-FDDP: Auton. Robots 2022 | ProxDDP: T-RO 2025 |

### 54.9.5 ParallelRiccati:打破 30 年的教条 ⭐⭐⭐⭐

**"backward pass 不可并行"**:这是 DDP 社区从 1970 年 Jacobson 的论文以来的共识。$V_k$ 的计算依赖 $V_{k+1}$,必须严格顺序从 $N$ 到 $0$。

**Jallet et al. (RSS 2024) 的突破**:Riccati 递推可以表达为**矩阵链乘**,而链乘可以用 **parallel scan** 算法并行化。(注意:并行 backward pass 这一贡献出自 RSS 2024 的 *Parallel and Proximal...* 一文,而约束处理的 ProxDDP 出自 T-RO 2025 一文,两者同属 Aligator 库但是不同论文,引用时勿混淆。)

**核心数学**:每一步的 Riccati 递推可以写成一个仿射变换:

$$\begin{bmatrix} V_{\mathbf{xx},k} \\ V_{\mathbf{x},k} \\ 1 \end{bmatrix} = \underbrace{\mathbf{T}_k}_{\text{Riccati operator}} \begin{bmatrix} V_{\mathbf{xx},k+1} \\ V_{\mathbf{x},k+1} \\ 1 \end{bmatrix}$$

其中 $\mathbf{T}_k$ 是一个**矩阵**,可以从 $Q_{\mathbf{xx}}, Q_{\mathbf{uu}}, Q_{\mathbf{ux}}$ 等构造。

**链乘的并行化**:

$$V_0 = \mathbf{T}_0 \cdot \mathbf{T}_1 \cdot \mathbf{T}_2 \cdots \mathbf{T}_{N-1} \cdot V_N$$

矩阵链乘是**可结合**的 $(\mathbf{A} \cdot \mathbf{B}) \cdot \mathbf{C} = \mathbf{A} \cdot (\mathbf{B} \cdot \mathbf{C})$,因此可以用二叉树合并:

```
Parallel Scan (二叉树合并):

Level 0: T₀  T₁  T₂  T₃  T₄  T₅  T₆  T₇   ← 8 个 Riccati 算子
              ↘  ↙      ↘  ↙      ↘  ↙      ↘  ↙
Level 1:    T₀₁      T₂₃      T₄₅      T₆₇   ← 4 次乘法(并行)
              ↘      ↙          ↘      ↙
Level 2:     T₀₁₂₃           T₄₅₆₇            ← 2 次乘法(并行)
                ↘              ↙
Level 3:        T₀₁₂₃₄₅₆₇                     ← 1 次乘法

总共: log₂(8) = 3 步,而不是 7 步(串行)
```

**复杂度对比**:

| 方法 | 时间复杂度 | N=200 时间步 |
|------|-----------|-------------|
| 串行 Riccati | $O(N \cdot n_x^3)$ | 200 步顺序 |
| Parallel Scan | $O(\log N \cdot n_x^3)$ on $P$ 处理器 | ~8 步 (32 核) |

> 🧠 **深入理解**: Parallel scan 算法在计算机科学中早已成熟(parallel prefix sum),但把它应用到 Riccati 递推上需要一个关键观察:Riccati 算子构成一个**半群**,即它满足结合律。这不是显然的——需要仔细证明 $\mathbf{T}_k$ 的乘法确实是结合的。

**性能数据** (Jallet et al. RSS 2024):

```
ParallelRiccati 性能 (ANYmal 全身 MPC):
┌─────────────────────────────────────────┐
│ N=200, nx=37, nu=12                     │
│                                         │
│ 串行 backward pass:  0.8 ms            │
│ ParallelRiccati (4核): 0.35 ms (2.3x)  │
│ ParallelRiccati (8核): 0.18 ms (4.4x)  │
│ ParallelRiccati (32核): 0.10 ms (8.0x) │
│                                         │
│ 注:额外开销来自合并步的同步              │
└─────────────────────────────────────────┘
```

> ⚠️ **陷阱**: ParallelRiccati 的每次乘法涉及 $(2n_x+1) \times (2n_x+1)$ 的矩阵乘法,比普通 Riccati 的 $n_x \times n_x$ 操作大得多。因此,只有 N 足够大(> 50)时才值得并行化。对于 N=10 的短 horizon MPC,串行 Riccati 反而更快。

**练习 54.9a** (⭐⭐⭐): 用 Eigen 实现一个简化版的 Parallel Scan:给定 N 个 $3 \times 3$ 矩阵,用二叉树合并计算它们的连乘。与顺序连乘对比结果的正确性和速度。

**练习 54.9b** (⭐⭐⭐⭐): 阅读 Jallet et al. (RSS 2024, *Parallel and Proximal...*) 关于并行 LQ 求解的章节,用自己的话复述 ParallelRiccati 的完整算法。重点理解:为什么 Riccati 算子满足结合律?

---

## 54.10 Crocoddyl 实战:从零搭建四足 trot 优化 ⭐⭐⭐

### 54.10.1 整体流程 ⭐⭐

```
四足 trot 轨迹优化搭建流程:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Step 1: 加载机器人模型 (Pinocchio)
Step 2: 定义接触序列 (哪条腿在什么时候触地)
Step 3: 为每个时间步创建 ActionModel
        ├─ 有接触: ContactFwdDynamics + 接触约束
        └─ 无接触(摆动): FreeFwdDynamics
Step 4: 定义代价函数
        ├─ 末端 CoM 追踪
        ├─ 足端位姿追踪
        ├─ 控制正则化
        └─ 状态正则化
Step 5: 组装 ShootingProblem
Step 6: 创建 FDDP Solver + 求解
Step 7: 可视化结果
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 54.10.2 Python 完整代码 ⭐⭐

**代码结构概述**：下面的代码实现了一个完整的四足 trot 轨迹优化。在阅读代码之前，先理解其整体架构：

1. **加载模型**：从 `example_robot_data` 获取 ANYmal 四足机器人的 URDF 模型，Pinocchio 将其解析为 `Model` + `Data` 对象。
2. **定义接触配置**：trot 步态中对角脚同时着地，因此每个 ActionModel 需要指定当前时间步哪些脚处于接触状态。接触模型使用 `ContactModel3D`（只有力，没有力矩），这对应于点接触假设。
3. **定义代价函数**：残差式代价 `CostModelResidual` 由一个 `ResidualModel`（计算残差向量）和一个 `ActivationModel`（定义惩罚函数，默认 L2）组成。这种分离的设计使得同一个残差可以用不同的惩罚函数（L2, weighted L2, smooth L1 等）。
4. **组装 ShootingProblem**：将 $N$ 个 `IntegratedActionModel`（每个包含接触 + 代价 + 积分器）串成一个 `ShootingProblem`。
5. **求解**：`SolverFDDP` 执行 FDDP 算法，内部交替 backward pass（计算反馈增益）和 forward pass（rollout 新轨迹）。

这个流程等价于：手动写出 NLP $\min \sum l_k(x_k, u_k)$ s.t. $x_{k+1} = f_k(x_k, u_k)$，然后用 FDDP 利用 Markov 结构高效求解。理解这一点，代码中的每一步就都有了对应的数学意义。

```python
import numpy as np
import crocoddyl
import pinocchio
import example_robot_data

# ========================================================
# Step 1: Load robot model
# ========================================================
robot = example_robot_data.load("anymal")
model = robot.model
state = crocoddyl.StateMultibody(model)
actuation = crocoddyl.ActuationModelFloatingBase(state)

# ========================================================
# Step 2: Define contact frames
# ========================================================
foot_frames = ["LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"]
foot_ids = [model.getFrameId(f) for f in foot_frames]

# Trot gait: diagonal legs move together
# Phase 0: LF+RH stance, RF+LH swing
# Phase 1: RF+LH stance, LF+RH swing
trot_phases = [
    {"stance": [0, 3], "swing": [1, 2]},  # LF+RH stance
    {"stance": [1, 2], "swing": [0, 3]},  # RF+LH stance
]

dt = 0.01   # 10 ms time step
N_phase = 25  # steps per phase
N = N_phase * len(trot_phases)  # total steps

# ========================================================
# Step 3-4: Create ActionModels for each time step
# ========================================================
running_models = []

q_standing = model.referenceConfigurations["standing"]
for phase in trot_phases:
    # 从站立位形计算各脚的标称接触位置
    pinocchio.forwardKinematics(model, robot.data, q_standing)
    pinocchio.updateFramePlacements(model, robot.data)
    nominal_foot_pos = {idx: robot.data.oMf[foot_ids[idx]].translation.copy() for idx in range(4)}

    for step in range(N_phase):
        # Create contact model
        contacts = crocoddyl.ContactModelMultiple(state, actuation.nu)
        for idx in phase["stance"]:
            contact = crocoddyl.ContactModel3D(
                state, foot_ids[idx],
                nominal_foot_pos[idx],  # 各脚的标称接触位置（由正运动学计算）
                pinocchio.LOCAL_WORLD_ALIGNED,
                actuation.nu,
                np.array([0., 50.])  # Baumgarte gains
            )
            contacts.addContact(foot_frames[idx], contact)

        # Create cost model
        costs = crocoddyl.CostModelSum(state, actuation.nu)
        
        # State regularization
        x_ref = np.concatenate([
            model.referenceConfigurations["standing"],
            np.zeros(model.nv)
        ])
        state_residual = crocoddyl.ResidualModelState(
            state, x_ref, actuation.nu)
        state_cost = crocoddyl.CostModelResidual(
            state,
            crocoddyl.ActivationModelWeightedQuad(
                np.ones(state.ndx) * 1e-1),
            state_residual)
        costs.addCost("state_reg", state_cost, 1e-1)
        
        # Control regularization
        ctrl_residual = crocoddyl.ResidualModelControl(
            state, actuation.nu)
        ctrl_cost = crocoddyl.CostModelResidual(
            state,
            crocoddyl.ActivationModelQuad(actuation.nu),
            ctrl_residual)
        costs.addCost("ctrl_reg", ctrl_cost, 1e-3)

        # Swing foot tracking (for swing legs)
        for idx in phase["swing"]:
            swing_target = np.array([0., 0., 0.1])  # lift 10cm
            frame_residual = crocoddyl.ResidualModelFrameTranslation(
                state, foot_ids[idx], swing_target, actuation.nu)
            frame_cost = crocoddyl.CostModelResidual(
                state,
                crocoddyl.ActivationModelQuad(3),
                frame_residual)
            costs.addCost(
                f"swing_{foot_frames[idx]}", frame_cost, 1e2)

        # Create differential action model
        diff_model = \
            crocoddyl.DifferentialActionModelContactFwdDynamics(
                state, actuation, contacts, costs, 0., True)
        
        # Wrap with Euler integrator
        int_model = crocoddyl.IntegratedActionModelEuler(
            diff_model, dt)
        running_models.append(int_model)

# ========================================================
# Terminal model (no dynamics, just terminal cost)
# ========================================================
terminal_costs = crocoddyl.CostModelSum(state)
com_residual = crocoddyl.ResidualModelCoMPosition(
    state, np.array([0.5, 0., 0.35]))  # target CoM
com_cost = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelQuad(3), com_residual)
terminal_costs.addCost("com_goal", com_cost, 1e4)

terminal_model = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelContactFwdDynamics(
        state, actuation,
        crocoddyl.ContactModelMultiple(state, actuation.nu),
        terminal_costs, 0., True), 0.)

# ========================================================
# Step 5-6: Assemble problem and solve
# ========================================================
x0 = np.concatenate([
    model.referenceConfigurations["standing"],
    np.zeros(model.nv)])

problem = crocoddyl.ShootingProblem(
    x0, running_models, terminal_model)
solver = crocoddyl.SolverFDDP(problem)
solver.setCallbacks([crocoddyl.CallbackVerbose()])

# Initialize with standing configuration
xs_init = [x0] * (N + 1)
us_init = solver.problem.quasiStatic(xs_init[:-1])

solver.solve(xs_init, us_init, maxiter=100, isFeasible=False)

print(f"Converged in {solver.iter} iterations")
print(f"Final cost: {solver.cost:.4f}")
print(f"Final gap norm: "
      f"{max(np.linalg.norm(g) for g in solver.fs):.6f}")
```

> ⚠️ **陷阱**: `isFeasible=False` 告诉 FDDP 初始轨迹**不满足动力学**(有 gaps)。如果你传 `True` 但轨迹实际不可行,FDDP 的 line search 会行为异常。保险起见,warm-start 时始终传 `False`。

> 💡 **洞察**: `quasiStatic()` 是一个聪明的初始化方法:它计算让机器人在每个构型下"静力平衡"的控制输入。这比零初始化好得多,因为零扭矩会让机器人在重力下自由坠落。

**练习 54.10a** (⭐⭐⭐): 修改上面的代码,把 trot 步态改为 bound 步态(前两脚同时、后两脚同时)。观察优化出的运动有什么不同。

**练习 54.10b** (⭐⭐⭐): 添加摩擦锥约束:确保每个接触点的接触力在摩擦锥内。提示:使用 `CostModelResidual` + `ResidualModelContactFrictionCone`。

---

## 54.11 DDP 在 MPC 中的使用模式 ⭐⭐⭐

### 54.11.1 Warm-Starting:MPC 的核心技巧 ⭐⭐

MPC 每个控制周期(通常 1-10ms)需要重新求解一次 OCP。关键问题:**如何用上一次的解来"热启动"本次求解?**

```
MPC Warm-Start (时间平移):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
上一次 MPC 解 (时刻 t):
  x₀ᵒˡᵈ u₀ᵒˡᵈ x₁ᵒˡᵈ u₁ᵒˡᵈ ... x_{N-1}ᵒˡᵈ u_{N-1}ᵒˡᵈ x_Nᵒˡᵈ

本次 MPC 初始猜测 (时刻 t+dt), 时间平移:
  x₁ᵒˡᵈ u₁ᵒˡᵈ x₂ᵒˡᵈ u₂ᵒˡᵈ ... x_Nᵒˡᵈ [u_ext]   [x_ext]
                                        ↑          ↑
                                    新增控制     新增状态
                                  (通常用u_{N-1}ᵒˡᵈ 或 零)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**关键点**:时间平移后,新增的最后一步是"猜测"——这产生了 FDDP 的 gap。这就是 FDDP 对 MPC 至关重要的原因。

### 54.11.2 实时性策略:不等收敛 ⭐⭐⭐

MPC 不需要每次都求解到收敛!常见策略:

```
策略对比:
┌────────────────────────────────────────────────────┐
│ 策略 1: 固定迭代次数                                │
│   每次 MPC 只跑 1-3 次 DDP 迭代                    │
│   优点:时间可预测                                  │
│   缺点:可能远未收敛                                │
│                                                     │
│ 策略 2: 固定时间预算                                │
│   给 DDP 5ms 的时间预算,能跑几次跑几次             │
│   优点:实时性保证                                  │
│   缺点:迭代次数不确定                              │
│                                                     │
│ 策略 3: 双线程 (OCS2 方案)                         │
│   线程 A: 连续运行 DDP/SQP 直到收敛               │
│   线程 B: 取最新可用的解执行                       │
│   优点:解质量最高,求解器不受实时约束              │
│   缺点:使用的解可能不是最新状态的                  │
│   足式/110_OCS2完整栈与双线程MPC 详细展示了 OCS2 如何用 lock-free buffer     │
│   在两个线程之间安全交换轨迹数据                   │
└────────────────────────────────────────────────────┘
```

### 54.11.3 反馈增益的使用 ⭐⭐⭐

DDP 的 backward pass 给出了反馈增益 $\mathbf{K}_k$。在 MPC 两次求解之间,可以用这个增益做闭环控制:

$$\mathbf{u}_{\text{applied}} = \mathbf{u}_k^* + \mathbf{K}_k (\mathbf{x}_{\text{measured}} - \mathbf{x}_k^*)$$

```
MPC + DDP 反馈增益:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
时间轴:
 ─────┬──────────┬──────────┬─────────→
      │          │          │
    MPC求解₁   MPC求解₂   MPC求解₃
      │          │          │
      └──────────┘  这段时间内:
       用 K_k 做反馈  (比零阶保持好得多)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

> 💡 **洞察**: 这是 DDP 相对于 SQP 的天然优势。SQP 只给出开环轨迹 $\{\mathbf{u}_k^*\}$,没有反馈增益。OCS2 的 SQP 方案需要额外跑一次 LQR 来得到反馈增益,而 DDP 的反馈增益是"免费"的副产品。

> ⚠️ **陷阱**: 反馈增益 $\mathbf{K}_k$ 只在当前最优解附近有效。如果实际状态偏离太多(比如外部扰动),$\mathbf{K}_k$ 的修正可能不够——需要等下一次 MPC 重新规划。

**练习 54.11** (⭐⭐⭐): 在 54.10 的四足 trot 例子上实现一个简单的 MPC 循环:每次只跑 3 次 FDDP 迭代,然后用反馈增益 $\mathbf{K}_0$ 应用第一步控制。在循环中引入随机扰动,观察系统的鲁棒性。

---

## 54.12 DDP vs SQP:两大流派的对比 ⭐⭐⭐

### 54.12.1 核心分歧 ⭐⭐

```
DDP 流派 (Crocoddyl, MuJoCo)       SQP 流派 (OCS2, ALTRO)
┌──────────────────────────┐       ┌──────────────────────────┐
│ 动力学: 嵌入 rollout     │       │ 动力学: 作为约束处理     │
│ 约束: 不自然             │       │ 约束: 统一处理           │
│ 输出: 轨迹 + 反馈增益    │       │ 输出: 只有轨迹           │
│ 并行: calc/calcDiff      │       │ 并行: QP 子问题          │
│ Warm-start: FDDP gaps    │       │ Warm-start: 移位+投影    │
│ 代表: Crocoddyl+Pinocchio│       │ 代表: OCS2 (→足式/110_OCS2完整栈与双线程MPC)       │
└──────────────────────────┘       └──────────────────────────┘
```

### 54.12.2 场景分析 ⭐⭐

| 场景 | 推荐 | 理由 |
|------|------|------|
| 离线轨迹优化 | DDP | 收敛快,反馈增益可用于仿真 |
| 实时 MPC (约束少) | DDP/FDDP | 每次迭代快,warm-start 好 |
| 实时 MPC (约束多) | SQP 或 ProxDDP | 约束处理更优雅 |
| 工业级产品 | OCS2 (SQP) | 代码成熟度,ETH ANYmal 验证 |
| 研究前沿 | Aligator (ProxDDP) | 最灵活,ParallelRiccati |

> 🧠 **深入理解**: 两派的分歧不是技术水平的差异,而是**设计哲学**的差异。DDP 把动力学"嵌入"求解器(通过 rollout),获得了效率和反馈增益;SQP 把动力学"外化"为约束,获得了通用性和约束处理能力。ProxDDP 试图兼得两者——用 ALM 在 DDP 框架内处理约束。

---

## 🔧 故障排查手册

> 本表覆盖 DDP/FDDP/Crocoddyl 工程中最高频的 8 类故障。列顺序遵循"症状(可观测)→可能原因(机制)→排查步骤(可执行)→相关章节"。注意排查时**先看现象再猜原因**——很多人一上来就调 $\mu$,结果真正的 bug 是 URDF 惯量写错了。

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| DDP 迭代代价不下降,反复回退步长 $\alpha$ 直至 $\alpha_{\min}$ | 正则化 $\mu$ 过小使 $Q_{\mathbf{uu}}$ 不正定,或初始轨迹离最优解太远 | 1. 打印 $Q_{\mathbf{uu}}$ 的最小特征值确认正定性 2. 增大初始正则化 $\mu_0$(从 $10^{-9}$ 提到 $10^{-4}$) 3. 用 `quasiStatic()` 或逆动力学生成更好初值 | §54.2.8, §54.3 |
| Forward pass 出现 NaN/Inf | 动力学积分发散——$\boldsymbol{u}$ 过大导致关节加速度爆炸;或 `dt` 太大使 Euler 不稳定 | 1. 打印 forward pass 每步状态范数,定位发散起始步 2. 加 control bounds(Box-FDDP) 3. 减小 `dt` 或改 RK4 4. 检查 URDF 惯量/质量是否为正定 | §54.2.7, §54.4 |
| 求解结果中接触力违反摩擦锥 | 无约束 FDDP 下摩擦锥仅是代价惩罚,非硬约束 | 1. 增大 `ResidualModelContactFrictionCone` 权重 2. 切换 ProxDDP/CSQP 用硬约束 3. 对比约束违反量与摩擦锥余量 | §54.6.3, §54.9.2 |
| MPC warm-start 后首次迭代代价反而上升 | 移位后追加的末步初值不良,产生大 gap | 1. 确认平移逻辑(shift + extrapolate)正确 2. 末步控制复制倒数第二步而非填零 3. 首次迭代多跑几步消化 gap 4. 确认 `isFeasible=False` | §54.4, §54.11.1 |
| `calcDiff()` 耗时远超预期(>5ms) | 每次调用重新 `createData()` 触发堆分配;或误用 RK4 | 1. 确认用 `problem.createData()` 一次性预分配 2. 用 `EIGEN_RUNTIME_NO_MALLOC` 检测运行时 malloc 3. 分别测 `calc()`/`calcDiff()` 耗时定位瓶颈 | §54.5.3, §54.6.5 |
| `calcDiff()` 段错误(segfault) | Model 与 Data 不匹配,或 Data 未由对应 Model 的 `createData()` 生成 | 1. 确认每个 `running_models_[t]` 配对正确的 `running_datas_[t]` 2. 自定义 ActionModel 时检查 `createData()` 返回类型 3. 用 AddressSanitizer 定位越界 | §54.5.4 |
| OpenMP 并行结果与串行不一致 | 多线程共享同一 Data,或 Eigen 临时对象 data race | 1. 确认每时间步独立 Data 2. 用 ThreadSanitizer 检测竞态 3. 检查 Eigen 16 字节对齐(`EIGEN_MAKE_ALIGNED_OPERATOR_NEW`) | §54.5.4, §54.8 |
| FDDP 的 gap 范数不下降反而振荡 | Armijo line search 参数不当,或正则化模式选错 | 1. 打印每次迭代 `max‖fs‖` 观察单调性 2. 检查 line search 接受准则 3. 尝试切换 Crocoddyl 的 `xreg`/`ureg` 两种正则模式 | §54.4.6, §54.2.8B |

---

## 54.13 本章小结

```
知识图谱:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                    NLP (Ipopt)
                       │
              "利用时间 Markov 结构"
                       │
                      DDP
                    ╱     ╲
               iLQR        FDDP
             (丢弃f_xx)  (允许gaps)
                            │
                     ┌──────┴──────┐
                Crocoddyl      Aligator
              (ActionModel)    (StageModel)
              (Virtual poly)   (C++20 Concepts)
              (OpenMP calc)    (ParallelRiccati)
                     │              │
                   FDDP          ProxDDP
                                (ALM约束)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 自检问题

1. DDP 的 backward pass 为什么要从 $t = N$ 开始?如果从 $t = 0$ 开始会怎样?
2. iLQR 丢弃了什么?这在什么条件下影响最大?
3. FDDP 的 "gap" 是什么?为什么 MPC warm-start 会产生 gaps?
4. Crocoddyl 为什么用虚函数而不是 CRTP?如果 `calc()` 只做一次加法而不是 Pinocchio 运算,结论会改变吗?
5. ParallelRiccati 的 parallel scan 为什么能把 $O(N)$ 降到 $O(\log N)$?前提条件是什么?

---

## 累积项目:四足控制器添加"轨迹优化模块"

### 项目目标

在前几章搭建的四足控制器(Pinocchio 模型 + WBC)基础上,添加 DDP 轨迹优化模块,形成完整的 MPC -> WBC -> 扭矩控制栈。

```
四足控制器架构:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[步态管理器 (足式/120_步态管理与接触序列)] → 接触序列
         │
         ▼
[DDP/FDDP 轨迹优化 (本章)] → 最优轨迹 + 反馈增益
         │
         ▼
[WBC (足式/90_WBC分层优化与TSID)] → 关节扭矩
         │
         ▼
[电机驱动 (足式/180_腿足硬件栈)]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 任务分解

| 子任务 | 难度 | 内容 |
|--------|------|------|
| T54.1 | ⭐⭐ | 用 Crocoddyl 建模 Go2 的 trot 优化(Python) |
| T54.2 | ⭐⭐⭐ | 把 Python 原型翻译为 C++ |
| T54.3 | ⭐⭐⭐ | 实现 MPC 循环:warm-start + 固定迭代 |
| T54.4 | ⭐⭐⭐⭐ | 把 MPC 输出接入 足式/90_WBC分层优化与TSID 的 WBC |
| T54.5 | ⭐⭐⭐⭐ | 用 Aligator/ProxDDP 替换 Crocoddyl FDDP |

---

## 精读点与项目文件索引

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **Crocoddyl** | `include/crocoddyl/core/action-base.hpp` | `ActionModelAbstract` 虚基类 | 代码阅读 |
| **Crocoddyl** | `include/crocoddyl/multibody/actions/free-fwddyn.hpp` | 无接触全身动力学 | 代码阅读 |
| **Crocoddyl** | `include/crocoddyl/multibody/actions/contact-fwddyn.hpp` | **接触动力学 KKT**(腿足核心) | 代码阅读(核心) |
| **Crocoddyl** | `include/crocoddyl/core/solvers/fddp.hpp` | **FDDP 实现** | 代码阅读(核心) |
| **Crocoddyl** | `include/crocoddyl/core/integrator/euler.hpp` | Euler 积分器 | 代码阅读 |
| **Crocoddyl** | `examples/cpp/arm_manipulation.cpp` | 机械臂轨迹优化 | 实战 |
| **Crocoddyl** | `examples/cpp/quadrupedal_walking.cpp` | **四足 trot 轨迹优化** | 实战(重要) |
| **Aligator** | `include/aligator/solvers/proxddp/solver-proxddp.hpp` | ProxDDP 求解器 | 前沿阅读 |
| **Aligator** | `include/aligator/gar/parallel-riccati.hpp` | **ParallelRiccati** | 前沿阅读 |

---

## API 速查表

> 下表汇总本章涉及的 Crocoddyl 核心 API(以 Python 接口为主,C++ 接口同名)。签名以 Crocoddyl 2.x 为准;不同小版本参数可能微调,以本地 `help(crocoddyl.XXX)` 为准。**记忆抓手**:几乎所有类都遵循 `Model`(无状态、可共享)+ `Data`(有状态、每线程一份)+ `createData()` 的三件套模式。

### 状态与驱动

| API | 作用 | 关键参数/返回 |
|------|------|-------------|
| `crocoddyl.StateMultibody(model)` | 用 Pinocchio 模型构造状态流形(含 $SE(3)$ 浮动基) | 提供 `ndx`(切空间维度)、`integrate`/`diff`(流形上的加减) |
| `crocoddyl.StateVector(nx)` | 欧氏状态空间(无流形) | 简单系统用 |
| `crocoddyl.ActuationModelFloatingBase(state)` | 浮动基驱动:前 6 维(基座)不可驱动 | `.nu` = 关节数 |
| `crocoddyl.ActuationModelFull(state)` | 全驱动(每个状态都有控制) | 机械臂用 |

### 动力学(DifferentialActionModel)

| API | 作用 | 关键参数 |
|------|------|---------|
| `DifferentialActionModelFreeFwdDynamics(state, actuation, costs)` | 无接触全身动力学(内部调 Pinocchio ABA) | 机械臂/自由飞行 |
| `DifferentialActionModelContactFwdDynamics(state, actuation, contacts, costs, JMinvJt_damping, enable_force)` | 含接触 KKT 动力学(腿足核心) | `JMinvJt_damping`:KKT 阻尼正则;`enable_force`:是否输出接触力导数 |
| `IntegratedActionModelEuler(diffModel, dt)` | 把连续动力学离散化(显式 Euler) | MPC 默认 |
| `IntegratedActionModelRK(diffModel, RKType, dt)` | RK2/RK3/RK4 积分 | 离线、大 `dt` |

### 接触与代价

| API | 作用 | 关键参数 |
|------|------|---------|
| `ContactModelMultiple(state, nu)` | 接触集合容器,`.addContact(name, model)` 增删 | 步态切换时改这个 |
| `ContactModel3D(state, frameId, ref, type, nu, gains)` | 点接触(3 维力,无力矩) | `gains`=[Kp, Kd] Baumgarte 稳定化 |
| `ContactModel6D(...)` | 面接触(6 维力+力矩) | 人形脚掌 |
| `CostModelSum(state, nu)` | 代价集合,`.addCost(name, cost, weight)` | 权重即代价系数 |
| `CostModelResidual(state, activation, residual)` | 残差式代价 = 激活 ∘ 残差 | 见 §54.6.4 |
| `ResidualModelState / Control / FrameTranslation / CoMPosition / ContactForce / ContactFrictionCone` | 各类残差 | 残差 = 当前量 − 参考量 |
| `ActivationModelQuad / WeightedQuad / Smooth1Norm / QuadraticBarrier` | 激活函数 $\Phi(\mathbf{r})$ | `QuadraticBarrier` 可做软限位 |

### 问题与求解器

| API | 作用 | 关键返回/字段 |
|------|------|-------------|
| `ShootingProblem(x0, runningModels, terminalModel)` | 组装 OCP;`.createData()` 一次性预分配所有 Data | `.calc/.calcDiff` 内部 OpenMP 并行 |
| `problem.quasiStatic(xs)` | 计算静力平衡控制(优于零初值) | 见 §54.10 陷阱 |
| `SolverFDDP(problem)` | FDDP 求解器 | 容忍不可行初值 |
| `SolverBoxFDDP(problem)` | Box-FDDP(FDDP + 控制限位 box-QP) | 见 §54.4B |
| `SolverIntro / SolverCSQP`(Crocoddyl 2.1+) | 带等式/不等式硬约束的求解器 | 摩擦锥硬约束 |
| `solver.solve(xs_init, us_init, maxiter, isFeasible)` | 主求解;`isFeasible=False` 表示初值不可行 | 返回 `True` 表示收敛 |
| `solver.setCallbacks([CallbackVerbose()])` | 注册回调(打印/记录) | 调试必备 |
| `solver.xs / .us / .K / .fs` | 最优状态/控制/反馈增益/gap 序列 | `.K[k]` 即 §54.11.3 的 $\mathbf{K}_k$ |
| `solver.iter / .cost` | 实际迭代数 / 最终代价 | 收敛诊断 |

---

## 实战练习汇总

### A 型(基础)

**练习 54.A1** (⭐): 手写 2D 小车的 iLQR,用 Eigen 实现 backward pass + forward pass。

**练习 54.A2** (⭐⭐): 用 Crocoddyl Python 接口求解 Panda 机械臂的末端追踪问题,N=100, FDDP。

**练习 54.A3** (⭐⭐): 运行 `examples/cpp/quadrupedal_walking.cpp`,理解接触模型和代价模型的配置。

### B 型(进阶)

**练习 54.B1** (⭐⭐⭐): 精读 `contact-fwddyn.hpp` 的 `calc()` 和 `calcDiff()`,回答:它如何调用 Pinocchio 计算 $\mathbf{M}, \mathbf{J}_c, \mathbf{h}$?如何求解 KKT 系统?

**练习 54.B2** (⭐⭐⭐): 实现 FDDP 的 gap 处理:在 54.A1 的 2D 小车上,给不可行的初始轨迹,实现 gap-aware forward pass。

**练习 54.B3** (⭐⭐⭐⭐): 用 Crocoddyl + Pinocchio 搭建 Go2 四足的 trot MPC 原型(Python),实现 warm-start 循环。

### C 型(研究级)

**练习 54.C1** (⭐⭐⭐⭐): 阅读 Jallet et al. T-RO 2025,复述 ParallelRiccati 的核心算法,并用 Eigen 实现简化版。

**练习 54.C2** (⭐⭐⭐⭐): 对比 Crocoddyl FDDP 和 Aligator ProxDDP 在同一个问题上的收敛速度、约束满足度、求解时间。

### 思考题

**思考 54.T1**: DDP vs SQP 的选型——如果你要做一个需要处理摩擦锥约束 + 关节限位的 MPC,你选 FDDP (Crocoddyl)、ProxDDP (Aligator) 还是 SQP (OCS2)?为什么?

**思考 54.T2**: Crocoddyl 的 ActionModel 不模板化 Scalar(不像 Pinocchio)。为什么?如果要支持 CppAD,怎么办?

**思考 54.T3**: ParallelRiccati 能否移植到 GPU?GPU 的 SIMT 架构与 parallel scan 的匹配度如何?

---

## 研究前沿与论文阅读

### 必读经典

1. **Jacobson D. H., Mayne D. Q. (1970)** *Differential Dynamic Programming*. 原始 DDP 专著。
2. **Li W., Todorov E. (2004)** "Iterative Linear Quadratic Regulator Design for Nonlinear Biological Movement Systems". ICINCO. iLQR 的奠基。
3. **Tassa Y., Mansard N., Todorov E. (2014)** "Control-Limited Differential Dynamic Programming". ICRA. 带控制限位的 DDP。
4. **Mastalli C., et al. (2020)** "Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control". ICRA 2020 (Paris), arXiv 1909.04947. **Crocoddyl 主论文**。
5. **Mastalli C., et al. (2022)** "A Feasibility-Driven Approach to Control-Limited DDP". Autonomous Robots 46(8):985-1005, arXiv 2010.00411. **Box-FDDP 完整版**(feasibility-driven + control-bounded 双模式)。

### 近期进展 (2023-2025)

6. **Jallet W., et al. (2025)** "PROXDDP: Proximal Constrained Trajectory Optimization". IEEE T-RO 41:2605-2624. **ProxDDP 增广拉格朗日约束**, 博士必读。
7. **Jallet W., Dantec E., et al. (2024)** "Parallel and Proximal Constrained Linear-Quadratic Methods for Real-Time Nonlinear MPC". RSS 2024 (Delft). **ParallelRiccati ($O(\log N)$ backward pass)**。
8. **Mastalli C., et al. (2023)** "Inverse-Dynamics MPC via Nullspace Resolution". IEEE T-RO, arXiv 2209.05375. **逆动力学 MPC + 零空间约束消去**,首次在 ANYmal 硬件上实现 ID-MPC(计算量减少达 47.3%),已并入 Crocoddyl。
9. **(2025)** "Primal-Dual iLQR for GPU-Accelerated Learning and Control in Legged Robots". arXiv 2506.07823. **GPU 并行 primal-dual associative scan**,$O(n\log N+m)$,JAX 实现,支持 MPC-in-the-loop 学习。
10. **Kleff S., et al. (2022)** "On the Derivation of Contact Dynamics in Arbitrary Frames". Humanoids 2022. 接触动力学导数的推广。

> 上述 6、7 两篇正是 **Aligator**(LAAS-CNRS / Inria,Jallet、Carpentier 等开发,随 Pinocchio 3.x 一同推出的新一代轨迹优化库)的核心成果——ProxDDP 提供约束处理,ParallelRiccati 提供并行 backward pass。其完整架构、代码示例与性能对比见 §54.9。

### MuJoCo MPC (Predictive Sampling) 与 DDP 的对比 ⭐⭐⭐

**MuJoCo MPC** (Howell et al., 2022) 采用了与 DDP 完全不同的轨迹优化策略——**Predictive Sampling**（预测采样），也称为 Model Predictive Path Integral (MPPI) 的变体。

**核心思想差异**：

| 维度 | DDP (Crocoddyl/Aligator) | MuJoCo MPC (Predictive Sampling) |
|------|--------------------------|----------------------------------|
| **优化方法** | 基于梯度的二阶方法（Gauss-Newton） | 无梯度的采样方法（蒙特卡罗） |
| **需要导数？** | 需要动力学的一阶导数（$f_x, f_u$） | 不需要——只需要仿真器前向模拟 |
| **处理不可微动力学？** | 困难（接触切换处梯度不连续） | 自然处理——采样不关心可微性 |
| **并行性** | 串行 backward pass（除非 ParallelRiccati） | 天然并行——每条采样轨迹独立 |
| **最优性** | 局部最优（二阶收敛） | 统计意义上接近最优（采样精度） |
| **样本效率** | 高（梯度提供精确方向） | 低（需要大量采样） |
| **计算硬件** | CPU 友好（矩阵运算） | GPU 友好（大规模并行仿真） |

**什么时候选 DDP，什么时候选 Predictive Sampling？**

- **动力学可微且需要精确最优**：选 DDP / ProxDDP。Pinocchio 提供了精确的解析导数，DDP 可以达到二阶收敛精度。
- **动力学含不可微环节（复杂接触、变摩擦）且有 GPU**：选 Predictive Sampling / MPPI。MuJoCo MJX 在 GPU 上可以并行仿真数千条轨迹，暴力搜索补偿了采样方法的低效率。
- **混合策略（前沿方向）**：用 DDP 计算名义轨迹，然后在 DDP 解的邻域内用采样方法做鲁棒化。这结合了两者的优点——DDP 的精度和采样的鲁棒性。

> **本质洞察**：DDP 的 backward pass 本质上是一个"信息从未来传递到现在"的过程——$V_{xx}(k)$ 编码了"从时间 $k$ 到终点的累积代价对状态扰动的敏感度"。这个信息传递是严格因果的（只能从后往前），因此 backward pass 天然串行。ParallelRiccati 的突破在于发现了一种数学等价形式，使得这个"因果传递"可以用非因果的并行计算来实现。

> **本质洞察**：DDP 和 MPC 的关系不是"DDP 是 MPC 的一种求解器"，而是"DDP 利用了控制问题的 Markov 结构来高效求解 NLP"。任何 NLP 求解器（Ipopt, SQP）都可以做 MPC，但只有 DDP 家族利用了"时间步之间只有相邻耦合"这一特殊稀疏结构。这就是为什么 DDP 的复杂度是 $O(N)$ 而通用 NLP 是 $O(N^3)$——不是因为 DDP 更聪明，而是因为它利用了问题结构。

### 近年关键进展深读 (2023-2025) ⭐⭐⭐⭐

本章主线讲的是"forward-dynamics + 串行 backward pass"的经典范式。近三年有两条研究主线正在改写这个范式,博士方向的读者应当熟悉它们的核心思想。

**进展一:Inverse-Dynamics MPC(逆动力学 MPC)。** Mastalli et al., *Inverse-Dynamics MPC via Nullspace Resolution*(IEEE T-RO, 2023;arXiv 2209.05375)。

回顾 §54.6.3:`DifferentialActionModelContactFwdDynamics` 走的是**正动力学**(forward dynamics)路线——给定扭矩 $\boldsymbol{\tau}$,解 KKT 系统算出加速度 $\ddot{\mathbf{q}}$ 和接触力 $\boldsymbol{\lambda}$。逆动力学 MPC 反过来:把 $(\ddot{\mathbf{q}}, \boldsymbol{\lambda})$ 也当作决策变量,动力学方程 $\mathbf{M}\ddot{\mathbf{q}}+\mathbf{h}=\mathbf{S}^T\boldsymbol{\tau}+\mathbf{J}_c^T\boldsymbol{\lambda}$ 变成一组**等式约束**。

```
正动力学 OCP vs 逆动力学 OCP:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
正动力学:  决策变量 = (x, u=τ)
           动力学嵌入:  q̈ = ABA(q,q̇,τ)  (内部求逆 M)
           → 变量少,但每步要解 KKT/求 M⁻¹

逆动力学:  决策变量 = (x, u, q̈, λ)
           动力学作约束:  M q̈ + h = Sᵀτ + Jᵀλ
           → 变量多,但约束是"粗粒度"的,收敛更快
           → 大量等式约束 → 需高效处理
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

逆动力学路线的好处是 Mastalli 所说的"coarse optimization 与高收敛率"——优化问题的曲率更友好,迭代次数更少。代价是引入了**大量等式约束**(每个时间步一组动力学方程)。论文的核心贡献是用 **nullspace 参数化**(零空间分解)高效消去这些等式约束——这与 足式/90_WBC分层优化与TSID 里 WBC 的零空间投影是同一套数学工具(把约束方向投影掉,只在零空间里自由优化)。报告的计算量减少最高达 **47.3%**,并首次在 ANYmal 硬件上实现逆动力学 MPC,做出了 state-of-the-art 的动态攀爬。该算法已并入 Crocoddyl。

> 🧠 **深入理解**:为什么逆动力学能"粗粒度优化"?正动力学把"解 $\mathbf{M}^{-1}$"这一非线性操作埋进了每次 `calc()`,使得代价对 $\boldsymbol{\tau}$ 的依赖高度非线性;逆动力学把 $\ddot{\mathbf{q}}$ 暴露成独立变量后,代价对 $\ddot{\mathbf{q}}$ 往往是简单的二次型,$\mathbf{M}$ 只出现在线性的等式约束里。优化器面对的"非线性"被从代价转移到了约束,而约束的非线性更容易用 SQP/ALM 的局部线性化处理。这是"变量增多反而更快"的典型案例——维度不是唯一的复杂度指标,**问题的曲率结构**同样关键。

**进展二:GPU 并行 Primal-Dual iLQR。** *Primal-Dual iLQR for GPU-Accelerated Learning and Control in Legged Robots*(arXiv 2506.07823, 2025)。

§54.9.5 的 ParallelRiccati 把 backward pass 从 $O(N)$ 降到 $O(\log N)$,但它仍是 CPU 多核(几十核)的并行。这条新主线把 **parallel associative scan**(并行结合扫描)直接搬上 **GPU**,并且并行的对象不是纯 Riccati 而是**原始-对偶 KKT 系统**(primal-dual KKT,把约束的对偶变量一起纳入扫描)。

| 维度 | ParallelRiccati (RSS 2024) | Primal-Dual iLQR GPU (2025) |
|------|---------------------------|----------------------------|
| 并行硬件 | CPU 多核(~32) | GPU(数千线程)/ JAX |
| 扫描对象 | 价值函数 Riccati 算子 | 原始-对偶 KKT 系统 |
| 复杂度 | $O(\log N)\cdot n_x^3$ | $O(n\log N + m)$ |
| 报告加速 | 8 核 $4.4\times$,32 核 $8\times$ | 对比 acados/Crocoddyl,WB-MPC 最高 +60%,SRBD-MPC 最高 +700% |
| 额外能力 | — | 可在 GPU 上集中控制 $\leq 16$ 个四足($< 25$ ms);支持 MPC-in-the-loop 学习 |

> 💡 **洞察**:GPU 版本最深远的意义不在"更快",而在**把 MPC 求解器变成可微、可批量的计算图**(JAX 实现)。这让"MPC 在训练回路里"成为可能——可以在 GPU 上同时跑数千个环境,每个环境内嵌一个 MPC 求解器,端到端训练。这正是 足式/210_RL与MPC混合范式 讨论的 RL+MPC 融合方向的算力基础:当 MPC 本身可微且可批量,它就不再是 RL 之外的黑盒,而能作为一层"可微分控制先验"嵌入策略网络。

> ⚠️ **思维陷阱**:看到"对比 Crocoddyl 加速 700%"不要简单理解为"Crocoddyl 过时了"。这些加速是在**特定问题规模**(长 horizon、SRBD)和**特定硬件**(高端 GPU)上测得的。对于短 horizon、需要在嵌入式 CPU 上跑的场景,Crocoddyl 的 CPU 友好、零 GPU 依赖、成熟稳定仍是巨大优势(回顾 §54.9.5 陷阱:N 小时串行 Riccati 反而更快)。选型永远是"问题规模 $\times$ 硬件 $\times$ 工程成熟度"的综合权衡,不是单看 benchmark 数字。

### 开放研究问题 ⭐⭐⭐⭐

- **GPU ParallelRiccati**: parallel scan 移植到 GPU 已有初步答案(见上文 Primal-Dual iLQR, arXiv 2506.07823),但 SIMT 架构与树形归约的匹配仍有优化空间——如何处理 horizon 不能被线程块整除、如何隐藏合并步的同步延迟,仍是开放问题。
- **学习 + DDP**: 用神经网络预测初始 $V_\mathbf{x}, V_{\mathbf{xx}}$ 来热启动 DDP (MPC-Net 方向, 足式/210_RL与MPC混合范式)。
- **Contact-Implicit DDP**: 接触模式作为决策变量——极其困难的开放问题。
- **Differentiable DDP**: 让 DDP 本身可微,嵌入端到端学习管线。
- **DDP + Predictive Sampling 混合**：DDP 提供名义轨迹 + 采样做鲁棒化,兼顾精度与鲁棒性。

### 跨章综合练习 ⭐⭐⭐

**综合题：从 NLP 建模到 DDP 求解再到 WBC 执行的完整 MPC-WBC 链路**

本题需要综合 足式/60_QP_NLP建模（NLP 问题结构）、足式/70_腿足简化模型理论（SRBD 动力学）、足式/90_WBC分层优化与TSID（WBC）和本章（DDP 求解）四章知识。

1. **NLP 建模**（足式/60_QP_NLP建模 + 足式/70_腿足简化模型理论）：写出 Go2 四足机器人 trot 步态的 SRBD MPC 问题的完整 NLP 形式：(a) 状态变量 $\mathbf{x}_k$ 和控制变量 $\mathbf{u}_k$ 的定义及维度，(b) 动力学约束 $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)$ 的具体形式，(c) 摩擦锥路径约束。

2. **DDP 求解**（本章）：说明为什么这个 NLP 适合用 FDDP 而不是 Ipopt 求解（提示：Markov 结构、$O(N)$ 复杂度）。如果摩擦锥是硬约束而非 penalty，FDDP 是否仍然适用？为什么这时需要 ProxDDP？

3. **MPC → WBC 接口**（本章 + 足式/90_WBC分层优化与TSID）：DDP MPC 求解后输出最优轨迹 $\{\mathbf{x}_0^*, \mathbf{u}_0^*, \mathbf{x}_1^*, ...\}$ 和反馈增益 $\{\mathbf{K}_0, \mathbf{K}_1, ...\}$。WBC 需要的输入是什么？说明如何从 MPC 输出中提取 WBC 的 (a) 期望 CoM 加速度、(b) 期望接触力、(c) 反馈增益的使用方式（当实际状态偏离名义轨迹时）。

4. **整体时序分析**：画出一个控制周期（1 kHz）内 MPC 和 WBC 的时间预算分配。如果 MPC 以 30 Hz 运行（每 33 ms 更新一次），WBC 以 1 kHz 运行（每 1 ms 更新一次），那么在 MPC 两次更新之间，WBC 如何利用反馈增益 $\mathbf{K}_k$ 做实时补偿？

---

## 预计学习时间

| 内容 | 时间 |
|------|------|
| DDP 理论 (54.1-54.3) | 6-7 小时 |
| FDDP + Box-DDP 控制限位 (54.4-54.4B) | 3-4 小时 |
| Crocoddyl 架构 (54.5-54.7) | 4-5 小时 |
| OpenMP + 并行 (54.8) | 2-3 小时 |
| Aligator/ProxDDP/ParallelRiccati (54.9) | 4-5 小时 |
| 实战 + MPC + DDP/SQP 对比 (54.10-54.12) | 4-5 小时 |
| 练习 + 思考题 | 3-4 小时 |
| **合计** | **26-33 小时** |

---

## 本章关键术语

**DDP 核心**: Bellman Equation, Value Function $V_k$, Q-Function, Backward Pass, Forward Pass, Feedback Gain $\mathbf{K}_k$, Feedforward $\mathbf{k}_k$, Line Search (Armijo), Regularization (Levenberg-Marquardt)

**算法变体**: iLQR (Gauss-Newton DDP), FDDP (Feasibility-Driven DDP), Box-FDDP, box-DDP (Control-Limited DDP), ProxDDP (Proximal DDP), SQP

**Crocoddyl 架构**: ActionModelAbstract, ActionDataAbstract, ShootingProblem, DifferentialActionModel, IntegratedActionModel, ContactModelMultiple, CostModelResidual, ResidualModel, ActivationModel, SolverFDDP

**Aligator 架构**: StageModel, ProxDDP Solver, ParallelRiccati, Parallel Scan, Augmented Lagrangian

---

## 与其他章节的衔接

**向前承接**:
- 02_C++基础与进阶/10_Eigen CRTP → **足式/100_DDP家族与Crocoddyl Crocoddyl 为什么不用 CRTP(反例)**
- 02_C++基础与进阶/30_并发与实时 → **足式/100_DDP家族与Crocoddyl OpenMP 并行 + ParallelRiccati**
- 足式/30_Pinocchio深度精读 Pinocchio Model-Data → **足式/100_DDP家族与Crocoddyl ActionModel-ActionData**
- 足式/40_CppAD与代码生成 CppAD → **足式/100_DDP家族与Crocoddyl Crocoddyl 的 CodeGen 集成**
- 足式/60_QP_NLP建模 NLP/QP → **足式/100_DDP家族与Crocoddyl DDP 是特殊的 NLP 求解器**
- 足式/80_接触力学与约束优化 接触力学 → **足式/100_DDP家族与Crocoddyl ContactFwdDynamics KKT**
- 足式/90_WBC分层优化与TSID WBC → **足式/100_DDP家族与Crocoddyl DDP 输出轨迹给 WBC 跟踪**

**向后指向**:
- 足式/100_DDP家族与Crocoddyl DDP → **足式/110_OCS2完整栈与双线程MPC OCS2 用 SQP(设计对比)**
- 足式/100_DDP家族与Crocoddyl DDP 反馈增益 → **足式/110_OCS2完整栈与双线程MPC OCS2 的 Riccati 反馈**
- 足式/100_DDP家族与Crocoddyl MPC warm-start → **足式/120_步态管理与接触序列 步态管理驱动接触序列变化**
- 足式/100_DDP家族与Crocoddyl Crocoddyl 并行 → **足式/170_实时CPP工程 实时 C++ 的 PREEMPT_RT**
- 足式/100_DDP家族与Crocoddyl DDP → **足式/210_RL与MPC混合范式 RL+MPC 混合(MPC-Net 从 DDP 训练)**
