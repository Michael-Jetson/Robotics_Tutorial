## MPPI / 采样式 MPC 方向 C++ 进阶教学大纲(v0.1 · 八段式完整版)

> **定位**：本大纲是规划与控制 C++ 进阶体系的**采样式 MPC 增量扩展**，面向已掌握梯度式 MPC（OCS2/Crocoddyl/acados/SQP-RTI）和 RL 基础（PPO/SAC/IsaacLab）的机器人算法工程师，从梯度式 MPC 扩展到**采样式 MPC / 路径积分控制**范式。
>
> **与梯度式 MPC 的关系**：梯度式 MPC（iLQR/DDP/SQP）要求可微动力学与可微代价，擅长凸/近凸问题；采样式 MPC（MPPI/CEM/Predictive Sampling）**不需要任何梯度**，天然兼容不连续代价、黑箱仿真器和神经网络动力学模型，且可在 GPU 上大规模并行。两者互补，不替代。
>
> **章节编号**：共 10 章，约 18 周。Part 1（Ch1-Ch2，理论+核心）→ Part 2（Ch3-Ch4，变体+统一）→ Part 3（Ch5-Ch6，扩散+世界模型）→ Part 4（Ch7-Ch8，应用）→ Part 5（Ch9-Ch10，对比+实战）。
>
> **数据基础**：基于 30+ 篇顶会/顶刊论文的脉络梳理（ICRA/RSS/T-RO/JGCD/RA-L/CoRL/NeurIPS/ICLR/L4DC）、15+ 个开源项目的代码精读、6 轮深度调研的交叉验证。
>
> **前置假设**：学员已掌握——现代 C++17（含模板元编程）、Eigen 高级（Map/Block/固定大小）、CUDA 基础（kernel/shared memory/warp）、梯度式 MPC（OCS2 或 acados 用过至少一个）、RL 训练管线（PPO/SAC + IsaacLab/MuJoCo）、ROS2 基础。在此基础上切入采样式 MPC **不需要重新学 C++**，只需补"路径积分数学 + GPU 并行采样工程 + 采样分布设计"。
>
> **风格对齐**：沿用本文档统一的**八段式**章节结构。
>
> **核心实验室缩写**：Georgia Tech ACDS（Theodorou 组，MPPI 原创）、CMU LeCAR（Shi 组，CoVO-MPC/DIAL-MPC）、UCSD Hansen（TD-MPC 系列）、MPI Martius（iCEM）、Indiana VAIL（Mohamed 组，Log-MPPI/U-MPPI/GP-MPPI）、UW/NVIDIA Fox-Boots（SV-MPC/STORM）、DeepMind Tassa（MuJoCo MPC/Predictive Sampling）、Nagoya Honda（SVG-MPPI）、ROS Nav2 Macenski（Nav2 MPPI Controller）、UPenn KumarRobotics（微分平坦原创）。

---

## 整体路线图

```
梯度式 MPC 基础已完成(OCS2/Crocoddyl/acados)
RL 基础已完成(PPO/SAC/IsaacLab)
         │
         │  学生此时具备:
         │   - iLQR/DDP/SQP-RTI 的数学与代码
         │   - 可微动力学(Pinocchio/CasADi)
         │   - GPU 并行 RL 训练(IsaacLab)
         │   - ROS2 部署经验
         ▼
┌──────────────────────────────────────────────────────┐
│ Part 1：路径积分理论与 MPPI 核心（Ch1-Ch2，4 周）       │
│   Ch1 路径积分控制——从 Kappen 到信息论 MPPI            │
│   Ch2 MPPI 核心算法与 GPU 并行实现                     │
└──────────────────────────────────────────────────────┘
         ▼
┌──────────────────────────────────────────────────────┐
│ Part 2：MPPI 变体与 CEM 统一（Ch3-Ch4，3.5 周）        │
│   Ch3 MPPI 六大变体——鲁棒/平滑/多模态/不确定性         │
│   Ch4 CEM 家族与采样 MPC 统一视角                      │
└──────────────────────────────────────────────────────┘
         ▼
┌──────────────────────────────────────────────────────┐
│ Part 3：扩散启发 MPC 与学习世界模型（Ch5-Ch6，4 周）     │
│   Ch5 扩散启发采样 MPC——MPPI 作为单步去噪              │
│   Ch6 学习世界模型 + 采样规划——TD-MPC 系列              │
└──────────────────────────────────────────────────────┘
         ▼
┌──────────────────────────────────────────────────────┐
│ Part 4：应用领域专精（Ch7-Ch8，3.5 周）                 │
│   Ch7 腿足全身 MPPI 与接触丰富操作                     │
│   Ch8 导航、自驾与无人机采样 MPC                       │
└──────────────────────────────────────────────────────┘
         ▼
┌──────────────────────────────────────────────────────┐
│ Part 5：对比与实战（Ch9-Ch10，3 周）                    │
│   Ch9 MPPI vs 梯度式 MPC——何时选谁                    │
│   Ch10 Mini-MPPI 综合实战                              │
└──────────────────────────────────────────────────────┘
```

**全部投入**：Part 1-5 共 **~18 周**，全职等效约 4-5 个月；业余 15-20 小时/周约 8-12 个月。

**与梯度式 MPC 的对照**：

| 维度 | 采样式 MPC（本大纲） | 梯度式 MPC（已有基础） |
|------|-------------------|---------------------|
| 梯度需求 | **零梯度**——纯前向采样 | 需要 $\partial f/\partial x$、$\partial f/\partial u$ 和代价 Hessian |
| 代价函数 | 任意（含不连续 indicator、CNN cost-map） | 需平滑、可二次近似 |
| 动力学 | 黑箱/神经网络/仿真器皆可 | 需可微（CasADi/Pinocchio） |
| GPU 友好度 | **极高**——数千线程并行 rollout | 低——Riccati sweep 串行性强 |
| 收敛 | 全局/概率性，方差大 | 局部二次收敛，确定性 |
| 约束处理 | 软约束（barrier/投影）为主 | 硬约束（QP/NLP）为主 |
| 典型求解器 | MPPI-Generic/pytorch_mppi/MJPC | acados/OCS2/Crocoddyl |
| 实时性 | 依赖 GPU，~1-10 ms @ K=2048 | 依赖 CPU，~0.1-5 ms @ SQP-RTI |
| 代表应用 | AutoRally 激进驾驶、Nav2 导航、DIAL-MPC 腿足 | rpg_mpc 四旋翼、OCS2 腿足 MPC |

---

## 三大认知跨越（从梯度式 MPC 到采样式 MPC）

**跨越一：从"求解 KKT 系统"到"蒙特卡洛估计最优控制"。** 梯度式 MPC 的核心是构造并求解 KKT 系统（Riccati/内点法）；采样式 MPC 的核心是用大量随机 rollout 的加权平均**估计**最优控制。数学从**矩阵分解**转向**重要性采样与指数加权**。好消息是：RL 训练的直觉（reward shaping、温度调节、variance reduction）直接可迁移。

**跨越二：从"CPU 串行求解"到"GPU 大规模并行采样"。** 梯度式 MPC 的 Riccati sweep 天然串行；采样式 MPC 的 K 条 rollout 天然并行。这要求掌握 CUDA 编程模型（thread→warp→block）、shared memory 优化、以及 Eigen::Map 与 CUDA 的零拷贝互操作。IsaacLab 的 GPU 并行经验在此直接复用。

**跨越三：从"精确梯度指导搜索"到"采样分布设计决定一切"。** 梯度式 MPC 的收敛由局部曲率保证；采样式 MPC 的性能**完全取决于采样分布的质量**——高斯噪声的方差、有色噪声的频谱、协方差的自适应调度（CoVO-MPC）、甚至扩散式退火（DIAL-MPC）。理解"如何设计好的采样分布"是本大纲的**核心技能目标**。

---

## 梯度式 MPC / RL → 采样式 MPC 的技能迁移速查

| 已有技能 | 在采样式 MPC 中的对应 | 迁移难度 |
|---------|---------------------|---------|
| iLQR/DDP backward sweep | MPPI 无 backward sweep——**对照而非复用** | 概念对照 |
| CasADi 可微动力学 | MPPI 不需可微——MuJoCo/PyBullet 黑箱即可 | 零迁移（直接放弃） |
| acados SQP-RTI | MPPI warm-start shift 与 SQP-RTI 准备阶段**类似** | 低 |
| Pinocchio RNEA/ABA | 可作为 MPPI rollout 的动力学模型——但通常用仿真器替代 | 低 |
| PPO/SAC 策略梯度 | MPPI 权重公式与 REINFORCE **数学同构** | **零迁移** |
| IsaacLab GPU 并行环境 | MPPI CUDA 并行 rollout **思路完全一致** | **零迁移** |
| reward shaping | MPPI cost function 设计 = reward shaping | **零迁移** |
| 温度/熵系数调节 | MPPI 温度 $\lambda$ = 探索-利用权衡 | **零迁移** |
| Eigen 矩阵运算 | MPPI-Generic 的 Eigen::Map + CUDA 互操作 | 低 |
| ROS2 节点/消息 | Nav2 MPPI Controller 直接用 ROS2 | 零迁移 |
| 约束 QP（OSQP/ProxQP） | MPPI 用 barrier 函数/投影替代硬约束——**范式不同** | **高** |

---

## Part 1：路径积分理论与 MPPI 核心（Ch1-Ch2，4 周）

> **本部分定位**：Part 1 回答两个问题——**为什么 HJB 方程能通过一次指数变换变成线性的？**（Ch1 理论）和**如何在 GPU 上以 50 Hz 执行数千条 rollout 的加权平均？**（Ch2 工程）。这是整个大纲的数学地基和工程地基。

---

