# MuJoCo 生态与可微分仿真 · 规控方向交叉能力层教学大纲(v0.1)

> **定位**：本大纲是整个"机器人规划与控制 C++ 进阶"教学体系的**交叉能力层扩展**，面向已完成（或正在进行）腿足/机械臂/无人机任一方向学习的工程师，补充两个在现有大纲中**系统性缺失但对 RL 背景工程师至关重要**的主题：(1) MuJoCo 仿真器生态的完整教学；(2) 可微分仿真与可微分 MPC 的理论与实践。
>
> **为什么需要这份大纲**：现有大纲体系以 IsaacGym/IsaacLab 作为 RL 训练的默认仿真器、以 Pinocchio + CppAD 作为动力学微分的默认工具链。但 2024-2026 年的生态剧变使得两个判断需要修正：(a) **MuJoCo 已不是"CPU 研究玩具"**——MJX/Warp/Playground/mjlab 构成了完整的 GPU 并行 RL 训练栈，且 Newton 物理引擎使 MuJoCo Warp 成为 Isaac Lab 的可选后端；(b) **可微分仿真已从理论探索进入工程实践**——首个四足 sim2real 可微分仿真部署（Schwarke CoRL 2025）和工业级可微分 NMPC（acados + leap-c）标志着该领域的成熟。
>
> **章节编号**：S1-S5 共 5 章，约 12 周。可插入腿足大纲 足式/210_RL与MPC混合范式 之后（MPC+RL 混合之后）或作为独立模块学习。
>
> **数据基础**：基于 MuJoCo 3.7.0（2026-04-14）+ MJX 3.6.0 + MuJoCo Warp + Playground（RSS 2025 Outstanding Demo）+ mjlab（arXiv 2601.22074）+ Holosoma（Amazon FAR，arXiv 2512.01996）+ Newton 1.0（Linux Foundation）+ 30+ 个可微分仿真开源项目 + 50+ 篇 2022-2026 顶会/顶刊论文的源码级分析。
>
> **前置假设**：学员具备以下任一背景——(a) 完成腿足方向 足式/30_Pinocchio深度精读-70（掌握 Pinocchio/OCS2/RL 部署）；(b) 完成机械臂方向 M1-M15（掌握 Pinocchio/MoveIt2）；(c) 完成无人机方向 D1-D12（掌握 acados/RL 敏捷飞行）；(d) 具备独立的 RL + C++ 背景。核心前置是：**Eigen 高级 + ROS2 基础 + PPO/SAC 训练经验 + 至少一个仿真器的使用经验**。
>
> **风格对齐**：每章按 `科研发展脉络 / 教学目标 / 前置依赖 / 核心知识点 / 前沿工作与开放问题 / 项目精读清单 / 实战练习(A型+B型+思考题) / 预计学习时间` 的**八段式**结构，与无人机方向大纲完全对齐。
>
> **核心实验室缩写**：DeepMind Robotics（MuJoCo/MJX/Playground 核心维护）、UC Berkeley BAIR（mjlab/Pieter Abbeel/Kevin Zakka）、Amazon FAR（Holosoma/Guanya Shi）、Freiburg Diehl 组（acados/leap-c）、UZH RPG（Swift/可微分竞速）、CMU CDFG（SHAC/DiffRL）、Stanford MSL（Dojo/Drake）、ETH RSL（OCS2/legged_control）、UCSD Hao Su 组（TD-MPC2）、NVLabs DiffRL（SHAC/DFlex）。

---

## 整体路线图

```
任一方向完成后（腿足 足式/30_Pinocchio深度精读-70 / 机械臂 M1-M15 / 无人机 D1-D12）
         │
         │  学生此时具备：
         │   - Pinocchio 动力学 或 微分平坦 或 MoveIt2
         │   - MPC（OCS2/acados/Crocoddyl）或 RL 部署（LibTorch/ONNX）
         │   - IsaacGym/IsaacLab 或 Gazebo 仿真经验
         │   - 但 MuJoCo 生态和可微分仿真是系统性盲区
         ▼
┌──────────────────────────────────────────────────────┐
│ Part S-I：MuJoCo 仿真器核心（第 1-4 周，4 周）         │
│   S1 MuJoCo 核心引擎与 MJCF 建模                      │
│   S2 MuJoCo 交互式控制——MJPC / mjctrl / mink          │
└──────────────────────────────────────────────────────┘
         ▼
┌──────────────────────────────────────────────────────┐
│ Part S-II：MuJoCo GPU 生态与多仿真器融合（第 5-7 周）  │
│   S3 MJX / Warp / Playground / mjlab / Holosoma       │
└──────────────────────────────────────────────────────┘
         ▼
┌──────────────────────────────────────────────────────┐
│ Part S-III：可微分仿真与可微分 MPC（第 8-12 周）       │
│   S4 可微分仿真理论与框架                              │
│   S5 可微分 MPC 与学习-控制融合                        │
└──────────────────────────────────────────────────────┘
```

**全部投入**：5 章 ~12 周，全职等效约 3 个月；业余 15-20 小时/周约 6 个月。

**与现有大纲的对照**：

| 维度 | 本大纲（MuJoCo + 可微分仿真） | 腿足方向（足式/30_Pinocchio深度精读-70） | 无人机方向（D1-D12） |
|------|---------------------------|-------------------|-------------------|
| 核心仿真器 | **MuJoCo**（C API + MJX + Warp） | IsaacGym/IsaacLab | Flightmare/OmniDrones/gym-pybullet-drones |
| 动力学微分 | MJX 可微 + Drake AutoDiffXd + Brax | Pinocchio 解析导数 + CppADCodeGen | 微分平坦解析映射 |
| GPU 并行 | MJX(JAX) / Warp(CUDA) / Playground | IsaacGym tensor API | Aerial Gym / OmniDrones |
| MPC 工具 | MJPC(iLQG/Predictive Sampling) + leap-c | OCS2(SQP+HPIPM) + Crocoddyl(DDP) | acados(SQP-RTI) + rpg_mpc(ACADO) |
| RL 训练框架 | Playground + mjlab + Holosoma(rsl_rl) | IsaacLab + rsl_rl | OmniDrones + stable-baselines3 |
| 模型格式 | **MJCF**（原生）+ URDF 导入 | URDF（Pinocchio/ros2_control） | URDF/SDF（Gazebo/PX4） |
| 接触模型 | 软接触（Gauss 原理凸优化）+ 椭圆摩擦锥 | 硬接触（互补约束/摩擦锥 QP） | 无接触（空气动力学） |

---

## 三大认知跨越（从 IsaacGym/Pinocchio 到 MuJoCo/可微分仿真）

**跨越一：从"黑盒仿真器"到"可检查可逆的物理引擎"。** IsaacGym/PhysX 的核心物理计算对用户不透明——你只能调用 `gym.simulate()` 然后观察结果。MuJoCo 的设计哲学完全不同：**`mj_forward` 计算正动力学（状态→加速度），`mj_inverse` 计算逆动力学（加速度→力），两者在有接触时都是 well-defined 的**。这意味着你可以在仿真中的任意时刻精确检查"为什么机器人这样动"——哪些力来自重力、哪些来自接触、哪些来自执行器。对于算法工程师，这是调试控制器和理解物理的根本性优势。

**跨越二：从"只能前向传播"到"梯度流经物理"。** 传统 RL 训练的核心循环是：仿真器前向步进（不可微）→ 收集 reward → PPO/SAC 零阶策略梯度。可微分仿真打破这个限制：**梯度可以从 loss 函数反向流经物理步进、穿过接触和约束**，直达策略参数或系统参数。但这不是免费午餐——接触的非光滑性会让梯度有偏差（Suh ICML 2022 Outstanding Paper）。理解"何时可微分仿真比 PPO 好、何时不好"是本大纲的核心教学目标之一。

**跨越三：从"单一仿真器锁定"到"多仿真器协同"。** 2025-2026 的行业趋势是**多仿真器训练+验证**——ASAP/HumanoidVerse 在 IsaacGym 训练、MuJoCo 验证；Holosoma 同时支持 IsaacGym/IsaacSim/MuJoCo Warp/MuJoCo CPU；Isaac Lab 3.0 的 Newton 后端实质上运行的是 MuJoCo Warp。**"一套代码、多个仿真器"的 sim2sim 验证已成为工业默认实践**——锁定在单一仿真器上是风险。

---

## IsaacGym/Lab → MuJoCo 的技能迁移速查

| IsaacGym/Lab 技能 | 在 MuJoCo 生态中的对应 | 迁移难度 |
|------------------|---------------------|---------|
| PhysX GPU 并行仿真 | MJX(JAX) / MuJoCo Warp(CUDA) / Playground | 低——API 不同但概念一致 |
| IsaacLab Manager API（ManagerBasedRLEnv） | **mjlab**——几乎 1:1 API 移植 | **极低**——mjlab 就是为此而生 |
| rsl_rl PPO 训练 | Playground / mjlab / Holosoma 均集成 rsl_rl | **零迁移** |
| URDF 机器人模型 | MuJoCo Menagerie 提供 55+ MJCF 模型；URDF 可直接导入 | 低 |
| Isaac Lab terrain curriculum | Playground locomotion terrain；mjlab 自带 terrain | 低——实现不同但概念一致 |
| GPU tensor 接口 | MJX 返回 `jax.Array`；Warp 返回 `wp.array` | 中——需学 JAX 或 Warp 张量操作 |
| domain randomization（runtime API） | MuJoCo `mj_setConst` + Rucker 伪惯量参数化 | 中——MuJoCo 的 DR 更底层但更灵活 |
| 碰撞检测（PhysX GJK/EPA） | MuJoCo nativeccd（3.3.5+ 默认）+ SDF 碰撞原语 | 中——接触模型理念不同 |

| Pinocchio 技能 | 在 MuJoCo 中的对应 | 迁移难度 |
|---------------|-------------------|---------|
| `pinocchio::Model` / `pinocchio::Data` | `mjModel` / `mjData`——**设计完全同构** | **极低** |
| `pinocchio::rnea()` 逆动力学 | `mj_inverse()`——含接触力的逆动力学 | 低——MuJoCo 的更强（含接触） |
| `pinocchio::aba()` 正动力学 | `mj_forward()`——含接触求解 | 低 |
| Pinocchio 解析导数 | MJX `jax.grad(mj_step)`——AD 而非解析 | **高——范式不同** |
| CppADCodeGen 预编译 | MJX JIT(XLA)/ Warp JIT(CUDA) | **高——JAX/Warp 是新技能** |
| URDF 加载 | MuJoCo 可直接 `mj_loadXML("robot.urdf")` | 零迁移 |

---

