# MuJoCo 生态与可微分仿真 · 规控方向交叉能力层教学大纲(v0.1)

> **定位**：本大纲是整个"机器人规划与控制 C++ 进阶"教学体系的**交叉能力层扩展**，面向已完成（或正在进行）腿足/机械臂/无人机任一方向学习的工程师，补充两个在现有大纲中**系统性缺失但对 RL 背景工程师至关重要**的主题：(1) MuJoCo 仿真器生态的完整教学；(2) 可微分仿真与可微分 MPC 的理论与实践。
>
> **为什么需要这份大纲**：现有大纲体系以 IsaacGym/IsaacLab 作为 RL 训练的默认仿真器、以 Pinocchio + CppAD 作为动力学微分的默认工具链。但 2024-2026 年的生态剧变使得两个判断需要修正：(a) **MuJoCo 已不是"CPU 研究玩具"**——MJX/Warp/Playground/mjlab 构成了完整的 GPU 并行 RL 训练栈，且 Newton 物理引擎使 MuJoCo Warp 成为 Isaac Lab 的可选后端；(b) **可微分仿真已从理论探索进入工程实践**——首个四足 sim2real 可微分仿真部署（Schwarke CoRL 2025）和工业级可微分 NMPC（acados + leap-c）标志着该领域的成熟。
>
> **章节编号**：S1-S5 共 5 章，约 12 周。可插入腿足大纲 Ch65 之后（MPC+RL 混合之后）或作为独立模块学习。
>
> **数据基础**：基于 MuJoCo 3.7.0（2026-04-14）+ MJX 3.6.0 + MuJoCo Warp + Playground（RSS 2025 Outstanding Demo）+ mjlab（arXiv 2601.22074）+ Holosoma（Amazon FAR，arXiv 2512.01996）+ Newton 1.0（Linux Foundation）+ 30+ 个可微分仿真开源项目 + 50+ 篇 2022-2026 顶会/顶刊论文的源码级分析。
>
> **前置假设**：学员具备以下任一背景——(a) 完成腿足方向 Ch47-70（掌握 Pinocchio/OCS2/RL 部署）；(b) 完成机械臂方向 M1-M15（掌握 Pinocchio/MoveIt2）；(c) 完成无人机方向 D1-D12（掌握 acados/RL 敏捷飞行）；(d) 具备独立的 RL + C++ 背景。核心前置是：**Eigen 高级 + ROS2 基础 + PPO/SAC 训练经验 + 至少一个仿真器的使用经验**。
>
> **风格对齐**：每章按 `科研发展脉络 / 教学目标 / 前置依赖 / 核心知识点 / 前沿工作与开放问题 / 项目精读清单 / 实战练习(A型+B型+思考题) / 预计学习时间` 的**八段式**结构，与无人机方向大纲完全对齐。
>
> **核心实验室缩写**：DeepMind Robotics（MuJoCo/MJX/Playground 核心维护）、UC Berkeley BAIR（mjlab/Pieter Abbeel/Kevin Zakka）、Amazon FAR（Holosoma/Guanya Shi）、Freiburg Diehl 组（acados/leap-c）、UZH RPG（Swift/可微分竞速）、CMU CDFG（SHAC/DiffRL）、Stanford MSL（Dojo/Drake）、ETH RSL（OCS2/legged_control）、UCSD Hao Su 组（TD-MPC2）、NVLabs DiffRL（SHAC/DFlex）。

---

## 整体路线图

```
任一方向完成后（腿足 Ch47-70 / 机械臂 M1-M15 / 无人机 D1-D12）
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

| 维度 | 本大纲（MuJoCo + 可微分仿真） | 腿足方向（Ch47-70） | 无人机方向（D1-D12） |
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

# Part S-I：MuJoCo 仿真器核心（第 1-4 周）

> **本部分定位**：为算法工程师建立 MuJoCo 的完整心智模型——不是"学一个新软件的按钮在哪里"，而是理解**MuJoCo 的物理建模哲学为什么与 PhysX/Bullet 根本不同**，以及这种差异如何影响你做 RL 训练、MPC 控制和 sim-to-real 的每一个决策。

---

## S1 MuJoCo 核心引擎与 MJCF 建模（第 1-2 周，2 周）

**性质**：✅ **全方向共享**——MuJoCo 是 RL locomotion 研究的事实标准仿真器（DeepMind/OpenAI/Berkeley/Meta 的几乎所有工作），同时也是操作（robosuite/robomimic）和生物力学的标准。所有方向的工程师都应掌握。

### 科研发展脉络

| 年份 | 事件 | 意义 |
|------|------|------|
| 2012 | Todorov, Erez, Tassa, "MuJoCo: A physics engine for model-based control", IROS | **奠基作**：Gauss 原理 + 凸优化接触 + 软约束；对标 ODE/Bullet |
| 2014 | Tassa, Todorov, "Control-limited DDP", ICRA | MJPC 的理论基础——iLQG 变体，约束处理通过 clamp |
| 2018 | Tassa 等, "DeepMind Control Suite: A Set of Safe RL Benchmarks" | **dm_control**：28 个标准连续控制任务；MuJoCo 成为 RL 基准 |
| 2021 | DeepMind 收购 MuJoCo 并开源（Apache-2.0） | **转折点**：从付费闭源 → 免费开源；社区爆发 |
| 2023 | MuJoCo 3.0：MJX(JAX GPU 后端) | GPU 可微分仿真能力；千级并行 |
| 2024 | MuJoCo 3.1-3.2：SDF 碰撞原语 + 多线程 + 肌肉执行器 | 物理保真度大幅提升 |
| 2025 | MuJoCo 3.3-3.5：nativeccd 默认 + MuJoCo Warp + Newton solver | **GPU 性能跃升**；Newton 物理引擎集成 |
| 2025 | MuJoCo Playground（RSS 2025 Outstanding Demo Award） | **端到端 GPU RL 训练→sim2real 验证平台** |
| 2025 | mjlab（UC Berkeley，arXiv 2601.22074） | Isaac Lab 的 Manager API 移植到 MuJoCo Warp |
| 2026 | MuJoCo 3.7.0（2026-04-14，当前最新） | Newton solver 集成到核心；flex 可变形体移入引擎 |

**关键人物**：Emanuel Todorov（MuJoCo 之父，UW→DeepMind）→ Yuval Tassa（dm_control/MJPC/MuJoCo Playground 核心）→ Kevin Zakka（Menagerie/mjctrl/mink/mjlab）→ Tom Erez（MuJoCo 共同创始人）。

### 教学目标

1. 理解 MuJoCo 的**物理建模哲学**：为什么选择"Gauss 原理 + 凸优化 + 软约束"而非"LCP + 硬接触"——这决定了接触行为、梯度质量和逆动力学的可行性
2. 掌握 **mjModel / mjData 双结构设计**，并能与 Pinocchio Model/Data 做逐字段映射
3. 理解 **`mj_step` / `mj_forward` / `mj_inverse` 三大核心函数**的调用链和物理含义
4. 掌握 **MJCF 建模语言**的核心概念——相对于 URDF 的表达力差异、接触参数（solref/solimp）、执行器模型
5. 理解 MuJoCo 的**接触模型数学**——椭圆摩擦锥、soft contact 的弹性-阻尼参数、与 PhysX/Bullet 的本质差异
6. 能用 Python 和 C API 完成基本的仿真循环、传感器读取、执行器控制

### 前置依赖

- v8 Ch11（Eigen 基础）——mjData 中所有矩阵/向量可通过 NumPy view 访问
- v8 Ch30-31（ROS2 基础）——MuJoCo 不依赖 ROS 但 ros2_control 有 MuJoCo 后端
- 任一方向的仿真经验（Gazebo/IsaacGym/Flightmare）——对比性理解

### 核心知识点

#### S1.1 MuJoCo 的物理建模哲学——为什么它与 PhysX/Bullet 根本不同

**核心区别**：PhysX/Bullet 把接触建模为**硬约束（LCP/NCP）**——碰撞瞬间速度跳变，摩擦力通过互补条件求解。MuJoCo 把接触建模为**软约束（凸优化）**——penetration 被允许但受弹性-阻尼力惩罚，摩擦通过椭圆锥约束软化。

**Todorov 2012 的核心论点**：

> "硬接触在数学上导致 NP-hard 问题（LCP 一般性是 P-complete），且在多接触时解不唯一。我们的方法把物理求解统一为凸优化，保证唯一解和多项式时间。"

这个设计选择的**工程后果**：

| 维度 | MuJoCo（软约束/凸优化） | PhysX/Bullet（硬约束/LCP） |
|------|----------------------|-------------------------|
| 解的唯一性 | ✅ 凸优化保证唯一解 | ❌ LCP 可能多解或无解 |
| 逆动力学 | ✅ `mj_inverse` 在有接触时 well-defined | ❌ 硬接触使逆动力学 ill-posed |
| 梯度质量 | 良好（软约束光滑→MJX 可微） | 差（硬接触跳变→梯度不连续） |
| penetration | 允许微小穿透（可调） | 理论上零穿透（实际有数值漂移） |
| 稳定性 | 大步长（5 ms）仍稳定 | 需小步长或多次子步 |
| 物理准确度 | 柔性接触更自然（橡胶足/软体） | 刚性接触更准确（钢铁碰撞） |

**对 RL 工程师的影响**：
- MuJoCo 的软接触让 reward landscape 更平滑——PPO 收敛更快
- PhysX 的硬接触可能导致 reward 在接触边界震荡——需要更多 reward shaping
- MuJoCo 的 `mj_inverse` 可以在**仿真过程中精确检查"这个力是从哪来的"**——IsaacGym 做不到

#### S1.2 mjModel / mjData 双结构设计——与 Pinocchio 的逐字段映射

MuJoCo 的运行时数据结构是两个大 C 结构体：**`mjModel`（只读，编译自 MJCF/URDF）**和 **`mjData`（可变，仿真状态+中间计算）**。这与 Pinocchio 的 `Model`（只读拓扑+惯量）/ `Data`（可变缓冲）**设计完全同构**。

**mjModel 关键字段**（对应 Pinocchio Model）：

| mjModel 字段 | 含义 | Pinocchio Model 对应 |
|-------------|------|---------------------|
| `nq` | 广义坐标维度 | `model.nq` |
| `nv` | 广义速度维度 | `model.nv` |
| `nbody` | 刚体数量 | `model.nbodies` |
| `njnt` | 关节数量 | `model.njoints` |
| `body_mass[i]` | 第 i 个刚体质量 | `model.inertias[i].mass()` |
| `body_inertia[i]` | 惯量对角元（体坐标系） | `model.inertias[i].inertia()` |
| `jnt_type[i]` | 关节类型（hinge/slide/ball/free） | `model.joints[i].shortname()` |
| `opt.gravity` | 重力向量 | `model.gravity` |
| `opt.timestep` | 积分步长 | —（Pinocchio 不做积分） |
| `opt.solver` | 求解器类型（PGS/CG/Newton） | —（Pinocchio 无接触） |

**mjData 关键字段**（对应 Pinocchio Data）：

| mjData 字段 | 含义 | Pinocchio Data 对应 |
|------------|------|---------------------|
| `qpos[nq]` | 广义坐标（位置+四元数） | `data.q` |
| `qvel[nv]` | 广义速度 | `data.v` |
| `qacc[nv]` | 广义加速度（`mj_forward` 后填充） | `data.a`（ABA 后填充） |
| `ctrl[nu]` | 执行器控制输入 | —（Pinocchio 直接用 τ） |
| `qfrc_applied[nv]` | 外加广义力 | `data.tau`（RNEA 输入） |
| `xpos[nbody×3]` | 刚体世界位置 | `data.oMi[i].translation()` |
| `xquat[nbody×4]` | 刚体世界四元数 | `data.oMi[i].rotation()` |
| `contact[ncon]` | 活跃接触列表 | —（Pinocchio 无接触） |
| `qfrc_inverse[nv]` | 逆动力学力（`mj_inverse` 后填充） | `data.tau`（RNEA 输出） |
| `sensordata[nsensordata]` | 传感器读数 | —（Pinocchio 无传感器） |

**关键设计差异**：
1. **MuJoCo 的 mjModel 包含接触参数、执行器模型、传感器定义**——Pinocchio 的 Model 只有运动学/惯量
2. **MuJoCo 的 mjData 包含接触列表和接触力**——Pinocchio 不处理接触
3. **两者共享的设计哲学**：Model 只读→线程安全共享；Data 可变→每线程一份

**Python 中的零拷贝访问**：
```python
import mujoco
m = mujoco.MjModel.from_xml_path("humanoid.xml")
d = mujoco.MjData(m)
mujoco.mj_step(m, d)

# NumPy view——零拷贝，修改会直接反映到 mjData
qpos = d.qpos  # shape (nq,)，不是 copy！
qpos[0] += 0.01  # 直接修改了 mjData 内部的 C 数组

# Named access——更可读
d.joint('left_knee').qpos  # 单关节位置
d.sensor('imu_acc').data   # 传感器读数
```

#### S1.3 三大核心函数——mj_step / mj_forward / mj_inverse

```
mj_step(m, d)
  ├── mj_step1(m, d)           ← 位置相关计算 + 碰撞检测
  │   ├── mj_fwdPosition(m, d)   ← 正运动学、质心、惯量矩阵
  │   ├── mj_sensor(m, d)        ← 与位置相关的传感器
  │   └── mj_collision(m, d)     ← 宽相位+窄相位碰撞检测
  ├── mj_step2(m, d)           ← 速度相关计算 + 积分
  │   ├── mj_fwdVelocity(m, d)   ← 科氏力、被动力
  │   ├── mj_fwdActuation(m, d)  ← 执行器力
  │   ├── mj_fwdConstraint(m, d) ← **接触约束求解**（核心！）
  │   └── mj_Euler(m, d)         ← 数值积分（或 mj_RungeKutta）
  └── d.time += m.opt.timestep

mj_forward(m, d)
  └── 与 mj_step 相同，但 **不做积分**（不更新 qpos/qvel/time）
  └── 用途：给定 (qpos, qvel, ctrl) 计算 qacc

mj_inverse(m, d)
  └── 给定 (qpos, qvel, qacc) 计算 qfrc_inverse
  └── **即使有接触也是 well-defined**——因为软约束使力可反解
  └── 用途：验证控制器（"实现这个加速度需要多大力？"）
```

**`mj_inverse` 的独特价值**——IsaacGym/PhysX 做不到：

```python
# 在仿真中任意时刻检查"为什么机器人这样动"
mujoco.mj_forward(m, d)     # 计算当前加速度
mujoco.mj_inverse(m, d)     # 反解出所需力

# 力的分解
gravity_force = d.qfrc_bias     # 重力+科氏力
contact_force = d.qfrc_constraint  # 接触力
actuator_force = d.qfrc_actuator   # 执行器力
# 检验：qfrc_inverse ≈ gravity_force + contact_force + actuator_force
```

**积分器选择**：

| 积分器 | 稳定性 | 精度 | 速度 | 推荐场景 |
|--------|--------|------|------|---------|
| `mjINT_EULER` | 低 | O(h) | 最快 | 向后兼容 |
| `mjINT_IMPLICIT` | 高 | O(h) | 中 | 刚性系统 |
| `mjINT_IMPLICITFAST` | **高** | O(h) | **快** | **官方推荐默认** |
| `mjINT_RK4` | 中 | O(h⁴) | 慢 | 无接触的精确积分 |

**`implicitfast`（3.2.3+ 引入）是当前官方推荐**——跳过科氏力 Jacobian 计算，在稳定性和速度间取得最佳平衡。

#### S1.4 MJCF 建模语言——相对于 URDF 的表达力差异

**核心差异总结**：

| 维度 | URDF | MJCF |
|------|------|------|
| 设计哲学 | 描述运动学树 | **描述完整物理仿真** |
| 接触参数 | ❌ 不支持 | ✅ solref/solimp/friction/condim |
| 执行器模型 | ❌ 不支持 | ✅ motor/position/velocity/cylinder/muscle |
| 传感器 | ❌ 不支持 | ✅ 30+ 种传感器类型 |
| 肌腱 | ❌ 不支持 | ✅ 固定/空间肌腱 |
| 等式约束 | ❌ 不支持 | ✅ connect/weld/joint/tendon |
| 求解器配置 | ❌ 不支持 | ✅ solver/iterations/tolerance |
| 一个 body 多关节 | ❌ 需 dummy body | ✅ 直接支持 |
| 默认值继承 | ❌ | ✅ `<default>` 机制 |
| include 机制 | ❌ | ✅ `<include file="..."/>` |

**MJCF 的执行器模型——URDF 完全没有的概念**：

```xml
<!-- 位置伺服（PD 控制器内置在执行器中） -->
<actuator>
  <position name="knee_servo" joint="knee" kp="100" kv="10"
            ctrlrange="-1.57 1.57" forcerange="-50 50"/>
</actuator>

<!-- 力矩执行器（直接施加扭矩） -->
<actuator>
  <motor name="knee_motor" joint="knee" gear="1"
         ctrlrange="-50 50"/>
</actuator>
```

**IsaacGym/legged_gym 的 PD 驱动器等价于 MuJoCo 的 `<position>` 执行器**——但 MuJoCo 把 PD 参数（kp/kv）放在 MJCF 中而非代码中，让模型定义更完整。

**接触参数 solref/solimp**：

```xml
<geom name="foot" type="box" size="0.05 0.03 0.01"
      solref="0.02 1.0"    <!-- [timeconst, dampratio] -->
      solimp="0.9 0.95 0.001 0.5 2"  <!-- [dmin, dmax, width, midpoint, power] -->
      friction="0.8 0.005 0.0001"  <!-- [sliding, torsional, rolling] -->
      condim="4"/>          <!-- 接触维度：1=法向 3=滑动+法向 4=+扭转 6=+滚动 -->
```

- `solref[0]`（time constant）：接触弹簧的时间常数，值越大接触越软
- `solref[1]`（damping ratio）：临界阻尼比，1.0 = 临界阻尼
- **这两个参数对 sim-to-real 影响巨大**——橡胶足需要软接触（solref=[0.02, 1.0]），金属碰撞需要硬接触（solref=[0.002, 1.0]）

**URDF→MJCF 转换实践**：

```python
# 方法 1：MuJoCo 原生加载 URDF
import mujoco
model = mujoco.MjModel.from_xml_path("robot.urdf")
# 注意：URDF 中无法表达的信息（接触/执行器/传感器）将使用 MuJoCo 默认值

# 方法 2：URDF 中嵌入 MuJoCo 扩展块
# 在 URDF 的 <robot> 标签内添加：
# <mujoco>
#   <compiler meshdir="meshes/" balanceinertia="true"/>
# </mujoco>

# 方法 3：加载后导出完整 MJCF
mujoco.mj_saveLastXML("robot_full.xml", model)
# 手动编辑 robot_full.xml 添加执行器/传感器/接触参数

# 方法 4：使用 MuJoCo Menagerie 的高质量 MJCF 模型
# git clone https://github.com/google-deepmind/mujoco_menagerie
# 直接使用 unitree_go2/scene.xml 等精调模型
```

#### S1.5 MuJoCo Menagerie——标准化机器人模型库

**GitHub**: `google-deepmind/mujoco_menagerie`, ~3.2k★, Apache-2.0。

Menagerie 提供 55-60 个工业级 MJCF 模型，每个模型都经过严格验证（惯量参数与 CAD 一致、执行器参数标定、接触参数调优）。**这是 MuJoCo 生态对 URDF 生态的关键优势**——URDF 模型通常只有运动学正确但接触/执行器未调。

**腿足/人形方向可用模型**：

| 机器人 | 目录 | DOF | 用途 |
|--------|------|-----|------|
| Unitree Go1 | `unitree_go1/` | 12 | 四足 locomotion |
| Unitree Go2 | `unitree_go2/` | 12 | 四足 locomotion（更新） |
| Unitree G1 | `unitree_g1/` | 29 | 人形 whole-body |
| Unitree H1 | `unitree_h1/` | 19 | 人形 locomotion |
| ANYmal B | `anybotics_anymal_b/` | 12 | 四足（ETH 标准） |
| ANYmal C | `anybotics_anymal_c/` | 12 | 四足（ETH 最新） |
| Google Barkour vB | `google_barkour_vb/` | 12 | 四足敏捷（DeepMind 标准） |
| Booster T1 | `booster_t1/` | 35 | 人形（Playground 验证） |
| Robotis OP3 | `robotis_op3/` | 20 | 小型人形（DeepMind Soccer） |
| Franka FR3 | `franka_fr3/` | 7 | 机械臂 |
| ALOHA 2 | `aloha_2/` | 14 | 双臂操作 |

**教学价值**：Menagerie 的每个模型目录都包含 `scene.xml`（完整场景）和 `<robot>.xml`（裸模型）。`scene.xml` 是一个**最小可运行的仿真场景**——含地面、光源、相机、执行器、传感器，可直接 `python -m mujoco.viewer --mjcf=scene.xml` 查看和交互。

#### S1.6 Python bindings 的 2026 推荐

**官方 `mujoco` 包是唯一推荐**（`pip install mujoco`，当前 3.7.0）。历史上存在三套 Python 绑定的混乱：

| 包名 | 维护者 | 状态 | 使用建议 |
|------|--------|------|---------|
| `mujoco`（官方） | DeepMind | ✅ 活跃 | **唯一推荐** |
| `mujoco-py` | OpenAI | ❌ 已弃用 | 仅用于重现 2021 前的旧工作 |
| `dm_control` | DeepMind | ✅ 活跃 | PyMJCF + Control Suite 仍有价值 |

**三种 viewer 模式**：
```python
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("humanoid.xml")
d = mujoco.MjData(m)

# 模式 1：阻塞式（最简单，适合快速查看）
mujoco.viewer.launch(m, d)

# 模式 2：非阻塞式（适合自定义控制循环）
with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        d.ctrl[:] = policy(d.qpos, d.qvel)  # 你的控制器
        mujoco.mj_step(m, d)
        viewer.sync()

# 模式 3：mjpython 启动器（macOS 需要）
# mjpython my_script.py
```

### 前沿工作与开放问题

- **Newton 求解器（MuJoCo 3.6+）**：新增 `mjSOL_NEWTON` 替代 PGS，在 GPU 上显著加速（人形场景 3× 提升）——特别适合 MuJoCo Warp 的大规模并行
- **MjSpec 程序化模型编辑（3.2+）**：替代 dm_control 的 PyMJCF，允许运行时修改模型结构（添加/删除关节/刚体）——域随机化的新范式
- **SDF 碰撞原语（3.1+）**：任意形状的隐式几何描述——不要求凸分解
- **开放问题**：MuJoCo 的软接触在极硬表面（金属碰金属）时不够物理准确；MJCF 与 URDF 的互操作仍需手动调整接触参数；MuJoCo 无原生的光线追踪渲染（需外接 Mujoco Madrona/Blender）

### 项目精读清单

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **MuJoCo** | `src/engine/engine_forward.c` | `mj_forward` 的完整调用链 | 源码阅读（进阶） |
| **MuJoCo** | `src/engine/engine_inverse.c` | `mj_inverse` 的实现——为什么有接触时也可逆 | 源码阅读（进阶） |
| **MuJoCo** | `python/mujoco/tutorial.ipynb`（官方 Colab） | 基本 API 使用、渲染、仿真循环 | 教程 |
| **Menagerie** | `unitree_go2/go2.xml` | MJCF 的完整工业级示例——执行器/传感器/接触参数如何配置 | 模型阅读 |
| **Menagerie** | `unitree_g1/g1.xml` | 29-DOF 人形——复杂关节树的 MJCF 表示 | 模型阅读 |
| **dm_control** | `dm_control/suite/humanoid.py` | 如何用 Python 封装 MuJoCo 为 RL 环境 | 代码阅读 |

### 实战练习

- **[A 型·基础仿真]** 安装 `mujoco>=3.5`，加载 Menagerie 的 `unitree_go2/scene.xml`，运行 `mujoco.viewer.launch(m, d)` 交互式查看。用 Tab 键切换透视/正交，F1 查看帮助。**拖拽机器人感受软接触**——对比 Gazebo 中同一机器人的触感差异
- **[A 型·正逆动力学验证]** 写一个脚本：(1) 设置 Go2 的 qpos/qvel/ctrl；(2) 调用 `mj_forward` 得到 qacc；(3) 调用 `mj_inverse` 得到 qfrc_inverse；(4) 验证 `qfrc_inverse ≈ qfrc_bias + qfrc_constraint + qfrc_actuator`（力平衡）。这个练习在 IsaacGym 中**无法完成**
- **[A 型·URDF→MJCF 转换]** 取一个你熟悉的 URDF 机器人模型（如 Panda、UR5），用 `mj_saveLastXML` 导出 MJCF。手动添加执行器（position servo）和传感器（IMU、足端力），在 viewer 中验证
- **[A 型·接触参数调优]** 修改 Go2 的 `solref` 参数：(1) 极硬 [0.001, 1.0]；(2) 默认 [0.02, 1.0]；(3) 极软 [0.1, 1.0]。在 viewer 中让机器人自由落体到地面，观察弹跳行为差异。**理解 solref 对 sim-to-real 的影响**
- **[B 型·mjModel 精读]** 精读 MuJoCo 文档 "Simulation" 章节中的 `mjModel` 字段定义。画出 `mjModel` 和 `pinocchio::Model` 的字段映射表。标注：哪些是 MuJoCo 独有的（接触/执行器/传感器）？
- **[B 型·engine_forward.c 精读]** 精读 `engine_forward.c` 中 `mj_forward` 的实现（约 200 行）。画出调用图：`mj_fwdPosition` → `mj_collision` → `mj_fwdVelocity` → `mj_fwdActuation` → `mj_fwdConstraint`。标注：接触求解在哪一步发生？
- **[思考题]** MuJoCo 的 `mj_inverse` 在有接触时也是 well-defined 的，但 Bullet/PhysX 的逆动力学在有接触时是 ill-posed 的。为什么？提示：软接触使接触力是状态的光滑函数；硬接触使接触力是集合值映射（set-valued）
- **[思考题]** Pinocchio 的 RNEA 计算逆动力学需要 O(N) 时间（N=关节数）；MuJoCo 的 `mj_inverse` 也需要 O(N) 时间。两者的算法是否相同？提示：Pinocchio 用 Featherstone 空间向量递推；MuJoCo 用 composite rigid body + constraint Jacobian 反解

### 预计学习时间

2 周（20-28 小时），其中：
- MuJoCo 安装与 viewer 交互：3 小时
- mjModel/mjData 结构理解 + 与 Pinocchio 对比：4 小时
- 三大核心函数理解：3 小时
- MJCF 建模语言学习：4 小时
- Menagerie 模型探索：2 小时
- 实战练习：6 小时
- 源码精读与思考题：4 小时

---

## S2 MuJoCo 交互式控制——MJPC / mjctrl / mink（第 3-4 周，2 周）

**性质**：⚪ **部分共享**——MJPC 的 iLQG/Predictive Sampling 算法与所有方向共享（MPC 通用方法论）；但 MuJoCo 原生的控制工具链（mjctrl 的梯度 IK、mink 的 QP IK）主要用于操作方向。

### 科研发展脉络

| 年份 | 论文/项目 | 核心贡献 |
|------|---------|---------|
| 2012 | Tassa, Erez, Todorov, "Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization", IROS | **iLQG 用于 MuJoCo 实时控制**——MPC 作为行为合成器 |
| 2014 | Tassa, Mansard, Todorov, "Control-Limited DDP", ICRA | 控制受限 iLQG——通过 clamp 投影处理执行器约束 |
| 2022 | Howell, Gileadi, Tunyasuvunakool, Zakka, Erez, Tassa, "Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo", arXiv 2212.00541 | **MJPC 的理论基础**——Predictive Sampling(MPPI 变体)+ MuJoCo 作为高速前向模型 |
| 2023 | Google DeepMind, `mujoco_mpc` (MJPC) 发布 | **实时交互式 MPC GUI**——拖动目标即时看到行为；支持 iLQG / Gradient / Sampling |
| 2024 | Zakka, `mjctrl`——单文件教学级控制器 | 梯度下降/GN/LM IK + 差分 IK + OSC，**每个 <200 行** |
| 2024 | Zakka, `mink`——QP-based 差分 IK | 从 Stéphane Caron 的 Pink 移植到 MuJoCo；生产级 QP IK |
| 2025 | Zakka, `mjinx`——JAX 可微分 IK | MuJoCo + JAX 的可微分逆运动学 |

**关键人物**：Yuval Tassa（iLQG→MJPC 核心）→ Taylor Howell（Predictive Sampling 理论 + Dojo）→ Kevin Zakka（mjctrl/mink/mjinx——教材级代码质量的典范）→ Stéphane Caron（Pink→mink 的 QP IK 理论）。

### 教学目标

1. 理解 MJPC 的三种求解器——**iLQG / Gradient Descent / Predictive Sampling**——的数学原理和适用场景
2. 能用 MJPC 的 GUI **实时可视化 MPC 行为**——拖动目标位姿看轨迹如何变化——这是学习 MPC 最直觉的方式
3. 掌握 mjctrl 的 **IK 算法族**——梯度下降 / Gauss-Newton / Levenberg-Marquardt / 差分 IK / Operational Space Control——**每个 <200 行 Python**
4. 理解 mink 的 **QP-based 差分 IK**——如何把关节限位、碰撞避免、优先级任务编码为 QP 约束
5. 能将 mjctrl/mink 作为**快速原型工具**——在 MuJoCo 中 10 分钟搭建一个机械臂 pick-and-place demo

### 前置依赖

- S1（MuJoCo 核心）——mjModel/mjData/mj_step
- v8 Ch11（Eigen）——QP 的矩阵结构
- 腿足 Ch50 或机械臂 M5（QP 建模，如果已学）——OSQP/ProxQP 可直接复用
- 腿足 Ch54（DDP 家族，如果已学）——iLQG 是 DDP 的变体

### 核心知识点

#### S2.1 MJPC——MPC 教学的最佳交互式工具

**GitHub**: `google-deepmind/mujoco_mpc`, ~1.5k★, Apache-2.0。

MJPC 是 DeepMind 发布的**实时交互式 MPC 框架**，内置 MuJoCo 作为前向模型，提供三种 planner 和一个可实时拖拽目标的 GUI。**对于 MPC 教学，MJPC 的价值是不可替代的**——学生可以直接"看到"MPC 在做什么。

**三种 Planner 对比**：

| 求解器 | 算法 | 最优性 | 计算 | 适用场景 |
|--------|------|--------|------|---------|
| **iLQG** | 二阶方法（Gauss-Newton 近似 Hessian）+ 正则化 + 线搜索 | 局部最优 | 中-高 | 平滑动力学、需要精确跟踪 |
| **Gradient** | 一阶梯度下降 + Armijo 线搜索 | 收敛慢 | 低 | baseline / 简单系统 |
| **Predictive Sampling** | MPPI 变体：N 条随机轨迹 + 指数加权平均 | 全局探索 | 高（可并行） | 非光滑/接触丰富 |

**iLQG 的数学核心**（Tassa 2014）：

```
前向传播：rollout nominal trajectory τ = {x₀, u₀, ..., u_{N-1}}
反向传播：对每步 k=N-1,...,0：
  Q_{xx} = ℓ_{xx} + A^T V'_{xx} A          ← 状态二阶展开
  Q_{ux} = ℓ_{ux} + B^T V'_{xx} A          ← 交叉项
  Q_{uu} = ℓ_{uu} + B^T V'_{xx} B + μI     ← 控制二阶展开 + 正则化
  k_k = −Q_{uu}⁻¹ q_u                      ← 前馈增益
  K_k = −Q_{uu}⁻¹ Q_{ux}                   ← 反馈增益
  V_{xx} = Q_{xx} - K^T Q_{uu} K           ← Riccati 更新
控制律：δu_k = αk_k + K_k δx_k              ← α 是线搜索步长
```

**与 OCS2 SLQ 的关系**：OCS2 的 SLQ 求解器是 iLQG 的**连续时间版本 + 约束增广**——数学结构完全同构。在 MJPC 中先理解 iLQG，再看 OCS2 SLQ 会容易很多。

**Predictive Sampling 的数学核心**（Howell 2022）：

```
采样 N 条轨迹：u^(i) = u_nominal + ε^(i),  ε^(i) ~ N(0, Σ)
评估代价：S^(i) = Σ_k ℓ(x_k^(i), u_k^(i)) + ℓ_f(x_N^(i))
指数加权：w^(i) = exp(−S^(i) / λ) / Σ_j exp(−S^(j) / λ)
更新：u_new = Σ_i w^(i) · u^(i)
```

**λ（温度参数）的物理含义**：λ 小 → 只取最好的轨迹（exploit）；λ 大 → 均匀混合（explore）。这是 MPPI（Model Predictive Path Integral）的核心，在 Nav2 的 MPPI controller 中也用了同样的数学。

**MJPC GUI 的教学流程**：

```bash
# 编译运行
cd mujoco_mpc && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
./bin/mjpc

# GUI 中的操作：
# 1. 选择 Task（humanoid/walker/cartpole/...）
# 2. 选择 Planner（iLQG / Sampling / Gradient）
# 3. 拖动绿色目标球——观察 MPC 实时规划轨迹
# 4. 调整 Q/R 权重——观察行为变化
# 5. 切换 Planner——对比 iLQG（光滑最优）vs Sampling（全局探索但抖动）
```

#### S2.2 mjctrl——教材级单文件控制器

**GitHub**: `kevinzakka/mjctrl`, ~400★, Apache-2.0。

Kevin Zakka 的 mjctrl 是**教材级代码质量的典范**——每个控制算法一个 Python 文件，不超过 200 行，无框架依赖，直接用 MuJoCo 和 NumPy。

**IK 算法族**：

| 文件 | 算法 | 核心公式 | 行数 |
|------|------|---------|------|
| `gradient_ik.py` | 梯度下降 IK | `Δq = α · J^T · e` | ~80 |
| `gauss_newton_ik.py` | Gauss-Newton IK | `Δq = (J^T J + μI)⁻¹ · J^T · e` | ~100 |
| `levenberg_marquardt_ik.py` | LM 阻尼最小二乘 IK | `Δq = (J^T J + λ·diag(J^T J))⁻¹ · J^T · e` | ~120 |
| `diffik.py` | 差分 IK | `q̇ = J⁺ · ẋ_d + (I−J⁺J) · q̇_null` | ~100 |
| `osc.py` | Operational Space Control | `τ = J^T · Λ · (K_p e_x + K_d e_v) + τ_null` | ~150 |

**教学设计精华**：每个文件的 `main()` 都是一个完整的 demo——加载 Menagerie 的 Panda 模型、设置目标位姿、运行 IK/控制器、在 viewer 中可视化。**10 分钟内从零到可运行**。

**差分 IK 的核心代码**（约 30 行核心逻辑）：

```python
def diffik_step(m, d, target_pos, target_quat, dt):
    """单步差分 IK——与 MoveIt2 的 Servo 模块等价但简洁 100 倍"""
    # 当前末端位姿
    ee_pos = d.site('attachment_site').xpos
    ee_mat = d.site('attachment_site').xmat.reshape(3, 3)
    
    # 位置误差
    pos_err = target_pos - ee_pos
    
    # 姿态误差（轴角表示）
    target_mat = quat2mat(target_quat)
    err_mat = target_mat @ ee_mat.T
    ori_err = mat2axisangle(err_mat)
    
    # 6D 误差向量
    error = np.concatenate([pos_err, ori_err])
    
    # 几何 Jacobian
    jac = np.zeros((6, m.nv))
    mujoco.mj_jacSite(m, d, jac[:3], jac[3:], site_id)
    
    # 阻尼伪逆
    jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + damping**2 * np.eye(6))
    
    # 速度命令
    dq = jac_pinv @ (error / dt)
    
    return dq  # 发送给 MuJoCo 的 velocity actuator
```

#### S2.3 mink——生产级 QP-based 差分 IK

**GitHub**: `kevinzakka/mink`, ~500★, Apache-2.0。

mink 从 Stéphane Caron 的 Pink（Pinocchio 生态的 QP IK）移植到 MuJoCo，把差分 IK 形式化为带约束的 QP：

```
min  ½ ‖J q̇ - ẋ_d‖² + regularization terms
s.t. q_min ≤ q + q̇·dt ≤ q_max      ← 关节限位
     q̇_min ≤ q̇ ≤ q̇_max              ← 速度限制
     A_collision q̇ ≤ b_collision      ← 碰撞避免（可选）
```

**与 MoveIt2 Servo 的对比**：

| 维度 | mink | MoveIt2 Servo |
|------|------|--------------|
| 后端 | MuJoCo + OSQP | Pinocchio/KDL + 无 QP（Jacobian 伪逆） |
| 约束处理 | QP 显式约束（关节限/碰撞） | 截断（clamp）——不保证可行 |
| 安装 | `pip install mink`（2 分钟） | ROS2 + MoveIt2 + colcon build（30 分钟+） |
| 代码量 | 核心 ~500 行 | 核心 ~5000 行 |
| 适用场景 | 快速原型、RL 数据采集 | 工业部署 |

**mink 的多任务优先级**（与腿足 WBC/TSID 的数学同构）：

```python
import mink

configuration = mink.Configuration(m, d)

# 定义任务（优先级递减）
tasks = [
    mink.FrameTask(
        frame_name="ee_site",
        frame_type="site",
        position_cost=1.0,
        orientation_cost=0.5,
    ),
    mink.PostureTask(model=m, cost=0.01),  # 正则化
]

# 定义约束
limits = [
    mink.ConfigurationLimit(m),     # 关节限位
    mink.VelocityLimit(m, max_vel), # 速度限制
]

# 求解 QP
vel = mink.solve_ik(configuration, tasks, dt, solver="osqp", limits=limits)
```

#### S2.4 MJPC 与 OCS2/acados 的定位对比

| 维度 | MJPC | OCS2 | acados |
|------|------|------|--------|
| **定位** | **研究原型 + 教学** | **生产 MPC 框架** | **嵌入式 MPC 求解器** |
| **物理后端** | MuJoCo（内置） | Pinocchio + CppAD | CasADi → C codegen |
| **求解器** | iLQG / Sampling / Gradient | SLQ / SQP(HPIPM) | SQP-RTI(HPIPM/BLASFEO) |
| **约束** | iLQG 用 clamp；Sampling 自然满足 | 显式等式/不等式约束 | 显式等式/不等式约束 |
| **ROS 集成** | ❌（独立 GUI） | ✅ MPC_Node/MRT_Node | ⚪ 需自行包装 |
| **实时性** | 研究级（~10 ms） | 生产级（~1 ms） | **嵌入式级（~0.1 ms）** |
| **GUI** | ✅ **最强**——实时拖拽目标 | ❌ 仅 RViz 可视化 | ❌ |
| **代码可读性** | ★★★★★ | ★★★ | ★★★ |

**教学建议**：用 MJPC 建立 MPC 直觉（"什么是预测时域、什么是代价权重"），然后用 OCS2/acados 做工程部署。

### 前沿工作与开放问题

- **mjinx（Zakka 2025）**：JAX 可微分 IK——MuJoCo 前向运动学 + JAX 自动微分 + 基于梯度的 IK 求解；可嵌入 RL 训练循环
- **MuJoCo MPC + RL 融合**：arXiv:2503.04613 "Whole-Body MPC with MuJoCo" 展示了 MuJoCo iLQR + Pinocchio 的组合架构——在 Go1 和人形上实时全身 MPC
- **Predictive Sampling 的 GPU 并行**：Playground 将 MJPC 的 Sampling planner 移植到 MJX/Warp 上做 GPU 并行采样——性能提升 10-100×
- **开放问题**：如何在 MJPC 中处理接触模式切换（当前 iLQG 在接触边界处梯度不连续）；Predictive Sampling 的采样效率在高维系统（30+ DOF 人形）中退化

### 项目精读清单

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **MJPC** | `mjpc/planners/ilqg/planner.cc` | iLQG 的 C++ 实现——前向/反向传播 + 正则化 + 线搜索 | 源码阅读（核心） |
| **MJPC** | `mjpc/planners/sampling/planner.cc` | Predictive Sampling——MPPI 变体 | 源码阅读（核心） |
| **MJPC** | `mjpc/tasks/humanoid/stand/task.xml` | 人形站立任务的代价定义——MJPC 的 XML task 格式 | 配置阅读 |
| **mjctrl** | `diffik.py` | 差分 IK 的最简实现——教材级 | 代码阅读（核心） |
| **mjctrl** | `osc.py` | Operational Space Control——力控基础 | 代码阅读 |
| **mink** | `src/mink/solve_ik.py` | QP-based IK 的 OSQP 求解 | 代码阅读 |

### 实战练习

- **[A 型·MJPC 体验]** 编译运行 MJPC。选择 `humanoid/stand` 任务，分别用 iLQG 和 Predictive Sampling 求解。拖动目标位置，观察两种 planner 的轨迹差异。用 MJPC 的 plots 面板观察代价函数收敛曲线
- **[A 型·MJPC 自定义任务]** 在 MJPC 中定义一个新任务：让 Menagerie 的 Panda 机械臂末端跟踪一个圆形轨迹。编写 `task.xml` 定义代价函数（位置跟踪 + 速度正则）。对比 iLQG 和 Sampling 的跟踪质量
- **[A 型·mjctrl IK]** 运行 mjctrl 的 `levenberg_marquardt_ik.py`，让 Panda 达到一个目标位姿。修改阻尼参数 λ，观察收敛速度和稳定性的 tradeoff。尝试给一个不可达的目标位姿——观察 LM 如何优雅退化
- **[A 型·mink QP IK]** 用 mink 实现一个带关节限位的差分 IK。设置目标位姿接近关节限位边界——观察 QP 如何同时满足跟踪和约束。对比无约束的 mjctrl 差分 IK——后者会违反关节限位
- **[A 型·mjctrl OSC]** 运行 mjctrl 的 `osc.py`，让 Panda 在末端施加一个恒力（如按压桌面）。修改刚度 K_p 和阻尼 K_d，观察力跟踪质量。这是力控/阻抗控制的最简入门
- **[B 型·MJPC iLQG 精读]** 精读 `mjpc/planners/ilqg/planner.cc` 的 `BackwardPass` 函数（约 150 行）。对照 Tassa 2014 论文的公式，标注 Q_{xx}/Q_{ux}/Q_{uu}/k/K 的计算。对比腿足 Ch54 讲的 Crocoddyl DDP——两者数学结构相同但约束处理不同
- **[B 型·Predictive Sampling 精读]** 精读 `mjpc/planners/sampling/planner.cc`。标注：采样分布 Σ、温度参数 λ、指数加权公式。对比无人机 D2 讲的 Nav2 MPPI——两者数学完全同构
- **[思考题]** MJPC 的 iLQG 用 `clamp(u, u_min, u_max)` 处理执行器约束；OCS2 的 SQP 用 HPIPM QP 求解器的不等式约束处理。两者的优劣是什么？提示：clamp 简单但可能导致 iLQG 反向传播的梯度信息丢失；QP 约束精确但计算量大
- **[思考题]** mink 的 QP IK 和腿足 Ch53 的 TSID WBC 在数学上有什么关系？提示：两者都是"把多个运动学/动力学任务编码为 QP 约束"——TSID 增加了动力学方程作为等式约束和接触力作为决策变量

### 预计学习时间

2 周（20-28 小时），其中：
- MJPC 安装、交互、自定义任务：6 小时
- mjctrl IK 算法族理解与实践：4 小时
- mink QP IK 理解与实践：3 小时
- MJPC vs OCS2/acados 对比分析：2 小时
- 源码精读：5 小时
- 思考题与论文阅读：4 小时

---

# Part S-II：MuJoCo GPU 生态与多仿真器融合（第 5-7 周）

> **本部分定位**：2024-2026 年 MuJoCo 生态经历了从"CPU 研究工具"到"GPU 并行 RL 训练平台"的剧变。MJX（JAX 后端）、MuJoCo Warp（NVIDIA Warp 后端）、Playground（端到端训练→sim2real）、mjlab（Isaac Lab API 移植）和 Holosoma（多仿真器人形）构成了一个完整但仍在快速整合的生态。本章帮助学员**在这个分裂的生态中做出正确的技术选型**。

---

## S3 MuJoCo GPU 生态——MJX / Warp / Playground / mjlab / Holosoma（第 5-7 周，3 周）

**性质**：✅ **全方向共享**——GPU 并行 RL 训练是所有方向（腿足/机械臂/无人机）的共同需求。

### 科研发展脉络

| 年份 | 事件 | 意义 |
|------|------|------|
| 2021 | Freeman 等(Google), "Brax — A Differentiable Physics Engine for Large Scale RL", NeurIPS | **JAX 可微物理引擎先驱**：定制物理（非 MuJoCo），千级并行 |
| 2023 | MuJoCo 3.0：MJX 发布 | **MuJoCo 物理 + JAX 并行+可微**——Brax 物理逐步被 MJX 替代 |
| 2024 | Haarnoja 等(DeepMind), "Learning agile soccer skills for a bipedal robot with deep RL", Science Robotics | **OP3 Soccer**：MuJoCo 训练 → zero-shot 真机部署（20 DOF 人形足球） |
| 2025-01 | Tassa 等(DeepMind), "MuJoCo Playground", arXiv 2502.08844, **RSS 2025 Outstanding Demo** | **端到端 GPU RL 训练→sim2real**：6 个平台验证（Go1/G1/Barkour/T1/LEAP/Panda） |
| 2025-03 | MuJoCo Warp 发布（DeepMind + NVIDIA 联合） | **NVIDIA GPU 原生 MuJoCo**——Warp CUDA kernel，比 MJX 快数倍 |
| 2025-09 | Newton 1.0（Linux Foundation：NVIDIA + DeepMind + Disney Research） | **统一物理引擎**：MuJoCo Warp 为主求解器 + Disney Kamino 等 |
| 2025-10 | leap-c v0.1.0-alpha（Freiburg Diehl 组） | **acados NMPC 作为 PyTorch 可微层** |
| 2025-12 | Holosoma（Amazon FAR, arXiv 2512.01996） | **多仿真器人形 sim2real**：IsaacGym + IsaacSim + MuJoCo Warp；FastSAC/FastTD3 15 分钟训练 |
| 2026-01 | mjlab v1.0.0（UC Berkeley, arXiv 2601.22074） | **Isaac Lab Manager API → MuJoCo Warp**：~1.8k★ |
| 2026-03 | Isaac Lab 3.0 beta：多后端工厂架构 | `isaaclab_newton` 后端可无 Isaac Sim 运行，返回 `wp.array` |

**关键实验室**：DeepMind Robotics（MuJoCo/MJX/Playground/Warp 核心维护）→ UC Berkeley BAIR（mjlab：Zakka/Liao/Sreenath/Abbeel）→ Amazon FAR（Holosoma：Sferrazza/Shi/Abbeel）→ NVIDIA（Newton + Isaac Lab 3.0）→ Linux Foundation（Newton 治理）。

### 教学目标

1. 理解 **MJX / MuJoCo Warp / Playground / mjlab / Holosoma** 五个项目的定位差异——它们不是竞争关系而是**不同抽象层级的互补**
2. 掌握 **MJX 的 JAX 编程模式**——`jax.vmap(mj_step)` 做批量仿真、`jax.grad(mj_step)` 做可微分物理
3. 理解 **MuJoCo Warp 与 MJX 的性能差异**——Warp 在 NVIDIA GPU 上更快但当前不支持 autodiff
4. 能用 **Playground 在 30 分钟内训练一个四足 locomotion 策略**并理解 sim2real pipeline
5. 理解 **mjlab 为什么是 IsaacGym/Lab 用户迁移到 MuJoCo 的最低摩擦路径**——Manager API 几乎 1:1 对应
6. 掌握 **多仿真器 sim2sim 验证**的工程方法论——IsaacGym 训练 + MuJoCo 验证是当前工业默认实践
7. 能做出**项目级的仿真器选型决策**——什么场景用 Playground、什么场景用 mjlab、什么场景留在 IsaacLab

### 前置依赖

- S1（MuJoCo 核心）——mjModel/mjData/MJCF
- v8 Ch36（CUDA 基础，如果已学）——理解 GPU 并行
- 腿足 Ch67-68 或无人机 D9（RL 训练经验）——PPO/SAC + IsaacGym/Lab

### 核心知识点

#### S3.1 MJX——MuJoCo 的 JAX 后端

**MJX（MuJoCo XLA）** 是 MuJoCo 3.0+ 内置的 JAX 后端——**用纯 JAX 重写了 MuJoCo 的核心物理引擎**，可在 GPU/TPU 上运行，支持 `jax.vmap`（批量并行）和 `jax.grad`（自动微分）。

**MJX 的编程模型**——与 CPU MuJoCo 的对比：

```python
import mujoco
from mujoco import mjx
import jax
import jax.numpy as jnp

# === CPU MuJoCo ===
m = mujoco.MjModel.from_xml_path("go2.xml")
d = mujoco.MjData(m)
mujoco.mj_step(m, d)  # 单次步进

# === MJX（GPU） ===
mx = mjx.put_model(m)           # 上传模型到 GPU
dx = mjx.put_data(m, d)         # 上传数据到 GPU
dx = mjx.step(mx, dx)           # GPU 上步进（返回新 dx，JAX 函数式）

# === 批量并行（4096 环境）===
batch_dx = jax.vmap(mjx.step, in_axes=(None, 0))(mx, batch_dx)

# === 可微分物理 ===
def loss(qpos):
    dx = dx.replace(qpos=qpos)
    dx = mjx.step(mx, dx)
    return jnp.sum(dx.qpos[:3]**2)  # CoM 位置损失
grad_fn = jax.grad(loss)
g = grad_fn(dx.qpos)  # 梯度流经物理步进！
```

**MJX 的关键限制**：
- **静态形状约束**（JAX 要求）：接触数按"可能接触数"分配，不按"活跃接触数"——内存效率低于 CPU MuJoCo
- **JIT 编译时间**：首次调用 `mjx.step` 需 1-3 分钟编译——之后缓存
- **特性覆盖**：MJX 不支持所有 MuJoCo 特性——截至 3.7.0，free/slide/hinge 关节、软约束、condim=1/3/4 已支持；SDF 碰撞、flex 可变形体尚未支持

**MJX vs Brax 的关系**：Brax 的 README 明确声明：**物理仿真功能应迁移至 MJX/MuJoCo Warp**，Brax 仅维护 `brax/training`（PPO/SAC 训练循环）。MJX 继承了 Brax 的训练基础设施但使用 MuJoCo 物理——**更准确的物理 + 相同的并行性能**。

#### S3.2 MuJoCo Warp——NVIDIA GPU 原生 MuJoCo

**GitHub**: `google-deepmind/mujoco_warp`，DeepMind + NVIDIA 联合维护。

MuJoCo Warp 用 **NVIDIA Warp** CUDA kernel 重写了 MuJoCo 核心——**比 MJX(JAX) 在 NVIDIA GPU 上更快**（NVIDIA 官方 benchmark：locomotion 152×、manipulation 313× 加速 vs MJX on RTX 4090），但**当前不支持 autodiff**。

**MJX vs Warp 选型**：

| 维度 | MJX（JAX 后端） | MuJoCo Warp（CUDA 后端） |
|------|---------------|----------------------|
| 硬件 | GPU(NVIDIA/AMD) + TPU | **仅 NVIDIA GPU** |
| 并行性能 | 高（千级并行） | **更高**（数千-万级并行） |
| 可微分 | ✅ `jax.grad` | ❌ **当前不支持** |
| 张量类型 | `jax.Array` | `wp.array` |
| 编译模型 | JAX JIT（XLA） | Warp JIT（CUDA） |
| 框架集成 | Brax training / Playground | **mjlab / Holosoma / Newton / Isaac Lab 3.0** |
| 成熟度 | 较成熟（2023 起） | 较新（2025 起） |

**关键差异**：MJX 适合"需要可微分梯度"的场景（可微分仿真、system ID）；Warp 适合"只需要大规模并行前向仿真"的场景（PPO/SAC 训练）。**大多数 RL 训练不需要可微分**——Warp 是更优选择。

#### S3.3 MuJoCo Playground——端到端 GPU RL 训练→sim2real

**GitHub**: `google-deepmind/mujoco_playground`, ~1.8k★, Apache-2.0。
**论文**: arXiv 2502.08844, **RSS 2025 Outstanding Demo Award**。

Playground 是 DeepMind 发布的**端到端 GPU 加速 RL 训练+sim2real 框架**——基于 MJX（可选 Warp 后端），集成 Brax PPO 训练，已在 6 个机器人平台验证 sim2real。

**已验证的 sim2real 平台**：

| 机器人 | 类型 | 训练时间（单 GPU） | 任务 |
|--------|------|-------------------|------|
| Unitree Go1 | 四足 | ~15 分钟 | locomotion |
| Google Barkour vB | 四足 | ~15 分钟 | locomotion + agility |
| Unitree G1 | 人形 | ~30 分钟 | standing + walking |
| Booster T1 | 人形 | ~30 分钟 | standing + walking |
| LEAP Hand | 灵巧手 | ~10 分钟 | cube rotation |
| Franka Panda | 机械臂 | ~10 分钟 | reaching |

**最小训练示例**：

```python
from mujoco_playground import registry
from brax.training.agents.ppo import train as ppo

# 1. 创建环境
env = registry.load("Go1JoystickFlatTerrain")

# 2. PPO 训练（~15 分钟，单 A100）
make_policy, params, metrics = ppo.train(
    environment=env,
    num_timesteps=200_000_000,
    num_evals=10,
    episode_length=1000,
    batch_size=8192,
    num_minibatches=32,
    learning_rate=3e-4,
)

# 3. 导出策略
policy = make_policy(params)
# → ONNX 或 SavedModel 导出用于真机部署
```

**Playground 的 Warp 后端**（2025-08 加入）：

```python
# 切换到 MuJoCo Warp 后端（Beta）
env = registry.load("Go1JoystickFlatTerrain", impl="warp")
# 性能提升 1.5-2× on manipulation tasks
```

#### S3.4 mjlab——Isaac Lab 的 Manager API 移植到 MuJoCo Warp

**GitHub**: `mujocolab/mjlab`, ~1.8k★, Apache-2.0。
**论文**: arXiv 2601.22074（2026-01）。
**作者**: Kevin Zakka, Qiayuan Liao, Brent Yi, Louis Le Lay, Koushil Sreenath, Pieter Abbeel（UC Berkeley）。

**mjlab 解决的核心问题**：IsaacGym/Lab 用户想用 MuJoCo 物理，但不想重写环境代码。mjlab 把 Isaac Lab 的 Manager-based API（ManagerBasedRLEnv、观测/动作/奖励/终止/事件/课程管理器）**原封不动移植到 MuJoCo Warp 上**。

**与 Isaac Lab 的 API 对比**（几乎 1:1）：

```python
# Isaac Lab 环境定义
class Go2Env(ManagerBasedRLEnv):
    cfg = Go2EnvCfg()
    
# mjlab 环境定义——**完全相同的 API**
class Go2Env(ManagerBasedRLEnv):  # mjlab 的版本
    cfg = Go2EnvCfg()

# 差异仅在底层：Isaac Lab 用 PhysX，mjlab 用 MuJoCo Warp
```

**mjlab 的关键设计选择**：

| 设计 | 说明 |
|------|------|
| **统一 Entity 类** | 替代 Isaac Lab 的 RigidObject/Articulation/Collection 三分类——参数化 base_type 和 articulation 标志 |
| **实例化配置** | 替代 Isaac Lab 的 dataclass 继承——减少样板代码 |
| **CUDA Graph 捕获** | 自动捕获 Warp 仿真步进的 CUDA Graph——消除 dispatch 开销 |
| **Viser 可视化** | Brent Yi 的 web viewer——支持远程/无头可视化 |
| **RSL-RL 集成** | 自带 `MjlabOnPolicyRunner`，多 GPU 支持，W&B 集成 |
| **Rucker 伪惯量参数化** | 域随机化保证物理一致性（质量/CoM/惯量联合采样） |

**Unitree 官方采纳**：`unitreerobotics/unitree_rl_mjlab` 仓库使 mjlab 成为 Go2/A2/G1/H1_2 等全线平台的**官方 RL 训练模板**。

**安装与快速体验**：

```bash
# 零安装体验
uvx --from mjlab demo

# 完整安装
pip install mjlab
# 训练 Go1 locomotion
python -m mjlab.train --task Go1Flat --num_envs 4096 --max_iterations 1000
```

#### S3.5 Holosoma——多仿真器人形 sim2real

**GitHub**: `amazon-far/holosoma`, ~1.1k★, Apache-2.0。
**论文**: arXiv 2512.01996, "Learning Sim-to-Real Humanoid Locomotion in 15 Minutes"。
**作者**: Carlo Sferrazza, Younggyo Seo, Guanya Shi 等（Amazon FAR + CMU RI）。

**Holosoma 解决的核心问题**：不同仿真器的接触模型差异会导致同一策略在仿真器 A 成功但在仿真器 B 失败——Holosoma 通过**同时支持 IsaacGym/IsaacSim/MuJoCo Warp/MuJoCo CPU 四种后端**，让策略的 sim2sim 验证成为标准流程。

**FastSAC/FastTD3——15 分钟训练可部署人形策略**：
- **off-policy 算法**（SAC/TD3）代替 PPO——样本效率更高
- 单 RTX 4090 + 4096 并行环境 + 15 分钟 = 可部署的 G1 locomotion 策略
- 对比：PPO 在相同设置下需要 30-60 分钟

**与 mjlab 的差异**：

| 维度 | mjlab | Holosoma |
|------|-------|----------|
| 仿真器 | MuJoCo Warp only | **IsaacGym + IsaacSim + MuJoCo Warp + MuJoCo** |
| API 风格 | Isaac Lab Manager API（移植） | Manager-based（独立设计） |
| 目标平台 | 通用（四足/臂/手） | **人形为主**（G1/T1） |
| RL 算法 | PPO（RSL-RL） | **FastSAC/FastTD3**（off-policy） |
| 附加工具 | 无 | OmniRetarget（动作重定向）、OmniH2O 数据集 |
| 维护者 | UC Berkeley | Amazon FAR |

#### S3.6 Newton 物理引擎与 Isaac Lab 3.0——生态融合的桥梁

**Newton**（`newton-physics/newton`, ~4.3k★, Apache-2.0）是 Linux Foundation 托管的**统一 GPU 物理引擎**，由 NVIDIA + DeepMind + Disney Research 联合发起。**MuJoCo Warp 是 Newton 的主刚体求解器**——这意味着 Newton 下的"物理"实质上是 MuJoCo 物理。

**Isaac Lab 3.0**（beta 2025-12）引入多后端工厂架构：
- `isaaclab_physx`：默认后端（PhysX 5 GPU）
- `isaaclab_newton`：Newton 后端（底层 = MuJoCo Warp）——**可无 Isaac Sim/Omniverse 运行**

**这意味着**：Isaac Lab 用户未来可以"用 Isaac Lab 的 API + MuJoCo 的物理"——mjlab 和 Isaac Lab 3.0 Newton 后端在做同一件事，只是路径不同（社区 vs NVIDIA 官方）。

#### S3.7 仿真器选型决策树

```
你的需求是什么？
│
├── "我是 IsaacGym/Lab 老用户，想用 MuJoCo 物理但不想重写代码"
│   └─→ **mjlab**（API 几乎 1:1，迁移成本最低）
│
├── "我想用最简单的方式训练一个四足/人形策略并部署真机"
│   └─→ **Playground**（pip install → 30 分钟训练 → sim2real）
│
├── "我需要多仿真器交叉验证，目标是人形"
│   └─→ **Holosoma**（4 种仿真器 + FastSAC）
│
├── "我需要可微分仿真梯度（system ID / 可微分 MPC）"
│   └─→ **MJX**（JAX `jax.grad` 可微）——Warp 当前不支持 autodiff
│
├── "我需要极大规模并行（>10k 环境）且有 NVIDIA GPU"
│   └─→ **MuJoCo Warp**（raw 性能最高）或 **mjlab**（Warp + Manager API）
│
├── "我做的是操作/灵巧手，需要高保真接触"
│   └─→ **MuJoCo CPU**（最准确的接触 + `mj_inverse` 调试）
│
├── "我要用 NVIDIA 的完整生态（Omniverse/USD/GR00T/Cosmos）"
│   └─→ **Isaac Lab 3.0**（PhysX 或 Newton 后端）
│
└── "我只想学 MuJoCo，不做 GPU 并行"
    └─→ **MuJoCo CPU + dm_control / Gymnasium**
```

### 前沿工作与开放问题

- **MuJoCo Warp autodiff**：当前最大短板——Warp 有 tape-based 反向模式 AD（`wp.Tape`），但 MuJoCo Warp 尚未启用。一旦启用，Warp 将同时具备"最快并行 + 可微分"——可能统一 MJX 和 Warp 的定位
- **Newton 1.0 + 多求解器**：Newton 除 MuJoCo Warp 外还集成了 Disney Kamino（布料/流体）、VBD/MPM（软体）、XPBD（快速约束）——**多物理场统一仿真**是 2026+ 的趋势
- **rsl_rl 跨仿真器化**：rsl_rl 已同时对接 IsaacLab、Playground、mjlab、Holosoma——成为**跨仿真器的统一学习库**
- **开放问题**：多仿真器训练的策略是否比单仿真器更鲁棒？（初步证据：Holosoma sim2sim 验证提升了 sim2real 成功率）；Newton 的多求解器耦合（刚体+布料+流体）在机器人任务中的实际价值

### 项目精读清单

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **MuJoCo** | `mjx/tutorial.ipynb`（官方 Colab） | MJX 的 JAX 编程模式——vmap/grad/jit | 教程（核心） |
| **Playground** | `playground/locomotion/go1/joystick.py` | Go1 locomotion 环境定义——观测/动作/奖励 | 代码阅读（核心） |
| **Playground** | `playground/common/sim2real.py` | sim2real pipeline——域随机化 + 部署 | 代码阅读 |
| **mjlab** | `src/mjlab/envs/manager_based_rl_env.py` | Manager API 的 MuJoCo Warp 实现——对比 Isaac Lab 的同名类 | 代码阅读（核心） |
| **mjlab** | `src/mjlab/sim/mj_sim.py` | MuJoCo Warp 仿真步进——CUDA Graph 捕获 | 代码阅读 |
| **Holosoma** | `holosoma/envs/g1_locomotion.py` | G1 locomotion 环境——多仿真器抽象层 | 代码阅读 |
| **unitree_rl_mjlab** | `unitree_rl_mjlab/envs/` | Unitree 官方 RL 环境定义——Go2/G1/H1 | 代码阅读 |

### 实战练习

- **[A 型·Playground 快速体验]** 安装 Playground（`pip install mujoco-playground`），运行 Go1JoystickFlatTerrain 训练。用 Brax PPO 在单 GPU 上训练 15 分钟。可视化训练曲线和策略行为。对比训练 5 分钟 vs 15 分钟 vs 30 分钟的策略质量
- **[A 型·MJX 可微物理]** 运行 MuJoCo 官方的"Differentiable Physics"Colab。用 `jax.grad` 对一个 cart-pole 的 `mj_step` 求梯度。验证梯度与有限差分一致。**理解梯度如何流经物理步进**
- **[A 型·mjlab 环境迁移]** 如果你有一个 IsaacLab 环境定义（如 legged_gym 的 Go2），把它迁移到 mjlab。对比：代码改动量、训练速度、策略质量。**验证 API 对等性**
- **[A 型·sim2sim 验证]** 在 Playground（MJX）中训练一个 Go1 策略，然后在 MuJoCo CPU 中评估（零样本迁移）。对比 MJX 仿真和 CPU 仿真的行为差异——**理解 GPU 仿真的近似误差**
- **[A 型·Holosoma FastTD3]** 安装 Holosoma，用 FastTD3 训练 G1 locomotion（15 分钟目标）。切换仿真器后端（IsaacGym → MuJoCo Warp），对比相同策略的表现差异
- **[B 型·mjlab vs Playground 架构对比]** 精读 mjlab 的 `ManagerBasedRLEnv` 和 Playground 的环境定义方式。分析：Manager API 的"分离关注点"（观测管理器/奖励管理器独立）vs Playground 的"单片式环境"各自的优劣。在什么规模的项目中 Manager API 更有优势？
- **[B 型·Newton 求解器]** 阅读 MuJoCo 3.6 changelog 中 `mjSOL_NEWTON` 的说明。对比 PGS（Projected Gauss-Seidel）、CG（Conjugate Gradient）和 Newton 三种求解器的数学原理。在什么场景下 Newton 更快？提示：Newton 求解器在 GPU 上避免了 PGS 的顺序依赖
- **[思考题]** mjlab 和 Isaac Lab 3.0 的 Newton 后端都实现了"Isaac Lab API + MuJoCo 物理"。两者的技术路径有什么本质差异？提示：mjlab 直接集成 MuJoCo Warp，绕过 Newton；Isaac Lab 3.0 通过 Newton 间接使用 MuJoCo Warp——多了一层抽象。这层抽象带来什么好处和代价？
- **[思考题]** Playground 在 6 个平台上验证了 sim2real（Go1/Barkour/G1/T1/LEAP/Panda）。这些平台覆盖了四足/人形/灵巧手/机械臂——但没有无人机。为什么 MuJoCo Playground 不适合无人机 RL？提示：参考无人机方向 P0.D 中的 CTBR 动作空间和气动模型讨论

### 预计学习时间

3 周（30-40 小时），其中：
- MJX 编程与可微物理实践：6 小时
- Playground 训练 + sim2real 体验：6 小时
- mjlab 安装、迁移、对比：6 小时
- Holosoma 多仿真器体验：4 小时
- Newton/Isaac Lab 3.0 架构理解：3 小时
- 选型决策与对比分析：3 小时
- 精读与思考题：6 小时

---

# Part S-III：可微分仿真与可微分 MPC（第 8-12 周）

> **本部分定位**：可微分仿真是 2022-2026 年机器人学习领域最活跃的前沿之一——它承诺"梯度流经物理"，让策略学习和系统辨识从零阶（PPO/采样）提升为一阶（解析梯度）甚至二阶（Hessian）。但这个承诺是有条件的——接触的非光滑性会让梯度有偏差（Suh ICML 2022 Outstanding Paper），工程中的梯度爆炸/消失是真实障碍。本部分的教学目标是**既理解可微分仿真的威力，也理解它的边界**——然后做出正确的工程决策。

---

## S4 可微分仿真理论与框架（第 8-10 周，3 周）

**性质**：⚪ **部分共享**——可微分仿真的理论对所有方向通用；但具体框架（Brax/MJX 用 JAX、Warp 用 CUDA、Drake 用 C++ AD）各方向偏好不同。

### 科研发展脉络

| 年份 | 论文 | Venue | 引用 | 核心贡献 |
|------|------|-------|------|---------|
| 2018 | de Avila Belbute-Peres 等, "End-to-End Differentiable Physics for Learning and Control" | NeurIPS | ~800 | **先驱作**：LCP 作为可微层嵌入神经网络 |
| 2019 | Hu 等(MIT CSAIL), "DiffTaichi: Differentiable Programming for Physical Simulation" | ICLR | ~600 | **DiffTaichi**：source-to-source AD 编译器做物理仿真 |
| 2021 | Freeman 等(Google), "Brax — A Differentiable Physics Engine for Large Scale RL" | NeurIPS | ~700 | **Brax**：JAX 可微物理+千级并行——开创 GPU 可微仿真时代 |
| 2022 | Howell, Gileadi 等(Stanford/DeepMind), "Dojo: A Differentiable Physics Engine for Robotics" | 6th RSS Workshop | ~200 | **Dojo**：内点法 NCP 松弛——解析平滑的接触梯度 |
| 2022 | **Suh, Simchowitz, Zhang, Tedrake, "Do Differentiable Simulators Give Better Policy Gradients?"** | **ICML Outstanding Paper** | ~350 | **破除迷信**：证明刚性接触下一阶梯度（FoBG）有严重偏差 |
| 2022 | Xu, Makoviychuk 等(NVLabs), "Accelerated Policy Learning with Parallel Differentiable Simulation" | ICLR | ~250 | **SHAC**：短 horizon 截断 + critic smoothing + 并行可微仿真——Humanoid MTU 比 PPO 快 17× |
| 2024 | Georgiev 等, "AHAC: Adaptive Horizon Actor-Critic for Policy Learning in Contact-Rich Environments" | ICML | ~80 | **AHAC**：自适应 horizon 长度——接触密集场景的梯度稳定化 |
| 2024 | Song, Kim, Scaramuzza(UZH RPG), "Learning Quadruped Locomotion Using Differentiable Simulation" | CoRL | ~60 | **首个四足可微 sim2real**：代理模型分离 floating-base + joint |
| 2025 | Schwarke 等, "Sim-to-Real Locomotion via Analytical Smoothing of Hard Contacts" | CoRL | ~30 | **首个 ANYmal 真机可微 sim zero-shot 部署**：解析平滑硬接触 |
| 2025 | Xing 等, "SAPO: Stabilizing Policy Optimization via Saponiﬁcation" | ICLR | ~20 | **SAPO**：熵正则化稳定可微仿真中的长 horizon 策略梯度 |
| 2025 | Luo, "Residual Policy via Differentiable Simulation" | arXiv | ~10 | 可微仿真残差策略——数秒训练带感知的 locomotion |

**关键实验室**：MIT MCube/CDFG（Suh/Tedrake 理论边界）→ NVLabs（SHAC/DFlex）→ UZH RPG（Song 四足 sim2real）→ TU Munich（Georgiev AHAC）→ Stanford MSL（Howell Dojo）→ CMU CDFG（Xu SHAC 实现）。

### 教学目标

1. 理解**为什么接触的梯度是困难的**——非光滑动力学、互补约束、集合值映射
2. 理解 **Suh 2022 的核心结论**：一阶批量梯度（FoBG）在刚性/接近不连续系统中有**严重偏差**——可微分仿真**不总是**比 PPO 好
3. 掌握**四类接触梯度处理策略**的优劣：penalty-based 软化 / 解析平滑 / 代理解耦 / value function smoothing
4. 理解 **SHAC 算法**：短 horizon 截断（h≈32）+ critic 作为 terminal value → 回避长 horizon 梯度病态
5. 能在 MJX 或 Brax 中**运行一个可微分仿真实验**并观察梯度质量
6. 能做出**"什么时候用可微分仿真、什么时候用 PPO"**的工程判断

### 前置依赖

- S1（MuJoCo 核心）——理解 MuJoCo 的软接触模型为什么对可微分仿真友好
- S3（MJX）——`jax.grad(mjx.step)` 是可微分仿真的基本操作
- 腿足 Ch48（CppAD）或无人机 D2（acados）——理解自动微分的基本原理
- v8 Ch17-18（Ceres Jet / GTSAM）——理解前向模式 vs 反向模式 AD

### 核心知识点

#### S4.1 为什么接触的梯度是困难的——从数学到直觉

**直觉**：一个球从高处落到地面。在"刚好接触"的瞬间，速度从正变成负（弹跳）——这是一个**跳变**。对跳变求导会得到什么？**Dirac δ 函数**——无穷大的"梯度"，在工程中表现为梯度爆炸。

**数学精确表述**：

硬接触（LCP/NCP 形式化）：
```
φ(q) ≥ 0            ← 间隙非负（不穿透）
λ ≥ 0               ← 法向力非负
φ(q) · λ = 0         ← 互补条件：要么不接触(φ>0,λ=0)，要么接触(φ=0,λ>0)
```

互补条件 `φ · λ = 0` 定义了一个**非光滑的约束曲面**——在 φ=0（刚好接触）处，λ 从 0 跳变到正值。状态转移映射 x_{t+1} = f(x_t, u_t) 在接触事件处**不可微**（C⁰ 但非 C¹）。

**四类处理策略**：

| 策略 | 方法 | 梯度质量 | 物理准确度 | 代表工作 |
|------|------|---------|-----------|---------|
| **Penalty-based 软化** | 弹簧-阻尼代替硬约束 | 光滑但步长依赖 | **差**（穿透+回弹不物理） | Brax spring/DFlex/DiffTaichi |
| **解析平滑** | 内点法松弛/隐函数定理 | **精确**（隐式梯度） | 好（保留硬接触结构） | Dojo/Schwarke 2025 |
| **代理解耦** | 高保真前向+简化可微代理 | 近似但实用 | 前向准确，代理近似 | Song CoRL 2024 |
| **Value function smoothing** | 截断 horizon + critic 作为 terminal value | 回避非光滑 | 不直接处理 | SHAC/AHAC/SAPO |

#### S4.2 Suh 2022 的核心结论——破除可微分仿真的迷信

**论文**: Suh, Simchowitz, Zhang, Tedrake, "Do Differentiable Simulators Give Better Policy Gradients?", **ICML 2022 Outstanding Paper**。

**核心实验**：在一个简单的"球撞墙"场景中：
- **零阶策略梯度**（REINFORCE/PPO）：高方差但**无偏**
- **一阶批量梯度**（FoBG，通过可微仿真）：低方差但**有偏**

**为什么 FoBG 有偏？**

在刚性接触系统中，状态转移 f(x, u) 在接触事件处是分段线性的。对 N 个样本求 FoBG：
```
FoBG = (1/N) Σᵢ ∇_θ J(θ; xᵢ)    ← 每个样本的梯度在各自的"分段"内
```

**问题**：`∇_θ J` 在每个分段内是常数（线性区域的梯度不变），但**真实的策略梯度需要考虑"分段边界的跳变"贡献**——FoBG 完全忽略了这些跳变。当系统越刚性（接触越硬），跳变越大，FoBG 的偏差越严重。

**教学意义**：这是本大纲中**最重要的"反直觉"结论**——可微分仿真给出的梯度看起来方差低、计算快，但可能指向错误的方向。学员必须理解这个局限才能做出正确的工程决策。

**什么时候可微分仿真确实更好？**
- 系统接触较软（MuJoCo 的 soft contact 天然适合！）
- 接触模式不频繁切换（操作中缓慢推物体 vs locomotion 中快速步态切换）
- 目标函数光滑（跟踪误差 vs 稀疏奖励）
- 短 horizon（<50 步）——长 horizon 梯度指数退化

#### S4.3 SHAC 算法——可微分仿真 RL 的事实标准

**论文**: Xu, Makoviychuk 等(NVLabs), "Accelerated Policy Learning with Parallel Differentiable Simulation", ICLR 2022。
**GitHub**: `NVlabs/DiffRL`, ~350★。

**SHAC（Short-Horizon Actor-Critic）的核心思想**：

```
问题：长 horizon（H=1000 步）的可微分仿真梯度会指数爆炸/消失
解决：截断到短 horizon（h≈32 步）+ 用 critic V(s) 估计 h 步之后的回报

策略梯度 = ∇_θ [Σ_{t=0}^{h-1} r_t + γ^h · V(s_h)]
           ├── 前 h 步：通过可微仿真反传（精确梯度）
           └── h 步后：通过 critic 近似（避免长 horizon 病态）
```

**SHAC 的训练循环**：

```python
for iteration in range(max_iterations):
    # 1. 前向 rollout（可微仿真，h 步）
    states, actions, rewards = differentiable_rollout(policy, env, h=32)
    
    # 2. critic 估计 terminal value
    V_terminal = critic(states[-1])
    
    # 3. 计算总回报
    total_return = sum(rewards) + gamma**h * V_terminal
    
    # 4. 反向传播——梯度流经物理步进！
    loss = -total_return
    loss.backward()  # 梯度穿过 h 步仿真 + critic
    
    # 5. 更新 actor 和 critic
    actor_optimizer.step()
    critic_optimizer.step()  # critic 用 TD(λ) 更新
```

**SHAC 的关键结果**：
- Humanoid（MuJoCo）：SHAC 比 PPO **快 17× 达到相同回报**
- Ant（MuJoCo）：SHAC 比 PPO 快 5×
- **但在接触密集场景（manipulation with contacts）中优势减小**——Suh 2022 的偏差问题开始显现

**AHAC（Adaptive Horizon Actor-Critic，Georgiev ICML 2024）**的改进：
- h 不固定——**根据梯度范数自适应调整** horizon 长度
- 接触少时 h 增大（利用长 horizon 精确梯度）；接触密集时 h 减小（避免梯度爆炸）
- 在 manipulation 任务上显著优于固定 h 的 SHAC

#### S4.4 框架生态的 2026 全景

| 框架 | GitHub | Stars | 可微方式 | 2026 状态 | 推荐场景 |
|------|--------|-------|---------|----------|---------|
| **MJX** | google-deepmind/mujoco | 12k+ | JAX reverse-mode | ✅ 主流 | MuJoCo 生态内的可微分仿真 |
| **Brax** | google/brax | ~2.7k | JAX reverse | ⚠️ 物理维护期 | 训练循环（PPO/SAC）仍活跃 |
| **MuJoCo Warp** | google-deepmind/mujoco_warp | 新 | **暂不可微** | 🆕 速度王 | 大规模 PPO（不需要梯度） |
| **NVIDIA Warp** | NVIDIA/warp | ~5.4k | tape-based reverse | ✅ 活跃 | 自定义可微物理+渲染 |
| **Newton** | newton-physics/newton | ~4.3k | Warp tape | 🆕 alpha | Isaac Lab 3.0 后端 |
| **Genesis** | Genesis-Embodied-AI/Genesis | ~25k | 部分（MPM 可微） | ⚠️ 性能宣称有争议 | 多物理场 |
| **Drake AutoDiffXd** | RobotLocomotion/drake | ~3.8k | forward-mode C++ | ✅ | MPC/TrajOpt 的 C++ 可微 |
| **Dojo** | dojo-sim/Dojo.jl | ~400 | 隐函数定理（Julia） | 🟡 研究原型 | 接触丰富场景的精确梯度 |
| **DiffTaichi** | taichi-dev/difftaichi | ~2.4k | source-to-source | 🟡 基本冻结 | 教学+MPM 软体 |
| **Nimble** | keenon/nimblephysics | ~480 | PyTorch reverse | ✅ | 生物力学 |

**教学建议**：用 MJX 做可微分仿真实验（最成熟+MuJoCo 物理保真度最高）；用 Brax 的训练代码做 SHAC 实现；用 DiffTaichi 做 MPM 软体的教学演示。

#### S4.5 首个四足可微分 sim2real——Song/Scaramuzza CoRL 2024

**论文**: Song, Kim, Scaramuzza, "Learning Quadruped Locomotion Using Differentiable Simulation", CoRL 2024。

**核心技术**：把全身动力学分解为两部分——
1. **Floating-base dynamics**（6 DOF，非可微/近似可微）→ 用代理模型近似
2. **Joint dynamics**（12 DOF，平滑可微）→ 直接用可微仿真

**分离的原因**：floating-base 通过足端接触力驱动——接触切换使其高度非光滑；joint dynamics 通过电机驱动——平滑且容易可微。

**结果**：
- 单 GPU **分钟级训练收敛**（vs PPO 需要数十分钟）
- 策略质量与 PPO 相当甚至更好
- **首次证明可微分仿真训练的 locomotion 策略可 sim2real**

**Schwarke CoRL 2025 的后续突破**：不再分离 floating-base 和 joint——用**解析平滑硬接触**（内点法 + 隐函数定理）直接对全身动力学求可微梯度，**zero-shot 部署到 ANYmal 真机**——这是可微分仿真 sim2real 的最强结果。

### 前沿工作与开放问题

- **D.Va（NeurIPS 2025 Spotlight）**：从像素输入用可微分仿真训练 Humanoid running——4 小时内完成
- **SAPO（Xing, ICLR 2025）**：熵正则化稳定长 horizon 策略梯度——理论保证收敛
- **Rewarped（Xing, `etaoxing/rewarped`, ~200★）**：基于 Warp 的可微分仿真 RL 平台——SAPO 的实现基础
- **开放问题**：可微分仿真的梯度偏差能否被理论性地修正（而非工程性地回避）？MuJoCo Warp 何时启用 autodiff？可微分仿真+大规模并行（>10k 环境）的内存瓶颈如何解决？

### 项目精读清单

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **MuJoCo** | `mjx/tutorial_diff_physics.ipynb`（官方 Colab） | MJX 可微分物理教程——梯度训练 locomotion | 教程（核心） |
| **NVlabs/DiffRL** | `dflex/` + `shac/` | SHAC 实现——短 horizon 截断 + critic terminal value | 代码阅读（核心） |
| **Suh 2022 论文** | arXiv 2202.00817 | FoBG 偏差的数学证明和实验——**必读** | 论文精读 |
| **Song CoRL 2024** | arXiv 2403.14864 | 四足可微 sim2real——代理模型分离 | 论文精读 |
| **Schwarke CoRL 2025** | arXiv 2404.02887 | ANYmal 解析平滑硬接触 sim2real | 论文精读 |
| **Rewarped** | `etaoxing/rewarped` | SAPO 实现——Warp 可微仿真 RL | 代码阅读 |

### 实战练习

- **[A 型·MJX 可微物理基础]** 在 MJX 中实现：给定一个 cart-pole，用 `jax.grad` 对 `mj_step` 求 ∂x_{t+1}/∂u_t。验证与有限差分（(f(u+ε)−f(u−ε))/(2ε)）一致。逐步增大步数（1步、10步、100步），观察梯度范数的变化——**你会看到梯度指数增长**
- **[A 型·FoBG 偏差实验]** 复现 Suh 2022 的"球撞墙"玩具例：(1) 用可微仿真计算 FoBG；(2) 用 REINFORCE 计算零阶梯度；(3) 用解析解计算真实梯度。对比三者——**FoBG 方向错误的情况应该清晰可见**
- **[A 型·SHAC 简化实现]** 用 MJX + JAX 实现一个简化 SHAC：Ant 环境、截断 horizon h=32、MLP critic 估计 terminal value。对比 SHAC 和纯 PPO 的样本效率。预期：SHAC 快 3-5× 收敛
- **[A 型·可微分 System ID]** 用 MJX 做 system identification：(1) 用"真实"MuJoCo 模型生成轨迹数据；(2) 定义一个参数化模型（质量/摩擦为可学参数）；(3) 用 `jax.grad` 最小化预测误差，优化物理参数。验证：估计的参数与真实参数的偏差
- **[B 型·Suh 2022 精读]** 精读 Suh 2022 的 §3（Stiff Spring 和 Heaviside 例子）。手推：为什么 FoBG 在 Heaviside 函数处的期望梯度不等于期望的梯度？画出 FoBG 和真实梯度的对比图
- **[B 型·SHAC vs PPO 理论对比]** 分析 SHAC 和 PPO 的梯度估计：SHAC = 一阶解析梯度（短 horizon + critic 补偿）；PPO = 零阶策略梯度（REINFORCE + importance sampling）。哪个方差更低？哪个偏差更低？在什么条件下两者等价？
- **[思考题]** MuJoCo 的软接触模型（solref/solimp）让接触力成为状态的光滑函数——这是否意味着 Suh 2022 的 FoBG 偏差问题在 MuJoCo 中不存在？提示：软接触缓解但不消除——当 solref 趋向零（接触趋硬）时偏差仍会出现
- **[思考题]** 可微分仿真（一阶梯度）vs PPO（零阶梯度）vs 进化策略（零阶采样）——三者在理论上的关系是什么？提示：Salimans 2017 "Evolution Strategies as a Scalable Alternative to RL" 展示了 ES 在某些场景下与 PPO 性能相当；可微分仿真是 ES 的"一阶化"

### 预计学习时间

3 周（30-40 小时），其中：
- 接触梯度理论理解：6 小时
- Suh 2022 论文精读 + 偏差实验：6 小时
- SHAC 原理与简化实现：8 小时
- 框架对比与选型：3 小时
- 四足可微 sim2real 论文精读：4 小时
- 实战练习：6 小时
- 思考题与讨论：4 小时

---

## S5 可微分 MPC 与学习-控制融合（第 11-12 周，2 周）

**性质**：⚪ **部分共享**——可微分 MPC 的理论对所有方向通用；但具体工具（acados+leap-c 偏腿足/无人机，theseus 偏 SLAM/操作）各方向侧重不同。

### 科研发展脉络

| 年份 | 论文 | Venue | 引用 | 核心贡献 |
|------|------|-------|------|---------|
| 2018 | Amos, Jimenez, Sacks, Boots, Kolter, "Differentiable MPC for End-to-End Planning and Control" | NeurIPS | ~500 | **奠基作**：box-constrained iLQR 的 KKT 隐式求导；backward pass ≈ 一次 LQR |
| 2020 | Pineda, Amos, Kolter, "Theseus: A Library for Differentiable Nonlinear Optimization" | NeurIPS | ~250 | **Theseus**：通用可微非线性最小二乘层（Gauss-Newton/LM） |
| 2022 | Hansen, Wang, Su, "Temporal Difference Learning for Model Predictive Control" (TD-MPC) | ICML | ~400 | **TD-MPC**：学到的世界模型 + MPPI 规划——不是可微 MPC 但常被混淆 |
| 2023 | Hansen, Su 等, "TD-MPC2: Scalable, Robust World Models for Continuous Control" | ICLR 2024 | ~300 | **TD-MPC2**：19M 参数单超参覆盖 104 任务——model-based RL SOTA |
| 2024 | Romero 等(UZH RPG), "Actor-Critic MPC" | T-RO 2025 | ~80 | **AC-MPC**：可微 MPC 作为 actor + RL 优化 MPC 参数→21 m/s 竞速 |
| 2025 | Frey, Baumgärtner 等(Freiburg), "Differentiable Nonlinear MPC" | arXiv 2505.01353 | ~30 | **acados 可微 NMPC**：IFT + IPM 平滑→前向/伴随灵敏度→比 mpc.pytorch 快 3×+ |
| 2025 | Fichtner, Reinhardt 等(Freiburg), "leap-c: Learning Predictive Control" | v0.1.0-alpha | — | **acados NMPC 作为 PyTorch 可微层**——IL + RL + MPC 统一框架 |
| 2026 | Jahncke 等, "Differentiable Weights-Varying Nonlinear MPC via Gradient-based Policy Learning" | RA-L 2026 | 新 | **自动驾驶的可微 NMPC**——RL 在线调 MPC 权重 |

**关键实验室**：CMU Kolter 组（Amos/OptNet/可微优化先驱）→ Meta FAIR（Theseus）→ UCSD Hao Su 组（TD-MPC/TD-MPC2）→ UZH RPG（AC-MPC 竞速）→ **Freiburg Diehl 组（acados/leap-c——当前最活跃的工业级可微 MPC 团队）**。

### 教学目标

1. 理解 **Amos 2018 可微分 MPC 的数学核心**——KKT 不动点 + 隐函数定理 → backward pass ≈ 修改版 LQR
2. 理解 **TD-MPC2 不是可微分 MPC**——它是"学到的世界模型 + MPPI 规划"，没有对 MPC QP 求导
3. 掌握 **acados 可微分 NMPC（Frey 2025）**——IFT + IPM 平滑 + SQP 中的灵敏度计算
4. 掌握 **leap-c 的使用方式**——如何把 acados NMPC 嵌入 PyTorch 训练循环作为可微层
5. 理解 **AC-MPC（Romero T-RO 2025）**——可微 MPC 作为 actor，RL 优化 MPC 的代价权重和残差模型
6. 能做出**"可微 MPC vs 纯 RL vs 纯 MPC"**的工程选型决策

### 前置依赖

- S4（可微分仿真理论）——理解接触梯度的困难
- 腿足 Ch55（OCS2 MPC）或无人机 D2（acados MPC）——理解 MPC 的 OCP 结构
- v8 Ch17-18（Ceres 非线性优化）——理解 KKT 条件和隐函数定理

### 核心知识点

#### S5.1 Amos 2018 可微分 MPC——KKT + 隐函数定理

**核心思想**：把 MPC 的 QP 求解看作一个**隐函数** u*(θ) = argmin_u J(u; θ)，其中 θ 是可学参数（代价权重、动力学模型参数）。在 KKT 不动点处，用隐函数定理求 ∂u*/∂θ。

**数学推导**（简化版）：

MPC 问题（box-constrained iLQR）：
```
min_u  J(x, u; θ) = Σ_k ℓ(x_k, u_k; θ) + ℓ_f(x_N; θ)
s.t.   x_{k+1} = f(x_k, u_k)
       u_min ≤ u_k ≤ u_max
```

KKT 条件在最优解处：
```
∂J/∂u + λ_min - λ_max = 0    ← 稳定性
λ_min ≥ 0, λ_max ≥ 0         ← 对偶可行
(u - u_min) · λ_min = 0       ← 互补松弛
(u_max - u) · λ_max = 0       ← 互补松弛
```

**隐函数定理**：定义 F(u*, λ*, θ) = 0（KKT 条件），则：
```
∂u*/∂θ = −(∂F/∂(u*,λ*))⁻¹ · ∂F/∂θ
```

**计算效率**：backward pass 只需求解一次修改版 LQR（复用 forward pass 的分解），计算量与 forward pass **同阶**——这使得可微 MPC "几乎免费"。

**GitHub**: `locuslab/mpc.pytorch`, ~944★, Apache-2.0。自 2020 年后半停滞但至今是教学标准参考。

#### S5.2 辨析：TD-MPC2 / DreamerV3 不是可微分 MPC

**这是教学中必须澄清的概念混淆**：

| 方法 | 是否对 MPC 的 QP 求导？ | 本质 |
|------|----------------------|------|
| **可微分 MPC**（Amos 2018, acados+leap-c） | ✅ 是——KKT 隐函数定理 | 控制理论 + 可微优化 |
| **TD-MPC / TD-MPC2** | ❌ 否——用 MPPI 采样规划 | model-based RL（学到的世界模型 + 采样 MPC） |
| **DreamerV3** | ❌ 否——在世界模型中想象 | model-based RL（RSSM 世界模型 + actor-critic in imagination） |
| **MPC-Net**（OCS2） | ❌ 否——用 MPC 生成数据训 NN | 蒸馏（MPC→NN 模仿学习） |

**TD-MPC2 的价值**：它用 19M 参数单超参覆盖 104 任务——是 model-based RL 的 SOTA。但它不涉及"对 QP 求导"——规划阶段用零阶 MPPI 采样，学习阶段用 TD 更新世界模型。

**什么时候用哪种？**

| 场景 | 推荐 |
|------|------|
| 已有准确动力学模型 + 需要安全约束 | **acados 可微 NMPC + leap-c** |
| 无模型 + 多任务泛化 | **TD-MPC2** |
| 需要想象/梦境规划 | **DreamerV3** |
| 已有 MPC + 想用 RL 调参 | **AC-MPC 或 leap-c** |

#### S5.3 acados 可微分 NMPC——Frey 2025

**论文**: Frey, Baumgärtner 等, "Differentiable Nonlinear Model Predictive Control", arXiv 2505.01353（2025-05）。
**GitHub**: `FreyJo/differentiable_nmpc`。

**核心贡献**：把 Amos 2018 从 box-constrained iLQR 扩展到**通用 NMPC（非线性动力学 + 一般不等式约束）**，用 SQP + IPM 的内部结构高效计算灵敏度。

**关键技术**：
1. **两求解器方法**：一个 SQP 求解 NLP（得到 u*），另一个 IPM 求灵敏度（在 u* 处平滑 KKT）
2. **IPM 平滑**：用对数障碍参数 τ_min 平滑活动集切换——τ_min 大时梯度更光滑但近似更粗；τ_min=0 恢复精确（但不连续）灵敏度
3. **前向+伴随灵敏度**：前向灵敏度 ∂u*/∂θ 用于 Jacobian；伴随灵敏度用于 backpropagation——**伴随比前向快 2.5×**

**性能**：比 `mpc.pytorch` 快 **3×+**，且支持非凸约束（mpc.pytorch 仅支持 box 约束）。

#### S5.4 leap-c——acados NMPC 作为 PyTorch 可微层

**GitHub**: `leap-c/leap-c`, BSD-3, v0.1.0-alpha（2025-10）。

**leap-c 的使用模式**：

```python
import torch
from leap_c import AcadosMPCLayer

# 1. 定义 acados OCP（CasADi 符号表达式）
ocp = define_my_ocp()  # 标准 acados Python API

# 2. 包装为 PyTorch 可微层
mpc_layer = AcadosMPCLayer(ocp)

# 3. 在 PyTorch 训练循环中使用
class Policy(torch.nn.Module):
    def __init__(self):
        self.encoder = MLP(obs_dim, param_dim)  # 从观测学 MPC 参数
        self.mpc = AcadosMPCLayer(ocp)
    
    def forward(self, obs, state):
        theta = self.encoder(obs)  # 学到的 MPC 参数（Q/R 权重、残差模型）
        u_star = self.mpc(state, theta)  # 可微 MPC 求解
        return u_star  # 梯度可反传到 encoder！

# 4. 标准 PyTorch 训练
loss = rl_loss(policy, env)  # RL 或 IL 损失
loss.backward()  # 梯度穿过 MPC 求解！
optimizer.step()
```

**leap-c 支持的学习模式**：
- **模仿学习**：用专家数据训练 encoder 预测 MPC 参数
- **强化学习**：PPO/SAC 优化 MPC 参数（Q/R/残差模型）
- **混合**：先 IL 预训练再 RL 微调

**与 OCS2 MPC-Net 的对比**：

| 维度 | leap-c | OCS2 MPC-Net |
|------|--------|-------------|
| 方法 | MPC 在循环中（RL 优化 MPC 参数） | MPC 生成数据→NN 模仿 |
| 部署 | **仍然是 MPC**（保留约束满足） | NN 替代 MPC（约束不保证） |
| 安全性 | ✅ 约束始终满足 | ❌ NN 可能违反约束 |
| 推理速度 | 慢（仍需在线求 MPC） | **快**（NN 推理 ~0.1 ms） |
| 适用 | 安全关键系统（自驾/飞行） | 对速度敏感的系统（腿足 1kHz） |

#### S5.5 AC-MPC——可微分 MPC 在无人机竞速中的应用

**论文**: Romero, Bauersfeld, Scaramuzza, "Actor-Critic MPC", T-RO 2025。
**GitHub**: `uzh-rpg/acmpc_public`。

**核心思想**：把 MPC 视为一个可微的"actor"——策略参数 θ 包括 MPC 的代价权重 Q/R、残差动力学模型的参数、和终值代价的参数。RL（PPO）通过可微分 MPC 的梯度优化 θ。

**结果**：在真实无人机赛道上达到 **21 m/s**——比手调 MPC 快 15%，比纯 RL（Swift）在约束满足方面更安全。

**这是 MPC+RL 混合的最新形态**——不是"用 RL 替代 MPC"，也不是"用 MPC 数据训 NN"，而是"**RL 通过 MPC 的梯度学习 MPC 本身的最优参数**"。

#### S5.6 工程决策矩阵——什么时候用什么

| 场景 | 推荐方案 | 理由 |
|------|---------|------|
| 有准确模型 + 安全约束（自驾/飞行/手术） | **acados NMPC + leap-c** | 约束满足、公式化调参 |
| 大 DOF 人形/loco-manipulation | **IsaacGym/Lab + PPO + DR** | 成熟并行、sim2real 经验丰富 |
| 样本贵/低接触/软体 | **可微分仿真 + SHAC/AHAC** | 解析梯度优势最大化 |
| 通用多任务/像素输入 | **TD-MPC2 / DreamerV3** | 单超参跨任务 |
| System-ID / real-to-sim | **MJX + `jax.grad`** | 低样本标定物理参数 |
| 学习 MPC 参数（Q/R/残差） | **leap-c / AC-MPC** | RL 作为外层参数优化器 |
| 需要 MPC 极速推理（1kHz） | **OCS2 MPC-Net / 蒸馏** | NN 替代 MPC 在线求解 |

### 前沿工作与开放问题

- **Differentiable Weights-Varying NMPC（Jahncke RA-L 2026）**：自动驾驶 MPC 的权重由 RL 在线调整——acados 后端
- **l4casadi（Salzmann 2023，~350★）**：PyTorch/JAX 模型→CasADi SX/MX→acados——Neural-MPC 的桥梁
- **AC4MPC（Baumgärtner 2024）**：actor-critic + NMPC 初值热启动 + value function 作为 terminal cost——AC-MPC 的推广
- **开放问题**：可微分 MPC 在接触丰富的 manipulation 中能否工作？（当前成功案例主要在无接触的无人机/车辆上）；leap-c 的 GPU 后端何时成熟？（当前仅 CPU）

### 项目精读清单

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **mpc.pytorch** | `mpc/mpc.py` | Amos 2018 的 PyTorch 实现——理解 KKT 隐式求导的 backward pass | 代码阅读（核心） |
| **differentiable_nmpc** | `benchmark_diff_mpc/linear_mpc.py` | acados vs mpc.pytorch vs cvxpygen 的 benchmark——理解性能差异 | 代码阅读 |
| **leap-c** | `examples/` | acados NMPC 作为 PyTorch 层的最小示例 | 代码阅读（核心） |
| **TD-MPC2** | `tdmpc2/world_model.py` | 学到的世界模型——理解"不是可微 MPC"但很强的 model-based RL | 代码阅读 |
| **AC-MPC** | `uzh-rpg/acmpc_public` | 可微 MPC actor + PPO critic——无人机竞速 | 代码阅读 |
| **theseus** | `theseus/optimizer/` | 可微非线性最小二乘——理解 IFT 在优化层中的通用实现 | 代码阅读（参考） |

### 实战练习

- **[A 型·mpc.pytorch 入门]** 用 `mpc.pytorch` 实现一个 2D 双积分器（质点）的可微 MPC。定义代价函数参数 θ（目标位置），用 PyTorch `loss.backward()` 优化 θ 使轨迹通过指定中间点。**验证梯度确实穿过了 MPC 求解**
- **[A 型·leap-c 快速体验]** 安装 leap-c，运行 `examples/` 中的基础示例。观察：acados 求解 + PyTorch 反传的联合工作流。修改 MPC 的代价权重 Q/R 作为可学参数，用模仿学习损失训练
- **[A 型·TD-MPC2 对比]** 安装 TD-MPC2，在 DMC Walker-Run 任务上训练。对比：TD-MPC2（model-based RL）vs SAC（model-free RL）的样本效率。**理解世界模型的价值**
- **[A 型·可微 MPC System ID]** 用 leap-c 做一个四旋翼的 system identification：MPC 的动力学模型中有未知参数（质量、阻力系数），用真实轨迹数据通过可微 MPC 的梯度优化这些参数。对比有限差分 vs 解析灵敏度的速度差异
- **[B 型·Amos 2018 精读]** 精读 Amos 2018 的 §3（differentiable LQR control）和 §4（differentiable MPC with control bounds）。手推：为什么 backward pass 是一次修改版 LQR？clamp 投影如何影响梯度？
- **[B 型·Frey 2025 精读]** 精读 Frey 2025 的 §3（IFT for NLP）和 §4（IPM smoothing for active set changes）。理解：为什么 τ_min > 0 让灵敏度光滑？代价是什么？画出 Figure 2——解的灵敏度在活动集切换处的跳变和平滑效果
- **[B 型·AC-MPC 精读]** 精读 Romero T-RO 2025 的 §III。理解：MPC 的代价权重和残差动力学如何参数化？PPO 如何通过 MPC 的梯度更新这些参数？对比纯 RL（Swift）和 AC-MPC 在约束满足方面的差异
- **[思考题]** leap-c 让 MPC 保留在部署循环中（有约束保证但慢），OCS2 MPC-Net 用 NN 替代 MPC（快但无约束保证）。对于腿足机器人的 1kHz 全身控制，哪个更实用？提示：考虑 MPC 求解时间（~1 ms for acados）vs 控制周期（1 ms for 1kHz）——acados 刚好卡在边界上
- **[思考题]** 如果把 Pinocchio 的解析导数（Ch47-48 学过的）和 acados 的 solution sensitivity（本章）结合起来，你能得到什么？提示：Pinocchio 给出 ∂dynamics/∂q（动力学梯度），acados 给出 ∂u*/∂θ（策略梯度）——链式法则给出端到端的 ∂u*/∂(模型参数)
- **[思考题]** 可微分 MPC（Amos 2018）、可微分仿真（S4）和可微分渲染（NeRF/3DGS）分别对"什么"求导？如果把三者链接成一条端到端的可微分管线（图像→状态→控制→物理→图像），你得到的是什么？提示：这是视觉伺服（visual servoing）的终极形态——从像素到执行器的完整梯度

### 预计学习时间

2 周（20-28 小时），其中：
- Amos 2018 理论理解：4 小时
- TD-MPC2 vs 可微 MPC 辨析：2 小时
- acados 可微 NMPC + leap-c 实践：6 小时
- AC-MPC 精读：3 小时
- 工程选型分析：2 小时
- 实战练习：6 小时
- 思考题与论文讨论：4 小时

---

# 附录

---

## 附录 A：项目精读优先级表

| 优先级 | 项目 | GitHub | Stars | 精读重点 | 对应章节 |
|--------|------|--------|-------|---------|---------|
| ★★★★★ | **MuJoCo** | google-deepmind/mujoco | ~12.2k | mjModel/mjData 设计 + mj_forward/inverse + 接触模型 | S1 |
| ★★★★★ | **MuJoCo Menagerie** | google-deepmind/mujoco_menagerie | ~3.2k | 55+ 标准化机器人 MJCF 模型 | S1 |
| ★★★★★ | **MJPC** | google-deepmind/mujoco_mpc | ~1.5k | iLQG / Predictive Sampling / 交互式 GUI | S2 |
| ★★★★★ | **mjctrl** | kevinzakka/mjctrl | ~400 | 单文件 IK 算法族（<200 行/个） | S2 |
| ★★★★★ | **MuJoCo Playground** | google-deepmind/mujoco_playground | ~1.8k | GPU RL 训练 + sim2real 验证 | S3 |
| ★★★★★ | **mjlab** | mujocolab/mjlab | ~1.8k | Isaac Lab Manager API → MuJoCo Warp | S3 |
| ★★★★☆ | **mink** | kevinzakka/mink | ~500 | QP-based 差分 IK | S2 |
| ★★★★☆ | **Holosoma** | amazon-far/holosoma | ~1.1k | 多仿真器人形 sim2real | S3 |
| ★★★★☆ | **NVlabs/DiffRL** | NVlabs/DiffRL | ~350 | SHAC 可微分仿真 RL | S4 |
| ★★★★☆ | **leap-c** | leap-c/leap-c | ~200 | acados NMPC 作为 PyTorch 层 | S5 |
| ★★★★☆ | **acados** | acados/acados | ~2.1k | 嵌入式 MPC 求解器 + 可微 NMPC | S5 |
| ★★★☆☆ | **mpc.pytorch** | locuslab/mpc.pytorch | ~944 | Amos 2018 可微 MPC 教学参考 | S5 |
| ★★★☆☆ | **TD-MPC2** | nicklashansen/tdmpc2 | ~800 | Model-based RL SOTA（辨析：不是可微 MPC） | S5 |
| ★★★☆☆ | **dm_control** | google-deepmind/dm_control | ~4k | PyMJCF + Control Suite | S1 |
| ★★★☆☆ | **Rewarped** | etaoxing/rewarped | ~200 | SAPO 可微仿真 RL 平台 | S4 |
| ★★★☆☆ | **theseus** | facebookresearch/theseus | ~2k | 可微非线性最小二乘（SLAM/触觉） | S5 |
| ★★☆☆☆ | **Dojo** | dojo-sim/Dojo.jl | ~400 | 解析平滑接触梯度（Julia） | S4 |
| ★★☆☆☆ | **Genesis** | Genesis-Embodied-AI/Genesis | ~25k | 多物理场（性能宣称有争议） | S4 |
| ★★☆☆☆ | **robosuite** | ARISE-Initiative/robosuite | ~1.6k | MuJoCo 操作任务基准 | S1 |
| ★★☆☆☆ | **differentiable_nmpc** | FreyJo/differentiable_nmpc | 新 | Frey 2025 的复现代码 | S5 |

---

## 附录 B：论文精读路线（按重要性排序）

### 必读论文（8 篇）

1. **Todorov, Erez, Tassa (IROS 2012)** — MuJoCo 奠基：Gauss 原理+凸优化接触
2. **Suh, Simchowitz, Zhang, Tedrake (ICML 2022 Outstanding Paper)** — 可微分仿真的梯度偏差——**本大纲最重要的理论论文**
3. **Xu, Makoviychuk 等 (ICLR 2022)** — SHAC：短 horizon + critic smoothing
4. **Amos, Jimenez, Sacks, Boots, Kolter (NeurIPS 2018)** — 可微分 MPC 奠基
5. **Frey, Baumgärtner 等 (arXiv 2505.01353, 2025)** — acados 可微 NMPC
6. **Tassa 等 (arXiv 2502.08844, 2025)** — MuJoCo Playground 技术报告
7. **Zakka 等 (arXiv 2601.22074, 2026)** — mjlab 技术报告
8. **Song, Kim, Scaramuzza (CoRL 2024)** — 首个四足可微 sim2real

### 推荐论文（8 篇）

9. Freeman 等 (NeurIPS 2021) — Brax：JAX 可微物理先驱
10. Howell 等 (2022) — Dojo：解析平滑接触
11. Schwarke 等 (CoRL 2025) — ANYmal 可微 sim2real zero-shot
12. Georgiev 等 (ICML 2024) — AHAC：自适应 horizon
13. Hansen, Su (ICLR 2024) — TD-MPC2：model-based RL SOTA
14. Romero 等 (T-RO 2025) — AC-MPC：可微 MPC + RL 竞速
15. Sferrazza 等 (arXiv 2512.01996, 2025) — Holosoma/FastTD3
16. Haarnoja 等 (Science Robotics 2024) — OP3 Soccer：MuJoCo sim2real

---

## 附录 C：教学资源索引

### 官方教程与课程

| 资源 | URL | 内容 |
|------|-----|------|
| MuJoCo 官方 Colab | github.com/google-deepmind/mujoco 的 `python/` | 基础 API、LQR、MJX、可微物理、模型编辑 |
| MuJoCo Bootcamp | pab47.github.io/mujocopy.html | 12 周从零入门课程（Pranav Bhounsule） |
| PyMuJoCoBase | github.com/BolunDai0216/PyMuJoCoBase | Bootcamp 配套代码 |
| CMU 16-745 | Zac Manchester 的最优控制课程 | LQR/iLQR/DDP 理论+实践 |
| MIT 6.832 | Russ Tedrake Underactuated Robotics | Drake + MPC/轨迹优化（全球最佳控制课程之一） |
| Stanford AA 203 | Schwager/Pavone 最优控制 | MPC + RL 理论 |
| GAMES 201 | 太极图形（胡渊鸣），B 站 | DiffTaichi 实战——中文可微仿真课程 |
| CoRL 2024 Workshop | "Differentiable Optimization Everywhere" | 可微 MPC/仿真最前沿报告集 |

### 中文社区资源

| 资源 | 平台 | 内容 |
|------|------|------|
| GAMES 201 可微物理仿真 | B 站 | DiffTaichi MPM/弹性体/流体——中文讲解 |
| CMU 16-745 字幕版 | B 站搬运 | 最优控制理论+iLQR 实践 |
| MIT 6.832 字幕版 | B 站搬运 | Drake + 欠驱动机器人控制 |
| MuJoCo 中文教程 | 知乎/CSDN 散见 | 基础使用教程（质量参差，建议以官方 Colab 为主） |

---

## 附录 D：仿真器选型速查矩阵

```
┌─────────────────────────────────────────────────────────────────┐
│                    2026 年机器人仿真器选型指南                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  需要可微分梯度？──── 是 ─→ MJX（JAX grad）                      │
│       │                    或 Brax training + MJX physics        │
│       否                   或 Warp tape（自定义可微物理）          │
│       │                                                         │
│  需要 GPU 大规模并行？                                            │
│       │                                                         │
│       是 ─→ 有 NVIDIA GPU？                                      │
│       │        │                                                │
│       │        是 ─→ 需要 Isaac Lab API？                        │
│       │        │        │                                       │
│       │        │        是 ─→ **mjlab**（MuJoCo Warp）           │
│       │        │        │    或 Isaac Lab 3.0 Newton             │
│       │        │        │                                       │
│       │        │        否 ─→ **Playground**（最简训练）         │
│       │        │             或 **MuJoCo Warp**（raw 性能）     │
│       │        │                                                │
│       │        否 ─→ MJX（支持 AMD/TPU）                        │
│       │                                                         │
│       否 ─→ 需要精确物理 + 调试？                                │
│                │                                                │
│                是 ─→ **MuJoCo CPU**（mj_inverse 可调试）         │
│                │                                                │
│                否 ─→ 教学入门？ → **dm_control / Gymnasium**     │
│                     ROS 集成？ → **Gazebo Sim + MuJoCo 插件**    │
└─────────────────────────────────────────────────────────────────┘
```


---

## S3-B mjlab 深度实战——从环境定义到真机部署（第 6-7 周的实战重心，2 周）

**性质**：✅ **全方向共享**——mjlab 的 Manager API + MuJoCo Warp 组合适用于四足/人形/机械臂/灵巧手所有平台。

> **本节定位**：S3.4 介绍了 mjlab 的架构和定位。本节是 S3 的**深度实战扩展**——从"理解 mjlab 是什么"推进到"能用 mjlab 独立完成一个完整的 RL 训练→部署流程"。对于有 IsaacLab/legged_gym 经验的 RL 工程师，这是**从 NVIDIA 生态切入 MuJoCo 生态的最低摩擦路径**。

### 科研发展脉络（mjlab 生态的快速演进）

| 时间 | 事件 | 意义 |
|------|------|------|
| 2025-10 | mjlab 内部开发启动（UC Berkeley ME 292b 课程项目） | Zakka/Liao 发现 IsaacLab 学生难以安装 Omniverse——需要轻量替代 |
| 2026-01 | mjlab v1.0.0 + arXiv 2601.22074 公开发布 | ~1.8k★ 快速增长；Unitree 官方采纳 |
| 2026-02 | `anymal_c_velocity` 示例发布 | **自定义机器人集成的参考实现**——如何把非内置机器人接入 mjlab |
| 2026-02 | `g1_spinkick_example` 发布 | **motion tracking + BeyondMimic + 真机部署**的完整流程示例 |
| 2026-04 | `mjlab_playground` 发布 | 从 MuJoCo Playground 移植的任务集合——Go1 getup / T1 locomotion |
| 2026-04 | Unitree `unitree_rl_mjlab` 持续更新 | Go2/A2/As2/G1/R1/H1_2/H2 全线平台支持 |

### 教学目标

1. 掌握 mjlab 的**五层架构**——Simulation→Entity→Scene→Manager→Task——每层的职责和数据流
2. 能**从零创建一个自定义 locomotion 环境**——定义观测/动作/奖励/终止/事件管理器
3. 能**集成一个非内置机器人**（如 ANYmal C）——从 MJCF 模型到可训练环境
4. 能完成**velocity tracking 训练→ONNX 导出→MuJoCo CPU 评估**的完整流程
5. 理解 mjlab 的 **domain randomization 实现**——Rucker 伪惯量参数化保证物理一致性
6. 理解 **motion tracking 管线**——从参考动作（CSV/NPZ）到 BeyondMimic 风格的策略训练
7. 能做 **IsaacLab↔mjlab 的环境迁移**——理解 API 对等点和差异点

### 前置依赖

- S1（MuJoCo 核心）——mjModel/mjData 结构
- S3（GPU 生态概览）——mjlab 在生态中的定位
- 腿足 Ch67-68 或 IsaacLab 经验——Manager-based API 的概念

### 核心知识点

#### S3-B.1 mjlab 五层架构精析

```
┌──────────────────────────────────────────────────────────┐
│ 第五层：Task Registry                                     │
│   register_mjlab_task() 绑定 env_cfg + rl_cfg + runner    │
│   命令行 `uv run train Mjlab-Velocity-Flat-Unitree-G1`   │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│ 第四层：ManagerBasedRLEnv                                 │
│   组合 6 个 Manager：                                     │
│   ├── ObservationManager  ← 定义观测空间                  │
│   ├── ActionManager       ← 定义动作空间+执行器           │
│   ├── RewardManager       ← 定义奖励函数（多项加权）      │
│   ├── TerminationManager  ← 定义终止条件                  │
│   ├── EventManager        ← 定义域随机化事件              │
│   └── CurriculumManager   ← 定义课程学习（可选）          │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│ 第三层：Scene                                             │
│   组合多个 Entity（机器人 + 地形 + 物体）                 │
│   MjSpec 编译 → MjModel → GPU 上传                       │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│ 第二层：Entity                                            │
│   统一的机器人/物体抽象：                                  │
│   ├── EntityCfg（MJCF 路径 + 执行器配置）                 │
│   ├── 自动批量化（N 个环境共享 Model，各有独立 Data）      │
│   └── GPU 高效数据访问（零拷贝 PyTorch tensor）           │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│ 第一层：Simulation（MuJoCo Warp）                         │
│   ├── N 并行世界的 GPU 步进                               │
│   ├── CUDA Graph 捕获——消除 dispatch 开销                │
│   └── 域随机化：per-world 参数扩展                        │
└──────────────────────────────────────────────────────────┘
```

**与 IsaacLab 的层级对照**：

| mjlab 层级 | IsaacLab 对应 | 关键差异 |
|-----------|-------------|---------|
| Simulation（MuJoCo Warp） | SimulationContext（PhysX GPU） | 物理引擎不同；mjlab 直接访问 mjModel/mjData |
| Entity | Articulation / RigidObject / RigidObjectCollection | **mjlab 用统一的 Entity 类**——通过参数化替代三分类 |
| Scene | InteractiveScene | 基本对等 |
| ManagerBasedRLEnv | ManagerBasedRLEnv | **API 几乎 1:1**——最大的迁移优势 |
| Task Registry | `configclass` + `gymnasium.register` | mjlab 用 `register_mjlab_task()` |

**IsaacLab 用户的迁移注意点**：
1. **Entity 统一**：IsaacLab 中 `Articulation` 和 `RigidObject` 是不同类；mjlab 中都是 `Entity`，通过 `EntityCfg` 的参数区分
2. **配置风格**：IsaacLab 用 `@configclass` dataclass 继承；mjlab 用**实例化配置**减少样板
3. **张量类型**：IsaacLab 返回 `torch.Tensor`；mjlab 也返回 `torch.Tensor`（通过 Warp→PyTorch 零拷贝共享）
4. **模型格式**：IsaacLab 通常用 USD/URDF；mjlab 用 **MJCF**（来自 Menagerie 或 URDF 转换）

#### S3-B.2 从零创建一个 Go2 velocity tracking 环境

**步骤一：定义 Scene（机器人 + 地形）**

```python
# envs/go2_velocity/scene_cfg.py
from mjlab.scene import SceneCfg
from mjlab.entities import EntityCfg
from mjlab.actuators import ActuatorCfg
from mjlab.terrains import TerrainImporterCfg

class Go2SceneCfg(SceneCfg):
    # 机器人实体
    robot = EntityCfg(
        mjcf_path="mujoco_menagerie/unitree_go2/go2.xml",
        actuators={
            "legs": ActuatorCfg(
                joint_names=["FR_.*", "FL_.*", "RR_.*", "RL_.*"],
                effort_limit=23.7,       # Nm
                velocity_limit=30.0,     # rad/s
                stiffness=25.0,          # PD kp
                damping=0.5,             # PD kd
            ),
        },
    )
    
    # 地形
    terrain = TerrainImporterCfg(
        terrain_type="plane",  # 或 "heightfield" 做课程学习
    )
    
    # 环境参数
    num_envs: int = 4096
    env_spacing: float = 2.0
```

**步骤二：定义 Manager 管理器**

```python
# envs/go2_velocity/env_cfg.py
from mjlab.envs import ManagerBasedRLEnvCfg
from mjlab.managers import (
    ObservationGroupCfg, ObservationTermCfg,
    RewardTermCfg, TerminationTermCfg, EventTermCfg,
)

class Go2VelocityEnvCfg(ManagerBasedRLEnvCfg):
    scene = Go2SceneCfg()
    
    # === 观测管理器 ===
    observations = {
        "policy": ObservationGroupCfg(
            terms={
                # 基座线速度（body frame）
                "base_lin_vel": ObservationTermCfg(
                    func=mdp.base_lin_vel, scale=2.0,
                ),
                # 基座角速度（body frame）
                "base_ang_vel": ObservationTermCfg(
                    func=mdp.base_ang_vel, scale=0.25,
                ),
                # 重力方向在 body frame 中的投影
                "projected_gravity": ObservationTermCfg(
                    func=mdp.projected_gravity,
                ),
                # 速度命令
                "velocity_commands": ObservationTermCfg(
                    func=mdp.velocity_commands,
                ),
                # 关节位置偏差（相对默认位姿）
                "joint_pos": ObservationTermCfg(
                    func=mdp.joint_pos_rel, scale=1.0,
                ),
                # 关节速度
                "joint_vel": ObservationTermCfg(
                    func=mdp.joint_vel, scale=0.05,
                ),
                # 上一步动作
                "last_action": ObservationTermCfg(
                    func=mdp.last_action,
                ),
            },
            # 观测噪声（域随机化）
            noise={"joint_pos": 0.01, "joint_vel": 1.5},
        ),
    }
    
    # === 奖励管理器 ===
    rewards = {
        # 主任务：线速度跟踪
        "track_lin_vel_xy": RewardTermCfg(
            func=mdp.track_lin_vel_xy_exp,
            weight=1.0,
            params={"std": 0.25},
        ),
        # 主任务：角速度跟踪
        "track_ang_vel_z": RewardTermCfg(
            func=mdp.track_ang_vel_z_exp,
            weight=0.5,
            params={"std": 0.25},
        ),
        # 正则化：线速度 z 惩罚
        "lin_vel_z_l2": RewardTermCfg(
            func=mdp.lin_vel_z_l2, weight=-2.0,
        ),
        # 正则化：角速度 xy 惩罚
        "ang_vel_xy_l2": RewardTermCfg(
            func=mdp.ang_vel_xy_l2, weight=-0.05,
        ),
        # 正则化：关节力矩惩罚
        "joint_torques_l2": RewardTermCfg(
            func=mdp.joint_torques_l2, weight=-0.0001,
        ),
        # 正则化：关节加速度惩罚
        "joint_acc_l2": RewardTermCfg(
            func=mdp.joint_acc_l2, weight=-2.5e-7,
        ),
        # 正则化：动作变化率惩罚
        "action_rate_l2": RewardTermCfg(
            func=mdp.action_rate_l2, weight=-0.01,
        ),
        # 鼓励：足端腾空时间
        "feet_air_time": RewardTermCfg(
            func=mdp.feet_air_time,
            weight=0.125,
            params={"threshold": 0.5, "sensor_names": ["foot_.*"]},
        ),
    }
    
    # === 终止管理器 ===
    terminations = {
        "time_out": TerminationTermCfg(
            func=mdp.time_out, time_out=True,
        ),
        "bad_orientation": TerminationTermCfg(
            func=mdp.bad_orientation,
            params={"limit_angle": 0.5},  # rad，约 28.6°
        ),
    }
    
    # === 事件管理器（域随机化）===
    events = {
        # 每次 reset 时随机化基座位姿
        "reset_base": EventTermCfg(
            func=mdp.reset_root_state_uniform,
            mode="reset",
            params={"pos_range": [0.0, 0.0], "vel_range": [-0.5, 0.5]},
        ),
        # 每次 reset 时随机化关节位置
        "reset_joints": EventTermCfg(
            func=mdp.reset_joints_by_offset,
            mode="reset",
            params={"position_range": [-0.1, 0.1], "velocity_range": [-0.1, 0.1]},
        ),
        # 训练中持续随机化物理参数
        "physics_material": EventTermCfg(
            func=mdp.randomize_rigid_body_material,
            mode="interval",
            interval_range=(250, 750),
            params={"friction_range": [0.5, 1.25]},
        ),
        # 外力扰动
        "push_robot": EventTermCfg(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range=(10, 15),
            params={"velocity_range": {"x": [-0.5, 0.5], "y": [-0.5, 0.5]}},
        ),
    }
    
    # === 动作管理器 ===
    actions = {
        "joint_pos": {
            "class": mdp.JointPositionAction,
            "scale": 0.25,  # 动作乘以 scale 再加到默认关节位置
        },
    }
```

**步骤三：注册任务 + 训练**

```python
# __init__.py
from mjlab.tasks import register_mjlab_task

register_mjlab_task(
    name="Mjlab-Velocity-Flat-Unitree-Go2",
    entry_point="mjlab.envs:ManagerBasedRLEnv",
    env_cfg=Go2VelocityEnvCfg,
    rl_cfg=Go2PPOCfg,  # RSL-RL PPO 配置
)
```

```bash
# 训练（单 GPU，4096 环境）
MUJOCO_GL=egl uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 5000

# 多 GPU 训练
uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --gpu-ids "[0, 1]" \
    --env.scene.num-envs 4096

# 评估（实时可视化）
uv run play Mjlab-Velocity-Flat-Unitree-Go2-Play \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8
```

#### S3-B.3 域随机化——Rucker 伪惯量参数化

mjlab 的域随机化使用 **Rucker & Wensing（RA-L 2022）的伪惯量参数化**——这是对 IsaacLab 的一个关键改进。

**问题**：传统域随机化分别随机化质量 m、质心偏移 c、惯量矩阵 I。但**不是所有 (m, c, I) 的组合都是物理合理的**——例如惯量矩阵必须满足三角不等式 I_xx + I_yy ≥ I_zz。随机采样可能产生不物理的参数。

**Rucker 的解决方案**：把 (m, c, I) 参数化为一个 **10 维伪惯量向量** σ ∈ R¹⁰，然后在 σ 空间中做随机化——任何 σ 值都对应一个**物理合法**的 (m, c, I) 组合。

```python
# mjlab 中的 Rucker 随机化
"randomize_inertia": EventTermCfg(
    func=mdp.randomize_rigid_body_inertia_rucker,
    mode="reset",
    params={
        "mass_range": [0.8, 1.2],   # 质量缩放范围
        "com_range": [-0.05, 0.05], # 质心偏移范围（米）
    },
)
```

**与 IsaacLab 的对比**：IsaacLab 的 `randomize_rigid_body_mass` 和 `randomize_rigid_body_inertia` 是分别操作的——可能产生不一致的参数组合。mjlab 通过 Rucker 参数化**从数学上保证一致性**。

#### S3-B.4 自定义机器人集成——ANYmal C 示例

**参考仓库**: `mujocolab/anymal_c_velocity`, ~57★。

**集成步骤**：

1. **准备 MJCF 模型**：从 Menagerie 获取 `anybotics_anymal_c/anymal_c.xml`
2. **配置执行器**：定义 PD 增益和力矩限制
3. **定义足端接触传感器**：用于足端腾空时间奖励
4. **编写 EntityCfg**：指定 MJCF 路径和执行器映射
5. **复用 velocity tracking 环境**：大部分 Manager 配置可直接从 Go2 复制

**关键代码差异**（相对于 Go2）：

```python
# ANYmal C 的关节命名与 Go2 不同
robot = EntityCfg(
    mjcf_path="mujoco_menagerie/anybotics_anymal_c/anymal_c.xml",
    actuators={
        "legs": ActuatorCfg(
            joint_names=["LF_HAA", "LF_HFE", "LF_KFE",  # 左前
                         "RF_HAA", "RF_HFE", "RF_KFE",  # 右前
                         "LH_HAA", "LH_HFE", "LH_KFE",  # 左后
                         "RH_HAA", "RH_HFE", "RH_KFE"], # 右后
            effort_limit=40.0,    # ANYmal C 的力矩更大
            stiffness=80.0,       # ANYmal C 用更高 PD 增益
            damping=2.0,
        ),
    },
)
```

**教学意义**：这个示例展示了 mjlab 的**环境复用能力**——90% 的环境逻辑（观测/奖励/终止/事件）在不同机器人之间共享，只需要修改 EntityCfg 和少量超参数。

#### S3-B.5 Motion Tracking——G1 回旋踢到真机部署

**参考仓库**: `mujocolab/g1_spinkick_example`, ~213★。

**完整流程**：

```
参考动作数据（MimicKit .pkl）
  ↓ pkl_to_csv.py（加过渡动作 + 重复循环）
  ↓ csv_to_npz.py（重采样到 50 fps + 上传 WandB Registry）
  ↓
mjlab Motion Tracking 环境
  ├── BeyondMimic 风格的 reward：
  │   ├── 关节角跟踪误差
  │   ├── 末端位置跟踪误差
  │   ├── 基座位姿跟踪误差
  │   └── 速度跟踪误差
  ├── 渐进式课程：
  │   └── 随训练进度增加动作平滑/关节速度/功率惩罚
  └── Phase-conditioned：
      └── 观测中包含运动相位变量（周期性动作）
  ↓
uv run train Mjlab-Spinkick-Unitree-G1 --env.scene.num-envs 4096
  ↓ 20,000 iterations（~1-2 小时，单 GPU）
  ↓
ONNX 导出（WandB artifacts 自动存储）
  ↓
motion_tracking_controller（RoboJuDo 框架）
  ↓ BeyondmimicPolicy 加载 ONNX
  ↓
Unitree G1 真机部署
```

**安全注意**：仓库明确声明"此仓库仅供教学目的。我们不对因尝试复现结果而可能发生的财产损坏或人身伤害承担任何责任。请在与硬件交互时谨慎行事。"——**真机部署回旋踢是一个高风险操作**。

#### S3-B.6 Sim2Sim 验证——mjlab → MuJoCo CPU

**为什么需要 sim2sim**：MuJoCo Warp（GPU 并行）和 MuJoCo CPU（单线程精确）的物理行为可能存在微小差异（浮点精度、接触求解迭代数等）。在部署到真机前，先在 MuJoCo CPU 上验证是一个低成本的安全检查。

```python
# 在 MuJoCo CPU 上评估 mjlab 训练的策略
import mujoco
import onnxruntime as ort

# 1. 加载 ONNX 策略
session = ort.InferenceSession("policy.onnx")

# 2. 加载 MuJoCo CPU 模型
m = mujoco.MjModel.from_xml_path("unitree_go2/scene.xml")
d = mujoco.MjData(m)

# 3. 仿真循环
with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        # 构造观测（与训练时一致）
        obs = build_observation(m, d)  # 提取 qpos/qvel/gravity/cmd
        
        # 策略推理
        action = session.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        
        # 施加动作（PD 位置控制）
        target_pos = default_joint_pos + action * 0.25
        d.ctrl[:] = target_pos
        
        # 步进
        mujoco.mj_step(m, d)
        viewer.sync()
```

#### S3-B.7 mjlab vs IsaacLab 的完整迁移对照表

| 操作 | IsaacLab 代码 | mjlab 代码 | 迁移难度 |
|------|-------------|-----------|---------|
| 安装 | Omniverse Launcher + Isaac Sim + pip | `pip install mjlab` | **mjlab 极简** |
| 创建环境 | `class MyEnv(ManagerBasedRLEnv)` | `class MyEnv(ManagerBasedRLEnv)` | **零迁移** |
| 定义观测 | `ObservationTermCfg(func=...)` | `ObservationTermCfg(func=...)` | **零迁移** |
| 定义奖励 | `RewardTermCfg(func=..., weight=...)` | `RewardTermCfg(func=..., weight=...)` | **零迁移** |
| 定义机器人 | `ArticulationCfg(prim_path=..., usd_path=...)` | `EntityCfg(mjcf_path=...)` | **低——改模型路径** |
| 训练命令 | `python scripts/rsl_rl/train.py --task Isaac-Velocity-Flat-G1` | `uv run train Mjlab-Velocity-Flat-Unitree-G1` | **低** |
| 多 GPU | `torchrun --nproc_per_node=2 ...` | `--gpu-ids "[0, 1]"` | **低** |
| 地形课程 | `TerrainGeneratorCfg(...)` | `TerrainImporterCfg(...)` | **中——API 略不同** |
| 域随机化 | `EventTermCfg(func=events.randomize_mass, ...)` | `EventTermCfg(func=mdp.randomize_rigid_body_inertia_rucker, ...)` | **中——Rucker 新增** |
| 模型格式 | USD / URDF（PhysX 需要） | **MJCF**（Menagerie 优先） | **高——需换模型** |
| 物理接触 | PhysX GPU rigid body | MuJoCo Warp soft contact | **高——接触行为不同** |
| 渲染 | Omniverse RTX Ray Tracing | Viser web viewer / MuJoCo native | **不同但够用** |

### 前沿工作与开放问题

- **mjlab_playground（2026-04 最新）**：从 MuJoCo Playground 移植的任务集合——Go1 getup **单卡 5090 上 2 分钟收敛**、Booster T1 **8 分钟收敛**，训练过程使用渐进式课程逐步收紧动作平滑和功率惩罚
- **Berkeley ME 292b 课程部署**：mjlab 已在 UC Berkeley 研究生课程中使用——RL 新手学员在课程中完成首个 legged locomotion 策略训练
- **开放问题**：mjlab 目前仅支持 PPO（RSL-RL）——off-policy 算法（SAC/TD3）支持尚在规划中（Holosoma 的 FastSAC/FastTD3 是参考）；Terrain curriculum 的 heightfield 分辨率受 GPU 内存限制；与 ROS2 的集成尚未官方支持

### 项目精读清单

| 项目 | 文件路径 | 精读重点 | 类型 |
|------|---------|---------|------|
| **mjlab** | `src/mjlab/envs/manager_based_rl_env.py` | 核心环境循环——reset/step/reward 如何组织 | 代码阅读（核心） |
| **mjlab** | `src/mjlab/managers/reward_manager.py` | 奖励管理器——多 term 加权的实现 | 代码阅读 |
| **mjlab** | `src/mjlab/sim/simulation.py` | MuJoCo Warp 仿真步进 + CUDA Graph 捕获 | 代码阅读（进阶） |
| **mjlab** | `src/mjlab/sim/randomization.py` | 域随机化——Rucker 伪惯量参数化的实现 | 代码阅读 |
| **mjlab** | `src/mjlab/terrains/terrain_generator.py` | 地形生成——heightfield + primitive 两种方式 | 代码阅读 |
| **anymal_c_velocity** | 整个仓库（~57★） | **自定义机器人集成的完整参考** | 代码阅读（核心） |
| **g1_spinkick_example** | `pkl_to_csv.py` + `csv_to_npz.py` + 训练配置 | **motion tracking 完整管线** | 代码阅读（核心） |
| **mjlab_playground** | 整个仓库 | Playground 任务移植 + 渐进式课程 | 代码阅读 |
| **unitree_rl_mjlab** | `unitree_rl_mjlab/envs/` | Unitree 官方环境定义——Go2/G1/H1 | 代码阅读 |

### 实战练习

#### 基础实战（A 型）

- **[A 型·零安装体验]** 运行 `uvx --from mjlab demo`（无需 clone 代码），在浏览器中查看 Viser 可视化。**验证：从零到可视化 < 5 分钟**
- **[A 型·G1 velocity tracking 训练]** clone mjlab，运行 `uv run train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 4096 --agent.max-iterations 3000`。观察 W&B 上的训练曲线。**预期：单 RTX 3090/4090 约 15-30 分钟收敛**
- **[A 型·G1 策略评估]** 训练完成后，用 `uv run play` 加载最新 checkpoint。拖动 Viser viewer 中的摇杆，观察 G1 对速度命令的响应。对比 3000 iteration 和 5000 iteration 的策略质量差异
- **[A 型·Go2 环境复制]** 参照内置的 G1 velocity tracking 环境，创建一个 Go2 velocity tracking 环境。步骤：(1) 复制 G1 的环境配置；(2) 修改 EntityCfg 指向 `unitree_go2/go2.xml`；(3) 调整执行器参数（Go2 的力矩限制和 PD 增益与 G1 不同）；(4) 注册新任务；(5) 训练并对比
- **[A 型·域随机化消融]** 在 Go2 velocity tracking 环境中，分别训练：(1) 无域随机化；(2) 仅质量随机化 ±20%；(3) 完整 Rucker 伪惯量随机化 + 摩擦随机化 + 外力扰动。对比三者在测试时的鲁棒性（加 ±30% 质量扰动的 sim2sim 测试）
- **[A 型·ANYmal C 集成]** 参照 `mujocolab/anymal_c_velocity` 仓库，把 ANYmal C 集成到 mjlab 的 velocity tracking 任务中。完成训练并与 Go2 对比策略质量。**这个练习验证了"mjlab 的环境复用能力"**
- **[A 型·Sim2Sim 验证]** 将 mjlab 训练好的 Go2 策略导出为 ONNX，然后在 MuJoCo CPU（非 Warp）上加载和运行。对比 Warp 仿真和 CPU 仿真中策略的行走速度和稳定性。记录差异并分析原因（浮点精度/接触参数/求解器迭代数）

#### 进阶实战（A 型）

- **[A 型·Motion Tracking]** 参照 `g1_spinkick_example`，用 mjlab 的 motion tracking 管线训练一个简单的 G1 步行动作。步骤：(1) 准备 CSV 参考动作数据（可用 Menagerie G1 的默认站姿生成简单的左右摆动）；(2) 转换为 NPZ 格式；(3) 上传到 W&B Registry；(4) 训练 tracking 策略。**不需要做真机部署——仿真验证即可**
- **[A 型·自定义奖励函数]** 在 Go2 velocity tracking 环境中添加一个自定义奖励函数——"能量效率奖励"：`r_energy = -Σ|τ_i · q̇_i|`（关节功率的负值）。实现为一个 `RewardTermCfg`，添加到奖励管理器中。训练后对比：有/无能量奖励的策略在行走能耗上的差异
- **[A 型·地形课程]** 为 Go2 velocity tracking 环境配置 heightfield 地形课程——从平地开始，随训练进度逐步增加斜面/台阶/随机凹凸。使用 `TerrainImporterCfg(terrain_type="heightfield")` + `CurriculumManager`。观察策略是否能泛化到未见过的地形难度

#### 精读与思考（B 型 + 思考题）

- **[B 型·ManagerBasedRLEnv 精读]** 精读 `src/mjlab/envs/manager_based_rl_env.py`（约 300 行）。画出 `step()` 方法的完整流程：(1) 施加动作→(2) 物理步进→(3) 计算观测→(4) 计算奖励→(5) 检查终止→(6) 触发事件→(7) 处理 reset。标注：哪些步骤是 Manager 负责的？哪些是底层仿真器负责的？
- **[B 型·CUDA Graph 精读]** 精读 `src/mjlab/sim/simulation.py` 中 CUDA Graph 捕获的实现。理解：为什么第一步仿真很慢（编译+捕获），后续步骤很快（replay）？CUDA Graph 在什么条件下会失效（需要重新捕获）？
- **[B 型·Entity 统一设计精读]** 精读 `src/mjlab/entities/entity.py`。对比 IsaacLab 的 `Articulation` 类。分析：mjlab 的统一 Entity 设计在什么场景下比 IsaacLab 的三分类更简洁？在什么场景下更不方便？
- **[思考题]** mjlab 的 Manager API 和无人机方向 D8 讲的 Agilicious Pipeline 有什么本质差异？提示：Agilicious 是"线性处理链"（Estimator→Sampler→Reference→Controller→Bridge）；mjlab 是"星型管理器"（6 个 Manager 独立定义，环境循环统一调度）。两种设计适合什么规模的项目？
- **[思考题]** mjlab 用 MuJoCo Warp 做 GPU 并行，但 MuJoCo Warp 当前不支持 autodiff（S3.2 讲过）。如果 Warp autodiff 未来启用，mjlab 的训练管线能获得什么新能力？提示：可以从零阶 PPO 升级为一阶 SHAC（S4.3 讲过）——梯度穿过物理步进
- **[思考题]** `mjlab_playground` 中 Go1 getup 在 5090 上 2 分钟收敛，但使用渐进式课程"逐步收紧动作平滑和功率惩罚"。为什么不从一开始就加这些惩罚？提示：过早的惩罚限制了策略探索空间——策略先学"怎么站起来"（粗糙但有效），再学"怎么优雅地站起来"（平滑高效）

### 预计学习时间

2 周（25-35 小时），其中：
- mjlab 安装与零体验：2 小时
- G1/Go2 velocity tracking 训练：4 小时
- 域随机化消融实验：4 小时
- ANYmal C 自定义集成：4 小时
- Motion Tracking 管线：4 小时
- Sim2Sim 验证：3 小时
- 地形课程：3 小时
- 精读与思考题：6 小时
- 自定义奖励函数实验：3 小时
