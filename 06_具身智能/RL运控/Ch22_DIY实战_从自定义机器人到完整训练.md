# 第 22 章 DIY 实战：从自定义机器人到完整训练

---

## 前置自测

📋 **答不出 $\ge$ 2 题 → 先回对应章节复习**

1. **[Ch04 Manager-Based]** mjlab 的九大 Manager 分别管理 MDP 的哪些组件？`ObservationManager` 和 `RewardManager` 的调用时序是什么？
2. **[Ch05 Obs/Action]** `ObservationGroupCfg` 中 `enable_corruption=False` 的含义是什么？它在 teacher-student 训练中扮演什么角色？
3. **[Ch11 建模]** 用 sw2urdf 导出的 URDF 中，`<inertial>` 标签的三个子元素（mass、origin、inertia）分别描述什么？如果 inertia tensor 的非对角元素全为零意味着什么？
4. **[Ch12 Actuator]** mjlab 中 `ImplicitActuatorCfg` 的 `stiffness` 和 `damping` 参数如何映射到 MuJoCo 的 PD 控制？当 `stiffness=0` 时 actuator 行为是什么？
5. **[Ch06 Reward]** 设计 reward 时"乘法门控"（multiplicative gating）和"加法组合"（additive combination）各自的优缺点是什么？

## 本章目标

学完本章后，你应该能够：

1. **在 mjlab 中从零注册一个全新任务**，包含 EntityCfg、SceneCfg、所有 Manager 配置和 task registry
2. **在 Isaac Lab 中创建一个独立的 extension**，遵循 HOVER 仓库的目录范式
3. **执行完整的验证三步走**：smoke test → zero agent → random agent，每一步有明确的通过标准
4. **系统性地诊断和修复**环境搭建中最常见的 10 类 bug
5. **把任意 URDF/MJCF 机器人接入双框架**，完成从模型导入到策略训练的全流程
6. **根据任务特征选择 Manager-Based 还是 Direct workflow**，并说清楚选择理由

---

## 22.1 为什么需要一个专门的"DIY 章节" ⭐

> **这一节解决什么问题**：建立从"用框架内置任务训练"到"为自己的机器人和任务搭建全新环境"的跨越，明确这个跨越中的核心难点。

### 动机：框架内置任务的局限

从 Ch13 到 Ch21，所有实战任务都基于框架已有的机器人模型和环境配置。Go2 速度跟踪用 mjlab 内置的 `anymal_c_velocity`，G1 人形用内置的 `g1_velocity`，YAM 操作用内置的 `lift_cube_env_cfg`。这些内置任务提供了完善的 MJCF/USD 模型、经过验证的 obs/action/reward 配置、可复现的训练超参。你只需要修改参数就能跑通训练。

但作为博士研究者，你最终要面对的场景是：**你有一个自己设计（或实验室购买）的机器人，有一个独特的研究任务，框架里没有现成的环境**。这时你需要从零搭建：

- 把机器人的 CAD 模型转换为 MJCF 或 USD
- 设计这个任务特有的 obs、action、reward、termination
- 配置合适的 DR 和 curriculum
- 在双框架中注册并验证环境
- 调试到策略能收敛

这个过程中的每一步都可能出错，而错误的表现形式通常不是报错，而是"策略训练不收敛"——这是最难排查的问题类型，因为你不知道错误在 MDP 的哪个组件中。

### 如果直接照搬内置任务的配置会怎样

一个常见的做法是复制一个类似的内置任务（比如 `anymal_c_velocity`），把机器人模型换成自己的，其他配置尽量不改。这种做法的问题在于：

**问题一：关节名不匹配。** 内置任务的 obs 和 action 配置通过关节名引用特定关节（如 `["FL_hip_joint", "FL_thigh_joint", ...]`）。你的机器人关节名不同，配置静默失败——obs 返回全零或错误维度。

**问题二：动力学特性不同。** 内置任务的 action scale、reward sigma、termination 阈值都是针对特定机器人调好的。你的机器人可能更重、更矮、关节力矩范围不同——直接使用旧配置，轻则训练慢，重则策略崩溃。

**问题三：任务语义不同。** 速度跟踪任务的 reward 是追踪指令速度，但你的任务可能是抓取、平衡或导航——reward 结构完全不同，照搬没有意义。

> **本质洞察**：自定义环境搭建的核心困难不在于代码量——一个完整的环境配置通常只有 300-500 行代码。困难在于**每一行配置都与物理模型和任务语义紧密耦合**，任何一行的错误都会以"训练不收敛"的形式表现出来，而非编译错误或运行时异常。这就是为什么本章不是教你"怎么写代码"（这在前面的章节已经学过），而是教你"怎么系统性地确保每一步都正确"。

### 本章的方法论：分治-验证-集成

本章采用的核心方法论是**分治-验证-集成**（Divide-Verify-Integrate, DVI）：

1. **分治**：把环境搭建拆成独立的模块——模型验证、action 验证、obs 验证、reward 验证、termination 验证
2. **验证**：每个模块都有独立的验证方法（不需要训练策略就能检查正确性）
3. **集成**：模块逐步组合，每加一个模块跑一次 smoke test

这个方法论在 Ch21 的"五步验证法"中已经初步体现。本章把它发展为一套完整的、可在双框架中通用的工程流程。

> **跨领域类比**：DVI 方法论就像硬件工程中的"板级测试"。你不会把所有芯片焊到 PCB 上才通电测试——你先测试电源模块能否输出正确电压，再测试时钟信号是否稳定，然后逐步焊接其他芯片。每焊一个芯片就重新测试——如果新芯片导致系统不工作，错误一定出在最后焊的那个芯片上。类比到 RL 环境：每加一个 Manager 配置就重新跑 smoke test——如果环境崩溃，错误一定在最后加的那个配置中。这个类比的边界在于：PCB 测试有明确的"通过/不通过"判据（电压在阈值内），而 RL 环境的"正确性"有时需要更多领域判断（比如"random agent 的 reward 方差应该多大"）。

### ⚠️ 常见陷阱

⚠️ **思维陷阱：认为"环境搭建是一次性工作"**
- 错误想法：花两天搭好环境，后面只需要调超参
- 实际上：环境中的细微问题（obs 归一化错误、reward 量纲不对、DR 范围不合理）可能在训练数千 iterations 后才暴露。环境搭建是一个持续验证和迭代的过程
- 正确做法：把环境验证脚本作为 CI 的一部分——每次修改环境后自动运行 smoke test

⚠️ **编程陷阱：不在 git 中管理环境配置**
- 错误做法：在 Jupyter notebook 或交互式 shell 中修改配置、运行训练
- 后果：三天后你不记得改了什么，无法复现之前的结果
- 正确做法：每次环境修改都 commit + WandB 自动记录 git hash（AGILE 的标准流程）

⚠️ **概念误区：认为"MuJoCo 中能跑就够了"**
- 错误想法：先在 MuJoCo 中把环境搞对，Isaac Lab 版本以后再说
- 实际上：双框架的 API 差异可能导致同一个"正确"的 obs 配置在另一个框架中行为不同（如 quaternion 顺序、frame 约定）
- 正确做法：从一开始就维护双框架版本，用 sim2sim 交叉验证

### 练习

1. **[回顾题]** 列出 Ch13-Ch21 中每个实战章节使用的内置环境名称，标注哪些是 mjlab 内置、哪些是 Isaac Lab 内置。
2. **[设计题]** 你实验室有一个六自由度机械臂（不是 YAM，关节名完全不同）。如果你直接复制 Ch17 的 `lift_cube_env_cfg`，列出至少 5 个需要修改的配置项和修改原因。
3. **[思考题]** 为什么说"训练不收敛"是最难排查的 bug 类型？从 MDP 的四个组件（S, A, R, T）分析可能的错误来源。

---

DVI 方法论确立了"先验证模块、再集成系统"的工程原则。但具体怎么在 mjlab 中创建一个全新任务？从哪个文件开始写？配置项之间的依赖关系是什么？这正是下节的主题。

---


## 22.2 mjlab 自定义任务全流程 ⭐⭐⭐

> **这一节解决什么问题**：在 mjlab 中从零创建一个完整的自定义 RL 环境，掌握 EntityCfg → SceneCfg → ManagerBasedRlEnvCfg → Registry 的全流程。

### 动机：理解 mjlab 的"配置即环境"哲学

mjlab 的环境设计哲学是**配置即环境**——你不需要继承任何环境基类或重写 `step()` 函数。整个环境由一系列 dataclass 配置组成，框架根据配置自动编排物理仿真、obs 计算、reward 计算、termination 检查和 reset 逻辑。

这意味着创建一个新环境本质上是**填写一系列配置表**。但配置表之间有严格的依赖关系：action 配置引用 robot entity 的关节名，obs 配置引用 scene 中各 entity 的状态，reward 配置引用 obs 计算的中间结果。理解这些依赖关系是正确填写配置的前提。

回顾 Ch04 中介绍的 mjlab 九大 Manager：ObservationManager、ActionManager、RewardManager、TerminationManager、EventManager、CommandManager、CurriculumManager、MetricsManager、RecorderManager。在自定义任务中，前七个是必须配置的（MetricsManager 和 RecorderManager 有合理的默认值）。

> **双重解读**：Manager-Based 架构可以从两个完全不同的角度理解。**从软件工程的角度**，它是一个依赖注入（Dependency Injection）框架——每个 Manager 是一个可替换的组件，框架（容器）负责按正确顺序调用它们。你不需要关心调用顺序，只需要提供正确的配置。**从强化学习的角度**，它是 MDP 形式化的直接映射——ObservationManager 计算 $s_t$，ActionManager 施加 $a_t$，RewardManager 计算 $r_t$，TerminationManager 判定 $d_t$，EventManager 处理 $\rho_0$（初始状态分布）和环境参数的随机化。这两个视角的交汇点在于：好的 MDP 设计恰好也是好的模块化设计——obs、reward、action 的独立性既是 MDP 的数学性质（reward 不应该依赖 action 空间的具体实现），也是软件工程的最佳实践（低耦合高内聚）。理解这个交汇点，可以帮助你在面对复杂的自定义环境时做出正确的设计决策。

### 如果跳过某个 Manager 会怎样

**反事实推理：如果不配置 TerminationManager 会怎样？** 环境永远不会 reset——episode 长度趋向无穷。PPO 的 GAE 计算依赖 episode 边界来截断 trajectory，无穷长的 episode 导致 return 估计方差极大，训练不稳定。更严重的是，物理状态可能逐渐漂移到不合理的区域（关节超限、穿模、NaN），最终导致仿真崩溃。

**反事实推理：如果不配置 EventManager 会怎样？** 每个 episode 的初始状态完全相同。策略会 overfit 到这个特定的初始状态——换一个稍微不同的初始位姿，策略就失效。这在固定基座操作中尤其严重：物体每次都在同一个位置，策略学到了一个固定的抓取轨迹而不是泛化的抓取能力。

### 全流程六步法 ⭐⭐

以下用一个"自定义四足速度跟踪"任务作为贯穿案例。假设你有一个非标四足机器人（不是 Go2/ANYmal），MJCF 文件已经准备好（来自 Ch11 的 sw2urdf 流程）。

**Step 1：定义 Robot EntityCfg**

EntityCfg 告诉框架"这个机器人的 MJCF 在哪、初始状态是什么、关节和 body 怎么分组"：

```python
# === Step 1: Robot Entity 配置 ===
# 文件: my_robot/robot_cfg.py

from mjlab.envs import EntityCfg
from mjlab.actuators import ImplicitActuatorCfg

MY_QUADRUPED_CFG = EntityCfg(
    # MJCF 文件路径（相对于 mjlab asset zoo 或绝对路径）
    spawn=MjCfg(
        mjcf_path="my_robot/xmls/my_quad.xml",
    ),
    # 初始状态
    init_state=EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.45),  # 初始高度（需要高于地面+腿长）
        joint_pos={
            ".*_hip_joint": 0.0,       # 所有 hip 关节初始角度
            ".*_thigh_joint": 0.7,     # 所有 thigh 关节
            ".*_calf_joint": -1.4,     # 所有 calf 关节
        },
    ),
    # Actuator 配置
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*"],   # 正则匹配所有关节
            stiffness=25.0,            # PD 刚度 (Nm/rad)
            damping=0.5,               # PD 阻尼 (Nm·s/rad)
            velocity_limit=20.0,       # 关节速度上限 (rad/s)
            effort_limit=33.5,         # 力矩上限 (Nm)
        ),
    },
)
```

**关键决策点——初始高度**：`pos` 的 z 分量必须大于机器人站立时脚到 base 的距离。如果设太低，初始时刻腿部穿入地面，MuJoCo 的接触求解器会产生巨大的弹出力，机器人被弹飞。如果设太高，机器人自由落体，落地时的冲击可能触发 termination。正确的做法：在 MuJoCo viewer 中手动测试——加载 MJCF，用 `mj_step` 运行几步，观察机器人是否稳定站立。

**关键决策点——joint_pos 初始值**：使用 `".*_hip_joint": 0.0` 这种正则表达式匹配是 mjlab 的标准模式。但注意：如果你的 MJCF 中关节名不遵循 `prefix_type_joint` 的命名惯例（比如用了 `j1, j2, j3`），正则表达式需要相应调整。

**验证 Step 1**：

```python
# 验证 robot entity（不需要完整环境）
import mujoco
from mjlab.utils import load_entity

entity = load_entity(MY_QUADRUPED_CFG)
model = entity.model
data = entity.data

print(f"Bodies: {model.nbody}")
print(f"Joints: {model.njnt}")
print(f"Actuators: {model.nu}")
print(f"Joint names: {[model.joint(i).name for i in range(model.njnt)]}")

# 验证初始状态
mujoco.mj_step(model, data)
print(f"Base height after 1 step: {data.qpos[2]:.3f}")
assert data.qpos[2] > 0.1, "Robot fell through ground!"
assert not any(np.isnan(data.qpos)), "NaN in initial state!"
```

**Step 2：定义 SceneCfg**

SceneCfg 把所有 entity（机器人、地面、物体等）组合成一个场景：

```python
# === Step 2: Scene 配置 ===
# 文件: my_robot/scene_cfg.py

from mjlab.scene import InteractiveSceneCfg

class MyQuadSceneCfg(InteractiveSceneCfg):
    """自定义四足场景。"""
    # 环境间距（多环境并行时每个 env 的空间范围）
    env_spacing = 3.0

    # 机器人
    robot = MY_QUADRUPED_CFG.replace(
        prim_path="/World/envs/env_.*/Robot"
    )

    # 地面
    ground = EntityCfg(
        spawn=MjCfg(mjcf_path="terrain/flat_ground.xml"),
    )

    # 可选：地形（Ch13 中的 terrain curriculum）
    # terrain = TerrainCfg(...)
```

`env_spacing` 决定了多环境并行时相邻 env 的间距。如果间距太小，不同 env 的机器人可能碰撞；太大则浪费 GPU 内存。经验值：机器人最大尺寸的 3-5 倍。

**Step 3：定义 ActionsCfg**

```python
# === Step 3: Action 配置 ===
# 文件: my_robot/actions_cfg.py

from mjlab.envs.mdp import JointPositionActionCfg

class ActionsCfg:
    """四足关节位置控制动作。"""
    joint_pos = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],    # 匹配所有关节
        scale=0.25,            # action ∈ [-1,1] → 实际增量 ∈ [-0.25, 0.25] rad
        use_default_offset=True,  # action=0 对应初始关节角
    )
```

**scale 选择的工程经验**：`scale` 太大（如 1.0 rad），random agent 会产生剧烈的关节运动，可能立即触发 termination；`scale` 太小（如 0.01 rad），策略的表达能力被限制，无法完成需要大幅关节运动的任务。四足速度跟踪的经验值是 0.2-0.3 rad，人形是 0.1-0.2 rad（人形关节范围更小），操作任务可能需要更大的 scale（0.5-1.0 rad，因为臂关节范围大）。

**Step 4：定义 ObservationsCfg**

```python
# === Step 4: Observation 配置 ===
# 文件: my_robot/obs_cfg.py

from mjlab.envs.mdp import *

class ObservationsCfg:
    class PolicyCfg(ObservationGroupCfg):
        """Actor observation（部署时可用）。"""
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)         # 3
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)         # 3
        proj_gravity = ObsTerm(func=mdp.projected_gravity)    # 3
        velocity_commands = ObsTerm(func=mdp.generated_commands,
                                    params={"command_name": "base_velocity"})  # 3
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)           # N_joints
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)           # N_joints
        actions = ObsTerm(func=mdp.last_action)               # N_joints

    class CriticCfg(ObservationGroupCfg):
        """Critic observation（privileged 信息）。"""
        enable_corruption = False
        # 包含 PolicyCfg 的所有 obs + 额外的 privileged obs
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        proj_gravity = ObsTerm(func=mdp.projected_gravity)
        velocity_commands = ObsTerm(func=mdp.generated_commands,
                                    params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        # Privileged: 真实 base 速度、地形高度、摩擦系数
        true_base_lin_vel = ObsTerm(func=mdp.root_lin_vel_w)  # 3
        friction_coeffs = ObsTerm(func=mdp.friction_coefficients)  # per-foot
```

**为什么 PolicyCfg 和 CriticCfg 要分开定义（而不是继承）**：在 Python dataclass 继承中，子类修改父类字段的顺序可能导致 field 顺序不一致，而 obs 的维度拼接对顺序敏感。mjlab 的推荐做法是显式列出每个 group 的所有 obs term，即使有重复——这保证了 actor 和 critic 的 obs 维度在任何修改下都是可预测的。

**Step 5：定义 RewardsCfg 和 TerminationsCfg**

```python
# === Step 5: Reward 和 Termination 配置 ===
# 文件: my_robot/rewards_cfg.py

class RewardsCfg:
    """四足速度跟踪奖励。"""
    # Tracking
    lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=0.75,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    # Regularization
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-0.0002)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    # Contact
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.125,
        params={"sensor_cfg": ..., "threshold": 0.5},
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": ..., "threshold": 1.0},
    )

class TerminationsCfg:
    """四足终止条件。"""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": ...,
            "threshold": 1.0,
        },
    )
```

**Reward 权重的初始设定经验法则**：

| 类别 | 典型权重范围 | 调节原则 |
|------|------------|---------|
| Tracking (正) | 0.5 - 3.0 | 占总正 reward 的 60-80% |
| Regularization (负) | -0.001 - -0.1 | 不应超过 tracking reward 的 20% |
| Contact (正/负) | 0.05 - 1.0 | 不让触地惩罚压过 tracking |
| Style (正) | 0.01 - 0.5 | 最后添加，微调行为质量 |

一个实用技巧：先只用 tracking reward 训练（其他项 weight=0），确认策略能移动。然后逐步打开 regularization 项，每次只加一项，观察 reward 曲线和行为变化。这样可以精确定位每个 reward 项对行为的影响。

**Step 6：组装 EnvCfg 并注册**

```python
# === Step 6: 环境配置组装和注册 ===
# 文件: my_robot/env_cfg.py

from mjlab.envs import ManagerBasedRlEnvCfg

class MyQuadEnvCfg(ManagerBasedRlEnvCfg):
    """自定义四足速度跟踪环境配置。"""
    # Scene
    scene = MyQuadSceneCfg(num_envs=4096, env_spacing=3.0)

    # MDP 组件
    observations = ObservationsCfg()
    actions = ActionsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventsCfg()       # DR 配置（详见 Ch08）
    commands = CommandsCfg()     # 速度指令配置
    curriculum = CurriculumCfg() # 可选

    # 仿真参数
    sim = SimCfg(
        dt=0.005,              # 仿真步长 5ms
        decimation=4,          # 每 4 个仿真步输出一个 obs → 策略频率 50 Hz
    )
    episode_length_s = 20.0    # 20 秒一个 episode

# === 注册到 task registry ===
# 文件: my_robot/__init__.py

from mjlab.envs import register_task

register_task(
    task_id="MyQuad-Velocity-Flat",
    entry_point="mjlab.envs:ManagerBasedRlEnv",
    env_cfg_entry_point="my_robot.env_cfg:MyQuadEnvCfg",
    rsl_rl_cfg_entry_point="my_robot.train_cfg:MyQuadPPOCfg",
)
```

注册完成后，就可以用标准的训练命令启动：

```bash
# 训练
python scripts/train.py MyQuad-Velocity-Flat --env.scene.num-envs=4096

# 可视化
python scripts/play.py MyQuad-Velocity-Flat --num-envs=1
```

### 验证三步走 ⭐⭐⭐

环境注册后，**不要立即开始训练**。先执行三步验证，确保 MDP 的每个组件都工作正常：

**第一步：Smoke Test（10 秒）**

目的：环境能否成功创建和 reset，不报错。

```python
# smoke_test.py
from mjlab.envs import make

env = make("MyQuad-Velocity-Flat", num_envs=4)
obs, info = env.reset()
print(f"✅ Obs shape: {obs['policy'].shape}")
print(f"✅ Action shape: {env.action_space.shape}")

# 运行 10 步
for _ in range(10):
    action = torch.zeros(4, env.action_space.shape[-1])
    obs, reward, terminated, truncated, info = env.step(action)
    assert not torch.isnan(obs['policy']).any(), "NaN in obs!"
    assert not torch.isnan(reward).any(), "NaN in reward!"

print("✅ Smoke test passed!")
env.close()
```

**通过标准**：不报错，obs 和 reward 不含 NaN。如果 smoke test 失败，最常见的原因是 MJCF 编译错误或 joint name 不匹配。

**第二步：Zero Agent（30 秒）**

目的：在 action=0 的情况下（机器人保持初始姿态，因为 `use_default_offset=True`），观察物理行为是否合理。

```python
# zero_agent_test.py
env = make("MyQuad-Velocity-Flat", num_envs=16)
obs, info = env.reset()
rewards_history = []

for step in range(200):  # 2 秒（假设 50 Hz）
    action = torch.zeros(16, env.action_space.shape[-1])
    obs, reward, terminated, truncated, info = env.step(action)
    rewards_history.append(reward.mean().item())

    if step == 0:
        base_height = obs['policy'][:, 6].mean().item()  # 投影重力 z 分量
        print(f"Step 0 base_height proxy: {base_height:.3f}")

# 检查
print(f"Mean reward: {np.mean(rewards_history):.4f}")
print(f"Reward std: {np.std(rewards_history):.4f}")
print(f"Any terminated: {terminated.any().item()}")
```

**通过标准**：
- 机器人应该在 200 步内**不倒下**（terminated 全为 False）。如果倒了，说明初始关节角设置不对——机器人在 action=0 的姿态下无法站立
- reward 应该是**小负值**（tracking reward 接近零因为机器人不移动，regularization penalty 接近零因为没有动作，但 contact reward 可能非零）
- base 高度应该**基本不变**（允许几毫米的下沉，因为阻尼会让关节略微松弛）

> **本质洞察**：Zero agent 是自定义环境最有价值的诊断工具。如果机器人在 action=0 时都站不稳，任何 RL 算法都不可能学到有用的策略——因为 action=0 是策略探索的起点，如果起点就不稳定，探索的 baseline 太差，PPO 无法获得有效的梯度信号。

**第三步：Random Agent（2 分钟）**

目的：随机动作下，reward 有方差、termination 有触发、reset 能正常工作。

```python
# random_agent_test.py
env = make("MyQuad-Velocity-Flat", num_envs=64)
obs, info = env.reset()

ep_rewards = []
ep_lengths = []
current_ep_reward = torch.zeros(64)
current_ep_length = torch.zeros(64)

for step in range(2000):  # 20 秒
    action = torch.randn(64, env.action_space.shape[-1])
    obs, reward, terminated, truncated, info = env.step(action)

    current_ep_reward += reward
    current_ep_length += 1

    # 记录完成的 episode
    done = terminated | truncated
    if done.any():
        ep_rewards.extend(current_ep_reward[done].tolist())
        ep_lengths.extend(current_ep_length[done].tolist())
        current_ep_reward[done] = 0
        current_ep_length[done] = 0

print(f"Completed episodes: {len(ep_rewards)}")
print(f"Avg episode reward: {np.mean(ep_rewards):.2f} ± {np.std(ep_rewards):.2f}")
print(f"Avg episode length: {np.mean(ep_lengths):.1f} ± {np.std(ep_lengths):.1f}")
print(f"Min/Max reward: {np.min(ep_rewards):.2f} / {np.max(ep_rewards):.2f}")
```

**通过标准**：
- 完成的 episode 数 > 0（说明 termination 在工作）
- episode 长度有方差（说明不同随机动作导致不同存活时间）
- reward 有方差（说明 reward 对动作有区分度）
- episode reward 应该是**负值**（随机策略不应该获得正的 tracking reward）

如果 random agent 的 reward 总是相同的常数，说明 reward 函数没有和动作关联——可能是 obs 中的状态量没有被 reward 使用，或者 reward 函数引用了错误的变量。

### 自定义 Reward/Obs Term 的编写 ⭐⭐

当框架内置的 reward/obs term 无法满足需求时，你需要编写自定义 term。以下是完整的编写模式：

```python
# === 自定义 Reward Term ===

def my_custom_tracking_reward(
    env,
    asset_cfg: EntityCfg,
    command_name: str,
    std: float = 0.25,
) -> torch.Tensor:
    """自定义速度跟踪奖励。

    Args:
        env: 环境实例
        asset_cfg: 机器人 entity 配置（用于索引）
        command_name: 速度指令的名称
        std: 高斯核标准差

    Returns:
        [B] tensor, 每个 env 的 reward 值
    """
    # 获取实际速度（base frame）
    actual_vel = env.robot.data.root_link_lin_vel_b[:, :2]
    # 获取指令速度
    cmd_vel = env.command_manager.get_command(command_name)[:, :2]
    # 计算误差
    error = torch.norm(actual_vel - cmd_vel, dim=-1)
    # 高斯核 reward
    reward = torch.exp(-error ** 2 / std ** 2)
    return reward

# 注册到 RewardsCfg
class RewardsCfg:
    my_tracking = RewTerm(
        func=my_custom_tracking_reward,
        weight=2.0,
        params={
            "asset_cfg": EntityCfg(name="robot"),
            "command_name": "base_velocity",
            "std": 0.25,
        },
    )
```

**自定义 term 的命名约定**：函数名应该描述"计算什么"而不是"在哪个任务中用"。好的命名：`track_lin_vel_xy_exp`、`feet_air_time`、`base_height_penalty`。坏的命名：`my_reward`、`task1_bonus`、`reward_v2`。好的命名让 term 可以在不同任务间复用。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：joint_names 正则表达式不匹配**
- 错误做法：直接用 `[".*"]` 匹配所有关节，但 MJCF 中还有 freejoint（6-DOF 基座关节）
- 后果：action 维度包含了 freejoint 的 6 维，策略直接控制底盘位姿——训练出的策略无意义
- 正确做法：用精确的正则 `["FL_.*", "FR_.*", "HL_.*", "HR_.*"]` 或 `["(?!root).*"]`（排除 root joint）
- 验证：打印 `env.action_manager.action_term_dim` 确认维度

⚠️ **编程陷阱：sim.dt 和 decimation 设置不合理**
- 错误做法：dt=0.01, decimation=1 → 策略频率 100 Hz
- 后果：策略频率太高，每步之间状态变化极小，reward 信号极弱，PPO 难以学到有效更新
- 正确做法：dt=0.002~0.005, decimation=4~10 → 策略频率 20-50 Hz。策略频率应该和真机部署频率匹配

⚠️ **思维陷阱：一次性配置所有 Manager**
- 错误做法：把 7 个 Manager 的配置全部写完再跑 smoke test
- 后果：如果 smoke test 失败，不知道是哪个 Manager 的配置有问题
- 正确做法：按 DVI 流程逐步添加——先只配置 scene + action（其他 Manager 用默认值），跑通后加 obs，再加 reward，每步验证

⚠️ **编程陷阱：RewTerm 的 func 返回了错误形状**
- 错误做法：reward 函数返回 `[B, 1]` 而不是 `[B]`
- 后果：RewardManager 内部 squeeze 失败或维度广播出错
- 正确做法：所有 reward/termination term 的返回值必须是 `[B]` 形状（一维 tensor，长度等于 num_envs）

⚠️ **编程陷阱：observation 中的 base_lin_vel 使用了 world frame**
- 错误做法：用 `root_link_lin_vel_w`（world frame 速度）作为 obs
- 后果：底盘朝向改变时，相同的前进速度在 world frame 中有不同的表示——obs 变得不稳定
- 正确做法：用 `root_link_lin_vel_b`（body frame 速度）——前进方向始终是 x 轴，与朝向无关

### 练习

1. **[实践题]** 按照六步法，为一个简单的 inverted pendulum（倒立摆）在 mjlab 中创建自定义环境。action 是 cart 的力，obs 是 pole 角度和角速度，reward 是 pole 保持直立的时间。跑通 smoke test + zero agent。
2. **[调试题]** 你创建了一个四足环境，smoke test 通过，但 zero agent 测试中机器人在第一步就"弹飞"（base height > 5m）。列出三个可能的原因和排查步骤。
3. **[跨章综合题]** 结合 Ch08 的 DR 配置，为你的自定义四足环境添加三项 DR（mass、friction、joint damping），并设计一个验证实验：先在无 DR 下训练 baseline，再加 DR 训练，用 sim2sim 对比两个策略在不同物理参数下的鲁棒性。

---

mjlab 的全流程展示了"配置即环境"的开发模式。但如果你需要使用 Isaac Lab 的特性（RTX 渲染、USD 场景、多 RL 后端），或者你的合作者使用 Isaac Lab，你需要在 Isaac Lab 中创建等价的环境。Isaac Lab 的推荐开发模式是 extension——下节以 HOVER 仓库为模板，讲解如何创建一个独立的 Isaac Lab 环境。

---


## 22.3 Isaac Lab Extension 开发模式 ⭐⭐⭐

> **这一节解决什么问题**：在 Isaac Lab 中创建独立的 extension 项目，遵循 HOVER 仓库的目录范式，理解 Manager-based 和 Direct workflow 的选择依据。

### 动机：为什么要用 extension 模式

Isaac Lab 的环境开发有两种模式：(1) **内部模式**——把代码直接写在 `isaac-sim/IsaacLab` 仓库内部；(2) **Extension 模式**——创建独立的 Python 包，作为 Isaac Lab 的外部扩展加载。

Extension 模式是**推荐的生产模式**，原因有三：
- **版本隔离**：你的代码不和 Isaac Lab 主仓库耦合，升级 Isaac Lab 时不需要合并冲突
- **协作友好**：团队成员可以独立开发不同的 extension，互不影响
- **可发布**：extension 可以打包成 pip 包发布（HOVER 就是这样做的）

回顾 writing_guide.md §6.1 中的 HOVER 项目：HOVER（ICRA 2025，NVIDIA + CMU + UT Austin）是 Isaac Lab extension 的标杆范例。其仓库结构清晰地展示了"如何为一个研究项目组织 Isaac Lab 代码"。

### HOVER 仓库结构精读 ⭐⭐

HOVER 的仓库 `NVlabs/HOVER` 的顶层目录结构如下：

```
NVlabs/HOVER/
├── neural_wbc/                 # 核心 Python 包
│   ├── core/                   # 环境 + Oracle 策略逻辑
│   │   ├── envs/               # Isaac Lab 环境定义
│   │   ├── tasks/              # 任务配置（obs/action/reward）
│   │   └── oracles/            # Oracle teacher 策略
│   ├── isaac_lab_wrapper/      # Isaac Lab 特定的 wrapper
│   ├── mujoco_wrapper/         # MuJoCo sim2sim wrapper
│   ├── hw_wrappers/            # 真机部署 wrapper
│   ├── inference_env/          # 推理环境（ONNX 加载）
│   └── data/
│       ├── motions/            # AMASS 动作片段
│       └── policy/             # 预训练 checkpoint
├── scripts/
│   └── rsl_rl/
│       ├── train_teacher_policy.py
│       ├── train_student_policy.py
│       ├── play.py
│       └── eval.py
├── third_party/
│   └── human2humanoid/         # 第三方子模块
├── setup.py                    # pip install -e .
└── pyproject.toml
```

**核心设计决策**：

| 决策 | HOVER 的选择 | 理由 |
|------|-------------|------|
| 代码组织 | 按功能分（core/wrapper/inference） | 同一个 task 的训练/评估/部署代码分离 |
| 环境基类 | Manager-based | 多 obs group（oracle/student）、多 action term |
| RL 后端 | RSL-RL（定制 fork） | PPO 足够，且与部署链兼容 |
| 配置方式 | Hydra-style YAML | 灵活覆盖超参而不修改源码 |
| 部署 | ONNX export + MuJoCo sim2sim + hw_wrapper | 完整的三阶段部署链 |

### 从零创建 Isaac Lab Extension ⭐⭐

Isaac Lab 从 2025 版开始提供了 **template generator**，可以快速生成 extension 骨架。但为了理解每个文件的作用，这里手动创建：

**Step 1：创建目录结构**

```bash
mkdir -p my_robot_lab/
cd my_robot_lab/

# 创建 Python 包结构
mkdir -p my_robot_lab/envs
mkdir -p my_robot_lab/tasks/locomotion
mkdir -p scripts/rsl_rl
touch my_robot_lab/__init__.py
touch my_robot_lab/envs/__init__.py
touch my_robot_lab/tasks/__init__.py
touch my_robot_lab/tasks/locomotion/__init__.py
```

**Step 2：定义 Scene**

```python
# my_robot_lab/tasks/locomotion/scene_cfg.py

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg

MY_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="my_robot_lab/assets/my_quad.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.45),
        joint_pos={
            ".*_hip_joint": 0.0,
            ".*_thigh_joint": 0.7,
            ".*_calf_joint": -1.4,
        },
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
            stiffness=25.0,
            damping=0.5,
        ),
    },
)

class MyQuadSceneCfg(InteractiveSceneCfg):
    """Isaac Lab 版本的自定义四足场景。"""
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    robot = MY_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )
```

**mjlab vs Isaac Lab 的 EntityCfg 对比**：

| 配置项 | mjlab `EntityCfg` | Isaac Lab `ArticulationCfg` |
|--------|-------------------|---------------------------|
| 模型格式 | MJCF (XML) | USD |
| 路径 | `mjcf_path` | `usd_path` |
| 碰撞传感器 | 自动（MuJoCo 原生） | 需显式 `activate_contact_sensors=True` |
| 刚体属性 | MJCF 内定义 | `RigidBodyPropertiesCfg` |
| 求解器参数 | MJCF 全局设置 | `ArticulationRootPropertiesCfg` 每个 articulation 独立 |
| Actuator | `ImplicitActuatorCfg` | `ImplicitActuatorCfg`（API 名相同，内部实现不同） |

**Step 3：定义 MDP 组件**

Isaac Lab 的 MDP 配置结构与 mjlab 高度对称——两者共享 `ObsTerm`、`RewTerm`、`DoneTerm` 等概念，但具体 API 名略有差异：

```python
# my_robot_lab/tasks/locomotion/env_cfg.py

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    TerminationTermCfg,
    EventTermCfg,
)
import isaaclab.envs.mdp as mdp

class ObservationsCfg:
    """Isaac Lab 版 obs 配置。"""
    class PolicyCfg(ObservationGroupCfg):
        base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel)
        base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity)
        velocity_commands = ObservationTermCfg(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel)
        actions = ObservationTermCfg(func=mdp.last_action)

    class CriticCfg(ObservationGroupCfg):
        enable_corruption = False
        # ... 同 Policy + privileged obs ...

class RewardsCfg:
    track_lin_vel_xy = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    track_ang_vel_z = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp,
        weight=0.75,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    # Regularization
    lin_vel_z_l2 = RewardTermCfg(func=mdp.lin_vel_z_l2, weight=-2.0)
    action_rate_l2 = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)

class TerminationsCfg:
    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)

class MyQuadEnvCfg(ManagerBasedRLEnvCfg):
    """Isaac Lab 版完整环境配置。"""
    scene = MyQuadSceneCfg(num_envs=4096, env_spacing=3.0)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
```

**Step 4：注册环境**

```python
# my_robot_lab/tasks/locomotion/__init__.py
import gymnasium as gym

gym.register(
    id="MyQuad-Velocity-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": "my_robot_lab.tasks.locomotion.env_cfg:MyQuadEnvCfg",
        "rsl_rl_cfg_entry_point": "my_robot_lab.tasks.locomotion.agents:rsl_rl_ppo_cfg",
    },
    disable_env_checker=True,
)
```

**Step 5：安装和训练**

```bash
# 安装 extension（开发模式）
pip install -e .

# 训练
python -m isaaclab.app --task MyQuad-Velocity-Flat-v0 \
    --headless --num_envs 4096

# 或使用 Isaac Lab 的标准脚本
python scripts/rsl_rl/train.py --task MyQuad-Velocity-Flat-v0
```

### Manager-Based vs Direct Workflow 选择 ⭐⭐

Isaac Lab 提供两种环境开发 workflow。选择哪种取决于任务特征：

| 维度 | Manager-Based | Direct |
|------|--------------|--------|
| **代码组织** | 分散在多个 Cfg dataclass | 集中在一个类中 |
| **模块复用** | 高（obs/reward term 跨任务共享） | 低（逻辑内嵌于 env 类） |
| **灵活性** | 中（受限于 Manager API） | 高（完全控制 step 逻辑） |
| **学习成本** | 高（需理解九大 Manager） | 低（一个类搞定） |
| **协作** | 好（团队成员独立开发不同 term） | 差（修改同一个文件） |
| **JIT 优化** | 受限（Manager 间调度有 Python 开销） | 可以对整个 step JIT trace |
| **适用任务** | 标准 RL 任务（locomotion/manipulation） | 需要复杂 step 逻辑的任务（multi-agent/hierarchical） |
| **代表项目** | HOVER、ExBody、基本所有 Isaac Lab 内置任务 | Legged Lab（Direct for legged robots） |

> **跨领域类比**：Manager-Based vs Direct 的关系就像 React 的组件化开发 vs 原生 JavaScript。React 的组件系统让代码更模块化、更易维护，但有运行时开销（virtual DOM diff）；原生 JS 更灵活但维护成本高。对于大多数 Web 应用，React 是更好的选择；只有在极端性能需求下才需要回退到原生 JS。同理，对于大多数 RL 任务，Manager-Based 是更好的选择；只有在需要极端 step 性能或非标准 MDP 结构时才需要 Direct。

**决策流程图**：

```
你的任务是否需要非标准的 step 逻辑？
（如 multi-agent turn-taking、hierarchical 策略切换、自定义物理回调）
│
├── 是 → Direct Workflow
│
└── 否 → 你的团队是否 > 2 人协作？
    │
    ├── 是 → Manager-Based（强烈推荐）
    │
    └── 否 → Manager-Based（推荐，除非你需要 JIT 全图优化）
```

对于本教材的绝大多数任务（速度跟踪、操作、移动操作），Manager-Based 是正确选择。HOVER、ExBody、unitree_rl_lab 等主流项目都使用 Manager-Based。

### URDF/MJCF → USD 转换 ⭐⭐

如果你已经有 MJCF 模型（来自 Ch11 的 sw2urdf → MJCF 流程），需要转换为 USD 格式才能在 Isaac Lab 中使用。Isaac Lab 提供了两个转换工具：

```bash
# URDF → USD
python -m isaaclab.app --headless \
    -p scripts/tools/convert_urdf.py \
    --input_path my_robot.urdf \
    --output_path my_robot.usd

# MJCF → USD（需要 MuJoCo USD exporter）
python -m isaaclab.app --headless \
    -p scripts/tools/convert_mjcf.py \
    --input_path my_robot.xml \
    --output_path my_robot.usd
```

**转换后的验证清单**：

| 检查项 | 验证方法 | 常见问题 |
|--------|---------|---------|
| 关节数量 | 比较 MJCF 和 USD 的 joint count | URDF 中的 fixed joint 可能被省略或保留 |
| 质量/惯量 | 比较 total mass | 转换工具可能使用默认惯量 |
| 碰撞体 | 在 Isaac Sim viewer 中可视化碰撞体 | mesh 碰撞体可能被简化为凸包 |
| Actuator | 打印 actuator 列表 | MJCF 的 velocity actuator 需要手动映射到 `ImplicitActuatorCfg` |
| Frame 约定 | 比较初始状态下的 base 位姿 | MuJoCo 用 z-up，PhysX 也用 z-up，但某些 URDF 可能是 y-up |

> **反事实推理：如果跳过转换验证会怎样？** 一个常见问题是 URDF → USD 转换时 mesh 碰撞体被简化为凸包——原来的凹面（如机器人底盘内部的空腔）变成了凸面，导致本不应该碰撞的 body 产生了接触。在训练中的表现是：机器人的某些关节角度范围被"墙"挡住了（凸包碰撞），策略学到的运动范围比预期小。这类 bug 极难通过 reward 曲线发现——你只会看到"策略的步幅不够大"，但真正的原因是碰撞体形状错误。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Isaac Lab 版本不兼容**
- HOVER 要求 Isaac Lab v2.0.0 + Isaac Sim 4.5。如果你用了更新的版本，可能遇到 `rsl_rl` → `rsl_rl_lib` 的重命名问题
- 正确做法：检查 extension 的 README 中指定的版本要求，用 conda 环境隔离不同版本

⚠️ **编程陷阱：`{ENV_REGEX_NS}` 占位符遗漏**
- 在 SceneCfg 中，robot 的 `prim_path` 必须包含 `{ENV_REGEX_NS}`，它在运行时被替换为每个 env 的唯一路径
- 如果写成了固定路径（如 `/World/Robot`），所有 env 共享同一个机器人——物理完全混乱

⚠️ **思维陷阱：认为 Manager-Based 和 Direct 可以混用**
- 一个环境只能选择一种 workflow，不能在 Manager-Based 环境中插入 Direct-style 的自定义 step 逻辑
- 如果需要自定义 step 逻辑中的一小部分，优先考虑用自定义 Manager term 实现

### 练习

1. **[实践题]** Fork HOVER 仓库（`NVlabs/HOVER`），阅读其 `neural_wbc/core/envs/` 目录下的环境定义代码，画出 Oracle teacher 和 Student 的 obs 维度差异表。
2. **[对比题]** 分别用 Manager-Based 和 Direct workflow 实现一个简单的 CartPole 平衡任务。对比两种实现的代码行数、可读性和 steps/s 性能。
3. **[设计题]** 你要实现一个双机器人协作搬运任务（两个四足各抬箱子一端）。这个任务应该用 Manager-Based 还是 Direct？为什么？

---

双框架的环境创建流程已经清楚。但真实的 DIY 场景不是"从已有 MJCF 出发"——你可能需要从 CAD 模型开始，经过一系列转换步骤，才能得到可用的仿真模型。下节给出完整的端到端流程。

---


## 22.4 完整端到端流程：从 CAD 到策略训练 ⭐⭐⭐

> **这一节解决什么问题**：给出从机器人 CAD 设计到 RL 策略训练的完整九步流程，每一步有明确的输入/输出和验证标准。

### 动机：把零散的知识串成流水线

从 Ch11（SolidWorks → URDF）到 Ch12（Actuator 建模）再到 §22.2-22.3（环境创建），各个步骤分散在不同章节。实际项目中，这些步骤是一条流水线——前一步的输出是后一步的输入，任何一步出错都会在下游放大。本节把所有步骤串联，给出端到端的检查清单。

### 九步流水线

```
Step 1        Step 2        Step 3        Step 4
选构型 ─→ SolidWorks ─→ sw2urdf ─→ MJCF/USD
                建模         导出       调优

Step 5        Step 6        Step 7        Step 8       Step 9
双框架 ─→ 环境设计 ─→ 训练启动 ─→ sim2sim ─→ ONNX
接入                                验证       导出
```

### Step 1：选构型 ⭐

构型（morphology）选择决定了后续所有步骤的复杂度。以下决策表帮助你评估选择：

| 构型 | DOF | 接触模式 | 训练难度 | 建模难度 | 本书参考 |
|------|-----|---------|---------|---------|---------|
| 倒立摆 | 1-2 | 简单 | ⭐ | ⭐ | 入门练习 |
| 固定基座臂 | 6-7 | 中等 | ⭐⭐ | ⭐⭐ | Ch17 |
| 四足 | 12 | 复杂（步态） | ⭐⭐⭐ | ⭐⭐ | Ch13 |
| 轮式+臂 | 8-10 | 混合 | ⭐⭐⭐ | ⭐⭐⭐ | Ch21 |
| 人形 | 19-29 | 非常复杂 | ⭐⭐⭐⭐ | ⭐⭐⭐ | Ch14 |
| 人形+灵巧手 | 50+ | 极其复杂 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | Ch20 |

**经验法则**：你的第一个自定义环境应该选择 DOF $\le$ 12 的构型。DOF 越高，debug 的搜索空间越大——12 DOF 的四足已经有 $2^{12} = 4096$ 种"某个关节配置错误"的可能性。

### Step 2-3：SolidWorks → sw2urdf → URDF ⭐

这两步在 Ch11 中已经详细讲解。这里只列出从 Ch11 到 Ch22 的衔接检查清单：

| 检查项 | 标准 | 来源章节 |
|--------|------|---------|
| 所有 revolute joint 的旋转轴方向正确 | 在 RViz 中拖动每个 joint slider，观察旋转方向 | Ch11 §11.3 |
| 质量和惯量合理 | 总质量与真机称重误差 < 10% | Ch11 §11.4 |
| 碰撞体覆盖所有需要接触的表面 | 在 MuJoCo viewer 中开启碰撞体可视化 | Ch11 §11.5 |
| 没有自碰撞（在初始姿态下） | `mj_step` 后 `data.ncon == 0`（不含地面接触） | Ch12 §12.2 |
| Actuator effort 和 velocity 上限与 datasheet 匹配 | 比较 MJCF 中的 `ctrlrange` 和电机 spec | Ch12 §12.3 |

### Step 4：MJCF/USD 调优 ⭐⭐

从 URDF 转换来的 MJCF/USD 通常需要手动调优以下参数：

**（1）接触参数**

MuJoCo 的接触参数决定了碰撞行为的"软硬"和"弹跳"：

```xml
<!-- MJCF 接触参数调优 -->
<default>
  <geom condim="4"
        friction="0.8 0.005 0.0001"
        solref="0.005 1.0"
        solimp="0.9 0.95 0.001"/>
</default>
```

| 参数 | 含义 | 调优方向 |
|------|------|---------|
| `condim` | 接触约束维度（1=法向，3=+2D切向，4=+扭转） | 足式机器人用 4（需要扭转摩擦防旋转滑动） |
| `friction` | 切向摩擦、扭转摩擦、滚动摩擦 | 第一项 0.5-1.2 for 室内地面 |
| `solref` | 约束求解器参考频率和阻尼比 | timeconst $\approx$ 2$\times$dt，dampratio $\approx$ 1.0 |
| `solimp` | 约束穿透允许范围 | dmin=0.9, dmax=0.95（几乎不允许穿透） |

**（2）仿真步长**

```python
# 仿真步长选择的经验法则
dt = 0.002  # 2 ms → 500 Hz 物理仿真
decimation = 10  # 每 10 个物理步输出一个 obs → 50 Hz 策略

# 验证步长是否足够小：
# 1. 让机器人从 0.5m 高度自由落体
# 2. 检查落地后的弹跳次数和稳定时间
# 3. 如果弹跳过多（>3次）或不稳定（>1秒），减小 dt
```

**（3）Actuator 模型选择**

| Actuator 类型 | 控制模式 | 适用场景 | mjlab API | Isaac Lab API |
|--------------|---------|---------|-----------|--------------|
| Implicit PD | 位置目标 | 电机+减速器（大多数场景） | `ImplicitActuatorCfg` | `ImplicitActuatorCfg` |
| Explicit PD | 力矩输出 | 需要精确力矩控制 | — | `DCMotorCfg` |
| Velocity | 速度目标 | 轮式底盘 | `velocity` actuator | `ImplicitActuatorCfg(stiffness=0)` |
| Actuator Net | 学习模型 | 精确 sim2real | — | `ActuatorNetLSTMCfg` |

### Step 5：双框架接入 ⭐⭐

接入就是 §22.2（mjlab）和 §22.3（Isaac Lab）中描述的流程。两个框架可以并行进行，用以下对照表确保配置一致：

```python
# === 双框架配置一致性验证脚本 ===

def verify_dual_framework_consistency(mjlab_env, isaaclab_env):
    """验证两个框架的环境配置是否一致。"""
    # 1. 关节数
    mj_njnt = mjlab_env.robot.data.joint_pos.shape[-1]
    il_njnt = isaaclab_env.robot.data.joint_pos.shape[-1]
    assert mj_njnt == il_njnt, f"Joint count mismatch: mjlab={mj_njnt}, IsaacLab={il_njnt}"

    # 2. Action 维度
    mj_act = mjlab_env.action_space.shape[-1]
    il_act = isaaclab_env.action_space.shape[-1]
    assert mj_act == il_act, f"Action dim mismatch: mjlab={mj_act}, IsaacLab={il_act}"

    # 3. Obs 维度
    mj_obs = mjlab_env.observation_space["policy"].shape[-1]
    il_obs = isaaclab_env.observation_space["policy"].shape[-1]
    assert mj_obs == il_obs, f"Obs dim mismatch: mjlab={mj_obs}, IsaacLab={il_obs}"

    # 4. Zero action 后的 base 高度
    mj_obs_val, _ = mjlab_env.reset()
    il_obs_val, _ = isaaclab_env.reset()
    for _ in range(100):
        zero_act_mj = torch.zeros(1, mj_act)
        zero_act_il = torch.zeros(1, il_act)
        mj_obs_val, _, _, _, _ = mjlab_env.step(zero_act_mj)
        il_obs_val, _, _, _, _ = isaaclab_env.step(zero_act_il)

    mj_height = mjlab_env.robot.data.root_link_pos_w[0, 2].item()
    il_height = isaaclab_env.robot.data.root_link_pos_w[0, 2].item()
    print(f"Base height after 100 steps: mjlab={mj_height:.3f}, IsaacLab={il_height:.3f}")
    assert abs(mj_height - il_height) < 0.05, "Height mismatch > 5cm!"

    print("✅ Dual framework consistency verified!")
```

如果两个框架的 base height 差异超过 5cm，最常见的原因是：
- 接触摩擦参数不同（MJCF 和 PhysX 的摩擦模型不完全等价）
- 初始关节角的 offset 计算方式不同
- 求解器精度设置不同（MuJoCo 的 Newton solver vs PhysX 的 TGS solver）

### Step 6：环境设计 ⭐⭐

环境设计的核心是 MDP 四元组 $(S, A, R, T)$ 的工程化实现。这在 §22.2 中已经展示了代码模板。这里补充设计层面的决策框架：

**Observation 设计决策树**：

```
这个信息在真机上可获取吗？
│
├── 是 → 放入 PolicyCfg（actor obs）
│     该信息的噪声水平如何？
│     ├── 低噪声（编码器、IMU）→ 直接使用
│     └── 高噪声（视觉、触觉）→ 考虑滤波或 teacher-student
│
└── 否 → 只放入 CriticCfg（privileged obs）
      例如：真实摩擦系数、真实物体质量、完美 base velocity
```

**Reward 设计决策树**：

```
任务目标是否可以用距离度量？
│
├── 是 → 用 exp(-d²/σ²) 形式
│     目标距离的量级？
│     ├── < 0.1 m (精细操作) → σ ≈ 0.05-0.1
│     ├── 0.1-1 m (接近)     → σ ≈ 0.1-0.5
│     └── > 1 m (导航)       → σ ≈ 0.5-2.0
│
└── 否 → 用 boolean/stage reward
      例如：抓取成功（bool）、站立（bool）、连续步数（counter）
```

### Step 7：训练启动 ⭐

训练的标准流程（回顾 Ch07 的训练管线章节）：

```python
# RSL-RL PPO 训练配置
class MyQuadPPOCfg:
    seed = 42
    runner_type = "OnPolicyRunner"

    class policy:
        class_name = "ActorCritic"
        init_noise_std = 1.0
        actor_hidden_dims = [256, 256, 128]
        critic_hidden_dims = [256, 256, 128]
        activation = "elu"

    class algorithm:
        class_name = "PPO"
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4
        learning_rate = 1e-3
        schedule = "adaptive"  # 自适应 LR
        gamma = 0.99
        lam = 0.95
        max_grad_norm = 1.0

    class runner:
        num_steps_per_env = 24
        max_iterations = 10000
        save_interval = 500
        log_interval = 10
```

**训练启动后的前 5 分钟检查**：

| 时间点 | 检查内容 | 正常表现 | 异常处理 |
|--------|---------|---------|---------|
| 第 1 iteration | reward 数值 | -5 ~ -20（负值，因为正则惩罚） | 如果是 NaN → 检查 obs/reward 中的除零 |
| 前 10 iterations | entropy | 初始约 3-5（取决于 action 维度） | 如果 < 1 → init_noise_std 太小 |
| 前 50 iterations | reward 趋势 | 缓慢上升 | 如果完全平坦 → reward 没有和动作关联 |
| 前 100 iterations | episode length | 逐渐增长 | 如果始终很短 → termination 太严格 |
| 前 500 iterations | KL divergence | 在 target 附近波动 | 如果持续偏高 → LR 太大 |

### Step 8：Sim2Sim 交叉验证 ⭐⭐

在一个框架中训练完成后，在另一个框架中加载策略验证——这是 sim2real 的前置步骤。

```python
# sim2sim 验证流程
# 1. 在 mjlab 中训练 → 导出 policy.pt
# 2. 在 Isaac Lab 中加载相同 obs 配置 + policy.pt → 运行 eval

def sim2sim_eval(
    policy_path: str,
    eval_env: str = "MyQuad-Velocity-Flat",
    num_episodes: int = 100,
):
    """跨框架策略评估。"""
    env = make(eval_env, num_envs=64)
    policy = torch.jit.load(policy_path)

    success_count = 0
    for ep in range(num_episodes):
        obs, _ = env.reset()
        ep_reward = 0
        for step in range(1000):
            action = policy(obs['policy'])
            obs, reward, done, truncated, info = env.step(action)
            ep_reward += reward.mean().item()
            if done.all():
                break
        if ep_reward > threshold:
            success_count += 1

    success_rate = success_count / num_episodes
    print(f"Sim2Sim success rate: {success_rate:.1%}")
    return success_rate
```

**Sim2Sim 成功标准**：两个框架的 tracking error 差异 < 15%。如果差异更大，优先检查接触摩擦和 actuator 参数的跨框架差异。

### Step 9：ONNX 导出 ⭐

ONNX 导出是部署到真机的必要步骤。回顾 Ch23 的 ProtoMotions "obs computation baked-in" 模式——部署时不需要重写 obs 函数：

```python
# ONNX 导出（RSL-RL 标准流程）
import torch.onnx

def export_onnx(policy, obs_dim, path="policy.onnx"):
    dummy_input = torch.randn(1, obs_dim)
    torch.onnx.export(
        policy.actor,
        dummy_input,
        path,
        input_names=["obs"],
        output_names=["action"],
        opset_version=11,
        dynamic_axes={
            "obs": {0: "batch"},
            "action": {0: "batch"},
        },
    )
    print(f"✅ ONNX exported to {path}")

    # 验证
    import onnxruntime as ort
    session = ort.InferenceSession(path)
    result = session.run(None, {"obs": dummy_input.numpy()})
    torch_result = policy.actor(dummy_input).detach().numpy()
    max_diff = np.abs(result[0] - torch_result).max()
    print(f"Max diff (ONNX vs PyTorch): {max_diff:.6f}")
    assert max_diff < 1e-5, "ONNX export verification failed!"
```

**导出时的关键注意事项**：
- **obs 归一化**必须 baked-in：如果训练时使用了 `EmpiricalNormalization`（running mean/std），导出时必须把 mean 和 std 固化到 ONNX 图中，否则部署时 obs 不归一化，策略行为完全错误
- **RNN 隐状态**：如果使用了 LSTM 策略，hidden state 的初始化和传递必须在 ONNX 图中显式处理

### ⚠️ 常见陷阱

⚠️ **流程陷阱：跳过 Step 4 直接训练**
- 错误做法：URDF 转换完就直接接入框架训练
- 后果：接触参数不对导致机器人脚底打滑、actuator 力矩限制不对导致关节锁死
- 正确做法：在 MuJoCo viewer 中花 30 分钟手动验证模型——拖动关节、检查碰撞、确认 actuator 范围

⚠️ **流程陷阱：在 Step 7 之前花大量时间调 reward 权重**
- 错误做法：在纸上设计完美的 reward 权重再开始训练
- 后果：你的权重假设基于对环境行为的猜测——真实的行为往往和猜测差异很大
- 正确做法：用粗略的权重快速跑一次（500 iterations），看行为再调。"先跑后调"比"先想后跑"高效得多

⚠️ **编程陷阱：ONNX 导出忘记 bake obs normalization**
- 后果：部署时 obs 不归一化，policy 输出随机动作
- 正确做法：导出时显式检查 `policy.normalizer` 是否被包含在 ONNX 图中

### 练习

1. **[流程题]** 画出从 SolidWorks CAD 到真机部署的完整流水线图，标注每一步的输入/输出格式和使用的工具。
2. **[验证题]** 设计一个"双框架一致性测试套件"——至少包含 5 个测试用例（如 zero action base height、random action reward 分布、episode length 分布等），编写对应的 pytest 脚本。
3. **[跨章综合题]** 结合 Ch08 的 DR 和 Ch23 的 sim2real，为九步流水线中的 Step 8（sim2sim）设计一个更严格的验证协议：除了 tracking error，还应该检查哪些指标？

---

端到端流程给出了通用的九步模板。但抽象的模板需要具体案例来加深理解——下节展示两个来自顶会论文的真实 DIY 案例，让你看到"真正的研究项目是怎么做 DIY 的"。

---


## 22.5 实战案例精读：HOVER 和 HUSKY ⭐⭐⭐

> **这一节解决什么问题**：通过两个来自顶会的真实项目（HOVER 使用 Isaac Lab，HUSKY 使用 mjlab），展示 DIY 环境搭建在研究级项目中的实际样貌。

### 案例一：HOVER — Isaac Lab Extension 范式 ⭐⭐

**论文**：Tairan He et al., "HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots," ICRA 2025, arXiv:2410.21229
**框架**：Isaac Lab v2.0.0 + Isaac Sim 4.5
**机器人**：Unitree H1（19-DOF）
**代码**：`github.com/NVlabs/HOVER`

HOVER 的核心贡献是一个多模态全身控制器——一个策略同时支持 root velocity tracking、upper body joint angle tracking、hand keypoint tracking 等多种控制模式。其工程实现的关键创新是**双重 mask 机制**——mode-specific mask（选择激活哪些控制模式）和 sparsity-based mask（随机屏蔽部分命令维度）。

**HOVER 的训练 pipeline**：

```
Stage 1: Oracle Teacher
────────────────────────
输入: 完整 SMPL 参考姿态 + 全 state（privileged）
输出: 全关节 action
算法: PPO (RSL-RL)
网络: MLP [512, 256, 128], ELU activation
环境数: 4096
训练量: ~10k iterations

        │ 保存 Oracle checkpoint
        ▼

Stage 2: Student Distillation (DAgger)
────────────────────────────────────
输入: 25 帧历史 proprioception + masked command
      [q, q̇, ω_base, g]_{t-25:t} ∪ [a_{t-25:t-1}]
      + M_mode ⊙ M_sparsity ⊙ command
标签: Oracle 的 action（不是策略梯度，是监督学习！）
损失: L = ‖â_oracle − a_student‖²₂  (MSE action matching)
```

**从 HOVER 中学到的 DIY 经验**：

| 经验 | 具体做法 | 对应章节 |
|------|---------|---------|
| Oracle → Student 两阶段 | 先训练有 privileged obs 的强策略，再蒸馏 | Ch09 |
| Mask 随机化 | 每个 episode 开始时采样 mask，episode 内固定 | Ch05 obs 设计 |
| 历史帧堆叠 | 25 帧 $\times$ proprioception 维度 = 大 obs | Ch05 obs 设计 |
| DAgger 而非 KL | action MSE 比 distribution matching 更稳定 | Ch09 蒸馏 |
| Config YAML | 超参通过 YAML 管理，不修改源码 | Ch07 训练管线 |

**HOVER 的关键代码模式——mask 生成**：

```python
# HOVER 的 mask 生成逻辑（简化版）
def generate_masks(num_envs, mode_dim, device):
    """每个 episode 采样独立的 mode + sparsity mask。

    mode mask: 选择激活哪些控制子空间
      - kinematic position tracking
      - local joint angle tracking
      - root tracking (velocity/height/orientation)

    sparsity mask: 在已激活的子空间内随机屏蔽部分维度
    """
    # Mode mask: Bernoulli(0.5) per mode channel
    mode_mask = torch.bernoulli(
        0.5 * torch.ones(num_envs, mode_dim, device=device)
    )

    # Sparsity mask: Bernoulli(0.5) per dimension
    sparsity_mask = torch.bernoulli(
        0.5 * torch.ones(num_envs, mode_dim, device=device)
    )

    # 最终 command 被双重 mask 过滤
    # masked_cmd = M_sparsity ⊙ (M_mode ⊙ raw_command)
    return mode_mask, sparsity_mask
```

这种双重 mask 机制的训练效果是：student 策略在任意 mask 组合下都能正确执行——它学到了"如果命令被屏蔽，就保持默认行为"的泛化能力。这比为每种控制模式训练独立策略高效得多。

### 案例二：HUSKY — mjlab 研究项目范式 ⭐⭐

**论文**：Jinrui Han et al., "HUSKY: Humanoid Skateboarding System via Physics-Aware Whole-Body Control," RSS 2026, arXiv:2602.03205
**框架**：**mjlab** + RSL-RL + MuJoCo Warp
**机器人**：Unitree G1（29-DOF）+ 滑板
**代码**：`github.com/TeleHuman/humanoid_skateboarding`

HUSKY 是目前最高影响力的**使用 mjlab 框架**的研究项目。它实现了人形机器人在真实滑板上的推进和转向——一个涉及非完整约束、器具耦合和高动态动作的复合任务。

**HUSKY 的 MJCF 建模挑战**：

滑板和人形的建模核心是**转向架（truck）的运动学耦合**。真实的滑板转向架将板面的倾斜角 $\gamma$（tilt）映射为轮轴的转向角 $\sigma$（steering），通过主销（kingpin）角度 $\lambda$ 耦合：

$$\tan \sigma = \tan \lambda \cdot \sin \gamma$$

HUSKY 在 MuJoCo 中用 equality constraint 实现这个耦合——这是一个 DIY 建模的高级技巧：

```xml
<!-- HUSKY 的滑板 MJCF（简化） -->
<mujoco>
  <worldbody>
    <body name="board" pos="0 0 0.1">
      <joint name="board_x" type="slide" axis="1 0 0"/>
      <joint name="board_y" type="slide" axis="0 1 0"/>
      <joint name="board_yaw" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.4 0.1 0.01" mass="3.5"/>

      <!-- 前转向架 -->
      <body name="front_truck" pos="0.2 0 -0.03">
        <joint name="front_tilt" type="hinge" axis="0 1 0"/>
        <joint name="front_steer" type="hinge" axis="0 0 1"/>
        <!-- 前轮 -->
        <body name="front_wheel_l" pos="0 -0.08 -0.03">
          <joint name="front_wheel_l_spin" type="hinge" axis="0 1 0"/>
          <geom type="cylinder" size="0.03 0.01"/>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- Kingpin 耦合约束 -->
  <equality>
    <!-- tan(steer) = tan(kingpin_angle) * sin(tilt) -->
    <!-- MuJoCo 的 joint 约束用 polynomial 近似 -->
    <joint joint1="front_steer" joint2="front_tilt"
           polycoef="0 0.45 0 0 0"/>
  </equality>
</mujoco>
```

**HUSKY 的训练 pipeline**：

```
Phase 1: 推进技能（Pushing）
──────────────────────
方法: AMP (Adversarial Motion Priors)
参考动作: 人类推滑板的 MoCap 片段
环境数: 4096
时长: 每 episode 20 秒

Phase 2: 转向技能（Steering）
──────────────────────
方法: Physics-guided heading reward
  r_heading = cos(θ_actual - θ_target)
不使用 AMP（因为转向的运动模式和人类不同）

Phase 3: 推进↔转向切换
──────────────────────
方法: Trajectory-guided transition
  根据预规划的路径，在需要转向时切换到 steering mode
```

**从 HUSKY 中学到的 DIY 经验**：

| 经验 | 具体做法 | 对应章节 |
|------|---------|---------|
| 器具建模需要 equality constraint | 滑板转向架的运动学耦合 | Ch11 建模、Ch03 物理引擎 |
| AMP 只用于"人类做得好"的子技能 | 推进用 AMP，转向用 physics reward | Ch10 模仿学习 |
| 多阶段训练而非 end-to-end | 推进→转向→切换，各阶段独立训练 | Ch06 Curriculum |
| mjlab 支持复杂的自定义环境 | 4096 并行 $\times$ 20 秒 episode | Ch24 大规模训练 |

### 两个案例的对比总结

| 维度 | HOVER (Isaac Lab) | HUSKY (mjlab) |
|------|-------------------|---------------|
| 框架 | Isaac Lab extension | mjlab 内置任务格式 |
| 训练方法 | Oracle PPO + DAgger 蒸馏 | PPO + AMP |
| 关键建模技巧 | Mask obs group | Equality constraint |
| 控制频率 | 50 Hz policy / 200 Hz sim | 50 Hz policy / 500 Hz sim |
| 部署平台 | Unitree H1 | Unitree G1 |
| 代码规模 | ~5000 行（含 wrapper/inference） | ~3000 行（更紧凑） |
| 开源许可 | Apache 2.0 | CC BY-NC 4.0 |

**选择建议**：如果你的任务需要多模态控制、视觉输入或 USD 资产，参考 HOVER 的 Isaac Lab extension 模式。如果你的任务涉及复杂的物理建模（自定义约束、器具耦合）或偏好更轻量级的 codebase，参考 HUSKY 的 mjlab 模式。两者不互斥——你可以在 mjlab 中快速原型验证，然后用 Isaac Lab extension 做生产级训练。

### ⚠️ 常见陷阱

⚠️ **概念误区：认为"HOVER/HUSKY 的代码可以直接用"**
- 这些项目的代码针对特定机器人（H1/G1）和特定任务（全身控制/滑板）优化
- 正确做法：学习它们的**架构模式**（mask obs group、AMP 集成、equality constraint），然后在你的任务中重新实现

⚠️ **编程陷阱：HOVER 的 rsl_rl 是定制 fork**
- HOVER 依赖的 `rsl_rl` 是 NVIDIA 定制版本，和 upstream RSL-RL 有 API 差异
- 如果你用 pip install rsl-rl 安装的标准版本，训练脚本会报错
- 正确做法：使用 HOVER 仓库中 `third_party/` 目录下 pin 的版本

### 练习

1. **[代码阅读题]** 阅读 HOVER 仓库的 `neural_wbc/core/oracles/` 目录，回答：Oracle teacher 的 obs 维度是多少？它和 Student 的 obs 维度差异来自哪里？
2. **[设计题]** 如果你要用 HOVER 的 mask 机制实现一个"导航 + 抓取"的双模态控制器，mask 应该怎么设计？哪些 obs 维度属于导航模式，哪些属于抓取模式？
3. **[实践题]** Clone HUSKY 仓库，在 MuJoCo viewer 中加载滑板 MJCF（`test_scene/mjlab_scene.xml`），手动控制人形站上滑板。观察 equality constraint 是否生效——倾斜板面时轮子是否自动转向。

---

研究级案例展示了"好的 DIY 项目长什么样"。但在你到达那个水平之前，你更可能遇到的是各种 bug——环境不报错但策略不学习、物理行为不合理、reward 曲线诡异。下节系统化地覆盖 DIY 中最常见的 10 类 bug 及其排查方法。

---

## 22.6 EventsCfg 与 CommandsCfg 设计 ⭐⭐

> **这一节解决什么问题**：设计完整的 DR（Domain Randomization）和指令采样配置——这两个 Manager 是自定义环境中最容易被忽视但对 sim2real 影响最大的组件。

### EventsCfg 的四种模式

回顾 Ch08：EventManager 支持四种触发模式，每种模式对应不同的随机化需求：

| 模式 | 触发时机 | 典型用途 | 性能影响 |
|------|---------|---------|---------|
| `startup` | 环境创建时，只执行一次 | 物理参数随机化（质量、摩擦、惯量） | 无（只执行一次） |
| `reset` | 每次 episode reset 时 | 初始状态随机化（位姿、关节角、速度） | 低 |
| `interval` | 每隔 N 步执行一次 | 外部扰动（push force、风力） | 中 |
| `step` | 每个仿真步都执行 | Action delay、obs noise | 高（需要 batch 优化） |

**自定义任务中 EventsCfg 的推荐配置模板**：

```python
class EventsCfg:
    """DIY 任务的标准 DR 配置。"""

    # === startup 模式：物理参数（episode 间不变） ===
    randomize_friction = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": EntityCfg(name="robot", body_names=".*"),
            "static_friction_range": (0.7, 1.3),
            "dynamic_friction_range": (0.7, 1.3),
        },
    )
    randomize_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": EntityCfg(name="robot", body_names="base"),
            "mass_distribution_params": (-1.0, 3.0),  # 加 -1~3 kg 载荷
            "operation": "add",
        },
    )

    # === reset 模式：初始状态（每 episode 不同） ===
    reset_base_pose = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5), "y": (-0.5, 0.5),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.5, 0.5), "y": (-0.25, 0.25), "z": (-0.3, 0.3),
                "roll": (-0.25, 0.25), "pitch": (-0.25, 0.25), "yaw": (-0.25, 0.25),
            },
        },
    )
    reset_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.25, 0.25),
            "velocity_range": (-0.5, 0.5),
        },
    )

    # === interval 模式：外部扰动 ===
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),  # 每 10-15 秒推一次
        params={
            "velocity_range": {
                "x": (-0.5, 0.5), "y": (-0.5, 0.5),
            },
        },
    )

    # === step 模式：执行延迟 ===
    action_delay = EventTermCfg(
        func=mdp.randomize_action_delay,
        mode="step",
        params={"delay_range": (0, 2)},  # 0-2 个 sim step 的延迟
    )
```

**EventsCfg 的分阶段引入策略**：不要一开始就打开所有 DR。按照 Ch08 的经验，推荐的引入顺序是：

| 训练阶段 | 引入的 DR | 原因 |
|---------|---------|------|
| 0-2000 iter | 无 DR（只有 reset randomization） | 让策略先学会基本行为 |
| 2000-5000 iter | + friction + mass randomization | 增加鲁棒性，不影响基本行为 |
| 5000-8000 iter | + push disturbance | 测试抗扰能力 |
| 8000+ iter | + action delay + obs noise | sim2real 准备 |

### CommandsCfg 设计 ⭐⭐

CommandsCfg 定义了策略需要完成的"任务指令"。对于速度跟踪任务，指令是目标速度；对于导航任务，指令是目标位置；对于操作任务，指令是目标物体位姿。

**速度指令的设计考量**：

```python
class CommandsCfg:
    base_velocity = UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),  # 每 10 秒重新采样
        heading_command=True,  # 包含 heading 指令
        debug_vis=True,  # 在 viewer 中显示指令方向

        ranges=UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0),   # 前后速度 (m/s)
            lin_vel_y=(-0.5, 0.5),   # 侧向速度 (m/s)
            ang_vel_z=(-1.0, 1.0),   # 偏航角速度 (rad/s)
            heading=(-math.pi, math.pi),  # 目标朝向 (rad)
        ),
    )
```

**关键参数解释**：

| 参数 | 含义 | 调优方向 |
|------|------|---------|
| `resampling_time_range` | 指令重新采样的间隔 | 太短（<3s）策略来不及执行；太长（>30s）训练效率低 |
| `heading_command` | 是否使用 heading 而非 ang_vel_z | heading 更适合导航；ang_vel_z 更适合速度跟踪 |
| `lin_vel_x` 范围 | 前后速度的采样范围 | 从小范围开始（$\pm$0.5），curriculum 逐步扩大 |
| `lin_vel_y` 范围 | 侧向速度范围 | 差速底盘设为 0（不能横移）；全向底盘可设非零 |

**自定义 Command 的编写模式**：如果你的任务不是速度跟踪（如目标位置导航），需要编写自定义 Command：

```python
class GoalPositionCommandCfg(CommandTermCfg):
    """目标位置指令（用于导航任务）。"""
    resampling_time_range: tuple = (15.0, 20.0)
    goal_range_x: tuple = (-3.0, 3.0)
    goal_range_y: tuple = (-3.0, 3.0)

class GoalPositionCommand(CommandTerm):
    """实现目标位置的采样和提供。"""
    cfg: GoalPositionCommandCfg

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.goal_pos = torch.zeros(env.num_envs, 2, device=env.device)

    def _resample_command(self, env_ids):
        """在指定 env 中重新采样目标位置。"""
        self.goal_pos[env_ids, 0] = torch.uniform(
            self.cfg.goal_range_x[0], self.cfg.goal_range_x[1],
            size=(len(env_ids),), device=self.device,
        )
        self.goal_pos[env_ids, 1] = torch.uniform(
            self.cfg.goal_range_y[0], self.cfg.goal_range_y[1],
            size=(len(env_ids),), device=self.device,
        )

    def _compute_command(self):
        """每步返回目标位置（base frame）。"""
        # 将目标从 world frame 转换到 base frame
        base_pos = self.robot.data.root_link_pos_w[:, :2]
        base_quat = self.robot.data.root_link_quat_w
        rel_w = self.goal_pos - base_pos
        rel_b = quat_rotate_inverse(base_quat, 
            torch.cat([rel_w, torch.zeros_like(rel_w[:, :1])], dim=-1))
        return rel_b[:, :2]  # [B, 2]

    def _compute_metrics(self):
        dist = (self.goal_pos - self.robot.data.root_link_pos_w[:, :2]).norm(dim=-1)
        return {"goal_dist": dist.mean()}
```

> **本质洞察**：CommandTerm 不一定是"给策略的目标指令"——它也可以是 **episode 级实验条件生成器**。在 Ch22 §22.2 的速度跟踪中，command 是策略要追踪的目标速度；但在自定义的物理验证环境中（如网球发射器），command 可以是"发球参数"——速度、仰角、落点范围——策略不需要追踪这些参数，它们只是定义了每个 episode 的物理条件。把随机化条件封装为 CommandTerm 而非硬编码到 reset 函数中，有两个工程好处：(1) 条件的采样范围可以通过 CLI 参数覆盖（如 `--env.commands.launch.speed-range "[20, 45]"`），不需要改代码；(2) 每个 episode 的条件被显式记录在 command buffer 中，可复现、可分析。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：resampling_time_range 太短导致指令抖动**
- 如果 `resampling_time_range=(1.0, 1.0)`，策略每秒收到一个新指令——还没执行完上一个就被要求做新的
- 后果：策略学到"忽略指令"的行为（因为执行指令反而被惩罚——tracking error 在指令切换时瞬间变大）
- 正确做法：resampling time $\ge$ 机器人到达目标速度所需的时间 $\times$ 2

⚠️ **编程陷阱：Command 返回的是 world frame 而非 base frame**
- 后果：obs 中的指令随底盘旋转而变化——相同的"向前走"在不同朝向下有不同的 obs 表示
- 正确做法：Command 的 `_compute_command()` 必须返回 base frame 中的值

### 练习

1. **[设计题]** 为一个"物体导航"任务（机器人走到物体旁边）设计 CommandsCfg。指令应该包含什么信息？重新采样的时机应该是什么？
2. **[实验题]** 在速度跟踪任务中，分别用 `resampling_time_range=(3.0, 3.0)` 和 `(15.0, 15.0)` 训练，对比 tracking reward 和行为平滑度。
3. **[跨章综合题]** 结合 Ch08 的分阶段 DR 策略，设计一个自动化脚本：根据当前训练 iteration 自动决定激活哪些 EventTerm。提示：用 CurriculumManager 驱动 EventsCfg 的修改。

---

## 22.7 环境搭建的十大常见 Bug 与排查 ⭐⭐⭐

> **这一节解决什么问题**：系统化地覆盖自定义环境中最常见的 10 类 bug，每类给出症状、根因、排查步骤和修复方法。

### 动机：为什么需要"Bug 字典"

自定义环境的 bug 有一个独特特征：**大多数 bug 不会导致运行时错误，而是导致"策略训练不收敛"**。这意味着你可能花了一整天训练，看到 reward 曲线平坦，然后开始怀疑是 reward 设计的问题、是超参的问题、是算法的问题——但实际上可能只是 obs 中某个维度的 frame 转换搞反了。

以下 10 类 bug 覆盖了作者和社区经验中 90% 以上的自定义环境问题。每类 bug 按"症状→根因→排查→修复"的固定格式组织。

### Bug 1：机器人初始化时"弹飞" ⭐

**症状**：环境 reset 后第一步，机器人 base height 暴增到 5-50 m，然后 terminated。

**根因**：初始关节角导致身体部件之间（或身体与地面）穿模。MuJoCo/PhysX 的接触求解器检测到穿透后产生巨大的排斥力。

**排查步骤**：
```python
# 1. 加载模型，不运行物理，检查初始碰撞
model = mujoco.MjModel.from_xml_path("my_robot.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)  # 只计算运动学，不推进物理
print(f"Initial contacts: {data.ncon}")
for i in range(data.ncon):
    c = data.contact[i]
    print(f"  Contact {i}: geom1={model.geom(c.geom1).name}, "
          f"geom2={model.geom(c.geom2).name}, dist={c.dist:.4f}")
```

**修复**：调整 `init_state.joint_pos` 使初始姿态无穿模。或者增大 `solimp` 的 width 参数，允许更大的初始穿透被逐步修正（不推荐，治标不治本）。

### Bug 2：Zero Agent 下机器人缓慢倒塌 ⭐

**症状**：action=0 时机器人在 100-200 步内逐渐倒下。

**根因**：PD 控制器的 stiffness 不够大，无法支撑机器人自重。或者初始关节角不在平衡点。

**排查步骤**：
```python
# 检查每个关节的力矩需求
# 在初始姿态下，重力产生的关节力矩 vs PD 控制器能提供的最大力矩
for i in range(model.njnt):
    # 重力力矩 = 该关节以上所有 body 的重力 × 力臂
    # 简化检查：打印 qfrc_bias（包含重力和科氏力）
    gravity_torque = data.qfrc_bias[i + 6]  # 跳过 freejoint 的 6 维
    max_pd_torque = model.actuator_forcerange[i, 1]
    ratio = abs(gravity_torque / max_pd_torque) if max_pd_torque > 0 else float('inf')
    print(f"Joint {model.joint(i+1).name}: "
          f"gravity_torque={gravity_torque:.2f}, "
          f"max_pd={max_pd_torque:.2f}, "
          f"ratio={ratio:.2%}")
    if ratio > 0.8:
        print(f"  ⚠️ 该关节可能无法支撑重力！")
```

**修复**：增大 stiffness（但不要太大，否则 PD 控制会产生高频振荡）。或者调整初始关节角到更"平衡"的姿态。

### Bug 3：Obs 维度不匹配导致训练崩溃 ⭐

**症状**：训练开始时报 RuntimeError: shape mismatch，或者 obs tensor 形状和 policy 网络输入维度不一致。

**根因**：ObservationsCfg 中的 term 维度之和不等于网络输入维度。常见于修改了 obs 配置但忘记更新 PPO 配置中的 `num_observations`。

**排查步骤**：
```python
env = make("MyTask", num_envs=4)
obs, _ = env.reset()
actual_dim = obs['policy'].shape[-1]
print(f"Actual obs dim: {actual_dim}")
# 逐项打印每个 obs term 的维度
for name, term in env.observation_manager.active_terms["policy"]:
    val = term.func(env, **term.params)
    print(f"  {name}: shape={val.shape}")
```

**修复**：确保 PPO 配置中的 `num_observations` 等于所有 obs term 维度之和。更好的做法：让 PPO 配置自动从环境推断 obs 维度。

### Bug 4：Reward 始终为常数 ⭐⭐

**症状**：reward 曲线从第一步开始就是一个固定值，不随训练变化。

**根因**：reward 函数没有和当前状态/动作关联。常见于 reward 函数引用了错误的变量名（如 `env.robot` 写成了 `env.arm`，返回了默认值零）。

**排查步骤**：
```python
# 分项打印每个 reward term 在 random action 下的值
env = make("MyTask", num_envs=64)
obs, _ = env.reset()
for _ in range(10):
    action = torch.randn(64, env.action_space.shape[-1])
    obs, reward, _, _, info = env.step(action)

# 打印 reward 分项
for name, val in env.reward_manager.compute().items():
    print(f"  {name}: mean={val.mean():.4f}, std={val.std():.4f}")
```

**修复**：找到 std=0 的 reward term，检查其函数实现。

### Bug 5：策略学到"原地抖动"而不是行走 ⭐⭐

**症状**：reward 曲线上升，但 viewer 中机器人原地高频颤抖而不是行走。

**根因**：action rate penalty 太小（或没有），tracking reward 在原地就有正信号（因为速度指令包含 0），或者 feet_air_time reward 的阈值设置不对。

**排查步骤**：在 viewer 中观察 + 打印 velocity command 和 actual velocity 的分布。如果 command 经常为零且 reward 在零速度下较高，策略没有动力去行走。

**修复**：(1) 增大 action_rate_l2 权重；(2) 检查 velocity command 的分布——如果零速度概率太高，调整 command sampler；(3) 增加 feet_air_time reward 鼓励迈步。

### Bug 6：训练中途突然 NaN ⭐⭐

**症状**：训练前 1000 iterations 正常，然后 obs 或 reward 突然出现 NaN，训练崩溃。

**根因**：物理状态漂移到极端区域——关节角超限、body 位置超过仿真边界、速度过大导致接触求解器发散。

**排查步骤**：
```python
# 开启 NaN 监控
# mjlab: --enable-nan-guard True
# 手动检查：
obs, _ = env.reset()
for step in range(10000):
    action = policy(obs['policy'])
    obs, reward, done, truncated, info = env.step(action)
    if torch.isnan(obs['policy']).any():
        # 找到哪些 env 和哪些 obs 维度是 NaN
        nan_envs = torch.isnan(obs['policy']).any(dim=-1)
        nan_dims = torch.isnan(obs['policy'][nan_envs[0]]).nonzero()
        print(f"NaN at step {step}, envs: {nan_envs.nonzero()}")
        print(f"NaN dims: {nan_dims}")
        # 打印该 env 的物理状态
        print(f"qpos: {env.robot.data.joint_pos[nan_envs[0]]}")
        print(f"qvel: {env.robot.data.joint_vel[nan_envs[0]]}")
        break
```

**修复**：(1) 加入更严格的 termination（关节角超限、base 高度过低/过高）；(2) clip obs 到合理范围；(3) 降低学习率或 gradient clip。

### Bug 7：两个框架训练结果差异巨大 ⭐⭐

**症状**：同一个任务在 mjlab 中 reward 正常上升，在 Isaac Lab 中 reward 远低于预期（或反之）。

**根因**：物理引擎差异（MuJoCo vs PhysX 的接触模型、摩擦模型、积分方法不同）导致相同配置下机器人行为不同。

**排查步骤**：执行 §22.4 Step 5 的双框架一致性验证脚本。特别检查 zero action 下的 base height drift 和 contact forces。

**修复**：调整接触参数使两个框架的物理行为尽可能接近。MuJoCo 的 `solref/solimp` 和 PhysX 的 `solver_position_iteration_count` 没有直接对应关系——需要通过实验对齐（drop test、slide test）。

### Bug 8：Curriculum 不推进 ⭐

**症状**：策略在 Phase 0 训练了数千 iterations，success rate 不达标，curriculum 永远不推进到下一阶段。

**根因**：Phase 0 的难度已经太高（物体太远、目标区域太小），或者 success 的判定标准太严格。

**排查步骤**：
```python
# 打印 Phase 0 的 success rate 和 metrics
print(f"Phase: {env.curriculum_phase}")
print(f"Success rate: {env.metrics.get('success_rate', 0):.1%}")
print(f"Avg episode length: {env.metrics.get('ep_len_mean', 0):.0f}")
```

**修复**：降低 Phase 0 的 success 阈值（如从 50% 降到 30%），或者缩小物体初始范围让任务更容易。

### Bug 9：env.reset() 后 obs 不变 ⭐

**症状**：连续调用 `env.reset()` 多次，返回的 obs 完全相同。

**根因**：EventManager 中没有配置 reset-mode 的随机化事件。每次 reset 都恢复到完全相同的初始状态。

**排查步骤**：检查 EventsCfg 中是否有 `mode="reset"` 的 EventTerm。

**修复**：添加初始状态随机化：
```python
class EventsCfg:
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )
    reset_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={"position_range": (-0.2, 0.2)},
    )
```

### Bug 10：训练极慢（< 100 steps/s） ⭐

**症状**：预期 4096 envs 下应该达到 ~50,000 steps/s，但实际只有几百。

**根因**：(1) num_envs 设置太小；(2) obs/reward 计算中有 Python for 循环而非 batch 操作；(3) GPU 利用率不足（CPU-GPU 数据搬运成为瓶颈）。

**排查步骤**：
```bash
# 用 torch.profiler 定位瓶颈
python scripts/train.py MyTask --profiler=True --max-iterations=10

# 检查 GPU 利用率
nvidia-smi -l 1  # 持续监控
```

**修复**：(1) 增大 num_envs 到 4096+；(2) 把 for 循环替换为 torch batch 操作；(3) 检查 obs 函数中是否有不必要的 `.cpu()` 调用。

### 排查总表

| Bug | 症状 | 排查工具 | 修复方向 | 参考节 |
|-----|------|---------|---------|--------|
| 1. 弹飞 | base height > 5m | `mj_forward` + ncon | 调 init_state | §22.2 |
| 2. 倒塌 | zero agent 倒下 | qfrc_bias 分析 | 调 stiffness/init_pos | §22.2 |
| 3. Obs 维度 | shape mismatch | 逐项打印 obs shape | 更新 num_obs | §22.2 |
| 4. 常数 reward | reward std=0 | 分项 reward 打印 | 检查变量引用 | §22.2 |
| 5. 原地抖动 | 高频颤抖 | viewer + cmd 分布 | 加 action_rate | Ch06 |
| 6. NaN | 训练中途崩溃 | NaN guard + qpos 打印 | 加 termination/clip | Ch24 |
| 7. 跨框架差异 | reward 差异 > 30% | 双框架一致性脚本 | 调接触参数 | §22.4 |
| 8. Curriculum 卡 | 不推进 | success rate 打印 | 降低阈值 | Ch06 |
| 9. Reset 不变 | obs 恒定 | 检查 EventsCfg | 加 reset event | Ch08 |
| 10. 训练慢 | < 100 steps/s | torch.profiler | 增 env / 消除 for loop | Ch24 |

### ⚠️ 常见陷阱

⚠️ **排查陷阱：只看 reward 曲线不看行为**
- reward 上升不等于行为正确。策略可能找到了 reward hacking 的捷径（如利用仿真 bug 获得高 reward）
- 正确做法：每 500 iterations 用 viewer 观察一次行为，确认和预期一致

⚠️ **排查陷阱：修改多项配置后一起测试**
- 同时改了 obs、reward 和 DR，训练变好了——你不知道是哪个改动起了作用
- 正确做法：每次只改一项，记录结果。这就是 AGILE 四阶段 workflow 中 Prepare 阶段的核心精神

### 练习

1. **[诊断题]** 你训练了一个四足走路任务，reward 在 2000 iterations 后趋于平坦。viewer 中机器人能向前走但步幅很小。列出你的排查步骤（按优先级排序），以及每一步的预期发现。
2. **[实践题]** 故意在你的自定义环境中引入 Bug 4（reward 常数化）：把 tracking reward 的 `command_name` 改为一个不存在的名字。观察训练的 reward 曲线和行为，确认你能从"症状"反推到"根因"。
3. **[跨章综合题]** 结合 Ch25 的训练诊断方法，设计一个"自定义环境健康检查报告模板"——包含至少 10 个自动检查项、每项的通过标准和不通过时的建议修复动作。

---

### 系统化排查工作流 ⭐⭐

当你遇到"策略不收敛"这类模糊症状时，按以下决策树系统化排查，避免盲目猜测：

```
训练不收敛
│
├── reward 全为 0 或常数？
│   ├── 是 → Bug 4（reward 函数断路）
│   │   排查：分项打印每个 reward term
│   └── 否 ↓
│
├── reward 在上升但行为不对？
│   ├── 原地抖动 → Bug 5（缺 action rate penalty / command 分布问题）
│   ├── 不移动 → 检查 termination 是否太严格（一动就 done）
│   └── 动作剧烈 → action scale 太大 / init_noise_std 太大
│
├── reward 在 N iter 后突然崩溃（NaN）？
│   └── Bug 6（物理状态越界）→ 加 obs clip + 更严格的 termination
│
├── reward 缓慢上升但太慢？
│   ├── KL divergence 太高 → 降低 learning_rate
│   ├── entropy 太低（< 1.0）→ 增大 init_noise_std 或 entropy_coef
│   └── episode 太短 → 放宽 termination 阈值
│
└── 一切看起来正常但 sim2sim 结果差？
    └── Bug 7（跨框架物理差异）→ 对比 zero agent base height
```

**排查的第一原则：先排除环境 bug，再怀疑算法/超参。** 在自定义环境中，80% 的"训练不收敛"问题来自环境配置错误，而非 PPO 超参不对。如果你对环境配置没有 100% 的信心（Phase A + B 的 checklist 全部通过），不要去调超参——那是在错误的方向上努力。

**排查的第二原则：用 random agent 作为 baseline。** 如果训练了 1000 iterations 的策略表现还不如 random agent（episode reward 更低、episode length 更短），说明 PPO 的更新方向完全错误——大概率是 obs 的 frame 转换搞反或 reward 的符号搞反。这个简单的 baseline 对比可以在 5 分钟内揪出方向性错误。

```python
# 快速 baseline 对比脚本
def compare_with_random(env, policy, n_episodes=50):
    """对比训练策略和 random agent 的表现。"""
    results = {}
    for agent_name, agent_fn in [
        ("random", lambda obs: torch.randn_like(obs[:, :env.action_space.shape[-1]])),
        ("trained", lambda obs: policy(obs)),
    ]:
        ep_rewards = []
        obs, _ = env.reset()
        ep_rew = torch.zeros(env.num_envs, device=obs['policy'].device)
        ep_count = 0
        for _ in range(2000):
            action = agent_fn(obs['policy'])
            obs, reward, done, trunc, info = env.step(action)
            ep_rew += reward
            finished = done | trunc
            if finished.any():
                ep_rewards.extend(ep_rew[finished].tolist())
                ep_rew[finished] = 0
                ep_count += finished.sum().item()
            if ep_count >= n_episodes:
                break
        results[agent_name] = {
            "mean_reward": np.mean(ep_rewards[:n_episodes]),
            "std_reward": np.std(ep_rewards[:n_episodes]),
        }

    print(f"Random: {results['random']['mean_reward']:.2f} "
          f"± {results['random']['std_reward']:.2f}")
    print(f"Trained: {results['trained']['mean_reward']:.2f} "
          f"± {results['trained']['std_reward']:.2f}")

    if results['trained']['mean_reward'] < results['random']['mean_reward']:
        print("⚠️ 训练策略不如 random agent！检查 obs frame 和 reward 符号。")
    else:
        improvement = (results['trained']['mean_reward'] -
                       results['random']['mean_reward'])
        print(f"✅ 策略优于 random agent，提升: {improvement:.2f}")
```

---


## 22.8 双框架 DIY 实战：自定义四足速度跟踪完整代码 ⭐⭐⭐

> **这一节解决什么问题**：以一个假想的非标四足机器人（MyQuad）为例，给出双框架中从配置到训练的**完整可运行代码**，读者可以直接复制修改。

### 动机：从模板到可运行代码

前面的节给出了概念框架和代码片段。但真正的 DIY 需要一个完整的、端到端的代码参考——不是 30 行的片段，而是 300 行的完整配置文件。本节就是这个参考。

### mjlab 完整环境代码 ⭐⭐

以下代码展示了一个完整的 mjlab 自定义四足速度跟踪环境的所有配置文件。读者可以用自己的 MJCF 替换 `my_quad.xml`，修改 joint names，即可运行。

```python
# ============================================================
# 文件 1/4: my_quad_robot_cfg.py — 机器人 Entity 配置
# ============================================================

import math
from mjlab.envs import EntityCfg, MjCfg
from mjlab.actuators import ImplicitActuatorCfg

MY_QUAD_CFG = EntityCfg(
    spawn=MjCfg(
        mjcf_path="my_quad/xmls/my_quad.xml",
    ),
    init_state=EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.42),
        rot=(1.0, 0.0, 0.0, 0.0),  # wxyz quaternion
        joint_pos={
            # === 前左腿 ===
            "FL_hip_joint": 0.0,
            "FL_thigh_joint": 0.8,
            "FL_calf_joint": -1.6,
            # === 前右腿 ===
            "FR_hip_joint": 0.0,
            "FR_thigh_joint": 0.8,
            "FR_calf_joint": -1.6,
            # === 后左腿 ===
            "HL_hip_joint": 0.0,
            "HL_thigh_joint": 1.0,
            "HL_calf_joint": -1.6,
            # === 后右腿 ===
            "HR_hip_joint": 0.0,
            "HR_thigh_joint": 1.0,
            "HR_calf_joint": -1.6,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "FL_.*", "FR_.*", "HL_.*", "HR_.*"
            ],
            stiffness={
                ".*_hip_joint": 20.0,
                ".*_thigh_joint": 20.0,
                ".*_calf_joint": 25.0,  # calf 需要更大刚度支撑重量
            },
            damping={
                ".*_hip_joint": 0.5,
                ".*_thigh_joint": 0.5,
                ".*_calf_joint": 0.5,
            },
            velocity_limit=21.0,
            effort_limit=33.5,
        ),
    },
)
```

```python
# ============================================================
# 文件 2/4: my_quad_env_cfg.py — 完整环境配置
# ============================================================

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import *
from mjlab.scene import InteractiveSceneCfg
from my_quad_robot_cfg import MY_QUAD_CFG

# ---------- Scene ----------
class MyQuadSceneCfg(InteractiveSceneCfg):
    env_spacing = 3.0
    robot = MY_QUAD_CFG.replace(
        prim_path="/World/envs/env_.*/Robot"
    )
    ground = EntityCfg(spawn=MjCfg(mjcf_path="terrain/flat_ground.xml"))

# ---------- Actions ----------
class ActionsCfg:
    joint_pos = JointPositionActionCfg(
        asset_name="robot",
        joint_names=["FL_.*", "FR_.*", "HL_.*", "HR_.*"],
        scale=0.25,
        use_default_offset=True,
    )

# ---------- Observations ----------
class ObservationsCfg:
    class PolicyCfg(ObservationGroupCfg):
        # 本体感受
        base_lin_vel = ObsTerm(func=base_lin_vel)           # 3
        base_ang_vel = ObsTerm(func=base_ang_vel)           # 3
        projected_gravity = ObsTerm(func=projected_gravity)  # 3
        # 指令
        velocity_commands = ObsTerm(
            func=generated_commands,
            params={"command_name": "base_velocity"}
        )                                                     # 3
        # 关节状态
        joint_pos = ObsTerm(func=joint_pos_rel)              # 12
        joint_vel = ObsTerm(func=joint_vel_rel)              # 12
        # 历史动作
        actions = ObsTerm(func=last_action)                  # 12
        # 总计: 3+3+3+3+12+12+12 = 48

    class CriticCfg(ObservationGroupCfg):
        enable_corruption = False
        # 同 PolicyCfg + privileged
        base_lin_vel = ObsTerm(func=base_lin_vel)
        base_ang_vel = ObsTerm(func=base_ang_vel)
        projected_gravity = ObsTerm(func=projected_gravity)
        velocity_commands = ObsTerm(
            func=generated_commands,
            params={"command_name": "base_velocity"}
        )
        joint_pos = ObsTerm(func=joint_pos_rel)
        joint_vel = ObsTerm(func=joint_vel_rel)
        actions = ObsTerm(func=last_action)
        # Privileged
        base_lin_vel_w = ObsTerm(func=root_lin_vel_w)        # 3
        heights = ObsTerm(func=height_scan,
                          params={"sensor_cfg": ...})        # 可选

# ---------- Commands ----------
class CommandsCfg:
    base_velocity = UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        heading_command=True,
        ranges=UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(-0.5, 0.5),
            ang_vel_z=(-1.0, 1.0),
            heading=(-math.pi, math.pi),
        ),
    )

# ---------- Rewards ----------
class RewardsCfg:
    # === Tracking (正) ===
    track_lin_vel_xy = RewTerm(
        func=track_lin_vel_xy_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    track_ang_vel_z = RewTerm(
        func=track_ang_vel_z_exp,
        weight=0.75,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    # === Regularization (负) ===
    lin_vel_z_l2 = RewTerm(func=lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=ang_vel_xy_l2, weight=-0.05)
    flat_orientation_l2 = RewTerm(func=flat_orientation_l2, weight=-1.0)
    dof_torques_l2 = RewTerm(func=joint_torques_l2, weight=-2e-4)
    dof_acc_l2 = RewTerm(func=joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=action_rate_l2, weight=-0.01)
    # === Contact ===
    feet_air_time = RewTerm(
        func=feet_air_time,
        weight=0.2,
        params={"sensor_cfg": ContactSensorCfg(body_names=".*_foot"),
                "threshold": 0.5},
    )
    undesired_contacts = RewTerm(
        func=undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": ContactSensorCfg(
            body_names=[".*_thigh", "base"]),
                "threshold": 1.0},
    )

# ---------- Terminations ----------
class TerminationsCfg:
    time_out = DoneTerm(func=time_out, time_out=True)
    base_contact = DoneTerm(
        func=illegal_contact,
        params={"sensor_cfg": ContactSensorCfg(body_names="base"),
                "threshold": 1.0},
    )

# ---------- Events (DR) ----------
class EventsCfg:
    # Reset 随机化
    reset_base = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5), "pitch": (-0.5, 0.5), "yaw": (-0.5, 0.5),
            },
        },
    )
    reset_joints = EventTermCfg(
        func=reset_joints_by_offset,
        mode="reset",
        params={"position_range": (-0.25, 0.25), "velocity_range": (-0.5, 0.5)},
    )
    # Startup 随机化 (DR)
    physics_material = EventTermCfg(
        func=randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": EntityCfg(name="robot", body_names=".*"),
            "static_friction_range": (0.7, 1.3),
            "dynamic_friction_range": (0.7, 1.3),
        },
    )
    add_base_mass = EventTermCfg(
        func=randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": EntityCfg(name="robot", body_names="base"),
            "mass_distribution_params": (-1.0, 1.0),
            "operation": "add",
        },
    )

# ---------- 组装 ----------
class MyQuadVelocityFlatEnvCfg(ManagerBasedRlEnvCfg):
    scene = MyQuadSceneCfg(num_envs=4096, env_spacing=3.0)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    commands = CommandsCfg()
    events = EventsCfg()

    sim = SimCfg(dt=0.005, decimation=4)  # 50 Hz 策略
    episode_length_s = 20.0
```

### Isaac Lab 等价代码要点 ⭐⭐

将上述 mjlab 代码迁移到 Isaac Lab 时，核心差异集中在以下几处：

```python
# === Isaac Lab 等价代码（仅展示差异部分） ===

# 差异 1: 模型加载 — USD 替代 MJCF
MY_QUAD_IL_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="my_quad_lab/assets/my_quad.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            max_depenetration_velocity=10.0,  # PhysX 特有：最大去穿透速度
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,  # PhysX 求解器迭代次数
            solver_velocity_iteration_count=4,
        ),
    ),
    # init_state 和 actuators 与 mjlab 配置完全相同
    # ...
)

# 差异 2: Scene 的 prim_path 使用 {ENV_REGEX_NS}
class MyQuadILSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    robot = MY_QUAD_IL_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"  # 注意这个占位符！
    )
    # Isaac Lab 的 contact sensor 需要显式配置
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=3,
        track_air_time=True,
    )

# 差异 3: 注册方式 — gymnasium.register 替代 register_task
import gymnasium as gym

gym.register(
    id="MyQuad-Velocity-Flat-IL-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}:MyQuadILEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}:MyQuadILPPOCfg",
    },
    disable_env_checker=True,
)

# 差异 4: 训练命令
# Isaac Lab: python -m isaaclab.app --task MyQuad-Velocity-Flat-IL-v0 --headless
# mjlab:     python scripts/train.py MyQuad-Velocity-Flat --headless
```

**双框架配置差异速查表**：

| 配置项 | mjlab | Isaac Lab | 注意事项 |
|--------|-------|-----------|---------|
| 模型格式 | MJCF (.xml) | USD (.usd) | 需要转换工具 |
| Entity 基类 | `EntityCfg` | `ArticulationCfg` | API 名不同但语义相同 |
| Scene 路径 | `/World/envs/env_.*/...` | `{ENV_REGEX_NS}/...` | Isaac Lab 用占位符 |
| Contact sensor | 自动（MuJoCo 原生） | 需 `ContactSensorCfg` | Isaac Lab 必须显式配置 |
| 注册 | `register_task()` | `gym.register()` | Isaac Lab 用 gymnasium 标准 |
| 训练脚本 | `scripts/train.py` | `scripts/rsl_rl/train.py` 或 `isaaclab.app` | 路径不同 |
| CLI 参数 | `--env.scene.num-envs` (tyro) | `--num_envs` (argparse/hydra) | 参数格式不同 |
| PhysX 求解器 | — | `solver_position_iteration_count` | mjlab 不需要（MuJoCo 统一求解） |
| 地面 | MJCF 内定义 | `GroundPlaneCfg()` | Isaac Lab 需要显式添加地面 |

### DIY 全流程检查清单 ⭐⭐⭐

以下是一个可打印的检查清单，覆盖从模型导入到策略训练的所有关键步骤。每个检查项有"通过"和"不通过"两种结果，不通过时指向对应的修复小节。

**Phase A：模型验证（训练前必须 100% 通过）**

| # | 检查项 | 验证方法 | 通过标准 | 不通过修复 |
|---|--------|---------|---------|-----------|
| A1 | MJCF/USD 编译无错误 | `mujoco.MjModel.from_xml_path()` | 不报错 | 检查 XML 语法 |
| A2 | Joint 数量正确 | `model.njnt` / `model.nu` | 与设计一致 | 检查 URDF→MJCF 转换 |
| A3 | 总质量合理 | `sum(model.body_mass)` | 与真机误差 <10% | 修正 inertial 参数 |
| A4 | 初始姿态无穿模 | `mj_forward` 后 `data.ncon` | 自碰撞 = 0 | 调 init joint_pos |
| A5 | Zero action 站稳 | 200 步后 base height | 下沉 < 3cm | 增大 stiffness |
| A6 | 关节名匹配 action 配置 | 打印 joint names vs regex | 全匹配 | 修正正则表达式 |
| A7 | Actuator 力矩足够 | gravity torque / max PD < 0.8 | 所有关节通过 | 增大 effort_limit |

**Phase B：环境验证（训练前必须 100% 通过）**

| # | 检查项 | 验证方法 | 通过标准 | 不通过修复 |
|---|--------|---------|---------|-----------|
| B1 | Smoke test | 创建 + reset + 10 步 | 无报错、无 NaN | 检查配置依赖 |
| B2 | Obs 维度正确 | 打印 obs shape | 与设计表一致 | 逐项检查 obs term |
| B3 | Action 维度正确 | 打印 action shape | = 受控关节数 | 检查 joint_names 正则 |
| B4 | Random agent reward 有方差 | std(reward) > 0.01 | 是 | §22.7 Bug 4 |
| B5 | Random agent 有 termination | episode count > 0 in 2000 steps | 是 | 检查 TerminationsCfg |
| B6 | Reset 后 obs 有变化 | 连续 reset 的 obs diff > 0 | 是 | §22.7 Bug 9 |
| B7 | Reward 各分项可分离打印 | 每项独立 print | 全部有数值 | 检查 term func 引用 |

**Phase C：训练早期验证（前 500 iterations）**

| # | 检查项 | 验证方法 | 通过标准 | 不通过修复 |
|---|--------|---------|---------|-----------|
| C1 | Reward 缓慢上升 | WandB reward 曲线 | 100 iter 内有上升趋势 | 检查 reward shaping |
| C2 | Entropy 缓慢下降 | WandB entropy 曲线 | 不陡降（>50% init） | 增大 init_noise_std |
| C3 | KL 在 target 附近 | WandB KL 曲线 | 0.005 - 0.02 | 调 LR / clip_param |
| C4 | Episode length 增长 | WandB ep_len 曲线 | 逐渐增长 | 检查 termination 阈值 |
| C5 | Viewer 中行为合理 | 视觉检查 | 有移动倾向 | 全面 debug |

```python
# ============================================================
# 文件 3/4: my_quad_train_cfg.py — PPO 训练配置
# ============================================================

class MyQuadPPORunnerCfg:
    seed = 42
    runner_type = "OnPolicyRunner"

    class policy:
        class_name = "ActorCritic"
        init_noise_std = 1.0
        actor_hidden_dims = [256, 256, 128]
        critic_hidden_dims = [256, 256, 128]
        activation = "elu"

    class algorithm:
        class_name = "PPO"
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4
        learning_rate = 1e-3
        schedule = "adaptive"
        desired_kl = 0.01
        gamma = 0.99
        lam = 0.95
        max_grad_norm = 1.0

    class runner:
        num_steps_per_env = 24
        max_iterations = 10000
        save_interval = 500
        log_interval = 10
        experiment_name = "my_quad_velocity"
        run_name = "flat_v1"
```

```python
# ============================================================
# 文件 4/4: __init__.py — 注册
# ============================================================

from mjlab.envs import register_task

register_task(
    task_id="MyQuad-Velocity-Flat",
    entry_point="mjlab.envs:ManagerBasedRlEnv",
    env_cfg_entry_point="my_quad.my_quad_env_cfg:MyQuadVelocityFlatEnvCfg",
    rsl_rl_cfg_entry_point="my_quad.my_quad_train_cfg:MyQuadPPORunnerCfg",
)
```

**训练命令**：

```bash
# 1. 训练
python scripts/train.py MyQuad-Velocity-Flat \
    --env.scene.num-envs=4096 \
    --headless

# 2. 可视化评估
python scripts/play.py MyQuad-Velocity-Flat \
    --num-envs=1 \
    --load-run=latest

# 3. 导出 ONNX
python scripts/export_onnx.py MyQuad-Velocity-Flat \
    --load-run=latest
```

### 关键配置参数调优指南 ⭐⭐

在上述完整代码中，有些参数需要根据你的具体机器人调整。以下是调优的优先级排序：

| 优先级 | 参数 | 默认值 | 调优方法 | 影响 |
|--------|------|--------|---------|------|
| P0 | `init_state.pos[2]`（初始高度） | 0.42 | viewer 中手动测试 | 高度错误 → 弹飞/穿地 |
| P0 | `init_state.joint_pos`（初始关节角） | 见代码 | zero agent 测试 | 关节角错误 → 倒塌 |
| P1 | `stiffness`/`damping`（PD 参数） | 20/0.5 | gravity torque 分析 | PD 弱 → 倒塌；PD 强 → 振荡 |
| P1 | `action.scale`（动作缩放） | 0.25 | random agent 测试 | 太大 → 关节冲击；太小 → 表达力不足 |
| P2 | `reward.weight`（奖励权重） | 见代码 | 500 iter 快速训练 | 权重失衡 → 行为偏移 |
| P2 | `sim.dt` / `decimation` | 0.005/4 | 自由落体测试 | dt 太大 → 物理不稳定 |
| P3 | `DR ranges`（随机化范围） | 见代码 | sim2sim 验证 | 范围太窄 → sim2real gap；太宽 → 训练不收敛 |
| P3 | PPO `learning_rate` | 1e-3 | KL divergence 监控 | LR 太大 → KL spike；太小 → 收敛慢 |

**P0 参数必须在训练前通过 zero agent 确认正确。P1 参数可以在 500 iterations 的快速训练中迭代调整。P2-P3 参数在大规模训练（10k iterations）中优化。**

> **跨领域类比**：参数调优的优先级就像修车——先确保轮子安装正确（P0），再调整悬挂和刹车（P1），然后优化发动机参数（P2），最后做空气动力学微调（P3）。不要在轮子还没装好的时候就去风洞做空气动力学测试。不幸的是，很多人在环境搭建中做的正是这件事——初始化都不对就开始调 reward 权重。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：复制整段代码后忘记改 joint names**
- 后果：action 引用了不存在的 joint name，action 维度变为 0，策略没有动作输出
- 正确做法：复制后第一件事是在你的 MJCF 中搜索所有 joint name，更新代码中的正则表达式

⚠️ **编程陷阱：ContactSensorCfg 的 body_names 和 MJCF 不匹配**
- 后果：feet_air_time reward 返回零（因为没有检测到脚接触），策略不学步态
- 正确做法：打印 `model.geom_bodyid` 和 body names 的映射，确认 ContactSensorCfg 引用的 body name 确实存在

### 练习

1. **[实践题]** 使用本节的完整代码，替换 robot MJCF 为 MuJoCo Menagerie 中的 Unitree Go2（`github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go2`），更新 joint names 和 init_state，跑通训练并达到 80% 的速度跟踪成功率。
2. **[扩展题]** 在完整代码的基础上，添加 terrain curriculum（参考 Ch13）：Phase 0 在平地上训练，Phase 1 加入随机台阶地形。实现 CurriculumCfg 并验证阶段推进。
3. **[对比题]** 分别用 PPO 默认配置和 AGILE 推荐的配置（actor [256,256,128], critic [512,256,128], LR 1e-3, entropy 0.005, $\gamma=0.99$）训练 5000 iterations，比较 reward 曲线和最终行为。AGILE 的配置是否更好？为什么？

### 从零到第一次成功训练的典型时间线

以下是一个有 Ch13-Ch21 实战经验的读者为一个新四足机器人搭建自定义环境的典型时间投入。如果你的实际进度明显慢于这个时间线，说明某个步骤卡住了——回到 §22.7 的排查总表定位问题。

| 时间 | 任务 | 预期产出 | 常见卡点 |
|------|------|---------|---------|
| Day 1 上午 | MJCF 导入 + MuJoCo viewer 验证 | 机器人能在 viewer 中站立 | 初始高度/关节角不对 |
| Day 1 下午 | EntityCfg + SceneCfg + Smoke test | `make()` 成功、obs 有值 | joint name 正则不匹配 |
| Day 2 上午 | ActionsCfg + ObsCfg + Zero agent | Zero agent 下不倒塌 | PD stiffness 不够 |
| Day 2 下午 | RewardsCfg + TerminationsCfg + Random agent | Random agent 有 termination、reward 有方差 | Reward 常数化 |
| Day 3 上午 | EventsCfg + CommandsCfg + 第一次训练（500 iter） | Reward 有上升趋势 | Command 范围太大 |
| Day 3 下午 | 观察 viewer + 调 reward 权重 + 训练 5000 iter | 基本的行走行为 | 原地抖动 |
| Day 4 | DR 引入 + 10000 iter 训练 | 鲁棒的速度跟踪 | DR 范围过宽 |
| Day 5 | Sim2sim 验证 + ONNX 导出 | 双框架行为一致 | 跨框架接触差异 |

**如果你是第一次做自定义环境**，实际时间可能是上述的 2-3 倍。这完全正常——大部分额外时间花在理解框架 API 和排查 §22.7 中列出的 bug 上。第二次做自定义环境时，你会发现速度提升 3-5 倍，因为大部分 bug 你已经见过一次了。

> **跨领域类比**：这和学习开车类似——第一次上路时你需要有意识地思考每个操作（看后视镜、松离合、打方向盘），花 30 小时才能熟练驾驶。但一旦形成肌肉记忆，开车变成了下意识行为。自定义环境搭建也是这样——第一次你需要对照 checklist 逐项检查每个配置，但做过 3-5 次后，大部分配置你可以直接写出正确版本，只在关键参数（如 init_state 和 PD stiffness）上用 zero agent 验证。

---

## 本章小结

| 知识点 | 核心内容 | 对应练习/实战 |
|--------|---------|-------------|
| DVI 方法论 | 分治-验证-集成，每个模块独立验证 | §22.1 练习 3 |
| mjlab 六步法 | EntityCfg → SceneCfg → Actions → Obs → Reward → Register | §22.2 全流程 |
| 验证三步走 | smoke test → zero agent → random agent | §22.2 验证代码 |
| Isaac Lab extension | HOVER 目录范式、Manager-Based vs Direct 选择 | §22.3 对比表 |
| 九步流水线 | CAD → URDF → MJCF/USD → 框架接入 → 训练 → sim2sim → ONNX | §22.4 完整流程 |
| HOVER 案例 | Oracle→DAgger 蒸馏、双重 mask 机制 | §22.5 代码精读 |
| HUSKY 案例 | mjlab + AMP、equality constraint 建模 | §22.5 实践题 |
| 十大 Bug | 从弹飞到 NaN 的系统化排查 | §22.7 排查表 |
| 完整代码模板 | 4 文件 / ~300 行完整可运行配置 | §22.8 代码 |
| 参数调优优先级 | P0→P3，先物理后学习 | §22.8 调优表 |

本章的核心工程经验可以浓缩为一句话：**自定义环境搭建的核心困难不在于写代码，而在于确保每一行配置都与物理模型和任务语义正确对齐**。DVI 方法论（分治-验证-集成）是应对这个困难的系统性方法——先让每个组件独立正确，再组合成完整系统。这种方法论不仅适用于 RL 环境搭建，也适用于任何复杂系统的工程开发。

回顾本章的教学路径：§22.1 建立了方法论框架（DVI），§22.2-22.3 在双框架中实践了这个框架，§22.4 将其扩展为完整的九步流水线，§22.5 通过 HOVER 和 HUSKY 两个顶会案例展示了研究级的实践，§22.6 补充了 EventsCfg 和 CommandsCfg 这两个容易被忽视的 Manager，§22.7 系统化地覆盖了 DIY 中最常见的陷阱，§22.8 给出了可以直接复制使用的完整代码模板。这个从"方法论 → 双框架实践 → 研究案例 → Bug 排查 → 完整模板"的递进结构，确保了不同水平的读者都能找到自己的切入点。

---

## 累积项目：本章新增模块

本章不增加新的累积项目编号——它本身就是一个"方法论章节"，教你怎么做 DIY。但本章的方法论直接服务于以下后续任务：

**从 Ch22 出发的项目路径**：

| 路径 | 起点 | 终点 | 本章贡献 |
|------|------|------|---------|
| 自定义四足 | §22.8 代码模板 | Ch13 terrain curriculum | EntityCfg + 验证三步走 |
| 自定义人形 | §22.3 HOVER 模板 | Ch14-15 motion imitation | Isaac Lab extension 模式 |
| 自定义操作 | §22.2 mjlab 六步法 | Ch17 lift cube 变体 | 自定义 reward term |
| 轮式双臂 | Ch21 + §22.7 | Ch23 sim2real | 双框架一致性验证 |
| 网球项目 | §22.5 HUSKY + Ch15 Tennis Launcher 环境 | Ch26-28 | MJCF 建模 + AMP 集成 |

每条路径的具体操作指南：
- 自定义四足：复制 §22.8 代码，替换 MJCF，按 P0→P3 调参
- 自定义人形：Fork HOVER 仓库，替换 H1 USD 为你的人形，跑 Oracle → Student pipeline
- 轮式双臂：从 Ch21 环境出发，用 §22.2 六步法添加自定义 reward/obs
- 网球项目：参考 HUSKY 的 mjlab + AMP 模式和 Ch15 Tennis Launcher 环境的球场建模

## 延伸阅读

| 资源 | 内容 | 难度 |
|------|------|------|
| Isaac Lab 官方 Tutorial "Creating a Manager-Based Base Environment" | 从 CartPole 出发的 step-by-step 教程 | ⭐ |
| Isaac Lab 官方 Tutorial "Creating a Direct Workflow RL Environment" | Direct 模式的 CartPole 教程 | ⭐ |
| NVlabs/HOVER 仓库 README + 代码 | Isaac Lab extension 的标杆范例 | ⭐⭐⭐ |
| TeleHuman/humanoid_skateboarding 仓库 | mjlab 研究项目的标杆范例 | ⭐⭐⭐ |
| AGILE 论文（arXiv:2603.20147）| 四阶段工业级 workflow | ⭐⭐⭐ |
| Isaac Lab Extension Development 官方文档 | Omniverse extension 机制详解 | ⭐⭐ |
| Legged Lab（github.com/Hellod035/LeggedLab） | Direct 模式的足式 RL 实现 | ⭐⭐ |
| MuJoCo Menagerie（google-deepmind/mujoco_menagerie） | MJCF 模型库（Go2/G1/ANYmal 等） | ⭐ |
| mjlab 官方文档 + task 注册示例 | mjlab 的 EntityCfg → Registry 完整流程 | ⭐⭐ |
| awesome-robot-descriptions（github.com/robot-descriptions） | 跨格式（URDF/MJCF/USD）机器人模型聚合索引 | ⭐ |
| awesome-loco-manipulation（github.com/aCodeDog/awesome-loco-manipulation） | 复合机器人 URDF 集合（Go2+Arx、B1+Z1 等） | ⭐⭐ |
| Isaac Lab Quickstart: Template Generator | 自动生成 extension 骨架的命令行工具 | ⭐ |

> **阅读建议**：如果你只有时间读一个外部参考，读 HOVER 仓库——它是目前 Isaac Lab extension 模式的最佳工程范例。如果你用 mjlab，读 HUSKY 仓库——它展示了如何用 mjlab 做复杂的器具建模和多阶段训练。两个仓库的 README 都写得非常详细，包含安装、训练、评估的完整命令。

## 🔧 故障排查手册

以下手册覆盖了本章 DIY 流程中最常见的故障场景。遇到问题时，先定位症状所在行，然后按排查步骤逐一检查。

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| `register_task` 后 `make()` 报 KeyError | task_id 拼写错误或 `__init__.py` 未被 import | 1. 打印已注册 task 列表 2. 检查 import 路径 3. 确认 `__init__.py` 存在 | §22.2 |
| MJCF 编译错误 "undefined joint" | actuator 引用了不存在的 joint name | 1. 打印 MJCF 中所有 joint name 2. 对照 actuator 配置 3. 修正正则表达式 | §22.2 |
| Isaac Lab USD 加载时 "prim not found" | prim_path 不包含 `{ENV_REGEX_NS}` 或路径错误 | 1. 在 Isaac Sim GUI 中查看 USD 的 prim 树 2. 对照 prim_path | §22.3 |
| Smoke test 通过但 zero agent 倒塌 | PD stiffness 不足或 init_state 不在平衡点 | 1. 打印 gravity torque vs max PD torque 2. 调整 stiffness 或 joint_pos | §22.7 Bug 2 |
| Random agent reward 全为零 | reward term 引用了错误的变量或返回了错误维度 | 1. 逐项打印 reward term 值 2. 检查返回形状 [B] | §22.7 Bug 4 |
| 训练开始后第 1 步 NaN | obs 中有除零操作（如 normalize by zero distance） | 1. 打印每个 obs term 值 2. 检查是否有 division 3. 加 clamp(min=1e-8) | §22.7 Bug 6 |
| 双框架 reward 差异 > 30% | 物理引擎接触模型差异 | 1. 运行一致性验证脚本 2. 对比 zero action base height 3. 调接触参数 | §22.4 Step 5 |
| 训练 5000 iter 后 reward 完全平坦 | reward 没有和动作/状态关联，或 exploration 不足 | 1. 打印 reward 分项 2. 检查 entropy 3. 增大 init_noise_std | §22.7 Bug 4,5 |
| ONNX 导出后策略行为随机 | obs normalization 没有 baked-in | 1. 对比 PyTorch 和 ONNX 输出 2. 检查 normalizer 是否包含 3. 手动 bake | §22.4 Step 9 |
| Isaac Lab extension 安装后 import 失败 | `setup.py` 未正确配置或 Isaac Sim AppLauncher 未初始化 | 1. 确认 `pip install -e .` 成功 2. 确认 import 在 AppLauncher 之后 3. 检查 Python path | §22.3 |

---

> **下一章预告**：Ch23 将把训练好的策略从仿真带到真实机器人上——这是整个 RL 工程闭环的最后一环。本章的 DVI 方法论和验证三步走在 sim2real 部署中同样适用：先验证 sim2sim（MuJoCo ↔ Isaac Lab），再验证 sim2real（仿真 → C++ SDK → 真机）。Ch22 的环境搭建质量直接决定了 Ch23 的 sim2real 成功率——如果仿真中的物理行为和真机差距过大（本章 §22.4 Step 5 的双框架一致性验证不通过），任何 DR 策略都无法弥补这个 gap。Ch23 会给出一个系统化的 Sim2Real Checklist（20 项检查），其中至少 8 项直接依赖本章搭建的环境的质量——包括接触参数的准确性、actuator 模型的保真度、obs 归一化的正确性等。



