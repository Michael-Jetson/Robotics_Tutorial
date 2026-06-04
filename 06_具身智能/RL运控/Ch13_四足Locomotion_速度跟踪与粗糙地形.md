# 13. 四足 Locomotion：速度跟踪与粗糙地形

> **本章定位**：Part II 建立了 RL 工程基础——observation、action、reward、DR、PPO、teacher-student。本章是 Part IV 的起点，把这些模块全部组合到一个完整的四足速度跟踪任务上。你将在 mjlab 和 Isaac Lab 中分别精读、训练和对比同一类任务，建立"从配置文件到策略行为"的完整因果链理解。
>
> **参考项目**：🔧 mjlab velocity（Go1/Go2 内置任务） · 🔧 Isaac Lab velocity（ANYmal-C/Go2 内置任务） · ✅ unitree_rl_mjlab（`github.com/unitreerobotics/unitree_rl_mjlab`） · ✅ basic-locomotion-isaaclab（`github.com/iit-DLSLab/basic-locomotion-isaaclab`）
>
> **机器人**：Go1/Go2（mjlab）、ANYmal-C/Go2（Isaac Lab） · **累积项目**：**A**

---

## 前置自测

📋 **答不出 ≥ 3 题 → 先回前置章节复习**

> 本章是 Part IV（单形态实战）的第一章，依赖 Part II（Ch04-Ch10）的全部基础知识。以下自测题覆盖最关键的前置概念。

1. **[Ch05]** mjlab 的 `ObservationManager` 如何区分 actor 和 critic 两组？`enable_corruption=True` 的物理语义是什么？为什么 critic 组通常不加 corruption？
2. **[Ch06]** `RewardManager` 如何把多个 reward term 聚合成标量？如果某个 reward term 的权重符号写反了（本应是负惩罚写成了正奖励），训练会出现什么现象？
3. **[Ch07]** RSL-RL 的 PPO 训练中，`terminated` 和 `truncated` 分别如何传递给 value bootstrap？如果 timeout 被当作 terminal state 处理，value function 会产生什么偏差？
4. **[Ch08]** mjlab 的 EventManager 支持四种触发模式（startup/reset/interval/step）。为什么 `foot_friction` randomization 通常用 startup 而不是 step？
5. **[MuJoCo]** 四足机器人的 `freejoint` 的 `qpos` 有 19 维（7 浮基 + 12 关节），`qvel` 却只有 18 维。为什么二者不相等？

## 本章目标

学完本章后，你应该能够：

1. **沿 task registry → env cfg → manager term → RL cfg 的完整链路**追溯 mjlab 和 Isaac Lab 中任意 velocity task 的数据流，发现 wiring 错误
2. **独立复现 Go1/Go2 flat 和 rough velocity task**，从 zero agent 到 large train 分四阶段验证
3. **解释 flat 和 rough 之间的 sensor/reward/termination/curriculum 差异**，说清 flat 为什么是 rough 的"减法版本"
4. **配置和解读 height scan 传感器**，理解 grid pattern、ray alignment 和 observation 维度之间的关系
5. **在双框架中执行对比实验**，用相同机器人（Go2）比较 mjlab 与 Isaac Lab 的训练曲线和策略行为
6. **用 Reward 四层框架（Tracking/Regularization/Style/Contact）系统性地分析和调整 reward 权重**，执行 reward ablation 实验并解读结果

---

## 13.1 足式机器人运动学概要与速度跟踪问题 ⭐⭐

> **这一节解决什么问题**：建立四足机器人的自由度心智模型，理解速度跟踪任务的 MDP 结构和设计动机。

### 动机：为什么四足速度跟踪是 RL locomotion 的标准入门

四足速度跟踪是足式 RL 的"Hello World"。它的目标看似简单——接收平面速度命令 $(v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}})$，让机器人按命令行走。但这个任务涵盖了几乎所有足式 RL 的核心工程挑战：浮动基座动力学、周期性接触切换、高维 action 空间、多目标 reward 设计、terrain adaptation、sim-to-real。如果你能在这个任务上从零跑通训练、理解每个配置项的因果关系、并在真机上部署成功，你就掌握了扩展到更复杂任务（人形、操作、运动模仿）的核心方法论。

### 四足 RL 运控的学术演进

在深入工程细节之前，先看看四足 RL 运控在学术上经历了怎样的演进。这条时间线帮助你理解本章的 velocity tracking 任务处在什么位置，以及后续章节要去往何方。

| 阶段 | 代表工作 | 平台 | 关键贡献 | 对应本书章节 |
|------|---------|------|---------|------------|
| **盲控制** | Hwangbo et al. 2019, *Science Robotics* | ANYmal | actuator network + sim-to-real，首次证明 RL 可以在真机上做 agile locomotion | Ch12 (Actuator) |
| **盲控制** | Lee et al. 2020, *Science Robotics* | ANYmal | 纯本体感知 zero-shot 到自然地形，teacher-student pipeline 的奠基 | Ch09, **Ch13** |
| **在线适应** | RMA (Kumar et al. RSS 2021) | Unitree A1 | base policy + adaptation module，实时在线适应地形/负载/磨损 | Ch09 |
| **命令条件化** | Walk These Ways (Margolis & Agrawal, CoRL 2022) | Unitree Go1 | Multiplicity of Behavior (MoB)，单策略支持多种步态参数化 | Ch13 延伸 |
| **大规模并行** | Rudin et al. CoRL 2021 (legged_gym) | ANYmal | GPU 并行训练 + terrain curriculum，从数小时降至数分钟 | **Ch13** |
| **感知控制** | Miki et al. 2022, *Science Robotics* | ANYmal | 高程图（elevation map） + 特权学习感知运动控制，DARPA SubT 1700m 零跌倒 | Ch18 |
| **高动态跑酷** | Extreme Parkour, ICRA 2024 | Unitree A1 | 视觉端到端穿越极端地形，2× 身高跳跃 | Ch18 |

每一阶段都继承前一阶段的训练基础设施（特权教师、DR、课程），在感知和动态性上递进。**本章处于"盲控制 + 大规模并行"阶段**——你不需要视觉（那是 Ch18），不需要 motion imitation（那是 Ch15），但你需要 terrain curriculum + teacher-student + DR 的完整工程链条。

**反事实推理：如果跳过本章直接做 Ch18（视觉控制）会怎样？** 你需要先训练一个 state teacher（用 height scan 的 privileged policy），然后 distill 到 depth student。但如果你连 state teacher 都跑不通——observation 没配对、reward 不涨、terrain curriculum 不工作——你根本无法开始视觉部分。本章就是确保你能稳定地训练 state teacher 的基础。

### 浮动基座 + 四条腿的自由度分析

四足机器人（以 Unitree Go1 为例）的自由度由两部分组成。**浮动基座**（floating base）有 6 个自由度——3 个平移 + 3 个旋转，用 `freejoint` 表示。在 MuJoCo 中，`freejoint` 的位置用 7 个数表示（3 平移 + 4 四元数），速度用 6 个数表示（3 平移速度 + 3 角速度）——这就是为什么 `qpos` 和 `qvel` 维度不同。**四条腿**每条有 3 个关节（hip abduction/adduction、hip flexion/extension、knee flexion），共 12 个 actuated joints。总自由度：6（浮基）+ 12（关节）= 18。

| 组件 | qpos 维度 | qvel 维度 | 说明 |
|------|----------|----------|------|
| 浮动基座位置 | 3 | 3 | 世界坐标系下 xyz |
| 浮动基座旋转 | 4（四元数） | 3（角速度） | 四元数 → 角速度差 1 维 |
| 12 个关节 | 12 | 12 | hinge joints |
| **总计** | **19** | **18** | qpos > qvel 因为四元数表示 |

这和计算机图形学中的欧拉角 vs 四元数问题有相似的根源：旋转的最小表示（3 个参数）存在万向节锁问题，四元数用 4 个参数消除了奇异性但引入了 1 个冗余维度。MuJoCo 的设计选择是 qpos 用四元数（无奇异性，适合积分），qvel 用角速度（3 维，更自然地参与动力学方程）。

### 运动学链 vs 质心动力学

理解四足机器人有两种互补视角。**运动学链视角**把每条腿看成一个 3R 串联机械臂——给定关节角度 $q_{\text{leg}}$，通过正运动学计算足端位置 $p_{\text{foot}}(q_{\text{leg}})$。这个视角适合分析单步的几何约束：脚能伸多远、髋关节角度极限、工作空间边界。**质心动力学视角**把整个机器人看成一个集中质量——关注质心轨迹、角动量、支撑多边形。这个视角适合分析整体稳定性：质心是否在支撑区域内、倾覆力矩有多大。

> **本质洞察**：RL 策略隐式地同时学习了两个层面的控制——运动学层面的足端轨迹生成和动力学层面的质心稳定维持。你不需要显式地分离这两个层面（MPC 方法通常需要），但你必须在 observation 和 reward 中提供足够的信号让策略"看到"这两个层面。joint position/velocity 提供运动学层面的信号，projected gravity 和 base velocity 提供动力学层面的信号。

**反事实推理：如果 observation 中去掉 projected gravity 会怎样？** 策略失去了对基座倾斜方向的直接感知。在平地上影响不大——重力方向几乎不变。但在斜坡上，策略无法区分"我在上坡"和"我在平地但腿伸得不一样"。它会用关节位置推断姿态（间接信号），但这种推断不准确且容易被噪声干扰。结果是坡地行走不稳，策略需要更长的训练时间来学会一个 implicit 的姿态估计器。

### 速度跟踪任务的 MDP 结构

Velocity task 的 MDP 完整定义如下：

| MDP 组件 | 具体内容 | 维度（Go1） |
|---------|---------|------------|
| **State** $s$ | 全部物理状态（qpos + qvel + contacts） | 远大于 obs |
| **Actor obs** $o^a$ | base_lin_vel + base_ang_vel + projected_gravity + joint_pos + joint_vel + actions + command + height_scan(rough) | ~47(flat) / ~234(rough) |
| **Critic obs** $o^c$ | actor obs + foot_height + foot_air_time + foot_contact + foot_contact_forces | ~270(rough) |
| **Action** $a$ | joint position offsets | 12 |
| **Command** $c$ | $(v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}})$ | 3 |
| **Reward** $r$ | tracking + style + regularization + contact | 标量 |
| **Termination** | fell_over / illegal_contact / time_out / out_of_bounds | bool |

这里有一个至关重要的设计决策：**command 必须在 actor observation 中**。command 是外部目标——同一个物理状态下，不同 command 要求不同 action。如果 actor 看不到 command，MDP 对它来说就是不完整的。这就像给出租车司机 GPS 定位但不告诉目的地——他知道在哪但不知道该往哪开。

**反事实推理：如果 actor 看不到 command 会怎样？** 策略只能学一个"平均"行为——对所有可能 command 的折中。由于训练中前进 command 出现概率最高，策略可能学到缓慢前进。tracking reward 偶尔碰巧匹配某个方向，给出正信号，但策略永远无法根据 command 切换行为。你在 tensorboard 上会看到 tracking reward 很低但其他 reward 正常，容易误判为 tracking 权重不够。

### Action 设计：为什么用 Joint Position Target

当前 velocity task 使用 `JointPositionActionCfg`：策略输出无量纲数值，经 `ACTION_SCALE` 缩放后叠加到默认关节姿态。关键参数 `use_default_offset=True` 让 zero action 对应默认姿态附近——这对 zero agent smoke test 至关重要。

```python
JointPositionActionCfg(
    entity_name="robot",
    actuator_names=(".*",),
    scale=GO1_ACTION_SCALE,      # rad 量级的位置偏移
    use_default_offset=True,      # action = default_qpos + output * scale
)
```

为什么不用 torque action？Torque control 给策略更大的控制自由度，但探索空间也更大——初始随机力矩可能让机器人瞬间飞出场景。Position target 通过底层 PD 控制器提供隐含的稳定性。这类似于自动驾驶中"输出方向盘角度"vs"输出轮胎力矩"的区别——前者有 power steering 兜底，后者需要策略自己学会力控。在 sim-to-real 中，position target 的 gap 也更小，因为实物关节伺服本身就是 position/velocity 控制模式。

**Action 链路的完整数据流**：

```
策略输出 a ∈ [-1, 1]^12
    ↓ × scale (0.25 rad)
位置偏移 δq = a × 0.25
    ↓ + default pose
目标位置 q_target = q_default + δq
    ↓ PD 控制器
力矩 τ = kp × (q_target - q_actual) + kd × (0 - q̇_actual)
    ↓ 力矩限制 clamp
施加力矩 τ_clamp = clamp(τ, -τ_max, τ_max)
    ↓ MuJoCo/PhysX physics step
关节运动 q_actual(t+dt)
```

**PD gains 的配置**：

| 参数 | Go1 典型值 | 来源 | 影响 |
|------|----------|------|------|
| kp (position gain) | 25-50 | MJCF actuator | 越大→跟踪越紧，但可能振荡 |
| kd (velocity gain) | 0.5-1.0 | MJCF actuator | 阻尼——抑制振荡 |
| τ_max (force limit) | 23.7 Nm (Go1 knee) | MJCF actuator | 物理限制 |

**action scale 的计算方法**：

```python
# 计算合理的 action scale
import mujoco

m = mujoco.MjModel.from_xml_path("go1.xml")
for i in range(m.nu):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    joint_id = m.actuator_trnid[i, 0]
    
    if m.jnt_limited[joint_id]:
        lo, hi = m.jnt_range[joint_id]
        full_range = hi - lo
        
        # 建议 scale = full_range × 30-50%
        suggested_scale = full_range * 0.35
        print(f"{name}: range=[{lo:.2f}, {hi:.2f}], "
              f"full={full_range:.2f}, suggested_scale={suggested_scale:.2f}")
```

Go1 的 12 个关节范围比较一致（大部分 ±1 到 ±2 rad），所以可以用统一 scale=0.25。但 G1（Ch14）的关节范围差异极大（ankle ±0.3 rad vs hip ±2.5 rad），必须使用 per-joint scale——这是从四足迁移到人形时最关键的配置变化之一。

**反事实推理：如果 action scale 设得太大（如 1.0）会怎样？** 策略的随机探索（初始 action 接近 N(0,1)）会产生 ±1.0 rad 的关节偏移——这可能直接打到限位。PD 控制器在限位处产生最大力矩，机器人被弹飞。如果太小（如 0.01）？策略几乎无法移动关节——即使输出最大值(1.0)也只偏移 0.01 rad，机器人站着不动。0.25 是经验中的 sweet spot——覆盖关节范围的约 15-25%，足以走路但不会打限位。

### 步态的物理本质与周期性

四足行走在物理上是一个周期性的接触切换过程。在一个完整步态周期中，每条腿在支撑相（stance phase）和摆动相（swing phase）之间交替。支撑相腿承担支撑和推进力，摆动相腿脱离地面向前摆动。不同步态的区别在于四条腿的相位关系：

| 步态 | 相位关系 | 同时支撑腿数 | 速度范围 | RL 中常见？ |
|------|---------|------------|---------|-----------|
| 步态 | Duty Factor | 相位 (FR/FL/HR/HL) | 同时支撑腿 | 典型速度 | RL 涌现条件 |
|------|------------|-------------------|-----------|---------|------------|
| Walk | 0.6-0.8 | 0/0.5/0.75/0.25 | 3-4 | 低速 (<0.5 m/s) | 低速命令 + 能耗惩罚 |
| **Trot** | ~0.5 | 0/0.5/0.5/0 (对角) | 2 | 中速 (0.5-2.0) | **最常见的涌现步态** |
| Pace | ~0.5 | 0/0.5/0/0.5 (同侧) | 2 | 中速 | 较少自然涌现 |
| Bound | ~0.4 | 0/0/0.5/0.5 (前后) | 1-2 | 高速 (2.0-3.0) | 高速命令 + 足底滞空奖励 |
| Gallop | 0.3-0.4 | 0/~0.1/0.5/~0.6 | 1-2 | 最高速 (>3.0) | 极高速命令 |
| Pronk | ~0.3 | 0/0/0/0 (全同步) | 0 或 4 | 跳跃 | 专用跳跃奖励 |

其中 Duty Factor 是单腿触地时间占整个步态周期的比例。值越小意味着飞行相越长、动态性越强。在 velocity tracking task 中，命令速度范围通常是 0-1.5 m/s，这个范围下 trot 是能量效率和稳定性的最优平衡点——所以你几乎总是看到策略涌现出 trot。

RL 策略通常自动学出 trot 步态——因为 trot 在中速范围内最稳定（始终有两条对角线腿支撑），而 velocity tracking reward 在这个速度范围内给出最强信号。你不需要显式定义步态——策略会从 reward 中自主发现最优步态。但你需要通过 reward 提供足够的信号来引导合理的步态特征：抬脚高度（foot clearance）、触地柔度（soft landing）、关节位姿回归（pose reward）等。

Walk These Ways (Margolis & Agrawal, CoRL 2022) 提出了另一种方式：将步态参数（频率、duty factor、相位偏移、步幅、步高）作为**额外的 command 输入**，让高层控制器在运行时实时调节。这种 Multiplicity of Behavior (MoB) 方法让单个策略支持多种步态风格，无需重新训练。本章的 velocity task 不使用 MoB——它只有 $(v_x, v_y, \omega_z)$ 三个 command，步态完全由策略自发涌现。但理解 MoB 的思路有助于你在 Ch14（人形）中设计更灵活的命令空间。

**跨领域类比**：RL 策略发现步态的过程，类似于优化器在损失面上发现极小值。步态模式是"吸引域"——一旦策略进入某个步态的邻域，reward 梯度会把它拉向该步态的中心。不同步态对应不同的极小值，reward 设计决定了哪个极小值被选中。如果 tracking reward 的速度范围主要在 0.5-1.5 m/s，trot 是最稳定的极小值；如果要求 3+ m/s，策略可能过渡到 bound/gallop。

**效率参考——Cost of Transport (CoT)**：CoT = 能耗 / (质量 × 重力 × 行走距离)，衡量行走效率。

| 系统 | CoT | 说明 |
|------|-----|------|
| 人类步行 | ~0.2 | 生物优化基准 |
| 四足机器人（传统控制） | 1.5-5.0 | 电机效率和控制策略相关 |
| 最优 RL 四足 (ANYmal) | ~0.43 | 已显著优于传统四足控制 |

CoT 在训练中通常不直接作为 reward（因为需要积分计算），但可以作为评估指标：如果你训练的策略 CoT > 5.0，说明动作效率极低，通常意味着 action rate 或 torque 惩罚不够。

### 平衡与稳定性——RL 如何隐式处理经典概念

经典腿足控制有一套成熟的稳定性分析工具。RL 不显式使用它们，但你需要理解它们在 reward 设计中的隐式等价——这帮助你在策略行为异常时知道该调什么。

| 经典概念 | 物理含义 | RL 中的隐式等价 |
|---------|---------|---------------|
| ZMP (Zero Moment Point) | 地面反力合力矩为零的点 | base roll/pitch 惩罚 ≈ ZMP 的软约束版本 |
| Support Polygon | 所有接触点的凸包 | 四足天然宽（4 点），人形极窄（2 点） |
| Capture Point | 必须踏到此点才能阻止跌倒 | 隐含在步态自动调整中 |
| CoM (Center of Mass) | 全身质心位置 | base height tracking + projected gravity |

RL 策略通过 reward 隐式维持稳定：base 姿态惩罚（roll/pitch 偏差）≈ 要求 ZMP 在支撑多边形内；角速度惩罚 ≈ 抑制动态不稳定；base 高度跟踪 ≈ 防止蹲下或过度伸展。这比传统方法更灵活（不需要显式建模），但也更难解释为什么某个策略能稳定。

**工程含义**：当策略在侧向风或推力扰动下失稳时，不要去加一个新的"anti-fall reward"——先检查现有的 base orientation penalty 和 angular velocity penalty 的权重是否足够大。这两个 reward 联合起来就是 ZMP 稳定性的 RL 版本。

**四足 vs 人形的关键差异**：四足的支撑多边形面积比人形大一个数量级——即使 trot 步态（两条对角腿）也有足够的支撑面积。这意味着四足速度跟踪在平衡上相对"容易"，策略可以把更多精力放在 tracking 精度和能量效率上。Ch14（人形 Locomotion）将把同样的框架应用到支撑面极窄的双足上，你会发现角动量管理成为核心挑战。

### 状态估计——为什么 base_lin_vel 是 privileged

在 MDP 结构中我们看到 base_lin_vel 在 critic obs 中但不在 actor obs 中（某些配置下 actor 也有，但带噪声）。这个设计决策背后是一个重要的物理事实：**基座线速度在真机上无法直接测量**。

没有传感器能直接输出全局坐标系下的线速度。GPS 精度不够（~1m），光流/VIO 在腿足振动下误差大，轮式里程计不适用。标准估计管线是：

```
IMU（加速度计 + 陀螺仪）
  → 积分得速度估计（drift 严重）
  → + 腿部正运动学（假设接触脚不滑移）
  → + EKF 融合（接触状态作为门控信号）
  → 得到 base_lin_vel 估计值
```

Go1/Go2 的特殊性：不像 ANYmal 有足端六维力传感器，Go1/Go2 只能从关节力矩反推接触状态——这个估计更嘈杂。所以 actor observation 中通常用 base_ang_vel（陀螺仪直接测量，噪声低，~0.01 rad/s 量级）+ projected_gravity（加速度计推算，中等噪声）替代 base_lin_vel。

RMA (Kumar et al., RSS 2021) 提出了一种优雅的解决方案：用 proprioceptive history（过去若干步的关节位置/速度/动作序列）通过一个 adaptation module 隐式估计环境参数（包括速度）。base policy 在 100 Hz 运行，adaptation module 在 10 Hz 运行。这种方法不需要显式的状态估计器，但要求训练时的 DR 覆盖足够宽的参数范围，让 adaptation module 学会区分不同环境条件。

**工程建议**：在本章的 velocity task 中，flat 任务可以把 base_lin_vel 放入 actor obs（有噪声）——平地上估计精度尚可。但 rough terrain 任务中，建议把 base_lin_vel 只放 critic，actor 用 proprioceptive history 替代。这为后续 Ch23（Sim2Real）的部署铺路。

### Asymmetric Actor-Critic

velocity task 采用非对称 actor-critic（Pinto et al., "Asymmetric Actor Critic for Image-Based Robot Learning," RSS 2018）：actor 看带噪声的可部署信号，critic 看更多更干净的 privileged 信号。回顾 Ch09（Teacher-Student 与 Privileged Learning）：critic 需要更多信息提供准确 value estimate，actor 在训练时就学会从带噪声输入中提取信息。critic 额外看到的 privileged 信号包括：foot height（精确足端高度）、foot air time（空中持续时间）、foot contact forces（接触力大小）。这些在真机上要么不可用、要么有较大噪声。

这种设计在足式 RL 中几乎是标准配置。为什么？因为 locomotion 的 reward 函数需要接触信息（脚是否着地、接触力多大、滑移多少），这些信息在仿真中可以精确获取，但在真机上只能通过力传感器或估计器近似获取。如果 actor 在训练时就依赖精确接触信息，部署时切换到噪声信号会导致性能骤降。让 actor 从一开始就用带噪声的 proprioception 训练，策略就学会了从不完美信号中提取足够信息。

> **双重解读**：非对称 actor-critic 可以从两个角度理解。从 RL 角度：critic 是辅助训练的工具，部署时不需要，所以给它更多信息只会帮助训练不会增加部署负担。从 teacher-student 角度：critic 充当了隐式的"教师"——它用 privileged 信息评估状态价值，通过 advantage 信号引导 actor（"学生"）的更新方向。这两个角度的结合解释了为什么 asymmetric AC 在 sim-to-real 中如此有效：它在训练时就建立了 actor 的"信息贫乏抵抗力"。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：action scale 照搬其他机器人的值**。Go1 的关节范围和 Go2 不同，G1（人形）更是差异巨大。一个对 Go1 合适的 scale 可能让 Go2 的膝关节打到限位。正确做法是查看每个机器人的关节限位范围，设置 scale 使得 $[-1, 1]$ 的 action 覆盖合理的关节偏移区间（通常是限位范围的 30-50%）。

💡 **概念误区：认为 12 个关节的 action 空间"很小"**。维度小不代表搜索容易。12 维连续空间的体积随维度指数增长，关键是好的初始化（default pose offset）和 reward shaping 把搜索引向合理区域。不给 default offset 的话，初始策略输出接近零的 raw target 角度，机器人以扭曲姿态开始探索。

🧠 **思维陷阱：认为 critic 信息越多越好**。critic 加入不相关的信号（比如遥远目标物体的位置）不会帮助 value estimation，反而增加拟合难度。critic 的 privileged 信息应该和 reward 函数直接相关——你 reward 什么，critic 就应该能看到什么。

⚠️ **工程陷阱：flat 和 rough 共用同一套 actor observation**。flat 任务不需要 height scan（地面是平的），但如果 actor obs 中保留了 height_scan term 却没有对应的 sensor，启动就会 crash。反过来，rough 任务如果漏掉 height_scan，策略会"瞎走"——在平地上可能还行，但在台阶/斜坡上立即失败。

### 练习

1. **[计算题]** Go2 有 12 个 actuated joints。计算 Go2 的 qpos 和 qvel 总维度。如果 actor observation 包含 base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3) + joint_pos(12) + joint_vel(12) + actions(12) + command(3)，flat 任务的 actor obs 总维度是多少？
2. **[分析题]** 对比 joint position target 和 joint velocity target 两种 action 空间。分别列出 2 个优点和 2 个缺点。说明在 sim-to-real 场景下为什么 position target 更常用。
3. **[设计题]** 如果你要为一个有弹性脚掌（compliant foot）的四足机器人设计 observation，除了标准的 proprioception，你还需要什么额外的传感信号？为什么？
4. **[跨章综合题，Ch05+Ch09+Ch13]** 假设你在训练一个 Go2 rough terrain policy。critic obs 包含 height_scan + foot_contact_forces + base_lin_vel（无噪声）。actor obs 只包含 proprioception + command。解释：(a) 为什么 critic 能帮助 actor 学到更好的策略，即使 actor 看不到地形？(b) 如果把 height_scan 从 critic 移到 actor，对训练和部署分别有什么影响？(c) RMA 的 adaptation module 在这个场景中扮演什么角色？

---

上节建立了四足机器人的自由度模型和 velocity task 的 MDP 结构。但知道 MDP 的数学定义和知道如何在框架中配置它是两回事——这正是本节的主题。

## 13.2 mjlab Velocity Task 逐行精读 ⭐⭐⭐

> **这一节解决什么问题**：从 task registry 到每个 manager term，建立 mjlab velocity task 的完整源码地图。

### 链路阅读法

精读一个 task 最有效的方法不是从某个 reward 函数开始，而是沿数据流链路从入口追到出口。完整链路如下：

```
Task Registry → Entity → Scene → Sensors → Managers → RL cfg → CLI → Logs → Play
```

这类似 C++ 项目中从 `main` 函数追到接口实现——你不是在读某个函数，而是在追踪数据的完整生命周期。如果只读 reward 函数，会漏掉 sensor wiring；如果只读 train.py，会漏掉 robot-specific overrides。

### 核心源码地图

| 文件路径 | 角色 |
|---------|------|
| `src/mjlab/tasks/velocity/config/go1/__init__.py` | task 注册入口 |
| `src/mjlab/tasks/velocity/config/go1/env_cfgs.py` | Go1 rough/flat env cfg |
| `src/mjlab/tasks/velocity/config/go1/rl_cfg.py` | Go1 PPO runner cfg |
| `src/mjlab/tasks/velocity/velocity_env_cfg.py` | robot-agnostic base cfg |
| `src/mjlab/tasks/velocity/mdp/rewards.py` | velocity reward 函数 |
| `src/mjlab/tasks/velocity/mdp/terminations.py` | termination 条件 |
| `src/mjlab/tasks/velocity/mdp/velocity_command.py` | command sampler |
| `src/mjlab/tasks/velocity/mdp/curriculums.py` | curriculum |

### 从 task id 到 env cfg

`__init__.py` 中的 `register_mjlab_task()` 绑定了五个要素：

```python
register_mjlab_task(
    task_id="Mjlab-Velocity-Rough-Unitree-Go1",
    env_cfg=unitree_go1_rough_env_cfg,      # 训练 env cfg 工厂函数
    play_env_cfg=unitree_go1_rough_play_cfg, # 播放 env cfg（关闭 noise/push）
    rl_cfg=unitree_go1_rl_cfg,              # RSL-RL runner cfg
    runner_cls=VelocityOnPolicyRunner,       # 可选自定义 runner
)
```

play cfg 和 train cfg 分离的意义在于：play 时关闭 observation corruption、移除 random push event、rough play 用固定 terrain 而非 curriculum terrain。如果 play 和 train 行为不一致，先检查这两个 cfg 的差异。

### base cfg 的模板方法模式

Go1 rough cfg 的第一行调用 `make_velocity_env_cfg()`，它创建 robot-agnostic 的 base cfg，定义所有 manager 的骨架结构但把 robot-specific 的名字留空。Go1 cfg 随后填入具体的 entity wiring、sensor frame names、reward site names、action scale。

这和 C++ 中的模板方法设计模式完全对应——base class 定义算法骨架，derived class 填入 hook。区别是这里用 Python dataclass 和 dict 组合实现。这个设计让同一个 base cfg 能适配 Go1、Go2、ANYmal 等不同机器人——只需替换 robot-specific 部分。

### Observation 配置精读

**actor terms**：base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3) + joint_pos(12) + joint_vel(12) + actions(12) + command(3) + height_scan(187, rough only)。actor group 设置 `concatenate_terms=True`（所有 term 拼成一维向量）、`enable_corruption=True`（添加传感器噪声）。

**noise 幅度**的设置反映真实传感器特性：

| term | noise 幅度 | 对应真实传感器 |
|------|-----------|-------------|
| base_lin_vel | ±0.5 m/s | IMU + 腿部运动学估计 |
| base_ang_vel | ±0.2 rad/s | IMU 陀螺仪 |
| projected_gravity | ±0.05 | IMU 加速度计 |
| joint_pos | ±0.01 rad | 关节编码器 |
| joint_vel | ±1.5 rad/s | 编码器差分 |
| height_scan | ±0.1 m | elevation map 精度 |

**critic terms**：复制 actor 的所有 term（但 `enable_corruption=False`），额外加入 foot_height、foot_air_time、foot_contact、foot_contact_forces 四个 privileged term。这些 privileged 信号在真机上不可用或噪声较大，但能帮助 critic 更准确地估计状态价值。

### Reward 配置精读

reward 设计是 velocity task 中最需要仔细理解的部分。每个 term 的权重不是随意设定的——它反映了设计者对行为优先级的判断：

| 类别 | reward term | 权重 | 数据来源 | 设计意图 |
|------|-----------|------|---------|---------|
| tracking | `track_linear_velocity` | +2.0 | command + robot vel | 核心目标：跟踪线速度 |
| tracking | `track_angular_velocity` | +2.0 | command + robot ang vel | 核心目标：跟踪角速度 |
| style | `upright` | +1.0 | projected gravity | 保持基座水平 |
| style | `pose` | +1.0 | joint pos + default | 关节回归默认姿态 |
| regularization | `dof_pos_limits` | -1.0 | joint pos | 远离关节限位 |
| regularization | `action_rate_l2` | -0.1 | action history | 抑制动作抖动 |
| contact | `foot_clearance` | -2.0 | foot site height | 鼓励抬脚 |
| contact | `foot_slip` | -0.1 | foot site velocity | 惩罚打滑 |

tracking reward 权重最高（+2.0），因为这是任务的核心目标。style reward 权重中等（+1.0），引导自然姿态。regularization 和 contact reward 使用负权重作为惩罚。

### mjlab Go1 Velocity RewardsCfg 的完整注册

在 mjlab 中，上面的 reward table 对应以下 Python 代码。理解这段代码是后续 Ch14（人形）修改 reward 的基础：

```python
# src/mjlab/tasks/velocity/config/go1/env_cfgs.py（简化版）
def unitree_go1_flat_env_cfg():
    cfg = make_velocity_env_cfg()
    
    # === Entity wiring ===
    cfg.scene.entity.asset_path = "unitree_go1/xmls/go1.xml"
    cfg.scene.entity.default_joint_pos = GO1_DEFAULT_POSE  # 12 维
    
    # === Action ===
    cfg.actions.joint_pos.scale = 0.25  # 统一 scale（Go1 所有关节范围类似）
    
    # === Rewards: 四层框架 ===
    cfg.rewards = {
        # --- Tracking 层 ---
        "track_linear_velocity": RewardTermCfg(
            func=track_lin_vel_xy_exp,
            weight=2.0,
            params={"command_name": "twist", "sigma": 0.25},
        ),
        "track_angular_velocity": RewardTermCfg(
            func=track_ang_vel_z_exp,
            weight=2.0,
            params={"command_name": "twist", "sigma": 0.25},
        ),
        
        # --- Style 层 ---
        "upright": RewardTermCfg(
            func=upright_reward,
            weight=1.0,
        ),
        "pose": RewardTermCfg(
            func=joint_deviation_l1,
            weight=1.0,
            params={"asset_cfg": SceneEntityCfg("entity")},
        ),
        "feet_air_time": RewardTermCfg(
            func=feet_air_time_reward,
            weight=0.5,
            params={"sensor_name": "feet_contact", "threshold": 0.5},
        ),
        
        # --- Regularization 层 ---
        "action_rate_l2": RewardTermCfg(
            func=action_rate_l2,
            weight=-0.1,
        ),
        "dof_acceleration": RewardTermCfg(
            func=dof_accel_l2,
            weight=-0.0025,
        ),
        "joint_torques": RewardTermCfg(
            func=joint_torques_l2,
            weight=-0.0001,
        ),
        "dof_pos_limits": RewardTermCfg(
            func=dof_pos_limits_penalty,
            weight=-1.0,
        ),
        "linear_velocity_z": RewardTermCfg(
            func=lin_vel_z_l2,
            weight=-2.0,
        ),
        "angular_velocity_xy": RewardTermCfg(
            func=ang_vel_xy_l2,
            weight=-0.05,
        ),
        
        # --- Contact/Safety 层 ---
        "foot_clearance": RewardTermCfg(
            func=foot_clearance_penalty,
            weight=-2.0,
            params={"min_height": 0.03, "sensor_name": "foot_sites"},
        ),
        "foot_slip": RewardTermCfg(
            func=foot_slip_penalty,
            weight=-0.1,
            params={"sensor_name": "feet_contact"},
        ),
        "undesired_contacts": RewardTermCfg(
            func=undesired_contacts_penalty,
            weight=-1.0,
            params={"sensor_name": "contact_sensor",
                    "body_names": ("thigh_FL", "thigh_FR",
                                   "thigh_RL", "thigh_RR")},
        ),
    }
    
    return cfg
```

**代码解读要点**：

1. 每个 `RewardTermCfg` 包含三部分：`func`（计算函数）、`weight`（权重）、`params`（传给函数的额外参数）。修改 reward 只需改这三项，不需要改底层的 RewardManager。

2. `sigma=0.25` 出现在 tracking reward 的 params 中——这就是 exponential kernel 的 σ 参数。如果你觉得策略跟踪不够精确，可以减小 σ（如 0.15）；如果训练初期 reward 太稀疏，增大 σ（如 0.4）。

3. `weight=-2.0` 的 `foot_clearance` 是所有惩罚中绝对值最大的——这强迫策略抬脚而不是拖行。拖脚在仿真中可能"工作"（利用接触弹性滑行），但真机上会磨损脚底。

4. 总共 **13 个** reward terms。Ch14（人形）将在此基础上新增 4 个人形特有 terms（base_height、variable_posture、angular_momentum、self_collision），达到约 17 个。

**从这段代码到"策略不走"的排查**：如果策略不走，第一步检查 `track_linear_velocity` 和 `track_angular_velocity` 的 weight 是否 > 0（确认是正奖励不是惩罚）。第二步检查 `params` 中的 `command_name` 是否与 obs 中的 command group 名字一致。这两个错误分别对应"策略没有跟踪目标"和"策略看不到目标"。

### Actor/Critic Observation 的完整配置

Go1 velocity task 的 observation 分为 actor（可部署）和 critic（仅训练用）两组：

```python
# Go1 flat velocity actor observation
cfg.observations.actor = ObservationGroupCfg(
    enable_corruption=True,   # 加噪声模拟真机传感器
    concatenate_terms=True,
    terms={
        "base_lin_vel": ObsTerm(
            func=base_lin_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.1),
        ),  # 3 维
        "base_ang_vel": ObsTerm(
            func=base_ang_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.2),
        ),  # 3 维
        "projected_gravity": ObsTerm(
            func=projected_gravity,
            noise=GaussianNoiseCfg(mean=0.0, std=0.05),
        ),  # 3 维
        "command": ObsTerm(
            func=generated_commands,
            params={"command_name": "twist"},
        ),  # 3 维: (vx_cmd, vy_cmd, wz_cmd)
        "joint_pos": ObsTerm(
            func=joint_pos_rel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),
        ),  # 12 维
        "joint_vel": ObsTerm(
            func=joint_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=1.5),
        ),  # 12 维
        "last_action": ObsTerm(func=last_action),  # 12 维
    },
)
# actor obs 总维度: 3+3+3+3+12+12+12 = 48

# critic 额外添加 privileged terms
cfg.observations.critic = ObservationGroupCfg(
    enable_corruption=False,  # critic 看干净数据
    concatenate_terms=True,
    terms={
        **cfg.observations.actor.terms,  # 复制 actor 所有 terms
        "foot_height": ObsTerm(
            func=foot_position_in_base_frame,
            params={"foot_sites": ("FL", "FR", "RL", "RR")},
        ),  # 4 × 3 = 12 维
        "foot_contact_forces": ObsTerm(
            func=contact_forces,
            params={"sensor_name": "feet_contact"},
        ),  # 4 × 3 = 12 维
    },
)
# critic obs 总维度: 48 + 12 + 12 = 72 (rough 地形会更大)
```

**为什么 actor 和 critic 的 obs 不同？** 这是 Ch09（Privileged Learning）的核心思想在 velocity task 中的应用。Critic 看到更多更干净的信息（foot height、contact forces），能给出更准确的 value estimate。Actor 只看到可部署的传感器信号（IMU + 编码器），学会从这些有限信号中推断环境状态。

**噪声参数的物理依据**：
- `base_ang_vel` 的 std=0.2 rad/s ≈ 11.5°/s——这比真机 IMU 的噪声（~0.01 rad/s）大很多。有意加大噪声是为了让策略对传感器误差更鲁棒。
- `joint_pos` 的 std=0.01 rad ≈ 0.57°——编码器精度通常 < 0.1°，但 DR 中的电机零位偏移可能引入更大的系统误差。

**tracking reward 的数学形式**：`track_linear_velocity` 使用 exponential kernel：

$$r_{\text{track}} = \exp\left(-\frac{\|v_{\text{actual}} - v_{\text{cmd}}\|^2}{\sigma^2}\right)$$

为什么用 exponential 而不是简单的 $-\|v - v_{\text{cmd}}\|^2$？L2 距离在远离目标时给出很大的负值，策略可能选择"不动"来回避大惩罚。exponential kernel 的值域是 $[0, 1]$，远离目标时快速衰减到零但不产生大惩罚——策略得到的信号是"没有奖励"而非"被惩罚"，这在 PPO 的 advantage 计算中有本质区别。$\sigma$ 参数控制精度容忍度：$\sigma$ 小时只有非常接近目标才有显著奖励，$\sigma$ 大时奖励更宽容。

**reward 之间的交互关系**：tracking 和 contact reward 不是独立的——`foot_slip` penalty 惩罚脚在地面上滑动，但抑制滑动的代价可能是降低 tracking accuracy（策略为了不打滑而走得更保守）。调参原则：先固定 tracking weight，再调 penalty 的相对比例。

### Reward 设计的四层工程框架

上面的 reward 表格可以按功能分为四层。这个四层框架被 extreme-parkour (ICRA 2024)、walk-these-ways (CoRL 2022)、mjlab velocity task 共同验证，是足式 RL reward 设计的标准组织方式：

| 层级 | 目的 | 典型 terms | 权重量级 | 调参策略 |
|------|------|-----------|---------|---------|
| **Tracking** | 跟踪命令 | `lin_vel_xy`, `ang_vel_yaw` | 1.0-5.0 | 固定不动，作为 baseline |
| **Regularization** | 平滑动作 | `action_rate`, `dof_accel`, `torque`, `dof_pos_limits` | 0.01-1.0 | 从小值开始，观察动作质量后调大 |
| **Style** | 步态美学 | `feet_airtime`, `upright`, `pose`, `hip_pos` | 0.1-2.0 | 决定步态风格，主观性最强 |
| **Contact/Safety** | 避免伤害 | `foot_slip`, `undesired_contacts`, `termination_height` | 0.1-1.0 | 定义行为红线 |

**四层的设计哲学**：Tracking 是你要策略做什么（目标），Regularization 是你不要策略做什么（约束），Style 是你希望策略怎么做（偏好），Contact/Safety 是策略绝对不能做什么（红线）。调参时四层独立扰动——如果 tracking 很好但步态很丑，只调 Style 层；如果策略抖动严重，只调 Regularization 层。

**Reward weight 调参的工程方法**：
1. **先跑一个"只有 tracking"的 baseline**（关闭所有其他 reward），观察策略能学到什么（通常是一个丑但能走的步态）。
2. **逐层添加 reward**：先加 Regularization → 观察动作变平滑；再加 Style → 观察步态变自然；最后加 Contact/Safety。
3. **记录每层添加后的指标变化**：tracking error、episode length、action jerk、视觉观察。
4. **exponential kernel 的 σ 选择**：σ 控制"精度容忍度"。`lin_vel_xy` 通常 σ=0.25（允许 ±0.25 m/s 误差仍有较高 reward），`ang_vel_yaw` 通常 σ=0.25。σ 过小（<0.1）会让 reward 信号过于稀疏，策略难以学到初始的粗略跟踪。
5. **step_dt scaling**：所有 reward 应该乘以 `dt`（control step 时间），使得不同 `control_dt` 下的 return 可比。mjlab 和 Isaac Lab 的 reward manager 默认会做这个 scaling。

`pose` reward 让关节回归默认姿态，防止策略学到"蹲着走"或"扭曲着走"的丑步态。但过强的 pose reward 会限制策略的表达能力——策略不敢偏离默认姿态，就无法做出大幅度的步态动作。这是 reward 设计中经典的"自由度 vs 规范性"权衡。

**反事实推理：如果把所有 reward 权重设为 1.0 会怎样？** 不同 reward 项的数值量级差异很大。`track_linear_velocity` 的值域约 $[0, 1]$（归一化后），而 `foot_clearance` 惩罚在某些步态下可能达到较大值。如果权重都是 1.0，某个数值量级大的惩罚项会主导优化，策略可能学会"不动"来最小化惩罚。正确做法是先单独运行每个 reward term 几个 iteration，观察其数值范围，再设定权重。

### Termination 配置

| 条件 | 类型 | 含义 | flat | rough |
|------|------|------|------|-------|
| `time_out` | truncation | episode 超时 | ✅ | ✅ |
| `fell_over` | terminal | 倾斜超限 | ✅ (70°) | ❌ |
| `illegal_contact` | terminal | 非法接触 | ❌ | ✅ |
| `out_of_terrain_bounds` | truncation | 走出地形 | ❌ | ✅ |

为什么 rough 删除 `fell_over` 而用 `illegal_contact`？在粗糙地形上，机器人在斜坡上的倾斜角可能远超平地的合理范围。一个 30° 斜坡上的合理姿态在平地上会被判为"摔倒"。`illegal_contact` 更精确——只有当大腿或小腿碰到地面才判定失败，这才是真正的失败信号。

### Flat 与 Rough 的减法关系

Go1 flat cfg 不是从零构建的，而是从 rough cfg 做减法：

```python
def unitree_go1_flat_env_cfg():
    cfg = unitree_go1_rough_env_cfg()
    # 减法操作
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None
    del cfg.scene.sensors["terrain_scan"]
    del cfg.observations.actor.height_scan
    del cfg.observations.critic.height_scan
    del cfg.curriculum["terrain_levels"]
    del cfg.terminations["illegal_contact"]
    cfg.terminations["fell_over"] = FellOverTerminationCfg(...)
    # ... 更多减法
```

> **本质洞察**：task variant 不只是改 terrain type。它要同步修改 observation、reward、termination 和 curriculum。任何不一致都会导致训练信号与物理环境脱节——比如 flat 保留 terrain_scan 会白算 raycast 拖慢 10-20% 训练速度，rough 保留 fell_over 会在合理坡度下频繁 reset。

### RL Config 精读

Go1 PPO 配置的关键参数及其设计意图：

```python
# actor/critic 网络
actor_hidden_dims = (512, 256, 128)   # 逐层递减
critic_hidden_dims = (512, 256, 128)
activation = "elu"                     # 比 ReLU 更适合 RL（非零梯度）
init_noise_std = 1.0                   # 初始探索噪声

# PPO 超参
clip_param = 0.2                       # surrogate objective clipping
entropy_coef = 0.01                    # 防止过早收敛
num_learning_epochs = 5                # 每个 rollout batch 更新 5 次
num_mini_batches = 4                   # 4 个 mini-batch
learning_rate = 1e-3                   # 初始学习率
schedule = "adaptive"                  # 基于 KL 自适应调整
desired_kl = 0.01                      # 目标 KL divergence
gamma = 0.99                           # 折扣因子
lam = 0.95                             # GAE lambda

# Runner
num_steps_per_env = 24                 # 每个环境收集 24 步
max_iterations = 10000                 # 最大迭代数
save_interval = 50                     # 每 50 iterations 保存
```

**为什么 activation 用 ELU 而不是 ReLU？** ReLU 在负区间梯度为零（"dead neuron"问题）。ELU 在负区间有非零梯度（$\alpha (e^x - 1)$），让 RL 中的策略网络在探索阶段不容易"卡死"。这对初始训练阶段尤其重要——初始策略的输出分布可能让很多 neuron 处于负区间。

**为什么 num_steps_per_env = 24？** 这决定了每个 rollout 的长度。太短（如 4）导致 GAE 的 bootstrap 主导，value function 的误差传播快。太长（如 128）增加了 GPU 内存需求（需要存储更长的 rollout buffer），且可能跨越多个 command 切换周期。24 步 × 0.02s/step ≈ 0.48 秒的 rollout 长度，覆盖了大约 1-2 个步态周期，是足式 RL 的典型选择。

**adaptive learning rate schedule** 的工作方式：如果 KL divergence 超过 `desired_kl` 的 2 倍，学习率减半；如果低于 `desired_kl` 的 0.5 倍，学习率加倍。这确保策略更新的幅度不会太大（避免训练不稳定）也不会太小（避免训练过慢）。

### 从 legged_gym 到 Manager-Based 的工程演进

上述配置结构——分离的 env cfg / rl cfg / reward functions / termination functions——并不是天然存在的。理解它的来历有助于你把握为什么 mjlab 和 Isaac Lab 的 Manager-Based 架构是当前的标准。

**legged_gym 时代（2021-2023）**：Rudin et al. 的 legged_gym (arXiv 2109.11978) 把 obs / reward / event 全部写在单个 `LeggedRobot` 类中。`compute_observations()` 方法手动拼接 tensor，`compute_reward()` 方法调用一系列内联函数。这种单体设计在最初很直观，但每加一个新机器人就必须 fork 整个类——Go1 一套代码、ANYmal 一套代码、Go2 又一套。reward 改一个权重需要在 Python 文件中翻找数百行 hardcoded 逻辑。3 个月后不同 fork 的代码开始严重分叉，bug fix 无法传播。

**Manager-Based 架构（2024-）**：Isaac Lab 引入、mjlab 继承的 Manager-Based 架构把每个组件（obs / reward / event / termination / curriculum）拆成独立的 `TermCfg`，通过配置文件组合。这就像从"所有逻辑写在一个 God Class 里"迁移到"Strategy Pattern + Factory Pattern"。每个 reward term 是一个独立函数，通过 `RewardTermCfg(func=..., weight=..., params=...)` 注册。新增机器人只需写一个 cfg 文件覆盖 robot-specific 名字，不需要 fork 任何逻辑代码。

**工程含义**：如果你在阅读 2021-2023 的论文代码（legged_gym、walk-these-ways、humanoid-gym），它们的 reward 和 obs 都是 hardcoded 在类方法中的。这和 mjlab 的 cfg-based 方式在功能上等价，但代码组织完全不同。阅读旧代码时不要被 inline reward 函数吓到——它们的数学公式和 mjlab 的 reward term 完全一样，只是组织方式不同。

### RSL-RL 4.0 的关键更新

mjlab 和 Isaac Lab 都使用 RSL-RL 作为 PPO 训练后端。截至 2025 年，RSL-RL 升级到 4.0（Schwarke, Mittal, Rudin, Hoeller, Hutter, "RSL-RL: A Learning Library for Robotics Research," arXiv 2509.10771, 2025），引入了几个对本章 RL Config 有影响的变化：

| 变化 | 旧版 (≤3.x) | 新版 (4.0+) | 影响 |
|------|------------|------------|------|
| 网络配置 | `RslRlPpoActorCriticCfg` | 统一 `MLPModel` / `CNNModel` / `RNNModel` | config 字段名不同 |
| Actor/Critic 分离 | 共享 hidden dims | 可独立配置 | critic 可以比 actor 更大 |
| Distillation | 不内置 | 内置 BC distillation | Ch09 teacher→student 可直接用 |
| Symmetry | 不内置 | 内置 `data_augmentation_func` | 四足左右对称 swap |

如果你使用的 mjlab 版本绑定了 rsl_rl 4.0+，上面的 `actor_hidden_dims` 配置对应的新字段是 `MLPModelCfg(hidden_layer_sizes=[512, 256, 128], activation="elu")`。旧版字段名在新版中会报 deprecation warning。

### Manager 加载顺序

mjlab 的 `load_managers()` 按固定顺序加载 manager：

| 顺序 | Manager | 为什么必须这个顺序 |
|------|---------|-----------------|
| 1 | EventManager | domain randomization 可能扩展 model fields |
| 2 | CommandManager | observation 可能包含 command |
| 3 | ActionManager | 定义 action space 和 last action |
| 4 | ObservationManager | 依赖 command 和 action |
| 5 | TerminationManager | 依赖 observation 和 action |
| 6 | RewardManager | 依赖 observation、action、termination |
| 7 | CurriculumManager | 依赖 reward 和 termination |
| 8 | MetricsManager | 依赖所有上层 manager |
| 9 | RecorderManager | 记录所有 manager 输出 |

如果你自定义 env 时打乱了这个顺序——比如在 ObservationManager 之前加载 RewardManager——某些 observation term 可能还未初始化，导致 NaN 或 shape error。

### Events（Domain Randomization）配置

velocity task 通过 EventManager 配置 domain randomization。回顾 Ch08（Domain Randomization 工程实践）：DR 的目的不是让仿真更逼真，而是让策略对参数变化更鲁棒。velocity task 的典型 DR 配置：

| Event | 触发模式 | 随机化参数 | 范围 | 目的 |
|-------|---------|-----------|------|------|
| `foot_friction` | startup | 脚底摩擦系数 | [0.5, 1.5] | 鲁棒于地面材质变化 |
| `base_com` | startup | 质心偏移 | [-0.05, 0.05] m | 鲁棒于负载变化 |
| `encoder_bias` | startup | 编码器偏置 | [-0.01, 0.01] rad | 鲁棒于传感器零点漂移 |
| `random_push` | interval | 外力扰动 | [-5.0, 5.0] N | 鲁棒于外部冲击 |

`startup` 事件在 env 创建时执行一次，之后不变。这模拟了"不同真机的参数差异"——你的 Go1 的摩擦系数和别人的 Go1 不同，但在一个 episode 内不会突变。`interval` 事件每隔 N 步执行一次，模拟"运行过程中的随机扰动"。

有两个 DR 项对足式 RL 特别重要但容易被忽略。**motor_strength randomization**：将 actuator 的 kp/kd gains 缩放 [0.8, 1.2]。这模拟了真机电机特性的个体差异——不同关节的电机出厂参数不完全一致，且随使用时间会老化。如果策略在训练时只见过标称 kp=40 的电机，部署到实际 kp=35 的关节上就会出现跟踪不足。**added_mass randomization**：在机器人 base 上添加 [-1.0, 3.0] kg 的质量偏移。这模拟了机器人携带不同负载（传感器、电池、有效载荷）的场景。added_mass 会改变重心位置和转动惯量，策略必须学会在不同负载下维持稳定步态。

```python
# 典型的 motor strength 和 added mass DR
"motor_strength": EventCfg(
    func=randomize_actuator_gains,
    mode="startup",
    params={"asset_cfg": EntityCfg("robot"), "scale_range": (0.8, 1.2)},
),
"added_mass": EventCfg(
    func=randomize_rigid_body_mass,
    mode="startup",
    params={"asset_cfg": EntityCfg("robot", body_names="trunk"),
            "mass_range": (-1.0, 3.0)},
),
```

**跨领域类比**：startup DR 类似于机器学习中的数据增强——每个训练样本（episode）看到的参数不同，策略必须学会对这些变化不敏感。interval DR（random push）类似于对抗训练——策略必须能在受到扰动后恢复稳定。两种 DR 的鲁棒性目标不同：startup 针对静态参数不确定性，interval 针对动态扰动。

### DR 的两个高级工程陷阱

**Pseudo-inertia 物理一致性**：独立随机化 link mass 和 diagonal inertia 可能产生**非物理**惯性参数。物理上，三个 principal moments of inertia $(I_{xx}, I_{yy}, I_{zz})$ 必须满足三角不等式：$I_{xx} + I_{yy} \geq I_{zz}$（以及所有循环排列）。独立扰动可能违反这些约束。PhysX 会 **silently accept** 非物理惯性（计算结果不可预测），而 MuJoCo 会 **拒绝加载** 或产生警告。这意味着同一个 DR 配置在 Isaac Lab 中能跑但在 mjlab 中崩溃——看起来像框架 bug，实际是 DR 参数 bug。

```python
# 安全做法：使用 scale randomization 而非 additive
# scale 保持惯性矩阵的比例关系，不会违反三角不等式
"mass_randomization": EventCfg(
    func=randomize_rigid_body_mass,
    mode="startup",
    params={"operation": "scale", "mass_range": (0.8, 1.2)},
    # 而不是 "operation": "add", "mass_range": (-2.0, 5.0)
)
```

Wensing, Kim, Slotine (RA-L 2018) 提出了 4×4 pseudo-inertia matrix 和 LMI (Linear Matrix Inequality) 方法来保证物理一致性。完整实现需要 SDP 求解器，但对于本章的 velocity task，使用 `scale` 模式已经足够避免问题。

**DR 与 Privileged Learning 的交互**：startup DR 使环境参数在 episode 间变化。当你同时使用 privileged learning（Ch09）时，teacher 的 privileged obs 让 critic 能"看到"当前 episode 的环境参数（摩擦、质量等），使 value estimation 更准确。但 student（部署时的 actor）看不到这些参数——它必须从 proprioceptive history 中隐式推断。**DR 范围过窄时**，student 不需要推断（参数变化太小，一个固定策略就够用）；**DR 范围过宽时**，teacher 都学不好（环境太多样化）。这个 sweet spot 通常通过 ablation 找到。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：sensor name regex 匹配空集不报错**。`SceneEntityCfg("robot", site_names=("FR", "FL", "RR", "RL"))` 中的名字必须精确匹配 asset 定义。如果 asset 的 site 命名为 `FR_site` 而你写的是 `FR`，regex 不匹配，ids 为空列表。sensor 和 reward 函数收到空 ids 后返回全零张量，训练继续但对应项完全失效。自检方法：在 zero agent 阶段打印 `entity.data` 确认所有命名。

💡 **概念误区：flat 忘记同步删除 height scan observation**。flat 删了 terrain scan sensor，但如果 observation 中仍有 `height_scan` term，它引用不存在的 sensor，启动报错。正确做法是同时删除 actor 和 critic 的 `height_scan` term。

🧠 **思维陷阱：看到 loss 下降就认为训练正确**。PPO 的 loss 下降只说明策略在当前 reward 信号下改善了。如果 reward 信号本身是错的（某个关键 term 因 sensor name 拼错而为零），策略只是在最小化其他项。判断正确性的唯一方法是 zero/random agent 先验证 wiring，再看视频。

⚠️ **编程陷阱：command 重采样频率设错**。`resampling_time_range=(3.0, 8.0)` 意味着每 3-8 秒随机重采样一次 command。太频繁（<1 秒）让策略无法学稳定步态，太慢（>15 秒）降低 command 多样性。

### 快速读懂陌生 Velocity Task 配置的 5 分钟流程

当你遇到一个新的 velocity task 配置文件（比如从 GitHub 下载的第三方项目），以下流程让你在 5 分钟内理解它的核心结构：

```
分钟 1：找 task registration
  → 搜索 register_mjlab_task 或 gymnasium.register
  → 记下 env_cfg 和 rl_cfg 的引用路径

分钟 2：打开 env_cfg，直奔 scene entity
  → 找到机器人的 MJCF/USD 路径和 default pose
  → 确认 entity key（通常是 "robot"）

分钟 3：检查 actor observation terms
  → 列出所有 term 名字和维度
  → 确认有 command term（没有 = 策略不知道目标）
  → 确认有 height_scan（如果是 rough 任务）

分钟 4：检查 reward terms
  → 按四层分类标注每个 term
  → 记下权重分布（tracking 通常是最大的正权重）
  → 找到 exponential kernel 的 sigma 值

分钟 5：检查 termination + DR
  → flat 应有 fell_over，rough 应有 illegal_contact
  → DR events 有哪些？startup / reset / interval 各有什么？
```

把这个流程跑完后，你就有了该任务的心智模型。后续如果需要修改（换机器人、调 reward），你知道该去哪个文件改哪一行。

### 练习

1. **[追踪题]** 在 Go1 rough env cfg 中找到 `feet_ground_contact` sensor。搜索该 sensor name 在哪些 reward term 和 observation term 中被引用，画出依赖图。
2. **[设计题]** 假设你要把 Go1 velocity task 迁移到 Go2。列出至少 5 个需要修改的 site/geom/body 名字。解释如果漏改了 foot contact sensor 的 geom name 会产生什么现象。
3. **[配置题]** 列出 actor 和 critic observation 的所有 term 及其维度。计算两组的维度差值，解释这个差值来自哪些 privileged term。

---

mjlab 的 velocity task 配置精读让你理解了一个框架内的完整链路。但跨框架对比才能建立真正的抽象理解——哪些是框架特定的实现细节，哪些是足式 RL 的通用设计模式。这正是本节要做的。

## 13.3 Isaac Lab 对应 Task 对照 ⭐⭐⭐

> **这一节解决什么问题**：用 Isaac Lab 的 velocity task 做对照，建立双框架心智模型。

### Isaac Lab 的 velocity task 入口

Isaac Lab 的标准四足速度跟踪任务是 `Isaac-Velocity-Rough-Anymal-C-v0`（也有 Flat 版和 Go2 版）。其配置文件结构和 mjlab 的 Manager-Based 架构高度对应，但命名约定和某些实现细节不同。

回顾 Ch01（双框架对比）：mjlab 和 Isaac Lab 共享 Manager-Based API 设计理念——ObservationManager、ActionManager、RewardManager、TerminationManager、EventManager、CommandManager、CurriculumManager。两个框架的核心差异在于物理后端（MuJoCo Warp vs PhysX）、模型格式（MJCF vs USD）和可视化系统（Viser vs Isaac Sim Viewer）。

### 双框架 API 对照表

| 概念 | mjlab | Isaac Lab |
|------|-------|-----------|
| 环境 cfg 基类 | `ManagerBasedRlEnvCfg` | `ManagerBasedRLEnvCfg` |
| 机器人定义 | `EntityCfg` + asset zoo | `ArticulationCfg` + USD asset |
| 场景 | `SceneCfg` | `InteractiveSceneCfg` |
| 观测组命名 | `actor` / `critic` | `policy` / `critic` |
| 观测项 | `ObsTerm` | `ObsTerm` |
| 奖励项 | `RewTerm` | `RewTerm` |
| 地形 | `TerrainEntityCfg` + generator | `TerrainImporterCfg` + generator |
| Raycast | `RayCastSensorCfg` | `RayCasterCfg` |
| 动作 | `JointPositionActionCfg` | `JointPositionActionCfg` |
| 注册方式 | `register_mjlab_task()` | `gymnasium.register()` |
| RL 后端 | RSL-RL（唯一） | RSL-RL / RL Games / SKRL |
| 训练命令 | `uv run train <task_id>` | `python scripts/rsl_rl/train.py --task <task_id>` |

### 观测组命名差异：actor/critic vs policy/critic

mjlab 用 `actor` 和 `critic` 命名观测组，Isaac Lab 用 `policy` 和 `critic`。这不仅是名字不同——RSL-RL 的 `obs_groups` 配置必须与框架的命名一致：

```python
# mjlab RSL-RL cfg
obs_groups = {"actor": ("actor",), "critic": ("critic",)}

# Isaac Lab RSL-RL cfg
obs_groups = {"policy": ("policy",), "critic": ("critic",)}
```

如果你从 mjlab 项目复制配置到 Isaac Lab 但忘记改 obs group 名字，RSL-RL 会找不到对应的 observation group。这个错误有时不会立即报错（取决于字典的默认行为），而是静默地使用错误的 observation，导致训练表现异常。

### 地形系统差异

Isaac Lab 的地形系统更成熟，内置了更多 terrain type 和配置选项：

| 特性 | mjlab | Isaac Lab |
|------|-------|-----------|
| terrain generator | `TerrainGeneratorCfg` | `TerrainGeneratorCfg` |
| 内置 terrain 类型 | ~5 种（flat, rough, slope, stairs, random） | ~10 种（含 wave, stepping stones, pyramid stairs 等） |
| curriculum 排列 | row = difficulty, col = terrain type | 同上 |
| heightfield 分辨率 | 由 generator 参数决定 | 由 `HfTerrainBaseCfg.size` 决定 |
| flat patches | 支持 | 支持（用于 reset 位置采样） |

两个框架的 terrain curriculum 逻辑相似：训练过程中根据 episode 表现调整 terrain level。表现好的环境移到更难的 terrain，表现差的回到更简单的 terrain。这是"自动课程学习"的一种形式——不需要手动设置阶段切换条件。

**Isaac Lab RayCaster 配置对比**：Isaac Lab 使用 `RayCasterCfg` 进行 GPU raycast height scan，配置方式与 mjlab 略有不同：

```python
# Isaac Lab RayCaster 配置
scene.height_scanner = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base",
    offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),  # 从高处向下发射
    attach_yaw_only=True,     # 等效 mjlab 的 yaw alignment
    pattern_cfg=GridPatternCfg(
        resolution=0.1,       # 10 cm 间隔
        size=(1.6, 1.0),      # 1.6m × 1.0m 覆盖区域
    ),
    mesh_prim_paths=["/World/ground"],  # 只检测地面
    debug_vis=False,
)
```

关键差异：Isaac Lab 的 `attach_yaw_only=True` 对应 mjlab 的 `ray_alignment="yaw"`；Isaac Lab 的 `mesh_prim_paths` 指定 raycast 目标 mesh，而 mjlab 通过 `include_geom_groups` 指定。功能相同，API 不同。

### Isaac Lab velocity task 的 reward 对比

Isaac Lab 的 ANYmal-C velocity task 和 mjlab 的 Go1 velocity task 在 reward 设计上有几个值得注意的差异：

| reward 类别 | mjlab Go1 | Isaac Lab ANYmal-C | 差异原因 |
|------------|-----------|-------------------|---------|
| tracking 形式 | exponential | exponential | 相同思路 |
| base height | 通常不加 | 有（惩罚偏离标称高度） | ANYmal 体型更大，高度偏差更危险 |
| feet air time | critic-only obs | 可作为 reward | 不同设计哲学 |
| termination 触发 | fell_over or illegal_contact | 可配置 | 地形依赖 |

**跨框架类比**：mjlab 和 Isaac Lab 的 velocity task 之间的关系，类似于同一部小说的两个翻译版本——故事情节（MDP 结构）相同，但措辞（API 命名）和细节处理（默认参数）不同。理解故事（通用设计模式）比记忆措辞（特定 API）更重要。

### Isaac Lab 配置精读片段

```python
# Isaac Lab ANYmal-C rough velocity task（简化版）
@configclass
class AnymalCRoughEnvCfg(ManagerBasedRLEnvCfg):
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096)
    
    observations: ObservationsCfg = ObservationsCfg(
        policy=ObservationGroupCfg(
            concatenate_terms=True,
            enable_corruption=True,
            terms={
                "base_lin_vel": ObsTerm(func=base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1)),
                "base_ang_vel": ObsTerm(func=base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2)),
                "projected_gravity": ObsTerm(func=projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05)),
                "velocity_commands": ObsTerm(func=generated_commands, params={"command_name": "base_velocity"}),
                "joint_pos": ObsTerm(func=joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01)),
                "joint_vel": ObsTerm(func=joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5)),
                "actions": ObsTerm(func=last_action),
                "height_scan": ObsTerm(func=height_scan, ..., noise=Unoise(n_min=-0.1, n_max=0.1)),
            },
        ),
        critic=ObservationGroupCfg(enable_corruption=False, ...),
    )
    
    actions: ActionsCfg = ActionsCfg(
        joint_pos=JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.25)
    )
```

注意 `joint_names` 用列表而 mjlab 用 `actuator_names` 元组；`asset_name` 对应 mjlab 的 `entity_name`。这些细节差异在跨框架移植时需要逐一对照。

### env.step() 的执行顺序

理解 env.step() 内部的执行顺序是调试的关键。mjlab 的 step() 按以下顺序执行：

```text
1.  清空 extras["log"]
2.  action_manager.process_action(raw_action)
3.  decimation 循环：apply action → scene.write_data_to_sim() → sim.step()
4.  scene.update(dt=physics_dt)
5.  metrics_manager.compute_substep()
6.  episode counter += 1
7.  termination_manager 计算 done
8.  reward_manager 计算 reward
9.  metrics_manager 计算 step metrics
10. 需要 reset 的 env → _reset_idx()
11. sim.forward() 刷新派生量
12. command_manager 更新 command
13. step/interval event 执行
14. sim.sense() 更新 sensor
15. observation_manager 计算 observation
16. recorder 记录
```

这个顺序有几个关键设计：termination 和 reward 在 reset 之前计算（使用当前 step 的状态）；reset 之后执行 forward 刷新派生量；sensor 更新（包括 raycast）在 observation 计算之前；command 更新也在 observation 之前（因为 obs 可能包含 command）。

Isaac Lab 的 step() 顺序大致相同，但某些细节不同（如 event 触发的时机）。当跨框架调试时，如果同样的配置在一个框架中工作正常但在另一个框架中行为异常，step 执行顺序的差异可能是原因之一。

### EventManager 的五种触发模式

step 执行顺序中第 13 步（step/interval event 执行）背后是一个灵活的 EventManager 系统。Isaac Lab 文档定义了五种 event 模式，mjlab 继承了相同的抽象（但某些模式的行为有差异）：

| 模式 | 触发时机 | 典型用途 | mjlab 特殊限制 |
|------|---------|---------|---------------|
| `prestartup` | USD 加载后（仅 Isaac Lab） | 修改 PhysX 场景级参数 | N/A（mjlab 无 USD） |
| `startup` | sim 启动后执行一次 | 摩擦、restitution、body mass 随机化 | 部分 MuJoCo Warp 字段不安全（PR #631） |
| `reset` | 每个 episode 开始 | PD gains randomization、初始姿态扰动 | 同 Isaac Lab |
| `interval` | 每隔 N 秒（wall time 或 sim time） | gravity vector perturbation | 同 Isaac Lab |
| 用户自定义（如 `step`） | 每个 control step | per-step external force push | 两个框架都支持 |

**工程要点**：

1. **`startup` vs `reset` 的关键区别**：`startup` 在整个训练过程中只执行一次，之后不变——每个 env 在整个训练期间都用同一个摩擦系数、同一个 body mass。`reset` 在每个 episode 开始时重新随机化。前者模拟"不同真机的个体差异"（你的 Go1 和别人的 Go1 摩擦不同），后者模拟"同一真机在不同时间点的参数变化"（关节磨损导致 kp 变化）。

2. **mjlab 的 DR 安全限制**：MuJoCo Warp 中某些字段在运行时 mutate 不安全（如 body parent/child 指针、contact pair table）。mjlab PR #631 移除了这些字段的 DR 支持。实践中这意味着你不能在 mjlab 中随机化 body 拓扑结构——只能随机化数值参数。

3. **PhysX 材料桶限制**：Isaac Lab 的 DR 在随机化 friction/restitution 时可能产生大量 unique materials。PhysX 对每个 scene 的 unique materials 有 **64,000 上限**。4096 envs × 13 bodies × 不同摩擦值可以轻松超限。解决方案是设置 `num_buckets=250`（或更小），将连续摩擦值离散化到 250 个桶中，共享 material 对象。

**执行顺序差异的实际影响**：两个框架最显著的差异在于 "post-reset observation" 的处理。mjlab 在 reset 之后执行 sim.forward() + sim.sense() + observation_manager，确保 reset 后的环境返回的是新状态的 observation。Isaac Lab 的处理方式类似但细节不同——在跨框架对比实验中，如果发现 episode 开头的第一步行为不同，很可能是 post-reset observation 的差异导致的。

另一个值得关注的差异是 **decimation 循环中 sim.step() 的调用方式**。mjlab 在 decimation 循环内部处理 action → write_data → sim.step()，整个循环内 action 不变。Isaac Lab 的实现也是类似的低级物理步循环，但两个框架的 physics timestep 默认值可能不同——mjlab 的默认 dt 通常是 0.005s，Isaac Lab 可能使用 1/120s ≈ 0.0083s。decimation × physics_dt = control_dt 决定了策略的控制频率。如果两个框架的 control_dt 不同，策略行为和奖励值都不可直接比较。

### RSL-RL Wrapper 的桥接角色

mjlab 和 Isaac Lab 都通过 wrapper 把 Manager-Based env 转换为 RSL-RL 期望的接口。wrapper 的核心任务是把框架的 `terminated`/`truncated` 信号转换为 RSL-RL 的 `dones`/`time_outs` 格式：

```python
# 简化的 wrapper 逻辑
dones = terminated | truncated
extras["time_outs"] = truncated & ~terminated
```

`time_outs` 的语义至关重要：它告诉 RSL-RL 哪些 done 是因为 timeout 而不是 real termination。timeout 的 done 需要 bootstrap（因为 episode 还可以继续，只是被截断了），而 real termination 不需要。回顾 Ch07（PPO 训练工程）：如果把 timeout 当成 terminal 处理（不做 bootstrap），value function 会在 episode 末端产生下偏——策略会学到"在 episode 快结束时不需要好好走"的行为。

**wrapper 还负责 observation 的拼接与归一化**。RSL-RL 期望接收两个 tensor：`obs`（actor 用）和 `critic_obs`（critic 用）。wrapper 把框架的多个 observation group（每个是一个 dict of terms）拼接成连续 tensor。如果 observation term 的顺序在两次训练之间改变（比如你在 dict 中插入了一个新 term），拼接后的 tensor 布局会变——已有的 checkpoint 就无法加载。这是一个常见但容易忽略的 breaking change。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：从 mjlab 复制 obs_groups 到 Isaac Lab 忘记改名**。mjlab 用 `"actor"`，Isaac Lab 用 `"policy"`。RSL-RL wrapper 按名字查找 observation group，名字不匹配导致静默错误或 KeyError。

💡 **概念误区：认为两个框架的同名 reward 函数行为相同**。`track_linear_velocity` 在两个框架中实现细节可能不同（exponential kernel 的 sigma 参数、坐标系处理）。迁移 reward 时必须查看源码确认数学形式。

🧠 **思维陷阱：认为 Isaac Lab 因为 stars 多所以一定更好**。Stars 反映社区规模不反映技术优劣。mjlab 在接触密集任务上因 MuJoCo Warp 的凸优化接触模型可能更稳定；Isaac Lab 在视觉任务上因 RTX 渲染有本质优势。选型应基于任务需求（回顾 Ch01）。

### 练习

1. **[对照题]** 找到 Isaac Lab 中 `Isaac-Velocity-Rough-Anymal-C-v0` 的配置文件，列出其 reward terms 和权重。与 mjlab Go1 rough 的 reward 做对照表，标出哪些是共有的、哪些是框架特有的。
2. **[跨框架实践]** 如果你要在 Isaac Lab 中实现 mjlab 的 `foot_clearance` reward，需要引用什么 sensor？Isaac Lab 中对应的 sensor 配置方式是什么？
3. **[调试题]** 你把 mjlab 的 Go1 flat 配置移植到 Isaac Lab，训练后发现 tracking reward 只有 mjlab 的一半。列出你会按什么顺序排查这个问题。提示：先比对 env.step() 中 control_dt（decimation × physics_dt）是否一致，再检查 observation 值域，最后比对 reward 实现。

---

两个框架的 velocity task 都支持 rough terrain 变体。但 rough terrain 不只是换一个场景——它要求策略具备地形感知能力。height scan 传感器是这个能力的核心，也是从 flat 升级到 rough 最关键的新增组件。

## 13.4 地形系统与 Height Scan 传感器 ⭐⭐⭐

> **这一节解决什么问题**：理解地形生成、height scan 配置和 observation 维度之间的关系。

### 从 flat 到 rough：信息缺口

平地任务只需 proprioception——策略不需要"看路"，因为地面永远是平的。粗糙地形打破了这个假设：台阶、斜坡、碎石引入了外部几何约束，策略必须提前知道"下一步踩哪里高度是多少"。

height scan 就是这个"提前看路"的传感器。它从机器人基座发射一组射线，测量与地形的交点距离，返回一个低维的高度向量。这类似于在黑暗中用手电筒照前方——你不需要完整的视觉图像，只需要知道脚前方的高度轮廓。

### 为什么不直接用深度相机

深度相机提供更丰富的三维信息，但在 RL 训练中有四个工程困难：高维输入需要 CNN 编码器（训练更慢）、机器人头部相机看不到脚下正下方（盲区）、仿真与真实深度相机的 sim-to-real gap 显著（噪声、伪影）、渲染计算成本远高于 raycast。

> **本质洞察**：height scan 不是把地形"拍成图"。它是在控制周期内抽取一组和落脚决策直接相关的可学习几何特征。好的 height scan 不是最像地图的输入，而是能让策略提前改变步态的最小信息集。

实践中推荐的路线是：先用 raycast height scan 建立策略的地形感知能力上限，再通过 teacher-student distillation 把这种能力迁移到以深度相机或 proprioception history 为输入的可部署策略（参见 Ch09 和后续 Ch18）。

### 从 flat 升级到 rough 的具体改动清单

理解 flat→rough 的差异最直接的方式是列出所有改动。以 mjlab Go1 为例，从 flat cfg 升级到 rough cfg 需要的全部改动：

| 组件 | flat | rough | 改动类型 |
|------|------|-------|---------|
| terrain | `plane` | `TerrainGeneratorCfg(...)` | 替换 |
| terrain scan sensor | 无 | `RayCastSensorCfg(...)` | 新增 |
| actor obs: height_scan | 无 | `ObsTerm(func=terrain_scan, ...)` | 新增 |
| critic obs: height_scan | 无 | `ObsTerm(func=terrain_scan, ...)` | 新增 |
| critic obs: foot_contact | 可选 | 必须有 | 强化 |
| termination: fell_over | ✅ (70°) | ❌ 删除 | 删除 |
| termination: illegal_contact | ❌ | ✅ 新增 | 新增 |
| termination: out_of_terrain_bounds | ❌ | ✅ 新增 | 新增 |
| curriculum: terrain_levels | ❌ | ✅ 新增 | 新增 |
| reward: 可能调整权重 | 标准 | 增大 clearance 等 | 调整 |

注意这不是"加了一个地形"那么简单——总共涉及至少 10 处修改，跨越 scene、sensor、observation、termination、curriculum 五个 manager。遗漏任何一处都会导致训练异常。

### Grid Pattern 与 Ray 数计算

mjlab 的 `RayCastSensorCfg` 使用 `GridPatternCfg` 生成均匀采样的射线网格：

```python
RayCastSensorCfg(
    entity_cfg=SceneEntityCfg("robot", body_names=("trunk",)),
    pattern_cfg=GridPatternCfg(size=(1.6, 1.0), resolution=0.1),
    ray_alignment="yaw",
    max_distance=5.0,
    include_geom_groups=(0,),
    exclude_parent_body=True,
)
```

Ray 数的计算公式：

$$N_x = \lfloor \text{size}_x / \text{res} \rfloor + 1, \quad N_y = \lfloor \text{size}_y / \text{res} \rfloor + 1, \quad N_{\text{ray}} = N_x \times N_y$$

`size=(1.6, 1.0)` 和 `resolution=0.1` 得到 $N_x = 17, N_y = 11, N_{\text{ray}} = 187$。这个值直接决定 height scan 在 observation 中的维度。

### 分辨率、范围与计算成本的权衡

| 配置 | ray/frame | 每步总 ray (4096 envs) | 相对成本 |
|------|----------|----------------------|---------|
| (1.6, 1.0) @ 0.10 m | 187 | 765,952 | 1× |
| (1.6, 1.0) @ 0.05 m | 693 | 2,838,528 | 3.7× |
| (2.0, 1.5) @ 0.10 m | 336 | 1,376,256 | 1.8× |

分辨率从 0.10 m 降到 0.05 m，ray 数增长约 3.7 倍。训练速度（steps/s）可能下降 20-40%，但策略在细粒度地形上的表现改善可能有限——因为机器人脚掌本身就有几厘米的尺寸，比脚掌小的地形特征对落脚决策影响有限。

### Ray Alignment 的物理意义

`ray_alignment` 控制射线网格如何跟随机器人姿态：

| 对齐方式 | 旋转矩阵 | 效果 |
|---------|---------|------|
| `"yaw"` | 只保留 yaw 旋转 | ray 跟随转向但不随俯仰/侧倾 |
| `"base"` | 完整基座旋转 | ray 跟随所有旋转 |
| `"world"` | 单位矩阵 | ray 始终向下 |

**反事实推理：如果用 `"base"` 而非 `"yaw"` 会怎样？** 当机器人上坡 pitch 前倾 10° 时，ray 也前倾 10°。原本垂直向下的射线斜向前方射出，前方地面被高估、后方被低估。height scan 的均值随步态周期性波动（pitch 随步态周期变化），策略把这种波动误解为地形变化。这个 bug 在平地上完全看不出——只有引入粗糙地形后才暴露。

### 地形生成器配置

mjlab 和 Isaac Lab 都支持程序化地形生成。地形排列成 grid：row 对应 difficulty level，column 对应 terrain type。curriculum 控制环境在 difficulty axis 上的移动。

```python
terrain_generator = TerrainGeneratorCfg(
    size=(8.0, 8.0),                # 单块地形大小
    num_rows=10,                     # difficulty levels
    num_cols=5,                      # terrain types per level
    curriculum=True,                 # 启用 terrain curriculum
    sub_terrains={
        "flat": FlatTerrainCfg(proportion=0.2),
        "rough": RoughTerrainCfg(proportion=0.3, noise_range=(0.01, 0.06)),
        "slope": SlopeTerrainCfg(proportion=0.2, slope_range=(0.0, 0.3)),
        "stairs": StairsTerrainCfg(proportion=0.2, step_height_range=(0.05, 0.15)),
        "random": RandomUniformTerrainCfg(proportion=0.1),
    },
)
```

每种 terrain type 在低 difficulty 时特征更温和（小噪声、缓坡、矮台阶），高 difficulty 时更激进（大噪声、陡坡、高台阶）。这确保 curriculum 的渐进性——策略不会一上来就面对最难的地形。

**各 terrain type 的参数范围与 difficulty 关系**：

| 地形类型 | 低 difficulty (level 0) | 高 difficulty (level 9) | 关键参数 |
|---------|----------------------|----------------------|---------|
| rough | noise: 0.01 m | noise: 0.06 m | `noise_range=(0.01, 0.06)` |
| slope | 倾斜: 0° | 倾斜: ~17° | `slope_range=(0.0, 0.3)` |
| stairs | 台阶高: 5 cm | 台阶高: 15 cm | `step_height_range=(0.05, 0.15)` |
| random | 高度变化: ±1 cm | 高度变化: ±5 cm | 随机均匀 |

**反事实推理：如果所有 difficulty level 的参数都设成最大值会怎样？** 等于没有 curriculum——所有环境一开始就面对最难地形。训练初期策略频繁摔倒，episode 极短，PPO 几乎学不到有效信息。Curriculum 的价值在于让策略**渐进式**地面对更难的挑战——先在 level 0 学会基本步态，再逐步适应更难的地形。

**Terrain 配置的完整代码示例**：

```python
# 完整的 terrain generator 配置（mjlab 风格）
terrain_generator = TerrainGeneratorCfg(
    size=(8.0, 8.0),                    # 单块 8m × 8m
    border_width=5.0,                   # 边界宽度
    num_rows=10,                        # 10 个 difficulty levels
    num_cols=5,                         # 5 种 terrain type
    curriculum=True,                    # 启用 curriculum
    max_init_terrain_level=3,           # 初始最高 level（不从 0 开始可加速）
    
    sub_terrains={
        "flat": FlatTerrainCfg(
            proportion=0.2,             # 20% 是平地
        ),
        "rough": RoughTerrainCfg(
            proportion=0.3,             # 30% 是粗糙地面
            noise_range=(0.01, 0.06),   # 噪声范围随 level 增加
            noise_step=0.005,           # 每 level 增加 0.005m 噪声
            platform_width=2.0,         # 中心平台宽度（reset 位置）
        ),
        "slope": SlopeTerrainCfg(
            proportion=0.2,
            slope_range=(0.0, 0.3),     # 0° 到 ~17°
        ),
        "stairs": StairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.15),  # 5cm 到 15cm
            step_width=0.3,             # 台阶深度 30cm
            platform_width=2.0,
        ),
        "random": RandomUniformTerrainCfg(
            proportion=0.1,
            min_height=-0.05,
            max_height=0.05,
            step=0.01,                  # 高度分辨率
        ),
    },
)
```

**`max_init_terrain_level=3` 的工程意义**：训练开始时，部分环境直接从 level 3 开始（而不是全部从 level 0）。这让策略从一开始就接触到一些中等难度的地形，加速 curriculum 的推进。如果设为 0，所有环境从最简单开始，前 2000 iterations 几乎没有在困难地形上的经验——浪费了训练时间。

**各种地形类型对策略行为的影响**：

| 地形类型 | 物理特征 | 对策略的挑战 | 关键 reward | 框架支持 |
|---------|---------|------------|-----------|---------|
| flat | 无高度变化 | 无 | tracking | mjlab + Isaac Lab |
| rough | 低幅随机噪声 | 脚落点不确定 | foot_clearance | mjlab + Isaac Lab |
| slope | 连续坡面 | 需要调整重心分布 | upright, tracking | mjlab + Isaac Lab |
| pyramid_stairs | 台阶（金字塔排列） | 需要精确抬脚高度 | foot_clearance, contact | mjlab + Isaac Lab |
| random_uniform | 任意高度随机 | 所有挑战综合 | 所有 | mjlab + Isaac Lab |
| **wave** | 正弦波起伏 | 连续高度变化中的平衡 | upright, base_height | Isaac Lab |
| **gap** | 间隔缺口 | 必须跨越缺口 | foot_clearance, tracking | Isaac Lab |
| **discrete_obstacles** | 散落障碍物 | 绕行或跨越 | contact, tracking | Isaac Lab |
| **stepping_stones** | 离散可踩踏区域 | 精确脚放置 | foot placement | Isaac Lab |

Isaac Lab 的 `TerrainImporterCfg` 支持的地形类型比 mjlab 更多（后 4 种为 Isaac Lab 特有）。如果你的任务需要 gap 或 stepping stones 等精确脚放置的地形，目前 Isaac Lab 是更好的选择。mjlab 侧可以通过自定义 heightfield 生成器实现这些地形，但没有内置支持。

`proportion` 参数控制每种地形在 terrain grid 中占的列数比例。总比例必须加起来等于 1.0。如果你的任务场景主要是台阶（比如室内），可以提高 `stairs` 的 proportion 让策略在台阶上获得更多训练样本。但不要把某种地形的 proportion 设为 0——训练时完全没见过的地形类型在部署时遇到会导致策略失败。

**flat patches 的作用**：在粗糙地形中保留一些 flat 区域有重要意义。terrain curriculum 通常配合 flat patches 使用——当环境 reset 时，机器人被放置在对应 terrain 区域内的一个 flat patch 上，确保每个 episode 从一个稳定站立位置开始。如果 reset 位置恰好在台阶边缘或斜坡中间，机器人可能在第一步就摔倒，产生极短 episode 降低训练效率。

### Terrain Curriculum 的工作机制

terrain curriculum 不需要手动设置切换条件——它根据每个环境的表现自动调整。具体机制：

1. 每个环境 $i$ 有一个 terrain level $l_i \in \{0, 1, ..., L-1\}$
2. episode 结束时，计算该 episode 的 "performance metric"（通常是位移距离或平均 tracking reward）
3. 如果 performance 超过阈值，$l_i$ 加 1（移到更难的地形）
4. 如果 performance 低于阈值，$l_i$ 减 1（退回更简单的地形）
5. 环境被 reset 到对应 level 的 terrain 区域

这种机制的巧妙之处在于：不同环境可以处于不同的 difficulty level。表现好的环境在高难度地形上训练，提供挑战性样本；表现差的环境回到低难度地形，提供基础经验。PPO 在一个 batch 中同时看到不同难度的样本，value function 学会估计不同地形条件下的 expected return。

**反事实推理：如果不用 terrain curriculum，直接在最难地形上训练会怎样？** 初始策略完全是随机的，在高难度地形上 episode 极短（几步就摔倒或走出边界）。PPO 的 rollout 几乎全是失败经验，reward 信号极其稀疏。策略可能很长时间都无法突破"学会站立"的阶段。Curriculum 通过从简单地形开始，给策略足够的正信号来学会基本步态，然后逐步增加难度。

**学术出处**：这种 game-inspired terrain curriculum 由 Rudin, Hoeller, Reist, Hutter 提出（"Learning to Walk in Minutes Using Massively Parallel Deep RL," CoRL 2021, arXiv 2109.11978）。原始设计使用 10 行 × 20 列的 terrain grid，地形比例 `[0.1, 0.1, 0.35, 0.25, 0.2]` 分配给 smooth slope / rough slope / stairs up / stairs down / discrete obstacles。这篇论文也是 legged_gym 的来源——mjlab 和 Isaac Lab 的 terrain curriculum 机制都可以追溯到这里。

**从 Height Scan 到视觉感知的下一步**：本章使用 raycast height scan 作为地形感知手段。Miki et al. (2022, *Science Robotics*) 展示了更进一步的方案——用 attention-based encoder 从 proprioceptive history 中隐式重建地形估计，在 DARPA SubT Challenge 中实现了 1700 米零跌倒。但 Miki 的方法在训练阶段仍然需要 height scan 作为 teacher 的输入——与本章建立的 privileged height scan teacher 是同一个起点。Ch18（视觉感知运动控制）将详细讨论从 height scan 到深度相机的迁移路径。

### Height Scan 的 Domain Randomization

在训练时为 height scan 添加噪声是 sim-to-real 鲁棒性的关键。mjlab 通过 observation corruption 机制实现：

```python
# actor obs 配置中
"height_scan": ObsTerm(
    func=terrain_scan,
    noise=GaussianNoiseCfg(mean=0.0, std=0.1),  # ±0.1m noise
    clip=(-1.0, 1.0),
)
```

这模拟了真实 elevation map 的噪声特性。真实的 elevation map 来自 LiDAR 或深度相机，其误差来源包括：传感器噪声（~1-3cm）、姿态估计漂移（~5-10cm）、遮挡和缺失区域、点云稀疏性导致的栅格化误差。0.1m 的 Gaussian noise 大致覆盖了这些误差的综合效果。

如果真机使用 teacher-student 框架（Ch09），teacher 看完美 height scan，student 看深度相机或 proprioception history。这种情况下 teacher 不需要加噪声——student 的输入天然有噪声。但如果直接部署 actor（不用蒸馏），训练时加噪声就是唯一的鲁棒性保障。

### Contact Sensor 与足端状态观测

地形感知解决了"前方地形长什么样"，但还有另一个关键问题："脚现在在哪？踩到了什么？用了多大力？" 这些信息来自 contact sensor 和 foot state observation，它们在 rough terrain task 中扮演与 height scan 互补的角色——height scan 提供前馈（proactive）信息，contact sensor 提供反馈（reactive）信息。

mjlab 的 rough velocity task 通常在 **critic observation** 中包含以下足端状态量：

```python
# critic 额外 obs（rough velocity task 典型配置）
"foot_height": ObsTerm(
    func=foot_position_in_base_frame,  # 4 feet × 3 xyz → 12 维
    entity_name="robot",
    foot_sites=("FR_foot", "FL_foot", "RR_foot", "RL_foot"),
),
"foot_contact_forces": ObsTerm(
    func=contact_forces,
    sensor_name="contact_sensor",
    threshold=1.0,  # N，低于此值视为无接触
),
"foot_air_time": ObsTerm(
    func=foot_air_time,
    sensor_name="contact_sensor",
    threshold=0.5,  # 秒，期望空中时间
),
```

这里有一个重要的设计选择：**足端状态量放在 critic 而不是 actor 中**。原因与 asymmetric actor-critic 的动机一致——真机上很难精确获取足端力（需要力传感器或状态估计器），但仿真中可以直接读取。让 critic 看到这些信号帮助它更准确地估计 value，间接指导 actor 学到更好的步态策略。如果把 contact forces 放进 actor observation，训练时策略会依赖这个信号，部署时没有对应传感器就会失败。

**Contact Sensor 的配置要点**：

```python
# mjlab contact sensor 配置
scene.contact_sensor = ContactSensorCfg(
    prim_path="/World/envs/env_.*/Robot",
    update_period=0.0,  # 每个 physics step 更新
    filter_prim_paths=["/World/ground"],  # 只检测与地面的接触
    history_length=0,   # 不保留历史
)
```

`filter_prim_paths` 控制接触对——如果不设置，sensor 会检测所有碰撞对（包括自碰撞），计算量更大且数据更嘈杂。在 locomotion task 中，我们通常只关心脚-地接触，所以显式指定 ground 作为 filter。

Isaac Lab 的 contact sensor 配置逻辑类似但 API 略有不同：

```python
# Isaac Lab contact sensor 配置
scene.contact_forces = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*",
    history_length=3,
    track_air_time=True,
    update_period=0.0,
)
```

Isaac Lab 内置了 `track_air_time` 选项，自动计算每只脚的空中时间。mjlab 中这个功能通常由 observation function 内部实现——又一个 API 层级差异的例子。

**foot_air_time reward 的物理语义**：鼓励策略保持合理的空中时间（通常 0.3-0.5 秒）。如果空中时间为零，说明脚一直在拖地——能量浪费，且在粗糙地形上容易绊倒。如果空中时间过长（> 0.8s），说明步态过于夸张——不稳定，且 base 上下摆动大。这个 reward 本质上是一个软约束，把步态频率限制在合理范围内。

**反事实推理：如果 critic 也看不到 foot contact 信息会怎样？** Critic 必须仅从 proprioception 推断足端接触状态——这在物理上是可能的（关节力矩突变通常意味着接触），但信号不如直接读取 contact force 清晰。结果是 value function 估计不够准确，尤其在 terrain transition 区域（从平地到台阶的边缘）。PPO 的 advantage estimation 噪声增大，训练需要更多 sample 才能收敛。在简单地形上差异可能不大，但在复杂地形上你可能需要 2-3 倍的训练 iteration 才能达到相同的策略质量。

### 感知输入的工程选型决策树

当面临"用什么地形感知输入"的选择时：

```
1. 任务需要前向地形信息吗？
   ├─ 否（平地）→ 只用 proprioception
   └─ 是 →
       2. 训练阶段还是部署阶段？
          ├─ 训练 → 用 raycast height scan（最快、最简单）
          └─ 部署 →
              3. 真机有深度相机吗？
                 ├─ 有 → teacher(scan)-student(depth) distillation
                 └─ 没有 →
                     4. 能加 proprioception history？
                        ├─ 可以 → teacher(scan)-student(history)
                        └─ 不可以 → 考虑加传感器
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：ray 打到机器人自身**。`include_geom_groups` 如果包含了机器人 body 的 geom group，ray 会命中腿部，返回异常低的高度值。正确做法：确认只包含 terrain geom 的 group，并启用 `exclude_parent_body=True`。

💡 **概念误区：认为分辨率越高越好**。ray 数随分辨率倒数平方增长。训练速度显著下降的同时，策略改善可能很小。从 0.10 m 开始，只有明确需要时才加密。

🧠 **思维陷阱：修改 resolution 后忘记检查 observation 维度**。ray 数改变直接影响 observation tensor 形状。如果网络输入维度没有同步更新（或者不是动态推断），训练会报 shape mismatch。

### 练习

1. **[计算题]** 如果把 grid pattern 的 size 改为 (2.0, 1.5)，resolution 保持 0.1 m，新的 ray 数是多少？observation 维度增加了多少？
2. **[设计题]** 如果机器人需要在窄桥上行走（宽 30cm），标准的 1.0m y 范围 scan 大部分 ray 落在桥外。你会如何修改 scan 配置？
3. **[跨章综合题]** 结合 Ch08 的 domain randomization 知识，设计一种对 height scan 的随机化方案，使策略在训练时对 scan 噪声具有鲁棒性。你会在什么阶段引入这种随机化？

---

地形系统和传感器配置好了，下一步就是把环境跑起来。但"跑起来"不是一步完成的——从 zero agent 到 large train 需要分阶段验证，每一步都有明确的通过标准。

## 13.5 完整训练流程：从 Zero Agent 到 Large Train ⭐⭐

> **这一节解决什么问题**：给出从 zero agent 到完整训练的分阶段验证方法，建立可重复的实验流程。

### 为什么需要分阶段验证

四足 velocity task 中最隐蔽的失败模式不是算法 bug，而是 task wiring 错误——sensor name 拼错、observation term 缺失、reward 权重符号错误。这些错误的共同特点是：局部看都合理，环境能启动，PPO 能训练，loss 在下降，但策略永远学不会走路。分阶段验证能在 5 分钟内暴露 90% 的 wiring 错误，避免浪费数小时 GPU 时间。

这和软件工程中的测试金字塔类似：先跑单元测试（zero agent），再跑集成测试（small train），最后跑端到端测试（large train）。跳过前面的步骤直接跑 large train，就像不写单元测试直接部署到生产环境。

### 阶段 0：确认 task id 存在

```bash
# mjlab
uv run train --help  # 列出所有可用 task id
uv run list-envs     # 更详细的环境列表

# Isaac Lab
python scripts/rsl_rl/train.py --task Isaac-Velocity-Rough-Anymal-C-v0 --help
```

### 阶段 1：Zero Agent

```bash
# mjlab flat
uv run play Mjlab-Velocity-Flat-Unitree-Go1 \
  --agent zero --num-envs 4 --viewer viser

# mjlab rough
uv run play Mjlab-Velocity-Rough-Unitree-Go1 \
  --agent zero --num-envs 4 --viewer viser

# Isaac Lab
python scripts/rsl_rl/play.py --task Isaac-Velocity-Flat-Anymal-C-v0 \
  --num_envs 4 --load_run "" --checkpoint ""  # 使用 zero policy
```

**通过标准**：不 crash，机器人以默认姿态站立（可能轻微抖动），无 NaN，无 missing sensor 报错。如果 `use_default_offset=True` 配置正确，zero action 对应默认关节姿态，机器人应该能静止站立几秒。

### 阶段 2：Random Agent

```bash
# mjlab
uv run play Mjlab-Velocity-Flat-Unitree-Go1 \
  --agent random --num-envs 4 --viewer viser
```

**通过标准**：机器人剧烈乱动但不飞出场景。reset 正常触发。无 NaN。如果 action scale 过大，机器人可能瞬间弹飞——这说明 scale 需要调小。

### 阶段 3：Small Train（50-100 iterations）

```bash
# mjlab flat
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
  --env.scene.num-envs 256 --agent.max-iterations 50 \
  --agent.logger tensorboard --agent.upload-model False \
  --gpu-ids "[0]"

# mjlab rough
uv run train Mjlab-Velocity-Rough-Unitree-Go1 \
  --env.scene.num-envs 256 --agent.max-iterations 50 \
  --agent.logger tensorboard --gpu-ids "[0]"

# Isaac Lab
python scripts/rsl_rl/train.py --task Isaac-Velocity-Flat-Anymal-C-v0 \
  --num_envs 256 --max_iterations 50
```

**通过标准**：日志完整写入，无 shape error，tensorboard 可打开。不要求 reward 上升——50 iterations 对复杂四足任务远远不够。小规模训练只验证接口。

### 阶段 4：Large Train

```bash
# mjlab flat baseline
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
  --env.scene.num-envs 4096 --agent.max-iterations 5000 \
  --agent.run-name go1_flat_baseline

# mjlab rough baseline
uv run train Mjlab-Velocity-Rough-Unitree-Go1 \
  --env.scene.num-envs 4096 --agent.max-iterations 10000 \
  --agent.run-name go1_rough_baseline
```

**通过标准**：flat 约 1000-2000 iterations 后 tracking reward 明显上升。rough 需要更长——先用 flat baseline 确认 pipeline 正确，再启动 rough。

### 阶段 5：Play trained checkpoint

```bash
uv run play Mjlab-Velocity-Flat-Unitree-Go1 \
  --checkpoint-file logs/rsl_rl/go1_velocity/<run>/model_5000.pt \
  --num-envs 4 --viewer viser
```

**通过标准**：视频中机器人能按 command 方向行走，步态合理，无明显抖动或拖脚。

### 四阶段验证对照表

| 阶段 | 时间 | 目的 | 不应期待 | 通过标准 |
|------|------|------|---------|---------|
| zero | ~10秒 | 验证 wiring | 会走 | 不 crash、无 NaN |
| random | ~10秒 | 验证动作范围 | 稳定 | reset 正常 |
| small train | ~2分钟 | 验证接口 | reward 上升 | 日志完整 |
| large train | ~1小时 | 验证策略 | 立刻收敛 | 视频和指标一致 |

### 超越 Total Reward 的评估指标

total reward 是训练过程中的优化目标，但不是策略质量的唯一衡量标准。一个 total reward 很高的策略可能在真机上完全不可用（reward hacking），而一个 total reward 中等的策略可能在真机上表现更好（更鲁棒）。以下是评估 velocity tracking 策略时应关注的完整指标体系：

| 指标 | 计算方式 | 合格范围（Go1 flat） | 测量对象 |
|------|---------|-------------------|---------|
| **Tracking error (m/s)** | $\|v_{\text{actual}} - v_{\text{cmd}}\|$ 的 episode 平均 | < 0.15 m/s | 任务完成度 |
| **Angular tracking error (rad/s)** | $\|\omega_{\text{actual}} - \omega_{\text{cmd}}\|$ 的 episode 平均 | < 0.2 rad/s | 转弯精度 |
| **Fall rate (%)** | 因非 timeout 终止的 episode 比例 | < 5% (flat), < 15% (rough) | 安全性 |
| **Episode length (步)** | 平均 episode 持续步数 | > 800 (flat), > 500 (rough) | 存活能力 |
| **Action smoothness (jerk)** | $\|\ddot{a}\|$ 的 episode 平均 | — | 部署可行性 |
| **Energy (J/m)** | 总关节力矩 × 角速度 / 行走距离 | — | 效率 |
| **CoT** | Energy / (mass × g × distance) | < 2.0 | 归一化效率 |

**工程建议**：在 tensorboard 中不只看 `reward/total`，还要添加自定义 metric 记录 tracking error 和 fall rate。mjlab 的 `MetricsManager` 支持注册自定义 metric term。在做 reward ablation 或 DR 调参时，tracking error 比 total reward 更有诊断价值——它直接告诉你策略能否完成任务，而不是它在某个 reward 函数下的得分。

### 训练日志阅读指南

Large train 过程中需要关注的关键指标：

| 指标 | 健康范围 | 异常信号 |
|------|---------|---------|
| `reward/total` | 持续上升 | 平坦或下降 → 检查 reward 配置 |
| `reward/track_linear_velocity` | 占总 reward 主要比例 | 接近零 → command obs 缺失 |
| `loss/value` | 先升后降 | 持续上升 → critic 不稳定 |
| `policy/kl` | 0.005-0.02 | >0.05 → 降低学习率 |
| `policy/entropy` | 缓慢下降 | 快速坍塌到零 → 增大 entropy coef |
| `episode/length` | 逐渐增长 | 极短 → termination 过严 |
| `terrain/level` | 逐渐上升 | 停滞 → curriculum 阈值过高 |

这些指标之间有关联：如果 `episode/length` 极短，`reward/total` 不太可能持续上升——短 episode 让策略没有足够时间探索有奖励的行为。如果 `policy/entropy` 快速坍塌，策略过早收敛到一个 deterministic policy，探索停止，后续性能停滞。entropy coefficient 的作用就是防止这种过早收敛——它在 PPO loss 中添加了一个鼓励策略保持随机性的项。

**训练曲线的典型形态与诊断**：

一个健康的四足速度跟踪训练曲线通常经历四个阶段：

| 阶段 | 迭代范围（flat） | 现象 | 解释 |
|------|----------------|------|------|
| I. 探索期 | 0-500 | reward 缓慢上升 | 策略从随机动作中发现"站立"比"摔倒"收益更高 |
| II. 步态涌现期 | 500-1500 | reward 快速上升 | 策略学会基本步态，tracking reward 开始贡献 |
| III. 精修期 | 1500-3000 | reward 缓慢上升 | 策略细化步态细节——减少打滑、优化能耗 |
| IV. 收敛期 | 3000+ | reward 平台 | 策略接近最优，改善空间有限 |

如果你的训练在阶段 I 停留过久（> 1000 iterations），通常不是 PPO 的问题——而是 MDP 定义或 reward shaping 的问题。常见原因包括：action scale 过大导致机器人被弹飞（无法探索"站立"）、默认 pose offset 不合理（zero action 时机器人就倒了）、reward 中有一个数值量级异常大的惩罚项压制了所有探索。

**"策略不走"的系统性排查流程**：

这是四足 RL 中最常见的问题——训练了 5000 iterations，reward 缓慢上升但策略只会站着不动或原地抖动。按以下顺序排查：

```
Step 1: 确认 command obs 存在
  → 打印 actor obs group，确认有 command term
  → 打印 command 值，确认不全为零

Step 2: 确认 tracking reward 有信号
  → 查看 tensorboard 中 reward/track_linear_velocity
  → 如果为零 → command source 与 tracking reward 不匹配

Step 3: 确认 action 有效
  → play 时打印 action tensor 的均值和标准差
  → 如果 std 接近零 → entropy 坍塌，策略变成 deterministic
  → 如果 mean 非零但机器人不动 → action scale 太小或 kp 太低

Step 4: 确认 reward balance
  → 打印所有 reward term 的初始值
  → 如果某个惩罚项量级远大于 tracking reward → 策略选择"不动"来避免惩罚
  → 临时关闭所有惩罚项，确认策略在只有 tracking reward 时能走

Step 5: 确认物理
  → zero agent 检查机器人默认姿态是否稳定
  → random agent 检查 action 范围是否合理（机器人应该随机扭动但不飞出）
```

这个流程从最简单的 wiring 问题开始，逐步深入到 reward balance 和物理问题。80% 的"策略不走"问题在 Step 1-2 就能定位。

**rough terrain 的训练曲线有额外特征**：terrain curriculum level 是一个重要的辅助指标。在健康训练中，terrain level 应该呈阶梯状上升——每隔一段时间，大部分环境的 level 增加一级。如果 terrain level 快速上升到最高然后波动，说明 curriculum 阈值设得太低，策略还没有真正掌握当前难度就被推到下一级。如果 terrain level 完全不上升，说明策略在最简单地形上都无法达标——需要回去检查 flat 训练是否正常。

### Reward Ablation 实验方法

理解 reward 各项的实际作用，最可靠的方法是 ablation 实验——每次关闭一个 reward term，观察策略行为变化。具体操作：

```bash
# baseline（全部 reward）
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
  --env.scene.num-envs 4096 --agent.max-iterations 3000 \
  --agent.run-name go1_flat_baseline

# ablation: 关闭 foot_slip
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
  --env.scene.num-envs 4096 --agent.max-iterations 3000 \
  --env.rewards.foot_slip.weight 0.0 \
  --agent.run-name go1_flat_no_slip

# ablation: 关闭 pose
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
  --env.scene.num-envs 4096 --agent.max-iterations 3000 \
  --env.rewards.pose.weight 0.0 \
  --agent.run-name go1_flat_no_pose
```

Ablation 实验的常见发现：

| 关闭的 term | 典型行为变化 | 解释 |
|------------|------------|------|
| foot_slip | 策略更激进，脚在地上打滑 | 没有滑动惩罚，策略用更大力推进 |
| foot_clearance | 拖脚步态 | 不被惩罚抬脚不够高 |
| pose | 关节姿态扭曲但 tracking 可能更好 | 更大的关节自由度有时有利于速度跟踪 |
| action_rate_l2 | 动作高频抖动 | 没有平滑约束，策略用高频振荡增益 |
| upright | 基座倾斜但不一定摔倒 | 倾斜可降低风阻（物理正确但不美观） |

**关键洞察**：ablation 实验可能显示关闭某个 penalty 后 tracking reward 反而上升——这不意味着这个 penalty 是有害的。策略可能通过"打滑前进"获得更高 tracking 分数，但这种行为在真机上不可行。reward 设计的目标不只是最大化 tracking accuracy，而是在 tracking 和 behavior quality 之间取得平衡。

**Ablation 实验的完整模板**：

```bash
#!/bin/bash
# reward_ablation.sh — 一键运行完整 ablation 实验
TASK="Mjlab-Velocity-Flat-Unitree-Go1"
ENVS=4096
ITERS=3000

# 要 ablate 的 reward terms
TERMS=("foot_slip" "foot_clearance" "pose" "action_rate_l2" "upright")

# Baseline (3 seeds)
for seed in 42 43 44; do
    uv run train $TASK --env.scene.num-envs $ENVS \
        --agent.max-iterations $ITERS --agent.seed $seed \
        --agent.run-name baseline_seed${seed} &
done
wait

# Ablation (每个 term × 3 seeds)
for term in "${TERMS[@]}"; do
    for seed in 42 43 44; do
        uv run train $TASK --env.scene.num-envs $ENVS \
            --agent.max-iterations $ITERS --agent.seed $seed \
            --env.rewards.${term}.weight 0.0 \
            --agent.run-name no_${term}_seed${seed} &
    done
    wait
done

echo "完成！共 $(( (1 + ${#TERMS[@]}) * 3 )) 个实验"
echo "用 tensorboard --logdir logs/rsl_rl/ 查看对比"
```

**为什么需要 3 个 seed？** RL 训练的随机性很大——同一配置不同 seed 的 tracking reward 最终值可能差 20%。3 个 seed 的均值和标准差能可靠地判断"关闭某个 term 是否真的有影响"。如果 baseline 的 reward 是 0.75±0.05，ablation 是 0.70±0.05，差异不显著。如果 ablation 是 0.55±0.03，差异显著——这个 term 确实重要。

**Ablation 结果的报告格式**：

| 配置 | Tracking Reward (↑) | Episode Length (↑) | Action Jerk (↓) | 视觉评估 |
|------|-------|------|------|------|
| Baseline | 0.75 ± 0.05 | 18.5 ± 0.3 | 0.12 ± 0.01 | 正常步态 |
| No foot_slip | 0.78 ± 0.04 | 18.0 ± 0.5 | 0.11 ± 0.02 | 明显打滑 |
| No pose | 0.73 ± 0.06 | 17.8 ± 0.4 | 0.15 ± 0.02 | 姿态扭曲 |
| No foot_clearance | 0.70 ± 0.05 | 16.5 ± 0.8 | 0.13 ± 0.01 | 拖脚 |
| No action_rate | 0.72 ± 0.04 | 18.2 ± 0.3 | **0.35 ± 0.05** | 高频抖动 |
| No upright | 0.74 ± 0.05 | 17.0 ± 0.6 | 0.12 ± 0.01 | 基座倾斜 |

**跨领域类比**：reward ablation 就像药物临床试验中的对照组——你不知道每种"药物"（reward term）的实际效果，直到和"不吃药"（关闭该 term）的组对比。光看整体效果（总 reward）不够——需要看多维指标（tracking/length/jerk/视觉），因为不同 reward term 影响不同维度的行为。

### Command Curriculum

velocity task 的 command curriculum 随训练进度逐步扩大命令范围。回顾 Ch06（Reward 与 Curriculum）：curriculum 的目的是避免一开始就给策略太难的任务。command curriculum 的典型配置：

| 训练步数 | lin_vel_x 范围 | lin_vel_y 范围 | ang_vel_z 范围 |
|---------|---------------|---------------|---------------|
| 0 | [-1.0, 1.0] m/s | [-1.0, 1.0] m/s | [-0.5, 0.5] rad/s |
| 120K steps | [-1.5, 2.0] m/s | [-1.0, 1.0] m/s | [-1.0, 1.0] rad/s |
| 240K steps | [-2.0, 3.0] m/s | [-1.0, 1.0] m/s | [-1.5, 1.5] rad/s |

注意 lin_vel_x 的上限增长比下限快——前进速度比后退速度更重要，也更容易学。ang_vel_z 范围也逐步扩大，让策略先学会直线走再学转弯。

**零速命令的特殊处理**：command sampler 通常有一个特殊逻辑——当采样到的 command 三个分量的绝对值都很小时（比如 $\|v_x\| < 0.1, \|v_y\| < 0.1, \|\omega_z\| < 0.2$），把 command 置零。这是因为非常小的 command 在物理上难以执行——机器人无法精确跟踪 0.05 m/s 的速度，且 tracking reward 在这个区域对噪声极其敏感。置零后策略学到一个明确的"站立不动"行为，比学一个"几乎不动"的模糊行为更有实际价值。

```python
# command sampler 中的零速处理（概念性代码）
cmd = sample_uniform(cmd_ranges)
if (abs(cmd[0]) < 0.1 and abs(cmd[1]) < 0.1 and abs(cmd[2]) < 0.2):
    cmd = [0.0, 0.0, 0.0]
```

**command curriculum 与 terrain curriculum 的交互**：当同时启用两种 curriculum 时，策略面临双重递进的挑战——命令范围逐步扩大，地形难度也在增加。如果两个 curriculum 同步推进太快，策略可能在"高速 + 困难地形"组合上表现崩溃。实践建议是让 command curriculum 的推进速度慢于 terrain curriculum——先在简单命令下掌握困难地形，再扩大命令范围。

**反事实推理：如果跳过 zero/random 直接 large train 会怎样？** 假设 foot contact sensor 的 geom name 拼错了，所有 contact reward 为零。large train 跑 2 小时后策略学到一个拖脚步态（contact reward 不提供信号，策略不被鼓励抬脚）。你看视频觉得"差强人意"，花 3 小时调 reward 权重无果。而 zero agent 只需 10 秒，打印一下 sensor 数据就能发现 contact 全为零。

### 调参优先级

调参应从任务级开始，RL 超参最后调。推荐顺序：

```
1. 检查 wiring（zero/random agent）
2. 调 terrain difficulty 和 command range
3. 调 reward 权重和 penalty 强度
4. 调 episode length 和 termination 条件
5. 调 PPO 超参（lr, clip, epochs, entropy_coef）
```

这个顺序的逻辑是：MDP 定义决定了学习问题的结构，PPO 超参只决定优化效率。如果 MDP 有问题（wrong reward, bad termination），调 PPO 超参永远无法修复。

| 现象 | 第一检查项 | 调整方向 |
|------|-----------|---------|
| 不走 | command obs | 确认 actor 有 command term |
| 摔倒多 | fell_over 计数 | 检查 action scale 和 termination |
| 抖动 | action rate metric | 增大 action_rate_l2 weight |
| 拖脚 | clearance metric | 增大 foot_clearance weight |
| rough reset 太频繁 | illegal_contact 计数 | 查 contact sensor geom 匹配 |
| KL divergence 高 | learning rate | 降低 LR 或用 adaptive schedule |
| entropy 快塌 | entropy coef | 增大 entropy coef（如 0.01→0.02） |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：train 和 play 的 num_envs 参数位置不同**。mjlab train 用 `--env.scene.num-envs`，play 用 `--num-envs`。Isaac Lab 类似但参数格式略有不同。混用导致 CLI 解析失败或使用默认值。

💡 **概念误区：small train reward 不升就是 bug**。50 iterations × 4096 envs × 24 steps_per_env = ~500 万 steps。四足策略通常需要数千万 steps 才开始学会基本步态。small train 只验证接口不验证效果。

🧠 **思维陷阱：只看 tensorboard 曲线不看视频**。reward 上升但策略可能学到了 hack（利用 simulator 接触 bug 获取 reward）。每隔 1000 iterations 看一次 play 视频是必要的 sanity check。

### 练习

1. **[实践题]** 依次执行 zero → random → small train（flat 和 rough 各一次）。记录每步是否通过。如果遇到报错，按本节的排查思路定位原因。
2. **[分析题]** 在 small train 的 tensorboard 日志中找到每个 reward term 的初始值。解释为什么有些 term 初始为零，有些不为零。
3. **[ablation 实验]** 选择 flat velocity task，做一组 reward ablation：分别关闭 foot_slip、action_rate_l2 和 pose reward（每次只关闭一个，加上一个 baseline 共 4 次实验），各训练 3000 iterations。记录每组的最终 tracking reward、平均 episode length 和肉眼观察到的步态特征差异。用一张表格总结发现。
4. **[设计题]** 如果你的 Go1 策略在 flat 上 tracking reward = 0.85，但在 rough 上只有 0.45。你怀疑是 height scan 的配置问题。列出排查步骤：(a) 如何确认 height scan 数据正确？(b) 如何确认 critic 能看到 height scan？(c) 如果把 height scan 的分辨率从 11×11 降到 5×5，对策略有什么影响？
5. **[跨章综合题，Ch07+Ch08+Ch13]** PPO 的 GAE 参数 λ 和 γ 如何影响 velocity task 的训练？如果你把 γ 从 0.99 降到 0.95，策略会更关注短期 reward（即时跟踪准确度）还是长期 reward（走更远、不摔倒）？结合 Ch08 的 DR 讨论：如果同时降低 γ 和增加 DR 强度，两者的交互效应是什么？

---

前面四节讲的都是单个机器人的标准配置。但真实研究中，你可能需要在同一个框架中快速切换机器人——Go1、Go2、A1。unitree_rl_mjlab 项目展示了如何用同一个 env cfg 适配多款机器人，这种设计模式对你自己的研究非常有参考价值。

## 13.6 精读：unitree_rl_mjlab 多机器人配置 ⭐⭐

> **这一节解决什么问题**：通过精读 unitree_rl_mjlab 项目，学习在 mjlab 中复用同一 velocity env cfg 适配不同机器人的工程模式。

### 项目概览

unitree_rl_mjlab（`github.com/unitreerobotics/unitree_rl_mjlab`）是 Unitree 官方维护的 mjlab 生态项目。它在 mjlab 内置的 velocity task 基础上，展示了如何为 Unitree 旗下多款不同机器人配置统一的训练管线。

**支持的机器人和任务**：

| 机器人 | 类型 | Velocity task | Tracking task | 说明 |
|--------|------|:---:|:---:|------|
| Go2 | 四足 | ✅ | ❌ | 本章主要参考 |
| A2 | 四足 | ✅ | ❌ | 工业级四足 |
| As2 | 四足 | ✅ | ❌ | — |
| G1 (29-DoF) | 人形 | ✅ | ✅ | Ch14 主角 |
| R1 | 轮式人形 | ✅ | ❌ | — |
| H1_2 | 人形 | ✅ | ❌ | H1 升级版 |
| H2 | 人形 | ✅ | ❌ | — |

**Task ID 命名规范**：`Mjlab-<TaskFamily>-<Terrain>-<Vendor>-<Robot>`，例如：

```bash
# 四足 velocity
uv run train Mjlab-Velocity-Flat-Unitree-Go2 --env.scene.num-envs 4096
uv run train Mjlab-Velocity-Rough-Unitree-Go2 --env.scene.num-envs 4096

# 人形 velocity
uv run train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 4096

# 人形 motion tracking (需要先 csv_to_npz.py 处理动作文件)
uv run train Mjlab-Tracking-Flat-Unitree-G1 --registry-name your-org/motions/...
```

这个命名规范看起来像 URL routing——从 task family 到 terrain type 到 vendor 到 robot，层层缩小范围。记住这个规范可以帮你在不查文档的情况下猜出任何任务的 task ID。

**Isaac Lab 对应仓库**：unitree_rl_lab（`github.com/unitreerobotics/unitree_rl_lab`）是 Isaac Lab 侧的对应项目，任务命名相同（`Unitree-G1-29dof-Velocity`、`Unitree-Go2-Velocity` 等）。两个仓库共享相同的 ONNX 导出格式，使得 C++ 部署端代码对两个框架训练出的策略都兼容。

### 训练到部署的完整 Pipeline

unitree_rl_mjlab 不只是训练框架——它展示了从仿真到真机的完整闭环。虽然 sim2real 的细节在 Ch23，但在精读代码时理解完整路径有助于你把握每个模块的最终目的：

```
mjlab 训练 (GPU, 4096 envs, ~2h)
  → 自动 ONNX 导出 (policy.onnx + policy.onnx.data)
  → C++ onnxruntime 推理 (50 Hz, Jetson Orin NX)
  → Unitree SDK2 (DDS over Ethernet, 500 Hz)
  → 真机 PD 控制器 (joint position target → motor torque)
```

**ONNX 自动导出**：训练过程中每隔 `save_interval` iterations 自动导出 `policy.onnx` 和 `policy.onnx.data`。ONNX 文件包含 actor 网络（不包含 critic），输入是 actor observation tensor，输出是 action tensor。

**C++ 部署 FSM**：unitree_rl_mjlab 的部署端使用有限状态机管理机器人的启动序列：

```
State_Passive → State_FixStand → State_RLBase
   (等待)        (站立到默认姿态)    (RL 策略接管)
```

`State_FixStand` 使用纯 PD 控制器把关节角度从当前位置平滑过渡到 default pose（训练时的 `use_default_offset=True` 对应的姿态）。这个过渡持续约 2-3 秒，确保机器人在 RL 策略接管前处于稳定的初始状态。如果跳过这个状态直接进入 `State_RLBase`，RL 策略接收到的 observation 可能是完全 out-of-distribution 的（机器人在躺着或歪斜的状态），导致瞬间输出极端 action。

**工程含义**：这个 FSM 解释了为什么 `use_default_offset=True` 如此重要——它不只是让 zero action 好看，它确保了部署时的 FSM 过渡有一个明确的目标姿态。

### 共享 env cfg 的设计模式

unitree_rl_mjlab 的核心设计如下：

```
velocity_env_cfg.py          # robot-agnostic base cfg（共享）
├── go1/env_cfgs.py          # Go1 specific overrides
├── go2/env_cfgs.py          # Go2 specific overrides
├── a1/env_cfgs.py           # A1 specific overrides
├── h1/env_cfgs.py           # H1 specific overrides
└── g1/env_cfgs.py           # G1 specific overrides
```

base cfg 定义完整的 manager 骨架——所有的 observation terms、reward terms、termination conditions、curriculum 逻辑。robot-specific cfg 只做三件事：

1. **指定 Entity**：哪个机器人的 MJCF、default pose、action scale
2. **填入 site/body/geom 名字**：base body、foot sites、contact geoms
3. **调整 robot-specific 参数**：episode length、command range、reward weights

这种设计的好处是**新增机器人只需写约 100 行的 robot-specific override**，不需要复制整个 env cfg。坏处是需要理解 base cfg 的所有"空槽"——哪些参数是必须覆盖的、哪些有合理默认值。

### 从 Go1 到 Go2 的迁移步骤

假设你要在 unitree_rl_mjlab 中添加 Go2 支持。核心步骤：

| 步骤 | 操作 | 需要查的信息 |
|------|------|------------|
| 1 | 获取 Go2 MJCF 模型 | MuJoCo Menagerie 或 Unitree 官方 |
| 2 | 确认 body/site/geom 命名 | 打开 MJCF 文件或用 MuJoCo viewer |
| 3 | 设置 default pose | 查看 MJCF 的 keyframe 或 qpos0 |
| 4 | 设置 action scale | 基于关节限位范围的 30-50% |
| 5 | 填入 foot site names | 通常是 `FR_foot`/`FL_foot`/`RR_foot`/`RL_foot` |
| 6 | 填入 contact geom names | foot collision geom 的 regex |
| 7 | 注册 task id | 在 `__init__.py` 中调用 `register_mjlab_task` |
| 8 | 四阶段验证 | zero → random → small → large |

### MJCF 模型的关键属性

四足机器人的 MJCF 模型中有几个对训练直接影响的属性需要特别关注：

**actuator 定义**：Go1 使用 `position` actuator + `kp`/`kd` gains。这定义了 PD 控制器的行为——`kp` 控制位置跟踪刚度，`kd` 控制阻尼。`kp` 太小导致关节跟踪缓慢（策略发出 target 但关节响应慢），`kp` 太大导致高频振荡。典型值因机器人而异——Go1 和 Go2 的电机特性不同，照搬 gains 会导致行为差异。

```xml
<!-- Go1 MJCF 中的 actuator 定义（简化） -->
<actuator>
  <position name="FR_hip" joint="FR_hip_joint" kp="40" kd="1" />
  <position name="FR_thigh" joint="FR_thigh_joint" kp="40" kd="1" />
  <position name="FR_calf" joint="FR_calf_joint" kp="40" kd="1" />
  <!-- ... 其他 9 个关节 -->
</actuator>
```

**collision geom 的 group 分组**：MuJoCo 的 geom group（0-5）控制渲染和碰撞行为。通常 group 0 用于 terrain 碰撞检测，group 1 用于机器人碰撞几何，group 2 用于视觉渲染。raycast sensor 的 `include_geom_groups=(0,)` 只查询 group 0 的 geom，避免 ray 打到机器人自身。如果机器人模型的碰撞 geom 也在 group 0，raycast 会打到腿部——这是一个隐蔽的 bug。

**关节限位**：MJCF 中的 `range` 定义了关节的物理限位。action scale 应该确保 $[-1, 1]$ 的 action 范围加上 default offset 不会超出限位——否则 MuJoCo 会施加限位力，导致额外的能量注入或不稳定。`dof_pos_limits` reward 惩罚接近限位，但最好的做法是在 action mapping 阶段就 clip 掉超限的 target。

**反事实推理：如果不使用共享 base cfg，每个机器人复制独立的完整 cfg 会怎样？** 假设你修复了 Go1 cfg 中的一个 reward bug。如果 Go2、A1、H1 各有独立的完整 cfg，你需要在每个文件中做相同的修改——容易漏改，一个月后不同机器人的 cfg 开始分叉。共享 base cfg 确保 bug fix 和改进传播到所有机器人。

### MJCF 模型检查工作流

添加新机器人之前，必须对 MJCF 模型做系统检查。以下是推荐的工作流——在 MuJoCo viewer 中检查完以下项目后再开始写配置：

**Step 1：在 viewer 中打开模型**

```bash
# 用 MuJoCo 自带 viewer 检查模型
python -m mujoco.viewer --mjcf path/to/go2.xml
```

在 viewer 中检查以下内容：机器人是否正常站立（重力方向正确？）、关节能否手动拖动（范围合理？）、碰撞体是否覆盖正确（脚底有碰撞 geom？）。

**Step 2：提取关键参数**

```python
import mujoco
m = mujoco.MjModel.from_xml_path("go2.xml")

# 关节名字和限位
for i in range(m.njnt):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
    if m.jnt_limited[i]:
        lo, hi = m.jnt_range[i]
        print(f"{name}: [{lo:.3f}, {hi:.3f}] rad")

# body 名字（用于 base body 和 terrain scan frame）
for i in range(m.nbody):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
    print(f"body {i}: {name}")

# site 名字（用于 foot 位置）
for i in range(m.nsite):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, i)
    print(f"site {i}: {name}")

# geom 名字和 group（用于 contact sensor 和 raycast）
for i in range(m.ngeom):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
    group = m.geom_group[i]
    print(f"geom {i}: {name}, group={group}")
```

这段代码的输出告诉你填写 env cfg 时需要的所有命名信息。把输出保存下来——配置 contact sensor 的 geom regex、raycast 的 body frame、foot site 名字时会反复查看。

**Step 3：Go1 vs Go2 关键参数对比**

| 参数 | Go1 | Go2 | 迁移影响 |
|------|-----|-----|---------|
| 总质量 | ~12 kg | ~15 kg | reward 中的力和力矩量级变化 |
| 腿长 | ~0.2 m（大腿+小腿） | ~0.23 m | 默认站高、foot clearance 阈值需调整 |
| hip 关节范围 | 约 [-0.86, 0.86] rad | 约 [-1.05, 1.05] rad | action scale 需重新计算 |
| kp / kd | 40 / 1 | 依模型定义 | 直接影响 PD 控制器响应 |
| base body 名 | `trunk` | `base_link` | terrain scan frame、projected gravity frame |
| foot site 前缀 | `FR`/`FL`/`RR`/`RL` | 可能是 `FR_foot` 等 | 所有引用 foot site 的 obs/reward |

这张表揭示了一个关键事实：从 Go1 迁移到 Go2 不是改一个 asset 路径——至少有 6 类参数需要逐一确认。遗漏任何一项都可能导致训练能跑但策略行为异常。

### basic-locomotion-isaaclab 的对照

Isaac Lab 生态中，`basic-locomotion-isaaclab`（`github.com/iit-DLSLab/basic-locomotion-isaaclab`，IIT Dynamic Legged Systems Lab）展示了类似的多机器人配置模式，但提供了比 unitree_rl_mjlab 更丰富的功能集。

**支持的机器人**：Go2、B2、Aliengo、ANYmal-C。配置文件区分 electric（Go2/B2/Aliengo）vs hydraulic（ANYmal-C/HyQ）actuator，因为两者的 kp/kd 量级和响应特性差异大。

**Reward 体系**：提供 25+ reward terms，按三类组织：

| 类别 | terms | 数量 |
|------|-------|------|
| **Tracking objectives** | lin_vel_xy, ang_vel_yaw, body_height | ~5 |
| **Regularization penalties** | action_rate, action_diff, joint_vel, joint_accel, torques, orientation | ~12 |
| **Feet-related rewards** | airtime, ground_impact, feet_slip, feet_slide, feet_clearance, undesired_contacts | ~8 |

这套 reward 比 mjlab 内置的更细致——例如它区分了 `feet_slip`（接触中的滑移）和 `feet_slide`（接触瞬间的滑移），并有独立的 `ground_impact`（着地冲击力）reward。如果你想构建一个更精细的 reward 体系，这个仓库是很好的参考。

**DAgger 蒸馏脚本**：`scripts/dagger/train_dagger.py` 实现了从 teacher policy 到 student policy 的 DAgger 蒸馏。teacher 可以是用 privileged height scan 训练的策略，student 只用 proprioceptive history。这直接对应 Ch09 (Teacher-Student) 的工程实践。

**MPC 对比**：集成了 Quadruped-PyMPC，允许在同一机器人上对比 RL 策略和 MPC controller 的表现。这为"RL vs 传统控制"的讨论提供了定量证据。

**state estimator 集成**：集成 `muse` state estimator，在 sim-to-real 部署时提供 base velocity 估计——对应本章 13.1 讨论的"为什么 base_lin_vel 是 privileged"问题。

对比两个项目的工程组织差异：

| 维度 | unitree_rl_mjlab | basic-locomotion-isaaclab |
|------|-----------------|--------------------------|
| 框架 | mjlab | Isaac Lab |
| 模型格式 | MJCF | USD/URDF |
| 共享 base cfg 方式 | Python 函数 + override dict | Python class 继承 |
| 注册方式 | `register_mjlab_task()` | `gymnasium.register()` |
| Reward 数量 | ~10 terms | 25+ terms |
| 部署支持 | ONNX + C++ FSM | ONNX + ROS2 |
| 蒸馏 | 无（需手动实现） | DAgger script 内置 |
| MPC 对比 | 无 | Quadruped-PyMPC 集成 |

**工程建议**：如果你的目标是快速跑通一个 velocity task 并理解配置，unitree_rl_mjlab 更轻量。如果你需要做系统性的 reward 研究、teacher-student 蒸馏、或 RL vs MPC 对比，basic-locomotion-isaaclab 提供了更完整的工具链。两者可以互补使用——在 mjlab 中快速迭代配置，在 Isaac Lab 中做精细评估。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：迁移新机器人时 site names 不匹配**。不同 MJCF 模型的 foot site 命名不统一——Go1 可能用 `FR`/`FL`/`RR`/`RL`，Go2 可能用 `FR_foot`/`FL_foot`/`RR_foot`/`RL_foot`。必须打开 MJCF 确认精确命名。

💡 **概念误区：认为换个机器人只需改 asset 路径**。action scale、default pose、joint 命名、body 命名都是 robot-specific 的。任何遗漏都可能导致静默的 wiring 错误。

### 练习

1. **[源码分析题]** 克隆 unitree_rl_mjlab 仓库，找到 Go1 和 Go2 的 env_cfgs.py。列出两者的所有差异行。按"entity 相关 / reward 相关 / training 相关"分类。
2. **[实践题]** 如果你有一个新的四足机器人（如 MIT Mini Cheetah）的 MJCF 文件，列出在 unitree_rl_mjlab 中添加它的完整步骤和需要填入的所有参数。
3. **[部署分析题]** unitree_rl_mjlab 的 C++ 部署端使用 FSM：`State_Passive → State_FixStand → State_RLBase`。解释：(a) `State_FixStand` 为什么必须在 RL 策略接管前执行？(b) 如果训练时 `use_default_offset=False`，部署端的 `State_FixStand` 需要做什么调整？(c) ONNX 推理频率（50 Hz）和 SDK2 通信频率（500 Hz）之间的 10× 差距如何处理？

---

前面六节分别精读了 mjlab 和 Isaac Lab 的 velocity task。但一个自然的问题是：同一个机器人在两个框架中训练出来的策略有什么差异？这个对比实验不仅帮助你建立跨框架直觉，还能验证你对两个框架的理解是否正确。

## 13.7 双框架对比实验 ⭐⭐⭐

> **这一节解决什么问题**：用同一机器人（Go2）在 mjlab 和 Isaac Lab 中做对照实验，建立跨框架的定量直觉。

### 实验设计

双框架对比的目标不是判定"哪个更好"，而是理解差异的来源。一个公平的对比需要控制以下变量：

| 变量 | 控制方式 |
|------|---------|
| 机器人 | 同一个 Go2（MJCF vs USD 版本，物理参数尽量一致） |
| 任务定义 | 相同的 velocity tracking 目标 |
| RL 算法 | 都用 RSL-RL PPO |
| 超参数 | 尽量对齐（lr、gamma、lam、clip、epochs） |
| 环境数量 | 都用 4096 |
| 随机种子 | 各跑 3 个 seed |

### 无法控制的差异

即使尽力对齐，以下差异是框架固有的：

| 差异源 | mjlab | Isaac Lab | 对训练的影响 |
|--------|-------|-----------|------------|
| 物理引擎 | MuJoCo Warp（凸优化接触） | PhysX（TGS 迭代求解） | 接触响应和穿透处理不同 |
| 接触模型 | soft contact + cone | rigid contact + pyramid 近似 | 摩擦力计算有差异 |
| 积分器 | Euler / RK4 | 固定步长 Euler | 数值精度略有不同 |
| 模型格式 | MJCF（MuJoCo 原生） | USD（需要从 URDF 转换） | 惯性参数、关节阻尼可能有舍入差异 |

> **本质洞察**：双框架对比实验的价值不在于证明"A 比 B 好"，而在于建立对"物理差异如何传导到策略行为"的工程直觉。如果两个框架在同一任务上训练出行为差异明显的策略，差异来源几乎一定是物理引擎的接触模型——而不是 RL 算法或 reward 设计。这种直觉对 sim-to-real（Ch23）至关重要：sim-to-real gap 的根源和跨框架 gap 是同一类问题。

### 实验步骤

**Step 1：mjlab 训练**

```bash
# Go2 flat baseline × 3 seeds
for seed in 42 123 456; do
  uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --env.scene.num-envs 4096 --agent.max-iterations 5000 \
    --agent.run-name go2_flat_mjlab_seed${seed} \
    --agent.seed ${seed}
done

# Go2 rough baseline × 3 seeds
for seed in 42 123 456; do
  uv run train Mjlab-Velocity-Rough-Unitree-Go2 \
    --env.scene.num-envs 4096 --agent.max-iterations 10000 \
    --agent.run-name go2_rough_mjlab_seed${seed} \
    --agent.seed ${seed}
done
```

**Step 2：Isaac Lab 训练**

```bash
# Go2 flat baseline × 3 seeds
for seed in 42 123 456; do
  python scripts/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 4096 --max_iterations 5000 \
    --seed ${seed} --run_name go2_flat_isaaclab_seed${seed}
done
```

**Step 3：统计量收集**

对比实验必须报告统计量而非单次结果。收集以下数据：

```python
# 伪代码：对比分析
for framework in ["mjlab", "isaac_lab"]:
    for seed in [42, 123, 456]:
        # 收集最终 1000 iterations 的平均 tracking reward
        final_tracking = mean(rewards[-1000:])
        # 收集平均 episode length
        avg_ep_len = mean(episode_lengths[-1000:])
        # 收集 steps/s 吞吐
        throughput = mean(steps_per_second)
```

报告格式：`metric = mean ± std`（3 seeds）。

**Step 4：策略行为对比**

在相同的 command 序列下评估两个框架训练出的策略。创建一个固定的测试 command 序列：

```python
test_commands = [
    (1.0, 0.0, 0.0),   # 前进 1 m/s
    (0.0, 0.5, 0.0),   # 侧移 0.5 m/s
    (0.5, 0.0, 0.5),   # 前进 + 转弯
    (-0.5, 0.0, 0.0),  # 后退
    (0.0, 0.0, 1.0),   # 原地转弯
]
```

记录每个 command 下的实际速度和稳定性指标。

**定量报告方法论**：超越"3 seeds 取均值 ± 标准差"的基本报告，考虑以下几点：

1. **效应量 (Cohen's d)**：两个框架的均值差异 / pooled 标准差。d < 0.2 是"差异微小"，d > 0.8 是"差异大"。如果 tracking reward 差异的 Cohen's d 只有 0.3，说明差异虽然存在但在实际意义上不大。
2. **训练曲线对比**：不只比最终值，还要比学习速率（达到某个 threshold 需要多少 iterations）。这在工程中更有意义——两个框架最终 reward 相近但一个需要 2× iterations，选择就很明确了。
3. **行为对比优于数值对比**：两个框架 tracking reward 相同但步态不同是完全可能的——一个可能 trot，另一个可能 pace。数值相同不代表行为等价。每次对比实验都应附带 play 视频。

### 对比分析框架

在 tensorboard 中叠加两个框架的曲线，关注以下指标：

| 指标 | 预期差异 | 差异来源 | 谁通常更优 |
|------|---------|---------|-----------|
| tracking reward 收敛速度 | 中等 | reward 函数实现细节 | 取决于具体配置 |
| 最终 tracking 精度 | 小-中等 | 接触模型差异影响动力学 | 通常相近 |
| episode length | 可能不同 | termination 条件实现差异 | 取决于具体条件 |
| steps/s 吞吐 | 显著 | GPU kernel 效率不同 | mjlab 在纯 locomotion 通常更快 |
| 最终策略步态 | 可能不同 | 接触力、摩擦力计算差异 | 无客观优劣 |
| 内存占用 | 显著 | 框架开销不同 | mjlab 通常更轻量 |

### sim-to-sim 交叉验证

**历史脉络**：sim-to-sim 验证最早被 humanoid-gym (Gu et al., RSS 2024 Best Paper Finalist) 作为正式工程步骤推广。他们在 Isaac Gym 中训练 XBot-S/XBot-L 人形策略后，先在 MuJoCo 中做 sim-to-sim 验证，确认策略在不同物理引擎中也能正常行走，才进行真机部署。这个模式的价值在于：真机测试成本高（可能损坏硬件），而 sim-to-sim 几乎零成本且能发现大部分物理引擎依赖性问题。

一种更深入的对比方法是 sim-to-sim 验证：在框架 A 中训练策略，导出 ONNX，在框架 B 中 play。如果策略在框架 A 中表现好但在框架 B 中崩溃，说明策略过拟合了框架 A 的物理特性。这种交叉验证方法在 sim-to-real 部署前非常有价值——如果策略连跨引擎都不鲁棒，在真机上大概率也不行。

```bash
# Step 1: 在 mjlab 中训练并导出 ONNX
uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
  --env.scene.num-envs 4096 --agent.max-iterations 5000

# Step 2: 在 Isaac Lab 中加载 ONNX play（需要适配 obs 接口）
# 注意：直接加载可能需要处理 obs 顺序和归一化差异
```

回顾 Ch23（Sim2Real 部署全链路）的预告：sim-to-sim 交叉验证是 sim-to-real 管线中的关键中间步骤。ASAP（RSS'25，`github.com/LeCAR-Lab/ASAP`）项目系统性地展示了跨引擎验证的方法论。

**跨领域类比**：sim-to-sim 交叉验证类似于机器学习中的交叉数据集验证——在 ImageNet 上训练的模型在 COCO 上测试，如果性能骤降说明模型过拟合了 ImageNet 的数据分布。同理，在 MuJoCo Warp 中训练的策略在 PhysX 中测试，性能下降反映了策略对特定物理引擎的过拟合程度。Domain Randomization（Ch08）的目的之一就是降低这种过拟合。

### 跨框架迁移的实用经验

基于对比实验的经验，以下是从一个框架迁移到另一个框架的实用建议：

| 迁移方向 | 重点关注 | 常见坑 |
|---------|---------|-------|
| mjlab → Isaac Lab | obs group 名字改 actor→policy | reward 函数实现细节不同 |
| Isaac Lab → mjlab | action cfg 参数名不同 | terrain generator 配置格式不同 |
| 通用 | 打印两个框架的 obs shape 和值域 | 归一化方式可能不同 |

**跨领域类比**：跨框架迁移类似于跨语言翻译——不是逐词替换（逐 API 替换），而是理解语义（理解设计意图）后重新表达。直接复制 mjlab 的 reward 函数到 Isaac Lab 不会工作，因为两个框架的 tensor 布局、sensor 接口、归一化约定可能不同。但如果你理解了"这个 reward 奖励的是什么物理行为"，在新框架中重新实现就很直接。

**跨框架迁移 Checklist**：

从 mjlab 迁移到 Isaac Lab（或反向）时，按以下顺序逐项检查。每一项看起来微小，但遗漏任何一项都可能导致静默的训练失败——策略能训练、loss 在下降，但行为完全不对。

```
□ 模型文件
  - MJCF → USD/URDF 转换后关节命名是否一致？
  - 惯性参数、质量、碰撞几何是否一致？
  - 关节限位、阻尼、摩擦系数是否对齐？
  
□ 时间参数
  - physics_dt 是否一致？（mjlab 默认 0.005s vs Isaac Lab 可能不同）
  - decimation 是否一致？（确保 control_dt = decimation × physics_dt 相同）
  - episode_length 是否一致？
  
□ Observation
  - actor obs group 名字（actor vs policy）
  - observation term 顺序是否影响 tensor 布局？
  - 噪声参数是否对齐？（Gaussian vs Uniform，均值和方差）
  - 值域和归一化方式？

□ Action
  - action scale 数值是否一致？
  - default offset 来源是否一致？（MJCF keyframe vs USD default pose）
  - PD controller gains（kp/kd）是否一致？

□ Reward
  - 同名 reward 的数学形式是否相同？（检查源码）
  - exponential kernel 的 sigma 参数是否一致？
  - 权重是否对齐？

□ Termination
  - 条件和阈值是否一致？
  - timeout 的 bootstrap 处理是否一致？
```

这份 checklist 的核心思想是：两个框架的 MDP 定义必须在语义上等价——不是 API 调用相同，而是物理行为和信号含义相同。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Go2 的 MJCF 和 USD 版本物理参数不一致**。MJCF 版本来自 MuJoCo Menagerie，USD 版本可能来自 URDF 转换。惯性参数、关节阻尼、碰撞几何可能有微妙差异。对比实验前应先确认关键参数一致。

💡 **概念误区：认为框架差异不影响策略行为**。物理引擎的接触模型差异会传导到步态——MuJoCo 的凸优化接触模型在足-地交互中可能比 PhysX 的 TGS 更稳定，导致策略学到的步态模式不同。

🧠 **思维陷阱：只用 1 个 seed 对比**。RL 训练的随机性很大，单 seed 的对比可能被随机噪声淹没。至少用 3 个 seed，报告均值和标准差。

### 练习

1. **[实验设计题]** 设计一个对比实验：在 mjlab 和 Isaac Lab 中分别训练 Go2 flat velocity，用同一组 command 序列评估两个策略的 tracking error。你需要控制哪些变量？如何确保评估的公平性？
2. **[分析题]** 如果两个框架训练出的策略在平地上行为相似，但在 stairs 上差异明显，最可能的原因是什么？从物理引擎的角度分析。
3. **[跨章综合题]** 结合 Ch03（物理引擎工程实践）和 Ch08（Domain Randomization）的知识，解释为什么 domain randomization 在一定程度上能弥补两个物理引擎之间的差异。如果 DR 范围覆盖了两个引擎的行为差异区间，训练出的策略是否能同时在两个引擎中表现良好？

---

## 本章小结

| 知识点 | 核心要点 | 难度 |
|--------|---------|------|
| 四足自由度 | 6 浮基 + 12 关节 = 18 DoF；qpos 19 维因为四元数 | ⭐⭐ |
| 四足运控谱系 | 盲控制(Hwangbo/Lee) → 感知控制(Miki) → 视觉跑酷(Extreme Parkour) | ⭐⭐ |
| 速度跟踪 MDP | command 必须在 actor obs 中；action = default + offset × scale | ⭐⭐ |
| 状态估计 | base_lin_vel 真机不可直接测量；IMU+FK+EKF 或 RMA adaptation module | ⭐⭐⭐ |
| 链路阅读法 | Registry → Entity → Scene → Sensors → Managers → RL cfg | ⭐⭐ |
| Reward 四层框架 | Tracking → Regularization → Style → Contact/Safety；四层独立调参 | ⭐⭐⭐ |
| 非对称 AC | actor 带噪声看可部署信号，critic 看 privileged 无 noise (Pinto et al. RSS 2018) | ⭐⭐ |
| Manager-Based 演进 | 从 legged_gym 单体类到 Manager-Based 配置化；RSL-RL 4.0 统一模型抽象 | ⭐⭐ |
| Flat/Rough | flat = rough − (terrain scan + collision sensors + terrain curriculum) | ⭐⭐ |
| Height scan | grid pattern、yaw alignment、分辨率-成本权衡 | ⭐⭐⭐ |
| Terrain curriculum | game-inspired (Rudin CoRL 2021)；per-env difficulty；auto advance/regress | ⭐⭐ |
| 分阶段验证 | zero → random → small train → large train | ⭐⭐ |
| 多机器人配置 | 共享 base cfg + robot-specific override；7 款 Unitree 机器人统一管线 | ⭐⭐ |
| 部署闭环 | ONNX export → C++ onnxruntime (50Hz) → SDK2 (500Hz) → 真机 PD | ⭐⭐⭐ |
| 双框架对比 | 控制变量实验设计；差异来源是物理引擎接触模型 | ⭐⭐⭐ |
| Action 链路 | 策略 → scale → default offset → PD → torque clamp → physics | ⭐⭐ |
| RewardsCfg 注册 | 13 个 RewardTermCfg 字典 → RewardManager 自动聚合 | ⭐⭐⭐ |
| Reward ablation | 每次关闭一个 term × 3 seeds → 对比表判断重要性 | ⭐⭐⭐ |
| PD gains | kp=25-50, kd=0.5-1.0; 影响跟踪刚度和阻尼 | ⭐⭐ |
| Command curriculum | 渐进扩大速度范围；零速命令特殊置零处理 | ⭐⭐ |

### 关键数字速查

| 数字 | 含义 | 来源 |
|------|------|------|
| 12 | Go1 actuated joints | 13.1 |
| 18 | Go1 总自由度 | 13.1 |
| 48 | Go1 flat actor obs 维度 | 13.2 |
| 76 | Go1 flat critic obs 维度 | 13.2 |
| ~280 | Go1 rough critic obs（含 height scan） | 13.4 |
| 13 | Go1 reward terms 总数 | 13.2 |
| σ=0.25 | tracking reward exponential kernel 参数 | 13.2 |
| 0.25 rad | Go1 统一 action scale | 13.1 |
| kp=25-50 | Go1 PD position gain | 13.1 |
| ~12000 | Go1 flat 训练速度 (steps/s, 4090) | 13.5 |
| ~2000 | Go1 flat 收敛 iterations | 13.5 |
| 4096 | 标准 num_envs | 13.5 |

### 本章与其他章节的关系

| 本章知识 | 前置来源（回顾） | 后续应用（预告） |
|---------|----------------|----------------|
| Asymmetric AC | Ch09 Teacher-Student 的理论基础 | Ch14 人形更依赖 privileged info；Ch18 视觉策略的 teacher |
| Terrain curriculum | Ch06 Curriculum 设计的通用原理 | Ch14 人形多地形；Ch23 Sim2Real terrain transfer |
| Height scan | Ch05 Observation 设计的具体实例 | Ch18 深度相机替代 height scan |
| DR 配置 (startup/interval) | Ch08 DR 原理和 EventManager API | Ch23 Sim2Real 中 DR 是核心手段 |
| 链路阅读法 | Ch04 Manager-Based 架构总论 | 所有后续实战章节（Ch14-Ch22）的源码阅读入口 |
| Reward 四层框架 | Ch06 Reward 设计原理 | Ch14 人形 reward（加入角动量层）；Ch17 操作 reward（去掉步态层，加入物体层） |
| 状态估计 | Ch05 Observation 中的 base_lin_vel | Ch23 Sim2Real 中状态估计是关键 gap |
| 部署 FSM | — | Ch23 完整 sim2real 部署流程 |

## 累积项目 A：本章新增模块

本章完成了累积项目 A 的核心：Go1/Go2 四足在 mjlab 和 Isaac Lab 中的 flat + rough 速度跟踪。

| 模块 | 状态 | 说明 |
|------|------|------|
| Go1 flat velocity | ✅ 完成 | mjlab 内置 |
| Go1 rough velocity | ✅ 完成 | mjlab 内置 |
| Go2 flat velocity | ✅ 完成 | mjlab 内置 / unitree_rl_mjlab |
| ANYmal-C rough velocity | ✅ 完成 | Isaac Lab 内置 |
| 双框架对比实验 | ✅ 完成 | Go2 在两个框架中的对照 |
| Height scan 配置 | ✅ 完成 | grid pattern、ray alignment |
| 四阶段验证 | ✅ 完成 | zero → random → small → large |

**后续扩展方向**：Ch14 将把四足的方法论迁移到人形（G1/H1），核心挑战是支撑面缩小一个数量级、质心更高、角动量管理更关键。Ch18 将引入视觉感知（深度相机替代 height scan）。Ch19 将在四足基座上加装机械臂，引入 loco-manipulation。

### 实践里程碑（建议用时 3-4 天）

| 里程碑 | 预计用时 | 完成标准 | 前置 |
|--------|---------|---------|------|
| M1: Go1 flat zero+smoke | 2h | zero agent 能站 >5s，smoke train 无报错 | Ch04-Ch10 完成 |
| M2: Go1 flat baseline | 4h | tracking reward >0.7，3000 iter，能走 | M1 |
| M3: Reward ablation | 6h | 关闭 pose/foot_slip/upright 各一组，对比表 | M2 |
| M4: Go1 rough 训练 | 4h | rough 地形 tracking >0.5，terrain level >3 | M2 |
| M5: Isaac Lab ANYmal 对照 | 3h | ANYmal flat+rough 训练完成 | M2 |
| M6: 双框架对比实验 | 4h | Go2 在两端训练+评估，对比报告 | M2+M5 |
| M7: 新机器人迁移 | 4h | Go2 或 B2 从零配置到训练成功 | M2 |

**总计 ~27 GPU-hours**（RTX 4090）。M1-M2 是最关键的——如果你能在 Go1 flat 上跑通训练并理解 reward 的因果关系，后续所有里程碑都是同一模式的扩展。

### 快速读懂任意 Velocity Task 配置的 5 分钟流程

当你遇到一个新的 velocity task 配置（如从论文代码下载的第三方项目），以下流程让你在 5 分钟内理解它的核心结构：

```
分钟 1：找 task registration 和 entity
  → 确认机器人型号和 DoF
  → 确认 MJCF/URDF 文件路径

分钟 2：检查 action 配置
  → 是统一 scale 还是 per-joint scale？
  → default pose 来自 MJCF keyframe 还是 hardcoded？

分钟 3：检查 reward 四层
  → Tracking: 有 lin_vel 和 ang_vel？sigma 是多少？
  → Style: 有 pose/upright/feet_air_time？
  → Regularization: 有 action_rate/dof_accel？
  → Contact: 有 foot_slip/undesired_contacts？

分钟 4：检查 observation
  → actor obs 包含 command 吗？（没有→策略不会走）
  → critic 有哪些 privileged terms？
  → base_lin_vel 在 actor 还是只在 critic？

分钟 5：检查 termination + DR
  → fell_over 的阈值？（四足通常 70°，人形 50°）
  → 有 push force？范围多大？
  → 有 terrain curriculum？升级条件是什么？
```

**本章建立的核心能力检查**：完成本章后，你应该具备以下能力。如果对任何一项感到不确定，回到对应小节复习。

| 能力 | 验证方式 | 对应小节 |
|------|---------|---------|
| 读懂任意 velocity task 的完整配置 | 拿到一个新的 velocity env cfg，能在 5 分钟内画出数据流图 | 13.2 |
| 独立跑通 flat + rough 训练 | 从零开始配置，四阶段验证全部通过 | 13.5 |
| 诊断 wiring 错误 | 面对"策略不走"能在 10 分钟内定位原因 | 13.2, 13.5 |
| 解释 flat vs rough 的配置差异 | 能说清删了什么、加了什么、为什么 | 13.2, 13.4 |
| 配置 height scan 传感器 | 给定机器人和地形，能选择合理的 grid 参数 | 13.4 |
| 执行双框架对比实验 | 控制变量设计、多 seed 统计、结果解读 | 13.7 |
| 迁移新机器人 | 给定新 MJCF，能逐项填入配置 | 13.6 |

### 从本章到下一章

本章的四足速度跟踪建立了足式 RL 的完整工程基础。但四足机器人的挑战在于——它"太稳了"。四条腿提供的支撑面积足够大，策略可以通过相对简单的步态维持稳定。Ch14（人形 Locomotion）将把同样的方法论应用到双足机器人上，你会发现本章学到的每一个组件——MDP 结构、reward 设计、terrain curriculum、分阶段验证——都可以直接复用，但所有参数都需要重新调整。这就是工程模式的威力：掌握了模式，新任务的工作量从"从零设计"变成了"调参+验证"。

### 性能基准参考

作为你自己实验的参考，以下是 Go1 velocity task 的典型性能数据（单 GPU，RTX 4090）：

| 配置 | num_envs | steps/s | 收敛 iterations | 最终 tracking reward |
|------|----------|---------|---------------|-------------------|
| Go1 flat (mjlab) | 4096 | ~12000 | ~2000 | ~0.85 |
| Go1 rough (mjlab) | 4096 | ~8000 | ~5000 | ~0.70 |
| ANYmal-C rough (Isaac Lab) | 4096 | ~6000 | ~4000 | ~0.75 |

这些数字仅供参考——实际值取决于 GPU 型号、驱动版本、reward 权重配置等。重要的不是绝对数字，而是数量级和相对关系：rough 比 flat 慢约 30-40%（因为 raycast 开销），收敛需要约 2-3 倍的 iterations（因为任务更复杂）。

### 知识树总结

回顾本章，读者脑中应该有如下知识树：

```
四足速度跟踪
├── 物理基础
│   ├── 浮动基座 + 12 关节 = 18 DoF
│   ├── 步态周期和接触切换
│   └── 运动学链 vs 质心动力学
├── MDP 设计
│   ├── observation：proprioception + command + height scan
│   ├── action：joint position offset + default pose
│   ├── reward：tracking + style + regularization + contact
│   ├── termination：fell_over vs illegal_contact
│   └── asymmetric actor-critic
├── 工程实现
│   ├── mjlab：链路阅读法、base cfg + robot override
│   ├── Isaac Lab：对应 API、obs group 命名差异
│   ├── height scan：grid pattern、yaw alignment
│   └── terrain curriculum：自动难度调整
├── 实验方法
│   ├── 四阶段验证：zero → random → small → large
│   ├── reward ablation
│   └── 双框架对比 + sim-to-sim 交叉验证
└── 调试
    ├── wiring 错误排查
    ├── 调参优先级
    └── 故障排查手册
```

## 延伸阅读

### 学术论文

| 资料 | 难度 | 会议/期刊 | 说明 |
|------|------|----------|------|
| Hwangbo et al., "Learning agile and dynamic motor skills for legged robots," 2019 | ⭐⭐⭐ | *Science Robotics* 4(26) | 四足 RL sim-to-real 的奠基工作；actuator network 的原始出处；ANYmal 平台 |
| Lee et al., "Learning quadrupedal locomotion over challenging terrain," 2020 | ⭐⭐⭐ | *Science Robotics* 5(47) | 纯本体感知 zero-shot 到自然地形；teacher-student pipeline 的奠基 |
| Kumar et al., "RMA: Rapid Motor Adaptation for Legged Robots," 2021 | ⭐⭐⭐ | RSS 2021 | base policy + adaptation module 实现实时在线适应；Unitree A1 部署 |
| Rudin et al., "Learning to Walk in Minutes Using Massively Parallel Deep RL," 2021 | ⭐⭐ | CoRL 2021 | legged_gym 论文；GPU 并行训练 + terrain curriculum 的工程标准 |
| Margolis & Agrawal, "Walk These Ways: Tuning Robot Control for Generalization with MoB," 2022 | ⭐⭐ | CoRL 2022 | Multiplicity of Behavior 步态参数化；单策略支持多种步态风格 |
| Miki et al., "Learning robust perceptive locomotion for quadrupedal robots in the wild," 2022 | ⭐⭐⭐ | *Science Robotics* | 深度图 + 特权学习；DARPA SubT 1700m 零跌倒；从盲控制到感知控制的里程碑 |
| Pinto et al., "Asymmetric Actor Critic for Image-Based Robot Learning," 2018 | ⭐⭐ | RSS 2018 | Asymmetric actor-critic 的原始论文；actor 看图像，critic 看完整状态 |
| Cheng et al., "Extreme Parkour with Legged Robots," 2024 | ⭐⭐⭐ | ICRA 2024 | 视觉端到端四足跑酷；三阶段训练管线（盲控制→深度蒸馏→部署） |
| Schwarke et al., "RSL-RL: A Learning Library for Robotics Research," 2025 | ⭐⭐ | arXiv 2509.10771 | RSL-RL 4.0 论文；PPO + Distillation + Symmetry 统一库 |

### 工具和文档

| 资料 | 难度 | 说明 |
|------|------|------|
| RSL-RL 配置文档 | ⭐ | `leggedrobotics.github.io/rsl_rl/guide/configuration.html` |
| mjlab 官方文档 | ⭐ | `github.com/mujocolab/mjlab` |
| Isaac Lab velocity tutorial | ⭐⭐ | `isaac-sim.github.io/IsaacLab/main/source/tutorials/` |
| unitree_rl_mjlab | ⭐⭐ | `github.com/unitreerobotics/unitree_rl_mjlab`，Unitree 官方 mjlab 项目 |
| unitree_rl_lab | ⭐⭐ | `github.com/unitreerobotics/unitree_rl_lab`，Unitree 官方 Isaac Lab 项目 |
| basic-locomotion-isaaclab | ⭐⭐ | `github.com/iit-DLSLab/basic-locomotion-isaaclab`，IIT DLS Lab 多机器人项目 |
| MuJoCo Menagerie | ⭐ | `github.com/google-deepmind/mujoco_menagerie`，50+ 机器人模型库 |
| MuJoCo XML Reference | ⭐ | `mujoco.readthedocs.io/en/stable/XMLreference.html` |

### 阅读路线建议

- **最小路线**（只做 flat velocity）：Rudin 2022 → 本章 13.1-13.2 → 13.5
- **标准路线**（flat + rough）：上述 + Lee 2020 → 本章 13.4 → 13.5-13.6
- **进阶路线**（准备做视觉控制）：上述 + Miki 2022 + Extreme Parkour → Ch18
- **研究路线**（准备发论文）：上述 + RMA + Walk These Ways + Hwangbo 2019 → 设计自己的 reward/DR 方案

## 🔧 故障排查手册

| # | 症状 | 可能原因 | 排查步骤 | 相关小节 |
|---|------|---------|---------|---------|
| 1 | 策略不响应 command 变化 | actor obs 缺 command term | 1. 打印 actor obs keys 2. 确认 command term 存在 3. 检查 command_name 是否为 `"twist"` | 13.1, 13.2 |
| 2 | foot contact reward 全零 | sensor name 或 geom name regex 匹配空集 | 1. 打印 sensor ids 2. 检查 MJCF 中 geom 精确命名 3. 用 zero agent 打印 sensor data | 13.2 |
| 3 | rough episode 极短 | 保留了 flat 的 `fell_over` termination | 1. 打印 termination 计数 2. 确认 rough cfg 删除了 `fell_over` 3. 确认使用 `illegal_contact` | 13.2 |
| 4 | flat 启动 crash "sensor not found" | flat 保留了 `height_scan` obs 但删了 terrain scan sensor | 1. 检查 flat cfg 的 obs terms 2. 确认删除了 actor 和 critic 的 height_scan | 13.2, 13.4 |
| 5 | tracking reward 很低但其他 reward 正常 | command 和 tracking reward 引用不同 command source | 1. 确认 command obs 和 tracking reward 都引用 `"twist"` 2. 打印实际 command 值 | 13.1, 13.2 |
| 6 | 粗糙地形步态不稳定 | height scan 的 ray alignment 设为 "base" 而非 "yaw" | 1. 检查 `ray_alignment` 参数 2. 在坡地上可视化 scan 数据 3. 改为 "yaw" 重新训练 | 13.4 |
| 7 | steps/s 突然下降 20%+ | 增加了高分辨率 terrain scan 或额外 contact sensor | 1. 记录增加 sensor 前后的 steps/s 2. 计算 ray 数变化 3. 考虑降低分辨率 | 13.4 |
| 8 | 跨框架迁移后 obs shape mismatch | obs group 名字不匹配（actor vs policy） | 1. 检查 RSL-RL cfg 的 obs_groups 2. mjlab 用 "actor"，Isaac Lab 用 "policy" 3. 统一命名 | 13.3 |
| 9 | Go2 迁移后 reward 不涨 | robot-specific site/body names 未更新 | 1. 比较 Go1 和 Go2 的 MJCF naming 2. 逐一检查 entity wiring 3. 用 zero agent 打印所有 entity data | 13.6 |
| 10 | 双框架训练结果差异大 | 物理引擎接触模型差异 + 超参未对齐 | 1. 确认 PPO 超参一致 2. 打印两框架的 obs 值域 3. 接受物理引擎差异是正常的 | 13.7 |
| 11 | 训练初期 reward 剧烈震荡（非单调上升） | random push event 与 tracking reward 冲突 | 1. 检查 push interval 和 push force magnitude 2. 关闭 push 观察 reward 趋势 3. 如需保留 push，在 push 后 3-5 步 mask tracking reward | 13.5 |
| 12 | PhysX 报错 material limit exceeded | DR 中材料数超过 64,000 上限 | 1. 检查 `num_buckets` 参数是否设置 2. 4096 envs × N bodies × M friction variants 是否超限 3. 增加 `num_buckets` | 13.2 |
| 13 | MuJoCo 启动时拒绝模型 "inertia not valid" | DR 产生了非物理惯性参数 | 1. 检查 mass/inertia 随机化范围 2. 确保 principal moments 满足三角不等式 3. 使用 LMI 投影或更保守的随机化范围 | 13.2 |
| 14 | ONNX 导出后 C++ 推理结果与 PyTorch 不同 | obs 归一化参数（running mean/var）未导出 | 1. 确认 ONNX 文件包含 normalization layer 2. 检查 C++ 端是否加载了 obs_mean 和 obs_var 3. 手动对比单步推理输出 | 13.6 |
| 15 | sim-to-sim 速度差异 >30% | control_dt 不一致 | 1. 计算 decimation × physics_dt 是否两框架相同 2. 对齐 physics_dt 或 decimation 3. 重新训练 | 13.7 |

## Debug Checklist

**Entity & Sensor Wiring**

- [ ] task id 拼写正确（`Mjlab-Velocity-Flat-Unitree-Go1`，注意大小写和连字符）
- [ ] entity key 是 `"robot"`
- [ ] terrain scan frame 是 `trunk`（或机器人对应的 base body 名）
- [ ] foot contact sensor 的 geom name 与 MJCF 精确匹配
- [ ] reward 中 foot site names 已填入（非空 tuple）

**Observation & Action**

- [ ] actor observation 包含 `command` term，command_name 是 `"twist"`
- [ ] critic observation 包含 privileged contact terms（foot_height, air_time 等）
- [ ] `obs_groups` 与 env observation group 名字一致（mjlab: actor/critic, Isaac Lab: policy/critic）
- [ ] action scale 适合当前机器人（查看关节限位范围的 30-50%）

**Flat / Rough 切换**

- [ ] flat cfg 已同步删除 actor 和 critic 的 `height_scan`
- [ ] rough cfg 删除 `fell_over`，使用 `illegal_contact`
- [ ] flat cfg 恢复 `fell_over`，删除 `illegal_contact` 和 `out_of_terrain_bounds`
- [ ] rough terrain 的 flat patches 已启用（避免 reset 到地形边缘）
- [ ] terrain_scan sensor 的 `include_geom_groups` 不包含机器人自身的 geom group

**训练配置**

- [ ] command 重采样时间在合理范围（3-8 秒）
- [ ] PPO experiment name 有区分度（含机器人名 + terrain type）
- [ ] 训练日志目录名包含日期+机器人+terrain type+seed（便于回溯）
- [ ] 所有 mjlab 命令使用 `uv run`

**验证流程**

- [ ] 先 zero agent 验证 → 再 small train → 再 large train
- [ ] zero agent 下机器人默认姿态稳定（不倒、不飘）

**Domain Randomization**

- [ ] motor strength DR 的 scale_range 不超过 [0.7, 1.3]（过大导致部分关节完全失效）
- [ ] added_mass 范围不超过机器人自身质量的 25%（过大导致动力学差异太极端）
- [ ] mass/inertia 随机化使用 `scale` 模式（避免 pseudo-inertia 违反三角不等式）

**跨框架对比**

- [ ] decimation × physics_dt 在两个框架中一致（跨框架对比实验必检项）
- [ ] obs group 名字已适配（mjlab: actor/critic → Isaac Lab: policy/critic）
- [ ] ONNX 导出包含 obs normalization 参数

---

## 附录 A：Velocity Task Reward Term 完整参考表

以下表格汇总 mjlab Go1/Go2 rough velocity task 的所有 reward terms。这张表是 reward ablation 实验和调参时的查阅工具。

| Term | 四层分类 | 默认权重 | Kernel 类型 | 数学形式 | 对步态的影响 |
|------|---------|---------|------------|---------|------------|
| `track_linear_velocity` | Tracking | +2.0 | exponential | $\exp(-\|v_{xy} - v_{cmd}\|^2 / \sigma^2)$ | 核心：跟踪线速度 |
| `track_angular_velocity` | Tracking | +2.0 | exponential | $\exp(-\|\omega_z - \omega_{cmd}\|^2 / \sigma^2)$ | 核心：跟踪角速度 |
| `upright` | Style | +1.0 | L2 | $-\|g_{projected} - [0,0,-1]\|^2$ | 保持基座水平 |
| `pose` | Style | +1.0 | L2 | $-\|q - q_{default}\|^2$ | 关节回归默认姿态 |
| `base_height` | Style | +0.5 | L2 | $-(h - h_{target})^2$ | 防止蹲下或伸展 |
| `dof_pos_limits` | Regularization | -1.0 | soft penalty | 接近限位时线性增长 | 避免关节撞限位 |
| `action_rate_l2` | Regularization | -0.1 | L2 | $-\|a_t - a_{t-1}\|^2$ | 抑制动作抖动 |
| `dof_acceleration` | Regularization | -0.0025 | L2 | $-\|\ddot{q}\|^2$ | 减少关节加速度 |
| `joint_torques` | Regularization | -0.0001 | L2 | $-\|\tau\|^2$ | 降低能耗 |
| `foot_clearance` | Contact | -2.0 | conditional | swing 阶段脚低于阈值则惩罚 | 鼓励抬脚（防止拖脚） |
| `foot_slip` | Contact | -0.1 | L2 | $-\|v_{foot}\|^2$ (contact 时) | 惩罚触地时滑移 |
| `feet_air_time` | Style | +0.5 | bonus | 空中时间在目标范围内给 bonus | 鼓励有节奏的步态 |
| `undesired_contacts` | Safety | -1.0 | binary | 膝/大腿碰地 = 1 | 避免非法接触 |
| `angular_velocity_xy` | Regularization | -0.05 | L2 | $-\|\omega_{xy}\|^2$ | 抑制 roll/pitch 振荡 |
| `linear_velocity_z` | Regularization | -2.0 | L2 | $-\|v_z\|^2$ | 抑制垂直振动 |

**使用方法**：做 reward ablation 时，每次只关闭一个 term（设权重为 0），训练 3000 iterations，观察步态变化。关闭 `foot_slip` 通常导致打滑步态；关闭 `action_rate_l2` 导致高频抖动；关闭 `pose` 导致蹲走或扭曲步态。

**权重调参经验法则**：
- Tracking 权重固定为 baseline（不调），其余相对于 tracking 调
- Regularization 从 0 开始逐步增大，直到"动作变平滑但 tracking 不明显下降"
- Style 权重决定步态风格——想要更高的抬脚高度就增大 `foot_clearance`
- Safety 权重通常保持较大——它是红线，不应被其他 reward 压制

---

## 附录 B：Velocity Task 实验记录模板

每次训练实验应记录以下信息，便于复现和对比：

```
实验名称: go1_flat_baseline_seed42
日期: 2026-05-20
框架: mjlab v0.2.1
机器人: Go1
地形: flat
commit: abc1234
GPU: RTX 4090
num_envs: 4096
max_iterations: 5000
seed: 42

配置差异（相对 base）:
  reward/track_lin_vel_weight: 2.0
  reward/foot_slip_weight: -0.1
  dr/motor_strength_range: [0.8, 1.2]

结果:
  final tracking error: 0.12 m/s
  final ang tracking error: 0.18 rad/s
  fall rate: 3.2%
  avg episode length: 920 steps
  steps/s: 11800
  wall time: 47 min

观察:
  - 步态：稳定 trot
  - 缺陷：左转时偶尔打滑
  - 下一步：增大 foot_slip 权重到 -0.2

视频: logs/rsl_rl/go1_flat_baseline_seed42/videos/iter_5000.mp4
```

这个模板看起来繁琐，但在你做了 30+ 实验后，它是唯一能帮你回溯"为什么那次实验效果好"的工具。建议把实验记录存为 YAML 或 JSON 文件，方便后续脚本化分析。

---

## 附录 C：Go1 Velocity Task 完整 Observation 参考表

### Actor Observation（可部署传感器）

| Term | 维度 | 来源 | 噪声 (std) | 说明 |
|------|------|------|-----------|------|
| `base_lin_vel` | 3 | IMU + FK + EKF | 0.1 m/s | 在某些配置中只放 critic |
| `base_ang_vel` | 3 | 陀螺仪 | 0.2 rad/s | 真机可直接测量 |
| `projected_gravity` | 3 | 加速度计 | 0.05 | 重力在 base frame 的投影 |
| `command` | 3 | 外部输入 | 0 | $(v_x^{cmd}, v_y^{cmd}, \omega_z^{cmd})$ |
| `joint_pos` | 12 | 编码器 | 0.01 rad | 相对 default pose 的偏差 |
| `joint_vel` | 12 | 编码器差分 | 1.5 rad/s | 真机噪声约 0.5 rad/s |
| `last_action` | 12 | 上一步输出 | 0 | 策略自身的历史 |
| **总计** | **48** | | | |

### Critic 额外 Observation（仅训练用 privileged）

| Term | 维度 | 来源 | 说明 |
|------|------|------|------|
| `foot_height` | 12 | FK 精确计算 | 4脚 × 3D 位置 |
| `foot_contact_forces` | 12 | 仿真接触力 | 4脚 × 3D 力 |
| `foot_air_time` | 4 | 接触检测 | 每脚空中持续时间 |
| **critic 总计** | **48+28=76** | | rough 时更大（+height scan ~200维） |

### Rough 地形额外 Observation

| Term | 维度 | 来源 | 说明 |
|------|------|------|------|
| `height_scan` | ~200 | Raycast | grid 采样的地形高度（仅 critic/teacher） |
| `terrain_normal` | 3 | Heightfield 梯度 | 脚底地形法向量 |

**维度对比总结**：

| 配置 | Actor obs | Critic obs | 说明 |
|------|----------|-----------|------|
| Go1 flat | 48 | ~76 | 最小配置 |
| Go1 rough (teacher) | 48 | ~280 | +height scan ~200 维 |
| G1 flat (Ch14) | 99 | 114 | 29-DoF → 维度更大 |
| G1 tracking (Ch15) | 160 | — | +motion reference ~60 维 |

---

## 附录 D：PPO 训练超参参考

### RSL-RL 默认超参

| 超参 | 默认值 | 合理范围 | 调参建议 |
|------|--------|---------|---------|
| `learning_rate` | 1e-3 | 3e-4 ~ 3e-3 | 如果 KL 过大→降低 |
| `gamma` | 0.99 | 0.95 ~ 0.999 | 长 horizon 任务用 0.99+ |
| `lam` (GAE λ) | 0.95 | 0.9 ~ 0.97 | 偏差-方差平衡 |
| `clip_range` | 0.2 | 0.1 ~ 0.3 | 通常不需要改 |
| `entropy_coef` | 0.01 | 0.001 ~ 0.05 | entropy 坍塌时增大 |
| `value_loss_coef` | 1.0 | 0.5 ~ 2.0 | 通常不需要改 |
| `max_grad_norm` | 1.0 | 0.5 ~ 5.0 | 梯度爆炸时减小 |
| `num_learning_epochs` | 5 | 3 ~ 8 | 每次 rollout 后更新几次 |
| `num_mini_batches` | 4 | 2 ~ 8 | mini-batch 数量 |
| `num_steps_per_env` | 24 | 16 ~ 48 | 每个 env 采集多少步 |

### 网络架构默认值

| 参数 | Actor | Critic | 说明 |
|------|-------|--------|------|
| `hidden_dims` | [512, 256, 128] | [512, 256, 128] | 四足标准大小 |
| `activation` | ELU | ELU | 避免 dead neuron |
| `init_noise_std` | 1.0 | — | 初始 action 噪声 |

**何时需要改网络大小？**
- 如果 tracking reward 上升极慢（>5000 iter 仍未进入步态涌现期）→ 网络可能太小，增大到 [768, 512, 256]
- 如果训练非常快但泛化差（不同 command 的表现差异大）→ 网络可能太大，减小到 [256, 128]
- 人形（Ch14）通常需要更大网络（obs 维度更大）

### 不同任务规模的推荐配置

| 任务 | num_envs | max_iterations | GPU 时间 | 预期 reward |
|------|----------|---------------|---------|-----------|
| Go1 flat smoke test | 256 | 50 | 2 min | 不重要 |
| Go1 flat baseline | 4096 | 3000 | 45 min | >0.7 |
| Go1 rough | 4096 | 8000 | 2 h | >0.5 |
| Go2 flat (mjlab) | 4096 | 3000 | 45 min | >0.7 |
| ANYmal rough (Isaac Lab) | 4096 | 5000 | 1.5 h | >0.6 |

---

## 附录 E：关键数字速查

| 数字 | 含义 | 来源 |
|------|------|------|
| 12 | Go1 actuated joints | 13.1 |
| 18 | Go1 总自由度 (6 浮基 + 12 关节) | 13.1 |
| 19/18 | qpos/qvel 维度差（四元数 4 vs 角速度 3） | 13.1 |
| 48 | Go1 flat actor obs 维度 | 13.2 |
| 13 | Go1 reward terms 数量 | 13.2 |
| σ=0.25 | tracking reward exponential kernel 参数 | 13.2 |
| 0.25 | Go1 action scale (统一) | 13.2 |
| ~200 | height scan obs 维度 (rough) | 13.4 |
| ~12000 | Go1 flat 训练速度 (steps/s, 4090) | 13.5 |
| ~2000 | Go1 flat 收敛 iterations | 13.5 |
| 4096 | 标准 num_envs | 13.5 |
| 3 | ablation 最少 seed 数 | 13.5 |
| 10 | terrain curriculum difficulty levels | 13.4 |
| 5 | terrain types per difficulty | 13.4 |
| 0.03 m | foot clearance 最小抬脚阈值 | 13.2 |
| 23.7 Nm | Go1 knee 最大力矩 | 13.1 |

---

> **结语**：四足速度跟踪是整本教材最核心的 baseline task。本章建立的每一个概念——链路阅读法、四层 reward 框架、分阶段验证、reward ablation、双框架对比——都会在后续章节中反复使用。如果你在实践中对某个配置项的作用感到困惑，回到本章的对应小节，从反事实推理开始重新建立因果链理解。从 Ch14 开始，你将把这些方法论迁移到人形、操作和视觉控制——形式变了，但工程模式不变。
