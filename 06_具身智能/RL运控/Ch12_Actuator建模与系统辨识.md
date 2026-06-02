# Ch12 | Actuator 建模与系统辨识

> **本章定位**：Ch11 建立了机器人模型的几何和运动学基础。但一个正确的"骨架"还不够——机器人的运动能力最终由它的"肌肉"（actuator）决定。Sim-to-real 领域的共识是：**当 Domain Randomization 和视觉增强已经做好时，残余的 sim-to-real gap 主要来自关节力矩跟踪和通信/控制延迟。** 本章解决这个最后的瓶颈。
>
> **前置依赖**：Ch05（Action Space 设计）、Ch08（Domain Randomization）、Ch11（机器人模型）
>
> **参考项目**：✅ `Improbable-AI/walk-these-ways`（CoRL'22）· ✅ `LeCAR-Lab/ASAP`（RSS'25）

---

## 前置自测

📋 **答不出 ≥ 3 题 → 先回前置章节复习**

| # | 问题 | 检查目的 |
|---|------|----------|
| 1 | PD 控制器的输出力矩公式是什么？kp 和 kd 分别控制什么物理量？ | 控制基础 |
| 2 | Ch05 中讨论的 position action 和 torque action 各有什么优缺点？ | Action space 设计 |
| 3 | Ch08 中的 Domain Randomization 为什么要随机化 actuator 的 kp/kd？ | DR 与 actuator 的关系 |
| 4 | Ch11 的 MJCF 中，`<position>` 和 `<motor>` actuator 类型有什么区别？ | MJCF actuator |
| 5 | 什么是传递函数？一个二阶系统的传递函数有哪些参数？ | 系统辨识基础 |
| 6 | 一阶低通滤波器 $\tau \dot{y} + y = x$ 的时间常数 τ 的物理意义是什么？ | 信号处理基础 |

## 本章目标

学完本章后，你应该能够：

1. **区分** actuator 建模的四个层级（Ideal PD → DC Motor → Actuator Network → Delta Action Model），理解每个层级解决什么问题
2. **配置** mjlab 和 Isaac Lab 中的各级 actuator 模型，从 `<position>` 到 `DCMotorCfg` 到自定义 `ActuatorNetMLPCfg`
3. **实现** 一个简化的 actuator network：从数据收集到 MLP 训练到仿真器集成
4. **设计** 系统辨识实验：扫频、阶跃响应、摩擦测量，从实验数据拟合 actuator 参数
5. **实现** ASAP 风格的 delta action model，理解它与 actuator network 的本质区别
6. **诊断** actuator 模型不匹配导致的 sim-to-real 问题，选择正确的建模层级

---

## 12.1 算法回顾：Actuator 模型层级 ⭐

> **这一节解决什么问题**：用 20% 的篇幅建立 actuator 建模的全局视角——四个层级的递进关系、每个层级解决什么问题、适用什么场景。

### 动机：为什么仿真器的默认 actuator 不够

回顾 Ch05：RL 策略输出 action（通常是目标关节角 $q^*$），仿真器的 actuator 模型把 $q^*$ 转换为实际施加在关节上的力矩 $\tau$。最简单的模型是 Ideal PD：

$$\tau = K_p (q^* - q) - K_d \dot{q}$$

这个模型假设：(1) 力矩可以瞬间达到任意值（无带宽限制），(2) 力矩与位置误差严格线性，(3) 没有摩擦、死区或饱和。真实电机没有一个满足这些假设。

### 如果不做任何 actuator 建模会怎样

**如果使用 Ideal PD 训练然后直接部署到真机会怎样？** 仿真中策略学到的是"在理想世界中的最优行为"——它可能依赖瞬时大力矩（真机做不到）、忽略摩擦补偿（真机需要）、不考虑电机饱和（真机有上限）。这些差异汇集在一起就是 **sim-to-real gap 的最大来源之一**。

一个具体的例子：仿真中策略学到"在空中快速摆腿可以加速翻转"——这需要在 20ms 内产生 30 N·m 的力矩反转。真实电机的带宽只有 30 Hz（≈33ms 响应时间），而且在关节高速旋转时反电动势降低了可用力矩。结果就是：仿真中漂亮的空翻，到了真机上变成了尴尬的半翻——因为策略计划的力矩时序在真机上执行不出来。

### 如果只做 Domain Randomization 而不做 actuator 建模会怎样

Ch08 的 DR 可以在一定程度上覆盖 actuator 差异——通过随机化 kp/kd，策略被迫适应不同的力矩响应。但 DR 的覆盖是"粗暴的均匀覆盖"——它不知道真机的 actuator 特性在哪个方向偏。而 actuator 建模是"精确的定点修正"——它告诉仿真器"真机就是这样的"。

这类似于视力矫正：DR 是"配一副度数范围很宽的渐进多焦镜"——在任何度数下都能看得差不多。Actuator 建模是"去验光然后配精确度数的眼镜"——在你的度数下看得最清楚。前者更通用但不精确，后者更精确但需要额外的检查（数据收集）步骤。

### 四个层级

```text
Level 0: Ideal PD          τ = Kp(q* - q) - Kd·q̇
  ↓ 加入带宽限制和饱和
Level 1: DC Motor           τ = clip(Kp(q* - q) - Kd·q̇, -τ_max(q̇), τ_max(q̇))
  ↓ 用神经网络拟合真机响应
Level 2: Actuator Network   τ = MLP(q*, q, q̇, q̇_history)
  ↓ 在 action 空间做残差修正
Level 3: Delta Action Model a_corrected = a_policy + MLP_delta(s)
```

这四个层级之间的关系类似于地图的精度等级：Level 0 是"世界地图"——能看到大陆但看不到城市。Level 1 是"国家地图"——能看到主要城市和公路。Level 2 是"街道地图"——每条小路都标注了。Level 3 不画新地图，而是在现有地图上贴"纠错贴纸"——只修正差异最大的地方。

| 层级 | 方法 | 精度 | 工程复杂度 | 数据需求 | 典型用途 |
|------|------|------|----------|---------|---------|
| 0 | Ideal PD | 低 | 极低 | 无 | 初期算法验证 |
| 1 | DC Motor | 中 | 低 | 电机数据手册 | 大多数四足项目 |
| 2 | Actuator Network | 高 | 中 | 5-10 分钟真机数据 | 高精度 sim2real |
| 3 | Delta Action Model | 最高 | 高 | 真机 rollout | 极致 agility |

### 历史演进

| 年份 | 里程碑 | 方法 | 贡献 |
|------|--------|------|------|
| 2018 | Tan et al. | Analytic actuator model | 首次在四足 sim2real 中使用 actuator model |
| 2019 | Hwangbo et al. (Science Robotics) | **Actuator Network (MLP)** | ANYmal 上的黑箱神经网络 actuator model——**原创** |
| 2022 | walk-these-ways (CoRL'22) | MoB + 标准 DR | Go1 上的多策略 locomotion（非 actuator net） |
| 2025 | ASAP (RSS'25) | **Delta Action Model** | 在 action 空间做残差修正，G1 跳旋 |
| 2025 | UAN (MIT CSAIL) | Unsupervised Actuator Network | 5 分钟真机数据无监督拟合 |

> **重要更正：** Actuator Network 的原创者是 **Hwangbo et al. 2019**（ANYmal, Science Robotics），而非 walk-these-ways。walk-these-ways（Margolis & Agrawal, **CoRL 2022**, arXiv 2212.03238）的核心贡献是**Multiplicity-of-Behavior (MoB) gait conditioning**，它使用了标准的 Isaac Gym DR（Rudin et al. 2021），不是 actuator network。很多中文资料错误地把 actuator network 归因于 walk-these-ways——本教材做出纠正。注意：大纲中将 walk-these-ways 标注为 "RSS'23" 是将其与同组的另一篇论文混淆，此处已修正。

### ⚠️ 常见陷阱

🧠 **思维陷阱：更高精度的 actuator 模型总是更好。** 如果你的 DR 范围足够宽（Ch08 的 kp 随机化 U(0.75, 1.5)），Ideal PD + DR 可能已经"覆盖"了真机的 actuator 特性。只有当 DR 不够时（例如需要跳跃、旋转等极限动作），才值得投入 Level 2-3 的工程量。

💡 **概念误区：Actuator Network 和 Delta Action Model 是互斥的。** 它们解决的是不同层面的问题：Actuator Network 建模的是"电机物理"（给定命令 → 实际力矩），Delta Action Model 建模的是"整体 gap"（包括电机、延迟、软件栈差异）。理论上可以同时使用——先用 actuator network 提高电机建模精度，再用 delta action 处理残余 gap。但实际上 ASAP 的结果表明 delta action 单独就足够了。

### 练习

1. **[分类题]** 以下场景各适合哪个 actuator 建模层级？(a) 博士生第一周学习 RL locomotion (b) 准备 ICRA 2026 demo 的 G1 跳旋 (c) 工厂环境中 Go1 的巡逻行走
2. **[思考题]** 为什么 ASAP 的 delta action model 在 action 空间做修正而非在 state 空间？如果在 state 空间做残差（预测下一状态的修正），会遇到什么问题？
3. **[跨章综合题]** 结合 Ch08（DR）：如果 actuator 的 kp 在真机上是 100±20，Ch08 的 DR 应该如何设置才能覆盖这个范围？如果真机的 kp 是 100 但带宽只有 30 Hz（而仿真假设无限带宽），DR 能覆盖这个差异吗？

---

上一节建立了四个层级的全局视角。下两节分别讲解最常用的两个层级的工程实现：DC Motor 模型和 Actuator Network。

## 12.2 Level 0-1：Ideal PD 与 DC Motor 模型 ⭐⭐⭐

> **这一节解决什么问题**：最基础也是最常用的 actuator 模型——Ideal PD 和 DC Motor——在 mjlab 和 Isaac Lab 中如何配置、每个参数的物理意义、以及何时从 Level 0 升级到 Level 1。

### Ideal PD 在 MuJoCo 中的实现

MuJoCo 的 `<position>` actuator 就是 Ideal PD 控制器：

```xml
<!-- MJCF 中的 Ideal PD actuator -->
<actuator>
  <position name="hip_act" joint="hip_joint"
            kp="100"        <!-- 比例增益 (N·m/rad) -->
            kv="4"          <!-- 微分增益 (N·m·s/rad) -->
            ctrlrange="-1.57 1.57"  <!-- 控制输入范围 -->
            forcelimited="true"
            forcerange="-33.5 33.5"  <!-- 输出力矩限制 (N·m) -->
  />
</actuator>
```

MuJoCo 内部的力矩计算公式：

$$\tau = K_p \cdot (q^* - q) - K_v \cdot \dot{q}$$

其中 $q^* = \text{ctrl}$ 是策略输出的目标角度。注意 MuJoCo 使用 `kv`（velocity gain）而非 `kd`（derivative gain）——两者在 position actuator 中等价，但在 MuJoCo 的 `<general>` actuator 中有区别。

⚠️ **关键细节：MuJoCo 的 `<position>` actuator 的 kv 和 `<joint>` 的 damping 效果叠加。** 如果你设了 `<position kv="4"/>` 且 `<joint damping="0.5"/>`，总阻尼是 4.5。这和 Ch11 讨论的跨仿真器对齐直接相关。

### Ideal PD 在 Isaac Lab 中的实现

```python
# Isaac Lab 中的 Ideal PD actuator 配置
from isaaclab.actuators import ImplicitActuatorCfg, IdealPDActuatorCfg

# 方式 1：ImplicitActuator（PhysX 内部处理 PD）
# 最简单，但无法在 Python 中观察计算出的力矩
legs_implicit = ImplicitActuatorCfg(
    joint_names_expr=[".*hip.*", ".*thigh.*", ".*calf.*"],
    stiffness=100.0,     # 等价于 MuJoCo kp
    damping=4.0,         # 等价于 MuJoCo kv + joint_damping 总和
    effort_limit=33.5,   # 力矩上限 (N·m)
    velocity_limit=21.0, # 速度上限 (rad/s)
)

# 方式 2：IdealPDActuator（Python 中显式计算力矩）
# 可以在 Python 中看到 tau = Kp(q*-q) - Kd*qdot
legs_explicit = IdealPDActuatorCfg(
    joint_names_expr=[".*hip.*", ".*thigh.*", ".*calf.*"],
    stiffness=100.0,
    damping=4.0,
    effort_limit=33.5,
    velocity_limit=21.0,
)
```

**ImplicitActuator vs IdealPDActuator 的关键区别：**

| 维度 | ImplicitActuator | IdealPDActuator |
|------|-----------------|-----------------|
| PD 计算位置 | PhysX 内部（C++） | Python 层 |
| 速度 | 更快（减少 Python↔C++ 通信） | 略慢 |
| 可观测性 | 无法观察中间力矩 | 可以 log 每步力矩 |
| 扩展性 | 不可扩展 | 可以继承并修改 |
| 推荐场景 | 训练（吞吐量优先） | 调试（需要看力矩） |

### DC Motor 模型：加入力矩-速度约束

真实直流电机有一个关键物理约束：**力矩和转速不能同时达到最大值。** 当关节高速旋转时，反电动势（back-EMF）减小了可用力矩。DC Motor 模型用一个线性力矩-速度曲线近似这个关系：

```text
力矩 τ
^
|  τ_max ────────────┐
|                     \
|                      \
|                       \
|                        \
|─────────────────────────→ 速度 q̇
0                        q̇_max
```

$$\tau_{\max}(\dot{q}) = \tau_{\text{stall}} \cdot \left(1 - \frac{|\dot{q}|}{q̇_{\max}}\right)$$

在 Isaac Lab 中，DCMotorCfg 实现了这个模型：

```python
# Isaac Lab 中的 DC Motor actuator
from isaaclab.actuators import DCMotorCfg

legs_dc = DCMotorCfg(
    joint_names_expr=[".*hip.*", ".*thigh.*", ".*calf.*"],
    stiffness=100.0,
    damping=4.0,
    saturation_effort=33.5,  # 堵转力矩 τ_stall (N·m)
    effort_limit=33.5,       # 力矩上限（与 saturation_effort 通常相同）
    velocity_limit=21.0,     # 空载最大速度 q̇_max (rad/s)
)
```

DC Motor 的力矩计算：

```python
# DC Motor 的力矩计算逻辑（Isaac Lab 内部实现简化版）
def dc_motor_torque(q_star, q, q_dot, kp, kd,
                     tau_stall, q_dot_max):
    """
    DC Motor 模型的力矩计算。

    Step 1: 计算 PD 命令力矩
    Step 2: 根据速度限制可用力矩
    Step 3: clip 到可用范围
    """
    # Step 1: PD 命令力矩
    tau_cmd = kp * (q_star - q) - kd * q_dot

    # Step 2: 速度限制的可用力矩
    speed_ratio = torch.abs(q_dot) / q_dot_max
    tau_available = tau_stall * (1.0 - speed_ratio).clamp(min=0.0)

    # Step 3: clip
    tau_applied = tau_cmd.clamp(-tau_available, tau_available)
    return tau_applied
```

### 在 MuJoCo 中实现 DC Motor 约束

MuJoCo 没有内置的 DC Motor 类型，但可以通过 `forcerange` 和 `<general>` actuator 近似实现：

```xml
<!-- MuJoCo 中的 DC Motor 近似 -->
<!-- 方法 1：简单版（只有力矩上限，没有速度依赖的限制） -->
<actuator>
  <position name="hip" joint="hip_joint" kp="100"
            forcelimited="true" forcerange="-33.5 33.5"/>
</actuator>

<!-- 方法 2：使用 <general> actuator + dyntype="filter" 模拟带宽限制 -->
<actuator>
  <general name="hip" joint="hip_joint"
           gaintype="fixed" gainprm="100"
           biastype="affine" biasprm="0 -100 -4"
           dyntype="filter" dynprm="0.02"
           ctrllimited="true" ctrlrange="-1.57 1.57"
           forcelimited="true" forcerange="-33.5 33.5"/>
</actuator>
<!-- dyntype="filter" + dynprm="0.02" = 一阶低通，时间常数 20ms
     这模拟了电机的带宽限制 -->
```

> **本质洞察：** MuJoCo 的 `<general>` actuator 是一个极其灵活的框架——通过组合 `gaintype`、`biastype` 和 `dyntype`，你可以实现几乎任何线性 actuator 模型。`gaintype="fixed" gainprm="100"` 设置 kp=100。`biastype="affine" biasprm="0 -100 -4"` 设置 bias = 0 + (-100)*q + (-4)*q̇，即 kp=100, kd=4 的 PD 控制。`dyntype="filter" dynprm="0.02"` 加上 20ms 一阶低通滤波——模拟电机带宽。这三个参数的组合实现了一个带宽限制的 PD 控制器。

### 从 Level 0 到 Level 1 的升级决策

```text
问自己：
├── 我的策略是否需要极端力矩？（跳跃、翻转）
│   ├── 是 → 需要力矩饱和模型 → Level 1
│   └── 否 → Level 0 可能足够
│
├── 真机上是否观察到高速运动时力矩不足？
│   ├── 是 → 需要力矩-速度约束 → Level 1 (DC Motor)
│   └── 否 → Level 0
│
└── DR 是否已经覆盖了 actuator 差异？
    ├── 是 → 不需要更高精度模型
    └── 否 → 考虑升级到 Level 1 或 Level 2
```

### 完整对比实验：Ideal PD vs DC Motor vs 带宽限制

以下代码在 MuJoCo 中同时测试三种 actuator 模型对阶跃输入的响应：

```python
# actuator_comparison.py — 三种 actuator 模型的阶跃响应对比
import mujoco
import numpy as np

IDEAL_PD_XML = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body>
      <joint name="j" type="hinge" axis="0 0 1"/>
      <geom type="cylinder" size="0.05 0.2" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="act" joint="j" kp="100" kv="4"
              forcelimited="true" forcerange="-33.5 33.5"/>
  </actuator>
</mujoco>
"""

BANDWIDTH_LIMITED_XML = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body>
      <joint name="j" type="hinge" axis="0 0 1"/>
      <geom type="cylinder" size="0.05 0.2" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <general name="act" joint="j"
             gaintype="fixed" gainprm="100"
             biastype="affine" biasprm="0 -100 -4"
             dyntype="filter" dynprm="0.02"
             ctrllimited="true" ctrlrange="-1.57 1.57"
             forcelimited="true" forcerange="-33.5 33.5"/>
  </actuator>
</mujoco>
"""

def run_step_response(xml_string, step_target=1.0, num_steps=500):
    """运行阶跃响应实验。"""
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    positions = []
    torques = []

    for step in range(num_steps):
        # 阶跃输入：前 50 步 = 0，之后 = step_target
        data.ctrl[0] = step_target if step >= 50 else 0.0
        mujoco.mj_step(model, data)
        positions.append(data.qpos[0])
        torques.append(data.actuator_force[0])

    return np.array(positions), np.array(torques)


def compare_actuators():
    """对比三种 actuator 模型的阶跃响应。"""
    pos_ideal, tau_ideal = run_step_response(IDEAL_PD_XML)
    pos_bw, tau_bw = run_step_response(BANDWIDTH_LIMITED_XML)

    # 分析
    dt = 0.002
    t = np.arange(len(pos_ideal)) * dt

    print("=== 阶跃响应对比 ===")
    print(f"{'指标':<20} {'Ideal PD':<15} {'带宽限制':<15}")
    print(f"{'-'*50}")

    # 上升时间 (10% → 90%)
    for name, pos in [("Ideal PD", pos_ideal), ("带宽限制", pos_bw)]:
        idx_10 = np.argmax(pos > 0.1)
        idx_90 = np.argmax(pos > 0.9)
        rise_time = (idx_90 - idx_10) * dt * 1000
        overshoot = (pos.max() - 1.0) * 100
        settling_idx = len(pos) - np.argmax(
            np.abs(pos[::-1] - 1.0) > 0.02
        )
        settling_time = settling_idx * dt * 1000
        print(f"  {name}: 上升时间={rise_time:.1f}ms, "
              f"超调={overshoot:.1f}%, "
              f"安定时间={settling_time:.1f}ms")

    # 力矩对比
    print(f"\n最大力矩: Ideal={tau_ideal.max():.1f} N·m, "
          f"带宽限制={tau_bw.max():.1f} N·m")
    print(f"（带宽限制的力矩上升更平缓，更接近真机）")

compare_actuators()
```

预期输出：

```text
=== 阶跃响应对比 ===
指标                Ideal PD        带宽限制
--------------------------------------------------
  Ideal PD: 上升时间=18.0ms, 超调=8.5%, 安定时间=120ms
  带宽限制: 上升时间=42.0ms, 超调=3.2%, 安定时间=85ms

最大力矩: Ideal=33.5 N·m, 带宽限制=28.2 N·m
（带宽限制的力矩上升更平缓，更接近真机）
```

**关键观察**：带宽限制的 actuator 上升时间更长（42ms vs 18ms），但超调更小、安定更快。这更接近真实电机的行为——真机的力矩不会瞬间跳变。

### MuJoCo `<general>` Actuator 深度解析

MuJoCo 的 `<general>` actuator 是构建自定义 actuator 模型的瑞士军刀。理解它的三个组件（gain, bias, dynamics）是掌握 MuJoCo actuator 系统的关键。

```text
<general> actuator 的力矩计算管线：

  ctrl (策略输出)
    ↓
  [dynamics] → act (actuator 内部状态)
    可选：filter (一阶低通), integrator, filterexact, none
    ↓
  [gain] × act + [bias]·(1, q, q̇) → force (力矩)
    gain: fixed, affine, muscle
    bias: none, affine, muscle
```

各 `dyntype` 的含义：

| dyntype | 公式 | 物理意义 | 用途 |
|---------|------|---------|------|
| `none` | act = ctrl | 无动态 | Ideal PD（默认） |
| `filter` | τ·ȧ + a = ctrl | 一阶低通滤波 | 模拟电机带宽 |
| `integrator` | ȧ = ctrl | 积分器 | 速度控制 → 位置 |
| `filterexact` | 精确一阶滤波 | 同 filter 但更精确 | 数值敏感场景 |

各 `gaintype` 和 `biastype` 的含义：

```python
# gaintype + biastype 的组合实现不同的控制器
"""
gaintype="fixed", gainprm=[kp]
  → gain = kp

biastype="affine", biasprm=[b0, b1, b2]
  → bias = b0 + b1*q + b2*q̇

组合后的力矩：
  τ = kp * act + b0 + b1*q + b2*q̇

要实现 PD 控制器 τ = kp*(q* - q) - kd*q̇：
  act = q* (通过 ctrl 输入)
  gain = kp  → gaintype="fixed", gainprm=[100]
  bias = 0 + (-kp)*q + (-kd)*q̇
       → biastype="affine", biasprm=[0, -100, -4]

验证：τ = kp*q* + 0 - kp*q - kd*q̇
       = kp*(q* - q) - kd*q̇  ✅
"""
```

> **本质洞察：** MuJoCo 的 `<general>` actuator 比 Isaac Lab 的 actuator 层级更灵活——通过组合 gaintype、biastype 和 dyntype，你可以表达任何**线性时不变**的 actuator 模型而不需要写 Python 代码。对于非线性模型（actuator network），MuJoCo 提供 `mjcb_control` 回调——但这牺牲了 GPU 并行性（callback 在 CPU 上执行）。Isaac Lab 的 ActuatorNetMLP/LSTM 则在 GPU 上执行——更适合大规模训练。

### 双框架 actuator 配置速查表

| 需求 | MuJoCo (MJCF) | Isaac Lab |
|------|---------------|-----------|
| Ideal PD | `<position kp kv>` | `ImplicitActuatorCfg(stiffness, damping)` |
| PD + 力矩限制 | `<position forcerange>` | `ImplicitActuatorCfg(effort_limit)` |
| PD + 带宽限制 | `<general dyntype="filter">` | `DelayedPDActuatorCfg(min_delay, max_delay)` |
| DC Motor | `<general> + 自定义 clip` | `DCMotorCfg(saturation_effort)` |
| Actuator Network | `mjcb_control` callback | `ActuatorNetMLPCfg(network_file)` |
| 摩擦 | `<joint frictionloss>` | `ActuatorBaseCfg(friction)` |
| armature | `<joint armature>` | `ActuatorBaseCfg(armature)` |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Isaac Lab 的 ImplicitActuator 和 IdealPDActuator 在 DR 下行为不同。** ImplicitActuator 的 DR 通过 PhysX 内部参数修改实现，IdealPDActuator 的 DR 在 Python 中修改。两种方式可能有微小的数值差异。如果你在对比实验中混用这两种，需要注意。

⚠️ **编程陷阱：MuJoCo 的 forcerange 在 position actuator 上限制的是"actuator 输出力矩"，而非"关节总力矩"。** 关节总力矩还包括重力补偿、约束力等——forcerange 只限制 actuator 贡献的部分。

💡 **概念误区：DC Motor 的 saturation_effort 就是电机的最大力矩。** saturation_effort 是**堵转力矩**（转速为零时的最大力矩）。当关节高速旋转时，实际可用力矩小于 saturation_effort。如果你只关心静止或低速情况，saturation_effort ≈ effort_limit 就够了。

### 练习

1. **[计算题]** 一个关节的 kp=100 N·m/rad, kd=4 N·m·s/rad, 目标角度 q*=0.5 rad，当前角度 q=0.3 rad，当前角速度 q̇=2 rad/s。Ideal PD 的输出力矩是多少？如果 DC Motor 的 τ_stall=33.5 N·m, q̇_max=21 rad/s，最终施加的力矩是多少？
2. **[编码题]** 在 MuJoCo 中创建一个单关节机器人，分别配置 Ideal PD 和带 `dyntype="filter"` 的 `<general>` actuator。给两者相同的阶跃输入（q*=1.0），比较响应曲线。滤波时间常数 dynprm 对响应有什么影响？
3. **[跨章综合题]** 结合 Ch11（actuator kp 调优）：如果 DC Motor 模型显示关节在高速运动时力矩不足，除了增大 kp，还可以调整哪些 actuator 参数？

---

### 工程实战：从电机数据手册配置 actuator 参数

以下是从 Unitree Go1 的公开电机数据手册（A1 电机）配置 actuator 参数的完整流程：

```python
# configure_from_datasheet.py — 从电机数据手册配置 actuator
"""
Unitree A1 电机数据手册参数（Go1 使用的电机）：

额定扭矩 (nominal torque):    6.0 N·m
峰值扭矩 (peak torque):       23.7 N·m
空载转速 (no-load speed):      21 rad/s (约 200 RPM)
减速比 (gear ratio):           6.33:1
力矩常数 (torque constant):    0.0525 N·m/A
额定电流 (nominal current):    4.8 A
峰值电流 (peak current):       19.0 A
堵转扭矩 = 力矩常数 × 峰值电流 × 减速比
         = 0.0525 × 19.0 × 6.33 ≈ 6.3 N·m（输出端）
注意：不同来源的数据可能不一致，以实测为准
"""

def datasheet_to_mjcf(motor_params):
    """从电机数据手册生成 MJCF actuator 配置。"""

    # 力矩限制 = 峰值扭矩
    tau_max = motor_params["peak_torque"]

    # 空载转速（考虑减速比后的输出端速度）
    vel_max = motor_params["no_load_speed"]

    # PD 增益的初始估计
    # kp 的选择：让关节在 0.1 rad 位置误差下产生 ~60% 峰值力矩
    kp_initial = 0.6 * tau_max / 0.1  # ≈ 142 N·m/rad

    # kd 的选择：临界阻尼条件 kd = 2*sqrt(kp * J)
    # J ≈ 0.01 kg·m² 对于四足机器人的腿部关节
    J_estimated = 0.01
    kd_initial = 2 * (kp_initial * J_estimated) ** 0.5  # ≈ 2.4

    # 带宽估计：一般电机的带宽 ≈ 20-50 Hz
    bandwidth_hz = motor_params.get("bandwidth_hz", 30)
    filter_time_constant = 1.0 / (2 * 3.14159 * bandwidth_hz)

    print(f"=== 从数据手册生成的 actuator 配置 ===")
    print(f"kp (初始估计): {kp_initial:.1f} N·m/rad")
    print(f"kd (临界阻尼): {kd_initial:.2f} N·m·s/rad")
    print(f"forcerange: [-{tau_max:.1f}, {tau_max:.1f}] N·m")
    print(f"velocity_limit: {vel_max:.1f} rad/s")
    print(f"filter 时间常数: {filter_time_constant*1000:.1f} ms")
    print()

    # 生成 MJCF
    mjcf = f"""
<!-- 从电机数据手册生成的 MJCF actuator -->
<default class="go1_motor">
  <general gaintype="fixed" gainprm="{kp_initial:.0f}"
           biastype="affine" biasprm="0 -{kp_initial:.0f} -{kd_initial:.1f}"
           dyntype="filter" dynprm="{filter_time_constant:.4f}"
           forcelimited="true" forcerange="-{tau_max:.1f} {tau_max:.1f}"
           ctrllimited="true" ctrlrange="-3.14 3.14"/>
</default>
"""
    print(mjcf)

    # 生成 Isaac Lab 配置
    isaac_cfg = f"""
# Isaac Lab 配置
DCMotorCfg(
    joint_names_expr=[".*"],
    stiffness={kp_initial:.0f},
    damping={kd_initial:.1f},
    saturation_effort={tau_max:.1f},
    velocity_limit={vel_max:.1f},
    effort_limit={tau_max:.1f},
)
"""
    print(isaac_cfg)
    return kp_initial, kd_initial, tau_max

# 使用
go1_motor = {
    "peak_torque": 23.7,
    "no_load_speed": 21.0,
    "bandwidth_hz": 30,
}
datasheet_to_mjcf(go1_motor)
```

> **工程提示：** 数据手册上的参数是"标称值"——实际的 kp/kd/τ_max 需要通过系统辨识（12.5 节）验证。数据手册的峰值扭矩是短时峰值（通常 <1 秒），持续可用扭矩（连续输出不过热的扭矩）通常只有峰值的 30-50%。RL 训练中的 forcerange 应该设为持续扭矩而非峰值扭矩——因为训练中关节力矩可能持续处于限制边界。

---

上一节讲完了 Level 0-1 的工程实现。下一节进入 Level 2——用神经网络直接拟合真机的 actuator 响应。

## 12.3 Level 2：Actuator Network ⭐⭐⭐

> **这一节解决什么问题**：当 DC Motor 模型的精度不够时（真机力矩响应有复杂的非线性特征），用神经网络直接拟合真机的 actuator 响应。本节从数据收集到 MLP 训练到仿真器集成完整讲解。

### 动机：为什么需要神经网络

DC Motor 模型假设力矩-速度关系是线性的。但真实电机有复杂的非线性特征：

- **Coulomb 摩擦**：有一个恒定的摩擦力矩，不依赖于速度（但有方向）
- **粘性摩擦 + 库仑摩擦混合**：低速时以库仑为主，高速时以粘性为主
- **齿轮间隙（backlash）**：方向反转时有一小段"死区"
- **温度依赖**：电机温度升高时效率下降
- **位置依赖的摩擦**：某些关节角度处摩擦更大（齿轮啮合特性）

这些非线性很难用解析模型精确描述——但一个小型 MLP 可以从数据中学到这些模式。

### Hwangbo 2019 的原始 Actuator Network

Hwangbo et al.（Science Robotics, 2019）在 ANYmal 四足机器人上首创了 actuator network。核心思路：

```text
输入: [q*(t), q*(t-1), ..., q*(t-H), q(t), q(t-1), ..., q(t-H), q̇(t)]
      ↑ 过去 H 步的命令历史    ↑ 过去 H 步的位置历史    ↑ 当前速度
  ↓
MLP: 2 层, [128, 128], ELU 激活
  ↓
输出: τ_predicted  (预测的实际力矩)
```

**为什么需要历史信息？** 因为真实 actuator 有延迟和动态响应——当前力矩不仅取决于当前命令，还取决于"之前的命令和状态"。这就像一个有惯性的系统——你踩油门后车速不会瞬间改变，而是逐渐加速。

### 数据收集：从真机获取 actuator 数据

```python
# collect_actuator_data.py — 在真机上收集 actuator 训练数据
"""
数据收集协议：
1. 让机器人站在地面上（不运动）
2. 对每个关节依次施加随机正弦命令
3. 记录：命令角度、实际角度、实际角速度、实际力矩
4. 总时长：5-10 分钟

注意：需要关节力矩传感器或电流测量来获取实际力矩。
如果没有力矩传感器，可以用电机电流 × 力矩常数近似。
"""
import numpy as np

def collect_single_joint_data(robot, joint_idx, duration_sec=30.0,
                               cmd_freq_range=(0.5, 5.0),
                               cmd_amp_range=(0.2, 0.8),
                               control_freq=200):
    """
    对单个关节进行随机正弦扫频，收集数据。

    Args:
        robot: 机器人接口（提供 read/write 方法）
        joint_idx: 关节索引
        duration_sec: 收集时长（秒）
        cmd_freq_range: 命令频率范围 (Hz)
        cmd_amp_range: 命令幅度范围 (rad)
        control_freq: 控制频率 (Hz)
    """
    dt = 1.0 / control_freq
    num_steps = int(duration_sec * control_freq)

    data = {
        "cmd_pos": np.zeros(num_steps),      # q*
        "actual_pos": np.zeros(num_steps),   # q
        "actual_vel": np.zeros(num_steps),   # q̇
        "actual_torque": np.zeros(num_steps), # τ (测量值)
        "timestamp": np.zeros(num_steps),
    }

    t = 0.0
    for step in range(num_steps):
        # 随机正弦命令：周期性切换频率和幅度
        if step % (control_freq * 3) == 0:  # 每 3 秒换一次
            freq = np.random.uniform(*cmd_freq_range)
            amp = np.random.uniform(*cmd_amp_range)

        cmd = amp * np.sin(2 * np.pi * freq * t)

        # 发送命令到机器人
        robot.set_joint_position(joint_idx, cmd)

        # 读取反馈
        data["cmd_pos"][step] = cmd
        data["actual_pos"][step] = robot.get_joint_position(joint_idx)
        data["actual_vel"][step] = robot.get_joint_velocity(joint_idx)
        data["actual_torque"][step] = robot.get_joint_torque(joint_idx)
        data["timestamp"][step] = t

        t += dt

    return data
```

### MLP 训练

```python
# train_actuator_net.py — 训练 actuator network
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import numpy as np

class ActuatorNet(nn.Module):
    """
    Actuator Network: 从关节状态历史预测实际力矩。
    架构：2 层 MLP, [128, 128], ELU 激活。
    （与 Hwangbo 2019 / UAN 2025 一致）
    """
    def __init__(self, history_len=6):
        super().__init__()
        # 输入：H 步命令历史 + H 步位置历史 + 当前速度
        input_dim = history_len * 2 + 1  # q* history + q history + q̇
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ELU(),
            nn.Linear(128, 128),
            nn.ELU(),
            nn.Linear(128, 1),  # 输出：预测力矩
        )

    def forward(self, x):
        return self.net(x)


def prepare_training_data(raw_data, history_len=6):
    """
    从原始时间序列构造训练样本。
    每个样本包含：过去 H 步的 (q*, q) + 当前 q̇ → 当前 τ。
    """
    cmd = raw_data["cmd_pos"]
    pos = raw_data["actual_pos"]
    vel = raw_data["actual_vel"]
    torque = raw_data["actual_torque"]

    N = len(cmd) - history_len
    X = np.zeros((N, history_len * 2 + 1))
    Y = np.zeros((N, 1))

    for i in range(N):
        idx = i + history_len
        # 命令历史（从最近到最远）
        X[i, :history_len] = cmd[idx:idx-history_len:-1]
        # 位置历史
        X[i, history_len:2*history_len] = pos[idx:idx-history_len:-1]
        # 当前速度
        X[i, -1] = vel[idx]
        # 标签：实际力矩
        Y[i, 0] = torque[idx]

    return torch.FloatTensor(X), torch.FloatTensor(Y)


def train_actuator_network(raw_data, history_len=6, epochs=200,
                            batch_size=256, lr=1e-3):
    """训练 actuator network。"""
    X, Y = prepare_training_data(raw_data, history_len)

    # 分割训练集/验证集
    n = X.shape[0]
    n_val = int(n * 0.1)
    perm = torch.randperm(n)
    X_train, Y_train = X[perm[n_val:]], Y[perm[n_val:]]
    X_val, Y_val = X[perm[:n_val]], Y[perm[:n_val]]

    # 归一化
    X_mean, X_std = X_train.mean(0), X_train.std(0) + 1e-8
    Y_mean, Y_std = Y_train.mean(0), Y_train.std(0) + 1e-8

    X_train_norm = (X_train - X_mean) / X_std
    X_val_norm = (X_val - X_mean) / X_std
    Y_train_norm = (Y_train - Y_mean) / Y_std

    # 训练
    model = ActuatorNet(history_len)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loader = DataLoader(
        TensorDataset(X_train_norm, Y_train_norm),
        batch_size=batch_size, shuffle=True,
    )

    best_val_loss = float("inf")
    for epoch in range(epochs):
        model.train()
        for xb, yb in loader:
            pred = model(xb)
            loss = nn.functional.mse_loss(pred, yb)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        # 验证
        model.eval()
        with torch.no_grad():
            val_pred = model((X_val - X_mean) / X_std)
            val_pred_real = val_pred * Y_std + Y_mean
            val_mse = nn.functional.mse_loss(val_pred_real, Y_val)

        if val_mse < best_val_loss:
            best_val_loss = val_mse
            torch.save({
                "model": model.state_dict(),
                "X_mean": X_mean, "X_std": X_std,
                "Y_mean": Y_mean, "Y_std": Y_std,
            }, "actuator_net.pt")

        if epoch % 20 == 0:
            print(f"Epoch {epoch}: val_mse={val_mse.item():.6f} N²·m²")

    print(f"训练完成, 最佳 val_mse = {best_val_loss.item():.6f}")
    return model
```

### 在 Isaac Lab 中集成 Actuator Network

Isaac Lab 提供了 `ActuatorNetMLPCfg` 和 `ActuatorNetLSTMCfg` 来直接使用训练好的 actuator network：

```python
# Isaac Lab 中配置 actuator network
from isaaclab.actuators import ActuatorNetMLPCfg

legs_net = ActuatorNetMLPCfg(
    joint_names_expr=[".*hip.*", ".*thigh.*", ".*calf.*"],
    # 指向训练好的 .pt 文件
    network_file="/path/to/actuator_net.pt",
    # DC Motor 基础参数（actuator net 在此基础上修正）
    saturation_effort=33.5,
    velocity_limit=21.0,
    # stiffness 和 damping 设为 None（由网络输出力矩）
    stiffness=None,
    damping=None,
)
```

### 在 MuJoCo/mjlab 中集成 Actuator Network

mjlab 有两种集成方式：

```python
# 方式 1：在训练循环中手动替换力矩
# （简单但侵入性强）
class ActuatorNetWrapper:
    """包装 actuator network，在 env.step 中替换力矩。"""

    def __init__(self, model_path, num_joints, history_len=6,
                 device="cuda"):
        ckpt = torch.load(model_path, map_location=device)
        self.net = ActuatorNet(history_len)
        self.net.load_state_dict(ckpt["model"])
        self.net.to(device).eval()
        self.X_mean = ckpt["X_mean"].to(device)
        self.X_std = ckpt["X_std"].to(device)
        self.Y_mean = ckpt["Y_mean"].to(device)
        self.Y_std = ckpt["Y_std"].to(device)

        self.num_joints = num_joints
        self.history_len = history_len
        # 历史缓冲区：(history_len, num_envs, num_joints)
        self.cmd_history = None
        self.pos_history = None

    def reset(self, num_envs):
        """训练开始或 episode reset 时调用。"""
        device = self.X_mean.device
        self.cmd_history = torch.zeros(
            self.history_len, num_envs, self.num_joints,
            device=device
        )
        self.pos_history = torch.zeros(
            self.history_len, num_envs, self.num_joints,
            device=device
        )

    def update_history(self, cmd, pos):
        """每步调用，更新历史缓冲区。"""
        # 滚动历史：最新的在前
        self.cmd_history = torch.cat([
            cmd.unsqueeze(0),
            self.cmd_history[:-1]
        ], dim=0)
        self.pos_history = torch.cat([
            pos.unsqueeze(0),
            self.pos_history[:-1]
        ], dim=0)

    def compute_torque(self, cmd, pos, vel):
        """
        用 actuator network 计算所有关节的力矩。

        Args:
            cmd: (num_envs, num_joints) 当前命令
            pos: (num_envs, num_joints) 当前位置
            vel: (num_envs, num_joints) 当前速度
        Returns:
            tau: (num_envs, num_joints) 预测力矩
        """
        self.update_history(cmd, pos)

        # 对每个关节独立计算力矩
        # 输入格式：(num_envs, H*2 + 1) per joint
        all_tau = []
        for j in range(self.num_joints):
            x = torch.cat([
                self.cmd_history[:, :, j].T,   # (num_envs, H)
                self.pos_history[:, :, j].T,   # (num_envs, H)
                vel[:, j:j+1],                 # (num_envs, 1)
            ], dim=-1)

            # 归一化 + 前向 + 反归一化
            x_norm = (x - self.X_mean) / self.X_std
            with torch.no_grad():
                tau_norm = self.net(x_norm)
            tau_j = tau_norm * self.Y_std + self.Y_mean
            all_tau.append(tau_j)

        return torch.cat(all_tau, dim=-1)  # (num_envs, num_joints)
```

**在 mjlab 训练循环中使用 ActuatorNetWrapper：**

```python
# mjlab 训练循环中集成 actuator network
class TrainingWithActuatorNet:
    """
    修改 mjlab 的标准训练循环，在 env.step 前替换力矩。
    """
    def __init__(self, env, agent, actuator_net_path):
        self.env = env
        self.agent = agent
        self.act_net = ActuatorNetWrapper(
            actuator_net_path,
            num_joints=env.num_joints,
        )

    def train_step(self):
        # 1. 策略输出 action（目标关节角度）
        obs = self.env.get_observations()
        action = self.agent.act(obs)

        # 2. 用 actuator network 计算力矩
        current_pos = self.env.robot.joint_pos
        current_vel = self.env.robot.joint_vel
        tau = self.act_net.compute_torque(action, current_pos, current_vel)

        # 3. 直接写入力矩（绕过默认 PD 计算）
        self.env.robot.set_joint_torque(tau)

        # 4. 仿真前进一步
        self.env.sim_step()

        # 5. 计算 reward 等
        reward = self.env.compute_reward()
        return obs, action, reward
```

```python
# 方式 2：使用 MuJoCo 的 mjcb_control 回调
# （更优雅，不需要修改训练循环）
import mujoco

def setup_actuator_net_callback(net_wrapper):
    """注册 MuJoCo control callback。"""
    def callback(model, data):
        """MuJoCo control callback：用 actuator network 替代默认 PD。"""
        cmd = torch.from_numpy(data.ctrl.copy()).float()
        pos = torch.from_numpy(data.qpos[7:].copy()).float()  # 去掉 root
        vel = torch.from_numpy(data.qvel[6:].copy()).float()

        tau = net_wrapper.compute_torque(
            cmd.unsqueeze(0), pos.unsqueeze(0), vel.unsqueeze(0)
        ).squeeze(0)

        # 直接写入 qfrc_applied
        data.qfrc_applied[6:] = tau.numpy()

    mujoco.set_mjcb_control(callback)
    print("已注册 actuator network control callback")
```

### Actuator Network 的评估

训练完成后，需要评估 actuator network 的力矩预测精度：

```python
# evaluate_actuator_net.py — 评估 actuator network
def evaluate_actuator_net(net, test_data, device="cuda"):
    """
    评估 actuator network 的预测精度。

    指标：
    1. MSE (N²·m²)
    2. RMSE (N·m)
    3. 相对误差 (%)
    4. 最大误差 (N·m)
    5. R² 决定系数
    """
    X_test, Y_test = prepare_training_data(test_data)
    X_test = X_test.to(device)
    Y_test = Y_test.to(device)

    ckpt = torch.load("actuator_net.pt", map_location=device)
    X_mean = ckpt["X_mean"].to(device)
    X_std = ckpt["X_std"].to(device)
    Y_mean = ckpt["Y_mean"].to(device)
    Y_std = ckpt["Y_std"].to(device)

    net.eval()
    with torch.no_grad():
        X_norm = (X_test - X_mean) / X_std
        Y_pred_norm = net(X_norm)
        Y_pred = Y_pred_norm * Y_std + Y_mean

    # 计算指标
    errors = Y_pred - Y_test
    mse = (errors ** 2).mean().item()
    rmse = mse ** 0.5
    rel_error = (errors.abs() / (Y_test.abs() + 1e-6)).mean().item() * 100
    max_error = errors.abs().max().item()

    ss_res = (errors ** 2).sum().item()
    ss_tot = ((Y_test - Y_test.mean()) ** 2).sum().item()
    r_squared = 1.0 - ss_res / ss_tot

    print(f"=== Actuator Network 评估 ===")
    print(f"  MSE:        {mse:.6f} N²·m²")
    print(f"  RMSE:       {rmse:.4f} N·m")
    print(f"  相对误差:   {rel_error:.1f}%")
    print(f"  最大误差:   {max_error:.3f} N·m")
    print(f"  R²:         {r_squared:.4f}")

    # 判断质量
    if rmse < 0.5 and r_squared > 0.95:
        print(f"  ✅ 精度优秀，可以用于 RL 训练")
    elif rmse < 1.0 and r_squared > 0.85:
        print(f"  ⚠️ 精度一般，建议增加训练数据或调整架构")
    else:
        print(f"  ❌ 精度不足，需要更多数据或检查数据质量")

    return {"mse": mse, "rmse": rmse, "rel_error": rel_error,
            "max_error": max_error, "r_squared": r_squared}
```

### UAN：无监督 Actuator Network（2025 前沿）

Fey, Margolis, Peticco & Agrawal（MIT CSAIL, arXiv 2502.10894, 2025）提出了 **Unsupervised Actuator Network (UAN)**——不需要力矩传感器，只用 5 分钟的真机 rollout 数据就能训练 actuator network。

UAN 的关键思路：不直接监督力矩预测（因为没有力矩传感器），而是监督**状态转移**——让仿真器在 actuator network 修正后的力矩下产生的下一状态接近真机观测的下一状态。

```python
# UAN 的核心损失函数（概念化）
def uan_loss(actuator_net, sim_env, real_trajectory):
    """
    UAN 损失：最小化仿真状态与真机状态的差异。
    不需要力矩标签——只需要 (s, a, s'_real) 三元组。
    """
    states = real_trajectory["states"]
    actions = real_trajectory["actions"]
    next_states_real = real_trajectory["next_states"]

    # 用 actuator network 计算力矩
    tau_predicted = actuator_net(actions, states[:, :num_joints],
                                 states[:, num_joints:])

    # 在仿真器中用预测力矩前进一步
    next_states_sim = sim_env.step_with_torque(states, tau_predicted)

    # 损失 = 仿真状态与真机状态的差异
    loss = F.mse_loss(next_states_sim, next_states_real)
    return loss
```

UAN 和 ASAP Delta Action Model 的关系：两者都只需要 (s, a, s'_real) 数据，不需要力矩传感器。区别是 UAN 修正的是力矩（actuator 层面），ASAP 修正的是 action（策略层面）。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：actuator network 的训练数据和部署数据的分布必须匹配。** 如果你在低频正弦（0.5-5 Hz）上收集数据，网络在高频命令（>10 Hz）上的预测可能不准确。数据收集时的频率范围应该覆盖 RL 策略实际产生的命令频率。

⚠️ **编程陷阱：历史长度 H 的选择影响延迟补偿。** H 太小（如 H=2）无法捕捉电机的动态延迟。H 太大（如 H=20）增加了 observation 维度但收益递减。推荐 H=4-8，对应 20-40ms 历史（在 200 Hz 控制频率下）。

🧠 **思维陷阱：认为 actuator network 可以替代 DR。** Actuator network 提高了 nominal model 的精度，但真机的 actuator 特性仍然会随温度、磨损、负载变化。仍然需要 DR 来覆盖这些变化——只是 DR 的范围可以缩小。

### 练习

1. **[编码题]** 实现一个简化版的 actuator network 训练流程：用 MuJoCo 的 Ideal PD 作为"真机"，收集数据训练 MLP，然后把 MLP 输出的力矩与 Ideal PD 的力矩对比。两者的差异在多大量级？
2. **[设计题]** 如果你的机器人没有力矩传感器（很多消费级机器人如 Go1 没有），如何间接获取 actuator 数据？提示：电机电流 × 力矩常数 = 力矩近似。

---

## 12.4 Level 3：ASAP Delta Action Model ⭐⭐

> **这一节解决什么问题**：当 actuator network 仍然不够精确（或获取力矩数据困难）时，ASAP 提出了一种更优雅的方法：不建模 actuator 物理，而是直接学习 action 空间中的修正。

### 动机：Actuator Network 的局限性

Actuator Network 建模的是 `(q*, q, q̇) → τ` 的映射。但 sim-to-real gap 不仅来自 actuator——还包括通信延迟、控制器内部非线性、离散化效应、甚至仿真器的积分误差。这些 "非 actuator" 因素无法被 actuator network 捕获。

ASAP（He et al., RSS 2025）的关键洞察：**与其建模 gap 的物理原因，不如直接在 action 空间修正 gap 的效果。**

### Delta Action Model 的核心思路

```text
标准管线：
  policy(s) → a → simulator → s'_sim   ≠ s'_real

ASAP 管线：
  policy(s) → a → a + Δa → simulator → s'_sim ≈ s'_real
                    ↑
              Δa = MLP_delta(s, a)  ← 从真机数据学习
```

Delta action model 不预测力矩，而是预测**action 修正量**。这个修正量让仿真器在修正后的 action 下产生的下一状态接近真机的下一状态。

### 与 Actuator Network 的本质区别

> **双重解读：** Actuator Network 和 Delta Action Model 可以从两个视角理解：

**视角 A（建模目标不同）：** Actuator Network 建模的是"从命令到力矩"的**前向模型**（forward model）——它告诉仿真器"真实电机在收到这个命令后会施加什么力矩"。Delta Action Model 建模的是"仿真和真机之间的差异"的**残差模型**（residual model）——它不关心差异的物理原因，只补偿差异的效果。

**视角 B（训练数据不同）：** Actuator Network 需要 `(q*, q, q̇, τ)` 数据——需要力矩传感器或电流测量。Delta Action Model 需要 `(s, a, s'_real)` 数据——只需要状态观测（IMU、关节编码器），不需要力矩传感器。这意味着 Delta Action Model 可以在没有力矩传感器的消费级机器人上使用。

### ASAP 的训练管线

```text
ASAP 完整管线（4 步）：

Step 1: 在仿真中预训练多个 motion tracking 策略
  policies = [train_tracker(motion_i) for motion_i in motions]

Step 2: 在真机上 rollout，收集 (s, a, s'_real) 数据
  for policy in policies:
    real_data += deploy_and_record(policy, robot)

Step 3: 训练 delta action model
  Δa_model = train_delta(sim_data, real_data)
  # 最小化 ||step(s, a + Δa(s,a)) - s'_real||

Step 4: 冻结 Δa_model，在修正后的仿真器中微调策略
  for policy in policies:
    finetune(policy, env_with_delta_action=Δa_model)

Step 5: 部署（不需要 Δa_model——策略已经适应了修正后的仿真）
  deploy(finetuned_policy, robot)
```

**为什么部署时不需要 Δa_model？** 因为策略在 Step 4 中已经在"修正后的仿真器"中微调——它学会了在这个更接近真机的仿真器中行动。部署时策略直接输出 action 给真机，不需要额外的修正。这是 delta action model 相比 actuator network 的一个工程优势——部署时不增加任何计算开销。

### 简化版 Delta Action Model 实现

```python
# delta_action_model.py — ASAP 风格的 delta action model（简化版）
import torch
import torch.nn as nn

class DeltaActionModel(nn.Module):
    """
    Delta Action Model：从 (state, action) 预测 action 修正量。
    Δa = MLP(s, a)

    输出的 Δa 应该在 action space 内——
    通过 tanh 和缩放确保修正量不会太大。
    """
    def __init__(self, state_dim, action_dim, max_delta=0.1):
        super().__init__()
        self.max_delta = max_delta
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),  # 输出 ∈ [-1, 1]
        )

    def forward(self, state, action):
        x = torch.cat([state, action], dim=-1)
        delta = self.net(x) * self.max_delta  # 缩放到 [-max_delta, max_delta]
        return delta


def train_delta_model(sim_env, real_trajectories, model, cfg):
    """
    训练 delta action model。

    real_trajectories: List[Dict] — 真机轨迹
      每个 Dict 包含 {"states": (T, state_dim), "actions": (T, action_dim),
                       "next_states": (T, state_dim)}
    """
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.lr)

    for epoch in range(cfg.num_epochs):
        total_loss = 0
        for traj in real_trajectories:
            states = traj["states"].to(cfg.device)
            actions = traj["actions"].to(cfg.device)
            next_states_real = traj["next_states"].to(cfg.device)

            # 计算 delta action
            delta_a = model(states, actions)

            # 在仿真器中用修正后的 action 前进一步
            corrected_actions = actions + delta_a
            next_states_sim = sim_env.step_batch(states, corrected_actions)

            # 损失：仿真下一状态与真机下一状态的差异
            loss = nn.functional.mse_loss(next_states_sim, next_states_real)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            total_loss += loss.item()

        if epoch % 20 == 0:
            avg_loss = total_loss / len(real_trajectories)
            print(f"Epoch {epoch}: delta_model_loss = {avg_loss:.6f}")

    return model
```

### 在 mjlab/Isaac Lab 中集成 Delta Action Model

Delta Action Model 的集成比 Actuator Network 更简单——它修改的是策略输出的 action，不需要替换仿真器的力矩计算：

```python
# 在 mjlab 训练循环中集成 delta action model
class EnvWithDeltaAction:
    """
    包装标准 env，在 step 时自动应用 delta action 修正。
    """
    def __init__(self, base_env, delta_model):
        self.env = base_env
        self.delta_model = delta_model
        self.delta_model.eval()  # 冻结！不参与训练

    def step(self, action):
        """
        修改后的 step：
        1. 获取当前 state
        2. 计算 delta_a = MLP(state, action)
        3. 用 action + delta_a 执行仿真
        """
        state = self.env.get_observations()["policy"]

        with torch.no_grad():
            delta_a = self.delta_model(state, action)

        corrected_action = action + delta_a
        return self.env.step(corrected_action)

    # 代理其他方法
    def reset(self):
        return self.env.reset()

    def get_observations(self):
        return self.env.get_observations()


# 使用
delta_model = DeltaActionModel(state_dim=48, action_dim=12, max_delta=0.1)
delta_model.load_state_dict(torch.load("delta_action_model.pt"))
wrapped_env = EnvWithDeltaAction(base_env, delta_model)

# 在 wrapped_env 上微调策略
agent.train(wrapped_env, num_iterations=2000)
```

### Delta Action Model 的真机数据收集

ASAP 的 Step 2 需要在真机上部署预训练策略并收集轨迹。以下是数据收集的实践指南：

```python
# collect_real_data_for_delta.py — 为 delta action model 收集真机数据
"""
数据收集最佳实践：

1. 使用多个不同的策略/命令，增加数据多样性
   - 不同的速度命令：0.5, 1.0, 1.5, 2.0 m/s
   - 不同的转向命令：-0.5, 0, 0.5 rad/s
   - 不同的步态：trot, pace（如果策略支持）

2. 每条轨迹长度 ≥ 5 秒（确保包含完整的步态周期）

3. 总数据量：5-10 分钟通常足够
   - ASAP 使用的典型数据量
   - UAN 显式声明 "5 分钟 rollout"

4. 记录的数据：
   - state: IMU + 关节编码器 + 上一步 action
   - action: 策略输出的目标关节角度
   - next_state: 下一步的 state
   - timestamp: 用于对齐仿真步
"""

def collect_real_trajectories(robot, policies, commands, cfg):
    """在真机上收集多条轨迹。"""
    trajectories = []

    for policy_idx, policy in enumerate(policies):
        for cmd in commands:
            traj = {"states": [], "actions": [], "next_states": []}

            obs = robot.get_observation()
            for step in range(cfg.steps_per_trajectory):
                # 设置速度命令
                obs["velocity_command"] = cmd

                # 策略输出 action
                with torch.no_grad():
                    action = policy(obs)

                # 记录 state 和 action
                traj["states"].append(obs["state"].clone())
                traj["actions"].append(action.clone())

                # 执行 action
                robot.send_action(action)
                robot.wait_for_step()

                # 记录 next_state
                next_obs = robot.get_observation()
                traj["next_states"].append(next_obs["state"].clone())
                obs = next_obs

            # 转为 tensor
            for key in traj:
                traj[key] = torch.stack(traj[key])

            trajectories.append(traj)
            print(f"收集轨迹 {len(trajectories)}: "
                  f"policy={policy_idx}, cmd={cmd}, "
                  f"steps={cfg.steps_per_trajectory}")

    print(f"\n总共 {len(trajectories)} 条轨迹, "
          f"{sum(t['states'].shape[0] for t in trajectories)} 步")
    return trajectories
```

### Delta Action Model 的诊断和验证

```python
# validate_delta_model.py — 验证 delta action model 的效果
def validate_delta_model(delta_model, sim_env, real_trajectories):
    """
    验证 delta action model 是否有效缩小了 sim-real gap。

    指标：
    1. 未修正的 sim-real state 差异（baseline）
    2. 修正后的 sim-real state 差异
    3. 改善百分比
    """
    delta_model.eval()

    uncorrected_errors = []
    corrected_errors = []

    for traj in real_trajectories:
        states = traj["states"]
        actions = traj["actions"]
        next_states_real = traj["next_states"]

        for t in range(len(states)):
            s, a, s_real_next = states[t], actions[t], next_states_real[t]

            # 未修正：直接用原始 action 在仿真中前进
            s_sim_next = sim_env.step_single(s, a)
            uncorrected_errors.append(
                (s_sim_next - s_real_next).norm().item()
            )

            # 修正：用 delta action 修正后在仿真中前进
            with torch.no_grad():
                delta_a = delta_model(s.unsqueeze(0), a.unsqueeze(0))
            corrected_a = a + delta_a.squeeze(0)
            s_sim_corrected = sim_env.step_single(s, corrected_a)
            corrected_errors.append(
                (s_sim_corrected - s_real_next).norm().item()
            )

    uncorrected_mean = np.mean(uncorrected_errors)
    corrected_mean = np.mean(corrected_errors)
    improvement = (1 - corrected_mean / uncorrected_mean) * 100

    print(f"=== Delta Action Model 验证 ===")
    print(f"  未修正 state 误差: {uncorrected_mean:.4f}")
    print(f"  修正后 state 误差: {corrected_mean:.4f}")
    print(f"  改善: {improvement:.1f}%")

    if improvement > 30:
        print(f"  ✅ 显著改善，值得在训练中使用")
    elif improvement > 10:
        print(f"  ⚠️ 有改善但不大，考虑增加真机数据")
    else:
        print(f"  ❌ 改善很小，检查数据质量或 max_delta 设置")

    return improvement
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：`max_delta` 设得太大会导致策略微调不稳定。** 如果 Δa 的范围太大，修正后的 action 可能超出策略训练时见过的 action 范围——导致策略在这些"陌生"的 action 下行为不可预测。推荐从 max_delta=0.05 开始，逐步增大到不超过 0.2。

⚠️ **编程陷阱：ASAP 要求 Step 4 的微调在"冻结的 delta action model"下进行。** 如果 delta model 和策略同时训练，会出现两个网络互相"追逐"的不稳定现象。冻结 delta model 确保策略适应的是一个固定的仿真器。

### 练习

1. **[设计题]** ASAP 的 Step 2 需要在真机上收集数据。如果你只有 5 分钟的真机时间，应该如何设计 rollout 策略以最大化数据的信息量？提示：考虑多样性（不同速度命令）vs 覆盖度（足够长的连续轨迹）。
2. **[分析题]** Delta Action Model 的 max_delta=0.1 意味着 action 的修正幅度最大为 0.1 rad。如果 sim-to-real gap 的真实修正需要 0.3 rad，这个模型能工作吗？如何诊断"max_delta 不够大"的问题？

---

上两节讲完了 Level 2（Actuator Network）和 Level 3（Delta Action Model）的工程实现。但无论选择哪个层级，都需要真机的实验数据来确定参数或训练网络。下一节讲解三种基本的系统辨识实验——它们是所有 actuator 建模的"数据基础"。

## 12.5 系统辨识方法 ⭐⭐

> **这一节解决什么问题**：如何从真机的实验数据中获取 actuator 模型的参数——扫频、阶跃响应和摩擦测量的实战流程。

### 三种基本实验

**实验 1：扫频（Frequency Sweep / Chirp）**

```python
# frequency_sweep.py — 扫频实验
def frequency_sweep(robot, joint_idx, f_start=0.1, f_end=50.0,
                     amplitude=0.3, duration=60.0, fs=200):
    """
    对单个关节进行线性扫频（chirp signal）。

    目的：拟合传递函数 G(s) = q(s) / q*(s)
    从传递函数可以读出：
    - 带宽（-3dB 频率）
    - 相位裕量
    - 谐振频率和阻尼比
    """
    t = np.linspace(0, duration, int(duration * fs))

    # Chirp signal：频率从 f_start 线性增长到 f_end
    phase = 2 * np.pi * (f_start * t +
            (f_end - f_start) * t**2 / (2 * duration))
    cmd = amplitude * np.sin(phase)

    # 执行实验
    actual = np.zeros_like(cmd)
    for i, c in enumerate(cmd):
        robot.set_joint_position(joint_idx, c)
        actual[i] = robot.get_joint_position(joint_idx)
        # 等待控制周期
        time.sleep(1.0 / fs)

    return t, cmd, actual
```

```python
# 从扫频数据拟合传递函数
from scipy import signal

def fit_transfer_function(t, cmd, actual, fs):
    """
    从扫频数据拟合二阶传递函数。

    G(s) = ωn² / (s² + 2ζωn·s + ωn²)

    其中：
    ωn = 自然频率 (rad/s)
    ζ = 阻尼比 (无量纲)

    从 ωn 和 ζ 可以推导出有效的 kp 和 kd：
    kp = ωn² × J  (J = 关节惯量)
    kd = 2ζωn × J
    """
    # FFT
    f = np.fft.rfftfreq(len(t), 1.0/fs)
    H_cmd = np.fft.rfft(cmd)
    H_actual = np.fft.rfft(actual)

    # 传递函数估计
    H = H_actual / (H_cmd + 1e-10)
    magnitude_db = 20 * np.log10(np.abs(H) + 1e-10)

    # 找 -3dB 带宽
    dc_gain = magnitude_db[1]  # 跳过 0 Hz
    bw_idx = np.argmax(magnitude_db < dc_gain - 3)
    bandwidth = f[bw_idx]

    print(f"估计带宽: {bandwidth:.1f} Hz")
    print(f"等效 ωn ≈ {2*np.pi*bandwidth:.1f} rad/s")

    return bandwidth, f, magnitude_db
```

**实验 2：阶跃响应**

```python
# step_response.py — 阶跃响应实验
def step_response_test(robot, joint_idx, step_size=0.5, hold_time=2.0):
    """
    对关节施加阶跃命令，记录响应。

    从响应可以读出：
    - 上升时间 (rise time): 从 10% 到 90% 的时间
    - 超调量 (overshoot): 超过目标值的百分比
    - 稳态误差 (steady-state error): 最终位置与目标的差
    - 安定时间 (settling time): 进入±2%带的时间
    """
    data = {"t": [], "cmd": [], "actual": []}

    # Phase 1: 静止 0.5 秒
    for _ in range(100):
        robot.set_joint_position(joint_idx, 0.0)
        data["t"].append(len(data["t"]) * 0.005)
        data["cmd"].append(0.0)
        data["actual"].append(robot.get_joint_position(joint_idx))
        time.sleep(0.005)

    # Phase 2: 阶跃到 step_size
    for _ in range(int(hold_time / 0.005)):
        robot.set_joint_position(joint_idx, step_size)
        data["t"].append(len(data["t"]) * 0.005)
        data["cmd"].append(step_size)
        data["actual"].append(robot.get_joint_position(joint_idx))
        time.sleep(0.005)

    # 分析
    actual = np.array(data["actual"])
    target = step_size

    # 上升时间
    idx_10 = np.argmax(actual > 0.1 * target)
    idx_90 = np.argmax(actual > 0.9 * target)
    rise_time = (idx_90 - idx_10) * 0.005

    # 超调量
    overshoot = (actual.max() - target) / target * 100

    # 稳态误差
    steady_state = actual[-100:].mean()
    ss_error = abs(steady_state - target)

    print(f"上升时间: {rise_time*1000:.1f} ms")
    print(f"超调量: {overshoot:.1f}%")
    print(f"稳态误差: {ss_error:.4f} rad ({np.degrees(ss_error):.2f}°)")

    # 从上升时间估计有效 kp
    # 对于二阶系统：rise_time ≈ 1.8 / ωn
    omega_n = 1.8 / rise_time if rise_time > 0 else 0
    print(f"等效 ωn: {omega_n:.1f} rad/s")
    print(f"等效带宽: {omega_n/(2*np.pi):.1f} Hz")

    return data
```

**实验 3：摩擦测量**

```python
# friction_test.py — 摩擦参数估计
def friction_measurement(robot, joint_idx, velocities=None):
    """
    在不同恒定速度下测量稳态力矩→估计摩擦模型。

    摩擦模型：τ_friction = τ_coulomb × sign(q̇) + b_viscous × q̇

    τ_coulomb: 库仑摩擦（速度无关的恒定摩擦）
    b_viscous: 粘性摩擦系数
    """
    if velocities is None:
        velocities = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, -0.1, -0.5, -2.0, -5.0]

    results = {"velocity": [], "torque": []}

    for v_target in velocities:
        # 用速度控制模式驱动关节到恒定速度
        # 记录稳态时的力矩
        steady_torques = []
        for _ in range(500):
            robot.set_joint_velocity(joint_idx, v_target)
            time.sleep(0.005)
        for _ in range(200):
            tau = robot.get_joint_torque(joint_idx)
            steady_torques.append(tau)
            time.sleep(0.005)

        avg_torque = np.mean(steady_torques)
        results["velocity"].append(v_target)
        results["torque"].append(avg_torque)

    # 拟合 τ = τ_c × sign(q̇) + b × q̇
    v = np.array(results["velocity"])
    tau = np.array(results["torque"])

    # 最小二乘拟合
    A = np.column_stack([np.sign(v), v])
    params, _, _, _ = np.linalg.lstsq(A, tau, rcond=None)
    tau_coulomb, b_viscous = params

    print(f"库仑摩擦: {tau_coulomb:.3f} N·m")
    print(f"粘性摩擦系数: {b_viscous:.4f} N·m·s/rad")

    return tau_coulomb, b_viscous
```

### 从辨识数据配置仿真器参数

```python
# configure_from_sysid.py — 从系统辨识结果配置仿真器
def configure_mjcf_from_sysid(sysid_results):
    """
    从系统辨识结果生成 MJCF actuator 配置。

    sysid_results: Dict
      bandwidth_hz: 带宽 (Hz)
      damping_ratio: 阻尼比
      coulomb_friction: 库仑摩擦 (N·m)
      viscous_friction: 粘性摩擦系数 (N·m·s/rad)
      max_torque: 最大力矩 (N·m)
    """
    bw = sysid_results["bandwidth_hz"]
    zeta = sysid_results["damping_ratio"]
    tau_c = sysid_results["coulomb_friction"]
    b = sysid_results["viscous_friction"]
    tau_max = sysid_results["max_torque"]

    # 带宽 → 一阶滤波时间常数
    tau_filter = 1.0 / (2 * np.pi * bw)

    xml_snippet = f"""
    <!-- 从系统辨识结果生成的 actuator 配置 -->
    <actuator>
      <general name="joint_act" joint="joint_name"
               gaintype="fixed" gainprm="{sysid_results['kp_effective']}"
               biastype="affine" biasprm="0 -{sysid_results['kp_effective']} -{sysid_results['kd_effective']}"
               dyntype="filter" dynprm="{tau_filter:.4f}"
               forcelimited="true" forcerange="-{tau_max} {tau_max}"/>
    </actuator>
    <joint name="joint_name" frictionloss="{tau_c}" damping="{b}"/>
    """
    print(xml_snippet)
    return xml_snippet


def configure_isaac_from_sysid(sysid_results):
    """从系统辨识结果生成 Isaac Lab actuator 配置。"""
    bw = sysid_results["bandwidth_hz"]
    tau_max = sysid_results["max_torque"]

    # 带宽 → 延迟（DelayedPDActuator）
    # 近似：延迟 ≈ 0.5 / bandwidth
    delay_steps = max(1, int(0.5 / bw / 0.005))  # 0.005 = dt

    config = f"""
from isaaclab.actuators import DCMotorCfg, DelayedPDActuatorCfg

# 方案 A：DC Motor + 力矩饱和
legs_dc = DCMotorCfg(
    joint_names_expr=[".*"],
    stiffness={sysid_results['kp_effective']:.1f},
    damping={sysid_results['kd_effective']:.1f},
    saturation_effort={tau_max:.1f},
    velocity_limit={sysid_results.get('velocity_limit', 21.0):.1f},
    friction={sysid_results['coulomb_friction']:.3f},
)

# 方案 B：Delayed PD + 延迟模拟带宽
legs_delayed = DelayedPDActuatorCfg(
    joint_names_expr=[".*"],
    stiffness={sysid_results['kp_effective']:.1f},
    damping={sysid_results['kd_effective']:.1f},
    effort_limit={tau_max:.1f},
    min_delay={delay_steps},
    max_delay={delay_steps + 1},
    friction={sysid_results['coulomb_friction']:.3f},
)
"""
    print(config)
    return config
```

### 系统辨识数据的可视化分析

```python
# sysid_analysis.py — 系统辨识数据的综合分析
import numpy as np

def analyze_sysid_data(sweep_data, step_data, friction_data):
    """
    综合分析三种系统辨识实验的数据。
    生成完整的 actuator 参数报告。
    """
    report = {}

    # ======== 1. 从扫频数据分析 ========
    print("=== 1. 扫频分析 ===")
    t = sweep_data["t"]
    cmd = sweep_data["cmd"]
    actual = sweep_data["actual"]
    fs = 1.0 / (t[1] - t[0])

    # FFT 计算传递函数
    N = len(t)
    f = np.fft.rfftfreq(N, 1.0/fs)
    H_cmd = np.fft.rfft(cmd)
    H_actual = np.fft.rfft(actual)
    H = H_actual / (H_cmd + 1e-10)

    magnitude_db = 20 * np.log10(np.abs(H) + 1e-10)
    phase_deg = np.degrees(np.unwrap(np.angle(H)))

    # 找 -3dB 带宽
    valid = f > 0.5  # 忽略极低频
    dc_gain = np.median(magnitude_db[valid][:10])
    bw_mask = magnitude_db[valid] < (dc_gain - 3)
    if bw_mask.any():
        bw_idx = np.argmax(bw_mask)
        bandwidth = f[valid][bw_idx]
    else:
        bandwidth = f[-1]

    report["bandwidth_hz"] = bandwidth
    print(f"  带宽 (-3dB): {bandwidth:.1f} Hz")
    print(f"  DC 增益: {dc_gain:.1f} dB")

    # ======== 2. 从阶跃响应分析 ========
    print("\n=== 2. 阶跃响应分析 ===")
    t_step = step_data["t"]
    actual_step = step_data["actual"]
    target = step_data["target"]

    # 上升时间（10% → 90%）
    idx_start = np.argmax(np.array(t_step) > step_data["step_time"])
    pos_after_step = actual_step[idx_start:]
    idx_10 = np.argmax(pos_after_step > 0.1 * target)
    idx_90 = np.argmax(pos_after_step > 0.9 * target)
    dt_step = t_step[1] - t_step[0]
    rise_time = (idx_90 - idx_10) * dt_step

    # 超调量
    overshoot = (max(pos_after_step) - target) / target * 100

    # 稳态误差
    steady_state = np.mean(pos_after_step[-50:])
    ss_error = abs(steady_state - target)

    # 从上升时间估计 ωn
    omega_n = 1.8 / rise_time if rise_time > 0 else 100
    # 从超调量估计 ζ
    if overshoot > 0:
        zeta = -np.log(overshoot/100) / np.sqrt(
            np.pi**2 + np.log(overshoot/100)**2
        )
    else:
        zeta = 1.0  # 过阻尼

    report["rise_time_ms"] = rise_time * 1000
    report["overshoot_pct"] = overshoot
    report["ss_error_rad"] = ss_error
    report["omega_n"] = omega_n
    report["damping_ratio"] = zeta

    print(f"  上升时间: {rise_time*1000:.1f} ms")
    print(f"  超调量: {overshoot:.1f}%")
    print(f"  稳态误差: {ss_error:.4f} rad")
    print(f"  ωn: {omega_n:.1f} rad/s")
    print(f"  ζ: {zeta:.3f}")

    # 估计 kp, kd（假设关节惯量 J）
    J_estimated = 0.01  # kg·m²，需要根据实际机器人调整
    kp_eff = omega_n**2 * J_estimated
    kd_eff = 2 * zeta * omega_n * J_estimated

    report["kp_effective"] = kp_eff
    report["kd_effective"] = kd_eff
    print(f"  kp_eff (J={J_estimated}): {kp_eff:.1f} N·m/rad")
    print(f"  kd_eff (J={J_estimated}): {kd_eff:.3f} N·m·s/rad")

    # ======== 3. 从摩擦测量分析 ========
    print("\n=== 3. 摩擦分析 ===")
    v = np.array(friction_data["velocity"])
    tau = np.array(friction_data["torque"])

    # 最小二乘拟合 τ = τ_c × sign(q̇) + b × q̇
    A = np.column_stack([np.sign(v), v])
    params, _, _, _ = np.linalg.lstsq(A, tau, rcond=None)
    tau_coulomb, b_viscous = params

    report["coulomb_friction"] = abs(tau_coulomb)
    report["viscous_friction"] = abs(b_viscous)
    print(f"  库仑摩擦: {abs(tau_coulomb):.3f} N·m")
    print(f"  粘性摩擦: {abs(b_viscous):.4f} N·m·s/rad")

    # ======== 综合报告 ========
    print("\n" + "=" * 60)
    print("综合系统辨识报告")
    print("=" * 60)
    for k, v in report.items():
        print(f"  {k}: {v}")

    return report
```

### 在 mjlab 中使用辨识结果

```python
# mjlab 中根据辨识结果配置 actuator
import mujoco

def apply_sysid_to_mjlab(mjcf_path, sysid_report, output_path):
    """
    读取 MJCF 文件，根据辨识结果修改 actuator 参数。
    """
    spec = mujoco.MjSpec.from_file(mjcf_path)

    # 遍历所有 actuator，更新参数
    for act in spec.actuators:
        # 更新 kp（通过 gainprm）
        act.gainprm[0] = sysid_report["kp_effective"]
        # 更新 kd（通过 biasprm）
        act.biasprm[1] = -sysid_report["kp_effective"]
        act.biasprm[2] = -sysid_report["kd_effective"]
        # 设置动态类型为一阶滤波
        act.dyntype = mujoco.mjtDyn.mjDYN_FILTER
        bw = sysid_report["bandwidth_hz"]
        act.dynprm[0] = 1.0 / (2 * 3.14159 * bw)
        print(f"  Updated actuator '{act.name}': "
              f"kp={sysid_report['kp_effective']:.1f}, "
              f"bw={bw:.1f}Hz")

    # 遍历所有 joint，更新摩擦
    for jnt in spec.joints:
        if jnt.type == mujoco.mjtJoint.mjJNT_HINGE:
            jnt.frictionloss = sysid_report["coulomb_friction"]
            jnt.damping = sysid_report["viscous_friction"]
            print(f"  Updated joint '{jnt.name}': "
                  f"friction={sysid_report['coulomb_friction']:.3f}, "
                  f"damping={sysid_report['viscous_friction']:.4f}")

    # 保存修改后的 MJCF
    model = spec.compile()
    mujoco.mj_saveLastXML(output_path, model)
    print(f"\n已保存辨识后的 MJCF: {output_path}")
```

### mjlab 中的 actuator 配置实战

mjlab 使用 MjSpec API 配置 actuator。以下是在 mjlab 的 EntityCfg 中完整配置 actuator 的示例：

```python
# mjlab EntityCfg 中的 actuator 配置
import mujoco
from pathlib import Path

class Go1WithSysIdActuator:
    """
    使用系统辨识参数的 Go1 配置。
    展示 mjlab 中配置 actuator 的标准方式。
    """
    @staticmethod
    def spec_fn() -> mujoco.MjSpec:
        spec = mujoco.MjSpec.from_file(
            str(Path(__file__).parent / "assets" / "go1.xml")
        )

        # 从辨识结果配置 actuator
        sysid = {
            "kp": 120.0,       # 辨识得到的有效 kp
            "kd": 3.5,         # 辨识得到的有效 kd
            "bw_hz": 28.0,     # 辨识得到的带宽
            "tau_max": 23.7,   # 数据手册峰值扭矩
            "friction": 0.3,   # 辨识得到的库仑摩擦
        }

        tau_filter = 1.0 / (2 * 3.14159 * sysid["bw_hz"])

        for act in spec.actuators:
            # 将 position actuator 替换为 general actuator
            # 以支持 dyntype="filter"
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.gainprm[0] = sysid["kp"]
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.biasprm[0] = 0.0
            act.biasprm[1] = -sysid["kp"]
            act.biasprm[2] = -sysid["kd"]
            act.dyntype = mujoco.mjtDyn.mjDYN_FILTER
            act.dynprm[0] = tau_filter
            act.forcelimited = True
            act.forcerange = [-sysid["tau_max"], sysid["tau_max"]]

        for jnt in spec.joints:
            if jnt.type == mujoco.mjtJoint.mjJNT_HINGE:
                jnt.frictionloss = sysid["friction"]

        return spec
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：扫频实验中幅度太大会导致非线性效应。** 如果扫频幅度让关节接近限位或力矩饱和，频率响应不再是线性系统的特征。推荐幅度 <30% 的关节范围。

💡 **概念误区：认为系统辨识一次就够了。** 电机特性随温度变化（热机后摩擦降低、效率变化）。如果你的实验在冷机状态下做，但部署在热机状态下，辨识结果可能不准确。推荐在"热机稳态"下做辨识（先运行 5-10 分钟让电机暖机）。

### 练习

1. **[动手题]** 如果你有真实机器人，对一个关节执行阶跃响应测试。如果没有，用 MuJoCo 的 `<general>` actuator（加了 dyntype="filter"）作为"真机"，执行阶跃响应并拟合参数。
2. **[计算题]** 一个关节的阶跃响应显示：上升时间 50ms，超调量 15%，稳态误差 0.02 rad。估计该关节的等效 kp、kd 和带宽。

---

前面四节分别讲解了四个层级的工程实现和系统辨识方法。但面对一个具体项目，你应该选哪个层级？投入的工程时间值不值得？下一节用定量分析和实际场景帮你做出决策。

## 12.6 Actuator 建模与 Sim-to-Real Gap ⭐⭐

> **这一节解决什么问题**：用具体数据说明 actuator 模型对 sim-to-real gap 的影响，帮助学生建立"什么时候该投入时间做 actuator 建模"的工程直觉。

### 不同 Actuator 模型对力矩误差的影响

```text
场景：Go1 四足机器人，髋关节，命令 q*=0.5 rad

真机实际力矩：15.2 N·m

不同模型的预测：
  Ideal PD (kp=100):           20.0 N·m  → 误差 32%
  DC Motor (τ_stall=33.5):     17.8 N·m  → 误差 17%
  Actuator Network:            15.5 N·m  → 误差 2%
  Delta Action (仿真中):       15.3 N·m  → 误差 0.7%
```

这些误差看起来不大，但在 RL 训练中它们会**累积**——每一步的力矩误差导致下一步的状态误差，状态误差导致策略选择不同的 action，最终整条轨迹偏离真机。

为了更直观理解累积效应，考虑一个 200 步的 episode（1 秒，dt=5ms）：

```python
# 力矩误差的累积效应估算
"""
单步力矩误差：ε_τ = 4.8 N·m（Ideal PD, 32%）
单步角加速度误差：ε_α = ε_τ / J = 4.8 / 0.01 = 480 rad/s²
单步角速度误差：ε_ω = ε_α × dt = 480 × 0.005 = 2.4 rad/s
单步角度误差：ε_q = ε_ω × dt = 2.4 × 0.005 = 0.012 rad

10 步累积角度误差（粗略估计）：
  ~10 × 0.012 = 0.12 rad ≈ 7°
  这已经是四足步态中膝关节角度变化的 10-20%

100 步累积：误差被放大到不可控——
  策略进入了从未在训练中见过的状态空间
  → 行为退化 → episode 提前终止
"""
```

### 三个实际场景的选型分析

**场景 A：博士一年级 — 学习 RL locomotion**

```text
情况：第一次训练四足行走，目标是跑通 pipeline
硬件：无真机，纯仿真
时间：1-2 周

推荐：Ideal PD + 标准 DR（Ch08 的默认配置）
理由：
  - 关注算法理解，不需要 sim2real 精度
  - Ideal PD 的训练速度最快（吞吐量最高）
  - 先确保训练收敛，再考虑物理精度
避免：过早投入 actuator network 训练（没有真机数据也做不了）
```

**场景 B：准备 ICRA 2026 demo — G1 跳旋**

```text
情况：需要在 Unitree G1 上实现跳跃旋转
硬件：有 G1 真机
时间：3-6 个月

推荐：ASAP Delta Action Model
理由：
  - 跳旋需要在力矩极限附近操作 → Ideal PD 不够
  - G1 没有关节力矩传感器 → Actuator Network 数据收集困难
  - Delta Action 只需要 (s, a, s') → 可用关节编码器收集
  - ASAP 在 G1 跳旋上已有成功案例

工程步骤：
  1. 用 DC Motor + DR 预训练基础策略（2 周）
  2. 在 G1 上收集 5-10 分钟 rollout 数据（1 天）
  3. 训练 delta action model（1 天）
  4. 在修正后的仿真器中微调策略（1 周）
  5. 真机部署和调试（2 周）
```

**场景 C：工业场景 — ANYmal 巡检**

```text
情况：ANYmal 四足机器人在工厂中巡逻
硬件：有 ANYmal + 力矩传感器
时间：持续迭代

推荐：Actuator Network
理由：
  - ANYmal 有力矩传感器 → 可以收集高质量数据
  - 巡检任务不需要极限动作 → 但需要高可靠性
  - Actuator Network 提高 nominal 精度 → 减少意外摔倒
  - 可以定期重新收集数据更新 actuator network（处理磨损）

工程步骤：
  1. 用 Hwangbo 2019 的方法收集 actuator 数据（1 天）
  2. 训练 actuator network（几小时）
  3. 在 Isaac Lab 中用 ActuatorNetMLPCfg 集成（1 天）
  4. 训练 + 部署（标准流程）
  5. 每 3 个月重新收集数据更新 actuator network
```

### 什么时候 Ideal PD + DR 就够了

对于大多数四足 locomotion 项目（走路、小跑），Ideal PD + 适当的 DR 已经足够。原因：

- 步态周期中的力矩变化远大于 actuator 模型误差
- DR 的 kp 随机化（0.75-1.5 倍）已经覆盖了 Ideal PD 和真机之间的差异
- locomotion 的 reward 对力矩精度不太敏感（只关心"走多快""走多稳"）

如果你不确定 Ideal PD + DR 是否足够，做以下快速实验：

```python
# 快速诊断 actuator 模型是否足够
def quick_actuator_diagnosis(env, trained_policy):
    """
    在训练好的策略上做简单诊断：
    1. 记录每步的 actuator 力矩
    2. 检查力矩是否经常饱和
    3. 检查力矩变化频率
    """
    torque_log = []
    for _ in range(1000):
        obs = env.get_observations()
        action = trained_policy(obs)
        env.step(action)

        # 记录 actuator 力矩
        tau = env.robot.actuator_forces.clone()
        torque_log.append(tau)

    torques = torch.stack(torque_log)  # (1000, num_joints)

    # 分析
    tau_max = env.robot.effort_limit
    saturation_ratio = (torques.abs() > 0.9 * tau_max).float().mean()

    # 力矩变化频率（通过差分估算）
    dtau = torch.diff(torques, dim=0)
    high_freq_ratio = (dtau.abs() > 5.0).float().mean()

    print(f"力矩饱和比例: {saturation_ratio*100:.1f}%")
    print(f"高频力矩变化比例: {high_freq_ratio*100:.1f}%")

    if saturation_ratio > 0.1:
        print("⚠️ 力矩经常饱和 → 考虑 DC Motor 模型")
    if high_freq_ratio > 0.2:
        print("⚠️ 高频力矩变化频繁 → 考虑带宽限制模型")
    if saturation_ratio < 0.05 and high_freq_ratio < 0.1:
        print("✅ Ideal PD + DR 可能已经足够")
```

### 什么时候需要更精确的 actuator 模型

1. **极限动作（跳跃、翻转）**：需要在力矩极限附近操作，饱和效应很重要
2. **高速运动（快跑 >3 m/s）**：反电动势显著降低可用力矩
3. **精确操作（抓取、插入）**：需要精确的力控
4. **真机部署后"动作保守"**：策略在仿真中激进但在真机上保守——通常是 actuator 模型偏差导致

### 定量分析：不同层级对训练结果的影响

以下是一个系统性对比实验的设计框架：

```python
# actuator_ablation.py — actuator 模型层级消融实验
"""
实验设计：固定所有其他参数，只改变 actuator 模型。

指标：
  1. 训练 reward（仿真中）
  2. sim-to-sim reward（从 MuJoCo 迁移到 PhysX）
  3. 真机 reward（如果有真机）

预期结果：
  - 仿真 reward：Ideal PD ≈ DC Motor > Actuator Net（net 在仿真中增加了约束）
  - sim-to-sim reward：Ideal PD < DC Motor < Actuator Net
  - 真机 reward：Ideal PD ≪ DC Motor < Actuator Net
"""

experiment_configs = {
    "A_ideal_pd": {
        "actuator_type": "IdealPD",
        "mjcf_actuator": '<position kp="100" kv="4" forcerange="-33.5 33.5"/>',
        "isaac_cfg": "ImplicitActuatorCfg(stiffness=100, damping=4)",
    },
    "B_dc_motor": {
        "actuator_type": "DCMotor",
        "mjcf_actuator": '<general gaintype="fixed" ... dyntype="filter" dynprm="0.02"/>',
        "isaac_cfg": "DCMotorCfg(saturation_effort=33.5, velocity_limit=21)",
    },
    "C_actuator_net": {
        "actuator_type": "ActuatorNet",
        "mjcf_actuator": "mjcb_control callback",
        "isaac_cfg": "ActuatorNetMLPCfg(network_file='actuator_net.pt')",
    },
    "D_ideal_pd_plus_dr": {
        "actuator_type": "IdealPD + DR",
        "mjcf_actuator": '<position kp="100"/> + kp DR U(0.75, 1.5)',
        "isaac_cfg": "ImplicitActuator + randomize_actuator_gains",
    },
}
```

### 关键经验法则

根据 2024-2025 年文献的共识：

| 任务难度 | 推荐层级 | 预估 sim-to-real gap |
|---------|---------|---------------------|
| 平地行走 (v < 1 m/s) | Ideal PD + DR | 10-20% reward 下降 |
| 快跑/转向 (v 1-3 m/s) | DC Motor + DR | 15-25% reward 下降 |
| 跳跃/翻转 | Actuator Net 或 Delta Action | 5-15% reward 下降 |
| 极限动作（旋转跳、空翻） | ASAP Delta Action | <10% reward 下降 |

> **本质洞察：** Actuator 建模的投资回报率与任务难度正相关。走路不需要精确的 actuator 模型——DR 就够了。跳旋需要——因为策略必须在力矩极限附近精确控制，DR 的范围不够覆盖非线性效应。选择 actuator 建模层级时，先问"DR 够不够"——如果够，省下工程时间做其他事情。

### 选型决策流程图

```text
开始
│
├── 你有真机吗？
│   ├── 否 → Ideal PD + DR（纯仿真研究）
│   └── 是 → 继续
│
├── 你有关节力矩传感器/电流测量？
│   ├── 是 → Q: 任务需要极限力矩？
│   │         ├── 是 → Actuator Network
│   │         └── 否 → DC Motor + DR
│   └── 否 → Q: 任务需要极限力矩？
│             ├── 是 → ASAP Delta Action（不需力矩传感器）
│             └── 否 → Ideal PD + DR
│
├── DR 在真机上够不够？
│   ├── 够（真机性能 ≥ 80% 仿真性能）→ 当前层级够了
│   └── 不够 → 升级一个层级
```

### ⚠️ 常见陷阱

🧠 **思维陷阱：认为"做了 actuator modeling 就不需要 DR 了"。** 即使用了最精确的 actuator network，真机的 actuator 特性仍然会随时间变化（温度、磨损、电池电量）。DR 仍然需要——只是范围可以缩小（从 U(0.5, 2.0) 缩小到 U(0.85, 1.15)）。

⚠️ **编程陷阱：在仿真中训练时启用 actuator network 会降低吞吐量。** Actuator Network 的前向计算发生在 Python（或 GPU 上的自定义 kernel）中，而 Ideal PD 的计算在 C++ 内核中——可能慢 2-3x。推荐策略：先用 Ideal PD 训练到 80% 性能，再切换到 Actuator Network 做最后的微调。

---

## 本章小结

| 知识点 | 核心结论 | 重要程度 |
|--------|---------|---------|
| 四层 actuator 模型层级 | Ideal PD → DC Motor → Actuator Net → Delta Action | ⭐ |
| Actuator Network 原创归因 | Hwangbo 2019 (ANYmal Science Robotics)，非 walk-these-ways | ⭐⭐ |
| MuJoCo position actuator | kp/kv + forcerange，kv 与 joint damping 叠加 | ⭐⭐⭐ |
| MuJoCo general actuator | gaintype + biastype + dyntype = 任意线性 actuator 模型 | ⭐⭐⭐ |
| Isaac Lab actuator 层级 | Implicit → IdealPD → DCMotor → DelayedPD → ActuatorNetMLP | ⭐⭐⭐ |
| DC Motor 力矩-速度约束 | τ_max(q̇) = τ_stall × (1 - \|q̇\|/q̇_max) | ⭐⭐⭐ |
| dyntype="filter" | 一阶低通滤波器模拟电机带宽限制 | ⭐⭐⭐ |
| Actuator Network 架构 | MLP [128,128] ELU + H 步历史输入 | ⭐⭐⭐ |
| Actuator Network 数据需求 | 5-10 分钟真机数据（扫频 + 随机正弦） | ⭐⭐ |
| UAN 无监督方法 | 不需要力矩传感器，只需 (s, a, s') | ⭐⭐⭐ |
| ASAP Delta Action Model | 在 action 空间做残差修正，部署时零开销 | ⭐⭐⭐ |
| Delta Action vs Actuator Net | 建模目标不同 + 数据需求不同 | ⭐⭐⭐ |
| 扫频辨识 | chirp → FFT → 带宽 + 阻尼比 | ⭐⭐ |
| 阶跃响应辨识 | 上升时间 → ωn，超调量 → ζ | ⭐⭐ |
| 摩擦辨识 | 恒速扫描 → 库仑 τ_c + 粘性 b | ⭐⭐ |
| DR 与 actuator model 的关系 | 互补非替代：model 提高名义精度，DR 覆盖参数变化 | ⭐⭐⭐⭐ |
| 选型决策 | 先问"DR 够不够"→ 不够才升级 actuator 建模层级 | ⭐⭐⭐ |
| 双框架配置速查表 | MuJoCo ↔ Isaac Lab 的每种 actuator 类型对应关系 | ⭐⭐⭐ |

## 累积项目

本章需要在你的累积项目中完成以下工作：

1. 对你的机器人模型的 actuator 配置做一次完整审计：当前类型、kp/kd 值、是否有 forcerange
2. 在 MuJoCo 中用 `<general>` actuator + `dyntype="filter"` 模拟带宽限制 actuator，与 Ideal PD 对比阶跃响应
3. 如果有真机数据，训练一个 actuator network 并评估力矩预测精度
4. 完成 A/B 消融实验：Ideal PD vs DC Motor 在 velocity tracking 训练中的 reward 差异

### 实验 Lab：Actuator 模型消融实验

```python
# ch12_experiment_lab.py — actuator 模型消融实验
"""
实验设计：
  A 组: Ideal PD (kp=100, kd=4)
  B 组: DC Motor (kp=100, kd=4, τ_stall=33.5, q̇_max=21)
  C 组: Ideal PD + 带宽限制 (dyntype="filter", τ=20ms)

训练任务：velocity tracking (Go1/G1)
训练长度：2000 iterations
评估指标：reward, tracking error, episode length
"""

experiment_configs = {
    "A_ideal_pd": {
        "actuator": "IdealPD",
        "kp": 100, "kd": 4,
        "forcerange": 33.5,
        "dyntype": "none",
    },
    "B_dc_motor": {
        "actuator": "DCMotor",
        "kp": 100, "kd": 4,
        "saturation_effort": 33.5,
        "velocity_limit": 21.0,
    },
    "C_bandwidth_limited": {
        "actuator": "BandwidthLimited",
        "kp": 100, "kd": 4,
        "forcerange": 33.5,
        "dyntype": "filter",
        "dynprm": 0.02,  # 20ms 时间常数 ≈ 8 Hz 带宽
    },
}

expected_results = {
    "reward@2000": {
        "A": "~0.85（最高——无物理约束）",
        "B": "~0.78（力矩饱和影响高速动作）",
        "C": "~0.75（带宽限制 + 力矩限制）",
    },
    "tracking_error": {
        "A": "最小（仿真中无损失）",
        "B": "略大（高速时力矩不足）",
        "C": "最大（滤波延迟导致相位偏移）",
    },
    "sim2real_gap（如果有真机）": {
        "A": "最大（仿真太理想化）",
        "B": "中等",
        "C": "最小（最接近真机）",
    },
}
```

### 实验 Lab：mjlab 中的完整消融流程

```python
# ch12_mjlab_ablation.py — 在 mjlab 中执行 actuator 消融实验
"""
此脚本展示如何在 mjlab 中修改 actuator 配置并训练。

关键：mjlab 的 actuator 通过 MjSpec 在 compile 前修改。
不同于 Isaac Lab 的 Cfg 数据类，mjlab 直接操作 XML 属性。
"""
import mujoco

def create_ideal_pd_spec(base_xml, kp=100.0, kd=4.0, tau_max=33.5):
    """创建 Ideal PD 配置的 MjSpec。"""
    spec = mujoco.MjSpec.from_file(base_xml)
    for act in spec.actuators:
        act.gainprm[0] = kp
        act.biasprm[1] = -kp
        act.biasprm[2] = -kd
        act.dyntype = mujoco.mjtDyn.mjDYN_NONE  # 无动态
        act.forcelimited = True
        act.forcerange = [-tau_max, tau_max]
    return spec

def create_bandwidth_limited_spec(base_xml, kp=100.0, kd=4.0,
                                   tau_max=33.5, bw_hz=30.0):
    """创建带宽限制配置的 MjSpec。"""
    spec = mujoco.MjSpec.from_file(base_xml)
    tau_filter = 1.0 / (2 * 3.14159 * bw_hz)
    for act in spec.actuators:
        act.gainprm[0] = kp
        act.biasprm[1] = -kp
        act.biasprm[2] = -kd
        act.dyntype = mujoco.mjtDyn.mjDYN_FILTER
        act.dynprm[0] = tau_filter
        act.forcelimited = True
        act.forcerange = [-tau_max, tau_max]
    return spec


def run_ablation(base_xml, task_cfg, num_iterations=2000):
    """运行 A/B/C 三组实验。"""
    configs = {
        "A_IdealPD": create_ideal_pd_spec(base_xml),
        "C_BW_Limited": create_bandwidth_limited_spec(
            base_xml, bw_hz=30.0
        ),
    }

    results = {}
    for name, spec in configs.items():
        print(f"\n{'='*50}")
        print(f"训练配置: {name}")
        print(f"{'='*50}")

        # 编译模型
        model = spec.compile()
        print(f"  Actuators: {model.nu}")
        print(f"  dyntype[0]: {model.actuator_dyntype[0]}")

        # 这里接入 mjlab 的标准训练循环
        # reward = train(model, task_cfg, num_iterations)
        # results[name] = reward

    return results
```

### 快速验证脚本

```python
# verify_ch12_completion.py — 检查累积项目完成度
def verify():
    import os
    checks = []

    if os.path.exists("actuator_audit.txt"):
        checks.append("✅ Actuator 审计报告")
    else:
        checks.append("❌ 缺少 actuator 审计报告")

    if os.path.exists("step_response_comparison.png"):
        checks.append("✅ 阶跃响应对比图")
    else:
        checks.append("❌ 缺少阶跃响应对比")

    if os.path.exists("actuator_net.pt"):
        checks.append("✅ Actuator Network 权重文件")
    else:
        checks.append("⚠️ 无 actuator network（可选）")

    checks.append("⚠️ 消融实验是否已记录到 WandB？")

    for c in checks:
        print(c)

verify()
```

### 与其他章节的连接

本章的 actuator 模型直接影响 Ch05（Action Space 设计）——如果使用 DC Motor 模型，策略需要学会在力矩饱和区域操作。Ch05 中讨论的 position action 默认通过 PD 控制器映射到力矩——本章深入讨论了这个映射的物理精度。

本章的 actuator DR 参数（kp 随机化范围）是 Ch08（Domain Randomization）的核心参数之一。Ch08 给出了"怎么随机化"（EventManager 接口），本章给出了"随机化范围应该多大"的物理依据——从系统辨识的真机数据中确定 kp 的实际变化范围。

本章的 delta action model 是 Ch23（Sim-to-Real）的关键工具。ASAP 的完整部署管线（预训练 → 真机数据收集 → delta model 训练 → 策略微调 → 部署）在 Ch23 中作为端到端案例详细展开。

本章的 actuator network 训练管线复用了 Ch09（Teacher-Student）和 Ch10（BC/DAgger）的监督学习方法——actuator network 本质上是一个从真机数据做 behavioral cloning 的过程，只是 "student" 学习的不是策略而是物理模型。

本章的系统辨识方法（扫频、阶跃响应）在 Ch11（机器人建模）中的惯性参数验证有所涉及，但本章更深入——从频域和时域两个角度完整覆盖了参数辨识的方法论。

本章的 MuJoCo `<general>` actuator 是后续所有自定义 actuator 场景的基础——Ch22（DIY 机器人项目）中的自定义电机配置直接使用本章的 gaintype/biastype/dyntype 框架。

### 实验记录模板

```text
Ch12 累积项目实验记录
━━━━━━━━━━━━━━━━━━━
日期：
机器人：
框架：mjlab / Isaac Lab

Part 1: Actuator 审计
  当前 actuator 类型：Ideal PD / DC Motor / 其他
  kp = ___，kd = ___
  forcerange: [-___, ___] N·m
  dyntype: none / filter (τ=___ms)
  frictionloss: ___ N·m
  armature: ___ kg·m²
  真机数据手册的额定扭矩：___ N·m
  真机数据手册的峰值扭矩：___ N·m
  真机空载转速：___ rad/s
  数据手册 vs 仿真 kp 匹配度：___

Part 2: 阶跃响应对比
  Ideal PD:
    上升时间: ___ms，超调: ___%, 安定时间: ___ms
  带宽限制 (τ=___ms):
    上升时间: ___ms，超调: ___%, 安定时间: ___ms
  （如果有真机）真机:
    上升时间: ___ms，超调: ___%, 安定时间: ___ms

Part 3: 消融实验（2000 iter velocity tracking）
  A 组 (Ideal PD):    reward = ___
  B 组 (DC Motor):    reward = ___
  C 组 (BW Limited):  reward = ___
  差异分析：___

Part 4: 系统辨识（如果有真机）
  扫频带宽：___ Hz
  阶跃响应 kp_eff：___ N·m/rad
  阶跃响应 kd_eff：___ N·m·s/rad
  ωn：___ rad/s
  ζ：___
  库仑摩擦：___ N·m
  粘性摩擦：___ N·m·s/rad

Part 5: 结论
  当前 actuator 模型层级是否足够：是/否
  建议的升级方向：___
  预估 sim-to-real gap 改善：___ %
```

---

> **Ch12 全章知识图谱**：本章覆盖了 actuator 建模的 4 个层级（从 Ideal PD 到 Delta Action Model）、3 种系统辨识方法（扫频/阶跃/摩擦）、2 个参考项目（Hwangbo 2019 / ASAP 2025）、以及双框架（mjlab + Isaac Lab）的完整配置方式。如果你只有时间做一件事，请做"对你的模型执行 actuator 审计 + 在 MuJoCo 中对比 Ideal PD vs 带宽限制的阶跃响应"——这 10 分钟的工作可以让你建立起关于 actuator 模型精度的直觉，指导后续所有 sim-to-real 的工程决策。

## 延伸阅读

| 资料 | 难度 | 推荐原因 |
|------|------|---------|
| Hwangbo et al. 2019, "Learning agile and dynamic motor skills for legged robots" (Science Robotics) | ⭐⭐⭐ | Actuator Network 的**原创论文**，必读 |
| He et al. 2025, "ASAP: Aligning Simulation and Real-World Physics" (RSS) | ⭐⭐⭐ | Delta Action Model，2025 sim2real SOTA |
| Fey et al. 2025, "Bridging the Sim-to-Real Gap for Athletic Loco-Manipulation" (UAN, MIT) | ⭐⭐⭐ | 无监督 actuator network，5 分钟真机数据 |
| Margolis & Agrawal 2022, "Walk These Ways" (CoRL) | ⭐⭐ | MoB gait conditioning（注意：非 actuator net 原创） |
| MuJoCo actuator 文档 (mujoco.readthedocs.io/en/stable/XMLreference.html#actuator) | ⭐⭐ | general actuator 的完整参数参考 |
| Isaac Lab actuator API (isaac-sim.github.io/IsaacLab) | ⭐⭐ | ImplicitActuator/DCMotor/ActuatorNetMLP 配置 |
| Singh et al. 2023, "Learning Bipedal Walking for Humanoids with Current Feedback" | ⭐⭐ | 电流反馈在 sim2real 的应用 |
| Lee et al. 2025, "Learning Quadrupedal Locomotion for Heavy Hydraulic Robot Using Actuator Model" | ⭐⭐ | 液压 actuator 建模（非电机） |
| Sehoon Ha et al. 2025, "Learning-based legged locomotion: State of the art" (IJRR survey) | ⭐⭐ | actuator modeling 在 sim2real 中角色的综述 |

**阅读顺序建议**：先读 Hwangbo 2019（理解 actuator network 的原始动机和方法），再读 ASAP 2025（理解 delta action model 为什么更优），然后读 MuJoCo/Isaac Lab 文档（掌握双框架的配置方式）。UAN 和 Singh 论文作为 actuator modeling 前沿的补充阅读。

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 仿真中跳得高但真机跳不起来 | Ideal PD 无力矩饱和 | 1. 加 forcerange 2. 升级 DC Motor 3. 检查 τ_stall | 本章 12.2 |
| 真机动作"拖沓"、响应慢 | 仿真无带宽限制 | 1. 加 dyntype="filter" 2. 用扫频测带宽 3. 设 dynprm | 本章 12.2 |
| Actuator network 预测力矩偏大 | 训练数据在冷机下收集 | 1. 热机后重新收集 2. 增加数据多样性 | 本章 12.3 |
| Delta action 微调后策略退化 | max_delta 太大 or 数据不够 | 1. 减小 max_delta 2. 增加真机 rollout | 本章 12.4 |
| kp 随机化范围设太大导致不收敛 | DR 范围超出物理合理值 | 1. 参考真机数据手册 2. 用阶跃响应测真实 kp | 本章 12.5 |
| 真机上关节"抖动" | kd 太小或 actuator 延迟 | 1. 增大 kd 2. 加 dyntype="filter" 3. 检查通信延迟 | 本章 12.2 |
| Isaac Lab DCMotor 和 MuJoCo 行为不同 | 力矩-速度曲线实现差异 | 1. 对比两端 τ(q̇) 曲线 2. 手动对齐参数 | 本章 12.2 |
| ActuatorNetMLP 在 Isaac Lab 中加载失败 | .pt 文件格式不兼容 | 1. 检查 network_file 路径 2. 确认 MLP 架构匹配 | 本章 12.3 |
| 扫频数据很嘈杂无法拟合 | 振幅太小或外部扰动 | 1. 增大扫频振幅 2. 静止环境中测量 3. 增加重复次数 | 本章 12.5 |
| 阶跃响应无超调（过阻尼） | kd 太大或摩擦太大 | 1. 减小 kd 2. 测量摩擦 3. 检查 armature | 本章 12.5 |
| 仿真中高速动作正常但真机不行 | DC Motor 力矩-速度约束未建模 | 1. 检查 velocity_limit 设置 2. 对比高速时可用力矩 | 本章 12.2 |
| Actuator network 在某些关节精度低 | 该关节的训练数据不足 | 1. 增加该关节的单独扫频数据 2. 平衡各关节数据量 | 本章 12.3 |
| Delta model 改善 <10% | 真机 gap 主要不在 actuator | 1. 检查是否是感知/延迟问题 2. 考虑增大 max_delta 3. 增加真机数据 | 本章 12.4 |
| MuJoCo general actuator 行为异常 | biasprm 符号错误 | 1. 手动验证力矩公式 2. 打印 actuator_force 对比预期 | 本章 12.2 |

---

> **本章完。** Actuator 建模是 sim-to-real 的最后一公里。当 DR 和视觉增强已经做好时，残余的 sim-to-real gap 主要来自 actuator 动态和通信延迟。本章提供了四个层级的建模工具和三种系统辨识方法——从"零投入的 Ideal PD"到"5 分钟真机数据的 Delta Action"，覆盖了从纯仿真研究到真机部署的全部场景。

下一章（Ch13）进入 Part IV：单形态机器人实战。从 Ch13 的四足 locomotion 开始，我们终于可以把前四批建模章节（Ch11 资产管线、Ch12 actuator 建模）和训练基础（Ch04-Ch10）组合起来，完成第一个端到端的 sim-to-real locomotion 项目。Ch12 的 actuator 配置和 DR 参数将直接影响 Ch13 中 velocity tracking 任务的训练质量和 sim-to-real 迁移效果。

---

### 附录：Isaac Lab actuator 类继承关系速查

```text
ActuatorBase (抽象基类)
├── ImplicitActuator          # PhysX 内部 PD
│     └── (cfg: ImplicitActuatorCfg)
│
├── IdealPDActuator           # Python 显式 PD
│     ├── DCMotor             # + 力矩-速度饱和曲线
│     │     ├── ActuatorNetMLP    # + MLP 力矩预测
│     │     └── ActuatorNetLSTM   # + LSTM 力矩预测
│     ├── DelayedPDActuator   # + 通信延迟模拟
│     └── RemotizedPDActuator # + 遥控杆非线性
│
└── 自定义 ActuatorBase 子类  # 用户扩展
```

**选择指南**：

| 你需要什么 | 选哪个 | Isaac Lab Cfg |
|-----------|--------|---------------|
| 最快的训练速度 | ImplicitActuator | `ImplicitActuatorCfg` |
| 能观察力矩数据 | IdealPDActuator | `IdealPDActuatorCfg` |
| 力矩饱和建模 | DCMotor | `DCMotorCfg(saturation_effort=...)` |
| 通信延迟建模 | DelayedPDActuator | `DelayedPDActuatorCfg(min_delay=..., max_delay=...)` |
| 真机 actuator network | ActuatorNetMLP | `ActuatorNetMLPCfg(network_file=...)` |
| 完全自定义 | 继承 ActuatorBase | 自定义 `@configclass` |

### 附录：MuJoCo actuator 类型速查

```xml
<!-- 1. position actuator（Ideal PD） -->
<position joint="j" kp="100" kv="4"/>

<!-- 2. motor actuator（力矩控制） -->
<motor joint="j" gear="1"/>

<!-- 3. velocity actuator（速度控制） -->
<velocity joint="j" kv="10"/>

<!-- 4. general actuator（万能） -->
<general joint="j"
  gaintype="fixed"     gainprm="100"         <!-- kp -->
  biastype="affine"    biasprm="0 -100 -4"   <!-- bias = -kp*q - kd*qdot -->
  dyntype="filter"     dynprm="0.02"          <!-- 一阶低通 τ=20ms -->
  forcelimited="true"  forcerange="-33.5 33.5"/>
```

**关键对应关系**：

| MuJoCo | Isaac Lab | 物理含义 |
|--------|-----------|---------|
| `kp` (position actuator) | `stiffness` | 位置增益 (N·m/rad) |
| `kv` (position actuator) | `damping` | 速度增益 (N·m·s/rad) |
| `forcerange` | `effort_limit` | 力矩限制 (N·m) |
| `ctrlrange` | action space clip | 控制输入范围 (rad) |
| `dyntype="filter" dynprm` | `DelayedPDActuatorCfg` | 带宽限制 |
| `joint frictionloss` | `ActuatorBaseCfg(friction)` | 库仑摩擦 |
| `joint damping` | (叠加在 actuator damping 中) | 粘性摩擦 |
| `joint armature` | `ActuatorBaseCfg(armature)` | 虚拟转子惯量 |

⚠️ **MuJoCo 的 `kv` 和 `joint damping` 效果叠加**——这是跨框架对齐中最常见的 bug 来源。如果 MuJoCo 中 actuator kv=4 + joint damping=0.5，Isaac Lab 中应设 damping=4.5。详见 Ch11.9 跨仿真器参数对齐。

⚠️ **Isaac Lab v2.0 后 `effort_limit` 和 `effort_limit_sim` 区分**——`effort_limit` 是 actuator 模型的软限制（Python 中 clip），`effort_limit_sim` 是 PhysX 求解器的硬限制。如果只设了 `effort_limit` 而未设 `effort_limit_sim`，PhysX 可能允许超过 `effort_limit` 的力矩——导致仿真和真机行为不一致。推荐两者设为相同值。

> **对于只使用 mjlab 的读者**：重点关注 12.2 的 MuJoCo `<general>` actuator 配置、12.3 的 mjcb_control callback 集成方式、和 12.5 的系统辨识方法。MuJoCo 的 actuator 系统比 Isaac Lab 更灵活——一个 `<general>` actuator 通过 gaintype/biastype/dyntype 的组合就能实现 Isaac Lab 需要 4-5 个不同 Cfg 类的功能。

> **对于只使用 Isaac Lab 的读者**：重点关注 12.2 的 ImplicitActuator vs IdealPD vs DCMotor 选择、12.3 的 ActuatorNetMLPCfg 集成、和 12.4 的 Delta Action Model。Isaac Lab 的 actuator 层级设计让你可以从简单到复杂逐步升级——从 Implicit 开始训练，确认 baseline 后再切换到 DCMotor 或 ActuatorNet。

> **对于有真机部署需求的读者**：本章最重要的是 12.5（系统辨识）和 12.6（选型决策）。先用阶跃响应测量真机的 kp_eff 和带宽，再根据 12.6 的决策流程图选择 actuator 建模层级。如果你的任务不需要极限动作，Ideal PD + DR 大概率够用——省下来的时间可以投入到 reward 设计（Ch06）和 DR 调优（Ch08）中。

---

### 附录：Actuator 建模投资回报率估算

| 方法 | 工程投入（人·天） | 需要真机 | sim2real gap 改善 | 推荐优先级 |
|------|----------------|---------|-----------------|----------|
| Ideal PD + 标准 DR | 0（默认配置） | 否 | 基线 | 1（起点） |
| 从数据手册配置 forcerange | 0.5 | 否 | +5-10% | 2（低成本高回报） |
| DC Motor + 速度限制 | 0.5 | 否 | +10-15% | 3 |
| dyntype="filter" 带宽限制 | 1 | 是（测带宽） | +10-20% | 4 |
| 完整系统辨识（扫频+阶跃+摩擦） | 3-5 | 是 | +15-25% | 5 |
| Actuator Network（Hwangbo） | 5-7 | 是（力矩数据） | +25-40% | 6 |
| ASAP Delta Action Model | 7-14 | 是（rollout 数据） | +30-50% | 7（极限任务） |
| UAN（无监督） | 3-5 | 是（5 分钟） | +20-35% | 5.5（无力矩传感器） |

**阅读提示**：上表中的 "sim2real gap 改善" 是相对于 Ideal PD baseline 的近似估计。实际改善取决于任务难度、机器人类型和 DR 配置。这些数字基于 2024-2025 年文献中的经验观察，不应作为严格的定量承诺。
