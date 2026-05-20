# S3-B mjlab 深度实战：从单环境 MuJoCo 到批量强化学习训练

> **本章定位**：S3 介绍了 MuJoCo GPU 生态的全景。
> 本章把视角收窄到 mjlab/MJX 风格的批量环境工程：
> 如何把一个能在 MuJoCo CPU 中跑起来的机器人环境，
> 改造成能在 GPU 上批量采样、训练、诊断、导出和回放的 RL 训练系统。
>
> **读者画像**：已经会写基础 MuJoCo 环境、理解 PPO/RSL-RL 或 IsaacLab/legged_gym 的工程师。
> 读完后应能独立设计一个 velocity tracking 任务，
> 并能解释观测、动作、奖励、终止、域随机化和日志指标为什么这样写。
>
> **事实边界**：mjlab、MJX 和 MuJoCo Warp 仍在快速演进。
> 本章优先讲可迁移的工程思想和稳定抽象；
> 涉及安装命令、类名、任务名和版本特性的细节，
> 以官方文档和当前安装版本为准。

---

## S3-B.0 前置自测 ⭐

答不出 2 题以上，建议先回到 S1 和 S3 复习 MuJoCo 数据结构与 GPU 生态。

| # | 问题 | 期望关键词 |
|---|------|------------|
| 1 | `mjModel` 和 `mjData` 分别保存什么？为什么 MuJoCo 要把二者分开？ | 模型常量、运行状态、结构共享、多环境独立状态 |
| 2 | 单个 MuJoCo 环境中，一次 `mj_step(m, d)` 会读写哪些核心状态？ | `qpos`、`qvel`、`ctrl`、接触、传感器、派生量 |
| 3 | PPO 为什么需要一次收集很多环境的 rollout，而不是只跑一个长 episode？ | 降低方差、提高吞吐、减少样本相关性 |
| 4 | JAX 的 `vmap` 和 Python `for` 循环在执行模型上有什么差异？ | 函数式、批量轴、编译、静态形状、设备端并行 |
| 5 | 域随机化要覆盖哪些 sim2real gap？ | 动力学、接触、传感器、执行器、延迟、外扰 |

### 本章目标 ⭐

学完本章后，你应能完成 7 件事。

1. 用数学形式解释批量环境：
   $B$ 个世界共享同一套模型结构，
   但每个世界拥有独立的状态、随机数、命令和终止标志。
2. 用 JAX/MJX 的函数式思维理解自动向量化：
   `step(state, action, key)` 是纯函数，
   `vmap(step)` 把单环境函数提升为批量函数。
3. 设计 mjlab 风格的 Manager-based 环境：
   Scene、Entity、Observation、Action、Reward、Termination、Event 分别负责什么。
4. 为四足 velocity tracking 任务构造观测、动作、奖励、终止和域随机化。
5. 写出一个教学版训练 loop：
   rollout、reset、GAE、PPO update、日志、checkpoint、导出。
6. 建立性能诊断方法：
   区分物理步进慢、策略推理慢、数据搬运慢、reset 慢、日志慢。
7. 把已有的单环境 MuJoCo 代码迁移为批量训练环境，
   并能在 MuJoCo CPU 中做 sim2sim 回放。

### 知识地图 ⭐

```text
单环境 MuJoCo
  |
  |  把 mj_step 看成状态转移函数
  v
函数式环境 step/reset
  |
  |  增加 batch 维度 [num_envs, ...]
  v
MJX/JAX 风格批量环境
  |
  |  用 Manager 拆分观测、动作、奖励、终止、事件
  v
mjlab 风格训练环境
  |
  |  PPO rollout + GAE + policy update
  v
批量训练系统
  |
  |  日志、诊断、域随机化、导出、CPU 回放
  v
可调试的 sim2real 工作流
```

| 层级 | 核心问题 | 本章对应小节 |
|------|----------|--------------|
| 物理层 | 一个世界如何步进？ | S3-B.1、S3-B.2 |
| 批量层 | 多个世界如何同时步进？ | S3-B.1、S3-B.3 |
| 环境层 | RL 所需的 MDP 如何定义？ | S3-B.4 到 S3-B.7 |
| 训练层 | PPO 如何消费批量环境？ | S3-B.9 |
| 工程层 | 怎么查慢、查坏、查不收敛？ | S3-B.10 到 S3-B.14 |
| 迁移层 | 旧 MuJoCo 环境怎么改？ | S3-B.12 |

---

## S3-B.1 为什么批量环境是 RL 仿真的第一性原理 ⭐⭐⭐

### 动机：单环境很直观，但训练效率很低 ⭐

在 S1 中，MuJoCo CPU 的最小循环通常长这样。

```python
import mujoco

m = mujoco.MjModel.from_xml_path("robot.xml")
d = mujoco.MjData(m)

while True:
    d.ctrl[:] = policy(build_obs(m, d))
    mujoco.mj_step(m, d)
```

这段代码适合调试控制器。

它有三个好处。

1. 每一步都能检查 `qpos`、`qvel`、`ctrl`、`contact`。
2. 出问题时容易暂停、打印、可视化。
3. 单个机器人行为和真实世界直觉一致。

但它不适合训练深度 RL。

PPO 不是每一步更新一次策略。

PPO 的典型流程是先收集一批轨迹，
再用这些轨迹估计 advantage，
然后对同一批样本做多轮 mini-batch 更新。

如果只有 1 个环境，
一个 batch 中的样本会高度相关。

相邻状态只差一个仿真步，
奖励波动也来自同一条 episode。

这会带来两个问题。

| 问题 | 单环境表现 | 批量环境解决方式 |
|------|------------|------------------|
| 样本相关性强 | 一个 episode 内的状态连续相似 | 多个初始状态、命令和随机参数同时采样 |
| 吞吐低 | CPU 逐步执行，GPU 策略网络吃不满 | 一次推理 `[B, obs_dim]`，一次物理步进 `[B, ...]` |
| reset 浪费 | 一个环境终止时整个采样等待 | 每个环境独立 reset，其他环境继续 |
| 探索窄 | 一次只看到一种地形/命令 | 同一时刻覆盖多种任务条件 |

> **本质洞察**：批量环境不是把单环境复制很多份那么简单。
> 它把“时间上的长采样”改造成“时间 × 世界”的二维采样。
> PPO 看到的不是一条很长的故事，
> 而是一张由很多短故事拼成的经验表格。

### 从控制视角理解批量环境 ⭐⭐

回顾腿足简化模型章节：
LIPM、SRBD、Centroidal Model 都在做一件事，
就是把全身机器人压缩成可控的低维状态。

在 MPC 中，
我们通常写：

$$
x_{k+1} = f(x_k, u_k, \theta)
$$

其中 $x_k$ 是状态，
$u_k$ 是控制输入，
$\theta$ 是模型参数。

批量 RL 环境只是把这个式子加上批量维度：

$$
X_{k+1}^{(i)} = f(X_k^{(i)}, U_k^{(i)}, \theta^{(i)}, \xi_k^{(i)}),
\quad i = 1,\ldots,B
$$

这里的 $i$ 是环境编号。

每个环境拥有自己的状态 $X^{(i)}$，
自己的控制 $U^{(i)}$，
自己的随机物理参数 $\theta^{(i)}$，
自己的随机扰动 $\xi^{(i)}$。

但这些环境可以共享同一个函数 $f$。

这就是自动向量化的入口。

```text
单环境:
  x:      [state_dim]
  action: [action_dim]
  reward: scalar

批量环境:
  x:      [num_envs, state_dim]
  action: [num_envs, action_dim]
  reward: [num_envs]
```

如果把单环境看成“一个学生做一张试卷”，
批量环境就是“全班同时做同一类试卷”。

题目结构相同，
每个学生的答案不同。

老师批改时不需要换一套规则，
只需要对每一行答案应用同一个评分函数。

### 如果不批量化会怎样 ⭐

假设一个四足 velocity tracking 策略需要 $2 \times 10^8$ 个仿真步。

单环境 1000 step/s 时，
训练时间大约是：

$$
\frac{2 \times 10^8}{1000} = 2 \times 10^5 \text{ 秒} \approx 55.6 \text{ 小时}
$$

如果 4096 个环境有效并行，
即使考虑同步、reset、GPU kernel 和日志开销，
总吞吐也会提升几个数量级。

这不是“优化一点性能”，
而是改变实验迭代方式。

| 训练方式 | 实验反馈节奏 | 典型后果 |
|----------|--------------|----------|
| 单环境训练 | 半天到数天看一次曲线 | 调参成本过高，容易只调一两个参数 |
| 中等批量训练 | 几十分钟看一次趋势 | 能系统做奖励/观测消融 |
| 大批量训练 | 分钟级看早期症状 | 能快速筛掉明显错误的 MDP |

反事实推理很重要：
如果坚持用单环境训练，
你会把大量时间花在等待上。

等待会降低实验密度。

实验密度低会让奖励设计缺少对照。

缺少对照时，
策略失败到底是动作空间错、奖励错、终止错、物理参数错，
就很难分辨。

### 批量环境的数学接口 ⭐⭐

一个 RL 环境可以抽象成两个函数。

$$
s_0 = \text{reset}(\rho)
$$

$$
s_{t+1}, o_{t+1}, r_t, d_t, \text{info}_t
= \text{step}(s_t, a_t, \rho_t)
$$

其中：

| 符号 | 含义 | 批量形状 |
|------|------|----------|
| $s_t$ | 仿真内部状态 | pytree，每个叶子带 `[B, ...]` |
| $o_t$ | 策略观测 | `[B, obs_dim]` |
| $a_t$ | 策略动作 | `[B, action_dim]` |
| $r_t$ | 即时奖励 | `[B]` |
| $d_t$ | 终止标志 | `[B]` |
| $\rho_t$ | 随机数/事件输入 | `[B, ...]` 或 per-env key |

注意这里的 `state` 不等于观测。

状态是环境内部用于继续仿真的全部信息。

观测是策略能看到的信息。

四足 velocity tracking 中，
环境状态包含完整 `qpos/qvel/contact`，
但策略观测通常不直接包含世界系绝对位置。

这不是遗漏，
而是故意设计。

如果策略看到世界系绝对位置，
它可能把训练场地坐标当成任务线索。

换一个起点后，
这种线索就失效。

### 批量 reset 的关键细节 ⭐⭐

批量环境中最容易被低估的是 reset。

单环境 reset 很简单：

```python
if done:
    reset()
```

批量环境 reset 必须是逐环境的。

某些环境终止了，
其他环境还在继续。

因此 reset 逻辑是：

$$
s_{t+1}^{(i)}
=
\begin{cases}
\text{reset}(\rho^{(i)}) & d_t^{(i)} = 1 \\
\text{step}(s_t^{(i)}, a_t^{(i)}) & d_t^{(i)} = 0
\end{cases}
$$

工程上常用 mask 实现：

```python
next_state = where(done, reset_state, stepped_state)
```

这里的 `where` 不只是数组选择。

它保证所有环境仍然保持相同形状。

JAX、MJX 和 GPU kernel 都喜欢固定形状。

如果一个环境终止后把它从 batch 中删除，
batch 形状就会变化。

形状变化会破坏编译缓存，
也会让 PPO 的 rollout buffer 变得复杂。

### 教学版批量环境接口 ⭐⭐

下面的代码不依赖 mjlab，
用于说明批量环境的最小数学形态。

```python
from dataclasses import dataclass
import jax
import jax.numpy as jnp


@dataclass
class BatchPointMassState:
    # 位置，形状 [num_envs, 2]
    pos: jax.Array
    # 速度，形状 [num_envs, 2]
    vel: jax.Array
    # 每个环境已经运行的步数，形状 [num_envs]
    step_count: jax.Array
    # 每个环境的目标速度命令，形状 [num_envs, 2]
    cmd_vel: jax.Array


def reset_batch(key: jax.Array, num_envs: int) -> BatchPointMassState:
    # 为每个环境生成独立随机数，避免所有环境初始状态完全相同
    keys = jax.random.split(key, 3)
    pos = jax.random.uniform(keys[0], (num_envs, 2), minval=-0.2, maxval=0.2)
    vel = jax.random.uniform(keys[1], (num_envs, 2), minval=-0.1, maxval=0.1)
    cmd = jax.random.uniform(keys[2], (num_envs, 2), minval=-1.0, maxval=1.0)
    step_count = jnp.zeros((num_envs,), dtype=jnp.int32)
    return BatchPointMassState(pos=pos, vel=vel, step_count=step_count, cmd_vel=cmd)


def step_batch(state: BatchPointMassState, action: jax.Array, dt: float = 0.02):
    # action 是加速度命令，形状 [num_envs, 2]
    # clip 相当于执行器饱和，防止策略输出无限大的加速度
    acc = jnp.clip(action, -3.0, 3.0)
    next_vel = state.vel + acc * dt
    next_pos = state.pos + next_vel * dt
    next_step = state.step_count + 1

    # 速度跟踪奖励：越接近期望速度越高
    vel_error = jnp.sum((next_vel - state.cmd_vel) ** 2, axis=-1)
    reward = jnp.exp(-vel_error / 0.25)

    # 终止条件：跑太久或位置漂出训练区域
    timeout = next_step >= 1000
    out_of_bounds = jnp.linalg.norm(next_pos, axis=-1) > 5.0
    done = jnp.logical_or(timeout, out_of_bounds)

    next_state = BatchPointMassState(
        pos=next_pos,
        vel=next_vel,
        step_count=next_step,
        cmd_vel=state.cmd_vel,
    )
    obs = jnp.concatenate([next_vel, state.cmd_vel], axis=-1)
    info = {"vel_error": vel_error}
    return next_state, obs, reward, done, info
```

这段代码故意把“状态”和“观测”分开。

状态里有位置 `pos`，
观测里没有位置。

这对应腿足 locomotion 的常见设计：
策略关心速度命令和身体姿态，
不应该记住训练场地的绝对坐标。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 终止后把环境从 batch 中删掉 | JIT 反复编译或 buffer 维度错 | 批量训练要求静态形状 | 用 mask 原地 reset |
| 概念 | 把内部状态全部给策略看 | 训练 reward 高，换起点失败 | 策略利用了不该有的信息 | 区分 state 与 observation |
| 思维 | 认为更多环境一定更好 | 显存爆、更新变慢、曲线不稳 | batch size、horizon、mini-batch 有耦合 | 先看 throughput 和 learning stability |
| 编程 | 所有环境使用同一个随机种子 | 行为高度同步，样本多样性差 | 随机数没有 per-env 分裂 | reset/event 都使用 per-env key |

### 练习 ⭐

1. 把上面的 `BatchPointMassState` 扩展为 1D LIPM 环境：
   状态包含 $x,\dot{x}$，
   动作是 CoP 位置，
   奖励是速度跟踪和 CoP 范围惩罚。
2. 在草稿纸上推导：
   如果每个环境 episode 长度为 1000，
   `num_envs=4096`，
   `horizon=24`，
   一个 PPO rollout 中有多少 transition？
3. 设计一个 mask reset 公式：
   终止环境重新采样命令，
   未终止环境保持原命令。

---

## S3-B.2 MJLab/MJX/MuJoCo Warp 的分层关系 ⭐⭐

### 动机：名字相近，但抽象层级不同 ⭐

MuJoCo GPU 生态里有几个名字容易混在一起。

MJX、MuJoCo Warp、mjlab、Playground 都能和 GPU RL 相关。

但它们解决的问题不一样。

| 名称 | 更接近哪一层 | 主要价值 | 使用者关心的问题 |
|------|--------------|----------|------------------|
| MuJoCo CPU | 物理引擎 | 高可调试性、完整 C/Python API | 这个模型物理是否正确 |
| MJX | JAX 物理后端 | `jit/vmap/grad` 风格的批量与可微 | 如何在 JAX 中批量步进 |
| MuJoCo Warp | NVIDIA GPU 物理后端 | 大规模前向仿真吞吐 | 如何更快跑很多环境 |
| mjlab | RL 环境框架 | Manager-based 环境组织 | 如何像 IsaacLab 一样定义任务 |
| Playground | 任务与训练平台 | 快速训练示例和 sim2real 流程 | 如何快速跑通已有任务 |

截至 2026-05-15 可核验的稳定信息是：
官方 MJX 文档说明 MJX 提供 JAX API，
包含纯 JAX 实现和 Warp 实现；
MJX-Warp 面向 NVIDIA GPU 优化，
但与 MJX-JAX 不同，
当前不支持自动微分。

mjlab 官方仓库说明其目标是把 Isaac Lab 的 manager-based API
与 MuJoCo Warp 结合，
用于 GPU 加速机器人学习。

这些信息足以支撑本章的核心教学：
MJX 负责“函数式批量物理”的思维模型，
MuJoCo Warp 负责“高吞吐前向仿真”，
mjlab 负责“把 RL 任务拆成可维护的 Manager 配置”。

### 五层数据流 ⭐⭐

```text
Task / Registry
  选择环境配置、训练配置、命令行入口
        |
        v
ManagerBasedRLEnv
  统一调度 action -> sim -> obs -> reward -> done -> reset
        |
        v
Managers
  Observation / Action / Reward / Termination / Event / Curriculum / Metrics
        |
        v
Scene / Entity
  机器人、地形、物体、传感器、执行器配置
        |
        v
Simulation Backend
  MuJoCo Warp 或 MJX 风格的批量物理步进
```

这个层次和传统手写环境的区别很大。

手写环境往往把所有逻辑放进一个 `step()`。

小项目这样很快。

但任务变大后，
`step()` 会同时包含动作缩放、PD 控制、仿真步进、观测拼接、奖励计算、终止检查、域随机化和日志。

这些逻辑互相缠绕后，
两个问题会出现。

1. 想改一个 reward，
   却不小心改变了 observation 或 reset。
2. 想迁移到另一个机器人，
   却发现关节名、默认姿态和奖励函数写死在一起。

Manager-based API 的价值就是分离关注点。

| Manager | 输入 | 输出 | 设计目标 |
|---------|------|------|----------|
| Action | 策略输出、默认关节位姿、动作尺度 | 执行器目标或控制量 | 控制动作语义 |
| Observation | 仿真状态、命令、历史动作 | 策略观测向量 | 决定策略能看见什么 |
| Reward | 状态、动作、命令、接触 | 多个 reward term | 定义学习目标 |
| Termination | 状态、接触、时间 | done mask | 定义 episode 边界 |
| Event | reset/interval 时机、随机数 | 随机化后的状态/参数 | 覆盖 sim2real gap |
| Curriculum | 训练进度、指标 | 难度或权重变化 | 控制学习节奏 |
| Metrics | rollout 数据 | 日志指标 | 让训练可诊断 |

> **本质洞察**：Manager API 的本质不是“配置更多类”，
> 而是把 MDP 的五个组成部分拆成可替换零件。
> 好的环境不是 `step()` 很短，
> 而是每个变化都有明确归属：
> 动作错查 Action，
> 观测错查 Observation，
> 不收敛查 Reward 和 Termination，
> sim2real 差查 Event。

### 与 IsaacLab 的迁移关系 ⭐⭐

如果读者有 IsaacLab 经验，
mjlab 最容易理解成：

```text
IsaacLab Manager API
  + MuJoCo Warp 物理后端
  + MuJoCo/MJCF 模型生态
  + 更轻量的安装与可视化路径
  = mjlab 风格工作流
```

这并不意味着两者完全等价。

底层物理差异仍然很大。

| 维度 | IsaacLab 常见路径 | mjlab 常见路径 | 迁移时要检查 |
|------|------------------|----------------|--------------|
| 模型格式 | USD/URDF | MJCF/URDF 转换后 MJCF | 关节轴、限位、惯量、接触参数 |
| 物理引擎 | PhysX 或相关后端 | MuJoCo Warp | 接触、摩擦、关节驱动 |
| 张量生态 | PyTorch/Isaac tensor | Warp/PyTorch 互操作 | 设备、dtype、拷贝路径 |
| 环境组织 | Manager-based | Manager-based | 配置项名称与版本差异 |
| 可视化 | Omniverse/Isaac viewer | MuJoCo/Viser 等 | 无头训练和远程调试 |

反事实推理：
如果只看 API 对齐，
忽略物理差异，
迁移后的策略很可能“代码能跑但行为不对”。

例如原环境在 PhysX 中的脚底摩擦、
关节阻尼、
电机饱和和接触软硬度，
未必能直接映射到 MuJoCo。

正确迁移不是逐行翻译配置，
而是先建立物理等价性，
再迁移 MDP 结构。

### MJX 风格与 mjlab 风格的互补 ⭐⭐

MJX 风格强调函数式。

你会看到：

```python
next_data = mjx.step(model, data)
batched_step = jax.vmap(mjx.step, in_axes=(None, 0))
```

mjlab 风格强调任务组织。

你会看到：

```python
observations = {...}
actions = {...}
rewards = {...}
terminations = {...}
events = {...}
```

两者不是竞争关系。

前者解释“怎么在设备上批量计算”，
后者解释“怎么组织一个可维护的 RL 环境”。

教学上先理解 MJX 的函数式批量思维，
再看 mjlab 的 Manager 配置，
会比直接背 API 更稳。

### 安装与快速试跑的稳健方式 ⭐

根据 mjlab 官方仓库说明，
训练通常需要 NVIDIA GPU；
macOS 更适合做评估或轻量体验。

常见入口包括：

```bash
# 不 clone 仓库，直接体验 demo
uvx --from mjlab --refresh demo
```

```bash
# 从 PyPI 安装；具体依赖以官方安装页为准
pip install mjlab
```

```bash
# 训练示例任务；任务名随版本可能变化
uv run train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 4096
```

这些命令的教学意义大于记忆意义。

你要观察的是：

1. 环境数量如何通过命令行覆盖。
2. 训练配置和环境配置是否分离。
3. 可视化评估是否能加载训练中的 checkpoint。
4. 失败时日志能否指出是安装、模型、GPU 还是任务配置问题。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念 | 把 MJX、Warp、mjlab 当成同一层东西 | 选型混乱 | 后端、框架、任务集层级不同 | 先问自己要物理后端还是环境框架 |
| 编程 | 直接复制 IsaacLab 配置不检查物理 | 能跑但步态怪异 | 接触和执行器语义不同 | 先做站立、随机动作、零动作 sanity check |
| 思维 | 认为 API 迁移等于 sim2real 迁移 | 训练曲线正常，部署失败 | sim2real gap 在物理和传感器层 | 增加 MuJoCo CPU 回放与参数消融 |
| 编程 | 在 CPU/GPU 间频繁拷贝观测 | GPU 利用率低 | Python 端同步破坏批量吞吐 | 让 rollout 主路径留在设备端 |

### 练习 ⭐

1. 画出你熟悉的 IsaacLab 或 legged_gym 环境的 `step()` 数据流，
   标注哪些部分可以对应到 Observation/Action/Reward/Termination/Event。
2. 选择一个已有 MJCF 机器人模型，
   列出迁移前必须核对的 10 个物理字段：
   质量、惯量、关节轴、限位、阻尼、摩擦、执行器、传感器、接触 geom、默认姿态。
3. 解释为什么“训练需要 NVIDIA GPU”和“策略评估可在 CPU 或其他平台上运行”并不矛盾。

---

## S3-B.3 JAX 自动向量化直觉：把一个环境提升成一批环境 ⭐⭐⭐

### 动机：不要把 `vmap` 理解成语法糖 ⭐⭐

很多 C++/Python 工程师第一次看到 `jax.vmap`，
会把它理解成自动写了一个 for 循环。

这种理解只对了一半。

`vmap` 的表面效果确实像：

```python
for i in range(num_envs):
    out[i] = step(in[i])
```

但编译执行模型完全不同。

Python `for` 循环是一行行调度。

JAX `vmap` 是把函数变成带批量轴的数组程序。

再配合 `jit`，
这个数组程序可以被 XLA 编译成设备端执行图。

| 写法 | 执行位置 | 调度粒度 | 典型瓶颈 |
|------|----------|----------|----------|
| Python for | Python 解释器逐次调度 | 每个环境一次 | Python overhead |
| NumPy 批量数组 | CPU 向量化 | 每个数组操作一次 | CPU 内存带宽 |
| JAX `vmap+jit` | GPU/TPU/CPU 后端 | 编译后的计算图 | 静态形状、编译时间 |
| MJX/Warp | 物理 kernel | 接触/约束/积分 kernel | 接触复杂度、显存 |

> **本质洞察**：`vmap` 的关键不是“少写循环”，
> 而是让编译器看见批量维度。
> 编译器只有看见整个批量计算图，
> 才能把调度、内存布局和 kernel 融合做对。

### 函数式 step 的三个约束 ⭐⭐

JAX 风格环境通常要求 step 是接近纯函数的。

纯函数不是道德要求，
而是编译和自动向量化的工程要求。

一个环境 step 应该接近：

```python
next_state, obs, reward, done, info = step(state, action, key)
```

而不是：

```python
env.internal_state += ...
env.random_generator.seed(...)
env.viewer.render(...)
```

三条约束最重要。

| 约束 | 含义 | 不满足时的后果 |
|------|------|----------------|
| 显式状态 | 所有会变的东西都在 state 中 | 编译器不知道依赖关系 |
| 显式随机数 | 随机 key 作为输入输出管理 | 环境同步或不可复现 |
| 固定形状 | 每一步数组形状不变 | 反复编译或直接报错 |

### 用 `vmap` 写单环境到批量环境 ⭐⭐

下面代码演示单环境函数如何被提升。

```python
import jax
import jax.numpy as jnp


def single_step(pos, vel, action, dt):
    # 单环境：pos/vel/action 都是 [2]
    action = jnp.clip(action, -1.0, 1.0)
    next_vel = vel + action * dt
    next_pos = pos + next_vel * dt
    return next_pos, next_vel


# in_axes 指定哪些参数带批量轴
# pos、vel、action 是 [B, 2]，dt 是标量，所以 dt 的 in_axes=None
batched_step = jax.vmap(single_step, in_axes=(0, 0, 0, None))


def rollout(pos, vel, actions, dt):
    # actions 形状 [T, B, 2]
    # scan 沿时间轴循环，vmap 沿环境轴并行
    def body(carry, action_t):
        pos_t, vel_t = carry
        next_pos, next_vel = batched_step(pos_t, vel_t, action_t, dt)
        return (next_pos, next_vel), next_pos

    (final_pos, final_vel), pos_history = jax.lax.scan(body, (pos, vel), actions)
    return final_pos, final_vel, pos_history
```

这段代码体现了两个维度。

1. `vmap` 管环境维度 $B$。
2. `scan` 管时间维度 $T$。

在 PPO 中，
rollout buffer 的自然形状就是：

```text
obs:      [T, B, obs_dim]
actions:  [T, B, action_dim]
rewards:  [T, B]
dones:    [T, B]
values:   [T, B]
log_prob: [T, B]
```

很多训练 bug 都来自把 $T$ 和 $B$ 搞反。

`[T, B, ...]` 便于按时间计算 GAE。

`[B, T, ...]` 便于按环境查看单条 episode。

更新 PPO 时通常会 reshape 成：

```json
[T * B, ...]
```

### PRNG：为什么不能用全局随机数 ⭐⭐⭐

JAX 的随机数是显式 key。

这让初学者觉得麻烦，
但批量仿真中它非常重要。

如果所有环境共用一个隐式随机数，
你很难保证：

1. 每个环境 reset 独立。
2. 每次运行可复现。
3. 不同设备、多 GPU 或不同编译顺序下结果仍可追踪。

正确做法是为每个环境分配 key。

```python
def reset_one(key):
    # 单环境 reset，返回一个随机初始状态
    q = jax.random.uniform(key, (3,), minval=-0.1, maxval=0.1)
    return q


def reset_many(master_key, num_envs):
    # 每个环境使用独立 key
    keys = jax.random.split(master_key, num_envs)
    return jax.vmap(reset_one)(keys)
```

进一步，
每一步事件随机化也要 split。

```python
def step_with_random_push(state, action, key):
    key_push, key_next = jax.random.split(key)
    push = jax.random.uniform(key_push, state.vel.shape, minval=-0.2, maxval=0.2)
    next_vel = state.vel + action + push
    return state.replace(vel=next_vel), key_next
```

如果不这样做，
你会看到很怪的训练现象：
所有机器人同一时刻被推一下，
同一时刻摔倒，
同一时刻 reset。

曲线看上去很平滑，
但样本多样性很差。

### 静态形状：接触仿真的隐藏约束 ⭐⭐⭐

JAX 编译喜欢静态形状。

但机器人接触数量天然是动态的。

走路时一会儿两只脚接触，
一会儿四只脚接触，
摔倒时身体和地面可能产生很多接触。

如果接触数组按“实际接触数”变化，
形状就会变化。

批量 GPU 仿真通常采用预分配策略：

```text
contact_buffer: [num_envs, max_contacts, contact_dim]
active_contact_mask: [num_envs, max_contacts]
```

这样每一步形状固定。

真正有效的接触用 mask 标识。

反事实推理：
如果接触数组每步动态增长，
编译器无法为一个固定图生成高效 kernel。

更糟的是，
一个环境摔倒产生很多接触，
会改变整个 batch 的内存布局。

因此“最大接触数”不是小细节，
而是性能和稳定性的边界条件。

### `jit` 的第一次慢与后续快 ⭐⭐

JAX/MJX/Warp 风格代码经常出现：
第一次运行很慢，
后面很快。

这是编译和图捕获导致的。

可以这样理解。

```text
第一次:
  Python 函数
    -> tracing
    -> 生成计算图
    -> 编译 kernel / 捕获图
    -> 执行

后续:
  复用已编译图
    -> 直接执行
```

因此评估性能时不能只看第一步。

应该分开记录：

| 指标 | 含义 | 用途 |
|------|------|------|
| compile time | 首次 JIT/图捕获时间 | 判断开发迭代成本 |
| warmup steps | 预热步数 | 排除一次性开销 |
| steady SPS | 稳态 steps per second | 衡量训练吞吐 |
| reset SPS | 含 reset 的吞吐 | 发现终止过频问题 |

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 在 `jit` 函数里用 Python list 追加日志 | 编译失败或退回慢路径 | Python 副作用不可追踪 | 返回数组指标，外部汇总 |
| 编程 | 每次 step 改变数组形状 | 反复编译 | JAX 需要静态形状 | 预分配 + mask |
| 概念 | 把 `vmap` 当成多线程 | 期待任意 Python 代码都能加速 | `vmap` 向量化数组程序，不加速 Python 副作用 | 先函数式化 step |
| 思维 | 第一帧慢就判断性能差 | 误判后端吞吐 | 编译/捕获是一次性开销 | 分开记录 warmup 和 steady |

### 练习 ⭐

1. 修改 `rollout()`，
   让它同时返回 `reward_history` 和 `done_history`，
   并保持形状 `[T, B]`。
2. 写一个 `reset_where_done(state, done, key)`：
   对 `done=True` 的环境重置，
   对其他环境保持状态。
3. 解释为什么接触数量变化会影响 JAX/MJX 性能，
   并给出一个 mask buffer 的设计。

---
## S3-B.4 mjlab 风格环境：把 MDP 拆成可维护的 Manager ⭐⭐⭐

### 动机：环境不是一个 `step()` 函数那么简单 ⭐

强化学习中的 MDP 通常写成五元组：

$$
\mathcal{M} = (\mathcal{S}, \mathcal{A}, P, R, \gamma)
$$

其中：

| 符号 | 含义 | mjlab 风格对应 |
|------|------|----------------|
| $\mathcal{S}$ | 状态空间 | MuJoCo/MJX/Warp 内部状态 |
| $\mathcal{A}$ | 动作空间 | ActionManager |
| $P$ | 状态转移 | Simulation backend + EventManager |
| $R$ | 奖励函数 | RewardManager |
| $\gamma$ | 折扣因子 | RL 配置 |

实际机器人任务还需要额外元素。

1. 观测函数 $O(s)$：
   策略不能直接看到完整状态。
2. 终止函数 $D(s)$：
   episode 什么时候结束。
3. 命令采样 $C(\rho)$：
   velocity tracking 的目标速度从哪里来。
4. 域随机化 $E(s,\theta,\rho)$：
   训练时物理和传感器如何变化。
5. 日志函数 $M(s,a,r)$：
   如何知道训练失败在哪里。

Manager-based API 的设计就是把这些函数拆开。

```text
环境 step 的逻辑顺序:

1. ActionManager
   策略输出 action -> 执行器目标 ctrl

2. Simulation
   ctrl -> 物理步进 decimation 次

3. ObservationManager
   仿真状态 -> policy obs / critic obs

4. RewardManager
   状态、动作、命令 -> reward terms

5. TerminationManager
   状态、时间、接触 -> done mask

6. EventManager
   reset 或 interval -> 随机化状态和参数

7. Metrics/Recorder
   保存 episode return、速度误差、摔倒率等
```

### 为什么 Manager 分离比手写大函数更适合教学 ⭐⭐

在单环境调试中，
手写一个 300 行 `step()` 也能工作。

但学生常会遇到这种情况：

训练不收敛。

然后开始同时改观测、奖励、动作尺度和终止条件。

过两小时曲线变好了，
却不知道是哪一个改动起作用。

这种方式无法积累判断力。

Manager 分离带来实验隔离。

| 想研究的问题 | 只应该改哪里 | 不应该顺手改哪里 |
|--------------|--------------|------------------|
| 策略是否缺少速度信息 | ObservationManager | RewardManager |
| 动作是否太大导致抖动 | ActionManager | TerminationManager |
| 速度跟踪是否权重太低 | RewardManager | Scene/Entity |
| 摔倒是否终止太严格 | TerminationManager | ActionManager |
| sim2real 是否缺少摩擦扰动 | EventManager | Policy 网络 |

这与多模态 MPC 章节中的思想一致：
复杂系统必须让每类约束和代价有清晰归属。

如果所有逻辑都混在一起，
调试就会从“定位问题”变成“猜测问题”。

### 一个 Go2 velocity tracking 任务的最小配置形态 ⭐⭐

下面代码是 mjlab 风格的教学配置。

它展示结构和职责，
具体类名和字段以当前安装版本文档为准。

```python
# envs/go2_velocity/scene_cfg.py
# 这段代码表达配置结构；实际项目中应对照 mjlab 当前版本导入路径。

from dataclasses import dataclass, field


@dataclass
class ActuatorCfg:
    # 用正则或显式列表选择关节
    joint_names: list[str]
    # 电机力矩上限，训练时动作不能绕过这个限制
    effort_limit: float
    # 位置控制 PD 增益
    stiffness: float
    damping: float


@dataclass
class EntityCfg:
    # MJCF 模型路径；真实项目中优先使用已检查过惯量和接触的模型
    mjcf_path: str
    # 一个机器人可以有多组执行器，例如 legs、arm、gripper
    actuators: dict[str, ActuatorCfg]


@dataclass
class TerrainCfg:
    # plane 用于第一阶段 sanity check
    terrain_type: str = "plane"
    # 地形课程打开后再增加 heightfield 或随机台阶
    difficulty: float = 0.0


@dataclass
class Go2SceneCfg:
    robot: EntityCfg = field(
        default_factory=lambda: EntityCfg(
            mjcf_path="mujoco_menagerie/unitree_go2/go2.xml",
            actuators={
                "legs": ActuatorCfg(
                    joint_names=["FR_.*", "FL_.*", "RR_.*", "RL_.*"],
                    effort_limit=23.7,
                    stiffness=25.0,
                    damping=0.5,
                ),
            },
        )
    )
    terrain: TerrainCfg = field(default_factory=TerrainCfg)
    # 批量环境数量是训练吞吐和显存占用的主要旋钮
    num_envs: int = 4096
    # 环境间距用于可视化和避免几何重叠
    env_spacing: float = 2.0
```

这段代码还没有定义 RL 任务。

它只回答：

1. 机器人从哪里加载。
2. 哪些关节由哪些执行器控制。
3. 地形是什么。
4. 一次并行多少个环境。

这就是 Scene 层的边界。

### MDP 配置骨架 ⭐⭐

```python
# envs/go2_velocity/env_cfg.py
# 教学版配置骨架：展示 Manager 的职责分离。

from dataclasses import dataclass, field


@dataclass
class TermCfg:
    # func 是一个函数句柄或函数名，用于计算观测/奖励/终止等
    func: object
    # weight 只对奖励项有意义
    weight: float = 1.0
    # params 保存该 term 的局部参数
    params: dict = field(default_factory=dict)


@dataclass
class Go2VelocityEnvCfg:
    scene: Go2SceneCfg = field(default_factory=Go2SceneCfg)

    # control_dt = physics_dt * decimation
    # 例如物理 0.002s，decimation=10，则策略频率 50Hz。
    physics_dt: float = 0.002
    decimation: int = 10
    episode_length_s: float = 20.0

    observations: dict = field(default_factory=dict)
    actions: dict = field(default_factory=dict)
    rewards: dict = field(default_factory=dict)
    terminations: dict = field(default_factory=dict)
    events: dict = field(default_factory=dict)
    commands: dict = field(default_factory=dict)
```

注意 `decimation`。

这是仿真训练中非常关键的参数。

策略通常不需要每个物理积分步都输出一次动作。

如果物理步长是 0.002s，
每 10 个物理步更新一次策略，
策略频率就是 50Hz。

这接近很多腿足策略部署时的控制频率。

反事实推理：
如果策略频率过高，
训练时动作会学到依赖仿真细节的高频抖动。

如果策略频率过低，
机器人来不及响应扰动，
尤其在奔跑、跳跃和快速转向任务中会失败。

### Manager 的调度顺序与时间尺度 ⭐⭐⭐

不同 Manager 不一定以同一频率运行。

| 模块 | 典型频率 | 说明 |
|------|----------|------|
| 物理积分 | 500Hz 到 2000Hz | 由 `physics_dt` 决定 |
| 策略推理 | 20Hz 到 100Hz | 由 `decimation` 决定 |
| 奖励计算 | 策略频率 | 每次 action 后计算一次 |
| 终止检查 | 策略频率或物理频率 | 摔倒检测通常策略频率足够 |
| reset 事件 | episode 结束时 | 随机化初始状态 |
| interval 事件 | 每隔若干秒 | 外推、摩擦变化、命令重采样 |
| 日志汇总 | 每个 rollout 或 episode | 不应阻塞主训练循环 |

把这些频率混在一起是常见 bug。

例如每个物理步都重采样速度命令，
策略会看到目标速度疯狂跳变。

再如每个策略步都随机化质量，
系统参数变成高速时变系统，
策略学到的不是鲁棒控制，
而是在噪声中挣扎。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 在 RewardManager 中修改状态 | 奖励开关影响动力学 | 奖励函数应只读状态 | 状态改变放进 Event 或 Simulation |
| 概念 | 把命令采样写进观测函数 | 观测调用次数改变会改变任务 | 观测应是读操作 | 命令由 Command/Event 管理 |
| 思维 | 一次性打开复杂地形和强随机化 | 训练早期完全不动 | 任务难度超过探索能力 | 平地 sanity check 后逐步加难 |
| 编程 | 忽略 `decimation` 对动作频率的影响 | 策略抖动或迟钝 | 物理频率和策略频率混淆 | 明确 `control_dt = physics_dt * decimation` |

### 练习 ⭐

1. 写出一个机械臂 reaching 任务的 Manager 划分：
   哪些属于 Observation，
   哪些属于 Reward，
   哪些属于 Termination。
2. 对 Go2 velocity tracking，
   设 `physics_dt=0.002`、`decimation=10`、episode 20s，
   计算每个 episode 有多少策略步和物理步。
3. 设计一个命令重采样事件：
   每 5 到 10 秒随机更换目标线速度和角速度。

---

## S3-B.5 观测设计：策略应该看见什么 ⭐⭐⭐

### 动机：观测不是”数据越多越好” ⭐

观测设计决定策略的输入信息。

对四足 velocity tracking 来说，
一个直觉错误是把所有 MuJoCo 状态都喂给策略。

例如：

```text
qpos + qvel + contact + actuator + world position + full body poses
```

这看起来信息完整，
但未必是好观测。

策略输入应满足四个原则。

| 原则 | 含义 | 四足例子 |
|------|------|----------|
| 可部署 | 真机也能获得或估计 | IMU、关节编码器、命令 |
| 坐标一致 | 不随世界原点变化 | body frame 速度、投影重力 |
| 足够马尔可夫 | 能判断下一步动作 | 关节位置、速度、上一步动作 |
| 不泄漏答案 | 不包含训练专属信息 | 不给未来地形、不直接给 reward |

回顾腿足简化模型章节：
SRBD 关心基座速度、角速度、姿态和接触力，
而不关心世界中绝对 $x,y$ 坐标。

RL locomotion 的观测也遵循类似思想。

策略要知道身体相对自身怎么动，
而不是知道“训练场地第 12 米处有什么”。

### 典型 policy observation ⭐⭐

四足速度跟踪常见观测如下。

| 观测项 | 形状 | 坐标系 | 作用 |
|--------|------|--------|------|
| base angular velocity | 3 | body | 稳定姿态、转向 |
| projected gravity | 3 | body | 估计 roll/pitch |
| velocity command | 3 | command frame | 给出目标 $v_x,v_y,\omega_z$ |
| joint position error | 12 | joint | 知道腿在哪 |
| joint velocity | 12 | joint | 抑制高频和判断运动趋势 |
| last action | 12 | action | 建立动作平滑记忆 |
| optional base linear velocity | 3 | body | 仿真可用，真机需估计 |

很多真实部署策略不直接使用 base linear velocity，
因为真机线速度估计更难。

但教学阶段可以先包含它，
再做消融。

这能帮助学生理解“仿真最优”和“部署稳健”不是同一个目标。

### 投影重力为什么重要 ⭐⭐

IMU 可以给出身体姿态或重力方向。

在足式策略中，
常用 projected gravity：

$$
g_b = R_{wb}^\top \begin{bmatrix}0 \\ 0 \\ -1\end{bmatrix}
$$

其中 $R_{wb}$ 是 body 到 world 的旋转矩阵。

$g_b$ 表示世界重力方向在机体系中的坐标。

它有三个好处。

1. 避免欧拉角奇异和角度 wrap。
2. 直接反映身体倾斜方向。
3. 与 IMU 观测关系自然。

当机器人水平站立时：

$$
g_b \approx [0, 0, -1]
$$

向左侧倾时，
$g_b$ 的横向分量会变化。

策略不需要知道 roll/pitch 的具体参数化，
只需要知道重力从身体哪个方向“指过来”。

### 观测归一化 ⭐⭐

神经网络对尺度很敏感。

如果一个输入是 $0.01$ 量级，
另一个输入是 $50$ 量级，
早期训练中大尺度项会主导梯度。

因此观测通常做缩放。

| 观测 | 原始范围 | 常见缩放目标 | 说明 |
|------|----------|--------------|------|
| 角速度 rad/s | 几到十几 | 约 $[-1,1]$ | 乘 0.25 或类似尺度 |
| 关节速度 rad/s | 几十 | 约 $[-1,1]$ | 乘 0.05 左右 |
| 关节位置偏差 rad | 小于 1 | 保持或轻缩放 | 相对默认姿态 |
| 速度命令 m/s | 0 到 2 | 约 $[-1,1]$ | 按最大命令归一 |
| 上一步动作 | 已归一 | 不再缩放 | 动作本身通常在 [-1,1] |

归一化不是为了让数字好看。

它决定优化地形。

PPO 的 policy 网络第一层看到的是观测向量。

如果各维尺度差异很大，
网络需要先花容量学习尺度变换。

这会降低样本效率。

### privileged observation 与 policy observation ⭐⭐⭐

训练时可以给 critic 更多信息。

这叫 privileged learning。

policy observation 是部署时可用的信息。

critic observation 可以包含仿真内部信息，
因为 critic 只在训练中估计 value。

| 观测集合 | 给谁用 | 可包含什么 | 部署时是否需要 |
|----------|--------|------------|----------------|
| policy obs | actor | IMU、关节、命令、历史动作 | 需要 |
| critic obs | critic | base lin vel、地形高度、随机化参数 | 不需要 |
| logging obs | 日志系统 | reward terms、contact、能耗 | 不需要 |

> **本质洞察**：privileged critic 不是让策略作弊。
> 它让训练阶段的 value 估计更准，
> 从而降低 policy gradient 的方差。
> 只要 actor 输入不包含部署不可得信息，
> 策略执行时仍然是可部署的。

### MJLab 风格观测配置示例 ⭐⭐

```python
# 观测配置示例：字段名以当前 mjlab 版本为准。

observations = {
    "policy": {
        # base_ang_vel：机体系角速度，帮助策略抑制翻滚和俯仰
        "base_ang_vel": {
            "func": "base_ang_vel",
            "scale": 0.25,
            "noise": 0.02,
        },
        # projected_gravity：重力在机体系的方向，比欧拉角更连续
        "projected_gravity": {
            "func": "projected_gravity",
            "scale": 1.0,
            "noise": 0.01,
        },
        # velocity_commands：目标 vx、vy、yaw rate
        "velocity_commands": {
            "func": "velocity_commands",
            "scale": 1.0,
        },
        # joint_pos_rel：相对默认关节角，避免网络记忆绝对零点
        "joint_pos": {
            "func": "joint_pos_rel",
            "scale": 1.0,
            "noise": 0.01,
        },
        # joint_vel：关节速度通常量级较大，需要缩放
        "joint_vel": {
            "func": "joint_vel",
            "scale": 0.05,
            "noise": 1.5,
        },
        # last_action：让无记忆 MLP 获得一点动作历史
        "last_action": {
            "func": "last_action",
            "scale": 1.0,
        },
    },
    "critic": {
        # critic 可以看到更多仿真内部信息，帮助 value 学得更稳
        "base_lin_vel": {"func": "base_lin_vel", "scale": 2.0},
        "feet_contact": {"func": "feet_contact_state", "scale": 1.0},
        "terrain_heights": {"func": "height_scan", "scale": 1.0},
    },
}
```

### 观测噪声的设计 ⭐⭐⭐

观测噪声是域随机化的一部分。

它不应该随便加。

| 噪声项 | 物理来源 | 设计建议 |
|--------|----------|----------|
| IMU 角速度噪声 | 陀螺仪噪声、振动 | 小幅高频 |
| projected gravity 噪声 | 姿态估计误差 | 小幅，避免破坏重力方向 |
| 关节位置噪声 | 编码器量化、零点误差 | 小到中等 |
| 关节速度噪声 | 差分估计放大噪声 | 可比位置噪声大 |
| 命令噪声 | 通信/上层规划变化 | 通常不加随机噪声，改用命令课程 |

如果噪声过大，
策略会学得很保守。

如果完全无噪声，
策略可能依赖仿真中不存在的精确信息。

正确做法是：
先在无噪声环境确认 MDP 能学，
再逐步加入传感器噪声。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念 | 把 world position 放进 policy obs | 换起点性能下降 | 策略记住场地位置 | 使用 body frame 速度和相对量 |
| 编程 | 拼接观测时顺序训练和部署不一致 | ONNX 回放行为完全错 | 网络输入语义错位 | 固定 obs spec 并导出检查表 |
| 思维 | 观测维度越多越好 | 收敛慢、泛化差 | 信息泄漏和噪声放大 | 只给任务必要且可部署的信息 |
| 编程 | 观测缩放只在训练用，部署忘记 | 真机动作异常 | policy 输入分布漂移 | 缩放参数随模型一起导出 |

### 练习 ⭐

1. 为机械臂 reaching 任务设计 policy obs 和 critic obs，
   标出哪些信息部署不可得。
2. 删除 Go2 policy obs 中的 `last_action`，
   预测训练曲线和动作平滑性会如何变化。
3. 设计一个观测导出清单：
   每一维记录名称、单位、缩放、噪声、部署来源。

---

## S3-B.6 动作设计：策略输出不是电机力矩的自由通行证 ⭐⭐⭐

### 动机：动作空间决定学习难度和部署风险 ⭐

在 MuJoCo 中，
`d.ctrl[:]` 可以代表不同含义。

它可能是力矩，
也可能是位置执行器目标，
还可能是速度目标或肌肉激活。

RL 策略的 action 也有多种语义。

| 动作语义 | 形式 | 优点 | 风险 |
|----------|------|------|------|
| 关节力矩 | $\tau = a \tau_{\max}$ | 表达能力强 | 探索危险，sim2real 难 |
| 绝对关节位置 | $q^{des}=a$ | 简单 | 容易输出离谱姿态 |
| 相对默认位置 | $q^{des}=q_0+s a$ | 稳定，常用 | 动作范围受默认姿态影响 |
| 相对当前关节 | $q^{des}=q+s a$ | 动作连续 | 漂移风险 |
| 足端目标 | $p_{foot}^{des}$ | 接近控制结构 | 需要 IK/WBC |

腿足 locomotion 中常用“相对默认关节位置”。

$$
q^{des} = q^{default} + s_a \cdot a
$$

其中 $a \in [-1,1]^{n_j}$，
$s_a$ 是动作尺度。

低层执行器再用 PD：

$$
\tau = k_p(q^{des}-q) - k_d \dot{q}
$$

最后经过力矩限幅。

这种动作空间有三个好处。

1. 策略初始输出接近零时，
   机器人保持默认站姿附近。
2. 动作尺度直接限制最大关节偏移。
3. PD 控制提供局部稳定性，
   减少纯力矩探索的危险。

### 动作尺度不是小超参数 ⭐⭐

如果 $s_a$ 太小，
机器人迈不开腿。

如果 $s_a$ 太大，
策略一开始就能把腿甩到极限。

动作尺度影响可探索动作集合：

$$
q^{des}_i \in [q^{default}_i - s_a,\; q^{default}_i + s_a]
$$

还影响等效力矩：

$$
|\tau_i| \approx k_{p,i} s_a
$$

在有力矩限幅时，
过大的 $k_p s_a$ 会频繁撞限幅。

力矩限幅后，
策略以为自己输出了很大动作，
但电机实际只执行饱和值。

这会让学习信号变钝。

| 现象 | 可能动作问题 | 调整方向 |
|------|--------------|----------|
| 原地抖动 | 动作尺度过大或 action rate 惩罚太低 | 降低尺度、增加平滑 |
| 迈不开步 | 动作尺度过小 | 增大尺度或调整默认姿态 |
| 关节长期撞限位 | 默认姿态或尺度不合理 | 检查关节范围和动作映射 |
| reward 初期全为负 | 初始动作导致摔倒 | 降尺度、加站立课程 |

### 动作延迟与 action history ⭐⭐⭐

真机执行有延迟。

动作从策略输出到电机产生力矩，
会经过推理、通信、驱动器、控制环。

如果仿真中没有任何延迟，
策略可能学到依赖瞬时响应的行为。

常见做法：

1. 在观测中加入 `last_action`。
2. 在域随机化中加入 action delay。
3. 在奖励中惩罚 action rate。

动作变化率惩罚：

$$
r_{\Delta a} = - w_{\Delta a}\|a_t - a_{t-1}\|^2
$$

动作二阶变化惩罚：

$$
r_{\Delta^2 a} =
- w_{\Delta^2 a}\|a_t - 2a_{t-1}+a_{t-2}\|^2
$$

第二项更像惩罚 jerk，
可以让动作更平滑。

但过早加太强的平滑惩罚，
会压制探索。

因此课程学习中常见策略是：
先让策略学会站立和跟踪，
再逐步提高平滑和能耗惩罚。

### 动作配置示例 ⭐⭐

```python
actions = {
    "joint_pos": {
        # 策略输出 action in [-1, 1]
        "class": "JointPositionAction",
        # 动作乘以 scale 后加到默认关节位置
        "scale": 0.25,
        # 是否裁剪策略动作，防止异常网络输出破坏仿真
        "clip": 1.0,
        # 默认姿态来自机器人配置或 MJCF keyframe
        "use_default_joint_pos": True,
    }
}
```

部署侧要保存同样的映射。

```python
def policy_action_to_motor_target(action, default_joint_pos, action_scale):
    # 训练和部署必须完全一致：clip -> scale -> add default
    action = action.clip(-1.0, 1.0)
    target_joint_pos = default_joint_pos + action_scale * action
    return target_joint_pos
```

### 正确写法与错误写法 ⭐⭐

```python
# 正确：动作语义明确，训练和部署共用同一函数
def apply_joint_position_action(action, default_q, scale, q_min, q_max):
    # 先裁剪策略输出，避免极端值进入执行器
    action = action.clip(-1.0, 1.0)
    # 从归一化动作映射到目标关节角
    target_q = default_q + scale * action
    # 再根据真实关节限位裁剪目标角
    target_q = target_q.clip(q_min, q_max)
    return target_q
```

```python
# 错误：训练时用相对默认位置，部署时误当成绝对角度
def wrong_deploy_action(action):
    # 问题：action 范围是 [-1, 1]，不是机器人真实关节目标
    return action
```

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 训练和部署动作映射不同 | sim 中会走，回放立刻摔 | action 语义错位 | 导出 action scale 和 default pose |
| 概念 | 直接用力矩动作做初学任务 | 训练不稳、动作暴力 | 探索空间太大且无局部稳定 | 先用 PD 位置动作 |
| 思维 | action rate 惩罚越大越好 | 策略不动或跟踪差 | 平滑惩罚压制必要动作 | 逐步调度平滑权重 |
| 编程 | 忘记力矩限幅 | 仿真电机不现实 | 策略使用真实电机无法输出的力 | 执行器层强制 limit |

### 练习 ⭐

1. 给定 $k_p=25$、动作尺度 $s_a=0.25$，
   估算初始最大等效 PD 力矩。
   再与 Go2 电机力矩上限比较。
2. 设计一个动作延迟 buffer：
   随机延迟 0 到 2 个 policy step，
   并保持批量形状 `[B, delay_max+1, action_dim]`。
3. 比较关节位置动作和足端目标动作：
   哪个更容易训练？
   哪个更接近 WBC？

---

## S3-B.7 奖励与终止：把控制目标翻译成可学习信号 ⭐⭐⭐

### 动机：奖励不是 MPC 代价的简单复制 ⭐

多模态 MPC 章节中，
代价函数通常写成：

$$
J = \int_0^T
\|x-x^{ref}\|_Q^2 + \|u\|_R^2
\,dt
$$

RL 奖励也会包含跟踪和正则项，
但两者并不等价。

MPC 每次求解一个短 horizon 的优化问题。

RL 策略是在大量交互中学习一个反馈函数。

因此奖励不仅要表达目标，
还要提供学习梯度和探索引导。

| 目标 | MPC 中的写法 | RL 中的写法 | 差异 |
|------|--------------|-------------|------|
| 速度跟踪 | 二次代价 | 指数奖励或负误差 | RL 常需正奖励引导 |
| 姿态稳定 | 状态权重 | roll/pitch 惩罚或终止 | 终止会改变采样分布 |
| 能耗 | 控制代价 | torque/power penalty | 权重过大导致不动 |
| 平滑 | 控制变化率代价 | action rate penalty | 通常逐步加强 |
| 接触模式 | 接触约束/模式表 | air time/contact reward | 奖励会塑造步态 |

### 速度跟踪奖励 ⭐⭐

常见写法是指数核：

$$
r_{v_{xy}} =
\exp\left(
-\frac{\|v_{xy}^{body} - v_{xy}^{cmd}\|^2}{\sigma_v^2}
\right)
$$

角速度跟踪：

$$
r_{\omega_z} =
\exp\left(
-\frac{(\omega_z^{body} - \omega_z^{cmd})^2}{\sigma_\omega^2}
\right)
$$

为什么不用简单负二次误差？

负二次误差可以工作，
但初期策略很差时，
所有样本都很负。

指数奖励把“接近目标”压到 $0$ 到 $1$ 之间，
能更清楚地区分好样本和坏样本。

这类似控制中的核函数：
误差小的时候梯度明显，
误差大到离谱时奖励接近 0，
不让极端坏样本主导更新。

### 正则项不是越多越好 ⭐⭐

典型正则项如下。

| 奖励项 | 形式 | 目的 | 过强后果 |
|--------|------|------|----------|
| z 方向速度惩罚 | $-\|\dot z\|^2$ | 不要跳动 | 不敢跨障碍 |
| roll/pitch 角速度惩罚 | $-\|\omega_{xy}\|^2$ | 稳定身体 | 转身迟钝 |
| 力矩惩罚 | $-\|\tau\|^2$ | 降能耗 | 不愿迈步 |
| 功率惩罚 | $-\sum|\tau_i\dot q_i|$ | 降机械功率 | 动作保守 |
| 关节加速度惩罚 | $-\|\ddot q\|^2$ | 平滑 | 学习变慢 |
| action rate | $-\|a_t-a_{t-1}\|^2$ | 平滑策略输出 | 命令跟踪变差 |

> **本质洞察**：奖励设计不是把所有好性质加起来。
> 它是在训练早期可探索性和最终行为质量之间做权衡。
> 早期奖励要让策略找到“能动起来”的通道，
> 后期奖励才逐步要求平滑、节能、优雅。

### 足端腾空时间奖励 ⭐⭐⭐

四足 locomotion 中常用 air time 奖励鼓励迈步。

简化形式：

$$
r_{air} =
\sum_{foot}
\mathbb{1}[\text{first contact}]
\cdot
\max(t_{air} - t_{min}, 0)
$$

直觉是：
脚离地一段时间后再接触，
说明形成了摆腿。

但这个奖励很容易被误用。

如果权重过高，
策略可能为了腾空时间故意跳。

如果没有速度命令门控，
站立命令下也会鼓励迈脚。

更稳健的写法是：
只有命令速度超过阈值时才启用 air time。

```python
def feet_air_time_reward(first_contact,
                         air_time,
                         cmd_vel,
                         threshold=0.5,
                         lin_threshold=0.05,
                         yaw_threshold=0.15):
    # first_contact: [B, num_feet]，本步是否刚接触地面
    # air_time: [B, num_feet]，每只脚已经离地多久
    # cmd_vel: [B, 3]，目标 vx, vy, yaw rate
    # 只有命令足够大时才鼓励迈步，避免站立时乱动。
    # 线速度和 yaw rate 单位不同，不能直接平方相加。
    lin_speed = jnp.linalg.norm(cmd_vel[:, :2], axis=-1)
    yaw_rate = jnp.abs(cmd_vel[:, 2])
    moving = (lin_speed > lin_threshold) | (yaw_rate > yaw_threshold)
    reward_per_foot = first_contact * jnp.maximum(air_time - threshold, 0.0)
    reward = jnp.sum(reward_per_foot, axis=-1)
    return reward * moving.astype(reward.dtype)
```

### 终止条件的教学意义 ⭐⭐

终止条件决定哪些状态被认为 episode 结束。

常见终止：

| 终止项 | 判断方式 | 作用 |
|--------|----------|------|
| timeout | 步数达到上限 | 正常结束 |
| bad orientation | projected gravity 或 roll/pitch 超限 | 摔倒结束 |
| base height too low | 基座高度低于阈值 | 趴地结束 |
| illegal contact | 非脚部 body 接触地面 | 防止用身体蹭地 |
| joint limit | 关节接近硬限位 | 防止不现实姿态 |

终止过严会导致探索不足。

终止过松会让策略在“已摔倒状态”中继续采样。

这会污染 rollout。

判断终止是否合适，
看两个指标。

1. early termination rate：
   训练前期高很正常，
   中后期应下降。
2. timeout rate：
   策略稳定后应上升。

如果 early termination 长期接近 100%，
先不要调 PPO。

应检查动作尺度、默认姿态、初始 reset 和终止阈值。

### 奖励配置示例 ⭐⭐

```python
rewards = {
    # 主任务：线速度跟踪
    "track_lin_vel_xy": {
        "func": "track_lin_vel_xy_exp",
        "weight": 1.0,
        "params": {"std": 0.25},
    },
    # 主任务：yaw 角速度跟踪
    "track_ang_vel_z": {
        "func": "track_ang_vel_z_exp",
        "weight": 0.5,
        "params": {"std": 0.25},
    },
    # 不希望身体上下跳
    "lin_vel_z_l2": {
        "func": "lin_vel_z_l2",
        "weight": -2.0,
    },
    # 不希望 roll/pitch 方向高速旋转
    "ang_vel_xy_l2": {
        "func": "ang_vel_xy_l2",
        "weight": -0.05,
    },
    # 力矩惩罚权重通常很小，过大会让策略不动
    "joint_torques_l2": {
        "func": "joint_torques_l2",
        "weight": -1.0e-4,
    },
    # 动作变化率惩罚用于平滑策略输出
    "action_rate_l2": {
        "func": "action_rate_l2",
        "weight": -0.01,
    },
    # 足端腾空时间奖励需结合命令门控
    "feet_air_time": {
        "func": "feet_air_time",
        "weight": 0.125,
        "params": {"threshold": 0.5},
    },
}
```

### 终止配置示例 ⭐⭐

```python
terminations = {
    "time_out": {
        "func": "time_out",
        # time_out=True 表示正常截断，不应当当成失败惩罚
        "time_out": True,
    },
    "bad_orientation": {
        "func": "bad_orientation",
        # 过小会让早期探索频繁中断，过大又会保留摔倒样本
        "params": {"limit_angle": 0.7},
    },
    "base_height": {
        "func": "base_height_below",
        "params": {"minimum_height": 0.18},
    },
    "illegal_contact": {
        "func": "illegal_contact",
        "params": {"allowed_body_regex": ".*foot.*"},
    },
}
```

### reward terms 的日志化 ⭐⭐

只记录总 reward 不够。

总 reward 是多个 term 的和。

如果总 reward 不涨，
你需要知道哪一项坏了。

建议每个 rollout 记录：

| 日志名 | 含义 |
|--------|------|
| `reward/track_lin_vel_xy` | 线速度跟踪项均值 |
| `reward/track_ang_vel_z` | 角速度跟踪项均值 |
| `reward/action_rate_l2` | 动作平滑惩罚 |
| `reward/torque_l2` | 力矩惩罚 |
| `episode/termination_bad_orientation` | 姿态失败比例 |
| `episode/timeout_rate` | 正常结束比例 |
| `task/command_speed_mean` | 命令分布是否合理 |

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念 | 把所有惩罚一开始开很大 | 策略站着不动 | 最优早期探索被压制 | 先主任务，后逐步加正则 |
| 编程 | time-out 当成失败 done | value target 偏差 | 截断和失败语义不同 | 区分 timeout 与 terminal |
| 思维 | 总 reward 不涨就调学习率 | 越调越乱 | 可能是某个 reward term 错 | 先看分项日志 |
| 编程 | 站立命令也给 air time 奖励 | 原地踏步 | 奖励和命令条件冲突 | 用命令速度门控 |

### 练习 ⭐

1. 设计一个能耗奖励：
   $r=-\sum_i|\tau_i\dot q_i|$，
   并说明为什么它不应该在训练第一阶段权重过大。
2. 给出三种 bad orientation 的实现方式：
   欧拉角阈值、projected gravity 阈值、四元数夹角。
   比较优缺点。
3. 做一个奖励消融计划：
   固定观测和动作，
   只比较是否加入 action rate、torque、air time。

---

## S3-B.8 域随机化：不是让仿真更像真实，而是让策略更不挑剔 ⭐⭐⭐

### 动机：真实世界不是某一个精确参数点 ⭐

sim2real gap 常被描述成“仿真和真实不一样”。

这句话太粗。

需要分类。

| 类别 | 例子 | 对策略的影响 |
|------|------|--------------|
| 刚体参数 | 质量、质心、惯量 | 步态频率、姿态响应 |
| 接触参数 | 摩擦、恢复系数、接触软硬 | 打滑、弹跳、足端冲击 |
| 执行器 | 力矩上限、PD 增益、延迟、带宽 | 动作迟滞、饱和 |
| 传感器 | 噪声、偏置、丢包、延迟 | 观测分布漂移 |
| 初始状态 | 姿态、关节角、初速度 | 起步鲁棒性 |
| 外界扰动 | 推力、地形、负载变化 | 抗扰能力 |

域随机化的目标不是找到真实参数。

真实机器人参数本身也会变化。

电池电量、温度、地面材料、负载、磨损都会改变系统。

> **本质洞察**：域随机化不是让仿真更逼真，
> 而是扩大训练分布，
> 让真实世界更可能落在策略能处理的分布内部。
> 它解决的是“策略过度挑剔”的问题，
> 不是替代 system identification 的万能方法。

### reset 随机化与 interval 随机化 ⭐⭐

事件按触发时机分两类。

| 类型 | 触发时机 | 适合随机化什么 |
|------|----------|----------------|
| reset event | episode 开始 | 初始姿态、关节角、质量、摩擦、命令 |
| interval event | episode 中间 | 外推、命令重采样、传感器扰动 |

刚体质量不应每 0.2 秒变化一次。

摩擦可以在 reset 时采样，
也可以在不同地形 patch 中变化。

外推则适合 interval。

命令重采样也适合 interval。

### 物理一致的惯量随机化 ⭐⭐⭐

传统写法会分别随机：

1. 质量 $m$。
2. 质心位置 $c$。
3. 惯量矩阵 $I$。

但不是任意组合都物理可行。

刚体惯量必须满足正定性和几何约束。

例如三个主惯量不是随便三个正数。

它们要来自某个质量分布。

如果随机得到不物理的惯量，
仿真可能仍然能跑，
但策略会在不存在的物理世界中训练。

Rucker/Wensing 一类参数化方法的核心思想是：
把质量、质心一阶矩和二阶矩组织到一个满足物理约束的参数空间中，
在这个空间中采样或扰动，
再映射回质量、质心和惯量。

不同库的实现细节可能不同，
但工程原则相同。

| 随机化方式 | 优点 | 风险 |
|------------|------|------|
| 独立随机质量/质心/惯量 | 简单 | 可能不物理 |
| 只随机质量缩放 | 安全 | 覆盖不足 |
| 物理一致参数化 | 约束更合理 | 实现更复杂 |

### 域随机化配置示例 ⭐⭐

```python
events = {
    # reset 时随机化基座姿态和速度
    "reset_base": {
        "func": "reset_root_state_uniform",
        "mode": "reset",
        "params": {
            "pos_range": {"x": [-0.05, 0.05], "y": [-0.05, 0.05], "z": [0.0, 0.03]},
            "roll_pitch_range": [-0.05, 0.05],
            "yaw_range": [-3.14, 3.14],
            "lin_vel_range": [-0.2, 0.2],
            "ang_vel_range": [-0.1, 0.1],
        },
    },
    # reset 时随机化关节角，避免策略只会从标准站姿起步
    "reset_joints": {
        "func": "reset_joints_by_offset",
        "mode": "reset",
        "params": {
            "position_range": [-0.1, 0.1],
            "velocity_range": [-0.1, 0.1],
        },
    },
    # reset 时随机化摩擦；范围不宜一开始过宽
    "randomize_friction": {
        "func": "randomize_rigid_body_material",
        "mode": "reset",
        "params": {
            "friction_range": [0.6, 1.2],
        },
    },
    # reset 时随机化质量和惯量；实际字段以当前版本为准
    "randomize_body_inertia": {
        "func": "randomize_rigid_body_inertia_consistent",
        "mode": "reset",
        "params": {
            "mass_scale_range": [0.8, 1.2],
            "com_offset_range": [-0.03, 0.03],
        },
    },
    # episode 中间施加外推，训练抗扰恢复
    "push_robot": {
        "func": "push_by_setting_velocity",
        "mode": "interval",
        "interval_range_s": [8.0, 15.0],
        "params": {
            "velocity_range": {"x": [-0.5, 0.5], "y": [-0.5, 0.5]},
        },
    },
}
```

### 随机化课程 ⭐⭐⭐

不要一开始就开最大随机化。

这和奖励正则一样。

训练早期策略还不会站。

如果同时随机质量、摩擦、噪声、延迟、外推和地形，
它连第一个稳定行为都找不到。

推荐三阶段。

| 阶段 | 随机化 | 目标 |
|------|--------|------|
| Stage 1 | 初始状态小扰动 | 学会站立和基础跟踪 |
| Stage 2 | 摩擦、质量、观测噪声 | 学会参数鲁棒 |
| Stage 3 | 外推、延迟、地形课程 | 学会扰动恢复和泛化 |

### 域随机化的诊断 ⭐⭐⭐

域随机化开得越多，
训练曲线越难解释。

因此每个随机化项都要可记录。

建议记录：

| 指标 | 用途 |
|------|------|
| `dr/friction_mean` | 确认摩擦分布是否生效 |
| `dr/mass_scale_mean` | 确认质量随机化范围 |
| `dr/push_speed_mean` | 外推强度 |
| `dr/sensor_noise_std` | 噪声课程 |
| `episode/failure_by_friction_low` | 低摩擦是否导致失败 |

更好的诊断方式是分桶测试。

例如固定策略，
在不同摩擦区间评估。

| 摩擦范围 | 成功率 | 平均速度误差 |
|----------|--------|--------------|
| 0.3-0.5 | 低摩擦极限 | 观察打滑 |
| 0.6-0.9 | 普通地面 | 主测试 |
| 1.0-1.3 | 高摩擦 | 观察高冲击 |

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 思维 | 随机化范围越大越好 | 策略保守、不收敛 | 训练分布过宽 | 从窄范围课程化扩大 |
| 概念 | 独立随机惯量矩阵元素 | 仿真奇怪但不报错 | 可能违反刚体物理约束 | 使用物理一致参数化或保守缩放 |
| 编程 | 随机化参数不进日志 | 不知道失败来自哪个扰动 | 随机性不可追踪 | 记录采样值和分桶成功率 |
| 思维 | 用 DR 替代所有系统辨识 | 真机仍失败 | DR 覆盖不了结构性错误 | 先修正模型方向性错误，再随机化 |

### 练习 ⭐

1. 为 Go2 velocity tracking 设计三阶段域随机化课程，
   每阶段列出随机化项、范围、开启条件。
2. 解释为什么质心偏移随机化比单纯质量缩放更影响转向和加减速。
3. 设计一个分桶评估实验：
   固定策略，在 5 个摩擦区间分别跑 100 个 episode。

---

## S3-B.9 训练 loop：从批量 rollout 到 PPO 更新 ⭐⭐⭐

### 动机：环境写对不等于训练系统写对 ⭐

很多训练失败不是物理环境错，
而是训练 loop 的数据语义错。

典型错误包括：

1. `done` 和 `timeout` 混用。
2. GAE 计算时没有 mask terminal。
3. rollout buffer 的 `[T, B]` 轴顺序混乱。
4. reward normalization 和 observation normalization 部署不一致。
5. checkpoint 只保存网络，
   没保存观测缩放和动作缩放。

因此需要把训练 loop 的结构讲清楚。

### PPO rollout 数据结构 ⭐⭐

一个 on-policy rollout 通常包含：

| 字段 | 形状 | 说明 |
|------|------|------|
| obs | `[T, B, obs_dim]` | actor 输入 |
| critic_obs | `[T, B, critic_dim]` | critic 输入 |
| actions | `[T, B, action_dim]` | 策略采样动作 |
| log_probs | `[T, B]` | 旧策略 log probability |
| values | `[T, B]` | critic 估计 |
| rewards | `[T, B]` | 环境奖励 |
| dones | `[T, B]` | 失败终止 mask |
| timeouts | `[T, B]` | 正常截断 mask |
| infos | pytree | 日志分项 |

PPO 更新时，
优势函数计算常用 GAE：

$$
\delta_t =
r_t + \gamma V(s_{t+1})(1-d_t) - V(s_t)
$$

$$
A_t =
\delta_t + \gamma\lambda(1-d_t)A_{t+1}
$$

这里的 $d_t$ 应表示真正 terminal。

timeout 不一定应该让 value bootstrap 断开。

如果 episode 是因为固定长度截断，
状态本身并未失败。

这时通常需要用 $V(s_{t+1})$ bootstrap。

### 教学版 PPO rollout loop ⭐⭐

```python
def collect_rollout(env, policy, state, obs, rollout_len):
    # buffer 用 Python list 表示教学逻辑；高性能实现会预分配张量
    obs_buf = []
    action_buf = []
    reward_buf = []
    done_buf = []
    value_buf = []
    logprob_buf = []
    info_buf = []

    for t in range(rollout_len):
        # 1. 策略根据当前观测输出动作分布
        dist, value = policy(obs)

        # 2. 从分布采样动作；评估时通常用均值动作
        action = dist.sample()
        log_prob = dist.log_prob(action)

        # 3. 批量环境前进一步
        next_state, next_obs, reward, done, info = env.step(state, action)

        # 4. 保存 rollout 数据，注意每个字段第一维都是时间
        obs_buf.append(obs)
        action_buf.append(action)
        reward_buf.append(reward)
        done_buf.append(done)
        value_buf.append(value)
        logprob_buf.append(log_prob)
        info_buf.append(info)

        # 5. 环境内部应对 done 的子环境做 reset，并返回 reset 后观测
        state = next_state
        obs = next_obs

    return {
        "obs": stack_time(obs_buf),
        "actions": stack_time(action_buf),
        "rewards": stack_time(reward_buf),
        "dones": stack_time(done_buf),
        "values": stack_time(value_buf),
        "log_probs": stack_time(logprob_buf),
        "infos": info_buf,
    }, state, obs
```

这段代码的关键点是：
rollout loop 不负责手动重置单个环境。

批量环境的 `step()` 应该处理 done mask，
并返回可继续训练的 next state。

否则训练 loop 会把 reset 逻辑和环境逻辑耦合在一起。

### GAE 计算示例 ⭐⭐⭐

```python
def compute_gae(rewards, dones, values, last_value, gamma=0.99, lam=0.95):
    # rewards/dones/values 形状都是 [T, B]
    T = rewards.shape[0]
    advantages = []
    gae = zeros_like(last_value)
    next_value = last_value

    for t in reversed(range(T)):
        # done=True 表示真正终止，bootstrap 应断开
        not_done = 1.0 - dones[t].float()
        delta = rewards[t] + gamma * next_value * not_done - values[t]
        gae = delta + gamma * lam * not_done * gae
        advantages.append(gae)
        next_value = values[t]

    advantages = reverse_stack(advantages)
    returns = advantages + values
    return advantages, returns
```

如果有 timeout，
可以把 `dones` 拆成：

```python
terminal = dones & (~timeouts)
```

然后 GAE 中用 `terminal` 断开 bootstrap。

这对长 episode 截断任务非常重要。

### PPO update 的核心 ⭐⭐⭐

PPO 的 clipped objective：

$$
L^{CLIP}(\theta) =
\mathbb{E}
\left[
\min(
r_t(\theta)A_t,
\text{clip}(r_t(\theta),1-\epsilon,1+\epsilon)A_t
)
\right]
$$

其中：

$$
r_t(\theta) =
\frac{\pi_\theta(a_t|s_t)}
{\pi_{\theta_{old}}(a_t|s_t)}
$$

训练中还会加：

1. value loss。
2. entropy bonus。
3. gradient clipping。
4. learning rate schedule。

### 训练 loop 的最小闭环 ⭐⭐

```python
for iteration in range(max_iterations):
    # 收集一段 [T, B] rollout
    rollout, env_state, obs = collect_rollout(env, policy, env_state, obs, rollout_len)

    # 用 critic 对最后状态估值，供 GAE bootstrap
    last_value = policy.value(obs)

    # 计算 advantage 和 return
    adv, ret = compute_gae(
        rewards=rollout["rewards"],
        dones=rollout["dones"],
        values=rollout["values"],
        last_value=last_value,
        gamma=0.99,
        lam=0.95,
    )

    # 展平 [T, B] -> [T*B]，进入 PPO mini-batch 更新
    batch = flatten_rollout(rollout, adv, ret)

    # 多轮 epoch 更新同一批 on-policy 数据
    train_stats = ppo_update(policy, optimizer, batch)

    # 记录训练和环境指标
    logger.log({
        "train/policy_loss": train_stats.policy_loss,
        "train/value_loss": train_stats.value_loss,
        "train/entropy": train_stats.entropy,
        "env/episode_return": mean_episode_return(rollout),
        "env/termination_rate": mean_terminal_rate(rollout),
    })

    # 定期保存 checkpoint，必须包含归一化和动作配置
    if iteration % save_interval == 0:
        save_checkpoint(policy, optimizer, obs_normalizer, action_spec, iteration)
```

### checkpoint 与导出 ⭐⭐

一个可部署 checkpoint 不只是网络权重。

至少应包含：

| 内容 | 为什么需要 |
|------|------------|
| actor 网络参数 | 策略本体 |
| observation normalization | 保证输入分布一致 |
| observation order/spec | 防止部署拼接顺序错 |
| action scale/default pose | 保证输出语义一致 |
| control frequency | 保证动作更新频率一致 |
| joint name order | 防止左右腿或关节顺序错 |
| policy version/config | 复现实验 |

ONNX 导出时尤其要检查输入输出名称。

```python
def build_deploy_package(policy_path, obs_spec, action_spec, control_dt):
    # 这是教学用结构，实际项目可以保存为 yaml/json
    return {
        "policy_path": policy_path,
        "obs_spec": obs_spec,
        "action_spec": action_spec,
        "control_dt": control_dt,
        "notes": "部署端必须按 obs_spec 拼接观测，并按 action_spec 映射动作。",
    }
```

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | `[T,B]` 和 `[B,T]` 混用 | GAE 数值怪，训练不稳 | 时间轴递推错 | 固定 buffer 轴规范 |
| 概念 | timeout 直接当失败 | value 低估长 episode | 截断不等于终止 | 拆分 terminal 和 timeout |
| 编程 | checkpoint 只保存网络 | ONNX 回放失败 | 缺少观测/动作语义 | 保存 obs/action spec |
| 思维 | PPO 不收敛先调网络结构 | 反复无效 | 多数问题在 MDP 和数据 | 先做 zero/random policy sanity check |

### 练习 ⭐

1. 写出一个包含 `timeouts` 的 GAE 公式。
2. 给定 `T=24`、`B=4096`、`obs_dim=48`，
   估算 obs buffer 的 float32 显存。
3. 设计一个部署包格式，
   至少包含观测顺序、归一化、动作尺度、控制频率和关节顺序。

---

## S3-B.10 日志系统：让训练曲线能回答工程问题 ⭐⭐

### 动机：没有分项日志，就没有可调试训练 ⭐

训练曲线不是给人看的装饰。

它是调试工具。

一个合格日志系统至少要回答 8 个问题。

| 问题 | 需要的日志 |
|------|------------|
| 策略有没有学会跟踪命令？ | 速度误差、角速度误差 |
| 策略是不是靠摔倒刷奖励？ | termination reason、episode length |
| 动作是否太抖？ | action rate、action std、torque/power |
| 是否发生电机饱和？ | torque saturation rate |
| 接触是否合理？ | foot contact ratio、air time |
| 域随机化是否生效？ | 随机参数分布 |
| 训练是否过大步更新？ | KL、clip fraction、entropy |
| 环境是否慢在 reset 或物理？ | SPS 分解、reset count |

只看 episode return 很危险。

return 上升可能来自速度跟踪变好。

也可能来自策略学会少动，
从而减少能耗惩罚。

还可能来自终止条件太宽，
摔倒后继续累积某些奖励。

因此日志必须分层。

### 四类日志 ⭐⭐

| 类别 | 频率 | 示例 | 目的 |
|------|------|------|------|
| train | 每个 update | policy loss、value loss、KL、entropy | 判断 PPO 是否健康 |
| reward | 每个 rollout | reward term 均值 | 判断目标和正则的权衡 |
| task | 每个 episode/rollout | 速度误差、命令分布 | 判断任务完成度 |
| system | 每秒或每 rollout | SPS、GPU 显存、reset 次数 | 判断性能瓶颈 |

### 推荐日志字段 ⭐⭐

```python
def build_log_dict(train_stats, env_stats, reward_terms, perf_stats):
    # 训练算法指标
    logs = {
        "train/policy_loss": train_stats.policy_loss,
        "train/value_loss": train_stats.value_loss,
        "train/entropy": train_stats.entropy,
        "train/approx_kl": train_stats.approx_kl,
        "train/clip_fraction": train_stats.clip_fraction,
        "train/learning_rate": train_stats.learning_rate,
    }

    # 环境任务指标
    logs.update({
        "episode/return_mean": env_stats.return_mean,
        "episode/length_mean": env_stats.length_mean,
        "episode/timeout_rate": env_stats.timeout_rate,
        "episode/bad_orientation_rate": env_stats.bad_orientation_rate,
        "task/lin_vel_error": env_stats.lin_vel_error,
        "task/ang_vel_error": env_stats.ang_vel_error,
        "task/command_speed_mean": env_stats.command_speed_mean,
    })

    # 奖励分项
    for name, value in reward_terms.items():
        logs[f"reward/{name}"] = value

    # 性能指标
    logs.update({
        "perf/steps_per_second": perf_stats.sps,
        "perf/sim_steps_per_second": perf_stats.sim_sps,
        "perf/reset_per_second": perf_stats.reset_sps,
        "perf/gpu_memory_gb": perf_stats.gpu_memory_gb,
    })
    return logs
```

### 训练健康指标 ⭐⭐

PPO 的训练指标可以快速发现算法层异常。

| 指标 | 正常含义 | 异常信号 |
|------|----------|----------|
| entropy | 策略探索程度 | 过快归零表示过早确定 |
| approx KL | 新旧策略差异 | 长期过大表示更新太猛 |
| clip fraction | 被 PPO clip 的比例 | 接近 0 可能更新太小，过高可能更新太大 |
| value loss | critic 拟合误差 | 持续爆炸表示 reward/value 尺度问题 |
| explained variance | value 解释 reward 的程度 | 长期接近 0 表示 critic 无效 |

这些指标不能单独决定好坏。

例如 entropy 下降是正常现象。

但如果 episode return 没涨，
entropy 却很快归零，
通常说明策略过早收敛到坏行为。

### 环境健康指标 ⭐⭐

环境指标比算法指标更接近机器人行为。

| 指标 | 解释 | 排查方向 |
|------|------|----------|
| `task/lin_vel_error` | 线速度跟踪误差 | 观测、奖励、动作尺度 |
| `task/ang_vel_error` | yaw rate 跟踪误差 | 命令范围、角速度奖励 |
| `episode/bad_orientation_rate` | 姿态失败比例 | 初始姿态、动作尺度、终止阈值 |
| `robot/torque_sat_rate` | 力矩饱和比例 | PD 增益、动作尺度、奖励 |
| `robot/action_rate` | 动作变化率 | 平滑惩罚、策略频率 |
| `contact/feet_air_time_mean` | 足端腾空时间 | gait 是否成形 |

### 可视化 rollout ⭐⭐

纯标量日志仍然不够。

每隔固定迭代保存短 rollout 视频或状态轨迹。

建议保存：

1. 平地直行。
2. 原地转向。
3. 横向移动。
4. 低摩擦测试。
5. 外推恢复。

每个 rollout 记录同一组曲线：

```text
time
  base height
  roll / pitch / yaw
  command vx / actual vx
  command vy / actual vy
  command yaw rate / actual yaw rate
  joint torque max
  action norm
  foot contacts
```

这能把“训练曲线还可以”转化为“行为真的对”。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 思维 | 只看总 reward | 调参方向错误 | 总和掩盖分项冲突 | 必须记录 reward terms |
| 编程 | 日志每步同步到 CPU | 训练变慢 | GPU/CPU 同步阻塞 | rollout 末尾批量汇总 |
| 概念 | KL 大就一定坏 | 误调学习率 | 需要结合 return 和 entropy 看 | 建立指标组合判断 |
| 编程 | 不记录终止原因 | 不知道为什么 episode 短 | done mask 信息丢失 | 每类 termination 单独计数 |

### 练习 ⭐

1. 为 S3-B.7 的奖励配置设计完整日志字段。
2. 解释为什么 `torque_sat_rate` 高时，
   单纯增大奖励权重通常无效。
3. 设计一个 5 个固定场景的 policy evaluation suite。

---

## S3-B.11 性能诊断：把慢分解到物理、策略、数据和日志 ⭐⭐⭐

### 动机：GPU 训练慢不一定是物理慢 ⭐

看到 steps per second 低，
初学者常说“仿真器慢”。

这通常不够精确。

训练吞吐由多个环节组成。

```text
policy forward
  -> action manager
  -> physics step x decimation
  -> observation manager
  -> reward / termination
  -> reset / events
  -> rollout buffer write
  -> PPO update
  -> logging / checkpoint
```

每一段都可能慢。

性能诊断的第一原则：
不要先优化，
先分解计时。

### 性能指标层级 ⭐⭐

| 指标 | 定义 | 用途 |
|------|------|------|
| env SPS | `num_envs * policy_steps / wall_time` | 总训练采样吞吐 |
| sim SPS | `num_envs * physics_steps / wall_time` | 物理后端墙钟吞吐 |
| RTF | `simulated_physical_time / wall_time` | 仿真速度相对真实时间的倍数 |
| update time | PPO 更新耗时 | 判断网络/mini-batch 开销 |
| reset time | reset 和 event 耗时 | 发现终止过频和随机化慢 |
| host sync count | CPU/GPU 同步次数 | 发现 `.item()`、打印、日志阻塞 |
| GPU memory | 显存占用 | 判断 batch 是否过大 |

### num_envs 不是越大越好 ⭐⭐

增加环境数通常会提高物理吞吐，
直到遇到瓶颈。

瓶颈可能是：

1. 显存不够。
2. 接触 buffer 太大。
3. PPO update batch 太大。
4. reset/event 太多。
5. 单个环境太复杂，
   导致 kernel occupancy 下降。

典型扫描方式：

| num_envs | 观察 |
|----------|------|
| 512 | 基线，确认正确 |
| 1024 | 看吞吐是否接近翻倍 |
| 2048 | 看显存和 KL |
| 4096 | 常见训练规模 |
| 8192+ | 只在硬件和任务允许时尝试 |

如果从 4096 到 8192，
SPS 只提升 5%，
但显存和 update 时间明显增加，
就没有必要继续增大。

### decimation 的性能与控制权衡 ⭐⭐⭐

策略频率：

$$
f_{policy} = \frac{1}{dt_{physics}\cdot decimation}
$$

物理频率固定时，
decimation 越大，
策略推理越少，
但控制响应越慢。

| decimation | 策略频率示例 | 优点 | 风险 |
|------------|--------------|------|------|
| 4 | 125Hz, dt=0.002 | 响应快 | 策略输出高频，推理开销大 |
| 10 | 50Hz | 常见折中 | 需要动作平滑 |
| 20 | 25Hz | 推理少 | 快速扰动恢复差 |

### 接触复杂度 ⭐⭐⭐

腿足和灵巧手任务性能常被接触支配。

影响接触复杂度的因素：

| 因素 | 影响 |
|------|------|
| geom 数量 | broad phase 碰撞候选增加 |
| mesh 复杂度 | narrow phase 成本增加 |
| 同时接触点 | 约束求解成本增加 |
| 摔倒样本比例 | 身体大面积接触地面 |
| 地形复杂度 | heightfield/mesh 接触增加 |

如果训练早期大量摔倒，
物理吞吐可能很差。

这不是后端退化，
而是 MDP 导致接触变复杂。

解决方式包括：

1. 更保守的初始状态。
2. 更严格的摔倒终止。
3. 先在平地和站姿课程训练。
4. 简化碰撞几何。

### 性能 profile 示例 ⭐⭐⭐

```python
class Timer:
    def __init__(self):
        self.records = {}

    def add(self, name, dt):
        # 记录各阶段耗时，真实实现应避免每步 CPU 同步
        self.records.setdefault(name, []).append(dt)

    def summary(self):
        return {name: sum(values) / len(values) for name, values in self.records.items()}


def train_iteration(env, policy, timer):
    with timer_block(timer, "rollout_total"):
        with timer_block(timer, "policy_forward"):
            action = policy(env.obs)

        with timer_block(timer, "env_step"):
            next_obs, reward, done, info = env.step(action)

        with timer_block(timer, "buffer_write"):
            rollout_buffer.add(env.obs, action, reward, done)

    with timer_block(timer, "ppo_update"):
        ppo_update(policy, rollout_buffer)
```

真实 GPU profile 应使用框架提供的 profiler，
并减少计时本身造成的同步。

教学阶段可以先做粗粒度计时。

### 常见性能故障 ⭐⭐

| 症状 | 可能原因 | 快速验证 |
|------|----------|----------|
| 第一轮极慢，后续正常 | JIT/图捕获 | 排除 warmup 后计时 |
| 每轮都慢 | 形状变化导致反复编译 | 打印编译次数或缓存命中 |
| GPU 利用率低 | CPU 同步、日志、Python loop | 关闭日志对比 |
| 显存爆 | num_envs 或 contact buffer 太大 | 降环境数/接触上限 |
| reset 很慢 | 终止太频繁或随机化复杂 | 统计 reset rate |
| update 很慢 | 网络太大或 mini-batch 不合理 | 固定 rollout，单测 PPO update |

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 思维 | 只报一个 SPS | 无法定位瓶颈 | 总吞吐不可解释 | 分解 sim/update/reset/log |
| 编程 | 在训练主循环频繁 `.item()` | GPU 利用率低 | CPU/GPU 同步 | 批量汇总后再转 CPU |
| 概念 | 盲目增加 num_envs | 学习变差或显存爆 | batch 与优化超参耦合 | 扫描环境数并调 mini-batch |
| 编程 | 复杂 mesh 直接用于训练碰撞 | 接触慢、不稳定 | 碰撞几何过复杂 | 使用简化 collision geom |

### 练习 ⭐

1. 设计一个性能扫描表：
   `num_envs = 512, 1024, 2048, 4096, 8192`，
   记录 SPS、显存、KL、return。
2. 解释为什么摔倒率高会降低物理吞吐。
3. 在一个训练 loop 中加入粗粒度 timer，
   区分 rollout、env_step、ppo_update 和 logging。

---

## S3-B.12 从 MuJoCo 单环境迁移到批量训练 ⭐⭐⭐⭐

### 动机：迁移不是把 for 循环外面套一层 batch ⭐

已有 MuJoCo 环境通常是面向调试写的。

它可能依赖：

1. 可变长度 Python list。
2. 全局随机数。
3. viewer 状态。
4. `mjData` 原地修改。
5. 每步打印。
6. 单个 episode 的控制流。

批量训练需要重构这些假设。

迁移顺序建议如下。

```text
Step 1  单环境可复现
Step 2  分离 reset / step / observe / reward / done
Step 3  明确 state 与 observation
Step 4  明确动作映射和控制频率
Step 5  把随机数显式化
Step 6  增加 batch 维度和 mask reset
Step 7  接入 PPO rollout buffer
Step 8  增加日志和 sim2sim 回放
```

### Step 1：先修好单环境 ⭐⭐

单环境必须通过 sanity check。

| 检查 | 期望 |
|------|------|
| zero action | 机器人不应立刻爆炸 |
| default PD target | 能保持基本站姿或按模型预期运动 |
| random action small | 不出现 NaN |
| reset repeatability | 同一种子结果一致 |
| observation finite | 没有 NaN/Inf |
| reward finite | 没有 NaN/Inf |

如果单环境都不稳定，
批量化只会把 bug 放大。

### Step 2：拆出函数 ⭐⭐

手写单环境常见结构：

```python
class OldEnv:
    def step(self, action):
        self.d.ctrl[:] = action
        mujoco.mj_step(self.m, self.d)
        obs = self.build_obs()
        reward = self.compute_reward()
        done = self.check_done()
        if done:
            self.reset()
        return obs, reward, done, {}
```

迁移前先拆成：

```python
class SplitEnv:
    def apply_action(self, action):
        # 只负责把策略动作映射到 MuJoCo ctrl
        pass

    def simulate(self):
        # 只负责物理步进和 decimation
        pass

    def observe(self):
        # 只读状态，不修改环境
        pass

    def reward(self):
        # 只读状态、动作、命令
        pass

    def done(self):
        # 只判断终止，不执行 reset
        pass

    def reset(self, seed):
        # 只重置状态和命令
        pass
```

这个拆分对应 Manager API。

后续迁移到 mjlab 时，
每个函数都能找到归属。

### Step 3：写出观测规范 ⭐⭐

迁移前必须写 obs spec。

| index | name | dim | unit | scale | source | deploy |
|-------|------|-----|------|-------|--------|--------|
| 0:3 | base_ang_vel | 3 | rad/s | 0.25 | IMU/MuJoCo | yes |
| 3:6 | projected_gravity | 3 | unit | 1.0 | IMU/MuJoCo | yes |
| 6:9 | command | 3 | m/s, rad/s | 1.0 | command sampler | yes |
| 9:21 | joint_pos_rel | 12 | rad | 1.0 | encoder | yes |
| 21:33 | joint_vel | 12 | rad/s | 0.05 | encoder | yes |
| 33:45 | last_action | 12 | normalized | 1.0 | policy memory | yes |

有了 spec，
部署和训练才不会拼错。

### Step 4：动作映射单元测试 ⭐⭐

动作映射必须单独测试。

```python
def test_action_mapping(default_q, q_min, q_max):
    action_scale = 0.25

    # 零动作应返回默认姿态
    target = apply_joint_position_action(
        action=zeros_like(default_q),
        default_q=default_q,
        scale=action_scale,
        q_min=q_min,
        q_max=q_max,
    )
    assert allclose(target, default_q)

    # 最大动作不能超过关节限位
    target = apply_joint_position_action(
        action=ones_like(default_q),
        default_q=default_q,
        scale=action_scale,
        q_min=q_min,
        q_max=q_max,
    )
    assert all(target <= q_max)
```

这类测试比直接训练更快暴露错误。

### Step 5：从 Python list 到静态 buffer ⭐⭐⭐

回顾 S3-B.3 中的静态形状要求：JAX/MJX/Warp 风格的 GPU 编译需要在编译期确定所有张量的形状。接触数量天然是动态的——走路时两脚接地，站稳时四脚接地，摔倒时接触可能更多。如果每步根据实际接触数分配数组，形状就会变化，编译器无法为固定计算图生成高效 kernel。因此迁移时必须把动态长度的 Python list 改为预分配的固定大小 buffer，用 mask 标记哪些位置有效。

单环境常写：

```python
contacts = []
for i in range(d.ncon):
    contacts.append(d.contact[i])
```

批量训练应改成固定大小 buffer。核心思路是预分配一个 `max_contacts` 大小的数组，无论实际接触数是多少都保持同一形状，然后用布尔 mask 标记哪些位置存放了真实接触数据。这样 JIT 编译器在 trace 阶段就能确定所有中间张量的维度，不会因为接触数变化而触发重新编译。超出 `max_contacts` 的接触会被截断，因此这个上界需要根据机器人和场景合理设定。

```python
def build_contact_buffer(raw_contacts, max_contacts):
    # contact_buffer 形状固定，mask 表示哪些 contact 有效
    contact_buffer = zeros((max_contacts, contact_dim))
    contact_mask = zeros((max_contacts,), dtype=bool)

    n = min(len(raw_contacts), max_contacts)
    contact_buffer[:n] = raw_contacts[:n]
    contact_mask[:n] = True
    return contact_buffer, contact_mask
```

这一步对应 MJX/Warp 的静态形状要求。

### Step 6：批量 reset mask ⭐⭐⭐

批量环境中，每个 episode 独立结束。某些环境达到 timeout 或 failure 时需要 reset，其余环境继续运行。在 CPU 单环境中直接调用 `reset()` 即可；在批量环境中，reset 必须用 mask 化的 `where` 操作选择性地覆盖已结束环境的状态，同时保持其余环境不受影响。这是批量化迁移中最容易出错的步骤之一，因为每个状态字段的维度不同，broadcast 规则也不同。

为什么不能用条件分支 `if done[i]: reset(i)` 逐个处理？原因与静态形状相同：GPU 编译要求控制流在编译期确定，运行时按 mask 选择数据路径。`where(done, reset_val, keep_val)` 在所有环境上同时执行，已结束的环境取 `reset_val`，未结束的取 `keep_val`，没有分支、没有循环，符合 SIMT 执行模型。需要注意 `done` 形状为 `[B]`，而状态字段可能是 `[B, D]`，必须正确扩展维度才能 broadcast。

```python
def reset_where_done(state, reset_state, done):
    # done 形状 [B]
    # 需要扩展到每个字段的维度
    done_pos = done[:, None]

    next_pos = where(done_pos, reset_state.pos, state.pos)
    next_vel = where(done_pos, reset_state.vel, state.vel)
    next_cmd = where(done[:, None], reset_state.cmd, state.cmd)
    next_step_count = where(done, reset_state.step_count, state.step_count)

    return BatchState(
        pos=next_pos,
        vel=next_vel,
        cmd=next_cmd,
        step_count=next_step_count,
    )
```

这里的难点不是代码。

难点是每个状态字段都必须知道 batch 维度在哪里。

### Step 7：从单环境 reward 到批量 reward ⭐⭐

单环境 reward：

```python
def reward_single(base_vel, cmd_vel):
    error = base_vel[0] - cmd_vel[0]
    return math.exp(-(error * error) / 0.25)
```

批量 reward：

```python
def reward_batch(base_vel, cmd_vel):
    # base_vel/cmd_vel 形状 [B, 3]
    error_xy = base_vel[:, :2] - cmd_vel[:, :2]
    error_sq = jnp.sum(error_xy ** 2, axis=-1)
    return jnp.exp(-error_sq / 0.25)
```

迁移时要避免在批量函数里调用 Python `math`。

应使用数组库函数。

### Step 8：迁移到 mjlab Manager ⭐⭐⭐

迁移完成后，
原来的拆分函数对应：

| 原函数 | Manager 归属 |
|--------|--------------|
| `apply_action` | ActionManager |
| `observe` | ObservationManager |
| `reward` | RewardManager |
| `done` | TerminationManager |
| `reset` | EventManager |
| `command_sample` | Command/Event |
| `log_metrics` | Metrics/Recorder |

这一步不是机械复制。

你要重新检查每个函数是否只做自己的事。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 迁移前不写 obs spec | 部署拼接错 | 观测语义没有文档化 | 先写表，再写代码 |
| 思维 | 单环境不稳就上 GPU | 大量 NaN，难定位 | bug 被批量放大 | 先通过 sanity check |
| 编程 | 批量函数中保留 Python `if done` | JIT/向量化失败 | done 是数组，不是标量 | 使用 mask/where |
| 概念 | reset 改变 batch 大小 | rollout buffer 崩 | 批量训练需要静态形状 | 原地 reset，不删除环境 |

### 练习 ⭐

1. 找一个已有 MuJoCo Gym 风格环境，
   写出它的 `apply_action/observe/reward/done/reset` 拆分表。
2. 把一个单环境速度奖励改写成批量数组版本。
3. 设计迁移验收：
   单环境和批量环境在 `B=1` 时，
   同一动作序列下观测和奖励应一致。

---

## S3-B.13 CPU reference replay、Sim2Sim 与部署前回放 ⭐⭐⭐

### 动机：GPU 训练成功只是第一关 ⭐

策略在 mjlab/MuJoCo Warp 中训练成功，
不代表可以直接上真机。

部署前至少应做 MuJoCo CPU reference replay。它不是严格意义上的跨引擎 Sim2Sim，而是同一 MuJoCo 物理家族内更容易单步调试、记录接触和复现实验的回放路径。若还要证明策略不依赖某个物理实现细节，应继续做跨引擎 Sim2Sim。

原因有三个。

1. CPU MuJoCo 更适合逐步调试和可视化。
2. GPU 批量后端和 CPU 后端可能存在数值、接触和特性覆盖差异。
3. 回放代码更接近最终部署控制循环。

### 回放循环 ⭐⭐⭐

CPU 回放的核心要求是让观测构造、动作映射和控制频率与训练环境完全对齐。下面的 `PolicyPlayer` 类展示了这个对齐结构：它加载 MuJoCo CPU 模型和导出的 ONNX 策略，按训练时的 obs spec 拼接观测，按训练时的 action spec 反归一化动作，并用 decimation 控制物理步与策略步的比例。如果其中任何一个环节与训练环境不一致，回放时策略的行为就会偏离训练预期——这正是 sim2sim 最常见的失败原因。

对齐原则可以概括为三条：第一，obs spec 对齐——观测的拼接顺序、归一化 scale 和噪声处理必须与训练环境一一对应，多一个字段或少一个字段都会让策略收到错误的输入；第二，action spec 对齐——策略输出的 `[-1, 1]` 范围如何映射到关节目标位置，包括 default pose、action scale 和 clip 范围，必须与训练时完全相同；第三，decimation 对齐——每个策略步对应多少个物理子步，决定了实际的 `control_dt`，这个比例不一致会导致动作保持时间错误，直接表现为步态频率异常或摔倒。

```python
import time
import numpy as np
import mujoco
import onnxruntime as ort


class PolicyPlayer:
    def __init__(self, model_path, policy_path, obs_spec, action_spec):
        # 加载 MuJoCo CPU 模型
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)

        # 加载 ONNX 策略
        self.session = ort.InferenceSession(policy_path)
        self.obs_spec = obs_spec
        self.action_spec = action_spec

        # 保存上一步动作，用于构造观测
        self.last_action = np.zeros(action_spec["dim"], dtype=np.float32)

    def build_obs(self, command):
        # 按训练时 obs_spec 的顺序拼接观测
        parts = []
        parts.append(get_base_ang_vel_body(self.m, self.d) * self.obs_spec["base_ang_vel"]["scale"])
        parts.append(get_projected_gravity(self.m, self.d))
        parts.append(command.astype(np.float32))
        parts.append(get_joint_pos_rel(self.m, self.d) * self.obs_spec["joint_pos"]["scale"])
        parts.append(get_joint_vel(self.m, self.d) * self.obs_spec["joint_vel"]["scale"])
        parts.append(self.last_action)
        return np.concatenate(parts).astype(np.float32)

    def policy(self, obs):
        # ONNX 输入名应在导出时固定
        action = self.session.run(None, {"obs": obs[None, :]})[0][0]
        return np.clip(action, -1.0, 1.0)

    def apply_action(self, action):
        # 训练和部署必须使用同一动作映射
        default_q = self.action_spec["default_joint_pos"]
        scale = self.action_spec["scale"]
        target_q = default_q + scale * action
        self.d.ctrl[:] = target_q
        self.last_action = action.astype(np.float32)

    def run(self, command, control_dt):
        # 根据 MuJoCo timestep 计算 decimation
        decimation = int(round(control_dt / self.m.opt.timestep))
        while True:
            obs = self.build_obs(command)
            action = self.policy(obs)
            self.apply_action(action)

            # 一个策略周期内执行多个物理步
            for _ in range(decimation):
                mujoco.mj_step(self.m, self.d)

            time.sleep(control_dt)
```

上面代码省略了具体的 `get_*` 函数。

重点是结构。

1. 观测顺序必须按训练 spec。
2. 动作映射必须按训练 spec。
3. 控制频率必须按训练配置。
4. 物理步进和策略步进必须用 decimation 对齐。

### 回放验收清单 ⭐⭐

| 检查项 | 通过标准 |
|--------|----------|
| zero command | 能稳定站立，不原地乱走 |
| forward command | 实际 $v_x$ 跟踪命令 |
| yaw command | 能原地转向或按任务设定转弯 |
| command switch | 速度切换不过度摔倒 |
| low friction | 低摩擦下不立即失稳 |
| push recovery | 中等外推后能恢复 |
| joint limits | 关节不过度撞限 |
| torque saturation | 饱和比例可接受 |

### 与真机部署的边界 ⭐⭐⭐

本章只覆盖仿真到回放。

真机部署还需要：

1. 安全绳或支撑架。
2. 电机急停。
3. 低速命令起步。
4. 关节限位和力矩限幅硬保护。
5. 状态估计延迟处理。
6. 通信丢包处理。
7. 人员和场地安全隔离。

对于高动态动作，
还需要更严格的实验审批和保护措施。

这里的核心原则是：
任何训练框架都不能替代硬件安全工程。

### 常见陷阱 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | ONNX 输入顺序与训练不同 | 行为随机 | obs spec 错 | 导出和回放共用 spec |
| 编程 | 控制频率不一致 | 步态频率异常 | decimation 错 | 保存并使用 control_dt |
| 概念 | GPU 成功就跳过 CPU 回放 | 难定位部署问题 | 缺少 sim2sim 检查 | 必做 CPU 回放 |
| 思维 | 回放只看一个命令 | 泛化未知 | 测试覆盖不足 | 固定评估套件 |

### 练习 ⭐

1. 写一个 CPU 回放日志表，
   每 0.1 秒记录命令速度、实际速度、base height、最大力矩。
2. 解释为什么 `time.sleep(control_dt)` 不能保证实时控制精度，
   真机部署应使用什么形式的实时循环。
3. 设计一个“部署前禁止上机”的失败标准清单。

---

## S3-B.14 故障排查手册 ⭐⭐

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|----------|----------|----------|
| 训练一开始大量 NaN | 初始状态不合法、动作尺度过大、观测除零 | 1. B=1 单环境跑 zero action；2. 检查 obs/reward finite；3. 降低动作尺度 | S3-B.4 到 S3-B.7 |
| reward 不涨但机器人不摔 | 速度奖励太弱、能耗/平滑惩罚太强 | 1. 看 reward 分项；2. 暂时关闭正则；3. 缩小命令范围 | S3-B.7 |
| reward 上涨但行为抖 | action rate 弱、策略频率太高、PD 增益不合适 | 1. 看 action_rate；2. 调 decimation；3. 检查 torque saturation | S3-B.6、S3-B.11 |
| episode 很短 | 终止过严或初始 reset 太难 | 1. 记录终止原因；2. 放宽姿态阈值；3. 减小 reset 扰动 | S3-B.7、S3-B.8 |
| 训练很慢 | GPU 同步、接触复杂、环境数不合适 | 1. 分解计时；2. 关闭视频/日志；3. 扫描 num_envs；4. 检查摔倒率 | S3-B.10、S3-B.11 |
| sim 中能走，CPU 回放摔 | 观测顺序、动作映射、控制频率不一致 | 1. 对比 obs spec；2. 打印前 10 维观测；3. 核对 action scale/default pose | S3-B.12、S3-B.13 |
| 策略只会向前不会转弯 | yaw 命令分布窄或角速度奖励太弱 | 1. 记录命令分布；2. 单独训练 yaw command；3. 增加 yaw reward 权重 | S3-B.5、S3-B.7 |
| 低摩擦测试失败 | 摩擦随机化范围不足或接触模型差异 | 1. 分桶评估摩擦；2. 扩大课程范围；3. 检查脚底 geom 和摩擦参数 | S3-B.8 |
| 多 GPU 或大 batch 后学习变差 | batch size 改变但 PPO 超参未调 | 1. 看 KL/clip fraction；2. 调 mini-batch 和 epoch；3. 调学习率 | S3-B.9、S3-B.11 |
| 足端乱碰或拖地 | reward 缺少足端节奏约束、默认姿态不合适 | 1. 看 foot contact ratio；2. 检查 air time；3. 可视化足端轨迹 | S3-B.7、S3-B.10 |

### 调试顺序建议 ⭐⭐

不要同时改多个大项。

推荐顺序：

1. 单环境 sanity check。
2. B=1 批量环境一致性。
3. zero/random policy rollout。
4. 小批量训练 512 env。
5. 奖励分项和终止原因日志。
6. 扩大 num_envs。
7. 加域随机化课程。
8. CPU 回放。
9. 固定评估套件。

这条顺序的本质是先排除确定性错误，
再处理随机性和性能。

---

## S3-B.15 综合练习 ⭐

### A 型：最小可行 Go2 velocity tracking ⭐⭐

目标：
创建一个平地 Go2 velocity tracking 任务。

要求：

1. policy obs 包含：
   projected gravity、base angular velocity、velocity command、joint pos、joint vel、last action。
2. action 使用相对默认关节位置。
3. reward 至少包含：
   线速度跟踪、yaw rate 跟踪、z 速度惩罚、roll/pitch 角速度惩罚、action rate。
4. termination 至少包含：
   timeout、bad orientation、base height。
5. 记录每个 reward term 和 termination reason。

验收：

| 项目 | 通过标准 |
|------|----------|
| zero action | 不出现 NaN |
| random action | 不出现仿真爆炸 |
| 训练 100 iter | reward 分项有非零变化 |
| play 回放 | 命令改变时策略响应方向正确 |

### A 型：域随机化消融 ⭐⭐⭐

训练三组策略：

1. 无域随机化。
2. 只随机初始姿态和关节。
3. 随机初始状态、摩擦、质量、观测噪声和外推。

测试时固定策略，
在以下条件评估：

| 测试 | 参数 |
|------|------|
| nominal | 默认参数 |
| low friction | 摩擦降低 |
| high mass | 质量增加 |
| sensor noise | 观测噪声加倍 |
| push | 每 10 秒外推 |

分析：

1. 哪组 nominal reward 最高？
2. 哪组鲁棒性最好？
3. 哪个随机化项带来最大收益？

### A 型：单环境迁移 ⭐⭐⭐

选择一个已有 MuJoCo Python 环境。

完成：

1. 拆出 `apply_action/observe/reward/done/reset`。
2. 写 obs spec 表。
3. 写 action spec 表。
4. 写 B=1 批量一致性测试。
5. 改成 `[B, ...]` 批量版本。

验收：

同一随机种子和同一动作序列下，
B=1 批量环境与原单环境的前 100 步观测、奖励差异可解释。

### B 型：训练 loop 精读 ⭐⭐

精读你所用 runner 的 rollout 和 PPO update 代码。

画出：

1. rollout buffer 中每个字段的 shape。
2. done/time_out 如何进入 GAE。
3. mini-batch 如何从 `[T,B]` 展平。
4. checkpoint 保存了哪些非网络参数。

回答：

如果删除 observation normalizer，
训练和部署分别会出现什么问题？

### B 型：性能 profile ⭐⭐⭐

对同一个任务扫描：

```text
num_envs = 512, 1024, 2048, 4096
```

每组记录：

1. steady SPS。
2. GPU memory。
3. rollout time。
4. update time。
5. reset rate。
6. average return。
7. approximate KL。

分析：

1. 最优环境数是多少？
2. 瓶颈在物理还是 PPO update？
3. 环境数变化后是否需要调 learning rate 或 mini-batch？

### 跨章综合题 ⭐⭐⭐

结合腿足简化模型和多模态 MPC 两章，
设计一个“RL policy + SRBD safety monitor”的混合系统。

要求：

1. RL policy 输出期望速度或关节目标。
2. SRBD monitor 根据 base 状态和足端接触估计稳定性。
3. 当 pitch/roll 或接触力指标超过阈值时，
   降级到站立恢复策略。
4. 写出日志字段，
   让你能区分是策略失败还是 monitor 触发过严。

思考：

为什么 RL 策略不应该直接绕过安全 monitor 输出电机力矩？

---

## S3-B.16 本章小结 ⭐⭐

### 一张表回顾全章 ⭐

| 模块 | 本章结论 | 工程检查 |
|------|----------|----------|
| 批量环境 | 时间 × 世界二维采样是 RL 吞吐基础 | buffer 形状 `[T,B,...]` 清晰 |
| MJX/JAX | `vmap` 让编译器看见批量维度 | step 函数接近纯函数 |
| mjlab | Manager 分离 MDP 组成部分 | 每个变化有明确归属 |
| 观测 | 只给可部署且任务必要的信息 | obs spec 固化 |
| 动作 | 常用相对默认关节位置 + PD | 保存 action scale/default pose |
| 奖励 | 主任务先学会，正则后加强 | 分项日志必备 |
| 终止 | timeout 和 failure 要区分 | 记录 termination reason |
| 域随机化 | 扩大训练分布，不替代辨识 | 课程化并记录采样值 |
| 训练 loop | GAE、mask、buffer 轴顺序决定正确性 | 检查 `[T,B]` 和 timeout |
| 日志 | 日志要回答工程问题 | train/reward/task/system 分层 |
| 性能 | 慢要分解到物理、更新、reset、日志 | 做 num_envs 扫描 |
| 迁移 | 先拆单环境，再批量化 | B=1 一致性测试 |
| sim2sim | GPU 成功后仍要 CPU 回放 | obs/action/control_dt 对齐 |

### 三句话记忆 ⭐

1. 批量环境的本质是：
   同一套物理和 MDP 函数，
   同时作用在很多独立世界上。
2. mjlab 的价值是：
   把观测、动作、奖励、终止、事件和日志拆开，
   让环境能被迁移、消融和诊断。
3. sim2real 的第一步不是上真机，
   而是让训练环境、回放环境和部署控制循环共享同一份观测与动作语义。

### 本章形成的判断力 ⭐⭐

读到一个新的 GPU RL 仿真框架时，
不要先问“它有多快”。

先问 8 个问题。

1. 它的状态和观测是否分离？
2. 它的批量维度在哪里？
3. reset 是否 mask 化？
4. timeout 和 failure 是否分开？
5. 动作映射是否可导出到部署端？
6. reward term 是否可单独记录？
7. 域随机化参数是否可追踪？
8. 训练策略能否在 CPU 或另一后端回放？

能回答这些问题，
说明你理解的是训练系统，
而不只是会运行命令。

---

## 累积项目：批量 Go2 训练环境模块 ⭐⭐

本章新增模块：

| 文件/模块 | 功能 |
|-----------|------|
| `scene_cfg.py` | 机器人、地形、执行器、环境数量 |
| `obs_terms.py` | policy obs、critic obs、缩放、噪声 |
| `action_terms.py` | 相对默认关节位置动作映射 |
| `reward_terms.py` | 速度跟踪、正则、足端节奏 |
| `termination_terms.py` | timeout、姿态、高度、非法接触 |
| `event_terms.py` | reset、摩擦、质量、外推、噪声 |
| `train_cfg.py` | PPO horizon、mini-batch、学习率、保存 |
| `play_cpu.py` | ONNX 策略 MuJoCo CPU 回放 |
| `eval_suite.py` | 固定命令和扰动测试集 |

验收顺序：

1. `B=1` 与单环境一致。
2. `B=512` 小批量能稳定 rollout。
3. `B=4096` 训练曲线正常。
4. 关闭域随机化能学会平地。
5. 打开域随机化后鲁棒性提升。
6. 导出 ONNX 后 CPU 回放通过固定评估套件。

---

## 延伸阅读 ⭐

| 资料 | 难度 | 阅读重点 |
|------|------|----------|
| MuJoCo 官方 MJX 文档：`https://mujoco.readthedocs.io/en/latest/mjx.html` | ⭐⭐⭐ | `put_model`、`make_data`、`vmap`、MJX-Warp 限制 |
| MuJoCo Warp 仓库：`https://github.com/google-deepmind/mujoco_warp` | ⭐⭐⭐ | Warp 后端定位、功能覆盖、不可微边界 |
| mjlab 官方仓库：`https://github.com/mujocolab/mjlab` | ⭐⭐ | Manager-based API、demo、训练命令 |
| mjlab 论文：`https://arxiv.org/abs/2601.22074` | ⭐⭐⭐ | 框架动机、组合式环境、GPU robot learning |
| S1 MuJoCo 核心引擎 | ⭐⭐ | `mjModel/mjData`、MJCF、CPU 调试 |
| S3 MuJoCo GPU 生态 | ⭐⭐ | MJX-JAX、MuJoCo Warp、Isaac Lab、Playground 等 GPU 仿真路线选型 |
| 腿足简化模型理论 | ⭐⭐⭐ | LIPM/SRBD/Centroidal 与 locomotion 状态设计 |
| 多模态 MPC | ⭐⭐⭐ | 代价、约束、模式和 RL 环境设计的边界 |

---

### ⚠️ 实战陷阱：只看总奖励不看分项 ⭐

MJLab 这类 manager 化训练环境会把 reward 拆成多个 term。若只看总奖励，速度跟踪、动作惩罚、接触节奏和生存奖励之间的互相抵消会被隐藏。训练日志必须记录每个 term 的均值、方差和关闭后的消融结果。

### ⚠️ 实战陷阱：reset 随机化改变了观测分布 ⭐

随机化不只改变物理参数，也会改变策略看到的状态分布。若初始姿态、地形、命令和质量同时大范围随机，策略可能在站稳之前就被失败样本淹没。更稳妥的做法是先固定 reset，再逐项打开随机化。

### ⚠️ 实战陷阱：批量训练成功不代表部署接口正确 ⭐

训练环境里的动作往往是归一化目标，部署端可能需要关节位置、速度或力矩命令。导出策略前必须写明动作反归一化、默认姿态、PD 增益和控制频率，否则 CPU 回放和真机部署会出现同一个策略控制两套物理接口的问题。
