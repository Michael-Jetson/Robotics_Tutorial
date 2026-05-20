> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 S3 章：GPU 并行仿真生态——从环境批量化到多仿真器验证

| 元信息 | 值 |
|--------|-----|
| 难度 | ⭐⭐⭐（GPU 执行模型 + RL 采样吞吐 + 多仿真器工程） |
| 预计时间 | 3 周（30-40 小时） |
| 前置依赖 | S1 MuJoCo 核心引擎、S2 MuJoCo 交互式控制、腿足/70_腿足简化模型理论、复合/30_多模态MPC、至少一次 PPO/SAC 训练经验 |
| 下游章节 | S04 可微分仿真理论、S05 可微分 MPC、腿足 RL 与 MPC 混合、复合全身控制策略训练 |
| 章节主线 | 为什么 GPU 并行仿真能把 RL 训练从“等待数据”变成“消耗数据”，以及这种速度背后有哪些物理、数值和工程代价 |

> **资料口径**：生态项目的版本号、支持矩阵、安装方式和硬件要求变化很快。
> 本章只给稳定的定位差异与工程判断。
> 具体版本、支持特性和命令行参数以各项目官方文档或官方仓库为准。
> 重要资料入口包括 MuJoCo MJX 文档、MuJoCo Warp 仓库、Isaac Lab 文档、Brax 仓库和 Genesis 文档。

---

## S3.0 前置自测 ⭐

📋 **答不出 ≥ 2 题 → 先回 S1/S2 或 RL 基础章节复习**

| # | 问题 | 预期关键词 |
|---|------|-----------|
| 1 | `mjModel` 与 `mjData` 的职责分别是什么？为什么这个划分对批量仿真很重要？ | 只读模型、可变状态、Model 可共享、Data 可复制 |
| 2 | PPO 中 `num_envs × horizon` 决定了什么？为什么环境数量不足会让 GPU 策略网络吃不饱？ | rollout batch、采样吞吐、矩阵批量计算 |
| 3 | 四足控制中常见的 `sim_dt=0.002s`、`decimation=10`、`control_dt=0.02s` 分别表示什么？ | 物理步长、动作保持步数、策略频率 |
| 4 | 为什么在 GPU 仿真中频繁 `.cpu().numpy()` 会破坏吞吐？ | 设备同步、PCIe 拷贝、流水线停顿 |
| 5 | 域随机化的目标是什么？它与“把仿真调得更像真实”有什么区别？ | 鲁棒性、覆盖真实、不是单点拟合 |

## S3.0.1 本章目标 ⭐

学完这一章后，你应该能够：

1. **解释** GPU 并行仿真的真正加速来源：环境批量化、SIMT 执行、张量驻留和减少 Python 调度，而不是“单个机器人变得无限快”。
2. **区分** Isaac Gym、Isaac Lab、MJX、MuJoCo Warp、Brax、Genesis 等生态的定位差异，并能根据任务选择合适工具。
3. **设计** 一个高吞吐 RL 采样循环，正确处理控制频率、物理子步、GPU/CPU 数据拷贝、随机化、日志和评估。
4. **诊断** GPU 仿真常见故障：吞吐异常低、显存爆掉、随机种子不复现、策略在 sim2sim 中崩溃、性能剖析结果被同步点污染。

## S3.0.2 章节地图 ⭐

本章按”动机 → 原理 → 选型 → 工程 → 验证 → 实践”递进。下表给出每一节的主题域和关键词，帮助读者按需跳转。

| 小节 | 主题域 | 关键词 | 难度 |
|------|--------|--------|------|
| S3.1 | 动机 | 采样瓶颈、样本预算、吞吐 vs 网络 | ⭐⭐⭐ |
| S3.2 | 原理 | 环境批量化、SoA/AoS、固定形状 | ⭐⭐⭐ |
| S3.3 | 原理 | SIMT、分支分歧、JIT、CUDA Graph | ⭐⭐⭐⭐ |
| S3.4 | 选型 | Isaac Gym/Lab、MJX、Warp、Brax、Genesis | ⭐⭐⭐ |
| S3.5 | 工程 | sim_dt、decimation、policy_dt、吞吐公式 | ⭐⭐⭐ |
| S3.6 | 工程 | 设备驻留、PCIe 拷贝、Tensor API | ⭐⭐⭐ |
| S3.7 | 方法 | 域随机化、课程调度、分布覆盖 | ⭐⭐⭐ |
| S3.8 | 工程 | 随机种子、确定性、统计复现 | ⭐⭐⭐ |
| S3.9 | 调试 | 性能剖析、瓶颈图谱、异步计时 | ⭐⭐⭐ |
| S3.10 | 验证 | sim2sim、跨引擎检查、接口对齐 | ⭐⭐⭐ |
| S3.11 | 实践 | GPU 训练流水线最小闭环 | ⭐⭐⭐ |
| S3.12 | 总结 | 核心结论、判断力回顾 | ⭐⭐ |

```text
为什么需要 GPU 并行仿真
    │
    ├── S3.1 采样瓶颈：RL 训练为什么不是”算网络”这么简单
    │
    ├── S3.2 环境批量化：num_envs 不是超参数，而是计算模型
    │
    ├── S3.3 SIMT 与向量化：GPU 喜欢同形状、同路径、少分支
    │
    ├── S3.4 生态定位：Isaac Gym/Lab、MJX、Brax、Genesis、Warp
    │
    ├── S3.5 控制频率与采样吞吐：sim_dt、decimation、policy_dt
    │
    ├── S3.6 CPU/GPU 数据拷贝：性能崩溃最常见原因
    │
    ├── S3.7 随机化：从单点标定到分布覆盖
    │
    ├── S3.8 复现性：种子、异步、归约顺序和确定性
    │
    ├── S3.9 性能剖析：先测量，再优化
    │
    ├── S3.10 多仿真器验证：同后端、CPU reference 与跨引擎检查
    │
    ├── S3.11 累积项目：GPU 训练流水线最小闭环
    │
    └── S3.12 本章小结：把吞吐、物理和部署连成闭环
```

| 小节 | 核心问题 | 层级 |
|------|----------|------|
| S3.1 | 为什么仿真吞吐比网络更重要 | 动机 |
| S3.2 | num_envs 改变了什么计算形状 | 原理 |
| S3.3 | GPU 执行模型对代码有什么约束 | 原理 |
| S3.4 | 不同框架解决什么问题 | 选型 |
| S3.5 | sim_dt 和 policy_dt 怎么配合 | 工程 |
| S3.6 | 什么操作会破坏 GPU 流水线 | 工程 |
| S3.7 | 随机化为什么是训练而非标定 | 方法 |
| S3.8 | 种子和确定性怎么保证 | 工程 |
| S3.9 | 怎么定位慢在哪里 | 调试 |
| S3.10 | 训练成功后怎么验证 | 验证 |
| S3.11 | 把上面连起来 | 实践 |
| S3.12 | 全章回顾 | 总结 |

---

## S3.1 为什么机器人 RL 需要 GPU 并行仿真 ⭐⭐⭐

> **这一节解决什么问题**：为什么强化学习训练机器人策略时，仿真吞吐常常比神经网络本身更重要？

### 从控制器调参到数据生产 ⭐

回顾腿足/70_腿足简化模型理论：

四足 MPC 不直接优化所有关节状态，而是用 LIPM、SRBD 或 centroidal model 降维。

这种做法的核心动机是实时性。

MPC 需要在几十毫秒内给出动作。

它愿意牺牲一部分模型完整性，换取控制频率。

RL 训练面对的是另一个实时性问题。

训练时并不要求每一步在真机 20 ms 内算完。

训练时要求在有限墙钟时间内产生足够多的交互样本。

PPO、SAC、TD3 这类算法本质上都是“用数据修正策略”。

如果仿真器每秒只能产生几千步，策略网络再快也没有足够样本可学。

如果仿真器每秒能产生几百万步，策略网络、优势估计、回放缓冲和日志系统都会变成新的瓶颈。

这就是 GPU 并行仿真的基本出发点：

把单个环境的仿真循环扩展成上千个同构环境的批量循环。

### 一个粗略但有用的样本预算 ⭐⭐

考虑一个四足 locomotion PPO 训练任务：

| 参数 | 典型含义 | 示例值 |
|------|---------|--------|
| `num_envs` | 并行环境数量 | 4096 |
| `horizon` | 每轮 rollout 的步数 | 24 |
| `policy_dt` | 策略动作周期 | 0.02 s |
| `sim_dt` | 物理积分步长 | 0.002 s |
| `decimation` | 每个动作保持多少个物理步 | 10 |
| `updates` | PPO 更新轮数 | 10000 |

单轮 rollout 产生：

$$
N_{\text{samples}} = N_{\text{env}} \cdot H
$$

代入上表：

$$
4096 \times 24 = 98304
$$

也就是说，一次 PPO 更新前就能获得接近十万条 transition。

若只有 16 个 CPU 环境：

$$
16 \times 24 = 384
$$

两者相差 256 倍。

这个差距并不来自策略网络参数量。

它来自环境数量。

### 如果不批量化会怎样 ⭐

假设用 Python 循环逐个环境步进：

```python
for env in envs:
    obs = env.get_observation()
    action = policy(obs)
    env.step(action)
```

这段写法直观，但它有三个根本问题：

1. Python 每次循环都有解释器调度开销。
2. 每个环境单独调用物理引擎，无法形成大批量 GPU kernel。
3. 策略网络每次只处理一个观测，矩阵乘法规模太小，GPU 利用率很低。

GPU 的优势是一次调度处理大量同构数据。

逐环境调用相当于让一条高速公路每次只过一辆车。

道路本身很宽，但调度方式把宽度浪费掉了。

### “单个仿真更快”不是核心 ⭐⭐

GPU 并行仿真容易被误解为：

“GPU 让一个机器人仿真得更快。”

这只说对了一小部分。

对机器人 RL 更关键的是：

“GPU 让许多相同结构的机器人一起前进。”

单个四足机器人的一步物理仿真包含：

- 正运动学
- 碰撞检测
- 接触约束求解
- 执行器模型
- 积分
- 传感器更新

这些计算量并不总是足以填满 GPU。

但 4096 个四足机器人同时做这件事，计算图就足够大。

因此 GPU 仿真的主战场不是“一个环境低延迟”，而是“大批量环境高吞吐”。

> **本质洞察**：GPU 并行仿真的本质不是把物理时间变快，而是把许多独立世界折叠成一个大张量计算。
> 单个环境仍然要遵守积分稳定性、接触求解和控制频率。
> 加速来自横向复制环境，而不是纵向跳过物理。

### 与数据库批处理的类比 ⭐

如果把仿真步进看作数据库写入：

| 数据库场景 | GPU 仿真场景 |
|------------|--------------|
| 每次插入一行 | 每次步进一个环境 |
| 批量插入 10000 行 | 一次步进 10000 个环境 |
| 网络往返成为瓶颈 | Python 调度和 CPU/GPU 同步成为瓶颈 |
| 表结构一致才能批量写 | 环境状态形状一致才能向量化 |

类比成立的地方：

批量化减少调度开销。

批量化让底层系统看到更大的工作块。

类比不成立的地方：

物理仿真不是纯数据写入。

接触数量、碰撞分支和终止条件会让每个环境的执行路径不同。

GPU 仿真必须把这些“不整齐”重新编码成固定形状或掩码。

### 采样吞吐与学习稳定性的关系 ⭐⭐

吞吐高不等于训练一定好。

吞吐只是让算法更快看到更多状态。

策略是否能学好还取决于：

| 维度 | 高吞吐的帮助 | 高吞吐不能解决的问题 |
|------|--------------|----------------------|
| 探索 | 同时尝试更多初始状态 | 奖励设计错误仍然会学偏 |
| 稳定性 | 大 batch 降低梯度方差 | 物理模型错误仍然会迁移失败 |
| 课程学习 | 快速推进难度 | 课程过快仍会遗忘 |
| 随机化 | 覆盖更多参数组合 | 随机化范围不合理会破坏任务 |
| 调参 | 缩短实验周期 | 错误指标会让优化方向错误 |

反事实地看：

如果没有高吞吐仿真，机器人 RL 训练往往会被样本等待时间主导。

工程师会倾向于减少随机化、缩短 episode、降低评估次数。

这些“省时间”的做法会让策略更容易过拟合单一仿真条件。

GPU 并行仿真让训练时间下降，但也会放大错误设置的影响。

错误奖励在 16 个环境里可能只是慢慢学坏。

错误奖励在 4096 个环境里会很快学坏。

### 练习 S3.1 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 设 `num_envs=8192`、`horizon=32`、每次 PPO 更新训练 5 个 epoch。估算一次更新处理多少 transition，并说明为什么 minibatch 设计会影响 GPU 利用率。 | ⭐ |
| 2 | 设计一个实验，对比 `num_envs=512/2048/8192` 时的 wall-clock training curve。要求同时记录 reward、SPS、显存占用和策略失败模式。 | ⭐⭐ |
| 3 | 跨章综合题：结合腿足/70 的 SRBD 频率预算和复合/30 的 MPC 代价权重思想，解释为什么 RL 策略训练可以用 50 Hz 动作频率，而真机底层电流环仍要运行在更高频率。 | ⭐⭐⭐ |

### 常见陷阱 S3.1 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念误区 | 把 GPU 仿真理解为“物理精度更高” | 策略在另一个仿真器中失败 | GPU 改变的是执行方式，不自动提升接触模型准确性 | 做 sim2sim 验证和物理参数敏感性分析 |
| 编程陷阱 | 每个环境单独调用策略网络 | GPU 利用率很低，SPS 上不去 | 矩阵乘法 batch 太小，调度开销占主导 | 把观测堆成 `[num_envs, obs_dim]` 一次前向 |
| 思维陷阱 | 只看 reward 不看吞吐和失败模式 | 训练曲线好看但真机不稳 | reward 是优化目标，不是物理诊断 | 同时记录接触力、动作幅值、跌倒原因和 sim2sim 结果 |

---

## S3.2 环境批量化：从数组形状理解并行仿真 ⭐⭐⭐

> **这一节解决什么问题**：`num_envs` 为什么不只是“开多少个环境”，而是整个训练程序的数据结构中心？

### 单环境状态与批量状态 ⭐⭐

一个单环境四足状态可以写成：

$$
x = [q, \dot{q}, c, s]
$$

其中：

- $q$ 是广义坐标。
- $\dot{q}$ 是广义速度。
- $c$ 是接触相关状态。
- $s$ 是传感器或任务状态。

单环境观测通常是：

$$
o \in \mathbb{R}^{d_{\text{obs}}}
$$

批量化后，观测变成：

$$
O \in \mathbb{R}^{N_{\text{env}} \times d_{\text{obs}}}
$$

动作变成：

$$
A \in \mathbb{R}^{N_{\text{env}} \times d_{\text{act}}}
$$

奖励变成：

$$
r \in \mathbb{R}^{N_{\text{env}}}
$$

终止标志变成：

$$
d \in \{0,1\}^{N_{\text{env}}}
$$

这四个张量是 GPU RL 环境的基本接口。

如果一个环境框架无法自然返回这些批量张量，它就很难充分利用 GPU。

### 批量化的核心不是复制代码 ⭐⭐

错误想法是：

“我把单环境代码复制 N 份就批量化了。”

真正的批量化要求：

1. 状态数据连续存储。
2. 每一步用张量操作同时更新所有环境。
3. reset、randomization、reward、termination 都接受环境索引集合。
4. 策略网络一次处理整个 batch。
5. 日志只抽样或聚合，不把全量状态拉回 CPU。

### SoA 与 AoS ⭐⭐

在 CPU 面向对象程序里，常见布局是 AoS：

```cpp
struct EnvState {
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd obs;
    double reward;
    bool done;
};

std::vector<EnvState> envs;
```

这种写法容易理解。

但 GPU 更喜欢 SoA：

```python
q = torch.zeros(num_envs, nq, device="cuda")
dq = torch.zeros(num_envs, nv, device="cuda")
obs = torch.zeros(num_envs, obs_dim, device="cuda")
rew = torch.zeros(num_envs, device="cuda")
done = torch.zeros(num_envs, dtype=torch.bool, device="cuda")
```

AoS 是“一行一个环境”。

SoA 是“一张表一个字段”。

GPU kernel 访问同一个字段时，SoA 更容易形成连续内存访问。

### 为什么 reset 不能破坏批量化 ⭐⭐

每个环境会在不同时间终止。

如果用 Python 写：

```python
for i in range(num_envs):
    if done[i]:
        reset_one_env(i)
```

批量执行会被打散。

GPU 友好的写法是：

```python
done_ids = torch.nonzero(done, as_tuple=False).squeeze(-1)
reset_envs(done_ids)
```

其中 `reset_envs` 也应该是批量操作。

它一次重置一组环境，而不是逐个环境调用。

### 批量环境最小骨架 ⭐⭐

```python
import torch


class BatchedRobotEnv:
    def __init__(self, num_envs: int, obs_dim: int, act_dim: int, device: str = "cuda"):
        self.num_envs = num_envs
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.device = torch.device(device)

        # 所有环境共享同一组张量，避免 Python 对象列表带来的调度开销
        self.q = torch.zeros(num_envs, 19, device=self.device)
        self.dq = torch.zeros(num_envs, 18, device=self.device)
        self.obs = torch.zeros(num_envs, obs_dim, device=self.device)
        self.rew = torch.zeros(num_envs, device=self.device)
        self.done = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        self.episode_step = torch.zeros(num_envs, dtype=torch.int32, device=self.device)

    def compute_observation(self):
        # 真实项目中这里会拼接 IMU、关节位置、关节速度、上一步动作和命令
        # 关键点：不要对每个环境单独构造 observation
        self.obs[:, 0:19] = self.q
        self.obs[:, 19:37] = self.dq
        return self.obs

    def step(self, action: torch.Tensor):
        # action 的形状必须是 [num_envs, act_dim]
        # 这里用简化积分说明数据流，真实项目应调用物理后端的批量 step
        torque = torch.clamp(action, -1.0, 1.0)
        self.dq[:, : self.act_dim] += 0.01 * torque
        self.q[:, : self.act_dim] += 0.02 * self.dq[:, : self.act_dim]

        # 奖励也必须批量计算
        self.rew = -torch.sum(torque * torque, dim=1)

        # 终止条件用张量逻辑表达
        self.episode_step += 1
        self.done = self.episode_step > 1000

        # 批量重置，避免 Python 逐环境循环
        done_ids = torch.nonzero(self.done, as_tuple=False).squeeze(-1)
        if done_ids.numel() > 0:
            self.reset(done_ids)

        return self.compute_observation(), self.rew, self.done

    def reset(self, env_ids: torch.Tensor):
        # env_ids 本身也放在 GPU 上，避免索引时触发设备拷贝
        self.q[env_ids] = 0.0
        self.dq[env_ids] = 0.0
        self.episode_step[env_ids] = 0
        self.done[env_ids] = False
```

这段代码不是完整物理仿真器。

它展示的是批量环境的基本数据形状。

真实后端可以是 PhysX、MuJoCo、MJX、Warp、Genesis 或自研 CUDA kernel。

上层训练循环看到的仍然应该是同样的批量接口。

### 固定形状与动态接触 ⭐⭐

机器人接触是批量化最大的麻烦之一。

一个环境可能没有脚接触。

另一个环境可能四只脚都接触。

机械臂抓取时，接触点数量还会随几何形状变化。

CPU 仿真器可以用动态数组保存接触列表。

GPU 批量仿真更常见的做法是：

| 方法 | 思路 | 优点 | 代价 |
|------|------|------|------|
| 固定最大接触数 | 为每个环境预留 `max_contacts` | 形状稳定，便于 JIT | 浪费显存，可能 overflow |
| 全局接触缓冲 | 所有环境共享一个大 buffer | 资源利用更好 | 索引复杂 |
| 接触传感器接口 | 不暴露原始接触列表，只读传感器 | API 稳定 | 调试细节变少 |
| CPU 验证 | GPU 训练后在 CPU 仿真器中复查 | 物理可检查 | 速度低 |

MJX 文档中把 `mjx.Model` / `mjx.Data` 放到设备端，并说明 batch 维度可以用于 RL 高吞吐和模型随机化。

Genesis 文档也把 `n_envs` 作为并行场景构建参数。

这些 API 看起来不同，但核心都是固定批量维度。

### 练习 S3.2 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 将一个单环境 reward 函数改写成批量 reward 函数，输入形状为 `[N, obs_dim]` 和 `[N, act_dim]`。要求不出现 Python 逐环境循环。 | ⭐ |
| 2 | 设计一个批量 reset 接口。要求支持：只重置失败环境、给每个重置环境采样不同初始姿态、保留未失败环境状态。 | ⭐⭐ |
| 3 | 对比 AoS 与 SoA。用 1000 个环境、每个 48 维状态，估算两种布局在 GPU 连续读写上的差异，并说明为什么 C++ 面向对象布局不一定适合 GPU。 | ⭐⭐⭐ |

### 常见陷阱 S3.2 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程陷阱 | `done.cpu().numpy()` 后在 CPU 上找重置索引 | SPS 明显下降 | 每步强制同步 GPU | 用 `torch.nonzero(done)` 留在 GPU |
| 概念误区 | 以为 batch 维度只属于策略网络 | 仿真、奖励、reset 仍然逐环境执行 | 环境没有真正向量化 | 让状态、动作、奖励、终止全链路批量化 |
| 思维陷阱 | 认为 `num_envs` 越大越好 | 显存爆掉或 reward 下降 | batch 过大导致更新频率、课程推进和归一化统计变化 | 同时调 `num_envs`、horizon、minibatch 和学习率 |

---

## S3.3 SIMT、向量化与 GPU 物理的执行模型 ⭐⭐⭐⭐

> **这一节解决什么问题**：为什么 GPU 仿真喜欢“同构环境”，又为什么复杂接触和分支会让并行效率下降？

### SIMT 的直觉 ⭐⭐

GPU 的执行方式常被概括为 SIMT：

Single Instruction, Multiple Threads。

多个线程在同一时刻执行相同指令，但处理不同数据。

把它放到机器人仿真里：

| SIMT 元素 | 仿真对应物 |
|-----------|------------|
| 同一条指令 | 同一个物理 step kernel |
| 多个线程 | 多个环境、多个刚体、多个约束 |
| 不同数据 | 每个环境的状态、动作、接触 |
| 分支分歧 | 有的环境接触、有的环境飞行 |

GPU 喜欢：

- 同样的机器人结构。
- 同样的状态维度。
- 同样的动作维度。
- 同样的控制流程。
- 尽量少的动态内存分配。

GPU 不喜欢：

- 每个环境不同模型拓扑。
- 每步创建不同大小数组。
- 接触数无限动态增长。
- Python 回调插在仿真循环中。
- 大量 `if env_id == ...` 分支。

### 向量化不是自动发生的 ⭐⭐

写成 NumPy 或 PyTorch 并不自动高效。

下面是常见错误：

```python
def reward_bad(obs, action):
    rewards = []
    for i in range(obs.shape[0]):
        # 错误：Python 循环逐环境处理
        alive = 1.0 if obs[i, 2] > 0.25 else 0.0
        effort = torch.sum(action[i] ** 2)
        rewards.append(alive - 0.001 * effort)
    return torch.tensor(rewards, device=obs.device)
```

这段代码表面上用了 Torch。

但主体仍然是 Python 循环。

GPU 无法把它变成一个大 kernel。

正确写法：

```python
def reward_good(obs, action):
    # obs[:, 2] 是所有环境的 base height
    # torch.where 在 GPU 上批量处理分支
    alive = torch.where(obs[:, 2] > 0.25, 1.0, 0.0)

    # dim=1 表示对每个环境的动作维度求和
    effort = torch.sum(action * action, dim=1)

    # 返回形状 [num_envs]，不发生 CPU 拷贝
    return alive - 0.001 * effort
```

### 分支分歧：接触仿真的隐形成本 ⭐⭐⭐

接触丰富任务会让 GPU 线程走不同路径。

例如四足机器人：

- 环境 A 正在站立，四足接触。
- 环境 B 正在跳跃，无足接触。
- 环境 C 一只脚擦过地面，接触瞬间产生又消失。
- 环境 D 侧翻，身体和地面多点接触。

如果 kernel 内部根据接触情况分支，线程束中的部分线程会等待其他分支。

这叫 branch divergence。

它不会让结果错误。

它会降低吞吐。

### 接触求解器的并行性差异 ⭐⭐⭐

不同物理求解器对 GPU 的友好程度不同。

| 求解思路 | 并行性 | 典型特点 |
|----------|--------|----------|
| PGS 类迭代 | 较难并行 | 顺序更新约束，依赖上一约束结果 |
| Jacobi 类迭代 | 易并行 | 同步更新，可能需要更多迭代 |
| CG / Newton 类方法 | 中到高 | 大矩阵操作更适合 GPU，但实现复杂 |
| PBD / XPBD | 易并行 | 常用于图形和软体，物理参数解释需谨慎 |
| 软接触凸优化 | 取决于实现 | 接触平滑，梯度和逆动力学更友好 |

这也是为什么“物理精度”和“GPU 吞吐”经常存在张力。

更严格的接触模型可能带来更复杂的求解。

更简单的求解可能带来更快的训练，但 sim2real gap 需要额外处理。

### JIT 编译：把 Python 计算图变成设备程序 ⭐⭐⭐

MJX 使用 JAX/XLA。

JAX 的 `jit` 会把函数编译成设备端计算图。

编译后运行很快。

但第一次编译可能很慢。

更重要的是，形状变化会触发重新编译。

```python
import jax
import jax.numpy as jnp


@jax.jit
def batched_pd(q, dq, q_des, kp, kd):
    # q、dq、q_des 的形状都是 [num_envs, dof]
    # 这段函数被编译后会在设备端批量执行
    return kp * (q_des - q) - kd * dq


num_envs = 4096
dof = 12
q = jnp.zeros((num_envs, dof))
dq = jnp.zeros((num_envs, dof))
q_des = jnp.zeros((num_envs, dof))
tau = batched_pd(q, dq, q_des, 30.0, 1.0)
```

如果下一次把 `num_envs` 改成 5000，JAX 可能需要重新编译。

因此 GPU 仿真项目通常会固定：

- `num_envs`
- `obs_dim`
- `act_dim`
- `max_contacts`
- `horizon`
- 传感器数量

### CUDA Graph 的直觉 ⭐⭐⭐

许多仿真循环每步都调用同一串 kernel：

1. 刷新状态。
2. 计算观测。
3. 策略前向。
4. 应用动作。
5. 物理步进。
6. 计算奖励。
7. reset。

每个 kernel 都有启动开销。

CUDA Graph 的思路是把这串调用录下来，下次直接重放。

它像把一套固定动作编排成舞蹈。

只要动作顺序和内存地址稳定，就能减少调度开销。

但如果每步 tensor 地址变了、形状变了、分支结构变了，图捕获就会失效或频繁重建。

### 练习 S3.3 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 将 `reward_bad` 改写成不含 Python 循环的张量表达式，并用 `torch.cuda.Event` 对比时间。 | ⭐ |
| 2 | 解释为什么“每个环境随机使用不同机器人 URDF”会破坏 GPU 并行效率。给出一种保留随机性的替代设计。 | ⭐⭐ |
| 3 | 阅读 MJX 文档中关于 `mjx.Model` / `mjx.Data` 的结构字段说明，区分哪些字段改变会触发 JIT 重新编译，哪些字段可以运行时随机化。 | ⭐⭐⭐⭐ |

### 常见陷阱 S3.3 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程陷阱 | 在 `jax.jit` 内部根据 Python list 长度创建数组 | 反复编译，训练启动很慢 | JAX 需要静态形状 | 固定 shape，用 mask 表示有效项 |
| 概念误区 | 认为 `torch.where` 等于 CPU `if` | 对批量分支理解混乱 | `torch.where` 两侧表达式通常都会构造 | 分支代价高时拆分环境或重写公式 |
| 思维陷阱 | 把所有物理差异都随机成拓扑差异 | GPU 利用率下降，代码复杂 | 拓扑差异破坏同构批量 | 优先随机质量、摩擦、延迟、噪声等数值参数 |

---

## S3.4 生态定位：Isaac Gym、Isaac Lab、MJX、MuJoCo Warp、Brax、Genesis ⭐⭐⭐

> **这一节解决什么问题**：这些框架到底谁解决什么问题，为什么不能只用“快/慢”排序？

### 先按抽象层级分类 ⭐⭐

GPU 仿真生态可以按抽象层级理解：

```text
物理内核层
    PhysX GPU
    MuJoCo CPU
    MJX-JAX
    MuJoCo Warp
    Genesis physics
    Brax pipelines

环境封装层
    Isaac Gym task API
    Isaac Lab Manager/Direct workflow
    MuJoCo Playground tasks
    Genesis gym-style tasks
    自研 vector env

训练算法层
    rsl_rl
    Brax training
    Stable-Baselines3 风格接口
    自研 PPO/SAC/TD3
    分布式训练框架

验证与部署层
    CPU MuJoCo 复查
    Isaac Sim 传感器和渲染
    真机 SDK
    ROS2/ros2_control
    ONNX/TensorRT/LibTorch
```

许多争论来自把不同层级混在一起。

Isaac Lab 不是单纯的物理引擎。

MJX 不是完整的任务库。

Brax 既有物理历史，又有训练算法价值。

Genesis 同时覆盖物理、渲染、并行场景和数据生成愿景。

### Isaac Gym：GPU tensor API 的历史转折点 ⭐⭐⭐

Isaac Gym 的核心贡献是把机器人 RL 的整个循环放到 GPU 附近：

- 物理仿真在 GPU 上跑。
- 物理状态通过 tensor 暴露。
- 观测和奖励可以在 GPU 上计算。
- 策略网络也在 GPU 上训练。

它改变了机器人 RL 的实验尺度。

在 Isaac Gym 之前，许多项目仍用 CPU 多进程环境收集样本。

CPU 多进程可以并行，但跨进程通信、状态序列化和 CPU/GPU 拷贝开销明显。

Isaac Gym 展示了另一种模式：

把环境状态做成大张量。

让物理、奖励和策略共处一个设备流水线。

适合 Isaac Gym 的场景：

| 场景 | 理由 |
|------|------|
| 复现实验室早期 legged_gym 生态 | 大量旧代码直接基于 Isaac Gym |
| 学习 GPU tensor API 思维 | API 比较直接，能看清底层张量 |
| 无需复杂相机和 USD 场景 | 早期 locomotion 任务主要是 proprioception |

不适合作为新项目唯一选择的原因：

- 生态重心已经向 Isaac Lab、Isaac Sim 和多后端方向迁移。
- 场景表达、传感器、资产管理和任务配置能力有限。
- 很多工程能力需要自行搭建。

这里的判断不是“Isaac Gym 不好”。

更准确地说：

Isaac Gym 是 GPU 机器人 RL 的重要起点。

Isaac Lab 是更完整的工程化上层。

### Isaac Lab：从快速训练到机器人学习平台 ⭐⭐⭐

Isaac Lab 的定位是机器人学习框架。

它继承了 Isaac Gym 的 GPU 并行训练思想，同时加入：

- Isaac Sim / Omniverse 场景体系。
- USD 资产和渲染。
- Manager-based environment。
- Direct workflow。
- 观测、奖励、终止、随机化、课程学习的模块化管理。
- 传感器、相机、ray caster、接触传感器等能力。
- 多任务、多机器人和多学习库接口。

Manager-based workflow 的价值在大型项目中非常明显。

当一个任务包含几十个 reward term、十几个 observation term、多种随机化事件和课程学习时，把所有逻辑写在一个 `step()` 里会很难维护。

Manager 把关注点拆开：

| Manager | 负责内容 |
|---------|----------|
| ObservationManager | 观测项计算、噪声、拼接 |
| RewardManager | 奖励项计算、权重、日志 |
| TerminationManager | 失败条件和超时条件 |
| EventManager | reset 随机化、push、外部扰动 |
| CurriculumManager | 训练难度调度 |

这种设计与复合/30_多模态MPC 中的代价项拆分很像。

MPC 里把 CoM 跟踪、末端跟踪、力跟踪、自碰撞约束分成不同 term。

Isaac Lab 把 RL 环境里的观测、奖励和随机化也拆成可组合 term。

相似处是分离关注点。

不同处是 MPC term 进入优化器求解。

RL term 进入采样和梯度估计。

### MJX：MuJoCo 物理进入 JAX/XLA 与 Warp 世界 ⭐⭐⭐

MJX 的定位不是“一个单一 JAX 后端”，而是 MuJoCo 提供 JAX API 的硬件加速入口。

官方文档把 MJX 描述为 MuJoCo 的 JAX API，并明确区分两个实现方向：

- **MJX-JAX**：纯 JAX 实现，可在 XLA 支持的硬件上运行，适合研究 JAX 训练栈和局部梯度。
- **MJX-Warp / MuJoCo Warp**：面向 NVIDIA GPU 的高吞吐实现，不应当作自动微分路线使用。

因此工程上要分开看三种能力：

1. `vmap`：MJX-JAX 里批量化许多环境。
2. `jit`：MJX-JAX 里把 step 和 reward 编译成设备程序；MJX-Warp 侧更关注 CUDA/Warp 图执行和吞吐。
3. `grad`：只应写成 MJX-JAX 路线的能力，并且要核对接触、传感器和模型特性的可微覆盖。

MJX 适合：

| 场景 | 价值 |
|------|------|
| 可微分仿真 | 使用 MJX-JAX，让梯度穿过可支持的物理 step |
| system identification | 在 MJX-JAX 覆盖范围内，用梯度拟合质量、摩擦、阻尼等参数 |
| 研究 JAX 训练栈 | 与 JAX RL、Brax training、XLA 工具链自然衔接 |
| 中小规模 MuJoCo 模型批量训练 | 保留 MuJoCo 模型语义 |

MJX 的代价：

- 静态形状约束更强。
- 首次 JIT 编译可能耗时。
- 并非所有 MuJoCo 特性都有同等支持。
- JAX 调试方式与传统 Python 不同。
- MJX-Warp 追求前向吞吐，不支持自动微分。

### MuJoCo Warp：面向 NVIDIA GPU 的 MuJoCo 执行路径 ⭐⭐⭐

MuJoCo Warp 的定位是 GPU 优化的 MuJoCo 实现，面向 NVIDIA 硬件。

它与 MJX-JAX 的核心区别不是“谁更高级”，而是目标不同：

| 维度 | MJX-JAX | MuJoCo Warp |
|------|---------|-------------|
| 编程生态 | JAX / XLA | NVIDIA Warp / CUDA |
| 主要优势 | 可微分、JAX 组合性 | NVIDIA GPU 前向仿真吞吐 |
| 自动微分 | MJX-JAX 支持 JAX AD | MuJoCo Warp 路径按官方文档不作为 AD 路径使用 |
| 适合任务 | 梯度研究、参数辨识、可微分 MPC 原型 | 大规模前向采样、RL 训练、接触密集任务 |
| 调试方式 | JAX tracing、JIT cache | CUDA/Warp 工具链、CUDA Graph |

对大多数 model-free RL，策略梯度不需要物理可微分。

PPO 的梯度来自 log probability 和 advantage。

SAC 的梯度来自 critic 和 actor loss。

仿真器只需要快速、稳定地产生 transition。

因此前向吞吐往往比可微性更重要。

但对 S04/S05 的可微分仿真和可微分 MPC，梯度路径会重新变得重要。

### Brax：从 JAX 物理先驱到训练库价值 ⭐⭐⭐

Brax 的历史意义很大。

它较早展示了 JAX 加速硬件上的大规模可微刚体仿真。

许多研究者第一次通过 Brax 体验到“几分钟内训练一个连续控制策略”。

但从官方仓库说明看，Brax 的重心已经更偏向 `brax/training`。

对物理模拟需求，官方说明建议使用 MJX 或 MuJoCo Warp。

这意味着 Brax 的生态定位要分开看：

| 部分 | 定位 |
|------|------|
| `brax/training` | 仍有价值的 JAX RL 训练基础设施 |
| Brax 原始物理管线 | 适合学习 JAX 物理设计和研究历史 |
| MJX 结合 Brax training | 更符合 MuJoCo 生态延续 |

因此在新机器人项目里，不要只因为“Brax 很快”就默认用 Brax 物理。

如果目标是 MuJoCo 模型、真实机器人迁移或与 Menagerie 资产对齐，MJX/MuJoCo Warp 往往更自然。

### Genesis：面向通用物理与机器人学习的一体化平台 ⭐⭐⭐

Genesis（Xian et al. 2024）是一个面向 Robotics / Embodied AI / Physical AI 的通用物理平台，由多所高校和机构联合开发。它的核心定位是用统一的 Python 接口覆盖刚体、柔性体、流体和粒子等多种物理模态，并在 GPU 上实现高吞吐并行仿真。与 MJX 和 Isaac Lab 相比，Genesis 更强调多物理统一和渲染-仿真一体化，但在具体机器人 locomotion 任务的工程成熟度上仍需逐项验证。

它的特点包括：

- Pythonic API。
- 并行场景构建。
- GPU/CPU 后端。
- 渲染和传感器能力。
- RL locomotion 示例。
- 域随机化文档。
- 可微分仿真接口。
- 更广泛材料和多物理愿景。

Genesis 的 `n_envs` API 对初学者很友好：

创建场景时指定批量环境数量。

控制接口接受带 batch 维度的 Torch 张量。

这让它非常适合学习“批量环境”概念。

同时要注意：

通用平台的覆盖范围越广，具体机器人任务的工程成熟度越需要验证。

选用 Genesis 做研究原型很有吸引力。

选用 Genesis 做真机部署训练栈时，仍要做：

- sim2sim 验证。
- 接触参数敏感性分析。
- 传感器延迟和噪声建模。
- 真实控制接口对齐。
- 训练日志和复现性记录。

### 生态定位总表 ⭐⭐

| 生态 | 最适合的问题 | 不要误用为 | 关键检查 |
|------|--------------|------------|----------|
| Isaac Gym | 理解早期 GPU tensor RL、复现旧 locomotion 代码 | 新项目的完整机器人学习平台 | 依赖版本、驱动、旧 API 兼容 |
| Isaac Lab | 大型机器人学习任务、传感器、USD 场景、模块化环境 | 轻量单文件教学脚本 | Isaac Sim 依赖、启动开销、资源要求 |
| MJX-JAX | MuJoCo + JAX + 可微分/批量研究 | 所有任务最快后端 | JIT 时间、特性支持、静态形状 |
| MuJoCo Warp | NVIDIA GPU 上大规模前向仿真 | 自动微分替代品 | 硬件、支持矩阵、接触 buffer 配置 |
| Brax | JAX RL 训练基础设施和历史物理管线 | 新 MuJoCo 物理项目默认后端 | 官方仓库维护范围 |
| Genesis | Pythonic 通用物理、并行场景、渲染/数据生成探索 | 无需验证的真机部署捷径 | 任务成熟度、sim2sim、复现设置 |
| MuJoCo CPU | 物理调试、逆动力学、精确检查、单环境验证 | 高吞吐 PPO 采样器 | 速度瓶颈、并行策略 |

### 决策流程图 ⭐⭐

按第一目标选择入口框架：

| 第一目标 | 推荐入口 | 关键判据 |
|----------|----------|----------|
| 复现 legged_gym/IsaacGym 旧代码 | Isaac Gym | 先跑通旧代码，再规划迁移 |
| 构建长期维护的大型机器人学习项目 | Isaac Lab | 模块化任务、传感器、USD、部署工具链 |
| 使用 MuJoCo 模型并需要 JAX 梯度 | MJX-JAX | vmap/jit/grad 是核心价值 |
| 使用 MuJoCo 模型并追求 NVIDIA GPU 前向吞吐 | MuJoCo Warp | 或基于它的上层环境 |
| 使用 JAX 训练算法，物理不一定用 Brax 原始管线 | Brax training + MJX/Playground 风格环境 | 训练库与物理管线分开评价 |
| Pythonic 通用物理、渲染、数据生成和快速原型 | Genesis | 并用独立仿真器做交叉验证 |
| 解释接触力、检查逆动力学、定位物理错误 | MuJoCo CPU | 单环境逐步调试 |

### 练习 S3.4 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 给定一个 Go2 纯本体感知 locomotion 任务，分别设计 Isaac Lab、MJX、Genesis 三条技术路线，并列出每条路线的主要风险。 | ⭐⭐ |
| 2 | 一个项目需要 system ID 梯度、PPO 训练和最终 CPU MuJoCo 复查。画出你会如何组合 MJX、MuJoCo Warp 和 MuJoCo CPU。 | ⭐⭐⭐ |
| 3 | 精读 Brax 官方仓库说明，解释为什么“Brax 很快”不能直接推出“新机器人项目应优先用 Brax 物理”。 | ⭐⭐ |

### 常见陷阱 S3.4 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念误区 | 按 star 数或宣传 benchmark 选仿真器 | 项目中途发现缺传感器或部署接口 | benchmark 不覆盖你的任务约束 | 先列任务需求，再选后端 |
| 编程陷阱 | 直接复制不同框架的 reward scale | 策略行为异常 | 动作定义、dt、termination 不同 | 先统一动作、频率、观测归一化 |
| 思维陷阱 | 以为一个仿真器成功就代表真机稳 | sim2real 失败 | 接触模型和执行器模型有系统偏差 | 做 sim2sim、参数扫描和延迟建模 |

---

## S3.5 RL 采样吞吐与控制频率 ⭐⭐⭐

> **这一节解决什么问题**：为什么训练速度、物理步长、动作频率和真机控制频率必须一起设计？

### 三个时间尺度 ⭐⭐

机器人学习里至少有三个时间尺度：

| 时间尺度 | 符号 | 例子 | 负责对象 |
|----------|------|------|----------|
| 物理积分步长 | `sim_dt` | 0.001-0.005 s | 仿真器稳定积分 |
| 策略动作周期 | `policy_dt` | 0.01-0.05 s | 神经网络输出动作 |
| 底层控制周期 | `motor_dt` | 0.001-0.002 s | PD/电流/力矩控制 |

通常有：

$$
policy\_dt = decimation \times sim\_dt
$$

例如：

$$
0.02 = 10 \times 0.002
$$

含义是：

策略每 20 ms 输出一次动作。

仿真器每 2 ms 积分一次。

每个动作保持 10 个物理步。

### 为什么策略不一定要 500 Hz ⭐⭐

回顾腿足简化模型：

MPC 常以 20-50 Hz 更新质心或接触力规划。

WBC 或关节 PD 可以运行在 500-1000 Hz。

这说明高层策略不必直接等于底层电机频率。

RL locomotion 也类似。

策略网络输出：

- 期望关节位置。
- 期望关节速度。
- 力矩残差。
- 足端目标。
- latent command。

底层 PD 或 WBC 在更高频率执行这些目标。

如果强行让策略以 1000 Hz 输出动作：

1. rollout 序列变得很长。
2. credit assignment 更难。
3. 策略容易学习高频抖动。
4. 训练吞吐按物理步被消耗。
5. 真机部署时通信和推理延迟压力变大。

### 吞吐公式 ⭐⭐

定义：

- $N$：并行环境数。
- $f_{\text{policy}}$：策略动作频率。
- $K$：每个动作下的物理子步数。
- $SPS_{\text{physics}}$：物理 step 每秒总数。
- $SPS_{\text{policy}}$：策略 transition 每秒总数。

有近似关系：

$$
SPS_{\text{policy}} \approx \frac{SPS_{\text{physics}}}{K}
$$

如果物理后端每秒能做 20M 个物理步，`decimation=10`，则策略 transition 约为 2M/s。

但训练算法看到的是 policy step。

奖励和终止通常也按 policy step 记录。

因此报告性能时必须写清楚：

- physics steps per second。
- policy steps per second。
- real-time factor。
- `num_envs`。
- `sim_dt`。
- `decimation`。
- 是否包含渲染。
- 是否包含策略前向和学习更新。

### 动作保持的物理意义 ⭐⭐

动作保持不是偷懒。

它是在模拟低频策略和高频执行器之间的接口。

对于位置控制策略：

```python
for _ in range(decimation):
    # 策略动作 action 被解释为期望关节位置
    # PD 控制在每个物理小步计算力矩
    torque = kp * (action + default_q - q) - kd * dq
    sim.step(torque)
```

这更接近真实机器人：

上层策略以 50 Hz 发送目标。

电机控制器以更高频率闭环。

如果仿真中每个物理步都重新采样策略动作，策略可能利用真机不存在的高频自由度。

### 频率错配的失败模式 ⭐⭐⭐

| 错配 | 表现 | 原因 | 修正 |
|------|------|------|------|
| 训练策略频率高于部署频率 | 真机动作滞后、步态散掉 | 策略依赖高频修正 | 训练时使用部署频率 |
| 训练 `sim_dt` 太大 | 接触抖动、穿透、能量爆炸 | 积分不稳定 | 减小步长或增加子步 |
| `decimation` 太大 | 动作响应迟钝 | 策略更新太慢 | 增加策略频率或改动作空间 |
| PD 增益未随 `dt` 检查 | 数值震荡 | 离散控制稳定性变化 | 重新标定 kp/kd |
| reward 按物理步和策略步混用 | 奖励尺度随 decimation 改变 | 时间积分单位不一致 | reward 明确乘以 `policy_dt` 或 `sim_dt` |

### 时间单位一致性 ⭐⭐

很多 reward 其实是连续时间积分的离散近似：

$$
J = \int_0^T \ell(x(t), u(t)) dt
$$

离散化后：

$$
J \approx \sum_k \ell(x_k, u_k)\Delta t
$$

如果 reward 没有乘以时间步长，那么改 `policy_dt` 会改变任务偏好。

例如动作惩罚：

```python
reward = forward_speed - 0.01 * torch.sum(action ** 2, dim=1)
```

当策略频率翻倍时，单位真实时间内动作惩罚次数也翻倍。

更稳妥的写法是：

```python
reward = (
    forward_speed * policy_dt
    - 0.01 * torch.sum(action ** 2, dim=1) * policy_dt
)
```

这样 reward 更接近连续时间代价。

### 练习 S3.5 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | `sim_dt=0.002s`、`decimation=8` 时策略频率是多少？如果真机只能 50 Hz 推理，这个设置是否合适？ | ⭐ |
| 2 | 修改一个 locomotion reward，使速度奖励、能耗惩罚和动作变化惩罚都显式乘以时间步长。解释这样做对调参有什么好处。 | ⭐⭐ |
| 3 | 设计一个频率扫描实验：`policy_dt=0.01/0.02/0.04s`，记录训练 reward、动作频谱和 sim2sim 稳定性。 | ⭐⭐⭐ |

### 常见陷阱 S3.5 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念误区 | 把 `sim_dt` 当作策略频率 | 训练和部署对不上 | 物理积分和策略决策是两层循环 | 同时记录 `sim_dt`、`decimation`、`policy_dt` |
| 编程陷阱 | 改 decimation 后不改 reward scale | reward 权重失衡 | 单位真实时间内奖励累计次数变化 | 连续时间代价乘以 dt |
| 思维陷阱 | 只追求高策略频率 | 策略高频抖动 | 高频动作空间更难部署 | 让策略频率贴近真实上层控制接口 |

---

## S3.6 CPU/GPU 数据拷贝：吞吐最常见的破坏点 ⭐⭐⭐

> **这一节解决什么问题**：为什么明明用了 GPU 仿真，性能却像 CPU 程序？

### 设备驻留原则 ⭐⭐

GPU 高吞吐训练有一条核心原则：

状态在哪里产生，就尽量在哪里消费。

物理状态在 GPU 产生。

观测在 GPU 计算。

策略在 GPU 前向。

动作在 GPU 写回。

奖励在 GPU 计算。

优势估计和 rollout buffer 也尽量留在 GPU。

只有日志、评估视频、少量标量统计需要回到 CPU。

### PCIe 拷贝不是唯一问题 ⭐⭐⭐

CPU/GPU 数据拷贝慢，大家都知道。

更隐蔽的是同步。

GPU kernel 默认异步执行。

Python 代码发出 kernel 后可能立即继续运行。

但以下操作会迫使 CPU 等 GPU 完成：

- `tensor.item()`
- `tensor.cpu()`
- `tensor.numpy()`
- 打印 CUDA tensor 的具体值。
- 某些计时方式没有同步却读取结果。
- 在 Python `if` 中使用 GPU tensor 的值。

同步点会破坏流水线。

它像在高速路上每隔 100 米设置一个收费站。

每个收费站本身可能只停一小会儿。

但上千次停顿会吞掉全部收益。

### 错误与正确示例 ⭐⭐

错误写法：

```python
def train_step_bad(env, policy):
    obs = env.obs
    action = policy(obs)
    obs, rew, done = env.step(action)

    # 错误：每一步把 reward 拉回 CPU，只为了写日志
    mean_rew = rew.mean().item()
    print("reward:", mean_rew)

    # 错误：把 done 转成 numpy，再做 reset 统计
    done_np = done.cpu().numpy()
    num_done = done_np.sum()
    return num_done
```

正确写法：

```python
class GpuLogger:
    def __init__(self, log_interval: int):
        self.log_interval = log_interval
        self.step = 0
        self.reward_acc = None

    def update(self, rew: torch.Tensor):
        # rew 留在 GPU 上累计，避免每步同步
        if self.reward_acc is None:
            self.reward_acc = torch.zeros((), device=rew.device)
        self.reward_acc += rew.mean()
        self.step += 1

        if self.step % self.log_interval == 0:
            # 只在低频日志点同步一次
            mean_rew = (self.reward_acc / self.log_interval).item()
            print(f"mean_reward={mean_rew:.3f}")
            self.reward_acc.zero_()


def train_step_good(env, policy, logger):
    obs = env.obs
    action = policy(obs)
    obs, rew, done = env.step(action)
    logger.update(rew)
    return obs
```

### 渲染也是数据拷贝 ⭐⭐⭐

训练时打开 viewer 常常让吞吐下降一个数量级。

原因包括：

- 渲染本身占 GPU。
- 图像从渲染管线转成训练 tensor 有开销。
- GUI 需要同步帧。
- 屏幕显示频率远低于仿真频率。
- 相机传感器生成的是大 tensor。

工程建议：

| 阶段 | 渲染策略 |
|------|----------|
| 调试单环境 | 打开 viewer，慢速观察接触和动作 |
| 大规模训练 | 关闭 viewer，只记录标量 |
| 周期评估 | 少量环境、低频录像 |
| 视觉策略训练 | 使用批量渲染接口，明确统计渲染成本 |
| sim2real 复查 | 单独评估脚本，不和训练吞吐混在一起 |

### Tensor API 的核心价值 ⭐⭐

Isaac Gym 的 tensor API、Isaac Lab 的 batched tensor view、Genesis 的 batched Torch 输入、MJX 的 JAX array，本质上都在解决同一个问题：

不要把状态取回 CPU 后再处理。

物理状态应该直接成为张量计算输入。

控制输出应该直接写回物理后端。

这就是端到端 GPU RL 的关键闭环。

### 练习 S3.6 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 在一个训练脚本中搜索 `.item()`、`.cpu()`、`.numpy()`。把每处用途分类为“必须同步”和“可延迟同步”。 | ⭐ |
| 2 | 用 `torch.cuda.Event` 测量每步日志同步对 SPS 的影响。分别测试每步打印、每 100 步打印、每 1000 步打印。 | ⭐⭐ |
| 3 | 设计一个评估录像流程：训练时不渲染，每隔固定 update 保存策略参数，评估进程独立加载参数并录制 4 个环境。 | ⭐⭐⭐ |

### 常见陷阱 S3.6 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程陷阱 | 每步 `rew.mean().item()` | GPU 利用率呈锯齿状 | 强制同步 | 低频聚合后同步 |
| 概念误区 | 认为只要 tensor 在 CUDA 上就没有拷贝 | 性能仍低 | 可能存在隐式同步和 D2D copy | 用 profiler 看 memcpy 和 synchronize |
| 思维陷阱 | 训练和可视化用同一进程同一频率 | 吞吐不稳定 | viewer 与训练争抢资源 | 训练、评估、录像解耦 |

---

## S3.7 随机化：让策略面对分布，而不是背答案 ⭐⭐⭐

> **这一节解决什么问题**：域随机化为什么是 GPU 并行仿真的天然搭档，又为什么随机化过度会毁掉训练？

### 随机化的基本目的 ⭐⭐

域随机化不是让仿真更逼真。

域随机化是让策略对物理和观测变化更鲁棒。

真实世界只是未知分布中的一个点。

如果训练分布覆盖了真实世界附近的关键变化，策略更可能迁移成功。

如果训练只拟合一个精调仿真模型，策略可能利用该模型的细节漏洞。

> **本质洞察**：随机化不是“把参数摇乱”，而是定义一个可信的训练分布。
> 分布太窄会过拟合仿真。
> 分布太宽会把任务变成噪声。
> 好的随机化不是越大越好，而是覆盖关键不确定性并保留任务可学性。

### 随机化分类 ⭐⭐

| 类别 | 参数 | 对 sim2real 的影响 |
|------|------|-------------------|
| 刚体参数 | 质量、质心、惯量 | 改变加速度和接触响应 |
| 接触参数 | 摩擦、恢复、接触刚度、阻尼 | 改变打滑、弹跳和支撑稳定性 |
| 执行器参数 | kp、kd、力矩上限、齿隙、死区 | 改变动作到运动的映射 |
| 延迟 | 观测延迟、动作延迟、通信延迟 | 改变闭环相位裕度 |
| 传感器噪声 | IMU 噪声、编码器噪声、丢包 | 改变状态估计质量 |
| 地形 | 高度场、坡度、障碍、软硬地面 | 改变接触模式 |
| 命令 | 速度指令、转向指令、目标位置 | 改变任务覆盖范围 |
| 初始状态 | 姿态、速度、关节偏差 | 提高 reset 后恢复能力 |
| 外部扰动 | push、拉力、冲击 | 提高抗扰能力 |

### 参数一致性 ⭐⭐⭐

随机化质量时，不能只随机质量标量。

惯量与质心也应保持物理一致。

否则会出现不可能的刚体：

- 质量变大但惯量不变。
- 质心移出几何体很远。
- 惯量矩阵不正定。
- 三角不等式不满足。

这类模型可能训练出奇怪策略。

它们不是鲁棒性训练，而是物理污染。

### 批量随机化示例 ⭐⭐

```python
import torch


def randomize_friction(num_envs: int, device: torch.device):
    # 每个环境采样一个地面摩擦系数
    # 使用 log-uniform 可以让比例变化更均匀
    low = torch.log(torch.tensor(0.4, device=device))
    high = torch.log(torch.tensor(1.2, device=device))
    u = torch.rand(num_envs, device=device)
    friction = torch.exp(low + (high - low) * u)
    return friction


def randomize_pd_gain(kp: torch.Tensor,
                      kd: torch.Tensor,
                      nominal_kp: torch.Tensor,
                      nominal_kd: torch.Tensor,
                      env_ids: torch.Tensor):
    # kp/kd 是当前环境使用的增益，nominal_kp/nominal_kd 是固定标称值。
    # 不要对 kp/kd 连续 *= scale，否则多次 reset 后增益会随机游走。
    # 只对重置的环境随机化，未重置环境保持连续
    device = kp.device
    n = env_ids.numel()
    dof = kp.shape[1]

    # kp 在 0.8 到 1.2 倍之间变化
    kp_scale = 0.8 + 0.4 * torch.rand(n, dof, device=device)

    # kd 随 kp 的平方根缩放，保持阻尼比大致稳定
    kd_scale = torch.sqrt(kp_scale)

    kp[env_ids] = nominal_kp[env_ids] * kp_scale
    kd[env_ids] = nominal_kd[env_ids] * kd_scale
```

注意第二个函数。

`kd` 没有独立随意随机。

同时，`kp/kd` 的随机化必须每次从标称值生成，而不是在上一轮随机结果上继续相乘。

对二阶系统，阻尼比与 $k_d / \sqrt{k_p}$ 相关。

如果 `kp` 增大但 `kd` 不变，系统可能更容易震荡。

随机化应尊重物理结构。

### 课程学习与随机化调度 ⭐⭐⭐

训练早期随机化过强，策略可能连站立都学不会。

训练后期随机化过弱，策略可能无法迁移。

一种常见做法是随机化 curriculum：

$$
\alpha(t) = \min(1, t / T_{\text{warmup}})
$$

$$
p(t) = p_0 + \alpha(t)(p_{\max} - p_0)
$$

其中 $p(t)$ 是随机化范围。

早期范围小。

随着策略能力提升，范围逐渐扩大。

### 随机化与复现的矛盾 ⭐⭐⭐

随机化提高鲁棒性。

复现要求可追踪。

两者不矛盾。

做法是记录随机化配置：

- 随机种子。
- 每个参数的分布类型。
- 每个参数的上下界。
- curriculum 进度。
- 训练代码提交版本。
- 物理后端版本。
- 模型文件 hash。

不要只记录“开了 domain randomization”。

这句话没有复现实验价值。

### 练习 S3.7 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 为 Go2 设计一组最小随机化参数：摩擦、质量、动作延迟、IMU 噪声。给出上下界和理由。 | ⭐⭐ |
| 2 | 写一个随机化调度函数，让摩擦范围从 `[0.7, 1.0]` 逐渐扩展到 `[0.4, 1.2]`。要求输出可复现。 | ⭐⭐ |
| 3 | 解释为什么随机化接触参数比随机化视觉颜色更可能影响四足本体感知 locomotion。再说明视觉策略中结论如何变化。 | ⭐⭐⭐ |

### 常见陷阱 S3.7 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念误区 | 随机化范围越大越好 | 策略保守、学不会高速运动 | 任务分布过宽 | 从可学范围开始，用课程扩大 |
| 编程陷阱 | reset 时随机化所有环境 | 未终止环境状态突变 | 参数不连续改变物理系统 | 只随机化 `env_ids` |
| 思维陷阱 | 只调摩擦，不调执行器延迟 | 仿真很稳，真机相位滞后 | sim2real gap 主要来自闭环延迟 | 接触、执行器、传感器一起建模 |

---

## S3.8 复现性：GPU 并行训练为什么很难“一模一样” ⭐⭐⭐

> **这一节解决什么问题**：为什么同一个配置跑两次结果不同，哪些差异可以接受，哪些差异必须定位？

### 复现性的三个层次 ⭐⭐

| 层次 | 目标 | 难度 | 工程意义 |
|------|------|------|----------|
| 配置复现 | 同一配置总体趋势相近 | 低 | 日常调参足够 |
| 统计复现 | 多个 seed 的均值和方差相近 | 中 | 论文和项目对比 |
| 位级复现 | 每一步数值完全一致 | 高 | 回归测试和数值定位 |

机器人 RL 通常追求统计复现。

位级复现很难。

原因包括：

- GPU 浮点归约顺序不固定。
- 原子操作顺序不固定。
- CUDA kernel 异步调度。
- 多线程环境 reset 顺序。
- 随机数生成器分布在多个库。
- 物理求解器迭代停止条件受微小误差影响。
- 神经网络库可能启用非确定性算法。

### 随机种子不是一个数 ⭐⭐⭐

一个训练程序可能有多套随机源：

| 随机源 | 用途 |
|--------|------|
| Python `random` | 配置采样、文件顺序 |
| NumPy | CPU 数据处理 |
| PyTorch CPU | 初始化、CPU tensor |
| PyTorch CUDA | GPU tensor、网络 dropout |
| JAX PRNGKey | JAX 随机化、MJX rollout |
| 仿真器内部 RNG | reset、扰动、传感器噪声 |
| 分布式 rank seed | 多 GPU 或多进程训练 |

只设置 `torch.manual_seed(0)` 不够。

### 复现配置示例 ⭐⭐

```python
import os
import random
import numpy as np
import torch


def set_reproducibility(seed: int, deterministic: bool = False):
    # Python 层随机数
    random.seed(seed)

    # NumPy 随机数
    np.random.seed(seed)

    # PyTorch CPU/GPU 随机数
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

    # 让哈希相关顺序更稳定
    os.environ["PYTHONHASHSEED"] = str(seed)

    if deterministic:
        # 确定性模式可能显著降低性能
        torch.use_deterministic_algorithms(True)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True

        # TF32 会改变矩阵乘法精度；是否关闭取决于实验需求
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
```

这段代码不能保证所有仿真器位级复现。

它只是把常见随机源收束起来。

对 JAX 项目，还要显式传递 PRNGKey。

对 Isaac Lab、Genesis、MJX、MuJoCo Warp 等框架，还要查各自官方文档中的 seed 和 determinism 设置。

### 统计复现报告 ⭐⭐⭐

单个 seed 的曲线不可靠。

报告训练结果时应至少给出：

- seed 数量。
- reward 均值。
- reward 标准差。
- 成功率均值。
- 成功率置信区间。
- 最差 seed 行为。
- 训练失败 seed 数。

表格示例：

| 配置 | seeds | 平均成功率 | 最差成功率 | 平均 SPS | 备注 |
|------|-------|------------|------------|----------|------|
| baseline | 5 | 0.82 | 0.61 | 1.8M | 有一个 seed 学到侧跳 |
| +delay randomization | 5 | 0.88 | 0.79 | 1.7M | 训练稍慢但更稳 |
| +wide friction | 5 | 0.74 | 0.30 | 1.7M | 随机化过宽 |

### 什么时候必须追求位级复现 ⭐⭐⭐

位级复现成本高。

但以下场景值得做：

1. 重构环境代码后 reward 曲线突变。
2. 更换物理后端后同一策略立即失败。
3. CUDA kernel 优化后出现偶发 NaN。
4. 单元测试验证 reward term。
5. 检查 reset 是否污染未终止环境。

### 练习 S3.8 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 为你的训练脚本列出所有随机源，并给出每个随机源的 seed 设置位置。 | ⭐ |
| 2 | 用 5 个 seed 跑一个短训练，画出均值和标准差曲线。比较“单 seed 最优曲线”和“多 seed 均值曲线”的差异。 | ⭐⭐ |
| 3 | 设计一个位级回归测试：固定初始状态、动作序列和随机化参数，验证 100 步后的观测误差是否低于阈值。 | ⭐⭐⭐ |

### 常见陷阱 S3.8 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念误区 | 认为 seed 相同就应完全一致 | GPU 上仍有微小差异 | 并行归约和非确定性 kernel | 区分统计复现和位级复现 |
| 编程陷阱 | JAX 随机数重复使用同一个 key | 所有环境随机化相同 | PRNGKey 未 split | 每步和每环境显式 split |
| 思维陷阱 | 只报告最好 seed | 迁移时失败率高 | 隐藏了训练方差 | 报告多 seed 统计和失败案例 |

---

## S3.9 性能剖析：先找到瓶颈，再谈优化 ⭐⭐⭐

> **这一节解决什么问题**：GPU 仿真慢时，如何判断瓶颈在物理、策略、奖励、拷贝、渲染还是日志？

### 不要只报一个 FPS ⭐⭐⭐

“FPS 很高”或“FPS 很低”都不够。

需要拆分指标：

| 指标 | 含义 |
|------|------|
| physics SPS | 物理积分步总数每秒 |
| policy SPS | 策略 transition 每秒 |
| real-time factor | 仿真物理时间 / 墙钟时间 |
| GPU utilization | GPU 计算单元利用率 |
| memory bandwidth | 显存带宽利用率 |
| GPU memory | 显存占用 |
| kernel launch count | 每步 kernel 数量 |
| CPU time | Python 和框架调度时间 |
| memcpy time | CPU/GPU 或 GPU/GPU 拷贝时间 |
| render time | 相机和 viewer 成本 |
| learner time | PPO/SAC 更新成本 |

### 正确计时：异步需要同步 ⭐⭐⭐

错误计时：

```python
import time

start = time.time()
env.step(action)
elapsed = time.time() - start
```

如果 `env.step` 只是发起 CUDA kernel，这个计时会低估真实耗时。

正确计时：

```python
import torch


def measure_cuda_step(env, action, warmup: int = 20, repeat: int = 200):
    # 预热：触发 JIT、缓存和内存分配
    for _ in range(warmup):
        env.step(action)
    torch.cuda.synchronize()

    start = torch.cuda.Event(enable_timing=True)
    end = torch.cuda.Event(enable_timing=True)

    start.record()
    for _ in range(repeat):
        env.step(action)
    end.record()
    torch.cuda.synchronize()

    ms = start.elapsed_time(end)
    return ms / repeat
```

JAX 项目也要在计时前后使用 `block_until_ready()`。

否则测到的是提交计算图的时间，而不是执行时间。

### 剖析顺序 ⭐⭐⭐

性能优化应按顺序进行：

1. 关闭渲染，测纯训练。
2. 固定策略输出为零，测纯物理和 reward。
3. 打开策略前向，测 inference 成本。
4. 打开 learner update，测训练成本。
5. 打开日志，测同步成本。
6. 打开评估录像，测渲染成本。
7. 分别扫描 `num_envs`、`horizon`、`decimation`。
8. 用 profiler 检查 kernel、memcpy、synchronize。

如果不分层测量，优化很容易走错方向。

例如看到 SPS 低就去改物理参数。

结果真正瓶颈是每步 `.item()` 日志同步。

### 常见瓶颈图谱 ⭐⭐⭐

| 症状 | 可能瓶颈 | 证据 | 处理 |
|------|----------|------|------|
| GPU 利用率低，CPU 占用高 | Python 调度 | profiler 显示大量小 kernel | 合并张量操作、CUDA Graph、JIT |
| GPU 利用率高，显存带宽高 | 内存访问 | kernel bandwidth 接近上限 | 减少状态字段、优化布局 |
| SPS 周期性掉到很低 | 日志/评估同步 | 掉速与 print 或 eval 对齐 | 降低频率、异步评估 |
| 首轮极慢，后面正常 | JIT 编译 | 只有第一次慢 | 预热，不计入 steady-state |
| 训练越跑越慢 | 显存泄漏或日志堆积 | 显存持续增长 | 检查保存 tensor 引用 |
| 开 viewer 后慢很多 | 渲染 | render kernel 和图像拷贝明显 | 训练关闭 viewer |

### 性能报告模板 ⭐⭐

```text
硬件：
  GPU: RTX 4090 24GB
  CPU: 14900K
  Driver/CUDA: 按实际填写

仿真：
  backend: 按实际填写
  num_envs: 4096
  sim_dt: 0.002
  decimation: 10
  policy_dt: 0.02
  rendering: off

训练：
  obs_dim: 48
  act_dim: 12
  horizon: 24
  minibatch_size: 24576
  learner: PPO

指标：
  physics SPS:
  policy SPS:
  GPU memory:
  average step time:
  learner update time:
  logging interval:
```

没有这些信息，单独比较“训练 15 分钟”或“SPS 多少”没有意义。

### 练习 S3.9 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 为训练脚本增加分段计时：policy、env.step、reward、reset、learner update、logging。 | ⭐ |
| 2 | 用 `num_envs=512/1024/2048/4096/8192` 扫描 SPS 和显存，画出吞吐曲线，找出饱和点。 | ⭐⭐ |
| 3 | 人为加入每步 `.item()`，用 profiler 证明同步点导致吞吐下降，再移除并复测。 | ⭐⭐⭐ |

### 常见陷阱 S3.9 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程陷阱 | 用 `time.time()` 测 CUDA 异步代码 | 时间看起来过小 | kernel 尚未执行完 | 用 CUDA Event 或 synchronize |
| 概念误区 | 把 JIT 编译时间算进 steady-state SPS | 首轮结果异常差 | 编译不是稳定运行成本 | 区分 compile、warmup、steady-state |
| 思维陷阱 | 看见 GPU 利用率低就增加环境数 | 显存爆掉，训练不稳 | 瓶颈可能在 CPU 同步 | 先用 profiler 定位 |

---

## S3.10 多仿真器验证：速度之外的物理可信度 ⭐⭐⭐

> **这一节解决什么问题**：为什么高吞吐训练完成后，还要在另一个仿真器或 CPU 路径中复查？

### 单一仿真器的风险 ⭐⭐⭐

一个策略可能在训练仿真器里表现很好。

原因可能是它学会了任务。

也可能是它利用了训练仿真器的偏差。

典型例子包括：

- 利用接触穿透获得额外推进。
- 利用过低摩擦惩罚产生滑步。
- 利用关节限位软约束弹开。
- 利用传感器无延迟做高频反馈。
- 利用 reset 边界条件刷 reward。

多仿真器验证的目的不是证明另一个仿真器完全正确。

它是寻找策略对物理实现细节的依赖。

### sim2sim 验证流程 ⭐⭐⭐

| 步骤 | 阶段 | 目的 | 关键检查 |
|------|------|------|----------|
| 1 | 训练后端产生策略参数 | 完成训练 | 收敛曲线、reward 分项 |
| 2 | 同后端评估 | 排除训练噪声 | 固定种子、关闭随机化 |
| 3 | CPU reference replay | 在同一物理家族内获得可解释的单步日志 | obs/action 对齐、接触力记录 |
| 4 | 跨引擎 sim2sim | 检查接触、执行器、积分和资产表达差异 | 关节顺序、摩擦模型、PD 增益 |
| 5 | 参数扫描 | 评估策略对物理参数的敏感度 | 摩擦、质量、延迟、地形 |
| 6 | 部署前 dry-run | 验证部署接口 | 观测归一化、动作限幅、频率、安全保护 |

### 要对齐的接口 ⭐⭐⭐

sim2sim 失败常常不是物理问题，而是接口没对齐：

| 接口 | 常见不一致 |
|------|------------|
| 坐标系 | base frame、world frame、IMU frame 不一致 |
| 关节顺序 | URDF/MJCF/USD 中 DOF 顺序不同 |
| 动作定义 | 位置目标、速度目标、力矩残差混淆 |
| 归一化 | obs mean/std 来自训练但评估没加载 |
| 频率 | policy_dt 和 decimation 不一致 |
| 延迟 | 训练有 action delay，评估没有 |
| termination | 评估 early stop 规则不同 |
| default pose | 零姿态和默认站姿定义不同 |

### 评估脚本最小检查 ⭐⭐

```python
def check_policy_io(policy, obs, action_limit):
    # 检查观测是否有限
    assert torch.isfinite(obs).all(), "观测中出现 NaN 或 Inf"

    # 策略前向
    with torch.no_grad():
        action = policy(obs)

    # 检查动作形状
    assert action.ndim == 2, "动作应为 [num_envs, act_dim]"

    # 检查动作是否有限
    assert torch.isfinite(action).all(), "动作中出现 NaN 或 Inf"

    # 检查动作限幅
    max_abs = torch.max(torch.abs(action)).item()
    if max_abs > action_limit:
        raise RuntimeError(f"动作超过限幅: {max_abs:.3f} > {action_limit:.3f}")

    return action
```

这类检查看起来基础。

但很多 sim2sim 崩溃来自动作顺序或归一化错误。

先检查接口，再怀疑物理。

### 练习 S3.10 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 选择一个训练策略，列出从训练后端迁移到 MuJoCo CPU 评估时要对齐的 10 个接口。 | ⭐⭐ |
| 2 | 设计一个 sim2sim dashboard：同一策略在两个后端上记录 base height、foot contact、joint torque、action、reward 分项。 | ⭐⭐⭐ |
| 3 | 如果策略在训练后端稳定但在 CPU MuJoCo 中脚底打滑，按顺序给出排查计划。 | ⭐⭐⭐ |

### 常见陷阱 S3.10 ⭐

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念误区 | sim2sim 失败就断定训练后端错 | 来回换后端仍失败 | 可能是接口或动作定义错 | 先做 I/O 对齐检查 |
| 编程陷阱 | 用名称匹配关节但忽略顺序 | 动作施加到错误关节 | 模型文件排序不同 | 建立显式 joint name 到 index 映射 |
| 思维陷阱 | 只在平地评估 | 真机遇到扰动就失败 | 评估分布太窄 | 加入摩擦、坡度、push 和延迟扫描 |

---

## S3.11 累积项目：Mini-GPU-RL 训练与诊断流水线 ⭐⭐⭐

> **项目目标**：构建一个不绑定具体物理后端的 GPU RL 训练诊断框架，能记录吞吐、随机化、复现配置和 sim2sim 评估结果。

### 模块划分 ⭐⭐

| 模块 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `VectorEnvAdapter` | 后端环境 | `obs, rew, done, info` | 统一批量接口 |
| `TimeScaleConfig` | `sim_dt, decimation` | `policy_dt` | 检查频率一致性 |
| `RandomizationConfig` | 分布参数 | 每环境物理参数 | 记录随机化范围 |
| `GpuRolloutBuffer` | GPU transition | PPO batch | 避免不必要拷贝 |
| `PerfMeter` | CUDA/JAX event | 分段耗时 | 定位瓶颈 |
| `ReproRecord` | seed 和版本 | YAML/JSON 记录 | 支持复现实验 |
| `Sim2SimEvaluator` | policy checkpoint | 对齐评估日志 | 验证物理依赖 |

### 配置示例 ⭐⭐

```yaml
experiment:
  name: go2_gpu_locomotion_baseline
  seed: 7

simulation:
  backend: fill_with_project_backend
  num_envs: 4096
  sim_dt: 0.002
  decimation: 10
  policy_dt: 0.02
  rendering: false

randomization:
  friction: [0.4, 1.2]
  mass_scale: [0.9, 1.1]
  action_delay_steps: [0, 2]
  imu_noise_std: 0.02

profiling:
  warmup_steps: 100
  measure_steps: 1000
  log_interval: 100

evaluation:
  sim2sim_backends:
    - mujoco_cpu
  episodes_per_seed: 20
  record_video: true
```

### 频率检查代码 ⭐⭐

```python
from dataclasses import dataclass


@dataclass
class TimeScaleConfig:
    sim_dt: float
    decimation: int
    expected_policy_dt: float

    def validate(self):
        # 计算策略周期
        policy_dt = self.sim_dt * self.decimation

        # 用绝对误差判断，避免浮点表示造成误报
        err = abs(policy_dt - self.expected_policy_dt)
        if err > 1e-9:
            raise ValueError(
                f"频率配置不一致: sim_dt*decimation={policy_dt}, "
                f"expected={self.expected_policy_dt}"
            )

        # 返回策略频率，后续写入实验记录
        return 1.0 / policy_dt
```

### 项目验收标准 ⭐⭐

| 项 | 标准 |
|----|------|
| 批量接口 | 所有核心张量第一维为 `num_envs` |
| 拷贝控制 | 训练步内不出现高频 CPU 同步 |
| 随机化记录 | 每个随机化参数有分布和范围 |
| 频率记录 | 明确 `sim_dt/decimation/policy_dt` |
| 性能剖析 | 至少分出 physics、policy、reward、learner、logging |
| 复现记录 | 保存 seed、后端版本、模型 hash、训练配置 |
| sim2sim | 至少一个独立评估后端或 CPU 路径 |

### 项目练习 ⭐

| # | 练习 | 难度 |
|---|------|------|
| 1 | 实现 `TimeScaleConfig.validate()` 的单元测试，覆盖正常配置和错误配置。 | ⭐ |
| 2 | 给 `PerfMeter` 增加 CUDA Event 计时，并输出每 1000 step 的平均耗时。 | ⭐⭐ |
| 3 | 把一次训练的随机化配置、seed 和性能指标保存成 JSON，并写一个脚本比较两次实验差异。 | ⭐⭐⭐ |

---

## S3.12 本章小结 ⭐⭐

### 核心结论表 ⭐

| 知识点 | 一句话总结 | 工程检查 |
|--------|------------|----------|
| GPU 并行仿真 | 加速来自横向批量环境，不是跳过物理 | `num_envs`、batch shape、SPS |
| 环境批量化 | `num_envs` 是数据结构中心 | obs/action/reward/done 全链路批量 |
| SIMT | GPU 喜欢同形状、少分支、固定内存 | 避免动态拓扑和逐环境 Python |
| 生态定位 | 框架按物理内核、环境封装、训练算法分层理解 | 先列需求，再选工具 |
| 控制频率 | `policy_dt = sim_dt × decimation` | 训练频率贴近部署接口 |
| 数据拷贝 | 高频同步会摧毁吞吐 | 限制 `.item()`、`.cpu()`、渲染 |
| 随机化 | 训练分布覆盖真实不确定性 | 分布合理、物理一致、记录完整 |
| 复现性 | 多 seed 统计比单 seed 曲线可靠 | 记录所有随机源 |
| 性能剖析 | 没有分段测量就不要优化 | physics/policy/reward/learner/logging 分开 |
| sim2sim | 速度之外要验证物理依赖 | 接口对齐、参数扫描、CPU 复查 |

### 三个最重要的判断 ⭐⭐

1. GPU 仿真项目首先要问：“我的数据是否真的一直留在设备上？”
2. 生态选型首先要问：“我的任务更需要物理梯度、前向吞吐、传感器场景、还是长期工程维护？”
3. 迁移真机之前首先要问：“策略是否只在一个仿真器和一个参数点上成功？”

### 本质洞察回顾 ⭐⭐

> **本质洞察**：GPU 并行仿真的本质不是让一个世界变快，而是让许多世界共享同一张计算图。

> **本质洞察**：域随机化不是追求更逼真的单点模型，而是构造覆盖真实不确定性的训练分布。

> **本质洞察**：多仿真器验证不是寻找唯一正确的仿真器，而是暴露策略对某个物理实现细节的依赖。

---

## 延伸阅读与官方资料入口 ⭐

| 资料 | 用途 | 难度 |
|------|------|------|
| [MuJoCo MJX 文档](https://mujoco.readthedocs.io/en/latest/mjx.html) | 理解 `mjx.put_model`、`mjx.Data` batch 维度、MJX-JAX 与 MJX-Warp 差异 | ⭐⭐⭐ |
| [MuJoCo Warp 官方仓库](https://github.com/google-deepmind/mujoco_warp) | 了解 NVIDIA GPU 优化 MuJoCo 路径 | ⭐⭐⭐ |
| [Isaac Lab 文档](https://isaac-sim.github.io/IsaacLab/main/index.html) | 学习 Manager-based / Direct workflow、传感器和环境列表 | ⭐⭐ |
| [NVIDIA Isaac Lab 技术博客](https://developer.nvidia.com/blog/fast-track-robot-learning-in-simulation-using-nvidia-isaac-lab) | 理解 Isaac Gym 到 Isaac Lab 的迁移动机 | ⭐⭐ |
| [Brax 官方仓库](https://github.com/google/brax) | 理解 Brax training 与 MJX/MuJoCo Warp 的关系 | ⭐⭐ |
| [Genesis 文档](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/what_is_genesis.html) | 理解 Genesis 的通用物理平台定位 | ⭐⭐ |
| [Genesis Parallel Simulation](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/parallel_simulation.html) | 学习 `n_envs` 批量环境 API | ⭐ |
| [Genesis Differentiable Simulation](https://genesis-world.readthedocs.io/en/latest/api_reference/differentiation/index.html) | 了解 Genesis 的可微分仿真接口 | ⭐⭐⭐ |

---

## 🔧 故障排查手册 ⭐⭐

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|----------|----------|----------|
| GPU 利用率很低，SPS 远低于预期 | Python 逐环境循环、batch 太小、频繁同步 | 1. 搜索 `.item/.cpu/.numpy` 2. 用 profiler 看 CPU 时间 3. 增大 batch 或合并张量操作 | S3.2/S3.6/S3.9 |
| 首次运行几分钟没有输出 | JIT 编译或 CUDA Graph 捕获 | 1. 区分首次编译和稳定运行 2. 加 warmup 3. 固定 shape 避免反复编译 | S3.3/S3.9 |
| 显存突然爆掉 | `num_envs`、接触 buffer、图像传感器或 rollout buffer 太大 | 1. 降 `num_envs` 2. 关闭渲染 3. 检查保存 tensor 引用 4. 分析 buffer shape | S3.2/S3.6 |
| reward 曲线很好但 sim2sim 崩溃 | 策略利用仿真器偏差或接口未对齐 | 1. 检查关节顺序 2. 检查动作定义 3. 扫描摩擦/延迟 4. 用 CPU 路径复查接触 | S3.10 |
| 同一 seed 结果仍有差异 | GPU 非确定性、随机源未全部设置、异步归约 | 1. 列出随机源 2. 打开确定性选项 3. 做多 seed 统计 4. 必要时做位级测试 | S3.8 |
| 改 `decimation` 后策略行为变差 | reward 尺度和动作周期变化 | 1. 重新计算 `policy_dt` 2. reward 乘以时间步长 3. 检查动作变化惩罚 | S3.5 |
| 开 viewer 后训练很慢 | 渲染与训练争抢 GPU，图像拷贝触发同步 | 1. 训练时关闭 viewer 2. 独立评估录像 3. 降低相机数量和分辨率 | S3.6/S3.9 |
| 随机化后学不会站立 | 随机化范围过宽或不物理一致 | 1. 缩小范围 2. 加 curriculum 3. 检查质量/惯量一致性 4. 单独关闭某类随机化定位 | S3.7 |
| 策略在真机上高频抖动 | 策略频率、延迟或动作滤波与训练不一致 | 1. 对齐 `policy_dt` 2. 训练加入 action delay 3. 检查动作频谱 4. 限制动作变化率 | S3.5/S3.7 |

---

## 综合练习 ⭐

### A 型：批量环境与吞吐测量 ⭐⭐

1. 写一个最小批量环境，观测形状为 `[4096, 48]`，动作形状为 `[4096, 12]`。
2. 奖励包含速度跟踪、动作惩罚和 alive bonus。
3. 不允许使用 Python 逐环境循环。
4. 使用 CUDA Event 测量 1000 次 step 的平均耗时。
5. 报告是否出现 CPU 同步点。

### A 型：频率与 reward 尺度 ⭐⭐

1. 选取 `sim_dt=0.002`。
2. 分别设置 `decimation=5/10/20`。
3. 让 reward 中所有连续时间项乘以 `policy_dt`。
4. 比较三组训练中的动作平滑性和奖励尺度。
5. 解释哪组更接近目标真机控制接口。

### B 型：生态选型报告 ⭐⭐⭐

为以下三个任务分别选择仿真生态，并写出理由：

| 任务 | 约束 |
|------|------|
| Go2 本体感知 locomotion | 需要高吞吐 PPO、未来真机部署 |
| Panda 接触丰富操作 | 需要相机、接触调试和资产管理 |
| 参数辨识 + 可微分 MPC 原型 | 需要物理梯度和 MuJoCo 模型 |

报告必须包含：

- 候选框架。
- 选择理由。
- 最大风险。
- 验证计划。
- 放弃其他框架的原因。

### B 型：sim2sim 诊断 ⭐⭐⭐

给定一个在训练后端中成功的策略，在另一个后端中出现脚底打滑。

按顺序完成：

1. 检查关节顺序。
2. 检查动作尺度。
3. 检查观测归一化。
4. 检查摩擦参数。
5. 检查 PD 增益。
6. 检查延迟。
7. 检查接触力日志。
8. 给出最可能的三个根因。

### 思考题 ⭐⭐

1. 为什么“GPU 仿真更快”可能让工程师更容易忽略 reward 设计错误？
2. 如果一个策略只在 MuJoCo Warp 中成功，但在 MuJoCo CPU 中失败，应优先怀疑物理后端还是接口对齐？为什么？
3. 如果一个项目同时需要视觉渲染和高吞吐 locomotion，应把渲染放在训练主循环里，还是拆成独立数据生成/评估流程？说明理由。
4. 为什么 Brax 的训练库价值和 Brax 原始物理管线价值需要分开评价？
5. 随机化范围应该由真实测量决定，还是由训练稳定性决定？如何在两者之间折中？

### ⚠️ 章末陷阱：吞吐量不是唯一目标 ⭐

GPU 后端最容易诱导的错误，是把每秒步数当成唯一指标。每秒步数很高但观测接口、动作尺度、奖励时间尺度和部署控制频率不一致，训练出来的策略仍然无法迁移。

### ⚠️ 章末陷阱：把批量化误解成复制环境 ⭐

批量环境必须让 reset、随机化、终止、统计和日志都带有 batch 语义。只把单环境对象复制很多份，再用 Python 循环调用，并没有真正利用 GPU 并行。

### ⚠️ 章末陷阱：忽略首次编译和稳定运行的区别 ⭐

JIT 编译、图捕获和 kernel warmup 会让首次运行时间很长。性能报告应区分 cold start、warmup 和 steady state，否则容易把编译时间误算成物理步进速度。
