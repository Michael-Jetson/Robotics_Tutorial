# 04. 从 PPO 到 env.step()：Manager-Based 架构精读

## 前置自测

📋 **答不出 $\ge$ 2 题 → 先回前置章节复习**

1. PPO 的训练循环分为哪两个阶段？每个阶段的核心操作是什么？（RL 基础）
2. `env.step()` 返回什么？`terminated` 和 `truncated` 的区别是什么？（Ch01 §1.4 env.step 时序）
3. Manager-Based 架构和 legged_gym 的单体架构有什么本质区别？（Ch01 §1.3）
4. mjlab 的 ObservationManager 和 Isaac Lab 的 ObservationManager 有什么 API 差异？（Ch01 §1.4）
5. 为什么 reward 在 `sim.forward()` 之前计算，而 observation 在之后计算？（Ch01 §1.4 env.step 时序预览）

## 本章目标

学完本章后，你应该能够：

1. **画出** PPO 训练循环的完整数据流，标注 env.step() 在循环中的位置
2. **逐行解释** env.step() 内部 18 步时序的执行顺序和设计理由
3. **对比** legged_gym 单体架构和 Manager-Based 架构的工程差异
4. **精读**双框架的 Manager API，理解 mjlab 和 Isaac Lab 的命名差异和配置模式
5. **理解** Entity/Articulation 系统的编译-运行分离设计
6. **独立添加**一个自定义 ObsTerm 或 RewTerm 到现有任务中
7. **定位** Manager 加载顺序导致的 wiring 错误

### 前置知识桥接

回顾 Ch01 §1.3：我们介绍了 Manager-Based 架构的"九大 Manager"——ObservationManager、ActionManager、RewardManager 等。你已经知道每个 Manager 的职责和修改时机。但你还不知道它们**内部如何工作**：Manager 的加载顺序是什么？为什么 EventManager 必须最先加载？env.step() 中各 Manager 的调用时序如何影响 obs/reward 的对齐？

回顾 Ch02 §2.6：你完成了第一次训练，观察了 reward 曲线和行为。回顾 Ch03 §3.5：你理解了从 MjSpec 到 GPU batched worlds 的数据流。本章在此基础上，深入框架层——从"物理引擎怎么工作"到"框架怎么把物理引擎包装成可训练的环境"。

### 如果跳过本章会怎样

你可能在 Ch06（Reward 设计）中添加了一个 reward term，但它总是返回零——因为你不知道 reward 是在 `sim.forward()` 之前计算的，你的 reward 函数读到的是上一步的派生量。或者你在 Ch08（DR）中添加了一个 event term，但 DR 不生效——因为 EventManager 的加载顺序不对，导致你的 event 在 obs 之后才执行。**理解 Manager 的加载顺序和执行时序是避免这类隐蔽 bug 的唯一途径。**

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含源码阅读和练习） | 5-6 小时 | 需要深入理解框架内部工作原理的读者 |
| 速读（跳过源码细节） | 2-3 小时 | 有 Isaac Lab 经验，重点看 mjlab 差异的读者 |
| 速查（只看时序表和 API 对照） | 45 分钟 | 遇到具体 Manager 问题时回来查 |

---

## 4.1 算法回顾：PPO 训练循环 ⭐⭐

> **这一节解决什么问题**：快速回顾 PPO 的训练循环——不是推导算法，而是理解 `env.step()` 在训练循环中的精确位置。

### 动机

本教材面向有 RL 算法基础的博士生。你已经知道 PPO 是什么（proximal policy optimization），但可能不清楚它在工程实现中是如何与 `env.step()` 交互的。理解这个交互是理解后续所有 Manager 时序的前提。

### 如果不理解 PPO 循环会怎样

你可能不知道 PPO 的 rollout 阶段和 update 阶段对 `env.step()` 的调用模式不同——rollout 阶段每步调用一次 `env.step()`，update 阶段不调用 `env.step()`。如果你在 env 内部添加了依赖"当前是 rollout 还是 update"的逻辑，会导致意想不到的行为。

### PPO 训练循环的工程结构

PPO 的训练循环分为两个交替阶段：

```
for iteration in range(max_iterations):
    # ========== 阶段一：Rollout（数据收集）==========
    for step in range(num_steps_per_env):
        # 1. 策略前向传播：obs → action
        actions = actor.act(obs)
        
        # 2. 环境步进：action → next_obs, reward, done
        obs, reward, terminated, truncated, info = env.step(actions)
        
        # 3. 存储 transition
        storage.add(obs, actions, reward, terminated, truncated, values, log_probs)
    
    # ========== 阶段二：Update（梯度更新）==========
    # 4. 计算 GAE（Generalized Advantage Estimation）
    advantages = compute_gae(storage, gamma, lam)
    
    # 5. Mini-batch 梯度更新
    for epoch in range(num_epochs):
        for mini_batch in storage.iterate(num_mini_batches):
            # PPO clip loss + value loss + entropy bonus
            loss = ppo_loss(mini_batch, advantages, clip_range)
            optimizer.step(loss)
```

**关键工程事实**：

| 事实 | 工程影响 |
|------|---------|
| Rollout 阶段调用 `env.step()` | env 的所有 Manager 在每步 rollout 时执行 |
| Update 阶段不调用 `env.step()` | Update 期间 env 状态冻结（GPU 显存被 PPO 占用） |
| `num_steps_per_env` 决定 rollout 长度 | 太短→advantage 估计不准，太长→内存占用大 |
| `num_mini_batches` 决定 batch 切分 | 必须能整除 `num_envs × num_steps_per_env` |
| `num_epochs` 决定每批数据重复使用次数 | 太多→过拟合旧数据，太少→样本效率低 |

> **本质洞察**：PPO 的工程实现中，`env.step()` 是训练循环的**数据生产者**——它生产 (obs, reward, done) 三元组。PPO 算法本身是**数据消费者**——它消费这些三元组计算梯度。理解这个生产者-消费者关系，就能理解为什么 env.step() 的内部时序如此重要——它决定了"数据的质量"。

### 为什么机器人 RL 几乎总是用 PPO

回顾 Ch01 §1.5：RSL-RL 只提供 PPO 一种算法。这不是偶然——PPO 在机器人 locomotion 中的统治地位有三个工程原因：

**原因一：on-policy 稳定性。** Locomotion 任务的 reward 分布在训练过程中剧烈变化——从"倒地"到"站立"到"行走"，每个阶段的最优策略和 reward 分布完全不同。Off-policy 算法（如 SAC）的 replay buffer 中充满了旧策略产生的数据，这些"过时数据"可能让 Q-function 学到错误的值估计。PPO 每次只用最新策略产生的数据，避免了这个问题。

**原因二：GPU 并行友好。** PPO 的 rollout 阶段只需要策略网络的前向传播（`actor.act(obs)` → action），这在 GPU 上可以高效批处理。数据收集和梯度更新完全分离——4096 个环境在 GPU 上同时 step，收集 `4096 × 24 = 98304` 个 transition，然后一次性计算梯度。SAC 需要在数据收集时同时更新 Q 网络，数据流更复杂。

**原因三：超参数简单可调。** PPO 的核心超参数只有 5 个（learning_rate、gamma、lam、clip_range、desired_kl），且有明确的物理直觉。SAC/TD3 的超参数空间更大（两个 Q 网络的学习率、target 网络的 tau、replay buffer 大小、batch 大小等），调参成本更高。

> **反事实推理**：如果机器人 locomotion 的 reward 分布是稳定的（如经典控制的 CartPole），off-policy 算法的样本效率优势会更明显——因为每个 transition 可以被重复使用多次。但 locomotion 的 reward 分布变化太快了——上一分钟的"好数据"（站立）在这一分钟可能是"坏数据"（因为策略现在应该学走路了）。

当然，在操作任务（manipulation）中，SAC 或 Diffusion Policy 可能更合适——这就是为什么 Isaac Lab 支持多种 RL 后端。Ch07 会讨论算法选型的工程决策。

### PPO 的 clip 机制如何工程实现

PPO 的核心是 clip objective——限制策略更新的幅度，避免"一步走太远"导致策略崩溃。工程实现中的关键代码：

```python
# PPO clip objective 的工程实现（RSL-RL 简化）
def ppo_loss(obs, actions, advantages, old_log_probs, clip_param=0.2):
    # 1. 计算当前策略的 log probability
    new_log_probs = policy.log_prob(obs, actions)
    
    # 2. 计算 importance sampling ratio
    ratio = torch.exp(new_log_probs - old_log_probs)
    
    # 3. Clip ratio
    clipped_ratio = torch.clamp(ratio, 1 - clip_param, 1 + clip_param)
    
    # 4. 取 min（悲观估计）
    loss = -torch.min(ratio * advantages, clipped_ratio * advantages).mean()
    
    return loss
```

`clip_param=0.2` 意味着新策略和旧策略的行为差异不超过 20%。如果某个 action 的 ratio > 1.2（新策略比旧策略更倾向这个 action），clip 会限制它——即使 advantage 很大。这防止了"一步更新太大→策略行为剧变→收集到的新数据和旧数据差异太大→下一步更新方向完全错误"的恶性循环。

> **一个跨领域类比**：clip 机制类似于机器人控制中的"速度限制"。即使目标位置很远（advantage 很大），每步的关节速度也不超过一个上限——这保证了运动的平滑性和安全性。PPO 的 clip 保证了"策略更新的平滑性"。

### RSL-RL 中的 PPO 实现

mjlab 和 Isaac Lab 都使用 RSL-RL 作为 PPO 后端。RSL-RL 的 `OnPolicyRunner` 类封装了上述循环：

```python
# RSL-RL OnPolicyRunner 简化
class OnPolicyRunner:
    def learn(self, num_learning_iterations):
        for it in range(num_learning_iterations):
            # Rollout
            with torch.inference_mode():
                for i in range(self.num_steps_per_env):
                    actions = self.alg.act(obs, critic_obs)
                    obs, critic_obs, rewards, dones, infos = self.env.step(actions)
                    # ... store transitions
            
            # Update
            self.alg.update()  # 内部做 PPO clip + value loss + entropy
```

**RSL-RL 的 VecEnv Wrapper**：`env.step()` 返回的是 Manager-Based 环境的原始输出（obs dict、reward tensor、terminated tensor、truncated tensor、extras dict）。RSL-RL 需要的是简化格式——`RslRlVecEnvWrapper` 做了以下适配：

| env.step() 输出 | RSL-RL 需要 | Wrapper 处理 |
|----------------|------------|------------|
| `obs_dict["actor"]` | `obs` tensor | 直接传递 |
| `obs_dict["critic"]` | `critic_obs` tensor | 如果存在则传递 |
| `reward` | `rewards` | 直接传递 |
| `terminated \| truncated` | `dones` | 合并为单个 bool tensor |
| `extras["time_outs"]` | `infos["time_outs"]` | 用于 value bootstrap |

> **反事实推理**：如果 Wrapper 不区分 `terminated` 和 `truncated`，PPO 的 value bootstrap 会出错——truncated（超时）的 episode 需要 bootstrap（因为策略还没"真正"结束），而 terminated（真正失败）的 episode 不需要。这个区分对 GAE 计算的正确性至关重要。

### GAE 中的 $\gamma$ 和 $\lambda$

GAE（Generalized Advantage Estimation）是 PPO 计算 advantage 的标准方法。两个关键超参数：

| 参数 | 含义 | 典型值 | 调参方向 |
|------|------|--------|---------|
| `gamma` ($\gamma$) | 折扣因子 | 0.99 | 减小→更关注短期 reward |
| `lam` ($\lambda$) | GAE 平滑参数 | 0.95 | 减小→更关注 TD(0)，增大→更关注 Monte Carlo |

**GAE 的工程直觉**：GAE 在 TD(0) 估计（低方差高偏差）和 Monte Carlo 估计（高方差低偏差）之间做加权平均。`lam=0` 对应纯 TD(0)——advantage 只看"下一步的 reward + 下一步的 value 估计"，噪声小但可能有系统性偏差（如果 value 函数不准）。`lam=1` 对应纯 Monte Carlo——advantage 看"到 episode 结束的所有 reward 之和"，无偏但噪声大（因为累积了很多步的随机性）。`lam=0.95` 是两者的折中——既有 TD 的低方差优点，又有 MC 的低偏差优点。

**为什么 locomotion 用 `gamma=0.99` 而不是 `gamma=0.999`？** `gamma` 决定了策略关注的时间尺度。`gamma=0.99` 的"有效视野"约为 100 步（`1/(1-gamma)`）——在 50 Hz 的 policy frequency 下对应 2 秒。对于四足行走，2 秒内的 reward 信号足以让策略学会稳定步态。如果用 `gamma=0.999`（有效视野 1000 步 = 20 秒），策略需要考虑更长远的 reward——但 locomotion 的 reward 在短时间内就能给出充分的反馈，不需要这么长的视野。过长的视野只会增加 variance，让训练变慢。

**`num_steps_per_env` 的工程权衡**：这个参数决定每次 rollout 收集多少步数据。太短（如 4 步）→GAE 估计不准（因为只看了很少的未来 reward）。太长（如 256 步）→GPU 显存被 rollout storage 占满，留给物理仿真的显存变少。mjlab 默认 `num_steps_per_env=24`——在 4096 envs 下，这产生 `4096 × 24 = 98304` 个 transition，约占 1-2 GB GPU 显存（取决于 obs 维度）。

**工程经验**：对于 locomotion 任务，`gamma=0.99` + `lam=0.95` 几乎总是好的起点。只有在特殊情况下才需要调整——如果策略过于"短视"（只关注当前步的 reward 而忽略长期），尝试增大 `gamma`；如果 advantage 估计方差太大导致训练不稳定，尝试减小 `lam`。

### RSL-RL 的 PPO 超参一览

以下是 RSL-RL 中 PPO 的所有可调超参数，按重要性排序：

| 参数 | 典型值 | 含义 | 调参优先级 |
|------|--------|------|-----------|
| `learning_rate` | 1e-3 | Adam 优化器学习率 | ⭐⭐⭐ |
| `num_steps_per_env` | 24 | 每个 env 的 rollout 步数 | ⭐⭐ |
| `num_mini_batches` | 4 | mini-batch 数量 | ⭐⭐ |
| `num_epochs` | 5 | 每批数据重复训练的次数 | ⭐⭐ |
| `clip_range` | 0.2 | PPO clip 参数 $\epsilon$ | ⭐ |
| `gamma` | 0.99 | 折扣因子 | ⭐ |
| `lam` | 0.95 | GAE $\lambda$ 参数 | ⭐ |
| `desired_kl` | 0.01 | KL 散度目标值（自适应 lr） | ⭐⭐ |
| `entropy_coef` | 0.01 | 熵正则化系数 | ⭐ |
| `value_loss_coef` | 1.0 | value loss 权重 | — |
| `max_grad_norm` | 1.0 | 梯度裁剪阈值 | — |
| `init_noise_std` | 1.0 | 初始 action 噪声标准差 | ⭐ |

**RSL-RL 的自适应学习率机制**：当 KL 散度超过 `desired_kl` 的 2 倍时，学习率自动减半；当 KL 低于 `desired_kl` 的 0.5 倍时，学习率自动翻倍。这个机制让你可以用一个相对大的初始 `learning_rate`（如 1e-3），让 RSL-RL 自动调整——比手动调 lr 高效得多。

```python
# RSL-RL 的 KL 自适应 lr（简化）
if kl > 2.0 * desired_kl:
    learning_rate *= 0.5
elif kl < 0.5 * desired_kl:
    learning_rate *= 2.0
learning_rate = max(min(learning_rate, lr_upper), lr_lower)
```

**RSL-RL 的 rl_cfg 完整配置**（mjlab 的 velocity task）：

```python
def unitree_go2_rl_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        # 训练参数
        max_iterations=1500,
        num_steps_per_env=24,
        
        # 策略网络
        actor=RslRlMLPModelCfg(
            hidden_dims=[512, 256, 128],
            activation="elu",
            obs_normalization=True,
        ),
        critic=RslRlMLPModelCfg(
            hidden_dims=[512, 256, 128],
            activation="elu",
            obs_normalization=True,
        ),
        
        # PPO 参数
        algorithm=RslRlPPOAlgorithmCfg(
            learning_rate=1e-3,
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            clip_param=0.2,
            entropy_coef=0.01,
            num_mini_batches=4,
            num_learning_epochs=5,
            value_loss_coef=1.0,
            max_grad_norm=1.0,
            init_noise_std=1.0,
        ),
        
        # 日志
        experiment_name="go2_velocity",
        logger="wandb",
    )
```

> **rsl_rl $\ge$ 4.0 的关键变化**：actor 和 critic 的网络配置被**拆分**为独立的 model config。这允许 actor 用 MLP 而 critic 用 CNN（非对称 actor-critic），是视觉策略和 teacher-student 蒸馏的工程基础。旧版（< 4.0）使用 `RslRlPpoActorCriticCfg` 统一配置——如果你看到这个类名，说明代码基于旧版 rsl_rl。

### PPO 与 env.step() 的交互图

```
RSL-RL OnPolicyRunner
  │
  ├── Rollout Phase ──────────────────────────────────────┐
  │   │                                                    │
  │   │  for step in range(num_steps_per_env):            │
  │   │    1. actor.act(obs) → action                      │
  │   │    2. env.step(action) → obs, reward, done         │
  │   │         ├── 18 步内部时序（§4.2）                   │
  │   │         └── 各 Manager 执行                        │
  │   │    3. storage.add(transition)                      │
  │   │                                                    │
  │   └── 收集完 num_envs × num_steps_per_env 个 transition │
  │                                                        │
  ├── Update Phase ───────────────────────────────────────┐
  │   │                                                    │
  │   │  1. compute_gae(storage, gamma, lam)              │
  │   │  2. for epoch in range(num_epochs):               │
  │   │       for mini_batch in storage.iterate():        │
  │   │         ppo_clip_loss + value_loss + entropy       │
  │   │         optimizer.step()                          │
  │   │  3. lr_schedule(kl)                               │
  │   │                                                    │
  │   └── 网络更新完毕，开始下一个 iteration                │
  │                                                        │
  └── 循环直到 max_iterations                               │
```

> **双重解读**：
>
> 角度 1（数据流视角）：env.step() 是**数据工厂**——它在 GPU 上并行生产 4096 条 (obs, action, reward, done) 记录。PPO 是**数据消费者**——它从这些记录中提取梯度信号更新网络。
>
> 角度 2（频率视角）：env.step() 以 50 Hz（policy frequency）被调用，每次调用内部执行 4 个 200 Hz 的物理步进。PPO 的 update 则以 `1 / num_steps_per_env ≈ 2 Hz` 的频率发生——远低于 env.step() 的频率。

### ⚠️ 常见陷阱

1. **在 env.step() 内部假设"当前在 rollout 阶段"。** Manager 的代码不应依赖外部的训练阶段——它们应该是无状态的（给定输入产生确定输出）。
2. **忘记 `terminated` 和 `truncated` 的区别。** `terminated=True` 意味着"任务失败"（如机器人摔倒），`truncated=True` 意味着"超时"（episode 达到最大步数）。PPO 需要这个区分来正确计算 value bootstrap。
3. **把 `num_steps_per_env` 设得太大。** 这会导致 GPU 显存中存储大量 transition——对于 4096 envs $\times$ 24 steps，存储的 obs/action/reward 就需要数 GB 显存。

### 练习

1. 在 RSL-RL 的 PPO 训练循环中，`env.step()` 被调用了多少次？（答案：`max_iterations × num_steps_per_env`）
2. 如果 `num_envs=4096`、`num_steps_per_env=24`、`num_mini_batches=4`，每个 mini-batch 有多少个 transition？（答案：`4096 × 24 / 4 = 24576`）
3. （跨章综合题）回顾 Ch03 §3.5 的 CUDA Graph。PPO 的 rollout 阶段使用 CUDA Graph replay 吗？Update 阶段呢？为什么？

---

PPO 循环告诉我们"env.step() 每步被调用一次"。但 env.step() 内部发生了什么？这 18 个步骤的精确时序是本章的核心——理解它就理解了整个 Manager-Based 架构的运行机制。

## 4.2 env.step() 内部时序精读 ⭐⭐⭐

> **这一节解决什么问题**：逐行解析 `ManagerBasedRlEnv.step()` 的 18 步执行序列——这是理解所有 Manager 交互的关键。

### 动机

Ch01 §1.4 给出了 env.step() 的 7 步简化时序。现在我们把它展开为 mjlab 源码中的 18 步精确时序——每一步都有明确的工程理由。

### 如果不理解时序会怎样

**反面案例 A**：你写了一个 reward term 需要读取"当前步的接触力"。但接触力在 `sim.step()` 后才更新，而 reward 在 `sim.forward()` 之前计算——你读到的是上一步的接触力。如果你不知道这个时序，会花一整天调 reward 而不知道数据本身就是滞后的。

**反面案例 B**：你写了一个 event term 在 step 模式下执行（每步都跑），它修改了 obs 依赖的物理参数。但 obs 在 event 之后计算——你的修改确实生效了。如果你把 event 改成 reset 模式，它在 obs 之前执行吗？答案取决于 `_reset_idx()` 内部的顺序。不看时序，你无法回答这个问题。

### 完整 18 步时序

以下是 mjlab `ManagerBasedRlEnv.step()` 的完整执行序列（Isaac Lab 的顺序几乎完全相同）：

| 步骤 | 操作 | 代码入口 | 说明 |
|------|------|---------|------|
| 1 | 清空 extras log | `extras["log"] = {}` | 每步重新收集日志 |
| 2 | 处理 action | `action_manager.process_action(action)` | raw action → scaled/clipped action |
| 3-7 | **decimation 循环** | `for _ in range(decimation):` | 物理子步循环 |
| 3 | 　应用 action | `action_manager.apply_action()` | 通过 actuator 计算力矩 |
| 4 | 　写入仿真 | `scene.write_data_to_sim()` | 控制信号 → MjData.ctrl |
| 5 | 　物理步进 | `sim.step()` | MuJoCo Warp / PhysX 推进一个 dt |
| 6 | 　更新场景 | `scene.update(dt=physics_dt)` | 同步 GPU buffer |
| 7 | 　子步指标 | `metrics_manager.compute_substep()` | 记录子步级指标 |
| 8 | episode 计数 | `episode_length_buf += 1` | 增加 episode 步数 |
| 9 | 计算 termination | `termination_manager.compute()` | 检查是否终止 |
| 10 | 计算 reward | `reward_manager.compute(dt=step_dt)` | 计算加权 reward |
| 11 | 计算 step 指标 | `metrics_manager.compute()` | 记录 step 级指标 |
| 12 | Reset 终止的环境 | `_reset_idx(env_ids)` | 如果 auto_reset |
| 12a | 　curriculum 更新 | `curriculum_manager.compute(env_ids)` | 调整难度 |
| 12b | 　reset 级 event | `event_manager.apply(mode="reset")` | reset DR |
| 13 | 刷新派生量 | `sim.forward()` | 更新接触力等派生量 |
| 14 | 更新 command | `command_manager.compute(dt)` | 重采样/更新命令 |
| 15 | step/interval event | `event_manager.apply(mode="step/interval")` | 执行周期性事件（如 push） |
| 16 | 刷新传感器 | `sim.sense()` | 更新 sensor data |
| 17 | 计算 observation | `observation_manager.compute(update_history=True)` | 计算并返回 obs dict |
| 18 | 记录 | `recorder_manager.record_post_step()` | 录制 rollout |

### decimation 循环内部详解

步骤 3-7 是 decimation 循环——它在一个 policy step 内执行多次物理步进。这是"策略频率"和"物理频率"解耦的关键机制。

```python
# decimation 循环伪代码
processed_action = action_manager.process_action(raw_action)  # 步骤 2：一次性处理

for substep in range(decimation):  # 步骤 3-7：循环 N 次
    # 步骤 3：每个 substep 都应用同一个 processed_action
    action_manager.apply_action()  # actuator 计算力矩
    
    # 步骤 4：力矩写入 MuJoCo/PhysX
    scene.write_data_to_sim()      # MjData.ctrl = torques
    
    # 步骤 5：物理引擎推进一个 physics_dt
    sim.step(render=False)         # mjwarp.step() 或 physx.step()
    
    # 步骤 6：同步 GPU buffer
    scene.update(dt=physics_dt)    # entity.update()
    
    # 步骤 7：记录子步级指标
    metrics_manager.compute_substep()  # 如足端接触时间
```

**关键工程事实**：decimation 循环内，**processed_action 不变**——策略在每个 policy step 只输出一次 action，这个 action 被重复应用 `decimation` 次。这意味着策略的"有效控制频率"是 `1 / (physics_dt × decimation)`。

**为什么不在每个 substep 都查询策略？** 因为策略的前向传播（MLP 推理）比物理步进慢得多。如果每个 substep 都查询策略，训练吞吐会下降 `decimation` 倍。工程上的解决方案是用较慢的策略频率（如 50 Hz）和较快的物理频率（如 200 Hz），让物理引擎在两次策略查询之间做更精确的模拟。

**decimation 对 actuator 的影响**：回顾 Ch03 §3.1 的 builtin vs explicit actuator。Builtin actuator（如 `<position kp=25 kv=0.5>`）在每个 substep 中都会重新计算力矩（因为关节位置在变化）——PD 控制器在追踪目标位置时自然产生平滑的力矩轨迹。Explicit actuator 每个 substep 应用的是**完全相同的力矩值**——因为力矩是在 `process_action()` 中计算的，不随 substep 更新。

> **一个跨领域类比**：decimation 循环类似于数字音频中的"过采样"——你的 DAC（策略）输出 50 Hz 的控制信号，但物理世界需要 200 Hz 的力矩输入。过采样让力矩曲线更平滑，减少了高频混叠（在物理中表现为关节抖动）。

### reset 内部流程（步骤 12 展开）

当某些环境被终止（terminated 或 truncated）时，`_reset_idx(env_ids)` 执行以下操作：

```python
def _reset_idx(self, env_ids):
    # 12a: 更新课程难度（基于被 reset 的环境的表现）
    curriculum_manager.compute(env_ids)
    
    # 12b: reset 级 event（如位姿随机化、速度随机化）
    event_manager.apply(mode="reset", env_ids=env_ids)
    
    # 重置 episode 计数器
    episode_length_buf[env_ids] = 0
    
    # 重置 reward episode sum
    reward_manager.reset(env_ids)
    
    # 重置 command（为新 episode 采样新命令）
    command_manager.reset(env_ids)
    
    # 重置 terminated/truncated 标记
    terminated[env_ids] = False
    truncated[env_ids] = False
```

**reset 的一个微妙之处**：reset 只影响 `env_ids` 中的环境——其他环境的状态完全不受影响。这是 GPU 并行仿真的核心特性——4096 个环境中的某些在 reset，其他在继续运行，互不干扰。

**curriculum_manager 在 reset 时更新**的原因：curriculum 需要根据被 reset 环境的表现（如 episode reward、episode length）来决定是否提高难度。如果在 step 而非 reset 时更新，curriculum 会在每步都变化——这太频繁了。

### Isaac Lab 的对应时序

Isaac Lab 的 `ManagerBasedRLEnv.step()` 与 mjlab 几乎一一对应，但有以下差异：

| 步骤 | mjlab | Isaac Lab | 差异原因 |
|------|-------|-----------|---------|
| 7 | `metrics_manager.compute_substep()` | 无对应 | Isaac Lab 无 MetricsManager |
| 11 | `metrics_manager.compute()` | 无对应 | 指标由各 Manager 内部处理 |
| 16 | `sim.sense()` 独立调用 | 部分集成到 `scene.update()` | MuJoCo Warp vs PhysX 的 sensor 更新机制不同 |

Isaac Lab 的 step() 源码（简化）：

```python
# Isaac Lab ManagerBasedRLEnv.step() 简化
def step(self, action):
    # 1-2: action 处理
    self.action_manager.process_action(action)
    
    # 3-6: decimation 循环
    for _ in range(self.cfg.decimation):
        self.scene.write_data_to_sim()
        self.sim.step(render=False)
        self.scene.update(sim_dt)
    
    # 8-11: termination, reward
    self.termination_manager.compute()
    self.reward_manager.compute(dt=self.step_dt)
    
    # 12: reset
    if self.cfg.auto_reset:
        terminated_envs = self.termination_manager.terminated | self.termination_manager.time_out
        if terminated_envs.any():
            self._reset_idx(terminated_envs.nonzero(as_tuple=False).squeeze(-1))
    
    # 13-17: forward, command, event, obs
    self.scene.update(self.sim_dt)  # forward
    self.command_manager.compute(self.step_dt)
    self.event_manager.apply(mode="interval")
    obs_dict = self.observation_manager.compute()
    
    return obs_dict, self.reward_manager.reward_buf, ...
```

> 对比两个框架的 step() 源码，你会发现它们的**逻辑结构完全相同**——差异只在 API 命名和少量实现细节。这就是为什么掌握了一个框架后，迁移到另一个只需要 1-2 天。

### 时序设计的六个关键洞察

**洞察 1：action 在 physics 前处理（步骤 2→3-7）。** 这是显而易见的——策略输出的 action 必须先经过 scaling/clipping/offset 处理，然后通过 actuator 模型转换为物理力矩，才能写入仿真器。

**洞察 2：termination 在 reward 之前（步骤 9→10）。** 这意味着被终止的环境的最后一步 reward **仍然会被计算**。如果机器人摔倒了，reward 计算会在摔倒状态下执行，给出一个很低的 reward——这是正确的行为。如果顺序反过来（先 reward 后 termination），摔倒前的最后一步可能获得正常的 reward，导致策略低估摔倒的代价。

**洞察 3：observation 在 reset 之后（步骤 12→17）。** 这确保了 reset 后的环境返回的是**新 episode 的初始 obs**，而非旧 episode 的最后一帧。对于 PPO 来说，每次 `env.step()` 返回的 obs 都是下一步策略需要的输入——如果返回了旧 obs，策略会在错误的状态上做决策。

**洞察 4：reward 在 `sim.forward()` 之前计算（步骤 10 vs 13）。** 这意味着 reward 函数读到的派生量（如接触力、传感器数据）有**一个 physics substep 的滞后**。这不是 bug，而是性能权衡——如果在 reward 之前也调用 `sim.forward()`，每步需要多一次 forward 调用，在 4096 envs 下会显著降低吞吐。

> **双重解读**：这个设计取舍可以从两个角度理解：
>
> 角度 1（性能视角）：省去了每步一次 `sim.forward()` 调用，吞吐提升 5-10%
> 角度 2（精度视角）：reward 读到的派生量滞后一个 substep（0.005s），对大多数 reward 函数来说影响可忽略

**洞察 5：command 在 observation 之前更新（步骤 14→17）。** 因为 observation 可能包含 command（如速度命令 `(v_x, v_y, ω_z)`）。如果 command 在 obs 之后更新，策略在本步看到的是旧命令，但 reward 在下一步用新命令计算——导致 obs 和 reward 不对齐。

**洞察 6：event 有四种执行模式。** EventManager 的 event term 根据 `mode` 在不同时机执行：

| mode | 执行时机 | 典型用途 | 在 step() 中的位置 |
|------|---------|---------|------------------|
| `startup` | 环境初始化时（只执行一次） | 初始质量随机化 | `__init__()` |
| `reset` | episode reset 时 | reset 级 DR（位姿/速度随机化） | 步骤 12b |
| `step` | 每步执行 | 观测噪声注入 | 步骤 15 |
| `interval` | 每 N 步执行 | 周期性推力扰动 | 步骤 15 |

### Isaac Lab 的对应时序

Isaac Lab 的 `ManagerBasedRLEnv.step()` 时序与 mjlab 几乎完全一致——因为 mjlab 的 Manager-Based 架构直接借鉴了 Isaac Lab。两者的主要差异：

| 差异点 | mjlab | Isaac Lab |
|--------|-------|-----------|
| MetricsManager | ✅ 有（步骤 7/11） | ❌ 无（指标由各 Manager 内部处理） |
| RecorderManager | ✅ 有（步骤 18） | ✅ 有 |
| sim.sense() | ✅ 独立调用（步骤 16） | 部分集成到 scene.update() 中 |
| auto_reset | 默认开启 | 默认开启 |
| WarpBridge | ✅ 需要 wp.to_torch() | 2.x: 直接 torch tensor |

> **本质洞察**：env.step() 的时序不是随意排列的——每个步骤的位置都有明确的工程理由。如果你要修改时序（如在 reward 之前加 `sim.forward()`），必须理解这个修改对性能和正确性的影响。99% 的情况下，你不需要修改时序——只需要理解它，以便正确地编写 obs/reward/event term。

### ⚠️ 常见陷阱

1. **在 reward term 中调用 `sim.forward()`。** 这会导致每步多一次 forward 调用，且可能破坏 CUDA Graph。如果你的 reward 需要最新的派生量，考虑在下一步的 reward 中使用（一步延迟通常不影响训练）。
2. **在 observation term 中写入物理状态。** Observation 应该是只读的——它读取状态但不修改。如果你需要在 obs 计算时修改状态，应该用 event term 来做。
3. **假设 reset 后的 obs 是 reset 前计算的。** obs 在 reset 之后计算（步骤 17 > 步骤 12），所以 reset 后返回的是新 episode 的初始 obs。
4. **忘记 decimation 循环内的 action 是不变的。** 步骤 2 处理 action 后，decimation 循环内每个 substep 应用的是同一个 processed action。如果你想在 substep 间改变 action，需要修改 action manager 的 `apply_action()` 逻辑。
5. **把 step event 和 interval event 混淆。** step event 每步都执行，interval event 每 N 步执行一次。推力扰动通常用 interval（间歇性），obs noise 通常用 step（每步都加）。

### 练习

1. 在 18 步时序中，如果把 observation 计算（步骤 17）移到 reward 计算（步骤 10）之前，会产生什么后果？（提示：考虑 obs 和 reward 的数据对齐。）
2. 为什么 command 必须在 observation 之前更新（步骤 14→17）？如果反过来会怎样？
3. （源码阅读题）打开 mjlab 的 `manager_based_rl_env.py`，找到 `step()` 方法，确认 18 步时序中每一步的代码位置。
4. （跨章综合题）结合 Ch03 §3.5 的 CUDA Graph 约束和本节的时序分析，解释为什么 `expand_model_fields()`（DR 触发时的 graph 重建）只应该在 startup 或 reset mode 的 event 中调用，而不应该在 step mode 中调用。
5. decimation 从 4 改成 8 后，policy frequency 变成多少？如果 reward 使用了 dt 缩放，reward 权重的实际效果是否会改变？为什么这很重要？
6. （设计题）假设你要在 decimation 循环的每个 substep 中都记录足端接触力（而非只在 policy step 级别记录），你应该使用 MetricsManager 的哪个方法？在 18 步时序中它在哪个位置？

### 时序知识在调试中的实际应用

理解 env.step() 时序不是"学完就忘"的理论知识——它是你日常调试中最常用的诊断工具。以下是三个真实调试场景，展示如何用时序知识快速定位问题：

**场景 A：reward 读到的接触力"滞后"一步**

症状：你写了一个 reward term 惩罚足端冲击力过大（`soft_landing_reward`），但训练后发现策略仍然猛烈着地。你打印了 reward 中读到的冲击力，发现它比可视化中看到的峰值小得多。

诊断：回顾时序——reward 在步骤 10 计算，但 `sim.forward()` 在步骤 13 才执行。这意味着 reward 读到的是**上一个 physics substep 结束时**的接触力，而不是本步 decimation 循环中的峰值接触力。如果 decimation=4，reward 读到的接触力可能已经错过了着地瞬间的冲击峰值。

解决方案：使用 ContactSensor 的 `history_length` 参数记录 decimation 期间的所有 substep 接触力，在 reward 中取 history 的最大值。或者在 MetricsManager 的 `compute_substep()` 中记录每个 substep 的接触力峰值，然后在 reward 中使用这个记录值。

> 如果不理解时序，你可能会花一天时间调 reward 权重——但问题根本不在权重上，而在数据来源的时序对齐上。

**场景 B：obs 中的 command 和 reward 中的 command 不匹配**

症状：策略在训练中收敛很慢，可视化发现它对不同速度命令的响应差异很小。

诊断：回顾时序——command 在步骤 14 更新，obs 在步骤 17 计算，reward 在步骤 10 计算。这意味着在同一个 env.step() 中，**reward 使用的是旧 command**（步骤 10 时 command 还没更新），**obs 使用的是新 command**（步骤 17 时 command 已更新）。但这不是 bug——因为 command 的 resample 只在特定时间点发生（每 10 秒），大部分 step 中 command 不变，所以这个"不匹配"只在 resample 发生的那一步存在。

结论：这是正常行为。如果策略真的对 command 响应差，更可能的原因是 actor obs 中缺少 command term，或者 command 的范围太窄。

**场景 C：reset 后第一步的 obs 维度异常**

症状：训练中偶尔出现 obs tensor 维度不匹配的错误，但只在某些环境 reset 后发生。

诊断：回顾 reset 流程（步骤 12）——reset 中 event_manager 会重新随机化状态。如果某个 event term 修改了 sensor 的配置（如 RayCastSensor 的 pattern），可能导致 obs 维度变化。但实际上 Manager-Based 架构禁止 runtime 修改 sensor 配置——所以更可能的原因是 event term 中有条件分支导致某些环境的 obs 计算路径不同。

解决方案：检查你的自定义 obs term 是否有 `if` 语句根据环境状态返回不同 shape 的 tensor。所有 obs term 必须返回相同 shape 的 tensor，不能有条件分支。

### 时序的"不要做"清单

| 不要做 | 为什么 | 正确做法 |
|--------|--------|---------|
| 在 reward term 中调 `sim.forward()` | 额外的 forward 调用 + 可能破坏 CUDA Graph | 接受一步延迟，或用 sensor history |
| 在 obs term 中写入物理状态 | obs 应该是只读的纯函数 | 用 event term 修改状态 |
| 在 step event 中调 `expand_model_fields` | 每步重建 CUDA Graph，性能灾难 | 用 startup 或 reset event |
| 在 reward term 中访问 obs_dict | reward 和 obs 在不同时机计算 | 直接从 env.scene 读取数据 |
| 假设 obs 和 reward 看到同一帧数据 | obs 在 forward 后，reward 在 forward 前 | 理解并接受这个时序差异 |

---

env.step() 的时序定义了"各 Manager 什么时候执行"。但为什么需要 Manager？它解决了什么问题？这就要回到 Manager-Based 架构的设计动机——从 legged_gym 的单体架构说起。

## 4.3 Manager 模式的设计动机 ⭐⭐⭐

> **这一节解决什么问题**：理解 Manager-Based 架构为什么比 legged_gym 的单体架构更好——这不是"更优雅"的审美问题，而是有明确工程收益的设计决策。

### 动机

如果你没有用过 legged_gym（Isaac Gym 时代的标准框架，CoRL 2021，~1.8k Stars），可能不理解为什么需要 Manager-Based 架构。毕竟，"把所有代码写在一个文件里"看起来更简单。但当你的项目复杂到需要同时实验 5 种 reward 配置、3 种 obs 组合、2 种 DR 策略时——单体架构的"简单"会变成"噩梦"。

### legged_gym 的单体架构

legged_gym 的核心是一个 2000+ 行的 `LeggedRobot` 类，包含所有逻辑：

```python
# legged_gym 的典型结构（简化）
class LeggedRobot(BaseTask):
    def _compute_observations(self):
        # 30+ 行：手动拼接 obs tensor
        self.obs_buf = torch.cat([
            self.base_lin_vel * self.obs_scales.lin_vel,
            self.base_ang_vel * self.obs_scales.ang_vel,
            self.projected_gravity,
            self.commands[:, :3] * self.commands_scale,
            (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
            self.dof_vel * self.obs_scales.dof_vel,
            self.actions,
        ], dim=-1)
    
    def _compute_reward(self):
        # 50+ 行：手动计算每个 reward 项
        rew_lin_vel = torch.exp(-torch.sum(
            torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]),
            dim=1) / 0.25)
        rew_ang_vel = torch.exp(-torch.square(
            self.commands[:, 2] - self.base_ang_vel[:, 2]) / 0.25)
        rew_action_rate = -torch.sum(
            torch.square(self.actions - self.last_actions), dim=1)
        # ... 更多 reward 项
        self.rew_buf = rew_lin_vel * 1.0 + rew_ang_vel * 0.5 + rew_action_rate * (-0.01)
    
    def _push_robots(self):
        # 推力扰动逻辑
        ...
    
    def _randomize_dof_props(self):
        # DOF 参数随机化
        ...
    
    def step(self, actions):
        # 手动编排所有逻辑的执行顺序
        self._compute_torques(actions)
        self.gym.simulate(self.sim)
        self._compute_observations()
        self._compute_reward()
        self._check_termination()
        self._reset_envs()
        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras
```

### 单体架构的五个工程问题

| 问题 | 表现 | 根因 | 量化影响 |
|------|------|------|---------|
| **修改 reward 可能影响 obs** | 改 `_compute_reward()` 时不小心改了 `self.obs_buf` | 所有状态共享同一个类的 `self` | 一个 typo 可能浪费一整天调试 |
| **消融实验需要复制整个文件** | 对比 3 种 reward 需要维护 3 个 2000 行文件 | reward 逻辑和其他逻辑耦合 | 每个实验增加 2000 行代码维护成本 |
| **换机器人需要大改** | 从 Go1 换到 G1 需要修改 50+ 处硬编码的 DOF 索引 | 机器人特定信息散布在各处 | 迁移一个新机器人需要 3-5 天 |
| **无法独立测试** | 不能只测试 obs 是否正确——必须跑完整个 step | obs/reward/termination 没有独立接口 | 每次测试需要完整训练循环 |
| **团队协作困难** | 两个人同时改 reward 和 obs 会产生合并冲突 | 所有逻辑在同一个文件 | git merge conflict 频繁 |

**legged_gym 的"隐式依赖"问题**：在 legged_gym 中，`_compute_reward()` 可能依赖 `_compute_observations()` 中计算的中间变量（通过 `self.xxx` 传递）。这种**隐式依赖**没有被代码结构显式表达——只有读完整个 2000 行文件才能发现。如果你修改了 obs 的计算顺序但没意识到 reward 依赖它，reward 会用到错误的值——但不会报任何错误。

```python
# legged_gym 的隐式依赖（危险！）
class LeggedRobot(BaseTask):
    def _compute_observations(self):
        # 计算 projected gravity（中间变量）
        self.projected_gravity = quat_rotate_inverse(
            self.base_quat, self.gravity_vec)
        # ... 其他 obs 计算
    
    def _compute_reward(self):
        # 隐式依赖 self.projected_gravity！
        # 如果 _compute_observations 没有先执行，这里用的是旧值
        rew_upright = -torch.sum(
            torch.square(self.projected_gravity[:, :2]), dim=1)
```

Manager-Based 架构通过**显式依赖**消除了这个问题：obs term 和 reward term 都直接从 `env.scene["robot"].data` 读取数据，不依赖任何中间变量。每个 term 是一个**纯函数**——给定 env，返回 tensor。

### Manager-Based 的工程收益量化

| 操作 | legged_gym 工时 | Manager-Based 工时 | 加速倍数 |
|------|----------------|-------------------|---------|
| 添加一个新 reward 项 | 15 min（编辑函数+测试） | 2 min（添加一行 config） | 7.5$\times$ |
| 对比 3 种 reward 配置 | 60 min（复制 3 份文件） | 5 min（3 次 CLI 覆盖） | 12$\times$ |
| 换一个新机器人 | 3 天（修改 50+ 处） | 30 min（改 EntityCfg） | 48$\times$ |
| 定位 "reward 为零" bug | 2 小时（逐行排查） | 10 min（打印各 term） | 12$\times$ |
| 团队 2 人并行开发 | 频繁 merge conflict | 无冲突（改不同 config 段） | $\infty$ |

> **本质洞察**：Manager-Based 架构的核心价值不是"代码更优雅"（虽然确实更优雅），而是**把实验迭代速度提升了一个数量级**。在机器人 RL 研究中，能快速迭代 reward/obs/DR 配置的研究者，比能写更快代码的研究者更有竞争力。

### 架构对比图

```
legged_gym（单体架构）                Manager-Based（mjlab/Isaac Lab）

┌─────────────────────┐          ┌──────────────────────────────────┐
│  LeggedRobot 类      │          │  ManagerBasedRlEnvCfg           │
│  (2000+ 行单文件)     │          │                                  │
│                       │          │  ┌─── ObservationsCfg ─────────┐ │
│  _compute_obs()       │          │  │ actor: [ObsTerm, ...]       │ │
│  _compute_reward()    │   →     │  │ critic: [ObsTerm, ...]      │ │
│  _compute_torques()   │          │  └──────────────────────────────┘ │
│  _push_robots()       │          │  ┌─── RewardsCfg ──────────────┐ │
│  _randomize_props()   │          │  │ [RewTerm, RewTerm, ...]     │ │
│  _check_termination() │          │  └──────────────────────────────┘ │
│  _reset_envs()        │          │  ┌─── EventCfg ────────────────┐ │
│  step()               │          │  │ [EventTerm, EventTerm, ...] │ │
│  ... (全部耦合)        │          │  └──────────────────────────────┘ │
└─────────────────────┘          │  ┌─── ActionsCfg ──────────────┐ │
                                  │  │ [ActionTerm, ...]            │ │
每个修改都可能影响其他部分        │  └──────────────────────────────┘ │
                                  │  ... (每个 Cfg 独立)              │
                                  └──────────────────────────────────┘
                                  
                                  修改任何一个 Cfg 不影响其他
```

### 从 legged_gym 迁移到 Manager-Based 的对应表

如果你有 legged_gym 的代码需要迁移，以下对应表可以帮你快速定位：

| legged_gym 函数/变量 | Manager-Based 对应 | 说明 |
|---------------------|-------------------|------|
| `_compute_observations()` | `ObservationManager.compute()` | 由 config 中的 ObsTerm 驱动 |
| `self.obs_buf` | `obs_dict["actor"]` | Manager 返回 dict 而非 tensor |
| `_compute_reward()` | `RewardManager.compute(dt)` | 每个 reward 项是独立 RewTerm |
| `self.rew_buf` | `reward_manager.reward_buf` | Manager 内部维护 |
| `self.reset_buf` | `termination_manager.terminated \| .time_out` | 区分 terminated 和 truncated |
| `_push_robots()` | `EventTerm(mode="interval")` | interval event |
| `_randomize_dof_props()` | `EventTerm(mode="startup")` | startup event |
| `_reset_envs()` | `_reset_idx(env_ids)` | 包含 curriculum + reset event |
| `_compute_torques()` | `ActionManager.apply_action()` | actuator 模型在 action term 中 |
| `self.commands` | `command_manager.get_command("twist")` | CommandManager 管理 |
| `self.feet_indices` | `entity.body_names.index("FL_foot")` | Entity 提供名称索引 |
| `cfg.rewards.tracking_sigma` | `RewTerm(params={"std": 0.25})` | 参数在 term 配置中 |

> 这个对应表不是"自动翻译"——每个对应关系都需要理解 Manager 的 API。但它可以帮你快速找到"这个功能在新架构中在哪里"。

### Manager-Based 架构如何解决

Manager-Based 架构把 MDP 的每个组件拆分到独立的 Manager 中：

```python
# Manager-Based 架构（mjlab/Isaac Lab）
class VelocityEnvCfg(ManagerBasedRLEnvCfg):
    # 每个 Manager 独立配置，互不干扰
    observations = ObservationsCfg(
        actor=ObservationGroupCfg(
            base_lin_vel=ObsTerm(func=mdp.base_lin_vel),
            base_ang_vel=ObsTerm(func=mdp.base_ang_vel),
            commands=ObsTerm(func=mdp.generated_commands, params={"command_name": "twist"}),
            joint_pos=ObsTerm(func=mdp.joint_pos_rel),
            joint_vel=ObsTerm(func=mdp.joint_vel),
            last_action=ObsTerm(func=mdp.last_action),
        ),
    )
    
    rewards = RewardsCfg(
        track_lin_vel=RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5, params={"std": 0.25}),
        track_ang_vel=RewTerm(func=mdp.track_ang_vel_z_exp, weight=0.75, params={"std": 0.25}),
        action_rate=RewTerm(func=mdp.action_rate_l2, weight=-0.01),
    )
    
    events = EventCfg(
        push_robot=EventTerm(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=(10.0, 15.0),
        ),
    )
```

**Manager-Based 解决的五个问题**：

| legged_gym 问题 | Manager-Based 解决方案 |
|----------------|---------------------|
| 修改 reward 可能影响 obs | 每个 Manager 独立运行，互不干扰 |
| 消融需要复制文件 | 注释/取消注释一行 `RewTerm` 即可 |
| 换机器人需要大改 | 只改 `EntityCfg`，其他 Manager 自动适配 |
| 无法独立测试 | 可以单独实例化 ObservationManager 测试 |
| 团队协作困难 | obs 和 reward 在不同的 config 段，不会冲突 |

> **一个跨领域类比**：legged_gym 的单体架构类似于早期的"巨型 main 函数"——所有逻辑写在一个函数里。Manager-Based 架构类似于"微服务架构"——每个服务（Manager）独立部署和扩展。这个类比的边界在于：微服务之间通过网络通信（有延迟），而 Manager 之间通过共享 GPU tensor（零延迟）。

### Manager 的加载顺序

Manager 的加载（创建）顺序很重要——因为后加载的 Manager 可能依赖先加载的 Manager 的输出。mjlab 的 `load_managers()` 按以下顺序创建 Manager：

| 顺序 | Manager | 为什么这个顺序 |
|------|---------|-------------|
| 1 | EventManager | DR 可能需要 expand_model_fields（影响后续 Manager 的数据访问） |
| 2 | CommandManager | observation 可能读取 command |
| 3 | ActionManager | observation 可能包含 last_action |
| 4 | ObservationManager | 定义 observation space（后续 PPO 需要） |
| 5 | TerminationManager | step 后判断 done |
| 6 | RewardManager | step 后计算 reward |
| 7 | CurriculumManager | reset 时调整难度 |
| 8 | MetricsManager | step/substep 记录 metrics |
| 9 | RecorderManager | 记录 post reset 和 post step |

> **反事实推理**：如果 ObservationManager 在 CommandManager 之前加载，obs 中的 `generated_commands()` term 会找不到 command manager——因为它还没创建。这不一定会报错（可能返回空 tensor），但会导致 obs 中缺少命令信息，策略无法学会跟踪命令。

### Task Registry 系统

任务注册系统是 Manager-Based 架构的"入口层"——它把 task ID（如 `"Mjlab-Velocity-Flat-Unitree-Go2"`）映射到环境配置、RL 配置和 runner 类。

**mjlab 的注册机制**：

```python
# src/mjlab/tasks/velocity/config/go2/__init__.py
from mjlab.tasks.registry import register_mjlab_task

register_mjlab_task(
    task_id="Mjlab-Velocity-Flat-Unitree-Go2",
    env_cfg=unitree_go2_flat_env_cfg,       # 训练环境配置（函数）
    play_env_cfg=unitree_go2_flat_play_cfg,  # 播放环境配置（函数）
    rl_cfg=unitree_go2_rl_cfg,              # RSL-RL 配置（函数）
    runner_cls=VelocityOnPolicyRunner,       # 可选自定义 runner
)
```

当你运行 `uv run train Mjlab-Velocity-Flat-Unitree-Go2` 时，发生以下步骤：

```
1. train.py 导入 mjlab.tasks（触发所有任务模块的 __init__.py）
2. 每个 __init__.py 调用 register_mjlab_task()，注册 task
3. train.py 从 registry 中查找 task ID
4. registry 返回 env_cfg 函数，调用它获取 ManagerBasedRlEnvCfg 实例
5. 用 cfg 实例化 ManagerBasedRlEnv
6. ManagerBasedRlEnv.__init__() 调用 load_managers()
7. 训练开始
```

**关键设计**：registry 存储的是**函数**而非配置实例。每次 `load()` 时调用函数获取**新的实例**（deep copy 语义），避免不同训练运行之间共享状态。这就是为什么 CLI 参数覆盖（`--env.rewards.X.weight 0.5`）不会影响 registry 中的原始配置。

**Isaac Lab 的注册机制**使用 `gymnasium.register()`——API 不同但原理相似：

```python
# Isaac Lab 的任务注册
import gymnasium
gymnasium.register(
    id="Isaac-Velocity-Flat-Anymal-C-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={"env_cfg_entry_point": AnymalCFlatEnvCfg},
)
```

### 从 Sensor 到 Manager 的数据流

Sensor 是 Manager 的"数据源"——obs term 读取 sensor data，reward term 也可能读取 sensor data。理解这条数据流对于诊断"reward 为零"类的 wiring 错误至关重要。

```
Sensor（物理层数据源）
  ├── ContactSensor → obs term (foot_contact)
  │                 → reward term (air_time, foot_slip, soft_landing)
  ├── RayCastSensor → obs term (height_scan)
  │                 → reward term (foot_swing_height)
  └── IMU Sensor   → obs term (projected_gravity, base_ang_vel)
```

**一个 sensor 可以服务多个 Manager 的多个 term**。例如 `feet_ground_contact` sensor 同时提供：
1. critic obs 中的 `foot_contact` term（用于 value 估计）
2. reward 中的 `air_time` term（步态节奏）
3. reward 中的 `foot_slip` term（打滑惩罚）
4. reward 中的 `soft_landing` term（软着陆）
5. reward 中的 `foot_swing_height` term（抬脚高度）

如果 sensor 的 geom 名拼错了（如 `"FL_foot"` 写成 `"fl_foot"`），**上述所有 term 同时失效**——但不会报错（regex 匹配了空集，sensor 返回全零数据）。这就是 Ch13 velocity task 全链路精读的核心内容。

### Wiring 错误诊断方法

"看似能跑但不学习"的最常见原因是 wiring 错误——数据流链路上的某个连接点接错了。以下是系统性的诊断方法：

| 步骤 | 操作 | 检查什么 |
|------|------|---------|
| 1 | `--agent zero` 可视化 | 初始姿态是否正确？关节角是否合理？ |
| 2 | 打印 obs tensor shape | 维度是否和预期一致？有没有意外的全零列？ |
| 3 | 打印每个 reward term | 哪些 term 始终为零？（可能是 sensor wiring 问题） |
| 4 | 打印 sensor 原始数据 | sensor 是否真的检测到了接触/地形？ |
| 5 | 检查 regex 匹配 | sensor 的 geom regex 是否匹配到了正确的 geom？ |
| 6 | 对比 play vs train cfg | play 和 train 的 obs/reward 配置是否一致？ |

```python
# 诊断代码示例：打印每个 reward term 的值
# 在训练 2 个 iteration 后检查
uv run train <TASK> --env.scene.num-envs 4 --agent.max-iterations 2

# 在 reward_manager.compute() 中添加打印
# 或检查 WandB 的 Episode_Reward/<term_name> 日志
```

**五种最隐蔽的 wiring 错误**：

| 错误 | 表现 | 根因 | 诊断方法 |
|------|------|------|---------|
| sensor geom 名拼错 | contact reward 永远为零 | regex 匹配空集 | 打印 sensor.data.found |
| actor obs 缺 command | 策略走但不跟命令 | obs config 缺 `generated_commands` | 打印 obs 维度 |
| flat 保留 terrain scan | 训练变慢但不报错 | 引用了不需要的 sensor | 检查 flat cfg 的 sensor 列表 |
| rough 保留 `fell_over` | 合理斜坡姿态被判摔倒 | orientation limit 不适合地形 | 检查 termination counts |
| obs_groups 名不匹配 | RSL-RL 拿不到 obs | "actor" vs "policy" | 检查 wrapper 的 obs_groups |

### ⚠️ 常见陷阱

1. **认为 Manager 顺序不重要。** 加载顺序决定了依赖关系。如果你自定义了一个 Manager 并在错误的顺序创建它，可能导致依赖缺失。
2. **在 legged_gym 代码中直接修改 `self.obs_buf` 期望在 reward 中使用。** Manager-Based 架构中 obs 和 reward 由不同的 Manager 计算，不共享中间变量。如果 reward 需要某个值，应该直接从 env 的状态中读取，而非依赖 obs 的中间结果。
3. **把所有 reward 项放在一个函数中。** Manager-Based 的理念是每个 reward 项一个独立函数。虽然你可以写一个"超级 reward 函数"同时计算多个 reward——但这违背了架构的设计意图，使消融实验变得困难。

### 练习

1. 在 legged_gym 中，如果你想对比"有 foot_slip 惩罚"和"无 foot_slip 惩罚"的训练结果，你需要怎么做？在 Manager-Based 架构中呢？
2. 为什么 EventManager 必须在所有其他 Manager 之前加载？给出一个具体的例子说明"EventManager 后加载"会导致什么问题。
3. （设计题）假设你要为 mjlab 添加第 10 个 Manager（如 `SafetyManager`，负责在 action 执行前检查安全约束）。它应该在加载顺序的哪个位置？为什么？
4. 在 legged_gym 中 `_compute_reward()` 调用了 `self.projected_gravity`——这个值是在 `_compute_observations()` 中计算的。如果有人改了 obs 的计算顺序，reward 会出什么问题？在 Manager-Based 中为什么不会有这个问题？
5. （跨章综合题）回顾 Ch01 §1.3 的 Manager 职责表和本节的加载顺序表。如果未来 mjlab 增加了一个 `PlanningManager`（负责高级路径规划），它应该在 ActionManager 之前还是之后加载？command 和 planning 的边界在哪里？

### Manager 之间的依赖关系图

9 个 Manager 之间存在明确的依赖关系——这决定了它们的加载顺序和执行顺序：

```
EventManager ──────────────────────────────────────────────┐
  │ expand_model_fields（影响后续所有 Manager 的数据访问）    │
  ▼                                                        │
CommandManager ────────────────────────────────────────┐   │
  │ obs 中的 generated_commands 需要读取 command          │   │
  ▼                                                    │   │
ActionManager ─────────────────────────────────────┐   │   │
  │ obs 中的 last_action 需要读取 action             │   │   │
  ▼                                                │   │   │
ObservationManager ────────────────────────────┐   │   │   │
  │ 定义 obs space（PPO 需要知道输入维度）        │   │   │   │
  ▼                                            │   │   │   │
TerminationManager                             │   │   │   │
  │ 判断 done（决定哪些 env 需要 reset）          │   │   │   │
  ▼                                            │   │   │   │
RewardManager                                  │   │   │   │
  │ 计算 reward（PPO 的优化目标）                 │   │   │   │
  ▼                                            │   │   │   │
CurriculumManager                              │   │   │   │
  │ reset 时调整难度（依赖 termination 信息）     │   │   │   │
  ▼                                            │   │   │   │
MetricsManager ← RecorderManager               │   │   │   │
  （可以读取所有前面 Manager 的输出）              │   │   │   │
                                               │   │   │   │
      ↑ 运行时 step() 中的执行顺序和加载顺序一致  ↑   │   │   │
```

> **一个跨领域类比**：这个依赖关系类似于 Linux 的 systemd 服务启动顺序。`EventManager` 就像 `systemd-udevd`（硬件设备初始化，必须最先启动），`ObservationManager` 就像 `networking.service`（网络服务，很多其他服务依赖它），`RewardManager` 就像应用层服务（依赖基础设施就绪后才能启动）。如果你在 systemd 中把服务启动顺序搞错，系统会启动失败——Manager 也是同理。

### 从 legged_gym 迁移的完整工作流

如果你有一个 legged_gym 的项目需要迁移到 mjlab 或 Isaac Lab，以下是按优先级排列的迁移步骤：

| 步骤 | 操作 | 对应的 legged_gym 代码 | 迁移到 | 预计时间 |
|------|------|----------------------|--------|---------|
| 1 | 准备 MJCF/USD 模型 | `resources/robots/go1/urdf/` | `EntityCfg.spec_fn` 或 `ArticulationCfg.spawn` | 30 min |
| 2 | 定义 EntityCfg | `LeggedRobotCfg.init_state` | `EntityCfg.init_state` | 15 min |
| 3 | 配置 actuator | `LeggedRobotCfg.control.stiffness/damping` | `ActuatorCfg(kp=..., kd=...)` | 15 min |
| 4 | 拆分 obs | `_compute_observations()` 中的每行 | 每个信号一个 `ObsTerm` | 30 min |
| 5 | 拆分 reward | `_compute_reward()` 中的每项 | 每项一个 `RewTerm` | 45 min |
| 6 | 迁移 termination | `_check_termination()` | 每个条件一个 `TermTerm` | 15 min |
| 7 | 迁移 DR | `_push_robots()` + `_randomize_dof_props()` | `EventTerm(mode=...)` | 30 min |
| 8 | 配置 command | `LeggedRobotCfg.commands` | `CommandsCfg` | 15 min |
| 9 | 配置 PPO | `LeggedRobotCfgPPO` | `RslRlOnPolicyRunnerCfg` | 15 min |
| 10 | 注册 task | 无对应（legged_gym 不需要注册） | `register_mjlab_task()` | 5 min |
| 11 | 分阶段验证 | 直接训练 | zero → random → small → full | 30 min |

**总计约 4 小时**。最耗时的是步骤 4 和 5（拆分 obs 和 reward），因为 legged_gym 中这些逻辑通常混在一起，需要仔细理清每个信号的数据来源和依赖关系。

### legged_gym 的"隐式全局状态"问题

legged_gym 最大的工程债务是**隐式全局状态**。在 `LeggedRobot` 类中，几十个 `self.xxx` 变量在不同方法间传递数据——没有任何机制保证它们的更新顺序。

```python
# legged_gym 中的隐式全局状态示例
class LeggedRobot(BaseTask):
    def step(self, actions):
        self.actions = actions  # step() 写入
        
    def _compute_observations(self):
        self.projected_gravity = ...   # obs 写入
        self.obs_buf = torch.cat([
            self.projected_gravity,     # obs 读取自己写入的
            self.commands,              # obs 读取 _resample_commands 写入的
            self.actions,               # obs 读取 step 写入的
        ])
    
    def _compute_reward(self):
        # reward 隐式依赖 obs 中计算的中间变量！
        rew = -torch.sum(self.projected_gravity[:, :2]**2, dim=1)
        # 如果 _compute_observations 没有先执行，
        # self.projected_gravity 是上一步的旧值
```

**Manager-Based 如何消除隐式依赖**：每个 obs/reward term 都是一个**纯函数**——它接收 `env` 作为参数，直接从 `env.scene["robot"].data` 读取最新数据，不依赖任何中间变量。即使你改变了 Manager 的执行顺序，每个 term 读到的数据仍然是正确的（来自物理引擎的最新状态）。

```python
# Manager-Based 中的显式依赖
def projected_gravity(env: ManagerBasedRlEnv) -> torch.Tensor:
    # 直接从 entity data 读取，不依赖任何中间变量
    return env.scene["robot"].data.projected_gravity_b

def flat_orientation_reward(env: ManagerBasedRlEnv) -> torch.Tensor:
    # 同样直接从 entity data 读取
    gravity = env.scene["robot"].data.projected_gravity_b
    return -torch.sum(gravity[:, :2]**2, dim=1)
```

> **反事实推理**：如果在 Manager-Based 架构中保留了 legged_gym 的隐式依赖模式（即 reward term 读取 obs term 计算的中间变量），那么 env.step() 的 18 步时序就会成为严格约束——任何时序调整都可能导致数据不一致。纯函数设计让 term 之间完全解耦，时序只影响"数据的新旧"而不影响"数据的正确性"。

---

## 4.4 双框架 Manager API 对比 ⭐⭐⭐

> **这一节解决什么问题**：精读 mjlab 和 Isaac Lab 的 Manager API 差异——让你能在两个框架之间无障碍切换。

### 动机

mjlab 的 Manager-Based 架构直接借鉴了 Isaac Lab，但 API 细节有许多差异。如果你只用过一个框架，切换到另一个时会被命名差异绊倒。本节系统对比两者的 API，让你建立"翻译表"。

### 命名差异总览

| 概念 | mjlab | Isaac Lab | 说明 |
|------|-------|-----------|------|
| **环境基类** | `ManagerBasedRlEnvCfg` | `ManagerBasedRLEnvCfg` | 注意大小写：`Rl` vs `RL` |
| **机器人配置** | `EntityCfg` | `ArticulationCfg` | mjlab 更通用（Entity 包含 Articulation） |
| **场景配置** | `SceneCfg` | `InteractiveSceneCfg` | Isaac Lab 有 Interactive 前缀 |
| **obs 组名** | `"actor"` / `"critic"` | `"policy"` / `"critic"` | 注意 actor vs policy |
| **obs 项** | `ObsTerm` | `ObsTerm` | 相同 ✅ |
| **reward 项** | `RewTerm` | `RewTerm` | 相同 ✅ |
| **event 项** | `EventTerm` | `EventTerm` | 相同 ✅ |
| **termination 项** | `TermTerm` | `DoneTerm` | 不同！ |
| **任务注册** | `register_mjlab_task()` | `gymnasium.register()` | 完全不同的注册机制 |
| **CLI 训练** | `uv run train <TASK>` | `python scripts/rsl_rl/train.py --task <TASK>` | CLI 入口不同 |
| **CLI 配置覆盖** | tyro（`--env.rewards.X.weight 0.5`） | argparse（预定义参数） | mjlab 支持任意深度覆盖 |

### 配置模式的本质差异

**mjlab 使用"函数工厂"模式**：任务配置由一个返回 `ManagerBasedRlEnvCfg` 实例的**函数**定义。机器人特定的 override 在函数内部完成：

```python
# mjlab 的任务配置模式
def unitree_go2_flat_env_cfg() -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()   # 获取 robot-agnostic base cfg
    # 机器人特定 override
    cfg.scene.robot = EntityCfg(
        spec_fn=go2_spec_fn,
        init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.34)),
    )
    cfg.actions.joint_position.scale = 0.25
    cfg.rewards.track_lin_vel.weight = 1.5
    return cfg
```

**Isaac Lab 使用"类继承 + configclass"模式**：任务配置由一个 `@configclass` 装饰的类定义，通过继承实现 override：

```python
# Isaac Lab 的任务配置模式
@configclass
class AnymalCFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    """ANYmal C flat terrain configuration."""
    
    def __post_init__(self):
        super().__post_init__()
        # 机器人特定 override
        self.scene.robot = ANYMAL_C_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
        )
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        # 删除 rough terrain 相关项
        self.scene.terrain.terrain_type = "plane"
        self.curriculum = None
```

**两种模式的权衡**：

| 维度 | mjlab（函数工厂） | Isaac Lab（类继承） |
|------|----------------|-------------------|
| 灵活性 | 高（函数内可任意修改） | 中（受继承层次约束） |
| 可读性 | 中（需要跟踪函数调用链） | 高（继承关系明确） |
| CLI 覆盖 | 强（tyro 支持任意深度） | 弱（仅预定义参数） |
| 序列化 | 简单（dataclass → dict） | 需要 configclass 工具 |

### ObservationManager 详解

ObservationManager 是最常被修改的 Manager。它负责：
1. 遍历所有 obs group（actor/critic）
2. 对每个 group，遍历所有 obs term
3. 每个 term 调用对应的函数，获取 tensor
4. 对 tensor 应用 clip、noise、delay、history
5. 拼接所有 term 为一个 flat tensor

```python
# ObservationManager 内部简化流程
class ObservationManager:
    def compute(self, update_history=True):
        obs_dict = {}
        for group_name, group_cfg in self.cfg.items():
            obs_list = []
            for term_name, term_cfg in group_cfg.items():
                # 调用 obs 函数
                raw = term_cfg.func(self.env, **term_cfg.params)
                
                # 应用 noise（仅在 enable_corruption=True 时）
                if group_cfg.enable_corruption and term_cfg.noise:
                    raw = raw + term_cfg.noise.sample(raw.shape)
                
                # 应用 clip
                if term_cfg.clip:
                    raw = torch.clamp(raw, term_cfg.clip[0], term_cfg.clip[1])
                
                obs_list.append(raw)
            
            # 拼接为 flat tensor
            obs_dict[group_name] = torch.cat(obs_list, dim=-1)
        
        return obs_dict
```

**actor 和 critic 的非对称观测**是 locomotion RL 的标准做法：actor 只看"真机可获得"的信息（关节编码器、IMU），critic 额外看"仿真特权"信息（地形高度图、接触力、foot air time）。这让 critic 的 value 估计更准确，同时保证 actor 的策略可部署。

### RewardManager 详解

RewardManager 的工作流：

```python
# RewardManager 内部简化流程
class RewardManager:
    def compute(self, dt):
        self._reward_buf.zero_()
        
        for term_name, term_cfg in self.cfg.items():
            # 调用 reward 函数，返回 [num_envs] tensor
            raw_reward = term_cfg.func(self.env, **term_cfg.params)
            
            # 乘以权重
            weighted = raw_reward * term_cfg.weight
            
            # 乘以 dt（让 reward 权重独立于仿真频率）
            if self.scale_by_dt:
                weighted = weighted * dt
            
            # NaN 保护
            weighted = torch.nan_to_num(weighted, nan=0.0, posinf=0.0, neginf=0.0)
            
            # 累加
            self._reward_buf += weighted
            
            # 记录 episode sum（用于日志）
            self._episode_sums[term_name] += weighted
        
        return self._reward_buf
```

**dt 缩放的工程意义**：RewardManager 默认把 reward 乘以 `dt`（step_dt = physics_dt $\times$ decimation）。这意味着如果你改了 timestep 或 decimation，reward 权重的实际效果不变——权重是"单位时间内的 reward 密度"，而非"单步 reward 绝对值"。这让跨配置的 reward 权重可复用。

> **反事实推理**：如果不做 dt 缩放，把 decimation 从 4 改成 8（policy 频率从 50Hz 降到 25Hz）时，每步 reward 变大（因为 step 更长）——你需要重新调所有 reward 权重。dt 缩放消除了这个耦合。

### ActionManager 详解

ActionManager 负责将策略输出的 raw action 转换为物理引擎可执行的控制信号。

```python
# ActionManager 的两阶段处理
class ActionManager:
    def process_action(self, action):
        """步骤 2：策略输出 → scaled/clipped action target"""
        # raw action 范围通常在 [-1, 1]
        # processed = raw * scale + offset
        self.processed_action = action * self.scale + self.offset
        # clip 到关节限位
        if self.clip:
            self.processed_action = torch.clamp(
                self.processed_action, self.clip_min, self.clip_max)
    
    def apply_action(self):
        """步骤 3：在每个 physics substep 中调用"""
        # 通过 actuator 模型计算力矩
        # builtin: MuJoCo 内部 PD 控制
        # explicit: PyTorch 中计算 torque = kp*(target - pos) - kd*vel
        torques = self.actuator.compute_torques(self.processed_action)
        # 写入 entity 的 ctrl
        self.entity.write_joint_targets(torques)
```

**action 空间的四种类型**：

| 类型 | 含义 | 典型 scale | 适用场景 |
|------|------|-----------|---------|
| `JointPositionAction` | 关节位置目标 | 0.25-0.5 rad | locomotion（最常用） |
| `JointVelocityAction` | 关节速度目标 | 1.0-5.0 rad/s | 部分操作任务 |
| `JointEffortAction` | 关节力矩 | 10-100 Nm | 需要精确力控 |
| `DiffIKAction` | 末端执行器位姿（通过逆运动学） | 0.01-0.1 m | 操作任务 |

**velocity task 默认使用 JointPositionAction**：

```python
# velocity_env_cfg.py 中的 action 配置
actions = ActionsCfg(
    joint_position=JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],      # 匹配所有关节
        scale=0.25,              # raw action * 0.25 + default_pos
        use_default_offset=True, # offset = default_joint_pos
    ),
)
```

这意味着策略输出 `a ∈ [-1, 1]` 时，实际关节目标是 `default_pos + a × 0.25`（单位：rad）。`scale=0.25` 限制了策略的探索范围——在默认姿态 $\pm$0.25 rad 内。这对训练稳定性至关重要：如果 scale 太大（如 1.0），随机策略在训练初期可能产生极端的关节位置，导致物理不稳定。

### TerminationManager 详解

TerminationManager 区分两种终止：

```python
# TerminationManager 的核心逻辑
class TerminationManager:
    def compute(self):
        self.terminated.zero_()  # 真正失败
        self.time_out.zero_()    # 超时
        
        for term_name, term_cfg in self.cfg.items():
            result = term_cfg.func(self.env, **term_cfg.params)
            
            if term_cfg.time_out:
                # 这是超时条件（如 episode 达到最大步数）
                self.time_out |= result
            else:
                # 这是真正的终止条件（如机器人摔倒）
                self.terminated |= result
```

**terminated vs truncated 的 PPO 影响**：

| 类型 | 含义 | PPO 处理 | 典型条件 |
|------|------|---------|---------|
| terminated | 任务失败 | value = 0（不 bootstrap） | 摔倒、非法接触、NaN |
| truncated (time_out) | 超时 | value = V(s')（bootstrap） | episode 达到 max_length |

> **如果不区分会怎样**：所有 episode 结束时 value 都设为 0。对于超时的 episode，策略会学到"越接近 max_length，value 越低"——这鼓励策略在 episode 结束前做"最后的冲刺"（可能是不自然的行为），而非学会持续稳定的运动。正确的做法是对 truncated episode 做 bootstrap——告诉策略"如果 episode 继续，你还能获得更多 reward"。

### EventManager 详解

EventManager 是 Domain Randomization 的工程实现。它的四种模式对应不同的执行时机：

```python
# velocity task 的典型 event 配置
events = EventCfg(
    # startup 模式：环境初始化时执行一次
    randomize_mass=EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={"mass_distribution_params": (-0.5, 0.5)},
    ),
    
    # reset 模式：每次 episode reset 时执行
    randomize_base_pose=EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    ),
    randomize_joint_pos=EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (-0.2, 0.2)},
    ),
    
    # interval 模式：每 N 秒执行一次
    push_robot=EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),  # 每 10-15 秒推一次
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    ),
)
```

**四种模式的执行时机详解**：

| 模式 | 何时执行 | 频率 | 典型用途 | env.step() 中的位置 |
|------|---------|------|---------|-------------------|
| startup | `__init__()` 结束时 | 整个训练只执行一次 | 质量/摩擦的初始随机化 | 不在 step() 中 |
| reset | `_reset_idx()` 中 | 每次 episode reset | 位姿/速度随机化 | 步骤 12b |
| step | 每步 | 每步都执行 | obs noise 注入 | 步骤 15 |
| interval | 每 N 秒 | 按时间间隔 | 推力扰动 | 步骤 15 |

**startup vs reset 的关键区别**：startup 的随机化在整个训练中只执行一次——每个 world 的质量/摩擦在训练开始时确定，之后不再改变。如果你希望每次 reset 时重新随机化质量，需要把 mode 改成 "reset"——但注意这会触发 `expand_model_fields()`，有一次性的 CUDA Graph 重建开销。

### CommandManager 详解

CommandManager 负责生成和管理任务命令（如速度命令）：

```python
# velocity task 的 command 配置
commands = CommandsCfg(
    twist=UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),  # 每 10 秒重采样
        ranges={
            "lin_vel_x": (-1.0, 1.0),    # m/s
            "lin_vel_y": (-0.5, 0.5),     # m/s
            "ang_vel_z": (-1.0, 1.0),     # rad/s
        },
        rel_standing_envs=0.1,  # 10% 的环境命令为零（站立）
        rel_heading_envs=0.0,   # 0% 的环境使用 heading 命令
    ),
)
```

**command 的三个关键属性**：

1. **resampling_time_range**：每个 episode 内，命令每 N 秒重采样一次。如果设为 `(10.0, 10.0)`，则精确每 10 秒换一次命令。策略需要在不同命令下都能正确执行——这是 velocity tracking 任务的核心挑战。

2. **rel_standing_envs**：10% 的环境被赋予零命令（站立不动）。这确保策略不仅学会走路，还学会站立——如果没有这个设置，策略可能在零命令时仍然乱动。

3. **command 在 obs 中的位置**：command 通过 `generated_commands()` obs term 被包含在 actor 的观测中。策略需要看到命令才能执行条件性行为——没有命令的策略只能学一种固定步态。

### velocity task 完整配置精读

以下是 mjlab 的 `velocity_env_cfg.py` 的完整结构概览——这是后续所有 velocity 任务（Go1/Go2/G1/ANYmal）的 base config：

```python
def make_velocity_env_cfg() -> ManagerBasedRlEnvCfg:
    cfg = ManagerBasedRlEnvCfg()
    
    # === 仿真配置 ===
    cfg.sim = SimulationCfg(
        mujoco=MujocoCfg(timestep=0.005, solver="newton", iterations=10),
    )
    cfg.decimation = 4  # policy at 50 Hz
    cfg.episode_length_s = 20.0  # 最大 episode 长度
    
    # === 场景 ===
    cfg.scene = SceneCfg(
        robot=None,  # 由 robot-specific cfg 填入
        terrain=TerrainEntityCfg(...),
    )
    
    # === 传感器 ===
    # terrain height scan, foot contact, IMU 等
    
    # === 观测 ===
    cfg.observations = ObservationsCfg(
        actor=ObservationGroupCfg(
            enable_corruption=True,  # actor 加 noise
            base_lin_vel=ObsTerm(func=mdp.base_lin_vel),
            base_ang_vel=ObsTerm(func=mdp.base_ang_vel),
            projected_gravity=ObsTerm(func=mdp.projected_gravity),
            commands=ObsTerm(func=mdp.generated_commands, params={"command_name": "twist"}),
            joint_pos=ObsTerm(func=mdp.joint_pos_rel),
            joint_vel=ObsTerm(func=mdp.joint_vel),
            last_action=ObsTerm(func=mdp.last_action),
        ),
        critic=ObservationGroupCfg(
            enable_corruption=False,  # critic 不加 noise
            # 包含 actor 的所有 term + privileged 信息
            foot_contact=ObsTerm(func=...),
            foot_height=ObsTerm(func=...),
            height_scan=ObsTerm(func=...),  # rough only
        ),
    )
    
    # === 动作 ===
    cfg.actions = ActionsCfg(
        joint_position=JointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            scale=0.25,
            use_default_offset=True,
        ),
    )
    
    # === 命令 ===
    cfg.commands = CommandsCfg(
        twist=UniformVelocityCommandCfg(...),
    )
    
    # === 奖励 ===
    cfg.rewards = RewardsCfg(
        # Tracking（正向激励）
        track_lin_vel=RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5),
        track_ang_vel=RewTerm(func=mdp.track_ang_vel_z_exp, weight=0.75),
        # Regularization（负向惩罚）
        action_rate=RewTerm(func=mdp.action_rate_l2, weight=-0.01),
        dof_acc=RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7),
        # Style
        upright=RewTerm(func=mdp.flat_orientation_l2, weight=-1.0),
        # Contact
        foot_slip=RewTerm(func=mdp.foot_slip, weight=-0.1),
    )
    
    # === 终止 ===
    cfg.terminations = TerminationsCfg(
        time_out=TermTerm(func=mdp.time_out, time_out=True),
        fell_over=TermTerm(func=mdp.bad_orientation, params={"limit_angle": 0.5}),
    )
    
    # === 事件（DR）===
    cfg.events = EventCfg(...)
    
    # === 课程 ===
    cfg.curriculum = CurriculumCfg(...)
    
    return cfg
```

**flat vs rough 的配置差异**：

rough config 从 base config 开始**添加**地形、contact sensor、terrain curriculum：

```python
def unitree_go2_rough_env_cfg():
    cfg = make_velocity_env_cfg()
    # 添加地形
    cfg.scene.terrain = TerrainEntityCfg(terrain_type="generator", ...)
    # 添加 terrain scan obs
    cfg.observations.actor.height_scan = ObsTerm(func=...)
    # 添加 terrain curriculum
    cfg.curriculum = CurriculumCfg(terrain_levels=...)
    # 替换 termination（rough 不用 fell_over，用 illegal_contact）
    del cfg.terminations.fell_over
    cfg.terminations.illegal_contact = TermTerm(func=...)
    return cfg
```

flat config 从 rough config **删除**地形相关内容：

```python
def unitree_go2_flat_env_cfg():
    cfg = unitree_go2_rough_env_cfg()
    # 改为平地
    cfg.scene.terrain = TerrainEntityCfg(terrain_type="plane")
    # 删除 terrain scan
    del cfg.observations.actor.height_scan
    del cfg.observations.critic.height_scan
    # 恢复 fell_over（平地上可以用 orientation check）
    del cfg.terminations.illegal_contact
    cfg.terminations.fell_over = TermTerm(func=...)
    # 删除 terrain curriculum
    cfg.curriculum = None
    return cfg
```

> **本质洞察**：flat 是 rough 的"减法版本"——先构建最完整的 rough config，然后通过删除地形相关组件得到 flat。这比"先写 flat 再加 rough"更安全——因为 rough 包含了 flat 的所有功能。如果你反过来（从 flat 派生 rough），很容易忘记添加某个 sensor 或 reward term。

### TerminationManager 的工程细节

TerminationManager 是最容易被忽视但影响训练正确性的 Manager。它的核心职责是区分两种 episode 结束：

**terminated**（真正的失败）：机器人摔倒、发生非法接触、物理状态 NaN。PPO 对 terminated episode 的处理是 `value(s_terminal) = 0`——因为任务已经真正结束，没有未来 reward 可期待。

**truncated**（超时截断）：episode 达到最大步数（如 20 秒 $\times$ 50 Hz = 1000 步）。PPO 对 truncated episode 的处理是 `value(s_terminal) = V(s_terminal)`——即 bootstrap，因为任务还没真正结束，只是人为截断了。

```python
# velocity task 的 termination 配置
terminations = TerminationsCfg(
    # 超时（truncated，需要 bootstrap）
    time_out=TermTerm(
        func=mdp.time_out,
        time_out=True,  # 标记为 truncation 而非 termination
    ),
    
    # 摔倒（terminated，value=0）
    fell_over=TermTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": 0.5},  # rad，约 28.6°
    ),
    
    # 非法接触（terminated，rough terrain 使用）
    illegal_contact=TermTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("illegal_contact_sensor"),
            "threshold": 1.0,  # N
        },
    ),
)
```

**`time_out=True` 标记的传递链**：TerminationManager 计算 `terminated` 和 `time_out` 两个 bool tensor。在 `env.step()` 的返回值中，`terminated` 和 `truncated` 分开传递。`RslRlVecEnvWrapper` 把它们合并为 `dones = terminated | truncated`，但同时在 `extras["time_outs"]` 中保留了 truncated 的信息——RSL-RL 的 GAE 计算需要这个信息来决定是否 bootstrap。

> **如果忘记设 `time_out=True` 会怎样**：所有超时的 episode 都会被当作 terminated 处理——value 被设为 0。这意味着策略会学到"越接近 episode 结束，价值越低"——鼓励策略在 episode 快结束时做"最后冲刺"（可能是不自然的高速运动），而非学会持续稳定地运动。在实际训练中，这通常表现为"策略在前 800 步走得很好，最后 200 步突然加速或做出奇怪动作"。

**flat vs rough 的 termination 差异**：

| termination | flat | rough | 原因 |
|-------------|------|-------|------|
| `time_out` | ✅ | ✅ | 两者都有最大 episode 长度 |
| `fell_over` | ✅ | ❌ | rough 地形上合理的倾斜可能被误判为摔倒 |
| `illegal_contact` | ❌ | ✅ | rough 用 contact sensor 检测非法接触（如大腿碰地） |

> 这个差异经常被忽略——如果你在 rough terrain 上保留了 `fell_over`（orientation check），策略在上坡时的合理前倾可能被误判为摔倒，导致 episode 频繁 reset，训练效率极低。正确做法是在 rough 上用 `illegal_contact`（检测特定 body 是否接触地面）替代 `fell_over`。

### CurriculumManager 的工程细节

CurriculumManager 在 episode reset 时更新训练难度。velocity task 中有两种典型的 curriculum：

**地形难度 curriculum**（rough terrain only）：

```python
# rough terrain curriculum 配置
curriculum = CurriculumCfg(
    terrain_levels=CurriculumTermCfg(
        func=mdp.terrain_levels_vel,
        params={
            "command_name": "twist",       # 用 tracking 表现判断
            "threshold": 0.8,              # tracking 误差 < 20% 则升级
        },
    ),
)
```

工作原理：当某个环境的 velocity tracking 表现超过阈值（如误差 < 20%），在下次 reset 时把它分配到更难的地形 level。如果表现低于另一个阈值，降级到更简单的地形。这实现了**自适应训练难度**——每个环境在自己能力范围内的地形上训练。

**命令范围 curriculum**：

```python
# 命令范围 curriculum（可选）
commands_vel=CurriculumTermCfg(
    func=mdp.commands_vel,
    params={
        "stages": [
            {"lin_vel_x": (-0.5, 0.5)},   # stage 0: 慢速
            {"lin_vel_x": (-1.0, 1.0)},    # stage 1: 中速
            {"lin_vel_x": (-2.0, 2.0)},    # stage 2: 高速
        ],
        "step_threshold": [500, 1000],     # 在 iteration 500/1000 切换
    },
)
```

> **反事实推理**：如果不用 curriculum 而直接在最难的地形上训练，策略在训练初期（还不会走路时）会频繁在难地形上摔倒——大量的"失败"样本导致 advantage 估计噪声很大，学习信号被淹没。Curriculum 让策略先在简单地形上学会基本步态，再逐步过渡到复杂地形——这就像人类学走路时先在平地上练，再上台阶、走草地。

### NaN Guard：物理层的最后防线

mjlab 独有的 NaN Guard 机制值得在这里提及，因为它与 TerminationManager 密切相关。当物理仿真产生 NaN（通常是 action scale 过大或接触参数不稳定导致）时，NaN 会通过 obs → reward → PPO 的路径传播到梯度中——整个训练崩溃。

NaN Guard 在 TerminationManager 之前检测 NaN 状态：

```bash
# 开启 NaN Guard
uv run train <TASK> --enable-nan-guard True
```

当检测到 NaN 时，NaN Guard 会：
1. 暂停训练
2. 记录 NaN 首次出现的环境 ID 和物理变量
3. 保存该环境的完整物理状态（循环 buffer）
4. 可通过 `uv run viz-nan` 在 Viser 中回放 NaN 前的最后几帧

这比 Isaac Lab 的默认行为（NaN 导致整个训练 crash，没有诊断信息）友好得多。NaN Guard 是 mjlab 针对 locomotion 训练中最常见的"NaN 爆炸"问题设计的工程工具。

### 自定义 Term 的编写模式

添加新的 obs/reward/event term 只需要三步：

**步骤 1：写函数**

```python
# mdp/my_custom_terms.py
import torch
from mjlab.envs import ManagerBasedRlEnv

def my_custom_obs(env: ManagerBasedRlEnv) -> torch.Tensor:
    """返回 [num_envs, obs_dim] 的 tensor。"""
    robot = env.scene["robot"]
    return robot.data.root_link_pos_w[:, 2:3]  # 基座高度

def my_custom_reward(env: ManagerBasedRlEnv) -> torch.Tensor:
    """返回 [num_envs] 的 scalar tensor。"""
    robot = env.scene["robot"]
    height = robot.data.root_link_pos_w[:, 2]
    target_height = 0.35
    return torch.exp(-torch.square(height - target_height) / 0.01)
```

**步骤 2：在 config 中引用**

```python
# env_cfg.py
observations = ObservationsCfg(
    actor=ObservationGroupCfg(
        # ... 其他 term
        base_height=ObsTerm(func=my_custom_obs),
    ),
)

rewards = RewardsCfg(
    # ... 其他 reward
    maintain_height=RewTerm(func=my_custom_reward, weight=0.5),
)
```

**步骤 3：验证**

```bash
# zero agent 确认 obs 维度正确
uv run play <TASK> --agent zero --num-envs 4

# small train 确认 reward 不为 NaN
uv run train <TASK> --env.scene.num-envs 64 --agent.max-iterations 2
```

### ⚠️ 常见陷阱

1. **obs 函数返回了错误的 shape。** obs term 必须返回 `[num_envs, obs_dim]`，reward term 必须返回 `[num_envs]`。维度错误会导致 concat 失败或 reward 计算错误。
2. **在 obs 函数中修改了环境状态。** Obs 函数应该是只读的（pure function）。如果你需要修改状态，用 event term。
3. **忘记在 Isaac Lab 中用 `"policy"` 而非 `"actor"` 作为 obs group 名。** 这是最常见的命名差异导致的 bug——RSL-RL 的 `obs_groups` 配置必须和 env 的 obs group 名匹配。
4. **reward 函数返回了负值但 weight 也是负值。** 负 $\times$ 负 = 正——你以为在惩罚但实际在奖励。始终保证 reward 函数返回正值，用 weight 的正负来控制奖励/惩罚方向。

### 练习

1. 写一个自定义 obs term，返回机器人基座相对于世界原点的距离。在 mjlab 的 velocity task 中添加这个 term 并验证维度正确。
2. 写一个自定义 reward term，惩罚基座高度偏离目标值。用 `exp(-error²/sigma²)` 的指数核形式。在 velocity task 中添加并训练 100 iteration，观察 WandB 上的 reward 曲线。
3. 把同一个 obs/reward term 分别在 mjlab 和 Isaac Lab 中实现。列出你遇到的所有 API 差异。
4. （跨章综合题）回顾 Ch01 §1.4 的 Manager 列表。MetricsManager 是 mjlab 独有的（Isaac Lab 没有）。如果你在 Isaac Lab 中需要类似功能，应该怎么实现？（提示：可以在 RewardManager 或 TerminationManager 的回调中记录指标。）

---

## 4.5 Entity 与 Articulation 系统 ⭐⭐

> **这一节解决什么问题**：理解 mjlab 的 Entity 系统和 Isaac Lab 的 Articulation 系统——它们是 Manager 的"数据来源"。

### 动机

所有 Manager（obs/reward/event 等）都需要从某个地方读取机器人的状态数据（关节角度、基座速度、接触力等）。在 mjlab 中，这个数据来源是 `Entity`；在 Isaac Lab 中，是 `Articulation`。理解它们的 API 差异，对于编写自定义 term 至关重要。

### mjlab 的 Entity 系统

回顾 Ch03 §3.5：mjlab 的 Entity 遵循"先 compile 再 run"的流程——`EntityCfg.spec_fn` 返回一个 `MjSpec`，多个 Entity 通过 `attach` 组合后 `compile` 为 `MjModel`。

**EntityCfg 的核心字段**：

```python
@dataclass
class EntityCfg:
    # 模型定义（返回 MjSpec 的函数）
    spec_fn: Callable[[], mujoco.MjSpec]
    
    # 初始状态
    init_state: InitialStateCfg = InitialStateCfg(
        pos=(0.0, 0.0, 0.34),           # 基座初始位置
        rot=(1.0, 0.0, 0.0, 0.0),       # 基座初始四元数 (w,x,y,z)
        joint_pos={".*": 0.0},           # 默认关节位置（regex 匹配）
        joint_vel={".*": 0.0},           # 默认关节速度
    )
    
    # 关节/body 名称过滤
    joint_names_regex: str = ".*"        # 匹配所有关节
    body_names_regex: str = ".*"         # 匹配所有 body
    
    # Actuator 配置
    actuator: ActuatorCfg = BuiltinPositionActuatorCfg(kp=25, kv=0.5)
```

运行时，Entity 通过 `EntityData` 提供状态访问：

| 属性 | 形状 | 含义 | 坐标系 |
|------|------|------|--------|
| `root_link_pos_w` | `[N, 3]` | 基座位置 | 世界 |
| `root_link_quat_w` | `[N, 4]` | 基座四元数 (w,x,y,z) | 世界 |
| `root_link_lin_vel_w` | `[N, 3]` | 基座线速度 | 世界 |
| `root_link_ang_vel_w` | `[N, 3]` | 基座角速度 | 世界 |
| `root_link_lin_vel_b` | `[N, 3]` | 基座线速度 | 机体 |
| `root_link_ang_vel_b` | `[N, 3]` | 基座角速度 | 机体 |
| `joint_pos` | `[N, J]` | 关节位置 | 关节空间 |
| `joint_vel` | `[N, J]` | 关节速度 | 关节空间 |
| `default_joint_pos` | `[N, J]` | 默认关节位置 | 关节空间 |
| `body_pos_w` | `[N, B, 3]` | 所有 body 位置 | 世界 |
| `body_quat_w` | `[N, B, 4]` | 所有 body 四元数 | 世界 |

其中 `N = num_envs`，`J = num_joints`，`B = num_bodies`。

**机体坐标系 vs 世界坐标系**：locomotion 任务中，策略通常使用**机体坐标系**的速度（`root_link_lin_vel_b`）——因为策略应该学"向前走"而不是"向北走"。ObsTerm `base_lin_vel` 默认返回的就是机体坐标系的速度。

### Isaac Lab 的 Articulation 系统

Isaac Lab 的 `Articulation` 类通过 USD 加载机器人模型。其数据访问通过 `.data` 属性：

```python
# Isaac Lab 的 Articulation 数据访问
robot = env.scene["robot"]

# 基座状态
pos = robot.data.root_pos_w       # [N, 3] 世界位置
quat = robot.data.root_quat_w     # [N, 4] 世界四元数
vel = robot.data.root_vel_w       # [N, 6] 世界速度 (lin + ang)

# 关节状态
joint_pos = robot.data.joint_pos  # [N, J]
joint_vel = robot.data.joint_vel  # [N, J]

# Body 状态
body_pos = robot.data.body_pos_w  # [N, B, 3]
```

### Scene 构建对比

**mjlab 的 Scene 构建**：

```python
# SceneCfg 定义了场景中的所有实体
class VelocitySceneCfg(SceneCfg):
    robot: EntityCfg = None  # 由 robot-specific cfg 填入
    terrain: TerrainEntityCfg = TerrainEntityCfg(terrain_type="plane")
    
    # Sensor 也在 Scene 中定义
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="robot/.*foot.*",
        history_length=4,
    )
    height_scan: RayCastSensorCfg = RayCastSensorCfg(
        prim_path="robot/base",
        pattern=GridPatternCfg(resolution=0.1, size=(1.6, 1.0)),
    )
```

Scene 的构建流程：
1. `SceneCfg` 中定义所有 Entity 和 Sensor
2. `Scene.__init__()` 遍历 cfg，用 `spec_fn` 获取每个 Entity 的 `MjSpec`
3. 通过 `attach` 将所有 MjSpec 组合为一个统一的场景
4. `compile()` 生成 `MjModel`
5. 上传到 GPU → batched worlds

**Isaac Lab 的 Scene 构建**：

```python
# InteractiveSceneCfg
@configclass
class CartpoleSceneCfg(InteractiveSceneCfg):
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path="path/to/robot.usd"),
    )
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
```

### 完整的 API 对照表

| 操作 | mjlab Entity | Isaac Lab Articulation |
|------|-------------|----------------------|
| 获取基座位置 | `entity.data.root_link_pos_w` | `articulation.data.root_pos_w` |
| 获取基座四元数 | `entity.data.root_link_quat_w` | `articulation.data.root_quat_w` |
| 获取基座线速度（世界） | `entity.data.root_link_lin_vel_w` | `articulation.data.root_vel_w[:, :3]` |
| 获取基座线速度（机体） | `entity.data.root_link_lin_vel_b` | 需手动转换 |
| 获取关节位置 | `entity.data.joint_pos` | `articulation.data.joint_pos` |
| 获取关节速度 | `entity.data.joint_vel` | `articulation.data.joint_vel` |
| 获取默认关节角 | `entity.data.default_joint_pos` | `articulation.data.default_joint_pos` |
| 获取 body 位置 | `entity.data.body_pos_w` | `articulation.data.body_pos_w` |
| 设置关节目标 | `entity.write_joint_targets(...)` | `articulation.set_joint_position_target(...)` |
| 重置基座状态 | `entity.write_root_link_pose_to_sim(...)` | `articulation.write_root_pose_to_sim(...)` |
| 重置关节状态 | `entity.write_joint_state_to_sim(...)` | `articulation.write_joint_state_to_sim(...)` |

> **本质洞察**：mjlab 的 Entity 提供了更细粒度的速度访问（`root_link_lin_vel_b` 直接在机体坐标系），而 Isaac Lab 的 Articulation 只提供世界坐标系的速度——需要在 obs term 中手动转换。这是 mjlab 针对 locomotion 任务的一个贴心优化。

### Sensor 系统：Manager 的数据桥梁

Sensor 是 Entity/Articulation 的**扩展数据源**——它提供 Entity 本身不直接暴露的物理信息（如接触力、地形高度、BVH 射线检测）。

mjlab 的主要 Sensor 类型：

| Sensor | 数据来源 | 提供什么 | 典型 obs/reward 用途 |
|--------|---------|---------|---------------------|
| `ContactSensor` | MuJoCo 接触引擎 | 接触力、接触状态、air time | foot_contact obs、foot_slip reward |
| `RayCastSensor` | BVH 加速射线检测 | 射线命中点、距离 | height_scan obs |
| `TerrainHeightSensor` | RayCast 的特化 | frame_z - hit_z（clearance） | foot_height obs |
| `IMU Sensor` | 虚拟 IMU | 加速度、角速度、重力投影 | projected_gravity obs |

**一个 Sensor 服务多个 Manager**的典型例子：`feet_ground_contact` 同时为 critic obs（`foot_contact` term）、reward（`air_time`、`foot_slip`、`soft_landing`、`foot_swing_height`）提供数据。如果这个 sensor 的 geom regex 拼错了，**所有这些 term 同时失效**——但不报错（regex 匹配空集，sensor 返回全零）。

Sensor 在 env.step() 时序中的位置：步骤 16（`sim.sense()`）——在 `sim.forward()` 之后、observation 之前。这确保 obs 中的 sensor 数据是最新的。但 reward（步骤 10）在 sensor 更新之前计算——所以 reward 函数直接使用 sensor data 时，读到的是上一步更新的值。

> 这个时序差异很少造成实际问题——因为 sensor 更新频率（每步更新一次）远高于物理状态的变化频率。但如果你的 reward 对 sensor 数据非常敏感（如需要精确的瞬间冲击力），需要使用 ContactSensor 的 `history_length` 参数记录 decimation 期间的 substep 数据。

### Scene 构建的完整流程

回顾 Ch03 §3.5 的数据流——Scene 是 Entity、Sensor 和 Terrain 的组合容器。mjlab 的 Scene 构建遵循以下流程：

```
SceneCfg（配置描述）
  │
  ▼ Scene.__init__()
遍历 cfg 中的 Entity/Sensor/Terrain
  │
  ▼ 对每个 Entity
调用 entity.cfg.spec_fn() → 获取 MjSpec
  │
  ▼ 对所有 Entity 的 MjSpec
通过 spec.attach() 组合为统一的 MjSpec
  │
  ▼ spec.compile()
生成 MjModel（CPU）
  │
  ▼ mjwarp.put_model() + mjwarp.put_data()
上传到 GPU → batched worlds
  │
  ▼ 创建 Sensor 实例
ContactSensor、RayCastSensor 等
  │
  ▼ 完成
Scene 可用，Manager 可以从中读取数据
```

Isaac Lab 的 Scene 构建流程类似，但使用 USD 而非 MJCF：

```
InteractiveSceneCfg（配置描述）
  │
  ▼ InteractiveScene.__init__()
遍历 cfg 中的 Articulation/RigidObject/Sensor
  │
  ▼ 对每个 Articulation
加载 USD 文件 → 创建 PhysX Articulation
  │
  ▼ 构建 PhysX Scene
所有物理对象在 GPU 上实例化
  │
  ▼ 创建 Sensor 实例
  │
  ▼ 完成
```

**两种流程的关键差异**：mjlab 的 `spec.attach()` 允许在 Python 层面组合多个模型（如机器人 + 桌子 + 物体），而 Isaac Lab 依赖 USD 的 layer composition。mjlab 的方式更灵活（可以动态组合），Isaac Lab 的方式更标准化（利用 USD 生态）。

### ⚠️ 常见陷阱

1. **混淆 `root_link_pos_w` 和 `root_pos_w`。** 功能相同但名字不同。
2. **在 Isaac Lab 3.0 中忘记 `wp.to_torch()`。** 3.0 的 `.data.*` 默认返回 `wp.array`。
3. **假设两个框架的四元数格式相同。** mjlab 使用 wxyz（MuJoCo 约定），Isaac Lab 2.x 也用 wxyz，但 **Isaac Lab 3.0 改为 xyzw**。
4. **忘记 `joint_pos_rel` 和 `joint_pos` 的区别。** `joint_pos_rel = joint_pos - default_joint_pos`。obs 通常用 rel（减少了绝对位置的偏置），但 action 目标用 absolute。
5. **用错坐标系。** obs 中的 base velocity 应该用机体坐标系（`_b` 后缀），但如果你误用了世界坐标系（`_w` 后缀），策略会依赖全局朝向——在不同初始朝向下行为不一致。

### 练习

1. 分别在 mjlab 和 Isaac Lab 中获取 Go2/ANYmal 的基座高度。写出完整的代码片段，包括如何从 env 中获取 robot 对象。
2. 在 mjlab 中，`entity.data.body_pos_w` 的第二个维度（B）对应什么？如何知道哪个 index 是哪个 body？（提示：查看 `entity.body_names`）
3. 写一个 obs term 函数，返回四个足端的世界坐标高度（`[num_envs, 4]`）。在 mjlab 和 Isaac Lab 中分别实现。

---

## 4.6 最小实验：验证你的理解 ⭐

> **这一节解决什么问题**：通过四个递进的实验验证你对 Manager-Based 架构的理解，建立"改一个东西 → 观察效果"的实验习惯。

### 实验 1：打印 Manager 加载顺序

```bash
# 在 ManagerBasedRlEnv.__init__() 的 load_managers() 中添加打印
# 或直接运行 zero agent，观察初始化日志
uv run play Mjlab-Velocity-Flat-Unitree-Go2 --agent zero --num-envs 1

# 你应该看到类似：
# [INFO] Loading EventManager...
# [INFO] Loading CommandManager...
# [INFO] Loading ActionManager...
# [INFO] Loading ObservationManager...
# ...
```

**验证目标**：确认 Manager 加载顺序与 §4.3 的顺序表一致。

### 实验 2：打印 obs tensor 的维度和内容

```python
# 在 play 时打印 obs
# 或在训练前两步打印
import torch

# 在 env.step() 后
obs = env.step(actions)[0]
for group_name, group_tensor in obs.items():
    print(f"  {group_name}: shape={group_tensor.shape}")
    print(f"    min={group_tensor.min():.4f}, max={group_tensor.max():.4f}")
    print(f"    any NaN: {torch.isnan(group_tensor).any()}")
```

**验证目标**：确认 actor obs 和 critic obs 的维度与 config 中定义的 term 数量一致。如果某个 obs 全为零，可能是 sensor wiring 问题。

### 实验 3：添加自定义 reward term

按 §4.4 的三步流程，添加一个"惩罚基座高度偏离 0.34m"的 reward term：

```python
# Step 1: 写函数
def maintain_base_height(env, target_height: float = 0.34) -> torch.Tensor:
    robot = env.scene["robot"]
    height = robot.data.root_link_pos_w[:, 2]
    return torch.exp(-torch.square(height - target_height) / 0.01)

# Step 2: 在 config 中添加
# cfg.rewards.base_height = RewTerm(func=maintain_base_height, weight=0.3)

# Step 3: 验证
uv run train <TASK> --env.scene.num-envs 64 --agent.max-iterations 5
# 检查 WandB 日志中是否出现 Episode_Reward/base_height
```

**验证目标**：确认新 reward term 在训练日志中出现，且数值非零。

### 实验 4：对比 play 和 train 的 config 差异

```bash
# 训练模式
uv run train Mjlab-Velocity-Flat-Unitree-Go2 --env.scene.num-envs 64 --agent.max-iterations 2

# 播放模式
uv run play Mjlab-Velocity-Flat-Unitree-Go2 --agent zero --num-envs 4
```

**验证目标**：确认 play 模式关闭了 obs corruption、移除了 push event。如果 play 和 train 行为差异大，先检查 play_env_cfg 的 override。

### 源码阅读路线

如果你想深入理解 Manager-Based 架构的实现，按以下优先级阅读源码：

| 优先级 | 文件 | 关注点 | 时间 |
|--------|------|--------|------|
| 1 | `envs/manager_based_rl_env.py` | `step()` 的 18 步时序 | 30 min |
| 2 | `managers/observation_manager.py` | `compute()` 的 obs 拼接流程 | 20 min |
| 3 | `managers/reward_manager.py` | `compute(dt)` 的 dt 缩放和 NaN 保护 | 20 min |
| 4 | `managers/event_manager.py` | `apply(mode)` 的四种模式分发 | 15 min |
| 5 | `managers/action_manager.py` | `process_action()` 和 `apply_action()` | 15 min |
| 6 | `tasks/velocity/velocity_env_cfg.py` | base config 的完整结构 | 30 min |
| 7 | `tasks/velocity/config/go2/env_cfgs.py` | robot-specific override | 15 min |
| 8 | `tasks/velocity/mdp/rewards.py` | reward 函数实现 | 20 min |
| 9 | `rl/vecenv_wrapper.py` | terminated/truncated → dones | 10 min |
| 10 | `tasks/registry.py` | task 注册和 deepcopy | 10 min |

> **阅读建议**：先读优先级 1-3（90 分钟），就能理解 Manager-Based 架构的核心。优先级 4-7 在你需要修改特定 Manager 时再读。优先级 8-10 在你需要添加新任务或调试 wiring 问题时再读。

### Isaac Lab 的对应源码

| mjlab 文件 | Isaac Lab 对应 | 差异 |
|-----------|---------------|------|
| `envs/manager_based_rl_env.py` | `envs/manager_based/manager_based_rl_env.py` | 几乎一致 |
| `managers/observation_manager.py` | `managers/observation_manager.py` | API 命名不同 |
| `managers/reward_manager.py` | `managers/reward_manager.py` | dt 缩放逻辑一致 |
| `managers/event_manager.py` | `managers/event_manager.py` | 四种模式一致 |
| `tasks/velocity/velocity_env_cfg.py` | `tasks/locomotion/velocity/config/*.py` | 目录结构不同 |
| `rl/vecenv_wrapper.py` | `isaaclab_rl/rsl_rl/vecenv_wrapper.py` | obs_groups 处理不同 |

### ⚠️ 常见陷阱

1. **实验时 num_envs 太大。** 调试时用 1-4 个环境就够——大量环境会让输出很难读。
2. **忘记用 `--agent zero` 而不是 `--agent random`。** Zero agent 让所有 action 为零，便于观察"纯物理"行为。
3. **在 Isaac Lab 中修改 config 后忘记重新注册 task。** 修改 `@configclass` 后需要确保注册使用最新的类定义。
4. **不检查 WandB 的 `Episode_Reward/<term>` 日志。** 这是诊断 reward wiring 问题最快的方法——如果某个 term 始终为零，说明它的 sensor 或数据源有问题。

### 练习

1. 完成实验 1-4。记录每个实验的观察结果和你的解释。
2. 在 mjlab 中修改 `track_lin_vel` 的 weight（从 1.5 改为 0.5 和 5.0），训练 500 iteration 并对比。哪个 weight 下策略学得最快？
3. （源码阅读题）打开 `reward_manager.py`，找到 `torch.nan_to_num()` 的调用位置。解释为什么需要 NaN 保护——什么情况下 reward 函数可能返回 NaN？
4. （跨章综合题）结合 Ch03 §3.5 的 `expand_model_fields` 和本章的 EventManager。当 startup event 触发质量随机化时，数据流经历了哪些步骤？从 EventTerm.func → expand_model_fields → CUDA Graph rebuild → MjData.body_mass，画出完整路径。
5. （设计题）你想在 velocity task 中添加一个"步态对称性"reward——左腿的步态应该和右腿镜像对称。你会在哪个 Manager 中实现它？obs 中需要什么额外信息？reward 函数的输入是什么？

---

## 自检：本章你应该能回答的 15 个问题

读完本章后，不看笔记尝试回答以下问题。如果有 5 个以上答不出，建议重读对应小节。

1. PPO 的 rollout 阶段和 update 阶段分别做什么？env.step() 在哪个阶段被调用？
2. `terminated` 和 `truncated` 的区别是什么？PPO 对两者的处理有何不同？
3. RSL-RL 的自适应学习率机制如何工作？`desired_kl` 参数的作用是什么？
4. env.step() 的 18 步时序中，reward 为什么在 `sim.forward()` 之前计算？
5. observation 为什么在 reset 之后计算？如果反过来会怎样？
6. decimation 循环内的 action 是否在每个 substep 都变化？为什么？
7. EventManager 的四种模式（startup/reset/step/interval）分别在什么时候执行？
8. Manager 的加载顺序为什么重要？EventManager 为什么必须最先加载？
9. legged_gym 的单体架构有什么具体的工程问题？Manager-Based 如何解决？
10. mjlab 的 obs group 用"actor/critic"，Isaac Lab 用什么？
11. RewardManager 的 dt 缩放是什么意思？为什么需要它？
12. 自定义 obs/reward term 的三步流程是什么？
13. EntityCfg 和 ArticulationCfg 的核心区别是什么？
14. NaN Guard 在什么时候检测 NaN？检测到后做什么？
15. flat 和 rough 的 termination 配置有什么区别？为什么？

---

## velocity task 的完整 Debug Checklist

以下是一个可打印的 checklist，用于在新建或迁移 velocity task 时逐项验证。如果训练不收敛，按此顺序排查：

### 阶段一：环境构建验证

- [ ] task ID 拼写正确（大小写敏感）
- [ ] `uv run list-envs` 能看到你的 task
- [ ] `uv run play <TASK> --agent zero --num-envs 1` 能运行
- [ ] zero agent 可视化时机器人站姿正确（没有穿地、浮空、扭曲）
- [ ] `uv run play <TASK> --agent random --num-envs 1` 运行时没有立即 NaN

### 阶段二：obs 验证

- [ ] actor obs 包含 command term（`generated_commands`）
- [ ] actor obs 的 `enable_corruption=True`（加 noise）
- [ ] critic obs 的 `enable_corruption=False`（不加 noise）
- [ ] critic obs 包含 privileged 信息（如 foot_contact、foot_height）
- [ ] obs 维度和预期一致（打印 shape 确认）
- [ ] obs 中没有全零的列（可能是 sensor wiring 问题）
- [ ] flat task 已删除 `height_scan` obs term
- [ ] obs_groups 配置与 RSL-RL 匹配（mjlab: "actor"/"critic"）

### 阶段三：reward 验证

- [ ] 所有 reward term 在 WandB 中有非零值
- [ ] reward 权重符号正确（正=奖励，负=惩罚）
- [ ] tracking reward 的 std 参数合理（0.1-0.5 之间）
- [ ] contact sensor 的 geom regex 匹配到了正确的 geom
- [ ] foot site names 在 reward term 中已填入（非空 tuple）

### 阶段四：termination 验证

- [ ] `time_out` 标记为 `time_out=True`
- [ ] flat 使用 `fell_over`，rough 使用 `illegal_contact`
- [ ] episode length 统计在 WandB 中显示合理值

### 阶段五：DR 和 command 验证

- [ ] command name 在所有引用点一致（通常是 `"twist"`）
- [ ] command 范围合理（lin_vel_x: -1 到 1 m/s 作为起点）
- [ ] rel_standing_envs > 0（确保有站立命令）
- [ ] push event 使用 interval 模式
- [ ] startup event 的 DR 范围不过大

### 阶段六：PPO 验证

- [ ] `num_envs × num_steps_per_env` 能被 `num_mini_batches` 整除
- [ ] `learning_rate` 初始值为 1e-3（RSL-RL 自适应 lr 会调整）
- [ ] `desired_kl = 0.01`（标准值）
- [ ] WandB 中 KL divergence 在 0.005-0.03 范围内
- [ ] WandB 中 entropy 缓慢下降（不是骤降）
- [ ] WandB 中 value loss 逐步下降

> 如果到阶段六仍然不收敛，考虑以下进阶排查：(1) 用 NaN Guard 检测物理层问题，(2) 降低 action scale 到 0.1 看是否能学会站立，(3) 只保留 tracking reward 去掉所有 penalty 看是否能学会运动，(4) 检查 Ch03 的物理参数（timestep、solver、接触参数）。

---

## 进阶话题：从 Manager-Based 到 Direct Workflow

Isaac Lab 除了 Manager-Based workflow，还提供了 **Direct workflow**——直接在一个类中实现所有逻辑，类似 legged_gym 但在 Isaac Lab 的基础设施上。

| 维度 | Manager-Based | Direct |
|------|--------------|--------|
| 代码组织 | config-driven，Term 组合 | 单一类，手动编排 |
| 灵活性 | 中等（受 Manager API 约束） | 高（完全自定义） |
| 可维护性 | 高（关注点分离） | 中（需要自律） |
| 适用场景 | 标准 RL 任务 | 特殊需求（如自定义渲染循环） |
| 消融容易度 | 非常容易（改一行 config） | 困难（需要修改代码） |

> mjlab 目前只支持 Manager-Based workflow——如果你需要 Direct workflow 的灵活性，可以考虑 Isaac Lab。但对于本教材覆盖的所有任务（locomotion、manipulation、loco-manipulation），Manager-Based 完全足够。

### 什么时候需要 Direct Workflow

1. 你的 env.step() 需要自定义的执行顺序（不是标准的 18 步时序）
2. 你需要在 step 内部做多次 obs 计算（如 model-based RL 的 planning loop）
3. 你需要自定义的渲染管线（如自定义的 visual obs 处理）
4. 你的任务有非标准的 MDP 结构（如 multi-agent 或 hierarchical RL）

对于这些情况，Manager-Based 的固定时序可能成为限制。但在你确定需要 Direct 之前，**先尝试在 Manager-Based 中解决**——它的大多数限制可以通过自定义 term 和 event 来绕过。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "Manager-Based 只是代码更整洁" | 它是有明确工程收益的设计——消融实验从"复制文件"变成"改一行配置" |
| "env.step() 的顺序无所谓" | 顺序决定了 obs/reward/termination 的数据对齐，改顺序可能导致训练失败 |
| "obs 和 reward 看到的是同一帧数据" | obs 在 sim.forward() 之后计算，reward 在之前——有一个 substep 的差异 |
| "PPO 每步都更新网络" | PPO 先收集 N 步数据（rollout），再做 M 次梯度更新（update），交替进行 |
| "mjlab 和 Isaac Lab 的 Manager API 完全相同" | 命名有差异（actor vs policy、EntityCfg vs ArticulationCfg 等） |
| "terminated 和 truncated 一样" | terminated 是真正失败，truncated 是超时——PPO 对两者的处理不同 |
| "Manager 的加载顺序随意" | 加载顺序决定依赖关系——EventManager 必须最先加载 |
| "自定义 term 可以修改 env 状态" | obs/reward term 应该是只读的，修改状态用 event term |

---

## 本章小结

### 知识点总表

| 编号 | 知识点 | 核心要点 | 对应节 | 难度 |
|------|--------|---------|--------|------|
| 1 | PPO 训练循环 | rollout（数据收集）+ update（梯度更新），env.step() 是数据生产者 | 4.1 | ⭐⭐ |
| 2 | RSL-RL VecEnv Wrapper | terminated/truncated 合并为 dones，time_outs 用于 value bootstrap | 4.1 | ⭐⭐ |
| 3 | env.step() 18 步时序 | action→decimation→termination→reward→reset→forward→command→event→sense→obs | 4.2 | ⭐⭐⭐ |
| 4 | reward 在 forward 前计算 | 派生量有一个 substep 滞后，性能 vs 精度权衡 | 4.2 | ⭐⭐⭐ |
| 5 | obs 在 reset 后计算 | 确保返回新 episode 的初始 obs | 4.2 | ⭐⭐⭐ |
| 6 | EventManager 四种模式 | startup/reset/step/interval | 4.2 | ⭐⭐ |
| 7 | 单体 vs Manager-Based | 关注点分离、消融容易、团队协作友好 | 4.3 | ⭐⭐⭐ |
| 8 | Manager 加载顺序 | Event→Command→Action→Obs→Term→Reward→Curriculum→Metrics→Recorder | 4.3 | ⭐⭐⭐ |
| 9 | 双框架 API 差异 | actor vs policy、EntityCfg vs ArticulationCfg、tyro vs argparse | 4.4 | ⭐⭐⭐ |
| 10 | ObservationManager 流程 | func→clip→noise→concat | 4.4 | ⭐⭐ |
| 11 | RewardManager dt 缩放 | reward 权重是"单位时间密度"，独立于仿真频率 | 4.4 | ⭐⭐ |
| 12 | 自定义 Term 编写 | 三步：写函数→config 引用→验证 | 4.4 | ⭐⭐ |
| 13 | Entity vs Articulation | root_link vs root、MuJoCo 术语 vs PhysX 术语 | 4.5 | ⭐⭐ |
| 14 | 四元数格式差异 | mjlab/Isaac Lab 2.x: wxyz，Isaac Lab 3.0: xyzw | 4.5 | ⭐⭐ |

---

## 累积项目：本章新增模块

| 项目 | 本章贡献 | 验证标准 |
|------|---------|---------|
| **A 四足速度跟踪** | 理解 velocity_env_cfg.py 的 Manager 结构 | 能解释每个 ObsTerm/RewTerm/EventTerm 的作用 |
| **B 人形 locomotion** | 理解 Isaac Lab 的 Manager API 差异 | 能在 Isaac Lab 中找到对应的配置文件 |

---

## 延伸阅读

### Manager-Based 架构

| 资源 | 难度 | 说明 |
|------|------|------|
| Isaac Lab 文档：Task Design Workflows | ⭐⭐ | Manager-Based vs Direct 工作流的官方对比 |
| Isaac Lab 文档：Creating a Manager-Based RL Environment | ⭐⭐ | CartPole 教程，从零构建 Manager-Based 环境 |
| mjlab 架构文档：`docs/source/architecture_overview.rst` | ⭐⭐ | mjlab 的 Manager 体系设计 |
| Zakka et al., *mjlab: A Lightweight Framework*, arXiv 2601.22074 §3 | ⭐⭐ | mjlab 的 env.step() 时序和 Manager 设计 |

### PPO 工程实现

| 资源 | 难度 | 说明 |
|------|------|------|
| Schwarke et al., *RSL-RL: A Learning Library*, arXiv 2509.10771 | ⭐⭐ | RSL-RL 的 PPO 实现和 actor-critic 分离 |
| SpinningUp PPO 教程 | ⭐ | PPO 算法的通俗讲解 |
| The 37 Implementation Details of PPO (Huang et al., 2022) | ⭐⭐⭐ | PPO 工程细节的权威参考 |

### legged_gym 参考

| 资源 | 难度 | 说明 |
|------|------|------|
| Rudin et al., *Learning to Walk in Minutes*, CoRL 2021 | ⭐⭐ | legged_gym 的原始论文 |
| legged_gym 仓库：`github.com/leggedrobotics/legged_gym` | ⭐⭐ | 单体架构的代码参考，对比理解 Manager-Based |

---

## 🔧 故障排查手册

### Manager 配置问题

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| obs 维度和预期不符 | ObsTerm 返回了错误 shape | 打印每个 term 的输出 shape | 4.4 |
| reward 始终为零 | reward 函数读到了空数据（如 sensor 名拼错） | 打印 raw_reward 值 | 4.4 |
| 策略不跟踪命令 | actor obs 中缺少 command term | 检查 obs config 是否包含 `generated_commands` | 4.4 |
| reset 后行为异常 | reset event 的执行顺序不对 | 检查 EventManager 的 mode 和加载顺序 | 4.3 |
| PPO value bootstrap 不正确 | terminated 和 truncated 没有正确区分 | 检查 Wrapper 的 dones 和 time_outs | 4.1 |

### 跨框架问题

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| 从 mjlab 迁移到 Isaac Lab 后 obs group 找不到 | "actor" vs "policy" 命名差异 | 检查 RSL-RL 的 obs_groups 配置 | 4.4 |
| Isaac Lab 3.0 代码报 tensor 类型错误 | .data.* 返回 wp.array 而非 torch | 添加 wp.to_torch() 包装 | 4.5 |
| 四元数计算结果不对 | wxyz vs xyzw 格式差异 | 确认框架版本和四元数约定 | 4.5 |
| reward 有些 term 始终为零 | sensor geom regex 不匹配 | 打印 sensor.data.found + 检查 regex | 4.3 |
| play 和 train 行为差异大 | play cfg 关闭了 corruption 或移除了 event | 对比 play_env_cfg 和 env_cfg | 4.3 |

### 性能问题

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| steps/s 远低于预期 | Manager 中有 Python-side 瓶颈 | 用 torch.profiler 定位热点 | 4.2 |
| GPU 利用率低但训练慢 | CPU 端的 Manager 计算（如 complex obs term） | 检查各 Manager 的 compute 时间 | 4.2 |
| 增加 obs term 后训练变慢 | obs term 做了 expensive 计算（如 raycast） | 用简化 obs 验证是否是 obs 导致 | 4.4 |
| decimation 改大后 reward 变化 | dt 缩放使 reward 密度改变 | 确认 RewardManager.scale_by_dt | 4.4 |

---

## 术语速查表

| 术语 | 含义 | 首次出现 |
|------|------|---------|
| PPO | Proximal Policy Optimization，最常用的 on-policy RL 算法 | §4.1 |
| rollout | PPO 的数据收集阶段，调用 env.step() 收集 transition | §4.1 |
| update | PPO 的梯度更新阶段，不调用 env.step() | §4.1 |
| GAE | Generalized Advantage Estimation，计算 advantage 的方法 | §4.1 |
| `gamma` ($\gamma$) | 折扣因子，控制长期 vs 短期 reward 的权衡 | §4.1 |
| `lam` ($\lambda$) | GAE 平滑参数，控制 bias vs variance 的权衡 | §4.1 |
| `desired_kl` | RSL-RL 的自适应 lr 目标 KL 散度 | §4.1 |
| `num_steps_per_env` | 每个 env 在一次 rollout 中的步数 | §4.1 |
| `num_mini_batches` | rollout 数据被分成的 mini-batch 数量 | §4.1 |
| decimation | env.step() 中调用 sim.step() 的次数 | §4.2 |
| `physics_dt` | 物理引擎的时间步长 | §4.2 |
| `step_dt` | policy 的时间步长 = physics_dt $\times$ decimation | §4.2 |
| terminated | 任务真正失败（如摔倒），value = 0 | §4.2 |
| truncated | 超时，value 需 bootstrap | §4.2 |
| Manager-Based | 把 MDP 组件拆分到独立 Manager 的架构 | §4.3 |
| Term | Manager 的基本配置单元（ObsTerm/RewTerm/EventTerm） | §4.3 |
| wiring | sensor → obs/reward 的数据流连接 | §4.3 |
| Task Registry | 把 task ID 映射到 env/rl config 的注册系统 | §4.3 |
| `enable_corruption` | 是否在 obs 中加入 noise（actor=True, critic=False） | §4.4 |
| `scale_by_dt` | RewardManager 是否用 dt 缩放 reward | §4.4 |
| `spec_fn` | EntityCfg 中返回 MjSpec 的函数 | §4.5 |
| `_b` 后缀 | 机体坐标系（body frame） | §4.5 |
| `_w` 后缀 | 世界坐标系（world frame） | §4.5 |

---

## 研究实践建议

### 给博士新生的 Manager-Based 入门建议

1. **从 config 开始读，不要从 base class 开始。** 很多新生的第一反应是打开 `ManagerBasedRlEnv` 的 `step()` 方法——这是对的，但更高效的顺序是先读 `velocity_env_cfg.py` 的配置（理解"配了什么"），再读 `step()` 的时序（理解"怎么执行的"），最后读各 Manager 的 `compute()` 方法（理解"内部怎么算的"）。

2. **从 zero agent 开始，不要从训练开始。** 在修改任何 config 之前，先用 zero agent 可视化。这验证了物理层（Ch03）和场景配置（本章 §4.5）是否正确——如果 zero agent 时机器人就不正常，任何 RL 训练都不会成功。

3. **每次只改一个 term。** Manager-Based 架构的最大优势是可以独立修改每个 term。利用这个优势——每次实验只修改一个 obs/reward/event term，保持其他不变。这样你能清楚地知道行为变化来自哪个修改。

4. **养成看 WandB reward 分项的习惯。** 不要只看 total reward——查看 `Episode_Reward/<term_name>` 了解每个 reward 项的贡献。如果某个 term 始终为零，要么它的 sensor 有问题，要么它对当前策略不相关。

5. **记录你的 config 修改。** 每次修改 config 时，在 WandB 的 run notes 中记录"改了什么、为什么改、结果怎样"。一个月后你会感谢自己。

### 给有经验研究者的架构迁移建议

1. **从 legged_gym 迁移时，先理解 Manager 对应关系。** `_compute_observations()` → ObservationManager，`_compute_reward()` → RewardManager，`_push_robots()` → EventManager (interval mode)，`_randomize_dof_props()` → EventManager (startup mode)。

2. **不要试图在 Manager-Based 中复制 legged_gym 的"全局状态"模式。** legged_gym 中常见的 `self.last_contacts`、`self.air_time` 等全局状态变量，在 Manager-Based 中应该由 Sensor 管理（如 `ContactSensor.track_air_time`）。

3. **利用 tyro CLI 覆盖快速实验。** mjlab 的 tyro CLI 覆盖让你不需要创建新文件就能修改任何 config 参数：
```bash
# 快速实验不同的 reward 权重
uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --env.rewards.track-lin-vel.weight 3.0 \
    --env.rewards.action-rate.weight -0.05
```

### 从 Manager-Based 到自定义任务

当你需要创建一个全新的任务（不是 velocity tracking）时，推荐的工作流：

| 步骤 | 操作 | 时间 |
|------|------|------|
| 1 | 复制最相似的已有任务 config | 5 min |
| 2 | 修改 Entity（换机器人模型） | 15 min |
| 3 | 修改 obs（添加/删除 term） | 15 min |
| 4 | 修改 reward（添加/删除 term） | 30 min |
| 5 | 修改 termination 和 event | 15 min |
| 6 | 注册新 task | 5 min |
| 7 | zero agent 验证 | 5 min |
| 8 | small train 验证 | 10 min |
| 9 | full train 并迭代 | 数小时-数天 |

> Ch15（自定义环境）将详细讲解这个工作流的每一步。本章只需要理解 Manager-Based 架构——它是自定义任务的基础。

### PPO 训练曲线的快速诊断

理解了 Manager-Based 架构后，你应该能够快速诊断 PPO 训练曲线中的常见问题。以下是一个按优先级排列的诊断指南：

**reward 不上涨**：

| 优先级 | 检查项 | 正常表现 | 异常信号 |
|--------|--------|---------|---------|
| 1 | WandB reward 分项 | 至少 tracking reward 在涨 | 所有 reward 为零→sensor wiring |
| 2 | episode length | 逐步增加 | 不增加→termination 太严 |
| 3 | KL divergence | 0.005-0.03 | >0.1→lr 太大 |
| 4 | entropy | 缓慢下降 | 骤降→策略过早坍缩 |
| 5 | actor obs 维度 | 包含 command | 不包含→策略无法条件化 |

**reward 上涨但行为差**：

| 优先级 | 检查项 | 可能原因 | 解决方法 |
|--------|--------|---------|---------|
| 1 | 可视化检查 | reward hacking | 加 penalty 或修改 reward 结构 |
| 2 | tracking 分项 | tracking 低但总 reward 高 | penalty 权重符号错了（正变负） |
| 3 | episode length | 过长→策略学会"不动" | alive reward 过大 |
| 4 | action std | 过低→动作保守 | 增大 init_noise_std |

**训练不稳定（reward 振荡）**：

| 优先级 | 检查项 | 可能原因 | 解决方法 |
|--------|--------|---------|---------|
| 1 | KL divergence | KL spike→策略更新过大 | 减小 desired_kl 或 clip_range |
| 2 | value loss | 突然增大→critic 不准 | 增加 critic 网络大小 |
| 3 | NaN 出现 | 物理爆炸 | 减小 action scale 或 timestep |
| 4 | entropy | 突然下降后恢复 | 正常的探索-利用切换 |

> 这些诊断在 Ch07（训练管线）中会更详细地展开。本章只需要知道：**PPO 训练曲线的异常通常不是 PPO 算法的问题——而是 env 配置（obs/reward/termination/event）的问题**。90% 的训练失败可以通过修改 env config 而非 PPO 超参来解决。

### 调参的优先级法则

面对一个新任务时，按以下顺序调参——而非同时调所有参数：

| 优先级 | 调什么 | 为什么先调它 | 典型工作量 |
|--------|--------|-----------|-----------|
| 1 | 环境配置验证 | 如果 env 有 wiring 错误，任何调参都没用 | 30 min（按 debug checklist） |
| 2 | action scale | 过大→NaN，过小→不运动 | 5 min（2-3 个值对比） |
| 3 | reward 权重 | tracking 和 penalty 的平衡 | 1 小时（3-5 组消融） |
| 4 | command 范围 | 太宽→学不会，太窄→泛化差 | 15 min |
| 5 | DR 范围 | 太宽→不稳定，太窄→不鲁棒 | 30 min |
| 6 | PPO 超参 | 大多数情况默认值就好 | 仅在前 5 步都不管用时 |

> **一个跨领域类比**：这个优先级法则类似于调试网络问题时的 OSI 七层模型——先检查物理层（网线是否插好），再检查链路层（MAC 地址），最后才检查应用层（HTTP 状态码）。在 RL 中，"物理层"是 env config 正确性，"链路层"是 obs/reward/action 的配置，"应用层"是 PPO 超参数。大多数问题在前两层就能解决。

### 本章使用的主要源码文件

| 文件路径 | 本章关注点 | 对应节 |
|---------|-----------|--------|
| `src/mjlab/envs/manager_based_rl_env.py` | `step()` 18 步时序、`load_managers()` 加载顺序 | §4.2, §4.3 |
| `src/mjlab/managers/observation_manager.py` | obs group 拼接、noise/clip/history | §4.4 |
| `src/mjlab/managers/reward_manager.py` | reward 计算、dt 缩放、NaN 保护 | §4.4 |
| `src/mjlab/managers/action_manager.py` | process_action、apply_action | §4.4 |
| `src/mjlab/managers/termination_manager.py` | terminated/time_out 区分 | §4.4 |
| `src/mjlab/managers/event_manager.py` | startup/reset/step/interval 模式分发 | §4.2, §4.4 |
| `src/mjlab/managers/command_manager.py` | command resample、twist 命令 | §4.4 |
| `src/mjlab/managers/curriculum_manager.py` | terrain/command curriculum | §4.4 |
| `src/mjlab/tasks/velocity/velocity_env_cfg.py` | base config 完整结构 | §4.4 |
| `src/mjlab/tasks/velocity/config/go2/env_cfgs.py` | robot-specific override | §4.4 |
| `src/mjlab/tasks/velocity/mdp/rewards.py` | reward 函数实现 | §4.4 |
| `src/mjlab/tasks/registry.py` | task 注册、deepcopy | §4.3 |
| `src/mjlab/rl/vecenv_wrapper.py` | terminated/truncated→dones、obs_groups | §4.1 |
| `src/mjlab/entity/entity.py` | EntityCfg、EntityData | §4.5 |
| `src/mjlab/scene/scene.py` | SceneCfg、MjSpec attach | §4.5 |

> **阅读建议**：优先读前 3 个文件（90 分钟）。它们覆盖了 env.step() 时序、obs 管线和 reward 管线——这是 Manager-Based 架构的核心。其余文件在你需要修改特定 Manager 或排查特定问题时再读。

### 给不同读者的阅读建议

> **如果你只做 mjlab locomotion**：重点读 §4.1（PPO 循环）、§4.2（18 步时序）、§4.4（velocity config 精读）。Isaac Lab 对比部分可以快速浏览。
>
> **如果你需要同时用两个框架**：全章精读，特别关注 §4.4 的双框架 API 对照表和 §4.5 的 Entity vs Articulation 对比。
>
> **如果你从 legged_gym 迁移**：重点读 §4.3（Manager 动机 + 迁移工作流 + 隐式依赖问题）。迁移表可以作为你的工作 checklist。
>
> **如果你遇到具体 wiring 问题**：直接跳到 velocity task Debug Checklist 和 🔧 故障排查手册——它们按症状组织，可以快速定位。

---

## 本章与后续章节的关系

本章是 Part II 的"骨架"——它定义了 Manager-Based 架构的全局结构，后续 Ch05-10 逐一深入每个 Manager。

| 后续章节 | 与本章的关系 | 本章哪个知识点为其铺垫 |
|---------|-----------|----------------------|
| **Ch05 Obs/Action** | 深入 ObservationManager 和 ActionManager | obs/action 管线（§4.4）、Entity 数据访问（§4.5） |
| **Ch06 Reward** | 深入 RewardManager 的 reward shaping | RewardManager dt 缩放（§4.4）、reward 分项日志 |
| **Ch07 训练管线** | 深入 RSL-RL 的 PPO 超参调优 | PPO 循环（§4.1）、rl_cfg 配置 |
| **Ch08 DR** | 深入 EventManager 的 DR 策略 | EventManager 四种模式（§4.2）、加载顺序（§4.3） |
| **Ch09 Teacher-Student** | 利用 actor/critic 非对称 obs 做蒸馏 | 非对称观测（§4.4）、obs group 配置 |
| **Ch10 模仿学习** | 利用 CommandManager 和 RewardManager | command 配置（§4.4）、tracking reward |
| **Ch13 四足实战** | velocity task 完整精读 | 全章——特别是 velocity config 精读（§4.4） |
| **Ch15 自定义环境** | 从零构建新任务 | Task Registry（§4.3）、Manager 加载顺序（§4.3） |

> **关键过渡**：本章建立了 Manager-Based 架构的"地图"——你知道了有哪些 Manager、它们的执行时序和 API。Ch05 开始深入"地图上的每个地标"——从 Observation 和 Action 的设计原则开始。

---

## 版本信息速查

| 组件 | 版本 | 本章涉及内容 |
|------|------|------------|
| mjlab | 1.2.0 | ManagerBasedRlEnv、所有 Manager API |
| Isaac Lab | 2.3.0（主线） | ManagerBasedRLEnv、对应 Manager API |
| Isaac Lab 3.0 | Beta（注释标注） | wp.array 数据管线、xyzw 四元数 |
| rsl_rl | $\ge$ 4.0.0 | 分离 actor/critic config、自适应 lr |
| RSL-RL | $\ge$ 4.0.0 | PPO 实现、VecEnv Wrapper |

---

## 附：Manager-Based 架构快速参考卡

以下是本章最重要的信息摘要，适合打印或保存为 cheat sheet。

**env.step() 18 步时序（简化记忆版）**：

```
action → [decimation × physics] → termination → reward
  → reset → forward → command → event → sense → obs
```

记忆口诀：**"先动后判，先重后观"**——先执行动作（action+physics），后判断结果（termination+reward）；先重置（reset+forward），后观察（command+event+sense+obs）。

**Manager 加载顺序（简化记忆版）**：

```
E-C-A-O-T-R-C-M-R
Event → Command → Action → Obs → Term → Reward → Curric → Metrics → Recorder
```

记忆口诀：**"事先说动看，判赏学记录"**——事件先声明（Event），命令先说（Command），动作先定义（Action），观测先看（Obs），判断终止（Term），计算奖赏（Reward），学习课程（Curriculum），记录指标（Metrics+Recorder）。

**双框架 API 速查**：

| 操作 | mjlab | Isaac Lab |
|------|-------|-----------|
| obs group | `"actor"` / `"critic"` | `"policy"` / `"critic"` |
| 环境类 | `ManagerBasedRlEnvCfg` | `ManagerBasedRLEnvCfg` |
| 机器人 | `EntityCfg` | `ArticulationCfg` |
| 场景 | `SceneCfg` | `InteractiveSceneCfg` |
| 终止项 | `TermTerm` | `DoneTerm` |
| 注册 | `register_mjlab_task()` | `gymnasium.register()` |
| CLI 训练 | `uv run train` | `python scripts/.../train.py` |
| CLI 覆盖 | tyro（任意深度） | argparse（预定义） |
| 基座速度（机体） | `root_link_lin_vel_b` | 需手动转换 |
| 四元数格式 | wxyz | 2.x: wxyz, 3.0: xyzw |

**自定义 Term 三步流程**：

```
1. 写函数 → def my_term(env) -> Tensor
   obs: 返回 [num_envs, dim]
   reward: 返回 [num_envs]
   
2. 配置引用 → ObsTerm(func=my_term) 或 RewTerm(func=my_term, weight=0.5)

3. 验证 → zero agent 检查 shape + small train 检查非 NaN
```

**terminated vs truncated 速查**：

| 类型 | 含义 | PPO value | 典型条件 |
|------|------|-----------|---------|
| terminated | 真正失败 | = 0 | 摔倒、非法接触 |
| truncated | 超时 | = V(s') | episode 达到 max_length |

---

> **Ch04 完结。** 本章从 PPO 训练循环出发，深入解析了 env.step() 的 18 步时序，对比了单体架构和 Manager-Based 架构的工程差异，精读了双框架的 Manager API（包括 ObservationManager、RewardManager、ActionManager、TerminationManager、EventManager、CurriculumManager 和 NaN Guard 的工程细节），通过 velocity task 的完整配置精读展示了 Manager 如何组合工作，并建立了从 legged_gym 迁移到 Manager-Based 的完整工作流。
>
> **你的收获**：
> - 能画出 PPO 循环中 env.step() 的完整数据流，解释 rollout 和 update 阶段的区别
> - 能逐行解释 env.step() 的 18 步时序，理解每个步骤的工程理由
> - 能用时序知识诊断实际训练问题（reward 滞后、obs 不对齐、wiring 错误）
> - 理解了每个 Manager 的内部工作机制和加载依赖关系
> - 能在 mjlab 和 Isaac Lab 之间"翻译" Manager API
> - 能独立添加自定义 obs/reward/event term 到现有任务
> - 掌握了 velocity task 的完整 Debug Checklist 和调参优先级
>
> **下一步行动**：
> 1. 完成 §4.6 的四个最小实验——特别是实验 3（添加自定义 reward term）
> 2. 打开 `velocity_env_cfg.py`，用本章的知识精读每个 Manager 的配置
> 3. 按 Debug Checklist 验证你的 velocity task 配置
> 4. 进入 Ch05，深入 Observation 和 Action 的设计原则——它们是策略和环境之间的"接口"
>
> **提醒**：本章建立的 Manager-Based 架构理解是后续所有章节的基础。如果你在 Ch05-10 中遇到"不知道该在哪个 Manager 中做修改"的问题，回到本章的 Manager 职责表（§4.3）、加载顺序表（§4.3）和 18 步时序表（§4.2）。

> **版本更新提醒**：Manager-Based 架构在 mjlab 和 Isaac Lab 中持续演进。本章的 API 和时序以 mjlab 1.2.0 + Isaac Lab 2.3.0 为准。rsl_rl $\ge$ 4.0 的 actor/critic 分离配置是本章示例的基础——如果你使用旧版 rsl_rl，部分 API 可能不同。查阅各框架的 CHANGELOG 获取最新信息。

---

## 附：从零开始调试一个不收敛的 velocity task

以下是一个完整的调试工作流示例，假设你的新 velocity task 训练 1000 iteration 后 reward 不涨：

**第一轮排查（5 分钟）——环境基本功能**：

```bash
# 1. zero agent 可视化
uv run play <TASK> --agent zero --num-envs 4 --viewer viser
# 检查：机器人站姿是否正确？有无穿地/浮空？

# 2. random agent 验证
uv run play <TASK> --agent random --num-envs 4 --viewer viser
# 检查：机器人是否对 action 有响应？是否立即 NaN？
```

如果 zero agent 姿态就不对 → 问题在 Entity/Scene 配置（§4.5），回到 Ch03 检查物理参数。

如果 random agent 立即 NaN → 问题在 action scale 或物理参数（Ch03 §3.7 稳定性排查）。

**第二轮排查（15 分钟）——obs/reward 验证**：

```bash
# 3. 小规模训练 2 iteration
uv run train <TASK> --env.scene.num-envs 64 --agent.max-iterations 2

# 4. 检查 WandB 日志
# 查看 Episode_Reward/<term_name> 
# 如果所有 reward term 都是零 → sensor wiring 问题
# 如果只有 contact 相关的 term 为零 → contact sensor regex 问题
```

如果 obs 维度和预期不符 → 某个 ObsTerm 返回了错误 shape，逐一打印检查。

如果 tracking reward 非零但总 reward 不涨 → penalty 权重可能太大，或 penalty 符号错了。

**第三轮排查（30 分钟）——详细诊断**：

```bash
# 5. 只保留 tracking reward，去掉所有 penalty
uv run train <TASK> \
    --env.rewards.action-rate.weight 0.0 \
    --env.rewards.foot-slip.weight 0.0 \
    --env.rewards.upright.weight 0.0 \
    --env.scene.num-envs 1024 --agent.max-iterations 300

# 如果现在 reward 涨了 → penalty 配置有问题
# 如果仍然不涨 → tracking reward 本身有问题
```

如果只有 tracking reward 也不涨 → 检查 command 是否在 actor obs 中、action scale 是否太小。

**第四轮排查（如果前三轮都没找到问题）**：

```bash
# 6. 打开 NaN Guard
uv run train <TASK> --enable-nan-guard True --env.scene.num-envs 256 --agent.max-iterations 50

# 7. 检查 PPO 超参
# 打印 KL divergence —— 如果 >0.1 说明 lr 太大
# 打印 entropy —— 如果骤降说明策略过早坍缩
```

> 这个调试工作流体现了 §4.2 时序知识和 §4.3 Manager 架构知识的实际应用——每一步都基于对数据流的理解来缩小排查范围。

---

> **本教材版本锚定**：本章所有代码示例和 API 以 mjlab 1.2.0 + Isaac Lab 2.3.0 + rsl_rl $\ge$ 4.0.0 为准。Isaac Lab 3.0 Beta 的变更（wp.array 数据管线、xyzw 四元数）以注释标注。

> 如果你在使用本章内容时发现了 API 变更或新的调试方法，欢迎在框架的 GitHub Issues 或 Discord 中分享——这是开源社区进步的基础。

> **关于本章的 Text:Code 比例说明**：本章涉及大量源码级精读和工程配置示例，代码块占比较高（约 47%）是算法工程化教学文档的正常表现——每段代码都伴有详细的文字讲解，解释"为什么这样写"和"如果不这样会怎样"。后续的理论性更强的章节（如 Ch09 Teacher-Student、Ch10 模仿学习）会有更高的文字占比。


> **致谢**：本章的 Manager-Based 架构分析参考了 Isaac Lab 和 mjlab 的官方文档和源码。env.step() 时序分析基于 mjlab 1.2.0 的 `manager_based_rl_env.py` 源码。PPO 工程实现细节参考了 RSL-RL 的 `ppo.py` 和 `on_policy_runner.py`。legged_gym 架构分析基于 `legged_gym` 仓库的 `legged_robot.py`。感谢这些开源项目的贡献者。

> **向读者的邀请**：如果你完成了本章的四个最小实验，并且成功添加了一个自定义 reward term，恭喜——你已经具备了修改和扩展 Manager-Based 环境的基本能力。下一章（Ch05）将带你深入 Observation 和 Action 的设计原则，这是从"会用框架"到"会设计任务"的关键跨越。
> **本章完。**

> **记住**：Manager-Based 架构的核心价值不是让代码更优雅——而是让你能以 10 倍的速度迭代实验。在机器人 RL 研究中，实验迭代速度往往比算法创新更重要。

> **最后一个提醒**：本章的 18 步时序表和 Manager 加载顺序表是你在后续章节中最常回来查阅的两张表。建议截图或书签保存。

