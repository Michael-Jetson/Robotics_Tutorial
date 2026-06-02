# 第 24 章 大规模训练、性能优化与 NaN 排查

---

## 前置自测

📋 **答不出 ≥ 2 题 → 先回对应章节复习**

1. **[Ch03 MuJoCo Warp]** CUDA Graph 的 capture 机制对数组地址有什么要求？如果 domain randomization 扩展了 per-world 字段，需要做什么操作？
2. **[Ch07 训练管线]** PPO 的 `num_steps_per_env` 和 `num_mini_batches` 分别控制 rollout 的什么维度？它们对 GPU 显存有什么影响？
3. **[Ch22 DIY]** 一个自定义环境的 smoke test 通过但 500 iterations 训练后出现 NaN。按 Ch22 §22.7 的排查优先级，你应该先检查什么？
4. **[系统]** `CUDA_VISIBLE_DEVICES=2,3` 后，PyTorch 的 `cuda:0` 对应物理 GPU 几号？这个映射为什么容易出错？
5. **[分布式]** 数据并行（Data Parallel）和环境并行（Environment Parallel）在多 GPU 场景下的区别是什么？哪种更适合 GPU 仿真 RL？

## 本章目标

学完本章后，你应该能够：

1. **配置**双框架的多 GPU 训练——mjlab 的 `--gpu-ids` + torchrunx 和 Isaac Lab 的 torchrun
2. **诊断**训练中 NaN 的根因——使用 `--enable-nan-guard`、`viz-nan` 和系统化的排查优先级表
3. **量化**环境的性能瓶颈——区分 physics cost、sensor cost、manager cost，用 profiler 定位热点
4. **理解** AGILE 的四阶段工业级 workflow——Prepare → Train → Evaluate → Deploy 及其算法工具箱
5. **设计**公平的大规模实验——控制总样本量、管理 checkpoint/resume、组织可复查的运行包

---

## 24.1 为什么需要大规模训练 ⭐

> **这一节解决什么问题**：从"能训练"到"高效训练"的工程跨越需要关注什么？

### 动机：单 GPU 的天花板

Ch13-Ch23 的所有实战都在单 GPU 上完成——4096 个并行环境、10000-20000 iterations、通常 1-4 小时 wall-clock。对于四足速度跟踪这类任务，单 GPU 足够。但当你面对以下场景时，单 GPU 成为瓶颈：

- **高 DOF 机器人**：29-DOF 人形的状态空间比 12-DOF 四足大 10 倍，收敛需要更多数据
- **复杂任务**：loco-manipulation 需要同时学习移动和操作，探索空间指数增长
- **大规模 DR**：宽范围的 domain randomization 意味着策略需要在更大的参数空间上泛化
- **超参搜索**：找到最优的 reward 权重组合需要并行跑多个实验
- **多 seed 评估**：发论文需要至少 3-5 个 seed 的统计显著性

**单 GPU 的吞吐天花板**：在单块 A100 上，4096 个并行环境的典型吞吐量是 150k-250k env-steps/s（12-DOF 四足）或 80k-120k env-steps/s（29-DOF 人形）。超过 4096 envs 后吞吐提升趋缓（GPU 计算资源饱和），增加 envs 主要增加显存占用而非速度。

**何时需要多 GPU？** 不是"越多越好"，而是有明确的决策标准：

| 信号 | 建议 |
|------|------|
| 单 GPU 4096 envs 吞吐已饱和（增加 envs 不增速） | 多 GPU 环境并行 |
| 需要 >10k envs 做大 DR 范围覆盖 | 多 GPU 环境并行 |
| 需要同时跑 5+ seed | 多 GPU 各跑一个 seed |
| 需要搜索 10+ 超参组合 | 多 GPU + WandB Sweep |
| 单 GPU 显存不足（>40 GB VRAM） | 多 GPU 或减小模型 |
| 以上都不是 | 不需要多 GPU |

> **本质洞察**：多 GPU 训练不是"加速按钮"——它是一个工程复杂度的放大器。单 GPU 上的 bug 在多 GPU 上更难排查（因为需要区分是哪个 rank 的问题）、checkpoint 更难管理（多个 rank 写同一个文件）、实验结果更难比较（样本量核算更复杂）。只有在单 GPU 确实不够用时才引入多 GPU。

> **跨领域类比**：多 GPU 训练就像从单厨师厨房升级到多厨师厨房。如果只有一道菜要做，多一个厨师反而碍手碍脚（沟通成本）。但如果要同时准备 5 道不同的菜（多 seed），或者一道菜需要两个人同时操作不同部分（环境并行），多厨师就有价值了。关键是明确"为什么需要多人"，而不是"人多力量大"。

### ⚠️ 常见陷阱

⚠️ **思维陷阱：认为"多 GPU 训练更好"**
- 多 GPU 的总样本量更大（更多 envs），但每个 update 的 PPO batch 组成可能不同（不同 GPU 上的 env 状态分布不同）
- 如果不正确核算样本量，可能得出"多 GPU 收敛更快"的错误结论——实际上只是"多 GPU 看了更多数据"
- 正确做法：比较时使用相同的总样本量（total env steps），而非相同的 iteration 数

⚠️ **编程陷阱：`CUDA_VISIBLE_DEVICES` 映射混淆**
- 设置 `CUDA_VISIBLE_DEVICES=2,3` 后，PyTorch 的 `cuda:0` 对应物理 GPU 2，`cuda:1` 对应物理 GPU 3
- 如果在代码中硬编码 `--gpu-ids "[2, 3]"`，程序看不到逻辑 id 2 和 3——因为可见设备只有 0 和 1
- 正确做法：传 `--gpu-ids "[0, 1]"`，让框架自动映射到 `CUDA_VISIBLE_DEVICES` 指定的物理设备

### 练习

1. **[计算题]** 单 GPU A100 上，4096 envs 的四足任务吞吐量约 200k steps/s。计算训练 10000 iterations（每 iteration 采 24 steps/env）需要多少 wall-clock 时间。
2. **[设计题]** 你需要为一篇论文跑 5 个 seed × 3 个 reward 权重配置 = 15 个实验。单个实验需要 2 小时。设计一个最优的 GPU 分配方案（假设有 4 块 GPU）。
3. **[思考题]** 为什么过了 4096 envs 后继续增加 envs 不再线性提升吞吐？从 GPU 计算与访存的比例角度解释。

---

有了"何时需要大规模训练"的判断标准，下一步是学习具体的多 GPU 配置方法——双框架各有不同的实现路径。

---


## 24.2 多 GPU 训练：双框架配置详解 ⭐⭐⭐

> **这一节解决什么问题**：在 mjlab 和 Isaac Lab 中分别如何配置多 GPU 训练？数据并行和环境并行有什么区别？

### 数据并行 vs 环境并行 ⭐⭐

GPU 仿真 RL 的多 GPU 有两种范式：

| 维度 | 数据并行 (Data Parallel) | 环境并行 (Environment Parallel) |
|------|------------------------|-------------------------------|
| 核心思想 | 每个 GPU 跑相同数量的 envs，梯度在 GPU 间同步 | 每个 GPU 跑独立的 envs，样本量翻倍 |
| 实现 | PyTorch DDP (all-reduce gradients) | 每个 GPU 独立 rollout，汇总后 PPO update |
| 总 envs 数 | 不变（4096 = 4 GPU × 1024） | 翻倍（4096 × N_GPU） |
| PPO batch 大小 | 不变 | 翻倍 |
| 通信开销 | 每次 update 一次 all-reduce | 每次 update 一次 gather + all-reduce |
| 适用场景 | 显存不够放 4096 envs | 需要更多 envs 做 DR 覆盖 |
| 推荐度 | ⭐⭐（除非显存不足否则不优先） | ⭐⭐⭐（GPU 仿真的主要多 GPU 模式） |

**关键差异**：在 GPU 仿真 RL 中，env step 通常是计算瓶颈（占总时间 60-80%），而非 PPO update。因此**环境并行**（更多 envs → 更多数据 → 更快收敛）比数据并行（相同数据量 → 梯度同步 → 不增加收敛速度）更有效。

> **双重解读**：多 GPU 训练可以从两个完全不同的角度理解。**从统计学的角度**，更多 GPU = 更多并行 envs = 每次 PPO update 的 batch size 更大。大 batch 降低了梯度估计的方差（更多样本 → 更准确的梯度），但也可能降低泛化能力（过度拟合于大 batch 的统计特性）。这就是为什么 AGILE 的默认 4096 envs 是一个经过验证的平衡点——增加到 8192+ 的收益递减。**从系统工程的角度**，更多 GPU = 更高的通信开销（DDP all-reduce）+ 更复杂的日志管理 + 更难排查的 bug（rank 间异步性）。每多一块 GPU，工程复杂度增加的不是线性而是超线性。这两个视角的交汇点在于：多 GPU 的最优配置不是"尽可能多"，而是"刚好够用"——从统计角度确定需要多少 envs，从工程角度确定能稳定管理多少 GPU。

mjlab 和 Isaac Lab 都默认使用**环境并行 + DDP 梯度同步**的混合模式——每个 GPU 运行独立的环境集合（环境并行），PPO update 时梯度通过 DDP all-reduce 同步（数据并行）。

### mjlab 多 GPU 配置 ⭐⭐⭐

mjlab 使用 **torchrunx** 实现多 GPU 训练。torchrunx 是一个基于 SSH 的纯 Python 分布式启动器——比标准 torchrun 更灵活（支持从单个 Python 脚本启动多节点训练，无需 SLURM）。

```bash
# === mjlab 单 GPU 训练（基准） ===
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 10000 \
    --gpu-ids "[0]"

# === mjlab 双 GPU 训练 ===
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 10000 \
    --gpu-ids "[0, 1]"
# 效果：每个 GPU 各跑 4096 envs，总共 8192 envs
# 每次 PPO update 的样本量翻倍

# === mjlab 四 GPU 训练 ===
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 5000 \
    --gpu-ids "[0, 1, 2, 3]"
# 注意：iteration 数减半——因为每次 update 样本量 ×4
# 总样本量 = 4096 × 24 × 4 × 5000 = 4096 × 24 × 1 × 20000（等价）
```

**torchrunx 的工作原理**：

```
启动进程 (rank 0 on GPU 0)
├── 创建 mjlab 环境 (4096 envs on GPU 0)
├── 创建 PPO runner (actor+critic on GPU 0)
└── DDP wrapper (gradient all-reduce)

启动进程 (rank 1 on GPU 1)
├── 创建 mjlab 环境 (4096 envs on GPU 1)
├── 创建 PPO runner (actor+critic on GPU 1)
└── DDP wrapper (gradient all-reduce)

每个 iteration:
  1. 每个 rank 独立 rollout 24 steps
  2. 每个 rank 计算 local gradients
  3. DDP all-reduce: 平均梯度 across ranks
  4. 每个 rank 用平均梯度更新策略
  → 所有 rank 的策略权重保持同步
```

**多 GPU 训练的样本量核算** ⭐⭐⭐：

```python
# === 样本量核算公式 ===
samples_per_update = num_envs_per_gpu * num_steps_per_env * num_gpus
total_env_steps = samples_per_update * max_iterations

# 单 GPU 基准
single_gpu = 4096 * 24 * 1 * 10000  # = 983,040,000

# 双 GPU（相同 iterations）
dual_gpu_same_iter = 4096 * 24 * 2 * 10000  # = 1,966,080,000 (2× 数据！)

# 双 GPU（公平比较：缩放 iterations 使总样本量相同）
dual_gpu_fair = 4096 * 24 * 2 * 5000  # = 983,040,000 (相同！)
```

> **反事实推理：如果用双 GPU 跑相同的 iterations 而不缩放会怎样？** 你会看到双 GPU 的 reward 曲线更好——但这不是因为"双 GPU 训练效果更好"，而是因为双 GPU 总共看了 2 倍的数据。这就像两个学生做不同数量的练习题后比考试成绩——做更多题的当然分更高，但这不能证明两人的学习能力有差异。在论文中报告多 GPU 结果时，必须明确标注 x 轴是 total env steps 而非 iterations。

### Isaac Lab 多 GPU 配置 ⭐⭐⭐

Isaac Lab 使用标准的 PyTorch **torchrun** 启动分布式训练：

```bash
# === Isaac Lab 单 GPU 训练 ===
python -m isaaclab.app \
    --task Unitree-G1-29dof-Velocity-v0 \
    --num_envs 4096 \
    --headless

# === Isaac Lab 双 GPU 训练 ===
torchrun --nproc_per_node=2 \
    -m isaaclab.app \
    --task Unitree-G1-29dof-Velocity-v0 \
    --num_envs 4096 \
    --headless

# === Isaac Lab 多节点训练 ===
# 节点 0:
torchrun --nnodes=2 --nproc_per_node=4 \
    --node_rank=0 --master_addr=10.0.0.1 --master_port=29500 \
    -m isaaclab.app --task ... --headless
# 节点 1:
torchrun --nnodes=2 --nproc_per_node=4 \
    --node_rank=1 --master_addr=10.0.0.1 --master_port=29500 \
    -m isaaclab.app --task ... --headless
```

**NVIDIA OSMO** 是 Isaac Lab 的生产级多节点编排器——支持 AWS/GCP/Azure/Alibaba Cloud + on-prem K8s。对于大规模集群训练（>8 GPU），OSMO 比手动 torchrun 更可靠。

### 双框架多 GPU 对比

| 维度 | mjlab | Isaac Lab |
|------|-------|-----------|
| 启动器 | torchrunx (`--gpu-ids`) | torchrun (`--nproc_per_node`) |
| 配置方式 | CLI 参数 | CLI 参数 |
| 通信后端 | NCCL (via PyTorch DDP) | NCCL (via PyTorch DDP) |
| 多节点 | torchrunx SSH | torchrun + OSMO |
| 日志隔离 | 每个 rank 独立日志目录 | 每个 rank 独立日志目录 |
| Checkpoint | rank 0 写入 | rank 0 写入 |
| 兼容 RL 库 | RSL-RL | RSL-RL / rl_games / SKRL |

### 单写者原则 ⭐⭐

多 GPU 训练中一个高频 bug 是**多个 rank 同时写同一个文件**——导致 checkpoint 损坏、日志混乱或视频帧交错。

**核心原则：所有文件写操作只在 rank 0 执行。**

```python
# === 单写者原则的实现 ===
import torch.distributed as dist

def is_main_process():
    """判断是否是 rank 0（主进程）。"""
    if not dist.is_initialized():
        return True
    return dist.get_rank() == 0

# Checkpoint 保存
if is_main_process():
    torch.save(policy.state_dict(), "model.pt")

# WandB 日志
if is_main_process():
    wandb.log({"reward": reward_mean}, step=iteration)

# 视频录制
if is_main_process():
    recorder.record_frame(env.render())

# TensorBoard
if is_main_process():
    writer.add_scalar("reward", reward_mean, iteration)
```

如果忘记了 rank 检查——比如每个 rank 都往同一个 TensorBoard 目录写 events 文件——TensorBoard 会显示混乱的多条曲线（每个 rank 一条，x 轴重叠），你可能误以为训练不稳定（因为看到了多条振荡的曲线），实际上只是日志重复。

### Checkpoint 与 Resume 链路 ⭐⭐

多 GPU 训练的 checkpoint 管理比单 GPU 复杂——需要处理"从哪个 rank 加载"和"resume 后 rank 数量变化"的问题：

```python
# === Checkpoint 保存（只 rank 0） ===
def save_checkpoint(runner, path, iteration):
    if is_main_process():
        state = {
            "iteration": iteration,
            "policy_state_dict": runner.policy.state_dict(),
            "optimizer_state_dict": runner.optimizer.state_dict(),
            "obs_normalizer": runner.obs_normalizer.state_dict(),
            "world_size": dist.get_world_size() if dist.is_initialized() else 1,
            "num_envs_per_gpu": runner.env.num_envs,
        }
        torch.save(state, path)
        print(f"Checkpoint saved: {path}")

# === Checkpoint 恢复（所有 rank 都加载同一个文件） ===
def load_checkpoint(runner, path):
    state = torch.load(path, map_location=runner.device)
    runner.policy.load_state_dict(state["policy_state_dict"])
    runner.optimizer.load_state_dict(state["optimizer_state_dict"])
    if "obs_normalizer" in state:
        runner.obs_normalizer.load_state_dict(state["obs_normalizer"])

    # 检查 world_size 变化
    saved_ws = state.get("world_size", 1)
    current_ws = dist.get_world_size() if dist.is_initialized() else 1
    if saved_ws != current_ws:
        print(f"⚠️ World size changed: {saved_ws} → {current_ws}")
        print(f"   总样本量核算需要调整 max_iterations!")
    return state["iteration"]
```

**Resume 时的样本量一致性**：如果你用 2 GPU 训练了 5000 iterations 后中断，用 4 GPU resume，每个 iteration 的样本量变成了原来的 2 倍——如果不调整 `max_iterations`，总样本量会超出预期。resume 前必须重新核算。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：多个 rank 都写 WandB**
- 每个 rank 独立创建 WandB run，结果出现 N 个重复的 run
- 正确做法：只在 rank 0 初始化 WandB

⚠️ **编程陷阱：GPU id 映射错误**
- `CUDA_VISIBLE_DEVICES=2,3` + `--gpu-ids "[2, 3]"` → 设备不可见错误
- 正确做法：`--gpu-ids "[0, 1]"`（逻辑 id 相对于 CUDA_VISIBLE_DEVICES）

⚠️ **思维陷阱：多 GPU 但不缩放 iterations**
- 双 GPU × 10000 iterations vs 单 GPU × 10000 iterations：前者看了 2× 数据
- 论文中报告时必须标明 x 轴是 total env steps 还是 iterations

⚠️ **编程陷阱：torchrunx rank 1 崩溃但 rank 0 日志正常**
- torchrunx 的每个 rank 有独立的日志文件（在 `torchrunx/` 目录下）
- 只看 rank 0 的标准输出可能漏掉 rank 1 的 CUDA crash
- 正确做法：检查所有 rank 的日志，特别是 stderr

### 练习

1. **[计算题]** 4 GPU 训练，每 GPU 4096 envs，`num_steps_per_env=24`，`max_iterations=5000`。计算总 env steps。如果要和单 GPU × 4096 envs × 24 steps × 20000 iterations 公平比较，应该怎么设置？
2. **[实践题]** 在一台双 GPU 机器上，分别用 mjlab 的 `--gpu-ids "[0]"` 和 `--gpu-ids "[0, 1]"` 跑 100 iterations，记录两者的 wall-clock 时间和 total env steps。双 GPU 的加速比是多少？
3. **[分析题]** 为什么 GPU 仿真 RL 偏好环境并行而非纯数据并行？从 "env step 是瓶颈" 的角度解释。

---

多 GPU 解决了"训练规模"的问题。但在训练过程中，一个更紧迫的问题经常出现——训练突然崩溃，reward 变成 NaN。这种问题在多 GPU 环境中更难排查，因此我们先在单 GPU 上建立 NaN 排查方法论。

---


## 24.3 NaN 排查：从症状到根因的系统方法论 ⭐⭐⭐

> **这一节解决什么问题**：训练中突然出现 NaN 时，如何系统化地定位根因并修复？

### 动机：NaN 是 RL 训练中最高频的致命错误

NaN（Not a Number）是 RL 训练中最常见的崩溃原因——策略输出的动作、reward 计算的结果或 obs 中的某个维度突然变成 NaN，从此训练不可恢复。在 GPU 并行仿真中，NaN 尤其危险：一个 env 中的 NaN 可能通过 batch 操作传播到所有 env，在一帧之内感染整个训练。

> **跨领域类比**：NaN 在 RL 训练中就像心脏骤停在临床上——它不是疾病本身，而是某种底层病因的终端表现。你不能只治疗心脏骤停（清除 NaN）——你必须找到并治疗底层病因（导致 NaN 的物理/算法配置）。就像急诊流程先 CPR（恢复心跳）再找病因，NaN 排查也是先定位哪个 env/step/variable 最先出现 NaN（等价于 CPR），再追溯根因。

### NaN 的五大根因（按频率排序） ⭐⭐⭐

根据 mjlab、Isaac Lab 社区和本教材作者的经验，NaN 的根因按频率排序如下：

| 排名 | 根因 | 典型症状 | 频率 |
|------|------|---------|------|
| 1 | 接触求解器发散 | 高冲击接触后 qvel 中出现 NaN | 30% |
| 2 | Reward 函数中的除零/溢出 | reward 计算时出现 inf → NaN | 25% |
| 3 | Obs normalizer 未预热 | 前几步 obs 方差为 0，归一化除零 | 20% |
| 4 | Policy std 变负 | log_prob 计算时出现 NaN | 15% |
| 5 | CUDA Graph capture 失效 | 随机化改变内存布局后 graph 无效 | 10% |

### 根因 1：接触求解器发散 ⭐⭐

**症状**：训练前 1000 iterations 正常，然后突然某些 env 的 `qvel` 出现 NaN。通常发生在策略学到了高速动作（接触力增大）或 DR 引入了低摩擦（接触更容易滑动）之后。

**根因**：MuJoCo 的接触求解器在处理高 condim（如 `condim=6`，带扭转摩擦）+ 高摩擦值（如 `friction=5.0`）的接触时，可能产生数值不稳定的 constraint force。PhysX 也有类似问题——`solver_position_iteration_count` 太少时穿透修正不足。

**排查代码**：

```python
# === 接触发散 NaN 排查 ===

def diagnose_contact_nan(env, max_steps=10000):
    """逐步运行环境，在 NaN 出现前捕获物理状态。"""
    obs, _ = env.reset()
    policy = load_policy("model.pt")

    for step in range(max_steps):
        action = policy(obs['policy'])
        obs, reward, done, truncated, info = env.step(action)

        # 检查 qvel 中的 NaN
        qvel = env.robot.data.joint_vel
        if torch.isnan(qvel).any():
            nan_envs = torch.isnan(qvel).any(dim=-1).nonzero().squeeze()
            print(f"NaN at step {step}, envs: {nan_envs.tolist()}")

            # 打印 NaN 前一步的状态
            if step > 0:
                print(f"  Previous step action: {prev_action[nan_envs[0]]}")
                print(f"  Previous step qpos: {prev_qpos[nan_envs[0]]}")
                print(f"  Contact count: {env.sim.data.ncon}")
                # 查看接触力
                for c_idx in range(min(5, env.sim.data.ncon)):
                    contact = env.sim.data.contact[c_idx]
                    print(f"  Contact {c_idx}: "
                          f"geom1={contact.geom1}, geom2={contact.geom2}, "
                          f"dist={contact.dist:.4f}, "
                          f"force={contact.force}")
            break

        # 保存当前步状态用于下一步对比
        prev_action = action.clone()
        prev_qpos = env.robot.data.joint_pos.clone()
```

**修复方向**：

```python
# 修复 1：降低 condim 和摩擦
# MJCF 中
# <geom condim="4" friction="0.8 0.005 0.001"/>  # 而非 condim=6 friction=5.0

# 修复 2：增加求解器迭代次数
sim = SimCfg(
    mujoco=MujocoCfg(
        iterations=15,      # 从 10 增到 15
        ls_iterations=25,    # 从 20 增到 25
    ),
)

# 修复 3：减小 timestep（更小的 dt = 更精确的积分）
sim = SimCfg(
    mujoco=MujocoCfg(timestep=0.002),  # 从 0.005 减到 0.002
    # 注意：decimation 也需要调整以保持相同的策略频率
    # 新 decimation = old_decimation * (old_dt / new_dt) = 4 * (0.005/0.002) = 10
)

# 修复 4（PhysX）：增加求解器迭代
articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    solver_position_iteration_count=8,   # 从 4 增到 8
    solver_velocity_iteration_count=8,
)
```

### 根因 2：Reward 函数中的除零/溢出 ⭐⭐

**症状**：某个 reward term 在特定 env 状态下返回 inf 或 NaN。通常是 `1/distance` 形式的 reward 在距离趋近零时溢出。

**排查**：逐项检查每个 reward term 的输出范围。

```python
# === Reward 函数 NaN 排查 ===
def diagnose_reward_nan(env, policy, num_steps=100):
    """分项检查每个 reward term 的输出。"""
    obs, _ = env.reset()
    for step in range(num_steps):
        action = policy(obs['policy'])
        obs, reward, done, truncated, info = env.step(action)

        # 分项打印 reward
        for name, value in env.reward_manager.compute_terms().items():
            if torch.isnan(value).any():
                nan_envs = torch.isnan(value).nonzero()
                print(f"  NaN in reward '{name}' at step {step}, "
                      f"envs: {nan_envs.squeeze().tolist()}")
            if torch.isinf(value).any():
                inf_envs = torch.isinf(value).nonzero()
                print(f"  Inf in reward '{name}' at step {step}, "
                      f"envs: {inf_envs.squeeze().tolist()}")
```

**修复**：永远不用 `1/d` 形式的 reward——改用 `exp(-d²/σ²)`。

```python
# ❌ 错误：距离趋零时溢出
def bad_distance_reward(env):
    d = compute_distance(env)
    return 1.0 / (d + 1e-8)  # d=0 时返回 1e8，梯度爆炸

# ✅ 正确：指数核，有界且处处可导
def good_distance_reward(env, sigma=0.25):
    d = compute_distance(env)
    return torch.exp(-d**2 / sigma**2)  # d=0 返回 1.0，d→∞ 返回 0
```

### 根因 3：Obs normalizer 未预热 ⭐⭐

**症状**：训练的前 1-10 个 iteration 出现 NaN（非常早期）。

**根因**：RSL-RL 的 `EmpiricalNormalization` 使用 Welford 算法在线计算 running mean 和 std。在训练刚开始时，std 可能为零（所有 obs 相同）或接近零（方差很小），归一化操作 `(obs - mean) / std` 产生极大的数值 → PPO 的 loss 溢出 → 梯度 NaN。

**修复**：在正式训练前用 random agent 预热 normalizer。

```python
# === Normalizer 预热 ===
def warmup_normalizer(env, policy, num_warmup_steps=500):
    """用随机动作预热 obs normalizer。"""
    obs, _ = env.reset()
    for _ in range(num_warmup_steps):
        action = torch.randn(env.num_envs, env.action_space.shape[-1],
                            device=env.device)
        obs, _, _, _, _ = env.step(action)
        # normalizer 在 env.step 内自动更新
    print(f"Normalizer warmup done. "
          f"obs_mean range: [{policy.obs_normalizer.running_mean.min():.2f}, "
          f"{policy.obs_normalizer.running_mean.max():.2f}]")
    print(f"obs_std range: [{policy.obs_normalizer.running_var.sqrt().min():.4f}, "
          f"{policy.obs_normalizer.running_var.sqrt().max():.4f}]")
    # 检查是否有零方差的维度
    zero_var = (policy.obs_normalizer.running_var < 1e-8).nonzero()
    if len(zero_var) > 0:
        print(f"⚠️ 零方差 obs 维度: {zero_var.squeeze().tolist()}")
        print("  这些维度可能是常数 obs（如固定 command）——检查是否合理")
```

### 根因 4：Policy std 变负 ⭐⭐

**症状**：PPO update 时 `log_prob` 计算出现 NaN。

**根因**：某些 RSL-RL 版本中 policy 的 action std 直接用线性层输出——如果输出值为负数，`Normal(mean, std)` 中 `std < 0` 导致 NaN。

**修复**：使用 `softplus(std)` 或 `log` 参数化。

```python
# ❌ 错误：std 可能为负
self.std = nn.Parameter(torch.ones(action_dim) * init_noise_std)
# 训练中 std 可能被梯度更新为负数

# ✅ 正确：log 参数化
self.log_std = nn.Parameter(torch.zeros(action_dim) + np.log(init_noise_std))
# std = exp(log_std) > 0，永远为正

# ✅ 正确：softplus 参数化
self.raw_std = nn.Parameter(torch.ones(action_dim))
# std = softplus(raw_std) > 0
```

### 根因 5：CUDA Graph capture 失效 ⭐

**症状**：训练中突然出现随机的 NaN——不是每次都在同一个 step 或同一个 env。

**根因**：Domain randomization 的某些 EventTerm 改变了 GPU 数组的地址或形状（如 `expand_model_fields()` 扩展了 per-world 摩擦数组），但 CUDA Graph 仍在 replay 旧地址的 kernel——访问了无效内存。

**排查**：

```python
# 检查是否有 CUDA Graph 相关的 warning
# mjlab 中会打印:
# "[WARN] Graph invalidated after expand_model_fields, re-capturing..."
# 如果没有这个 warning，说明 graph 没有被重新 capture

# 修复 1：确认 expand_model_fields 后调用了 create_graph()
# 修复 2：禁用 CUDA Graph 测试是否 NaN 消失
#   如果禁用后 NaN 消失 → 确认是 graph 问题
#   如果禁用后 NaN 仍在 → graph 不是根因
```

### mjlab 的 NaN Guard 工具 ⭐⭐⭐

mjlab 提供了内置的 NaN 检测和 dump 机制：

```bash
# 启用 NaN guard
uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 256 \
    --agent.max-iterations 100 \
    --enable-nan-guard True

# NaN 发生时自动 dump:
# /tmp/mjlab/nan_dumps/nan_dump_latest.npz
# 包含: env_id, step, qpos, qvel, qacc, action, obs, reward

# 可视化 NaN dump
uv run viz-nan /tmp/mjlab/nan_dumps/nan_dump_latest.npz
# 在 Viser viewer 中显示 NaN 发生时的物理状态
```

**NaN dump 的分析流程**：

```python
# === 分析 NaN dump ===
import numpy as np

def analyze_nan_dump(path):
    """分析 mjlab 的 NaN dump 文件。"""
    dump = np.load(path, allow_pickle=True)
    print(f"=== NaN Dump Analysis ===")
    print(f"Env ID: {dump['env_id']}")
    print(f"Step: {dump['step']}")

    # 检查哪些物理量是 NaN
    for key in ['qpos', 'qvel', 'qacc', 'action', 'obs', 'reward']:
        if key in dump:
            data = dump[key]
            nan_mask = np.isnan(data)
            if nan_mask.any():
                nan_indices = np.where(nan_mask)[0]
                print(f"  {key}: NaN at indices {nan_indices[:10]}...")
            else:
                print(f"  {key}: no NaN (range: [{data.min():.2f}, {data.max():.2f}])")

    # 判断根因
    if 'qvel' in dump and np.isnan(dump['qvel']).any():
        print("\n  → 根因可能是接触求解器发散")
        print("    修复：降低 condim/摩擦、增加 solver iterations、减小 timestep")
    elif 'reward' in dump and np.isnan(dump['reward']).any():
        print("\n  → 根因可能是 reward 函数中的除零/溢出")
        print("    修复：检查所有 reward term，替换 1/d 为 exp(-d²/σ²)")
    elif 'obs' in dump and np.isnan(dump['obs']).any():
        print("\n  → 根因可能是 obs normalizer 问题或传感器异常")
        print("    修复：预热 normalizer，检查传感器配置")

analyze_nan_dump("/tmp/mjlab/nan_dumps/nan_dump_latest.npz")
```

### NaN 排查优先级表 ⭐⭐

当 NaN 发生时，按以下优先级逐步排查——每步确认无问题后再进入下一步：

| 优先级 | 检查项 | 检查方法 | 修复方向 |
|--------|--------|---------|---------|
| 1 | Reward 中是否有除零 | 分项打印 reward term | 替换为 exp(-d²/σ²) |
| 2 | Action scale 是否过大 | 打印 action 范围 | 减小到 0.5× 试试 |
| 3 | Obs normalizer 是否预热 | 打印 running_var | 加 warmup 步骤 |
| 4 | Actuator gains 是否合理 | PD 阶跃响应测试 | 降低 kp/kd |
| 5 | Timestep 是否太大 | 自由落体弹跳测试 | 减小 dt |
| 6 | Solver iterations 是否足够 | 接触力稳定性检查 | 增加 iterations |
| 7 | nconmax/njmax 是否足够 | 检查 warning 信息 | 增大 buffer |
| 8 | Contact 参数 | solref/solimp 检查 | 使用更软的接触 |
| 9 | Policy std 参数化 | 检查 std 是否有负值 | 用 softplus/log |
| 10 | CUDA Graph 是否有效 | 禁用 graph 重试 | 重新 capture |

**排查的黄金法则：先缩小复现范围，再定位根因。** 不要在 4096 envs 上排查 NaN——先用 16 envs + 短训练复现。如果 16 envs 不复现，逐步增大 envs 数量直到复现——这能帮你判断 NaN 是否和 env 数量（内存、CUDA Graph）相关。

> **反事实推理：如果不用 NaN guard 直接凭经验猜会怎样？** 你看到 reward 变成 NaN，猜测"可能是 contact 参数不对"，花了两天调 solref/solimp——结果问题是 reward 函数中的 `1/d` 在距离趋零时溢出。NaN guard 的 dump 能在 5 分钟内告诉你"NaN 首先出现在 reward，不是 qvel"——直接指向 reward 函数而非 contact 参数。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：用 `torch.nan_to_num()` 掩盖 NaN**
- 某些框架在 reward 计算后用 `nan_to_num` 把 NaN 替换为 0——这消除了错误信号但不修复根因
- 训练可能继续但策略行为诡异（因为某些 reward 被静默替换为 0）
- 正确做法：找到并修复 NaN 的根因，而非掩盖它

⚠️ **思维陷阱：NaN + 多 GPU 一起排查**
- 如果每个 rank 都偶发 NaN，不要在多 GPU 环境下排查
- 先缩小到单 GPU + 小 env 数定位根因
- 确认修复后再扩展回多 GPU 验证

⚠️ **编程陷阱：NaN 在 curriculum 阶段切换后出现**
- Curriculum 推进时新增了 reward term 或修改了权重，如果新 term 有 NaN 风险且旧 term 没有，NaN 只在推进后出现
- 排查：固定 curriculum 在推进前后的阶段各跑 100 iterations，确认 NaN 是哪个阶段的问题

### 练习

1. **[实践题]** 在你的自定义环境中故意引入一个 `1/d` 形式的 reward（距离趋零时溢出）。运行训练，观察 NaN 何时出现。然后用 `--enable-nan-guard` 重新运行，分析 dump。
2. **[分析题]** 一个训练在第 3000 iteration 出现 NaN（之前 2999 iterations 正常）。列出三个可能的触发条件，并解释为什么这些条件在训练后期才触发。
3. **[跨章综合题]** 结合 Ch08 的 DR 配置，分析以下场景：DR 的摩擦范围从 [0.5, 1.0] 扩大到 [0.1, 2.0] 后训练开始频繁 NaN。可能的根因是什么？应该怎么修复？（提示：低摩擦 + 高速接触 → 求解器发散）

### CUDA Graph 诊断与修复 ⭐⭐

CUDA Graph 是 mjlab 和 Isaac Lab 高吞吐的关键机制——但它也是 NaN 和性能异常的隐蔽来源。当 Graph 失效但未被正确重新 capture 时，GPU 可能访问无效的内存地址，产生随机的错误值。

**CUDA Graph 失效的三种场景**：

| 场景 | 触发条件 | 症状 | 修复 |
|------|---------|------|------|
| DR 扩展 model fields | `expand_model_fields()` 改变了 per-world 数组 | 随机 NaN 或物理行为异常 | 确认 `create_graph()` 被重新调用 |
| Sensor 配置变更 | `set_sensor_context()` 改变传感器数组 | sense graph 失效，传感器返回旧值 | 重新 capture sense graph |
| Env 数量变化 | resume 时 num_envs 和 capture 时不同 | 维度不匹配崩溃 | 用相同 num_envs resume |

**诊断脚本**：

```python
# === CUDA Graph 状态诊断 ===
def diagnose_cuda_graph(sim):
    """检查 CUDA Graph 的状态是否有效。"""
    print("=== CUDA Graph Diagnostics ===")

    # 检查 graph 是否存在
    has_step = hasattr(sim, 'step_graph') and sim.step_graph is not None
    has_sense = hasattr(sim, 'sense_graph') and sim.sense_graph is not None
    has_reset = hasattr(sim, 'reset_graph') and sim.reset_graph is not None
    print(f"  step_graph: {'✅' if has_step else '❌ Missing'}")
    print(f"  sense_graph: {'✅' if has_sense else '❌ Missing'}")
    print(f"  reset_graph: {'✅' if has_reset else '❌ Missing'}")

    # 检查是否有 expanded fields（可能需要 re-capture）
    if hasattr(sim, '_expanded_fields'):
        expanded = sim._expanded_fields
        if expanded:
            print(f"  ⚠️ Expanded fields: {list(expanded.keys())}")
            print(f"    → 这些字段被 expand 后是否重新 capture 了 graph？")
        else:
            print(f"  ✅ No expanded fields")

    # 检查 device
    print(f"  Device: {sim.device}")
    print(f"  Num worlds: {sim.num_worlds}")

    # 尝试禁用 graph 运行一步，对比
    print("\n  Running one step WITHOUT graph (direct call)...")
    try:
        import mujoco.warp as mjwarp
        mjwarp.step(sim.wp_model, sim.wp_data)
        print("  ✅ Direct step succeeded")
    except Exception as e:
        print(f"  ❌ Direct step failed: {e}")
```

**Graph 失效的修复模式**：

```python
# 当 DR 修改了 model fields 时的正确流程：
def apply_dr_with_graph_rebuild(sim, dr_events):
    """应用 DR 并正确重建 CUDA Graph。"""
    # 1. 应用 DR（可能 expand fields）
    for event in dr_events:
        event.apply(sim)

    # 2. 检查是否有 field 被 expand
    if sim._needs_graph_rebuild:
        # 3. 重建所有 graph
        sim.create_graph()
        sim._needs_graph_rebuild = False
        print("[INFO] CUDA Graph rebuilt after DR expansion")
```

### 多 GPU 环境下的 NaN 特殊处理 ⭐⭐

多 GPU 训练中 NaN 的排查更困难——因为 NaN 可能只发生在某个 rank 上，而其他 rank 的日志看起来正常。

```python
# === 多 GPU NaN 监控 ===
def check_nan_all_ranks(tensor, name="tensor"):
    """跨 rank 检查 NaN 并报告哪个 rank 有问题。"""
    has_nan = torch.isnan(tensor).any()

    if dist.is_initialized():
        # 收集所有 rank 的 NaN 状态
        nan_counts = torch.tensor([has_nan.int()], device=tensor.device)
        dist.all_reduce(nan_counts, op=dist.ReduceOp.SUM)

        if nan_counts.item() > 0:
            rank = dist.get_rank()
            if has_nan:
                print(f"⚠️ Rank {rank}: NaN detected in {name}")
                print(f"   Shape: {tensor.shape}")
                nan_idx = torch.isnan(tensor).nonzero()
                print(f"   NaN indices: {nan_idx[:5].tolist()}")
            dist.barrier()  # 所有 rank 同步
            return True
    elif has_nan:
        print(f"⚠️ NaN detected in {name}")
        return True

    return False

# 在训练循环中使用
for iteration in range(max_iterations):
    obs, reward, done = rollout(env, policy)
    if check_nan_all_ranks(obs['policy'], "observation"):
        print("Saving NaN dump and stopping...")
        save_nan_dump(env, iteration)
        break
    if check_nan_all_ranks(reward, "reward"):
        print("Saving NaN dump and stopping...")
        save_nan_dump(env, iteration)
        break
```

---

NaN 排查建立了"训练能稳定运行"的基础。但"能运行"不等于"运行得快"——环境的吞吐量瓶颈可能在物理计算、传感器、或 Python 层面。下一节讲如何量化和优化性能。

---


## 24.4 性能优化：从 Profiling 到 Tuning ⭐⭐⭐

> **这一节解决什么问题**：如何量化训练的性能瓶颈？在物理计算、传感器、Manager 和 PPO update 之间，时间花在了哪里？

### 性能度量标准 ⭐⭐

RL 训练的性能不只是"steps/s"一个数字——需要区分多个层次：

| 度量 | 定义 | 含义 |
|------|------|------|
| `physics_sps` | 物理步/秒（含 decimation 内的所有 substep） | 物理引擎的原始吞吐 |
| `env_sps` | 环境步/秒（不含 decimation） | 策略频率下的吞吐 |
| `train_sps` | 训练步/秒（含 PPO update） | 端到端训练速度 |
| `VRAM_MB` | GPU 显存占用 | 决定能跑多少 envs |
| `wall_time_per_iter` | 每次 PPO iteration 的 wall-clock 时间 | 决定总训练时间 |

**关键关系**：`env_sps = physics_sps / decimation`，`train_sps < env_sps`（因为 PPO update 也需要时间）。

如果 `physics_sps` 很高但 `env_sps` 很低，瓶颈在传感器或 Manager 的 Python 逻辑。如果 `env_sps` 很高但 `train_sps` 很低，瓶颈在 PPO update（网络太大或 mini_batch 太多）。

### 性能 Profiling 工具链 ⭐⭐

**快速 Benchmark（5 分钟）**：

```bash
# 测量 env 吞吐（不包含 PPO update）
# 在 play 模式下用 random agent 跑 1000 steps
time uv run play Mjlab-Velocity-Flat-Unitree-Go1 \
    --agent random --num-envs 4096 --max-steps 1000 --no-render
# 输出的 wall time 用于计算:
# env_sps = num_envs * max_steps / wall_time_seconds

# 测量 train 吞吐（包含 PPO update）
time uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 4096 --agent.max-iterations 10 \
    --agent.logger tensorboard --headless
# train_sps = num_envs * num_steps_per_env * 10 / wall_time_seconds
```

**深度 Profiling（30 分钟）**：

```python
# === torch.profiler 深度分析 ===
import torch.profiler

def profile_training(env, policy, runner, num_iters=5):
    """使用 torch.profiler 分析训练的时间分布。"""
    with torch.profiler.profile(
        activities=[
            torch.profiler.ProfilerActivity.CPU,
            torch.profiler.ProfilerActivity.CUDA,
        ],
        schedule=torch.profiler.schedule(
            wait=1, warmup=1, active=3, repeat=1
        ),
        on_trace_ready=torch.profiler.tensorboard_trace_handler("./profile"),
        record_shapes=True,
        with_stack=True,
    ) as prof:
        for i in range(num_iters):
            runner.train_one_iteration()
            prof.step()

    # 打印 top 20 耗时操作
    print(prof.key_averages().table(
        sort_by="cuda_time_total", row_limit=20
    ))

    # 查找瓶颈
    events = prof.key_averages()
    total_cuda = sum(e.cuda_time_total for e in events)
    for e in sorted(events, key=lambda x: x.cuda_time_total, reverse=True)[:5]:
        pct = e.cuda_time_total / total_cuda * 100
        print(f"  {e.key:40s} {e.cuda_time_total/1000:.1f} ms ({pct:.1f}%)")
```

**NVIDIA nsys Profiling（GPU kernel 级）**：

```bash
# 使用 nsys 分析 GPU kernel 执行
nsys profile --trace=cuda,nvtx \
    --output=train_profile \
    uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 4096 --agent.max-iterations 5 --headless

# 打开 nsys UI 分析 timeline
nsys-ui train_profile.nsys-rep
```

### 传感器成本量化 ⭐⭐

传感器（contact sensor、height scan、camera）通常是环境吞吐的最大瓶颈。以下代码展示如何量化传感器的性能开销：

```python
# === 传感器成本量化 ===
import time

def benchmark_sensor_cost(task, num_envs=4096, num_steps=500):
    """量化传感器对吞吐的影响。"""

    # 1. 有传感器的吞吐
    env_with_sensor = make(task, num_envs=num_envs)
    obs, _ = env_with_sensor.reset()
    t0 = time.perf_counter()
    for _ in range(num_steps):
        action = torch.randn(num_envs, env_with_sensor.action_space.shape[-1],
                            device="cuda")
        env_with_sensor.step(action)
    t_with = time.perf_counter() - t0
    sps_with = num_envs * num_steps / t_with
    env_with_sensor.close()

    # 2. 无传感器的吞吐（需要修改 env config 移除 sensor）
    # 或者用更简单的方法：对比有无 height scan
    # task_no_sensor = task.replace("Rough", "Flat")  # Flat 没有 height scan
    # ...

    print(f"With sensors: {sps_with:.0f} steps/s")
    # print(f"Without sensors: {sps_without:.0f} steps/s")
    # print(f"Sensor overhead: {(1 - sps_with/sps_without)*100:.1f}%")
```

**各组件的典型成本比例**（在 A100 上，4096 envs，29-DOF 人形）：

| 组件 | 占比 | 说明 |
|------|------|------|
| 物理引擎 (mj_step) | 40-50% | 接触检测 + 求解器 |
| 传感器 (sense) | 15-30% | height scan > contact > raycast |
| Manager (obs/reward/term) | 10-20% | Python 层 dispatch + tensor ops |
| PPO update | 10-20% | 网络 forward + backward |
| 其他 (reset/logging/CUDA sync) | 5-10% | |

### nconmax/njmax 调优 ⭐⭐

`nconmax`（per-world 最大接触数）和 `njmax`（per-world 最大约束行数）直接影响 GPU 显存和性能。设太小会导致接触截断（穿透、物理不稳定）；设太大浪费显存和降低 cache locality。

**正确的调优方法**：先用默认值跑，记录实际接触数分布，然后设为 1.5× 最大观测值。

```python
# === nconmax 调优 ===
def find_optimal_nconmax(env, num_steps=1000):
    """记录实际接触数分布，推荐 nconmax。"""
    obs, _ = env.reset()
    ncon_history = []

    for _ in range(num_steps):
        action = torch.randn(env.num_envs, env.action_space.shape[-1],
                            device=env.device)
        env.step(action)
        # 记录每个 env 的接触数
        ncon = env.sim.data.ncon  # [num_envs] or scalar
        if isinstance(ncon, int):
            ncon_history.append(ncon)
        else:
            ncon_history.extend(ncon.tolist())

    ncon_arr = np.array(ncon_history)
    print(f"Contact count statistics:")
    print(f"  mean: {ncon_arr.mean():.1f}")
    print(f"  p50: {np.percentile(ncon_arr, 50):.0f}")
    print(f"  p95: {np.percentile(ncon_arr, 95):.0f}")
    print(f"  max: {ncon_arr.max():.0f}")
    recommended = int(ncon_arr.max() * 1.5)
    print(f"  Recommended nconmax: {recommended}")
    return recommended
```

### num_envs 选择指南 ⭐⭐

```python
# === 自动寻找最优 num_envs ===
def find_optimal_num_envs(task, env_range=[256, 512, 1024, 2048, 4096, 8192]):
    """找到吞吐最高的 num_envs。"""
    results = []
    for n in env_range:
        try:
            env = make(task, num_envs=n)
            obs, _ = env.reset()
            # Warmup
            for _ in range(50):
                env.step(torch.randn(n, env.action_space.shape[-1], device="cuda"))
            # Benchmark
            t0 = time.perf_counter()
            for _ in range(200):
                env.step(torch.randn(n, env.action_space.shape[-1], device="cuda"))
            wall = time.perf_counter() - t0
            sps = n * 200 / wall
            vram = torch.cuda.max_memory_allocated() / 1e6
            results.append({"num_envs": n, "sps": sps, "vram_mb": vram})
            print(f"  num_envs={n:5d}: {sps:.0f} sps, {vram:.0f} MB VRAM")
            env.close()
            torch.cuda.empty_cache()
        except RuntimeError as e:
            if "out of memory" in str(e):
                print(f"  num_envs={n:5d}: OOM")
                break
            raise

    # 找到 sps 最高的配置
    best = max(results, key=lambda x: x["sps"])
    print(f"\nOptimal: num_envs={best['num_envs']} "
          f"({best['sps']:.0f} sps, {best['vram_mb']:.0f} MB)")
    return best
```

### 性能调优决策表 ⭐⭐

| 现象 | 优先怀疑 | 第一检查项 | 调整方向 |
|------|----------|------------|----------|
| steps/s < 预期的 50% | sensor 过重 | 关闭 sensor 对比 | 降低 sensor 数量/分辨率 |
| GPU 利用率 < 50% | num_envs 太小 | 增大 num_envs | 翻倍到 GPU 饱和 |
| VRAM 接近上限 | num_envs 或 buffer 太大 | nvidia-smi 监控 | 降低 num_envs 或 nconmax |
| 训练前几步极慢 | CUDA Graph capture | 检查 warning 信息 | 等待 capture 完成 |
| PPO update 占比 > 30% | 网络太大或 mini_batch 太多 | profiler 检查 | 减小网络层或 mini_batch |
| physics_sps 低但 env_sps 合理 | timestep/iterations 过多 | 增大 timestep 试试 | 权衡精度和速度 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：在 reward 函数中用 Python for 循环**
- 对每个 env 单独计算 reward（而非 batch tensor 操作）——N 个 env 就执行 N 次 Python 循环
- 修复：所有 reward/obs 函数必须是 batch tensor 操作

⚠️ **思维陷阱：性能不好就换框架**
- "mjlab 太慢，换 Isaac Lab"——更可能的原因是 sensor 配置或 num_envs 不对
- 正确做法：先 profile，找到瓶颈再决定

⚠️ **编程陷阱：benchmark 时包含了 warmup**
- 前几步包含 CUDA Graph capture、JIT 编译等一次性开销——会拉低平均 steps/s
- 正确做法：先跑 50-100 步 warmup，再开始计时

### 练习

1. **[实践题]** 对你的自定义环境运行 `find_optimal_num_envs()`。画出 num_envs vs steps/s 和 num_envs vs VRAM 的曲线。最优 num_envs 是多少？
2. **[实验题]** 在一个有 height scan sensor 的四足任务上，分别测量有无 height scan 的 env_sps。Sensor 带来了多大的开销百分比？
3. **[分析题]** 为什么过大的 nconmax 会降低性能？从 GPU cache locality 的角度解释。

### 完整性能 Profiling 案例 ⭐⭐⭐

以下是一个真实的性能优化案例——从"训练太慢"到"找到瓶颈"到"优化后加速 2.5×"的完整流程。

**背景**：Unitree G1 29-DOF 人形 locomotion 任务，4096 envs，单 A100。初始 train_sps ≈ 45,000 steps/s，目标 >100,000 steps/s。

**Step 1：定位瓶颈层次**

```python
# 分层测量
# 1. 纯物理 step（无 sensor，无 manager）
physics_sps = benchmark_physics_only(env)  # 220,000 sps

# 2. 物理 + sensor
physics_sensor_sps = benchmark_with_sensors(env)  # 120,000 sps
sensor_overhead = 1 - physics_sensor_sps / physics_sps  # 45%!

# 3. 物理 + sensor + managers
env_sps = benchmark_env_step(env)  # 95,000 sps
manager_overhead = 1 - env_sps / physics_sensor_sps  # 21%

# 4. 完整训练（含 PPO update）
train_sps = benchmark_training(env, runner)  # 45,000 sps
ppo_overhead = 1 - train_sps / env_sps  # 53%!
```

**Step 2：逐项优化**

```
发现 1: sensor 开销 45% → 罪魁祸首是 1024-ray height scan
  修复: 减少到 121 rays (11×11 grid) → sensor 开销降到 15%

发现 2: PPO 开销 53% → critic 网络太大 [1024, 512, 256]
  修复: 改为 [512, 256, 128] (AGILE 默认) → PPO 开销降到 25%

发现 3: manager overhead 21% → reward 函数中有 per-env 循环
  修复: 改为 batch tensor 操作 → manager 开销降到 8%
```

**Step 3：优化后结果**

```
优化前: train_sps = 45,000
优化后: train_sps = 115,000 (2.56× 加速)

分解:
  Physics: 220,000 → 220,000 (不变)
  + Sensors: 120,000 → 195,000 (+63%)
  + Managers: 95,000 → 180,000 (+89%)
  + PPO: 45,000 → 115,000 (+156%)
```

**关键教训**：性能瓶颈很少在你猜测的地方——这个案例中，用户最初怀疑"MuJoCo Warp 物理引擎太慢"，但实际瓶颈在 sensor 和 PPO 网络大小。没有 profiling 的猜测只会浪费时间。

### Timestep × Decimation × 策略频率的权衡 ⭐⭐

这三个参数之间的关系经常被混淆，但它们直接影响训练质量和速度：

```python
# 参数关系
physics_freq = 1.0 / timestep          # 物理更新频率
policy_freq = physics_freq / decimation  # 策略决策频率
control_dt = timestep * decimation       # 策略的控制周期

# 典型配置
# 四足 locomotion:
#   timestep=0.005, decimation=4 → physics 200Hz, policy 50Hz
# 人形 locomotion:
#   timestep=0.002, decimation=10 → physics 500Hz, policy 50Hz
# 灵巧手操作:
#   timestep=0.001, decimation=5 → physics 1000Hz, policy 200Hz
```

| 如果改变... | 物理精度 | 训练速度 | 策略频率 | 策略质量 |
|------------|---------|---------|---------|---------|
| 减小 timestep | ↑ | ↓ | 不变（需调 decimation） | ↑（更稳定的物理） |
| 增大 decimation | 不变 | ↑ | ↓ | ↓（策略看到更旧的信息） |
| 两者同时调 | 取决于比例 | 取决于比例 | 取决于比例 | 需要实验验证 |

> **反事实推理：如果把 timestep 从 0.005 增大到 0.01（为了加速），但 decimation 不变会怎样？** Policy 频率从 50Hz 降到 25Hz。对于四足行走，25Hz 可能足够（步态周期 ~0.5s，每步 12.5 个决策点）。但对于跑步（步态周期 ~0.3s），25Hz 只有 7.5 个决策点——可能不够精细，导致脚步落点不精确。此外，更大的 timestep 会降低接触求解的精度——高冲击接触（如足端着地）可能产生更大的穿透和不稳定。

### Isaac Lab vs mjlab 性能对比 ⭐⭐

两个框架在同一硬件上的性能差异主要来自物理引擎（PhysX vs MuJoCo Warp）和框架层开销：

| 维度 | mjlab (MuJoCo Warp) | Isaac Lab (PhysX) |
|------|--------------------|--------------------|
| 接触求解器 | Newton solver (GPU) | TGS solver (GPU) |
| 典型 env_sps (四足) | 150k-250k | 120k-200k |
| 典型 env_sps (人形) | 80k-120k | 60k-100k |
| 接触精度 | 高（Newton 全局求解） | 中（TGS 迭代求解） |
| CUDA Graph 支持 | 原生 | 部分（取决于 PhysX 版本） |
| 多 GPU 方式 | torchrunx | torchrun |
| Sensor 成本 | 中（raycast in Warp） | 中（raycast in PhysX） |
| Camera sensor 成本 | 高（需要渲染） | 高（但 tiled rendering 优化） |

注意：这些数字是 order-of-magnitude 参考，实际值取决于具体任务配置、GPU 型号和框架版本。不应该仅基于 steps/s 选择框架——物理精度、API 设计、生态支持和部署管线的完整性更重要。

### Benchmark 协议必备字段 ⭐⭐

报告性能数字时，必须附带完整的实验协议——否则数字无法被复查。"mjlab 比 X 快 3 倍"缺少了使结论有效的全部上下文：什么任务？什么机器人？什么传感器？多少并行环境？什么硬件？warmup 如何处理？没有这些上下文的数字，就像说"A 比 B 跑得快"——但 A 在塑胶跑道上穿跑鞋，B 在沙滩上赤脚。

一个规范的性能报告应当包含以下字段：

| 字段 | 示例写法 | 缺少时的风险 |
|------|---------|------------|
| task_id | `Mjlab-Velocity-Flat-Unitree-Go1` | 不同任务不可比 |
| robot | Unitree Go1 (12-DOF) | DOF 不同性能差异巨大 |
| sensors | height_scan 121 rays + contact | sensor 是主要性能瓶颈 |
| num_envs | 4096 | 不同 env 数吞吐不同 |
| GPU | NVIDIA A100 40GB | 不同 GPU 不可比 |
| framework_version | mjlab v0.3.1 / Isaac Lab v2.1 | 版本间性能可能差异 20%+ |
| warmup_steps | 100 steps (excluded) | 包含 warmup 会拉低平均值 |
| measurement_steps | 1000 steps | 太少方差大 |
| metric | env_sps (steady-state) | env_sps ≠ train_sps |
| CUDA Graph | enabled / disabled | graph 启用后快 30-50% |
| viewer/video | disabled | 开启 viewer 性能降 2-5× |

**规范的性能结论示例**："在 Mjlab-Velocity-Flat-Unitree-Go1 任务上，使用 RTX 4090、4096 envs、无 viewer/video、121-ray height scan、100 步 warmup 后测量 1000 步，steady-state env_sps 为 185,000。该数字只代表环境步吞吐，不代表训练收敛速度。"

### 显存估算与 num_envs 上限 ⭐⭐

```python
# === 显存估算公式 ===
def estimate_vram(
    num_envs: int,
    num_joints: int,
    nconmax: int,
    policy_params: int,  # actor + critic 参数量
    obs_dim: int,
    num_steps: int = 24,  # PPO rollout length
) -> float:
    """估算训练所需的 GPU 显存 (MB)。"""
    # Per-env 物理状态（qpos + qvel + qacc + ctrl + sensor）
    physics_per_env = (num_joints * 4 + nconmax * 10) * 4  # bytes, float32

    # Per-env obs/action/reward rollout buffer
    rollout_per_env = (obs_dim + num_joints + 1) * num_steps * 4

    # Policy 网络参数 + optimizer states
    policy_mem = policy_params * 4 * 3  # param + grad + optimizer state

    # 总计
    total_mb = (
        num_envs * (physics_per_env + rollout_per_env) / 1e6
        + policy_mem / 1e6
    )
    return total_mb

# 示例：G1 29-DOF, 4096 envs, AGILE 网络
vram = estimate_vram(
    num_envs=4096, num_joints=29, nconmax=35,
    policy_params=500000, obs_dim=48,
)
print(f"Estimated VRAM: {vram:.0f} MB")
# 输出约 2000-3000 MB → A100 40GB 可以跑 4096 envs

# 估算最大 num_envs
for n in [4096, 8192, 16384, 32768]:
    v = estimate_vram(n, 29, 35, 500000, 48)
    print(f"  {n} envs: ~{v:.0f} MB {'✅' if v < 38000 else '❌ OOM'}")
```

---

性能优化确保了训练"跑得快"。但如果训练是在云端进行的，还需要管理集群生命周期和成本控制——这是下节的内容。

---

## 24.5 云端训练与成本控制 ⭐⭐

> **这一节解决什么问题**：如何在云端 GPU 上运行训练？如何控制成本？

### SkyPilot 云端训练 ⭐⭐

SkyPilot 是 mjlab 和 Isaac Lab 社区常用的云端 GPU 编排工具——它抽象了 AWS/GCP/Azure 等不同云厂商的 GPU 资源，用统一的 YAML 配置启动训练。

```yaml
# === sky_train.yaml: SkyPilot 训练配置 ===
name: go1-velocity-training

resources:
  accelerators: A100:1      # 1 块 A100
  use_spot: true             # 使用 spot 实例节省成本
  disk_size: 100             # GB

setup: |
  # 安装 mjlab
  pip install uv
  uvx --from mjlab demo  # 验证安装

run: |
  # 训练
  uv run train Mjlab-Velocity-Flat-Unitree-Go1 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 10000 \
    --agent.run-name cloud_go1_flat \
    --agent.logger wandb

  # 训练完成后自动上传 checkpoint
  uv run upload-artifacts logs/rsl_rl/
```

```bash
# === SkyPilot 命令 ===

# 启动训练
sky launch sky_train.yaml

# 查看状态
sky status

# 查看日志
sky logs go1-velocity-training

# 停止并释放资源（重要！忘记会持续计费）
sky down go1-velocity-training
```

### 云端成本控制 ⭐⭐

| GPU 类型 | 按需价格 ($/hr) | Spot 价格 ($/hr) | 4096 envs 四足 10k iter 时间 | 总成本 |
|---------|----------------|-----------------|---------------------------|--------|
| A100 40GB | ~3.0 | ~1.0 | ~2 hr | ~$2-6 |
| A100 80GB | ~4.0 | ~1.5 | ~2 hr | ~$3-8 |
| H100 | ~5.0 | ~2.0 | ~1.5 hr | ~$3-10 |
| L40S | ~1.5 | ~0.5 | ~3 hr | ~$1.5-4.5 |

**成本控制清单**：

- [ ] 使用 spot 实例（节省 50-70%），但需要处理中断
- [ ] 训练完成后立即 `sky down`——idle 也计费
- [ ] 设置 `max_iterations` 上限——防止训练跑飞
- [ ] 先用小 num_envs smoke test 确认能跑，再启动大规模训练
- [ ] 使用 WandB 远程监控——不需要 SSH 进去看日志

> **反事实推理：如果忘记 `sky down` 会怎样？** SkyPilot 启动的 GPU 实例不会自动关闭（除非配置了自动关闭策略）。一块 A100 闲置一天的成本是 ~$72。一个周五下午忘记关的实例到周一上午已经烧掉了 ~$200。正确做法：在 WandB 的训练完成 callback 中自动执行 `sky down`，或设置最大运行时间。

### WandB Sweep 超参搜索 ⭐

当你需要搜索多组超参时，WandB Sweep 可以自动化管理：

```yaml
# === wandb_sweep.yaml ===
method: bayes  # 贝叶斯优化
metric:
  name: reward_mean
  goal: maximize
parameters:
  learning_rate:
    min: 0.0001
    max: 0.01
  action_scale:
    values: [0.15, 0.2, 0.25, 0.3]
  entropy_coef:
    min: 0.001
    max: 0.02
  reward_tracking_weight:
    values: [1.0, 1.5, 2.0]
```

```bash
# 创建 sweep
wandb sweep wandb_sweep.yaml
# 输出: wandb sweep agent <SWEEP_ID>

# 在每块 GPU 上启动一个 agent
CUDA_VISIBLE_DEVICES=0 wandb agent <SWEEP_ID> &
CUDA_VISIBLE_DEVICES=1 wandb agent <SWEEP_ID> &
CUDA_VISIBLE_DEVICES=2 wandb agent <SWEEP_ID> &
CUDA_VISIBLE_DEVICES=3 wandb agent <SWEEP_ID> &
# 4 个 agent 并行搜索超参
```

### ⚠️ 常见陷阱

⚠️ **成本陷阱：Spot 实例被中断丢失 checkpoint**
- Spot 实例可能被云厂商随时回收
- 正确做法：每 500 iterations 自动保存 checkpoint + 上传到 WandB 或 S3

⚠️ **编程陷阱：Sweep agent 抢同一 GPU**
- 多个 agent 不加 `CUDA_VISIBLE_DEVICES` 限制，全部用 GPU 0
- 正确做法：每个 agent 绑定独立 GPU

### 练习

1. **[设计题]** 为一篇论文的 3 seed × 4 配置 = 12 个实验设计云端训练方案。假设单个实验需要 A100 × 2 小时。计算总 GPU-hours 和预估成本（spot 价格）。
2. **[实践题]** 在本地用 `sky launch` 启动一次小规模 smoke test（10 iterations）。验证 `sky down` 是否成功释放资源。记录 provision、setup、run 各阶段耗时。

---

## 24.6 实验管理与可复现性 ⭐⭐

> **这一节解决什么问题**：如何组织大规模训练的实验，使每个结论都可追溯到具体的配置和数据？

### 运行包协议 ⭐⭐

每次训练都应该产出一个可交接的"运行包"——包含复现该训练所需的所有信息。

```text
runs/go1_flat_v2_seed0/
├── command.txt              ← 完整命令（从 shell history 复制，不是回忆版）
├── sample_ledger.yaml       ← 样本量核算
├── params/
│   ├── env_cfg.yaml         ← rank 0 写出的完整环境配置快照
│   └── agent_cfg.yaml       ← PPO 超参
├── git_info.txt             ← commit hash + branch + uncommitted diff
├── logs/
│   ├── tensorboard/         ← TensorBoard events
│   └── wandb/               ← WandB 本地备份
├── checkpoints/
│   ├── model_5000.pt
│   └── model_10000.pt
├── eval/
│   ├── eval_report.html     ← AGILE Stage 3 评估报告
│   └── eval_videos/         ← 评估视频
└── deployment/
    ├── policy.onnx
    └── deploy.yaml
```

**样本量账本**（sample_ledger.yaml）是运行包的核心——没有它，任何关于"多少数据"或"收敛快慢"的结论都无法验证：

```yaml
# sample_ledger.yaml
num_envs_per_gpu: 4096
num_steps_per_env: 24
num_gpus: 1
max_iterations: 10000
samples_per_update: 98304         # 4096 * 24
total_env_steps: 983040000        # 98304 * 10000
comparison_mode: same_total_samples
x_axis: env_steps                 # 论文中的 x 轴
seed: 42
wall_time_hours: 2.3
gpu_type: A100-40GB
framework: mjlab
```

### 自动化实验记录 ⭐⭐

```python
# === 自动化实验记录 ===
import subprocess
import yaml
import os

def create_run_package(run_dir, env_cfg, agent_cfg, command):
    """创建标准化的运行包。"""
    os.makedirs(run_dir, exist_ok=True)

    # 1. 保存命令
    with open(f"{run_dir}/command.txt", "w") as f:
        f.write(command)

    # 2. 保存 Git 信息
    try:
        git_hash = subprocess.check_output(
            ["git", "rev-parse", "HEAD"]).decode().strip()
        git_branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"]).decode().strip()
        git_diff = subprocess.check_output(
            ["git", "diff", "--stat"]).decode().strip()
        with open(f"{run_dir}/git_info.txt", "w") as f:
            f.write(f"commit: {git_hash}\n")
            f.write(f"branch: {git_branch}\n")
            f.write(f"diff:\n{git_diff}\n")
    except subprocess.CalledProcessError:
        print("⚠️ Git info not available")

    # 3. 保存配置快照
    os.makedirs(f"{run_dir}/params", exist_ok=True)
    with open(f"{run_dir}/params/env_cfg.yaml", "w") as f:
        yaml.dump(vars(env_cfg), f, default_flow_style=False)
    with open(f"{run_dir}/params/agent_cfg.yaml", "w") as f:
        yaml.dump(vars(agent_cfg), f, default_flow_style=False)

    # 4. 计算样本量账本
    num_envs = env_cfg.scene.num_envs
    num_steps = agent_cfg.runner.num_steps_per_env
    num_gpus = len(env_cfg.gpu_ids) if hasattr(env_cfg, 'gpu_ids') else 1
    max_iters = agent_cfg.runner.max_iterations
    ledger = {
        "num_envs_per_gpu": num_envs,
        "num_steps_per_env": num_steps,
        "num_gpus": num_gpus,
        "max_iterations": max_iters,
        "samples_per_update": num_envs * num_steps * num_gpus,
        "total_env_steps": num_envs * num_steps * num_gpus * max_iters,
    }
    with open(f"{run_dir}/sample_ledger.yaml", "w") as f:
        yaml.dump(ledger, f)

    print(f"✅ Run package created: {run_dir}")
    return run_dir
```

### 公平实验对比的原则 ⭐⭐

在论文或报告中对比不同配置的训练结果时，必须确保对比的公平性：

| 维度 | 公平做法 | 不公平做法 |
|------|---------|-----------|
| 样本量 | 相同 total env steps | 相同 iterations（但每 iter 样本量不同） |
| Seed | 多 seed (≥3) + 标准差 | 单 seed |
| x 轴 | env_steps 或 wall_time | iterations（不同 GPU 数时误导） |
| 基准 | 冻结的 baseline（不随实验修改） | 每次实验都调整 baseline |
| 配置 | 只改一个变量 | 同时改多个变量 |

**结论分级**：不是所有结论都有相同的可信度。

| 层级 | 需要什么 | 可以说什么 |
|------|---------|-----------|
| L1 | smoke test 通过 | "配置可启动" |
| L2 | 单 seed 训练完成 | "reward 有上升趋势" |
| L3 | 3+ seed 完成 + 标准差 | "方法 A 的平均 reward 高于方法 B" |
| L4 | 消融实验 + 统计检验 | "改进来自组件 X（p<0.05）" |
| L5 | 跨任务/机器人验证 | "方法具有泛化性" |

### 失败案例库 ⭐⭐

以下是多 GPU 和大规模训练中的常见失败案例——从中可以学到工程直觉。

**案例 1: GPU id 映射误解**
- 背景：`CUDA_VISIBLE_DEVICES=2,3` 后仍传 `--gpu-ids "[2, 3]"`
- 症状：程序看不到逻辑 id 2 和 3，报设备不可见
- 修复：传 `[0, 1]`
- 教训：日志中 `cuda:0` 不等于物理 GPU 0

**案例 2: 多 GPU 样本量不公平**
- 背景：1 GPU 与 4 GPU 都跑 6000 iterations
- 症状：4 GPU 曲线更好被误写成"算法更优"
- 修复：按目标缩放 `max_iterations`
- 教训：报告中必须写 total env steps 和比较口径

**案例 3: 云端 job 失败仍计费**
- 背景：run step 崩溃后集群保持 UP
- 症状：`sky status` 仍显示运行
- 修复：立刻 `sky down`
- 教训：成本 checklist 必须执行

**案例 4: 每个 rank 都写视频**
- 背景：自定义代码没有 rank 判断
- 症状：视频文件损坏且 I/O 拖慢训练
- 修复：只在 rank 0 创建 recorder
- 教训：所有副作用先问 rank

**案例 5: Sweep agent 抢同一 GPU**
- 背景：多个 agent 手工启动没有限制资源
- 症状：利用率冲突，曲线互相污染
- 修复：每个 agent 绑定独立 GPU
- 教训：agent 数量等于 GPU 数

### 故障分流表 ⭐⭐

| 失败层 | 典型现象 | 第一证据 | 优先处理 |
|--------|---------|---------|---------|
| CLI/配置 | 参数不识别 | stdout 报错信息 | 修命令或 registry |
| GPU 选择 | invalid device | `CUDA_VISIBLE_DEVICES` 值 | 修 `--gpu-ids` |
| torchrunx | rank 未启动 | torchrunx 目录下的 rank 日志 | 修 launch 配置 |
| MuJoCo Warp | 某 rank 崩溃 | rank stderr、NaN dump | 转 §24.3 流程 |
| RSL-RL | all-reduce 报错 | runner 日志 | 查 batch/world size |
| Logger | W&B 登录失败 | W&B stderr | 切 TensorBoard 或修凭据 |
| Cloud | capacity/setup/billing | SkyPilot status/logs | 换资源或 teardown |

**分流能减少无效调参**。云端 capacity 不足不是 PPO 参数问题。W&B 登录失败不是模型问题。Rank 1 的 CUDA crash 不一定能从 rank 0 曲线看出来。

### ⚠️ 常见陷阱

⚠️ **思维陷阱：不记录实验就开始调参**
- "跑了十次实验，第三次效果最好但不记得配置是什么"
- 正确做法：每次实验自动生成运行包，用 WandB 或 Git 追踪

⚠️ **编程陷阱：Checkpoint 名字不匹配**
- Resume 时传 `model_latest.pt`，但 run files 只有 `model_500.pt`
- 正确做法：显式指定存在的 checkpoint 文件名

### 练习

1. **[设计题]** 为一个双 GPU 同总样本量实验创建完整的 `sample_ledger.yaml`。验证单 GPU baseline 和双 GPU variant 的 `total_env_steps` 相等。
2. **[分析题]** 一个报告写"双 GPU 训练更快达到 90% success rate"，但没有 world_size、num_steps_per_env 和 x_axis。列出需要补充的字段，并解释缺少每个字段会导致什么误读。
3. **[实践题]** 使用 `create_run_package()` 为你的一次训练生成完整运行包。检查运行包是否包含复现该训练的所有必要信息。

---

## 24.7 精读：AGILE 四阶段工业级 Workflow ⭐⭐⭐

> **这一节解决什么问题**：AGILE（NVIDIA, arXiv:2603.20147）提出了一个覆盖"准备→训练→评估→部署"全流程的工业级 workflow。它如何组织大规模训练的每个环节？

### AGILE 的四阶段架构 ⭐⭐

AGILE（A Comprehensive Workflow for Humanoid Loco-Manipulation Learning）是 NVIDIA 在 2026 年发布的人形 RL 工程化框架。它不是一个算法——而是一套**标准化的工程流程**，把大规模训练中的每个环节（调试、训练、评估、部署）形式化为可重复的阶段。

```
Stage 1: Prepare (准备)
─────────────────────
├── Joint Position GUI: 逐关节 slider 测试
├── Object Manipulation GUI: 6-DOF 物体交互
├── Reward Visualizer: 逐项 reward 实时叠加
└── 目标: 在几分钟内发现配置 bug

Stage 2: Train (训练)
─────────────────────
├── Git commit + branch + diff 自动记录
├── Docker 化训练环境（可复现）
├── WandB 日志 + checkpoint 管理
├── Scaled-dict 超参搜索
└── 目标: 可复现的训练

Stage 3: Evaluate (评估)
───────────────────────
├── 确定性场景测试（fixed commands）
├── 随机性 rollout（1k envs, randomized）
├── 运动质量诊断:
│   ├── RMS 关节加速度
│   ├── RMS jerk
│   ├── 关节限位违规率
│   └── 高频能量比（>10 Hz 占比）
└── 目标: 自动化的 HTML 评估报告

Stage 4: Deploy (部署)
─────────────────────
├── TorchScript / ONNX 导出
├── 自动生成 YAML descriptor
│   ├── joint_names
│   ├── observation_ordering
│   ├── history_buffer_length
│   └── action_scaling
├── Sim2Sim 验证
└── 真机 C++ controller
```

### AGILE 的算法工具箱 ⭐⭐⭐

AGILE 提供了一系列可开关的 PPO 增强技术——每个都有明确的适用场景和默认参数。

**L2C2（Local Lipschitz Constraint for Continuous Control）**：

```python
# === L2C2: 平滑策略的正则化 ===
# 思想：相邻状态的策略输出应该相似（Lipschitz 约束）
# 实现：在 PPO update 时，对 obs 做微小扰动，要求输出变化也小

def l2c2_loss(policy, obs, obs_next, lambda_pi=1.0, lambda_v=0.1):
    """L2C2 正则化损失。
    
    在 obs_t 和 obs_{t+1} 之间插值，
    要求插值点的 policy 输出和 obs_t 的输出接近。
    """
    alpha = torch.rand(obs.shape[0], 1, device=obs.device)
    obs_interp = obs + alpha * (obs_next - obs)

    # 策略平滑性
    action_t = policy.actor(obs)
    action_interp = policy.actor(obs_interp)
    pi_loss = lambda_pi * (action_interp - action_t).pow(2).mean()

    # 价值函数平滑性（如果 critic 有 privileged obs）
    if hasattr(policy, 'critic'):
        v_t = policy.critic(obs)  # 简化，实际用 privileged obs
        v_interp = policy.critic(obs_interp)
        v_loss = lambda_v * (v_interp - v_t).pow(2).mean()
    else:
        v_loss = 0.0

    return pi_loss + v_loss
```

**在线 Reward 归一化**：

```python
# === AGILE 的 reward normalization ===
# 问题：不同 reward term 的量级差异很大
# 解决：EMA 估计 reward 的标准差，动态归一化

class OnlineRewardNormalizer:
    """AGILE 风格的在线 reward 归一化。"""
    def __init__(self, gamma=0.99, beta=0.999, epsilon=0.01):
        self.sigma = 1.0  # EMA 估计的 reward std
        self.beta = beta
        self.epsilon = epsilon
        # 折扣回报修正因子
        self.phi_gamma = 1.0 / np.sqrt(1 - gamma**2)

    def normalize(self, reward):
        """归一化 reward。"""
        # EMA 更新 sigma
        batch_var = reward.var().item()
        self.sigma = self.beta * self.sigma + (1 - self.beta) * np.sqrt(batch_var)
        # 归一化
        return reward / (self.sigma * self.phi_gamma + self.epsilon)
```

**Virtual Harness（虚拟安全绳）**：

```python
# === Virtual Harness: 训练初期的辅助力 ===
# 思想：在训练初期用外部 PD 力辅助机器人站立，
# 随训练进展逐渐衰减到零——类似婴儿学步器

class VirtualHarness:
    """虚拟安全绳——训练初期的辅助力。"""
    def __init__(self, kp_pos=100, kd_pos=10, kp_rot=50, kd_rot=5,
                 decay_mode="exponential", decay_iters=2000):
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.kp_rot = kp_rot
        self.kd_rot = kd_rot
        self.decay_mode = decay_mode
        self.decay_iters = decay_iters

    def get_scale(self, iteration):
        """计算当前迭代的 harness 强度 s ∈ [0, 1]。"""
        t = min(iteration / self.decay_iters, 1.0)
        if self.decay_mode == "linear":
            return 1.0 - t
        elif self.decay_mode == "exponential":
            return np.exp(t * np.log(0.01))  # 从 1.0 衰减到 0.01
        elif self.decay_mode == "adaptive":
            # 只有当站立比例 > 阈值时才衰减
            return None  # 需要外部 standing_ratio 驱动

    def compute_force(self, env, iteration):
        """计算辅助力/力矩。"""
        s = self.get_scale(iteration)
        if s is None or s < 0.01:
            return torch.zeros_like(env.robot.data.root_link_lin_vel_w)

        # 位置 PD：把 base 拉向目标高度
        pos_error = env.target_height - env.robot.data.root_link_pos_w[:, 2:3]
        vel = env.robot.data.root_link_lin_vel_w[:, 2:3]
        f_z = s * (self.kp_pos * pos_error - self.kd_pos * vel)

        # 姿态 PD：保持直立
        rpy = env.robot.data.root_link_euler_w
        ang_vel = env.robot.data.root_link_ang_vel_w
        tau_rp = s * (self.kp_rot * (-rpy[:, :2]) - self.kd_rot * ang_vel[:, :2])

        return f_z, tau_rp
```

Virtual Harness 对人形站立训练特别有价值——没有它，人形在训练初期几乎立刻摔倒（因为策略还是随机的），产生的数据全是"摔倒"的经历，PPO 无法从中学到"如何站立"。Harness 给予辅助力让人形在初期能保持站立，策略可以在站立状态下探索——随着训练进展，辅助力逐渐衰减，策略必须自己维持平衡。

**Value-Bootstrapped Terminations**：

```python
# === 防"自杀"的 termination 处理 ===
# 问题：terminal state 的 V(s) = 0（按 RL 定义）
# 但如果 termination 是因为摔倒（负 reward 状态），
# 策略可能学到"早点摔倒结束 episode"来避免累积更多负 reward
# AGILE 的解决方案：terminal state 也 bootstrap V(s)

def value_bootstrapped_terminal(reward, value, done, gamma=0.99, sigma=5.0):
    """对 terminal state 做 value bootstrapping。
    
    不把 terminal V(s) 设为 0，而是设为一个大负值
    （通过 value prediction + penalty），阻止策略主动摔倒。
    """
    # 正常 GAE 计算中，done=True 时 V(s') = 0
    # 修改为：done=True 时 V(s') = V_predicted - sigma
    # sigma=5 在 gamma=0.99 下约等于 value space 中的 500
    # 这让"摔倒"的代价远大于"继续活着"
    terminal_penalty = done.float() * sigma
    adjusted_value = value - terminal_penalty
    return adjusted_value
```

### AGILE 的 PPO 默认配置 ⭐⭐

以下是 AGILE 在 Unitree G1 和 Booster T1 上验证过的 PPO 默认配置——可以作为新项目的起点：

| 参数 | AGILE 默认值 | 说明 |
|------|-------------|------|
| Actor 网络 | [256, 256, 128], ELU | 比 [256, 128] 多一层，容量更大 |
| Critic 网络 | [512, 256, 128], ELU | Critic 更大（处理 privileged obs） |
| Learning rate | 1e-3 | 自适应 LR schedule |
| γ (discount) | 0.99（locomotion），0.995（stand-up） | 长时程任务需要更大 γ |
| GAE λ | 0.95 | 标准值 |
| Clip ε | 0.2 | 标准值 |
| Epochs per update | 5 | |
| Mini-batches | 4 | |
| Entropy coeff | 0.005 | 比默认 0.01 略小 |
| Num envs | 4096 | |
| Num steps per env | 24 | |
| Max iterations | ~20k | |

### AGILE 与本书前序章节的对应 ⭐⭐

| AGILE 阶段 | 本书对应 | 章节 |
|------------|---------|------|
| Stage 1: Prepare (Joint GUI) | Zero agent + 逐关节测试 | Ch22 §22.2 |
| Stage 1: Prepare (Reward Viz) | 分项 reward 打印 | Ch06 §06.7 |
| Stage 2: Train (reproducible) | Git hash + WandB 日志 | Ch07 §07.5 |
| Stage 2: Train (scaled-dict) | Reward 权重搜索 | Ch06 §06.7 |
| Stage 3: Evaluate (rollout) | Sim2Sim 验证 | Ch23 §23.5 |
| Stage 3: Evaluate (motion quality) | 行为质量诊断 | Ch25 §25.1 |
| Stage 4: Deploy (export) | ONNX 导出 + metadata | Ch23 §23.3 |
| Stage 4: Deploy (descriptor) | deploy.yaml | Ch23 §23.5 |

### AGILE Stage 3: 运动质量诊断代码 ⭐⭐⭐

AGILE 的 Stage 3 不只看 reward 曲线——它还计算一系列运动质量指标（motion quality metrics），用于判断策略的行为是否适合真机部署：

```python
# === AGILE Stage 3: 运动质量诊断 ===

def evaluate_motion_quality(env, policy, num_episodes=100):
    """AGILE 风格的运动质量评估。"""
    metrics = {
        "rms_joint_acc": [],      # 关节加速度的 RMS
        "rms_jerk": [],           # 关节 jerk (加速度的导数) 的 RMS
        "joint_limit_violations": [],  # 关节角超限次数
        "hf_energy_ratio": [],     # 高频 (>10 Hz) 能量占比
        "tracking_error": [],      # 速度跟踪误差
        "survival_time": [],       # 存活时间
    }

    obs, _ = env.reset()
    ep_step = torch.zeros(env.num_envs, device=env.device)
    prev_vel = torch.zeros_like(env.robot.data.joint_vel)
    prev_acc = torch.zeros_like(env.robot.data.joint_vel)

    for step in range(5000):
        with torch.no_grad():
            action = policy(obs['policy'])
        obs, reward, done, truncated, info = env.step(action)
        ep_step += 1

        # 关节加速度
        curr_vel = env.robot.data.joint_vel
        acc = (curr_vel - prev_vel) / env.dt
        prev_vel = curr_vel.clone()

        # Jerk
        jerk = (acc - prev_acc) / env.dt
        prev_acc = acc.clone()

        # 记录指标
        if step > 10:  # 跳过初始化阶段
            metrics["rms_joint_acc"].append(acc.pow(2).mean(dim=-1).sqrt().mean().item())
            metrics["rms_jerk"].append(jerk.pow(2).mean(dim=-1).sqrt().mean().item())

            # 关节限位违规
            joint_pos = env.robot.data.joint_pos
            lo = env.robot.data.soft_joint_pos_limits[..., 0]
            hi = env.robot.data.soft_joint_pos_limits[..., 1]
            violations = ((joint_pos < lo) | (joint_pos > hi)).float().sum(dim=-1)
            metrics["joint_limit_violations"].append(violations.mean().item())

        # 记录完成的 episode
        finished = done | truncated
        if finished.any():
            for idx in finished.nonzero(as_tuple=True)[0]:
                metrics["survival_time"].append(
                    ep_step[idx].item() * env.dt
                )
            ep_step[finished] = 0

    # 高频能量分析
    acc_array = np.array(metrics["rms_joint_acc"])
    if len(acc_array) > 100:
        # FFT 分析
        from scipy.fft import rfft, rfftfreq
        spectrum = np.abs(rfft(acc_array))
        freqs = rfftfreq(len(acc_array), d=env.dt)
        high_freq_mask = freqs > 10.0  # >10 Hz
        hf_energy = spectrum[high_freq_mask].sum()
        total_energy = spectrum.sum()
        hf_ratio = hf_energy / (total_energy + 1e-8)
        metrics["hf_energy_ratio"] = [hf_ratio]

    # 汇总报告
    print("=" * 60)
    print("  AGILE Stage 3: Motion Quality Report")
    print("=" * 60)
    print(f"  RMS Joint Acceleration: {np.mean(metrics['rms_joint_acc']):.2f} rad/s²")
    print(f"    (< 50 rad/s² 为佳, > 100 可能有高频振荡)")
    print(f"  RMS Jerk: {np.mean(metrics['rms_jerk']):.0f} rad/s³")
    print(f"    (< 500 为佳, > 2000 表示动作不平滑)")
    print(f"  Joint Limit Violations: {np.mean(metrics['joint_limit_violations']):.2f} per step")
    print(f"    (0 为最佳, > 0.5 表示策略不尊重关节限位)")
    if metrics["hf_energy_ratio"]:
        hf = metrics["hf_energy_ratio"][0]
        print(f"  High-Freq Energy Ratio (>10Hz): {hf:.1%}")
        print(f"    (< 10% 为佳, > 30% 表示策略有高频抖动)")
    print(f"  Avg Survival Time: {np.mean(metrics['survival_time']):.1f} s")
    print(f"  Completed Episodes: {len(metrics['survival_time'])}")

    # 判定
    issues = []
    if np.mean(metrics['rms_joint_acc']) > 100:
        issues.append("⚠️ 关节加速度过高 → 增大 action_rate penalty")
    if np.mean(metrics['rms_jerk']) > 2000:
        issues.append("⚠️ Jerk 过高 → 增大 action 平滑惩罚")
    if np.mean(metrics['joint_limit_violations']) > 0.5:
        issues.append("⚠️ 关节限位违规 → 增大 dof_pos_limits penalty")
    if metrics["hf_energy_ratio"] and metrics["hf_energy_ratio"][0] > 0.3:
        issues.append("⚠️ 高频能量占比过高 → 检查 PD gains 和 timestep")

    if issues:
        print("\n  ⚠️ 发现以下运动质量问题：")
        for issue in issues:
            print(f"    {issue}")
    else:
        print("\n  ✅ 运动质量诊断通过")

    return metrics
```

**运动质量判定标准**：

| 指标 | 良好 | 可接受 | 需要修复 |
|------|------|--------|---------|
| RMS 关节加速度 | < 50 rad/s² | 50-100 | > 100 |
| RMS Jerk | < 500 rad/s³ | 500-2000 | > 2000 |
| 关节限位违规 | 0 per step | < 0.1 | > 0.5 |
| 高频能量比 | < 10% | 10-30% | > 30% |

这些指标直接影响 sim2real 的成功率——如果仿真中就有高频振荡或关节限位违规，真机上的表现只会更差（因为真实电机响应比仿真慢）。

> **本质洞察**：AGILE 的核心贡献不是某个新算法，而是把"好的工程实践"标准化为可强制执行的流程。在没有 AGILE 的情况下，一个团队的训练质量取决于最有经验的成员是否在场；有了 AGILE，即使新手也能通过遵循四阶段流程达到接近专家的工程质量。这就是"流程"相对于"经验"的价值——它让质量不再依赖个人。

### AGILE 的对称性增强 ⭐⭐

四足和人形机器人通常具有左右对称性——左前腿和右前腿的结构完全相同（镜像）。策略如果学到了左腿的控制，理论上应该能直接镜像应用到右腿。对称性增强（Symmetry Augmentation）利用这个结构先验，在训练数据中注入镜像样本。

```python
# === 对称性增强 ===

class SymmetryAugmentation:
    """AGILE / WoCoCo 风格的对称性增强。

    在每次 PPO update 前，把 rollout buffer 中的数据
    镜像一份（左右互换），加入训练 batch。
    效果：相当于 2× 数据量 + 强制左右对称行为。
    """
    def __init__(self, obs_mirror_indices, action_mirror_indices,
                 obs_negate_indices=None, action_negate_indices=None):
        """
        Args:
            obs_mirror_indices: obs 中左右互换的维度对
                例如: [(3, 9), (4, 10), (5, 11)] 表示
                dim 3↔9, 4↔10, 5↔11 互换
            action_mirror_indices: action 中左右互换的维度对
            obs_negate_indices: obs 中镜像后需要取反的维度
                例如: lateral velocity 在镜像后方向反转
            action_negate_indices: action 中镜像后需要取反的维度
        """
        self.obs_swap = obs_mirror_indices
        self.act_swap = action_mirror_indices
        self.obs_neg = obs_negate_indices or []
        self.act_neg = action_negate_indices or []

    def mirror_obs(self, obs):
        """镜像 observation。"""
        mirrored = obs.clone()
        for i, j in self.obs_swap:
            mirrored[:, i], mirrored[:, j] = obs[:, j].clone(), obs[:, i].clone()
        for idx in self.obs_neg:
            mirrored[:, idx] *= -1
        return mirrored

    def mirror_action(self, action):
        """镜像 action。"""
        mirrored = action.clone()
        for i, j in self.act_swap:
            mirrored[:, i], mirrored[:, j] = action[:, j].clone(), action[:, i].clone()
        for idx in self.act_neg:
            mirrored[:, idx] *= -1
        return mirrored

    def augment_batch(self, obs_batch, action_batch, reward_batch, **kwargs):
        """把原始 batch 和镜像 batch 拼接。"""
        mirror_obs = self.mirror_obs(obs_batch)
        mirror_action = self.mirror_action(action_batch)
        # reward 不变（左右对称的行为应该获得相同的 reward）
        aug_obs = torch.cat([obs_batch, mirror_obs], dim=0)
        aug_action = torch.cat([action_batch, mirror_action], dim=0)
        aug_reward = torch.cat([reward_batch, reward_batch], dim=0)
        return aug_obs, aug_action, aug_reward

# 使用示例（四足机器人）
# FL/FR 和 HL/HR 的关节索引互换
symmetry = SymmetryAugmentation(
    obs_mirror_indices=[
        (3, 6), (4, 7), (5, 8),    # FL ↔ FR joint pos
        (9, 12), (10, 13), (11, 14),  # HL ↔ HR joint pos
        # ... joint vel 同理 ...
    ],
    action_mirror_indices=[
        (0, 3), (1, 4), (2, 5),    # FL ↔ FR action
        (6, 9), (7, 10), (8, 11),  # HL ↔ HR action
    ],
    obs_negate_indices=[1],  # lateral velocity 取反
    action_negate_indices=[],
)
```

**对称性增强的效果**：在四足 locomotion 中，开启对称性增强通常能：
- 减少步态不对称（左右腿行为一致）
- 加速收敛 15-30%（等效于 2× 数据量）
- 提高 sim2real 成功率（真机的左右腿硬件差异更小于仿真随机性）

**注意事项**：对称性增强假设 reward 对左右对称——如果你的 reward 本身不对称（如"只用右手抓取"），不要开启对称性增强。

### AGILE 的状态缓存训练技巧 ⭐⭐

对于人形 stand-up 任务，策略需要从各种倒地姿态恢复站立。传统方法是在训练中让机器人自己摔倒产生不同的起始姿态——但这很慢，因为"摔倒"本身需要仿真时间。AGILE 的 State Caching 技巧是：

```python
# === State Caching: 预生成多样化初始状态 ===

def generate_diverse_initial_states(env, num_states=10000):
    """通过随机 rollout 预生成多样化的初始状态。

    运行一次随机策略 rollout，在不同时间点保存物理状态，
    之后的训练直接从这些保存的状态 reset——
    不需要每次都重新模拟"摔倒"过程。
    """
    obs, _ = env.reset()
    states = []

    for step in range(num_states * 10):
        # 随机动作 + 外部扰动（产生各种姿态）
        action = torch.randn(env.num_envs, env.action_space.shape[-1],
                            device=env.device) * 2.0
        obs, _, done, _, _ = env.step(action)

        # 每隔 10 步保存一次状态
        if step % 10 == 0:
            state = {
                "qpos": env.robot.data.joint_pos.clone(),
                "qvel": env.robot.data.joint_vel.clone(),
                "root_pos": env.robot.data.root_link_pos_w.clone(),
                "root_quat": env.robot.data.root_link_quat_w.clone(),
            }
            states.append(state)

        if len(states) >= num_states:
            break

    print(f"Generated {len(states)} diverse initial states")
    # 分析姿态分布
    heights = [s["root_pos"][:, 2].mean().item() for s in states]
    print(f"  Height range: [{min(heights):.2f}, {max(heights):.2f}] m")
    print(f"  States with height < 0.3m (fallen): "
          f"{sum(1 for h in heights if h < 0.3)} / {len(states)}")

    return states

# 在训练中使用缓存的状态做 reset
class CachedStateReset:
    """从预缓存的状态中随机选取做 reset。"""
    def __init__(self, cached_states):
        self.states = cached_states

    def reset(self, env, env_ids):
        # 随机选取一个缓存状态
        idx = torch.randint(0, len(self.states), (1,)).item()
        state = self.states[idx]
        # 设置物理状态
        env.robot.write_root_state_to_sim(
            state["root_pos"][env_ids],
            state["root_quat"][env_ids],
        )
        env.robot.write_joint_state_to_sim(
            state["qpos"][env_ids],
            state["qvel"][env_ids],
        )
```

**State Caching 的工程价值**：对于 stand-up 任务，传统方法需要每次 episode 先"摔倒 2 秒"再"站起来 3 秒"——摔倒的 2 秒是浪费的（策略不学习任何东西）。State Caching 直接从倒地状态开始——节省 40% 的仿真时间。

> **本质洞察**：AGILE 不是新算法——而是**把本书前 23 章教过的零散工程最佳实践组织成一条流水线**。如果你已经掌握了 Ch01-Ch23 的内容，AGILE 的每个阶段你都能理解并实现。AGILE 的价值在于**形式化和标准化**——把"好的工程习惯"变成"必须遵循的流程步骤"，从而在团队协作中确保一致性。

### AGILE 的 Scaled-Dict 超参搜索 ⭐⭐

传统的超参搜索对每个参数独立搜索——如果有 10 个 reward 权重，搜索空间是 10 维。AGILE 的 scaled-dict 技巧把搜索空间压缩到 1 维：

```python
# === Scaled-Dict 超参搜索 ===
# 思想：reward 权重之间的相对比例通常是合理的
# 只搜索一个全局缩放因子来调整整组权重

class ScaledDictSearch:
    """把 N 维 reward 权重搜索压缩为 1 维。"""
    def __init__(self, base_weights: dict):
        self.base = base_weights  # 基准权重
        # 例如: {"tracking": 1.5, "regularization": 0.05, "contact": 0.2}

    def apply_scale(self, group_name: str, scale: float) -> dict:
        """对某组权重应用缩放。"""
        scaled = self.base.copy()
        for key in scaled:
            if group_name in key:
                scaled[key] *= scale
        return scaled

# 使用示例：
# 搜索 regularization 组的缩放因子 ∈ [0.5, 2.0]
# 而非独立搜索 action_rate, torque, acc 各自的权重
searcher = ScaledDictSearch({
    "tracking_lin": 1.5, "tracking_ang": 0.75,
    "reg_action_rate": 0.01, "reg_torque": 0.0002, "reg_acc": 2.5e-7,
    "contact_air_time": 0.2, "contact_slip": 1.0,
})
```

### ⚠️ 常见陷阱

⚠️ **概念误区：把 AGILE 当作"又一个 RL 框架"**
- AGILE 不提供 env.step()、不实现 PPO——它是工程流程的组织框架
- 它在 Isaac Lab 之上运行，使用 RSL-RL 做训练

⚠️ **思维陷阱：跳过 Stage 1 (Prepare) 直接 Stage 2 (Train)**
- "我的 reward 应该没问题，直接训练吧"——AGILE 的经验表明，90% 的训练失败可以在 Stage 1 的 5 分钟 GUI 检查中提前发现

### 练习

1. **[实践题]** 用 AGILE 的 PPO 默认配置（见上表）训练一个四足速度跟踪任务 5000 iterations，和你之前使用的配置对比 reward 曲线。
2. **[设计题]** 为一个人形站立任务实现 Virtual Harness。设计衰减曲线：前 1000 iterations 保持 harness，然后 exponential 衰减到 3000 iterations 时为零。
3. **[分析题]** 为什么 Value-Bootstrapped Terminations 能防止策略"自杀"？用一个简化的 2-state MDP 例子说明：如果 terminal V(s)=0，策略如何利用这一点来避免负 reward。

---

## 24.8 训练诊断预览：reward 曲线之外的五个信号 ⭐⭐

> **这一节解决什么问题**：当 reward 曲线"看起来还行"但行为不对时，还应该看什么指标？

### 动机

"reward 在涨"是一个必要但不充分的信号——策略可能在 reward hacking（找到了不合理但高 reward 的行为）、action 在饱和（输出都在 ±1 附近）、entropy 塌缩（策略变成了确定性的，不再探索）、value function 不准（GAE 的 baseline 估计偏差大）。这些问题不会让 reward 下降，但会让部署失败。

### 五个信号的联合阅读 ⭐⭐

```python
# === PPO 训练诊断五信号联合监控 ===

def log_diagnostics(runner, iteration):
    """记录五个关键诊断信号到 WandB。"""
    stats = runner.get_training_stats()

    diagnostics = {
        # 1. Reward（任务完成度）
        "diag/reward_mean": stats["reward_mean"],
        "diag/reward_std": stats["reward_std"],

        # 2. KL divergence（策略更新幅度）
        "diag/kl_divergence": stats["kl"],
        # 正常: 0.005-0.02, 异常: >0.05 (LR 太大) 或 ≈0 (LR 太小)

        # 3. Entropy（探索程度）
        "diag/entropy": stats["entropy"],
        # 正常: 逐渐下降但不陡降
        # 异常: 突然从 3.0 降到 0.5 → mode collapse

        # 4. Value loss（critic 精度）
        "diag/value_loss": stats["value_loss"],
        # 正常: 先升后降，最终稳定
        # 异常: 持续上升 → critic 无法拟合 value function

        # 5. Episode length（行为质量）
        "diag/episode_length_mean": stats["ep_len_mean"],
        # 正常: 逐渐增长（活得更久）
        # 异常: 始终很短 → termination 太严格 或 初始化有问题
    }

    if runner.logger:
        runner.logger.log(diagnostics, step=iteration)

    return diagnostics
```

**五信号联合诊断表**：

| 症状组合 | 可能原因 | 修复方向 |
|---------|---------|---------|
| reward↑ + entropy 陡降 | Mode collapse，策略找到固定解 | 增大 entropy coeff |
| reward 平坦 + KL≈0 | LR 太小，策略几乎不更新 | 增大 LR |
| reward 平坦 + KL>0.05 | LR 太大，每步更新过大导致振荡 | 减小 LR |
| reward↑ + value_loss 持续升 | Critic 拟合不了（网络太小或 obs 不够） | 增大 critic 网络 |
| reward↑ + ep_len 不变 | 可能是 reward hacking | 看 viewer 确认行为 |
| reward↓ + ep_len 短 | 训练崩溃 → NaN 或 termination 突变 | 转 §24.3 NaN 排查 |
| reward↑ + action 饱和 (>90% 在 ±1) | Action scale 太小或探索不足 | 增大 action_scale 或 init_noise |

### Action 饱和检测 ⭐⭐

Action 饱和（策略输出持续接近 ±1）是一个常被忽视但影响严重的问题——饱和意味着策略被"困在角落"，无法精细调节动作。

```python
# === Action 饱和检测 ===
def check_action_saturation(env, policy, num_steps=500):
    """检测策略输出是否饱和。"""
    obs, _ = env.reset()
    all_actions = []

    for _ in range(num_steps):
        with torch.no_grad():
            action = policy(obs['policy'])
        all_actions.append(action)
        obs, _, _, _, _ = env.step(action)

    actions = torch.cat(all_actions, dim=0)  # [N*B, act_dim]
    abs_actions = actions.abs()

    # 统计各维度的饱和率（|a| > 0.9 的比例）
    saturation_rate = (abs_actions > 0.9).float().mean(dim=0)
    print("Action saturation rate per dimension:")
    for i, rate in enumerate(saturation_rate):
        status = "⚠️" if rate > 0.5 else "✅"
        print(f"  dim {i}: {rate:.1%} {status}")

    overall = saturation_rate.mean().item()
    print(f"\nOverall saturation: {overall:.1%}")
    if overall > 0.3:
        print("⚠️ 策略输出过度饱和！")
        print("  可能原因: action_scale 太小，策略需要更大的动作幅度")
        print("  修复: 增大 action_scale 或增大 init_noise_std")
    return saturation_rate
```

### Reward Hacking 检测 ⭐⭐

Reward hacking 是策略找到了一种"技术上"满足 reward 定义但"物理上"不合理的行为——例如通过快速抖动来获得高 tracking reward（因为抖动的平均速度恰好接近目标速度），或者通过把脚卡在地面裂缝中来获得稳定的 air_time reward。

**检测方法**：定期用 viewer 观察行为，或用 §24.7 的运动质量诊断自动检测。

```python
# === Reward Hacking 检测 ===
def detect_reward_hacking(env, policy, num_steps=1000):
    """通过对比 reward 和行为质量检测 reward hacking。"""
    obs, _ = env.reset()
    rewards = []
    tracking_errors = []
    action_rates = []
    prev_action = None

    for step in range(num_steps):
        with torch.no_grad():
            action = policy(obs['policy'])
        obs, reward, _, _, _ = env.step(action)
        rewards.append(reward.mean().item())

        # 实际跟踪误差（不通过 reward 函数）
        actual_vel = env.robot.data.root_link_lin_vel_b[:, :2]
        cmd_vel = env.command_manager.get_command("base_velocity")[:, :2]
        error = (actual_vel - cmd_vel).norm(dim=-1).mean().item()
        tracking_errors.append(error)

        # Action rate
        if prev_action is not None:
            rate = (action - prev_action).norm(dim=-1).mean().item()
            action_rates.append(rate)
        prev_action = action.clone()

    # 如果 reward 高但 tracking error 也高 → reward hacking
    avg_reward = np.mean(rewards)
    avg_error = np.mean(tracking_errors)
    avg_rate = np.mean(action_rates) if action_rates else 0

    print(f"Avg reward: {avg_reward:.3f}")
    print(f"Avg tracking error: {avg_error:.3f} m/s")
    print(f"Avg action rate: {avg_rate:.4f}")

    if avg_reward > 0.5 and avg_error > 0.3:
        print("⚠️ 可能的 reward hacking: reward 高但 tracking 差")
        print("  检查 reward 定义是否和实际任务目标一致")
    if avg_rate > 0.3:
        print("⚠️ 动作抖动频率过高")
        print("  增大 action_rate_l2 penalty")
```

### 从诊断到调参的决策流程 ⭐⭐

当训练"看起来不太对"时，按以下决策树行动：

```
训练表现不符合预期
│
├── Reward 在涨吗？
│   ├── 不涨（平坦）
│   │   ├── KL ≈ 0 → LR 太小，增大
│   │   ├── KL > 0.05 → LR 太大，减小
│   │   ├── Entropy 很低 → Mode collapse，增大 entropy coeff
│   │   └── Episode length 很短 → Termination 太严格，放宽
│   │
│   └── 在涨
│       ├── 行为合理吗？（viewer 检查）
│       │   ├── 合理 → 继续训练
│       │   └── 不合理 → Reward hacking
│       │       ├── 动作抖动 → 增大 action_rate penalty
│       │       ├── 利用仿真 bug → 修复物理配置
│       │       └── 忽略命令 → 检查 command obs 是否正确
│       │
│       ├── Action 饱和 > 30%？
│       │   └── 是 → 增大 action_scale
│       │
│       └── Value loss 持续升？
│           └── 是 → Critic 网络太小，增大隐藏层
│
├── 有 NaN 吗？
│   └── 是 → 转 §24.3 NaN 排查
│
└── 太慢了？
    └── 是 → 转 §24.4 性能优化
```

> **跨领域类比**：训练诊断就像医生的"查房"——不是只看血压（reward），还要看心率（KL）、体温（entropy）、血氧（value loss）和意识状态（episode length）。五个指标联合阅读才能得出正确的诊断。只看 reward 曲线做决策，就像只看血压就开药一样危险。

### ⚠️ 常见陷阱

⚠️ **思维陷阱：reward 在涨就不看其他指标**
- reward 涨但 entropy 陡降 = 策略找到了一个固定解不再探索
- 正确做法：五个信号联合阅读

⚠️ **思维陷阱：性能不好先换算法**
- "PPO 太慢，换 SAC"——更可能的原因是 env step 慢或 reward 设计不好
- 正确做法：先 profile，区分是 env 慢还是 PPO 慢

### 练习

1. **[实践题]** 对你训练中的策略运行 `check_action_saturation()`。哪些 action 维度饱和了？这和你的 action_scale 设置有关吗？
2. **[分析题]** 设计一个 WandB Dashboard，同时显示五个诊断信号。描述你会如何在 Dashboard 中布局这些图表，以及什么"模式"表示训练健康。
3. **[跨章综合题]** 结合 Ch06 的 reward 设计和 §24.7 的 AGILE 运动质量诊断，设计一个自动化的"训练健康检查"脚本——每 1000 iterations 自动运行并生成报告。

---

> **下一章预告**：Ch25 将深入训练诊断——把本节初步介绍的五信号联合阅读发展为覆盖全书所有任务的"症状→调参"索引表。Ch25 的贡献是：当你遇到任何训练问题时，查表就能找到对应的修复方向——而不需要从头推理。AGILE 的 Stage 3 评估报告（§24.7 的运动质量诊断）和本节的五信号诊断表都是 Ch25 索引表的输入。

> **全书定位**：Ch24 是 Part VI（大规模训练与调试）的第一章。它解决的是"如何高效、稳定、可复现地训练"——从多 GPU 配置到 NaN 排查到性能优化到云端训练到 AGILE 工业级流程。Ch25 紧接其后解决"如何诊断和修复训练问题"——两章合在一起构成了"训练工程"的完整工具箱。

## 本章小结

| 知识点 | 核心内容 | 对应练习/实战 |
|--------|---------|-------------|
| 多 GPU 配置 | mjlab torchrunx `--gpu-ids` / Isaac Lab torchrun | §24.2 |
| 样本量核算 | `total = envs × steps × gpus × iters` | §24.2 公平比较 |
| 单写者原则 | 只 rank 0 写文件 | §24.2 代码模板 |
| NaN 五大根因 | 接触发散/除零/normalizer/std 负/graph 失效 | §24.3 排查表 |
| NaN Guard | `--enable-nan-guard` + `viz-nan` | §24.3 代码 |
| 性能 Profiling | torch.profiler + nsys + 传感器成本量化 | §24.4 代码 |
| nconmax/njmax 调优 | 记录实际分布 → 设为 1.5× max | §24.4 代码 |
| num_envs 选择 | 逐步增大找吞吐峰值 | §24.4 代码 |
| SkyPilot 云端 | launch → run → down 生命周期 | §24.5 |
| WandB Sweep | 自动化超参搜索 | §24.5 代码 |
| AGILE Prepare | Joint GUI + Reward Viz（5 分钟发现 bug） | §24.7 |
| AGILE Train | Git hash + Docker + scaled-dict | §24.7 |
| AGILE Evaluate | 确定性+随机性 rollout + HTML 报告 | §24.7 |
| AGILE Deploy | TorchScript + YAML descriptor | §24.7 |
| L2C2 | 相邻状态 policy 输出平滑正则 | §24.7 代码 |
| Reward 归一化 | EMA σ + 折扣修正 | §24.7 代码 |
| Virtual Harness | 训练初期辅助力 + 衰减 | §24.7 代码 |
| Value-Bootstrap | 防止策略"自杀" | §24.7 代码 |

---

## 累积项目：本章新增模块

| # | 模块 | 描述 | 依赖 |
|---|------|------|------|
| 1 | 多 GPU 训练脚本 | 带样本量核算的 `--gpu-ids` 配置 | §24.2 |
| 2 | NaN 排查工具链 | `--enable-nan-guard` + dump 分析脚本 | §24.3 |
| 3 | 性能 Benchmark 脚本 | env_sps / train_sps / VRAM 自动测量 | §24.4 |
| 4 | nconmax 调优工具 | 记录接触数分布 + 推荐值 | §24.4 |
| 5 | SkyPilot 训练配置 | sky_train.yaml 模板 | §24.5 |
| 6 | AGILE 工具箱 | L2C2 / reward norm / virtual harness / value bootstrap | §24.7 |

---

## 延伸阅读

| 资料 | 难度 | 说明 |
|------|------|------|
| AGILE（arXiv:2603.20147） | ⭐⭐⭐ | 四阶段工业级 workflow + 算法工具箱 |
| torchrunx（github.com/apoorvkh/torchrunx） | ⭐⭐ | mjlab 使用的分布式启动器 |
| PyTorch DDP 文档 | ⭐⭐ | 理解 all-reduce 和进程组 |
| SkyPilot 文档 | ⭐⭐ | 云端任务生命周期和成本管理 |
| WandB Sweep 文档 | ⭐ | 超参搜索配置 |
| RSL-RL（arXiv:2509.10771） | ⭐⭐ | EmpiricalNormalization + PPO 实现 |
| MuJoCo Warp 文档 | ⭐⭐ | CUDA Graph capture + nconmax/njmax |
| NVIDIA OSMO | ⭐⭐⭐ | Isaac Lab 的生产级多节点编排 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| 训练后突然 NaN | 接触/reward/normalizer | 1. nan_guard 2. viz-nan 3. 按 §24.3 优先级 | §24.3 |
| 多 GPU 训练但 reward 曲线重复 | 多 rank 都写了日志 | 1. 检查 rank 判断 2. 确认只 rank 0 写 | §24.2 |
| `--gpu-ids` 报设备不可见 | CUDA_VISIBLE_DEVICES 冲突 | 1. 传逻辑 id [0,1] 而非物理 id | §24.2 |
| steps/s 远低于预期 | sensor/nconmax/num_envs | 1. 关闭 sensor 对比 2. 调 nconmax 3. 找最优 num_envs | §24.4 |
| GPU 利用率 < 50% | num_envs 太小 | 1. 增大 num_envs 2. profiler 检查 | §24.4 |
| SkyPilot job 失败仍计费 | 未 `sky down` | 1. `sky status` 2. `sky down` | §24.5 |
| Sweep agent 抢同一 GPU | 未绑定 CUDA_VISIBLE_DEVICES | 1. 每个 agent 绑定独立 GPU | §24.5 |
| Curriculum 推进后 NaN | 新 reward term 有除零风险 | 1. 固定 curriculum 两侧各跑 100 iter | §24.3 |
| Checkpoint resume 后行为不同 | normalizer 未恢复 | 1. 检查 state_dict 内容 2. 加载 normalizer | §24.2 |
| CUDA Graph 失效警告 | DR 改变了数组地址 | 1. 确认 create_graph() 被调用 | §24.3 |

---


### 训练诊断的完整自动化脚本 ⭐⭐

以下脚本整合了五信号监控、action 饱和检测、reward hacking 检测和运动质量诊断，可以在训练过程中每 1000 iterations 自动运行：

```python
# === 训练健康检查自动化 ===

class TrainingHealthChecker:
    """训练过程中的自动化健康检查。
    
    集成五信号监控 + action 饱和 + reward hacking + 运动质量，
    每隔 check_interval iterations 自动运行。
    """
    def __init__(self, env, policy, check_interval=1000, logger=None):
        self.env = env
        self.policy = policy
        self.interval = check_interval
        self.logger = logger
        self.history = []

    def check(self, iteration, training_stats):
        """运行完整的健康检查并返回报告。"""
        if iteration % self.interval != 0:
            return None

        report = {"iteration": iteration, "issues": []}

        # 1. 五信号诊断
        report["reward_mean"] = training_stats.get("reward_mean", 0)
        report["kl"] = training_stats.get("kl", 0)
        report["entropy"] = training_stats.get("entropy", 0)
        report["value_loss"] = training_stats.get("value_loss", 0)
        report["ep_len_mean"] = training_stats.get("ep_len_mean", 0)

        if report["kl"] > 0.05:
            report["issues"].append("KL 过高 → 减小 LR")
        if report["entropy"] < 0.5:
            report["issues"].append("Entropy 过低 → 增大 entropy_coef")

        # 2. Action 饱和检测（采样 200 步）
        obs, _ = self.env.reset()
        abs_actions = []
        for _ in range(200):
            with torch.no_grad():
                action = self.policy(obs['policy'])
            abs_actions.append(action.abs())
            obs, _, _, _, _ = self.env.step(action)
        saturation = torch.cat(abs_actions).gt(0.9).float().mean().item()
        report["action_saturation"] = saturation
        if saturation > 0.3:
            report["issues"].append(f"Action 饱和 {saturation:.0%} → 增大 action_scale")

        # 3. 汇总
        status = "✅ 健康" if not report["issues"] else "⚠️ 需要关注"
        print(f"\n[Iter {iteration}] 训练健康检查: {status}")
        for issue in report["issues"]:
            print(f"  {issue}")

        if self.logger:
            self.logger.log({
                "health/saturation": saturation,
                "health/num_issues": len(report["issues"]),
            }, step=iteration)

        self.history.append(report)
        return report

# 在训练循环中使用
checker = TrainingHealthChecker(env, policy, check_interval=1000)
for iteration in range(max_iterations):
    stats = runner.train_one_iteration()
    checker.check(iteration, stats)
```

### 从"单次训练"到"批量实验"的工程升级路径 ⭐

本章教授的工具按使用阶段可以分为三个层次：

| 层次 | 工具 | 什么时候用 |
|------|------|-----------|
| **L1 单次训练** | NaN Guard, profiler, num_envs 调优 | 每次训练都用 |
| **L2 对比实验** | 样本量账本, 公平比较, 多 seed | 写论文/做消融时用 |
| **L3 生产级训练** | AGILE workflow, 运行包, SkyPilot, OSMO | 团队协作/大规模训练时用 |

新手应该从 L1 开始——确保单次训练稳定、高效。当你需要做消融实验或对比不同配置时，升级到 L2——确保比较的公平性。当你在团队中工作或需要大规模训练时，升级到 L3——确保结果的可复现性和流程的标准化。

> **跨领域类比**：这三个层次就像软件开发中的"个人脚本 → 单元测试 → CI/CD 流水线"的升级路径。个人脚本能跑就行（L1），单元测试确保功能正确（L2），CI/CD 确保团队协作中的质量一致性（L3）。不需要一开始就搭建完整的 CI/CD——但当团队和项目规模增长时，这些基础设施是必要的。

### 典型训练时间参考 ⭐

以下是不同任务和配置的典型训练时间参考（单 A100，4096 envs）：

| 任务 | DOF | Iterations | Wall-clock | 说明 |
|------|-----|-----------|-----------|------|
| 四足 flat velocity | 12 | 5,000 | ~1 hr | 基础任务 |
| 四足 rough velocity | 12 | 10,000 | ~2 hr | + 地形 curriculum |
| 人形 flat velocity | 29 | 15,000 | ~4 hr | 更大的状态空间 |
| 人形 stand-up | 29 | 20,000 | ~6 hr | 需要 virtual harness |
| 人形 motion imitation | 29 | 20,000 | ~6 hr | + 参考动作数据 |
| 操作 lift cube | 7 | 5,000 | ~1 hr | 固定基座 |
| Loco-manipulation | 19 | 20,000 | ~8 hr | 底盘 + 臂 |
| 网球击球 | 29 | 30,000+ | ~12+ hr | 多阶段 curriculum |

这些时间参考帮助你判断"训练是不是太慢了"——如果你的四足 flat 训练需要 8 小时（而参考是 1 小时），说明性能有问题，应该用 §24.4 的 profiling 工具定位瓶颈。

---

> **下一章预告**：Ch25 将深入训练诊断——把 §24.8 初步介绍的五信号联合阅读发展为覆盖全书所有任务的"症状→调参"索引表。AGILE 的 Stage 3 评估报告和 §24.8 的五信号诊断表都是 Ch25 索引表的输入。

> **全书定位**：Ch24 是 Part VI（大规模训练与调试）的第一章。它解决的是"如何高效、稳定、可复现地训练"——从多 GPU 配置到 NaN 排查到性能优化到云端训练到 AGILE 工业级流程。Ch25 紧接其后解决"如何诊断和修复训练问题"——两章合在一起构成了"训练工程"的完整工具箱。


### 快速定位决策树（可打印版） ⭐

遇到训练问题时，按此决策树在 5 分钟内定位到正确的排查方向：

```
训练出了问题
│
├── 崩溃了（NaN / OOM / CUDA Error）？
│   ├── NaN in reward → §24.3 根因 2：检查除零
│   ├── NaN in qvel → §24.3 根因 1：接触求解器
│   ├── NaN in obs → §24.3 根因 3：normalizer 预热
│   ├── NaN in log_prob → §24.3 根因 4：policy std
│   ├── OOM → 降低 num_envs 或 nconmax（§24.4）
│   ├── CUDA Error → CUDA Graph 失效（§24.3 根因 5）
│   └── 不确定 → `--enable-nan-guard True` 重跑
│
├── 太慢了（steps/s 低于预期）？
│   ├── 有 sensor？→ 关闭 sensor 测 env_sps 差异（§24.4）
│   ├── GPU 利用率低？→ 增大 num_envs（§24.4）
│   ├── PPO update 慢？→ 减小网络或 mini_batch（§24.4）
│   └── 不确定？→ torch.profiler / nsys 定位（§24.4）
│
├── 不收敛（reward 不涨）？
│   ├── KL ≈ 0 → LR 太小（§24.8）
│   ├── KL > 0.05 → LR 太大（§24.8）
│   ├── Entropy 陡降 → Mode collapse（§24.8）
│   ├── Ep_len 很短 → Termination 太严格
│   └── 不确定？→ 五信号联合阅读（§24.8）
│
├── 收敛了但行为不对？
│   ├── 原地抖动 → 缺 action_rate penalty
│   ├── Action 饱和 → action_scale 太小（§24.8）
│   ├── Reward hacking → 检查 reward 定义（§24.8）
│   └── 不确定？→ AGILE 运动质量诊断（§24.7）
│
├── 多 GPU 问题？
│   ├── 设备不可见 → GPU id 映射（§24.2）
│   ├── 曲线重复 → 多 rank 写日志（§24.2）
│   ├── Rank 崩溃 → 检查所有 rank 日志（§24.2）
│   └── Resume 异常 → normalizer 未恢复（§24.2）
│
└── 结果不可复现？
    ├── 缺 seed → 加 seed 参数
    ├── 缺配置 → 运行包协议（§24.6）
    └── 比较不公平 → 样本量账本（§24.6）
```

这棵决策树覆盖了本章 90% 以上的故障场景。把它打印贴在屏幕旁边——每次训练出问题时，从根节点开始走，通常 2-3 步就能找到正确的排查方向。

### Ch24 与全书的关系图 ⭐

```
Ch01-03  仿真基础设施
  ↓
Ch04-10  RL 工程方法论
  ↓
Ch11-12  机器人建模
  ↓
Ch13-21  单/复合形态实战
  ↓
Ch22     DIY 自定义环境（环境搭建）
Ch23     Sim2Real 部署（部署链路）
  ↓
Ch24 ←── 你在这里（训练工程）
  │
  │   解决: 如何高效、稳定、可复现地训练
  │   工具: NaN Guard + Profiler + AGILE + 运行包
  │
  ↓
Ch25     训练诊断（调参地图）
  ↓
Ch26-28  网球机器人综合项目
```

Ch24 是 Part I-V 所有知识的"工程化整合"——你在前面学到的 obs/reward/DR/distillation 等技术，到了大规模训练阶段会面临新的工程挑战（NaN、性能瓶颈、实验管理）。Ch24 提供了应对这些挑战的系统化方法论。


### 本章核心方法论总结

本章可以浓缩为三条核心方法论：

**方法论一：分层诊断。** 无论是 NaN、性能问题还是训练不收敛，都不要直接猜测原因——先用工具定位问题层级（物理层？算法层？系统层？），再针对该层排查。NaN Guard 定位到物理层还是算法层；profiler 定位到 sensor 层还是 PPO 层；五信号诊断定位到 reward 设计还是超参设置。

**方法论二：先缩小再放大。** 在多 GPU 上出 NaN？先缩到单 GPU 16 envs 复现。训练太慢？先关掉所有 sensor 测基础吞吐。结果不可复现？先固定单 seed 确认基线。从最小的复现范围开始排查，确认后再扩展。

**方法论三：流程标准化。** AGILE 的核心价值不是某个算法，而是把"经验"变成"流程"。运行包协议确保结果可追溯，样本量账本确保比较公平，四阶段 workflow 确保每个环节都不被跳过。流程的力量在于：它让质量不再依赖于"最有经验的人是否在场"。

这三条方法论不仅适用于机器人 RL 训练——它们是任何复杂工程系统的通用调试方法论。分层诊断对应软件工程中的"从日志定位模块"，先缩小再放大对应"最小复现用例"，流程标准化对应"CI/CD 和代码审查"。掌握了这些方法论，你面对任何训练问题都不会"无从下手"——你总有一个系统化的排查起点。


---

> **致读者**：本章涵盖的工具和方法论是你在 Ch13-Ch23 实战中可能已经"凭直觉"在使用的——比如"训练 NaN 了就减小 action scale"、"太慢了就增大 num_envs"。本章的价值在于把这些直觉**系统化**——给出完整的排查优先级表、profiling 工具链、和标准化流程。当直觉失效时（比如减小 action scale 后仍然 NaN），你可以回到本章的决策树，按系统化的步骤逐一排查——而不是继续猜测。

> **写给第二次阅读本章的读者**：第一次阅读本章时，你可能只需要 §24.3（NaN 排查）和 §24.4（性能优化）——因为这是你在训练中最常遇到的问题。当你开始做论文实验时，回来看 §24.6（实验管理）和 §24.8（训练诊断）。当你在团队中协作或使用云端 GPU 时，回来看 §24.2（多 GPU）、§24.5（SkyPilot）和 §24.7（AGILE）。本章是一个**工具箱**——不需要一次性全部掌握，需要什么查什么。

---

> **从 Part V 到 Part VI 的过渡**：Part V（Ch19-Ch23）教你"怎么搭建和部署"，Part VI（Ch24-Ch25）教你"怎么高效和诊断"。两者合在一起构成了完整的机器人 RL 工程能力——从环境搭建到策略训练到部署验证到问题诊断。从 Ch26 开始的 Part VII（网球项目）将在一个综合案例中同时使用 Part V 和 Part VI 的所有工具。


### 本章命令速查表 ⭐

以下是本章涉及的所有关键命令，按使用场景分组：

```bash
# === NaN 排查 ===
uv run train <TASK> --enable-nan-guard True --env.scene.num-envs 16 --agent.max-iterations 100
uv run viz-nan /tmp/mjlab/nan_dumps/nan_dump_latest.npz

# === 性能 Benchmark ===
time uv run play <TASK> --agent random --num-envs 4096 --max-steps 1000 --no-render
nsys profile --trace=cuda,nvtx uv run train <TASK> --agent.max-iterations 5 --headless

# === 多 GPU 训练 ===
# mjlab
uv run train <TASK> --gpu-ids "[0, 1]" --env.scene.num-envs 4096
# Isaac Lab
torchrun --nproc_per_node=2 -m isaaclab.app --task <TASK> --num_envs 4096 --headless

# === 云端训练 ===
sky launch sky_train.yaml
sky status
sky logs <JOB_NAME>
sky down <JOB_NAME>  # 重要！

# === WandB Sweep ===
wandb sweep sweep.yaml
CUDA_VISIBLE_DEVICES=0 wandb agent <SWEEP_ID> &
CUDA_VISIBLE_DEVICES=1 wandb agent <SWEEP_ID> &
```

