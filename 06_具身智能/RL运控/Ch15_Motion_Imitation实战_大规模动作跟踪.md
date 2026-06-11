# Ch15 | Motion Imitation 实战：大规模动作跟踪

> **本章定位**：Part IV（单形态实战）第三章。Ch13 建立了四足速度跟踪，Ch14 把它迁移到人形。但速度跟踪只告诉机器人"走多快"——它不能指定**怎么走**。本章从 velocity tracking 切换到 motion imitation：给定一个参考运动序列（来自 MoCap 或视频），策略学习忠实地复现这个动作。这是从"让机器人走路"到"让机器人像人一样动"的关键跨越。
>
> **参考**：🔧 mjlab tracking (BeyondMimic) · ✅ ProtoMotions · ✅ g1_spinkick · ✅ PHC（ICCV'23）· ✅ KungfuBot（NeurIPS'25）
>
> **机器人**：G1 · **累积项目**：**C**

---

## 前置自测

📋 **答不出 ≥ 3 题 → 先回前置章节复习**

> 本章直接依赖 Ch14 的人形控制经验。如果你还没有在 Ch14 中跑通 G1 的 velocity task 并理解 per-joint action scale 和 variable posture，强烈建议先完成。

1. **[Ch14]** G1 的 29 个 actuated joints 分为哪几组？per-joint action scale 的物理公式是什么？为什么不能统一设置？
2. **[Ch14]** variable posture reward 与传统 pose reward 的区别是什么？它对步态有什么影响？
3. **[Ch09]** Teacher-Student 蒸馏中，teacher 和 student 的 observation 分别看到什么？蒸馏 loss 是什么？
4. **[Ch06]** exponential kernel reward $r = \exp(-\|x\|^2/\sigma^2)$ 中 $\sigma$ 的物理含义是什么？$\sigma$ 过小或过大会怎样？
5. **[动力学]** 给定一个参考运动序列（关节角度随时间变化），如何判断这个序列是否在目标机器人上物理可行？需要检查哪些量？

## 本章目标

学完本章后，你应该能够：

1. **用 mjlab BeyondMimic 管线从零跑通 G1 motion tracking task**：从 motion 数据准备（CSV → NPZ）、WandB 注册、reward 配置到训练和评估
2. **用 ProtoMotions 在 Isaac Lab 中跑通 AMP/ASE/CALM**：理解算法切换只需修改实验配置文件（~30 行差异）
3. **理解大规模 motion tracking 的工程挑战**：142K motions 的 per-GPU 分片、adaptive sampling 的动机和实现
4. **解释 PHC 的 Progressive Neural Networks 如何避免灾难性遗忘**：为什么单个 MLP 无法扩展到数千个动作
5. **精读 KungfuBot 的 physics filter + bi-level optimization**：理解如何从视频中筛选物理可行动作并自适应跟踪难度
6. **在双框架中对比 motion tracking 的配置差异和训练表现**

---

Ch14 中的 velocity task 给 G1 一个速度命令 $(v_x, v_y, \omega_z)$，策略自由决定如何走。Motion imitation 给策略一个完整的参考运动——每一帧每个关节的目标位置——策略必须忠实复现。这是一个更强的约束，但也提供了更丰富的学习信号。

## 15.1 在 mjlab 中跑通 G1 Tracking Task ⭐⭐⭐

> **这一节解决什么问题**：用 mjlab 的 BeyondMimic 管线从零完成一个 G1 motion tracking task——从数据准备到训练到评估。

### Velocity Tracking vs Motion Tracking：MDP 的本质区别

| 维度 | Velocity Tracking (Ch14) | Motion Tracking (Ch15) |
|------|------------------------|----------------------|
| **Command** | $(v_x, v_y, \omega_z)$ 三维 twist | 参考运动序列（全身关节角度 + root 位置/朝向） |
| **Command 维度** | 3 | ~100+（取决于 body 数量和维度） |
| **Reward 核心** | $\exp(-\|v_{cmd} - v_{actual}\|^2)$ | $\exp(-\frac{1}{N}\sum_i\|p_i^{ref} - p_i^{sim}\|^2)$ |
| **时间对齐** | 无（command 在 episode 内固定或定期重采样） | 有（策略必须在正确时间做正确动作） |
| **数据需求** | 无（只需 command range） | 参考运动文件（MoCap/retarget/video） |
| **自由度** | 高（策略自己决定步态） | 低（必须跟踪参考） |
| **典型用途** | 导航、速度控制 | 技能学习、动作复现、风格控制 |

**跨领域类比**：velocity tracking 就像告诉司机"以 60km/h 的速度开"——司机自己决定打方向盘的方式。motion tracking 就像给司机一条精确的轨迹——"在第 3 秒转方向盘 30°，第 5 秒踩刹车到 50%"。后者更难学（必须精确跟踪），但学会后可以执行更精细的操作。

### 数据管线：从原始动作到训练数据

mjlab 的 BeyondMimic 管线使用以下数据流：

```
原始动作数据
  ├── MoCap (.bvh/.c3d) → retarget → CSV
  ├── AMASS (.npz) → SMPL → retarget → CSV
  └── 视频 → 4DHumans/WHAM → SMPL → retarget → CSV
         ↓
    csv_to_npz.py (帧率对齐 + 格式转换)
         ↓
    motion.npz (mjlab 可读格式)
         ↓
    WandB Registry (版本管理 + 团队共享)
         ↓
    uv run train Mjlab-Tracking-Flat-Unitree-G1 --registry-name ...
```

**csv_to_npz.py 的关键参数**：

```bash
# --input-csv 输入 CSV，--output-npz 输出 NPZ
# --input-fps 原始帧率，--output-fps 目标帧率（匹配训练频率），--render 可选渲染预览
# 注意：bash 续行反斜杠必须在行尾，反斜杠后不能再跟注释，否则续行失效
python scripts/tracking/csv_to_npz.py \
    --input-csv g1_walking.csv \
    --output-npz g1_walking.npz \
    --input-fps 30 \
    --output-fps 50 \
    --render
```

**为什么要对齐帧率？** mjlab 的 tracking task 以 50 Hz 运行（physics_dt × decimation = 0.002 × 10 = 0.02s = 50Hz）。如果参考运动是 30 Hz，直接使用会导致时间不对齐——策略在第 100 步看到的参考帧应该对应 t=2.0s，但 30 Hz 数据的第 100 帧对应 t=3.33s。`csv_to_npz.py` 通过插值把 30 Hz 升采样到 50 Hz，确保帧号和仿真时间一致。

**CSV 格式规范**：

```csv
# 每行一帧，列顺序：
# root_pos(3), root_quat(4), joint_angles(29)
# 总共 36 列（G1 29-DoF）
0.0,0.0,0.76, 1.0,0.0,0.0,0.0, 0.0,0.0,-0.39,0.80,-0.42,0.0,...
0.0,0.01,0.76, 0.999,0.001,0.0,0.0, 0.01,0.0,-0.40,0.82,-0.43,0.0,...
...
```

**WandB 注册**：mjlab 使用 Weights & Biases 的 Registry 功能管理参考运动数据集。这不是必须的——你也可以直接传文件路径——但 WandB 提供了版本管理和团队共享能力。

```bash
# 创建 WandB registry collection（一次性）
wandb artifact put --name your-org/motions/g1_walking \
    --type dataset g1_walking.npz

# 训练时引用
uv run train Mjlab-Tracking-Flat-Unitree-G1 \
    --registry-name your-org/motions/g1_walking \
    --env.scene.num-envs 4096
```

### g1_spinkick 示例详解

`mujocolab/g1_spinkick_example` 是 mjlab 中最小的 motion tracking 示例。它展示了如何让 G1 执行一个 double spin kick（双旋踢）。

```bash
# 克隆示例
git clone https://github.com/mujocolab/g1_spinkick_example
cd g1_spinkick_example

# 数据准备：从 pkl 转换为 csv，添加安全起止过渡
# --input 为 2.65 秒的旋踢动作；--safe-pose-duration 前后各加 0.5 秒安全过渡
# （续行反斜杠后不要写注释，否则 shell 续行失效）
python pkl_to_csv.py \
    --input g1_spinkick.pkl \
    --output g1_spinkick.csv \
    --safe-pose-duration 0.5

# CSV → NPZ
python scripts/tracking/csv_to_npz.py \
    --input-csv g1_spinkick.csv \
    --output-npz g1_spinkick.npz \
    --input-fps 30 --output-fps 50 --render

# 上传到 WandB
wandb artifact put --name your-org/motions/mimickit_spinkick_safe \
    --type dataset g1_spinkick.npz

# 训练
MUJOCO_GL=egl uv run train Mjlab-Spinkick-Unitree-G1 \
    --registry-name your-org/motions/mimickit_spinkick_safe \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 20000
```

**safe-pose-duration 的工程意义**：参考运动的第一帧可能是一个极端姿态（如空中旋转中）。如果 episode 从这个姿态开始，初始策略（接近随机）完全无法维持平衡——episode 在 1-2 步内就终止。safe-pose-duration 在动作前后各加入 0.5 秒的静止站立过渡，让策略有机会从稳定的站立姿态开始跟踪。

**反事实推理：如果不加 safe-pose-duration 会怎样？** 训练初期，策略在极端初始姿态下立即摔倒，reward 信号极稀疏。即使 PPO 偶尔采样到好的 rollout，也被大量的短 episode 淹没。加入安全过渡后，策略至少能在站立阶段获得正 reward（pose 接近 default），然后逐步学会过渡到动态动作。

### Reference State Initialization (RSI) 与三种采样模式

safe-pose-duration 解决的是"从极端姿态开始"的问题。另一个同样重要的工程决策是：**episode 从参考动作的哪一帧开始？** 这就是 DeepMimic 提出的 Reference State Initialization (RSI)。

**三种 episode 初始化模式**：

| 模式 | 初始帧 | 适用场景 | 优劣 |
|------|-------|---------|------|
| **start** | 总从第 0 帧开始 | 调试、短动作 | 简单但只练习前半段 |
| **uniform** | 从全段均匀随机采样 | 标准训练（RSI） | 覆盖全段，训练效率高 |
| **adaptive** | 按失败历史加权采样 | 后期精修 | 聚焦难点，但过早开启会聚焦伪难点 |

```python
# mjlab motion command 的采样模式配置（概念性）
cfg.commands.motion = MotionCommandCfg(
    motion_file="data/g1_walking.npz",
    sampling_mode="uniform",  # "start" / "uniform" / "adaptive"
)
```

**RSI (uniform 模式) 的工程意义**：如果总从第 0 帧开始（start 模式），策略只有在成功跟踪完前半段后才能接触后半段——如果动作后半段更难（如旋踢的空中阶段），策略可能永远学不到这部分。Uniform RSI 让策略有机会从任意位置开始练习——包括后半段的困难片段。

**adaptive 模式的使用时机**：不要一上来就用 adaptive——训练初期所有片段的失败率都很高（策略还不会），adaptive 会把采样集中到"随机失败"而不是"真正困难"的片段。推荐流程：

```
Phase 1: uniform（覆盖全段，让策略学会基本跟踪）
  → 切换条件: episode_length_ratio > 0.5

Phase 2: adaptive（聚焦失败片段，精修困难部分）
  → 结合 15.3 的 AdaptiveMotionSampler 使用
```

**注意区分两层 adaptive**：本节讨论的 adaptive 是**单条动作内**的帧采样——从一条参考动作的哪个时刻开始 episode。15.3 讨论的 AdaptiveMotionSampler 是**多条动作间**的采样——从数据集中选哪条动作来训练。两者是互补的、可以同时使用：先用 inter-motion adaptive 选一条困难动作，再用 intra-motion adaptive 选该动作中的困难片段。

**跨领域类比**：这就像学钢琴——先把整首曲子从头到尾过一遍（uniform），等大部分段落能弹下来后，再重点练习总是弹错的段落（adaptive）。如果一开始就只练难段（过早 adaptive），你可能永远不知道整首曲子的完整结构。

### Tracking Task 的 Reward 设计

motion tracking 的 reward 与 velocity tracking（Ch14）有本质区别——不再跟踪 twist command，而是跟踪全身参考姿态：

```python
# mjlab tracking task reward 配置（概念性）
cfg.rewards = {
    # === 核心跟踪 reward ===
    "body_position_tracking": RewardTermCfg(
        func=body_pos_tracking_exp,
        weight=3.0,
        params={
            "body_names": [
                "pelvis", "left_foot", "right_foot",
                "torso_link", "left_hand_link", "right_hand_link",
                "left_shoulder_link", "right_shoulder_link",
            ],
            "sigma": 0.1,  # 位置容忍度 (m)
        },
    ),
    "body_orientation_tracking": RewardTermCfg(
        func=body_ori_tracking_exp,
        weight=1.0,
        params={
            "body_names": ["pelvis", "torso_link"],
            "sigma": 0.2,
        },
    ),
    "joint_position_tracking": RewardTermCfg(
        func=joint_pos_tracking_exp,
        weight=1.5,
        params={"sigma": 0.3},
    ),
    "root_velocity_tracking": RewardTermCfg(
        func=root_vel_tracking_exp,
        weight=1.0,
        params={"sigma": 0.5},
    ),
    
    # === Regularization（复用 Ch14） ===
    "action_rate_l2": RewardTermCfg(func=action_rate_l2, weight=-0.1),
    "dof_acceleration": RewardTermCfg(func=dof_accel_l2, weight=-0.0025),
    "angular_momentum": RewardTermCfg(
        func=angular_momentum_penalty, weight=-0.01,
        params={"sensor_name": "root_angmom"},
    ),
    
    # === Safety（复用 Ch14） ===
    "self_collision": RewardTermCfg(func=self_collision_penalty, weight=-1.0,
        params={"body_pairs": [("left_hand_link", "left_thigh_link"), ...]}),
    "undesired_contacts": RewardTermCfg(func=undesired_contacts_penalty, weight=-1.0,
        params={"body_names": ("torso_link", "pelvis", "left_thigh_link", "right_thigh_link")}),
}
```

**与 Ch14 velocity reward 的关键区别**：

| 对比项 | Velocity (Ch14) | Tracking (Ch15) |
|--------|----------------|-----------------|
| 核心 reward | `track_lin_vel_xy` + `track_ang_vel_z` | `body_position_tracking` + `joint_position_tracking` |
| 需要参考数据 | 否 | **是**（每帧的参考姿态） |
| `variable_posture` | 是（约束上肢） | **否**（参考动作本身约束姿态） |
| `angular_momentum` weight | -0.02 | **-0.01**（更小——参考动作的角动量可能很大） |
| `body_position_tracking` | 否 | **是**（跟踪 pelvis/foot/hand 等关键点的 3D 位置） |
| body_names 列表 | N/A | **8-12 个 body**（pelvis, feet, hands, shoulders, elbows） |

**Tracking reward 的 body_names 选择非常关键**。如果 body_names 只包含 pelvis 和 feet——策略会学到"root 位置和脚位置对了就行"，但手臂和 torso 可能完全不对。如果包含所有 29 个 joint 对应的 body——计算量大且可能过度约束（某些关节的误差不影响整体动作质量）。标准做法是选择 **8-12 个关键 body**，覆盖 root、末端（手脚）和躯干。

### body_position_tracking 的实现细节

```python
def body_pos_tracking_exp(env, body_names, sigma):
    """跟踪关键身体点的 3D 位置。"""
    # 获取参考动作中当前帧的 body 位置
    ref_body_pos = env.command_manager.get_command("motion").body_positions
    # ref_body_pos shape: (N, num_bodies, 3)
    
    # 获取仿真中的实际 body 位置
    body_ids = [env.scene.entity.find_body(name) for name in body_names]
    sim_body_pos = env.data.body_pos_w[:, body_ids, :]  # (N, num_bodies, 3)
    
    # 转换到 base frame（相对于 pelvis）
    # 这很重要：绝对位置会被 root drift 影响
    base_pos = env.data.root_pos_w[:, :3]  # (N, 3)
    base_quat = env.data.root_quat_w  # (N, 4)
    
    ref_local = quat_rotate_inverse(base_quat, ref_body_pos - base_pos.unsqueeze(1))
    sim_local = quat_rotate_inverse(base_quat, sim_body_pos - base_pos.unsqueeze(1))
    
    # 计算 per-body L2 误差
    error = torch.sum((ref_local - sim_local) ** 2, dim=-1)  # (N, num_bodies)
    mean_error = torch.mean(error, dim=-1)  # (N,)
    
    # Exponential kernel
    reward = torch.exp(-mean_error / (sigma ** 2))
    return reward
```

**为什么要转换到 base frame？** 如果使用世界坐标系下的绝对位置，root drift（pelvis 在世界中的位置漂移）会同时影响所有 body 的误差——即使策略的关节跟踪完美，只是 root 位置偏了一点，所有 body 的误差都会变大。转换到 base frame 后，body tracking 只关注**相对于 root 的关节配置**，root 位置误差由 `root_velocity_tracking` 单独处理。

### Anchor 机制

BeyondMimic 使用 **anchor** 来处理参考运动和仿真之间的 root 对齐问题：

```python
# anchor 概念
# 参考动作的 root 轨迹: ref_root_traj[t] = (x_ref, y_ref, z_ref, quat_ref)
# 仿真中的 root: sim_root[t] = (x_sim, y_sim, z_sim, quat_sim)
#
# Anchor = 参考动作 root 轨迹的一个"锚定点"
# 在 episode 开始时，把参考动作的第一帧对齐到仿真中的实际位置
# 后续帧的参考位置 = anchor + 参考动作的相对偏移

anchor_body_name = "pelvis"  # or "torso_link"
```

**工程陷阱：anchor_body_name 与参考动作的 root 不一致**。如果参考动作以 `pelvis` 为 root，但 anchor 设为 `torso_link`，对齐会有偏差——特别是当腰部有弯曲时（pelvis 和 torso 的相对位置变化）。必须确保 anchor body 和参考动作的 root body 一致。

### 完整训练流程

```bash
# Step 1: 数据准备（5 分钟）
python scripts/tracking/csv_to_npz.py \
    --input-csv data/g1_walking.csv \
    --output-npz data/g1_walking.npz \
    --input-fps 30 --output-fps 50

# Step 2: Zero play 验证（3 分钟）
uv run play Mjlab-Tracking-Flat-Unitree-G1 \
    --agent zero --num-envs 2 \
    --viewer viser --no-terminations \
    --env.commands.motion.motion-file data/g1_walking.npz

# 观察要点：
# - 参考动作可视化是否正常？（透明模型跟随参考轨迹）
# - G1 和参考的初始对齐是否正确？
# - body_names 列出的 body 是否都在 MJCF 中存在？

# Step 3: Small train 验证（5 分钟）
uv run train Mjlab-Tracking-Flat-Unitree-G1 \
    --env.commands.motion.motion-file data/g1_walking.npz \
    --env.scene.num-envs 256 --agent.max-iterations 50

# Step 4: Large train（1-3 小时，取决于动作复杂度）
uv run train Mjlab-Tracking-Flat-Unitree-G1 \
    --env.commands.motion.motion-file data/g1_walking.npz \
    --env.scene.num-envs 4096 --agent.max-iterations 20000 \
    --agent.run-name g1_walking_tracking

# Step 5: 评估
uv run play Mjlab-Tracking-Flat-Unitree-G1 \
    --checkpoint-file logs/rsl_rl/g1_tracking/g1_walking_tracking/model_20000.pt \
    --env.commands.motion.motion-file data/g1_walking.npz \
    --num-envs 4 --viewer viser
```

### Tracking Task 的 Observation 配置

Motion tracking 的 observation 与 velocity tracking（Ch14）有一个核心区别：actor obs 需要包含**参考运动信息**。

```python
# mjlab tracking task actor observation
cfg.observations.actor = ObservationGroupCfg(
    enable_corruption=True,
    concatenate_terms=True,
    terms={
        # === 本体感知（与 Ch14 velocity 相同） ===
        "base_ang_vel": ObsTerm(
            func=base_ang_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.2),
        ),  # 3 维
        "projected_gravity": ObsTerm(
            func=projected_gravity,
            noise=GaussianNoiseCfg(mean=0.0, std=0.05),
        ),  # 3 维
        "joint_pos": ObsTerm(
            func=joint_pos_rel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),
        ),  # 29 维
        "joint_vel": ObsTerm(
            func=joint_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=1.5),
        ),  # 29 维
        "last_action": ObsTerm(func=last_action),  # 29 维
        
        # === 参考运动信息（tracking 特有） ===
        "motion_body_positions": ObsTerm(
            func=motion_ref_body_pos_in_base,
            params={"body_names": TRACKING_BODY_NAMES},
        ),  # 8 body × 3 = 24 维
        "motion_body_orientations": ObsTerm(
            func=motion_ref_body_ori_in_base,
            params={"body_names": ["pelvis", "torso_link"]},
        ),  # 2 body × 4 = 8 维（四元数）
        "motion_joint_positions": ObsTerm(
            func=motion_ref_joint_pos,
        ),  # 29 维
        "motion_root_velocity": ObsTerm(
            func=motion_ref_root_vel,
        ),  # 6 维（线速度 3 + 角速度 3）
    },
)
# actor obs 总维度: 3+3+29+29+29 + 24+8+29+6 = 160（远大于 velocity 的 99）
```

**注意**：tracking 的 actor obs **没有 command term**（velocity 有 twist command），但有大量的 **motion reference terms**。这些 reference terms 告诉策略"下一帧你应该做什么"——这就是 tracking 和 velocity 的本质区别：velocity 给目标速度，tracking 给目标姿态。

**也注意 actor obs 没有 base_lin_vel**：与 Ch14 相同，base_lin_vel 在真机上不可直接测量。tracking task 通过 motion_root_velocity（参考动作的 root 速度，不是仿真中的实际速度）间接提供速度信息。

> **本质洞察**：motion tracking 和 velocity tracking 的根本区别不在于 reward 函数——而在于 **obs 中是否包含参考运动信息**。velocity task 的 obs 中没有任何关于"怎么走"的信息（只有 command 告诉"走多快"），策略必须自己发明步态。tracking task 的 obs 中有完整的参考姿态（"下一帧每个关节该在哪"），策略只需要学会"跟上参考"。这个区别解释了为什么 tracking 策略的 obs 维度是 velocity 的 ~1.6 倍（160 vs 99）——额外的 ~60 维全是参考运动信息。

### Tracking Task 的 Termination 配置

| 条件 | 类型 | 阈值 | 说明 |
|------|------|------|------|
| `time_out` | truncation | = 参考动作长度 | 跟踪完整个动作 = episode 结束 |
| `tracking_error_too_large` | terminal | MPJPE > 0.5m | 跟踪误差过大时提前终止 |
| `fell_over` | terminal | pitch/roll > 50° | 同 Ch14 |
| `base_height_too_low` | terminal | < 0.3m | 同 Ch14 |

**`tracking_error_too_large` 是 tracking 特有的 termination**。如果策略的跟踪误差超过阈值，说明它已经完全偏离参考动作——继续运行只会浪费训练时间（PPO 从这些 out-of-distribution 的 rollout 中学不到有用信息）。

但这个阈值不能太严格——训练初期所有策略的误差都很大。建议使用 **curriculum**：初始阈值设为 0.8m（宽松），随训练进度逐步收紧到 0.3m。

### 训练过程监控

除了 Ch14 介绍的标准指标，tracking task 还需要关注以下 tensorboard panel：

| Panel | 健康趋势 | 异常信号 | 对应动作 |
|-------|---------|---------|---------|
| `reward/body_position_tracking` | 持续上升到 >0.5 | 持续 <0.2 | 检查 body_names 有效性 |
| `reward/joint_position_tracking` | 持续上升 | 停滞 | 检查参考动作关节范围是否合理 |
| `reward/root_velocity_tracking` | 上升 | 上升但 body tracking 不涨 | root 对了但关节不对——anchor 问题 |
| `metric/mpjpe` | 下降到 <80mm | 持续 >200mm | 参考动作可能物理不可行 |
| `metric/episode_length_ratio` | 上升到 >0.8 | 持续 <0.5 | termination 过严或动作太难 |

**关键交叉检查**：如果 `root_velocity_tracking` 很高但 `body_position_tracking` 很低，说明策略的 root 位置/速度对了但全身姿态不对——这通常是 anchor 对齐的问题（root 跟踪好但相对姿态偏离）。

### Tracking 评估指标

| 指标 | 计算方式 | 合格范围 | 说明 |
|------|---------|---------|------|
| **MPJPE (mm)** | 所选跟踪 body 的平均欧氏距离（逐点位置误差取均值，非均方） | < 50 mm | Mean Per-Joint Position Error |
| **MPBPE (mm)** | 所有 body 的平均欧氏距离 | < 80 mm | Mean Per-Body Position Error |
| **Episode Length Ratio** | 实际长度 / 参考动作长度 | > 0.9 | 是否能跟踪到动作结束 |
| **Root Tracking Error (m)** | root 位置的 L2 误差 | < 0.15 m | root 漂移程度 |
| **Angular Momentum (Nm·s)** | 全身角动量 RMS | < 10 | 动作自然度 |

**MPJPE 和 MPBPE 的区别**：MPJPE 只计算 body_names 中列出的关键 body（8-12 个），MPBPE 计算所有 body。MPJPE 更常用因为不是所有 body 都重要——手腕角度对行走来说不关键。

### Tracking 训练曲线的典型形态

健康的 tracking 训练曲线与 velocity 训练（Ch14）有显著不同。以下是 G1 tracking 的典型五个阶段：

| 阶段 | 迭代范围 | 现象 | 解释 |
|------|---------|------|------|
| I. 站立期 | 0-1000 | body tracking 缓慢上升，ELR 极低 | 策略学会在 safe pose 阶段站稳 |
| II. 初始跟踪 | 1000-3000 | root tracking 快速上升 | 策略学会跟随 root 位置移动 |
| **III. 全身对齐** | 3000-8000 | **body tracking 跳跃式上升** | 策略从"root 对了但关节乱"过渡到"全身大致对齐" |
| IV. 精修 | 8000-15000 | MPJPE 缓慢下降 | 关节细节优化 |
| V. 极限 | 15000+ | 收敛到平台 | 物理引擎精度限制 |

**Stage III 是 tracking 训练最关键的突破点**。在这个阶段之前，策略可能只是跟着 root 位置走但全身姿态不对。Stage III 中 body_position_tracking reward 会出现一个明显的跳跃——这是策略从"只跟 root"到"理解全身参考姿态"的相变。

如果训练停滞在 Stage II（root 对了但 body 不涨）：
1. 检查 `body_position_tracking` reward 是否在 base frame 下计算——如果在世界坐标系下，root drift 会同时影响所有 body
2. 检查 body_names 是否包含足够的关键 body（至少需要 pelvis + 双脚 + 双手 = 5 个）
3. 检查 σ 是否太小导致初始 reward 信号太弱

### "G1 跟不上参考"的系统性排查流程

当 tracking 效果不好时（MPJPE > 100mm 或 ELR < 0.5），以下流程帮助你定位原因：

```
Step 1: 确认参考动作可视化正确
  → zero play + --no-terminations
  → 参考动作的透明模型是否显示正确？
  → 如果不正确 → motion file 格式或坐标系错误

Step 2: 确认参考动作物理可行
  → 关节角度在限位内？root 高度合理？
  → 如果不可行 → 需要 physics filter 或重新 retarget

Step 3: 确认 anchor 对齐正确
  → zero play 时 G1 和参考模型的初始位置是否重合？
  → 如果不重合 → anchor_body_name 配置错误

Step 4: 分解 tracking reward
  → root_velocity_tracking 高但 body_position_tracking 低
    → root 对了但关节不对 → 检查 body_names 和 σ
  → body_position_tracking 高但 ELR 低
    → 跟踪好但摔倒了 → termination 阈值太严或 DR 问题
  → 所有 tracking reward 都低
    → 策略完全没学会 → 检查 obs 中是否包含 motion reference terms

Step 5: 检查动作难度
  → 高动态动作（旋踢、翻滚）的 ELR 自然比行走低
  → 如果 safe_pose_duration 不够 → 策略从极端姿态开始就摔倒
  → 尝试增大 safe_pose_duration 或降低 tracking_error termination 阈值
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：motion file 的帧率与训练帧率不匹配**。错误做法：直接使用 30 Hz 的 MoCap 文件训练 50 Hz 的 tracking task。现象：策略看到的参考帧每 1.67 个仿真步才更新一次——中间步看到重复的帧，tracking reward 出现阶梯状跳变。正确做法：用 `csv_to_npz.py --input-fps 30 --output-fps 50` 对齐帧率后再训练。

💡 **概念误区：tracking reward 越高动作越像**。tracking reward 使用 exponential kernel，σ 决定了"多接近算好"。σ 太大时 reward 容易饱和（误差 5cm 和误差 1cm 的 reward 差异不大）；σ 太小时 reward 信号稀疏（只有极接近参考时才有明显 reward）。标准配置 σ=0.1m 是经验平衡点。

🧠 **思维陷阱：velocity reward 和 tracking reward 可以同时使用**。理论上可以，但实践中会产生冲突：velocity reward 鼓励策略按命令走，tracking reward 要求策略跟踪参考动作。如果参考动作的速度和 velocity command 不一致，两个 reward 会相互矛盾。标准做法是二选一，或者用 HOVER 的 mask 机制明确分离两种模态。

⚠️ **编程陷阱：body_names 拼写错误静默失败**。错误做法：`body_names=["torso", "left_foot", ...]`（但 MJCF 中的名字实际是 `torso_link`）。现象：函数返回零 tensor，reward 看起来在涨但实际上缺少关键 body 的约束——策略的 torso 完全不跟踪参考。正确做法：训练前用 zero play 打印所有 body_names 对应的 body_id，确认均非空。

⚠️ **编程陷阱：RSI 初始化到关节超限位置**。使用 uniform RSI 时，如果参考动作某些帧的关节角度超出机器人限位（retarget 不精确），episode 从这些帧开始会把机器人初始化到不可行状态——关节被 clamp 到限位，产生大量 `dof_pos_limits` penalty，策略从"被惩罚的初始状态"开始学习。正确做法：retarget 后用 physics filter（Ch15.5）检查所有帧的关节是否在限位内，超限帧从 RSI 候选集中剔除。

### Tracking Policy 的 ONNX 导出注意事项

与 velocity task（Ch14）的 ONNX 导出不同，tracking policy 的 ONNX 需要额外处理 **motion reference obs**：

```bash
# 导出 ONNX
uv run export-onnx Mjlab-Tracking-Flat-Unitree-G1 \
    --checkpoint-file logs/rsl_rl/g1_tracking/model_20000.pt \
    --output-dir exports/

# ONNX 输入：proprio obs + motion reference obs
# 输入维度: ~160 (不是 velocity 的 ~99)
```

**部署时的差异**：velocity task 的 ONNX 输入只需要 IMU + 编码器数据。tracking task 的 ONNX 额外需要**参考运动的当前帧数据**——这意味着部署时必须有一个 motion player 在后台按时间步进参考动作，并把当前帧的 body positions、joint angles、root velocity 喂给 ONNX。

```
部署时的数据流：
  Motion File → Motion Player → 当前帧参考数据 ─┐
  IMU + Encoders → Proprio Obs ────────────────┤
                                                ↓
                                          ONNX Model → Joint Commands → Robot
```

这增加了部署复杂度，但 g1_spinkick 示例已经提供了完整的部署管线——包括 `motion_tracking_controller` 和 RoboJuDo 的 `BeyondmimicPolicy`。ProtoMotions 更进一步：导出的 ONNX 内置了 obs 计算，部署框架只需提供原始传感器信号。

### 练习

1. **[实验题]** 使用 g1_spinkick_example 的完整流程，从 pkl 转换到训练。记录 (a) 训练 20000 iterations 后的 MPJPE，(b) episode length ratio，(c) 视频观察旋踢是否完整执行。
2. **[分析题]** 比较 tracking task 和 velocity task 的 actor observation terms。哪些是共有的？哪些是 tracking 特有的？tracking 特有的 terms 来自哪里（sensor 还是 command）？
3. **[设计题]** 如果你要让 G1 跟踪一段跑步动作，body_names 应该怎么选择？与行走相比需要增加什么 body？为什么？

---

mjlab 的 BeyondMimic 提供了 motion tracking 的 MuJoCo 侧实现。Isaac Lab 侧有一个更全面的工具——ProtoMotions，它不仅支持直接跟踪（Mimic），还支持 AMP、ASE、CALM 等更高级的 motion prior 算法。本节学习如何在 ProtoMotions 中切换这些算法。

## 15.2 在 Isaac Lab 中用 ProtoMotions 跑通 AMP/ASE ⭐⭐⭐

> **这一节解决什么问题**：学习 ProtoMotions 的统一框架，理解 AMP/ASE/CALM 的工程差异——算法切换如何只需修改配置文件。

### 为什么需要 AMP/ASE——直接跟踪的局限

15.1 的 BeyondMimic 式直接跟踪有一个根本局限：**每一帧都要精确对齐参考动作**。这对单条参考动作效果好，但面对多样化任务（导航、避障、交互）时限制了策略的灵活性。

AMP (Adversarial Motion Priors) 用一个不同的思路：不要求精确跟踪，而是训练一个**判别器**来区分"策略生成的动作"和"参考数据集中的动作"。策略的目标不是跟踪某一条特定轨迹，而是让自己的动作分布接近参考数据集的分布——"像人一样动"而不是"精确复现某个动作"。

| 方法 | 约束方式 | 灵活性 | 精确度 |
|------|---------|--------|--------|
| **Direct Tracking (BeyondMimic)** | 逐帧 MSE/exponential reward | 低——必须按参考走 | 高——MPJPE < 50mm |
| **AMP** | 判别器区分 real/fake motion | 中——可以自由组合多种 motion | 中——风格对但不精确 |
| **ASE** | AMP + latent skill embedding | 高——高层策略可以选择/混合技能 | 中 |
| **CALM** | ASE + text/condition embedding | 最高——文本控制动作风格 | 低——条件越抽象越不精确 |

### ProtoMotions 架构概览

ProtoMotions（`NVlabs/ProtoMotions`，Apache-2.0）是 NVIDIA 的统一 motion learning 框架。它把 AMP、ASE、CALM、MaskedMimic 和直接跟踪（Mimic）统一在一个代码库中，通过配置文件切换算法。

**类继承结构**：

```
BaseAgent
  ├── PPO
  │   ├── AMP      (PPO + discriminator)
  │   │   └── ASE  (AMP + latent encoder)
  │   └── Mimic    (PPO + tracking reward)
  │       └── ADD  (Mimic + adversarial)
  └── (BC variants for MaskedMimic distillation)
```

**关键设计**：所有 agent 共享 PPO 的训练循环，差异在于 loss 函数的组成——AMP 在 PPO loss 上加判别器 loss，ASE 在 AMP 上加 encoder loss。这意味着从 PPO → AMP 的切换只需要"打开判别器"，从 AMP → ASE 只需要"打开 encoder"。

### DeepMimic → AMP 的算法演进

理解 AMP 之前，先回顾 DeepMimic（2018）的方法——它是所有 motion imitation 工作的源头：

**DeepMimic 方法**：逐帧计算仿真姿态和参考姿态的 MSE，作为 reward。

$$r_{deepmimic} = w_p \cdot r_{pose} + w_v \cdot r_{vel} + w_e \cdot r_{end} + w_c \cdot r_{com}$$

其中 $r_{pose}$ 是关节角度误差，$r_{vel}$ 是关节速度误差，$r_{end}$ 是末端位置误差，$r_{com}$ 是质心位置误差。每个 term 都是 hand-crafted 的 exponential kernel。

**DeepMimic 的局限**：
1. **需要精确的时间对齐**——策略必须在正确的时间做正确的动作，不允许"快一点"或"慢一点"
2. **每次只能跟一条参考**——切换动作需要重新训练
3. **reward 权重需要手动调**——$w_p, w_v, w_e, w_c$ 的比例对效果影响大

**AMP 的解决方案**：不要求逐帧对齐，而是用**判别器**区分"真动作"和"假动作"。

**跨领域类比**：DeepMimic 就像素描教学中的"看一笔画一笔"——照着参考一笔一笔描。学到的是"精确复制"但不灵活。AMP 就像"学习了大师的绘画风格后自由创作"——不需要逐笔对照，只要最终的画"看起来像大师的风格"就行。判别器扮演的是"艺术评委"的角色——它不检查每一笔是否和参考一样，而是判断整体风格是否和参考数据集一致。

**AMP 的 Reward 组成**：

```
r_total = r_task + w_amp * r_amp

r_task: 任务 reward（如跟踪速度命令）
r_amp:  判别器 reward = max[0, 1 - 0.25*(D(s, s') - 1)^2]   （LSGAN，D 为回归输出）
w_amp:  判别器 reward 的权重（通常 0.5-1.0）
```

这意味着 AMP 可以同时追求**任务目标**（如走到目标点）和**风格约束**（如走得像人）。这是 AMP 相比 DeepMimic 的核心优势——DeepMimic 只能跟踪参考动作，没有额外的任务灵活性。

### 安装和运行

```bash
# 安装 ProtoMotions
git clone https://github.com/NVlabs/ProtoMotions.git
cd ProtoMotions
pip install -e .
pip install -r requirements_isaacgym.txt  # 或 requirements_isaaclab.txt

# 下载 AMASS 数据（需要注册）
# https://amass.is.tue.mpg.de/
# 下载后转换为 ProtoMotions 格式

# 运行 AMP（最简单的 adversarial motion prior）
python protomotions/train_agent.py \
    +exp=amp_mlp \
    +robot=g1 \
    +simulator=isaacgym \
    motion_file=data/motions/walking.yaml \
    experiment_name=g1_amp_walking
```

### 从 AMP 到 ASE 到 CALM：配置差异精读

ProtoMotions 的强大之处在于算法切换只需修改实验配置文件。以下是三种算法的配置差异：

**AMP 配置**（基础）：

```yaml
# examples/experiments/amp/mlp.yaml
agent:
  _target_: protomotions.agents.amp.AMP
  discriminator:
    _target_: protomotions.modules.AMPDiscriminator
    hidden_dims: [1024, 512]
    gradient_penalty_weight: 5.0
    discriminator_weight: 1.0
  
  # PPO 基础参数
  learning_rate: 5.0e-4
  gamma: 0.99
  clip_range: 0.2
```

**ASE 配置**（AMP + latent encoder，差异约 15 行）：

```yaml
# examples/experiments/ase/mlp.yaml
agent:
  _target_: protomotions.agents.ase.ASE    # ← 换 agent 类
  discriminator:
    _target_: protomotions.modules.AMPDiscriminator
    hidden_dims: [1024, 512]
    gradient_penalty_weight: 5.0
    discriminator_weight: 1.0
  
  # ASE 新增：latent encoder
  encoder:                                  # ← 新增
    _target_: protomotions.modules.SkillEncoder
    latent_dim: 64                          # ← 技能潜空间维度
    hidden_dims: [512, 256]
  encoder_weight: 1.0                       # ← encoder loss 权重
  
  learning_rate: 5.0e-4
  gamma: 0.99
  clip_range: 0.2
```

**CALM 配置**（ASE + text conditioning，再差异约 15 行）：

```yaml
# examples/experiments/calm/mlp.yaml
agent:
  _target_: protomotions.agents.calm.CALM  # ← 换 agent 类
  discriminator:
    _target_: protomotions.modules.ConditionalAMPDiscriminator  # ← 换判别器
    hidden_dims: [1024, 512]
    gradient_penalty_weight: 5.0
    discriminator_weight: 1.0
    condition_dim: 512                      # ← text embedding 维度
  
  encoder:
    _target_: protomotions.modules.ConditionalSkillEncoder  # ← 换 encoder
    latent_dim: 64
    hidden_dims: [512, 256]
    condition_dim: 512
  encoder_weight: 1.0
  
  # CALM 新增：text encoder
  text_encoder:                             # ← 新增
    _target_: protomotions.modules.CLIPTextEncoder
    model_name: "ViT-B/32"
  
  learning_rate: 5.0e-4
  gamma: 0.99
  clip_range: 0.2
```

**从 AMP → CALM 的总差异约 30 行**（符合 research brief 的估计）。这意味着你不需要理解三个算法的全部实现细节来切换它们——只需要理解每层新增了什么组件。

### AMP 判别器的工程实现

AMP 的核心是一个判别器 $D(s, s')$，输入是状态转移 $(s_t, s_{t+1})$。注意 AMP（Peng et al., SIGGRAPH 2021）用的是 **LSGAN（最小二乘）判别器**，$D$ 输出的是一个**回归值**而非概率——训练目标让它对参考数据回归到 $+1$、对策略数据回归到 $-1$（输出端不接 sigmoid）。策略的额外 reward 取 LSGAN 风格形式 $r_{amp} = \max\!\left[0,\; 1 - 0.25\,(D(s, s') - 1)^2\right]$：当 $D(s,s') \to 1$（判别器认为"像参考"）时 reward 接近 $1$，远离时 reward 衰减并被截断到 $0$。

```python
# AMP discriminator (概念性简化)
class AMPDiscriminator(nn.Module):
    def __init__(self, obs_dim, hidden_dims=(1024, 512)):
        super().__init__()
        # 输入: [s_t, s_{t+1}] 拼接
        input_dim = obs_dim * 2
        layers = []
        prev_dim = input_dim
        for h in hidden_dims:
            layers.extend([nn.Linear(prev_dim, h), nn.ReLU()])
            prev_dim = h
        layers.append(nn.Linear(prev_dim, 1))
        self.net = nn.Sequential(*layers)
    
    def forward(self, obs, next_obs):
        x = torch.cat([obs, next_obs], dim=-1)
        return self.net(x)  # LSGAN 回归输出（无 sigmoid；参考≈+1，策略≈-1）
    
    def compute_reward(self, obs, next_obs):
        with torch.no_grad():
            d = self.forward(obs, next_obs)
            # AMP 的 LSGAN style reward：D→+1 时 reward→1，远离则衰减并截断到 0
            reward = torch.clamp(1.0 - 0.25 * (d - 1.0) ** 2, min=0.0)
        return reward
```

**gradient penalty** 是 AMP 训练稳定的关键——没有它判别器会过度自信，reward 信号变成二值化（0 或很大），策略学不到平滑的改进方向。gradient penalty 约束判别器的梯度范数，使 reward 更平滑。

### Motion Data 准备（AMASS 数据集）

ProtoMotions 使用 YAML manifest 管理多个参考运动：

```yaml
# data/motions/walking_dataset.yaml
motions:
  - file: data/amass/walking_01.npy
    fps: 30
    text: "A person walks forward slowly"
  - file: data/amass/walking_02.npy
    fps: 30
    text: "A person walks in a circle"
  - file: data/amass/jogging_01.npy
    fps: 30
    text: "A person jogs forward"
```

**text 字段**是 CALM 需要的——AMP/ASE 忽略它。这种设计让同一个 manifest 文件可以被所有算法使用。

**NPY 格式内部结构**（与 mjlab 的 NPZ 不同）：

```python
# ProtoMotions 的 motion NPY 格式
import numpy as np
motion = np.load("walking_01.npy", allow_pickle=True).item()
# motion 是一个 dict:
# {
#   "root_pos": (T, 3),       # root 位置
#   "root_rot": (T, 4),       # root 四元数
#   "dof_pos": (T, J),        # 关节角度
#   "body_pos": (T, B, 3),    # 所有 body 的 3D 位置（可选）
#   "body_rot": (T, B, 4),    # 所有 body 的朝向（可选）
#   "fps": 30,                # 帧率
# }
```

**从 mjlab NPZ 到 ProtoMotions NPY 的转换**：两者的数据内容相同（root + joints），但键名和组织方式不同。需要写一个转换脚本来对齐字段名和坐标系约定。

### AMP 完整训练流程

```bash
# Step 1: 安装 ProtoMotions
git clone https://github.com/NVlabs/ProtoMotions.git
cd ProtoMotions
pip install -e .
pip install -r requirements_isaacgym.txt

# Step 2: 下载并准备 motion 数据
# 参考 ProtoMotions README 的数据准备步骤
# 或使用自己的 retarget 数据

# Step 3: 用 AMP 训练（判别器方法）
python protomotions/train_agent.py \
    +exp=amp_mlp \
    +robot=g1 \
    +simulator=isaacgym \
    motion_file=data/motions/walking_dataset.yaml \
    experiment_name=g1_amp_walking \
    num_envs=4096 \
    max_iterations=10000

# Step 4: 用 Mimic 训练（直接跟踪，类似 BeyondMimic）
python protomotions/train_agent.py \
    +exp=mimic_mlp \
    +robot=g1 \
    +simulator=isaacgym \
    motion_file=data/motions/walking_01.npy \
    experiment_name=g1_mimic_walking \
    num_envs=4096 \
    max_iterations=10000

# Step 5: 评估
python protomotions/eval_agent.py \
    +robot=g1 +simulator=isaacgym \
    motion_file=data/motions/walking_01.npy \
    checkpoint=results/g1_amp_walking/last.ckpt

# Step 6: 切换到 ASE（只改实验配置）
# 相比上面的 AMP，只需把 +exp 那一行换成 ase_mlp
python protomotions/train_agent.py \
    +exp=ase_mlp \
    +robot=g1 \
    +simulator=isaacgym \
    motion_file=data/motions/walking_dataset.yaml \
    experiment_name=g1_ase_walking \
    num_envs=4096
```

**关键观察**：AMP → ASE 只需要把 `+exp=amp_mlp` 改为 `+exp=ase_mlp`。其余参数完全相同。这就是 ProtoMotions 统一配置系统的威力。

### AMP vs Mimic 的训练行为差异

| 阶段 | AMP | Mimic (直接跟踪) |
|------|-----|-----------------|
| 0-1000 iter | 判别器学习真/假区分 | tracking reward 缓慢上升 |
| 1000-3000 iter | 策略开始"骗过"判别器 | MPJPE 快速下降 |
| 3000-5000 iter | 判别器和策略对抗平衡 | MPJPE 收敛 |
| 5000+ iter | 风格稳定，高层任务可用 | 精确度继续微调 |

**AMP 的特殊现象**：训练中会看到 discriminator loss 和 policy reward 交替波动——这是 GAN 式训练的正常现象。如果 discriminator loss 降到 0（判别器完胜），说明梯度消失，策略无法学习——需要加大 gradient penalty weight。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：ProtoMotions 依赖特定的 simulator 版本**。`+simulator=isaacgym` 需要 IsaacGym Preview 4；`+simulator=isaaclab` 需要特定版本的 Isaac Lab。版本不匹配会导致隐晦的 tensor shape 错误。

💡 **概念误区：AMP 不需要精确的参考动作**。AMP 的判别器学习动作分布的统计特性，不需要逐帧对齐。但参考数据的质量仍然非常重要——如果参考数据包含大量不自然的动作（retarget 质量差），判别器会学到"不自然也是正常的"。

🧠 **思维陷阱：ASE 的 latent space 自动学到有意义的技能分解**。ASE 的 encoder 学到的 latent 可能不对应人类直觉中的"技能"（如走、跑、跳）。它学到的是数据驱动的分解——可能是"左脚先迈 vs 右脚先迈"这种无语义的划分。

⚠️ **编程陷阱：MuJoCo backend 只支持 num_envs=1**。ProtoMotions 的 MuJoCo 后端是 CPU-only 的，只用于调试和可视化，不用于训练。训练必须使用 GPU 后端（IsaacGym、IsaacLab、Newton、Genesis）。

### 练习

1. **[实验题]** 在 ProtoMotions 中分别用 AMP 和直接跟踪（Mimic）训练同一个行走动作。训练 5000 iterations 后对比 (a) 动作自然度（视频观察），(b) MPJPE，(c) 策略在收到不同 command 时的灵活性。
2. **[配置题]** 写出从 AMP 配置切换到 ASE 配置需要修改的具体字段。解释 `latent_dim: 64` 意味着什么——如果改为 8 或 256，分别会有什么影响？
3. **[跨章综合题，Ch09+Ch15]** AMP 的判别器和 Ch09 的 teacher-student 蒸馏都使用了"额外的网络来指导策略训练"。列出它们的 3 个相同点和 3 个不同点。

### CALM 的 Text Conditioning 如何工作

CALM 在 ASE 的基础上加入了文本条件——用户可以通过自然语言描述（如 "walk forward slowly"）来控制策略生成的动作风格。工程上的关键是：

1. **Text encoder**：使用 CLIP 的 ViT-B/32 text encoder 将文本映射到 512 维 embedding
2. **条件判别器**：判别器不只看 state transition，还看 text embedding——它学习"这个动作是否匹配这段文字描述"
3. **条件 encoder**：ASE 的 skill encoder 也接受 text embedding 作为条件

```python
# CALM 的条件判别器（概念性）
class ConditionalAMPDiscriminator(AMPDiscriminator):
    def __init__(self, obs_dim, condition_dim=512, **kwargs):
        super().__init__(obs_dim + condition_dim, **kwargs)
        self.text_encoder = CLIPTextEncoder("ViT-B/32")
    
    def forward(self, obs, next_obs, text):
        text_emb = self.text_encoder(text)  # (N, 512)
        # 拼接 text embedding 到 observation
        x = torch.cat([obs, next_obs, text_emb], dim=-1)
        return self.net(x)
```

**从 AMP 到 CALM 的工程路径**：先训练 AMP（无条件判别器），确认判别器能区分真/假动作。然后切换到 CALM 配置（加入条件判别器 + text encoder），在相同数据上重新训练。CALM 训练时间通常比 AMP 多 30-50%——因为条件判别器需要学习更复杂的映射。

### 部署时算法选择的影响

不同算法对部署的影响：

| 算法 | ONNX 模型大小 | 推理输入 | 推理输出 | 部署复杂度 |
|------|-------------|---------|---------|-----------|
| Mimic | ~2 MB | obs + motion reference | action (29-D) | 低——但需要 motion file |
| AMP | ~2 MB | obs | action (29-D) | 低——不需要 motion file |
| ASE | ~3 MB | obs + latent (64-D) | action (29-D) | 中——需要高层策略提供 latent |
| CALM | ~5 MB | obs + text embedding (512-D) | action (29-D) | 高——需要 CLIP encoder |

**Mimic 的部署限制**：需要在运行时提供参考动作文件并实时查询当前帧——这增加了部署时的数据管理复杂度。AMP 没有这个限制——训练好的策略自身就包含了"像人走路"的知识。

---

单条动作跟踪只需要一个 GPU、几小时训练。但要构建一个**通用的运动技能库**——让机器人会走、跑、跳、踢、转身——需要从数千甚至数十万条动作中训练。这就是大规模 motion tracking 的工程挑战。

## 15.3 大规模训练工程 ⭐⭐

> **这一节解决什么问题**：学习大规模 motion tracking 训练（AMASS 全集约 4×A100/12h；BONES-SEED ~142K motions 官方用 24×A100）中总结出的工程经验——per-GPU 分片、adaptive sampling、motion quality filtering。

### 为什么需要大规模

单条或少量参考动作训练出的 tracking policy 只能执行特定动作。要构建通用的运动技能，需要覆盖尽可能多的动作种类。ProtoMotions 的最新版本支持在 AMASS（40+ 小时 MoCap）和 BONES-SEED（~142K motions）上训练。

| 规模 | 动作数 | GPU | 训练时间 | 典型用途 |
|------|-------|-----|---------|---------|
| 单条 | 1 | 1 × 4090 | 1-3 h | 特定技能（旋踢、舞蹈） |
| 小规模 | 10-50 | 1 × 4090 | 6-12 h | 基本运动库（走、跑、转身） |
| 中规模 | 100-1000 | 1-2 × A100 | 1-3 天 | 丰富的运动库 |
| **大规模（AMASS 全集）** | **40+ 小时（约上万条）** | **4 × A100** | **~12 h** | **通用 tracking policy** |
| **超大规模（BONES-SEED）** | **~142K motions** | **24 × A100（官方预训练）** | **需查日志** | **G1 通用 tracker** |

注意区分两个不同的基准，不要把它们的硬件/时间混为一谈：

- ProtoMotions README 明确说明 **AMASS 全集（40+ 小时）** 可在 **4 × A100 上约 12 小时** 训练完："Train your fully physically simulated character to learn motion skills from the entire public AMASS human animation dataset (40+ hours) within 12 hours on 4 A100s."
- 而 **BONES-SEED 约 142K motions** 是更大的数据集；官方 G1 deployment 文档说明其预训练模型使用了 **24 × A100**（需要 sharded MotionLib），具体耗时需另查日志。

### Per-GPU Motion 分片的详细实现

当 motion 数据集太大（142K 条动作），无法全部加载到单个 GPU 的内存中。解决方案是 **per-GPU 分片**：

```python
# per-GPU motion 分片的完整实现
import torch
import torch.distributed as dist
from typing import List

class DistributedMotionLoader:
    """在多 GPU 训练中分片加载 motion 数据"""
    
    def __init__(self, motion_manifest_path: str, num_envs_per_gpu: int = 4096):
        self.world_size = dist.get_world_size()
        self.rank = dist.get_rank()
        self.num_envs = num_envs_per_gpu
        
        # 加载 manifest（所有 GPU 都加载完整列表以确保一致性）
        all_motions = self._load_manifest(motion_manifest_path)
        
        # 随机打乱后分片（确保每个分片包含均匀的动作类型）
        import random
        random.seed(42)  # 所有 GPU 使用相同种子保证一致分片
        random.shuffle(all_motions)
        
        # 分片
        total = len(all_motions)
        per_gpu = total // self.world_size
        start = self.rank * per_gpu
        end = start + per_gpu if self.rank < self.world_size - 1 else total
        
        self.my_motions = all_motions[start:end]
        print(f"GPU {self.rank}: loaded {len(self.my_motions)}/{total} motions "
              f"[{start}:{end}]")
    
    def _load_manifest(self, path: str) -> List[dict]:
        """加载 YAML manifest"""
        import yaml
        with open(path) as f:
            manifest = yaml.safe_load(f)
        return manifest["motions"]
    
    def sample_motions(self, batch_size: int):
        """为 batch 中的每个 env 采样一个 motion"""
        indices = torch.randint(0, len(self.my_motions), (batch_size,))
        return [self.my_motions[i] for i in indices]
```

```
4 GPUs, 142K motions:
  GPU 0: motions[0:35500]      → 4096 envs 在这些 motions 中采样
  GPU 1: motions[35500:71000]  → 4096 envs 在这些 motions 中采样
  GPU 2: motions[71000:106500] → 4096 envs 在这些 motions 中采样
  GPU 3: motions[106500:142000]→ 4096 envs 在这些 motions 中采样
```

每个 GPU 只加载自己分片的 motion 数据，但所有 GPU 共享同一个策略网络（通过 distributed PPO 的 gradient allreduce 同步）。

**工程注意**：分片不应该按动作类型分——否则 GPU 0 可能只有走路、GPU 3 只有跳跃，策略在不同 GPU 上看到的数据分布不同，导致 gradient 不一致。应该**随机分片**或确保每个分片包含均匀的动作类型分布。

### Adaptive Sampling 的完整实现

均匀采样所有动作并不是最优策略。有些动作对当前策略来说很容易（如站立），有些很难（如旋踢）。Adaptive sampling 根据策略在每个动作上的表现动态调整采样概率：

```python
class AdaptiveMotionSampler:
    """根据策略表现动态调整 motion 采样概率"""
    
    def __init__(self, motions, alpha=0.01, min_weight=0.1):
        self.motions = motions
        self.alpha = alpha      # EMA 更新率
        self.min_weight = min_weight  # 最小采样权重（防止完全不采样）
        
        # 每个 motion 的成功率估计（初始 50%）
        self.success_rates = {i: 0.5 for i in range(len(motions))}
        
        # 每个 motion 被采样的次数（用于统计）
        self.sample_counts = {i: 0 for i in range(len(motions))}
    
    def update(self, motion_indices, episode_lengths, max_lengths):
        """批量更新成功率估计
        
        Args:
            motion_indices: (B,) 每个 env 当前在跟踪的 motion id
            episode_lengths: (B,) 每个 env 的 episode 长度
            max_lengths: (B,) 每个 motion 的最大长度
        """
        for i in range(len(motion_indices)):
            mid = motion_indices[i].item()
            success = episode_lengths[i].item() / max(max_lengths[i].item(), 1)
            
            # EMA 更新
            self.success_rates[mid] = (
                (1 - self.alpha) * self.success_rates[mid] + self.alpha * success
            )
    
    def sample(self, batch_size):
        """按失败率采样（失败的动作更可能被采样）"""
        weights = torch.zeros(len(self.motions))
        for i in range(len(self.motions)):
            failure_rate = 1.0 - self.success_rates[i]
            weights[i] = failure_rate + self.min_weight
        
        # 归一化
        weights = weights / weights.sum()
        
        # 多项式采样
        indices = torch.multinomial(weights, batch_size, replacement=True)
        
        # 更新采样计数
        for idx in indices:
            self.sample_counts[idx.item()] += 1
        
        return indices
    
    def get_statistics(self):
        """返回采样统计信息"""
        success_rates = list(self.success_rates.values())
        return {
            "mean_success_rate": sum(success_rates) / len(success_rates),
            "min_success_rate": min(success_rates),
            "max_success_rate": max(success_rates),
            "num_hard_motions": sum(1 for s in success_rates if s < 0.5),
        }
```

**与 Ch13 terrain curriculum 的对比**：

| 维度 | Terrain Curriculum (Ch13) | Adaptive Sampling (Ch15) |
|------|-------------------------|------------------------|
| 调整对象 | 地形难度 level | Motion 采样概率 |
| 调整粒度 | per-env | per-motion |
| 反馈信号 | episode return | episode length ratio |
| 调整方向 | 好的 env 去更难的地形 | 差的 motion 获得更多采样 |
| 目标 | 策略逐步掌握更难地形 | 策略关注还没学会的动作 |

两者都是"根据策略表现自适应调整训练分布"的实例——是 curriculum learning 在不同场景下的具体应用。

**工程效果**：BeyondMimic 论文报告 adaptive sampling 将困难动作的 episode length ratio 提升了 15-25%。直觉上这很合理——如果策略在旋踢上只能跟踪 30% 的帧，增加旋踢的采样频率让 PPO 看到更多旋踢的经验，策略逐步学会更长的跟踪。

**反事实推理：如果不用 adaptive sampling，所有动作均匀采样会怎样？** 简单动作（站立、慢走）很快收敛，后续的训练预算浪费在已经会的技能上。困难动作（旋踢、翻滚）的采样频率不够，策略可能永远学不会——因为 PPO 需要足够多的正样本来学习一个行为，均匀采样下困难动作的正样本太少。

### Distributed PPO 的 Gradient 同步

大规模训练使用 PyTorch DistributedDataParallel (DDP) 同步 gradient：

```bash
# 4 GPU 分布式训练启动
torchrun --nproc_per_node=4 \
    protomotions/train_agent.py \
    +exp=mimic_mlp +robot=g1 +simulator=isaacgym \
    motion_file=data/bones_seed_142k.yaml \
    num_envs=4096 \
    experiment_name=g1_large_scale
```

**工程注意**：
- `num_envs=4096` 是 **per-GPU** 的——4 GPU 总共 16384 envs
- 每个 GPU 的 motion 分片不同，但 gradient 通过 allreduce 同步
- learning rate 通常需要根据有效 batch size 线性放大（linear scaling rule）

### Motion Quality Filtering

不是所有参考动作都适合训练。从 AMASS 或视频中提取的动作可能包含物理不可行的序列——比如脚在空中但没有对应的接触力、质心超出支撑多边形但没有摔倒。直接使用这些动作训练会让策略学到"物理上不可能但参考数据中存在"的行为。

KungfuBot 的 **physics filter** 是目前最系统的方案（15.5 详细讨论）。这里先介绍基本的质量检查：

```python
# 基本 motion quality check
def check_motion_quality(motion_data, robot_model):
    """检查参考动作的物理可行性"""
    issues = []
    
    for t in range(len(motion_data)):
        # 1. 关节角度在限位范围内
        for j in range(robot_model.nq):
            if motion_data.joint_pos[t, j] < robot_model.joint_range[j, 0]:
                issues.append(f"Frame {t}: joint {j} below lower limit")
            if motion_data.joint_pos[t, j] > robot_model.joint_range[j, 1]:
                issues.append(f"Frame {t}: joint {j} above upper limit")
        
        # 2. root 高度合理（不在地面以下）
        if motion_data.root_pos[t, 2] < 0.1:
            issues.append(f"Frame {t}: root height too low ({motion_data.root_pos[t, 2]:.2f}m)")
        
        # 3. 相邻帧速度不超过物理极限
        if t > 0:
            dt = 1.0 / motion_data.fps
            root_vel = (motion_data.root_pos[t] - motion_data.root_pos[t-1]) / dt
            if torch.norm(root_vel) > 5.0:  # 5 m/s 合理上限
                issues.append(f"Frame {t}: root velocity too high ({torch.norm(root_vel):.1f} m/s)")
    
    return issues
```

### ⚠️ 常见陷阱

⚠️ **编程陷阱：per-GPU 分片后 batch statistics 不一致**。如果每个 GPU 的 motion 分布不同，PPO 的 advantage normalization 在不同 GPU 上会产生不同的 mean/std。使用 `torch.distributed.all_reduce` 在 normalize 前同步统计量。

💡 **概念误区：更多 motion = 更好的策略**。如果低质量动作占比过大，策略会学到"平均化"行为——每个动作都能跟踪一点但没有一个做得好。quality > quantity。先用 physics filter 清洗数据。

⚠️ **编程陷阱：motion 长度差异导致 batch padding 问题**。不同 motion 长度不同（3 秒 vs 30 秒），如果 episode 长度等于 motion 长度，batch 中的 episode 长度差异很大。PPO 的 advantage 计算需要固定长度的 trajectory。解决方案：(a) 把长 motion 切成固定长度的片段，(b) 使用 variable-length rollout + padding。

### 大规模训练的监控仪表盘

当在 4 A100 上训练 142K motions 时，标准的 tensorboard 指标不够用——你需要额外的监控来判断训练是否健康：

**Per-motion 统计**（每 1000 iterations 采样一次）：

```python
# 大规模训练的 per-motion 监控
def log_per_motion_stats(sampler, writer, global_step):
    """记录每个 motion 的成功率分布"""
    stats = sampler.get_statistics()
    
    # 记录全局统计
    writer.add_scalar("motion/mean_success_rate", stats["mean_success_rate"], global_step)
    writer.add_scalar("motion/min_success_rate", stats["min_success_rate"], global_step)
    writer.add_scalar("motion/num_hard_motions", stats["num_hard_motions"], global_step)
    
    # 记录成功率直方图
    success_rates = list(sampler.success_rates.values())
    writer.add_histogram("motion/success_rate_distribution", 
                         torch.tensor(success_rates), global_step)
    
    # 记录最难的 10 个 motion
    sorted_motions = sorted(sampler.success_rates.items(), key=lambda x: x[1])
    for i, (mid, rate) in enumerate(sorted_motions[:10]):
        writer.add_scalar(f"hardest_motions/rank_{i}_rate", rate, global_step)
```

**健康的训练趋势**：

| 指标 | 0-2k iter | 2k-5k iter | 5k-10k iter | 不健康信号 |
|------|----------|-----------|------------|-----------|
| mean_success_rate | 0.2-0.4 | 0.4-0.6 | 0.6-0.8 | 停滞在 <0.3 |
| num_hard_motions | ~80% | ~50% | ~20% | 不下降 |
| min_success_rate | ~0.0 | ~0.1 | ~0.3 | 始终为 0 |
| success_rate 分布 | 集中在 0-0.3 | 双峰（0.2 + 0.7） | 集中在 0.7-1.0 | 始终双峰 |

**双峰分布是大规模训练的特征**——初始阶段一些动作快速收敛（简单的走/站），另一些完全不会（高动态）。随着训练进行，双峰应该合并为单峰（大部分动作都会了）。如果双峰持续不合并，说明困难动作可能物理不可行——需要 physics filter 清洗。

### 大规模训练的资源规划

| 动作数 | 每 GPU envs | GPU 数量 | 总 envs | 训练时间 (A100) | 内存需求/GPU |
|--------|------------|---------|---------|---------------|-------------|
| 100 | 4096 | 1 | 4096 | ~6h | ~12 GB |
| 1000 | 4096 | 1 | 4096 | ~18h | ~18 GB |
| 10K | 4096 | 2 | 8192 | ~24h | ~20 GB |
| 50K | 4096 | 4 | 16384 | ~12h | ~25 GB |
| 142K | 4096 | 4 | 16384 | ~12h | ~35 GB |

**跨领域类比**：大规模 motion 训练就像同时教一个学生 142K 道不同的数学题。如果均匀出题（均匀采样），学生会在简单题上浪费时间、在难题上练习不够。adaptive sampling 相当于"智能题库"——根据学生的弱项自动增加相关题目的出现频率。per-GPU 分片相当于"分考场"——每个考场只负责一部分题目，但所有考场共享同一个评分标准（策略网络）。

### 练习

1. **[设计题]** 设计一个 adaptive sampling 的改进版本，考虑"最近成功率"而不是"累积成功率"。为什么最近成功率可能更好？在什么情况下累积成功率更好？
2. **[计算题]** 142K motions，平均每条 3 秒 × 50 Hz = 150 帧。每帧 36 列（root 7 + joints 29）× float32 = 144 bytes。整个数据集大约多少 GB？能放入一个 40GB A100 的 GPU 内存吗？
3. **[跨章综合题，Ch08+Ch15]** Adaptive sampling 和 Ch08 的 terrain curriculum 有什么共同点？都是根据策略表现动态调整训练分布——对比两者的调整对象、调整频率和反馈信号。

---

## 15.4 PHC 的 Progressive Neural Networks ⭐⭐⭐

> **这一节解决什么问题**：理解为什么单个 MLP 无法扩展到数千个动作，以及 PHC 的 Progressive Multiplicative Control Policy (PMCP) 如何解决灾难性遗忘。

### 问题：大规模 Motion 的灾难性遗忘

用单个 MLP 同时跟踪 1000+ 条不同的动作会遇到**灾难性遗忘**（catastrophic forgetting）：策略在学习新动作时，会覆盖之前学到的旧动作的参数。

| 阶段 | 动作 | 效果 |
|------|------|------|
| Step 1 | 训练走路 | ✅ 走路学会 |
| Step 2 | 训练跑步 | ✅ 跑步学会，⚠️ 走路变差 |
| Step 3 | 训练旋踢 | ✅ 旋踢学会，❌ 走路和跑步严重退化 |

**跨领域类比**：这就像用同一张白纸写了三份文件——每次写新文件都会部分擦除旧文件。纸的面积（网络容量）有限，新知识和旧知识争夺同一个参数空间。

### PHC 的 Progressive Multiplicative Control Policy (PMCP)

PHC (Perpetual Humanoid Control, Luo et al., ICCV 2023) 的解决方案是 **Progressive Neural Networks**——不在同一个网络上反复训练，而是为每组新动作分配新的网络容量：

```
Primitive 0 (基础动作: 走、跑、站)
  → MLP_0: obs → action_0
  → 训练直到收敛
  → 冻结 MLP_0 参数

Primitive 1 (困难动作: 跳、蹲、弯腰)
  → MLP_1: obs → action_1
  → 最终 action = action_0 * gate_1 + action_1
  → 训练 MLP_1，MLP_0 冻结
  → 冻结 MLP_1 参数

Primitive 2 (高动态: 旋踢、翻滚)
  → MLP_2: obs → action_2
  → 最终 action = (action_0 * gate_1 + action_1) * gate_2 + action_2
  → 训练 MLP_2，MLP_0/MLP_1 冻结
```

**关键设计**：

1. **冻结旧网络**：已训练好的 primitive 参数不再更新，保证旧技能不被遗忘
2. **Multiplicative composition**：新 primitive 的 action 通过乘法门控（gate）和加法残差与旧 primitive 组合
3. **Progressive mining**：每轮训练后，找出当前策略失败的动作（"hard sequences"），作为下一个 primitive 的训练集

> **本质洞察**：PMCP 的真正创新不是"用更大的网络"——它是把"学习新技能"和"保持旧技能"两个目标解耦了。传统方法试图在同一个参数空间中同时满足两个目标（必然冲突），PMCP 给每个新技能分配独立的参数空间（冻结旧参数 + 新增 primitive），用 gate 机制在推理时动态选择。这就像一个人学了钢琴再学吉他——不是用学钢琴的手指肌肉去弹吉他（会破坏钢琴技巧），而是发展新的肌肉记忆，同时保留旧的。

### Progressive Mining 的工程实现

```python
# PHC progressive mining 流程
def progressive_mine(policy, motion_dataset, threshold=0.7):
    """找出当前策略无法成功跟踪的动作"""
    hard_motions = []
    
    for motion in motion_dataset:
        # 用当前策略在每个动作上评估
        success_rate = evaluate_tracking(policy, motion, num_episodes=10)
        
        if success_rate < threshold:
            hard_motions.append(motion)
    
    print(f"Mined {len(hard_motions)}/{len(motion_dataset)} hard motions")
    return hard_motions

# 完整的 progressive training 循环
all_motions = load_amass_dataset()

# Primitive 0: 在所有动作上训练
policy = train_primitive(all_motions, num_iterations=50000)
freeze(policy.primitive_0)

# Primitive 1: 在失败动作上训练
hard_set_1 = progressive_mine(policy, all_motions, threshold=0.7)
policy = add_primitive(policy)
train_primitive(hard_set_1, num_iterations=30000)
freeze(policy.primitive_1)

# Primitive 2: 在仍然失败的动作上训练
hard_set_2 = progressive_mine(policy, all_motions, threshold=0.7)
policy = add_primitive(policy)
train_primitive(hard_set_2, num_iterations=20000)
freeze(policy.primitive_2)

# 评估
final_success = evaluate_all(policy, all_motions)
print(f"Final success rate: {final_success:.1%}")  # PHC+ 达到 100% on AMASS
```

### PHC 的训练命令

```bash
# PHC 使用 Hydra 配置系统
python phc/run_hydra.py \
    learning=im \                    # imitation learning
    exp_name=phc_prim_iccv \
    env=env_im \
    robot=smpl_humanoid_shape \
    robot.freeze_hand=True \         # 冻结手部关节（简化）
    env.motion_file=data/amass_train.pkl
```

**评估指标**：PHC 使用 `eval_success_rate` 而不是 per-episode `success_rate`。区别在于 `eval_success_rate` 是在完整的评估集上运行（每个动作跑 10+ episodes），而 per-episode 是单次采样。W&B 中监控 `eval_success_rate` 更可靠。

### PMCP vs 单个大 MLP

| 方案 | 参数量 | 新增动作成本 | 旧技能保持 | 推理速度 |
|------|-------|-----------|-----------|---------|
| 单个 MLP (1024,512,256) | ~1.5M | 需要从头训练 | ❌ 遗忘 | 快 |
| PMCP (3 primitives) | ~4.5M | 只训 1 个新 primitive | ✅ 冻结保证 | 稍慢（~1.3×） |
| PMCP (5 primitives) | ~7.5M | 同上 | ✅ | 稍慢（~1.5×） |

推理时需要串行执行所有 primitive——但每个 primitive 都是小 MLP，总开销可接受（人形控制频率 50 Hz，每步推理 < 1ms 足够）。

### PMCP 的网络架构详解

```python
# PMCP 架构（概念性实现）
class PMCP(nn.Module):
    """Progressive Multiplicative Control Policy"""
    
    def __init__(self, obs_dim, action_dim, num_primitives=3):
        super().__init__()
        self.primitives = nn.ModuleList()
        self.gates = nn.ModuleList()
        
        for i in range(num_primitives):
            # 每个 primitive 是一个独立的 MLP
            self.primitives.append(nn.Sequential(
                nn.Linear(obs_dim, 512),
                nn.ELU(),
                nn.Linear(512, 256),
                nn.ELU(),
                nn.Linear(256, action_dim),
            ))
            
            # gate 网络：决定新 primitive 的贡献比例
            if i > 0:
                self.gates.append(nn.Sequential(
                    nn.Linear(obs_dim, 128),
                    nn.ELU(),
                    nn.Linear(128, action_dim),
                    nn.Sigmoid(),  # 输出 [0, 1]
                ))
    
    def forward(self, obs):
        # Primitive 0: 基础动作
        action = self.primitives[0](obs)
        
        # Primitive 1+: multiplicative composition
        for i in range(1, len(self.primitives)):
            gate = self.gates[i-1](obs)  # (N, action_dim), 值在 [0,1]
            residual = self.primitives[i](obs)
            
            # Multiplicative gating + additive residual
            action = action * gate + residual
        
        return action
    
    def freeze_primitive(self, index):
        """冻结指定 primitive 的参数"""
        for param in self.primitives[index].parameters():
            param.requires_grad = False
        if index > 0:
            for param in self.gates[index-1].parameters():
                param.requires_grad = False
        print(f"Frozen primitive {index}")
    
    def add_primitive(self):
        """添加新的 primitive"""
        obs_dim = self.primitives[0][0].in_features
        action_dim = self.primitives[0][-1].out_features
        
        new_primitive = nn.Sequential(
            nn.Linear(obs_dim, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, action_dim),
        )
        new_gate = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
            nn.Sigmoid(),
        )
        
        self.primitives.append(new_primitive)
        self.gates.append(new_gate)
        print(f"Added primitive {len(self.primitives)-1}")
```

**gate 的作用**：gate 输出在 [0,1] 之间（Sigmoid 激活），决定旧 primitive 的输出有多少被保留。gate=1 时完全保留旧输出（忽略新 primitive），gate=0 时完全用新 primitive 的输出替换。训练中 gate 会学到"什么时候该用新动作、什么时候保留旧动作"——这就是为什么 PMCP 能避免遗忘：旧 primitive 冻结不变，gate 只在需要新行为时才"打开"新 primitive。

**反事实推理：如果用 additive composition 而不是 multiplicative 会怎样？** Additive: $a = a_0 + a_1 + a_2$。新 primitive 的输出直接叠加到旧的上面——如果新 primitive 对旧动作产生了错误的残差，旧技能就被破坏了。Multiplicative: $a = a_0 \cdot g_1 + a_1$。gate $g_1$ 可以选择性地"关闭"新 primitive 的影响（$g_1 = 1, a_1 = 0$ 时完全保留旧行为）。乘法门控提供了更强的旧行为保护。

### PHC 的完整训练循环

```python
# PHC progressive training 完整流程
def train_phc(all_motions, num_rounds=3, threshold=0.7):
    """PHC 的 progressive 训练循环"""
    
    # 初始化
    policy = PMCP(obs_dim=160, action_dim=29, num_primitives=1)
    
    for round_idx in range(num_rounds):
        print(f"\n{'='*60}")
        print(f"Round {round_idx}: training primitive {round_idx}")
        print(f"{'='*60}")
        
        if round_idx == 0:
            # Primitive 0: 在所有动作上训练
            train_set = all_motions
            max_iters = 50000
        else:
            # Primitive 1+: progressive mining
            hard_set = progressive_mine(policy, all_motions, threshold)
            print(f"Mined {len(hard_set)} hard motions out of {len(all_motions)}")
            
            if len(hard_set) == 0:
                print("All motions solved! Stopping.")
                break
            
            # 添加新 primitive
            policy.add_primitive()
            train_set = hard_set
            max_iters = 30000
        
        # 训练当前 primitive
        train_primitive_ppo(policy, train_set, max_iters)
        
        # 冻结
        policy.freeze_primitive(round_idx)
        
        # 评估
        eval_result = evaluate_all(policy, all_motions)
        print(f"Round {round_idx} done. "
              f"Overall success rate: {eval_result['success_rate']:.1%}, "
              f"Mean MPJPE: {eval_result['mpjpe']:.1f}mm")
    
    return policy
```

### PHC+ 的里程碑

PHC+ 在 AMASS 数据集上达到了 **100% 的 eval_success_rate**——意味着策略能成功跟踪 AMASS 中的每一条动作。这是通过 5 轮 progressive mining + 5 个 primitive 实现的。每一轮的 hard set 越来越小（Round 0: 全集，Round 1: ~40% 失败，Round 2: ~15% 失败，...），到 Round 4 时只剩最极端的动作（如快速旋转、极限弯曲）。

**工程经验**：
- 每个 primitive 的网络大小可以不同——Round 0 处理大量动作用大网络，Round 4 处理少量极端动作可以用小网络
- progressive mining 的 threshold 不宜太高（>0.9）——这会导致太多动作被标为"hard"，训练集不够聚焦
- 评估时必须用 `eval_success_rate`（所有动作跑 10+ episodes）而不是训练中的 per-episode success_rate

### ⚠️ 常见陷阱

⚠️ **编程陷阱：PHC 基于 Isaac Gym 而非 Manager-Based**。PHC 的原始代码使用 IsaacGym Preview 4，不兼容 mjlab/Isaac Lab 的 Manager-Based API。MuJoCo 版本在 `ZhengyiLuo/PHC_MJX`。

💡 **概念误区：更多 primitive 一定更好**。每个 primitive 增加了推理时间和训练复杂度。通常 3-5 个 primitive 就能覆盖 AMASS 的全部动作。过多 primitive 意味着每个 primitive 只处理很少的动作——相当于记忆而非泛化。

🧠 **思维陷阱：PMCP 完全解决了灾难性遗忘**。PMCP 通过冻结旧参数避免了参数级别的遗忘，但新 primitive 的 gate 机制可能在某些 obs 下错误地抑制旧 primitive 的输出——这是一种更微妙的"功能性遗忘"。

### PMCP vs 其他避免遗忘的方案

灾难性遗忘是多任务/多技能 RL 中的核心挑战。除了 PMCP，还有其他方案：

| 方案 | 机制 | 优点 | 缺点 | 代表工作 |
|------|------|------|------|---------|
| **PMCP (PHC)** | 冻结旧网络 + 新增 primitive | 旧技能100%保留 | 推理时间线性增长 | PHC/PHC+ |
| **AMP 替代** | 用判别器学分布而非逐动作跟踪 | 天然支持多动作 | 精确度不如直接跟踪 | ProtoMotions AMP |
| **EWC** | 正则化：约束重要参数不变 | 不需要增加网络 | 大规模时约束冲突 | Kirkpatrick 2017 |
| **Multi-Head** | 每个动作一个输出头 | 简单直接 | 头数 = 动作数 → 不可扩展 | — |
| **Replay Buffer** | 保存旧动作经验、混合训练 | 实现简单 | 内存需求大 | — |
| **大网络** | 用足够大的网络容纳所有动作 | 不需要特殊机制 | 收敛慢、泛化差 | — |

**为什么 PHC 选择 PMCP 而不是其他方案？** 在 motion tracking 这个具体问题中，PMCP 有两个独特优势：

1. **旧技能的保证**：冻结参数意味着旧动作的跟踪质量不可能下降。其他方案（EWC、Replay）只能"尽量不下降"但没有硬保证。
2. **渐进式扩展**：progressive mining 自然地把问题分解为"先解决简单的、再解决困难的"——每一轮只需要关注当前剩余的失败动作。

**PMCP 的代价是推理时间线性增长**——5 个 primitive 需要串行执行 5 个 MLP。但对于 50 Hz 的人形控制来说，5 个小 MLP（每个 ~1.5M 参数）的总推理时间 < 1ms，远在实时要求内。

### PHC 在 mjlab 中的等价实现路径

PHC 原始代码基于 Isaac Gym（非 Manager-Based），不能直接用于 mjlab。但 PMCP 的思想可以在 mjlab 中实现：

```python
# 在 mjlab 中实现 PMCP 的策略
# Step 1: 用标准 RSL-RL 训练 primitive 0
uv run train Mjlab-Tracking-Flat-Unitree-G1 \
    --env.commands.motion.motion-file data/amass_all.npz \
    --env.scene.num-envs 4096 --agent.max-iterations 50000 \
    --agent.run-name prim0

# Step 2: 评估所有动作，找出失败的（需要自己写评估脚本）
python evaluate_per_motion.py \
    --checkpoint logs/prim0/model_50000.pt \
    --motion-manifest data/amass_manifest.yaml \
    --threshold 0.7 \
    --output hard_motions_round1.yaml

# Step 3: 用 hard_motions 训练 primitive 1
# （需要修改 RSL-RL 支持 PMCP 架构——这是一个非 trivial 的工程任务）
# 目前 mjlab/RSL-RL 没有内置 PMCP 支持
# 选项 a: 自己实现 PMCP actor 并注册到 RSL-RL
# 选项 b: 使用 PHC_MJX（MuJoCo 版本的 PHC）
# 选项 c: 放弃 PMCP，改用 AMP 统一策略（不需要 progressive）
```

**工程建议**：如果你的动作库 < 100 条，直接用 BeyondMimic + adaptive sampling 通常足够，不需要 PMCP。PMCP 的价值在 1000+ 条动作时才显现。

### 练习

1. **[架构分析题]** 画出 3 个 primitive 的 PMCP 计算图。输入是 obs，输出是 action。标注每个 primitive 的 gate 和 residual 连接。
2. **[设计题]** 如果你有一个已训练好的 2-primitive PMCP 策略，现在要加入"后空翻"动作。列出完整步骤：(a) progressive mining, (b) 新增 primitive, (c) 训练, (d) 评估。
3. **[对比题]** PHC 的 PMCP 和 Multi-Task Learning 的区别是什么？Multi-Task 把所有 task 的 loss 加在一起训练一个网络——为什么这在 motion tracking 中效果差？
4. **[跨章综合题，Ch09+Ch14+Ch15]** PHC 的 progressive mining 和 Ch09 的 teacher-student 蒸馏都涉及"从一个策略到另一个策略的知识传递"。对比两者在以下维度的差异：(a) 传递什么知识，(b) 旧策略是否保留，(c) 新策略的训练数据从哪来。

---

## 15.5 精读：KungfuBot 高动态全身控制 ⭐⭐⭐

> **这一节解决什么问题**：通过精读 KungfuBot 的 physics filter + bi-level optimization，理解如何从视频中筛选物理可行动作并自适应跟踪难度。

### KungfuBot 是什么

KungfuBot (Xie, Han, Zheng et al., NeurIPS 2025, arXiv 2506.12851, `TeleHuman/PBHC`) 解决一个具体的挑战：**从互联网视频中学习高动态全身动作（功夫、跑酷、舞蹈）并部署到真机 G1 上**。

与 15.1 的 BeyondMimic（假设参考动作已经是高质量的 MoCap）不同，KungfuBot 从**视频**出发——视频中的人体估计天然包含噪声和物理不一致。

### 两阶段管线

```
Stage 1: Motion Processing（离线）
  视频 → GVHMR → SMPL 参数 → retarget 到 G1 URDF
    → Physics Filter (CoM/CoP + contact mask)
    → 接受 or 拒绝
    → 接受的 motions → training dataset

Stage 2: Adaptive Tracking（在线训练）
  训练 dataset → PPO + bi-level optimization
    → 自适应调整 tracking tolerance σ
    → asymmetric actor-critic
    → 输出: tracking policy
```

### 从视频到训练数据的完整管线

KungfuBot 的数据管线是目前最完整的"视频 → 真机动作"管线之一。以下是每一步的工具和注意点：

**Step 1：视频人体估计（GVHMR）**

```bash
# GVHMR: Gravity-View monocular Human Mesh Recovery
# 输入: 视频文件；输出: 每帧的 SMPL 参数 (body_pose, global_orient, transl, betas)
# 官方 demo 入口为 tools/demo/demo.py（-s 表示相机静止，可跳过视觉里程计）
python tools/demo/demo.py --video=input_video.mp4 -s
# 整文件夹推理可用 tools/demo/demo_folder.py
```

随后把 GVHMR 输出的 SMPL 序列转成 SMPL/G1 重定向所需格式（这一步属于 PBHC/HoloMotion 等下游管线，不是 GVHMR 原生命令）。GVHMR 相比 4DHumans 或 WHAM 的优势是更好的 gravity alignment——输出的 root 朝向和地面法向量对齐更准确，这对后续的物理可行性检查很重要。

**Step 2：SMPL → G1 Retarget**

```python
# retarget 的核心步骤
def retarget_smpl_to_g1(smpl_params, g1_urdf):
    """将 SMPL 参数转换为 G1 关节角度"""
    
    # 1. SMPL forward kinematics → 24 个关节的 3D 位置
    smpl_joints = smpl_forward(smpl_params)  # (T, 24, 3)
    
    # 2. 骨骼长度映射
    # SMPL 骨骼长度和 G1 不同（如 SMPL 前臂比 G1 长）
    # 使用 scale factor 调整
    scale = g1_limb_lengths / smpl_limb_lengths
    scaled_joints = smpl_joints * scale
    
    # 3. IK 求解 G1 关节角度
    g1_joint_pos = inverse_kinematics(
        scaled_joints, g1_urdf,
        joint_limits=g1_joint_limits,  # 确保在限位内
    )  # (T, 29)
    
    # 4. root 位置调整
    # SMPL root 高度基于 ~1.7m 人类，G1 是 1.32m
    root_pos = smpl_params.transl * (1.32 / 1.70)
    root_pos[:, 2] = compute_com_height(g1_joint_pos, g1_urdf)
    
    return g1_joint_pos, root_pos
```

**Retarget 的常见问题**：
- 手腕自由度不匹配：SMPL 手腕是 3-DoF ball joint，G1 手腕也是 3-DoF 但关节类型不同。需要 axis-angle 到 euler angle 的转换。
- 脚底穿地：retarget 后某些帧的脚可能在地面以下。需要 post-processing 把所有帧的最低点抬到地面以上。
- 速度不连续：IK 求解是逐帧独立的，可能产生不连续的关节轨迹。用低通滤波器平滑。

**Step 3：Physics Filter（工程实现）**

视频估计的动作可能包含物理不可行的片段——比如人在空中悬浮、重心超出脚的支撑区域但不摔倒。KungfuBot 用以下两个检查过滤不可行的动作：

**检查 1：CoM-CoP 邻近度**

```python
def check_com_cop_proximity(motion, robot_model, threshold=0.3):
    """检查质心投影是否在接触点附近"""
    issues = 0
    total_frames = len(motion.root_pos)
    
    for t in range(total_frames):
        # 计算全身质心 (Center of Mass)
        com = compute_com(motion.joint_pos[t], motion.root_pos[t], robot_model)
        com_xy = com[:2]  # 水平投影
        
        # 计算压力中心 (Center of Pressure)
        contact_points = get_contact_points(motion, t)
        if len(contact_points) == 0:
            continue  # 空中阶段跳过（跳跃是允许的）
        cop = np.mean(contact_points, axis=0)[:2]
        
        # 距离检查
        dist = np.linalg.norm(com_xy - cop)
        if dist > threshold:
            issues += 1
    
    # 超过 20% 帧不合格则拒绝
    issue_rate = issues / total_frames
    return issue_rate < 0.2, f"Issue rate: {issue_rate:.1%}"
```

**检查 2：Contact Mask 一致性**

```python
def check_contact_consistency(motion, height_threshold=0.02, vel_threshold=0.1):
    """检查接触标记与运动学是否一致"""
    issues = 0
    total_checks = 0
    
    for t in range(len(motion.root_pos)):
        for foot_name in ["left_foot", "right_foot"]:
            foot_pos = get_body_pos(motion, foot_name, t)
            
            total_checks += 1
            
            # 脚在地面附近
            near_ground = foot_pos[2] < height_threshold
            
            if near_ground:
                # 脚在地面上但速度大 → 应该是滑动（不物理）
                if t > 0:
                    dt = 1.0 / motion.fps
                    foot_vel = (get_body_pos(motion, foot_name, t)
                               - get_body_pos(motion, foot_name, t-1)) / dt
                    if np.linalg.norm(foot_vel[:2]) > 0.5:  # 水平速度 > 0.5 m/s
                        issues += 1  # 脚在地上但滑了很远
    
    issue_rate = issues / max(total_checks, 1)
    return issue_rate < 0.15, f"Contact issue rate: {issue_rate:.1%}"
```

**Physics filter 的效果**：KungfuBot 论文的 Figure 6 显示，通过 physics filter 筛选后的动作，策略的 episode length ratio 显著高于未筛选的动作——说明物理可行的参考动作更容易被跟踪。

### Bi-Level Optimization (BLO) 自适应 σ

在 15.1 中我们讨论了 tracking reward 的 σ：$r = \exp(-\|error\|^2 / \sigma^2)$。σ 太大 reward 不灵敏，σ 太小 reward 信号稀疏。

KungfuBot 的核心创新是**自动调整 σ**——不用手动选择，而是通过 bi-level optimization 在训练过程中自适应调整：

```python
class AdaptiveTrackingReward:
    """KungfuBot 的 bi-level optimization for tracking tolerance σ"""
    
    def __init__(self, initial_sigma=0.3, sigma_lr=0.001,
                 sigma_min=0.05, sigma_max=1.0, target_success=0.7):
        self.sigma = initial_sigma
        self.sigma_lr = sigma_lr
        self.sigma_min = sigma_min
        self.sigma_max = sigma_max
        self.target_success = target_success
        
        # 在线误差估计
        self.error_ema = 0.2  # 初始误差估计
        self.error_alpha = 0.01  # EMA 更新率
    
    def compute_reward(self, tracking_error):
        """外层：使用当前 σ 计算 tracking reward
        
        Args:
            tracking_error: (N,) 每个 env 的跟踪误差 (L2)
        Returns:
            reward: (N,) tracking reward
        """
        reward = torch.exp(-tracking_error ** 2 / (self.sigma ** 2))
        return reward
    
    def update_sigma(self, tracking_errors, episode_lengths, max_lengths):
        """内层：根据当前策略表现更新 σ
        
        BLO 的核心思想：
        - 如果当前成功率 < target → σ 太小（要求太严格）→ 增大 σ
        - 如果当前成功率 > target → σ 可以再小一些 → 减小 σ
        """
        # 更新在线误差估计
        mean_error = tracking_errors.mean().item()
        self.error_ema = ((1 - self.error_alpha) * self.error_ema 
                          + self.error_alpha * mean_error)
        
        # 计算当前成功率
        success_rate = (episode_lengths / max_lengths).mean().item()
        
        # σ 更新规则
        if success_rate < self.target_success:
            # 太难了 → 放宽 σ
            self.sigma *= (1 + self.sigma_lr)
        else:
            # 可以更严格 → 收紧 σ
            self.sigma *= (1 - self.sigma_lr * 0.5)  # 收紧比放宽慢
        
        # Clamp
        self.sigma = max(self.sigma_min, min(self.sigma_max, self.sigma))
    
    def get_info(self):
        return {
            "sigma": self.sigma,
            "error_ema": self.error_ema,
        }
```

**工程含义**：训练初期策略误差大，σ 自动增大让 reward 不那么苛刻（策略能获得 reward 信号开始学习）。训练后期策略误差小，σ 自动缩小要求更精确的跟踪。这避免了手动调 σ 的繁琐过程。

**BLO 的"bi-level"含义**（注意外/内层的约定要前后一致）：
- **外层（outer）**：σ（tracking factor）的选择，目标是让跟踪难度自适应（维持合适的跟踪精度/成功率）
- **内层（inner）**：在给定 σ 下做策略优化（PPO），最大化 reward

两层优化交替进行——PPO 在固定 σ 下优化策略几个 iteration，然后 σ 根据当前 tracking 表现更新。这不是复杂的二阶优化——只是一个简单的在线估计+调整循环。

### KungfuBot 的真机部署

KungfuBot 已在 Unitree G1 上验证了多种高动态动作：功夫套路、跑酷翻滚、舞蹈旋转。部署使用与 Ch14 相同的 ONNX + SDK2 管线。

**与 BeyondMimic 的对比**：

| 维度 | BeyondMimic | KungfuBot |
|------|-----------|-----------|
| 数据来源 | MoCap（高质量） | 视频（需过滤） |
| Physics filter | 无（假设数据可靠） | CoM/CoP + contact mask |
| σ 调整 | 手动固定 | BLO 自动调整 |
| 目标动作 | 通用（走、跑、跳） | 高动态（功夫、跑酷） |
| 指标 | MPJPE, ELR | MPBPE, Episode Length Ratio |
| 真机验证 | ✅ G1 | ✅ G1 |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：GVHMR 的估计质量影响全局**。如果视频中的人体估计不准确（遮挡、低分辨率），physics filter 会拒绝大量动作，导致训练数据不足。使用高质量视频源。

💡 **概念误区：BLO 是一个复杂的优化算法**。KungfuBot 的 BLO 实际上是一个简单的在线估计——根据当前误差调整 σ。"bi-level"指的是 σ 优化（外层）和策略优化（内层 PPO）的嵌套关系，不涉及复杂的二阶优化。

### 练习

1. **[分析题]** KungfuBot 的 physics filter 有两个检查（CoM-CoP 和 contact mask）。如果只用其中一个，分别会漏掉什么类型的不可行动作？
2. **[设计题]** 如果你要从功夫电影视频中提取 G1 可用的训练数据，列出完整的管线：从视频文件到 mjlab tracking task 的 NPZ 文件。每一步需要什么工具？
3. **[实验题]** 用同一条参考动作，分别用 σ=0.05, 0.1, 0.3, 1.0 训练 3000 iterations。画出 (a) tracking reward 曲线, (b) MPJPE 曲线。哪个 σ 最终 MPJPE 最低？
4. **[跨章综合题，Ch09+Ch14+Ch15]** KungfuBot 的 asymmetric actor-critic 与 Ch09 的 Privileged Learning 和 Ch14 的 HOVER teacher-student 都使用了"训练时有额外信息，部署时没有"的范式。列出三者在以下维度的对比：(a) teacher 的额外信息是什么，(b) student 如何学习，(c) 蒸馏 loss 是什么。

### KungfuBot vs BeyondMimic 的选择指南

| 场景 | 推荐 | 理由 |
|------|------|------|
| 有高质量 MoCap 数据 | BeyondMimic | MoCap 数据已经物理一致，不需要 filter |
| 从 YouTube 视频学动作 | **KungfuBot** | 视频估计噪声大，必须 filter |
| 训练多条普通动作（走/跑） | BeyondMimic | 普通动作通常物理可行 |
| 训练高动态动作（功夫/跑酷） | **KungfuBot** | 高动态动作的估计误差更大 |
| σ 不确定怎么设 | **KungfuBot** | BLO 自动调整 |
| 需要最快出结果 | BeyondMimic | 管线更简单，调参更少 |

**反事实推理：如果把 KungfuBot 的 physics filter 应用到 BeyondMimic 管线中会怎样？** 这是完全可行的组合——先用 physics filter 清洗 MoCap 数据（即使 MoCap 质量高，极端动作也可能有物理不一致），然后用 BeyondMimic 训练。这种组合可能比单独使用任何一方都更好——physics filter 保证数据质量，BeyondMimic 提供精确跟踪能力。

---

## 15.6 双框架对比：mjlab Tracking vs ProtoMotions ⭐⭐

> **这一节解决什么问题**：用同一个参考动作在 mjlab 和 ProtoMotions 中训练，建立跨框架的 motion tracking 理解。

### 配置差异总结

| 维度 | mjlab (BeyondMimic) | ProtoMotions (Mimic/AMP) |
|------|-------------------|------------------------|
| 物理引擎 | MuJoCo Warp | IsaacGym / IsaacLab / Newton / Genesis |
| 模型格式 | MJCF | URDF (→ USD) |
| Motion 格式 | NPZ (via csv_to_npz.py) | NPY / YAML manifest |
| Motion 管理 | WandB Registry | 本地文件 + YAML |
| 训练命令 | `uv run train Mjlab-Tracking-*` | `python train_agent.py +exp=...` |
| 配置系统 | Python dataclass | Hydra YAML |
| 算法支持 | 直接跟踪（BeyondMimic） | PPO / AMP / ASE / CALM / MaskedMimic |
| 判别器 | 无 | 有（AMP/ASE/CALM） |
| Retarget 工具 | csv_to_npz.py | PyRoki-based retarget |
| 部署支持 | ONNX + RoboJuDo | ONNX + RoboJuDo |

### 同一动作的跨框架实验设计

用同一条 G1 行走动作，在两个框架中训练，对比结果。实验步骤：

**Step 1：准备统一的参考动作**

```bash
# 从同一个 AMASS 源文件开始
# 先转为 mjlab 格式
python scripts/tracking/csv_to_npz.py \
    --input-csv data/g1_walking.csv \
    --output-npz data/g1_walking.npz \
    --input-fps 30 --output-fps 50

# 再转为 ProtoMotions 格式
python convert_mjlab_to_protomotions.py \
    --input data/g1_walking.npz \
    --output data/g1_walking.npy \
    --fps 50
```

**Step 2：对齐训练超参**

| 超参 | mjlab 设置 | ProtoMotions 设置 | 说明 |
|------|----------|-----------------|------|
| num_envs | 4096 | 4096 | 一致 |
| max_iterations | 10000 | 10000 | 一致 |
| PPO clip range | 0.2 | 0.2 | 一致 |
| learning rate | 5e-4 | 5e-4 | 一致 |
| gamma | 0.99 | 0.99 | 一致 |
| 网络 hidden dims | [512, 256, 128] | [512, 256, 128] | 一致 |
| physics_dt | 0.002 | 0.002 | 一致 |
| decimation | 10 | 10 | 一致 |

**Step 3：训练并记录**

```bash
# mjlab 侧
uv run train Mjlab-Tracking-Flat-Unitree-G1 \
    --env.commands.motion.motion-file data/g1_walking.npz \
    --env.scene.num-envs 4096 --agent.max-iterations 10000 \
    --agent.run-name mjlab_tracking_walk

# ProtoMotions 侧
python protomotions/train_agent.py \
    +exp=mimic_mlp +robot=g1 +simulator=isaacgym \
    motion_file=data/g1_walking.npy \
    num_envs=4096 max_iterations=10000 \
    experiment_name=proto_tracking_walk
```

### 预期差异和解释

| 指标 | mjlab 典型值 | ProtoMotions 典型值 | 差异来源 |
|------|------------|-------------------|---------|
| MPJPE (mm) | 40-60 | 45-70 | 接触模型差异（MuJoCo 凸优化 vs PhysX TGS） |
| Episode Length Ratio | 0.85-0.95 | 0.80-0.90 | termination 配置差异 |
| 训练速度 (steps/s) | ~10000 | ~15000 | GPU 利用率和物理引擎吞吐差异 |
| 收敛 iterations | ~5000 | ~4000 | reward 计算和接触处理差异 |
| Action smoothness | 较高 | 中等 | MuJoCo 默认阻尼更大 |

**差异是正常的**——两个框架使用不同的物理引擎（MuJoCo vs PhysX），接触模型、积分方法和数值精度都不同。关键是差异在可接受范围内（MPJPE 差 < 20mm，ELR 差 < 0.1）。

**如果差异超出预期**：
1. 检查 obs 维度和 term 顺序是否对齐
2. 检查 reward 权重是否一致
3. 检查 action scale 和 default pose 是否一致
4. 检查 physics_dt × decimation 是否一致
5. 检查参考动作的坐标系约定是否一致（y-up vs z-up）

### 跨框架 Motion 格式互转工具

```python
# mjlab NPZ → ProtoMotions NPY 转换
def convert_mjlab_to_protomotions(npz_path, npy_path, fps=50):
    """将 mjlab NPZ 格式转为 ProtoMotions NPY 格式"""
    import numpy as np
    
    data = np.load(npz_path)
    
    # mjlab NPZ 结构: root_pos(T,3), root_quat(T,4), joint_pos(T,29)
    # ProtoMotions NPY 结构: dict with root_pos, root_rot, dof_pos
    
    motion = {
        "root_pos": data["root_pos"],     # (T, 3)
        "root_rot": data["root_quat"],    # (T, 4) — 确认四元数约定一致
        "dof_pos": data["joint_pos"],     # (T, 29)
        "fps": fps,
    }
    
    # 注意：ProtoMotions 可能使用不同的四元数约定（wxyz vs xyzw）
    # mjlab 使用 MuJoCo 约定 (w, x, y, z)
    # ProtoMotions 可能使用 (x, y, z, w)
    # 必须检查并转换！
    if needs_quat_conversion():
        motion["root_rot"] = convert_wxyz_to_xyzw(motion["root_rot"])
    
    np.save(npy_path, motion, allow_pickle=True)
    print(f"Converted {npz_path} → {npy_path}")

# ProtoMotions NPY → mjlab NPZ 转换
def convert_protomotions_to_mjlab(npy_path, npz_path):
    """反方向转换"""
    import numpy as np
    
    motion = np.load(npy_path, allow_pickle=True).item()
    
    np.savez(npz_path,
        root_pos=motion["root_pos"],
        root_quat=convert_xyzw_to_wxyz(motion["root_rot"]),
        joint_pos=motion["dof_pos"],
    )
```

⚠️ **四元数约定是跨框架最常见的 bug 来源**。MuJoCo 使用 (w,x,y,z)，PhysX/IsaacGym 使用 (x,y,z,w)。如果不转换，root 朝向会完全错误——策略看到的参考动作是"扭曲"的。

### 什么时候用哪个框架

| 需求 | 推荐框架 | 理由 |
|------|---------|------|
| 快速原型（单条动作） | mjlab | 安装简单，一行命令训练 |
| AMP/ASE/CALM 对比 | ProtoMotions | 内置所有算法 |
| 大规模训练（4+ GPU） | ProtoMotions | 内置分布式支持 |
| MuJoCo sim-to-sim 验证 | mjlab | 原生 MuJoCo 后端 |
| Isaac Lab extension 开发 | ProtoMotions | 与 Isaac Lab 深度集成 |
| 真机部署 (ONNX) | 两者皆可 | 都支持 ONNX + RoboJuDo |

### ⚠️ 常见陷阱

⚠️ **编程陷阱：两个框架的 body_names 对应关系不同**。mjlab 使用 MJCF body 名字（如 `torso_link`），ProtoMotions 可能使用 URDF link 名字（如 `torso`）。跨框架对比时必须确认 body 对应关系。

💡 **概念误区：一个框架的结果更好就说明它更优**。motion tracking 的效果高度依赖配置（reward 权重、body_names、σ 值）。在没有对齐所有配置之前，不能说某个框架"更好"——只能说在当前配置下表现不同。

⚠️ **编程陷阱：四元数约定不一致**。MuJoCo (w,x,y,z) vs PhysX (x,y,z,w)。不转换 → root 朝向完全错误。这是跨框架 motion tracking 最常见的 bug。

🧠 **思维陷阱：跨框架对比的目的是找"更好的框架"**。真正的目的是建立你对两个物理引擎差异的理解——哪些行为是物理引擎共有的（物理真实），哪些是特定引擎的 artifact（可能在真机上不成立）。

### 练习

1. **[实验题]** 用同一条行走动作在 mjlab 和 ProtoMotions 中训练。对比 MPJPE、ELR 和训练速度。差异超过预期范围吗？如果是，排查原因。
2. **[分析题]** 列出从 mjlab 的 NPZ 格式转换到 ProtoMotions 的 NPY 格式需要的具体步骤。写出转换脚本并验证正确性。
3. **[设计题]** 如果你要设计一个"跨框架 motion tracking benchmark"——用 10 条不同类型的动作在两个框架中评估，应该怎么设计？列出动作选择、评估指标和报告格式。

---

## 本章小结

| 知识点 | 核心要点 | 难度 |
|--------|---------|------|
| Motion Tracking MDP | command 从 3D twist 变为 ~100D 参考姿态；需要时间对齐 | ⭐⭐ |
| BeyondMimic 管线 | CSV → NPZ → WandB → uv run train；body_names 选择是关键 | ⭐⭐⭐ |
| body_position_tracking | base frame 下的关键点跟踪；anchor 处理 root 对齐 | ⭐⭐⭐ |
| AMP 判别器 | 学习动作分布而非逐帧跟踪；gradient penalty 保证稳定性 | ⭐⭐⭐ |
| ProtoMotions 算法切换 | AMP → ASE → CALM 只需修改 ~30 行配置 | ⭐⭐ |
| 大规模训练 | per-GPU 分片 + adaptive sampling + motion quality filter | ⭐⭐ |
| PHC PMCP | Progressive networks 避免灾难性遗忘；冻结旧 primitive | ⭐⭐⭐ |
| KungfuBot | Physics filter (CoM/CoP + contact) + BLO 自适应 σ | ⭐⭐⭐ |
| 双框架对比 | mjlab (BeyondMimic) vs ProtoMotions 的配置和性能差异 | ⭐⭐ |

### 本章与其他章节的关系

| 本章知识 | 前置来源（回顾） | 后续应用（预告） |
|---------|----------------|----------------|
| body_position_tracking reward | Ch06 Reward 设计 + Ch14 四层框架 | Ch20 全身控制的跟踪组件 |
| AMP 判别器 | Ch06 Reward（替代 hand-crafted reward） | Ch16 CALM 文本控制 |
| Adaptive sampling | Ch08 DR + Ch13 terrain curriculum | Ch19 大规模实验管理 |
| PMCP progressive networks | Ch09 Teacher-Student | Ch20 multi-skill 组合 |
| Physics filter | Ch14 motion quality | Ch22 自定义 env 的数据管线 |
| Anchor 机制 | Ch14 root 对齐 | Ch20 loco-manipulation root 管理 |
| σ 自适应 (BLO) | Ch06 exponential kernel reward | Ch22 自定义 reward 调参 |
| Motion data pipeline (CSV→NPZ) | Ch13 数据管理 | Ch16 text→motion→tracking 管线 |

### 本章建立的核心能力检查

| 能力 | 验证方式 | 对应小节 |
|------|---------|---------|
| 独立跑通 G1 tracking task | g1_spinkick 从 pkl 到 ONNX 全流程完成 | 15.1 |
| 理解 BeyondMimic 和 AMP 的区别 | 能解释何时用直接跟踪、何时用判别器 | 15.1-15.2 |
| 在 ProtoMotions 中切换算法 | 能从 AMP 切到 ASE 并解释配置差异 | 15.2 |
| 诊断 tracking 失败 | 面对 MPJPE > 100mm 能在 15 分钟内定位原因 | 15.1 |
| 理解大规模训练的工程挑战 | 能解释 per-GPU 分片和 adaptive sampling 的动机 | 15.3 |
| 理解 PMCP 架构 | 能画出 PMCP 计算图并解释 gate 机制 | 15.4 |
| 理解 KungfuBot 管线 | 能列出从视频到真机的完整步骤 | 15.5 |
| 执行跨框架对比 | mjlab vs ProtoMotions 对比报告 | 15.6 |

### 关键数字速查

| 数字 | 含义 | 来源 |
|------|------|------|
| ~160 维 | G1 tracking actor obs 总维度 | 15.1 obs 配置 |
| σ = 0.1 m | body_position_tracking 默认容忍度 | 15.1 reward 配置 |
| ~30 行 | AMP → CALM 配置差异 | 15.2 ProtoMotions |
| 142K | BONES-SEED 数据集动作数 | 15.3 大规模训练 |
| 12 小时 | AMASS 全集（40+ 小时）在 4×A100 上的训练时间（BONES-SEED 142K 官方用 24×A100） | 15.3 ProtoMotions 基准 |
| 100% | PHC+ 在 AMASS 上的 eval_success_rate | 15.4 PHC 里程碑 |
| 3-5 个 | PMCP 覆盖 AMASS 所需的 primitive 数量 | 15.4 经验法则 |
| 0.7 | progressive mining 的 success rate 阈值 | 15.4 PHC 配置 |
| (w,x,y,z) vs (x,y,z,w) | MuJoCo vs PhysX 四元数约定 | 15.6 跨框架 bug |

## 累积项目 C：本章新增模块

### 模块清单

| 模块 | 状态 | 说明 |
|------|------|------|
| G1 motion tracking (mjlab) | ✅ | BeyondMimic 管线 + g1_spinkick 示例 |
| ProtoMotions AMP/ASE (Isaac Lab) | ✅ | 三种算法的配置和训练 |
| Motion data pipeline | ✅ | CSV → NPZ + YAML manifest + WandB |
| MPJPE/MPBPE 评估 | ✅ | 定量评估 tracking 质量 |
| Adaptive sampling | ✅ | 大规模训练的动态采样 |
| Physics filter | ✅ | KungfuBot 的 CoM/CoP + contact mask |

### 实践里程碑（建议用时 4-5 天）

| 里程碑 | 预计用时 | 完成标准 | 前置 |
|--------|---------|---------|------|
| M1: g1_spinkick 跑通 | 3h | 从 pkl 到训练完成，MPJPE < 80mm | Ch14 M2 完成 |
| M2: 自定义动作 CSV→NPZ→train | 4h | 用自己的参考动作训练成功 | M1 |
| M3: ProtoMotions AMP 跑通 | 4h | AMP 训练收敛，判别器 loss 稳定 | M1 |
| M4: AMP vs Mimic 对比 | 4h | 同一动作两种算法训练，对比报告 | M3 |
| M5: Adaptive sampling 实现 | 6h | 10+ motions 训练，确认困难动作被更多采样 | M2 |
| M6: Physics filter 实现 | 4h | 从视频动作中筛选物理可行子集 | M2 |
| M7: 跨框架对比实验 | 6h | mjlab vs ProtoMotions 对比报告 | M2+M3 |
| M8: σ 敏感性实验 | 3h | 4 个 σ 值 × 3000 iter 对比 | M2 |

**总计 ~34 GPU-hours**（RTX 4090）。

**项目依赖链**：Ch14 的 G1 velocity baseline 是本章的前置。Ch14 建立的 per-joint action scale、variable posture 和 angular momentum 知识在本章直接复用——tracking task 使用相同的 action space 和 regularization reward。Ch15 的 tracking policy 又是 Ch16 和 Ch20 的底层组件。

**最关键的里程碑**是 M1（g1_spinkick）。如果你只有有限的时间，完成 M1 就足以理解 motion tracking 的核心工程流程。M3-M4（ProtoMotions AMP/Mimic 对比）是理解判别器方法 vs 直接跟踪的最有价值的实验。

### 快速读懂任意 Motion Tracking 配置的 5 分钟流程

```
分钟 1：找 motion data 来源
  → 是 MoCap 还是视频估计？帧率是多少？
  → 是否经过 retarget？目标机器人是什么？

分钟 2：检查 tracking reward
  → 有哪些 body 被跟踪？（body_names 列表）
  → σ 值是多少？是固定还是自适应？
  → 是直接跟踪（BeyondMimic）还是判别器（AMP）？

分钟 3：检查 observation
  → actor obs 中有参考运动信息吗？（motion_body_positions 等）
  → obs 总维度是多少？（tracking 通常 > 130）

分钟 4：检查 termination
  → 有 tracking_error_too_large termination 吗？阈值是多少？
  → episode 长度是固定的还是等于参考动作长度？

分钟 5：检查 anchor/root 对齐
  → anchor_body_name 是什么？
  → 参考动作的 root 是哪个 body？两者一致吗？
```

### 从本章到下一章

本章的 motion tracking 建立了"给定参考动作 → 忠实复现"的能力。但参考动作从哪来？Ch16（多模态动作获取）将回答这个问题——通过文本描述（"walk forward happily"）或视频（YouTube 上的舞蹈）生成参考动作。你会发现本章的 BeyondMimic 和 AMP 直接作为 Ch16 的底层 tracker 被复用——Ch16 的高层策略负责"生成什么动作"，Ch15 的 tracker 负责"物理地执行这个动作"。

**技术路线延续**：

```
Ch14 人形 velocity (基础运动)
  → Ch15 Motion Imitation (跟踪参考动作) ← 本章
     → Ch16 多模态获取 (文本/视频 → 参考动作)
        → Ch20 全身控制 (跟踪 + 任务目标)
```

---

## 延伸阅读

### 学术论文

| 资料 | 难度 | 会议/期刊 | 说明 |
|------|------|----------|------|
| Peng et al., "DeepMimic: Example-Guided Deep RL of Physics-Based Character Skills," 2018 | ⭐⭐ | SIGGRAPH 2018 | 物理角色动作模仿的奠基工作 |
| Peng et al., "AMP: Adversarial Motion Priors," 2021 | ⭐⭐⭐ | SIGGRAPH 2021 | 判别器替代手工 reward |
| Peng et al., "ASE: Large-Scale Reusable Adversarial Skill Embeddings," 2022 | ⭐⭐⭐ | SIGGRAPH 2022 | AMP + latent skill space |
| Tessler et al., "CALM: Conditional Adversarial Latent Models," 2023 | ⭐⭐⭐ | SIGGRAPH 2023 | ASE + text conditioning |
| Tessler et al., "MaskedMimic: Unified Physics-Based Character Control," 2024 | ⭐⭐⭐ | SIGGRAPH Asia 2024 | Masked motion inpainting |
| Luo et al., "PHC: Perpetual Humanoid Control," 2023 | ⭐⭐⭐ | ICCV 2023 | Progressive networks for large-scale tracking |
| Xie et al., "KungfuBot: Physics-Based Humanoid Whole-Body Control," 2025 | ⭐⭐⭐ | NeurIPS 2025 | Physics filter + BLO adaptive tracking |
| Liao et al., "BeyondMimic: Motion Tracking to Versatile Humanoid Control," 2025 | ⭐⭐⭐ | arXiv 2508.08241 | Guided diffusion for versatile control |

### 工具和代码

| 资料 | 难度 | 说明 |
|------|------|------|
| NVlabs/ProtoMotions | ⭐⭐⭐ | 统一 AMP/ASE/CALM/MaskedMimic，多 simulator |
| HybridRobotics/whole_body_tracking | ⭐⭐ | BeyondMimic Isaac Lab + mjlab port |
| mujocolab/g1_spinkick_example | ⭐⭐ | 最小 mjlab tracking 示例 |
| ZhengyiLuo/PHC | ⭐⭐⭐ | PMCP progressive tracking，IsaacGym |
| TeleHuman/PBHC | ⭐⭐⭐ | Physics filter + BLO，G1 部署 |

### 阅读路线

- **最小路线**（单条动作跟踪）：15.1 g1_spinkick → 跑通训练
- **标准路线**（多算法对比）：上述 + 15.2 ProtoMotions AMP/ASE → 15.6 对比
- **进阶路线**（大规模训练）：上述 + 15.3 adaptive sampling + 15.4 PHC PMCP
- **研究路线**：上述 + 15.5 KungfuBot + 自己实现 physics filter

---

## 🔧 故障排查手册

| # | 症状 | 可能原因 | 排查步骤 | 相关小节 |
|---|------|---------|---------|---------|
| 1 | tracking reward 一直为零 | body_names 中有无效名字 | 1. zero play 检查 body 可视化 2. 打印 body_names 对应的 id | 15.1 |
| 2 | episode 极短（<10 步） | 参考动作第一帧是极端姿态 | 1. 加 safe-pose-duration 2. 检查 CSV 第一行 | 15.1 |
| 3 | root 漂移严重 | anchor 和 motion root 不一致 | 1. 检查 anchor_body_name 2. zero play 对比 root 对齐 | 15.1 |
| 4 | AMP reward 不涨 | 判别器过度自信 | 1. 检查 gradient_penalty_weight 2. 增大 penalty | 15.2 |
| 5 | ProtoMotions MuJoCo 只能用 1 env | MuJoCo backend 是 CPU-only | 正常——用 GPU 后端训练 | 15.2 |
| 6 | 大规模训练 GPU OOM | motion 数据太大 | 1. 使用 per-GPU 分片 2. 减少 num_envs | 15.3 |
| 7 | 多动作训练后旧动作退化 | 灾难性遗忘 | 1. 用 PMCP 而非单 MLP 2. 或用 AMP 替代直接跟踪 | 15.4 |
| 8 | 视频提取的动作跟踪失败 | 参考动作物理不可行 | 1. 运行 physics filter 2. 检查 CoM-CoP 距离 | 15.5 |
| 9 | σ 过小导致 reward 稀疏 | 策略初始误差远大于 σ | 1. 增大 σ 或使用 BLO 2. 打印初始 tracking error | 15.1, 15.5 |
| 10 | 跨框架 MPJPE 差异大 | body_names 对应关系不同 | 1. 对齐两端 body_names 2. 确认 reference frame 一致 | 15.6 |

## Debug Checklist

**数据准备**

- [ ] 参考动作帧率与训练帧率一致（csv_to_npz.py 对齐）
- [ ] CSV 格式正确：root_pos(3) + root_quat(4) + joints(29) = 36 列
- [ ] 关节角度在机器人限位范围内
- [ ] 有 safe-pose-duration（动态动作）
- [ ] WandB artifact 已上传（如果使用 WandB）

**Reward 配置**

- [ ] body_names 中所有名字在 MJCF/USD 中有效
- [ ] σ 值合理（建议初始 0.1-0.3，或使用 BLO）
- [ ] body_position_tracking 在 base frame 下计算
- [ ] anchor_body_name 与参考动作 root 一致

**训练验证**

- [ ] zero play 确认参考动作可视化正常
- [ ] zero play 确认初始对齐正确
- [ ] small train 无 shape error
- [ ] tensorboard 中 tracking reward 在上升

**评估**

- [ ] MPJPE < 80mm（简单动作 < 50mm）
- [ ] Episode Length Ratio > 0.8
- [ ] 视频观察动作是否完整执行

---

## 附录 A：Motion Tracking Reward 完整参考表

| Term | 分类 | 典型权重 | 输入 | 说明 |
|------|------|---------|------|------|
| `body_position_tracking` | Tracking | +3.0 | 8-12 body 的 3D 位置 | 核心跟踪 reward |
| `body_orientation_tracking` | Tracking | +1.0 | pelvis + torso 朝向 | 防止扭曲 |
| `joint_position_tracking` | Tracking | +1.5 | 29 关节角度 | 精细关节匹配 |
| `root_velocity_tracking` | Tracking | +1.0 | root 线速度 + 角速度 | 运动动态匹配 |
| `action_rate_l2` | Regularization | -0.1 | 相邻步 action 差异 | 动作平滑 |
| `dof_acceleration` | Regularization | -0.0025 | 关节加速度 | 减少冲击 |
| `angular_momentum` | Regularization | -0.01 | pelvis 子树角动量 | 动作自然度 |
| `self_collision` | Safety | -1.0 | 异常碰撞对 | 防止穿模 |
| `undesired_contacts` | Safety | -1.0 | 非法 body 接触 | 防止膝/大腿着地 |

**与 velocity task reward 的差异总结**：
- 删除：`track_lin_vel_xy`, `track_ang_vel_z`, `variable_posture`, `base_height`
- 新增：`body_position_tracking`, `body_orientation_tracking`, `joint_position_tracking`, `root_velocity_tracking`
- 修改：`angular_momentum` 权重从 -0.02 减为 -0.01（参考动作本身可能有大角动量）

---

## 附录 B：算法谱系与工程选择指南

```
DeepMimic (2018)
  │  单条动作 + MSE reward
  │
  ├── AMP (2021): 判别器替代 MSE
  │   │  多条动作分布学习
  │   │
  │   ├── ASE (2022): AMP + latent encoder
  │   │   │  可复用技能表示
  │   │   │
  │   │   └── CALM (2023): ASE + text condition
  │   │       │  文本/条件控制
  │   │       │
  │   │       └── MaskedMimic (2024): BC distillation
  │   │           统一 inpainting 控制器
  │   │
  │   └── ExBody (2024): 上下半身解耦 AMP
  │
  ├── PHC (2023): Progressive Networks
  │   │  大规模不遗忘
  │   │
  │   └── PULSE/PHC+: 100% AMASS success
  │
  └── BeyondMimic (2025): 直接跟踪 + 扩散策略
      │  高动态真机部署
      │
      └── KungfuBot (2025): Physics filter + BLO
          从视频到真机的完整管线
```

**选择指南**：

| 需求 | 推荐方案 | 理由 |
|------|---------|------|
| 跟踪单条精确动作 | BeyondMimic (mjlab tracking) | 最简单，MPJPE 最低 |
| 风格化行走（多种步态） | AMP (ProtoMotions) | 灵活性好，不需要精确对齐 |
| 可控技能组合 | ASE/CALM (ProtoMotions) | latent space 支持高层策略 |
| 大规模技能库（1000+） | PHC (PMCP) | 避免灾难性遗忘 |
| 从视频学习高动态动作 | KungfuBot | Physics filter 确保可行性 |
| 统一控制（导航+操作+遥操作） | HOVER (Ch14.6) | Mask-conditioned 多模态 |

---

## 附录 C：AMASS 到 G1 的 Retarget 管线

将 AMASS 数据（SMPL 格式）转换为 G1 可用的参考动作是 motion imitation 的前置工作。以下是标准管线：

```
AMASS (.npz, SMPL 参数)
  │
  ├── body_pose (72 维: 24 joints × 3 axis-angle)
  ├── global_orient (3 维: root 朝向)
  ├── transl (3 维: root 平移)
  └── betas (10 维: 体形参数)
  │
  ↓ SMPL forward kinematics
  │
  SMPL 关节位置 (24 joints × 3D)
  │
  ↓ Retarget (IK + shape fitting)
  │  ├── 骨骼长度映射 (SMPL → G1 URDF)
  │  ├── 关节角度求解 (IK)
  │  └── root 高度调整 (G1 和 SMPL 身高不同)
  │
  G1 关节角度 + root 轨迹
  │
  ↓ csv_to_npz.py (帧率对齐)
  │
  G1 motion.npz (训练可用)
```

**Retarget 工具选项**：
- `PHC` 自带 `scripts/data_process/convert_amass.py`
- `ProtoMotions` 内置 PyRoki-based retarget
- `HOVER` 的 `third_party/human2humanoid` 子模块
- 手动 IK：适合自定义机器人但工作量大

**质量检查**：retarget 后必须检查 (a) 脚在地面以上，(b) 关节角度在限位内，(c) 根高度合理，(d) 无自碰撞。使用 15.1 的 zero play 可视化来快速验证。

---

## 附录 D：Motion Tracking 实验记录模板

```yaml
experiment:
  name: g1_tracking_spinkick_v2_seed42
  date: 2026-05-21
  framework: mjlab v0.2.1
  robot: Unitree G1 (29-DoF)
  algorithm: BeyondMimic (direct tracking)
  commit: abc1234
  gpu: RTX 4090
  
motion_data:
  source: MimicKit g1_spinkick.pkl
  duration: 2.65 s
  fps: 50 (after csv_to_npz.py)
  safe_pose_duration: 0.5 s
  retarget_tool: pkl_to_csv.py
  
config:
  num_envs: 4096
  max_iterations: 20000
  seed: 42
  
  tracking_bodies: [pelvis, left_foot, right_foot, torso_link,
                    left_hand_link, right_hand_link,
                    left_shoulder_link, right_shoulder_link]
  tracking_sigma: 0.1
  angular_momentum_weight: -0.01
  
results:
  mpjpe: 42.3 mm
  mpbpe: 65.1 mm
  episode_length_ratio: 0.92
  root_tracking_error: 0.08 m
  angular_momentum_rms: 4.5 Nm·s
  wall_time: 2.5 h
  steps_per_second: 9800
  
observations:
  gait: "旋踢动作基本完整执行"
  weaknesses:
    - "落地瞬间有轻微晃动"
    - "手臂在旋转阶段略偏离参考"
  
next_steps:
  - "增大 body_orientation_tracking 权重"
  - "收紧 shoulder body 的 σ"
  - "尝试 safe_pose_duration = 0.3s"
  
video: logs/rsl_rl/g1_tracking/spinkick_v2/videos/iter_20000.mp4
```

---

## 附录 E：σ 敏感性指南

tracking reward 的 σ 是 motion imitation 中最关键的超参数。以下是从大量实验中总结的经验：

### σ 的物理含义

σ 决定了 exponential reward $r = \exp(-\|error\|^2 / \sigma^2)$ 的"容忍度"：
- 当 error ≈ σ 时，reward ≈ $e^{-1}$ ≈ 0.37
- 当 error = 2σ 时，reward ≈ $e^{-4}$ ≈ 0.018（几乎为零）
- 当 error = σ/2 时，reward ≈ $e^{-0.25}$ ≈ 0.78（很高）

因此 **σ 大约是"奖励半衰期"**——误差为 σ 时 reward 下降到约 37%。

### 不同 reward term 的推荐 σ 范围

| Term | σ 单位 | 推荐范围 | 默认值 | 说明 |
|------|-------|---------|--------|------|
| `body_position_tracking` | m | 0.05-0.3 | **0.1** | 核心跟踪精度 |
| `body_orientation_tracking` | rad | 0.1-0.5 | **0.2** | 朝向容忍度 |
| `joint_position_tracking` | rad | 0.1-0.5 | **0.3** | 关节角度容忍度 |
| `root_velocity_tracking` | m/s | 0.2-1.0 | **0.5** | 速度匹配精度 |

### σ 对训练的影响

| σ 值 | 训练行为 | MPJPE 趋势 | 适用场景 |
|------|---------|-----------|---------|
| 0.03 | reward 稀疏，训练极慢 | 可能收敛到 <30mm（如果能收敛） | 高精度需求（精细操作） |
| **0.10** | 标准配置，收敛稳定 | 40-60mm | **大多数场景** |
| 0.20 | reward 容易饱和，精度较低 | 60-100mm | 初始调试、困难动作 |
| 0.50 | 非常宽松 | >100mm | 只需要大致风格匹配 |
| 1.00 | 几乎无约束 | >200mm | AMP 风格的统计匹配 |

### σ 的自适应策略

如果不确定 σ 该设多少，以下三种策略可选：

1. **固定 σ = 0.1**：最简单，适合大多数场景
2. **σ curriculum**：从 0.3 开始逐步降到 0.05——先让策略学到大致动作，再要求精度
3. **BLO 自适应**（KungfuBot）：根据当前误差自动调整——最灵活但实现更复杂

**跨领域类比**：σ 就像考试的评分标准。σ 很大时是"选择题"——差不多对就得分，容易拿分但区分度低。σ 很小时是"精确填空"——必须完全正确才得分，难度高但能训练出更精确的答案。BLO 就像"自适应评分"——考生水平低时用选择题（大 σ），水平提高后切换到填空题（小 σ）。

---

## 附录 F：Motion Imitation 方案选择决策树

```
你要解决什么问题？
│
├── "精确跟踪一条参考动作"
│   └── → BeyondMimic (mjlab tracking)
│       要求：高质量 MoCap/retarget 参考
│       指标：MPJPE < 50mm, ELR > 0.9
│
├── "让机器人的动作风格像人"
│   └── → AMP (ProtoMotions)
│       要求：motion 数据集（可以多条）
│       指标：判别器 accuracy ~50%, 视觉自然度
│
├── "可控的技能库"
│   ├── 需要文本控制？
│   │   ├── 是 → CALM (ProtoMotions)
│   │   └── 否 → ASE (ProtoMotions)
│   └── 要求：motion 数据集 + text labels (CALM)
│
├── "大规模（1000+）motion 跟踪"
│   └── → PHC (PMCP) 或 ProtoMotions Mimic
│       PHC: 避免灾难性遗忘（progressive networks）
│       ProtoMotions: 统一框架（adaptive sampling）
│
├── "从视频学习高动态动作"
│   └── → KungfuBot pipeline
│       要求：视频 + GVHMR + physics filter
│       指标：ELR on filtered motions
│
└── "统一控制器（导航+操作+遥操作）"
    └── → HOVER (Ch14.6) + tracking 作为底层
        要求：HOVER teacher-student pipeline
```

---

## 附录 G：Tracking Body Names 选择指南

body_names 的选择直接影响 tracking 质量。以下是不同动作类型的推荐配置：

### 基本配置（适合大多数动作）

```python
BASIC_TRACKING_BODIES = [
    "pelvis",              # root — 全局位置/朝向
    "left_foot", "right_foot",  # 末端 — 脚的位置决定步态
    "torso_link",          # 躯干 — 上半身朝向
    "left_hand_link", "right_hand_link",  # 末端 — 手的位置
]
# 6 个 body, 6 × 3 = 18 维位置信息
```

### 增强配置（适合上半身动作丰富的动作）

```python
ENHANCED_TRACKING_BODIES = [
    "pelvis", "torso_link",
    "left_foot", "right_foot",
    "left_hand_link", "right_hand_link",
    "left_shoulder_link", "right_shoulder_link",  # 肩部 — 摆臂幅度
    "left_elbow_link", "right_elbow_link",        # 肘部 — 手臂弯曲度
]
# 10 个 body, 10 × 3 = 30 维位置信息
```

### 高精度配置（适合舞蹈、功夫等精细动作）

```python
HIGH_PRECISION_TRACKING_BODIES = [
    "pelvis", "torso_link", "head_link",
    "left_foot", "right_foot",
    "left_hand_link", "right_hand_link",
    "left_shoulder_link", "right_shoulder_link",
    "left_elbow_link", "right_elbow_link",
    "left_knee_link", "right_knee_link",          # 膝部 — 弯膝幅度
]
# 13 个 body, 13 × 3 = 39 维位置信息
```

### 选择原则

1. **末端优先**：手和脚的位置对视觉效果影响最大
2. **root 必须包含**：pelvis 决定全局位移
3. **按需添加中间 body**：只有在动作涉及该 body 时才添加（如功夫需要 knee，行走不需要）
4. **不要过多**：超过 15 个 body 的收益递减，且增加 obs 维度和计算量
5. **检查 MJCF 中名字正确**：不同 MJCF 文件的 body 命名可能不同

---

## 附录 H：Motion Imitation 工程时间线与关键里程碑

### 从 DeepMimic 到真机部署：8 年演进

| 年份 | 工作 | 关键贡献 | 对本章的意义 |
|------|------|---------|------------|
| 2018 | **DeepMimic** | 逐帧 MSE reward + RSI + early termination | 建立了 motion imitation 的基本范式 |
| 2019 | **AMASS** | 统一 40+ 小时 SMPL MoCap 数据 | 提供了大规模训练的数据基础 |
| 2021 | **AMP** | 判别器替代手工 reward | 15.2 的核心方法 |
| 2022 | **ASE** | AMP + latent skill embedding | 15.2 的进阶方法 |
| 2023 | **CALM** | ASE + text conditioning | 15.2 的条件控制方法 |
| 2023 | **PHC** | PMCP progressive networks | 15.4 的大规模跟踪方法 |
| 2024 | **MaskedMimic** | Masked motion inpainting | ProtoMotions 最新统一方法 |
| 2024 | **ExBody** | 上下半身解耦跟踪 | Ch20 全身控制的前置 |
| 2025 | **BeyondMimic** | 直接跟踪 + 扩散策略 | 15.1 的 mjlab 管线 |
| 2025 | **KungfuBot** | Physics filter + BLO | 15.5 的视频→真机管线 |
| 2025 | **HOVER** | Mask-conditioned WBC | Ch14.6 的多模态控制器 |
| 2025 | **SONIC** | 42M params 行为基础模型 | ProtoMotions → GR00T-WBC 演进 |
| 2025 | **ProtoMotions v2.3.2** | G1 支持 + pretrained MaskedMimic | 15.2 的统一框架 |

**趋势观察**：2018-2022 年的重点是**算法创新**（DeepMimic → AMP → ASE → CALM），2023-2025 年的重点转向**工程落地**（PHC 大规模化、KungfuBot 视频管线、BeyondMimic/HOVER 真机部署）。这反映了领域成熟度的变化——算法已经够用，瓶颈转移到了数据管线、训练效率和 sim-to-real。

### 本章各参考工作的 GitHub 活跃度（star 数为易变数字，以下为截至 2026-06-09 的近似值，请以仓库当前为准）

| 仓库 | ⭐ 数（近似） | 维护状态 | 推荐场景 |
|------|------|---------|---------|
| NVlabs/ProtoMotions | ~1.7k | ✅ 积极维护 | AMP/ASE/CALM 研究和教学 |
| ZhengyiLuo/PHC | ~1.2k | 🔶 算法稳定 | 大规模 tracking 参考 |
| HybridRobotics/whole_body_tracking | ~2.1k | ✅ 活跃 | BeyondMimic Isaac Lab + mjlab |
| mujocolab/g1_spinkick_example | ~240 | ✅ 活跃 | mjlab 最小 tracking 示例 |
| TeleHuman/PBHC | ~890 | ✅ 活跃 | 视频→真机高动态动作 |

**工程建议**：选择活跃维护的仓库——它们的 issue 回复更快、与最新框架版本兼容性更好。ProtoMotions 和 BeyondMimic/mjlab 是 2025 年开始做 motion imitation 的最推荐起点。PHC 的原始代码虽然更新较少，但算法非常稳定——如果你需要大规模 tracking，PHC 的思想可以在 mjlab 中重新实现。

### 从"能跟踪"到"能部署"的工程差距

许多研究论文展示了精美的仿真结果，但从仿真到真机还有显著的工程差距：

| 维度 | 仿真中 | 真机上 | 差距来源 |
|------|-------|-------|---------|
| 物理引擎 | 确定性 | 随机性 | DR 不够 → sim-to-real gap |
| 传感器 | 精确 obs | 有噪声 IMU/编码器 | obs noise → 需要更鲁棒策略 |
| 通信延迟 | 0 | 5-20 ms | action delay → 需要 action buffer |
| 接触模型 | 近似 | 真实 | contact → 脚底打滑/碰撞 |
| 电机模型 | 理想 PD | 有限带宽/饱和 | actuator → 关节跟不上 target |

本章的所有方法（BeyondMimic, AMP, PHC, KungfuBot）都已经有真机验证——但部署的额外工程（DR 策略、ONNX 导出、SDK 接口）将在 Ch23（Sim2Real）中详细讨论。本章的目标是建立**仿真中的 motion tracking 能力**，这是真机部署的前提。

**对于想要快速上手的读者**：从 15.1 的 g1_spinkick 开始——它是整个 motion imitation 领域最小可运行的示例。理解了它的完整流程后，切换到 AMP（15.2）和大规模训练（15.3）都是自然的扩展。不要跳过 15.1 直接去 15.4 或 15.5——后者的理论需要以前者的实践经验为基础。

**Motion imitation 的终极目标不是让机器人完美复现参考动作**——那只是中间步骤。真正的目标是让机器人通过模仿人类动作获得一组**可复用的运动技能**，然后用这些技能去完成实际任务（导航、操作、交互）。Ch16-Ch20 将展示如何把本章建立的跟踪能力转化为任务级别的能力。

> **结语**：Motion imitation 是从"让机器人走路"到"让机器人像人一样动"的关键跨越。本章建立了两条工程路线——mjlab BeyondMimic（直接跟踪）和 ProtoMotions（AMP/ASE/CALM 判别器方法），以及大规模训练的工程实践（adaptive sampling、PMCP、physics filter）。这些技术将在 Ch16（文本/视频 → 动作生成）和 Ch20（全身控制）中作为底层组件被复用。如果你能在 G1 上跑通 motion tracking 并理解 BeyondMimic 和 AMP 的工程差异，你就掌握了人形动作模仿的核心工程能力。
