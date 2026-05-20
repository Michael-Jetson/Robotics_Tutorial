# RL Locomotion 工程实战指南 (IsaacLab / mjlab)

> **定位**: 本文档是 RL locomotion 的**工程操作手册**——覆盖安装、训练、调参、部署的全流程实操。假设读者已有 PPO/SAC 理论基础和基本机器人学知识。
>
> **不包含**: 算法理论推导（DeepMimic 奖励数学、AMP GAN loss、ASE 潜空间等）。需要理论背景请参阅对应论文或其他文档。
>
> **关联文档**:
> - mjlab 五层架构与代码精读 → `仿真/S03B_mjlab深度实战.md`
> - MuJoCo 引擎原理 → `仿真/S01_MuJoCo核心引擎_教学版.md`
> - legged_gym / rsl_rl 代码精读 → `足式/190_腿足RL训练栈.md`

---

## 目录

- [Part 1: 平台选型速查](#part-1-平台选型速查)
- [Part 2: 环境搭建与安装](#part-2-环境搭建与安装)
- [Part 3: 四足训练实战 (Go2)](#part-3-四足训练实战-go2)
- [Part 4: 人形训练实战 (H1/G1)](#part-4-人形训练实战-h1g1)
- [Part 5: 动作模仿实战](#part-5-动作模仿实战)
- [Part 6: 部署](#part-6-部署)
- [Part 7: 进阶工程技巧](#part-7-进阶工程技巧)
- [附录: 速查表](#附录-速查表)

---

## Part 1: 平台选型速查

### 1.1 MuJoCo vs Isaac Lab 一页对比表 ⭐

**背景**: MuJoCo 由 Emanuel Todorov 创建，2021 年 10 月被 DeepMind 收购，2022 年 5 月以 Apache-2.0 完全开源。Isaac Lab 是 NVIDIA 当前官方推荐的机器人 RL 训练框架，承接 Isaac Gym (legacy)。

| 维度 | MuJoCo CPU | MuJoCo MJX/Warp | Isaac Lab (PhysX 5 GPU) |
|------|-----------|-----------------|------------------------|
| **物理引擎** | 凸优化 + 软约束 | 同左 (JAX/Warp 重写) | PhysX 5 (硬约束 + GPU TGS) |
| **接触模型** | 椭圆摩擦锥, 软约束 | 同左 | Coulomb 摩擦, 硬约束 |
| **可微分** | MJX: `jax.grad` | MJX 支持, Warp 不支持 | 不支持 |
| **GPU 并行** | 不支持 | MJX: ~千级; Warp: ~万级 | ~千-万级 |
| **渲染** | 简单 viewer | 同左 | RTX 渲染 (照片级, 视觉 RL 必须) |
| **传感器** | 基础 (力/位置) | 同左 | 丰富 (深度相机, LiDAR, 触觉) |
| **安装** | `pip install mujoco` | 同左 + jax/warp | Isaac Sim ~20GB + NVIDIA 驱动 |
| **学习曲线** | 低 | 中 | 高 |

**速度对比 (RTX 4090, 4096 环境, locomotion 任务, 近似值):**

| 平台 | 物理步进频率 | 训练 10 亿步耗时 |
|------|------------|----------------|
| MuJoCo CPU (单线程) | ~1,000 steps/s | ~12 天 |
| MuJoCo MJX (JAX GPU) | ~100,000 steps/s | ~3 小时 |
| MuJoCo Warp (CUDA) | ~1,000,000+ steps/s | ~15-30 分钟 |
| Isaac Lab (PhysX 5 GPU) | ~500,000 steps/s | ~30-60 分钟 |

> **Note**: MuJoCo Warp 的官方 benchmark (NVIDIA, 2025) 报告在 manipulation 任务上比 MJX 快约 313 倍。具体速度取决于机器人复杂度和接触数量。

### 1.2 选型决策树 ⭐

```
你的项目需求是什么?
│
├─ 需要可微分物理 (system ID / 可微分 MPC)?
│  └─ → MuJoCo MJX (JAX)
│
├─ 需要视觉 RL (深度相机输入)?
│  └─ → Isaac Lab (Isaac Sim 高保真渲染)
│
├─ 需要最快速度训练 locomotion?
│  ├─ 已有 IsaacGym/Lab 代码?
│  │  └─ → Isaac Lab 或 mjlab (API 对等, 迁移成本低)
│  └─ 新项目?
│     └─ → MuJoCo Playground (最快上手) 或 mjlab (Manager API)
│
├─ 需要多仿真器验证 (sim2sim)?
│  └─ → Isaac Lab 训练 + MuJoCo CPU 验证 (工业默认实践)
│
└─ 教学/入门/快速原型?
   └─ → MuJoCo Playground (pip install, 15 分钟训完四足)
```

**关键开源项目速查:**

| 项目 | GitHub | Stars | 用途 | 仿真后端 |
|------|--------|-------|------|---------|
| IsaacLab | `isaac-sim/IsaacLab` | ~7.1k | NVIDIA 官方 RL 框架 | PhysX 5 GPU |
| legged_gym | `leggedrobotics/legged_gym` | ~2.9k | ETH RSL 四足 RL (legacy) | Isaac Gym |
| mjlab | `mujocolab/mjlab` | ~2.2k | Isaac Lab API on MuJoCo Warp | MuJoCo Warp |
| MuJoCo Playground | `google-deepmind/mujoco_playground` | ~1.8k | 端到端 GPU RL | MJX / Warp |
| mujoco_menagerie | `google-deepmind/mujoco_menagerie` | ~3.4k | 高质量 MJCF 模型库 | MuJoCo |
| humanoid-gym | `roboterax/humanoid-gym` | ~1.2k | 人形 RL 训练 | Isaac Gym |
| rsl_rl | `leggedrobotics/rsl_rl` | ~1k | 轻量 PPO 实现 | - |

---

## Part 2: 环境搭建与安装

### 2.1 IsaacLab 完整安装流程 ⭐

**前提条件:**
- NVIDIA GPU: RTX 3070+ (推荐 RTX 4090, 24GB VRAM)
- Ubuntu 22.04 / 24.04 (WSL2 可用但不推荐)
- NVIDIA Driver >= 535
- Python 3.10-3.11
- 磁盘空间: 100GB+ (Isaac Sim ~20GB + 模型 + 日志)
- RAM: 32GB 推荐

```bash
# === Step 1: 安装 Isaac Sim ===

# 方式 A: pip 安装 (推荐, 2024+ 版本支持)
pip install isaacsim-rl isaacsim-replicator \
    isaacsim-extscache-physics isaacsim-extscache-kit-sdk

# 方式 B: Omniverse Launcher 安装 (传统方式)
# 下载 Omniverse Launcher → 安装 Isaac Sim 4.x
# 安装包 ~20GB

# === Step 2: 克隆 IsaacLab ===
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# === Step 3: 创建环境并安装 ===
./isaaclab.sh --install       # Linux
# ./isaaclab.bat --install    # Windows

# === Step 4: 验证安装 ===
./isaaclab.sh -p source/standalone/tutorials/00_sim/create_empty.py
# 预期输出: Isaac Sim 窗口弹出, 显示空场景

# === Step 5: 快速验证训练 ===
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 1024 \
    --max_iterations 100
# 预期输出: 训练日志滚动, 无报错
```

**conda 安装变体:**

```bash
# 如果使用 conda 管理环境
conda create -n isaaclab python=3.10 -y
conda activate isaaclab
pip install isaacsim-rl isaacsim-replicator \
    isaacsim-extscache-physics isaacsim-extscache-kit-sdk
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab && ./isaaclab.sh --install
```

### 2.2 mjlab 完整安装流程 ⭐

> mjlab 的架构和代码精读详见 `S3B_mjlab深度实战.md`。本节只覆盖安装和验证。

**前提条件:**
- NVIDIA GPU (支持 CUDA)
- Python 3.10+
- 推荐使用 `uv` 作为包管理器

```bash
# === 零安装体验 (5 分钟) ===
# 无需 clone 代码, 直接运行 demo
uvx --from mjlab demo
# 预期输出: 浏览器打开 Viser 可视化界面

# === 完整安装 (用于开发和训练) ===
git clone https://github.com/mujocolab/mjlab.git
cd mjlab

# 方式 A: uv 安装 (推荐)
uv sync --extra dev
# 方式 B: pip 安装
pip install -e ".[dev]"

# === 验证安装 ===
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
    --env.scene.num-envs 64 \
    --agent.max-iterations 10
# 预期输出: 训练日志滚动, 无报错
# 完整训练只需改 num-envs 和 max-iterations
```

### 2.2.1 unitree_rl_mjlab 安装与使用 ⭐

> [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) 是 Unitree 官方基于 mjlab 的 RL 训练仓库，支持全线机器人的 velocity tracking 和 motion imitation 训练，并提供完整的真机部署管线（含 sim 部署验证）。

#### 安装 ⭐

```bash
# Step 1: 安装系统依赖 (部署编译需要)
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev

# Step 2: 创建环境
conda create -n unitree_rl_mjlab python=3.11 -y
conda activate unitree_rl_mjlab

# Step 3: Clone 和安装
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git
cd unitree_rl_mjlab
pip install -e .
```

**系统要求**: Ubuntu 22.04, NVIDIA GPU, 驱动版本 >= 550。

#### 支持的机器人 ⭐

| 机器人 | 类型 | 训练任务 ID | DoF | 备注 |
|--------|------|-----------|-----|------|
| **Go2** | 四足 | `Unitree-Go2-Flat` | 12 | 入门首选 |
| **A2** | 四足 | `Unitree-A2-Flat` | 12 | 工业级四足 |
| **As2** | 四足 | - | 12 | As2 变体 |
| **G1** | 人形 | `Unitree-G1-Flat` / `Unitree-G1-23Dof-Flat` | 12/23 | 支持 12DoF 和 23DoF 配置 |
| **R1** | 人形 | `Unitree-R1-Flat` | - | 轮式人形 |
| **H1_2** | 人形 | `Unitree-H1_2-Flat` | - | H1 第二代 |
| **H2** | 人形 | - | - | 最新人形 |

#### Velocity Tracking 训练 ⭐⭐

```bash
# Go2 四足 (入门推荐, 单 GPU ~30 分钟收敛)
python scripts/train.py Unitree-Go2-Flat --env.scene.num-envs=4096

# G1 人形 (12 DoF 腿部)
python scripts/train.py Unitree-G1-Flat --env.scene.num-envs=4096

# G1 人形 (23 DoF 全身)
python scripts/train.py Unitree-G1-23Dof-Flat --env.scene.num-envs=4096

# H1_2 人形
python scripts/train.py Unitree-H1_2-Flat --env.scene.num-envs=4096

# 多 GPU 训练
python scripts/train.py Unitree-G1-Flat --gpu-ids 0 1 --env.scene.num-envs=4096
```

训练结果保存在: `logs/rsl_rl/<robot>_velocity/<date_time>/model_<iteration>.pt`

#### Motion Imitation 训练 ⭐⭐

```bash
# Step 1: 准备参考动作 — CSV → NPZ 转换
python scripts/csv_to_npz.py \
    --input-file src/assets/motions/g1/dance1_subject2.csv \
    --output-name dance1_subject2.npz \
    --input-fps 30 \
    --output-fps 50 \
    --robot g1        # g1 或 g1_23dof

# Step 2: 训练 motion tracking
python scripts/train.py Unitree-G1-Tracking-No-State-Estimation \
    --motion_file=src/assets/motions/g1/dance1_subject2.npz \
    --env.scene.num-envs=4096
```

> 训练过程中自动导出 `policy.onnx` 和 `policy.onnx.data`，可直接用于真机部署。

#### Sim 部署验证 (unitree_mujoco) ⭐⭐

在部署到真机前，**强烈建议**先在 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) 仿真中验证：

```bash
# Step 1: 编译 unitree_mujoco 仿真器
cd simulate
mkdir build && cd build
cmake .. && make -j8

# Step 2: 启动仿真器 (需要连接手柄)
./simulate/build/unitree_mujoco
# 可在 simulate/config 中选择机器人

# Step 3: 启动仿真控制程序
cd deploy/robots/g1/build
./g1_ctrl --network=lo    # lo = 本地回环, 仿真用
```

#### 真机部署 ⭐⭐⭐

```bash
# Step 1: 开机 → 等待进入 zero-torque 模式
# Step 2: 按 L2 + R2 进入 debug mode (关节阻尼生效)
# Step 3: 以太网连接机器人
#   地址: 192.168.123.222
#   掩码: 255.255.255.0

# Step 4: 放置 ONNX 模型
# 将 policy.onnx 和 policy.onnx.data 放入:
# deploy/robots/g1/config/policy/velocity/v0/exported/

# Step 5: 编译控制程序
cd deploy/robots/g1
mkdir build && cd build
cmake .. && make

# Step 6: 运行 (替换 enp5s0 为实际网口名, 用 ifconfig 查看)
./g1_ctrl --network=enp5s0
```

**部署前置依赖:**
- [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds) — DDS 通讯中间件
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) — Unitree 通讯 SDK

**真机部署安全检查清单:**

| 检查项 | 操作 |
|--------|------|
| sim 验证通过 | 先在 unitree_mujoco 中完整测试 |
| debug mode 确认 | L2+R2 进入，关节有阻尼 |
| 网络连通 | `ping 192.168.123.1` 成功 |
| ONNX 路径正确 | 检查 `exported/` 目录下文件完整 |
| 初始悬挂测试 | 先悬挂运行，观察关节行为正常 |

### 2.3 legged_gym (Legacy) 安装 ⭐

> legged_gym 基于 Isaac Gym (legacy, NVIDIA 不再积极维护)。大量开源代码仍基于此。新项目建议用 Isaac Lab 或 mjlab。

```bash
# === Step 1: 下载 Isaac Gym Preview 4 ===
# 从 https://developer.nvidia.com/isaac-gym 手动下载
# 解压后进入目录

# === Step 2: 安装 Isaac Gym ===
cd isaacgym/python
pip install -e .
# 验证:
python -c "import isaacgym; print('Isaac Gym OK')"

# === Step 3: 安装 rsl_rl ===
git clone https://github.com/leggedrobotics/rsl_rl.git
cd rsl_rl && pip install -e . && cd ..

# === Step 4: 安装 legged_gym ===
git clone https://github.com/leggedrobotics/legged_gym.git
cd legged_gym && pip install -e . && cd ..

# === Step 5: 验证训练 ===
python legged_gym/scripts/train.py --task=anymal_c_flat
# 预期输出: Isaac Gym 窗口 + 训练日志
```

### 2.4 MuJoCo Playground 安装 ⭐

```bash
# 纯 pip 安装, 无需 NVIDIA 特殊驱动 (需要 CUDA GPU)
pip install mujoco-playground

# 验证: 训练 Go1 locomotion (~15 分钟)
python -m mujoco_playground.train \
    --task Go1JoystickFlatTerrain \
    --num_timesteps 200_000_000
# 预期输出: 训练进度和 reward 曲线
```

### 2.5 常见安装问题排查表 ⭐

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| `GLIBCXX_3.4.30 not found` | GCC/libstdc++ 版本过低 | `conda install -c conda-forge libstdcxx-ng` |
| GPU 内存不足 (OOM) | num_envs 太多 | 减少到 1024 或 512; 检查是否有其他 GPU 进程 |
| Isaac Sim 无法启动 | 显卡驱动版本 < 535 | `nvidia-smi` 检查驱动版本, 升级到 535+ |
| Isaac Sim 渲染黑屏 | 远程服务器无显示器 | 加 `--headless` 参数 |
| pip 安装冲突 | Python 版本不对 | 确保 Python 3.10-3.11 |
| `ModuleNotFoundError: isaacgym` | Isaac Gym 未正确安装 | 检查 `pip install -e .` 是否在正确目录执行 |
| mjlab `uv run` 失败 | uv 未安装 | `curl -LsSf https://astral.sh/uv/install.sh \| sh` |
| `CUDA error: out of memory` | GPU 显存不够 | 减少 num_envs; 用 `torch.cuda.empty_cache()` |
| MuJoCo Playground 报错 `jax not found` | JAX GPU 版本未安装 | `pip install jax[cuda12]` |
| IsaacLab `configclass` 导入失败 | 安装步骤不完整 | 重新执行 `./isaaclab.sh --install` |

---

## Part 3: 四足训练实战 (Go2)

### 3.1 从零开始: Go2 velocity tracking 训练 ⭐⭐

#### IsaacLab 版本 ⭐⭐

**完整训练命令:**

```bash
# 进入 IsaacLab 目录
cd IsaacLab

# 训练 Go2 velocity tracking (平地)
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 4096 \
    --max_iterations 5000 \
    --headless
# RTX 4090 预期: ~2 小时收敛, ~5 亿步

# 训练完成后评估
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/Isaac-Velocity-Flat-Unitree-Go2-v0/*/model_5000.pt
# 预期: 看到 Go2 在平地上跟随速度命令行走
```

**环境配置骨架 (IsaacLab Manager-based API):**

```python
# my_go2_env.py — 最小可运行配置
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import (
    ObservationGroupCfg as ObsGroup,
    ObservationTermCfg as ObsTerm,
    RewardTermCfg as RewTerm,
    TerminationTermCfg as DoneTerm,
    EventTermCfg as EventTerm,
)
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.sim as sim_utils

@configclass
class Go2FlatEnvCfg(ManagerBasedRLEnvCfg):
    scene = InteractiveSceneCfg(num_envs=4096, env_spacing=2.5)
    sim = sim_utils.SimulationCfg(dt=0.005, decimation=4)  # 策略频率 50Hz

    observations = {
        "policy": ObsGroup(obs_terms={
            "joint_pos":       ObsTerm(func=mdp.joint_pos_rel),       # 12 维
            "joint_vel":       ObsTerm(func=mdp.joint_vel_rel),       # 12 维
            "base_ang_vel":    ObsTerm(func=mdp.base_ang_vel),        # 3 维
            "projected_gravity": ObsTerm(func=mdp.projected_gravity), # 3 维
            "velocity_commands": ObsTerm(func=mdp.generated_commands,
                                        params={"command_name": "base_velocity"}),  # 3 维
            "last_action":     ObsTerm(func=mdp.last_action),         # 12 维
        }),
    }

    actions = {
        "joint_positions": mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"],
            scale=0.25, use_default_offset=True,
        ),
    }

    rewards = {
        "track_lin_vel":  RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5,
                                  params={"std": 0.25}),
        "track_ang_vel":  RewTerm(func=mdp.track_ang_vel_z_exp, weight=0.75,
                                  params={"std": 0.25}),
        "lin_vel_z":      RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0),
        "ang_vel_xy":     RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05),
        "torques":        RewTerm(func=mdp.joint_torques_l2, weight=-1e-5),
        "action_rate":    RewTerm(func=mdp.action_rate_l2, weight=-0.01),
    }

    terminations = {
        "time_out":        DoneTerm(func=mdp.time_out, time_out=True),
        "bad_orientation": DoneTerm(func=mdp.bad_orientation,
                                    params={"limit_angle": 0.5}),
    }

    events = {
        "push_robot": EventTerm(
            func=mdp.push_by_setting_velocity,
            mode="interval", interval_range_s=(10.0, 15.0),
            params={"velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}},
        ),
    }
```

#### mjlab 版本 ⭐⭐

> mjlab 与 IsaacLab 的 Manager API 高度对等。以下只展示命令和差异点。完整环境定义代码参见 `S3B_mjlab深度实战.md` 的 S3-B.2 节。

**完整训练命令:**

```bash
cd mjlab

# 训练 Go2 velocity tracking (4096 环境)
MUJOCO_GL=egl uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 5000
# RTX 4090 预期: ~1-2 小时收敛

# 评估 (Viser web viewer)
uv run play Mjlab-Velocity-Flat-Unitree-Go2-Play \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8
# 预期: 浏览器打开, 可用摇杆控制 Go2
```

**mjlab 环境配置的关键差异 (相对于 IsaacLab):**

```python
# mjlab 使用统一的 Entity 类 (IsaacLab 中是 Articulation/RigidObject 分类)
from mjlab.entities import EntityCfg
from mjlab.actuators import ActuatorCfg

robot = EntityCfg(
    mjcf_path="mujoco_menagerie/unitree_go2/go2.xml",  # MJCF 而非 USD
    actuators={
        "legs": ActuatorCfg(
            joint_names=["FR_.*", "FL_.*", "RR_.*", "RL_.*"],
            effort_limit=23.7,   # Nm
            velocity_limit=30.0, # rad/s
            stiffness=25.0,      # PD kp
            damping=0.5,         # PD kd
        ),
    },
)
# 其余 Manager 配置 (observations/rewards/terminations/events)
# 与 IsaacLab 几乎 1:1 对等
```

### 3.1.1 mjlab 环境开发完整流程 ⭐⭐

> 本节补充在 mjlab 中从零创建一个自定义训练环境的完整步骤。五层架构的深度解析请参阅 `S3B_mjlab深度实战.md`。

#### 五层架构回顾 ⭐

mjlab 的环境由五层组成，从底向上：

```
Simulation（MuJoCo Warp GPU）→ Entity（机器人/物体抽象）→ Scene（组合多个 Entity）
  → ManagerBasedRLEnv（6 个 Manager 定义 MDP）→ Task Registry（注册 + 命令行入口）
```

每层职责：
- **Simulation**: GPU 并行物理步进 + CUDA Graph 捕获
- **Entity**: MJCF 模型加载 + 执行器参数 + 批量化
- **Scene**: 组合机器人、地形、物体，编译为统一的 MjModel
- **Manager**: 定义 MDP 的六个维度 — 观测/动作/奖励/终止/事件/课程
- **Task Registry**: `register_mjlab_task()` 绑定配置，暴露 CLI 入口

#### 完整示例: 自定义 Go2 环境（从零开始） ⭐⭐

**文件结构:**

```
my_go2_task/
├── __init__.py          # 注册任务
├── scene_cfg.py         # Scene + Entity 配置
├── env_cfg.py           # Manager 配置 (MDP 定义)
├── rewards.py           # 自定义奖励函数 (可选)
└── rl_cfg.py            # PPO 超参数
```

**Step 1: 定义 EntityCfg 和 SceneCfg**

```python
# scene_cfg.py
from mjlab.scene import SceneCfg
from mjlab.entity import EntityCfg
from mjlab.actuator import ActuatorCfg
from mjlab.terrains import TerrainImporterCfg

class Go2SceneCfg(SceneCfg):
    num_envs: int = 4096
    env_spacing: float = 2.0

    robot = EntityCfg(
        mjcf_path="mujoco_menagerie/unitree_go2/go2.xml",
        actuators={
            "legs": ActuatorCfg(
                joint_names=["FR_.*", "FL_.*", "RR_.*", "RL_.*"],
                effort_limit=23.7,   # Nm
                velocity_limit=30.0, # rad/s
                stiffness=25.0,      # PD kp
                damping=0.5,         # PD kd
            ),
        },
    )

    terrain = TerrainImporterCfg(terrain_type="plane")
```

**Step 2: 定义 ManagerBasedRLEnvCfg（MDP 六维度）**

```python
# env_cfg.py
from mjlab.envs import ManagerBasedRLEnvCfg
from mjlab.managers import (
    ObservationGroupCfg, ObservationTermCfg,
    RewardTermCfg, TerminationTermCfg, EventTermCfg,
    ActionCfg,
)
import mjlab.envs.mdp as mdp
from .scene_cfg import Go2SceneCfg

class MyGo2EnvCfg(ManagerBasedRLEnvCfg):
    scene = Go2SceneCfg()

    # 观测管理器
    observations = {
        "policy": ObservationGroupCfg(terms={
            "base_ang_vel": ObservationTermCfg(func=mdp.base_ang_vel, scale=0.25),
            "projected_gravity": ObservationTermCfg(func=mdp.projected_gravity),
            "velocity_commands": ObservationTermCfg(func=mdp.velocity_commands),
            "joint_pos": ObservationTermCfg(func=mdp.joint_pos_rel, scale=1.0),
            "joint_vel": ObservationTermCfg(func=mdp.joint_vel, scale=0.05),
            "last_action": ObservationTermCfg(func=mdp.last_action),
        }),
    }

    # 动作管理器 (JointPositionAction)
    actions = {
        "joint_pos": ActionCfg(
            class_type=mdp.JointPositionAction,
            scale=0.25,
        ),
    }

    # 奖励管理器
    rewards = {
        "track_lin_vel_xy": RewardTermCfg(
            func=mdp.track_lin_vel_xy_exp, weight=1.0, params={"std": 0.25}),
        "track_ang_vel_z": RewardTermCfg(
            func=mdp.track_ang_vel_z_exp, weight=0.5, params={"std": 0.25}),
        "lin_vel_z_l2": RewardTermCfg(func=mdp.lin_vel_z_l2, weight=-2.0),
        "ang_vel_xy_l2": RewardTermCfg(func=mdp.ang_vel_xy_l2, weight=-0.05),
        "action_rate_l2": RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01),
        "joint_torques_l2": RewardTermCfg(func=mdp.joint_torques_l2, weight=-1e-5),
    }

    # 终止管理器
    terminations = {
        "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
        "bad_orientation": TerminationTermCfg(
            func=mdp.bad_orientation, params={"limit_angle": 0.5}),
    }

    # 事件管理器 (域随机化)
    events = {
        "reset_base": EventTermCfg(
            func=mdp.reset_root_state_uniform, mode="reset",
            params={"pos_range": [0.0, 0.0], "vel_range": [-0.5, 0.5]}),
        "push_robot": EventTermCfg(
            func=mdp.push_by_setting_velocity, mode="interval",
            interval_range=(10, 15),
            params={"velocity_range": {"x": [-0.5, 0.5], "y": [-0.5, 0.5]}}),
    }
```

**Step 3: 注册任务**

```python
# __init__.py
from mjlab.tasks.registry import register_mjlab_task
from .env_cfg import MyGo2EnvCfg

# Play 模式的配置（少量 envs, 关闭 DR）
play_cfg = MyGo2EnvCfg()
play_cfg.scene.num_envs = 8
play_cfg.events = {}  # 评估时关闭域随机化

register_mjlab_task(
    task_id="MyCustomTask-Go2",
    env_cfg=MyGo2EnvCfg(),
    play_env_cfg=play_cfg,
    rl_cfg=Go2PPOCfg(),       # RSL-RL PPO 配置
)
```

**Step 4: 训练和评估**

```bash
# 训练
MUJOCO_GL=egl uv run train MyCustomTask-Go2 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 5000

# 评估
uv run play MyCustomTask-Go2 \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8
```

> **与 S3B 的关系**: 本节提供了快速上手的配置骨架。五层架构每一层的源码精读、数据流分析、和 IsaacLab 的逐行对比，请参阅 `S3B_mjlab深度实战.md` 的 S3-B.1 至 S3-B.2 节。

### 3.2 奖励函数设计与调参 ⭐⭐

#### 每个奖励项的含义和典型权重 ⭐⭐

四足 locomotion 的奖励函数是**多项加权和**: $r = \sum_i w_i \cdot r_i$

包含 2-3 个正向追踪奖励和 8-15 个负向正则化项:

| 类别 | 奖励项 | 作用 | 典型权重 | 备注 |
|------|--------|------|---------|------|
| **追踪** | `track_lin_vel_xy_exp` | 跟踪目标线速度 (xy) | **1.0 ~ 1.5** | 核心任务奖励, 用 exp 形式 |
| **追踪** | `track_ang_vel_z_exp` | 跟踪目标角速度 (z) | **0.5 ~ 0.75** | 转向能力 |
| **正则** | `lin_vel_z_l2` | 抑制上下弹跳 | -2.0 | 权重大 = 压制弹跳行为 |
| **正则** | `ang_vel_xy_l2` | 抑制 roll/pitch 摇晃 | -0.05 | 保持身体水平 |
| **正则** | `flat_orientation_l2` | 保持身体水平 | -0.5 | 与 ang_vel_xy 互补 |
| **正则** | `joint_torques_l2` | 减少力矩消耗 | -1e-5 ~ -1e-4 | 权重小, 仅做正则 |
| **正则** | `action_rate_l2` | 动作平滑 | -0.01 | 抑制抖动的关键项 |
| **正则** | `joint_acc_l2` | 关节加速度惩罚 | -2.5e-7 | 进一步平滑 |
| **正则** | `dof_pos_limits` | 关节接近限位惩罚 | -10.0 | 安全项, 权重大 |
| **正则** | `undesired_contacts` | 大腿/小腿碰地惩罚 | -1.0 | 只允许脚掌接触 |
| **鼓励** | `feet_air_time` | 鼓励足端腾空 | 0.125 ~ 1.0 | 防止拖脚, 促进步态 |

**动作空间设计:**

RL 输出 12 个目标关节角偏移, 加上默认站姿后由 PD 控制器执行:

$$\tau_j = k_p (q_\text{default} + \Delta q_j \times \text{scale} - q_j) + k_d (0 - \dot{q}_j)$$

典型参数: $k_p = 20 \sim 50$, $k_d = 0.5 \sim 1.0$, action scale $= 0.25$ rad.

#### 调参策略 ⭐⭐

**第一原则: 先跑 baseline, 再逐项调整**

```
Step 1: 跑 baseline
  使用上表中的典型权重, 训练 3000 iterations, 看 tensorboard

Step 2: 判断主导奖励项
  看哪个奖励项的曲线值最大 → 它主导了策略行为
  如果正则项绝对值 >> 追踪项 → 策略"不敢动" → 降正则权重
  如果追踪项很大但正则项很小 → 可能抖动/粗暴 → 增正则权重

Step 3: 调一个参数, 跑对比实验
  每次只改一个权重, 对比前后差异
  保持追踪奖励量级 ~1, 正则项量级 ~0.01
```

**常见问题与对应调参:**

| 症状 | 原因分析 | 调哪个参数 |
|------|---------|-----------|
| 机器人剧烈抖动 | action_rate 惩罚不够 | 增大 `action_rate_l2` 权重 (如 -0.01 → -0.05) |
| 机器人原地不动 | 追踪奖励太弱或正则太强 | 增大 `track_lin_vel` 权重; 减小正则项权重 |
| 机器人摔倒频繁 | orientation 惩罚不够; 或 PD 增益不对 | 增大 `flat_orientation_l2`; 检查 kp/kd |
| 机器人弹跳/跳着走 | `lin_vel_z` 惩罚不够 | 增大 `lin_vel_z_l2` (如 -2.0 → -5.0) |
| 机器人拖脚滑行 | 缺少 `feet_air_time` 奖励 | 添加或增大 feet_air_time 权重 |
| 转弯不灵活 | `track_ang_vel_z` 权重太低 | 增大到 0.75 ~ 1.0 |
| 行走能耗过高 | 缺少力矩/功率惩罚 | 增大 `joint_torques_l2` (如 -1e-5 → -1e-4) |
| 关节接近限位 | `dof_pos_limits` 未配置 | 添加, 权重设 -5.0 ~ -10.0 |
| 步态不对称 | 缺乏对称性约束 | 可添加 `hip_symmetry` 自定义奖励 |

**自定义奖励函数模板:**

```python
def my_energy_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """能量效率奖励: 惩罚关节功率"""
    robot = env.scene[asset_cfg.name]
    # 关节力矩 x 关节速度 = 功率
    power = torch.sum(
        torch.abs(robot.data.applied_torque * robot.data.joint_vel),
        dim=-1
    )
    return -power  # shape (num_envs,)

# 在 rewards 字典中添加:
# "energy": RewTerm(func=my_energy_reward, weight=-1e-4,
#                   params={"asset_cfg": SceneEntityCfg("robot")}),
```

### 3.2.1 mjlab 自定义奖励函数编写 ⭐⭐⭐

> mjlab 的奖励函数 API 与 IsaacLab 高度对等，但底层数据访问通过 MuJoCo 的 `mjData` 结构。本节详细说明如何在 mjlab 中编写自定义奖励。

#### RewardTermCfg API ⭐⭐⭐

mjlab 的奖励管理器 (`RewardManager`) 接受一组 `RewardTermCfg`，每个 term 定义一个奖励函数及其权重：

```python
from mjlab.managers import RewardTermCfg

# 奖励 term 的标准签名
RewardTermCfg(
    func=my_reward_function,   # Callable，返回 shape (num_envs,) 的 tensor
    weight=1.0,                # 奖励权重（正=鼓励，负=惩罚）
    params={"key": value},     # 传递给 func 的额外参数
)
```

**奖励函数的签名要求:**

```python
def my_reward_function(
    env: ManagerBasedRLEnv,
    **kwargs,        # 来自 RewardTermCfg.params
) -> torch.Tensor:
    """返回 shape (num_envs,) 的奖励值"""
    ...
```

#### 访问 MuJoCo 数据 ⭐⭐

mjlab 的 Entity 提供了对底层 MuJoCo 数据的零拷贝 PyTorch tensor 访问：

```python
# 获取机器人 Entity
robot = env.scene["robot"]

# 常用数据访问 (均为 GPU tensor, shape 含 num_envs 维度)
robot.data.joint_pos          # 关节位置 (num_envs, num_joints)
robot.data.joint_vel          # 关节速度 (num_envs, num_joints)
robot.data.applied_torque     # 施加力矩 (num_envs, num_joints)
robot.data.root_lin_vel_b     # 基座线速度 body frame (num_envs, 3)
robot.data.root_ang_vel_b     # 基座角速度 body frame (num_envs, 3)
robot.data.projected_gravity  # 重力在 body frame 投影 (num_envs, 3)
```

#### 示例: 自定义足端接触奖励 ⭐⭐⭐

```python
import torch
from mjlab.envs import ManagerBasedRLEnv
from mjlab.managers import SceneEntityCfg

def foot_contact_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces"),
    threshold: float = 1.0,
) -> torch.Tensor:
    """鼓励足端与地面接触——接触力 > threshold 时给正奖励"""
    contact_sensor = env.scene[sensor_cfg.name]
    # 获取足端接触力 (num_envs, num_feet, 3)
    foot_forces = contact_sensor.data.net_forces_w
    # 计算每只脚的接触力范数
    foot_force_norms = torch.norm(foot_forces, dim=-1)  # (num_envs, num_feet)
    # 有接触的脚数量
    num_contacts = torch.sum(foot_force_norms > threshold, dim=-1).float()
    return num_contacts  # (num_envs,)

# 注册到奖励管理器:
# "foot_contact": RewardTermCfg(
#     func=foot_contact_reward,
#     weight=0.1,
#     params={"threshold": 1.0},
# ),
```

#### 示例: 自定义能量效率惩罚 ⭐⭐⭐

```python
def energy_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """惩罚关节机械功率: P = |tau * dq/dt|"""
    robot = env.scene[asset_cfg.name]
    power = torch.sum(
        torch.abs(robot.data.applied_torque * robot.data.joint_vel),
        dim=-1,
    )
    return -power  # (num_envs,) — 负值因为是惩罚

# 注册:
# "energy": RewardTermCfg(func=energy_penalty, weight=-1e-4),
```

#### 调试奖励: 打印各项奖励值 ⭐⭐

mjlab 的 wandb 集成会自动记录每个 reward term 的值。在 wandb dashboard 的 `rewards/` 面板下可以看到每个 term 的独立曲线。

如需本地调试，可以在短期训练中观察日志输出：

```bash
# 小规模训练，观察奖励分解
WANDB_MODE=offline uv run train MyTask \
    --env.scene.num-envs 64 \
    --agent.max-iterations 20
# 查看终端输出中各项 reward 的量级
# 正常: 追踪奖励量级 ~1.0, 正则项量级 ~0.01
# 异常: 某个正则项量级 >> 追踪项 → 策略会"不敢动"
```

**奖励调试速查:**

| 症状 | 排查方向 | 解决方案 |
|------|---------|---------|
| 自定义奖励值全为 0 | 数据访问路径不对 | 用 `--agent zero` 检查数据是否有值 |
| 奖励值为 NaN | 除零或物理爆炸 | 加 `torch.clamp` 或检查仿真稳定性 |
| 自定义奖励淹没了追踪奖励 | 权重/量级不平衡 | 先打印 raw value 再调权重 |
| 奖励函数报 shape 错误 | 返回值维度不对 | 确保返回 `(num_envs,)` 而非 `(num_envs, 1)` |

### 3.3 Domain Randomization 配置 ⭐⭐⭐

#### 关键参数表 ⭐⭐

| 随机化维度 | 参数 | 典型范围 | 重要性 |
|-----------|------|---------|-------|
| **电机** | 控制延迟 | [0, 20] ms | **最高** — 不加延迟 DR, 真机几乎必然失败 |
| **电机** | 电机强度 (scale) | [0.85, 1.15] | **高** |
| **电机** | PD 增益 kp | kp x [0.9, 1.1] | **高** |
| **刚体** | 质量 | 基准值 x [0.8, 1.2] | **高** |
| **刚体** | 摩擦系数 | [0.5, 1.25] | **高** |
| **刚体** | CoM 偏移 | [-2, 2] cm 各轴 | 中 |
| **刚体** | 弹性恢复系数 | [0.0, 0.5] | 中 |
| **噪声** | IMU 角速度噪声 | +/-0.2 rad/s | 中 |
| **噪声** | 编码器角度噪声 | +/-0.01 rad | 中 |
| **扰动** | 外力推动 | 每 5-15s, 0-1.0 m/s | 中 |
| **地形** | 地面坡度 | [-0.1, 0.1] rad | 中 |

#### IsaacLab DR 配置示例 ⭐⭐⭐

```python
events = {
    # 每次 reset 时随机化基座位姿
    "reset_base": EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {}, "velocity_range": {}},
    ),
    # 每次 reset 时随机化关节位置
    "reset_joints": EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={"position_range": [-0.1, 0.1], "velocity_range": [-0.1, 0.1]},
    ),
    # 持续随机化摩擦
    "physics_material": EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="interval",
        interval_range=(250, 750),  # 仿真步
        params={"friction_range": [0.5, 1.25]},
    ),
    # 外力扰动
    "push_robot": EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range=(10, 15),  # 秒
        params={"velocity_range": {"x": [-0.5, 0.5], "y": [-0.5, 0.5]}},
    ),
    # 质量随机化
    "randomize_mass": EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={"mass_distribution_params": [0.8, 1.2], "operation": "scale"},
    ),
}
```

#### mjlab DR 配置 (Rucker 伪惯量参数化) ⭐⭐⭐

mjlab 使用 Rucker & Wensing (RA-L 2022) 的伪惯量参数化, 从数学上保证随机化后的 (质量, 质心, 惯量) 组合物理合法:

```python
events = {
    # Rucker 伪惯量随机化 (质量 + 质心 + 惯量 联合随机化)
    "randomize_inertia": EventTermCfg(
        func=mdp.randomize_rigid_body_inertia_rucker,
        mode="reset",
        params={
            "mass_range": [0.8, 1.2],
            "com_range": [-0.05, 0.05],  # 质心偏移 (米)
        },
    ),
    # 摩擦随机化
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
```

> IsaacLab 的 `randomize_rigid_body_mass` 和 `randomize_rigid_body_inertia` 是分别操作的, 可能产生不一致的参数组合。mjlab 的 Rucker 参数化一步到位。

#### 从零 DR 到全 DR 的渐进策略 ⭐⭐

```
Phase 1: 无 DR (调试阶段)
  关闭所有随机化, 确认策略能在标称参数下收敛
  目的: 验证奖励设计和环境配置正确

Phase 2: 最小 DR
  仅加延迟随机化 [0, 10] ms + 轻微质量 [0.95, 1.05]
  目的: 检查策略是否对微小变化鲁棒

Phase 3: 标准 DR
  加完整参数表中的所有项, 范围取中值
  目的: 准备 sim2real

Phase 4: 强 DR (可选)
  增大随机化范围 (如质量 [0.7, 1.3])
  加外力扰动和地形变化
  目的: 提高真机部署鲁棒性
  注意: 过强的 DR 会导致策略过于保守
```

### 3.4 训练监控与调试 ⭐⭐

#### TensorBoard 关键曲线解读 ⭐⭐

```bash
# 启动 tensorboard
tensorboard --logdir logs/rsl_rl/

# IsaacLab 日志默认路径:
#   logs/rsl_rl/<task_name>/<timestamp>/
# mjlab 日志通常在 W&B, 也可用 tensorboard
```

**必看曲线与正常范围:**

| 曲线 | 正常行为 | 异常信号 |
|------|---------|---------|
| `rewards/total` | 前 500 iter 快速上升, 1000-3000 iter 缓慢收敛 | 持续下降 = 奖励设计有 bug; 突然崩塌 = KL 发散 |
| `rewards/track_lin_vel_xy` | 收敛到 0.7-0.9 | < 0.3 = 策略没学会跟踪速度 |
| `rewards/action_rate_l2` | 绝对值逐渐减小 | 持续增大 = 策略越来越抖 |
| `policy/mean_std` | 从 ~1.0 逐渐下降到 0.1-0.3 | 不下降 = 策略没收敛; 降到 0 = exploration 停止 |
| `policy/learning_rate` | adaptive KL: 在 1e-3 ~ 1e-5 之间波动 | 骤降到 1e-7+ = KL 过大, 可能崩了 |
| `policy/kl` | 在 desired_kl (0.01) 附近波动 | 持续远大于 desired_kl = 不稳定 |
| `episode/length` | 逐渐增长到接近 max_episode_length | 始终很短 = 策略一直摔倒 |

#### 什么是"训练崩了" vs "正常探索" ⭐⭐

```
正常探索:
  - reward 波动但总趋势上升
  - episode_length 偶尔短暂下降后恢复
  - KL 在 0.005-0.02 范围波动

训练崩了:
  - reward 持续大幅下降 (50%+), 不恢复
  - KL 突然飙到 0.1+
  - learning_rate 骤降 (被 adaptive KL 急刹)
  - episode_length 全部掉到 < 1 秒

处理方式:
  如果崩了 → 回滚到上一个好的 checkpoint, 调低 learning_rate 或减小 clip_range
  如果只是震荡 → 正常, 等待收敛
```

#### 常见训练失败模式和修复方法 ⭐⭐

| 失败模式 | 表现 | 根因 | 修复 |
|---------|------|------|------|
| 策略不收敛 | reward 不上升, episode_length 始终短 | 奖励量级不平衡或环境 bug | 检查各奖励项量级; 用少量环境 (64) 单步调试 |
| NaN 出现 | 训练中断, loss = NaN | 物理仿真爆炸或观测未归一化 | 加 `clip_observations`; 降低 action_scale |
| 策略震荡 | reward 反复上下波动 | learning_rate 太大或 batch 太小 | 降低 lr; 增大 num_envs 或 num_steps |
| 过拟合到一种行为 | 只向前走, 不会转弯或后退 | 速度命令分布不均匀 | 检查 command generator 是否包含 vy 和 omega_z |
| 策略在 eval 时表现差 | 训练 reward 高但评估不行 | 训练时利用了 DR 的特定分布 | 加大 DR 范围; 多 seed 评估 |
| OOM (显存不足) | CUDA out of memory | num_envs 太多 | 减少 num_envs; 减小 network hidden_dims |

**PPO 超参数速查 (rsl_rl 标准配置):**

```python
class PPOCfg:
    num_envs = 4096
    num_steps_per_rollout = 24
    # batch_size = 4096 x 24 = 98304
    num_mini_batches = 4
    num_epochs = 5
    learning_rate = 1e-3        # rsl_rl 默认 adaptive KL 调整
    gamma = 0.99
    lam = 0.95                  # GAE lambda
    clip_range = 0.2
    entropy_coef = 0.01
    desired_kl = 0.01           # adaptive KL 目标
    max_grad_norm = 1.0
    # rsl_rl 中 Actor 和 Critic 使用独立 MLP
    actor_hidden_dims = [512, 256, 128]
    critic_hidden_dims = [512, 256, 128]
    activation = "elu"
```

---

## Part 4: 人形训练实战 (H1/G1)

### 4.1 人形 vs 四足的配置差异 ⭐⭐

| 维度 | 四足 (Go2) | 人形 (H1) | 人形 (G1) |
|------|-----------|-----------|-----------|
| 驱动关节数 | 12 | 19 | 23+ |
| 动作空间维度 | 12 | 19 | 23+ |
| 观测空间维度 | ~45 | ~70 | ~80+ |
| 支撑点 | 4 (quasi-static 稳定) | 2 (动态不稳定) | 2 |
| 典型训练时间 (4090) | 2-4 小时 | 24-72 小时 | 48-96 小时 |
| 训练成功率 (不同 seed) | 高 (~90%) | 中 (~50-70%) | 低 (~30-50%) |
| 默认 PD 增益 (kp) | 20-50 | 150-300 (腿), 50-100 (臂) | 类似 |

**人形比四足更难的原因:**
1. 动态不稳定 — 2 个支撑点, 必须持续主动控制
2. 动作空间更大 — 搜索空间指数增长
3. 手臂对平衡有影响 — 需要全身协调而非仅控制腿部

### 4.2 H1 行走训练完整流程 ⭐⭐

#### IsaacLab 版本 ⭐⭐

```bash
# 训练 H1 velocity tracking
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-H1-v0 \
    --num_envs 4096 \
    --max_iterations 10000 \
    --headless
# RTX 4090 预期: 24-72 小时

# 评估
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-H1-v0 \
    --num_envs 8 \
    --checkpoint logs/rsl_rl/Isaac-Velocity-Flat-Unitree-H1-v0/*/model_best.pt
```

#### mjlab 版本 ⭐⭐

```bash
# 训练 H1 velocity tracking
MUJOCO_GL=egl uv run train Mjlab-Velocity-Flat-Unitree-H1 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 10000

# 评估
uv run play Mjlab-Velocity-Flat-Unitree-H1-Play \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8
```

**人形训练的关键配置差异:**

```python
# 人形 vs 四足的配置差异点
class H1EnvCfg(ManagerBasedRLEnvCfg):
    # 仿真步长和 decimation 通常与四足相同
    sim = SimulationCfg(dt=0.005, decimation=4)

    # 观测: 比四足多了上半身关节角
    observations = {
        "policy": ObsGroup(obs_terms={
            "joint_pos":   ObsTerm(func=mdp.joint_pos_rel),  # 19 维 (含手臂)
            "joint_vel":   ObsTerm(func=mdp.joint_vel_rel),  # 19 维
            "base_ang_vel": ObsTerm(func=mdp.base_ang_vel),  # 3 维
            "projected_gravity": ObsTerm(func=mdp.projected_gravity),
            "velocity_commands": ObsTerm(func=mdp.generated_commands,
                                        params={"command_name": "base_velocity"}),
            "last_action": ObsTerm(func=mdp.last_action),    # 19 维
        }),
    }

    # PD 增益: 人形电机更大, 需要更高 kp
    actions = {
        "joint_positions": mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"],
            scale=0.25, use_default_offset=True,
            # 不同关节组可能需要不同 PD 增益
        ),
    }
```

### 4.3 奖励函数差异: 平衡/步态/能耗 ⭐⭐⭐

**人形额外需要的奖励项:**

```python
# 在四足 baseline 奖励的基础上, 添加:

rewards_humanoid_extra = {
    # 躯干直立惩罚 (人形核心!)
    "upright": RewTerm(
        func=mdp.flat_orientation_l2, weight=-2.0,
        # 或自定义: -(1 - cos(trunk_pitch))
    ),
    # 重心高度维持
    "com_height": RewTerm(
        func=my_com_height_reward, weight=-1.0,
        params={"target_height": 0.98},  # H1 站立高度 ~0.98m
    ),
    # 步频追踪 (可选, 但推荐)
    "step_frequency": RewTerm(
        func=my_step_freq_reward, weight=0.5,
        params={"target_freq": 1.5},  # ~1.5 Hz 步频
    ),
    # 手臂正则化 (防止手臂乱甩)
    "arm_regularization": RewTerm(
        func=my_arm_default_pose, weight=-0.1,
        # 惩罚手臂偏离默认垂放位置
    ),
    # 脚步接触时序 (鼓励交替落地)
    "feet_contact_schedule": RewTerm(
        func=my_feet_schedule, weight=0.5,
    ),
}
```

**步态时钟 (gait clock) 技巧 (He et al., 2024):**

在观测中加入周期性相位信号, 明确告诉策略当前处于步态的哪个阶段:

```python
# 添加到 observations
"gait_phase": ObsTerm(
    func=my_gait_clock,
    # 输出: [sin(2*pi*f*t), cos(2*pi*f*t)]
    # f = 目标步频 (如 1.5 Hz)
)
```

这个技巧可以大幅加速人形训练收敛并改善步态质量。

### 4.4 常见问题: 不平衡/步态不自然/关节抖动 ⭐⭐

| 症状 | 原因分析 | 解决方案 |
|------|---------|---------|
| 站不稳, 持续摔倒 | PD 增益不够; orientation 惩罚太弱 | 增大脚踝 kp (如 150→300); 增大 upright 惩罚权重 |
| 能站但不走 | 追踪奖励太弱; 摔倒终止过于敏感 | 增大 track_lin_vel 权重; 放宽 bad_orientation 的 limit_angle |
| 步态不自然 (僵硬/像僵尸) | 手臂没有自然摆动; 步频不对 | 添加 arm_swing 奖励; 添加 gait_clock 观测 |
| 上半身剧烈摇摆 | ang_vel_xy 惩罚不够 | 增大 ang_vel_xy 权重; 添加 torso_orientation 惩罚 |
| 关节抖动 (高频震荡) | action_rate 惩罚不够; 或仿真步长太大 | 增大 action_rate; 减小 sim_dt (如 0.005→0.002) |
| 膝关节过伸 | 关节限位惩罚不够 | 增大 dof_pos_limits 权重; 在 URDF/MJCF 中硬限制膝关节范围 |
| 训练不收敛 (10000 iter 后 reward 仍低) | 人形本来就难; 可能需要 curriculum | 先在平地训练; 降低速度命令范围 (如 max 0.5 m/s); 增大 num_envs |
| 左右步态不对称 | 初始化不对称或奖励不对称 | 添加 hip_symmetry 惩罚: $-\|q_\text{left} - (-q_\text{right})\|^2$ |

**人形训练的实用建议:**

1. **多 seed**: 至少跑 3-5 个 seed, 人形训练方差大
2. **先站后走**: 可以先训练 standing policy, 再用 curriculum 逐步增加速度
3. **手臂可以先锁定**: 初期调试时固定手臂, 减小动作空间, 先确保腿部策略收敛
4. **减小速度范围**: 初始 cmd 范围设小 (如 [0, 0.5] m/s), 确认能走再增大

---

## Part 5: 动作模仿实战

> 本节只讲 HOW to configure, 不讲算法原理。需要理论背景 (AMP GAN loss, ASE latent space, DeepMimic reward math) 请参阅原始论文。

### 5.1 MoCap 数据获取与预处理 ⭐⭐

#### 数据来源 ⭐

| 数据集 | 规模 | 格式 | 获取方式 | 适用场景 |
|--------|------|------|---------|---------|
| **CMU MoCap** | ~2500 段, ~6 小时 | BVH/C3D | 免费下载 | 基础行走/跑步 |
| **AMASS** | ~12,000 段, ~40 小时 | SMPL 参数 | 注册下载 | **推荐**: 最大统一格式数据集 |
| **100STYLE** | ~100 种风格 | BVH | 免费 | AMP 多风格训练 |
| **LaFAN1** | ~500k 帧 | BVH | 免费 | 高质量过渡动作 |
| **自采集** | 自定义 | 视频/IMU → SMPL | 需要工具链 | 特定动作需求 |

#### Retargeting 工具链和流程 ⭐⭐

将人体 MoCap 运动映射到机器人:

```
Step 1: 数据格式统一
  BVH/C3D → SMPL 参数 (使用 AMASS 工具链)
  SMPL 参数: θ ∈ R^72 (24 关节旋转), β ∈ R^10 (体型)

Step 2: 骨架映射
  定义 SMPL 关节 → 机器人关节 的对应关系
  例: SMPL.left_hip → H1.left_hip_yaw + left_hip_roll + left_hip_pitch

Step 3: IK 求解
  给定 SMPL 末端位置 (脚/手) → IK 求解机器人关节角
  工具: poselib (NVIDIA), dm_control IK, 自定义 IK

Step 4: 关节限制裁剪 + 平滑
  clip(q_retarget, q_min, q_max)
  时域低通滤波消除 retarget 跳变

Step 5: 仿真验证
  在 MuJoCo viewer 中回放 retarget 结果
  检查: 不穿模, 不飘浮, 脚正确接地
```

**mjlab G1 spinkick 示例的数据流程:**

```bash
# 来自 mujocolab/g1_spinkick_example (~213 stars)

# 1. MimicKit pkl → CSV (加过渡动作 + 循环)
python pkl_to_csv.py --input motion.pkl --output motion.csv

# 2. CSV → NPZ (重采样到 50 fps + 上传 W&B Registry)
python csv_to_npz.py --input motion.csv --output motion.npz --fps 50

# 3. 上传参考动作到 W&B
wandb artifact put --type motion-data motion.npz
```

### 5.2 AMP 训练配置 ⭐⭐⭐

> AMP (Adversarial Motion Priors, Peng et al. 2021) 用 discriminator 自动学习"什么是自然运动", 无需手工设计 motion tracking 奖励。以下只讲怎么配。

**总奖励结构:**

```
total_reward = w_task * r_task + w_style * r_style

其中:
  r_task  = 任务奖励 (如速度追踪, 与普通 locomotion 相同)
  r_style = discriminator 输出 (自动学习)
```

**IsaacGymEnvs 中 AMP 的关键配置:**

```yaml
# 在 IsaacGymEnvs 的 AMP 配置文件中
# 路径: cfg/train/HumanoidAMPPPO.yaml

params:
  config:
    # Discriminator 配置
    disc_hidden_units: [1024, 512]    # discriminator MLP 大小
    disc_logit_reg: 0.05              # logit 正则化
    disc_grad_penalty: 10.0           # gradient penalty 权重
    disc_weight_decay: 0.0001         # weight decay
    disc_reward_scale: 2.0            # style reward 缩放

    # Style reward 与 task reward 权重
    task_reward_weight: 0.5           # r_task 权重
    disc_reward_weight: 0.5           # r_style 权重

    # 参考运动数据
    amp_motion_files:
      - "data/motions/walk_forward.npy"
      - "data/motions/run_forward.npy"
      # 可加多个文件 → discriminator 学习整个分布

    # Discriminator 观测 (用于区分真/假运动)
    amp_obs_size: 105                 # 状态转移 (s_t, s_{t+1}) 的维度
```

**Discriminator 调参要点:**

| 参数 | 作用 | 调参建议 |
|------|------|---------|
| `disc_reward_scale` | 放大 style reward | 从 2.0 开始; 如果策略忽略风格 → 增大; 如果忽略任务 → 减小 |
| `disc_grad_penalty` | 稳定 discriminator 训练 | 10.0 是标准值; 如果 discriminator loss 震荡 → 增大 |
| `task_reward_weight` : `disc_reward_weight` | 任务 vs 风格的平衡 | 0.5:0.5 是常见起点; 需要更精确追踪 → 增大 task; 需要更自然 → 增大 disc |
| `disc_hidden_units` | discriminator 容量 | [1024, 512] 足够; 更多运动风格 → 可增大 |
| `amp_motion_files` | 参考运动数据 | 数据越多, 学到的风格分布越广 |

**参考运动文件格式:**

```python
# AMP 参考运动通常是 numpy 数组
# shape: (num_frames, state_dim)
# state_dim 包含: root_pos(3) + root_rot(4) + joint_angles(n) + 速度信息
import numpy as np
motion = np.load("walk_forward.npy")
print(motion.shape)  # e.g., (500, 105)
```

**开源 AMP 实现:**
- `NVIDIA-Omniverse/IsaacGymEnvs` — Isaac Gym 版本 (最成熟)
- Isaac Lab 可通过 Manager API 实现 AMP (需要自行添加 discriminator 训练循环)

### 5.3 Motion Tracking 训练 (BeyondMimic 风格) ⭐⭐⭐

> Motion tracking 直接追踪参考运动的每一帧, 比 AMP 更精确但需要精确的 retarget 数据。

#### 参考运动 CSV/NPZ 格式 ⭐⭐

```python
# mjlab motion tracking 的参考运动格式
# NPZ 文件包含:
import numpy as np
data = np.load("reference_motion.npz")

# 必须字段:
data["joint_positions"]    # shape: (T, num_joints) — 关节角轨迹
data["root_position"]      # shape: (T, 3) — 根节点位置
data["root_orientation"]   # shape: (T, 4) — 根节点四元数
data["dt"]                 # scalar — 采样间隔 (如 0.02 = 50fps)

# 可选字段:
data["joint_velocities"]   # shape: (T, num_joints) — 关节角速度
data["root_linear_vel"]    # shape: (T, 3) — 根节点线速度
data["end_effector_pos"]   # shape: (T, num_ee, 3) — 末端位置
```

#### Motion Tracking 奖励配置 ⭐⭐⭐

```python
# BeyondMimic 风格的 tracking reward 配置
rewards = {
    # 关节角跟踪 (核心)
    "joint_tracking": RewTerm(
        func=mdp.joint_position_tracking_exp,
        weight=1.0,
        params={"std": 0.1},  # tracking 精度, 越小越严格
    ),
    # 末端位置跟踪
    "end_effector_tracking": RewTerm(
        func=mdp.end_effector_tracking_exp,
        weight=0.5,
        params={"std": 0.05},  # 米
    ),
    # 基座位姿跟踪
    "root_pose_tracking": RewTerm(
        func=mdp.root_pose_tracking_exp,
        weight=0.3,
    ),
    # 基座速度跟踪
    "root_vel_tracking": RewTerm(
        func=mdp.root_velocity_tracking_exp,
        weight=0.2,
    ),
    # 正则化项 (与 velocity tracking 相同)
    "action_rate":    RewTerm(func=mdp.action_rate_l2, weight=-0.01),
    "joint_torques":  RewTerm(func=mdp.joint_torques_l2, weight=-1e-5),
}
```

#### G1 Spinkick 完整训练流程 ⭐⭐

```bash
# 来自 mujocolab/g1_spinkick_example

# 1. 准备参考动作 (pkl → csv → npz)
python pkl_to_csv.py --input motion.pkl --output motion.csv
python csv_to_npz.py --input motion.csv --output motion.npz --fps 50

# 2. 上传到 W&B Registry
wandb artifact put --type motion-data motion.npz

# 3. 训练 (单 GPU, ~1-2 小时, 20000 iterations)
uv run train Mjlab-Spinkick-Unitree-G1 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 20000

# 4. 评估
uv run play Mjlab-Spinkick-Unitree-G1-Play \
    --wandb-run-path your-org/mjlab/run-id

# 5. ONNX 导出 (自动存储到 W&B artifacts)
# 训练完成后自动导出, 或手动:
uv run export Mjlab-Spinkick-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id
```

**BeyondMimic 的渐进式课程 (关键配置):**

```python
# 训练初期: 宽松惩罚, 让策略先学会大致动作
# 训练后期: 逐步收紧, 提高动作质量

curriculum = {
    "action_smoothness": CurriculumTermCfg(
        func=mdp.linear_curriculum,
        params={
            "start_weight": -0.001,   # 初期: 轻微平滑惩罚
            "end_weight": -0.01,      # 后期: 收紧
            "ramp_iterations": 10000,
        },
    ),
    "joint_velocity_limit": CurriculumTermCfg(
        func=mdp.linear_curriculum,
        params={
            "start_weight": -0.0001,
            "end_weight": -0.001,
            "ramp_iterations": 10000,
        },
    ),
    "power_penalty": CurriculumTermCfg(
        func=mdp.linear_curriculum,
        params={
            "start_weight": 0.0,
            "end_weight": -0.0005,
            "ramp_iterations": 15000,
        },
    ),
}
```

> 为什么不从一开始就加强惩罚? 过早的惩罚限制了策略探索空间——策略先学"怎么做这个动作"(粗糙但有效), 再学"怎么优雅地做"(平滑高效)。

### 5.3.1 g1_spinkick_example 完整解析 ⭐⭐⭐

> 来自 `mujocolab/g1_spinkick_example` (~213 stars)。这是目前 mjlab 生态中最完整的 motion imitation → 真机部署示例。

#### Clone 与安装 ⭐

```bash
git clone https://github.com/mujocolab/g1_spinkick_example.git
cd g1_spinkick_example
uv sync
```

#### 参考动作数据结构 ⭐⭐

参考动作来自 Jason Peng 的 [MimicKit](https://github.com/xbpeng/MimicKit)。原始格式为 pkl，包含：

```python
# pkl 文件结构:
{
    "frames": np.array,  # shape: (N, 3+3+29) = (N, 35)
                         # 3: root_pos (x,y,z)
                         # 3: root_rot (axis-angle)
                         # 29: joint DoF (G1 全身)
    "fps": 60,           # 原始采样率
}
```

#### 数据转换管线 ⭐⭐

```bash
# Step 1: pkl → csv (加安全过渡动作 + 循环)
uv run pkl_to_csv.py \
    --pkl-file g1_spinkick.pkl \
    --csv-file g1_spinkick.csv \
    --duration 2.65 \
    --add-start-transition \       # 从安全站姿平滑过渡到动作起始
    --add-end-transition \         # 从动作末尾平滑回到安全站姿
    --transition-duration 0.5 \    # 过渡时长 0.5 秒
    --pad-duration 1.0             # 末尾静止保持 1 秒

# Step 2: csv → npz (重采样 + 上传 W&B Registry)
MUJOCO_GL=egl uv run -m mjlab.scripts.csv_to_npz \
    --input-file g1_spinkick.csv \
    --output-name mimickit_spinkick_safe \
    --input-fps 60 \
    --output-fps 50 \
    --render True
# 输出: NPZ 文件 + 参考动作回放视频
# 自动上传到 wandb registry
```

**过渡动作的意义**: `pkl_to_csv.py` 使用 cubic ease-in/ease-out 插值在安全站姿和动作之间创建平滑过渡。这对真机部署至关重要——避免策略在起始帧产生突变力矩。

#### 训练配置要点 ⭐⭐

```bash
# 训练命令 (单 GPU, 4096 envs, ~1-2 小时)
MUJOCO_GL=egl CUDA_VISIBLE_DEVICES=0 uv run train \
    Mjlab-Spinkick-Unitree-G1 \
    --registry-name your-org/registry-name/mimickit_spinkick_safe \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 20000
```

**预期训练收敛:**
- ~2000 iterations: 策略开始尝试踢腿动作
- ~8000 iterations: 基本动作形态出现
- ~15000 iterations: 动作趋于稳定和流畅
- ~20000 iterations: 收敛，总训练时间约 1-2 小时 (单 RTX 4090)

#### 评估与可视化 ⭐⭐

```bash
# Viser 可视化评估
uv run play Mjlab-Spinkick-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8
```

#### 适配其他动作 ⭐⭐⭐

替换参考动作数据即可复用整个管线：

```
1. 获取新动作的 pkl/csv/BVH 数据
2. 转换为 csv 格式 (root_pos + root_quat_xyzw + joint_dof)
3. 使用 csv_to_npz 重采样到 50 fps
4. 上传到 wandb registry
5. 修改训练命令中的 --registry-name 即可
```

> 注意：不同动作可能需要调整 BeyondMimic 的奖励权重——高动态动作（如回旋踢）需要更宽松的惩罚，低动态动作（如行走）可以更严格。

#### 真机部署安全提醒 ⭐

仓库明确声明："此仓库仅供教学目的。我们不对因尝试复现结果而可能发生的财产损坏或人身伤害承担任何责任。" 真机部署高动态动作是高风险操作。部署前务必完成 sim2sim 验证（见 6.2 节）和挂起测试（见 6.4 节检查清单）。

### 5.4 常见问题: 模仿不像/摔倒/抖动 ⭐⭐

| 症状 | 原因分析 | 解决方案 |
|------|---------|---------|
| 动作完全不像参考 | Retarget 数据有误; tracking reward 权重太低 | 先在 MuJoCo viewer 中回放 retarget 数据, 确认正确; 增大 tracking 权重 |
| 部分关节跟不上 | 参考运动超出关节限制 | 检查 retarget 后的关节角是否在 URDF/MJCF 限制内 |
| 追踪前半段好, 后半段摔倒 | 缺少 RSI (Reference State Initialization) | 训练时从参考运动的随机时刻初始化, 不只从 t=0 |
| AMP 风格不自然 | Discriminator 容量不够; 数据太少 | 增大 disc_hidden_units; 增加参考运动数据量 |
| AMP 的 discriminator loss 震荡 | gradient penalty 不够 | 增大 disc_grad_penalty (10→20) |
| Motion tracking + 任务奖励冲突 | tracking 和 task reward 方向矛盾 | 降低 tracking weight, 让策略有更多任务适应空间 |
| 模仿时关节高频抖动 | action_rate 惩罚不够; 参考动作本身抖动 | 增大 action_rate 权重; 对参考动作做时域低通滤波 |

---

## Part 6: 部署

### 6.1 ONNX 导出 ⭐⭐

```python
# 从 PyTorch checkpoint 导出 ONNX

import torch
import onnx

# 1. 加载训练好的策略
checkpoint = torch.load("model_5000.pt", map_location="cpu")
actor = checkpoint["actor"]  # 或根据框架获取 actor 网络

# 2. 准备 dummy input
obs_dim = 45  # 与训练时一致
dummy_obs = torch.randn(1, obs_dim)

# 3. 导出
torch.onnx.export(
    actor,
    dummy_obs,
    "policy.onnx",
    input_names=["obs"],
    output_names=["action"],
    opset_version=11,
    dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
)

# 4. 验证
model = onnx.load("policy.onnx")
onnx.checker.check_model(model)
print("ONNX export OK")
print(f"Input shape: {[d.dim_value for d in model.graph.input[0].type.tensor_type.shape.dim]}")
print(f"Output shape: {[d.dim_value for d in model.graph.output[0].type.tensor_type.shape.dim]}")
```

**mjlab 自动导出:**

```bash
# mjlab 训练完成后自动导出 ONNX 到 W&B artifacts
# 或手动:
uv run export <task-name> --wandb-run-path <path>
```

**rsl_rl 导出 (legged_gym / IsaacLab):**

```python
# rsl_rl 的 OnPolicyRunner 提供了导出接口
from rsl_rl.runners import OnPolicyRunner

runner = OnPolicyRunner(env, train_cfg, log_dir="logs", device="cuda")
runner.load("model_5000.pt")

# 导出
runner.export_policy(export_path="policy.onnx")
# 同时导出观测归一化参数:
runner.export_obs_normalizer(export_path="obs_normalizer.json")
```

### 6.2 MuJoCo CPU 评估 (Sim-to-Sim 验证) ⭐⭐

在部署到真机前, 先在 MuJoCo CPU (单线程, 高精度) 上验证策略:

```python
import mujoco
import onnxruntime as ort
import numpy as np

# 1. 加载模型和策略
m = mujoco.MjModel.from_xml_path("unitree_go2/scene.xml")
d = mujoco.MjData(m)
session = ort.InferenceSession("policy.onnx")

# 2. 定义默认关节位置 (必须与训练一致)
default_joint_pos = np.array([...])  # 12 维, 从训练配置中获取
action_scale = 0.25  # 与训练一致

# 3. 构造观测函数 (必须与训练一致!)
def build_observation(m, d, last_action, cmd):
    """构造与训练时完全一致的观测向量"""
    # 关节角偏差
    joint_pos_rel = d.qpos[7:] - default_joint_pos  # 前 7 个是 root
    # 关节速度
    joint_vel = d.qvel[6:]  # 前 6 个是 root
    # 基座角速度 (body frame)
    base_ang_vel = d.sensordata[0:3]  # 假设 gyro sensor
    # 重力方向投影
    quat = d.qpos[3:7]
    gravity_proj = rotate_vector_by_quat(np.array([0, 0, -1]), quat)
    # 拼接
    obs = np.concatenate([
        joint_pos_rel, joint_vel, base_ang_vel,
        gravity_proj, cmd, last_action
    ]).astype(np.float32)
    return obs

# 4. 仿真循环
cmd = np.array([0.5, 0.0, 0.0])  # 前进 0.5 m/s
last_action = np.zeros(12, dtype=np.float32)

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        obs = build_observation(m, d, last_action, cmd)
        action = session.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        last_action = action.copy()

        # PD 控制
        d.ctrl[:] = default_joint_pos + action * action_scale
        mujoco.mj_step(m, d)
        viewer.sync()
```

**Sim2Sim 检查项:**

| 检查项 | 方法 | 通过标准 |
|--------|------|---------|
| 行走速度一致性 | 对比训练仿真器和 CPU 的平均速度 | 差异 < 10% |
| 稳定性 | 在 CPU 上连续运行 60 秒 | 不摔倒 |
| 抖动对比 | 对比两边的关节加速度 RMS | 量级一致 |
| 外力恢复 | 在 CPU 上施加侧向推力 | 能恢复平衡 |

### 6.3 ROS2 部署接口 ⭐⭐⭐

```python
#!/usr/bin/env python3
"""
ROS2 RL 策略部署节点示例
订阅: /joint_states, /imu, /cmd_vel
发布: /joint_commands
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
import onnxruntime as ort
import numpy as np

class RLPolicyNode(Node):
    def __init__(self):
        super().__init__("rl_policy")

        # 加载 ONNX 策略
        self.session = ort.InferenceSession("policy.onnx")
        self.last_action = np.zeros(12, dtype=np.float32)

        # 存储最新传感器数据
        self.joint_pos = np.zeros(12, dtype=np.float32)
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.gravity_proj = np.array([0, 0, -1], dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)

        # 订阅
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        # 发布
        self.pub = self.create_publisher(JointState, "/joint_commands", 10)

        # 策略执行定时器 (50 Hz, 与训练一致!)
        self.create_timer(0.02, self.policy_step)

    def policy_step(self):
        # 构造观测 (顺序和 scale 必须与训练一致)
        obs = np.concatenate([
            self.joint_pos - DEFAULT_JOINT_POS,  # joint_pos_rel
            self.joint_vel,
            self.base_ang_vel,
            self.gravity_proj,
            self.cmd,
            self.last_action,
        ]).astype(np.float32)

        # 推理
        action = self.session.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        self.last_action = action.copy()

        # 发送关节命令
        msg = JointState()
        msg.position = (DEFAULT_JOINT_POS + action * ACTION_SCALE).tolist()
        self.pub.publish(msg)

    def joint_cb(self, msg):
        self.joint_pos = np.array(msg.position, dtype=np.float32)
        self.joint_vel = np.array(msg.velocity, dtype=np.float32)

    def imu_cb(self, msg):
        self.base_ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ], dtype=np.float32)
        # 从 IMU 四元数计算重力投影
        q = msg.orientation
        self.gravity_proj = quat_rotate_inverse(
            [q.w, q.x, q.y, q.z], [0, 0, -1]
        ).astype(np.float32)

    def cmd_cb(self, msg):
        self.cmd = np.array([
            msg.linear.x, msg.linear.y, msg.angular.z
        ], dtype=np.float32)
```

### 6.4 Sim2Real 检查清单 ⭐⭐⭐

```
□ 动作空间匹配
  □ 训练和真机使用相同的动作 scale
  □ 关节顺序一致 (训练中的 joint index = 真机的 joint index)
  □ default_joint_pos 一致 (不同框架可能有不同的零位定义!)

□ 观测一致
  □ IMU 坐标系与训练一致 (常见错误: x/y/z 轴定义不同)
  □ 如果训练时用了观测归一化 (running mean/std), 部署时必须加载相同参数
  □ 关节编码器零位校准
  □ 传感器噪声 level 在 DR 覆盖范围内

□ 延迟补偿
  □ 测量真机通讯延迟 (电机驱动器 → 控制板 → 策略推理 → 返回)
  □ 确认训练时的延迟 DR 覆盖了真机延迟
  □ 策略执行频率与训练时的 decimation 一致 (如 50 Hz)

□ PD 增益校准
  □ 真机 PD 增益 kp/kd 与训练完全一致
  □ 在真机上单独测试 PD 响应: 设定目标角度, 检查跟踪精度
  □ 如果真机电机有内置 PD, 确认其行为与训练中的 PD 模型一致

□ Safety Limits
  □ 力矩 clip: 不超过电机额定力矩
  □ 关节限位: 软件保护 (clip) + 硬件保护 (机械限位)
  □ 紧急停止按钮已测试
  □ 速度限制: 初始部署时限制最大速度命令

□ 渐进测试流程
  □ 1. 挂起/支架上运行, 观察关节运动
  □ 2. 放到地上, 站立稳定性 (零速度命令)
  □ 3. 原地踏步 (微小速度命令)
  □ 4. 低速前进 (0.2 m/s)
  □ 5. 中速前进 (0.5 m/s)
  □ 6. 目标速度 (1.0+ m/s)
  □ 7. 转弯测试
  □ 8. 外力扰动恢复
```

---

## Part 7: 进阶工程技巧

### 7.1 多 GPU 训练 ⭐⭐⭐

**IsaacLab 多 GPU:**

```bash
# 使用 torchrun (PyTorch 分布式)
torchrun --nproc_per_node=2 \
    source/standalone/workflows/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 4096 \
    --distributed
# 每个 GPU 运行 4096/2 = 2048 个环境
```

**mjlab 多 GPU:**

```bash
# mjlab 内置多 GPU 支持
uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --gpu-ids "[0, 1]" \
    --env.scene.num-envs 4096
# 环境自动分配到多个 GPU
```

**多 GPU 注意事项:**
- 多 GPU 不一定线性加速 — 通信开销可能抵消收益
- 建议先在单 GPU 上调好超参数, 再用多 GPU 放大训练
- 如果只是为了更多环境, 增大 num_envs (单 GPU 上) 通常更高效

### 7.2 Curriculum Learning 配置 ⭐⭐⭐

**地形课程 (IsaacLab):**

```python
from omni.isaac.lab.terrains import TerrainGeneratorCfg, SubTerrainCfg

terrain = TerrainGeneratorCfg(
    sub_terrains={
        "flat": SubTerrainCfg(proportion=0.2, type="plane"),
        "rough": SubTerrainCfg(proportion=0.3, type="rough",
                               params={"height_range": (0.02, 0.08)}),
        "slopes": SubTerrainCfg(proportion=0.2, type="slope",
                                params={"slope_range": (0.0, 0.3)}),
        "stairs": SubTerrainCfg(proportion=0.3, type="stairs",
                                params={"step_height": (0.05, 0.15)}),
    },
    curriculum=True,  # 启用自适应难度
)
```

**mjlab 地形课程:**

```python
from mjlab.terrains import TerrainImporterCfg

terrain = TerrainImporterCfg(
    terrain_type="heightfield",
    # heightfield 分辨率受 GPU 内存限制
)

# 课程配置
curriculum = {
    "terrain_level": CurriculumTermCfg(
        func=mdp.terrain_level_curriculum,
        params={
            "success_threshold": 0.8,  # 环境成功率 > 80% → 升级地形
        },
    ),
}
```

**课程学习的核心逻辑 (legged_gym 标准):**
- 每个环境有独立的 `terrain_level`
- 当某个 level 的环境完成率 > 80% → 该环境升级
- 部分环境始终保持简单地形, 防止遗忘
- 地形从平地 → 小坡 → 台阶 → 随机地形渐进

### 7.3 自定义机器人接入 (URDF/MJCF → 训练环境) ⭐⭐⭐

#### 通用步骤 ⭐⭐

```
Step 1: 获取模型
  - 优先: mujoco_menagerie (~3.4k stars) 已有校准模型
  - 备选: 从 URDF 转换

Step 2: URDF → MJCF 转换 (如果需要)
  python -c "import mujoco; mujoco.MjModel.from_xml_path('robot.urdf')"
  # MuJoCo 自动转换

Step 3: 模型验证
  python -m mujoco.viewer robot.xml
  # 检查: 关节自由度, 碰撞几何, 惯量参数

Step 4: 配置执行器 (关键!)
  - 确认 PD 增益与真机一致
  - 确认力矩限制与真机一致
  - 确认关节限位与真机一致

Step 5: 修改环境配置
  - 更换模型路径
  - 修改关节名映射
  - 调整观测/动作维度
  - 调整奖励权重 (不同机器人最优权重不同!)

Step 6: 训练 + 验证
```

#### mjlab ANYmal C 接入示例 ⭐⭐⭐

> 来自 `mujocolab/anymal_c_velocity` (~57 stars)。详细代码见 `S3B_mjlab深度实战.md` S3-B.4 节。

```python
# 关键差异: 关节命名和 PD 增益不同
robot = EntityCfg(
    mjcf_path="mujoco_menagerie/anybotics_anymal_c/anymal_c.xml",
    actuators={
        "legs": ActuatorCfg(
            joint_names=["LF_HAA", "LF_HFE", "LF_KFE",  # 左前
                         "RF_HAA", "RF_HFE", "RF_KFE",  # 右前
                         "LH_HAA", "LH_HFE", "LH_KFE",  # 左后
                         "RH_HAA", "RH_HFE", "RH_KFE"], # 右后
            effort_limit=40.0,    # ANYmal 力矩更大
            stiffness=80.0,       # PD kp 更高
            damping=2.0,
        ),
    },
)
# 其余 Manager 配置 90% 可从 Go2 直接复制
```

**关键教训**: 不同机器人之间, 环境逻辑 (观测/奖励/终止/事件) 大部分共享, 只需修改 EntityCfg 和少量超参数。

### 7.4 IsaacLab 到 mjlab 环境迁移 ⭐⭐

> 详细对照表见 `S3B_mjlab深度实战.md` S3-B.7 节。以下只列关键差异。

| 操作 | IsaacLab | mjlab | 迁移难度 |
|------|----------|-------|---------|
| 安装 | Omniverse + Isaac Sim + pip | `pip install mjlab` | **mjlab 极简** |
| 环境定义 | `class MyEnv(ManagerBasedRLEnv)` | 相同 | **零迁移** |
| 观测/奖励 | `ObservationTermCfg(func=...)` | 相同 | **零迁移** |
| 机器人定义 | `ArticulationCfg(usd_path=...)` | `EntityCfg(mjcf_path=...)` | **低 — 改模型路径** |
| 训练命令 | `./isaaclab.sh -p ... --task ...` | `uv run train ...` | **低** |
| DR 惯量 | 分别随机化 mass/inertia | Rucker 联合随机化 | **中 — 新 API** |
| 模型格式 | USD / URDF | MJCF (Menagerie) | **高 — 需换模型** |
| 接触物理 | PhysX GPU rigid body | MuJoCo Warp soft contact | **高 — 行为不同** |

**迁移建议:**
1. Manager API 代码基本可以 1:1 复制
2. 最大工作量在模型转换 (USD → MJCF) 和接触参数调整
3. 建议做 sim2sim 对比: 同一策略分别在两个仿真器中评估

### 7.4.1 mjlab 自定义机器人接入 ⭐⭐⭐

> 本节补充 mjlab 框架下接入一个非内置机器人的完整流程。从获取模型到验证训练，覆盖常见的坑。

#### Step 1: 获取 MJCF 模型 ⭐⭐

**优先方案: mujoco_menagerie**

[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) (~3.4k stars) 提供了经过校准的高质量 MJCF 模型。mjlab 的内置机器人（Go2、G1、H1 等）均来自 menagerie。

```bash
# menagerie 已作为 mjlab 的 submodule 包含
# 模型路径: mjlab/third_party/mujoco_menagerie/<vendor>_<robot>/

# 查看可用模型
ls third_party/mujoco_menagerie/
# 输出: unitree_go2/ unitree_g1/ unitree_h1/ anybotics_anymal_c/
#        franka_emika_panda/ ...

# 验证模型（本地 viewer）
python -m mujoco.viewer third_party/mujoco_menagerie/unitree_go2/scene.xml
```

**备选方案: 从 URDF 转换**

如果你的机器人不在 menagerie 中，需要从 URDF 转换：

```python
# MuJoCo 支持直接加载 URDF（自动转换为内部 MJCF）
import mujoco
m = mujoco.MjModel.from_xml_path("my_robot.urdf")

# 或使用 compile 脚本生成 MJCF
# python -c "
# import mujoco
# m = mujoco.MjModel.from_xml_path('my_robot.urdf')
# mujoco.mj_saveLastXML('my_robot.xml', m)
# "
```

> URDF → MJCF 转换的常见问题：mesh 路径需要修正；惯量参数可能需要手动校准；接触几何体可能需要简化。

#### Step 2: 配置 EntityCfg ⭐⭐

```python
from mjlab.entity import EntityCfg
from mjlab.actuator import ActuatorCfg

# 关键: 从真机 datasheet 获取执行器参数
my_robot = EntityCfg(
    mjcf_path="path/to/my_robot/scene.xml",
    actuators={
        "legs": ActuatorCfg(
            joint_names=["joint_1", "joint_2", "..."],  # 必须与 MJCF 中的 joint name 一致
            effort_limit=33.5,      # Nm — 从电机 datasheet 获取
            velocity_limit=21.0,    # rad/s — 从电机 datasheet 获取
            stiffness=40.0,         # PD kp — 需要在真机上标定
            damping=1.0,            # PD kd — 需要在真机上标定
        ),
    },
)
```

**执行器参数标定（关键！）:**

| 参数 | 来源 | 注意事项 |
|------|------|---------|
| `effort_limit` | 电机 datasheet 峰值力矩 | 考虑减速比: $\tau_{output} = \tau_{motor} \times \text{ratio}$ |
| `velocity_limit` | 电机 datasheet 最大转速 / 减速比 | 留 10-20% 余量 |
| `stiffness` (kp) | 真机 PD 标定实验 | 给定阶跃输入，调 kp 直到无超调/无振荡 |
| `damping` (kd) | 真机 PD 标定实验 | 通常 kd = 0.01~0.1 * kp |

#### Step 3: 测试模型（训练前验证） ⭐⭐

```bash
# 使用 demo 工具验证模型加载和基本物理行为
uvx --from mjlab demo  # 默认模型

# 使用 zero/random agent 验证自定义环境
uv run play MyCustomTask --agent zero    # 应该倒下
uv run play MyCustomTask --agent random  # 应该乱动但不爆炸
```

**验证清单:**

| 检查项 | 方法 | 通过标准 |
|--------|------|---------|
| 关节自由度数量 | 打印 `entity.num_joints` | 与真机一致 |
| 关节限位 | 在 viewer 中拖动关节 | 范围与 datasheet 一致 |
| 质量/惯量 | 打印 `mjModel.body_mass` | 与 CAD/datasheet 一致 |
| 碰撞几何 | viewer 中开启碰撞可视化 | 无穿模，足端几何合理 |
| PD 响应 | 施加阶跃位置命令 | 快速收敛，无振荡 |

### 7.5 性能优化: 提高训练速度的技巧 ⭐⭐⭐

| 技巧 | 加速幅度 | 实现难度 | 说明 |
|------|---------|---------|------|
| 增大 num_envs | 1.5-3x | 低 | 到 GPU 内存上限为止; 4090 通常 4096-8192 |
| 关闭渲染 | 1.2-1.5x | 低 | 训练时 `--headless`; 评估时再开 |
| 减小 network 大小 | 1.1-1.3x | 低 | [256, 128] 通常够用, 不需要 [512, 256, 128] |
| 减小 num_epochs | 1.3-2x | 中 | 从 5 → 3, 但可能降低样本效率 |
| CUDA Graph 捕获 | 1.3-2x | 自动 | mjlab 默认启用; IsaacLab 部分启用 |
| 混合精度 (FP16) | 1.2-1.5x | 中 | rsl_rl 默认 FP32; 可手动开 AMP |
| 多 GPU 并行 | 1.5-2x | 中 | 2 GPU 不一定 2x — 通信开销 |
| 减小 sim_dt | 负加速 | - | 更小步长 → 更多仿真步 → 更慢, 但更精确 |

**GPU 内存估算:**

```
粗略公式:
  VRAM ≈ num_envs × (model_size + obs_size + policy_params) + 框架开销

示例 (Go2, 4096 envs):
  模型: ~2MB/env × 4096 ≈ 8 GB
  观测+梯度: ~0.5MB/env × 4096 ≈ 2 GB
  框架开销: ~2-4 GB
  总计: ~12-14 GB → RTX 3090 (24GB) OK, RTX 3060 (12GB) 紧张

如果 OOM:
  1. 先减 num_envs (4096 → 2048 → 1024)
  2. 减小 network 大小
  3. 检查是否有内存泄漏 (tensorboard 日志占磁盘)
```

### 7.6 mjlab 调试与开发技巧 ⭐⭐

> 本节汇总 mjlab 日常开发中的高频操作和调试方法。

#### 7.6.1 Viser Web Viewer 使用 ⭐⭐

mjlab 使用 [Viser](https://viser.studio/) 作为 web 可视化后端，无需本地显示器即可在浏览器中查看仿真。

```bash
# 评估时自动启动 Viser viewer（默认端口 8080）
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8
# 浏览器打开 http://localhost:8080

# 零安装体验（自动打开浏览器）
uvx --from mjlab --refresh demo
```

**Viser 操作速查:**

| 操作 | 方法 |
|------|------|
| 旋转视角 | 鼠标左键拖动 |
| 平移视角 | 鼠标右键拖动 |
| 缩放 | 鼠标滚轮 |
| 速度命令 | GUI 侧边栏摇杆 / 滑块 |
| telemetry 覆盖 | 侧边栏展开观测/奖励面板 |

#### 7.6.2 wandb 集成 ⭐⭐

mjlab 原生集成 Weights & Biases，训练时自动记录所有指标。

```bash
# 首次使用需要登录
wandb login

# 训练时自动上传到 wandb（默认行为）
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
    --env.scene.num-envs 4096

# 关闭 wandb 记录（离线调试时）
WANDB_MODE=offline uv run train Mjlab-Velocity-Flat-Unitree-G1 \
    --env.scene.num-envs 64 \
    --agent.max-iterations 50
```

**wandb 常用功能:**
- **Run 对比**: 在 wandb dashboard 中选择多个 run，对比 reward 曲线
- **Artifacts**: ONNX 模型、motion NPZ 文件自动存储为 artifacts
- **Registry**: 参考动作数据通过 `wandb artifact put` 上传到 Registry

#### 7.6.3 Checkpoint 管理 ⭐⭐

```bash
# 训练自动保存 checkpoint 到 logs/ 和 wandb artifacts
# 日志路径: logs/rsl_rl/<task_name>/<date_time>/model_<iteration>.pt

# 从 checkpoint 恢复训练
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
    --agent.resume True \
    --env.scene.num-envs 4096

# 从 wandb run 加载 checkpoint 评估
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id

# 手动导出 ONNX
uv run export Mjlab-Velocity-Flat-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id
```

#### 7.6.4 `uv run play` 选项与 Sanity Check ⭐⭐

```bash
# 标准评估
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id \
    --num-envs 8

# 零动作 agent（验证环境 MDP 是否正确——机器人应该倒下）
uv run play Mjlab-Velocity-Flat-Unitree-G1 --agent zero

# 随机动作 agent（验证动作空间和物理是否合理）
uv run play Mjlab-Velocity-Flat-Unitree-G1 --agent random

# Motion tracking 评估（需要指定 motion registry）
uv run play Mjlab-Tracking-Flat-Unitree-G1 \
    --wandb-run-path your-org/mjlab/run-id \
    --registry-name your-org/motions/motion-name
```

> `--agent zero` 和 `--agent random` 是调试环境配置的关键工具——在编写自定义环境后，先用这两个 agent 验证基本物理行为是否合理，再开始训练。

#### 7.6.5 性能 Profiling ⭐⭐⭐

**识别训练瓶颈的方法:**

```bash
# 观察 wandb 面板中的 timing 指标:
#   - sim_step_time: 物理步进耗时
#   - policy_time:   策略推理耗时
#   - total_step_time: 每个训练 step 的总耗时

# 如果 sim_step_time >> policy_time → 物理是瓶颈
#   → 减少 num_envs 或降低接触点数量
# 如果 policy_time >> sim_step_time → 策略推理是瓶颈
#   → 减小 network 大小 (如 [512,256,128] → [256,128])
```

#### 7.6.6 常见开发工作流 ⭐⭐

```
1. 编辑配置 (修改奖励权重/观测/DR 参数)
     ↓
2. 短期训练验证 (64 envs, 100 iterations, ~2 分钟)
   uv run train MyTask --env.scene.num-envs 64 --agent.max-iterations 100
     ↓
3. 评估行为 (play + Viser)
   uv run play MyTask --wandb-run-path ...
     ↓
4. 检查奖励曲线 (wandb dashboard)
     ↓
5. 调整参数 → 回到 step 1
     ↓ 确认收敛方向正确
6. 全规模训练 (4096 envs, 5000+ iterations)
   MUJOCO_GL=egl uv run train MyTask --env.scene.num-envs 4096 --agent.max-iterations 5000
```

**调试常见问题速查:**

| 症状 | 排查方向 | 解决方案 |
|------|---------|---------|
| `uv run train` 报 CUDA 错误 | GPU 驱动版本不对 | 确保 NVIDIA Driver >= 550; `nvidia-smi` 检查 |
| Viser viewer 打不开 | 端口被占用或防火墙 | 换端口或检查 `8080` 是否被其他进程占用 |
| wandb 上传失败 | 网络问题 | `WANDB_MODE=offline` 离线训练; 稍后 `wandb sync` |
| 训练速度异常慢 | GPU 利用率低 | `nvidia-smi` 检查; 增大 `num-envs`; 确认 `MUJOCO_GL=egl` |
| checkpoint 文件很大 | 保存了 optimizer state | 正常行为; ONNX 导出文件会小很多 |

---

## 附录: 速查表

### A. 所有命令行速查 ⭐

```bash
# ============================================================
# IsaacLab
# ============================================================

# 训练
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 4096 \
    --max_iterations 5000 \
    --headless

# 评估
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 16 \
    --checkpoint <path_to_model.pt>

# 多 GPU
torchrun --nproc_per_node=2 \
    source/standalone/workflows/rsl_rl/train.py \
    --task ... --num_envs 4096 --distributed

# ============================================================
# mjlab
# ============================================================

# 零安装体验
uvx --from mjlab demo

# 训练
MUJOCO_GL=egl uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --env.scene.num-envs 4096 \
    --agent.max-iterations 5000

# 评估
uv run play Mjlab-Velocity-Flat-Unitree-Go2-Play \
    --wandb-run-path <org/project/run-id> \
    --num-envs 8

# 多 GPU
uv run train Mjlab-Velocity-Flat-Unitree-Go2 \
    --gpu-ids "[0, 1]" \
    --env.scene.num-envs 4096

# ONNX 导出
uv run export <task-name> --wandb-run-path <path>

# ============================================================
# legged_gym (legacy)
# ============================================================

# 训练
python legged_gym/scripts/train.py --task=anymal_c_flat
python legged_gym/scripts/train.py --task=go2_flat --num_envs 4096

# 评估
python legged_gym/scripts/play.py --task=anymal_c_flat

# ============================================================
# MuJoCo Playground
# ============================================================

# 训练
python -m mujoco_playground.train \
    --task Go1JoystickFlatTerrain \
    --num_timesteps 200_000_000

# ============================================================
# 通用工具
# ============================================================

# TensorBoard
tensorboard --logdir logs/

# MuJoCo viewer (验证模型)
python -m mujoco.viewer robot.xml
```

### B. 奖励函数参数速查 ⭐

**四足 (Go2) baseline:**

| 奖励项 | 函数 | 权重 | 备注 |
|--------|------|------|------|
| track_lin_vel_xy | `mdp.track_lin_vel_xy_exp` | **1.5** | std=0.25 |
| track_ang_vel_z | `mdp.track_ang_vel_z_exp` | **0.75** | std=0.25 |
| lin_vel_z | `mdp.lin_vel_z_l2` | -2.0 | 抑制弹跳 |
| ang_vel_xy | `mdp.ang_vel_xy_l2` | -0.05 | 抑制摇晃 |
| flat_orientation | `mdp.flat_orientation_l2` | -0.5 | 保持水平 |
| joint_torques | `mdp.joint_torques_l2` | -1e-5 | 能耗正则 |
| action_rate | `mdp.action_rate_l2` | -0.01 | 抑制抖动 |
| joint_acc | `mdp.joint_acc_l2` | -2.5e-7 | 平滑加速度 |
| dof_pos_limits | `mdp.joint_pos_limits` | -10.0 | 安全 |
| feet_air_time | `mdp.feet_air_time` | 0.125 | 防止拖脚 |

**人形 (H1) 额外项:**

| 奖励项 | 函数 | 权重 | 备注 |
|--------|------|------|------|
| upright | `mdp.flat_orientation_l2` | -2.0 | 躯干直立 (关键!) |
| com_height | 自定义 | -1.0 | 维持站立高度 |
| arm_regularization | 自定义 | -0.1 | 手臂不乱甩 |
| step_frequency | 自定义 | 0.5 | 步频追踪 (可选) |

### C. Domain Randomization 参数速查 ⭐

| 参数 | 范围 (标准) | 范围 (保守) | 范围 (激进) |
|------|-----------|-----------|-----------|
| 质量 scale | [0.8, 1.2] | [0.9, 1.1] | [0.7, 1.3] |
| 摩擦系数 | [0.5, 1.25] | [0.7, 1.1] | [0.3, 1.5] |
| CoM 偏移 (cm) | [-2, 2] | [-1, 1] | [-3, 3] |
| 电机强度 scale | [0.85, 1.15] | [0.9, 1.1] | [0.8, 1.2] |
| PD kp scale | [0.9, 1.1] | [0.95, 1.05] | [0.85, 1.15] |
| 控制延迟 (ms) | [0, 20] | [0, 10] | [0, 30] |
| IMU 噪声 (rad/s) | +/-0.2 | +/-0.1 | +/-0.4 |
| 编码器噪声 (rad) | +/-0.01 | +/-0.005 | +/-0.02 |
| 外力推动 (m/s) | 0-1.0 | 0-0.5 | 0-1.5 |
| 推动间隔 (s) | 5-15 | 10-20 | 3-10 |

### D. 训练问题排查速查 ⭐

| 症状 | 可能原因 | 第一步 | 解决方案 |
|------|---------|-------|---------|
| reward 不上升 | 奖励设计 bug | 打印各项奖励值, 找量级问题 | 调整权重比例 |
| reward 上升后崩塌 | KL 发散 | 看 tensorboard KL 曲线 | 降低 lr 或减小 clip_range |
| episode_length 始终短 | 策略一直摔 | 检查终止条件是否过严 | 放宽 limit_angle; 增大 orientation 惩罚 |
| NaN in loss | 物理爆炸 | 减少 num_envs, 查看异常环境 | 加 obs clipping; 降 action_scale |
| OOM | 显存不足 | `nvidia-smi` 看内存 | 减 num_envs; 减 network 大小 |
| 策略抖动 | action_rate 不够 | 看 action_rate reward 曲线 | 增大 action_rate 权重 |
| 只前进不转弯 | cmd 分布不均 | 检查 command generator 配置 | 确保 vy, omega_z 在 cmd 中 |
| sim2real 失败 | 观测/动作不一致 | 逐项核对训练 vs 部署参数 | 见 6.4 检查清单 |
| 训练速度异常慢 | GPU 利用率低 | `nvidia-smi` 看 GPU 使用率 | 增大 num_envs; 关闭渲染; 检查数据管线 |
| 多 seed 收敛差异大 | 超参不鲁棒 | 跑 5 个 seed 看分布 | 调整 lr/entropy_coeff; 增大 batch |

### E. 关键论文与事实速查 ⭐

| 事实 | 正确信息 |
|------|---------|
| MuJoCo 被收购 | 2021 年 10 月被 DeepMind 收购 |
| MuJoCo 开源 | 2022 年 5 月 Apache-2.0 |
| Hwangbo 2019 仿真器 | ETH 自研仿真环境, **非 MuJoCo** |
| RMA 年份/会议 | Kumar et al., 2021, RSS |
| Walk These Ways | Margolis & Agrawal, 2023, CoRL |
| rsl_rl 学习率调整 | Adaptive KL-based (非固定 schedule) |
| rsl_rl 网络结构 | Actor 和 Critic 使用**独立** MLP |

| 项目 | Stars (近似) |
|------|-------------|
| IsaacLab | ~7.1k |
| legged_gym | ~2.9k |
| mujoco_menagerie | ~3.4k |
| mjlab | ~2.2k |
| MuJoCo Playground | ~1.8k |
| humanoid-gym | ~1.2k |
| rsl_rl | ~1k |
| PHC | ~600 |
