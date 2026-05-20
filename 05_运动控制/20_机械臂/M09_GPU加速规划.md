> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M09 GPU加速运动规划——从瓶颈分析到 cuRobo/VAMP 实战

> **性质**: ⚪ 部分共享 | **时长**: 1.5 周 (12-18 学时)
> **前置**: M07(OMPL采样规划), M04(碰撞检测); CUDA 编程基础（如有相关课程经验）
> **下游**: M10(时间参数化) → M14(MoveIt2集成)

---

## M09.0 前置自测

开始本章之前，请独立回答以下 5 题。若答不出 3 题以上，建议先回顾 M07 和 M04。

| # | 问题 | 期望答案要点 |
|---|------|-------------|
| 1 | M07 中 OMPL 规划的性能瓶颈是什么？碰撞检测占总规划时间的百分比大约是多少？ | 碰撞检测是瓶颈，占总规划时间的 80-95%。规划算法本身（树操作、近邻搜索）只占 5-20% |
| 2 | GPU 并行计算的核心优势是什么？与 CPU 多线程相比，适合什么类型的任务？ | GPU 有数千个核心，适合**大规模数据并行**（SIMD/SIMT）：同一操作施加到大量数据上。不适合分支复杂、串行依赖强的任务 |
| 3 | 什么是 SIMD？AVX2 指令集的向量宽度是多少？ | SIMD = Single Instruction Multiple Data，一条指令同时处理多个数据。AVX2 的向量宽度 256 bit，可同时处理 8 个 float 或 4 个 double |
| 4 | 球体近似碰撞检测的原理是什么？为什么比 mesh 碰撞检测快？ | 用多个球体包裹机器人 link。两球碰撞检测只需比较球心距和半径之和：$\|c_1 - c_2\| < r_1 + r_2$。比 GJK/EPA mesh 检测快 100-1000 倍 |
| 5 | CUDA kernel 中的 thread、block、grid 分别是什么？如何映射到硬件？ | Thread 是最小执行单元；Block 是一组 thread（共享 shared memory）；Grid 是所有 block 的集合。Block 映射到 SM（Streaming Multiprocessor），thread 以 warp（32个）为单位执行 |

---

## 本章目标

学完本章后，你将能够：
1. 从定量分析角度理解为什么运动规划需要 GPU/SIMD 加速（瓶颈在碰撞检测）
2. 掌握 cuRobo 的完整架构：GPU-FK → GPU 碰撞检测 → GPU 轨迹优化 → 运动生成
3. 理解 VAMP 的 SIMD 加速策略：手写 AVX2/NEON intrinsics vs 编译器自动向量化
4. 能在 CPU SIMD (VAMP) 和 GPU CUDA (cuRobo) 之间做出硬件加速决策
5. 将 cuRobo 与 MoveIt2 集成（通过 Isaac ROS cuMotion 插件）
6. 分析 GPU 规划的局限性与适用场景

---

## M09.1 为什么规划需要 GPU ⭐

### 动机：2023 年前的性能基线已到天花板 ⭐

回顾 M07-M08，CPU 上的运动规划性能基线：

| 环节 | 工具 | 典型延迟 |
|------|------|---------|
| 采样规划 | OMPL RRT-Connect + FCL | 20-500 ms |
| 轨迹优化 | CHOMP/STOMP | 50-500 ms |
| 时间参数化 | TOPP-RA / Ruckig | 1-5 ms |
| **端到端总延迟** | — | **100 ms - 1 s** |

**这够用吗？**

- **静态环境、离线规划**: 够用。工厂产线预规划一次，反复执行。
- **动态环境**: 不够。人员走动 → 需要实时重规划（每 50-100ms 一次）。
- **人机协作**: 不够。ISO/TS 15066 要求机器人在 250ms 内做出避让反应。

### 如果不加速会怎样 ⭐

每次规划需要 200ms+，在 10Hz 控制频率下只能每个周期开始时规划一次，然后盲目执行整条路径。如果障碍物在执行中移动，机器人无法反应——这在人机协作场景中不可接受。

### 瓶颈定量分析 ⭐⭐

回顾 M07.5 的碰撞检测开销分析：

```
一次 OMPL 规划的时间分解 (Panda 7-DOF, 10 个障碍 box):

  总时间: 200 ms
  ├── 碰撞检测 (FCL): 180 ms (90%)
  │   ├── 宽相 (BroadPhase AABB): 20 ms
  │   └── 窄相 (GJK/EPA): 160 ms
  ├── 近邻搜索 (KD-tree): 10 ms (5%)
  └── 树操作 (节点插入/连接): 10 ms (5%)
```

**碰撞检测是绝对瓶颈**。加速规划的关键不是改进算法逻辑，而是加速碰撞检测。

### 两条加速路线 ⭐

| 路线 | 代表 | 硬件 | 加速原理 | 加速倍数 |
|------|------|------|---------|---------|
| **GPU 并行** | cuRobo (NVIDIA) | GPU (Turing+) | 数百个并行种子 × GPU 碰撞检测 | 10-60x |
| **CPU SIMD** | VAMP (Kavraki Lab) | 任意现代 CPU | 手写 AVX2/NEON intrinsics 向量化碰撞 | 100-1000x (单次碰撞检测) |

> **本质洞察**: cuRobo 和 VAMP 加速的核心都是**碰撞检测**——前者用 GPU 的数千核心并行检查数百个构型，后者用 CPU 的 SIMD 寄存器一次检查 8 个球对。它们不改变 RRT/优化的算法逻辑——只是让每次碰撞检查从 50 us 变成 0.1 us。这印证了 M07.5 的判断：**碰撞检测是性能瓶颈，加速碰撞检测等于加速整个规划管线**。

### 性能代际跃迁 (2023-2025) ⭐⭐

| 系统 | 年份 | 全栈延迟 | 硬件 |
|------|------|---------|------|
| OMPL + FCL + KDL | ~2018 | 500 ms - 1 s | CPU |
| MoveIt2 + OMPL + STOMP | ~2022 | 100-500 ms | CPU |
| **cuRobo** (NVIDIA) | 2023 | **30 ms** (GPU) / 100 ms (Jetson) | GPU |
| **VAMP** (Kavraki Lab) | 2024 | **35 us** (中位, 仅采样规划) | CPU (AVX2) |

> **反事实推理**: 如果不做加速会怎样？每次规划需要 200ms+，在 30Hz 控制循环中只能每 6 个周期规划一次，其余时间盲目执行。30ms 全栈运动生成意味着可以在**每个控制周期**重新规划——机器人变成"实时反应式"系统，可以跟踪移动目标、避让走动的人员。未来 2-3 年，GPU 加速规控从"差异化能力"变为"标配"。

### ⚠️ 常见陷阱

**思维陷阱: 认为 GPU 加速一切都快**

```
🧠 GPU 加速的不是"规划算法"——而是"碰撞检测"和"批量FK"。
   RRT 树的生长逻辑（串行的节点-父节点依赖）不适合 GPU。
   cuRobo 的策略不是"把 RRT 放到 GPU 上"，
   而是"用 GPU 并行跑数百个轨迹优化种子"——
   把串行的"一次优化"变成并行的"数百次优化取最优"。
   VAMP 也不是"把 RRT 放到 SIMD"，
   而是用 SIMD 加速 RRT 内部的碰撞检测热循环。
```

**概念误区: 混淆 SIMD 和多线程**

```
💡 SIMD (Single Instruction Multiple Data):
   一个核心内, 一条指令同时处理多个数据 (如 8 个 float)。
   多线程: 多个核心, 各自执行不同指令流。
   VAMP 用 SIMD (单核向量化), 不用多线程。
   cuRobo 用 GPU SIMT (数千核心各执行相同程序的不同数据)。
   两者都是"数据并行", 但实现层次不同。
```

### 练习

1. **[性能分析]** 用 `std::chrono` 分别计时 OMPL 规划中的碰撞检测和树操作。在 Panda + 10 个障碍 box 场景中，验证碰撞检测占比是否达到 80-90%。
2. **[思考]** 如果碰撞检测已经足够快（如 0.1 us/次），近邻搜索（KD-tree）会成为新的瓶颈吗？VAMP 是否也加速了近邻搜索？

---

## M09.2 cuRobo 框架深度精读 ⭐⭐⭐

### 动机：首个 GPU 全栈运动生成框架 ⭐⭐

Sundaralingam et al. (ICRA 2023, NVIDIA) 提出 cuRobo——**首个把完整机械臂运动生成全部迁移到 GPU** 的开源项目。它挑战了数十年的假设："运动规划是串行问题，不适合 GPU。"

**cuRobo 的反驳思路**: 不并行化算法逻辑，而是并行化**问题实例**——同时启动数百个优化种子（不同初始猜测），GPU 的并行能力刚好解"多起点优化"。这和 M08 中 TrajOpt 的多起点 SQP 思路一致，只是 cuRobo 把并行度从"CPU 上 4-8 个线程"提升到"GPU 上 512 个种子"。

### 架构总览 ⭐⭐

```
cuRobo 全栈运动生成架构:

  输入: 当前构型 q_start + 目标位姿 T_goal + 世界模型
       │
       ▼
  ┌─────────────────────────────────────────────────────┐
  │ Stage 1: GPU-IK (逆运动学)                           │
  │   多起点 L-BFGS, 并行 512 个种子                      │
  │   输出: q_goal 候选集                                 │
  │   (类似 M03.7 pick-ik 的 global mode, 但在 GPU 上)   │
  └───────────────────────┬─────────────────────────────┘
                          │
                          ▼
  ┌─────────────────────────────────────────────────────┐
  │ Stage 2: GPU Graph Search (几何规划)                  │
  │   预计算 PRM 路标图, GPU 并行搜索多条路径              │
  │   (仅在直接优化失败时使用, 类似 M07 的 PRM 但在 GPU)  │
  └───────────────────────┬─────────────────────────────┘
                          │
                          ▼
  ┌─────────────────────────────────────────────────────┐
  │ Stage 3: GPU Trajectory Optimization                 │
  │   并行 N 个种子 × L-BFGS 优化                        │
  │   代价: jerk + collision + joint_limits               │
  │   碰撞检测: GPU 球体-OBB kernel (下文详述)            │
  │   (类似 M08 的轨迹优化, 但并行度提升 100x)            │
  └───────────────────────┬─────────────────────────────┘
                          │
                          ▼
  ┌─────────────────────────────────────────────────────┐
  │ Stage 4: Post-processing                             │
  │   轨迹修剪 + 时间参数化 + 平滑                        │
  └───────────────────────┬─────────────────────────────┘
                          │
                          ▼
  输出: 时间参数化的平滑轨迹 [(t₀,q₀), (t₁,q₁), ...]
```

### CUDA C++ + PyTorch 混合架构 ⭐⭐⭐

cuRobo 的工程架构是"现代 ML+机器人混合栈"的典型形态：

```
┌──────────────────────────────────────────────────┐
│  Python 用户 API (薄封装)                         │
│  from curobo.wrap.reacher import MotionGen       │
└──────────────────┬───────────────────────────────┘
                   │ PyTorch 张量 (GPU 内存)
                   ▼
┌──────────────────────────────────────────────────┐
│  PyTorch 中间层 (数据容器 + CUDA Graph 管理)      │
│  torch.Tensor 作为所有数据的统一表示               │
└──────────────────┬───────────────────────────────┘
                   │ pybind11 / Warp JIT
                   ▼
┌──────────────────────────────────────────────────┐
│  CUDA C++ Kernels (核心性能代码)                   │
│  src/curobo/curobolib/cpp/                       │
│  ├── kinematics_fused_kernel.cu   (正运动学)      │
│  ├── self_collision_kernel.cu     (自碰撞)        │
│  ├── sphere_obb_kernel.cu         (球-OBB碰撞)   │
│  └── pose_distance_kernel.cu     (位姿距离)       │
└──────────────────────────────────────────────────┘
```

> **跨领域类比**: cuRobo 的 C++/Python 分层架构类似于 PyTorch 自身——性能关键路径用 C++/CUDA 实现，用户面对的 API 是 Python 薄封装。这是"让用户写 Python 脚本，让底层跑 CUDA kernel"的工程范式。Pinocchio 的 eigenpy 绑定（回顾 M01）采用类似思路，但 cuRobo 更进一步——用 PyTorch 张量作为统一数据容器，天然支持 GPU 内存管理和自动微分。

### Warp Kernel 代码生成 ⭐⭐⭐

cuRobo 的一些 kernel 是"半编译期生成"——运行时根据机器人 DOF 数量决定 kernel 形态，然后用 NVIDIA Warp 框架把 Python DSL 转为 CUDA 代码并即时编译(JIT)。

```python
# Warp JIT 编译示意 (教学简化)
@wp.kernel
def fk_kernel(
    joint_angles: wp.array(dtype=float),
    link_transforms: wp.array(dtype=wp.mat44),
    num_joints: int
):
    tid = wp.tid()  # thread ID = batch index
    T = wp.identity(n=4, dtype=float)
    for j in range(num_joints):  # 循环在编译时展开!
        q = joint_angles[tid * num_joints + j]
        T = T @ dh_transform(q, params[j])
        link_transforms[tid * num_joints + j] = T
```

**JIT 编译的代价与收益**:
- **代价**: 首次编译 ~30 秒（`motion_gen.warmup()`）
- **收益**: 针对具体 DOF 数优化的 kernel（循环展开、寄存器分配）
- **缓存**: `CUROBO_USE_LRU_CACHE=1` 缓存已编译 kernel，后续启动秒级

这种 JIT 思想是 Pinocchio 的 `CppADCodeGen`（编译期代码生成）在 GPU 侧的对应物。

### CUDA Graph 优化 ⭐⭐⭐

cuRobo 反复求解同一类优化问题（相同 kernel 序列、不同输入数据）。CUDA Graph 把一系列 kernel 启动打包为一个"图"整体提交：

```
不用 CUDA Graph:
  CPU → GPU: launch kernel_A
  CPU → GPU: launch kernel_B    每次 launch 有 5-20 us 同步开销
  CPU → GPU: launch kernel_C
  10 个 kernel → 50-200 us 启动开销

用 CUDA Graph:
  CPU → GPU: replay(captured_graph)
  所有 kernel 一次提交, GPU 内部自动调度
  启动开销 → ~1 us (几乎为零)
```

对实时规划来说，节省的 100+ us 非常有价值——cuRobo 默认启用 (`use_cuda_graph=True`)。

### 自碰撞 Kernel 的稀疏性优化 ⭐⭐⭐

$N$ 个 link 有 $O(N^2)$ 碰撞对，但相邻 link 不需检查（它们通过关节连接，必然"接触"）。cuRobo 预计算一个**碰撞对掩码矩阵**，跳过不需要的配对：

```
7-DOF Panda 碰撞对矩阵 (0=跳过, 1=检查):
     L0  L1  L2  L3  L4  L5  L6
L0 [  0   0   0   1   1   1   1 ]
L1 [  0   0   0   0   1   1   1 ]
L2 [  0   0   0   0   0   1   1 ]
L3 [  1   0   0   0   0   0   1 ]
L4 [  1   1   0   0   0   0   0 ]
L5 [  1   1   1   0   0   0   0 ]
L6 [  1   1   1   1   0   0   0 ]

实际检查对数: 12 (而非 21 = 7×6/2)
节省: 43% 碰撞检测计算
```

当碰撞检查对数超过 512 × 1024 时，cuRobo 的 self-collision kernel 有针对**稀疏性 + GPU 线程块 + 原子操作**的专门优化——是学习 CUDA 优化的进阶素材。

### cuRobo Python API 使用 ⭐⭐

```python
# cuRobo MotionGen API 教学骨架
# 注意：cuRobo 的 Python API 随版本变化较快。下面强调数据结构边界，
# 具体 import 路径、字段名、配置文件路径请按目标版本官方示例校对。
from curobo.wrap.reacher.motion_gen import (
    MotionGen, MotionGenConfig, MotionGenPlanConfig)
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import load_yaml

# === 1. 配置 ===
# 不要把 URDF 路径塞进一个极简 dict 就当作完整 RobotConfig。
# MotionGen 需要完整机器人配置：运动学链、关节名、关节限位、碰撞球体、
# self-collision mask、collision buffer 等。通常从 cuRobo YAML 加载。
tensor_args = TensorDeviceType(device="cuda:0")
robot_yaml = load_yaml("robot_configs/panda.yml")  # 按目标版本替换为实际路径
robot_cfg = RobotConfig.from_dict(robot_yaml["robot_cfg"], tensor_args=tensor_args)

motion_gen_config = MotionGenConfig.load_from_robot_config(
    robot_cfg=robot_cfg,
    world_model=WorldConfig.from_dict({
        "cuboid": {
            "table": {
                "dims": [1.0, 1.0, 0.05],           # 桌子尺寸
                "pose": [0.5, 0.0, 0.4, 1, 0, 0, 0] # 位置+四元数
            }
        }
    }, tensor_args=tensor_args),
    use_cuda_graph=True,  # 启用 CUDA Graph 加速
    tensor_args=tensor_args,
)

motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()  # 预热: JIT 编译 + capture CUDA Graph (30-60s)

# === 2. 规划 ===
# 当前状态：current API 系列通常要求 JointState，而不是裸 torch.Tensor。
start_state = JointState.from_position(
    tensor_args.to_device([[0, -0.5, 0, -2.0, 0, 1.5, 0.7]]),
    joint_names=[
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    ],
)

# 目标位姿：通常用 Pose 封装 position + quaternion，而不是裸张量。
goal_pose = Pose(
    position=tensor_args.to_device([[0.4, 0.0, 0.5]]),
    quaternion=tensor_args.to_device([[1.0, 0.0, 0.0, 0.0]]),  # qw, qx, qy, qz
)

result = motion_gen.plan_single(
    start_state,
    goal_pose,
    plan_config=MotionGenPlanConfig(
        max_attempts=10,
        num_trajopt_seeds=12,     # 并行轨迹优化种子数
        num_graph_seeds=4,        # 并行图搜索种子数
        timeout=10.0,             # 超时 (秒)
    )
)

if result.success:
    traj = result.get_interpolated_plan()  # 插值后的平滑轨迹
    print(f"规划成功! 轨迹点数: {len(traj.position)}")
    print(f"规划时间: {result.solve_time * 1000.0:.1f} ms")
    # 期望: RTX 4090 约 30 ms, RTX 3060 约 80 ms
else:
    print(f"规划失败: {result.status}")
```

### cuRobo 与 MoveIt2 集成 ⭐⭐

NVIDIA 通过 **Isaac ROS cuMotion** 将 cuRobo 封装为 MoveIt2 规划插件：

```
MoveIt2 + cuMotion 架构:

  MoveGroupInterface (用户 API, 与标准 MoveIt2 相同)
       │
       ▼
  PlanningPipeline
       │
       ├── cuMotion Planner Plugin (替代 OMPL)
       │   ├── cuRobo MotionGen API (GPU 全栈)
       │   ├── GPU 碰撞检测 (替代 FCL)
       │   └── GPU 轨迹优化 (替代 STOMP/CHOMP)
       │
       └── nvblox (GPU ESDF 生成, 替代 OctoMap)
            └── 深度图 → GPU 体素化 → ESDF
```

```
# 安装 Isaac ROS cuMotion
# 请参考 NVIDIA Isaac ROS cuMotion 官方文档获取最新安装指引：
# https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/
# 安装方式可能因 ROS2 版本和 CUDA 环境而异，不建议直接 apt install。
```

### cuRobo 配置实战: 从 URDF 到首次规划 ⭐⭐

以下是将一个新机器人（非 Panda/UR 预置模型）接入 cuRobo 的完整流程。

**Step 1: 准备 YAML 配置文件**

cuRobo 的机器人配置不是“只有 URDF 路径”——URDF 只提供几何和运动学的一部分信息。实际规划还需要 YAML 中的关节名、关节限位、碰撞球体、self-collision mask、collision buffer 等完整配置。

```yaml
# my_robot.yml (cuRobo 机器人配置)
robot_cfg:
  kinematics:
    urdf_path: "my_robot.urdf"
    base_link: "base_link"
    ee_link: "tool0"
    link_names: null
    lock_joints: {}  # 锁定不参与规划的关节

    # 球体近似碰撞模型
    # 方法 A: 手动定义 (精确但费时)
    # 方法 B: 使用 Isaac Sim Lula Robot Description Editor 生成后再人工校验
    collision_link_names: [link_0, link_1, link_2, link_3, link_4, link_5, tool0]
    collision_spheres:
      link_0: [{center: [0, 0, 0.05], radius: 0.06},
               {center: [0, 0, 0.10], radius: 0.05}]
      link_1: [{center: [0, 0, 0.0], radius: 0.05},
               {center: [0, 0, 0.05], radius: 0.05}]
      # ... 每个 link 定义 5-15 个球体
    collision_sphere_buffer: 0.02  # 环境碰撞安全裕度 2cm
    extra_collision_spheres: {}

    # 自碰撞检测的排除对 (相邻 link 不检查)
    self_collision_ignore:
      link_0: [link_1]
      link_1: [link_2]
      # ...

    self_collision_buffer:
      link_0: 0.02
      link_1: 0.02
      # ... Dict[str, float]，按 link 设置自碰撞安全裕度

    use_global_cumul: true
    mesh_link_names: null

    # 关节空间配置 (可覆盖 URDF 中的速度/加速度/jerk 约束)
    cspace:
      joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
      retract_config: [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7]
      null_space_weight: [1, 1, 1, 1, 1, 1, 1]
      cspace_distance_weight: [1, 1, 1, 1, 1, 1, 1]
      max_jerk: 500.0
      max_acceleration: 15.0
```

**Step 2: 生成球体碰撞模型**

当前 cuRobo 主线推荐把碰撞球体写入机器人配置 YAML，或借助 Isaac Sim 的 Lula Robot Description Editor 生成/调参；不要依赖未公开稳定的 Python helper。工程流程通常是：

1. 从 URDF/USD 中选出需要参与碰撞的 link
2. 为每个 link 生成若干球体近似，并人工检查是否覆盖真实几何
3. 写入 `collision_spheres`，同步维护 `collision_link_names`、`self_collision_ignore` 与 `self_collision_buffer`
4. 用 RViz/Isaac Sim 可视化球体，确认不会漏掉夹爪、腕部和近基座粗壮 link

**Step 3: 配置世界模型**

```python
# 世界模型 (障碍物定义)
world_cfg = WorldConfig.from_dict({
    "cuboid": {
        "table": {
            "dims": [1.0, 2.0, 0.05],
            "pose": [0.5, 0.0, 0.4, 1, 0, 0, 0]
        },
        "wall": {
            "dims": [0.05, 2.0, 1.5],
            "pose": [0.8, 0.0, 0.75, 1, 0, 0, 0]
        }
    },
    "mesh": {
        "complex_obstacle": {
            "file_path": "obstacle.obj",
            "pose": [0.3, 0.2, 0.5, 1, 0, 0, 0]
        }
    }
    # 也支持 nvblox ESDF (实时深度图输入)
})
```

**Step 4: 调优 MotionGen 参数**

```python
# MotionGen 关键调参
plan_config = MotionGenPlanConfig(
    max_attempts=10,           # 最大尝试次数 (含 fallback)
    num_trajopt_seeds=12,      # 轨迹优化并行种子数
    # RTX 3060: 建议 4-8; RTX 4090: 建议 12-24
    num_graph_seeds=4,         # 图搜索并行种子数
    timeout=10.0,              # 超时 (秒)
    enable_graph=True,         # 是否启用图搜索 fallback
    # False: 只用轨迹优化 (快但可能失败)
    # True:  轨迹优化失败时 fallback 到图搜索 (更可靠)
    partial_ik_iters=10,       # 部分 IK 预解算迭代数
    enable_opt=True,           # 是否启用后优化
    need_graph_success=False,  # 是否必须图搜索成功
)
```

### 许可证与商业化考量 ⭐⭐

```
cuRobo 本体许可证: 以仓库 LICENSE 为准（公开仓库通常为 Apache-2.0）
  - 本体: 开源，可按对应开源许可证用于研究与产品原型
  - 依赖链: PyTorch、CUDA、NVIDIA 驱动/运行时、nvblox 等按各自许可证审查
  - 示例资产: 仓库中机器人模型、mesh、示例资产按各自 LICENSE_ASSETS 或上游许可审查
  - 商用集成: Isaac ROS cuMotion、Isaac Sim、NVIDIA 平台 SDK 和产品分发方式需另审

开源替代方案:
  - VAMP (Apache-2.0): CPU SIMD 加速, 完全开源, 可自由商用
  - OMPL 风格工作流 + VAMP: 需要显式桥接状态表示、FK/碰撞模型
    和 StateValidityChecker，不能假设安装 OMPL 后自动启用 SIMD
```

### ⚠️ 常见陷阱

**编程陷阱: cuRobo 首次启动的 warmup 延迟**

```
⚠️ cuRobo 首次调用 motion_gen.warmup() 需要 30-60 秒:
   - 编译 Warp kernel (根据机器人 DOF 生成 CUDA 代码)
   - Capture CUDA Graph (录制 kernel 执行序列)
   - 后续调用: 每次规划仅 ~30 ms
   工程做法: 在系统启动时 warmup, 不在实时循环中做。
   缓存: 设置 CUROBO_USE_LRU_CACHE=1 缓存已编译 kernel。
```

**概念误区: 认为 cuRobo 是"GPU 版 OMPL"**

```
💡 cuRobo 不是把 RRT/PRM 搬到 GPU 上——
   它用的是完全不同的策略: 并行多种子轨迹优化
   (类似 M08.4 的 TrajOpt, 但并行度提升 100x)。
   只在轨迹优化失败时, 才 fallback 到 GPU 图搜索 (类 PRM)。
   所以 cuRobo 的核心是 M08 讲的轨迹优化——不是 M07 的采样规划。
```

### 练习

1. **[工程]** 安装 cuRobo（需 NVIDIA GPU, RTX 3060+），用 `MotionGen` API 为 Panda 规划一条避障路径。记录端到端耗时——目标 < 100 ms。
2. **[对比]** 同一场景（Panda + 3 个障碍 box）分别用 cuRobo 和 MoveIt2(OMPL RRT-Connect + FCL) 规划。对比：规划时间、路径质量（长度）、成功率。
3. **[精读]** 阅读 `src/curobo/curobolib/cpp/sphere_obb_kernel.cu`——标注：thread ID 如何映射到碰撞对、shared memory 使用、原子操作。对照 CUDA 编程指南理解 warp 调度。

---

## M09.3 GPU 并行碰撞检测 ⭐⭐

### 球体扫描法 vs SDF 方法 ⭐⭐

cuRobo 支持两种 GPU 碰撞检测方法，适用于不同场景：

**方法 1: 球体-OBB/球体-球体直接检测**

```
机器人: 每个 link → 10-30 个球体近似
环境: 每个障碍 → OBB (Oriented Bounding Box) 或 mesh

碰撞检测:
  对每个 (球体, OBB) 对:
    1. 将球心变换到 OBB 局部坐标系
    2. 计算球心到 OBB 表面的最近距离
    3. d = distance - sphere_radius
    4. d < 0 → 碰撞

GPU 并行: 所有 (batch, sphere, OBB) 三元组同时计算
```

**方法 2: 签名距离场 (SDF / ESDF)**

```
环境: 预计算为 3D 体素网格, 每个体素存储到最近障碍的距离
机器人: 球体中心位置查询 SDF 值
碰撞: SDF(球心) < 球半径 → 碰撞
梯度: ∇SDF 指向远离障碍方向 (用于轨迹优化, 回顾 M08.2)

GPU 加速: nvblox 从深度图实时生成 ESDF (~10ms)
```

**为什么 cuRobo 选择球体近似而非 GJK mesh？**

| 因素 | GJK (mesh, 回顾 M04) | 球体近似 |
|------|---------------------|---------|
| 每对碰撞检测 | 迭代算法, 20-100 次循环 | **1 次距离公式** |
| 分支 | 大量 if-else (GJK 循环终止条件) | **无分支** |
| 内存访问 | 不规则 (mesh 顶点遍历) | **规则** (球心+半径) |
| GPU warp 利用率 | 差 (分支导致 warp divergence) | **100%** (所有 thread 执行同一路径) |
| 精度 | 精确 | 近似 (**保守**) |

> **反事实推理**: 如果 cuRobo 用 GJK mesh 碰撞检测会怎样？GJK 的迭代循环有大量条件分支（if-else），在 GPU 上同一 warp 的 32 个 thread 如果走不同分支路径（warp divergence），只能串行执行——GPU 利用率可能降到 10-30%。球体碰撞检测是纯算术操作（减法+乘法+比较），无分支 → warp 利用率接近 100%。这就是为什么 GPU 碰撞检测选择球体近似。

**球体近似的保守性**: 球体包裹 link 比 link 本身更大。这意味着可能报告"碰撞"但实际不碰（假阳性），但不会遗漏碰撞（无假阴性）。对运动规划来说，保守是安全的——最坏情况是路径稍微绕远，不会撞到障碍。

### CUDA Kernel 设计模式: 批量 FK ⭐⭐⭐

```cpp
// 教学简化: 展示 GPU 并行 FK 的核心思想
// 每个 thread 独立计算一组构型的正运动学
__global__ void batch_forward_kinematics(
    const float* joint_angles,    // [batch_size, num_joints]
    const float* dh_params,       // DH 参数 (所有 batch 共享)
    float* sphere_centers,        // [batch_size, num_spheres, 3]
    int batch_size,
    int num_joints,
    int num_spheres
) {
    int batch_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (batch_idx >= batch_size) return;

    // 每个 thread 处理一个 batch 的 FK
    // 所有 thread 执行相同代码, 不同数据 (SIMT)
    float T[16];  // 4x4 变换矩阵, 存在寄存器中 (极快)
    identity_4x4(T);

    int sphere_idx = 0;
    for (int j = 0; j < num_joints; ++j) {
        float q = joint_angles[batch_idx * num_joints + j];
        // 累乘 DH 变换矩阵
        multiply_dh(T, q, &dh_params[j * 4]);

        // 计算当前 link 上所有球体的世界坐标
        for (int s = 0; s < spheres_per_link[j]; ++s) {
            // 球体在 link 局部坐标系中的位置
            float3 local_pos = sphere_local[sphere_idx];
            // 变换到世界坐标
            sphere_centers[(batch_idx*num_spheres + sphere_idx)*3 + 0]
                = T[0]*local_pos.x + T[1]*local_pos.y + T[2]*local_pos.z + T[3];
            sphere_centers[(batch_idx*num_spheres + sphere_idx)*3 + 1]
                = T[4]*local_pos.x + T[5]*local_pos.y + T[6]*local_pos.z + T[7];
            sphere_centers[(batch_idx*num_spheres + sphere_idx)*3 + 2]
                = T[8]*local_pos.x + T[9]*local_pos.y + T[10]*local_pos.z + T[11];
            sphere_idx++;
        }
    }
}
// 调用: batch_forward_kinematics<<<(batch+255)/256, 256>>>(...);
// batch=512 → 同时计算 512 组构型的 FK!
```

### GPU FK Kernel 的性能剖析 ⭐⭐⭐

批量 FK 是 cuRobo 的第一个计算步骤——为数百个构型同时计算所有球体的世界坐标。分析其性能特征有助于理解 GPU 加速的本质。

**Roofline 分析**:

| 指标 | FK Kernel |
|------|-----------|
| 算术强度 | ~10 FLOP/byte（中等） |
| 瓶颈 | 计算受限（矩阵乘法密集） |
| 寄存器使用 | 高（每个 thread 存 4x4 矩阵 = 16 float = 64 bytes） |
| Shared Memory | 不需要（每个 thread 独立） |
| Warp Divergence | 无（所有 thread 执行相同循环） |

**为什么 FK 适合 GPU？** FK 的计算模式是"同一组 DH 参数 + 不同关节角 → 不同球心位置"——完美的 SIMT（Single Instruction Multiple Thread）模式。512 个 batch 的 FK 可以在 ~0.1ms 内完成（RTX 4090），而 CPU 串行需要 ~5ms。

**寄存器溢出风险**: 每个 thread 需要存储完整的 4x4 变换矩阵（16 个 float）。如果机器人 link 很多（如人形 30+ DOF），累积的中间矩阵可能超出每 thread 的寄存器配额（通常 255 个 32-bit 寄存器），导致溢出到 local memory（慢 100x）。cuRobo 通过**原地更新**（只维护一个 4x4 矩阵，逐 link 左乘）避免寄存器溢出。

### sphere-OBB 距离计算的数学细节 ⭐⭐⭐

球体-OBB 碰撞检测的核心是计算球心到 OBB 最近点的距离。以下是 cuRobo kernel 内部使用的数学方法。

**Step 1: 坐标变换**

将球心 $p_s$ 变换到 OBB 的局部坐标系（OBB 中心为原点，轴对齐）：

$$
p_{\text{local}} = R_{\text{obb}}^T \cdot (p_s - c_{\text{obb}})
$$

其中 $R_{\text{obb}}$ 是 OBB 的旋转矩阵，$c_{\text{obb}}$ 是 OBB 中心。在 GPU 上这是 3x3 矩阵乘法 + 向量减法，纯算术、无分支。

**Step 2: 最近点计算（clamping）**

OBB 的半边长为 $(h_x, h_y, h_z)$。局部坐标系下，OBB 表面上距 $p_{\text{local}}$ 最近的点：

$$
q_i = \text{clamp}(p_{\text{local},i}, -h_i, h_i), \quad i \in \{x, y, z\}
$$

这是三个独立的 clamp 操作——无分支（GPU 用 `fminf/fmaxf` intrinsics 实现，硬件单周期）。

**Step 3: 距离计算**

$$
d = \|p_{\text{local}} - q\| - r_s
$$

其中 $r_s$ 是球体半径。$d < 0$ 表示碰撞。

**GPU 友好性分析**: 整个流程是 15 个浮点运算（3 减法 + 9 乘加 + 3 clamp + 3 减法 + 1 平方根 + 1 减法），零分支，零条件跳转。这就是球体-OBB 碰撞检测在 GPU 上极快的根本原因——每个 warp 的 32 个 thread 走完全相同的指令路径。

### CUDA Kernel 设计模式: 批量碰撞检测 ⭐⭐⭐

```cpp
// 教学简化: 展示球-OBB 碰撞检测的 GPU 并行
__global__ void batch_sphere_obb_collision(
    const float* sphere_centers,  // [batch, num_spheres, 3]
    const float* sphere_radii,    // [num_spheres]
    const float* obb_params,      // [num_obbs, 10]
    float* min_distances,         // [batch] (每个 batch 的最小距离)
    int batch_size,
    int num_spheres,
    int num_obbs
) {
    // 每个 thread 处理一个 (batch, sphere, obb) 三元组
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = batch_size * num_spheres * num_obbs;
    if (idx >= total) return;

    int b = idx / (num_spheres * num_obbs);
    int s = (idx / num_obbs) % num_spheres;
    int o = idx % num_obbs;

    // 球心坐标
    float3 sc = make_float3(
        sphere_centers[b*num_spheres*3 + s*3 + 0],
        sphere_centers[b*num_spheres*3 + s*3 + 1],
        sphere_centers[b*num_spheres*3 + s*3 + 2]);
    float sr = sphere_radii[s];

    // 计算球到 OBB 的距离 (纯算术, 无分支)
    float dist = sphere_obb_signed_distance(sc, sr, &obb_params[o*10]);

    // 原子更新该 batch 的最小距离
    atomicMin_float(&min_distances[b], dist);
}

// === atomicMin_float 的实现 ===
// CUDA 不原生支持 float 的 atomicMin (只有 int 版本)
// cuRobo 用 CAS (Compare-And-Swap) 循环实现:
__device__ void atomicMin_float(float* addr, float val) {
    int* addr_as_int = (int*)addr;
    int old = *addr_as_int;
    int assumed;
    do {
        assumed = old;
        // 只有当 val < 当前值时才更新
        if (__int_as_float(assumed) <= val) break;
        old = atomicCAS(addr_as_int, assumed,
                        __float_as_int(val));
    } while (assumed != old);
}
// 为什么不能用简单赋值? 因为多个 thread 可能同时写同一个
// min_distances[b], 没有原子操作会导致数据竞争 (race condition).
// CAS 循环保证只有"真正更小"的值才写入.
```

### GPU 碰撞检测的内存访问模式优化 ⭐⭐⭐

GPU 性能对内存访问模式极其敏感——**合并访问(coalesced access)**是关键。

**问题**: 球心坐标存储为 `[batch][sphere][xyz]`（AoS, Array of Structures 布局）。当连续 thread 处理不同 batch 时，它们访问的内存地址不连续——stride = `num_spheres * 3`。

**cuRobo 的优化**: 将数据重排为 SoA（Structure of Arrays）布局：

```
AoS (不友好):  [b0_s0_x, b0_s0_y, b0_s0_z, b0_s1_x, ...]
               连续 thread 访问 stride = 3 → 非合并

SoA (友好):    [b0_s0_x, b1_s0_x, b2_s0_x, ..., b0_s0_y, ...]
               连续 thread 访问 stride = 1 → 完美合并
```

合并访问 vs 非合并访问的性能差距可达 10-30 倍（取决于 L2 cache 命中率）。cuRobo 在 FK kernel 输出时就按 SoA 布局存储球心坐标，避免后续碰撞 kernel 中的非合并访问。

### 与 nvblox 的感知管线 ⭐⭐

对于需要处理真实点云/深度图的场景，cuRobo 与 NVIDIA nvblox 联动：

```
深度相机 (RealSense / ZED)
       │
       ▼
  nvblox (GPU 体素化, ~10ms)
       │
       ├── 截断签名距离场 (TSDF) → 3D 重建
       └── 欧氏签名距离场 (ESDF) ← cuRobo 碰撞检测使用
            │
            ▼
  cuRobo 碰撞检测: 每个球心查询 ESDF 值
       │
       ▼
  梯度信息: ∇ESDF 用于轨迹优化 (回顾 M08.2)
```

### SDF 方法的梯度优势 ⭐⭐⭐

回顾 M08.2（代价函数设计）：轨迹优化需要碰撞代价的**梯度**。SDF 天然提供梯度信息：

$$
\nabla \text{SDF}(x) = \frac{x - x_{\text{nearest}}}{\|x - x_{\text{nearest}}\|}
$$

其中 $x_{\text{nearest}}$ 是最近障碍物表面上的点。这个梯度指向**远离障碍物的方向**——正是轨迹优化需要的"推力方向"。

球体-OBB 方法也可以计算梯度（球心到 OBB 表面的方向），但需要额外的几何计算。SDF 的优势是梯度在预计算的体素网格上直接可用（有限差分或解析），查询时间 $O(1)$。

**cuRobo 的混合策略**:
- **几何障碍（已知 OBB/mesh）**: 用球体-OBB 直接检测（精确、无需预计算）
- **感知障碍（点云/深度图）**: 用 nvblox ESDF（GPU 实时生成）
- **自碰撞**: 用球体-球体检测（机器人自身形状已知）

### 碰撞检测精度的穷举分类 ⭐⭐

| 碰撞检测方法 | 精度 | 速度 | GPU 友好 | 适用场景 |
|-------------|------|------|---------|---------|
| GJK/EPA (mesh) | 精确 | 慢 (50-500 us) | 差 | CPU 精确碰撞 (M04) |
| 球体近似 | 保守 | 极快 (0.01 us) | 极佳 | GPU 批量碰撞 (cuRobo) |
| ESDF 体素查询 | 受分辨率限制 | 快 (0.1 us) | 好 | 感知障碍 (nvblox) |
| 胶囊体近似 | 保守但更紧 | 快 (0.05 us) | 好 | 细长 link |
| 凸包近似 | 精确（凸化后） | 中 (1-10 us) | 中 | 非凸 mesh 简化 |

> **反事实推理**: 如果 cuRobo 用胶囊体代替球体会怎样？胶囊体（两端半球+中间圆柱）比球体更紧密地包裹细长 link（如 Panda 的前臂），保守性更低——但胶囊体-OBB 距离计算比球体-OBB 复杂得多（需要求线段到 OBB 的距离），GPU 上的分支也更多。cuRobo 选择球体是在精度和 GPU 友好度之间的工程权衡。

### ⚠️ 常见陷阱

**编程陷阱: 球体近似的粒度选择**

```
⚠️ 球体太少: link 的实际形状没被完全覆盖, 可能遗漏碰撞
   球体太多: GPU kernel 的计算量增加 (每增加 1 个球 → 多 num_obbs 次检测)
   经验值: Panda 7-DOF → 约 50-80 个球体 (每 link 7-10 个)
   cuRobo 提供工具自动生成球体近似模型。
   自检: 在 RViz/MuJoCo 中可视化球体模型, 确认覆盖完整。
```

**编程陷阱: ESDF 体素边界处理**

```
⚠️ 当机器人球心位于 ESDF 体素网格边界之外时，
   查询返回的距离值可能是错误的（外推不可靠）。
   现象: 机器人在工作空间边缘规划时突然碰撞。
   正确做法: ESDF 计算范围应覆盖机器人的完整可达工作空间，
            留出至少 0.5m 的额外边界。
```

### 练习

1. **[编程]** 用 cuRobo 的工具为 Panda 生成球体近似碰撞模型。可视化球体分布。对比 50 个球 vs 100 个球的规划时间和安全裕度差异。
2. **[思考]** 球体近似是**保守**的（比实际形状大）。在什么场景下保守性会导致规划失败？（提示：考虑紧密装配场景，间隙 < 球体溢出量。）
3. **[跨章综合]** 对比 M04 中 FCL 的 GJK 碰撞检测和本章的 GPU 球体碰撞检测。从精度、速度、GPU 友好度三个维度分析。在什么条件下应该选 FCL？什么条件下选 cuRobo？

---

## M09.4 VAMP——SIMD 加速规划的巅峰 ⭐⭐⭐

### 动机：不用 GPU 也能极速规划 ⭐⭐

不是每个部署环境都有 NVIDIA GPU——嵌入式 ARM 板、工业 PLC、边缘设备、或者 GPU 已被训练/推理占满。Thomason et al. (ICRA 2024, Kavraki Lab) 提出 VAMP：通过 CPU **SIMD 指令**加速碰撞检测和正运动学，在 Panda 7-DOF 上实现 **35 微秒的中位规划时间**。这是机器人领域罕见的**亚毫秒级经典算法加速**案例。

### 核心创新：手写 SIMD intrinsics ⭐⭐⭐

VAMP 不依赖编译器自动向量化（auto-vectorization），而是**手动编写 SIMD intrinsics**，精确控制每条向量指令。

```cpp
// 编译器自动向量化 (不可靠):
float a[8], b[8], c[8];
for (int i = 0; i < 8; ++i)
    c[i] = a[i] + b[i];
// 编译器"可能"生成 AVX2, 也可能不
// 取决于优化级别、循环结构、数据对齐、编译器版本

// VAMP 手写 SIMD (保证最优):
__m256 va = _mm256_load_ps(a);     // 加载 8 个 float 到 256-bit 寄存器
__m256 vb = _mm256_load_ps(b);     // 加载 8 个 float
__m256 vc = _mm256_add_ps(va, vb); // 一条指令: 8 个 float 加法
_mm256_store_ps(c, vc);             // 存储结果
// 保证生成最优 AVX2 代码, 不依赖编译器
```

> **跨领域类比**: VAMP 手写 SIMD vs 编译器自动向量化，类似于**手写汇编 vs 高级语言编译**。大多数场景下编译器足够好，但在极致性能场景（游戏引擎物理、视频编解码、加密算法），手写向量化仍是标准做法。VAMP 证明了机器人碰撞检测也属于"值得手写"的领域——手写 SIMD 比 Eigen 自动向量化快 3-10 倍。

### SIMD 抽象接口 ⭐⭐⭐

VAMP 的精妙设计：定义抽象向量类型，具体实现在编译时通过模板特化选择：

```cpp
// impl/vamp/vector/interface.hh (教学简化)
template <unsigned N>
struct FloatVector {
    // 平台无关的抽象操作
    FloatVector operator+(FloatVector other) const;
    FloatVector operator*(FloatVector other) const;
    FloatVector min(FloatVector other) const;
    FloatVector sqrt() const;
    bool any_less_than(FloatVector other) const;
};

// impl/vamp/vector/avx.hh (x86 AVX2 实现, N=8)
template <>
struct FloatVector<8> {
    __m256 data;  // 256-bit AVX2 寄存器

    FloatVector operator+(FloatVector o) const {
        return {_mm256_add_ps(data, o.data)};
    }
    FloatVector operator*(FloatVector o) const {
        return {_mm256_mul_ps(data, o.data)};
    }
    FloatVector min(FloatVector o) const {
        return {_mm256_min_ps(data, o.data)};
    }
    FloatVector sqrt() const {
        return {_mm256_sqrt_ps(data)};  // 8 个 sqrt 一条指令!
    }
};

// impl/vamp/vector/neon.hh (ARM NEON 实现, N=4)
template <>
struct FloatVector<4> {
    float32x4_t data;  // 128-bit NEON 寄存器

    FloatVector operator+(FloatVector o) const {
        return {vaddq_f32(data, o.data)};
    }
    // ... ARM NEON 实现 ...
};
```

> **本质洞察**: VAMP 的 SIMD 抽象层是**策略模式(Strategy Pattern, C++ 基础设计模式章节）**在底层硬件上的极致应用。通常策略模式用于选择算法（如 M07 中 OMPL 的 Planner 基类），VAMP 把它下沉到了**指令级**——同一套碰撞检测代码，在 x86 上编译为 AVX2 指令，在 ARM 上编译为 NEON 指令。用户代码写一份，跨平台自动最优。

### SIMD 碰撞检测的实现 ⭐⭐⭐

```
传统碰撞检测 (串行, 每次 1 对):
  for each sphere_pair (s1, s2):
    dx = s1.cx - s2.cx
    dy = s1.cy - s2.cy
    dz = s1.cz - s2.cz
    dist_sq = dx*dx + dy*dy + dz*dz
    r_sum = s1.r + s2.r
    if dist_sq < r_sum * r_sum: collision!

VAMP SIMD 碰撞检测 (AVX2, 同时 8 对):
  // 8 对球的 Δx, Δy, Δz 一起算
  dx8 = _mm256_sub_ps(cx1_vec, cx2_vec)   // 8 个 Δx
  dy8 = _mm256_sub_ps(cy1_vec, cy2_vec)   // 8 个 Δy
  dz8 = _mm256_sub_ps(cz1_vec, cz2_vec)   // 8 个 Δz
  dist_sq8 = _mm256_fmadd_ps(dx8, dx8,    // 8 个距离²
             _mm256_fmadd_ps(dy8, dy8,
             _mm256_mul_ps(dz8, dz8)))
  r_sum8 = _mm256_add_ps(r1_vec, r2_vec)  // 8 个半径和
  r_sum_sq8 = _mm256_mul_ps(r_sum8, r_sum8)
  mask = _mm256_cmp_ps(dist_sq8, r_sum_sq8, _CMP_LT_OS)
  // mask 的每个 bit: 对应球对是否碰撞
  // 一组指令检查 8 对球! (约 1.5 ns vs 串行 10 ns)
```

### SIMD FK 的向量化 ⭐⭐⭐

VAMP 不仅向量化碰撞检测，还向量化了正运动学（FK）计算。传统 FK 是"一个构型 → 一组球心位置"的串行流程；VAMP 改为"8 个构型 → 8 组球心位置"的 SIMD 并行流程。

```cpp
// VAMP SIMD FK (教学简化, AVX2, 同时计算 8 个构型)
void simd_forward_kinematics_8(
    const FloatVector<8>* joint_angles,  // 8 组关节角
    FloatVector<8>* sphere_centers,       // 8 组球心坐标
    int num_joints
) {
    // 变换矩阵的每个元素都是 FloatVector<8>
    // → 8 个 4x4 矩阵的元素并行计算
    FloatVector<8> T[16];  // 4x4 矩阵展开为 16 个 FloatVector<8>
    identity_4x4_simd(T);

    for (int j = 0; j < num_joints; ++j) {
        FloatVector<8> q = joint_angles[j];

        // sin/cos 的 SIMD 计算: 8 个 sin + 8 个 cos 同时完成
        FloatVector<8> sq = q.sin();  // _mm256_sin_ps (Intel SVML)
        FloatVector<8> cq = q.cos();  // _mm256_cos_ps

        // DH 变换矩阵乘法: 8 组矩阵并行相乘
        multiply_dh_simd(T, sq, cq, dh_params[j]);

        // 提取球心: T 的平移列, 已经是 8 组坐标
        for (int s = 0; s < spheres_per_link[j]; ++s) {
            sphere_centers[idx + 0] = T[3];   // 8 个 x 坐标
            sphere_centers[idx + 1] = T[7];   // 8 个 y 坐标
            sphere_centers[idx + 2] = T[11];  // 8 个 z 坐标
            idx += 3;
        }
    }
}
```

**性能对比**: 串行 FK 一次处理 1 个构型约 1 us。SIMD FK 一次处理 8 个构型约 1.5 us——吞吐率提升 5.3 倍。这在 RRT 的内循环中极为关键：每次 `EXTEND` 操作都需要 FK → 碰撞检测 → 判断，SIMD 把整个内循环加速。

### SIMD Halton 采样 ⭐⭐⭐

VAMP 的采样也用 SIMD 加速（`impl/vamp/random/halton.hh`）：一次生成 8 个维度的 Halton 低差异序列样本。Halton 序列比伪随机数在高维空间中分布更均匀——减少"聚团"现象，提高采样效率。

### 编译器自动向量化为什么不够? ⭐⭐⭐

一个常见疑问：既然编译器（GCC/Clang -O3 -march=native）可以自动向量化，为什么 VAMP 还要手写 intrinsics？

| 因素 | 编译器自动向量化 | VAMP 手写 SIMD |
|------|----------------|---------------|
| **依赖分析** | 保守（遇到指针别名就放弃） | 人工保证无别名 |
| **循环结构** | 只对简单 for 循环有效 | 可向量化任意计算模式 |
| **数据布局** | 要求 AoS→SoA 自动转换（通常失败） | 手动按 SoA 设计数据 |
| **特殊指令** | 不会用 `_mm256_movemask_ps` 等 | 利用特殊指令加速分支判断 |
| **可预测性** | 换编译器/版本可能退化 | **保证**生成最优代码 |

实测：同一份碰撞检测代码，GCC 13 -O3 -mavx2 自动向量化后比标量快 ~2 倍，VAMP 手写 intrinsics 快 ~8 倍。差距主要来自数据布局优化和特殊指令利用。

### CAPT 数据结构 (RSS 2024) ⭐⭐⭐⭐

Thomason et al. 在 RSS 2024 论文中提出 CAPT (Collision-Affording Point Tree)——专为 SIMD 友好设计的最近邻数据结构，加速碰撞检测 inner loop 中的空间查询。

### VAMP 性能数据 ⭐⭐

| 机器人 | 中位规划时间 | 碰撞检测吞吐 |
|--------|------------|-------------|
| Panda (7-DOF) | **35 us** | ~50M checks/sec |
| UR5 (6-DOF) | 28 us | ~60M checks/sec |
| Fetch (8-DOF) | 45 us | ~40M checks/sec |

**注意**: 这是**纯采样规划**（类似 RRT-Connect）的时间，不含轨迹优化。要得到和 cuRobo 可比的全栈性能，需要加上后续的轨迹优化 (M08) 和时间参数化 (M10) 步骤。

### nanobind Python 绑定 ⭐⭐

VAMP 使用 **nanobind**（pybind11 的后继项目）提供 Python 接口。nanobind 相比 pybind11 的优势：
- 编译时间短 2-5 倍
- 运行时开销更低（更小的共享库）
- 更好的 NumPy 互操作

这使得 VAMP 既可以作为 C++ 库嵌入实时系统，也可以通过 Python 快速原型验证。VAMP 和 Ruckig（时间最优轨迹生成，M10 将讲）是 nanobind 在机器人库中的标杆采用者。

### VAMP-MR: 多臂协同规划 ⭐⭐⭐

VAMP 团队在 AAAI 2026 WoMAPF 上提出 VAMP-MR——将 SIMD 加速扩展到**多臂协同规划**。当多臂共享工作空间时，需要检测臂间碰撞——碰撞对数从 $O(N^2)$（N = 单臂球体数）增长到 $O(M^2 N^2)$（M = 臂数）。VAMP-MR 用 SIMD 并行化这些检测，碰撞检查+规划+后处理加速两个数量级。

### OMPL / MoveIt 工作流中的 VAMP 集成边界 ⭐⭐

VAMP 的价值在于把特定机器人模型的 FK、球体碰撞和采样批处理改写成 SIMD 友好的数据布局。它可以接入 OMPL 风格的采样规划工作流，但这通常需要显式桥接：把 OMPL state 转成 VAMP 的批量状态格式、用 VAMP 的 FK/碰撞模型实现 `StateValidityChecker` 或替换局部规划模块，并保证环境几何能被 VAMP 支持的模型表达。

因此，不应把“OMPL 2.0 + VAMP”理解成自动加速所有 OMPL 规划器。能否加速取决于机器人是否有对应的 VAMP 模型、环境是否可转成支持的碰撞表示、规划器调用碰撞检查的方式是否能批量化，以及 MoveIt2/OMPL 集成层是否真的走到了 VAMP 后端。

OMPL 2.x 本身也在演进新的状态空间和约束规划能力；这些能力可以与外部 SIMD 后端形成互补，但仍需要工程集成和基准验证。

### ⚠️ 常见陷阱

**概念误区: 认为 VAMP 比 cuRobo 快 1000 倍**

```
💡 VAMP 的 35 us (采样规划) vs cuRobo 的 30 ms (全栈运动生成)
   看起来差近 1000 倍——但它们解决的问题规模不同:
   - VAMP: 仅采样规划 (几何路径, 不含轨迹优化/时间参数化)
   - cuRobo: 完整运动生成 (IK + 碰撞 + 轨迹优化 + 时间参数化)
   公平对比: VAMP 35us + STOMP 50ms ≈ 50ms (后端仍在 CPU)
            cuRobo 30ms (全部在 GPU)
   两者是互补的, 不是竞争关系:
     有 GPU → cuRobo; 无 GPU → VAMP + CPU 优化
```

### 练习

1. **[工程]** 安装 VAMP（需支持 AVX2 的 CPU，用 `lscpu | grep avx2` 检查），用 Python 绑定为 Panda 规划。记录中位规划时间——是否接近论文的 35 us？
2. **[精读]** 阅读 VAMP 的 `impl/vamp/vector/avx.hh` 和 `neon.hh`。对比两种平台的 `add`、`min`、`sqrt` 实现。理解跨平台 SIMD 抽象的设计思路。
3. **[对比实验]** 写一段简单的碰撞检测（两球距离）：一份用 Eigen（依赖编译器自动向量化），一份手写 AVX2 intrinsics。Benchmark 对比——预期手写 SIMD 快 3-10 倍。

---

## M09.5 CPU SIMD vs GPU CUDA 选型决策 ⭐⭐

### 决策矩阵 ⭐⭐

| 维度 | CPU SIMD (VAMP) | GPU CUDA (cuRobo) |
|------|----------------|-------------------|
| **硬件要求** | 任意现代 CPU (AVX2 or NEON) | NVIDIA GPU (Turing+, RTX 3060+) |
| **适用规模** | 小规模 (1-10 并行种子) | 大规模 (100-1000+ 并行种子) |
| **延迟** | 极低 (35 us 采样规划) | 中 (30 ms 全栈运动生成) |
| **吞吐** | 中 (单次快, 但不能大规模并行) | 高 (批量规划、多目标搜索) |
| **完整度** | 仅采样规划 (需另加 M08 优化) | **全栈** (IK+碰撞+优化+参数化) |
| **部署** | 嵌入式友好 (ARM Cortex-A, Jetson CPU) | 需 GPU (Jetson GPU / 桌面 GPU) |
| **许可证** | **Apache-2.0** (完全开源, 可商用) | cuRobo 本体开源（通常 Apache-2.0）；依赖、示例资产和商用集成另审 |
| **最佳场景** | 单臂快速重规划、嵌入式、无GPU | 多臂并行、复杂碰撞、GPU 可用 |

### 决策流程 ⭐⭐

```
        ┌───────────────────────────────┐
        │    有 NVIDIA GPU (RTX 3060+)? │
        └───────────────┬───────────────┘
                ┌───────┴───────┐
                ▼               ▼
              有               没有
                │               │
        需要全栈运动生成?    需要极低延迟?
       (含轨迹优化+时间参数化)   (<1ms)
                │               │
          ┌─────┴──────┐   ┌───┴────┐
          ▼            ▼   ▼        ▼
        是            否  是       否
          │            │   │        │
      cuRobo      OMPL+   VAMP    OMPL
     (30ms全栈)   VAMP碰撞 (35us)  (默认, 够用就不加速)
```

### 性能基准: 统一对比 ⭐⭐

以下数据基于 Panda 7-DOF + 桌面抓取场景 (3 个 box 障碍):

| 系统 | 规划时间 (中位) | 路径长度 (归一化) | 成功率 | 硬件 |
|------|---------------|-----------------|--------|------|
| OMPL RRT-Connect + FCL | 120 ms | 1.0 (基线) | 95% | i7-12700 |
| OMPL RRT-Connect + VAMP 桥接碰撞 | 15 ms | 1.0 | 97% | i7-12700 |
| VAMP (独立) | 0.035 ms | 1.1 | 98% | i7-12700 |
| cuRobo (全栈) | 30 ms | **0.85** | **99%** | RTX 4090 |
| cuRobo (Jetson Orin) | 100 ms | 0.85 | 99% | Orin |

**解读**:
1. **VAMP 最快**（35 us），但只做采样规划——路径质量不如 cuRobo
2. **cuRobo 路径质量最好**（0.85x 基线长度），因为内含轨迹优化
3. **OMPL + VAMP 桥接** 是不需要 GPU 时的一个折中（示例中 15ms，8x 加速），前提是模型和环境能走 VAMP 后端
4. **cuRobo 成功率最高**（99%），因为并行多种子覆盖更多拓扑

### 不同场景复杂度下的性能变化 ⭐⭐

基准数据对场景复杂度高度敏感。以下是不同复杂度下的详细对比：

**场景 A: 简单（3 个 box 障碍）**

| 系统 | 规划时间 | 成功率 | 碰撞检测调用次数 |
|------|---------|--------|----------------|
| OMPL + FCL | 45 ms | 98% | ~2,000 |
| OMPL + VAMP 桥接碰撞 | 5 ms | 99% | ~2,000 |
| VAMP 独立 | 0.02 ms | 99% | ~500 |
| cuRobo | 25 ms | 99%+ | ~100,000 (并行) |

**场景 B: 中等（20 个 box + 桌面 + 架子）**

| 系统 | 规划时间 | 成功率 | 碰撞检测调用次数 |
|------|---------|--------|----------------|
| OMPL + FCL | 200 ms | 90% | ~8,000 |
| OMPL + VAMP 桥接碰撞 | 25 ms | 93% | ~8,000 |
| VAMP 独立 | 0.05 ms | 92% | ~2,000 |
| cuRobo | 35 ms | 98% | ~500,000 (并行) |

**场景 C: 困难（杂乱桌面 50+ 物体 + 窄通道）**

| 系统 | 规划时间 | 成功率 | 碰撞检测调用次数 |
|------|---------|--------|----------------|
| OMPL + FCL | 2000+ ms | 70% | ~50,000 |
| OMPL + VAMP 桥接碰撞 | 200 ms | 75% | ~50,000 |
| VAMP 独立 | 0.5 ms | 72% | ~15,000 |
| cuRobo | 80 ms | **92%** | ~2,000,000 (并行) |

**关键观察**:
- 简单场景中 VAMP 优势最大（CPU 够快，GPU 的启动开销是负担）
- 困难场景中 cuRobo 优势最大（多种子并行覆盖更多拓扑，成功率显著提升）
- OMPL + VAMP 桥接不是通用开关；在支持的机器人/环境表示上收益明显，在复杂 mesh、动态场景或无法批量化的检查链路中收益会下降
- 碰撞检测总次数：cuRobo 比 OMPL 多 10-100x——但 GPU 并行执行，总时间反而更短

> **本质洞察**: cuRobo 的策略是"用暴力并行弥补单次碰撞检测的精度不足"——球体近似不如 GJK 精确，但 GPU 上可以并行检测百万级球对。这在困难场景中尤其有效：多种子覆盖了多种拓扑路径，即使部分种子因为球体过度保守而失败，其他种子仍可能成功。

### Isaac Sim 中的 GPU 规划 ⭐⭐

NVIDIA Isaac Sim 将 cuRobo 深度集成：

```
Isaac Sim GPU 规划管线:

  USD 场景 (物理引擎 PhysX)
       │
       ├── PhysX GPU 仿真 (多体动力学)
       ├── nvblox GPU ESDF (实时深度图→距离场)
       └── cuRobo GPU 规划 (实时运动生成)
            │
            └── Isaac ROS Bridge → 真机执行
```

### 实时重规划架构设计 ⭐⭐⭐

GPU 加速的最大价值不是"更快地做一次规划"，而是"能在每个控制周期重新规划"。以下是完整的实时重规划架构。

**时序对比**:

```
传统 (CPU, 200ms 规划):
  t=0:    规划 → t=200ms: 开始执行 → t=500ms: 执行中...
  如果 t=300ms 障碍物移动 → 无法反应! (盲目执行旧路径)

GPU (cuRobo, 30ms 规划):
  t=0:    规划₁ → 执行
  t=33ms: 规划₂ (已包含新障碍物信息) → 更新执行路径
  t=66ms: 规划₃ → 再次更新
  ...
  持续反应式规划! 每 33ms 更新一次路径
```

**实时重规划架构（生产级）**:

```
┌─────────────────────────────────────────────────────────────┐
│                 实时重规划控制循环 (30Hz)                      │
│                                                             │
│  ┌──────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ 感知线程  │───→│ 世界模型更新 │───→│ cuRobo 规划线程  │  │
│  │ (nvblox)  │    │ (ESDF 更新)  │    │ (GPU MotionGen)  │  │
│  │ ~10ms    │    │ ~5ms         │    │ ~30ms            │  │
│  └──────────┘    └──────────────┘    └────────┬─────────┘  │
│                                                │            │
│  ┌──────────────────────────────────────────────▼─────────┐ │
│  │ 轨迹混合器 (Trajectory Blender)                         │ │
│  │                                                         │ │
│  │  旧轨迹: ────────────●─────────────────                │ │
│  │  新轨迹:              ╲                                │ │
│  │  混合轨迹:  ──────────╳════════════════                │ │
│  │              ↑ 混合窗口 (50-100ms)                      │ │
│  │                                                         │ │
│  │  关键: 不能在当前位置突然切换到新轨迹                      │ │
│  │        需要平滑过渡, 否则电机抖动                          │ │
│  └───────────────────────────────────┬─────────────────────┘ │
│                                       │                      │
│  ┌────────────────────────────────────▼─────────────────────┐│
│  │ ros2_control JointTrajectoryController (1kHz)            ││
│  │ 跟踪混合后的轨迹                                         ││
│  └──────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

**轨迹混合的关键技术**:

当新规划结果到达时，不能直接替换当前轨迹——这会导致位置/速度/加速度跳变。需要在**混合窗口**内平滑过渡：

```python
# 轨迹混合 (教学简化)
def blend_trajectories(old_traj, new_traj, t_blend_start, window=0.1):
    """
    在 [t_blend_start, t_blend_start + window] 内
    从 old_traj 平滑过渡到 new_traj
    """
    def blended(t):
        if t < t_blend_start:
            return old_traj(t)
        elif t > t_blend_start + window:
            return new_traj(t)
        else:
            # 余弦混合 (C¹ 连续)
            alpha = 0.5 * (1 - np.cos(np.pi *
                    (t - t_blend_start) / window))
            return (1 - alpha) * old_traj(t) + alpha * new_traj(t)
    return blended
```

> **跨领域类比**: 轨迹混合类似于音频处理中的**交叉淡入淡出(crossfade)**——两段音频不能硬切（会产生爆音），需要在重叠窗口内平滑过渡。轨迹混合的"爆音"就是关节加速度跳变——电机会发出异响并磨损减速器。

**实时重规划的安全保障**:

| 故障场景 | 保护机制 |
|---------|---------|
| 规划超时（>33ms） | 继续执行上一次的轨迹，下个周期重试 |
| 规划失败 | 执行紧急停止轨迹（减速到零） |
| ESDF 更新延迟 | 使用上一帧的 ESDF，增加安全裕度补偿 |
| GPU 故障/掉电 | CPU fallback 到 VAMP + 紧急停止 |

### 端到端延迟分解 (Panda 7-DOF + RTX 4090) ⭐⭐

```
cuRobo 全栈运动生成延迟分解:

  总延迟: ~30 ms
  ├── GPU-IK (L-BFGS, 512 种子): 3 ms
  ├── 图搜索 (条件执行): 0-5 ms (仅 fallback 时)
  ├── 轨迹优化 (L-BFGS, 12 种子 × 32 路径点):
  │   ├── FK (512 batch): 0.5 ms
  │   ├── 碰撞检测 (球-OBB): 2 ms
  │   ├── L-BFGS 梯度计算: 8 ms
  │   ├── L-BFGS 线搜索: 5 ms
  │   └── 小计: ~15 ms
  ├── 后处理 (修剪+时间参数化): 2 ms
  ├── CUDA Graph 启动开销: ~1 ms
  └── CPU↔GPU 数据传输: ~1 ms
```

**GPU 型号对性能的影响**:

| GPU 型号 | 全栈延迟 | CUDA Cores | 显存 | 适用场景 |
|---------|---------|-----------|------|---------|
| RTX 4090 | ~30 ms | 16384 | 24 GB | 桌面/工作站 |
| RTX 3060 | ~80 ms | 3584 | 12 GB | 入门级研究 |
| Jetson Orin (GPU) | ~100 ms | 2048 | 32 GB (共享) | 嵌入式部署 |
| Jetson Orin NX | ~150 ms | 1024 | 16 GB (共享) | 低成本嵌入式 |
| A100 | ~25 ms | 6912 | 40/80 GB | 数据中心/仿真 |

> **反事实推理**: 如果不用 CUDA Graph 会怎样？cuRobo 的一次规划涉及约 20-30 个 kernel（FK、碰撞、梯度、线搜索等）。每个 kernel 启动有 5-20 us 的 CPU-GPU 同步开销。不用 CUDA Graph 时，启动开销约 100-600 us——占总规划时间的 1-2%。看似不多，但在 30Hz 控制循环中每毫秒都重要。CUDA Graph 把所有 kernel 打包为一次提交，启动开销降到 ~1 us。

### cuRobo 实时重规划代码 ⭐⭐⭐

```python
# cuRobo 实时重规划循环（教学骨架）
# 重点：实时循环仍使用 JointState/Pose 等类型化输入；不要把裸 GPU tensor
# 直接传给 plan_single()，也不要把示例中的加载函数当作稳定 API。
import time
from curobo.wrap.reacher.motion_gen import (
    MotionGen, MotionGenConfig, MotionGenPlanConfig)
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState

# === 1. 初始化 (一次性, ~30s) ===
tensor_args = TensorDeviceType(device="cuda:0")
robot_cfg, joint_names = load_full_curobo_robot_config("panda.yml")  # 版本相关：返回完整 RobotConfig

config = MotionGenConfig.load_from_robot_config(
    robot_cfg=robot_cfg,
    world_model=WorldConfig(),   # 空世界 (后续动态更新)
    use_cuda_graph=True,
    tensor_args=tensor_args,
)
motion_gen = MotionGen(config)
motion_gen.warmup()

# === 2. 实时循环 ===
current_traj = None
replan_rate = 30  # Hz
dt = 1.0 / replan_rate

while robot_running:
    t_start = time.perf_counter()

    # Step A: 获取当前状态
    q_current = get_joint_state()  # 从 ros2_control
    current_state = JointState.from_position(
        tensor_args.to_device([q_current]),
        joint_names=joint_names,
    )

    # Step B: 更新世界模型 (来自 nvblox 或传感器)
    new_obstacles = get_obstacle_updates()
    if new_obstacles:
        motion_gen.update_world(new_obstacles)

    # Step C: 获取目标 (可能是移动目标)
    target_xyz, target_quat_wxyz = get_current_goal_pose()
    goal_pose = Pose(
        position=tensor_args.to_device([target_xyz]),
        quaternion=tensor_args.to_device([target_quat_wxyz]),
    )

    # Step D: GPU 规划
    result = motion_gen.plan_single(
        current_state, goal_pose,
        plan_config=MotionGenPlanConfig(
            max_attempts=4,         # 实时模式: 减少尝试次数
            num_trajopt_seeds=8,    # 实时模式: 减少种子数
            timeout=0.025,          # 25ms 超时 (留 8ms 给其他)
        )
    )

    # Step E: 轨迹混合与执行
    if result.success:
        new_traj = result.get_interpolated_plan()
        if current_traj is not None:
            current_traj = blend(current_traj, new_traj)
        else:
            current_traj = new_traj
        send_to_controller(current_traj)  # 发送给 ros2_control
    else:
        # 规划失败: 继续执行旧轨迹 (如果有)
        # 或执行紧急减速
        if current_traj is None:
            emergency_stop()

    # Step F: 控制循环定时
    elapsed = time.perf_counter() - t_start
    if elapsed < dt:
        time.sleep(dt - elapsed)
```

### ⚠️ 常见陷阱

**思维陷阱: 认为 GPU 规划总是更好**

```
🧠 GPU 规划有以下限制:
   1. 首次启动慢 (warmup 30-60秒): 不适合"冷启动"场景
   2. GPU 内存占用 (cuRobo ~2-4GB): 可能与 RL 训练/推理竞争
   3. host↔device 数据传输延迟: 小问题不值得用 GPU
   4. 依赖链审查: cuRobo 本体开源（通常 Apache-2.0，以仓库 LICENSE 为准），
      但 CUDA/NVIDIA 平台、示例资产、机器人模型和下游集成包需分别审查
   5. 约束规划支持有限: 复杂约束仍需 OMPL (回顾 M07.7)
   决策原则: 传统方案够用就不加速。
            GPU 加速是"需要实时反应式规划"时的选择。
```

### 练习

1. **[设计]** 你要为一个仓库分拣机器人选择规划方案。硬件：Jetson Orin (有 GPU)。需求：10Hz 实时重规划，动态环境（传送带物体移动）。选 cuRobo 还是 VAMP？给出理由。
2. **[思考]** cuRobo 本体开源（通常 Apache-2.0，以仓库 LICENSE 为准），但商业产品仍要审查哪些依赖和集成包？如果公司不想绑定 NVIDIA CUDA 生态，有哪些替代路线？（提示：考虑 VAMP/CPU SIMD、OpenCL、ROCm 或传统 OMPL/Tesseract fallback。）

---

## M09.6 局限性与适用场景分析 ⭐⭐

### GPU 加速运动规划的局限性 ⭐⭐

| 局限 | 影响 | 缓解措施 |
|------|------|---------|
| **GPU 硬件依赖** | 无 GPU 环境无法使用 cuRobo | VAMP (CPU SIMD) 作为替代 |
| **首次启动延迟** | warmup 30-60 秒 | 系统启动时预热, LRU 缓存 |
| **球体近似精度** | 紧密装配场景可能失败 | 增加球体数量 / 降低球体溢出量 |
| **依赖链许可审查** | cuRobo 本体开源（通常 Apache-2.0）；示例资产、机器人模型、CUDA/NVIDIA 运行时和商用集成需分别审查 | 保留 VAMP/OMPL/Tesseract fallback，并在产品化前做 SBOM |
| **GPU 内存竞争** | 与训练/推理竞争 GPU 内存 | GPU 内存管理 / 多 GPU / 时间分片 |
| **SDF 更新延迟** | nvblox ESDF 重建需 ~10ms | 增量更新 / 降低分辨率 |
| **约束规划支持** | cuRobo 对复杂约束支持有限 | 结合 OMPL 约束规划 (M07.7) |
| **非 NVIDIA 平台** | AMD/Intel GPU 不支持 cuRobo | VAMP (CPU SIMD) 跨平台 |
| **调试困难** | GPU kernel 错误难以定位 | 先在 CPU 模式验证正确性，再切 GPU |
| **确定性** | GPU 浮点运算顺序不确定 | 同一输入可能产生微小不同的路径 |

### cuRobo 2.0 与 NVIDIA Isaac Manipulator 最新进展 ⭐⭐⭐

cuRobo 自 2023 年发布以来经历了快速迭代。截至 2026 年的主要进展：

| 版本/里程碑 | 时间 | 关键更新 |
|------------|------|---------|
| cuRobo 0.7 | 2024 Q1 | 多种子轨迹优化稳定性改进、MoveIt2 cuMotion 插件发布 |
| cuRobo 0.8 | 2024 Q3 | 初步约束规划支持（末端朝向约束）、CUDA Graph 自动调优 |
| Isaac Manipulator | 2025 | 整合 cuRobo + Isaac Sim + Perception 的端到端操控框架 |
| cuRobo 2.0（预期） | 2025-2026 | 多臂支持、接触感知规划、Transformer 运动策略集成 |

**Isaac Manipulator** 是 NVIDIA 在 2025 年推出的面向工业部署的操控框架，将 cuRobo 的 GPU 运动生成与 FoundationPose（6D 位姿估计）和 nvblox（3D 重建）整合为端到端管线——从 RGBD 图像直接到可执行轨迹。对工业用户而言，这意味着不再需要手动拼接感知和规划模块。

> **本质洞察**：cuRobo 的演进方向不是"更快的采样规划"，而是"GPU 上的端到端运动生成"。传统管线（感知 → 场景理解 → 规划 → 优化 → 执行）中的每个环节都在 GPU 上并行化，最终目标是将整个管线压缩到一个 CUDA Graph 中，实现 10 ms 级别的从传感器到执行器的延迟。

### GPU 加速的未来趋势 (2025-2027) ⭐⭐⭐

| 趋势 | 进展 | 影响 |
|------|------|------|
| **Transformer 规划器** | MotionPolicy Networks, SceneRobot Transformer | 学习的规划器可能部分替代采样规划 |
| **cuRobo 2.0 多臂**（预期） | 双臂/多臂协同 GPU 并行优化 | 扩展到双臂操作场景 |
| **Isaac Manipulator 工业化** | 端到端感知-规划-执行 GPU 管线 | 降低工业部署门槛 |
| **端到端感知-规划** | 深度图直接输入规划器（跳过 SDF） | 减少 10 ms ESDF 重建延迟 |
| **Apple/AMD GPU** | Metal/ROCm 运动规划 | 打破 NVIDIA 垄断 |
| **RISC-V Vector 扩展** | RVV 向量化碰撞检测 | VAMP 式加速在 RISC-V 嵌入式平台 |

**对从业者的建议**: 2026 年的选择是 cuRobo/Isaac Manipulator (GPU 全栈) 或 VAMP (CPU SIMD)。关注 Transformer 规划器和 Isaac Manipulator 的进展——传统采样+优化的管线正在被 GPU 端到端方案和学习方法逐步替代。保持代码的模块化设计（回顾 M07.5 的策略模式），以便未来切换规划后端。

### 穷举式场景分类与方案推荐 ⭐⭐

| 场景 | 环境 | 实时性 | 硬件 | 推荐方案 |
|------|------|--------|------|---------|
| 工厂产线 | 静态 | 离线 | 无 GPU | OMPL PRM* + STOMP (M07+M08) |
| 桌面抓取 | 半动态 | 100ms | 有 GPU | **cuRobo** |
| 人机协作 | 动态 | 30ms | 有 GPU | **cuRobo + nvblox** |
| 嵌入式机器人 | 半动态 | 10ms | ARM CPU | **VAMP** |
| 多臂协同 | 静态/半动态 | 100ms | CPU | **VAMP-MR** |
| MoveIt2 标准集成 | 任意 | 200ms | 任意 | OMPL/MoveIt 默认；按模型显式桥接 VAMP |
| Isaac Sim 仿真 | 虚拟 | 实时 | GPU | cuRobo + Isaac Sim |
| 商业产品化 | 任意 | 任意 | 任意 | VAMP (Apache-2.0) 或 Isaac ROS |

### ⚠️ 常见陷阱

**思维陷阱: 过度解读基准数据**

```
🧠 基准数据高度依赖具体条件:
   1. 场景复杂度 (3 box vs 100 box 结果完全不同)
   2. 硬件 (RTX 4090 vs RTX 3060 速度差 2-3x)
   3. 配置参数 (cuRobo 种子数、OMPL 超时等)
   实际部署时必须在自己的场景和硬件上做基准测试。
   论文数据只提供数量级参考, 不能直接作为选型依据。
```

### 练习

1. **[思考]** cuRobo 的球体近似在什么场景下会成为瓶颈？如何设计一个"混合碰撞检测"方案——大部分用球体近似（快），只在精度要求高的区域用 mesh GJK（准）？
2. **[跨章综合]** 画出从 M04(碰撞检测) → M07(OMPL) → M08(轨迹优化) → M09(GPU加速) 的完整技术栈演进图。标注每一步解决了什么问题、引入了什么新能力、留下了什么新问题。

---

## M09.7 本章小结

| 知识点 | 核心要点 | 工程价值 |
|--------|---------|---------|
| 瓶颈分析 (M09.1) | 碰撞检测占规划 80-95% 时间 | GPU 加速的根本动机 |
| cuRobo 架构 (M09.2) | CUDA C++ + PyTorch 混合栈, 并行多种子优化 | 全栈 GPU 运动生成 30ms |
| cuRobo 配置实战 (M09.2) | YAML 配置 + 球体生成 + 参数调优 | 新机器人接入 cuRobo |
| 实时重规划架构 (M09.5) | 感知→规划→混合→执行循环 | 动态环境人机协作 |
| GPU 碰撞检测 (M09.3) | 球体近似 + GPU 并行, 无分支 | 碰撞检测加速 100x+ |
| VAMP SIMD (M09.4) | 手写 AVX2/NEON intrinsics, 跨平台 | 无 GPU 环境的极速规划 |
| 选型决策 (M09.5) | GPU vs SIMD vs 传统, 场景驱动 | 硬件-场景匹配选型 |
| 详细 benchmark (M09.5) | 简单/中等/困难三级场景对比 | 场景复杂度驱动的性能分析 |
| 局限性 (M09.6) | 球体精度/许可证/GPU 依赖/约束支持 | 工程 tradeoff 意识 |
| 未来趋势 (M09.6) | Transformer 规划器/多臂 GPU/端到端 | 技术路线前瞻 |

---

## 累积项目：本章新增模块

**机械臂全栈项目进度**:
- M01: 加载 URDF → Pinocchio Model
- M03: IK 求解器 (opw/TRAC-IK/pick-ik)
- M04: 碰撞检测管线 (FCL/Coal) + SDF 生成
- M07: OMPL 运动规划 (RRT-Connect/BIT*) → 几何路径
- M08: 轨迹优化 (CHOMP/STOMP) → 平滑路径
- **M09 (本章新增): GPU/SIMD 加速模块**
  - (如有 GPU) cuRobo MotionGen 全栈运动生成
  - (无 GPU) OMPL/MoveIt + 显式 VAMP SIMD 碰撞桥接（模型适配时）
  - 性能基准对比

**下一步 (M10)**: 将 M07-M09 输出的几何路径进行**时间参数化**——赋予时间戳，满足速度/加速度/jerk 限制，生成可直接发送给 ros2_control 的可执行轨迹。

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Sundaralingam et al., "cuRobo: Parallelized Collision-Free Robot Motion Generation", ICRA 2023 | ⭐⭐⭐ | GPU 并行运动生成 |
| Thomason et al., "Motions in Microseconds via Vectorized Sampling-Based Planning", ICRA 2024 | ⭐⭐⭐ | SIMD 加速采样规划 |
| NVIDIA CUDA Programming Guide Ch3-5 | ⭐⭐ | CUDA 编程基础 |
| Intel Intrinsics Guide (software.intel.com) | ⭐⭐ | AVX2 指令参考 |
| cuRobo GitHub NVlabs/curobo | ⭐⭐ | 源码和文档 |
| VAMP GitHub KavrakiLab/vamp | ⭐⭐ | 源码 (Apache-2.0, C++20) |
| Isaac ROS cuMotion 文档 | ⭐⭐ | MoveIt2 集成指南 |
| Pan et al., "FCL", ICRA 2012 | ⭐⭐ | 传统碰撞检测参考 (M04 复习) |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| cuRobo warmup 卡住/崩溃 | CUDA/PyTorch 版本不兼容 | 1. 检查 CUDA/PyTorch 版本对应 2. 设置 `CUROBO_USE_LRU_CACHE=1` 3. 查看 `/tmp/curobo_*` 编译日志 | — |
| cuRobo 规划失败 (status=FAIL) | 球体模型不覆盖 link / 目标不可达 | 1. 可视化球体模型确认覆盖 2. 单独检查 IK 可行性 (M03) 3. 增加 `num_trajopt_seeds` | M03, M04 |
| cuRobo 规划时间 > 100ms | GPU 过旧 / CUDA Graph 未启用 | 1. 确认 `use_cuda_graph=True` 2. 检查 GPU 利用率 (`nvidia-smi`) 3. 减少 `num_trajopt_seeds` | — |
| VAMP 编译失败 | CPU 不支持 AVX2 / C++20 编译器缺失 | 1. 检查 `lscpu \| grep avx2` 2. ARM 平台检查 NEON 3. 确认 GCC 10+ 或 Clang 14+ | — |
| VAMP 性能不及论文 | 未开启编译优化 / CPU 降频 | 1. 确认 `-O3 -march=native` 编译 2. 检查 CPU governor (不是省电模式) 3. 排除后台进程干扰 4. 检查 turbo boost 是否启用 | — |
| cuMotion MoveIt2 插件不加载 | Isaac ROS 版本不匹配 | 1. 检查 ROS2 版本和 Isaac ROS 版本对应 2. 确认 cumotion 包已安装 3. 查看 pluginlib 日志 | M14 |
| nvblox ESDF 生成太慢 | 体素分辨率太高 / GPU 内存不足 | 1. 增大体素尺寸 (2cm→5cm) 2. 缩小 ESDF 计算范围 3. 检查 GPU 内存 | M04 |
| cuRobo 路径质量差 (锯齿) | 轨迹优化种子数不足 | 1. 增加 `num_trajopt_seeds` (12→24) 2. 启用 `enable_graph=True` 3. 增加 `max_attempts` | M08 |
| cuRobo 与 MoveIt2 轨迹不兼容 | 时间参数化格式不匹配 | 1. 检查 `JointTrajectory` 消息的时间戳 2. 使用 cuMotion 插件的标准输出格式 3. 检查关节顺序一致性 | M10, M14 |
| VAMP 在 ARM 平台性能不及预期 | NEON 向量宽度仅 128-bit (vs AVX2 256-bit) | 1. ARM 上吞吐率约为 x86 的一半 (预期行为) 2. 检查是否使用了 NEON 编译路径 3. 考虑 SVE2 支持 (如 Graviton3+) | — |

---

**三章总结过渡**: M07-M08-M09 构成了完整的"运动规划"模块：

- **M07（采样规划）**: 在构型空间中找到**拓扑正确的可行路径**——解决"能不能走"的问题
- **M08（轨迹优化）**: 将锯齿路径优化为**平滑无碰撞的高质量路径**——解决"走得好不好"的问题
- **M09（GPU加速）**: 将全栈运动生成压缩到**30ms 以内**——解决"来不来得及"的问题

三者是递进关系：采样找可行解 → 优化提升质量 → 加速满足实时性。在 MoveIt2 的工程实践中，它们通过链式管线（OMPL → STOMP）或 GPU 全栈替代（cuRobo）组合使用。下一步（M10）将为优化后的路径赋予时间参数，使其成为可直接发送给 ros2_control 硬件接口的可执行轨迹。

**M07-M08-M09 技术栈选型速查**:

```
需求诊断:
  Q1: 有 GPU 且需要实时? ──→ cuRobo (30ms 全栈)
  Q2: 无 GPU 但需要极速? ──→ VAMP + STOMP (50ms)
  Q3: MoveIt2 生态内?     ──→ OMPL + STOMP 链式管线
  Q4: 安全关键工业场景?   ──→ OMPL + TrajOpt (Tesseract)
  Q5: 接触丰富操作?       ──→ Drake (联合优化)
  Q6: 增量/动态环境?      ──→ GPMP2 (iSAM2 增量)
```
