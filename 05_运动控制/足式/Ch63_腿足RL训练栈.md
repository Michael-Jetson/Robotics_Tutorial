> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 63 章 腿足 RL 训练栈——IsaacGym/Lab + rsl_rl + 奖励设计 + Domain Randomization + Teacher-Student

> **难度**: ⭐⭐ ~ ⭐⭐⭐⭐ | **预计学时**: 30-40 小时(1.5 周) | **text:code = 7:3**（理论+实践混合章节）
>
> **一句话概要**: 腿足 RL 不是"换个环境跑 PPO"——从 GPU 并行仿真到奖励工程、从 Domain Randomization 的贝叶斯解释到 Teacher-Student 的信息论本质,本章系统构建腿足 RL 训练栈的完整知识体系。

**向前承接**: Ch61-62 搭好了实时 C++ 和硬件栈的完整链路。Ch55 展示了 OCS2 MPC 的工业级方案。本章转入控制的另一条路线——用强化学习端到端地学习运动策略,而 MPC 的概念(如步态、接触约束)将作为奖励设计的参照基准。

**向后指向**: Ch64 将把本章训好的策略部署到 C++/ros2_control 实时控制循环。Ch65 则探索 RL 与 MPC 混合的前沿范式——两条路线的融合。

---

## 前置自测

📋 **答不出 $\geq$ 2 题 → 先回对应章节复习**

1. **[RL 基础]** PPO 的 clipped objective 要解决什么问题?如果不 clip,直接用 TRPO 的 KL 约束,有什么工程缺陷?
2. **[RL 基础]** GAE($\lambda$) 中的 $\lambda$ 参数控制什么权衡?$\lambda=0$ 和 $\lambda=1$ 分别退化为什么?
3. **[Ch61]** 为什么实时控制循环内不能调用 `malloc`?在 1 kHz 频率下,动态内存分配会导致什么后果?
4. **[Ch55]** OCS2 的 MPC 求解频率一般是多少?为什么不直接把 MPC 跑到 1 kHz?
5. **[Ch62]** 四足机器人的关节电机延迟大约在什么量级?这对 RL 策略的 sim-to-real 有何影响?

---

## 本章目标

学完本章,你应能:

1. 解释腿足 RL 与通用 RL 的关键差异,理解为什么需要专用的训练基础设施
2. 在 IsaacLab 中配置并训练一个四足 trot 策略,理解 GPU 并行仿真的架构
3. 精读 rsl_rl 的 PPO 实现——包括 GAE 完整推导和 clipped objective 的数学意义
4. 系统地设计奖励函数,理解每个奖励项的物理动机和数学形式
5. 从贝叶斯视角解释 Domain Randomization 为什么能提高鲁棒性
6. 从信息论角度理解 Teacher-Student 蒸馏的本质
7. 独立训练一个 Go2 trot 策略并在仿真中验证

---

## 63.1 腿足 RL 与通用 RL 的关键差异 ⭐

### 动机:为什么不能直接用 Atari/MuJoCo Gym 的方法?

如果你在 OpenAI Gym 的 `HalfCheetah-v4` 上训过 PPO,可能觉得 RL 训练就是"写好环境,跑 PPO,调调超参"。但当你把同样的方法搬到真实四足机器人上,几乎一定会失败。这不是算法不行——PPO 在腿足场景下仍然是最常用的算法——而是腿足 RL 面临的工程挑战与通用 RL 有本质不同。

理解这些差异,是选择正确工具链的前提。

### 如果不理解差异会怎样

一个常见的失败路径:研究者在 MuJoCo 的 `Humanoid-v4` 上训出了漂亮的行走策略,然后直接迁移到真机。结果:

- MuJoCo 的接触模型过于理想化,真机接触力完全不同
- 单环境训练太慢,训 48 小时还没收敛
- 训好的策略在真机上第一步就摔倒——sim-to-real gap 太大
- 没有 Domain Randomization,策略对摩擦系数极其敏感

这些失败促使了腿足 RL 专用训练栈的诞生。

### 差异的系统分析

| 维度 | 通用 RL (Atari/MuJoCo Gym) | 腿足 RL |
|------|---------------------------|---------|
| **环境复杂度** | 规则固定(游戏/简化物理) | 地形、摩擦、电机动力学都是变量 |
| **观测维度** | 几十到几百 | 50-200(本体感知),加视觉可达数万 |
| **动作维度** | 几个到几十个 | 12-30 关节(四足12,人形20+) |
| **训练并行度** | 单环境或少量并行 | **4096+ 环境 GPU 并行**——必须 |
| **训练时长** | 百万步/小时(小任务) | 数千万到数亿步,24-72 小时 GPU |
| **最大挑战** | 探索效率、奖励稀疏 | **sim-to-real gap**——仿真训好 $\neq$ 真机能用 |
| **物理引擎** | MuJoCo / PyBullet (CPU) | IsaacGym / IsaacLab (GPU 原生) |
| **RL 库** | SB3 / RLlib (通用) | rsl_rl (轻量,腿足专用) |

这些差异导致了三个核心技术选择:

1. **GPU 原生仿真器**替代 CPU 仿真器——10-100 倍加速
2. **轻量 RL 库**替代通用库——减少数据管道开销
3. **Domain Randomization + Curriculum + Teacher-Student**——系统对抗 sim-to-real gap

### 历史脉络:腿足 RL 的三个时代

腿足 RL 并非一蹴而就。理解历史演进有助于把握当前技术栈为什么长这个样子:

**第一时代:CPU 仿真 + 通用 RL (2017-2019)**。早期工作如 Hwangbo et al. (2019, Science Robotics) 在 RaiSim(CPU 仿真器)上训练 ANYmal。训练一个策略需要数天到数周。这个阶段证明了"仿真训练,真机部署"的可行性,但训练效率是瓶颈。

**第二时代:GPU 并行仿真爆发 (2020-2023)**。IsaacGym (2021) 让训练时间从天降到小时。Rudin et al. (2022, CoRL) 的 "Learning to Walk in Minutes" 展示了 4096 并行环境下 20 分钟训好 ANYmal 行走。legged_gym + rsl_rl 成为事实标准。

**第三时代:IsaacLab + 多模态 (2023-至今)**。IsaacLab 逐步取代 IsaacGym,支持更丰富的传感器(深度相机、LiDAR)、更完整的场景资产和更活跃的维护生态。2026 年的 Isaac Lab 3.0 beta 开始引入 Newton backend 等新能力,但 API 仍需按官方版本锁定。训练范式从纯本体感知扩展到视觉+本体感知,rsl_rl 也开始支持更大规模的多 GPU 训练。

```
时间线:
2018: Tan et al. (RSS) — 首次四足 sim-to-real, 域随机化+执行器建模
2019: Hwangbo (Science Robotics) — ANYmal RL, 神经网络执行器建模, CPU 仿真
2019: Haarnoja et al. (RSS) — 真机直接训练 SAC, 自动温度调节
2020: Miki前身 (Science Robotics) — Teacher-Student + 地形课程, ANYmal
2021: IsaacGym 发布 — GPU 并行仿真, 小时级训练
2021: Kumar RMA (RSS) — 在线自适应, A1 真机
2022: Miki (Science Robotics) — Teacher-Student + 感知, ANYmal Alps 徒步
2022: Rudin (CoRL) — 20 分钟训练, legged_gym 开源
2022: Margolis Walk These Ways (CoRL) — 行为多样性, Go1
2023: IsaacLab 发布 — 取代 IsaacGym
2024: Hoeller (Science Robotics) — ANYmal Parkour
2026: IsaacLab 3.0 beta — Newton backend, 多 GPU, API 快速演进
```

值得展开的是 2018-2019 年的两项奠基性工作,它们确立了至今仍在沿用的 sim-to-real 范式:

**Tan et al. (RSS 2018)** 是首个在四足机器人上实现零样本 sim-to-real RL 迁移的工作。其核心公式——"精确执行器建模 + 域随机化 = 成功迁移"——至今仍是黄金法则。该工作的关键工程决策包括:(1) 使用直流电机物理方程(包含反电动势和磁饱和)替代物理引擎的理想化电机模型;(2) 通过维护观测历史记录并线性插值来模拟真实系统延迟;(3) 将控制器解耦为"开环参考信号 + 策略残差"的形式——即使策略输出为零,机器人也能按照参考轨迹执行基本步态,策略只需学习如何微调。这种"基准步态先验 + 残差调制"的范式大幅降低了学习难度。

**Hwangbo et al. (Science Robotics 2019)** 将执行器建模推进了一步:对于串联弹性执行器(SEA)这类复杂传动机构,解析建模的难度极高(涉及近百个参数),因此直接用监督学习训练一个"动作到力矩"的神经网络模型。训练数据的采集非常简单——用正弦波足端轨迹驱动机器人,配合人工干扰(推动和阻碍)来丰富数据集。该工作还揭示了一个违反直觉的发现:RL 策略学会了在运动学奇异点(膝盖完全伸直)附近操作以节省能量——传统方法必须刻意回避这些奇异点,而 RL 策略不直接依赖逆运动学矩阵求逆,因此规避了传统解析控制中的奇异点求解问题。不过这不等于奇异构型在物理上"无影响":力矩裕度、速度限幅、接触冲击和电机带宽仍然会约束策略可用的动作。

> 💡 **执行器建模：sim-to-real 的隐形杀手**
>
> 真实电机的力矩输出远非理想。Tan et al. (2018) 首次系统建模了以下非理想特性,其解析执行器模型由以下几个关键方程组成:
>
> 1. **力矩-电流关系**: $\tau = K_t I$,电机输出的机械力矩与流过线圈的电流成正比
> 2. **电压平衡方程**: $I = \frac{V_{pwm} - V_{emf}}{R}$,流过电机的电流并不直接等于电压除以电阻,施加在电机两端的有效电压必须先克服反电动势
> 3. **反电动势(Back-EMF)**: $V_{emf} = K_e \dot{q}$,当电机转动时产生一个与转速成正比、方向与输入电压相反的电压。这解释了为什么高速运动时力矩会衰减——随着转速 $\dot{q}$ 增加,$V_{emf}$ 增大,有效电压差 $(V_{pwm} - V_{emf})$ 减小,电流下降,最终**高速时实际力矩显著低于命令力矩**
> 4. **非线性饱和(Saturation)**: 真实电机的铁芯存在磁饱和现象,力矩不会随电流无限线性增长,需要引入分段线性函数截断过大的力矩
> 5. **PD 伺服控制**: 先由
>    $$u_{pwm}=k_p(\bar{q}-q_n)+k_d(\bar{\dot{q}}-\dot{q}_n)$$
>    生成归一化 PWM / duty-cycle 命令,再计算 $V_{pwm}=V_{battery}\cdot \text{clip}(u_{pwm},-1,1)$。这里的 $k_p,k_d$ 是底层伺服器中的无量纲缩放系数,不是关节力矩控制里的物理 PD 增益($\mathrm{Nm/rad}$、$\mathrm{Nms/rad}$);目标速度通常设为 $\bar{\dot{q}}=0$。
>
> Hwangbo et al. (2019) 则提出用**神经网络直接学习执行器模型**（Actuator Net）,避免手工建模的复杂性:
> - **数据采集**：用简单控制器生成正弦波足端轨迹,通过逆运动学计算关节位置指令,改变振幅和频率扩大覆盖范围,并通过手动推动/阻碍机器人进一步丰富数据集。由于电机的高频特性,每秒可为单个电机产生数百个样本
> - **网络输入**：当前及过去两个时间步($t$, $t-0.01$s, $t-0.02$s)的关节位置误差和关节速度历史窗口——因为执行器内部状态(如控制器状态、电机转速)无法直接测量,需要用历史信息来估计
> - **训练**：监督学习,预测实际输出力矩。最终误差接近传感器分辨率,远优于理想执行器模型
> - **部署**：在仿真中替换理想力矩模型,让 RL 策略"感受到"真实电机的非理想特性

### ⚠️ 常见陷阱

> ⚠️ **概念误区:认为"RL 算法创新"是腿足 RL 的核心**
> **新手想法**: "PPO 太旧了,应该用 SAC / DreamerV3 / TD-MPC 来训腿足。"
> **实际情况**: 腿足 RL 的核心挑战不在算法,而在**工程**——奖励设计、Domain Randomization、仿真精度、部署流水线。PPO 在腿足场景下已经工作得非常好,绝大多数 SOTA 工作(ANYmal Parkour、Walk These Ways、RMA)都使用 PPO。换算法的边际收益远小于优化训练管道。
> **正确认知**: 先把 PPO + legged_gym 跑通,再考虑算法创新。

> ⚠️ **思维陷阱:低估 sim-to-real gap 的严重性**
> **新手想法**: "仿真里跑得很好,应该直接能上真机。"
> **现象**: 仿真里完美的策略,真机第一步就摔倒。
> **根本原因**: 仿真器的接触模型、电机模型、延迟模型都与真实世界有系统偏差。没有 Domain Randomization 的策略会 overfit 到仿真器的特定参数。
> **正确做法**: sim-to-real gap 是腿足 RL 的**主要工程挑战**,必须用 DR + Teacher-Student + 真机微调来系统解决。

> ⚠️ **工程警示：真机直接训练的风险与缓解**
>
> 直接在真实机器人上进行 RL 探索（而非 sim-to-real）可以避免仿真-现实差距，但面临严重的安全和成本问题：
>
> | 风险 | 后果 | 缓解机制 |
> |------|------|---------|
> | 随机探索初期碰撞/摔倒 | 电机过流、齿轮损坏、结构变形 | 安全感知 RL（约束探索空间） |
> | 关节力矩/速度超限 | 硬件不可逆损伤 | 硬件层力矩限幅 + 软件安全监控 |
> | 探索效率极低 | 训练时间数天至数周 | 仿真预训练 + 真机微调（fine-tune） |
> | 失败后无法自动恢复 | 人工干预成本高 | 跌倒检测 + 自主恢复策略 |
>
> **实践建议**：除非研究目标本身就是真机在线学习（如 Dreamer/MBPO 等 model-based 方法），否则推荐 **sim-to-real 路线**（仿真中训练 → Domain Randomization → 零样本迁移），真机仅做验证和少量微调。

#### 真机在线训练的系统工程（以 SAC 为例）

Haarnoja et al. (RSS 2019) 在 Minitaur 四足上实现了真机直接训练 SAC，其系统设计远不只是"换个算法"：

**异步采集-训练架构**：
```
采集进程 (100 Hz)              训练进程 (异步)
  └─ 执行策略 → 收集 (s,a,r,s')    └─ 从 Replay Buffer 采样
  └─ 写入 Replay Buffer             └─ 更新 Q/π 网络
  └─ 定期同步最新策略参数            └─ 推送更新后的策略
```
采集和训练解耦：采集进程以固定频率与真机交互，训练进程在后台异步更新。两者通过共享 Replay Buffer 和定期参数同步通信。

**安全机制**：
| 机制 | 实现方式 | 作用 |
|------|---------|------|
| 力矩硬限幅 | 底层驱动器层面截断 | 防止电机过载（不可绕过） |
| 动作空间裁剪 | 策略输出 tanh + scale | 限制关节目标位置范围 |
| 姿态监控 | IMU roll/pitch 超阈值触发 | 检测即将摔倒 |
| 自动重置 | 检测到摔倒后执行恢复动作序列 | 减少人工干预 |
| 早期终止 | 关节速度/力矩异常时中断 episode | 保护硬件 |

**自动重置的工程挑战**：
真机无法像仿真一样 `env.reset()`。Haarnoja 的方案是检测到不可恢复状态后，切换到预编程的恢复控制器（如 PD 回到站立姿态），成功站立后再切回 RL 策略继续探索。这要求恢复控制器本身足够鲁棒。

> ⚠️ **关键限制**：真机训练的样本效率远低于仿真（无并行、每步真实时间）。Haarnoja et al. (RSS 2019) 报告 Minitaur 上约 2 小时、约 400 个 rollout、约 160k control steps 才获得基本行走策略。这个样本量在 GPU 并行仿真中只是很小的一段 rollout;真机训练的价值不在效率,而在**消除 sim-to-real gap**。

> ⚠️ **编程陷阱:在 CPU 仿真器上训腿足策略**
> **错误做法**: 用 MuJoCo(CPU)训四足行走,单环境串行。
> **后果**: 训练一个策略需要 3-7 天,迭代一次奖励设计就浪费一周。
> **根本原因**: 腿足 RL 需要数亿步交互,CPU 串行速度远远不够。GPU 并行仿真(4096 环境)将这个过程压缩到小时级。
> **正确做法**: 使用 IsaacLab + rsl_rl,一天内完成多轮奖励迭代。

### 练习

**练习 63.1.1**: 列出你知道的 RL 环境(Atari、MuJoCo、DMControl 等),与腿足环境对比,哪些维度的差异最大?为什么腿足需要特殊的训练基础设施?

**练习 63.1.2**: 如果你有一个已经在 MuJoCo 的 `Ant-v4` 上训好的 PPO 策略,列出 3 个它不能直接部署到 Unitree Go2 真机的原因。对于每个原因,说明腿足 RL 训练栈是如何解决的。

---

上节分析了腿足 RL 与通用 RL 的差异,引出了对 GPU 原生仿真器的需求。下面我们深入 IsaacGym 到 IsaacLab 的演进,理解当前训练基础设施的架构。

## 63.2 IsaacGym → IsaacLab:GPU 并行仿真的演进 ⭐⭐

### 动机:为什么需要 GPU 原生仿真?

一个直觉的问题:为什么不直接在 MuJoCo 上并行跑多个 CPU 进程?

答案在于**数据传输开销**。传统方案中,物理仿真在 CPU 上运行,RL 算法(PyTorch)在 GPU 上运行。每一步都需要:

```
CPU 仿真 → 观测数据拷贝到 GPU → GPU 前向推理 → 动作拷贝回 CPU → CPU 下一步仿真
```

这个 CPU-GPU 数据拷贝在 4096 并行环境下成为严重瓶颈。IsaacGym 的革命性创新是:**把物理仿真也放到 GPU 上**,消除所有 CPU-GPU 数据拷贝:

```
GPU 仿真 → GPU 上的观测 tensor → GPU 前向推理 → GPU 上的动作 tensor → GPU 下一步仿真
           (全程不经过 CPU,不经过 PCIe 总线)
```

### 如果不用 GPU 仿真会怎样

以训练一个 Go2 trot 策略为例,比较训练速度:

| 方案 | 并行环境数 | 每秒仿真步 | 训练 1 亿步耗时 |
|------|-----------|-----------|--------------|
| MuJoCo (单 CPU) | 1 | ~5,000 | ~5.6 小时 |
| MuJoCo (32 CPU 进程) | 32 | ~100,000 | ~16.7 分钟(理论值) |
| IsaacGym (RTX 4090) | 4096 | ~4,000,000 | ~25 秒 |
| IsaacLab (RTX 4090) | 4096 | ~3,500,000 | ~29 秒 |

差距是**两个数量级**。这意味着在 GPU 仿真下,你可以在一个下午完成 10 轮奖励设计迭代;而在 CPU 仿真下,同样的迭代需要两周。

> **口径说明**：上表为**裸仿真吞吐量**（纯物理步进 + 数据收集，不含策略网络前向/反向传播和梯度更新）。完整 PPO 训练的 wall-clock 时间通常比裸仿真慢 10-100 倍（参见下文 IsaacLab 性能数据）。

### IsaacGym:开创者(2021-2023,已 legacy)

IsaacGym 由 NVIDIA 在 2021 年发布,是第一个广泛使用的 GPU 原生物理引擎:

- **底层**: NVIDIA PhysX 5.x(GPU 加速的刚体/关节/接触仿真)
- **接口**: Python API(通过 CUDA 与底层交互)
- **并行能力**: 单 GPU 上 4096+ 个机器人并行
- **地位**: legged_gym + rsl_rl 的原始仿真后端

IsaacGym 有显著的历史局限:

| 局限 | 影响 |
|------|------|
| 渲染能力弱 | 不支持高质量视觉传感器(深度相机、LiDAR) |
| 传感器模型有限 | 难以训练视觉 RL 策略 |
| 独立运行 | 不与 NVIDIA 的完整仿真生态集成 |
| 停止维护 | 2024 年起不再更新,用户被建议迁移到 IsaacLab |

### IsaacLab:当前主流(2023-至今)

IsaacLab(原名 Orbit,后被 NVIDIA 官方吸收)是 IsaacGym 的继任者。截至撰写时,Isaac Lab 已迭代至 3.x 版本,配合 Isaac Sim 使用(请以 NVIDIA 官方文档确认当前最新版本)（注：截至撰写时 Isaac Lab 仍处于快速迭代阶段，API 可能在版本间有较大变化）:

| 特性 | IsaacGym | IsaacLab / Isaac Lab 3.x |
|------|----------|-------------|
| **物理后端** | PhysX 5.x | PhysX 5.x;3.0 beta 提供 Newton backend |
| **渲染** | 基础 | 光线追踪(RTX),支持深度相机、LiDAR |
| **传感器** | IMU、接触力 | IMU、接触力、深度相机、LiDAR、RGB |
| **机器人模型** | URDF | URDF + USD(更丰富的资产描述) |
| **RL 库集成** | rsl_rl(手动配置) | rsl_rl + rl_games(原生支持) |
| **多 GPU** | 不支持 | 新版本支持多 GPU 训练,具体能力需按版本核实 |
| **维护状态** | 已停止 | 活跃维护,NVIDIA 官方项目 |
| **社区** | 大量历史代码 | 快速增长,已成新标准 |

### IsaacLab 的架构

IsaacLab 采用**管理器驱动(Manager-Based)**的环境设计。每个环境由多个 Manager 组成,各自负责不同职责。这种设计的好处是**模块化**:修改奖励不需要改环境代码,只需要替换 `RewardManager` 的配置。

```python
# IsaacLab 的 ManagerBasedRLEnv 架构概览
class Go2FlatEnv(ManagerBasedRLEnv):
    """Go2 平地行走环境"""

    # 各 Manager 负责不同职责:
    # ObservationManager  → 构建观测(哪些信号给 Actor,哪些给 Critic)
    # ActionManager       → 定义动作空间(关节位置/速度/扭矩)
    # RewardManager       → 计算奖励(多个奖励项加权求和)
    # TerminationManager  → 判断 episode 是否结束
    # CurriculumManager   → 管理课程学习(地形难度等)
    # CommandManager       → 生成速度命令(vx, vy, wz)
    # EventManager        → Domain Randomization(质量、摩擦等随机化)
```

以下是一个完整的环境配置示例:

```python
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import (
    ObservationGroupCfg, ObservationTermCfg,
    RewardTermCfg, TerminationTermCfg,
)

@configclass
class Go2FlatEnvCfg(ManagerBasedRLEnvCfg):
    """Go2 平地行走——完整配置"""

    # 场景:4096 个 Go2 并行
    scene = Go2SceneCfg(num_envs=4096, env_spacing=2.5)

    # 观测:分为 policy 组(给 Actor)和 critic 组(给 Critic)
    observations = ObservationsCfg(
        policy=ObservationGroupCfg(
            terms={
                "base_ang_vel": ObservationTermCfg(func=base_ang_vel),      # (3,)
                "projected_gravity": ObservationTermCfg(func=proj_gravity),  # (3,)
                "velocity_commands": ObservationTermCfg(func=vel_commands),  # (3,)
                "joint_pos": ObservationTermCfg(func=joint_pos_rel),        # (12,)
                "joint_vel": ObservationTermCfg(func=joint_vel_rel),        # (12,)
                "last_action": ObservationTermCfg(func=last_action),        # (12,)
            }
            # 总观测维度: 3+3+3+12+12+12 = 45
        )
    )
```

#### 为什么需要历史观测：POMDP 视角

腿足机器人的控制问题本质上是一个**部分可观测马尔可夫决策过程（POMDP）**，而非标准 MDP。原因很直接：

- **状态 ≠ 观测**：系统的完整状态包含基座速度、地面摩擦系数、负载质量、地形形状等——但真机的传感器只能提供关节编码器和 IMU 数据（部分观测）
- 在 POMDP 中，最优策略不是 $\pi(a|o_t)$（基于当前观测），而是 $\pi(a|b_t)$（基于**信念状态** belief state）
- 信念状态 $b_t$ 是对完整状态的后验概率分布，理论上需要贝叶斯滤波器维护——但在高维连续空间中不可行

**工程上的近似**：理论上,策略需要基于 belief state 做决策;工程上通常用**观测历史** $(o_{t-H}, a_{t-H}, \ldots, o_{t-1}, a_{t-1}, o_t)$、RNN 隐状态或适应模块 latent 来近似这个后验分布。这不是单纯"提高性能的技巧"——单帧观测在 POMDP 中通常信息不足,必须用某种记忆机制补上被遮蔽的状态信息。

这就是为什么腿足 RL 的各种方法都需要历史信息：
| 方法 | 历史编码方式 | 推断目标 |
|------|------------|---------|
| 直接堆叠 | MLP 输入拼接 $[o_{t-k}, \ldots, o_t]$ | 隐式推断速度/接触 |
| RMA Adaptation Module | MLP/1D-CNN 处理 50 步历史 | 显式推断环境 latent $\hat{z}_t$ |
| Miki2022 TCN | 时序卷积网络 | 推断接触/打滑/地形隐变量 |
| RNN/LSTM/GRU | 循环网络维护隐状态 | 维护 belief state 近似 |

理解这一点后，Teacher-Student 框架的设计就很自然了：Teacher 直接访问完整的特权状态,Student 必须从历史观测中重建一个可用于控制的近似 latent。这个 latent 可以被理解为 belief state 的工程替代品,但 Teacher 并不真的维护贝叶斯后验——它只是读取了仿真中真机不可直接获得的信息。

#### 观测空间的三分法

腿足 RL 的观测输入可系统性地分为三类：

| 类别 | 内容 | 来源 | 真机可用性 |
|------|------|------|-----------|
| **本体感知** (Proprioception) | 关节角度/角速度/力矩、IMU（姿态/角速度/加速度）、基座角速度 | 电机编码器、IMU | ✓ 直接可用 |
| **外感知** (Exteroception) | 地形高程图、深度图、点云、接触力 | 深度相机、LiDAR、力传感器 | ✓ 但有噪声/延迟 |
| **任务输入** (Task Command) | 期望速度 ($v_x, v_y, \omega_z$)、步态选择、目标位置 | 操作员/高层规划器 | ✓ |

外感知不是"把地形真值送给策略"。真实机器人通常先从深度相机或 LiDAR 得到点云,再融合成高程图或局部 traversability map。这个管线有两个局限:(1) 高程图是 2.5D 表示,对草丛、灌木、泥浆、雪地这类非刚性或语义复杂地形容易误判;(2) 感知更新有 25-50 ms 甚至更高延迟,高速运动时旧地图会把落脚点推向错误位置。因此感知型 RL 不能只依赖"更大的 CNN"解决问题,还要在训练中注入感知噪声、延迟、遮挡和错误地形标签,并在 Ch66/Ch67 的高程图与 Perceptive MPC 中继续处理这些结构性误差。

> 💡 **工程要点**：
> - 仿真调试时可用 **cheater state**（地面真值）快速验证控制逻辑。但训练 Actor 时应区分：Actor 输入只能包含真机可获取的观测；Critic/Teacher 可以使用特权真值作为辅助信号。部署时必须确认所有 Actor 输入均来自状态估计器，而非仿真真值
> - **特权信息**（摩擦系数、质量、地形参数）只在 Teacher 网络中使用。Student 网络只能依赖**真机可获取的观测**——可以是本体感知，也可以包括有噪声/延迟的外感知（如深度相机），但不能依赖仿真特权真值
> - 观测历史可隐式编码环境动态特性：短历史（3-5 步）用于缓解部分可观测性；RMA 等适应模块通常需要更长窗口（如 50 步）以辨识环境参数

```python
    # 动作:关节位置偏移量
    actions = ActionsCfg(
        joint_pos=JointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*"],      # 所有 12 个关节
            scale=0.25,              # action 缩放因子
            use_default_offset=True  # 以默认站立姿态为偏移基准
        )
    )
```

#### 动作空间设计选择

| 设计方式 | 输出内容 | 优点 | 缺点 | 代表工作 |
|---------|---------|------|------|---------|
| **关节位置 PD** | 目标关节角度 $q_d$，底层 PD 跟踪 | 最常用、稳定、易调 | PD 增益是先验假设 | rsl_rl, Walk These Ways |
| **关节力矩直接** | 关节力矩 $\tau$ | 最大控制自由度 | 训练困难、真机危险 | 早期工作 |
| **结构化动作** | 足端轨迹参数（步高/步长/频率） | 物理先验约束动作空间 | 灵活性受限 | ETH ANYmal |
| **残差 RL + CPG** | CPG 基础步态 + 学习残差 $\delta q$ | 安全初始行为、稳定探索 | CPG 设计引入偏置 | 多篇2023-2024工作 |

> 💡 **工程建议**：新手推荐从**关节位置 PD** 起步（rsl_rl 默认方式）。PD 模式提供了天然的"安全网"——策略输出的是目标位置偏移量而非直接力矩，实际安全由 action clip（限制输出范围）、action scale（缩放幅度）、关节限位和底层力矩限幅共同保障。相比直接力矩输出，PD 模式的输出空间更紧凑、探索更温和。直接输出力矩需要更精细的奖励设计和安全机制。

**性能数据**: RTX 4090 上,4096 个 Go2 并行训练,完整 PPO wall-clock 约 **1 亿步 2.5 小时**（含网络前向/反向传播、梯度更新、日志记录等开销，远慢于上表的裸仿真吞吐）,24 小时可达 **10 亿步**。

### legged_gym 的维护现状

需要特别说明的是:legged_gym 仓库(`leggedrobotics/legged_gym`)基于 IsaacGym,目前已声明**仅提供有限更新和支持**,ETH RSL 团队鼓励用户迁移到 IsaacLab。但 legged_gym 的代码结构和设计思想仍然是理解腿足 RL 工程的最佳教材——大量论文和项目依然基于它。新项目推荐直接使用 IsaacLab 并锁定具体版本;若采用 Isaac Lab 3.0 beta 的 Newton backend 等能力,需要预留 API 迁移成本。也可以使用 fan-ziqi 的 `robot_lab`(legged_gym 概念到 IsaacLab 的迁移封装)作为过渡。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:混淆 IsaacGym 和 IsaacLab 的 API**
> **错误做法**: 在 2025/2026 年的新项目中使用 `from isaacgym import gymapi`。
> **后果**: IsaacGym 已停止更新,新特性(Newton 物理引擎、多 GPU、深度相机)都不可用。且社区支持逐渐消失。
> **根本原因**: legged_gym 基于 IsaacGym,但 ETH RSL 已将新工作迁移到 IsaacLab。legged_gym 仓库本身声明"limited updates and support"。
> **正确做法**: 新项目优先使用 IsaacLab,并在项目文档中锁定具体版本。若使用 Isaac Lab 3.0 beta/预览版特性(如 Newton backend),需要明确 API 可能变化;老代码迁移参考 NVIDIA 官方迁移指南。

> ⚠️ **概念误区:认为 GPU 仿真 = 更准确的物理**
> **新手想法**: "GPU 算力更强,所以物理仿真更精确。"
> **实际情况**: GPU 仿真的优势是**并行度**,不是精度。PhysX 的接触模型(penalty-based)本身就有局限——接触穿透、摩擦力近似。Isaac Lab 3.0 beta 引入的 Newton backend 在接触求解方面提供了新选择,但仍然不是"真实物理"本身。
> **关键认知**: 仿真精度靠 Domain Randomization 来补偿,而不是靠仿真器本身完美。

> ⚠️ **思维陷阱:等 IsaacLab 完全稳定再开始学**
> **新手想法**: "IsaacLab API 还在变,等稳定了再学。"
> **实际情况**: IsaacLab 3.x 仍在快速演进,3.0 发布阶段也带有 beta/预览性质,API 可能随版本调整。但腿足 RL 的**核心概念**(并行仿真、奖励设计、DR、Teacher-Student)不依赖于特定 API——这些概念在任何仿真器上都适用。
> **正确做法**: 先学概念,再学 API。概念不变,API 可以适配。

### 练习

**练习 63.2.1**: 计算在 RTX 4090(4096 并行环境)上,如果每个环境每步仿真耗时 0.25 ms、PPO 前向推理耗时 0.1 ms,理论上每秒能仿真多少步?与 MuJoCo 单 CPU 环境(每步 0.2 ms)对比。

**练习 63.2.2**: IsaacLab 的 `ManagerBasedRLEnv` 与传统 Gym 的 `step(action) -> obs, reward, done, info` 接口有什么区别?这种 Manager 驱动的设计对奖励工程有什么好处?

**练习 63.2.3**: 阅读 IsaacLab 官方文档中 Go2 velocity tracking 的环境配置,列出所有观测项及其维度。为什么不把机器人的全局位置(x, y, z)作为观测?

---

GPU 并行仿真提供了训练效率,但"用什么 RL 算法来训"同样关键。下一节深入 rsl_rl 的 PPO 实现,完整推导 GAE 和 clipped objective。

## 63.3 rsl_rl 的 PPO 实现详解 ⭐⭐

### 动机:为什么不用 Stable-Baselines3?

这是一个合理的问题。Stable-Baselines3 (SB3) 是最流行的 RL 库,支持 PPO/SAC/DQN 等多种算法,文档完善,社区庞大。为什么腿足 RL 社区放弃 SB3,自己写了 rsl_rl?

答案在于**开销**。SB3 为通用性付出的代价:

| 特性 | SB3 | rsl_rl |
|------|-----|--------|
| **代码量** | ~30,000 行 | ~2,000 行 |
| **支持算法** | PPO, SAC, DQN, A2C, TD3, HER 等 | 仅 PPO(+ Student-Teacher 蒸馏) |
| **数据流** | CPU/GPU 切换、Gym wrapper 层层包装 | 纯 GPU tensor 操作,无包装 |
| **rollout 存储** | 通用 Buffer(支持各种算法) | 精简 Buffer(仅 PPO 需要的字段) |
| **网络定义** | 通用 `policies.py`(处理各种空间) | 专用 `ActorCritic`(MLP / MLP+RNN） |
| **每步开销** | ~2 ms(包含各种检查和转换) | ~0.1 ms |

在 4096 环境 x 24 步 = 98,304 步/iteration 的规模下,SB3 的每步 2 ms 开销意味着每次 rollout 多花 200 ms——累积起来显著拖慢训练。

rsl_rl 在 2025 年发布了 arXiv 预印本(arXiv:2509.10771 "RSL-RL: A Learning Library for Robotics Research"),系统整理了库的设计与机器人训练实践。新版本增加了多 GPU 训练支持,并可通过 PyPI 直接安装。

### PPO 算法核心回顾与完整推导

你已经熟悉 PPO 的基本思想。这里我们做完整推导,重点是**每一步的数学动机**。

#### 从策略梯度到 PPO 的演进

**起点:策略梯度定理** (Sutton et al., 2000)

策略梯度的核心思想是:直接对策略参数 $\theta$ 求目标函数的梯度。目标函数是期望累积回报:

$$J(\theta) = \mathbb{E}_{\tau \sim \pi_\theta} \left[ \sum_{t=0}^{T} \gamma^t r_t \right]$$

策略梯度定理告诉我们:

$$\nabla_\theta J(\theta) = \mathbb{E}_{\tau \sim \pi_\theta} \left[ \sum_{t=0}^{T} \nabla_\theta \log \pi_\theta(a_t | s_t) \cdot A^{\pi_\theta}(s_t, a_t) \right]$$

其中 $A^{\pi_\theta}(s_t, a_t) = Q^{\pi_\theta}(s_t, a_t) - V^{\pi_\theta}(s_t)$ 是优势函数(advantage function)。为什么要用优势函数而不是直接用 $Q$ 或累积回报?因为优势函数**减去了一个基线** $V(s_t)$,保持梯度估计无偏的同时大幅降低方差。直觉上:如果某个动作 $a_t$ 的优势为正(即它比"在该状态下的平均表现"好),就增大 $\pi_\theta(a_t|s_t)$;如果优势为负,就减小。

**问题 1:方差太大**

直接用 Monte Carlo 估计 $A(s_t, a_t)$ 方差极大——因为需要从 $t$ 到 episode 结束的所有奖励,包含了未来所有时间步里环境和策略的随机波动。从统计学角度看,回报 $G_t = \sum_k \gamma^k r_{t+k}$ 是 $V^\pi(s_t)$ 的无偏估计,但方差随 horizon 指数增长;而 TD 目标 $r_t + \gamma V(s_{t+1})$ 是有偏估计(依赖于 $V$ 的近似精度),但方差极低(只涉及一步随机性)。这就是 bias-variance 权衡的核心矛盾。解决方案:Generalized Advantage Estimation (GAE)——通过指数加权平均在两个极端之间找到最优折中。

**问题 2:步长太大**

策略梯度一步更新可能走得太远,导致策略崩溃(性能骤降且不可恢复)。这在监督学习中不常见(损失函数通常是光滑的),但在 RL 中,策略的微小变化可能导致采样分布剧变,一旦策略"走偏",就很难恢复。TRPO 通过 KL 散度约束 $D_{KL}(\pi_{\theta_{old}} \| \pi_\theta) \leq \delta$ 来限制策略在**分布空间**(而非参数空间)中的变化——这一点至关重要,因为参数空间中的微小变化可能对应策略分布的巨大变化(例如高斯分布的方差参数微调就能让分布从窄变宽)。TRPO 的 KL 约束经二阶泰勒展开可近似为 $\frac{1}{2}(\theta' - \theta)^T F (\theta' - \theta) \leq \delta$,其中 $F$ 是 Fisher 信息矩阵,由此得到自然梯度更新 $\theta \leftarrow \theta + \alpha F^{-1} \nabla_\theta J$。但这需要求解 Fisher 矩阵的逆(共轭梯度法),工程复杂度高。PPO 用 clipping 近似同样的效果,只用一阶梯度,实现极其简单。

**为什么腿足 RL 用 PPO 而非 SAC/DDPG 等 off-policy 算法?** On-policy 算法(PPO)要求数据必须由当前策略产生,用一次就丢弃,样本效率低;off-policy 算法(SAC、DDPG)可以重用历史数据(经验回放池),样本效率高。但在 GPU 并行仿真(4096 环境)下,采样速度极快,on-policy 的样本效率劣势被并行度完全抵消。而 off-policy 算法面临的"致命三要素"(Deadly Triad)——函数近似 + 自举 + 离策略数据——在腿足场景中更容易导致训练不稳定。PPO 的优势在于梯度估计无偏、收敛曲线平滑、对超参数相对鲁棒,这在需要快速迭代奖励设计的腿足 RL 工程中至关重要。

> ⚠️ **为什么需要截断：重要性采样的方差爆炸**
>
> 在 off-policy 策略梯度中,重要性权重是多步概率比的连乘:
> $$w = \prod_{t=0}^{T} \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$$
>
> 即使每一步的比值只有微小偏差(如 0.99 到 1.01),100 步连乘后方差会指数爆炸。更危险的是,如果采样分布 $q(x)$ 在某处概率极小(如 0.0001),而目标分布 $p(x)$ 在该处概率较大(如 0.1),那么权重 $\rho = p(x)/q(x) = 1000$——只要抽到一个这样的罕见样本,梯度的值就会瞬间放大 1000 倍,导致网络参数巨幅震荡甚至直接 NaN。这就是为什么 TRPO/PPO 必须限制策略更新幅度——不是出于理论优雅,而是**数值生存需要**。PPO 的 clip 操作本质上是截断了这个连乘链中的极端值。

#### GAE 完整推导

GAE (Schulman et al., 2016) 的目标是在 **偏差(bias)** 和 **方差(variance)** 之间取平衡。这个 bias-variance 权衡贯穿了 RL 优势估计的方方面面。

> 💡 **TD 自举的直觉**
>
> TD 更新的核心思想:用"一部分事实(即时奖励 $r_t$ 是确定的)+ 一个估计($V(s_{t+1})$ 是猜的)"来修正"另一个估计($V(s_t)$)"。即时奖励是锚点——它提供了真实信号,防止估计值完全脱离现实。在最开始阶段,所有的状态价值都是随机初始化的,此时状态价值完全不准确,但仍然可以通过奖励来进行估计和更新,因为真实奖励带来了绝对准确的数值。这也是 GAE 的基础:$\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$ 中的 $r_t$ 就是那个"事实锚点"。这种用"一部分事实 + 一个估计"去更新"另一个估计"的方法,在强化学习中被称为自举(Bootstrapping)。

**Step 1: TD 残差定义**

一步 TD 残差(temporal difference residual):

$$\delta_t^V = r_t + \gamma V(s_{t+1}) - V(s_t)$$

直觉:$\delta_t^V$ 是"实际收到的即时回报 + 对下一状态的估值"与"对当前状态的估值"的差。如果 $V$ 是完美的值函数,则 $\mathbb{E}[\delta_t^V] = A(s_t, a_t)$——TD 残差的期望就是真实优势函数。这一点可以严格证明:

$$\mathbb{E}[\delta_t^V] = \mathbb{E}[r_t + \gamma V(s_{t+1})] - V(s_t) = Q(s_t, a_t) - V(s_t) = A(s_t, a_t)$$

但如果 $V$ 不完美(实际训练中总是近似的),则 $\delta_t^V$ 是有偏的优势估计。

**Step 2: 多步优势估计**

用 1 步 TD 估计优势:

$$\hat{A}_t^{(1)} = \delta_t^V = r_t + \gamma V(s_{t+1}) - V(s_t)$$

特性:偏差高(只看一步,严重依赖 $V$ 的准确性),方差低(只有一个随机项 $r_t$)。

用 2 步 TD 估计:

$$\hat{A}_t^{(2)} = \delta_t^V + \gamma \delta_{t+1}^V = r_t + \gamma r_{t+1} + \gamma^2 V(s_{t+2}) - V(s_t)$$

用 $k$ 步 TD 估计:

$$\hat{A}_t^{(k)} = \sum_{l=0}^{k-1} \gamma^l \delta_{t+l}^V$$

$k \to \infty$ 时退化为 Monte Carlo:

$$\hat{A}_t^{(\infty)} = \sum_{l=0}^{\infty} \gamma^l r_{t+l} - V(s_t) = G_t - V(s_t)$$

特性:偏差零(用完整回报,不依赖 $V$ 近似),方差高(所有 $r_{t+l}$ 都是随机变量,方差累积)。

这形成了一个连续谱:$k$ 越大,偏差越低但方差越高。

**Step 3: 指数加权平均——GAE 的核心洞察**

GAE 的关键思想:不选特定的 $k$,而是对所有 $k$ 步估计做**指数加权平均**,权重为 $\lambda^{k-1}$(归一化后):

$$\hat{A}_t^{GAE(\gamma, \lambda)} = (1-\lambda) \sum_{k=1}^{\infty} \lambda^{k-1} \hat{A}_t^{(k)}$$

为什么用指数加权?因为 $k$ 越大,估计的方差越大,应该给予越低的权重。指数衰减 $\lambda^{k-1}$ 恰好实现了这种"信任度随步数衰减"的效果。

展开计算(这是关键推导,每步都写清楚):

$$\hat{A}_t^{GAE} = (1-\lambda) \left[ \hat{A}_t^{(1)} + \lambda \hat{A}_t^{(2)} + \lambda^2 \hat{A}_t^{(3)} + \cdots \right]$$

代入 $\hat{A}_t^{(k)} = \sum_{l=0}^{k-1} \gamma^l \delta_{t+l}$:

$$= (1-\lambda) \left[ \delta_t + \lambda(\delta_t + \gamma \delta_{t+1}) + \lambda^2(\delta_t + \gamma \delta_{t+1} + \gamma^2 \delta_{t+2}) + \cdots \right]$$

收集 $\delta_t$ 的系数:$(1-\lambda)(1 + \lambda + \lambda^2 + \cdots) = (1-\lambda) \cdot \frac{1}{1-\lambda} = 1$

收集 $\delta_{t+1}$ 的系数:$(1-\lambda) \cdot \gamma(\lambda + \lambda^2 + \cdots) = (1-\lambda) \cdot \frac{\gamma \lambda}{1-\lambda} = \gamma \lambda$

收集 $\delta_{t+l}$ 的系数:类推得 $(\gamma \lambda)^l$

因此:

$$\boxed{\hat{A}_t^{GAE(\gamma, \lambda)} = \sum_{l=0}^{\infty} (\gamma \lambda)^l \delta_{t+l}^V}$$

这是一个极其优雅的结果:GAE 就是 TD 残差的**指数衰减求和**,衰减因子为 $\gamma \lambda$。

**$\lambda$ 的物理含义**:

| $\lambda$ 值 | 效果 | 退化形式 | 适用场景 |
|-------------|------|---------|---------|
| $\lambda = 0$ | $\hat{A}_t = \delta_t^V$(一步 TD) | 低方差,高偏差 | $V$ 非常准确时 |
| $\lambda = 1$ | $\hat{A}_t = G_t - V(s_t)$(MC) | 零偏差,高方差 | 需要无偏估计时 |
| $\lambda = 0.95$ | 偏向低偏差的折中 | — | **腿足 RL 默认值** |

腿足 RL 选 $\lambda = 0.95$ 的理由:腿足的 reward 信号密集(每步都有 tracking reward + 多个正则项),不像 Atari 那样奖励稀疏(玩数十步才得一分)。密集 reward 意味着方差本来就不大,所以可以用较高的 $\lambda$ 降低偏差。

**Step 4: rsl_rl 的递归实现**

无穷级数在实际实现中用**从后往前的递推**:

$$\hat{A}_T = \delta_T, \quad \hat{A}_t = \delta_t + \gamma \lambda \cdot \hat{A}_{t+1}$$

这等价于无穷和(在有限 horizon $T$ 内)。递推的好处:$O(T)$ 时间复杂度,不需要存储所有 $\delta$。

```python
# rsl_rl/algorithms/ppo.py 中 GAE 的实际实现
def compute_returns(self, last_values, dones):
    """从后往前递推计算 GAE advantage"""
    advantage = 0
    for step in reversed(range(self.num_transitions_per_env)):
        if step == self.num_transitions_per_env - 1:
            next_values = last_values  # 最后一步用 Critic 估值
            next_is_not_terminal = 1.0 - dones  # done=1 时截断
        else:
            next_values = self.values[step + 1]
            next_is_not_terminal = 1.0 - self.dones[step + 1]

        # TD 残差: delta = r + gamma * V(s') - V(s)
        delta = (self.rewards[step]
                 + self.gamma * next_values * next_is_not_terminal
                 - self.values[step])

        # GAE 递推: A_t = delta_t + gamma * lambda * A_{t+1}
        advantage = delta + self.gamma * self.lam * next_is_not_terminal * advantage
        self.returns[step] = advantage + self.values[step]  # Return = A + V
```

注意 `next_is_not_terminal`:当 episode 结束(`done=True`)时,$\hat{A}_{t+1}$ 不应跨越 episode 边界,必须截断为零。这在腿足训练中非常重要——机器人摔倒后 reset 的环境,其 advantage 不应该传播到下一个 episode。

#### Clipped Objective 的数学意义

**PPO 要解决的问题**: 在用旧策略 $\pi_{\theta_{old}}$ 收集的数据上,更新到新策略 $\pi_\theta$。如果更新步长太大,$\pi_\theta$ 可能偏离 $\pi_{\theta_{old}}$ 太远,导致重要性采样比率失真,策略崩溃。

**TRPO 的解**: 加 KL 散度约束 $D_{KL}(\pi_{\theta_{old}} \| \pi_\theta) \leq \delta$。需要共轭梯度法求解约束优化,工程复杂度高。

**PPO 的解**: 用 clipping 近似 KL 约束——不需要任何二阶信息,只用一阶梯度,计算极其简单:

$$L^{CLIP}(\theta) = \mathbb{E}_t \left[ \min \left( r_t(\theta) \hat{A}_t, \; \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon) \hat{A}_t \right) \right]$$

其中 $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$ 是重要性采样比率(importance sampling ratio)。$r_t = 1$ 意味着新旧策略对该动作的概率相同;$r_t > 1$ 意味着新策略更倾向于该动作;$r_t < 1$ 则相反。

**分情况理解 clipping 的作用**:

| 条件 | $\hat{A}_t > 0$（好动作） | $\hat{A}_t < 0$（坏动作） |
|------|------------------------|------------------------|
| $r_t > 1+\epsilon$ | clip 生效,**阻止**过度增大概率 | 不 clip,允许减小概率 |
| $r_t < 1-\epsilon$ | 不 clip,允许增大概率 | clip 生效,**阻止**过度减小概率 |
| $1-\epsilon \leq r_t \leq 1+\epsilon$ | 正常梯度更新 | 正常梯度更新 |

直觉:clipping 是一个"安全带"——它不阻止你往正确方向更新,但**阻止你走得太远**。$\epsilon = 0.2$(腿足 RL 常用值)意味着每次更新最多让某个动作的概率变为原来的 0.8 到 1.2 倍。

> 💡 **PPO 截断的直觉：悲观策略**
>
> PPO 的 $\min(\text{ratio} \cdot A, \text{clip}(\text{ratio}) \cdot A)$ 本质上是一个**悲观策略**:当优势函数为正(好动作)时,限制策略往好的方向更新的幅度("少拿奖励");当优势函数为负(坏动作)时,不限制远离的速度("多担责任")。换言之:如果这一步更新让你赚翻了(提升太多),PPO 认为这可能是步长太大导致的估计误差,所以只给你算一点奖励(截断);如果这一步更新让你亏惨了(变差),PPO 认为这是真实的反馈,让你承担全部损失。这种"少拿奖励、多担责任"的不对称确保策略宁可"少学好的"也不会"突然变差",正是 PPO 能够防止策略在更新中突然崩溃(Collapse)的核心原因。

**rsl_rl 的 PPO loss 实现**:

```python
# rsl_rl/algorithms/ppo.py — 核心 loss 计算
def update(self, batch):
    # 1. 计算当前策略下动作的 log_prob
    actions_log_prob, entropy = self.actor_critic.evaluate(
        batch['obs'], batch['actions']
    )

    # 2. 重要性采样比率 r(theta)
    ratio = torch.exp(actions_log_prob - batch['old_actions_log_prob'])

    # 3. Surrogate loss (两项取 min — PPO-clip 核心)
    surrogate1 = ratio * batch['advantages']
    surrogate2 = (torch.clamp(ratio, 1.0 - self.clip_param, 1.0 + self.clip_param)
                  * batch['advantages'])
    policy_loss = -torch.min(surrogate1, surrogate2).mean()

    # 4. Value loss (Critic 更新)
    value_loss = (batch['returns']
                  - self.actor_critic.evaluate_critic(batch['obs'])).pow(2).mean()

    # 5. Entropy bonus (鼓励探索,防止策略过早收敛)
    entropy_loss = -entropy.mean()

    # 6. 总 loss = policy + value + entropy
    loss = (policy_loss
            + self.value_loss_coef * value_loss
            + self.entropy_coef * entropy_loss)
    return loss
```

#### 训练超参数指南

| 超参数 | 腿足典型值 | 为什么选这个值 |
|--------|----------|---------------|
| `clip_param` ($\epsilon$) | 0.2 | 太小(0.05)→学太慢;太大(0.5)→策略不稳定 |
| `num_epochs` | 5 | 每批数据训 5 轮;更多轮会导致 overfitting 到旧数据 |
| `num_mini_batches` | 4 | 98304 步切成 4 个 mini-batch,每个约 24k 样本 |
| `learning_rate` | 1e-3 → 5e-4 | 通常用自适应 schedule,基于实际 KL 散度 |
| `gamma` | 0.99 | 折扣因子;腿足不需要太长期规划(不像棋类) |
| `lam` ($\lambda$) | 0.95 | GAE 参数;偏向低偏差(密集 reward) |
| `entropy_coef` | 0.01 | 太大→动作过于随机;太小→过早收敛到次优 |
| `desired_kl` | 0.01 | rsl_rl 特有:实际 KL 超标时自动降 lr |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:GAE 计算时不处理 episode 边界**
> **错误做法**: 不用 `dones` mask 截断 advantage 传播。
> **后果**: 一个 episode 的 reward 信号"泄漏"到另一个 episode——例如,摔倒后的负奖励影响到 reset 后新 episode 的优势估计,导致 Critic 估值混乱,训练不稳定。
> **正确做法**: 在 `done=True` 的步骤上,`next_is_not_terminal = 0`,截断 GAE 递推。

> ⚠️ **概念误区:认为 PPO 的 clipping 保证 KL 约束**
> **新手想法**: "clipping 等价于 KL 约束,所以 PPO 就是简化版 TRPO。"
> **实际情况**: clipping 只是一个启发式近似。对于某些网络结构和数据分布,即使 clipping 了,实际 KL 散度可能仍然很大。rsl_rl 因此增加了 `desired_kl` 的自适应机制——如果实际 KL 超过目标值的 1.5 倍,自动将学习率减半;如果 KL 低于目标值的 0.5 倍,自动将学习率加倍。
> **自检方法**: 训练时监控 `approx_kl` 指标。如果它频繁超过 0.03-0.05,说明策略更新过于激进。

> ⚠️ **思维陷阱:过度调优 PPO 超参数**
> **新手想法**: "训练效果不好,一定是 PPO 超参数没调好。"
> **实际情况**: 腿足 RL 中,PPO 超参数的影响远小于**奖励设计**和 **Domain Randomization**。上表中的典型值已经被社区大量验证过,除非有特殊需求(如极长 horizon 的任务),否则不需要大幅调整。
> **正确方法**: 先用默认超参数训练;如果效果不好,优先检查奖励函数和 DR 配置。

### 练习

**练习 63.3.1**: 手推 GAE 在 3 步 episode ($T=3$) 上的展开。给定 $r_0=1, r_1=0, r_2=2$,$V(s_0)=1, V(s_1)=1.5, V(s_2)=0.5, V(s_3)=0$,$\gamma=0.99, \lambda=0.95$。先计算 $\delta_0, \delta_1, \delta_2$,再用递推公式计算 $\hat{A}_2, \hat{A}_1, \hat{A}_0$。

**练习 63.3.2**: 在 PPO 的 clipped objective 中,如果 $\hat{A}_t > 0$ 且 $r_t = 1.5$($\epsilon = 0.2$),clip 后的梯度与不 clip 时有何不同?画出 $L^{CLIP}$ 关于 $r_t$ 的函数图像(分 $\hat{A}_t > 0$ 和 $\hat{A}_t < 0$ 两种情况)。

**练习 63.3.3**: 阅读 rsl_rl 的 `ppo.py` 源码(约 200 行)。找到 `desired_kl` 自适应学习率的实现位置,解释它的完整逻辑:当实际 KL 超过目标值的几倍时降低 lr?降低到多少?

---

PPO 提供了策略优化的算法引擎,但"优化什么"取决于奖励函数的设计。下一节深入腿足 RL 中最难、最重要的工程问题——奖励设计。

## 63.4 奖励设计——腿足 RL 最难的工程问题 ⭐⭐⭐

### 动机:为什么奖励设计比算法更重要?

一个残酷的事实:腿足 RL 中,90% 的工程时间花在**奖励设计**上,而不是算法调优。PPO 的超参数基本固定(上节的表格),但奖励函数的设计空间是无限的。一个好的奖励函数让机器人优雅行走,一个差的奖励函数让机器人学会"作弊"。

### Reward Hacking:RL 的创意远超你的想象

RL 代理是极致的"考试机器"——它会找到任何你没想到的方式来最大化奖励。这种现象称为 **reward hacking**(奖励黑客)。以下是腿足 RL 中真实出现过的 reward hacking 案例:

| 你的奖励设计 | 你的期望 | RL 学到的行为 | 原因 |
|-------------|---------|-------------|------|
| 只奖励前进速度 $v_x$ | 优雅地向前走 | 用头撞地面,反复跌倒翻滚前进 | 翻滚前进比走路更快 |
| 奖励脚触地次数 | 稳定步态 | 高频抖脚(>100 Hz) | 抖脚可以极大化触地次数 |
| 惩罚关节扭矩 $\|\tau\|^2$ 权重太大 | 节能行走 | 几乎不动,微微颤抖 | 不动是扭矩最小化的全局最优 |
| 奖励活着(alive bonus)太大 | 存活越久越好 | 趴在地上不动 | 趴着=永远不摔=最大化 alive |
| 只惩罚非脚部碰撞 | 别用膝盖撞地 | 把膝盖磨在地面上滑行 | 碰撞检测范围定义不精确 |

这些案例说明:奖励设计不是"写几个数学公式"那么简单,而是一种**博弈**——你和 RL 代理之间的博弈。你定义规则,RL 找漏洞。

### 奖励设计的系统方法论

经过社区多年实践,腿足 RL 的奖励设计已经形成了系统的方法论。核心框架是**追踪奖励 + 正则项 + 终止条件**三层结构。

#### 追踪奖励(Task Reward):驱动目标行为

追踪奖励告诉 RL "你应该做什么"。腿足行走的核心目标是跟踪速度命令:

$$r_{tracking}^{lin} = \exp\left( -\frac{\|v_{xy}^{actual} - v_{xy}^{cmd}\|^2}{\sigma_{lin}^2} \right)$$

$$r_{tracking}^{ang} = \exp\left( -\frac{(\omega_z^{actual} - \omega_z^{cmd})^2}{\sigma_{ang}^2} \right)$$

**为什么用 $\exp(-\|\cdot\|^2 / \sigma^2)$ 而不是 $-\|\cdot\|^2$?** 这是一个重要的设计选择,值得深入理解:

| 形式 | $-\|v - v_{cmd}\|^2$ | $\exp(-\|v - v_{cmd}\|^2 / \sigma^2)$ |
|------|---------------------|-------------------------------------|
| 值域 | $(-\infty, 0]$ | $[0, 1]$ |
| 完美匹配时 | 0 | 1 |
| 微小偏差 | 接近 0(梯度小) | 接近 1(梯度小) |
| 大偏差 | 大负值(梯度大) | 接近 0(梯度趋零) |
| 问题 | 大偏差时梯度过大,可能不稳定 | **有界**,不会梯度爆炸 |

$\exp$ 形式的本质好处:奖励有界在 $[0, 1]$ 区间,与其他奖励项的权重容易平衡。$\sigma$ 参数控制"容错范围"——$\sigma$ 越大,对偏差越宽容。典型的 $\sigma_{lin} = 0.25$ m/s,意味着误差在 0.25 m/s 以内时奖励仍大于 $1/e \approx 0.37$。

#### 正则项(Regularization):限制不良行为

正则项告诉 RL "你不应该做什么"。每个正则项背后都有明确的物理动机。下面逐一解析:

```python
class RewardTerms:
    """每个奖励项的物理动机和数学形式"""

    # === 身体姿态正则 ===

    # 1. 身体水平: r = -(roll^2 + pitch^2)
    # 物理动机: 倾斜的身体重心偏离支撑多边形,容易摔倒
    # 如果没有: 机器人可能学会侧倾行走(更快但不稳定)
    # 与 Ch53 WBC 的联系: WBC 也有姿态保持 task,这里用 reward 替代
    r_orientation = -(projected_gravity_xy.norm(dim=-1))**2

    # 2. Z 方向线速度: r = -v_z^2
    # 物理动机: 垂直方向速度意味着跳跃或正在摔倒
    # 如果没有: 机器人可能学会"蹦跶着走"(跳跃前进)
    r_lin_vel_z = -base_lin_vel[:, 2]**2

    # 3. Roll/Pitch 角速度: r = -(w_x^2 + w_y^2)
    # 物理动机: 绕非 yaw 轴的旋转意味着姿态不稳
    r_ang_vel_xy = -base_ang_vel[:, :2].norm(dim=-1)**2

    # === 能量效率正则 ===

    # 4. 扭矩正则: r = -||tau||^2
    # 物理动机: 真机电机功率有限,省电=续航更长
    # 如果没有: 策略可能用过大的扭矩(仿真不在乎,真机烧电机)
    # 关键: 权重必须很小(~1e-5),否则机器人不敢动
    r_torques = -torch.sum(torques**2, dim=-1)

    # 5. 关节加速度: r = -||ddq||^2
    # 物理动机: 高加速度 -> 高冲击力 -> 机械磨损 + 噪声
    r_dof_acc = -torch.sum(
        ((last_dof_vel - dof_vel) / dt)**2, dim=-1
    )

    # === 动作平滑正则 ===

    # 6. 动作变化率: r = -||a_t - a_{t-1}||^2
    # 物理动机: 动作剧烈变化 -> 关节指令跳变 -> 电机抖动
    # 如果没有: 策略输出可能在相邻步之间剧烈波动
    r_action_rate = -torch.sum((actions - last_actions)**2, dim=-1)

    # === 接触正则 ===

    # 7. 非脚部碰撞: r = -count(collisions)
    # 物理动机: 膝盖、身体撞地 = 硬件损坏
    r_collision = -torch.sum(
        (contact_forces[:, non_feet_indices, :].norm(dim=-1) > 0.1).float(),
        dim=-1
    )

    # 8. 脚部空气时间: r = f(air_time)
    # 物理动机: 鼓励合理步频(不是一直贴地拖行)
    # 形式: 只在脚着地瞬间给奖励,奖励与空气时间成正比
    r_feet_air_time = torch.sum(
        (feet_air_time - 0.5) * first_contact, dim=-1
    )
```

#### 权重平衡的艺术

奖励函数的最终形式:$r_{total} = \sum_i w_i \cdot r_i$。权重的设计原则:

1. **追踪奖励的量级远大于正则项**: 追踪 $w \sim 1.0$,正则 $w \sim 0.001-0.1$。如果正则项太大,机器人"不敢动"(因为任何动作都会增加扭矩/加速度/碰撞等惩罚)
2. **先追踪,再正则**: 先只用追踪奖励训到能走,然后逐个加正则项,观察行为变化
3. **用 TensorBoard 监控每个奖励项的贡献**: 如果某个正则项的累积贡献超过追踪项,说明权重失衡

```python
# legged_gym 的典型权重配置 (anymal_c_config.py)
class scales:
    # 追踪(正,大权重)— 主导行为方向
    tracking_lin_vel = 1.0     # 主目标:跟踪线速度
    tracking_ang_vel = 0.5     # 次目标:跟踪角速度

    # 正则(负,小权重)— 约束行为品质
    lin_vel_z = -2.0           # 中等惩罚(跳跃对硬件冲击大)
    ang_vel_xy = -0.05         # 轻微惩罚(轻微摇晃可接受)
    orientation = -0.0         # 可选(某些配置不启用)
    torques = -0.00001         # 极小(否则不动)
    dof_vel = -0.0             # 可选
    dof_acc = -2.5e-7          # 极小
    action_rate = -0.01        # 中等惩罚(平滑性重要)
    collision = -1.0           # 强惩罚(碰撞是硬约束)
    dof_pos_limits = -10.0     # 极强惩罚(关节超限很危险)
```

#### 奖励系数的课程学习

一个重要的实践经验是:如果从训练一开始就施加全部正则惩罚,智能体很可能学会"站着不动"——因为任何运动都会产生力矩、加速度和碰撞等惩罚,不动反而是全局最优。解决方案是对惩罚系数做课程学习:训练初期使用极小的惩罚系数,让策略先学会"如何动起来";随训练推进逐步增大系数,迫使策略优化动作品质。同时可以对域随机化的扰动难度(如质量偏移幅度、外力大小)进行线性递增。这种"先学会走,再学会走好"的策略比"一开始就要求完美"更容易收敛。

> 💡 **训练技巧：奖励系数 curriculum**
>
> 除了常见的**地形难度 curriculum**，**惩罚项系数 curriculum** 在实践中同样重要。正确做法：保留足够的任务奖励（速度跟踪等），训练初期将能耗、平滑度、冲击力等**惩罚项系数设低**，让策略先学会运动；随训练推进逐步加严惩罚，迫使策略优化动作品质。
>
> 如果初期惩罚过高，策略会发现"不动 = 零惩罚"是局部最优，退化为静止——这是腿足 RL 最常见的训练病灶。

#### 额外的物理启发奖励项

除了上面列出的标准奖励集,以下几个来自具体工程实践的奖励项也值得考虑:

| 奖励项 | 公式 | 物理动机 |
|--------|------|---------|
| 机械功 | $-\|\tau \odot (q_t - q_{t-1})\|_1$ | 惩罚逐关节绝对做功,避免正负功在关节间互相抵消 |
| 地面冲击 | $-\|f_t - f_{t-1}\|^2$ | 惩罚足端接触力突变,保护硬件 |
| 足端打滑 | $-\|\text{diag}(g_t) \cdot v_{f,t}\|^2$ | 触地脚不应有水平速度 |
| 关节机械功率 | $-\sum_i |\tau_i \cdot \dot{q}_i|$ | 正功率之和应尽可能小（取绝对值） |
| 动作平滑度(二阶) | $-\|a_t - 2a_{t-1} + a_{t-2}\|^2$ | 抑制动作的二阶变化(抖动) |

其中机械功和地面冲击奖励源于生物能量学的启发——自然界中动物的步态正是能量效率和冲击最小化的演化结果。在实践中,这些奖励对于在仿真中学到可直接迁移到真机的自然步态非常关键。注意不同论文对"work"的离散写法并不完全相同:有的对总功取绝对值,有的对每个关节的功逐项取绝对值再求和。后者更保守,可以避免一个关节正功被另一个关节负功抵消。

#### 标志性论文的奖励函数设计

以下两篇标志性论文的完整奖励配置具有重要参考价值——它们提供了可直接复现的具体数值,而不仅仅是定性描述。

**Miki et al. (2022) — Learning robust perceptive locomotion (Science Robotics)**

该工作使用教师-学生架构训练 ANYmal 在阿尔卑斯山徒步,教师网络使用 7 个奖励项,总奖励为线性加权和:

$$r = 0.05\,r_{lv}+0.05\,r_{av}+0.04\,r_b+0.01\,r_{fc}+0.02\,r_{bc}+0.025\,r_s+2\times 10^{-5}\,r_{\tau}$$

| 奖励项 | 含义 | 公式/描述 | 权重 |
|--------|------|---------|------|
| $r_{lv}$ | 线速度奖励 | 基座速度在目标方向上的投影 $v_{pr}$;$v_{pr}<0.6$ m/s 时指数逼近,$v_{pr}\ge 0.6$ m/s 时饱和满分 | 0.05 |
| $r_{av}$ | 角速度奖励 | 鼓励绕基座 z 轴按指令方向转动,与 $r_{lv}$ 类似的分段/饱和设计 | 0.05 |
| $r_b$ | 机身运动稳定 | 惩罚偏离目标方向的速度分量(横向/无效速度)与 roll/pitch 方向角速度 | 0.04 |
| $r_{fc}$ | 摆动腿足部净空 | 摆动相位时鼓励足端高度高于周围地形高度,统计满足净空条件的比例 | 0.01 |
| $r_{bc}$ | 机身碰撞惩罚 | 惩罚身体部件与地形的不期望接触(不包含足端正常接触) | 0.02 |
| $r_s$ | 目标足端轨迹平滑 | 惩罚目标足端位置的二阶差分(离散"加速度/抖动"),抑制抖动 | 0.025 |
| $r_{\tau}$ | 关节力矩惩罚 | 惩罚关节力矩绝对值之和,降低能耗并保护执行器 | $2\times 10^{-5}$ |

注意力矩惩罚权重 $2\times 10^{-5}$ 远小于追踪项——这是"先让机器人动起来,再优化能耗"思想的体现。

**RMA (Kumar et al., 2021) — Rapid Motor Adaptation (RSS)**

RMA 使用 10 个奖励项,受生物能量学启发,鼓励减少做功与地面冲击。记基座线速度为 $v$,角速度为 $\omega$,关节角度为 $q$,关节速度为 $\dot{q}$,关节力矩为 $\tau$,足端接触力为 $f$,足端速度为 $v_f$,足端接触指示为 $g$:

| 序号 | 奖励项 | 公式 | 缩放系数 |
|------|--------|------|---------|
| 1 | Forward（前进奖励） | $\min(v_x^t,\; 0.35)$ | 20 |
| 2 | Lateral + Rotation（横向 + 旋转惩罚） | $-\|v_y^t\|^2 - \|\omega_{yaw}^t\|^2$ | 21 |
| 3 | Work（机械功惩罚） | $-\left|\tau^{tT}(q^t - q^{t-1})\right|$ | 0.002 |
| 4 | Ground Impact（地面冲击惩罚） | $-\|f^t - f^{t-1}\|^2$ | 0.02 |
| 5 | Smoothness（平滑度惩罚） | $-\|\tau^t - \tau^{t-1}\|^2$ | 0.001 |
| 6 | Action Magnitude（动作幅度惩罚） | $-\|a^t\|^2$ | 0.07 |
| 7 | Joint Speed（关节速度惩罚） | $-\|\dot{q}^t\|^2$ | 0.002 |
| 8 | Orientation（姿态稳定性惩罚） | $-\|\theta_{roll, pitch}\|^2$ | 1.5 |
| 9 | Z Velocity（垂直速度惩罚） | $-\|v_z^t\|^2$ | 2.0 |
| 10 | Foot Slip（足端打滑惩罚） | $-\|\text{diag}(g^t) \cdot v_f^t\|^2$ | 0.8 |

RMA 的一个关键训练技巧:如果直接使用上述奖励训练,智能体会保持不动(因为关节运动相关的惩罚项压制运动)。因此训练初期使用很小的惩罚系数,用固定 curriculum 逐步增大;同时随训练推进线性增加质量、摩擦、电机强度等扰动难度。地形则从一开始就随机采样固定难度,不做 curriculum。

### Margolis "Walk These Ways" 的奖励设计创新

Margolis 和 Agrawal (2022, CoRL) 提出了 **Multiplicity of Behavior (MoB)** 的概念。核心思想:不是训一个固定步态的策略,而是训一个能执行**多种步态**的策略,通过命令向量在运行时选择。把步态参数(步频、步幅、身体高度、姿态偏移等)作为**额外的命令输入**:

$$\text{cmd} = [v_x, v_y, \omega_z, \underbrace{f_{step}, h_{body}, \phi_{offset}, \ldots}_{\text{步态参数命令}}]$$

训练时随机采样这些参数,策略学会"给什么参数就做什么步态"。一个策略即可做:慢走、快跑、蹲行、跳跃、舞蹈等。这对奖励设计的启示是:奖励函数不应该 hard-code 一种"正确行为",而应该**参数化地**定义目标。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:奖励函数中的数值溢出**
> **错误做法**: `r = exp(-error)` 其中 `error` 可能是极大的正数(如 $10^6$)。
> **后果**: `exp(-1e6)` 在 float32 下精确为 0.0,梯度为零,策略对这个奖励项失去学习信号。
> **根本原因**: 训练早期,跟踪误差可能非常大(策略几乎是随机的)。
> **正确做法**: 使用 `exp(-error / sigma**2)` 并确保 `sigma` 足够大(如 `sigma=0.25` 意味着误差需要到 ~1.0 才严重衰减),或者先 clip error 再取 exp。

> ⚠️ **概念误区:认为所有正则项都应该同时打开**
> **新手想法**: "legged_gym 配置里有 20 个奖励项,全部打开肯定更好。"
> **实际情况**: 过多的正则项会产生**冲突**。例如,`feet_air_time`(鼓励抬脚)和 `dof_vel`(限制关节速度)可能矛盾——抬脚需要关节速度。冲突的奖励项让 RL 找不到好的解,可能收敛到一个所有正则项的"妥协"——但这个妥协往往不是你想要的行为。
> **正确做法**: 从最小集开始(追踪 + 3-5 个关键正则),逐步增加,每次 ablation 验证。

> ⚠️ **思维陷阱:照搬论文的奖励权重到不同机器人**
> **新手想法**: "Miki 2022 的权重在 ANYmal 上效果好,直接用到 Go2 上。"
> **实际情况**: ANYmal(~50 kg,SEA 驱动器,关节减速比高)和 Go2(~12 kg,QDD 电机,低减速比)的动力学特性完全不同。ANYmal 的扭矩惩罚权重对 Go2 来说太小(Go2 电机更小,扭矩范围不同)。
> **正确做法**: 理解每个权重背后的**物理含义**,根据机器人特性(质量、电机型号、减速比)重新标定。Ch62 的电机动力学知识在这里非常有用。

### 练习

**练习 63.4.1**: 设计一个"让 Go2 后退行走"的奖励函数。需要修改哪些追踪项?需要添加哪些新的正则项?(提示:后退时机器人看不到身后,碰撞风险更高)

**练习 63.4.2**: 做一个 ablation 实验:以训好的 Go2 trot 策略为基线,分别(a) 删除 `action_rate` 奖励,(b) 将 `torques` 惩罚权重放大 100 倍,(c) 删除 `lin_vel_z` 惩罚。预测每个 ablation 的行为变化,然后训练 30 分钟验证。

**练习 63.4.3**: Margolis 的 Walk These Ways 引入了"步态参数命令"。如果你想让策略能在 trot 和 bound(两腿同步)之间切换,你需要添加哪些命令维度?对应的奖励项如何设计?

---

奖励函数告诉 RL "什么是好的行为",但仿真环境与真实世界的差距可能让这些行为在真机上失效。下一节深入 Domain Randomization——系统化地缩小 sim-to-real gap。

## 63.5 Domain Randomization:从贝叶斯视角理解鲁棒性 ⭐⭐⭐

### 动机:为什么仿真训好的策略在真机上会失败?

即使你的奖励设计完美,策略在仿真中表现优异,部署到真机时仍然可能失败。原因是 **sim-to-real gap**(仿真-真实差距):

| 差距来源 | 仿真中的值 | 真实世界 |
|---------|-----------|---------|
| **接触模型** | penalty-based(柔性穿透) | 刚性接触,更复杂的力学 |
| **摩擦** | 单一系数,恒定 | 各向异性,与速度/温度/磨损有关 |
| **电机** | 理想扭矩源 | 有带宽、延迟、齿槽效应(Ch62) |
| **传感器** | 完美数值 | 有噪声、偏置、延迟、漂移 |
| **质量/惯量** | CAD 标称值 | 制造公差,负载变化 |
| **通信延迟** | 0 或固定 | 1-5 ms,有抖动(Ch61) |

如果策略 overfit 到仿真的特定参数(如摩擦系数恰好是 0.8),那么在真实地面(摩擦系数可能是 0.5 或 1.2)上就会失败——策略的控制行为是针对 0.8 的摩擦"量身定制"的,换了摩擦就像换了鞋子一样不适应。

将 sim-to-real gap 的来源系统地展开,可以归纳为四大类:

**动力学建模误差**。真实物体的质量分布无法完全可知,惯性张量标定存在误差,关节摩擦通常呈非线性特性(包含库仑摩擦、粘滞摩擦和 Stribeck 效应),而仿真中往往用线性模型近似。地面摩擦系数在不同材料和工况下变化剧烈,接触力学被简化为 penalty-based 模型,无法精确表征形变和能量耗散。

**物理参数偏差**。重力加速度因地理位置和海拔略有差异(影响 IMU 校准),空气阻力在高速运动和轻型机器人上不可忽视但仿真通常不建模,材料弹性和阻尼系数影响碰撞响应但仿真参数难以匹配真实值。

**感知层面差异**。真实传感器的噪声远比仿真中假设的高斯模型复杂——IMU 存在偏置漂移和温漂,编码器存在量化误差,视觉传感器受光照、纹理和遮挡影响。传感器采样延迟和处理延迟在仿真中通常被假设为零或固定值,但真实系统中会出现抖动。数据丢帧和读取失败在仿真中几乎不考虑。

**执行层面问题**。电机从接收指令到输出力矩存在时间延迟,真实控制器的数字量化带来指令精度损失。电机存在力矩和速度饱和,其饱和曲线是高度非线性的——尤其当电机高速转动时,反电动势(Back-EMF)会显著降低有效力矩输出:

$$\tau = K_t \cdot \frac{V_{pwm} - K_e \dot{q}}{R}$$

其中 $K_t$ 为力矩常数,$K_e$ 为反电动势常数,$R$ 为线圈电阻。当转速 $\dot{q}$ 增大时,可用于产生力矩的有效电压 $(V_{pwm} - K_e \dot{q})$ 减小,导致高速时力矩衰减。如果仿真中忽略这一特性,策略会在高速运动时因力矩不足而失控。Hwangbo et al. (2019, Science Robotics) 的工作表明,用神经网络或解析模型精确建模执行器动力学(包括反电动势、磁饱和、非线性摩擦)对 sim-to-real 成功至关重要,该方法已被 IsaacLab 的 `ActuatorNetMLP` 和 `ActuatorNetLSTM` 模块标准化。

理解这四类 gap 的实际意义在于:每一类都对应着不同的缓解策略——动力学误差靠 DR 和执行器建模来覆盖,物理参数偏差靠系统辨识和 DR 范围校准,感知差异靠观测噪声注入和历史堆叠,执行层面靠延迟建模和动作平滑正则化(参见 63.4 节的 `action_rate` 奖励项)。

上述四类 gap 按**缓解手段**分类。下表从**来源**角度重新整理，便于逐项检查：

#### Sim-to-Real Gap 的五大来源

在讨论如何缩小仿真-现实差距之前，需要先理解差距从何而来：

| Gap 类别 | 具体表现 | 影响程度 | 主要缓解手段 |
|---------|---------|---------|------------|
| **动力学模型** | 接触模型不准确（穿透/弹跳）、关节摩擦建模粗糙 | 高 | 系统辨识 + DR |
| **执行器特性** | 力矩-电流非线性、高速力矩衰减、传动间隙、延迟 | 高 | 执行器网络建模(Hwangbo2019) |
| **感知噪声** | IMU 漂移/偏置、编码器量化、相机延迟/遮挡 | 中 | 传感器噪声 DR + 观测滤波 |
| **接触特性** | 摩擦系数不确定、地形形状/刚度未知 | 中 | 摩擦/地形 DR |
| **环境复杂度** | 仿真场景过于简化（无风/无外部干扰/平坦地形） | 中 | 外力扰动 DR + 地形 curriculum |

> 💡 **工程判断**：执行器特性 Gap 往往比动力学模型 Gap 影响更大。精确建模 MuJoCo/Isaac 的刚体动力学后，残留的最大误差源通常是电机非理想特性——这也是为什么 Hwangbo et al. (2019) 的 Actuator Net 在 sim-to-real 领域如此重要。

### Domain Randomization 的贝叶斯解释

Domain Randomization 好比**考试前做各种难度的模拟题**——如果你只刷一种难度的题目（固定仿真参数），上了考场遇到稍有变化的题型就会懵；但如果平时练习时随机混合简单题、中等题和难题（随机化质量、摩擦、延迟等参数），虽然每套模拟卷的分数可能不是最高，但真正考试时无论出什么难度都能稳定发挥。DR 的本质就是用训练时的"题型多样性"换取部署时的"鲁棒性"。

Domain Randomization (DR) 的操作很简单:**训练时随机化环境参数**。但**为什么这样做能提高鲁棒性?** 从贝叶斯推断的视角可以给出严格的理论基础。

**形式化**: 设环境参数为 $\xi \in \Xi$($\xi$ 是一个向量,包括质量、摩擦、延迟等)。标准训练目标:

$$\max_\theta J(\theta, \xi_{sim})$$

仅在仿真参数 $\xi_{sim}$ 下最优——如果 $\xi_{real} \neq \xi_{sim}$,性能可能大幅下降。

DR 的训练目标:

$$\max_\theta \mathbb{E}_{\xi \sim p(\xi)} \left[ J(\theta, \xi) \right]$$

即最大化在参数分布 $p(\xi)$ 下的**期望**性能。

**贝叶斯视角**: 我们不知道真机的精确参数 $\xi_{real}$,但有先验信念 $p(\xi)$ 认为 $\xi_{real}$ 落在某个范围内。DR 训练等价于在**先验分布**下做贝叶斯最优决策——策略在整个先验覆盖的参数空间上表现最好,而不是在某个特定点上最优。

**DR 成功的必要条件**: 如果 $\xi_{real} \in \text{support}(p(\xi))$——即先验分布覆盖了真实参数值——那么 DR 训练出的策略**有可能**在真机上工作。覆盖是必要条件,不是充分条件:最终效果还取决于分布权重、策略容量、优化质量和仿真模型结构误差。这给出了一个明确的工程指导:**DR 范围必须覆盖真实参数的可能范围**,但不能指望"范围足够大"自动解决所有 sim-to-real 问题。

**与其他鲁棒方法的关系**:

| 方法 | 目标 | 特点 |
|------|------|------|
| **Robust Control (minimax)** | $\max_\theta \min_\xi J(\theta, \xi)$ | 对最坏情况鲁棒,过度保守 |
| **Domain Randomization** | $\max_\theta \mathbb{E}_\xi [J(\theta, \xi)]$ | 对分布鲁棒,温和有效 |
| **System Identification** | 先辨识 $\xi_{real}$,再针对优化 | 精确但费时,不泛化 |

DR 是这三者之间的甜蜜点:不像 minimax 那样过度保守(最坏情况可能永远不会发生),也不像 System ID 那样脆弱(辨识一次就固定了)。

### 腿足 DR 的典型配置

```python
class DomainRandCfg:
    """Go2 的 Domain Randomization 配置"""

    # === 动力学参数 ===
    mass_randomization_range = [0.85, 1.15]   # 质量 ±15%
    friction_range = [0.5, 1.25]               # 摩擦系数
    restitution_range = [0.0, 0.5]             # 弹性恢复
    com_displacement_range = [-0.05, 0.05]     # 质心偏移(m)

    # === 电机参数 ===
    motor_strength_range = [0.9, 1.1]          # 电机强度 ±10%
    joint_friction_range = [0.0, 0.05]         # 关节摩擦(Nm)

    # === 控制参数 ===
    action_delay_range = [0, 2]   # 步数(每步 10ms → 0~20ms)
    kp_range = [0.8, 1.2]         # PD 比例增益 ±20%
    kd_range = [0.8, 1.2]         # PD 微分增益 ±20%

    # === 观测噪声 ===
    imu_angular_velocity_noise = 0.2  # rad/s
    imu_gravity_noise = 0.05
    joint_pos_noise = 0.01            # rad
    joint_vel_noise = 1.5             # rad/s

    # === 外部扰动 ===
    push_interval_s = [5, 15]          # 每隔 5-15 秒推一次
    push_velocity_range = [0, 1.0]     # 推力效果(m/s)
```

#### 域随机化参数速查表

下表汇总了腿足 RL 训练中常用的域随机化参数、典型取值范围及其物理依据,可直接用于配置训练环境。数值不是固定标准,应根据机器人质量、执行器带宽、传感器噪声和目标地形重新标定:

| 类别 | 参数 | 随机化范围（典型值） | 说明 |
|------|------|-------------------|------|
| **质量/惯性** | 机器人总质量 | $\pm 20\%$ | 覆盖制造误差和磨损导致的质量变化 |
| | 各 link 质量 | $\pm 15\%$ | 各部件的质量分布不确定性 |
| | 负载质量 | 0-5 kg | 模拟不同负载情况下的运动 |
| **摩擦** | 地面摩擦系数 | 0.3-1.2 | 覆盖湿滑地面到粗糙地面的摩擦范围 |
| | 关节摩擦力 | $\pm 30\%$ | 考虑关节磨损和润滑状态变化 |
| | 滚动阻力 | $\pm 50\%$ | 模拟不同地面材质的滚动阻力 |
| **刚度/阻尼** | 关节刚度 | $\pm 20\%$ | 考虑关节柔性和弹性变化 |
| | 关节阻尼 | $\pm 30\%$ | 不同阻尼特性对运动的影响 |
| | 地面刚度 | $\pm 40\%$ | 覆盖软地面到硬地面的刚度范围 |
| **传感器噪声** | IMU 噪声 | 高斯噪声 + 偏置漂移 | 模拟加速度计和陀螺仪的测量误差 |
| | 编码器噪声 | $\pm 0.01$ rad | 考虑关节角度测量的精度限制 |
| | 力传感器噪声 | $\pm 5\%$ | 模拟接触力测量的不确定性 |
| **执行器** | 电机响应延迟 | 0-50 ms | 电机从接收指令到输出力矩的延迟 |
| | 控制指令延迟 | 0-20 ms | 控制器计算和通信延迟 |
| | 力矩限制 | $\pm 10\%$ | 模拟电机力矩输出能力的变化 |
| **环境参数** | 重力加速度 | $\pm 0.2$ m/s$^2$ | 人为鲁棒性扰动；真实地球表面重力变化远小于此量级,这里不是在模拟地理位置差异 |
| | 空气密度 | $\pm 10\%$ | 不同温度和湿度下的空气阻力 |
| | 温度影响 | -10$^\circ$C 到 40$^\circ$C | 温度对材料特性和电机性能的影响 |

更粗略的经验法则:物理参数(质量 $\pm 30\%$、摩擦系数 0.3-1.5、阻尼 $\pm 50\%$),动力学参数(延迟 0-50 ms、力矩噪声 $\pm 10\%$),视觉参数(光照、纹理、颜色、相机位置)。

> 💡 **使用建议**：**调试阶段**可短暂关闭 DR 以排查 pipeline 问题（奖励函数、观测维度等）。但**正式训练应从一开始就开启 DR**（参见下文"先训好策略再加 DR"陷阱）。如果训练不稳定，应缩小 DR 范围而非完全关闭。具体做法是在每个 Episode 开始时，从预定义的范围内随机采样物理参数和传感器噪声等,让策略在多样化的动力学条件下学习。

### DR 强度的权衡

| DR 强度 | 仿真内表现 | 真机表现 | 策略特征 |
|---------|-----------|---------|---------|
| **无 DR** | 完美 | 几乎必定失败 | 激进,overfit 到仿真 |
| **适中 DR** | 很好 | 大概率成功 | 自然,有一定保守性 |
| **过大 DR** | 一般 | 能跑但保守 | 缓慢,过度小心 |

直觉:DR 越大,策略需要在更广的参数范围内"妥协",单点最优性能越低。关键是找到**刚好覆盖真实参数**的 DR 范围——不多不少。仿真环境和真实环境本质上是两个高维分布,DR 的目标是让仿真分布**包含**真实分布,从而使仿真下学到的策略在真实环境中仍然有效。

### 自适应 DR (ADR)

OpenAI 在 Rubik's Cube 项目中提出了 Automatic Domain Randomization (ADR),自动调整 DR 范围:

1. 初始化一个小的 DR 范围 $[\xi_{min}^{(0)}, \xi_{max}^{(0)}]$
2. 训练策略到性能阈值
3. 在边界参数 $\xi_{min}$ 和 $\xi_{max}$ 上评估策略
4. 如果策略在边界表现好,**自动扩大范围**
5. 如果表现差,**缩小范围**
6. 重复直到收敛

ADR 避免了手动调 DR 范围的痛苦。但实现复杂度较高,目前腿足社区更多使用**手动 DR + 消融实验**的方式。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:质量随机化后不更新惯量矩阵**
> **错误做法**: 只随机化质量标量 $m$,不更新 inertia tensor $I$。
> **后果**: 质量和惯量不匹配——质量增大但惯量不变,意味着旋转太容易(仿真中出现非物理的动力学行为)。策略学到的行为在真机上不成立。
> **正确做法**: 假设密度均匀变化,惯量按质量比例缩放:$I_{new} = \frac{m_{new}}{m_{old}} I_{old}$。

> ⚠️ **概念误区:认为 DR 能解决所有 sim-to-real gap**
> **新手想法**: "只要 DR 范围够大,真机一定能工作。"
> **实际情况**: DR 只能解决**参数化的**差距——即那些可以用数值描述并在仿真中调整的差距(质量、摩擦等)。如果仿真器的物理模型与真实世界**结构性不同**(如 penalty-based 接触 vs 刚性接触),DR 无法弥补这种**模型误差**。
> **正确认知**: DR 是 sim-to-real 的必要条件,但不充分。还需要 Teacher-Student(63.7)、Actuator Network(学习电机非理想特性)和真机微调(Ch64)。

> 💡 **前沿范式：Real2Sim2Real 闭环**
>
> 传统 sim-to-real 是单向的（仿真→现实），Real2Sim2Real 构成闭环：
> 1. **Real→Sim**：从真机数据（运动捕捉/力传感器）辨识系统参数，校准仿真模型
> 2. **Sim（训练）**：在校准后的仿真中训练/改进策略
> 3. **Sim→Real**：部署到真机验证
> 4. **Real→Sim（迭代）**：用新的真机数据进一步校准仿真
>
> 核心观念转变：**仿真不是一次性建模，而是持续校准的闭环过程**。每次真机测试的数据都是校准仿真的宝贵资源。

#### 系统辨识的具体流程

Real→Sim 环节的核心是**系统辨识**（System Identification）：用真机数据校准仿真参数。典型流程：

**Step 1：数据采集**
在真机上执行**标准化动作序列**（非 RL 策略，而是预设的关节运动）：
- 各关节独立正弦扫频（0.5-10 Hz），覆盖工作范围
- 阶跃响应（突然施加/撤除力矩）
- 自由落体/着地冲击（标定接触参数）

记录三元组：$(q(t),\; \dot{q}(t),\; \tau_{cmd}(t))$ 以及 IMU 数据，采样率 ≥ 1 kHz。

**Step 2：定义待辨识参数**
典型参数集：

| 参数类别 | 具体参数 | 初始值来源 |
|---------|---------|-----------|
| 连杆惯性 | 质量、质心位置、惯性张量 | CAD 模型 |
| 关节摩擦 | 库仑摩擦、粘滞摩擦系数 | 经验值 |
| 执行器 | 力矩常数、反电动势常数、电阻 | 电机数据手册 |
| 接触 | 刚度、阻尼、摩擦系数 | 材料手册 |

**Step 3：优化求解**
定义代价函数为仿真轨迹与真机轨迹的差异：
$$\min_{\xi} \sum_{t} \|q_{sim}(t; \xi) - q_{real}(t)\|^2 + \|\dot{q}_{sim}(t; \xi) - \dot{q}_{real}(t)\|^2$$

常用优化器：
- **CMA-ES**（协方差矩阵自适应进化策略）：无梯度、全局搜索，适合参数空间 < 50 维
- **贝叶斯优化**：样本效率高，适合评估代价昂贵的情况
- **差分进化**：简单鲁棒，常作为 baseline

**Step 4：验证**
用辨识后的参数在仿真中回放**未参与优化的**真机轨迹，检查泛化性。如果仅在优化数据上拟合好但新数据上偏差大，说明过拟合或模型结构不足。

> ⚠️ **工程陷阱**：不要用 RL 策略的轨迹做系统辨识——RL 策略会主动适应仿真误差，掩盖了真实的参数偏差。应使用开环或简单闭环的标准化动作。

> ⚠️ **思维陷阱:先训好策略再加 DR**
> **新手想法**: "先在无 DR 环境训到满意的策略,然后加 DR fine-tune。"
> **实际情况**: 从无 DR 到有 DR 的切换通常会让策略严重退化甚至崩溃——因为策略已经 overfit 到仿真的特定参数,面对随机化的参数很难立即适应。从头带 DR 训练效果远优于后期加 DR。
> **正确做法**: 从一开始就用 DR 训练。如果训练困难,可以降低 DR 范围(但不去掉)。

### 练习

**练习 63.5.1**: 训练两个版本的 Go2 策略——Version A 无 DR,Version B 有 DR(使用上面的配置)。在仿真中分别测试:固定摩擦=0.8 vs 摩擦=0.3。哪个版本的性能下降更大?为什么?

**练习 63.5.2**: 从贝叶斯角度分析:如果你知道真机的摩擦系数在 $[0.6, 0.9]$ 范围内,但你的 DR 设置为 $[0.3, 1.5]$。DR 范围过大有什么后果?(提示:策略需要在更大范围内"妥协",单点性能下降。)如何确定合适的 DR 范围?

---

DR 通过参数化地模拟环境不确定性来提高鲁棒性。但有些信息(如地形形状)无法简单地参数化。下一节介绍 Curriculum Learning——让训练从简单到复杂渐进。

## 63.6 Curriculum Learning:从简单到复杂的渐进训练 ⭐⭐

### 动机:为什么不能一开始就在最难的地形上训?

想象你在教一个孩子骑自行车。你不会一开始就带他去山路——你会先在平地上让他找到平衡,再去缓坡,最后才是复杂路况。Curriculum Learning 的思想完全一样。

如果一开始就在困难地形(高台阶、陡坡)上训练 RL 策略,策略大概率**学不到任何有用的东西**——因为在困难地形上,随机策略几乎立刻摔倒,收集不到有意义的正向 reward 信号。这在 RL 中叫做**探索困难**(exploration difficulty):奖励如此稀疏,以至于策略无法通过随机探索找到好的行为。

### 如果不用 Curriculum 会怎样

两种极端都有问题:

| 策略 | 后果 |
|------|------|
| **只在简单地形(平地)训** | 策略在平地完美,遇到台阶立刻摔倒 |
| **直接在最难地形训** | 策略长时间学不到东西(随机动作在难地形上立刻失败),训练效率极低 |

Curriculum 是两者的平衡:从简单开始建立基本技能,逐步增加难度。

### 可通行性度量与自适应地形生成

设计 curriculum 的核心问题是:如何量化"当前策略在某个难度的地形上的表现",以及如何据此决定是升级还是降级?

一种经过验证的方法是定义**可通行性(traversability)**度量:衡量策略在特定地形参数下的通过概率。具体做法是设定一个速度阈值(如 0.2 m/s,约为最大速度的 1/3),如果策略能以高于该阈值的速度沿指令方向持续前进,则认为该地形是"可通行"的。可通行性定义为多次采样的成功比例:

$$Tr(c_T, \pi) = \mathbb{E}_{\tau \sim p_\pi(\tau|c_T)}\{v(s_t, a_t, s_{t+1} | c_T)\} \in [0, 1]$$

这里 $\tau$ 表示策略 $\pi$ 在地形参数 $c_T$ 下生成的轨迹,不要与前面 Domain Randomization 中表示环境参数的 $\xi$ 混用。符号分开后,含义更清楚:DR 的随机变量是"世界参数",Curriculum 的随机变量是"在某个世界参数下跑出来的轨迹"。

基于此,定义地形的"期望度"——一个地形的期望度高,当且仅当其可通行性处于**中等水平**(如 $[0.5, 0.9]$):太容易($Tr > 0.9$)的地形学不到新东西,太难($Tr < 0.5$)的地形导致策略频繁失败而无法学习。训练过程中持续维护一个地形难度分布,使采样到的地形大部分落在"刚好能学"的难度区间内。

地形类型通常由参数化方式生成,包括:(1) Hills 地形——基于 Perlin 噪声的起伏面,由粗糙度、频率和振幅三个参数控制;(2) Steps 地形——随机方块台阶,由块宽度和高度上限控制,造成离散高度突变和陷足;(3) Stairs 地形——规则楼梯,考察周期性抬脚和前后腿协调。每个 episode 都用不同随机种子重新生成地形,防止策略记忆特定地图。

### 腿足 Curriculum 的多维度设计

Curriculum 不只是地形难度,腿足训练通常有多个 curriculum 维度同时推进:

**维度 1:地形难度**

```
Level 0: 完全平地
Level 1: 小幅随机高度扰动(±2 cm)
Level 2: 缓坡(坡度 < 15 度)
Level 3: 小台阶(高度 5 cm)
Level 4: 大台阶(高度 15-20 cm)
Level 5: 混合地形(台阶 + 坡度 + 间隙)
Level 6: 极端地形(大间隙、高台阶、碎石)
```

**维度 2:速度命令范围**

```
Level 0: 站立(v = 0)
Level 1: 慢速(|v| < 0.3 m/s)
Level 2: 中速(|v| < 0.8 m/s)
Level 3: 快速(|v| < 1.5 m/s)
Level 4: 极速(|v| < 2.5 m/s)
```

**维度 3:外部扰动强度**

```
Level 0: 无扰动
Level 1: 轻微推力(v_push < 0.3 m/s)
Level 2: 中等推力(v_push < 0.8 m/s)
Level 3: 强推力(v_push < 1.5 m/s)
```

### legged_gym 的 Curriculum 实现

legged_gym 中,每个环境有独立的 `terrain_level`,根据 episode 成功率自动调整:

```python
def update_terrain_curriculum(self, env_ids):
    """根据 episode 成功率调整地形难度"""
    # 1. 计算每个环境的"距离走得够远" -> 成功
    distance_walked = torch.norm(
        self.root_states[env_ids, :2] - self.env_origins[env_ids, :2],
        dim=1
    )
    success = distance_walked > self.cfg.terrain.curriculum_distance_threshold

    # 2. 成功的环境升级(80%概率),失败的降级(20%概率)
    move_up = success.float()
    move_down = (~success).float()

    self.terrain_levels[env_ids] += 1 * (torch.rand_like(move_up) < 0.8 * move_up)
    self.terrain_levels[env_ids] -= 1 * (torch.rand_like(move_down) < 0.2 * move_down)

    # 3. Clamp 到有效范围
    self.terrain_levels[env_ids] = torch.clamp(
        self.terrain_levels[env_ids], 0, self.cfg.terrain.num_levels - 1
    )
```

关键设计思想:

- **独立进度**: 4096 个环境中,有些在 Level 0,有些已经到 Level 5——保证了**数据多样性**
- **概率升降级**: 不是"成功必定升级",而是概率性的——避免过拟合到特定难度的边界
- **保留低难度环境**: 即使大部分环境在高难度,仍有少量留在低难度——防止策略"忘记"简单场景(catastrophic forgetting)

### ⚠️ 常见陷阱

> ⚠️ **概念误区:Curriculum 升级越快越好**
> **新手想法**: "成功率到 50% 就升级,加速训练。"
> **实际情况**: 升级太快会导致策略在当前难度上**不够稳固**,到了下一难度直接崩溃,形成"学了忘,忘了学"的循环。
> **正确做法**: 等成功率到 80% 再升级,确保策略已经 robust。宁慢勿快。

> ⚠️ **编程陷阱:所有环境同步升级**
> **错误做法**: `if avg_success_rate > 0.8: terrain_level += 1`(全局统一升级)。
> **后果**: 所有 4096 个环境同时跳到新难度,策略突然面对全部困难环境,reward 可能断崖式下降,训练不稳定。
> **正确做法**: 每个环境独立进度,渐进过渡。

### 练习

**练习 63.6.1**: 设计一个三维 Curriculum(地形 $\times$ 速度 $\times$ 扰动)的状态机。当机器人在 Level (2, 1, 0)(中坡、慢速、无扰动)上成功率到 80% 时,下一步应该提升哪个维度?设计一个优先级策略并论证。

**练习 63.6.2**: legged_gym 的 curriculum 只考虑"距离走得够远"作为成功标准。设计一个更综合的成功标准,同时考虑能量效率(累积扭矩)和平滑度(动作变化率),写出数学表达式。

---

Curriculum 解决了"从简单学到复杂"的问题,但还有一个核心挑战:真机只有本体感知(IMU + 关节编码器),没有完美的地形信息。如何让策略在有限信息下也能表现好?这是 Teacher-Student 范式的动机。

## 63.7 Teacher-Student 范式:信息论视角 ⭐⭐⭐

### 动机:信息不对称的困境

真实机器人与仿真环境之间存在根本的**信息不对称**:

| 信息类型 | 仿真中可获取 | 真机可获取 |
|---------|------------|----------|
| 关节位置/速度 | Yes | Yes (编码器) |
| IMU (角速度/加速度) | Yes | Yes |
| **接触力真值** | Yes | 困难(需要力传感器,Ch62) |
| **地形高度图** | Yes | 困难(需要深度相机+处理) |
| **摩擦系数** | Yes | No |
| **机器人精确质量** | Yes | 近似(标称值,有制造误差) |
| **外部扰动力** | Yes | No |

仿真中那些"额外可用"的信息被称为**特权信息**(privileged information)。直接用纯本体感知训练 RL 策略的困难:

1. **信号太弱**: 只有关节和 IMU 数据,无法直接"看到"前方地形
2. **探索困难**: 不知道地形,随机行动容易摔倒,难以收集好的经验
3. **收敛慢**: 策略需要**隐式地学会推断**那些看不到的信息(如从脚底反馈力推断地形硬度)——这比直接"看到"难得多

### Teacher-Student 的两阶段训练

Miki et al. (2022, Science Robotics) 在 ANYmal 上提出的 Teacher-Student 范式优雅地解决了这个问题:

**第一阶段:训练 Teacher(在仿真中)**

Teacher 网络看到**所有信息**(包括特权信息):

$$o_{teacher} = [\underbrace{q, \dot{q}, g, \omega}_{本体感知}, \underbrace{h_{terrain}, f_{contact}, \mu, f_{ext}}_{\textbf{特权信息}}]$$

用 PPO 训练 Teacher。因为信息完整,Teacher 收敛快、策略质量高。整个 Teacher-Student 范式好比驾校教学——教练（Teacher）坐在副驾驶位上能看到所有后视镜和仪表盘（特权信息），在这种全视野条件下先练出完美的驾驶技术；然后学员（Student）只靠前挡风玻璃的有限视野（本体感知），通过模仿教练的操作来学会同样的驾驶技能。

**第二阶段:蒸馏到 Student(监督学习)**

Student 网络只看**本体感知**(真机能拿到的):

$$o_{student} = [q, \dot{q}, g, \omega, a_{t-1}, a_{t-2}, \ldots]$$

注意 Student 的输入中有**历史动作和观测**——这让 Student 可以从"我做了什么动作,世界如何响应"来推断环境特性(类似系统辨识)。

蒸馏 loss:

$$L_{distill} = \| \pi_{student}(o_{student}) - \pi_{teacher}(o_{teacher}) \|^2$$

即让 Student 的输出(动作)尽量匹配 Teacher 的输出。训练数据:在仿真中用 Teacher 策略 rollout,同时记录 $o_{student}$ 和 Teacher 的动作。

这一步看起来像普通 Behavior Cloning,但腿足场景必须警惕两个经典失败模式:

| 失败模式 | 发生原因 | 工程后果 | 常见缓解 |
|----------|----------|----------|----------|
| 累计误差 | 训练数据来自 Teacher 的状态分布,部署时 Student 的小偏差会把系统带到未见过的状态 | 一开始动作只差一点,几步后姿态和接触相位完全偏离训练集 | 用 DAgger / on-policy 蒸馏收集 Student 自己访问到的状态,再由 Teacher 标注 |
| 多峰动作分布 | 同一观测下可能有多个合理动作,如左绕/右绕、抬左脚/抬右脚 | MSE 会学到多个专家动作的平均值,平均动作可能不可执行 | 分阶段任务拆解、混合密度策略、扩散策略,或用 GAIL/AMP 学风格奖励 |

因此 Teacher-Student 不是"把 Teacher rollout 存下来做一次监督学习"这么简单。真正可靠的做法是让 Student 或当前适应模块参与采样,暴露它自己会犯错后到达的状态,再用 Teacher/Privileged Encoder 给这些状态提供监督信号。Ch65 会从 MPC-Net 和 DAgger 角度展开"分布偏移如何累积";Ch70 会把 GAIL/AMP 放到更大的模仿学习和风格化运动框架里讨论。本章先建立接口:只要用了模仿或蒸馏,就必须问数据分布是否覆盖部署时状态,以及动作分布是否存在多峰。

### 信息论解释:为什么 Teacher-Student 有效?

从信息论角度,Teacher-Student 的本质是**信息压缩和传递**:

**Teacher 阶段**: Teacher 学到了一个映射 $f: (o_{proprio}, o_{priv}) \mapsto a$。这个映射隐式地包含了"在给定完整信息下的最优行为"。

**Student 阶段**: Student 学到了一个映射 $g: o_{proprio} \mapsto a$。由于 $g$ 的输入比 $f$ 少(缺少 $o_{priv}$),Student 必须通过**推断** $o_{priv}$ 来近似 $f$ 的输出。

信息论的视角:设 $I(o_{proprio}; o_{priv})$ 是本体感知与特权信息之间的互信息(mutual information)。如果互信息为零(两者完全独立),Student 无法从本体感知推断任何特权信息,蒸馏必然失败。如果互信息大(如关节力矩包含地形硬度信息),Student 可以从本体感知中**推断**特权信息。

Teacher-Student 的隐式假设:**本体感知与特权信息之间存在足够的互信息**。在腿足场景中,这个假设通常成立:

- 脚底接触力 $\leftrightarrow$ 关节电流(电机扭矩 $\propto$ 接触力)
- 地形坡度 $\leftrightarrow$ 身体倾斜(IMU 可测)
- 摩擦系数 $\leftrightarrow$ 打滑时的速度变化(关节速度可测)

### 变种:RMA (Rapid Motor Adaptation)

Kumar et al. (2021, RSS) 提出了 RMA(Rapid Motor Adaptation)的变体。与 Miki 的整体蒸馏不同,RMA 在网络结构中显式地引入了"适应模块":

```
Teacher 架构:
  Base Policy: o_proprio + e_priv  --> action
  e_priv = Encoder(o_priv)  <-- 特权信息编码为向量

Student 架构(部署):
  Base Policy: o_proprio + e_hat  --> action   (同一个 Base Policy)
  e_hat = AdaptationModule(history)  <-- 从历史推断特权编码
```

RMA 的关键创新:Base Policy 是**共享的**(Teacher 和 Student 使用同一个 Base Policy 网络)。区别只在于特权编码的来源——Teacher 用 Encoder 从真值编码,Student 用 Adaptation Module 从历史观测推断。这使蒸馏更聚焦:只需要训 Adaptation Module 去匹配编码后的 latent $z_t = \mu(e_t)$（而非原始特权参数 $e_t$）,不需要蒸馏整个策略。

RMA 与 Miki 方法的核心区别值得强调:Miki 的方法是**动作蒸馏**——Student 同时模仿 Teacher 的动作输出和环境编码;而 RMA 只让 Adaptation Module 模仿环境编码 $z_t$,将动作的产生完全留给 Base Policy。这种模块化设计带来了两个直接收益:(1) Adaptation Module 只负责"辨识/适应",Base Policy 只负责"控制",两者可以以不同频率运行(如 Adaptation Module 10 Hz,Base Policy 100 Hz);(2) 自适应模块不追求恢复真实物理参数(如精确的摩擦系数值),而是恢复一个"足够用"的控制表征——只要能指导正确的动作即可。

RMA 还有一个重要的训练技巧:在训练 Adaptation Module 时,使用**在线策略数据**(类似 DAgger 算法)——用当前(可能不完美的)Adaptation Module 产生的 $\hat{z}_t$ 驱动 Base Policy 采集轨迹,然后用真实的 $z_t$ 作为标签训练。这种做法确保 Adaptation Module 在训练时就见过因自身预测不准确而导致的"偏离最优轨迹"的数据,从而在部署时更加鲁棒。如果只用 Teacher 的完美轨迹数据训练 Adaptation Module,则 Student 在部署中一旦出现偏差就无法处理(分布偏移问题)。

> 💡 **RMA 关键工程细节**
>
> 1. **预测 latent 而非物理参数**：适应模块输出的是环境编码向量 $\hat{z}_t$（对应 Teacher Encoder 输出的 $z_t$），而非物理参数本身（质量、摩擦系数等）。这是因为不同的物理参数组合可能产生相同的行为效果——适应模块只需要编码"行为应如何改变"，不需要精确辨识每个物理量。
>
> 2. **异步运行频率**：适应模块运行在 ~10 Hz（基于 50 步观测历史），策略网络运行在 ~100 Hz。适应模块不需要高频更新——环境特性（摩擦、质量）变化远慢于控制周期。
>
> 3. **on-policy 数据采集**：适应模块的训练数据必须由**当前策略**（而非旧策略）采集，否则观测历史的分布与部署时不匹配。这本质上是一个 DAgger 式的在线蒸馏过程。

> 💡 **为什么 TCN 而非 MLP：时序编码器的必要性**
>
> Miki et al. (2022) 使用**时序卷积网络（TCN）**而非简单 MLP 来处理观测历史，原因在于：
>
> 1. **因果卷积**：TCN 的卷积核只看过去、不看未来，天然适合实时控制（MLP 拼接历史在数学上等价但参数效率低）
> 2. **多尺度时序特征**：通过扩张卷积（dilated convolution），TCN 用少量层数就能覆盖长时间窗口——底层捕捉快速接触事件（~10 ms），高层捕捉缓慢环境变化（~1 s）
> 3. **参数共享**：同一组卷积核在所有时间步上共享，比 MLP（为每个时间步分配独立权重）参数量小一个量级
>
> TCN 从本体感知历史中推断的隐变量包括：
> - 当前接触状态（哪只脚触地）——通过力矩/速度突变模式
> - 地面摩擦特性——通过打滑时的速度-力矩关系
> - 负载/质量变化——通过同样动作下加速度的差异
> - 地形坡度——通过重力分量在 IMU 中的投影变化
>
> 这些信息在单帧观测中不可见（或噪声过大），必须从时序模式中提取。

> 💡 **"辅助轮骑自行车"：参考轨迹 + 策略残差**
>
> Tan et al. (2018) 的开创性设计：先用运动捕捉或轨迹优化生成参考运动（辅助轮），RL 策略只需学习残差修正（骑自行车）。这大幅降低了探索难度——策略从"学会走路"简化为"学会微调已有的走路方式"。代价是参考轨迹限制了运动多样性，后续工作（如 RMA）通过去除参考轨迹、完全依赖奖励塑形来换取更大的行为空间。

### Asymmetric Actor-Critic：更简单的替代方案

如果不想用两阶段训练,可以用 Asymmetric Actor-Critic(不对称演员-评论家):

- **Actor**: 输入 = 本体感知(部署时使用)
- **Critic**: 输入 = 本体感知 + 特权信息(仅训练时使用)

Critic 信息更完整,可以给出更准确的 value 估计,从而让 Actor 的 PPO 更新更稳定。部署时只用 Actor。

| 方法 | 训练复杂度 | 策略质量 | 部署简洁性 |
|------|-----------|---------|-----------|
| 直接训(纯本体感知) | 低 | 较差 | 高 |
| Asymmetric Actor-Critic | 中(一阶段) | 中等 | 高(只部署 Actor) |
| Teacher-Student (Miki) | 高(两阶段) | 高 | 高(只部署 Student) |
| RMA (Kumar) | 高(两阶段) | 高 | 中(需要 Adaptation Module) |

### 补充:事后经验回放(HER)与稀疏奖励场景

在某些足式任务中(如精确的足端定位或物体操纵),奖励可能非常稀疏——只有完成目标才能获得正向信号。这类场景下标准 PPO 几乎无法收敛,因为随机探索极难触及目标状态。

事后经验回放(Hindsight Experience Replay, HER)是一种数据增强技术,核心思想是"从失败中学习":即使智能体未能达成既定目标 $A$,它实际上也到达了某个位置 $B$。HER 通过事后将目标替换为 $B$,把这次"失败"的轨迹重新标记为"成功",从而将稀疏奖励转化为稠密的学习信号。这种机制在早期的 sim-to-real 域随机化工作(如 OpenAI 的机械臂操纵)中被证明是训练成功的关键之一——在极端困难的随机化物理参数下,HER 保证了智能体仍能获得正向反馈。

对于腿足 RL 的主流任务(速度跟踪),奖励通常是稠密的,因此 HER 不是必需的。但在设计涉及精确目标到达或物体交互的高层策略时,HER 是值得了解的重要工具。

### 补充:真机直接训练与自动温度调节

尽管 sim-to-real 是腿足 RL 的主流范式,也有研究探索了直接在真实机器人上训练的路线。其中一个关键挑战是:on-policy 算法(如 PPO)的样本效率极低,而 off-policy 算法(如 SAC)虽然样本高效,但其温度参数 $\alpha$ 对性能极为敏感——不同任务甚至同一任务的不同训练阶段,最优的 $\alpha$ 值都不同,手动调参在真机上几乎不可行。

解决方案是将温度参数从超参数转变为可学习的对偶变量:将策略熵视为约束条件而非目标的一部分,即要求策略的期望熵不低于目标下界 $\mathcal{H}$,然后通过拉格朗日对偶法自动调节 $\alpha$。这一技巧(Haarnoja et al., 2019)使 SAC 在真机上的部署门槛显著降低,代表了"不依赖仿真"的另一条技术路线。不过,这条路线受限于真机训练的数据采集速度和安全风险,目前工业界仍以仿真训练为主。

### ⚠️ 常见陷阱

> ⚠️ **概念误区:认为 Teacher-Student 只是"降级"**
> **新手想法**: "Student 信息更少,性能一定比 Teacher 差很多。"
> **实际情况**: Student 的性能通常只比 Teacher 低 5-15%。原因:腿足行走的大部分决策不需要精确的地形信息——"感觉脚底变软了就小心走"这种粗略推断就够了。
> **类比**: 人类闭眼走路不会立刻摔倒——因为从脚底触觉和身体平衡感就能推断很多。

> ⚠️ **编程陷阱:蒸馏时不冻结 Teacher 网络**
> **错误做法**: 蒸馏阶段同时更新 Teacher 和 Student。
> **后果**: Teacher 的输出会变化,Student 追逐一个移动目标(moving target),训练不稳定。
> **正确做法**: 蒸馏阶段 Teacher 完全冻结(`requires_grad_(False)`)。

> ⚠️ **思维陷阱:所有任务都用 Teacher-Student**
> **新手想法**: "所有腿足 RL 都应该用 Teacher-Student。"
> **实际情况**: 如果任务不需要感知(如平地 trot),直接用本体感知训练+Asymmetric AC 就够了。Teacher-Student 的价值在于**感知密集型任务**(崎岖地形、楼梯攀爬、复杂交互)。多一个阶段的训练意味着更多的调参工作和更长的开发周期。
> **判断标准**: 如果 Asymmetric Actor-Critic 就能达到满意效果,就不需要两阶段蒸馏。

### 练习

**练习 63.7.1**: 设计一个简化的 Teacher-Student 实验。Teacher 输入:本体感知 + 摩擦系数(单个标量)。Student 输入:纯本体感知。训练后测试:在不同摩擦系数上,Student 的性能与 Teacher 差多少?

**练习 63.7.2**: RMA 的 Adaptation Module 输入是"历史观测和动作的滑动窗口"。窗口长度对推断精度有什么影响?太短(如 5 步)和太长(如 200 步)分别有什么问题?

**练习 63.7.3**: 从信息论角度分析:如果机器人的脚底没有力传感器(只有关节电流间接估计接触力,Ch62 讲过),Teacher-Student 的蒸馏效果会如何变化?互信息 $I(o_{proprio}; o_{priv})$ 是增大还是减小?

---

前面七节覆盖了腿足 RL 训练栈的核心组件。下一节把 legged_gym 的代码结构做一次系统精读,将概念对应到具体实现。

## 63.8 legged_gym 代码精读与完整训练流程 ⭐⭐

### 动机:为什么要精读源码?

legged_gym 是腿足 RL 的"事实标准"代码库。虽然新项目建议使用 IsaacLab,但 legged_gym 的代码结构和设计思想至今仍是理解腿足 RL 工程的最佳教材——仅约 3000 行核心代码,但覆盖了环境、奖励、DR、Curriculum 的完整流水线。很多 IsaacLab 的设计直接继承自 legged_gym。

### 项目结构与模块分工

```
legged_gym/
├── envs/
│   ├── base/
│   │   ├── legged_robot.py         # 通用腿足环境基类 (~1500 行)
│   │   │   ├── create_sim()         # 创建 IsaacGym 仿真场景
│   │   │   ├── step()               # 核心:并行执行 4096 环境
│   │   │   ├── compute_observations()
│   │   │   ├── compute_reward()     # 向量化奖励计算
│   │   │   ├── reset_idx()          # 部分环境 reset
│   │   │   └── _push_robots()       # 随机推力(DR)
│   │   │
│   │   └── legged_robot_config.py   # 配置类(所有参数集中管理)
│   │
│   ├── anymal_c/                     # ANYmal 特化配置
│   └── go1/                          # Go1 特化配置
│
├── scripts/
│   ├── train.py                      # 训练入口
│   └── play.py                       # 推理可视化
│
└── utils/
    └── task_registry.py              # 任务注册表
```

### 核心函数精读:`step()` 的并行执行

```python
def step(self, actions):
    """在 GPU 上并行执行 4096 个环境的一步"""
    # 1. 动作处理: clip + scale
    # actions shape: (4096, 12) — 4096 个环境 x 12 个关节
    clip_actions = self.cfg.normalization.clip_actions
    self.actions = torch.clip(actions, -clip_actions, clip_actions)

    # 2. PD 控制: target = default_pos + action * scale
    # action 被解释为"关节位置偏移量",不是直接的扭矩
    target = self.default_dof_pos + self.actions * self.cfg.control.action_scale

    # 3. 物理仿真: 调用 IsaacGym 的 GPU 物理引擎
    # decimation: 每个 RL 步包含多个物理步
    # (如 RL 50Hz, 物理 200Hz -> decimation=4)
    for _ in range(self.cfg.control.decimation):
        torques = self._compute_torques(target)  # PD -> 扭矩
        self.gym.set_dof_actuation_force_tensor(
            self.sim, gymtorch.unwrap_tensor(torques)
        )
        self.gym.simulate(self.sim)               # GPU 物理仿真一步
        self.gym.refresh_dof_state_tensor(self.sim)

    # 4. 后处理: 计算观测、奖励、done
    self.post_physics_step()  # -> compute_reward() + compute_observations()

    # 5. 部分 reset: 只 reset 摔倒/超时的环境,其他继续
    env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
    if len(env_ids) > 0:
        self.reset_idx(env_ids)

    return self.obs_buf, self.rew_buf, self.reset_buf, self.extras
```

关键设计点:

- **`decimation`**: RL 频率(50 Hz)和物理频率(200 Hz)解耦。每个 RL 步内跑 4 个物理步,提高物理精度同时不增加 RL 的 sample 数量
- **`reset_idx`**: 只 reset 摔倒的环境,其他环境继续运行。4096 个环境中,每步可能只有 10-50 个需要 reset——这保证了高利用率
- **全部操作在 GPU 上**: 没有任何 CPU 调用,所有 tensor 操作都是批量的 GPU 运算

### 奖励计算的向量化

```python
def compute_reward(self):
    """向量化计算所有奖励项——不是 for 循环,而是 tensor 运算"""
    self.rew_buf[:] = 0.  # 清零

    for name, scale in self.reward_scales.items():
        # 每个奖励项是一个函数,输入/输出都是 (4096,) 的 tensor
        reward_fn = getattr(self, '_reward_' + name)
        reward = reward_fn() * scale  # (4096,) x scalar
        self.rew_buf += reward         # 累加到总奖励

        # 记录每个奖励项的均值(用于 TensorBoard 监控)
        self.episode_sums[name] += reward
```

核心思想:每个 `_reward_xxx()` 函数操作的都是 `(4096, ...)` 的 tensor,一次计算所有环境的奖励。没有 Python for 循环遍历环境,纯 GPU tensor 操作。

### Asymmetric Actor-Critic 的实现

```python
class ActorCritic(nn.Module):
    """rsl_rl 的 Actor-Critic 网络"""

    def __init__(self, num_actor_obs, num_critic_obs, num_actions,
                 actor_hidden_dims=[256, 256, 256],
                 critic_hidden_dims=[256, 256, 256]):
        super().__init__()

        # Actor: 输入 = 本体感知 (如 45 维)
        self.actor = build_mlp(num_actor_obs, actor_hidden_dims, num_actions)

        # Critic: 输入 = 本体感知 + 特权信息 (如 45 + 73 = 118 维)
        self.critic = build_mlp(num_critic_obs, critic_hidden_dims, 1)

        # 动作标准差(可学习参数)
        self.std = nn.Parameter(0.3 * torch.ones(num_actions))

    def act(self, observations):
        """Actor 推理(训练和部署都用)"""
        mean = self.actor(observations)
        distribution = Normal(mean, self.std)
        action = distribution.sample()
        log_prob = distribution.log_prob(action).sum(dim=-1)
        return action, log_prob

    def evaluate(self, critic_observations):
        """Critic 估值(仅训练时用)"""
        return self.critic(critic_observations)
```

关键:Actor 和 Critic 的**输入维度不同**——Actor 只有本体感知(45 维),Critic 有更完整的信息(118 维)。部署时只导出 Actor。

### 完整训练 Pipeline

```
              完整训练流程
┌────────────────────────────────────────────┐
│ 1. 配置                                     │
│    环境 config + RL config + 奖励权重        │
│    4096 并行环境 + PPO 超参数                │
├────────────────────────────────────────────┤
│ 2. 训练循环 (4000-8000 iterations)          │
│    ┌──────────────────────────────────┐     │
│    │ a. Rollout: 4096 env x 24 steps │     │
│    │    = 98,304 transitions/iter    │     │
│    │                                  │     │
│    │ b. GAE 计算 advantage           │     │
│    │                                  │     │
│    │ c. PPO 更新 (5 epochs x 4 mini) │     │
│    │                                  │     │
│    │ d. Curriculum 更新(地形难度)    │     │
│    │                                  │     │
│    │ e. 日志: reward/KL/FPS/etc.     │     │
│    └──────────────────────────────────┘     │
│                                             │
│ 3. Checkpoint 保存                          │
│    model_XXXX.pt (每 200 iterations)        │
├────────────────────────────────────────────┤
│ 4. 评估                                     │
│    play.py 加载 checkpoint, 可视化行为       │
│    观察: 步态自然性/速度跟踪/地形适应        │
├────────────────────────────────────────────┤
│ 5. 迭代: 修改奖励权重/DR 配置 -> 回步骤 1  │
└────────────────────────────────────────────┘
```

**TensorBoard 监控指标**:

| 指标 | 含义 | 健康范围 |
|------|------|---------|
| `mean_reward` | 平均 episode 奖励 | 持续上升,最终收敛 |
| `mean_episode_length` | 平均 episode 长度 | 从短到长(说明不再早期摔倒) |
| `approx_kl` | PPO 的近似 KL 散度 | 0.005 ~ 0.03 |
| `policy_loss` | Actor loss | 从大到小,接近 0 |
| `value_loss` | Critic loss | 持续下降 |
| `FPS` | 每秒仿真帧数 | RTX 4090: 40k-80k |
| `curriculum/terrain_level` | 平均地形难度 | 缓慢上升 |

> 💡 **评价指标：不要只看 reward 曲线**
>
> 仅用 reward 曲线评价腿足 RL 策略是不够的——reward 可以通过"抖动"等非物理行为获得高分但无法部署。建议补充以下量化指标：
>
> | 指标 | 含义 | 为什么重要 |
> |------|------|-----------|
> | **CoT (Cost of Transport)** | 单位质量单位距离的能耗 | 反映运动效率，值越小越好 |
> | **接触冲击力峰值** | 足端着地瞬间的力峰值 | 过大→硬件损伤/噪声大 |
> | **命令跟踪误差** | 实际速度 vs 期望速度 | 核心控制性能 |
> | **延迟鲁棒性** | 增加观测/执行延迟后的性能衰减 | 真机必然有延迟 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:修改 legged_gym 代码时用 Python for 循环遍历环境**
> **错误做法**: `for env_id in range(4096): rewards[env_id] = compute_single(env_id)`
> **后果**: Python for 循环在 4096 个环境上极慢(每步 ~100 ms vs tensor 操作 ~0.1 ms),训练速度降低 1000 倍。
> **正确做法**: 所有计算都用 PyTorch tensor 操作(广播、索引、归约),保持 GPU 并行性。

> ⚠️ **编程陷阱:保存 checkpoint 时只存 Actor**
> **错误做法**: 只保存 `actor.state_dict()`,不保存 Critic 和 optimizer。
> **后果**: 无法从 checkpoint 恢复训练(Critic 状态和 optimizer 动量丢失)。虽然部署只需要 Actor,但训练恢复需要完整状态。
> **正确做法**: `torch.save({'actor': ..., 'critic': ..., 'optimizer': ..., 'iter': ...}, path)`。

> ⚠️ **思维陷阱:训练时间越长越好**
> **新手想法**: "训 72 小时肯定比 24 小时好。"
> **实际情况**: 腿足 RL 通常在 3000-6000 iterations 后收敛。过度训练可能导致策略 overfit 到训练地形的特定特征(如某种台阶高度的精确步法),泛化性反而下降。
> **判断方法**: 监控 `mean_reward` 曲线,如果连续 1000 iterations 不再上升,就可以停止。

> ⚠️ **概念误区:认为 legged_gym 是唯一选择**
> **新手想法**: "所有腿足 RL 项目都必须 fork legged_gym。"
> **实际情况**: legged_gym 基于 IsaacGym(已 legacy)。2025/2026 年的新项目应使用 IsaacLab 原生环境,或者使用 fan-ziqi 的 `robot_lab`(legged_gym 概念到 IsaacLab 的迁移封装)。
> **正确策略**: 用 legged_gym 学习概念和架构,用 IsaacLab 做实际项目。

### 练习

**练习 63.8.1**: 精读 `legged_robot.py` 的 `_reward_tracking_lin_vel()` 函数。它的返回值 shape 是什么?为什么用 `exp(-error / sigma**2)` 而不是 `-error**2`?

**练习 63.8.2**: 精读 `reset_idx()` 函数。当一个环境 reset 时,哪些状态量被重置?观测 buffer 是否也被 reset?如果不 reset 观测 buffer,会导致什么问题?(提示:考虑 `last_action` 观测项)

**练习 63.8.3**: 配置并训练一个 Go2 flat terrain trot 策略(使用 IsaacLab 或 legged_gym)。记录完整的训练曲线(reward、episode length、KL),分析何时收敛。做 3 个奖励 ablation(删除 action_rate / 放大 torques / 删除 lin_vel_z),用 TensorBoard 对比训练曲线和行为差异。

---

## 常见故障与排查

腿足 RL 训练涉及仿真环境、奖励函数、网络优化和 sim-to-real 迁移等多个层面，调试往往比传统控制更难定位根因。以下是训练和部署中最常见的故障场景。

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| Reward 曲线不涨或持续下降 | 学习率过高/过低、奖励缩放失衡（正则项淹没追踪项）、GAE 参数不当 | 1. 打印各奖励分项均值，检查追踪 vs 正则量级比 2. 将学习率减半重试 3. 检查 $\gamma$ 和 $\lambda_{\text{GAE}}$ 是否在合理范围（0.99 / 0.95） | 63.3, 63.4 |
| Reward hacking（奖励高但行为异常，如原地抖动或单脚跳） | 奖励函数存在漏洞——智能体找到了高回报但非预期的策略 | 1. 渲染策略行为确认异常 2. 逐项检查哪个奖励分量贡献最大 3. 加入针对异常行为的惩罚项（如 `action_rate`、`feet_air_time`） 4. 用 $\exp(-e^2/\sigma^2)$ 替代线性奖励限制饱和 | 63.4 |
| Sim-to-real gap 过大（仿真表现好，真机摔倒） | Domain Randomization 范围未覆盖真实参数、或观测延迟未建模 | 1. 测量真机摩擦系数/质量/电机延迟，确认在 DR 范围内 2. 加入 1-2 步动作延迟随机化 3. 在仿真中测试 DR 极端参数下策略是否仍存活 | 63.5 |
| 训练收敛但策略在复杂地形上失败 | Curriculum 升级过快或地形难度分布不均 | 1. 检查各地形类别成功率，确认是否有类别低于 50% 2. 降低升级阈值（如从 80% 改为 90%） 3. 增加困难地形比例或添加降级机制 | 63.6 |
| Teacher-Student 蒸馏后 Student 性能严重退化 | Student 观测空间信息不足以重建 Teacher 隐状态、或蒸馏损失权重不当 | 1. 打印 Teacher 和 Student 的动作 MSE 2. 检查 Student 观测历史长度是否足够（通常需 10-50 步） 3. 先冻结 Adaptation Module 单独训练 Student Policy，再联合微调 | 63.7 |

### Reward 调优实战技巧——从 20+ 个项目中总结的经验 ⭐⭐⭐

**技巧 1：先跑"纯追踪"基线**

在加入任何正则化项之前，先只用速度追踪奖励训练一版策略。这个策略可能行为很丑（抖动、能耗大），但它建立了"追踪项的基准 reward 量级"。后续加入正则项时，确保正则项的量级不超过追踪项的 50%——否则 RL 会优先"不动"（正则项最优）而非"追踪命令"。

**技巧 2：$\sigma$ 参数是 reward shaping 最强的旋钮**

对于 $r = \exp(-\|e\|^2 / \sigma^2)$ 形式的奖励，$\sigma$ 决定了"多大的误差仍能获得正回报"。$\sigma$ 太小则只有完美追踪才有 reward（训练早期无信号），太大则粗糙追踪也有高 reward（训练后期不精细）。

| 奖励项 | 推荐 $\sigma$ | 直觉 |
|--------|-------------|------|
| 线速度追踪 | 0.25 m/s | 误差 0.25 m/s 时 reward 降到 $1/e$ |
| 角速度追踪 | 0.25 rad/s | 允许约 14 度/秒偏差 |
| 躯体高度 | 0.05 m | 5cm 偏差即显著惩罚 |
| 关节加速度 | 按量级定，通常 1.0 | 正则化不宜太严 |

**技巧 3：用 TensorBoard 的 reward 分项图诊断**

训练时记录每个奖励分量的均值，不只是总 reward。当总 reward 卡住时，逐项检查：

- 如果追踪项在涨但正则项在跌 → 正则权重太大，降低
- 如果所有项都不变 → 学习率太小或 GAE 参数不当
- 如果某项突然跳变 → 策略发现了新行为模式，可能是 reward hacking

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关键公式/概念 |
|--------|---------|------|-------------|
| 63.1 腿足 RL 差异 | GPU 并行仿真、sim-to-real gap、专用训练栈 | ⭐ | — |
| 63.2 IsaacGym→Lab | GPU 原生仿真、Manager-Based 架构、性能数据 | ⭐⭐ | 4096 并行, 10 亿步/天 |
| 63.3 rsl_rl PPO | GAE 完整推导、clipped objective、训练超参数 | ⭐⭐ | $\hat{A}_t = \sum (\gamma\lambda)^l \delta_{t+l}$ |
| 63.4 奖励设计 | 追踪+正则框架、reward hacking、权重平衡 | ⭐⭐⭐ | $r = \exp(-\|e\|^2/\sigma^2)$ |
| 63.5 Domain Rand. | 贝叶斯解释、典型配置、ADR | ⭐⭐⭐ | $\max_\theta \mathbb{E}_{\xi \sim p(\xi)}[J(\theta,\xi)]$ |
| 63.6 Curriculum | 多维 Curriculum、独立进度、升降级规则 | ⭐⭐ | 80% 成功率 $\to$ 升级 |
| 63.7 Teacher-Student | 信息不对称、两阶段蒸馏、RMA、信息论 | ⭐⭐⭐ | $L = \|\pi_S - \pi_T\|^2$ |
| 63.8 代码+流程 | legged_gym 精读、Asymmetric AC、训练 pipeline | ⭐⭐ | 98,304 步/iteration |

---

## 累积项目:本章新增模块

**四足站立控制器进度更新**:

```
Ch47: 加载 URDF + Pinocchio 建模           [完成]
Ch49: 正/逆运动学 + 接触 Jacobian           [完成]
Ch50: QP 求解器集成                         [完成]
Ch52: 摩擦锥约束                            [完成]
Ch53: WBC 实现 (加权 QP)                    [完成]
Ch55: OCS2 MPC 集成                         [完成]
Ch61: 实时部署框架                          [完成]
Ch62: 硬件接口 + Unitree SDK                [完成]
Ch63: RL 策略训练                           <-- 本章新增
  |-- IsaacLab Go2 环境配置
  |-- rsl_rl PPO 训练脚本
  |-- 自定义奖励函数(追踪 + 正则)
  |-- DR 配置(质量/摩擦/延迟/噪声)
  |-- Curriculum 地形配置
  |-- TensorBoard 训练监控
  |-- 导出 .pth checkpoint
```

**本章新增的具体任务**:
1. 在 IsaacLab 中配置 Go2 flat terrain 环境(4096 并行)
2. 使用 rsl_rl 的 PPO 训练 trot 策略(约 2-4 小时)
3. 做 3 个奖励 ablation 实验并记录行为差异
4. 配置 DR(摩擦 + 质量 + 推力),对比有/无 DR 的鲁棒性
5. 导出训好的 checkpoint(`.pth`),为 Ch64 部署做准备

---

## 延伸阅读

### 必读经典 ⭐⭐

| 资料 | 类型 | 说明 |
|------|------|------|
| Schulman et al. (2017) "Proximal Policy Optimization Algorithms", arXiv:1707.06347 | 论文 | PPO 原始论文 |
| Hwangbo J., Lee J., Dosovitskiy A. et al. (2019) "Learning agile and dynamic motor skills for legged robots", Science Robotics 4(26), eaau5872 | 论文 | ANYmal RL 奠基之作,首次在中型四足上实现 sim-to-real RL 部署 |
| Miki T. et al. (2022) "Learning robust perceptive locomotion for quadrupedal robots in the wild", Science Robotics 7(62), eabk2822 | 论文 | Teacher-Student + 感知,ANYmal 完成阿尔卑斯山徒步 |

### 核心工作 ⭐⭐⭐

| 资料 | 类型 | 说明 |
|------|------|------|
| Kumar A., Fu Z., Pathak D., Malik J. (2021) "RMA: Rapid Motor Adaptation for Legged Robots", RSS | 论文 | Adaptation Module 在线自适应,A1 真机验证 |
| Margolis G. B., Agrawal P. (2022) "Walk These Ways: Tuning Robot Control for Generalization with Multiplicity of Behavior", CoRL | 论文 | 行为多样性,一个策略多种步态,Go1 部署 |
| Rudin N., Hoeller D., Reist P., Hutter M. (2022) "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning", CoRL | 论文 | legged_gym 训练框架,20 分钟训好 ANYmal |
| Mittal M. et al. (2023) "Orbit: A Unified Simulation Framework for Interactive Robot Learning Environments", RA-L | 论文 | IsaacLab 前身 |

### 前沿进展 ⭐⭐⭐⭐

| 资料 | 类型 | 说明 |
|------|------|------|
| Hoeller D. et al. (2024) "ANYmal parkour: Learning agile navigation for quadrupedal robots", Science Robotics | 论文 | 四足跑酷,感知+RL 的极致展示 |
| Zhuang Z. et al. (2023) "Robot Parkour Learning", CoRL | 论文 | 视觉 RL parkour |
| rsl_rl GitHub (`leggedrobotics/rsl_rl`) | 代码 | ETH RSL 的 RL 训练库,多 GPU 支持 |
| IsaacLab 官方文档 (`developer.nvidia.com/isaac/lab`) | 文档 | NVIDIA 官方,Isaac Lab 3.x/3.0 beta 需以当前文档为准 |
| robot_lab (`fan-ziqi/robot_lab`) | 代码 | legged_gym 概念到 IsaacLab 的迁移封装 |

---

本章从概念到实现,系统构建了腿足 RL 训练栈的完整知识体系。训好的策略存在 PyTorch checkpoint 里——但这只是一个 Python 对象。要在真实机器人上以 1 kHz 运行,必须解决一个核心工程问题:**如何把 Python 模型变成 C++ 可执行的实时推理引擎?** 这正是 Ch64 的主题——RL 策略的 C++ 部署,从 TorchScript/LibTorch 到 ONNX Runtime,从零拷贝数据转换到安全降级设计。
