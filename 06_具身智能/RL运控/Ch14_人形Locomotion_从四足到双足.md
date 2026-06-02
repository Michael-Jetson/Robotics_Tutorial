# Ch14 | 人形 Locomotion：从四足到双足

> **本章定位**：Part IV（单形态实战）第二章。Ch13 建立了四足速度跟踪的完整工程方法论——链路阅读法、四层 reward 框架、分阶段验证、双框架对比。本章把同样的方法论迁移到人形机器人（Unitree G1/H1），但你会发现：四足的很多工程直觉在双足上不再成立，甚至会误导。本章通过系统性对比，建立人形控制的正确心智模型。
>
> **参考**：🔧 mjlab G1 velocity · 🔧 Isaac Lab H1 velocity · ✅ HOVER（ICRA'25）· ✅ humanoid-gym（RSS'24）· ✅ HoST（RSS'25）
>
> **机器人**：G1/H1 · **累积项目**：**B**

---

## 前置自测

📋 **答不出 ≥ 3 题 → 先回前置章节复习**

> 本章直接依赖 Ch13 的四足速度跟踪经验。如果你还没有在 Ch13 中跑通 Go1/Go2 的 flat + rough velocity task，强烈建议先完成。

1. **[Ch13]** mjlab velocity task 的 reward 四层框架是什么？每一层的调参策略有什么不同？如果把所有 reward 权重设为 1.0 会发生什么？
2. **[Ch13]** 链路阅读法的完整链条是什么？为什么精读一个 task 不应该从 reward 函数开始？
3. **[Ch09]** Asymmetric actor-critic 的设计动机是什么？critic 看到的 privileged 信息应该满足什么条件？
4. **[物理]** 什么是"支撑多边形"（support polygon）？为什么双足机器人的支撑多边形比四足小一个数量级？
5. **[动力学]** 什么是角动量？上肢挥动如何影响躯干的姿态？如果你走路时双臂绑在身上，走路会变得更困难吗？为什么？

## 本章目标

学完本章后，你应该能够：

1. **系统性对比**四足和双足人形在支撑面、质心高度、角动量、上肢和接触方面的本质差异，解释为什么四足经验不能直接迁移
2. **解读** G1 的 29 个 actuated joints 的分组，计算每个关节的 action scale，理解 variable posture reward 的设计动机
3. **用 Reward 四层框架分析人形特有 reward**——angular momentum penalty、variable posture reward、upper-body 约束 vs 自然摆臂的权衡
4. **独立跑通 G1 flat velocity task**，从 zero agent 到 large train 的分阶段验证，诊断人形特有的失败模式
5. **精读 HOVER 的 mask-conditioned distillation 架构**，理解如何用一个策略支持多种控制模态
6. **精读 HoST 的 multi-critic 跌倒恢复**，理解如何将其集成到 locomotion pipeline 中作为安全兜底
7. **在 Isaac Lab 和 mjlab 中执行 sim-to-sim 验证**，建立人形策略跨引擎鲁棒性的评估方法

---

从 Ch13 的四足速度跟踪到人形，看起来只是"换了个机器人"。但这个"换"会导致你在 Ch13 中建立的大部分工程直觉失效。本节先建立清晰的差异认知，避免你带着四足思维去调人形参数——这是人形 RL 新手最常犯的错误。

## 14.1 人形 vs 四足本质差异 ⭐⭐

> **这一节解决什么问题**：从四足到双足，哪些工程直觉会失效？为什么 G1 的控制问题在本质上更难？

### 动机：为什么四足经验不能直接迁移

最常见的反面案例是把四足 velocity task 的配置直接搬到 G1。表面上看，G1 的 velocity task 和 Go1 共享同一个 `velocity_env_cfg.py` base cfg——observation terms、reward terms、termination conditions 看起来都类似。但如果你不做任何修改地运行，G1 会在几步之内摔倒。

这不是 bug——这是物理。

四足有四个潜在支撑点，trot 步态下始终有两条对角腿支撑。双足通常只有一只脚或两只脚短暂支撑。四足的侧向误差可以由另一侧两条腿缓冲；G1 的侧向误差会直接变成躯干 roll、髋 roll 和脚掌边缘接触问题。四足上轻微的脚滑可能只是 tracking reward 下降；G1 上轻微脚滑可能立即导致 pelvis 横移、torso 俯仰和终止。

### 支撑多边形：问题的物理根源

支撑多边形（support polygon）是所有接触点的凸包。只有当质心的垂直投影落在支撑多边形内时，机器人才是静态稳定的。

| 步态模式 | 接触点数 | 支撑面积（典型值） | 容错空间 |
|---------|---------|-----------------|---------|
| 四足 trot（Go1） | 2（对角） | ~0.12 m²（0.4m × 0.3m） | 大——质心偏移 5cm 仍稳定 |
| 四足 walk（Go1） | 3-4 | ~0.18 m² | 更大 |
| **双足双支撑（G1）** | 2（并排） | ~0.05 m²（0.25m × 0.2m） | 中等 |
| **双足单支撑（G1）** | 1 | **~0.025 m²**（0.25m × 0.1m） | **极小——质心偏移 2cm 就失衡** |

**跨领域类比**：从四足到双足的跨越，就像从宽桥走到钢丝上。宽桥（四足）上你可以步态不太优美但不会掉下去——大支撑面给了大量容错空间。钢丝（双足）上每一步都必须精确——质心稍微偏离就会失衡。更关键的是，钢丝上你必须主动用手臂和腰部来调整重心——这就是为什么人形控制必须考虑上肢和角动量，而四足通常可以忽略。

### 倒立摆不稳定性

从控制论角度，双足行走可以近似为一个**倒立摆**（inverted pendulum）。质心在支撑脚上方，但由于高质心（G1 pelvis 高度 ~0.76m）和窄支撑面（脚掌宽度 ~0.1m），系统是本征不稳定的——即使站着不动，也需要主动控制来维持平衡。

四足机器人的 trot 步态不是倒立摆——它更像一个弹簧质量系统（spring-loaded inverted pendulum, SLIP），天然具有被动稳定性。这就是为什么你可以在 Ch13 中看到"四足 zero agent 站着不倒"的现象——PD 控制器 + default pose 足以维持静态平衡。但 G1 的 zero agent 如果没有精确的 default pose 和足够的 PD gains，**站都站不住**。

**工程含义**：在四足上，你可以先跑 zero agent 验证 wiring 而不用担心摔倒。在人形上，zero agent 可能在 1-2 秒内就倒了。这不代表配置有错——这是物理上的正常现象。你需要用 `--no-terminations` 来让 zero agent 检查完 sensor 数据。

### 系统性对比表

| 维度 | Go1 四足 | G1 双足 | 工程后果 |
|------|---------|---------|---------|
| **支撑面** | 多边形大（~0.12 m²） | 极窄（单脚 ~0.025 m²） | G1 对 roll 和 lateral slip 极其敏感 |
| **质心高度** | trunk 低（~0.3 m） | pelvis 高（~0.76 m） | 倒立摆不稳定性更强，恢复时间更短 |
| **自由度** | 12 个 actuated joints | 29 个 actuated joints（23-DoF 或 29-DoF 版本） | action space 维度翻倍以上 |
| **上肢** | 无手臂 | 肩/肘/腕共 14 个关节 | 上肢角动量直接影响躯干姿态 |
| **腰部** | 无腰关节（或固定） | 3-DoF waist（yaw/roll/pitch） | 上下半身角动量耦合 |
| **足端** | 小足端点接触 | 长脚掌多个 collision geom | contact sensor 需覆盖 foot1-foot7 |
| **角动量** | 主要来自腿和 trunk | 腿、腰、手臂共同贡献 | `angular_momentum` penalty 更重要 |
| **步态对称性** | trot 天然对称 | 行走是左右交替 | 可以用 mirror augmentation |
| **DR 敏感性** | friction 和 push 已关键 | CoM、encoder bias、self collision 更敏感 | 随机化需要更保守地逐步放大 |
| **空气时间 reward** | 鼓励四脚交替抬起 | 双足摆动节奏完全不同 | 四足 air time reward **不能**照搬 |

这个对比表的**每一行**都对应一个具体的工程陷阱。比如"上肢"这一行：如果你把四足的 reward 直接搬到 G1，reward 函数中不会有任何关于上肢的约束——策略会发现"让手臂大幅摆动可以产生角动量来补偿走路时的旋转不平衡"，从而学到一种手臂乱甩但 tracking reward 很高的非自然步态。

> **本质洞察**：人形控制的难点不是 DoF 多这一件事。真正的难点是**支撑面小 + 质心高 + 上肢和腰部会反向影响躯干角动量**。同一个动作幅度，在四足上可能只是步态变化，在双足上可能就是摔倒触发器。理解这一点是后续所有工程决策的基础。

### 四足到双足的迁移检查清单

当你把 Ch13 的四足 velocity task 迁移到人形时，以下每一项都需要重新思考：

```
□ reward
  - tracking reward 可以保留，但 σ 可能需要调大（人形更难精确跟踪）
  - 必须新增 angular_momentum penalty
  - 必须新增 variable posture reward（上肢约束）
  - foot_clearance 的阈值需要重新设置（步高不同）
  - foot_slip 权重可能需要增大（打滑后果更严重）

□ observation
  - actor obs 维度增大（29 joints vs 12）
  - critic 需要包含 angular_momentum
  - height_scan 的 grid pattern 可能需要调整（足端覆盖不同）

□ termination
  - fell_over 的角度阈值需要更严格（人形倾斜容忍度更低）
  - 需要新增 self_collision termination（手臂可能碰到腿）

□ action
  - action scale 必须 per-joint 设置（不能统一值）
  - default pose 极其关键（决定 zero agent 能否站住）

□ DR
  - push force 需要更保守（同样力量对人形影响更大）
  - friction 范围需要更保守（单脚支撑时打滑更致命）
  - mass randomization 对高质心影响更大
```

### 反面案例诊断

| 反面案例 | 表面现象 | 真实根因 | 第一检查项 |
|---------|---------|---------|-----------|
| 四足 reward 照搬 | G1 能走几步但侧向摇摆越来越大 | 无 angular momentum penalty | 加 `angular_momentum` reward |
| action scale 统一 0.5 | 膝和踝快速抽动，髋步幅不够 | 力矩能力差异被忽略 | 改用 per-joint action scale |
| 上肢完全放松 | 手臂乱甩但速度 reward 还在涨 | 上肢角动量补偿未被约束 | 加 variable posture reward |
| 上肢完全锁死 | 跑步时躯干更僵更容易摔 | 缺少自然摆臂来抵消腿部角动量 | 放宽 shoulder pitch std |
| 只奖励速度 | G1 低头冲刺后翻倒 | base height 和 torso 姿态没有约束 | 加 upright + base_height reward |
| DR 一次全开 | 训练初期全部摔倒 | push/friction/CoM 同时扩大 | events 分阶段打开 |

### 你的第一个 G1 Velocity Task：30 分钟工程流程

如果你已经在 Ch13 中完成了 Go1 velocity task 的训练，以下是在 G1 上复现的快速路径。整个流程约 30 分钟（不含 large train 时间）。

**Step 1：确认 G1 模型可用（2 分钟）**

```bash
# 确认 G1 task 已注册
uv run list-envs | grep G1

# 预期输出包含：
# Mjlab-Velocity-Flat-Unitree-G1
# Mjlab-Velocity-Rough-Unitree-G1
```

如果没有输出，说明 unitree_rl_mjlab 未安装或版本不对。

**Step 2：Zero Agent 验证（5 分钟）**

```bash
# 注意 --no-terminations：人形可能很快摔倒，这不代表配置错误
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --agent zero --num-envs 4 \
  --viewer viser --no-terminations
```

观察要点：
- G1 能站 >1 秒吗？如果不能 → default pose 或 PD gains 有问题
- 关节有没有明显抖动？→ action scale 或 damping 有问题
- 脚掌是否平稳接地？→ contact geom 配置检查
- 在 viewer 中打印 `obs` tensor shape，确认维度正确

**Step 3：Random Agent 验证（3 分钟）**

```bash
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --agent random --num-envs 4 \
  --viewer viser
```

观察要点：
- 关节运动范围是否合理？（不应该有关节旋转 180°）
- reset 是否正常触发？
- 有没有 self-collision 的视觉证据？（手臂穿过身体）

**Step 4：Small Train 验证（10 分钟）**

```bash
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
  --env.scene.num-envs 256 --agent.max-iterations 50 \
  --agent.logger tensorboard --gpu-ids "[0]"
```

检查：
- 无 shape error、无 NaN
- tensorboard 日志正常写入
- 打印 obs dim：flat G1 29-DoF 的 actor obs 约 99 维（3+3+3+3+29+29+29 = 99，具体依 obs 配置而定）
- 打印 action dim：29（29-DoF 版本）

**Step 5：Large Train（训练时间约 1-2 小时）**

```bash
# flat baseline
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
  --env.scene.num-envs 4096 --agent.max-iterations 5000 \
  --agent.run-name g1_flat_baseline

# 训练完后 play
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --checkpoint-file logs/rsl_rl/g1_velocity/g1_flat_baseline/model_5000.pt \
  --num-envs 4 --viewer viser
```

**人形特有的训练监控**：除了 Ch13 提到的标准指标外，G1 训练还需要关注：

| 指标 | 健康范围 | 异常信号 | 对应问题 |
|------|---------|---------|---------|
| `reward/upright` | 持续高位 | 持续下降 | 策略前倾或侧倾换速度 |
| `reward/angular_momentum` | 缓慢下降到小值 | 震荡或持续大值 | 手臂/腰部大幅摆动 |
| `reward/variable_posture` | 从负值缓慢趋零 | 持续很大负值 | 关节大幅偏离 default |
| `reward/self_collision` | 接近零 | 频繁非零 | 手臂碰腿/腰 |
| `episode/length` | 逐渐增长到 >500 | 持续 <100 | termination 过严或 reward balance 问题 |

**与 Go1 的对比基准**：

| 指标 | Go1 flat (Ch13 基准) | G1 flat (本章预期) | 说明 |
|------|-------------------|------------------|------|
| 收敛 iterations | ~1500 | ~2500-3000 | 人形需要更多样本 |
| 最终 tracking error | ~0.12 m/s | ~0.18 m/s | 人形更难精确跟踪 |
| Fall rate | <3% | <8% | 人形更脆弱 |
| steps/s | ~12000 | ~8000-10000 | 29-DoF 比 12-DoF 计算量大 |

### G1 训练曲线的典型形态与诊断

一个健康的 G1 velocity 训练曲线通常经历五个阶段（比 Go1 多一个"上肢收敛"阶段）：

| 阶段 | 迭代范围 | 现象 | 解释 |
|------|---------|------|------|
| I. 探索期 | 0-800 | reward 缓慢上升，episode 很短 | 策略从随机动作中学会"站立" |
| II. 站稳期 | 800-1500 | episode length 开始增长 | 策略学会在 default pose 附近维持平衡 |
| III. 步态涌现期 | 1500-2500 | tracking reward 快速上升 | 策略学会基本的行走步态 |
| **IV. 上肢收敛期** | 2500-3500 | angular_momentum 下降，posture 改善 | **人形特有**：上肢从乱甩过渡到自然摆臂 |
| V. 精修期 | 3500+ | reward 缓慢上升到平台 | 步态细节优化——着地柔度、能耗 |

**Stage II 是人形特有的关键阶段**：Go1 的 zero agent 天然能站稳，所以不需要额外的"站稳"阶段。G1 需要先学会在双足上平衡，然后才能开始行走。如果你的训练在 Stage I 停留过久（>1500 iterations），通常是 default pose 或 PD gains 的问题——策略连站都站不住，当然无法学会走路。

**Stage IV 是判断 variable posture reward 是否有效的关键窗口**：如果 Stage III 后 tracking reward 继续上升但 angular_momentum 也在上升，说明策略在用"不自然但有效"的方式走路（如手臂乱甩换角动量）。此时 variable posture reward 应该开始发挥作用——如果它的权重或 std 配置不合理，Stage IV 不会出现，策略停留在"乱走"状态。

**反事实推理：如果跳过 Stage IV 直接认为训练完成会怎样？** 策略可能 tracking reward 很高（>0.7），看起来"会走了"。但部署到真机或跨引擎验证时，由于角动量管理不佳，微小的扰动就会导致失衡。angular_momentum 下降是鲁棒性的必要（非充分）条件。

### 人形特有的 DR 引入时机

与 Go1 不同，G1 的 DR 引入需要更精确地与训练阶段对齐：

| 训练阶段 | 可以引入的 DR | 不应引入的 DR |
|---------|------------|------------|
| 站稳期 (Stage II) | friction、motor_strength（小范围） | push（会导致站不住） |
| 步态涌现期 (Stage III) | 扩大 friction 范围 | push（步态还不稳定） |
| 上肢收敛期 (Stage IV) | obs noise、CoM offset | push（上肢还在收敛） |
| 精修期 (Stage V) | **push**（此时才安全）、全部 DR | — |

**核心原则**：random push 对人形的影响远大于四足（14.1 已分析）。只有在策略已经建立了稳定的步态和角动量管理后，才能引入 push。过早引入 push 会让策略学到"蹲下来降低重心"的保守策略——tracking 变差但不容易被推倒。

### ⚠️ 常见陷阱

💡 **概念误区：人形控制就是更多自由度的四足控制**。新手想法："G1 有 29 个关节，Go1 有 12 个，只是规模更大而已。" 实际上：质的变化不仅是量的增加。双足的欠驱动特性、窄支撑面导致的本征不稳定性、上肢对躯干角动量的反向影响——这些都是四足中不存在的物理约束。四足到双足的跨越就像从双翼飞机到直升机——两者都在飞，但飞行原理完全不同。

🧠 **思维陷阱：reward 高就说明步态正确**。G1 可以通过低头冲刺、手臂乱甩或膝盖内扣来满足速度命令。必须**同时看** torso pitch/roll、angular momentum、foot contact 和 self collision。正确做法：永远不要只看总 return。把 reward 分项画出来，尤其关注 upright 和 angular_momentum 是否在恶化。

⚠️ **编程陷阱：四足的 air time reward 照搬给 G1**。四足 trot 和双足行走的支撑/摆动模式完全不同。直接复用四足的 `feet_air_time` reward 可能导致 G1 在站立时也不断抬脚，或者步频异常。需要重新设计适合双足步态的接触 reward。

⚠️ **工程陷阱：用 Go1 的 push force 范围给 G1**。Go1 可以承受 ±15 N 的随机 push 而不摔倒。同样的力施加到 G1 上，由于质心更高和支撑面更窄，可能直接触发失衡。G1 的 push force 初始值应该设为 Go1 的 30-50%。

### 练习

1. **[估算题]** 计算 Go1 四足 trot 和 G1 双足单脚支撑时的支撑多边形面积。假设 Go1 对角脚间距 0.4m × 0.3m，G1 脚掌 0.25m × 0.10m。面积差了多少倍？
2. **[概念题]** 为什么双足机器人在静止站立时也需要主动控制？用倒立摆模型解释。如果把 G1 的踝关节锁死（只有髋关节可动），这个控制问题的性质会怎样变化？
3. **[跨章综合题，Ch08+Ch13+Ch14]** 结合 Ch08 的 DR 分阶段原则和 Ch13 的四足 DR 配置，为 G1 设计 DR 引入方案。哪些随机项需要比 Go1 更保守？按什么顺序引入？

---

Ch13 中 Go1 的 12 个关节你已经很熟悉——3 个 hip + 1 个 knee × 4 条腿。G1 的 29 个关节分布在腿、腰和手臂上，每一组关节的 action scale、default pose 和 reward 约束都需要独立设计。本节从 G1 的关节地图开始，建立你对人形 action space 的完整理解。

## 14.2 G1 自由度分析与 Action Space 设计 ⭐⭐⭐

> **这一节解决什么问题**：理解 G1 的关节布局、action scale 的物理来源和 variable posture reward 的设计动机。

### G1 机器人规格

根据 Unitree 官方规格，G1 有两个版本：

| 规格 | 23-DoF 版本 | 29-DoF 版本 |
|------|-----------|-----------|
| 身高 | 1.32 m | 1.32 m |
| 体重（含电池） | ~35 kg | ~35 kg |
| 腿部 | 6×2 = 12 个 actuated joints | 同左 |
| 腰部 | 3 个 actuated joints | 同左 |
| 手臂 | 4×2 = 8 个 actuated joints | 同左 |
| 手腕 | — | 3×2 = 6 个 actuated joints |
| **总计** | **23 个 actuated joints** | **29 个 actuated joints** |

H1 是更大的人形平台：1.8 m / ~47 kg / 19 个 actuated joints。H1 没有手腕关节，手臂自由度更少，但腿部力矩更大，更适合高速运动。本章以 G1（29-DoF）为主要示例，H1 的差异会在 14.4 中对照介绍。

### G1 关节地图

G1 的 29 个关节分为 5 组。理解每组的物理功能和风险是设计 action scale 和 reward 的前提：

| 关节组 | 关节名 | 数量 | 主要功能 | 过大时的风险 |
|--------|--------|------|---------|------------|
| **腿-髋** | hip_pitch/roll/yaw × 2 | 6 | 步幅、侧向平衡、转向 | 甩腿、摆胯、步态失稳 |
| **腿-膝** | knee × 2 | 2 | 支撑高度、摆腿折叠 | 膝部抽动、蹲走 |
| **腿-踝** | ankle_pitch/roll × 2 | 4 | 脚尖离地、脚掌平衡 | 脚掌边缘抖动、打滑 |
| **腰部** | waist_yaw/roll/pitch | 3 | 转身、侧倾、前后倾 | torso 扭动、上下半身不协调 |
| **手臂-肩肘** | shoulder_pitch/roll/yaw × 2 + elbow × 2 | 8 | 自然摆臂、侧向平衡 | 手臂乱甩、自碰撞 |
| **手腕** | wrist_roll/pitch/yaw × 2 | 6 | 末端姿态（29-DoF 版本） | 对行走贡献小但影响自碰撞 |

**3-DoF 腰部是最关键的教学点**。四足机器人没有腰关节——上下半身是刚性连接的。G1 的腰部 yaw/roll/pitch 耦合了上下半身的角动量。当腿部迈步产生 yaw 方向角动量时，腰部可以旋转来部分吸收这个角动量，而不是全部传递到上半身。但这也意味着——如果 reward 不约束腰部，策略可能学到用腰部大幅扭动来换取速度优势。

### 浮动基座与间接控制

G1 的 XML 中有一个 `floating_base_joint`——它不是策略直接控制的 hinge joint，而是浮动基座（6 DoF：3 平移 + 3 旋转），代表机器人整体在空间中的位姿。策略输出的是 29 个关节的位置目标（或增量），仿真中的 PD 控制器将位置目标转化为力矩，力矩通过接触产生地面反力，地面反力最终驱动浮动基座运动。

```
策略输出 → joint position target (29 维)
       → PD 控制器 → joint torque
       → 接触力学 → ground reaction force
       → 牛顿力学 → base 加速度
       → 积分 → base 速度和位置
```

这就是腿式控制比固定机械臂难的根本原因。固定机械臂的末端位置误差可以由关节空间直接解释。**人形 base 速度误差必须通过接触间接实现**——策略不能直接"命令" base 移动，只能通过改变腿的姿态来改变接触力。这个间接性是所有足式 RL 的核心挑战，但在人形上因为窄支撑面而被放大。

### Action Scale 的物理来源

Ch13 中 Go1 的 action scale 是一个统一值（如 0.25 rad）。这在四足上勉强可行，因为 12 个关节的力矩能力差异不大。但 G1 的 29 个关节的力矩能力差异巨大——髋 pitch 电机有 88 Nm，而腕 yaw 只有 5 Nm。统一 action scale 会导致：

- 力矩大的关节（髋 pitch）步幅不够——action 被限制在 ±0.25 rad，对应的力矩远小于电机能力
- 力矩小的关节（腕 yaw）频繁饱和——action 对应的力矩超过电机能力，PD 控制器截断

mjlab G1 配置使用 **per-joint action scale**，公式为：

$$\text{action\_scale}_j = 0.25 \times \frac{\text{effort\_limit}_j}{\text{stiffness}_j}$$

这个公式把力矩能力（effort_limit）和 PD 刚度（stiffness）连接起来。当策略输出 $a_j \in [-1, 1]$ 时，实际关节位置增量是 $a_j \times \text{action\_scale}_j$。这确保了每个关节在 $[-1, 1]$ 的 action 范围内都能充分利用其力矩能力，不会饱和也不会浪费。

**具体计算示例**：

| 关节 | effort_limit (Nm) | stiffness | action_scale (rad) | 含义 |
|------|-------------------|-----------|-------------------|------|
| `left_hip_pitch` | 88 | 150 | 0.25 × 88/150 = **0.147** | 髋 pitch 可偏移 ±0.147 rad |
| `left_knee` | 139 | 200 | 0.25 × 139/200 = **0.174** | 膝关节可偏移 ±0.174 rad |
| `left_ankle_pitch` | 50 | 40 | 0.25 × 50/40 = **0.313** | 踝 pitch 可偏移 ±0.313 rad |
| `left_shoulder_pitch` | 25 | 40 | 0.25 × 25/40 = **0.156** | 肩 pitch 可偏移 ±0.156 rad |
| `left_wrist_yaw` | 5 | 40 | 0.25 × 5/40 = **0.031** | 腕 yaw 只能偏移 ±0.031 rad |

差距超过 **10 倍**（踝 0.313 vs 腕 0.031）。如果统一用 0.25 rad，腕关节每一步都在饱和振荡，而踝关节的实际利用率不到 80%。

**反事实推理：如果所有关节统一 action scale = 0.5 rad 会怎样？** 力矩小的关节（腕、手指）被要求做大幅度运动，但电机力矩不够——PD 控制器的目标位置和实际位置之间有持续大偏差，产生最大力矩输出。关节在限位之间快速振荡（抽动）。同时，力矩大的关节（髋 pitch）被限制在 ±0.5 rad，本来可以做更大步幅但被人为截断。整体表现是：上肢抽动 + 下肢步幅受限 + 能耗异常高。

### Default Pose 与 Keyframe

G1 的 default pose 极其重要。在 Ch13 中，Go1 的 default pose 是一个自然站立姿态，zero action 时机器人可以稳定站立数秒。G1 的 default pose 同样定义在 MJCF 的 `<keyframe>` 中：

```xml
<!-- G1 MJCF keyframe 片段 -->
<keyframe>
  <key name="standing" qpos="0 0 0.76  1 0 0 0
    0 0 -0.39  0.80  -0.42 0    <!-- 左腿: hip_roll hip_yaw hip_pitch knee ankle_pitch ankle_roll -->
    0 0 -0.39  0.80  -0.42 0    <!-- 右腿 -->
    0 0 0                        <!-- 腰: yaw roll pitch -->
    0.3 0 0  -0.5               <!-- 左臂: shoulder_pitch shoulder_roll shoulder_yaw elbow -->
    0 0 0                        <!-- 左腕: roll pitch yaw (29-DoF) -->
    0.3 0 0  -0.5               <!-- 右臂 -->
    0 0 0                        <!-- 右腕 -->
  "/>
</keyframe>
```

关键观察：
- 基座高度 0.76 m（`qpos[2]`），与 G1 的实际站高一致
- 髋 pitch -0.39 rad + 膝 0.80 rad + 踝 -0.42 rad ≈ 微弯站立姿态
- 手臂 shoulder_pitch 0.3 rad + elbow -0.5 rad ≈ 自然下垂但微弯

如果 default pose 不合理（比如膝关节完全伸直 = 0），zero agent 在 PD 控制下可能无法维持平衡——完全伸直的膝关节没有"弹性余量"来吸收扰动。微弯站立（knee ≈ 0.8 rad）提供了被动的弹簧效应。

### 从 MJCF 提取关节信息的工程流程

与 Ch13 类似，添加新人形机器人前需要系统性地提取关节信息。以下是 G1 专用的检查脚本：

```python
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("g1_29dof.xml")

# 提取所有 actuated joints 的信息
print("=" * 60)
print("G1 Actuated Joints Summary")
print("=" * 60)
for i in range(m.nu):  # nu = number of actuators
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    joint_id = m.actuator_trnid[i, 0]
    joint_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    
    # PD gains
    kp = m.actuator_gainprm[i, 0]  # position gain
    
    # effort limit
    effort = m.actuator_ctrlrange[i, 1]  # upper bound
    
    # joint range
    if m.jnt_limited[joint_id]:
        lo, hi = m.jnt_range[joint_id]
    else:
        lo, hi = -np.inf, np.inf
    
    # action scale = 0.25 * effort / kp
    action_scale = 0.25 * effort / kp if kp > 0 else 0.0
    
    print(f"{name:30s} | kp={kp:6.1f} | effort={effort:5.1f} Nm | "
          f"range=[{lo:+.2f}, {hi:+.2f}] | scale={action_scale:.4f} rad")
```

运行这段代码会生成 29 行输出，每行包含关节名、PD gain、力矩限制、关节范围和计算出的 action scale。这些数据直接填入 env cfg 的 `ActionCfg` 中。

**工程建议**：把这段代码的输出保存为 `g1_joint_summary.txt`，在后续配置 reward、DR 和 termination 时反复查阅。

### Variable Posture Reward 的设计

四足的 `pose` reward 使用统一的 std（标准差）——所有关节回归 default pose 的惩罚强度相同。这对 12 个功能相似的腿关节是合理的。但 G1 的 29 个关节功能差异巨大——锁死手臂不利于行走，完全放开手臂会导致乱甩。

Variable posture reward 的核心思想：**根据当前运动状态自适应地调整每个关节的约束强度**。

```python
# variable posture reward 的概念性实现
def variable_posture_reward(joint_pos, default_pos, cmd_speed, std_table):
    """
    Args:
        joint_pos: (N, 29) 当前关节位置
        default_pos: (29,) 默认姿态
        cmd_speed: (N,) 命令速度的范数
        std_table: dict, 每个运动状态下每个关节的 std
    """
    # 根据速度选择运动状态
    state = where(cmd_speed < 0.2, "standing",
                  where(cmd_speed < 1.0, "walking", "running"))
    
    # 每个关节的 std
    per_joint_std = std_table[state]  # (N, 29)
    
    # 加权 L2 惩罚
    deviation = joint_pos - default_pos  # (N, 29)
    reward = -sum((deviation / per_joint_std) ** 2, dim=-1)
    
    return reward
```

不同运动状态下的 std 配置示例：

| 关节 | standing | walking | running | 设计理由 |
|------|----------|---------|---------|---------|
| shoulder_pitch | 0.10 | 0.15 | **0.50** | 跑步需要大摆臂补偿角动量 |
| shoulder_roll | 0.10 | 0.15 | 0.15 | 侧向摆臂在任何速度下都不希望过大 |
| shoulder_yaw | 0.05 | 0.10 | 0.10 | yaw 方向的旋转几乎不用于行走 |
| elbow | 0.10 | 0.15 | **0.35** | 跑步时弯肘摆臂更自然 |
| waist_roll | 0.02 | 0.02 | **0.05** | 腰部侧倾始终要严格约束 |
| waist_pitch | 0.05 | 0.10 | 0.10 | 允许一定的前后倾 |
| waist_yaw | 0.05 | 0.10 | 0.10 | 允许一定的转体 |
| ankle_roll | 0.02 | 0.02 | 0.05 | 踝 roll 过大 → 脚掌边缘接触 |
| knee | 0.10 | 0.15 | 0.20 | 跑步时需要更大的膝弯幅度 |

> **本质洞察**：完全锁死和完全放松都不是好配置。工程目标是让上肢帮助平衡，而不是成为投机通道。variable posture reward 通过运动状态自适应的约束强度来实现这个平衡——standing 时严格约束所有关节（保持安静站立），running 时放宽肩和肘（允许摆臂），但始终严格约束 ankle_roll 和 waist_roll（防止失稳）。

### 角动量与 subtreeangmom

G1 MJCF 中有一个 `subtreeangmom` sensor（名为 `root_angmom`，body 是 `pelvis`），测量以 pelvis 为根的子树角动量。velocity cfg 把 `angular_momentum` weight 设为 -0.02。

**angular_momentum penalty 的物理意义**：抑制全身角动量增长。正常行走中，每迈一步都涉及摆腿产生的角动量和摆臂的补偿角动量——这些是必要的。penalty 太大时策略过度保守——步幅缩小、行走变慢、效率下降。penalty 太小时上肢和腰可能高频摆动换速度——手臂乱甩、步态不自然。

```python
# angular momentum reward term
"angular_momentum": RewardTermCfg(
    func=angular_momentum_penalty,
    weight=-0.02,
    params={"sensor_name": "root_angmom"},
)
```

**跨领域类比**：angular momentum penalty 就像汽车的 ESP（电子稳定程序）。ESP 限制车辆的偏航角速率——不是禁止转弯（那样车就不能动了），而是在偏航角速率超过安全范围时介入。angular momentum penalty 同理——不是禁止所有角动量变化（那样就走不了路），而是在角动量超过正常行走范围时给出惩罚。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：action scale 设置不当导致关节抽动**。错误做法：统一给所有关节 0.5 rad 的 action scale。现象：踝和腕关节快速抽动，膝关节动作幅度太小。正确做法：使用 per-joint action scale = 0.25 × effort_limit / stiffness。

💡 **概念误区：angular momentum penalty 越大越稳定**。正常行走需要角动量变化——每一步都涉及摆腿角动量和摆臂补偿。过大 penalty 逼策略到极小步幅，反而因为步态不自然而更容易在扰动下失稳。

🧠 **思维陷阱：手臂关节不重要**。"行走主要靠腿，手臂 reward 权重可以很低"——实际上手臂挥动产生的角动量直接影响躯干姿态。不约束手臂等于给策略一个"免费"的角动量来源，策略会利用它来换取短期的速度提升。

⚠️ **编程陷阱：23-DoF 和 29-DoF 配置混用**。G1 的 23-DoF 版本没有手腕关节，action dim 是 23；29-DoF 版本包含手腕，action dim 是 29。混用会导致 action shape mismatch——这个错误在 env 创建时就会 crash，但错误信息可能不直观（显示为 tensor shape error）。

### G1 Velocity Task 的 Observation 配置

G1 velocity task 的 observation 遵循 Ch13 相同的 actor/critic 双组结构，但维度更大。以下是典型配置：

```python
# G1 flat velocity actor observation terms
cfg.observations.actor = ObservationGroupCfg(
    enable_corruption=True,
    concatenate_terms=True,
    terms={
        "base_lin_vel": ObsTerm(
            func=base_lin_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.1),  # 同 Go1
        ),
        "base_ang_vel": ObsTerm(
            func=base_ang_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.2),
        ),
        "projected_gravity": ObsTerm(
            func=projected_gravity,
            noise=GaussianNoiseCfg(mean=0.0, std=0.05),
        ),
        "command": ObsTerm(
            func=generated_commands,
            params={"command_name": "twist"},  # 3 维: vx, vy, wz
        ),
        "joint_pos": ObsTerm(
            func=joint_pos_rel,  # 相对于 default pose 的偏差
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),
        ),  # → 29 维（29-DoF）
        "joint_vel": ObsTerm(
            func=joint_vel,
            noise=GaussianNoiseCfg(mean=0.0, std=1.5),
        ),  # → 29 维
        "last_action": ObsTerm(func=last_action),  # → 29 维
    },
)
# actor obs 总维度: 3+3+3+3+29+29+29 = 99

# critic 额外添加 privileged terms
cfg.observations.critic = ObservationGroupCfg(
    enable_corruption=False,
    concatenate_terms=True,
    terms={
        # 复制 actor 所有 terms（但 enable_corruption=False）
        **cfg.observations.actor.terms,
        # 额外 privileged terms
        "foot_height": ObsTerm(
            func=foot_position_in_base_frame,
            params={"foot_sites": ("left_foot", "right_foot")},
        ),  # → 2 × 3 = 6 维
        "foot_contact_forces": ObsTerm(
            func=contact_forces,
            params={"sensor_name": "feet_contact"},
        ),  # → 2 × 3 = 6 维
        "angular_momentum": ObsTerm(
            func=sensor_data,
            params={"sensor_name": "root_angmom"},
        ),  # → 3 维
    },
)
# critic obs 总维度: 99 + 6 + 6 + 3 = 114
```

**维度对比**：

| obs 组 | Go1 (12-DoF) | G1 (29-DoF) | 增幅 | 原因 |
|--------|-------------|-------------|------|------|
| actor | ~48 | **99** | +106% | joint_pos/vel/action 各多 17 维 |
| critic | ~63 | **114** | +81% | 同上 + angular_momentum 3 维 |

这个维度增幅意味着：如果 Go1 使用 `actor_hidden_dims = (512, 256, 128)` 足够，G1 可能需要更大的网络（如 `(768, 512, 256)`）来处理更多的输入维度。但也不要盲目增大——过大的网络需要更多样本才能收敛，可能导致训练时间过长。建议从 Go1 相同的网络大小开始，如果 tracking reward 收敛过慢（>3000 iterations 仍未进入步态涌现期），再考虑增大。

### G1 Termination 配置

| 条件 | 类型 | 阈值 (flat) | 阈值 (rough) | 与 Go1 的差异 |
|------|------|-----------|------------|-------------|
| `time_out` | truncation | 20 秒 | 20 秒 | 同 |
| `fell_over` | terminal | pitch/roll > 50° | — (删除) | G1 阈值更严（Go1 用 70°） |
| `illegal_contact` | terminal | — | 膝/大腿触地 | 同 |
| `out_of_terrain` | truncation | — | 走出地形 | 同 |
| `base_height` | terminal | < 0.4m | < 0.4m | **新增**：防止蹲走或摔倒但不触发倾斜检测 |
| `self_collision` | terminal | 异常碰撞对 | 异常碰撞对 | **新增**：手臂碰腿 |

**为什么 G1 的 `fell_over` 阈值比 Go1 更严（50° vs 70°）？** 因为 G1 在倾斜 50° 时已经很难恢复——窄支撑面意味着恢复所需的力矩远大于四足。如果允许策略在 50-70° 的倾斜下继续运行，它可能学到"严重倾斜但不摔倒"的怪步态，部署到真机上会立即摔倒。更严格的阈值迫使策略在倾斜早期就采取纠正动作。

### 练习

1. **[计算题]** 使用上述公式计算 `left_hip_pitch_joint`（effort=88 Nm, stiffness=150）和 `left_wrist_yaw_joint`（effort=5 Nm, stiffness=40）的 action scale。两者差多少倍？解释这个差异的物理含义。
2. **[设计题]** 如果 G1 需要搬运重物（双手持物体），上肢的 variable posture std 应该如何调整？这时 angular momentum penalty 的权重需要变化吗？为什么？
3. **[实验题]** 用 `uv run play Mjlab-Velocity-Flat-Unitree-G1 --agent zero --num-envs 4 --viewer viser --no-terminations` 观察 G1 在 zero agent 下的行为。记录：(a) G1 能站多久？(b) 哪些关节最先达到 limit？(c) 如果去掉 `--no-terminations` 会怎样？

---

理解了 G1 的关节结构和 action space 后，下一步是设计人形特有的 reward。Ch13 的四层框架（Tracking/Regularization/Style/Contact）仍然适用，但每一层都需要针对人形做出调整——特别是 Style 层需要加入 angular momentum 和 variable posture 两个四足中不存在的组件。

## 14.3 人形特有 Reward 设计 ⭐⭐⭐

> **这一节解决什么问题**：用 Ch13 的 Reward 四层框架分析人形 velocity task 的 reward 设计，理解每个人形特有 term 的工程动机。

### 四层框架在人形上的适配

回顾 Ch13 的四层 reward 框架——Tracking（目标）→ Regularization（约束）→ Style（偏好）→ Contact/Safety（红线）。人形的每一层都需要修改：

| 层级 | 四足 (Ch13) | 人形 (Ch14) | 新增/修改项 |
|------|------------|------------|-----------|
| **Tracking** | lin_vel_xy, ang_vel_yaw | 同左 | σ 可能需要调大 |
| **Regularization** | action_rate, dof_accel, torque | 同左 + **angular_momentum** | 新增角动量惩罚 |
| **Style** | upright, pose, feet_airtime | upright, **variable_posture**, feet_airtime | pose → variable_posture |
| **Contact/Safety** | foot_slip, undesired_contacts | foot_slip, undesired_contacts, **self_collision** | 新增自碰撞 |

### 人形 Velocity Task 的完整 Reward 配置

以下是 mjlab G1 velocity task 的典型 reward 配置——与 Ch13 的 Go1 配置逐项对照：

| Term | 四层分类 | G1 权重 | Go1 权重 (Ch13) | 差异说明 |
|------|---------|--------|----------------|---------|
| `track_linear_velocity` | Tracking | +2.0 | +2.0 | 不变 |
| `track_angular_velocity` | Tracking | +2.0 | +2.0 | 不变 |
| `upright` | Style | +1.0 | +1.0 | 不变，但对人形更关键 |
| **`variable_posture`** | Style | +1.0 | — | **新增**：替代统一 pose reward |
| `base_height` | Style | +0.5 | — | **新增**：防止蹲走 |
| **`angular_momentum`** | Regularization | **-0.02** | — | **新增**：抑制全身角动量 |
| `action_rate_l2` | Regularization | -0.1 | -0.1 | 不变 |
| `dof_acceleration` | Regularization | -0.0025 | -0.0025 | 不变 |
| `joint_torques` | Regularization | -0.0001 | -0.0001 | 不变 |
| `dof_pos_limits` | Regularization | -1.0 | -1.0 | 不变 |
| `foot_clearance` | Contact | -2.0 | -2.0 | 阈值可能不同 |
| `foot_slip` | Contact | -0.2 | -0.1 | **加大**：打滑后果更严重 |
| **`self_collision`** | Safety | **-1.0** | — | **新增**：手臂可能碰到腿 |
| `undesired_contacts` | Safety | -1.0 | -1.0 | 监测的 body 不同 |

总共 **14 个** reward terms（Go1 是 ~10 个）。新增的 4 个 term 都是人形特有的。

### mjlab G1 Velocity RewardsCfg 的完整注册

在 mjlab 中，reward terms 通过 `RewardTermCfg` 注册在 env cfg 的 rewards 字典中。以下是 G1 velocity task 的完整 reward 配置——每一行都有注释说明与 Go1 的差异：

```python
# src/mjlab/tasks/velocity/config/g1/env_cfgs.py（简化版）
def unitree_g1_flat_env_cfg():
    cfg = make_velocity_env_cfg()
    
    # === Entity wiring ===
    cfg.scene.entity.asset_path = "unitree_g1/xmls/g1_29dof.xml"
    cfg.scene.entity.default_joint_pos = G1_DEFAULT_POSE  # from g1_constants.py
    cfg.scene.entity.actuator_names = G1_ACTUATOR_NAMES   # 29 actuators
    
    # === Action: per-joint scale ===
    cfg.actions.joint_pos.scale = G1_ACTION_SCALE  # (29,) tensor, 非统一值
    
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
        "base_height": RewardTermCfg(  # ★ 人形新增
            func=base_height_l2,
            weight=0.5,
            params={"target_height": 0.76, "sensor_name": "body_pos"},
        ),
        "variable_posture": RewardTermCfg(  # ★ 人形新增（替代统一 pose）
            func=variable_posture_reward,
            weight=1.0,
            params={
                "joint_names": G1_ACTUATOR_NAMES,
                "default_joint_pos": G1_DEFAULT_POSE,
                "std_standing": G1_POSTURE_STD_STANDING,
                "std_walking": G1_POSTURE_STD_WALKING,
                "std_running": G1_POSTURE_STD_RUNNING,
                "speed_thresholds": (0.2, 1.0),
            },
        ),
        "feet_air_time": RewardTermCfg(
            func=feet_air_time_reward,
            weight=0.5,
            params={"sensor_name": "feet_contact", "threshold": 0.5},
        ),
        
        # --- Regularization 层 ---
        "angular_momentum": RewardTermCfg(  # ★ 人形新增
            func=angular_momentum_penalty,
            weight=-0.02,
            params={"sensor_name": "root_angmom"},
        ),
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
            weight=-0.2,  # ★ 比 Go1 的 -0.1 加大
            params={"sensor_name": "feet_contact"},
        ),
        "self_collision": RewardTermCfg(  # ★ 人形新增
            func=self_collision_penalty,
            weight=-1.0,
            params={
                "body_pairs": [
                    ("left_hand_link", "left_thigh_link"),
                    ("right_hand_link", "right_thigh_link"),
                    ("left_forearm_link", "torso_link"),
                    ("right_forearm_link", "torso_link"),
                ],
            },
        ),
        "undesired_contacts": RewardTermCfg(
            func=undesired_contacts_penalty,
            weight=-1.0,
            params={"sensor_name": "contact_sensor",
                    "body_names": ("torso_link", "pelvis",
                                   "left_thigh_link", "right_thigh_link")},
        ),
    }
    
    return cfg
```

这段代码的核心教学价值在于：你可以看到人形和四足的 reward 配置在**结构上完全相同**（都是 `RewardTermCfg` 字典），但在**内容上有四处关键差异**（标记为 ★）。这意味着——如果你理解了 Ch13 的 Go1 配置，阅读 G1 配置只需要关注这四个 ★ 标记的新增项。

**从 Go1 cfg 到 G1 cfg 的 diff 总结**：

```diff
# 新增
+ "base_height": weight=0.5, target=0.76
+ "variable_posture": weight=1.0, per-joint std
+ "angular_momentum": weight=-0.02
+ "self_collision": weight=-1.0
# 修改
- "pose": weight=1.0 (统一 std)        → 替换为 variable_posture
- "foot_slip": weight=-0.1             → weight=-0.2
# 删除
- (无删除)
```

### 新增 Term 1：Variable Posture Reward 详解

Ch13 的 `pose` reward 对所有关节使用统一 std：

$$r_{\text{pose}} = -\sum_{j=1}^{N} \left(\frac{q_j - q_j^{\text{default}}}{\sigma}\right)^2$$

人形的 `variable_posture` 把 $\sigma$ 替换为 per-joint, per-state 的 $\sigma_j^{\text{state}}$：

$$r_{\text{vp}} = -\sum_{j=1}^{29} \left(\frac{q_j - q_j^{\text{default}}}{\sigma_j^{\text{state}}}\right)^2$$

其中 $\text{state} \in \{\text{standing}, \text{walking}, \text{running}\}$，由当前 command 速度的范数决定。$\sigma_j^{\text{state}}$ 的配置已在 14.2 中给出。

**实现要点**：在 mjlab 中，variable posture 通过自定义 reward function 实现，而不是标准的 `pose` term：

```python
def variable_posture_reward(
    env,
    joint_names: list[str],
    default_joint_pos: dict[str, float],
    std_standing: dict[str, float],
    std_walking: dict[str, float],
    std_running: dict[str, float],
    speed_thresholds: tuple[float, float] = (0.2, 1.0),
):
    """人形 variable posture reward function."""
    # 获取当前关节位置
    joint_pos = env.data.joint_pos[:, env.joint_ids]  # (N, 29)
    
    # 获取 command 速度范数
    cmd = env.command_manager.get_command("twist")  # (N, 3)
    cmd_speed = torch.norm(cmd[:, :2], dim=-1)  # (N,) 线速度范数
    
    # 选择 std
    is_standing = cmd_speed < speed_thresholds[0]
    is_running = cmd_speed > speed_thresholds[1]
    is_walking = ~is_standing & ~is_running
    
    # 构建 per-env, per-joint std tensor
    std = torch.zeros_like(joint_pos)
    for j, name in enumerate(joint_names):
        std[is_standing, j] = std_standing[name]
        std[is_walking, j] = std_walking[name]
        std[is_running, j] = std_running[name]
    
    # 计算偏差
    default = torch.tensor([default_joint_pos[n] for n in joint_names],
                           device=joint_pos.device)
    deviation = joint_pos - default
    
    # 加权 L2
    reward = -torch.sum((deviation / std) ** 2, dim=-1)
    
    return reward
```

**工程陷阱**：`speed_thresholds` 的选择影响策略行为。如果 standing→walking 的阈值太低（如 0.05 m/s），策略在接近零速命令时频繁在 standing 和 walking std 之间切换，导致关节抖动。建议使用 hysteresis（滞回）来避免频繁切换。

### 新增 Term 2：Angular Momentum Penalty 详解

```python
def angular_momentum_penalty(env, sensor_name: str = "root_angmom"):
    """惩罚以 pelvis 为根的子树角动量。"""
    angmom = env.data.sensor_data[sensor_name]  # (N, 3) 三轴角动量
    penalty = torch.sum(angmom ** 2, dim=-1)    # (N,) L2 范数
    return penalty
```

这个 reward 的权重（-0.02）是 Ch14 中最需要仔细调整的超参数之一：

| 权重 | 行为 | 适用场景 |
|------|------|---------|
| -0.005 | 几乎不约束，手臂可能乱甩 | 不推荐 |
| **-0.02** | 标准配置，平衡自然摆臂和约束 | 大多数场景 |
| -0.05 | 保守，步幅缩小但姿态更稳 | 高安全要求（sim2real 前） |
| -0.1 | 过保守，策略极小步幅蹭走 | 不推荐 |

### 新增 Term 3：Self Collision 详解

四足机器人的自碰撞风险很低——四条腿的运动范围不会交叉。人形不同：手臂可能碰到腿、腰可能过度扭曲导致上下半身碰撞。

```python
"self_collision": RewardTermCfg(
    func=self_collision_penalty,
    weight=-1.0,
    params={
        "asset_cfg": EntityCfg("robot"),
        # 只监测特定的碰撞对
        "body_pairs": [
            ("left_hand", "left_thigh"),
            ("right_hand", "right_thigh"),
            ("left_elbow", "torso"),
            ("right_elbow", "torso"),
        ],
    },
)
```

**工程要点**：不要监测所有碰撞对——这会产生大量 false positive（相邻 link 之间的正常接触也会被检测到）。只监测**异常碰撞对**——那些在正常运动中不应该发生的接触。

### 新增 Term 4：Foot Slip 权重加大

同样是 `foot_slip` reward，但权重从 Go1 的 -0.1 加大到 G1 的 **-0.2**。原因在 14.1 已经分析过：四足上脚滑只是 tracking 下降，人形上脚滑可能直接导致失衡。更大的惩罚权重让策略更积极地避免打滑。

### Reward 调参的人形特有工作流

在 Ch13 中，我们介绍了"先跑只有 tracking 的 baseline → 逐层添加 reward"的方法。在人形上，这个流程需要修改：

```
Step 1: 只有 tracking + upright + base_height（~2000 iterations）
  → 验证策略能走但可能姿态难看
  → 通过标准：tracking reward > 0.3，fall rate < 30%
  → 如果不通过：检查 action scale、default pose、command range

Step 2: 加入 variable_posture（walking std）（~2000 iterations）
  → 上肢行为应该明显改善
  → 通过标准：手臂不再乱甩，angular momentum 有所下降
  → 如果手臂仍乱甩：收紧 shoulder 和 elbow std

Step 3: 加入 angular_momentum（~2000 iterations）
  → 全身角动量应该下降，步态更稳
  → 通过标准：angular_momentum 指标下降 >50%
  → 如果步幅变得过小：降低 angular_momentum weight 到 -0.01

Step 4: 加入 foot_slip + self_collision（~2000 iterations）
  → 安全性提升
  → 通过标准：self_collision 几乎为零

Step 5: 逐步调整 variable_posture 的 running std（~2000 iterations）
  → 高速命令时步态更自然
  → 通过标准：高速时有明显摆臂但不乱甩
```

**关键区别**：四足上可以同时加入所有 reward 然后微调权重。人形上建议逐步添加——因为 reward 之间的交互更复杂（angular_momentum 和 variable_posture 会相互约束），一次性加入所有 term 后如果出问题，很难定位是哪个 term 导致的。

### 人形 Reward Ablation 实验指南

与 Ch13 的 reward ablation 方法类似，但针对人形新增 terms 有额外的实验建议：

| 实验 | 关闭的 term | 预期结果 | 观察指标 |
|------|-----------|---------|---------|
| Baseline | 无（全部开启） | 正常步态 | 所有指标 |
| No angular_momentum | angular_momentum | 手臂可能乱甩，腰部扭动增大 | root_angmom sensor 值 |
| No variable_posture | variable_posture | 上肢行为不受控，可能出现怪步态 | 视频观察 |
| No self_collision | self_collision | 手臂可能穿过身体 | 碰撞检测计数 |
| No base_height | base_height | 可能蹲走（降低重心换稳定性） | base height 指标 |
| No foot_slip 加大 | foot_slip 回到 -0.1 | 侧向移动时可能更滑 | tracking error |

**工程建议**：每个 ablation 只关闭一个 term，训练 3000 iterations，3 个 seed。用表格记录每组的 tracking error、fall rate、angular_momentum 均值和视频链接。这个实验大约需要 4090 上 3 × 6 × 1h = 18 GPU-hours——是验证你的 reward 设计是否合理的最高性价比投入。

### "G1 不走"的系统性排查流程

与 Ch13 的"策略不走"排查流程类似，但加入人形特有的检查项：

```
Step 1: 确认 command obs 存在（同 Ch13）
  → 打印 actor obs group，确认有 command term

Step 2: 确认 tracking reward 有信号（同 Ch13）
  → 查看 tensorboard 中 reward/track_linear_velocity

Step 3: 确认 action 有效
  → play 时打印 action tensor 的均值和标准差
  → 如果 std 接近零 → entropy 坍塌
  → ★ 人形特有：检查 per-joint action scale 是否正确
  →   如果某些关节 scale 是 0 → 那些关节不会动

Step 4: 确认 reward balance
  → 打印所有 reward term 的初始值
  → ★ 人形特有：angular_momentum penalty 是否过大？
  →   如果 angular_momentum 贡献 > tracking 贡献 → 策略选择不动
  →   临时把 angular_momentum weight 设为 0 确认

Step 5: 确认 default pose 稳定
  → ★ 人形特有：zero agent 能站 >1 秒吗？
  →   如果不能 → PD gains 太低或 default pose 不合理
  →   增大 kp 或调整 keyframe 中的关节角度

Step 6: 确认 termination 不过严
  → ★ 人形特有：fell_over 阈值是否太小（<30°）？
  →   episode 极短（<50 步）通常是 termination 的问题
  →   临时放宽到 70° 确认

Step 7: 确认不是上肢投机
  → ★ 人形特有：看视频时策略是否在用手臂做奇怪动作？
  →   如果是 → variable_posture std 太松
  →   收紧所有上肢 std 到 0.05 确认
```

80% 的"G1 不走"问题在 Step 1-4 就能定位。Step 5-7 是人形特有的检查项，只有在基本排查都通过后才需要。

### Command 配置的人形特有考虑

velocity task 的 command 是 $(v_x, v_y, \omega_z)$ 三维。四足上的典型 command 范围是 $v_x \in [-1, 2]$, $v_y \in [-0.5, 0.5]$, $\omega_z \in [-1, 1]$。人形上需要更保守：

```python
# G1 velocity command 配置
cfg.commands.twist = UniformVelocityCommandCfg(
    resampling_time_range=(5.0, 10.0),  # 比 Go1 的 (3, 8) 更长
    ranges={
        "lin_vel_x": (-0.5, 1.0),     # ★ 比 Go1 更保守（后退更危险）
        "lin_vel_y": (-0.2, 0.2),     # ★ 侧向大幅缩窄（侧向最危险）
        "ang_vel_z": (-0.5, 0.5),     # ★ 转向也更保守
    },
)
```

**为什么后退范围只有 -0.5 而前进有 1.0？** 人形后退时视觉和前庭觉的反馈不如前进——真机上后退碰到障碍物的风险更高。更重要的是，后退时脚跟先着地（前进时脚尖先着地），接触模式不同，策略需要学到完全不同的步态。在训练初期把后退范围缩窄，策略先把前进学好。

**为什么侧向只有 ±0.2？** 14.1 已经分析过——侧向运动直接挑战窄支撑面。$v_y = 0.5$ m/s 对 Go1 不难，但对 G1 可能导致 roll 发散。先用 ±0.2 建立稳定的侧向步态，然后通过 command curriculum 逐步扩大。

**Command Curriculum**：与 terrain curriculum 类似，command 范围也可以渐进式扩大：

```python
# command curriculum（概念性）
cfg.curriculum.command_ranges = CurriculumCfg(
    initial_ranges={"lin_vel_x": (-0.3, 0.5), "lin_vel_y": (-0.1, 0.1)},
    target_ranges={"lin_vel_x": (-0.5, 1.0), "lin_vel_y": (-0.3, 0.3)},
    metric="tracking_reward",  # 达到阈值后扩大范围
    threshold=0.5,
    steps=5,  # 分 5 步扩大
)
```

**反事实推理：如果一开始就用 $v_y \in [-0.5, 0.5]$ 会怎样？** 训练初期策略在侧向大速命令下频繁摔倒，episode 极短。PPO 的 rollout 中大量是失败经验，tracking reward 信号稀疏。策略可能学到"收到侧向大命令时站着不动"——因为"不动"比"尝试侧移但摔倒"的 return 更高。Command curriculum 避免了这个问题。

### 人形训练的 Tensorboard 阅读重点

除了 Ch13 介绍的标准指标（total reward, episode length, KL, entropy），人形训练需要额外关注以下 tensorboard panel：

| Panel | 健康趋势 | 异常信号 | 对应动作 |
|-------|---------|---------|---------|
| `reward/track_linear_velocity` | 上升到 >0.5 | 持续 <0.2 | 检查 command obs、action scale |
| `reward/upright` | 持续接近 0 | 持续负值 | 增大 upright 权重 |
| `reward/angular_momentum` | 下降并稳定 | 持续大负值 | 减小 angular_momentum 权重 |
| `reward/variable_posture` | 从负值趋近 0 | 持续大负值 | 放宽某些 std |
| `reward/self_collision` | 接近 0 | 频繁脉冲 | 检查 body_pairs 或收紧上肢 std |
| `reward/base_height` | 接近 0 | 持续负值 | 策略在蹲走，增大 base_height 权重 |
| `metric/angular_momentum_rms` | 下降 | 上升或震荡 | 手臂/腰部行为异常 |

**关键的交叉检查**：如果 `track_linear_velocity` 在上升但 `upright` 在下降，策略在**用前倾换速度**——这在 Go1 上少见但在 G1 上常见。此时不要调 tracking reward——应该增大 upright 权重或收紧 waist_pitch std。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：self_collision 监测了相邻 link**。如果 body_pairs 包含（thigh, shank）这种相邻 link 对，正常弯膝时就会触发自碰撞惩罚——策略会学到不弯膝的僵硬步态。只监测异常碰撞对。

💡 **概念误区：angular_momentum 权重应该和 tracking 一个量级**。angular_momentum 是 L2 范数，其数值量级可能远大于 exponential tracking reward（值域 [0,1]）。-0.02 的权重看起来很小，但乘以 angular_momentum 的实际值后贡献可能很大。先打印初始值确认量级。

🧠 **思维陷阱：variable posture 只影响美学**。variable posture 不只是让步态"好看"——它约束了策略的探索空间。松的 std 允许策略利用关节做任何动作，包括不安全的动作。variable posture 是一种隐式的安全约束。

### 练习

1. **[配置题]** 写出 G1 velocity task 的完整 RewardsCfg（14 个 terms），标注每个 term 属于四层框架的哪一层。如果只保留 Tracking + Regularization 两层（去掉 Style 和 Safety），预测策略会学到什么样的步态。
2. **[实验题]** 做一个 ablation：分别关闭 angular_momentum 和 variable_posture（每次只关闭一个），各训练 2000 iterations。比较 (a) 最终 tracking reward, (b) 视觉观察到的步态差异, (c) angular_momentum 的实际值。
3. **[跨章综合题，Ch06+Ch13+Ch14]** 结合 Ch06 的 reward 设计原理和 Ch13 的 exponential kernel 分析，解释为什么 tracking reward 使用 exponential kernel 而 angular_momentum 使用 L2 penalty。如果把 angular_momentum 也改成 exponential kernel（奖励角动量接近零）会怎样？

---

mjlab 的 G1 velocity task 配置你已经从 14.2-14.3 中理解了核心设计。但本书的双框架特色要求你也能在 Isaac Lab 中读懂和运行人形 velocity task。本节精读 Isaac Lab 的 H1 velocity task，与 mjlab G1 做对照。

## 14.4 Isaac Lab 人形 Velocity Task 配置精读 ⭐⭐⭐

> **这一节解决什么问题**：通过精读 Isaac Lab 的 H1 velocity task，建立人形任务的双框架心智模型。

### Isaac Lab 的人形 velocity task 入口

Isaac Lab 内置了 Unitree H1 的 velocity task：`Isaac-Velocity-Flat-Unitree-H1-v0` 和 `Isaac-Velocity-Rough-Unitree-H1-v0`。H1 有 19 个 actuated joints（比 G1 少 10 个——没有手腕，手臂更简单）。

mjlab 侧通过 unitree_rl_mjlab 提供 G1 velocity task：`Mjlab-Velocity-Flat-Unitree-G1`。

| 维度 | mjlab G1 | Isaac Lab H1 |
|------|---------|-------------|
| 机器人 | Unitree G1 (29-DoF) | Unitree H1 (19-DoF) |
| 身高/体重 | 1.32m / 35kg | 1.8m / 47kg |
| 模型格式 | MJCF | USD (从 URDF 转换) |
| obs group 名 | actor / critic | policy / critic |
| action space | per-joint scale | per-joint scale |
| RL 后端 | RSL-RL | RSL-RL |
| terrain | MuJoCo Warp heightfield | PhysX terrain |

### H1 vs G1 的关节对比

| 关节组 | H1 (19-DoF) | G1 (29-DoF) | 差异影响 |
|--------|------------|------------|---------|
| 腿部 | 5×2 = 10 | 6×2 = 12 | H1 少 ankle_roll（脚掌平衡更依赖 hip） |
| 腰部 | 1 (yaw only) | 3 (yaw/roll/pitch) | H1 上下半身耦合更弱 |
| 手臂 | 4×2 = 8 | 4×2 = 8 | 相同 |
| 手腕 | 0 | 3×2 = 6 | G1 有更精细的末端控制 |
| 手指 | 1 (gripper) | 0 | H1 有简单夹持能力 |

最关键的差异是 **H1 没有 ankle_roll**。这意味着 H1 无法通过踝关节的内外翻来调整侧向平衡——必须完全依赖髋关节和步幅调整。这让 H1 的侧向稳定性控制更难，但也更像人类的行走模式（人类的踝 roll 范围也很小）。

### Isaac Lab H1 配置精读

```python
# Isaac Lab H1 velocity task 配置（简化版）
@configclass
class UnitreeH1RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    
    # 机器人
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        robot=ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=UsdFileCfg(
                usd_path="unitree_h1/h1.usd",
                activate_contact_sensors=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 1.05),  # H1 更高
                joint_pos={
                    ".*_hip_yaw": 0.0,
                    ".*_hip_roll": 0.0,
                    ".*_hip_pitch": -0.28,
                    ".*_knee": 0.79,
                    ".*_ankle": -0.52,
                    "torso": 0.0,
                    ".*_shoulder_pitch": 0.28,
                    ".*_shoulder_roll": 0.0,
                    ".*_shoulder_yaw": 0.0,
                    ".*_elbow": 0.52,
                },
            ),
            actuators={
                "legs": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip_.*", ".*_knee", ".*_ankle"],
                    stiffness=150.0,
                    damping=5.0,
                    effort_limit=300.0,
                ),
                "arms": ImplicitActuatorCfg(
                    joint_names_expr=[".*_shoulder_.*", ".*_elbow", "torso"],
                    stiffness=40.0,
                    damping=10.0,
                    effort_limit=87.0,
                ),
            },
        ),
    )
    
    # Reward 配置
    rewards: RewardsCfg = RewardsCfg(
        track_lin_vel_xy_exp=RewTerm(func=..., weight=1.5),
        track_ang_vel_z_exp=RewTerm(func=..., weight=0.75),
        lin_vel_z_l2=RewTerm(func=..., weight=-2.0),
        ang_vel_xy_l2=RewTerm(func=..., weight=-0.05),
        dof_torques_l2=RewTerm(func=..., weight=-1.0e-5),
        dof_acc_l2=RewTerm(func=..., weight=-2.5e-7),
        action_rate_l2=RewTerm(func=..., weight=-0.01),
        feet_air_time=RewTerm(func=..., weight=0.125),
        undesired_contacts=RewTerm(func=..., weight=-1.0),
        flat_orientation_l2=RewTerm(func=..., weight=-1.0),
    )
```

**Isaac Lab 与 mjlab 的 Reward 命名差异**：

| mjlab 命名 | Isaac Lab 命名 | 语义 |
|-----------|---------------|------|
| `track_linear_velocity` | `track_lin_vel_xy_exp` | 线速度跟踪 |
| `upright` | `flat_orientation_l2` | 基座水平 |
| `dof_acceleration` | `dof_acc_l2` | 关节加速度 |
| `foot_clearance` | — (需自定义) | 抬脚高度 |
| `variable_posture` | — (需自定义) | 自适应姿态约束 |
| `angular_momentum` | — (需自定义) | 角动量惩罚 |

注意 Isaac Lab 的 H1 内置配置**没有** variable_posture 和 angular_momentum——这两个人形特有 reward 需要用户自定义添加。这是 Isaac Lab 和 mjlab 在"开箱即用"程度上的差异：mjlab 的 G1 配置已经包含了这些人形特有设计；Isaac Lab 的 H1 配置更基础，需要用户根据需要扩展。

### env.step() 执行顺序（人形特殊注意点）

Ch13 已经精读了两个框架的 env.step() 执行顺序。在人形上，有两个额外需要关注的点：

1. **self_collision 检测的时机**：在 reward 计算之前。如果在 reward 之后才检测，策略可能在某一步获得了高 tracking reward 但同时发生了自碰撞——reward 信号会鼓励这种危险行为。

2. **angular_momentum sensor 的更新时机**：subtreeangmom 在 sim.step() 之后、reward 计算之前更新。如果 sensor 更新延迟一步，angular_momentum penalty 使用的是上一步的数据——这会削弱惩罚效果。确认 sensor update 在 reward 之前。

### humanoid-gym 的 sim-to-sim 验证思路

humanoid-gym (Gu et al., RSS 2024 Best Paper Finalist, `roboterax/humanoid-gym`) 首创了 Isaac Gym → MuJoCo 的 sim-to-sim 验证层。它的代码结构值得精读——虽然基于旧版 Isaac Gym（非 Manager-Based），但验证方法论直接适用于本章的双框架对比。

**humanoid-gym 的核心创新**：

1. **sim-to-sim 验证层**：在 Isaac Gym 训练后导出 policy，在 MuJoCo 中加载同一 URDF/MJCF 和 policy 进行验证。这比直接上真机安全且便宜。
2. **Symmetric mirror augmentation**：人形行走是左右对称的。在训练数据中添加镜像版本（左右腿/臂对调），等效于 2× sample efficiency。
3. **Reference-trajectory phase variable**：为步态周期添加显式的相位信号，帮助策略学到有节奏的步态。

```python
# humanoid-gym 的 mirror augmentation 概念
def mirror_obs(obs, left_indices, right_indices):
    """交换左右肢体的 observation"""
    mirrored = obs.clone()
    mirrored[:, left_indices] = obs[:, right_indices]
    mirrored[:, right_indices] = obs[:, left_indices]
    # 侧向速度取反
    mirrored[:, vy_index] = -obs[:, vy_index]
    mirrored[:, yaw_index] = -obs[:, yaw_index]
    return mirrored
```

在 RSL-RL 4.0 中，mirror augmentation 已经被集成为 `data_augmentation_func`，不需要手动实现。但理解原理很重要——特别是在你需要为自定义人形（非对称机器人，如单臂人形）禁用或修改这个功能时。

**humanoid-gym 的系统性 DR 策略**：humanoid-gym 的 DR 配置为人形量身设计——mass ±15%、friction ±30%、motor strength ±20%、IMU noise（gyro bias + accel noise）。注意这些范围比四足更保守——符合 14.1 的分析：人形对 DR 更敏感，需要更小范围的渐进式引入。

### Isaac Lab H1 训练的完整命令流程

与 mjlab G1 的 30 分钟工程流程对照，以下是 Isaac Lab H1 的等价流程：

```bash
# Step 1: 确认任务注册
python -m isaaclab.envs --list | grep H1

# Step 2: Small train
python scripts/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-H1-v0 \
    --num_envs 256 --max_iterations 50 \
    --headless

# Step 3: Large train
python scripts/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-H1-v0 \
    --num_envs 4096 --max_iterations 5000 \
    --run_name h1_flat_baseline \
    --headless

# Step 4: Play trained checkpoint
python scripts/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-H1-v0 \
    --num_envs 4 \
    --load_run h1_flat_baseline \
    --checkpoint model_5000.pt
```

**Isaac Lab 与 mjlab 命令差异总结**：

| 操作 | mjlab | Isaac Lab |
|------|-------|-----------|
| 训练 | `uv run train <task_id> ...` | `python scripts/rsl_rl/train.py --task <task_id> ...` |
| 播放 | `uv run play <task_id> ...` | `python scripts/rsl_rl/play.py --task <task_id> ...` |
| Zero agent | `--agent zero` | 无内置（用未训练 checkpoint） |
| 环境数 | `--env.scene.num-envs` | `--num_envs` |
| GPU | `--gpu-ids "[0]"` | `CUDA_VISIBLE_DEVICES=0` |
| Headless | `--viewer none` | `--headless` |

### G1 vs H1 Observation 维度对比

| Observation Term | G1 (29-DoF) 维度 | H1 (19-DoF) 维度 | 差异 |
|-----------------|-----------------|-----------------|------|
| base_lin_vel | 3 | 3 | 同 |
| base_ang_vel | 3 | 3 | 同 |
| projected_gravity | 3 | 3 | 同 |
| command | 3 | 3 | 同 |
| joint_pos | **29** | **19** | G1 多 10 (手腕) |
| joint_vel | **29** | **19** | G1 多 10 |
| last_action | **29** | **19** | G1 多 10 |
| **actor obs 总维度** | **~99** | **~69** | 差 30 |

这个维度差异意味着 G1 的策略网络需要处理更多输入维度。如果你在 G1 上训练效果不好但 H1 上正常，首先检查是否因为 obs 维度增大导致网络容量不足——可能需要增大 hidden_dims（如从 `[512,256,128]` 到 `[768,512,256]`）。

### 人形诊断矩阵

当人形训练出现问题时，以下矩阵帮助你快速定位原因：

| 现象 | 优先读数 | 源码落点 | 单变量动作 |
|------|---------|---------|-----------|
| 站立抖动 | zero cmd action_rate | pose std / action scale | 只调 standing std |
| 低头冲刺 | torso pitch / upright | upright reward weight | 收紧 waist_pitch std |
| 侧向倒 | torso roll / hip roll | lin_vel_y range | 缩窄 y curriculum |
| 转向摔 | yaw cmd / body_ang_vel | heading cfg | 只降 yaw range |
| 手臂乱甩 | root_angmom / self_collision | variable posture std | 只调上肢 std |
| 脚掌抖 | ankle roll / contact force | foot collision geom | 收紧 ankle_roll std |
| rough 绊 | foot height / ray scan | rough sensor cfg | 固定物理查 scan |
| 膝盖内扣 | knee angle / hip_roll | action scale / default pose | 检查 knee 范围 |
| 蹲着走 | base_height / knee angle | base_height reward | 增大 base_height 权重 |

**使用方法**："现象"列描述你从视频中观察到的行为。按"优先读数"在 tensorboard 中找到对应指标，确认是否异常。然后去"源码落点"对应的配置文件检查。最后做"单变量动作"——每次只改一个参数，不要同时调多个。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：Isaac Lab H1 的 obs group 名是 `policy` 而不是 `actor`**。Ch13 已经提到这个差异，但在人形上更容易犯错——因为你可能在 mjlab 中调试好 G1，然后复制 obs group 名到 Isaac Lab 的 H1 配置中。

💡 **概念误区：H1 配置可以直接套用到 G1**。H1 和 G1 的关节数、力矩范围、身高体重都不同。即使都是人形，配置也不能互换——action scale、default pose、reward 权重全部需要重新计算。

🧠 **思维陷阱：Isaac Lab 内置 H1 配置已经是最优的**。内置配置是一个能跑的 baseline，但通常不是最优的——缺少 variable_posture 和 angular_momentum 意味着策略可能学到不自然的步态。把它当作起点，用 14.3 的 reward 增强来改进。

### 练习

1. **[对照题]** 列出 mjlab G1 和 Isaac Lab H1 velocity task 的 reward terms 对照表。标注哪些是共有的、哪些是框架特有的、哪些需要用户自定义。
2. **[配置题]** 为 Isaac Lab H1 添加 `angular_momentum` reward term。写出完整的 `RewTerm` 配置，包括 func、weight 和 params。需要先确认 H1 USD 中是否有 `subtreeangmom` sensor——如果没有，说明如何添加。
3. **[跨框架实践]** 如果你要在 Isaac Lab 中实现 mjlab 的 `variable_posture` reward，需要哪些修改？列出 (a) 需要读取的数据（joint_pos, command）, (b) 需要的配置参数（std_table）, (c) 实现中需要注意的 tensor 操作。

### 增强 Isaac Lab H1 配置的工程指南

Isaac Lab 内置的 H1 velocity 配置缺少 14.3 讨论的人形特有 reward。以下是逐步增强的步骤：

**Step 1：添加 angular_momentum sensor**

如果 H1 USD 中没有 `subtreeangmom` sensor，需要在 scene 配置中添加：

```python
# 在 scene 配置中添加 angular momentum sensor
scene.angular_momentum_sensor = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/pelvis",
    update_period=0.0,  # 每步更新
    history_length=0,
    track_angular_momentum=True,  # Isaac Lab 需要此选项
)
```

在 MuJoCo (mjlab) 中，`subtreeangmom` 是 MJCF 内置 sensor。在 Isaac Lab (PhysX) 中，需要通过 rigid body dynamics API 手动计算。如果内置 sensor 不支持，你可以用自定义 obs term 实现：

```python
def angular_momentum_obs(env, asset_cfg):
    """通过 rigid body 数据计算角动量"""
    robot = env.scene[asset_cfg.name]
    # 获取所有 body 的速度和惯性
    body_vel = robot.data.body_ang_vel_w  # (N, num_bodies, 3)
    body_inertia = robot.data.body_inertia_w  # (N, num_bodies, 3, 3)
    # 简化：用 base body 的角动量近似
    angmom = torch.bmm(
        body_inertia[:, 0],  # pelvis inertia
        body_vel[:, 0].unsqueeze(-1)  # pelvis angular velocity
    ).squeeze(-1)  # (N, 3)
    return angmom
```

**Step 2：添加 variable_posture reward**

Isaac Lab 使用 `RewTerm` 而不是 mjlab 的 `RewardTermCfg`。注册方式略有不同：

```python
# 在 Isaac Lab env cfg 中注册自定义 reward
from omni.isaac.lab.managers import RewardTermCfg as RewTerm

@configclass
class CustomRewardsCfg:
    # 保留原有 terms...
    track_lin_vel = RewTerm(func=..., weight=1.5)
    # ...
    
    # 新增人形 terms
    variable_posture = RewTerm(
        func=variable_posture_reward_isaac,  # 自定义函数
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "std_standing": H1_POSTURE_STD_STANDING,
            "std_walking": H1_POSTURE_STD_WALKING,
            "std_running": H1_POSTURE_STD_RUNNING,
        },
    )
    angular_momentum = RewTerm(
        func=angular_momentum_penalty_isaac,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    self_collision = RewTerm(
        func=self_collision_penalty_isaac,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "body_pairs": [("left_elbow_link", "torso_link"), ...],
        },
    )
```

**Step 3：调整 H1 的 variable posture std**

H1 没有手腕关节，std 表只需 19 行（vs G1 的 29 行）。H1 没有 ankle_roll，侧向平衡完全依赖 hip——因此 hip_roll 的 std 应该比 G1 更放松（允许更大的髋部侧摆来补偿缺失的 ankle_roll）。

| H1 关节 | standing | walking | running | 与 G1 差异 |
|---------|----------|---------|---------|-----------|
| hip_roll | 0.08 | 0.15 | 0.15 | G1 用 0.05/0.10/0.10 → H1 更松（补偿无 ankle_roll） |
| torso (waist_yaw) | 0.05 | 0.10 | 0.10 | H1 只有 1-DoF 腰，约束与 G1 相似 |

这个增强流程完成后，Isaac Lab H1 的训练效果应该显著改善——步态更自然、手臂更受控、角动量更稳定。

### ONNX 跨框架兼容性

从 mjlab 训练的 G1 策略导出的 ONNX 文件，和从 Isaac Lab 训练的 H1 策略导出的 ONNX 文件，内部结构相同——都是 actor 网络（MLP），输入是 obs tensor，输出是 action tensor。但**它们不能互换使用**，因为：

1. **obs 维度不同**：G1 29-DoF 的 actor obs ~99 维，H1 19-DoF 的 actor obs ~69 维
2. **action 维度不同**：G1 输出 29 维，H1 输出 19 维
3. **obs 语义不同**：即使同名 term（如 joint_pos），G1 的第 15 维是 left_shoulder_pitch，H1 的第 15 维可能是不同的关节
4. **归一化参数不同**：每个框架训练时的 running mean/var 基于各自的数据分布

跨机器人（G1 → H1）的策略迁移不是简单的 ONNX 复制——需要重新训练或至少 fine-tune。但跨框架（mjlab G1 → Isaac Lab G1）的策略迁移是可能的，只要 obs 对齐（14.5 的核心内容）。

---

## 14.5 sim-to-sim 验证：Isaac Lab → MuJoCo 交叉验证 ⭐⭐

> **这一节解决什么问题**：学习人形策略的跨引擎验证方法，建立 sim-to-real 之前的鲁棒性评估能力。

### 为什么人形需要 sim-to-sim

Ch13 已经介绍了 sim-to-sim 的概念。在人形上，sim-to-sim 验证**更加重要**，原因有三：

1. **人形真机测试风险更高**。Go1 摔倒了弹一下就好；G1 从 0.76m 高度摔下可能损坏精密关节和传感器。sim-to-sim 是零成本的预筛选。
2. **人形对物理引擎差异更敏感**。窄支撑面意味着接触模型的微小差异（MuJoCo 凸优化 vs PhysX TGS）会被放大。四足上两个引擎可能都能走好，人形上一个引擎走得好另一个可能摔倒。
3. **人形的 reward hacking 更隐蔽**。四足的 reward hacking（如利用接触 bug）通常很明显——机器人会做出不自然的动作。人形的 reward hacking 可能表现为"看起来在走但实际上在利用特定引擎的接触特性"——只有跨引擎验证才能暴露这种过拟合。

### HOVER 提供的 sim-to-sim 基础设施

HOVER（`NVlabs/HOVER`）的代码仓库提供了完整的 sim-to-sim 验证工具：

```
neural_wbc/
├── isaac_lab_wrapper/     # Isaac Lab 训练环境
├── mujoco_wrapper/        # MuJoCo 验证环境
├── hw_wrappers/           # 真机部署接口
└── inference_env/         # 通用推理环境
```

验证流程：

```bash
# Step 1: 在 Isaac Lab 中训练
python scripts/rsl_rl/train_teacher_policy.py \
    --num_envs 4096 \
    --reference_motion_path data/stable_punch.pkl

# Step 2: 在 MuJoCo 中验证
python scripts/rsl_rl/play.py \
    --checkpoint_path logs/teacher/model_80000.pt \
    --sim mujoco \
    --num_envs 1 \
    --render
```

关键的 `--sim mujoco` 参数切换到 MuJoCo 后端进行验证。HOVER 的 mujoco_wrapper 自动处理了 observation 和 action 空间的对齐。

### 实际操作：从 mjlab 训练到 Isaac Lab 验证

如果你在 mjlab 中训练 G1，想在 Isaac Lab 中验证（反方向），流程是：

```bash
# Step 1: 在 mjlab 中训练并导出 ONNX
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
    --env.scene.num-envs 4096 --agent.max-iterations 5000

# ONNX 自动导出到 logs/ 目录

# Step 2: 在 Isaac Lab 中加载 ONNX
# 需要确保 obs 维度和顺序对齐
python eval_onnx.py \
    --onnx_path logs/rsl_rl/g1_velocity/model.onnx \
    --task Isaac-Velocity-Flat-Unitree-G1-v0 \
    --num_envs 4
```

**关键注意事项**：

1. **obs 维度和顺序必须对齐**。mjlab actor obs 的 term 顺序可能与 Isaac Lab 不同——ONNX 模型期望特定的输入布局。在导出前打印两个框架的 obs term 列表并逐一对照。

2. **归一化参数必须一致**。如果 mjlab 训练时使用了 running normalization，ONNX 中会包含 obs_mean 和 obs_var。Isaac Lab 端必须使用相同的归一化参数，否则输入分布不匹配。

3. **action 后处理必须一致**。per-joint action scale、default pose offset 在两个框架中可能用不同的数据结构表示。

### 跨引擎验证的定量评估

| 指标 | 同引擎基线 | 跨引擎目标 | 不可接受 |
|------|----------|----------|---------|
| Tracking error (m/s) | 0.10-0.15 | < 0.25 | > 0.5 |
| Fall rate (%) | < 5% | < 20% | > 50% |
| Episode length | > 800 步 | > 400 步 | < 100 步 |
| Angular momentum (Nm·s) | < 5 | < 10 | > 20 |

如果跨引擎 fall rate > 50%，通常不是"验证失败"那么简单——它暴露了策略对特定物理引擎的过拟合。解决方案：加大 DR（特别是 friction 和 contact 相关的参数），迫使策略学习更鲁棒的行为。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：obs 顺序不一致导致跨引擎评估全部失败**。这是最常见的问题。mjlab 和 Isaac Lab 的 obs term 注册顺序可能不同，导致 ONNX 模型收到的输入"乱套"。必须在两端打印 obs term name 和 shape 逐一对照。

💡 **概念误区：跨引擎性能下降就是失败**。一定程度的性能下降是正常的——两个物理引擎的接触模型确实不同。关键是判断下降是否在可接受范围内。tracking error 从 0.12 上升到 0.20 是正常的；从 0.12 上升到 0.50 是过拟合信号。

### 练习

1. **[实验题]** 在 mjlab 中训练一个 G1 flat velocity 策略（3000 iterations）。导出 ONNX 后，在 mjlab 中用不同的 friction 系数（0.5, 1.0, 1.5）评估。记录 tracking error 和 fall rate 的变化——这模拟了跨引擎差异。
2. **[分析题]** HOVER 的 `mujoco_wrapper` 需要处理哪些 Isaac Lab → MuJoCo 的差异？列出至少 3 个需要对齐的方面。
3. **[实践题]** 编写一个 obs 对齐检查脚本：分别在 mjlab 和 Isaac Lab 中 reset 一次环境，打印两端的 obs term 名字和 shape，逐一对照。如果发现不一致，说明如何修复。

### Obs 对齐检查的工程实现

跨引擎验证中最容易出错的环节是 obs 对齐。以下脚本可以帮助你快速检查：

```python
# obs_alignment_check.py
import torch

def print_obs_layout(env, framework_name):
    """打印 env 的 obs term 布局"""
    print(f"\n{'='*60}")
    print(f"  {framework_name} Observation Layout")
    print(f"{'='*60}")
    
    obs_dict = env.observation_manager.compute()
    offset = 0
    for group_name, group_data in obs_dict.items():
        print(f"\nGroup: {group_name}")
        if isinstance(group_data, dict):
            for term_name, tensor in group_data.items():
                dim = tensor.shape[-1]
                print(f"  [{offset:3d}:{offset+dim:3d}] {term_name:30s} dim={dim}")
                offset += dim
        else:
            dim = group_data.shape[-1]
            print(f"  [{0:3d}:{dim:3d}] concatenated tensor dim={dim}")
    
    print(f"\nTotal obs dim: {offset}")
    return offset

# 使用示例
# mjlab 端
# total_mjlab = print_obs_layout(mjlab_env, "mjlab G1")
# Isaac Lab 端
# total_isaac = print_obs_layout(isaac_env, "Isaac Lab H1")
# assert total_mjlab == total_isaac, f"Obs dim mismatch: {total_mjlab} vs {total_isaac}"
```

这段代码的输出类似于：

```
============================================================
  mjlab G1 Observation Layout
============================================================

Group: actor
  [  0:  3] base_lin_vel                   dim=3
  [  3:  6] base_ang_vel                   dim=3
  [  6:  9] projected_gravity              dim=3
  [  9: 12] command                        dim=3
  [ 12: 41] joint_pos                      dim=29
  [ 41: 70] joint_vel                      dim=29
  [ 70: 99] last_action                    dim=29

Total obs dim: 99
```

如果 Isaac Lab 端的 term 顺序不同（比如 `command` 在 `projected_gravity` 之前），ONNX 模型收到的第 6-8 维是 command 而不是 projected_gravity——策略行为完全错乱。修复方式：在 Isaac Lab 端添加一个 obs 重排层，或者修改 obs term 注册顺序使其与 mjlab 一致。

### DR 鲁棒性扫描作为 sim-to-sim 的简化替代

如果你暂时只有一个框架（比如只有 mjlab），可以用 **DR 鲁棒性扫描**模拟跨引擎效果：

```bash
# 基线：标准 friction
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --checkpoint-file model.pt --num-envs 100 \
  --env.events.physics_material.params.static_friction_range "(0.8, 0.8)"

# 低 friction（模拟 PhysX 的不同接触特性）
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --checkpoint-file model.pt --num-envs 100 \
  --env.events.physics_material.params.static_friction_range "(0.4, 0.4)"

# 高 friction
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --checkpoint-file model.pt --num-envs 100 \
  --env.events.physics_material.params.static_friction_range "(1.5, 1.5)"

# 外部推力
uv run play Mjlab-Velocity-Flat-Unitree-G1 \
  --checkpoint-file model.pt --num-envs 100 \
  --env.events.push_robot.params.velocity_range "(-1.5, 1.5)"
```

在每个配置下记录 tracking error 和 fall rate。如果策略在低 friction 下 fall rate 从 5% 跳到 60%，说明它严重依赖当前的摩擦设定——跨引擎验证大概率也会失败。加大训练时的 friction DR 范围可以缓解。

---

前面五节建立了人形 velocity tracking 的完整工程方法。但 velocity tracking 只是人形控制的起点——更高级的应用需要一个统一的 whole-body controller 支持多种控制模态（导航、操作、遥操作）。HOVER 正是这样一个系统，它展示了如何用一个策略同时支持多种控制需求。

## 14.6 精读：HOVER 的统一 Whole-Body Controller ⭐⭐⭐

> **这一节解决什么问题**：通过精读 HOVER 的架构和训练管线，理解 mask-conditioned distillation 如何让一个策略支持多种控制模态。

### HOVER 是什么

HOVER (Humanoid Versatile Controller) 由 NVIDIA 团队开发（Tairan He, Wenli Xiao, Toru Lin, Zhengyi Luo 等，ICRA 2025），代码在 `NVlabs/HOVER`。它的核心贡献是：

> **Key insight**：全身运动模仿（full-body kinematic motion imitation）可以作为所有人形控制任务的**公共抽象**。不同控制模态（导航、操作、遥操作）只是对不同身体部位的约束——通过 mask 选择哪些部位需要跟踪，一个统一策略就能覆盖所有模态。

### 架构概览

HOVER 采用 **teacher-student 两阶段训练**：

```
Stage 1: Oracle Teacher (全信息)
  - 输入: proprio + 完整参考姿态 (全身 kinematic target)
  - 输出: 关节位置目标
  - 训练: PPO, ~50k-80k iterations, 4096 envs
  
Stage 2: Multi-Mode Student (mask-conditioned)
  - 输入: proprio + 部分参考姿态 (masked command)
  - 输出: 关节位置目标
  - 训练: DAgger 蒸馏 from teacher, ~10k iterations
  
  Masks:
    - 全身跟踪 → mask = [1,1,1,1,...,1] (所有身体部位)
    - 导航 → mask = [1,0,0,...] (只有 root 位置/朝向)
    - 上半身操作 → mask = [0,1,1,0,0] (只有手臂)
    - 遥操作 → mask = [1,1,0,...] (头 + 手)
```

**跨领域类比**：mask-conditioned distillation 类似于 NLP 中的 masked language modeling（BERT）。BERT 通过随机 mask 一些 token 来学习上下文理解能力。HOVER 通过随机 mask 一些身体部位的跟踪目标来学习"即使部分信息缺失也能保持全身协调"的能力。

### 代码结构

```
NVlabs/HOVER/
├── neural_wbc/
│   ├── core/                          # 核心算法
│   │   ├── teacher_policy.py          # Oracle teacher
│   │   ├── student_policy.py          # Mask-conditioned student
│   │   └── distillation.py            # DAgger 蒸馏
│   ├── isaac_lab_wrapper/             # Isaac Lab 训练环境
│   │   ├── neural_wbc_env_cfg_h1.py   # H1 配置
│   │   └── neural_wbc_env_cfg_g1.py   # G1 配置
│   ├── mujoco_wrapper/                # MuJoCo sim-to-sim
│   └── hw_wrappers/                   # 真机部署
├── scripts/rsl_rl/
│   ├── train_teacher_policy.py        # 训练 teacher
│   ├── train_student_policy.py        # 训练 student
│   ├── play.py                        # 评估
│   └── eval.py                        # 定量评估
└── third_party/human2humanoid         # SMPL→robot retargeting
```

### Teacher 训练的 Reward 设计

HOVER teacher 的 reward 核心是全身 kinematic tracking：

```python
# HOVER teacher reward (概念性简化)
r_body = exp(-1/N * sum(||p_i^ref - p_i^sim||^2) / sigma_body^2)  # 身体关键点
r_root = exp(-||root_pos^ref - root_pos^sim||^2 / sigma_root^2)     # root 位置
r_joint = exp(-||q^ref - q^sim||^2 / sigma_joint^2)                 # 关节角度
r_vel = exp(-||v^ref - v^sim||^2 / sigma_vel^2)                     # 速度匹配

r_total = w_body * r_body + w_root * r_root + w_joint * r_joint + w_vel * r_vel
```

与本章 14.3 的 velocity task reward 对比：velocity task 用 twist command 作为目标，HOVER teacher 用 full kinematic reference 作为目标。HOVER 的 reward 更像 Ch15 的 motion tracking，但 teacher 阶段的目标是建立**全身运动能力**，而不是跟踪特定动作。

### Student 蒸馏的 Mask 机制

Student 的核心创新是 **distill_mask_modes**：

```python
# mask modes 配置 (概念性)
distill_mask_modes = {
    "full_body": [1, 1, 1, 1, 1, 1],      # 全身跟踪
    "navigation": [1, 0, 0, 0, 0, 0],      # 只跟踪 root
    "upper_body": [0, 1, 1, 0, 0, 0],      # 只跟踪手臂
    "head_hands": [0, 0, 0, 1, 1, 0],      # 头 + 手（遥操作）
}
# mask 对应: [root, left_arm, right_arm, head, left_hand, right_hand]
```

训练时，每个 episode 随机选择一个 mask mode。被 mask 掉的身体部位的跟踪目标设为零（或 default pose）。student 在训练过程中看到了各种 mask 组合，学会了"当某些部位没有明确目标时，也能保持整体平衡和协调"。

**工程含义**：蒸馏完成后，inference 时只需指定 mask 就能切换控制模态——不需要重新训练。这是 HOVER 相比 per-task 训练的核心优势：per-task 需要为每种模态各训练一个策略（N 个策略），HOVER 只需一个策略 + N 个 mask。

### 训练资源需求

| 阶段 | envs | iterations | RTX 4090 时间 | GPU 内存 |
|------|------|-----------|-------------|---------|
| Teacher | 4096 (推荐) | 50k-80k | ~12-23 h | ~8 GB |
| Student | 4096 (推荐) | ~10k | ~16 min | ~6 GB |

Teacher 需要的时间远多于 student——这是因为 teacher 需要从零学习全身运动能力，而 student 只需要从 teacher 的经验中蒸馏。HOVER README 特别指出："For good results we recommend to train with at least 4096 environments."

### HOVER 作为 Isaac Lab Extension 的模式

HOVER 代码仓库是一个标准的 **Isaac Lab extension**——这意味着它不修改 Isaac Lab 核心代码，而是通过 extension 机制注册新的 env、reward 和 action。这个模式是 Isaac Lab 生态中添加新功能的推荐方式：

```python
# Isaac Lab extension 注册
# 在 HOVER 的 setup.py 中
entry_points={
    "isaaclab.envs": [
        "neural_wbc = neural_wbc.isaac_lab_wrapper",
    ],
},
```

**工程建议**：如果你要在 Isaac Lab 中实现自定义人形任务，参考 HOVER 的 extension 模式而不是 fork Isaac Lab。extension 模式让你的代码独立于 Isaac Lab 版本，升级 Isaac Lab 时不需要重新合并。

### HOVER 完整训练工作流

以下是从安装到验证的完整步骤。假设你已安装 Isaac Lab 2.0.0 和 Isaac Sim 4.5。

```bash
# Step 0: 克隆 HOVER
git clone https://github.com/NVlabs/HOVER.git
cd HOVER
pip install -e .

# Step 1: 准备参考运动数据
# HOVER 自带几个示例运动（data/ 目录）
ls data/
# stable_punch.pkl, walking_forward.pkl, ...

# Step 2: 训练 Teacher (Oracle Policy)
python scripts/rsl_rl/train_teacher_policy.py \
    --num_envs 4096 \
    --max_iterations 80000 \
    --reference_motion_path data/stable_punch.pkl \
    --headless

# 训练指标监控（tensorboard）
# - reward/body_tracking: 身体关键点跟踪质量
# - reward/root_tracking: root 位置跟踪质量
# - episode_length: 应逐步增长
# - fall_rate: 应逐步下降

# Step 3: 评估 Teacher
python scripts/rsl_rl/play.py \
    --checkpoint_path logs/teacher/model_80000.pt \
    --num_envs 4 \
    --render

# Step 4: 训练 Student (Multi-Mode Distillation)
python scripts/rsl_rl/train_student_policy.py \
    --num_envs 4096 \
    --max_iterations 10000 \
    --teacher_checkpoint_path logs/teacher/model_80000.pt \
    --distill_mask_modes full_body navigation upper_body \
    --headless

# Step 5: 评估 Student（切换不同 mask mode）
# 全身跟踪模式
python scripts/rsl_rl/play.py \
    --checkpoint_path logs/student/model_10000.pt \
    --mask_mode full_body \
    --num_envs 4 --render

# 导航模式（只跟踪 root 位置）
python scripts/rsl_rl/play.py \
    --checkpoint_path logs/student/model_10000.pt \
    --mask_mode navigation \
    --num_envs 4 --render

# Step 6: Sim-to-Sim 验证
python scripts/rsl_rl/play.py \
    --checkpoint_path logs/student/model_10000.pt \
    --sim mujoco \
    --num_envs 1 --render
```

**训练过程中的关键判断点**：

| checkpoint | 检查项 | 不通过时的动作 |
|-----------|--------|------------|
| teacher 10k | body tracking reward > 0.3 | 检查 reference motion 质量 |
| teacher 30k | fall rate < 30% | 加大 upright reward 或收紧 command |
| teacher 50k | body tracking > 0.6, fall rate < 10% | 继续训练或开始 student |
| teacher 80k | body tracking > 0.7 | 导出 teacher，开始 student |
| student 5k | navigation mode 能走直线 | 检查 mask 配置 |
| student 10k | 所有 mode 工作正常 | 完成，进入 sim-to-sim |

### HOVER 与 GR00T-WholeBodyControl 的关系

2026 年 NVIDIA 发布了 `NVlabs/GR00T-WholeBodyControl`，这是 HOVER 的后续演化。GR00T-WBC 统一了三个子系统：

1. **Decoupled WBC**：用于 GR00T N1.5 / N1.6 的解耦全身控制
2. **GEAR-SONIC**：SONIC 行为基础模型（42M 参数，700 小时运动数据）
3. **MotionBricks**：交互式动作拼接

对于本教材的读者，HOVER 仍然是最好的学习入口——它的代码更简洁、依赖更少、训练资源需求更低。当你掌握了 HOVER 的 teacher-student + mask 模式后，GR00T-WBC 的扩展就很容易理解。

### ⚠️ 常见陷阱

⚠️ **编程陷阱：HOVER 依赖特定 Isaac Lab 版本**。HOVER README 明确说明 "Currently HOVER has been tested with Isaac Lab versions 2.0.0"。使用其他版本可能导致 API 不兼容。

💡 **概念误区：mask-conditioned = 简单地丢弃输入**。mask 不只是把某些 obs 设为零——它改变了 student 的学习信号。被 mask 的部位不贡献 tracking reward，student 学会在这些部位上维持默认行为（而不是随机动作）。

🧠 **思维陷阱：HOVER 可以替代 velocity task**。HOVER 是一个 whole-body controller，它的能力比 velocity task 更通用。但 HOVER 需要参考运动数据（kinematic reference），而 velocity task 只需要速度命令。如果你的应用只需要速度控制（如导航），velocity task 更简单直接。

### 练习

1. **[架构分析题]** HOVER 的 teacher-student 蒸馏与 Ch09（Privileged Learning）的 teacher-student 有什么异同？列出至少 3 个相同点和 2 个不同点。
2. **[设计题]** 如果你要为 HOVER 添加一个新的 mask mode "legs_only"（只跟踪腿部目标，上半身自由），mask 应该怎么设置？预测这种模式下策略的行为。
3. **[工程题]** HOVER 的 teacher 训练需要 reference motion。如何从 AMASS 数据集获取 G1 可用的参考运动？列出从 SMPL → G1 retarget 的关键步骤。

---

人形策略无论多稳定，真机部署中总有可能摔倒——外部推力、障碍物、滑倒等意外不可完全避免。摔倒后如何安全地站起来，是人形系统投入实际使用的必要条件。HoST 正是解决这个问题的专用系统。

## 14.7 精读：HoST 跌倒恢复与 Multi-Critic 架构 ⭐⭐

> **这一节解决什么问题**：通过精读 HoST 的 multi-critic 架构和训练管线，理解如何构建安全的跌倒恢复能力并集成到 locomotion pipeline 中。

### HoST 是什么

HoST (Humanoid Standing-up Control) 由上海 AI Lab / 上交 / 港大 / 浙大 / 港中文联合开发（Tao Huang, Junli Ren 等，RSS 2025 Best Systems Paper Finalist），代码在 `InternRobotics/HoST`。

它解决一个看似简单但工程上极具挑战的问题：**让人形机器人从任意摔倒姿态（仰卧、俯卧、侧卧）站起来**。

### 为什么站起来很难

传统方法为每种摔倒姿态设计一个固定的站起轨迹——仰卧一套动作、俯卧一套动作。这在平地上可以工作，但：
- 真实场景中摔倒姿态是连续的（不只是仰卧/俯卧/侧卧，还有各种中间姿态）
- 地形可能不是平的（在斜坡上站起来和平地不同）
- 关节有力矩限制——某些站起动作在动力学上不可行

HoST 用 RL 从零学习站起控制，让策略自己发现每种姿态下的最优站起路径。

### Multi-Critic 架构

标准 PPO 使用一个 critic 估计所有 reward 的总 value。但站起过程有多个阶段（翻身 → 撑地 → 起身 → 站稳），每个阶段的 reward 重要性不同。单 critic 在这种多阶段任务上容易出现 reward 之间的 negative interference——翻身阶段的 reward 进展可能抵消起身阶段的 reward 退步。

HoST 的方案：**为每组 reward 训练一个独立的 critic**：

```python
# Multi-critic PPO (概念性)
class MultiCriticPPO:
    def __init__(self, reward_groups):
        # 每组 reward 一个 critic
        self.critics = {
            "posture": Critic(),    # 姿态相关 reward
            "balance": Critic(),    # 平衡相关 reward  
            "progress": Critic(),   # 站起进度 reward
            "safety": Critic(),     # 安全相关 reward
        }
        self.actor = Actor()  # 共享一个 actor
    
    def compute_advantages(self, rewards_dict, obs):
        advantages = {}
        for group_name, critic in self.critics.items():
            # 每组 reward 独立计算 advantage
            values = critic(obs)
            advantages[group_name] = gae(
                rewards_dict[group_name], values, gamma, lam
            )
        # 合并所有组的 advantage
        total_advantage = sum(advantages.values())
        return total_advantage
```

**每个 critic 独立估计各自 reward 组的 value → 独立计算 advantage → 合并 advantage 更新 actor**。这避免了单个 critic 在不同 reward 组之间的 trade-off 困难。

### HoST 的四组 Reward 设计

HoST 将站起任务的 reward 分为四个独立的组，每组由自己的 critic 优化：

| Reward 组 | 目标 | 典型 terms | 阶段重要性 |
|-----------|------|-----------|-----------|
| **Posture** | 达到目标姿态 | `target_joint_pos`, `target_base_height`, `target_orientation` | 全程 |
| **Balance** | 维持平衡不二次摔倒 | `base_ang_vel_penalty`, `projected_gravity`, `com_in_support` | 中后期 |
| **Progress** | 向站立状态推进 | `base_height_progress`, `head_height`, `vertical_velocity` | 前中期 |
| **Safety** | 避免危险动作 | `action_rate`, `torque_limit`, `self_collision`, `smoothness` | 全程 |

**为什么不用单 critic？** 考虑一个具体场景：策略正在学习从仰卧翻身到俯卧。翻身过程中 Progress 组的 reward（base height 暂时下降）和 Posture 组的 reward（关节偏离目标更远）都会暂时恶化。单 critic 把所有 reward 加在一起，advantage 变成负数——策略被惩罚"尝试翻身"这个行为。Multi-critic 中，Progress critic 可以独立认识到"虽然现在 height 降了，但翻身后能站得更高"——它的 advantage 可以是正的，即使 Posture advantage 是负的。

**实现细节**：HoST 的 multi-critic 基于 L2C2（Learning to Coordinate Critics，Ren et al. 2024）的思想。每个 advantage 在归并前做**独立归一化**（zero mean, unit variance），避免某组 reward 因为数值量级大而主导 actor 更新。

```python
# HoST multi-critic advantage 归并（概念性）
def compute_total_advantage(reward_groups, critics, obs, gamma, lam):
    all_advantages = []
    for group_name, rewards in reward_groups.items():
        critic = critics[group_name]
        values = critic(obs)
        adv = gae(rewards, values, gamma, lam)
        # 关键：独立归一化
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        all_advantages.append(adv)
    
    # 合并：简单求和（各组已归一化，贡献等权）
    total_advantage = sum(all_advantages)
    return total_advantage
```

**反事实推理：如果不做独立归一化会怎样？** Progress 组的 reward 可能有 ±100 的量级（base height 变化大），而 Safety 组的 reward 可能只有 ±1 的量级。不归一化时 Progress 组完全主导 actor 更新——策略会学到"不惜一切代价站起来"，包括关节猛力一弹（Safety 组的 smoothness 惩罚被淹没）。

### Curriculum-Based 辅助力

训练初期，随机初始化的策略完全不知道怎么站起来——从任意摔倒姿态到站立的动作序列极长，reward 信号极稀疏。HoST 使用一个**垂直辅助力 curriculum**来解决冷启动问题：

```python
# 辅助力 curriculum（概念性实现）
class VerticalPullForceCurriculum:
    def __init__(self, robot_mass, gravity=9.81):
        self.mg = robot_mass * gravity  # G1: 35 * 9.81 ≈ 343 N
        self.force_ratio = 0.5  # 初始 50% 体重
        self.min_ratio = 0.0
        self.decay_rate = 0.9998  # 每 step 衰减
    
    def get_force(self, base_height, target_height=0.76):
        # 只在 base 低于目标高度时施加辅助力
        if base_height < target_height:
            force_magnitude = self.force_ratio * self.mg
            return torch.tensor([0.0, 0.0, force_magnitude])
        return torch.zeros(3)
    
    def step(self, success_rate):
        # 根据成功率调整衰减速度
        if success_rate > 0.7:
            self.force_ratio *= 0.995  # 成功率高时快速衰减
        else:
            self.force_ratio *= 0.9999  # 成功率低时慢衰减
        self.force_ratio = max(self.force_ratio, self.min_ratio)
```

训练进程：

| 阶段 | 辅助力比例 | 策略行为 | iterations |
|------|----------|---------|-----------|
| 冷启动 | 50% 体重 | 在辅助力帮助下学会翻身和撑地 | 0-5k |
| 过渡 | 25% 体重 | 学会用更多自身力矩站起来 | 5k-15k |
| 弱辅助 | 10% 体重 | 几乎独立站起，辅助力只是安全网 | 15k-25k |
| 独立 | 0% | 完全独立站起来 | 25k+ |

**跨领域类比**：辅助力 curriculum 就像教小孩骑自行车时的辅助轮。一开始辅助轮完全接触地面（50%），小孩只需要学会踏板和方向。逐步抬高辅助轮（减小力），小孩必须学会更多的平衡技巧。最终完全拆掉辅助轮（0%），小孩已经掌握了全部技巧。直接不给辅助轮（从零开始无辅助力）让大多数小孩一上来就摔倒——和初始策略在没有辅助力时完全无法站起来是同一回事。

### Smoothness Regularization 和隐式速度约束

为了 sim-to-real 部署安全，HoST 加入了两个关键约束：

**Smoothness regularization** 惩罚关节速度和加速度的高频分量：

```python
# smoothness reward terms
"joint_velocity_penalty": RewardTermCfg(
    func=joint_vel_l2, weight=-0.5,
),
"joint_acceleration_penalty": RewardTermCfg(
    func=joint_accel_l2, weight=-0.001,
),
"action_difference_penalty": RewardTermCfg(
    func=action_diff_l2, weight=-0.1,  # |a_t - a_{t-1}|^2
),
```

仿真中高频抖动可能产生有效力矩（利用接触弹性），但真机上这种抖动会损坏电机。HoST 论文指出："we constrain the motion with smoothness regularization and implicit motion speed bound to alleviate oscillatory and violent motions on physical hardware."

**Implicit motion speed bound** 不是显式设置速度限制，而是通过 reward 隐式约束。如果策略学到"猛力一弹"的站起方式——仿真中可以工作，但真机上的瞬时力矩可能超过安全限制。在 Safety reward 组中添加 `joint_velocity_penalty` 和 `joint_acceleration_penalty`，让策略自动倾向于平缓、渐进的站起动作。

### 多地形训练

HoST 不只在平地上训练站起——它在多种地形上训练：

| 地形 | 目的 | 对策略的挑战 |
|------|------|------------|
| 平地 | 基础能力 | 标准站起 |
| 软垫 | 模拟沙发/床 | 支撑面下沉，需要更大力矩 |
| 斜坡 | 户外场景 | 重力方向与支撑面不垂直 |
| 台阶边缘 | 室内场景 | 部分身体悬空 |

多地形训练使策略学到**姿态自适应**的站起策略——在不同表面上自动调整力的分配，而不是记忆固定的动作序列。

### 作为安全兜底集成到 Locomotion Pipeline

HoST 的实际部署方式是作为 locomotion pipeline 的**安全层**：

```
正常运行:
  locomotion_policy → joint commands → robot

检测到摔倒:
  fall_detector (IMU pitch > 60° or height < 0.3m)
    → 切换到 HoST
    → standing_up_policy → joint commands → robot
    → 站稳后 (height > 0.65m, pitch < 10°, 持续 2s)
    → 切换回 locomotion_policy
```

**Fall Detector 状态机的工程实现**：

```python
class FallDetectorFSM:
    """人形跌倒检测和恢复状态机"""
    NORMAL = 0
    FALLING = 1
    STANDING_UP = 2
    STABILIZING = 3
    
    def __init__(self):
        self.state = self.NORMAL
        self.stable_counter = 0
        self.stable_threshold = 100  # 2 秒 @ 50 Hz
    
    def update(self, base_height, base_pitch, base_roll):
        if self.state == self.NORMAL:
            # 检测摔倒：高度过低 OR 倾斜过大
            if base_height < 0.3 or abs(base_pitch) > 1.05 or abs(base_roll) > 1.05:
                self.state = self.FALLING
                self.stable_counter = 0
                return "switch_to_host"
        
        elif self.state == self.FALLING:
            # 等待机器人停止运动后开始站起
            self.state = self.STANDING_UP
            return "start_standing_up"
        
        elif self.state == self.STANDING_UP:
            # 检测是否已经站起来
            if base_height > 0.65 and abs(base_pitch) < 0.17 and abs(base_roll) < 0.17:
                self.state = self.STABILIZING
                self.stable_counter = 0
            return "continue_host"
        
        elif self.state == self.STABILIZING:
            # 保持稳定一段时间再切回
            if base_height > 0.65 and abs(base_pitch) < 0.17:
                self.stable_counter += 1
                if self.stable_counter >= self.stable_threshold:
                    self.state = self.NORMAL
                    return "switch_to_locomotion"
            else:
                # 不稳定，回到站起状态
                self.state = self.STANDING_UP
                self.stable_counter = 0
            return "continue_host"
        
        return "continue_current"
```

**工程要点**：
- fall_detector 必须足够快（<50ms）——摔倒后如果还在执行 locomotion 策略，可能做出更危险的动作
- `STABILIZING` 状态是关键——如果过早切回 locomotion，G1 可能在未完全稳定时接收到速度命令导致二次摔倒
- 切换时的 obs adapter 需要处理 locomotion obs（含 command）到 HoST obs（含摔倒姿态信息）的转换
- 站稳后切回 locomotion 时，command 应该先设为零速（站立），再逐步恢复

### ⚠️ 常见陷阱

⚠️ **编程陷阱：HoST 基于 legged_gym 而非 Manager-Based**。HoST 代码仓库使用 IsaacGym + legged_gym 架构（单体类），与本书使用的 mjlab/Isaac Lab Manager-Based 架构不同。直接复制 HoST 代码到 mjlab 不可行——需要重新实现 multi-critic 和辅助力 curriculum。

💡 **概念误区：站起来是简单的逆过程**。摔倒是被动的（重力做功），站起来是主动的（关节力矩做功对抗重力）。两者的动力学完全不同——你不能"倒放摔倒轨迹"来站起来。

🧠 **思维陷阱：locomotion 策略足够鲁棒就不需要 HoST**。即使 locomotion 策略在仿真中从不摔倒，真机上的意外（被推、绊倒、滑倒）不可完全避免。没有 HoST 兜底，一次摔倒 = 人工介入 = 系统不可用。

### 练习

1. **[架构分析题]** Multi-critic 和 multi-head critic（一个网络多个输出头）有什么区别？列出各自的优缺点。HoST 为什么选择完全独立的 critic？
2. **[设计题]** 设计一个 fall_detector 的状态机。列出 (a) 正常→摔倒的触发条件，(b) 摔倒→站起的触发条件，(c) 站起→正常的触发条件。每个条件需要什么传感器数据？
3. **[跨章综合题，Ch06+Ch14]** HoST 的辅助力 curriculum 与 Ch06 的 curriculum learning 有什么共同点和不同点？如果你要在 velocity task 中加入类似的"辅助力"来帮助训练初期的 G1 站稳，应该怎么设计？

---

## 本章小结

| 知识点 | 核心要点 | 难度 |
|--------|---------|------|
| 四足 vs 人形 | 支撑面缩小一个数量级、质心更高、角动量管理是新挑战 | ⭐⭐ |
| G1 关节结构 | 29 DoF 分 5 组，per-joint action scale = 0.25 × effort/stiffness | ⭐⭐ |
| Variable posture | 运动状态自适应的 per-joint 约束强度，完全锁死和完全放松都不行 | ⭐⭐⭐ |
| Angular momentum penalty | 抑制全身角动量增长，权重 -0.02 是平衡点 | ⭐⭐ |
| 人形 reward 四层适配 | Style 层加入 variable_posture/angular_momentum，Safety 层加入 self_collision | ⭐⭐⭐ |
| Isaac Lab H1 vs mjlab G1 | H1 19-DoF / G1 29-DoF，内置配置缺少人形特有 reward | ⭐⭐ |
| sim-to-sim 验证 | HOVER 提供 Isaac Lab → MuJoCo 工具链，人形上更重要 | ⭐⭐ |
| HOVER | mask-conditioned distillation，一个策略支持多种控制模态 | ⭐⭐⭐ |
| HoST | multi-critic + 辅助力 curriculum + smoothness regularization | ⭐⭐⭐ |
| 安全兜底 | fall_detector → HoST → 站稳后切回 locomotion | ⭐⭐ |

### 本章与其他章节的关系

| 本章知识 | 前置来源（回顾） | 后续应用（预告） |
|---------|----------------|----------------|
| 四层 reward 框架的人形适配 | Ch06 Reward 设计 + Ch13 四足 reward | Ch15 Motion Imitation reward 设计 |
| Per-joint action scale | Ch05 Action 设计 + Ch13 统一 scale | Ch17 操作的 DiffIK action |
| sim-to-sim 验证 | Ch13 双框架对比 | Ch23 Sim2Real 验证管线 |
| HOVER extension 模式 | Ch04 Manager-Based 架构 | Ch22 自定义 task 的标准模式 |
| HoST 安全兜底 | Ch08 DR 的鲁棒性思想 | Ch23 真机安全保障 |
| Variable posture | Ch13 统一 pose reward | Ch15 Motion tracking 的风格约束 |
| Angular momentum | — | Ch20 全身控制的角动量管理 |

## 累积项目 B：本章新增模块

### 模块清单

| 模块 | 状态 | 说明 |
|------|------|------|
| G1 flat velocity (mjlab) | ✅ 完成 | per-joint scale + variable posture + angular momentum |
| G1 rough velocity (mjlab) | ✅ 完成 | 加 terrain scan + contact sensor |
| H1 flat velocity (Isaac Lab) | ✅ 完成 | 内置配置 + 用户自定义 reward |
| sim-to-sim 验证 | ✅ 完成 | mjlab ↔ Isaac Lab 交叉验证 |
| HOVER 精读 | ✅ 完成 | 理解 mask-conditioned distillation |
| HoST 精读 | ✅ 完成 | 理解 multi-critic 和安全兜底 |

### 实践里程碑（建议用时 3-4 天）

| 里程碑 | 预计用时 | 完成标准 | 前置 |
|--------|---------|---------|------|
| M1: G1 flat zero+smoke | 2h | zero agent 能站 >1s，smoke train 无报错 | Ch13 完成 |
| M2: G1 flat baseline | 4h | tracking error < 0.25 m/s，fall rate < 15%，5000 iter | M1 |
| M3: Reward ablation | 6h | 6 组 ablation × 3000 iter，对比表完成 | M2 |
| M4: Variable posture 调优 | 3h | 手臂不乱甩，angular_momentum < 5 Nm·s | M3 |
| M5: G1 rough 训练 | 4h | rough 地形 fall rate < 25%，terrain curriculum 工作 | M4 |
| M6: DR 三阶段引入 | 4h | 阶段 3 DR 下 fall rate < 30% | M5 |
| M7: Isaac Lab H1 对照 | 3h | H1 flat 训练完成，与 G1 对比 reward curves | M2 |
| M8: sim-to-sim 验证 | 3h | 跨引擎 fall rate < 20% 或找到过拟合原因 | M2+M7 |

**总计 ~29 GPU-hours**（RTX 4090）。如果你在 Ch13 中已经熟悉了 mjlab/Isaac Lab 的工作流，M1 可以在 30 分钟内完成（使用 14.1 的 30 分钟工程流程）。

### 快速读懂任意人形 Velocity Task 配置的 5 分钟流程

当你遇到一个新的人形 velocity task 配置（如从论文代码下载的第三方人形项目），以下流程让你在 5 分钟内理解它的核心结构并发现与标准配置的差异：

```
分钟 1：找 task registration + entity
  → 确认机器人型号（G1/H1/其他）和 DoF
  → 确认 default pose 来源

分钟 2：检查 action 配置
  → 是 per-joint scale 还是统一 scale？
  → 如果统一 scale → 可能有关节抽动问题
  → action scale 的计算公式是什么？

分钟 3：检查人形特有 reward
  → 有 angular_momentum penalty 吗？
  → 有 variable_posture（或 per-joint pose）吗？
  → 有 self_collision 吗？
  → 如果三个都缺 → 这是一个基础配置，需要增强

分钟 4：检查 termination
  → fell_over 阈值是多少？（人形应 <60°）
  → 有 base_height termination 吗？
  → 有 self_collision termination 吗？

分钟 5：检查 DR 配置
  → push force 范围是多少？（应比四足更保守）
  → friction 范围是多少？
  → 是否分阶段引入？
```

### 从本章到下一章

本章的人形 velocity tracking 建立了双足 RL 的基础工程能力。但 velocity tracking 只告诉机器人"走多快、往哪走"——它不能指定**怎么走**（步态风格、身体姿态、动作序列）。Ch15（Motion Imitation 实战）将在 G1 上实现动作跟踪——给定一个参考运动序列（来自 MoCap 或视频），策略学习忠实地复现这个动作。你会发现本章的 per-joint action scale、variable posture、angular momentum 设计都可以直接复用，但 reward 体系需要从 velocity tracking 切换到 body pose tracking。

**后续扩展路径**：

```
Ch14 人形 velocity (本章)
  → Ch15 Motion Imitation (动作跟踪, BeyondMimic)
     → Ch20 全身控制 (velocity + manipulation, HOVER/WoCoCo)
        → Ch23 Sim2Real (真机部署)
```

每一步都复用前一步的基础设施：per-joint action scale、reward 框架、DR 策略、sim-to-sim 验证方法。Ch14 是这条路径的地基。

**本章建立的核心能力检查**：

| 能力 | 验证方式 | 对应小节 |
|------|---------|---------|
| 读懂人形 velocity task 配置 | 拿到新人形 cfg 能在 5 分钟内列出与四足的差异 | 14.1-14.4 |
| 独立跑通 G1 flat + rough 训练 | 从零开始配置，四阶段验证全部通过 | 14.1-14.4 |
| 诊断人形特有失败 | 面对"手臂乱甩"能在 10 分钟内定位到 variable posture | 14.3 |
| 做 reward ablation | 6 组实验的对比表能在 1 天内完成 | 14.3 |
| 理解 HOVER 架构 | 能解释 mask-conditioned distillation 的训练和推理流程 | 14.6 |
| 设计安全兜底 | 能写出 fall_detector 状态机的伪代码 | 14.7 |
| 执行 sim-to-sim 验证 | 跨引擎评估并判断性能下降是否在可接受范围内 | 14.5 |

---

## 延伸阅读

### 学术论文

| 资料 | 难度 | 会议/期刊 | 说明 |
|------|------|----------|------|
| He et al., "HOVER: Versatile Neural Whole-Body Controller for Humanoid Robots," 2025 | ⭐⭐⭐ | ICRA 2025 | mask-conditioned 多模态统一控制器 |
| Gu et al., "Humanoid-Gym: RL for Humanoid Robot with Zero-Shot Sim2Real Transfer," 2024 | ⭐⭐ | RSS 2024 | sim-to-sim 验证方法论 + XBot-S/L 部署 |
| Huang et al., "Learning Humanoid Standing-up Control across Diverse Postures," 2025 | ⭐⭐⭐ | RSS 2025 | multi-critic + 辅助力 curriculum |
| Radosavovic et al., "Real-World Humanoid Locomotion with RL," 2024 | ⭐⭐ | *Science Robotics* | 真机人形 RL 部署的里程碑 |
| Luo et al., "SONIC: Supersizing Motion Tracking for Natural Humanoid WBC," 2025 | ⭐⭐⭐ | arXiv 2511.07820 | 100M frames / 42M params 的人形运动基础模型 |
| Cheng et al., "Expressive Humanoid Whole-Body Control by Matching Full-Body Motion," 2024 | ⭐⭐ | RSS 2024 | ExBody：上下半身解耦的表达性控制 |

### 工具和代码

| 资料 | 难度 | 说明 |
|------|------|------|
| NVlabs/HOVER | ⭐⭐⭐ | Isaac Lab extension，teacher-student 训练管线 |
| InternRobotics/HoST | ⭐⭐ | IsaacGym + legged_gym，multi-critic 跌倒恢复 |
| roboterax/humanoid-gym | ⭐⭐ | Isaac Gym，sim-to-sim 验证 + symmetric mirror |
| unitreerobotics/unitree_rl_mjlab | ⭐⭐ | mjlab，G1 velocity + tracking 完整管线 |
| NVlabs/GR00T-WholeBodyControl | ⭐⭐⭐ | SONIC + GR00T 统一 WBC 平台（2026 最新） |

### 阅读路线建议

- **最小路线**（只做 G1 velocity）：14.1→14.2→14.3 + 跑通训练
- **标准路线**（velocity + sim-to-sim）：上述 + 14.4→14.5
- **进阶路线**（准备做 whole-body control）：上述 + 14.6 HOVER 精读
- **研究路线**（准备发论文）：上述 + 14.7 HoST + SONIC 论文

---

## 🔧 故障排查手册

| # | 症状 | 可能原因 | 排查步骤 | 相关小节 |
|---|------|---------|---------|---------|
| 1 | G1 zero agent 立即摔倒 | default pose 不合理或 PD gains 太低 | 1. 检查 keyframe qpos 2. 用 `--no-terminations` 观察 3. 增大 kp | 14.2 |
| 2 | 速度涨但 G1 前倾摔倒 | upright 约束不足 | 1. 画 velocity+upright 时间图 2. 增大 upright 权重 3. 收紧 waist_pitch std | 14.3 |
| 3 | 手臂乱甩但 reward 还在涨 | variable_posture std 太松或未配置 | 1. 检查 walking/running std 2. 只调上肢 std 3. 观察 angular_momentum | 14.3 |
| 4 | 侧向命令导致 roll 发散 | 侧向命令范围过大 | 1. 检查 lin_vel_y range 2. 收窄命令范围 3. 不要同时改 yaw | 14.1 |
| 5 | 踝关节频繁抽动 | action scale 统一设置或太大 | 1. 检查 per-joint action scale 2. 计算 effort/stiffness 3. 用公式重算 | 14.2 |
| 6 | 自碰撞频繁 | self_collision body_pairs 缺失 | 1. 加入 self_collision reward 2. 检查 body_pairs 3. play 视频确认碰撞位置 | 14.3 |
| 7 | 跨引擎验证全部摔倒 | obs 顺序不一致 | 1. 两端打印 obs term name 和 shape 2. 逐一对齐 3. 检查归一化参数 | 14.5 |
| 8 | Isaac Lab H1 训练效果差 | 缺少 angular_momentum 和 variable_posture | 1. 添加自定义 reward term 2. 参考 mjlab G1 的权重 3. 逐步调整 | 14.4 |
| 9 | DR 后 G1 全部摔倒 | push/friction/mass 同时开太强 | 1. 分阶段引入 DR 2. push force = Go1 的 30-50% 3. 先只开 friction | 14.1 |
| 10 | 23-DoF / 29-DoF 混用报错 | action dim 不匹配 | 1. 确认 MJCF 中的 actuator 数量 2. 检查 env cfg 的 action dim 3. 统一版本 | 14.2 |

## Debug Checklist

**Entity & 关节**

- [ ] G1 版本确认（23-DoF vs 29-DoF），action dim 匹配
- [ ] default pose 来自 MJCF keyframe，zero agent 能站 >1s
- [ ] per-joint action scale 已用 0.25 × effort/stiffness 计算
- [ ] entity key 是 `"robot"`，base body 是 `pelvis`（G1）或 `torso`（H1）

**Reward**

- [ ] 四层 reward 齐全：Tracking + Regularization + Style + Safety
- [ ] 包含 angular_momentum penalty（权重 ~-0.02）
- [ ] 包含 variable_posture reward（per-joint, per-state std）
- [ ] 包含 self_collision penalty（只监测异常碰撞对）
- [ ] foot_slip 权重已加大（相对 Go1）

**Observation**

- [ ] actor obs 包含 command（29-DoF: 行走用 twist）
- [ ] critic obs 包含 angular_momentum（privileged）
- [ ] obs group 名字正确（mjlab: actor/critic, Isaac Lab: policy/critic）

**训练**

- [ ] 分阶段验证：zero → random → small train → large train
- [ ] zero agent 用 `--no-terminations`（人形可能很快摔倒，这不代表配置错误）
- [ ] DR 分阶段引入（不要一次全开）
- [ ] push force 初始值 = Go1 的 30-50%
- [ ] 训练日志包含 upright 和 angular_momentum 的分项数据

---

## 附录 A：G1 关节完整参考表

以下表格汇总 G1 29-DoF 版本的所有 actuated joints 的关键参数。这是配置 action scale、default pose 和 variable posture reward 时的查阅工具。

| # | 关节名 | 组 | effort (Nm) | stiffness | damping | range (rad) | action_scale (rad) |
|---|--------|-----|------------|-----------|---------|-------------|-------------------|
| 0 | `left_hip_yaw` | 腿-髋 | 88 | 150 | 5 | [-0.43, 0.43] | 0.147 |
| 1 | `left_hip_roll` | 腿-髋 | 88 | 150 | 5 | [-0.43, 0.43] | 0.147 |
| 2 | `left_hip_pitch` | 腿-髋 | 88 | 150 | 5 | [-1.57, 1.57] | 0.147 |
| 3 | `left_knee` | 腿-膝 | 139 | 200 | 5 | [-0.26, 2.05] | 0.174 |
| 4 | `left_ankle_pitch` | 腿-踝 | 50 | 40 | 2 | [-0.87, 0.52] | 0.313 |
| 5 | `left_ankle_roll` | 腿-踝 | 50 | 40 | 2 | [-0.26, 0.26] | 0.313 |
| 6 | `right_hip_yaw` | 腿-髋 | 88 | 150 | 5 | [-0.43, 0.43] | 0.147 |
| 7 | `right_hip_roll` | 腿-髋 | 88 | 150 | 5 | [-0.43, 0.43] | 0.147 |
| 8 | `right_hip_pitch` | 腿-髋 | 88 | 150 | 5 | [-1.57, 1.57] | 0.147 |
| 9 | `right_knee` | 腿-膝 | 139 | 200 | 5 | [-0.26, 2.05] | 0.174 |
| 10 | `right_ankle_pitch` | 腿-踝 | 50 | 40 | 2 | [-0.87, 0.52] | 0.313 |
| 11 | `right_ankle_roll` | 腿-踝 | 50 | 40 | 2 | [-0.26, 0.26] | 0.313 |
| 12 | `waist_yaw` | 腰 | 88 | 200 | 5 | [-0.79, 0.79] | 0.110 |
| 13 | `waist_roll` | 腰 | 88 | 200 | 5 | [-0.52, 0.52] | 0.110 |
| 14 | `waist_pitch` | 腰 | 88 | 200 | 5 | [-0.52, 0.52] | 0.110 |
| 15 | `left_shoulder_pitch` | 手臂 | 25 | 40 | 2 | [-2.87, 2.87] | 0.156 |
| 16 | `left_shoulder_roll` | 手臂 | 25 | 40 | 2 | [-1.34, 2.53] | 0.156 |
| 17 | `left_shoulder_yaw` | 手臂 | 25 | 40 | 2 | [-1.57, 2.09] | 0.156 |
| 18 | `left_elbow` | 手臂 | 25 | 40 | 2 | [-1.92, 0.52] | 0.156 |
| 19 | `right_shoulder_pitch` | 手臂 | 25 | 40 | 2 | [-2.87, 2.87] | 0.156 |
| 20 | `right_shoulder_roll` | 手臂 | 25 | 40 | 2 | [-2.53, 1.34] | 0.156 |
| 21 | `right_shoulder_yaw` | 手臂 | 25 | 40 | 2 | [-2.09, 1.57] | 0.156 |
| 22 | `right_elbow` | 手臂 | 25 | 40 | 2 | [-1.92, 0.52] | 0.156 |
| 23 | `left_wrist_roll` | 手腕 | 5 | 40 | 1 | [-0.52, 0.52] | 0.031 |
| 24 | `left_wrist_pitch` | 手腕 | 5 | 40 | 1 | [-0.87, 0.87] | 0.031 |
| 25 | `left_wrist_yaw` | 手腕 | 5 | 40 | 1 | [-0.87, 0.87] | 0.031 |
| 26 | `right_wrist_roll` | 手腕 | 5 | 40 | 1 | [-0.52, 0.52] | 0.031 |
| 27 | `right_wrist_pitch` | 手腕 | 5 | 40 | 1 | [-0.87, 0.87] | 0.031 |
| 28 | `right_wrist_yaw` | 手腕 | 5 | 40 | 1 | [-0.87, 0.87] | 0.031 |

**注意**：以上参数值为典型配置，具体数值以实际 MJCF/USD 文件为准。不同版本的 G1 MJCF 可能有微调。使用 14.2 中的关节信息提取脚本获取你使用的模型的精确参数。

**使用方法**：
- 配置 action scale → 直接使用最后一列
- 配置 DR（motor strength）→ effort 列确定随机化范围
- 配置 termination（joint limit）→ range 列确定合理阈值
- 配置 variable posture → 根据组确定 std 级别（腿松、腰紧、手臂中等、手腕紧）

---

## 附录 B：Variable Posture std 完整配置表

以下表格是 mjlab G1 velocity task 的典型 variable posture std 配置。数值越小，约束越严格（关节越不允许偏离 default pose）。

| 关节 | 组 | standing | walking | running | 设计理由 |
|------|------|----------|---------|---------|---------|
| hip_yaw | 腿 | 0.05 | 0.10 | 0.10 | yaw 偏离导致脚朝向变化 |
| hip_roll | 腿 | 0.05 | 0.10 | 0.10 | roll 偏离直接影响侧向稳定 |
| hip_pitch | 腿 | 0.10 | 0.25 | 0.40 | pitch 是步幅的主要来源 |
| knee | 腿 | 0.10 | 0.15 | 0.20 | 跑步需要更大膝弯 |
| ankle_pitch | 腿 | 0.05 | 0.10 | 0.15 | 推离地面的主要关节 |
| ankle_roll | 腿 | **0.02** | **0.02** | **0.05** | **始终严格约束**——脚掌 roll 过大直接失衡 |
| waist_yaw | 腰 | 0.05 | 0.10 | 0.10 | 允许一定转体 |
| waist_roll | 腰 | **0.02** | **0.02** | **0.05** | **始终严格约束**——腰部侧倾极危险 |
| waist_pitch | 腰 | 0.05 | 0.10 | 0.10 | 允许一定前后倾 |
| shoulder_pitch | 手臂 | 0.10 | 0.15 | **0.50** | **跑步时大幅放宽**——需要摆臂补偿角动量 |
| shoulder_roll | 手臂 | 0.10 | 0.15 | 0.15 | 侧向摆臂始终不需要太大 |
| shoulder_yaw | 手臂 | 0.05 | 0.10 | 0.10 | yaw 旋转几乎不用于行走 |
| elbow | 手臂 | 0.10 | 0.15 | **0.35** | 跑步时弯肘摆臂更自然 |
| wrist_* | 手腕 | **0.02** | **0.05** | **0.05** | 手腕对行走贡献极小，严格约束避免抽动 |

**调参经验法则**：
- `0.02`：几乎锁死——只允许极小偏离
- `0.05`：很严格——允许约 3° 偏离
- `0.10`：中等——允许约 6° 偏离
- `0.15-0.20`：放松——允许约 10-12° 偏离
- `0.35-0.50`：很放松——允许约 20-30° 偏离

**反事实推理：如果所有关节在所有状态下都用 std=0.15 会怎样？** 站立时手臂会微微摆动（不够安静），跑步时摆臂幅度被限制（不够自然），ankle_roll 偏离导致脚掌不稳（危险）。统一 std 看起来"简单公平"，但忽略了不同关节和不同运动状态的物理约束差异。

---

## 附录 C：人形 DR 策略指南

人形比四足对 DR 更敏感。以下是推荐的 DR 引入策略——分 3 个阶段逐步增强。

### 阶段 1：基础 DR（训练初期开启）

| DR 项 | 模式 | 范围 | 说明 |
|--------|------|------|------|
| friction | startup | [0.7, 1.3] | 比 Go1 的 [0.5, 1.5] 更保守 |
| restitution | startup | [0.0, 0.2] | 弹性系数 |
| motor_strength | startup | [0.85, 1.15] | 比 Go1 的 [0.8, 1.2] 更保守 |
| obs_noise | step | 参见 obs table | 传感器噪声 |

### 阶段 2：中级 DR（baseline 策略能走 >500 步后开启）

| DR 项 | 模式 | 范围 | 说明 |
|--------|------|------|------|
| added_mass | startup | [-1.0, 2.0] | Go1 用 [-1.0, 3.0]，G1 更保守 |
| CoM_offset | startup | [-0.05, 0.05] m | 质心偏移 |
| random_push | interval | [-0.5, 0.5] m/s | Go1 用 [-1.0, 1.0]，G1 **必须更保守** |
| joint_damping | startup | [0.8, 1.2] × default | — |

### 阶段 3：高级 DR（策略在中级 DR 下能走 >300 步后开启）

| DR 项 | 模式 | 范围 | 说明 |
|--------|------|------|------|
| ground_height_noise | startup | [-0.02, 0.02] m | 模拟不平地面 |
| IMU_bias | startup | gyro ±0.02, accel ±0.1 | 传感器偏置 |
| action_delay | step | [0, 2] 步 | 通信延迟 |
| expanded friction | startup | [0.5, 2.0] | 扩大到 Go1 级别 |
| expanded push | interval | [-1.0, 1.0] m/s | 扩大到 Go1 级别 |

**工程建议**：每个阶段训练约 2000-3000 iterations 后评估策略质量。如果 fall rate 显著增加（>20%），退回上一阶段增加训练量。不要跳级——阶段 3 的 DR 在没有阶段 1-2 基础的策略上几乎肯定失败。

**反事实推理：如果跳过阶段 1-2 直接开启阶段 3 会怎样？** 初始策略在极端 DR 下根本无法站稳——episode 极短、reward 信号极稀疏。PPO 无法从中学到有意义的行为。即使偶尔出现好的 rollout，也被大量的失败 rollout 淹没。策略收敛到"蹲在地上不动"——这是一个局部最优，避免了 push 导致的摔倒惩罚但完全不走路。

### DR 参数的四足→人形迁移经验法则

| DR 参数 | 四足 (Go1) 典型值 | 人形 (G1) 建议值 | 缩小比例 | 原因 |
|---------|----------------|-----------------|---------|------|
| push velocity | ±1.5 m/s | ±0.5 m/s | **3×** | 高质心 + 窄支撑面 |
| friction range | [0.5, 2.0] | [0.7, 1.5] | ~1.5× | 单脚支撑时摩擦更关键 |
| mass range | [-1, 3] kg | [-1, 2] kg | ~1.3× | 质心高度变化影响更大 |
| motor strength | [0.8, 1.2] | [0.85, 1.15] | ~1.3× | 力矩余量更小 |
| IMU noise | 标准 | 略大 | 1× | 人形更依赖 IMU |

---

## 附录 D：人形训练实验记录模板

```yaml
experiment:
  name: g1_flat_velocity_v3_seed42
  date: 2026-05-21
  framework: mjlab v0.2.1
  robot: Unitree G1 (29-DoF)
  terrain: flat
  commit: abc1234
  gpu: RTX 4090
  
config:
  num_envs: 4096
  max_iterations: 5000
  seed: 42
  
  # 与 baseline 的配置差异
  diff_from_baseline:
    reward/angular_momentum_weight: -0.02 → -0.03
    reward/variable_posture/walking/shoulder_pitch_std: 0.15 → 0.20
  
  # DR 阶段
  dr_stage: 2  # 基础 + 中级
  
results:
  final_tracking_error_xy: 0.18 m/s
  final_tracking_error_yaw: 0.22 rad/s
  fall_rate: 7.2%
  avg_episode_length: 620 steps
  angular_momentum_mean: 3.2 Nm·s
  steps_per_second: 9200
  wall_time: 68 min
  
observations:
  gait: "稳定 trot-like 双足步态"
  arms: "自然摆臂，无明显乱甩"
  weaknesses:
    - "侧向命令 >0.3 m/s 时偶尔 roll 不稳"
    - "后退时步频降低，有拖步现象"
  
next_steps:
  - "收窄 lin_vel_y range 到 [-0.2, 0.2]"
  - "增大 ankle_roll variable posture 约束"
  - "准备进入 DR 阶段 3"
  
video: logs/rsl_rl/g1_velocity/v3_seed42/videos/iter_5000.mp4
```

---

## 附录 E：人形 Velocity Task Reward 完整参考表

| Term | 四层分类 | 典型权重 | Kernel | Go1 对照权重 | 人形特有？ |
|------|---------|--------|--------|------------|----------|
| `track_lin_vel_xy` | Tracking | +2.0 | exp | +2.0 | 否 |
| `track_ang_vel_z` | Tracking | +2.0 | exp | +2.0 | 否 |
| `upright` | Style | +1.0 | L2 | +1.0 | 否（但人形更关键） |
| `base_height` | Style | +0.5 | L2 | — | **是** |
| `variable_posture` | Style | +1.0 | weighted L2 | — (pose +1.0) | **是** |
| `feet_air_time` | Style | +0.5 | bonus | +0.5 | 否（但参数不同） |
| `angular_momentum` | Regularization | **-0.02** | L2 | — | **是** |
| `action_rate_l2` | Regularization | -0.1 | L2 | -0.1 | 否 |
| `dof_acceleration` | Regularization | -0.0025 | L2 | -0.0025 | 否 |
| `joint_torques` | Regularization | -0.0001 | L2 | -0.0001 | 否 |
| `dof_pos_limits` | Regularization | -1.0 | soft | -1.0 | 否 |
| `linear_velocity_z` | Regularization | -2.0 | L2 | -2.0 | 否 |
| `angular_velocity_xy` | Regularization | -0.05 | L2 | -0.05 | 否 |
| `foot_clearance` | Contact | -2.0 | conditional | -2.0 | 否（阈值不同） |
| `foot_slip` | Contact | **-0.2** | L2 | -0.1 | 否（权重加大） |
| `self_collision` | Safety | **-1.0** | binary | — | **是** |
| `undesired_contacts` | Safety | -1.0 | binary | -1.0 | 否（body 不同） |

**人形新增 4 项**（base_height、variable_posture、angular_momentum、self_collision）都是因为人形的物理特性（高质心、多上肢、窄支撑面）而必需的。

---

## 附录 F：两条技术路线——Velocity Tracking vs Motion Imitation

本章专注于 velocity tracking 路线。但 G1 还有另一条路线——motion imitation（Ch15 详细讨论）。两者的选择取决于任务需求。

### 路线对比

| 特征 | Velocity Tracking (本章) | Motion Imitation (Ch15) |
|------|------------------------|----------------------|
| 输入目标 | twist 速度命令 $(v_x, v_y, \omega_z)$ | 参考动作序列 + body list |
| 主要 reward | 速度、upright、pose、foot、angmom | anchor、body pose/ori、velocity |
| 适合问题 | 通用走路跑步，灵活性高 | 复现特定动作序列 |
| 主要风险 | 速度好但姿态不自然 | 参考数据质量决定上限 |
| 框架 | mjlab velocity / Isaac Lab velocity | mjlab tracking / ProtoMotions |
| 训练时间 | 1-2 h (flat) | 2-6 h（取决于动作复杂度） |
| 数据需求 | 无（只需 command） | 参考运动文件（MoCap/retarget） |

### 源码映射

| 概念 | Velocity 路径 | Tracking 路径 |
|------|-------------|--------------|
| task id | `Mjlab-Velocity-Flat/Rough-Unitree-G1` | `Mjlab-Tracking-Flat-Unitree-G1` |
| command | `UniformVelocityCommandCfg` | `MotionCommandCfg` |
| base cfg | `velocity_env_cfg.py` | `tracking_env_cfg.py` |
| G1 cfg | `velocity/config/g1/env_cfgs.py` | `tracking/config/g1/env_cfgs.py` |
| rewards | velocity, upright, pose, foot, angmom | anchor, body pose/ori, velocity |
| action space | 29-dim joint position | 29-dim joint position（相同） |

### 何时选择哪条路线

- "让 G1 按指令走到目的地" → velocity tracking（最大灵活性）
- "让 G1 复现特定舞蹈/搬运动作" → motion imitation（最精确控制）
- "让 G1 像人一样走但方向可控" → 混合路线（velocity + style prior from imitation）
- "让 G1 做 whole-body control（导航 + 操作）" → HOVER（14.6 的统一框架）

### ⚠️ 常见陷阱

⚠️ **编程陷阱：混用 velocity 和 tracking 配置**。velocity 的 command 是 `UniformVelocityCommandCfg`（3 维 twist），tracking 的 command 是 `MotionCommandCfg`（参考运动文件）。混用导致 command 维度不匹配，actor observation shape 错误。

💡 **概念误区：tracking 一定比 velocity 更自然**。tracking 的自然度取决于参考动作质量。retarget 质量差时（比如 SMPL → G1 的骨骼比例不匹配），tracking 忠实复现的是不自然的动作。velocity 配合好的 variable posture 可能反而更自然。

---

## 附录 G：Unitree G1 物理规格速查

| 参数 | 值 | 来源 |
|------|-----|------|
| 身高 | 1.32 m | Unitree 官方 |
| 体重（含电池） | ~35 kg | Unitree 官方 |
| DoF 版本 | 23-DoF / 29-DoF | 29-DoF 含 3-DoF 手腕 |
| pelvis 高度（站立） | ~0.76 m | MJCF keyframe |
| 脚掌尺寸（约） | 0.25m × 0.10m | MJCF collision geom |
| 最大步速（RL） | ~1.5-2.0 m/s | 训练结果，非硬件限制 |
| 电池续航 | ~2 小时（行走） | Unitree 官方 |
| 通信接口 | DDS over Ethernet | Unitree SDK2 |
| 推理硬件 | Jetson Orin NX | 板载 |
| 控制频率（部署） | 50 Hz（策略） + 500 Hz（PD） | 两级频率 |

**H1 对照**：

| 参数 | G1 | H1 | 差异影响 |
|------|-----|-----|---------|
| 身高 | 1.32 m | 1.80 m | H1 质心更高，更不稳定 |
| 体重 | ~35 kg | ~47 kg | H1 惯性更大，响应更慢 |
| DoF | 23/29 | 19 | H1 没有手腕和 ankle_roll |
| 腰部 | 3-DoF | 1-DoF (yaw only) | G1 上下半身耦合更强 |
| 力矩 | 小（max 139 Nm） | 大（max 300 Nm） | H1 更适合高速运动 |

---

## 附录 H：人形 RL 的三条技术路线概览

本章专注于 velocity tracking，但人形 RL 有三条平行的技术路线，理解它们的关系有助于你规划后续学习路径：

| 路线 | 代表工作 | 输入 | 输出 | 本书章节 |
|------|---------|------|------|---------|
| **Velocity Tracking** | mjlab velocity, humanoid-gym | twist command $(v_x, v_y, \omega_z)$ | joint position target | **Ch14**（本章） |
| **Motion Imitation** | BeyondMimic, PHC, KungfuBot | 参考运动序列 (MoCap/retarget) | joint position target | Ch15 |
| **Whole-Body Control** | HOVER, ExBody, SONIC | 多模态目标 (velocity + body pose + ...) | joint position target | Ch20 |

三条路线的**输出完全相同**（关节位置目标），区别在于**输入**和**reward 设计**。这意味着本章建立的 action space（per-joint scale）、termination、DR 策略可以在三条路线中复用——你切换路线时只需要修改 observation 和 reward，不需要重建基础设施。

**技术路线的演进关系**：

```
Velocity Tracking (Ch14)
  ↓ 加入参考运动
Motion Imitation (Ch15)
  ↓ 加入多模态 mask
Whole-Body Control (Ch20, HOVER)
  ↓ 加入语言/视觉输入
VLA (Ch22, GR00T/LeVERB)
```

每一步都在前一步的基础上增加新的输入模态，但底层的关节控制、物理仿真和训练基础设施保持不变。这就是为什么本章的工程基础如此重要——它是后续所有路线的共同地基。

---

> **结语**：从四足到人形，工程方法论不变（链路阅读法、四层 reward、分阶段验证、双框架对比），但每一个具体的配置参数都需要重新思考。本章建立的人形特有知识——per-joint action scale、variable posture、angular momentum penalty、HOVER 的 mask-conditioned distillation、HoST 的 multi-critic 安全兜底——将在后续的 motion imitation（Ch15）、whole-body control（Ch20）和 sim-to-real（Ch23）中反复使用。如果你能在 G1 上跑通 velocity tracking 并理解每个 reward term 的物理意义，你就掌握了人形 RL 的核心工程能力。
