# 第88章：Deep Whole-Body Control 精读——端到端 RL 18-DOF 统一策略

| 元信息 | 值 |
|---|---|
| 难度 | ⭐⭐⭐⭐（PPO、统一动作空间、在线适应、优势函数分解、Sim2Real） |
| 预计时间 | 1 周，25-35 小时 |
| 前置依赖 | 复合/160_四足臂动力学概览、足式/190_腿足RL训练栈、足式/90_WBC分层优化与TSID |
| 项目定位 | 四足+臂端到端强化学习的经典基线 |
| 代表问题 | 一个策略能否同时学会行走稳定、基座补偿和末端跟踪 |

---

## 88.0 前置自测

开始本章前，请先回答下面问题。

| # | 问题 | 前置 |
|---|---|---|
| 1 | PPO clipped objective 限制的是什么？为什么腿足 RL 常选 PPO？ | 足式/190_腿足RL训练栈 |
| 2 | 观测历史为什么能缓解 POMDP？它能隐式推断哪些物理量？ | 足式/190_腿足RL训练栈 |
| 3 | 四足臂统一动力学中，臂运动为什么会影响基座角动量？ | 复合/160_四足臂动力学概览 |
| 4 | 传统 MPC+WBC 的接口是什么？RL 策略替代了哪一部分？ | 足式/90_WBC分层优化与TSID |
| 5 | Domain Randomization 能解决哪些 sim-to-real 差距，不能解决哪些差距？ | 足式/190_腿足RL训练栈 |

**自测建议**：

| 正确题数 | 建议 |
|---|---|
| 5 | 直接阅读本章 |
| 3-4 | 边读边查 PPO 和观测设计 |
| 0-2 | 先补腿足 RL 训练栈 |

---

## 88.1 本章目标

本章精读 Deep Whole-Body Control 的思想，而不是只复述仓库文件。

读完后你应该能做到：

1. 解释为什么 Deep-WBC 选择一个统一策略控制腿和臂。
2. 写出四足臂强化学习的 MDP：状态、观测、动作、奖励和终止条件。
3. 解释 ROA 与 RMA 的区别，以及在线适应为什么能帮助迁移。
4. 推导 Advantage Mixing 的直觉：为什么腿任务和臂任务的优势估计可以分解。
5. 在 IsaacLab 中复现一个最小版四足臂全身策略。
6. 识别端到端 RL 在安全约束、视觉缺失和真机部署上的局限。

---

## 88.2 问题设定：一个策略同时管腿和臂 ⭐⭐

这一节回答最基本的问题：Deep-WBC 到底在学什么。

回顾复合/160：model-based 四足臂控制通常分成 MPC 和 WBC。

MPC 预测质心、接触力、末端目标。

WBC 高频求解关节力矩。

Deep-WBC 的核心反向选择是：不显式写 MPC/WBC，而是训练一个神经网络策略，从观测直接输出全身关节目标。

### 88.2.1 平台和任务

经典设置是 Unitree A1 加 WidowX 250 机械臂。

系统有四条腿和一个轻量机械臂。

动作空间覆盖腿和臂。

| 子系统 | 自由度 | 作用 |
|---|---:|---|
| 四条腿 | 12 | 支撑、移动、姿态稳定 |
| 机械臂 | 6 | 末端位置跟踪和操作 |
| 夹爪 | 1 | 抓取开合，部分任务中可单独处理 |
| 浮动基座 | 6 | 不直接驱动，由腿和臂共同影响 |

### 88.2.2 为什么统一策略有吸引力

传统分层控制需要人工决定：

| 决策 | 传统做法 |
|---|---|
| 末端任务和基座稳定谁优先 | 手调 WBC 权重或层级 |
| 臂摆动引起的基座扰动如何补偿 | 动力学模型 + 反馈控制 |
| 摩擦、质量、延迟变化如何适应 | 域随机化或鲁棒控制 |
| 不同任务阶段怎么切换 | 状态机或 mode schedule |

统一策略希望通过数据学习这些权衡。

它像一个经验丰富的操作员。

操作员看到身体姿态、关节状态和目标，就同时调整腿和臂。

但这个类比有边界。

神经网络不是显式求解约束。

它学到的是训练分布内的可行行为。

> **本质洞察**：Deep-WBC 不是把 WBC 的 QP 换成神经网络求解器。
> 它是把"任务权衡、扰动补偿和隐式步态选择"整体学习成一个反馈策略。
> 这种方式能吸收复杂非线性，但失去了显式可行性保证。

### 88.2.3 统一策略不是简单扩大动作维度

如果只是把腿策略的动作维度从 12 扩到 18，通常不会成功。

原因有三点。

| 问题 | 解释 |
|---|---|
| 多时间尺度 | 腿部接触在几十毫秒内变化，臂末端目标相对慢 |
| 任务耦合 | 臂跟踪会扰动基座，基座扰动会改变末端误差 |
| 奖励冲突 | 末端精度、能耗、稳定性和速度追踪互相牵制 |

所以 Deep-WBC 的关键不只是网络结构。

关键是观测、奖励、适应和优势估计的整体设计。

### 88.2.4 与 model-based 的对比

| 维度 | MPC+WBC | Deep-WBC |
|---|---|---|
| 物理约束 | 显式约束 | 通过奖励和训练分布隐式学习 |
| 调参方式 | 权重、层级、约束 | 奖励、随机化、网络、课程 |
| 实时性 | 求解 QP/MPC | 前向推理很快 |
| 解释性 | 强 | 中到弱 |
| 泛化性 | 模型泛化强 | 取决于训练覆盖 |
| 接触丰富 | 显式建模困难 | 可隐式学习 |
| 安全边界 | 可写硬约束 | 需要外部安全过滤 |

### ⚠️ 常见陷阱

> ⚠️ **概念误区：端到端策略不需要动力学理解**
>
> 训练时仍要理解质心、角动量、接触、延迟和执行器限制。
>
> 奖励和随机化本质上是在把动力学知识编码进数据生成过程。

> ⚠️ **工程陷阱：直接从纯四足 RL 配置加 6 个臂关节**
>
> 这样会得到不稳定训练。
>
> 必须重新设计目标观测、末端奖励、臂正则、基座稳定奖励和动作缩放。

> 🧠 **思维陷阱：RL 成功就说明 model-based 没用**
>
> RL 策略仍依赖仿真、奖励和安全层。
>
> model-based 方法常作为基线、教师、残差框架或安全过滤器继续存在。

### 练习 88.2

| # | 练习 | 难度 |
|---|---|---|
| 1 | 画出 MPC+WBC 与 Deep-WBC 的数据流图，标出二者各自的安全边界。 | ⭐ |
| 2 | 设臂末端目标突然向前移动 20 cm，分析统一策略需要同时调整哪些身体量。 | ⭐⭐ |
| 3 | 写出一个 model-based 与 Deep-WBC 混合方案：哪些量由模型给，哪些量由策略补偿。 | ⭐⭐⭐ |

---

## 88.3 MDP 建模：状态、观测、动作和终止 ⭐⭐⭐

这一节把四足臂任务写成强化学习问题。

不要直接把所有仿真状态喂给策略。

部署时拿不到的量只能用于 Critic、教师或奖励计算。

### 88.3.1 状态与观测的区别

完整状态可能包含：

$$
s_t=
(q,v,T_{ee},T_{target},\mu,m_{payload},contact,terrain,delay)
$$

真实可用观测则更少：

$$
o_t=
(q_j,\dot{q}_j,R_b,\omega_b,g_b,T_{target}^{body},a_{t-1},history)
$$

其中 $g_b$ 是重力方向在机体系中的投影。

| 类别 | 可部署给策略 | 可用于训练辅助 |
|---|---|---|
| 关节位置速度 | 是 | 是 |
| IMU 姿态和角速度 | 是 | 是 |
| 目标末端相对位姿 | 任务给定时是 | 是 |
| 地面摩擦 | 否 | 是 |
| 外部扰动力真值 | 否 | 是 |
| 接触力真值 | 取决于硬件 | 是 |
| payload 质量 | 通常否 | 是 |

### 88.3.2 观测设计

一个典型观测向量可包含：

| 项 | 维度示例 | 作用 |
|---|---:|---|
| base angular velocity | 3 | 姿态阻尼 |
| projected gravity | 3 | roll/pitch 感知 |
| command velocity | 3 | 行走目标 |
| joint position error | 18 | 姿态反馈 |
| joint velocity | 18 | 阻尼和接触推断 |
| previous action | 18 | 动作平滑和延迟补偿 |
| EE target in body frame | 3 或 6 | 操作目标 |
| observation history | $H\times d$ | 隐式估计摩擦、负载、延迟 |

观测历史的作用类似短期记忆。

单帧观测不能直接告诉策略地面摩擦或负载变化。

但如果同样动作产生了不同身体响应，历史序列就包含辨识信息。

如果不区分"部署可见观测"和"训练特权状态"，策略会在仿真中学会依赖摩擦系数、真实外力或 payload 质量这类真机拿不到的变量。结果不是泛化更强，而是出现典型的 privileged leakage：训练曲线很好，部署时一旦这些隐藏变量缺失，动作就会变得迟钝或抖动。

> **本质洞察**：四足臂 RL 的观测设计不是把所有传感器拼成一个大向量，而是在"可部署信息"和"可辅助训练信息"之间划清边界。Actor 只能吃真机能测到的量；Critic、教师或奖励可以使用特权量。这个边界决定了策略是在学习可部署反馈，还是在记忆仿真器答案。

### 88.3.3 网络架构细节：MLP 设计选择 ⭐⭐⭐

Deep-WBC 的策略网络通常是一个标准 MLP。但层数、宽度、激活函数和归一化的选择直接影响训练稳定性和部署性能。

**Actor 网络**

典型结构为三层 MLP：

| 层 | 输入维度 | 输出维度 | 激活函数 |
|---|---:|---:|---|
| Linear 1 | obs_dim (约 50-80) | 256 或 512 | ELU |
| Linear 2 | 256 或 512 | 256 或 128 | ELU |
| Linear 3 | 256 或 128 | action_dim (18) | 无（线性输出） |

输出是高斯分布的均值 $\mu(o)$，标准差 $\sigma$ 通常作为独立可学习参数（不依赖观测），初始化为 0.5-1.0。

**为什么用 ELU 而不是 ReLU？** ReLU 在负半轴梯度为零，高维动作空间中容易出现"死神经元"——某些单元永远不被激活。ELU 在负半轴提供非零梯度 $\alpha(e^x - 1)$，使网络在探索阶段更容易从次优策略中恢复。Leaky ReLU 也可以，但 ELU 的光滑性对 PPO 的策略梯度更友好。

**Critic 网络**

Critic 结构与 Actor 类似，但：
- 输入可以包含特权信息（如摩擦系数、负载质量）。
- 输出是标量 $V(s)$。
- 层宽度通常与 Actor 相同或略大。

**归一化选择**

| 归一化方式 | 适用场景 | 在四足臂中的使用 |
|---|---|---|
| 输入归一化（running mean/std） | 观测量级差异大时 | 几乎必须，关节位置和角速度量级差 10-100 倍 |
| Layer Normalization | 训练不稳定时 | 可选，对四足臂影响中等 |
| Batch Normalization | 固定 batch 时 | 不推荐，并行环境 batch 统计不稳定 |
| 无归一化 | 观测已手动缩放时 | 如果 obs 已乘缩放系数可行 |

观测缩放的经验值：角速度乘 0.25、关节速度乘 0.05、关节位置不缩放或减去默认值。这些系数的目标是让各分量的典型值落在 $[-1, 1]$ 附近。

> **反事实推理**：如果不做观测缩放会怎样？
> 假设角速度范围是 $[-10, 10]$ rad/s，关节位置范围是 $[-0.5, 0.5]$ rad。
> 网络前几层的梯度会被角速度主导，关节位置信号被淹没。
> 策略会学到"只看角速度调整动作"，末端跟踪精度下降。

### 88.3.4 动作空间

Deep-WBC 常用关节位置目标动作：

$$
a_t
=
\Delta q_{target}
\in\mathbb{R}^{18}
$$

底层 PD 生成力矩：

$$
\tau
=
K_p(q_{nom}+\alpha a_t-q)
-
K_d\dot{q}
$$

为什么不用直接力矩？

| 动作 | 优势 | 风险 |
|---|---|---|
| 关节位置目标 | 探索更安全，动作空间平滑 | 高频力控能力受 PD 限制 |
| 关节速度目标 | 平滑，适合运动学控制 | 力控弱 |
| 关节力矩 | 表达能力强 | 探索危险，sim-to-real 难 |
| 残差动作 | 可结合 model-based 先验 | 需要额外基线控制器 |

对于入门和复现，关节位置目标是更稳妥的选择。

### 88.3.4 终止条件

训练中常见终止条件：

| 条件 | 目的 |
|---|---|
| base height 过低 | 判定摔倒 |
| roll/pitch 超阈值 | 判定失稳 |
| 关节越限 | 保护仿真和硬件 |
| 末端或身体碰撞 | 避免学到撞击策略 |
| episode 时间到 | 正常结束 |
| 任务失败超时 | 防止长时间无效采样 |

终止条件过严会导致探索不足。

终止条件过松会让策略在摔倒边缘学习奇怪行为。

### 88.3.5 环境配置代码框架

```python
from dataclasses import dataclass
import torch

@dataclass
class WidowGo1ObsCfg:
    history_len: int = 10
    num_joints: int = 18
    use_ee_goal: bool = True

class QuadArmEnv:
    def compute_observations(self):
        obs = []
        obs.append(self.base_ang_vel * 0.25)
        obs.append(self.projected_gravity)
        obs.append(self.commands[:, :3])
        obs.append((self.dof_pos - self.default_dof_pos) * 1.0)
        obs.append(self.dof_vel * 0.05)
        obs.append(self.last_actions)
        obs.append(self.ee_goal_body)
        obs_now = torch.cat(obs, dim=-1)
        self.obs_history = torch.roll(self.obs_history, shifts=-1, dims=1)
        self.obs_history[:, -1, :] = obs_now
        return self.obs_history.reshape(self.num_envs, -1)

    def apply_actions(self, actions):
        actions = torch.clamp(actions, -1.0, 1.0)
        q_des = self.default_dof_pos + self.action_scale * actions
        self.torques = self.kp * (q_des - self.dof_pos) - self.kd * self.dof_vel
        self.torques = torch.clamp(self.torques, -self.torque_limit, self.torque_limit)
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：训练观测里混入仿真真值**
>
> 例如直接把摩擦系数、真实接触力或 payload 质量喂给 Actor。
>
> 仿真中效果很好，部署时因为拿不到这些量直接失效。
>
> 正确做法：这些量只给 Critic、教师网络或奖励函数。

> 💡 **概念误区：动作维度越直接越好**
>
> 直接力矩动作表达能力强，但探索风险高。
>
> 对四足臂这种高维接触系统，位置目标加 PD 是更稳的起点。

> 🧠 **思维陷阱：观测越多越好**
>
> 无关或噪声大的观测会增加训练难度。
>
> 观测设计应遵循部署可得、物理有用、尺度清晰三个原则。

### 练习 88.3

| # | 练习 | 难度 |
|---|---|---|
| 1 | 为 Go2+Z1 设计一个部署可用的观测向量，列出每项维度和来源。 | ⭐⭐ |
| 2 | 比较关节位置目标和关节力矩动作在探索安全性上的差异。 | ⭐⭐ |
| 3 | 将一个含特权信息的观测配置改成可部署配置，并说明性能可能下降在哪里。 | ⭐⭐⭐ |

---

## 88.4 奖励设计：末端、稳定、能耗和动作平滑 ⭐⭐⭐

这一节是 Deep-WBC 复现的核心。

奖励不是随便加项。

每个奖励项都在表达一个控制目标或安全边界。

### 88.4.1 奖励总式

常见总奖励：

$$
r_t
=
w_{ee}r_{ee}
+
w_{base}r_{base}
+
w_{vel}r_{vel}
+
w_{alive}r_{alive}
-
w_{\tau}c_{\tau}
-
w_{\Delta a}c_{\Delta a}
-
w_{joint}c_{joint}
$$

| 项 | 作用 |
|---|---|
| $r_{ee}$ | 末端目标跟踪 |
| $r_{base}$ | 基座姿态稳定 |
| $r_{vel}$ | 行走速度命令跟踪 |
| $r_{alive}$ | 存活奖励 |
| $c_{\tau}$ | 扭矩和能耗惩罚 |
| $c_{\Delta a}$ | 动作变化率惩罚 |
| $c_{joint}$ | 关节限位和姿态正则 |

### 88.4.2 末端位置奖励

常用指数形式：

$$
r_{ee}
=
\exp\left(
-
\frac{\|p_{ee}-p_{target}\|^2}{\sigma_{ee}^{2}}
\right)
$$

为什么用指数？

线性负误差会让远离目标时奖励非常负，训练早期梯度混乱。

指数奖励在接近目标时提供强信号，远离目标时饱和。

但 $\sigma$ 不能太小。

太小会导致策略早期几乎拿不到正反馈。

### 88.4.3 基座稳定奖励

可以使用 projected gravity：

$$
c_{tilt}
=
g_{b,x}^{2}+g_{b,y}^{2}
$$

也可以惩罚 base angular velocity：

$$
c_{\omega}
=
\|\omega_b\|^2
$$

基座稳定奖励是防止策略为了追手而牺牲身体。

### 88.4.4 动作平滑和能耗

动作变化率：

$$
c_{\Delta a}
=
\|a_t-a_{t-1}\|^2
$$

扭矩惩罚：

$$
c_{\tau}
=
\|\tau_t\|^2
$$

关节速度惩罚：

$$
c_{\dot{q}}
=
\|\dot{q}_j\|^2
$$

这些项看似只是让动作更好看。

实际上它们是 sim-to-real 的关键。

真实电机无法承受仿真中高频抖动的命令。

### 88.4.5 奖励尺度

| 奖励项 | 初始权重建议 | 监控方式 |
|---|---:|---|
| EE tracking | 1.0 | 误差 RMS |
| base stability | 0.5-2.0 | roll/pitch |
| velocity tracking | 0.5-1.0 | base 速度误差 |
| alive | 0.1 | episode length |
| torque penalty | 1e-4 到 1e-3 | 平均扭矩 |
| action rate | 1e-2 到 1e-1 | 动作频谱 |
| joint limit | 1.0 | 越限次数 |

不要只看总 reward。

必须记录每个分项。

总 reward 上升但动作抖动变大，说明策略可能在利用奖励漏洞。

### 88.4.6 奖励代码框架

```python
from dataclasses import dataclass

@dataclass
class RewardWeights:
    ee: float = 1.0
    base: float = 1.0
    vel: float = 0.5
    alive: float = 0.1
    torque: float = 1e-4
    action_rate: float = 2e-2
    joint: float = 1.0

def reward_ee_tracking(self):
    err = torch.norm(self.ee_pos - self.ee_target, dim=-1)
    return torch.exp(-(err ** 2) / (self.cfg.reward.ee_sigma ** 2))

def reward_base_stability(self):
    # projected_gravity[:, 2] 接近 -1 表示机身 z 轴接近世界 z 轴。
    tilt = torch.sum(self.projected_gravity[:, :2] ** 2, dim=-1)
    return torch.exp(-tilt / 0.1)

def reward_velocity_tracking(self):
    lin_err = torch.sum(
        (self.base_lin_vel[:, :2] - self.command_lin_vel[:, :2]) ** 2,
        dim=-1,
    )
    yaw_err = (self.base_ang_vel[:, 2] - self.command_yaw_rate) ** 2
    return torch.exp(-(lin_err + 0.5 * yaw_err) / (self.cfg.reward.vel_sigma ** 2))

def reward_alive(self):
    return torch.ones(self.num_envs, device=self.device)

def penalty_action_rate(self):
    return torch.sum((self.actions - self.last_actions) ** 2, dim=-1)

def penalty_torque(self):
    return torch.sum(self.torques ** 2, dim=-1)

def penalty_joint_regularization(self):
    posture = torch.sum(
        (self.joint_pos - self.default_joint_pos) ** 2,
        dim=-1,
    )
    lower = torch.relu(self.joint_limits_lower - self.joint_pos)
    upper = torch.relu(self.joint_pos - self.joint_limits_upper)
    limits = torch.sum(lower ** 2 + upper ** 2, dim=-1)
    return posture + self.cfg.reward.limit_scale * limits

def compute_reward(self):
    terms = {
        "ee": self.reward_ee_tracking(),
        "base": self.reward_base_stability(),
        "vel": self.reward_velocity_tracking(),
        "alive": self.reward_alive(),
        "torque": self.penalty_torque(),
        "action_rate": self.penalty_action_rate(),
        "joint": self.penalty_joint_regularization(),
    }
    reward = (
        self.w.ee * terms["ee"]
        + self.w.base * terms["base"]
        + self.w.vel * terms["vel"]
        + self.w.alive * terms["alive"]
        - self.w.torque * terms["torque"]
        - self.w.action_rate * terms["action_rate"]
        - self.w.joint * terms["joint"]
    )
    return reward, terms
```

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：正则项压过追踪项**
>
> 如果能耗、动作平滑、关节正则太强，策略会学会不动。
>
> 先只训练基础追踪，再逐步加入正则并记录分项量级。

> 💡 **概念误区：reward 越高行为越好**
>
> 策略可能通过抖动、卡边界、利用仿真接触漏洞获得高分。
>
> 必须结合视频、动作频谱、接触力和能耗指标评估。

> 🧠 **思维陷阱：末端奖励越强越能学会操作**
>
> 末端奖励过强会牺牲稳定性。
>
> 四足臂奖励应体现"先站住，再操作"。

### 练习 88.4

| # | 练习 | 难度 |
|---|---|---|
| 1 | 训练三组权重：高 EE、高 base、平衡配置，比较末端误差和摔倒率。 | ⭐⭐ |
| 2 | 删除 action rate 惩罚，观察动作频谱和仿真行为。 | ⭐⭐ |
| 3 | 设计一个奖励漏洞检测表，至少覆盖抖动、贴地、越限和假接触。 | ⭐⭐⭐ |

---

## 88.5 ROA：从 RMA 到在线适应 ⭐⭐⭐⭐

这一节讲 Deep-WBC 的关键迁移机制。

回顾足式/190：RMA 使用特权信息训练教师，再用历史观测训练适应模块估计 latent。

Deep-WBC 使用 Regularized Online Adaptation。

它强调在线适应过程和正则项，让策略在部署时能从近期历史中调整行为。

### 88.5.1 为什么需要适应

四足臂的 sim-to-real 差距比纯四足更大。

| 差距 | 四足臂中的表现 |
|---|---|
| 质量偏差 | 臂、夹爪、负载改变 CoM |
| 摩擦差异 | 脚底和末端接触都不确定 |
| 延迟 | 臂和腿执行器延迟不同 |
| backlash | 轻量机械臂间隙明显 |
| 传感噪声 | 末端目标和基座估计都有噪声 |

固定策略很容易过拟合一个仿真参数点。

适应模块试图从历史中推断这些隐含参数的影响。

### 88.5.2 RMA 的基本形式

Teacher 使用特权变量 $e_t$：

$$
a_t
=
\pi_{\theta}(o_t,z_t)
$$

$$
z_t
=
\mu_{\phi}(e_t)
$$

Student 用历史观测估计：

$$
\hat{z}_t
=
\psi_{\eta}(o_{t-H:t},a_{t-H:t-1})
$$

蒸馏损失：

$$
L_{adapt}
=
\|\hat{z}_t-z_t\|^2
$$

### 88.5.3 ROA 的直觉

ROA 的重点是让适应在训练过程中保持稳定。

可以把它理解为给在线估计加约束。

如果 latent 在相邻时间剧烈跳动，策略动作也会跳。

因此需要正则：

$$
L_{roa}
=
L_{ppo}
+
\beta\|\hat{z}_t-z_t\|^2
+
\gamma\|\hat{z}_t-\hat{z}_{t-1}\|^2
$$

第二项逼近教师 latent。

第三项让在线适应平滑。

### 88.5.4 为什么历史窗口有效

历史窗口包含"动作-响应"关系。

例如同样的腿部动作：

| 环境 | 身体响应 |
|---|---|
| 高摩擦 | 足端不滑，base 速度符合预期 |
| 低摩擦 | 足端打滑，base 速度偏小 |
| 重负载 | 加速度变小，pitch 更明显 |
| 高延迟 | 响应滞后 |

策略不能直接看到摩擦和负载。

但可以从历史响应中推断。

### 88.5.5 适应模块代码框架

```python
import torch
import torch.nn as nn

class AdaptationModule(nn.Module):
    def __init__(self, obs_dim, action_dim, history_len, latent_dim):
        super().__init__()
        self.input_dim = history_len * (obs_dim + action_dim)
        self.net = nn.Sequential(
            nn.Linear(self.input_dim, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, latent_dim),
        )

    def forward(self, obs_hist, act_hist):
        x = torch.cat([obs_hist, act_hist], dim=-1)
        x = x.reshape(x.shape[0], -1)
        return self.net(x)

def roa_loss(z_student, z_teacher, z_prev, beta=1.0, gamma=0.05):
    match = torch.mean((z_student - z_teacher) ** 2)
    smooth = torch.mean((z_student - z_prev.detach()) ** 2)
    return beta * match + gamma * smooth
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：历史 buffer reset 不完整**
>
> episode reset 后如果历史仍包含上一轮数据，适应模块会读到错误上下文。
>
> 必须同步 reset 观测历史、动作历史和 latent 状态。

> 💡 **概念误区：适应模块能弥补所有仿真误差**
>
> 它只能利用观测中可辨识的差异。
>
> 如果某个真实误差完全不反映在历史观测里，策略无法推断。

> 🧠 **思维陷阱：历史窗口越长越好**
>
> 太短不足以辨识慢变量。
>
> 太长会引入过期信息和训练负担。
>
> 需要根据控制频率和环境变化时间尺度选择。

### 练习 88.5

| # | 练习 | 难度 |
|---|---|---|
| 1 | 训练短历史和长历史两组适应模块，对比低摩擦环境中的末端误差。 | ⭐⭐⭐ |
| 2 | 关闭 latent 平滑正则，观察动作变化率和 base 姿态。 | ⭐⭐⭐ |
| 3 | 设计一个只改变 payload 的 DR 实验，验证适应模块是否能推断负载变化。 | ⭐⭐⭐⭐ |

---

## 88.6 Advantage Mixing：腿臂因果解耦的优势估计 ⭐⭐⭐⭐

这一节解释 Deep-WBC 的另一个关键思想。

PPO 依赖 advantage。

如果优势估计把腿和臂的贡献混在一起，学习会变慢。

### 88.6.1 问题：一个总回报难以归因

总奖励包含腿和臂：

$$
r_t
=
r_t^{leg}
+
r_t^{arm}
+
r_t^{shared}
$$

腿动作影响稳定和移动。

臂动作影响末端跟踪。

但二者互相影响。

例如末端误差变大，可能是臂没追上，也可能是基座晃动导致目标相对移动。

单一 advantage 会把这些贡献混在一起。

### 88.6.2 优势函数回顾

PPO 使用：

$$
A_t
=
G_t
-
V(s_t)
$$

或者 GAE：

$$
A_t^{GAE}
=
\sum_{l=0}^{\infty}
(\gamma\lambda)^l
\delta_{t+l}
$$

其中：

$$
\delta_t
=
r_t+\gamma V(s_{t+1})-V(s_t)
$$

### 88.6.3 分解直觉

如果动作可以分成：

$$
a_t
=
\begin{bmatrix}
a_t^{leg}\\
a_t^{arm}
\end{bmatrix}
$$

则可以为不同动作子块提供更有针对性的优势信号。

直觉上：

| 动作子块 | 更应关注的奖励 |
|---|---|
| 腿动作 | base 稳定、速度、能耗、支撑 |
| 臂动作 | EE tracking、臂正则、碰撞 |
| 共享部分 | 全身稳定、任务成功 |

Advantage Mixing 不是说腿臂完全独立。

它是在承认耦合的前提下，减少信用分配噪声。

### 88.6.4 一个简化公式

设：

$$
A_t^{leg}
=
A(r^{leg}+r^{shared})
$$

$$
A_t^{arm}
=
A(r^{arm}+r^{shared})
$$

策略损失可写成分块加权：

$$
L_{\pi}
=
L_{\pi}^{leg}(A^{leg})
+
L_{\pi}^{arm}(A^{arm})
$$

其中两个动作子块使用同一个网络输出，但在损失中使用不同 advantage。

### 88.6.5 实现框架

```python
def split_rewards(reward_terms):
    r_leg = (
        reward_terms["base_stability"]
        + reward_terms["velocity_tracking"]
        - reward_terms["leg_energy"]
    )
    r_arm = (
        reward_terms["ee_tracking"]
        - reward_terms["arm_energy"]
        - reward_terms["collision"]
    )
    r_shared = reward_terms["alive"] - reward_terms["action_rate"]
    return r_leg, r_arm, r_shared

def policy_loss_with_mixing(log_prob_leg, log_prob_arm,
                            old_log_prob_leg, old_log_prob_arm,
                            adv_leg, adv_arm, clip=0.2):
    ratio_leg = torch.exp(log_prob_leg - old_log_prob_leg)
    ratio_arm = torch.exp(log_prob_arm - old_log_prob_arm)
    loss_leg = -torch.min(
        ratio_leg * adv_leg,
        torch.clamp(ratio_leg, 1.0 - clip, 1.0 + clip) * adv_leg)
    loss_arm = -torch.min(
        ratio_arm * adv_arm,
        torch.clamp(ratio_arm, 1.0 - clip, 1.0 + clip) * adv_arm)
    return torch.mean(loss_leg + loss_arm)
```

### 88.6.6 边界条件

Advantage Mixing 有帮助，但不是魔法。

如果臂动作强烈影响腿稳定，过度分解会错误归因。

例如举重物时，臂关节动作直接改变 CoM。

此时共享奖励和耦合项必须保留。

| 场景 | 分解强度 |
|---|---|
| 轻量臂空中跟踪 | 可较强分解 |
| 重物搬运 | 需要更多共享项 |
| 推门强接触 | 需要力和支撑共享 |
| 静态末端到达 | 分解较有效 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：奖励分项命名和日志不一致**
>
> Advantage Mixing 依赖奖励分项。
>
> 如果日志和实际计算不一致，很难调试信用分配。

> 💡 **概念误区：腿臂可以完全解耦**
>
> 四足臂动力学本质耦合。
>
> Advantage Mixing 是降低学习噪声，不是物理解耦。

> 🧠 **思维陷阱：分解越细越好**
>
> 过细分解会让共享稳定性被削弱。
>
> 奖励分解应对应动作因果结构，而不是对应所有文件函数。

### 练习 88.6

| # | 练习 | 难度 |
|---|---|---|
| 1 | 关闭 Advantage Mixing，记录 arm reward 和 leg reward 的收敛速度。 | ⭐⭐⭐ |
| 2 | 为推门任务重新设计 leg/arm/shared 三类奖励分解。 | ⭐⭐⭐ |
| 3 | 分析重负载搬运时 Advantage Mixing 可能错误归因的场景。 | ⭐⭐⭐⭐ |

---

## 88.7 Domain Randomization 与课程学习 ⭐⭐⭐

这一节把训练从"仿真能跑"推向"可能迁移"。

四足臂的随机化范围要覆盖腿和臂。

### 88.7.1 随机化维度

| 类别 | 参数 | 作用 |
|---|---|---|
| 机身 | base mass、CoM offset | 覆盖安装误差 |
| 腿 | motor strength、joint friction | 覆盖执行器差异 |
| 臂 | link mass、backlash、payload | 覆盖机械臂和载荷 |
| 地面 | friction、restitution、roughness | 覆盖接触 |
| 传感器 | IMU noise、joint noise | 覆盖测量误差 |
| 执行 | action delay、PD gain | 覆盖控制链延迟 |
| 目标 | EE goal noise、target delay | 覆盖高层误差 |

### 88.7.2 不能随机化得过猛

随机化过小会过拟合。

随机化过大会让任务难以学习。

合理流程：

1. 先在无随机化下验证奖励和动作空间。
2. 加入轻微质量和摩擦随机化。
3. 加入延迟和噪声。
4. 加入 payload 和目标扰动。
5. 最后做极端参数测试。

### 88.7.3 课程学习

四足臂任务可按三个维度逐渐加难：

| 维度 | 简单 | 困难 |
|---|---|---|
| locomotion | 原地站立 | 速度跟踪、转弯、粗糙地形 |
| manipulation | 静态目标 | 移动目标、6D 姿态、接触 |
| disturbance | 无扰动 | push、payload、低摩擦 |

课程学习不是降低标准。

它是让策略先学到可用行为，再逐步扩大能力。

### 88.7.4 随机化配置示例

```python
class RandomizationCfg:
    friction_range = (0.4, 1.2)
    base_mass_delta = (-1.0, 1.0)
    arm_link_mass_scale = (0.8, 1.2)
    payload_mass_range = (0.0, 0.5)
    motor_strength_range = (0.8, 1.2)
    action_delay_steps = (0, 2)
    imu_noise_std = 0.02
    joint_pos_noise_std = 0.01
    ee_goal_noise_std = 0.01

def randomize_episode(env_ids):
    friction = sample_uniform(RandomizationCfg.friction_range, env_ids)
    payload = sample_uniform(RandomizationCfg.payload_mass_range, env_ids)
    motor = sample_uniform(RandomizationCfg.motor_strength_range, env_ids)
    apply_physics_randomization(env_ids, friction, payload, motor)
```

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：策略还没学会就加入强随机化**
>
> 训练早期需要足够稳定的奖励信号。
>
> 先保证基础任务收敛，再逐步加随机化。

> 💡 **概念误区：随机化范围越大迁移越好**
>
> 过大范围会让策略为了兼容极端情况而牺牲常见情况性能。
>
> 范围应由系统辨识和硬件经验决定。

> 🧠 **思维陷阱：只随机化腿，不随机化臂**
>
> 四足臂的主要扰动可能来自臂质量、载荷和目标误差。
>
> 只随机化地面摩擦不够。

### 练习 88.7

| # | 练习 | 难度 |
|---|---|---|
| 1 | 设计四足臂 DR 表，至少包含 15 个参数和取值范围。 | ⭐⭐ |
| 2 | 比较无 payload 随机化和有 payload 随机化策略在载物任务中的性能。 | ⭐⭐⭐ |
| 3 | 设计三维课程：速度、EE 目标距离、扰动强度，并写出升级规则。 | ⭐⭐⭐ |

---

## 88.7B PPO 超参数敏感性分析：四足臂的特殊考量 ⭐⭐⭐

四足臂 RL 的训练稳定性对 PPO 超参数非常敏感。本节给出敏感性排序和调参直觉。

### 88.7B.1 超参数敏感性排序

根据消融实验经验，对四足臂训练影响从大到小排列：

| 排序 | 超参数 | 典型范围 | 敏感度 | 影响机制 |
|---:|---|---|---|---|
| 1 | learning rate | 1e-4 到 5e-4 | 极高 | 过大导致策略崩溃，过小收敛太慢 |
| 2 | clip ratio $\epsilon$ | 0.1 到 0.3 | 高 | 过大策略更新过猛，腿臂耦合导致连锁崩溃 |
| 3 | entropy coefficient | 0.001 到 0.01 | 高 | 过小探索不足，过大动作噪声影响稳定 |
| 4 | GAE $\lambda$ | 0.9 到 0.99 | 中 | 影响信用分配质量 |
| 5 | discount $\gamma$ | 0.99 到 0.999 | 中 | 影响策略对长期目标的重视程度 |
| 6 | minibatch size | 每次更新用多少样本 | 中低 | 影响梯度估计方差 |
| 7 | epochs per update | 3 到 8 | 低 | 过多可能过拟合当前 batch |

### 88.7B.2 四足臂特有的敏感点

与纯四足相比，四足臂 PPO 训练有三个额外敏感点。

**第一，学习率衰减更关键。** 纯四足策略在 reward 上升后学习率保持不变往往也能稳定。四足臂因为腿臂奖励耦合，reward 上升后策略可能过度利用某一子任务。建议使用线性或余弦衰减，从 3e-4 降到 1e-5。

**第二，clip ratio 不能太大。** 纯四足用 $\epsilon = 0.2$ 是标准值。四足臂建议从 $\epsilon = 0.15$ 开始。原因是：腿部动作的一次大更新可能通过角动量耦合影响末端精度，导致下一轮 arm reward 突然下降，进而触发"腿臂拉锯"式训练震荡。

**第三，entropy bonus 需要分阶段。** 训练早期需要较高 entropy（0.01）来探索腿臂协调模式。训练后期应降低到 0.001，让策略精细化末端跟踪。如果 entropy 始终很高，末端 RMS 会停在 5-8 cm 而无法继续下降。

### 88.7B.3 调参决策树

```text
reward 不涨？
  ├─ 检查分项奖励是否有正值
  │    ├─ 全为负 → 降低 action scale，降低 DR
  │    └─ 有正有负 → 检查权重比例
  ├─ entropy 在升高？
  │    └─ 策略在"忘记"已学行为 → 降低 lr
  └─ value loss 发散？
       └─ critic 训练不稳定 → 减小 minibatch 或降低 lr

reward 涨了但动作抖动？
  ├─ entropy 过高 → 降低 entropy coeff
  ├─ clip ratio 过大 → 降低到 0.15
  └─ action rate penalty 不足 → 增大 action_rate 权重

reward 涨了但真机失败？
  └─ 见 88.8 sim-to-real 部分
```

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：使用纯四足的超参数直接训练四足臂**
>
> 纯四足的成功配置（lr=3e-4, clip=0.2, entropy=0.01）不能直接套用。
>
> 四足臂的动作维度从 12 增加到 18，奖励结构更复杂，需要更保守的更新。

> 💡 **概念误区：学习率越大训练越快**
>
> 四足臂中学习率过大会导致"策略振荡"：这一步学会站稳，下一步因为更新过猛忘记怎么站。
>
> 正确策略是用较小学习率配合更多并行环境来提高样本效率。

---

## 88.7C Sim-to-Real 部署管线 ⭐⭐⭐

将仿真中训练好的策略部署到真机是 Deep-WBC 落地的最后一步。本节梳理从"仿真训练完成"到"真机可运行"的完整管线。

### 88.7C.1 管线总览

```text
仿真训练 (IsaacGym/IsaacLab, 4096 envs)
    │ policy checkpoint
    ▼
Sim2Sim 验证 (MuJoCo / 不同物理参数)
    │ 确认鲁棒性
    ▼
策略导出 (ONNX / TorchScript / JIT)
    │ 推理引擎
    ▼
真机观测管线 (IMU, 编码器, 目标系统)
    │ 归一化、滤波、坐标系对齐
    ▼
动作后处理 (缩放, 限幅, PD 控制)
    │ 关节命令
    ▼
安全监控 (姿态限, 力矩限, 超时)
    │ 降级或停机
    ▼
硬件执行
```

### 88.7C.2 关键步骤详解

**Sim2Sim 验证**：在与训练不同的仿真器中测试。例如训练用 IsaacGym，验证用 MuJoCo。如果两个仿真器中行为一致，说明策略没有过拟合训练仿真器的特定接触模型或数值积分器。

**观测管线对齐**：真机观测和仿真观测必须逐项对齐。最常见的错误包括：

| 错误 | 后果 | 检查方法 |
|---|---|---|
| IMU 坐标系不一致 | 重力方向投影错误 | 静置时检查 projected gravity 是否为 $(0, 0, -1)$ |
| 关节零位偏移 | 默认站立姿态不对 | 手动把机器人摆到标定姿态，比较读数 |
| 观测缩放遗漏 | 某些输入量级错误 | 记录训练时的缩放系数，部署时严格复用 |
| 延迟不匹配 | 动作和观测不同步 | 训练时加入 1-2 步延迟随机化 |

**安全监控**：真机必须有独立于策略的安全层。

| 监控项 | 阈值 | 触发动作 |
|---|---|---|
| base roll/pitch | > 30 deg | 立即切换到阻尼关节 |
| 关节越限 | 超过 soft limit 95% | 限制该关节动作 |
| 扭矩持续饱和 | > 1 秒 | 降低动作 scale |
| 通信超时 | > 20 ms | 保持上一命令 + 减速 |

### 88.7C.3 执行器模型的 Sim2Real 差距

四足臂 sim2real 中最难弥合的差距不是质量和摩擦，而是执行器模型。

| 仿真假设 | 真实情况 | 影响 |
|---|---|---|
| PD 控制器瞬时响应 | 电机有响应延迟和带宽限制 | 高频动作被滤波 |
| 扭矩无上限噪声 | 电流纹波和减速器摩擦 | 低扭矩区域精度差 |
| 关节阻尼恒定 | 温度和速度相关 | 长时间运行行为漂移 |
| 无通信延迟 | CAN/EtherCAT 有固定延迟 | 控制环路相位裕度下降 |

---

## 88.7D 与并发工作的对比 ⭐⭐⭐

Deep-WBC 不是四足臂 RL 控制的唯一路线。理解并发工作的设计选择有助于判断何时使用哪种方法。

### 88.7D.1 RoboDuet（2024）

RoboDuet 选择**双策略架构**：一个策略专门负责腿部运动（locomotion policy），另一个策略专门负责臂部操作（manipulation policy）。

| 维度 | Deep-WBC | RoboDuet |
|---|---|---|
| 策略数量 | 1 个统一策略 | 2 个独立策略 |
| 腿臂协调 | 网络内部隐式学习 | 通过共享 base 状态和残差接口协调 |
| 训练难度 | 高维动作空间、奖励耦合 | 分别训练更容易，但接口设计难 |
| 可迁移性 | 腿臂必须一起训练 | 可独立更换腿或臂策略 |
| 性能上限 | 紧密耦合时可能更优 | 松耦合时更稳 |

> **本质洞察**：Deep-WBC 和 RoboDuet 反映了控制系统设计中"集中式 vs 分布式"的经典权衡。
> 集中式（统一策略）理论上能找到全局更优解，但训练更难、可解释性更差。
> 分布式（双策略）更模块化、更可调试，但可能在强耦合任务中次优。

### 88.7D.2 HATO（2024）

HATO（Hand-Arm-Torso Optimization）关注的是灵巧手+臂+移动基座的全身操作。它与 Deep-WBC 的主要区别在于：

- **动作空间更大**：加入灵巧手后动作维度可达 30+。
- **任务更复杂**：不只是 EE 位置跟踪，而是手指级操作。
- **训练策略**：大量使用基于演示的预训练，再用 RL 微调。

| 维度 | Deep-WBC | HATO |
|---|---|---|
| 末端 | 简单夹爪 | 灵巧手 |
| 动作维度 | 18 | 30+ |
| 训练范式 | 纯 RL | 演示预训练 + RL |
| 操作复杂度 | EE 位置跟踪 | 手指级精细操作 |
| 真机验证 | A1+WidowX | 人形或类人平台 |

### 88.7D.3 选型建议

| 任务 | 推荐方法 | 理由 |
|---|---|---|
| 教学入门 | Deep-WBC | 代码最完整，概念最清晰 |
| 模块化部署 | RoboDuet | 可独立调试腿和臂 |
| 灵巧操作 | HATO 路线 | 需要手指级控制 |
| 安全关键 | model-based + RL 残差 | 需要可验证约束 |
| 快速原型 | Deep-WBC + 简化奖励 | 端到端最快 |

### 练习 88.7BCD

| # | 练习 | 难度 |
|---|---|---|
| 1 | 训练两组不同 clip ratio 的策略，记录训练曲线中 policy loss 的方差。 | ⭐⭐⭐ |
| 2 | 设计一个 Sim2Sim 验证流程：列出必须对比的 5 个行为指标。 | ⭐⭐ |
| 3 | 为 RoboDuet 双策略架构设计腿-臂接口，说明共享哪些观测、如何处理冲突动作。 | ⭐⭐⭐ |

---

## 88.8 IsaacGym 到 IsaacLab：迁移路线 ⭐⭐

Deep-WBC 原始生态基于 IsaacGym 和 legged_gym。

新项目更建议迁移到 IsaacLab。

迁移不是逐行翻译。

迁移是把环境的概念映射到新的管理器结构。

### 88.8.1 迁移对照

| legged_gym / IsaacGym | IsaacLab |
|---|---|
| `LeggedRobot` env class | `ManagerBasedRLEnv` |
| reward functions | `RewardManager` |
| observation buffer | `ObservationManager` |
| command sampler | `CommandManager` |
| reset/randomize | `EventManager` |
| terrain curriculum | `CurriculumManager` |
| hand-written cfg | dataclass config |

### 88.8.2 最小迁移步骤

1. 导入四足臂 USD 或 URDF 资产。
2. 定义关节名、默认姿态和 actuator 配置。
3. 定义观测项。
4. 定义动作项和 action scale。
5. 定义奖励项。
6. 定义 reset 和随机化事件。
7. 先训练站立 EE tracking。
8. 再加入行走命令。

### 88.8.3 IsaacLab 配置骨架

```python
@configclass
class QuadArmActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True,
    )

@configclass
class QuadArmRewardsCfg:
    ee_tracking = RewTerm(func=mdp.ee_tracking_exp, weight=1.0)
    base_stability = RewTerm(func=mdp.base_stability, weight=1.0)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.02)
    joint_limits = RewTerm(func=mdp.joint_limits, weight=-1.0)

@configclass
class QuadArmEnvCfg(ManagerBasedRLEnvCfg):
    actions = QuadArmActionsCfg()
    rewards = QuadArmRewardsCfg()
    decimation = 4
    episode_length_s = 20.0
```

### 88.8.4 迁移验证

| 验证 | 通过标准 |
|---|---|
| 资产加载 | 所有关节和 body 名正确 |
| 默认站立 | 不施加随机动作时不爆炸 |
| 动作方向 | 单关节动作方向正确 |
| 奖励分项 | 每项均有合理量级 |
| reset | 历史 buffer 清零 |
| 并行环境 | 不同环境随机化互不污染 |

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：迁移时没有锁定版本**
>
> IsaacLab 和 Isaac Sim 更新较快。
>
> 训练结果需要记录版本、驱动、CUDA、PyTorch 和配置。

> 💡 **概念误区：迁移只改 API**
>
> 仿真器接触、actuator、随机化和 reset 语义都可能变化。
>
> 迁移后必须重新验证奖励量级和行为。

> 🧠 **思维陷阱：一开始就迁移完整任务**
>
> 应从站立、单目标 EE tracking、速度命令逐步扩展。
>
> 完整任务失败时很难定位问题。

### 练习 88.8

| # | 练习 | 难度 |
|---|---|---|
| 1 | 把一个 legged_gym 奖励函数改写成 IsaacLab RewardTerm。 | ⭐⭐ |
| 2 | 在 IsaacLab 中实现最小四足臂站立环境，并验证动作方向。 | ⭐⭐⭐ |
| 3 | 对比 IsaacGym 和 IsaacLab 下同一策略的 sim2sim 行为差异。 | ⭐⭐⭐ |

---

## 88.9 评估与局限：Deep-WBC 能做什么，不能做什么 ⭐⭐

这一节给出清醒边界。

Deep-WBC 是强基线，但不是最终答案。

### 88.9.1 评估指标

| 类别 | 指标 |
|---|---|
| 操作 | EE position RMS、goal success rate |
| 稳定 | episode length、fall rate、base roll/pitch |
| 平滑 | action rate、torque RMS、joint velocity |
| 鲁棒 | 摩擦变化、payload、push recovery |
| 迁移 | sim2sim、真实延迟、噪声注入 |
| 学习 | reward curve、KL、value loss、entropy |

### 88.9.2 与基线对比

| 基线 | 比较目的 |
|---|---|
| 纯四足策略 + 独立臂 IK | 验证统一策略是否更会补偿基座 |
| MPC+WBC | 验证精度和安全边界 |
| RMA | 验证 ROA 的在线适应价值 |
| RoboDuet 双策略 | 验证统一策略和分工策略的取舍 |
| Blind 无目标策略 | 验证目标观测有效性 |

### 88.9.3 局限

| 局限 | 后果 | 后续章节 |
|---|---|---|
| 无视觉 | 目标需要外部给定 | 复合/190_Visual_WBC精读 |
| 无显式硬约束 | 低摩擦和力矩边界风险 | 复合/210_RAMBO混合MPC_RL |
| 真机部署代码不完整 | 复现门槛高 | 后续部署章节 |
| 接触力控弱 | 推门、插入类任务不稳 | UMI/RAMBO/力敏感章节 |
| IsaacGym 生态旧 | 新项目需迁移 | IsaacLab 路线 |

### ⚠️ 常见陷阱

> ⚠️ **评价陷阱：只报告成功率**
>
> 成功率不能说明动作是否平滑、安全、可部署。
>
> 必须同时报告能耗、动作变化率、姿态和失败模式。

> 💡 **概念误区：无视觉不重要**
>
> 没有视觉意味着目标位姿必须由外部系统给定。
>
> 在真实移动操作中，目标感知是任务成败的一半。

> 🧠 **思维陷阱：策略在仿真中鲁棒就能真机部署**
>
> 还需要执行器模型、延迟、状态估计、安全过滤和硬件保护。

### 练习 88.9

| # | 练习 | 难度 |
|---|---|---|
| 1 | 设计 Deep-WBC 与 MPC+WBC 的公平对比实验，列出控制频率和指标。 | ⭐⭐ |
| 2 | 给一个训练日志，判断是否出现 reward hacking，并提出修正。 | ⭐⭐⭐ |
| 3 | 设计 Deep-WBC 接入视觉目标的接口，不改变低层动作空间。 | ⭐⭐⭐ |

---

## 88.10 本章小结

### 核心概念表

| 概念 | 一句话理解 |
|---|---|
| 统一策略 | 一个网络同时输出腿和臂的关节目标 |
| 观测历史 | 用动作-响应序列隐式估计摩擦、负载和延迟 |
| 关节位置动作 | 用 PD 提供探索安全边界 |
| EE 奖励 | 把操作目标转成可学习信号 |
| ROA | 在线估计环境 latent，并用正则保持稳定 |
| Advantage Mixing | 给腿和臂动作更合适的优势信号 |
| DR | 让训练分布覆盖真实参数变化 |
| 课程学习 | 从站立和简单目标逐步过渡到行走操作 |

### 核心公式速查

| 公式 | 含义 |
|---|---|
| $\tau=K_p(q_{nom}+\alpha a-q)-K_d\dot{q}$ | 关节位置动作到力矩 |
| $r_{ee}=\exp(-\|p_{ee}-p_{target}\|^2/\sigma^2)$ | 末端跟踪奖励 |
| $A_t=\sum_l(\gamma\lambda)^l\delta_{t+l}$ | GAE |
| $L_{roa}=L_{ppo}+\beta\|\hat z-z\|^2+\gamma\|\hat z_t-\hat z_{t-1}\|^2$ | 在线适应正则 |
| $L_\pi=L_\pi^{leg}(A^{leg})+L_\pi^{arm}(A^{arm})$ | Advantage Mixing 简化形式 |

### 与后续章节的连接

| 后续章节 | 连接 |
|---|---|
| 复合/190_Visual_WBC精读 | 在 Deep-WBC 基础上加入视觉高层和蒸馏 |
| 复合/200_UMI_on_Legs精读 | 用操作轨迹作为低层 WBC 目标 |
| 复合/210_RAMBO混合MPC_RL | 用 model-based 前馈补齐 RL 的安全约束 |

---

## 累积项目：IsaacLab 最小 Deep-WBC 复现

目标：实现一个最小四足臂 RL 环境。

### 阶段 1：资产和动作

1. 导入四足臂模型。
2. 设置默认站立姿态。
3. 使用 18 维关节位置目标动作。
4. 验证单关节动作方向。

### 阶段 2：观测和奖励

1. 加入 base angular velocity、projected gravity。
2. 加入 joint position、joint velocity、last action。
3. 加入 EE target in body frame。
4. 实现 EE tracking、base stability、action rate、torque penalty。

### 阶段 3：训练和消融

1. 先训练站立 EE tracking。
2. 加入 base velocity command。
3. 加入轻度 DR。
4. 关闭 Advantage Mixing 做消融。
5. 关闭 ROA 做消融。

### 阶段 4：评估

| 指标 | 目标 |
|---|---|
| EE RMS | 小于 3-5 cm |
| fall rate | 低于 5% |
| action rate | 无高频抖动 |
| payload robustness | 轻载变化下不崩溃 |
| sim2sim | MuJoCo 或不同物理参数下可运行 |

---

## 延伸阅读

| 材料 | 类型 | 难度 | 阅读目标 |
|---|---|---|---|
| Fu et al. 2022 Deep Whole-Body Control | 论文 | ⭐⭐⭐ | 本章主轴 |
| Deep-Whole-Body-Control 仓库 | 代码 | ⭐⭐⭐ | 环境、奖励、ROA、Advantage Mixing |
| Kumar et al. 2021 RMA | 论文 | ⭐⭐⭐ | 适应模块来源 |
| Rudin et al. 2022 Learning to Walk in Minutes | 论文 | ⭐⭐ | GPU 并行训练基础 |
| Margolis 2022 Walk These Ways | 论文 | ⭐⭐⭐ | 行为多样性和腿足 RL |
| IsaacLab 文档 | 文档 | ⭐⭐ | 新平台复现 |
| RoboDuet | 论文/代码 | ⭐⭐⭐ | 统一策略的替代路线 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|---|---|---|---|
| reward 不涨 | 奖励尺度错、学习率不合适、动作缩放过大 | 1. 打印分项奖励 2. 降低 action scale 3. 先关闭 DR | 88.4, 88.7 |
| 策略学会不动 | 正则项过强或 alive 奖励过高 | 1. 降低 torque/action penalty 2. 提高 EE/velocity 奖励 3. 检查命令采样 | 88.4 |
| 末端能追踪但机器人摔倒 | EE 奖励过强，base 稳定不足 | 1. 提高 base stability 2. 降低 EE 权重 3. 加入 fall penalty | 88.4 |
| 动作高频抖动 | action rate 惩罚不足或 latent 不平滑 | 1. 增大 action_rate 2. 加 ROA smooth 3. 检查 policy std | 88.5 |
| 有 DR 后训练崩溃 | 随机化范围过大或过早 | 1. 缩小范围 2. 分阶段课程 3. 先恢复基础收敛 | 88.7 |
| 适应模块无效果 | 历史窗口不足或特权监督不合理 | 1. 增加 history 2. 检查 latent loss 3. 做 payload 单变量测试 | 88.5 |
| Advantage Mixing 变差 | 奖励分解错误，耦合项丢失 | 1. 增加 shared reward 2. 检查 leg/arm reward 日志 3. 做重载场景测试 | 88.6 |
| IsaacLab 迁移后行为异常 | actuator、reset、奖励尺度与原环境不同 | 1. 单关节测试 2. 对比奖励分项 3. 关闭随机化 | 88.8 |
| 仿真成功但实机风险高 | 延迟、扭矩限制、执行器模型不足 | 1. 加动作延迟 2. 加 torque clip 3. 做 sim2sim | 88.7, 88.9 |

---

## 综合项目：Deep-WBC 与 model-based WBC 的混合基线

目标：构建一个保守混合方案，让 Deep-WBC 输出关节位置目标，同时由 model-based monitor 进行安全过滤。

### 项目要求

1. RL 策略输出 18 维关节位置目标。
2. Pinocchio 实时计算 CoM、CMM 和足端 Jacobian。
3. 安全层监控 base roll/pitch、足端法向力估计、动作变化率。
4. 当危险指标超过阈值时，逐步降低臂动作 scale。
5. 当姿态继续恶化时，切换到站立 hold。
6. 记录策略原始动作和过滤后动作。

### 交付物

| 交付 | 内容 |
|---|---|
| 环境配置 | 观测、动作、奖励、DR |
| 网络说明 | Actor、Critic、适应模块 |
| 安全过滤 | 阈值和状态机 |
| 实验曲线 | EE error、base pose、action rate、fall rate |
| 消融 | 无过滤、有过滤、强过滤三组对比 |
