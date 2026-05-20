# 第 S5 章：可微分 MPC 与学习-控制融合

> **本章定位**：可微分 MPC 不是把 MPC 代码包进神经网络那么简单，而是把“最优控制求解器”看成一个可求导的隐式函数层。
> 它连接了三条线：第一条是经典 MPC 的有限时域最优控制问题；第二条是可微分仿真的状态转移梯度；第三条是学习系统中的外层损失与参数更新。
> 学完本章后，你应能判断梯度应该穿过动力学、穿过求解器、穿过代价参数，还是根本不应该穿过某个非光滑边界。

---

## S5.0 前置自测

答不出 2 题以上时，建议先回到 S4 可微分仿真理论、足式/70_腿足简化模型理论、足式/110_OCS2完整栈与双线程MPC、复合/30_多模态MPC 和数学中的非线性优化章节补齐背景。

| # | 问题 | 预期关键词 |
|---|------|------------|
| 1 | 写出一个离散时间 MPC 问题的标准形式，包含动力学约束、路径约束和终端代价。 | $x_{k+1}=f(x_k,u_k,\theta)$，$\ell$，$\ell_f$，$g\le 0$，$h=0$ |
| 2 | iLQR/DDP 的 backward pass 本质上在求解什么？ | 二次近似，Riccati 递推，局部 LQR，前馈项和反馈增益 |
| 3 | SQP 如何把非线性规划变成一串 QP？ | 线性化约束，二次近似拉格朗日函数，信赖域/线搜索 |
| 4 | KKT 条件包含哪几类条件？ | stationarity、原始可行、对偶可行、互补松弛 |
| 5 | 可微分仿真中的接触梯度为什么容易有偏？ | 非光滑、活动集切换、硬接触、事件时间不连续 |

**自测的目的**不是筛人，而是让后面的符号有落点。

本章会反复使用这些符号。

如果符号含义不清，读到隐式函数定理时会觉得只是矩阵求逆。

如果 MPC 结构不清，读到可微层时会误以为 `loss.backward()` 自动解决了一切。

---

## S5.0.1 本章目标

学完本章后，你应能完成五类判断。

| 能力 | 具体表现 |
|------|----------|
| 建模判断 | 能把可微分仿真器、MPC 求解器和学习损失连接成一条清晰的计算图 |
| 数学判断 | 能从 KKT 条件推导 $\partial z^\star/\partial\theta$，理解隐式求导与 unroll 求导的差别 |
| 算法判断 | 能区分 shooting、collocation、iLQR、DDP、SQP 和 IPM 的求导接口 |
| 工程判断 | 能说明代价权重、残差模型、物理参数、终端价值函数分别如何被学习 |
| 安全判断 | 能识别不适合端到端求导的场景，保留 MPC 的稳定性、约束和调试边界 |

本章不会把任何一个框架神化。

mpc.pytorch、Theseus、acados、leap-c、MJX、OCS2、Crocoddyl 都只是不同层面的工具。

真正重要的是看懂它们在计算图中的位置。

---

## S5.0.2 知识地图

可微分 MPC 的知识树可以按“谁对谁求导”展开。

```text
可微分 MPC
├── 对动力学求导
│   ├── 解析导数：Pinocchio / CppAD / CasADi
│   ├── 自动微分：JAX / PyTorch / CppAD
│   └── 可微分仿真：MJX / Brax / Drake AutoDiffXd
├── 对轨迹优化求导
│   ├── unroll solver：把迭代过程展开成计算图
│   ├── implicit differentiation：对最优性条件求导
│   └── hybrid：短展开 + 隐式修正 / 截断反传
├── 对 MPC 参数求导
│   ├── 代价权重 Q/R
│   ├── 参考轨迹和终端代价
│   ├── 残差动力学模型
│   └── 物理参数：质量、摩擦、阻尼、延迟
└── 对外层任务求导
    ├── 模仿学习：专家动作误差
    ├── 强化学习：critic / advantage / return
    ├── 系统辨识：轨迹预测误差
    └── 安全学习：约束裕度和风险指标
```

这个结构能避免一个常见误解。

可微分 MPC 不是“所有东西都可微”。

它只是在合适的边界上提供梯度。

有些边界应该光滑化。

有些边界应该保留为硬约束。

有些边界应该停止梯度。

> **本质洞察**：可微分 MPC 的本质不是让控制器变成神经网络，而是把“求解一个带物理约束的优化问题”变成可被外层学习算法调用的结构化算子。
> 神经网络负责学习难以手写的部分，MPC 负责守住物理、约束和可解释性。

---

## S5.0.3 科研脉络与概念辨析

| 年份 | 工作 | 关键意义 | 本章使用方式 |
|------|------|----------|--------------|
| 2018 | Amos 等，Differentiable MPC | box-constrained iLQR 的 KKT 隐式求导 | 建立可微 MPC 的核心数学 |
| 2022 | Theseus | 可微非线性最小二乘层 | 说明优化层不只服务控制，也服务 SLAM/操作 |
| 2022 | TD-MPC | 学到世界模型 + MPPI | 澄清“名字里有 MPC”不等于对求解器求导 |
| 2024 | TD-MPC2 | 可扩展 model-based RL | 对比采样规划与可微优化 |
| 2025 | Differentiable NMPC / acados 灵敏度 | 通用约束 NMPC 的隐式灵敏度 | 扩展到 SQP/IPM/NLP |
| 2025 | leap-c | acados NMPC 作为 PyTorch 层 | 工程接口示例 |
| 2025 | Actor-Critic MPC | MPC 作为 actor，RL 学 MPC 参数 | 学习-控制融合案例 |

需要先澄清三组概念。

| 容易混淆的说法 | 正确区分 |
|----------------|----------|
| “MPC 在神经网络训练里，所以就是可微 MPC” | 只有梯度穿过 MPC 的最优解映射时，才是本章意义上的可微 MPC |
| “TD-MPC2 很强，所以它是可微 MPC” | TD-MPC2 主要是世界模型和采样式规划，不是对 QP/NLP 求解器做隐式求导 |
| “可微分仿真已经有梯度，所以不需要 MPC 求导” | 仿真梯度描述 $x_{t+1}$ 对参数的敏感性；MPC 求导描述 $u^\star$ 对参数的敏感性 |

这三组区分在工程中很关键。

如果目标是学习世界模型，TD-MPC2 可能比可微 MPC 更适合。

如果目标是保持硬约束并学习权重，leap-c/acados 这类可微 NMPC 更贴近需求。

如果目标是大量接触 locomotion 策略训练，PPO/SHAC/AHAC 可能比把整个 MPC 端到端求导更稳。

---

## S5.1 从普通 MPC 到可微 MPC ⭐⭐⭐

这一节解决一个基本问题：普通 MPC 已经会在线优化控制，为什么还要让它可微？

### S5.1.1 普通 MPC 的输入输出

离散时间 MPC 通常写成：

$$
\begin{aligned}
\min_{\mathbf{x}_{0:N},\mathbf{u}_{0:N-1}} \quad
& \sum_{k=0}^{N-1} \ell_k(x_k,u_k;\theta) + \ell_N(x_N;\theta) \\
\text{s.t.}\quad
& x_0 = \hat{x} \\
& x_{k+1}=f_k(x_k,u_k;\theta), \quad k=0,\ldots,N-1 \\
& h_k(x_k,u_k;\theta)=0 \\
& g_k(x_k,u_k;\theta)\le 0 .
\end{aligned}
$$

这里 $\hat{x}$ 是当前估计状态。

$\theta$ 是一组外部参数。

它可以包含代价权重。

它可以包含动力学参数。

它可以包含参考轨迹。

它也可以包含神经网络输出的残差模型参数。

MPC 求解器返回最优轨迹：

$$
z^\star(\hat{x},\theta)
=
(x_0^\star,\ldots,x_N^\star,u_0^\star,\ldots,u_{N-1}^\star).
$$

部署时只执行第一步控制：

$$
\pi_{\text{MPC}}(\hat{x};\theta)=u_0^\star(\hat{x},\theta).
$$

这已经是一个策略。

区别在于这个策略不是一个显式神经网络，而是一个优化问题的解。

### S5.1.2 外层学习问题

学习系统关心的不是单次 MPC 的局部代价。

它关心外层损失：

$$
\mathcal{L}(\theta)
=
\Phi(\tau(\theta)),
$$

其中 $\tau(\theta)$ 是把 MPC 放入闭环、仿真或真实数据后得到的轨迹。

常见外层损失如下。

| 外层目标 | 损失形式 | 学习对象 |
|----------|----------|----------|
| 模仿学习 | $\sum_t\|u_t^\star(\theta)-u_t^{expert}\|^2$ | 权重、参考生成器、encoder |
| 系统辨识 | $\sum_t\|x_t^{pred}(\theta)-x_t^{real}\|^2$ | 质量、摩擦、阻尼、延迟 |
| 任务性能 | $-\sum_t r(x_t,u_t)$ | 代价权重、终端价值、残差模型 |
| 安全裕度 | $\sum_t \operatorname{softplus}(-m_t)$ | barrier 参数、安全距离、风险权重 |

如果 MPC 不可微，外层只能使用零阶方法。

例如网格搜索权重。

例如 CMA-ES。

例如 PPO 把 MPC 参数当作动作。

这些方法能工作，但样本效率低。

如果 MPC 可微，可以直接计算：

$$
\frac{d\mathcal{L}}{d\theta}
=
\frac{\partial \mathcal{L}}{\partial z^\star}
\frac{\partial z^\star}{\partial \theta}.
$$

这里的难点全部集中在第二项。

### S5.1.3 为什么不能直接把求解器循环全部展开

最直接的思路是把求解器的每一步迭代都放进计算图。

例如 SQP 迭代：

```text
z^0 -> 线性化 -> QP 求解 -> z^1 -> 线性化 -> QP 求解 -> z^2 -> ...
```

如果展开 $T$ 次迭代，反向传播会穿过 $T$ 个求解步骤。

这种方法叫 unrolling。

它的优点是实现概念直接。

它的缺点也明显。

| 问题 | 表现 |
|------|------|
| 内存开销 | 需要保存每次迭代的中间变量 |
| 迭代数敏感 | 训练时求解器迭代次数变化会改变计算图长度 |
| 梯度不稳定 | 线搜索、clamp、活动集切换会制造非光滑点 |
| 误差来源混杂 | 梯度同时包含“最优解敏感性”和“算法迭代路径敏感性” |

隐式求导走另一条路。

它不关心求解器怎么走到最优点。

它只关心最优点满足什么方程。

如果最优点满足 $F(z^\star,\theta)=0$，就对这个方程求导。

这就像用地图和终点坐标计算终点敏感性，而不是回放司机每一次打方向盘。

### S5.1.4 可微 MPC 的三层接口

工程实现时，可以把可微 MPC 分成三层接口。

```text
外层学习器
  输入：观测 obs、任务目标 goal、历史轨迹
  输出：MPC 参数 theta
        │
        ▼
可微 MPC 层
  输入：当前状态 x0、参数 theta
  输出：最优控制 u0*、轨迹 z*
        │
        ▼
可微 / 不可微环境
  输入：u0*
  输出：下一状态、任务损失、约束裕度
```

每一层都可以选择是否求导。

| 梯度路径 | 适用场景 | 风险 |
|----------|----------|------|
| 只穿过 MPC 层 | 学权重、学残差、学终端代价 | 外层环境不可微时需要 surrogate loss |
| 穿过 MPC + 可微仿真 | 系统辨识、短 horizon 策略优化 | 接触梯度偏差、长链梯度爆炸 |
| 不穿过 MPC，只蒸馏其输出 | 需要极快部署 | 失去硬约束保证 |
| 不穿过仿真，只用真实 rollout | 安全关键系统 | 需要低样本、保守更新 |

可微 MPC 的工程价值恰恰在于它允许这些边界清晰存在。

不是所有路径都必须打开。

### S5.1.5 代码示例：把 MPC 看成 PyTorch 层的外形

下面代码不是某个库的完整实现。

它展示的是可微 MPC 层在学习系统中的接口形状。

```python
import torch
from torch import nn


class MPCParameterEncoder(nn.Module):
    """从观测中预测 MPC 参数。

    这里的输出不是关节动作，而是结构化控制器的参数。
    这样做能保留 MPC 的约束和可解释性。
    """
    def __init__(self, obs_dim: int, param_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, param_dim),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        # 输出 theta，例如 Q/R 权重、残差模型系数、终端代价参数。
        raw = self.net(obs)
        # 权重必须为正，softplus 比 exp 更不容易造成数值爆炸。
        theta = torch.nn.functional.softplus(raw) + 1e-4
        return theta


class DifferentiableMPCLayer(nn.Module):
    """可微 MPC 层的接口示意。

    真正的实现会在 forward 中调用 iLQR/SQP/acados/Theseus。
    backward 可以来自隐式求导，而不是展开所有求解迭代。
    """
    def forward(self, x0: torch.Tensor, theta: torch.Tensor) -> torch.Tensor:
        # 这里假设底层求解器返回最优第一步控制 u0_star。
        # 在真实工程中，这个函数通常由自定义 autograd.Function 包装。
        u0_star = solve_mpc_and_return_first_control(x0, theta)
        return u0_star


def imitation_training_step(encoder, mpc_layer, batch, optimizer):
    """用专家动作训练 MPC 参数生成器。"""
    obs = batch["obs"]
    x0 = batch["state"]
    expert_u = batch["expert_action"]

    theta = encoder(obs)
    u = mpc_layer(x0, theta)

    # 外层损失不需要等于 MPC 内部代价。
    # 这里用专家动作误差训练 theta，让 MPC 变得更像专家。
    loss = torch.mean((u - expert_u) ** 2)

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    return float(loss.detach())
```

这段代码有两个重要细节。

第一，神经网络输出的是 $\theta$，不是直接输出 $u$。

第二，`loss.backward()` 的关键不是 PyTorch 能否求导，而是 `mpc_layer` 是否提供了正确的 backward。

如果 backward 只是展开 5 次不收敛的求解迭代，梯度可能是在学习求解器的错误。

如果 backward 来自 KKT 隐式求导，梯度更接近最优策略本身的敏感性。

### S5.1.6 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 把 MPC 内部代价当作外层 loss | 权重学到退化值，控制器只会最小化自己定义的问题 | 内外层目标不是同一个对象 | 明确外层任务损失，MPC 内部代价只是结构化策略 |
| 神经网络直接输出无约束权重 | 训练中出现负权重，Hessian 变不定，求解器崩溃 | MPC 代价需要正定或半正定结构 | 用 softplus、Cholesky 或对角正参数化 |
| 对未收敛的求解器做隐式求导 | backward 结果与有限差分不一致 | KKT 条件没有满足，隐式方程不成立 | 检查 KKT 残差，必要时使用展开求导或停止更新 |
| 盲目打开仿真反传 | 短期 loss 下降，实机更差 | 接触梯度有偏，学习利用仿真光滑化漏洞 | 限制 horizon，加入真实数据校验和安全约束 |

### S5.1.7 练习

1. **概念题**：写出一个外层损失，它不等于 MPC 内部代价，但可以通过 MPC 参数改善任务表现。
2. **推导题**：假设 $u^\star(\theta)=\arg\min_u \frac{1}{2}u^THu+\theta^Tu$，手推 $\frac{du^\star}{d\theta}$。
3. **编程题**：实现一个 PyTorch encoder 输出正的 $Q/R$ 权重，打印训练前后权重变化，并解释哪些权重变大是合理的。

---

## S5.2 MPC 与可微分仿真的接口 ⭐⭐⭐

这一节解决的问题是：MPC 求解的是预测模型，可微分仿真也提供预测模型，两者怎样连接而不互相混淆？

### S5.2.1 三种动力学来源

MPC 需要动力学：

$$
x_{k+1}=f(x_k,u_k;\theta).
$$

动力学可以来自三种来源。

| 来源 | 例子 | 梯度来源 | 优点 | 风险 |
|------|------|----------|------|------|
| 解析模型 | LIPM、SRBD、Centroidal、四旋翼模型 | 手写导数、CppAD、CasADi | 快、可解释、适合实时 | 模型误差 |
| 可微分仿真 | MJX、Brax、Drake AutoDiffXd、Genesis | AD 穿过仿真步 | 与仿真一致，适合辨识 | 接触梯度不可靠 |
| 学习模型 | MLP、latent dynamics、residual model | PyTorch/JAX AD | 捕获难建模效应 | 分布外风险、约束弱 |

这三种不是互斥。

最常见的结构是物理模型加残差：

$$
x_{k+1}
=
f_{\text{phys}}(x_k,u_k;p)
+
r_\phi(x_k,u_k).
$$

$p$ 是物理参数。

$r_\phi$ 是神经网络残差。

MPC 使用这个组合模型预测。

外层训练可以同时更新 $p$ 和 $\phi$。

### S5.2.2 可微分仿真作为 rollout 环境

第一种接口是让 MPC 仍使用简化模型，但外层 loss 通过可微分仿真评估。

```text
theta -> MPC(simple model) -> u0*
                         -> MJX/MuJoCo differentiable rollout -> loss
```

这种结构适合系统辨识和短 horizon 训练。

MPC 的模型可以是 SRBD。

仿真器可以是 MJX 的高保真模型。

如果只评估一次 MPC 求出的开环控制序列，外层梯度会包含两部分。

$$
\frac{d\mathcal{L}}{d\theta}
=
\underbrace{
\frac{\partial \mathcal{L}}{\partial x_{1:T}}
\frac{\partial x_{1:T}}{\partial u_{0:T-1}^\star}
}_{\text{仿真梯度}}
\underbrace{
\frac{\partial u_{0:T-1}^\star}{\partial \theta}
}_{\text{MPC 梯度}}.
$$

这个公式解释了“可微分仿真 + 可微 MPC”的接口。

可微分仿真告诉你控制序列如何影响轨迹。

可微 MPC 告诉你参数如何影响控制序列。

二者通过链式法则相乘。

这条式子是开环版本。闭环 MPC 更常见：每个时刻都重新求解，控制律是

$$
u_t^\star=\pi_{\text{MPC}}(x_t;\theta).
$$

此时状态灵敏度要按递推计算：

$$
\frac{dx_{t+1}}{d\theta}
=
\left(F_x+F_u\pi_x\right)\frac{dx_t}{d\theta}
+F_u\pi_\theta,
$$

其中 $F_x,F_u$ 来自仿真/动力学模型，$\pi_x,\pi_\theta$ 来自 MPC 解对状态和参数的灵敏度。缺少 $\pi_x$ 时，只能解释“固定一条开环控制序列”的梯度，不能解释闭环 MPC 策略的梯度。

### S5.2.3 可微分仿真作为 MPC 内部模型

第二种接口是把可微分仿真步直接放进 MPC 的动力学约束。

$$
x_{k+1}=\operatorname{mjx\_step}(x_k,u_k;p).
$$

这看起来很诱人，因为模型与仿真完全一致。

但实时 MPC 往往不会这样做。

原因有三点。

| 原因 | 具体影响 |
|------|----------|
| 维度太高 | 仿真状态包含完整广义坐标、速度、接触求解器状态，SQP 线性化成本高 |
| 非光滑太多 | 接触切换、摩擦锥、求解器迭代都可能破坏局部二次模型 |
| 实时性不足 | MPC 需要每个节点多次评估动力学和导数，高保真仿真代价太高 |

因此，工程上更常见的是：

```text
MPC 内部：简化模型 / 学习残差
外部评估：高保真可微仿真
真实部署：WBC + 传感器反馈兜底
```

这与足式/70_腿足简化模型理论中的分层思想一致。

回顾该章：LIPM、SRBD、Centroidal 不是为了替代全身动力学，而是为了在 MPC 层保留最关键的低维物理。

本章复用同一个思想，只是把“低维物理模型”放进可微学习循环。

### S5.2.4 双模型结构：优化模型与评估模型

一个成熟系统通常同时维护两个模型。

| 模型 | 作用 | 典型要求 |
|------|------|----------|
| 优化模型 | MPC 内部预测，生成控制 | 低维、光滑、导数稳定、可实时 |
| 评估模型 | 可微仿真或真实系统，计算外层损失 | 高保真、覆盖真实误差、可验证 |

两者不一致不是缺陷。

它是实时控制的基本取舍。

如果优化模型过于高保真，MPC 算不动。

如果评估模型过于简化，学到的参数会过拟合简化误差。

可微 MPC 的目标不是消除这两个模型之间的差异。

它的目标是让这个差异可以通过数据、梯度和约束被系统性校正。

### S5.2.5 接口代码：简化 MPC + 可微仿真评估

下面用 JAX 风格伪代码展示接口。

代码重点是梯度路径。

```python
import jax
import jax.numpy as jnp


def srbd_mpc_solve(x0, theta):
    """简化模型 MPC。

    theta 可以包含 Q/R 权重、摩擦估计、残差模型参数。
    真正实现中会调用 iLQR/SQP，并在 backward 使用隐式求导。
    """
    u_seq = differentiable_mpc_layer(x0, theta)
    return u_seq


def mjx_rollout(sim_model, x0, u_seq, physical_params):
    """用可微分仿真评估 MPC 输出。

    这里假设 mjx_step 对状态、控制和物理参数可微。
    """
    xs = [x0]
    x = x0
    for u in u_seq:
        # 中文注释：仿真器提供更高保真的状态转移。
        # 这一步的梯度描述“控制变化会怎样改变真实运动”。
        x = mjx_step(sim_model, x, u, physical_params)
        xs.append(x)
    return jnp.stack(xs)


def outer_loss(theta, x0, target, sim_model, physical_params):
    u_seq = srbd_mpc_solve(x0, theta)
    xs = mjx_rollout(sim_model, x0, u_seq, physical_params)

    # 中文注释：外层任务可以是仿真中的轨迹误差，
    # 不必等于 MPC 内部的二次代价。
    tracking = jnp.mean(jnp.sum((xs[:, :3] - target[:, :3]) ** 2, axis=-1))
    smoothness = 1e-3 * jnp.mean(jnp.sum(u_seq ** 2, axis=-1))
    return tracking + smoothness


# 对 theta 求梯度。
# 如果 differentiable_mpc_layer 的 backward 是隐式求导，
# 这里就同时利用了 MPC 梯度和仿真梯度。
grad_theta = jax.grad(outer_loss)
```

这个结构可以做四类实验。

第一，固定 MPC，学习仿真物理参数。

第二，固定仿真，学习 MPC 代价权重。

第三，同时学习残差模型和权重。

第四，把外层 loss 替换为 critic 估计，形成 actor-critic MPC。

### S5.2.6 接触梯度与 MPC 约束的边界

S4 已经说明：接触梯度不是免费午餐。

这里要补上 MPC 视角。

MPC 中的接触常通过约束表达。

可微仿真中的接触常通过状态转移表达。

两者对“脚是否接触地面”的处理方式不同。

| 位置 | 接触表示 | 优点 | 风险 |
|------|----------|------|------|
| MPC 约束 | 接触模式、摩擦锥、法向力非负 | 可解释，可做硬约束 | 模式给错时预测错误 |
| 可微仿真 | 接触求解器、软约束、冲量/力 | 高保真，能模拟意外接触 | 梯度在切换处偏差大 |

如果把可微仿真梯度直接用于学习接触丰富的步态，梯度可能鼓励策略利用软接触穿透。

如果 MPC 接触模式过于刚硬，实际落脚延迟会造成不可行。

折中策略通常包括：

| 策略 | 作用 |
|------|------|
| 过渡模式 | 在接触建立/释放时平滑约束 |
| barrier smoothing | 让活动集切换附近的灵敏度有界 |
| contact-invariant loss | 外层 loss 不过度依赖单个接触事件时间 |
| gradient clipping | 防止接触边界产生巨大更新 |
| finite-difference check | 用数值差分抽查解析梯度方向 |

> **本质洞察**：MPC 约束和可微分仿真梯度不是替代关系。
> 约束负责告诉优化器哪些动作物理可行；仿真梯度负责告诉学习器参数变化怎样影响闭环结果。
> 一个守边界，一个给方向；把二者混成一个黑盒，调试就会失去抓手。

### S5.2.7 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 用高保真仿真直接做实时 NMPC 模型 | 求解时间暴涨，线性化失败 | 高维接触状态不适合作为低频优化模型 | 用简化模型做 MPC，高保真仿真做评估 |
| 认为软接触一定可微且正确 | 训练策略学会“压进地面”获取虚假反力 | 软化改变了物理问题 | 加穿透惩罚、接触力上限、真实数据校验 |
| 忽略模型双轨制 | MPC 训练好但部署失效 | 优化模型和评估模型误差未监控 | 同时记录两套模型预测误差 |
| 把接触模式固定得过死 | 一次落脚延迟导致整个预测窗口错误 | 真实接触事件有噪声和延迟 | 加过渡模式和反馈修正 mode schedule |

### S5.2.8 练习

1. **框图题**：画出“SRBD MPC + MJX 外层评估”的计算图，标出哪些边需要梯度，哪些边可以停止梯度。
2. **实验题**：在一个二维弹跳球环境中，比较硬接触、软接触和光滑 barrier 接触的梯度方向。
3. **综合题**：结合足式/70_腿足简化模型理论，说明为什么四足 trot 的 MPC 内部常用 SRBD，而不是完整 MuJoCo 仿真器。

---

## S5.3 Shooting 与 Collocation：MPC 离散化如何影响梯度 ⭐⭐⭐

这一节解决的问题是：同一个连续时间最优控制问题，用 shooting 或 collocation 离散化后，最优解和梯度有什么差别？

### S5.3.1 连续时间问题

连续时间 OCP 写成：

$$
\begin{aligned}
\min_{x(\cdot),u(\cdot)}\quad
& \int_0^T \ell(x(t),u(t),t;\theta)\,dt + \ell_f(x(T);\theta) \\
\text{s.t.}\quad
& \dot{x}(t)=f(x(t),u(t),t;\theta) \\
& c(x(t),u(t),t;\theta)\le 0 \\
& x(0)=\hat{x}.
\end{aligned}
$$

计算机不能直接求解无限维问题。

必须离散化。

离散化不只是数值细节。

它决定了优化变量、约束结构、稀疏性和梯度路径。

### S5.3.2 Single Shooting

Single shooting 只把控制序列作为决策变量。

状态由前向积分得到。

```text
决策变量：u0, u1, ..., u_{N-1}
状态生成：x1 = F(x0,u0), x2 = F(x1,u1), ...
```

优化问题：

$$
\min_{u_{0:N-1}}
\sum_{k=0}^{N-1}\ell(x_k,u_k)
+\ell_N(x_N),
\quad
x_{k+1}=F_k(x_k,u_k).
$$

Single shooting 的梯度通过整条 rollout 链传播。

$$
\frac{\partial x_N}{\partial u_0}
=
\frac{\partial F_{N-1}}{\partial x_{N-1}}
\cdots
\frac{\partial F_1}{\partial x_1}
\frac{\partial F_0}{\partial u_0}.
$$

这个式子解释了长 horizon 的梯度爆炸/消失。

如果每个 $\partial F/\partial x$ 的谱半径略大于 1，乘起来会爆炸。

如果略小于 1，乘起来会消失。

| 优点 | 缺点 |
|------|------|
| 变量少，实现简单 | 对初值敏感，长 horizon 梯度病态 |
| 动力学自动满足 | 不容易处理路径约束和不稳定系统 |
| 适合小系统、短 horizon | 接触事件会让 rollout 梯度不稳定 |

### S5.3.3 Multiple Shooting

Multiple shooting 把每个节点的状态也作为决策变量。

动力学通过缺陷约束连接：

$$
d_k(x_k,u_k,x_{k+1})
=
x_{k+1}-F_k(x_k,u_k)
=0.
$$

优化问题：

$$
\begin{aligned}
\min_{x_{0:N},u_{0:N-1}}\quad
& \sum_{k=0}^{N-1}\ell_k(x_k,u_k)+\ell_N(x_N) \\
\text{s.t.}\quad
& x_0=\hat{x} \\
& x_{k+1}-F_k(x_k,u_k)=0.
\end{aligned}
$$

Multiple shooting 的核心好处是把长链乘法变成局部约束。

状态轨迹可以被求解器整体调整。

这对不稳定系统尤其重要。

回顾足式/70 中的 LIPM：倒立摆含有 $e^{+\omega t}$ 发散模态。

如果用 single shooting 从错误控制初值开始 rollout，状态会迅速发散。

Multiple shooting 可以把中间状态作为变量，让 SQP 在每个节点局部修正。

| 优点 | 缺点 |
|------|------|
| 适合不稳定系统 | 变量更多 |
| 稀疏约束结构清晰 | 需要处理等式约束 |
| 适合 SQP/acados/OCS2 | 初始状态轨迹仍需要热启动 |

### S5.3.4 Direct Collocation

Collocation 不只要求节点间积分一致，还在区间内部放置 collocation 点。

例如梯形法：

$$
x_{k+1}-x_k
-
\frac{\Delta t}{2}
\left[
f(x_k,u_k)+f(x_{k+1},u_{k+1})
\right]
=0.
$$

这个写法使用节点控制 $u_0,\ldots,u_N$。如果你的 OCP 只把 $u_0,\ldots,u_{N-1}$ 作为决策变量，就需要改成区间常值控制、区间中点控制，或额外定义 $u_N$ 的插值/保持规则。否则最后一个区间的 $u_{k+1}$ 没有定义。

Hermite-Simpson collocation 使用区间中点：

$$
x_{k+\frac{1}{2}}
=
\frac{1}{2}(x_k+x_{k+1})
+
\frac{\Delta t}{8}
\left[
f(x_k,u_k)-f(x_{k+1},u_{k+1})
\right],
$$

并要求：

$$
x_{k+1}-x_k
-
\frac{\Delta t}{6}
\left[
f_k+4f_{k+\frac{1}{2}}+f_{k+1}
\right]
=0.
$$

Collocation 的特点是对轨迹优化更稳。

它也更适合状态约束。

但在高速实时 MPC 中，collocation 的每个区间约束更复杂。

因此嵌入式 NMPC 常使用 multiple shooting + RK 积分。

离线轨迹优化常使用 direct collocation。

### S5.3.5 梯度视角的对比

| 方法 | 决策变量 | 梯度链长度 | 稀疏性 | 适用 |
|------|----------|------------|--------|------|
| Single shooting | 控制 | 长链 | 密集 | 短 horizon、小系统、采样 MPC |
| Multiple shooting | 状态 + 控制 | 局部链 | 块稀疏 | 实时 NMPC、acados、OCS2 SQP |
| Collocation | 状态 + 控制 + 中点 | 局部 implicit 约束 | 块稀疏 | 离线优化、高精度轨迹 |

从可微 MPC 角度看，multiple shooting 和 collocation 更适合隐式求导。

因为它们天然形成 KKT 系统。

KKT 矩阵具有块稀疏结构。

伴随灵敏度可以利用这个结构。

Single shooting 也能求导。

但梯度更像普通 RNN 的反向传播。

长 horizon 时更病态。

### S5.3.6 代码示例：multiple shooting 的缺陷约束

```cpp
#include <Eigen/Dense>
#include <vector>

struct DoubleIntegratorNode {
    Eigen::Vector2d x;  // 中文注释：状态 [位置, 速度]
    double u;           // 中文注释：控制输入，加速度
};

Eigen::Vector2d dynamicsEuler(const Eigen::Vector2d& x, double u, double dt) {
    // 中文注释：最简单的欧拉离散化。
    // x_next[0] = position + dt * velocity
    // x_next[1] = velocity + dt * acceleration
    Eigen::Vector2d next;
    next(0) = x(0) + dt * x(1);
    next(1) = x(1) + dt * u;
    return next;
}

Eigen::VectorXd buildDefectConstraints(
    const std::vector<DoubleIntegratorNode>& nodes,
    double dt)
{
    const int N = static_cast<int>(nodes.size()) - 1;
    Eigen::VectorXd defect(2 * N);

    for (int k = 0; k < N; ++k) {
        // 中文注释：multiple shooting 的等式约束。
        // 如果 defect 为零，说明相邻节点满足动力学积分关系。
        Eigen::Vector2d predicted = dynamicsEuler(nodes[k].x, nodes[k].u, dt);
        Eigen::Vector2d residual = nodes[k + 1].x - predicted;
        defect.segment<2>(2 * k) = residual;
    }
    return defect;
}
```

这段代码在真实 SQP 中会配套 Jacobian。

缺陷约束的 Jacobian 只依赖相邻两个状态和当前控制。

因此矩阵呈块带状。

这就是 multiple shooting 高效的原因。

### S5.3.7 梯形 collocation 的 C++ 残差

```cpp
Eigen::Vector2d continuousDynamics(const Eigen::Vector2d& x, double u) {
    // 中文注释：双积分器连续动力学 xdot = [v, u]。
    return Eigen::Vector2d(x(1), u);
}

Eigen::Vector2d trapezoidDefect(
    const Eigen::Vector2d& xk,
    const Eigen::Vector2d& xkp1,
    double uk,
    double ukp1,
    double dt)
{
    // 中文注释：梯形法要求区间两端动力学的平均值解释状态变化。
    const Eigen::Vector2d fk = continuousDynamics(xk, uk);
    const Eigen::Vector2d fkp1 = continuousDynamics(xkp1, ukp1);
    return xkp1 - xk - 0.5 * dt * (fk + fkp1);
}
```

注意这里的残差同时依赖 $x_k$ 和 $x_{k+1}$。

这使得 collocation 的每个区间是隐式的。

隐式积分通常更稳定。

但求解和求导也更复杂。

### S5.3.8 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 以为离散化不影响学习 | 同一模型不同 dt 学出不同权重 | 离散动力学改变梯度尺度和稳定性 | 固定 dt 后再调权重，记录离散化方式 |
| single shooting 用长 horizon 接触任务 | 梯度爆炸，优化器只会缩小动作 | 状态转移 Jacobian 长链乘法病态 | 使用 multiple shooting 或短 horizon |
| collocation 用在强冲击接触 | 求解器在接触点附近振荡 | 隐式约束假设区间内动力学平滑 | 引入事件分段、过渡模式或非光滑处理 |
| 忽略缺陷约束残差 | 外层 loss 下降但动力学不满足 | 状态变量被优化器当自由变量使用 | 记录每个节点 defect norm |

### S5.3.9 练习

1. **推导题**：对双积分器写出 multiple shooting 缺陷约束的 Jacobian。
2. **编程题**：比较 Euler、RK4、梯形法在同一 LIPM 模型上的预测误差和梯度误差。
3. **思考题**：为什么 acados 这类实时 NMPC 更偏好 multiple shooting，而不是完整 collocation？

---

## S5.4 iLQR、DDP 与 SQP：求解器如何提供梯度 ⭐⭐⭐

这一节解决的问题是：不同 MPC 求解器的 forward pass 和 backward pass 到底在算什么，可微化时应复用哪些结构？

### S5.4.1 iLQR 的局部二次化

iLQR 从一条名义轨迹开始：

$$
\bar{\tau}=
\{\bar{x}_0,\bar{u}_0,\ldots,\bar{x}_N\}.
$$

在每个时刻，把动力学一阶线性化：

$$
\delta x_{k+1}
=
A_k\delta x_k+B_k\delta u_k,
$$

其中：

$$
A_k=\frac{\partial f_k}{\partial x},\quad
B_k=\frac{\partial f_k}{\partial u}.
$$

把代价二阶近似：

$$
\ell_k(x_k,u_k)
\approx
\ell_k
+
\ell_x^T\delta x
+
\ell_u^T\delta u
+
\frac{1}{2}
\begin{bmatrix}\delta x\\\delta u\end{bmatrix}^T
\begin{bmatrix}
\ell_{xx} & \ell_{xu}\\
\ell_{ux} & \ell_{uu}
\end{bmatrix}
\begin{bmatrix}\delta x\\\delta u\end{bmatrix}.
$$

然后用 Riccati 递推求局部控制律：

$$
\delta u_k = k_k + K_k\delta x_k.
$$

这里 $k_k$ 是前馈修正。

$K_k$ 是反馈增益。

前馈项推动名义控制更新。

反馈项让 rollout 偏离名义状态时仍能被拉回。

### S5.4.2 iLQR backward pass 的完整递推

递推从终端代价开始：

$$
V_x^N = \ell_{f,x}(x_N), \quad V_{xx}^N=\ell_{f,xx}(x_N).
$$

定义 value function 的二次近似：

$$
V_{k+1}(\delta x)
\approx
V_{k+1}
+
V_x^T\delta x
+
\frac{1}{2}\delta x^T V_{xx}\delta x.
$$

构造 Q 函数：

$$
Q_k(\delta x,\delta u)
=
\ell_k(x,u)
+
V_{k+1}(f(x,u)).
$$

对线性化动力学代入后，得到：

$$
Q_x = \ell_x + A^T V_x',
$$

$$
Q_u = \ell_u + B^T V_x',
$$

$$
Q_{xx} = \ell_{xx}+A^TV_{xx}'A,
$$

$$
Q_{ux} = \ell_{ux}+B^TV_{xx}'A,
$$

$$
Q_{uu} = \ell_{uu}+B^TV_{xx}'B.
$$

这里撇号表示下一时刻。

最优 $\delta u$ 满足：

$$
\frac{\partial Q}{\partial \delta u}
=
Q_u+Q_{ux}\delta x+Q_{uu}\delta u
=0.
$$

所以：

$$
k = -Q_{uu}^{-1}Q_u,
$$

$$
K = -Q_{uu}^{-1}Q_{ux}.
$$

这就是 iLQR backward pass 的核心。

有了 $k$ 和 $K$ 之后，还必须把 value function 继续向前一时刻回传：

$$
V_x =
Q_x + Q_{xu}k + K^T Q_u + K^T Q_{uu}k,
$$

$$
V_{xx} =
Q_{xx} + Q_{xu}K + K^T Q_{ux} + K^T Q_{uu}K.
$$

实现时通常会对 $V_{xx}$ 做对称化：

$$
V_{xx}\leftarrow \frac{1}{2}(V_{xx}+V_{xx}^T),
$$

避免浮点误差让 Hessian 失去对称性。没有这两条 $V_x,V_{xx}$ 更新，就只能得到当前一步的增益，不能从 $N-1$ 一直递推到 $0$。

如果 $Q_{uu}$ 不正定，需要加正则：

$$
Q_{uu}^{reg}=Q_{uu}+\lambda I.
$$

正则不是数值小技巧。

它控制了局部二次模型的可信程度。

当线性化不可靠时，增大 $\lambda$ 让更新更保守。

### S5.4.3 DDP 与 iLQR 的差别

DDP 比 iLQR 更完整。

iLQR 忽略动力学的二阶导数。

DDP 保留：

$$
f_{xx},\quad f_{xu},\quad f_{uu}.
$$

这些项进入 $Q_{xx}$、$Q_{ux}$、$Q_{uu}$。

| 方法 | 动力学展开 | 代价展开 | 计算量 | 适用 |
|------|------------|----------|--------|------|
| LQR | 线性动力学固定 | 二次代价固定 | 最低 | 线性系统 |
| iLQR | 一阶动力学 | 二阶代价 | 中 | 大多数实时 MPC |
| DDP | 二阶动力学 | 二阶代价 | 高 | 需要高精度二阶信息 |
| FDDP | DDP + 缺陷处理 | 二阶 | 高 | Crocoddyl 轨迹优化 |

DDP 的二阶信息在强非线性系统中更准确。

但在机器人实时控制中，动力学二阶导数代价很高。

很多工程框架使用 iLQR 或 Gauss-Newton 近似。

这就是 OCS2 SLQ、MJPC iLQG 和许多移动机器人 MPC 的选择。

### S5.4.4 约束如何进入 iLQR

无约束 iLQR 只处理动力学约束。

真实 MPC 还需要控制限幅、状态限幅、摩擦锥和避碰。

约束处理有几种方式。

| 方法 | 思路 | 优点 | 缺点 |
|------|------|------|------|
| clamp | 把控制投影到上下界 | 简单快速 | 梯度在边界处不光滑 |
| barrier | 把约束写入代价 | 光滑，可用 iLQR | 不能严格保证可行 |
| augmented Lagrangian | 罚函数 + 乘子 | 约束处理更强 | 参数调节复杂 |
| SQP 子问题 | 每步解带约束 QP | 约束准确 | 计算量更高 |

Amos 2018 的 differentiable MPC 处理 box-constrained iLQR。

它的核心是把控制上下界的活动集也放入 KKT 条件。

这让 backward pass 仍然类似一次 LQR，但需要考虑哪些控制维度被夹住。

### S5.4.5 SQP 的局部 QP

SQP 从一般非线性规划开始：

$$
\min_z J(z;\theta)
\quad
\text{s.t.}
\quad
c(z;\theta)=0,\quad
g(z;\theta)\le 0.
$$

在当前迭代点 $z^i$ 处，构造 QP：

$$
\begin{aligned}
\min_{\Delta z}\quad
& \frac{1}{2}\Delta z^T H_i \Delta z + \nabla J_i^T\Delta z \\
\text{s.t.}\quad
& c_i + C_i\Delta z = 0 \\
& g_i + G_i\Delta z \le 0.
\end{aligned}
$$

其中：

$$
C_i=\frac{\partial c}{\partial z}(z^i),\quad
G_i=\frac{\partial g}{\partial z}(z^i).
$$

$H_i$ 可以是拉格朗日函数 Hessian。

也可以是 Gauss-Newton 近似。

也可以是 BFGS 更新。

实时 MPC 常用 SQP-RTI。

RTI 指每个控制周期只做一次 SQP 迭代。

它依赖热启动和高频滚动。

如果上一周期解接近当前最优解，一次迭代足够。

如果突然换模式或目标跳变，一次迭代可能不够。

### S5.4.6 SQP 与 iLQR 的关系

iLQR 可以看成特殊结构的 SQP。

它利用动力学约束的时序结构，把 QP 通过 Riccati 递推高效求解。

一般 SQP 则把问题交给 QP 求解器。

| 维度 | iLQR/DDP | SQP |
|------|----------|-----|
| 动力学约束 | 通过 rollout 和 Riccati 内隐处理 | 显式线性化等式约束 |
| 不等式约束 | 需要特殊处理 | QP 中自然表达 |
| 稀疏结构 | 时序结构强 | 多种稀疏结构 |
| 求解速度 | 无约束时非常快 | 约束多时更稳 |
| 可微化 | Riccati backward 或 KKT | KKT/IPM/active set |

这解释了工具选择。

Crocoddyl 偏 DDP。

OCS2 支持 SLQ/SQP。

acados 偏 SQP-RTI + HPIPM。

mpc.pytorch 是 box-constrained iLQR 的教学经典。

### S5.4.7 求解器结构与 backward 的复用

可微 MPC 最重要的工程事实是：forward 求解器中的分解可以在 backward 中复用。

iLQR forward 已经计算了 $Q_{uu}$ 分解。

SQP/IPM forward 已经因子化了 KKT 矩阵。

隐式 backward 需要求解的线性系统与 forward 末端的 KKT 系统同构。

因此 backward 不应该重新从零构造一个密集逆矩阵。

应该求解伴随线性系统。

如果外层只需要 $\nabla_\theta\mathcal{L}$，不需要完整 Jacobian：

$$
\frac{\partial z^\star}{\partial\theta}
$$

那么伴随法更高效。

它先解：

$$
KKT_z^T \lambda_{\text{adj}}
=
\left(\frac{\partial \mathcal{L}}{\partial z^\star}\right)^T,
$$

再计算：

$$
\frac{d\mathcal{L}}{d\theta}
=
-
\lambda_{\text{adj}}^T
KKT_\theta
+
\mathcal{L}_\theta.
$$

若外层损失不直接依赖 $\theta$，$\mathcal{L}_\theta$ 才可以省略。很多学习型 MPC 会把正则、先验或参数边界直接写进外层 loss，此时这个直接项必须保留。

这比显式形成巨大 Jacobian 快得多。

### S5.4.8 C++ 示例：Riccati backward 的核心形状

```cpp
#include <Eigen/Dense>
#include <vector>

struct ILQRBackwardStep {
    Eigen::VectorXd k;   // 中文注释：前馈控制修正
    Eigen::MatrixXd K;   // 中文注释：反馈增益，处理状态偏差
};

ILQRBackwardStep computeBackwardStep(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::VectorXd& lx,
    const Eigen::VectorXd& lu,
    const Eigen::MatrixXd& lxx,
    const Eigen::MatrixXd& luu,
    const Eigen::MatrixXd& lux,
    const Eigen::VectorXd& Vx_next,
    const Eigen::MatrixXd& Vxx_next,
    double regularization)
{
    // 中文注释：构造 Q 函数的一阶与二阶项。
    Eigen::VectorXd Qx = lx + A.transpose() * Vx_next;
    Eigen::VectorXd Qu = lu + B.transpose() * Vx_next;
    Eigen::MatrixXd Qxx = lxx + A.transpose() * Vxx_next * A;
    Eigen::MatrixXd Qux = lux + B.transpose() * Vxx_next * A;
    Eigen::MatrixXd Quu = luu + B.transpose() * Vxx_next * B;

    // 中文注释：正则化保证 Quu 可逆且更新不过激。
    Quu += regularization * Eigen::MatrixXd::Identity(Quu.rows(), Quu.cols());

    // 中文注释：使用 LDLT 分解，不显式求逆。
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Quu);
    ILQRBackwardStep out;
    out.k = -ldlt.solve(Qu);
    out.K = -ldlt.solve(Qux);
    return out;
}
```

这段代码体现一个重要原则。

数学上写 $Q_{uu}^{-1}$。

工程上永远不要显式求逆。

使用 Cholesky、LDLT 或稀疏 KKT 求解。

### S5.4.9 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 把 DDP backward 等同于神经网络 backward | 对控制约束和活动集完全忽略 | DDP backward 是动态规划，不是普通链式反传 | 分清 Riccati 递推和 autograd 反传 |
| $Q_{uu}$ 非正定仍强行更新 | rollout 代价上升，控制发散 | 局部二次模型不可信 | 加正则、线搜索、信赖域 |
| SQP-RTI 在目标跳变时仍只做一次迭代 | MPC 输出大幅滞后 | 热启动失效 | 检测残差，临时增加迭代或重置初值 |
| backward 显式求大 Jacobian | 显存/内存爆炸 | 不需要完整灵敏度矩阵 | 用伴随法求向量-Jacobian 乘积 |

### S5.4.10 练习

1. **推导题**：从 $Q_u+Q_{ux}\delta x+Q_{uu}\delta u=0$ 推导 $k$ 和 $K$。
2. **编程题**：实现双积分器 iLQR，记录 $Q_{uu}$ 的最小特征值和正则化次数。
3. **对比题**：解释为什么摩擦锥约束较多时，SQP 通常比纯 iLQR 更自然。

---

## S5.5 KKT 条件与隐式函数定理 ⭐⭐⭐⭐

这一节是本章数学核心：梯度如何穿过优化器。

### S5.5.1 从无约束最优化开始

先看最简单的问题：

$$
z^\star(\theta)
=
\arg\min_z
\phi(z,\theta).
$$

最优性条件是：

$$
\nabla_z\phi(z^\star,\theta)=0.
$$

定义：

$$
F(z,\theta)=\nabla_z\phi(z,\theta).
$$

于是：

$$
F(z^\star(\theta),\theta)=0.
$$

对 $\theta$ 求导：

$$
\frac{\partial F}{\partial z}
\frac{\partial z^\star}{\partial\theta}
+
\frac{\partial F}{\partial\theta}
=0.
$$

如果 $\frac{\partial F}{\partial z}$ 可逆，则：

$$
\boxed{
\frac{\partial z^\star}{\partial\theta}
=
-
\left(
\frac{\partial F}{\partial z}
\right)^{-1}
\frac{\partial F}{\partial\theta}
}
$$

因为 $F=\nabla_z\phi$，所以：

$$
\frac{\partial F}{\partial z}
=
\nabla_{zz}^2\phi.
$$

这就是 Hessian。

无约束问题的隐式求导就是解一个 Hessian 线性系统。

### S5.5.2 一个手算例子

令：

$$
\phi(z,\theta)=\frac{1}{2}az^2+\theta z,
\quad a>0.
$$

最优性条件：

$$
az^\star+\theta=0.
$$

所以：

$$
z^\star=-\frac{\theta}{a}.
$$

显式求导：

$$
\frac{dz^\star}{d\theta}=-\frac{1}{a}.
$$

隐式求导：

$$
F(z,\theta)=az+\theta.
$$

$$
F_z=a,\quad F_\theta=1.
$$

所以：

$$
\frac{dz^\star}{d\theta}
=
-F_z^{-1}F_\theta
=
-\frac{1}{a}.
$$

两者一致。

这个例子看似简单，但已经包含所有核心结构。

复杂 MPC 只是把 $z$ 换成整条轨迹，把 $F$ 换成 KKT 条件。

### S5.5.3 带等式约束的 KKT

考虑：

$$
\min_z \phi(z,\theta)
\quad
\text{s.t.}
\quad
c(z,\theta)=0.
$$

拉格朗日函数：

$$
\mathcal{L}(z,\lambda,\theta)
=
\phi(z,\theta)
+
\lambda^Tc(z,\theta).
$$

KKT 条件：

$$
\nabla_z\mathcal{L}(z^\star,\lambda^\star,\theta)=0,
$$

$$
c(z^\star,\theta)=0.
$$

把变量合并：

$$
y=
\begin{bmatrix}
z\\
\lambda
\end{bmatrix}.
$$

定义：

$$
F(y,\theta)
=
\begin{bmatrix}
\nabla_z\mathcal{L}(z,\lambda,\theta)\\
c(z,\theta)
\end{bmatrix}
=0.
$$

对 $y$ 的 Jacobian 是 KKT 矩阵：

$$
F_y
=
\begin{bmatrix}
\nabla_{zz}^2\mathcal{L} & C^T\\
C & 0
\end{bmatrix}.
$$

其中：

$$
C=\frac{\partial c}{\partial z}.
$$

隐式求导：

$$
\frac{\partial y^\star}{\partial\theta}
=
-
\begin{bmatrix}
\nabla_{zz}^2\mathcal{L} & C^T\\
C & 0
\end{bmatrix}^{-1}
\frac{\partial F}{\partial\theta}.
$$

### S5.5.4 带不等式约束的 KKT

考虑：

$$
g(z,\theta)\le 0.
$$

拉格朗日函数：

$$
\mathcal{L}
=
\phi(z,\theta)
+
\lambda^Tc(z,\theta)
+
\mu^Tg(z,\theta).
$$

KKT 条件：

$$
\nabla_z\mathcal{L}=0.
$$

$$
c(z,\theta)=0.
$$

$$
g(z,\theta)\le 0.
$$

$$
\mu\ge 0.
$$

$$
\mu_i g_i(z,\theta)=0,\quad \forall i.
$$

最后一条是互补松弛。

它制造了非光滑性。

如果一个约束从不活跃变成活跃，$\mu_i$ 从 0 变成正数。

最优解对参数的导数可能跳变。

这就是可微 MPC 的困难之一。

### S5.5.5 活动集视角

如果已知活动集 $\mathcal{A}$：

$$
\mathcal{A}=\{i\mid g_i(z^\star,\theta)=0\},
$$

可以把活动不等式当等式处理。

非活动约束不进入 KKT 线性系统。

在活动集不变、约束 Jacobian 满秩（LICQ）、二阶充分条件成立，并且 KKT 系统非退化的小邻域内，最优解才是局部光滑的。若还希望活动/非活动边界稳定，通常还需要严格互补这类条件。

活动集变化时，导数不连续。

这解释了为什么 box-constrained iLQR 可以高效求导。

控制限幅的活动集相对容易判断。

一般接触约束、自碰撞约束和摩擦锥活动集更复杂。

### S5.5.6 IPM 平滑视角

内点法把不等式约束转成 barrier：

$$
\phi_\tau(z,\theta)
=
\phi(z,\theta)
-
\tau\sum_i \log(-g_i(z,\theta)).
$$

$\tau>0$ 时，最优解留在约束内部。

梯度是光滑的。

$\tau\to 0$ 时，解接近原始约束问题。

但灵敏度也可能变得不连续。

这就是 Frey 等可微 NMPC 中 barrier smoothing 的直觉。

较大的 $\tau$ 给出更平滑的梯度。

较小的 $\tau$ 给出更精确但更尖锐的梯度。

| $\tau$ | 梯度 | 解的精度 | 训练表现 |
|--------|------|----------|----------|
| 大 | 平滑 | 离真实活动集更远 | 稳定但可能有偏 |
| 中 | 折中 | 可接受 | 常用 |
| 小 | 接近真实灵敏度 | 高 | 活动集切换处容易抖 |
| 0 | 非光滑 | 原问题 | 需要活动集处理 |

### S5.5.7 伴随法推导

外层损失依赖最优解：

$$
\mathcal{J}(\theta)=r(y^\star(\theta),\theta).
$$

直接求：

$$
\frac{d\mathcal{J}}{d\theta}
=
r_y
\frac{\partial y^\star}{\partial\theta}
+
r_\theta.
$$

代入隐式求导：

$$
\frac{d\mathcal{J}}{d\theta}
=
-
r_y F_y^{-1}F_\theta
+
r_\theta.
$$

不要显式计算 $F_y^{-1}$。

定义伴随变量 $a$：

$$
F_y^Ta=r_y^T.
$$

则：

$$
\boxed{
\frac{d\mathcal{J}}{d\theta}
=
-
a^TF_\theta
+
r_\theta
}
$$

这就是伴随灵敏度。这正是 Pontryagin 极大值原理（1962）的离散版本——伴随变量 $a$ 的递推方程与经典的协态方程完全对应。在连续时间最优控制中，协态变量 $\lambda(t)$ 满足 $\dot{\lambda}=-H_x^T$，其中 $H$ 是 Hamilton 函数；这里的离散伴随方程 $F_y^T a = r_y^T$ 是同一思想在 KKT 条件上的体现。

伴随法只需要解一次转置线性系统。

当 $\theta$ 维度很高时，这比前向灵敏度更划算。

### S5.5.8 KKT 矩阵为什么可能奇异

隐式函数定理需要 $F_y$ 可逆。

实际中这不总成立。

常见原因如下。

| 原因 | 现象 | 处理 |
|------|------|------|
| Hessian 半正定但不正定 | KKT 求解不稳定 | 正则化、Gauss-Newton、信赖域 |
| 约束 Jacobian 不满秩 | 乘子不唯一 | 删除冗余约束、SVD 检查 |
| 活动集退化 | 梯度跳变巨大 | barrier smoothing、活动集锁定 |
| 最优解不唯一 | Jacobian 不存在或集合值 | 加小正则，定义选择规则 |
| 求解器未收敛 | KKT 残差大 | 增加迭代或停止外层更新 |

这也是为什么“梯度能算出来”不等于“梯度可信”。

KKT 残差和条件数必须进入日志。

### S5.5.9 小型 QP 的隐式求导代码

下面用一个等式约束 QP 展示 KKT 求导。

问题：

$$
\min_z
\frac{1}{2}z^THz+q(\theta)^Tz,
\quad
Az=b.
$$

KKT：

$$
\begin{bmatrix}
H & A^T\\
A & 0
\end{bmatrix}
\begin{bmatrix}
z\\
\lambda
\end{bmatrix}
=
\begin{bmatrix}
-q\\
b
\end{bmatrix}.
$$

若 $q(\theta)=M\theta$，则：

$$
\frac{\partial}{\partial\theta}
\begin{bmatrix}
z\\
\lambda
\end{bmatrix}
=
-
K^{-1}
\begin{bmatrix}
M\\
0
\end{bmatrix}.
$$

```python
import torch


def equality_qp_solution(H, A, q, b):
    """求解等式约束 QP。

    中文注释：这里直接构造 KKT 系统用于教学。
    大规模 MPC 不应使用 dense solve，而应利用块稀疏结构。
    """
    n = H.shape[0]
    m = A.shape[0]
    KKT = torch.zeros(n + m, n + m, dtype=H.dtype, device=H.device)
    KKT[:n, :n] = H
    KKT[:n, n:] = A.T
    KKT[n:, :n] = A

    rhs = torch.cat([-q, b], dim=0)
    sol = torch.linalg.solve(KKT, rhs)
    return sol[:n], sol[n:], KKT


def implicit_grad_q(H, A, q, b, grad_z):
    """计算外层损失对 q 的梯度。

    若 loss 对 z 的梯度为 grad_z，
    则先解 KKT^T * adj = [grad_z, 0]，
    再由 stationarity 方程得到 dloss/dq = -adj_z。
    """
    z, lam, KKT = equality_qp_solution(H, A, q, b)
    n = H.shape[0]
    m = A.shape[0]

    rhs = torch.cat([grad_z, torch.zeros(m, dtype=H.dtype, device=H.device)])
    adj = torch.linalg.solve(KKT.T, rhs)
    grad_q = -adj[:n]
    return grad_q
```

这段代码故意写得直白。

真实系统会把 KKT 分解缓存起来。

如果只需要反传到 $q$，伴随法不需要构造完整 $\partial z/\partial q$。

### S5.5.10 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 忘记互补松弛的非光滑性 | 有限差分和解析梯度在边界处差异大 | 活动集变化 | 对边界点做单独测试，使用平滑或子梯度策略 |
| 对 KKT 矩阵直接求逆 | 小例子能跑，大问题崩溃 | 逆矩阵数值不稳定且破坏稀疏 | 用线性求解和分解复用 |
| 忽略乘子含义 | 学到的参数让约束长期贴边 | 乘子记录了约束压力 | 日志中保存 active constraints 和 multipliers |
| 对不可唯一解求导 | 梯度随机跳变 | 最优解映射不是单值函数 | 加正则或定义稳定的 tie-breaker |

### S5.5.11 练习

1. **手推题**：对等式约束 QP 推导 KKT 矩阵，并写出对 $q$ 的灵敏度。
2. **数值题**：用有限差分验证上面代码的 `implicit_grad_q`。
3. **研究题**：构造一个一维不等式问题，观察最优解在活动集切换处的导数跳变。

---

## S5.6 把求解器作为可微层 ⭐⭐⭐

这一节解决工程问题：一个求解器怎样变成深度学习框架中的 layer？

### S5.6.1 可微层的 forward

可微 MPC layer 的 forward 通常做四件事。

| 步骤 | 内容 | 日志信号 |
|------|------|----------|
| 参数解包 | 把神经网络输出转成 OCP 参数 | 参数范围、正定性 |
| 求解 OCP | iLQR/SQP/IPM 求出 $z^\star$ | 迭代数、KKT 残差、求解时间 |
| 输出选择 | 返回 $u_0^\star$ 或整条轨迹 | 控制限幅、约束裕度 |
| 保存 backward 上下文 | 保存分解、轨迹、乘子、活动集 | 条件数、active set |

如果 forward 没有收敛，backward 的数学前提就不成立。

因此 layer 应该把求解状态返回给训练循环。

训练代码不能只看 loss。

它也要看 solver health。

### S5.6.2 可微层的 backward

Backward 有三种路线。

| 路线 | 做法 | 适用 |
|------|------|------|
| unroll | 展开求解器迭代并让 AD 反传 | 小问题、固定迭代、教学 |
| implicit | 对 KKT/固定点条件求导 | 收敛求解器、生产可微层 |
| straight-through / custom | forward 用投影或离散操作，backward 用近似梯度 | 非光滑但需要训练信号 |

对 MPC 来说，implicit 是主线。

但在早期实验中，unroll 仍有价值。

它更容易定位某一步导数是否写错。

一旦进入高维机器人问题，unroll 的内存和稳定性会成为瓶颈。

### S5.6.3 PyTorch 自定义 autograd 的形状

```python
import torch


class MPCFunction(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x0, theta, solver_handle):
        """前向：调用外部 MPC 求解器。

        中文注释：solver_handle 可以封装 C++/acados/自研 iLQR。
        forward 返回最优第一步控制和必要的轨迹信息。
        """
        with torch.no_grad():
            result = solver_handle.solve(x0, theta)
            u0 = result.u0

        # 中文注释：保存 backward 所需信息。
        # 真实实现应避免保存巨大对象，可保存指针、压缩轨迹或 KKT 分解句柄。
        ctx.solver_handle = solver_handle
        ctx.result = result
        ctx.save_for_backward(x0, theta)
        return u0

    @staticmethod
    def backward(ctx, grad_u0):
        """反向：用隐式求导计算 VJP。

        grad_u0 是外层 loss 对 u0* 的梯度。
        返回 loss 对 x0 和 theta 的梯度。
        """
        x0, theta = ctx.saved_tensors
        result = ctx.result
        solver = ctx.solver_handle

        # 中文注释：调用求解器提供的伴随灵敏度接口。
        grad_x0, grad_theta = solver.adjoint_sensitivity(
            result=result,
            grad_output=grad_u0,
        )

        # solver_handle 不是 Tensor，没有梯度。
        return grad_x0, grad_theta, None


class MPCLayer(torch.nn.Module):
    def __init__(self, solver_handle):
        super().__init__()
        self.solver_handle = solver_handle

    def forward(self, x0, theta):
        return MPCFunction.apply(x0, theta, self.solver_handle)
```

这段代码的重点是 `adjoint_sensitivity`。

真正的可微 MPC 工作量在这里。

它需要知道求解器最终的 KKT 系统。

它需要知道外层梯度对应的是第一步控制、整条控制轨迹还是状态轨迹。

它还需要把梯度映射回 $\theta$ 的参数化方式。

### S5.6.4 acados 与 leap-c 的角色

acados 的核心价值在于快速 NMPC 求解。

它使用 CasADi 建模、C 代码生成、SQP-RTI、HPIPM/BLASFEO 这类高性能线性代数后端。

可微 NMPC 关注的是 solution sensitivity。

leap-c 的价值在于把这种能力包装进 PyTorch 训练循环。

工程上可理解为：

```text
CasADi OCP 定义
  -> acados 生成求解器
  -> NMPC forward 求 z*
  -> sensitivity/adjoint backward 求梯度
  -> PyTorch optimizer 更新 theta
```

这类框架最适合：

| 场景 | 理由 |
|------|------|
| 自动驾驶/无人机 | 低维、约束明确、模型较准确 |
| 系统辨识 | 参数物理意义清楚，数据可对齐 |
| 学 MPC 权重 | 仍保留求解器和约束 |
| 安全关键策略改进 | 更新的是控制器参数，不是直接动作 |

它不一定适合 1kHz 全身控制内环。

因为在线仍需求解 MPC。

对腿足 1kHz 关节层，常用做法是 MPC 低频、WBC/PD 高频，或把 MPC 蒸馏成网络。

### S5.6.5 Theseus 的启发

Theseus 把非线性最小二乘问题作为可微层。

虽然它不是专门的 MPC 工具，但它提供了重要思想：

$$
\min_x \sum_i \|r_i(x,\theta)\|^2.
$$

Gauss-Newton 的 normal equation：

$$
(J^TJ)\Delta x=-J^Tr.
$$

在最优点附近，隐式求导同样通过线性系统完成。

SLAM 因子图、位姿图优化、接触位姿估计、短 horizon 轨迹优化都可以共享这套模式。

这说明可微优化层是一个更大的范畴。

可微 MPC 是其中带时序动力学约束的一支。

### S5.6.6 输出整条轨迹还是只输出第一步

MPC 部署只执行第一步。

但学习时输出可以有多种选择。

| 输出 | 外层 loss | 适用 |
|------|----------|------|
| $u_0^\star$ | 动作模仿、即时 reward | 部署一致 |
| $u_{0:N-1}^\star$ | 控制序列模仿、平滑性 | 训练信号更丰富 |
| $x_{0:N}^\star$ | 轨迹形状、终端任务 | 规划层学习 |
| 乘子/裕度 | 安全学习、约束压力 | 调参和诊断 |

只输出第一步的梯度较少。

输出整条轨迹的梯度更丰富。

但外层可能会优化未执行的未来控制。

实践中常组合：

$$
\mathcal{L}
=
\mathcal{L}_{u_0}
+
\alpha\mathcal{L}_{traj}
+
\beta\mathcal{L}_{margin}.
$$

### S5.6.7 训练循环中的求解器健康监控

可微层训练必须记录求解器状态。

| 信号 | 异常含义 |
|------|----------|
| KKT residual | 隐式梯度前提不满足 |
| primal infeasibility | 约束或初值不可行 |
| dual infeasibility | 乘子或 Hessian 结构异常 |
| line search failures | 局部模型不可信 |
| active set changes | 梯度可能跳变 |
| barrier parameter | 梯度平滑程度变化 |
| solve time p99 | 部署实时性风险 |

如果训练时 loss 下降但 KKT 残差上升，说明网络在把求解器推向难问题。

这不是进步。

这通常是外层损失缺少可行性正则。

### S5.6.8 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| forward 失败还继续 backward | 梯度出现 NaN 或巨大尖峰 | 最优性条件不成立 | 对失败样本跳过更新或使用保守 fallback |
| 保存完整求解器对象到 autograd ctx | 显存和内存泄漏 | 上下文生命周期被计算图持有 | 只保存必要张量或外部句柄 |
| 外层网络输出任意参考轨迹 | MPC 经常不可行 | 参考不满足动力学和约束 | 对参考生成器加可达域和速度限制 |
| 只监控训练 loss | 部署时超时或违反约束 | 求解器健康被忽略 | 把 residual、solve time、slack 写入训练日志 |

### S5.6.9 练习

1. **接口题**：设计一个 `MPCLayer` 的 forward 返回值，要求同时支持训练和部署。
2. **工程题**：列出可微 MPC 训练日志中必须记录的 10 个字段，并解释每个字段的用途。
3. **对比题**：说明 Theseus 的可微最小二乘层与可微 MPC 层的共同点和差别。

---

## S5.7 学习代价、模型参数与残差动力学 ⭐⭐⭐

这一节解决的问题是：可微 MPC 到底学什么，哪些参数适合学，哪些参数不适合交给神经网络？

### S5.7.1 学习代价权重

经典 MPC 代价常写成：

$$
\ell(x,u)
=
\frac{1}{2}
(x-x^{ref})^TQ(x-x^{ref})
+
\frac{1}{2}
(u-u^{ref})^TR(u-u^{ref}).
$$

人工调 $Q/R$ 很困难。

可微 MPC 可以让外层数据调权重。

但必须保持权重结构。

最简单的参数化：

$$
Q=\operatorname{diag}(\operatorname{softplus}(q_\theta)+\epsilon).
$$

更一般的正定矩阵：

$$
Q=LL^T+\epsilon I,
$$

其中 $L$ 是下三角矩阵。

| 参数化 | 优点 | 缺点 |
|--------|------|------|
| 对角 softplus | 稳定、易解释 | 不能表达交叉耦合 |
| Cholesky | 可表达完整 SPD | 参数更多，易过拟合 |
| 分组权重 | 对应物理任务 | 表达能力中等 |
| 状态相关权重 | 自适应能力强 | 稳定性分析更难 |

### S5.7.2 权重学习的物理解释

在复合/30_多模态MPC 中，末端跟踪权重不能压过 locomotion 稳定权重。

本章从学习角度重新看这个原则。

如果让网络自由学习所有权重，它可能为了降低短期末端误差，把 CoM 稳定项降到很小。

仿真中也许能过。

实机上会失去支撑裕度。

因此权重学习需要先验边界。

| 权重 | 可学习范围建议 | 理由 |
|------|----------------|------|
| CoM / 基座姿态 | 下界较高 | 跌倒是硬失败 |
| 末端位置 | 可随任务调节 | 操作精度任务相关 |
| 控制正则 | 保持正下界 | 避免过激动作 |
| slack 惩罚 | 高且有限 | 允许恢复但不掩盖失败 |
| 碰撞 barrier | 下界高 | 安全边界不能被学没 |

这就是“结构化学习”的含义。

学习器可以调节控制器。

但不能删除控制器的安全骨架。

### S5.7.3 学习参考轨迹和终端代价

有时不应该直接学 $Q/R$。

更合适的是学参考或终端价值。

例如无人机竞速中，MPC 的动力学和约束很清楚。

难的是如何在弯道前选择速度和姿态。

可以让网络输出：

$$
x^{ref}_{0:N}=\rho_\theta(obs,goal).
$$

MPC 负责在约束内跟踪。

另一种方式是学习终端价值：

$$
\ell_f(x_N;\theta)=V_\theta(x_N).
$$

这能补偿有限 horizon 的短视。

AC4MPC 和 actor-critic MPC 类方法都利用了类似思想。

终端价值的优点是保留 MPC 的局部约束，同时让 critic 表达长期收益。

风险是 critic 外推错误会把 MPC 推向危险状态。

因此终端价值也需要 trust region 或保守下界。

### S5.7.4 学习物理参数

系统辨识是可微 MPC 的自然应用。

参数 $p$ 可以是：

| 参数 | 例子 | 可辨识性风险 |
|------|------|--------------|
| 质量/惯量 | 载荷变化、人形手持物体 | 与控制增益耦合 |
| 摩擦 | 地面摩擦、关节摩擦 | 接触数据不足时不可辨 |
| 阻尼 | 关节阻尼、空气阻力 | 与速度范围有关 |
| 延迟 | 电机延迟、通信延迟 | 离散时间建模敏感 |
| 接触参数 | solref/solimp、刚度 | 与地面材料和步长有关 |

轨迹误差 loss：

$$
\mathcal{L}(p)
=
\sum_{t=0}^{T}
\|x_t^{pred}(p)-x_t^{real}\|_{\Sigma^{-1}}^2.
$$

如果 $x_t^{pred}$ 由 MPC 闭环产生，则：

$$
p \to u_t^\star(p) \to x_{t+1}^{pred}(p).
$$

这时参数既影响模型预测，也影响控制动作。

可微 MPC 能捕捉这种闭环敏感性。

普通开环辨识捕捉不到。

### S5.7.5 学习残差动力学

残差模型写成：

$$
x_{k+1}
=
f_{\text{nominal}}(x_k,u_k)
+
\Delta_\phi(x_k,u_k).
$$

为了安全，残差不应无限制。

常用约束包括：

| 约束 | 作用 |
|------|------|
| 残差幅值限制 | 防止网络推翻物理模型 |
| Lipschitz 正则 | 防止局部剧烈变化 |
| 能量一致性 | 防止凭空注入能量 |
| 坐标不变性 | 保持 SE(3) / 接触坐标结构 |
| 不确定性输出 | 分布外时降低信任 |

残差模型尤其适合无人机空气阻力、轮地滑移、关节摩擦、软接触偏差。

不适合让它学习所有物理。

如果残差比主模型还大，说明名义模型选错了。

### S5.7.6 代码示例：安全的权重参数化

```python
import torch
from torch import nn


class PositiveDiagonalWeights(nn.Module):
    """把网络输出映射为有上下界的正权重。"""
    def __init__(self, dim, min_weight, max_weight):
        super().__init__()
        self.raw = nn.Parameter(torch.zeros(dim))
        self.register_buffer("min_weight", torch.full((dim,), min_weight))
        self.register_buffer("max_weight", torch.full((dim,), max_weight))

    def forward(self):
        # 中文注释：sigmoid 保证权重在 [min, max] 内。
        # 比无上界 softplus 更适合安全关键权重。
        alpha = torch.sigmoid(self.raw)
        w = self.min_weight + alpha * (self.max_weight - self.min_weight)
        return torch.diag(w)


class StructuredMPCWeights(nn.Module):
    """按物理分组学习 MPC 权重。"""
    def __init__(self):
        super().__init__()
        # 中文注释：稳定项设置高下界，不能被学习器降为零。
        self.base_Q = PositiveDiagonalWeights(dim=6, min_weight=10.0, max_weight=1000.0)
        # 中文注释：末端任务允许更大范围，适应不同操作阶段。
        self.ee_Q = PositiveDiagonalWeights(dim=6, min_weight=0.1, max_weight=500.0)
        # 中文注释：控制正则保持正值，避免控制过激。
        self.R = PositiveDiagonalWeights(dim=12, min_weight=1e-4, max_weight=10.0)

    def forward(self):
        return {
            "base_Q": self.base_Q(),
            "ee_Q": self.ee_Q(),
            "R": self.R(),
        }
```

这段代码体现了一个原则。

学习不是放弃先验。

学习是在先验允许的范围内自动调参。

### S5.7.7 代码示例：残差模型的幅值限制

```python
class BoundedResidualDynamics(nn.Module):
    """物理模型 + 有界残差。

    中文注释：残差只修正模型误差，不允许完全改写动力学。
    """
    def __init__(self, state_dim, action_dim, max_residual):
        super().__init__()
        self.max_residual = max_residual
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, state_dim),
        )

    def nominal_step(self, x, u):
        # 中文注释：这里应调用解析动力学或可微仿真简化步。
        return nominal_dynamics_step(x, u)

    def forward(self, x, u):
        x_nominal = self.nominal_step(x, u)
        raw_residual = self.net(torch.cat([x, u], dim=-1))
        # 中文注释：tanh 限制残差幅值，避免网络制造不物理的状态跳变。
        residual = self.max_residual * torch.tanh(raw_residual)
        return x_nominal + residual
```

如果模型误差很大，不要简单增大 `max_residual`。

应先检查名义模型、单位、坐标系、延迟和输入定义。

### S5.7.8 学习参数的可辨识性

不是所有参数都能从数据中辨识。

如果系统没有激励某个方向，该方向的参数梯度会接近零。

例如机器人一直低速直行，很难辨识横向摩擦。

例如四旋翼只悬停，很难辨识高速阻力。

例如机械臂没有接触，很难辨识接触刚度。

可辨识性可以用 Fisher 信息近似判断：

$$
\mathcal{I}(\theta)
=
\sum_t
J_t^T
\Sigma^{-1}
J_t,
\quad
J_t=\frac{\partial x_t^{pred}}{\partial\theta}.
$$

如果 $\mathcal{I}$ 某些特征值很小，对应参数方向不可辨。

这时继续学习会让参数漂移。

正确做法是：

| 情况 | 处理 |
|------|------|
| 激励不足 | 设计辨识动作 |
| 参数耦合 | 固定一部分参数，只学关键参数 |
| 噪声太大 | 加先验和正则 |
| 分布外 | 降低残差信任，回到保守控制 |

### S5.7.9 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 所有权重都自由学习 | 稳定项被降为零 | 外层 loss 没有覆盖所有失败模式 | 给安全相关权重设置下界 |
| 残差模型过强 | 仿真训练好，真实部署差 | 网络学会利用仿真漏洞 | 限制残差幅值和变化率 |
| 同时学习太多物理参数 | 参数漂移且不可解释 | 数据对参数不可辨 | 逐步解冻参数，做 Fisher/灵敏度检查 |
| 学到的终端价值过乐观 | MPC 把状态推向边界 | critic 分布外外推 | 终端价值加保守约束和 uncertainty |

### S5.7.10 练习

1. **设计题**：为 Go2 站立推扰任务设计可学习的 $Q/R$ 权重分组，并说明每组上下界。
2. **辨识题**：给定四旋翼飞行数据，哪些动作最有利于辨识质量，哪些动作最有利于辨识阻力？
3. **实现题**：把上面的 `BoundedResidualDynamics` 接到一个 double integrator MPC 中，比较有界和无界残差的训练稳定性。

---

## S5.8 稳定性、安全边界与可学习性的冲突 ⭐⭐⭐⭐

这一节解决的问题是：学习会改变 MPC 参数，怎样避免学到一个在训练 loss 上更好、但在物理上更危险的控制器？

### S5.8.1 MPC 稳定性的传统条件

经典 MPC 稳定性通常依赖三件事。

| 条件 | 作用 |
|------|------|
| 终端集合 $\mathcal{X}_f$ | 保证 horizon 结束时还能被局部控制器稳定 |
| 终端代价 $V_f(x)$ | 近似无限时域 cost-to-go |
| 递归可行性 | 当前可行意味着下一步仍有可行解 |

典型离散系统：

$$
x_{k+1}=f(x_k,u_k).
$$

若终端集合内存在局部控制律 $\kappa_f(x)$，满足：

$$
f(x,\kappa_f(x))\in \mathcal{X}_f,
$$

并且：

$$
V_f(f(x,\kappa_f(x)))-V_f(x)
\le
-\ell(x,\kappa_f(x)),
$$

则有限时域 MPC 可以继承稳定性。

这个条件的直觉是：

窗口结束时，系统不是掉进未知状态，而是进入一个已知可稳定区域。

终端代价则告诉优化器不要在窗口末端留下烂摊子。

### S5.8.2 学习会破坏什么

学习参数可能破坏上述条件。

| 学习对象 | 可能破坏的性质 |
|----------|----------------|
| $Q/R$ 权重 | 破坏 Lyapunov 下降方向，过度偏向任务误差 |
| 终端价值 $V_\theta$ | 过度乐观，诱导系统进入边界 |
| 动力学残差 | 改变可达集合和约束可行性 |
| 约束软化参数 | 让安全边界变成可被牺牲的代价项 |
| 参考生成器 | 输出不可达或不可稳定的参考 |

这说明可微 MPC 训练不能只优化平均任务损失。

必须同时约束学习空间。

### S5.8.3 安全边界的分层

安全条件应分成三层。

```text
硬边界：绝不允许违反
  例如关节硬限位、急停、最大电流、不可碰撞区域

控制边界：MPC 应显式满足
  例如摩擦锥、力矩限幅、ZMP/支撑裕度、CBF 约束

性能边界：允许短暂退化但要记录
  例如末端误差、速度误差、能耗、舒适性
```

学习器最多调节第二层和第三层之间的权衡。

第一层不应被学习器改写。

例如安全距离可以根据感知不确定性增大。

但不能让网络把安全距离学成负数。

### S5.8.4 CBF 与 MPC 的接口

控制障碍函数 CBF 给出安全集合：

$$
\mathcal{C}=\{x\mid h(x)\ge 0\}.
$$

离散时间可写成：

$$
h(x_{k+1})-(1-\alpha)h(x_k)\ge 0,
\quad \alpha\in(0,1].
$$

在 MPC 中，它是路径约束。

学习器可以调整参考和权重。

但 CBF 约束负责守住安全集合。

如果外层学习让任务目标穿过障碍，MPC 应拒绝而不是跟随。

这就是“学习意图，控制守边界”的结构。

### S5.8.5 鲁棒裕度

真实系统存在模型误差。

如果约束写成：

$$
g(x,u)\le 0,
$$

训练中最优解长期贴着 $g=0$，实机很容易违反。

所以需要裕度：

$$
g(x,u)+\rho \le 0.
$$

$\rho$ 可以固定。

也可以随不确定性变化：

$$
\rho(x)=\rho_0+\beta\sigma(x),
$$

其中 $\sigma(x)$ 是模型不确定性。

这与 S4 中可微仿真边界一致。

梯度能让参数更优。

裕度让系统不把最优解压在悬崖边。

### S5.8.6 分布外检测

可微 MPC 常和神经网络结合。

神经网络最怕分布外输入。

可用的检测信号包括：

| 信号 | 含义 |
|------|------|
| encoder 输出接近上下界 | 网络试图使用极端参数 |
| MPC slack 长期非零 | 任务或模型超出可行域 |
| KKT 残差上升 | 求解器面对陌生问题 |
| 残差模型不确定性高 | 学习模型缺少数据 |
| 实测预测误差上升 | sim-to-real 偏差变大 |

一旦检测到分布外，应进入保守模式。

保守模式可以是固定权重 MPC。

也可以是安全停机。

也可以是低速重新观察。

不要让网络在分布外继续自由调节关键参数。

### S5.8.7 代码示例：训练中的安全过滤

```python
def safe_parameter_filter(theta_raw, solver_stats, bounds):
    """对学习到的 MPC 参数做安全过滤。

    bounds[name] = (low, high, default)

    中文注释：这不是替代 MPC 约束，而是在进入求解器前防止明显危险参数。
    """
    theta = {}
    for name, value in theta_raw.items():
        low, high, default = bounds[name]
        theta[name] = torch.clamp(value, low, high)

    # 中文注释：如果上一轮求解器健康状态差，收缩参数到保守默认值附近。
    if solver_stats["kkt_residual"] > 1e-3 or solver_stats["solve_failed"]:
        for name, value in theta.items():
            _, _, default = bounds[name]
            theta[name] = 0.8 * default + 0.2 * value

    return theta
```

这里的过滤器看起来保守。

但安全关键控制中，保守是结构的一部分。

学习器应该在安全壳内探索。

### S5.8.8 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 把安全约束写成普通代价 | 任务压力大时越界 | 代价权重有限，约束可被交换 | 安全项用硬约束或高优先级软约束 |
| 学习终端价值但不限制外推 | MPC 故意进入危险状态 | critic 在未知区域过乐观 | 加终端集合、置信度和保守剪裁 |
| 训练只看平均回报 | 少数失败被平均掩盖 | 安全是尾部风险问题 | 记录 worst-case、CVaR 和约束违反率 |
| 可微梯度覆盖安全监控 | 实机出现高频危险动作 | 学习更新直接影响底层控制 | MPC 低频学习，高频 WBC/安全监控独立兜底 |

### S5.8.9 练习

1. **推导题**：写出一个离散 CBF 约束，并说明它如何放入 multiple shooting MPC。
2. **设计题**：为四足臂“边走边抓”任务列出硬边界、控制边界和性能边界。
3. **实验题**：训练可学习末端权重时，比较有安全过滤和无安全过滤的约束违反次数。

---

## S5.9 何时不该端到端求导 ⭐⭐⭐⭐

这一节很重要：可微并不等于应该求导。

### S5.9.1 不该求导的第一类场景：离散决策主导

如果任务成败主要由离散模式决定，端到端梯度往往帮助有限。

例子：

| 任务 | 离散决策 |
|------|----------|
| 四足换步 | 哪条腿何时接触 |
| 抓取 | 抓哪个点，何时闭合 |
| 开门 | 是否已接触门把手 |
| 避障 | 走左边还是右边 |
| 多接触操作 | 哪个接触面激活 |

这些决策改变问题结构。

梯度只能描述局部连续变化。

它很难从“走左边”连续变成“走右边”。

正确方法通常是：

```text
离散层：搜索 / 状态机 / 高层策略 / 采样
连续层：MPC / SQP / iLQR / WBC
```

不要指望连续梯度替代离散规划。

### S5.9.2 不该求导的第二类场景：硬接触事件时间决定结果

如果 loss 对接触事件时间高度敏感，梯度会非常不稳定。

例如球刚好撞墙。

控制稍微改变，碰撞发生或不发生。

损失会跳变。

这类问题中，有限差分也会对步长极其敏感。

可微仿真给出的局部梯度可能方向错误。

更稳妥的方法：

| 方法 | 作用 |
|------|------|
| 采样式 MPC / MPPI | 用零阶采样跨越事件边界 |
| SHAC/AHAC 短 horizon | 限制梯度穿过太多接触 |
| 接触模式显式建模 | 把事件变成模式切换 |
| 真实数据校验 | 防止仿真梯度过拟合 |

### S5.9.3 不该求导的第三类场景：安全认证链路

如果某个模块是安全认证的一部分，不应让学习梯度直接改变它。

例如：

| 模块 | 原因 |
|------|------|
| 急停逻辑 | 必须独立于学习系统 |
| 电流/力矩硬限幅 | 保护硬件 |
| 碰撞保护区 | 保护人和设备 |
| 状态估计异常检测 | 防止错误状态进入控制器 |
| 低层电机保护 | 高频实时安全 |

这些模块可以影响 loss。

但不应被梯度更新。

它们是控制系统的护栏。

### S5.9.4 不该求导的第四类场景：求解器残差主导梯度

如果求解器没有收敛，反传得到的是“算法残差敏感性”，不是“最优策略敏感性”。

判断标准：

| 信号 | 不该继续隐式反传的阈值例子 |
|------|---------------------------|
| KKT 残差 | 大于训练初期基线 10 倍 |
| QP 失败 | primal/dual infeasible |
| 线搜索失败 | 多次 step rejected |
| 正则过大 | $\lambda$ 长期处于上限 |
| 迭代超时 | 超过实时预算 |

此时应跳过该样本、降低学习率、回退参数或使用保守默认控制。

不要让 optimizer 用坏梯度更新网络。

### S5.9.5 端到端求导的决策流程

```text
是否有明确连续参数需要学习？
├── 否 → 不需要可微 MPC
└── 是
    ├── 任务是否主要由离散模式决定？
    │   ├── 是 → 分离离散规划和连续 MPC
    │   └── 否
    │       ├── 接触是否频繁且刚性？
    │       │   ├── 是 → 短 horizon / 平滑 / 采样校验
    │       │   └── 否
    │       │       ├── 求解器能稳定收敛吗？
    │       │       │   ├── 否 → 先修求解器与模型
    │       │       │   └── 是 → 使用隐式求导
    │       │       └── 安全边界能独立守住吗？
    │       │           ├── 否 → 先加安全过滤
    │       │           └── 是 → 可以端到端训练
```

这张流程图是工程选型工具。

当它给出“不要端到端求导”时，不代表技术落后。

它代表控制边界清晰。

### S5.9.6 与纯 RL、纯 MPC、蒸馏的比较

| 方案 | 优点 | 缺点 | 推荐场景 |
|------|------|------|----------|
| 纯 MPC | 可解释、约束强 | 权重和模型人工调 | 模型准确、安全关键 |
| 可微 MPC | 样本效率高、保留结构 | 梯度边界复杂 | 学参数、学残差、低中维系统 |
| 纯 RL | 表达能力强 | 样本多、安全弱 | 高维接触、仿真训练充分 |
| MPC 蒸馏 | 推理极快 | 约束保证弱化 | 1kHz 部署、已有大量 MPC 数据 |
| 采样式 MPC | 跨非光滑边界 | 采样成本高 | 多峰、非光滑、离散性强 |

> **本质洞察**：可微 MPC 位于“纯模型控制”和“纯数据策略”之间。
> 它最适合学习那些人难以手调、但又必须留在物理约束内的连续参数。
> 如果问题的关键是离散选择、感知语义或强接触事件，强行求导常常不如把边界拆清楚。

### S5.9.7 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 为了可微而软化所有约束 | 训练顺滑，部署危险 | 安全边界被变成可交换代价 | 只软化可训练边界，硬边界保留 |
| 用可微 MPC 解决离散规划 | 卡在局部最优 | 连续梯度不能跨模式 | 上层用搜索/采样/策略选择模式 |
| 把真实系统也当可微环境 | 期待实机能直接反传 | 真实系统只给数据，不给解析梯度 | 用离线数据、有限差分、策略梯度或 sim surrogate |
| 忽略求解器失败样本 | 训练后期突然崩溃 | 坏梯度累计 | 失败样本进入诊断队列而非正常反传 |

### S5.9.8 练习

1. **判断题**：下面任务是否适合端到端可微 MPC：四旋翼轨迹跟踪、四足乱石地形落脚选择、机械臂慢速插孔、从图像开抽屉。
2. **设计题**：为“四足跨障碍”设计一个混合方案，说明哪些部分用采样/搜索，哪些部分用可微 MPC。
3. **实验题**：构造一个带障碍的二维导航问题，比较梯度法和采样法在左右绕行选择上的差异。

---

## S5.10 调试可微 MPC：从梯度到行为 ⭐⭐⭐

这一节给出排查顺序。

可微 MPC 的错误通常不在一个层级。

它可能是数学梯度错。

可能是求解器没收敛。

可能是模型坐标系错。

可能是外层 loss 错。

也可能是训练数据不可辨。

### S5.10.1 五层排查链

| 层级 | 先问的问题 | 证据 |
|------|------------|------|
| 模型层 | 动力学、代价、约束写对了吗？ | 单步预测、单位、坐标系 |
| 求解层 | MPC 是否真正收敛到可行解？ | KKT 残差、slack、迭代数 |
| 梯度层 | backward 是否匹配有限差分？ | VJP 测试、方向导数 |
| 学习层 | 外层 loss 是否表达真实目标？ | 权重变化、训练/验证差距 |
| 行为层 | 闭环行为是否有裕度？ | 摩擦裕度、力矩余量、安全距离 |

不要反过来。

如果模型层错了，调学习率没有意义。

如果求解层没收敛，梯度层测试没有意义。

如果梯度不对，外层训练看似下降也不可信。

### S5.10.2 有限差分检查

对任意参数方向 $v$，检查方向导数：

$$
\frac{\mathcal{L}(\theta+\epsilon v)-\mathcal{L}(\theta-\epsilon v)}{2\epsilon}
\approx
\nabla_\theta\mathcal{L}^T v.
$$

$\epsilon$ 不能太大。

太大会跨过活动集边界。

$\epsilon$ 也不能太小。

太小会被浮点误差淹没。

常用扫描：

```text
epsilon = 1e-1, 1e-2, 1e-3, 1e-4, 1e-5
```

如果只有中间几个 $\epsilon$ 对得上，通常是正常。

如果全部对不上，梯度实现有问题。

如果在约束边界附近对不上，可能是非光滑活动集切换。

### S5.10.3 VJP 测试

完整 Jacobian 很大。

更实用的是测试 VJP。

给定输出空间随机向量 $w$ 和参数空间随机方向 $v$：

$$
w^T
\frac{\partial z^\star}{\partial\theta}
v
$$

应该与有限差分的标量方向导数匹配：

$$
\frac{
w^T z^\star(\theta+\epsilon v)
-
w^T z^\star(\theta-\epsilon v)
}{2\epsilon}
\approx
w^T
\frac{\partial z^\star}{\partial\theta}
v.
$$

这正是 autograd backward 需要的量。

不要为了测试构造完整 Jacobian。

它既慢，又不符合实际训练路径。

### S5.10.4 梯度异常的典型症状

| 症状 | 可能原因 | 检查 |
|------|----------|------|
| 梯度全零 | 参数未接入 OCP，活动约束屏蔽了影响 | 打印 $\partial F/\partial\theta$ 范数 |
| 梯度巨大 | KKT 矩阵病态，活动集切换 | 条件数、正则、barrier |
| 梯度方向与有限差分相反 | 符号错误，左/右误差定义错 | 单参数手算例子 |
| 训练 loss 下降但约束违反上升 | 外层 loss 缺少安全项 | 加裕度 loss 和硬约束 |
| 参数贴上下界 | 学习器想逃出安全壳 | 检查任务是否不可行 |

### S5.10.5 结构化排查表

| 症状 | 可能原因 | 排查步骤 | 修复方向 |
|------|----------|----------|----------|
| `loss.backward()` 出 NaN | KKT 线性系统奇异，或 barrier 中 $g\ge 0$ | 打印最小 slack、KKT 条件数、$Q_{uu}$ 特征值 | 加正则、增大 barrier、跳过失败样本 |
| 有限差分不匹配 | 活动集切换或 backward 符号错误 | 扫描 $\epsilon$，固定活动集测试，小维度手算 | 修正符号，远离边界测试 |
| MPC forward 经常 infeasible | 网络输出不可达参考或危险权重 | 打印参考轨迹、约束残差、slack | 限制参考速度，加参数过滤 |
| 训练越久求解越慢 | 学习器把问题推向约束边界 | 统计 active set 数量和迭代数 | 加 solve-time penalty 或裕度正则 |
| 仿真表现好实机差 | 学到软接触漏洞或残差过拟合 | 对比真实预测误差、穿透量、接触力 | 降低残差信任，加域随机化 |
| 末端任务好但机器人失稳 | 外层 loss 过度奖励末端误差 | 看 CoM/ZMP/摩擦裕度曲线 | 提高稳定权重下界，加入安全 loss |

### S5.10.6 调试日志建议

每次 MPC solve 至少记录：

```text
time_stamp
mode_id
solve_time_ms
sqp_iterations
kkt_residual
primal_residual
dual_residual
max_slack
active_constraints_count
min_friction_margin
min_collision_distance
max_torque_ratio
theta_min
theta_max
outer_loss
grad_norm_theta
finite_difference_check_status
```

这些字段能把学习问题和控制问题连接起来。

例如 grad norm 很大且 min friction margin 很小，说明学习正在把控制器推向摩擦边界。

例如 solve time 上升但 loss 下降，说明学习器在制造更难的优化问题。

### S5.10.7 常见陷阱

| 陷阱 | 现象 | 根本原因 | 正确做法 |
|------|------|----------|----------|
| 只用最终 reward 判断好坏 | 训练曲线漂亮但控制器不可部署 | reward 没有反映约束压力 | 同时画约束裕度和求解器健康 |
| 有限差分只测一个 $\epsilon$ | 误判梯度错误或正确 | 步长敏感 | 做 $\epsilon$ 扫描 |
| 只测随机大问题 | 很难定位符号错误 | 变量太多 | 先用 1D/2D 手算问题验证 |
| 不记录活动集 | 梯度跳变无法解释 | 约束状态缺失 | 保存 active constraints 和 multipliers |

### S5.10.8 练习

1. **实现题**：为一个等式约束 QP 写 VJP 测试，比较隐式梯度与有限差分。
2. **排查题**：给定“loss 下降但求解时间上升”的日志，列出三种可能原因和对应修复。
3. **综合题**：为可微 MPC 训练设计一张 TensorBoard 面板，至少包含 12 条曲线。

---

## S5.11 累积项目：可微双积分器 MPC 到四旋翼参数学习 ⭐⭐⭐

本章项目分四个阶段。

它从最小系统开始，逐步加入求解器求导、物理参数学习和安全边界。

### 阶段 1：双积分器可微 QP

目标：实现一个 1D 双积分器 MPC。

状态：

$$
x=[p,v]^T.
$$

控制：

$$
u=a.
$$

离散动力学：

$$
x_{k+1}
=
\begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix}x_k
+
\begin{bmatrix}
\frac{1}{2}\Delta t^2\\
\Delta t
\end{bmatrix}u_k.
$$

要求：

| 内容 | 验收 |
|------|------|
| 组装 QP | 能跟踪目标位置 |
| 隐式求导 | 对 $Q/R$ 的梯度通过有限差分 |
| 约束 | 控制满足 $|u|\le u_{max}$ |
| 日志 | 记录 KKT 残差和活动集 |

### 阶段 2：学习代价权重

用专家轨迹训练 $Q/R$。

专家可以来自手调 MPC。

也可以来自解析 LQR。

外层 loss：

$$
\mathcal{L}
=
\sum_t
\|u_t^\star(Q,R)-u_t^{expert}\|^2.
$$

要求权重有上下界。

记录训练前后的控制平滑度和跟踪误差。

### 阶段 3：加入可微仿真评估

用一个带摩擦误差的环境评估。

名义模型：

$$
v_{k+1}=v_k+\Delta t u_k.
$$

真实模型：

$$
v_{k+1}=v_k+\Delta t(u_k-cv_k).
$$

学习参数 $c$ 或残差模型。

比较：

| 方法 | 预期表现 |
|------|----------|
| 只学 $Q/R$ | 能补偿一部分误差 |
| 学物理参数 $c$ | 预测误差下降 |
| 学无界残差 | 训练快但容易不稳定 |
| 学有界残差 | 稳定且可解释 |

### 阶段 4：迁移到四旋翼简化模型

四旋翼平动简化模型：

$$
m\ddot{p}
=
R e_3 T - mg e_3 - D\dot{p}.
$$

可学习参数：

| 参数 | 物理意义 |
|------|----------|
| $m$ | 质量 |
| $D$ | 阻力系数 |
| $Q_p,Q_v$ | 位置和速度跟踪权重 |
| $R_T$ | 推力正则 |
| $V_f$ | 终端价值 |

任务是用轨迹数据学习 $m$ 和 $D$。

MPC 仍然保持推力限幅和倾角约束。

这能体现可微 MPC 的真正价值：学习改善模型，但不放弃控制约束。

---

## S5.12 延伸阅读与项目精读

| 资料 | 难度 | 重点 |
|------|------|------|
| Amos 等，Differentiable MPC for End-to-End Planning and Control | ⭐⭐⭐ | KKT 隐式求导和 box-constrained iLQR |
| locuslab/mpc.pytorch | ⭐⭐⭐ | 教学级 PyTorch 实现，适合读 backward |
| Frey 等，Differentiable Nonlinear Model Predictive Control | ⭐⭐⭐⭐ | 通用 NMPC 灵敏度、SQP/IPM 平滑 |
| leap-c | ⭐⭐⭐ | acados NMPC 作为 PyTorch 层的工程接口 |
| Theseus | ⭐⭐⭐ | 可微非线性最小二乘层，理解优化层通用模式 |
| TD-MPC2 | ⭐⭐ | model-based RL 强基线，用于区分采样 MPC 与可微 MPC |
| Actor-Critic MPC | ⭐⭐⭐ | MPC 作为 actor，RL 学习 MPC 参数 |
| S4 可微分仿真理论 | ⭐⭐⭐ | 接触梯度偏差和 SHAC/AHAC 边界 |

阅读顺序建议：

1. 先读 mpc.pytorch 的接口和示例，建立“求解器作为层”的直觉。
2. 再读 Amos 2018 的 KKT 推导，理解 backward 为什么像一次 LQR。
3. 再读 acados/leap-c 相关材料，理解一般约束 NMPC 的灵敏度。
4. 最后读 TD-MPC2 和 AC-MPC，形成“学习世界模型、学习 MPC 参数、采样规划”之间的边界。

---

## S5.13 本章小结

### S5.13.1 一张总表

| 主题 | 核心结论 | 工程落点 |
|------|----------|----------|
| 可微 MPC 定义 | 对最优解映射 $z^\star(\theta)$ 求导 | 求解器需要自定义 backward |
| MPC 与仿真接口 | 仿真梯度和 MPC 梯度通过链式法则连接 | 简化模型做优化，高保真模型做评估 |
| Shooting / Collocation | 离散化决定梯度链和稀疏结构 | 实时 NMPC 常用 multiple shooting |
| iLQR/DDP/SQP | backward 复用 Riccati/KKT 结构 | 不显式求逆，使用分解和伴随 |
| 隐式函数定理 | 对 KKT 条件求导 | KKT 残差和条件数决定梯度可信度 |
| 可微层实现 | forward 求解，backward 解伴随系统 | 记录 solver health |
| 学习对象 | 权重、参考、终端价值、物理参数、残差 | 必须有正定性、上下界和安全先验 |
| 稳定与安全 | 学习不能删除硬边界 | CBF、终端集合、裕度、过滤器 |
| 不该求导 | 离散模式、硬接触事件、安全认证链路 | 拆分离散规划和连续优化 |

### S5.13.2 三个核心公式

第一个公式是最优控制问题：

$$
z^\star(\theta)
=
\arg\min_z J(z;\theta)
\quad
\text{s.t.}
\quad
c(z;\theta)=0,\ g(z;\theta)\le 0.
$$

第二个公式是 KKT 隐式求导：

$$
\frac{\partial y^\star}{\partial\theta}
=
-
F_y^{-1}F_\theta.
$$

第三个公式是伴随梯度：

$$
F_y^Ta=r_y^T,
\quad
\frac{d\mathcal{J}}{d\theta}
=
-
a^TF_\theta+r_\theta.
$$

把这三个公式连起来，就是可微 MPC 的数学骨架。

### S5.13.3 本质洞察回收

可微 MPC 不是把控制器变成黑盒。

它恰恰要求更清楚地暴露控制器结构。

你必须知道状态是什么。

必须知道约束是什么。

必须知道哪个参数可学。

必须知道求解器是否收敛。

必须知道梯度是否可信。

它比普通端到端策略更麻烦。

也正因为麻烦，它保留了物理系统最需要的东西：约束、解释和边界。

### S5.13.4 章末自检题

1. **数学自检**：不看正文，推导无约束最优化的隐式求导公式。
2. **算法自检**：解释 iLQR backward pass 与 KKT 隐式 backward 的关系。
3. **工程自检**：写出一个可微 MPC layer 的 forward/backward 接口，说明需要保存哪些上下文。
4. **安全自检**：列出三个不应该被学习器直接修改的安全边界。
5. **综合自检**：为“四旋翼竞速”设计一个 AC-MPC 风格系统，说明 actor、critic、MPC 参数和外层 loss 分别是什么。

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 修复方向 |
|------|----------|----------|----------|
| 反向传播出现 NaN | KKT 矩阵奇异、barrier 参数过小、求解器未收敛 | 打印 KKT 残差、最小 slack、条件数、$Q_{uu}$ 特征值 | 加正则、增大平滑、失败样本跳过 |
| 隐式梯度与有限差分不一致 | 活动集切换、符号错误、步长不合适 | 对 $\epsilon$ 做扫描，固定活动集，在小问题上手算 | 修正符号，避开非光滑点测试 |
| 训练 loss 下降但实机更差 | 学到仿真漏洞、残差过强、安全裕度不足 | 对比 sim/real 预测误差、穿透量、摩擦裕度 | 限制残差、加入真实数据、增大鲁棒裕度 |
| MPC 求解越来越慢 | 学习器把参数推向边界，活动约束增多 | 统计 active constraints、迭代数、solve time p99 | 加 solve-time 正则，收缩参数范围 |
| 控制动作过激 | 控制正则过小，终端价值过乐观 | 检查 $R$ 权重、控制限幅乘子、终端状态 | 提高 $R$ 下界，限制 critic 外推 |
| 约束长期依赖 slack | 任务不可行或权重冲突 | 按约束类型打印 slack 时间序列 | 调整参考、增大安全距离、降低任务权重 |
| 梯度全零 | 参数未进入 OCP，或对应约束非活动 | 打印 $F_\theta$ 范数和参数到代价/动力学的路径 | 修复参数接线，改用更有激励的数据 |
| 学到的物理参数漂移 | 数据不可辨，参数耦合 | 计算灵敏度矩阵特征值，设计激励轨迹 | 固定不可辨参数，加先验正则 |

---

## 附：术语速查

| 术语 | 一句话解释 |
|------|------------|
| unroll differentiation | 把求解器迭代展开成计算图后反传 |
| implicit differentiation | 对最优性条件或固定点方程求导 |
| KKT residual | 最优性条件的残差，衡量求解是否可信 |
| active set | 当前恰好贴住边界的不等式约束集合 |
| adjoint sensitivity | 用一次转置线性系统求外层梯度 |
| solution sensitivity | 最优解对参数的导数 |
| RTI | 每个 MPC 周期只做一次 SQP 迭代的实时策略 |
| barrier smoothing | 用内点法屏障让不等式灵敏度更光滑 |
| residual dynamics | 在物理模型之外学习的小修正项 |
| terminal value | 用于弥补有限 horizon 短视的终端代价 |

### ⚠️ 章末陷阱：把优化器未收敛的解拿去反传

可微 MPC 的 backward 通常默认 forward 解接近最优。若 KKT 残差很大、约束违反明显或求解器提前中止，反传得到的梯度会混合求解误差和真实灵敏度。训练 loop 应记录 solve status，并对失败样本降权或跳过。

### ⚠️ 章末陷阱：让学习器直接改安全约束

外层学习可以调代价权重、模型残差或参考轨迹，但不应随意放松碰撞、力矩、关节限位和摩擦锥这类安全边界。若必须学习约束边界，应加物理先验和人工下界。

### ⚠️ 章末陷阱：忽略 active set 切换

不等式约束的活动集一旦变化，最优解对参数的导数可能出现不连续。有限差分验证时应同时记录 active set，否则同一个参数扰动可能比较的是两个不同局部问题。

### 练习：从一个小 QP 开始理解可微 MPC

**A 型**：写出一维约束 QP $\min_x (x-\theta)^2$，约束 $x \leq 1$。分别在 $\theta<1$、$\theta=1$、$\theta>1$ 三种情况下求 $dx^*/d\theta$。

**B 型**：实现一个无约束 iLQR 小车问题，记录 backward pass 中 $Q_{uu}$ 的最小特征值。解释为什么需要 regularization。

**C 型**：给一个可微 MPC layer 设计日志字段，至少包含 KKT 残差、求解时间、active set 数量、slack 最大值、外层 loss 和梯度范数。

---

## 参考资料

- Amos, Jimenez, Sacks, Boots, Kolter, “Differentiable MPC for End-to-End Planning and Control”, NeurIPS 2018.
- locuslab/mpc.pytorch: `https://github.com/locuslab/mpc.pytorch`
- Frey, Baumgärtner 等, “Differentiable Nonlinear Model Predictive Control”, arXiv 2505.01353.
- FreyJo/differentiable_nmpc: `https://github.com/FreyJo/differentiable_nmpc`
- leap-c: `https://github.com/leap-c/leap-c`
- Pineda, Amos, Zhang, Mukadam, Boots, “Theseus: A Library for Differentiable Nonlinear Optimization”, NeurIPS 2022.
- facebookresearch/theseus: `https://github.com/facebookresearch/theseus`
- Hansen, Su 等, “TD-MPC2: Scalable, Robust World Models for Continuous Control”, ICLR 2024.
- Romero, Bauersfeld, Scaramuzza, “Actor-Critic Model Predictive Control”, T-RO 2025.
