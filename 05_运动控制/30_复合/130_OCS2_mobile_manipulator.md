# 第 83 章：OCS2 mobile_manipulator 精读——运动学 OCP、SE(3) 代价与约束设计

> 本章定位：把第 82 章（120_底盘臂联合规划.md）的底盘 + 臂联合运动学放进 OCS2 最优控制框架。
> 核心对象：`ocs2_robotic_examples/ocs2_mobile_manipulator`。
> 学习重点：模型选择、状态输入定义、末端代价、自碰撞约束、关节限位、SQP/SLQ 求解配置。
> 难度：⭐⭐⭐。
> 预计时间：2 周，源码阅读 14 小时，推导 10 小时，实验 12 小时。

---

## 83.0 前置自测

阅读前请回答：

| # | 问题 | 建议回顾 |
|---|------|----------|
| 1 | OCS2 的 `OptimalControlProblem` 聚合了哪些组件？ | 足式/110 |
| 2 | 移动操作系统的状态 $x=[x_b,y_b,\theta_b,q_a]$ 和输入 $u=[v,\omega,\dot q_a]$ 如何解释？ | 复合/120 |
| 3 | $SE(3)$ 末端误差为什么不应使用欧拉角差？ | 复合/30 |
| 4 | Pinocchio 正运动学如何得到末端 frame 位姿？ | 机械臂/M01 |
| 5 | 自碰撞距离约束的梯度为什么需要雅可比？ | 机械臂/M04 |

自测标准：

- 5/5 正确：可以直接读源码。
- 3-4/5 正确：阅读时重点看状态、输入和代价定义。
- 0-2/5 正确：建议先回顾 OCS2 五层架构、Pinocchio 和联合雅可比。

---

## 83.1 本章目标

学完本章，你应能：

1. 画出 OCS2 `mobile_manipulator` 的文件级架构。
2. 区分固定基座、轮式底盘、浮动基座等模型族；具体配置字段以当前仓库版本为准。
3. 写出差速底盘 + n 轴臂的状态、输入和动力学。
4. 推导末端 $SE(3)$ tracking cost。
5. 解释为什么示例采用运动学模型而非完整刚体动力学。
6. 设计输入权重、末端权重和关节正则权重。
7. 解释关节限位、自碰撞和外部障碍约束如何进入 OCP。
8. 编写一个简化版 OCP 构造框架。
9. 调试 SQP 不收敛、末端抖动、底盘绕圈和约束不可行。

---

## 83.2 为什么 OCS2 mobile_manipulator 是重要样例

OCS2 最出名的是腿足 MPC。

但 `mobile_manipulator` 样例展示了另一类问题：没有接触切换，但有强冗余和末端任务。

它适合学习 OCS2 的原因：

- 状态维度低。
- 动力学简单。
- 末端代价清晰。
- 约束类型完整。
- Pinocchio、CppAD、SQP、ROS 封装都能看到。

与腿足 `legged_robot` 对比：

| 维度 | legged_robot | mobile_manipulator |
|------|--------------|--------------------|
| 模式切换 | 足端接触切换 | 通常无接触切换 |
| 状态 | 质心动量 + 基座 + 关节 | 底盘位姿 + 臂关节 |
| 输入 | 接触力 + 关节速度/力 | 底盘速度 + 臂关节速度 |
| 代价 | 速度、姿态、接触力 | 末端位姿、输入、姿态正则 |
| 约束 | 摩擦锥、接触、摆腿 | 关节限位、自碰撞、障碍 |
| 模型 | centroidal / kino-centroidal | kinematic |

> **本质洞察**：`mobile_manipulator` 是 OCS2 中“运动学最优控制”的代表。
> 它不追求真实力矩，而是直接优化可执行速度。
> 这种简化让系统可以高频重规划，非常适合服务机器人和低速移动操作。

---

## 83.3 文件级地图

典型目录结构：

```text
ocs2_robotic_examples/
  ocs2_mobile_manipulator/
    include/
      ocs2_mobile_manipulator/
        MobileManipulatorInterface.h
        MobileManipulatorPreComputation.h
        ManipulatorModelInfo.h
        dynamics/
        cost/
        constraint/
    src/
      MobileManipulatorInterface.cpp
      MobileManipulatorPreComputation.cpp
      ManipulatorModelInfo.cpp
      dynamics/
      cost/
      constraint/
    config/
      franka/
      kinova/
      ridgeback_ur5/
    urdf/
    ros/
```

核心文件职责：

不同 OCS2 版本的文件名可能变化，下表按职责阅读。

| 文件 | 职责 |
|------|------|
| `MobileManipulatorInterface.cpp` | 读取配置、加载模型、组装 OCP、创建 solver |
| `MobileManipulatorPreComputation.cpp` | 预计算 Pinocchio 运动学和缓存 |
| `ManipulatorModelInfo.cpp` | 解析模型类型、维度、frame 名称 |
| 轮式底盘动力学源文件 | 差速底盘 + 臂运动学流映射 |
| 默认机械臂动力学源文件 | 默认机械臂运动学模型 |
| `EndEffectorCost.cpp` | 末端位姿 tracking 代价 |
| `EndEffectorConstraint.cpp` | 末端约束或软约束包装 |
| `JointVelocityLimits.cpp` | 关节速度限制 |
| `task.info` | horizon、权重、约束和求解器参数 |

阅读顺序建议：

1. `task.info`。
2. `ManipulatorModelInfo`。
3. dynamics 文件。
4. EndEffector cost。
5. PreComputation。
6. Interface 的 OCP 组装。
7. solver 设置和 ROS 封装。

---

## 83.4 模型类型

OCS2 mobile manipulator 的不同版本在模型类型字符串、枚举值和配置字段上可能不完全相同。

下面使用教学抽象名称描述模型族，不把这些名称当成稳定 API。

写工程配置时，应以正在使用的仓库版本、示例配置和 `ManipulatorModelInfo` 解析逻辑为准。

| 教学模型族 | 底盘 | 状态 | 输入 | 适用场景 |
|------|------|------|------|----------|
| fixed-base manipulator | 无 | $q_a$ | $\dot q_a$ | 固定机械臂 |
| default manipulator | 简化基座 | $q_a$ 或简化浮动项 | $\dot q_a$ | 教学和测试 |
| wheel-based mobile manipulator | 差速底盘 | $x_b,y_b,\theta_b,q_a$ | $v,\omega,\dot q_a$ | Ridgeback+UR5 |
| floating-base arm | 全驱动浮动 | 浮动位姿 + 臂 | 浮动速度 + 臂速度 | 全向或研究模型 |

本章重点是轮式底盘模型族。

因为它最能体现第 82 章（120_底盘臂联合规划.md）的差速底盘非完整约束。

---

## 83.5 状态与输入定义

差速底盘 + n 轴臂：

$$
x=
\begin{bmatrix}
x_b\\y_b\\\theta_b\\q_a
\end{bmatrix}
\in\mathbb{R}^{3+n_a}
$$

输入：

$$
u=
\begin{bmatrix}
v\\\omega\\\dot q_a
\end{bmatrix}
\in\mathbb{R}^{2+n_a}
$$

动力学：

$$
\dot x=
f(x,u)=
\begin{bmatrix}
v\cos\theta_b\\
v\sin\theta_b\\
\omega\\
\dot q_a
\end{bmatrix}
$$

这不是完整刚体动力学。

它是运动学模型。

为什么足够？

因为许多移动操作任务速度低。

底盘速度控制器已经在下层闭环。

机械臂也常由速度或位置控制器执行。

OCS2 负责生成平滑、避障、可达的速度轨迹。

---

## 83.6 运动学模型的代价

运动学模型不预测力矩。

因此它无法直接保证：

- 电机力矩不饱和。
- 轮胎不打滑。
- 抓取重物时基座不晃。
- 动态接触力可行。

但它带来巨大优势：

- 状态小。
- 线性化简单。
- 求解快。
- 适合频繁重规划。
- 容易与 Nav2/MoveIt2 共存。

> **反事实推理**：如果把完整多体动力学放入 mobile manipulator OCP，会怎样？
> 状态要加入速度，输入要加入力矩，约束要加入驱动极限和接触力。
> 求解问题更真实，但也更慢。
> 对低速抓取而言，这些动态细节往往由下层控制器处理，统一放进 MPC 反而不划算。

---

## 83.7 flowMap 代码骨架

```cpp
#include <ocs2_core/Types.h>

class WheelBasedDynamics {
 public:
  ocs2::vector_t flowMap(ocs2::scalar_t time,
                         const ocs2::vector_t& state,
                         const ocs2::vector_t& input) const {
    (void)time;
    const double theta = state(2);
    const double v = input(0);
    const double omega = input(1);

    ocs2::vector_t dx(state.size());
    dx.setZero();

    // 中文注释：差速底盘在世界系的速度。
    dx(0) = v * std::cos(theta);
    dx(1) = v * std::sin(theta);
    dx(2) = omega;

    // 中文注释：输入后 n_a 维直接作为机械臂关节速度。
    dx.tail(state.size() - 3) = input.tail(input.size() - 2);
    return dx;
  }
};
```

自动微分版本需要支持 `ad_scalar_t`。

因此实际 OCS2 代码通常写成模板化实现。

不要在模板 flowMap 中调用只支持 double 的函数。

---

## 83.8 线性化直觉

动力学：

$$
\dot x_b=v\cos\theta
$$

$$
\dot y_b=v\sin\theta
$$

对状态求导：

$$
\frac{\partial \dot x_b}{\partial\theta}=-v\sin\theta
$$

$$
\frac{\partial \dot y_b}{\partial\theta}=v\cos\theta
$$

对输入求导：

$$
\frac{\partial \dot x_b}{\partial v}=\cos\theta
$$

$$
\frac{\partial \dot y_b}{\partial v}=\sin\theta
$$

$$
\frac{\partial \dot\theta}{\partial\omega}=1
$$

机械臂部分：

$$
\frac{\partial \dot q_a}{\partial \dot q_a}=I
$$

线性化矩阵非常稀疏。

这也是该模型求解快的重要原因。

---

## 83.9 末端位姿计算

末端位姿：

$$
T_{we}(x)=T_{wb}(x_b,y_b,\theta_b)T_{be}(q_a)
$$

Pinocchio 负责计算 $T_{be}(q_a)$。

底盘位姿 $T_{wb}$ 由平面位姿构造。

在实现中要注意 frame 名称。

`base_link`、`arm_base_link`、`tool0`、`ee_link` 可能因 URDF 不同而不同。

配置文件必须明确末端 frame。

错误 frame 会导致：

- 末端目标偏移。
- 姿态误差方向不对。
- 规划看似收敛但工具实际不到位。

---

## 83.10 SE(3) 末端误差

期望末端位姿：

$$
T_d\in SE(3)
$$

当前末端位姿：

$$
T=T_{we}(x)
$$

本章固定采用下面这个误差方向：

$$
T_e=T_d^{-1}T
$$

也就是先把当前末端位姿拉回期望末端坐标系，再取李代数误差。

在这个约定下，代码应写成：

```cpp
const SE3 T_error = T_desired.inverse() * T_current;
const Vector6 e = log(T_error);
```

有人会把另一种方向写成：

$$
T_e=T^{-1}T_d
$$

这不是同一个残差。

在小误差附近，它近似改变误差符号；在大姿态误差下，它还会改变雅可比伴随项的坐标表达。

因此不要只写“左不变”或“右不变”这类名称。

更可靠的写法是直接写清乘法方向：本章使用 `desired.inverse() * current`。

关键是整个代码一致。

误差向量：

$$
e=\log(T_e)^\vee\in\mathbb{R}^6
$$

末端代价：

$$
\ell_{ee}(x)=\frac12 e^\top Q_{ee}e
$$

权重：

$$
Q_{ee}=\mathrm{diag}(q_x,q_y,q_z,q_{roll},q_{pitch},q_{yaw})
$$

位置和姿态单位不同。

因此权重不能盲目相同。

常见起点：

```text
position weights: 100, 100, 100
orientation weights: 10, 10, 10
```

若姿态不重要，先降低姿态权重。

若末端抖动，先增加输入权重而不是继续增加末端权重。

---

## 83.11 末端代价展开

在 SQP 或 SLQ 中，需要代价的一阶和二阶近似。

若：

$$
\ell=\frac12 e(x)^\top Q e(x)
$$

一阶梯度：

$$
\ell_x=J_e^\top Qe
$$

其中：

$$
J_e=\frac{\partial e}{\partial x}
$$

Gauss-Newton 近似 Hessian：

$$
\ell_{xx}\approx J_e^\top QJ_e
$$

为什么不用完整 Hessian？

完整 Hessian 包含误差函数的二阶导。

Gauss-Newton 近似在误差较小时足够好。

它保证 Hessian 半正定。

这对数值稳定很重要。

---

## 83.12 输入代价

输入代价：

$$
\ell_u=\frac12(u-u_{ref})^\top R(u-u_{ref})
$$

对差速底盘：

$$
R=\mathrm{diag}(R_v,R_\omega,R_{\dot q_1},...,R_{\dot q_n})
$$

调参直觉：

- $R_v$ 太小：底盘喜欢前后移动。
- $R_\omega$ 太小：底盘喜欢原地转。
- $R_{\dot q}$ 太小：机械臂动作激进。
- 所有 $R$ 太大：末端跟踪变慢。

输入权重还表达偏好。

若希望先动手臂、少动底盘，提高 $R_v,R_\omega$。

若希望底盘主动帮忙，降低底盘输入权重。

---

## 83.13 关节正则代价

关节正则：

$$
\ell_q=\frac12(q_a-q_a^{nom})^\top Q_a(q_a-q_a^{nom})
$$

它不是主任务。

它用于冗余空间。

作用：

- 远离关节限位。
- 避免肘部奇怪姿态。
- 保持机械臂收敛到可解释构型。
- 减少多解跳变。

权重过高会牺牲末端跟踪。

权重过低会导致关节漂移。

---

## 83.14 代价组合

总运行代价：

$$
\ell(x,u)=
\ell_{ee}(x)
+\ell_u(u)
+\ell_q(x)
+\ell_b(x)
$$

其中 $\ell_b$ 是底盘姿态或位置正则。

例如希望底盘保持朝向：

$$
\ell_b=\frac12 w_\theta \mathrm{wrap}(\theta-\theta_{ref})^2
$$

若目标是边走边操作，$\theta_{ref}$ 可来自路径方向。

若目标是抓取，$\theta_{ref}$ 可来自目标物体方向。

---

## 83.15 约束一：关节速度限制

输入中包含 $\dot q_a$。

因此关节速度限制直接是输入边界：

$$
\dot q_{\min}\le \dot q_a\le\dot q_{\max}
$$

底盘速度也有边界：

$$
v_{\min}\le v\le v_{\max}
$$

$$
\omega_{\min}\le \omega\le \omega_{\max}
$$

在 OCS2 中，边界可作为 constraint 或 penalty 处理。

硬边界更安全。

软惩罚更容易收敛。

工程上常先用软约束调通，再逐步收紧。

---

## 83.16 约束二：关节位置限制

状态中包含 $q_a$。

位置限制：

$$
q_{\min}\le q_a\le q_{\max}
$$

如果只限制速度，不限制位置，长时间运行可能慢慢漂到限位。

常用 relaxed barrier。

### 83.16.1 Relaxed Barrier 的完整推导 ⭐⭐⭐

**动机**：内点法使用对数障碍函数 $-\mu \log(h)$ 来处理不等式约束 $h \ge 0$。当 $h > 0$ 时，对数障碍在边界附近提供无穷大代价，完美阻止越界。但它有一个致命缺点：当约束已经被违反（$h \le 0$）时，$\log(h)$ 无定义。在 MPC 中，初始猜测、线性化误差或数值扰动都可能让状态暂时越界。此时 $-\mu \log(h)$ 会返回 NaN，整个求解崩溃。

**Relaxed barrier 的解决方案**：在 $h > \delta$（远离边界）时保留对数障碍的行为；在 $h \le \delta$（接近或越过边界）时用一个二次函数平滑延拓，使得函数在全域有定义且连续可微。

完整定义为：

$$
\phi(h) =
\begin{cases}
-\mu \log(h), & h > \delta \\
\frac{\mu}{2}\left(\frac{h - 2\delta}{\delta^2} - 1\right) - \mu \log(\delta), & h \le \delta
\end{cases}
$$

**推导过程**：

**Step 1**：在 $h = \delta$ 处要求函数值连续：

$$
\lim_{h \to \delta^+} \phi(h) = -\mu \log(\delta)
$$

$$
\lim_{h \to \delta^-} \phi(h) = \frac{\mu}{2}\left(\frac{\delta - 2\delta}{\delta^2} - 1\right) - \mu \log(\delta) = \frac{\mu}{2}\left(-\frac{1}{\delta} - 1\right) - \mu \log(\delta)
$$

这里需要选择二次延拓的系数使两端相等。上面的特定形式保证了在 $h = \delta$ 处 $\phi(\delta) = -\mu \log(\delta)$。

**Step 2**：在 $h = \delta$ 处要求一阶导连续：

$$
\phi'(h)\big|_{h>\delta} = -\frac{\mu}{h}
$$

$$
\phi'(\delta^+) = -\frac{\mu}{\delta}
$$

二次部分的一阶导为 $\frac{\mu}{2\delta^2}$（对 $h$ 求导），在 $h = \delta$ 处等于 $\frac{\mu}{2\delta^2}$。通过调整系数可以匹配。

**Step 3**：一阶导连续保证了 SQP 在约束边界附近不会产生梯度跳变，从而避免迭代振荡。

**参数 $\mu$ 和 $\delta$ 的物理含义**：

| 参数 | 含义 | 典型值 | 调参方向 |
|------|------|--------|----------|
| $\mu$ | 障碍强度 | 1e-2 到 1e0 | 越大越不容易越界，但约束附近代价梯度更陡 |
| $\delta$ | 对数-二次切换点 | 1e-3 到 1e-1 | 越小越接近纯对数障碍，越大越平滑但越早偏离理想约束边界 |

> **跨领域类比**：relaxed barrier 类似于弹簧缓冲器的设计。纯对数障碍像刚性墙壁——碰到就崩溃。relaxed barrier 像在墙前加了一段弹簧——接近时阻力逐渐增大，即使越过设定边界也不会突然断裂，而是提供一个有限但很大的恢复力。

> **反事实推理**：如果不用 relaxed barrier 而用纯对数障碍会怎样？
> 当 SQP 的线性化步长过大，某个关节从 $q = q_{\min} + 0.01$ 跳到 $q = q_{\min} - 0.001$ 时，$\log(h)$ 未定义。
> 求解器返回 NaN，MPC 无法输出有效命令，机器人在该周期失去控制。
> relaxed barrier 在这种情况下返回一个有限但很大的代价值和梯度，把关节"推回"可行域内。

**OCS2 中的实现方式**

OCS2 使用 `RelaxedBarrierPenalty` 类来包装不等式约束。配置文件中通常包含：

```ini
relaxedBarrier
{
  mu 0.01
  delta 0.001
}
```

工程上调参的经验是：先用较大的 $\mu = 0.1$ 和 $\delta = 0.01$ 确保求解器能收敛，再逐步减小使约束更紧。如果直接用小值，初始猜测违约时求解器可能找不到可行方向。

---

## 83.17 约束三：自碰撞距离

自碰撞对 $(i,j)$ 的最小距离：

$$
d_{ij}(q)
$$

安全约束：

$$
h_{ij}(q)=d_{ij}(q)-d_{\min}\ge 0
$$

距离梯度：

$$
\nabla_q d_{ij}
=\hat n^\top(J_{p_i}-J_{p_j})
$$

其中 $\hat n$ 是最近点连线方向。

$J_{p_i}$ 和 $J_{p_j}$ 是两个最近点的平移雅可比。

Pinocchio 计算 frame 雅可比。

FCL/Coal 计算最近点和距离。

CppAD 需要可微路径。

实际实现中，最近碰撞对集合通常离线配置或在线筛选。

---

## 83.18 约束四：外部障碍距离

环境可以用 ESDF 表示。

对于机器人上包络球中心 $p_k(q)$：

$$
h_k(q)=\Phi(p_k(q))-r_k-d_{\min}\ge 0
$$

$\Phi(p)$ 是点到障碍物的 signed distance。

梯度：

$$
\nabla_q h_k=\nabla\Phi(p_k)^\top J_{p_k}
$$

外部障碍约束的难点不是公式。

难点是地图同步和延迟。

如果 ESDF 在求解中途被另一个线程修改，优化器会看到不一致环境。

应使用 solver synchronized module 或双缓冲。

---

## 83.19 PreComputation 的意义

OCS2 的 cost 和 constraint 都可能需要 Pinocchio 正运动学。

如果每个 cost 都自己计算一次，开销和状态不一致风险都会增加。

PreComputation 负责在一个状态输入点上统一计算：

- generalized coordinates。
- forward kinematics。
- frame placements。
- frame Jacobians。
- 自碰撞相关缓存。
- 目标轨迹插值。

然后 cost 和 constraint 读取缓存。

> **本质洞察**：PreComputation 是 OCS2 中“避免重复计算”和“保证同一点线性化一致”的关键结构。
> 它不是性能小优化，而是复杂 OCP 的组织方式。

---

## 83.20 task.info 配置结构

典型配置包含：

```ini
mpc
{
  timeHorizon 2.0
  solutionTimeWindow 0.1
}

sqp
{
  maxNumIterations 2
  minRelCost 1e-4
  constraintTolerance 1e-3
}

model
{
  type wheel_based
  urdfFile ridgeback_ur5.urdf
  endEffectorFrame tool0
}

cost
{
  endEffector
  {
    positionWeight 100 100 100
    orientationWeight 10 10 10
  }
  input
  {
    baseLinear 1.0
    baseAngular 1.0
    armVelocity 0.1
  }
}
```

字段名以实际版本为准。

教学重点是理解配置背后的数学意义。

---

## 83.21 权重调参顺序

推荐顺序：

1. 关闭碰撞约束，只保留输入代价和末端位置代价。
2. 调通位置跟踪。
3. 加入姿态代价。
4. 加入关节正则。
5. 加入关节限位。
6. 加入自碰撞。
7. 加入外部障碍。
8. 提高跟踪权重。
9. 收紧约束容差。

不要一开始打开所有约束。

否则失败时无法判断是模型、权重还是约束导致。

---

## 83.22 Solver 选择

移动操作 OCP 可用：

| 求解器 | 优点 | 缺点 | 适用 |
|--------|------|------|------|
| SLQ/iLQR | 快、适合低维平滑问题 | 硬约束处理弱 | 早期调试 |
| SQP | 硬约束清晰 | 每次迭代更重 | 约束较多 |
| IPM | 约束处理强 | 配置更复杂 | 大规模约束 |

低速移动操作常用 SQP。

若只做末端跟踪和输入代价，SLQ 也能很好工作。

当自碰撞和外部障碍变多，SQP 更稳。

### 83.22.1 SQP-RTI 求解器内部：多重射击与 KKT 系统 ⭐⭐⭐

OCS2 的 SQP 求解器使用**多重射击（Multiple Shooting）**离散化连续 OCP。理解它的内部结构对调试收敛问题至关重要。

**多重射击的核心思想**

单射击把整条轨迹从初始状态一路积分到末端，初值偏差会在 horizon 末端被放大。多重射击则在每个离散时间节点 $t_k$ 上引入独立的状态变量 $x_k$，并通过**连续性约束（defect constraint）**强制相邻节点的积分一致：

$$
x_{k+1} = F(x_k, u_k, \Delta t_k) + d_k, \quad d_k = 0
$$

其中 $F(\cdot)$ 是一步积分（如显式 Euler 或 Runge-Kutta），$d_k$ 是"缺陷向量"。优化同时调整所有 $x_k$ 和 $u_k$，让 $d_k \to 0$。

> **跨领域类比**：多重射击的思想类似于动画关键帧插值。单射击像只定第一帧然后自动播放，中间帧由物理引擎决定——如果参数有偏，最后一帧偏很远。多重射击像在多个关键帧上都放锚点，然后同时调整所有帧，使整条动画既平滑又满足约束。

**KKT 系统结构**

在每次 SQP 迭代中，求解器需要求解一个**KKT（Karush-Kuhn-Tucker）线性系统**。将所有节点的决策变量叠成一个大向量 $\mathbf{w} = [x_0, u_0, x_1, u_1, \ldots, x_N]$，KKT 系统的结构为：

$$
\begin{bmatrix}
H_0 & & A_0^T & & \\
& H_1 & -I & A_1^T & \\
A_0 & -I & & & \ddots \\
& A_1 & & & \\
& & \ddots & &
\end{bmatrix}
\begin{bmatrix}
\Delta w_0 \\
\Delta w_1 \\
\lambda_0 \\
\lambda_1 \\
\vdots
\end{bmatrix}
=
\begin{bmatrix}
-g_0 \\
-g_1 \\
d_0 \\
d_1 \\
\vdots
\end{bmatrix}
$$

其中 $H_k$ 是第 $k$ 段的 Hessian（代价 + 约束的二阶近似），$A_k = \partial F / \partial (x_k, u_k)$ 是线性化动力学，$g_k$ 是梯度，$\lambda_k$ 是连续性约束的对偶变量。

这个矩阵有**带状稀疏结构**：每个时间节点只与相邻节点耦合。OCS2 利用 Riccati 递推高效求解，复杂度为 $O(N \cdot n^3)$ 而非稠密求解的 $O(N^3 n^3)$。

**RTI（Real-Time Iteration）**

移动操作 MPC 常采用 RTI 策略：每个控制周期只做**一次 SQP 迭代**，然后立刻输出当前最优步长作为控制命令。下一个周期用新的状态作为初始点继续迭代。

| 对比 | 完整 SQP | RTI |
|------|----------|-----|
| 每周期迭代 | 直到收敛（2-10 次） | 1 次 |
| 计算量 | 大 | 小 |
| 解的质量 | 全局较优 | 持续改进 |
| 实时性 | 难保证 | 强保证 |
| 适用场景 | 离线轨迹优化 | 在线 MPC |

RTI 的代价是单步解可能不最优。但由于 MPC 每周期都重新求解，误差会在多次迭代中被逐步消除。

> **反事实推理**：如果不用 RTI 而要求每周期完全收敛，会怎样？
> 当 SQP 需要 5 次迭代才收敛时，求解时间可能从 1 ms 增加到 5 ms。
> 对 100 Hz MPC，5 ms 已经占了半个周期。
> 如果遇到约束激活或参考突变导致需要更多迭代，求解会超时。
> RTI 把这个风险从"偶发超时"变成"每次固定计算量"，代价是解的质量略低。

### 83.22.2 从零组装一个 Mobile Manipulator MPC：完整工作流 ⭐⭐⭐

本节给出一个完整的思维过程，展示如何从头构建一个移动操作 MPC。不是贴代码，而是把每一步的决策逻辑写清楚。

**Step 1：确定系统维度和坐标**

假设平台是 Ridgeback 差速底盘 + UR5 六轴臂。

$$
n_x = 3 + 6 = 9, \quad n_u = 2 + 6 = 8
$$

底盘状态 $(x_b, y_b, \theta_b)$ 在世界系，臂状态 $q_a \in \mathbb{R}^6$ 为关节角。输入 $(v, \omega)$ 是底盘线速度和角速度，$\dot{q}_a$ 是臂关节速度。

**Step 2：写出 Flow Map**

$$
\dot{x} = f(x, u) = \begin{bmatrix} v \cos\theta_b \\ v \sin\theta_b \\ \omega \\ \dot{q}_a \end{bmatrix}
$$

这是纯运动学模型。为什么不加惯性项？因为底盘有自己的速度控制器，臂有自己的关节伺服。MPC 负责"在哪个方向走多快"，不负责"要施加多大力矩"。

**Step 3：选择代价函数**

总代价分四项：

$$
\ell(x,u) = \underbrace{\frac{1}{2}\xi^T Q_{ee} \xi}_{\text{末端任务}} + \underbrace{\frac{1}{2}u^T R u}_{\text{输入正则}} + \underbrace{\frac{1}{2}(q_a - q_a^{nom})^T Q_a (q_a - q_a^{nom})}_{\text{关节居中}} + \underbrace{\frac{1}{2}w_\theta(\theta_b - \theta_{ref})^2}_{\text{底盘朝向}}
$$

设计时必须回答三个问题：(a) 末端和底盘朝向谁重要？先让末端占主导。(b) 输入正则多大？太小底盘抖，太大末端追不上。(c) 关节居中正则多大？太大牺牲末端，太小关节漂移。

**Step 4：选择约束**

按重要性排列：

| 优先级 | 约束 | 类型 | 理由 |
|------|------|------|------|
| 1 | 关节位置限幅 | relaxed barrier | 硬件安全 |
| 2 | 关节速度限幅 | 输入边界 | 电机能力 |
| 3 | 底盘速度限幅 | 输入边界 | 运动能力 |
| 4 | 自碰撞 | relaxed barrier 或软约束 | 安全 |
| 5 | 外部障碍 | 软约束 | 环境安全 |

**Step 5：组装 PreComputation**

每个节点需要计算一次 Pinocchio FK 来获取末端位姿和雅可比。PreComputation 在状态 $(x_b, y_b, \theta_b, q_a)$ 上完成这些计算，然后把结果缓存给所有代价和约束使用。这避免了 FK 被重复调用 3-5 次的问题。

**Step 6：选择求解器和参数**

对 9 维状态、8 维输入、2 秒 horizon、20 个离散节点的问题：
- SQP-RTI 每次迭代约 0.5-2 ms，适合 50-100 Hz MPC。
- SLQ 更快但处理不等式约束弱，适合只有代价没有硬约束的简化版。
- 初始配置建议 `maxNumIterations = 2`，让 MPC 在每个周期最多做 2 次 SQP 迭代。

**Step 7：调试顺序**

严格按照 83.21 节的推荐：先关闭碰撞约束，只保留末端位置代价和输入正则。确认位置跟踪后逐步加入姿态、正则、限位、碰撞。每加一项都记录求解时间、收敛状态和行为变化。

---

## 83.23 MPC/MRT 数据流

```text
State estimator
  │ x0
  ▼
Reference manager
  │ target trajectories
  ▼
MPC solver
  │ stateTrajectory, inputTrajectory, controller
  ▼
MPC-MRT buffer
  │ policy
  ▼
MRT query
  │ interpolated input
  ▼
base velocity controller + arm velocity controller
```

移动操作中，MRT 输出常是速度命令。

底盘执行 $v,\omega$。

机械臂执行 $\dot q_a$ 或转成位置增量。

若下层只接受位置轨迹，需要积分：

$$
q_a(t+\Delta t)=q_a(t)+\dot q_a(t)\Delta t
$$

并做速度和位置限幅。

---

## 83.24 目标轨迹设计

目标轨迹不应只包含最终位姿。

最好给 OCS2 一个随时间变化的末端参考：

```text
t0: 当前末端位姿
t1: 预抓取位姿
t2: 抓取位姿
t3: 抬起位姿
```

参考轨迹太突变会导致输入尖峰。

可以用 $SE(3)$ 插值。

位置用三次样条。

姿态用四元数 slerp 或 Lie 群插值。

---

## 83.25 与 MoveIt2 的关系

MoveIt2 擅长全局避障路径。

OCS2 擅长局部连续重规划和控制。

常见组合：

1. MoveIt2 生成粗路径。
2. OCS2 跟踪路径并实时避障。
3. OCS2 输出速度命令。
4. 下层控制器执行。

另一种组合：

1. OCS2 直接优化短时域末端目标。
2. MoveIt2 仅在 OCS2 失败时提供全局重规划。

二者不是替代关系。

它们在时间尺度上互补。

---

## 83.26 适配新机器人

适配步骤：

1. 准备 URDF。
2. 确认 base link 和 arm base link。
3. 确认 end-effector frame。
4. 设置模型类型。
5. 设置状态维度和输入维度。
6. 设置关节名顺序。
7. 设置关节限位。
8. 设置输入限幅。
9. 设置末端权重。
10. 运行静态 FK 测试。
11. 运行零输入 rollout。
12. 运行单目标跟踪。
13. 加入约束。
14. 接入 ROS 或控制器。

关节顺序错误是高频错误。

必须用一个已知关节构型验证末端位姿。

---

### 83.26.1 适配中的关键验证步骤 ⭐⭐

在按照 83.26 的清单完成基本配置后，必须通过以下验证才能认为适配成功：

**验证一：单关节脉冲测试**。对每个关节发送一个微小正向速度命令（如 $\dot{q}_i = 0.01$ rad/s），观察机器人响应。如果第 3 关节的命令导致第 5 关节运动，说明关节索引错位——这是最高频的适配错误。

**验证二：已知构型下的 FK 比较**。把所有关节设为零位（或一个已知角度），用 Pinocchio 计算末端位姿，然后与 URDF 可视化或 CAD 测量对比。误差应在 1 mm 以内。如果差距大，排查 URDF 中 joint origin 的平移和旋转。

**验证三：零输入 rollout**。给 MPC 零目标（保持当前位姿），观察求解器是否在第一步就收敛。如果发散，通常是初始状态不可行或输入权重过小导致的。

> **反事实推理**：如果跳过单关节脉冲测试直接调末端跟踪会怎样？
> 关节索引错位时，末端可能朝错误方向移动。工程师会以为是权重问题，反复调权重。
> 实际上无论怎么调权重，只要索引错误，控制方向就永远不对。
> 单关节脉冲测试用 10 分钟排除了可能浪费 10 小时的问题。

---

## 83.27 简化 OCP 构造框架

```cpp
#include <Eigen/Dense>

#include <string>
#include <utility>

struct MobileManipulatorOcpConfig {
  std::string urdf;
  std::string base_link;
  std::string ee_frame;
  int arm_dof = 6;
  Eigen::Matrix<double, 6, 6> Q_ee;
  Eigen::MatrixXd R_input;
};

class MobileManipulatorOcpBuilder {
 public:
  explicit MobileManipulatorOcpBuilder(MobileManipulatorOcpConfig cfg)
      : cfg_(std::move(cfg)) {}

  void build() {
    loadRobotModel();
    createDynamics();
    createPreComputation();
    createCosts();
    createConstraints();
  }

 private:
  void loadRobotModel() {
    // 中文注释：加载 URDF，建立 Pinocchio 模型，查找末端 frame。
  }

  void createDynamics() {
    // 中文注释：根据模型类型创建差速底盘或全向底盘动力学。
  }

  void createPreComputation() {
    // 中文注释：创建共享的运动学预计算缓存。
  }

  void createCosts() {
    // 中文注释：注册末端位姿代价、输入代价、关节正则代价。
  }

  void createConstraints() {
    // 中文注释：注册关节限位、自碰撞、外部障碍等约束。
  }

  MobileManipulatorOcpConfig cfg_;
};
```

这个框架对应 `MobileManipulatorInterface` 的职责。

真正项目中要把每个步骤拆成可测试函数。

---

## 83.28 故障排查：末端到不了目标

现象：

- OCS2 求解成功。
- 末端仍离目标很远。
- 输入没有饱和。

原因：

- 末端 frame 配错。
- 目标位姿坐标系错误。
- $SE(3)$ 误差左右乘约定不一致。
- 末端权重太低。
- horizon 太短。
- 初始状态不是当前真实状态。

处理：

1. 用 Pinocchio 单独验证 FK。
2. 在 RViz 显示目标 frame。
3. 打印当前末端位姿和目标位姿。
4. 先只做位置目标。
5. 增加 horizon。
6. 检查状态估计输入。

---

## 83.29 故障排查：底盘绕圈

原因：

- 角速度权重太低。
- 目标姿态权重过高。
- 差速底盘无法横向移动，优化器用旋转补偿。
- 初始底盘朝向不利。
- 末端目标超出机械臂舒适工作区。

处理：

1. 提高 $R_\omega$。
2. 降低姿态权重。
3. 用 base placement 给更好初值。
4. 增加关节正则。
5. 给底盘朝向参考。

---

## 83.30 故障排查：末端抖动

原因：

- 末端权重过高。
- 输入权重过低。
- 参考轨迹不连续。
- MRT 插值周期与控制周期不匹配。
- 视觉目标噪声直接输入 OCS2。

处理：

1. 对目标轨迹低通或样条平滑。
2. 增加输入权重。
3. 降低姿态权重。
4. 检查控制频率。
5. 加入目标更新限速。

---

## 83.31 故障排查：约束不可行

原因：

- 目标在障碍物内。
- 关节限位过紧。
- 自碰撞安全距离过大。
- 初值已经违反约束太多。
- 同时要求末端跟踪和避障但空间不足。

处理：

1. 先关掉外部障碍约束。
2. 降低安全距离。
3. 把硬约束改成软约束调试。
4. 用 MoveIt2 提供可行初值。
5. 降低末端跟踪权重。
6. 增加 horizon。

---

## 83.32 故障排查：求解时间过长

原因：

- 自动微分模型太复杂。
- 自碰撞 pair 太多。
- horizon 太长。
- SQP 迭代次数太多。
- PreComputation 未复用。
- 日志或可视化进入求解路径。

处理：

1. 减少 collision pair。
2. 缩短 horizon。
3. 降低 SQP 最大迭代。
4. 检查代码生成是否启用。
5. 确认 FK 没被重复调用。
6. 将可视化放到非求解线程。

---

## 83.33 练习

### 练习 A：动力学手推

给定状态 $x=[x,y,\theta,q_1,...,q_6]$，输入 $u=[v,\omega,\dot q_1,...,\dot q_6]$。

要求：

1. 写出 flowMap。
2. 手推 $A=\partial f/\partial x$。
3. 手推 $B=\partial f/\partial u$。
4. 与自动微分输出比较。

### 练习 B：末端代价

实现一个 $SE(3)$ 末端代价。

要求：

1. 用 Pinocchio 计算当前末端位姿。
2. 用 Lie 群对数计算 6D 误差。
3. 实现代价值。
4. 数值差分验证梯度方向。
5. 比较欧拉角误差和 Lie 群误差。

### 练习 C：权重实验

对同一抓取目标，测试三组权重：

1. 底盘输入权重大。
2. 机械臂输入权重大。
3. 姿态权重大。

观察：

- 底盘移动距离。
- 机械臂关节速度。
- 末端误差。
- 求解时间。

### 练习 D：自碰撞约束

选择两个 link pair。

要求：

1. 计算距离。
2. 写出 $h=d-d_{\min}$。
3. 用雅可比近似距离梯度。
4. 在接近碰撞时观察约束值。
5. 将安全距离从 2 cm 调到 10 cm，观察可行性变化。

---

## 83.34 累积项目：适配一个新移动操作平台

目标：把 OCS2 mobile manipulator 思路适配到一个新平台。

平台可选：

- Ridgeback + UR5。
- Husky + UR5。
- Stretch 简化模型。
- 自建差速底盘 + 6 轴臂。
- Go2-W + 小型机械臂的低速轮式模式。

模块：

1. URDF 准备。
2. frame 名称确认。
3. 状态和输入定义。
4. flowMap。
5. 末端代价。
6. 输入代价。
7. 关节正则。
8. 关节限位。
9. 自碰撞。
10. ROS 或仿真执行接口。

交付：

1. 一份模型维度说明。
2. 一份 `task.info` 或等价参数文件。
3. 一张 OCP 组件图。
4. 一段末端目标跟踪日志。
5. 一份权重调参记录。
6. 一份故障排查记录。

成功标准：

- 零输入 rollout 正确。
- 单点目标可收敛。
- 末端轨迹平滑。
- 输入不饱和。
- 约束开启后仍可求解。
- 日志能解释每项代价和约束。

---

## 83.35 本章速查

1. `mobile_manipulator` 是 OCS2 运动学 OCP 的代表。
2. 差速底盘状态为 $x,y,\theta$。
3. 输入为 $v,\omega,\dot q_a$。
4. flowMap 非常稀疏。
5. 模型简单带来高频重规划能力。
6. 运动学模型不直接保证力矩可行。
7. 末端位姿由底盘位姿和臂 FK 相乘得到。
8. 末端误差应使用 $SE(3)$ 对数。
9. Gauss-Newton Hessian 常用于末端二次代价。
10. 输入权重表达底盘和臂的运动偏好。
11. 关节正则利用冗余空间。
12. 关节速度限制是输入边界。
13. 关节位置限制是状态约束。
14. 自碰撞距离约束需要最近点和雅可比。
15. 外部障碍约束常用 ESDF。
16. 地图同步比距离公式更容易出错。
17. PreComputation 避免重复 FK。
18. `task.info` 是数学权重的工程入口。
19. 调参应从少约束开始。
20. SQP 适合带硬约束的移动操作。
21. SLQ 适合平滑低约束调试。
22. MRT 输出通常是速度命令。
23. 若硬件只接收位置，需要积分速度。
24. 参考轨迹应平滑。
25. MoveIt2 和 OCS2 可以互补。
26. MoveIt2 更偏全局几何路径。
27. OCS2 更偏局部连续最优控制。
28. 新机器人适配首先要验证 frame 和关节顺序。
29. 大多数奇怪行为来自坐标系或权重尺度。
30. 约束不可行时先缩小问题再逐项恢复。

---

## 83.36 延伸阅读

| 材料 | 难度 | 阅读重点 |
| --- | --- | --- |
| OCS2 mobile_manipulator 示例源码 | ⭐⭐⭐ | `MobileManipulatorInterface`、flowMap、代价项和约束注册方式 |
| OCS2 optimal control problem 文档 | ⭐⭐⭐ | cost、constraint、pre-computation 与 solver 设置的职责边界 |
| Pinocchio 文档中的 forward kinematics 与 frame placement | ⭐⭐ | 末端位姿、frame Jacobian 和自动微分所需的运动学接口 |
| 复合/120_底盘臂联合规划 | ⭐⭐ | 本章状态、输入和联合雅可比的上游来源 |
| MoveIt2 与 OCS2 组合案例 | ⭐⭐⭐ | 全局几何规划与局部连续最优控制的分工 |

---

## 83.37 调试检查清单

- 001 URDF 是否能被 Pinocchio 加载。
- 002 `nq` 和状态维度是否匹配。
- 003 `nv` 和输入维度是否匹配。
- 004 base link 名称是否正确。
- 005 end-effector frame 是否正确。
- 006 关节顺序是否与状态向量一致。
- 007 底盘 yaw 单位是否为弧度。
- 008 初始状态是否来自当前机器人。
- 009 目标位姿是否在 world frame。
- 010 当前末端位姿是否单独验证。
- 011 $SE(3)$ 误差左右乘约定是否一致。
- 012 位置和姿态权重是否尺度合理。
- 013 输入权重是否避免过大速度。
- 014 底盘线速度限幅是否设置。
- 015 底盘角速度限幅是否设置。
- 016 关节速度限幅是否设置。
- 017 关节位置限幅是否设置。
- 018 关节正则目标是否在限位内。
- 019 自碰撞 pair 是否过多。
- 020 自碰撞安全距离是否过大。
- 021 外部障碍 frame 是否正确。
- 022 ESDF 是否双缓冲。
- 023 PreComputation 是否复用 FK。
- 024 CppAD 代码生成是否成功。
- 025 SQP 迭代次数是否合理。
- 026 horizon 是否足够。
- 027 参考轨迹是否连续。
- 028 MRT 插值是否与控制周期匹配。
- 029 速度积分是否做限幅。
- 030 位置命令是否平滑。
- 031 日志是否记录每项代价。
- 032 日志是否记录约束最小值。
- 033 求解失败是否保存当时状态。
- 034 可视化是否显示目标 frame。
- 035 可视化是否显示预测轨迹。
- 036 底盘绕圈时是否检查角速度权重。
- 037 末端抖动时是否检查目标噪声。
- 038 约束不可行时是否能逐项关闭。
- 039 新机器人是否先跑无障碍目标。
- 040 控制器是否能安全停止。

---

## 83.38 小结

本章从源码组织、数学模型、代价函数和约束四个角度拆解了 OCS2 `mobile_manipulator`。

它的核心选择是运动学 OCP：用较低维状态和速度输入换取快速重规划。

末端 $SE(3)$ 代价、输入正则、关节正则、自碰撞和障碍距离共同定义了移动操作任务。

掌握这个样例后，再阅读腿足操作或轮足加臂的统一 MPC，会更容易分清哪些复杂性来自动力学，哪些复杂性来自任务和约束设计。

---

## 83.39 OCS2 版本演进与工程注意事项 ⭐⭐⭐

### OCS2 的开源现状

OCS2 由 ETH Zurich 的 Robotic Systems Lab 开发并开源。截至 2025 年，仓库仍在维护，但更新节奏有所放缓。主要变化集中在以下方面。

第一，求解器选择。早期版本主要提供 SLQ/iLQR 和 SQP。后续版本引入了 IPM（内点法）求解器，对硬约束处理更优。如果项目需要大量碰撞约束，应优先考虑 IPM。

第二，代码生成。CppAD/CppADCodeGen 用于自动微分和代码生成。不同编译器版本对代码生成路径的影响较大。GCC 12+ 和 Clang 15+ 的 C++17 特性支持可能改变模板实例化行为。在适配新平台时，建议先编译 `ocs2_core` 和 `ocs2_pinocchio_interface`，确认代码生成不报错。

第三，ROS 2 适配。OCS2 最初基于 ROS 1。社区和实验室已有 ROS 2 移植尝试。如果项目使用 ROS 2，需要关注 MPC-MRT 通信方式是否已迁移到 ROS 2 的 DDS 机制。

> **反事实推理**：如果直接使用一年前的教程配置而不检查当前版本的 API 变化，最常见的错误是 `EndEffectorCost` 的构造函数签名变化和 `task.info` 中配置字段的重命名。这类错误会在编译阶段暴露，但如果只改到能编译通过而不理解新参数含义，运行时行为可能与预期不同。

### 与 Pinocchio 3.x 的兼容

Pinocchio 从 2.x 到 3.x 有较大 API 变化。OCS2 对 Pinocchio 的依赖主要在运动学和动力学计算。如果 OCS2 版本要求 Pinocchio 2.x 而系统安装了 3.x，需要注意：

- `pinocchio::forwardKinematics()` 的调用方式可能不同。
- `example_robot_data` 的加载路径已从宏定义改为函数。
- frame 访问方式可能从 `model.frames[idx]` 变为 `model.getFrame(name)`。

建议按 OCS2 仓库的 `CMakeLists.txt` 中指定的 Pinocchio 版本锁定依赖。

### 跨章综合练习

结合本章的 OCS2 OCP 结构和第 82 章（120_底盘臂联合规划.md）的联合雅可比，完成以下任务：

1. 在 OCS2 `task.info` 中修改末端权重，使底盘几乎不动而臂完成抓取。
2. 再改权重使底盘主动前移帮助臂够到更远目标。
3. 记录两种配置下的底盘移动距离、末端误差、求解迭代次数和求解时间。
4. 分析权重变化如何影响冗余度分配。

---

## 83.40 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|----------|----------|----------|
| 末端到不了目标 | frame 配错或 SE(3) 误差约定不一致 | 1. Pinocchio 单独验证 FK 2. RViz 显示目标 3. 先只做位置目标 | 83.28 |
| 底盘绕圈 | 角速度权重太低或目标超出舒适区 | 1. 提高 R_omega 2. 降低姿态权重 3. 给底盘朝向参考 | 83.29 |
| 末端抖动 | 末端权重过高或参考不连续 | 1. 对目标做平滑 2. 增加输入权重 3. 检查控制频率 | 83.30 |
| 约束不可行 | 目标在障碍内或安全距离过大 | 1. 关掉障碍约束 2. 降低安全距离 3. 硬约束改软约束 | 83.31 |
| 求解时间过长 | collision pair 过多或 CppAD 未启用 | 1. 减少碰撞对 2. 缩短 horizon 3. 检查代码生成 | 83.32 |
| CppAD 编译失败 | 编译器版本或 Pinocchio 版本不匹配 | 1. 检查 CMakeLists 依赖 2. 确认 C++17 支持 3. 降级或升级编译器 | 83.39 |
