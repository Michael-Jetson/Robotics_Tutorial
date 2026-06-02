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

下表的文件名以 `leggedrobotics/ocs2` 仓库 `main` 分支为准。不同版本/旧 fork 命名可能略有出入，但职责划分稳定，按职责阅读即可。

| 文件 | 职责 |
|------|------|
| `MobileManipulatorInterface.cpp` | 读取配置、加载模型、组装 `OptimalControlProblem`、创建 solver |
| `MobileManipulatorPreComputation.h/.cpp` | 预计算 Pinocchio 运动学和缓存 |
| `ManipulatorModelInfo.h` | 模型类型枚举、维度、frame 名称、关节名列表 |
| `FactoryFunctions.h/.cpp` | 从 URDF/配置构造 `ManipulatorModelInfo`、加载 Pinocchio 模型 |
| `MobileManipulatorPinocchioMapping.h` | OCS2 状态向量 $x$ ↔ Pinocchio 广义坐标 $q$ 的映射 |
| `AccessHelperFunctions.h` | 从状态/输入向量切片读取底盘位姿、臂关节等子块 |
| `dynamics/WheelBasedMobileManipulatorDynamics.h` | 差速底盘 + 臂运动学 flow map |
| `dynamics/DefaultManipulatorDynamics.h` | 固定/默认机械臂运动学模型 |
| `dynamics/FloatingArmManipulatorDynamics.h` | 浮动基座臂（欠驱动）模型 |
| `dynamics/FullyActuatedFloatingArmManipulatorDynamics.h` | 全驱动浮动基座臂模型 |
| `cost/QuadraticInputCost.h` | 输入正则代价 $\frac12(u-u_{ref})^\top R(u-u_{ref})$ |
| `constraint/EndEffectorConstraint.h` | 末端 $SE(3)$ 误差约束（被软约束包装成跟踪代价） |
| `constraint/MobileManipulatorSelfCollisionConstraint.h` | 自碰撞距离约束 |
| `task.info` | horizon、权重、约束和求解器参数 |

> **注意**：真实仓库**没有** `EndEffectorCost.cpp`，也没有独立的 `JointVelocityLimits.cpp`。末端跟踪走 `EndEffectorConstraint` + 软约束包装（详见 §83.19A）；关节限位由 `MobileManipulatorInterface::getJointLimitSoftConstraint()` 统一产出。若按旧教程找这两个文件会扑空。

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

### 83.17.1 OCS2 自碰撞的真实实现：SelfCollision 与 PinocchioGeometryInterface ⭐⭐⭐

上面的公式 $\nabla_q d_{ij}=\hat n^\top(J_{p_i}-J_{p_j})$ 是教学推导。这一节看 `leggedrobotics/ocs2` 仓库 `ocs2_self_collision` 包是怎么把它落到代码的，以及为什么这里藏着移动操作 MPC 最容易踩的几个坑。

**两层职责划分**。OCS2 把自碰撞拆成几何层和约束层两个职责：

| 类 | 职责 | 关键方法 |
|----|------|----------|
| `PinocchioGeometryInterface` | 持有 Pinocchio `GeometryModel`，调用 hpp-fcl/Coal 算每个碰撞对的最近距离和最近点 | `computeDistances(pinocchioInterface)` |
| `SelfCollision` | 把距离打包成约束值 $h=d-d_{\min}$，并算解析雅可比 | `getValue(...)`、`getLinearApproximation(...)`、`getNumCollisionPairs()` |
| `MobileManipulatorSelfCollisionConstraint` | OCS2 状态约束接口，把上面的几何结果接进 OCP | 继承 `StateConstraint` |

`SelfCollision::getLinearApproximation()` 返回的是 `std::pair<vector_t, matrix_t>`——`first` 是所有碰撞对的约束值向量 $h\in\mathbb{R}^{n_{pair}}$，`second` 是对广义坐标的雅可比 $\partial h/\partial q$，正是 $\hat n^\top(J_{p_i}-J_{p_j})$ 逐对堆叠的结果。

**解析路径 vs 自动微分路径**。仓库同时提供两套实现，这是一个重要的工程选择点：

| 实现 | 类 | 距离梯度怎么来 | 取舍 |
|------|----|--------------|------|
| 解析 | `SelfCollision` + `SelfCollisionConstraint` | 用 $\hat n^\top(J_{p_i}-J_{p_j})$ 闭式公式，雅可比由 Pinocchio frame Jacobian 给 | 快、无需代码生成；但 $\hat n$ 在距离穿过零（互相穿透）时不连续 |
| 自动微分 | `SelfCollisionCppAd` + `SelfCollisionConstraintCppAd` | 把整条距离计算写成 CppAD 可微表达式，代码生成出梯度 | 首次编译慢、需要可微的距离实现；运行期快且导数一致 |

> **不是 X 而是 Y**：自碰撞的难点**不是**"算距离的公式不会写"，**而是**"距离函数在穿透和切换最近点对时不光滑"。最近点连线方向 $\hat n$ 在两个凸体刚好接触时方向定义模糊，在最近特征从"面-面"跳到"边-边"时方向突变。这会让解析雅可比出现跳变，进而让 SQP 步长在碰撞临界处来回横跳。

> **跨领域类比**：距离函数的不光滑性，类似于绝对值函数 $|x|$ 在原点的不可导。远离零点时梯度干净（$\pm 1$），但跨过零点梯度翻转。自碰撞距离在"刚好接触"这个点上就有类似的尖角。relaxed barrier（83.16.1）的二次延拓把约束推到"还没接触"就开始发力，正是为了让优化器**永远不要走到那个尖角**。相似之处仅在"都有一个导数不连续点"；不同之处是距离函数的不连续来自几何最近特征切换，而 $|x|$ 来自符号翻转——所以自碰撞还需要额外处理"最近点对集合在线变化"的问题。

**碰撞对从哪来**。`PinocchioGeometryInterface` 构造时需要一份碰撞对列表。常见两种来源：

1. URDF/SRDF 里声明的 collision geometry，配合 SRDF 的 `disable_collisions` 排除天然相邻、永不碰撞的连杆对（如相邻关节的连杆）。
2. 在 `task.info` 里手工列出关心的 link pair 名字。

如果不排除相邻连杆，会出现一类隐蔽 bug：相邻连杆在装配上本来就"贴着"，它们的距离恒小于 $d_{\min}$，于是自碰撞约束**永远被违反**，relaxed barrier 持续输出巨大代价，整个 OCP 被这个假约束带偏。

> **常见陷阱（概念误区）**：忘记用 SRDF 排除相邻连杆的碰撞对。
> 现象：MPC 一启动就"求解成功但行为诡异"，末端怎么都到不了，或求解代价居高不下。
> 根本原因：相邻连杆本就接触，$d_{ij}<d_{\min}$ 恒成立，barrier 项常驻，污染梯度。
> 正确做法：提供 SRDF 的 `disable_collisions`，或在碰撞对列表里只放真正可能相撞的远端 link pair（如夹爪 vs 底盘、肘部 vs 底盘）。
> 自检方法：打印每个碰撞对的初始 $d_{ij}$，零位构型下任何恒小于 $d_{\min}$ 的对要么排除、要么调小 $d_{\min}$。

**为什么不能放太多碰撞对**。`getNumCollisionPairs()` 直接决定约束维度。$n_{pair}$ 个对意味着每个节点要算 $n_{pair}$ 次 hpp-fcl 最近距离查询，外加 $n_{pair}\times n_q$ 的雅可比。对 20 节点 horizon、6 轴臂，把碰撞对从 5 个加到 50 个，求解时间可能翻好几倍（对应 83.32 求解时间过长）。

> **反事实推理**：如果把机器人所有连杆两两组合全设成碰撞对会怎样？
> $n$ 个连杆有 $\binom{n}{2}$ 对，6 连杆就是 15 对，加上底盘和夹爪更多。
> 每对每节点一次 hpp-fcl 查询 + 一次雅可比，约束维度爆炸。
> 求解时间从毫秒级涨到几十毫秒，100 Hz MPC 直接超时。
> 实践中只保留"几何上真有可能相撞"的少数对（通常 3-10 对），这是精度和实时性的工程折中，而非懒惰。

把这套实现接进 OCP 的方式，正是 §83.19A 第 (4) 步：

```cpp
// 中文注释：自碰撞 = SelfCollision 几何 + StateSoftConstraint + 单个 RelaxedBarrierPenalty。
problem.stateSoftConstraintPtr->add(
    "selfCollision",
    getSelfCollisionConstraint(pinocchioInterface, taskFile, urdfFile, "selfCollision", usePreComputation, ...));
```

`getSelfCollisionConstraint` 内部：构造 `PinocchioGeometryInterface`（读碰撞对）→ 包成 `SelfCollisionConstraint`（解析）或 `SelfCollisionConstraintCppAd`（自动微分）→ 再用一个 `RelaxedBarrierPenalty` 软化 → 装进 `StateSoftConstraint`。约束维度等于碰撞对数，但所有对共享同一个 barrier 参数 $(\mu,\delta)$。

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

## 83.19A OCP 的真实组装结构：成本、约束、软约束三层 ⭐⭐⭐

前面几节分别讲了末端代价、输入代价、关节限位和自碰撞约束的**数学形式**。但如果你打开真实仓库的 `MobileManipulatorInterface.cpp`，会发现这些项不是平铺直叙地"加到一个代价里"，而是被分门别类放进 `OptimalControlProblem` 的几个不同容器中。理解这套容器结构，是从"会读公式"过渡到"会读 OCS2 源码"的关键一步。本节完全基于 `leggedrobotics/ocs2` 仓库 `ocs2_mobile_manipulator` 包的真实接口讲解。

### 动机：为什么不能把所有项塞进一个 cost

设想一个朴素实现：把末端误差、输入正则、关节正则、自碰撞惩罚全部加权求和，写进一个 `getCost()` 函数返回。这样做能跑，但会立刻暴露三个工程问题。

第一，**有些项只依赖状态 $x$，有些项同时依赖状态和输入 $(x,u)$**。末端位姿 $\ell_{ee}(x)$ 只看状态；输入正则 $\ell_u(u)$ 只看输入。如果混在一起，求解器无法利用"这一项的 $\partial/\partial u = 0$"这个稀疏性，白白多算一堆零块。

第二，**有些项是真正的约束，有些项是真正的代价**。关节限位本质是不等式约束 $q_{\min}\le q\le q_{\max}$，而末端跟踪本质是软目标。把二者写成同一个加权和，就丢失了"这是硬性安全边界"和"这是可以妥协的偏好"的语义区分。

第三，**终端项和运行项需要分开**。MPC 在 horizon 末端 $t_N$ 处的代价（final cost）和中间节点 $t_k$ 的运行代价（intermediate cost）作用不同：终端项决定"轨迹最终往哪收敛"，运行项决定"路上怎么走"。

> **不是 X 而是 Y**：OCS2 的 `OptimalControlProblem` **不是一个返回标量代价的函数对象，而是一组带名字的容器**。每个容器装一类项（纯状态代价、状态-输入代价、软约束、终端软约束……），求解器分别对每个容器线性化，再按时间结构组装 KKT 系统。

### `OptimalControlProblem` 的容器结构

OCS2 把一个最优控制问题拆成下面几个核心成员（位于 `ocs2_oc/optimal_control_problem/OptimalControlProblem.h`）：

| 成员指针 | 装什么 | 依赖变量 | 本章对应项 |
|----------|--------|----------|-----------|
| `dynamicsPtr` | 系统动力学 flow map | $(x,u)$ | `WheelBasedMobileManipulatorDynamics` |
| `costPtr` | 运行期状态-输入代价集合 | $(x,u)$ | 输入正则 `QuadraticInputCost` |
| `stateCostPtr` | 运行期纯状态代价集合 | $x$ | （本例未直接用，末端走软约束） |
| `softConstraintPtr` | 运行期状态-输入软约束 | $(x,u)$ | 关节限位 `jointLimits` |
| `stateSoftConstraintPtr` | 运行期纯状态软约束 | $x$ | 末端跟踪 `endEffector`、自碰撞 `selfCollision` |
| `finalCostPtr` | 终端纯状态代价 | $x(t_N)$ | （可选） |
| `finalSoftConstraintPtr` | 终端纯状态软约束 | $x(t_N)$ | 终端末端跟踪 `finalEndEffector` |
| `equalityConstraintPtr` | 硬等式约束 | $(x,u)$ | （本例未用） |
| `preComputationPtr` | 共享预计算 | $(x,u)$ | `MobileManipulatorPreComputation` |

每个集合（Collection）暴露一个 `add(name, std::move(term))` 方法。`name` 是字符串句柄，调试时可以按名字打印每一项的当前值——这正是 83.21 节"日志记录每项代价"能落地的底层机制。

### 真实组装代码（基于 `MobileManipulatorInterface.cpp`）

下面这段是对真实接口组装逻辑的**忠实重构**（精简了文件读取细节，类名和容器名与仓库一致）：

```cpp
// 中文注释：以下类名、容器名均来自 leggedrobotics/ocs2 仓库真实接口。
ocs2::OptimalControlProblem problem;

// (1) 动力学：按模型类型选择，本章是轮式底盘。
switch (modelInfo.manipulatorModelType) {
  case ManipulatorModelType::WheelBasedMobileManipulator:
    problem.dynamicsPtr.reset(
        new WheelBasedMobileManipulatorDynamics(modelInfo, "dynamics", libraryFolder, ...));
    break;
  case ManipulatorModelType::DefaultManipulator:
    problem.dynamicsPtr.reset(new DefaultManipulatorDynamics(modelInfo, ...));
    break;
  // FloatingArmManipulator / FullyActuatedFloatingArmManipulator 略
}

// (2) 输入正则：纯 (x,u) 代价，进 costPtr。
problem.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

// (3) 末端跟踪：注意——它是一个【约束】被【软约束包装】成代价，进 stateSoftConstraintPtr。
problem.stateSoftConstraintPtr->add(
    "endEffector",
    getEndEffectorConstraint(pinocchioInterface, taskFile, "endEffector", /*useCaching*/ usePreComputation, ...));
problem.finalSoftConstraintPtr->add(
    "finalEndEffector",
    getEndEffectorConstraint(pinocchioInterface, taskFile, "finalEndEffector", usePreComputation, ...));

// (4) 自碰撞：纯状态软约束，进 stateSoftConstraintPtr。
problem.stateSoftConstraintPtr->add(
    "selfCollision",
    getSelfCollisionConstraint(pinocchioInterface, taskFile, urdfFile, "selfCollision", usePreComputation, ...));

// (5) 关节限位：状态-输入软约束（位置看 x，速度看 u），进 softConstraintPtr。
problem.softConstraintPtr->add(
    "jointLimits", getJointLimitSoftConstraint(pinocchioInterface, taskFile));

// (6) 共享预计算缓存。
problem.preComputationPtr.reset(
    new MobileManipulatorPreComputation(pinocchioInterface, modelInfo));
```

逐项对照一下，会发现几个和"朴素加权和"印象不同的地方，下面逐一拆开。

### 关键点一：末端跟踪是"约束"而非"代价"

最容易让初学者困惑的是第 (3) 步。明明 83.10 节把末端跟踪写成代价 $\ell_{ee}=\frac12 e^\top Q_{ee}e$，为什么源码里调的是 `getEndEffectorConstraint` 而不是 `getEndEffectorCost`？

答案是 OCS2 的一种**通用建模惯例**：先把"末端位姿误差"建模为一个**6 维约束函数** $g(x)=e(x)\in\mathbb{R}^6$（理想情况下希望它等于零），再用一个**惩罚函数**把这个约束"软化"成代价。具体地，`getEndEffectorConstraint` 内部做两件事：

1. 构造一个 `EndEffectorConstraint` 对象，它的 `getValue(x)` 返回 6 维 $SE(3)$ 误差 $e(x)$（位置 3 维 + 姿态 3 维）。
2. 用 `ocs2::StateSoftConstraint` 把这个约束包起来，每一维配一个 `ocs2::QuadraticPenalty`：位置 3 维用位置权重，姿态 3 维用姿态权重。

`QuadraticPenalty` 对单个约束分量 $h$ 的惩罚就是 $\frac12 \mu h^2$。把 6 维拼起来，正好还原成 $\frac12 e^\top Q_{ee} e$，其中 $Q_{ee}=\mathrm{diag}(\mu_1,...,\mu_6)$。

> **本质洞察**：在 OCS2 里，"代价"和"软约束"在数学上可以是同一个东西——一个约束函数 $h(x)$ 加一个惩罚 $p(h)$。`QuadraticPenalty` 让软约束退化成二次代价，`RelaxedBarrierPenalty` 让软约束退化成障碍代价。这种"约束 + 惩罚"的统一抽象，让你只需写一次约束函数 $h(x)$，就能在"硬约束 / 二次软约束 / 障碍软约束"之间自由切换，而不必重写代价类。

这解释了为什么 83.3 节文件表里**找不到** `EndEffectorCost.cpp`——真实仓库根本没有这个文件，末端跟踪走的是 `constraint/EndEffectorConstraint.h` + 核心库的 `StateSoftConstraint` + `QuadraticPenalty`。同理也没有独立的 `JointVelocityLimits.cpp`，关节限位统一由 `getJointLimitSoftConstraint` 产出。

> **常见陷阱（概念误区）**：照着旧博客找 `EndEffectorCost` 类。
> 现象：在仓库里 grep 不到，或找到的是早期 fork 的残留命名。
> 根本原因：把"教学里叫它代价"误当成"源码里有个叫 Cost 的类"。OCS2 用"约束 + 惩罚"实现代价。
> 正确做法：找末端跟踪就看 `EndEffectorConstraint` + `stateSoftConstraintPtr->add("endEffector", ...)`，权重在 `QuadraticPenalty` 的 $\mu$ 里。

### 关键点二：终端项与运行项用不同容器

第 (3) 步里 `endEffector` 进的是 `stateSoftConstraintPtr`（每个中间节点都算），而 `finalEndEffector` 进的是 `finalSoftConstraintPtr`（只在 horizon 末端 $t_N$ 算一次）。二者通常共享同一份 `EndEffectorConstraint` 逻辑，但**权重可以不同**。

为什么要分开？因为运行项和终端项的职责不同：

- 运行项的末端权重决定"路上是否一直贴着参考末端轨迹走"。权重太高会让末端在每个中间时刻都死贴参考，牺牲底盘和关节的自由度，反而抖动（对应 83.30 节末端抖动）。
- 终端项的末端权重决定"horizon 末端必须收敛到目标"。它通常设得比运行项更高，给优化器一个明确的"终点锚"。

> **反事实推理**：如果只有运行项、没有终端项会怎样？
> 在有限 horizon 下，优化器没有"必须在末端到达目标"的强约束，可能选择"全程缓慢逼近、末端仍差一截"的轨迹，因为这样运行代价更低。
> 加上终端项后，末端被强制锚定，整条轨迹被"拉直"向目标。
> 这就是为什么几乎所有 OCS2 示例都同时注册 intermediate 和 final 两个末端项。

### 关键点三：限位为什么进 `softConstraintPtr` 而不是 `stateSoftConstraintPtr`

第 (5) 步关节限位进的是 `softConstraintPtr`（状态-输入软约束），而不是纯状态的 `stateSoftConstraintPtr`。原因在 83.15、83.16 已埋下伏笔：**关节位置限位看状态 $q_a\subset x$，关节速度限位看输入 $\dot q_a\subset u$**。`getJointLimitSoftConstraint` 返回的是一个 `StateInputSoftConstraint`，内部用 `RelaxedBarrierPenalty`（而非 `QuadraticPenalty`）——因为限位是真正的安全边界，需要在接近边界时代价急剧上升，这正是 83.16.1 节 relaxed barrier 的用武之地。

把三种惩罚-容器搭配整理成一张表，是本节的核心记忆点：

| 项 | 约束函数 $h$ | 惩罚类 | 容器 | 依赖 |
|----|-------------|--------|------|------|
| 输入正则 | 无（直接是代价） | 无（`QuadraticInputCost` 直接给代价） | `costPtr` | $(x,u)$ |
| 末端跟踪（运行） | $SE(3)$ 误差 $e(x)$ | `QuadraticPenalty` $\times 6$ | `stateSoftConstraintPtr` | $x$ |
| 末端跟踪（终端） | $SE(3)$ 误差 $e(x)$ | `QuadraticPenalty` $\times 6$ | `finalSoftConstraintPtr` | $x(t_N)$ |
| 自碰撞 | $d_{ij}-d_{\min}$ | `RelaxedBarrierPenalty` | `stateSoftConstraintPtr` | $x$ |
| 关节位置+速度限位 | $q-q_{\min}$ 等 | `RelaxedBarrierPenalty` | `softConstraintPtr` | $(x,u)$ |

> **本质洞察**：选 `QuadraticPenalty` 还是 `RelaxedBarrierPenalty`，本质是回答"违反这一项是'不理想'还是'不允许'"。末端跟踪是不理想（可以暂时差一点），用二次惩罚；限位和碰撞是不允许（越界即危险），用障碍惩罚。容器（`cost`/`softConstraint`/`stateSoftConstraint`）则回答"这项依赖谁、什么时候算"。这两个正交维度组合，就覆盖了移动操作 OCP 的全部项。

### 与硬约束的对比

本例全部用软约束，没有用 `equalityConstraintPtr` 或硬不等式。这是 SQP-RTI 在线 MPC 的常见选择：软约束几乎总能求出解（最坏是约束被违反但有限代价），而硬约束在初值不可行时会直接判定 infeasible，导致 MPC 当周期无输出（对应 83.31 节约束不可行）。

| 处理方式 | 可行性鲁棒 | 约束满足精度 | 适用 |
|----------|-----------|-------------|------|
| 硬约束（`equalityConstraintPtr` / IPM 不等式） | 差（不可行即失败） | 精确 | 离线轨迹优化、安全关键 |
| 软约束 + `RelaxedBarrierPenalty` | 好 | 近似（取决于 $\mu,\delta$） | 在线 MPC（本章） |
| 软约束 + `QuadraticPenalty` | 最好 | 最松（只是偏好） | 跟踪目标、正则项 |

工程上的常见路线（呼应 83.21 调参顺序）：先全用 `QuadraticPenalty` 把跟踪调通，再把安全相关项换成 `RelaxedBarrierPenalty`，最后在必须精确的场合（如绝对不能撞的关节硬限位）才考虑硬约束或 IPM。

### 练习

**练习 19A-1（源码对照）**：到 `leggedrobotics/ocs2` 的 `ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`，找出 `endEffector`、`finalEndEffector`、`selfCollision`、`jointLimits`、`inputCost` 五个 `add` 调用，写出每个分别进了哪个容器、用了哪个惩罚类。与本节表格核对。

**练习 19A-2（惩罚切换实验）**：把末端跟踪的 `QuadraticPenalty` 在概念上换成 `RelaxedBarrierPenalty`，推导这会如何改变"末端离目标较远时"的梯度行为。提示：二次惩罚梯度随误差线性增长，障碍惩罚在远离边界时梯度趋近常数。哪种更适合"必须精确到达"的末端跟踪？

**练习 19A-3（容器归类）**：现在要新增一项"底盘朝向正则" $\ell_b=\frac12 w_\theta\,\mathrm{wrap}(\theta-\theta_{ref})^2$（来自 83.14）。它只依赖状态，是偏好而非安全。你会把它注册到哪个容器、用哪个惩罚？写出对应的 `add` 调用。

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

### 83.20.1 task.info 字段与 OCP 容器的对应 ⭐⭐⭐

上面的精简配置只是示意。真实 `ridgeback_ur5/task.info` 的结构和 §83.19A 的容器、惩罚是**一一对应**的——读懂这个对应，你就能从配置文件反推出 OCP 是怎么组装的，反之亦然。把关键块拆开看：

```ini
; ===== 末端跟踪（运行项）：对应 stateSoftConstraintPtr 的 "endEffector" =====
; 6 维 SE(3) 误差，每维一个 QuadraticPenalty，mu 即权重 Q_ee 的对角元。
endEffector
{
  muPosition      100.0     ; 位置 3 维的二次惩罚强度
  muOrientation   10.0      ; 姿态 3 维的二次惩罚强度
}

; ===== 末端跟踪（终端项）：对应 finalSoftConstraintPtr 的 "finalEndEffector" =====
; 通常比运行项更高，给 horizon 末端一个强"终点锚"（§83.19A 关键点二）。
finalEndEffector
{
  muPosition      100.0
  muOrientation   10.0
}

; ===== 输入正则：对应 costPtr 的 "inputCost"（QuadraticInputCost）=====
; R = diag(...)，前 2 维底盘速度，后 n_a 维臂关节速度。
inputCost
{
  R
  {
    scaling 1e-2
    (0,0)  5.0    ; v：底盘线速度权重
    (1,1)  5.0    ; omega：底盘角速度权重
    (2,2)  1.0    ; 臂关节 1 速度权重
    ; ... 其余臂关节
  }
}

; ===== 自碰撞：对应 stateSoftConstraintPtr 的 "selfCollision" =====
; 单个 RelaxedBarrierPenalty 作用在所有碰撞对上（§83.17.1）。
selfCollision
{
  mu              1e-2      ; barrier 强度（§83.16.1）
  delta           1e-3      ; 对数-二次切换点
  minimumDistance 0.05      ; d_min，安全距离（米）
  collisionObjectPairs  [ ... ]   ; 或 collisionLinkPairs，关心的 link/object 对
}

; ===== 关节限位：对应 softConstraintPtr 的 "jointLimits" =====
; 位置看状态、速度看输入，都用 RelaxedBarrierPenalty。
jointPositionLimits { mu 1e-2  delta 1e-3 }
jointVelocityLimits { mu 1e-2  delta 1e-3
  lowerBound [ ... ]   upperBound [ ... ] }   ; 含底盘 v、omega 和臂速度边界
```

逐块对照，能得到一张"配置 → 容器 → 惩罚"的总览：

| task.info 块 | OCP 容器 | 惩罚类 | 数学对应 |
|--------------|----------|--------|----------|
| `endEffector` | `stateSoftConstraintPtr` | `QuadraticPenalty` $\times 6$ | $\frac12 e^\top Q_{ee}e$（运行） |
| `finalEndEffector` | `finalSoftConstraintPtr` | `QuadraticPenalty` $\times 6$ | 终端末端代价 |
| `inputCost.R` | `costPtr` | 无（直接二次代价） | $\frac12 u^\top R u$ |
| `selfCollision` | `stateSoftConstraintPtr` | `RelaxedBarrierPenalty` | $\phi(d-d_{\min})$ |
| `jointPositionLimits` | `softConstraintPtr` | `RelaxedBarrierPenalty` | $\phi(q-q_{\min})$ 等 |
| `jointVelocityLimits` | `softConstraintPtr` | `RelaxedBarrierPenalty` | $\phi(\dot q-\dot q_{\min})$ 等 |

> **理论-工程桥接**：`endEffector` 块里的 `muPosition`/`muOrientation` 不是凭空的工程参数，它们**就是** §83.10 里 $Q_{ee}=\mathrm{diag}(q_x,...,q_{yaw})$ 的对角元，只不过被 `QuadraticPenalty` 以 $\frac12\mu h^2$ 的形式吃进去。改 `task.info` 等价于改 $Q_{ee}$——这就是为什么本章反复强调"调权重前先理解公式"。

> **常见陷阱（思维陷阱）**：把 `selfCollision.mu` 和 `endEffector.muPosition` 当成同一类参数同步调。
> 现象：调大碰撞 `mu` 想"更安全"，结果末端跟踪也被挤偏，或求解变难。
> 根本原因：二者惩罚类不同——`endEffector` 的 mu 是二次权重（线性梯度），`selfCollision` 的 mu 是 barrier 强度（边界附近梯度急升）。语义不可类比。
> 正确做法：碰撞安全主要调 `minimumDistance` 和 `delta`，跟踪精度主要调 `endEffector` 的 mu；分开调、分开记录。

> **常见陷阱（编程陷阱）**：`jointVelocityLimits` 的 `lowerBound`/`upperBound` 漏了底盘 $v,\omega$ 维度，只填了臂关节。
> 现象：底盘速度不受限，优化器输出过大 $v$ 或 $\omega$，仿真里底盘"窜出去"。
> 根本原因：输入向量是 $[v,\omega,\dot q_a]$，速度边界必须覆盖全部 $n_u$ 维，前 2 维是底盘。
> 正确做法：边界数组长度等于 `inputDim`，前 2 个对应底盘线/角速度（§83.15）。

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

> **反事实推理**：如果直接使用一年前的教程配置而不检查当前版本的 API 变化，最常见的错误是 `EndEffectorConstraint` 的构造函数签名变化、`getEndEffectorConstraint` 的参数顺序调整，以及 `task.info` 中配置字段的重命名。这类错误会在编译阶段暴露，但如果只改到能编译通过而不理解新参数含义，运行时行为可能与预期不同。

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

## 83.39A 从 mobile_manipulator 到近年 loco-manipulation 工作 ⭐⭐⭐

本章的 `mobile_manipulator` 是 OCS2 里"运动学 OCP"的入门样例。它故意把动力学砍到最简，好让你专注于状态/输入/代价/约束的组织。但 OCS2 这套 `OptimalControlProblem` 容器结构（§83.19A）真正的威力，在腿足 + 机械臂的**全身 loco-manipulation** 上才完全展开。把视野往这个方向延伸，能帮你理解"为什么要学这个简单样例"。

**主线一：从运动学到全身动力学。** 本章用 $\dot x=f(x,u)$ 的纯运动学 flow map，输入是速度。Sleiman 等人的 "Versatile multicontact planning and control for legged loco-manipulation"（Science Robotics, 2023，基于 OCS2）把同一套 OCP 结构用在 ANYmal + 机械臂上，状态升级到包含质心动量和接触力，动力学变成 centroidal/whole-body，约束加入摩擦锥和接触切换。但你会发现**组织方式没变**：还是 `dynamicsPtr` + 各种 cost/constraint 容器 + `PreComputation` 缓存运动学。本章学到的"约束 + 惩罚 + 容器分类"思维直接迁移。

| 维度 | 本章 mobile_manipulator | loco-manipulation（如 Sleiman 2023） |
|------|------------------------|--------------------------------------|
| 动力学 | 运动学速度模型 | centroidal / whole-body 动力学 |
| 输入 | $v,\omega,\dot q_a$（速度） | 接触力 + 关节速度/加速度 |
| 接触 | 无切换 | 多接触模式序列、接触切换 |
| 末端任务 | $SE(3)$ 跟踪（软约束） | $SE(3)$ 跟踪 + 力交互 |
| 共用结构 | `OptimalControlProblem` 容器 + 软约束 + barrier | 完全相同 |

**主线二：从已知地图到感知闭环。** 本章 §83.18 把外部障碍当成已知 ESDF。Grandia、Jenelten、Hutter 等人的 perceptive locomotion through whole-body MPC（arXiv:2305.08926）把感知直接接进 MPC：地形/障碍以 signed distance 形式实时进约束，配合 solver synchronized module 处理地图更新。这正是本章 §83.18 "地图同步比距离公式更容易出错"那句话在真实系统里的放大版。

**主线三：社区生态。** 围绕 OCS2 形成了若干开源全身控制框架，例如 `qiayuanl/legged_control`（NMPC + WBC + 状态估计 + sim2real）和类似的 `quad_mpc`。它们把 OCS2 的 MPC 输出接到全身控制器（WBC）做力矩跟踪——这恰好补上了本章运动学模型"不保证力矩可行"的短板（§83.6）：MPC 给运动学/质心层的参考，WBC 在更高频做动力学一致的力矩分配。

> **本质洞察**：`mobile_manipulator` 的"简单"是教学上的脚手架，不是 OCS2 的能力上限。它把 OCP 的**组织复杂度**（容器、软约束、惩罚、预计算）和**模型复杂度**（运动学 vs 全身动力学）解耦开，让你先掌握前者。一旦组织方式内化，换成 loco-manipulation 只是替换 `dynamicsPtr` 和增加几类约束。

> **本质洞察**：近年 loco-manipulation 的趋势是"MPC 负责中长时域的运动与接触规划，WBC/逆动力学负责高频力矩一致性"的分层。本章的运动学 MPC + 下层速度控制器（§83.23），正是这个分层思想在低速移动操作上的最简实例——理解它，再看 ANYmal 全身控制就只是每一层都变重了而已。

> **前沿延伸**：更近的工作（如 2025 年的 whole-body inverse dynamics MPC for legged loco-manipulation）尝试在 MPC 内部直接用全阶逆动力学优化关节力矩，把运动与力规划统一。这与本章"运动学 MPC + 下层控制器"是两种工程路线：前者更统一但更重，后者更轻但分层。先记住这个权衡，§83.6 的反事实推理已经预演了"把全动力学塞进 MPC"的代价。

---

## 83.39B 本章常见误解汇总

下表汇总本章涉及的高频误解。每条给出"错误认知 → 正确理解"，是 G4 教学完整性自检的快速复盘清单。

| # | 错误认知 | 正确理解 | 相关节 |
|---|----------|----------|--------|
| 1 | 末端跟踪由一个 `EndEffectorCost` 类实现 | 由 `EndEffectorConstraint` + `StateSoftConstraint` + `QuadraticPenalty` 实现，源码无 `EndEffectorCost` | §83.19A |
| 2 | 所有代价/约束加权求和塞进一个 cost | 分装进 `costPtr`/`softConstraintPtr`/`stateSoftConstraintPtr`/`finalSoftConstraintPtr` 等容器 | §83.19A |
| 3 | 运动学模型"太简单不实用" | 用低维换高频重规划，是低速移动操作的合理工程折中，且可与 WBC 分层 | §83.6, §83.39A |
| 4 | $SE(3)$ 误差用欧拉角差就行 | 欧拉角有奇异和缠绕，必须用 $\log(T_d^{-1}T)^\vee$ 的李代数误差 | §83.10 |
| 5 | $T_d^{-1}T$ 和 $T^{-1}T_d$ 等价 | 大姿态误差下残差和雅可比伴随项不同，全代码必须统一一种约定 | §83.10 |
| 6 | 末端权重越高跟踪越好 | 过高会抖动，应先加输入权重而非继续加末端权重 | §83.10, §83.30 |
| 7 | relaxed barrier 只是"对数障碍的小改" | 它保证约束被违反时仍有定义的有限代价和梯度，是 MPC 不崩溃的关键 | §83.16.1 |
| 8 | 限位用 `QuadraticPenalty` 就够 | 安全边界用 `RelaxedBarrierPenalty`，越界代价急升；二次惩罚太软 | §83.19A |
| 9 | 自碰撞难在距离公式 | 难在距离函数在穿透/最近特征切换处不光滑，以及碰撞对管理 | §83.17.1 |
| 10 | 碰撞对越多越安全 | 维度爆炸拖垮实时性，只保留几何上真可能相撞的少数对 | §83.17.1 |
| 11 | 忘了排除相邻连杆碰撞对没关系 | 相邻连杆恒接触会让 barrier 常驻、污染梯度 | §83.17.1 |
| 12 | RTI 每周期不收敛是 bug | RTI 故意每周期只做一次迭代，误差靠多周期逐步消除 | §83.22.1 |
| 13 | 只设运行末端项即可 | 缺终端项时优化器可能"全程缓慢逼近、末端差一截" | §83.19A |
| 14 | PreComputation 只是性能优化 | 它还保证所有 cost/constraint 在同一点一致线性化 | §83.19 |
| 15 | 末端到不了一定是权重问题 | 更可能是 frame 配错、坐标系错、误差约定不一致 | §83.28 |

---

## 83.39C API 速查表 ⭐⭐

下表为本章涉及的 OCS2 核心类型与方法签名，均以 `leggedrobotics/ocs2` 仓库 `main` 分支为准（精读时以实际版本头文件为准）。分四组：模型信息、动力学、代价/约束、求解器。

**模型信息与映射**（`ocs2_mobile_manipulator`）：

```cpp
// 模型类型枚举（ManipulatorModelInfo.h）
enum class ManipulatorModelType {
  DefaultManipulator = 0,                  // 固定/默认机械臂
  WheelBasedMobileManipulator = 1,         // 差速底盘 + 臂（本章重点）
  FloatingArmManipulator = 2,              // 浮动基座臂（欠驱动）
  FullyActuatedFloatingArmManipulator = 3  // 全驱动浮动基座臂
};

// 模型信息结构体字段（ManipulatorModelInfo.h）
struct ManipulatorModelInfo {
  ManipulatorModelType manipulatorModelType;  // 模型类型
  size_t stateDim;                             // 状态维度 n_x
  size_t inputDim;                             // 输入维度 n_u
  size_t armDim;                               // 臂关节数 n_a
  std::string baseFrame;                       // 底盘 frame 名
  std::string eeFrame;                         // 末端 frame 名
  std::vector<std::string> dofNames;           // 关节名顺序（验证顺序的关键）
};

// 状态/输入子块读取（AccessHelperFunctions.h）：从 x、u 切片取底盘位姿、臂关节
// 状态 <-> Pinocchio 广义坐标映射（MobileManipulatorPinocchioMapping.h）
```

**接口与组装**（`MobileManipulatorInterface.h`）：

```cpp
class MobileManipulatorInterface : public RobotInterface {
 public:
  MobileManipulatorInterface(const std::string& taskFile,
                             const std::string& libraryFolder,
                             const std::string& urdfFile);
  const OptimalControlProblem& getOptimalControlProblem() const override;
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override;
  const Initializer& getInitializer() const override;
  const RolloutBase& getRollout() const;
  const PinocchioInterface& getPinocchioInterface() const;
  const ManipulatorModelInfo& getManipulatorModelInfo() const;
  ddp::Settings& ddpSettings();   // SLQ/DDP 参数
  mpc::Settings& mpcSettings();   // MPC horizon、时间窗等
 private:
  // 组装各项的私有工厂方法（见 §83.19A）
  StateInputCost::Ptr getQuadraticInputCost(const std::string& taskFile);
  StateConstraint::Ptr getEndEffectorConstraint(const PinocchioInterface&, const std::string& taskFile,
                                                const std::string& prefix, bool useCaching, ...);
  StateConstraint::Ptr getSelfCollisionConstraint(const PinocchioInterface&, const std::string& taskFile,
                                                  const std::string& urdfFile, const std::string& prefix, ...);
  StateInputConstraint::Ptr getJointLimitSoftConstraint(const PinocchioInterface&, const std::string& taskFile);
};
```

**动力学**（`dynamics/`，均继承 `ocs2::SystemDynamicsBaseAD`）：

```cpp
WheelBasedMobileManipulatorDynamics              // 差速底盘 flow map（本章）
DefaultManipulatorDynamics                       // 固定臂
FloatingArmManipulatorDynamics                   // 浮动基座臂
FullyActuatedFloatingArmManipulatorDynamics      // 全驱动浮动基座臂
// 核心被覆写方法：
ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state,
                          const ad_vector_t& input, const ad_vector_t& parameters) const override;
```

**自碰撞几何与约束**（`ocs2_self_collision`）：

```cpp
class SelfCollision {                            // 解析路径
 public:
  SelfCollision(PinocchioGeometryInterface geometryInterface, scalar_t minimumDistance);
  size_t getNumCollisionPairs() const;
  vector_t getValue(const PinocchioInterface& pinocchioInterface) const;            // h = d - d_min
  std::pair<vector_t, matrix_t> getLinearApproximation(                              // (h, dh/dq)
      const PinocchioInterface& pinocchioInterface) const;
};
// SelfCollisionCppAd / SelfCollisionConstraintCppAd：自动微分路径
// PinocchioGeometryInterface：持有 GeometryModel，调 hpp-fcl/Coal 算最近距离
```

**OCP 容器与惩罚**（`ocs2_oc`、`ocs2_core`）：

```cpp
struct OptimalControlProblem {
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  std::unique_ptr<StateInputCostCollection> costPtr;             // add(name, term)
  std::unique_ptr<StateCostCollection> stateCostPtr;
  std::unique_ptr<StateInputCostCollection> softConstraintPtr;       // (x,u) 软约束（限位）
  std::unique_ptr<StateCostCollection> stateSoftConstraintPtr;       // x 软约束（末端、自碰撞）
  std::unique_ptr<StateCostCollection> finalSoftConstraintPtr;       // 终端 x 软约束
  std::unique_ptr<PreComputation> preComputationPtr;
  // 还有 finalCostPtr、equalityConstraintPtr、stateEqualityConstraintPtr 等
};
// 惩罚类（ocs2_core/penalties）：
QuadraticPenalty        //  p(h) = 0.5 * mu * h^2          —— 末端跟踪
RelaxedBarrierPenalty   //  对数障碍 + 二次延拓（§83.16.1）—— 限位、自碰撞
// 软约束包装：StateSoftConstraint、StateInputSoftConstraint
```

**求解器**（`ocs2_sqp`、`ocs2_ddp`）：

```cpp
// SQP（多重射击，§83.22.1）：ocs2_sqp 包
SqpSolver        // 求解器类（SqpSolver.h）
SqpMpc           // MPC 封装（SqpMpc.h）
SqpSettings      // 参数：sqpIteration、deltaTol、costTol、useFeedbackPolicy 等（SqpSettings.h）
// SLQ/DDP：ocs2_ddp 包
SLQ              // 连续时间 SLQ
ILQR             // 离散 iLQR
ddp::Settings    // maxNumIterations、minRelCost、constraintTolerance 等
```

> **使用提示**：`getEndEffectorConstraint` 既被 `stateSoftConstraintPtr->add("endEffector", ...)` 用作运行项，又被 `finalSoftConstraintPtr->add("finalEndEffector", ...)` 用作终端项，二者用不同的 `prefix` 读取各自的权重（§83.19A）。

---

## 83.39D 研究实践建议 ⭐⭐

按学习/工程目标分层给出建议：

**入门（先跑通再理解）**：

1. 先用仓库自带的 `ridgeback_ur5` 配置直接跑，看 RViz 里末端跟踪和预测轨迹，不要一开始改代码。
2. 改 `task.info` 里的末端位置权重和输入权重，观察底盘 vs 臂的分工变化（对应练习 C）。
3. 打开/关闭自碰撞约束，对比求解时间，建立"约束维度 ↔ 实时性"的直觉。

**进阶（读懂组织结构）**：

1. 在 `MobileManipulatorInterface.cpp` 里用断点或打印，确认每个 `add(name, ...)` 进的是哪个容器（对照 §83.19A 表）。
2. 把末端跟踪的 `QuadraticPenalty` 临时换成 `RelaxedBarrierPenalty`，观察"末端离目标远时"行为变化，理解两种惩罚的梯度差异。
3. 自己写一个最小的 `WheelBasedMobileManipulatorDynamics` 单元测试：固定 $(x,u)$，对比 `systemFlowMap` 输出与手推的 $f(x,u)$，再用数值差分核对自动微分雅可比（对应练习 A）。

**工程（适配新平台）**：

1. 严格走 §83.26 的 14 步清单，**先过单关节脉冲测试和零位 FK 比较**（§83.26.1），再碰权重。
2. 碰撞对从空集开始，逐对加入并验证零位 $d_{ij}>d_{\min}$，用 SRDF 排除相邻连杆（§83.17.1）。
3. 锁定 Pinocchio 和编译器版本，先单独编 `ocs2_core`、`ocs2_pinocchio_interface`、`ocs2_self_collision`，确认代码生成无误（§83.39）。
4. 在线 MPC 优先全软约束 + RTI；只在必须精确处才上硬约束或 IPM（§83.19A、§83.22）。

**研究（往全身/感知方向走）**：

1. 读 Sleiman 等 loco-manipulation 与 Grandia 等 perceptive WB-MPC，对照本章容器结构看"复杂性加在哪一层"（§83.39A）。
2. 把本章运动学 MPC 接一个下层 WBC，验证"MPC 给参考、WBC 保力矩一致"的分层（§83.6、§83.39A）。

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
| 关节顺序错位（末端位姿全错但不报错） | `dofNames` 顺序与 URDF/状态切片不一致 | 1. 打印 `ManipulatorModelInfo::dofNames` 2. 与 URDF joint 顺序逐项比对 3. 用 `AccessHelperFunctions` 复核状态切片 | 83.5, 83.26.1 |
| RViz 里预测轨迹与真机不同步 | MRT 策略时间戳与控制周期未对齐 | 1. 检查 `updatePolicy` 的查询时间 2. 确认 MRT 插值时间基准 3. 比较 MPC 发布频率与控制频率 | 83.23 |
| 自碰撞约束恒为激活（求解被拖死） | 零位下相邻连杆距离已小于 $d_{\min}$ | 1. 零位单独算各 pair 距离 2. 用 SRDF 排除相邻连杆 3. 调小 $d_{\min}$ 或剔除该 pair | 83.17.1 |

> **如何使用本表**：故障排查表覆盖了从"配置错"（frame/关节顺序）、"权重错"（绕圈/抖动）、"约束错"（不可行/恒激活）到"系统错"（求解超时/同步/编译）四类最常遇到的失败。诊断时建议按这个由浅入深的顺序排查：先用 Pinocchio 单独验证运动学（排除配置错），再看代价权重尺度（排除权重错），最后才动求解器和约束（排除系统错）。绝大多数"机器人行为诡异但程序不崩溃"的问题，根因都落在前两类——尤其是关节顺序与 frame 约定，这两处错了不会报错，却会让一切结果安静地失真。这正是 §83.26.1 把"单关节脉冲测试 + 零位 FK 比较"列为适配第一步的原因。

---

## 83.41 术语速查表

83.35 给的是"结论速查"（要记住的论断），本表给的是"术语速查"（名词的中英对照与一句话定义）。本章首次出现或反复使用的术语集中如下，便于回读源码时随手对照：

| 术语（中文） | English | 一句话定义 |
|------------|---------|----------|
| 移动操作 | Mobile Manipulation | 可移动底盘搭载机械臂、需同时协调底盘与臂完成末端任务 |
| 运动学最优控制 | Kinematic OCP | 以速度为输入、不显式建模力矩/惯量的最优控制问题 |
| 差速底盘 | Differential-drive Base | 状态 $[x,y,\theta]$、输入线速度+角速度 $[v,\omega]$ 的非完整底盘 |
| 末端执行器 | End-effector (EE) | 机械臂末端工具坐标系，跟踪任务的作用对象 |
| 末端位姿误差 | SE(3) Error | 用 $SE(3)$ 对数 $\log(T_{\text{ref}}^{-1}T)$ 表示的 6 维位姿误差 |
| 接口类 | `MobileManipulatorInterface` | 组装 OCP（动力学+代价+约束+求解设置）的总入口类 |
| 模型信息 | `ManipulatorModelInfo` | 保存模型类型、状态/输入/臂维度与关节名顺序的结构体 |
| 最优控制问题容器 | `OptimalControlProblem` | 聚合动力学、代价、软约束、预计算的 OCP 顶层结构 |
| 系统流映射 | `systemFlowMap` / flow map | 返回 $\dot{\mathbf{x}}=f(\mathbf{x},\mathbf{u})$ 的连续动力学函数 |
| 自动微分 | CppAD / AD | 用 CppAD 对 flow map、代价、约束生成解析导数并代码生成 |
| 关节正则 | Joint Regularization | 在零空间把冗余臂拉向参考位形的二次代价 |
| 软约束 | Soft Constraint | 以惩罚项形式进入代价、可被违反的约束（限位、末端、自碰撞） |
| 松弛障碍惩罚 | `RelaxedBarrierPenalty` | 对数障碍 + 二次延拓的惩罚，约束接近时梯度陡增 |
| 二次惩罚 | `QuadraticPenalty` | $p(h)=\tfrac12\mu h^2$ 形式的惩罚，用于末端跟踪 |
| 自碰撞 | Self-collision | 机器人连杆之间的碰撞，用最近距离 $d-d_{\min}$ 约束 |
| 几何接口 | `PinocchioGeometryInterface` | 持有几何模型、调用 hpp-fcl/Coal 计算最近距离 |
| 预计算 | `PreComputation` | 每步缓存共享的 FK 等中间量，避免代价/约束重复计算 |
| 多重射击 | Multiple Shooting | 把轨迹分段、各段独立积分并以连续性约束缝合的离散化 |
| 实时迭代 | RTI (Real-Time Iteration) | 每控制周期只做一次 SQP 迭代以保证实时性的策略 |
| 序列二次规划 | SQP | 在当前点线性化约束、二次近似代价反复求解 QP 的非线性求解法 |
| 内点法 | IPM (Interior Point Method) | 用障碍函数处理不等式约束的求解器，硬约束鲁棒性好 |
| 模型预测控制运行时 | MPC-MRT | MPC 求解线程与控制运行时（MRT）解耦的双线程架构 |
| 任务配置文件 | `task.info` | 把权重、限幅、求解设置等数学参数暴露给工程使用的配置入口 |
| 欧氏符号距离场 | ESDF | 存储到最近障碍有符号距离的体素场，供外部障碍约束查询 |

> **本质洞察**：通读这张术语表会发现一条暗线——表里几乎每个"数学概念"都对应一个"OCS2 类名"：末端误差 ↔ `EndEffectorConstraint`、软约束 ↔ `StateSoftConstraint`、自碰撞 ↔ `SelfCollision`、动力学 ↔ `SystemDynamicsBaseAD`。这正是 OCS2 设计哲学的体现：它把最优控制的数学结构（动力学 / 代价 / 约束 / 求解器）一对一地映射成 C++ 的类型层次。读懂这套映射，就读懂了 OCS2——你看到一个数学项，就知道它该进哪个容器、由哪个类实现；反过来看到一个类名，也能立刻说出它在 OCP 公式里对应哪一项。这种"数学—代码同构"的能力，比记住任何单个 API 都更有迁移价值。

---

**—— 本章终 ——**
