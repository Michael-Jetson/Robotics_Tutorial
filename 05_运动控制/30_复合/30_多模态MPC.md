# 第73章：多模态MPC——SE(3)末端跟踪与自碰撞约束

| 元信息 | 值 |
|--------|-----|
| 难度 | ⭐⭐⭐（MPC代价扩展 + Lie群误差 + 碰撞约束） |
| 预计时间 | 1.5 周（25-30 小时） |
| 前置依赖 | 足式/30_Pinocchio深度精读(Pinocchio CRTP), 足式/50_空间向量与浮动基座动力学(空间向量), 足式/60_QP_NLP建模(QP/NLP), 足式/110_OCS2完整栈与双线程MPC(OCS2 MPC栈), 复合/20_浮动基座臂统一动力学(统一动力学) |
| 下游章节 | 复合/40_RL全身控制基础(RL全身控制), 复合/120_底盘臂联合规划-85(移动操作应用) |

---

## 本章目标

学完本章后，你应能把足式 MPC 扩展为同时处理接触模式、末端 SE(3) 跟踪、自碰撞约束和操作接触力的多模态优化问题，并能判断统一 MPC、分层 MPC、WBC 接口与 RL/VLA 上层策略之间的工程边界。

## 73.0 前置自测

在开始本章前，请独立回答以下五个问题。如果有两个以上答不出，建议先回顾对应章节。

| # | 问题 | 来源 | 预期答案关键词 |
|---|------|------|---------------|
| 1 | OCS2 的双线程 MPC 架构中，solver thread 和 control thread 如何同步？MRT (Model Reference Tracking) 的作用是什么？ | 足式/110_OCS2完整栈与双线程MPC | Buffer + 线性插值、MRT将最优轨迹插值到控制频率 |
| 2 | $SE(3)$ 上的对数映射 $\text{Log}: SE(3) \to \mathfrak{se}(3)$ 的输出维度是多少？物理含义是什么？ | 足式/50_空间向量与浮动基座动力学 | 6维 $(\omega, v)$，等效旋转轴角+平移 |
| 3 | 一个 QP 中的不等式约束 $g(x) \geq 0$ 如何转化为 MPC 的 path constraint？写出 OCS2 的约束接口形式。 | 足式/60_QP_NLP建模/足式/110_OCS2完整栈与双线程MPC | `StateInputConstraint::getValue()` 返回 $h(x,u) \geq 0$ 向量 |
| 4 | 复合/20_浮动基座臂统一动力学 中 Go2+Z1 的状态向量维度 $n_v$ 是多少？写出 centroidal dynamics + full kinematics 的混合模型中状态如何分块。 | 复合/20_浮动基座臂统一动力学 | $n_v=24$；状态 $= [h_G(6), q_b(6), q_{leg}(12), q_{arm}(6)]$ |
| 5 | hpp-fcl（现 Coal）的 `distance()` 函数返回什么？如何获取最近点对的坐标？ | 足式/30_Pinocchio深度精读 | 返回 `DistanceResult`：最小距离标量 + 两个最近点的3D坐标 |

**自测标准**：
- 5/5 正确：直接阅读本章
- 3-4/5 正确：边读边查阅前置章节
- 0-2/5 正确：建议先完成 足式/110_OCS2完整栈与双线程MPC(OCS2) + 复合/20_浮动基座臂统一动力学(统一动力学)

---

## 73.1 从纯足式MPC到复合MPC——代价项的扩展 ⭐⭐

### 动机：为什么不能简单地把臂"加进去"？

足式/110_OCS2完整栈与双线程MPC 中我们学习了 OCS2 的 `legged_robot` example——一个经典的四足 MPC，其代价函数关注的是：**让机器人按照期望的 CoM 轨迹行走**。现在我们有了一条手臂，直觉上只需要"再加一个末端跟踪代价"就行了。

但现实要复杂得多：

| 纯足式 MPC 的关注点 | 复合 MPC 新增的挑战 |
|---------------------|-------------------|
| CoM 稳定性 | 臂运动产生的反作用力矩扰动 CoM |
| 步态时序（ModeSchedule） | 臂运动需要与步态协调（摆腿时臂别乱动） |
| 摩擦锥约束 | 臂受力通过 Jacobian 传递到基座再传到脚 |
| 12-DOF 求解 | 18-24 DOF 求解维度翻倍 |
| 无碰撞风险 | 臂可能撞到腿/身体 |

> **陷阱警告** ⚠️
> 直接在 OCS2 `legged_robot` 的 cost 中加一个 EE tracking term 而不调整权重，大概率导致：
> 1. MPC 为了让手到达目标位置而扭曲身体，破坏行走稳定性
> 2. SQP 不收敛（代价函数内部冲突）
> 3. 自碰撞（臂甩进腿的运动空间）
>
> 正确做法：**locomotion 安全 > manipulation 精度**，权重差两个数量级。

### 纯足式 OCS2 的代价结构

回顾 足式/110_OCS2完整栈与双线程MPC 中 `legged_robot` 的 MPC 代价：

$$
J_{legged} = \int_0^T \left[ \underbrace{\|\mathbf{h}_G - \mathbf{h}_G^{ref}\|_{\mathbf{Q}_h}^2}_{\text{CoM动量跟踪}} + \underbrace{\|\mathbf{q}_b - \mathbf{q}_b^{ref}\|_{\mathbf{Q}_b}^2}_{\text{基座姿态}} + \underbrace{\|\mathbf{q}_j - \mathbf{q}_j^{ref}\|_{\mathbf{Q}_j}^2}_{\text{关节正则化}} + \underbrace{\|\boldsymbol{\lambda}\|_{\mathbf{R}_\lambda}^2}_{\text{GRF最小化}} \right] dt
$$

其中各项的物理含义：

| 代价项 | 权重量级 | 物理含义 |
|--------|---------|---------|
| $\|\mathbf{h}_G - \mathbf{h}_G^{ref}\|^2$ | $10^3$ | 质心线/角动量跟踪指令速度 |
| $\|\mathbf{q}_b - \mathbf{q}_b^{ref}\|^2$ | $10^2$ | 基座高度和姿态保持水平 |
| $\|\mathbf{q}_j - \mathbf{q}_j^{ref}\|^2$ | $10^0$ | 关节回归默认站立构型 |
| $\|\boldsymbol{\lambda}\|^2$ | $10^{-3}$ | 最小化接触力（能量效率） |

### 复合 MPC 的代价扩展

加入手臂后，代价函数变为：

$$
J_{compound} = J_{legged} + \underbrace{\|\text{Log}(\mathbf{T}_{ee}^{-1} \cdot \mathbf{T}_{ref})\|_{\mathbf{Q}_{ee}}^2}_{\text{末端6D位姿跟踪}} + \underbrace{\|\mathbf{f}_{ee} - \mathbf{f}_{ref}\|_{\mathbf{Q}_f}^2}_{\text{末端力跟踪}} + \underbrace{\|\mathbf{q}_{arm} - \mathbf{q}_{arm}^{ref}\|_{\mathbf{Q}_a}^2}_{\text{臂关节正则化}}
$$

完整的权重设计策略：

| 代价项 | 权重量级 | 优先级 | 理由 |
|--------|---------|--------|------|
| CoM 动量 | $10^3$ | **最高** | 跌倒 = 任务失败 |
| 基座姿态 | $10^2$ | 高 | 保持行走稳定 |
| EE 位姿跟踪 | $10^1$ | **中** | 操作精度需求 |
| GRF 最小化 | $10^{-2}$ | 低 | 能效软优化 |
| 臂关节正则化 | $10^0$ | 低 | 避免奇异构型 |
| EE 力跟踪 | $10^0$ | 低 | 力控精度需求 |

> **设计原则**：locomotion 代价项的权重至少比 manipulation 代价项高 **一到两个数量级**。只有当机器人静止站立时，才可以提高 EE 跟踪权重。

### OCS2 中的代价实现接口

OCS2 的代价由 `StateCost`、`StateInputCost` 等接口组成：

```cpp
// ocs2_core/include/ocs2_core/cost/StateCost.h
class StateCost {
public:
    // 返回标量代价值 l(x, t)
    virtual scalar_t getValue(scalar_t time,
                              const vector_t& state,
                              const TargetTrajectories& targetTrajectories,
                              const PreComputation& preComp) const = 0;

    // 返回二次近似 (dfdx, dfdxx) 用于 SQP/iLQR
    virtual ScalarFunctionQuadraticApproximation
    getQuadraticApproximation(scalar_t time,
                             const vector_t& state,
                             const TargetTrajectories& targetTrajectories,
                             const PreComputation& preComp) const = 0;
};
```

复合 MPC 中需要新增的 cost term：

```cpp
// 伪代码：EndEffectorTrackingCost 继承 StateCost
class EndEffectorTrackingCost final : public StateCost {
public:
    scalar_t getValue(scalar_t time, const vector_t& state,
                      const TargetTrajectories& targets,
                      const PreComputation& preComp) const override {
        // 1. 从 state 提取关节角 q
        const auto q = centroidal_model::getGeneralizedCoordinates(state, info_);
        // 2. 正运动学得 T_ee
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, eeFrameIdx_);
        const auto T_ee = data_.oMf[eeFrameIdx_];  // SE(3)
        // 3. 从 targetTrajectories 插值得 T_ref
        const auto T_ref = interpolateEEPose(targets, time);
        // 4. SE(3) 对数映射误差
        const auto error = pinocchio::log6(T_ee.actInv(T_ref)).toVector();  // R^6
        // 5. 加权范数
        return 0.5 * error.transpose() * Q_ee_ * error;
    }
private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    matrix_t Q_ee_;  // 6x6 权重矩阵
    size_t eeFrameIdx_;
};
```

### 练习 73.1

| # | 练习 | 难度 |
|---|------|------|
| 1 | 如果将 EE 跟踪权重从 $10^1$ 提高到 $10^4$（与 CoM 同量级），用 MuJoCo 仿真预测会发生什么？画出可能的失败模式。 | ⭐⭐ |
| 2 | 设计一个**自适应权重**策略：当机器人站立时 $Q_{ee} = 10^2$，行走时 $Q_{ee} = 10^0$。写出基于步态相位的权重调度公式。 | ⭐⭐⭐ |

---

## 73.2 SE(3)末端跟踪代价——Lie群对数映射 ⭐⭐⭐

### 动机：为什么欧拉角不行？

在经典机器人学中，末端跟踪通常分解为"位置误差 + 姿态误差"：

$$
e_{naive} = \begin{bmatrix} \mathbf{p}_{ee} - \mathbf{p}_{ref} \\ \text{EulerAngle}(\mathbf{R}_{ee}) - \text{EulerAngle}(\mathbf{R}_{ref}) \end{bmatrix} \in \mathbb{R}^6
$$

这种方法有三个致命问题：

| 问题 | 表现 | 后果 |
|------|------|------|
| 万向锁（Gimbal Lock） | pitch = $\pm 90°$ 时失去一个自由度 | MPC 在特定姿态无法求解 |
| 表示不连续 | 绕 z 轴转过 $360°$ 时误差跳变 | SQP 线性化产生错误梯度 |
| 非最短路径 | 欧拉角差 ≠ 实际最短旋转 | 末端走"冤枉路" |

> **陷阱警告** ⚠️
> 四元数虽然避免了万向锁，但四元数差 $\mathbf{q}_{err} = \mathbf{q}_{ref}^{-1} \otimes \mathbf{q}_{ee}$ 仍然有**双覆盖**问题（$\mathbf{q}$ 和 $-\mathbf{q}$ 代表同一旋转），在 MPC 中会导致"抖动"。正确的方法是使用 **Lie 群对数映射**。

### SE(3) 误差的正确定义

$SE(3)$ 是刚体运动群（Special Euclidean Group），其元素是 $4 \times 4$ 齐次变换矩阵：

$$
\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0}^\top & 1 \end{bmatrix} \in SE(3), \quad \mathbf{R} \in SO(3), \; \mathbf{p} \in \mathbb{R}^3
$$

两个位姿之间的误差定义为**左不变误差**（left-invariant error）：

$$
\mathbf{T}_{err} = \mathbf{T}_{ee}^{-1} \cdot \mathbf{T}_{ref} \in SE(3)
$$

然后通过对数映射投影到李代数 $\mathfrak{se}(3) \cong \mathbb{R}^6$：

$$
\boldsymbol{\xi}_{err} = \text{Log}(\mathbf{T}_{err})^{\vee} = \text{Log}(\mathbf{T}_{ee}^{-1} \cdot \mathbf{T}_{ref})^{\vee} \in \mathbb{R}^6
$$

其中 $\boldsymbol{\xi} = (\boldsymbol{\omega}, \mathbf{v})^\top$，$\boldsymbol{\omega} \in \mathbb{R}^3$ 是等效旋转轴角，$\mathbf{v} \in \mathbb{R}^3$ 是耦合平移。

### SE(3) 对数映射的完整推导

**Step 1：SO(3) 对数映射**

对于旋转矩阵 $\mathbf{R} \in SO(3)$，对数映射为：

$$
\text{Log}(\mathbf{R}) = \theta \cdot \hat{\mathbf{a}} \in \mathfrak{so}(3)
$$

其中旋转角 $\theta$ 和旋转轴 $\mathbf{a}$ 通过以下方式计算：

$$
\theta = \arccos\left(\frac{\text{tr}(\mathbf{R}) - 1}{2}\right), \quad \hat{\mathbf{a}} = \frac{\mathbf{R} - \mathbf{R}^\top}{2\sin\theta}
$$

这里 $\hat{\mathbf{a}}$ 是 $\mathbf{a}$ 的反对称矩阵形式（hat operator）。

| 特殊情况 | 条件 | 处理方式 |
|---------|------|---------|
| $\theta \approx 0$ | $\text{tr}(\mathbf{R}) \approx 3$ | Taylor展开：$\text{Log}(\mathbf{R}) \approx (\mathbf{R} - \mathbf{R}^\top)/2$ |
| $\theta \approx \pi$ | $\text{tr}(\mathbf{R}) \approx -1$ | 从 $\mathbf{R}$ 的列向量提取轴 |

**Step 2：SE(3) 对数映射**

对于 $\mathbf{T} = (\mathbf{R}, \mathbf{p})$：

$$
\text{Log}(\mathbf{T}) = \begin{bmatrix} \hat{\boldsymbol{\omega}} & \mathbf{v} \\ \mathbf{0}^\top & 0 \end{bmatrix} \in \mathfrak{se}(3)
$$

其中 $\hat{\boldsymbol{\omega}} = \text{Log}(\mathbf{R})$，平移部分通过 **Jacobian 逆** 计算：

$$
\mathbf{v} = \mathbf{J}^{-1}(\boldsymbol{\omega}) \cdot \mathbf{p}
$$

左 Jacobian 的逆为：

$$
\mathbf{J}^{-1}(\boldsymbol{\omega}) = \mathbf{I} - \frac{1}{2}\hat{\boldsymbol{\omega}} + \frac{1}{\theta^2}\left(1 - \frac{\theta \sin\theta}{2(1-\cos\theta)}\right)\hat{\boldsymbol{\omega}}^2
$$

> **关键洞察**：SE(3) 的对数映射不是简单地"分别取旋转和平移的log"。平移部分 $\mathbf{v}$ 通过 $\mathbf{J}^{-1}$ 与旋转**耦合**。这意味着当旋转误差很大时，平移误差的方向也会改变。

### EE 跟踪代价的最终形式

$$
J_{ee} = \frac{1}{2} \|\text{Log}(\mathbf{T}_{ee}^{-1} \cdot \mathbf{T}_{ref})^{\vee}\|_{\mathbf{W}}^2 = \frac{1}{2} \boldsymbol{\xi}_{err}^\top \mathbf{W} \boldsymbol{\xi}_{err}
$$

权重矩阵 $\mathbf{W} \in \mathbb{R}^{6 \times 6}$ 通常是对角的：

$$
\mathbf{W} = \text{diag}(w_{\omega_x}, w_{\omega_y}, w_{\omega_z}, w_{v_x}, w_{v_y}, w_{v_z})
$$

| 典型任务 | 旋转权重 | 平移权重 | 理由 |
|---------|---------|---------|------|
| 抓取（精确对位） | $10^2$ | $10^2$ | 旋转平移同样重要 |
| 指向（pointing） | $10^2$ | $10^0$ | 只关心方向 |
| 到达（reaching） | $10^0$ | $10^2$ | 只关心位置 |
| 擦拭（wiping） | $10^1$ | $10^2$ | 法向重要，切向不重要 |

### Jacobian 推导：代价对关节角的梯度

MPC 的 SQP/iLQR 需要代价的梯度和 Hessian。关键链式法则：

$$
\frac{\partial J_{ee}}{\partial \mathbf{q}} = \boldsymbol{\xi}_{err}^\top \mathbf{W} \cdot \frac{\partial \boldsymbol{\xi}_{err}}{\partial \mathbf{q}}
$$

其中误差的 Jacobian 为：

$$
\frac{\partial \boldsymbol{\xi}_{err}}{\partial \mathbf{q}} = \mathbf{J}_{log} \cdot \mathbf{J}_{FK}
$$

- $\mathbf{J}_{FK} = \frac{\partial \mathbf{T}_{ee}}{\partial \mathbf{q}} \in \mathbb{R}^{6 \times n_v}$：正运动学 Jacobian（Pinocchio 的 `computeFrameJacobian`）
- $\mathbf{J}_{log} = \frac{\partial \text{Log}(\mathbf{T})}{\partial \mathbf{T}} \in \mathbb{R}^{6 \times 6}$：对数映射的 Jacobian（Pinocchio 的 `Jlog6`）

完整的梯度计算链：

$$
\frac{\partial J_{ee}}{\partial \mathbf{q}} = \boldsymbol{\xi}_{err}^\top \mathbf{W} \cdot \underbrace{\mathbf{J}_{log}(\mathbf{T}_{err})}_{\text{Jlog6}} \cdot \underbrace{\text{Ad}(\mathbf{T}_{ref}^{-1})}_{\text{坐标变换}} \cdot \underbrace{\mathbf{J}_{ee}(\mathbf{q})}_{\text{FK Jacobian}}
$$

### Pinocchio API 实现

```cpp
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/log.hpp>     // log6(), Jlog6()
#include <pinocchio/spatial/explog.hpp>  // 对数/指数映射

// ========== SE(3) 误差计算 ==========
// 输入: model, data, q (关节角), T_ref (目标位姿), eeFrameId
// 输出: error (6维), Jacobian (6 x nv)

void computeEETrackingError(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const pinocchio::SE3& T_ref,
    const pinocchio::FrameIndex eeFrameId,
    Eigen::Matrix<double, 6, 1>& error,      // 输出：误差向量
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J_error)  // 输出：误差Jacobian
{
    // Step 1: 正运动学
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, eeFrameId);
    const pinocchio::SE3& T_ee = data.oMf[eeFrameId];

    // Step 2: SE(3) 误差 T_err = T_ee^{-1} * T_ref（左不变误差）
    const pinocchio::SE3 T_err = T_ee.actInv(T_ref);

    // Step 3: 对数映射 -> 6维误差向量
    error = pinocchio::log6(T_err).toVector();  // [omega(3), v(3)]

    // Step 4: 对数映射的 Jacobian (Jlog6)
    Eigen::Matrix<double, 6, 6> J_log;
    pinocchio::Jlog6(T_err, J_log);  // d_log(T_err)/d_T_err

    // Step 5: FK Jacobian（body frame）
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_fk(6, model.nv);
    J_fk.setZero();
    pinocchio::computeFrameJacobian(model, data, q, eeFrameId,
                                     pinocchio::LOCAL, J_fk);

    // Step 6: 链式法则 = Jlog * (-J_fk)
    // 注意负号：误差定义为 T_ee^{-1} * T_ref，T_ee 增大时误差减小
    J_error = -J_log * J_fk;
}
```

> **实现细节** ⚠️
> Pinocchio 的 `Jlog6` 返回的是**右 Jacobian** $\mathbf{J}_r$，而非左 Jacobian。在 OCS2 的实际代码中（`PinocchioEndEffectorKinematicsCppAd.cpp`），这一步通过 CppAD 自动求导绕过了手动推导。但理解手动推导对调试至关重要。

### 位置-only vs SE(3) 跟踪的对比实验

| 指标 | 位置-only ($\|\mathbf{p}_{ee} - \mathbf{p}_{ref}\|^2$) | SE(3) 完整跟踪 |
|------|-------------------------------------------------------|---------------|
| 计算量 | 3维误差，$O(n_v)$ Jacobian | 6维误差，需要 Jlog6 |
| 方向精度 | 无保证（夹爪可能朝错方向） | 完全约束方向 |
| 奇异性 | 无（平移误差总是良定义的） | $\theta = \pi$ 时 Jlog6 有奇异 |
| 适用场景 | reaching、粗抓取 | 精确对位、插入、工具使用 |
| MPC 计算增量 | 基线 | +15% |

```cpp
// OCS2 中仅位置跟踪的简化版本（对比参考）
scalar_t positionOnlyCost(const vector_t& state) {
    // 仅需前三维误差
    Eigen::Vector3d p_ee = computeFK_position(state);
    Eigen::Vector3d p_ref = getReference_position(time);
    Eigen::Vector3d error = p_ee - p_ref;
    return 0.5 * error.transpose() * Q_pos_ * error;  // Q_pos: 3x3
}
```

### 练习 73.2

| # | 练习 | 难度 |
|---|------|------|
| 1 | 用 Pinocchio Python API 实现 SE(3) 误差计算：给定 Go2+Z1 的 URDF，令末端跟踪一个绕 z 轴旋转的圆形轨迹。绘制 6 维误差随时间的变化曲线。 | ⭐⭐ |
| 2 | 当 $\theta_{err} \to \pi$ 时，$\text{Jlog6}$ 的条件数会怎样变化？设计一个数值安全的 clamp 策略。 | ⭐⭐⭐ |
| 3 | 推导：如果用**右不变误差** $\mathbf{T}_{ref} \cdot \mathbf{T}_{ee}^{-1}$ 代替左不变误差 $\mathbf{T}_{ee}^{-1} \cdot \mathbf{T}_{ref}$，代价的物理含义有何不同？哪个更适合 MPC？ | ⭐⭐⭐ |

---

## 73.3 自碰撞约束 ⭐⭐⭐

### 动机：为什么复合机器人必须考虑自碰撞？

纯四足机器人的腿部运动空间是**预定义的**——正常步态中，各腿的工作空间互不重叠。但加入机械臂后：

| 碰撞场景 | 发生条件 | 危险程度 |
|---------|---------|---------|
| 手臂末端撞击前腿 | 手臂向下抓取物体 | 高（可能绊倒） |
| 肘关节撞击身体 | 手臂折叠回收 | 中（卡住） |
| 手臂撞击后腿 | 手臂向后传递物体 | 高（破坏步态） |
| 夹爪撞击地面 | 基座晃动 + 手臂下垂 | 中（损坏夹爪） |

在仿真中不考虑自碰撞只是"不真实"，但在实机上：**一次自碰撞就可能损坏价值 5 万元的机械臂关节模组**。

### 约束的数学形式

对于每对可能碰撞的 link 对 $(i, j)$，定义安全距离约束：

$$
d(\text{link}_i, \text{link}_j) \geq d_{safe} \quad \forall (i, j) \in \mathcal{C}
$$

其中 $\mathcal{C}$ 是碰撞检测对集合，$d(\cdot, \cdot)$ 是两个几何体之间的最小距离。

转化为 OCS2 的不等式约束形式：

$$
h_{col}(\mathbf{q}) = d(\text{link}_i(\mathbf{q}), \text{link}_j(\mathbf{q})) - d_{safe} \geq 0
$$

### hpp-fcl (Coal) 距离计算

hpp-fcl（现已更名为 Coal）是 Pinocchio 生态系统的碰撞检测库。核心 API：

```cpp
#include <hpp/fcl/distance.h>              // 注意：hpp-fcl 已更名为 Coal，
#include <hpp/fcl/shape/geometric_shapes.h>  // 较新版本应使用 #include <coal/distance.h>
#include <pinocchio/algorithm/geometry.hpp>   // 和 #include <coal/shape/geometric_shapes.h>

// ========== 碰撞对距离计算 ==========
// 方式1：使用 Pinocchio 的 GeometryModel（推荐）
void computeCollisionDistances(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const pinocchio::GeometryModel& geom_model,
    pinocchio::GeometryData& geom_data,
    const Eigen::VectorXd& q)
{
    // 更新所有frame的位姿
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

    // 计算所有碰撞对的距离
    pinocchio::computeDistances(geom_model, geom_data);

    // 遍历结果
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k) {
        const auto& pair = geom_model.collisionPairs[k];
        const auto& result = geom_data.distanceResults[k];

        double distance = result.min_distance;       // 最小距离
        Eigen::Vector3d p1 = result.nearest_points[0]; // link_i 上最近点
        Eigen::Vector3d p2 = result.nearest_points[1]; // link_j 上最近点

        // 约束值：distance - d_safe >= 0
        double constraint_value = distance - d_safe_;

        if (constraint_value < 0) {
            // 碰撞！在MPC中这应该被约束阻止
            std::cerr << "碰撞对 " << pair.first << "-" << pair.second
                      << " 距离 = " << distance << " < " << d_safe_ << std::endl;
        }
    }
}
```

### 距离梯度：$\frac{\partial d}{\partial \mathbf{q}}$ 的计算

MPC 的 SQP 线性化需要约束的 Jacobian。距离函数的梯度可以解析求得：

$$
\frac{\partial d}{\partial \mathbf{q}} = \hat{\mathbf{n}}^\top \left( \mathbf{J}_{p_2}(\mathbf{q}) - \mathbf{J}_{p_1}(\mathbf{q}) \right)
$$

其中：
- $\hat{\mathbf{n}} = \frac{\mathbf{p}_2 - \mathbf{p}_1}{\|\mathbf{p}_2 - \mathbf{p}_1\|}$ 是最近点连线方向的单位法向量
- $\mathbf{J}_{p_k}(\mathbf{q}) \in \mathbb{R}^{3 \times n_v}$ 是最近点 $p_k$ 相对于关节角 $\mathbf{q}$ 的位置 Jacobian

```cpp
// ========== 距离梯度计算 ==========
Eigen::VectorXd computeDistanceGradient(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const pinocchio::FrameIndex frame_i,  // link_i 的 frame
    const pinocchio::FrameIndex frame_j,  // link_j 的 frame
    const Eigen::Vector3d& p1_world,      // link_i 上最近点（世界坐标）
    const Eigen::Vector3d& p2_world)      // link_j 上最近点（世界坐标）
{
    const int nv = model.nv;

    // 法向量（从 p1 指向 p2）
    Eigen::Vector3d n = (p2_world - p1_world).normalized();

    // 计算 p1 的 Jacobian（将 frame Jacobian 转换到点 p1）
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_frame_i(6, nv);
    J_frame_i.setZero();
    pinocchio::computeFrameJacobian(model, data, q, frame_i,
                                     pinocchio::WORLD, J_frame_i);
    // p1 相对于 frame_i 原点的偏移
    Eigen::Vector3d offset_i = p1_world - data.oMf[frame_i].translation();
    // 点 Jacobian: J_p = J_v + [omega] x offset = J_v - hat(offset) * J_w
    Eigen::Matrix3d hat_offset_i = pinocchio::skew(offset_i);
    Eigen::MatrixXd J_p1 = J_frame_i.topRows(3) - hat_offset_i * J_frame_i.bottomRows(3);

    // 对 p2 同理
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_frame_j(6, nv);
    J_frame_j.setZero();
    pinocchio::computeFrameJacobian(model, data, q, frame_j,
                                     pinocchio::WORLD, J_frame_j);
    Eigen::Vector3d offset_j = p2_world - data.oMf[frame_j].translation();
    Eigen::Matrix3d hat_offset_j = pinocchio::skew(offset_j);
    Eigen::MatrixXd J_p2 = J_frame_j.topRows(3) - hat_offset_j * J_frame_j.bottomRows(3);

    // dd/dq = n^T (J_p2 - J_p1)
    return n.transpose() * (J_p2 - J_p1);  // 1 x nv -> 转置后 nv x 1
}
```

### 碰撞对选择策略

并非所有 link 对都需要检查碰撞。Go2+Z1 有约 20 个 link，$C(20,2)=190$ 对——在 MPC 的每一步全部计算是不现实的。

| 策略 | 碰撞对数 | 计算时间 | 安全性 |
|------|---------|---------|--------|
| 全对检查 | 190 | ~5ms | 完全安全 |
| 手动选择关键对 | 10-15 | ~0.3ms | 覆盖主要风险 |
| 层次化过滤（AABB先验） | ~20（动态） | ~0.5ms | 安全 |
| 仅相邻关节跳过 | ~100 | ~2.5ms | 安全（冗余） |

推荐的碰撞对选择（Go2+Z1）：

```yaml
# task.info 中的碰撞对配置
selfCollision:
  activate: true
  minimumDistance: 0.05  # 5cm 安全距离
  pairs:
    # 臂 link 与腿的碰撞对
    - [arm_link3, FL_thigh]   # 肘关节 vs 左前大腿
    - [arm_link3, FR_thigh]   # 肘关节 vs 右前大腿
    - [arm_link4, FL_thigh]   # 前臂 vs 左前大腿
    - [arm_link4, FR_thigh]   # 前臂 vs 右前大腿
    - [arm_link5, FL_calf]    # 腕部 vs 左前小腿
    - [arm_link5, FR_calf]    # 腕部 vs 右前小腿
    # 臂 link 与身体的碰撞对
    - [arm_link3, base_link]  # 肘关节 vs 身体
    - [arm_link5, base_link]  # 腕部 vs 身体
    # 末端与地面
    - [gripper_link, ground_plane]  # 夹爪 vs 地面
```

### 软约束 vs 硬约束

在 MPC 中，碰撞约束可以用三种方式实现：

| 方法 | 形式 | 优点 | 缺点 |
|------|------|------|------|
| **硬约束** | $h(\mathbf{q}) \geq 0$ 作为 QP 不等式 | 严格保证安全 | QP 可能无解（infeasible） |
| **罚函数** | $J_{col} = \mu \cdot \max(0, d_{safe} - d)^2$ | 总有解 | 可能轻微违反约束 |
| **对数障碍** | $J_{bar} = -\mu \ln(d - d_{safe})$ | 光滑，梯度好 | $d \to d_{safe}$ 时梯度爆炸 |
| **松弛障碍** | $J_{relax} = \mu \cdot \text{huber}(d_{safe} - d)$ | 兼顾光滑和有界梯度 | 参数 $\mu$ 需要调 |

OCS2 默认使用**松弛障碍**（relaxed barrier）：

$$
B_\mu(h) = \begin{cases}
-\mu \ln(h) & \text{if } h > \delta \\
\frac{\mu}{2}\left(\frac{(h - 2\delta)^2}{\delta^2} - 1\right) - \mu \ln(\delta) & \text{if } h \leq \delta
\end{cases}
$$

其中 $\delta$ 是光滑化参数，确保当约束活跃时梯度不会趋向无穷。

```cpp
// OCS2 中添加自碰撞约束
#include <ocs2_pinocchio_interface/PinocchioEndEffectorConstraintCppAd.h>

// 在 OCP 设置中注册碰撞约束
void MobileManipulatorInterface::setupCollisionConstraints(
    ocs2::OptimalControlProblem& ocp) {
    // 创建碰撞约束对象
    auto collisionConstraint = std::make_unique<
        ocs2::SelfCollisionConstraintCppAd>(
            pinocchioInterface_,       // Pinocchio 模型
            collisionPairs_,           // 碰撞对列表
            minimumDistance_,           // d_safe = 0.05m
            "selfCollision"            // 约束名称
    );

    // 注册为 inequality constraint (softened by relaxed barrier)
    ocp.inequalityConstraintPtr->add(
        "selfCollision",
        std::move(collisionConstraint)
    );
}
```

### 计算性能分析

| 操作 | 单次耗时 | 每MPC步总耗时（10对，20节点） |
|------|---------|---------------------------|
| `updateGeometryPlacements` | 15 us | 0.3 ms |
| `computeDistance`（单对） | 2-10 us | 0.04-0.2 ms |
| 梯度计算（单对） | 5 us | 0.1 ms |
| **总碰撞处理** | — | **~0.5 ms** |

结论：碰撞约束增加约 **10-15%** 的 MPC 求解时间，完全可接受。

### 练习 73.3

| # | 练习 | 难度 |
|---|------|------|
| 1 | 在 Pinocchio Python 中加载 Go2+Z1 的 URDF + collision meshes，用 `pin.computeDistances()` 计算默认站立构型下所有碰撞对的距离。哪些对距离最小？ | ⭐⭐ |
| 2 | 实验对比：硬约束 vs 罚函数 vs 松弛障碍在"手臂穿越前腿工作空间"场景下的行为差异。记录约束违反量和求解时间。 | ⭐⭐⭐ |
| 3 | 当机器人行走时，腿的位置在变化。碰撞对的最近点也在变化。这是否导致梯度不连续？如何处理？ | ⭐⭐⭐ |

---

## 73.4 OCS2 mobile_manipulator 模板精读 ⭐⭐⭐

### 动机：从零开始还是从模板出发？

OCS2 提供了两个相关的 example：
- `ocs2_legged_robot`：四足 MPC（centroidal dynamics + switched system）
- `ocs2_mobile_manipulator`：移动底座 + 机械臂（kinematic model）

复合机器人（四足+臂）的 MPC 实际上是两者的**融合**：

| 组件 | 来自 `legged_robot` | 来自 `mobile_manipulator` | 新增 |
|------|--------------------|-----------------------------|------|
| 动力学模型 | centroidal dynamics | — | 合并臂关节 |
| 步态调度 | ModeSchedule | — | 不变 |
| 接触力决策 | GRF 作为输入 | — | 不变 |
| EE 位姿跟踪 | — | SE(3) cost | 权重调低 |
| 自碰撞 | — | collision constraint | 新增臂-腿对 |
| 状态维度 | $n_v = 18$ | $n_v = 9$ | $n_v = 24$ |

### 文件结构对比

```text
ocs2_mobile_manipulator/
├── config/
│   └── mpc/
│       ├── task.info           # 代价权重、约束参数、求解器设置
│       └── reference.info      # 参考轨迹参数
├── include/
│   └── ocs2_mobile_manipulator/
│       ├── MobileManipulatorInterface.h    # OCP 构造入口
│       ├── MobileManipulatorPreComputation.h  # 预计算缓存
│       ├── dynamics/
│       │   └── MobileManipulatorDynamics.h    # 运动学模型
│       └── cost/
│           └── EndEffectorCost.h              # EE 跟踪代价
├── src/
│   ├── MobileManipulatorInterface.cpp   # 组装 OCP
│   └── MobileManipulatorDynamics.cpp    # 动力学实现
├── urdf/
│   └── mobile_manipulator.urdf          # 机器人描述
└── launch/
    └── mobile_manipulator.launch        # ROS 启动
```

### task.info 关键配置解析

```ini
; ========== OCS2 mobile_manipulator task.info ==========
; 核心 MPC 参数

[model_information]
  manipulatorModelType = 0     ; 0=默认, 1=浮动基座
  ; URDF 中的 end-effector frame 名称
  endEffectorFrameName = "tool0"
  ; 基座类型: 0=fixed, 1=planar(x,y,yaw), 2=6DOF
  baseType = 2

[mpc]
  timeHorizon = 1.0           ; MPC 预测时域 T = 1.0s
  numPartitions = 1           ; SQP 时间分区数
  ; SQP 迭代次数（实时性 vs 最优性权衡）
  sqpIteration = 1            ; RTI: 仅1次迭代
  ; 离散化步长
  dt = 0.05                   ; 50ms -> 20个决策节点/秒时域

[cost]
  ; 末端位姿跟踪权重 [omega_x, omega_y, omega_z, v_x, v_y, v_z]
  endEffectorWeight = [100, 100, 100, 200, 200, 200]
  ; 关节正则化权重
  jointPositionWeight = [1, 1, 1, 1, 1, 1]
  ; 输入（关节速度）正则化
  inputWeight = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

[constraint]
  ; 自碰撞约束
  selfCollision.activate = true
  selfCollision.minimumDistance = 0.05
  selfCollision.relaxedBarrierMu = 1e-2
  selfCollision.relaxedBarrierDelta = 1e-3
  ; 关节限位约束
  jointLimits.activate = true
  jointLimits.relaxedBarrierMu = 1e-2

[solver]
  ; SQP-RTI 设置
  projectStateInputEqualityConstraints = true
  printSolverStatistics = true
  useFeedbackPolicy = true
```

### 从 legged_robot 到复合 MPC 的修改路径

将 OCS2 `legged_robot` 改造为四足+臂的复合 MPC，需要以下**最小修改**：

**Step 1：URDF 扩展**

```xml
<!-- 在 legged_robot URDF 的 base_link 上添加臂 -->
<joint name="arm_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base_link"/>
    <origin xyz="0.3 0.0 0.05" rpy="0 0 0"/>
</joint>

<!-- Z1 机械臂的 6 个旋转关节 -->
<joint name="arm_joint1" type="revolute">
    <parent link="arm_base_link"/>
    <child link="arm_link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.618" upper="2.618" effort="33" velocity="3.14"/>
</joint>
<!-- ... arm_joint2 ~ arm_joint6 类似 ... -->
```

**Step 2：frame_declaration 添加 EE frame**

```cpp
// 在 Interface 的初始化中注册末端 frame
void CompoundMPCInterface::initialize(const std::string& urdfFile) {
    // 加载模型
    pinocchioInterface_.reset(new PinocchioInterface(
        buildModelFromUrdf(urdfFile, JointModelFreeFlyer())));

    // 注册 end-effector frame（从 URDF 中查找）
    eeFrameIdx_ = pinocchioInterface_->getModel().getFrameId("gripper_link");

    // 验证 frame 存在
    if (eeFrameIdx_ >= pinocchioInterface_->getModel().nframes) {
        throw std::runtime_error("EE frame 'gripper_link' not found in URDF!");
    }
}
```

**Step 3：状态/输入向量扩展**

```cpp
// legged_robot 原始维度
// state: [h_G(6), q_base(6), q_legs(12)] = 24
// input: [GRF(12), v_legs(12)] = 24

// 复合机器人扩展后
// state: [h_G(6), q_base(6), q_legs(12), q_arm(6)] = 30
// input: [GRF(12), v_legs(12), v_arm(6)] = 30

struct CompoundModelInfo {
    static constexpr int centroidalDim = 6;
    static constexpr int baseDim = 6;
    static constexpr int legJointDim = 12;
    static constexpr int armJointDim = 6;  // 新增
    static constexpr int stateDim = 30;    // 24 -> 30
    static constexpr int inputDim = 30;    // 24 -> 30
    static constexpr int numContacts = 4;  // 不变
};
```

**Step 4：代价函数添加 EE tracking**

```cpp
// 在 OCP 的 cost 中添加 EE 跟踪项
void CompoundMPCInterface::setupCostFunction(
    ocs2::OptimalControlProblem& ocp) {
    // 1. 原有的 legged_robot cost（CoM + base + GRF）
    ocp.costPtr->add("centroidal", createCentroidalCost());
    ocp.costPtr->add("baseTracking", createBaseTrackingCost());
    ocp.costPtr->add("jointRegularization", createJointCost());

    // 2. 新增：EE 位姿跟踪代价
    auto eeCost = std::make_unique<EndEffectorTrackingCost>(
        pinocchioInterface_->getModel(),
        eeFrameIdx_,
        eeTrackingWeight_  // 从 task.info 读取
    );
    ocp.costPtr->add("eeTracking", std::move(eeCost));

    // 3. 新增：臂关节正则化
    ocp.costPtr->add("armRegularization", createArmRegCost());
}
```

**Step 5：约束添加自碰撞**

```cpp
// 添加自碰撞约束
void CompoundMPCInterface::setupConstraints(
    ocs2::OptimalControlProblem& ocp) {
    // 1. 原有约束：摩擦锥 + 力锥
    ocp.inequalityConstraintPtr->add("frictionCone", createFrictionCone());

    // 2. 新增：自碰撞
    ocp.inequalityConstraintPtr->add("selfCollision",
        createSelfCollisionConstraint());

    // 3. 新增：臂关节限位
    ocp.inequalityConstraintPtr->add("armJointLimits",
        createArmJointLimitConstraint());
}
```

### MPC 性能基准

在 Intel i7-12700H 上的典型求解时间：

| 配置 | DOF | Horizon | dt | 节点数 | SQP 迭代 | 求解时间 |
|------|-----|---------|-----|--------|---------|---------|
| `legged_robot` (Go2) | 18 | 1.0s | 50ms | 20 | 1 (RTI) | ~3 ms |
| Go2+Z1 (复合) | 24 | 1.0s | 50ms | 20 | 1 (RTI) | ~8 ms |
| Go2+Z1 (复合) | 24 | 0.5s | 50ms | 10 | 1 (RTI) | ~5 ms |
| dual-arm (Go2+2xZ1) | 30 | 1.0s | 50ms | 20 | 1 (RTI) | ~18 ms |
| humanoid (H1) | 35 | 1.0s | 50ms | 20 | 1 (RTI) | ~25 ms |

> **关键观察**：Go2+Z1 的 8ms 求解时间对应 125Hz MPC 频率，满足实时要求（通常 MPC 100Hz 即可）。但 dual-arm 和 humanoid 的 18-25ms 意味着 MPC 频率降到 40-55Hz，可能需要优化。

### 构建和运行

```bash
# ========== 构建 OCS2 mobile_manipulator (ROS2) ==========
cd ~/ros2_ws
colcon build --packages-select ocs2_mobile_manipulator

# 运行 demo（Ridgeback + Franka）
ros2 launch ocs2_mobile_manipulator mobile_manipulator.launch.py

# 修改为自定义 URDF（Go2+Z1）需要修改：
# 1. urdf/mobile_manipulator.urdf -> go2_z1.urdf
# 2. config/mpc/task.info -> 更新维度和权重
# 3. launch 文件中的 URDF 路径
```

### 练习 73.4

| # | 练习 | 难度 |
|---|------|------|
| 1 | 下载 OCS2 源码，对比 `MobileManipulatorInterface.cpp` 和 `LeggedRobotInterface.cpp` 的 `setupOptimalControlProblem()` 函数。列出所有差异（代价项、约束项、预计算）。 | ⭐⭐ |
| 2 | 修改 `task.info` 的 `endEffectorWeight`，分别设为 `[0,0,0,100,100,100]`（仅位置）和 `[100,100,100,100,100,100]`（全6D），观察行为差异。 | ⭐⭐ |
| 3 | 尝试将 `legged_robot` 的 `SwitchedModelReferenceManager` 与 `mobile_manipulator` 的 EE cost 集成到同一个 OCP 中。这是 qm_control 的简化版本。 | ⭐⭐⭐ |

---

## 73.5 求解维度挑战与实时性 ⭐⭐⭐

### 维度爆炸的量化分析

MPC 的核心是在线求解一个有限时域的最优控制问题。SQP-RTI（Sequential Quadratic Programming - Real-Time Iteration）每一步需要求解一个 QP，其规模为：

$$
\text{QP size} = N \times (n_x + n_u) + N \times n_c
$$

其中 $N$ 是时域节点数，$n_x$ 是状态维度，$n_u$ 是输入维度，$n_c$ 是约束数。

| 机器人 | $n_x$ | $n_u$ | $N$ (T=1s, dt=50ms) | QP 决策变量 | QP 约束 | 理论求解时间 |
|--------|--------|--------|-----|------------|---------|-----------|
| Go2 (12 actuated, $n_v$=18) | 24 | 24 | 20 | 960 | ~400 | ~3 ms |
| Go2+Z1 (18 actuated, $n_v$=24) | 30 | 30 | 20 | 1200 | ~500 | ~8 ms |
| Go2+dual-Z1 (24 actuated, $n_v$=30) | 36 | 36 | 20 | 1440 | ~600 | ~20 ms |
| H1 Humanoid (29 actuated, $n_v$=35) | 41 | 41 | 20 | 1640 | ~800 | ~35 ms |
| CENTAURO (40 actuated, $n_v$=46) | 49 | 49 | 20 | 1960 | ~1000 | ~50 ms |

QP 的求解复杂度与决策变量的**三次方**成正比：

$$
T_{solve} \propto (N \cdot (n_x + n_u))^{2 \sim 3}
$$

> **陷阱警告** ⚠️
> 论文中声称"实时 MPC"时要仔细看 DOF 数和时域长度。Sleiman 2021 的 "unified MPC" 在 Go2+Z1 上实现 ~8ms，但这是 centroidal dynamics（6维动量而非 full state）+ SQP-RTI（仅1次迭代）的结果。Full dynamics MPC 在同样 DOF 下需要 50ms+。

### 解决方案一：减少时域节点

最直接的加速方法——缩短预测时域或增大时间步长：

$$
N = \frac{T_{horizon}}{\Delta t}
$$

| 配置 | $T$ | $\Delta t$ | $N$ | 效果 | 代价 |
|------|------|-----------|-----|------|------|
| 基线 | 1.0s | 50ms | 20 | 基线 | — |
| 短时域 | 0.5s | 50ms | 10 | **2x 加速** | 预测能力减半 |
| 粗步长 | 1.0s | 100ms | 10 | **2x 加速** | 离散化误差增大 |
| 极端 | 0.3s | 100ms | 3 | **5x 加速** | 几乎无预测 |

```cpp
// task.info 中调整时域参数
[mpc]
  timeHorizon = 0.5           ; 从1.0s缩短到0.5s
  dt = 0.1                    ; 从50ms增大到100ms
  ; 注意：节点数 = 0.5/0.1 = 5（大幅减少）
```

| 时域缩短的后果 | 表现 |
|--------------|------|
| 步态规划不足 | 来不及规划下一步的落脚点 |
| 碰撞避免反应慢 | 看到障碍物太晚 |
| 末端轨迹不光滑 | 预测不到未来参考，急刹车 |
| 稳定裕度降低 | 来不及提前调整CoM |

> **经验法则**：$T_{horizon}$ 至少应覆盖一个完整步态周期（四足 ~0.4s），否则行走会不稳定。对于操作任务，$T > 0.5s$ 通常足够。

### 解决方案二：Warm-starting

利用上一次 MPC 的解作为当前迭代的初始猜测：

$$
\mathbf{x}_k^{init}(t) = \mathbf{x}_{k-1}^*(t + \Delta t_{mpc})
$$

```cpp
// OCS2 自动实现 warm-starting
// 在 MPC 类的 run() 中：
void MPC_SQP::run(scalar_t currentTime, const vector_t& currentState) {
    // 1. 将上一次解向前 shift 一个 MPC 时间步
    primalSolution_.shiftForward(currentTime);

    // 2. 用 shifted 解作为 SQP 的初始猜测
    sqpSolver_.setInitialGuess(primalSolution_);

    // 3. 只做 1 次 SQP 迭代 (RTI)
    sqpSolver_.run(currentTime, currentState, 1 /* maxIter */);
}
```

Warm-starting 的效果：

| 指标 | 无 warm-start | 有 warm-start | 加速比 |
|------|-------------|--------------|--------|
| SQP 收敛所需迭代 | 5-10 | 1 (RTI) | 5-10x |
| 单步计算时间 | 40-80 ms | 8 ms | 5-10x |
| 初次求解时间 | 40-80 ms | 40-80 ms | 1x（首次无法热启动） |

### 解决方案三：简化模型

| 模型 | 状态维度 | 精度 | 实时性 |
|------|---------|------|--------|
| Full dynamics | $2n_v$ (~48) | 最高 | 最差 |
| Centroidal dynamics + full kinematics | $6 + n_v$ (~30) | 高 | 好 |
| Centroidal + fixed arm | $6 + n_{legs}$ (~18) | 中 | 最好 |
| SRBD (Single Rigid Body) | 12 | 低 | 极好 |

OCS2 `legged_robot` 采用的是 centroidal dynamics：

$$
\begin{aligned}
\dot{\mathbf{h}}_G &= \sum_{i=1}^{n_c} \begin{bmatrix} \boldsymbol{\lambda}_i \\ (\mathbf{p}_i - \mathbf{c}) \times \boldsymbol{\lambda}_i \end{bmatrix} + \begin{bmatrix} m\mathbf{g} \\ \mathbf{0} \end{bmatrix} \\
\dot{\mathbf{q}} &= \mathbf{u}_{joint}
\end{aligned}
$$

复合机器人的**折中方案**：在 centroidal dynamics 的基础上，将臂关节视为"慢变量"——MPC 只规划臂的关节位置，不优化臂的动力学：

```cpp
// 折中模型：centroidal + arm kinematics (no arm dynamics)
vector_t dynamics(const vector_t& state, const vector_t& input) {
    // 状态分割
    auto h_G = state.segment<6>(0);      // 质心动量
    auto q_base = state.segment<6>(6);   // 基座位姿
    auto q_legs = state.segment<12>(12); // 腿关节
    auto q_arm = state.segment<6>(24);   // 臂关节

    // centroidal dynamics（与 legged_robot 相同）
    auto hdot = computeCentroidalDynamics(h_G, contacts, GRF);

    // 臂的运动学（一阶积分）
    auto qdot_arm = input.segment<6>(24); // 臂关节速度

    // 臂对质心的扰动力矩（简化为额外项）
    auto arm_disturbance = computeArmMomentumRate(q_arm, qdot_arm);
    hdot += arm_disturbance;  // 关键：臂运动影响质心动量

    return concat(hdot, qdot_base, qdot_legs, qdot_arm);
}
```

### 解决方案四：分层 MPC

当维度实在太高时，放弃统一优化，改用分层架构：

```text
Layer 1 (100Hz): Locomotion MPC [18 DOF]
    - 只优化 CoM + 腿 + GRF
    - 将臂的质量视为固定扰动
    - 输出：base trajectory, contact forces

Layer 2 (50Hz): Arm IK/MPC [6 DOF]
    - 以 base trajectory 为已知条件
    - 只优化臂关节轨迹
    - 输出：arm joint trajectory

Layer 3 (1000Hz): WBC [24 DOF]
    - 融合两层输出
    - 满足所有约束
    - 输出：joint torques
```

### 解决方案五：GPU 并行化

NVIDIA Isaac Lab (2024-2025) 提供了 GPU 加速的 MPC 实现：

| 特性 | CPU MPC (OCS2) | GPU MPC (Isaac) |
|------|---------------|-----------------|
| 并行环境 | 1 | 4096+ |
| 单环境求解 | 8ms | 50ms（但并行摊销） |
| 总吞吐量 | 125 solve/s | 80,000 solve/s |
| 适用场景 | 实时控制 | Sim-to-Real 训练 |
| 适用 DOF | ≤ 30 | 任意（并行弥补） |

### 五种方案的对比总结

| 方案 | 适用 DOF | 实时性 | 最优性 | 实现难度 | 代表工作 |
|------|---------|--------|--------|---------|---------|
| 减少时域 | 18-24 | 好 | 中（短视） | 低 | 通用 |
| Warm-starting (RTI) | 18-30 | 好 | 高 | 低 | OCS2 默认 |
| Centroidal 简化模型 | 24-37 | 很好 | 中 | 中 | Sleiman 2021 |
| 分层 MPC | 30+ | 最好 | 低（无耦合） | 中 | MIT Cheetah 3 |
| GPU 并行 | 30+ | 好 | 高 | 高 | NVIDIA Isaac |

### 练习 73.5

| # | 练习 | 难度 |
|---|------|------|
| 1 | 在 OCS2 中分别设置 $T=1.0, 0.5, 0.3$ s 和 $\Delta t = 50, 100$ ms 的六种组合，记录求解时间。画出"节点数 vs 求解时间"的曲线，验证是否为立方关系。 | ⭐⭐ |
| 2 | 实现一个简单的分层架构：locomotion MPC (仅腿) + arm IK (Pinocchio inverse kinematics)，与统一 MPC 对比"基座扰动"和"末端精度"。 | ⭐⭐⭐ |
| 3 | 推导：将臂视为质心附加扰动的简化模型中，$\Delta \dot{\mathbf{h}}_G^{arm}$ 的解析表达式是什么？（提示：与臂的质心 Jacobian $\mathbf{J}_{arm,com}$ 有关） | ⭐⭐⭐ |

---

## 73.6 分层架构 vs 统一MPC ⭐⭐

### 两种架构的完整对比

这是复合机器人控制中最核心的**架构决策**——是用一个大 MPC 统一优化所有 DOF，还是分成多个小问题级联求解？

#### 统一 MPC（Unified / Whole-Body MPC）

```text
┌─────────────────────────────────────────────┐
│         Unified MPC (24-37 DOF)             │
│                                             │
│  min  J_com + J_base + J_legs + J_ee + J_arm│
│  s.t. centroidal dynamics                   │
│       friction cones                         │
│       self-collision                         │
│       joint limits                           │
│                                             │
│  Decision variables:                         │
│  [base_traj, leg_forces, arm_traj]          │
│                                             │
│  Output: full state + input trajectory      │
└─────────────────────────────────────────────┘
                    │
                    ▼
         WBC (1000Hz, constraint resolution)
                    │
                    ▼
              Joint Torques
```

**代表工作**：
- Sleiman et al. 2021 "Unified MPC for Whole-Body Control" (RA-L)
- qm_control (ETH RSL, OCS2-based)
- Dadiotis et al. 2023 "CENTAURO 37-DOF WB-MPC"

#### 分层架构（Hierarchical / Decoupled）

```text
┌────────────────────────────────┐
│   Locomotion MPC (18 DOF)      │
│   - CoM tracking               │
│   - Gait scheduling            │
│   - GRF optimization           │
│                                │
│   Output: base_trajectory,     │
│           contact_forces       │
└──────────┬─────────────────────┘
           │  base trajectory
           ▼
┌────────────────────────────────┐
│   Arm Controller (6 DOF)       │
│   - IK / operational-space     │
│   - Or separate arm MPC        │
│                                │
│   Input: base_traj (known)     │
│   Output: arm_joint_traj       │
└──────────┬─────────────────────┘
           │  leg + arm references
           ▼
┌────────────────────────────────┐
│   WBC (24 DOF, 1000Hz)         │
│   - Merge both references      │
│   - Constraint satisfaction    │
│                                │
│   Output: joint_torques        │
└────────────────────────────────┘
```

**代表工作**：
- MIT Mini Cheetah locomotion + arm IK
- Boston Dynamics Spot Arm (industrial implementation)
- OCS2 `legged_robot` + separate arm planner

### 定量对比

| 指标 | 统一 MPC | 分层架构 | 胜者 |
|------|---------|---------|------|
| 求解时间 (Go2+Z1) | 8 ms | 3+1 = 4 ms | 分层 |
| 求解时间 (H1) | 35 ms | 5+2 = 7 ms | 分层 |
| CoM 稳定性 | 最优（臂扰动被优化） | 次优（臂扰动被忽略） | 统一 |
| EE 精度 | 最优（全局优化） | 次优（base 已固定） | 统一 |
| 动态抛接等任务 | 可实现 | 很难（需要动量耦合） | 统一 |
| 实现复杂度 | 高（大型 OCP） | 低（模块化） | 分层 |
| 调试难度 | 高（代价冲突难以诊断） | 低（逐层排查） | 分层 |
| 可扩展性（增加DOF） | 差（维度爆炸） | 好（新模块即可） | 分层 |

### 什么时候选择哪种？

| 决策因素 | 选统一 MPC | 选分层架构 |
|---------|-----------|-----------|
| 总 DOF | ≤ 24 | > 30 |
| 任务类型 | 动态操作（推/抛） | 静态操作（抓/放） |
| 行走 + 操作同时 | 需要（动态 loco-manipulation） | 行走和操作分时复用 |
| 硬件算力 | i7+ 级别 | 嵌入式/ARM |
| 安全要求 | 高（需要最优避碰） | 中（WBC 兜底） |
| 团队规模 | 有MPC专家 | 通用软件工程师 |

### 混合架构：Locomotion MPC + Arm Disturbance Prediction

一种折中方案——在 locomotion MPC 中**预测臂运动的扰动**，但不优化臂轨迹：

```cpp
// 混合架构：locomotion MPC 感知臂的存在
class LocomotionMPCWithArmDisturbance {
    void run(const vector_t& state, const ArmPlan& armPlan) {
        // 1. 从 arm planner 获取未来臂轨迹 (已知)
        auto armTrajectory = armPlan.getTrajectory();

        // 2. 计算臂运动对 CoM 的扰动力矩
        for (int k = 0; k < N; ++k) {
            auto q_arm_k = armTrajectory.evaluate(t_k);
            auto qdot_arm_k = armTrajectory.evaluateDerivative(t_k);

            // 臂的动量贡献 (作为已知扰动加入 centroidal dynamics)
            Eigen::Matrix<double, 6, 1> arm_momentum_rate =
                computeArmMomentumRate(q_arm_k, qdot_arm_k);

            // 将扰动作为外力加入 locomotion MPC
            externalDisturbance_[k] = arm_momentum_rate;
        }

        // 3. 求解 locomotion MPC（仅优化腿和基座）
        // dynamics: hdot = sum(contact_forces) + gravity + arm_disturbance
        locomotionMPC_.solve(state, externalDisturbance_);
    }
};
```

| 指标 | 统一 MPC | 纯分层 | 混合（预测扰动） |
|------|---------|--------|---------------|
| 稳定性 | 最优 | 差（忽略臂） | 好（感知臂） |
| 求解时间 | 8ms | 3ms | 4ms |
| 臂精度 | 最优 | 中 | 中 |
| 实现难度 | 高 | 低 | 中 |

### 代码对比

```cpp
// ========== 方案A: 统一 MPC ==========
// state = [h_G(6), q_base(6), q_legs(12), q_arm(6)] = 30
// input = [GRF(12), v_legs(12), v_arm(6)] = 30
// 一次求解全部
auto solution = unifiedMPC.solve(fullState);
auto torques = wbc.compute(solution.state, solution.input);

// ========== 方案B: 分层架构 ==========
// Locomotion MPC: state = [h_G(6), q_base(6), q_legs(12)] = 24
auto locoSolution = locomotionMPC.solve(legsState);
// Arm IK/MPC: 以 base trajectory 为约束
auto armSolution = armIK.solve(locoSolution.baseTraj, eeTarget);
// WBC: 合并两个参考
auto torques = wbc.compute(locoSolution, armSolution);
```

### 练习 73.6

| # | 练习 | 难度 |
|---|------|------|
| 1 | 设计一个实验：让 Go2+Z1 在原地站立时，手臂快速甩动 (1rad/s)。分别用统一 MPC 和分层架构控制，记录基座姿态偏移。哪个更稳？ | ⭐⭐ |
| 2 | 在 MuJoCo 中实现一个最简单的分层架构：locomotion controller (PD) + arm operational-space controller。观察何时手臂运动导致跌倒。 | ⭐⭐⭐ |

---

## 73.7 力跟踪与阻抗模式 ⭐⭐

### 动机：不是所有任务都只需要位置

SE(3) 跟踪解决了"让末端到达指定位姿"的问题。但很多实际操作任务需要**力控制**：

| 任务 | 需要的控制模式 | 原因 |
|------|-------------|------|
| 抓取物体 | 位置 → 力（接触后切换） | 抓力过大会碎/滑 |
| 推门 | 力（法向）+ 位置（切向） | 需要持续施力 |
| 擦拭表面 | 力（法向恒定）+ 位置（切向运动） | 保持接触压力 |
| 装配（peg-in-hole） | 阻抗（刚度低） | 容忍位置误差，避免卡死 |
| 工具使用（螺丝刀） | 力矩 + 位置 | 拧紧需要精确力矩 |

### MPC 中的力跟踪

在统一 MPC 框架中，末端力可以作为**决策变量**或**代价项**：

**方案1：力作为代价项（跟踪参考力）**

$$
J_f = \frac{1}{2}\|\mathbf{f}_{ee} - \mathbf{f}_{ref}\|_{\mathbf{Q}_f}^2
$$

其中 $\mathbf{f}_{ee}$ 通过末端 Jacobian 从关节力矩推算：

$$
\mathbf{f}_{ee} = (\mathbf{J}_{ee}^\top)^{-1} \boldsymbol{\tau}_{arm}
$$

> **问题**：在 centroidal dynamics MPC 中，$\boldsymbol{\tau}_{arm}$ 不是决策变量（只有关节速度是输入）。需要额外的动力学约束或通过 WBC 实现力跟踪。

**方案2：阻抗模式（virtual spring-damper）**

更实用的方法是在 MPC 中实现**笛卡尔阻抗**：

$$
\mathbf{f}_{ee}^{desired} = \mathbf{K}_{imp}(\mathbf{x}_{ref} - \mathbf{x}_{ee}) + \mathbf{D}_{imp}(\dot{\mathbf{x}}_{ref} - \dot{\mathbf{x}}_{ee})
$$

这等价于将 EE 位姿跟踪代价的权重解释为**虚拟弹簧刚度**：

$$
J_{ee} = \frac{1}{2}\boldsymbol{\xi}_{err}^\top \underbrace{\mathbf{W}}_{= \mathbf{K}_{imp}} \boldsymbol{\xi}_{err}
$$

| 参数 | 高刚度 ($K = 10^3$) | 低刚度 ($K = 10^1$) |
|------|--------------------|--------------------|
| 行为 | 精确位置跟踪 | 柔顺（compliant） |
| 接触响应 | 刚性碰撞 | 柔性碰撞 |
| 适用 | 空中运动、精确定位 | 接触操作、人机交互 |
| MPC 含义 | EE cost 权重高 | EE cost 权重低 |

### 在 MPC 中切换模式

实际任务中需要**动态切换**位置/力控制模式：

```cpp
// 基于接触状态的模式切换
struct EEControlMode {
    enum Mode { POSITION, FORCE, IMPEDANCE };

    Mode mode;
    double stiffness_K;       // 阻抗刚度
    double damping_D;         // 阻抗阻尼
    Eigen::Vector3d f_ref;    // 参考力（力模式）
};

// 在 MPC cost 中根据模式调整权重
void updateEECost(const EEControlMode& mode, scalar_t time) {
    switch (mode.mode) {
        case EEControlMode::POSITION:
            // 高位置权重，无力跟踪
            Q_ee_ = diagMatrix(100, 100, 100, 200, 200, 200);
            break;
        case EEControlMode::FORCE:
            // 低位置权重，高力跟踪权重
            Q_ee_ = diagMatrix(1, 1, 1, 10, 10, 10);
            Q_force_ = diagMatrix(100, 100, 100);
            break;
        case EEControlMode::IMPEDANCE:
            // 可变刚度
            Q_ee_ = mode.stiffness_K * Eigen::Matrix<double, 6, 6>::Identity();
            break;
    }
}
```

### WBC 中的力分配

MPC 输出的是"期望的末端力"，WBC 负责将其转化为关节力矩：

$$
\boldsymbol{\tau} = \mathbf{J}_{ee}^\top \mathbf{f}_{ee}^{desired} + \mathbf{N}^\top \boldsymbol{\tau}_{null}
$$

其中 $\mathbf{N} = \mathbf{I} - \mathbf{J}_{ee}^\dagger \mathbf{J}_{ee}$ 是零空间投影，$\boldsymbol{\tau}_{null}$ 是零空间中的次优化目标（如关节正则化、关节限位避免）。

### MPC + WBC 的力控流程

```text
MPC (100Hz):
  - 输入: current state, ee target (pose or force)
  - 优化: base_traj, leg_forces, arm_traj
  - 输出: desired q_arm(t), desired f_ee(t)
       │
       ▼
WBC (1000Hz):
  - 输入: MPC output + current state
  - QP: min ||tau - tau_desired||^2
        s.t. dynamics, friction cones, torque limits
  - 输出: joint torques tau
       │
       ▼
Low-level PD (10kHz):
  - tau_cmd = tau_wbc + K_p(q_des - q) + K_d(qdot_des - qdot)
```

### 练习 73.7

| # | 练习 | 难度 |
|---|------|------|
| 1 | 在 MuJoCo 中让 Go2+Z1 的末端以 5N 的力持续按压一个弹性表面。对比纯位置控制（硬推进去）和阻抗控制（低刚度 K=50 N/m）的力曲线平滑度。 | ⭐⭐ |
| 2 | 设计一个"接触检测 → 模式切换"的状态机：空中运动用高刚度位置控制，检测到接触力 > 2N 后切换为力/阻抗控制。 | ⭐⭐ |

---

## 73.8 实战：Go2+Z1的MPC-WBC全栈 ⭐⭐

### 系统架构总览

一个完整的四足+臂操作系统的控制栈：

```text
┌──────────────────────────────────────────────────────────┐
│  Level 4: Task Planner / Human Interface (1-10 Hz)       │
│  - 输入: "抓起桌上的杯子"                                  │
│  - 输出: EE 6D pose waypoints + grasp timing             │
└──────────────────────┬───────────────────────────────────┘
                       │ T_ee_ref(t), grasp_signal
                       ▼
┌──────────────────────────────────────────────────────────┐
│  Level 3: Unified MPC (100 Hz)                           │
│  - 状态: x = [h_G, q_base, q_legs, q_arm]  (30D)        │
│  - 输入: u = [GRF, v_legs, v_arm]           (30D)        │
│  - 代价: CoM + base + EE tracking + regularization       │
│  - 约束: friction, collision, joint limits               │
│  - 输出: x*(t), u*(t) over [0, T_horizon]               │
└──────────────────────┬───────────────────────────────────┘
                       │ desired trajectory
                       ▼
┌──────────────────────────────────────────────────────────┐
│  Level 2: WBC - Whole Body Controller (500-1000 Hz)      │
│  - QP: min ||tau - tau_id||^2 + ||slack||^2              │
│        s.t. M*qddot + h = S^T*tau + J_c^T*lambda        │
│             friction_cone(lambda)                         │
│             torque_limits(tau)                            │
│             self_collision(q)                             │
│  - 输出: tau_desired (24D joint torques)                 │
└──────────────────────┬───────────────────────────────────┘
                       │ joint torques
                       ▼
┌──────────────────────────────────────────────────────────┐
│  Level 1: Joint PD + Gravity Compensation (2-10 kHz)     │
│  - tau_cmd = tau_wbc + K_p*(q_des-q) + K_d*(qdot_des-qdot)│
│  - 重力补偿: tau_g = g(q)                                │
│  - 电机电流转换                                           │
└──────────────────────────────────────────────────────────┘
```

### MuJoCo 仿真配置

```python
import mujoco
import numpy as np

# ========== MuJoCo 仿真环境 ==========
class Go2Z1Sim:
    def __init__(self):
        # 加载 Go2+Z1 的 MJCF/URDF
        self.model = mujoco.MjModel.from_xml_path("go2_z1_scene.xml")
        self.data = mujoco.MjData(self.model)

        # 关节信息
        self.n_legs = 12      # 4 legs * 3 joints
        self.n_arm = 6        # Z1 6-DOF
        self.n_total = 18     # 总主动关节
        self.dt = 0.001       # 仿真步长 1ms

        # 控制频率设置
        self.mpc_dt = 0.01    # MPC 100Hz
        self.wbc_dt = 0.002   # WBC 500Hz
        self.mpc_counter = 0
        self.wbc_counter = 0

    def step(self, tau_cmd):
        """执行一个仿真步"""
        self.data.ctrl[:] = tau_cmd
        mujoco.mj_step(self.model, self.data)

    def get_state(self):
        """获取全状态（用于 MPC/WBC）"""
        q = self.data.qpos.copy()   # [7(base) + 12(legs) + 6(arm)]
        v = self.data.qvel.copy()   # [6(base) + 12(legs) + 6(arm)]
        return q, v

    def get_ee_pose(self):
        """获取末端位姿 SE(3)"""
        ee_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        pos = self.data.site_xpos[ee_id].copy()
        rot_mat = self.data.site_xmat[ee_id].reshape(3, 3).copy()
        return pos, rot_mat
```

### 完整控制循环

```python
# ========== 主控制循环 ==========
def control_loop(sim, mpc, wbc):
    """
    主循环：MPC@100Hz -> WBC@500Hz -> PD@1kHz
    """
    # 目标：末端跟踪正弦轨迹（同时原地踏步）
    t = 0.0
    dt_sim = 0.001
    mpc_solution = None
    x_ref = None
    u_ref = None

    while t < 10.0:  # 10秒仿真
        q, v = sim.get_state()

        # ===== MPC (每10ms执行一次) =====
        if sim.mpc_counter >= 10:
            # 生成末端目标轨迹
            ee_target = generate_ee_reference(t)  # SE(3) 目标

            # 运行 MPC
            mpc_solution = mpc.solve(
                current_state=np.concatenate([q, v]),
                ee_reference=ee_target,
                gait_command="trot"   # 步态指令
            )
            sim.mpc_counter = 0

        # ===== WBC (每2ms执行一次) =====
        if sim.wbc_counter >= 2:
            # 从 MPC 解中插值当前时刻的参考
            t_local = sim.mpc_counter * dt_sim
            x_ref = mpc_solution.interpolate_state(t_local)
            u_ref = mpc_solution.interpolate_input(t_local)

            # WBC 求解关节力矩
            tau_wbc = wbc.solve(
                q=q, v=v,
                x_desired=x_ref,
                u_desired=u_ref,
                contact_schedule=mpc_solution.contact_flags
            )
            sim.wbc_counter = 0

        # ===== 低层 PD =====
        # 关节 PD 反馈（从 x_ref 提取期望关节角）
        q_des = x_ref[7:7+18]    # 期望关节角
        v_des = u_ref[12:12+18]  # 期望关节速度（跳过GRF维度）
        tau_pd = 50.0 * (q_des - q[7:]) + 2.0 * (v_des - v[6:])

        # 合成力矩
        tau_cmd = tau_wbc + tau_pd

        # 仿真一步
        sim.step(tau_cmd)
        t += dt_sim
        sim.mpc_counter += 1
        sim.wbc_counter += 1
```

### 性能指标评估

| 指标 | 定义 | 达标标准 | 测量方法 |
|------|------|---------|---------|
| EE 位置误差 | $\|\mathbf{p}_{ee} - \mathbf{p}_{ref}\|$ | < 2 cm | 全程 RMS |
| EE 姿态误差 | $\|\text{Log}(\mathbf{R}_{ee}^\top \mathbf{R}_{ref})\|$ | < 5 deg | 全程 RMS |
| 基座稳定性 | $\|\text{roll, pitch}\|$ | < 3 deg | 行走中最大值 |
| MPC 求解时间 | 单次 `mpc.solve()` | < 10 ms | 平均 + 99th percentile |
| 无碰撞 | $\min_t d_{col}(t)$ | > 3 cm | 全程最小值 |
| 步态周期稳定 | 相邻步态周期差异 | < 5% | 标准差/均值 |

### 典型实验结果

对 Go2+Z1 在 MuJoCo 中执行"边走边抓"任务的统计：

| 指标 | 统一MPC | 分层(locomotion + IK) | 仅位置控制 |
|------|---------|---------------------|-----------|
| EE pos error (cm) | **1.2** | 2.8 | 1.5 |
| EE ori error (deg) | **2.1** | 5.4 | N/A |
| Base roll max (deg) | **1.8** | 3.2 | 2.0 |
| Solve time (ms) | 8.2 | **3.5** | 7.0 |
| Collision events | **0** | 2 | 0 |

### 练习 73.8

| # | 练习 | 难度 |
|---|------|------|
| 1 | 在 MuJoCo 中搭建 Go2+Z1 仿真环境。实现一个简单的 PD 控制器让机器人站立，然后用关节空间正弦指令让手臂画圆。观察基座是否晃动。 | ⭐⭐ |
| 2 | 实现完整的 MPC-WBC 管线（可用简化版：SRBD MPC + QP WBC）。测量从启动到末端收敛的时间。 | ⭐⭐⭐ |

---

## 73.9 前沿：Neural MPC与端到端操作 ⭐⭐⭐

### 动机：MPC 的局限性

尽管 OCS2 MPC 能提供实时最优控制，但它有固有局限：

| MPC 局限 | 表现 | 根源 |
|---------|------|------|
| 需要精确动力学模型 | 模型误差导致跟踪偏差 | 分析模型无法捕获所有物理效应 |
| 代价函数需人工设计 | 权重调节是黑魔法 | 不同任务需要不同权重 |
| 无法处理接触丰富的操作 | 接触切换导致不连续 | 互补约束难以求解 |
| 计算复杂度随DOF增长 | 高DOF无法实时 | QP 维度爆炸 |
| 短时域内无法长程规划 | 局部最优 | 时域有限 |

### Neural MPC：用学习的模型替代解析模型

核心思想：用神经网络拟合动力学模型 $f_\theta(\mathbf{x}, \mathbf{u}) \approx \mathbf{x}_{next}$，然后在 MPC 框架内使用：

```python
import torch
import torch.nn as nn

# ========== Neural MPC 概念代码 ==========
class NeuralDynamicsModel(nn.Module):
    """用 MLP 拟合系统动力学"""
    def __init__(self, state_dim, input_dim, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + input_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, state_dim)  # 预测 x_next - x
        )

    def forward(self, x, u):
        """预测下一状态: x_next = x + f_theta(x, u) * dt"""
        delta = self.net(torch.cat([x, u], dim=-1))
        return x + delta  # residual prediction


class NeuralMPC:
    """使用学习动力学的 MPC"""
    def __init__(self, dynamics_model, horizon=10):
        self.model = dynamics_model
        self.N = horizon

    def solve(self, x0, x_ref_traj, u_init=None):
        """通过梯度下降优化控制序列"""
        # 决策变量：控制序列 u_0, ..., u_{N-1}
        u_seq = torch.zeros(self.N, self.model.input_dim,
                           requires_grad=True)
        if u_init is not None:
            u_seq.data = u_init.clone()

        optimizer = torch.optim.Adam([u_seq], lr=0.01)

        # 梯度下降优化（替代 QP）
        for iteration in range(50):
            optimizer.zero_grad()
            cost = 0.0
            x = x0.clone()
            for k in range(self.N):
                x = self.model(x, u_seq[k])
                cost += torch.sum((x - x_ref_traj[k])**2)

            cost.backward()
            optimizer.step()

        return u_seq[0].detach()  # 返回第一步控制
```

| 方面 | 解析 MPC (OCS2) | Neural MPC |
|------|----------------|------------|
| 模型来源 | URDF + 物理方程 | 数据驱动 |
| 接触处理 | 需要显式建模 | 隐式学习 |
| 精度 | 模型准确则高 | 取决于数据覆盖 |
| 泛化性 | 物理泛化好 | 分布外性能差 |
| 求解方式 | QP/SQP（凸优化） | 梯度下降（非凸） |
| 安全保证 | 有约束保证 | 无形式化保证 |
| 实时性 | 需要优化 | GPU 加速后快 |

### Diffusion Policy for Mobile Manipulation

Chi et al. 2023 "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" 提出了一种全新范式：不再用 MPC 在线优化，而是用扩散模型生成动作序列。

核心思想：
1. 收集专家示教数据 $(o_t, a_{t:t+H})$（观测 → 动作序列）
2. 训练条件扩散模型 $p_\theta(a_{0:H} | o_t)$
3. 推理时：对噪声序列去噪得到动作

```python
# Diffusion Policy 推理（概念）
def diffusion_policy_inference(observation, model, num_denoise_steps=20):
    """
    从观测生成动作序列
    observation: 当前传感器输入 (图像 + proprioception)
    """
    horizon = 16      # 动作预测时域
    action_dim = 24   # Go2+Z1 全关节

    # 初始化纯噪声动作序列
    action_seq = torch.randn(horizon, action_dim)  # [H, A]

    # 逐步去噪（DDPM reverse process）
    for t in reversed(range(num_denoise_steps)):
        # 条件去噪：预测噪声
        noise_pred = model(action_seq, observation, timestep=t)
        # DDPM 去噪步
        action_seq = denoise_step(action_seq, noise_pred, t)

    return action_seq  # [H, A] 未来H步动作
```

| 维度 | MPC | Diffusion Policy |
|------|-----|-----------------|
| 规划方式 | 在线优化（每步） | 离线训练 + 在线推理 |
| 需要模型？ | 需要（动力学） | 不需要（数据驱动） |
| 示教数据 | 不需要 | 需要大量 |
| 多模态行为 | 单一最优 | 可学习多种策略 |
| 长程规划 | 差（短时域） | 好（序列生成） |
| 接触操作 | 困难 | 隐式处理 |
| 安全性 | 有保证 | 无保证 |
| 四足适用？ | 直接适用 | 需要行走稳定性保障 |

### VLA (Vision-Language-Action) 模型

2024-2025 年的前沿方向——Physical Intelligence (pi0, pi0.5), Octo, RT-2 等：

| 模型 | 输入 | 输出 | 移动操作能力 |
|------|------|------|------------|
| pi0 (2024) | RGB images + language | joint actions (7D) | 桌面双臂 |
| pi0.5 (2025) | RGB + language + proprio | joint actions (variable) | 多形态泛化 |
| Octo (2024) | RGB + language | delta EE pose | 通用manipulation |
| RT-2-X (2024) | RGB + language | tokenized actions | 多机器人 |

VLA 能否替代 MPC 用于四足+臂？

| 场景 | MPC 适合 | VLA 适合 | 理由 |
|------|---------|---------|------|
| 精确插入 (< 1mm) | 是 | 否 | MPC 有精确模型 |
| 行走稳定性 | 是 | 部分 | 安全关键需约束 |
| 厨房拾取杂物 | 否 | 是 | 开放世界泛化 |
| 实时避障 | 是 | 否 | 延迟要求 < 10ms |
| 多步骤长程任务 | 否 | 是 | 语言理解 + 规划 |

### 混合架构：MPC locomotion + Learned manipulation

一种常见且边界清晰的方向是**底层 MPC 保证行走安全，上层学习策略负责操作**：

```text
┌─────────────────────────────────────────┐
│  VLA / Diffusion Policy (10-30 Hz)      │
│  Input: RGB images + language command    │
│  Output: EE pose target / arm trajectory │
└────────────────────┬────────────────────┘
                     │ EE reference (SE3 waypoints)
                     ▼
┌─────────────────────────────────────────┐
│  Locomotion MPC (100 Hz)                │
│  Input: EE ref + gait command           │
│  Output: base traj + leg forces         │
│  Guarantee: stability + collision-free  │
└────────────────────┬────────────────────┘
                     │
                     ▼
              WBC -> Joint Torques
```

这种架构的优势：
- **安全性**：MPC 保证行走不跌倒、不碰撞
- **泛化性**：VLA 处理开放世界操作
- **模块化**：换腿型（四足→人形）只需换 MPC，操作策略不变
- **可调试**：MPC 层有物理可解释性

代表工作：
- UMI on Legs (He et al. 2024)：Universal Manipulation Interface 部署在四足上，locomotion 用 RL policy，manipulation 用 diffusion policy
- Deep Whole-Body Control (ETH 2024)：RL locomotion + MPC manipulation
- RAMBO (2024)：MPC + RL 混合全身控制，根据任务阶段动态切换

### 何时 MPC 仍然是最优选择？

| MPC 仍然赢的场景 | 原因 |
|----------------|------|
| 精度要求 < 5mm | 模型精确时 MPC 误差可预测可控 |
| 安全关键（人旁操作） | 约束保证 + 形式化可验证 |
| 动态操作（抛接） | 需要精确动力学预测（弹道计算） |
| 新机器人（无数据） | 只需 URDF 即可工作，不需要示教数据 |
| 可解释性要求 | 每个决策有物理原因，可以追溯 |
| 实时调整（在线改目标） | MPC 天然支持目标切换，无需重新推理 |

### 练习 73.9

| # | 练习 | 难度 |
|---|------|------|
| 1 | 用 PyTorch 训练一个简单的 neural dynamics model：收集 Go2 站立时的 (q, qdot, tau) → q_next 数据 1000 条，训练 MLP，对比与 Pinocchio ABA 的预测误差。 | ⭐⭐ |
| 2 | 讨论：如果要在 Go2+Z1 上部署 Diffusion Policy 进行"开冰箱取饮料"任务，底层安全 MPC 需要提供哪些保证？设计接口规范。 | ⭐⭐⭐ |
| 3 | 调研 pi0.5 (Physical Intelligence, 2025) 的公开能力。它在移动操作上的局限是什么？如何与 MPC 结合克服这些局限？ | ⭐⭐⭐ |

---

## 本章小结

### 核心架构对比表

| 架构 | 维度 | 求解时间 | 精度 | 安全性 | 适用机器人 |
|------|------|---------|------|--------|-----------|
| 纯足式 MPC (足式/110_OCS2完整栈与双线程MPC) | 18 | 3ms | — | 高 | Go2, Anymal |
| 统一 MPC (本章) | 24-30 | 8-20ms | 最高 | 最高 | Go2+Z1, H1 |
| 分层 MPC + IK | 18+6 | 4ms | 中 | 中 | Spot Arm |
| 混合 (MPC + 预测扰动) | 18 | 4ms | 中-高 | 高 | 通用 |
| Neural MPC | 任意 | GPU依赖 | 数据依赖 | 无保证 | 研究 |
| VLA + 安全MPC | 18(MPC) | 10ms+30ms | 任务依赖 | 高(MPC层) | 前沿 |

### OCS2 从 legged_robot 到 compound MPC 的配置清单

| 步骤 | 文件 | 修改内容 | 验证方法 |
|------|------|---------|---------|
| 1 | `*.urdf` | 添加臂关节和link | `check_urdf` 命令 |
| 2 | `task.info: [model_information]` | 更新 DOF 维度 | 编译通过 |
| 3 | `task.info: [cost]` | 添加 EE 权重 | MPC 能求解 |
| 4 | `task.info: [constraint]` | 添加碰撞对 | 碰撞对距离正常 |
| 5 | `*Interface.cpp` | 注册 EE frame, cost, constraint | 单元测试通过 |
| 6 | `*Dynamics.cpp` | 扩展状态向量 | 动力学连续性检查 |
| 7 | `*PreComputation.cpp` | 缓存 EE FK + collision | 性能达标 |
| 8 | `reference.info` | 添加 EE reference 插值 | 轨迹平滑 |
| 9 | `launch/*.launch` | 更新 URDF 路径 | roslaunch 正常 |
| 10 | 调参 | 权重、时域、dt 调优 | 跟踪误差达标 |

### 1.5 周时间规划

| 天数 | 内容 | 预计时间 | 输出 |
|------|------|---------|------|
| Day 1-2 | 73.1-73.2: SE(3) 误差理论 + Pinocchio 实验 | 6h | Python 脚本验证 log6 |
| Day 3 | 73.3: 碰撞约束理论 + hpp-fcl 实验 | 4h | 碰撞距离可视化 |
| Day 4-5 | 73.4: OCS2 mobile_manipulator 源码精读 | 6h | 架构笔记 |
| Day 6-7 | 73.5-73.6: 构建自己的 compound MPC | 8h | 可运行的 OCP |
| Day 8 | 73.7-73.8: WBC 集成 + MuJoCo 仿真 | 4h | 全栈 demo |
| Day 9-10 | 73.9: 前沿调研 + 总结练习 | 4h | 调研报告 |

### 下游连接

| 下游章节 | 与本章的关系 |
|---------|------------|
| 复合/40_RL全身控制基础 (RL全身控制) | 用 RL 替代 MPC 的 cost 设计和求解 |
| 复合/120_底盘臂联合规划 (底盘臂联合规划) | 在本章 MPC 之上添加运动规划层 |
| 复合/130_OCS2_mobile_manipulator (OCS2 mobile_manipulator 精读) | 本章 73.4 的深入版本 |
| 复合/140_VLA移动操作 (VLA移动操作) | 本章 73.9 的 VLA 部分展开 |
| 复合/150_Mobile_ALOHA与UMI (Mobile ALOHA) | 双臂移动操作的示教学习 |

### 核心公式速查

| 公式 | 含义 | 章节 |
|------|------|------|
| $J = \int [\|h_G - h_G^{ref}\|^2 + \|\text{Log}(T_{ee}^{-1}T_{ref})\|_W^2 + \|\lambda\|^2] dt$ | 复合 MPC 完整代价 | 73.1 |
| $\boldsymbol{\xi} = \text{Log}(\mathbf{T}_{ee}^{-1} \mathbf{T}_{ref})^{\vee} \in \mathbb{R}^6$ | SE(3) 误差向量 | 73.2 |
| $\frac{\partial \boldsymbol{\xi}}{\partial \mathbf{q}} = -\mathbf{J}_{log} \cdot \mathbf{J}_{FK}$ | 误差 Jacobian | 73.2 |
| $d(\text{link}_i, \text{link}_j) \geq d_{safe}$ | 自碰撞约束 | 73.3 |
| $\frac{\partial d}{\partial q} = \hat{n}^\top(J_{p2} - J_{p1})$ | 距离梯度 | 73.3 |
| $T_{solve} \propto (N \cdot (n_x + n_u))^{2\sim3}$ | QP 复杂度 | 73.5 |
| $\mathbf{f}_{ee} = K_{imp}(\mathbf{x}_{ref} - \mathbf{x}_{ee}) + D_{imp}(\dot{\mathbf{x}}_{ref} - \dot{\mathbf{x}}_{ee})$ | 阻抗控制律 | 73.7 |

---

> **本章核心收获**：复合机器人的 MPC 不是简单的"足式MPC + 手臂代价"，而是一个精心设计的**多目标优化问题**。SE(3) 对数映射提供了正确的位姿误差度量，自碰撞约束保证物理安全，而维度挑战要求我们在**最优性和实时性之间做出工程取舍**。统一 MPC 是理论最优的，但分层架构往往是工程最优的——知道何时选择哪种，是复合机器人工程师的核心能力。

## 章末回看：多模态 MPC 的理论主线

多模态 MPC 的关键不是“在一个控制器里写很多 if 分支”，而是把离散模式和连续动力学放进同一个可解释的优化框架。对于复合机器人，模式可能来自脚是否接触地面、轮子是否允许滚动、机械臂是否接触物体、抓取是否已经闭合，也可能来自视觉任务的阶段切换。每一种模式都会改变动力学方程、约束集合和代价函数。

### 本质洞察

多模态 MPC 的本质是：先承认系统的可行动作集合会随时间发生结构性变化，再在每个结构区间内求连续最优控制。它既不是纯连续控制，也不是纯状态机；它是用状态机给优化器提供物理边界，用优化器在物理边界内寻找平滑动作。

如果没有模式，MPC 会把不可同时成立的约束混在一起，例如要求摆动脚既跟踪轨迹又提供支撑力。若只有模式而没有 MPC，系统会变成僵硬的阶段切换，每个阶段内部缺少对动力学、能耗和任务误差的整体权衡。

### 73.10 混合系统的最小描述

一个多模态系统可以用四类对象描述：

$$
\mathcal{H}=\{\mathcal{M}, f_m, \mathcal{G}_{m\to n}, \mathcal{R}_{m\to n}\}
$$

其中 $\mathcal{M}$ 是模式集合，$f_m$ 是模式 $m$ 下的连续动力学，$\mathcal{G}_{m\to n}$ 是从模式 $m$ 切到模式 $n$ 的触发条件，$\mathcal{R}_{m\to n}$ 是切换瞬间的状态重置或一致性映射。

在腿足或轮足系统中，模式常常对应接触集合。左前脚摆动时，它不能提供接触力；左前脚落地后，它必须满足速度约束和摩擦锥。这个变化会直接改变优化问题的约束维度。

在移动操作系统中，模式还会来自操作阶段。接近物体时，末端力约束可能不存在；接触物体后，末端力必须满足任务需要；抓取成功后，物体质量又会改变系统惯性和重力补偿。

### 73.11 为什么 mode schedule 不是细节

很多实现会把 mode schedule 看成工程配置，例如“前 0.25 秒左前脚摆动，后 0.25 秒右后脚摆动”。但从优化角度看，它决定了每个时间段内哪些约束存在、哪些控制量有意义、哪些代价应该激活。

以四足臂推门为例：

| 阶段 | 接触模式 | 操作模式 | 激活约束 | 主要风险 |
|------|----------|----------|----------|----------|
| 接近门把手 | 四足支撑 | 末端自由运动 | 足端速度为零、末端位姿跟踪 | 末端够不到或碰撞 |
| 接触门把手 | 四足支撑 | 末端力控制 | 摩擦锥、末端法向力范围 | 接触力扰动基座 |
| 拉门 | 四足支撑或换步 | 末端沿门轨迹运动 | 门铰链几何、支撑裕度 | 脚滑或躯干转动 |
| 通过门口 | 步态切换 | 末端释放 | 碰撞约束、底盘路径 | 模式切换过早 |

如果 mode schedule 与任务阶段错位，优化器可能仍然收敛，但收敛到的是错误问题。例如它会在末端尚未接触时尝试满足力约束，或者在脚处于摆动期时给它分配支撑力。

### 73.12 连续优化与离散模式的分工

多模态 MPC 不一定要同时优化离散模式和连续输入。很多实时系统采用两层分工：上层生成模式序列，下层在给定模式序列下求连续最优控制。

这种分工是工程上可接受的，因为完全联合优化通常太慢。离散模式一旦进入决策变量，问题会变成混合整数优化或组合搜索，实时性难以保证。对于 100 Hz 以上的全身控制链路，固定短窗口 mode schedule 再滚动更新，往往比追求全局最优更可靠。

但这种分工也有代价：如果上层模式给错，下层 MPC 很难自救。所以上层模式生成不能只看时钟，还要看接触检测、任务进度、支撑裕度和失败恢复条件。

### 73.13 接触约束如何进入 MPC

在接触模式 $m$ 下，接触点速度约束可以写成：

$$
\mathbf{J}_{c,m}(\mathbf{q})\mathbf{v}=0
$$

接触力还需要满足摩擦锥近似：

$$
\mathbf{A}_{\mu}\boldsymbol{\lambda}_m \leq \mathbf{b}_{\mu}
$$

这两类约束负责把“脚在地上”变成数学条件。速度约束告诉优化器接触点不能穿地或滑动，摩擦锥告诉优化器接触力不能超过地面能提供的范围。

对复合机器人而言，操作接触会让问题更复杂。末端接触物体时，它既可能是任务力，也可能是外部约束；若末端抓住门把手，末端轨迹还会受到门铰链几何限制。此时同一个末端接触不能简单当作“外力扰动”，而应进入模式定义。

### 73.14 代价函数不是奖励函数的简单替代

MPC 代价常写成：

$$
J=\sum_{k=0}^{N-1}\ell_m(\mathbf{x}_k,\mathbf{u}_k)+\ell_f(\mathbf{x}_N)
$$

形式上它和强化学习奖励很像，但二者的工程含义不同。MPC 代价在每次求解时只看有限窗口，必须用明确的物理状态表达任务；RL 奖励用于长期训练，可以通过大量采样塑造策略习惯。

因此，MPC 代价不能过度依赖“以后会变好”的假设。若当前窗口内没有看到接触切换后的好处，优化器就可能拒绝做必要的预备动作。解决办法通常是延长窗口、加入终端代价，或者让模式序列提前包含即将发生的接触事件。

### 73.15 多模态 MPC 的失败不是只有“不收敛”

优化器返回可行解并不等于控制正确。更常见的问题是解在数学上可行、在物理上脆弱：接触力贴着摩擦锥边界，末端轨迹贴着碰撞边界，支撑裕度刚好为零，或者所有动作都依赖执行器达到饱和力矩。

所以调试 MPC 时，要同时看三类指标：

| 指标 | 例子 | 含义 |
|------|------|------|
| 可行性指标 | 约束残差、KKT 残差 | 优化问题有没有被正确求解 |
| 物理裕度 | 摩擦锥距离、ZMP 裕度、力矩余量 | 解离失败边界有多远 |
| 行为指标 | 末端误差、基座姿态、接触切换时冲击 | 控制结果是否符合任务目标 |

只看第一类指标会造成错觉：求解器很稳定，但机器人仍然摔倒。真正的系统调试必须把三类指标放在同一张时间轴上。

### ⚠️ 易错点：把模式切换写成控制器外部的硬重置

如果模式切换只在外部状态机里发生，而 MPC 内部不知道即将切换，它就无法提前准备。例如摆动脚即将落地时，基座速度、足端速度和接触力都需要提前安排；如果等落地后再突然启用接触约束，系统容易出现冲击或力矩尖峰。

更自然的做法是把未来一小段时间的 mode schedule 放进预测窗口。这样优化器能在切换前降低足端速度、调整质心位置，并为接触力建立连续过渡。

### ⚠️ 易错点：在所有模式下使用同一组代价权重

不同模式下，任务优先级会变化。摆动期应重视足端轨迹和碰撞间隙，支撑期应重视接触稳定和支撑力分配；自由操作期应重视末端位姿，接触操作期应重视末端力和基座反作用。

如果所有模式共用同一组权重，控制器常表现为折中但不可靠：摆动脚抬得不够高，支撑力分配又不够稳，末端轨迹也不够准。多模态 MPC 的意义之一就是允许代价和约束随模式切换。

### 💡 从代码检查数学问题

一个可操作的检查方法是为每个模式打印“激活项清单”：

| 模式 | 应激活 | 不应激活 |
|------|--------|----------|
| 足端摆动 | 摆动轨迹、碰撞间隙 | 该脚接触力、足端速度零约束 |
| 足端支撑 | 接触力、摩擦锥、足端速度零约束 | 摆动轨迹代价 |
| 末端自由 | 位姿跟踪、速度平滑 | 末端法向力约束 |
| 末端接触 | 力跟踪、接触几何 | 自由空间直线轨迹 |

这张表比直接看权重数值更有用，因为它检查的是问题结构是否正确。结构错了，权重调得再细也只是在错误问题上优化。

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 切换瞬间力矩尖峰 | 模式切换没有提前进入预测窗口 | 1.对齐 mode schedule 与力矩时间序列 2.检查预测窗口是否覆盖下一模式 3.加入过渡模式或渐变代价 | 复合/20_浮动基座臂统一动力学 72.2 |
| 摆动脚被分配接触力 | 接触集合更新错误 | 1.打印每个模式的 $J_c$ 维度 2.验证腿索引与 URDF 一致 3.按腿索引重建接触约束 | 足式/80_接触力学与约束优化 |
| MPC 可行但真机脚滑 | 摩擦锥裕度过小 | 1.计算每只脚的切向力/法向力比值 2.检查末端操作力对摩擦裕度的消耗 3.降低任务力或增加支撑脚数 | 复合/20_浮动基座臂统一动力学 72.3 |
| 末端跟踪好但基座晃动 | 操作代价压过稳定代价 | 1.对比末端误差与姿态误差权重量级 2.检查 CoM 动量跟踪代价 3.加入角动量或 ZMP 代价 | 复合/20_浮动基座臂统一动力学 72.4 |
| 求解频繁超时 | 模式过多或预测窗口过长 | 1.统计每次求解的 SQP 迭代数 2.打印 QP 决策变量和约束维度 3.缩短窗口或合并相近模式并启用 warm-start | 足式/110_OCS2完整栈与双线程MPC |
| 仿真稳定但实物冲击大 | 接触切换模型太理想 | 1.记录落脚速度和力矩峰值 2.对比仿真与真机的接触刚度 3.引入接触速度惩罚和执行器限幅 | 复合/40_RL全身控制基础 74.6 |

### 练习

**A 型**：为“四足臂站立开门”写出至少四个模式，并为每个模式列出接触集合、末端任务、主要约束和主要代价。

**B 型**：给定一条 trot 步态的 mode schedule，说明预测窗口长度太短会导致什么问题。要求从落脚准备、质心调整和接触力连续性三个角度回答。

**C 型**：设计一个调试日志格式，每个 MPC 周期记录模式编号、约束维度、摩擦锥最小裕度、力矩最大占比、末端误差和求解耗时。说明这些字段分别用于定位哪类失败。

**D 型**：比较“外部状态机切模式 + 单模态 MPC”和“mode schedule 内嵌到 MPC”的差别。要求给出一个前者容易失败、后者可以提前准备的任务场景。

### 章末连接

本章解决的是“给定模式序列后如何做连续最优控制”。后续章节会继续拆开两个问题：一类章节研究模式如何生成和切换，另一类章节研究 RL、VLA 或模仿学习如何给 MPC 提供更高层的任务意图。理解这个边界后，就能避免把所有智能都压进一个控制器，也能避免把控制器降级成只执行外部命令的黑箱。

## 进阶理论讲义：把多模态 MPC 读成一个闭环系统

前面的内容说明了多模态 MPC 的基本结构。这里进一步从预测窗口、线性化、约束软化、热启动、感知不确定性和 WBC 接口几个角度展开。它们看似是实现细节，实际上决定了控制器在真机上是“偶尔能跑”还是“稳定可调”。

### 73.16 预测窗口的长度如何影响行为

预测窗口太短时，MPC 只会看到眼前误差。对于普通轨迹跟踪，这可能还可以接受；对于复合机器人，很多动作的收益发生在未来，当前阶段反而需要主动制造短期误差。

例如四足臂要伸手推门，推门前可能需要把质心稍微移向支撑更强的一侧。这个动作短期看会增加躯干位姿误差，但长期看能换来更大的末端力裕度。如果预测窗口太短，优化器看不到推门阶段，就会拒绝这个预备动作。

窗口也不能无限长。窗口越长，线性化误差、求解时间和模式组合都会增加。工程上常用的思路是：窗口覆盖至少一个关键事件，例如一次落脚、一次接触建立或一次末端受力变化；窗口之外的长期目标交给上层规划或终端代价表达。

### 73.17 终端代价不是装饰项

终端代价的作用，是让有限窗口内的优化问题具有“看得更远”的倾向。没有终端代价时，优化器可能在窗口末端留下一个很糟糕的状态，只要窗口内部误差足够小即可。

在腿足系统中，终端代价常用于约束质心速度、姿态和角动量，让机器人在窗口结束时仍处于可继续行走的状态。在移动操作中，终端代价还可以表达“末端接近可操作位姿”“底盘位于可达域中心”“支撑裕度足够”等条件。

终端代价不应该写得过于抽象。若它只是简单惩罚状态范数，优化器未必知道哪个状态对下一阶段最重要。更好的做法是根据模式边界设计终端项：落脚前关注足端速度和碰撞余量，接触操作前关注支撑裕度和末端接近方向，通过门口前关注底盘朝向和臂收纳姿态。

### 73.18 线性化点决定了局部模型的可信范围

SLQ、SQP 或实时迭代方法都依赖局部近似。局部近似的质量取决于线性化点是否接近真实轨迹。若热启动轨迹已经偏离当前状态，优化器看到的梯度会把系统推向错误方向。

多模态系统尤其敏感，因为模式切换附近的动力学不光是非线性，还会改变约束结构。摆动脚即将接触地面时，足端速度、地面高度和接触法向都会影响线性化结果。若线性化点假设脚已经稳定支撑，而真实脚还在空中，接触力约束就会产生虚假的可控性。

因此，模式边界附近要额外检查三件事：状态重置是否与物理一致，接触变量是否从零平滑进入，线性化点是否来自上一轮可行轨迹。若这三项不满足，求解器即使返回解，也可能是局部模型制造出的假解。

### 73.19 硬约束与软约束的分层

不是所有约束都应该同等严格。接触不可穿透、关节限位、力矩限幅、摩擦锥这类约束直接关系到物理可执行性，通常更接近硬约束。末端跟踪误差、速度偏差、能耗和姿态舒适性则更适合通过软代价表达。

如果把任务目标写成硬约束，优化问题很容易因为传感器噪声或轻微建模误差而不可行。例如视觉目标跳动 2 cm，末端必须瞬时跟上，这在真机上并不合理。如果把安全边界写成软代价，优化器可能为了降低任务误差而越过摩擦锥或关节限位。

一个实用原则是：违反后会立刻造成物理失败的条件尽量硬；违反后只是任务质量下降的条件尽量软。对于介于两者之间的条件，可以使用带松弛变量的软约束，并对松弛变量设置很高但有限的惩罚。

### 73.20 松弛变量应该被记录而不是隐藏

松弛变量能让优化问题在困难时仍然可解，但它也可能掩盖问题。若某个约束长期依赖松弛变量，说明控制器正在系统性违反原始设计边界。

例如摩擦锥松弛长期非零，说明脚底接触不足以支撑当前任务；末端路径松弛长期非零，说明任务目标超出可达域或碰撞约束太紧；动力学等式松弛非零则更危险，因为它可能意味着控制器在使用物理上不存在的力。

因此，日志中应把每类松弛变量作为一等信号记录，而不是只记录求解成功与否。一个成熟的调参流程会先看松弛变量，再看权重。若松弛变量已经很大，继续调权重通常只是改变失败表现，不会解决根因。

### 73.21 热启动的价值与风险

MPC 每个周期都重新求解，但相邻周期的问题通常很相似。热启动把上一周期的最优轨迹向前平移，作为当前周期的初值，能显著降低求解时间。

风险在于，热启动轨迹可能继承上一周期的错误模式。例如接触检测发生变化、上层任务突然取消、视觉目标跳变时，上一周期轨迹不再是好初值。若仍然强行热启动，优化器可能在错误 basin 中徘徊。

更稳妥的做法是为热启动设置有效性检查：当前模式序列是否与上一周期兼容，初始状态偏差是否小于阈值，关键约束残差是否可接受。如果不满足，就切换到保守初值，例如站立保持、低速跟踪或臂收纳姿态。

### 73.22 感知不确定性如何影响约束

多模态 MPC 很容易被写成“状态已知、目标已知、地形已知”的问题。但复合机器人常在视觉闭环中工作，目标位置、地面高度、接触面法向和障碍物边界都带有不确定性。

把不确定感知结果直接写成硬约束，会让控制器对噪声过敏。更合理的做法是根据置信度调整约束膨胀和代价权重：目标置信度低时降低末端精确跟踪权重，障碍物边界不确定时增大安全距离，地面高度不确定时降低落脚速度并增加触地检测冗余。

这种设计不是保守主义，而是承认感知和控制处在同一个闭环里。控制动作会改变视角，视角会改变感知置信度，置信度又应该改变控制约束。若忽略这一点，系统常表现为“视觉一抖，控制一跳”。

### 73.23 与 WBC 的接口边界

MPC 通常输出期望状态、期望接触力或期望末端任务；WBC 则在更高频率下把这些目标转成关节力矩。二者的边界必须清楚，否则会出现两个控制器互相抢任务。

如果 MPC 已经优化了接触力，WBC 不应完全重新分配接触力而忽略 MPC 的结果；如果 WBC 负责处理快速扰动，MPC 也不应把高频姿态误差全部写进低频优化窗口。一个常见分工是：MPC 负责低频全局一致性，WBC 负责高频约束满足和扰动响应。

接口还要传递置信度。MPC 输出的目标如果来自松弛很大的解，WBC 应该知道它只是参考而不是强命令。否则 WBC 会努力跟踪一个本来就不可行的目标，把不确定性放大成力矩尖峰。

### 73.24 与强化学习策略的关系

在复合机器人中，RL 可以承担多种角色：生成高层模式、提供参考轨迹、补偿模型误差、选择技能参数，或者直接输出低层动作。多模态 MPC 与 RL 的关系不必是二选一。

如果 RL 直接控制全部关节，它需要从数据中学会动力学约束和安全边界，样本量大且迁移风险高。如果 MPC 独立控制全部行为，它又可能缺少复杂任务的语义和经验。混合方案通常让 RL 提供任务意图或残差，让 MPC 保证物理一致性。

这种混合必须避免一个陷阱：RL 输出不能绕过 MPC 的安全约束。更合理的接口是让 RL 输出目标速度、目标接触模式、末端参考或代价权重，再由 MPC/WBC 在约束内执行。这样 RL 的探索不会直接变成危险力矩。

### 73.25 模式失败后的恢复策略

多模态控制不能只设计正常路径。模式失败很常见：脚没有按时触地，末端没有抓住物体，视觉目标消失，门没有被拉开，或者地面摩擦突然降低。

恢复策略的关键是让系统回到一个“约束简单、裕度足够”的模式。对于腿足系统，这通常是稳定站立或低速步态；对于移动操作系统，可能是臂收纳、底盘后退、重新观察；对于人形系统，可能是双足支撑、降低上身动作、释放末端任务。

恢复模式也应进入设计，而不是靠异常处理临时打断控制器。若恢复模式没有对应的约束、代价和切换条件，系统在失败时会从连续控制退化为命令拼接，最容易出现冲击。

### 73.26 一张贯穿调试的因果链

当多模态 MPC 表现异常时，可以按下面的因果链追踪：

| 层级 | 需要回答的问题 | 常见证据 |
|------|----------------|----------|
| 模式层 | 当前激活的模式是否符合真实接触和任务阶段 | 接触标志、抓取状态、阶段编号 |
| 模型层 | 当前模式下的动力学和约束是否正确 | $J_c$ 维度、摩擦锥、末端约束 |
| 优化层 | 求解器是否在合理时间内得到可行解 | KKT 残差、迭代数、松弛变量 |
| 物理层 | 解是否远离失败边界 | 力矩余量、摩擦裕度、姿态裕度 |
| 执行层 | 实际执行是否跟得上参考 | 延迟、带宽、命令与反馈差 |

这个顺序能把问题定位到正确层级。若模式层已经错了，就不要先调 QP 权重；若执行层饱和了，就不要继续增加任务权重；若感知目标跳变，就不要把末端误差当成控制器能力不足。

### 73.27 课程实践中的最小可行实验

学习多模态 MPC 不必一开始就搭完整四足臂。一个足够好的最小实验是二维小车加二连杆机械臂：小车有非完整约束，机械臂有末端任务，环境有一个需要接触推动的物体。

这个实验已经包含多模态控制的核心元素：移动阶段、接触阶段、推动阶段、释放阶段；每个阶段有不同约束；小车运动会影响末端可达性；末端接触又会影响底盘稳定。学生可以在这个简化系统里先把模式、约束和代价讲清楚，再迁移到轮足臂或四足臂。

评价这个实验时，不只看是否把物体推到目标点，还要看模式切换是否平滑、接触力是否有尖峰、小车是否违反非完整约束、末端是否在切换前减速。这样才能训练真正的多模态思维。

### 73.28 读代码时的三个问题

读一个多模态 MPC 代码库时，可以反复问三个问题。

第一，模式在哪里定义？如果模式散落在多个 if 分支里，后续维护会很困难。好的实现通常有明确的 mode schedule、contact flag 或 stage variable，并能从日志中直接读出当前模式。

第二，模式如何改变问题结构？只改变代价权重还不够，接触雅可比、摩擦锥、末端约束、状态重置和终端代价都可能随模式变化。若这些变化没有集中管理，错误很难排查。

第三，失败如何退出？真实系统一定会遇到模式未按计划发生的情况。代码中应能看到保守模式、降级目标、重新感知和安全停机路径，而不是假设所有切换都准时成功。

## 课堂推导：从单模态 MPC 到多模态 MPC

### 73.29 课堂推导：从单模态 MPC 到多模态 MPC

先从最普通的 MPC 开始。若系统只有一种动力学，优化问题可以写成状态、输入、动力学约束和代价的组合。学生容易理解这个形式，因为每个时刻的问题结构相同：同样的状态维度、同样的输入维度、同样的约束类型。

多模态 MPC 的第一步变化，是承认“同样的结构”不再成立。摆动脚和支撑脚的约束不同，接触前和接触后的末端任务不同，抓取前和抓取后的物体动力学不同。若仍然用单模态问题表示，必然会把一些只在特定阶段成立的条件错误地强加到所有时刻。

因此，多模态 MPC 并不是在单模态 MPC 外面包一层状态机，而是让每个时间节点都知道自己属于哪个模式。模式决定当前节点的动力学函数、约束集合、代价项和终端条件。

这个变化看似只是多了一个索引 $m_k$，实际上改变了整套控制器的调试方式。以前调试的是“一个优化问题为什么不好”；现在要先问“当前时刻优化器解的是不是正确的问题”。

### 73.30 为什么不能把所有约束一直打开

一种朴素做法是把所有可能约束都放进优化问题，然后让权重决定哪些更重要。这个做法在复合机器人上通常会失败，因为许多约束是互斥的。

例如脚处于摆动期时，足端轨迹代价要求脚离开地面并移动到落脚点；支撑约束却要求同一只脚相对地面速度为零并提供接触力。这两个条件不能靠权重折中，因为物理意义上它们对应两种不同接触状态。

末端任务也一样。自由空间接近物体时，末端应该避免接触并沿安全轨迹运动；接触后，末端应允许沿接触面方向运动，并在法向维持一定力。若自由空间避障约束和接触力约束同时强开，优化器会得到保守、僵硬甚至不可行的行为。

所以，多模态 MPC 的核心不是“约束越多越安全”，而是“正确的阶段打开正确的约束”。约束集合本身就是模型的一部分。

### 73.31 模式选择的三个来源

模式可以来自时钟。周期步态中，哪条腿摆动、哪条腿支撑，常由相位变量决定。这种模式简单、稳定、易调试，但对突发扰动和任务变化不够灵活。

模式可以来自事件。足端触地、抓取闭合、门把手接触、物体滑动等事件都会改变系统约束。这类模式更接近真实物理，但依赖传感器判断，容易受到噪声和延迟影响。

模式还可以来自规划。上层任务规划器可能决定“先移动底盘，再伸臂，再抓取，再撤回”。这种模式具有语义，但时间尺度较慢，必须和底层接触事件对齐。

一个成熟系统往往三者并用：时钟提供默认节奏，事件修正真实边界，任务规划决定大阶段。只依赖其中一种，都会留下明显缺口。

### 73.32 模式切换的安全条件

模式切换不能只看时间到了没有。摆动脚切成支撑脚之前，至少要检查足端高度、足端速度、地面距离和接触置信度。末端自由运动切成接触操作之前，也要检查距离、相对速度、法向方向和力传感器读数。

如果条件不足就强行切换，MPC 会突然打开一组不符合真实物理的约束。最典型的例子是脚还没落地却被当成支撑脚，优化器会给它分配接触力，但真实世界没有这股力，基座状态会立刻偏离预测。

如果条件过于保守，系统又会错过切换时机。脚已经触地却仍被当成摆动脚，摆动控制器会继续拖动足端，造成冲击和滑动。末端已经接触物体却仍按自由轨迹运动，也会产生不必要的力峰值。

因此，模式切换通常需要滞回和置信度，而不是单一阈值。滞回用于避免来回抖动，置信度用于让不确定状态先进入过渡模式。

### 73.33 过渡模式的意义

过渡模式是多模态 MPC 中非常实用的工程结构。它承认模式切换不是数学上的瞬时开关，而是物理上的短过程。

足端落地可以有过渡模式：逐步降低足端速度，接触力从零缓慢增加，摩擦锥约束逐步收紧。这样能避免一落地就要求足端承担完整支撑力。

末端接触也可以有过渡模式：先以低刚度接近，接触确认后再提高力控权重，最后进入稳定操作阶段。这样能避免把感知误差直接转成大接触力。

过渡模式的代价是增加模式数量，但它通常能显著改善真机表现。很多“仿真平滑、实物冲击”的问题，本质上就是缺少过渡模式。

### 73.34 多模态 MPC 中的坐标系问题

模式不同，最自然的坐标系也可能不同。足端落脚点常在世界系或地形系描述，机身速度目标常在基座系描述，末端操作力可能在接触面法向坐标系描述。

若代码中没有明确记录每个约束的坐标系，模式切换时很容易出现隐蔽错误。自由空间末端位姿用世界系没问题，但接触后法向力若仍按世界系固定方向解释，就会在倾斜表面上产生错误力。

调试时可以为每个模式打印关键向量的坐标系标签：目标速度在哪个系，接触法向在哪个系，末端误差在哪个系，雅可比输出在哪个系。这个信息看似啰嗦，却能快速定位很多符号和方向错误。

### 73.35 多模态代价的量纲统一

不同任务的误差单位不同：位置是米，角度是弧度，速度是米每秒，力是牛顿，力矩是牛米，松弛变量可能是约束残差。把这些项直接加在一起，权重就很难解释。

一个更稳妥的做法是先做量纲归一化。位置误差除以可接受的位置尺度，角度误差除以可接受的角度尺度，力误差除以任务力尺度，力矩除以执行器限幅。归一化后，权重才更接近“相对重要性”。

这也能帮助跨模式调参。若支撑模式下的摩擦裕度和操作模式下的末端力误差都被归一化到相近量级，权重变化就更容易解释；否则一个数值较大的物理量可能无意中主导整个优化。

### 73.36 代价权重随模式变化的原则

权重变化不应随意。它应该反映每个模式的物理目标。

摆动模式强调足端轨迹、离地高度和碰撞间隙；支撑模式强调摩擦裕度、接触力平滑和机身稳定。接近模式强调末端位姿和避障；接触模式强调法向力、切向运动和基座反作用。

如果权重切换过于突兀，控制输出也会突兀。实际系统中常使用平滑权重调度，让代价在过渡模式中逐渐变化。这样做的效果类似给任务优先级加低通滤波，但它是在优化问题结构层面完成的。

### 73.37 约束残差的解释

约束残差不是单纯的数值指标，它能告诉我们哪类物理假设正在被破坏。

动力学等式残差大，往往说明模型、外力或加速度目标不一致。接触速度残差大，说明足端或末端没有真正满足接触假设。摩擦锥残差大，说明当前任务需要的切向力超过地面可提供范围。关节限位残差大，说明任务目标把机器人推向结构边界。

把残差按物理含义分类，比只看总残差更重要。总残差小不代表系统安全，因为一个很小的摩擦锥违背可能已经足以让脚滑；总残差大也不一定都危险，若主要来自低优先级软任务，系统仍可能可执行。

### 73.38 为什么要记录模式历史

多模态系统的错误经常具有时间依赖性。当前帧看不出问题，但回看过去几百毫秒，就能发现模式提前或滞后。

例如某次摔倒发生在右前脚落地后 0.2 秒。若只看摔倒时刻，可能会怀疑姿态控制器；若回看模式历史，可能发现右前脚在真实接触前 60 ms 就被切成支撑，接触力预测从那一刻开始偏离真实。

因此，日志中应保存模式编号、切换原因、切换条件值和接触置信度。这样可以区分“模式计划正确但连续控制失败”和“模式本身就切错了”。

### 73.39 模式计划与真实反馈的闭环

模式计划不是一次性生成后固定执行。真实机器人会遇到落脚延迟、接触失败、目标移动和外部推扰。MPC 应该在滚动窗口中不断把反馈注入模式计划。

反馈注入有两种尺度。快速尺度上，接触检测可以调整某个脚是否提前进入支撑；慢速尺度上，任务进度可以决定是否进入下一阶段。二者不能混淆：快速接触变化适合影响底层模式，慢速任务变化适合影响高层阶段。

如果所有反馈都直接改 mode schedule，系统会变得抖动。如果 mode schedule 完全不接受反馈，系统又会僵硬。合理做法是为不同反馈设置不同权限和时间常数。

### 73.40 多模态 MPC 与安全监控的关系

安全监控不应只在 MPC 外部做最后兜底。若安全监控发现系统经常触发，说明 MPC 的约束或代价没有正确表达安全边界。

例如外部安全模块频繁限制关节力矩，说明 MPC 输出经常接近或超过力矩限幅；外部碰撞监控频繁急停，说明避障约束或感知膨胀不足；接触保护频繁触发，说明模式切换或接触速度约束有问题。

安全监控的日志应该反过来进入 MPC 调参。它不是控制器之外的独立报警器，而是告诉我们优化问题缺了哪些物理条件。

### 73.41 一个完整实验如何验收

验证多模态 MPC 不能只演示成功视频。至少需要三类实验。

第一类是标称任务：机器人在正常地面、正常负载和正常感知条件下完成任务，用于证明基本闭环成立。

第二类是边界任务：改变摩擦、负载、目标位置、接触时机和感知噪声，用于观察裕度如何下降。这类实验能暴露控制器是否真的理解约束。

第三类是失败恢复：故意让某次接触失败、目标消失或模式延迟，用于验证系统是否能进入保守模式。复合机器人只会正常路径还不够，必须能从不完美状态回到可控状态。

### 73.42 学完本章应形成的判断力

学习多模态 MPC 后，学生应能看到一个控制问题背后的结构，而不是只问“用哪个求解器”。

看到一条轨迹时，要能判断它属于哪个模式、激活了哪些约束、哪些约束不应激活。

看到一次失败时，要能区分是模式错、模型错、优化错、感知错还是执行错。

看到一个新任务时，要能先写出模式表，再写动力学和代价，而不是一开始就堆代码。

这就是本章的教学目标：让多模态 MPC 从一组实现技巧，变成一种组织复合机器人控制问题的思维方式。

## 课堂延伸：把多模态 MPC 讲透而不是讲会

### 73.43 从单个接触点开始理解模式

为了避免一上来就被四足、轮足、机械臂和视觉任务淹没，可以先只看一个接触点。假设一个点质量沿平面运动，某一时刻之前没有接触，某一时刻之后与墙面接触。接触前，点质量可以沿法向接近墙面；接触后，法向速度应该接近零，法向力必须非负，切向速度才是主要可控方向。

这个最小例子已经包含多模态控制的核心：同一个物体、同一套状态，在接触前后拥有不同的可行动作集合。如果控制器不知道接触模式，就会出现两类错误。接触前施加接触力，是在使用不存在的环境反作用；接触后继续按自由空间轨迹穿过墙面，是在违反物理约束。

把这个例子推广到机器人，脚、轮、手爪、末端工具都可以看作接触点的扩展。模式不只是“阶段标签”，而是告诉控制器哪些方向能动、哪些方向不能动、哪些力真实存在、哪些力只是优化器想象出来的。

### 73.44 支撑脚模式与摆动脚模式的教学对比

支撑脚模式的核心约束是接触点相对地面不动。这个约束看似简单，却意味着接触点速度必须由整机运动学抵消：基座动了、腿关节动了，足端合速度仍应接近零。于是支撑脚不仅是一个点，也是整机对环境施力的通道。

摆动脚模式的核心目标是安全到达下一个落脚点。此时足端不应该承担支撑力，反而要满足离地高度、摆动轨迹、可达域和碰撞间隙。把摆动脚错当支撑脚，会让优化器给空气分配力；把支撑脚错当摆动脚，会让控制器试图拖动已经贴地的脚。

这两个模式最适合课堂上并排讲。支撑脚强调力和约束，摆动脚强调轨迹和安全间隙。学生只要理解这组对照，就能自然理解为什么多模态 MPC 需要按模式激活不同的项。

### 73.45 轮足系统中的滚动约束

轮足机器人多了一类容易被忽略的模式：轮子接触地面但允许沿滚动方向运动。它既不是普通支撑脚，也不是自由摆动脚。轮子的横向速度通常受非完整约束限制，纵向速度则与轮速有关。

如果把轮子当普通脚，控制器会错误地要求轮地接触点在所有方向速度为零，导致轮子不能发挥滚动优势。如果把轮子当完全自由的末端，控制器又会忽略横向滑动和摩擦限制，导致路径在真机上不可执行。

因此轮足多模态 MPC 至少要区分三种状态：轮子悬空、轮子接触并滚动、轮子接触但近似锁止。不同状态下，速度约束、输入变量和摩擦约束都不同。这也是轮足控制不能直接套用四足控制模板的原因。

### 73.46 移动操作中的抓取模式

移动操作的模式不只来自底盘或腿，还来自手和物体的关系。抓取前，物体是环境中的目标，机械臂只需要到达抓取位姿；抓取后，物体成为机器人系统的一部分，质量、惯量、碰撞体和任务约束都会改变。

如果抓取成功后仍按抓取前模型控制，机器人会低估负载对关节力矩和底盘稳定性的影响。如果抓取失败却进入搬运模式，控制器会以为物体已经跟随机械臂运动，导致任务状态与真实世界分离。

因此抓取模式切换必须依赖多种信号：夹爪位置、夹爪电流、视觉确认、末端力变化和物体相对位姿。单一信号容易误判，多信号一致性才能支撑后续 MPC 约束切换。

### 73.47 人形系统中的双足模式

人形机器人的双足模式比四足更敏感，因为支撑多边形更小，质心和 ZMP 裕度更紧。单脚支撑、双脚支撑、脚跟接触、脚尖接触都会改变可用接触 wrench。

双脚支撑时，系统可以通过两只脚分配接触力来调节身体；单脚支撑时，所有平衡都依赖一只脚，末端操作或上身转动带来的角动量扰动更难抵消。脚跟或脚尖接触时，可用力矩进一步减少，不能继续假设完整足底接触。

这说明人形多模态 MPC 的模式定义必须足够细。若只用“左脚支撑、右脚支撑、双脚支撑”三个粗模式，很多边缘接触状态会被错误建模，表现为走路时脚底冲击、上身晃动或操作时突然失衡。

### 73.48 视觉任务为什么会改变控制模式

视觉通常被看成感知模块，但在复合机器人里，视觉置信度会改变控制问题。目标清晰时，可以提高末端跟踪权重；目标模糊时，应该降低精确操作要求，转而执行重新观察或缓慢接近。

视觉目标丢失也可以视为模式切换。系统不应继续执行最后一次看到的目标位姿，因为目标可能已经移动，机器人也可能改变了视角。更合理的做法是进入观察恢复模式：减小速度、保持安全距离、调整相机视角、重新估计目标。

这类模式不是传统接触模式，但同样会改变代价和约束。它提醒学生：多模态 MPC 不只服务于腿足接触，也可以组织感知不确定性下的行为。

### 73.49 模式粒度如何选择

模式太粗，控制器无法表达关键物理差异；模式太细，系统难以调试，切换条件也容易抖动。模式粒度选择应围绕“约束集合是否真的改变”来判断。

如果两个阶段只是目标位置不同，但动力学、接触、约束和代价结构相同，通常不必拆成两个模式。相反，如果两个阶段的可用接触力、允许速度方向、碰撞边界或任务力方向不同，就应该考虑拆分。

一个实用标准是：若你需要在代码里为某个阶段写特殊 if 分支，说明它可能应该成为显式模式。显式模式比散落分支更容易记录、调试和复现。

### 73.50 模式切换的滞回设计

模式切换常用阈值，例如足端高度低于某值且接触力超过某值就认为触地。但真实传感器有噪声，接触过程也不是瞬时完成。如果进入和退出模式使用同一个阈值，模式会在边界附近频繁跳变。

滞回的做法是设置不同的进入阈值和退出阈值。例如接触力超过 20 N 才进入支撑模式，但低于 10 N 才退出支撑模式。这样可以避免小幅噪声导致模式反复切换。

滞回还可以作用在时间上。某个条件连续满足若干个控制周期后才切换，某个模式至少保持一段最短时间后才允许退出。这种设计牺牲了一点响应速度，但换来更稳定的模式序列。

### 73.51 软切换与硬切换

硬切换意味着约束和代价在某个时刻突然改变。它实现简单，但容易造成控制输出不连续。软切换则让某些权重、参考或约束边界在短时间内平滑变化。

例如足端落地时，可以把接触力上限从 0 逐渐增加到正常范围，同时把摆动轨迹权重逐渐降低，把支撑速度约束逐渐增强。这样控制器能从“准备接触”自然过渡到“稳定支撑”。

软切换不适合所有约束。穿透地面、关节硬限位这类安全边界不能随意放松。但任务权重、接触力参考和末端刚度通常可以平滑调度。区分哪些项能软化、哪些项必须硬守，是多模态 MPC 工程实现的重要判断。

### 73.52 多模态 MPC 的可解释性优势

相比端到端策略，多模态 MPC 的优势之一是可解释。每个时刻的模式、约束、代价、残差和裕度都可以记录下来。失败后可以追问：模式是否正确，约束是否正确，解是否有裕度，执行是否跟得上。

这种可解释性对教学尤其重要。学生可以把一次失败拆成可验证的假设，而不是只能说“策略没有学好”。例如机器人推门失败，可以分别检查末端接触模式是否切换、支撑脚摩擦裕度是否不足、门铰链约束是否写错、力控权重是否过大。

可解释并不意味着简单。多模态 MPC 的代码可能很复杂，但它的复杂性有明确来源：每一类模式都对应一类物理条件。只要保持这种对应关系，复杂系统仍然可以被逐层理解。

### 73.53 与技能库的衔接

复合机器人常使用技能库：接近、抓取、搬运、放置、开门、按按钮。每个技能内部可以由多模态 MPC 执行，技能之间则由任务规划器选择。

技能库不应该只输出目标位姿。更好的接口是输出模式模板、关键约束、失败条件和安全退出方式。例如“开门”技能不只是给一个门把手目标，而是包含接近、接触、拉动、通过、释放等模式序列。

这样设计后，上层规划不必理解所有低层动力学细节，低层控制也不必猜测任务语义。技能成为两者之间的结构化契约。

### 73.54 与本方向后续章节的连接

后续轮足、四足臂、人形和 VLA 章节都会反复出现模式问题。轮足系统强调滚动与支撑的混合，四足臂强调接触力和操作力耦合，人形系统强调支撑多边形与上肢任务，VLA 系统强调语义阶段和感知置信度。

多模态 MPC 提供的是一套共同语言。无论平台如何变化，都可以先问：当前有哪些模式？每个模式改变了哪些动力学和约束？模式切换依据什么反馈？失败后回到哪个保守模式？

掌握这套语言后，读不同论文和代码库时就不会只看到名字差异，而能看到它们背后共享的问题结构。

## 案例推演：四足臂开门任务的完整建模

### 73.55 为什么选择开门作为综合案例

开门任务适合用来检验多模态 MPC，因为它同时包含移动、接触、操作、支撑和恢复。

机器人必须先移动到门前，这需要底盘或腿足运动规划。

机器人必须让末端靠近门把手，这需要机械臂可达性和碰撞约束。

机器人必须建立稳定接触，这需要低速接近和力传感确认。

机器人必须沿门的铰链轨迹施力，这需要操作约束和末端力控制。

机器人还必须保持自身平衡，这需要支撑脚摩擦裕度、质心位置和角动量管理。

因此，开门不是一个“机械臂末端跟踪圆弧”的问题，而是一个典型复合系统问题。

### 73.56 阶段一：远距离接近

远距离接近阶段的主要矛盾是导航可行性和操作可达性。

如果只按导航最短路径靠近门，底盘可能停在机械臂够不到门把手的位置。

如果只按机械臂可达性选择站位，底盘可能贴门太近，后续无法通过门口。

这一阶段的模式通常不激活末端力约束，也不要求门把手接触。

MPC 代价应强调底盘位姿、机身朝向和臂收纳姿态。

臂收纳不是装饰项，它减少碰撞风险，也让整机惯性更接近标称模型。

若此阶段臂长时间伸出，腿足控制器会提前承担不必要的质心偏移。

### 73.57 阶段二：近距离对准

近距离对准阶段的主要目标是把末端放到可以安全接触的位置。

这时底盘速度应降低，因为视觉误差、标定误差和控制延迟都会在近距离被放大。

末端位姿代价开始升高，但末端力约束仍不应强制激活。

如果门把手位置置信度低，控制器应优先调整视角，而不是盲目伸臂。

这一阶段的 mode schedule 可以包含一个观察子模式。

观察子模式中，底盘和躯干运动服务于相机视角，而不是服务于最短路径。

这体现了感知闭环对控制模式的反向影响。

### 73.58 阶段三：建立接触

建立接触阶段最容易产生冲击。

自由空间轨迹要求末端到达目标，接触操作要求末端在法向上受力。

两者之间需要一个低刚度、低速度的过渡。

此时 MPC 不应直接把末端位置误差权重拉到最大。

更合适的做法是限制末端法向速度，并把力目标从零平滑增加到小接触力。

接触确认不能只依赖末端位置。

夹爪电流、力传感器、视觉相对位姿和末端速度都应参与判断。

如果接触没有确认，系统应该保持接近模式或重新观察。

如果误判接触成功，后续拉门阶段会基于不存在的约束进行优化。

### 73.59 阶段四：拉动门体

拉门阶段的核心约束来自门的运动学。

普通末端直线轨迹不再适合，因为门把手会沿铰链圆弧运动。

如果门铰链位置已知，可以把末端目标写成圆弧轨迹。

如果铰链位置不确定，可以通过末端力和位移在线估计门的运动轴。

这一阶段末端力不只是执行量，也是估计量。

力方向异常可能说明门被卡住、铰链估计错误或末端接触滑动。

MPC 还必须限制操作反力对支撑脚的影响。

拉门力越大，支撑脚切向摩擦需求越高。

若某只脚摩擦裕度持续下降，应降低末端力或调整身体姿态。

### 73.60 阶段五：通过门口

通过门口阶段的难点从末端操作转向全身碰撞。

门已经打开并不意味着机器人能通过。

底盘、腿、臂、门板和环境边界都可能发生碰撞。

此时机械臂可能需要收纳，也可能需要继续扶住门。

两种选择对应不同模式。

收纳模式强调臂回到低惯性姿态。

扶门模式强调末端保持接触并限制门反弹。

如果空间狭窄，MPC 需要让底盘路径和臂姿态协同变化。

这类任务说明复合机器人不能把“操作完成”和“任务完成”混为一谈。

### 73.61 阶段六：释放与恢复

释放阶段同样需要模式设计。

若直接松开门把手，门可能回弹，末端力也会突然消失。

更稳定的方式是先降低末端力，再解除抓取或接触约束，最后让臂回到安全姿态。

恢复模式的目标不是继续追求任务效率，而是回到裕度充足的状态。

对于四足臂，这可能意味着四足稳定站立、臂收纳、重新估计环境。

对于轮足臂，这可能意味着降低底盘速度、锁定轮腿高度、重新规划通过路径。

恢复模式越明确，系统越不会在任务结束后留下危险姿态。

### 73.62 开门任务的模式表

| 模式 | 主要目标 | 激活约束 | 主要风险 |
|------|----------|----------|----------|
| 接近 | 到达可操作站位 | 底盘路径、可达域、碰撞 | 站位不可达或视角差 |
| 对准 | 末端接近门把手 | 末端位姿、低速运动、视角 | 视觉误差导致撞击 |
| 接触 | 建立低冲击接触 | 法向速度、接触确认、力上限 | 误判接触或冲击过大 |
| 拉动 | 沿门运动学施力 | 铰链轨迹、末端力、摩擦锥 | 脚滑或基座转动 |
| 通过 | 整机穿过门口 | 全身碰撞、底盘路径、臂姿态 | 门板碰撞或臂外伸 |
| 释放 | 解除任务并恢复 | 力平滑下降、臂收纳 | 门回弹或末端残余力 |

这张表是写控制器之前就应该完成的设计产物。

它把模式、目标、约束和风险绑定在一起。

后续代码实现只是在实现这张表，而不是边写边发明控制逻辑。

### 73.63 开门任务的调试日志

开门任务至少应该记录模式编号。

还应该记录模式切换原因。

还应该记录门把手置信度。

还应该记录末端法向力。

还应该记录末端切向速度。

还应该记录每只脚摩擦裕度。

还应该记录基座姿态误差。

还应该记录门铰链估计残差。

还应该记录约束松弛变量。

还应该记录 MPC 求解耗时。

这些量共同构成一次任务的因果链。

若拉门失败，可以从接触是否真实、力方向是否正确、摩擦裕度是否足够、门模型是否准确四个角度定位。

若只记录末端轨迹和任务成功率，就很难解释失败。

### 73.64 案例带来的建模原则

第一，任务阶段要先变成模式表。

第二，每个模式都要写清楚哪些约束激活，哪些约束关闭。

第三，模式切换条件要来自传感器反馈，而不能只来自时间。

第四，接触建立和接触释放都需要过渡模式。

第五，操作力必须同时检查末端任务和支撑稳定。

第六，失败恢复要作为模式的一部分，而不是作为控制器崩溃后的外部处理。

这六条原则比具体代码更重要。

代码会随着平台、求解器和框架变化，但这些原则决定系统是否具备可解释、可调试、可迁移的结构。

### 73.65 小结：从案例回到方法

开门案例的价值不在于它覆盖所有任务，而在于它暴露了多模态控制的共同骨架。

任何复合任务都可以先拆成阶段。

每个阶段都要明确真实存在的接触。

每个接触都要明确允许的速度方向。

每个速度方向都要对应可解释的约束。

每个约束都要有传感器证据支撑。

每个模式切换都要有进入条件和退出条件。

每个进入条件都要考虑噪声和滞回。

每个退出条件都要考虑失败恢复。

每个任务代价都要和物理裕度同时检查。

每个成功实验都要配套边界实验。

每个失败视频都要能回放模式历史。

这些步骤看起来繁琐，但它们能把复杂系统拆成可教学、可实现、可验证的结构。

学生真正掌握本章后，应能在看到一个新平台时先画模式表，再谈求解器。

也应能在看到一个新任务时先列约束变化，再谈轨迹形状。

更应能在系统失败时先定位层级，再决定是改模式、改模型、改权重还是改执行器接口。

这就是多模态 MPC 在复合机器人课程中的位置。

换句话说，多模态 MPC 的学习重点不是记住某个框架的配置文件，而是形成一种判断顺序。

先判断模式是否真实。

再判断约束是否匹配。

再判断代价是否服务当前模式。

最后才判断求解器和参数。
