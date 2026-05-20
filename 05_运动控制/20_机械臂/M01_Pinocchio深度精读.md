> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M01 Pinocchio 深度精读——从刚体动力学引擎到实时 MPC 基石

> **本章定位**：Pinocchio 是 INRIA 学派（TSID / Crocoddyl / OCS2 / Aligator）的动力学内核，也是后续 M05 QP、M06 自动微分、M14 MoveIt2 共用的基础设施。本章从"为什么需要动力学引擎"出发，逐层拆解 Pinocchio 3.x 的 CRTP 访问者模式、Model/Data 分离架构、正运动学、雅可比、三大动力学算法（RNEA / ABA / CRBA）、解析导数、碰撞检测与约束动力学，最终通过 Franka Panda 示例贯穿完整 pipeline。
>
> **适用范围**：本章以 **固定基座机械臂** 为主线（vs 足式/30_Pinocchio深度精读 浮动基座腿足），但所有 API 和算法对腿足同样适用——浮动基座只是多了一个 `JointModelFreeFlyer` 根关节。
>
> **前置依赖**：P01（URDF/Xacro 建模）、02_基础/C++ 高级（CRTP 与设计模式）、02_基础/Eigen 线性代数（对齐与内存布局）
>
> **下游章节**：M02（动力学库对比）、M03（IK 求解器深度）、M06（自动微分与代码生成）、M14（MoveIt2 / MTC 集成）
>
> **建议用时**：1.5 周（理论 3 天 + 代码 4 天 + 综合练习 3 天）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **刚体动力学方程**：写出拉格朗日形式 $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$，解释每一项的物理含义（惯量、科里奥利/离心力、重力）。$M(q)$ 的对称正定性从何而来？ | 经典力学 / 机器人学导论 |
| 2 | **SE(3) 齐次变换**：给定 $R \in SO(3)$、$t \in \mathbb{R}^3$，写出 $T \in SE(3)$ 及其逆 $T^{-1} = \begin{bmatrix} R^\top & -R^\top t \\ 0 & 1 \end{bmatrix}$。为什么不是简单的矩阵转置？ | 李群基础 / Sophus 章节 |
| 3 | **URDF 关节类型**：`revolute` / `continuous` / `prismatic` / `fixed` / `floating` / `planar` 各自的自由度是多少？`revolute` 和 `continuous` 的区别是什么？ | P01 URDF/Xacro 建模 |
| 4 | **CRTP 模式**：写出 `template<class Derived> class Base { ... }` 的 CRTP 骨架，解释它如何在编译期实现"静态多态"并消除虚函数开销。 | 02_基础/C++ 高级（CRTP 与设计模式） |
| 5 | **线程安全**：什么是 data race？为什么 `const` 引用 + 不可变对象能避免 data race？在 MPC 求解器中，为什么需要"一个 Model 多个 Data"？ | C++ 并发基础 |

---

## 本章目标

学完本章后，你应该能够：

1. **解释** Pinocchio 3.x 的三大设计支柱——CRTP 静态多态 + Eigen 对齐 + 模板标量参数化（`double` / `CppAD::AD<double>` / `casadi::SX`），并与 Drake / DART / RBDL 的设计做对比
2. **使用** `pinocchio::urdf::buildModel()` 加载任意 URDF，理解 Model/Data 分离模式的多线程含义，独立完成 Franka Panda 的完整加载流程
3. **调用** 正运动学（FK）、雅可比（Jacobian）、逆动力学（RNEA）、正动力学（ABA）、惯量矩阵（CRBA）的完整 C++ 和 Python API，理解每个函数的输入输出语义
4. **区分** `LOCAL` / `WORLD` / `LOCAL_WORLD_ALIGNED` 三种参考坐标系对雅可比矩阵的影响，在速度级 IK 和力控场景中正确选择
5. **推导** O(N) 递归算法（RNEA 两遍递推、ABA 三遍递推）的原理，能手动跟踪 3-DOF 机械臂的完整递推过程
6. **配置** Coal（原 hpp-fcl）碰撞检测管线，理解 `computeABADerivatives` / `computeRNEADerivatives` 对 MPC 的性能意义

---

## 1. 为什么需要动力学引擎 ⭐

### 1.1 MPC 的 1ms 约束

考虑一个典型的模型预测控制（MPC）场景：

| 参数 | 典型值 | 说明 |
|------|--------|------|
| 控制频率 | 1 kHz | 伺服周期 1 ms |
| 预测步数 $N$ | 20~50 | 每步需要动力学评估 |
| 每步计算 | RNEA + ABA + Jacobian | 至少 3 次动力学调用 |
| 总调用次数 | 60~150 次/ms | 全部必须在 1 ms 内完成 |

对于 7-DOF Franka Panda，惯量矩阵 $M(q) \in \mathbb{R}^{7 \times 7}$ 是对称正定的，有 $7 \times 8 / 2 = 28$ 个独立元素。每个元素都是关节角 $q$ 的非线性三角函数组合。手写这 28 个公式：

- 容易出错——漏掉一个交叉惯量项就导致控制器不稳定
- 维护困难——换一个机械臂就要重新推导
- 无法自动微分——MPC 需要 $\partial \tau / \partial q$、$\partial \tau / \partial \dot{q}$ 等解析梯度

**结论**：工业级机械臂控制离不开一个高效、通用、可微分的动力学引擎。

### 1.2 手写动力学的困境

以 2-DOF 平面机械臂为例，若 $I_1, I_2$ 表示两个连杆绕各自质心、垂直于平面的转动惯量，$l_{c1}, l_{c2}$ 表示关节到质心的距离，则惯量矩阵已经是：

$$M(q) = \begin{bmatrix} I_1 + m_1 l_{c1}^2 + I_2 + m_2(l_1^2 + l_{c2}^2 + 2l_1 l_{c2}\cos q_2) & I_2 + m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) \\ I_2 + m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) & I_2 + m_2 l_{c2}^2 \end{bmatrix}$$

这里的 $m_1 l_{c1}^2$ 和 $m_2 l_{c2}^2$ 是平行轴项。若某本教材把 $I_1, I_2$ 定义为已经平移到关节轴上的惯量，上式中的对应平行轴项会被吸收到 $I_1, I_2$ 里；工程实现中必须先确认惯量张量的参考点。

仅仅 2-DOF 就已经相当复杂。对于 7-DOF 机械臂，$M(q)$ 的每个元素包含数十项三角函数嵌套。科里奥利矩阵 $C(q,\dot{q})$ 更是涉及 Christoffel 符号，手写几乎不可行。

```
2-DOF:   手写 ~30 分钟, 可以接受
7-DOF:   手写 ~1 周, 极易出错
14-DOF:  双臂/腿足, 手写不现实
通用:    需要算法化 → 这就是动力学引擎
```

### 1.3 从 ROBOOP 到 Pinocchio：历史演进

| 年代 | 库 | 语言 | 算法 | 局限 |
|------|------|------|------|------|
| 1990s | ROBOOP | C++ | DH 参数法 | 串行链专用，无 SE(3) |
| 2000s | KDL (Orocos) | C++ | 递归 Newton-Euler | 无自动微分，API 老旧 |
| 2010s | RBDL | C++ | Featherstone | 纯过程式，无碰撞检测 |
| 2012 | DART | C++ | Featherstone + LCP | 重仿真，MPC 性能不足 |
| 2014 | Drake | C++ | MultibodyPlant | 全栈但重量级，编译慢 |
| 2015 | **Pinocchio** | C++ | Featherstone + 解析微分 | INRIA 学派核心引擎 |

### 1.4 四大动力学库对比

| 维度 | Pinocchio 3.x | Drake | DART | RBDL |
|------|--------------|-------|------|------|
| **核心抽象** | Model/Data + Free Fn | MultibodyPlant (OOP) | Skeleton/BodyNode (OOP) | Model (过程式) |
| **多态策略** | CRTP 静态多态 | 虚函数 + 系统框架 | 虚函数 | C 风格函数 |
| **自动微分** | CppAD / CasADi / CodeGen | Drake AD | 有限差分 | 有限差分 |
| **碰撞检测** | Coal (hpp-fcl) | 内置 SceneGraph | FCL/DART | 无内置 |
| **Python** | eigenpy zero-copy | pydrake | dartpy | 无官方 |
| **线程安全** | 1 Model + N Data | Context 机制 | 非线程安全 | 手动管理 |
| **MPC 适配** | 原生支持（OCS2/Crocoddyl） | 原生支持 | 需额外集成 | 需额外集成 |
| **编译时间** | 中等（头文件模板） | 极长（Bazel） | 中等 | 快 |
| **7-DOF RNEA** | ~2 $\mu$s | ~10 $\mu$s | ~8 $\mu$s | ~3 $\mu$s |

> ⚠️ **常见陷阱**：性能数字因硬件、编译选项（`-O3 -march=native`）、是否启用 SIMD 而差异很大。上表仅供数量级参考，切勿直接引用于论文。严格对比请使用 `pinocchio-benchmarks` 仓库自行测量。

### 1.5 Pinocchio 的 INRIA 学派定位

```
                    ┌─────────────────────────────────────────┐
                    │         Pinocchio (动力学内核)           │
                    │   FK / Jacobian / RNEA / ABA / CRBA    │
                    │   解析微分 / CodeGen / Coal 碰撞        │
                    └────────────┬────────────────────────────┘
                                 │
              ┌──────────────────┼──────────────────────┐
              │                  │                      │
     ┌────────▼─────┐  ┌────────▼────────┐  ┌─────────▼────────┐
     │   Crocoddyl   │  │      OCS2       │  │    Aligator      │
     │  (DDP/iLQR)   │  │  (SLQ/iLQR)    │  │ (次世代 OC)      │
     └───────┬───────┘  └───────┬─────────┘  └────────┬─────────┘
             │                  │                     │
     ┌───────▼──────────────────▼─────────────────────▼──────────┐
     │              TSID / 全身控制 / 力控                        │
     └───────────────────────────┬────────────────────────────────┘
                                 │
     ┌───────────────────────────▼────────────────────────────────┐
     │              MoveIt2 / ROS 2 集成层                        │
     └────────────────────────────────────────────────────────────┘
```

Pinocchio 不是仿真器（不像 MuJoCo / Isaac），而是一个 **纯计算库**：给定关节状态，输出运动学/动力学量。这种"无状态计算"设计使它天然适合嵌入到任何控制/规划框架中。

> **跨领域类比**：Pinocchio 之于 MPC/控制框架，就像 BLAS/LAPACK 之于科学计算——它提供高度优化的基础计算原语（FK、RNEA、ABA），上层框架（Crocoddyl、OCS2、TSID）在其上构建算法。你不会用 BLAS 直接写应用程序，同样你也不会只用 Pinocchio 完成一个控制系统——但离开它，上层框架就失去了计算基础。

> **反事实推理**：如果不使用专门的动力学引擎而是手写动力学代码会怎样？对于 2-DOF 平面臂，手写 $M(q)$ 可以接受（约 30 分钟）。但对于 7-DOF Franka Panda，惯量矩阵的每个元素包含数十项三角函数嵌套，手写至少一周，且极易出错——一个符号错误就导致控制器不稳定。更关键的是，MPC 需要 $\partial \tau / \partial q$ 等解析梯度——手写梯度的工作量比动力学本身还大。Pinocchio 的解析导数功能在几微秒内完成这一切。

> **本质洞察**：Pinocchio 的 Model/Data 分离模式不仅仅是"好的软件工程"——它体现了**物理规律的数学性质**：运动方程的结构（拓扑连接、关节类型）是常量（Model），而具体的数值状态（位型、速度）是变量（Data）。分离常量和变量使得多线程并行成为自然的推论：多个控制器/MPC 求解器共享同一物理结构，各自维护独立的状态。

### 练习

1. ⭐ 用 Python 计算 7-DOF 机械臂惯量矩阵的独立元素数，验证 $n(n+1)/2 = 28$。对于 12-DOF 双臂系统呢？
2. ⭐ 列举你见过的至少 3 个使用 Pinocchio 的开源项目（提示：搜索 GitHub `pinocchio` + `robot`），记录它们分别用于机械臂还是腿足。
3. ⭐⭐ 解释为什么 Pinocchio 选择"纯计算库"而非"仿真器"的定位。MuJoCo 做仿真时，内部的动力学计算和 Pinocchio 有什么本质区别？

---

## 2. Pinocchio 设计哲学 ⭐⭐

### 2.1 三大设计支柱

Pinocchio 3.x 的架构建立在三个核心设计决策之上：

| 支柱 | 技术选择 | 解决的问题 |
|------|---------|-----------|
| **静态多态** | CRTP (Curiously Recurring Template Pattern) | 消除虚函数开销，1 kHz 调用无性能损失 |
| **内存对齐** | Eigen 对齐 + `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` | SIMD 向量化，SE(3) 运算加速 2~4x |
| **标量模板** | `ModelTpl<Scalar>` 参数化 | 同一套代码支持 `double` / AD / CasADi |

这三个支柱相互支撑：CRTP 使得模板展开在编译期完成，Eigen 对齐确保数值计算的 SIMD 友好性，标量参数化使得同一算法可以无缝切换到自动微分。

### 2.2 Model/Data 分离模式

这是 Pinocchio 最核心的设计模式，理解它是使用整个库的前提。

```cpp
// Model: 描述机器人的 **结构** (不可变)
//   - 关节类型、父子关系、惯量参数
//   - 创建后不再修改 → const 引用安全传递
pinocchio::Model model;
pinocchio::urdf::buildModel("panda.urdf", model);

// Data: 存储 **计算结果** (可变)
//   - FK 的位姿、Jacobian 矩阵、RNEA 的力矩
//   - 每次调用算法后更新
pinocchio::Data data(model);  // 必须从 model 构造
```

**设计动机对比**：

| 传统 OOP 设计 | Pinocchio Model/Data |
|-------------|---------------------|
| `robot.computeFK(q)` → 修改 `robot` 内部状态 | `forwardKinematics(model, data, q)` → 只修改 `data` |
| 多线程需要加锁保护 `robot` 对象 | `model` 是 `const`，无需加锁 |
| 状态隐藏在对象内部，难以序列化 | `data` 是纯数据结构，易于序列化 |
| 难以同时维护多个计算状态 | 一个 `model` + N 个 `data` |

### 2.3 线程安全：一个 Model，N 个 Data

```cpp
// MPC 场景: 主线程 + N 个 worker 线程
pinocchio::Model model;
pinocchio::urdf::buildModel("panda.urdf", model);
// 此后 model 视为 const

// 每个线程创建自己的 Data（必须在单线程中完成构造）
std::vector<pinocchio::Data> thread_data(num_threads, pinocchio::Data(model));

// 并行计算不同预测步
#pragma omp parallel for
for (int i = 0; i < N_horizon; ++i) {
    int tid = omp_get_thread_num();
    pinocchio::rnea(model, thread_data[tid], q[i], v[i], a[i]);
    // 每个线程写自己的 data → 无 data race
}
```

> ⚠️ **常见陷阱**：`pinocchio::Data` 的构造不是线程安全的——必须在 **单线程** 中完成所有 `Data` 对象的构造，然后再分发给各线程。构造过程中会分配 Eigen 对齐内存，并发构造可能导致 heap corruption。

### 2.4 算法即自由函数

Pinocchio 刻意避免面向对象的 `model.computeFK(data, q)` 风格，而采用 **自由函数**：

```cpp
// 所有算法都是 namespace pinocchio 下的自由函数
pinocchio::forwardKinematics(model, data, q);          // FK
pinocchio::computeJointJacobians(model, data, q);      // Jacobian
pinocchio::rnea(model, data, q, v, a);                 // 逆动力学
pinocchio::aba(model, data, q, v, tau);                // 正动力学
pinocchio::crba(model, data, q);                        // 惯量矩阵
```

**为什么用自由函数而不是成员函数？**

| 设计考量 | 自由函数优势 |
|---------|------------|
| **正交性** | 算法与数据结构解耦，新增算法不修改 Model/Data 类 |
| **模板友好** | 自由函数更容易做模板特化（如 CppAD 版本） |
| **ADL 查找** | 参数依赖查找使得 `pinocchio::rnea(...)` 可以自动选择正确的模板实例 |
| **测试性** | 可以单独测试每个算法，不需要构造复杂的对象层次 |

### 2.5 与 Drake OOP 和 RBDL 过程式的对比

```
Pinocchio (Model/Data + Free Fn):
  ┌─────────┐    ┌──────────┐
  │  Model   │    │   Data   │
  │ (const)  │    │ (mutable)│
  └────┬─────┘    └────┬─────┘
       │               │
       └───────┬───────┘
               │
       rnea(model, data, q, v, a)
       aba(model, data, q, v, tau)

Drake (OOP):
  ┌─────────────────────┐
  │   MultibodyPlant     │
  │ ├─ AddRigidBody()   │
  │ ├─ AddJoint()       │
  │ └─ CalcInverseDyn() │──→ Context (类似 Data)
  └─────────────────────┘

RBDL (过程式):
  Model model;
  InverseDynamics(model, Q, QDot, QDDot, Tau);
  // model 既是结构又是状态 → 线程不安全
```

| 维度 | Pinocchio | Drake | RBDL |
|------|-----------|-------|------|
| 模型/数据 | 严格分离 | Context 近似分离 | 混合 |
| 多态 | CRTP 编译期 | 虚函数运行期 | 无多态 |
| 扩展性 | 新增自由函数 | 继承 LeafSystem | 修改源码 |
| 学习曲线 | 中等（需理解模板） | 陡峭（系统框架庞大） | 平缓 |

### 练习

1. ⭐ 画出 `Model` 和 `Data` 的依赖关系图。为什么 `Data` 必须从 `Model` 构造？如果 `Model` 修改后 `Data` 不重建会怎样？
2. ⭐⭐ 在 8 线程 MPC 场景中，写出创建和分发 `Data` 对象的正确代码模板。如果错误地让 8 个线程共享一个 `Data`，最可能出现什么错误？
3. ⭐⭐ 解释为什么 RBDL 的设计（模型即状态）在多线程场景下是危险的。Drake 的 `Context` 机制如何缓解这个问题？

---

## 3. CRTP 访问者模式精读 ⭐⭐⭐

### 3.1 问题：虚函数在 1 kHz 调用下的开销

对于机械臂控制，典型的调用频率分析：

| 层级 | 频率 | 每周期动力学调用 | 累计 |
|------|------|---------------|------|
| 伺服控制 | 1 kHz | 1x RNEA (重力补偿) | 1,000/s |
| MPC 求解 | 100 Hz | 50x (RNEA + ABA) | 10,000/s |
| 轨迹优化 | 10 Hz | 200x RNEA + 梯度 | 2,000/s |
| **合计** | — | — | ~13,000/s |

每次 RNEA 调用会遍历所有关节。对于 7-DOF 机械臂，如果用虚函数：

```
虚函数开销 ≈ 5 ns/call × 7 joints × 13,000 calls/s = 0.455 ms/s
```

看似不多，但在 MPC 内循环中，虚函数还会导致 **分支预测失败** 和 **指令缓存污染**：

| 开销来源 | 虚函数 | CRTP |
|---------|--------|------|
| 间接跳转 | ~5 ns/call | 0（内联） |
| 分支预测失败 | ~15 ns（冷路径） | 0 |
| icache 污染 | 每种关节类型一个入口 | 编译器合并相同代码 |
| 内联可能性 | 无法内联 | 完全内联 |

### 3.2 CRTP 基础回顾

CRTP 的核心思想：**基类通过模板参数"知道"派生类的类型，在编译期完成分派**。

```cpp
// 经典 CRTP 骨架
template <typename Derived>
struct JointModelBase {
    // 编译期向下转型：无虚函数开销
    Derived& derived() { return static_cast<Derived&>(*this); }
    const Derived& derived() const { return static_cast<const Derived&>(*this); }

    // "虚函数"的 CRTP 等价：调用派生类实现
    template <typename ConfigVector>
    void calc(JointDataBase<typename Derived::JointDataDerived>& data,
              const Eigen::MatrixBase<ConfigVector>& q) const {
        derived().calc_impl(data.derived(), q);  // 编译期分派
    }
};
```

### 3.3 Pinocchio 的关节类型层次

```
JointModelBase<Derived>                    ← CRTP 基类
├── JointModelRevoluteTpl<Scalar, axis>    ← 旋转关节 (最常用)
├── JointModelPrismaticTpl<Scalar, axis>   ← 平移关节
├── JointModelFreeFlyerTpl<Scalar>         ← 浮动基座 (6-DOF)
├── JointModelSphericalTpl<Scalar>         ← 球关节 (3-DOF, 四元数)
├── JointModelPlanarTpl<Scalar>            ← 平面关节 (3-DOF)
├── JointModelCompositeTpl<Scalar>         ← 复合关节
└── JointModelMimic<JointModel>            ← 模仿关节 (v3.x 新增)
```

每种关节类型都实现了 `calc_impl` / `calc_aba_impl` 等方法。CRTP 确保这些方法在编译期被静态解析。

### 3.4 JointModelRevoluteTpl 源码精读

以最常用的旋转关节为例：

```cpp
template <typename _Scalar, int _axis>
struct JointModelRevoluteTpl
    : public JointModelBase<JointModelRevoluteTpl<_Scalar, _axis>>
{
    // 类型别名
    typedef _Scalar Scalar;
    typedef JointDataRevoluteTpl<Scalar, _axis> JointDataDerived;

    // nq=1 (配置空间维度), nv=1 (速度空间维度)
    // 对于旋转关节: q ∈ R^1, v ∈ R^1
    enum { NQ = 1, NV = 1 };

    // calc_impl: 给定关节角 q, 计算关节变换
    template <typename ConfigVector>
    void calc_impl(JointDataDerived& data,
                   const Eigen::MatrixBase<ConfigVector>& q) const
    {
        const Scalar& angle = q[idx_q()];  // 取出本关节的角度

        // 根据 _axis 模板参数, 编译期选择旋转轴
        // _axis=0 → X轴, _axis=1 → Y轴, _axis=2 → Z轴
        data.joint_transform.rotation() =
            Eigen::AngleAxis<Scalar>(angle,
                Eigen::Matrix<Scalar,3,1>::Unit(_axis));

        // 运动子空间 S: 旋转关节只有一个自由度
        // S = [0 0 0 e_i]^T (6x1, e_i 是旋转轴方向)
        data.S.setZero();
        data.S.angular()[_axis] = Scalar(1);
    }
};
```

**关键洞察**：`_axis` 是模板参数，编译器在实例化时就知道旋转轴方向，因此：
- `Unit(_axis)` 被常量折叠为 `[1,0,0]` / `[0,1,0]` / `[0,0,1]`
- `data.S.angular()[_axis] = 1` 被优化为单条 `mov` 指令
- 整个 `calc_impl` 可能被内联到调用者中

### 3.5 模板展开：编译器生成了什么

当你写 `JointModelRevoluteTpl<double, 2>` 时，编译器会生成一个 **完全特化** 的类，等价于：

```cpp
// 编译器生成的等价代码 (Z轴旋转关节, _axis=2)
struct JointModelRevoluteZ_double {
    void calc_impl(JointDataRevoluteZ_double& data,
                   double angle) const {
        // cos/sin 直接计算, 无分支
        double c = std::cos(angle);
        double s = std::sin(angle);

        // 绕 Z 轴旋转矩阵, 编译器已知哪些元素是 0/1
        data.joint_transform.rotation() <<
            c, -s, 0,
            s,  c, 0,
            0,  0, 1;

        // 运动子空间: [0,0,0, 0,0,1]^T
        data.S << 0, 0, 0, 0, 0, 1;
    }
};
```

### 3.6 JointModelVariant：运行时多态的桥梁

虽然 CRTP 提供了编译期多态，但 URDF 加载时关节类型是运行时确定的。Pinocchio 用 `boost::variant`（C++17 后为 `std::variant`）来桥接：

```cpp
// Model 内部存储关节的方式
typedef boost::variant<
    JointModelRevoluteTpl<Scalar, 0>,  // X轴旋转
    JointModelRevoluteTpl<Scalar, 1>,  // Y轴旋转
    JointModelRevoluteTpl<Scalar, 2>,  // Z轴旋转
    JointModelPrismaticTpl<Scalar, 0>, // X轴平移
    JointModelPrismaticTpl<Scalar, 1>, // Y轴平移
    JointModelPrismaticTpl<Scalar, 2>, // Z轴平移
    JointModelFreeFlyerTpl<Scalar>,    // 浮动基座
    JointModelSphericalTpl<Scalar>,    // 球关节
    // ... 更多关节类型
> JointModelVariant;

// Model 中的关节列表
std::vector<JointModelVariant> joints;
```

算法遍历关节时，用 **visitor 模式** 分派：

```cpp
// 访问者: 对每种关节类型调用对应的 calc
struct JointCalcVisitor : boost::static_visitor<void> {
    template <typename JointModel>
    void operator()(const JointModel& jmodel,
                    typename JointModel::JointDataDerived& jdata,
                    const Eigen::VectorXd& q) const {
        jmodel.calc(jdata, q);  // CRTP 分派 → 编译期解析
    }
};

// 遍历所有关节
for (int i = 1; i < model.njoints; ++i) {
    boost::apply_visitor(
        JointCalcVisitor{},
        model.joints[i],
        data.joints[i]
    );
}
```

**性能特征**：`boost::apply_visitor` 内部使用 **跳转表**（类似 `switch`），比虚函数的间接指针查找更高效，且对分支预测更友好（关节类型通常在循环中是固定的）。

> ⚠️ **常见陷阱**：不要自己实现关节类型的运行时分派。Pinocchio 的 visitor 模式已经高度优化，自定义分派通常会更慢且更容易出错。如果需要新关节类型，应该继承 `JointModelBase` 并注册到 `JointModelVariant`。

### 练习

1. ⭐⭐ 画出 `JointModelRevoluteTpl<double, 2>` 的 CRTP 继承链，标注每一层提供的功能。
2. ⭐⭐⭐ 用 `objdump -d` 或 Compiler Explorer (godbolt.org) 查看 `JointModelRevoluteTpl<double, 2>::calc_impl` 的汇编代码，验证 `_axis=2` 是否被常量折叠。
3. ⭐⭐⭐ 对比 `boost::variant` + visitor 和 `std::visit` + `std::variant` 的性能差异。Pinocchio 3.x 是否已迁移到 `std::variant`？查阅最新源码确认。

---

## 4. 标量参数化 ⭐⭐⭐

### 4.1 问题：同一算法，多种数值类型

MPC 求解器需要动力学的 **数值评估** 和 **解析梯度**。传统做法：

```
方案 A: 手写两套代码 (double 版 + 微分版) → 维护噩梦
方案 B: 有限差分 → 精度差, 7-DOF 需要 14 次额外 RNEA 调用
方案 C: 标量参数化 → Pinocchio 的选择 ✓
```

### 4.2 ModelTpl<Scalar> 的模板魔法

```cpp
// Pinocchio 的 Model 实际上是一个模板类
template <typename _Scalar, int _Options = 0>
struct ModelTpl {
    typedef _Scalar Scalar;

    // 所有数值成员都用 Scalar 参数化
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;
    typedef SE3Tpl<Scalar, _Options> SE3;
    typedef InertiaTpl<Scalar, _Options> Inertia;

    std::vector<Inertia> inertias;       // Scalar 类型的惯量
    std::vector<SE3> jointPlacements;     // Scalar 类型的位姿
    // ...
};

// 常用的具体类型别名
typedef ModelTpl<double> Model;                    // 数值计算
typedef ModelTpl<CppAD::AD<double>> ADModel;       // CppAD 自动微分
typedef ModelTpl<casadi::SX> CasadiModel;          // CasADi 符号计算
```

### 4.3 同一套 FK 代码，三种标量

```cpp
// 模板化的 FK 函数——对任意 Scalar 都有效
template <typename Scalar>
void forwardKinematics(
    const ModelTpl<Scalar>& model,
    DataTpl<Scalar>& data,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q)
{
    for (int i = 1; i < model.njoints; ++i) {
        // 所有运算都在 Scalar 类型上进行
        // Scalar=double      → 普通浮点运算
        // Scalar=AD<double>  → 同时记录计算图
        // Scalar=casadi::SX  → 构建符号表达式
        data.oMi[i] = data.oMi[model.parents[i]] * data.liMi[i];
    }
}
```

### 4.4 三种标量类型的使用场景

| 标量类型 | 典型场景 | 性能 | 精度 | 使用难度 |
|---------|---------|------|------|---------|
| `double` | 实时控制、仿真 | 最快 (~2 $\mu$s RNEA) | 机器精度 | ⭐ |
| `CppAD::AD<double>` | 离线梯度计算、NLP | 慢 3~5x | 机器精度（解析） | ⭐⭐ |
| `casadi::SX` | 符号推导、CodeGen | 非实时 | 符号精确 | ⭐⭐⭐ |

### 4.5 CppAD 自动微分示例

```cpp
#include <pinocchio/autodiff/cppad.hpp>

// 1. 创建 AD 模型
typedef CppAD::AD<double> ADScalar;
typedef pinocchio::ModelTpl<ADScalar> ADModel;
typedef pinocchio::DataTpl<ADScalar> ADData;

// 从 double 模型转换
pinocchio::Model model;
pinocchio::urdf::buildModel("panda.urdf", model);
ADModel ad_model = model.cast<ADScalar>();  // 类型转换
ADData ad_data(ad_model);

// 2. 声明自变量
// CppAD::Independent 推荐使用 CppAD 支持的 SimpleVector 容器。
// 这里先用 std::vector 录制 tape，再用 Eigen::Map 传给 Pinocchio。
std::vector<ADScalar> ad_x(model.nq);
for (int i = 0; i < model.nq; ++i)
    ad_x[i] = ADScalar(q_nominal[i]);  // 从 double 值初始化

CppAD::Independent(ad_x);  // 标记为自变量

Eigen::Map<Eigen::Matrix<ADScalar, Eigen::Dynamic, 1>> ad_q(
    ad_x.data(), model.nq);

// 3. 调用相同的 FK 函数——自动记录计算图
pinocchio::forwardKinematics(ad_model, ad_data, ad_q);

// 4. 提取末端位姿 (作为因变量)
Eigen::Matrix<ADScalar, 3, 1> ad_pos =
    ad_data.oMi[model.njoints-1].translation();

// 5. 创建 AD 函数并求雅可比
std::vector<ADScalar> pos_out = {ad_pos[0], ad_pos[1], ad_pos[2]};
CppAD::ADFun<double> ad_fun(ad_x, pos_out);
std::vector<double> q_nominal_vec(
    q_nominal.data(), q_nominal.data() + model.nq);
std::vector<double> jac = ad_fun.Jacobian(q_nominal_vec);
// jac 是 3x7 矩阵 (展平为向量): ∂pos/∂q
```

### 4.6 CasADi 符号计算与 CodeGen

Pinocchio 的 C++ Eigen API 不能直接接收一个裸的 `casadi::SX` 矩阵作为 `q`；要么使用 `pinocchio::casadi` 提供的桥接工具把符号变量复制到 `Eigen::Matrix<casadi::SX, ...>`，要么在 Python 中使用 `pinocchio.casadi` 封装。教学中推荐先用 Python 路径验证符号图：

```python
import casadi as ca
import pinocchio as pin
import pinocchio.casadi as cpin

model = pin.buildModelFromUrdf("panda.urdf")
cmodel = cpin.Model(model)
cdata = cmodel.createData()

# CasADi 符号变量，维度必须与 Pinocchio model.nq 一致。
cq = ca.SX.sym("q", model.nq, 1)

# 符号 FK
cpin.forwardKinematics(cmodel, cdata, cq)
cpin.updateFramePlacements(cmodel, cdata)

ee_id = model.getFrameId("panda_hand")
cpos = cdata.oMf[ee_id].translation

# 生成 C 代码 → 编译为动态库 → 实时调用
fk_fun = ca.Function("fk", [cq], [cpos])
fk_fun.generate("fk_codegen.c")
# 编译: gcc -O3 -shared -fPIC -o libfk.so fk_codegen.c
```

### 4.7 标量类型选择决策树

```
你需要什么？
│
├─ 实时控制 (< 1ms) ──→ double
│   └─ 需要梯度？
│       ├─ 否 ──→ pinocchio::rnea(...) 直接调用
│       └─ 是 ──→ computeRNEADerivatives() (解析微分, 仍用 double)
│
├─ 离线优化 (NLP/轨迹优化) ──→ CppAD::AD<double>
│   └─ 需要高阶导数？
│       ├─ 否 ──→ CppAD 一阶 Jacobian
│       └─ 是 ──→ CppAD 嵌套 AD<AD<double>>
│
└─ 代码生成 (部署到嵌入式) ──→ casadi::SX
    └─ 生成 C 代码 → 交叉编译 → 实时执行
```

> ⚠️ **常见陷阱**：`model.cast<NewScalar>()` 会 **深拷贝** 整个模型。对于大型机器人（如人形，30+ 关节），这个操作代价不可忽略。应在初始化阶段完成一次，而不是每帧调用。

### 练习

1. ⭐⭐ 用 `model.cast<CppAD::AD<double>>()` 将 Franka Panda 模型转换为 AD 模型，测量转换耗时。
2. ⭐⭐⭐ 比较 `computeRNEADerivatives()` (Pinocchio 内置解析微分) 和 CppAD 自动微分得到的 $\partial \tau / \partial q$ 矩阵，验证数值一致性（误差 < $10^{-10}$）。
3. ⭐⭐⭐ 用 CasADi 生成 Franka Panda 逆动力学的 C 代码，编译后测量单次调用耗时，与 Pinocchio `rnea()` 的 `double` 版本做对比。

---

## 5. URDF 加载与 Model 构建 ⭐

### 5.1 buildModel() 内部流程

```cpp
pinocchio::Model model;
pinocchio::urdf::buildModel(
    "panda.urdf",           // URDF 文件路径
    model                   // 输出: 构建好的模型
);
// 带根关节的版本 (腿足/移动机器人):
// pinocchio::urdf::buildModel("robot.urdf",
//     pinocchio::JointModelFreeFlyer(), model);
```

内部流程：

```
URDF 文件
  │
  ├─ 1. tinyxml2 解析 XML
  │
  ├─ 2. 遍历 <link> 标签
  │     └─ 提取 <inertial>: mass, CoM, inertia tensor
  │
  ├─ 3. 遍历 <joint> 标签
  │     ├─ type="revolute"  → JointModelRevoluteTpl<Scalar, axis>
  │     ├─ type="prismatic" → JointModelPrismaticTpl<Scalar, axis>
  │     ├─ type="fixed"     → 不创建关节, 合并到父连杆
  │     └─ type="continuous" → 同 revolute, 但无限位
  │
  ├─ 4. 构建运动树 (kinematic tree)
  │     └─ model.parents[i] = 父关节索引
  │
  └─ 5. 添加操作帧 (operational frames)
        └─ 关节帧、连杆帧、传感器帧等
```

### 5.2 关节排序与帧概念

Pinocchio 的关节索引从 1 开始（0 保留给"宇宙关节" universe joint）：

| 索引 | 含义 | 说明 |
|------|------|------|
| 0 | universe | 固定参考系，所有计算的起点 |
| 1 | 第一个可动关节 | URDF 中第一个非 fixed 关节 |
| ... | ... | 按 URDF 深度优先遍历顺序 |
| `model.njoints-1` | 最后一个可动关节 | — |

**帧 (Frame)** 是比关节更细粒度的概念：

```cpp
// Frame 类型枚举
enum FrameType {
    OP_FRAME,     // 操作帧 (用户自定义)
    JOINT,        // 关节帧
    FIXED_JOINT,  // 固定关节帧 (URDF 中 type="fixed")
    BODY,         // 连杆帧
    SENSOR        // 传感器帧
};

// 获取帧索引和信息
pinocchio::FrameIndex ee_id = model.getFrameId("panda_hand");
pinocchio::Frame& frame = model.frames[ee_id];
// frame.parentJoint  → 该帧所属的关节索引
// frame.placement    → 该帧相对于父关节的 SE(3) 变换
```

> ⚠️ **常见陷阱**：URDF 中的 `fixed` 关节不会创建 Pinocchio 关节（`nq=0, nv=0`），但会创建一个 `FIXED_JOINT` 类型的帧。如果你用关节索引去访问末端执行器（通常通过 fixed joint 连接到最后一个活动关节），你会得到错误的位姿。**始终用帧索引 + `updateFramePlacements`**。

### 5.3 加载 Franka Panda：完整代码

```cpp
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <iostream>

int main() {
    // ========== 1. 加载 URDF ==========
    const std::string urdf_path =
        "/path/to/franka_description/robots/panda.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    // 固定基座机械臂: 不需要指定根关节类型

    // ========== 2. 检查模型属性 ==========
    std::cout << "===== Franka Panda 模型信息 =====" << std::endl;
    std::cout << "关节数 (含 universe):  " << model.njoints << std::endl;
    std::cout << "配置空间维度 nq:       " << model.nq << std::endl;
    std::cout << "速度空间维度 nv:       " << model.nv << std::endl;
    std::cout << "帧数:                 " << model.nframes << std::endl;
    std::cout << "连杆数:               " << model.nbodies << std::endl;
    // 预期输出:
    //   关节数 (含 universe):  8  (universe + 7 revolute)
    //   配置空间维度 nq:       7
    //   速度空间维度 nv:       7
    //   帧数:                 ~26 (包含 fixed joint 帧)
    //   连杆数:               8

    // ========== 3. 打印关节信息 ==========
    std::cout << "\n===== 关节列表 =====" << std::endl;
    for (int i = 0; i < model.njoints; ++i) {
        std::cout << "Joint " << i << ": " << model.names[i]
                  << " | idx_q=" << model.idx_qs[i]
                  << " | idx_v=" << model.idx_vs[i]
                  << " | nq=" << model.nqs[i]
                  << " | nv=" << model.nvs[i] << std::endl;
    }

    // ========== 4. 打印帧信息 ==========
    std::cout << "\n===== 关键帧 =====" << std::endl;
    for (size_t i = 0; i < model.nframes; ++i) {
        const pinocchio::Frame& frame = model.frames[i];
        if (frame.name.find("panda") != std::string::npos) {
            std::cout << "Frame " << i << ": " << frame.name
                      << " | type=" << frame.type
                      << " | parent_joint="
                      << frame.parentJoint << std::endl;
        }
    }

    // ========== 5. 创建 Data ==========
    pinocchio::Data data(model);
    // data 内部预分配了所有需要的矩阵:
    //   data.oMi:  vector<SE3>, size = njoints
    //   data.M:    MatrixXd,    size = nv x nv (惯量矩阵)
    //   data.J:    MatrixXd,    size = 6 x nv  (雅可比矩阵)
    //   data.tau:  VectorXd,    size = nv      (关节力矩)

    std::cout << "\n模型加载成功！" << std::endl;
    return 0;
}
```

### 5.4 nq 与 nv 的区别——配置空间 vs 速度空间

这是初学者最容易混淆的概念之一：

| 关节类型 | nq (配置维度) | nv (速度维度) | 原因 |
|---------|:---:|:---:|------|
| Revolute | 1 | 1 | 角度 $\theta \in \mathbb{R}$，角速度 $\dot\theta \in \mathbb{R}$ |
| Prismatic | 1 | 1 | 位移 $d \in \mathbb{R}$，速度 $\dot d \in \mathbb{R}$ |
| FreeFlyer | **7** | **6** | 位置 $\mathbb{R}^3$ + 四元数 $\mathbb{S}^3$（4维）vs $\mathbb{R}^3 + \mathfrak{so}(3)$（3维） |
| Spherical | **4** | **3** | 四元数（4维）vs 角速度（3维） |
| Planar | 3 | 3 | $(x, y, \theta)$ 均为欧氏空间 |

**核心洞察**：配置空间 $\mathcal{Q}$ 不一定是欧氏空间（四元数在球面 $\mathbb{S}^3$ 上），而速度空间 $T_q\mathcal{Q}$ 总是欧氏空间。这就是 nq $\neq$ nv 的根本原因。

| 量 | 维度 | 用途 |
|----|------|------|
| `q` | `model.nq` | FK, RNEA, ABA, CRBA 的配置输入 |
| `v` ($\dot{q}$) | `model.nv` | 速度输入 |
| `a` ($\ddot{q}$) | `model.nv` | 加速度输入 |
| `tau` ($\tau$) | `model.nv` | 力矩输出/输入 |
| `M` | `model.nv x model.nv` | 惯量矩阵 |
| `J` | `6 x model.nv` | 雅可比矩阵 |

> ⚠️ **常见陷阱**：如果你的机器人有 `FreeFlyer` 根关节（腿足/移动基座），`q` 的前 7 维是 `[x, y, z, qx, qy, qz, qw]`，但 `v` 的前 6 维是 `[vx, vy, vz, wx, wy, wz]`。**不能** 直接用 `v = (q_new - q_old) / dt` 来计算速度——必须用 `pinocchio::difference(model, q_old, q_new) / dt`。

### 5.5 CMake 构建配置

```cmake
cmake_minimum_required(VERSION 3.14)
project(pinocchio_tutorial)

# 查找 Pinocchio (自动传递 Eigen, Boost 等依赖)
find_package(pinocchio REQUIRED)

add_executable(load_panda src/load_panda.cpp)
target_link_libraries(load_panda PRIVATE pinocchio::pinocchio)

# 推荐: 开启优化和原生指令集
target_compile_options(load_panda PRIVATE -O3 -march=native)
```

### 练习

1. ⭐ 加载 Franka Panda URDF，打印所有帧的名称、类型和父关节索引。找到末端执行器帧 `panda_hand` 的帧索引。
2. ⭐ 修改代码，加载一个带 `FreeFlyer` 根关节的人形机器人 URDF（如 Talos），验证 `nq = nv + 1`（因为四元数多一维）。
3. ⭐⭐ 解释为什么 `model.njoints` 比 URDF 中的可动关节数多 1（因为包含 universe joint）。这个设计对递归算法有什么便利？

---

## 6. 正向运动学 FK 深入 ⭐⭐

### 6.1 三种 forwardKinematics 重载

Pinocchio 提供了三个不同签名的 FK 函数，对应不同的计算深度：

| 重载 | 签名 | 计算内容 | 复杂度 |
|------|------|---------|--------|
| **零阶** | `forwardKinematics(model, data, q)` | 关节位姿 ${}^0 T_i$ | $O(N)$ |
| **一阶** | `forwardKinematics(model, data, q, v)` | + 关节速度 $v_i$ | $O(N)$ |
| **二阶** | `forwardKinematics(model, data, q, v, a)` | + 关节加速度 $a_i$ | $O(N)$ |

```cpp
Eigen::VectorXd q = pinocchio::randomConfiguration(model);
Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

// 零阶: 只要位姿
pinocchio::forwardKinematics(model, data, q);

// 一阶: 位姿 + 速度
pinocchio::forwardKinematics(model, data, q, v);

// 二阶: 位姿 + 速度 + 加速度
pinocchio::forwardKinematics(model, data, q, v, a);
```

### 6.2 data.oMi 与 data.liMi

FK 计算后，关键结果存储在 `data` 中：

| 成员 | 类型 | 含义 | 尺寸 |
|------|------|------|------|
| `data.oMi[i]` | `SE3` | 关节 $i$ 相对于世界坐标系的位姿 ${}^0 T_i$ | `njoints` |
| `data.liMi[i]` | `SE3` | 关节 $i$ 相对于父关节的位姿 ${}^{p(i)} T_i$ | `njoints` |
| `data.v[i]` | `Motion` | 关节 $i$ 的空间速度（一阶 FK 后有效） | `njoints` |
| `data.a[i]` | `Motion` | 关节 $i$ 的空间加速度（二阶 FK 后有效） | `njoints` |

FK 的递推关系非常简洁：

$${}^0 T_i = {}^0 T_{p(i)} \cdot {}^{p(i)} T_i$$

其中 $p(i)$ 是关节 $i$ 的父关节索引，即 `model.parents[i]`。

```cpp
// FK 的递推核心 (简化版伪代码)
for (int i = 1; i < model.njoints; ++i) {
    int parent = model.parents[i];
    // 1. 关节局部变换: 由关节角 q 决定
    data.liMi[i] = model.jointPlacements[i] * jdata.M();
    // 2. 世界坐标系变换: 递推
    data.oMi[i] = data.oMi[parent] * data.liMi[i];
}
```

### 6.3 updateFramePlacements：获取末端执行器位姿

`forwardKinematics` 只计算 **关节** 的位姿。要获取 **帧**（包括通过 fixed joint 连接的末端执行器）的位姿，必须额外调用：

```cpp
// 1. 先做 FK
pinocchio::forwardKinematics(model, data, q);

// 2. 更新所有帧的位姿
pinocchio::updateFramePlacements(model, data);

// 3. 获取末端执行器位姿
pinocchio::FrameIndex ee_id = model.getFrameId("panda_hand");
pinocchio::SE3 ee_pose = data.oMf[ee_id];

std::cout << "末端位置: " << ee_pose.translation().transpose()
          << std::endl;
std::cout << "末端旋转:\n" << ee_pose.rotation() << std::endl;
```

> ⚠️ **常见陷阱**：忘记调用 `updateFramePlacements` 是最高频的新手错误。`data.oMf` 在 FK 后 **不会** 自动更新——Pinocchio 遵循"不做不必要的计算"原则。如果你只需要关节位姿，调用 `updateFramePlacements` 就是浪费。

### 6.4 递推 FK 推导：以 3-DOF RRR 为例

考虑一个平面 3-DOF 机械臂（3 个绕 Z 轴的旋转关节）：

```
     q1          q2          q3
  ─────○────────○────────○────── EE
       │   L1   │   L2   │  L3
     Joint1   Joint2   Joint3

递推过程:
  Step 0: T_0 = I (universe)
  Step 1: T_1 = T_0 · placement_1 · Rz(q1)
        = Rz(q1)                        (假设 placement_1 = I)
  Step 2: T_2 = T_1 · placement_2 · Rz(q2)
        = Rz(q1) · Tx(L1) · Rz(q2)
  Step 3: T_3 = T_2 · placement_3 · Rz(q3)
        = Rz(q1) · Tx(L1) · Rz(q2) · Tx(L2) · Rz(q3)

末端位置:
  x = L1·cos(q1) + L2·cos(q1+q2) + L3·cos(q1+q2+q3)
  y = L1·sin(q1) + L2·sin(q1+q2) + L3·sin(q1+q2+q3)
```

这个递推过程正是 Pinocchio `forwardKinematics` 的核心：从根关节到叶关节，逐级累乘 SE(3) 变换。复杂度严格 $O(N)$，与关节数线性相关。

### 练习

1. ⭐ 设置 Franka Panda 为零位（`q = 0`），调用 FK 并打印每个关节的世界坐标系位姿 `data.oMi[i]`。验证 Joint 1 在基座上方。
2. ⭐⭐ 比较 `data.oMi[model.njoints-1]` 和 `data.oMf[ee_id]` 的区别。在什么条件下它们相等？
3. ⭐⭐ 用一阶 FK 获取关节速度 `data.v[i]`，验证它满足 $v_i = v_{p(i)} + S_i \dot{q}_i$（其中 $S_i$ 是运动子空间）。

---

## 7. 雅可比矩阵 ⭐⭐⭐

### 7.1 雅可比的物理含义

雅可比矩阵 $J(q) \in \mathbb{R}^{6 \times n_v}$ 建立了关节速度到末端空间速度的映射：

$$\xi = J(q) \dot{q}$$

其中 $\xi = [v^\top, \omega^\top]^\top \in \mathbb{R}^6$ 是末端的空间速度（线速度 + 角速度）。

| 用途 | 公式 | 说明 |
|------|------|------|
| 速度映射 | $\xi = J\dot{q}$ | 关节空间 → 任务空间 |
| 力映射 | $\tau = J^\top F$ | 任务空间 → 关节空间（虚功原理） |
| 速度级 IK | $\dot{q} = J^\dagger \xi_d$ | 伪逆求解 |
| 奇异性检测 | $\sigma_{\min}(J) < \epsilon$ | 最小奇异值 |
| 可操作度 | $w = \sqrt{\det(JJ^\top)}$ | Yoshikawa 指标 |

### 7.2 三种参考坐标系——最关键的选择

Pinocchio 的雅可比 API 要求指定 **参考坐标系（ReferenceFrame）**，这是最容易出错的地方：

| 枚举值 | 含义 | 适用场景 |
|--------|------|---------|
| `LOCAL` | 末端连杆坐标系 $\{e\}$ 中表达 | 力控（传感器在末端） |
| `WORLD` | 世界坐标系 $\{w\}$ 中表达，但速度参考点在世界原点 | 极少使用 |
| `LOCAL_WORLD_ALIGNED` | 末端点处、与世界坐标系平行的坐标系 | **最常用**：速度 IK、MPC |

**核心区别的数学表达**：

设末端位姿为 ${}^w T_e = (R, p)$，末端在 LOCAL 坐标系中的速度为 $\xi_L$：

$$\xi_{\text{LOCAL}} = \begin{bmatrix} R^\top v \\ R^\top \omega \end{bmatrix}, \quad
\xi_{\text{LWA}} = \begin{bmatrix} v \\ \omega \end{bmatrix}, \quad
\xi_{\text{WORLD}} = \begin{bmatrix} v + p \times \omega \\ \omega \end{bmatrix}
= \begin{bmatrix} v - \omega \times p \\ \omega \end{bmatrix}$$

```
LOCAL:                世界坐标系中的末端:   LOCAL_WORLD_ALIGNED:
  ┌──┐                    ┌──┐                   ┌──┐
  │ e│ ← 速度在此系表达    │ e│                    │ e│ ← 位置在此
  └──┘                    └──┘                    └──┘
                           │                         ↕
                    ┌──────┘                    ┌─────┐
                    │ w │ ← 速度在此系表达       │ w∥  │← 方向与 w 平行
                    └───┘   参考点在 w 原点      └─────┘  但原点在 e
```

### 7.3 Pinocchio 雅可比 API

```cpp
// 方法 1: computeJointJacobians + getJointJacobian
// 先计算所有关节的雅可比，再提取特定关节
pinocchio::computeJointJacobians(model, data, q);

Eigen::MatrixXd J(6, model.nv);
J.setZero();
pinocchio::getJointJacobian(
    model, data,
    joint_id,                               // 关节索引
    pinocchio::LOCAL_WORLD_ALIGNED,         // 参考坐标系
    J                                        // 输出
);

// 方法 2: computeFrameJacobian (直接计算特定帧的雅可比)
pinocchio::FrameIndex ee_id = model.getFrameId("panda_hand");
Eigen::MatrixXd J_frame(6, model.nv);
J_frame.setZero();
pinocchio::computeFrameJacobian(
    model, data, q,
    ee_id,                                  // 帧索引
    pinocchio::LOCAL_WORLD_ALIGNED,         // 参考坐标系
    J_frame                                  // 输出
);
```

### 7.4 三种坐标系的雅可比关系

已知 LOCAL 坐标系的雅可比 $J_L$，可以推导其他两种：

$$J_{\text{LWA}} = \begin{bmatrix} R & 0 \\ 0 & R \end{bmatrix} J_L$$

$$J_{\text{WORLD}} = \begin{bmatrix} R & [p]_\times R \\ 0 & R \end{bmatrix} J_L$$

其中 $[p]_\times$ 是平移向量 $p$ 的反对称矩阵。

| 转换 | 公式 | 说明 |
|------|------|------|
| LOCAL $\to$ LWA | 左乘 $\text{diag}(R, R)$ | 只旋转，不移动参考点 |
| LOCAL $\to$ WORLD | 左乘 ${}^w X_e$ (空间变换) | 旋转 + 移动参考点到世界原点 |
| LWA $\to$ WORLD | 左乘 $\begin{bmatrix} I & [p]_\times \\ 0 & I \end{bmatrix}$ | 只移动参考点 |

> ⚠️ **常见陷阱**：在速度级 IK 中使用 `WORLD` 坐标系会引入"鬼速度"——即使末端点的线速度为 0，只要存在角速度，`WORLD` 雅可比的线速度行也会包含 $p \times \omega$（等价于 $-\omega \times p$）这一参考点平移项。如果你的任务是跟踪一个在世界系表达、参考点在末端的期望 twist，通常使用 `LOCAL_WORLD_ALIGNED`。如果你的任务是下面这种 SE(3) 对数误差 CLIK，则误差是 body-frame 量，应使用 `LOCAL` Jacobian，并显式乘 `Jlog6`。

### 7.5 雅可比时间导数与奇异性

```cpp
// 雅可比时间导数: dJ/dt（用于加速度级控制）
pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
Eigen::MatrixXd dJ(6, model.nv);
dJ.setZero();
pinocchio::getJointJacobianTimeVariation(
    model, data, joint_id,
    pinocchio::LOCAL_WORLD_ALIGNED, dJ
);
// 末端加速度: a_ee = J * ddq + dJ * dq
```

**奇异性检测**：

```cpp
// 使用 SVD 分解检测奇异性
Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
double sigma_min = svd.singularValues().tail(1)(0);
double sigma_max = svd.singularValues()(0);
double condition = sigma_max / sigma_min;

std::cout << "最小奇异值: " << sigma_min << std::endl;
std::cout << "条件数: " << condition << std::endl;
// sigma_min < 1e-3 → 接近奇异位型
// condition > 1e6  → 数值不稳定
```

### 7.6 速度级 IK 代码示例

```cpp
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>

// 速度级 IK: 从当前位型 q 移动到目标位姿 T_desired
void velocityIK(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    Eigen::VectorXd& q,               // 输入/输出: 当前关节角
    const pinocchio::SE3& T_desired,   // 目标位姿
    pinocchio::FrameIndex ee_id,       // 末端帧索引
    int max_iter = 200,                // 最大迭代次数
    double eps = 1e-4,                 // 收敛阈值
    double dt = 0.1)                   // 步长
{
    for (int i = 0; i < max_iter; ++i) {
        // 1. 计算当前末端位姿
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        const pinocchio::SE3& T_current = data.oMf[ee_id];

        // 2. Pinocchio CLIK 语义：iMd = current^{-1} * desired
        pinocchio::SE3 iMd = T_current.actInv(T_desired);
        Eigen::Matrix<double, 6, 1> err =
            pinocchio::log6(iMd).toVector();

        // 3. 检查收敛
        if (err.norm() < eps) {
            std::cout << "IK 收敛于第 " << i << " 步, "
                      << "误差 = " << err.norm() << std::endl;
            return;
        }

        // 4. 计算 LOCAL 几何雅可比，并用 Jlog6 转成误差 Jacobian
        Eigen::MatrixXd J_local(6, model.nv);
        Eigen::MatrixXd J_task(6, model.nv);
        J_local.setZero();
        pinocchio::computeFrameJacobian(
            model, data, q, ee_id,
            pinocchio::LOCAL, J_local);
        J_task.noalias() =
            -pinocchio::Jlog6(iMd.inverse()) * J_local;

        // 5. 阻尼伪逆求解 (Levenberg-Marquardt)
        double lambda = 1e-6;  // 阻尼因子
        Eigen::MatrixXd JJt = J_task * J_task.transpose();
        JJt.diagonal().array() += lambda * lambda;
        Eigen::VectorXd v =
            -J_task.transpose() * JJt.ldlt().solve(err);

        // 6. 更新关节角 (配置空间积分)
        q = pinocchio::integrate(model, q, v * dt);
    }
    std::cerr << "IK 未收敛！" << std::endl;
}
```

> ⚠️ **常见陷阱**：在步骤 6 中，**必须** 用 `pinocchio::integrate` 而非简单的 `q += dq * dt`。对于包含四元数的配置空间（如 `FreeFlyer`、`Spherical`），简单加法会破坏四元数的单位约束。`integrate` 负责把切空间增量映射回配置流形；它不是 joint limit clamp，也不会自动把结果投影到 `lowerPositionLimit` / `upperPositionLimit` 内。若 IK 需要关节限位，必须在求解器层显式加入限位、饱和或投影策略。

### 练习

1. ⭐⭐ 对 Franka Panda 在零位处计算三种坐标系的雅可比 $J_L$、$J_{LWA}$、$J_W$，验证它们之间的转换关系。
2. ⭐⭐⭐ 找到 Franka Panda 的一个奇异位型（提示：当多个关节轴共线时），计算此处的最小奇异值，验证接近 0。
3. ⭐⭐⭐ 实现完整的速度级 IK，将末端移动到指定位姿。测试当目标位姿不可达时的行为。

---

## 8. 逆动力学 RNEA ⭐⭐⭐

### 8.1 问题定义

**逆动力学** (Inverse Dynamics)：已知关节角 $q$、角速度 $\dot{q}$、角加速度 $\ddot{q}$，求所需关节力矩 $\tau$：

$$\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$$

这正是 **递归 Newton-Euler 算法 (RNEA)** 要解决的问题。

### 8.2 RNEA 两遍递推推导

RNEA 的核心是两遍遍历运动树：

**第一遍：从根到叶（正向递推）**——计算每个连杆的速度和加速度

$$v_i = X_{i,p(i)} v_{p(i)} + S_i \dot{q}_i$$
$$a_i = X_{i,p(i)} a_{p(i)} + S_i \ddot{q}_i + v_i \times (S_i \dot{q}_i)$$

其中 $S_i$ 是关节 $i$ 的运动子空间矩阵，$X_{i,p(i)}$ 是从父连杆 $p(i)$ 空间速度/加速度坐标到子连杆 $i$ 坐标的空间变换，$\times$ 表示空间交叉积。没有这个父子空间变换项时，公式只在所有空间量已经被人为表达在同一坐标系下才成立，不能直接照搬成实现。

**第二遍：从叶到根（反向递推）**——计算每个关节的力矩

$$f_i = I_i a_i + v_i \times^* I_i v_i - f_{\text{ext},i}$$
$$f_{p(i)} \mathrel{+}= X_{i,p(i)}^\top f_i$$
$$\tau_i = S_i^\top f_i$$

其中 $I_i$ 是连杆 $i$ 的空间惯量，$\times^*$ 是力的空间交叉积；$X_{i,p(i)}^\top$ 是与上面运动变换对偶的力变换，把子连杆力映射回父连杆坐标。

```
第一遍 (根→叶):    Joint 1 → Joint 2 → ... → Joint N
  计算: v_i, a_i   ──────────────────────────────→

第二遍 (叶→根):    Joint N → Joint N-1 → ... → Joint 1
  计算: f_i, τ_i   ←──────────────────────────────
```

### 8.3 RNEA 的 Pinocchio API

```cpp
// 标准 RNEA: τ = M(q)a + C(q,v)v + g(q)
Eigen::VectorXd tau = pinocchio::rnea(
    model, data, q, v, a);
// 结果也存储在 data.tau 中

// 等价手动调用:
pinocchio::rnea(model, data, q, v, a);
Eigen::VectorXd tau_result = data.tau;
```

### 8.4 三种特殊用法

| 场景 | 调用 | 公式 |
|------|------|------|
| **重力补偿** | `rnea(model, data, q, 0, 0)` | $\tau_g = g(q)$ |
| **科里奥利力** | `rnea(model, data, q, v, 0) - g(q)` | $\tau_c = C(q,v)v$ |
| **无重力逆动力学** | `rnea(model, data, q, v, a) - g(q)` | $\tau = M(q)a + C(q,v)v$ |

```cpp
// 重力补偿力矩 (实时控制最常用)
Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model.nv);
Eigen::VectorXd zero_a = Eigen::VectorXd::Zero(model.nv);
pinocchio::rnea(model, data, q, zero_v, zero_a);
Eigen::VectorXd gravity_comp = data.tau;

// 也可以直接用 computeGeneralizedGravity
pinocchio::computeGeneralizedGravity(model, data, q);
Eigen::VectorXd g = data.g;  // 等价于上面的 gravity_comp
```

### 8.5 RNEA 性能基准

| 机器人 | DOF | RNEA 耗时 | 含导数 | 平台 |
|--------|:---:|:---------:|:------:|------|
| Franka Panda | 7 | ~1.8 $\mu$s | ~4.5 $\mu$s | i7-10700K, `-O3` |
| UR5 | 6 | ~1.5 $\mu$s | ~3.8 $\mu$s | 同上 |
| Talos (全身) | 32 | ~8 $\mu$s | ~20 $\mu$s | 同上 |
| Atlas (全身) | 30 | ~7.5 $\mu$s | ~18 $\mu$s | 同上 |

关键观察：RNEA 的复杂度严格为 $O(N)$，其中 $N$ 是关节数。这比直接计算 $\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$ 快得多——后者需要先计算 $M(q)$（$O(N^2)$ 存储 + $O(N)$ 计算）再做矩阵乘法。

### 练习

1. ⭐⭐ 对 Franka Panda 在零位处计算重力补偿力矩，验证 `rnea(model, data, q, 0, 0)` 与 `computeGeneralizedGravity(model, data, q)` 结果一致。
2. ⭐⭐⭐ 手动跟踪 3-DOF 平面 RRR 机械臂的 RNEA 两遍递推过程，用具体数值验证。与 Pinocchio 的计算结果比较。
3. ⭐⭐⭐ 用 `std::chrono::high_resolution_clock` 测量 Franka Panda 的 RNEA 耗时（取 10000 次平均），与上表对比。

---

## 9. 正动力学 ABA ⭐⭐

### 9.1 问题定义

**正动力学** (Forward Dynamics)：已知关节角 $q$、角速度 $\dot{q}$、施加力矩 $\tau$，求关节加速度 $\ddot{q}$：

$$\ddot{q} = M(q)^{-1} [\tau - C(q,\dot{q})\dot{q} - g(q)]$$

### 9.2 ABA 三遍递推

**关节空间惯量算法 (Articulated-Body Algorithm, ABA)** 是 Featherstone 的核心贡献，以 $O(N)$ 复杂度求解正动力学：

| 遍次 | 方向 | 计算内容 |
|------|------|---------|
| **第一遍** | 根 → 叶 | 关节速度 $v_i$，偏置力 $p_i$ |
| **第二遍** | 叶 → 根 | 关节空间惯量 $I_i^A$，偏置力向上传递 |
| **第三遍** | 根 → 叶 | 关节加速度 $\ddot{q}_i$ |

与朴素方法的对比：

| 方法 | 步骤 | 复杂度 |
|------|------|--------|
| 朴素法 | CRBA 求 $M$ + Cholesky 分解 + 回代 | $O(N^3)$ |
| **ABA** | 三遍递推 | $O(N)$ |

```cpp
// ABA: 给定 τ, 求 q̈
Eigen::VectorXd a = pinocchio::aba(
    model, data, q, v, tau);
// 结果也在 data.ddq 中
```

### 9.3 何时使用 ABA vs CRBA+Solve

| 场景 | 推荐方法 | 原因 |
|------|---------|------|
| 仿真积分 (每步一次) | **ABA** | $O(N)$，无需显式求 $M$ |
| MPC (需要 $M^{-1}$) | **CRBA + `computeMinverse`** | 可复用 $M^{-1}$ |
| 力控 (需要 $M$ 本身) | **CRBA** | 阻抗控制需要显式 $M(q)$ |
| 小型机械臂 (6~7-DOF) | 差异不大 | $O(N)$ vs $O(N^3)$ 差距小 |
| 高自由度 (30+ DOF) | **ABA** | $O(N)$ 优势明显 |

```cpp
// 仿真循环中典型的正动力学使用
double dt = 0.001;  // 1 kHz
for (int step = 0; step < num_steps; ++step) {
    // 计算加速度
    Eigen::VectorXd ddq = pinocchio::aba(
        model, data, q, v, tau_control);

    // 半隐式欧拉积分
    v += ddq * dt;
    q = pinocchio::integrate(model, q, v * dt);
}
```

### 练习

1. ⭐⭐ 验证 ABA 和 CRBA+Solve 给出相同的 $\ddot{q}$：先用 `aba` 得到 $\ddot{q}_1$，再用 `crba` 得到 $M$，用 LDLT 分解求 $\ddot{q}_2 = M^{-1}(\tau - C\dot{q} - g)$，比较误差。
2. ⭐⭐ 用 ABA 实现一个简单的 Franka Panda 仿真循环：给定初始 $q_0, v_0 = 0, \tau = 0$（自由落体），仿真 1 秒，观察关节角的变化。
3. ⭐⭐⭐ 测量 ABA 和 CRBA+Cholesky 在 7-DOF 和 30-DOF 机器人上的耗时差异，绘制对比图。

---

## 10. 惯量矩阵 CRBA ⭐⭐

### 10.1 CRBA 算法

**复合刚体算法 (Composite Rigid Body Algorithm, CRBA)** 计算关节空间惯量矩阵 $M(q) \in \mathbb{R}^{n_v \times n_v}$：

```cpp
// 计算惯量矩阵
pinocchio::crba(model, data, q);

// 结果存储在 data.M 中 (只填充上三角)
Eigen::MatrixXd M = data.M;
M.triangularView<Eigen::StrictlyLower>() =
    M.transpose().triangularView<Eigen::StrictlyLower>();
// 现在 M 是完整的对称矩阵
```

> ⚠️ **常见陷阱**：`crba()` 只填充 `data.M` 的 **上三角部分**。如果直接使用 `data.M` 而不补全下三角，很多矩阵运算会得到错误结果。要么手动补全（如上），要么使用 `Eigen::SelfAdjointView`：`data.M.selfadjointView<Eigen::Upper>()`。

### 10.2 惯量矩阵的性质

$M(q)$ 具有以下重要性质：

| 性质 | 含义 | 应用 |
|------|------|------|
| **对称** | $M = M^\top$ | 只需存储上三角 |
| **正定** | $x^\top M x > 0, \forall x \neq 0$ | Cholesky 分解总是成功 |
| **有界** | $\lambda_{\min}(M) > 0$，$\lambda_{\max}(M) < \infty$ | 系统始终可控 |
| **带状结构** | 串行链的 $M$ 近似带状 | 稀疏求解可加速 |

```cpp
// 验证正定性: Cholesky 分解
Eigen::LLT<Eigen::MatrixXd> llt(M);
if (llt.info() == Eigen::Success) {
    std::cout << "M(q) 正定性验证通过" << std::endl;
}

// 特征值范围
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(M);
std::cout << "最小特征值: " << eig.eigenvalues().minCoeff()
          << std::endl;
std::cout << "最大特征值: " << eig.eigenvalues().maxCoeff()
          << std::endl;
std::cout << "条件数: "
          << eig.eigenvalues().maxCoeff() /
             eig.eigenvalues().minCoeff()
          << std::endl;
```

### 10.3 computeMinverse：高效求 $M^{-1}$

当需要 $M^{-1}$（如正动力学 $\ddot{q} = M^{-1}(\tau - h)$）时，Pinocchio 提供了比"先 CRBA 再 Cholesky"更高效的方法：

```cpp
// 方法 1: CRBA + Cholesky (标准)
pinocchio::crba(model, data, q);
data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
Eigen::LDLT<Eigen::MatrixXd> ldlt(data.M);
Eigen::VectorXd ddq = ldlt.solve(tau - h);

// 方法 2: computeMinverse (利用运动树结构, 更快)
pinocchio::computeMinverse(model, data, q);
// data.Minv 现在包含 M^{-1} (同样只有上三角)
data.Minv.triangularView<Eigen::StrictlyLower>() =
    data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
Eigen::VectorXd ddq2 = data.Minv * (tau - h);
```

| 方法 | 复杂度 | 适用场景 |
|------|--------|---------|
| CRBA + Cholesky | $O(N^2) + O(N^3/3)$ | 需要 $M$ 本身（力控） |
| `computeMinverse` | $O(N^2)$ | 只需要 $M^{-1}$（MPC） |
| ABA | $O(N)$ | 只需要 $M^{-1} \tau'$（仿真） |

### 10.4 CRBA 耗时参考

| 机器人 | DOF | CRBA 耗时 | $M$ 尺寸 |
|--------|:---:|:---------:|:--------:|
| Franka Panda | 7 | ~2.5 $\mu$s | 7x7 |
| Talos (全身) | 32 | ~15 $\mu$s | 32x32 |
| 人形+手指 | 50 | ~35 $\mu$s | 50x50 |

### 练习

1. ⭐⭐ 计算 Franka Panda 在零位和随机位型下的 $M(q)$，比较特征值范围和条件数。哪个位型的条件数更大？
2. ⭐⭐ 验证 $M(q)$ 的对称性：计算 $\|M - M^\top\|_F$，确认为 0（注意先补全下三角）。
3. ⭐⭐⭐ 比较三种求 $\ddot{q}$ 的方法（ABA / CRBA+Cholesky / computeMinverse+乘法），在 7-DOF 和 30-DOF 上分别测时，验证理论复杂度分析。

---

## 11. 解析导数 ⭐⭐⭐

### 11.1 为什么 MPC 需要动力学导数

MPC 求解器（如 OCS2、Crocoddyl）在每次迭代中需要动力学的一阶导数来构建 QP 子问题。关键导数包括：

| 导数 | 维度 | 用途 |
|------|------|------|
| $\partial \tau / \partial q$ | $n_v \times n_v$ | RNEA 导数，轨迹优化 |
| $\partial \tau / \partial \dot{q}$ | $n_v \times n_v$ | RNEA 导数，阻尼估计 |
| $\partial \tau / \partial \ddot{q}$ | $n_v \times n_v$ | 即 $M(q)$ |
| $\partial \ddot{q} / \partial q$ | $n_v \times n_v$ | ABA 导数，MPC 线性化 |
| $\partial \ddot{q} / \partial \dot{q}$ | $n_v \times n_v$ | ABA 导数，MPC 线性化 |
| $\partial \ddot{q} / \partial \tau$ | $n_v \times n_v$ | 即 $M^{-1}(q)$ |

### 11.2 有限差分 vs 解析导数

**有限差分**（finite difference）是最直观的方法：

$$\frac{\partial \tau}{\partial q_j} \approx \frac{\tau(q + \epsilon e_j) - \tau(q - \epsilon e_j)}{2\epsilon}$$

但代价极高：

| 方法 | 调用次数 | 精度 | 7-DOF 耗时 |
|------|---------|------|-----------|
| 中心差分 | $2 n_v$ 次 RNEA = 14 次 | $O(\epsilon^2)$ | ~25 $\mu$s |
| **解析导数** | ~2 次 RNEA 等价 | 机器精度 | ~4.5 $\mu$s |
| CppAD 自动微分 | 1 次 AD-RNEA | 机器精度 | ~12 $\mu$s |

解析导数比有限差分快 ~5x，且精度高出 8~10 个数量级。这正是 Carpentier & Mansard (2018) 论文 "Analytical Derivatives of Rigid Body Dynamics Algorithms" 的核心贡献。

### 11.3 computeRNEADerivatives

```cpp
// 计算 RNEA 的解析导数
pinocchio::computeRNEADerivatives(model, data, q, v, a);
// 输出:
//   data.dtau_dq:  ∂τ/∂q   (nv x nv)
//   data.dtau_dv:  ∂τ/∂v   (nv x nv)
//   data.M:        ∂τ/∂a = M(q)  (C++ 侧只保证填充上三角)

std::cout << "∂τ/∂q:\n" << data.dtau_dq << std::endl;
std::cout << "∂τ/∂v:\n" << data.dtau_dv << std::endl;
```

使用 `data.M` 前应像 CRBA 后一样显式对称化下三角；否则直接按完整矩阵读取会把未填充部分当成有效数值：

```cpp
data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
```

算法内部在标准 RNEA 两遍递推基础上增加了导数的递推传播，总计算量约为标准 RNEA 的 **2 倍**——远低于有限差分的 $2N$ 倍。

### 11.4 computeABADerivatives

```cpp
// 计算 ABA 的解析导数
pinocchio::computeABADerivatives(model, data, q, v, tau);
// 输出:
//   data.ddq_dq:   ∂q̈/∂q   (nv x nv)
//   data.ddq_dv:   ∂q̈/∂v   (nv x nv)
//   data.Minv:     ∂q̈/∂τ = M⁻¹(q)  (nv x nv)

std::cout << "∂q̈/∂q:\n" << data.ddq_dq << std::endl;
std::cout << "∂q̈/∂v:\n" << data.ddq_dv << std::endl;
std::cout << "M⁻¹(q):\n" << data.Minv << std::endl;
```

### 11.5 导数在 MPC/DDP 中的使用

在 DDP (Differential Dynamic Programming) 的每次 backward pass 中，需要对离散化的动力学 $x_{k+1} = f(x_k, u_k)$ 求导：

$$A_k = \frac{\partial f}{\partial x}\bigg|_{x_k, u_k}, \quad B_k = \frac{\partial f}{\partial u}\bigg|_{x_k, u_k}$$

其中状态 $x = [q^\top, \dot{q}^\top]^\top$，控制 $u = \tau$。利用 Pinocchio 导数：

```cpp
// DDP backward pass 中的线性化
for (int k = N-1; k >= 0; --k) {
    // 计算 ABA 导数
    pinocchio::computeABADerivatives(
        model, data, q_traj[k], v_traj[k], tau_traj[k]);

    // 构建状态转移矩阵 A_k (2nv x 2nv)
    // 使用半隐式欧拉离散化:
    //   v_{k+1} = v_k + dt * q̈(q_k, v_k, τ_k)
    //   q_{k+1} = q_k + dt * v_{k+1}
    Eigen::MatrixXd A(2*nv, 2*nv);
    A.topLeftCorner(nv, nv) =
        Eigen::MatrixXd::Identity(nv, nv)
        + dt * dt * data.ddq_dq;            // ∂q_{k+1}/∂q_k
    A.topRightCorner(nv, nv) =
        dt * Eigen::MatrixXd::Identity(nv, nv)
        + dt * dt * data.ddq_dv;            // ∂q_{k+1}/∂v_k
    A.bottomLeftCorner(nv, nv) =
        dt * data.ddq_dq;                   // ∂v_{k+1}/∂q_k
    A.bottomRightCorner(nv, nv) =
        Eigen::MatrixXd::Identity(nv, nv)
        + dt * data.ddq_dv;                 // ∂v_{k+1}/∂v_k

    // 控制矩阵 B_k (2nv x nv)
    Eigen::MatrixXd B(2*nv, nv);
    B.topRows(nv) = dt * dt * data.Minv;    // ∂q_{k+1}/∂τ_k
    B.bottomRows(nv) = dt * data.Minv;      // ∂v_{k+1}/∂τ_k
}
```

### 11.6 Carpentier & Mansard 2018：论文解读

> 📖 **关键论文**：J. Carpentier, N. Mansard, "Analytical Derivatives of Rigid Body Dynamics Algorithms," RSS 2018.

核心贡献：

| 贡献 | 细节 |
|------|------|
| RNEA 导数 | 在两遍递推基础上增加导数传播，代价 ~2x RNEA |
| ABA 导数 | 在三遍递推基础上增加导数传播，代价 ~2x ABA |
| 数值验证 | 与有限差分对比，精度提升 8~10 个数量级 |
| 开源实现 | 集成在 Pinocchio 中，所有用户零成本使用 |

> ⚠️ **常见陷阱**：`computeRNEADerivatives` 和 `computeABADerivatives` 内部会修改 `data` 中的多个成员（不仅仅是导数矩阵）。在同一个 `data` 对象上交替调用这两个函数时，不要假设之前的计算结果仍然有效——每次调用都应视为"重置"了 `data` 的相关状态。

### 练习

1. ⭐⭐⭐ 验证 `computeRNEADerivatives` 的结果：对每个分量用中心差分近似，比较误差。误差应该在 $10^{-8}$ 量级。
2. ⭐⭐⭐ 测量 `computeRNEADerivatives` 和 $2N$ 次有限差分 RNEA 的耗时比，验证解析导数的速度优势。
3. ⭐⭐⭐ 利用 `computeABADerivatives` 构建一个完整的 DDP backward pass，对 Franka Panda 跟踪一条直线轨迹。

---

## 12. 碰撞检测：Pinocchio + Coal ⭐⭐

### 12.1 Coal（原 hpp-fcl）概述

Coal 是 Pinocchio 配套的碰撞检测库（2024 年从 hpp-fcl 更名为 Coal），提供：

| 功能 | API | 说明 |
|------|-----|------|
| 碰撞检测 | `computeCollisions` | 布尔查询：是否碰撞 |
| 距离计算 | `computeDistances` | 最短距离 + 最近点对 |
| 最近点 | `DistanceResult::nearest_points` | 用于梯度计算 |
| 几何形状 | Box / Sphere / Capsule / Mesh | URDF 碰撞几何 |

### 12.2 GeometryModel 与 GeometryData

碰撞检测延续了 Pinocchio 的 Model/Data 分离模式：

```cpp
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>

// 1. 加载运动学模型
pinocchio::Model model;
pinocchio::urdf::buildModel("panda.urdf", model);

// 2. 加载碰撞几何模型
pinocchio::GeometryModel geom_model;
pinocchio::urdf::buildGeom(
    model, "panda.urdf",
    pinocchio::COLLISION,     // COLLISION 或 VISUAL
    geom_model,
    "/path/to/meshes/"        // mesh 文件搜索路径
);

// 3. 添加碰撞对 (collision pairs)
geom_model.addAllCollisionPairs();  // 所有几何体两两配对
// 移除相邻连杆的碰撞对 (它们总是"碰撞"的)
pinocchio::srdf::removeCollisionPairs(model, geom_model,
    "panda_collision_pairs.srdf");

// 4. 创建 GeometryData
pinocchio::GeometryData geom_data(geom_model);
```

### 12.3 碰撞检测与距离查询

```cpp
// 设置关节角
Eigen::VectorXd q = pinocchio::randomConfiguration(model);

// 更新运动学 (碰撞检测需要关节位姿)
pinocchio::forwardKinematics(model, data, q);

// 碰撞检测: 返回是否存在任何碰撞
bool is_collision = pinocchio::computeCollisions(
    model, data, geom_model, geom_data, q, false);
// 最后一个参数: stopAtFirstCollision
//   true  → 发现第一个碰撞就停止 (快)
//   false → 检查所有碰撞对 (慢但完整)

if (is_collision) {
    std::cout << "检测到碰撞！" << std::endl;
    // 查看哪些碰撞对发生了碰撞
    for (size_t i = 0; i < geom_data.collisionResults.size(); ++i) {
        if (geom_data.collisionResults[i].isCollision()) {
            auto pair = geom_model.collisionPairs[i];
            std::cout << "碰撞: "
                      << geom_model.geometryObjects[pair.first].name
                      << " <-> "
                      << geom_model.geometryObjects[pair.second].name
                      << std::endl;
        }
    }
}

// 距离查询: 计算所有碰撞对的最短距离
pinocchio::computeDistances(
    model, data, geom_model, geom_data, q);

for (size_t i = 0; i < geom_data.distanceResults.size(); ++i) {
    double dist = geom_data.distanceResults[i].min_distance;
    if (dist < 0.05) {  // 距离小于 5cm 时警告
        auto pair = geom_model.collisionPairs[i];
        std::cout << "接近碰撞: "
                  << geom_model.geometryObjects[pair.first].name
                  << " <-> "
                  << geom_model.geometryObjects[pair.second].name
                  << " | 距离 = " << dist << " m" << std::endl;
    }
}
```

### 12.4 自碰撞 + 障碍物避让

```cpp
// 添加环境障碍物 (例如一个桌面)
auto table = coal::Box(1.0, 1.0, 0.02);  // 1m x 1m x 2cm
pinocchio::GeometryObject table_obj(
    "table",                           // 名称
    model.getFrameId("universe"),      // 附着帧
    pinocchio::SE3(                    // 位姿
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.5, 0.0, 0.4)  // 桌面高 40cm
    ),
    std::make_shared<coal::Box>(table) // 几何形状
);
pinocchio::GeomIndex table_id =
    geom_model.addGeometryObject(table_obj);

// 添加桌面与所有机器人几何体的碰撞对
for (size_t i = 0; i < geom_model.ngeoms - 1; ++i) {
    geom_model.addCollisionPair(
        pinocchio::CollisionPair(i, table_id));
}

// 重建 GeometryData
geom_data = pinocchio::GeometryData(geom_model);
```

### 12.5 距离梯度用于优化

在轨迹优化中，碰撞避让通常表示为不等式约束 $d(q) \geq d_{\min}$。距离关于关节角的梯度可以通过链式法则近似：

$$\frac{\partial d}{\partial q} \approx \frac{(p_1 - p_2)^\top}{\|p_1 - p_2\|} \cdot (J_1(q) - J_2(q))$$

其中 $p_1, p_2$ 是最近点对，$J_1, J_2$ 是对应点的位置雅可比。

```cpp
// 获取最近点对
auto& dr = geom_data.distanceResults[pair_idx];
Eigen::Vector3d p1 = dr.nearest_points[0];  // 第一个几何体上的最近点
Eigen::Vector3d p2 = dr.nearest_points[1];  // 第二个几何体上的最近点
double dist = dr.min_distance;

// 距离方向向量
Eigen::Vector3d n = (p1 - p2) / dist;

// 利用雅可比计算距离梯度
// (需要分别计算 p1 和 p2 处的雅可比——此处为简化示意)
Eigen::VectorXd grad_d = J1.topRows(3).transpose() * n
                       - J2.topRows(3).transpose() * n;
```

> ⚠️ **常见陷阱**：Coal 的距离查询在两物体深度穿透时可能返回不准确的结果。在优化中，应设置一个安全余量 `d_safe > 0`，使约束为 $d(q) \geq d_{\text{safe}}$，避免在穿透区域工作。

### 练习

1. ⭐⭐ 加载 Franka Panda 的碰撞几何，计算零位时所有碰撞对的最短距离，找到最接近碰撞的连杆对。
2. ⭐⭐ 添加一个球形障碍物到工作空间中，移动机械臂使其接近障碍物，验证距离查询是否正确。
3. ⭐⭐⭐ 实现一个简单的碰撞避让：在速度级 IK 中添加零空间投影，使末端到达目标位姿的同时保持所有距离 $> 3$ cm。

---

## 13. 约束动力学 v3.x ⭐⭐⭐

### 13.1 约束动力学的应用场景

Pinocchio 3.x 引入了约束动力学支持，解决以下场景：

| 场景 | 约束类型 | 说明 |
|------|---------|------|
| **闭链机构** | 环路闭合约束 | 如四连杆、Stewart 平台 |
| **接触力计算** | 接触约束 | 足底接触、抓取 |
| **铰接约束** | 关节约束 | 非标准运动副 |
| **腱驱动** | 耦合约束 | 关节间的腱绳耦合 |

### 13.2 约束动力学公式

带约束的动力学方程为 KKT 系统：

$$\begin{bmatrix} M(q) & J_c^\top \\ J_c & 0 \end{bmatrix} \begin{bmatrix} \ddot{q} \\ -\lambda \end{bmatrix} = \begin{bmatrix} \tau - h(q, \dot{q}) \\ -\gamma \end{bmatrix}$$

其中 $J_c$ 是约束雅可比，$\lambda$ 是约束力（拉格朗日乘子），$\gamma$ 是约束加速度偏移。

### 13.3 Pinocchio 3.x 约束 API

```cpp
#include <pinocchio/algorithm/constrained-dynamics.hpp>
#include <pinocchio/algorithm/contact-info.hpp>

// 定义接触约束
pinocchio::RigidConstraintModel contact_model(
    pinocchio::CONTACT_3D,              // 约束类型: 3D 点接触
    model,
    model.getJointId("panda_joint7"),   // 接触连杆
    pinocchio::SE3::Identity(),         // 接触点相对关节的位姿
    model.getJointId("universe"),       // 环境 (固定)
    pinocchio::SE3::Identity(),         // 环境接触点
    pinocchio::LOCAL_WORLD_ALIGNED      // 参考坐标系
);

// 约束列表
std::vector<pinocchio::RigidConstraintModel> contact_models;
contact_models.push_back(contact_model);

// 约束数据
std::vector<pinocchio::RigidConstraintData> contact_datas;
for (auto& cm : contact_models)
    contact_datas.push_back(pinocchio::RigidConstraintData(cm));

// 初始化约束动力学
pinocchio::initConstraintDynamics(
    model, data, contact_models);

// 求解约束动力学
pinocchio::constraintDynamics(
    model, data, q, v, tau,
    contact_models, contact_datas);

// 结果
Eigen::VectorXd ddq = data.ddq;        // 约束加速度
Eigen::VectorXd lambda = data.lambda_c; // 约束力/拉格朗日乘子
```

### 13.4 Delassus 算子与近端约束动力学

Pinocchio 3.x 内部使用 **Delassus 算子** $G = J_c M^{-1} J_c^\top$ 来高效求解约束力：

| 求解方法 | 适用约束类型 | 库 |
|---------|------------|-----|
| 直接 KKT | 等式约束 | Eigen LDLT |
| 近端约束动力学 | 刚性接触/闭链等式约束的正则化求解 | Pinocchio 内置 |
| QP 接触分配 | 摩擦锥、不等式和任务权重 | M05 层用 ProxQP/OSQP 建模 |

```cpp
// Pinocchio 内置近端约束动力学设置；这不是 proxsuite::proxqp。
#include <pinocchio/algorithm/proximal.hpp>

pinocchio::ProximalSettings prox_settings;
prox_settings.mu = 1e-8;         // 近端正则化
prox_settings.max_iter = 50;     // 最大迭代
prox_settings.accuracy = 1e-6;   // 收敛精度

pinocchio::constraintDynamics(
    model, data, q, v, tau,
    contact_models, contact_datas,
    prox_settings);

Eigen::VectorXd ddq = data.ddq;
Eigen::VectorXd lambda = data.lambda_c;
```

`ProximalSettings` 只控制 Pinocchio 约束动力学内部的近端正则化和停止条件，不表示 Pinocchio 把约束动力学委托给 ProxSuite/ProxQP。若问题包含摩擦锥线性化、力分配权重或关节/接触不等式，应在控制层显式组装 QP，再用 M05 中的 ProxQP、OSQP 或其它求解器求解。

### 13.5 闭链机构示例

```cpp
// 四连杆闭链: 需要一个环路闭合约束
// 假设 joint A 和 joint B 通过一个被动连杆相连

pinocchio::RigidConstraintModel loop_closure(
    pinocchio::CONTACT_6D,               // 6D: 完全锁定
    model,
    model.getJointId("joint_A"),         // 主动端
    placement_A,                          // 主动端接触点
    model.getJointId("joint_B"),         // 被动端
    placement_B,                          // 被动端接触点
    pinocchio::LOCAL                      // 参考坐标系
);
```

> ⚠️ **常见陷阱**：约束动力学的数值稳定性对正则化参数 `mu` 很敏感。太大会导致约束不精确（穿透），太小会导致 KKT 系统病态。建议从 `mu = 1e-8` 开始，根据约束违反量调整。

### 练习

1. ⭐⭐ 对 Franka Panda 末端施加一个 3D 点接触约束（模拟末端按在桌面上），求解约束加速度和接触力。
2. ⭐⭐⭐ 构建一个简单的四连杆闭链模型，使用环路闭合约束求解其动力学。验证约束违反量随时间不发散。
3. ⭐⭐⭐ 比较有约束和无约束的正动力学计算耗时，分析约束数量对性能的影响。

---

## 14. Python Bindings ⭐

### 14.1 eigenpy：零拷贝 NumPy 桥梁

Pinocchio 的 Python 绑定基于 eigenpy，实现 Eigen 矩阵与 NumPy 数组之间的 **零拷贝** 共享：

| 特性 | 说明 |
|------|------|
| 零拷贝 | `data.M` 返回的 NumPy 数组直接指向 C++ 内存 |
| 自动类型映射 | `Eigen::VectorXd` $\leftrightarrow$ `np.ndarray(dtype=float64)` |
| SE(3) 包装 | `pinocchio.SE3` 完整暴露 Python 端 |
| NumPy 2 兼容 | Pinocchio 3.x 已支持 NumPy 2.x |

> ⚠️ **常见陷阱**：零拷贝意味着 Python 端修改 `data.M` 会 **直接影响** C++ 端的数据。这在大多数时候是高效的，但如果你需要保存某次计算的结果，必须显式拷贝：`M_copy = data.M.copy()`。

### 14.2 完整 Python FK + Jacobian + IK 示例

```python
import numpy as np
import pinocchio as pin

# ========== 1. 加载模型 ==========
model = pin.buildModelFromUrdf(
    "/path/to/franka_description/robots/panda.urdf"
)
data = model.createData()

print(f"关节数: {model.njoints}")   # 8 (含 universe)
print(f"nq = {model.nq}")           # 7
print(f"nv = {model.nv}")           # 7
print(f"帧数: {model.nframes}")     # ~26

# ========== 2. 正向运动学 ==========
q = pin.randomConfiguration(model)  # 随机合法关节角
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

ee_id = model.getFrameId("panda_hand")
ee_pose = data.oMf[ee_id]  # SE3 对象
print(f"末端位置: {ee_pose.translation}")
print(f"末端旋转:\n{ee_pose.rotation}")

# ========== 3. 雅可比矩阵 ==========
J = pin.computeFrameJacobian(
    model, data, q, ee_id,
    pin.LOCAL_WORLD_ALIGNED       # 最常用的参考坐标系
)
print(f"雅可比形状: {J.shape}")    # (6, 7)

# 奇异值分析
U, S, Vt = np.linalg.svd(J)
print(f"奇异值: {S}")
print(f"可操作度: {np.prod(S):.6f}")

# ========== 4. RNEA: 重力补偿 ==========
v = np.zeros(model.nv)
a = np.zeros(model.nv)
tau_gravity = pin.rnea(model, data, q, v, a)
print(f"重力补偿力矩: {tau_gravity}")

# ========== 5. 惯量矩阵 ==========
pin.crba(model, data, q)
M = data.M.copy()  # 拷贝! 否则后续计算会覆盖
# 补全下三角
M = np.triu(M) + np.triu(M, 1).T
print(f"M(q) 条件数: {np.linalg.cond(M):.1f}")

# ========== 6. 速度级 IK ==========
T_desired = pin.SE3(
    pin.Quaternion(0.0, 1.0, 0.0, 0.0).normalized().matrix(),
    np.array([0.4, 0.0, 0.5])   # 目标位置
)

q_ik = pin.neutral(model)  # 从中性位开始
dt = 0.1
eps = 1e-4

for i in range(200):
    pin.forwardKinematics(model, data, q_ik)
    pin.updateFramePlacements(model, data)

    T_current = data.oMf[ee_id]
    iMd = T_current.actInv(T_desired)
    err = pin.log6(iMd).vector

    if np.linalg.norm(err) < eps:
        print(f"IK 收敛于第 {i} 步")
        break

    J_local = pin.computeFrameJacobian(
        model, data, q_ik, ee_id,
        pin.LOCAL
    )
    J_task = -pin.Jlog6(iMd.inverse()) @ J_local

    # 阻尼伪逆
    lam = 1e-6
    v = -J_task.T @ np.linalg.solve(
        J_task @ J_task.T + lam * lam * np.eye(6),
        err
    )
    q_ik = pin.integrate(model, q_ik, v * dt)

print(f"IK 结果: q = {q_ik}")

# ========== 7. 解析导数 ==========
q_test = pin.randomConfiguration(model)
v_test = np.random.randn(model.nv) * 0.1
a_test = np.random.randn(model.nv) * 0.1

pin.computeRNEADerivatives(model, data, q_test, v_test, a_test)
print(f"∂τ/∂q 形状: {data.dtau_dq.shape}")  # (7, 7)
print(f"∂τ/∂v 形状: {data.dtau_dv.shape}")  # (7, 7)
```

### 14.3 meshcat 可视化

```python
from pinocchio.visualize import MeshcatVisualizer

# 加载几何模型
urdf_path = "/path/to/panda.urdf"
mesh_dir = "/path/to/meshes/"
collision_model = pin.buildGeomFromUrdf(
    model, urdf_path,
    pin.COLLISION, mesh_dir
)
visual_model = pin.buildGeomFromUrdf(
    model, urdf_path,
    pin.VISUAL, mesh_dir
)

# 创建可视化器
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)  # 浏览器打开
viz.loadViewerModel()

# 显示当前位型
viz.display(q)

# 动画: 逐帧显示 IK 求解过程
import time
for q_step in q_trajectory:
    viz.display(q_step)
    time.sleep(0.05)
```

### 练习

1. ⭐ 用 Python 加载 Franka Panda，在 meshcat 中可视化零位和随机位型。
2. ⭐⭐ 将本章的速度级 IK 用 Python 实现，并在 meshcat 中实时可视化求解过程。
3. ⭐⭐ 比较 Python 版和 C++ 版 RNEA 的耗时。由于 eigenpy 的零拷贝，Python 版的额外开销来自哪里？

---

## 章节总结

### API 速查表

| 函数 | 输入 | 输出 | 复杂度 | data 成员 |
|------|------|------|--------|-----------|
| `forwardKinematics(m,d,q)` | $q$ | 关节位姿 | $O(N)$ | `data.oMi` |
| `forwardKinematics(m,d,q,v)` | $q,v$ | + 速度 | $O(N)$ | `data.v` |
| `forwardKinematics(m,d,q,v,a)` | $q,v,a$ | + 加速度 | $O(N)$ | `data.a` |
| `updateFramePlacements(m,d)` | — | 帧位姿 | $O(F)$ | `data.oMf` |
| `computeJointJacobians(m,d,q)` | $q$ | 所有关节雅可比 | $O(N)$ | 内部缓存 |
| `getJointJacobian(m,d,id,rf,J)` | 关节id, 参考系 | 单关节雅可比 | $O(N)$ | 输出到 $J$ |
| `computeFrameJacobian(m,d,q,id,rf,J)` | $q$, 帧id, 参考系 | 帧雅可比 | $O(N)$ | 输出到 $J$ |
| `rnea(m,d,q,v,a)` | $q,v,a$ | $\tau$ | $O(N)$ | `data.tau` |
| `computeGeneralizedGravity(m,d,q)` | $q$ | $g(q)$ | $O(N)$ | `data.g` |
| `aba(m,d,q,v,tau)` | $q,v,\tau$ | $\ddot{q}$ | $O(N)$ | `data.ddq` |
| `crba(m,d,q)` | $q$ | $M(q)$ (上三角) | $O(N^2)$ | `data.M` |
| `computeMinverse(m,d,q)` | $q$ | $M^{-1}(q)$ | $O(N^2)$ | `data.Minv` |
| `computeRNEADerivatives(m,d,q,v,a)` | $q,v,a$ | $\partial\tau/\partial q,v,a$ | $O(N)$ | `data.dtau_dq/dv`; 另外 `data.M`（= $\partial\tau/\partial a$，上三角填充）|
| `computeABADerivatives(m,d,q,v,tau)` | $q,v,\tau$ | $\partial\ddot{q}/\partial q,v,\tau$ | $O(N)$ | `data.ddq_dq/dv, Minv` |
| `computeCollisions(m,d,gm,gd,q)` | $q$ | 碰撞布尔 | 取决于几何 | `gd.collisionResults` |
| `computeDistances(m,d,gm,gd,q)` | $q$ | 距离值 | 取决于几何 | `gd.distanceResults` |

### 设计模式速查

| 模式 | Pinocchio 体现 | 关键优势 |
|------|---------------|---------|
| **Model/Data 分离** | `Model` (const) + `Data` (mutable) | 线程安全、可序列化 |
| **CRTP 静态多态** | `JointModelBase<Derived>` | 零开销分派、内联友好 |
| **Visitor 模式** | `JointModelVariant` + `boost::apply_visitor` | 运行时类型安全分派 |
| **标量参数化** | `ModelTpl<Scalar>` | 同一代码支持 AD/CasADi |
| **自由函数** | `pinocchio::rnea(model, data, ...)` | 正交、可扩展 |

### 1.5 周学习规划

| 天数 | 内容 | 对应章节 | 产出 |
|:----:|------|---------|------|
| Day 1 | 设计哲学、CRTP、Model/Data | 1-3 | 能解释三大支柱 |
| Day 2 | 标量参数化、URDF 加载 | 4-5 | 加载 Panda 并打印模型信息 |
| Day 3 | FK + 雅可比（理论） | 6-7 前半 | 推导三种坐标系转换 |
| Day 4 | 雅可比（代码）+ IK | 7 后半 | 速度级 IK 实现 |
| Day 5 | RNEA + ABA + CRBA | 8-10 | 三大算法 benchmark |
| Day 6 | 解析导数 | 11 | 验证导数精度 |
| Day 7 | 碰撞检测 | 12 | Coal 碰撞管线搭建 |
| Day 8 | 约束动力学 | 13 | 闭链约束示例 |
| Day 9 | Python + 可视化 | 14 | meshcat 完整演示 |
| Day 10 | 综合练习 + 复习 | 全章 | Panda 完整 pipeline |

### 下游章节衔接

| 下游章节 | 依赖本章内容 | 关键桥接 |
|---------|------------|---------|
| **M02 动力学库对比** | 1.4 四大库对比 | 在同一 URDF 上横向测试 |
| **M03 IK 求解器深度** | 7 雅可比 + 速度级 IK | 从速度级 IK 扩展到位置级 |
| **M06 自动微分** | 4 标量参数化 + 11 解析导数 | CppAD/CasADi 深度集成 |
| **M14 MoveIt2 集成** | 5 URDF 加载 + 12 碰撞检测 | MoveIt2 可通过 pick-ik 等插件调用 Pinocchio |

---

> **本章核心收获**：Pinocchio 不仅是一个动力学库，它的 Model/Data 分离 + CRTP + 标量参数化三大设计决策值得每个 C++ 机器人工程师学习。理解这些设计模式，比记住 API 调用更重要——因为你会在 Crocoddyl、OCS2、TSID 等下游框架中反复遇到相同的设计哲学。

---

## 累积项目：本章新增模块

本章是累积项目 **mini-manip** 的第二个模块（承接 P01 的 URDF 建模）。

### 本章新增

```
mini-manip/
├── urdf/                          ← P01 已完成
├── src/
│   └── load_panda.cpp             ← M01 新增：加载 URDF + FK/ID/CRBA 完整 pipeline
├── include/
│   └── pinocchio_utils.hpp        ← M01 新增：Pinocchio 常用操作封装
├── python/
│   └── panda_pipeline.py          ← M01 新增：Python 版完整 pipeline + meshcat
└── CMakeLists.txt                 ← M01 更新：添加 Pinocchio + Coal 依赖
```

### 本章具体任务

1. 用 C++ 加载 P01 章创建的 URDF，打印模型信息（nq, nv, 关节名称、帧列表）
2. 实现 FK → 雅可比 → RNEA → ABA → CRBA 的完整调用链
3. 实现速度级 IK（阻尼伪逆），在 meshcat 中可视化
4. 测量各算法耗时，输出 benchmark 数据

### 与后续章节的连接

| 后续章节 | 新增模块 |
|---------|---------|
| M02 动力学库对比 | 添加 RBDL 实现，构建多库对比实验台 |
| M03 IK 求解器 | 扩展速度级 IK 为位置级，集成多种求解器 |
| M04 碰撞检测 | 添加 Coal 碰撞管线，实现碰撞检查接口 |

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|:---:|------|
| Carpentier et al. (2019) "The Pinocchio C++ Library" | ⭐⭐ | Pinocchio 架构论文，INRIA 官方 |
| Featherstone (2008) "Rigid Body Dynamics Algorithms" | ⭐⭐⭐ | RNEA/ABA/CRBA 的算法理论基础 |
| Pinocchio 3.x 官方文档 + API reference | ⭐⭐ | `gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/` |
| Pinocchio GitHub 仓库 `examples/` 目录 | ⭐ | 官方示例代码，覆盖 FK/IK/ID/碰撞 |
| Carpentier & Mansard (2018) "Analytical Derivatives of Rigid Body Dynamics Algorithms" | ⭐⭐⭐ | RNEA/ABA 解析导数的数学推导 |
| Coal (hpp-fcl) 碰撞检测文档 | ⭐⭐ | Pinocchio 配套碰撞库 |
| eigenpy 文档 | ⭐ | Python-C++ 零拷贝绑定机制 |
| Pinocchio 3.x 迁移指南 | ⭐⭐ | 2.x → 3.x API 变更清单 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| `buildModel` 报错 "invalid joint type" | URDF 中使用了 Pinocchio 不支持的关节类型 | 1. 检查 URDF 中是否有 `planar`/`floating` 关节 2. Pinocchio 需要显式指定 `JointModelFreeFlyer` 3. 将 `floating` 关节替换为 `buildModel` 的根关节参数 | §5 URDF 加载 |
| CRBA 返回的 $M(q)$ 只有上三角有值 | Pinocchio CRBA 设计上只填充上三角 | 1. 用 `data.M.triangularView<StrictlyLower>() = data.M.transpose()...` 补全下三角 2. 或直接用 `selfadjointView<Upper>()` 做乘法 | §10 CRBA |
| 多线程下 RNEA 结果不一致或出现 NaN | 多个线程共享同一个 `Data` 对象 | 1. 确认每个线程有独立的 `Data` 2. `Model` 可共享（const） 3. 用 ThreadSanitizer 检测 data race | §2.3 线程安全 |
| FK 结果中末端位姿与预期不符 | 忘记调用 `updateFramePlacements` 或使用了错误的帧 ID | 1. 确认调用了 `forwardKinematics` + `updateFramePlacements` 2. 区分 `data.oMi`（关节帧）和 `data.oMf`（操作帧） 3. 用 `model.getFrameId("frame_name")` 获取正确 ID | §6 FK 深入 |
| CppAD 版本 Pinocchio 比 double 版慢 100 倍 | AD tape 录制在循环中重复执行 | 1. 将 `CppAD::Independent()` 移到循环外 2. 录制一次 tape，多次 Forward/Reverse 3. 考虑用 CppADCodeGen 生成原生代码 | §4 标量参数化 |
| Pinocchio 3.x 编译失败 "Coal not found" | 缺少 Coal（原 hpp-fcl）碰撞检测依赖 | 1. `conda install coal` 或从源码编译 2. 设置 `CMAKE_PREFIX_PATH` 指向安装路径 3. 确认 CMake 中 `find_package(coal)` 通过 | §12 碰撞检测 |
