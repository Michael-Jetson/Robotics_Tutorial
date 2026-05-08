> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第47章：Pinocchio 深度精读——CRTP + Model-Data + 模板 Scalar

> **难度**：⭐⭐⭐ | **建议用时**：1.5 周 | **前置要求**：Ch22 Eigen 深入（表达式模板/对齐/SIMD）、Ch23 李群与 manif 库、Ch29 CRTP/设计模式与高级惯用法

---

## 前置自测

> 📋 答不出 >= 2 题 → 先回顾对应前置章节

1. **CRTP 基础**：写出 CRTP 的基本结构——基类模板 `Base<Derived>` 如何通过 `static_cast<Derived*>(this)` 在编译期派发到派生类？与虚函数相比，CRTP 在 vtable 查找和内联优化上有什么具体差异？（答不出 → 回顾 Ch29 CRTP 部分）
2. **Eigen 模板参数**：`Eigen::Matrix<Scalar, Rows, Cols>` 中的 `Scalar` 模板参数意味着什么？如果将 `Scalar` 从 `double` 替换为 `Ceres::Jet<double, N>`，矩阵的加法和乘法运算是否仍然正确？为什么？（答不出 → 回顾 Ch22 表达式模板 + Ch24 Ceres 自动微分）
3. **SE(3) 运算**：给定两个齐次变换矩阵 $T_1, T_2 \in SE(3)$，$T_1 \cdot T_2$ 表示什么物理含义？manif 中如何用 `compose()` 完成这个运算？逆变换 $T^{-1}$ 的闭式表达式为什么不是 $T^T$？（答不出 → 回顾 Ch23 群作用与李群运算）
4. **线程安全**：`const` 引用为什么能避免数据竞争？如果两个线程同时读一个 `const` 对象、同时各自写一个独立对象，需要 mutex 吗？为什么？（答不出 → 回顾 Ch20 实时约束与并发）
5. **动力学方程**：写出拉格朗日形式的多刚体动力学方程 $M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau$，解释 $M(q)$ 的物理含义和数学性质（为什么是正定对称矩阵？）。（答不出 → 补充经典力学 / 机器人学导论基础）

---

## 本章目标

学完本章，你将能够：

1. **定位** Pinocchio 在 INRIA 学派机器人 C++ 生态中的"基础设施"角色——理解为什么 Crocoddyl / TSID / OCS2 / Aligator / Pink 全部以它为动力学后端，以及这与 Sophus / manif 处理"孤立 SE(3)"的本质区别
2. **精读** Pinocchio 的 CRTP 关节类型系统——从 `JointModelBase<Derived>` 出发，理解 12+ 种关节类型如何在编译期完成 `calc()` 派发，与 Ch29 讲过的 Sophus `SO3Base` 一个类型家族的 CRTP 进行规模对比
3. **掌握** Model / Data 分离范式——理解为什么这个设计让 Pinocchio 天然多线程安全且零运行时 malloc，与 MuJoCo 的 `mjModel` / `mjData` 做精确字段映射
4. **掌握** 模板化 `Scalar` 类型——理解同一份 `rnea.hxx` 如何服务 `double` / `CppAD::AD<double>` / `CppAD::cg::CG<double>` / 多精度浮点四种用途（本章上半部预览，47.4 详细展开）
5. **能独立** 加载 URDF，调用正运动学（FK）和 Jacobian 计算——为后续 Ch48 CppAD 微分和 Ch53 TSID WBC 打好 API 基础

**本章在课程中的位置**：本章是足式方向（Ch47）、机械臂方向（M01）、复合方向（Ch71 引用）的**共享基础设施章节**。前面的 Ch22 Eigen 让我们掌握了矩阵运算引擎，Ch23 manif 让我们掌握了单个 SE(3) 的李群运算，Ch29 让我们理解了 CRTP 的编译期多态原理。但这些都是"组件级"的知识。Pinocchio 把这些组件组装成了一个**完整的刚体动力学引擎**——它处理的不是一个变换矩阵，而是整棵关节树上 N 个串联变换的联合动力学。理解 Pinocchio 的设计，是理解整个 INRIA 学派规控生态（Crocoddyl、TSID、OCS2、Aligator）的前提。

---

## 47.1 Pinocchio 在机器人 C++ 生态中的位置

### 动机：从"一个 SE(3)"到"N 个串联 SE(3)"

前面的章节里，我们用 Sophus 和 manif 做了大量的 SE(3) 运算——一个旋转、一个位姿、相邻两帧的 delta。这些库解决的是"**孤立的刚体变换**"问题：给定两个坐标系之间的变换关系，如何组合、求逆、计算切向量、传播协方差。

但当你面对一个真实的机器人时，问题的规模发生了质变。考虑一个 12-DOF 的 Unitree Go2 四足机器人：

```
基座 (6-DOF 浮动) → 左前髋(Revolute) → 左前膝(Revolute) → 左前踝(Revolute) → 足端
                  → 右前髋(Revolute) → 右前膝(Revolute) → 右前踝(Revolute) → 足端
                  → 左后髋(Revolute) → 左后膝(Revolute) → 左后踝(Revolute) → 足端
                  → 右后髋(Revolute) → 右后膝(Revolute) → 右后踝(Revolute) → 足端
```

这是一棵**关节树**。每个关节有自己的关节变量 $q_i$、速度 $\dot{q}_i$、加速度 $\ddot{q}_i$，每个关节处有一个局部的 SE(3) 变换 $T_i(q_i)$。从基座到某条腿的足端，需要将这条链上所有局部变换**串联起来**：

$$T_{\text{foot}} = T_{\text{base}} \cdot T_{\text{hip}}(q_1) \cdot T_{\text{knee}}(q_2) \cdot T_{\text{ankle}}(q_3)$$

更重要的是，我们不仅需要位姿，还需要这条链的**速度传播**（用于雅可比计算）、**力传播**（用于逆动力学）、**惯量聚合**（用于惯量矩阵），以及这些量对 $q, \dot{q}, \ddot{q}$ 的**导数**（用于 MPC 优化）。

Sophus 和 manif 完全无法处理这个问题——它们不知道"关节"是什么、"关节树"是什么、"递归算法"是什么。它们只是 SE(3) 的计算器，不是动力学引擎。

**Pinocchio 就是这个引擎。** Sophus 之于 Pinocchio，就像单个音符之于整首交响曲——Sophus 处理一个孤立的 SE(3) 变换，Pinocchio 则指挥整棵关节树上数十个 SE(3) 变换的协调运算，包括速度传播、力传递和惯量聚合。

### 核心算法：Featherstone 递归族

Pinocchio 实现了 Roy Featherstone《Rigid Body Dynamics Algorithms》（2008）中的经典递归算法。这些算法的共同特征是**利用关节树的拓扑结构，通过一次或两次遍历完成全局动力学量的计算**：

| 算法 | 全称 | 功能 | 输入 → 输出 | 复杂度 | Go2 典型耗时 |
|------|------|------|-------------|--------|-------------|
| **RNEA** | Recursive Newton-Euler | 逆动力学 | $(q, v, a) \to \tau$ | $O(N)$ | ~0.8 $\mu$s |
| **ABA** | Articulated-Body | 正动力学 | $(q, v, \tau) \to a$ | $O(N)$ | ~1.2 $\mu$s |
| **CRBA** | Composite Rigid Body | 惯量矩阵 | $q \to M(q)$ | $O(N^2)$ | ~0.6 $\mu$s |
| **Cholesky** | — | 线性求解 | $M, b \to M^{-1}b$ | $O(N^3)$ | 可忽略（$N$ 小） |

这里 $N$ 是关节数（不是机器人自由度 $n_v$，但对串联链两者通常相同量级）。RNEA 的 $O(N)$ 复杂度意味着即使关节数翻倍，计算时间也只是线性增长——这是递归利用树结构的直接结果，后续 47.6 节会从递推公式层面详细拆解。RNEA 的两遍递归好比**从树叶到树根的力传递**：第一遍（前向）从根到叶传播速度和加速度，就像从树干到树梢逐级传导晃动；第二遍（后向）从叶到根汇聚力和力矩，就像风吹树叶产生的力经过每根树枝逐级累加，最终汇聚到树干——每个关节只需处理自己那一段，所以总计算量与关节数成线性关系。

### 杀手级特性：解析导数

Pinocchio 真正区别于其他动力学库（如 RBDL、Drake 的 MultibodyPlant）的杀手级特性是：**它提供了上述算法的解析导数（analytical derivatives）**。

这里要区分三种求导方式：

**数值差分（Numerical Differentiation）**：对每个变量做有限差分 $\partial f / \partial q_i \approx (f(q + \epsilon e_i) - f(q)) / \epsilon$。需要 $n_v$ 次额外的函数求值。对 12-DOF 系统，RNEA 的完整雅可比需要调用 RNEA 13 次。精度受 $\epsilon$ 选取影响，太大有截断误差，太小有舍入误差。

**自动微分（Automatic Differentiation, AD）**：通过 operator overloading（如 CppAD 的 `AD<double>` 类型）在前向计算的同时记录计算图，然后反向传播得到梯度。精度等于机器精度，但运行时有 tape 记录和回放的开销，通常比原始计算慢 3-10 倍。

**解析导数（Analytical Derivatives）**：根据 RNEA / ABA 的**数学递推结构**，手工推导出导数的闭式递推公式，然后直接实现为代码。这些公式发表在 Carpentier & Mansard 2018（RNEA 导数）和 Singh et al. 2022（ABA 导数）等论文中，实现在 Pinocchio 的 `rnea-derivatives.hpp` / `aba-derivatives.hpp` 等文件里。

**性能数据**（7-DOF 机械臂，Carpentier et al. 2019 基准测试）：

| 求导方法 | RNEA 导数耗时 | 相对解析导数 |
|---------|-------------|------------|
| 数值差分 | ~15 $\mu$s | 3x 慢 |
| CppAD AD | ~20 $\mu$s | 4x 慢 |
| **Pinocchio 解析导数** | **~5 $\mu$s** | **基准** |

当 MPC 优化器每次迭代需要调用数百次动力学导数时，3-4 倍的性能差距意味着 MPC 实时性的成败。这就是为什么 Crocoddyl / Aligator 能做到微秒级的 backward pass——底层原因是 Pinocchio 的解析导数。

### 生态图：一个库撑起一个学派

Pinocchio 不是一个孤立的库。它是 INRIA（法国国家信息与自动化研究所）Gepetto 团队的动力学内核，整个 INRIA 学派的规控生态都建立在它之上：

```
                     Pinocchio (动力学内核)
                           │
         ┌─────────────────┼─────────────────────┐
         │                 │                     │
      Crocoddyl          TSID                  OCS2
      (DDP 轨迹优化)     (WBC 逆动力学)         (腿足 MPC)
         │                 │                     │
         └─────────────────┼─────────────────────┘
                           │
                      Aligator (下一代,
                      ProxDDP 约束优化)

   Pink       ← 逆运动学（Python层，底层调用 Pinocchio FK/Jacobian）
   HPP-FCL / Coal  ← 可微碰撞检测（与 Pinocchio 深度集成）
   Stack-of-Tasks  ← 分层控制器家族
   ProxSuite       ← QP 求解器（OCS2/Aligator 的内层求解）
```

**Pinocchio 3.x 最新进展**（截至 2025 年）：
- **椭球体关节（ellipsoid joints）**：支持更灵活的关节约束建模
- **mimic 关节 URDF 解析**：正确处理 URDF 中的 `<mimic>` 标签（联动关节）
- **Coal 集成**：HPP-FCL 的下一代碰撞检测库，支持可微碰撞距离计算
- **CasADi 后端**：除 CppAD 外，新增对 CasADi 符号框架的原生支持
- **改进的 Python 绑定**：基于 eigenpy 3.x，NumPy 2.0 兼容

💡 **一句话定位**："Pinocchio 对机器人动力学计算的意义，等同于 Eigen 对数值线性代数的意义——它是基础设施层，不是应用层。你不直接用它写控制器，但每一个控制器都依赖它。"

> **本质洞察**：Pinocchio 的价值不是"又一个动力学库"，而是它把**递归算法的数学优雅性**和**现代 C++ 的工程安全性**统一在了同一套代码中。RNEA 的 $O(N)$ 复杂度来自对树结构的递归利用，模板 Scalar 的泛型设计来自 C++ 的零开销抽象——两者缺一不可。理解 Pinocchio，本质上是理解"数学结构如何驱动软件架构"。

---

## 47.2 CRTP 关节类型系统

### 动机：1kHz 控制循环中 vtable 开销不可接受

在 Ch29 中我们讨论过 CRTP 和虚函数的取舍——架构层用虚函数，热路径用 CRTP。现在让我们用具体数字来看 Pinocchio 为什么必须选择 CRTP。

一个腿足 MPC 控制器的典型执行路径：

```
1kHz 主循环 → MPC 求解（每次迭代调用 RNEA 200+ 次）
                              ↓
           每次 RNEA → 遍历 N 个关节 → 对每个关节调用 calc() / motion() / force()
```

以 Go2 四足为例：$N = 13$（1 个 FreeFlyer 基座 + 12 个 Revolute 关节），每次 RNEA 调用 `calc()` 13 次。MPC 一次求解调用 RNEA 200 次 = `calc()` 被调用 **2600 次**。1kHz 意味着每秒执行 1000 次 MPC → `calc()` 每秒被调用 **260 万次**。

虚函数每次调用的额外开销是 2-5 ns（vtable 间接寻址 + 阻止内联 + 阻止 SIMD 向量化）。260 万次 $\times$ 3 ns = **约 8 ms / 秒**的纯多态开销。这在 1ms 的控制周期预算中占比不小，而且更严重的是，vtable 跳转破坏了指令流水线，阻止编译器将 `calc()` 的函数体内联到 RNEA 的循环中，进而阻止了循环级别的 SIMD 优化。

CRTP 消灭了这些开销——`calc()` 的派发在编译期完成，函数体被内联，编译器可以将整个关节运算循环向量化。

### CRTP 层级结构：JointModelBase

Pinocchio 的关节类型系统的 CRTP 基类定义在 `include/pinocchio/multibody/joint/joint-base.hpp` 中（约 300 行）。核心结构如下：

```cpp
// 基类——所有关节模型的公共接口
template <typename Derived>
struct JointModelBase
{
    // CRTP 核心：把 this 转型为派生类引用
    Derived& derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const
    { return *static_cast<const Derived*>(this); }

    // 所有算法通过 derived() 派发到具体关节类型
    template <typename ConfigVector>
    void calc(JointDataBase<...>& data,
              const Eigen::MatrixBase<ConfigVector>& q) const
    {
        derived().calc_impl(data, q);  // 编译期决定调用谁
    }

    // 关节的运动子空间维度（编译期常量或运行期值）
    int nq() const { return derived().nq_impl(); }
    int nv() const { return derived().nv_impl(); }
};
```

**派生类列表**——Pinocchio 3.x 支持 12+ 种关节类型，每一种都是 CRTP 派生：

```cpp
template <typename Scalar, int Options = 0>
struct JointModelRevoluteTpl           // 旋转关节（绕固定轴，nq=1, nv=1）
    : JointModelBase<JointModelRevoluteTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelRevoluteUnalignedTpl  // 任意轴旋转关节
    : JointModelBase<JointModelRevoluteUnalignedTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelPrismaticTpl          // 平动关节（nq=1, nv=1）
    : JointModelBase<JointModelPrismaticTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelSphericalTpl          // 球关节（nq=4 四元数, nv=3）
    : JointModelBase<JointModelSphericalTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelFreeFlyerTpl          // 6-DOF 浮动基座（nq=7, nv=6）
    : JointModelBase<JointModelFreeFlyerTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelPlanarTpl             // 平面关节（nq=4, nv=3）
    : JointModelBase<JointModelPlanarTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelTranslationTpl        // 纯平动（nq=3, nv=3）
    : JointModelBase<JointModelTranslationTpl<Scalar, Options>> { ... };

template <typename Scalar, int Options = 0>
struct JointModelHelicalTpl            // 螺旋关节（旋转+平动耦合）
    : JointModelBase<JointModelHelicalTpl<Scalar, Options>> { ... };

// 以及 Composite（组合关节）、Mimic（镜像关节）、Universal（万向节）等
```

注意 `nq` 和 `nv` 的区别——这是 Pinocchio（和 MuJoCo）中一个关键概念：`nq` 是配置空间维度（用于存储关节位置），`nv` 是切空间维度（用于存储关节速度和加速度）。对于旋转关节两者都是 1，但对于球关节 `nq=4`（四元数）而 `nv=3`（角速度），对于浮动基座 `nq=7`（位置3 + 四元数4）而 `nv=6`（线速度3 + 角速度3）。这个差异来源于李群的流形结构——配置空间是流形，切空间是线性空间，两者维度不必相等。

### 与 Sophus CRTP 的规模对比

Ch29 中我们分析过 Sophus 的 CRTP：

```
Sophus:  SO3Base<Derived>  →  SO3<Scalar>
                           →  Map<SO3<Scalar>>
                           →  Map<const SO3<Scalar>>
```

这是**一个类型家族**的 CRTP——同一种数学对象（SO(3) 旋转）的不同存储形式（自有存储 vs 映射外部内存）共享运算代码。CRTP 在这里解决的是"如何让 `SO3` 和 `Map<SO3>` 共用 `inverse()`、`log()`、`operator*` 等运算而不用虚函数"。派生类总共 3 个。

Pinocchio 的 CRTP 是另一个量级：

```
Pinocchio:  JointModelBase<Derived>  →  JointModelRevoluteTpl
                                     →  JointModelRevoluteUnalignedTpl
                                     →  JointModelPrismaticTpl
                                     →  JointModelSphericalTpl
                                     →  JointModelFreeFlyerTpl
                                     →  JointModelPlanarTpl
                                     →  JointModelTranslationTpl
                                     →  JointModelHelicalTpl
                                     →  JointModelCompositeTpl
                                     →  JointModelMimicTpl
                                     →  JointModelUniversalTpl
                                     →  ... (12+ 种)
```

这是**不同物理对象**的 CRTP——十几种关节类型，每种有不同的 `nq`/`nv`、不同的 `calc()` 实现、不同的运动子空间矩阵 $S_i$。它们共享的是算法**接口**（`calc`、`jacobian`、`motion`、`force`），而不是实现。

### calc() 的编译期派发

当 RNEA 遍历关节树时，对每个关节调用 `calc()`。由于关节类型在编译期已知（通过 CRTP），编译器可以将 `calc()` 的实际实现内联到 RNEA 的递推循环中。以旋转关节为例：

```cpp
// JointModelRevoluteTpl::calc_impl —— 旋转关节的核心计算
// 给定关节角 q_i，计算该关节的局部变换 M_i 和运动子空间 S_i
template <typename Scalar, int Options>
template <typename ConfigVector>
void JointModelRevoluteTpl<Scalar, Options>::calc_impl(
    JointDataRevoluteTpl<Scalar, Options>& data,
    const Eigen::MatrixBase<ConfigVector>& q) const
{
    const Scalar& q_i = q[idx_q()];     // 取出该关节对应的配置变量
    data.M.rotation(q_i);                // 用 q_i 构造绕轴的旋转矩阵
    data.S.setZero();                    // 运动子空间：只有一个分量非零
    data.S(axis_) = Scalar(1);
    // 整个函数只有几条赋值——编译器将其完全内联
}
```

对比虚函数版本——编译器在 RNEA 的循环中看到的是 `joint_ptr->calc(data, q)`，无法内联，必须在运行时查 vtable → 读函数指针 → 跳转。这三步操作本身只有几纳秒，但它阻止了编译器对循环整体的优化（内联、SIMD、指令重排序）。

### CRTP 的"异构容器"难题与 Variant 解法

纯 CRTP 有一个致命限制：不同 `Derived` 类型在编译期是**不同的类型**，无法放进同一个 `std::vector`。但一个机器人的关节链里每个关节的类型可能不同——Go2 的基座是 FreeFlyer，髋膝踝都是 Revolute——我们需要把这些不同类型按顺序存入同一个容器。

Pinocchio 的解法是 **CRTP + `boost::variant`**（C++17 环境下等价于 `std::variant`）：

```cpp
// 将所有可能的关节类型注册到 variant 类型列表中
typedef boost::variant<
    JointModelRevoluteTpl<Scalar, Options>,
    JointModelPrismaticTpl<Scalar, Options>,
    JointModelSphericalTpl<Scalar, Options>,
    JointModelFreeFlyerTpl<Scalar, Options>,
    JointModelCompositeTpl<Scalar, Options>,
    // ... 共 12+ 种
> JointModelVariant;

// Model 类持有异构关节的 vector
struct ModelTpl {
    std::vector<JointModelVariant> joints;  // 每个元素可以是不同关节类型
    // ...
};

// 算法通过 visitor 模式分派到具体类型
struct CalcVisitor : boost::static_visitor<void> {
    template <typename JointModel>
    void operator()(const JointModel& jmodel) const {
        jmodel.calc(jdata, q);  // 此处 JointModel 是具体类型
    }
};

// RNEA 递推中对每个关节的调用
boost::apply_visitor(CalcVisitor{jdata, q}, model.joints[i]);
```

variant 的分派机制是**编译器生成的 switch-case**（根据 variant 内部的类型索引跳转），而不是 vtable 间接寻址。关键区别在于：switch-case 的每个 case 分支中，编译器**知道**具体类型是什么，因此可以将 `calc()` 的实现**内联**到 case 分支中。vtable 做不到这一点——编译器通过函数指针调用时无法内联。

**性能对比**（7-DOF 机械臂 RNEA，Carpentier et al. 2019）：

| 多态方式 | RNEA 耗时 | 相对基准 | 内联可能性 |
|---------|----------|---------|-----------|
| 虚函数继承 | ~2.5 $\mu$s | 178% | 不可能 |
| **CRTP + variant** | **~1.4 $\mu$s** | **100%** | 部分可能 |
| 纯 CRTP（假设可行） | ~1.3 $\mu$s | 93% | 完全可能 |

> ⚠️ **陷阱：自定义关节类型需要修改 Pinocchio 的 variant 列表**
>
> 如果你需要添加一种 Pinocchio 不支持的关节类型（例如柔性关节、腱驱动关节），仅仅写一个新的 `JointModelMyCustom : JointModelBase<...>` 是不够的。你还必须将它加入 `JointModelVariant` 的类型列表中，否则它无法被存入 `Model::joints`。这意味着你需要修改 Pinocchio 的头文件（`joint-collection.hpp`），然后重新编译 Pinocchio。这是 variant 方案相比虚函数继承的一个工程代价——虚函数继承允许在不修改框架代码的前提下添加新的派生类型。Pinocchio 团队正在通过 `JointModelComposite` 缓解这个问题——你可以将自定义关节表达为已有基本关节类型的组合，而不必注册新类型。

---

## 47.3 Model / Data 分离范式

### 动机——反面：如果不分离会怎样

假设我们不分离 Model 和 Data，而是把拓扑信息和计算缓冲都放在同一个结构体里：

```cpp
// 反面设计：everything-in-one
struct RobotState {
    // 拓扑信息（不变）
    std::vector<JointType> joint_types;
    std::vector<SE3> joint_placements;
    std::vector<Inertia> link_inertias;

    // 计算缓冲（每次算法调用都会被覆写）
    std::vector<SE3> oMi;            // 每个关节在世界坐标系下的位姿
    Eigen::MatrixXd J;               // 雅可比矩阵
    Eigen::MatrixXd M;               // 惯量矩阵
    Eigen::VectorXd nle;             // 非线性效应项（科氏力+重力）
};
```

这个设计的问题：

**问题 1：多线程时必须复制整个对象。** 假设 MPC 线程和 WBC 线程都需要做动力学计算。由于 `oMi`、`J`、`M` 等缓冲是可变的，两个线程不能同时写同一个对象。要么加 mutex（串行化，浪费第二个核心），要么每个线程拷贝一份 `RobotState`。但这意味着**每个线程都持有了 `joint_types`、`joint_placements`、`link_inertias` 的副本**——这些数据从 URDF 加载后就再也不会改变，复制它们纯粹是内存浪费。对于一个 37-DOF 的人形机器人，Model 的常量数据占几百 KB，如果有 4 个线程各复制一份，就浪费了接近 1 MB——在嵌入式控制器上这不是小数目。

**问题 2：无法判断哪些字段是陈旧的。** 当你调用正运动学更新了 `oMi`，然后去读 `J`——`J` 的值是上次 `computeJointJacobians` 计算的结果，可能对应的是**上一个** $q$ 值。在 everything-in-one 设计中，没有任何机制告诉你"哪些缓冲已经过期"。你需要记住调用顺序的依赖关系，这是 bug 的温床。

**问题 3：构造函数职责不清。** 每次创建新线程，你不知道应该从哪里复制——是深拷贝整个对象？还是只拷贝计算缓冲？常量数据该共享还是复制？

### 理论：Model 持有不变量，Data 持有缓冲

Pinocchio 的解法非常干净：

**`pinocchio::Model`**——从 URDF / MJCF 解析后构造，此后**只读**：
- 关节拓扑（父子关系、关节类型、关节在父 body 中的放置位姿 `jointPlacements`）
- 惯性参数（每个 link 的质量 `inertias`、质心位置）
- 关节限位（`lowerPositionLimit`、`upperPositionLimit`、`velocityLimit`、`effortLimit`）
- 命名信息（关节名 `names`、frame 名 `frames`）
- 全局维度常量（`nq`、`nv`、`njoints`、`nframes`、`nbodies`）

**`pinocchio::Data`**——根据 Model 的维度**一次性预分配**所有缓冲，此后每次算法调用只**覆写**：
- `oMi`：每个关节在世界坐标系下的 SE(3) 位姿（`std::vector<SE3>`，长度 `njoints`）
- `v`、`a`：每个关节的 6D 空间速度和加速度
- `f`：每个关节的 6D 空间力
- `J`：雅可比矩阵（$6 \times n_v$）
- `M`：广义惯量矩阵（$n_v \times n_v$）
- `nle`：非线性效应项（$n_v \times 1$）
- `ddq`：广义加速度结果
- `tau`：广义力/力矩结果
- 以及 Cholesky 分解缓冲、RNEA 导数缓冲等——**共 30+ 个预分配成员**

**Data 完整字段清单（按功能分组）**：

理解 Data 的字段结构对调试和性能优化至关重要——不同的算法写入不同的字段，弄混了就会读到过期数据。下表按功能分组列出 Data 最重要的成员，标注了哪些算法会更新它们：

| 分组 | 字段名 | 类型 | 含义 | 由哪些算法填充 |
|------|--------|------|------|---------------|
| **运动学** | `oMi` | `std::vector<SE3>` | 关节在世界坐标系下的位姿 | `forwardKinematics`, `computeAllTerms` |
| | `oMf` | `std::vector<SE3>` | frame 在世界坐标系下的位姿 | `updateFramePlacements`（须在 FK 之后调用） |
| | `liMi` | `std::vector<SE3>` | 关节相对于父 body 的**局部**变换 | `forwardKinematics` |
| | `v` | `std::vector<Motion>` | 每个关节的 6D 空间速度 | `forwardKinematics(model, data, q, v)` |
| | `a` | `std::vector<Motion>` | 每个关节的 6D 空间加速度 | `forwardKinematics(model, data, q, v, a)` |
| **雅可比** | `J` | `Matrix6x` (6 x nv) | 完整雅可比矩阵（所有关节） | `computeJointJacobians` |
| | `dJ` | `Matrix6x` (6 x nv) | 雅可比时间导数 | `computeJointJacobiansTimeVariation` |
| **动力学** | `M` | `MatrixXd` (nv x nv) | 广义惯量矩阵 | `crba`, `computeAllTerms` |
| | `nle` | `VectorXd` (nv) | 非线性效应项 $C\dot{q}+g$ | `nonLinearEffects`, `computeAllTerms` |
| | `g` | `VectorXd` (nv) | 重力项 | `computeGeneralizedGravity` |
| | `tau` | `VectorXd` (nv) | RNEA 输出的关节力矩 | `rnea` |
| | `ddq` | `VectorXd` (nv) | ABA 输出的关节加速度 | `aba` |
| | `f` | `std::vector<Force>` | 每个关节处的 6D 空间力 | `rnea`（RNEA 反向递归中间量） |
| **导数** | `dtau_dq` | `MatrixXd` (nv x nv) | $\partial\tau/\partial q$ | `computeRNEADerivatives` |
| | `dtau_dv` | `MatrixXd` (nv x nv) | $\partial\tau/\partial v$ | `computeRNEADerivatives` |
| | `Minv` | `MatrixXd` (nv x nv) | $M^{-1}$（Cholesky 求逆） | `computeMinverse` |
| **质心** | `com[0]` | `Vector3d` | 系统质心位置 | `centerOfMass` |
| | `vcom[0]` | `Vector3d` | 系统质心速度 | `centerOfMass`（传入 v） |
| | `Jcom` | `Matrix3x` (3 x nv) | 质心雅可比 | `jacobianCenterOfMass` |

> **⚠️ 陷阱：算法之间的依赖关系**
>
> 这些字段之间存在隐式的依赖关系。例如 `computeJointJacobians(model, data, q)` 内部会先调用 `forwardKinematics`，但 **不会** 自动调用 `updateFramePlacements`。如果你接下来用 `getFrameJacobian` 查 frame 的雅可比，必须确保 `updateFramePlacements` 已被调用。Pinocchio 提供了一个便利函数 `computeAllTerms(model, data, q, v)`，一次调用填充 FK + Jacobian + M + nle + CoM 等多个字段，避免遗漏依赖——但它不含 RNEA 导数和 ABA。

**`computeAllTerms` 的正确使用**：

```cpp
// computeAllTerms 是 WBC 中最常用的"一键计算"函数
// 它等价于依次调用：forwardKinematics + crba + nonLinearEffects
//                  + computeJointJacobians + centerOfMass
pinocchio::computeAllTerms(model, data, q, v);

// 调用后可直接读取：
// data.oMi      -> 关节位姿
// data.M        -> 惯量矩阵
// data.nle      -> 非线性效应项 (C*dq + g)
// data.J        -> 完整雅可比
// data.com[0]   -> 质心位置
// data.Jcom     -> 质心雅可比

// 但仍需手动调用：
pinocchio::updateFramePlacements(model, data);  // oMf
pinocchio::computeRNEADerivatives(model, data, q, v, a);  // 导数
```

如果你的控制循环既需要 FK、雅可比、惯量矩阵又需要质心，用 `computeAllTerms` 比分别调用节省约 30% 计算时间（因为中间变量只算一次）。但如果你只需要 FK 不需要惯量矩阵，单独调用 `forwardKinematics` 更快——`computeAllTerms` 会计算你可能不需要的 $M$ 和 $n_{le}$。

**所有算法函数的签名都遵循同一模式**：

```cpp
// const Model& → 只读，不修改
// Data& → 可写，算法将结果写入 Data 的缓冲
pinocchio::forwardKinematics(const Model& model, Data& data,
                             const Eigen::VectorXd& q);

pinocchio::computeJointJacobians(const Model& model, Data& data,
                                 const Eigen::VectorXd& q);

pinocchio::rnea(const Model& model, Data& data,
                const Eigen::VectorXd& q,
                const Eigen::VectorXd& v,
                const Eigen::VectorXd& a);

pinocchio::crba(const Model& model, Data& data,
                const Eigen::VectorXd& q);
```

这个设计的思想本质是**函数式编程在 C++ 中的体现**：Model 是不可变数据（immutable），算法函数是纯函数（输入 Model + q/v/a → 副作用写入 Data），Data 是显式的副作用容器。这好比一个连锁餐厅的运营模式：菜谱（Model）全店统一、永远不改，每个厨房（线程）各自有一套锅碗瓢盆（Data），厨师按菜谱操作自己的工具，互不干扰。换一个更精确的工程类比：Model 和 Data 的关系好比**模具和工件**——模具（Model）定义了形状和尺寸，一旦设计定型就不再修改；工件（Data）是每次加工的产物，用同一套模具可以在不同的车床（线程）上同时生产不同的工件，彼此互不干扰，而且绝不会有人在加工过程中去改模具的形状。

> **本质洞察**：Model/Data 分离表面上是"多线程优化"，但其更深层的价值在于**语义清晰性**——它强制工程师区分"机器人是什么"（拓扑、惯量、限位，不随时间变化）和"机器人此刻在做什么"（位姿、速度、力，每个控制周期都在变化）。这种区分消除了一整类 bug：你不可能意外地在算法执行过程中改变关节拓扑或惯量参数，因为 Model 是 const 的。

### 线程安全：N 线程共享 1 个 Model

Model / Data 分离的直接收益是**天然的多线程安全**：

```cpp
// 构造：一次性加载 URDF
pinocchio::Model model;
pinocchio::urdf::buildModel("go2.urdf",
                            pinocchio::JointModelFreeFlyer(),  // 浮动基座
                            model);

// MPC 线程：自己的 Data
std::thread mpc_thread([&model]() {
    pinocchio::Data data_mpc(model);   // 根据 model 维度预分配
    while (running) {
        pinocchio::forwardKinematics(model, data_mpc, q_current);
        pinocchio::computeJointJacobians(model, data_mpc, q_current);
        // ... 用 data_mpc.oMi 和 data_mpc.J 做 MPC 计算
    }
});

// WBC 线程：自己的 Data
std::thread wbc_thread([&model]() {
    pinocchio::Data data_wbc(model);   // 独立的缓冲
    while (running) {
        pinocchio::rnea(model, data_wbc, q, v, a);
        pinocchio::crba(model, data_wbc, q);
        // ... 用 data_wbc.tau 和 data_wbc.M 做 WBC 计算
    }
});

// 可视化线程：又一个独立 Data
std::thread viz_thread([&model]() {
    pinocchio::Data data_viz(model);
    while (running) {
        pinocchio::forwardKinematics(model, data_viz, q_current);
        // ... 用 data_viz.oMi 渲染
    }
});

// 三个线程共享一个 const Model，各自有独立 Data
// 不需要任何 mutex、atomic 或 lock
```

对照 Ch20 中分析的 ORB-SLAM3——它有 5 个 mutex 保护 `MapPoint`、`KeyFrame`、关键帧队列等共享可变状态。根本原因是 SLAM 的地图是**动态增长**的（每帧都在添加/删除地图点），而腿足机器人的模型在 URDF 加载后是**静态不变**的。Model / Data 分离并不是万能的——它适用于"**读多写少且常量数据和可变数据可以清晰划分**"的场景。

### 与 MuJoCo mjModel / mjData 的映射

MuJoCo 独立发展出了几乎相同的 Model / Data 分离设计。两者的对应关系：

| 概念 | Pinocchio | MuJoCo | 说明 |
|------|-----------|--------|------|
| 不可变模型 | `pinocchio::Model` | `mjModel` | URDF/MJCF 解析后只读 |
| 可变缓冲 | `pinocchio::Data` | `mjData` | 算法写入，一次性预分配 |
| 配置空间维度 | `model.nq` | `m->nq` | 关节位置向量长度 |
| 速度空间维度 | `model.nv` | `m->nv` | 关节速度向量长度 |
| 关节数 | `model.njoints` | `m->njnt` | — |
| body 质量 | `model.inertias[i].mass()` | `m->body_mass[i]` | — |
| 世界坐标系位姿 | `data.oMi[i]` (SE3) | `d->xpos[i]` + `d->xmat[i]` | Pinocchio 用 SE3，MuJoCo 用分离的 pos+mat |
| 雅可比矩阵 | `data.J` | `mj_jac()` 写入用户缓冲 | 存储方式不同 |

**关键差异**：MuJoCo 的 `mjModel` 是纯 C 结构体（`struct mjModel`），所有成员都是裸指针 + 长度字段，面向最大性能和 GPU 兼容性。Pinocchio 的 `Model` 是 C++ 模板类（`ModelTpl<Scalar, Options, JointCollection>`），使用 `std::vector<SE3>`、`Eigen::VectorXd` 等 RAII 容器，面向类型安全和自动微分兼容性。两种设计哲学各有取舍——MuJoCo 追求仿真吞吐量（GPU 上百万环境并行），Pinocchio 追求算法可微性（解析导数 + AD 类型替换）。

### 与 Drake MultibodyPlant 和 RBDL 的对比 ⭐⭐

Pinocchio 并非唯一的 C++ 刚体动力学库。理解三者的定位差异能帮助你在不同项目中做出正确选型。

| 维度 | Pinocchio | Drake MultibodyPlant | RBDL |
|------|-----------|---------------------|------|
| **开发团队** | INRIA Gepetto（法国） | Toyota Research / MIT | Martin Felis（个人+社区） |
| **设计定位** | 纯动力学引擎，极致性能 + AD | 完整机器人仿真+规控框架 | 轻量级教学/原型动力学库 |
| **仿真能力** | 无仿真器（需搭配 MuJoCo/Gazebo） | 内置多体仿真器 + 接触求解 | 无仿真器 |
| **AD 支持** | CppAD / CppADCodeGen / CasADi（模板 Scalar） | `AutoDiffXd`（Drake 自研） | 无原生 AD |
| **解析导数** | RNEA/ABA/CRBA 的闭式递推导数 | 无解析导数（依赖 AD） | 无解析导数 |
| **闭环约束** | v3.x 支持（ConstraintModel + Delassus） | 完整支持（Loop Constraint） | 有限支持（v2.6+） |
| **Python 绑定** | 完整（eigenpy） | 完整（pydrake） | 有限 |
| **依赖大小** | 轻量（Eigen + Boost） | 重量级（需 Bazel 构建，数 GB） | 极轻量（几个头文件） |
| **RNEA 性能（7-DOF）** | ~1.2 $\mu$s | ~3-5 $\mu$s | ~1.5 $\mu$s |
| **上层控制框架** | Crocoddyl, TSID, OCS2, Aligator | Drake 自带 MPC/轨迹优化 | 无 |

**选型建议**：

- **腿足 MPC/WBC**（本课程的核心场景）：**Pinocchio** 是唯一选择——OCS2、Crocoddyl、TSID 都依赖它，且解析导数对 MPC 性能至关重要
- **机械臂操作 + 仿真一体化**：**Drake** 更合适——它提供从仿真到规划到控制的完整管线，减少集成工作量
- **教学或快速原型验证**：**RBDL** 最简单——几个头文件、零额外依赖，10 分钟即可跑通 RNEA
- **同时需要仿真 + 控制**：Pinocchio + MuJoCo（仿真）或 Pinocchio + Gazebo（ROS 集成），而非 Drake 一体化方案——因为腿足社区的主流工具链围绕 Pinocchio 构建

> **反事实推理**：如果 Pinocchio 不提供解析导数会怎样？OCS2 的 SQP-RTI 每次迭代需要完整的动力学 Jacobian。用数值差分，18-DOF 系统需要 37 次 RNEA 调用（中心差分）；用 CppAD，慢 3-5 倍。解析导数将这个代价压缩到 1 次递推，使 MPC 频率从 10-20 Hz 提升到 50-100 Hz——这就是为什么 OCS2 选择 Pinocchio 而非 Drake/RBDL 的决定性理由。

### 代码实战：从 URDF 到 FK

下面是一个完整的"加载 URDF → 创建 Model + Data → 计算正运动学"流程：

```cpp
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <iostream>

int main()
{
    // 1. 构造 Model：从 URDF 文件解析
    pinocchio::Model model;
    pinocchio::urdf::buildModel(
        "go2_description/urdf/go2.urdf",
        pinocchio::JointModelFreeFlyer(),   // 指定基座关节类型
        model);

    std::cout << "模型名称: " << model.name << "\n";
    std::cout << "nq = " << model.nq       // 配置空间维度（Go2: 7+12=19）
              << ", nv = " << model.nv      // 速度空间维度（Go2: 6+12=18）
              << ", njoints = " << model.njoints << "\n";

    // 2. 构造 Data：根据 Model 维度一次性预分配所有缓冲
    pinocchio::Data data(model);
    // 此时 data.oMi 已分配但值未初始化——不要在调用算法前读取！

    // 3. 生成随机合法配置（考虑关节限位和四元数归一化）
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);

    // 4. 计算正运动学：填充 data.oMi（每个关节在世界坐标系下的 SE(3) 位姿）
    pinocchio::forwardKinematics(model, data, q);

    // 5. 读取结果
    for (int i = 0; i < model.njoints; ++i) {
        std::cout << "Joint " << model.names[i]
                  << " 位姿:\n" << data.oMi[i] << "\n";
    }

    // 也可以通过 frame 名称查找特定末端的位姿
    // 注意：需要先调用 updateFramePlacements
    pinocchio::updateFramePlacements(model, data);
    auto frame_id = model.getFrameId("FL_foot");  // 左前足端
    std::cout << "左前足端位姿:\n" << data.oMf[frame_id] << "\n";

    return 0;
}
```

> ⚠️ **陷阱：forwardKinematics 未调用前 oMi 是未初始化数据**
>
> 刚用 `pinocchio::Data data(model)` 构造 Data 时，`data.oMi` 的内存已经分配，但值是**未初始化的**（或者是默认构造的单位变换，取决于 Pinocchio 版本）。如果你在调用 `forwardKinematics` 之前就读取 `data.oMi[i]`，得到的是**毫无意义的旧数据**。同样，`data.J` 在调用 `computeJointJacobians` 之前是脏数据，`data.M` 在调用 `crba` 之前是脏数据。Pinocchio 的 Data 不会自动标记"哪些缓冲已更新"——**你必须记住算法调用的依赖顺序**。这是 Model / Data 分离设计的一个工程代价：灵活性和性能换来了更高的用户责任。
>
> 常见 bug 场景：在 MPC 循环中，上一帧调用过 `forwardKinematics`，这一帧改了 $q$ 但忘记重新调用，然后直接读 `data.oMi`——读到的是**上一帧的位姿结果**，对应的是旧的 $q$ 值。程序不会报错，但控制器会表现出一帧的延迟，在高速运动时可能导致不稳定。

> ⚠️ **陷阱：buildModel 的基座关节类型必须显式指定**
>
> 对于固定基座的机械臂，可以直接调用 `pinocchio::urdf::buildModel("robot.urdf", model)`——此时基座被视为固定在世界坐标系上。但对于腿足机器人（浮动基座），你**必须**传入 `pinocchio::JointModelFreeFlyer()` 作为第二个参数。如果遗漏了这个参数，Pinocchio 会把基座当作固定的，`model.nq` 和 `model.nv` 会少 7 和 6，所有后续的动力学计算都会得到错误的结果——因为浮动基座的 6 个自由度被忽略了。URDF 文件本身**不包含**基座关节类型的信息（URDF 规范假设基座固定），所以 Pinocchio 无法自动推断。

### 练习

**练习 47.3.1**（⭐⭐）：加载你的机器人 URDF（如果没有，用 Pinocchio 自带的 `example-robot-data` 中的 `solo12` 或 `talos`），打印 `model.nq`、`model.nv`、`model.njoints`、`model.nframes`。然后遍历 `model.joints`，对每个关节打印其名称和 `nq_impl()` / `nv_impl()` 值。验证所有关节的 `nq` 之和等于 `model.nq`，所有关节的 `nv` 之和等于 `model.nv`。

**练习 47.3.2**（⭐⭐⭐）：用两个线程模拟 MPC + WBC 的并行计算场景。主线程加载 URDF 构造 Model，然后启动两个线程：一个反复调用 `forwardKinematics` + `computeJointJacobians`，另一个反复调用 `rnea` + `crba`。两个线程各自持有独立的 Data，共享同一个 `const Model`。运行 10 秒，验证没有 crash、没有 data race（可用 ThreadSanitizer 编译检测）。然后尝试故意让两个线程共享同一个 Data 对象（不加锁），观察 ThreadSanitizer 报出的数据竞争警告。
# 第 47 章（下）：Pinocchio 深度精读——模板 Scalar、核心算法实战、约束动力学与碰撞

> **接续 47.3**（CRTP + variant 异构容器解法）。上半章解决了"Pinocchio 为什么要这样设计"的问题——CRTP 消灭虚函数开销，Model/Data 分离实现零锁多线程。但设计哲学只是地基，接下来我们要在这个地基上建房子：从模板化 Scalar 类型的设计哲学到 FK/RNEA/ABA/Jacobian 的完整 Python 代码，再到 v3.x 新增的约束动力学与碰撞接口。

---

## 47.4 模板化 Scalar 类型——一份代码四种用途 ⭐⭐⭐

### 动机：为什么同一个 RNEA 要"变身"四次？

回顾 47.3：Pinocchio 所有核心类型都带 `Scalar` 模板参数。但上一节只讲了"它是什么"，没有讲**"为什么非得这么做"**。答案藏在下游框架的需求里。

考虑一个腿足 MPC 系统的完整计算流水线：

```
[实时控制器]  需要 rnea(q,v,a) → τ       ← 用 double，追求极速
[Crocoddyl]   需要 rnea 的梯度 ∂τ/∂q     ← 用 CppAD::AD<double>，tape-based AD
[OCS2]         需要导出独立 C 代码部署到嵌入式 ← 用 CppAD::cg::CG<double>，代码生成
[CasADi 求解]  需要符号表达式做 NLP          ← 用 casadi::SX，符号计算
```

如果没有模板化 Scalar，你需要**维护四份几乎相同的 RNEA 实现**，每份只有标量运算细节不同。任何一处算法修正（比如 Featherstone 递归中的一个符号错误）都必须同步到四份代码。这在工业代码库中是灾难性的维护负担。

Pinocchio 的回答是：**RNEA 只写一次，通过 `Scalar` 模板参数实例化出四个版本，由编译器保证它们的逻辑完全一致。**

### `ModelTpl<Scalar>` 的模板实例化机制

所有核心类型的模板层级如下：

```cpp
// 层级 1：空间代数类型随 Scalar 变化
SE3Tpl<Scalar>          // 刚体位姿
MotionTpl<Scalar>       // 6D twist
ForceTpl<Scalar>        // 6D wrench
InertiaTpl<Scalar>      // 6D 空间惯量

// 层级 2：关节类型随 Scalar 变化
JointModelRevoluteTpl<Scalar, Options>
JointDataRevoluteTpl<Scalar, Options>

// 层级 3：整个 Model/Data 随 Scalar 变化
ModelTpl<Scalar, Options, JointCollectionTpl>
DataTpl<Scalar, Options, JointCollectionTpl>

// 层级 4：算法函数是 Scalar-generic 的模板函数
template <typename Scalar, int Options, template<...> class JC>
void rnea(const ModelTpl<Scalar,Options,JC>& model,
          DataTpl<Scalar,Options,JC>& data,
          const Eigen::Matrix<Scalar,Dynamic,1>& q,
          const Eigen::Matrix<Scalar,Dynamic,1>& v,
          const Eigen::Matrix<Scalar,Dynamic,1>& a);
```

当你写 `pinocchio::rnea(model_double, data_double, q, v, a)` 时，编译器推导 `Scalar = double`，实例化出纯数值版本。当你写 `pinocchio::rnea(model_ad, data_ad, q_ad, v_ad, a_ad)` 时，编译器推导 `Scalar = CppAD::AD<double>`，实例化出自动微分版本。**算法源码完全相同，只是标量运算被"替换"了。**

### 实战：同一调用 double vs AD 类型

```python
import pinocchio as pin
import numpy as np

# ---- double 版本：纯数值计算 ----
model = pin.buildSampleModelHumanoidRandom()
data  = model.createData()
q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)
a = np.random.randn(model.nv)

tau = pin.rnea(model, data, q, v, a)
print(f"tau (double): {tau[:4]}...")  # 数值向量
```

```cpp
// ---- CppAD 版本（C++ 侧）：同一行调用，得到梯度 ----
typedef CppAD::AD<double> ADScalar;
pinocchio::ModelTpl<ADScalar> model_ad = model.cast<ADScalar>();
pinocchio::DataTpl<ADScalar>  data_ad(model_ad);
Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> q_ad = q.cast<ADScalar>();

CppAD::Independent(q_ad);                                // 开始录制 tape
pinocchio::rnea(model_ad, data_ad, q_ad, v_ad, a_ad);    // 同一行 rnea()
CppAD::ADFun<double> f(q_ad, data_ad.tau);               // 构建函数对象
Eigen::MatrixXd dtau_dq = f.Jacobian(q_val);             // ∂τ/∂q 矩阵
```

```python
# ---- 解析导数（推荐的高性能路线）----
pin.computeRNEADerivatives(model, data, q, v, a)
dtau_dq = data.dtau_dq   # 解析 ∂τ/∂q
dtau_dv = data.dtau_dv   # 解析 ∂τ/∂v
dtau_da = data.M          # ∂τ/∂a = M(q)，即广义惯量矩阵
```

这里有一个关键区分：Pinocchio 同时提供**两种梯度路线**。路线一是 `Scalar = AD` 的自动微分，通用但较慢。路线二是 `computeRNEADerivatives()` 等**手推闭式解析导数**，是 Pinocchio 团队根据 Featherstone 算法的数学结构直接推导出来的，速度比 AD 快 3-5 倍。Crocoddyl 和 Aligator 主要走路线二。

### 为什么下游框架需要不同的 Scalar？

| 下游框架 | 需要的 Scalar | 原因 |
|---------|-------------|------|
| **实时控制器**（1kHz WBC） | `double` | 只需数值结果，追求最低延迟 |
| **Crocoddyl**（DDP 轨迹优化） | `double` + 解析导数 | DDP 的 backward pass 需要 ∂τ/∂q, ∂τ/∂v |
| **OCS2**（嵌入式 MPC 部署） | `CppAD::cg::CG<double>` | 代码生成：导出一个不依赖 Pinocchio 的纯 C 文件，可部署到 ARM 裸机 |
| **CasADi 用户** | `casadi::SX` | 在 CasADi 的符号框架中构建 NLP，用 IPOPT 求解 |

OCS2 的代码生成路线值得单独说明：它用 `CppAD::cg::CG<double>` 实例化 `rnea()`，将整个递归算法"展开"为一个无循环、无分支的纯算术表达式序列，然后编译为高度优化的 C 代码。这个生成的代码**不需要 Pinocchio 头文件**，可以直接在 ARM Cortex-M 等裸机上运行。这是"模板 Scalar"设计最极端也最精彩的应用。

### 与 Ch24 Ceres Jet 的对比

Ch24 讲的 Ceres 用 `Jet<double, N>` 做前向自动微分——把每个 `double` 扩展为 "值 + N 维偏导" 的双数。Pinocchio 的 `Scalar = CppAD::AD<double>` 做的是**反向自动微分（tape-based）**——先录制一遍前向计算的"磁带"，然后反向回放求梯度。

关键差异：

| 特性 | Ceres `Jet<double,N>` | Pinocchio + `CppAD::AD<double>` |
|------|----------------------|-------------------------------|
| 微分方向 | 前向（forward mode） | 反向（reverse mode） |
| 梯度复杂度 | O(N) per 输出，N = 自变量个数 | O(M) per 输入，M = 输出个数 |
| 适合场景 | 输出维度 >> 输入维度（稀疏残差） | 输入维度 >> 输出维度（动力学梯度） |
| 二阶导 | 需要嵌套 Jet（`Jet<Jet<...>>`） | tape 原生支持二阶导 |

腿足机器人的 RNEA 输入是 (q, v, a) 共 3N 维，输出 tau 是 N 维——输入远大于输出，所以**反向 AD 更合适**。这也是 Pinocchio 选择 CppAD 而非 Ceres Jet 的根本原因。但 Pinocchio 的设计并不排斥 Jet——如果你把 `Scalar` 设为 `ceres::Jet<double,N>`，代码一样能编译通过，只是性能特征不同。这种"Scalar 无关"的泛型设计赋予了下游用户完全的选择自由。

> **⚠️ 陷阱：AD 类型的性能代价**
>
> `CppAD::AD<double>` 的每次标量运算都要在"tape"上记录一个操作节点。对于 12-DOF 的 RNEA，`double` 版本耗时约 1.5 μs，AD 版本耗时约 50-150 μs——**慢 30-100 倍**。因此在实时控制循环中**绝不能用 AD 类型做在线计算**。正确做法是：离线用 AD 求一次梯度（或用代码生成导出 C 代码），在线只用 `double` 版本和预计算的解析导数。
>
> 更隐蔽的问题：`CppAD::AD<double>` 的 tape 消耗大量内存。一个 37-DOF 人形机器人的 RNEA tape 可能占用 10-50 MB。如果在循环中反复创建 tape 而不释放，内存会在几秒内耗尽。**正确模式是创建一次 `CppAD::ADFun`，然后反复调用其 `.Jacobian()` 方法。**

---

## 47.5 核心算法实战——FK / RNEA / ABA / Jacobian ⭐⭐

### 完整流程：加载 URDF → 正运动学 → 打印末端位姿

```python
import pinocchio as pin
import numpy as np
from pathlib import Path

# ---------- 1. 加载 URDF ----------
# 使用 example-robot-data 提供的 Panda 模型
from example_robot_data import load as erd_load
robot = erd_load("panda")
model, data = robot.model, robot.model.createData()

print(f"模型名称: {model.name}")
print(f"关节数 njoints: {model.njoints}")   # 含 universe 根关节
print(f"配置空间维度 nq: {model.nq}")        # 7（Panda 是 7-DOF）
print(f"速度空间维度 nv: {model.nv}")        # 7

# ---------- 2. 正运动学（FK）----------
q = pin.neutral(model)                      # 零位构型
pin.forwardKinematics(model, data, q)       # 计算所有关节位姿
pin.updateFramePlacements(model, data)      # 更新 frame 位姿

# 获取末端执行器的位姿
ee_frame_id = model.getFrameId("panda_hand")
ee_pose = data.oMf[ee_frame_id]             # SE3 对象
print(f"末端位置: {ee_pose.translation}")
print(f"末端旋转:\n{ee_pose.rotation}")
```

这段代码的关键调用链是 `buildModelFromUrdf()` -> `createData()` -> `forwardKinematics()` -> `updateFramePlacements()`。注意 `forwardKinematics()` 只更新**关节**位姿（存在 `data.oMi` 中，i 代表 joint index），并不更新**frame** 位姿（存在 `data.oMf` 中）。frame 包括 link 上的附加参考点（如传感器安装位置、末端执行器 TCP），需要额外调用 `updateFramePlacements()`。忘记这一步是初学者最常见的错误之一——`data.oMf` 全是上一次调用的过期数据，但程序不会报错，只会得到错误的位姿。

### RNEA：给定 (q, v, a)，计算 tau

```python
# ---------- 3. 逆动力学（RNEA）----------
q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)
a = np.random.randn(model.nv)

tau = pin.rnea(model, data, q, v, a)
print(f"所需扭矩 tau: {tau}")

# 特殊用法 1：重力补偿项（令 v=0, a=0，只剩重力在各关节产生的扭矩）
tau_gravity = pin.rnea(model, data, q,
                       np.zeros(model.nv), np.zeros(model.nv))
# 等价于：
tau_gravity_check = pin.computeGeneralizedGravity(model, data, q)
assert np.allclose(tau_gravity, tau_gravity_check, atol=1e-14)

# 特殊用法 2：科氏力 + 离心力（令 a=0，减去重力项）
tau_nle = pin.rnea(model, data, q, v, np.zeros(model.nv))
tau_coriolis = tau_nle - tau_gravity  # 纯科氏/离心力项
```

RNEA 是 Pinocchio 中**调用频率最高**的算法。在 Crocoddyl 的一次 DDP 求解中（horizon=100, iterations=50），RNEA 被调用约 5000 次。这就是为什么它必须快到微秒级。

### ABA：给定 (q, v, tau)，计算加速度

```python
# ---------- 4. 正动力学（ABA）----------
tau_input = np.random.randn(model.nv)

a_result = pin.aba(model, data, q, v, tau_input)
print(f"关节加速度: {a_result}")

# 验证互逆性：ABA 和 RNEA 互为逆运算
tau_check = pin.rnea(model, data, q, v, a_result)
error = np.linalg.norm(tau_check - tau_input)
print(f"RNEA(q, v, ABA(q, v, tau)) ≈ tau?  误差={error:.2e}")
# 误差应在 1e-12 量级（浮点精度极限）
```

ABA 比 RNEA 慢约 2 倍（三趟递归 vs 两趟），但仍然是 O(N) 复杂度。在仿真中（给定扭矩求加速度），ABA 是核心；在控制中（给定期望运动求扭矩），RNEA 是核心。两者的互逆关系 `RNEA(q, v, ABA(q, v, tau)) = tau` 是最好的正确性验证手段。

### Jacobian：关节雅可比矩阵

```python
# ---------- 5. 雅可比矩阵 ----------
q = pin.neutral(model)
pin.computeJointJacobians(model, data, q)
pin.updateFramePlacements(model, data)

# 获取末端 frame 的雅可比（6 x nv 矩阵）
J_local = pin.getFrameJacobian(model, data, ee_frame_id, pin.LOCAL)
J_world = pin.getFrameJacobian(model, data, ee_frame_id, pin.WORLD)
J_lwa   = pin.getFrameJacobian(model, data, ee_frame_id,
                                pin.LOCAL_WORLD_ALIGNED)

print(f"J_local shape: {J_local.shape}")    # (6, 7)
print(f"J_world shape: {J_world.shape}")    # (6, 7)
print(f"J_lwa   shape: {J_lwa.shape}")      # (6, 7)
```

### Frame vs Joint 雅可比：三种参考坐标系

这是初学者最常犯错的地方。Pinocchio 提供三种参考坐标系（`ReferenceFrame` 枚举）：

| 枚举值 | 含义 | 速度关系 |
|--------|------|---------|
| `LOCAL` | 表达在**末端 frame 自身坐标系**（body-fixed） | ${}^B v = J_{\text{local}} \cdot \dot{q}$ |
| `WORLD` | 表达在**世界坐标系原点** | ${}^W v = J_{\text{world}} \cdot \dot{q}$ |
| `LOCAL_WORLD_ALIGNED` | 表达在**末端位置处，但朝向与世界坐标系对齐**的坐标系 | ${}^{W,p} v = J_{\text{lwa}} \cdot \dot{q}$ |

三者之间的数学关系：$J_{\text{world}} = {}^W X_B \cdot J_{\text{local}}$，其中 ${}^W X_B$ 是 6x6 伴随变换矩阵。

**完整的坐标系验证实战**——理解三种坐标系最好的方式是数值验证它们之间的关系：

```python
import pinocchio as pin
import numpy as np

# 加载模型并计算 FK
robot = erd_load("panda")
model, data = robot.model, robot.model.createData()
q = pin.randomConfiguration(model)
pin.forwardKinematics(model, data, q)
pin.computeJointJacobians(model, data, q)
pin.updateFramePlacements(model, data)

ee_id = model.getFrameId("panda_hand")

# 获取三种参考坐标系下的 Jacobian
J_local = pin.getFrameJacobian(model, data, ee_id, pin.LOCAL)
J_world = pin.getFrameJacobian(model, data, ee_id, pin.WORLD)
J_lwa   = pin.getFrameJacobian(model, data, ee_id, pin.LOCAL_WORLD_ALIGNED)

# 验证 1: J_world = Ad(oMf) @ J_local
# Ad 是 6x6 伴随变换矩阵
oMf = data.oMf[ee_id]
Ad = np.zeros((6, 6))
Ad[:3, :3] = oMf.rotation
Ad[3:, 3:] = oMf.rotation
Ad[:3, 3:] = pin.skew(oMf.translation) @ oMf.rotation
J_world_check = Ad @ J_local
print(f"J_world 验证误差: {np.linalg.norm(J_world - J_world_check):.2e}")

# 验证 2: J_lwa 只旋转坐标轴，不引入耦合
# J_lwa 的线速度部分直接是末端在世界系下的平移速度
# 而 J_world 的线速度部分包含 ω × p 的耦合（因为参考点在世界原点）
R = oMf.rotation
J_lwa_lin_check = R @ J_local[:3, :]
print(f"J_lwa 线速度验证误差: {np.linalg.norm(J_lwa[:3,:] - J_lwa_lin_check):.2e}")
```

**何时选用哪种坐标系——决策指南**：

| 场景 | 推荐坐标系 | 原因 |
|------|-----------|------|
| IK/WBC 任务误差在世界坐标系下定义 | `LOCAL_WORLD_ALIGNED` | 最直观，线速度直接对应世界系平移 |
| SE(3) log 映射误差（body-fixed） | `LOCAL` | `log6(T_err)` 是 body-fixed 量，雅可比必须匹配 |
| 力控/阻抗控制 | `LOCAL` 或 `LOCAL_WORLD_ALIGNED` | 取决于力的参考坐标系 |
| 质心动量控制 | `WORLD` | 角动量相对世界坐标系原点定义 |
| Crocoddyl / OCS2 | 框架内部约定 | 检查框架文档，不要自行假设 |

`LOCAL_WORLD_ALIGNED` 是**最推荐的默认选择**。它在末端位置处建立一个与世界坐标系平行的坐标系，使得雅可比矩阵的线速度部分直接对应世界坐标系下的平移速度，角速度部分直接对应世界坐标系下的旋转速度，同时避免了 `WORLD` 模式中因末端远离世界原点而产生的非直觉线速度-角速度耦合项。

> **⚠️ 陷阱：Jacobian 参考坐标系选错导致 IK / WBC 静默失败**
>
> 在逆运动学和全身控制中，任务误差通常在世界坐标系下定义（如"末端移到 [0.5, 0, 0.3]"）。如果你用 `LOCAL` 雅可比来做 `dq = J_pinv @ e_world`，关节增量方向会完全错误——因为误差坐标系和雅可比坐标系不匹配。程序不会报错，但机器人会往奇怪的方向运动，IK 永远不收敛。
>
> **规则**：误差在哪个坐标系下定义，就用那个坐标系的雅可比。大多数情况下 `LOCAL_WORLD_ALIGNED` 是最安全的选择。如果你用 SE(3) 对数映射 `pin.log6(T_err)` 计算误差，那个误差是 body-fixed 的，此时应搭配 `LOCAL` 雅可比。

### RNEA 递推公式拆解——理解"为什么 O(N)" ⭐⭐⭐

前面多次提到 RNEA 是 O(N) 复杂度，但还没有解释**为什么**。理解递推公式有助于你在调试时"读懂" Pinocchio 内部在做什么，也是理解 Ch48 CppAD tape 机制的前提。

RNEA 由两趟递归组成——正向传播速度/加速度，反向传播力/力矩：

**正向递归（从基座到末端，i = 1, ..., N）**：

$$v_i = {}^i X_{\lambda(i)} \, v_{\lambda(i)} + S_i \, \dot{q}_i$$

$$a_i = {}^i X_{\lambda(i)} \, a_{\lambda(i)} + S_i \, \ddot{q}_i + v_i \times S_i \, \dot{q}_i$$

其中 $\lambda(i)$ 是关节 $i$ 的父关节，${}^i X_{\lambda(i)}$ 是从父体到子体的 6D 空间变换，$S_i$ 是关节运动子空间矩阵（对旋转关节是 $[0,0,0,0,0,1]^T$ 或其旋转轴方向），$v_i \times$ 是空间速度的叉积算子。

**直觉**：正向递归在做"速度和加速度的逐级传递"——基座的运动通过关节树一级一级传到末端，每一级加上该关节自身的运动贡献。

**反向递归（从末端到基座，i = N, ..., 1）**：

$$f_i = I_i \, a_i + v_i \times^* I_i \, v_i - f_i^{\text{ext}}$$

$$f_{\lambda(i)} \mathrel{+}= {}^{\lambda(i)} X_i^* \, f_i$$

$$\tau_i = S_i^T \, f_i$$

其中 $I_i$ 是 body $i$ 的 6D 空间惯量，$v_i \times^*$ 是空间力的叉积算子，$f_i^{\text{ext}}$ 是外力。

**直觉**：反向递归在做"力的逐级聚合"——先用 Newton-Euler 方程算出每个 body 需要多大的力才能产生要求的加速度，然后从末端往回传，把所有力聚合到每个关节处，最后投影到关节运动方向得到关节力矩。

**为什么是 O(N)**：每个关节只被访问两次（正向一次、反向一次），每次的计算量是常数（6x6 矩阵运算）。总计算量 = 2N 次常数操作 = O(N)。相比之下，直接组装 $M(q)\ddot{q} + h(q,\dot{q}) = \tau$ 再求解，组装 $M$ 需要 O(N^2)，求解线性系统需要 O(N^3)——在高自由度系统上递归方法的优势会越来越明显。

> **跨领域类比**：RNEA 的递推结构与 SLAM 中 Bayes 树的消元过程有深层相似性。SLAM 的因子图消元也是"沿树的正向传播信息、反向聚合信息"，利用稀疏结构将 O(N^3) 降到 O(N)。两者的共同本质是：**树结构允许自底向上/自顶向下的局部计算替代全局矩阵运算**。

### 性能基准

以下数据测量于 Intel i7-12700H（单核），Pinocchio 3.1 + Eigen 3.4（AVX2 开启）：

| 算法 | Panda 7-DOF | Go2 18-DOF | Talos 37-DOF |
|------|-------------|------------|-------------|
| `forwardKinematics` | 0.5 μs | 0.9 μs | 1.8 μs |
| `rnea` | **1.2 μs** | 2.0 μs | 3.5 μs |
| `aba` | **2.5 μs** | 4.0 μs | 7.0 μs |
| `computeRNEADerivatives` | 4.5 μs | 8.0 μs | 16 μs |
| `computeJointJacobians` | 1.0 μs | 1.8 μs | 3.5 μs |
| `crba`（惯量矩阵 M(q)） | 0.8 μs | 1.5 μs | 5.0 μs |

即使是 37-DOF 人形，完整的逆动力学 + 梯度计算也只需约 20 μs。在 1kHz 控制循环的 1000 μs 预算中，Pinocchio 的动力学计算仅占 2%，为上层控制器（QP 求解、轨迹优化）留出了充足的计算余量。

> **⚠️ 陷阱：首次调用延迟与预热**
>
> Python 绑定首次调用 `rnea()` 时会触发动态链接和缓存初始化，耗时可达 100-500 μs。做性能测试时，务必先"预热"几百次调用，再用 `timeit` 统计稳态性能。C++ 侧没有这个问题，但 `Data` 的首次构造涉及内存分配，同样应排除在计时之外。

---

## 47.6 约束动力学（v3.x 新增） ⭐⭐⭐⭐

Pinocchio 2.x 时代只能处理**开链机器人**（树形关节拓扑、无闭环约束）。但真实机器人大量存在闭链结构：

- **平行四连杆膝关节**：Go2、ANYmal C 的膝关节并非简单旋转关节，而是平行四连杆驱动——两根连杆形成闭环
- **人形机器人腰部平行连杆**：Talos、Atlas 的躯干用多连杆并联机构提高刚度
- **Mimic 关节**：一个关节的运动严格跟随另一个（如 Robotiq 2F-85 抓手的欠驱动手指，v3.3+ 原生支持）

Pinocchio 3.x（2024 年起逐步发布）引入了 `ConstraintModelTpl` 体系来处理这些约束。

### 约束建模接口

```python
import pinocchio as pin

model = pin.buildModelFromUrdf("robot_with_closed_loop.urdf")
data  = model.createData()

# 定义一个环路闭合约束：joint_A 和 joint_B 的末端应重合
constraint = pin.RigidConstraintModel(
    pin.ContactType.CONTACT_6D,     # 6D 约束（位置+姿态完全锁定）
    model,
    joint_A_id, placement_A,        # 约束在关节 A 上的附着位姿
    joint_B_id, placement_B         # 约束在关节 B 上的附着位姿
)
constraint_data = constraint.createData()
```

### Delassus 算子与约束求解

约束动力学的核心方程组是：

$$M(q)\,\ddot{q} + h(q, \dot{q}) = \tau + J_c^T \lambda$$

$$J_c\,\ddot{q} + \dot{J}_c\,\dot{q} = 0$$

其中 $\lambda$ 是约束力（拉格朗日乘子），$J_c$ 是约束雅可比矩阵。将第一式中的 $\ddot{q}$ 用 $M^{-1}$ 消去后代入第二式，得到 **Delassus 方程**：

$$G\,\lambda = b, \quad G = J_c\,M^{-1}\,J_c^T$$

矩阵 $G$ 称为 **Delassus 算子**——它描述了约束空间中的"等效惯量"。Pinocchio 3.x 提供了 `computeDelassusMatrix()` 的高效实现，利用树结构稀疏性避免显式构造 $M^{-1}$（直接用 Cholesky 分解的回代实现）。

对于包含不等式约束的接触问题（足端接触的法向力 $\lambda_n \geq 0$、摩擦锥 $\|\lambda_t\| \leq \mu\,\lambda_n$），约束动力学退化为一个 QP。Pinocchio 团队推荐使用他们同时开发的 **ProxQP** 作为后端：

```python
import proxsuite

# 构建接触 QP：min 0.5 x'Hx + g'x, s.t. Ax=b, Cl<=Cu
qp = proxsuite.proxqp.dense.QP(n, n_eq, n_ineq)
qp.init(H, g, A, b, C, l, u)        # H 来自 Delassus, C 编码摩擦锥
qp.solve()
lambda_opt = qp.results.x            # 最优约束力/接触力
```

### 与下游控制框架的衔接

约束动力学是 **Ch53（WBC 全身控制）** 和**接触隐式轨迹优化**的基础设施：

- **TSID / WBC**：将接触力 $\lambda$ 作为决策变量，与 $\ddot{q}$ 和 $\tau$ 一起在 QP 中联合求解——Delassus 算子出现在 QP 的 KKT 矩阵中
- **Aligator（ProxDDP）**：在 DDP 框架中用近端算子直接处理约束，不需要罚函数松弛
- **接触隐式优化**：将接触的建立与断开作为优化问题的一部分，让求解器自动发现最优接触时序——这需要对 Delassus 算子求导，Pinocchio 3.x 的解析导数支持正是为此准备

### ProximalSolver：近端正则化约束求解器 ⭐⭐⭐⭐

Pinocchio 3.x 引入的另一个重要组件是 **ProximalSolver**——一种基于近端算子（proximal operator）的约束动力学求解器。传统 KKT 方法直接求解鞍点问题 $(M, J_c^T; J_c, 0)$，当约束近似冗余（$J_c$ 接近行秩亏）时数值条件极差。ProximalSolver 通过在 Delassus 算子上加正则项 $\mu I$ 来改善条件数：

$$G_\mu = J_c M^{-1} J_c^T + \mu I$$

迭代格式为：

$$\lambda^{k+1} = \text{prox}_{\mathcal{C}} \left( G_\mu^{-1} (b + \mu \lambda^k) \right)$$

其中 $\text{prox}_{\mathcal{C}}$ 是约束集合 $\mathcal{C}$ 上的投影算子（对于摩擦锥约束，投影到二阶锥上）。

**为什么近端方法优于直接 KKT 求解**：

| 特性 | 直接 KKT | ProximalSolver |
|------|---------|----------------|
| 数值稳定性 | 约束冗余时条件数 $\to \infty$ | $\mu$ 正则化保证有界 |
| 不等式约束 | 需要 active set 或 interior point | 投影算子自然处理 |
| 摩擦锥 | 需要线性化（多面体近似） | 直接投影到 SOC 锥 |
| 收敛速度 | 一步精确（但可能不存在解） | 线性收敛（$\mu$ 小时快） |
| 与 Aligator 的关系 | — | Aligator/ProxDDP 内部使用同一套近端框架 |

```python
# ProximalSolver 的使用示意（Pinocchio 3.x）
import pinocchio as pin

model = ...  # 带闭环约束的模型
data  = model.createData()

# 构造约束动力学求解器
prox_settings = pin.ProximalSettings(
    mu=1e-8,           # 近端正则化系数（越小越精确，但收敛越慢）
    accuracy=1e-10,    # 收敛精度
    max_iter=10        # 最大迭代次数
)

# 约束正动力学：给定 (q, v, tau)，求 (a, lambda)
pin.constraintDynamics(
    model, data, q, v, tau,
    contact_models, contact_datas,
    prox_settings
)
# 结果：data.ddq = 加速度, contact_datas[i].contact_force = 接触力
```

> **本质洞察**：ProximalSolver 的核心思想与 ADMM（交替方向乘子法）同源——都是通过"分裂+正则化"将难以直接求解的约束优化问题转化为一系列易解的子问题。$\mu$ 参数的角色是"弹性系数"：$\mu = 0$ 是刚性约束（精确但可能不可解），$\mu > 0$ 是弹性约束（始终有解但引入微小违反）。这个思想在后续 Aligator 的 ProxDDP 中被进一步发展为轨迹优化中的约束处理方案。

### Worked Example：完整的正逆动力学闭环验证

以下代码演示从 URDF 加载到正逆动力学互逆验证的完整流程，可作为项目模板直接使用：

```python
import pinocchio as pin
import numpy as np
from example_robot_data import load as erd_load

# ===== 1. 加载模型 =====
robot = erd_load("go2")
model = robot.model
data  = model.createData()

print(f"模型: {model.name}")
print(f"nq={model.nq}, nv={model.nv}, njoints={model.njoints}")
print(f"nframes={model.nframes}, nbodies={model.nbodies}")

# ===== 2. 随机构型 =====
q = pin.randomConfiguration(model)
v = np.random.randn(model.nv) * 0.1
a = np.random.randn(model.nv) * 0.1

# ===== 3. 逆动力学 (RNEA): (q,v,a) -> tau =====
tau = pin.rnea(model, data, q, v, a)

# ===== 4. 正动力学 (ABA): (q,v,tau) -> a_check =====
a_check = pin.aba(model, data, q, v, tau)

# ===== 5. 互逆性验证 =====
err = np.linalg.norm(a - a_check)
print(f"RNEA-ABA 互逆性误差: {err:.2e}")  # 应 < 1e-12
assert err < 1e-10, "正逆动力学不一致！"

# ===== 6. 惯量矩阵验证：M*a + nle = tau =====
pin.crba(model, data, q)
M = data.M.copy()
M = M + M.T - np.diag(M.diagonal())  # CRBA 只填上三角

nle = pin.nonLinearEffects(model, data, q, v)
tau_check = M @ a + nle
err2 = np.linalg.norm(tau - tau_check)
print(f"M*a + nle = tau 验证误差: {err2:.2e}")  # 应 < 1e-10

# ===== 7. computeAllTerms 一键计算 =====
pin.computeAllTerms(model, data, q, v)
pin.updateFramePlacements(model, data)

# 现在 data.M, data.nle, data.oMi, data.J, data.com 全部可用
print(f"质心位置: {data.com[0]}")
print(f"质心雅可比 shape: {data.Jcom.shape}")  # (3, nv)

# ===== 8. 四个足端位置和雅可比 =====
foot_names = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
for name in foot_names:
    fid = model.getFrameId(name)
    pos = data.oMf[fid].translation
    J_foot = pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED)
    print(f"{name}: pos={pos}, J shape={J_foot.shape}")
```

这个 worked example 覆盖了控制栈中最常用的 Pinocchio API 调用链。在后续 Ch53 WBC 和 Ch55 OCS2 中，你会反复看到 `computeAllTerms` + `getFrameJacobian` 的组合——它们提供 QP 约束矩阵所需的所有信息。

### Worked Example：RNEA 解析导数 vs 数值差分——完整对比流程

这个 worked example 展示如何获取 RNEA 导数并用数值差分验证其正确性。理解这个流程是使用 Ch48 CppADCodeGen 和 Ch55 OCS2 MPC 的前提。

```python
import pinocchio as pin
import numpy as np
import time

robot = erd_load("panda")
model, data = robot.model, robot.model.createData()
q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)
a = np.random.randn(model.nv)

# ===== 方法 1：Pinocchio 解析导数 =====
t0 = time.perf_counter()
for _ in range(10000):
    pin.computeRNEADerivatives(model, data, q, v, a)
t_analytical = (time.perf_counter() - t0) / 10000 * 1e6
dtau_dq_analytical = data.dtau_dq.copy()
dtau_dv_analytical = data.dtau_dv.copy()
# dtau_da = M（惯量矩阵），因为 tau = M*a + nle，所以 ∂tau/∂a = M

# ===== 方法 2：数值差分（中心差分）=====
eps = 1e-8
dtau_dq_numerical = np.zeros((model.nv, model.nv))

t0 = time.perf_counter()
for i in range(model.nv):
    q_plus = pin.integrate(model, q, eps * np.eye(model.nv)[i])
    q_minus = pin.integrate(model, q, -eps * np.eye(model.nv)[i])
    tau_plus = pin.rnea(model, data, q_plus, v, a)
    tau_minus = pin.rnea(model, data, q_minus, v, a)
    dtau_dq_numerical[:, i] = (tau_plus - tau_minus) / (2 * eps)
t_numerical = (time.perf_counter() - t0) / 10000 * 1e6 * 10000 / model.nv

# ===== 对比 =====
err = np.linalg.norm(dtau_dq_analytical - dtau_dq_numerical, ord='fro')
print(f"解析 vs 数值差分误差: {err:.2e}")       # 应 < 1e-4
print(f"解析导数耗时: {t_analytical:.1f} us")   # ~5 us
print(f"数值差分耗时: ~{2*model.nv*t_analytical/5:.0f} us")  # ~20 us
```

注意数值差分中使用 `pin.integrate()` 而非 `q + dq`——这对于包含四元数的配置空间是必要的（直接加法会破坏四元数的单位范数约束，导致 Pinocchio 内部出错）。

---

## 47.7 Pinocchio + Coal 碰撞接口 ⭐⭐⭐

Pinocchio 通过 **Coal**（原名 HPP-FCL，2024 年更名为 Coal）提供碰撞检测和最近距离计算。Coal 支持的几何基元包括球体、胶囊、圆柱、凸多面体、BVH 三角网格等。

> **跨领域类比**：Coal 在 Pinocchio 生态中的角色类似于 OpenCV 在视觉 SLAM 生态中的角色——它是一个底层几何计算引擎，本身不关心机器人学，但被上层框架（Pinocchio / Crocoddyl）封装后成为了不可或缺的基础设施。正如 ORB-SLAM 不自己实现特征匹配而是调用 OpenCV，Crocoddyl 不自己实现碰撞检测而是通过 Pinocchio 调用 Coal。

### 为什么碰撞检测对腿足机器人重要

在腿足 MPC（特别是 Ch67 Perceptive MPC）中，碰撞检测有三个核心用途：

1. **摆动腿碰撞回避**：摆动阶段的腿不能撞到地面障碍物。MPC 需要"摆动轨迹与地面的最近距离"作为约束
2. **躯体碰撞检测**：低矮障碍物可能碰到机器人躯干。需要"躯干几何体与高程图的距离"
3. **自碰撞检测**：人形机器人的双臂和腿可能互相碰撞。需要"机器人自身不同 link 之间的距离"

Coal 提供了这三种场景都需要的底层计算——给定两个几何体在空间中的位姿，计算它们的最近距离和最近点对。

### 碰撞检测与距离计算

```python
import pinocchio as pin

# 加载带碰撞几何的 URDF（第三个返回值是 visual model，此处忽略）
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    "panda.urdf", package_dirs=["/path/to/meshes"]
)
data           = model.createData()
collision_data = collision_model.createData()

q = pin.randomConfiguration(model)

# 更新几何体位置（必须先调用 FK）
pin.forwardKinematics(model, data, q)
pin.updateGeometryPlacements(model, data, collision_model, collision_data, q)

# 碰撞检测：任意一对几何体碰撞即返回 True
is_colliding = pin.computeCollisions(
    model, data, collision_model, collision_data, q, True  # True = 遇到首个碰撞即停
)

# 距离计算：返回所有碰撞对的最近距离
pin.computeDistances(model, data, collision_model, collision_data, q)
for k, dr in enumerate(collision_data.distanceResults):
    if dr.min_distance < 0.05:
        print(f"碰撞对 {k}: 最近距离={dr.min_distance:.4f} m, "
              f"最近点A={dr.nearest_points[0]}, 最近点B={dr.nearest_points[1]}")
```

### 碰撞梯度与轨迹优化集成

仅知道"是否碰撞"对轨迹优化不够用——优化器需要**距离对关节角的梯度** $\partial d / \partial q$，才能将碰撞规避表示为可微约束。Coal 从 v3.0 起支持最近距离的解析导数，Pinocchio 将其封装为：

```python
# 注意：pin.computeDistanceDerivatives() 并非 Pinocchio 的实际 API。
# 碰撞距离梯度需要手动计算：先通过 computeDistances() 获取最近点对,
# 再利用最近点法向量与 Pinocchio 的 frame Jacobian 组合得到 ∂d/∂q。
# 具体步骤（概念伪代码）：
pin.computeDistances(model, data, collision_model, collision_data, q)
pin.computeJointJacobians(model, data, q)
for k, dr in enumerate(collision_data.distanceResults):
    # 法向量 n = (p2 - p1) / ||p2 - p1||
    n = (dr.nearest_points[1] - dr.nearest_points[0])
    n /= np.linalg.norm(n)
    # 获取对应 frame 的 Jacobian，投影到法向量方向得到 ∂d/∂q
    # dd_dq = n^T @ J_p1 - n^T @ J_p2
```

**碰撞距离梯度的数学推导**：

设两个几何体 A 和 B 分别附着在关节 $j_A$ 和 $j_B$ 上，它们的最近距离为 $d(q) = \|p_B(q) - p_A(q)\|$，最近点分别为 $p_A, p_B$。对 $d$ 关于 $q$ 求导：

$$\frac{\partial d}{\partial q} = \hat{n}^T \left( \frac{\partial p_B}{\partial q} - \frac{\partial p_A}{\partial q} \right) = \hat{n}^T \left( J_B(q) - J_A(q) \right)$$

其中 $\hat{n} = (p_B - p_A) / \|p_B - p_A\|$ 是单位法向量，$J_A, J_B$ 是对应最近点的位置 Jacobian（仅取线速度部分的前 3 行）。

这个梯度的物理含义很直观：**距离的变化率等于两个最近点"相互远离的速度"在法向量方向的投影**。如果关节运动使两个点沿法向量方向相互靠近，$\partial d/\partial q < 0$（距离减小）；反之 $\partial d/\partial q > 0$（距离增大）。轨迹优化器利用这个梯度信息"推"关节远离碰撞——这正是 Ch67 Perceptive MPC 中摆动腿碰撞回避约束的数学基础。

### 碰撞对的配置与性能优化

默认情况下，Pinocchio 会检测所有几何体对之间的碰撞——但对于 N 个几何体，这意味着 $O(N^2)$ 次检测，大部分是不必要的（如同一条腿上相邻 link 之间不可能碰撞）。

```python
# 禁用不需要检测的碰撞对（同一条腿的相邻 link）
collision_model.addCollisionPair(
    pin.CollisionPair(geom_id_A, geom_id_B))  # 显式添加需要检测的对

# 或从 SRDF 文件加载碰撞过滤规则
pin.removeCollisionPairs(model, collision_model,
                         "robot.srdf",
                         verbose=True)
# SRDF 文件定义了"哪些几何体对永远不会碰撞"——通常由 MoveIt! Setup Assistant 生成
```

配置合理的碰撞对可以将检测时间从数百 $\mu$s 降到几十 $\mu$s——对于 1 kHz 控制循环中的碰撞约束检查，这个优化是必要的。

> **⚠️ 陷阱：碰撞几何 vs 视觉几何**
>
> URDF 中通常定义了两套几何体：`<collision>` 用于碰撞检测（简化几何，如胶囊体），`<visual>` 用于可视化（精细 mesh）。`pin.buildModelsFromUrdf` 会同时加载两者。碰撞检测**必须使用 collision_model**——如果你误用了 visual_model 做碰撞检测，精细 mesh 的 BVH 查询会慢 10-100 倍，而且对 MPC 的碰撞回避约束来说精度完全没有必要。

这使得**碰撞规避约束可以直接嵌入基于梯度的轨迹优化器**（Crocoddyl、IPOPT），而不需要采样式规划器（RRT/PRM）的离散碰撞查询。对于腿足机器人的全身运动规划（如人形机器人穿越狭窄空间），这是必需能力。

---

## 本章小结

本章下半部分（47.4-47.7）从四个维度展开了 Pinocchio 的工程实战面：

| 维度 | 核心内容 | 解决的问题 |
|------|---------|-----------|
| **模板 Scalar** | 一份算法代码，四种标量类型实例化 | 消灭代码重复；统一数值/AD/代码生成/符号计算 |
| **核心算法** | FK/RNEA/ABA/Jacobian 完整 Python 代码 | 建立 URDF -> Model -> Data -> 算法的完整心智模型 |
| **约束动力学** | ConstraintModel + Delassus + ProxQP | 闭链、接触、mimic 关节——从教科书走向真实机器人 |
| **碰撞接口** | Coal 碰撞/距离查询 + 解析梯度 | 碰撞规避嵌入梯度优化，替代纯采样式规划 |

上半章（47.1-47.3）讲的是 Pinocchio 的**设计哲学**（CRTP、Model-Data 分离、variant 异构容器），本下半章讲的是**如何用它做事**。两者结合才能既知其所以然、又用得顺手。

---

## 练习

### 练习 47-A：加载 Panda URDF 并验证 RNEA 退化形式 ⭐⭐

**任务**：用 Pinocchio Python 绑定加载 Franka Panda 的 URDF（从 `example-robot-data` 获取）。在 5 个随机构型下分别计算：

1. `tau_rnea = pin.rnea(model, data, q, np.zeros(nv), np.zeros(nv))`
2. `tau_grav = pin.computeGeneralizedGravity(model, data, q)`

验证两者之差的范数小于 `1e-14`。

**意义**：理解 RNEA 在 v=0、a=0 时退化为纯重力项计算。这个退化形式在重力补偿控制器中直接使用——它就是"让机器人在任意构型下保持静止所需的关节扭矩"。

### 练习 47-B：FK -> IK 往返验证 ⭐⭐⭐

**任务**：

1. 取一个随机关节角 `q_target`，用 FK 计算末端位姿 `T_target`
2. 从另一个初始构型 `q_init` 出发，用阻尼最小二乘 IK 迭代：

```python
for i in range(200):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    T_current = data.oMf[ee_id]
    e = pin.log6(T_current.inverse() * T_target).vector  # body-fixed 误差
    if np.linalg.norm(e) < 1e-6:
        break
    J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL)
    dq = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
    q = pin.integrate(model, q, dq)
```

3. 验证最终位置误差 < 1mm，姿态误差 < 0.01 rad

**关键点**：必须用 `pin.integrate()` 而非 `q += dq`。配置空间可能包含四元数（自由浮动基座），直接加法会破坏单位范数约束。此外注意 `log6()` 返回的是 body-fixed 误差，所以雅可比要选 `LOCAL`——这正是 47.5 中 Jacobian 参考坐标系规则的直接应用。

### 练习 47-C：RNEA double vs 解析导数性能对比 ⭐⭐⭐

**任务**：对 Panda 7-DOF 模型，用 `timeit` 测量 10000 次调用的平均耗时：

1. `pin.rnea(model, data, q, v, a)`——纯逆动力学
2. `pin.computeRNEADerivatives(model, data, q, v, a)`——解析导数
3. 手写数值差分：对 q 的每一维做 $\pm\epsilon$ 扰动，调用 $2 \times \text{nv} + 1$ 次 RNEA

记录三者耗时比，并验证解析导数与数值差分的结果误差 < 1e-6。

**预期结果**：解析导数约为纯 RNEA 的 4-5 倍耗时；数值差分约为 15-20 倍。解析导数快的原因：它在一次正向/反向递归中"顺便"收集梯度信息，复用了所有中间变量；数值差分必须对每个自变量独立完整地重跑一遍 RNEA，无法复用任何中间结果。

### 练习 47-D：画 CRTP 继承关系图 ⭐⭐

**任务**：阅读以下 Pinocchio 头文件，用 Mermaid 或 PlantUML 画出完整的继承关系图：

| 文件 | 要提取的信息 |
|------|-------------|
| `joint-base.hpp` | `JointModelBase<Derived>` 基类接口 |
| `joint-revolute.hpp` | `JointModelRevoluteTpl`（1-DOF 旋转） |
| `joint-prismatic.hpp` | `JointModelPrismaticTpl`（1-DOF 平移） |
| `joint-free-flyer.hpp` | `JointModelFreeFlyerTpl`（6-DOF 浮动基座） |
| `joint-spherical.hpp` | `JointModelSphericalTpl`（3-DOF 球关节） |
| `joint-collection.hpp` | `JointCollectionDefaultTpl`（variant 类型列表） |

图中须标注三项信息：(a) 每个关节类型的自由度（nq / nv）；(b) CRTP `derived()` 的调用方向（基类 -> 派生类）；(c) variant 包含了哪些具体类型。完成后对照 47.2-47.3 的文字描述做自检——这张图就是上半章全部 CRTP + variant 内容的可视化总结。

---

## 累积项目：从零构建四足站立控制器——本章新增 URDF 加载 + FK/ID 计算模块

本课程的足式方向设置了一个贯穿多章的**累积项目**：从零构建一个能在仿真中让四足机器人（Unitree Go2）稳定站立的控制器。每一章在前一章的基础上添加新模块，最终在 Ch55 集成为完整的 MPC + WBC 控制栈。

**本章（Ch47）是累积项目的起点**，负责搭建动力学计算的基础设施层。具体交付物：

**阶段 1：URDF 加载与模型检查**
- 用 `pinocchio::urdf::buildModel` 加载 Go2 URDF（浮动基座模式），验证 `model.nq=19`、`model.nv=18`、`model.njoints=14`
- 封装为 `RobotModel` 类，持有 `pinocchio::Model`（只读）和一个工厂方法 `createData()` 用于创建线程独立的 `Data`

**阶段 2：正运动学模块**
- 实现 `computeFootPositions(model, data, q)` 函数，一次调用返回四个足端在世界坐标系下的位置（`LOCAL_WORLD_ALIGNED` 参考系）
- 实现 `computeFootJacobians(model, data, q)` 函数，返回四个足端的 $6 \times 18$ 雅可比矩阵

**阶段 3：逆动力学模块**
- 实现 `computeGravityCompensation(model, data, q)` 函数，调用 RNEA（v=0, a=0）计算重力补偿扭矩
- 实现 `computeInverseDynamics(model, data, q, v, a)` 函数，封装完整 RNEA 调用
- 编写单元测试：验证 RNEA 退化形式（练习 47-A）、ABA-RNEA 互逆性

**阶段 4：多线程验证**
- 创建两个线程（模拟 MPC + WBC），各自持有独立 `Data`，共享同一个 `const Model`，循环调用 FK + RNEA，用 ThreadSanitizer 验证无数据竞争

> **后续章节的扩展路线**：Ch48 将为这个项目添加 CppADCodeGen 动力学导数预编译模块 $\to$ Ch53 添加 WBC QP 求解层 $\to$ Ch55 添加 OCS2 MPC 外层 $\to$ 最终在 MuJoCo 仿真中实现稳定站立 + 小幅推扰恢复。

---

## 常见故障与排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| FK 结果全为单位变换，足端位置集中在原点附近 | `forwardKinematics()` 未调用或调用后未执行 `updateFramePlacements()`，`data.oMf` 中仍是未初始化/过期数据 | 在读取 `data.oMf` 前打印 `data.oMi[1]`，确认非单位阵；确保调用顺序为 `forwardKinematics` → `updateFramePlacements` → 读取 |
| 浮动基座模型的 `model.nq` 比预期少 7 | `buildModel()` 时未传入 `JointModelFreeFlyer()` 参数，Pinocchio 将基座视为固定 | 检查 `buildModel` 调用是否传入第二参数；对照 URDF 确认 `model.nq = 7 + n_joints` |
| RNEA 输出力矩全为零或量级异常 | 输入的 $q$ 向量维度与 `model.nq` 不匹配，或四元数部分未归一化 | 用 `pinocchio::normalize(model, q)` 归一化后重新调用；打印 `q.size()` 与 `model.nq` 对照 |
| IK 迭代不收敛，误差不下降 | Jacobian 参考坐标系（LOCAL / WORLD / LOCAL_WORLD_ALIGNED）与误差坐标系不匹配 | 若误差用 `log6(T_err)` 计算（body-fixed），雅可比须用 `LOCAL`；若误差在世界坐标系下定义，用 `LOCAL_WORLD_ALIGNED` |
| 多线程调用 Pinocchio 时出现数据竞争或结果不稳定 | 多个线程共享了同一个 `Data` 对象 | 确保每个线程持有独立的 `pinocchio::Data` 实例，仅共享 `const Model`；用 ThreadSanitizer 编译验证 |
| 约束动力学求解发散（ddq 爆炸或 NaN） | ProximalSolver 的 $\mu$ 太小或约束近乎冗余 | 增大 $\mu$（如 1e-6 → 1e-4）；检查约束雅可比 $J_c$ 的条件数；确认约束不矛盾（如两个约束要求同一关节去不同位置） |
| `computeAllTerms` 后 `data.oMf` 仍为旧值 | `computeAllTerms` 不调用 `updateFramePlacements` | 在 `computeAllTerms` 之后手动调用 `updateFramePlacements(model, data)` |
| 碰撞检测漏检（明显相交的几何体返回 is_colliding=false） | URDF 中碰撞几何体路径错误（mesh 未加载），或 `updateGeometryPlacements` 未在 FK 后调用 | 检查 `collision_model.ngeoms` 是否 > 0；确认 FK → `updateGeometryPlacements` → `computeCollisions` 的调用顺序 |

---

## Pinocchio API 速查手册

以下按使用场景组织最常用的 API 调用模式。这不是 API 文档的替代品，而是根据腿足控制栈的实际需求总结的"食谱"。

### 场景 A：WBC 控制循环（1 kHz）

```python
# 每个控制周期需要的完整计算
pin.computeAllTerms(model, data, q, v)   # M, nle, FK, J, CoM
pin.updateFramePlacements(model, data)   # oMf（frame 位姿）

# 提取 WBC 需要的量
M = data.M                              # 惯量矩阵
nle = data.nle                           # C*dq + g
for foot in foot_ids:
    J = pin.getFrameJacobian(model, data, foot, pin.LOCAL_WORLD_ALIGNED)
    pos = data.oMf[foot].translation
```

### 场景 B：MPC 轨迹优化（需要导数）

```python
# MPC 需要动力学导数
pin.computeRNEADerivatives(model, data, q, v, a)
dtau_dq = data.dtau_dq   # ∂τ/∂q
dtau_dv = data.dtau_dv   # ∂τ/∂v
# Pinocchio 也提供 M 的 Cholesky 分解用于快速求解 M^{-1} b
pin.computeMinverse(model, data, q)
Minv = data.Minv          # M^{-1}
```

### 场景 C：质心动力学（SRBD MPC）

```python
# 单刚体动力学 MPC 需要质心和质心 Jacobian
pin.centerOfMass(model, data, q, v)      # 填充 com, vcom
pin.jacobianCenterOfMass(model, data, q)  # 填充 Jcom
com = data.com[0]        # 质心位置 (3,)
vcom = data.vcom[0]      # 质心速度 (3,)
Jcom = data.Jcom          # 质心 Jacobian (3, nv)
total_mass = pin.computeTotalMass(model)
```

### 场景 D：Model 自省（调试用）

```python
# 遍历所有关节
for i in range(model.njoints):
    print(f"Joint {i}: {model.names[i]}, "
          f"parent={model.parents[i]}, "
          f"nq={model.joints[i].nq()}, nv={model.joints[i].nv()}")

# 遍历所有 frame
for i in range(model.nframes):
    frame = model.frames[i]
    print(f"Frame {i}: {frame.name}, "
          f"type={frame.type}, "
          f"parent_joint={frame.parentJoint}")

# 检查关节限位
print(f"q_lower: {model.lowerPositionLimit}")
print(f"q_upper: {model.upperPositionLimit}")
print(f"v_max:   {model.velocityLimit}")
print(f"tau_max: {model.effortLimit}")
```

### Pinocchio 2.x vs 3.x 迁移要点

如果你使用的是 Pinocchio 3.x（推荐），以下是相对于 2.x 的主要变化：

| 功能 | Pinocchio 2.x | Pinocchio 3.x |
|------|-------------|-------------|
| 加载示例模型 | `EXAMPLE_ROBOT_DATA_MODEL_DIR` 环境变量 | `from example_robot_data import load` |
| 碰撞库名称 | HPP-FCL (`hppfcl`) | Coal (`coal`) |
| 约束动力学 | 不支持 | `ConstraintModelTpl` + `constraintDynamics()` |
| Mimic 关节 | 手工处理 | v3.3+ 原生支持 |
| ProximalSolver | 不存在 | `ProximalSettings` + 近端约束求解 |
| 命名空间 | `pinocchio::` | `pinocchio::` (不变) |
| Python 包名 | `import pinocchio` | `import pinocchio` (不变) |

---

## 延伸阅读

### 必读经典

| 资料 | 类型 | 难度 | 说明 |
|------|------|------|------|
| Featherstone, *Rigid Body Dynamics Algorithms*, Springer 2008 | 教材 | ⭐⭐⭐ | 递归动力学算法的经典教材。RNEA/ABA/CRBA 的完整推导和伪代码均出自本书，是理解 Pinocchio 算法层的必读文献 |
| Carpentier & Mansard, "Analytical Derivatives of Rigid Body Dynamics Algorithms", RSS 2018 | 论文 | ⭐⭐⭐⭐ | Pinocchio 解析导数的理论基础。推导了 RNEA 导数的闭式递推公式，性能比 AD 快 3-5 倍。理解本文是深入 Crocoddyl/Aligator backward pass 的前提 |
| Pinocchio 官方 Tutorial（[gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/)） | 文档 | ⭐⭐ | 官方 API 文档和入门教程。Python 和 C++ 示例覆盖 FK/RNEA/Jacobian 等核心功能，适合边查边用 |

### 进阶与前沿

| 资料 | 类型 | 难度 | 说明 |
|------|------|------|------|
| Pinocchio GitHub 仓库（[github.com/stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)） | 源码 | ⭐⭐ | 阅读 `include/pinocchio/algorithm/rnea.hxx` 可直接看到模板化 RNEA 的实现；`joint-base.hpp` 和 `joint-collection.hpp` 是理解 CRTP + variant 的最佳入口 |
| Carpentier J., et al. (2019) "The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives", SII | 论文 | ⭐⭐⭐ | Pinocchio 的系统级论文，包含完整的性能基准测试和设计决策分析 |
| Jallet W., Bambade A., Mansard N., Carpentier J. (2024) "PROXDDP: Proximal Constrained Trajectory Optimization", RSS | 论文 | ⭐⭐⭐⭐ | Aligator 求解器的理论基础，与 Pinocchio 3.x 的 ProximalSolver 共享数学框架 |
| Singh S., Russell R., Wensing P. (2022) "Efficient Analytical Derivatives of Rigid-Body Dynamics using Spatial Vector Algebra", RA-L | 论文 | ⭐⭐⭐⭐ | ABA 导数的解析递推公式，补全了 RNEA 导数之外的理论空白 |
| Coal (HPP-FCL) GitHub（[github.com/coal-library/coal](https://github.com/coal-library/coal)） | 源码 | ⭐⭐ | Pinocchio 的碰撞检测后端，v3.0 起支持最近距离的解析导数 |
