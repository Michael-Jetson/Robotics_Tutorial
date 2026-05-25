# Eigen 库的高级用法专题

> 本章定位：把 Eigen 从“矩阵计算工具”理解为“表达式模板、内存布局、对齐、零拷贝和自动微分桥梁共同构成的 C++ 数值计算基础设施”。
> 机器人规控代码大量依赖小矩阵、高频矩阵块操作和求解器接口，Eigen 使用不当会直接导致性能抖动、悬垂表达式、未对齐崩溃和隐式堆分配。

## 前置自测

1. `Matrix3d` 和 `MatrixXd(3,3)` 在内存分配上有什么差异？
2. `.noalias()` 解决的是什么问题？什么时候使用它是错误的？
3. `Eigen::Map` 是否拥有被映射内存？如果原始内存释放会怎样？
4. 固定大小可向量化 Eigen 类型放进 `std::vector` 时为什么要关心对齐？
5. 为什么模板化 `Scalar` 能让同一份机器人模型代码支持自动微分？

## 本章目标

学完本章后，你应该能解释 Eigen 表达式模板如何避免临时对象，也能识别 `auto` 保存表达式导致的生命周期问题。
你应该能用 `Map` 和 `Ref` 设计零拷贝接口，知道它们何时不拥有数据。
你还应该能处理 Eigen 对齐、稀疏矩阵构造、SIMD 编译边界和实时路径 malloc 检测。

---

## 5.1 表达式模板与延迟求值 ⭐⭐

这一节解决的问题是：为什么 Eigen 表达式看起来像普通矩阵运算，实际却不是立即计算。

### 动机：机器人代码需要大量小而频繁的线性代数

在控制回路中，常见计算包括：

| 计算 | 维度 | 频率 | 性能风险 |
|------|------|------|----------|
| 姿态误差 | 3x3、3x1 | 1 kHz | 临时对象过多 |
| 雅可比块 | 6xn | 500 Hz | 动态分配 |
| QP Hessian | 数十到数百维 | 100 Hz 到 1 kHz | alias 和稀疏构造 |
| 协方差传播 | 15x15 | 100 Hz | 不必要拷贝 |

如果每个 `A + B + C` 都生成临时矩阵，性能会被内存带宽和分配拖垮。
Eigen 的表达式模板把表达式组织成编译期类型树，通常到赋值时才融合计算。

这像厨房备菜。
不是每切一种菜就装一次盘，而是最后根据菜谱一次下锅。
表达式模板的价值就在于减少中间盘子。

### 反面失败：`auto` 保存了表达式而不是值

```cpp
#include <Eigen/Dense>

double badAutoExpression() {
    Eigen::Vector3d a(1.0, 2.0, 3.0);
    Eigen::Vector3d b(4.0, 5.0, 6.0);
    auto expr = a + b;  // expr 是表达式对象，不是 Vector3d 结果。
    a.setZero();
    return expr.sum();  // 这里可能使用修改后的 a。
}
```

正确做法是在需要保存结果时显式求值。

```cpp
#include <Eigen/Dense>

double goodAutoExpression() {
    Eigen::Vector3d a(1.0, 2.0, 3.0);
    Eigen::Vector3d b(4.0, 5.0, 6.0);
    Eigen::Vector3d value = a + b;
    // 或者 auto value = (a + b).eval();
    a.setZero();
    return value.sum();
}
```

### `.eval()` 的意义

`.eval()` 强制表达式求值为临时结果。
它不是性能魔法，而是生命周期和 alias 边界工具。
当表达式依赖的对象即将改变，或者嵌套表达式会重复计算时，`.eval()` 可以让语义明确。

### `.noalias()` 的边界

矩阵乘法默认可能创建临时对象来避免 alias。
`.noalias()` 告诉 Eigen 左侧不会和右侧共享内存。

```cpp
#include <Eigen/Dense>

void multiplyNoAlias(const Eigen::MatrixXd& A,
                     const Eigen::MatrixXd& B,
                     Eigen::MatrixXd* C) {
    // C 不与 A/B 指向同一存储时，这样可避免乘法临时对象。
    C->noalias() = A * B;
}
```

错误示例：

```cpp
#include <Eigen/Dense>

void wrongNoAlias(Eigen::MatrixXd* A) {
    // 错误：左侧和右侧同一个矩阵，确实存在 alias。
    // noalias 会禁止 Eigen 做保护，结果未定义。
    A->noalias() = (*A) * (*A);
}
```

如果确实要原地平方，应显式 `.eval()`。

```cpp
#include <Eigen/Dense>

void safeInPlaceSquare(Eigen::MatrixXd* A) {
    // 先把右侧乘法落地成临时值，再写回 A，避免原地覆盖污染读取。
    *A = ((*A) * (*A)).eval();
}
```

> 本质洞察：Eigen 表达式的高性能来自“少算少拷贝”，但代价是表达式和值不再总是同一个概念。
> 写 Eigen 代码时要不断问：这一行是在描述一个延迟表达式，还是在持有一个已经计算出的数值对象？

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | `auto expr = A+B` 后修改 A | 结果随对象变化 | 保存的是表达式 | 用具体类型或 `.eval()` |
| 编程 | 滥用 `.noalias()` | 数值错误 | 隐瞒真实 alias | 只在确定不共享存储时使用 |
| 概念 | 认为 `.eval()` 总更快 | 临时对象增多 | 强制求值破坏融合 | 只在生命周期或重复计算需要时用 |
| 思维 | 把 Eigen 当普通数组库 | 难解释性能和 bug | 忽略表达式模板 | 区分表达式和值 |

### 练习

1. 写一个 `auto expr = A.block(...) + B.block(...)` 的例子，修改原矩阵后观察结果。
2. 对 `C = A * B + D` 分别测试 `.noalias()`、`.eval()` 和默认写法，比较分配次数。
3. 解释为什么 `A = A.transpose()` 需要特殊处理，并查找 Eigen 推荐写法。

---

## 5.2 Map、Ref 与零拷贝接口 ⭐⭐

这一节解决的问题是：怎样把 ROS 消息、求解器数组、C API 内存安全地映射成 Eigen 对象。

### `Map` 不拥有内存

`Eigen::Map` 只是视图。
它把一段连续内存解释成矩阵或向量。
原始内存的生命周期必须覆盖 `Map` 使用期。

```cpp
#include <Eigen/Dense>
#include <vector>

double sumJointPosition(const std::vector<double>& q_msg) {
    // Map 只借用 q_msg 的连续内存，不复制关节数组。
    Eigen::Map<const Eigen::VectorXd> q(q_msg.data(),
                                        static_cast<Eigen::Index>(q_msg.size()));
    return q.sum();
}
```

错误写法是返回指向局部数据的 Map。

```cpp
#include <Eigen/Dense>
#include <vector>

Eigen::Map<const Eigen::VectorXd> badReturnMap() {
    std::vector<double> local(12, 0.0);
    // 错误：local 返回后释放，Map 变成悬垂视图。
    return Eigen::Map<const Eigen::VectorXd>(local.data(), local.size());
}
```

### `Ref` 适合函数参数

如果函数只要求“能像向量一样读”，可用 `Eigen::Ref`。
它允许调用者传入 `VectorXd`、`Map` 或某些连续表达式。

```cpp
#include <Eigen/Dense>

double weightedNorm(Eigen::Ref<const Eigen::VectorXd> x,
                    Eigen::Ref<const Eigen::VectorXd> w) {
    // array() 切换到逐元素运算，用权重累加平方误差。
    return (w.array() * x.array().square()).sum();
}
```

`Ref` 的边界是它可能为不兼容表达式生成临时对象。
实时路径中应测试是否分配。
对固定大小接口，直接用 `const Eigen::Vector3d&` 常常更清楚。

### 非连续内存和 stride

点云中字段可能交错存储，例如 XYZRGBXYZRGB。
这时可以用 stride。

```cpp
#include <Eigen/Dense>

void scaleXCoordinates(float* xyzrgb, int n, float scale) {
    using Stride = Eigen::InnerStride<6>;
    // 每 6 个 float 取一个 x 字段，避免拷贝整份点云。
    Eigen::Map<Eigen::VectorXf, 0, Stride> xs(xyzrgb, n);
    xs *= scale;
}
```

### `Map` 与求解器输出

许多 C 求解器返回裸数组。
用 `Map` 可以避免拷贝，但不要让 Map 超过数组生命周期。

```cpp
#include <Eigen/Dense>

struct SolverRawResult {
    const double* x;
    int n;
};

Eigen::VectorXd copySolution(const SolverRawResult& raw) {
    Eigen::Map<const Eigen::VectorXd> view(raw.x, raw.n);
    // 返回拥有数据的 VectorXd，脱离求解器内部缓冲生命周期。
    return Eigen::VectorXd(view);
}
```

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 返回局部数组 Map | 随机崩溃 | Map 不拥有内存 | 返回拥有数据的 Matrix/Vector |
| 编程 | 假设 ROS 字段连续 | 数值错位 | 消息布局有 stride | 检查 point_step 和 field offset |
| 概念 | `Ref` 永远零拷贝 | 实时路径分配 | 不兼容表达式生成临时 | malloc 检测 |
| 工程 | 长期保存求解器 Map | 下次 solve 后数据变化 | 指向内部工作区 | 立即拷贝或限定作用域 |

### 练习

1. 用 `Map` 把 `std::array<double, 12>` 映射为关节向量，并传给接受 `Ref` 的函数。
2. 构造一个带 stride 的点云字段映射，验证提取出的 x 坐标正确。
3. 设计一个 API：内部用 `Ref` 读取输入，但输出由调用者预分配，说明所有权。

---

## 5.3 对齐、容器与 SIMD ⭐⭐⭐

这一节解决的问题是：为什么 Eigen 对齐问题常常在 Release 或特定平台才出现。

### 对齐的动机

SIMD 指令喜欢按固定字节边界读取数据。
Eigen 为固定大小可向量化类型做对齐优化。
如果这些对象被放入未正确对齐的内存，可能导致崩溃或性能下降。

现代 Eigen 和 C++ 标准已经缓解了许多历史问题。
但机器人项目常混合老编译器、第三方库、不同编译选项和自定义 allocator。
因此仍需要理解对齐边界。

### 类成员中的固定大小 Eigen 类型

```cpp
#include <Eigen/Dense>

struct PoseCache {
    // 类中包含固定大小可向量化 Eigen 成员，动态分配时需要对齐 new。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4d T_world_body = Eigen::Matrix4d::Identity();
    Eigen::Vector4d quaternion = Eigen::Vector4d::UnitW();
};
```

当类对象可能通过 `new` 分配，并包含需要对齐的 Eigen 成员时，宏可以确保类级 new 对齐。
在 C++17 之后，过对齐 new 支持更好，但跨平台项目仍常保留该宏。

### 容器中的 Eigen 类型

```cpp
#include <Eigen/Dense>
#include <vector>

// Vector4d 可能有对齐要求，容器分配器显式表达这一约束。
using Vec4List = std::vector<Eigen::Vector4d,
                             Eigen::aligned_allocator<Eigen::Vector4d>>;
```

如果使用 C++17 且 Eigen 版本较新，许多情况默认已可工作。
但公共库接口中显式 allocator 能让意图更清楚。

### 编译选项一致性

`-march=native` 可以让 Eigen 使用当前 CPU 的 SIMD 能力。
但同一个程序中不同编译单元若使用不一致 SIMD ABI，可能出现难查问题。

| 选项 | 价值 | 风险 |
|------|------|------|
| `-O3` | 提升数值代码性能 | Debug 体验差 |
| `-march=native` | 启用本机 SIMD | 二进制不可移植 |
| `-DEIGEN_NO_DEBUG` | 去掉 Eigen 运行检查 | 可能隐藏越界 |
| `-DEIGEN_RUNTIME_NO_MALLOC` | 检测动态分配 | 需调试路径启用 |

### 反面失败：一个库用 AVX，一个库按通用 x86 发布

在大型 C++ 项目中，某个静态库可能用 `-march=native` 编译，另一个库按通用 x86_64 编译。
如果 ABI 或对齐假设不一致，问题可能只在某台机器出现。
因此构建系统应把 SIMD 选项作为目标属性统一管理，而不是散落在环境变量里。

### 练习

1. 写一个含 `Vector4d` 的结构体，分别放入普通 `std::vector` 和 aligned allocator 容器，检查地址对齐。
2. 在 CMake 中为所有数值目标统一设置 SIMD 选项，并说明为什么不能只给某个源文件设置。
3. 对同一段矩阵乘法代码比较 `-O0`、`-O3`、`-O3 -march=native` 的耗时。

---

## 5.4 模板化标量类型与自动微分桥梁 ⭐⭐⭐

这一节解决的问题是：为什么从第一天开始模板化 `Scalar` 会让未来接入自动微分更容易。

### 动机：机器人模型既要数值，也要导数

轨迹优化、MPC、状态估计都需要导数。
手写导数容易错。
自动微分要求同一段代码能在 `double`、`CppAD::AD<double>` 或其他标量类型上运行。

这就要求代码不要把 `double` 写死在每个角落。

### 模板化 SE3 示例

```cpp
#include <Eigen/Dense>

template <typename Scalar>
struct SE3Tpl {
    // 所有矩阵和向量都使用 Scalar，便于替换为自动微分标量。
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Matrix3 R = Matrix3::Identity();
    Vector3 t = Vector3::Zero();

    Vector3 transform(const Vector3& p) const {
        // 刚体变换保持 Scalar 类型，不把中间量强行转成 double。
        return R * p + t;
    }
};

using SE3d = SE3Tpl<double>;
```

### 常见破坏自动微分的写法

```cpp
template <typename Scalar>
Scalar badCost(const Eigen::Matrix<Scalar, 3, 1>& x) {
    double weight = 0.1;  // 可以隐式转换，但会让类型意图混乱。
    return weight * x.squaredNorm();
}
```

更清楚的写法：

```cpp
template <typename Scalar>
Scalar goodCost(const Eigen::Matrix<Scalar, 3, 1>& x) {
    // 常量也构造成 Scalar，保证自动微分类型能贯穿整条计算链。
    const Scalar weight = Scalar(0.1);
    return weight * x.squaredNorm();
}
```

### 数学函数也要支持 Scalar

使用 `std::sin`、`std::cos` 时要确保 AD 类型有对应重载。
模板代码中常通过 `using std::sin;` 让 AD 类型的重载参与查找。

```cpp
#include <cmath>

template <typename Scalar>
Scalar smoothPenalty(Scalar x) {
    using std::sqrt;
    // 让 AD 类型自己的 sqrt 重载参与查找，避免退化到 double 版本。
    return sqrt(x * x + Scalar(1e-6));
}
```

### 练习

1. 把一个只支持 `double` 的二轮车运动学函数改成 `template <typename Scalar>`。
2. 找出代码中所有写死的 `0.0`、`1.0`、`double`，判断哪些会影响 AD 兼容。
3. 用有限差分验证模板化函数在 `double` 下的导数，再接入 AD 类型比较。

---

## 5.5 稀疏矩阵构造与求解 ⭐⭐

这一节解决的问题是：轨迹优化和 SLAM 中的稀疏结构怎样用 Eigen 安全表达。

### 动机：大问题不能用稠密矩阵硬扛

MPC 和轨迹优化的 Hessian 常有带状结构。
SLAM 的正规方程常是稀疏块结构。
如果把这些问题转成 `MatrixXd`，内存和计算都会浪费。

### Triplet 构造模式

```cpp
#include <Eigen/Sparse>
#include <vector>

Eigen::SparseMatrix<double> buildDiagonalSparse(int n, double value) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n);
    for (int i = 0; i < n; ++i) {
        // 每个 triplet 描述一个非零对角元素。
        triplets.emplace_back(i, i, value);
    }
    Eigen::SparseMatrix<double> H(n, n);
    // 构造阶段由 triplet 建立稀疏结构。
    H.setFromTriplets(triplets.begin(), triplets.end());
    H.makeCompressed();
    return H;
}
```

Triplet 适合构造阶段。
实时更新阶段不应每次重新 `setFromTriplets`。
如果稀疏结构固定，应保存非零位置，只更新数值数组。

### 稀疏求解器

| 求解器 | 矩阵类型 | 适合 |
|--------|----------|------|
| `SimplicialLLT` | 对称正定 | 协方差、正规方程 |
| `SimplicialLDLT` | 对称半正定或不定边界较轻 | 带约束线性化 |
| `SparseQR` | 一般稀疏 | 最小二乘但较慢 |
| 外部求解器 | 大规模或特殊结构 | CHOLMOD、Pardiso、QP 后端 |

### 练习

1. 构造一个三阶段 MPC 的块三对角 Hessian，打印非零模式。
2. 比较稠密求解和稀疏求解在 $n=100,1000,5000$ 下的内存占用。
3. 把实时循环中的 Triplet 构造移到初始化阶段，只更新数值。

---

## 5.6 实时路径中的 Eigen 审计 ⭐⭐

这一节解决的问题是：如何确认 Eigen 代码没有在控制循环中偷偷分配。

### 审计方法

```cpp
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>

class EigenRealtimeAudit {
public:
    EigenRealtimeAudit(int n) : A_(n, n), b_(n), x_(n) {
        A_.setIdentity();
        b_.setZero();
        x_.setZero();
    }

    void run() {
        Eigen::internal::set_is_malloc_allowed(false);
        // 这段代码应只使用已有缓冲。
        x_.noalias() = A_ * b_;
        Eigen::internal::set_is_malloc_allowed(true);
    }

private:
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;
    Eigen::VectorXd x_;
};
```

### 审计清单

| 检查项 | 方法 | 修复 |
|--------|------|------|
| 动态对象是否提前 resize | 构造阶段检查尺寸 | 预分配 |
| block 表达式是否保存过久 | 查找 `auto` | 用值对象或 `.eval()` |
| 求解器输出是否拷贝 | 检查 Map 生命周期 | 限定作用域 |
| 稀疏结构是否每周期重建 | 统计 `setFromTriplets` | 缓存结构 |
| 对齐是否一致 | ASan、地址检查 | aligned allocator |

### 练习

1. 对现有控制器开启 `EIGEN_RUNTIME_NO_MALLOC`，找出第一处分配。
2. 把动态 `MatrixXd` 替换成固定大小矩阵，比较性能和栈占用。
3. 解释为什么所有 malloc 都移出实时路径后，仍需要测量最坏耗时。

---

## 5.7 表达式求值规则的深水区 ⭐⭐⭐

这一节解决的问题是：为什么有些 Eigen 表达式应该保持延迟，有些表达式必须提前落地成值。

### 从“少临时对象”到“正确的求值边界”

前面已经看到，表达式模板让 `A + B + C` 不必逐段生成临时矩阵。
但在机器人代码里，真正难的不是知道“Eigen 会延迟求值”，而是判断哪一段表达式应该延迟，哪一段应该立刻求值。
这和运动规划中的轨迹分段很像：不是所有点都要成为关键帧，但接触切换、速度不连续、约束边界这些位置必须明确保存。
Eigen 的 `.eval()` 就是数值表达式里的“关键帧”。

回顾 C++ 模板章节中的一个核心事实：模板实例化不是运行时动态派发，而是在编译期生成具体类型。
Eigen 的表达式模板正是利用这一点，把 `A.block<3,3>(0,0) * v + b` 变成一棵编译期类型树。
树上的每个节点只描述“如何计算”，不一定保存“计算结果”。
这解释了为什么 `auto expr = A + B;` 的类型不是 `MatrixXd`，而是一个包含左右引用的表达式类型。

| 表达式场景 | 默认倾向 | 需要主动处理吗 | 典型写法 |
|------------|----------|----------------|----------|
| 逐元素加减 | 延迟融合 | 通常不用 | `x = a + b + c;` |
| 矩阵乘法 | Eigen 常自动引入临时 | 视 alias 而定 | `C.noalias() = A * B;` |
| block 参与后续修改 | 表达式持引用 | 需要 `.eval()` | `auto v = A.block<3,1>(0,0).eval();` |
| transpose 原地赋值 | alias 明显 | 用专用 API | `A.transposeInPlace();` |
| 复杂子式重复使用 | 可能重复计算 | 保存值 | `MatrixXd JtW = J.transpose() * W;` |

如果不建立“求值边界”的意识，会出现两类相反错误。
第一类是过度 `.eval()`，每个子表达式都落地，导致临时对象、内存写回和 cache 压力上升。
第二类是过度延迟，把依赖旧状态的表达式留到状态已经改变之后才计算，结果语义错了。
机器人控制代码常在同一个循环里更新状态、计算误差、组装 QP、写回缓存。
这些步骤共享许多矩阵块，求值边界一旦模糊，问题会表现为“偶发数值抖动”，而不是干净的编译错误。

> **本质洞察**：`.eval()` 不是“让代码更快”的按钮，而是“把表达式变成值”的语义边界。
> `.noalias()` 不是“让乘法更快”的按钮，而是“我承诺左右不共享存储”的别名边界。
> 一个边界解决生命周期，一个边界解决写入冲突；混淆这两者，是 Eigen 高级用法里最常见的根源性错误。

### 三种求值时机

Eigen 常见求值时机可以分成三层。

| 层级 | 发生时刻 | 例子 | 工程含义 |
|------|----------|------|----------|
| 描述表达式 | 构造表达式对象 | `auto e = A + B` | 几乎不计算，只保存引用和操作 |
| 赋值求值 | 写入 PlainObject | `C = A + B` | 遍历元素，计算结果 |
| 强制求值 | 显式 `.eval()` | `auto C = (A + B).eval()` | 创建拥有数据的对象 |

这三层对应机器人软件中的三种数据形态。
描述表达式像规划器中的“约束描述”，它还不是数值解。
赋值求值像求解器输出，它把描述变成具体数组。
强制求值像日志快照，即使后面的状态继续变化，快照仍表示当时的值。
这样理解后，`auto` 的危险就很自然：它保存的是约束描述，不一定是快照。

### 反面失败：block 表达式跨越状态更新

下面的例子来自状态估计和控制里很常见的模式：先从大状态向量取出姿态误差，再更新大状态。

```cpp
#include <Eigen/Dense>
#include <iostream>

void badBlockLifetime() {
    Eigen::VectorXd state(12);
    state.setLinSpaced(12, 1.0, 12.0);

    auto angular_error = state.segment<3>(3);  // 错误：这里保存的是视图表达式，不是独立值。
    state.segment<3>(3).setZero();             // 更新原始状态，视图随之变化。

    // 这里打印的是 0，而不是更新前的 [4, 5, 6]。
    std::cout << angular_error.transpose() << "\n";
}

void goodBlockSnapshot() {
    Eigen::VectorXd state(12);
    state.setLinSpaced(12, 1.0, 12.0);

    Eigen::Vector3d angular_error = state.segment<3>(3);  // 正确：固定大小向量拥有自己的数据。
    state.segment<3>(3).setZero();                        // 后续修改不会影响快照。

    // 这里打印更新前的角误差。
    std::cout << angular_error.transpose() << "\n";
}
```

这个问题的危险在于它不一定崩溃。
内存仍然有效，类型也合法，只是语义从“保存旧值”变成“跟随原始内存变化”。
在滤波器里，这会让残差计算使用已经更新后的状态；在控制器里，会让期望误差被写回动作覆盖。

### 矩阵乘法的求值策略

矩阵乘法比加法复杂，因为左侧写入可能覆盖右侧读取。
Eigen 对典型乘法会保守处理，例如 `A = A * B` 通常会生成临时对象，防止边算边覆盖。
`.noalias()` 的作用是跳过这种保护。
因此它只适用于“目标矩阵确实不在右侧表达式中出现”的情况。

| 写法 | 语义 | 是否安全 | 说明 |
|------|------|----------|------|
| `C = A * B` | 自动处理必要临时 | 通常安全 | Eigen 会判断乘法求值策略 |
| `C.noalias() = A * B` | 承诺 C 不与 A/B 共享 | 条件安全 | C 是独立输出时推荐 |
| `A.noalias() = A * B` | 隐瞒 alias | 错误 | 左右共享 A |
| `A = (A * B).eval()` | 显式旧值乘法 | 安全 | 语义清楚但会生成临时 |

反事实地看，如果 Eigen 对矩阵乘法也完全延迟并直接写入左侧，那么 `A = A * A` 会在第一列写入后污染后续列的读取。
这类似在做高斯消元时，一边覆盖原矩阵一边还希望使用旧矩阵的未消元行。
没有临时对象，算法的数据依赖关系就被破坏了。

### 编译期尺寸如何影响求值

固定大小矩阵不仅避免堆分配，还能让 Eigen 在编译期知道循环长度。
例如 `Matrix<double, 6, 6>` 的维度是类型的一部分，编译器可以展开循环、寄存器分配和向量化。
`MatrixXd(6,6)` 的尺寸只在运行时知道，即使实际总是 6x6，编译器也不能用同样激进的优化。

| 场景 | 推荐类型 | 原因 |
|------|----------|------|
| SO(3)、SE(3)、空间向量 | `Matrix3d`、`Matrix4d`、`Matrix<double,6,1>` | 尺寸物理固定 |
| Go2 12 关节向量 | `Matrix<double,12,1>` | 控制对象固定 |
| 可变关节数机器人 | `VectorXd` | 模型运行时加载 |
| 大规模稀疏问题 | `SparseMatrix<double>` | 非零结构主导 |

### 求值边界示例：构造 $J^T W J$

QP 组装中经常出现 $H = J^T W J$。
如果直接写 `H.noalias() += J.transpose() * W * J;`，Eigen 可能重复处理子表达式，具体策略取决于维度和表达式成本。
更稳妥的工程写法是显式保存高复用中间量。

```cpp
#include <Eigen/Dense>

void addWeightedJacobian(const Eigen::MatrixXd& J,
                         const Eigen::MatrixXd& W,
                         Eigen::MatrixXd* H) {
    // JtW 会被后续乘以 J，只计算一次，避免复杂表达式重复展开。
    const Eigen::MatrixXd JtW = J.transpose() * W;

    // H 是预分配输出，且不与 J/W/JtW 共用存储，因此 noalias 成立。
    H->noalias() += JtW * J;
}

void addWeightedJacobianSmall(const Eigen::Matrix<double, 6, 12>& J,
                              const Eigen::Matrix<double, 6, 6>& W,
                              Eigen::Matrix<double, 12, 12>* H) {
    // 固定大小场景下，中间量维度明确，栈上存储，适合高频控制循环。
    const Eigen::Matrix<double, 12, 6> JtW = J.transpose() * W;
    H->noalias() += JtW * J;
}
```

这里的关键不是“中间量一定更快”，而是“中间量让代价模型更可控”。
高频控制最怕的是延迟尾部变长。
显式中间量能让你更容易用 profiler 定位耗时，也能让 malloc 检测更直接。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | `auto e = state.segment<3>(i)` 后修改 `state` | 旧误差被新状态覆盖 | `auto` 保存视图表达式 | 用 `Vector3d` 或 `.eval()` |
| 编程 | 对原地乘法使用 `.noalias()` | 矩阵结果错误 | 右侧读取被左侧写入污染 | 去掉 `.noalias()` 或显式 `.eval()` |
| 概念 | 认为表达式对象等于数值对象 | 生命周期难判断 | 表达式保存操作和引用 | 区分“描述”和“快照” |
| 思维 | 所有表达式都提前 `.eval()` | 性能下降 | 破坏融合计算 | 只在生命周期、alias 或重复计算处求值 |

### 练习

1. 构造一个 `state.segment<3>()` 被后续状态更新污染的示例，并用 `Vector3d` 修复。
2. 对 `H += J.transpose() * W * J` 写三种版本，分别测量小矩阵和大矩阵下的端到端时间。
3. 解释为什么 `A = A.transpose()` 推荐使用 `transposeInPlace()` 或 `A = A.transpose().eval()`。

---

## 5.8 内存布局、步长与缓存局部性 ⭐⭐⭐

这一节解决的问题是：为什么同样的矩阵运算，换一个存储顺序或遍历顺序就会明显变慢。

### 列主序是 Eigen 的默认心智模型

Eigen 默认使用列主序（ColumnMajor）。
这意味着矩阵元素在内存中按列连续存放。
对于一个 $3 \times 4$ 矩阵：

```text
矩阵视图:
  a00 a01 a02 a03
  a10 a11 a12 a13
  a20 a21 a22 a23

ColumnMajor 内存:
  a00 a10 a20 | a01 a11 a21 | a02 a12 a22 | a03 a13 a23

RowMajor 内存:
  a00 a01 a02 a03 | a10 a11 a12 a13 | a20 a21 a22 a23
```

这不是数学差异，而是内存线性化差异。
矩阵乘法、分解和 BLAS 生态历史上大量采用列主序，Eigen 默认列主序可以减少与这些工具的转换成本。
但机器人软件还会接触图像、点云、C 数组和求解器接口，它们常常按行或结构体字段组织。
如果不理解布局，`Map` 会把正确的内存解释成错误的矩阵。

### stride 的本质

stride 描述的是“相邻逻辑元素在内存里隔多远”。
在列主序矩阵中，同一列相邻行的 inner stride 通常为 1；相邻列之间的 outer stride 通常等于行数。
在带 padding 的图像、交错点云、求解器工作区中，这两个数可能不再等于默认值。

| 数据来源 | 常见布局 | 风险 | 处理方式 |
|----------|----------|------|----------|
| Eigen 默认矩阵 | ColumnMajor | 与 C 行主序接口不一致 | 明确模板参数 |
| OpenCV 图像 | RowMajor + 行 padding | 每行末尾有额外字节 | 用 stride 或先拷贝 |
| ROS PointCloud2 | AoS 字段交错 | x/y/z 不连续 | 用 offset/stride |
| QP 求解器数组 | 一维 C 数组 | 约定可能列主序或行主序 | 对照求解器文档 |
| GPU SoA 缓冲 | 每个字段独立连续 | 适合并行访问 | Map 为独立向量 |

如果不这样区分，会出现一种很隐蔽的错误：数值没有 NaN，也没有越界，但矩阵被转置或字段错位。
例如把行主序雅可比按列主序映射，控制器看到的 $J$ 仍然是一个 $6 \times 12$ 矩阵，却已经不是物理上的雅可比。
QP 可能仍能求出一个解，但接触力方向会错。

### RowMajor 不是性能开关

很多初学者看到“按行遍历更快”，就把所有矩阵都改成 `RowMajor`。
这通常不是好策略。
布局应该由数据边界决定，而不是由直觉决定。
如果一个矩阵主要被 Eigen 自己做分解和乘法，保持默认列主序更省心。
如果它主要作为外部 C 接口的连续行缓存，`RowMajor` 才可能减少转换。

| 使用方式 | 更自然的布局 | 理由 |
|----------|--------------|------|
| Eigen 内部线性代数 | ColumnMajor | 默认路径、分解器兼容 |
| 逐行写入日志 CSV | RowMajor 或逐行输出 | 行连续更自然 |
| 图像每行扫描 | RowMajor | 与像素行一致 |
| 小固定矩阵 | 默认即可 | 编译器优化主导 |
| 与 Fortran/BLAS 数据交换 | ColumnMajor | 外部约定一致 |

> **本质洞察**：内存布局不是“哪种更快”的全局选择，而是“谁拥有数据、谁消费数据、谁最频繁遍历”的局部契约。
> 机器人系统跨越传感器、估计、规划、控制和日志，错误的布局契约会在模块边界放大。

### `Map` 行主序矩阵

下面示例把一个 C 风格行主序数组映射为 Eigen 矩阵。
关键是把布局写进类型，而不是靠注释提醒。

```cpp
#include <Eigen/Dense>
#include <array>
#include <iostream>

void mapRowMajorJacobian() {
    // 外部 C 接口给出的 2x3 行主序矩阵：[第一行][第二行]。
    std::array<double, 6> raw = {1.0, 2.0, 3.0,
                                 4.0, 5.0, 6.0};

    using RowMajorMat23 = Eigen::Matrix<double, 2, 3, Eigen::RowMajor>;
    Eigen::Map<const RowMajorMat23> J(raw.data());

    // 这里 J(0, 2) 正确等于 3.0，而不是列主序解释下的 5.0。
    std::cout << "J(0,2)=" << J(0, 2) << "\n";
}
```

### 带 padding 图像行的 Map

深度图或代价地图常有行 padding。
逻辑上一行有 `cols` 个元素，物理上一行可能有 `stride` 个元素。
这时不能把整块内存当作紧凑矩阵。

```cpp
#include <Eigen/Dense>

void scaleCostmapRows(float* data, int rows, int cols, int row_stride) {
    using RowMajorDynamic = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using Stride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;

    // outer stride 表示相邻行的间隔，inner stride 表示同一行相邻元素间隔。
    Eigen::Map<RowMajorDynamic, 0, Stride> cost(
        data, rows, cols, Stride(row_stride, 1));

    // 只缩放逻辑区域，不触碰每行末尾的 padding。
    cost *= 0.5f;
}
```

这段代码的工程含义是：算法只关心逻辑矩阵，布局差异被 `Map` 类型吸收。
如果把 padding 忽略掉，第二行会从第一行 padding 处开始，后续所有行都错位。

### 缓存友好的遍历

CPU cache 喜欢连续访问。
如果按列主序存储，却用外层列、内层行的方式遍历，访问连续；如果反过来外层行、内层列，就会跨列跳跃。
对小矩阵影响不大，对大代价地图、点云特征矩阵和批量轨迹矩阵影响明显。

```cpp
#include <Eigen/Dense>

double sumColumnMajorFast(const Eigen::MatrixXd& A) {
    double s = 0.0;
    // Eigen 默认列主序：内层遍历行，访问连续内存。
    for (Eigen::Index j = 0; j < A.cols(); ++j) {
        for (Eigen::Index i = 0; i < A.rows(); ++i) {
            s += A(i, j);
        }
    }
    return s;
}

double sumColumnMajorSlow(const Eigen::MatrixXd& A) {
    double s = 0.0;
    // 这种写法按行扫列，对列主序矩阵会跨步访问。
    for (Eigen::Index i = 0; i < A.rows(); ++i) {
        for (Eigen::Index j = 0; j < A.cols(); ++j) {
            s += A(i, j);
        }
    }
    return s;
}
```

不过在工程代码中，不应为了手写循环而放弃 Eigen 的向量化表达式。
如果操作能写成 `A.colwise().sum()`、`A.array()` 或矩阵乘法，应优先让 Eigen 生成合适的遍历。
手写循环主要用于自定义核函数、复杂条件分支或调试布局。

### 机器人矩阵的布局约定

建议在项目里把常见矩阵的布局约定写成类型别名。
这样读代码时能立刻知道数据边界。

```cpp
#include <Eigen/Dense>

// 机器人内部计算使用默认列主序，便于 Eigen 分解和乘法。
using DenseJacobian = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// 外部 C 接口的行主序缓冲显式标注，避免误映射。
using RowMajorBuffer = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// 固定大小控制矩阵保留默认布局，由编译器和 Eigen 决定最佳访问方式。
using SpatialJacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using WbcHessian12 = Eigen::Matrix<double, 12, 12>;
```

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 用默认 `Map<MatrixXd>` 读取行主序数组 | 矩阵转置或错位 | 默认列主序 | 显式 `RowMajor` |
| 编程 | 忽略图像行 padding | 每行起点漂移 | 物理 stride 大于逻辑列数 | 使用 `Stride` |
| 概念 | 把 RowMajor 当成普遍优化 | 性能不稳定 | 数据消费者不同 | 按模块边界选择 |
| 思维 | 只测 kernel 不测数据转换 | 局部快整体慢 | 布局转换被忽略 | 统计端到端时间 |

### 练习

1. 用同一段 `double raw[12]` 分别按列主序和行主序映射成 3x4 矩阵，手算并打印 `M(1,2)`。
2. 构造一个带 padding 的 4x5 浮点图像，用 `Stride` 只修改逻辑区域，验证 padding 不变。
3. 为一个机器人项目列出“内部矩阵、求解器输入、日志输出、传感器输入”的布局约定。

---

## 5.9 对齐问题的工程边界 ⭐⭐⭐

这一节解决的问题是：哪些对齐问题已经被现代 C++ 缓解，哪些在机器人项目里仍然需要主动防御。

### 对齐不是历史包袱，而是 ABI 契约

对齐问题常被误解为“旧 Eigen 才有的问题”。
确实，C++17 对过对齐 `operator new` 的支持改善了很多场景。
但机器人项目通常同时满足三个条件：编译单元多、第三方库多、部署平台多。
一个模块用 C++17 和 AVX2，另一个模块用旧编译选项；一个容器在主程序里分配，另一个库在插件里释放。
这时对齐就不再是单个类型的问题，而是 ABI 契约问题。

SIMD 读取可以类比叉车搬货。
货箱如果按托盘边界摆好，叉车一次就能搬；如果货箱偏了半个托盘，叉车要么多搬一次，要么需要特殊处理。
对齐的目标不是让数学结果不同，而是让硬件可以用它期望的方式取数。

| 场景 | 风险等级 | 原因 | 推荐动作 |
|------|----------|------|----------|
| 局部 `Matrix3d` | 低 | 栈对齐通常足够 | 正常使用 |
| `std::vector<Vector4d>` | 中 | 容器分配器影响元素地址 | 使用 aligned allocator 或确认标准/版本 |
| 类成员含 `Matrix4d` 且动态分配 | 中 | 类对象地址影响成员地址 | 加 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` |
| 跨库传递 Eigen 对象值 | 高 | ABI 和对齐假设跨边界 | 避免按值传递 |
| 自定义内存池 | 高 | 分配器可能不满足过对齐 | 明确对齐分配 |

### 为什么不要按值传 Eigen 对象

按值传递 `Vector4d` 看起来很小，但它会制造两个问题。
第一，拷贝成本和寄存器传参规则受 ABI 影响，不同平台表现不一致。
第二，某些固定大小向量化类型对栈对齐有要求，跨函数边界时更难诊断。
更稳妥的规则是：输入用 `const Eigen::Ref<const ...>&`、`const MatrixBase<Derived>&` 或具体类型引用；输出由调用者传入或返回 PlainObject。

```cpp
#include <Eigen/Dense>

double badPassByValue(Eigen::Vector4d q) {
    // 不推荐：按值传递会产生拷贝，对齐和 ABI 行为也更复杂。
    return q.squaredNorm();
}

double goodPassByRef(const Eigen::Vector4d& q) {
    // 推荐：固定大小对象按 const 引用传入，语义和成本都清楚。
    return q.squaredNorm();
}

template <typename Derived>
double goodPassExpression(const Eigen::MatrixBase<Derived>& q) {
    // 推荐：模板参数可接受表达式，适合只读且不保存引用的场景。
    return q.squaredNorm();
}
```

### 类成员、容器和内存池

如果类含有固定大小可向量化成员，并且类对象会通过 `new` 创建，保留 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 是低成本的防御。
在库代码中，这个宏还能向读者说明“这个类型有对齐要求”。

```cpp
#include <Eigen/Dense>
#include <memory>
#include <vector>

struct ContactFrameCache {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4d T_world_foot = Eigen::Matrix4d::Identity();
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
};

using ContactFrameCacheList =
    std::vector<ContactFrameCache, Eigen::aligned_allocator<ContactFrameCache>>;

std::unique_ptr<ContactFrameCache> makeCache() {
    // 动态创建时，类级 aligned new 保护固定大小成员的对齐。
    return std::make_unique<ContactFrameCache>();
}
```

如果项目使用自定义内存池，必须确认内存池返回地址满足 `alignof(T)`。
许多实时系统会替换默认分配器以避免 malloc 抖动，但如果只关注“是否分配”，不关注“分配地址是否对齐”，会把一个实时问题换成另一个崩溃问题。

### 对齐检查代码

下面的检查不应放进高频路径，但适合启动阶段断言和单元测试。

```cpp
#include <Eigen/Dense>
#include <cstdint>
#include <iostream>

template <typename T>
bool isAlignedFor(const T* ptr) {
    const std::uintptr_t addr = reinterpret_cast<std::uintptr_t>(ptr);
    return addr % alignof(T) == 0;
}

void checkVector4dAddress(const Eigen::Vector4d& v) {
    // data() 指向实际标量数组，检查它是否满足标量数组对齐要求。
    const std::uintptr_t addr = reinterpret_cast<std::uintptr_t>(v.data());
    std::cout << "data address = " << addr << "\n";
}
```

这类检查只能发现一部分问题。
更完整的方法是结合 AddressSanitizer、Eigen 运行时断言和不同编译选项矩阵测试。
尤其要在目标硬件上测试，而不是只在开发机上测试。

### SIMD 编译选项的统一

在 CMake 中，建议把数值库的编译选项挂在目标上，而不是依赖开发者的环境变量。
例如同一套控制器如果在一台机器上用 AVX2 编译，在另一台机器上用通用 x86_64 编译，性能差异可能被误判为算法问题。

```cpp
// C++ 代码无法直接展示 CMake 目标属性，这里用编译期宏记录当前构建假设。
#include <iostream>

void printSimdBuildInfo() {
#if defined(__AVX2__)
    std::cout << "当前编译启用了 AVX2\n";
#elif defined(__SSE2__)
    std::cout << "当前编译启用了 SSE2\n";
#elif defined(__ARM_NEON)
    std::cout << "当前编译启用了 NEON\n";
#else
    std::cout << "当前编译未检测到常见 SIMD 宏\n";
#endif
}
```

这段代码不是性能优化，而是可观测性。
当日志里同时记录 CPU 型号、编译选项和 Eigen 向量化状态，性能问题会更容易定位。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | Eigen 固定大小类型按值跨库传递 | 某些平台崩溃或慢 | ABI 与对齐不一致 | 使用引用或 `Ref` |
| 编程 | 自定义内存池只保证 8 字节对齐 | Release 偶发崩溃 | SIMD 需要更高对齐 | 按 `alignof(T)` 分配 |
| 概念 | 认为 C++17 后不用管对齐 | 跨平台问题仍存在 | 标准支持不等于所有库一致 | 公共类型保留对齐约定 |
| 思维 | 只在开发机测试 | 上车后才暴露 | CPU 指令集不同 | 在目标硬件上跑压力测试 |

### 练习

1. 写一个包含 `Matrix4d` 成员的类，分别用普通容器和 aligned allocator 容器保存，打印元素地址。
2. 在 CMake 或构建日志中输出 SIMD 宏，记录不同机器上的编译结果。
3. 解释为什么“无 malloc”不等价于“内存使用完全安全”。

---

## 5.10 稀疏矩阵的结构缓存与块组装 ⭐⭐⭐

这一节解决的问题是：怎样把稀疏矩阵从“能构造”推进到“适合实时重复求解”。

### 稀疏矩阵有两个层次：结构和值

稀疏矩阵不是“很多零的稠密矩阵”。
它由非零结构和值两部分组成。
结构回答“哪些位置可能非零”，值回答“这些位置现在是多少”。
SLAM、MPC、接触力 QP 中，结构往往在一段时间内保持不变，而值每个周期变化。

这和机器人 URDF 与当前关节角的关系类似。
URDF 拓扑告诉你哪些连杆相连，这是结构；当前关节角告诉你此刻位姿是多少，这是值。
每个控制周期重新解析 URDF 显然浪费，同理，每个周期重新构造稀疏结构也浪费。

| 问题类型 | 结构是否固定 | 值是否变化 | 工程策略 |
|----------|--------------|------------|----------|
| 固定 horizon MPC | 通常固定 | 每周期变化 | 初始化结构，循环更新值 |
| 滑窗 SLAM | 局部变化 | 每次线性化变化 | 增量维护或分块重建 |
| 接触模式切换 QP | 分相位固定 | 每周期变化 | 为常见接触模式缓存结构 |
| 一次性离线优化 | 不重要 | 变化 | Triplet 构造足够 |

如果不分离结构和值，实时循环会反复分配 triplet、排序、压缩和分析分解。
这些操作的代价不是浮点乘加，而是内存访问和动态分配。
它们最容易造成延迟尾部。

### Triplet 的正确位置

`setFromTriplets` 适合构造阶段。
它能接受未排序 triplet，也会合并重复位置，使用方便。
但这份方便对应的是构造成本。
在固定结构问题中，应把 triplet 用于初始化非零模式，然后保存每个物理块对应的非零索引。

```cpp
#include <Eigen/Sparse>
#include <vector>

class DiagonalSparseCache {
public:
    explicit DiagonalSparseCache(int n) : matrix_(n, n), diagonal_index_(n) {
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(n);
        for (int i = 0; i < n; ++i) {
            triplets.emplace_back(i, i, 1.0);
        }

        // 初始化阶段建立稀疏结构，并压缩存储。
        matrix_.setFromTriplets(triplets.begin(), triplets.end());
        matrix_.makeCompressed();

        // 保存每个对角元素在 valuePtr 中的位置，后续只更新数值。
        for (int k = 0; k < matrix_.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(matrix_, k); it; ++it) {
                if (it.row() == it.col()) {
                    diagonal_index_[it.row()] = it.index();
                }
            }
        }
    }

    void updateDiagonal(const Eigen::VectorXd& d) {
        // 控制循环中只写已有非零项，不重新分配结构。
        double* values = matrix_.valuePtr();
        for (Eigen::Index i = 0; i < d.size(); ++i) {
            values[diagonal_index_[i]] = d(i);
        }
    }

    const Eigen::SparseMatrix<double>& matrix() const { return matrix_; }

private:
    Eigen::SparseMatrix<double> matrix_;
    std::vector<int> diagonal_index_;
};
```

这段代码体现了稀疏实时组装的核心原则：结构查询可以慢，但只能在初始化阶段慢；循环中只做数组写入。

### 块稀疏：机器人问题的自然结构

机器人优化很少是随机稀疏。
它通常是块稀疏。
例如轨迹优化中，每个时间步有一个状态块和一个控制块；相邻时间步通过动力学约束相连。
SLAM 中，每个 pose 是 6 维块，每个 landmark 是 3 维块。
如果只把它们看成单个标量非零，代码会失去物理结构。

| 块来源 | 块大小 | 非零模式 |
|--------|--------|----------|
| IMU 预积分因子 | pose/velocity/bias | 相邻状态块 |
| MPC 动力学约束 | state/control | 块双对角 |
| WBC Hessian | task Jacobian | 关节块密集 |
| 接触约束 | 每个接触 3D 力 | 分脚块对角 |

Eigen 的 `SparseMatrix` 是标量级稀疏存储，不是块稀疏专用库。
但你仍然可以在代码组织上保留块概念：为每个块建立写入函数，内部展开成标量 triplet 或 valuePtr 更新。
这样既能使用 Eigen 求解器，又不丢失机器人问题的物理意义。

### 块写入示例

```cpp
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <vector>

void addDenseBlockTriplets(int row0,
                           int col0,
                           const Eigen::MatrixXd& block,
                           std::vector<Eigen::Triplet<double>>* triplets) {
    // 把一个稠密小块展开成稀疏 triplet，适合初始化或一次性构造。
    for (Eigen::Index r = 0; r < block.rows(); ++r) {
        for (Eigen::Index c = 0; c < block.cols(); ++c) {
            const double value = block(r, c);
            if (value != 0.0) {
                triplets->emplace_back(row0 + static_cast<int>(r),
                                       col0 + static_cast<int>(c),
                                       value);
            }
        }
    }
}

Eigen::SparseMatrix<double> buildBlockTridiagonal(int stages, int block_size) {
    const int n = stages * block_size;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(stages * block_size * block_size * 3);

    const Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(block_size, block_size);
    const Eigen::MatrixXd C = -0.1 * Eigen::MatrixXd::Identity(block_size, block_size);

    for (int k = 0; k < stages; ++k) {
        addDenseBlockTriplets(k * block_size, k * block_size, Q, &triplets);
        if (k + 1 < stages) {
            addDenseBlockTriplets((k + 1) * block_size, k * block_size, C, &triplets);
            addDenseBlockTriplets(k * block_size, (k + 1) * block_size, C, &triplets);
        }
    }

    Eigen::SparseMatrix<double> H(n, n);
    H.setFromTriplets(triplets.begin(), triplets.end());
    H.makeCompressed();
    return H;
}
```

在真正的 MPC 中，`Q` 和 `C` 不一定是常量。
但块位置通常固定。
因此可以先用单位值建立结构，再把每个块的 valuePtr 索引缓存起来。
如果接触模式改变导致约束行数改变，可以为常见模式预建多个矩阵，而不是在每个周期临时重建。

### 分析分解与数值分解

稀疏直接法通常分成两步：分析结构和数值分解。
结构分析决定消元顺序和填充模式；数值分解使用当前矩阵值。
当结构不变、值变化时，应复用结构分析结果。

| 阶段 | 输入 | 是否依赖数值 | 频率 |
|------|------|--------------|------|
| analyzePattern | 非零结构 | 否 | 结构改变时 |
| factorize | 当前数值 | 是 | 每次求解 |
| solve | 右端项 | 是 | 每次求解 |

```cpp
#include <Eigen/Sparse>

class SparseLinearSolverCache {
public:
    void analyze(const Eigen::SparseMatrix<double>& A_pattern) {
        // 只分析非零结构，适合初始化阶段或结构改变时调用。
        solver_.analyzePattern(A_pattern);
        analyzed_ = true;
    }

    Eigen::VectorXd solve(const Eigen::SparseMatrix<double>& A,
                          const Eigen::VectorXd& b) {
        if (!analyzed_) {
            // 保守兜底：如果调用者忘记分析结构，这里做一次完整计算。
            solver_.compute(A);
        } else {
            // 结构不变时，只做数值分解。
            solver_.factorize(A);
        }
        return solver_.solve(b);
    }

private:
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
    bool analyzed_{false};
};
```

如果矩阵结构悄悄变化，但仍然复用旧的分析结果，求解器可能报错或给出错误性能。
因此结构缓存要和问题模式绑定，例如 horizon、接触模式、变量维度、约束维度都应作为缓存键的一部分。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 每周期 `setFromTriplets` | 延迟尾部大 | 重复排序和压缩 | 缓存结构，只更新值 |
| 编程 | 修改结构后继续复用旧分析 | 求解失败或变慢 | 分解器结构信息过期 | 结构变化时重新 `analyzePattern` |
| 概念 | 把块问题写成散乱标量 | 代码难维护 | 丢失物理结构 | 用块写入函数 |
| 思维 | 只看非零数量 | 实际求解仍慢 | 填充和消元顺序主导 | 关注 factorization 时间 |

### 练习

1. 为 $N=20$ 的双积分 MPC 构造块三对角 Hessian，并打印非零数量。
2. 把对角稀疏矩阵的值更新从 `setFromTriplets` 改成 `valuePtr` 写入，比较 1000 次更新耗时。
3. 解释为什么结构固定时 `analyzePattern` 不应放在控制循环中。

---

## 5.11 机器人矩阵计算的高频坑 ⭐⭐⭐

这一节解决的问题是：机器人算法里哪些矩阵写法看起来没问题，却会让控制器数值错误或实时性变差。

### 坑一：把旋转矩阵当普通矩阵更新

旋转矩阵必须满足 $R^T R = I$ 和 $\det(R)=1$。
如果每个周期用普通加法积分：

$$R_{k+1} = R_k + \Delta t \, \hat{\omega} R_k$$

短时间内看起来能跑，但正交性会逐渐漂移。
漂移后的矩阵再参与重力投影、姿态误差和雅可比计算，会把一个物理旋转变成带缩放和剪切的线性变换。

| 更新方式 | 是否保持 SO(3) | 代价 | 适用 |
|----------|----------------|------|------|
| 欧拉加法 | 否 | 低 | 只作推导，不建议长期积分 |
| 每步 SVD 投影 | 是 | 中 | 离线修正 |
| 指数映射 | 是 | 低到中 | 实时姿态积分 |
| 四元数归一化 | 近似是 | 低 | IMU 姿态更新 |

```cpp
#include <Eigen/Dense>
#include <cmath>

Eigen::Matrix3d skew3(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0.0, -w.z(), w.y(),
         w.z(), 0.0, -w.x(),
        -w.y(), w.x(), 0.0;
    return W;
}

Eigen::Matrix3d expSO3Small(const Eigen::Vector3d& omega_dt) {
    const double theta = omega_dt.norm();
    const Eigen::Matrix3d W = skew3(omega_dt);
    if (theta < 1e-8) {
        // 小角度下使用一阶近似，避免除以很小的 theta。
        return Eigen::Matrix3d::Identity() + W;
    }
    const double a = std::sin(theta) / theta;
    const double b = (1.0 - std::cos(theta)) / (theta * theta);
    return Eigen::Matrix3d::Identity() + a * W + b * W * W;
}

void integrateRotation(Eigen::Matrix3d* R,
                       const Eigen::Vector3d& omega_body,
                       double dt) {
    // 使用指数映射更新旋转，避免普通矩阵加法破坏正交性。
    *R = (*R) * expSO3Small(omega_body * dt);
}
```

### 坑二：角度单位和坐标系混用

Eigen 不知道一个 `Vector3d` 是角速度、欧拉角还是平移。
它只知道三个 double。
这意味着坐标系和单位错误不会被类型系统发现。
例如把 IMU body frame 的角速度直接当 world frame 角速度用，维度完全匹配，物理却错。

建议对机器人状态采用命名清楚的结构体，而不是把所有量堆在一个无名向量里。

```cpp
#include <Eigen/Dense>

struct BodyState {
    Eigen::Matrix3d R_world_body = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_world_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_world_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_body = Eigen::Vector3d::Zero();
};

Eigen::Vector3d angularVelocityWorld(const BodyState& state) {
    // 明确把 body 系角速度变换到 world 系，避免坐标系混用。
    return state.R_world_body * state.omega_body;
}
```

如果不做这层命名，代码里很容易出现 `x.segment<3>(9)` 这种语义贫乏的表达。
当状态顺序变化、模型从欧拉角切到四元数、或从 body frame 切到 world frame 时，错误会扩散到所有使用 segment 的位置。

### 坑三：求逆代替求解

`A.inverse() * b` 在教学推导里直观，但工程中通常不推荐。
求逆会放大数值误差，计算量也更高。
更好的写法是根据矩阵性质选择分解器：正定用 Cholesky，一般方阵用 LU，最小二乘用 QR 或 SVD。

| 矩阵性质 | 推荐写法 | 机器人场景 |
|----------|----------|------------|
| 对称正定 | `LLT` / `LDLT` | Hessian、协方差 |
| 一般方阵 | `PartialPivLU` | 小规模线性方程 |
| 过定约束 | `ColPivHouseholderQR` | 最小二乘雅可比 |
| 病态或秩亏 | `JacobiSVD` | 伪逆、奇异姿态 |

```cpp
#include <Eigen/Dense>

Eigen::VectorXd solveDampedLeastSquares(const Eigen::MatrixXd& J,
                                        const Eigen::VectorXd& e,
                                        double damping) {
    const int n = static_cast<int>(J.cols());
    Eigen::MatrixXd H = J.transpose() * J;
    H.diagonal().array() += damping * damping;

    // 使用 LDLT 求解正规方程，避免显式求逆。
    return H.ldlt().solve(J.transpose() * e);
}
```

阻尼最小二乘常用于 IK。
这里的阻尼不是随意加一个小数，而是在奇异位形附近限制关节速度爆炸。
如果不用阻尼，$J^T J$ 的小特征值会让解沿奇异方向急剧放大，机械臂可能突然给出极大关节速度。

### 坑四：归一化零向量

法向量、四元数、轴角方向都需要归一化。
但零向量归一化会产生 NaN。
在接触法向估计、点云平面拟合和姿态误差计算中，退化输入并不罕见。

```cpp
#include <Eigen/Dense>

bool normalizeSafe(Eigen::Vector3d* v, double eps = 1e-12) {
    const double n = v->norm();
    if (n < eps) {
        // 退化向量没有可靠方向，调用者需要切换到备用策略。
        return false;
    }
    *v /= n;
    return true;
}
```

这个函数的价值不在于避免一次除零，而是把“退化情况”显式暴露给上层。
如果接触法向不可用，控制器可以保持上一帧法向或降低接触权重。
如果直接归一化得到 NaN，后续矩阵乘法会把 NaN 扩散到整个 QP。

### 坑五：小矩阵动态分配

机器人代码中很多矩阵维度具有物理含义。
空间向量是 6，四元数是 4，三维点是 3，Go2 关节数是 12。
如果这些尺寸在代码中全部用 `VectorXd` 表达，会让编译器失去信息，也增加动态分配风险。

| 物理对象 | 推荐类型 | 不推荐 |
|----------|----------|--------|
| 角速度 | `Vector3d` | `VectorXd(3)` |
| 空间力 | `Matrix<double,6,1>` | `VectorXd(6)` |
| 四足关节 | `Matrix<double,12,1>` | `VectorXd(12)` |
| 接触力四脚 | `Matrix<double,12,1>` 或数组 `Vector3d` | 嵌套动态向量 |

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | `R += dt * W * R` 长期积分 | 姿态矩阵不正交 | 普通矩阵空间不是 SO(3) | 用指数映射或四元数 |
| 编程 | `A.inverse() * b` | 慢且误差大 | 显式求逆不必要 | 用分解器 `.solve()` |
| 概念 | 认为 `Vector3d` 自带单位和坐标系 | 维度对但物理错 | 类型只表达数量 | 用命名结构体和转换函数 |
| 思维 | 遇到 NaN 只在末端截断 | 故障来源难找 | NaN 已扩散 | 在归一化和除法处防御 |

### 练习

1. 用普通欧拉加法积分旋转 10 秒，打印 $||R^T R-I||_F$，再用指数映射对比。
2. 把一个 `A.inverse()*b` 的 IK 求解改成 `ldlt().solve()` 或 `bdcSvd().solve()`，比较数值差异。
3. 设计一个 `RobotState` 结构体，消除状态向量中至少 5 处裸 `segment` 调用。

---

## 5.12 设计可维护的 Eigen 接口 ⭐⭐

这一节解决的问题是：怎样让 Eigen 代码既快，又不把调用者绑定到某一种矩阵类型。

### 接口设计的三种目标

Eigen 接口设计通常在三件事之间取平衡：

| 目标 | 典型需求 | 推荐工具 |
|------|----------|----------|
| 零拷贝读输入 | 接受 `MatrixXd`、`Map`、block | `Ref` 或 `MatrixBase` |
| 明确写输出 | 调用者预分配 | 非 const `Ref`、指针或引用 |
| 支持固定尺寸优化 | 小矩阵高频 | 具体固定类型或模板尺寸 |

没有一个签名适合所有场景。
`Ref` 适合稳定 ABI 和运行时尺寸接口；`MatrixBase<Derived>` 适合头文件模板和表达式输入；具体类型适合物理尺寸固定的小矩阵。
接口设计的关键是让函数签名表达真实约束。

### 输入只读：`Ref` 与模板的取舍

```cpp
#include <Eigen/Dense>

double normWithRef(const Eigen::Ref<const Eigen::VectorXd>& x) {
    // Ref 接口稳定，适合库边界；但不兼容的表达式可能生成临时。
    return x.norm();
}

template <typename Derived>
double normWithTemplate(const Eigen::MatrixBase<Derived>& x) {
    // 模板接口可接受表达式，适合头文件内联；但会暴露实现到编译单元。
    return x.norm();
}
```

如果函数位于动态库边界，模板会增加编译和 ABI 管理成本，`Ref` 更合适。
如果函数是小型数学工具，模板能保留编译期尺寸，优化空间更大。
如果输入必须是连续动态向量，就直接写 `Ref<const VectorXd>`。
如果输入可以是任意 3 维表达式，就写模板并在内部检查尺寸或使用固定大小派生约束。

### 输出写入：避免隐藏分配

返回 `MatrixXd` 很方便，但在高频路径中可能隐藏分配。
如果输出尺寸固定，返回固定大小对象没有问题。
如果输出尺寸动态且调用频繁，建议让调用者预分配。

```cpp
#include <Eigen/Dense>

void computeTaskResidual(const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& q_ref,
                         Eigen::Ref<Eigen::VectorXd> residual) {
    // 调用者负责 residual 的尺寸和内存，函数只写入已有缓冲。
    residual = q_ref - q;
}

Eigen::Vector3d computePositionError(const Eigen::Vector3d& p,
                                     const Eigen::Vector3d& p_ref) {
    // 固定大小返回值通常在寄存器或栈上处理，语义清晰。
    return p_ref - p;
}
```

反事实地看，如果所有函数都返回动态矩阵，调用链每一层都可能产生临时对象。
单个函数看起来无害，叠加到 1 kHz 控制循环里就会变成不可预测的分配。

### 模板化 Scalar 的接口边界

模板化 `Scalar` 时，函数签名也要避免把 `double` 固定在边界。
一个常见模式是定义 traits。

```cpp
#include <Eigen/Dense>

template <typename Scalar>
struct PlanarPoseTpl {
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Vector3 q = Vector3::Zero();  // [x, y, yaw]
};

template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> headingVector(const PlanarPoseTpl<Scalar>& pose) {
    using std::cos;
    using std::sin;
    const Scalar yaw = pose.q(2);
    // 返回朝向单位向量，Scalar 可以是 double 或自动微分类型。
    return Eigen::Matrix<Scalar, 2, 1>(cos(yaw), sin(yaw));
}
```

不要在模板函数中调用只接受 `double` 的工具函数。
如果确实需要输出到日志或可视化，可以在边界处显式 `static_cast<double>`，并保证这条路径不参与求导。

### API 选择表

| 函数类型 | 推荐签名 | 理由 |
|----------|----------|------|
| 小固定输入 | `const Vector3d&` | 简单、无分配 |
| 动态只读输入 | `Ref<const VectorXd>` | 可接收 Map 和 VectorXd |
| 头文件数学工具 | `MatrixBase<Derived>` | 保留表达式和尺寸 |
| 动态输出 | `Ref<VectorXd>` 或指针 | 调用者预分配 |
| 自动微分模型 | `template <typename Scalar>` | 支持 AD 标量 |

### 接口自检清单

| 检查项 | 问题 | 合格标准 |
|--------|------|----------|
| 所有权 | 函数是否保存输入引用 | 不保存临时表达式引用 |
| 尺寸 | 尺寸在编译期还是运行期 | 签名表达真实尺寸 |
| 分配 | 输出是否会隐式 resize | 高频路径由调用者预分配 |
| 标量 | 是否写死 `double` | 可导路径使用 `Scalar` |
| 布局 | 是否要求连续内存 | 用 `Ref`/`Map` 明确 |

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 函数内部 `VectorXd residual(n)` | 高频分配 | 输出未预分配 | 调用者传入缓冲 |
| 编程 | 保存 `MatrixBase` 引用为成员 | 悬垂引用 | 可能绑定临时表达式 | 保存 PlainObject |
| 概念 | 认为 `Ref` 总是万能 | 某些表达式分配临时 | 内存连续性不满足 | 对实时路径做 malloc 检测 |
| 思维 | 签名只追求短 | 调用约束不清 | 类型没表达语义 | 用类型别名和结构体 |

### 练习

1. 把一个返回 `VectorXd` 的残差函数改成调用者预分配输出，并用 malloc 检测验证。
2. 分别用 `Ref` 和 `MatrixBase` 实现同一个只读范数函数，测试传入 `Map`、`VectorXd`、`segment` 和表达式的行为。
3. 将一个只支持 `double` 的平面位姿函数改成 `Scalar` 模板，并说明哪些边界仍然应该转回 `double`。

---

## 5.13 Map/Ref 深入：与 SLAM 和求解器的零拷贝集成 ⭐⭐⭐

这一节解决的问题是：在多模块机器人系统中，如何用 Map 和 Ref 减少数据拷贝，同时不引入生命周期陷阱。

### SLAM 系统中的零拷贝数据传递

SLAM 系统在前端、后端和地图模块之间传递大量数据：特征点坐标、位姿向量、因子图的雅可比和残差。这些数据通常存储在连续内存中（`std::vector<double>`、裸数组或自定义分配器缓冲），频繁拷贝到 Eigen 矩阵会浪费时间和内存带宽。

`Map` 的价值在于它让 Eigen 的矩阵运算直接作用于外部内存。后端优化器可以把自己的内部状态数组直接映射为 Eigen 向量，避免在"优化器数组"和"算法矩阵"之间来回拷贝。

```cpp
#include <Eigen/Dense>
#include <vector>

class GraphOptimizerBackend {
public:
    explicit GraphOptimizerBackend(int num_poses)
        : state_(num_poses * 6, 0.0) {}  // 每个位姿 6 自由度。

    void linearize(int pose_id) {
        // 直接映射优化器内部数组为 Eigen 向量，无拷贝。
        Eigen::Map<Eigen::Matrix<double, 6, 1>> xi(
            state_.data() + pose_id * 6);

        // 可以直接用 xi 参与矩阵运算，修改会写回 state_。
        xi.head<3>() *= 0.99;  // 示例：对平移部分施加衰减。
    }

    const std::vector<double>& rawState() const { return state_; }

private:
    std::vector<double> state_;
};
```

这段代码的关键是 `state_` 的生命周期覆盖了所有 `Map` 的使用期。如果 `state_` 被 resize 或释放，所有指向它的 `Map` 都会变成悬垂视图。在 SLAM 后端中，这通常不是问题，因为状态向量在初始化后大小固定。但当新关键帧加入导致状态向量增长时，必须重新创建所有 `Map`。

### 求解器接口的 Map 模式

许多 QP 和 NLP 求解器（如 OSQP、qpOASES、IPOPT）使用 C 风格接口，输入和输出都是 `double*` 数组。Eigen 的 Map 是连接 C++ 矩阵代码和 C 求解器接口的标准桥梁。

```cpp
#include <Eigen/Dense>

struct OsqpResult {
    const double* primal;  // 求解器内部分配的解向量。
    int n;
    int status;
};

void processSolution(const OsqpResult& result,
                     Eigen::Ref<Eigen::VectorXd> q_out,
                     Eigen::Ref<Eigen::VectorXd> tau_out,
                     int nq) {
    // 映射求解器输出为 Eigen 视图，不拷贝完整解向量。
    Eigen::Map<const Eigen::VectorXd> x(result.primal, result.n);

    // 从解向量中提取不同物理量的段，写入调用者预分配的输出。
    q_out = x.head(nq);
    tau_out = x.segment(nq, tau_out.size());
    // 注意：x 的生命周期由求解器管理。下次 solve() 后 x 的内容可能改变。
}
```

反事实地看，如果每次都把求解器输出拷贝到一个新 `VectorXd`，在 100 Hz MPC 中每次多一次 malloc 和 memcpy。单次开销不大，但在尾延迟敏感的控制路径中，这种"无害"的拷贝可能成为 p99 抖动的来源。用 `Map` 读取求解器内部缓冲，再通过 `Ref` 写入预分配输出，整条路径没有动态分配。

### Ref 的隐式临时对象风险

`Ref<const VectorXd>` 可以接受 `VectorXd`、`Map<VectorXd>` 和某些连续表达式。但当传入的表达式不满足连续性要求时，Eigen 会生成临时对象来满足 `Ref` 的约束。这个临时对象涉及堆分配。

```cpp
#include <Eigen/Dense>

double normViaRef(Eigen::Ref<const Eigen::VectorXd> x) {
    return x.norm();
}

void refTemporaryDemo() {
    Eigen::MatrixXd M(10, 10);
    M.setRandom();

    // 传入 M 的一列：列在列主序矩阵中是连续的，不会生成临时对象。
    double n1 = normViaRef(M.col(3));

    // 传入 M 的一行：行在列主序矩阵中不连续，Ref 可能生成临时 VectorXd。
    double n2 = normViaRef(M.row(3));  // 潜在隐式分配。

    (void)n1;
    (void)n2;
}
```

在实时路径中，应使用 `EIGEN_RUNTIME_NO_MALLOC` 检测这类隐式分配。如果函数只在非实时路径调用，隐式临时对象不是问题。关键是区分哪些调用在实时路径、哪些不在。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | Map 跨越求解器 re-solve | 读到新解的数据 | 求解器覆盖了内部缓冲 | 在 solve 之后立即提取需要的段 |
| 编程 | 传行向量给 `Ref<const VectorXd>` | 实时路径隐式分配 | 行不连续导致临时 | 先 `.eval()` 或改用模板接口 |
| 概念 | 认为 Map 可以延长原始内存生命期 | use-after-free | Map 不管理内存 | 确保原始内存存活 |
| 工程 | SLAM 状态 resize 后旧 Map 未更新 | 悬垂视图 | resize 可能移动内存 | resize 后重建 Map |

### 练习

1. 用 Map 把一个 QP 求解器的 `double*` 输出映射为关节向量和接触力向量，不做任何拷贝。
2. 测试 `Ref<const VectorXd>` 分别接受列、行和对角线表达式时是否产生临时对象。
3. 设计一个 SLAM 后端的状态数组接口，使用 Map 提供位姿和路标的视图，说明何时需要重建 Map。

---

## 5.14 自定义标量类型与 Ceres Jet 适配 ⭐⭐⭐⭐

这一节解决的问题是：如何让 Eigen 代码在 `double` 和自动微分标量之间无缝切换。

### Ceres Jet 的本质

Ceres Solver 使用 `ceres::Jet<T, N>` 类型实现前向自动微分。一个 `Jet<double, N>` 对象同时携带一个标量值和 N 个偏导数。当你对 Jet 做加、乘、sin、cos 等运算时，运算符重载会自动传播导数。

这和双轨铁路很像。值在一条轨道上走，导数在另一条轨道上走，两条轨道由运算符同步推进。到终点时，你同时拿到了函数值和梯度。

Eigen 的矩阵可以把 Scalar 模板参数设为 `Jet`。这意味着 `Eigen::Matrix<Jet<double, 12>, 3, 1>` 是一个三维向量，每个分量同时携带值和 12 个偏导数。对这个向量做叉积、范数、矩阵乘法，Eigen 的表达式模板会逐元素调用 Jet 的运算符，最终结果自动包含完整的梯度信息。

### Eigen 对自定义标量的要求

Eigen 的矩阵运算不假设 Scalar 是 `float` 或 `double`。它通过 `Eigen::NumTraits<Scalar>` 获取标量类型的数学属性。Ceres 已经为 `Jet` 特化了 `NumTraits`，所以大多数 Eigen 运算"开箱即用"。

但有几个常见的兼容性陷阱：

| 问题 | 原因 | 解决 |
|------|------|------|
| `std::abs` 不接受 Jet | 标准库函数没有 Jet 重载 | 用 `ceres::abs` 或 ADL |
| `if (x > 0)` 不编译 | Jet 的比较返回不一定是 bool | 用 `x.a > 0` 取值部分 |
| 硬编码 `0.0` | 不能隐式转换为 Jet | 用 `Scalar(0.0)` |
| 三角函数 | `std::sin(Jet)` 需要 ADL | 用 `using std::sin; sin(x);` |

```cpp
#include <Eigen/Dense>
#include <cmath>

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> rodriguesSmall(
    const Eigen::Matrix<Scalar, 3, 1>& omega_dt) {
    using std::sqrt;
    using std::sin;
    using std::cos;

    const Scalar theta_sq = omega_dt.squaredNorm();
    const Scalar theta = sqrt(theta_sq + Scalar(1e-14));

    // 所有数学函数通过 ADL 查找 Jet 重载。
    // 常量用 Scalar() 构造，保证 Jet 类型能贯穿计算链。
    const Scalar sinc = sin(theta) / theta;
    return sinc * omega_dt;
}
```

### 与 Sophus 和 manif 的集成模式

Sophus 和 manif 是机器人学中常用的李群库。两者都支持模板化 Scalar，因此可以直接与 Ceres Jet 配合使用。

Sophus 的 `SO3<Scalar>` 和 `SE3<Scalar>` 在内部使用 `Eigen::Matrix<Scalar, ...>`。当 Scalar 是 Jet 时，李群的 exp、log、伴随等运算自动携带导数。这使得"用 Sophus 写模型，用 Ceres 求导"成为可能。

```cpp
// 概念示例：展示 Sophus + Ceres 的类型兼容性。
// #include <sophus/se3.hpp>
// #include <ceres/jet.h>

// template <typename Scalar>
// Scalar poseResidual(const Sophus::SE3<Scalar>& T_measured,
//                     const Sophus::SE3<Scalar>& T_estimated) {
//     // log 返回切空间向量，Scalar 为 Jet 时自动携带导数。
//     auto error = (T_measured.inverse() * T_estimated).log();
//     return error.squaredNorm();
// }
```

manif 的设计类似，但 API 风格不同。两者的共同原则是：模型代码只依赖 `Scalar` 模板参数和 Eigen 矩阵类型，不直接依赖具体的自动微分框架。这样同一份代码可以在 Ceres、CppAD 和 ADOL-C 之间切换。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 在模板函数中用 `std::abs` | Jet 编译失败 | 标准库无 Jet 重载 | `using std::abs; abs(x);` |
| 编程 | 条件分支用 `if (x > threshold)` | Jet 比较语义不同 | Jet 不是标量 | 用 `x.a > threshold` 或软化 |
| 概念 | 认为 Jet 和 double 行为完全相同 | 导数断裂或不连续 | Jet 需要可微的计算链 | 避免 `floor`、`ceil`、`static_cast<int>` |
| 工程 | 日志输出 Jet 对象 | 编译错误或输出混乱 | 日志框架不认识 Jet | 在日志边界取 `.a` 值 |

### 练习

1. 把一个只用 `double` 的二连杆正运动学函数改成 `template <typename Scalar>`，验证 Ceres Jet 能通过。
2. 找出代码中所有 `std::sin`、`std::cos`、`std::sqrt` 调用，改成 ADL 友好写法。
3. 用 Ceres 的 `AutoDiffCostFunction` 和你的模板化正运动学函数，验证自动微分和有限差分的一致性。

---

## 5.15 Eigen 在 GPU 上的使用限制与替代方案 ⭐⭐⭐

这一节解决的问题是：当计算负载需要 GPU 加速时，Eigen 的哪些部分可以迁移，哪些需要替代方案。

### Eigen 的 GPU 支持现状

Eigen 3.3 开始支持在 CUDA 设备代码中使用部分功能。固定大小矩阵的基本运算（加减乘、逐元素操作、小矩阵求解）可以在 `__device__` 函数中工作。但有几个重要限制。

| 功能 | GPU 支持 | 限制 |
|------|----------|------|
| 固定大小矩阵基本运算 | 支持 | 仅基本算术 |
| 动态大小矩阵 | 不支持 | 需要 malloc，GPU 上代价高 |
| 稀疏矩阵 | 不支持 | 应使用 cuSPARSE |
| 分解器（LLT、LU 等） | 不支持 | 应使用 cuSOLVER |
| 表达式模板 | 部分支持 | 复杂表达式可能编译失败 |
| `aligned_allocator` | 不适用 | GPU 有自己的内存管理 |

这意味着在 GPU 上，Eigen 主要适合小型固定大小矩阵的计算。例如在并行仿真中，每个环境实例需要独立计算 3x3 旋转矩阵或 4x4 变换矩阵。这些计算可以用 Eigen 的固定大小类型表达，编译器会把它们映射到 GPU 寄存器。

### 替代方案对比

| 需求 | Eigen（CPU） | GPU 替代 | 适用场景 |
|------|-------------|----------|----------|
| 小矩阵并行计算 | `Matrix3d`、`Matrix4d` | Eigen CUDA 或手写核函数 | 并行仿真 |
| 大规模稠密线性代数 | `MatrixXd` | cuBLAS、cuSOLVER | 批量轨迹优化 |
| 大规模稀疏求解 | `SparseMatrix` + 直接法 | cuSPARSE + cuSOLVER | 大 SLAM 问题 |
| 批量正向运动学 | Pinocchio | 自定义 CUDA 核 | RL 并行环境 |
| 张量运算 | 不适合 | PyTorch/JAX C++ API | 学习算法 |

对机器人学，GPU 加速主要出现在三个场景：大规模并行仿真（如 Isaac Gym/Sim）、批量轨迹优化（如 MPPI）和深度学习推理（如 RL 策略）。前两者可能需要在 GPU 上做小矩阵运算，此时 Eigen 的固定大小类型提供了熟悉的 API。第三者通常使用 PyTorch 或 JAX 的张量运算，不直接依赖 Eigen。

### CPU-GPU 数据传输的 Map 桥接

当 GPU 计算结果需要回传 CPU 进行后续处理时，Map 可以避免中间拷贝：

```cpp
#include <Eigen/Dense>
#include <vector>

void processGpuResults(const double* gpu_output_host_copy,
                       int num_envs, int state_dim) {
    // GPU 结果已经通过 cudaMemcpy 拷贝到 host 端连续数组。
    // 使用 Map 直接访问，不再做额外拷贝。
    for (int i = 0; i < num_envs; ++i) {
        Eigen::Map<const Eigen::VectorXd> state(
            gpu_output_host_copy + i * state_dim, state_dim);

        // 在 CPU 侧用 Eigen 做后处理，例如计算奖励或约束违反。
        double reward = -state.head(3).squaredNorm();
        (void)reward;
    }
}
```

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 在 CUDA 核函数中用 `MatrixXd` | 编译错误或崩溃 | 动态大小在 GPU 上需要 malloc | 只用固定大小类型 |
| 概念 | 认为 Eigen GPU 支持等同于 cuBLAS | 性能差 | Eigen 不做大规模 GPU 优化 | 大矩阵用专用 GPU 库 |
| 工程 | CPU/GPU 频繁来回拷贝小矩阵 | 传输延迟主导 | PCIe 带宽限制 | 批量传输或减少同步点 |
| 思维 | 把所有计算都搬到 GPU | 小矩阵反而慢 | GPU 启动开销大于计算 | 小矩阵留在 CPU |

### 练习

1. 在 CUDA `__device__` 函数中用 `Eigen::Matrix3f` 实现一个旋转矩阵乘法，验证编译和正确性。
2. 对比用 Eigen CPU 和 cuBLAS 计算 1000 个 12x12 矩阵乘法的耗时。
3. 设计一个并行仿真的数据流：GPU 计算批量正向动力学，CPU 用 Map 读取结果做策略更新。

---

## 5.16 Eigen 3.4 新特性概览 ⭐⭐

这一节解决的问题是：Eigen 3.4 引入了哪些与机器人开发相关的改进。

### 主要变化

Eigen 3.4 于 2021 年发布，是继 3.3 之后的重要版本。它引入了若干与机器人 C++ 开发直接相关的特性和改进。

| 特性 | 影响 | 机器人相关度 |
|------|------|-------------|
| C++11 作为最低要求 | 可用 `constexpr`、移动语义 | 中 |
| `Reshaped` 视图 | 无拷贝矩阵重塑 | 高：批量操作 |
| STL 迭代器支持 | `begin()`/`end()` 可用于范围 for | 中 |
| `indexing` 改进 | 类似 NumPy 的高级索引 | 高：子矩阵操作 |
| 改进的 AVX/AVX2/AVX-512 | 更好的 SIMD 代码生成 | 高：数值计算 |
| `Eigen::seq`、`Eigen::seqN` | 编译期和运行时切片 | 高：状态向量段操作 |

### Reshaped 视图

`Reshaped` 允许把矩阵重新解释为不同形状而不拷贝数据。这对机器人中常见的"把 N 个 3D 点从 3xN 矩阵变成 3N 向量"操作很有用。

```cpp
#include <Eigen/Dense>

void reshapedDemo() {
    Eigen::Matrix<double, 3, 4> points;
    points.setRandom();

    // 把 3x4 矩阵重塑为 12x1 向量，无拷贝。
    auto vec = points.reshaped();

    // 也可以指定目标形状。
    auto mat2x6 = points.reshaped(2, 6);

    (void)vec;
    (void)mat2x6;
}
```

### 高级索引

`Eigen::seq` 和 `Eigen::seqN` 提供了类似 NumPy 切片的语法，让状态向量的段操作更清楚。

```cpp
#include <Eigen/Dense>

void advancedIndexingDemo() {
    Eigen::VectorXd state(18);
    state.setLinSpaced(18, 0.0, 17.0);

    // 传统写法：segment 需要知道起始位置和长度。
    auto pos_old = state.segment<3>(0);

    // Eigen 3.4 写法：seq 可以表达起止范围。
    auto pos_new = state(Eigen::seq(0, 2));
    auto vel = state(Eigen::seq(3, 5));

    // seqN 从起始位置取 N 个元素。
    auto orient = state(Eigen::seqN(6, 4));

    (void)pos_old;
    (void)pos_new;
    (void)vel;
    (void)orient;
}
```

注意 `seq(0, 2)` 的含义是索引 0、1、2（闭区间），和 `segment<3>(0)` 等价。使用 `seq` 时仍要注意 `auto` 保存的是表达式视图。

### 与机器人生态的版本兼容

许多机器人开源项目仍在使用 Eigen 3.3。如果项目需要同时支持 3.3 和 3.4，应避免在公共接口中使用 3.4 独有特性（如 `Reshaped`）。一种做法是用宏检测版本号。

```cpp
#include <Eigen/Core>

// Eigen 版本宏从 3.3 开始可用。
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
// 使用 3.4 的 reshaped 特性。
#else
// 使用手动 Map 重塑。
#endif
```

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 用 `auto` 保存 `reshaped()` 结果跨修改 | 视图随原始数据变化 | Reshaped 是视图表达式 | 用具体类型或 `.eval()` |
| 工程 | 公共库接口使用 3.4 特性 | 依赖 3.3 的项目编译失败 | 版本不兼容 | 检查版本宏或限制内部使用 |
| 概念 | 认为 `seq` 索引是半开区间 | 少取或多取一个元素 | Eigen seq 是闭区间 | 对照文档确认语义 |
| 思维 | 为了用新特性升级所有依赖 | 编译和 ABI 冲突 | 大型项目依赖链复杂 | 在内部模块中逐步迁移 |

### 练习

1. 用 `reshaped` 把一个 3xN 点云矩阵变成 3N 向量，与手动 `Map` 重塑对比正确性和性能。
2. 用 `Eigen::seq` 重写一段使用 `segment` 的状态向量操作代码，比较可读性。
3. 检查你使用的 ROS2 发行版默认安装的 Eigen 版本，判断是否可以使用 3.4 特性。

---

## 5.17 表达式模板的性能调优 ⭐⭐⭐

这一节解决的问题是：如何判断一个 Eigen 表达式应该保持延迟还是手动求值，以及如何系统性地优化数值代码性能。

### 何时手动 `.eval()` 提升性能

表达式模板的默认策略是延迟求值和融合。但在三种情况下，手动求值反而更快。

| 情况 | 原因 | 判断方法 |
|------|------|----------|
| 子表达式被重复使用 | 延迟导致重复计算 | 检查同一子表达式出现次数 |
| 表达式深度导致编译爆炸 | 编译器展开过深的模板树 | 编译时间异常增长 |
| 中间结果尺寸远小于最终计算 | 保存中间量减少后续计算量 | 比较维度 |

在 QP 组装中，`J.transpose() * W` 是一个常见子表达式。如果它在后续被乘以 `J` 和添加到 Hessian 中多次使用，保存这个中间量可以避免重复计算 $J^T W$ 的每个元素。

```cpp
#include <Eigen/Dense>

void assembleHessianBad(const Eigen::MatrixXd& J1,
                        const Eigen::MatrixXd& J2,
                        const Eigen::MatrixXd& W,
                        Eigen::MatrixXd* H) {
    // 不好：J1.transpose() * W 在两行中重复出现，可能被重复计算。
    H->noalias() += J1.transpose() * W * J1;
    H->noalias() += J2.transpose() * W * J2;
}

void assembleHessianGood(const Eigen::MatrixXd& J1,
                         const Eigen::MatrixXd& J2,
                         const Eigen::MatrixXd& W,
                         Eigen::MatrixXd* H) {
    // 保存中间量，每个 J^T W 只计算一次。
    const Eigen::MatrixXd J1tW = J1.transpose() * W;
    const Eigen::MatrixXd J2tW = J2.transpose() * W;

    H->noalias() += J1tW * J1;
    H->noalias() += J2tW * J2;
}
```

### 性能测量方法

对 Eigen 代码的性能优化不应靠直觉，而应靠测量。

| 工具 | 测量内容 | 使用场景 |
|------|----------|----------|
| Google Benchmark | 函数级耗时 | 比较不同写法 |
| `EIGEN_RUNTIME_NO_MALLOC` | 是否有堆分配 | 检测实时路径 |
| perf stat | 缓存命中率、分支预测 | 定位硬件瓶颈 |
| 编译器报告 `-Rpass=loop-vectorize` | SIMD 向量化是否成功 | 检查是否被优化 |
| objdump | 生成的机器码 | 确认循环展开 |

一个常见误解是"固定大小矩阵总是更快"。对 3x3 或 4x4 矩阵确实如此，但当固定大小超过一定阈值（通常 20-30），栈上的大数组可能反而造成栈溢出或缓存压力。在这种边界情况下，应通过 benchmark 确定最佳策略。

### 避免表达式深度过大

当 Eigen 表达式嵌套过深时，编译器需要实例化非常复杂的模板类型。这不会影响运行时性能，但会显著增加编译时间，极端情况下可能导致编译器内存耗尽。

```cpp
#include <Eigen/Dense>

Eigen::VectorXd deepExpressionBad(const Eigen::VectorXd& a,
                                   const Eigen::VectorXd& b,
                                   const Eigen::VectorXd& c,
                                   const Eigen::VectorXd& d) {
    // 嵌套很深的表达式。编译器看到的类型是多层模板嵌套。
    return (a.array() * b.array() + c.array()).sqrt().matrix()
         + (d.array().square() + a.array()).matrix();
}

Eigen::VectorXd deepExpressionGood(const Eigen::VectorXd& a,
                                    const Eigen::VectorXd& b,
                                    const Eigen::VectorXd& c,
                                    const Eigen::VectorXd& d) {
    // 把表达式拆成语义清晰的中间步骤。
    Eigen::VectorXd weighted = (a.array() * b.array() + c.array()).sqrt().matrix();
    Eigen::VectorXd penalty = (d.array().square() + a.array()).matrix();
    return weighted + penalty;
}
```

后者不一定更快，但编译更快、调试更容易、中间变量可以被 profiler 和 debugger 观察到。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 重复子表达式不保存 | 计算量翻倍 | Eigen 不做公共子表达式消除 | 手动保存中间量 |
| 编程 | 所有中间量都 `.eval()` | 临时对象过多 | 破坏融合优化 | 只在重复使用或生命周期需要时求值 |
| 工程 | 不测量就猜哪种写法更快 | 优化方向错误 | Eigen 的优化与直觉不同 | 用 benchmark 比较 |
| 思维 | 认为 Eigen 自动做公共子表达式消除 | 重复计算 | 表达式模板只做延迟，不做 CSE | 手动管理共享子表达式 |

### 练习

1. 对一个包含 `J.transpose() * W` 的 Hessian 组装函数，分别测试保存和不保存中间量的耗时差异。
2. 用 `EIGEN_RUNTIME_NO_MALLOC` 检测一段控制循环代码，找出第一处意外分配并消除。
3. 编写一个深度嵌套的 Eigen 表达式，测量编译时间，然后拆分成中间变量后对比编译时间变化。

---

## 本章小结

| 知识点 | 关键结论 | 工程动作 |
|--------|----------|----------|
| 表达式模板 | 表达式和值不同 | 谨慎使用 `auto`、`.eval()`、`.noalias()` |
| Map/Ref | 视图不拥有内存 | 明确生命周期和 stride |
| 对齐/SIMD | 性能和崩溃都受影响 | 统一编译选项和 allocator |
| Scalar 模板 | 自动微分依赖泛型标量 | 避免写死 double |
| 稀疏矩阵 | 结构和数值分离 | 初始化建结构，循环更新值 |
| 实时审计 | Eigen 可能隐式分配 | 启用 malloc 检测 |
| 内存布局 | 布局是模块边界契约 | 明确 RowMajor、ColumnMajor 和 stride |
| 机器人矩阵坑 | 物理约束不会由 Matrix 类型自动保证 | 对 SO(3)、坐标系、求解器做显式防御 |
| 接口设计 | 签名决定所有权、尺寸和分配 | 用 `Ref`、模板和预分配输出表达真实约束 |
| Map/Ref 深入 | 求解器和 SLAM 中零拷贝的生命周期管理 | Map 不延长内存生命期 |
| 自定义标量 | Ceres Jet 需要 ADL 和 Scalar 构造 | 模板化数学函数 |
| GPU 限制 | Eigen GPU 只支持固定大小基本运算 | 大矩阵用 cuBLAS/cuSOLVER |
| Eigen 3.4 | Reshaped、seq、改进的 SIMD | 检查版本兼容性 |
| 性能调优 | 公共子表达式需手动保存 | benchmark 驱动优化 |

## 累积项目：Eigen 数值内核

本章新增模块是 `eigen_numeric_core`。

阶段 1：实现固定大小姿态误差和雅可比块操作。
阶段 2：实现 `Map`/`Ref` 输入接口，接入模拟 ROS 关节数组。
阶段 3：实现模板化 `Scalar` 的二轮车和 SE3 小模型。
阶段 4：实现稀疏 Hessian 构造，并缓存非零结构。
阶段 5：在实时循环中开启 malloc 检测，保证更新路径无分配。

## 延伸阅读

| 资料 | 难度 | 阅读目的 |
|------|------|----------|
| Eigen 官方文档 Expression Templates | ⭐⭐ | 理解延迟求值 |
| Eigen Map 与 Ref 文档 | ⭐⭐ | 设计零拷贝接口 |
| Eigen Alignment 文档 | ⭐⭐⭐ | 处理容器和 SIMD |
| Pinocchio 模板化源码 | ⭐⭐⭐ | 学习 Scalar 泛型设计 |
| SuiteSparse/CHOLMOD 资料 | ⭐⭐⭐ | 了解大型稀疏求解 |

## 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 处理 |
|------|----------|----------|------|
| 结果随输入修改而变化 | `auto` 保存表达式 | 打印类型或强制 `.eval()` | 用具体 Matrix 类型 |
| 矩阵乘法结果错误 | 错用 `.noalias()` | 检查左右是否共享存储 | 去掉 noalias 或用 eval |
| 偶发崩溃 | 对齐不满足 | ASan 和地址检查 | aligned allocator 或宏 |
| 实时循环断言 malloc | 动态 resize 或临时 | 开启 Eigen malloc 检测 | 预分配和固定尺寸 |
| 点云字段错位 | stride 或 offset 错 | 检查消息布局 | 用正确 Map stride |
| 稀疏求解很慢 | 每周期重建结构 | 统计 Triplet 构造次数 | 缓存 sparsity |
