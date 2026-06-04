# Eigen 基础与 SLAM 数学预备

> **难度**：⭐⭐⭐～⭐⭐⭐⭐ | **建议用时**：2 周 | **前置要求**：类型系统与值类别推导-Lambda与STL算法 的 C++ 基础、线性代数、刚体运动基础

---

## 本章目标

学完本章后，你应该能做到：

1. 把 `Matrix<Scalar, Rows, Cols>` 读成完整的编译期接口，而不是只读成“一个矩阵”。
2. 从对象模型角度解释固定大小、动态大小、行列主序、堆分配和对齐边界。
3. 用 `RowsAtCompileTime`、`ColsAtCompileTime`、`static_assert` 和运行时断言区分编译期错误与运行期错误。
4. 正确使用 `block()`、`head()`、`segment()` 切分 SLAM 状态、协方差和扰动量，并把 offset 维护成单一事实源。
5. 使用 `Eigen::Map`、`Eigen::Ref` 和 `Stride` 在 Ceres 参数块、原始数组、AoS/SoA 点云布局和 Eigen 类型之间零拷贝转换。
6. 从 SO(3)/SE(3) 不变量出发，处理旋转矩阵、单位四元数、Rodrigues 公式、指数映射、小角度扰动和 `Isometry3d`。
7. 解释 Eigen 表达式模板的延迟求值机制，避免 `auto`、aliasing、`.eval()`、`noalias()` 和 `Ref` 相关的真实工程错误。
8. 为 ICP、ESKF、图优化和点云处理选择合适的线性求解器，并用残差、奇异值和条件数诊断退化。

---

## 前置自测

答不出两题以上，建议先回到 类型系统与值类别推导-Lambda与STL算法 复习类型、引用、对象生命周期、模板基础和线性代数。Eigen 的难点不是 API 数量，而是它把 C++ 对象模型和数值数学放在同一层接口里：一个矩阵对象既有类型、生命周期、存储所有权，也有维度、坐标系语义和数值稳定性约束。

1. `Eigen::Matrix<double, 3, 1>` 与 `Eigen::Matrix<double, Eigen::Dynamic, 1>` 在类型、对象大小和数据缓冲区分配上有什么区别？
2. `auto block = x.segment<3>(6);` 中的 `block` 是值还是视图？如果 `x` 先析构，`block` 还能使用吗？
3. Ceres 残差函数里的 `const T* pose` 为什么不能直接写成 `Eigen::Vector3d`？`T` 可能是什么类型？
4. 四元数 `q` 和 `-q` 表示的旋转是否相同？这会如何影响优化残差和单元测试？
5. 为什么公式里的 `x = A^{-1} b` 不应该机械翻译成 `A.inverse() * b`？
6. 如果一个函数参数写成 `const Eigen::Matrix3d&`，它能不能接收 `A.block<3,3>(0,0)` 或 `R1 * R2` 这样的表达式？如果不能，应该考虑什么接口？

---

## 本章在课程中的位置

前面十章解决的是 C++ 程序的基本对象：值如何构造，引用如何绑定，资源如何释放，函数对象如何传递，异常如何改变控制流，STL 容器如何拥有元素。

从本章开始，同样的对象模型会被放进机器人数值计算。`Vector3d` 不是“比 `double[3]` 好看的写法”，它是一个拥有构造、析构、拷贝、引用、对齐、表达式求值和维度契约的 C++ 对象。它又同时代表三维向量、状态切片、雅可比列、协方差块或李代数扰动。一个 Eigen 对象只要跨过函数边界，就同时携带三类问题：

1. **C++ 问题**：谁拥有内存，表达式引用了谁，临时对象活多久，是否发生拷贝。
2. **数学问题**：维度是否匹配，旋转是否仍在 SO(3)，协方差是否保持对称半正定。
3. **工程问题**：数据来自 Ceres、PCL、ROS message 还是自有结构，行列主序和 stride 是否一致。

Eigen 是这个转折点，因为它把这些问题合并到同一行代码里。

机器人项目中的大部分核心计算都不会直接写成裸数组。

它们会写成：

```cpp
Eigen::Vector3d position;
Eigen::Matrix3d rotation;
Eigen::Quaterniond q;
Eigen::Isometry3d T_world_body;
Eigen::Matrix<double, 15, 15> covariance;
```

这不是为了“看起来数学化”。真正原因是：机器人算法中的很多 bug 本质上不是语法错误，而是接口契约没有被表达出来。裸数组只告诉编译器“这里有若干个 `double`”，却不告诉它这是三维点、四元数系数、15 维误差状态还是一个协方差块。Eigen 至少把一部分约束推进类型和表达式系统：

1. 矩阵维度本身就是接口契约。
2. 编译器可以利用固定维度优化小矩阵计算。
3. 表达式模板可以减少不必要的临时对象。
4. 与 Ceres、GTSAM、PCL、Sophus、Pinocchio 等库的接口能自然衔接。

但 Eigen 也有代价。它把模板、内存布局、对齐、惰性求值和线性代数数值稳定性同时带进代码。

如果只会写 `MatrixXd A; A.inverse();`，很容易写出能编译但数值很差、性能很差甚至生命周期错误的代码。

> **本质洞察**：Eigen 不是“矩阵语法糖”。它是把线性代数对象、编译期尺寸、内存布局、视图生命周期和数值求解策略绑定在一起的 C++ 数学基础设施。学 Eigen 的重点不是记 API，而是学会把数学结构翻译成可靠的类型、内存视图和计算路径。

---

## 知识树

```text
Eigen 基础与 SLAM 数学预备
├── 为什么使用 Eigen（11.1）⭐⭐⭐
│   └── 裸数组的局限 → Eigen 的类型接口 → 机器人库生态
├── Matrix/Vector 类型系统（11.2）⭐⭐⭐⭐
│   ├── 编译期维度 = 类型身份
│   ├── 固定大小 vs 动态大小 ← 编译器优化可见性
│   ├── 对齐要求 ← SIMD 指令硬件约束
│   └── 初始化与访问
├── Block/Segment 状态切片（11.3）⭐⭐⭐⭐
│   ├── ESKF 状态结构化访问
│   ├── 偏移量单一事实源
│   └── block 表达式生命周期
├── Eigen::Map 零拷贝视图（11.4）⭐⭐⭐⭐
│   ├── Ceres 参数块映射
│   ├── Map vs Ref vs owned object
│   └── Stride 与非连续内存
├── 旋转与变换（11.5）⭐⭐⭐⭐
│   ├── 四元数顺序陷阱 ← 构造 vs 存储
│   ├── Rodrigues/指数映射 ← SO(3) 李代数
│   ├── 双覆盖与残差连续性
│   └── Isometry3d vs Matrix4d
├── 表达式模板（11.6）⭐⭐⭐⭐
│   ├── lazy evaluation 机制
│   ├── auto 陷阱 ← 表达式 ≠ 值
│   ├── aliasing 与 noalias()
│   └── .eval() 决策规则
├── 内存布局与容器（11.7）⭐⭐⭐
│   └── 列主序/行主序 → 对齐 → STL 容器
├── 线性求解（11.8）⭐⭐⭐⭐
│   ├── 分解 vs 求逆 ← 数值稳定性
│   ├── 正规方程推导与条件数
│   └── SVD-ICP 与退化诊断
└── 工程案例（11.9-11.10）⭐⭐⭐⭐
    ├── Ceres 参数块 → ESKF 状态更新
    └── Mini Eigen SLAM Toolkit
```

上面是本章的知识树。树根是"机器人数值计算为什么需要 Eigen"，树干是 Eigen 的类型系统（维度、对齐、表达式模板），树枝是旋转表示、线性求解和工程应用。每个节点之间用箭头标注了因果或依赖关系。读完本章后，你脑中应该有这棵完整的树，而不是一堆散落的 API 知识点。

---

## 11.1 为什么机器人代码大量使用 Eigen ⭐⭐⭐

### 动机：裸数组无法表达数学意图

假设你要存一个三维位置。

可以写：

```cpp
double p[3] = {1.0, 2.0, 3.0};
```

这段代码能工作。

但它没有说明三件事：

| 问题 | 裸数组表达能力 |
|------|----------------|
| 这是列向量还是三个独立标量 | 看不出来 |
| 它的单位和坐标系是什么 | 看不出来 |
| 它能不能参与矩阵乘法 | 需要手写函数 |

Eigen 写法：

```cpp
Eigen::Vector3d p_world;
```

至少告诉读者：

1. 这是一个三维向量。
2. 它可以参与向量加减、点乘、叉乘和矩阵乘法。
3. 类型名可以带坐标系语义，例如 `p_world`。

类型不能替代文档。

但类型能把一部分数学结构放进编译器能检查的地方。

### 反面：用 `std::vector<double>` 表示状态向量

ESKF 常见状态包含：

```text
p     位置      3
v     速度      3
theta 姿态误差  3
bg    陀螺零偏  3
ba    加计零偏  3
```

总维度是 15。

如果写成：

```cpp
std::vector<double> x(15);
```

你可以访问 `x[0]` 到 `x[14]`。

但 `x.segment<3>(6)` 这种数学切片不存在。

你只能手写索引：

```cpp
Eigen::Vector3d theta{x[6], x[7], x[8]};
```

这种写法的风险是：

1. 索引容易错。
2. 维度约定散落在代码里。
3. 协方差块更新很难读。
4. 后续改状态维度时容易漏改。

Eigen 写法：

```cpp
using ErrorState = Eigen::Matrix<double, 15, 1>;
using Covariance = Eigen::Matrix<double, 15, 15>;

ErrorState dx = ErrorState::Zero();
Eigen::Vector3d dtheta = dx.segment<3>(6);
```

这段代码仍然需要你维护偏移。

但切片维度已经进入类型系统。

`segment<3>(6)` 比 `x[6]、x[7]、x[8]` 更接近数学表达。

### Eigen 在机器人库中的位置

| 库或项目 | Eigen 的典型角色 |
|----------|------------------|
| Ceres | 参数块映射、残差计算、雅可比矩阵 |
| GTSAM | 因子误差、线性化、李群局部坐标 |
| g2o | 顶点估计、边误差、Hessian 组装 |
| PCL | 点坐标、协方差、SVD、PCA |
| FAST-LIO2 | ESKF 状态、协方差、IMU 预积分 |
| Sophus | 李群元素的矩阵和向量表示 |
| Pinocchio | 空间向量、惯量、雅可比、动力学矩阵 |

因此，本章不是孤立的工具介绍。

它会贯穿后续模板、SLAM、优化、控制和动力学章节。

### 什么时候不用 Eigen

Eigen 不是所有数值数据的默认答案。

| 场景 | 更合适的选择 | 原因 |
|------|--------------|------|
| 大量同构传感器原始字节 | `std::span<std::byte>` / buffer | 先解析格式，不急着矩阵化 |
| GPU 张量训练 | CUDA / TensorRT / LibTorch | Eigen 主要面向 CPU |
| 超大稀疏图优化 | Eigen Sparse / SuiteSparse / GTSAM 封装 | 需要稀疏结构 |
| 网络消息字段 | ROS message 类型 | 需要序列化契约 |
| 固定小矩阵热路径 | Eigen 固定大小类型 | 正适合 Eigen |

判断标准很直接：

```text
如果数据天然是向量、矩阵、变换、协方差、雅可比，Eigen 合适。
如果数据首先是字节流、消息、纹理、张量批处理，先保留原生结构。
```

### 常见陷阱

> ⚠️ **抽象陷阱：把所有数组都换成 `MatrixXd`**
>
> `MatrixXd` 只表达“动态大小矩阵”，不表达语义。三维位置应优先用 `Vector3d`，四元数应优先用 `Quaterniond`，刚体变换应优先用 `Isometry3d` 或李群类型。

> ⚠️ **数学陷阱：把矩阵类型当作坐标系说明**
>
> `Vector3d` 只能说明三维向量，不能说明它在 `world`、`body` 还是 `camera` 坐标系下。坐标系仍要靠变量名、类型封装和接口文档表达。

> ⚠️ **性能陷阱：不区分小矩阵和大矩阵**
>
> `Matrix3d` 和 `MatrixXd` 的性能模型不同。小固定矩阵适合栈上分配和编译期优化，大动态矩阵适合尺寸运行期变化的场景。

### 练习

1. **命名题**：把 `Eigen::Vector3d p` 改成能表达坐标系和语义的变量名，至少给出三个例子。
2. **重构题**：把一个 `std::vector<double>` 的 15 维 ESKF 状态改成 `Matrix<double, 15, 1>`，列出每个 block 的偏移。
3. **讨论题**：为什么 ROS message 中的 `geometry_msgs::Point` 不应直接替代算法内部的 `Eigen::Vector3d`？

---

## 11.2 Matrix 与 Vector：维度是类型的一部分 ⭐⭐⭐⭐

### `Matrix<Scalar, Rows, Cols>` 的本质

Eigen 最核心的类型模板是：

```cpp
template <typename Scalar, int Rows, int Cols>
class Matrix;
```

常用别名只是它的简写：

```cpp
using Vector3d = Matrix<double, 3, 1>;
using Matrix3d = Matrix<double, 3, 3>;
using Matrix4d = Matrix<double, 4, 4>;
using VectorXd = Matrix<double, Dynamic, 1>;
using MatrixXd = Matrix<double, Dynamic, Dynamic>;
```

这里的 `Rows` 和 `Cols` 是非类型模板参数。

它们不是对象里的普通字段。

它们在编译期就确定。

这带来两个结果：

1. 编译器能知道矩阵大小。
2. 不同维度的矩阵是不同类型。

例如：

```cpp
Eigen::Matrix<double, 3, 1> a;
Eigen::Matrix<double, 4, 1> b;
```

`a` 和 `b` 不是同一种类型。

这正是 Eigen 能在编译期检查很多维度错误的原因。

从 C++ 类型系统看，`Rows` 和 `Cols` 是模板参数，属于类型身份的一部分。`Matrix<double,3,1>` 与 `Matrix<double,4,1>` 的关系不像“同一种类对象但长度不同”，而更像 `std::array<double,3>` 与 `std::array<double,4>`：它们都存 `double`，但接口契约不同，函数重载、模板匹配和编译期断言都会把它们看成不同类型。

这件事在机器人代码中非常关键。假设一个函数接收 IMU 误差状态：

```cpp
using ImuError = Eigen::Matrix<double, 15, 1>;

void applyCorrection(const ImuError& dx);
```

调用者不能把 `VectorXd`、`Vector6d` 或 `Matrix<double, 18, 1>` 随手塞进来。编译器会在函数边界阻止维度错误。反过来，如果接口写成：

```cpp
void applyCorrection(const Eigen::VectorXd& dx);
```

函数体内部就必须自己检查：

```cpp
if (dx.size() != 15) {
    throw std::invalid_argument("ESKF error state must have dimension 15");
}
```

这不是“固定大小一定优雅、动态大小一定糟糕”。真正的抽象不变量是：**如果数学维度属于算法定义，就让它进入类型；如果维度属于输入数据规模，就让它留在运行期对象状态里。**

### 编译期维度查询：`RowsAtCompileTime`

Eigen 类型会暴露编译期尺寸常量：

```cpp
template <typename Derived>
void requireColumnVector(const Eigen::MatrixBase<Derived>& x) {
    static_assert(Derived::ColsAtCompileTime == 1 ||
                  Derived::ColsAtCompileTime == Eigen::Dynamic,
                  "x must be a column vector");
}
```

常见常量包括：

| 常量 | 含义 |
|------|------|
| `RowsAtCompileTime` | 编译期行数，动态则为 `Eigen::Dynamic` |
| `ColsAtCompileTime` | 编译期列数，动态则为 `Eigen::Dynamic` |
| `SizeAtCompileTime` | 编译期元素数量，任一维动态则通常为 `Eigen::Dynamic` |
| `MaxRowsAtCompileTime` | 最大行数，适合部分动态但有上界的类型 |
| `MaxColsAtCompileTime` | 最大列数 |

这让模板函数可以同时接收固定维度和动态维度，但仍在能检查的地方尽早检查：

```cpp
template <typename Derived>
double squaredNorm3(const Eigen::MatrixBase<Derived>& v) {
    static_assert(Derived::ColsAtCompileTime == 1 ||
                  Derived::ColsAtCompileTime == Eigen::Dynamic,
                  "expected a column vector");
    static_assert(Derived::RowsAtCompileTime == 3 ||
                  Derived::RowsAtCompileTime == Eigen::Dynamic,
                  "expected a 3D vector");

    if (v.rows() != 3 || v.cols() != 1) {
        throw std::invalid_argument("expected runtime shape 3x1");
    }
    return v.squaredNorm();
}
```

这里有两道门：

1. 如果调用者传 `Matrix<double,4,1>`，编译期就失败。
2. 如果调用者传 `VectorXd`，编译期只能知道它是动态列向量，运行期还要检查 `rows() == 3`。

这就是固定维度和动态维度混合接口的基本模式。

### 固定大小与动态大小：从编译器优化能力推导选型

固定大小和动态大小不是"两种风格"或"两种偏好"——它们的区别根植于**编译器能优化什么、不能优化什么**。理解这个区别，才能在写 SLAM 和控制代码时做出正确的选型，而不是凭感觉。

**编译器优化的核心约束：只有编译期已知的信息才能参与编译期优化。** C++ 编译器的优化器在编译期工作——它能内联函数、展开循环、分配寄存器、选择 SIMD 指令。但所有这些优化都需要一个前提条件：编译器必须在编译期知道相关信息。如果矩阵的尺寸在编译期已知（如 `Matrix3d` 的 3x3），编译器可以：

1. **完全展开循环**：`Matrix3d` 的乘法是 27 次乘法 + 18 次加法，编译器可以把循环完全展开成 45 条独立指令，消除循环计数器和条件跳转的开销。
2. **使用 SIMD 指令**：知道数据恰好是 2 个或 4 个 `double`，编译器可以选择 SSE/AVX 指令一次处理一整组数据。`Vector4d` 的 4 个 double 正好装入一个 AVX-256 寄存器。
3. **寄存器分配**：小矩阵的所有元素可能常驻 CPU 寄存器，完全不经过内存读写。
4. **内联展开**：`Matrix3d` 的 `operator*` 可以被完全内联，消除函数调用开销。

如果矩阵尺寸在运行期才知道（如 `MatrixXd`），编译器必须生成通用代码——循环次数由运行期变量控制，不能展开；数据大小未知，不能选定 SIMD 寄存器宽度；缓冲区大小未知，必须在堆上分配。编译器不是"不够聪明"，而是**逻辑上不可能**对未知尺寸做特定优化。

**如果不区分固定大小和动态大小会怎样？** 假设 Eigen 只提供 `MatrixXd`——所有矩阵都在运行期确定尺寸。在 SLAM 的 IMU 传播中，每次循环需要计算 `Matrix3d` 旋转矩阵乘法、`Vector3d` 加法和 `Matrix<double,15,15>` 协方差更新。如果全部使用动态大小，每个 `3x3` 矩阵运算都要经过通用循环代码、堆分配和间接指针访问。实测表明，对 3x3 矩阵乘法，Eigen 的固定大小模板特化比动态大小快 5-10 倍——这个差距在 1kHz 控制循环中是决定性的。

> **本质洞察**：固定大小 vs 动态大小的本质不是"性能偏好"，而是**信息对编译器的可见性**。`Rows` 和 `Cols` 作为模板参数写进类型定义时，编译器在编译期就能看到这些数字，从而把它们当作常量参与循环展开、SIMD 选择和寄存器分配。`rows()` 和 `cols()` 作为运行期成员函数返回值时，编译器只能生成对任意尺寸都正确的通用代码。选择固定大小还是动态大小，取决于"这个维度对编译器是否可见"。

固定大小：

```cpp
Eigen::Matrix<double, 3, 3> R;   // 编译期已知 3x3，编译器可展开和向量化
Eigen::Matrix<double, 15, 15> P; // 编译期已知 15x15，数据内嵌于对象
```

动态大小：

```cpp
Eigen::MatrixXd H(n, n);  // 运行期才知道 n，必须堆分配，编译器生成通用代码
Eigen::VectorXd b(n);     // 运行期才知道 n，必须堆分配
```

对比：

| 类型 | 尺寸确定时间 | 常见分配方式 | 编译器能做的优化 | 适合场景 |
|------|--------------|--------------|-----------------|----------|
| `Matrix3d` | 编译期 | 对象内部存储 | 循环展开、SIMD、寄存器分配 | 旋转矩阵、小雅可比 |
| `Matrix<double, 15, 15>` | 编译期 | 对象内部存储 | 循环展开（部分）、编译期维度检查 | ESKF 协方差 |
| `VectorXd` | 运行期 | 堆上缓冲区 | 通用 BLAS 路径、运行期维度检查 | 优化变量维度变化 |
| `MatrixXd` | 运行期 | 堆上缓冲区 | 通用 BLAS 路径、大矩阵分块优化 | Hessian、批量点矩阵 |

固定大小并不等于一定在栈上。

如果一个类对象本身在堆上，它的固定大小 Eigen 成员也随对象在堆上。

更准确的说法是：固定大小 Eigen 对象通常把数据作为对象的一部分保存，不需要单独为数据缓冲区分配内存。

可以把两类对象粗略理解成：

```text
Matrix3d 对象:
  [ 9 个 double 直接作为对象数据成员的一部分 ]

MatrixXd 对象:
  [ rows, cols, pointer, capacity 等元数据 ] -> [ 堆上的 double 缓冲区 ]
```

这种对象模型直接影响实时路径。IMU 传播中每次循环都构造 `Matrix3d`、`Vector3d`、`Matrix<double,15,15>`，通常不会为每个对象单独向堆申请内存。相反，如果在 400Hz 路径里反复构造不同大小的 `MatrixXd`，就可能触发堆分配、缓存不友好和不可预测延迟。

工程边界也要讲清楚：固定大小对象并不保证”绝对无堆分配”。如果它被放进 `std::vector`，容器增长仍然会分配；如果它作为 `std::unique_ptr` 指向的对象成员，它随外层对象在堆上；如果固定矩阵非常大，放在栈上还可能造成栈压力。Eigen 的固定维度优势主要是：数据缓冲区尺寸是类型的一部分，单个对象不需要再维护运行期形状和独立动态缓冲区。

### 栈分配 vs 堆分配：在 SLAM 实时循环中的实际意义

固定大小和动态大小矩阵的性能差异，不是”理论上快一点”的学术问题，而是在 SLAM 和控制系统的实时路径中能否满足时间约束的关键因素。

**栈分配的代价模型**：当你声明 `Matrix3d R;` 时，编译器在函数入口处把栈指针减少 72 字节（9 个 double * 8 字节）。这不是一条”分配指令”——它只是修改了一个寄存器的值，代价和任何一条算术指令一样，通常 1 个时钟周期。更重要的是，栈内存几乎必然在 CPU 的 L1 cache 中，因为栈是程序最频繁访问的内存区域。对于 `Matrix3d` 这种 72 字节的数据结构，它完全可以驻留在一两条 cache line（通常 64 字节）中。

**堆分配的代价模型**：当你声明 `MatrixXd A(3, 3);` 时，Eigen 内部需要调用 `new double[9]`。这触发了 C++ 运行时的内存分配器（通常是 `malloc`），分配器需要：(1) 在空闲链表中查找足够大的内存块；(2) 可能需要调用操作系统 `mmap` 或 `sbrk` 获取新页面；(3) 更新分配器的元数据。即使使用高效的现代分配器（如 jemalloc 或 tcmalloc），一次小对象堆分配也需要 20-100 纳秒。析构时还需要一次 `free`，代价类似。

**在 1kHz 控制循环中的累积效应**：SLAM 中的 IMU 传播、ESKF 预测步骤或全身控制器的动力学计算通常以 400Hz-1000Hz 运行。假设每次循环中有 20 个小矩阵运算（旋转矩阵相乘、雅可比计算、协方差更新等）：

```text
固定大小方案：
  20 个 Matrix3d，全部在栈上
  每次循环内存分配代价：~0 ns
  每次循环总代价中的分配占比：0%

动态大小方案：
  20 个 MatrixXd(3,3)，全部在堆上
  每次循环内存分配代价：20 * (50ns 分配 + 50ns 释放) = 2000 ns = 2 μs
  1kHz 循环周期 1000 μs，分配占比：0.2%
```

0.2% 看起来不多，但这只是分配本身的代价。堆分配还有一个更隐蔽的性能影响：**cache 污染**。堆上的小对象可能分散在不同的内存页面中，访问它们会把 L1/L2 cache 中其他热数据驱逐出去。在一个紧密的数值循环中，cache miss 的代价是 100-300 个时钟周期，远超分配本身。

**编译器优化的差异**：固定大小矩阵还有一个动态大小矩阵无法获得的优势——编译器知道矩阵的确切尺寸，可以做更激进的优化：

- `Matrix3d` 的乘法可以完全展开为 27 次乘法和 18 次加法，全部使用 SIMD 指令（SSE/AVX）
- `Matrix<double, 4, 1>` 正好 32 字节，可以放入一个 AVX-256 寄存器
- 循环展开、指令调度和寄存器分配都可以在编译期完成

`MatrixXd` 的乘法必须生成通用循环代码，因为编译器不知道运行时的实际尺寸。即使 BLAS 库对大矩阵有极高效的分块实现，对 3x3 这种小矩阵，Eigen 的固定大小模板特化通常比通用 BLAS 路径更快。

**决策规则总结**：

| 场景 | 推荐 | 原因 |
|------|------|------|
| 数学维度在编译期已知（R, q, 15维状态） | 固定大小 | 零分配代价、编译器可优化 |
| 维度取决于地图大小、观测数量 | 动态大小 | 维度是运行期数据 |
| 热路径（1kHz 控制循环、每帧 ICP） | 尽可能固定大小 | 性能敏感 |
| 初始化路径、配置加载 | 动态大小可接受 | 不是性能瓶颈 |
| 维度 > 16 且在栈上 | 注意栈压力 | `Matrix<double, 100, 100>` 在栈上占 80KB |

### Eigen 的对齐要求：SSE/AVX 指令集为什么需要内存对齐

内存对齐是 Eigen 工程实践中最容易让初学者困惑的问题。它不是 C++ 语言层面的概念，而是 CPU 硬件架构对数据布局的物理约束。理解对齐问题的关键在于理解 SIMD 指令的工作方式——这些指令是 Eigen 高性能的基础，但它们对数据的存放位置有严格要求。

使用 Eigen 时，你可能遇到过 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 宏，或者看到 Eigen 的文档警告”固定大小向量化类型必须对齐”。这个要求不是 Eigen 的任性设计，而是 CPU 硬件的直接约束。

**什么是内存对齐？** 一个 N 字节的数据块如果存放在地址 A 处，当 A 是 N 的倍数时，就说这个数据块是”N 字节对齐”的。例如一个 16 字节的数据块存放在地址 0x100（256 的倍数）就是 16 字节对齐的，存放在地址 0x104 就不是。

**为什么 SIMD 指令需要对齐？** 现代 CPU 的 SSE（Streaming SIMD Extensions）指令可以一次处理 128 位（16 字节）的数据——例如同时加 2 个 `double` 或 4 个 `float`。AVX 指令可以一次处理 256 位（32 字节），AVX-512 可以处理 512 位（64 字节）。这些 SIMD 指令中，部分要求操作数在内存中按 16/32/64 字节对齐：

- `_mm_load_pd`（SSE2）：加载 2 个 double，要求 16 字节对齐
- `_mm256_load_pd`（AVX）：加载 4 个 double，要求 32 字节对齐

如果数据未对齐，有两种可能的后果：

1. **硬件异常**：某些旧指令（如 SSE 的 `movaps`）在数据未对齐时直接触发段错误
2. **性能惩罚**：现代 CPU 上的非对齐访问指令（如 `movups`）可以工作，但可能跨 cache line 边界，导致额外的内存周期

**Eigen 如何处理对齐？** `Vector2d`（2 个 double = 16 字节）和 `Vector4f`（4 个 float = 16 字节）是 Eigen 默认启用 SSE 向量化的类型。Eigen 要求这些类型的实例按 16 字节对齐。对于栈上的局部变量，编译器通常自动处理对齐（通过 `alignas` 或编译器特性）。但对于堆上分配的对象（`new`），C++ 默认的 `operator new` 在 C++17 之前只保证 `alignof(std::max_align_t)` 的对齐（通常 8 或 16 字节），这可能不满足 AVX 的 32 字节要求。

`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 宏为包含 Eigen 固定大小成员的类重载 `operator new`，确保堆分配返回正确对齐的内存。在 C++17 及更高版本中，如果类型的 `alignof` 超过 `__STDCPP_DEFAULT_NEW_ALIGNMENT__`，编译器会自动调用对齐版本的 `operator new`，因此这个宏在新标准下逐渐不再必需。

**实际影响场景**：最常触发对齐问题的场景是把包含 Eigen 固定大小成员的结构体放入 STL 容器：

```text
struct Pose {
    Eigen::Quaterniond q;   // 4 个 double = 32 字节
    Eigen::Vector3d p;      // 3 个 double = 24 字节
};

std::vector<Pose> poses;    // 内部元素可能未按 32 字节对齐
```

Eigen 3.4+ 和 C++17 配合使用时，大部分对齐问题已被自动处理。但在 C++14 项目或交叉编译到嵌入式平台时，对齐仍然是一个需要注意的工程细节。

### 编译失败与运行断言的边界

固定维度错误通常会在编译期暴露：

```cpp
Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
Eigen::Vector4d q = Eigen::Vector4d::Ones();
auto y = R * q;  // 编译期维度不匹配
```

动态维度错误只能在运行期暴露：

```cpp
Eigen::MatrixXd A(3, 3);
Eigen::VectorXd x(4);
auto y = A * x;  // 调试构建通常触发 Eigen 断言
```

很多项目在 Release 构建中会关闭断言，所以动态维度错误有时会变成更晚、更难定位的数值异常。对公共算法接口，一种稳妥写法是“固定数学结构用固定类型，外部输入规模用动态类型，动态入口处显式检查尺寸”：

```cpp
Eigen::Vector3d projectPoint(const Eigen::Matrix3d& K,
                             const Eigen::Vector3d& p_camera);

Eigen::VectorXd solveNormalEquation(const Eigen::MatrixXd& H,
                                    const Eigen::VectorXd& g) {
    if (H.rows() != H.cols() || H.rows() != g.rows()) {
        throw std::invalid_argument("normal equation dimension mismatch");
    }
    return H.ldlt().solve(-g);
}
```

### 机器人中的选型

| 数据 | 推荐类型 | 理由 |
|------|----------|------|
| 三维点 | `Vector3d` | 维度固定 |
| 旋转矩阵 | `Matrix3d` | 3x3 固定 |
| 齐次变换矩阵 | `Matrix4d` 或 `Isometry3d` | 4x4 固定，但语义不同 |
| ESKF 误差状态 | `Matrix<double, 15, 1>` | 维度固定且需要 block |
| ESKF 协方差 | `Matrix<double, 15, 15>` | 维度固定 |
| BA Hessian | `SparseMatrix<double>` | 大规模稀疏 |
| 小规模最小二乘 | `MatrixXd` | 变量数量运行期确定 |
| 点云批量矩阵 | `Matrix<double, 3, Dynamic>` | 列数随点数变化 |

### 初始化方式

Eigen 对象默认构造时通常不会自动清零——这和 Python 的 `numpy.zeros()` 完全不同。C++ 的 POD 类型在栈上分配时不自动初始化（回顾 现代类设计与特殊成员函数 中对象生命周期的讨论），Eigen 的固定大小矩阵遵循相同规则。未初始化的矩阵元素是栈上残留的垃圾值，可能是极小的数（如 `1e-308`）也可能是极大的数（如 `1e+15`），程序不会崩溃但结果完全不可信——这类 bug 在数值计算中非常隐蔽。

```cpp
// 错误：默认构造不是零初始化
Eigen::Vector3d p;
std::cout << p.transpose() << "\n";  // 输出不确定的垃圾值
```

正确的做法是在声明时显式初始化。Eigen 提供了多种初始化方式，适用于不同场景：

```cpp
// 静态工厂方法：语义清晰，推荐用于状态初始化
Eigen::Vector3d p = Eigen::Vector3d::Zero();       // 全零向量
Eigen::Matrix3d R = Eigen::Matrix3d::Identity();    // 单位矩阵
Eigen::Vector3d gravity(0.0, 0.0, -9.81);           // 构造函数直接赋值
```

对于需要逐元素指定的小矩阵，Eigen 提供了逗号初始化器（comma initializer）。它按行主序填充元素——先填第一行从左到右，再填第二行，依此类推。这种语法让代码中的矩阵布局和纸面上的矩阵写法完全对应：

```cpp
Eigen::Vector3d a(1.0, 2.0, 3.0);  // 向量用构造函数

// 矩阵用逗号初始化器——按行填充
Eigen::Matrix3d K;
K << 500.0, 0.0, 320.0,   // 第一行：fx, 0, cx
     0.0, 500.0, 240.0,   // 第二行：0, fy, cy
     0.0, 0.0, 1.0;       // 第三行：0, 0, 1
```

逗号初始化器适合小矩阵——它让代码中矩阵的视觉布局和数学公式保持一致。大矩阵更适合用循环、`block` 赋值或从文件加载，因为手写几百个逗号分隔的数字既容易出错也难以维护。

### 访问元素

Eigen 使用圆括号 `()` 而不是方括号 `[]` 访问矩阵元素——这和标准库容器的 `operator[]` 不同。原因是矩阵需要二维索引 `(row, col)`，而 C++23 之前 `operator[]` 只接受单个参数。向量作为单列矩阵，既可以用 `(i)` 单参数访问，也可以用命名访问器。

```cpp
// 矩阵：(行, 列) 索引
double fx = K(0, 0);  // 第0行第0列
double cx = K(0, 2);  // 第0行第2列
```

```cpp
// 向量：三种等价访问方式
double x = p(0);    // 通用索引——任何维度都适用
double y = p.y();   // 命名访问器——仅适用于 size >= 2 的向量
double z = p.z();   // 命名访问器——仅适用于 size >= 3 的向量
```

`x()`、`y()`、`z()` 是语法糖，让三维几何代码更可读（`point.x()` 比 `point(0)` 更直观）。但在通用算法中——例如遍历任意维度的状态向量——应使用 `p(i)` 索引形式，因为它不限制向量维度。

### 编译期维度检查

下面的代码不能通过编译：

```cpp
Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
Eigen::Vector4d q = Eigen::Vector4d::Ones();
auto bad = R * q;
```

原因是 `3x3` 无法乘以 `4x1`。

这比运行时才发现索引越界更好。

但动态矩阵不一定能在编译期发现：

```cpp
Eigen::MatrixXd A(3, 3);
Eigen::VectorXd x(4);
auto y = A * x;
```

这种错误通常会在运行时断言或调试版本中暴露。

### 常见陷阱

> ⚠️ **初始化陷阱：默认构造后直接使用**
>
> Eigen 默认构造不是数学零。状态、协方差、雅可比应显式 `Zero()`、`Identity()` 或完整赋值。

> ⚠️ **类型陷阱：为了省事全部使用 `MatrixXd`**
>
> 固定维度能表达接口契约。把旋转矩阵写成 `MatrixXd` 会丢掉编译期检查，也让读者不知道尺寸是否固定。

> ⚠️ **维度陷阱：动态矩阵错误更晚暴露**
>
> 动态尺寸适合运行期变化，但不要把固定数学结构动态化。越早暴露错误越好。

### 练习

1. **类型题**：给位置、速度、IMU 零偏、15 维误差状态和 BA Hessian 分别选择 Eigen 类型。
2. **编译题**：写一个故意维度不匹配的矩阵乘法，观察固定大小和动态大小的错误差异。
3. **初始化题**：把一个未初始化的 `Matrix3d` 示例改成稳定的相机内参矩阵初始化。

---

## 11.3 Block、Segment 与状态切片 ⭐⭐⭐⭐

### 动机：状态向量不是一串没有结构的数字

ESKF 误差状态常写成：

```text
delta x =
[
  delta p
  delta v
  delta theta
  delta bg
  delta ba
]
```

每一块都是三维。

总维度是 15。

如果代码把它当作没有结构的一维数组，后续的协方差更新、观测雅可比和误差注入都会变得难读。

Eigen 的 `segment()` 和 `block()` 正是为这类结构服务。

### 向量切片

ESKF 状态向量有明确的物理结构——前 3 个元素是位置误差，接下来 3 个是速度误差，再 3 个是姿态误差，以此类推。`segment<N>(offset)` 让代码直接按这种结构访问子块，而不是通过裸索引逐个取元素。模板参数 `<3>` 在编译期确定切片长度，编译器可以生成固定大小的拷贝代码；`kPos`、`kVel` 等常量把偏移量集中定义，避免魔法数字散落在代码各处。

```cpp
using ErrorState = Eigen::Matrix<double, 15, 1>;

// 偏移量集中定义——修改状态维度时只改这里
constexpr int kPos = 0;
constexpr int kVel = 3;
constexpr int kTheta = 6;
constexpr int kGyroBias = 9;
constexpr int kAccelBias = 12;

ErrorState dx = ErrorState::Zero();

// segment<3>(offset) 提取从 offset 开始的 3 个元素
Eigen::Vector3d dp = dx.segment<3>(kPos);       // 位置误差
Eigen::Vector3d dv = dx.segment<3>(kVel);       // 速度误差
Eigen::Vector3d dtheta = dx.segment<3>(kTheta); // 姿态误差
```

`segment<3>(kPos)` 中的 `3` 是编译期长度——这让编译器在编译期就知道切片大小，可以生成固定大小的操作代码。如果切片长度在运行期才知道，也可以使用动态版本：

```cpp
Eigen::VectorXd x(15);
auto part = x.segment(6, 3);  // 动态版本：偏移和长度都是运行期参数
```

固定长度版本更适合状态块——因为 ESKF 的每个子状态维度是算法定义的常量（位置永远是 3 维）。动态长度版本更适合通用算法——例如处理不同传感器提供的不同维度观测时。

### 矩阵块

协方差矩阵也有块结构——它的 `(i,j)` 子块描述第 `i` 个状态和第 `j` 个状态之间的协方差关系。例如 `P.block<3,3>(kPos, kVel)` 是位置和速度的交叉协方差，在 ESKF 的预测步骤中会被 IMU 加速度更新。

```cpp
using Covariance = Eigen::Matrix<double, 15, 15>;

Covariance P = Covariance::Identity();

// 提取子块——返回值是拷贝
Eigen::Matrix3d P_pp = P.block<3, 3>(kPos, kPos);   // 位置-位置协方差
Eigen::Matrix3d P_pv = P.block<3, 3>(kPos, kVel);   // 位置-速度交叉协方差
```

`block()` 的一个关键特性是它可以出现在赋值的左侧——这让就地修改协方差子块成为可能，而不需要先提取、修改、再写回：

```cpp
// 直接修改原矩阵的子块——不产生临时拷贝
P.block<3, 3>(kTheta, kTheta) =
    Eigen::Matrix3d::Identity() * 1e-4;  // 重置姿态误差的协方差
```

`block()` 返回的是表达式对象，不是立即复制的矩阵。它在语义上类似于引用——指向原矩阵的一个子区域。这意味着它可以出现在赋值左侧（修改原矩阵），也可以出现在赋值右侧（读取子块）。

### `head()`、`tail()`、`segment()`

向量常用切片：

| API | 含义 | 示例 |
|-----|------|------|
| `head<3>()` | 前 3 个元素 | 位置 |
| `tail<3>()` | 后 3 个元素 | 某些状态尾部偏置 |
| `segment<3>(i)` | 从 i 开始的 3 个元素 | 姿态误差 |
| `block<r,c>(i,j)` | 矩阵块 | 协方差子块 |

示例：

```cpp
Eigen::Matrix<double, 6, 1> se3;
Eigen::Vector3d rho = se3.head<3>();
Eigen::Vector3d phi = se3.tail<3>();
```

在李群代码里，6 维向量常被拆成平移和旋转。

具体顺序要看库约定。

有的库使用 `[rho, phi]`。

有的库使用 `[omega, upsilon]`。

切片之前必须确认顺序。

### 避免魔法数字

不推荐：

```cpp
auto dtheta = dx.segment<3>(6);
P.block<3, 3>(9, 9) *= 0.95;
```

推荐：

```cpp
namespace eskf_index {
constexpr int kPos = 0;
constexpr int kVel = 3;
constexpr int kTheta = 6;
constexpr int kGyroBias = 9;
constexpr int kAccelBias = 12;
constexpr int kErrorDim = 15;
}

auto dtheta = dx.segment<3>(eskf_index::kTheta);
P.block<3, 3>(eskf_index::kGyroBias, eskf_index::kGyroBias) *= 0.95;
```

这让代码接近状态定义表。

状态维度变化时，修改点也更集中。

更进一步，应把 offset 做成**单一事实源**。状态向量、协方差矩阵、观测雅可比、误差注入和日志输出都应引用同一份 layout，而不是每个文件各自写一遍 `6`、`9`、`12`。一旦你把重力方向、外参或时延加入状态，散落的数字会制造最难排查的错位错误：代码仍能编译，矩阵也仍是 `15x15` 或 `18x18`，但更新写到了错误物理量上。

推荐写成带维度检查的布局：

```cpp
struct EskfLayout {
    static constexpr int kPos = 0;
    static constexpr int kVel = kPos + 3;
    static constexpr int kTheta = kVel + 3;
    static constexpr int kGyroBias = kTheta + 3;
    static constexpr int kAccelBias = kGyroBias + 3;
    static constexpr int kDim = kAccelBias + 3;

    static_assert(kDim == 15);
};

using ErrorState = Eigen::Matrix<double, EskfLayout::kDim, 1>;
using Covariance = Eigen::Matrix<double, EskfLayout::kDim, EskfLayout::kDim>;
```

这样新增状态时，只要按链式 offset 修改，`kDim` 会自动跟随。协方差块访问也直接使用同一 layout：

```cpp
Covariance P = Covariance::Identity();

auto P_theta_bg =
    P.block<3, 3>(EskfLayout::kTheta, EskfLayout::kGyroBias);

P.block<3, 3>(EskfLayout::kAccelBias, EskfLayout::kAccelBias)
    .diagonal()
    .array() += 1e-6;
```

注意第一段 `auto P_theta_bg = ...` 保存的是 block 表达式，仍然指向 `P`；如果你想要一份快照值，应写：

```cpp
Eigen::Matrix3d P_theta_bg_value =
    P.block<3, 3>(EskfLayout::kTheta, EskfLayout::kGyroBias);
```

这一区分在“协方差块临时缓存”和“就地修改协方差”之间非常重要。

### `block` 表达式的生命周期

下面写法可以：

```cpp
auto theta_block = dx.segment<3>(kTheta);
theta_block.setZero();
```

`theta_block` 是指向 `dx` 子区域的表达式。

它没有复制数据。

如果 `dx` 活着，修改 `theta_block` 会修改 `dx`。

但不要让 block 表达式比原矩阵活得更久：

```cpp
auto makeBlock() {
    Eigen::Matrix<double, 15, 1> dx = Eigen::Matrix<double, 15, 1>::Zero();
    return dx.segment<3>(6);
}
```

这类写法会返回指向局部对象的表达式。

应返回具体值：

```cpp
Eigen::Vector3d makeTheta() {
    Eigen::Matrix<double, 15, 1> dx = Eigen::Matrix<double, 15, 1>::Zero();
    return dx.segment<3>(6);
}
```

### 工程例子：误差注入

```cpp
struct NavState {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
};

void injectError(NavState& state,
                 const Eigen::Matrix<double, 15, 1>& dx) {
    constexpr int kPos = 0;
    constexpr int kVel = 3;
    constexpr int kTheta = 6;
    constexpr int kGyroBias = 9;
    constexpr int kAccelBias = 12;

    state.p += dx.segment<3>(kPos);
    state.v += dx.segment<3>(kVel);

    const Eigen::Vector3d dtheta = dx.segment<3>(kTheta);
    const double angle = dtheta.norm();
    Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
    if (angle > 1e-12) {
        dq = Eigen::Quaterniond(
            Eigen::AngleAxisd(angle, dtheta / angle));
    }

    state.q = (dq * state.q).normalized();
    state.bg += dx.segment<3>(kGyroBias);
    state.ba += dx.segment<3>(kAccelBias);
}
```

这里用的是左乘姿态误差。

如果系统采用右乘误差，更新顺序会变成 `state.q = (state.q * dq).normalized();`。

这一点必须和雅可比推导一致。

从不变量角度看，误差注入不是“把 15 个数加到状态里”这么简单。位置、速度、零偏生活在线性空间，扰动可以相加；姿态生活在 SO(3)，误差生活在切空间。注入过程实际做了三种映射：

```text
delta p, delta v, delta b  : R^3 -> R^3，用加法
delta theta               : so(3) -> SO(3)，用 Exp 或角轴近似
P                          : 误差坐标系变化后的协方差重置
```

很多 ESKF 推导在误差注入后还会做 error reset，即把已注入的误差重新定义为零，并对协方差乘一个重置雅可比。入门实现可以先验证状态字段是否更新正确，但工程实现必须确认下面三件事：

1. `dx.segment<3>(kTheta)` 的左乘/右乘约定与姿态误差定义一致。
2. 观测雅可比的姿态列与同一个约定一致。
3. 协方差重置没有破坏对称性和半正定性。

可以把扰动注入拆成可测试的小函数：

```cpp
inline Eigen::Quaterniond leftPerturb(
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& dtheta) {
    const double angle = dtheta.norm();
    if (angle < 1e-12) {
        return q.normalized();
    }
    const Eigen::Quaterniond dq(
        Eigen::AngleAxisd(angle, dtheta / angle));
    return (dq * q).normalized();
}
```

测试时不要只检查“没有崩溃”，而要检查性质：

```cpp
Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
Eigen::Vector3d dz(0.0, 0.0, 1e-3);
Eigen::Quaterniond updated = leftPerturb(q, dz);

assert(std::abs(updated.norm() - 1.0) < 1e-12);
assert(updated.toRotationMatrix().determinant() > 0.999999);
```

这种性质测试比硬编码某个四元数系数更稳，因为四元数存在符号双覆盖，`q` 和 `-q` 可能表示同一个旋转。

### 常见陷阱

> ⚠️ **索引陷阱：直接写数字**
>
> `segment<3>(9)` 在几周后很难读。状态偏移应集中定义，并和状态表保持一致。

> ⚠️ **生命周期陷阱：保存 block 表达式超过原矩阵**
>
> `block()`、`segment()` 常返回引用式表达。跨函数返回时优先返回具体矩阵或向量。

> ⚠️ **扰动陷阱：姿态误差左乘右乘混用**
>
> `dq * q` 与 `q * dq` 对应不同扰动约定。代码、公式和雅可比必须一致。

### 练习

1. **切片题**：为 18 维状态 `[p, v, theta, bg, ba, g]` 定义所有偏移，并写出协方差重力块的访问代码。
2. **生命周期题**：构造一个返回 `segment()` 表达式的错误示例，再改成返回 `Vector3d`。
3. **推理题**：如果姿态误差从左乘改成右乘，`injectError()` 和观测雅可比的含义会发生什么变化？

---

## 11.4 Eigen::Map：零拷贝连接 C 接口、Ceres 与 Eigen ⭐⭐⭐⭐

### 动机：很多库给你的不是 Eigen 对象

Ceres 残差函数常见接口是：

```cpp
template <typename T>
bool operator()(const T* const pose,
                const T* const point,
                T* residual) const;
```

这里的 `pose` 和 `point` 是原始指针。

但残差计算更适合写成向量和矩阵。

如果每次都复制到 Eigen 对象，会增加开销，也会让代码更啰嗦。

`Eigen::Map` 的作用是：

```text
把一段已有连续内存解释为 Eigen 对象，不拥有内存，不拷贝数据。
```

### Map 类的设计动机：零拷贝访问外部内存

`Map` 的思路可以类比到数据库中的"视图"（view）：数据库视图不存储数据，只是对已有表的一种查看方式。修改视图中的数据等价于修改底层表。`Eigen::Map` 对原始内存做了同样的事情——它不存储矩阵数据，只是提供了一种"用 Eigen 接口查看 `double*` 内存"的方式。两者的生命周期语义也相同：如果底层表被删除，视图就失效；如果底层内存被释放，Map 就悬垂。

`Eigen::Map` 是 Eigen 中最重要也最容易被低估的工具之一。它解决的核心问题是：外部库（Ceres、PCL、ROS、C 接口）给你的数据不是 Eigen 对象，而是原始的 `double*` 或 `float*` 指针。如果每次使用前都把数据拷贝到 Eigen 矩阵中，不仅浪费时间和内存，还可能引入不一致——修改了 Eigen 副本但忘记写回原始指针。

`Map` 的设计思想是"视图"而非"容器"：它不拥有内存，只是在已有内存上叠加一层 Eigen 接口。通过 `Map`，你可以像操作普通 `Eigen::Vector3d` 一样操作 Ceres 给你的 `const double* pose` 参数块——所有 Eigen 的矩阵运算、块操作、向量化优化都可以直接使用，但底层数据没有发生任何拷贝。这种"零拷贝"设计在性能敏感的残差计算和数据转换中至关重要。

Map 的设计解决了一个在机器人系统中极为普遍的问题：**数据的生产者和消费者使用不同的内存表示**。

在一个典型的机器人软件栈中，数据流经多个组件：传感器驱动产生原始字节流，ROS 消息封装这些字节流并通过网络传输，SLAM 前端接收消息并提取特征，后端优化器将特征转化为残差方程，控制器读取优化结果并输出电机指令。每个组件对数据的看法不同——驱动看到的是 `uint8_t` 缓冲区，ROS 看到的是 `geometry_msgs::Pose`，优化器看到的是 `double*` 参数块，控制器看到的是 `Eigen::Vector3d`。

如果每次数据跨越组件边界都做一次拷贝，代价会累积：

```text
传感器 buffer (原始字节)
  → 拷贝到 ROS message 字段
    → 拷贝到 Eigen::Vector3d
      → 拷贝到 Ceres 参数块 double*
        → 拷贝到 Eigen::Map 做计算
```

Map 消除了这条链中的最后一步（也常常是最频繁的一步）。它的核心思想是**重新解释**（reinterpret）而非**复制**（copy）：Map 不分配新内存，不移动数据，只是告诉编译器"这段 `double*` 内存请按照 `Matrix<T, 3, 1>` 的方式访问"。这在以下场景中特别有价值：

**Ceres 自动微分**：Ceres 的残差函数签名要求参数是 `const T*`，因为 Ceres 需要控制参数块的内存布局（以支持自动微分和稀疏雅可比组装）。Map 让你在残差函数内部用 Eigen 的矩阵运算写残差公式，而不是手写逐元素的标量计算。这不仅更可读，也更安全——Eigen 的维度检查会在编译期捕获很多维度错误。

**传感器缓冲区**：IMU 驱动通常以固定速率（200-1000Hz）向一个环形缓冲区写入加速度和角速度数据。每次写入就是 6 个 `double`（或 `float`）。ESKF 传播步骤需要从缓冲区读取这些数据并做矩阵运算。Map 让 ESKF 直接"看到"缓冲区中的数据块为 `Vector3d`，无需每次传播都分配和拷贝两个 `Vector3d`。在 1kHz 循环中，这种零拷贝访问每次节省约 48 字节的拷贝（两个 `Vector3d`），一秒节省 48KB 的内存带宽——绝对数值不大，但在 cache 局部性和延迟可预测性方面有累积效应。

**ROS 消息**：`nav_msgs::Odometry` 中的位姿字段是 `geometry_msgs::Pose`，其内存布局是 `[px, py, pz, qx, qy, qz, qw]`。如果你的算法需要用 Eigen 对位姿做矩阵运算（例如坐标变换或协方差传播），Map 可以直接把消息字段解释为 `Eigen::Map<const Eigen::Vector3d>` 和 `Eigen::Map<const Eigen::Quaterniond>`，前提是确认内存布局和标量类型匹配。

**点云处理**：PCL 的 `PointXYZ` 结构体内存布局为 `[x, y, z, padding]`（16 字节对齐）。如果要对点云做矩阵运算（如刚体变换 `p' = R * p + t`），Map 结合 stride 可以直接把点云的 xyz 字段映射为 `Matrix<float, 3, Dynamic>`，在每个点上应用旋转矩阵，无需先把所有点拷贝到一个独立的 Eigen 矩阵中。

Map 的语义本质是"非拥有视图"（non-owning view）。这与 C++20 的 `std::span` 和 Rust 的切片（slice）是同一类抽象：提供对他人拥有的内存的安全（或至少方便）访问，不参与内存生命周期管理。Map 比 `span` 更强的地方在于它同时赋予了数据**矩阵语义**——维度、存储顺序、对齐、运算符重载全部就位。代价是使用者必须保证底层内存的生命周期覆盖 Map 的使用范围。

### 基本用法

Map 的构造只需要一个指向连续内存的指针。构造完成后，Map 对象在语法上和普通 Eigen 向量/矩阵完全相同——可以参与加减乘除、可以传入接受 `MatrixBase<Derived>` 参数的模板函数、可以用 `()` 访问元素。唯一的区别是：**Map 不拥有内存，修改 Map 就是修改底层的原始数组**。

```cpp
double raw[3] = {1.0, 2.0, 3.0};

Eigen::Map<Eigen::Vector3d> p(raw);  // p 直接"看到" raw 的 3 个 double
p(0) = 10.0;  // 修改 p 就是修改 raw[0]——此后 raw[0] == 10.0
```

如果底层数据不应被修改（例如只读传感器缓冲区），使用 `const` 映射来获得编译期的写保护：

```cpp
const double raw[3] = {1.0, 2.0, 3.0};
Eigen::Map<const Eigen::Vector3d> p(raw);  // 只读：p(0) = ... 会编译失败
```

### 映射矩阵

Map 也可以映射矩阵，但这里有一个关键的存储顺序问题。Eigen 默认使用**列主序**（column-major）——矩阵元素在内存中按列连续存放。这和 Fortran、MATLAB 的约定一致，但和 C 语言的二维数组（行主序）相反。如果你的外部数据来自 C 数组、图像库或序列化格式，很可能是行主序的——此时必须在 Map 类型中显式指定存储顺序，否则矩阵的行和列会被错误地互换。

```cpp
// 这个 buffer 看起来像单位矩阵的行主序排列
double buffer[9] = {
    1.0, 0.0, 0.0,   // "第一行"
    0.0, 1.0, 0.0,   // "第二行"
    0.0, 0.0, 1.0    // "第三行"
};

// 注意：Eigen 默认列主序——buffer 会被解释为按列填充
// buffer[0]=R(0,0), buffer[1]=R(1,0), buffer[2]=R(2,0), ...
Eigen::Map<Eigen::Matrix3d> R(buffer);
// 如果 buffer 实际是行主序的单位矩阵，恰好对角阵行列互换后仍是单位阵
// 但对于非对角矩阵，行列主序解释会给出完全不同的矩阵！
```

如果外部数据确实是行主序排列，应显式指定行主序类型：

```cpp
using RowMajorMatrix3d =
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

// 行主序映射——buffer[0]=R(0,0), buffer[1]=R(0,1), buffer[2]=R(0,2), ...
Eigen::Map<RowMajorMatrix3d> R_row_major(buffer);
```

### Ceres 参数块映射

Ceres 优化器把所有参数存储为连续的 `double` 数组。代价函数通过 `const T* const` 指针接收参数块。Map 在这里的作用是把裸指针"升级"为 Eigen 向量，让残差计算可以使用矩阵运算而不是手写索引。典型的位置残差如下：

```cpp
template <typename T>
struct PositionResidual {
    explicit PositionResidual(const Eigen::Vector3d& measured)
        : measured_(measured) {}

    bool operator()(const T* const p_raw,
                    T* residual_raw) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(p_raw);
        Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residual_raw);

        r = p - measured_.cast<T>();
        return true;
    }

    Eigen::Vector3d measured_;
};
```

注意两点：

1. 模板标量 `T` 可能是 `double`，也可能是 Ceres 的自动微分类型。
2. `measured_` 是 `double`，进入模板计算时要 `.cast<T>()`。

### `Map` 与生命周期

`Map` 不拥有内存。

这意味着：

```cpp
Eigen::Map<Eigen::Vector3d> badMap() {
    double raw[3] = {1.0, 2.0, 3.0};
    return Eigen::Map<Eigen::Vector3d>(raw);
}
```

返回后 `raw` 已经销毁。

`Map` 变成悬垂引用。

正确做法是让外部内存生命周期覆盖 `Map` 使用范围，或者返回具体值：

```cpp
Eigen::Vector3d makeVector() {
    double raw[3] = {1.0, 2.0, 3.0};
    return Eigen::Map<Eigen::Vector3d>(raw);
}
```

### owned object、`Map` 与 `Ref`

Eigen 接口里最容易混淆的是“值对象”和“视图对象”。

| 形式 | 是否拥有数据 | 典型用途 |
|------|--------------|----------|
| `Eigen::Vector3d` | 拥有 | 状态字段、计算结果、需要长期保存的值 |
| `Eigen::Map<Vector3d>` | 不拥有 | 把原始连续内存解释成 Eigen 对象 |
| `Eigen::Ref<const Vector3d>` | 不拥有 | 函数参数，接收兼容的 Eigen 对象或表达式值 |
| `auto block = x.segment<3>(i)` | 不拥有 | 指向原对象某个切片的表达式 |

如果函数只想读一个三维向量，可以写具体类型：

```cpp
double pointNorm(const Eigen::Vector3d& p) {
    return p.norm();
}
```

这对普通 `Vector3d` 很清晰，但对某些表达式或映射不够灵活。`Eigen::Ref` 常用于公共函数边界：

```cpp
double pointNormRef(const Eigen::Ref<const Eigen::Vector3d>& p) {
    return p.norm();
}
```

`Ref` 的含义是：调用者给我一段能被当作 `Vector3d` 读取的 Eigen 数据。它不拥有数据，也不应该被保存到对象成员里长期使用。对输入参数，`Ref<const ...>` 很常见；对输出参数，可以使用可写 `Ref`：

```cpp
void normalizeInPlace(Eigen::Ref<Eigen::Vector3d> p) {
    const double n = p.norm();
    if (n > 1e-12) {
        p /= n;
    }
}
```

工程边界是：`Map` 适合在“原始内存入口”创建视图，`Ref` 适合在“函数接口”接收视图。不要把 `Ref` 或 `Map` 当成成员变量保存，除非这个类本身就是明确的短生命周期视图类型。

### `Map` 与 stride

有些点云内存不是紧密的 `x y z x y z`。

例如每个点包含：

```text
x y z intensity timestamp
```

如果你只想映射 xyz，不能把整段点云简单当作 `3xN` 矩阵。

这时需要 stride。

概念示意：

```cpp
using Matrix3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Stride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;

Eigen::Map<Matrix3X, 0, Stride> xyz(
    raw,
    3,
    num_points,
    Stride(point_stride, 1));
```

stride 是高级用法。

实际项目中要同时确认：

1. 标量类型是否一致。
2. 字段是否连续。
3. 存储顺序是否匹配。
4. 对齐是否满足。
5. 生命周期是否覆盖计算。

要真正用对 stride，需要区分 inner stride 和 outer stride。对一个列主序矩阵，内层方向是“同一列内相邻元素”，外层方向是“相邻列的起点”。`Stride<Outer, Inner>` 中：

```text
inner stride = 沿内层方向走一步跨过多少个标量
outer stride = 沿外层方向走一步跨过多少个标量
```

Eigen 默认列主序的 `Matrix<double, 3, Dynamic>` 把每个点作为一列，紧密布局是：

```text
x0 y0 z0 x1 y1 z1 x2 y2 z2 ...
```

此时一个 `3xN` 映射的 inner stride 为 1，outer stride 为 3。若外部点结构是 AoS：

```cpp
struct PointXYZIT {
    double x;
    double y;
    double z;
    double intensity;
    double time;
};
```

内存是：

```text
x0 y0 z0 i0 t0 x1 y1 z1 i1 t1 ...
```

只映射 xyz 为 `3xN` 时，inner stride 仍为 1，outer stride 变为 5：

```cpp
using Matrix3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Stride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;

Eigen::Map<const Matrix3X, 0, Stride> xyz(
    raw,
    3,
    num_points,
    Stride(5, 1));
```

如果是 SoA 布局：

```text
x0 x1 x2 ... y0 y1 y2 ... z0 z1 z2 ...
```

它适合映射成行主语义的 `3xN`，但每一行内部连续，行与行之间相隔 `num_points`。这时最容易犯的错误是沿用 AoS 的 stride。正确做法取决于你选择的 Eigen 矩阵存储顺序和数据解释方式，通常需要写一个最小测试：给三点填入可区分的数，映射后检查每一列是否真的是 `(x_i,y_i,z_i)`。

### Ceres 参数块的视图边界

Ceres 常把参数块拆成多个 `double*` 或 `T*`。例如 `[tx,ty,tz,qx,qy,qz,qw]`：

```cpp
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> poseBlockToIsometry(
    const T* pose) {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose);
    Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
    q.normalize();

    Eigen::Transform<T, 3, Eigen::Isometry> T_world_body =
        Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    T_world_body.linear() = q.toRotationMatrix();
    T_world_body.translation() = t;
    return T_world_body;
}
```

这里 `Map` 只用于平移，因为平移三项连续且顺序一致。Eigen 可以映射四元数系数，但必须明确内存契约：Eigen 构造函数参数顺序是 `(w,x,y,z)`，而系数内存常按 `[x,y,z,w]` 组织。外部数组一旦涉及重排、归一化或坐标约定，就应该显式构造值对象，而不是让 `Map` 隐含语义。

### 常见陷阱

> ⚠️ **所有权陷阱：以为 `Map` 拷贝了数据**
>
> `Map` 只是视图。原始内存失效，`Map` 也失效。

> ⚠️ **存储顺序陷阱：把行主序 buffer 当列主序矩阵**
>
> Eigen 默认列主序。外部 C 数组、图像、某些消息数据常按行主序组织，映射前必须确认。

> ⚠️ **自动微分陷阱：在 Ceres 模板残差里写死 `Vector3d`**
>
> 自动微分时标量不是 `double`。残差内部应使用 `Matrix<T, 3, 1>`，常量用 `.cast<T>()`。

### 练习

1. **映射题**：把 `double pose[7]` 的前三维映射成平移，把后四维映射成四元数系数，并说明四元数顺序。
2. **Ceres 题**：写一个点到平面的残差，用 `Map<const Matrix<T,3,1>>` 读取点坐标。
3. **存储题**：构造一个行主序 3x3 数组，分别用默认 `Map<Matrix3d>` 和 `Map<RowMajorMatrix3d>` 解释，比较输出。

---

## 11.5 旋转、四元数、角轴与 Isometry ⭐⭐⭐⭐

### 动机：旋转不是普通三维向量

旋转和向量的关系，可以类比到球面上的方向和平面上的位置。在平面上，两个位置之间的"差"是一个位移向量，位移向量可以自由相加。但在球面上，两个方向之间的"差"不是另一个方向——从北纬 30 度到北纬 60 度的"旋转"不能和另一个旋转简单相加，因为球面不是平坦的。旋转矩阵生活在 SO(3) 这个弯曲的流形上，就像方向生活在球面上。要在流形上做微积分（优化、插值、误差计算），必须先把问题映射到切空间——一个局部平坦的线性空间——做完运算后再映射回流形。这就是 Exp/Log 映射和李代数的核心思想。

旋转是机器人学中最基础也最容易出错的数学对象。它的核心难点在于：旋转不生活在向量空间中——两个旋转矩阵相加没有物理意义，旋转矩阵的"中间值"也不是合法的旋转。这些性质使得旋转的表示、插值、优化和误差计算都需要专门的数学工具，不能简单套用向量空间中的线性代数方法。

Eigen 提供了四种旋转表示（`Matrix3d`、`Quaterniond`、`AngleAxisd`、`Rotation2Dd`），每种都有各自的工程适用场景和数值陷阱。选择错误的表示不仅会导致代码冗长，还可能引入数值退化——例如用欧拉角在万向节锁附近做优化，或者用未归一化的四元数做长时间的积分。

平移可以直接相加：

```cpp
p_new = p + delta_p;
```

旋转不能简单相加。

旋转矩阵必须保持正交：

```text
R^T R = I
det(R) = 1
```

四元数必须保持单位长度：

```text
||q|| = 1
```

如果把旋转当普通向量加减，很快会离开有效旋转集合。

Eigen 提供多种旋转表示：

| 表示 | Eigen 类型 | 适合场景 |
|------|------------|----------|
| 旋转矩阵 | `Matrix3d` | 线性变换、雅可比推导 |
| 四元数 | `Quaterniond` | 插值、状态保存、避免欧拉角奇异 |
| 角轴 | `AngleAxisd` | 构造旋转、解释单轴旋转 |
| 欧拉角 | `Vector3d` 或 `eulerAngles()` | 人类可读配置，谨慎使用 |
| 刚体变换 | `Isometry3d` | 旋转 + 平移 |

### Quaterniond 的顺序陷阱

四元数有一个让无数机器人工程师踩过的坑：**构造函数参数顺序和内部存储顺序不同**。这不是 Eigen 的 bug，而是历史约定冲突的结果。数学文献中四元数通常写成 $q = w + xi + yj + zk$，标量部 $w$ 在前；但 Hamilton 四元数的向量部 $(x, y, z)$ 在几何上对应旋转轴方向，很多序列化格式（包括 ROS、TF2、Bullet）把向量部放在前面。Eigen 选择了折中方案：构造函数按数学约定 `(w, x, y, z)`，存储按向量优先 `[x, y, z, w]`。

```cpp
// 构造函数：标量 w 在前（数学约定）
Eigen::Quaterniond q(w, x, y, z);
```

但内部系数 `coeffs()` 返回的是一个 4 维向量，顺序是 `[x, y, z, w]`——向量部在前，标量部在末尾：

```cpp
Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);  // 构造：w=1, x=0, y=0, z=0

std::cout << q.w() << "\n";                 // 输出 1.0（标量部）
std::cout << q.coeffs().transpose() << "\n"; // 输出 "0 0 0 1"（存储顺序 x,y,z,w）
```

这个差异在与外部系统交换数据时是最常见的 bug 来源。ROS 的 `geometry_msgs::Quaternion` 消息字段排列为 `x, y, z, w`——恰好和 Eigen 的存储顺序一致，但和构造函数参数顺序相反。如果不注意这个差异，把 ROS 消息的字段按顺序传入 Eigen 构造函数，`x` 会被当成 `w`，`w` 会被当成 `z`，旋转方向完全错误。

```cpp
// 正确：注意参数顺序——构造函数是 (w, x, y, z)，ROS 消息是 x, y, z, w
Eigen::Quaterniond q(msg.orientation.w,    // 第一个参数是标量部
                     msg.orientation.x,
                     msg.orientation.y,
                     msg.orientation.z);
```

### 旋转矩阵与四元数互转

```cpp
Eigen::Matrix3d R = q.toRotationMatrix();
Eigen::Quaterniond q_from_R(R);
q_from_R.normalize();
```

从数值矩阵构造四元数前，应确认矩阵接近旋转矩阵。

如果矩阵来自积分、优化或噪声数据，可能略微不正交。

工程上常见做法是：

1. 用 SVD 投影到最近旋转矩阵。
2. 或在每次四元数更新后 normalize。
3. 或使用李群库维护旋转不变量。

### 为什么四元数必须单位化：一个不变量约束的工程故事

四元数表示旋转的前提条件是 $\|q\| = 1$——这不是可选的"最佳实践"，而是数学上的硬性约束。如果四元数的模不为 1，它对应的变换不再是纯旋转，而是旋转加上缩放。在 SLAM 的状态估计中，优化器每次更新四元数后，数值误差会让模慢慢偏离 1。如果不定期归一化，经过几百次迭代后旋转矩阵会产生明显的缩放效应——点云配准结果"看起来对但尺度不对"，位姿图的轨迹长度莫名其妙地缩放，这些都是典型的四元数未归一化症状。

四元数可以写成：

```text
q = w + x i + y j + z k
```

其中 `i,j,k` 满足：

```text
i^2 = j^2 = k^2 = ijk = -1
```

用四元数旋转三维向量时，通常把三维向量写成纯虚四元数：

```text
p = 0 + p_x i + p_y j + p_z k
```

旋转写成：

```text
p' = q p q^{-1}
```

如果 `q` 是单位四元数，那么：

```text
q^{-1} = q^*
```

也就是只需要取共轭。

更重要的是，单位四元数对应 SO(3) 中的合法旋转。若 `||q|| != 1`，上式会把向量长度也缩放：

```text
||q p q^{-1}|| = ||p||        仅当 q^{-1} 被正确使用且 q 非零
||q p q^*|| = ||q||^2 ||p||   若误把共轭当逆，则非单位 q 会引入尺度
```

很多代码为了效率和习惯，会默认单位四元数的逆就是共轭。如果积分或优化后没有归一化，这个假设就失效。

所以工程代码常写：

```cpp
state.q = state.q.normalized();
```

这不是形式主义，而是在把数值结果投影回单位四元数流形。

### 小角度误差为什么用角轴构造

ESKF 中的姿态误差常用三维小量 `delta theta`。

它不是欧拉角。

更准确地说，它是 SO(3) 李代数中的一个三维切向量。

当 `delta theta` 很小时，可以把它理解为：

```text
方向：旋转轴
长度：旋转角度
```

因此：

```text
angle = ||delta theta||
axis  = delta theta / ||delta theta||
```

再用角轴构造四元数。

如果 `angle` 接近 0，轴方向本身没有稳定定义，因为零向量没有方向。此时应直接返回单位增量，或者使用李群库中带小角度展开的 `Exp()`。

这个推导解释了为什么代码要判断阈值：

```cpp
if (angle > 1e-12) {
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
}
```

阈值不是随便加的补丁，而是因为 `delta theta / angle` 在零附近没有数学意义。

### Rodrigues 公式与 SO(3) 指数映射

`delta theta` 更严格的名字是李代数元素 `phi`。给定 $\boldsymbol{\phi} \in \mathbb{R}^3$，定义反对称矩阵：

```text
[phi]_x =
[   0   -phi_z  phi_y
  phi_z    0   -phi_x
 -phi_y  phi_x    0   ]
```

SO(3) 的指数映射把切空间扰动映射成旋转矩阵：

```text
Exp(phi) = I
         + sin(theta) / theta [phi]_x
         + (1 - cos(theta)) / theta^2 [phi]_x^2

theta = ||phi||
```

这就是 Rodrigues 公式。它解释了为什么“小角度向量”不是欧拉角：`phi` 不是三个独立轴角依次旋转，而是一个切向量，通过指数映射一次性进入 SO(3)。当 `theta` 很小时，直接计算 `sin(theta)/theta` 和 `(1-cos(theta))/theta^2` 会有数值消失问题，需要用泰勒展开：

$$
\frac{\sin\theta}{\theta} \approx 1 - \frac{\theta^2}{6}
$$

$$
\frac{1 - \cos\theta}{\theta^2} \approx \frac{1}{2} - \frac{\theta^2}{24}
$$

Eigen 的 `AngleAxisd` 适合普通构造，但它不会替你表达整套李群扰动约定。SLAM 项目中常用 Sophus、manif 或自写 `ExpSO3`，原因就是需要统一左扰动、右扰动、雅可比和小角度展开。

一个最小 `hat` 函数如下：

```cpp
inline Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0.0, -w.z(), w.y(),
         w.z(), 0.0, -w.x(),
        -w.y(), w.x(), 0.0;
    return W;
}
```

性质测试比记公式更可靠：

```cpp
Eigen::Vector3d w(0.01, -0.02, 0.03);
Eigen::Matrix3d W = hat(w);

assert((W + W.transpose()).norm() < 1e-12);

Eigen::Vector3d v(1.0, 2.0, 3.0);
assert((W * v - w.cross(v)).norm() < 1e-12);
```

### 四元数双覆盖与残差连续性

单位四元数在三维球面 `S^3` 上表示 SO(3)，但它是双覆盖：

```text
q 和 -q 表示同一个旋转
```

这会造成两个工程边界。

第一，单元测试不能只比较四元数四个系数完全相等。更稳的比较方式是比较旋转矩阵，或比较四元数内积的绝对值：

```cpp
bool sameRotation(const Eigen::Quaterniond& a,
                  const Eigen::Quaterniond& b) {
    const double dot = std::abs(a.normalized().dot(b.normalized()));
    return std::abs(dot - 1.0) < 1e-12;
}
```

第二，优化中如果直接对四元数系数做残差，`q` 与 `-q` 的符号跳变会让残差不连续。更常见的做法是用局部坐标：

```text
rotation residual = Log(R_measured^T R_estimated)
```

也就是把旋转误差拉回三维切空间，而不是在四维单位球面上直接做欧氏减法。Eigen 能表达四元数和旋转矩阵，但“残差应该定义在哪个空间”仍然是算法设计问题。

### AngleAxisd

角轴表示：

```cpp
Eigen::AngleAxisd aa(angle, axis.normalized());
Eigen::Quaterniond q(aa);
Eigen::Matrix3d R = aa.toRotationMatrix();
```

适合表达“绕某个轴旋转多少角度”。

在小角度误差注入中，也常从误差向量构造角轴：

```cpp
Eigen::Vector3d dtheta = dx.segment<3>(6);
double angle = dtheta.norm();

Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
if (angle > 1e-12) {
    dq = Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, dtheta / angle));
}
```

小角度接近零时不能直接除以 `angle`。

要么做阈值判断，要么使用李群库的指数映射。

### Isometry3d

`Isometry3d` 表示刚体变换。

它内部通常可以看作 4x4 矩阵，但语义是：

```text
T =
[ R  t
  0  1 ]
```

构造：

```cpp
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
T.linear() = R;
T.translation() = t;
```

变换点：

```cpp
Eigen::Vector3d p_world = T_world_body * p_body;
```

组合变换：

```cpp
Eigen::Isometry3d T_world_lidar =
    T_world_body * T_body_lidar;
```

读法是：先从 lidar 到 body，再从 body 到 world。

变量名要表达方向。

`T_world_body` 应表示把 body 坐标中的点变到 world 坐标。

从 SE(3) 角度，`T_world_body` 是一个群元素，组合和求逆都应保持群不变量：

```cpp
Eigen::Isometry3d T_world_body = Eigen::Isometry3d::Identity();
Eigen::Isometry3d T_body_lidar = Eigen::Isometry3d::Identity();

Eigen::Isometry3d T_world_lidar = T_world_body * T_body_lidar;
Eigen::Isometry3d T_lidar_world = T_world_lidar.inverse();
```

最小性质测试应覆盖：

```cpp
Eigen::Vector3d p_lidar(1.0, 2.0, 3.0);
Eigen::Vector3d p_world = T_world_lidar * p_lidar;
Eigen::Vector3d recovered = T_lidar_world * p_world;

assert((recovered - p_lidar).norm() < 1e-12);
assert(std::abs(T_world_lidar.linear().determinant() - 1.0) < 1e-12);
assert((T_world_lidar.linear().transpose() *
        T_world_lidar.linear() -
        Eigen::Matrix3d::Identity()).norm() < 1e-12);
```

这类测试把“合法刚体变换”的抽象不变量写进代码，比只测试某个输出数值更能发现方向约定和归一化错误。

### 反面：用 Matrix4d 表示所有变换

选择 `Isometry3d` 还是 `Matrix4d` 不仅是风格偏好，而是类型系统能否帮你防止错误的问题。`Matrix4d` 可以表示任何 4x4 矩阵——包括投影矩阵、仿射变换、透视变换、甚至毫无几何意义的随机矩阵。`Isometry3d` 在语义上声明"我是一个保持距离的刚体变换"，虽然 Eigen 不在运行时强制检查旋转部分是否正交，但类型名本身就是对读者和协作者的承诺。在大型 SLAM 项目中，用 `Matrix4d` 表示所有变换会导致"这个矩阵到底是位姿、投影、还是雅可比？"的困惑——而 `Isometry3d` 一目了然。

`Matrix4d` 能表示齐次矩阵。

但它也能表示不合法的刚体变换。

例如：

```cpp
Eigen::Matrix4d T = Eigen::Matrix4d::Random();
```

这不是一个合法刚体变换。

`Isometry3d` 至少在语义上告诉读者这是等距变换。

但它也不能自动防止你写入非正交矩阵。

因此，关键变换仍应有测试和归一化策略。

### 常见陷阱

> ⚠️ **顺序陷阱：四元数构造顺序与存储顺序不同**
>
> Eigen 构造函数是 `(w,x,y,z)`，`coeffs()` 是 `[x,y,z,w]`。ROS、配置文件和优化参数块常用不同顺序，必须显式转换。

> ⚠️ **归一化陷阱：优化或积分后忘记 normalize**
>
> 四元数如果不保持单位长度，后续旋转矩阵会逐渐失真。

> ⚠️ **方向陷阱：`T_ab` 的含义不统一**
>
> 团队必须约定 `T_world_body` 表示从 body 到 world，还是 world 到 body。变量名、注释和测试要一致。

### 练习

1. **转换题**：把 ROS 风格 `[x,y,z,w]` 数组转换为 Eigen 四元数，再转换回数组。
2. **变换题**：写 `transformPoint(T_world_body, p_body)`，并用一个平移 + 90 度旋转的例子验证。
3. **约定题**：为 `T_camera_lidar` 写出两种可能解释，并说明哪一种更适合你的项目。
4. **[跨章综合题]**：结合 移动语义与完美转发 的移动语义和 类型转换 的 `static_cast`，解释为什么 `Eigen::Quaterniond q = static_cast<Eigen::Quaterniond>(rotation_matrix)` 不是合法写法，而必须通过构造函数 `Eigen::Quaterniond q(rotation_matrix)` 来转换。讨论 Eigen 为什么不提供隐式转换运算符，以及这种设计如何防止坐标系混淆。

---

## 11.6 表达式模板、`auto` 与 `.eval()` ⭐⭐⭐⭐

表达式模板是 Eigen 性能的核心机制，也是 Eigen 与"普通矩阵库"的根本区别。理解表达式模板不仅能避免 Eigen 中最常见的 bug（`auto` 悬垂引用、aliasing 错误），还能帮助你理解 C++ 模板元编程在数值计算中的典型应用。本节从"为什么需要延迟求值"出发，推导出表达式模板的工作机制，然后讨论它带来的工程陷阱。

### 动机：Eigen 为什么不立刻计算

表达式模板的思路可以类比到 SQL 查询优化器。当你写 `SELECT * FROM A JOIN B JOIN C WHERE ...` 时，数据库不会先执行 `A JOIN B` 得到一个中间表，再把中间表 `JOIN C`。它会先构建一棵查询计划树，分析整体结构后选择最优的执行顺序。Eigen 的表达式模板做了相同的事情——`A + B + D` 不是先算 `A + B` 得到临时矩阵，而是先构建"加法树"，在赋值时一次性遍历完成全部计算。两者的关键差异在于：SQL 优化器在运行时工作，Eigen 的表达式模板在编译期工作——优化决策被编码在类型系统中，零运行时开销。

普通写法：

```cpp
Eigen::Matrix3d C = A * B + D;
```

如果每个运算都立刻生成临时矩阵，过程可能是：

```text
tmp = A * B
C = tmp + D
```

Eigen 使用表达式模板，把”要计算什么”先记录成表达式，再在赋值给具体矩阵时统一求值。

这能减少临时对象，并让编译器优化小矩阵。

但它也带来一个重要边界：某些表达式对象保存的是对原矩阵的引用。

### 表达式模板的编译器视角：为什么 lazy evaluation 能避免临时变量

要真正理解 Eigen 的表达式模板，需要从编译器的角度看”一行矩阵运算”在底层经历了什么。

**如果没有表达式模板（eager evaluation）**：考虑 `C = A + B + D`，其中 `A`、`B`、`D` 都是 `Matrix3d`。如果 `operator+` 返回一个完整的 `Matrix3d` 结果，编译器看到的调用链是：

```text
1. tmp1 = operator+(A, B)    // 分配 72 字节（9 个 double），逐元素计算 A+B，写入 tmp1
2. tmp2 = operator+(tmp1, D) // 再分配 72 字节，逐元素计算 tmp1+D，写入 tmp2
3. C = tmp2                  // 把 tmp2 拷贝到 C（可能被移动优化省掉）
4. 析构 tmp2, tmp1           // 释放两个临时对象
```

问题不在于 72 字节本身——对 `Matrix3d` 来说这只是栈上分配。真正的代价是**内存访问模式**：计算 `A+B` 时要读 A 和 B 的全部 18 个 double，写 tmp1 的 9 个 double；计算 `tmp1+D` 时又要读 tmp1 的 9 个 double 和 D 的 9 个 double，写 tmp2 的 9 个 double。总共需要 36 次读 + 18 次写 = 54 次内存访问。如果是大矩阵 `MatrixXd`，这些临时对象还在堆上分配，触发 `malloc`/`free`，cache 局部性也会更差。

**有了表达式模板（lazy evaluation）**：`operator+(A, B)` 不返回 `Matrix3d`，而返回一个轻量级类型 `CwiseBinaryOp<sum, Matrix3d, Matrix3d>`。这个类型不保存 9 个 double 的结果，只保存两个引用（指向 A 和 B）和一个操作标签（加法）。`operator+(tmp_expr, D)` 同理，返回 `CwiseBinaryOp<sum, CwiseBinaryOp<...>, Matrix3d>`。整棵表达式树的大小只是几个指针。

赋值 `C = ...` 时，Eigen 的赋值运算符展开这棵树，生成等价于：

```cpp
// 编译器看到的等效代码（概念性展开）：
for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
        C(i,j) = A(i,j) + B(i,j) + D(i,j);  // 一次循环完成全部计算
```

只需要读 A、B、D 各一次（27 次读），写 C 一次（9 次写），总共 36 次内存访问——比 eager 模式少了 18 次，而且**零临时对象**。对于固定大小的小矩阵，编译器甚至可以把整个循环展开成 9 条独立的加法指令，全部在寄存器中完成。

**编译器为什么能做到这一点？** 关键在于 C++ 模板的特性：表达式树的形状被编码在类型系统中。`CwiseBinaryOp<sum, CwiseBinaryOp<sum, Matrix3d, Matrix3d>, Matrix3d>` 这个类型本身就完整描述了”先加前两个再加第三个”的计算结构。当赋值运算符模板实例化时，编译器看到的不是”某个抽象的表达式对象”，而是”一棵完全确定的、可以静态展开的计算树”。每个节点的 `operator()(i, j)` 都是 `inline` 函数，会被层层内联，最终变成单个循环体中的一条表达式。

这就是为什么表达式模板是 C++ 零开销抽象的经典案例：写出来的代码看起来像数学公式 `C = A + B + D`，生成的机器码却和手写的循环一样高效。代价是类型系统变得复杂——`auto expr = A + B` 保存的不是矩阵，而是一个引用了 A 和 B 的表达式节点。

**什么时候 lazy evaluation 反而不好？** 如果同一个子表达式被多次使用：

```cpp
auto ab = A + B;
C = ab + D;
E = ab + F;
```

`ab` 是表达式节点，每次使用都重新计算 `A + B`。如果 `A + B` 计算代价很高（例如大矩阵乘法），重复计算比保存一个临时结果更浪费。此时应该显式求值：`Matrix3d ab = (A + B).eval();`。

> **本质洞察**：Eigen 的表达式模板把”何时计算”的决定权从运算符移到了赋值点。运算符只构建计算描述（表达式树），赋值才触发计算执行。这种”描述与执行分离”的模式让编译器能看到完整的计算结构，从而做出最优的代码生成决策——消除临时变量、合并循环、利用 SIMD 指令。但代价是程序员必须意识到”表达式对象不是结果对象”这个关键区别。

#### 完整推演：`C = A + B * C` 的求值过程

为了让表达式模板从”概念理解”变成”能手动推导”的工具，下面以 `Result = A + B * D` 为例（其中 `A`、`B`、`D` 都是 `Matrix3d`），逐步展示有表达式模板和没有表达式模板时分别发生了什么。

**没有表达式模板（eager evaluation）的求值过程**：

```text
Step 1: tmp1 = B * D
  - 分配 72 字节临时矩阵 tmp1（9 个 double）
  - 计算 3x3 矩阵乘法：27 次乘法 + 18 次加法
  - 内存操作：读 B 的 9 个 double + 读 D 的 9 个 double + 写 tmp1 的 9 个 double
  - 合计：18 次读 + 9 次写 = 27 次内存访问

Step 2: tmp2 = A + tmp1
  - 分配 72 字节临时矩阵 tmp2
  - 逐元素加法：9 次加法
  - 内存操作：读 A 的 9 个 double + 读 tmp1 的 9 个 double + 写 tmp2 的 9 个 double
  - 合计：18 次读 + 9 次写 = 27 次内存访问

Step 3: Result = tmp2
  - 拷贝 tmp2 到 Result（可能被移动优化省掉）

Step 4: 析构 tmp2, tmp1
  - 对栈上固定大小矩阵代价低，但对 MatrixXd 则是两次 free

总计：2 个临时对象，54 次内存访问，最少 144 字节临时内存
```

**有表达式模板（lazy evaluation）的求值过程**：

```text
Step 1: B * D 不计算——返回 ProductExpr<Matrix3d, Matrix3d>
  - 这个表达式节点只保存两个引用（指向 B 和 D）
  - 大小：约 16 字节（两个指针），零内存分配

Step 2: A + (B * D) 不计算——返回 CwiseBinaryOp<sum, Matrix3d, ProductExpr<...>>
  - 这个表达式节点保存一个引用（指向 A）和一个子表达式节点
  - 大小：约 24 字节（指针 + 嵌入的乘法节点），零内存分配

Step 3: Result = 整棵表达式树
  - 赋值运算符展开整棵树，对乘法部分使用优化的 Kernel
  - Eigen 在赋值时检测到 “加法 + 乘法” 的模式，可能直接调用
    gemm(1.0, B, D, 1.0, A, Result) 一步完成
  - 内存操作：读 A, B, D 各一次 + 写 Result 一次
  - 合计：27 次读 + 9 次写 = 36 次内存访问

总计：0 个临时对象，36 次内存访问，0 字节临时内存
```

**对比总结**：

| 指标 | Eager（无表达式模板） | Lazy（有表达式模板） |
|------|----------------------|---------------------|
| 临时对象数量 | 2 个 | 0 个 |
| 内存访问次数 | 54 次 | 36 次 |
| 临时内存占用 | 144 字节 | 0 字节 |
| 循环次数 | 至少 2 次遍历 | 1 次遍历（可能融合为 gemm） |

对于 `MatrixXd`，差距更大——每个临时对象都需要 `malloc`/`free`，3x3 矩阵的 54 次 vs 36 次内存访问变成 1000x1000 矩阵的 600 万次 vs 400 万次内存访问。在 SLAM 后端的 Hessian 组装中，这种差距直接决定了每帧能否在时间预算内完成。

可以把表达式：

```cpp
Eigen::Matrix3d C = A * B + D;
```

理解成一棵表达式树：

```text
       assign to C
          |
          +
        /   \
       *     D
     /   \
    A     B
```

`A * B + D` 在赋值之前不一定生成一个 `Matrix3d` 临时对象。它可能是一个轻量表达式对象，内部保存对 `A`、`B`、`D` 的引用和”将来如何访问第 `(i,j)` 个元素”的规则。真正求值发生在赋给 `C`、调用 `.eval()`、或某些 API 需要具体值的时候。

这套机制的抽象不变量是：**表达式对象描述计算，具体矩阵对象保存结果。** 如果你用 `auto` 保存表达式对象，却以为保存了结果，就会把求值时间点推迟到自己没有意识到的位置。

### `auto` 陷阱：为什么 Eigen 是 `auto` 关键字最危险的搭档

在普通 C++ 代码中，`auto` 是一个几乎总是正确的类型推导工具——它减少冗余、避免类型错误、让代码更简洁。但在 Eigen 代码中，`auto` 可能把一个看似简单的赋值变成一颗定时炸弹。原因在于：Eigen 的运算符返回的不是结果矩阵，而是表达式节点——`auto` 会忠实地推导出这个表达式节点的类型，而不是你期望的 `Matrix3d`。表达式节点内部持有对原始矩阵的引用，如果原始矩阵在表达式节点被求值之前发生了变化或被销毁，求值结果就会是错误的或未定义的。

这个问题在普通 C++ 中不存在，因为普通的 `operator+` 返回的是值对象——`auto c = a + b` 中 `c` 拥有独立的数据副本。Eigen 为了性能选择了延迟求值，代价是 `auto` 的语义变了——从"保存结果值"变成"保存计算描述"。理解这个区别是避免 Eigen bug 的关键。

错误示例：

```cpp
Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
Eigen::Matrix3d B = Eigen::Matrix3d::Ones();

auto expr = A + B;
A.setZero();

std::cout << expr << "\n";
```

`expr` 不一定是计算结果矩阵。

它可能是记录 `A + B` 的表达式对象。

当打印时，它再读取已经被修改的 `A`。

更稳的写法：

```cpp
Eigen::Matrix3d value = A + B;
```

或：

```cpp
auto value = (A + B).eval();
```

### 什么时候需要 `.eval()`

| 场景 | 是否建议 `.eval()` |
|------|--------------------|
| 同一语句赋给具体矩阵 | 通常不需要 |
| 用 `auto` 保存表达式结果 | 建议 |
| 表达式引用的原对象马上要变 | 需要 |
| 函数返回局部矩阵的表达式 | 需要返回具体类型 |
| block 与原矩阵重叠赋值 | 视情况需要 |

示例：

```cpp
auto H = (J.transpose() * J).eval();
auto g = (J.transpose() * r).eval();
```

这明确告诉读者：这里要保存数值结果，不是保存表达式。

公共函数返回 Eigen 表达式时也要谨慎。下面的写法危险：

```cpp
auto makeResidualExpression() {
    Eigen::Vector3d a(1.0, 2.0, 3.0);
    Eigen::Vector3d b(0.5, 0.5, 0.5);
    return a - b;
}
```

返回的可能是引用局部变量的表达式。稳妥写法是返回具体值：

```cpp
Eigen::Vector3d makeResidualValue() {
    Eigen::Vector3d a(1.0, 2.0, 3.0);
    Eigen::Vector3d b(0.5, 0.5, 0.5);
    return a - b;
}
```

在函数参数上，如果你希望接收“已经可读的矩阵或向量视图”，用 `Ref` 或 `MatrixBase` 表达接口；如果你希望保存结果，函数内部转成具体矩阵：

```cpp
void consumeResidual(const Eigen::Ref<const Eigen::Vector3d>& r) {
    const Eigen::Vector3d value = r;
    // value 是当前数值快照，可以安全长期使用在本函数内。
}
```

### aliasing：左右两边共享内存

aliasing（别名）问题是表达式模板延迟求值带来的最危险副作用之一。当赋值的左边和右边引用同一块内存时，延迟求值可能导致"读取尚未修改的数据"和"覆盖还没读取的数据"交替发生。这在普通的 eager evaluation 中不成问题——因为右边先被完整求值到一个临时对象中，然后整体拷贝到左边。但在 lazy evaluation 下，右边的表达式在赋值过程中逐元素求值，写入左边的某个元素可能覆盖右边还需要读取的另一个元素。

考虑矩阵转置的自赋值：

```cpp
// 危险：左右两边是同一块内存，延迟求值可能导致数据覆盖
M = M.transpose();
```

对于 3x3 矩阵，`M(0,1)` 和 `M(1,0)` 需要互换。如果 Eigen 先计算并写入 `M(0,1) = M_old(1,0)`，然后在计算 `M(1,0)` 时，原始的 `M(0,1)` 已经被覆盖了——结果是转置后的矩阵不对称。Eigen 对这种特定情况有内部检测，但并非所有 aliasing 场景都能自动检测。更明确的做法是使用就地操作或显式求值：

```cpp
// 安全方案一：使用专门的就地转置函数
M.transposeInPlace();
```

```cpp
// 安全方案二：显式求值——先完整计算到临时对象，再赋值
M = M.transpose().eval();
```

### block aliasing

```cpp
v.segment<3>(0) = v.segment<3>(1);
```

左右区间重叠。

这种代码必须谨慎。

可以先求值：

```cpp
v.segment<3>(0) = v.segment<3>(1).eval();
```

### `noalias()`：告诉 Eigen 左右不重叠

默认情况下，Eigen 在矩阵乘法的赋值中会假设左右两边可能存在 aliasing——这意味着它会先把乘法结果写入一个临时矩阵，再拷贝到目标矩阵。这个额外拷贝在大多数情况下是不必要的（因为 `C = A * B` 中 `C`、`A`、`B` 通常是三个不同的矩阵），但 Eigen 必须保守处理以保证正确性。`noalias()` 是程序员给 Eigen 的显式承诺："我保证左边和右边不共享内存，你可以省掉临时对象"。这个承诺如果错误，行为是未定义的——所以只在确定没有 aliasing 时才使用。

有些矩阵乘法本来没有重叠风险，但 Eigen 为了保守处理 aliasing，可能生成临时对象。`noalias()` 用于告诉 Eigen：左侧目标与右侧表达式不共享底层数据，可以直接写入。

```cpp
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
Eigen::MatrixXd J(m, n);

H.noalias() += J.transpose() * J;
```

`noalias()` 不是“让代码更快的魔法开关”。它是一个承诺：右侧表达式读到的数据不会因为写左侧而被破坏。如果承诺是假的，结果就是未定义的数值错误。下面这种就不该随便写：

```cpp
A.noalias() = A * B;  // 左侧 A 同时出现在右侧
```

对 SLAM 后端，`H.noalias() += J.transpose() * W * J;` 是常见模式，因为 `H` 是累积 Hessian，`J` 和 `W` 来自当前残差块，内存不重叠。但在同一个矩阵内部搬移 block、转置或重排时，优先使用 `.eval()` 或专门的 in-place API。

### 表达式模板与 C++ 类型系统的深层关系

表达式模板不仅是一种性能优化技术，它还揭示了 C++ 模板元编程的一个核心能力：**把运行时计算的结构编码到编译期的类型系统中**。当你写 `A + B * C` 时，Eigen 在编译期构建了一棵由类型嵌套表示的计算树——`CwiseBinaryOp<sum, Matrix3d, Product<Matrix3d, Matrix3d>>` 这个类型本身就描述了"先乘后加"的计算结构。编译器在实例化赋值运算符模板时，看到的是一棵完全确定的、可以静态分析的类型树，而不是运行时的函数调用链。

这种"计算结构类型化"的思想在机器人 C++ 生态中广泛存在。Ceres 的自动微分用 `Jet<T, N>` 类型把"值 + N 个偏导数"编码到类型中，运算符重载让链式法则自动传播。Sophus 的李群用 `SE3<Scalar>` 类型把"旋转 + 平移"的群结构编码到类型中，`operator*` 表达群乘法。这些库的共同设计哲学是：**让类型系统替代运行时检查，让编译器替代程序员做正确性保证**。表达式模板是这种哲学最纯粹的体现——它用类型编码了计算的"什么"和"顺序"，把"何时"和"如何"的决定权交给编译器的优化器。

理解了这个深层联系，就能更好地理解为什么 Eigen 的错误信息如此冗长——因为类型名中编码了整棵表达式树的结构。一个简单的维度不匹配错误，类型名可能包含三四层嵌套的模板参数。这不是 Eigen 的设计缺陷，而是"计算结构类型化"的固有代价。

### 表达式接口的工程规则

可以用下面的规则减少大多数 Eigen 表达式模板错误：

| 目的 | 推荐写法 | 原因 |
|------|----------|------|
| 保存计算结果 | 显式矩阵类型或 `.eval()` | 固定求值时间点 |
| 只读函数参数 | `Ref<const T>` 或 `MatrixBase<Derived>` | 接收视图或表达式 |
| 可写函数参数 | `Ref<T>` | 明确非拥有输出视图 |
| 矩阵乘法累积且无重叠 | `noalias()` | 避免保守临时对象 |
| 左右可能重叠 | `.eval()` 或 in-place API | 避免覆盖未读取数据 |

### 表达式模板的工程决策准则

理解了表达式模板的工作机制后，工程中最常面临的问题不是"表达式模板怎么工作"，而是"我什么时候需要介入 Eigen 的自动决策"。大部分情况下，Eigen 的默认行为是最优的——直接写 `Matrix3d C = A + B + D;`，让 Eigen 自己决定何时求值。但有三种场景需要程序员显式介入：

**场景一：同一子表达式被多处使用。** 如果 `A * B` 在后续代码中出现两次——一次加 `D`，一次加 `E`——Eigen 的延迟求值会导致矩阵乘法被计算两次。此时应显式求值 `Matrix3d AB = (A * B).eval();`，把结果保存为具体矩阵，后续复用。判断标准是：如果子表达式的计算代价 > 一次内存拷贝的代价，就值得显式求值。对于 3x3 矩阵乘法（~45 次浮点运算），和拷贝 72 字节（9 个 double）相比，重复计算的代价远高于保存结果——应该显式求值。

**场景二：表达式结果需要跨越生命周期边界。** 函数返回值、保存到容器、传递给异步回调——这些场景中表达式节点内部的引用可能指向已经被销毁的局部变量。安全的做法是在生命周期边界处总是使用具体矩阵类型，而不是 `auto`。

**场景三：赋值的左右两边共享内存（aliasing）。** `M = M.transpose()` 这种自赋值需要特殊处理，因为延迟求值可能在写入某些元素时覆盖还没读取的其他元素。Eigen 对部分常见模式有内部检测，但不是所有 aliasing 都能自动处理。

### 表达式模板与调试

调试时，表达式类型可能非常长。

例如 `A * B + C` 的类型不是 `Matrix3d`。

断点里看类型会很复杂。

建议：

```cpp
const Eigen::Matrix3d predicted_cov =
    F * P * F.transpose() + Q;
```

在关键数学步骤处使用显式类型。

这不仅利于调试，也让文档和代码更接近公式。

### 常见陷阱

> ⚠️ **`auto` 陷阱：保存了表达式而不是值**
>
> 对 Eigen 表达式使用 `auto` 时，要问清楚自己想保存表达式还是计算结果。需要结果时使用显式类型或 `.eval()`。

> ⚠️ **重叠赋值陷阱：左右两边引用同一块内存**
>
> 转置、segment 移动、block 赋值都可能出现 aliasing。优先使用 Eigen 提供的 in-place API 或 `.eval()`。

> ⚠️ **可读性陷阱：一行公式太长**
>
> 表达式模板能写很长的一行，但教学和工程代码都应在关键中间量处命名，方便检查维度和物理意义。

### 练习

1. **实验题**：写 `auto expr = A + B; A.setZero();`，比较 `expr` 和 `(A+B).eval()` 的行为。
2. **重构题**：把一行很长的协方差预测公式拆成 `F`、`P_pred`、`Q` 等命名变量。
3. **分析题**：为什么 `M = M.transpose()` 比 `M.transposeInPlace()` 更危险？

---

## 11.7 内存布局、对齐与容器 ⭐⭐⭐

上一节解决了"Eigen 表达式什么时候求值"的问题。但表达式模板只是编译期的抽象——当数据真正被写入内存时，还有一个更底层的问题：数据在内存中以什么顺序排列？这直接影响与外部库交互时的正确性。

### 列主序与行主序

Eigen 默认列主序。

也就是说，矩阵元素按列连续存储。

```cpp
Eigen::Matrix<double, 2, 3> M;
M << 1, 2, 3,
     4, 5, 6;
```

逻辑矩阵是：

```text
1 2 3
4 5 6
```

但默认内存顺序按列：

```text
1 4 2 5 3 6
```

行主序类型：

```cpp
using RowMajorMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
```

图像、C 数组、某些外部库常使用行主序。

Eigen 内部数学计算默认列主序更常见。

### 对齐问题

Eigen 为了利用 SIMD，某些固定大小类型需要特定内存对齐。

现代 C++ 和新版 Eigen 已经改善了很多问题。

但当类中包含 Eigen 固定大小成员时，仍应理解边界。

示例：

```cpp
struct PoseWithCovariance {
    Eigen::Isometry3d pose;
    Eigen::Matrix<double, 6, 6> covariance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```

`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 用于确保动态分配对象时满足 Eigen 对齐要求。

在 C++17 以后，标准对 over-aligned new 支持更完善。

但跨平台库、老编译器和旧 ABI 项目中，仍能看到这个宏。

### Eigen 类型放入 STL 容器

常见写法：

```cpp
std::vector<Eigen::Vector3d> points;
```

对 `Vector3d` 通常问题不大。

对某些需要对齐的固定大小类型，历史上需要：

```cpp
std::vector<Eigen::Vector4d,
            Eigen::aligned_allocator<Eigen::Vector4d>> points;
```

项目中看到 `aligned_allocator` 时，不要急着删除。

要先确认：

1. C++ 标准版本。
2. Eigen 版本。
3. 编译器和平台。
4. 类型是否有特殊对齐要求。

### 避免过度暴露存储布局

算法代码最好依赖 Eigen 抽象，而不是依赖底层 `data()` 顺序。

只有在与外部库交互时才使用 `data()`：

```cpp
const double* raw = matrix.data();
```

使用前必须确认对方期待列主序还是行主序。

### 常见陷阱

> ⚠️ **布局陷阱：把 `data()` 交给外部 C API 前不确认顺序**
>
> Eigen 默认列主序。外部 API 如果按行主序解释，会得到转置式错误。

> ⚠️ **对齐陷阱：复制旧代码时删掉对齐宏**
>
> 某些项目保留对齐宏是为了兼容平台和编译选项。删除前要用目标环境验证。

> ⚠️ **容器陷阱：把对齐问题误判成随机数值错误**
>
> 对齐问题可能表现为崩溃、断言或只在某些优化级别出现。排查时要看类型、容器和编译标准。

### Eigen 3.4+ 新特性与工程影响 ⭐⭐⭐

Eigen 3.4（2021 年发布）是一次重要的版本升级，它引入了多项对机器人项目有实际影响的改进。如果你的项目仍在使用 Eigen 3.3，了解以下变化有助于评估是否需要升级，以及升级后需要注意哪些接口变化。

**C++17 对齐改进**：Eigen 3.4 利用了 C++17 的 over-aligned `operator new`。在 C++17 模式下编译时，如果类型的 `alignof` 超过默认对齐，编译器会自动调用对齐版本的 `new`。这意味着包含 `Vector4d`、`Quaterniond` 等需要 16 字节以上对齐的类型时，`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 在 C++17 模式下不再必需。但对于需要兼容 C++14 的项目，这个宏仍然是必要的安全网。

**索引类型统一**：Eigen 3.4 将内部索引类型从 `int` 改为 `Eigen::Index`（通常是 `std::ptrdiff_t`，64 位平台上为 `long`）。这修复了大型矩阵（行列数超过 `INT_MAX`）的潜在溢出问题。升级时需要检查所有与 Eigen 索引交互的代码——如果你的循环变量仍然使用 `int`，编译器可能会产生窄化转换警告。

**`Reshaped` 视图**：Eigen 3.4 新增了 `reshaped()` 方法，可以在不拷贝数据的情况下改变矩阵的逻辑形状。这在需要把状态向量"看作"矩阵（例如把 `VectorXd(9)` 当作 `Matrix3d` 处理）时很有用：

```cpp
// Eigen 3.4+：零拷贝 reshape
Eigen::VectorXd v(9);
auto M = v.reshaped(3, 3);  // 3x3 视图，不拷贝数据
```

**STL 迭代器支持**：Eigen 3.4 为向量和矩阵添加了 `begin()` / `end()` 迭代器，使 Eigen 对象可以直接用于范围 for 循环和 STL 算法。对向量，迭代器逐元素遍历；对矩阵，迭代器按存储顺序遍历所有元素。

> **本质洞察**：Eigen 的版本演进反映了 C++ 标准的演进——C++17 的对齐 `new` 让 Eigen 的对齐宏变得冗余，`std::ptrdiff_t` 索引类型修复了 `int` 的历史遗留问题。每次升级 Eigen 版本时，不仅要看新功能，还要检查已有代码中依赖旧行为的地方。

**与 CUDA 协作的注意事项**：在机器人深度学习和实时感知中，CUDA 代码越来越常见。Eigen 支持在 CUDA 设备代码中使用（需要定义 `EIGEN_CUDA_ARCH` 或使用 `__host__ __device__` 标注），但有几个关键限制：

| 限制 | 原因 | 工程影响 |
|------|------|----------|
| 动态大小矩阵不可在设备端使用 | `MatrixXd` 需要堆分配，CUDA 设备端无 `malloc` | 设备端只用固定大小类型 |
| 部分分解器不可用 | `JacobiSVD`、`ColPivHouseholderQR` 等使用了主机端 API | 设备端数值求解需用 cuBLAS/cuSOLVER |
| 对齐要求更严格 | CUDA 的 warp 访问需要 128 字节对齐 | 使用 `__align__` 属性或 Eigen 的对齐分配器 |
| 编译速度极慢 | 每个 CUDA 核函数中的 Eigen 模板都要完整实例化 | 把 Eigen 计算隔离到尽可能少的核函数中 |

实际项目中，最常见的做法是：在 CPU 端用 Eigen 做数学计算，用 `cudaMemcpy` 把结果传到 GPU 端；如果 GPU 端需要小矩阵运算（如逐像素的 3x3 旋转），使用 Eigen 的固定大小类型标注为 `__device__`。避免在 CUDA 核函数中做大规模的 Eigen 操作——那是 cuBLAS 和 cuSOLVER 的工作。

### 练习

1. **布局题**：打印一个 2x3 矩阵的 `data()`，观察默认列主序。
2. **映射题**：把同一段行主序数组分别映射为默认矩阵和 RowMajor 矩阵，比较结果。
3. **工程题**：查一个项目中 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 的使用位置，判断它保护的是什么类型。
4. **版本题**：查阅你当前项目使用的 Eigen 版本号（`EIGEN_WORLD_VERSION` / `EIGEN_MAJOR_VERSION`），判断是否需要对齐宏。

---

## 11.8 线性求解：不要轻易求逆 ⭐⭐⭐⭐

### 动机：求解线性方程不是求逆矩阵

很多公式写成：

```text
x = A^{-1} b
```

但代码不应直接写：

```cpp
Eigen::VectorXd x = A.inverse() * b;
```

原因有三点：

1. 显式求逆通常更慢。
2. 数值误差更大。
3. 矩阵结构信息没有被利用。

更好的写法：

```cpp
Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
```

### 常见分解

| 分解 | Eigen API | 适合场景 |
|------|-----------|----------|
| LLT | `llt()` | 对称正定，速度快 |
| LDLT | `ldlt()` | 对称半正定或更稳一些 |
| QR | `colPivHouseholderQr()` | 一般最小二乘，稳定 |
| SVD | `jacobiSvd()` | 秩亏、点云配准、伪逆 |
| Sparse Cholesky | `SimplicialLDLT` | 稀疏正定系统 |

### SLAM 后端中的正规方程

非线性最小二乘线性化后常得到：

```text
H delta = -g
H = J^T J
g = J^T r
```

如果问题约束充分，`H` 通常是对称正定或半正定。

小问题可以：

```cpp
Eigen::VectorXd delta = H.ldlt().solve(-g);
```

大规模图优化通常不会直接用 dense `MatrixXd`。

会使用稀疏矩阵和专门求解器。

### 正规方程从哪里来

最小二乘的基本问题是：

```text
minimize_x  1/2 ||A x - b||^2
```

定义残差：

```text
r(x) = A x - b
```

目标函数：

```text
J(x) = 1/2 r(x)^T r(x)
     = 1/2 (A x - b)^T (A x - b)
```

把它展开：

```text
J(x)
= 1/2 (x^T A^T - b^T)(A x - b)
= 1/2 (x^T A^T A x - x^T A^T b - b^T A x + b^T b)
```

因为 `x^T A^T b` 是标量，它等于自己的转置：

```text
x^T A^T b = b^T A x
```

所以：

```text
J(x) = 1/2 x^T A^T A x - x^T A^T b + 1/2 b^T b
```

对 $\mathbf{x}$ 求梯度：

$$
\nabla J(\mathbf{x}) = \mathbf{A}^T \mathbf{A} \mathbf{x} - \mathbf{A}^T \mathbf{b}
$$

最优点的一阶条件是梯度为 0：

```text
A^T A x = A^T b
```

这就是正规方程。

在非线性最小二乘中，当前估计附近有：

$$
\mathbf{r}(\mathbf{x} + \boldsymbol{\delta}) \approx \mathbf{r}(\mathbf{x}) + \mathbf{J} \boldsymbol{\delta}
$$

于是求：

```text
minimize_delta 1/2 ||J delta + r||^2
```

套用上面的结果：

```text
J^T J delta = -J^T r
```

因此：

```text
H = J^T J
g = J^T r
H delta = -g
```

这就是 SLAM 后端中 `H.ldlt().solve(-g)` 的来历。

### 正规方程的代价：条件数会平方

正规方程很常见，但它不是无条件最稳定的方式。如果原始最小二乘矩阵是 `J`，正规方程使用：

```text
H = J^T J
```

它的条件数满足近似关系：

```text
cond(H) = cond(J^T J) = cond(J)^2
```

这意味着如果 `J` 已经有某些方向约束很弱，构造 `J^T J` 会进一步放大病态性。小型、约束充分、近似正定的 SLAM 子问题用 `LDLT` 很常见；但如果残差几何可能退化，或者你正在调试新因子，QR/SVD 往往能提供更可靠的诊断。

对同一个问题，可以有三条计算路径：

```cpp
Eigen::VectorXd solveByNormalEquation(const Eigen::MatrixXd& J,
                                      const Eigen::VectorXd& r) {
    const Eigen::MatrixXd H = J.transpose() * J;
    const Eigen::VectorXd g = J.transpose() * r;
    return H.ldlt().solve(-g);
}

Eigen::VectorXd solveByQr(const Eigen::MatrixXd& J,
                          const Eigen::VectorXd& r) {
    return J.colPivHouseholderQr().solve(-r);
}

Eigen::VectorXd solveBySvd(const Eigen::MatrixXd& J,
                           const Eigen::VectorXd& r) {
    return J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
        .solve(-r);
}
```

工程上常用的策略是：正常路径用结构化求解器，诊断路径保留 QR/SVD，对异常场景输出秩、奇异值和残差变化。这样性能路径和调试路径都清楚。

### 为什么不直接求逆

从公式看：

```text
delta = -H^{-1} g
```

但代码中不应写：

```cpp
delta = -H.inverse() * g;
```

原因不是“大家习惯不用逆”，而是数值过程不同。

显式求逆需要先构造整个 `H^{-1}`，再乘 `g`。但求解线性方程只需要找到满足 `H delta = -g` 的 `delta`。分解法会利用矩阵结构，避免形成完整逆矩阵。

对称正定矩阵的 LDLT 分解可以理解为：

```text
H = L D L^T
```

求解：

```text
L D L^T delta = -g
```

分三步：

```text
L y = -g
D z = y
L^T delta = z
```

这三步是三角系统和对角系统求解，比显式求逆更稳定、更快，也更能利用对称结构。

> **本质洞察**：公式里的逆表示“解线性方程的数学关系”，不是“代码里必须构造逆矩阵”。数值计算的核心问题是稳定地求解方程，而不是把纸面公式逐字符翻译成 API。

### ICP 中的 SVD

点到点 ICP 的一步可以用 SVD 求旋转。

步骤：

1. 计算源点和目标点质心。
2. 去中心化。
3. 计算相关矩阵 `W`。
4. 对 `W` 做 SVD。
5. 得到 `R = V U^T`。
6. 修正反射情况。
7. 得到 `t = mu_target - R mu_source`。

代码骨架：

```cpp
Eigen::Matrix3d solveRotationSvd(const Eigen::MatrixXd& source,
                                 const Eigen::MatrixXd& target) {
    const Eigen::Vector3d mu_s = source.rowwise().mean();
    const Eigen::Vector3d mu_t = target.rowwise().mean();

    Eigen::MatrixXd src_centered = source.colwise() - mu_s;
    Eigen::MatrixXd tgt_centered = target.colwise() - mu_t;

    Eigen::Matrix3d W = src_centered * tgt_centered.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        W,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();

    if (R.determinant() < 0.0) {
        V.col(2) *= -1.0;
        R = V * U.transpose();
    }

    return R;
}
```

这里的 `source` 和 `target` 是 `3xN` 矩阵。

每一列是一个点。

这和很多点云库的点数组不同。

进入函数前要明确数据布局。

### SVD-ICP 的推导

点到点配准要解：

$$
\min_{\mathbf{R},\,\mathbf{t}} \sum_{i} \|\mathbf{R}\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i\|^2, \quad \text{s.t.}\; \mathbf{R}^T\mathbf{R} = \mathbf{I},\; \det(\mathbf{R})=1
$$

第一步先消掉平移。

对固定的 `R`，令目标函数对 `t` 的梯度为 0：

$$
\sum_{i} (\mathbf{R}\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i) = \mathbf{0}
$$

得到：

$$
N\,\mathbf{t} = \sum_{i}\mathbf{q}_i - \mathbf{R}\sum_{i}\mathbf{p}_i
$$

$$
\mathbf{t} = \boldsymbol{\mu}_q - \mathbf{R}\,\boldsymbol{\mu}_p
$$

也就是说，最优平移一定把源点质心变到目标点质心。

定义去中心化点：

$$
\mathbf{p}'_i = \mathbf{p}_i - \boldsymbol{\mu}_p, \quad \mathbf{q}'_i = \mathbf{q}_i - \boldsymbol{\mu}_q
$$

问题变成：

$$
\min_{\mathbf{R}} \sum_{i} \|\mathbf{R}\mathbf{p}'_i - \mathbf{q}'_i\|^2
$$

展开单项：

```text
||R p'_i - q'_i||^2
= (R p'_i - q'_i)^T (R p'_i - q'_i)
= p_i'^T R^T R p'_i - 2 q_i'^T R p'_i + q_i'^T q'_i
```

因为 `R^T R = I`：

```text
= ||p'_i||^2 + ||q'_i||^2 - 2 q_i'^T R p'_i
```

前两项与 `R` 无关。

因此最小化距离等价于最大化：

$$
\sum_{i} \mathbf{q}_i'^{\,T} \mathbf{R}\,\mathbf{p}'_i
$$

把相关矩阵定义为：

$$
\mathbf{W} = \sum_{i} \mathbf{p}'_i \,\mathbf{q}_i'^{\,T}
$$

可以把目标写成：

```text
maximize_R trace(R W)
```

对 `W` 做 SVD：

$$
\mathbf{W} = \mathbf{U}\,\boldsymbol{\Sigma}\,\mathbf{V}^T
$$

最优旋转为：

```text
R = V U^T
```

如果 `det(R) < 0`，说明出现了反射，需要翻转 `V` 的最后一列再计算。

这就是代码中：

```cpp
if (R.determinant() < 0.0) {
    V.col(2) *= -1.0;
    R = V * U.transpose();
}
```

的数学来源。

如果没有这一步，点集在某些退化或带噪情况下可能得到一个行列式为 -1 的“镜像变换”，它不是合法旋转。

### 条件数与退化

线性系统可能退化。

例如，所有点都在一条直线上，ICP 的某些旋转自由度就难以确定。

SVD 的奇异值能帮助判断退化：

```cpp
auto singular_values = svd.singularValues();
```

如果最小奇异值接近零，说明某些方向约束很弱。

这时不要盲目相信结果。

应输出诊断、增加先验或拒绝本次更新。

退化诊断不应只看“求解器是否返回 Success”。求解器完成分解，只表示数值过程没有在 API 层失败；它不保证物理约束充分。一个更接近工程需要的诊断结构如下：

```cpp
struct SolveDiagnostics {
    bool success = false;
    int rank = 0;
    double min_singular = 0.0;
    double max_singular = 0.0;
    double condition = std::numeric_limits<double>::infinity();
    double residual_norm = 0.0;
};
```

对 ICP 或小型最小二乘，可以直接用 SVD 填充：

```cpp
SolveDiagnostics diagnoseLeastSquares(const Eigen::MatrixXd& J,
                                      const Eigen::VectorXd& r,
                                      double eps) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J,
        Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto s = svd.singularValues();
    SolveDiagnostics d;
    d.success = svd.info() == Eigen::Success;
    d.max_singular = s.size() > 0 ? s(0) : 0.0;
    d.min_singular = s.size() > 0 ? s(s.size() - 1) : 0.0;
    d.rank = static_cast<int>((s.array() > eps).count());
    if (d.min_singular > eps) {
        d.condition = d.max_singular / d.min_singular;
    }
    d.residual_norm = r.norm();
    return d;
}
```

阈值 `eps` 不是固定真理。它应与数据尺度、噪声模型和单位一致。点云以米为单位、像素重投影以像素为单位、IMU 预积分混合米/米每秒/弧度，不能共用一个无脑阈值。实际项目常把诊断量记录到日志中，通过回放数据观察健康范围，再决定拒绝更新或增加先验的条件。

对 SVD-ICP，退化还可以从点集几何解释：

| 点集形状 | 退化方向 |
|----------|----------|
| 所有点几乎重合 | 平移和旋转都弱 |
| 所有点近似共线 | 绕直线方向的旋转弱 |
| 所有点近似共面 | 法向相关方向更敏感，点到面 ICP 尤其明显 |
| 视野太窄 | 某些平移/旋转耦合强，条件数变差 |

这类诊断应该出现在模块边界，而不是散落在求解调用之后。否则后端会在“求解成功但更新错误”的状态下继续运行，最后表现为轨迹慢慢漂移或突然跳变。

### 常见陷阱

> ⚠️ **求逆陷阱：写 `A.inverse() * b`**
>
> 公式中的逆不等于代码必须求逆。优先使用分解的 `solve()`。

> ⚠️ **结构陷阱：没有利用矩阵性质**
>
> 对称正定系统用通用逆会浪费结构。选择求解器前先判断矩阵是否对称、正定、稀疏。

> ⚠️ **退化陷阱：求解器返回结果就直接使用**
>
> 点云几何退化、约束不足、尺度不一致都可能让解不可靠。奇异值、残差和条件数要进入诊断。

### 练习

1. **改写题**：把 `A.inverse() * b` 改成 QR、LDLT、SVD 三种求解方式，并说明适用条件。
2. **ICP 题**：实现点到点 ICP 的 SVD 求解，并构造共线点集观察退化。
3. **诊断题**：给定 Hessian 的特征值，判断系统是否约束充分。

---

## 11.9 工程案例：从 Ceres 参数块到 ESKF 状态更新 ⭐⭐⭐⭐

### 案例目标

本节把本章知识串成一条工程链：

```text
原始参数指针
  ↓ Map
Eigen 向量
  ↓ block / segment
误差状态
  ↓ Quaternion / Isometry
状态注入
  ↓ solve
线性更新
```

### 参数块布局

假设优化参数是：

```text
pose[0:3]  平移
pose[3:7]  四元数，顺序 x y z w
```

Ceres 残差接口：

```cpp
template <typename T>
bool operator()(const T* const pose,
                const T* const point_lidar,
                T* residual) const;
```

映射：

```cpp
template <typename T>
bool operator()(const T* const pose,
                const T* const point_lidar,
                T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose);

    Eigen::Quaternion<T> q(
        pose[6],
        pose[3],
        pose[4],
        pose[5]);
    q.normalize();

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_lidar(point_lidar);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residual);

    const Eigen::Matrix<T, 3, 1> p_world = q * p_lidar + t;
    r = p_world - measured_world_.cast<T>();
    return true;
}
```

注意四元数参数块顺序是 `[x,y,z,w]`。

Eigen 构造函数仍是 `(w,x,y,z)`。

### ESKF 更新

假设线性系统已经得到误差状态：

```cpp
Eigen::Matrix<double, 15, 1> dx =
    H.ldlt().solve(-g);
```

状态注入：

```cpp
injectError(state, dx);
```

协方差更新：

```cpp
Eigen::Matrix<double, 15, 15> I =
    Eigen::Matrix<double, 15, 15>::Identity();

P = (I - K * H_obs) * P * (I - K * H_obs).transpose()
    + K * R * K.transpose();
```

这里使用 Joseph 形式。

它比简单的 `P = (I - K H) P` 更能保持数值对称性和半正定性。

实际代码中建议：

```cpp
P = 0.5 * (P + P.transpose());
```

用于清理浮点误差引入的轻微非对称。

### 代码组织建议

| 文件 | 职责 |
|------|------|
| `state_layout.hpp` | 状态维度和偏移 |
| `nav_state.hpp` | 状态结构 |
| `ceres_eigen_utils.hpp` | `Map` 和四元数转换 |
| `eskf_update.cpp` | 状态注入和协方差更新 |
| `linear_solve.cpp` | 求解器选择与诊断 |

不要把所有 Eigen 操作堆在一个函数里。

分层能让错误更容易定位。

### 最小测试

测试应该覆盖：

| 测试 | 目的 |
|------|------|
| 四元数顺序转换 | 防止 `[x,y,z,w]` 与 `(w,x,y,z)` 混淆 |
| `segment` 偏移 | 防止状态块错位 |
| `Map` 修改原数组 | 确认零拷贝语义 |
| `auto` 表达式 | 防止保存惰性表达式 |
| SVD ICP | 验证旋转恢复 |
| 求解器退化 | 验证诊断路径 |

### 常见陷阱

> ⚠️ **布局陷阱：参数块顺序没有写成测试**
>
> 四元数顺序错误可能让优化结果看起来“差一点”，但实际是坐标变换完全错了。

> ⚠️ **状态陷阱：误差状态偏移和协方差块偏移不一致**
>
> 状态布局应有唯一来源。不要在多个文件里重复写数字。

> ⚠️ **数值陷阱：线性求解后不看残差和退化**
>
> 求解成功不等于物理正确。残差、奇异值、条件数和更新量大小都应进入诊断。

### 练习

1. **参数块题**：为 `[tx,ty,tz,qx,qy,qz,qw]` 写 Eigen 映射函数，并返回 `Isometry3d`。
2. **状态题**：实现 15 维误差状态注入，并写测试验证每个 segment 修改了正确字段。
3. **求解题**：构造一个近奇异 Hessian，比较 LDLT 和 SVD 的行为。
4. **[跨章综合题]**：综合 类型系统与值类别推导-Lambda与STL算法 的对象生命周期、引用和移动语义知识，分析以下代码的问题：一个函数返回 `Eigen::Map<Vector3d>(local_array)`，其中 `local_array` 是函数内的局部 `double[3]`。解释为什么返回的 `Map` 对象是悬垂引用，给出两种修复方案（返回 `Vector3d` 值 vs 接受外部缓冲区指针），并讨论各自的性能权衡。

---

## 11.10 累积项目：Mini Eigen SLAM Toolkit ⭐⭐⭐⭐

### 项目目标

本章项目实现一个小型 Eigen 工具包。

它不是完整 SLAM 系统。

它的目标是把本章关键边界变成可测试代码。

项目的核心不是“把示例函数收进一个目录”，而是把 Eigen 工程边界拆成可验证模块。每个模块只守住一个不变量：

| 模块 | 守住的不变量 |
|------|--------------|
| 状态布局 | offset 与维度唯一来源 |
| 状态注入 | 线性状态加法、姿态扰动、四元数归一化 |
| 变换工具 | SO(3)/SE(3) 合法性和方向约定 |
| 映射工具 | 原始参数块顺序、零拷贝和生命周期边界 |
| 线性求解 | 尺寸检查、分解选择和退化诊断 |
| ICP SVD | 质心消元、旋转投影和反射修正 |

目录结构：

```text
mini_eigen_slam/
├── include/
│   ├── state_layout.hpp
│   ├── nav_state.hpp
│   ├── eigen_map_utils.hpp
│   ├── rigid_transform.hpp
│   └── linear_solve.hpp
├── src/
│   └── icp_svd.cpp
└── tests/
    ├── quaternion_order_test.cpp
    ├── state_segment_test.cpp
    ├── map_lifetime_test.cpp
    └── icp_svd_test.cpp
```

### `state_layout.hpp`

```cpp
#pragma once

#include <Eigen/Dense>

namespace mini_slam {

struct ErrorStateLayout {
    static constexpr int kPos = 0;
    static constexpr int kVel = 3;
    static constexpr int kTheta = 6;
    static constexpr int kGyroBias = 9;
    static constexpr int kAccelBias = 12;
    static constexpr int kDim = 15;
};

using ErrorState = Eigen::Matrix<double, ErrorStateLayout::kDim, 1>;
using Covariance = Eigen::Matrix<double,
                                 ErrorStateLayout::kDim,
                                 ErrorStateLayout::kDim>;

}  // namespace mini_slam
```

### `nav_state.hpp`

```cpp
#pragma once

#include <Eigen/Dense>

namespace mini_slam {

struct NavState {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
};

}  // namespace mini_slam
```

### `rigid_transform.hpp`

```cpp
#pragma once

#include "nav_state.hpp"
#include "state_layout.hpp"

namespace mini_slam {

inline Eigen::Quaterniond smallAngleQuaternion(
    const Eigen::Vector3d& dtheta) {
    const double angle = dtheta.norm();
    if (angle < 1e-12) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, dtheta / angle));
}

inline void injectError(NavState& state, const ErrorState& dx) {
    state.p += dx.segment<3>(ErrorStateLayout::kPos);
    state.v += dx.segment<3>(ErrorStateLayout::kVel);

    const Eigen::Vector3d dtheta =
        dx.segment<3>(ErrorStateLayout::kTheta);
    state.q = (smallAngleQuaternion(dtheta) * state.q).normalized();

    state.bg += dx.segment<3>(ErrorStateLayout::kGyroBias);
    state.ba += dx.segment<3>(ErrorStateLayout::kAccelBias);
}

inline Eigen::Isometry3d toIsometry(const NavState& state) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = state.q.toRotationMatrix();
    T.translation() = state.p;
    return T;
}

}  // namespace mini_slam
```

### `eigen_map_utils.hpp`

```cpp
#pragma once

#include <Eigen/Dense>

namespace mini_slam {

inline Eigen::Isometry3d poseArrayXyzwToIsometry(const double* pose) {
    Eigen::Map<const Eigen::Vector3d> t(pose);
    Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
    q.normalize();

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = t;
    return T;
}

inline void isometryToPoseArrayXyzw(const Eigen::Isometry3d& T,
                                    double* pose) {
    Eigen::Map<Eigen::Vector3d> t(pose);
    t = T.translation();

    Eigen::Quaterniond q(T.rotation());
    q.normalize();
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();
}

}  // namespace mini_slam
```

### `linear_solve.hpp`

```cpp
#pragma once

#include <Eigen/Dense>
#include <stdexcept>

namespace mini_slam {

inline Eigen::VectorXd solveSymmetricSystem(const Eigen::MatrixXd& H,
                                            const Eigen::VectorXd& g) {
    if (H.rows() != H.cols()) {
        throw std::invalid_argument("H must be square");
    }
    if (H.rows() != g.rows()) {
        throw std::invalid_argument("H and g dimension mismatch");
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() != Eigen::Success) {
        throw std::runtime_error("LDLT decomposition failed");
    }
    return ldlt.solve(-g);
}

}  // namespace mini_slam
```

### 项目检查

完成后至少验证：

1. `ErrorStateLayout::kDim == 15`，每个 offset 等于前一块 offset 加块维度。
2. `poseArrayXyzwToIsometry()` 对 `[0,0,0,0,0,0,1]` 返回单位旋转，且对非单位四元数会归一化。
3. `isometryToPoseArrayXyzw()` 输出顺序严格是 `[tx,ty,tz,qx,qy,qz,qw]`。
4. `injectError()` 的位置、速度、陀螺零偏、加计零偏 segment 都写入正确字段。
5. 小角度姿态注入后四元数仍满足 `abs(norm - 1) < 1e-12`。
6. 左乘扰动和右乘扰动在非单位初始旋转上产生不同结果，测试名称必须写清约定。
7. `Map` 写入 pose 数组后原始数组同步变化，返回值函数不得返回悬垂 `Map`。
8. 行主序 buffer 用默认 `Map<Matrix3d>` 与 `Map<RowMajorMatrix3d>` 得到不同解释，测试要显式展示差异。
9. `solveSymmetricSystem()` 对非方阵和维度不匹配输入抛出异常。
10. 近奇异系统能输出诊断量，测试不应只检查“求解器没有报错”。
11. SVD ICP 能恢复一个已知旋转和平移。
12. 共线点集会触发退化诊断，不能静默返回“可信更新”。

---

## 本章小结

本章围绕 Eigen 的工程边界展开：

| 主题 | 核心判断 |
|------|----------|
| Matrix/Vector | 维度是接口契约，固定大小优先表达固定数学结构 |
| 固定/动态维度 | 固定维度把尺寸放进类型，动态维度需要运行期检查 |
| Block/Segment | 状态向量有结构，offset 应集中成单一事实源 |
| Map/Ref/Stride | 零拷贝视图不拥有内存，必须确认生命周期、行列主序和 stride |
| Quaternion | 构造顺序与存储顺序不同，单位四元数是不变量 |
| SO(3)/SE(3) | 旋转不是向量，扰动要通过 Exp/角轴进入流形 |
| Isometry | 表示刚体变换语义，比裸 `Matrix4d` 更清晰 |
| 表达式模板 | 表达式描述计算，具体矩阵保存结果；`auto`、`.eval()`、`noalias()` 要按边界使用 |
| 对齐与布局 | 与外部接口交互前确认行列主序、对齐和容器分配行为 |
| 线性求解 | 用分解的 `solve()`，不要轻易显式求逆 |
| 正规方程 | 推导来自最小二乘一阶条件，但会平方条件数 |
| SVD | 适合退化分析和 ICP 旋转估计 |

最重要的经验是：Eigen 代码要同时读懂数学、类型和内存。

只看公式，会忽略生命周期和布局。

只看 C++，会忽略坐标系和数值稳定性。

高质量机器人代码必须让这三者一致。

---

## 延伸阅读

| 资源 | 重点 |
|------|------|
| Eigen 官方文档 | Matrix、Map、Geometry、Decompositions |
| 《视觉 SLAM 十四讲》 | 三维刚体运动与 Eigen 入门 |
| Ceres 官方教程 | 参数块、自动微分和 Eigen 映射 |
| GTSAM 文档 | 因子图中的向量、矩阵和李群局部坐标 |
| PCL 文档 | 点类型、协方差、SVD 和 PCA |
| FAST-LIO2 代码 | ESKF 状态、协方差和 Eigen block 使用 |

---

## 🔧 故障排查手册

| 现象 | 可能原因 | 检查路径 |
|------|----------|----------|
| 四元数旋转方向完全错 | `[x,y,z,w]` 与 `(w,x,y,z)` 混淆 | 打印 `q.w()` 和 `q.coeffs()` |
| 优化结果逐渐发散 | 四元数未归一化或扰动方向混用 | 检查 `dq*q` 与 `q*dq` |
| `Map` 后数据异常 | 生命周期或行列主序错误 | 检查原始 buffer 和 `Map` 类型 |
| Release 才崩溃 | 对齐、生命周期或未初始化 | 打开断言和 sanitizers |
| 矩阵乘法维度报错 | 固定大小尺寸不匹配 | 从类型名读 Rows 和 Cols |
| 求解器返回不稳定结果 | Hessian 退化或条件数差 | 看奇异值、残差和更新量 |
| `auto` 结果随原矩阵变化 | 保存了表达式 | 使用显式矩阵类型或 `.eval()` |
| 点云矩阵结果转置 | 点按列还是按行组织不一致 | 明确 `3xN` 或 `Nx3` 约定 |
