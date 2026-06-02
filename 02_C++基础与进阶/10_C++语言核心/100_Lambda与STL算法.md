# Lambda 表达式与 STL 算法深入

> **难度**：⭐⭐～⭐⭐⭐⭐ | **建议用时**：2周 | **前置要求**：继承与多态深入、错误处理与异常安全、运算符重载

---

## 前置自测

> 📋 答不出 >= 2 题时，先回顾 继承与多态深入-运算符重载 的对应内容。

1. 继承与多态深入 中虚函数和函数重载有什么区别？哪个发生在运行时，哪个发生在编译期？
2. 运算符重载 中 `operator()` 为什么能让对象像函数一样被调用？Lambda 和它有什么关系？
3. 如果一个异步任务捕获局部变量引用，而任务在函数返回后才执行，会发生什么？
4. `std::vector`、`std::deque`、`std::list` 的内存布局和典型使用场景分别是什么？
5. `std::function<void(const PointCloud&)>` 能保存哪些可调用对象？它为什么可能比直接模板参数更重？

---

## 本章目标

学完本章，你将能够：

- 区分函数重载、默认参数、函数指针、成员函数指针、虚函数、函数对象、Lambda 和 `std::function` 的适用场景。
- 熟练使用 Lambda 的捕获列表、参数、`mutable`、返回类型、泛型 Lambda 和初始化捕获。
- 理解 Lambda 的本质是匿名函数对象，从而预测它的大小、生命周期和性能边界。
- 判断引用捕获、值捕获、`this` 捕获、移动捕获在同步和异步场景中的安全性。
- 正确使用 `std::function` 存储回调，同时知道类型擦除、小对象优化和堆分配的实现边界。
- 用 STL 算法表达点云过滤、变换、归约、排序和分区，避免手写循环中的越界和迭代器失效。
- 理解 range-for 的展开规则、容器选型、结构化绑定、`optional`、`variant`、`span`、`if constexpr` 和并行 STL 的工程价值。
- 了解 C++23 对 Lambda 的三项重要改进：`static operator()`、deducing this 和 `std::ranges` 适配器与 Lambda 的配合。
- 为 Eigen基础与SLAM数学预备 Eigen 批量点云处理和 函数模板与类模板基础 模板编程建立可调用对象与泛型算法基础。

**本章在课程中的位置**：继承与多态深入 讲运行时多态，适合插件和算法模块边界；运算符重载 讲 `operator()`，它让对象可以像函数一样调用。本章把这两条线连接起来：Lambda 是编译器生成的匿名函数对象，STL 算法则把”遍历容器并对元素执行操作”抽象成统一接口。机器人代码中的 ROS2 回调、TBB 并行点云处理、配置过滤、轨迹排序和事件系统都会用到这些能力。

## 知识树

本章涉及的知识点形成以下树状结构。树的根是”C++ 如何表达可调用行为”，主干是 Lambda 的对象模型，分支是 STL 算法和容器工具。

```text
C++ 可调用机制
├── 函数指针（C 风格，无状态） ⭐⭐
├── 成员函数指针与 std::bind ⭐⭐⭐
├── Lambda 表达式 ⭐⭐⭐（核心主干）
│   ├── 闭包类型与 operator() ⭐⭐
│   ├── 捕获模式（值/引用/移动/mutable） ⭐⭐⭐⭐
│   ├── 泛型 Lambda（auto 参数） ⭐⭐⭐
│   ├── C++23 static operator() / deducing this ⭐⭐⭐⭐
│   └── 与 std::function 的关系 ⭐⭐⭐
├── STL 算法 ⭐⭐⭐
│   ├── 过滤（remove_if + erase） ⭐⭐
│   ├── 变换（transform） ⭐⭐
│   ├── 归约（accumulate） ⭐⭐
│   ├── 排序与分区 ⭐⭐
│   └── 并行执行策略 ⭐⭐⭐
└── 容器与工具 ⭐⭐⭐
    ├── range-for 与迭代器类别 ⭐⭐
    ├── optional / variant / span ⭐⭐⭐
    └── 容器选型（vector/deque/unordered_map） ⭐⭐
```

---

## 10.1 函数重载、默认参数与函数指针：Lambda 出现前的回调工具 ⭐⭐

### 动机：同一个操作名服务不同数据类型

先从函数重载开始。点到点距离可以有二维、三维、带协方差等版本：

```cpp
double distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (a - b).norm();
}

double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return (a - b).norm();
}

double distance(const PointXYZI& a, const PointXYZI& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
```

编译器根据实参类型在编译期选择最佳匹配。这不是 继承与多态深入 的运行时多态：没有虚函数表，也不依赖对象动态类型。

| 机制 | 决策时机 | 典型例子 |
|------|----------|----------|
| 函数重载 | 编译期 | `distance(Vector2d, Vector2d)` vs `distance(Vector3d, Vector3d)` |
| 虚函数 | 运行时 | `registration->align()` 调用具体派生类 |
| 函数模板 | 编译期实例化 | `transform<PointT>()` |
| Lambda | 编译期生成匿名类型 | STL 算法和回调 |

理解这个区别很重要：重载解决”同名函数适配不同静态类型”，虚函数解决”同一接口调用不同动态实现”。

> **本质洞察**：C++ 的可调用机制从函数指针到 Lambda 再到 `std::function`，本质上是在”静态类型确定性”和”运行时灵活性”之间的权衡。
> 函数指针类型固定但无法携带状态；Lambda 类型由编译器生成，可内联但每个表达式类型唯一；`std::function` 通过类型擦除统一接口，但丢失了具体类型信息。
> 选择哪种机制，取决于”变化点在编译期还是运行期确定”。

### 默认参数：接口便利，不是动态配置

默认参数可以减少调用端噪声：

```cpp
PointCloud filterByRange(const PointCloud& cloud,
                         double max_range = 80.0,
                         double min_range = 1.0);
```

调用：

```cpp
auto filtered = filterByRange(cloud);
```

默认参数只应在声明处指定一次，通常放在头文件声明中。不要在声明和定义中重复写默认值。

```cpp
// point_filter.hpp
PointCloud filterByRange(const PointCloud& cloud,
                         double max_range = 80.0,
                         double min_range = 1.0);

// point_filter.cpp
PointCloud filterByRange(const PointCloud& cloud,
                         double max_range,
                         double min_range) {
    // ...
}
```

回顾 继承与多态深入：虚函数默认参数静态绑定，不跟随动态派发。因此多态接口中默认参数要谨慎。普通自由函数的默认参数则相对直接。

### 函数指针：C 风格回调

函数有地址，可以用指针保存：

```cpp
using PointPredicate = bool (*)(const PointXYZI&);

bool isNear(const PointXYZI& p) {
    return p.x * p.x + p.y * p.y + p.z * p.z < 100.0;
}

std::vector<PointXYZI> filter(const std::vector<PointXYZI>& points,
                              PointPredicate predicate) {
    std::vector<PointXYZI> output;

    for (const auto& p : points) {
        if (predicate(p)) {
            output.push_back(p);
        }
    }

    return output;
}
```

函数指针简单、稳定，常用于 C 接口。但它不能捕获上下文。如果过滤阈值来自配置，就只能用全局变量、额外参数或结构体包装。

这个限制揭示了函数指针的根本缺陷：**它只是一个地址，没有任何附带的数据**。想象你雇了一个工人（函数指针），告诉他"按标准检查产品"。但"标准"是什么？如果标准写在他随身携带的工具箱里（Lambda 捕获的变量），他随时可以查阅。如果标准贴在工厂墙上（全局变量），所有工人共享同一份标准——但如果两条生产线需要不同标准就麻烦了。函数指针就像一个没有工具箱的工人——他能执行固定的操作，但无法根据上下文调整行为。这个问题在 C++11 引入 Lambda 之前困扰了 C++ 社区近 30 年，也是 `std::bind` 和函数对象存在的根本原因。

### 成员函数指针：需要对象

成员函数需要 `this`：

```cpp
class PointFilter {
public:
    bool isNear(const PointXYZI& p) const;
};

using MemberPredicate = bool (PointFilter::*)(const PointXYZI&) const;

PointFilter filter;
MemberPredicate pred = &PointFilter::isNear;

bool ok = (filter.*pred)(point);
```

成员函数指针语法较重。这也是 `std::bind` 和 Lambda 在 ROS2 回调中流行的原因：它们把“成员函数 + 对象”绑定成普通可调用对象。

### 常见陷阱

> ⚠️ **概念陷阱：函数重载不是多态**
>
> 重载在编译期根据静态类型选择函数。虚函数在运行时根据动态类型选择函数。两者都可以让调用代码看起来相似，但机制完全不同。

> ⚠️ **设计陷阱：用函数指针承载有状态回调**
>
> 函数指针不能捕获状态。需要阈值、配置、节点对象或统计信息时，Lambda、函数对象或 `std::function` 更自然。

### 练习

1. **代码题**：为 `distance()` 写二维、三维、`PointXYZI` 三个重载，并说明编译器如何选择。
2. **回调题**：用函数指针实现点云过滤，然后尝试加入运行时阈值。记录你需要额外引入什么状态传递方式。
3. **分析题**：为什么成员函数指针必须绑定对象才能调用？

函数指针和成员函数指针能表达回调，但写法不够自然。下一节看 `std::bind` 和 Lambda 如何改善这一点。

---

## 10.2 `std::bind`、成员回调与 ROS2 历史写法 ⭐⭐⭐

`std::bind` 是 C++11 引入的函数适配器。它的核心能力是"预先绑定一个可调用对象的部分参数"，生成一个新的可调用对象。在 ROS2 节点中，`std::bind` 最常见的用途是把成员函数和 `this` 指针绑定在一起——因为成员函数需要 `this` 才能调用，但订阅回调系统期望的是一个普通的可调用对象。

虽然 Lambda 在大多数场景下已经取代了 `std::bind`，但理解 `std::bind` 仍然有价值：大量现有 ROS2 代码和教程仍在使用它，而且它的"占位符"概念（`std::placeholders::_1`）有助于理解参数绑定的一般机制。

### 动机：订阅回调需要绑定对象

ROS2 节点中常见写法：

```cpp
sub_ = create_subscription<PointCloudMsg>(
    "/points",
    10,
    std::bind(&LidarNode::onPointCloud, this, std::placeholders::_1));
```

这句话的含义是：把成员函数 `LidarNode::onPointCloud` 绑定到当前对象 `this`，第一个参数由订阅系统在消息到来时填入。

如果直接传 `&LidarNode::onPointCloud`，订阅系统不知道该用哪个对象调用它。成员函数指针缺少 `this`。

### `std::bind` 的基本形式

```cpp
using std::placeholders::_1;
using std::placeholders::_2;

auto f = std::bind(&Processor::process, &processor, _1, _2);
f(frame, timestamp);
```

等价心智模型：

```cpp
auto f = [&](const Frame& frame, double timestamp) {
    return processor.process(frame, timestamp);
};
```

`std::bind` 在历史代码中很多，尤其是 ROS/ROS2 教程和旧 C++11 代码。但现代 C++ 更常用 Lambda，因为 Lambda 更直观，类型推导和捕获也更清晰。

### Lambda 替代写法

```cpp
sub_ = create_subscription<PointCloudMsg>(
    "/points",
    10,
    [this](const PointCloudMsg::SharedPtr msg) {
        this->onPointCloud(msg);
    });
```

Lambda 显式写出了参数和调用体。编译器通常更容易内联和优化这种具体类型，但不能绝对说 Lambda 一定比 `std::bind` 更快；实际性能取决于编译器、优化级别、是否类型擦除、是否存储到 `std::function`。

| 写法 | 优点 | 代价 |
|------|------|------|
| 函数指针 | 简单，C 接口友好 | 不能捕获状态 |
| 成员函数指针 | 精确表达成员函数 | 调用语法重，必须绑定对象 |
| `std::bind` | 旧代码常见，可部分绑定参数 | 可读性一般，错误信息复杂 |
| Lambda | 捕获清晰，局部可读 | 生命周期要自己负责 |
| `std::function` | 可存储任意同签名可调用对象 | 类型擦除成本 |

### 生命周期：`this` 捕获不是所有权

下面写法捕获了 `this`：

```cpp
auto callback = [this](const PointCloudMsg& msg) {
    this->onPointCloud(msg);
};
```

它只保存了一个裸指针，不延长对象生命周期。如果回调在对象析构后仍被调用，就会悬空。

在 ROS2 节点中，订阅对象通常由节点成员持有，生命周期较清晰。但在异步任务、线程池、延迟执行队列中，捕获 `this` 要谨慎。需要共享生命周期时，可以捕获 `std::weak_ptr`：

```cpp
std::weak_ptr<LidarNode> weak_self = shared_from_this();

auto callback = [weak_self](const PointCloudMsg& msg) {
    if (auto self = weak_self.lock()) {
        self->onPointCloud(msg);
    }
};
```

这要求对象本身由 `shared_ptr` 管理，并正确使用 `enable_shared_from_this`。不要在构造函数中调用 `shared_from_this()`；对象尚未被 `shared_ptr` 控制时调用会抛出 `std::bad_weak_ptr`。

### 常见陷阱

> ⚠️ **生命周期陷阱：异步 Lambda 捕获 `this`**
>
> `[this]` 只捕获裸指针，不拥有对象。回调可能晚于对象析构执行时，应使用明确的生命周期管理，例如 `weak_ptr`。

> ⚠️ **可读性陷阱：过度使用 `std::bind` 的参数标记**
>
> `_1`、`_2`、参数重排和嵌套 bind 会让回调意图不清晰。现代代码优先 Lambda。

### 练习

1. **改写题**：把一个 `std::bind(&Node::callback, this, _1)` 改写为 Lambda。
2. **生命周期题**：构造一个线程延迟执行 `[this]` 的例子，让对象提前析构，解释风险。
3. **设计题**：什么时候应该捕获 `weak_ptr`，什么时候简单捕获 `this` 就足够？

`std::bind` 解决了成员函数绑定，但 Lambda 更直接。下一节系统讲 Lambda 语法和匿名函数对象模型。

---

## 10.3 Lambda 语法与匿名函数对象本质 ⭐⭐⭐

Lambda 表达式是 C++11 引入的最具影响力的语言特性之一。在 Lambda 出现之前，给 STL 算法传递自定义操作需要定义完整的函数对象类——即使逻辑只有一行，也要写构造函数、`operator()`、数据成员。Lambda 把这个过程压缩到一行表达式中，同时保留了函数对象的全部能力：可以携带状态（通过捕获）、可以参与模板实例化（具有唯一的具体类型）、可以被内联（编译器能看到完整定义）。

理解 Lambda 的关键是认识到它不是一种新的调用机制——它只是编译器自动生成函数对象类的语法糖。每个 Lambda 表达式在编译期都会被转换成一个匿名类的定义和一个该类对象的构造。值捕获变成数据成员，引用捕获变成引用或指针成员，函数体变成 `operator()`。一旦理解了这个转换规则，Lambda 的大小、生命周期、可拷贝性和性能特征就都可以精确预测。

### 为什么 Lambda 不是"匿名函数"那么简单

很多教材把 Lambda 介绍为"匿名函数"——这个说法在表面上是对的（Lambda 确实没有名字），但它遮蔽了 Lambda 最重要的特性：**Lambda 是编译器生成的函数对象，而不是简单的函数指针或代码块**。

理解这个区别至关重要。一个普通函数（或函数指针）是一段固定的代码，它不能"记住"任何东西——每次调用都从零开始，除了参数没有其他输入。但 Lambda 可以**捕获**创建时的上下文——阈值、配置参数、计数器、甚至其他对象的引用。这种"记住上下文"的能力不是魔法，而是因为编译器为每个 Lambda 生成了一个**匿名类**，捕获的变量成为这个类的**数据成员**，Lambda 的函数体成为这个类的 **`operator()`**。

换言之，Lambda 表达式 `[threshold](const Point& p) { return p.x > threshold; }` 在编译后和你手动写一个 `struct ThresholdFilter { double threshold_; bool operator()(const Point& p) const { return p.x > threshold_; } };` 是完全等价的。Lambda 只是省去了命名和显式声明的步骤——这对编程效率的提升是巨大的（不需要为每个一次性比较器写一个完整的类定义），但底层机制完全一致。

一旦掌握了"Lambda = 编译器生成的函数对象类 + 该类的一个实例"这个心智模型，以下所有 Lambda 的行为都变得可预测：
- **Lambda 的大小**取决于捕获了多少变量（数据成员的总大小）
- **Lambda 的生命周期安全性**取决于捕获的是值还是引用（数据成员是副本还是指针）
- **Lambda 能否拷贝**取决于捕获的变量是否可拷贝（数据成员的可拷贝性）
- **无捕获 Lambda 可以转为函数指针**是因为没有数据成员的空类可以退化为纯函数

### 完整语法

Lambda 的完整形式：

```cpp
[capture](parameters) mutable -> ReturnType {
    body
}
```

常见简写：

```cpp
auto isValid = [](const PointXYZI& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
};
```

组成部分：

| 部分 | 作用 |
|------|------|
| `[capture]` | 捕获外部变量 |
| `(parameters)` | 参数列表 |
| `mutable` | 允许修改值捕获副本 |
| `-> ReturnType` | 显式返回类型 |
| `{ body }` | 函数体 |

上面这个表格列出了 Lambda 的五个组成部分，但不要把它当成需要背诵的语法清单。更好的理解方式是把它映射到 运算符重载 中学过的函数对象类：`[capture]` 对应构造函数的参数和数据成员初始化，`(parameters)` 对应 `operator()` 的参数列表，`mutable` 控制 `operator()` 是否为 `const`，`-> ReturnType` 是 `operator()` 的返回类型，`{ body }` 是 `operator()` 的函数体。这个对应关系不是类比——它是编译器实际做的事情。

### Lambda 的本质

Lambda 可以理解为编译器生成的匿名类：

```cpp
double threshold = 10.0;

auto near = [threshold](const PointXYZI& p) {
    return squaredNorm(p) < threshold * threshold;
};
```

大致等价于：

```cpp
class AnonymousNear {
public:
    explicit AnonymousNear(double threshold) : threshold_(threshold) {}

    bool operator()(const PointXYZI& p) const {
        return squaredNorm(p) < threshold_ * threshold_;
    }

private:
    double threshold_;
};

auto near = AnonymousNear{threshold};
```

这直接连接 运算符重载：Lambda 的调用能力来自 `operator()`。值捕获的变量会变成匿名类的数据成员，引用捕获通常变成某种引用或指针语义的成员。

Lambda 与函数对象的关系，类似于匿名类与命名类的关系：它们在语义上等价，只是 Lambda 省去了命名和显式声明的步骤。不同的是，Lambda 的类型由编译器生成且不可拼写，因此不能在头文件中用作接口签名——需要统一存储时要借助 `std::function` 或模板参数。

#### Lambda 闭包类型的完整编译器生成规则

理解编译器为 Lambda 生成了什么，能消除"Lambda 是魔法"的错觉，让你准确预测 Lambda 的大小、可调用性、可拷贝性和生命周期行为。

编译器为每个 Lambda 表达式生成一个唯一的匿名类（闭包类型），规则如下：

**1. `operator()` 的生成**。Lambda 的函数体变成闭包类型的 `operator()` 成员函数。默认情况下，它是 `const` 成员函数——这就是为什么不能在 Lambda 内部修改值捕获的变量。加上 `mutable` 后，`operator()` 变成非 `const`。返回类型如果未显式指定，由编译器从 `return` 语句推导。

**2. 值捕获变成数据成员**。每个值捕获的变量变成闭包类型的一个非静态数据成员，成员类型是被捕获变量的类型（去掉引用后）。成员在 Lambda 对象构造时从外部变量拷贝初始化。

**3. 引用捕获的内部表示**。标准没有规定引用捕获的精确实现方式，但常见实现是用指针或引用成员。这意味着引用捕获的 Lambda 对象大小通常只增加一个指针的大小，而不是被引用对象的大小。

**4. 无捕获 Lambda 可以转成函数指针**。如果 Lambda 不捕获任何变量，闭包类型还会生成一个到函数指针的隐式转换运算符。这就是为什么 `[](int x){ return x+1; }` 可以赋给 `int(*)(int)` 类型的函数指针——此时闭包类型是空对象，行为和普通函数完全等价。

**5. 特殊成员函数**。闭包类型的拷贝/移动构造函数由编译器隐式生成，行为取决于捕获成员的可拷贝/可移动性。如果某个值捕获的变量不可拷贝（如 `std::unique_ptr`），闭包类型也不可拷贝。赋值运算符在 C++20 之前被删除（因为 `const` 成员函数的对象通常不应被赋值），C++20 起在某些条件下允许。

```cpp
// 编译器为下面的 Lambda 生成的闭包类型大致等价于：
double threshold = 10.0;
std::string tag = "scan";

auto filter = [threshold, &tag](const Point& p) {
    return squaredNorm(p) < threshold * threshold;
};

// 等价闭包类型：
class __lambda_filter {
public:
    __lambda_filter(double threshold, std::string& tag)
        : threshold_(threshold), tag_(tag) {}

    bool operator()(const Point& p) const {
        return squaredNorm(p) < threshold_ * threshold_;
    }

    // sizeof(__lambda_filter) = sizeof(double) + sizeof(std::string*)
    // 值捕获: double 成员
    // 引用捕获: 通常是指针或引用

private:
    double threshold_;       // 值捕获 → 数据成员
    std::string& tag_;       // 引用捕获 → 引用/指针成员
};
```

这个模型解释了很多实际行为：Lambda 的 `sizeof` 等于所有值捕获成员加上引用捕获指针的总大小（含对齐填充）；两个长得一样的 Lambda 表达式类型不同，因为编译器为每个 Lambda 表达式生成独立的匿名类型；`auto` 是保存 Lambda 的唯一方式（除了 `std::function` 类型擦除），因为类型名不可拼写。

如果 C++ 没有 Lambda，程序员每次给 `std::sort` 传一个比较器，都要先写一个命名结构体、声明 `operator()`、把需要的上下文变量写成成员。一个 ROS2 节点订阅三个话题，就要写三个专用类。Lambda 把这个开销从"写一个类"缩短到"写一行表达式"，认知负担大幅降低，但背后的对象模型没有改变。

### Lambda 大小

不捕获变量的 Lambda 通常是空对象：

```cpp
auto f = [](int x) { return x + 1; };
```

值捕获会增加对象大小：

```cpp
int a = 1;
double b = 2.0;
auto g = [a, b](int x) { return x + a + static_cast<int>(b); };
```

可以用 `sizeof(g)` 观察。结果取决于捕获成员和对齐。

### 返回类型推导

简单 Lambda 可以自动推导返回类型：

```cpp
auto norm = [](const Eigen::Vector3d& v) {
    return v.norm();
};
```

如果多个分支返回类型不同，应显式写返回类型，或让分支返回同一类型：

```cpp
auto score = [](bool ok) -> double {
    if (ok) {
        return 1.0;
    }
    return 0;  // 转成 double
};
```

### 常见陷阱

> ⚠️ **类型陷阱：两个长得一样的 Lambda 类型也不同**
>
> 每个 Lambda 表达式都有唯一闭包类型。`auto f = []{}; auto g = []{};` 中 `f` 和 `g` 类型不同。需要统一存储时，可以用模板、`std::function` 或函数指针。

> ⚠️ **返回类型陷阱：分支返回类型不一致**
>
> Lambda 返回类型推导要求各返回语句能推导出一致类型。复杂分支中显式写 `-> ReturnType` 更清楚。

### 练习

1. **等价改写题**：把一个捕获 `threshold` 的 Lambda 改写成手写函数对象类。
2. **实验题**：用 `sizeof` 比较无捕获、捕获一个 `int`、捕获一个 `std::array<double, 6>` 的 Lambda 大小。
3. **分析题**：为什么 Lambda 能作为 STL 算法的比较器或谓词？

理解了 Lambda 的对象模型后，捕获列表就是最关键的安全边界。

---

## 10.4 捕获模式：值、引用、移动与 `mutable` ⭐⭐⭐⭐

捕获列表是 Lambda 最关键的安全边界。值捕获、引用捕获和移动捕获三种方式各有不同的所有权语义和生命周期保证。选择错误的捕获方式是 C++ 并发和异步编程中最常见的 bug 来源之一——悬垂引用、数据竞争和 use-after-free 都可能由不当捕获引起。

理解捕获模式的关键是把 Lambda 看成一个对象：值捕获的变量是对象的数据成员（拥有独立副本），引用捕获的变量是对象内部的指针或引用（不拥有数据）。这个所有权模型决定了 Lambda 的生命周期安全性——如果 Lambda 的存活时间超过引用捕获的变量，就会产生悬垂引用。

### 值捕获

值捕获复制外部变量，Lambda 内部保存被捕获变量的独立副本：

```cpp
double max_range = 80.0;

auto inRange = [max_range](const PointXYZI& p) {
    return squaredNorm(p) < max_range * max_range;
};
```

Lambda 内部保存 `max_range` 的副本。之后外部 `max_range` 改变，不影响已经创建的 Lambda。

值捕获适合小型配置值、阈值、枚举和不可变参数。

### 引用捕获

引用捕获是 Lambda 捕获模式中最危险的一种。它不复制外部变量，而是在 Lambda 内部保存一个指向外部变量的引用（通常实现为指针）。这意味着 Lambda 对象本身不拥有数据——它依赖外部变量在其生命周期内保持有效。如果 Lambda 的存活时间超过引用捕获的变量（例如 Lambda 被存入容器、传递给异步任务、或从函数中返回），引用捕获的变量可能已经被销毁，Lambda 访问的就是悬垂引用——这是未定义行为，可能导致随机崩溃或静默的数据损坏。

引用捕获适合一个明确的场景：Lambda 在创建它的作用域内同步使用，且需要修改外部变量或避免大对象的拷贝开销。ROS2 回调中的 `[this]` 捕获本质上也是引用捕获——如果 `this` 指向的节点对象在回调执行前被销毁，就会产生悬垂指针。

引用捕获引用外部变量：

```cpp
std::size_t count = 0;

auto countValid = [&count](const PointXYZI& p) {
    if (isValid(p)) {
        ++count;
    }
};
```

引用捕获适合同步、局部、生命周期明确的场景。异步任务中引用捕获非常危险：

```cpp
std::future<void> launchBadTask() {
    std::vector<PointXYZI> points = loadPoints();

    return std::async(std::launch::async, [&points] {
        process(points);  // points 可能已经析构
    });
}
```

函数返回后 `points` 析构，异步任务引用悬空。

### 初始化捕获与移动捕获

C++14 引入的初始化捕获（generalized lambda capture）解决了一个 C++11 Lambda 无法处理的场景：把只能移动的对象（如 `std::unique_ptr`）捕获进 Lambda。在 C++11 中，值捕获只能拷贝，引用捕获不能转移所有权——这意味着 `unique_ptr` 既不能值捕获（因为不可拷贝）也不能安全地引用捕获（因为 Lambda 可能比原始指针活得更久）。初始化捕获通过在捕获列表中写赋值表达式（如 `[ptr = std::move(ptr)]`），让 Lambda 能"接管"外部对象的所有权。

C++14 引入初始化捕获：

```cpp
auto task = [cloud = std::move(cloud)] {
    process(cloud);
};
```

这会把外部 `cloud` 移动进 Lambda 对象，适合把大数据或 `unique_ptr` 交给异步任务。

```cpp
auto task = [buffer = std::make_unique<Buffer>()] {
    buffer->run();
};
```

回顾 移动语义与完美转发：移动后外部对象处于有效但未指定状态，不应继续依赖其内容。

### `mutable`：让值捕获的副本可以被修改

默认情况下，Lambda 的 `operator()` 是 `const` 成员函数——这意味着值捕获的变量（闭包类型的数据成员）不能在 Lambda 内部被修改。这个默认行为是安全的：如果你值捕获了一个阈值 `threshold`，你期望的是"使用这个阈值的快照"，而不是"在每次调用中修改它"。

但有时候你确实需要在多次调用之间保持和修改状态——例如计数器、累积器或交替状态。`mutable` 关键字让 `operator()` 变成非 `const`，从而允许修改值捕获的副本。注意修改的是 Lambda 对象内部的副本，不是外部的原始变量。

默认情况下，Lambda 的 `operator()` 是 `const`，不能修改值捕获副本：

```cpp
int count = 0;
auto f = [count]() {
    // ++count;  // 编译错误
};
```

加 `mutable` 后可以修改副本：

```cpp
auto f = [count]() mutable {
    ++count;  // 修改 Lambda 内部副本
    return count;
};
```

外部 `count` 不变。`mutable` 不适合隐藏状态变化；如果状态需要被外部观察，应明确使用对象或引用。

#### 捕获的对象模型：值捕获和引用捕获在内存中的精确含义

在 C++ 对象模型的层面上，值捕获和引用捕获的语义差异非常具体。值捕获在闭包对象内部创建了一个独立的数据成员——它是被捕获变量的一份**拷贝**，拥有独立的存储、独立的生命周期。闭包对象析构时，值捕获的成员会被析构；闭包对象拷贝时，值捕获的成员也会被拷贝。

引用捕获则在闭包对象内部存储的是对外部变量的**引用**（实现上通常是指针）。闭包对象不拥有被引用的变量，不控制它的生命周期。这就像 RAII与智能指针 中讨论的"观察者"和"拥有者"的区别：值捕获是拥有者，引用捕获是观察者。

这个区别在闭包对象大小上直接体现：

```cpp
int a = 1;
double b = 2.0;
std::array<double, 100> big{};

auto val_capture = [a, b, big]() {};
// sizeof(val_capture) ≈ sizeof(int) + sizeof(double) + sizeof(std::array<double,100>)
// 大约 816 字节：拷贝了所有数据

auto ref_capture = [&a, &b, &big]() {};
// sizeof(ref_capture) ≈ 3 * sizeof(void*)
// 大约 24 字节：只存了三个指针
```

这也解释了为什么大型配置对象应该用 `shared_ptr<const Config>` 值捕获或只捕获需要的字段：直接值捕获一个 100KB 的配置对象会让闭包对象膨胀到 100KB，每次传递 Lambda 都要拷贝这么多数据。

C++14 的初始化捕获 `[x = std::move(expr)]` 在对象模型中的含义是：闭包类型增加一个数据成员，该成员由 `expr` 移动构造（或拷贝构造，取决于 `expr` 的值类别）。这让闭包对象可以拥有 `unique_ptr`、`vector` 等只可移动类型，从而安全地把资源所有权转移给异步任务。

### 默认捕获 `[=]` 和 `[&]` 的风险分析

C++ 允许用 `[=]` 默认值捕获所有使用的变量，或用 `[&]` 默认引用捕获所有使用的变量。这两种写法在教学示例中很常见，但在工程代码中应谨慎使用。

`[=]` 的风险在于隐式捕获了 `this`（在成员函数中）。在 C++20 之前，`[=]` 在成员函数中会隐式捕获 `this` 指针——这是一个引用捕获，不是值捕获。C++20 将 `[=, this]` 变成推荐写法，`[=]` 隐式捕获 `this` 被标记为废弃。

`[&]` 的风险在于隐式引用捕获所有变量，包括那些你可能不打算使用的。如果 Lambda 被存储或异步执行，你需要逐一检查所有捕获变量的生命周期——但 `[&]` 不告诉你捕获了哪些变量，只有阅读函数体才能知道。

C++ Core Guidelines F.52 建议：在局部同步使用的 Lambda 中 `[&]` 可以接受，但只要 Lambda 可能被存储、返回或传递给异步任务，就应该显式列出捕获变量。

### 捕获模式选型

捕获模式的选择类似于搬家时打包物品的策略：值捕获像把物品复印一份带走，原件留在原地不受影响；引用捕获像留下一个地址条，随时回去取用，但如果原地址被拆迁就会出问题；移动捕获像把物品本身搬走，原地方不再有这件东西。选择哪种方式，取决于"任务执行时原始数据是否还在"和"任务是否需要修改原始数据"。不同的是，程序中的"搬家"往往发生在微秒级别，而"原址被拆"（对象析构）的时机可能完全不受 Lambda 控制。

| 场景 | 推荐捕获 | 原因 |
|------|----------|------|
| 小型只读阈值 | 值捕获 | 生命周期安全 |
| 同步统计计数 | 引用捕获 | 需要写回外部变量 |
| 异步处理大点云 | 移动捕获 | 明确转移所有权 |
| 回调绑定对象 | `[this]` 或 `weak_ptr` | 取决于生命周期 |
| 大型只读配置 | `shared_ptr<const Config>` 值捕获 | 避免大拷贝并共享只读状态 |

### 常见陷阱

> ⚠️ **异步陷阱：默认引用捕获 `[&]`**
>
> `[&]` 在异步任务中很危险，因为它可能捕获多个局部变量引用。异步 Lambda 优先显式列出捕获，并偏向值捕获或移动捕获。

> ⚠️ **语义陷阱：`mutable` 修改的是副本**
>
> `[count]() mutable { ++count; }` 不会修改外部 `count`。需要写回外部变量时，应引用捕获或使用共享状态。

### 练习

1. **安全题**：把一个 `[&]` 异步 Lambda 改成显式移动捕获版本。
2. **实验题**：用 `mutable` 修改值捕获变量，打印外部变量和内部返回值。
3. **设计题**：ROS2 回调需要访问配置对象和发布器，分别应如何捕获？

捕获让 Lambda 能携带状态。下一节讨论泛型 Lambda 和 `std::function`：一个强调编译期具体类型，一个强调运行时统一存储。

---

## 10.5 泛型 Lambda、`std::function` 与类型擦除 ⭐⭐⭐⭐

前面几节把 Lambda 理解为"带有固定参数类型的匿名函数对象"。但在泛型编程中，参数类型本身也应该是可变的。C++14 引入的泛型 Lambda 让参数类型可以写 `auto`——编译器会为每种调用参数类型生成一份独立的 `operator()` 实例。泛型 Lambda 和函数模板的关系，就像普通 Lambda 和普通函数对象的关系：语义等价，只是写法更简洁。

另一个重要问题是：Lambda 的类型是匿名的、不可拼写的，如何把不同 Lambda 存进同一个容器？`std::function` 通过类型擦除解决了这个问题——但代价是间接调用开销和可能的堆分配。理解这个代价的来源，是在"性能"和"灵活性"之间做出正确选择的基础。

### 泛型 Lambda

C++14 允许 Lambda 参数写 `auto`，这实际上让 Lambda 的 `operator()` 变成了模板成员函数：

```cpp
auto printSize = [](const auto& container) {
    std::cout << container.size() << "\n";
};
```

它本质上是一个带模板 `operator()` 的匿名类：

```cpp
class Anonymous {
public:
    template <typename Container>
    void operator()(const Container& container) const {
        std::cout << container.size() << "\n";
    }
};
```

这为 函数模板与类模板基础 模板编程做了铺垫。泛型 Lambda 适合局部泛型操作，例如对不同容器打印大小、对不同点类型做转换。

### `std::function`：统一存储可调用对象

如果只是把 Lambda 传给模板算法，不需要 `std::function`：

```cpp
std::sort(points.begin(), points.end(),
          [](const Point& a, const Point& b) {
              return a.timestamp < b.timestamp;
          });
```

`std::sort` 是模板，能直接看到 Lambda 的具体类型。

如果需要把回调存进容器，就需要统一类型：

```cpp
class CallbackManager {
public:
    using Callback = std::function<void(const PointCloud&)>;

    void subscribe(Callback callback) {
        callbacks_.push_back(std::move(callback));
    }

    void publish(const PointCloud& cloud) const {
        for (const auto& callback : callbacks_) {
            callback(cloud);
        }
    }

private:
    std::vector<Callback> callbacks_;
};
```

`std::function` 使用类型擦除：它能保存函数指针、Lambda、函数对象、`std::bind` 结果，只要调用签名匹配。

如果没有 `std::function`，把不同类型的回调存进同一个容器会非常困难。每个 Lambda 都有唯一的匿名类型，无法放进 `std::vector<???>`。程序员要么退回到函数指针（丢失状态），要么手写类型擦除基类（大量样板代码），要么把所有回调都写成虚函数派生类（过度设计）。`std::function` 把这个常见需求标准化了，代价是一层间接调用和可能的堆分配。

### `std::function` 的类型擦除实现原理

`std::function` 是 C++ 标准库中最典型的类型擦除案例。类型擦除的核心思想是：用一个统一的接口（固定的调用签名）包装任意具体类型的可调用对象，让调用者不需要知道底层具体类型就能调用。

实现类型擦除的常见手段是**虚函数表**或**函数指针表**。`std::function` 内部大致包含以下组件：

```text
std::function<void(const PointCloud&)> 的内部结构（简化）：

┌──────────────────────────────────────┐
│ 函数指针表（vtable 或 manager 指针）  │ → 指向拷贝、析构、调用三个操作的函数
│ 存储区域                              │ → 保存可调用对象（内联或堆指针）
└──────────────────────────────────────┘
```

当你把一个 Lambda 赋给 `std::function` 时，内部发生的事情是：

1. 检查 Lambda 闭包类型的大小。如果足够小（通常是一两个指针的大小），直接存储在 `std::function` 内部的固定缓冲区中——这就是**小对象优化**（Small Buffer Optimization, SBO）。
2. 如果闭包类型太大，在堆上分配内存存储闭包对象，内部缓冲区存放指向堆内存的指针。
3. 记录三个操作的函数指针：如何调用、如何拷贝、如何析构这个具体类型的对象。

调用 `std::function` 时，通过存储的函数指针间接调用实际对象的 `operator()`。这次间接调用无法被内联——编译器看到的是函数指针，不知道它指向哪个具体函数。这就是 `std::function` 比直接传 Lambda 给模板算法更慢的根本原因：**模板参数保留了具体类型，编译器可以内联；`std::function` 擦除了具体类型，编译器只能做间接调用。**

不同标准库实现的 SBO 阈值不同。GCC 的 `libstdc++` 通常在 `std::function` 内部预留 16 字节的缓冲区；MSVC 的实现预留的空间可能不同。无捕获的 Lambda 是空对象（大小为 1 字节），总是能走 SBO。捕获一两个指针或 `double` 的 Lambda 通常也能走 SBO。捕获大型结构体或 `std::string` 的 Lambda 可能触发堆分配。

```cpp
// 走 SBO（通常不堆分配）：
std::function<bool(int)> f1 = [](int x) { return x > 0; };  // 无捕获

double threshold = 10.0;
std::function<bool(int)> f2 = [threshold](int x) { return x > threshold; };  // 捕获一个 double

// 可能触发堆分配：
std::string long_prefix = "sensor_data_frame_";
std::function<void()> f3 = [long_prefix]() { /* ... */ };  // 捕获了 std::string
```

小对象优化不是 C++ 标准强制保证，不同标准库实现细节不同。教学上应记住：需要存储异构可调用对象时用 `std::function`；只是立即传给模板算法时，直接传 Lambda 更轻。

| 场景 | 推荐写法 |
|------|----------|
| `std::sort` 比较器 | 直接 Lambda |
| `std::for_each` 操作 | 直接 Lambda |
| 回调列表 | `std::function` |
| C API 回调 | 函数指针或静态桥接函数 |
| 高性能内层策略 | 模板参数/函数对象 |

### `std::function` 和异常安全

`std::function` 为空时调用会抛出 `std::bad_function_call`：

```cpp
std::function<void()> f;
f();  // 抛异常
```

回顾 错误处理与异常安全：回调系统中应明确处理空回调。可以在调用前判断：

```cpp
if (callback) {
    callback(cloud);
}
```

### 常见陷阱

> ⚠️ **性能陷阱：把 STL 算法的 Lambda 先装进 `std::function`**
>
> `std::sort(begin, end, std::function<bool(...)>{lambda})` 会丢失具体类型信息，增加间接调用。算法参数直接传 Lambda 更好。

> ⚠️ **空回调陷阱：调用默认构造的 `std::function`**
>
> 默认构造的 `std::function` 为空。调用前检查，或在构造时保证有效。

### 练习

1. **代码题**：实现 `CallbackManager`，支持注册多个 `std::function<void(const PointCloud&)>` 并发布点云。
2. **对比题**：把同一个 Lambda 直接传给 `std::sort`，再包成 `std::function` 传入，比较代码和潜在开销。
3. **泛型题**：写一个泛型 Lambda，能打印任意有 `.size()` 的容器大小。

可调用对象准备好了，下一步是把它们和 STL 算法结合起来，让“对容器做什么”从手写循环变成可组合操作。

---

## 10.6 STL 算法：让遍历意图显式化 ⭐⭐⭐

STL 算法是 C++ 标准库中最强大也最被低估的工具集。它的核心设计哲学是"把遍历模式命名化"——`std::transform` 表达"逐元素变换"，`std::remove_if` 表达"按条件移除"，`std::accumulate` 表达"归约求和"。每个算法名字本身就是文档：读者看到算法名就知道遍历的目的，不需要逐行阅读循环体来推断意图。

在机器人代码中，点云处理、轨迹变换、传感器数据过滤等操作天然适合 STL 算法。用 `std::transform` 把世界坐标系的点云变换到相机坐标系，用 `std::partition` 把近距离点和远距离点分开，用 `std::accumulate` 计算点云质心——这些操作的意图在算法名中一目了然。

### 动机：为什么"命名遍历模式"很重要

要理解 STL 算法的价值，首先要理解**为什么给遍历操作命名是一件重要的事**。

考虑一个类比：在日常语言中，"清洗数据"和"转换坐标系"是两个独立的概念。你不会把这两件事混在同一句话里说——"把无效的点删掉同时把剩下的点从 LiDAR 坐标系变换到世界坐标系"这句话虽然语法正确，但认知负担远高于"先清洗，再转换"。

代码也是一样。手写循环的问题不是"效率低"（现代编译器对手写循环和 STL 算法生成的机器码通常一样快），而是**它不表达意图**。一个 `for` 循环告诉读者"我要遍历这些元素"，但没有说"我要做什么"——读者必须阅读循环体的每一行才能推断出操作的性质。`std::transform` 直接说"我要逐元素变换"，`std::remove_if` 直接说"我要按条件移除"——**算法名本身就是文档**。

这不是代码风格的偏好问题，而是**软件工程中的可维护性问题**。三个月后回来修改代码的人（可能是你自己）看到 `std::partition`，立刻知道这段代码把序列分成满足条件和不满足条件的两半。如果看到的是一个 30 行的手写双指针循环，他需要花 5 分钟理解循环不变量才能确认这段代码在做什么。在大型 SLAM 项目中，这种理解成本的差异会累积成巨大的维护负担。

### 手写循环容易混合多个意图

点云预处理常见流程：

1. 删除 NaN。
2. 按距离过滤。
3. 坐标变换。
4. 按时间排序。
5. 分离近点和远点。

如果全部写成手工 `for`，循环体里会混合遍历、条件、转换、插入和统计。一个 80 行的手写循环可能同时做过滤、转换和统计，后续维护者很难判断修改过滤条件是否会影响统计逻辑。STL 算法把意图写在函数名里：`remove_if` 就是过滤，`transform` 就是变换，`accumulate` 就是归约。

STL 算法和手写循环的关系，类似于函数和 `goto` 的关系：`goto` 能跳到任意位置，但结构化的 `for`/`while`/`if` 让控制流可预测。同理，手写循环能做任何事，但 `std::sort`、`std::partition`、`std::transform` 把常见操作模式编码成名字，读者不需要逐行理解循环体就能知道"这段代码在做什么"。

如果不用 STL 算法，而是所有点云处理都写成手写循环，常见后果包括：迭代器手动管理容易越界、erase 操作中忘记更新循环变量导致跳过元素、过滤和变换逻辑缠绕使单元测试无法拆分。这些问题在代码量小时不明显，但随着管线步骤增多会迅速恶化。

### STL 算法的复杂度保证与迭代器类别

STL 算法不只是"方便的循环替代品"——它们携带标准保证的复杂度上界，这些保证和迭代器类别紧密关联。

C++ 标准把迭代器分成五个类别，形成从弱到强的层次结构：

| 迭代器类别 | 支持的操作 | 典型容器 |
|-----------|-----------|---------|
| InputIterator | 单遍只读（`*it`、`++it`） | `std::istream_iterator` |
| ForwardIterator | 多遍只读/写（`*it`、`++it`、可复制） | `std::forward_list`、`unordered_map` |
| BidirectionalIterator | 双向移动（`++it`、`--it`） | `std::list`、`std::map` |
| RandomAccessIterator | 随机跳跃（`it + n`、`it[n]`、`it1 - it2`） | `std::vector`、`std::deque` |
| ContiguousIterator（C++20） | 连续内存 | `std::vector`、`std::array`、`std::span` |

算法的复杂度保证和它要求的最低迭代器类别之间存在直接关系：

| 算法 | 要求迭代器 | 复杂度保证 | 为什么需要这个类别 |
|------|----------|-----------|-----------------|
| `std::find` | InputIterator | O(n) | 单遍线性扫描 |
| `std::sort` | RandomAccessIterator | O(n log n) 平均 | 需要随机跳跃做划分 |
| `std::stable_sort` | RandomAccessIterator | O(n log n) 如有额外内存，否则 O(n log²n) | 需要归并 |
| `std::partition` | ForwardIterator（C++11 起） | O(n) | 单向扫描双指针 |
| `std::reverse` | BidirectionalIterator | O(n) | 需要从两端向中间靠拢 |
| `std::nth_element` | RandomAccessIterator | O(n) 平均 | 基于快速选择算法 |
| `std::lower_bound` | ForwardIterator（但只有 RandomAccess 才 O(log n)） | O(log n) 步 + O(n) 总移动 | 二分查找需要计算中点 |

最后一行特别值得注意：`std::lower_bound` 对 ForwardIterator 也能工作，但由于不能随机跳跃，计算中点需要线性前进 n/2 步，导致总复杂度从 O(log n) 退化到 O(n)。标准只保证"最多做 O(log n) 次比较"，但迭代器推进次数取决于类别。这就是为什么在 `std::list` 上调用 `lower_bound` 不如在 `std::vector` 上快——尽管两者使用同一个算法名。

对点云处理来说，`std::vector<PointXYZI>` 提供 RandomAccessIterator 和连续内存，因此 `std::sort`、`std::partition`、`std::nth_element` 等算法都能以最优复杂度运行，并且受益于 CPU 缓存的空间局部性。

理解迭代器类别和算法复杂度保证的关系，是从"会用 STL 算法"到"理解 STL 设计哲学"的关键一步。STL 的设计者（Alexander Stepanov）的核心洞见是：算法的效率不取决于具体的数据结构，而取决于数据结构能提供哪些**操作**。`std::sort` 需要 O(n log n) 次比较和 O(n log n) 次元素交换——这要求能在 O(1) 时间内访问任意位置的元素（RandomAccessIterator）。`std::find` 只需要 O(n) 次比较和单向前进——InputIterator 就够了。算法声明自己需要的最低迭代器类别，容器声明自己提供的迭代器类别，两者的匹配在编译期完成——如果你试图在 `std::list` 上调用 `std::sort`，编译器会报错，因为 `list` 的 BidirectionalIterator 不满足 `sort` 要求的 RandomAccessIterator。这种编译期约束是 STL 泛型设计的核心优势之一——错误在编译时暴露，而不是在运行时表现为性能退化。

下面用几个具体的点云处理场景展示 STL 算法如何替代手写循环，以及它们和 Lambda 的配合使用。

### `std::remove_if` + erase

删除无效点：

```cpp
points.erase(
    std::remove_if(points.begin(), points.end(),
                   [](const PointXYZI& p) {
                       return !std::isfinite(p.x) ||
                              !std::isfinite(p.y) ||
                              !std::isfinite(p.z);
                   }),
    points.end());
```

`std::remove_if` 不真的缩短容器，它把保留元素前移，返回新的逻辑末尾。`erase` 才真正删除尾部元素。这叫 erase-remove idiom。

### `std::transform`

点云坐标变换：

```cpp
std::vector<Eigen::Vector3d> world_points;
world_points.reserve(points.size());

std::transform(points.begin(), points.end(),
               std::back_inserter(world_points),
               [T_world_lidar](const PointXYZI& p) {
                   return T_world_lidar * Eigen::Vector3d(p.x, p.y, p.z);
               });
```

`transform` 表达“逐元素变换”。如果输入输出大小已知，也可以先 `resize` 再写入迭代器。

### `std::accumulate`

计算质心：

```cpp
#include <numeric>
#include <stdexcept>

Eigen::Vector3d sum = std::accumulate(
    points.begin(), points.end(), Eigen::Vector3d::Zero(),
    [](const Eigen::Vector3d& acc, const PointXYZI& p) {
        return acc + Eigen::Vector3d(p.x, p.y, p.z);
    });

if (points.empty()) {
    throw std::invalid_argument("empty point cloud");
}

Eigen::Vector3d centroid = sum / static_cast<double>(points.size());
```

这里初始值类型决定累积结果类型。如果初始值写成 `0`，可能得到整数累积，这是常见错误。

### `std::partition` 与 `std::sort`

分离近点和远点：

```cpp
auto split = std::partition(points.begin(), points.end(),
                            [](const PointXYZI& p) {
                                return squaredNorm(p) < 100.0;
                            });
```

`[points.begin(), split)` 是近点，`[split, points.end())` 是远点。`partition` 不保证相对顺序；需要保持顺序时用 `stable_partition`，代价可能更高。

按时间排序：

```cpp
std::sort(points.begin(), points.end(),
          [](const PointXYZI& a, const PointXYZI& b) {
              return a.time < b.time;
          });
```

比较器必须提供严格弱序。不要写不稳定的比较逻辑，例如浮点 NaN 参与比较会破坏排序假设。

### 常见陷阱

> ⚠️ **迭代器陷阱：`remove_if` 后忘记 `erase`**
>
> `remove_if` 只移动元素并返回新末尾，不改变容器大小。删除元素要配合 `erase`。

> ⚠️ **累积陷阱：`std::accumulate` 初始值类型错误**
>
> 初始值 `0` 会让累积走整数类型。矩阵、向量、浮点累积应给正确初始对象。

### 练习

1. **流水线题**：用 `remove_if`、`transform`、`accumulate` 写点云清洗、坐标变换和质心计算。
2. **分区题**：用 `partition` 把点云分成近点和远点，解释为什么输出顺序可能变化。
3. **排序题**：设计一个按时间排序的比较器，说明如何处理 NaN 或无效时间戳。

STL 算法需要合适的容器和迭代器。下一节讲 range-for 的底层机制和容器选型。

---

## 10.7 range-for、迭代器与容器选型 ⭐⭐⭐

容器选型是数据结构设计中最基础的决策之一。在机器人代码中，容器的选择直接影响内存布局、cache 效率和算法复杂度。`std::vector` 的连续内存布局让逐点遍历极其高效（cache 友好），但中间插入和删除是 O(n) 的。`std::deque` 支持两端高效插入，适合 IMU 数据缓冲。`std::unordered_map` 的 O(1) 查找适合体素地图，但哈希冲突会导致性能退化。

range-for 循环是 C++11 引入的语法糖，它把"遍历容器所有元素"的常见模式标准化。理解 range-for 的展开规则（编译器如何把 `for (auto& x : container)` 转换为迭代器循环）对避免生命周期陷阱至关重要。

### range-for 展开

```cpp
for (const auto& point : cloud) {
    process(point);
}
```

可以理解为：

```cpp
auto&& range = cloud;
auto begin_it = std::begin(range);
auto end_it = std::end(range);

for (; begin_it != end_it; ++begin_it) {
    const auto& point = *begin_it;
    process(point);
}
```

编译器查找 `begin`/`end` 的方式包括数组、成员函数和 ADL 自由函数。自定义点云容器只要提供合适的 `begin()` 和 `end()`，就能支持 range-for。

### range-for 生命周期陷阱

不要引用临时对象内部元素：

```cpp
for (const auto& point : makePointCloud()) {
    process(point);
}
```

这个形式本身通常会延长 range 临时对象到循环结束，较安全。但如果表达式返回的是某个临时对象的子范围或视图，就可能悬垂。C++20 ranges/view 中这类问题更常见。

稳妥方式是先命名中间对象：

```cpp
auto cloud = makePointCloud();
for (const auto& point : cloud) {
    process(point);
}
```

### 容器选型

| 容器 | 内存布局 | 随机访问 | 插入删除 | 机器人场景 |
|------|----------|----------|----------|------------|
| `std::vector` | 连续 | O(1) | 尾部快，中间慢 | 点云、特征、轨迹 |
| `std::deque` | 分段连续 | O(1) | 头尾快 | IMU/里程计时间缓冲 |
| `std::list` | 链表 | 无 | 节点插入快 | 少用，cache 不友好 |
| `std::array` | 固定大小连续 | O(1) | 不可变大小 | 小固定数组 |
| `std::unordered_map` | 哈希表 | 按 key | 平均 O(1) | 体素哈希地图 |
| `std::map` | 红黑树 | 按 key 有序 | O(log n) | 需要有序遍历的索引 |

点云通常用 `vector`，因为连续内存对 cache 和批量处理友好。传感器时间缓冲常用 `deque`，因为需要头部弹出旧数据、尾部追加新数据。

容器选型的核心原则是**让数据的内存布局匹配访问模式**。现代 CPU 的缓存机制意味着连续内存的顺序访问比随机内存的跳跃访问快 10-100 倍。一个 10 万点的 `std::vector<PointXYZI>` 在顺序遍历时几乎完全在 L1/L2 缓存中完成；同样大小的 `std::list<PointXYZI>` 每个节点在堆上的位置可能完全随机，每次访问下一个节点都可能触发缓存未命中。在 SLAM 系统中，每帧处理几十万个点是常态——选错容器可能让预处理从 5ms 变成 50ms，直接突破实时性约束。

如果不确定该选哪种容器，`std::vector` 几乎总是正确的起点。它的连续内存布局在绝大多数场景下都是性能最优的选择，只有在确认了特定的瓶颈（如频繁的中间插入/删除）后才需要考虑其他容器。

### `std::array`、C 数组与 `std::span`

固定大小数据：

```cpp
std::array<double, 3> accel{};
```

相比 C 数组，`std::array` 保留大小信息，支持 `.size()` 和 STL 算法。

`std::span` 是非拥有视图：

```cpp
void processSpan(std::span<const PointXYZI> points);
```

它不拷贝数据，只保存指针和长度。适合把连续数据切片传给函数。注意它不拥有数据，调用者必须保证底层内存存活。

### `unordered_map` 自定义哈希

体素哈希地图常用整数三维坐标做 key：

```cpp
struct VoxelKey {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey&) const = default;
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept {
        auto mix = [](std::size_t seed, std::size_t value) {
            value += 0x9e3779b97f4a7c15ull;
            value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
            value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
            value ^= value >> 31;
            return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2));
        };

        std::size_t h = 0;
        h = mix(h, static_cast<std::size_t>(key.x));
        h = mix(h, static_cast<std::size_t>(key.y));
        h = mix(h, static_cast<std::size_t>(key.z));
        return h;
    }
};

std::unordered_map<VoxelKey, VoxelBlock, VoxelKeyHash> map;
```

哈希函数质量会影响性能。体素地图中 key 分布有空间结构，简单拼接可能造成冲突，应结合实际数据 profile。

### 常见陷阱

> ⚠️ **容器陷阱：在点云主路径使用 `std::list`**
>
> 链表节点分散，cache 命中差。即使中间插入是 O(1)，遍历点云时通常比 `vector` 慢。

> ⚠️ **视图陷阱：`std::span` 不拥有数据**
>
> 返回指向局部 `vector` 的 `span` 会悬垂。`span` 适合参数，不适合延长生命周期。

### 练习

1. **容器题**：为点云、IMU 缓冲、体素地图、固定三轴数据选择容器并说明原因。
2. **迭代器题**：给自定义点云容器实现 `begin()`/`end()`，让它支持 range-for。
3. **哈希题**：为 `VoxelKey` 写哈希函数，并统计不同体素分布下的冲突情况。

容器决定数据组织，结构化绑定和现代工具能让容器使用更清晰。下一节讲 C++17/20 的常用补充。

---

## 10.8 结构化绑定、`optional`、`variant`、`if constexpr` 与并行 STL ⭐⭐⭐

C++17 引入了一系列实用工具，它们共同的设计目标是"让代码更接近意图表达"。结构化绑定让多返回值不再需要临时变量解包；`optional` 让"可能没有结果"成为类型系统的一部分；`variant` 让"有限类型集合"的安全访问成为编译器可检查的契约；`if constexpr` 让模板函数中的类型分支更加自然。这些工具不是独立的语法糖，而是现代 C++ 类型安全理念的具体体现——把运行时可能犯的错误尽可能推到编译期。

### `pair`、`tuple` 与结构化绑定

`std::unordered_map::insert` 返回一个 pair：

```cpp
auto [it, inserted] = voxel_map.insert({key, block});

if (inserted) {
    initializeBlock(it->second);
}
```

结构化绑定让多返回值更可读。对函数返回多个相关值时，也可以用结构体替代 tuple，让字段名更清楚。

```cpp
struct MatchResult {
    int index = -1;
    double distance = 0.0;
    bool valid = false;
};
```

### `std::optional`

`std::optional<T>` 是 C++17 中表达"可能没有值"的标准工具。在机器人代码中，很多操作天然有"失败但不是错误"的结果：配准可能因为特征点不足而无法给出位姿估计，最近邻搜索可能在空树上返回无结果，回环检测可能在当前帧没有匹配。这些情况不是程序错误（不应该抛异常），而是算法的正常输出之一。`optional` 让这种"有值或无值"的语义显式化——调用者必须检查是否有值才能访问，编译器和代码审查者都能看到这个检查是否存在。

配准可能失败：

```cpp
std::optional<Pose3> estimatePose(const Frame& frame);
```

调用端必须处理无值情况：

```cpp
if (auto pose = estimatePose(frame)) {
    publish(*pose);
} else {
    handleFailure();
}
```

这比“返回 bool + 输出参数”更不容易误用。

### `std::variant`

`std::variant` 是 C++17 中的"类型安全联合体"（type-safe union）。它表达"这个变量在任意时刻持有 N 种类型中的某一种"——和 C 的 `union` 不同，`variant` 在编译期知道当前持有的是哪种类型，尝试以错误类型访问会抛异常而非导致未定义行为。在机器人代码中，`variant` 适合表达固定类型集合的传感器数据、有限种类的配置选项、或多种算法结果。

`variant` 表达封闭集合：

```cpp
using SensorMeasurement = std::variant<ImuMeasurement,
                                       LidarMeasurement,
                                       CameraMeasurement>;
```

访问：

```cpp
std::visit([](const auto& measurement) {
    processMeasurement(measurement);
}, measurement);
```

如果类型集合固定，`variant` 可以替代一部分继承。但新增类型需要修改 variant 定义和访问逻辑，不适合开放插件体系。

### `if constexpr`

在泛型 Lambda 或模板中按类型分支：

```cpp
auto printMeasurement = [](const auto& m) {
    using T = std::decay_t<decltype(m)>;

    if constexpr (std::is_same_v<T, ImuMeasurement>) {
        printImu(m);
    } else if constexpr (std::is_same_v<T, LidarMeasurement>) {
        printLidar(m);
    } else {
        printGeneric(m);
    }
};
```

这连接 模板特化SFINAE与类型萃取：未选分支不实例化，适合类型条件。

### 并行 STL

C++17 提供执行策略：

```cpp
std::for_each(std::execution::par,
              points.begin(), points.end(),
              [](PointXYZI& p) {
                  p.intensity = normalize(p.intensity);
              });
```

注意边界：

1. 具体后端依赖标准库实现。
2. Lambda 内部不能有数据竞争。
3. 标准执行策略下，算法体抛出未捕获异常通常会调用 `std::terminate`，不要把异常传播当作普通控制流。
4. 对小数据量，并行开销可能大于收益。

点云逐点独立处理适合并行；涉及共享地图插入、累计统计、日志输出时要特别小心同步。

### 常见陷阱

> ⚠️ **并行陷阱：Lambda 捕获共享变量并写入**
>
> `std::execution::par` 下多个元素可能同时执行。引用捕获计数器并 `++count` 会数据竞争。使用归约、原子或线程局部累积。

> ⚠️ **设计陷阱：用 `variant` 模拟开放插件**
>
> `variant` 适合固定类型集合。插件系统需要第三方新增类型时，虚函数和工厂更合适。

### 练习

1. **optional 题**：把 `bool estimatePose(Frame, Pose*)` 改成 `std::optional<Pose>` 版本。
2. **variant 题**：用 `variant` 保存三类传感器测量，并用 `std::visit` 分发处理。
3. **并行题**：用并行 STL 归一化点云强度，确保没有共享写数据竞争。

---

## 10.9 C++23 Lambda 演进：`static operator()`、deducing this 与 ranges 适配 ⭐⭐⭐⭐

C++ 标准对 Lambda 的改进从未停止。C++23 带来了三项对 Lambda 有深远影响的特性：`static operator()` 消除了无捕获 Lambda 的隐式 `this` 指针开销；deducing this 让 Lambda 能够通过显式对象参数实现递归和 CRTP 风格的自引用；`std::ranges` 进一步强化了 Lambda 与管线式数据处理的配合。这些演进的共同方向是让 Lambda 更接近"零开销抽象"的 C++ 理想。

### `static operator()`：无捕获 Lambda 的最终形态 ⭐⭐⭐

C++23 之前，即使 Lambda 不捕获任何变量，编译器生成的 `operator()` 仍然是非静态成员函数——它有一个隐式的 `this` 参数（指向闭包对象）。对于无捕获 Lambda，这个 `this` 完全没有用处：闭包对象是空的，没有数据成员可以通过 `this` 访问。但 `this` 参数仍然占据一个寄存器传递位置，在某些调用约定下可能影响内联和参数传递效率。

C++23 允许 Lambda 的 `operator()` 声明为 `static`，显式消除 `this` 参数：

```cpp
// C++23: static operator()
auto square = [](double x) static -> double {
    return x * x;
};
```

编译器为这个 Lambda 生成的闭包类型大致等价于：

```cpp
struct __lambda_square {
    static double operator()(double x) {
        return x * x;
    }
};
```

注意 `static` 成员函数没有 `this` 参数——调用时只传递 `x`，不传递闭包对象指针。在 ROS2 节点的高频回调和 SLAM 内循环中，每个指针参数都可能影响寄存器分配和内联决策。`static operator()` 把无捕获 Lambda 的调用开销从"接近零"降低到"精确为零"。

限制条件很直接：只有不捕获任何变量的 Lambda 才能使用 `static`。一旦有任何捕获（值、引用或初始化捕获），`operator()` 就必须访问闭包对象的数据成员，不能是静态的。

```cpp
// 错误：有捕获的 Lambda 不能用 static
double threshold = 10.0;
// auto bad = [threshold](double x) static { return x > threshold; };  // 编译错误
```

`static operator()` 和运算符重载中讨论的 `operator()` 是同一个机制——C++23 只是允许它在闭包类型中被标记为 `static`，这在以前的 C++ 版本中对普通类的 `operator()` 也不被允许。

### deducing this：Lambda 的显式对象参数 ⭐⭐⭐⭐

C++23 的 deducing this（显式对象参数）是一个更深刻的语言扩展。它允许成员函数（包括 Lambda 的 `operator()`）把通常隐式的 `this` 参数写成显式的模板参数，从而在函数体内推导自身的类型和值类别。

对 Lambda 而言，deducing this 解决了一个长期存在的问题：**Lambda 无法在内部引用自己**。在 C++23 之前，递归 Lambda 需要借助 `std::function` 的间接调用（丧失内联和具体类型信息）或传入自身引用的额外参数（写法不自然）：

```cpp
// C++20 之前的递归 Lambda——笨拙且有间接调用开销
std::function<int(int)> factorial;
factorial = [&factorial](int n) -> int {
    return n <= 1 ? 1 : n * factorial(n - 1);  // 通过 std::function 间接调用
};
```

C++23 的 deducing this 让 Lambda 能直接引用自身，不需要 `std::function`：

```cpp
// C++23: deducing this 实现递归 Lambda
auto factorial = [](this auto self, int n) -> int {
    return n <= 1 ? 1 : n * self(n - 1);  // self 就是 Lambda 自身
};

int result = factorial(5);  // 120
```

这里的 `this auto self` 是显式对象参数——`self` 接收闭包对象本身，编译器能看到它的具体类型，可以做完整的内联优化。和 `std::function` 版本相比，deducing this 版本没有类型擦除开销，递归调用可以被编译器展开或尾调用优化。

> **反事实推理**：如果不用 deducing this 也不用 `std::function`，递归 Lambda 的另一种写法是"传入自身"的 Y 组合子模式：`auto fact = [](auto self, int n) -> int { return n <= 1 ? 1 : n * self(self, n-1); };`，调用时写 `fact(fact, 5)`。这种写法类型安全且无间接调用，但 `self(self, n-1)` 的语法很不自然。deducing this 正是为了消除这种语法笨拙。

Deducing this 在机器人代码中的一个实际用途是递归遍历树状数据结构——例如八叉树（Octree）的深度优先遍历：

```cpp
// C++23: 用 deducing this Lambda 递归遍历八叉树
auto traverse = [&visitor](this auto self, const OctreeNode& node) -> void {
    visitor(node);
    for (const auto& child : node.children) {
        if (child) {
            self(*child);  // 递归调用自身
        }
    }
};

traverse(root);
```

### `std::ranges` 与 Lambda 的管线式配合 ⭐⭐⭐

C++20 引入的 `std::ranges` 和 C++23 的扩展让 STL 算法的使用方式发生了范式转变。传统 STL 算法需要成对的迭代器（`begin`, `end`），ranges 版本直接接受容器，并支持管线式组合——用 `|` 运算符把多个操作串联起来：

```cpp
#include <ranges>
#include <algorithm>
#include <vector>

// C++20 ranges 版本的点云过滤 + 变换
auto valid_world_points = points
    | std::views::filter([](const PointXYZI& p) {
          return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      })
    | std::views::filter([max_range](const PointXYZI& p) {
          return squaredNorm(p) < max_range * max_range;
      })
    | std::views::transform([&T_world_lidar](const PointXYZI& p) {
          return T_world_lidar * Eigen::Vector3d(p.x, p.y, p.z);
      });
```

这段代码和前面用 `remove_if` + `erase` + `transform` 的版本语义等价，但有两个关键区别：

1. **惰性求值**：`views::filter` 和 `views::transform` 不会立即遍历数据，它们返回的是"视图"——只有在最终消费（如存入 `vector`、用 range-for 遍历）时才真正执行计算。这和运算符重载中 Eigen 表达式模板的思想一致：延迟计算以减少中间对象。

2. **管线可读性**：操作按执行顺序从上到下排列，每一步的 Lambda 描述该步骤的逻辑。读者不需要从内到外解嵌套函数调用。

> **本质洞察**：`std::ranges` 管线把"对数据做什么操作"和"如何遍历数据"彻底分离。Lambda 描述"做什么"，视图适配器描述"怎么组合"，最终消费点决定"何时执行"。这种分离让点云预处理的逻辑可以像搭积木一样组合和重排。

需要注意的是，ranges 视图持有对原始数据的引用——和 Eigen 表达式模板一样，如果原始容器在视图使用之前被销毁或修改，行为是未定义的。工程上应在同一作用域内完成视图的创建和消费，或显式用 `std::ranges::to<std::vector>()` (C++23) 物化结果。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：对有捕获的 Lambda 使用 `static`**
>
> **错误做法**：`[threshold](double x) static { return x > threshold; }`。
>
> **现象**：编译错误——`static operator()` 不能访问非静态数据成员。
>
> **根本原因**：`static` 成员函数没有 `this`，无法访问闭包对象的捕获成员。
>
> **正确做法**：只对无捕获 Lambda 使用 `static`。

> 💡 **概念误区：deducing this 等于把 Lambda 变成普通函数**
>
> **实际上**：deducing this 只是让 `this` 从隐式变成显式——Lambda 仍然是闭包对象，仍然可以有捕获。它的价值在于让 Lambda 能自引用，以及在值类别（左值/右值）上分派行为。
>
> **正确理解**：deducing this 是对 Lambda 对象模型的增强，不是简化。

> 🧠 **思维陷阱：ranges 视图总是比传统 STL 算法快**
>
> ranges 视图的惰性求值减少了中间容器分配，但每次解引用视图迭代器时都可能经过多层间接调用。对于简单的一次性遍历，传统 STL 算法加 erase-remove 的性能可能相当甚至更好。选择 ranges 的理由是可读性和组合性，不是性能。

### 练习

1. **代码题**：把一个无捕获的过滤 Lambda 改写为 `static operator()` 形式，并用 `sizeof` 验证闭包对象大小不变。
2. **递归题**：用 deducing this 实现一个递归 Lambda，计算给定目录树中所有子目录的文件数量。和 `std::function` 版本对比写法。
3. **管线题**：用 `std::views::filter` 和 `std::views::transform` 把传统的 erase-remove + transform 点云预处理改写成 ranges 管线，并讨论何时需要物化结果。

---

## 10.10 回调与算法选型：把工具放在正确层次 ⭐⭐⭐

### 回顾：本章工具不是互相替代

| 工具 | 适合层次 | 不适合场景 |
|------|----------|------------|
| 函数指针 | C 接口、无状态回调 | 有状态闭包 |
| 成员函数指针 | 底层反射/绑定 | 直接作为现代回调接口 |
| `std::bind` | 旧代码兼容 | 新代码大量参数标记 |
| Lambda | 局部回调、算法谓词 | 需要长期统一存储且类型不同 |
| `std::function` | 回调列表、事件系统 | 高频内层算法谓词 |
| 虚函数 | 开放算法接口 | 每点热循环 |
| 模板参数 | 高性能策略 | 运行时插件选择 |

### 机器人系统中的典型组合

ROS2 节点订阅：

```cpp
sub_ = create_subscription<PointCloudMsg>(
    "/points",
    10,
    [this](PointCloudMsg::SharedPtr msg) {
        onPointCloud(std::move(msg));
    });
```

点云过滤：

```cpp
points.erase(std::remove_if(points.begin(), points.end(),
                            [max_range](const PointXYZI& p) {
                                return squaredNorm(p) > max_range * max_range;
                            }),
             points.end());
```

事件系统：

```cpp
CallbackManager manager;
manager.subscribe([](const PointCloud& cloud) {
    std::cout << cloud.size() << "\n";
});
```

算法插件：

```cpp
std::unique_ptr<Registration> registration = makeRegistration(config.type);
registration->align(source, target);
```

每个工具在自己的层次上都合理。混用错误才会出问题，例如把每点距离计算存成 `std::function`，或把运行时插件写成模板参数。

> **本质洞察**：现代 C++ 的抽象工具不是“越新越好”，而是“越贴合变化维度越好”。Lambda 适合局部行为，`std::function` 适合存储回调，虚函数适合开放接口，模板适合热路径策略。

### 常见陷阱

> ⚠️ **层次陷阱：把局部 Lambda 提升成全局架构接口**
>
> Lambda 很适合局部操作，但大型模块边界仍需要命名清晰的接口、生命周期约束和测试入口。

> ⚠️ **性能陷阱：把所有策略都放进 `std::function`**
>
> `std::function` 提供运行时统一存储，但会隐藏具体类型。高频内核优先模板参数或函数对象。

### 练习

1. **选型题**：为 ROS2 订阅、点云过滤、插件注册、每点距离计算分别选择回调/策略表达方式。
2. **重构题**：把一个 `std::function` 每点距离计算改成模板策略，并说明调用端变化。
3. **综合题**：实现一个点云处理流水线：订阅回调用 Lambda，过滤用 STL 算法，处理完成后通过 `CallbackManager` 发布。
4. **跨章综合题**：设计一个完整的 `PointCloudPipeline` 类，综合运用以下前置章节知识：RAII与智能指针 RAII 管理订阅句柄的生命周期，移动语义与完美转发 移动语义在管线各阶段之间零拷贝传递点云，错误处理与异常安全 的 `expected` 风格表达预处理失败（点数不足、全部为 NaN），运算符重载 的 `operator()` 实现可配置的距离过滤器函数对象。要求：Lambda 回调捕获方式必须安全（说明为什么选值捕获或引用捕获），STL 算法替代所有手写循环，错误路径不抛异常而是返回 `expected`。

---

## 10.11 工程案例：从 ROS2 回调到点云预处理流水线 ⭐⭐⭐

### 动机：真实代码不是单独使用一个 Lambda

前面每节都把问题拆开讲：回调、捕获、容器、算法、`std::function`。真实机器人系统中，这些工具会连续出现。一个 LiDAR 前端节点可能按下面的路径运行：

```text
ROS2 订阅回调
  │
  ▼
消息转换为内部点云容器
  │
  ▼
STL 算法清洗无效点
  │
  ▼
按距离和时间字段过滤
  │
  ▼
转换到世界坐标或机体系
  │
  ▼
触发回调列表，通知配准/建图/可视化模块
```

如果每一步都随手写循环和裸回调，代码会很快难以维护。更好的做法是让每一层表达自己的意图：订阅层只做消息入口，预处理层用 STL 算法表达数据流，事件层用 `std::function` 保存长期回调。

### 消息入口：Lambda 只做转发

订阅回调不应塞入完整点云处理逻辑。它的职责是把消息交给成员函数：

```cpp
class LidarFrontend {
public:
    void start() {
        subscription_ = createSubscription(
            "/points",
            [this](PointCloudMsg::SharedPtr msg) {
                onPointCloud(std::move(msg));
            });
    }

private:
    void onPointCloud(PointCloudMsg::SharedPtr msg) {
        PointCloud cloud = convertMessage(*msg);
        PointCloud processed = preprocess(std::move(cloud), config_);
        callbacks_.publish(processed);
    }

    Subscription subscription_;
    FrontendConfig config_;
    PointCloudCallbackManager callbacks_;
};
```

这里 Lambda 捕获 `this`，但它不拥有节点。是否安全取决于 `subscription_` 和 `LidarFrontend` 的生命周期关系。若订阅对象由节点成员管理，且析构顺序清晰，这种写法通常可以接受；若回调可能进入线程池延迟执行，就要改用 `weak_ptr`。

### 预处理：让算法名表达意图

预处理函数可以拆成几个小步骤：

```cpp
PointCloud preprocess(PointCloud cloud, const FrontendConfig& config) {
    removeInvalidPoints(cloud);
    removeOutOfRangePoints(cloud, config.min_range, config.max_range);
    sortByTimestampIfNeeded(cloud);
    return cloud;
}
```

每个函数内部用 STL 算法：

```cpp
void removeInvalidPoints(PointCloud& cloud) {
    cloud.erase(std::remove_if(cloud.begin(), cloud.end(),
                               [](const PointXYZI& p) {
                                   return !std::isfinite(p.x) ||
                                          !std::isfinite(p.y) ||
                                          !std::isfinite(p.z);
                               }),
                cloud.end());
}
```

距离过滤：

```cpp
void removeOutOfRangePoints(PointCloud& cloud,
                            double min_range,
                            double max_range) {
    const double min2 = min_range * min_range;
    const double max2 = max_range * max_range;

    cloud.erase(std::remove_if(cloud.begin(), cloud.end(),
                               [min2, max2](const PointXYZI& p) {
                                   const double r2 = squaredNorm(p);
                                   return r2 < min2 || r2 > max2;
                               }),
                cloud.end());
}
```

时间排序：

```cpp
void sortByTimestampIfNeeded(PointCloud& cloud) {
    if (std::is_sorted(cloud.begin(), cloud.end(),
                       [](const PointXYZI& a, const PointXYZI& b) {
                           return a.time < b.time;
                       })) {
        return;
    }

    std::sort(cloud.begin(), cloud.end(),
              [](const PointXYZI& a, const PointXYZI& b) {
                  return a.time < b.time;
              });
}
```

这个写法看起来比一个大循环多了几个函数，但每个函数的目的清楚，后续插入统计、测试和性能计时更容易。

### 事件层：`std::function` 保存长期回调

一个前端可能有多个下游：

- 配准模块需要处理去畸变后的点云。
- 建图模块需要更新局部地图。
- 可视化模块需要发布 RViz marker。
- 诊断模块需要统计点数和耗时。

这些下游可以注册回调：

```cpp
class PointCloudCallbackManager {
public:
    using Callback = std::function<void(const PointCloud&)>;

    int subscribe(Callback callback) {
        callbacks_.push_back(Slot{
            .id = next_id_++,
            .callback = std::move(callback)
        });
        return callbacks_.back().id;
    }

    void publish(const PointCloud& cloud) const {
        for (const auto& slot : callbacks_) {
            if (slot.callback) {
                slot.callback(cloud);
            }
        }
    }

private:
    struct Slot {
        int id = 0;
        Callback callback;
    };

    int next_id_ = 0;
    std::vector<Slot> callbacks_;
};
```

这里用 `std::function` 是合理的，因为回调需要长期保存，而且不同下游的可调用对象类型不同。如果只是调用一次 STL 算法，就不应该先包成 `std::function`。

### 失败模式：回调中修改回调列表

一个容易忽略的问题是：回调执行期间，如果某个回调又订阅或取消订阅，会不会让 `callbacks_` 迭代器失效？

```cpp
manager.subscribe([&manager](const PointCloud& cloud) {
    manager.subscribe([](const PointCloud&) {
        // ...
    });
});
```

如果 `publish()` 正在遍历 `callbacks_`，内部 `push_back` 可能导致 `vector` 重新分配，使当前迭代器失效。简单系统可以规定“回调中不得修改订阅列表”；更稳的系统可以发布前复制回调快照：

```cpp
void publish(const PointCloud& cloud) const {
    auto callbacks = callbacks_;

    for (const auto& slot : callbacks) {
        if (slot.callback) {
            slot.callback(cloud);
        }
    }
}
```

复制 `std::function` 有成本，也可能分配或抛异常。若回调很多或频率很高，可以改成 `shared_ptr<const CallbackList>` 不可变快照，或者把订阅变更放进延迟队列，在非热路径统一提交。教学重点是：回调系统不仅是“存一个函数”，还要定义执行期间的修改规则和异常边界。

### 失败模式：Lambda 捕获大型配置副本

下面的捕获可能无意复制大型配置：

```cpp
auto filter = [config](const PointXYZI& p) {
    return squaredNorm(p) < config.max_range * config.max_range;
};
```

如果 `config` 很大，值捕获会让 Lambda 对象变大。更好的做法是只捕获需要的字段：

```cpp
const double max2 = config.max_range * config.max_range;

auto filter = [max2](const PointXYZI& p) {
    return squaredNorm(p) < max2;
};
```

这同时减少重复计算和闭包大小。

### 常见陷阱

> ⚠️ **架构陷阱：订阅回调里写完整业务逻辑**
>
> 回调入口应尽量薄。把转换、过滤、排序、发布拆成命名函数，测试和性能分析会更直接。

> ⚠️ **迭代器陷阱：回调发布期间修改回调列表**
>
> `vector` 增删可能使迭代器失效。需要明确禁止重入修改，或采用快照/延迟修改策略。

### 练习

1. **重构题**：把一个 80 行的 `onPointCloud()` 拆成消息转换、预处理、发布三层，每层只做一件事。
2. **回调题**：实现一个支持取消订阅的 `CallbackManager`。说明发布期间取消当前回调会如何处理。
3. **性能题**：比较捕获整个配置对象和只捕获 `max_range` 标量时 Lambda 的 `sizeof`。

这一节把本章工具串成一个前端节点。下一节再从调试角度补一层：当回调和 STL 算法出问题时，应该从哪里定位。

---

## 10.12 调试路径：回调生命周期、算法谓词与并行数据竞争 ⭐⭐⭐

### 回调没有触发

先区分三类问题：

| 现象 | 常见原因 | 检查方向 |
|------|----------|----------|
| 回调从未触发 | 订阅对象生命周期太短 | 订阅句柄是否保存为成员 |
| 回调偶尔不触发 | QoS、队列、线程调度 | 消息频率、队列深度、executor |
| 回调触发后崩溃 | 捕获对象悬空 | `[this]`、引用捕获、异步执行 |

在 C++ 层面，最常见的是订阅句柄被局部变量持有：

```cpp
void start() {
    auto sub = createSubscription("/points", callback);
}  // sub 析构，订阅消失
```

应保存为成员：

```cpp
class Node {
    Subscription sub_;
};
```

### STL 算法结果不对

定位算法问题时，先检查谓词是否满足算法要求：

| 算法 | 谓词要求 | 常见错误 |
|------|----------|----------|
| `remove_if` | 返回 true 表示删除 | 条件写反 |
| `partition` | 返回 true 表示前半区 | 误以为保持顺序 |
| `sort` | 严格弱序 | NaN、非对称比较 |
| `find_if` | true 表示找到 | 捕获阈值过期 |
| `accumulate` | 累积函数返回新累积值 | 初始值类型错误 |

对复杂 Lambda，可以先把它提取成命名函数或函数对象，单独写几个输入输出测试。Lambda 不应该成为无法测试的匿名黑盒。

### 并行数据竞争

下面写法有数据竞争：

```cpp
std::size_t valid_count = 0;

std::for_each(std::execution::par,
              points.begin(), points.end(),
              [&valid_count](const PointXYZI& p) {
                  if (isValid(p)) {
                      ++valid_count;
                  }
              });
```

修复方式包括：

1. 使用原子计数。
2. 使用并行归约。
3. 每个线程局部统计后合并。
4. 直接用串行算法，如果数据量小。

原子版本：

```cpp
std::atomic<std::size_t> valid_count = 0;

std::for_each(std::execution::par,
              points.begin(), points.end(),
              [&valid_count](const PointXYZI& p) {
                  if (isValid(p)) {
                      valid_count.fetch_add(1, std::memory_order_relaxed);
                  }
              });
```

原子也不是总是最快。大量高频原子加法会造成争用，归约通常更适合统计。

### 调试清单

| 问题 | 第一步 | 第二步 | 第三步 |
|------|--------|--------|--------|
| 回调悬空 | 检查捕获列表 | 检查对象生命周期 | 用 ASan/TSan |
| 过滤结果为空 | 打印谓词阈值 | 单测边界点 | 检查单位和坐标系 |
| 排序异常 | 检查比较器 | 过滤 NaN | 验证严格弱序 |
| 并行结果不稳定 | 查共享写 | 改串行复现 | TSan 检测 |
| 性能变差 | 看是否用了 `std::function` | profile 热路径 | 改模板策略 |

> **本质洞察**：Lambda 让局部行为更容易书写，但不会自动让生命周期、并发和算法语义变安全。越短的回调越需要清楚的所有权和执行时机约定。

### 练习

1. **调试题**：构造一个订阅句柄局部变量导致回调不触发的例子，修复为成员变量。
2. **并发题**：用 TSan 检查并行 `for_each` 中引用捕获计数器的数据竞争。
3. **谓词题**：为 `sort` 比较器写三个性质测试：反自反、非对称、传递性。

---

## 本章小结

本章从旧式回调工具讲到现代 Lambda 和 STL 算法，核心结论如下：

| 主题 | 核心结论 |
|------|----------|
| 函数重载 | 编译期静态选择，不是运行时多态 |
| 默认参数 | 便利接口，虚函数中要谨慎 |
| 函数指针 | 简单但无状态 |
| `std::bind` | 历史代码常见，新代码多用 Lambda |
| Lambda | 匿名函数对象，捕获决定状态和生命周期 |
| 引用捕获 | 同步短生命周期可用，异步场景危险 |
| 初始化捕获 | 适合移动所有权进入任务 |
| `std::function` | 类型擦除回调容器，有运行时成本 |
| STL 算法 | 用函数名表达遍历意图 |
| 容器选型 | 数据布局决定性能上限 |
| `optional/variant/span` | 分别表达可失败结果、封闭类型集合、非拥有视图 |
| 并行 STL | 简洁但必须避免数据竞争 |

本章最重要的工程判断是：可调用对象不是单一工具。局部行为用 Lambda，长期存储用 `std::function`，开放算法边界用虚函数，热路径策略用模板或函数对象。

---

## 累积项目：本章新增模块

本章为 Mini Registration 框架加入点云处理流水线和回调管理器。

### 点云流水线

```cpp
std::vector<Eigen::Vector3d> preprocess(
    std::vector<PointXYZI> points,
    const Eigen::Isometry3d& T_world_lidar,
    double max_range) {
    points.erase(std::remove_if(points.begin(), points.end(),
                                [](const PointXYZI& p) {
                                    return !std::isfinite(p.x) ||
                                           !std::isfinite(p.y) ||
                                           !std::isfinite(p.z);
                                }),
                 points.end());

    points.erase(std::remove_if(points.begin(), points.end(),
                                [max_range](const PointXYZI& p) {
                                    return squaredNorm(p) >
                                           max_range * max_range;
                                }),
                 points.end());

    std::vector<Eigen::Vector3d> output;
    output.reserve(points.size());

    std::transform(points.begin(), points.end(),
                   std::back_inserter(output),
                   [T_world_lidar](const PointXYZI& p) {
                       return T_world_lidar *
                              Eigen::Vector3d(p.x, p.y, p.z);
                   });

    return output;
}
```

### 回调管理器

```cpp
class PointCloudCallbackManager {
public:
    using Callback = std::function<void(std::span<const Eigen::Vector3d>)>;

    void subscribe(Callback callback) {
        callbacks_.push_back(std::move(callback));
    }

    void publish(std::span<const Eigen::Vector3d> cloud) const {
        for (const auto& callback : callbacks_) {
            if (callback) {
                callback(cloud);
            }
        }
    }

private:
    std::vector<Callback> callbacks_;
};
```

### 验收标准

| 检查项 | 合格标准 |
|--------|----------|
| 过滤 | 使用 erase-remove 删除无效点 |
| 捕获 | 阈值值捕获，异步任务不用 `[&]` |
| 回调 | `std::function` 只用于存储回调 |
| 算法 | 点云变换用 `std::transform` |
| 容器 | 输出 vector 预留容量 |
| 视图 | 发布接口用 `span` 表达非拥有 |
| 错误处理 | 空回调调用前检查 |

### 进阶要求

1. 给 `preprocess()` 增加 `std::optional` 返回值：当有效点太少时返回空。
2. 增加并行版本的强度归一化，确保没有数据竞争。
3. 用 `variant` 表达三类传感器测量，并用 `std::visit` 分发到对应预处理函数。

---

## 延伸阅读

| 资源 | 内容 | 难度 |
|------|------|------|
| cppreference: Lambda expressions | Lambda 语法、捕获规则、闭包类型和模板化 | ⭐⭐ |
| cppreference: `std::function` | 类型擦除机制、小对象优化和调用语义 | ⭐⭐ |
| cppreference: STL algorithms | 算法前置条件、迭代器类别和复杂度保证 | ⭐⭐ |
| cppreference: execution policies | `seq`、`par`、`par_unseq` 的语义差异和异常行为 | ⭐⭐⭐ |
| C++ Core Guidelines F.50-F.52 | Lambda 捕获、资源生命周期、算法优先于手写循环 | ⭐⭐ |
| ROS2 官方教程：订阅回调写法 | 对比 `std::bind` 和 Lambda 在 ROS2 节点中的实际用法 | ⭐⭐ |
| KISS-ICP、Patchwork++、Faster-LIO 源码 | 点云处理管线中 Lambda、STL 算法和并行循环的工程实践 | ⭐⭐⭐ |
| Anthony Williams, *C++ Concurrency in Action* | 异步任务中的捕获和生命周期，线程安全回调设计 | ⭐⭐⭐ |
| range-v3 / C++20 ranges 文档 | 管线式数据处理、惰性求值和视图组合 | ⭐⭐⭐⭐ |
| Scott Meyers, *Effective Modern C++*, Item 31-34 | Lambda 默认捕获、初始化捕获、`std::function` 替代方案 | ⭐⭐⭐ |

---

## 🔧 故障排查手册

| 现象 | 常见原因 | 检查方式 | 修复方式 |
|------|----------|----------|----------|
| Lambda 中变量值不是预期 | 值捕获创建时复制，后续外部修改不影响 | 打印捕获前后变量 | 改引用捕获或重新创建 Lambda |
| 异步任务崩溃 | 引用捕获局部变量或 `[this]` 悬空 | 检查任务是否晚于作用域执行 | 值捕获、移动捕获或 weak_ptr |
| `std::function` 调用抛异常 | 回调为空 | 调用前判断 `if (callback)` | 构造时保证有效或显式检查 |
| `remove_if` 后容器大小没变 | 忘记 `erase` | 打印 size 和新末尾距离 | 使用 erase-remove idiom |
| `accumulate` 结果类型错误 | 初始值类型不对 | 检查第三个参数类型 | 使用正确初始对象 |
| 并行 STL 结果不稳定 | Lambda 写共享变量导致数据竞争 | 查找引用捕获和共享写 | 使用归约、原子或线程局部数据 |
| `std::sort` 行为异常 | 比较器不满足严格弱序 | 检查 NaN、相等情况和不对称比较 | 过滤无效值，修正比较器 |
| `span` 悬垂 | 视图指向已销毁容器 | 检查底层数据生命周期 | 只把 span 用作短期参数 |

下一章进入 Eigen。回顾本章：STL 算法和 Lambda 让点云处理流水线更清晰；Eigen基础与SLAM数学预备 会把同样的思想带到矩阵和向量层面，讨论如何用 Eigen 做批量几何计算，并处理表达式模板、内存对齐和零拷贝映射这些机器人 C++ 必经问题。

### ROS2 回调中 Lambda 的工程实践总结

ROS2 是机器人 C++ 代码中 Lambda 使用最密集的场景之一。整理一下本章各节分散涉及的 ROS2 Lambda 要点：

**订阅回调的三代写法演进**：

| 时代 | 写法 | 优劣 |
|------|------|------|
| ROS1 / 早期 ROS2 | `std::bind(&Node::callback, this, _1)` | 占位符晦涩，错误信息差 |
| 现代 ROS2 | `[this](auto msg) { callback(msg); }` | 清晰，但 `this` 裸指针有生命周期风险 |
| 安全写法 | `[weak = weak_from_this()](auto msg) { if (auto self = weak.lock()) self->callback(msg); }` | 最安全，适合组件生命周期不确定的场景 |

**定时器回调中的 Lambda**：

```cpp
timer_ = create_wall_timer(
    100ms,
    [this]() {
        publishDiagnostics();
    });
```

定时器回调和订阅回调的生命周期约束相同——`this` 必须在回调执行期间存活。ROS2 的 `LifecycleNode` 状态转换中，定时器可能在节点 `deactivate` 后仍被触发一次，此时访问已释放的资源会崩溃。

**服务回调中的 Lambda**：

```cpp
service_ = create_service<SaveMap>(
    "save_map",
    [this](const SaveMap::Request::SharedPtr req,
           SaveMap::Response::SharedPtr res) {
        try {
            map_.save(req->path);
            res->success = true;
        } catch (const std::exception& e) {
            res->success = false;
            res->message = e.what();
        }
    });
```

服务回调结合了错误处理与异常安全中的边界捕获模式：异常不能逃出回调，错误信息通过服务响应字段返回。

> **反事实推理**：如果 ROS2 的回调接口不接受可调用对象（如 Lambda 和 `std::function`），而只接受虚函数派生类，每个订阅、定时器、服务都要写一个完整的类——一个有三个订阅的节点就需要四个类（节点类 + 三个回调类）。Lambda 把"行为定义"从"类定义"中解放出来，让一个节点类就能包含所有回调逻辑。这也是为什么 ROS2 教程从早期的 `std::bind` 全面转向 Lambda 写法。
