# C++20 Concepts 与 Policy-based Design

> **难度**：⭐⭐⭐～⭐⭐⭐⭐ | **建议用时**：2周 | **前置要求**：函数模板与类模板基础 函数模板与类模板基础、模板特化SFINAE与类型萃取 模板特化与类型萃取、变参模板折叠表达式与CRTP 变参模板与 CRTP、预处理器与宏 预处理器与宏

---

## 前置自测

> 📋 答不出 2 题以上时，先回顾 函数模板与类模板基础-预处理器与宏。

1. `std::enable_if_t<std::is_floating_point_v<T>, int> = 0` 为什么能让某个模板只对浮点类型参与重载？
2. `if constexpr` 和普通 `if` 的差异是什么？未选分支是否会实例化？
3. traits 和虚函数都能隐藏数据类型差异，它们分别适合哪个层次？
4. 宏能生成字段名和注册符号，Concepts 能不能替代这种能力？为什么？
5. 一个配准框架有数据访问、最近邻搜索、距离度量、优化器四个变化轴，为什么不应该写成一个巨大继承树？
6. `RotationLike` concept 能检查 `matrix()` 和 `inverse()`，为什么仍不能证明矩阵是合法旋转？

---

## 本章目标

学完本章，你将能够：

- 用 requirement kinds 理解 `requires` 表达式：简单要求、类型要求、复合要求、嵌套要求。
- 解释 constraint satisfaction、constraint normalization 和 subsumption 对模板重载选择的影响。
- 从 SFINAE 迁移到 Concepts，并比较同一需求下接口表达和错误诊断的差异。
- 清楚说明 Concepts 只能表达语法和类型契约，不能证明坐标系、单位、扰动方向、物理可行性等数学语义。
- 在机器人泛型接口中合理使用 `constexpr`、`consteval`、`if constexpr` 和 Ranges，而不让它们分散主线。
- 从真实算法变化轴出发设计 Policy：数据访问、搜索、距离、残差、归约、优化策略。
- 理解 Policy-based Design 的组合爆炸、显式实例化、ABI 和构建成本。
- 建立 Concepts + traits + policy + type erasure 的分层判断：热路径编译期，系统边界运行时。
- 为 Mini Registration 项目设计策略组合、正负例、错误诊断和测试目标。

**本章在课程中的位置**：函数模板与类模板基础 教模板，模板特化SFINAE与类型萃取 教 traits 和 SFINAE，变参模板折叠表达式与CRTP 教 CRTP，预处理器与宏 教宏在类型系统之前的边界。本章把这些工具收束成现代 C++ 泛型设计：Concepts 负责把模板参数要求写在接口上，Policy-based Design 负责把算法变化拆成可组合策略，traits 负责适配外部数据类型，type erasure 和虚接口负责运行时边界。

回顾 模板特化SFINAE与类型萃取：SFINAE 通过模板替换失败来排除不满足条件的重载，但失败信息往往指向替换深处而非接口表面。本章的 Concepts 用 `requires` 表达式把同样的约束写成可读的声明式接口。回顾 预处理器与宏：宏在预处理阶段操作 token，能做 Concepts 做不到的事（生成标识符、条件编译），但无法表达类型契约。Concepts 和宏的分界线就是"类型系统之内"和"类型系统之外"的边界。

理解本章需要把握一个核心判断框架：每种工具解决特定层次的变化。运行时配置变化交给虚函数和工厂；编译期算法变化交给 Policy 和 traits；模板入口的类型要求交给 Concepts；类型系统之外的 token 操作交给宏。掌握这个分层判断，比记住任何一种语法都更重要。

---

## 16.1 Concepts 的动机：把隐含模板要求提到接口上 ⭐⭐⭐⭐

### Concepts 的历史演进：从 Stroustrup 的二十年愿景到 C++20 的落地

Concepts 不是一个突然出现在 C++20 中的新特性。它的历史可以追溯到 C++ 模板系统诞生之初的一个根本缺陷，以及 Bjarne Stroustrup 等人近二十年的持续推动。

**模板系统的"先天缺陷"。** C++ 模板最初的设计（1988-1990，由 Stroustrup 引入）是"无约束的"——模板参数 `T` 可以是任何类型，编译器在实例化模板后才检查 `T` 是否满足函数体中的使用要求。这种设计被称为"鸭子类型"（duck typing）：如果 `T` 看起来像鸭子（有正确的成员函数），叫起来像鸭子（返回正确的类型），那它就是鸭子。问题在于：当 `T` 不是鸭子时，错误信息指向的是"叫声不对"（模板体内的某个表达式编译失败），而不是"你传进来的根本不是鸭子"（类型不满足模板的意图）。

**第一次尝试：C++0x Concepts（2005-2009）。** Stroustrup 和 Andrew Lumsdaine 在 2003-2005 年间提出了第一版 Concepts 提案。这个版本非常强大——它支持 concept maps（允许适配不满足 concept 的类型）、axioms（公理，用于表达语义约束）和 late checking（延迟检查）。但这个版本过于复杂，实现难度极大。2009 年 C++ 标准委员会投票将其从 C++0x（后来成为 C++11）中移除。Stroustrup 后来回忆说："我们试图一次解决太多问题。"

**第二次尝试：Concepts Lite（2013-2020）。** 从第一次失败中吸取教训后，Andrew Sutton 和 Stroustrup 提出了简化版——Concepts Lite。它只保留了核心的约束检查功能，去掉了 concept maps 和 axioms。设计哲学是"只做编译器能可靠做到的事"——检查表达式合法性和类型关系，不试图检查数学语义。这个简化版在 2017 年进入 C++ TS（技术规范），2020 年正式进入 C++20 标准。

**从 SFINAE 到 Concepts 的本质跃迁。** SFINAE（Substitution Failure Is Not An Error）是 C++98 就有的机制。它的原理是：在模板参数替换失败时，不报编译错误，而是从候选重载集中移除这个模板。程序员通过巧妙地构造"会在特定类型上替换失败的表达式"（如 `std::enable_if_t`、`std::void_t`），间接实现了模板约束。但 SFINAE 本质上是一种"hack"——它利用编译器的错误恢复机制来实现约束检查，而非使用专门设计的约束语言。SFINAE 的约束写在模板参数列表或返回类型中，语法晦涩，错误信息指向替换细节而非接口意图，多个约束之间没有偏序关系（需要手动用互斥条件消解二义性）。

Concepts 与 SFINAE 的本质差异不在于"能做什么"（两者能表达的约束集合几乎相同），而在于"怎么表达"和"错误信息指向哪里"。Concepts 把约束从"实现层的技巧"提升为"接口层的声明"——这和函数参数从 K&R 风格（无类型声明）到 ANSI 风格（显式类型声明）的跃迁是同一类进步。

### 工程问题：模板错误离真实原因太远

一个点云质心函数看似简单：

```cpp
template <typename Cloud>
Eigen::Vector3d computeCentroid(const Cloud& cloud) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();

    for (std::size_t i = 0; i < cloud.size(); ++i) {
        sum += cloud[i];
    }

    return sum / static_cast<double>(cloud.size());
}
```

它隐含要求：

1. `cloud.size()` 合法，并能转换为 `std::size_t`。
2. `cloud[i]` 合法。
3. `cloud[i]` 能作为三维向量累加。
4. `cloud.size()` 不是 0，否则除法无意义。

前 3 条是语法和类型要求，可以由 Concepts 表达；第 4 条是运行时数据条件，需要运行时检查。没有 Concepts 时，错误可能在 `sum += cloud[i]` 深处爆出，调用者很难从错误栈看出接口真正需要什么。

### 反面失败：无约束模板把要求藏在函数体里

```cpp
struct Image {
    int width;
    int height;
};

Image image{640, 480};
auto c = computeCentroid(image);
```

编译器会抱怨 `image.size()` 不存在，或者 `operator[]` 不存在。错误发生在模板实例化后的函数体内部，而不是函数签名处。这个问题在机器人项目中很常见：点云、轨迹、状态、残差、李群类型只要缺一个成员，错误信息就可能穿过 Eigen、PCL、Ceres 的模板层。

### 抽象不变量：Concept 是模板入口处的语法契约

```cpp
template <typename Cloud>
concept IndexableVector3Cloud =
    requires(const Cloud& cloud, std::size_t i) {
        { cloud.size() } -> std::convertible_to<std::size_t>;
        { cloud[i] } -> std::convertible_to<Eigen::Vector3d>;
    };

template <IndexableVector3Cloud Cloud>
Eigen::Vector3d computeCentroid(const Cloud& cloud) {
    const auto n = static_cast<std::size_t>(cloud.size());
    if (n == 0) {
        throw std::invalid_argument("centroid requires a non-empty cloud");
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < n; ++i) {
        sum += static_cast<Eigen::Vector3d>(cloud[i]);
    }
    return sum / static_cast<double>(n);
}
```

现在函数签名告诉调用者：`Cloud` 必须是可按索引访问的三维点云。错误类型会在约束层失败，而不是进入函数体后失败。

这个改进的意义不只是"错误信息更好"。在大型项目中，模板函数的调用者往往不是模板的作者——可能是另一个团队、另一个时间点、另一个代码库。没有 Concepts 时，调用者必须阅读函数体才能推断类型要求。有了 Concepts，要求写在签名上，就像函数参数类型一样显而易见。这对代码审查、接口文档和 IDE 提示都有实质帮助。

如果 Concepts 不存在会怎样？C++17 的替代方案是 SFINAE + `enable_if` + 检测惯用法。这能实现同样的约束效果，但代码的可读性和错误诊断都差得多。一个复杂的 `enable_if` 表达式可能需要十几秒才能理解，而同等功能的 `requires` 子句一眼就能看出要求什么。Concepts 不是引入了新的能力，而是把已有能力从"只有模板专家能写"降低到"中级 C++ 工程师能读懂"。

### 规则推导：Concepts 改善接口，不自动改善语义

`IndexableVector3Cloud` 不能证明：

- 点坐标是在 `map`、`odom` 还是 `base_link`。
- 单位是米、毫米还是归一化坐标。
- 点云是否已经去畸变。
- 时间戳是否和 IMU 对齐。
- 点云是否非空。

这些仍要靠类型命名、接口文档、运行时检查和测试样例表达。Concepts 的价值是把”能否编译地调用”前置，不是把数学和物理约定全交给编译器。

一个有用的思维实验是：想象两个完全不同的点云——一个在世界坐标系下单位是米，另一个在相机坐标系下单位是毫米。两者都满足 `IndexableVector3Cloud`，因为 concept 只检查”有 `size()` 和 `operator[]`”。但把它们混在同一个配准调用中会产生完全错误的结果。Concepts 挡住了”传入 `Image` 类型”这种明显的接口错误，但挡不住”传入坐标系不一致的点云”这种语义错误。这就是为什么工程上需要三层防线：concept 挡语法错误、命名约定（如 `points_world`、`points_camera`）防语义混淆、运行时断言检查数值合理性。

### 工程边界：接口要小，不要把当前实现塞进 concept

如果质心函数只需要 `.size()` 和 `operator[]`，就不要要求 `cloud.points`、`cloud.header`、`cloud.frame_id`。过大的 concept 会把本来可用的类型排除在外。Concept 应表达算法最小需求，而不是某个项目当前数据结构的全部细节。

### 代码验证：正例和负例

```cpp
struct VecCloud {
    std::vector<Eigen::Vector3d> points;

    std::size_t size() const { return points.size(); }
    const Eigen::Vector3d& operator[](std::size_t i) const {
        return points[i];
    }
};

struct BadCloud {
    int width = 0;
    int height = 0;
};

static_assert(IndexableVector3Cloud<VecCloud>);
static_assert(!IndexableVector3Cloud<BadCloud>);
```

正例证明接口能接受目标类型；负例证明错误类型在约束层被拒绝。教学项目应把这两类测试都写出来。

> **本质洞察**：Concepts 的本质不是"给模板加更多语法"，而是把模板的隐式契约变成显式契约。没有 Concepts 时，模板参数的要求藏在函数体深处——调用者只有在模板实例化失败后才知道缺了什么。Concepts 把这些要求提到函数签名上，就像函数声明中的参数类型一样：它是接口的一部分，不是实现的意外副产品。

Concepts 之于模板参数，就像类型声明之于函数参数。C 语言时代，函数声明可以不写参数类型（K&R 风格），编译器在调用时才发现类型不匹配。现代 C++ 的函数声明必须写明参数类型，错误在调用处就能捕获。Concepts 对模板参数做了同样的事——把"进入函数体后才发现不对"提前到"约束检查时就拒绝"。

> ⚠️ **编程陷阱：concept 接口过大导致可复用性下降**
> **错误做法**：为质心计算函数要求 `cloud.points`、`cloud.header`、`cloud.frame_id`、`cloud.sensor_origin`，而实际只需要 `.size()` 和 `operator[]`。
> **现象**：其他满足最小需求的点云容器（如 `std::vector<Eigen::Vector3d>`）被拒绝。
> **根本原因**：concept 应表达算法的最小需求，而不是某个具体数据结构的完整接口。过大的 concept 等于把实现细节泄漏到接口上。
> **正确做法**：只要求算法真正使用的操作。如果不同算法需要不同子集，拆成多个小 concept 组合。

> 💡 **概念误区：认为 Concepts 能替代运行时检查**
> **新手想法**："既然 `IndexableVector3Cloud` 约束了 `cloud.size()` 合法，那就不需要在函数体里检查空点云了。"
> **实际上**：Concepts 只检查表达式是否合法（语法层），不检查返回值是否合理（运行时层）。`cloud.size() == 0` 导致的除零错误是运行时问题，Concepts 无法预防。
> **正确理解**：Concepts 管"能不能编译"，运行时检查管"运行时行为是否正确"。两者互补，不能互相替代。

### 练习

1. **[分析题]**：`IndexableVector3Cloud` 为什么用 `std::convertible_to<Eigen::Vector3d>` 而不是 `std::same_as<Eigen::Vector3d>`？列出至少两种会被 `same_as` 拒绝但 `convertible_to` 接受的合法返回类型。
2. **[代码题]**：为你的机器人项目（或 Mini Registration）定义一个 `TimestampedCloud` concept，要求除了 `size()` 和 `operator[]` 外还有 `timestamp()` 方法。编写正例和负例的 `static_assert`。
3. **[跨章综合题]**：预处理器与宏 的宏能在预处理阶段操作 token 生成字段名；Concepts 能在编译阶段约束类型接口。解释为什么"从字段名生成 traits 特化"只能用宏，而"检查 traits 是否满足算法需求"可以用 Concepts。两者如何协作？

---

## 16.2 Requirement kinds：`requires` 表达式到底检查什么 ⭐⭐⭐⭐⭐

### 工程问题：一个策略接口由多种要求组成

配准内核里的距离策略可能需要：

- 有嵌套类型 `Scalar`。
- 能用源点和目标点调用。
- 返回值能转换成 `double`。
- 某些维度在编译期满足要求。

这些不是同一种要求。Concepts 把它们分成 requirement kinds。

### requires 表达式的四种要求形式——各自检查什么？

理解 `requires` 表达式的关键是认识到它不执行任何代码——它只在一个假想的上下文中检查表达式是否**能被编译**。这和 SFINAE 的检测惯用法（`std::void_t` + `decltype`）在本质上做同样的事情，但语法更直观、错误诊断更清晰。

C++20 标准定义了四种 requirement kinds，每种检查不同层次的类型约束：

**简单要求（Simple requirement）**：检查一个表达式是否合法——能否在假想上下文中通过编译。它不评估表达式的值，不检查返回类型，只问"这个表达式能写出来吗？"。这是最宽松的检查形式，适合"只需要存在某个操作"的场景。

**类型要求（Type requirement）**：检查一个类型名是否存在。通常用于验证嵌套类型（`typename T::value_type`）、别名（`typename T::Scalar`）或特化后的类型是否有效。类型要求不检查类型的性质（如是否是整数、是否可默认构造），只检查类型名本身是否能被解析。

**复合要求（Compound requirement）**：检查一个表达式是否合法，并且可以进一步约束其返回类型。语法 `{ expr } -> concept<Args...>;` 先检查 `expr` 合法，再把 `decltype((expr))` 代入 concept 模板参数检查返回类型。这是最常用的形式，因为大多数接口约束不只是"函数存在"，还要"返回类型合适"。

**嵌套要求（Nested requirement）**：在 `requires` 表达式内部嵌套另一个编译期布尔约束。语法 `requires constraint-expression;`。这让你可以在同一个 `requires` 块中组合表达式检查和布尔约束，而不需要拆成多个 concept。

```cpp
template <typename Policy, typename SourcePoint, typename TargetPoint>
concept CallableDistancePolicy =
    requires(const Policy& policy,
             const SourcePoint& source,
             const TargetPoint& target) {
        // 类型要求：Policy 必须有嵌套类型 Scalar
        typename Policy::Scalar;
        // 复合要求：policy(source, target) 合法且返回值可转换为 double
        { policy(source, target) } -> std::convertible_to<double>;
        // 嵌套要求：Scalar 必须是浮点类型
        requires std::floating_point<typename Policy::Scalar>;
    };

template <typename Policy>
concept ResettablePolicy =
    requires(Policy& policy) {
        // 简单要求：对非常量策略对象调用 reset() 合法
        policy.reset();
    };
```

| 形式 | 名称 | 检查什么 | 适用场景 |
|------|------|---------|---------|
| `policy.reset();` | 简单要求 | 表达式能否编译 | 只需要操作存在 |
| `typename Policy::Scalar;` | 类型要求 | 嵌套类型是否存在 | 需要关联类型 |
| `{ policy(source, target) } -> std::convertible_to<double>;` | 复合要求 | 表达式合法且返回类型满足约束 | 需要控制返回类型 |
| `requires std::floating_point<typename Policy::Scalar>;` | 嵌套要求 | 编译期布尔约束成立 | 组合布尔条件 |

**四种要求之间的递进关系**：简单要求是最基本的"存在性检查"；类型要求检查"有没有这个类型"；复合要求在简单要求基础上加了"返回类型约束"；嵌套要求可以表达任意的编译期布尔条件。从简单到复合再到嵌套，检查越来越精细，约束越来越强。工程上应该选择刚好够用的形式——约束太弱会放行不该通过的类型，约束太强会拒绝应该接受的类型。

### 反面失败：复合要求写得过窄

```cpp
template <typename Traits, typename Cloud>
concept StrictCloudTraits =
    requires(const Cloud& cloud, std::size_t i) {
        { Traits::point(cloud, i) } -> std::same_as<Eigen::Vector3d>;
    };
```

这个约束要求返回类型精确等于 `Eigen::Vector3d`。如果 traits 返回 `const Eigen::Vector3d&`、`Eigen::Map<const Eigen::Vector3d>` 或可转换代理对象，就会被拒绝。很多点云算法只需要“能转成三维向量”，应写成：

```cpp
template <typename Traits, typename Cloud>
concept CloudTraits =
    requires(const Cloud& cloud, std::size_t i) {
        { Traits::size(cloud) } -> std::convertible_to<std::size_t>;
        { Traits::point(cloud, i) } -> std::convertible_to<Eigen::Vector3d>;
    };
```

### 抽象不变量：Concept 检查表达式合法性和类型关系

Concepts 不运行代码。`requires(const T& x) { x.size(); }` 只检查表达式是否能在假想上下文中被编译；它不创建对象，不调用函数，不检查返回值是否合理。`std::predicate<F, Args...>` 要求可调用结果能作为布尔测试使用，但不证明谓词满足数学上的自反性、传递性或稳定性。

因此，机器人泛型接口通常要把契约拆成三层：

| 契约层 | 能否用 Concept 表达 | 示例 |
|--------|--------------------|------|
| 语法契约 | 适合 | 有 `size()`、能调用 `point(i)`、返回值能转成 `Vector3d` |
| 类型契约 | 适合一部分 | 标量是浮点、维度是编译期常量、策略返回 `double` |
| 数学语义契约 | 不适合单靠 Concept | 坐标系一致、四元数单位化、扰动方向匹配、协方差半正定 |
| 性能契约 | 通常不能直接表达 | 是否内联、是否分配内存、是否触发 Eigen 临时对象 |

这张表决定了 Concept 的写法：入口处应阻止明显错误的类型进入算法，但不要把“看起来像数学证明”的名字赋给只检查语法的 concept。比如 `RotationLike` 可以要求 `matrix()` 存在、返回 `Matrix3d`，但不能证明这个矩阵正交且行列式为 1；真正的 SO(3) 不变量仍要靠运行时性质测试。

### 规则推导：`requires` 子句、简写模板、后置 requires

单参数 concept 可以写成简写模板参数，例如：

```cpp
template <std::floating_point T>
T huberWeight(T residual, T delta);
```

`CloudTraits` 这类多参数 concept 通常用 `requires` 子句更清晰：

```cpp
template <typename Cloud>
requires CloudTraits<cloud_traits<Cloud>, Cloud>
Eigen::Vector3d centroid(const Cloud& cloud);
```

复杂组合建议使用 `requires` 子句，让读者先看模板参数，再看约束逻辑。

### 工程边界：Eigen 表达式和引用语义要谨慎

机器人代码里常见 `Eigen::Ref`、`Eigen::Map`、表达式模板。`std::convertible_to<Eigen::Vector3d>` 可读性好，但可能引入临时对象；`std::same_as<const Eigen::Vector3d&>` 性能意图明确，但过窄。工程上通常先写最小可用 concept，再通过性能测试决定是否收紧返回类型。

### 代码验证：把要求拆小

```cpp
template <typename T>
concept HasSize =
    requires(const T& value) {
        { value.size() } -> std::convertible_to<std::size_t>;
    };

template <typename T>
concept HasIndexAccess =
    requires(const T& value, std::size_t i) {
        value[i];
    };

template <typename T>
concept IndexableCloud = HasSize<T> && HasIndexAccess<T>;
```

拆小 concept 有两个好处：错误诊断更具体，算法能按最小需求复用。

> ⚠️ **编程陷阱：复合要求的返回类型约束写得过窄**
> **错误做法**：`{ Traits::point(cloud, i) } -> std::same_as<Eigen::Vector3d>;`
> **现象**：如果 traits 返回 `const Eigen::Vector3d&`、`Eigen::Map<const Eigen::Vector3d>` 或可转换的代理对象，concept 检查失败。算法本来能正确工作的类型被拒绝。
> **根本原因**：`std::same_as` 要求精确类型匹配，包括 cv 限定和引用。大多数数值算法只关心"能当 `Vector3d` 用"，不关心是值还是引用。
> **正确做法**：使用 `std::convertible_to<Eigen::Vector3d>` 表达"能转成 `Vector3d`"。只在性能关键场景中，经测量后才考虑收紧到 `same_as`。

> 🧠 **思维陷阱：认为 concept 名字能表达数学语义**
> **新手想法**："`CallableDistancePolicy` 这个名字说明它是距离度量，所以满足三角不等式。"
> **实际上**：concept 只检查 `policy(source, target)` 表达式合法且返回值能转成 `double`。它不检查返回值是否非负、是否满足对称性、是否满足三角不等式。一个返回随机数的 functor 也能通过这个 concept。
> **正确思维**：concept 名字是给人看的文档，不是给编译器用的数学证明。数学性质需要运行时测试或文档约定来保证。

### 练习

1. **[分析题]**：解释简单要求、类型要求、复合要求、嵌套要求分别检查什么。各给一个机器人项目中的例子。
2. **[代码题]**：编写一个 `ResettableDistancePolicy` concept，组合 `CallableDistancePolicy` 和 `ResettablePolicy`。编写一个满足它的策略类和一个不满足的策略类，用 `static_assert` 验证。
3. **[跨章综合题]**：模板特化SFINAE与类型萃取 用 SFINAE 和 `std::void_t` 做表达式检测。本节用 `requires` 表达式做同样的事。把一个 模板特化SFINAE与类型萃取 中的检测惯用法改写为 concept，对比代码量和错误信息质量。

---

## 16.3 Constraint satisfaction、normalization 与 subsumption ⭐⭐⭐⭐⭐

> **这一节解决什么问题**：当多个 concept 约束的重载同时匹配时，编译器怎么决定"谁更具体"？subsumption 规则不是需要死记硬背的语法细节，而是编译器判断"约束 A 是否逻辑上蕴含约束 B"的推理机制。理解它，就能设计出不会二义的 concept 层次。

### 从直觉到规则：编译器怎么判断哪个约束更强

在日常生活中，"更具体"是直觉性的判断。"会游泳的哺乳动物"比"哺乳动物"更具体，因为前者在后者的基础上多了一个条件。编译器面对 concept 重载时做的也是同样的推理——但编译器不理解英文单词的含义，它只能通过约束表达式的逻辑结构来判断包含关系。

考虑这样一个思维实验：如果有两个函数模板，一个要求"类型 T 有 `size()` 方法"，另一个要求"类型 T 有 `size()` 方法且有 `begin()`/`end()` 方法"。对于 `std::vector<int>` 这种同时满足两个条件的类型，编译器应该选哪个？直觉告诉我们应该选第二个——因为它的条件更严格，能匹配的类型范围更小，因此对当前类型的描述更精确。

但编译器怎么从约束表达式中推导出这个"更严格"的关系？这就需要三层机制的协作：先检查约束是否满足（satisfaction），再把约束分解成最小单元（normalization），最后比较哪个约束逻辑上蕴含另一个（subsumption）。

回忆 模板特化SFINAE与类型萃取 的 SFINAE——在没有 Concepts 的时代，如果你想让两个 `enable_if` 约束的重载共存，必须手动写互斥条件（如 `enable_if<!is_range_v<T>>` 在其中一个重载上），否则两个重载都满足时编译器直接报二义性错误。SFINAE 完全没有"谁更具体"的偏序判断能力——它只知道"满足/不满足"，不知道"更满足/不那么满足"。Concepts 的 subsumption 机制正是为了解决这个问题：让编译器能自动推断约束之间的蕴含关系，从而在多个满足的重载中选出最具体的那个。

### 工程问题：为什么某些 concept 重载会二义

考虑两个重载：

```cpp
template <typename T>
concept HasSize =
    requires(const T& value) {
        value.size();
    };

template <typename T>
concept HasBeginEnd =
    requires(const T& value) {
        value.begin();
        value.end();
    };

template <HasSize T>
void describe(const T& value);

template <HasBeginEnd T>
void describe(const T& value);
```

`std::vector<int>` 同时满足两个 concept。编译器无法知道哪个更具体，因为 `HasSize` 和 `HasBeginEnd` 没有定义包含关系。调用可能二义。

### 抽象不变量：约束满足、规范化与 subsumption 的完整机制

Concepts 重载选择看似简单（"更受约束的版本优先"），但理解其完整机制需要拆成三层。这三层不是可以跳过的"实现细节"——它们决定了你的重载是否会二义、你的 concept 层次是否能正确偏序、以及为什么某些看起来"更具体"的约束反而不被编译器识别。

**第一层：Constraint satisfaction（约束满足）。** 给定具体的模板实参，编译器替换约束表达式中的模板参数，然后判断约束是否为真。这一步像函数调用：把实参代入，看表达式结果。如果一个 `requires` 表达式中的某个子表达式在替换后不合法（编译失败），该原子约束被判为不满足。注意这和 SFINAE 不同——SFINAE 的替换失败发生在函数模板的签名中，而约束满足的检查发生在独立的约束评估上下文中，只影响约束结果，不影响函数模板的其他部分。

**第二层：Constraint normalization（约束规范化）。** 编译器需要比较两个约束"谁更具体"。为此，编译器把每个约束表达式递归地分解成原子约束（atomic constraints）的合取（conjunction，`&&`）和析取（disjunction，`||`）。原子约束是不可再分的最小约束单元——通常是一个 `requires` 表达式、一个 `concept` 引用或一个布尔表达式。

规范化的关键规则是：只有在**同一个 concept 定义的文本位置**出现的同一个表达式，才被视为"同一个原子约束"。这意味着：如果你在两个不同的 concept 定义中写了完全相同的 `requires(const T& x) { x.size(); }` 表达式，编译器不会认为它们是同一个原子约束——因为它们来自不同的源位置。这个规则乍看反直觉，但它避免了编译器需要做"两个任意表达式语义等价性判断"这个不可判定问题。

**第三层：Subsumption（包含偏序）。** 规范化之后，编译器检查约束 A 是否"包含"约束 B——也就是说，A 的满足是否在逻辑上蕴含 B 的满足。具体规则是：把 A 和 B 都表示为原子约束的合取/析取范式，然后检查 A 的范式中的每个析取分支是否至少包含 B 的范式中某个合取分支的所有原子约束。

直观理解：`SizedRange<T> = HasSize<T> && HasBeginEnd<T>` 比 `HasSize<T>` 更具体，因为 `SizedRange` 的原子约束集合**严格包含** `HasSize` 的原子约束集合——满足 `SizedRange` 必然满足 `HasSize`，反之不然。这就建立了偏序关系：当一个类型同时满足两者时，编译器选择更受约束（更具体）的版本。

**为什么两个互不蕴含的 concept 会导致二义性？** 如果 `HasSize` 和 `HasBeginEnd` 之间没有通过 `&&` 或 `||` 建立逻辑关系，编译器无法判断谁"包含"谁。当一个类型同时满足两者时，两个重载的约束强度不可比——编译器不知道该选哪个，只能报二义性错误。这不是编译器"不够聪明"，而是这两个约束在逻辑上确实没有包含关系。

**subsumption 的一个微妙限制**：subsumption 只在约束使用了**相同的 concept 定义**时才能跨越 concept 边界进行推理。如果 `SizedRange` 定义为 `HasSize<T> && HasBeginEnd<T>`，编译器能识别 `SizedRange` 比 `HasSize` 更具体，因为 `HasSize` 作为 concept 名出现在 `SizedRange` 的定义中。但如果你把 `HasSize` 的内容"内联"到 `SizedRange` 的定义里——即不引用 `HasSize` concept，而是直接写 `requires(const T& x) { x.size(); }`——编译器不会认为这和 `HasSize` 的约束是同一个原子约束，subsumption 就失效了。这就是为什么应该通过**组合命名 concept**来建立约束层次，而不是在每个 concept 里重写相同的 `requires` 表达式。

### 规则推导：显式建立 concept 层次

```cpp
template <typename T>
concept SizedRange = HasSize<T> && HasBeginEnd<T>;

template <SizedRange T>
void describe(const T& value) {
    std::cout << "sized range: " << value.size() << "\n";
}

template <HasSize T>
requires (!HasBeginEnd<T>)
void describe(const T& value) {
    std::cout << "sized non-range: " << value.size() << "\n";
}
```

现在 `std::vector<int>` 选择 `SizedRange` 版本，因为约束层次明确；只有有 `size()` 但没有 `begin()/end()` 的类型才走第二个重载。

### 反面失败：把业务分类写成一堆重叠重载

机器人接口里常见分类：

- 有 pose 的状态。
- 有 velocity 的状态。
- 有 covariance 的状态。
- 有 timestamp 的状态。

如果为每一类都写重载，很容易产生重叠。更稳的做法是建立角色层次，或用一个函数内的 `if constexpr` 明确优先级：

```cpp
template <typename State>
void logStateSummary(const State& state) {
    if constexpr (HasPose<State> && HasVelocity<State>) {
        logPoseAndVelocity(state);
    } else if constexpr (HasPose<State>) {
        logPoseOnly(state);
    } else {
        logGeneric(state);
    }
}
```

### 工程边界：Concept 名称不等于数学蕴含

编译器只比较约束表达式，不理解名字含义。`RigidBodyState<T>` 这个名字听起来比 `KinematicState<T>` 更具体，但如果定义上没有包含关系，重载选择不会按人类语义推断。

### 代码验证：二义性测试要保留

```cpp
static_assert(HasSize<std::vector<int>>);
static_assert(HasBeginEnd<std::vector<int>>);
static_assert(SizedRange<std::vector<int>>);
```

在教学项目里，可以保留一个被注释说明的负例：两个互不蕴含重载会二义。真正提交的代码应采用层次化 concept 或 `if constexpr` 优先级。

如果 C++ 没有 subsumption 机制会怎样？那么所有 concept 约束的重载都需要程序员手动用互斥条件（如 `requires (!HasBeginEnd<T>)`）消解二义性。每新增一个 concept 变体，就要修改所有已有重载的约束条件。Subsumption 让编译器自动判断"更具体的约束优先"，大幅减少了这种维护负担。

> ⚠️ **编程陷阱：两个互不蕴含的 concept 重载导致二义性**
> **错误做法**：分别写 `template<HasSize T> void f(T)` 和 `template<HasBeginEnd T> void f(T)`，期望编译器"选更合适的"。
> **现象**：对 `std::vector<int>` 调用 `f()` 时编译报错：二义性调用。
> **根本原因**：`HasSize` 和 `HasBeginEnd` 之间没有蕴含关系，编译器不知道哪个更具体。subsumption 只比较约束表达式的逻辑结构，不理解名字含义。
> **正确做法**：建立显式层次 `SizedRange = HasSize && HasBeginEnd`，或用 `if constexpr` 在一个函数内分派。

> 💡 **概念误区：认为 concept 名字能影响 subsumption 判断**
> **新手想法**："`RigidBodyState` 听起来比 `KinematicState` 更具体，所以编译器应该优先选 `RigidBodyState` 重载。"
> **实际上**：编译器只比较约束表达式的逻辑结构（原子约束的合取/析取），不理解英文单词。如果 `RigidBodyState` 的定义不包含 `KinematicState` 的约束作为子表达式，编译器不会认为它更具体。
> **正确理解**：subsumption 基于"约束 A 的原子集合是否包含约束 B 的原子集合"，是语法层面的判断，不是语义层面的推理。

### 练习

1. **[分析题]**：解释为什么 `SizedRange = HasSize && HasBeginEnd` 能在与 `HasSize` 的重载中胜出。用原子约束和蕴含关系推导。
2. **[代码题]**：设计三个状态 concept：`HasPose`、`HasVelocity`、`KinematicState = HasPose && HasVelocity`。编写三个重载函数 `logState()`，验证 subsumption 自动选择最具体版本。
3. **[跨章综合题]**：继承与多态深入 的虚函数重载解析发生在运行时（vtable 查找），本节的 concept subsumption 发生在编译时。对比两种"选择更具体实现"机制的适用场景和性能差异。

---

## 16.4 SFINAE 到 Concepts：同一需求的两种表达 ⭐⭐⭐⭐

### 工程问题：Concepts 到底解决了 SFINAE 的什么问题？

在深入代码对比之前，有必要从更高层次理解 Concepts 相对于 SFINAE 的改进。这种改进不只是"语法更短"，而是从三个根本维度提升了模板约束的工程质量。

**维度一：错误信息的可诊断性。** SFINAE 的错误信息之所以难读，不是因为编译器实现不好，而是因为 SFINAE 的失败发生在模板替换的深处。编译器报告的是"在替换 `enable_if_t<is_cloud_traits<Traits, Cloud>::value, int>` 时失败"，而不是"Cloud 类型缺少 `size()` 方法"。错误穿过了 `enable_if` → `is_cloud_traits` → `void_t` → `decltype` 多层包装，调用者看到的信息与真实原因之间隔了三四层间接。Concepts 把失败拉到约束层面——编译器直接报告"约束 `CloudTraits<cloud_traits<BadCloud>, BadCloud>` 不满足"，并能指出哪个具体的 `requires` 子表达式失败。

**维度二：可组合性。** SFINAE 约束通过 `enable_if` 的模板参数默认值或返回类型实现。如果要组合多个约束，需要嵌套 `enable_if` 或用 `std::conjunction`，表达式很快变得不可读。更麻烦的是，SFINAE 约束之间没有 subsumption 机制——两个使用不同 `enable_if` 条件的重载，编译器不知道谁更具体，需要程序员用互斥条件手动消解。Concepts 天然支持 `&&` 和 `||` 组合，subsumption 自动处理偏序关系。

**维度三：表达力和意图清晰度。** SFINAE 的约束写在模板参数列表或返回类型中——这些位置在语义上是"模板如何实例化"的技术细节，不是"这个函数需要什么"的接口声明。读 SFINAE 代码需要先"反编译"技术细节才能理解意图。Concepts 把约束写在函数签名的显著位置——`template <std::floating_point T>` 一眼就能看出"只接受浮点类型"。约束成为接口的一部分，而非实现的副产品。

理解了这三个维度，就能判断迁移的优先级：错误信息最差的 SFINAE 表达式应优先迁移；组合约束最复杂的应优先迁移；纯粹只检查一个类型特征的简单 `enable_if` 迁移优先级较低。

C++20 之前，模板约束常通过 SFINAE 表达：

```cpp
template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
T huberWeight(T residual, T delta) {
    const T abs_r = std::abs(residual);
    return abs_r <= delta ? T{1} : delta / abs_r;
}
```

这能工作，但接口读起来像“模板技巧”，业务需求反而不明显。

### Concepts 写法

```cpp
template <std::floating_point T>
T huberWeight(T residual, T delta) {
    const T abs_r = std::abs(residual);
    return abs_r <= delta ? T{1} : delta / abs_r;
}
```

同一需求变成“只接受浮点”。函数签名直接表达算法约束。

### 复杂迁移：检测点云 traits

C++17 检测惯用法：

```cpp
template <typename Traits, typename Cloud, typename = void>
struct is_cloud_traits : std::false_type {};

template <typename Traits, typename Cloud>
struct is_cloud_traits<Traits, Cloud, std::void_t<
    decltype(Traits::size(std::declval<const Cloud&>())),
    decltype(Traits::point(std::declval<const Cloud&>(), std::size_t{}))
>> : std::true_type {};

template <typename Cloud,
          std::enable_if_t<
              is_cloud_traits<cloud_traits<Cloud>, Cloud>::value, int> = 0>
Eigen::Vector3d centroid(const Cloud& cloud);
```

C++20 Concepts：

```cpp
template <typename Traits, typename Cloud>
concept CloudTraits =
    requires(const Cloud& cloud, std::size_t i) {
        { Traits::size(cloud) } -> std::convertible_to<std::size_t>;
        { Traits::point(cloud, i) } -> std::convertible_to<Eigen::Vector3d>;
    };

template <typename Cloud>
requires CloudTraits<cloud_traits<Cloud>, Cloud>
Eigen::Vector3d centroid(const Cloud& cloud);
```

迁移后的接口更短，错误也更靠近约束。

### 错误诊断差异

SFINAE 失败常表现为：

```text
no matching function for call to centroid(BadCloud&)
candidate template ignored: substitution failure ...
```

Concepts 失败通常会指出：

```text
constraints not satisfied
required expression 'Traits::size(cloud)' is invalid
```

不同编译器措辞不同，但方向一致：Concepts 把失败位置从函数体或替换细节拉到接口约束。

### 工程边界：不是所有 SFINAE 都要迁移

| 旧写法 | 是否适合迁移 | 判断 |
|--------|--------------|------|
| 简单类型约束 | 适合 | 标准 concept 可直接表达 |
| 表达式检测 | 适合 | `requires` 更可读 |
| traits 偏特化 | 保留 traits，外层加 concept | traits 仍负责适配 |
| 复杂偏特化选择 | 视情况 | 偏特化仍是合适工具 |
| C++17 公共库 | 不一定 | 工具链约束优先 |

### 代码验证：迁移不应改变可接受类型集合

迁移时要测试：

```cpp
static_assert(CloudTraits<cloud_traits<VecCloud>, VecCloud>);
static_assert(!CloudTraits<cloud_traits<BadCloud>, BadCloud>);
```

如果 Concepts 版本拒绝了旧版本可用的 `Eigen::Map` 或代理对象，要判断是旧接口过宽，还是新 concept 过窄。迁移的目标是表达更清楚，不是无意改变算法边界。

从 SFINAE 到 Concepts 的迁移过程，类似于把一栋老建筑的隐藏管线改为明线。SFINAE 就像藏在墙体里的水管——功能正常，但出了问题很难定位。Concepts 把管线外露到墙面上——接口一目了然，漏水点也容易找。迁移时最大的风险不是语法转换，而是在"明线化"过程中无意收紧或放宽了管径（接口约束）。

> ⚠️ **编程陷阱：迁移到 Concepts 后无意改变了可接受类型集合**
> **错误做法**：把 SFINAE 的 `std::void_t<decltype(...)>` 检测直接翻译成 `requires` 表达式，但添加了额外的返回类型约束。
> **现象**：旧代码能接受 `Eigen::Map` 或代理返回类型，新代码拒绝了它们。迁移后某些调用点报约束失败。
> **根本原因**：SFINAE 版本可能只检查"表达式能编译"，Concepts 版本可能同时加了 `-> std::same_as<...>` 的返回类型约束，比原来更严。
> **正确做法**：迁移前后对同一组正例和负例做 `static_assert`，确认可接受类型集合一致。

### 练习

1. **[分析题]**：对比 SFINAE 和 Concepts 两种写法的错误诊断输出。分别把一个错误类型传给两个版本的 `centroid()` 函数，对比 GCC 和 Clang 的报错信息。
2. **[代码题]**：把一个现有 SFINAE 约束迁移为 Concepts。编写正例和负例 `static_assert`，确认迁移前后可接受的类型集合完全一致。
3. **[跨章综合题]**：模板特化SFINAE与类型萃取 的 `is_cloud_traits` 检测惯用法使用 `std::void_t`。本节将其迁移为 `CloudTraits` concept。总结迁移的机械步骤，并讨论在 C++17 公共库中是否应该迁移（考虑工具链约束）。

---

## 16.5 Concepts 的语义边界：坐标系、单位、扰动方向不能靠它证明 ⭐⭐⭐⭐⭐

### 工程问题：机器人类型的关键错误常不是“有没有函数”

李群类型 concept 可以这样写：

```cpp
template <typename T>
concept LieGroupLike =
    requires(const T& a, const T& b, typename T::Tangent xi) {
        typename T::Tangent;
        { T::exp(xi) } -> std::same_as<T>;
        { a.log() } -> std::same_as<typename T::Tangent>;
        { a.inverse() } -> std::same_as<T>;
        { a * b } -> std::same_as<T>;
    };
```

它能检查 `exp`、`log`、`inverse`、乘法是否存在，返回类型是否符合接口。但它不能证明这些函数满足群公理，也不能证明扰动约定和优化器一致。

### 反面失败：满足 concept 但数学语义错

```cpp
struct BadSO3 {
    using Tangent = Eigen::Vector3d;

    static BadSO3 exp(const Tangent&) { return {}; }
    Tangent log() const { return Eigen::Vector3d::Zero(); }
    BadSO3 inverse() const { return {}; }
    BadSO3 operator*(const BadSO3&) const { return {}; }
};

static_assert(LieGroupLike<BadSO3>);
```

`BadSO3` 通过了语法约束，但可能完全不表示旋转。Concepts 不执行性质测试，不检查数值误差，不理解 SO(3) 的数学结构。

### 抽象不变量：三层契约缺一不可

成熟机器人接口通常需要三层：

```text
Concept：表达语法和类型要求
static_assert：表达编译期维度、标量类型、固定大小
运行时测试：表达坐标系、单位、正交性、扰动方向、数值稳定性
```

例如旋转类型可以这样补充运行时 invariant：

```cpp
template <typename R>
requires requires(const R& rotation) {
    { rotation.matrix() } -> std::convertible_to<Eigen::Matrix3d>;
}
void checkRotationInvariant(const R& rotation) {
    const Eigen::Matrix3d M = rotation.matrix();
    const double orth_error =
        (M.transpose() * M - Eigen::Matrix3d::Identity()).norm();
    const double det_error = std::abs(M.determinant() - 1.0);

    if (orth_error > 1e-9 || det_error > 1e-9) {
        throw std::runtime_error("invalid rotation matrix");
    }
}
```

### 规则推导：坐标系和单位属于接口语义

```cpp
template <typename Pose>
concept PoseLike =
    requires(const Pose& pose) {
        { pose.translation() } -> std::convertible_to<Eigen::Vector3d>;
        { pose.rotation() };
    };
```

`PoseLike` 不知道 `translation()` 是米还是毫米，不知道 pose 表达 `T_world_body` 还是 `T_body_world`，也不知道扰动是左乘还是右乘。这些约定应通过类型命名、函数名、文档和测试写清：

```text
Pose 约定：T_world_body，把 body 坐标转换到 world 坐标。
平移单位：meter。
扰动方向：左扰动，exp(delta) * T。
```

Concepts 让模板入口清晰，但不能替代这些约定。

### 工程边界：数值 concept 要避免伪装成数学证明

`std::floating_point<T>` 表示浮点类型，不表示精度足够；`std::integral<T>` 表示整数类型，不表示它适合作为索引；`std::predicate<F, Args...>` 表示可用作布尔测试，不表示谓词具有排序所需的严格弱序。

### 代码验证：正负语义测试

对 `LieGroupLike` 类型，至少测试：

```text
exp(0) 近似单位元
T * T.inverse() 近似单位元
exp(log(T)) 近似 T
boxplus 零扰动不改变状态
boxplus 后 boxminus 能恢复小扰动
左扰动和右扰动接口有明确测试
```

这些不是 Concepts 的失败，而是 Concepts 的边界。

> **本质洞察**：Concepts 检查的是语法契约（"这个表达式能编译吗？"），不是数学契约（"这个操作满足群公理吗？"）。一个通过 `LieGroupLike` concept 的类型可能根本不表示任何数学对象。这不是 Concepts 的缺陷，而是它的设计边界——编译器能验证的是有限的，数学不变量本质上需要运行时测试或形式化证明来保证。

如果没有三层契约的区分会怎样？如果程序员只写了 `LieGroupLike` concept 就认为类型"已经安全"，那么一个 `exp()` 返回零矩阵的假实现也能通过约束。当这个假实现被用于 SLAM 后端优化时，迭代立刻发散，但错误信息会指向优化器而非类型实现——因为 concept 已经"放行"了。三层契约的意义在于：concept 阻止明显错误的类型，`static_assert` 验证编译期维度，运行时测试验证数学性质。每层挡住一类错误，缺一层就多一类漏网之鱼。

> ⚠️ **编程陷阱：concept 名字暗示了它不能证明的数学性质**
> **错误做法**：命名为 `RotationMatrix` concept，但只检查 `.matrix()` 返回 `Matrix3d`。
> **现象**：一个返回非正交矩阵的类型也通过了约束，后续计算中出现数值发散。
> **根本原因**：concept 只检查语法（"有 `.matrix()` 方法且返回 `Matrix3d`"），不验证正交性（$R^TR = I$）或行列式（$\det(R) = 1$）。
> **正确做法**：concept 命名为 `RotationLike`（暗示"类似旋转"而非"是旋转"），并在文档中明确标注 concept 不保证的性质。用运行时函数 `checkRotationInvariant()` 验证数学约束。

> 💡 **概念误区：认为坐标系约定可以通过 Concepts 表达**
> **新手想法**："`PoseLike` concept 能保证 `translation()` 返回世界坐标系下的位移。"
> **实际上**：`PoseLike` 只知道 `pose.translation()` 能返回 `Vector3d`，完全不知道它是 `T_world_body` 还是 `T_body_world`，单位是米还是毫米，参考系是 ENU 还是 NED。坐标系和单位属于接口语义，只能通过类型命名、文档和测试表达。
> **正确理解**：Concepts 是"类型系统的海关"，它检查"证件格式"（接口合法性），不检查"证件内容"（数据语义）。

### 机器人库中 Concepts 语义边界的实际案例 ⭐⭐⭐⭐

在真实的机器人 C++ 库中，Concepts 语义边界的问题不是理论上的担忧，而是反复出现的实际 bug 来源。

**Eigen 表达式模板与 Concepts。** Eigen 的矩阵表达式（如 `A + B`）返回的不是 `Matrix` 而是 `CwiseBinaryOp` 等中间类型。这些类型满足所有"看起来像矩阵"的 concept 要求（有 `rows()`、`cols()`、支持 `operator()(i,j)`），但它们的行为和 `Matrix` 有关键区别——表达式模板延迟求值，多次访问同一个表达式可能重复计算。一个约束为 `MatrixLike` 的函数如果内部多次读取同一个表达式参数，性能可能严重退化，而 concept 无法检测到这一点。正确的做法是在函数入口处 `.eval()` 强制求值，或在文档中明确说明"传入表达式模板会导致重复计算"。

**Ceres 自动微分与标量 Concepts。** Ceres 的 `Jet<T, N>` 类型满足所有"看起来像标量"的 concept 要求（支持算术运算、比较运算、转换为 `bool`）。但 `Jet` 的比较运算只比较值部分，不比较导数部分——这意味着 `if (jet_a < jet_b)` 的分支选择不考虑导数信息。在残差函数中如果用 `std::min(a, b)` 这类涉及分支的操作，自动微分可能产生不连续的导数（因为分支在值域边界处导数不连续）。Concepts 检查不到这个问题——它只能检查 `operator<` 是否存在，不能检查比较操作在自动微分语境中的数学含义。

> **本质洞察**：Concepts 的语义边界不是设计缺陷，而是类型系统的根本局限——类型系统只能表达"结构层面"的信息，不能表达"物理层面"的信息。坐标系（世界系 vs 传感器系）、量纲（米 vs 毫米）、数学性质（正定性、正交性）都属于物理层面。C++ 的类型系统没有"坐标系"这个类型维度，因此 Concepts 无法区分"世界系的点"和"相机系的点"——它们在类型系统中是同一个 `Eigen::Vector3d`。

### 练习

1. **[分析题]**：列出 `LieGroupLike` concept 能检查和不能检查的性质各 3 条。对不能检查的性质，设计运行时测试方案。
2. **[代码题]**：为一个 `SO3` 类型编写运行时不变量检查函数 `checkSO3Invariant()`，验证正交性和行列式。结合 concept 和运行时检查，展示三层契约的协作方式。
3. **[跨章综合题]**：类型转换 讨论了 `dynamic_cast` 的运行时类型检查。Concepts 是编译时类型检查。对比两者："编译时就能发现的错误"和"必须等到运行时才能发现的错误"各有哪些？

---

## 16.6 `constexpr`、`consteval`、`if constexpr` 与 Ranges：只服务泛型接口主线 ⭐⭐⭐

### 工程问题：固定维度和运行时配置经常混在一起

机器人代码里既有编译期固定量，也有运行时配置：

```text
SO(3) 切空间维度：编译期 3
IMU bias 维度：编译期 6
体素大小：运行时配置
最大迭代次数：运行时配置
鲁棒核阈值：运行时配置
```

`constexpr` 和 `consteval` 只适合前一类。

### `constexpr` 与 `consteval` 的设计理念：编译期计算的层次

C++ 的编译期计算能力经历了四个阶段的演进，理解这条演进脉络有助于判断在什么场景下使用哪种工具。

**第一阶段（C++98/03）：模板元编程。** 编译期计算只能通过模板特化和递归实例化实现。计算阶乘需要写递归模板结构体——代码完全不像"计算"，而像类型系统的滥用。错误信息极难读，调试几乎不可能。但这证明了 C++ 模板系统是图灵完备的，理论上可以做任何编译期计算。

**第二阶段（C++11）：`constexpr` 函数的引入。** C++11 引入 `constexpr` 关键字，允许用普通函数语法编写编译期可求值的代码。但 C++11 的 `constexpr` 函数限制严格——函数体只能包含一条 `return` 语句，不能有局部变量、循环或条件分支。这使得它主要用于简单的数学计算（如 `constexpr double pi = 3.14159;`）。

**第三阶段（C++14/17）：`constexpr` 的放松。** C++14 大幅放松了 `constexpr` 函数的限制——允许局部变量、循环、条件分支、多条语句。C++17 进一步允许 `if constexpr`（编译期条件分支）。到此，`constexpr` 函数已经可以表达大多数编译期逻辑，包括复杂的数组初始化、查找表构建和数学计算。但 `constexpr` 函数有一个特性：它既可以在编译期执行，也可以在运行时执行——编译器根据实参是否是常量表达式来决定。这种双重性有时是优势（代码复用），有时是困扰（程序员不确定某个调用是否真的在编译期执行了）。

**第四阶段（C++20）：`consteval` 的引入。** `consteval` 解决了 `constexpr` 的双重性问题。`consteval` 函数**必须**在编译期执行——如果参数不是常量表达式，编译器直接报错而非降级到运行时。这对安全关键代码很有价值：如果某个查找表必须在编译期初始化（运行时初始化可能有时序问题），用 `consteval` 可以保证这一点。

**机器人系统中的分界线。** 在机器人代码中，编译期和运行时的分界线通常非常清晰：数学结构（流形维度、切空间维度、关节数量）在编译期确定，物理参数（传感器噪声、控制增益、体素大小）在运行时配置。混淆两者会导致要么过度约束（把应该可调的参数硬编码进模板），要么丧失优化机会（把可以固定的维度留到运行时确定）。

### `constexpr`：可编译期，也可运行期

```cpp
constexpr int stateDof(int pose_dof, int velocity_dof, int bias_dof) {
    return pose_dof + velocity_dof + bias_dof;
}

constexpr int kImuStateDof = stateDof(6, 3, 6);

int runtimeDof(int extra_dof) {
    return stateDof(15, extra_dof, 0);
}
```

`constexpr` 函数在实参是常量表达式时可编译期求值；实参是运行时变量时也可以运行期执行。

### `consteval`：必须编译期

```cpp
consteval int checkedFixedDof(int pose_dof, int vel_dof, int bias_dof) {
    return pose_dof + vel_dof + bias_dof;
}

constexpr int kFixedDof = checkedFixedDof(6, 3, 6);
```

`consteval` 不能读取 YAML、ROS 参数、传感器标定文件。不要为了“更现代”把运行时问题硬塞进编译期函数。

### `if constexpr + requires`：可选接口检测

```cpp
template <typename Policy>
void attachLoggerIfSupported(Policy& policy, Logger& logger) {
    if constexpr (requires(Policy& p) { p.setLogger(logger); }) {
        policy.setLogger(logger);
    }
}
```

这个模式适合非侵入式扩展：策略类型如果支持日志就注入，不支持就编译期丢弃分支。它比写 `has_set_logger_v` traits 更直接。

### Ranges：表达管线，但不强迫进入热路径

点云预处理可以写成管线：

```cpp
auto valid_points =
    points
    | std::views::filter([](const Point& p) {
          return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      })
    | std::views::transform([](const Point& p) {
          return Eigen::Vector3d{p.x, p.y, p.z};
      });
```

Ranges 很适合表达“过滤 -> 变换 -> 聚合”的接口意图。但许多机器人项目仍受 C++ 标准版本、编译器、标准库实现、编译时间和调试体验影响。在实时热路径中，是否使用 Ranges 要由 profile 和团队工具链决定。

### 工程边界

| 工具 | 适合 | 不适合 |
|------|------|--------|
| `constexpr` | 固定维度、小工具函数 | 强制读取运行时配置 |
| `consteval` | 必须编译期的表和检查 | 传感器数据、配置文件 |
| `if constexpr` | 类型能力分支 | 运行时数据分支 |
| Ranges | 中低频管线表达、接口层 | 未评估性能的高频内核 |

### 代码验证：编译期和运行时分开测

```cpp
static_assert(stateDof(6, 3, 6) == 15);

int configureIterations(const Config& config) {
    return std::max(1, config.max_iterations);
}
```

固定维度用 `static_assert`；运行时配置用普通测试和参数校验。分清边界比使用新语法更重要。

> ⚠️ **编程陷阱：用 `consteval` 函数处理运行时配置**
> **错误做法**：`consteval int maxIter(const Config& cfg) { return cfg.max_iterations; }` 尝试编译期读取配置文件。
> **现象**：编译报错——`consteval` 函数的参数必须是常量表达式，而 `Config` 来自运行时解析。
> **根本原因**：`consteval` 强制要求编译期求值。传感器参数、YAML 配置、ROS 参数都是运行时数据，不可能在编译期确定。
> **正确做法**：固定维度（如 SO(3) 切空间 3 维）用 `constexpr`/`consteval`；运行时参数（如体素大小、迭代次数）用普通函数和对象成员。

> 💡 **概念误区：认为 `if constexpr` 是运行时 `if` 的"更快版本"**
> **新手想法**："`if constexpr` 比普通 `if` 快，因为编译器优化了分支。"
> **实际上**：`if constexpr` 的核心价值不是速度，而是让未选分支不参与模板实例化。普通 `if (false)` 的分支在编译期就会被优化掉（零运行时开销），但仍必须通过类型检查。`if constexpr (false)` 的分支直接从实例化中丢弃，可以包含对当前类型不合法的代码。
> **正确理解**：`if constexpr` 是类型分派工具，不是性能优化工具。它让同一个模板函数能处理具有不同接口的类型。

### 练习

1. **[分析题]**：列出 `constexpr`、`consteval`、`if constexpr` 各自适合的一个机器人场景和一个不适合的场景。
2. **[代码题]**：编写一个 `attachLoggerIfSupported()` 函数，使用 `if constexpr + requires` 检测策略类型是否有 `setLogger()` 方法。对有和没有该方法的策略类型分别调用，验证编译结果。
3. **[跨章综合题]**：预处理器与宏 的条件编译 `#if defined(MINI_WITH_CUDA)` 在预处理阶段删除分支；`if constexpr` 在编译阶段删除分支。解释为什么 CUDA 选择通常必须用宏，而策略接口检测可以用 `if constexpr`。

---

## 16.7 Policy-based Design：从真实算法变化轴拆策略 ⭐⭐⭐⭐⭐

### 工程问题：配准算法变化按乘法增长

一个点云配准框架至少有这些变化轴：

| 变化轴 | 示例 |
|--------|------|
| 数据访问 | PCL、Open3D、`std::vector<Eigen::Vector3d>`、自定义点 |
| 搜索策略 | 暴力搜索、KD-Tree、体素哈希、投影关联 |
| 距离/残差 | 点到点、点到面、GICP Mahalanobis、VGICP |
| 外点处理 | 距离阈值、trimmed、Huber、Cauchy |
| 归约方式 | 串行、OpenMP、TBB、CUDA |
| 优化策略 | Gauss-Newton、Levenberg-Marquardt、线搜索 |

如果把每种组合都写成继承类，会出现组合爆炸：

```text
PointToPlaneKdTreeTbbHuberGaussNewton
PointToPlaneVoxelHashTbbHuberGaussNewton
GicpKdTreeOpenMpCauchyLevenberg
...
```

### 抽象不变量：Policy-based Design 的完整理论

**Alexandrescu 的设计动机与历史背景。** Policy-based Design 的理论基础来自 Andrei Alexandrescu 2001 年的著作《Modern C++ Design》。Alexandrescu 面对的核心问题是：面向对象设计模式（GoF 1994）中的"策略模式"（Strategy Pattern）用虚函数实现运行时分派，但在高性能数值计算和库设计中，虚调用的开销和间接性不可接受。他提出用模板参数替代虚函数参数——策略不再是运行时通过基类指针传入的对象，而是编译期通过模板参数传入的类型。

**策略组合的指数爆炸问题。** 这是 Policy-based Design 解决的最核心问题。当一个算法框架有 N 个独立变化轴，每个轴有 M 种选择时：

- 继承体系需要 $M^N$ 个叶子类（每种组合一个具体类）
- Policy 设计只需要 $N \times M$ 个策略类（每个轴的每种选择一个类）

组合由模板参数完成，不需要为每种组合手写代码。当 N=5、M=4 时，继承体系需要 1024 个类，Policy 只需要 20 个策略类。这种从指数增长到线性增长的转变，是 Policy-based Design 的数学本质。

**与虚函数策略模式的本质差异。** 虚函数策略模式（GoF Strategy）和 Policy-based Design 表面上解决同一类问题——把算法的可变部分提取出来。但它们的工作层次完全不同：

| 维度 | 虚函数 Strategy | Policy-based Design |
|------|-----------------|---------------------|
| 分派时机 | 运行时（vtable 查找） | 编译时（模板实例化） |
| 性能 | 每次调用有间接分派开销 | 编译器能内联策略调用 |
| 灵活性 | 运行时可替换策略 | 编译后策略固定 |
| 类型安全 | 通过基类接口约束 | 通过 Concepts 约束 |
| 代码膨胀 | 无——只有一份代码 | 每种组合生成一份代码 |
| ABI 稳定性 | 好——虚函数表是稳定的 ABI 边界 | 差——模板实现暴露在头文件 |
| 适用场景 | 插件、配置驱动、运行时选择 | 热路径、数值内核、库设计 |

两者不是互斥的——成熟的框架通常在外层用虚函数提供运行时灵活性，在内层用 Policy 提供编译期性能。16.9 节的分层设计正是这种组合。

**Policy 的接口契约。** 每个 Policy 不需要继承任何基类。它只需要满足一组隐式或显式的接口要求。在 C++20 之前，这些要求是隐式的——只有在模板实例化失败时才知道缺了什么。C++20 的 Concepts 让这些要求变成显式声明。

Policy 是一个变化轴的编译期策略。下面是一个完整的配准框架 Policy 结构：

```cpp
template <typename SearchPolicy,
          typename DistancePolicy,
          typename RejectorPolicy,
          typename OptimizerPolicy>
class RegistrationKernel {
public:
    RegistrationKernel(SearchPolicy search,
                       DistancePolicy distance,
                       RejectorPolicy rejector,
                       OptimizerPolicy optimizer)
        : search_(std::move(search)),
          distance_(std::move(distance)),
          rejector_(std::move(rejector)),
          optimizer_(std::move(optimizer)) {}

    template <typename SourceCloud, typename TargetCloud>
    RegistrationResult run(const SourceCloud& source,
                           const TargetCloud& target) {
        auto correspondences = search_.find(source, target, distance_);
        auto filtered = rejector_.filter(correspondences);
        return optimizer_.solve(source, target, filtered, distance_);
    }

private:
    SearchPolicy search_;
    DistancePolicy distance_;
    RejectorPolicy rejector_;
    OptimizerPolicy optimizer_;
};
```

每个策略只负责一个变化轴。新增 KD-Tree 搜索不修改距离策略；新增 GICP 距离不修改外点剔除；新增优化器不修改数据访问。

这种设计的关键优势是变化的正交性。在继承体系中，新增一种搜索策略需要为每种距离策略和优化器都派生一个新类——变化沿乘法增长。在 Policy 设计中，新增一种搜索策略只需要一个新类，它自动可以和已有的所有距离策略和优化器组合——变化沿加法增长。这是开闭原则（Open/Closed Principle）在泛型编程中的最直接体现。

如果不用 Policy 而用继承树会怎样？假设有 4 种搜索 x 3 种距离 x 2 种优化 = 24 种组合，继承树需要 24 个叶子类，每个类名都像 `PointToPlaneKdTreeGaussNewton`。新增一种搜索策略就要再派生 6 个类。而 Policy 设计只需要 4+3+2 = 9 个策略类，组合由模板参数完成。当变化轴达到 5-6 个时，继承树完全不可维护。

### 用 Concepts 约束 Policy

```cpp
template <typename Policy, typename SourcePoint, typename TargetPoint>
concept DistancePolicyFor =
    requires(const Policy& policy,
             const SourcePoint& source,
             const TargetPoint& target) {
        { policy(source, target) } -> std::convertible_to<double>;
    };

template <typename Search, typename SourceCloud, typename TargetCloud, typename Distance>
concept SearchPolicyFor =
    requires(const Search& search,
             const SourceCloud& source,
             const TargetCloud& target,
             const Distance& distance) {
        search.find(source, target, distance);
    };

template <typename Optimizer, typename Problem>
concept OptimizerPolicyFor =
    requires(Optimizer& optimizer, Problem& problem) {
        { optimizer.solve(problem) } -> std::same_as<RegistrationResult>;
    };
```

Concepts 不是让策略更快，而是让策略的接口要求可见。

### Policy 接口设计的核心原则：最小接口与正交分解

Policy-based Design 的成功不仅取决于"把变化拆成策略"，还取决于"策略接口如何设计"。两个常见的设计失误会破坏 Policy 的组合优势：

**失误一：策略接口过大。** 如果 `DistancePolicy` 不仅要求 `operator()(source, target) -> double`，还要求 `name() -> string`、`reset()`、`configure(params)`，那么一个只需要计算距离的简单函数对象就不能作为策略使用。接口过大等于人为缩小了可组合的策略集合。正确的做法是：每个策略接口只包含算法内核真正调用的最小操作集合。可选的辅助操作（如日志名称、重置、配置）可以用 `if constexpr + requires` 进行非侵入式探测——策略有就用，没有就跳过。

**失误二：策略之间不正交。** 如果 `SearchPolicy` 的接口依赖于 `DistancePolicy` 的具体类型（如要求 `search.find(cloud, PointToPointDistance)` 中的 `PointToPointDistance` 是具体类型而非模板参数），那么更换距离策略时必须同时更换搜索策略——两个变化轴不再独立。正交性要求每个策略只依赖其他策略的抽象接口（通过 Concepts 描述），而非具体类型。

**Policy 组合的编译时代价。** Policy-based Design 的一个工程现实是：每种策略组合都会实例化一份独立的模板代码。如果有 4 种搜索 x 3 种距离 x 2 种优化器 = 24 种组合，编译器可能需要实例化 24 份 `RegistrationKernel`。这增加了编译时间和二进制体积。工程上的缓解手段包括：(1) 用显式实例化收敛常用组合到 `.cpp` 文件；(2) 把不影响性能的策略（如日志策略）用 type erasure 替代编译期策略；(3) 控制策略组合数量，不要为每个微小差异都创建新策略。

### 反面失败：把运行时配置做成模板参数

```cpp
template <int MaxIterations, int VoxelSizeMillimeters>
class RegistrationKernel;
```

最大迭代次数、体素大小、鲁棒核阈值通常来自配置文件。把它们做成模板参数意味着每个配置都要重新编译，并产生新类型。更好的做法是把算法种类做成 Policy，把数值参数放进对象状态：

```cpp
struct GaussNewtonOptimizer {
    int max_iterations = 20;
    double min_delta = 1e-6;

    RegistrationResult solve(Problem& problem) const;
};
```

### 工程边界：Policy 适合热路径和封闭组合

Policy-based Design 的优势：

- 编译器能内联策略调用。
- 每个变化轴可单独测试。
- 类型组合表达清楚。
- 热路径没有虚调用。

代价：

- 每种组合可能生成一份代码。
- 模板实现常放头文件，改动触发大面积重编译。
- 错误信息可能仍很长。
- ABI 不稳定，不适合直接作为动态库边界。

### 代码验证：两个距离策略和两个搜索策略

```cpp
struct PointToPointDistance {
    double operator()(const Eigen::Vector3d& a,
                      const Eigen::Vector3d& b) const {
        return (a - b).squaredNorm();
    }
};

struct PointToPlaneDistance {
    double operator()(const Eigen::Vector3d& source,
                      const PlanePoint& target) const {
        return std::pow(target.normal.dot(source - target.point), 2);
    }
};

static_assert(DistancePolicyFor<
    PointToPointDistance, Eigen::Vector3d, Eigen::Vector3d>);
static_assert(DistancePolicyFor<
    PointToPlaneDistance, Eigen::Vector3d, PlanePoint>);
```

策略测试不需要完整配准流程。先证明每个策略满足概念，再测试组合行为。

Policy-based Design 的思想可以类比乐高积木。每种策略就像一种形状的积木块——搜索策略是一种形状，距离策略是另一种形状，优化策略又是一种。框架定义了插槽的接口（concept），每种积木只要符合插槽形状就能插上。你可以自由组合不同颜色和形状的积木，而不需要为每种组合专门铸造一个整体模具。如果没有这种组合机制，每种算法变体都需要一个完整的类——就像为每种颜色组合都铸造一个完整的乐高套装，库存管理会失控。

如果把运行时配置参数也做成模板参数会怎样？每种迭代次数、每种体素大小都会生成一个新的模板实例。改一个参数就要重新编译，二进制中充斥着只有参数值不同的重复代码。更致命的是，这些参数通常来自 YAML 配置文件——编译期根本不知道值是多少。Policy 应该只封装"算法种类"这种编译期决策，数值参数属于对象运行时状态。

> ⚠️ **编程陷阱：把运行时数值参数做成模板参数**
> **错误做法**：`template <int MaxIterations, int VoxelSizeMillimeters> class Kernel;`
> **现象**：每种参数组合生成一份代码，编译时间暴增，配置文件的参数值无法在运行时生效。
> **根本原因**：模板参数是编译期常量，不能接受运行时值。算法种类（ICP vs GICP）是编译期选择，参数值（迭代次数、阈值）是运行时选择。
> **正确做法**：算法种类做 Policy 模板参数，数值参数放进对象成员或构造函数参数。

> 🧠 **思维陷阱：认为 Policy 组合能覆盖所有可能的算法变体**
> **新手想法**："只要策略轴拆得够细，任何算法变体都能通过组合得到。"
> **实际上**：策略轴之间可能有隐含的依赖关系。例如 GICP 距离度量需要协方差矩阵，而点到点距离不需要——它们对点类型的要求不同。如果 `DistancePolicy` 的 concept 太宽，不兼容的组合可能在深层模板展开时才报错。
> **正确思维**：策略组合需要有效性约束。用 concept 表达"这种搜索策略和这种距离策略能组合"的条件，而不是假设所有组合都合法。

### Policy 在机器人库中的实际应用 ⭐⭐⭐

理解 Policy-based Design 的最好方式是看真实机器人库如何使用它。

**Ceres Solver 的代价函数 Policy。** Ceres 的 `Problem` 类接受任意的代价函数（cost function），但代价函数的雅可比计算可以是手动解析、自动微分（`AutoDiffCostFunction`）或数值微分（`NumericDiffCostFunction`）。这三种方式就是三种"求导 Policy"。`AutoDiffCostFunction` 的模板参数包括残差维度和参数块维度——这些编译期常量让编译器能生成固定大小的 `Jet` 类型和矩阵，避免了动态分配。用户只需要编写一个模板化的 `operator()`，微分策略在编译期确定。

**small_gicp 的搜索和距离策略。** small_gicp 把配准算法分解为数据访问（traits）、最近邻搜索（KD-Tree / 体素哈希）和距离度量（点到点 / 点到面 / GICP）三个正交轴。每个轴是一个 Policy，通过模板参数组合。这种设计让用户可以组合出 `RegistrationKernel<KdTreeSearch, GicpDistance>` 或 `RegistrationKernel<VoxelSearch, PointToPlaneDistance>` 等变体，而不需要为每种组合编写独立的类。

**PCL 的点类型系统。** PCL 的 `PointCloud<PointT>` 是 Policy 思想的早期体现——点类型 `PointT` 决定了每个点存储哪些字段（`PointXYZ` vs `PointXYZI` vs `PointXYZINormal`）。但 PCL 的设计早于 Concepts，因此点类型约束依赖宏注册和运行时检查，而非编译期约束。现代设计可以用 Concepts 约束 `PointT` 必须满足的接口。

> **本质洞察**：Policy-based Design 的核心价值不是"减少代码重复"——如果只有一种组合，直接写一个类更简单。Policy 的价值在于它把"算法变化的正交维度"显式化了。当你把配准算法分解为 `SearchPolicy`、`DistancePolicy`、`OptimizerPolicy` 时，你做的不只是代码组织——你在声明"这三个维度可以独立变化"。这个声明既帮助当前开发者理解系统结构，也帮助未来的开发者知道"新增一种距离度量不需要修改搜索代码"。

### 练习

1. **[分析题]**：对比继承体系（`PointToPlaneKdTreeGaussNewton`）和 Policy 组合（`Kernel<KdTree, PointToPlane, GaussNewton>`）。在可扩展性、编译时间和调试体验上各有什么优劣？
2. **[代码题]**：为 Mini Registration 实现一个 `HuberRejector` 策略，其 `huber_delta` 是构造函数参数（运行时值），而不是模板参数。编写 concept 验证和最小使用示例。
3. **[跨章综合题]**：变参模板折叠表达式与CRTP 的 CRTP 也能实现编译期多态。对比 CRTP 和 Policy-based Design 的适用场景：CRTP 适合什么结构？Policy 适合什么结构？能否把两者结合使用？

---

## 16.8 Traits + Policy：数据访问和算法策略分层 ⭐⭐⭐⭐⭐

### 工程问题：数据类型变化和算法变化不是一回事

点云可能来自 PCL、Open3D、Eigen vector、自定义结构体；配准算法可能选择 GICP、NDT、点到面。把这两类变化混在一个继承体系里，会导致每支持一种点云就要复制算法类。

更清晰的分层是把数据访问和算法策略放在两个独立的维度上。数据访问变化是"我的点云来自 PCL 还是 Open3D"——这只影响怎么读取一个点的坐标。算法变化是"我用 ICP 还是 GICP"——这影响匹配和优化的数学逻辑。如果把两个维度混在一个继承体系里，每支持一种新点云格式就要复制所有算法类，或者每种新算法都要为所有点云格式写适配代码。分层后，traits 负责"数据怎么看"，Policy 负责"算法怎么做"，两者独立变化、独立测试、独立扩展。

```text
traits：这个数据类型怎么看
  - size()
  - point(i)
  - normal(i)
  - covariance(i)

policy：算法怎么做
  - search
  - distance/residual
  - rejector
  - optimizer/reduction
```

### traits 示例

```cpp
template <typename Cloud>
struct cloud_traits;

template <>
struct cloud_traits<std::vector<Eigen::Vector3d>> {
    static std::size_t size(const std::vector<Eigen::Vector3d>& cloud) {
        return cloud.size();
    }

    static const Eigen::Vector3d& point(
        const std::vector<Eigen::Vector3d>& cloud,
        std::size_t i) {
        return cloud[i];
    }
};
```

用 concept 检查：

```cpp
template <typename Traits, typename Cloud>
concept CloudTraits =
    requires(const Cloud& cloud, std::size_t i) {
        { Traits::size(cloud) } -> std::convertible_to<std::size_t>;
        { Traits::point(cloud, i) } -> std::convertible_to<Eigen::Vector3d>;
    };
```

### 反面失败：traits 返回引用指向临时对象

```cpp
static const Eigen::Vector3d& point(const PclCloud& cloud, std::size_t i) {
    const auto& p = cloud.points[i];
    return Eigen::Vector3d{p.x, p.y, p.z};
}
```

这个函数返回悬空引用。若需要构造 `Eigen::Vector3d`，就返回值：

```cpp
static Eigen::Vector3d point(const PclCloud& cloud, std::size_t i) {
    const auto& p = cloud.points[i];
    return Eigen::Vector3d{p.x, p.y, p.z};
}
```

如果性能要求避免拷贝，可以设计明确的 `Eigen::Map` 或 `Eigen::Ref` 接口，但要保证生命周期。

### 抽象不变量：traits 适配外部类型，Policy 组合算法内核

```cpp
template <typename Distance, typename Search>
class Matcher {
public:
    Matcher(Distance distance, Search search)
        : distance_(std::move(distance)),
          search_(std::move(search)) {}

    template <typename SourceCloud, typename TargetCloud>
    requires CloudTraits<cloud_traits<SourceCloud>, SourceCloud> &&
             CloudTraits<cloud_traits<TargetCloud>, TargetCloud>
    std::vector<int> match(const SourceCloud& source,
                           const TargetCloud& target) const {
        return search_(source, target, distance_);
    }

private:
    Distance distance_;
    Search search_;
};
```

`Matcher` 不关心点云容器细节；`cloud_traits` 不关心使用 KD-Tree 还是体素哈希。边界清晰后，每个变化轴都能独立扩展。

### 工程边界：点访问热路径尽量保持静态

虚函数点云视图很清晰：

```cpp
class CloudView {
public:
    virtual ~CloudView() = default;
    virtual std::size_t size() const = 0;
    virtual Eigen::Vector3d point(std::size_t i) const = 0;
};
```

它适合系统边界、调试工具或低频路径。百万点内循环里，每点虚调用可能成为优化边界。traits 能让点访问内联，更适合热路径。选择不是“虚函数永远慢”，而是看调用频率和部署边界。

### 代码验证：正负例同时写

```cpp
using EigenCloud = std::vector<Eigen::Vector3d>;

struct NoTraitsCloud {};

static_assert(CloudTraits<cloud_traits<EigenCloud>, EigenCloud>);
static_assert(!requires(const NoTraitsCloud& cloud, std::size_t i) {
    cloud_traits<NoTraitsCloud>::size(cloud);
    cloud_traits<NoTraitsCloud>::point(cloud, i);
});
```

如果希望错误更集中，也可以给主算法入口加自定义静态断言或 concept 约束，让错误信息指向 `CloudTraits` 名称。

> ⚠️ **编程陷阱：traits 返回引用指向临时对象导致悬空引用**
> **错误做法**：`static const Eigen::Vector3d& point(const PclCloud& cloud, std::size_t i)` 内部构造临时 `Eigen::Vector3d` 并返回其引用。
> **现象**：返回值引用的临时对象在函数返回时已经销毁。调用者读到垃圾数据或段错误。
> **根本原因**：C++ 临时对象的生命周期在创建它的完整表达式结束时终止。函数返回局部临时对象的引用是经典的悬空引用。
> **正确做法**：如果需要构造新对象，返回值而非引用。如果底层数据已经是 `Vector3d`，可以返回 `const` 引用。

> 💡 **概念误区：认为 traits 和 Policy 是一回事**
> **新手想法**："traits 和 Policy 都是模板参数，功能差不多。"
> **实际上**：traits 负责"这个数据类型怎么看"——把外部类型适配成算法需要的接口。Policy 负责"算法怎么做"——选择搜索策略、距离度量、优化方法。traits 是非侵入式类型适配器，Policy 是算法变化轴的编译期策略。两者的设计目标、变化方向和使用模式都不同。
> **正确理解**：traits 适配数据，Policy 组合算法。traits 一般有偏特化，Policy 一般有 concept 约束。

### 练习

1. **[分析题]**：为什么 `cloud_traits` 使用偏特化（`template<> struct cloud_traits<std::vector<Eigen::Vector3d>>`）而不是继承或虚函数？讨论非侵入式适配的优势。
2. **[代码题]**：为 PCL 的 `pcl::PointCloud<pcl::PointXYZ>` 编写一个 `cloud_traits` 偏特化。用 `CloudTraits` concept 验证它满足约束。
3. **[跨章综合题]**：预处理器与宏 的 PCL 点类型注册宏生成字段的元信息（名字、类型、偏移）；本节的 traits 提供数据访问接口。两者在"让算法能使用外部数据类型"这个目标上如何分工？

---

## 16.9 Type erasure、虚接口与编译期内核：分层组合 ⭐⭐⭐⭐⭐

### Type Erasure 的理论基础：在运行时多态和编译期多态之间架桥

Type erasure（类型擦除）是 C++ 中一种介于虚函数和模板之间的多态技术。它解决了一个看似矛盾的需求：**在编译期不知道具体类型的情况下，调用类型特定的操作**——但不要求被调用的类型继承任何特定的基类。

**为什么需要 type erasure？** 虚函数要求所有多态类型继承同一个基类——这意味着类型的设计者必须"预见"到自己的类型会被多态使用，并提前添加继承关系。但在实际项目中，很多类型的设计者和使用者是不同的人（甚至是不同的库）。PCL 的 `pcl::PointCloud<pcl::PointXYZ>` 没有继承 Open3D 的点云基类，Eigen 的 `Matrix3d` 没有继承 Sophus 的旋转基类。如果一个配准框架想同时支持 PCL 和 Open3D 的点云，用虚函数就需要两个库都继承同一个接口——这在实际中不可能。

Type erasure 的解决方案是：**在包装层添加虚函数接口，但不要求被包装的类型继承这个接口**。这和 Go 语言的 interface 概念很接近——任何满足接口要求的类型都可以被包装，无需显式继承。`std::function<R(Args...)>` 就是标准库中最常用的 type erasure：它可以包装普通函数、lambda、函数对象和成员函数指针，这些被包装的类型之间没有任何继承关系。

**Type erasure 的实现原理。** Type erasure 在内部使用虚函数，但把虚函数"藏"在包装器内部：

1. 定义一个内部抽象基类 `Concept`，声明需要的虚函数接口。
2. 对每种具体类型 `T`，定义一个内部派生类 `Model<T>`，在虚函数中转发到 `T` 的具体操作。
3. 包装器 `AnyCallable` 持有一个 `unique_ptr<Concept>`，外部调用通过 `Concept` 的虚函数分派到正确的 `Model<T>`。

这种实现意味着 type erasure 有虚函数调用的运行时开销，但被包装的类型不需要任何修改。它是"非侵入式运行时多态"——结合了虚函数的运行时灵活性和模板的非侵入性。

**在机器人框架中的定位。** Type erasure 通常用在系统架构的边界层——工厂函数返回类型擦除的对象（如 `std::function<double(Vec3, Vec3)>`），运行时代码通过它调用具体的距离策略。在这个边界之外（配置层），类型是运行时确定的；在这个边界之内（数值内核），模板内核用具体类型提供最优性能。Type erasure 是连接这两层的桥梁。

### 工程问题：配置文件要运行时选择，热路径要静态优化

实际机器人系统往往这样运行：

```text
YAML 选择 method: gicp
launch 选择 parallel: tbb
运行时加载插件
内层每帧处理几十万点
```

外层需要运行时灵活性，内层需要编译期优化。纯模板框架不能从 YAML 直接选择模板参数；纯虚函数内核又可能把每点调用都变成间接分派。

### 推荐分层

```text
配置文件 / launch 参数
  ↓
运行时工厂：选择 ICP、GICP、NDT
  ↓
稳定虚接口或 type erasure：RegistrationBase
  ↓
具体实现类：GicpRegistration、NdtRegistration
  ↓
编译期内核：Policy + traits + Concepts
  ↓
数值热路径：Eigen、搜索、归约、优化
```

### 外层虚接口

```cpp
class RegistrationBase {
public:
    virtual ~RegistrationBase() = default;

    virtual RegistrationResult align(const CloudView& source,
                                     const CloudView& target) = 0;
};
```

工厂根据配置创建：

```cpp
std::unique_ptr<RegistrationBase> makeRegistration(
    const RegistrationConfig& config) {
    if (config.method == "gicp") {
        return std::make_unique<GicpRegistration>(config.gicp);
    }
    if (config.method == "point_to_plane") {
        return std::make_unique<PointToPlaneRegistration>(config.icp);
    }
    throw std::invalid_argument("unknown registration method");
}
```

### 内层 Policy 内核

```cpp
class GicpRegistration final : public RegistrationBase {
public:
    explicit GicpRegistration(GicpConfig config)
        : config_(config) {}

    RegistrationResult align(const CloudView& source,
                             const CloudView& target) override {
        using Kernel = RegistrationKernel<
            KdTreeSearch,
            MahalanobisDistance,
            HuberRejector,
            GaussNewtonOptimizer>;

        Kernel kernel{
            KdTreeSearch{},
            MahalanobisDistance{},
            HuberRejector{config_.huber_delta},
            GaussNewtonOptimizer{config_.max_iterations}};

        return kernel.run(source, target);
    }

private:
    GicpConfig config_;
};
```

这里外层有一个虚调用，内层循环仍由具体策略类型驱动。对大多数系统，这是清晰性和性能之间更稳的折中。

### `std::function` 作为局部 type erasure

低频回调或测试注入可以用 `std::function`：

```cpp
using LossFunction = std::function<double(double squared_error)>;

class RuntimeLoss {
public:
    explicit RuntimeLoss(LossFunction loss)
        : loss_(std::move(loss)) {}

    double operator()(double squared_error) const {
        return loss_(squared_error);
    }

private:
    LossFunction loss_;
};
```

`std::function` 有间接调用和可能分配成本，不应默认放进每点残差热路径。但它很适合配置层、调试层和测试替身。

### 工程边界：ABI 和构建成本

Policy-based Design 常把实现放在头文件，因为模板要在使用点实例化。代价包括：

| 成本 | 表现 |
|------|------|
| 编译时间 | 每个翻译单元重复实例化组合 |
| 二进制体积 | 不同策略组合生成多份代码 |
| ABI 稳定性 | 模板实现暴露在头文件，改动触发重编译 |
| 错误信息 | 深层模板错误仍可能很长 |
| 发布边界 | 二进制 SDK 不适合暴露过多模板细节 |

显式实例化可以控制成本：

```cpp
// registration_kernel.hpp
template <typename Search, typename Distance, typename Rejector, typename Optimizer>
class RegistrationKernel;

extern template class RegistrationKernel<
    KdTreeSearch,
    PointToPlaneDistance,
    HuberRejector,
    GaussNewtonOptimizer>;
```

```cpp
// registration_kernel.cpp
template class RegistrationKernel<
    KdTreeSearch,
    PointToPlaneDistance,
    HuberRejector,
    GaussNewtonOptimizer>;
```

这样常用组合只在一个翻译单元实例化。代价是支持组合集合更明确，新增组合要补实例化。

### 代码验证：观察构建成本

```bash
g++ -std=c++20 -ftime-trace -c registration_kernel.cpp
nm -C libmini_registration.a | grep RegistrationKernel
size mini_registration_demo
```

这些工具能回答：模板实例化是否耗时，符号是否膨胀，二进制体积是否被策略组合拉大。

分层设计的思想可以类比餐厅的前厅与后厨。前厅（虚接口/type erasure）对顾客展示统一菜单，顾客用菜名点餐；后厨（Policy + traits）内部用最高效的设备和流程做菜。顾客不需要知道后厨用了什么型号的烤箱，后厨也不需要因为换了烤箱就让顾客重新看菜单。虚接口是"菜单"，Policy 是"烹饪方法"，traits 是"食材处理方式"。分层让每一层的变化不影响其他层。

> ⚠️ **编程陷阱：`std::function` 放进每点残差热路径**
> **错误做法**：在百万点的内循环中用 `std::function<double(double)>` 作为损失函数。
> **现象**：性能比直接调用慢 10-50 倍。profiler 显示大量间接调用和可能的堆分配开销。
> **根本原因**：`std::function` 使用 type erasure，涉及间接调用（虚函数或函数指针）；如果捕获的 lambda 超过小缓冲区优化（SBO）大小，还会触发堆分配。
> **正确做法**：配置层和测试层可以用 `std::function`。热路径用 Policy 模板参数让编译器内联。

> 💡 **概念误区：认为显式实例化能解决所有模板编译成本**
> **新手想法**："把所有策略组合都显式实例化到 `.cpp` 里，编译时间就不是问题了。"
> **实际上**：显式实例化只减少了重复实例化次数，不减少每种组合的首次实例化成本。如果策略轴多、组合空间大（6 种搜索 x 5 种距离 x 4 种优化 = 120 种组合），显式实例化文件本身就很大。更重要的是，新增策略时必须手动更新实例化列表——忘记就会链接报错。
> **正确理解**：显式实例化适合"常用组合少且稳定"的情况。组合空间大时，应考虑减少策略轴、合并策略或在外层使用 type erasure。

### 练习

1. **[分析题]**：为什么 `GicpRegistration::align()` 的外层是虚函数，内层是 Policy 模板？如果把内层也改成虚函数会有什么性能后果？如果把外层也改成模板会有什么接口后果？
2. **[代码题]**：为 Mini Registration 添加显式实例化。在 `.hpp` 中 `extern template`，在 `.cpp` 中实例化。用 `nm -C` 比较有和没有显式实例化时的符号数量。
3. **[跨章综合题]**：继承与多态深入 讲了运行时多态（虚函数）；变参模板折叠表达式与CRTP 讲了编译时多态（CRTP）；本节讲了分层组合（外层虚+内层 Policy）。在一个完整的机器人框架中，这三者分别在哪层使用？画出一个三层架构图。

---

## 16.10 设计原则：小 concept、明确策略轴、边界处擦除类型 ⭐⭐⭐⭐

### 工程问题：工具太多时容易按语法而不是按变化选型

现代 C++ 给了 Concepts、traits、CRTP、Policy、虚函数、`std::function`、`std::variant`、宏。选型不应从语法喜好开始，而应从变化发生的位置开始。

### 决策流程

```text
1. 这个选择是否来自运行时配置、插件或用户输入？
   ├─ 是 → 虚接口、工厂、type erasure
   └─ 否 → 继续

2. 这段调用是否处在高频数值热路径？
   ├─ 是 → Policy、traits、CRTP、Concepts
   └─ 否 → 简单对象、函数、std::function 也可接受

3. 这个要求能否用表达式合法性描述？
   ├─ 是 → Concept
   └─ 否 → 文档契约、static_assert、运行时测试

4. 是否需要操作 token、字段名、条件编译或生成注册符号？
   ├─ 是 → 宏，但限制边界
   └─ 否 → 不使用宏
```

### 工具分层表

| 层次 | 推荐工具 | 示例 |
|------|----------|------|
| 系统插件边界 | 虚函数、工厂、type erasure | ROS2 controller、planner、registration 后端 |
| 算法对象 | 虚接口 + 内部静态内核 | GICP、NDT、ICP |
| 数值热路径 | Policy、traits、CRTP | 搜索、残差、归约、优化 |
| 模板入口约束 | Concepts | `CloudTraits`、`DistancePolicyFor` |
| 外部类型适配 | traits | PCL、Open3D、自定义点 |
| 固定维度 | `static_assert`、`constexpr` | 状态维度、块 offset |
| 预处理能力 | 宏 | PCL 字段注册、g2o 注册、条件编译 |

### 反面失败：一个 concept 承担过多角色

```cpp
template <typename T>
concept RobotState =
    requires(const T& state) {
        state.pose();
        state.velocity();
        state.bias();
        state.covariance();
        state.timestamp();
        state.frameId();
    };
```

如果某个函数只需要 pose，却要求完整 `RobotState`，复用性会变差。拆成小角色：

```cpp
template <typename T>
concept HasPose =
    requires(const T& state) {
        { state.pose() };
    };

template <typename T>
concept HasVelocity =
    requires(const T& state) {
        { state.velocity() } -> std::convertible_to<Eigen::Vector3d>;
    };

template <typename T>
concept KinematicState = HasPose<T> && HasVelocity<T>;
```

小 concept 更容易组合，也更容易定位错误。

### 工程边界：错误诊断也要设计

一个好泛型接口应在错误类型传入时给出可理解诊断。可以用负例编译目标或 `static_assert` 检查：

```cpp
struct MissingPoint {};

static_assert(!CloudTraits<cloud_traits<MissingPoint>, MissingPoint>);
```

如果编译器错误仍指向 Eigen 深处，说明约束写得太晚或太宽。把 concept 放到算法入口处，而不是等函数体内部失败。

### 代码验证：接口验收五问

```text
concept 是否足够小
返回类型约束是否过窄
错误类型是否在接口处失败
数学 invariant 是否有运行时测试
运行时配置是否留在运行时层
```

这五问比”是否用了 C++20”更能判断接口质量。

> ⚠️ **编程陷阱：一个 concept 承担过多角色**
> **错误做法**：定义 `RobotState` concept 要求同时有 `pose()`、`velocity()`、`bias()`、`covariance()`、`timestamp()`、`frameId()`。
> **现象**：某个只需要 `pose()` 的函数却要求完整的 `RobotState`，把只有位姿的轻量状态排除在外。
> **根本原因**：大 concept 把多个独立角色捆绑在一起。函数的约束应匹配其真实需求，不应要求它不使用的能力。
> **正确做法**：拆成 `HasPose`、`HasVelocity`、`HasCovariance` 等小角色。需要多个角色时用 `KinematicState = HasPose && HasVelocity` 组合。

> 🧠 **思维陷阱：按语法喜好而不是按变化位置选型**
> **新手想法**：”C++20 出了 Concepts，所有接口都应该用 Concepts 写。”
> **实际上**：选型应从”变化发生在哪里”出发。运行时配置变化→虚接口/工厂。编译期算法变化→Policy/traits。模板入口约束→Concepts。token 操作→宏。每种工具解决不同层次的变化，不存在”一种工具通吃”。
> **正确思维**：先问”这个变化是编译期的还是运行时的？是类型层的还是 token 层的？”然后选择对应层次的工具。

### 练习

1. **[分析题]**：对照”决策流程”的四个问题，为以下需求选择工具：(a) 根据 YAML 选择 ICP 或 GICP；(b) 点云内循环的距离计算；(c) 检查点云类型是否有 `size()` 方法；(d) 从字段名生成注册代码。
2. **[代码题]**：把 `RobotState` concept 拆成 3 个小角色 concept。编写一个只需要 `HasPose` 的函数和一个需要 `KinematicState` 的函数，验证类型在不同约束下的接受/拒绝行为。
3. **[跨章综合题]**：综合 继承与多态深入（虚函数）、变参模板折叠表达式与CRTP（CRTP）、预处理器与宏（宏）和本章（Concepts + Policy），为一个点云配准框架绘制完整的分层工具选型图：哪层用什么工具，层间如何连接。

---

## 16.11 C++23/26 前沿：`deducing this`、Contracts 与模式匹配 ⭐⭐⭐⭐

> **这一节解决什么问题**：Concepts 和 Policy 是 C++20 的最佳实践。但 C++ 标准仍在快速演进。理解即将到来的特性如何与 Concepts/Policy 交互，有助于做出更具前瞻性的设计决策。

### C++23/26 特性的工程影响评估

在评估新标准特性时，机器人项目需要权衡三个因素：编译器支持程度（GCC、Clang、MSVC 的版本要求）、构建系统兼容性（CMake 的 `CMAKE_CXX_STANDARD` 支持）和团队学习成本。C++23 的 `deducing this` 在 GCC 14+ 和 Clang 18+ 中可用，但 ROS 2 的 Jazzy/Rolling 发行版可能还在使用较旧的编译器。在采用新特性前，应先确认项目的编译器最低版本要求。

### `deducing this`（C++23）与 Policy/CRTP 的交汇

C++23 的显式对象参数（`this auto&& self`）不仅简化了 CRTP（见 变参模板折叠表达式与CRTP），还为 Policy-based Design 带来了新的可能性。传统 Policy 通常通过模板参数传递策略类型，然后在内部调用策略的静态方法。`deducing this` 允许基类的方法直接推导出完整的派生类型，使得 mixin 风格的 Policy 组合更加自然。

```cpp
// 传统 Policy：通过模板参数组合
template <typename DistancePolicy, typename SearchPolicy>
struct RegistrationKernel {
    double computeDistance(const Vec3& a, const Vec3& b) const {
        return DistancePolicy::compute(a, b);
    }
};

// C++23 mixin 风格：deducing this 让基类直接访问派生类的能力
struct RegistrationBase {
    auto computeDistance(this auto const& self, const Vec3& a, const Vec3& b) {
        return self.distanceImpl(a, b);  // 直接调用派生类的方法
    }
};
```

这种写法的优势是不需要把所有 Policy 都声明为模板参数——基类通过 `deducing this` 在调用时才推导出完整类型，减少了模板参数列表的冗长度。但缺点是失去了编译器在声明时对 Policy 接口的检查——约束只在调用时才触发。因此，建议 `deducing this` mixin 与 Concepts 配合使用，在入口处显式约束派生类满足必要的 concept。

### Contracts（C++26 提案）：运行时契约的标准化

回顾 16.5 节的核心结论：Concepts 只能表达语法和类型契约，不能证明坐标系、单位和数值合理性等数学语义。这些语义约束目前靠 `assert`、`if (...) throw` 和文档来保证——缺乏统一的表达方式。

C++26 正在推进的 Contracts 提案（前身是 C++20 被移除的 `[[expects]]`/`[[ensures]]`）将为运行时契约提供标准化语法。如果落地，它将填补 Concepts 留下的"语义契约"空白：

```cpp
// 未来 Contracts 语法（提案阶段，可能有变化）
template <PointCloud Cloud>
Vec3 computeCentroid(const Cloud& cloud)
    pre(cloud_traits<Cloud>::size(cloud) > 0)     // 前置条件：非空
    post(r: isFinite(r.x) && isFinite(r.y))       // 后置条件：结果有限
{
    // ...
}
```

Contracts 不替代 Concepts——它们工作在不同层次。Concepts 在编译期检查"类型层面的契约"（有没有 `size()` 方法），Contracts 在运行期检查"数据层面的契约"（点云是否非空）。两者组合形成完整的契约体系。

在机器人工程中，Contracts 的潜在价值尤其显著。当前机器人代码中大量的 `assert`、`if (...) throw` 和 `ROS_ASSERT` 缺乏统一的语义和可配置的执行策略。Contracts 提案允许在构建级别选择契约的执行模式（检查并终止、检查并抛异常、不检查），让安全关键的机器人系统可以在调试时严格检查所有契约、在生产环境中关闭性能敏感路径的契约检查。这比手动管理 `#ifndef NDEBUG` 包裹的断言更加系统化。

### 模式匹配（C++26 提案）与 `std::variant` 分派

C++26 还在推进模式匹配提案（P2688）。如果落地，它将极大简化 `std::variant` 的运行时分派——这正是 16.9 节"外层运行时选择"中 `std::visit` 所做的事情。模式匹配让 `visit` 的 lambda 嵌套变成类似 `switch` 的直观语法，降低了混合设计中运行时层的编写难度。

从机器人框架的角度看，模式匹配对"外层运行时选择"层的影响最大。当前 `std::visit` 的写法需要为每种 variant 类型写一个 lambda（或一个带多个 `operator()` 重载的 visitor struct），代码可读性较差。模式匹配将这些分支写成类似 `inspect (v) { <IcpKernel> k => k.run(); <GicpKernel> k => k.run(); }` 的直观形式，让运行时多态的代码和编译期多态的代码在风格上更加统一。

模式匹配与 Concepts 的关系是互补的——Concepts 约束编译期的类型选择，模式匹配简化运行期的值选择。两者共同服务于本章的核心目标：让每种变化在正确的层次被处理。编译期变化用 Concepts + Policy，运行期变化用模式匹配 + type erasure。这种分层设计在 C++26 中将变得更加自然和一致，编译期和运行期的工具各司其职，代码的声明式意图和实际的编译器行为之间的鸿沟将进一步缩小。

### 知识树：四章模板进阶的完整脉络

回顾 函数模板与类模板基础-本章 四章的递进关系，可以画出一棵完整的知识树：

```text
模板系统的核心问题
├── 类型不同 → 模板参数化（函数模板与类模板基础）
├── 类型能力不同 → 特化 + SFINAE + traits（模板特化SFINAE与类型萃取）
├── 参数数量不同 → 变参模板 + 折叠表达式（变参模板折叠表达式与CRTP）
├── 基类需知道派生类 → CRTP + 表达式模板（变参模板折叠表达式与CRTP）
└── 约束 + 策略组合 → Concepts + Policy（本章）
    ├── 接口约束：Concepts（编译期）
    ├── 算法变化：Policy（编译期）
    ├── 数据适配：traits（编译期）
    ├── 系统边界：type erasure/虚接口（运行期）
    └── 未来方向：
        ├── deducing this（简化 CRTP/mixin）
        ├── Contracts（运行时语义契约）
        ├── 模式匹配（简化 variant 分派）
        └── 静态反射（可能替代部分 traits）
```

每一章解决模板系统的一个变化维度；本章是汇合点，把前面所有工具收束成分层设计框架。掌握这棵知识树的结构，比记住任何一个 concept 的具体语法都更重要——因为语法会随标准版本变化，而"在正确的层次使用正确的工具"这个原则是稳定的。

> **本质洞察**：C++ 泛型设计的演进方向是"让程序员的意图和编译器的理解越来越接近"。SFINAE 是利用编译器的错误恢复机制间接表达约束——程序员的意图（"只对有 `.size()` 的类型可见"）和编译器看到的（"替换 `enable_if_t<has_size_v<T>, int>` 时，`has_size_v<int>` 为 false，`enable_if_t<false, int>` 没有 `type` 成员"）之间有巨大鸿沟。Concepts 把鸿沟缩小到"`HasSize<T>` 不满足"。Contracts 将进一步缩小到"前置条件 `size > 0` 不满足"。每一步都在让代码的声明式意图更接近实际的检查行为。

### 练习

1. **[设计题]**：假设你的机器人框架同时使用 Concepts（检查类型契约）和 Contracts（检查数据契约）。为一个 `alignPointClouds()` 函数设计完整的契约体系：Concepts 检查什么？Contracts 检查什么？哪些只能靠运行时断言？
2. **[代码题]**：用 C++23 的 `deducing this` 重写 变参模板折叠表达式与CRTP 中的一个 CRTP 基类。对比两种实现的代码量、编译错误信息质量和类型安全性。
3. **[调研题]**：查阅 C++26 模式匹配提案（P2688）的最新状态。它能否简化 16.9 节中 `std::visit` 的 lambda 嵌套？给出一个具体的改写示例。

---

## 本章小结

本章把模板进阶收束为工程分层。

| 主题 | 结论 |
|------|------|
| Concepts | 把模板语法和类型要求写到接口上 |
| requirement kinds | 简单、类型、复合、嵌套要求覆盖不同检查 |
| constraint satisfaction | 判断实参是否满足约束 |
| normalization | 把约束拆成可比较的原子组合 |
| subsumption | 更具体约束可在重载中胜出 |
| SFINAE 迁移 | 改善接口表达和错误诊断，不应无意改变类型集合 |
| 语义边界 | Concepts 不证明坐标系、单位、扰动方向和数学性质 |
| `constexpr`/`consteval` | 适合固定维度和编译期检查，不处理运行时配置 |
| Ranges | 适合表达数据管线，但热路径需按工具链和 profile 决策 |
| Policy | 把算法变化轴拆成编译期策略 |
| traits | 非侵入式适配外部数据类型 |
| type erasure | 运行时边界和配置层选择 |
| 显式实例化 | 控制模板编译时间和二进制体积 |

最终目标不是写更复杂的模板，而是让每种变化在正确层次发生：运行时变化放在工厂、虚接口或 type erasure；热路径变化放在 Policy 和 traits；模板入口要求用 Concepts 表达；预处理阶段不可替代的能力才交给宏。

### 本章知识树

回顾本章的核心脉络，可以画出一棵完整的知识树。这棵树的根节点是"如何管理泛型代码中的变化"，两个主干分别是"Concepts（约束表达）"和"Policy（策略组合）"：

```text
泛型代码的变化管理
├── 约束表达：Concepts（16.1-16.5）
│   ├── 动机：SFINAE 的痛点（16.1）
│   ├── 能力：requires 表达式的四种要求（16.2）
│   ├── 组合：subsumption 偏序（16.3）
│   ├── 迁移：SFINAE → Concepts 对照（16.4）
│   └── 边界：语法契约 ≠ 数学语义（16.5）
│
├── 编译期工具配合（16.6）
│   ├── constexpr：可编译期也可运行期
│   ├── consteval：必须编译期
│   ├── if constexpr + requires：可选接口
│   └── Ranges：数据管线
│
├── 策略组合：Policy-based Design（16.7-16.9）
│   ├── 从变化轴拆策略（16.7）
│   ├── traits 适配外部类型 + Policy 组合算法（16.8）
│   └── 外层运行时 + 内层编译期（16.9）
│
├── 选型决策（16.10）
│   └── 按变化位置选工具，不按语法喜好
│
└── C++23/26 展望（16.11）
    ├── deducing this：简化 mixin Policy
    ├── Contracts：填补语义契约空白
    └── 模式匹配：简化 variant 分派
```

从 16.1 的"为什么要把要求写在接口上"出发，经过 16.2 的"requires 表达式能检查什么"，到 16.3 的"编译器如何比较约束的具体性"，再到 16.4 的"从旧写法迁移"和 16.5 的"Concepts 不能证明什么"。下半章从 16.6 的"编译期工具配合"过渡到 16.7 的"算法变化轴如何拆成策略"，16.8 的"数据访问和算法策略分层"，16.9 的"外层运行时内层编译期"，最终在 16.10 收束为完整的选型决策流程。16.11 展望了 C++23/26 的新特性如何影响当前的设计决策。每一节不是孤立的语法知识点，而是分层设计中的一个环节。理解这个脉络比记住任何一个 concept 的语法都更重要。

如果整个分层设计只能记住一个判断框架，应该是这样的：面对一个接口设计问题时，先问"这个变化发生在编译期还是运行期？"编译期→用 Concepts + Policy + traits。运行期→用虚接口 + 工厂 + type erasure。两者都需要→外层运行时，内层编译期。这个二分法能覆盖 80% 的选型决策。

### 与前后章节的关系

本章是 函数模板与类模板基础-本章 四章模板进阶的终点和汇合点。函数模板与类模板基础 教"如何写模板"，模板特化SFINAE与类型萃取 教"类型不同时怎么办"，变参模板折叠表达式与CRTP 教"参数数量不同和编译期多态"，本章把前三章的所有工具收束成现代 C++ 的泛型设计框架。学完四章后，读者应该能在遇到任何泛型设计问题时，快速定位"这个问题属于哪个变化维度"并选择正确的工具。

---

## 累积项目：Mini Registration 的 Concepts + Policy 组合

本章把 Mini Registration 从“能注册算法”推进到“能约束并组合策略”。项目重点是正负例、错误诊断和分层测试。

### 目标结构

```text
mini_registration/
  concepts.hpp
  cloud_traits.hpp
  distance_policies.hpp
  search_policies.hpp
  optimizer_policies.hpp
  registration_kernel.hpp
  registration_factory.hpp
  checks/
    concept_positive_check.cpp
    concept_negative_check.cpp
    policy_combination_check.cpp
    semantic_invariant_check.cpp
```

### Concepts

```cpp
#pragma once

#include <concepts>
#include <cstddef>
#include <Eigen/Dense>

template <typename Traits, typename Cloud>
concept CloudTraits =
    requires(const Cloud& cloud, std::size_t i) {
        { Traits::size(cloud) } -> std::convertible_to<std::size_t>;
        { Traits::point(cloud, i) } -> std::convertible_to<Eigen::Vector3d>;
    };

template <typename Policy, typename SourcePoint, typename TargetPoint>
concept DistancePolicyFor =
    requires(const Policy& policy,
             const SourcePoint& source,
             const TargetPoint& target) {
        { policy(source, target) } -> std::convertible_to<double>;
    };

template <typename Search, typename SourceCloud, typename TargetCloud, typename Distance>
concept SearchPolicyFor =
    requires(const Search& search,
             const SourceCloud& source,
             const TargetCloud& target,
             const Distance& distance) {
        search.find(source, target, distance);
    };
```

### traits

```cpp
#pragma once

#include <vector>
#include <Eigen/Dense>

template <typename Cloud>
struct cloud_traits;

template <>
struct cloud_traits<std::vector<Eigen::Vector3d>> {
    static std::size_t size(const std::vector<Eigen::Vector3d>& cloud) {
        return cloud.size();
    }

    static const Eigen::Vector3d& point(
        const std::vector<Eigen::Vector3d>& cloud,
        std::size_t i) {
        return cloud[i];
    }
};
```

### 距离策略

```cpp
#pragma once

#include <Eigen/Dense>

struct PointToPointDistance {
    double operator()(const Eigen::Vector3d& source,
                      const Eigen::Vector3d& target) const {
        return (source - target).squaredNorm();
    }
};

struct PlanePoint {
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
};

struct PointToPlaneDistance {
    double operator()(const Eigen::Vector3d& source,
                      const PlanePoint& target) const {
        return std::pow(target.normal.dot(source - target.point), 2);
    }
};
```

### 策略组合

```cpp
#pragma once

#include <utility>
#include <vector>

template <typename Search, typename Distance>
class Matcher {
public:
    Matcher(Search search, Distance distance)
        : search_(std::move(search)),
          distance_(std::move(distance)) {}

    template <typename SourceCloud, typename TargetCloud>
    requires CloudTraits<cloud_traits<SourceCloud>, SourceCloud> &&
             CloudTraits<cloud_traits<TargetCloud>, TargetCloud>
    std::vector<int> match(const SourceCloud& source,
                           const TargetCloud& target) const {
        return search_.find(source, target, distance_);
    }

private:
    Search search_;
    Distance distance_;
};
```

### 正例检查

```cpp
#include "cloud_traits.hpp"
#include "concepts.hpp"
#include "distance_policies.hpp"

using EigenCloud = std::vector<Eigen::Vector3d>;

static_assert(CloudTraits<cloud_traits<EigenCloud>, EigenCloud>);
static_assert(DistancePolicyFor<
    PointToPointDistance,
    Eigen::Vector3d,
    Eigen::Vector3d>);
static_assert(DistancePolicyFor<
    PointToPlaneDistance,
    Eigen::Vector3d,
    PlanePoint>);

int main() {
    return 0;
}
```

### 负例检查

负例可以作为单独编译目标，目标是让错误信息指向 concept 名称。也可以用 `static_assert` 表示预期失败：

```cpp
#include "concepts.hpp"

struct BadCloud {};

template <typename Cloud>
struct cloud_traits;

static_assert(!requires(const BadCloud& cloud, std::size_t i) {
    cloud_traits<BadCloud>::size(cloud);
    cloud_traits<BadCloud>::point(cloud, i);
});

int main() {
    return 0;
}
```

如果构建系统支持“预期编译失败”的测试，可以写一个故意调用 `Matcher` 的坏例子，并检查诊断中包含 `CloudTraits` 或缺失表达式名称。

### 语义测试目标

Concepts 通过后，还要测语义：

| 测试 | 目的 |
|------|------|
| 空点云输入 | 运行时前置条件清晰 |
| 点到点距离非负 | 距离策略基本性质 |
| 点到面法向归一化检查 | 防止语义错误 |
| 坐标系约定样例 | 明确 source/target 方向 |
| 优化器最大迭代次数 | 运行时参数不做模板参数 |
| 策略组合结果一致 | 不同搜索策略在小数据上结果可比 |

### 显式实例化目标

如果项目只支持少数组合，可以给常用组合显式实例化：

```cpp
extern template class Matcher<KdTreeSearch, PointToPointDistance>;
extern template class Matcher<KdTreeSearch, PointToPlaneDistance>;
```

在 `.cpp` 中：

```cpp
template class Matcher<KdTreeSearch, PointToPointDistance>;
template class Matcher<KdTreeSearch, PointToPlaneDistance>;
```

同时用 `nm -C` 检查重复实例化是否减少，用构建日志观察增量编译是否改善。

### 项目验收点

| 检查项 | 合格标准 |
|--------|----------|
| Concepts | 至少有 `CloudTraits`、`DistancePolicyFor`、`SearchPolicyFor` |
| 正例 | 合法点云和合法策略通过 `static_assert` |
| 负例 | 错误类型在接口约束处失败，诊断可定位 |
| 策略组合 | 至少两种距离策略和两种搜索策略可组合 |
| 分层 | 数据访问只在 traits 中，算法变化只在 Policy 中 |
| 运行时参数 | 阈值、迭代次数、体素大小保留为对象成员或配置 |
| 语义测试 | 坐标系、单位、法向归一化等不依赖 Concepts 证明 |
| 构建成本 | 观察模板实例化、符号数量和二进制体积 |
| 边界 | 外层可接 预处理器与宏 注册表或 继承与多态深入 虚接口 |

---

## 延伸阅读

1. cppreference：Concepts、constraints and concepts、`requires` expression、standard library concepts。
2. ISO C++ Core Guidelines：模板约束、Concepts、泛型编程相关章节。
3. Andrei Alexandrescu, *Modern C++ Design*：Policy-based Design 的经典来源。
4. small_gicp 源码：观察搜索、因子、归约和 traits 的组合。
5. Eigen、Sophus、manif 源码：理解 CRTP、traits 和几何类型接口约定。
6. range-v3 与 C++20 Ranges 文档：理解管线式算法表达和工具链边界。

---

## 故障排查手册

| 现象 | 常见原因 | 检查方式 | 修复方式 |
|------|----------|----------|----------|
| concept 不满足 | 缺少 required expression | 看第一个 failed requirement | 补 traits 或调整 concept |
| 可用类型被拒绝 | 返回类型约束过窄 | 对比 `same_as` 和 `convertible_to` | 放宽到真实最小需求 |
| 重载二义 | concept 重叠但无蕴含关系 | 检查 subsumption 层次 | 建立层次或用 `if constexpr` |
| 迁移后行为变化 | Concepts 改变可接受类型集合 | 对比正负例 | 调整 concept 或保留旧 traits |
| Concepts 通过但结果错 | 语法满足，数学语义不满足 | 性质测试、数值测试 | 增加运行时 invariant 和文档约定 |
| `consteval` 编译失败 | 传入运行时变量 | 检查参数来源 | 改 `constexpr` 或普通函数 |
| Ranges 在某平台不可用 | 标准库或编译选项不匹配 | 检查 `-std` 和工具链 | 提供 STL 算法版本 |
| Policy 组合编译慢 | 模板实例太多 | `-ftime-trace`、构建日志 | 减少组合、显式实例化 |
| 二进制体积增大 | 每个组合生成一份代码 | `nm -C`、`size` | 合并策略或移到运行时层 |
| ABI 不稳定 | 模板实现暴露过多 | 观察头文件变更影响 | 边界处使用虚接口或 type erasure |
| `requires` 子句未参与重载 | 写成 `static_assert` 而非 `requires` | 检查函数声明位置 | 把约束从函数体移到声明 |
| Eigen 表达式传入 concept 函数后重复计算 | 表达式模板延迟求值 | profiler 检查热路径 | 入口处 `.eval()` 或用具体类型参数 |
| `auto` 参数推导出意外类型 | 简写模板的推导规则与预期不同 | 打印推导类型 `typeid(T).name()` | 使用显式 concept 约束 |

**常见故障场景的系统化诊断**：

故障排查时建议按三层顺序定位问题：

1. **编译期层**：检查 concept 是否满足（看 failed requirement）、检查模板参数推导是否正确、检查 subsumption 是否给出预期的偏序。
2. **链接期层**：检查模板是否在需要的翻译单元中实例化（显式实例化是否遗漏）、检查是否存在 ODR 违反（不同翻译单元看到不同的 concept 定义）。
3. **运行期层**：检查传入的数据是否满足语义约束（concept 无法检查的部分）、检查 Policy 组合是否在特定参数组合下产生数值问题。

到这里，C++ 语言进阶的泛型部分完成闭环：宏处理类型系统之前的 token，Concepts 表达类型系统内的接口要求，traits 适配外部数据类型，Policy 组合高性能算法内核，type erasure 和虚接口承担运行时边界。下一阶段进入并发与系统编程，关注点会从”如何表达接口”转向”如何在多线程、实时和硬件约束下稳定运行”。
