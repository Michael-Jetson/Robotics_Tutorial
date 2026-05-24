# CMake 从入门到工程化

> **难度**：⭐⭐～⭐⭐⭐ | **建议用时**：1 周 | **前置要求**：C++语言进阶/编译模型基础 C++ 编译模型、基本 CMake 使用经验

---

## 前置自测

> 📋 答不出 >= 2 题时，先回顾 C++语言进阶/编译模型基础 和 CMake 入门教程。

1. C++ 编译的四个阶段是什么？链接阶段做什么？
2. `#include` 是在哪个阶段被处理的？头文件路径如何传递给编译器？
3. `add_library()` 和 `add_executable()` 的区别是什么？
4. 什么是”静态库”和”共享库”？它们在链接时的行为有什么不同？
5. `find_package(Eigen3 REQUIRED)` 成功后，`Eigen3::Eigen` 代表什么？

---

## 本章目标

学完本章后，你应该能够：

- 解释 `PUBLIC`、`PRIVATE`、`INTERFACE` 三个关键字的传播语义，并根据头文件中是否暴露第三方类型正确选择
- 为一个 SLAM 核心库编写可安装、可被下游 `find_package()` 使用的 CMakeLists
- 将 ROS2 wrapper 与核心算法库分离，使核心库不依赖 `rclcpp`
- 配置 CUDA 可选后端，使 CPU 用户不需要安装 CUDA 工具链
- 使用 CTest 标签分层管理单元测试、集成测试和数据集测试

---

```text
CMake 从入门到工程化
├── 基础概念
│   ├── 0. CMake 为什么经常成为隐性故障源
│   ├── 1. 从全局变量到 target-based CMake（PUBLIC / PRIVATE / INTERFACE）
│   └── 2. find_package：Config Mode vs Module Mode
├── 项目组织
│   ├── 3. 可复用 SLAM 核心库的组织方式（核心库与 ROS2 分离）
│   ├── 4. 安装与导出（Config.cmake + Targets.cmake）
│   └── 5. FetchContent / ExternalProject / 系统依赖
├── ROS2 集成
│   └── 6. ament_cmake 与标准 CMake 的关系
├── 进阶工程化
│   ├── 7. CUDA 与混合语言构建
│   └── 8. 测试、Sanitizer 与编译缓存
├── 深入 target 模型
│   ├── 13. PUBLIC / PRIVATE / INTERFACE 依赖传播的完整案例
│   ├── 14. 安装导出完整链路（从源码到 find_package）
│   └── 15. ROS2 wrapper 与核心库边界
├── 工程闭环
│   ├── 16. CUDA、测试、Sanitizer 与 CI 的组合
│   └── 17. 可交付 CMake 工程的检查表
└── 故障排查手册
```

> **学习目标**：本章要解决的问题不是”会写一个能编译的 CMakeLists.txt”，而是理解现代 CMake 如何表达 C++ 项目的依赖关系、编译边界、安装导出、测试工具链和大型机器人项目组织方式。读完后，你应能看懂 KISS-ICP、GTSAM、Ceres、ROS2 wrapper 和 CUDA 项目的构建结构，并能为自己的 SLAM 或控制库设计可复用的构建系统。

---

## 0. 为什么 CMake 经常成为机器人项目的隐性故障源 ⭐

很多 C++ 初学者第一次写 CMake 时，会形成一个非常直接的心智模型：

```cmake
include_directories(...)
link_directories(...)
add_executable(...)
target_link_libraries(...)
```

只要程序能编译，就觉得构建系统完成了。但大型机器人项目的失败往往不发生在“完全编不过”的阶段，而发生在更隐蔽的地方：

- 在自己电脑上能编译，换一台机器找不到依赖。
- Debug 和 Release 链接了不同版本的 Eigen、OpenCV 或 PCL。
- 下游项目包含你的头文件后，还要手动补一堆 include 路径。
- ROS2 wrapper 和 standalone 库互相污染，核心算法离不开 ROS2。
- CUDA 架构没设对，在 Jetson 上编译很慢或运行失败。
- 测试、示例、工具、Python 绑定全部挤在一个 target 中，无法单独构建。

CMake 工程化的目标是把这些隐性关系显式表达出来：

| 关系 | CMake 中的表达 |
|------|----------------|
| 这个库需要哪些头文件 | `target_include_directories` |
| 这个库依赖哪些库 | `target_link_libraries` |
| 依赖是否传递给下游 | `PUBLIC/PRIVATE/INTERFACE` |
| 安装后如何被别人找到 | `install()` + `export()` + Config |
| 是否启用测试和工具 | `option()` |
| 是否支持 ROS2 wrapper | 独立 package 或条件构建 |

本章的主线是：现代 CMake 不是脚本拼接工具，而是 C++ 目标关系建模语言。

---

## 1. 从全局变量到 target-based CMake ⭐⭐

> **这一节解决什么问题**：为什么现代 CMake 不再推荐 `include_directories()`、`link_directories()` 这些全局命令？target-based 模型解决了什么根本性的工程问题？

### 为什么传统 CMake 模型不可持续

要理解现代 CMake 为什么以 target 为中心，首先要理解传统 CMake 的”全局变量模型”到底出了什么问题。这不是语法偏好的问题，而是工程规模增长后的结构性失败。

传统 CMake（2.x 时代，约 2000-2014 年）的心智模型是”设置一些全局变量，然后编译”。`include_directories()` 把路径加到一个全局列表里，`add_definitions()` 把宏加到全局列表里，`link_directories()` 也是全局的。这些命令的作用范围不是”当前 target”，而是”当前 CMakeLists.txt 文件中之后定义的所有 target”。

**这个模型的根本缺陷是依赖关系不可追踪。** 当项目只有一个可执行文件时，全局设置没有问题——所有编译选项都指向同一个目标。但机器人项目通常包含多个 target：核心算法库、ROS2 wrapper、命令行工具、单元测试、Python 绑定、CUDA 内核。每个 target 的依赖需求不同：核心库需要 Eigen，wrapper 额外需要 rclcpp，测试需要 GTest，CUDA 内核需要 CUDA toolkit。全局命令无法表达”这个依赖只属于某个 target”，也无法表达”这个依赖应该传递给下游还是不应该”。

可以把传统 CMake 类比为一间没有隔墙的开放式办公室。所有人共用同一张桌子（全局变量），你往桌上放了一杯咖啡（`add_definitions(-DUSE_FAST_MATH)`），所有人都得喝——即使有些人对咖啡过敏（某些 target 不应该带这个宏）。现代 CMake 的 target-based 模型则像有隔间的办公室：每个人有自己的桌子（target），桌上只放自己需要的东西（`target_compile_definitions`），需要共享的物品放在公共区域（`PUBLIC` 依赖），私人物品不外露（`PRIVATE` 依赖）。

**全局污染的真实案例。** 在一个典型的 SLAM 项目中，传统 CMake 可能这样写：

```cmake
# 传统 CMake 写法——所有命令都是全局的
include_directories(include)
add_definitions(-DUSE_FAST_MATH)
link_directories(/usr/local/lib)
```

这三行看似无害，但它们的影响范围是灾难性的：

| 全局命令 | 实际影响范围 | 可能引发的问题 |
|----------|-------------|---------------|
| `include_directories(include)` | 后续所有 target | 测试文件意外包含了生产代码的私有头文件 |
| `add_definitions(-DUSE_FAST_MATH)` | 后续所有 target | 单元测试的数值精度测试因为快速数学模式而失败 |
| `link_directories(/usr/local/lib)` | 后续所有 target | 工具程序链接了不该链接的库，部署时缺依赖 |

这些问题在小项目中不会出现，因为所有 target 碰巧需要相同的配置。但项目一旦增长到 5 个以上 target（这在机器人项目中很常见），全局污染就会以各种隐蔽的方式爆发。

**CMake 3.0（2014 年）的范式转变。** CMake 3.0 引入了”target 属性”的概念，把编译选项从全局变量迁移到 target 的属性上。这个转变的核心思想来自软件工程中的封装原则——每个 target 是一个自包含的构建单元，明确声明自己需要什么（构建需求）和向外提供什么（使用需求）。`PUBLIC`、`PRIVATE`、`INTERFACE` 三个关键字正是表达这两组关系的工具。

这个转变不只是语法变化。它从根本上改变了 CMake 的信息模型——从”设置全局状态后编译”变成”声明 target 之间的依赖关系后生成构建图”。后者更接近编译器和链接器的实际工作方式：编译器处理的是一个个翻译单元（对应 target 的源文件），链接器处理的是 target 之间的符号依赖。target-based CMake 让构建描述和实际构建过程更加一致。

> **本质洞察**：现代 CMake 的 target-based 模型不是”新语法”，而是把构建系统从”命令式脚本”提升到”声明式依赖图”。传统 CMake 描述的是”按什么步骤编译”，现代 CMake 描述的是”各个组件之间的关系是什么”。这和从 Makefile 到 CMake 的跃迁是同一类进步——抽象层次提高了，构建系统能做更好的决策（如并行构建、增量编译、安装导出）。

现代 CMake 的原则是：**所有属性绑定到 target，不使用全局命令**。

```cmake
add_library(slam_core src/tracker.cpp src/map.cpp)

target_include_directories(slam_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(slam_core PUBLIC cxx_std_17)
target_compile_definitions(slam_core PRIVATE SLAM_CORE_BUILDING_LIBRARY)
```

这段代码表达了三个边界：

1. `include/` 是对外 API，需要传递给下游。
2. `src/` 只给库内部使用，不传递给下游。
3. C++17 是 ABI 和头文件接口的一部分，必须让下游知道。

### 1.1 PUBLIC、PRIVATE、INTERFACE 的真实含义

| 关键字 | 当前 target 使用 | 下游 target 使用 | 典型场景 |
|--------|------------------|------------------|----------|
| `PRIVATE` | 是 | 否 | `.cpp` 内部使用的 spdlog |
| `PUBLIC` | 是 | 是 | 头文件暴露 Eigen 类型 |
| `INTERFACE` | 否 | 是 | header-only 库 |

机器人项目中最常见的错误是把所有依赖都写成 `PRIVATE`。比如你的头文件里有：

```cpp
#include <Eigen/Core>

class PoseEstimator {
public:
  Eigen::Matrix4d estimate();
};
```

那么 Eigen 是公共接口的一部分。CMake 应写：

```cmake
find_package(Eigen3 REQUIRED)
target_link_libraries(slam_core PUBLIC Eigen3::Eigen)
```

如果写成 `PRIVATE`，`slam_core` 自己能编译，但下游项目包含 `PoseEstimator` 头文件时可能找不到 Eigen。这种错误会在”别人使用你的库”时才暴露。

> **本质洞察**：现代 CMake 的 target-based 思想可以类比面向对象编程中的封装。传统 CMake 的 `include_directories()` 就像全局变量——所有 target 都能看到，你不知道谁在用。`target_include_directories()` 就像成员变量——只有声明了 `PUBLIC` 的才暴露给外部。这个类比还能延伸：`PRIVATE` 依赖是实现细节（private 成员），`PUBLIC` 依赖是公共接口（public 成员），`INTERFACE` 依赖是纯接口规范（abstract base class）。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：把所有依赖都写成 PUBLIC**
>
> **错误做法**：`target_link_libraries(slam_core PUBLIC Eigen3::Eigen Ceres::ceres spdlog::spdlog yaml-cpp)`
>
> **现象**：下游项目链接 `slam_core` 后，被迫安装和链接 Ceres、spdlog、yaml-cpp，即使它们只在 `slam_core` 内部使用。
>
> **根本原因**：混淆了”我需要”和”别人也需要”。
>
> **正确做法**：逐个检查每个依赖是否出现在公共头文件中。只在头文件中使用的才写 `PUBLIC`，只在 `.cpp` 中使用的写 `PRIVATE`。

> 💡 **概念误区：认为 `target_link_libraries` 只是链接库**
>
> **新手想法**：”`target_link_libraries` 就是告诉链接器要链接哪个 `.so`。”
>
> **实际上**：`target_link_libraries` 做三件事：(1) 链接库文件；(2) 传播 include 路径；(3) 传播编译定义和编译特性。它本质上是”使用一个 target 的全部使用需求”。所以 Eigen 虽然是 header-only 库（没有 `.so`），`target_link_libraries(... Eigen3::Eigen)` 仍然有意义——它传播了 Eigen 的 include 路径。

### 练习

1. **[判断题]** 一个库的公共头文件返回 `Eigen::Vector3d`，CMake 中 Eigen 应写 `PUBLIC` 还是 `PRIVATE`？如果实现文件使用 spdlog 但头文件不暴露 spdlog 类型，spdlog 应写什么？
2. **[排查题]** 你的 `slam_core` 库在本仓库内能编译，但安装后下游项目报 `fatal error: Eigen/Core: No such file or directory`。列出至少 3 个可能原因。

---

## 2. find_package：依赖是如何被找到的 ⭐⭐

> **这一节解决什么问题**：`find_package(Eigen3 REQUIRED)` 一行代码背后发生了什么？为什么有两种查找模式？理解这个机制，才能在依赖查找失败时知道从哪里排查。

### find_package 要解决的根本问题

C++ 没有像 Python（pip）、Rust（cargo）、Go（modules）那样的内置包管理器。一个 C++ 库安装到系统后，它的头文件可能在 `/usr/include`、`/usr/local/include`、`/opt/ros/humble/include` 或任何自定义路径；它的库文件可能在 `lib`、`lib64`、`lib/x86_64-linux-gnu` 或更深的子目录。不同发行版、不同安装方式、不同编译选项都会导致路径不同。

`find_package()` 的核心使命就是：**把"这个库叫什么名字"翻译成"头文件在哪里、库文件在哪里、需要什么编译选项"**。它是 CMake 中连接"我想用 Eigen"和"编译器需要 `-I/usr/include/eigen3`"之间的桥梁。

**为什么有两套查找模式？** 这涉及一个"谁更了解自己"的问题。一个库的构建配置，到底应该由库的作者提供，还是由 CMake 社区提供？

| 模式 | 查找对象 | 谁提供的 | 信息完整度 |
|------|----------|----------|-----------|
| Config Mode | `Eigen3Config.cmake` | **库的作者**在安装时生成 | 最完整——作者最了解自己的库 |
| Module Mode | `FindEigen3.cmake` | **CMake 社区**或项目自己编写 | 可能过时或不完整 |

**Config Mode 的设计动机。** 当一个库通过 `cmake --install` 安装时，它可以同时安装一个 `<Package>Config.cmake` 文件，里面记录了这个库的所有使用信息：头文件路径、库文件路径、依赖的其他库、需要的编译特性。这些信息由库的作者在 `CMakeLists.txt` 中用 `install(EXPORT ...)` 生成——没有人比作者自己更清楚这些细节。Config Mode 是"自描述包"的思想：库不只安装自己的文件，还安装一份"使用说明书"。

**Module Mode 的历史背景。** 在 Config Mode 普及之前（大约 2015 年以前），大多数库不提供 CMake 配置文件。CMake 自带了一批 `Find<Package>.cmake` 脚本（如 `FindOpenGL.cmake`、`FindPythonInterp.cmake`），用启发式搜索来定位库的头文件和库文件。这些脚本本质上是"猜"——在常见路径下搜索特定文件名，如果找到就设置一些变量。这种方式脆弱、容易过时，而且不同版本的 `Find` 脚本可能给出不同的结果。

**现代推荐：优先 Config Mode。** 今天大多数高质量 C++ 库（Eigen、Ceres、GTSAM、PCL 5.x、OpenCV 4.x）都提供 Config 文件。Config Mode 的优势是提供 imported target（如 `Eigen3::Eigen`、`Ceres::ceres`），这些 target 不只是库路径的简单包装——它们携带了完整的使用需求：include 路径、编译定义、依赖传播、编译特性。用 `target_link_libraries(my_lib PUBLIC Eigen3::Eigen)` 链接一个 imported target，比手写 `include_directories(${EIGEN3_INCLUDE_DIR})` 安全得多，因为前者会自动传播所有必要的编译选项。

如果没有 Config Mode，每次使用 Eigen 都要手动设置 include 路径，而且不同项目可能设置不同——有些写 `/usr/include/eigen3`，有些写 `/usr/local/include/eigen3`，交叉编译时路径又完全不同。Config Mode 把这些平台差异封装在库安装时就生成好的配置文件中，使用者只需要写 `find_package(Eigen3 REQUIRED)` 一行。

### 2.1 为什么不要手写路径

手写绝对路径是传统 CMake 时代的遗留习惯。它的根本问题是把"我的电脑上的路径"硬编码到了构建脚本里——这等于假设所有人的环境和你一模一样。在机器人团队中，开发者可能使用 Ubuntu 22.04 或 24.04，安装路径可能是 `/usr`、`/usr/local` 或 conda 环境，Jetson 嵌入式板的路径布局又完全不同。`find_package()` + imported target 的组合把平台差异封装在库安装时生成的配置文件中，使用者不需要关心底层路径。

反面写法：

```cmake
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
target_link_libraries(my_node ceres)
```

这个写法有几个问题：

- `/usr/local` 不一定存在。
- `ceres` 可能链接到错误版本。
- Ceres 依赖 glog、gflags、SuiteSparse 等，手写时容易漏。
- 下游无法知道依赖从哪里来。

更好的写法：

```cmake
find_package(Ceres REQUIRED)
target_link_libraries(slam_backend PRIVATE Ceres::ceres)
```

这样 Ceres 自己导出的依赖关系会随 target 传播。

### 2.2 组件化查找：只链接你真正需要的

大型库（如 PCL、OpenCV、Boost）通常包含几十个组件模块。如果不指定组件，`find_package` 会拉入整个库的所有模块——编译时间和链接依赖都会显著增加。在 Jetson 等嵌入式平台上，未使用的组件还会占用宝贵的存储空间和运行时内存。组件化查找就是"只点你要吃的菜"，不让构建系统替你点全套。

PCL 和 OpenCV 这种大库通常应按组件查找：

```cmake
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)
```

原因是机器人项目很容易链接过多组件，导致编译慢、链接慢、运行时依赖复杂。只链接需要的组件，可以减少二进制体积和部署难度。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：手写绝对路径代替 find_package**
>
> **错误做法**：`include_directories(/usr/local/include/eigen3)` `link_directories(/usr/local/lib)`
>
> **现象**：在你的机器上能编译。同事的机器、CI 服务器、Jetson 上全部失败。
>
> **根本原因**：绝对路径是本机特有的。`/usr/local` 不是标准路径，不同发行版和安装方式路径不同。
>
> **正确做法**：`find_package(Eigen3 REQUIRED)` + `target_link_libraries(... Eigen3::Eigen)`。让 CMake 的搜索机制处理平台差异。

### 练习

1. **[分析题]** `find_package` 的 Config Mode 和 Module Mode 有什么区别？现代 C++ 库（如 Ceres、GTSAM）更推荐哪种？为什么？
2. **[编程题]** 写一个 CMakeLists 只链接 PCL 的 `common` 和 `filters` 组件。解释为什么不应该写 `find_package(PCL REQUIRED)` 而不指定组件。

---

## 3. 一个可复用 SLAM 核心库的组织方式 ⭐⭐

> **这一节解决什么问题**：很多初学者的第一个 SLAM 项目把所有代码塞进一个 `CMakeLists.txt`，一个 target 编出一个可执行文件，就认为"构建完成了"。但当你想让别人用你的算法、想在没有 ROS2 的环境下测试、想为嵌入式设备交叉编译时，这种结构就会崩溃。本节展示的不只是一个目录结构，而是一种**让算法库可复用的工程思维**。

### 为什么核心算法必须独立于 ROS2

机器人项目最常见的架构错误是：在核心算法代码中直接 `#include <rclcpp/rclcpp.hpp>`。这看似方便——不需要在 ROS2 消息和内部数据结构之间转换。但这个"方便"的代价是：核心算法永远无法脱离 ROS2 运行。

想象你完成了一个激光 SLAM 系统，论文投出去后审稿人要求你在 KITTI 数据集上跑 benchmark。如果核心算法依赖 rclcpp，你就不得不在测试环境中安装完整的 ROS2 运行时——但 KITTI 数据集是纯文件格式，不需要 ROS2 的任何功能。更糟糕的是，如果你想给算法加 Python 绑定（用 pybind11），绑定层也会被 ROS2 消息类型拖住。如果你想部署到 Jetson 嵌入式板上做纯 C++ 推理，又要带整套 ROS2 运行时。

核心库与 ROS2 wrapper 分离的本质是**关注点分离**（Separation of Concerns）：算法只关心"给我点云，我输出位姿"；消息订阅、参数读取、TF 发布属于系统集成层。两者的变化频率不同——算法可能一个月不改，但 ROS2 wrapper 可能因为消息格式变更、QoS 策略调整而频繁修改。把它们放在同一个 target 里，意味着改 wrapper 也要重新编译算法。

推荐结构：

```text
my_slam/
  CMakeLists.txt
  include/my_slam/
    tracker.hpp
    map.hpp
    backend.hpp
  src/
    tracker.cpp
    map.cpp
    backend.cpp
  apps/
    run_dataset.cpp
  tests/
    test_tracker.cpp
  ros2/
    my_slam_ros/
      CMakeLists.txt
      src/slam_node.cpp
```

这个结构有一个重要设计：核心库不依赖 ROS2。ROS2 wrapper 位于单独目录，只做消息转换、参数读取和生命周期管理。

### 3.1 核心 CMakeLists 示例

下面的 CMakeLists 展示了核心库的完整构建配置。注意三个关键设计决策：Eigen 写 `PUBLIC`（因为公共头文件暴露了 Eigen 类型）、Ceres 写 `PRIVATE`（因为只在 `.cpp` 中使用）、应用程序通过 `option` 可选构建（不强迫库的使用者也编译命令行工具）。每个决策都反映了前面讨论的"依赖是否出现在公共接口"这个核心判断。

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_slam VERSION 0.1.0 LANGUAGES CXX)

option(MY_SLAM_BUILD_TESTS "Build unit tests" ON)
option(MY_SLAM_BUILD_APPS "Build command line tools" ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_library(my_slam_core
  src/tracker.cpp
  src/map.cpp
  src/backend.cpp
)

target_compile_features(my_slam_core PUBLIC cxx_std_17)

target_include_directories(my_slam_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(my_slam_core
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    Ceres::ceres
)

if(MY_SLAM_BUILD_APPS)
  add_executable(run_dataset apps/run_dataset.cpp)
  target_link_libraries(run_dataset PRIVATE my_slam_core)
endif()
```

这里把 Ceres 写成 `PRIVATE` 是因为假设公共头文件不暴露 Ceres 类型。如果 `backend.hpp` 中包含 Ceres 类型，则必须改成 `PUBLIC`。

---

## 4. 安装与导出：让别人能找到你的库 ⭐⭐

> **这一节解决什么问题**：你的库在自己的电脑上能编译，但安装到 `/usr/local` 后别人能用 `find_package()` 找到它吗？安装导出是把"本地能编"变成"任何人能用"的关键一步。

### 安装导出的本质：把"怎么用我"写成机器可读的说明书

一个 C++ 库安装到系统后，下游项目需要知道三件事：头文件在哪里、库文件在哪里、还要链接哪些传递依赖。手动写这些信息（`include_directories(/usr/local/include/my_slam)`、`link_directories(/usr/local/lib)`）不仅繁琐而且脆弱——不同平台的路径布局不同，传递依赖容易遗漏。

CMake 的安装导出机制自动生成一份"使用说明书"（`Config.cmake` + `Targets.cmake`），让下游只需要 `find_package(my_slam REQUIRED)` 一行就能获得完整的使用信息。这本质上是把 target 的所有公共属性——include 路径、链接库、编译特性、依赖传播——序列化到磁盘上，让跨项目复用成为可能。

一个库如果只能在源码目录里被使用，还不算工程化。安装导出让下游可以写：

```cmake
find_package(my_slam REQUIRED)
target_link_libraries(my_app PRIVATE my_slam::my_slam_core)
```

### 4.1 安装 target 和头文件

```cmake
install(TARGETS my_slam_core
  EXPORT my_slamTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
```

### 4.2 导出 Config

```cmake
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/my_slamConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(EXPORT my_slamTargets
  FILE my_slamTargets.cmake
  NAMESPACE my_slam::
  DESTINATION lib/cmake/my_slam
)
```

完整 Config 还需要处理依赖查找。工程中通常会写一个 `cmake/my_slamConfig.cmake.in`：

```cmake
@PACKAGE_INIT@

include(CMakeFindDependencyMacro)
find_dependency(Eigen3)

include("${CMAKE_CURRENT_LIST_DIR}/my_slamTargets.cmake")
```

如果你的公共接口暴露 Ceres，也要在 Config 中 `find_dependency(Ceres)`。

---

## 5. FetchContent、ExternalProject 与系统依赖 ⭐⭐

> **这一节解决什么问题**：C++ 没有 `pip install` 或 `cargo add`。当你的项目依赖十几个库时，怎么管理它们的版本、下载和构建？三种方式各有适用场景，选错了会让构建系统变成噩梦。

### C++ 依赖管理的根本困难

Python 有 pip，Rust 有 cargo，Go 有 modules——这些语言的包管理器能在一条命令内下载、编译、安装依赖，并自动处理版本冲突。C++ 至今没有一个被广泛采用的标准包管理器（vcpkg 和 Conan 在逐步填补这个空白，但渗透率仍然远低于 pip/cargo）。原因是 C++ 的编译模型决定了依赖管理的复杂性：不同编译器（GCC/Clang/MSVC）、不同标准版本（C++17/20/23）、不同链接方式（静态/动态）、不同平台（x86/ARM/RISC-V）和不同构建类型（Debug/Release）都会影响二进制兼容性。同一个库的同一个版本，在不同配置下编译出来的 `.so` 或 `.a` 可能互不兼容。

在这个背景下，CMake 提供了三种不同层次的依赖管理机制，每种解决不同的问题。理解它们的设计动机，比记住语法更重要。

机器人项目常见三种依赖管理方式：

| 方式 | 优点 | 缺点 | 适合场景 |
|------|------|------|----------|
| 系统安装 | 快、稳定 | 版本不可控 | Eigen、OpenCV、PCL |
| FetchContent | 与源码一起配置 | 配置时间变长 | 小型 header-only 或轻量库 |
| ExternalProject | 隔离构建 | 难直接链接 target | superbuild |

### 5.1 FetchContent 示例

FetchContent 在 CMake 配置阶段（`cmake -S . -B build`）下载并配置外部项目，使其 target 直接可用于 `target_link_libraries()`。这比手动 `git clone` + `add_subdirectory()` 更规范，也比 `ExternalProject` 更直接——因为 FetchContent 的 target 在同一个构建图中，而 `ExternalProject` 的 target 在独立的构建图中。

```cmake
include(FetchContent)

FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG        v1.5.5
)

FetchContent_MakeAvailable(nanoflann)
target_link_libraries(my_slam_core PRIVATE nanoflann::nanoflann)
```

FetchContent 的边界是：它适合相对小、构建简单、版本需要锁定的依赖。不要随意用它拉取巨大依赖，否则用户每次配置项目都会付出很大代价。

### 5.2 Superbuild 的适用场景

当项目需要从零构建一整套依赖——例如特定版本的 Ceres、g2o、Sophus、SuiteSparse 和 Eigen——可以使用 superbuild 模式。它把顶层 CMake 项目变成”构建其他项目的项目”：先用 `ExternalProject_Add` 下载和构建每个依赖，再把真正的项目作为最后一个 `ExternalProject` 构建。

superbuild 的价值在于**可复现性**。论文的实验结果能否被审稿人或其他研究者复现，很大程度上取决于依赖环境能否被精确重建。如果你的 SLAM 系统只在”Ceres 2.1 + Sophus commit abc1234 + Eigen 3.4.0”这个特定组合下工作，superbuild 可以把这些版本锁定在一个 CMake 文件中，任何人只需要 `cmake --build .` 就能从零构建完整环境。

但 superbuild 有明显的代价：首次构建可能需要半小时以上（下载并编译所有依赖），配置文件的维护负担也不小（每个依赖的 CMake 选项、补丁、安装路径都要管理）。对于初学者项目或依赖变化频繁的开发期，superbuild 的成本远超收益。更适合的场景是：发布论文代码的可复现 Docker 镜像、CI 中构建特定版本的测试环境、或在没有包管理器的嵌入式平台上部署。

---

## 6. ROS2 与 ament_cmake ⭐⭐

> **这一节解决什么问题**：ROS2 的构建系统 `ament_cmake` 在 CMake 之上添加了一层。理解它和原生 CMake 的关系，才能正确设计 ROS2 项目的构建结构。

### ament_cmake 不是另一个构建系统

初学者常常以为 `ament_cmake` 是 ROS2 独有的构建工具，和 CMake 是两个不同的东西。实际上 `ament_cmake` 是 CMake 的**扩展层**——它提供了一组 CMake 宏和函数（如 `ament_target_dependencies()`、`ament_package()`），简化了 ROS2 包的依赖声明和安装导出。底层仍然是标准 CMake。

理解这一点很重要：你在 `ament_cmake` 中仍然可以（也应该）使用标准的 `target_link_libraries()`、`target_include_directories()` 等命令。`ament_target_dependencies()` 只是对 `find_package()` + `target_link_libraries()` 的语法糖——对于 ROS2 包之间的依赖它很方便，但对于非 ROS2 依赖（如你自己的核心算法库），直接用标准 CMake 命令更清晰。

ROS2 包通常使用 `ament_cmake`：

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_slam_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_slam REQUIRED)

add_executable(slam_node src/slam_node.cpp)

ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
)

target_link_libraries(slam_node
  my_slam::my_slam_core
)

install(TARGETS slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

这里最重要的是依赖方向：**ROS2 wrapper 依赖核心库，核心库不反向依赖 ROS2。** 注意 `my_slam::my_slam_core` 通过标准 CMake 的 `target_link_libraries` 链接，而不是通过 `ament_target_dependencies`——这说明核心库是一个普通的 CMake 包，不是 ROS2 包。这种设计保证了核心库可以在任何 CMake 环境中独立构建和测试。

如果核心库包含 `rclcpp` 头文件，它就不再是纯算法库，后续 Python 绑定、离线测试、非 ROS 部署都会受影响。把 `rclcpp` 的依赖限制在 wrapper 内部，是保持核心库可复用性的关键架构决策。

---

## 7. CUDA 与混合语言构建 ⭐⭐

> **这一节解决什么问题**：机器人感知系统越来越多地使用 GPU 加速——体素滤波、最近邻搜索、深度学习推理都可以受益于 CUDA。但 CUDA 代码引入了混合语言构建的复杂性。本节展示如何用现代 CMake 管理 CUDA target，使得没有 GPU 的开发者不受影响。

### CUDA 构建的核心挑战：`.cu` 文件需要专门的编译器

CUDA 代码（`.cu` 文件）不是标准 C++——它包含 `__global__`、`__device__` 等 NVIDIA 扩展关键字，必须用 `nvcc` 编译器处理。这意味着引入 CUDA 后，项目的构建工具链从"只需要 GCC/Clang"变成"还需要 NVIDIA CUDA Toolkit"。如果核心算法库的 CMakeLists 中写了 `enable_language(CUDA)`，那么所有构建这个库的人都必须安装 CUDA——即使他们只想在 CPU 上运行。

正确的做法是把 CUDA 代码放在可选的独立 target 中，通过 `option()` 控制是否构建。这样 CPU 开发者完全不受影响，GPU 开发者通过 `-DMINI_SLAM_ENABLE_CUDA=ON` 启用加速后端。

机器人感知和学习部署常需要 CUDA。现代 CMake 可以直接把 CUDA 作为语言：

```cmake
project(my_gpu_slam LANGUAGES CXX CUDA)

add_library(gpu_kernels
  src/voxel_filter.cu
  src/nearest_neighbor.cu
)

set_target_properties(gpu_kernels PROPERTIES
  CUDA_ARCHITECTURES "75;86;89"
  POSITION_INDEPENDENT_CODE ON
)
```

`CUDA_ARCHITECTURES` 很重要。它决定生成哪些 GPU 架构的代码。设置过少，某些机器不能运行；设置过多，编译时间明显增加。

| 架构 | 常见设备 |
|------|----------|
| 72 | Jetson AGX Xavier |
| 87 | Jetson Orin |
| 86 | RTX 30 系列 |
| 89 | RTX 40 系列 |

实际项目中应把架构做成缓存变量，让用户在配置时指定目标硬件：

```cmake
# 让用户在 cmake -DCMAKE_CUDA_ARCHITECTURES=87 时指定目标架构
set(CMAKE_CUDA_ARCHITECTURES "87" CACHE STRING "CUDA architectures")
```

这样部署到 Jetson 和桌面显卡时可以分别配置。开发者在 Jetson Orin 上编译时用 `-DCMAKE_CUDA_ARCHITECTURES=87`，在 RTX 4090 桌面上用 `-DCMAKE_CUDA_ARCHITECTURES=89`。CI 中通常只编译一两种架构来控制构建时间。

如果不指定架构，CMake 会让 nvcc 自行选择——通常生成与当前 GPU 匹配的代码。这在开发机上没问题，但编译出的二进制在其他 GPU 上可能无法运行。对于需要分发的项目，显式指定架构是必需的。

---

## 8. 测试、Sanitizer 与编译缓存 ⭐⭐

> **这一节解决什么问题**：构建系统的职责不止于"编译出可执行文件"。一个成熟的构建配置还应该支持测试运行、内存错误检测和编译加速。这些工具在机器人项目中尤其重要——SLAM 和控制系统中的内存越界、数据竞争和未定义行为往往在部署后才以偶发崩溃的形式暴露。

### 为什么机器人项目特别需要 Sanitizer

C++ 的一大陷阱是：代码"能跑"不等于"正确"。一个越界访问可能只是读到了相邻变量的值，程序继续运行并给出"看起来差不多"的结果。一个数据竞争可能在 99% 的时候因为时序巧合而不触发，只在高负载或特定硬件上才表现为偶发崩溃。这类 bug 在单元测试中很难稳定复现，但在真实机器人运行中可能导致灾难性后果——SLAM 地图突然漂移、控制器输出 NaN、IPC 通道数据损坏。

AddressSanitizer (ASan) 和 ThreadSanitizer (TSan) 通过编译器插桩，在运行时检测这类隐蔽错误。ASan 在每次内存访问时检查边界，几乎能 100% 捕获越界、使用已释放内存和内存泄漏。TSan 跟踪所有内存访问的线程归属，能检测到即使在测试中没有触发的潜在数据竞争。它们的代价是运行速度降低 2-10 倍——这在开发和 CI 中完全可以接受。

构建系统不只负责生成可执行文件，还应支持质量工具。

### 8.1 GoogleTest 集成

```cmake
include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(test_tracker tests/test_tracker.cpp)
  target_link_libraries(test_tracker PRIVATE my_slam_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(test_tracker)
endif()
```

测试 target 应独立于生产 executable。不要把测试代码塞进主程序，也不要为了测试暴露不必要的全局状态。

### 8.2 Sanitizer 选项

Sanitizer 通过编译器插桩在运行时检测内存错误和并发问题。它们需要同时设置编译选项和链接选项——只设一半会导致链接失败或功能不完整。下面的写法把 ASan 作为可选的构建配置项，默认关闭以不影响正常开发。

```cmake
option(MY_SLAM_ENABLE_ASAN "Enable AddressSanitizer" OFF)

if(MY_SLAM_ENABLE_ASAN)
  target_compile_options(my_slam_core PRIVATE -fsanitize=address -fno-omit-frame-pointer)
  target_link_options(my_slam_core PRIVATE -fsanitize=address)
endif()
```

ASan 适合查越界、UAF、内存泄漏；TSan 适合查数据竞争，但会显著降低运行速度。多线程 SLAM 调试时，TSan 很有价值，但不应在性能测试时开启。

### 8.3 ccache：编译缓存加速迭代

机器人 C++ 项目的编译时间通常在 1-10 分钟级别——模板密集的 Eigen 代码、大量的 PCL 头文件和多 target 的构建配置都是编译时间的杀手。ccache 通过缓存编译结果，让"只改了一个 `.cpp` 但整个项目重新编译"的场景从几分钟缩短到几秒。它对增量构建尤其有效，因为大部分翻译单元在两次编译之间没有变化。

```cmake
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()
```

ccache 对模板多、头文件重的机器人项目非常有用，尤其是反复修改小文件时。安装方式通常是 `sudo apt install ccache`，无需其他配置即可生效。

---

## 9. 故障排查表 ⭐

> 构建系统的故障往往不是"完全编不过"，而是"在我的电脑上能编，换一台就不行"。下面的排查表覆盖了机器人 CMake 项目中最常见的六类隐蔽故障。遇到构建问题时，先在这张表中定位症状，再按检查方法逐步排查。

| 症状 | 可能原因 | 检查方法 |
|------|----------|----------|
| 下游找不到头文件 | include 没有 PUBLIC 传播 | `cmake --build` 下游最小例子 |
| 链接阶段找不到符号 | 依赖未链接或顺序错误 | `target_link_libraries` 和 imported target |
| 本机能编，别人不能 | 硬编码路径 | 搜索 `/usr/local` 和绝对路径 |
| Debug/Release 行为不同 | 链接不同依赖版本 | 打印 `CMAKE_PREFIX_PATH` |
| ROS2 wrapper 污染核心库 | 核心头文件包含 `rclcpp` | 检查 include 目录 |
| CUDA 编译很慢 | 架构列表过多 | 调整 `CMAKE_CUDA_ARCHITECTURES` |
| 测试无法单独构建 | target 边界不清 | 拆分库、工具、测试 target |

---

## 10. 累积项目

> **累积项目目标**：把前几章编写的 SLAM 算法骨架从"源码目录内能编译"提升到"可安装、可复用、可测试"的工程状态。这不是增加新功能，而是给已有功能补上工程基础设施——就像给一辆能跑的原型车加上方向盘、安全带和仪表盘。

把前几章的 SLAM 骨架整理成可安装 C++ 库：

1. `my_slam_core` 是纯 C++ target。
2. 公共头文件只放在 `include/my_slam/`。
3. Eigen 作为 PUBLIC 依赖传播。
4. Ceres 如果只在 `.cpp` 中使用，则作为 PRIVATE 依赖。
5. 提供 `install()` 和 `my_slamConfig.cmake`。
6. 单元测试通过 `BUILD_TESTING` 控制。
7. ROS2 wrapper 放在独立 package，依赖已安装的 `my_slam_core`。

---

## 11. 自测题 ⭐

> 以下问题检验你对本章前半部分的理解。如果答不出 2 题以上，建议重新阅读对应小节。这些问题覆盖了 CMake 工程化的核心判断——不是"怎么写命令"，而是"为什么这样写"。

1. 一个库的公共头文件返回 `Eigen::Vector3d`，但 CMake 中把 `Eigen3::Eigen` 写成 `PRIVATE`。下游会遇到什么问题？如何修复？
2. 为什么 ROS2 wrapper 不应该反向进入核心算法库？请从测试、Python 绑定和部署三个角度回答。
3. FetchContent 和 ExternalProject 的关键区别是什么？为什么 FetchContent 可以直接链接 target，而 ExternalProject 通常不行？
4. 设计一个支持 CUDA 和非 CUDA 后端的 CMake 选项，并说明如何让下游代码不依赖 CUDA 头文件。

---

## 12. 阶段小结：从能编译到能交付 ⭐⭐

CMake 工程化的核心是把项目关系表达清楚。从前面的内容中，可以提炼出一条贯穿全章的主线：**现代 CMake 不是构建脚本，而是项目关系的声明式描述。** 每一个 `target_*` 命令都在回答一个关系问题——“谁依赖谁？””这个依赖是公共的还是私有的？””安装后下游怎么找到我？”

target-based CMake 让 include、编译特性、依赖和宏定义都有明确归属；`PUBLIC/PRIVATE/INTERFACE` 让依赖传播可控；安装导出让库可以被下游稳定使用。这三层机制解决的是同一个问题的三个维度：target-based 解决”属性归属”，关键字解决”传播规则”，安装导出解决”跨项目复用”。

对机器人项目而言，构建系统还承担架构约束的角色。核心算法库是否包含 ROS2 头文件、CUDA 后端是否可选、测试是否独立于生产代码——这些看似构建配置的问题，实际上在定义项目的架构边界。一个把 `rclcpp` 写进核心库 `PUBLIC` 依赖的项目，从构建系统的层面就已经放弃了”脱离 ROS2 运行”的可能性。

核心算法库应保持纯净，ROS2 wrapper、Python 绑定、CUDA 内核、测试和工具都应有清晰 target 边界。这样的项目更容易测试、迁移、部署，也更不容易在依赖升级时崩溃。

下面把这些原则展开成完整工程链路。前面的内容回答了”现代 CMake 应该怎么写”，后面的内容回答”为什么这样写才能支撑一个可以安装、复用、测试、加速和持续集成的机器人项目”。

---

## 13. PUBLIC / PRIVATE / INTERFACE 的依赖传播模型 ⭐⭐⭐

> **这一节解决什么问题**：`PUBLIC`、`PRIVATE`、`INTERFACE` 不只是三个关键字，而是在描述“当前库怎么构建”和“别人怎么使用当前库”这两组关系。理解它们，是写出可复用 C++ 库的分水岭。

### 13.1 两张清单：构建需求与使用需求

CMake target 可以被理解成一个带说明书的工程产品。工厂内部需要的夹具、工装、调试脚本，不应该写进用户说明书；用户安装后必须知道的接口尺寸、电源规格、通信协议，必须写进说明书。CMake 中也是同一个道理：

| 需求类型 | 面向谁 | CMake 属性 | 对应关键字 |
|----------|--------|------------|------------|
| 构建需求 | 当前 target 自己 | `INCLUDE_DIRECTORIES`、`LINK_LIBRARIES` | `PRIVATE` 或 `PUBLIC` |
| 使用需求 | 链接当前 target 的下游 | `INTERFACE_INCLUDE_DIRECTORIES`、`INTERFACE_LINK_LIBRARIES` | `PUBLIC` 或 `INTERFACE` |

换句话说：

```text
PRIVATE   = 我自己需要，下游不需要知道
PUBLIC    = 我自己需要，下游也需要知道
INTERFACE = 我自己不编译，只把规则传给下游
```

这三个关键字的本质是“依赖是否出现在公共接口里”。公共接口不只包括 `.hpp` 中显式出现的类型，也包括会影响 ABI、模板实例化、编译宏和编译特性的内容。

> **本质洞察**：`PUBLIC`、`PRIVATE`、`INTERFACE` 的本质不是“链接时要不要用”，而是“这个依赖是否成为 target 使用契约的一部分”。公共契约越准确，下游项目越少猜测，安装导出越稳定。

### 13.2 案例一：Eigen 出现在头文件返回值中

先看一个最典型的 SLAM 核心库头文件：

```cpp
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mini_slam {

class PoseEstimator {
public:
  // Eigen 类型直接出现在公共函数签名中
  Eigen::Isometry3d estimatePose(const Eigen::MatrixXd& points) const;
};

}  // namespace mini_slam
```

这个头文件让 Eigen 成为公共接口的一部分。下游只要写：

```cpp
#include <mini_slam/pose_estimator.hpp>
```

编译器就必须能找到 `<Eigen/Core>` 和 `<Eigen/Geometry>`。因此 Eigen 必须是 `PUBLIC`：

```cmake
find_package(Eigen3 REQUIRED)

add_library(mini_slam_core src/pose_estimator.cpp)

target_include_directories(mini_slam_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mini_slam_core
  PUBLIC
    Eigen3::Eigen
)
```

如果误写成 `PRIVATE`，当前库仍然能编译，因为 `src/pose_estimator.cpp` 能看到 Eigen；但下游项目在包含头文件时会失败。这类错误像“产品在工厂能跑，到了客户现场缺零件”：在当前仓库内看不出问题，安装后才暴露。

### 13.3 案例二：Ceres 只在 `.cpp` 中使用

再看后端优化器。公共头文件只暴露自己的结构体：

```cpp
#pragma once

#include <Eigen/Core>
#include <vector>

namespace mini_slam {

struct PoseGraphEdge {
  int from = 0;
  int to = 0;
  Eigen::Matrix4d measurement = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
};

class PoseGraphOptimizer {
public:
  // 公共接口不出现 ceres::Problem、ceres::CostFunction 等类型
  std::vector<Eigen::Matrix4d> optimize(
      const std::vector<Eigen::Matrix4d>& initial_poses,
      const std::vector<PoseGraphEdge>& edges) const;
};

}  // namespace mini_slam
```

实现文件内部使用 Ceres：

```cpp
#include <mini_slam/pose_graph_optimizer.hpp>

#include <ceres/ceres.h>

namespace mini_slam {

std::vector<Eigen::Matrix4d> PoseGraphOptimizer::optimize(
    const std::vector<Eigen::Matrix4d>& initial_poses,
    const std::vector<PoseGraphEdge>& edges) const {
  ceres::Problem problem;
  // 这里只展示边界关系：Ceres 是实现细节，不进入头文件契约
  (void)initial_poses;
  (void)edges;
  (void)problem;
  return {};
}

}  // namespace mini_slam
```

此时 Ceres 应该是 `PRIVATE`：

```cmake
find_package(Ceres REQUIRED)

target_link_libraries(mini_slam_core
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    Ceres::ceres
)
```

这样做的好处有三点：

| 好处 | 解释 |
|------|------|
| 下游编译更轻 | 下游不需要解析 Ceres 的大量头文件 |
| ABI 边界更干净 | Ceres 类型不进入库的二进制接口 |
| 替换实现更容易 | 将来把 Ceres 换成 g2o 或 GTSAM，不会影响调用方代码 |

反事实地看，如果把 Ceres 类型暴露到公共头文件里，下游就会被迫安装和链接 Ceres。即使下游只是想调用一个读取地图的工具，也会被优化器依赖拖住。长期看，这会把一个“可复用核心库”变成“只能在原始开发环境中使用的代码集合”。

### 13.4 案例三：PCL 类型是否应该出现在核心 API 中

点云项目常见两种 API 设计：

| 设计 | 公共接口 | PCL 依赖关键字 | 取舍 |
|------|----------|----------------|------|
| PCL 原生接口 | `pcl::PointCloud<pcl::PointXYZI>` | `PUBLIC` | 使用方便，但核心库绑定 PCL |
| 轻量视图接口 | 自定义 `PointCloudView` | `PRIVATE` | 边界干净，但需要转换层 |

PCL 原生接口写法：

```cpp
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mini_slam {

class VoxelFilter {
public:
  pcl::PointCloud<pcl::PointXYZI> filter(
      const pcl::PointCloud<pcl::PointXYZI>& input) const;
};

}  // namespace mini_slam
```

对应 CMake：

```cmake
find_package(PCL REQUIRED COMPONENTS common filters)

target_link_libraries(mini_slam_core
  PUBLIC
    ${PCL_LIBRARIES}
)

target_include_directories(mini_slam_core
  PUBLIC
    ${PCL_INCLUDE_DIRS}
)
```

这不是错，但它意味着核心库使用者必须接受 PCL 的头文件、编译定义和 ABI。对一个只运行离线数据集的 SLAM 程序，这可能完全可以接受；对一个希望同时提供 Python 绑定、CUDA 后端和 ROS2 wrapper 的库，就会变得沉重。

更干净的做法是让核心 API 使用轻量视图：

```cpp
#pragma once

#include <cstddef>
#include <span>

namespace mini_slam {

struct PointXYZI {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

struct PointCloudView {
  std::span<const PointXYZI> points;
};

class VoxelFilter {
public:
  // 核心库只理解自己的点格式，不直接依赖 PCL 或 ROS2 消息
  std::vector<PointXYZI> filter(PointCloudView input) const;
};

}  // namespace mini_slam
```

然后把 PCL 转换放在适配层：

```cpp
#include <mini_slam/point_cloud.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mini_slam::pcl_adapter {

std::vector<PointXYZI> fromPcl(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
  std::vector<PointXYZI> output;
  output.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    output.push_back(PointXYZI{p.x, p.y, p.z, p.intensity});
  }
  return output;
}

}  // namespace mini_slam::pcl_adapter
```

对应 CMake 可以把 PCL 放进单独 target：

```cmake
add_library(mini_slam_pcl_adapter src/pcl_adapter.cpp)

target_link_libraries(mini_slam_pcl_adapter
  PUBLIC
    mini_slam_core
  PRIVATE
    ${PCL_LIBRARIES}
)

target_include_directories(mini_slam_pcl_adapter
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)
```

这段设计体现了一个工程判断：核心库不是不能使用 PCL，而是要有意识地区分“算法本身需要什么”和“某个生态接入层需要什么”。

### 13.5 INTERFACE target：只传播规则，不生成二进制

`INTERFACE` target 常用于三类场景：

| 场景 | 示例 | 为什么不用普通库 |
|------|------|------------------|
| header-only 库 | `Eigen3::Eigen`、`nanoflann::nanoflann` | 没有 `.cpp` 需要编译 |
| 项目通用警告规则 | `mini_slam_warnings` | 只传播编译选项 |
| 可选特性开关 | `mini_slam_options` | 只传播宏和 feature |

项目内可以定义一个通用警告 target：

```cmake
add_library(mini_slam_warnings INTERFACE)

target_compile_options(mini_slam_warnings
  INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_link_libraries(mini_slam_core
  PRIVATE
    mini_slam_warnings
)
```

这里把 `mini_slam_warnings` 作为 `PRIVATE` 链接到核心库，表示警告规则只约束当前库的构建，不强迫下游也使用同样的警告级别。否则下游项目可能因为你的警告规则而编译失败，这不是一个友好的安装库行为。

相反，C++ 标准经常应该是 `PUBLIC`：

```cmake
target_compile_features(mini_slam_core PUBLIC cxx_std_20)
```

如果公共头文件使用了 `std::span`，下游必须启用 C++20 才能编译这个头文件。把 `cxx_std_20` 写成 `PRIVATE` 会导致库自己能编译，下游包含头文件时失败。

### 13.6 传播关系的调试方法

CMake 出错时，不要只盯着 `CMakeLists.txt` 读。更可靠的方法是观察 target 的实际属性。

```cmake
get_target_property(core_includes mini_slam_core INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(core_links mini_slam_core INTERFACE_LINK_LIBRARIES)

message(STATUS "mini_slam_core public includes: ${core_includes}")
message(STATUS "mini_slam_core public links: ${core_links}")
```

如果安装后下游找不到头文件，优先检查三件事：

1. `target_include_directories()` 是否有 `$<INSTALL_INTERFACE:include>`。
2. `install(DIRECTORY include/ DESTINATION include)` 是否真的安装了头文件。
3. `install(EXPORT ...)` 导出的 target 是否包含正确的 `INTERFACE_INCLUDE_DIRECTORIES`。

### 13.7 常见陷阱

| 类型 | 错误做法 | 现象 | 正确做法 |
|------|----------|------|----------|
| 编程陷阱 | 头文件暴露 Eigen，CMake 写 `PRIVATE Eigen3::Eigen` | 下游包含头文件失败 | 改成 `PUBLIC` |
| 概念误区 | 认为 `PUBLIC` 表示“更强链接” | 所有依赖都写成 `PUBLIC` | 根据公共接口决定传播 |
| 思维陷阱 | 为了省事把所有 include 目录全局添加 | 测试和工具被隐式污染 | 所有属性绑定到 target |
| 工程陷阱 | 把警告规则作为 `PUBLIC` 传播 | 下游因为警告策略不同失败 | 警告 target 通常 `PRIVATE` |

### 13.8 练习

1. 一个公共头文件中只有 `std::vector<double>` 和 `Eigen::Vector3d`，实现文件中使用 Ceres、glog 和 OpenMP。请分别判断 Eigen、Ceres、glog、OpenMP 应该是 `PUBLIC` 还是 `PRIVATE`，并说明理由。
2. 给一个 header-only 的 `math_utils` 库写 `INTERFACE` target，要求它传播 C++20、Eigen 和一个 `MATH_UTILS_USE_FAST_PATH` 宏。
3. 设计一个 `mini_slam_warnings` target，使库本身启用严格警告，但安装后的下游不被迫继承这些警告。

---

## 14. 安装导出完整链路 ⭐⭐⭐

> **这一节解决什么问题**：让一个库从“源码目录里能用”变成“安装后能被 `find_package()` 稳定找到”。这一步是工程化 CMake 的核心闭环。

### 14.1 安装导出要解决的三件事

一个可安装 C++ 库至少要回答三件事：

| 问题 | 由谁回答 | 文件位置 |
|------|----------|----------|
| 头文件装到哪里 | `install(DIRECTORY ...)` 或 `FILE_SET` | `${prefix}/include` |
| 二进制库装到哪里 | `install(TARGETS ...)` | `${prefix}/lib` |
| 下游如何找到 target | `Config.cmake` + `Targets.cmake` | `${prefix}/lib/cmake/<pkg>` |

如果只安装 `.so` 和头文件，下游仍然要手写 include 路径和库路径；如果安装了 `Targets.cmake` 但没有 `Config.cmake`，`find_package()` 仍然找不到入口。完整链路必须四个环节都闭合：

```text
定义 target
  ↓
安装 target 与头文件
  ↓
导出 Targets.cmake
  ↓
生成 Config.cmake 与 ConfigVersion.cmake
  ↓
下游 find_package() + target_link_libraries()
```

### 14.2 推荐目录结构

```text
mini_slam/
  CMakeLists.txt
  cmake/
    mini_slamConfig.cmake.in
  include/
    mini_slam/
      pose_estimator.hpp
      point_cloud.hpp
  src/
    pose_estimator.cpp
    point_cloud.cpp
  tests/
    test_pose_estimator.cpp
```

这里 `cmake/mini_slamConfig.cmake.in` 是安装包入口模板。它不是构建脚本，而是下游执行 `find_package(mini_slam)` 时被读取的配置文件。

### 14.3 完整 CMakeLists 写法

```cmake
cmake_minimum_required(VERSION 3.22)
project(mini_slam VERSION 0.3.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(MINI_SLAM_BUILD_TESTS "Build mini_slam tests" ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_library(mini_slam_core
  src/pose_estimator.cpp
  src/point_cloud.cpp
)

add_library(mini_slam::mini_slam_core ALIAS mini_slam_core)

target_compile_features(mini_slam_core PUBLIC cxx_std_20)

target_include_directories(mini_slam_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(mini_slam_core
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    Ceres::ceres
)

install(TARGETS mini_slam_core
  EXPORT mini_slamTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mini_slamConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mini_slamConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mini_slamConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mini_slam
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/mini_slamConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/mini_slamConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mini_slam
)

install(EXPORT mini_slamTargets
  FILE mini_slamTargets.cmake
  NAMESPACE mini_slam::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mini_slam
)
```

注意两处细节：

| 写法 | 原因 |
|------|------|
| `GNUInstallDirs` | 避免硬编码 `lib`，兼容 `lib64` 等平台习惯 |
| `mini_slam::mini_slam_core ALIAS` | 让源码内和安装后使用同一种命名风格 |

### 14.4 Config.cmake.in 的依赖查找

`mini_slamConfig.cmake.in` 应该写：

```cmake
@PACKAGE_INIT@

include(CMakeFindDependencyMacro)

# Eigen 出现在 mini_slam_core 的公共接口中，下游必须先找到它
find_dependency(Eigen3)

# Ceres 是 PRIVATE 依赖，不需要在这里查找
# 如果公共头文件暴露 ceres 类型，才应改为 find_dependency(Ceres)

include("${CMAKE_CURRENT_LIST_DIR}/mini_slamTargets.cmake")
```

`find_dependency()` 不只是调用 `find_package()`。它还会把 `REQUIRED`、`QUIET` 等语义从上层 `find_package(mini_slam REQUIRED)` 传播下来，让错误信息更符合用户预期。

### 14.5 BUILD_INTERFACE 与 INSTALL_INTERFACE

这两个 generator expression 经常让初学者困惑。它们的作用是让同一个 target 在两个世界中使用不同路径：

| 表达式 | 使用时机 | 典型值 |
|--------|----------|--------|
| `$<BUILD_INTERFACE:...>` | 在源码构建树中使用 target | `/home/me/mini_slam/include` |
| `$<INSTALL_INTERFACE:...>` | 安装后被下游使用 target | `include` |

为什么安装接口不能写绝对路径？因为安装包应当可搬迁。假设在开发机安装到 `/tmp/install`，打包后放到机器人 `/opt/mini_slam`，如果导出的 target 中仍然记录 `/tmp/install/include`，机器人上就会找不到头文件。

> **本质洞察**：安装导出的目标不是“把当前机器上的路径记下来”，而是“把相对于安装前缀的使用契约保存下来”。可搬迁性是 CMake package 与手写路径的根本区别。

### 14.6 下游最小验证项目

安装导出是否正确，不能只在原项目里看。必须用一个全新的下游项目验证：

```text
consumer/
  CMakeLists.txt
  main.cpp
```

```cmake
cmake_minimum_required(VERSION 3.22)
project(consumer LANGUAGES CXX)

find_package(mini_slam REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE mini_slam::mini_slam_core)
```

```cpp
#include <mini_slam/pose_estimator.hpp>

int main() {
  mini_slam::PoseEstimator estimator;
  (void)estimator;
  return 0;
}
```

验证命令：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/tmp/mini_slam_install
cmake --build build
```

这个最小项目是最有价值的构建测试之一。它能暴露源码树中看不到的问题：头文件是否安装、命名空间是否正确、公共依赖是否写入 Config、安装路径是否可搬迁。

### 14.7 安装导出的常见陷阱

| 类型 | 错误做法 | 现象 | 正确做法 |
|------|----------|------|----------|
| 编程陷阱 | `INSTALL_INTERFACE` 写绝对路径 | 安装包换机器后失效 | 写相对路径 `${CMAKE_INSTALL_INCLUDEDIR}` |
| 概念误区 | 只安装库文件，不导出 target | 下游仍要手写路径 | 同时安装 `Targets.cmake` 和 `Config.cmake` |
| 思维陷阱 | 只在源码仓库内测试 | 安装后才发现缺依赖 | 用独立 consumer 项目验证 |
| 工程陷阱 | Config 中漏写公共依赖 | 下游找不到 Eigen、PCL 等头文件 | 对所有公共依赖写 `find_dependency()` |

### 14.8 练习

1. 把前文的 `mini_slam_core` 安装到 `/tmp/mini_slam_install`，再写一个独立 `consumer` 项目验证 `find_package(mini_slam REQUIRED)`。
2. 故意把 `Eigen3::Eigen` 改成 `PRIVATE`，观察 consumer 项目的报错，并解释为什么原项目仍可能编译成功。
3. 将项目版本从 `0.3.0` 改成 `1.0.0`，把 `COMPATIBILITY` 分别改为 `SameMajorVersion` 和 `ExactVersion`，观察 `find_package(mini_slam 0.3 REQUIRED)` 的行为差异。

---

## 15. ROS2 wrapper 与核心库边界 ⭐⭐⭐

> **这一节解决什么问题**：机器人项目经常既要支持 ROS2 在线运行，又要支持离线测试、Python 绑定和嵌入式部署。核心库与 ROS2 wrapper 的边界决定了这个项目能走多远。

### 15.1 依赖方向只能单向

正确依赖方向：

```text
mini_slam_core
  ↑
mini_slam_ros
```

含义是：ROS2 wrapper 调用核心库；核心库不知道 ROS2 的存在。

错误依赖方向：

```text
mini_slam_core → rclcpp → sensor_msgs → rosidl_runtime_cpp
```

一旦核心库包含 `rclcpp` 或 `sensor_msgs`，它就不再是纯 C++ 算法库。后果会连续出现：

| 场景 | 被污染后的后果 |
|------|----------------|
| 离线单元测试 | 测试需要 source ROS2 环境 |
| Python 绑定 | 绑定层被 ROS2 消息类型拖住 |
| 非 ROS 部署 | 机器人外的数据处理工具无法轻量使用 |
| 安装导出 | `Config.cmake` 必须处理 ROS2 依赖 |
| 交叉编译 | 嵌入式目标也要带整套 ROS2 运行时 |

这就像控制系统中的分层：MPC 输出期望力，WBC 转成关节力矩，电机驱动负责硬件协议。把硬件协议写进 MPC，会让规划器难以仿真和测试；把 ROS2 写进核心算法库，也是同样的问题。

### 15.2 核心库的数据结构应保持生态无关

核心库可以定义自己的轻量数据结构：

```cpp
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace mini_slam {

struct TimedPoint {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  float intensity = 0.0F;
  double relative_time = 0.0;
};

struct LidarFrame {
  std::int64_t stamp_ns = 0;
  std::string frame_id;
  std::vector<TimedPoint> points;
};

struct OdometryEstimate {
  std::int64_t stamp_ns = 0;
  Eigen::Isometry3d map_from_lidar = Eigen::Isometry3d::Identity();
};

class LidarOdometry {
public:
  OdometryEstimate addFrame(const LidarFrame& frame);
};

}  // namespace mini_slam
```

ROS2 wrapper 负责消息转换：

```cpp
#include <mini_slam/lidar_odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mini_slam_ros {

mini_slam::LidarFrame fromRosPointCloud(
    const sensor_msgs::msg::PointCloud2& msg) {
  mini_slam::LidarFrame frame;
  frame.stamp_ns =
      static_cast<std::int64_t>(msg.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg.header.stamp.nanosec);
  frame.frame_id = msg.header.frame_id;

  // 这里应使用 sensor_msgs::PointCloud2Iterator 解析字段
  // 示例只强调边界：ROS2 消息在 wrapper 内被转换成核心库结构
  frame.points.reserve(msg.width * msg.height);
  return frame;
}

}  // namespace mini_slam_ros
```

这段边界设计让核心库可以在四个环境中复用：

| 环境 | 输入来源 | 是否需要 ROS2 |
|------|----------|---------------|
| 单元测试 | 手写 `LidarFrame` | 否 |
| 离线评测 | 数据集解析器 | 否 |
| ROS2 在线节点 | `sensor_msgs::msg::PointCloud2` | 是，仅 wrapper 需要 |
| Python 工具 | pybind11 绑定核心结构 | 否 |

### 15.3 ROS2 wrapper 的 CMakeLists

ROS2 wrapper 可以是单独 package：

```cmake
cmake_minimum_required(VERSION 3.22)
project(mini_slam_ros LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mini_slam REQUIRED)

add_executable(lidar_odometry_node src/lidar_odometry_node.cpp)

target_compile_features(lidar_odometry_node PUBLIC cxx_std_20)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
)

target_link_libraries(lidar_odometry_node
  PRIVATE
    mini_slam::mini_slam_core
)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

`package.xml` 中也只声明 wrapper 所需依赖：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>mini_slam_ros</name>
  <version>0.3.0</version>
  <description>ROS2 wrapper for mini_slam.</description>
  <maintainer email="dev@example.com">mini_slam maintainers</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>

  <exec_depend>mini_slam</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

注意 `mini_slam` 是 wrapper 的依赖，而不是反过来。核心库如果使用普通 CMake 安装，可以通过 `CMAKE_PREFIX_PATH` 被 colcon workspace 找到。

### 15.4 wrapper 里的节点只做四件事

一个健康的 ROS2 wrapper 节点职责很窄：

1. 读取参数。
2. 订阅和发布消息。
3. 在 ROS2 消息与核心库结构之间转换。
4. 管理生命周期、诊断和 TF。

算法状态应在核心库对象中：

```cpp
#include <mini_slam/lidar_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarOdometryNode final : public rclcpp::Node {
public:
  LidarOdometryNode() : Node("lidar_odometry_node") {
    // wrapper 读取 ROS2 参数，并传给核心库配置
    declare_parameter<double>("voxel_size", 0.2);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          handleCloud(*msg);
        });
  }

private:
  void handleCloud(const sensor_msgs::msg::PointCloud2& msg) {
    auto frame = convertPointCloud(msg);
    const auto odom = odometry_.addFrame(frame);
    (void)odom;
    // 发布 nav_msgs::msg::Odometry 和 TF 的代码属于 wrapper
  }

  mini_slam::LidarFrame convertPointCloud(
      const sensor_msgs::msg::PointCloud2& msg) {
    mini_slam::LidarFrame frame;
    frame.frame_id = msg.header.frame_id;
    return frame;
  }

  mini_slam::LidarOdometry odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
```

如果节点回调中出现大量 ICP、优化、地图更新逻辑，说明算法正在流入 wrapper。正确做法是把这些逻辑移回核心库，并为核心库补单元测试。

### 15.5 常见陷阱

| 类型 | 错误做法 | 现象 | 正确做法 |
|------|----------|------|----------|
| 编程陷阱 | 核心头文件包含 `rclcpp/rclcpp.hpp` | 离线测试也要 ROS2 环境 | wrapper 做消息转换 |
| 概念误区 | 认为 ROS2 节点就是算法类 | 算法难复用、难测试 | 节点是适配层，算法在核心库 |
| 思维陷阱 | 为了少写转换代码直接传 ROS 消息 | 初期省事，后期被生态绑定 | 定义轻量核心结构 |
| 工程陷阱 | wrapper 和核心库同一个 target | 安装导出依赖混乱 | 分成 `mini_slam_core` 与 `mini_slam_ros` |

### 15.6 练习

1. 将一个接收 `sensor_msgs::msg::PointCloud2` 的 ICP 节点拆成 `IcpRegistration` 核心类和 `IcpNode` wrapper。要求核心类不包含任何 ROS2 头文件。
2. 为 `fromRosPointCloud()` 写单元测试。提示：转换函数可以放在 wrapper 内，但其输入输出应足够小，便于构造测试数据。
3. 设计一个离线评测工具 `run_kitti`，要求它链接 `mini_slam_core`，但不链接 `rclcpp`。

---

## 16. CUDA、测试、Sanitizer 与 CI 的组合 ⭐⭐⭐

> **这一节解决什么问题**：机器人项目的构建系统不应只覆盖“正常编译”。它还要让 CPU/GPU 后端可切换，让测试可分层运行，让内存错误尽早暴露，并让 CI 自动验证关键配置。

### 16.1 CUDA 后端应是可选 target

GPU 加速常用于体素滤波、最近邻搜索、投影渲染、深度图处理和神经网络推理。但核心库不能因为某个 CUDA 内核而强制所有用户安装 CUDA。推荐结构是：

```text
mini_slam_core        纯 C++ 算法接口
mini_slam_cpu         CPU 实现
mini_slam_cuda        CUDA 实现，可选构建
mini_slam_ros         ROS2 wrapper，只依赖核心接口
```

CMake 选项：

```cmake
option(MINI_SLAM_ENABLE_CUDA "Build CUDA acceleration backend" OFF)

add_library(mini_slam_cpu src/cpu/voxel_filter_cpu.cpp)
target_link_libraries(mini_slam_cpu PUBLIC mini_slam_core)

if(MINI_SLAM_ENABLE_CUDA)
  enable_language(CUDA)

  add_library(mini_slam_cuda
    src/cuda/voxel_filter_cuda.cu
    src/cuda/voxel_filter_cuda.cpp
  )

  target_link_libraries(mini_slam_cuda
    PUBLIC
      mini_slam_core
  )

  set_target_properties(mini_slam_cuda PROPERTIES
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
    POSITION_INDEPENDENT_CODE ON
  )

  target_compile_definitions(mini_slam_cuda
    PRIVATE
      MINI_SLAM_BUILDING_CUDA_BACKEND=1
  )
endif()
```

不要把 `.cu` 文件直接塞进核心库，除非项目明确规定 CUDA 是必需运行环境。否则下游即使只想使用 CPU 后端，也会被迫配置 CUDA 编译器。

### 16.2 GPU 架构选择

`CMAKE_CUDA_ARCHITECTURES` 决定生成哪些架构的 device code：

| 架构 | 常见硬件 | 工程建议 |
|------|----------|----------|
| 72 | Jetson AGX Xavier | 老一代 Jetson 部署 |
| 87 | Jetson Orin | 机器人嵌入式默认优先 |
| 86 | RTX 30 系列 | 桌面训练和开发 |
| 89 | RTX 40 系列 | 新桌面平台 |

配置建议：

```cmake
if(MINI_SLAM_ENABLE_CUDA AND NOT CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES "87" CACHE STRING "CUDA architectures" FORCE)
endif()
```

反事实地看，如果架构列表写成 `"72;75;80;86;87;89"`，兼容性更宽，但每个 `.cu` 文件都要为多个架构生成代码，编译时间明显上升。CI 中尤其要控制架构数量，否则一次提交可能把 GPU 构建拖到几十分钟。

### 16.3 Sanitizer 应封装成函数

ASan、UBSan、TSan 不应散落在每个 target 后面。更好的做法是封装成项目函数：

```cmake
option(MINI_SLAM_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(MINI_SLAM_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(MINI_SLAM_ENABLE_TSAN "Enable ThreadSanitizer" OFF)

function(mini_slam_enable_sanitizers target_name)
  if(MINI_SLAM_ENABLE_ASAN)
    target_compile_options(${target_name} PRIVATE
      -fsanitize=address
      -fno-omit-frame-pointer
    )
    target_link_options(${target_name} PRIVATE
      -fsanitize=address
    )
  endif()

  if(MINI_SLAM_ENABLE_UBSAN)
    target_compile_options(${target_name} PRIVATE
      -fsanitize=undefined
    )
    target_link_options(${target_name} PRIVATE
      -fsanitize=undefined
    )
  endif()

  if(MINI_SLAM_ENABLE_TSAN)
    target_compile_options(${target_name} PRIVATE
      -fsanitize=thread
      -fno-omit-frame-pointer
    )
    target_link_options(${target_name} PRIVATE
      -fsanitize=thread
    )
  endif()
endfunction()

mini_slam_enable_sanitizers(mini_slam_core)
```

实际工程中 ASan/UBSan 可以一起开，TSan 通常单独开。TSan 会显著降低运行速度，也容易与某些第三方库产生噪声，因此适合专门的数据竞争检查任务。

CUDA 设备侧错误需要使用 NVIDIA 工具链检查，例如 `compute-sanitizer`。主机侧 ASan 不会自动发现 device kernel 内部越界。

### 16.4 CTest 标签让测试分层

不是所有测试都应该每次提交都跑。可以用标签区分：

```cmake
include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(test_core tests/test_core.cpp)
  target_link_libraries(test_core PRIVATE mini_slam_core GTest::gtest_main)

  add_executable(test_dataset tests/test_dataset.cpp)
  target_link_libraries(test_dataset PRIVATE mini_slam_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(test_core PROPERTIES LABELS "unit")
  gtest_discover_tests(test_dataset PROPERTIES LABELS "dataset")
endif()
```

运行方式：

```bash
ctest --test-dir build --label-regex unit --output-on-failure
ctest --test-dir build --label-regex dataset --output-on-failure
```

测试分层的工程意义：

| 标签 | 运行频率 | 内容 |
|------|----------|------|
| `unit` | 每次提交 | 小输入、无外部数据 |
| `integration` | 每日或合并前 | 多模块组合 |
| `dataset` | 定期 | KITTI、EuRoC、rosbag 等数据 |
| `gpu` | GPU 环境 | CUDA kernel 与 TensorRT |

### 16.5 CMakePresets 固化常用构建配置

命令行参数越长，团队越容易出现“每个人本地配置不同”的问题。`CMakePresets.json` 可以把常用配置固化下来：

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "dev-debug",
      "generator": "Ninja",
      "binaryDir": "build/dev-debug",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "MINI_SLAM_BUILD_TESTS": "ON",
        "MINI_SLAM_ENABLE_ASAN": "ON",
        "MINI_SLAM_ENABLE_UBSAN": "ON"
      }
    },
    {
      "name": "release-cpu",
      "generator": "Ninja",
      "binaryDir": "build/release-cpu",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "MINI_SLAM_BUILD_TESTS": "ON",
        "MINI_SLAM_ENABLE_CUDA": "OFF"
      }
    },
    {
      "name": "release-cuda-orin",
      "generator": "Ninja",
      "binaryDir": "build/release-cuda-orin",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "MINI_SLAM_ENABLE_CUDA": "ON",
        "CMAKE_CUDA_ARCHITECTURES": "87"
      }
    }
  ]
}
```

开发者只需要执行：

```bash
cmake --preset dev-debug
cmake --build build/dev-debug
ctest --test-dir build/dev-debug --output-on-failure
```

### 16.6 CI 最小矩阵

CI 不需要覆盖所有可能组合，但要覆盖最容易坏的边界：

| 任务 | 构建类型 | 目的 |
|------|----------|------|
| `linux-debug-asan` | Debug + ASan + UBSan | 查内存与未定义行为 |
| `linux-release` | Release | 查优化构建与安装导出 |
| `linux-consumer` | 安装后 consumer | 查 `find_package()` |
| `linux-tsan` | Debug + TSan | 查数据竞争 |
| `cuda` | Release + CUDA | 查 GPU 后端 |

GitHub Actions 示例：

```yaml
name: ci

on:
  push:
  pull_request:

jobs:
  linux-release:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - name: Configure
        run: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
      - name: Build
        run: cmake --build build --parallel
      - name: Test
        run: ctest --test-dir build --output-on-failure
      - name: Install
        run: cmake --install build --prefix install

  linux-debug-asan:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - name: Configure
        run: >
          cmake -S . -B build
          -DCMAKE_BUILD_TYPE=Debug
          -DBUILD_TESTING=ON
          -DMINI_SLAM_ENABLE_ASAN=ON
          -DMINI_SLAM_ENABLE_UBSAN=ON
      - name: Build
        run: cmake --build build --parallel
      - name: Test
        run: ctest --test-dir build --output-on-failure
```

真正严谨的项目还会在安装后创建 consumer 项目：

```bash
cmake --install build --prefix "${PWD}/install"
cmake -S tests/consumer -B build-consumer \
  -DCMAKE_PREFIX_PATH="${PWD}/install"
cmake --build build-consumer
```

### 16.7 常见陷阱

| 类型 | 错误做法 | 现象 | 正确做法 |
|------|----------|------|----------|
| 编程陷阱 | ASan 只加 compile 选项，不加 link 选项 | 链接或运行异常 | compile 与 link 都加 |
| 概念误区 | 认为 CUDA 开启后所有测试都应跑 GPU | 普通 CI 变慢且依赖硬件 | 用 `gpu` 标签分离 |
| 思维陷阱 | 只测 Debug，不测 Release | 优化构建中的问题漏掉 | CI 至少覆盖 Debug 与 Release |
| 工程陷阱 | 只在源码树测试，不测试安装包 | 导出错误长期隐藏 | 加 consumer 构建任务 |

### 16.8 练习

1. 给 `mini_slam_core` 添加 ASan/UBSan 选项，并构造一个越界访问测试，观察 ASan 输出。
2. 写一个 `release-cuda-orin` preset，要求 CUDA 架构只包含 `87`，并说明为什么不在默认 preset 中开启 CUDA。
3. 为安装后的 consumer 项目写 CI 步骤，验证 `find_package(mini_slam REQUIRED)` 能在全新构建目录中工作。

---

## 17. 一套可交付 CMake 工程的检查表 ⭐⭐

> **这一节解决什么问题**：把本章的工程判断压缩成可执行的检查表。每次新建 C++/ROS2/CUDA 项目时，都可以按这个顺序排查构建系统是否健康。

### 17.1 target 边界检查

| 检查项 | 合格标准 |
|--------|----------|
| 核心库 | 不包含 ROS2、PCL、CUDA 等可选生态头文件，除非它们就是公共接口 |
| wrapper | 只做消息转换、参数、发布订阅、生命周期 |
| 工具程序 | 链接核心库，不反向影响核心库 |
| 测试程序 | 独立 target，不把测试代码编进生产库 |
| CUDA 后端 | 可选 target，不强制 CPU 用户安装 CUDA |

### 17.2 依赖传播检查

| 问题 | 判断方式 |
|------|----------|
| 头文件中出现的第三方类型 | 对应依赖通常是 `PUBLIC` |
| 只在 `.cpp` 中出现的第三方类型 | 对应依赖通常是 `PRIVATE` |
| header-only 工具库 | 通常用 `INTERFACE` |
| 编译标准 | 如果公共头文件需要，写 `PUBLIC` |
| 警告选项 | 通常写 `PRIVATE`，避免影响下游 |

### 17.3 安装导出检查

| 检查项 | 命令或现象 |
|--------|------------|
| 能安装 | `cmake --install build --prefix install` |
| 有 Config | `install/lib/cmake/<pkg>/<pkg>Config.cmake` |
| 有 Targets | `install/lib/cmake/<pkg>/<pkg>Targets.cmake` |
| 下游能找 | `find_package(<pkg> REQUIRED)` |
| 路径可搬迁 | 导出文件中不含源码树绝对 include 路径 |

### 17.4 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 修复方向 |
|------|----------|----------|----------|
| 下游包含头文件失败 | 公共依赖被写成 `PRIVATE` | 查看头文件中出现的第三方类型 | 改为 `PUBLIC` 并在 Config 中 `find_dependency()` |
| `find_package()` 找到包但 target 不存在 | `install(EXPORT ...)` 漏写或命名空间不一致 | 检查安装目录下 `Targets.cmake` | 统一 `NAMESPACE` 与链接名 |
| 安装包换路径后失效 | 导出文件记录了源码绝对路径 | 搜索安装目录中的源码路径 | 使用 `BUILD_INTERFACE` / `INSTALL_INTERFACE` |
| ROS2 wrapper 导致核心库难测试 | 核心库包含 ROS2 头文件 | `rg "rclcpp|sensor_msgs" include/` | 把消息转换移到 wrapper |
| CUDA 用户能编，CPU 用户不能编 | CUDA 被写进核心 target | 关闭 CUDA 重新配置 | 拆出可选 CUDA target |
| ASan 没有报错 | 只对部分 target 启用或链接选项缺失 | 查看编译与链接命令 | 封装 sanitizer 函数并应用到测试 target |
| CI 通过但安装包不可用 | CI 没有 consumer 验证 | 新建空目录消费安装包 | 增加安装后 `find_package()` 测试 |

### 17.5 跨章综合练习

回顾 C++语言进阶/Eigen基础与SLAM数学预备：Eigen 固定大小矩阵适合机器人中的小维度几何量，动态矩阵适合点云、雅可比和优化问题。回顾 ROS2高级集成：ROS2 的 QoS 与 composition 决定传感器管线的运行时行为。本章把这两者连接到构建系统中：公共头文件里的 Eigen 类型决定 `PUBLIC` 依赖，ROS2 节点边界决定 wrapper target。

综合练习：

1. 设计一个 `mini_lio_core`，公共 API 使用 `Eigen::Isometry3d`、`std::span` 和自定义点结构，内部使用 Ceres 做滑窗优化。
2. 写出核心库的 CMakeLists，要求安装导出后下游可通过 `find_package(mini_lio REQUIRED)` 使用。
3. 写出 `mini_lio_ros` wrapper 的 CMakeLists，要求订阅 `sensor_msgs::msg::PointCloud2`，但核心库不包含 ROS2 头文件。
4. 增加可选 CUDA 体素滤波后端，并设计 CI 矩阵，要求 CPU 路径不依赖 CUDA。

### 17.6 最终小结

现代 CMake 的核心能力是表达边界。`target_*` 命令表达构建边界，`PUBLIC/PRIVATE/INTERFACE` 表达使用边界，`install(EXPORT)` 表达发布边界，ROS2 wrapper 表达生态边界，CUDA 和 Sanitizer 选项表达运行环境边界。

一个成熟机器人项目的构建系统不追求”所有东西放进一个 target 里刚好能编过”，而是追求”每个 target 的职责、依赖、安装方式和测试方式都能被下游准确理解”。这正是从入门 CMake 走向工程化 CMake 的关键转变。

---

## 18. CMake 3.28+ 新特性与 C++ 模块支持 ⭐⭐⭐

> **这一节解决什么问题**：CMake 3.28（2023 年末）和 3.30（2024 年中）引入了对 C++20 模块的实验性支持，以及 `import std` 的能力。这对机器人项目意味着什么？现阶段是否应该采用？

### C++20 模块的根本动机

C++ 的 `#include` 机制有一个根本性问题——它是文本替换。每个 `#include <Eigen/Core>` 都会把 Eigen 的上万行头文件在预处理阶段展开到当前翻译单元中。如果你的项目有 100 个 `.cpp` 文件都包含了 Eigen，Eigen 的头文件就被解析了 100 次。这是 C++ 编译缓慢的首要原因。

C++20 模块用编译器预编译的二进制接口文件（BMI）替代文本包含。一个模块只编译一次，后续使用者直接导入编译好的接口——不再需要反复解析头文件。理论上这可以将大型项目的编译速度提升 5-10 倍。

CMake 3.28 通过 `CMAKE_EXPERIMENTAL_CXX_MODULE_CMAKE_API` 变量启用了对 C++20 模块的实验性支持。3.30 进一步改进了依赖扫描和 Ninja 集成。但截至 2026 年中，C++20 模块在机器人项目中的实际采用率仍然很低，原因包括：

| 障碍 | 现状 |
|------|------|
| 编译器支持 | GCC 14+、Clang 18+、MSVC 17.6+ 支持，但行为差异大 |
| 构建系统 | CMake 3.28+ 实验支持，Ninja 1.11+ 必需 |
| 库生态 | Eigen、PCL、OpenCV 等核心库均未模块化 |
| 工具链 | clangd、ccache、include-what-you-use 等工具支持不完整 |

### `import std` 的工程意义

CMake 3.30 支持 `import std;`——直接导入整个标准库作为一个模块。这是模块化最容易落地的第一步，因为标准库是所有项目都使用的公共依赖。

```cmake
# CMake 3.30+：启用 import std 支持
cmake_minimum_required(VERSION 3.30)
project(my_slam LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_MODULE_STD ON)  # 启用 import std

add_executable(slam_demo src/main.cpp)
```

```cpp
// main.cpp — 使用 import std 替代标准库头文件
import std;

int main() {
  std::vector<double> data{1.0, 2.0, 3.0};
  std::println(“Size: {}”, data.size());  // C++23 std::print
  return 0;
}
```

`import std` 的好处是消除了标准库头文件的重复解析。对于大量使用 `<vector>`、`<string>`、`<algorithm>`、`<functional>` 的项目，编译速度会有可感知的提升。但它不解决第三方库（Eigen、PCL、Ceres）的编译速度问题，因为这些库仍然通过 `#include` 提供。

### 现阶段的工程建议

对于机器人项目，建议采取保守策略：

| 时机 | 建议 |
|------|------|
| 2026 年 | 在独立工具项目中试用 `import std`，核心库保持头文件 |
| Eigen/PCL 提供模块接口后 | 核心算法库可以开始模块化 |
| ROS2 生态支持模块后 | wrapper 层可以模块化 |

过早采用模块化会遇到工具链不成熟带来的大量摩擦——ccache 缓存失效、clangd 无法跳转、CI 环境需要特定编译器版本。这些问题在工具生态成熟后会自然消失，但现在强行采用的调试成本远超编译速度收益。

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：在依赖未模块化的项目中强制使用 C++20 模块**
>
> **错误做法**：核心算法库使用 `export module slam_core;`，但内部 `#include <Eigen/Core>`。
>
> **现象**：模块和头文件混用时，宏定义的可见性、包含顺序和 ODR（One Definition Rule）违规问题频发。错误信息晦涩且因编译器而异。
>
> **正确做法**：等核心依赖库提供模块接口后再迁移。现阶段可以先在独立的工具程序中试用 `import std`。

---

## 19. FetchContent 高级用法 ⭐⭐⭐

> **这一节解决什么问题**：第 5 节介绍了 FetchContent 的基本用法。本节深入版本锁定、子项目隔离和依赖冲突等进阶工程问题。

### 版本锁定与可复现构建

FetchContent 的 `GIT_TAG` 参数决定了拉取的版本。使用分支名（如 `main`）而不是具体的 commit hash 或 tag 是一个常见的可复现性陷阱——今天拉取的 `main` 和下周拉取的 `main` 可能是不同的代码，但 CMake 不会报错。

```cmake
# ❌ 不可复现：main 分支随时可能更新
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG main
)

# ✅ 可复现：锁定到具体的 tag 或 commit hash
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG v1.5.5  # 或使用完整 commit hash
)
```

更严格的做法是同时指定 `GIT_TAG` 和 `FIND_PACKAGE_ARGS`。CMake 3.24+ 支持 `FetchContent_Declare` 的 `FIND_PACKAGE_ARGS` 选项，让 FetchContent 优先尝试 `find_package()` 寻找系统已安装的版本，找不到时再下载：

```cmake
# CMake 3.24+：优先使用系统安装的版本，找不到则下载
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.7.1
  FIND_PACKAGE_ARGS 3.7 REQUIRED  # 先尝试 find_package(Catch2 3.7)
)
FetchContent_MakeAvailable(Catch2)
```

这种方式的好处是 CI 环境可以预安装依赖（节省下载时间），开发者本地如果没有安装则自动下载——兼顾了便利性和可控性。

### 子项目隔离与选项污染

FetchContent 的一个隐蔽问题是**选项污染**。被拉取的子项目可能定义自己的 `option()` 和缓存变量，这些变量会进入当前项目的缓存空间。如果两个子项目定义了同名的选项（如 `BUILD_TESTING`），后加载的会覆盖先加载的。

```cmake
# 在 FetchContent_MakeAvailable 之前关闭子项目的测试和示例
# 避免子项目的测试 target 污染主项目
set(BUILD_TESTING OFF CACHE BOOL “” FORCE)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL “” FORCE)
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL “” FORCE)

FetchContent_MakeAvailable(nanoflann)

# 恢复主项目的测试设置
set(BUILD_TESTING ON CACHE BOOL “” FORCE)
```

这种做法有效但不优雅。更好的方式是使用 CMake 3.25+ 的 `OVERRIDE_FIND_PACKAGE` 和 `SYSTEM` 选项：

```cmake
# CMake 3.25+：将子项目标记为 SYSTEM，其警告不会传播到主项目
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG v1.5.5
  SYSTEM          # 子项目的 include 路径标记为 -isystem
)
```

`SYSTEM` 关键字让子项目的头文件以系统头文件方式包含，编译器对其禁用 `-Werror` 等警告策略——这防止了第三方库的警告导致主项目编译失败。

### colcon 与 CMake/FetchContent 的交互

在 ROS2 工作区中，colcon 和 FetchContent 的交互需要特别注意。colcon 假设每个包的依赖通过 `package.xml` 声明并由 `rosdep` 或其他 colcon 包满足。FetchContent 绕过了这个发现机制——colcon 不知道你的包通过 FetchContent 下载了 nanoflann。

这意味着 colcon 的依赖拓扑排序可能不正确——它不知道你的包在配置阶段需要网络访问来下载依赖。如果 CI 环境没有网络或网络不稳定，FetchContent 会失败。更稳妥的做法是把 FetchContent 的依赖限制在非 ROS2 的核心算法库中——这个库由标准 CMake 管理，colcon 只负责构建 ROS2 wrapper。

| 依赖来源 | 适合 FetchContent | 适合系统安装 + rosdep |
|----------|-------------------|----------------------|
| 小型 header-only 库 | nanoflann、json、CLI11 | |
| 测试框架 | Catch2、Google Benchmark | GTest（colcon 已提供） |
| 大型依赖 | | Eigen、PCL、OpenCV、Ceres |
| ROS2 包 | | rclcpp、sensor_msgs（通过 colcon） |

### ⚠️ 常见陷阱

> ⚠️ **工程陷阱：FetchContent 拉取的依赖与系统安装的版本冲突**
>
> **错误做法**：系统已安装 Eigen 3.4.0，同时通过 FetchContent 拉取 Eigen 3.3.9——两个版本的头文件路径同时存在。
>
> **现象**：链接时符号冲突，或运行时行为不一致。
>
> **根本原因**：CMake 的 `find_package()` 和 FetchContent 可能找到不同版本的同一个库。
>
> **正确做法**：使用 `FIND_PACKAGE_ARGS` 让 FetchContent 优先使用系统版本。对于 Eigen 这类被大量项目依赖的核心库，始终通过系统安装管理版本。

### 练习

1. **[实操题]** 用 FetchContent 拉取 Catch2 v3.7+，同时通过 `FIND_PACKAGE_ARGS` 支持系统已安装版本的优先使用。编写一个简单测试并用 CTest 运行。
2. **[分析题]** 解释为什么在 colcon 工作区中不应该用 FetchContent 拉取 `rclcpp`，即使技术上可行。从依赖拓扑排序、版本一致性和 CI 可靠性三个角度分析。
3. **[设计题]** 为一个 SLAM 核心库设计 FetchContent 策略：nanoflann（KD-tree）和 tsl-robin-map（高性能哈希表）通过 FetchContent 管理，Eigen 和 Ceres 通过系统安装。编写对应的 CMakeLists 片段。

---

## 本章小结

| 知识点 | 核心要点 | 工程意义 |
|--------|----------|----------|
| target-based CMake | 所有属性绑定到 target，不使用全局命令 | 多 target 项目不再互相污染 |
| PUBLIC / PRIVATE / INTERFACE | 控制依赖是否传递给下游 | “公共接口用什么”决定关键字 |
| find_package | Config Mode 优于 Module Mode，imported target 携带完整使用信息 | 不再手写绝对路径 |
| 核心库与 ROS2 分离 | 核心算法不依赖 rclcpp，wrapper 做消息转换 | 可测试、可复用、可跨平台 |
| 安装导出 | Config.cmake + Targets.cmake 让下游 find_package 可用 | 从”源码能编”到”任何人能用” |
| FetchContent / ExternalProject | 前者同构建图、后者隔离构建 | 按依赖大小和版本需求选择 |
| CUDA 可选后端 | option() 控制，CPU 用户不受影响 | GPU 加速不绑架所有用户 |
| Sanitizer | ASan / TSan / UBSan 通过编译器插桩检测隐蔽错误 | 越界、竞争和未定义行为尽早暴露 |
| CTest 标签 | unit / integration / dataset / gpu 分层运行 | CI 按频率控制测试范围 |
| CMakePresets | 固化常用配置组合 | 团队一致的构建环境 |

## 累积项目：本章新增模块

**SLAM 工程化实践项目**续：

本章新增模块：**可安装、可复用的 CMake 构建基础设施**

1. 为 `mini_slam_core` 编写完整 CMakeLists，包含 target-based 依赖管理和 PUBLIC/PRIVATE 正确分离
2. 实现安装导出链路（`Config.cmake.in` + `Targets.cmake`），确保 `find_package(mini_slam)` 可用
3. 编写 `mini_slam_ros` ROS2 wrapper 包，核心库不包含任何 ROS2 头文件
4. 添加可选 CUDA 后端，通过 `option(MINI_SLAM_ENABLE_CUDA)` 控制
5. 配置 GTest + CTest 分层测试（unit / dataset 标签），添加 ASan/UBSan 选项
6. 编写 consumer 项目验证安装导出的正确性

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| [CMake 官方文档](https://cmake.org/cmake/help/latest/) | ⭐ | 命令和变量的权威参考 |
| *Professional CMake: A Practical Guide* (Craig Scott, 2024) | ⭐⭐ | 最全面的现代 CMake 工程化指南 |
| [*Effective Modern CMake*](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1) | ⭐⭐ | 社区总结的现代 CMake 最佳实践清单 |
| [CMake 3.x 从入门到项目实战](https://www.youtube.com/playlist?list=PLalVdRk2RC6o5GHu618ARWh0VO0bFlif4) | ⭐ | 视频系列，适合初学者跟练 |
| [KISS-ICP CMakeLists.txt](https://github.com/PRBonn/kiss-icp) | ⭐⭐ | 简洁的核心库 + ROS2 wrapper 分离示范 |
| [Ceres Solver CMake](https://github.com/ceres-solver/ceres-solver) | ⭐⭐⭐ | 大型 C++ 库的安装导出和依赖管理参考 |
| [GTSAM CMake](https://github.com/borglab/gtsam) | ⭐⭐⭐ | 复杂依赖（Eigen、Boost、TBB）的工程化管理 |
