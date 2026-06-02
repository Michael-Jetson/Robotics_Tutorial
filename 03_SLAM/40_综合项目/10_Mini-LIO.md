# 第50章：Mini-LIO——从零构建 LiDAR-Inertial 里程计（第52-53周）

> **本章定位**：这是整个 SLAM 方向的毕业设计（Capstone）。前面所有章节——李群李代数、状态估计、卡尔曼滤波、点云处理、C++ 工程技术——在这里第一次拧成一根绳。我们不抄 FAST-LIO 的代码，而是用学过的工具**从零架构并实现**一个简化但闭环可跑的 LiDAR-Inertial 里程计（LIO）。
>
> **本章与第51章的分工**：本章负责"把系统搭出来并在自造数据上跑通"；下一章（第51章·系统集成评估与调优）负责"在 KITTI/MulRan 等真实公开数据集上评估精度、做性能 Profiling 与调参"。本章把每个模块的原理和可运行框架交给你，第51章教你如何把它推向产品级。

---

## 环境配置指南

LIO 系统的依赖比纯算法练习重：它需要点云库、矩阵库、图优化库和构建系统协同工作。先把环境钉死，再写代码——否则你会在"为什么 Eigen 和 PCL 的对齐冲突导致段错误"这种问题上浪费整个下午。

### 系统要求

| 项目 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 22.04 LTS | 与 ROS2 Humble 绑定；Ubuntu 20.04 + ROS2 Foxy 亦可，命令需相应调整 |
| 编译器 | GCC 11 / Clang 14 | 必须支持 C++17（结构化绑定、`std::optional`、`if constexpr`） |
| CPU | x86-64 四核以上 | LIO 是 CPU 密集型；ARM（Jetson）需交叉编译，见第51章 |
| 内存 | ≥ 8 GB | ikd-Tree 局部地图 + 点云缓冲；大场景需 16 GB |
| GPU | 不需要 | 本章是纯 CPU 实现；GPU 加速属于第51章可选进阶 |

### 版本兼容表

| 组件 | 推荐版本 | 最低版本 | 最高测试版本 | 备注 |
|------|---------|---------|------------|------|
| C++ 标准 | C++17 | C++17 | C++20 | C++20 的 concept 可优化 Policy 模板报错 |
| Eigen | 3.4.0 | 3.3.7 | 3.4.0 | 头文件库；注意 16 字节对齐 |
| PCL | 1.12 | 1.10 | 1.13 | 仅用 I/O、VoxelGrid、CropBox，不用其 ICP |
| Sophus | 1.22.10 | 1.0 | 1.22.10 | SO3/SE3 流形运算；可用 manif 替代 |
| GTSAM | 4.2 | 4.0 | 4.2 | 仅回环模块用到（可选） |
| ROS2 | Humble | Foxy | Humble | 数据 I/O 与可视化；非强依赖，可纯文件读取 |
| spdlog | 1.10 | 1.8 | 1.11 | 异步日志 |
| yaml-cpp | 0.7 | 0.6 | 0.7 | 配置文件解析 |
| TBB | 2021.5 | 2020 | 2021.8 | 并行 reduce；可降级为串行 |
| GoogleTest | 1.12 | 1.10 | 1.13 | 单元测试 |

> **本质洞察**：上面这张表里只有 Eigen 和 Sophus 是 LIO 的"算法内核依赖"——它们决定了你能不能在流形上做状态估计。PCL/GTSAM/ROS2 都是"工程外壳依赖"，可替换甚至可去掉。理解这个分层，你就明白为什么 FAST-LIO 的核心算法只有几千行，却能支撑起整个系统——**算法内核小，工程外壳大**，这是几乎所有 SLAM 系统的共同形态。

### 安装步骤（Ubuntu 22.04）

```bash
# 1. 基础工具链与构建系统
sudo apt update
sudo apt install -y build-essential cmake git libtbb-dev libyaml-cpp-dev libspdlog-dev libgtest-dev

# 2. Eigen（头文件库，apt 版本足够）
sudo apt install -y libeigen3-dev    # 装到 /usr/include/eigen3

# 3. PCL（点云 I/O 与滤波）
sudo apt install -y libpcl-dev pcl-tools

# 4. Sophus（依赖 Eigen 与 fmt）
sudo apt install -y libfmt-dev
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
cmake -B build -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF
sudo cmake --build build --target install

# 5. GTSAM（回环可选；体积大，编译约 20 分钟）
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# 6. ROS2 Humble（数据 I/O 与 RViz2 可视化，可选）
# 按官方文档安装：https://docs.ros.org/en/humble/Installation.html
```

> **配置陷阱：Eigen 头文件路径不一致导致 ABI 崩溃**
> 错误做法：系统装了 `libeigen3-dev`（3.4），又通过源码装了另一份 Eigen（3.3）到 `/usr/local/include`，CMake `find_package(Eigen3)` 找到旧版。
> 现象：编译通过，但运行时在 `Eigen::Matrix` 构造处随机段错误，AddressSanitizer 报 misaligned address。
> 根本原因：不同 Eigen 版本对固定大小矩阵的对齐策略不同，链接进同一进程时 ABI 不兼容。
> 正确做法：全项目只保留一份 Eigen。用 `cmake --debug-find` 确认 `EIGEN3_INCLUDE_DIR` 指向唯一路径，并在 `target_link_libraries` 中显式链接 `Eigen3::Eigen`。

### 项目骨架与构建系统

我们的目录结构遵循"算法内核 / 工程外壳"分层，每个模块一个子目录，对应骨架里列出的六大模块：

```
mini_lio/
├── CMakeLists.txt
├── config/
│   └── params.yaml              # 传感器外参 + 算法参数 + 噪声标定
├── include/mini_lio/
│   ├── common.hpp               # 数据类型：State, IMUData, PointXYZIRT
│   ├── preprocessing.hpp        # 点云预处理
│   ├── imu_integration.hpp      # IMU 中值积分 / 预积分
│   ├── ieskf.hpp                # 迭代误差状态卡尔曼滤波
│   ├── ikd_tree.hpp             # 增量 kd-Tree 局部地图
│   ├── registration.hpp         # 点到面残差
│   └── loop_closure.hpp         # 回环检测（可选）
├── src/
│   ├── lio_node.cpp             # 主循环：传感器同步 → 估计 → 建图
│   └── ...                      # 各模块实现
└── test/
    └── test_ieskf.cpp           # GoogleTest 单元测试
```

下面是顶层 `CMakeLists.txt` 的核心片段。注意我们用现代 CMake 的 `target_*` 写法而非全局变量，并默认开启 Sanitizer（调试时）：

```cmake
cmake_minimum_required(VERSION 3.16)
project(mini_lio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # LIO 必须 -O2 以上，否则实时性不达标
endif()

# Release 加向量化；Debug 加 Sanitizer
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g -fsanitize=address,undefined")

find_package(Eigen3 3.4 REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

add_library(mini_lio_core
  src/preprocessing.cpp src/imu_integration.cpp
  src/ieskf.cpp src/ikd_tree.cpp src/registration.cpp)
target_include_directories(mini_lio_core PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(mini_lio_core
  PUBLIC Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES}
  PRIVATE TBB::tbb yaml-cpp spdlog::spdlog)
```

> **配置陷阱：`-march=native` 在交叉部署时引发非法指令**
> 错误做法：开发机用 `-march=native` 编译，把二进制直接拷到 Jetson 运行。
> 现象：Jetson 上启动即 `SIGILL (Illegal instruction)`。
> 根本原因：`native` 按编译机的 CPU 生成 AVX 指令，目标 ARM CPU 不支持。
> 正确做法：交叉部署时改用目标架构的明确标志（见第51章交叉编译节）；本机开发用 `native` 没问题。

---

## Quick Start（5 分钟跑通最小骨架）

在写完整系统前，先确认"骨架能跑"。下面这段是一个**自包含的最小可运行示例**——不依赖 ROS、不依赖真实数据，用程序生成一段直线+旋转的模拟轨迹，喂给 IMU 积分模块，验证你的环境和构建链路通畅：

```bash
mkdir -p mini_lio && cd mini_lio
# 把后文 §50.3 的 imu_integration 代码与下面的 main 放进去
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/quickstart
# 预期输出：积分 100 帧 IMU 后，位置漂移 < 0.05 m（纯积分短时精度）
```

```cpp
// quickstart.cpp —— 验证环境：纯 IMU 中值积分一条已知轨迹
#include "mini_lio/imu_integration.hpp"
#include <iostream>
int main() {
  mini_lio::State x;                       // 初始静止状态
  x.grav = Eigen::Vector3d(0, 0, -9.81);   // 重力向量
  const double dt = 0.01;                  // 100 Hz IMU
  for (int k = 0; k < 100; ++k) {
    // 模拟：静止水平放置 → 加速度计读到 +g（抵消重力），陀螺读 0
    mini_lio::IMUData imu;
    imu.acc  = Eigen::Vector3d(0, 0, 9.81);
    imu.gyro = Eigen::Vector3d::Zero();
    mini_lio::imu_predict(x, imu, dt);     // 中值积分一步
  }
  std::cout << "position drift = " << x.pos.norm() << " m\n";
  return 0;                                // 静止应≈0，验证重力补偿正确
}
```

如果这段输出的漂移接近 0，说明你的 Eigen/Sophus/构建链路全部就绪，且重力补偿方向正确（这是 LIO 最常见的符号错误来源）。现在可以进入正题。

---

## 前置自测

开始前，请确认你能回答以下问题。答不出来的，回到对应章节补课——LIO 是知识的总集成，欠债会在调试时连本带利地还。

1. **（指向第10章·SLAM 理论 / 李群李代数）** 旋转矩阵 $\boldsymbol R_1, \boldsymbol R_2 \in SO(3)$，为什么 $\boldsymbol R_1 + \boldsymbol R_2 \notin SO(3)$，而我们却需要"两个旋转的差"？$\boxplus$（boxplus）和 $\boxminus$（boxminus）算子分别做什么？

2. **（指向状态估计 / 卡尔曼滤波章节）** 标准卡尔曼滤波的预测—更新两步分别更新什么？为什么 LIO 里要用"误差状态"（error state）而不是直接对名义状态（nominal state）做 EKF？

3. **（指向点云处理章节）** 给定一帧无序点云和一个查询点，最近邻搜索的朴素复杂度是多少？kd-Tree 把它降到多少？为什么"边插入点边查询"会让普通 kd-Tree 退化？

4. **（指向 C++ 工程章节）** `std::move(cloud)` 对一个 `pcl::PointCloud::Ptr`（其实是 `shared_ptr`）做了什么？对一个 `PointCloud`（值类型）又做了什么？哪种能避免百万级点的深拷贝？

5. **（指向 IMU / 传感器章节）** IMU 的加速度计测量的是"运动加速度"还是别的什么？为什么静止时它读到的是 $+g$ 而不是 $0$？陀螺仪和加速度计的"偏置（bias）"为什么会随时间漂移？

---

## 本章目标

学完本章后，你应该能够：

1. **画出并解释 LIO 的完整数据流**：从原始点云 + IMU 流，到去畸变、状态预测、scan-to-map 配准、地图更新的每一个环节，说清每个模块的输入输出与时序约束。
2. **实现 IMU 中值积分与预积分**：在流形上正确传播名义状态（位置/速度/旋转）与误差状态协方差，理解为什么用中值而非欧拉积分。
3. **实现基于 IMU 的点云去畸变（反向传播）**：用每个点的时间戳把一帧"运动中采集的畸变点云"矫正到统一时刻。
4. **从零实现迭代误差状态卡尔曼滤波（iESKF）**：写出 $\boxplus/\boxminus$、误差状态传播、point-to-plane 观测、迭代更新与"维度等于状态维度"的高效卡尔曼增益公式。
5. **实现 ikd-Tree 局部地图的核心操作**：增量插入、惰性删除、盒式范围搜索、最近邻查询，理解它为什么比静态 kd-Tree 快。
6. **接入回环检测做位姿图优化**：用 GTSAM 把里程计因子与回环因子组成因子图，修正长时漂移（LIO-SAM 风格，可选）。
7. **把六大模块组装成可在模拟/真实数据上跑通的闭环系统**，并用 GoogleTest 守护核心数学。

### 本章知识导航

整章按 LIO 的数据流顺序展开，每一节是流水线上的一个工位。建议第一遍按顺序读，理解数据如何流动；第二遍可针对某个模块深挖。

```
                      ┌──────────────────────────────────────────┐
   IMU 流 (200Hz) ───►│  §50.3 IMU 中值积分 / 预积分               │──┐
                      │   名义状态传播 + 误差协方差传播             │  │ 预测先验
                      └──────────────────────────────────────────┘  │
                                                                     ▼
   LiDAR 帧 ─► §50.2 ─► §50.4 ─────► §50.5 ───────────► §50.6/§50.7 ──► §50.8
   (10Hz)    预处理     去畸变       iESKF 更新           ikd-Tree      回环(可选)
            降采样/    (反向传播)   (point-to-plane     局部地图       位姿图优化
            裁剪/过滤               迭代收敛)            增量插入/搜索
                                       ▲                    │
                                       └────── 残差查询 ─────┘
                      ┌──────────────────────────────────────────┐
                      │  §50.9 主循环：传感器同步 + 模块编排        │
                      └──────────────────────────────────────────┘
```

| 节 | 模块 | 难度 | 核心产出 |
|----|------|------|---------|
| §50.1 | 系统架构与数据流 | ⭐⭐ | 全局视图、时序约束、紧/松耦合权衡 |
| §50.2 | 点云预处理 | ⭐⭐ | 自定义点类型、降采样、去自身点 |
| §50.3 | IMU 积分与预积分 | ⭐⭐⭐ | 名义状态传播、误差协方差传播 |
| §50.4 | 点云去畸变 | ⭐⭐⭐ | 反向传播运动补偿 |
| §50.5 | iESKF 状态估计 | ⭐⭐⭐⭐ | $\boxplus/\boxminus$、迭代更新、高效增益 |
| §50.6 | ikd-Tree 局部地图 | ⭐⭐⭐ | 增量插入、惰性删除、盒式搜索 |
| §50.7 | point-to-plane 配准 | ⭐⭐⭐ | 法向量估计、残差与雅可比 |
| §50.8 | 回环检测（可选） | ⭐⭐⭐ | 因子图、位姿图优化 |
| §50.9 | 系统集成 | ⭐⭐⭐ | 传感器同步、主循环、线程模型 |

### 前置知识桥接

本章大量复用前面的工具，这里先把会反复用到的三件武器激活，免得你边读边翻：

- **$SO(3)$ 与 $\boxplus/\boxminus$（回顾第10章·SLAM 理论）**：旋转构成李群 $SO(3)$，它是弯曲的流形，没有良好定义的加减法（$\boldsymbol R_1 + \boldsymbol R_2 \notin SO(3)$），只有乘法封闭。但在切空间（李代数 $\mathfrak{so}(3)\cong\mathbb R^3$）里可以做加减。$\boxplus$ 把"流形上的点 $\oplus$ 切空间的小增量"映回流形：$\boldsymbol R \boxplus \delta\boldsymbol\theta = \boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)$；$\boxminus$ 反过来求两个流形点之间的切空间增量：$\boldsymbol R_1 \boxminus \boldsymbol R_2 = \mathrm{Log}(\boldsymbol R_2^\top \boldsymbol R_1)$。**整个 iESKF 的全部"加减"都靠这两个算子**——这是本章最该记牢的一件事。

- **误差状态卡尔曼滤波（回顾状态估计章节）**：直接对位姿做 EKF 会遇到"旋转不能线性化"的麻烦。误差状态 KF 的思路是：名义状态 $\boldsymbol x$（大量、非线性、在流形上）用确定性积分传播；误差状态 $\delta\boldsymbol x$（小量、近似线性、在切空间里）用卡尔曼滤波估计协方差。两者通过 $\boldsymbol x_{\text{true}} = \boldsymbol x \boxplus \delta\boldsymbol x$ 联系。

- **kd-Tree 最近邻搜索（回顾点云处理章节）**：kd-Tree 把空间递归二分，把最近邻查询从 $O(N)$ 降到平均 $O(\log N)$。但它是**静态**结构——建好后插点会破坏平衡。LIO 的地图却在持续增长，这正是 §50.6 要解决的矛盾。

### 预计阅读时间

约 5-7 小时精读（不含动手实现）。完整实现并调通，预计 2 周（与课程周数一致）。建议节奏：第 1 周完成 §50.2-§50.5（能在模拟数据上跑出里程计），第 2 周完成 §50.6-§50.9（接真实数据 + 地图 + 可选回环）。

---

## §50.1 系统架构与数据流 ⭐⭐

### 这一节解决什么问题

在写任何一行算法代码之前，必须先回答一个工程问题：**这些模块怎么拼？数据按什么顺序、在什么线程、以什么频率流动？** 很多人一上来就写卡尔曼滤波的矩阵运算，结果写到一半发现"IMU 和 LiDAR 频率不一样，怎么对齐时间戳"——架构没想清楚，算法写得再漂亮也拼不起来。

### 动机：为什么需要 LiDAR 和 IMU 融合

先问一个反面问题：**只用 LiDAR 行不行？只用 IMU 行不行？**

| 传感器 | 单独使用的致命缺陷 | 互补关系 |
|--------|------------------|---------|
| 纯 LiDAR | 旋转太快时帧间重叠不足，配准失败；点云在快速运动时严重畸变；退化场景（长走廊、隧道）无几何约束 | IMU 提供高频运动先验，给配准好的初值，并解释畸变 |
| 纯 IMU | 加速度二次积分，位置误差随时间 $t^2$ 发散；偏置漂移；几秒内就不可用 | LiDAR 提供绝对几何约束，反过来标定 IMU 偏置 |

> **本质洞察**：LIO 的本质是一场"借力"——IMU 在 LiDAR 帧之间的空档（100 ms）里用高频积分撑住位姿，LiDAR 每帧到来时用几何约束把 IMU 积累的漂移"拽回来"，顺便估计 IMU 的偏置。两者的频率差（IMU 200 Hz vs LiDAR 10 Hz）不是麻烦，恰恰是融合的物理基础：**快的填空档，慢的纠漂移。**

### 反面：紧耦合 vs 松耦合

历史上有两种融合架构，理解它们的差异决定了你整个系统的形态：

| 维度 | 松耦合（Loosely-coupled） | 紧耦合（Tightly-coupled） |
|------|--------------------------|--------------------------|
| 做法 | LiDAR 独立算出一个位姿，IMU 独立积分一个位姿，再用一个滤波器融合两个"位姿结果" | LiDAR 的原始观测（点到面残差）和 IMU 直接进入同一个状态估计器 |
| 信息利用 | 丢失了原始观测的不确定性结构 | 保留所有原始信息，理论最优 |
| 鲁棒性 | LiDAR 配准一旦失败，松耦合直接喂错位姿 | 退化方向上自动更依赖 IMU |
| 代表系统 | 早期 LOAM + EKF | FAST-LIO、LIO-SAM、本章的 Mini-LIO |
| 复杂度 | 简单，模块解耦 | 复杂，状态估计器要懂 LiDAR 残差 |

**Mini-LIO 选紧耦合**，因为这才是现代 LIO 的主流，也才能真正训练你"把原始观测塞进状态估计器"的能力。具体说，我们采用 FAST-LIO 风格的 **iESKF 紧耦合**：LiDAR 的 point-to-plane 残差作为观测，IMU 积分作为预测。

### 历史：LIO 架构的两条技术路线

现代 LIO 有两大流派，本章会都讲到，但主线走滤波派：

- **滤波派（FAST-LIO / FAST-LIO2，HKU-MARS，2021）**：用迭代误差状态卡尔曼滤波（iESKF）做紧耦合，配 ikd-Tree 增量地图，直接对原始点做 scan-to-map。特点是计算极快、单帧延迟低、内核代码精简。本章 §50.3-§50.7 是这条路线的简化复刻。
- **图优化派（LIO-SAM，MIT，2020）**：用因子图（GTSAM）把 IMU 预积分因子、LiDAR 里程计因子、回环因子、GPS 因子统一优化，支持全局位姿图与回环。特点是精度高、易扩展多传感器、天然支持回环。本章 §50.8 借它的因子图做回环。

> **多视角理解（类比）**：滤波派像"流水线上的工人"——每帧到来就地处理、立刻产出、不回头；图优化派像"会计做账"——把所有约束记在一张大账本（因子图）上，定期全盘平账（优化）。**像的地方**：两者最终都在求最大后验估计（MAP）。**不像的地方**：滤波派用马尔可夫假设只保留当前状态（边缘化历史），图优化派显式保留一段历史状态做联合优化——这就是为什么图优化派天然能做回环（历史还在账本上），而纯滤波派要做回环必须额外挂一个位姿图。

### 理论：完整数据流与时序

下面是 Mini-LIO 一帧的完整生命周期。把它刻在脑子里，后面每一节都是在实现其中一个箭头：

```
时刻 t_k 收到第 k 帧 LiDAR（帧内点的时间戳横跨 [t_{k-1}, t_k]）:

 (1) 取出 [t_{k-1}, t_k] 区间内的所有 IMU 数据
        │
 (2) IMU 前向传播：从 x_{k-1} 出发，用中值积分逐个 IMU 步推进
     名义状态，同时传播误差协方差 P  →  得到预测先验 x̂_k, P̂_k  ……§50.3
        │
 (3) 反向传播去畸变：用 (2) 得到的每个 IMU 时刻的位姿，
     把帧内每个点按其时间戳投影到帧末时刻 t_k  ……§50.4
        │
 (4) iESKF 迭代更新（重复直到收敛）：  ……§50.5
       (4a) 对每个去畸变后的点，在 ikd-Tree 地图里找最近的 5 个点  ……§50.6
       (4b) 拟合局部平面，算 point-to-plane 残差与雅可比  ……§50.7
       (4c) 求卡尔曼增益，更新误差状态 δx，用 boxplus 注入名义状态
       (4d) 残差收敛？是→退出；否→回到 (4a) 用新位姿重新找最近邻
        │
 (5) 用收敛后的位姿把这帧点变换到世界系，增量插入 ikd-Tree 地图  ……§50.6
        │
 (6)（可选）若触发关键帧，送入回环检测线程做位姿图优化  ……§50.8
        │
 (7) 发布位姿/轨迹/点云地图（ROS topic 或写文件）
```

> **阶段小结**：到这里我们完成了"鸟瞰"——你已经知道 LIO 一帧要做 7 件事，以及每件事对应本章哪一节。接下来从 §50.2 开始，我们沿着这条流水线从头实现，每一节填上一个工位。

### 时序约束与线程模型

LIO 是实时系统，时序错了精度全无。三条硬约束：

1. **实时性约束**：单帧总处理时间必须小于 LiDAR 周期（10 Hz → 100 ms）。否则缓冲区堆积，延迟雪崩。
2. **因果性约束**：去畸变和更新只能用 $t_k$ 及之前的数据，不能用未来的 IMU。
3. **时间戳对齐**：IMU 和 LiDAR 来自不同硬件，时钟可能不同步；必须有统一时间基准（硬件触发或软件对齐）。

线程模型上，Mini-LIO 采用经典的"生产者—消费者"：

```cpp
// 概念性线程划分（详细实现见 §50.9）
// 线程 A（ROS 回调，高频）：只负责把 IMU/LiDAR 推进各自的缓冲队列，加锁要短
// 线程 B（主估计循环）：从队列取数据 → 预测 → 去畸变 → 更新 → 建图
// 线程 C（可选，回环）：低频，从关键帧数据库找回环，跑位姿图优化
```

> **编程陷阱：在 ROS 回调里直接做重计算**
> 错误做法：在 LiDAR 回调函数里直接调用 iESKF 更新。
> 现象：回调阻塞，后续消息在 ROS 执行器队列里堆积，丢帧、延迟暴涨。
> 根本原因：ROS 单线程执行器中，回调是串行的；一个回调跑 80 ms，期间到来的所有消息都得排队。
> 正确做法：回调只做"加锁→push 到 deque→解锁"这种 O(1) 操作，重计算放到独立的估计线程。这正是骨架里"`std::deque` + `std::mutex` + `lock_guard` 传感器缓冲"的设计动机。

### ⚠️ 常见陷阱

> **概念误区：以为"融合"就是把两个位姿加权平均**
> 新手想法："IMU 给一个位姿，LiDAR 给一个位姿，融合不就是 $0.5 \times$ 这个 $+ 0.5 \times$ 那个？"
> 实际上：那是松耦合里最朴素的做法，且权重不该是固定的 0.5，而应由各自的协方差决定（卡尔曼增益）。紧耦合根本不先算两个位姿，而是把 LiDAR 残差和 IMU 预测放进同一个最优化问题里联合求解。
> 正确理解：融合 = 在贝叶斯框架下用观测的似然去修正预测的先验，权重由不确定性（协方差）自动决定。

> **配置陷阱：IMU 和 LiDAR 外参（extrinsic）写反或漏标**
> 错误做法：以为 IMU 和 LiDAR 在同一点，外参设为单位阵。
> 现象：里程计在直线运动时还行，一旦剧烈旋转，轨迹出现周期性"画圈"漂移。
> 根本原因：IMU 和 LiDAR 物理上有几厘米到几十厘米的安装偏移（杆臂效应，lever-arm）。旋转时这个偏移会产生额外的线速度，外参错误使补偿错误。
> 正确做法：在 `config/params.yaml` 里明确标定 $\boldsymbol T_{IL}$（IMU 到 LiDAR 的变换），旋转时验证轨迹无规律性漂移。

> **概念误区：以为去畸变是可选的"锦上添花"**
> 新手想法："点云畸变那么小，先不做去畸变跑通再说。"
> 实际上：机械式 LiDAR 一帧扫 100 ms，期间车辆以 10 m/s 移动就是 1 米位移，固态 LiDAR 旋转时畸变更甚。不去畸变，scan-to-map 配准的残差里混入运动误差，迭代不收敛或收敛到错误位姿。
> 正确理解：去畸变是紧耦合 LIO 的必备环节，且它依赖 IMU 预测的位姿——这就是为什么 §50.3（IMU 积分）必须在 §50.4（去畸变）之前。

### 练习

1. **（架构推理）** 假设你的 LiDAR 是 10 Hz、IMU 是 200 Hz。一帧 LiDAR 区间内大约有多少个 IMU 数据？如果某次 IMU 因丢包只来了 5 个，你的去畸变和预测会受什么影响？设计一个降级策略。
2. **（紧/松耦合权衡）** 在长直走廊（沿走廊方向无几何约束、退化）场景下，分别分析松耦合和紧耦合 LIO 的表现差异。为什么紧耦合在退化方向上"更聪明"？（提示：从观测的雅可比秩亏角度想）
3. **（数据流画图）** 不看本节的数据流图，自己默画一遍 Mini-LIO 一帧的 7 步，并在每一步旁边标注"输入是什么、输出是什么、用到本章哪一节"。这是检验你是否真正理解架构的最佳方式。

---

## §50.2 点云预处理 ⭐⭐

### 模块功能与在管线中的位置

预处理是流水线的第一个工位，输入原始 LiDAR 帧，输出"干净、稀疏、带时间戳"的点云。它要解决三个工程现实问题：**点太多（百万级）算不动、有打到机器人自己身上的点、有无效的近/远点**。这一节的产出会喂给 §50.4 去畸变。

### 动机：为什么不能直接拿原始点云做配准

反面思考：**直接拿原始 10 万~200 万点的帧做 scan-to-map，会怎样？**

- 最近邻搜索是配准的瓶颈，点数翻倍，每帧耗时翻倍——实时性直接崩。
- 原始点云密度极不均匀：近处点密、远处点疏。不降采样，近处点主导配准，远处的几何约束被淹没。
- 打到自身（车顶、机械臂）的点会被错误地当成环境，污染地图。

所以预处理的本质是：**用最小的信息损失，把点云降到"配准够用且算得动"的规模。**

### 自定义点类型：为什么 PCL 自带的不够用

去畸变需要每个点的**采集时间戳**，而 `pcl::PointXYZI` 没有时间字段。不同厂商的 LiDAR 还会附带 `ring`（线束号）等信息。我们用 PCL 的宏注册一个自定义点类型：

```cpp
// include/mini_lio/common.hpp  ——  自定义点类型（适配 Velodyne/Ouster/Livox）
#define PCL_NO_PRECOMPILE      // 必须在 include PCL 之前，否则自定义点无法实例化模板
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mini_lio {

struct EIGEN_ALIGN16 PointXYZIRT {   // EIGEN_ALIGN16: 保证 SSE 对齐，避免未对齐访问崩溃
  PCL_ADD_POINT4D;                   // 展开为 x,y,z 加一个 padding（凑齐 16 字节）
  float    intensity;                // 反射强度
  uint16_t ring;                     // 线束号（Velodyne/Ouster 有；Livox 无，置 0）
  float    time;                     // 相对帧首的时间偏移（秒），去畸变的关键字段
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // 重载 new，保证堆分配也 16 字节对齐
} EIGEN_ALIGN16;

}  // namespace mini_lio

// 向 PCL 注册字段名与内存偏移，使 pcl::PointCloud<PointXYZIRT> 可用
POINT_CLOUD_REGISTER_POINT_STRUCT(mini_lio::PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (float, time, time))
```

逐行讲清几个"为什么"：

- **`#define PCL_NO_PRECOMPILE`**：PCL 默认对内置点类型预编译了模板实例。自定义点类型不在预编译名单里，不加这个宏，链接时会报"undefined reference to pcl::...<PointXYZIRT>"。
- **`EIGEN_ALIGN16` 与 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`**：点里含 `PCL_ADD_POINT4D`（本质是 Eigen 的 4 维向量），Eigen 要求 16 字节对齐才能用 SSE 指令。栈对象靠 `EIGEN_ALIGN16`，堆对象（`new`）靠重载的 operator new。漏了它们，在某些编译器下会出现诡异的段错误。
- **`time` 字段语义**：约定为"相对该帧第一个点的时间偏移（秒）"。不同厂商原始数据单位不同（Ouster 是纳秒绝对时间、Velodyne 是相对秒），驱动里要统一归一化到这个约定。

> **配置陷阱：不同 LiDAR 的 time 字段单位/含义不一致**
> 错误做法：照抄某个驱动的 time 解析，换一个雷达就用同一套。
> 现象：去畸变后点云"炸开"或畸变更严重，配准发散。
> 根本原因：Ouster 的 `t` 是纳秒级绝对时间戳，Velodyne 的是相对帧首的秒，Livox 自定义 SDK 又不同。单位/基准不统一，去畸变时乘错时间。
> 正确做法：在预处理里把所有雷达的时间统一归一化为"相对帧首的秒"，并写单元测试断言 `time` 落在 $[0, 0.1]$（10 Hz 帧）区间内。

### 配置详解：预处理的关键参数

```yaml
# config/params.yaml （预处理部分）
preprocessing:
  voxel_leaf_size: 0.3      # 体素降采样边长（米）。越大点越少越快、细节越糊
  min_range: 0.5            # 距离过滤：小于此值的点丢弃（近处噪点/自身反射）
  max_range: 100.0          # 大于此值的点丢弃（远处稀疏不可靠）
  ego_box_min: [-1.0, -1.0, -1.0]   # CropBox：去除自身点的包围盒（米）
  ego_box_max: [ 1.0,  1.0,  1.0]
```

| 配置项 | 默认值来源 | 调大/调小的影响 | 与其他配置的耦合 |
|--------|-----------|----------------|-----------------|
| `voxel_leaf_size` | 经验值，地面机器人常用 0.3-0.5 m | 调小→点多、精度高、变慢；调大→点少、快、地图糊 | 与 §50.6 地图体素分辨率应协调；与 ICP 收敛半径相关 |
| `min_range` | 略大于传感器最小量程 | 太小→保留自身反射噪点；太大→丢失近处有效约束 | 须覆盖 `ego_box` 之外仍可能的近场噪声 |
| `max_range` | 传感器有效量程 | 太大→引入稀疏不可靠远点；太小→丢失远处地标 | 影响局部地图范围（§50.6） |
| `ego_box` | 实测机器人外形 | 太小→车体点漏进地图；太大→误删近处有效点 | 与 `min_range` 共同决定近场过滤 |

### 代码走读：预处理流水线

```cpp
// src/preprocessing.cpp  ——  原始帧 → 干净稀疏帧
#include "mini_lio/preprocessing.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace mini_lio {

CloudPtr Preprocessor::process(const CloudPtr& raw) const {
  // 步骤 1：距离过滤 + 去自身点（一趟遍历，避免多次拷贝）
  auto filtered = std::make_shared<pcl::PointCloud<PointXYZIRT>>();
  filtered->reserve(raw->size());
  for (const auto& p : raw->points) {
    const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (r2 < min_range_ * min_range_ || r2 > max_range_ * max_range_) continue;  // 距离窗
    if (in_ego_box(p)) continue;                                                  // 去自身
    filtered->push_back(p);
  }

  // 步骤 2：体素降采样。注意：VoxelGrid 会丢失自定义字段，需特殊处理（见下方陷阱）
  auto downsampled = std::make_shared<pcl::PointCloud<PointXYZIRT>>();
  pcl::VoxelGrid<PointXYZIRT> voxel;
  voxel.setLeafSize(leaf_, leaf_, leaf_);
  voxel.setInputCloud(filtered);
  voxel.filter(*downsampled);

  return downsampled;   // 返回 shared_ptr，移动语义下零深拷贝
}

bool Preprocessor::in_ego_box(const PointXYZIRT& p) const {
  return p.x > ego_min_.x() && p.x < ego_max_.x() &&
         p.y > ego_min_.y() && p.y < ego_max_.y() &&
         p.z > ego_min_.z() && p.z < ego_max_.z();
}

}  // namespace mini_lio
```

> **本质洞察**：注意 `process` 返回的是 `shared_ptr`（`CloudPtr`），不是值。这是骨架"移动语义传递点云数据，避免百万级点拷贝"的体现——**在 LIO 里，点云永远用指针传递，绝不按值传参**。一次不经意的按值传参就是一次百万点深拷贝，足以让你的实时性泡汤。

### 运行验证

预处理跑通的标志：

- 输入 12 万点的 Velodyne 帧，`voxel_leaf_size=0.3` 后输出约 1-3 万点（具体看场景密度）。
- 打印过滤前后点数：`raw=120000 -> filtered=95000 -> downsampled=18000` 这样的递减是正常的。
- 可视化时车体周围 1 m 内应无点（自身点已去除）。

### ⚠️ 常见陷阱

> **编程陷阱：`pcl::VoxelGrid` 丢失 ring/time 自定义字段**
> 错误做法：对带 `time` 字段的点云直接用 PCL `VoxelGrid`，以为字段会保留。
> 现象：降采样后所有点的 `time` 变成 0 或脏值，去畸变全错。
> 根本原因：`VoxelGrid` 对落在同一体素的点做质心平均，平均逻辑只处理 XYZ 和已知字段，自定义字段被丢弃或置零。
> 正确做法：要么自己实现保留时间的体素降采样（每个体素取最近质心的那个原始点，保留其 time）；要么**先去畸变再降采样**（很多实现采用这个顺序）。本章主线采用"先去畸变后降采样"，规避此坑。

> **编程陷阱：在循环里对 `filtered` 反复 `push_back` 不 `reserve`**
> 错误做法：`filtered` 不预留容量，循环里直接 `push_back`。
> 现象：大帧点云预处理偶发卡顿。
> 根本原因：`vector` 容量不足时按倍数重新分配并拷贝已有元素，百万点会触发多次 $O(N)$ 重分配。
> 正确做法：`filtered->reserve(raw->size())` 一次性预留，把多次重分配降为一次。

> **概念误区：以为降采样越狠越好（追求速度）**
> 新手想法："点越少越快，leaf_size 设大点稳赚不赔。"
> 实际上：体素过大时，平面/边缘等几何特征被抹平，point-to-plane 的法向量估计不准，配准精度断崖下跌。
> 正确理解：降采样是"速度—精度"权衡，不是单调最优。地面机器人 0.3-0.5 m 是常见甜点；高速场景或退化环境要更精细。第51章会教你如何用评估指标找到这个甜点。

### 练习

1. **（配置练习）** 把 `voxel_leaf_size` 从 0.3 改到 0.1 和 0.5，分别记录输出点数和单帧预处理耗时，画出"点数 vs leaf_size""耗时 vs leaf_size"两条曲线，找出你机器上的拐点。
2. **（管线搭建）** 实现一个"保留 time 字段的体素降采样"：对每个体素，不取质心，而取距质心最近的那个原始点，从而保留其真实 `time` 和 `ring`。验证降采样后 `time` 字段仍落在 $[0, 0.1]$。
3. **（鲁棒性）** 给 `process` 加一个空帧/全 NaN 帧的防御：当输入为空或全部被过滤掉时，返回什么？下游模块如何感知"这帧没有有效点"并优雅降级？

---

## §50.3 IMU 积分与预积分 ⭐⭐⭐

### 模块功能与在管线中的位置

这是流水线上承上启下的关键工位。它消费 IMU 流，产出两样东西：(1) **名义状态的预测先验**（位置/速度/旋转），供去畸变（§50.4）和 iESKF 更新（§50.5）使用；(2) **误差状态的协方差 $\boldsymbol P$ 的传播**，告诉滤波器"这个预测有多不确定"。

### 动机：IMU 给我们什么，又欠我们什么

IMU 每秒输出 200 次"此刻的角速度 $\boldsymbol\omega_m$ 和比力 $\boldsymbol a_m$"。但它给的是**瞬时微分量**，我们要的是**位姿**——中间隔着两次积分。反面问题：**直接对 IMU 二次积分定位行不行？**

不行。原因有三，每一条都是 LIO 要靠 LiDAR 修正的理由：

1. **偏置（bias）漂移**：陀螺和加速度计都有缓慢漂移的零偏 $\boldsymbol b_g, \boldsymbol b_a$。积分会把偏置误差也积起来，位置误差随 $t^2$ 发散。
2. **重力耦合**：加速度计测的是"比力"$\boldsymbol a_m = \boldsymbol R^\top(\boldsymbol a - \boldsymbol g) + \boldsymbol b_a + \boldsymbol n_a$，里面混着重力 $\boldsymbol g$。要拿到运动加速度必须先精确知道姿态去扣除重力——姿态稍有误差，重力就漏进加速度，灾难性放大。
3. **噪声**：高频噪声 $\boldsymbol n_g, \boldsymbol n_a$ 积分后变成随机游走。

> **本质洞察**：IMU 是"快但会漂"的传感器，LiDAR 是"慢但准"的传感器。§50.3 的全部工作，就是把 IMU 这点"快"用好——在两帧 LiDAR 之间的 100 ms 里，提供一个足够好的位姿预测，好到能让去畸变和配准的迭代从一个靠谱的初值出发。预测的"漂"由后面的 LiDAR 更新来纠正。

### 历史：从 IMU 积分到预积分

- **朴素 IMU 积分**：每来一个 IMU 就更新一次状态。简单，但若后端要重新线性化（如图优化里位姿被调整），所有积分得重算——昂贵。
- **IMU 预积分（Preintegration，Forster 等 2015）**：把两个关键帧之间的 IMU 测量预先积分成一个"相对运动约束"，且与起始位姿无关，后端调整位姿时无需重积分。LIO-SAM 的因子图就用预积分因子。
- **本章选择**：滤波派（FAST-LIO 风格）的 iESKF 中，我们用**逐帧中值积分**做前向传播（§50.3 主线），它本质是预积分的"在线增量版本"。§50.8 接 GTSAM 时再用 Forster 预积分因子。两者数学根基相同。

### 理论：状态定义

先定义 Mini-LIO 的名义状态。这是整个系统的"主角"，后面所有模块都围着它转：

$$
\boldsymbol x = \begin{bmatrix} \boldsymbol p & \boldsymbol v & \boldsymbol R & \boldsymbol b_g & \boldsymbol b_a & \boldsymbol g \end{bmatrix}
$$

各分量含义与维度：

| 分量 | 含义 | 流形/空间 | 维度 |
|------|------|----------|------|
| $\boldsymbol p$ | 位置（世界系） | $\mathbb R^3$ | 3 |
| $\boldsymbol v$ | 速度（世界系） | $\mathbb R^3$ | 3 |
| $\boldsymbol R$ | 姿态（IMU 到世界） | $SO(3)$ | 3（流形自由度） |
| $\boldsymbol b_g$ | 陀螺偏置 | $\mathbb R^3$ | 3 |
| $\boldsymbol b_a$ | 加速度计偏置 | $\mathbb R^3$ | 3 |
| $\boldsymbol g$ | 重力向量（世界系） | $\mathbb R^3$（或 $S^2$） | 3（或 2） |

注意 $\boldsymbol R$ 在流形 $SO(3)$ 上，其余在欧氏空间。这是为什么不能对整个状态做普通加法、必须用 $\boxplus$ 的根源（回顾前置桥接）。把重力 $\boldsymbol g$ 也放进状态来在线估计，是 FAST-LIO 的做法——免去精确的初始水平标定。误差状态对应地是 18 维向量（重力用 $\mathbb R^3$ 表示时）：

$$
\delta\boldsymbol x = \begin{bmatrix} \delta\boldsymbol p & \delta\boldsymbol v & \delta\boldsymbol\theta & \delta\boldsymbol b_g & \delta\boldsymbol b_a & \delta\boldsymbol g \end{bmatrix} \in \mathbb R^{18}
$$

唯一特殊的是旋转误差 $\delta\boldsymbol\theta\in\mathbb R^3$（切空间），通过 $\boldsymbol R_{\text{true}} = \boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)$ 关联回流形。

代码里把状态实现为一个结构体，旋转用 Sophus 的 `SO3d`：

```cpp
// include/mini_lio/common.hpp  ——  名义状态
#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace mini_lio {

struct State {
  Eigen::Vector3d  pos  = Eigen::Vector3d::Zero();   // p
  Eigen::Vector3d  vel  = Eigen::Vector3d::Zero();   // v
  Sophus::SO3d     rot;                               // R （默认单位旋转）
  Eigen::Vector3d  bg   = Eigen::Vector3d::Zero();   // 陀螺偏置
  Eigen::Vector3d  ba   = Eigen::Vector3d::Zero();   // 加表偏置
  Eigen::Vector3d  grav = Eigen::Vector3d(0,0,-9.81);// 重力向量
  double           time = 0;                          // 该状态对应的时间戳
};

struct IMUData {
  double          stamp;                              // 时间戳（秒）
  Eigen::Vector3d acc;                                // 比力测量 a_m
  Eigen::Vector3d gyro;                               // 角速度测量 ω_m
};

}  // namespace mini_lio
```

### 理论：名义状态的中值积分

给定上一时刻状态 $\boldsymbol x_{k}$ 和当前 IMU 测量，要推进 $\Delta t$ 得到 $\boldsymbol x_{k+1}$。**中值积分（midpoint）**用区间两端测量的平均值，比只用左端的欧拉积分精度高一阶。先扣除偏置得到"真实"角速度和加速度（世界系）：

$$
\hat{\boldsymbol\omega} = \tfrac{1}{2}\big((\boldsymbol\omega_m^{k}-\boldsymbol b_g)+(\boldsymbol\omega_m^{k+1}-\boldsymbol b_g)\big), \qquad
$$

旋转先更新（用中值角速度）：

$$
\boldsymbol R_{k+1} = \boldsymbol R_k \, \mathrm{Exp}(\hat{\boldsymbol\omega}\,\Delta t)
$$

加速度用旋转前后的平均，并扣除重力（注意 $\boldsymbol g$ 在我们的约定里是 $\approx(0,0,-9.81)$，所以是 $+\boldsymbol g$）：

$$
\hat{\boldsymbol a} = \tfrac{1}{2}\big(\boldsymbol R_k(\boldsymbol a_m^{k}-\boldsymbol b_a) + \boldsymbol R_{k+1}(\boldsymbol a_m^{k+1}-\boldsymbol b_a)\big) + \boldsymbol g
$$

速度与位置（先更新速度，位置用中值速度）：

$$
\boldsymbol v_{k+1} = \boldsymbol v_k + \hat{\boldsymbol a}\,\Delta t, \qquad
\boldsymbol p_{k+1} = \boldsymbol p_k + \tfrac{1}{2}(\boldsymbol v_k+\boldsymbol v_{k+1})\,\Delta t
$$

偏置和重力在传播中视为常量（它们的变化由后面的 LiDAR 更新和随机游走模型处理）。

> **阶段小结**：到这里我们完成了"名义状态怎么往前推"。但卡尔曼滤波还需要知道"这个预测有多不确定"——也就是误差状态协方差 $\boldsymbol P$ 怎么传播。接下来推导误差状态的离散传播。

### 理论：误差状态协方差传播

误差状态的连续时间动力学线性化后，离散化为：

$$
\delta\boldsymbol x_{k+1} = \boldsymbol F_x\,\delta\boldsymbol x_k + \boldsymbol F_w\,\boldsymbol w
$$

其中 $\boldsymbol F_x$（$18\times18$）是误差状态转移矩阵，$\boldsymbol F_w$（$18\times12$）把过程噪声 $\boldsymbol w=[\boldsymbol n_g, \boldsymbol n_a, \boldsymbol n_{bg}, \boldsymbol n_{ba}]$ 映射到误差状态，$\boldsymbol Q$ 是噪声协方差。协方差按下式传播：

$$
\boldsymbol P_{k+1} = \boldsymbol F_x\,\boldsymbol P_k\,\boldsymbol F_x^\top + \boldsymbol F_w\,\boldsymbol Q\,\boldsymbol F_w^\top
$$

$\boldsymbol F_x$ 的关键非平凡块（其余为单位阵或零）：

| 块（行→列） | 表达式 | 物理含义 |
|------------|--------|---------|
| $\partial\delta\boldsymbol p / \partial\delta\boldsymbol v$ | $\boldsymbol I\,\Delta t$ | 速度误差积分进位置 |
| $\partial\delta\boldsymbol v / \partial\delta\boldsymbol\theta$ | $-\boldsymbol R(\boldsymbol a_m-\boldsymbol b_a)^\wedge\,\Delta t$ | 姿态误差→加速度方向误差→速度误差 |
| $\partial\delta\boldsymbol v / \partial\delta\boldsymbol b_a$ | $-\boldsymbol R\,\Delta t$ | 加表偏置误差→速度误差 |
| $\partial\delta\boldsymbol v / \partial\delta\boldsymbol g$ | $\boldsymbol I\,\Delta t$ | 重力误差→速度误差 |
| $\partial\delta\boldsymbol\theta / \partial\delta\boldsymbol\theta$ | $\mathrm{Exp}(-\hat{\boldsymbol\omega}\Delta t)$ | 旋转误差自身的流形传播 |
| $\partial\delta\boldsymbol\theta / \partial\delta\boldsymbol b_g$ | $-\boldsymbol J_r(\hat{\boldsymbol\omega}\Delta t)\,\Delta t$ | 陀螺偏置误差→姿态误差 |

其中 $(\cdot)^\wedge$ 是反对称矩阵算子，$\boldsymbol J_r$ 是 $SO(3)$ 的右雅可比。这些块来自对中值积分公式逐项求误差微分——细节可参考《Quaternion kinematics for the error-state KF》（Solà），本章给出结果与代码。

### 代码走读：IMU 前向传播

```cpp
// src/imu_integration.cpp  ——  名义状态中值积分 + 误差协方差传播
#include "mini_lio/imu_integration.hpp"

namespace mini_lio {

// 单步：用一对相邻 IMU（或上一帧缓存的 last_imu_ 与当前 imu）推进 dt
void imu_predict(State& x, const IMUData& imu, double dt) {
  // —— 名义状态中值积分（这里用单测量近似，完整中值见 propagate 重载）——
  const Eigen::Vector3d w = imu.gyro - x.bg;            // 去偏置角速度
  const Eigen::Vector3d a = imu.acc  - x.ba;            // 去偏置比力
  const Sophus::SO3d    R0 = x.rot;
  x.rot = R0 * Sophus::SO3d::exp(w * dt);               // R ⊞ (ω dt)
  const Eigen::Vector3d acc_world = R0 * a + x.grav;    // 转世界系并加重力
  const Eigen::Vector3d v0 = x.vel;
  x.vel += acc_world * dt;                              // v 更新
  x.pos += 0.5 * (v0 + x.vel) * dt;                     // p 用中值速度
  x.time = imu.stamp;
}

// 带协方差传播的完整版本：返回更新后状态并就地传播 P
void IMUIntegrator::propagate(State& x, Eigen::Matrix<double,18,18>& P,
                              const IMUData& imu, double dt) {
  const Eigen::Vector3d w = imu.gyro - x.bg;
  const Eigen::Vector3d a = imu.acc  - x.ba;
  const Sophus::SO3d    R0 = x.rot;

  // 构造误差状态转移矩阵 Fx（18x18），索引约定：
  //  [0:3]=δp [3:6]=δv [6:9]=δθ [9:12]=δbg [12:15]=δba [15:18]=δg
  Eigen::Matrix<double,18,18> Fx = Eigen::Matrix<double,18,18>::Identity();
  Fx.block<3,3>(0,3)  = Eigen::Matrix3d::Identity() * dt;          // δp ← δv
  Fx.block<3,3>(3,6)  = -R0.matrix() * Sophus::SO3d::hat(a) * dt;  // δv ← δθ
  Fx.block<3,3>(3,12) = -R0.matrix() * dt;                         // δv ← δba
  Fx.block<3,3>(3,15) = Eigen::Matrix3d::Identity() * dt;          // δv ← δg
  Fx.block<3,3>(6,6)  = Sophus::SO3d::exp(-w * dt).matrix();       // δθ ← δθ
  Fx.block<3,3>(6,9)  = -right_jacobian(w * dt) * dt;              // δθ ← δbg

  // 过程噪声映射 Fw（18x12）与噪声协方差 Q（12x12，来自 IMU 标定）
  Eigen::Matrix<double,18,12> Fw = Eigen::Matrix<double,18,12>::Zero();
  Fw.block<3,3>(6,0)   = -right_jacobian(w * dt) * dt;  // 陀螺白噪声 → δθ
  Fw.block<3,3>(3,3)   = -R0.matrix() * dt;             // 加表白噪声 → δv
  Fw.block<3,3>(9,6)   = Eigen::Matrix3d::Identity() * dt;  // 陀螺偏置游走
  Fw.block<3,3>(12,9)  = Eigen::Matrix3d::Identity() * dt;  // 加表偏置游走

  // —— 先传播协方差（必须用传播前的状态算的 Fx），再更新名义状态 ——
  P = Fx * P * Fx.transpose() + Fw * Q_ * Fw.transpose();

  // 名义状态推进（同 imu_predict）
  x.rot = R0 * Sophus::SO3d::exp(w * dt);
  const Eigen::Vector3d v0 = x.vel;
  x.vel += (R0 * a + x.grav) * dt;
  x.pos += 0.5 * (v0 + x.vel) * dt;
  x.time = imu.stamp;
}

}  // namespace mini_lio
```

逐行强调几个"为什么这样写、改了会怎样"：

- **协方差先于名义状态更新**：`Fx` 是在"传播前的 $\boldsymbol R_0, \boldsymbol a$"上构造的，必须在改 `x.rot` 之前算完 `P`。**顺序写反，雅可比就用了未来的状态，协方差传播错误**——这是新手最常见、最难查的 bug 之一。
- **`Sophus::SO3d::hat(a)`**：把向量变反对称矩阵 $(\cdot)^\wedge$。对应 $\partial\delta\boldsymbol v/\partial\delta\boldsymbol\theta$ 那一项。
- **`Q_` 来自标定**：陀螺/加表的白噪声和偏置随机游走方差，必须用真实 IMU 的 datasheet 或 Allan 方差标定填进 `config/params.yaml`，乱填会让滤波器"过度自信"或"过度怀疑"IMU。

### 配置详解：IMU 噪声参数

```yaml
imu:
  acc_noise:       0.01      # 加速度计白噪声 std (m/s²/√Hz)
  gyro_noise:      0.001     # 陀螺白噪声 std (rad/s/√Hz)
  acc_bias_noise:  0.0001    # 加表偏置随机游走 (m/s³/√Hz)
  gyro_bias_noise: 0.00001   # 陀螺偏置随机游走 (rad/s²/√Hz)
  init_static_sec: 1.0       # 启动时静止采样多久来初始化偏置和重力方向
```

> **本质洞察**：这四个噪声参数本质上是在告诉滤波器"该多相信 IMU"。`*_noise` 越大，$\boldsymbol Q$ 越大，预测协方差 $\boldsymbol P$ 增长越快，滤波器在更新时就越偏向 LiDAR 观测。理解了这一点，你就知道调参时如果"LIO 不信 LiDAR、轨迹跟着 IMU 漂"，该调小 IMU 噪声；反之调大。

### 运行验证

- **静止测试**：把 IMU 静止放置，喂给 `propagate`。位置应几乎不动（< 几厘米/分钟），速度应接近 0。若位置飞快漂移，多半是重力符号或偏置初始化错了。
- **协方差单调性**：纯预测阶段（无更新），$\boldsymbol P$ 的对角元应单调增长（不确定性只增不减）。若 $\boldsymbol P$ 出现负对角元或骤降，是 $\boldsymbol F_x$ 构造或数值问题。

### ⚠️ 常见陷阱

> **编程陷阱：重力符号与坐标系约定不一致**
> 错误做法：状态里 `grav=(0,0,-9.81)`，积分公式却写成 `acc_world = R*a - grav`。
> 现象：静止时位置以约 $9.8t^2$ 的速度"上天"或"入地"。
> 根本原因：比力 $\boldsymbol a_m = \boldsymbol R^\top(\boldsymbol a-\boldsymbol g)+\boldsymbol b_a$，解出运动加速度 $\boldsymbol a = \boldsymbol R\boldsymbol a_m + \boldsymbol g$。重力向下时 $\boldsymbol g_z<0$，所以是 $+\boldsymbol g$。符号写反，重力被加倍而非抵消。
> 正确做法：固定一套约定（本章：$\boldsymbol g\approx(0,0,-9.81)$，公式用 $+\boldsymbol g$），写静止单元测试断言位置漂移 ≈ 0。

> **编程陷阱：用欧拉积分代替中值积分还期望高精度**
> 错误做法：偷懒只用区间左端的测量（欧拉法）积分。
> 现象：旋转较快时姿态有可观的系统性偏差，长时间漂移明显。
> 根本原因：欧拉积分是一阶的，在角速度变化时有 $O(\Delta t)$ 误差；中值积分是二阶 $O(\Delta t^2)$。
> 正确做法：用中值积分（两端测量平均）。代码里要缓存上一个 IMU 测量 `last_imu_` 与当前测量配对。

> **概念误区：以为偏置是固定常数，标定一次就行**
> 新手想法："IMU 偏置不就是个固定的零点偏移，开机标一次填进去。"
> 实际上：偏置随温度、时间缓慢漂移（这就是为什么状态里要带 $\boldsymbol b_g,\boldsymbol b_a$ 并持续在线估计）。开机标的只是初值。
> 正确理解：偏置是状态的一部分，由 LiDAR 更新持续修正；过程噪声里的"偏置随机游走"项就是建模这种缓慢漂移。

### 练习

1. **（手推 + 验证）** 自己推导 $\partial\delta\boldsymbol v/\partial\delta\boldsymbol\theta = -\boldsymbol R(\boldsymbol a_m-\boldsymbol b_a)^\wedge\Delta t$ 这一项（提示：对 $\boldsymbol v_{k+1}=\boldsymbol v_k + (\boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)(\boldsymbol a_m-\boldsymbol b_a)+\boldsymbol g)\Delta t$ 关于 $\delta\boldsymbol\theta$ 在 0 处求导，用 $\mathrm{Exp}(\delta\boldsymbol\theta)\approx\boldsymbol I+\delta\boldsymbol\theta^\wedge$）。推完写一个数值雅可比对比的单元测试。
2. **（配置练习）** 把 `gyro_noise` 放大 10 倍和缩小 10 倍，在模拟数据上观察预测协方差 $\boldsymbol P$ 中姿态块的增长速度差异，并解释它将如何影响后续 LiDAR 更新时的"信任分配"。
3. **（管线搭建）** 实现启动时的静态初始化：采集 `init_static_sec` 秒的静止 IMU，用平均角速度估计 $\boldsymbol b_g$，用平均加速度方向估计初始姿态和重力方向。验证初始化后 `propagate` 静止漂移最小。

---

## §50.4 点云去畸变：反向传播 ⭐⭐⭐

### 模块功能与在管线中的位置

去畸变（motion compensation / de-skew）消费两样东西：预处理后的点云（带每点 `time`）和 §50.3 在该帧区间内传播出的一串 IMU 位姿。它产出一帧"仿佛在同一瞬间采集"的点云，供 §50.5 配准。这一节是 §50.3 和 §50.5 之间的桥。

### 动机：一帧点云不是"一个瞬间"拍下来的

这是激光 SLAM 区别于视觉 SLAM 的一个根本事实。反面思考：**为什么相机不需要去畸变，而 LiDAR 需要？**

相机（全局快门）一帧是某个瞬间整幅曝光。而机械式 LiDAR 一帧是激光头**旋转一整圈**扫出来的——一圈 100 ms（10 Hz）。这 100 ms 里如果载体在动，先扫到的点和后扫到的点是在**不同位姿**下采集的。把它们当成同一时刻的点云直接配准，等于把"运动"误当成"场景形状"。

举个数字：车以 10 m/s 直行，一帧 100 ms，车头扫到第一个点时车在 A，扫到最后一个点时车已前进 1 米。这 1 米的位移会让点云"拖尾"，平面被扭成曲面。固态 LiDAR（如 Livox）虽是非旋转，但仍有帧内时间跨度，快速旋转时畸变同样显著。

> **本质洞察**：去畸变的本质是一次"时间对齐"——把帧内每个点从"它各自被采集的时刻"投影到"统一参考时刻"（通常取帧末 $t_k$）。要做这个投影，必须知道每个时刻的位姿——这正是 §50.3 的 IMU 传播提供的。**没有 IMU，去畸变就没有位姿可用**；这也解释了为什么纯 LiDAR SLAM（如早期 LOAM）只能用恒速假设近似去畸变，精度不如 LIO。

### 反面：恒速假设 vs IMU 反向传播

历史上有两种去畸变法：

| 方法 | 做法 | 优缺点 |
|------|------|--------|
| 恒速模型（LOAM） | 假设帧内匀速运动，用上一帧速度线性插值每点位姿 | 无需 IMU；但加减速/急转时假设失效 |
| IMU 反向传播（FAST-LIO） | 用帧内真实的高频 IMU 积分位姿去补偿每个点 | 精度高，能处理任意运动；需 IMU 且要传播位姿 |

Mini-LIO 用 IMU 反向传播——这是"LiDAR-**Inertial**"里 inertial 发挥作用的一个直接体现。

### 理论：反向传播去畸变的数学

设帧末时刻为 $t_k$，对应位姿 $\boldsymbol T_k=(\boldsymbol R_k,\boldsymbol p_k)$（IMU 系到世界系）。一个点 $\boldsymbol q_i$ 在帧内 $t_i$（$t_i\le t_k$）采集，此时位姿为 $\boldsymbol T_i=(\boldsymbol R_i,\boldsymbol p_i)$。点 $\boldsymbol q_i$ 是在传感器系（LiDAR 系）下测得的，记 $\boldsymbol T_{IL}$ 为 LiDAR 到 IMU 的外参。

把点投影到帧末时刻的 LiDAR 系，等价于"先把 $t_i$ 时刻的点变换到世界系，再变回 $t_k$ 时刻的 LiDAR 系"：

$$
\boldsymbol q_i^{\text{deskew}} = \boldsymbol T_{IL}^{-1}\,\boldsymbol T_k^{-1}\,\boldsymbol T_i\,\boldsymbol T_{IL}\,\boldsymbol q_i
$$

逐段读这个链式变换（从右往左）：

1. $\boldsymbol T_{IL}\,\boldsymbol q_i$：点从 LiDAR 系到 $t_i$ 时刻的 IMU 系。
2. $\boldsymbol T_i\,(\cdot)$：从 $t_i$ 的 IMU 系到世界系——此时点被钉在它真实采集时的世界位置。
3. $\boldsymbol T_k^{-1}\,(\cdot)$：从世界系到 $t_k$ 时刻的 IMU 系——把点"搬"到帧末位姿下看。
4. $\boldsymbol T_{IL}^{-1}\,(\cdot)$：回到 LiDAR 系。

核心是中间的 $\boldsymbol T_k^{-1}\boldsymbol T_i$——它是"$t_i$ 到 $t_k$ 的相对运动"。如果载体没动，$\boldsymbol T_i=\boldsymbol T_k$，这一项是单位阵，去畸变不改变点（符合直觉：静止无需去畸变）。

> **多视角理解（双重解读）**：可以从两个角度理解去畸变。**几何角度**：把散落在不同位姿下的点统一搬到同一个观察位姿，消除运动拖影。**信号角度**：点云是位姿这个"载波"对环境的采样，运动是混入的"调制"，去畸变是解调，把环境信号从运动里剥离。两种角度指向同一个公式。

### 关键工程问题：每个点的位姿从哪来

帧内有上万个点、每个点 $t_i$ 不同，但 IMU 只有约 20 个测量。不可能为每个点单独积分。实现技巧（FAST-LIO 的做法）：

1. **正向传播时缓存位姿**：§50.3 前向传播每个 IMU 步时，把该 IMU 时刻的 $(\,t,\boldsymbol R,\boldsymbol p,\boldsymbol v\,)$ 存进一个 `std::vector<IMUPose>`。
2. **按点的时间戳查表 + 插值**：去畸变时，对每个点的 $t_i$，在缓存里找到相邻的两个 IMU 位姿，旋转用 $\mathrm{Exp}$ 插值、位置线性插值，得到 $\boldsymbol T_i$。

### 代码走读：反向传播去畸变

```cpp
// src/imu_integration.cpp（续）  ——  反向传播去畸变
namespace mini_lio {

// 帧内缓存的 IMU 位姿（由 §50.3 前向传播填充）
struct IMUPose {
  double          offset_t;   // 相对帧首的时间（秒）
  Sophus::SO3d    R;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
};

void undistort(const std::vector<IMUPose>& poses,   // 帧内 IMU 位姿序列（时间升序）
               const Sophus::SE3d& T_IL,            // LiDAR→IMU 外参
               CloudPtr& cloud) {                    // 就地去畸变
  const auto& end = poses.back();                    // 帧末位姿 T_k
  const Sophus::SE3d T_k(end.R, end.p);
  const Sophus::SE3d T_k_inv = T_k.inverse();
  const Sophus::SE3d T_IL_inv = T_IL.inverse();

  size_t j = poses.size() - 1;
  // 点按 time 降序遍历（从帧末往帧首），与 IMU 位姿指针同步回退 → 反向传播
  for (auto& pt : cloud->points) {
    const double ts = pt.time;                       // 该点相对帧首时间
    // 找到 poses[j-1].offset_t <= ts < poses[j].offset_t 的区间
    while (j > 0 && poses[j-1].offset_t > ts) --j;
    const IMUPose& a = poses[j-1];
    const IMUPose& b = poses[j];
    const double alpha = (ts - a.offset_t) / std::max(1e-9, b.offset_t - a.offset_t);

    // 插值出 t_i 时刻位姿 T_i：旋转用 SO3 指数插值，位置线性插值
    const Sophus::SO3d   R_i = a.R * Sophus::SO3d::exp(alpha * (a.R.inverse() * b.R).log());
    const Eigen::Vector3d p_i = a.p + alpha * (b.p - a.p);
    const Sophus::SE3d   T_i(R_i, p_i);

    // 应用去畸变变换链：q_deskew = T_IL^-1 · T_k^-1 · T_i · T_IL · q
    const Eigen::Vector3d q(pt.x, pt.y, pt.z);
    const Eigen::Vector3d q_deskew = T_IL_inv * (T_k_inv * (T_i * (T_IL * q)));
    pt.x = q_deskew.x();  pt.y = q_deskew.y();  pt.z = q_deskew.z();
  }
}

}  // namespace mini_lio
```

要点：

- **"反向"二字的来历**：遍历从帧末（time 最大）往帧首走，IMU 位姿指针 `j` 随之单调回退。因为正向传播把状态推到了帧末，去畸变是从帧末"倒推"每个点该往回搬多少——这就是 FAST-LIO 论文里 "backward propagation" 的字面含义。
- **旋转插值用 `exp(alpha * log(...))`**：这是 SO(3) 上的球面线性插值（slerp 的李群写法），保证插值出的旋转仍在流形上。直接对旋转矩阵线性插值会跑出 $SO(3)$。
- **外参 $\boldsymbol T_{IL}$ 不可省**：即使 IMU 和 LiDAR 靠得很近，旋转时杆臂效应也会让点位移，外参错误使去畸变引入新畸变。

### 运行验证

- **静止帧不变**：喂一帧静止采集的点云，去畸变前后点坐标应几乎不变（相对运动是单位阵）。
- **运动帧拉直**：在一面平直墙前匀速直行采集，去畸变前墙面点云"歪斜/拖尾"，去畸变后应明显变平直。这是最直观的肉眼验收。

### ⚠️ 常见陷阱

> **编程陷阱：点的时间戳与 IMU 位姿时间基准不一致**
> 错误做法：点的 `time` 是"相对帧首"，IMU 位姿缓存的 `offset_t` 却存成了"绝对时间戳"。
> 现象：去畸变后点云严重错位，配准发散。
> 根本原因：两个时间不在同一基准上，插值时 alpha 算成乱七八糟的值。
> 正确做法：统一所有时间为"相对帧首的秒"，在去畸变入口断言点的 time 与位姿的 offset_t 在同一区间。

> **编程陷阱：对旋转直接线性插值**
> 错误做法：`R_i = (1-alpha)*a.R.matrix() + alpha*b.R.matrix()`。
> 现象：插值出的"旋转矩阵"不再正交，点云轻微扭曲，长期累积漂移。
> 根本原因：$SO(3)$ 不是凸集，两个旋转矩阵的线性组合不是旋转矩阵（回顾：$\boldsymbol R_1+\boldsymbol R_2\notin SO(3)$）。
> 正确做法：在李代数上插值，`a.R * Exp(alpha * Log(a.R⁻¹ b.R))`。

> **概念误区：以为可以"先配准拿到位姿再去畸变"**
> 新手想法："反正最后要算位姿，先配准出位姿再用它去畸变不行吗？"
> 实际上：配准需要去畸变后的点云作为输入（畸变点云配准本身就不准），存在循环依赖。
> 正确理解：去畸变用的是 IMU **预测**的位姿（§50.3），不依赖配准结果。这是个先有鸡（IMU 预测）才有蛋（配准）的单向流程，不能颠倒。FAST-LIO 在 iESKF 迭代里会用更新后的位姿，但首次去畸变必须靠 IMU 预测。

### 练习

1. **（推导）** 证明：当载体在帧内静止（所有 $\boldsymbol T_i=\boldsymbol T_k$）时，去畸变变换链退化为恒等变换。再分析：仅有纯平移（无旋转）时，去畸变对点云做了什么？仅有纯旋转时呢？
2. **（管线搭建）** 实现 IMU 位姿缓存：修改 §50.3 的前向传播，每步把 `IMUPose` 压入 `vector`，确保 `offset_t` 用"相对帧首"。然后接上本节 `undistort`，在模拟匀速直行数据上验证墙面被拉直。
3. **（鲁棒性）** 如果某个点的 `time` 超出了 IMU 位姿缓存的时间范围（如 time > 帧末，或 IMU 丢包导致缓存稀疏），你的 `undistort` 会怎样？设计边界保护（外推 vs 钳制 vs 丢弃该点），并说明各自的代价。

---

## §50.5 iESKF 状态估计 ⭐⭐⭐⭐

### 模块功能与在管线中的位置

这是 Mini-LIO 的心脏，也是本章最难的一节（⭐⭐⭐⭐ 研究级）。它把 §50.3 的预测先验、§50.7 的 point-to-plane 残差、§50.6 的地图查询拧在一起，迭代求解出当前帧的最优位姿与状态。理解了它，你就理解了现代滤波派 LIO 的全部精髓。

### 动机：为什么是"迭代"的"误差状态"卡尔曼滤波

名字里三个修饰词，每一个都对应一个反面问题，逐个拆解：

**为什么用"误差状态"而非直接 EKF？**
直接对名义状态（含旋转 $\boldsymbol R$）做 EKF，会遇到"旋转不在向量空间、无法直接线性化和加协方差"的麻烦。误差状态 KF 把问题搬到切空间：名义状态 $\boldsymbol x$ 用确定性积分传播（§50.3 已做），误差 $\delta\boldsymbol x\in\mathbb R^{18}$ 是小量、近似线性，在切空间里做标准卡尔曼。两者用 $\boldsymbol x_{\text{true}}=\boldsymbol x\boxplus\delta\boldsymbol x$ 缝合。**误差状态永远在 0 附近，线性化误差小**，这是它比直接 EKF 稳定的根本原因。

**为什么要"迭代"？**
LiDAR 的 point-to-plane 观测是高度非线性的（残差关于位姿非线性），且最近邻关系本身依赖位姿——位姿变了，每个点对应的地图平面就变了。一次线性化更新不够准。迭代的思路（借鉴高斯—牛顿）：更新一次位姿 → 用新位姿重新找最近邻、重新线性化 → 再更新，直到收敛。这就是 **Iterated** EKF（iEKF）。

> **多视角理解（类比）**：iESKF ≈ "卡尔曼滤波的外壳 + 高斯-牛顿的内核"。**像高斯-牛顿的地方**：每次迭代重新线性化、重新求增量、迭代到收敛。**像卡尔曼的地方**：增量的求解不是纯最小二乘，而是带着"预测先验 $\boldsymbol P$"作为正则项的最大后验估计——既相信观测，也不抛弃 IMU 预测。**不像纯高斯-牛顿的地方**：纯 GN 没有先验，iESKF 把 IMU 预测当先验融了进来，退化场景下先验救场。

### 历史：从 EKF 到 iEKF on manifold

- EKF：一次线性化，无迭代。强非线性时精度差。
- IEKF（Iterated EKF）：观测更新内部迭代重线性化，等价于在观测模型上做高斯-牛顿。
- on-manifold IEKF（FAST-LIO 的 iESKF）：把 IEKF 搬到流形上，用 $\boxplus/\boxminus$ 处理 $SO(3)$，并配合 IKFOM 工具箱的通用流形 KF 框架。Mini-LIO 实现其简化版。

### 理论：观测模型与残差

§50.7 会详细推 point-to-plane 残差，这里先给 iESKF 需要的接口形式。对第 $i$ 个去畸变点 $\boldsymbol q_i$，把它用当前状态变换到世界系 $\boldsymbol p_i^W = \boldsymbol R\,(\boldsymbol T_{IL}\boldsymbol q_i)+\boldsymbol p$，在地图里找到它所属平面（法向 $\boldsymbol n_i$、过点 $\boldsymbol c_i$）。残差是点到平面的有符号距离：

$$
r_i = \boldsymbol n_i^\top (\boldsymbol p_i^W - \boldsymbol c_i)
$$

把所有点的残差堆成向量 $\boldsymbol z=[r_1,\dots,r_m]^\top$，观测雅可比 $\boldsymbol H=\partial\boldsymbol z/\partial\delta\boldsymbol x$（$m\times18$），观测噪声 $\boldsymbol R_{\text{obs}}$（点到面测量噪声，对角）。

### 理论：迭代更新方程

设当前迭代第 $\kappa$ 次的状态估计 $\boldsymbol x_\kappa$，预测先验为 $\hat{\boldsymbol x}$（§50.3 的传播结果）、先验协方差 $\hat{\boldsymbol P}$。两个状态之间的差用 $\boxminus$ 表达。iESKF 每次迭代求解如下最大后验问题（先验项 + 观测项）：

$$
\min_{\delta\boldsymbol x}\ \underbrace{\|\hat{\boldsymbol x}\boxminus(\boldsymbol x_\kappa\boxplus\delta\boldsymbol x)\|^2_{\hat{\boldsymbol P}^{-1}}}_{\text{相信 IMU 预测}} + \underbrace{\|\boldsymbol z_\kappa + \boldsymbol H\delta\boldsymbol x\|^2_{\boldsymbol R_{\text{obs}}^{-1}}}_{\text{相信 LiDAR 观测}}
$$

其解给出卡尔曼增益与更新。标准卡尔曼增益是：

$$
\boldsymbol K = \hat{\boldsymbol P}\boldsymbol H^\top(\boldsymbol H\hat{\boldsymbol P}\boldsymbol H^\top + \boldsymbol R_{\text{obs}})^{-1}
$$

但这里有个**计算瓶颈**：观测数 $m$ 是几千（每帧几千个点都有残差），$\boldsymbol H\hat{\boldsymbol P}\boldsymbol H^\top+\boldsymbol R_{\text{obs}}$ 是 $m\times m$，对它求逆是 $O(m^3)$，每帧几千维求逆——实时性崩溃。

> **本质洞察（FAST-LIO 的关键 trick）**：FAST-LIO 用一个数学上等价、但把求逆维度从"观测维 $m$"降到"状态维 $n=18$"的公式。直觉是：信息其实只有 18 个自由度（状态维），$m$ 个观测被压缩进 18 维的信息矩阵里。利用 Sherman-Morrison-Woodbury 恒等式重写，求逆变成 $18\times18$，与观测数无关。**这就是 FAST-LIO 名字里 "FAST" 的核心来源之一**——它让"用几千个点更新"和"用几十个点更新"的求逆代价一样。

降维后的增益（等价形式）：

$$
\boldsymbol K = (\boldsymbol H^\top\boldsymbol R_{\text{obs}}^{-1}\boldsymbol H + \hat{\boldsymbol P}^{-1})^{-1}\boldsymbol H^\top\boldsymbol R_{\text{obs}}^{-1}
$$

这里求逆的是 $18\times18$ 的 $(\boldsymbol H^\top\boldsymbol R^{-1}\boldsymbol H+\hat{\boldsymbol P}^{-1})$，与 $m$ 无关。误差状态更新（注意还有一个把"先验偏移"投影回来的修正项 $\boldsymbol J$，源于流形上 $\boxminus$ 的非线性，简化实现可近似为单位阵）：

$$
\delta\boldsymbol x = -\boldsymbol K\,\boldsymbol z_\kappa - (\boldsymbol I-\boldsymbol K\boldsymbol H)(\boldsymbol x_\kappa\boxminus\hat{\boldsymbol x})
$$

然后把误差注入名义状态，进入下一次迭代：

$$
\boldsymbol x_{\kappa+1} = \boldsymbol x_\kappa\boxplus\delta\boldsymbol x
$$

收敛后（$\|\delta\boldsymbol x\|$ 足够小）更新协方差：$\boldsymbol P = (\boldsymbol I-\boldsymbol K\boldsymbol H)\hat{\boldsymbol P}$。

> **阶段小结**：到这里我们把 iESKF 的数学全摆出来了——观测残差、最大后验目标、降维增益、迭代更新。下面把它落成代码。读代码时对照上面四个公式，每一行都能找到出处。

### 代码走读：iESKF 更新主循环

```cpp
// src/ieskf.cpp  ——  迭代误差状态卡尔曼滤波更新
#include "mini_lio/ieskf.hpp"

namespace mini_lio {

// 输入：预测先验 x_hat / P_hat，去畸变点云，地图（ikd-Tree）
// 输出：收敛后的状态 x 与协方差 P
bool IESKF::update(State& x, Eigen::Matrix<double,18,18>& P,
                   const CloudPtr& cloud, const IKDTree& map) {
  const State x_hat = x;                   // 备份预测先验（boxminus 要用）
  const auto  P_hat = P;
  constexpr int kMaxIter = 5;              // FAST-LIO 通常 3-5 次足够收敛

  for (int iter = 0; iter < kMaxIter; ++iter) {
    // (1) 用当前状态把点变换到世界系，找最近邻、拟合平面、算残差与雅可比
    std::vector<PointPlane> matches;       // 每个有效点：法向 n、平面点 c、残差 r
    build_residuals(x, cloud, map, matches);   // 内部调用 §50.6 搜索 + §50.7 拟合
    const int m = matches.size();
    if (m < 10) return false;              // 有效匹配太少，本帧放弃更新

    // (2) 组装 H (m×18)、残差 z (m×1)；这里用稀疏块只填非零列（pos/rot 相关）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, 18);
    Eigen::VectorXd z(m);
    const double Rinv = 1.0 / obs_noise_;  // 点到面观测噪声（标量，各点同）
    for (int i = 0; i < m; ++i) {
      z(i) = matches[i].residual;
      // ∂r/∂δp = nᵀ ；∂r/∂δθ = -nᵀ R (T_IL q)^∧   （§50.7 推导）
      H.block<1,3>(i,0) = matches[i].normal.transpose();
      H.block<1,3>(i,6) = -matches[i].normal.transpose()
                          * x.rot.matrix() * Sophus::SO3d::hat(matches[i].p_imu);
    }

    // (3) 降维卡尔曼增益：求逆的是 18×18，与 m 无关（FAST-LIO 关键 trick）
    Eigen::Matrix<double,18,18> HtRiH = H.transpose() * Rinv * H;          // 18×18
    Eigen::Matrix<double,18,18> S = HtRiH + P_hat.inverse();               // 18×18
    Eigen::Matrix<double,18,18> S_inv = S.inverse();
    Eigen::MatrixXd K = S_inv * H.transpose() * Rinv;                       // 18×m

    // (4) 误差状态更新：含先验偏移修正项 (x ⊟ x_hat)
    Eigen::Matrix<double,18,1> dx_prior = boxminus(x, x_hat);              // 当前相对先验的偏移
    Eigen::Matrix<double,18,1> dx = -K * z
        - (Eigen::Matrix<double,18,18>::Identity() - K * H) * dx_prior;

    // (5) 注入名义状态：x ← x ⊞ δx
    boxplus_inplace(x, dx);

    // (6) 收敛判据：增量足够小则退出迭代
    if (dx.norm() < 1e-3) break;
  }

  // 收敛后更新协方差
  Eigen::MatrixXd H_final;  // （可缓存最后一次的 H）
  // P = (I - K H) P_hat ；简化实现可用最后一次迭代的 K、H
  // ...（略，见仓库完整实现）
  return true;
}

}  // namespace mini_lio
```

### 代码走读：boxplus 与 boxminus

整个 iESKF 的"加减法"全靠这两个算子。它们是连接"流形上的名义状态"和"切空间里的误差状态"的桥，务必写对：

```cpp
// src/ieskf.cpp（续）  ——  状态流形的 ⊞ 与 ⊟
namespace mini_lio {

// x ⊞ δx：把 18 维切空间增量注入名义状态（旋转走流形，其余直接加）
void boxplus_inplace(State& x, const Eigen::Matrix<double,18,1>& d) {
  x.pos  += d.segment<3>(0);
  x.vel  += d.segment<3>(3);
  x.rot   = x.rot * Sophus::SO3d::exp(d.segment<3>(6));  // R ⊞ δθ = R·Exp(δθ)
  x.bg   += d.segment<3>(9);
  x.ba   += d.segment<3>(12);
  x.grav += d.segment<3>(15);
}

// a ⊟ b：求两个名义状态间的 18 维切空间差（旋转走 Log，其余直接减）
Eigen::Matrix<double,18,1> boxminus(const State& a, const State& b) {
  Eigen::Matrix<double,18,1> d;
  d.segment<3>(0)  = a.pos - b.pos;
  d.segment<3>(3)  = a.vel - b.vel;
  d.segment<3>(6)  = (b.rot.inverse() * a.rot).log();    // δθ = Log(b⁻¹ a)
  d.segment<3>(9)  = a.bg - b.bg;
  d.segment<3>(12) = a.ba - b.ba;
  d.segment<3>(15) = a.grav - b.grav;
  return d;
}

}  // namespace mini_lio
```

把这两段和前置桥接里的 $\boxplus/\boxminus$ 定义对照：旋转分量正是 $\boldsymbol R\boxplus\delta\boldsymbol\theta=\boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)$ 和 $\boldsymbol R_a\boxminus\boldsymbol R_b=\mathrm{Log}(\boldsymbol R_b^\top\boldsymbol R_a)$，其余分量退化为普通加减。

### 配置详解：iESKF 参数

```yaml
ieskf:
  max_iterations: 5         # 单帧最大迭代次数。3-5 常够；多了费时，少了不收敛
  converge_thresh: 0.001    # 增量范数收敛阈值（δx 小于此值停止迭代）
  obs_noise: 0.01           # point-to-plane 观测噪声 std（米），越小越信 LiDAR
  min_matches: 10           # 有效匹配点下限，少于此放弃本帧更新（退化保护）
```

> **本质洞察**：`obs_noise` 和 §50.3 的 IMU 噪声共同决定了"信 LiDAR 还是信 IMU"。`obs_noise` 小 → 信 LiDAR；IMU 噪声小 → 信预测。退化场景（长走廊）里某些方向 $\boldsymbol H$ 秩亏，那个方向的观测约束弱，iESKF 会自动更依赖 IMU 预测——**这种"自动按方向分配信任"的能力，正是紧耦合相对松耦合的核心优势**，也回答了 §50.1 练习 2。

### 运行验证

- **收敛性**：打印每次迭代的 `dx.norm()`，正常应单调下降，3-5 次降到阈值以下。若发散或震荡，多半是 $\boldsymbol H$ 雅可比符号错或 `obs_noise` 离谱。
- **单元测试（GoogleTest）**：构造一个已知的小位移真值，喂合成的平面点云，断言 iESKF 收敛到该真值（误差 < 1 cm / 0.5°）。这是骨架"GoogleTest 覆盖 ESKF 传播/收敛"的核心用例。

### ⚠️ 常见陷阱

> **编程陷阱：boxminus 旋转项左右乘写反**
> 错误做法：`(a.rot.inverse() * b.rot).log()` 或 `(a.rot * b.rot.inverse()).log()`。
> 现象：iESKF 不收敛或收敛到镜像/反向位姿，轨迹方向错乱。
> 根本原因：$\boxplus$ 用右乘 $\boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)$ 定义，$\boxminus$ 必须与之一致，为 $\mathrm{Log}(\boldsymbol R_b^\top\boldsymbol R_a)$（左边是 b 的逆）。左右乘搞反等于换了误差定义，与雅可比不自洽。
> 正确做法：固定一套 $\boxplus/\boxminus$ 约定（本章右乘），雅可比推导与之严格对应，写数值雅可比测试验证自洽。

> **编程陷阱：每帧用 m×m 求逆而非 18×18**
> 错误做法：照搬教科书的 $\boldsymbol K=\hat{\boldsymbol P}\boldsymbol H^\top(\boldsymbol H\hat{\boldsymbol P}\boldsymbol H^\top+\boldsymbol R)^{-1}$，对 $m\times m$ 求逆。
> 现象：点多时单帧耗时爆炸（几千维求逆），实时性全无。
> 根本原因：$m$（观测数）远大于 $n=18$（状态维），$O(m^3)$ 求逆是瓶颈。
> 正确做法：用降维等价公式，求逆 $18\times18$。这是 FAST-LIO 实时性的关键，务必用对。

> **概念误区：以为迭代一次就够（退化成普通 EKF）**
> 新手想法："卡尔曼滤波不就是预测一次、更新一次吗？"
> 实际上：point-to-plane 强非线性 + 最近邻随位姿变化，一次线性化的更新位姿偏差大，最近邻配错。
> 正确理解：迭代的意义是"更新位姿后重新找最近邻、重新线性化"，让点-面对应关系逐步收敛到正确。这是 iEKF 区别于 EKF 的本质，省掉它精度大跌。

### 练习

1. **（推导）** 用 Sherman-Morrison-Woodbury 恒等式，从标准增益 $\boldsymbol K=\hat{\boldsymbol P}\boldsymbol H^\top(\boldsymbol H\hat{\boldsymbol P}\boldsymbol H^\top+\boldsymbol R)^{-1}$ 推出降维形式 $\boldsymbol K=(\boldsymbol H^\top\boldsymbol R^{-1}\boldsymbol H+\hat{\boldsymbol P}^{-1})^{-1}\boldsymbol H^\top\boldsymbol R^{-1}$，并说明为什么两者数学等价但计算量天差地别。
2. **（集成测试）** 写 GoogleTest：给定真值位姿 $\boldsymbol T^*$，把一组共面点用 $\boldsymbol T^*$ 反变换成"观测"，喂给 iESKF（初值设为单位位姿），断言迭代后估计收敛到 $\boldsymbol T^*$。再加噪声，验证收敛仍在合理误差内。
3. **（退化分析）** 构造一个"所有平面法向都平行于 z 轴"的退化场景（只有水平地面），分析 $\boldsymbol H^\top\boldsymbol R^{-1}\boldsymbol H$ 的秩。哪些状态分量不可观？iESKF 在这些方向上的行为是什么？这对应现实中的什么场景？

---

## §50.6 ikd-Tree 局部地图 ⭐⭐⭐

### 模块功能与在管线中的位置

上一节的 iESKF 在每次迭代里都要做一件事：把当前帧的每个点变换到世界系，然后**在地图里找它的最近邻**（`build_residuals` 内部那行注释 "§50.6 搜索"）。这个"地图"就是本节的主角。它的输入是 iESKF 收敛后的点云（已经配准到世界系），输出是"给定一个查询点，返回它在地图中的 $k$ 个最近邻"。地图既要**持续吸收新点**（机器人走到哪、地图长到哪），又要**支持高频最近邻查询**（每帧每点每次迭代都查一次），还要**控制规模**（不能无限膨胀吃光内存）。这三件事同时要快，正是 ikd-Tree 存在的理由。

### 动机：静态 kd-Tree 在增量场景下的两难

前置桥接里我们复活了 kd-Tree（k-dimensional tree）：它把空间沿坐标轴递归二分，最近邻查询平均 $O(\log N)$。但它有个致命的隐含假设——**点集是静态的**。建树时它一次性看到所有点，挑中位数做分割，得到一棵平衡树。一旦建好，往里插点就麻烦了：

- 你可以"硬插"——沿着分割面一路下降到某个叶子挂上去。问题是连续插入会让树越长越偏，退化成链表，查询从 $O(\log N)$ 掉回 $O(N)$。
- 你可以"每帧重建"——把地图所有点拿出来重新建一棵平衡树。但 LIO 地图动辄几十万点，每帧（10 Hz）重建一次，光建树就吃掉所有时间预算。

LIO 的地图恰恰是**持续增长**的：机器人每走一步，新观测的点要并进地图，旧的、远的点要淘汰掉。这就和"kd-Tree 要静态才平衡"的前提直接冲突。FAST-LIO2 的作者 Yixi Cai 等人在 2021 年提出 **ikd-Tree（incremental k-d Tree，增量 kd-树）** 正是为化解这个两难：它允许**边查边插边删**，同时用一套"局部检测失衡 + 局部重建"的机制把树维持在近似平衡，把单点插入摊还到 $O(\log N)$。

> **本质洞察**：ikd-Tree 的核心思想可以一句话概括——**用"局部重建"代替"全局重建"**。静态 kd-Tree 失衡时要重建整棵树（$O(N\log N)$）；ikd-Tree 只在"失衡到一定程度的那棵子树"上重建，重建代价正比于子树大小而非全树。绝大多数插入只动到树的局部，于是摊还成本极低。这与第40章学过的"平衡二叉搜索树（如红黑树）靠局部旋转维持平衡"是同一种智慧的不同实现——**都是把全局不变量的维护，分摊成一系列廉价的局部修复**。区别在于：红黑树旋转是 $O(1)$ 的结构调整，ikd-Tree 局部重建是 $O(\text{子树})$ 的批量重排，因为 kd-Tree 的分割维度是轮转的，无法靠单次旋转修复。

### 反面：朴素方案为何不可行

把候选方案摆在一起对比，才能看清 ikd-Tree 每个设计点的必要性：

| 方案 | 插入成本 | 查询成本 | 删除支持 | 致命缺陷 |
|------|---------|---------|---------|---------|
| 静态 kd-Tree + 每帧全重建 | $O(N\log N)$/帧 | $O(\log N)$ | 重建时顺带 | 建树吃光时间预算，10 Hz 跑不动 |
| 静态 kd-Tree + 硬插不重建 | $O(\log N)$ | 退化到 $O(N)$ | 无 | 树越插越偏，查询崩溃 |
| 哈希体素地图（如 VoxelMap） | $O(1)$ | $O(1)$（格内） | $O(1)$ | 最近邻是"格内近似"，跨格漏检；本章选 ikd-Tree 以求精确近邻 |
| **ikd-Tree** | 摊还 $O(\log N)$ | $O(\log N)$ | 惰性删除 $O(\log N)$ | 实现复杂（但一次写对终身受益） |

> **对比性思维**：哈希体素地图（hash voxel map）和 ikd-Tree 是 LIO 建图的两大流派。前者快、实现简单，但最近邻是"先定位到体素、再在体素内找"，跨体素边界时可能漏掉更近的点；后者精确但复杂。FAST-LIO2 用 ikd-Tree，Faster-LIO 改用 iVox（增量体素），各有取舍。本章选 ikd-Tree，是因为它能让你**把 kd-Tree 这个数据结构吃透**——理解了增量平衡，你再去看任何空间索引都心里有数。第51章会让你实测两者的速度/精度差异。

### 历史：从 kd-Tree 到 ikd-Tree

kd-Tree 由 Jon Bentley 于 1975 年提出，是计算几何的奠基性结构。几十年里它一直被当作**静态**索引使用——离线建好、在线查询。机器人 SLAM 早期也这么用：PCL 的 `KdTreeFLANN` 每次地图更新就重建。直到实时 LIO 把帧率推到 10–20 Hz、地图推到百万点级，"每帧重建"才彻底撑不住。2021 年 Cai 等人的 ikd-Tree 论文（发表于 RAL/ICRA，作为 FAST-LIO2 的地图后端）系统性地解决了增量维护问题，引入了**惰性删除（lazy deletion）**、**属性下推（attribute push-down，记录子树的范围/删除计数）**、**自适应局部重建（自动判定哪棵子树该重建）** 三件套，并支持后台线程异步重建以隐藏延迟。本章实现一个**教学简化版**：保留增量插入、惰性删除、盒式范围搜索、$k$NN 查询四个核心操作，重建用同步阻塞版（够小场景跑通），把异步重建留作进阶练习。

### 理论：四个核心操作

ikd-Tree 对外暴露四个操作，对应 LIO 主循环的四种需求：

1. **增量插入（incremental insert）**：把新配准的点逐个或批量插入。插入时沿分割维度下降到合适位置；插入后检查途经子树是否失衡，失衡则触发该子树重建。
2. **$k$ 近邻查询（$k$NN search）**：iESKF 每点每迭代调用，找最近的 $k$（本章 $k=5$）个点用于平面拟合。标准 kd-Tree 的"分支限界 + 优先队列"算法。
3. **盒式范围搜索（box-wise range search）**：给定一个轴对齐包围盒（AABB），返回盒内所有点。用于"地图滑窗"——只保留机器人周围一定范围的点。
4. **惰性删除（lazy deletion）**：要淘汰旧点时，不真的从树里摘除（那会破坏结构），而是给节点打一个 `deleted` 标记，查询时跳过被标记的点。累积的删除点过多时，在重建时一并物理清除。

**平衡判据**是 ikd-Tree 的灵魂。每个节点记录其子树的点数 `treesize` 和已删除数 `invalidnum`。当满足下面任一条件时，以该节点为根的子树触发重建：

$$
\underbrace{S(\text{较重子树}) > \alpha_{\text{bal}}\cdot S(\text{整棵子树})}_{\text{形状失衡}}
\quad\text{或}\quad
\underbrace{N_{\text{deleted}} > \alpha_{\text{del}}\cdot S(\text{整棵子树})}_{\text{删除冗余过多}}
$$

其中 $S(\cdot)$ 是子树点数，$\alpha_{\text{bal}}$（平衡因子，典型 0.7）控制允许的形状倾斜，$\alpha_{\text{del}}$（删除因子，典型 0.5）控制允许的"墓碑"比例。重建时把该子树所有**未删除**的点收集起来，按标准 kd-Tree 方式（挑中位数）重新建一棵平衡子树，顺带物理清除所有墓碑。

> **多视角理解**：平衡判据可以从三个角度看。（1）**几何视角**：形状失衡意味着分割面没把点均分，查询时分支限界剪不掉多少分支，效率退化。（2）**摊还分析视角**：$\alpha_{\text{bal}}<1$ 保证每次重建后子树"足够平衡"，下次重建至少要等子树规模显著变化，于是重建频率低，摊还成本可控（与"动态数组倍增扩容"的摊还分析同理）。（3）**内存视角**：$\alpha_{\text{del}}$ 限制墓碑比例，防止删除标记堆积导致树名义大小远超有效点数、白白消耗查询时间。三个视角指向同一组参数，理解了就知道怎么调它们。

### 代码走读：节点定义与增量插入

先定义节点。每个节点除了存点和左右孩子，还要存"属性下推"信息——子树范围（用于盒式搜索剪枝）、子树大小、删除计数（用于平衡判据）：

```cpp
// include/mini_lio/ikd_tree.hpp  ——  教学简化版 ikd-Tree 节点
namespace mini_lio {

struct KdNode {
  Eigen::Vector3f point;            // 本节点存的点
  KdNode* left  = nullptr;
  KdNode* right = nullptr;
  int   axis    = 0;                // 本节点的分割维度（0/1/2 轮转）
  bool  deleted = false;            // 惰性删除标记（墓碑）

  // —— 属性下推（push-down）：聚合整棵子树的统计量 ——
  int   treesize   = 1;             // 子树总节点数（含墓碑）
  int   invalidnum = 0;             // 子树中已删除（墓碑）节点数
  Eigen::Vector3f range_min;        // 子树所有点的 AABB 下界
  Eigen::Vector3f range_max;        // 子树所有点的 AABB 上界
};

}  // namespace mini_lio
```

增量插入沿分割维度下降，到空位挂上新节点；回溯时更新途经节点的下推属性，并检查是否需要重建：

```cpp
// src/ikd_tree.cpp  ——  增量插入
namespace mini_lio {

// 递归插入；返回（可能因重建而改变的）子树根
KdNode* IKDTree::insert_rec(KdNode* node, const Eigen::Vector3f& p, int depth) {
  if (node == nullptr) {                 // 到达空位，创建叶子
    auto* leaf = new KdNode;
    leaf->point = p;
    leaf->axis  = depth % 3;             // 维度轮转：x→y→z→x...
    leaf->range_min = leaf->range_max = p;
    return leaf;
  }
  // 按当前节点的分割维度决定往左还是往右
  const int ax = node->axis;
  if (p[ax] < node->point[ax]) node->left  = insert_rec(node->left,  p, depth + 1);
  else                         node->right = insert_rec(node->right, p, depth + 1);

  pull_up(node);                          // 回溯：用孩子的属性更新本节点（下推的逆——上拉）

  // 平衡判据：失衡则就地重建以 node 为根的子树
  if (need_rebuild(node)) node = rebuild_subtree(node);
  return node;
}

// 上拉：聚合左右子树的 treesize / invalidnum / AABB 到本节点
void IKDTree::pull_up(KdNode* node) {
  node->treesize   = 1 + size_of(node->left) + size_of(node->right);
  node->invalidnum = (node->deleted ? 1 : 0)
                   + invalid_of(node->left) + invalid_of(node->right);
  node->range_min = node->point;
  node->range_max = node->point;
  merge_range(node, node->left);          // 把孩子的 AABB 并进来
  merge_range(node, node->right);
}

}  // namespace mini_lio
```

> **本质洞察**：`pull_up` 是整个数据结构正确性的命门。每次结构变化（插入/删除/重建）后，途经路径上**每一个**节点的 `treesize`/`invalidnum`/`range` 都必须重新聚合，否则平衡判据和盒式搜索剪枝就基于过期信息做决策，结果要么该重建的不重建（查询退化），要么盒式搜索误剪掉本该返回的点（漏检最近邻，配准全错）。**下推属性的一致性维护，是所有"带聚合信息的树形结构"的共同难点**——B+ 树、线段树、区间树都有同样的坑。

### 代码走读：失衡判定与局部重建

```cpp
// src/ikd_tree.cpp（续）  ——  平衡判据与局部重建
namespace mini_lio {

bool IKDTree::need_rebuild(const KdNode* node) const {
  const int n = node->treesize;
  if (n < kMinRebuildSize) return false;          // 太小不值得重建（避免抖动）
  const int heavier = std::max(size_of(node->left), size_of(node->right));
  const bool shape_bad   = heavier   > alpha_bal_ * n;   // 形状失衡
  const bool delete_bad  = node->invalidnum > alpha_del_ * n;  // 墓碑过多
  return shape_bad || delete_bad;
}

// 把子树拍平成有效点数组，重新建一棵平衡子树（顺带清墓碑）
KdNode* IKDTree::rebuild_subtree(KdNode* node) {
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(node->treesize);
  flatten(node, pts);                 // 中序遍历，只收集 deleted==false 的点
  free_subtree(node);                 // 物理释放旧子树（墓碑随之消失）
  return build_balanced(pts, 0, pts.size(), /*depth=*/0);  // 挑中位数递归建平衡树
}

// 标准 kd-Tree 建树：按当前维度对区间排序取中位数作根，左右递归
KdNode* IKDTree::build_balanced(std::vector<Eigen::Vector3f>& pts,
                                int lo, int hi, int depth) {
  if (lo >= hi) return nullptr;
  const int ax  = depth % 3;
  const int mid = (lo + hi) / 2;
  std::nth_element(pts.begin() + lo, pts.begin() + mid, pts.begin() + hi,
                   [ax](const auto& a, const auto& b){ return a[ax] < b[ax]; });
  auto* root  = new KdNode;
  root->point = pts[mid];
  root->axis  = ax;
  root->left  = build_balanced(pts, lo, mid, depth + 1);
  root->right = build_balanced(pts, mid + 1, hi, depth + 1);
  pull_up(root);
  return root;
}

}  // namespace mini_lio
```

注意 `build_balanced` 用 `std::nth_element`（平均 $O(n)$）而非整段排序（$O(n\log n)$）找中位数——这是建树常被忽略的优化点。重建一棵 $n$ 点子树的成本是 $O(n\log n)$（每层 $O(n)$ 的 nth_element，共 $\log n$ 层），但因为平衡判据保证重建不频繁，摊还到每次插入仍是 $O(\log N)$。

### 代码走读：kNN 查询与盒式搜索

$k$NN 用经典的"分支限界 + 大顶堆"：维护一个大小为 $k$ 的最大堆（堆顶是当前 $k$ 个候选里最远的），下降时先进"查询点所在的那一侧"，回溯时只有当"分割面另一侧可能存在更近点"才搜另一侧：

```cpp
// src/ikd_tree.cpp（续）  ——  kNN 查询（分支限界）
namespace mini_lio {

void IKDTree::knn_rec(const KdNode* node, const Eigen::Vector3f& q, int k,
                      std::priority_queue<DistIdx>& heap) const {
  if (node == nullptr) return;
  if (!node->deleted) {                          // 墓碑跳过，不计入候选
    const float d2 = (node->point - q).squaredNorm();
    if ((int)heap.size() < k)        heap.push({d2, node->point});
    else if (d2 < heap.top().dist2){ heap.pop(); heap.push({d2, node->point}); }
  }
  const int ax = node->axis;
  const float diff = q[ax] - node->point[ax];
  const KdNode* near = (diff < 0) ? node->left  : node->right;   // 先搜近侧
  const KdNode* far  = (diff < 0) ? node->right : node->left;
  knn_rec(near, q, k, heap);
  // 剪枝：只有当"到分割面的距离²"小于当前第 k 远，才有必要搜远侧
  if ((int)heap.size() < k || diff * diff < heap.top().dist2)
    knn_rec(far, q, k, heap);
}

// 盒式范围搜索：用子树 AABB 剪枝，整子树落盒外直接跳过
void IKDTree::box_search_rec(const KdNode* node, const Eigen::AlignedBox3f& box,
                             std::vector<Eigen::Vector3f>& out) const {
  if (node == nullptr) return;
  Eigen::AlignedBox3f sub(node->range_min, node->range_max);
  if (!box.intersects(sub)) return;              // 子树 AABB 与查询盒不交 → 整子树剪掉
  if (!node->deleted && box.contains(node->point)) out.push_back(node->point);
  box_search_rec(node->left,  box, out);
  box_search_rec(node->right, box, out);
}

}  // namespace mini_lio
```

> **本质洞察**：$k$NN 的剪枝条件 `diff*diff < heap.top().dist2` 是 kd-Tree 全部效率的来源。它的含义是："如果查询点到分割超平面的距离，已经比我手上第 $k$ 远的候选还远，那么超平面另一侧的所有点必然更远，整侧跳过。" 这一行剪枝把期望复杂度从 $O(N)$ 压到 $O(\log N)$。**写错这一行（比如忘了平方、或比较方向反了），程序仍能跑、仍返回"某些"近邻，但不再是真正的最近邻**——这是 LIO 里最隐蔽的 bug 之一，因为它不崩溃，只是精度悄悄变差。

### 配置详解：ikd-Tree 与地图参数

```yaml
# config/params.yaml （地图部分）
map:
  ikd_balance_alpha: 0.7    # 形状平衡因子 α_bal。越小越严格(更平衡但重建更频繁)
  ikd_delete_alpha:  0.5    # 删除冗余因子 α_del。墓碑超此比例触发重建清理
  min_rebuild_size:  20     # 子树小于此规模不重建（避免小树反复抖动）
  knn_k: 5                  # 平面拟合用的最近邻个数（§50.7 用）
  local_map_range: 150.0    # 局部地图滑窗半边长（米）：超出则盒式删除
  downsample_map: 0.4       # 入图前的地图体素分辨率（米），略大于扫描降采样
```

| 配置项 | 默认值来源 | 调大/调小的影响 | 与其他配置的耦合 |
|--------|-----------|----------------|-----------------|
| `ikd_balance_alpha` | FAST-LIO2 经验值 0.6–0.7 | 调小→树更平衡、查询更快，但重建更频繁、插入更慢 | 与点云吞吐量耦合：点越多越要平衡 |
| `ikd_delete_alpha` | 0.5 | 调小→墓碑清得勤、内存省，但重建频繁 | 与 `local_map_range` 耦合：滑窗越紧、删除越多 |
| `knn_k` | 平面拟合需 ≥3，取 5 留冗余 | 调大→平面拟合更稳但更慢、易跨平面取点 | 与 §50.7 平面有效性判据耦合 |
| `local_map_range` | 略大于传感器量程 | 调大→地图大、漂移约束多但内存涨、查询慢；调小→省内存但回到老区域无地图 | 与 §50.2 `max_range` 协调 |
| `downsample_map` | 略大于 scan 降采样 0.3 | 调小→地图密、精度高、内存涨；调大→地图疏、快 | 应 ≥ §50.2 `voxel_leaf_size`，否则地图比扫描还密无意义 |

> **理论-工程桥接**：注意 `downsample_map`（0.4）建议略大于扫描降采样 `voxel_leaf_size`（0.3）。理论上地图分辨率决定了 point-to-plane 能达到的精度下限——地图越密，平面拟合越准。但工程上地图是累积的，若地图比每帧扫描还密，内存和查询都吃不消却换不来精度（因为单帧扫描的精度已经被 0.3 限死）。所以"地图分辨率 ≥ 扫描分辨率"是个朴素但常被违反的工程约束。

### 运行验证

- **正确性回归**：随机生成 1 万个点全部插入，再对若干查询点分别用 ikd-Tree 的 $k$NN 和暴力 $O(N)$ 扫描求最近邻，断言两者结果**完全一致**。这是 ikd-Tree 单元测试的金标准，能立刻抓出剪枝写错的 bug。
- **平衡性观测**：插入 10 万点后打印树高。平衡树高约 $\log_2(10^5)\approx 17$；若树高几百上千，说明重建没触发或判据写错。
- **删除有效性**：盒式删除一片区域后，对该区域查询应返回空或仅剩边缘点；打印 `invalidnum/treesize` 比例，应稳定在 `alpha_del` 以下。
- **性能基线**：百万点地图上单次 $k$NN 应在微秒级（个位数到几十微秒）。若到了毫秒级，多半是退化成了近似线性扫描。

### ⚠️ 常见陷阱

> **数据结构陷阱：kNN 剪枝条件写反或漏平方**
> 错误做法：写成 `diff < heap.top().dist2`（漏平方）或 `diff*diff > heap.top().dist2`（方向反），甚至无条件搜两侧。
> 现象：程序不崩，但返回的"最近邻"并非真正最近；配准精度无明显报错地变差，轨迹缓慢漂移。
> 根本原因：剪枝条件是 kd-Tree 正确性与效率的交汇点。漏平方导致量纲不匹配（距离 vs 距离²）；方向反会该搜的不搜、漏掉更近点；无条件搜两侧虽正确但退化成 $O(N)$。
> 正确做法：剪枝条件恒为 `堆未满 || (到分割面距离)² < 当前第k远距离²`，全程用距离平方避免开方。用"暴力扫描对拍"单元测试守护这一行。

> **数据结构陷阱：插入/删除后忘记上拉（pull_up）下推属性**
> 错误做法：插入新节点后只挂上指针，不更新途经节点的 `treesize`/`invalidnum`/`range`。
> 现象：平衡判据永远不触发（treesize 不更新）导致树退化；或盒式搜索漏点（range 过期，本应相交的子树被误剪）。
> 根本原因：下推属性是聚合量，任何子树变化都必须沿回溯路径重新聚合，否则全树的统计信息与实际脱节。
> 正确做法：把 `pull_up` 作为递归插入/删除回溯阶段的**强制最后一步**，凡修改结构必上拉；重建建树时每个新节点也要 `pull_up`。

> **概念误区：以为惰性删除的墓碑不影响性能**
> 新手想法："删除只是打个标记，$O(1)$，不用管。"
> 实际上：墓碑节点仍占据树的位置、仍被查询遍历（只是不计入候选），墓碑堆积会让"名义树大小"远超有效点数，查询白白多走很多路。
> 正确理解：惰性删除是"把删除成本延迟、批量摊还"，不是"消除成本"。`alpha_del` 判据正是为了在墓碑过多时强制重建清理，把延迟的成本一次性偿还。删除多的场景（紧滑窗）要把 `alpha_del` 调小。

### 练习

1. **（对拍测试）** 实现 ikd-Tree 的 $k$NN 后，写一个 GoogleTest：随机插入 $N=5000$ 点，随机选 200 个查询点，对每个查询点同时用 ikd-Tree $k$NN（$k=5$）和暴力排序求前 5 近邻，断言两组结果按距离排序后逐一相等。再随机删除一半点，重复对拍。
2. **（摊还分析）** 把 `ikd_balance_alpha` 从 0.5 扫到 0.9，每个取值下插入 10 万点，记录：总插入耗时、触发重建的次数、最终树高。画出三条曲线，解释为什么存在一个"插入快但查询不退化"的甜点区间。
3. **（异步重建进阶）** 当前 `rebuild_subtree` 是同步阻塞的，大子树重建会卡住主循环。设计一个后台线程方案：主线程检测到需重建时，把子树"冻结"并交给后台线程重建，期间新插入暂存到一个缓冲；重建完成后用新子树替换并补放缓冲点。讨论这个方案的线程安全难点（提示：查询线程此刻可能正在遍历被冻结的子树）。

---

## §50.7 point-to-plane 配准 ⭐⭐⭐

### 模块功能与在管线中的位置

§50.5 的 iESKF 里有一行注释 "∂r/∂δθ ...（§50.7 推导）"和一个被频繁调用却没展开的函数 `build_residuals`。本节就把这两块补上：给定当前位姿估计、去畸变后的点云、ikd-Tree 地图，**为每个点构造一个"点到平面"残差及其雅可比**，打包成 iESKF 更新需要的 $\boldsymbol H$ 和 $\boldsymbol z$。这是 LiDAR 观测进入滤波器的唯一入口——iESKF 信不信 LiDAR、信多少、往哪个方向修正，全由这里产出的残差和雅可比决定。

### 动机：为什么是"点到面"而不是"点到点"

最朴素的配准残差是**点到点（point-to-point）**：当前点变换到世界系后，与地图里最近的那个点的欧氏距离。ICP（Iterative Closest Point，迭代最近点）最初就这么干。但点到点有个根本问题——**两次扫描几乎不可能打在同一个物理点上**。LiDAR 是离散采样，这一帧的激光打在墙上 A 处，下一帧打在相邻的 B 处，A、B 是同一面墙上的不同点。强行让 A 去贴 B，等于在拟合采样噪声，收敛慢且精度受限于点间距。

**点到面（point-to-plane）** 换了个思路：地图局部是连续表面（墙、地、桌面），当前点应该落在**这个表面上**，而不是某个特定的地图点上。于是残差定义为"当前点到这个局部平面的垂直距离"。这样，当前点可以沿平面自由滑动而不受惩罚，只在偏离平面（穿墙或离墙）时才受约束。这更贴合物理实际（表面是连续的），收敛快得多，精度也高。这是 Chen & Medioni 1991 年提出的 point-to-plane ICP，至今是 LIO 配准的主流残差形式。

> **本质洞察**：点到点 vs 点到面的区别，本质是**约束维度**的区别。一个点到点残差把当前点钉死在一个三维位置上（约束 3 个自由度，但其实只有沿连线方向有意义）；一个点到面残差只约束当前点在**法向上**的位置（1 个自由度），放任它在切向上滑动。后者更"诚实"——它只惩罚我们真正确信的东西（点该在面上），不编造我们不知道的东西（点该在面上的哪个位置）。**好的残差只编码你确信的约束，对不确定的自由度保持沉默**，这是状态估计建模的通用原则。

### 反面：点到点的代价

| 残差类型 | 几何含义 | 残差维度 | 收敛性 | 适用场景 |
|---------|---------|---------|--------|---------|
| 点到点 | 当前点到最近地图点的距离 | 标量（实为 3D 向量长度） | 慢，需大量迭代 | 点云稠密、点间距小 |
| **点到面** | 当前点到局部拟合平面的垂直距离 | 标量 | 快，少量迭代收敛 | 结构化环境（室内/城市），LIO 主流 |
| 点到线 | 当前点到局部拟合直线的距离 | 标量 | 中等 | 边缘特征（LOAM 风格用它配合点到面） |
| 面到面（GICP） | 协方差对齐 | 标量（马氏距离） | 快、最鲁棒 | 计算量大，FAST-LIO 不用 |

> **对比性思维**：LOAM/LeGO-LOAM 走的是"特征点"路线——先把点云分成边缘点（点到线）和平面点（点到面）两类，分别配准。FAST-LIO 系列走的是"直接法"路线——不分类，**所有点统一用点到面**。后者更简单（省掉特征提取）、更鲁棒（不依赖特征点够不够），代价是点数多、依赖 ikd-Tree 的高效近邻查询撑住实时性。本章跟随 FAST-LIO，全点点到面。**正是因为 §50.6 把近邻查询做到了微秒级，我们才敢"所有点都配"**——这就是数据结构选择如何反过来决定算法策略。

### 理论：从最近邻到平面

给定一个去畸变后的点 $\boldsymbol p^L$（在 LiDAR 系下），构造点到面残差分四步：

**第一步：变换到世界系。** 用当前位姿估计（IMU 系到世界系的 $\boldsymbol R,\boldsymbol p$，以及 LiDAR 到 IMU 的外参 $\boldsymbol R_{IL},\boldsymbol p_{IL}$）：

$$
\boldsymbol p^W = \boldsymbol R\,(\boldsymbol R_{IL}\,\boldsymbol p^L + \boldsymbol p_{IL}) + \boldsymbol p
\;=\; \boldsymbol R\,\boldsymbol p^I + \boldsymbol p,
\qquad \boldsymbol p^I \triangleq \boldsymbol R_{IL}\,\boldsymbol p^L + \boldsymbol p_{IL}
$$

这里 $\boldsymbol p^I$ 是该点在 IMU 系下的坐标（外参把它从 LiDAR 系搬到 IMU 系），后面求雅可比时会用到它。

**第二步：在地图里找 $k$ 近邻。** 用 §50.6 的 ikd-Tree 对 $\boldsymbol p^W$ 查 $k=5$ 个最近邻 $\{\boldsymbol q_1,\dots,\boldsymbol q_k\}$。

**第三步：拟合平面。** 平面用 $\boldsymbol n^\top\boldsymbol x + d = 0$ 表示（$\boldsymbol n$ 是单位法向，$d$ 是截距）。对 $k$ 个近邻求解这个平面——本质是个超定线性最小二乘：让所有近邻尽量满足 $\boldsymbol n^\top\boldsymbol q_i + d = 0$。常见两种解法：

- **法方程解法**：固定 $\|\boldsymbol n\|=1$ 的约束不好处理，工程上常令 $d=1$（或 $-1$），解 $\boldsymbol A\boldsymbol n = -\boldsymbol 1$（$\boldsymbol A$ 的每行是一个 $\boldsymbol q_i^\top$），再把 $\boldsymbol n$ 归一化、相应缩放 $d$。
- **PCA/SVD 解法**：对 $k$ 个点去均值后求协方差矩阵，**最小特征值对应的特征向量就是法向 $\boldsymbol n$**（点在该方向上散布最小，即垂直于平面的方向），$d = -\boldsymbol n^\top\bar{\boldsymbol q}$。这种解法更稳，还能顺带用特征值判断"这几个点像不像一个平面"。

**第四步：算残差。** 点到面的有向距离就是残差（标量）：

$$
r = \boldsymbol n^\top \boldsymbol p^W + d
$$

iESKF 的目标是把所有点的 $r$ 压到接近 0——即所有当前点都贴在它们各自的局部平面上。

> **本质洞察**：第三步里"最小特征值的特征向量是法向"这件事，是 PCA（主成分分析）在几何上的直接体现。把 $k$ 个共面点看成一团数据，它们在平面内两个方向上散得开（大特征值），在垂直平面方向上几乎没散布（最小特征值）。**散布最小的方向，就是平面的法向**。这同时给了我们一个免费的平面质量检测器：如果最小特征值不够小、或不显著小于次小特征值，说明这几个点根本不共面（可能跨了两面墙的拐角），这个残差不可信，应丢弃。

### 理论：残差对状态的雅可比

iESKF 需要 $r$ 对误差状态 $\delta\boldsymbol x$ 的雅可比 $\boldsymbol H$。残差 $r = \boldsymbol n^\top\boldsymbol p^W + d$ 只通过 $\boldsymbol p^W = \boldsymbol R\,\boldsymbol p^I + \boldsymbol p$ 依赖状态，而 $\boldsymbol p^W$ 只依赖**位置** $\boldsymbol p$ 和**旋转** $\boldsymbol R$（其余状态分量——速度、偏置、重力——不进入观测，对应雅可比列为 0）。

对位置求导最简单（$\boldsymbol p^W$ 对 $\boldsymbol p$ 是恒等）：

$$
\frac{\partial r}{\partial \delta\boldsymbol p}
= \boldsymbol n^\top \frac{\partial \boldsymbol p^W}{\partial \delta\boldsymbol p}
= \boldsymbol n^\top
$$

对旋转求导要用右扰动（与本章 $\boxplus$ 约定一致：$\boldsymbol R \leftarrow \boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)$）。把扰动代入 $\boldsymbol p^W$：

$$
\boldsymbol p^W(\delta\boldsymbol\theta)
= \boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)\,\boldsymbol p^I + \boldsymbol p
\approx \boldsymbol R\,(\boldsymbol I + \delta\boldsymbol\theta^\wedge)\,\boldsymbol p^I + \boldsymbol p
= \boldsymbol p^W_0 + \boldsymbol R\,\delta\boldsymbol\theta^\wedge\,\boldsymbol p^I
$$

用反对称矩阵恒等式 $\delta\boldsymbol\theta^\wedge\,\boldsymbol p^I = -\,(\boldsymbol p^I)^\wedge\,\delta\boldsymbol\theta$，得：

$$
\frac{\partial \boldsymbol p^W}{\partial \delta\boldsymbol\theta}
= -\,\boldsymbol R\,(\boldsymbol p^I)^\wedge
$$

于是残差对旋转的雅可比为：

$$
\boxed{\;
\frac{\partial r}{\partial \delta\boldsymbol\theta}
= \boldsymbol n^\top \frac{\partial \boldsymbol p^W}{\partial \delta\boldsymbol\theta}
= -\,\boldsymbol n^\top \boldsymbol R\,(\boldsymbol p^I)^\wedge
\;}
$$

这正是 §50.5 代码里那一行：

```cpp
H.block<1,3>(i,6) = -matches[i].normal.transpose()
                    * x.rot.matrix() * Sophus::SO3d::hat(matches[i].p_imu);
```

对照可见：`normal` 是 $\boldsymbol n$，`x.rot.matrix()` 是 $\boldsymbol R$，`p_imu` 是 $\boldsymbol p^I$，`hat()` 是 $(\cdot)^\wedge$，符号是负——**逐项对得上**。完整的单行雅可比是 $\boldsymbol H_i = [\,\boldsymbol n^\top,\ \boldsymbol 0_{1\times3},\ -\boldsymbol n^\top\boldsymbol R(\boldsymbol p^I)^\wedge,\ \boldsymbol 0,\ \boldsymbol 0,\ \boldsymbol 0\,]$，其中非零块只在位置（第 0–2 列）和旋转（第 6–8 列）。

> **理论-工程桥接**：雅可比的"非零结构"直接决定了 §50.5 里 $\boldsymbol H$ 的稀疏性——18 列里只有 6 列非零。这就是为什么 §50.5 代码用 `H.block<1,3>(i,0)` 和 `H.block<1,3>(i,6)` 只填两块，其余留零。理解雅可比的稀疏结构，不仅是数学正确性问题，更是性能问题：稠密地填一个 $m\times18$ 矩阵再相乘，浪费的是已知为零的 12 列。**雅可比的零，和它的非零一样有信息量。**

### 代码走读：平面拟合（PCA 法）

```cpp
// src/registration.cpp  ——  对 k 个近邻拟合平面 + 质量检测
#include "mini_lio/registration.hpp"

namespace mini_lio {

// 用 PCA 拟合平面：返回是否拟合成功；成功则填 n（单位法向）与 d（截距）
bool fit_plane(const std::vector<Eigen::Vector3f>& nbr,
               Eigen::Vector3f& n, float& d, float plane_thresh) {
  if (nbr.size() < 3) return false;
  // 1) 去均值
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& q : nbr) centroid += q;
  centroid /= static_cast<float>(nbr.size());
  // 2) 协方差矩阵
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& q : nbr) {
    const Eigen::Vector3f e = q - centroid;
    cov += e * e.transpose();
  }
  // 3) 自伴随特征分解：最小特征值的特征向量 = 法向
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
  n = es.eigenvectors().col(0);           // Eigen 按特征值升序排列，col(0) 即最小
  n.normalize();
  d = -n.dot(centroid);

  // 4) 平面质量检测：每个近邻到平面的距离都应很小，否则这不是个平面
  for (const auto& q : nbr) {
    if (std::fabs(n.dot(q) + d) > plane_thresh) return false;   // 有点离面太远→弃
  }
  return true;
}

}  // namespace mini_lio
```

注意第 4 步的**平面质量检测**：拟合出平面不等于这几个点真的共面。如果 5 个近邻里混进了拐角另一面的点，拟合出的"平面"会斜穿两面墙，残差完全无意义。检测方法是回代——要求每个近邻到拟合平面的距离都小于 `plane_thresh`（典型 0.1 m），有一个超标就判定"非平面"，整个残差作废。这一步是配准鲁棒性的关键防线。

### 代码走读：构造残差集合（build_residuals）

现在补上 §50.5 一直在调用的 `build_residuals`，它把上面所有步骤串起来，并支持 TBB 并行：

```cpp
// src/registration.cpp（续）  ——  为整帧点云构造 point-to-plane 残差
namespace mini_lio {

void build_residuals(const State& x, const CloudPtr& cloud, const IKDTree& map,
                     std::vector<PointPlane>& out) {
  const Eigen::Matrix3f R   = x.rot.matrix().cast<float>();
  const Eigen::Vector3f p   = x.pos.cast<float>();
  const Eigen::Matrix3f R_il= x.R_LtoI.cast<float>();   // LiDAR→IMU 外参旋转
  const Eigen::Vector3f p_il= x.p_LtoI.cast<float>();   // LiDAR→IMU 外参平移

  out.clear();
  out.resize(cloud->size());                 // 先占位，并行写各自下标（避免竞争）
  std::vector<char> valid(cloud->size(), 0); // 标记每个点是否产生了有效残差

  // 每个点独立：变换→近邻→拟合→残差，天然可并行（TBB parallel_for）
  tbb::parallel_for(size_t(0), cloud->size(), [&](size_t i) {
    const Eigen::Vector3f pL(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    const Eigen::Vector3f pI = R_il * pL + p_il;        // → IMU 系（雅可比要用）
    const Eigen::Vector3f pW = R * pI + p;              // → 世界系（查近邻用）

    std::vector<Eigen::Vector3f> nbr;
    map.knn(pW, /*k=*/5, nbr);                          // §50.6 的 kNN
    if (nbr.size() < 5) return;                         // 近邻不足，跳过

    Eigen::Vector3f n; float d;
    if (!fit_plane(nbr, n, d, /*plane_thresh=*/0.1f)) return;   // 非平面，跳过

    const float r = n.dot(pW) + d;                      // 点到面残差
    // 鲁棒性：残差过大多半是动态物体/错误匹配，按近邻距离自适应剔除
    if (std::fabs(r) > 0.3f) return;

    out[i].normal   = n;
    out[i].p_imu    = pI;                               // 存 IMU 系坐标供雅可比
    out[i].residual = r;
    valid[i] = 1;
  });

  // 紧致化：把有效残差压到数组前段（并行写完后单线程压缩）
  size_t w = 0;
  for (size_t i = 0; i < out.size(); ++i)
    if (valid[i]) out[w++] = out[i];
  out.resize(w);
}

}  // namespace mini_lio
```

> **本质洞察**：`build_residuals` 是整个 LIO 里**最热的函数**——每帧每次迭代调用一次，内部对每个点（上万个）做一次 ikd-Tree 查询。它是性能 Profiling 的头号嫌疑（第51章会验证）。它也是少数"天然可并行"的环节：每个点的残差构造彼此独立，没有数据依赖，所以用 `tbb::parallel_for` 一拆就是近线性加速。**找到这种"独立、密集、可并行"的热点并喂给 TBB，是 LIO 优化的最大单一杠杆**——对照骨架里"TBB 并行 reduce"的设计意图，正是此处。

### 配置详解：配准参数

```yaml
# config/params.yaml （配准部分）
registration:
  plane_thresh: 0.1        # 平面质量阈值（米）：近邻到拟合面距离超此则判非平面
  max_residual: 0.3        # 残差上限（米）：超此剔除（动态物体/误匹配）
  min_neighbors: 5         # 拟合平面所需最少近邻（须 ≥3，留冗余取 5）
  use_robust_kernel: true  # 是否对残差套 Huber 核（降低离群点权重）
  huber_delta: 0.1         # Huber 核拐点（米）
```

| 配置项 | 默认值来源 | 调大/调小的影响 | 与其他配置的耦合 |
|--------|-----------|----------------|-----------------|
| `plane_thresh` | 与地图分辨率同量级 0.1 m | 调小→只接受很平的面、约束少但准；调大→纳入弯曲面、约束多但引入误差 | 与 §50.6 `downsample_map` 耦合：地图越疏面越糙、阈值要放宽 |
| `max_residual` | 经验 0.3 m | 调小→剔除激进、抗动态物体强但可能误删真约束；调大→保留多、易被动态物体污染 | 与 `huber_delta` 配合做两级鲁棒 |
| `min_neighbors` | 平面需 3 点，取 5 | 调大→平面更稳但更慢、拐角处更易凑不齐而丢点 | 与 §50.6 `knn_k` 必须一致 |
| `huber_delta` | 与 `obs_noise` 同量级 | 调小→更早压制大残差权重、更鲁棒但收敛慢；调大→接近纯最小二乘 | 与 §50.5 `obs_noise` 耦合 |

> **理论-工程桥接**：`use_robust_kernel`（Huber 核）和 `max_residual`（硬阈值剔除）是处理离群点的两个层次。硬阈值是"一刀切"——残差超 0.3 m 直接扔掉；Huber 核是"软处理"——残差小时按平方惩罚（信任），残差大时按线性惩罚（降权但不完全丢弃）。现实中动态物体（行人、车辆）会制造大残差，两级配合既不让它们带偏估计，又不至于在拥挤场景把有效点也误删光。第51章在真实数据上调这两个参数时，你会真切感受到鲁棒核的价值。

### 运行验证

- **雅可比数值校验**：对单个残差，用数值差分（给状态加微小扰动 $\epsilon$，看 $r$ 的变化）求 $\partial r/\partial\delta\boldsymbol\theta$ 和 $\partial r/\partial\delta\boldsymbol p$，与解析雅可比对比，相对误差应 $<10^{-4}$。这是抓雅可比符号/系数错误的终极手段。
- **平面拟合正确性**：构造一组已知平面（如 $z=2$）上的点加小噪声，喂给 `fit_plane`，断言返回法向接近 $(0,0,1)$、截距接近 $-2$。
- **残差分布**：配准收敛后，打印所有有效残差的直方图，应集中在 0 附近、std 与 `obs_noise` 同量级。若残差整体偏置不为 0，说明外参或地图有系统误差。
- **有效点比例**：打印 `有效残差数 / 输入点数`，结构化环境应在 60%–90%；过低（<30%）说明地图太疏、平面阈值太严，或位姿初值太差导致近邻全配错。

### ⚠️ 常见陷阱

> **数学陷阱：雅可比用错扰动模型（左扰动配右 boxplus）**
> 错误做法：雅可比按左扰动 $\boldsymbol R\leftarrow\mathrm{Exp}(\delta\boldsymbol\theta)\boldsymbol R$ 推，得 $-\boldsymbol n^\top(\boldsymbol R\boldsymbol p^I)^\wedge$，但 §50.5 的 $\boxplus$ 用的是右扰动。
> 现象：iESKF 收敛慢、精度差，或在大转动时发散；数值雅可比校验不通过。
> 根本原因：雅可比的扰动模型必须与状态更新的 $\boxplus$ 约定严格一致。右 $\boxplus$ 对应右扰动雅可比 $-\boldsymbol n^\top\boldsymbol R(\boldsymbol p^I)^\wedge$（扰动在 $\boldsymbol p^I$ 上取 hat）；左扰动则是在 $\boldsymbol R\boldsymbol p^I$ 上取 hat，两者差一个旋转，混用即不自洽。
> 正确做法：全章固定右扰动，雅可比统一为 $-\boldsymbol n^\top\boldsymbol R(\boldsymbol p^I)^\wedge$；写数值雅可比单元测试，扰动方式与 $\boxplus$ 完全一致地施加。

> **算法陷阱：跳过平面质量检测，盲目用拟合结果**
> 错误做法：`fit_plane` 只求最小特征向量当法向，不回代检查近邻是否真共面。
> 现象：拐角、树木、灌木等非平面区域产生大量虚假平面残差，把估计往错误方向拽，轨迹在复杂场景抖动。
> 根本原因：PCA 对任意点云都能给出一个"最小散布方向"，但那未必是真平面的法向——5 个分布在拐角两面的点也有最小特征向量，只是毫无物理意义。
> 正确做法：拟合后必做回代检测（每个近邻到面距离 < `plane_thresh`），不通过则丢弃该残差。这是把"几何上能算"和"物理上有效"区分开的必要一步。

> **工程陷阱：在并行循环里共享可变状态导致数据竞争**
> 错误做法：`build_residuals` 里多线程 `out.push_back(...)` 往同一个 vector 追加。
> 现象：偶发崩溃、残差数随机变化、AddressSanitizer 报 data race；难复现的"薛定谔 bug"。
> 根本原因：`std::vector::push_back` 非线程安全，多线程同时写同一容器会破坏内部指针/size。
> 正确做法：预先 `resize` 到点数，每个线程只写自己的下标 `out[i]`（无竞争），并行结束后单线程紧致化；或每线程写局部 buffer 最后归并。本章用前者。

> **概念误区：以为残差越多越好（不剔除离群点）**
> 新手想法："约束越多估计越准，残差全留着不就行了？"
> 实际上：动态物体、错误匹配、非平面区域产生的残差是**有偏的错误约束**，它们不是噪声（均值为零）而是系统性误导，留得越多估计被带得越偏。
> 正确理解：配准的鲁棒性来自"剔除坏约束"而非"堆砌所有约束"。`max_residual` 硬阈值 + Huber 软降权 + 平面质量检测三道关卡，目的都是把错误约束挡在滤波器之外。质量比数量重要。

### 练习

1. **（数值雅可比）** 实现一个 GoogleTest：随机构造状态 $\boldsymbol x$、点 $\boldsymbol p^L$、法向 $\boldsymbol n$，用解析公式算 $\partial r/\partial\delta\boldsymbol\theta$；再对 $\boldsymbol R$ 施加 $10^{-6}$ 量级的右扰动数值差分，断言解析与数值雅可比的相对误差 $<10^{-4}$。把扰动改成左扰动，观察断言失败——亲手验证扰动模型必须自洽。
2. **（平面质量）** 给 `fit_plane` 喂两组数据：(a) 一个平面上的 5 点；(b) 拐角两面墙上各取若干点。打印两组的三个特征值。总结"是平面"和"不是平面"在特征值分布上的差异，并据此设计一个比"回代距离阈值"更鲁棒的平面判据（提示：用 $\lambda_0/\lambda_1$ 比值）。
3. **（鲁棒核对比）** 在合成数据里人为加入 10% 的离群点（残差服从大方差分布），分别用：(a) 纯最小二乘；(b) 硬阈值剔除；(c) Huber 核，跑 iESKF，对比三者的位姿估计误差。说明为什么 (c) 在"离群点比例适中"时往往优于 (b)。

---

## §50.8 回环检测与位姿图优化（可选）⭐⭐⭐

### 模块功能与在管线中的位置

到 §50.7 为止，我们有了一个**里程计**——它逐帧估计位姿、累积成轨迹。但里程计有个无法回避的宿命：**漂移（drift）**。每帧的微小误差会累积，走几百米后估计轨迹会偏离真实轨迹几米甚至几十米。本节解决的是漂移的"最后一公里"：当机器人**重新回到去过的地方**，我们识别出"这里我来过"（回环检测，loop closure detection），用这个约束把累积的漂移一次性拉回（位姿图优化，pose graph optimization）。这是 LIO**可选**但能显著提升长时一致性的模块，对应 LIO-SAM 的后端。它的输入是里程计输出的关键帧序列，输出是优化后的全局一致轨迹。

### 动机：里程计为什么必然漂移

iESKF 是个**纯前向**的估计器——它只看当前帧和局部地图，没有"全局记忆"。每帧更新都在上一帧的基础上增量修正，误差像滚雪球：第 $i$ 帧的位姿误差是前 $i$ 帧误差的累积。数学上，里程计估计的是相邻帧间的相对位姿 $\Delta\boldsymbol T_i$，全局位姿 $\boldsymbol T_i = \boldsymbol T_0\,\Delta\boldsymbol T_1\cdots\Delta\boldsymbol T_i$ 是一长串相对位姿连乘，每个 $\Delta\boldsymbol T_i$ 的小误差会沿链条传播放大。这不是 iESKF 没调好，而是**任何里程计的本质局限**——没有外部参照，误差无从校正。

回环提供的正是这个"外部参照"。当机器人走了一圈回到起点附近，回环检测告诉我们"第 500 帧的位置 = 第 5 帧的位置"。这是一个**跨越时间的约束**：它把 $\boldsymbol T_{500}$ 和 $\boldsymbol T_5$ 直接联系起来，绕过了中间 495 帧的误差累积链。把这个约束加进优化，求解器会把累积的漂移**均摊**到整条轨迹上，让首尾闭合。

> **本质洞察**：里程计和回环的关系，是**局部精度**与**全局一致**的分工。里程计擅长局部——相邻帧间它非常准（这正是 iESKF 紧耦合的功劳）；但它不擅长全局——走远了会漂。回环擅长全局——它提供跨越长时间跨度的约束；但它不管局部细节。**好的 SLAM 系统是"局部靠里程计、全局靠回环"的合奏**，谁也替代不了谁。这也解释了为什么本节标"可选"：如果你的应用只关心短程相对运动（如局部避障），里程计足矣；只有需要长时全局一致地图（如建图、重定位）时，回环才不可或缺。

### 反面：没有回环 vs 有回环

| 维度 | 纯里程计（无回环） | 加回环 + 位姿图优化 |
|------|------------------|--------------------|
| 局部精度 | 高（相邻帧很准） | 高（不破坏局部） |
| 全局漂移 | 随距离线性累积，无界 | 闭环处拉回，有界 |
| 首尾一致性 | 走一圈回来对不上，地图重影 | 闭合，地图单层清晰 |
| 计算成本 | 低（前向滤波） | 高（需检测 + 全局优化） |
| 实时性 | 严格实时 | 回环优化常异步、低频触发 |
| 失败模式 | 缓慢漂移 | 错误回环→灾难性形变（见陷阱） |

> **对比性思维**：回环检测有两大流派。**基于外观（appearance-based）**：把每帧点云编码成一个描述子（如 Scan Context、M2DP），用描述子相似度找候选回环——快，但易受视角变化影响。**基于几何（geometry-based）**：直接对候选帧做配准（如 ICP），用配准成功与否确认回环——准，但慢。实践中常**两段式**：先用描述子快速筛候选，再用配准精确验证。本章实现一个教学版：用"位置邻近 + ICP 验证"的简化几何方案（够小场景），把 Scan Context 等描述子留作进阶。

### 历史：从滤波 SLAM 到因子图

早期 SLAM 用扩展卡尔曼滤波（EKF-SLAM）维护机器人位姿和所有路标的联合协方差，状态维度随地图增长爆炸（$O(N^2)$）。2006 年前后，图优化（graph-based SLAM）成为主流：把 SLAM 建模成一个**图**——节点是待估变量（位姿），边是约束（里程计、回环）——求解一个非线性最小二乘。Dellaert 等人提出的 **因子图（factor graph）** 及其增量求解器 **iSAM/iSAM2** 把这套理论工程化，封装成 **GTSAM**（Georgia Tech Smoothing And Mapping）库。LIO-SAM（Shan et al., 2020）正是用 GTSAM 把 LIO 里程计因子和回环因子组成因子图，做全局优化。本章沿用这条路线，用 GTSAM 搭一个最小位姿图。

### 理论：位姿图与因子图

**位姿图（pose graph）** 是因子图的一个特例——所有变量都是位姿，没有路标。它包含两类边：

- **里程计因子（odometry factor）**：连接相邻关键帧 $i,i{+}1$，约束它们的相对位姿等于里程计测得的 $\Delta\boldsymbol T_i$。这类因子很多、很密，但单个约束的不确定度小。
- **回环因子（loop factor）**：连接两个时间相距很远但空间邻近的关键帧 $i,j$，约束它们的相对位姿等于回环配准测得的 $\Delta\boldsymbol T_{ij}$。这类因子稀少，但跨越长时间跨度，是消漂移的关键。

优化目标是找一组全局位姿 $\{\boldsymbol T_i\}$，使所有因子的残差加权平方和最小：

$$
\{\boldsymbol T_i^*\} = \arg\min_{\{\boldsymbol T_i\}}
\sum_{(i,j)\in\mathcal E}
\big\| \mathrm{Log}\big( \Delta\boldsymbol T_{ij}^{-1}\,(\boldsymbol T_i^{-1}\boldsymbol T_j) \big) \big\|^2_{\boldsymbol\Sigma_{ij}}
$$

每个因子的残差是"测量的相对位姿"与"当前估计的相对位姿"之差（在 $SE(3)$ 流形上用 $\mathrm{Log}$ 取差），$\boldsymbol\Sigma_{ij}$ 是该测量的协方差（里程计因子的 $\boldsymbol\Sigma$ 小、可信；回环因子的 $\boldsymbol\Sigma$ 据配准质量定）。这是一个 $SE(3)$ 上的非线性最小二乘，GTSAM 用 Levenberg-Marquardt 或 iSAM2 求解。

> **多视角理解**：位姿图优化可以从三个角度理解。（1）**弹簧网络视角**：每条边是一根弹簧，里程计弹簧硬（误差小），回环弹簧把远处两点拉到一起。系统松弛到能量最低的构型，就是把漂移均摊开。（2）**约束满足视角**：里程计给出一长串"局部应该这样"的约束，回环加一条"首尾应该重合"的约束，优化找一个同时尽量满足所有约束的折中。（3）**概率视角**：每个因子是一个高斯似然，乘起来是联合后验，最大后验估计就是上面的加权最小二乘。三个视角对应同一个优化问题——理解任一个都行，但弹簧视角最直观。

### 代码走读：构建位姿图（GTSAM）

```cpp
// src/loop_closure.cpp  ——  用 GTSAM 维护位姿图
#include "mini_lio/loop_closure.hpp"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace mini_lio {

// 每来一个新关键帧：加节点 + 加里程计因子（连上一帧）
void PoseGraph::add_keyframe(int id, const gtsam::Pose3& odom_pose) {
  initial_.insert(id, odom_pose);                 // 用里程计位姿做初值
  if (id == 0) {
    // 第一个关键帧加先验因子，锚定全局坐标系（否则规范自由度未定）
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-6,1e-6,1e-6,1e-8,1e-8,1e-8).finished());
    graph_.addPrior(0, odom_pose, prior_noise);
  } else {
    // 里程计因子：约束 id-1 与 id 的相对位姿 = 里程计增量
    gtsam::Pose3 rel = last_pose_.between(odom_pose);   // T_{i-1}^{-1} T_i
    auto odom_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-4,1e-4,1e-4,1e-4,1e-4,1e-4).finished());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(id - 1, id, rel, odom_noise));
  }
  last_pose_ = odom_pose;
}

// 加一条回环因子：约束帧 i 与帧 j 的相对位姿 = ICP 配准结果
void PoseGraph::add_loop(int i, int j, const gtsam::Pose3& rel_icp, double fitness) {
  // 噪声据配准质量自适应：fitness 越好（残差越小）越可信
  auto loop_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << fitness,fitness,fitness,fitness,fitness,fitness).finished());
  // 鲁棒核包裹：即使是错误回环，Huber 也能限制其破坏力
  auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.0), loop_noise);
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(i, j, rel_icp, robust));
}

}  // namespace mini_lio
```

> **本质洞察**：第一个关键帧的**先验因子**（`addPrior`）常被新手漏掉，但它不可或缺。位姿图只约束**相对**位姿，整个图可以在世界系里整体平移旋转而不改变任何因子残差——这叫**规范自由度（gauge freedom）**。不锚定它，优化问题就是欠定的（Hessian 奇异），求解器会报错或给出乱七八糟的结果。先验因子把第 0 帧钉死在原点，消除规范自由度，问题才良定。**任何位姿图都必须有至少一个先验（或固定一个节点），这是图优化的"接地线"。**

### 代码走读：回环检测与触发优化

```cpp
// src/loop_closure.cpp（续）  ——  位置邻近检测 + ICP 验证 + 触发优化
namespace mini_lio {

// 简化版回环检测：找时间上久远、空间上邻近的历史关键帧，用 ICP 验证
std::optional<LoopResult> PoseGraph::detect_loop(int cur_id, const CloudPtr& cur_scan) {
  const gtsam::Pose3 cur_pose = result_.at<gtsam::Pose3>(cur_id);
  for (int j = 0; j < cur_id - kMinFrameGap; ++j) {     // 只看 50+ 帧之前的（避免邻帧误判）
    const gtsam::Pose3 cand = result_.at<gtsam::Pose3>(j);
    const double dist = (cur_pose.translation() - cand.translation()).norm();
    if (dist > kLoopRadius) continue;                    // 空间不邻近，跳过

    // 几何验证：把当前帧与候选帧的局部子图做 ICP，配准成功才算真回环
    Eigen::Matrix4f T_init = relative_guess(cur_pose, cand);
    float fitness; Eigen::Matrix4f T_icp;
    if (icp_align(cur_scan, keyframe_cloud_[j], T_init, T_icp, fitness)
        && fitness < kFitnessThresh) {                   // 残差够小才接受
      return LoopResult{cur_id, j, to_pose3(T_icp), fitness};
    }
  }
  return std::nullopt;                                    // 没找到回环
}

// 触发全局优化：把所有因子交给 LM 求解，回写优化后的位姿
void PoseGraph::optimize() {
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(50);
  result_ = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
  initial_ = result_;                                    // 下次以本次结果为初值（热启动）
}

}  // namespace mini_lio
```

注意 `detect_loop` 的两道关卡缺一不可：**时间间隔** `kMinFrameGap`（只看久远的帧，否则相邻帧位置当然邻近，会误判成"回环"）和 **ICP 几何验证**（位置邻近不等于真回到原地——可能是平行走廊，必须配准确认）。

### 配置详解：回环参数

```yaml
# config/params.yaml （回环部分，可选模块）
loop_closure:
  enable: false            # 默认关闭；需要全局一致地图时打开
  min_frame_gap: 50        # 候选帧须与当前帧相隔的最少关键帧数（防邻帧误判）
  loop_radius: 10.0        # 空间邻近阈值（米）：超此距离不视为候选回环
  fitness_thresh: 0.3      # ICP 配准残差阈值（米）：超此判定回环无效
  optimize_every: 5        # 每检测到几个回环触发一次全局优化（控制频率）
  icp_max_iter: 30         # 回环验证 ICP 的最大迭代次数
```

| 配置项 | 默认值来源 | 调大/调小的影响 | 与其他配置的耦合 |
|--------|-----------|----------------|-----------------|
| `min_frame_gap` | 经验 30–50 帧 | 调小→易把邻帧误判为回环（灾难）；调大→错过短环 | 与关键帧抽取频率耦合 |
| `loop_radius` | 略大于漂移量级 | 调大→候选多、召回高但误检多、慢；调小→漏掉真回环 | 与里程计漂移率耦合：漂得多要放大 |
| `fitness_thresh` | 与 §50.7 `max_residual` 同量级 | 调小→只接受高质量回环、准但召回低；调大→召回高但易引入错误回环 | 决定回环假阳性率，最敏感参数 |
| `optimize_every` | 平衡实时与一致性 | 调小→每回环都优化、一致性好但卡顿；调大→优化少、可能积累形变 | 与计算预算耦合 |

> **理论-工程桥接**：`fitness_thresh` 是整个回环模块**最危险**的旋钮。它直接决定假阳性（false positive）率——一个错误的回环约束会把"两个其实不是同一地点"的帧强行拉到一起，优化器忠实地满足这个错误约束，结果把整条轨迹扭曲成灾难性的形状（见陷阱）。宁可漏检（保守，`fitness_thresh` 设小），不可误检。这与里程计"宁可慢漂不可突变"的哲学一脉相承——**SLAM 后端对错误约束的容忍度极低，因为全局优化会把局部错误放大成全局形变**。

### 运行验证

- **回环触发**：跑一段含明显回环的轨迹（绕一圈回起点），打印检测到的回环对 `(i,j)` 及其 ICP fitness。应在回到起点附近时触发，fitness 应明显小于阈值。
- **优化效果**：对比优化前后的轨迹——优化前首尾有明显错位（gap），优化后首尾闭合、错位消失。可视化地图应从"重影双层"变成"单层清晰"。
- **无回环退化**：在无回环的直线轨迹上跑，应**不触发**任何回环（`detect_loop` 始终返回空），位姿图退化成纯里程计链，优化不改变结果。这验证了"没有回环时不乱加约束"。
- **鲁棒性**：人为注入一个错误回环（强行连两个不相关的帧），观察 Huber 鲁棒核是否限制了它对轨迹的破坏（对比不加鲁棒核时的灾难性形变）。

### ⚠️ 常见陷阱

> **算法陷阱：回环检测只看距离不做几何验证**
> 错误做法：发现两帧位置邻近（`dist < loop_radius`）就直接加回环因子，跳过 ICP 验证。
> 现象：平行走廊、对称建筑、重复结构处疯狂误检，优化后轨迹被扭成麻花，地图彻底崩坏。
> 根本原因：空间邻近是回环的**必要非充分**条件。机器人可能在平行的另一条走廊里，位置坐标接近但根本不是同一地点；位置坐标本身还来自有漂移的里程计，更不可靠。
> 正确做法：位置邻近只用于**筛候选**，必须再用 ICP/NDC 等几何配准验证，且 fitness 严格达标才接受。两道关卡缺一不可。

> **算法陷阱：位姿图没有先验/锚点导致规范自由度未消**
> 错误做法：只加里程计和回环的 `BetweenFactor`，忘了给第一帧加 `addPrior`。
> 现象：GTSAM 优化报 "Indeterminant linear system"（Hessian 奇异），或结果整体漂到离谱位置。
> 根本原因：相对位姿因子对整图的全局平移旋转不敏感（规范自由度），不固定任一节点则优化问题欠定。
> 正确做法：给第一个关键帧加一个高置信度先验因子锚定原点；或在 GTSAM 里用 fix 把第 0 帧设为常量。这是位姿图的"接地线"，必加。

> **工程陷阱：回环优化与里程计主循环争抢同一份位姿，未做同步**
> 错误做法：回环优化在后台改 `result_`，主循环同时读它做下一帧预测，无锁。
> 现象：偶发位姿跳变、读到半更新的状态、崩溃。
> 根本原因：全局优化耗时较长（几十毫秒到秒级），与高频里程计主循环并发访问共享位姿，存在数据竞争。
> 正确做法：回环优化在独立线程做，优化完成后通过加锁/双缓冲一次性原子地把修正量应用回主循环（或只在两帧之间的安全点切换）。本章简化版同步阻塞，进阶版需此机制。

> **概念误区：以为回环越多越好（激进检测）**
> 新手想法："多检测几个回环，约束越多轨迹越准。"
> 实际上：回环约束的价值高度依赖**正确性**。一个正确回环消大量漂移；一个错误回环造成全局灾难。激进检测必然提高假阳性率。
> 正确理解：回环检测的目标是**高精确率（precision）而非高召回率（recall）**。宁可漏掉一些真回环（轨迹仅多些漂移），也绝不能引入错误回环（轨迹崩坏）。鲁棒核是兜底，但根子上要靠保守的检测阈值。

### 练习

1. **（位姿图搭建）** 用 GTSAM 手搓一个 4 节点的方形轨迹位姿图：4 条里程计因子（每条带小角度误差，使轨迹画出来不闭合）+ 1 条回环因子（连接末节点和首节点）。优化前后分别打印 4 个节点坐标，验证优化把累积误差均摊、首尾闭合。
2. **（假阳性实验）** 故意把 `fitness_thresh` 从 0.3 调到 2.0（极宽松），在一个有平行走廊的数据上跑，记录误检的回环数和优化后轨迹的形变。再调回 0.3，对比。用这个实验说服自己"精确率优先"。
3. **（鲁棒核进阶）** 对比 `noiseModel::Robust`（Huber）和普通高斯噪声模型：人为加入 1 个错误回环，分别用两种噪声模型优化，可视化轨迹。解释 Huber 核如何通过"大残差降权"限制错误回环的破坏，以及它为什么不能完全替代保守检测。

---

## §50.9 系统集成：主循环与线程模型 ⭐⭐⭐

### 模块功能与在管线中的位置

前面八节把六大模块逐个造好了——预处理、IMU 积分、去畸变、iESKF、ikd-Tree、配准、回环。但散落的模块不是系统。本节是 Capstone 的"收口"：把它们按正确的时序和线程编排，组装成 `lio_node.cpp` 里那个能真正跑起来的闭环。这一节几乎不引入新算法，而是回答一个工程问题——**异步到达的两路传感器数据（10 Hz LiDAR + 200 Hz IMU），如何同步、如何驱动各模块、如何在不同步的线程间安全传递状态**。这正是 §50.1 时序图的代码落地，也是把"算法 demo"变成"系统"的临门一脚。

### 动机：为什么集成本身是个难题

新手常以为"模块都写好了，调用一下就行"。但 LIO 的集成有三个非平凡的难点：

1. **时序同步**：IMU 200 Hz、LiDAR 10 Hz，两路数据各自到达、时间戳不对齐。一帧 LiDAR 覆盖一段时间窗（0.1 s），这段窗内有约 20 个 IMU 样本。必须把"这一帧 LiDAR 对应哪些 IMU"切准——切多了用到未来数据（因果错误），切少了去畸变不全。
2. **状态流转**：IMU 积分产出预测先验 → 喂给 iESKF → iESKF 收敛后更新地图 → 更新后的位姿又是下一段 IMU 积分的起点。这是一个有严格依赖的环，顺序错一步全盘错。
3. **线程安全**：若把可选的回环优化放到后台线程，它会异步修改全局位姿，而主循环正在用这些位姿做预测——共享可变状态的并发访问，是段错误和"薛定谔 bug"的温床。

> **本质洞察**：集成的本质是**管理数据依赖和时间因果**。每个模块都假设它的输入已经就绪、正确、对齐——预处理假设拿到完整帧，iESKF 假设拿到正确的预测先验和去畸变点云，地图更新假设拿到收敛的位姿。集成层的职责就是**让每个模块的输入假设在调用那一刻真的成立**。把这件事想成"给每个模块准备好它需要的一切再叫它"，集成的逻辑就清晰了。这也是为什么 §50.1 要先画时序图——**时序图就是集成层的规格说明书**。

### 反面：单线程串行 vs 多线程并发

| 架构 | 实现难度 | 实时性 | 线程安全风险 | 适用 |
|------|---------|--------|------------|------|
| 全单线程串行 | 低 | 受限（回环优化会卡住里程计） | 无 | 教学、小场景、本章主线 |
| 里程计单线程 + 回环单独线程 | 中 | 好（回环不阻塞里程计） | 中（需同步位姿） | 准产品级、LIO-SAM 风格 |
| 全多线程（每模块一线程 + 队列） | 高 | 最好 | 高（多处同步） | 高吞吐产品，第51章 |

> **对比性思维**：本章主线选**全单线程串行**——所有模块在一个循环里顺序跑。优点是没有任何线程同步问题，逻辑一目了然，最适合先把系统跑通、把算法调对。代价是回环优化（耗时）会暂时卡住里程计。这个取舍是刻意的：**先用单线程把正确性立住，再谈多线程的性能**。等你在第51章需要榨实时性时，再把回环拆到后台线程——那时你已经有了一个"正确的基线"来对照，调试并发问题才有锚点。**永远不要在还没跑通的系统上做并发优化。**

### 理论：传感器同步策略

LIO 的同步以**LiDAR 帧为节拍**。每当一帧 LiDAR 到达（覆盖时间窗 $[t_{\text{begin}}, t_{\text{end}}]$），我们要做的是：

1. 从 IMU 缓冲队列里取出落在 $[t_{\text{last}}, t_{\text{end}}]$ 的所有 IMU 样本（$t_{\text{last}}$ 是上一帧处理到的时刻）。
2. 用这些 IMU 样本做前向传播（§50.3），得到帧末时刻的预测先验，以及帧内每个时刻的位姿（供 §50.4 去畸变用）。
3. 用预测先验和去畸变点云跑 iESKF（§50.5 + §50.7），收敛得到帧末位姿。
4. 把配准后的点云并入地图（§50.6），更新 $t_{\text{last}} \leftarrow t_{\text{end}}$。

关键是**缓冲与对齐**：IMU 高频先到，必须缓存；处理一帧 LiDAR 时才"消费"对应时间窗的 IMU。这是典型的**生产者-消费者**模式——IMU 回调是生产者（往队列塞），LiDAR 处理是消费者（按时间窗取）。

> **多视角理解**：传感器同步可以从三个角度看。（1）**时间轴视角**：把两路数据画在同一条时间轴上，LiDAR 帧是一段段区间，IMU 是密集的点，同步就是"把落在区间里的点配给这个区间"。（2）**队列视角**：IMU 进队列，LiDAR 来了就从队头取到 $t_{\text{end}}$ 为止，取完丢弃（不再需要）。（3）**因果视角**：处理 $t_{\text{end}}$ 的 LiDAR 时，只能用 $\le t_{\text{end}}$ 的 IMU——绝不能用未来的数据，否则违反因果、在线系统根本拿不到。三个视角共同约束出唯一正确的同步逻辑。

### 代码走读：主循环（单线程串行）

```cpp
// src/lio_node.cpp  ——  LIO 系统主循环（传感器同步 + 模块编排）
#include "mini_lio/lio_node.hpp"

namespace mini_lio {

// IMU 回调（生产者）：仅入队，不做处理（处理留给 LiDAR 节拍）
void LioNode::on_imu(const IMUData& imu) {
  std::lock_guard<std::mutex> lk(buf_mutex_);
  imu_buf_.push_back(imu);
}

// LiDAR 回调（消费者）：一帧到达即驱动整条流水线
void LioNode::on_lidar(const CloudPtr& raw_scan, double t_begin, double t_end) {
  // —— 步骤 0：取出本帧时间窗内的 IMU（同步对齐）——
  std::vector<IMUData> imus;
  if (!fetch_imu_window(t_last_, t_end, imus)) return;   // IMU 没到齐，等下一帧

  // —— 步骤 1：预处理（§50.2）原始帧 → 干净稀疏帧 ——
  CloudPtr scan = preproc_.process(raw_scan);

  // —— 步骤 2：IMU 前向传播（§50.3），产出预测先验 + 帧内位姿轨迹 ——
  std::vector<PoseStamped> traj;                          // 帧内各时刻位姿（去畸变用）
  imu_integrator_.propagate(state_, P_, imus, t_last_, t_end, traj);

  // —— 步骤 3：反向传播去畸变（§50.4），把帧统一到帧末时刻 ——
  CloudPtr undistorted = deskew_.compensate(scan, traj, t_end);

  // —— 步骤 4：iESKF 更新（§50.5 + §50.7 残差 + §50.6 地图）——
  if (!ieskf_.update(state_, P_, undistorted, map_)) {
    spdlog::warn("iESKF 更新失败（有效匹配不足），本帧仅用 IMU 预测");
  }

  // —— 步骤 5：把配准后的点并入 ikd-Tree 地图（§50.6）——
  CloudPtr world = transform_to_world(undistorted, state_);
  map_.add_points(world);                                 // 增量插入 + 滑窗删除

  // —— 步骤 6：关键帧抽取 + 回环（§50.8，可选）——
  if (loop_enabled_ && is_keyframe(state_)) {
    pose_graph_.add_keyframe(kf_id_, to_pose3(state_));
    if (auto loop = pose_graph_.detect_loop(kf_id_, world)) {
      pose_graph_.add_loop(loop->cur, loop->ref, loop->rel, loop->fitness);
      pose_graph_.optimize();                             // 单线程：此处会短暂阻塞
      apply_correction(pose_graph_.latest());             // 把全局修正写回 state_/map_
    }
    ++kf_id_;
  }

  // —— 步骤 7：输出 + 推进时间 ——
  publish_odometry(state_, t_end);
  t_last_ = t_end;                                        // 推进到本帧末，供下帧切 IMU
}

}  // namespace mini_lio
```

> **本质洞察**：这段主循环的七个步骤，**顺序不能乱**。它精确复刻 §50.1 的时序图：IMU 先传播（产出先验）→ 再去畸变（用先验里的帧内位姿）→ 再 iESKF（用先验和去畸变点）→ 再更新地图（用收敛位姿）→ 最后回环。每一步的输入都是前一步的输出，是一条严格的依赖链。**把这七步背下来，你就掌握了任何 LIO 系统的骨架**——FAST-LIO、LIO-SAM、Point-LIO 的主循环都是这个结构的变体，区别只在某步用了什么具体算法。

### 代码走读：IMU 时间窗切分

同步的核心是 `fetch_imu_window`——它从缓冲队列里精确切出本帧对应的 IMU，处理边界插值：

```cpp
// src/lio_node.cpp（续）  ——  从缓冲取本帧时间窗的 IMU（含边界处理）
namespace mini_lio {

bool LioNode::fetch_imu_window(double t_begin, double t_end,
                               std::vector<IMUData>& out) {
  std::lock_guard<std::mutex> lk(buf_mutex_);
  // 必须等到 IMU 覆盖了整个帧末，否则会用"残缺"的 IMU 段（因果/完整性）
  if (imu_buf_.empty() || imu_buf_.back().t < t_end) return false;

  out.clear();
  while (!imu_buf_.empty() && imu_buf_.front().t < t_begin)
    imu_buf_.pop_front();                       // 丢弃比本帧还早的（已被上帧消费）

  for (const auto& imu : imu_buf_) {
    if (imu.t > t_end) break;                   // 只取 ≤ t_end 的（不用未来数据）
    out.push_back(imu);
  }
  if (out.size() < 2) return false;             // 至少两点才能积分

  // 注意：t_begin / t_end 处常需对 IMU 线性插值，使积分区间端点严格对齐
  // （此处略，见仓库完整实现的 interpolate_imu）
  return true;
}

}  // namespace mini_lio
```

注意 `if (imu_buf_.back().t < t_end) return false;` 这行——它强制**等到 IMU 覆盖整个帧末才处理**。在线运行时 IMU 和 LiDAR 的到达有抖动，有时 LiDAR 帧先到、对应的 IMU 还没来齐，此时必须等待而非用残缺数据硬算。这是从"离线跑 bag"过渡到"在线实时"时最容易翻车的地方。

### 配置详解：系统集成参数

```yaml
# config/params.yaml （系统集成部分）
system:
  lidar_freq: 10           # LiDAR 帧率（Hz），决定主循环节拍
  imu_freq: 200            # IMU 采样率（Hz），决定每帧 IMU 样本数（≈20）
  keyframe_dist: 1.0       # 关键帧抽取：位移超此距离（米）抽一帧
  keyframe_angle: 0.2      # 关键帧抽取：转角超此角度（rad）抽一帧
  imu_buffer_size: 2000    # IMU 缓冲队列上限（防内存无限增长）
  enable_loop_thread: false # 回环是否独立线程（true=准产品，false=单线程主线）
```

| 配置项 | 默认值来源 | 调大/调小的影响 | 与其他配置的耦合 |
|--------|-----------|----------------|-----------------|
| `keyframe_dist` | 经验 1–2 m | 调小→关键帧密、回环候选多但优化重；调大→关键帧疏、可能漏回环 | 与 §50.8 `loop_radius` 耦合 |
| `keyframe_angle` | 经验 0.1–0.3 rad | 原地转弯时靠它抽帧（位移为零） | 与 `keyframe_dist` 取或关系 |
| `imu_buffer_size` | 约 10 帧的 IMU 量 | 太小→IMU 抖动时溢出丢数据；太大→内存浪费 | 与 `imu_freq`、抖动容忍度耦合 |
| `enable_loop_thread` | 默认 false | true→实时性好但需处理同步；false→简单但回环卡循环 | 与 §50.8 同步陷阱直接相关 |

> **理论-工程桥接**：关键帧抽取（`keyframe_dist` + `keyframe_angle`）是个朴素但关键的工程决策。我们不可能把每一帧都加进位姿图（太密、优化爆炸），而是**空间均匀采样**——走够远或转够多才抽一帧当关键帧。两个条件取"或"是因为原地转弯时位移为零、靠角度触发，直线行驶时角度不变、靠位移触发。这保证了关键帧在空间上分布均匀，既不冗余也不漏。第51章在真实数据上你会看到，关键帧间距直接影响回环召回率和优化规模。

### 运行验证

- **端到端跑通**：喂一段自造的模拟数据（直线 + 转弯），系统应连续输出里程计，轨迹平滑无跳变，与真值对比 ATE（绝对轨迹误差）在分米级。这是 Capstone 的"系统跑通"里程碑。
- **同步正确性**：打印每帧用到的 IMU 样本数，10 Hz LiDAR + 200 Hz IMU 应稳定在约 20 个/帧。若忽多忽少或为 0，说明时间窗切分有问题。
- **退化优雅降级**：在某帧人为制造"有效匹配不足"（喂空地图或全离群点），系统应打 warn 并退回纯 IMU 预测，**不崩溃、不跳变**，下一帧恢复正常。这验证了步骤 4 的容错。
- **回环闭合（开 loop）**：跑含回环的轨迹，开启 `enable: true`，观察回环触发时轨迹被拉回、地图从重影变单层。

### ⚠️ 常见陷阱

> **工程陷阱：处理 LiDAR 帧时不检查 IMU 是否覆盖帧末**
> 错误做法：LiDAR 一到就立刻处理，用当时缓冲里**已有**的 IMU（可能还没覆盖到 $t_{\text{end}}$）。
> 现象：离线跑 bag 一切正常，一上实机就轨迹乱跳、去畸变错乱——因为实时数据到达有抖动。
> 根本原因：在线时 IMU 和 LiDAR 异步到达，LiDAR 帧可能先于其对应的尾部 IMU 到达，此时用残缺 IMU 段积分，帧末预测和帧内去畸变都错。
> 正确做法：处理前检查 `imu_buf_.back().t >= t_end`，不满足就等待（返回，下个回调再试）。把"等数据齐"作为硬前置条件。

> **工程陷阱：主循环步骤顺序颠倒（先更新地图后跑 iESKF）**
> 错误做法：图省事把"并入地图"放到 iESKF 之前，或用未收敛的位姿先建图。
> 现象：地图被错误位姿污染，越跑越歪，最终发散。
> 根本原因：地图是 iESKF 的观测来源，必须用**收敛后**的位姿把点并进去；用未收敛位姿建图会把误差喂回观测，形成正反馈发散。
> 正确做法：严格按"IMU 传播 → 去畸变 → iESKF 收敛 → 用收敛位姿建图"的顺序，地图更新永远在 iESKF 之后。

> **工程陷阱：IMU 缓冲队列无上限，长时间运行内存爆炸**
> 错误做法：IMU 回调无脑 `push_back`，从不清理。
> 现象：系统跑几小时后内存持续上涨直至 OOM 被杀。
> 根本原因：若某种原因 LiDAR 停了或处理变慢，IMU 仍以 200 Hz 涌入队列，消费速度跟不上生产速度，队列无限膨胀。
> 正确做法：给队列设上限 `imu_buffer_size`，超限时丢弃最旧样本并告警；正常情况下每帧消费会及时清空已用样本，队列稳定在约 10 帧的量。

> **概念误区：以为单线程一定比多线程慢、不可用**
> 新手想法："不上多线程怎么实时？必须一开始就并发。"
> 实际上：现代 CPU 单核足以让简化 LIO 在 10 Hz 下实时运行（前提是 §50.6/§50.7 的热点用 TBB 内部并行）。过早引入多线程只会让你在还没跑通时就陷入并发调试地狱。
> 正确理解：先单线程把正确性立住，把模块内部的热点（如 `build_residuals`）用 TBB 并行；只有当 Profiling（第51章）证明单线程确实达不到帧率，才把回环等耗时模块拆到独立线程。**并发是优化手段，不是起点。**

### 练习

1. **（同步实现）** 实现 `fetch_imu_window` 的边界插值版：当 $t_{\text{begin}}$、$t_{\text{end}}$ 落在两个 IMU 样本之间时，对加速度和角速度做线性插值，使积分区间端点严格对齐帧时间。写测试验证插值后的区间端点时间戳精确等于 $t_{\text{begin}}/t_{\text{end}}$。
2. **（端到端集成）** 把 §50.2–§50.7 的模块在 `lio_node.cpp` 里串成单线程主循环，喂一段自造模拟数据（已知真值轨迹），输出估计轨迹，计算 ATE 和 RPE（相对位姿误差）。这是 Capstone 的核心交付物。
3. **（线程化进阶）** 把回环模块（§50.8）拆到独立线程：主循环只负责抽关键帧并塞进一个线程安全队列，回环线程从队列取、检测、优化，优化结果通过双缓冲原子地回写。讨论"优化期间主循环又前进了若干帧"时，如何把全局修正正确地应用到这些新帧上（提示：修正是作用在优化时刻的位姿上，需把之后的增量重新叠加）。

---

## §50.10 系统调参指南 ⭐⭐⭐

### 这一节解决什么问题

系统跑通只是起点。真正让 LIO"好用"的，是把散落在六大模块里的几十个参数调到协同工作。新手面对一整页 `params.yaml` 常无从下手——改哪个？往哪改？改了看什么？本节给一套**系统化的调参方法论**：不是逐个乱试，而是按"先标定、再噪声、后算法"的顺序，每一步有明确的观测指标和因果逻辑。这是把前九节的"会搭"变成"调得好"的关键一节，也是通往第51章"在真实数据上做评估优化"的桥梁。

### 动机：为什么调参不能靠瞎试

LIO 的参数不是独立的——它们通过物理和算法耦合在一起。`obs_noise`（信 LiDAR 多少）和 IMU 噪声（信预测多少）是一对此消彼长；`voxel_leaf_size`（扫描分辨率）、`downsample_map`（地图分辨率）、`plane_thresh`（平面阈值）三者必须协调；`loop_radius` 要随里程计漂移率调整。瞎试一个参数，往往破坏另一处的平衡，结果按下葫芦浮起瓢。**正确的调参是顺着耦合链，从"物理上最确定的"调到"算法上最灵活的"。**

> **本质洞察**：调参的优先级遵循一条铁律——**越接近物理真实、越该先钉死；越是算法折中、越该后调**。外参和时间偏移是物理量（传感器怎么装的、时钟差多少），它们有客观真值，必须先标定准；标定错了，后面调什么噪声都是在错误的几何上打转。噪声参数次之（有物理依据但需结合场景）。算法参数（迭代次数、平衡因子、回环阈值）最灵活，最后调。**顺序错了，你会用算法参数去补偿标定错误，越补越乱。**

### 系统化调参顺序

下表给出推荐的调参顺序、每步的观测指标和判据。**严格按顺序，前一步没调对不要进下一步**：

| 顺序 | 调什么 | 关键参数 | 观测指标 | 判据 | 所属节 |
|------|--------|---------|---------|------|--------|
| 1 | 外参标定 | `R_LtoI`, `p_LtoI` | 静止时点云是否"晃动"；平面是否变厚 | 静止建图墙面应锐利不重影 | §50.4/§50.7 |
| 2 | 时间偏移 | `time_offset` | 转弯时点云是否"拖尾"/扭曲 | 快速转弯墙面不弯曲 | §50.4 |
| 3 | IMU 噪声 | `acc/gyr_noise`, `bias_*` | 预测轨迹平滑度；偏置估计收敛性 | 静止时偏置稳定、不漂 | §50.3 |
| 4 | 观测噪声 | `obs_noise` | iESKF 收敛速度；轨迹抖动 | 收敛 3–5 次、轨迹平滑 | §50.5 |
| 5 | 分辨率三件套 | `voxel_leaf`, `downsample_map`, `plane_thresh` | 有效残差比例；单帧耗时；精度 | 有效比例 60–90%、实时 | §50.2/§50.6/§50.7 |
| 6 | 鲁棒性 | `max_residual`, `huber_delta` | 动态场景轨迹稳定性 | 行人/车辆经过不带偏 | §50.7 |
| 7 | 回环（如启用） | `loop_radius`, `fitness_thresh` | 回环精确率 | 无误检、真回环能闭合 | §50.8 |

> **多视角理解**：这个顺序可以从三个角度理解其合理性。（1）**因果链视角**：外参/时间偏移在数据流最上游，错了污染下游一切，必须先治。（2）**可观测性视角**：标定误差表现为"静止/匀速时仍有系统性畸变"，最易诊断、最该先排除；噪声/算法参数的影响更微妙，要在标定干净后才看得清。（3）**自由度视角**：先固定物理自由度（外参、时差），再调统计自由度（噪声），最后调算法自由度（迭代、阈值），逐层收敛，避免高层参数补偿低层错误。

### 故障到参数的快速映射

调参时最常见的不是"从零调"，而是"系统有某个毛病、该动哪个参数"。下表是症状到参数的速查（与本章末故障排查手册互补，这里聚焦"调参"动作）：

| 症状 | 最可能的参数 | 调整方向 | 副作用提醒 |
|------|------------|---------|-----------|
| 轨迹整体漂移快 | `obs_noise` | 调小（更信 LiDAR） | 太小会在退化场景过信 LiDAR 抖动 |
| 轨迹高频抖动 | `obs_noise` / `voxel_leaf` | `obs_noise` 调大 / 降采样调粗 | 过度会变迟钝、丢细节 |
| 静止时墙面变厚/重影 | 外参 / `time_offset` | 重新标定（非调噪声） | 别用噪声去补标定错误 |
| 转弯时点云拖尾 | `time_offset` | 标定时间偏移 | 去畸变依赖它，须先标准 |
| 单帧耗时超 100 ms | `voxel_leaf`/`downsample_map`/`max_iterations` | 降采样调粗 / 迭代调少 | 过粗丢精度、过少不收敛 |
| 有效残差比例过低 | `plane_thresh`/`downsample_map` | 阈值放宽 / 地图调密 | 过宽纳入非平面引入误差 |
| 动态物体带偏轨迹 | `max_residual`/`huber_delta` | 调小（更激进剔除） | 过激进会误删真约束 |
| 回环把轨迹扭坏 | `fitness_thresh` | 调小（更保守） | 过小会漏掉真回环 |

> **理论-工程桥接**：注意表里两行特意强调"重新标定，非调噪声"——静止墙面变厚、转弯拖尾这类**几何性畸变**是标定问题，根子在外参/时差错误。新手最常犯的错是看到墙面厚就去调 `obs_noise` 或 `plane_thresh`，结果用统计参数去掩盖几何错误，治标不治本，还把噪声调得不合理。**症状的"几何性 vs 统计性"决定了该动标定参数还是噪声参数**——这是调参时最该建立的判断力。

### ⚠️ 常见陷阱

> **调参陷阱：跳过标定直接调算法参数**
> 错误做法：拿到系统先调 `obs_noise`、`max_iterations`、`plane_thresh`，外参用默认值或粗略估计。
> 现象：怎么调都有残余畸变，调好一个场景换个场景又崩，陷入无尽调参。
> 根本原因：外参/时差错误是几何性的系统误差，任何统计/算法参数都补偿不了；它污染所有下游模块，是误差的根。
> 正确做法：严格按"标定 → 噪声 → 算法"的顺序。先用静止和匀速数据把外参、时差标准（静止墙面锐利、转弯不弯曲），再动后面的参数。

> **调参陷阱：一次改多个参数，无法归因**
> 错误做法：一轮同时改 `obs_noise`、`voxel_leaf`、`max_iterations` 三个，看总体效果。
> 现象：效果变了但不知道是哪个参数起的作用，下次想复现或微调无从下手。
> 根本原因：参数间有耦合，同时改多个时各自的贡献混在一起，无法建立"参数→效果"的因果。
> 正确做法：控制变量，一次只改一个参数，记录"改前值/改后值/观测指标变化"。建一张调参日志，把每次实验记下来——这也是第51章做系统评估的基础工作流。

> **调参陷阱：在一个场景调到最优就以为通用**
> 错误做法：在某段数据上把参数调到 ATE 最小，当作"最优参数"固定下来。
> 现象：换个环境（室内→室外、慢速→高速）精度大跌。
> 根本原因：最优参数依赖场景特性（点云密度、运动剧烈程度、结构化程度），单场景过拟合的参数不具普适性。
> 正确做法：在多个代表性场景上验证，找一组"各场景都不差"的鲁棒参数，而非单场景最优。需要时按场景切换参数集。第51章的多数据集评估正是为此。

### 练习

1. **（标定先行）** 录一段机器人完全静止的数据，用当前外参建图，观察墙面是否"变厚"（理想应是一个薄面）。故意把 `p_LtoI` 平移分量改错 5 cm，再建图，对比墙面厚度变化。用这个实验建立"几何畸变=标定问题"的直觉。
2. **（控制变量调参）** 在一段含转弯的数据上，固定其他参数，单独把 `obs_noise` 在 $\{0.001, 0.01, 0.1\}$ 三个值上跑，分别记录 ATE 和轨迹抖动幅度。画出"`obs_noise` vs ATE"曲线，找到甜点，并解释两端为什么都变差（太小过信 LiDAR 抖动、太大过信 IMU 漂移）。
3. **（调参日志）** 设计一个调参实验记录模板（参数名/改前/改后/观测指标/结论），完整调一遍本章 LIO（按 7 步顺序），产出一份调参日志。这份日志和你的最终参数集，就是交给第51章做真实数据评估的"基线配置"。

---

## 本章常见误解汇总

把全章散落的概念误区收拢成一张表，做最后一遍体检。每一条都是真实会让你卡住或写出隐蔽 bug 的坑：

| # | 误解 | 真相 | 出处 |
|---|------|------|------|
| 1 | 松耦合和紧耦合只是"融合层次不同"，效果差不多 | 紧耦合能按方向自动分配 LiDAR/IMU 信任，退化场景下显著更鲁棒，这是本质优势 | §50.1 |
| 2 | 降采样越狠越快越好 | 体素过大抹平几何特征，平面法向估计失准，精度断崖下跌 | §50.2 |
| 3 | 用欧拉积分（只用区间起点 IMU）够了 | 旋转剧烈时一阶截断误差大，中值积分用区间中点显著更准 | §50.3 |
| 4 | 一帧点云是"一个瞬间"拍下的 | 一帧覆盖约 0.1 s 时间窗，运动中采集，必须按时间戳去畸变 | §50.4 |
| 5 | 卡尔曼滤波预测一次更新一次就够（EKF 即可） | point-to-plane 强非线性 + 最近邻随位姿变，必须迭代重线性化（iEKF） | §50.5 |
| 6 | 每帧用 $m\times m$ 求逆算卡尔曼增益 | $m\gg n=18$，必须用降维等价公式只求逆 $18\times18$，否则实时性崩 | §50.5 |
| 7 | 惰性删除的墓碑不影响性能 | 墓碑仍占位、仍被遍历，堆积会拖慢查询，需 `alpha_del` 判据定期清理 | §50.6 |
| 8 | kNN 无条件搜两侧才"保险" | 那退化成 $O(N)$；正确剪枝（到分割面距离²比较）才有 $O(\log N)$ | §50.6 |
| 9 | 点到点和点到面残差差不多 | 点到面只约束法向自由度、放任切向滑动，更贴合物理、收敛快得多 | §50.7 |
| 10 | PCA 拟合出平面就能直接用 | 拐角处 5 点也有最小特征向量，必须回代检测是否真共面 | §50.7 |
| 11 | 残差越多估计越准，不必剔除离群点 | 动态物体/误匹配是有偏错误约束，留得越多带偏越狠，质量比数量重要 | §50.7 |
| 12 | 回环越多越好（激进检测） | 错误回环造成全局灾难性形变，回环要高精确率而非高召回率 | §50.8 |
| 13 | 位姿图不加先验也能优化 | 相对位姿因子留有规范自由度，不锚定一个节点则问题欠定、Hessian 奇异 | §50.8 |
| 14 | 必须一开始就上多线程才能实时 | 单核 + 模块内 TBB 足以实时；并发是优化手段不是起点，先单线程立正确性 | §50.9 |
| 15 | 几何畸变（墙面变厚）调噪声参数能解决 | 那是标定问题，须重标外参/时差；用统计参数补几何错误治标不治本 | §50.10 |

---

## 故障排查手册

LIO 是个长链条系统，故障往往"症状在下游、根因在上游"。下面六个结构化排查表覆盖最高频的故障模式，每个按"症状 → 可能原因 → 排查步骤 → 相关章节"组织。遇到问题先定位到对应表，按步骤逐项排除。

### 故障 1：轨迹快速漂移 / 发散

| 维度 | 内容 |
|------|------|
| **症状** | 估计轨迹明显偏离真值且越走越偏，或在某帧后突然发散到无穷 |
| **可能原因** | ① iESKF 雅可比符号/扰动模型错；② `obs_noise` 过大（过信漂移的 IMU）；③ 地图被未收敛位姿污染；④ 外参错误；⑤ 有效残差太少滤波退化成纯 IMU |
| **排查步骤** | 1. 先跑 §50.7 数值雅可比校验，排除雅可比错（最常见根因）；2. 打印每帧有效残差比例，<30% 说明地图/平面阈值/初值有问题；3. 检查主循环顺序，确认建图在 iESKF 收敛之后；4. 用静止数据验证外参（墙面应锐利）；5. 把 `obs_noise` 调小试是否收敛 |
| **相关章节** | §50.5、§50.7、§50.9、§50.10 |

### 故障 2：点云去畸变后仍扭曲 / 墙面拖尾

| 维度 | 内容 |
|------|------|
| **症状** | 快速转弯或加减速时，本应是直墙的地方在地图里弯曲、拖尾、变厚 |
| **可能原因** | ① 点的 `time` 字段丢失或为零（被 VoxelGrid 抹掉）；② `time_offset`（IMU-LiDAR 时间偏移）未标定；③ 去畸变用的帧内位姿轨迹有误；④ 外参旋转分量错 |
| **排查步骤** | 1. 打印若干点的 `time` 字段，确认落在 $[0, 0.1]$ 且非零——若为零，是降采样顺序问题（应先去畸变后降采样）；2. 录匀速直线数据，墙面应直——若直但转弯弯，是 `time_offset` 问题；3. 标定 `time_offset`；4. 检查 §50.4 反向传播是否用了正确的帧内位姿 |
| **相关章节** | §50.2、§50.4、§50.10 |

### 故障 3：iESKF 不收敛 / 收敛到错误位姿

| 维度 | 内容 |
|------|------|
| **症状** | `dx.norm()` 不下降、震荡或越来越大；或收敛了但位姿方向/镜像错乱 |
| **可能原因** | ① $\boxplus/\boxminus$ 左右乘约定不一致；② 雅可比与扰动模型不自洽；③ 最大迭代次数太少；④ 初值（IMU 预测）太差；⑤ 残差里混入大量非平面/离群约束 |
| **排查步骤** | 1. 对照 §50.5 检查 $\boxplus$（右乘 Exp）与 $\boxminus$（$\mathrm{Log}(R_b^\top R_a)$）是否成对一致；2. 跑数值雅可比校验确认自洽；3. 把 `max_iterations` 加到 10 看是否只是迭代不够；4. 打印初值与真值差距，若过大说明 IMU 传播或同步有问题；5. 检查平面质量检测与 `max_residual` 是否生效 |
| **相关章节** | §50.5、§50.7 |

### 故障 4：单帧耗时过长 / 跟不上帧率

| 维度 | 内容 |
|------|------|
| **症状** | 单帧处理超过 100 ms（10 Hz 的预算），系统延迟累积、丢帧 |
| **可能原因** | ① 卡尔曼增益用了 $m\times m$ 求逆；② ikd-Tree 退化成近线性扫描（剪枝错或没重建）；③ `build_residuals` 未并行；④ 点云按值传参深拷贝；⑤ 降采样/地图分辨率过密 |
| **排查步骤** | 1. 确认 §50.5 增益用降维公式（求逆 18×18）；2. 打印 ikd-Tree 树高，过高说明平衡失效；3. 确认 `build_residuals` 用了 `tbb::parallel_for`；4. 检查点云是否全程 `shared_ptr` 传递（grep 按值传 `CloudPtr`/`PointCloud` 的地方）；5. 用 Profiler 定位真正热点（留给第51章系统化做） |
| **相关章节** | §50.2、§50.5、§50.6、§50.7、第51章 |

### 故障 5：随机段错误 / AddressSanitizer 报警

| 维度 | 内容 |
|------|------|
| **症状** | 偶发崩溃、难复现；ASan 报 misaligned address、heap-buffer-overflow 或 data race |
| **可能原因** | ① 多份 Eigen 版本 ABI 冲突；② 固定大小 Eigen 成员未做对齐（缺 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`）；③ `build_residuals` 并行里 `push_back` 共享 vector；④ ikd-Tree 指针在重建后悬空；⑤ 回环线程与主循环并发改位姿 |
| **排查步骤** | 1. `cmake --debug-find` 确认只有一份 Eigen；2. 给含固定大小 Eigen 成员的类加对齐宏或改用 `Eigen::aligned_allocator`；3. 检查并行循环是否写共享容器（改为各写各下标）；4. 检查 ikd-Tree 重建后是否更新了所有指向旧子树的指针；5. 用 ThreadSanitizer 跑回环线程相关代码 |
| **相关章节** | §50.1（环境）、§50.6、§50.7、§50.9 |

### 故障 6：回环把好好的轨迹扭坏

| 维度 | 内容 |
|------|------|
| **症状** | 开启回环后，触发回环的瞬间轨迹突然扭曲变形，比不开回环还差 |
| **可能原因** | ① 误检（`fitness_thresh` 过松，把不相关帧当回环）；② 只看距离没做 ICP 几何验证；③ 位姿图缺先验锚点；④ 回环因子噪声设得过小（过信错误回环）；⑤ `min_frame_gap` 太小把邻帧当回环 |
| **排查步骤** | 1. 打印每个被接受回环的 `(i,j)` 和 fitness，人工核对是否真是同一地点；2. 把 `fitness_thresh` 调小（更保守）；3. 确认 §50.8 加了 `addPrior`；4. 确认回环因子套了 Huber 鲁棒核；5. 把 `min_frame_gap` 加大避免邻帧误判 |
| **相关章节** | §50.8 |

---

## 里程碑 Checklist

Capstone 不是一口气写完的。下面把整个 Mini-LIO 拆成可逐个勾掉的里程碑——每个里程碑有明确的"完成标志"（一个能跑、能验证的产出），按顺序推进，每过一关你都有一个可演示的成果，而不是憋到最后才知道对不对。

### 里程碑 M0：环境与骨架（第 1 天）

- [ ] Ubuntu + Eigen/PCL/Sophus/TBB 装好，`cmake --debug-find` 确认只有一份 Eigen（防故障 5）
- [ ] 项目骨架目录建好，`CMakeLists.txt` 能编译出空的 `mini_lio_core` 库
- [ ] GoogleTest 接好，能跑一个空的 `TEST(Smoke, Builds){}`
- [ ] **完成标志**：`cmake --build build && ctest` 全绿（哪怕只有冒烟测试）

### 里程碑 M1：预处理 + 数据类型（第 2-3 天，§50.2）

- [ ] 定义 `PointXYZIRT` 自定义点类型（含 ring/time），PCL 宏注册通过
- [ ] 实现 `Preprocessor::process`：距离过滤 + 去自身点 + 体素降采样
- [ ] 点云全程 `shared_ptr` 传递，无按值深拷贝（防故障 4）
- [ ] **完成标志**：喂一帧 12 万点，输出约 1-3 万点，可视化车体 1 m 内无点

### 里程碑 M2：IMU 积分（第 4-5 天，§50.3）

- [ ] 定义 18 维 `State`，实现 $\boxplus/\boxminus$（右乘约定）
- [ ] 实现中值积分的名义状态传播 + 误差协方差传播
- [ ] **完成标志**：GoogleTest 喂合成 IMU（已知运动），传播结果与解析真值误差 < 1%

### 里程碑 M3：去畸变（第 6 天，§50.4）

- [ ] 实现反向传播去畸变，用帧内位姿轨迹把点统一到帧末时刻
- [ ] `time` 字段全程保留（先去畸变后降采样，防故障 2）
- [ ] **完成标志**：合成"匀速直行 + 转弯"的畸变点云，去畸变后墙面变直

### 里程碑 M4：iESKF + 配准（第 7-9 天，§50.5 + §50.7）⭐ 核心关

- [ ] 实现 point-to-plane 残差与雅可比（右扰动），**数值雅可比校验通过**（相对误差 < 1e-4，防故障 1/3）
- [ ] 实现 `fit_plane`（PCA）+ 平面质量回代检测
- [ ] 实现 iESKF 迭代更新，用降维卡尔曼增益（求逆 18×18，防故障 4）
- [ ] **完成标志**：GoogleTest 给定真值位姿、合成共面点，iESKF 收敛到真值（< 1 cm / 0.5°）

### 里程碑 M5：ikd-Tree 地图（第 10-11 天，§50.6）

- [ ] 实现增量插入 + `pull_up` 下推属性维护（防故障 5）
- [ ] 实现 kNN（分支限界剪枝）+ 盒式搜索 + 惰性删除 + 局部重建
- [ ] **完成标志**：kNN 与暴力扫描**对拍完全一致**；插 10 万点后树高约 17（平衡）

### 里程碑 M6：单线程系统集成（第 12-13 天，§50.9）⭐ 系统跑通关

- [ ] 实现 `fetch_imu_window` 传感器同步（等 IMU 覆盖帧末，防故障 4 的实时坑）
- [ ] 主循环七步按正确顺序串起（建图在 iESKF 之后，防故障 1）
- [ ] 退化帧优雅降级（有效匹配不足时退回 IMU 预测，不崩溃）
- [ ] **完成标志**：喂自造模拟数据，连续输出平滑轨迹，ATE 分米级——**Capstone 核心交付物**

### 里程碑 M7：回环 + 调参（第 14 天，§50.8 + §50.10，可选/进阶）

- [ ] 位姿图加先验锚点 + 里程计因子 + 回环因子（Huber 鲁棒核，防故障 6）
- [ ] 回环检测两道关卡：时间间隔 + ICP 几何验证（防故障 6）
- [ ] 按"标定 → 噪声 → 算法"七步顺序调参，产出调参日志
- [ ] **完成标志**：含回环轨迹优化后首尾闭合、地图从重影变单层；调参日志可交付第51章

> **本质洞察**：这七个里程碑的排序不是随意的——它严格沿数据流、从"可独立验证"到"需集成验证"推进。M1-M5 每个模块都能用 GoogleTest **单独**验证正确性（这正是骨架强调单元测试的意义），M6 才做集成。**先让每块砖都合格，再砌墙**——这样集成出问题时，你能确信根因在"编排"而非"模块本身"，调试范围被极大缩小。这种"模块先单测、最后才集成"的纪律，是大型系统不失控的根本，远比 LIO 本身更通用。

---

## 本章小结

我们从零搭起了一个虽简化但**闭环可跑**的 LiDAR-Inertial 里程计。回看整章，它把前面所有 SLAM 方向的积累拧成了一根绳：

- **李群李代数**（第10章）化作 §50.3/§50.5 里的 $\boxplus/\boxminus$ 和右扰动雅可比——旋转在流形上传播、在切空间里估计。
- **状态估计与卡尔曼滤波**化作 §50.5 的 iESKF——名义状态确定性积分、误差状态滤波，加上"迭代重线性化"和"降维高效增益"两个让它实时的关键。
- **点云处理**化作 §50.2 的预处理、§50.6 的 ikd-Tree、§50.7 的 point-to-plane——从原始帧到地图约束的完整链条。
- **C++ 工程技术**化作贯穿全章的 `shared_ptr` 传点云、TBB 并行热点、现代 CMake、GoogleTest 守护数学、Sanitizer 防内存坑。

更重要的是，本章传递了一套**做系统的方法论**，它比 LIO 本身更值得带走：

> **本质洞察**：一个 SLAM 系统的形态是"**算法内核小、工程外壳大**"。真正决定精度的核心算法（iESKF + 配准）只有几百行，但要让它在真实世界稳定跑起来，需要预处理、去畸变、地图、同步、容错、调参一整圈"外壳"。新手容易高估算法、低估工程；而本章从头到尾在示范——**把系统跑通的难点，多半不在算法多深，而在每个模块的输入假设是否被工程层老老实实地满足**（数据对齐没对齐、点云拷没拷贝、雅可比自洽不自洽、地图污染没污染）。这套"先单测每块、再集成编排、最后系统调参"的纪律，是你将来做任何机器人系统都能复用的底层能力。

本章只解决了"搭出来并在自造数据上跑通"。下一章（第51章·系统集成评估与调优）接手"在 KITTI/MulRan 等真实公开数据集上评估精度（ATE/RPE）、用 Profiler 定位性能瓶颈、把参数推向产品级"——也就是把你这台"能跑的原型"打磨成"能用的系统"。

### 速查表

**主循环七步顺序（§50.9，背下来）**：

```
取IMU窗(等覆盖帧末) → 预处理 → IMU传播(产出先验+帧内位姿)
→ 反向传播去畸变 → iESKF迭代更新 → 用收敛位姿并入地图 → (可选)回环 → 输出+推进时间
```

**核心公式速查**：

| 名称 | 公式 | 出处 |
|------|------|------|
| 旋转 $\boxplus$ | $\boldsymbol R\boxplus\delta\boldsymbol\theta=\boldsymbol R\,\mathrm{Exp}(\delta\boldsymbol\theta)$ | §50.5 |
| 旋转 $\boxminus$ | $\boldsymbol R_a\boxminus\boldsymbol R_b=\mathrm{Log}(\boldsymbol R_b^\top\boldsymbol R_a)$ | §50.5 |
| 点到面残差 | $r=\boldsymbol n^\top\boldsymbol p^W+d$ | §50.7 |
| 残差对位置雅可比 | $\partial r/\partial\delta\boldsymbol p=\boldsymbol n^\top$ | §50.7 |
| 残差对旋转雅可比 | $\partial r/\partial\delta\boldsymbol\theta=-\boldsymbol n^\top\boldsymbol R(\boldsymbol p^I)^\wedge$ | §50.7 |
| 降维卡尔曼增益 | $\boldsymbol K=(\boldsymbol H^\top\boldsymbol R^{-1}\boldsymbol H+\hat{\boldsymbol P}^{-1})^{-1}\boldsymbol H^\top\boldsymbol R^{-1}$ | §50.5 |
| ikd-Tree 重建判据 | 形状 $S_{\text{重}}>\alpha_{\text{bal}}S$ 或墓碑 $N_{\text{del}}>\alpha_{\text{del}}S$ | §50.6 |
| kNN 剪枝条件 | 堆未满 或 $(\text{到分割面距离})^2<\text{第}k\text{远}^2$ | §50.6 |
| 位姿图因子残差 | $\mathrm{Log}(\Delta\boldsymbol T_{ij}^{-1}\boldsymbol T_i^{-1}\boldsymbol T_j)$ | §50.8 |

**关键参数与默认值速查**：

| 参数 | 默认 | 作用 | 节 |
|------|------|------|----|
| `voxel_leaf_size` | 0.3 m | 扫描降采样 | §50.2 |
| `obs_noise` | 0.01 | 信 LiDAR 程度（小=更信） | §50.5 |
| `max_iterations` | 5 | iESKF 单帧迭代上限 | §50.5 |
| `downsample_map` | 0.4 m | 地图分辨率（须≥扫描） | §50.6 |
| `ikd_balance_alpha` | 0.7 | ikd-Tree 形状平衡因子 | §50.6 |
| `knn_k` | 5 | 平面拟合近邻数 | §50.6/§50.7 |
| `plane_thresh` | 0.1 m | 平面质量阈值 | §50.7 |
| `max_residual` | 0.3 m | 离群残差剔除阈值 | §50.7 |
| `fitness_thresh` | 0.3 | 回环 ICP 接受阈值（最敏感） | §50.8 |
| `keyframe_dist` | 1.0 m | 关键帧抽取距离 | §50.9 |

### 知识点总表

| 节 | 知识点 | 难度 | 核心产出 | 一句话精华 |
|----|--------|------|---------|-----------|
| §50.1 | 系统架构与数据流 | ⭐⭐ | 全局视图、时序约束 | 紧耦合能按方向自动分配 LiDAR/IMU 信任 |
| §50.2 | 点云预处理 | ⭐⭐ | 自定义点类型、降采样 | 降采样是速度-精度权衡，非越狠越好 |
| §50.3 | IMU 积分与预积分 | ⭐⭐⭐ | 名义/误差状态传播 | 名义状态确定性积分、误差状态滤波 |
| §50.4 | 点云去畸变 | ⭐⭐⭐ | 反向传播运动补偿 | 一帧是时间窗不是瞬间，须按时间戳矫正 |
| §50.5 | iESKF 状态估计 | ⭐⭐⭐⭐ | $\boxplus/\boxminus$、迭代更新、降维增益 | 迭代重线性化 + 求逆 18×18 是实时关键 |
| §50.6 | ikd-Tree 局部地图 | ⭐⭐⭐ | 增量插入、剪枝查询、局部重建 | 用局部重建代替全局重建维持平衡 |
| §50.7 | point-to-plane 配准 | ⭐⭐⭐ | 法向估计、残差与雅可比 | 好残差只编码确信的约束（法向），对切向沉默 |
| §50.8 | 回环检测与位姿图 | ⭐⭐⭐ | 因子图、全局优化 | 局部靠里程计、全局靠回环；要精确率非召回率 |
| §50.9 | 系统集成 | ⭐⭐⭐ | 传感器同步、主循环、容错 | 集成的本质是管理数据依赖和时间因果 |
| §50.10 | 系统调参 | ⭐⭐⭐ | 系统化调参方法论 | 标定→噪声→算法，越接近物理越先钉死 |

### 延伸阅读

- **FAST-LIO2**（Xu et al., 2022, T-RO）：本章 iESKF + ikd-Tree 架构的来源，紧耦合直接法 LIO 的标杆。
- **ikd-Tree**（Cai et al., 2021, RAL）：增量 kd-Tree 的原始论文，§50.6 的理论基础（含异步重建细节）。
- **LIO-SAM**（Shan et al., 2020, IROS）：基于因子图的 LIO，§50.8 回环 + 位姿图优化的来源。
- **FAST-LIO**（Xu & Zhang, 2021, RAL）：降维卡尔曼增益的提出，§50.5 高效更新的出处。
- **Point-LIO**（He et al., 2023）：逐点更新的 LIO 变体，处理高速剧烈运动，可作进阶。

### 后续章节关系

| 关系 | 章节 | 说明 |
|------|------|------|
| **直接后继** | 第51章·系统集成评估与调优 | 在 KITTI/MulRan 真实数据上评估 ATE/RPE、Profiling、产品级调参 |
| **理论根基** | 第10章·SLAM 理论 | $SO(3)/SE(3)$、$\boxplus/\boxminus$、图优化的数学基础 |
| **方法补充** | 状态估计/卡尔曼滤波章节 | 误差状态滤波、协方差传播的系统理论 |
| **数据来源** | 点云处理章节 | kd-Tree、体素滤波、PCL 使用 |
| **横向对比** | 视觉 SLAM（VIO）章节 | 把本章 LiDAR 观测换成视觉重投影残差，框架同构 |

> **跨章综合题**：本章用 LiDAR 的 point-to-plane 残差驱动 iESKF。请设想把传感器换成相机：视觉的观测残差是"重投影误差"（3D 地图点投影到像素 vs 实际观测像素）。试论证——iESKF 的整个框架（名义/误差状态、$\boxplus/\boxminus$、迭代更新、降维增益）能否原样复用？需要改动的只是 §50.7 的残差和雅可比这一块吗？这个问题的答案，正是"为什么 LIO 和 VIO 本质同构"——它把本章的 SLAM 框架与视觉 SLAM 章节直接桥接起来。

---
