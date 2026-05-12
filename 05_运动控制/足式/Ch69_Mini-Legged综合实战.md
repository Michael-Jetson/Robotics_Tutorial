> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 69 章 Mini-Legged 综合实战——从仿真到实机的四足控制全栈

> **难度**: ⭐⭐⭐ | **预计学时**: 60-80 小时(3-4 周) | **前置**: Ch47-68 全部完成
>
> **一句话概要**: 本章是足式控制方向的**毕业设计**——你将从零搭建一个完整的四足控制栈,覆盖环境搭建、URDF/MJCF 建模、站立控制、MPC 集成、WBC 集成、步态实现、RL 策略训练、Sim-to-Real 部署,最终在 Unitree Go2 上实现稳定行走。

---

## 前置自测

> 📋 **答不出 2 题以上 → 先回前置章节复习**

1. **Pinocchio 基础**(Ch47): 给定 Go2 的 URDF,如何用 Pinocchio 计算第 i 只脚在世界坐标系下的位置?写出 API 调用序列。
2. **QP 求解器**(Ch50): ProxQP 和 qpOASES 的主要区别是什么?在 WBC 场景下为什么推荐 ProxQP?
3. **SRB 模型**(Ch51): Single Rigid Body 模型的 13 维扩展状态向量包含哪些量?为什么要把常数项 1 放进状态来吸收重力仿射项?
4. **WBC 公式**(Ch53): WBC 的决策变量 $(\ddot{q}, \lambda)$ 的物理含义是什么?为什么不直接把 $\tau$ 也放进去?
5. **OCS2 双线程架构**(Ch55): MPC_Node 和 MRT_Node 之间用什么通信机制?为什么不用 mutex?

---

## 本章目标

学完本章,你将能够:

1. **独立搭建一个完整的四足机器人仿真与控制开发环境**——包括 ROS 2、OCS2、Pinocchio、MuJoCo、Gazebo、IsaacLab
2. **从 URDF 到 MJCF 完成机器人模型的转换与验证**——理解碰撞几何、惯性参数、执行器建模的工程细节
3. **实现从站立到行走的完整控制管线**——PD 站立 → MPC 集成 → WBC 集成 → 步态调度
4. **使用 IsaacLab + rsl_rl 训练端到端 RL 策略**——在单卡 GPU 上完成平地行走训练，并理解训练耗时如何随显卡、环境数和任务版本变化（需要 NVIDIA GPU，推荐 RTX 3070 及以上）
5. **完成 Sim-to-Real 部署**——通过 Unitree SDK2 将控制策略部署到真实 Go2 上
6. **具备系统级调试能力**——能诊断和解决 MPC 求解慢、WBC 扭矩爆表、状态估计漂移等常见问题

---

## 69.1 项目总览 ⭐

### 动机:为什么需要一个"综合实战"章节?

前面 Ch47-68 的二十多章,每章聚焦一个模块——动力学建模、QP 求解、MPC 设计、WBC 实现、步态管理、状态估计......但真实的机器人控制系统,绝不是把这些模块简单拼接就能工作的。模块之间的**接口设计**、**时序协调**、**参数联调**,才是工程的核心难点。

打个比方:Ch47-68 教你制造了发动机、变速箱、底盘、轮胎——Ch69 要你把它们装配成一辆能开上路的车。

> **跨领域类比**：Mini-Legged 项目之于前面 22 章的知识点,就像计算机科学中的"操作系统课程大作业"之于数据结构、计算机体系结构、编译原理等课程——单独学每门课时你理解了每个模块,但只有亲手实现一个微型 OS(或微型控制栈),你才真正理解模块之间的接口设计、时序依赖和故障传播。两者的共同教训是:**系统的难点不在于任何单个模块,而在于模块间的"间隙"**——数据格式不匹配、时钟不同步、错误处理不一致,这些"间隙"只有在集成时才会暴露。

### 如果不做综合实战会怎样?

学完理论但不做综合实战的典型后果:

| 表现 | 根本原因 |
|------|---------|
| 能读懂 legged_control 源码,但自己写不出来 | 缺少"从零开始"的经验,不知道第一步该做什么 |
| 调了一周参数,机器人还是摔 | 不理解模块间耦合关系,头痛医头 |
| 仿真里跑通了,真机上完全不行 | 忽略了 sim-to-real gap 的系统性处理 |
| 论文里的方法复现不出来 | 不清楚论文省略了哪些工程细节 |

### Mini-Legged 的定位与范围

**Mini-Legged 不是 legged_control 的简化版抄写**——它是你自己的控制栈,用来理解每个设计决策背后的"为什么"。

| 维度 | Mini-Legged(本章) | legged_control | ocs2_legged_robot |
|------|-------------------|---------------|-------------------|
| 定位 | 教学+研究原型 | 生产级框架 | MPC 参考实现 |
| 代码量 | ~5000-8000 行 C++ | ~15000 行 | ~10000 行 |
| MPC | SRB 凸 QP(简单直接) | OCS2 SQP(工业级) | OCS2 SQP |
| WBC | 单层加权 QP | 分层 HQP | 无(纯 MPC) |
| RL | IsaacLab 训练 + 部署 | 无 | 无 |
| Sim-to-Real | 完整流程 | 部分支持 | 仿真为主 |

**本章覆盖的完整开发流程**:

```
开发环境搭建          URDF/MJCF 模型           站立控制器
(69.2)          →    (69.3)            →     (69.4)
                                                │
                                                ▼
MPC 集成              WBC 集成                步态实现
(69.5)          ←    (69.6)            ←     (69.7)
    │
    ▼
RL 策略训练           Sim-to-Real 部署        性能调优
(69.8)          →    (69.9)            →     (69.10)
```

### 目标硬件与软件平台

| 组件 | 推荐选择 | 备选 |
|------|---------|------|
| 机器人 | Unitree Go2 EDU | Unitree A1、Anymal-B (仿真) |
| 操作系统 | Ubuntu 24.04 LTS | Ubuntu 22.04 LTS（维护旧工程） |
| ROS 版本 | ROS 2 Jazzy | ROS 2 Humble（旧包兼容路线） |
| 仿真器(快速验证) | MuJoCo 3.x | — |
| 仿真器(ROS 集成) | Gazebo Harmonic (Jazzy) | Gazebo Fortress (Humble) / Gazebo Classic |
| RL 训练 | IsaacLab + rsl_rl | legged_gym (旧版) |
| GPU | NVIDIA RTX 3070+ | RTX 3090/4090（更快训练） |
| 真机 SDK | Unitree SDK2 (CycloneDDS) | unitree_ros2 |

### 学习路线建议

| 阶段 | 时间 | 内容 | 产出 |
|------|------|------|------|
| 第 1 周 | 20h | 69.1-69.4:环境搭建 + 模型 + 站立 | Go2 在 MuJoCo 中稳定站立 |
| 第 2 周 | 20h | 69.5-69.7:MPC + WBC + 步态 | Go2 做 trot 步态行走 |
| 第 3 周 | 20h | 69.8-69.9:RL 训练 + Sim-to-Real | IsaacLab 训练策略 + 真机部署 |
| 第 4 周 | 20h | 69.10 + 综合调试 + 报告 | 完整演示视频 + 技术报告 |

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:一上来就写 MPC**
> - 错误做法:跳过站立控制器,直接写 MPC+WBC 全栈
> - 现象:MPC 输出看起来合理,但机器人一发关节命令就摔
> - 根本原因:没有验证 Pinocchio 模型加载、关节顺序映射、坐标系约定是否正确
> - 正确做法:先用最简单的 PD 站立控制器验证整个数据通路,确认 URDF→Pinocchio→关节命令 全链路正确后再加复杂控制器

> **🧠 思维陷阱:追求完美架构**
> - 新手想法:"我要先设计一个完美的类层次结构,支持所有未来扩展"
> - 实际上:过度设计是综合实战项目的头号杀手。Mini-Legged 的目标是**先跑通,再优化**
> - 正确思维:用最简单的设计让机器人站起来(Day 1 目标),然后逐步重构

### 练习

**练习 69.1.1** [必做]:画出你理解的 Mini-Legged 系统架构图。标注每个模块的输入输出、运行频率、所用的核心库。与本节给出的架构图对比,找出差异并解释原因。

**练习 69.1.2** [思考]:如果你只有 2 周时间(而不是 4 周),你会砍掉哪些内容?为什么?这个选择反映了你对"最小可行产品"的理解。

---

## 69.2 开发环境搭建 ⭐

### 动机:环境搭建为什么值得单独一节?

环境搭建是所有新手**最大的时间黑洞**。一个四足控制项目依赖的软件栈极其庞大——ROS/ROS 2、Pinocchio、hpp-fcl/coal、OCS2、MuJoCo、Gazebo、IsaacLab、Unitree SDK2......它们之间的版本兼容性像一张蜘蛛网。**花 2 天把环境搭对,胜过花 2 周在错误环境里调 bug**。

### 如果不仔细搭建环境会怎样?

真实案例:

- Pinocchio 3.x 的 `example_robot_data` API 与 2.x 完全不同,2.x 的 `EXAMPLE_ROBOT_DATA_MODEL_DIR` 在 3.x 中不存在
- OCS2 官方仓库(leggedrobotics/ocs2)主分支仅支持 ROS 1,ROS 2 需要用 `ros2` 分支或社区移植版 `legubiao/ocs2_ros2`
- hpp-fcl 于 2024 年底完成了到 coal 3.0 的品牌重命名,CMake target 名称变了
- MuJoCo 默认使用 Newton 求解器（早在 1.50 版本即为默认,并非 3.0+ 才改变）,仿真行为可能与使用 PGS 的旧教程不一致

### 69.2.1 基础系统配置 ⭐

**Step 1: 先选定系统路线**

Mini-Legged 项目可以走两条路线。不要把两条路线的命令混在同一个 workspace 中：

| 路线 | 推荐系统 | 适合目标 | 主要栈 |
|------|----------|----------|--------|
| 经典复现路线 | Ubuntu 20.04 + ROS Noetic | 复现 OCS2 main / legged_control 原版 | catkin、Gazebo Classic、robotpkg Pinocchio |
| ROS2 实战路线 | Ubuntu 24.04 + ROS 2 Jazzy | 新项目、ros2_control、现代 Gazebo | colcon、gz_ros2_control、OCS2 `ros2` 分支或社区移植 |

下面的命令采用 **ROS2 实战路线**。如果你的目标是复现原版 `legged_control`，应切换到 Noetic/catkin 路线。

```bash
# 确认系统版本
lsb_release -a
# 期望输出: Ubuntu 24.04.x LTS

# 安装基础开发工具
sudo apt update && sudo apt install -y \
    build-essential cmake git wget curl \
    python3-pip python3-venv \
    libeigen3-dev libboost-all-dev \
    liburdfdom-dev liboctomap-dev libassimp-dev \
    clang-format clang-tidy \
    htop tmux
```

**为什么选这些包?** 每个都有明确用途:

| 包 | 用途 | 在本项目中的角色 |
|---|------|----------------|
| `libeigen3-dev` | Eigen 3.4 线性代数库 | Pinocchio/OCS2/WBC 的矩阵运算基础 |
| `libboost-all-dev` | Boost C++ 库(1.74+) | OCS2 的序列化、文件系统、测试框架 |
| `liburdfdom-dev` | URDF 解析库 | Pinocchio 加载 URDF |
| `liboctomap-dev` | 八叉树地图库 | 碰撞检测(hpp-fcl 依赖) |
| `libassimp-dev` | 3D 模型导入库 | URDF 中 mesh 文件的加载 |

**Step 2: ROS 2 Jazzy 安装**

```bash
# 添加 ROS 2 apt 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers ros-jazzy-grid-map \
    ros-jazzy-gz-ros2-control \
    python3-colcon-common-extensions python3-rosdep

# 初始化 rosdep
sudo rosdep init
rosdep update

# 添加到 bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **⚠️ 编程陷阱:忘记 source setup.bash**
> - 错误做法:安装完 ROS 2 后直接开始编译
> - 现象:`colcon build` 报 "找不到 ament_cmake" 或 "找不到 rclcpp"
> - 根本原因:ROS 2 的环境变量未加载,CMake 找不到 ROS 2 的包
> - 正确做法:每次打开新终端前确认 `echo $ROS_DISTRO` 输出 `jazzy`

### 69.2.2 Pinocchio + hpp-fcl(coal)安装 ⭐⭐

Pinocchio 是本项目的**动力学计算引擎**。OCS2 依赖 Pinocchio 进行正/逆动力学、雅可比计算、自碰撞检测。

**方案 A:从 ROS 2 apt 安装(推荐初学者)**

```bash
# 具体版本随 ROS 发行版和 apt 源更新而变化，安装后务必打印确认
sudo apt install -y ros-jazzy-pinocchio
python3 -c "import pinocchio as pin; print(pin.__version__)"
```

这种方式最简单,但版本由 ROS 包仓库决定。教程中涉及 Pinocchio 2.x/3.x API 差异时，以本机打印出的版本为准。

**方案 B:从源码编译(推荐进阶)**

如果你需要 Pinocchio 3.x 的新特性(如改进的自动微分支持),需要从源码编译:

```bash
# 创建工作空间
mkdir -p ~/mini_legged_ws/src && cd ~/mini_legged_ws/src

# Pinocchio 3.x/4.x 使用 coal；Pinocchio 2.x 才对应旧 hpp-fcl 路线
git clone --recursive https://github.com/coal-library/coal.git -b releases/3.0

# 克隆 Pinocchio
git clone --recursive https://github.com/stack-of-tasks/pinocchio.git -b master

# 编译(注意顺序:先 coal,再 Pinocchio)
cd ~/mini_legged_ws
colcon build --packages-select coal --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_INTERFACE=ON
source install/setup.bash

colcon build --packages-select pinocchio --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_COLLISION_SUPPORT=ON \
    -DBUILD_PYTHON_INTERFACE=ON
```

> **⚠️ 编程陷阱:Pinocchio 头文件包含顺序错误**
> - 错误做法:在其他头文件之后 `#include <pinocchio/fwd.hpp>`
> - 现象:编译时出现 Boost Variant 大小不一致的奇怪错误
> - 根本原因:Pinocchio 需要在 Boost 序列化头文件之前定义某些宏,否则 Boost Variant 的模板参数数量不一致
> - 正确做法:**永远把 `#include <pinocchio/fwd.hpp>` 放在所有 include 的第一行**

### 69.2.3 OCS2 安装(ROS 2 版本) ⭐⭐

OCS2 官方主分支仍以 ROS 1/catkin 为主。对于 ROS 2,需要明确使用官方 `ros2` 分支或社区移植版，并接受 API/依赖随分支变化的成本:

```bash
cd ~/mini_legged_ws/src

# 方案 1:legubiao 的 ROS 2 移植（社区路线，先确认目标 ROS 发行版）
git clone --recursive https://github.com/legubiao/ocs2_ros2.git

# 方案 2:官方 ros2 分支（目标环境以仓库 README 为准）
# git clone --recursive https://github.com/leggedrobotics/ocs2.git -b ros2

# 克隆机器人模型资源
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

# 安装依赖
cd ~/mini_legged_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译(这一步耗时较长,20-40 分钟)
colcon build --packages-up-to ocs2_legged_robot_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**常见编译错误与解决方案**:

| 错误信息 | 原因 | 解决方案 |
|---------|------|---------|
| `hpp-fcl not found` | CMake 找不到 hpp-fcl | 确保 `source install/setup.bash` 后再编译 OCS2 |
| `CppAD/CG 版本不匹配` | OCS2 依赖特定版本的 CppAD | 使用 OCS2 仓库内自带的 CppAD 子模块 |
| `Boost serialization 链接错误` | Boost 版本与 Pinocchio 编译时不一致 | 统一使用系统 Boost,不混用 conda Boost |
| `HPIPM not found` | QP 求解器未编译 | OCS2 通常自带 HPIPM 子模块,确保 `--recursive` 克隆 |

### 69.2.4 MuJoCo 安装 ⭐

MuJoCo 是本项目的**快速仿真引擎**——用于控制器的初步验证,比 Gazebo 启动快 10 倍以上。

```bash
# MuJoCo 3.x 安装(pip 方式,最简单)
pip3 install mujoco

# 验证安装
python3 -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')"
# 期望输出: MuJoCo version: 3.x.x

# 安装 MuJoCo 查看器(可选但推荐)
pip3 install mujoco-viewer

# 克隆 mujoco_menagerie(包含 Go2 模型)
cd ~/mini_legged_ws/src
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

**为什么用 MuJoCo 而不是只用 Gazebo?**

| 特性 | MuJoCo | Gazebo Harmonic |
|------|--------|-----------------|
| 启动时间 | <1 秒 | 5-15 秒 |
| 仿真速度 | 10-50x 实时 | 1-3x 实时 |
| 接触模型 | 凸优化(更物理) | 弹簧-阻尼(简化) |
| ROS 集成 | 需要手动桥接 | 原生 gz_ros2_control |
| GPU 加速 | MJX(JAX 后端) | 不支持 |
| 最佳用途 | 快速迭代、RL 训练 | 完整系统集成测试 |

**开发流程建议**:先在 MuJoCo 中验证控制器算法(秒级启动,快速迭代),再到 Gazebo 中做 ROS 集成测试,最后部署到真机。

### 69.2.5 IsaacLab 安装(RL 训练) ⭐⭐

IsaacLab 是 NVIDIA 的机器人学习框架,提供 GPU 并行仿真和标准化 RL 任务。2026 年前后的 IsaacLab 版本变化很快：3.0 开始支持 kit-less / Newton 后端训练；如果你需要 PhysX、ROS、URDF/MJCF importer 等完整仿真能力，仍应按官方文档安装 Isaac Sim pip 包并用 IsaacLab 的脚本入口启动。

```bash
# 前置:按官方文档选择安装路线
# - 完整 PhysX/Isaac Sim 路线:适合可视化、导入 URDF/MJCF、接近常规 Isaac Sim 工作流
# - kit-less/Newton 路线:适合官方支持任务的快速训练
# 参考: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/

# 克隆 IsaacLab
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# 创建并激活环境。也可使用 IsaacLab 自带的 --conda/--uv 辅助命令。
./isaaclab.sh --conda env_isaaclab
conda activate env_isaaclab

# 安装 IsaacLab 扩展和学习框架
# --install 会安装 IsaacLab 扩展；参数 rsl_rl 表示只安装本章需要的学习框架
./isaaclab.sh --install rsl_rl  # 等价短选项: ./isaaclab.sh -i rsl_rl

# 验证安装
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

> **⚠️ 编程陷阱:Isaac Sim 版本与 IsaacLab 版本不匹配**
> - 错误做法:用旧版 Isaac Sim 配新版 IsaacLab
> - 现象:import 时报各种 module not found 或 API 变更错误
> - 根本原因:IsaacLab 每个大版本绑定特定 Isaac Sim 版本
> - 正确做法:严格按照 IsaacLab 官方文档的版本对照表安装

### 69.2.6 Docker 镜像与 MuJoCo 源码构建 ⭐

如果不想从零搭建 legged_control 的全部依赖,可以直接使用预构建的 Docker 镜像。该镜像基于云原生方法在国内服务器上构建,拉取速度有保障:

```bash
# legged_control Docker 镜像（预装所有依赖）
docker pull docker.cnb.cool/gpf2025/legged_control:v3
```

拉取完成后使用以下命令运行,每个参数都有明确用途:

```bash
docker run -it --rm \
    -e "DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v "$(pwd)/outdoor.bag:/tmp/rosbag.bag" \
    --gpus all \
    -e "QT_X11_NO_MITSHM=1"  \
    --network=host  \
    docker.cnb.cool/gpf2025/legged_control:v3
```

**参数详解**:

| 参数 | 用途 |
|------|------|
| `-e "DISPLAY=$DISPLAY"` | 将主机 X11 显示变量传入容器,支持 GUI 可视化 |
| `-v "/tmp/.X11-unix:/tmp/.X11-unix:rw"` | 挂载 X11 Unix socket,实现容器内的图形界面转发 |
| `-v "$HOME/.Xauthority:/root/.Xauthority:rw"` | 挂载 X11 认证文件,允许容器访问主机显示服务 |
| `-v "$(pwd)/outdoor.bag:/tmp/rosbag.bag"` | 将本地 rosbag 文件挂载到容器中使用 |
| `--gpus all` | 使用主机全部 GPU(Gazebo 仿真需要 GPU 加速) |
| `-e "QT_X11_NO_MITSHM=1"` | 禁用 MIT-SHM 扩展,避免 Docker 内 Qt 可视化崩溃 |
| `--network=host` | 使用主机网络,方便 ROS 话题通信 |

> **⚠️ 工程陷阱:Gazebo 仿真需要 GPU 支持**
>
> 因为 Gazebo 仿真器需要 GPU 支持才可以流畅运行,建议主机配有 NVIDIA 显卡并安装好驱动。如果没有 GPU 或驱动未正确配置,运行 Docker 内的 Gazebo 时会非常卡顿,甚至无法渲染。

**MuJoCo 源码构建常见依赖问题**

如果需要从源码编译 MuJoCo(例如需要 C++ API 或特定版本),可能遇到以下依赖缺失:

```bash
# 源码编译 MuJoCo
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

如果直接拉取最新版本的 MuJoCo,需要使用 C++17 及以上的编译器。如果 Ubuntu 版本较老,建议切换到较早的 MuJoCo 版本。

| 错误信息 | 解决方法 |
|---------|---------|
| `Failed to find wayland-scanner` | `sudo apt install libwayland-dev` |
| `No package 'xkbcommon' found` | `sudo apt install libxkbcommon-x11-dev` |

第一个错误来自 GLFW 的 CMakeLists 在构建过程中搜索 Wayland 依赖;第二个错误是因为 GLFW 需要 xkbcommon 库来处理键盘输入映射。两者都是 MuJoCo 渲染窗口的依赖,安装后重新运行 cmake 即可。

### 69.2.7 环境验证清单 ⭐

搭建完所有组件后,逐项验证:

```bash
# 1. ROS 2
ros2 topic list  # 应能看到 /parameter_events 等默认话题

# 2. Pinocchio
python3 -c "
import pinocchio as pin
print(f'Pinocchio version: {pin.__version__}')
model = pin.buildSampleModelManipulator()
print(f'Sample model: {model.nq} DoF')
"

# 3. OCS2
ros2 pkg list | grep ocs2  # 应能看到 ocs2_core, ocs2_pinocchio 等

# 4. MuJoCo + Go2 模型
python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path(
    '$HOME/mini_legged_ws/src/mujoco_menagerie/unitree_go2/scene.xml')
print(f'Go2 model: nq={m.nq}, nv={m.nv}, nu={m.nu}')
"
# 期望输出: Go2 model: nq=19, nv=18, nu=12

# 5. 编译器版本
g++ --version  # 需要 >= 9.0,支持 C++17
```

### ⚠️ 常见陷阱

> **💡 概念误区:认为 conda 和 ROS 2 能完美共存**
> - 新手想法:"我用 conda 管理 Python 环境,同时用 ROS 2"
> - 实际上:conda 会修改 `PATH` 和 `LD_LIBRARY_PATH`,导致 ROS 2 找到错误版本的 Python 或 Boost。这是**最常见的环境问题**
> - 正确做法:ROS 2 开发用系统 Python + venv;IsaacLab 用独立的 conda 环境。切换时注意 `conda deactivate`

> **⚠️ 编程陷阱:在 colcon build 时不指定 --cmake-args -DCMAKE_BUILD_TYPE=Release**
> - 错误做法:`colcon build` 不加任何 cmake 参数
> - 现象:Debug 模式下 Pinocchio/OCS2 计算速度慢 5-10 倍,MPC 无法实时
> - 根本原因:Debug 模式禁用编译器优化(-O0),Eigen 的表达式模板无法展开
> - 正确做法:始终用 `--cmake-args -DCMAKE_BUILD_TYPE=Release`,调试时对单个包切换到 `RelWithDebInfo`

### 练习

**练习 69.2.1** [必做]:按本节步骤搭建完整开发环境。截图保存每一步的验证输出。记录遇到的所有问题和解决方案(这就是你的"环境搭建手册")。

**练习 69.2.2** [进阶]:编写一个 `verify_env.sh` 脚本,自动检测所有依赖是否正确安装,版本是否兼容。输出一份表格显示每个组件的状态(PASS/FAIL)。

---

## 69.3 URDF 与 MuJoCo 模型 ⭐⭐

### 动机:为什么模型正确性如此重要?

控制器的数学公式再漂亮,如果输入的机器人模型有误——关节顺序搞反、惯性张量不对、碰撞体太大——控制器的输出就是垃圾。**GIGO(Garbage In, Garbage Out)原则在机器人控制中尤为致命**,因为错误的扭矩命令会直接损坏电机。

### 如果模型不正确会怎样?

| 模型错误 | 后果 | 调试难度 |
|---------|------|---------|
| 关节轴方向搞反(+z 写成 -z) | 机器人"反关节"运动,看起来像癫痫 | 中等(观察关节方向即可) |
| 惯性张量数量级错误 | 动力学补偿完全失效,机器人软塌或弹飞 | 高(数值错误不直观) |
| 碰撞体尺寸过大 | 仿真中腿互相碰撞,站立时被弹开 | 中等(可视化碰撞体) |
| 足端坐标系定义与控制器不一致 | WBC 的足端追踪方向相反 | 极高(看起来"差不多对") |

### 69.3.1 Go2 URDF 获取与理解 ⭐

Unitree Go2 的 URDF 可从多个来源获取:

```bash
# 来源 1:mujoco_menagerie(主要提供调校好的 MJCF/XML)
ls ~/mini_legged_ws/src/mujoco_menagerie/unitree_go2/

# 来源 2:Unitree 官方 ROS 2 包或官方模型仓库(URDF)
# https://github.com/unitreerobotics/unitree_ros2

# 来源 3:example-robot-data(Pinocchio 附带)
python3 -c "
import example_robot_data
robot = example_robot_data.load('go2')
print(f'Model path: {robot.model_path}')
print(f'nq={robot.model.nq}, nv={robot.model.nv}')
"
```

`example_robot_data.load('go2')` 是否可用取决于你安装的 `example-robot-data` 版本；如果本机版本没有 Go2 模型，不要为了这一行去修改控制器逻辑。URDF 优先从 Unitree 官方 ROS 2 包或官方模型仓库获取，MJCF/XML 优先使用 `mujoco_menagerie/unitree_go2`。教学项目的重点是**同一套控制代码始终通过关节名建立映射**，模型来源只是文件路径不同。

**Go2 的关节结构**(12 个自由度):

```
Go2 运动链(每条腿 3 个关节):

                    base_link (浮动基座, 6 DoF)
                   /    |    |    \
                 FL    FR    RL    RR        (4 条腿)
                 │     │     │     │
               hip_joint  (绕 x 轴,内外展)
                 │
              thigh_joint (绕 y 轴,前后摆)
                 │
               calf_joint (绕 y 轴,膝关节)
                 │
               foot (足端,无驱动)

关节命名约定(Unitree):
  FL = Front Left   FR = Front Right
  RL = Rear Left    RR = Rear Right

本章内部控制向量顺序:
  q = [FL_hip, FL_thigh, FL_calf,
       FR_hip, FR_thigh, FR_calf,
       RL_hip, RL_thigh, RL_calf,
       RR_hip, RR_thigh, RR_calf]  (12 维)

浮动基座(Pinocchio 的约定):
  q_float = [px, py, pz, qx, qy, qz, qw]  (7 维,四元数)
  v_float = [vx, vy, vz, wx, wy, wz]       (6 维,速度)
```

上面的 `[FL, FR, RL, RR]` 是本章为了讲清楚控制算法而采用的**内部教学顺序**，不是所有软件栈的共同约定。当前 Unitree SDK2 的 Go2 `JointIndex` 使用 `[FR, FL, RR, RL]` 顺序：

| 本章内部索引 | Unitree SDK2 索引 | 关节名 |
|-------------|------------------|--------|
| 0,1,2 | 3,4,5 | FL_Hip, FL_Thigh, FL_Calf |
| 3,4,5 | 0,1,2 | FR_Hip, FR_Thigh, FR_Calf |
| 6,7,8 | 9,10,11 | RL_Hip, RL_Thigh, RL_Calf |
| 9,10,11 | 6,7,8 | RR_Hip, RR_Thigh, RR_Calf |

因此实际工程里不要把 `q[0]` 直接当作某个硬件电机。更稳妥的做法是：Pinocchio/MuJoCo/IsaacLab/SDK2 各自打印关节名，然后构造 `name -> index` 映射；控制器内部统一使用一套顺序，进出仿真器或真机 SDK 时做显式转换。

> **⚠️ 编程陷阱:Pinocchio 的四元数约定与 Eigen 构造函数不同**
> - Pinocchio 的配置向量中四元数顺序为 `[qx, qy, qz, qw]`(xyz-w)
> - ROS 的 `geometry_msgs/Quaternion` 字段顺序也是 `[x, y, z, w]`(一致)
> - 但 Eigen 的 `Quaterniond` 构造函数参数是 `Quaterniond(qw, qx, qy, qz)`(w-xyz)
> - **这三者的混用是四足控制中最常见的 bug 来源之一**

### 69.3.2 从 URDF 到 MJCF 的转换 ⭐⭐

MuJoCo 使用自己的 MJCF 格式,与 URDF 有重要区别:

| 特性 | URDF | MJCF |
|------|------|------|
| 格式 | XML | XML |
| 坐标约定 | 每个 link 有自己的原点 | 相对于父 body |
| 执行器定义 | 无(需要 ros2_control 配置) | 内置 actuator 标签 |
| 接触参数 | 无 | 内置 friction、solref 等 |
| 碰撞检测 | 独立 collision 标签 | geom 标签统一处理 |

**转换方法 1:使用 mujoco_menagerie 的现成模型(推荐)**

Google DeepMind 维护的 mujoco_menagerie 已经包含了经过精心调校的 Go2 MJCF 模型:

```python
import mujoco
import mujoco.viewer

# 加载 Go2 模型
model = mujoco.MjModel.from_xml_path(
    'mujoco_menagerie/unitree_go2/scene.xml'
)
data = mujoco.MjData(model)

# 查看模型信息
print(f"自由度: nq={model.nq}, nv={model.nv}")
print(f"执行器数: nu={model.nu}")
print(f"body 数: nbody={model.nbody}")

# 打印所有关节名
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(f"  Joint {i}: {name}, type={model.jnt_type[i]}")

# 启动查看器
mujoco.viewer.launch(model, data)
```

**转换方法 2:手动 URDF→MJCF 转换(理解原理)**

```bash
# MuJoCo 自带的编译工具可以转换 URDF
# 注意:转换结果通常需要手动调整
python3 -c "
import mujoco
model = mujoco.MjModel.from_xml_path('go2.urdf')
mujoco.mj_saveLastXML('go2_converted.xml', model)
"
```

转换后需要手动调整的关键参数:

```xml
<!-- 执行器定义:URDF 没有,需要手动添加 -->
<actuator>
  <!-- 按关节填写 ctrlrange；不要把官网峰值扭矩当作所有关节的统一连续限幅 -->
  <motor name="FL_hip" joint="FL_hip_joint"
         ctrlrange="-TAU_HIP TAU_HIP" gear="1"/>
  <motor name="FL_thigh" joint="FL_thigh_joint"
         ctrlrange="-TAU_THIGH TAU_THIGH" gear="1"/>
  <!-- ... 其余 10 个关节同理 -->
</actuator>

<!-- 接触参数调整 -->
<default>
  <geom friction="0.8 0.02 0.01"
        solref="0.005 1"
        solimp="0.9 0.95 0.001"/>
</default>
```

Unitree 官方规格页会给出整机或关节电机的峰值能力（Go2 页面常见标注约 45 N.m），但控制器限幅应来自具体模型文件、SDK 参数表和实验安全策略。仿真中建议用 `tau_limit[12]` 数组逐关节设置，并区分峰值扭矩、连续扭矩和保守上机限幅。

### 69.3.3 碰撞几何与仿真稳定性 ⭐⭐

碰撞几何的选择直接影响仿真质量:

| 碰撞类型 | 计算速度 | 精度 | 适用场景 |
|---------|---------|------|---------|
| 球体(sphere) | 最快 | 低 | RL 大规模训练(MJX) |
| 胶囊体(capsule) | 快 | 中 | 快速仿真验证 |
| 凸包(convex mesh) | 中等 | 高 | 精确仿真 |
| 三角网格(mesh) | 最慢 | 最高 | 渲染(不推荐用于碰撞) |

**mujoco_menagerie 的 Go2 模型提供了两个版本**:
- `scene.xml`:使用精确碰撞几何,适合单机仿真
- `scene_mjx.xml`:使用简化球体碰撞,适合 MJX GPU 加速训练

```python
# 验证碰撞几何
import mujoco
model = mujoco.MjModel.from_xml_path('unitree_go2/scene.xml')

for i in range(model.ngeom):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    geom_type = model.geom_type[i]
    type_names = {0:'plane', 2:'sphere', 3:'capsule', 4:'ellipsoid',
                  5:'cylinder', 6:'box', 7:'mesh'}
    print(f"  Geom {name}: type={type_names.get(geom_type, geom_type)}, "
          f"size={model.geom_size[i]}")
```

**碰撞几何的工程经验教训**

碰撞几何的选择不仅影响仿真质量,更直接影响 MPC 求解器的实时性。以下是在自定义四足机器人上的真实失败案例。

> **⚠️ 工程陷阱:碰撞体过于复杂导致 OCS2/MPC 求解失败**
>
> SolidWorks 导出的 URDF 中,`<collision>` 和 `<visual>` 通常都指向同一个 STL 文件——这是一个常见错误。高精度 STL 网格作为碰撞几何体会导致:
> - 碰撞检测计算量暴增
> - OCS2 的 SQP/DDP 求解器无法在规定时间内完成迭代
> - 最终表现为**机器人乱动**(求解器超时返回随机解)
>
> **正确做法**:视觉网格(`<visual>`)可以使用高精度 STL,碰撞几何(`<collision>`)必须简化为基本体素(`<box>`、`<cylinder>`、`<sphere>`、`<capsule>`)。

以经典的 Go2 机器人为例,其 URDF 中 `<visual>` 使用精细 mesh 文件渲染外观,而 `<collision>` 使用简单几何体近似:

```xml
<link name="base">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
            <mesh filename="${mesh_path}/trunk.dae" scale="1 1 1"/>
        </geometry>
    </visual>
    <collision>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
            <!-- 碰撞体使用简单长方体近似,而非原始 mesh -->
            <box size="${trunk_length} ${trunk_width} ${trunk_height}"/>
        </geometry>
    </collision>
</link>
```

这种分离设计的核心逻辑是:视觉渲染只在可视化时计算一次,而碰撞检测在仿真的每个时间步都要执行。如果碰撞体是数万面的 STL 网格,每一步的碰撞检测计算量都会暴增,直接拖慢整个仿真循环。

> **⚠️ 工程陷阱:非标准垂直构型的碰撞检测**
>
> 在标准垂直构型中,膝关节位于髋关节正下方(沿 Z 轴),髋关节位于侧摆关节左右方(沿 Y 轴)。如果自定义机器人的关节坐标系定义不遵循标准垂直构型——例如直接按照真实机器人的关节物理位置定义坐标变换——碰撞检测可能在"本应无碰撞"的位置误报碰撞。这是因为非标准构型下,简单几何体的碰撞包围盒会产生不必要的交叠。
>
> **建议**:在机械设计之初就定义标准垂直构型下的坐标系。如果已经是非标准构型,需要进行一系列坐标变换和惯量矩阵变换才能构建正确的碰撞模型——这是一个很复杂的工程,远不如从源头避免来得高效。

### 69.3.4 Pinocchio 模型加载与验证 ⭐⭐

在控制器中使用模型之前,**必须验证 Pinocchio 加载的模型与 MuJoCo 一致**:

```cpp
// C++ 版本:PinocchioInterface 封装
#include <pinocchio/fwd.hpp>  // 必须是第一个 include!
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

class PinocchioInterface {
public:
    PinocchioInterface(const std::string& urdf_path) {
        // 加载 URDF(浮动基座)
        pinocchio::urdf::buildModel(urdf_path,
            pinocchio::JointModelFreeFlyer(), model_);
        data_ = pinocchio::Data(model_);

        // 验证维度
        assert(model_.nq == 19);  // 7(浮动基座) + 12(关节)
        assert(model_.nv == 18);  // 6(浮动基座) + 12(关节)

        // 打印关节信息用于验证
        for (int i = 0; i < model_.njoints; ++i) {
            std::cout << "Joint " << i << ": "
                      << model_.names[i] << std::endl;
        }
    }

    // 计算所有足端位置
    Eigen::Matrix<double, 3, 4> getFootPositions(
        const Eigen::VectorXd& q) {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        Eigen::Matrix<double, 3, 4> foot_pos;
        for (int i = 0; i < 4; ++i) {
            auto frame_id = model_.getFrameId(foot_names_[i]);
            foot_pos.col(i) = data_.oMf[frame_id].translation();
        }
        return foot_pos;
    }

    Eigen::VectorXd computeGravityCompensation(const Eigen::VectorXd& q) {
        // 等价于 RNEA(q, v=0, a=0)，返回 18 维广义重力项。
        return pinocchio::computeGeneralizedGravity(model_, data_, q);
    }

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::array<std::string, 4> foot_names_ = {
        "FL_foot", "FR_foot", "RL_foot", "RR_foot"
    };
};
```

**交叉验证:Pinocchio vs MuJoCo 的 FK 结果**

```python
# Python 验证脚本:两个引擎的前向运动学应一致
import numpy as np
import pinocchio as pin
import mujoco

# Pinocchio FK
pin_model = pin.buildModelFromUrdf("go2.urdf",
                                    pin.JointModelFreeFlyer())
pin_data = pin_model.createData()

q_pin = pin.neutral(pin_model)  # 零位姿
pin.forwardKinematics(pin_model, pin_data, q_pin)
pin.updateFramePlacements(pin_model, pin_data)

fl_foot_id = pin_model.getFrameId("FL_foot")
fl_pos_pin = pin_data.oMf[fl_foot_id].translation

# MuJoCo FK
mj_model = mujoco.MjModel.from_xml_path("unitree_go2/scene.xml")
mj_data = mujoco.MjData(mj_model)
mujoco.mj_forward(mj_model, mj_data)

# MuJoCo 查询对象取决于具体 MJCF：
# - mujoco_menagerie 的 scene.xml 常用 foot geom 名称（如 "FL"）
# - scene_mjx.xml 或自定义模型可能提供 site 名称（如 "FL_foot"）
fl_geom_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_GEOM, "FL")
if fl_geom_id >= 0:
    fl_pos_mj = mj_data.geom_xpos[fl_geom_id]
else:
    fl_site_id = mujoco.mj_name2id(
        mj_model, mujoco.mjtObj.mjOBJ_SITE, "FL_foot")
    assert fl_site_id >= 0, "找不到 FL 足端 geom/site，请先打印模型名称"
    fl_pos_mj = mj_data.site_xpos[fl_site_id]

# 比较
print(f"Pinocchio FL_foot: {fl_pos_pin}")
print(f"MuJoCo    FL_foot: {fl_pos_mj}")
print(f"Difference: {np.linalg.norm(fl_pos_pin - fl_pos_mj):.6f} m")
# 差异应 < 1mm,否则说明模型不一致
```

### 69.3.5 自定义四足模型集成最小闭环 ⭐⭐

前面的 69.3.1-69.3.4 以 Unitree Go2 为例讲解了模型的获取与验证。但在实际研究中,你很可能需要将**自己设计的四足机器人**集成到 legged_control 框架中。这一过程涉及 URDF 转换、配置文件适配、初始姿态设定等一系列工程细节,每一步都有潜在的坑。

本节以一个真实的自定义四足机器人(命名为 dmgo)为例,给出从 CAD 导出到仿真控制的最小闭环流程。

#### URDF 到 xacro 的分解

从 SolidWorks 导出的 URDF 通常是一个单一的大文件,包含所有 link、joint、mesh 引用。直接使用这个文件虽然可行,但难以维护和扩展。参考 Unitree 官方模型的组织方式,应将其拆分为模块化的 xacro 文件:

| 文件 | 职责 | 说明 |
|------|------|------|
| `robot.xacro` | 顶层入口 | 包含所有子 xacro 的 include,定义 base link |
| `leg.xacro` | 腿部运动链 | 定义 hip/thigh/calf/foot 的 link 和 joint |
| `transmission.xacro` | 关节力矩接口 | 为每个关节定义 `EffortJointInterface` |
| `gazebo.xacro` | Gazebo 插件 | ros_control 插件 + ground truth 插件 |
| `imu.xacro` | IMU 传感器 | IMU 的 link、joint 和 Gazebo 仿真参数 |

拆分后的文件通过 xacro 宏机制组合。例如在 `leg.xacro` 开头引用 transmission:

```xml
<xacro:include filename="$(find dmgo_description)/urdf/common/transmission.xacro"/>
```

在文件末尾调用宏,为每条腿创建力矩控制接口:

```xml
<xacro:leg_transmission name="${prefix}"/>
```

> **⚠️ 工程陷阱:xacro 路径格式**
>
> 使用 `package://dmgo_description/meshes/` 而非 `$(find dmgo_description)/meshes/`。后者在某些场景下(如 `xacro` 独立调用时)会因为环境变量缺失导致 "Unable to open file" 错误。典型报错如下:
>
> ```
> Could not load resource [.../dmgo_description/meshes/LF_FOOT.STL]:
> Unable to open file ".../dmgo_description/meshes/LF_FOOT.STL".
> ```

#### Transmission:虚拟关节电机

没有 transmission,Gazebo 中的四足机器人就无法运动——`gazebo_ros_control` 通过扫描 URDF 中所有注册了 `EffortJointInterface` 的关节来创建控制句柄。每个关节需要一个标准的 `SimpleTransmission` 块:

```xml
<transmission name="${prefix}_HAA_tran">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${prefix}_HAA">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="${prefix}_HAA_motor">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

这里 `mechanicalReduction` 设为 1 表示关节力矩和执行器力矩之间是 1:1 映射(无减速比缩放)。

#### IMU 传感器配置

IMU 的 xacro 可以直接从 Unitree 的参考实现中复制,需要注意的关键参数是所连接的坐标系、IMU 名称和位姿偏移:

```xml
<xacro:macro name="IMU" params="connected_to imu_name xyz:='0 0 0' rpy:='0 0 0'">
    <joint name="${imu_name}_joint" type="fixed">
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <parent link="${connected_to}"/>
        <child link="${imu_name}"/>
    </joint>
    <link name="${imu_name}">
        <inertial>
            <mass value="0.01"/>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <inertia ixx="0.000001" ixy="0" ixz="0"
                     iyy="0.000001" iyz="0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- 防止 Gazebo 合并固定关节 -->
    <gazebo reference="${imu_name}_joint">
        <disableFixedJointLumping>true</disableFixedJointLumping>
    </gazebo>
</xacro:macro>
```

> **⚠️ 工程陷阱:IMU link 需要质量**
>
> URDF 中 IMU 的 link 必须设置非零的 `<mass>` 和 `<inertia>`。缺少质量属性会导致 Gazebo 仿真异常或 link 被自动合并。即使 IMU 本身质量很小(如 0.01 kg),也必须显式声明。

#### Gazebo 物理参数与插件

腿部连杆需要在 `leg.xacro` 末尾添加 Gazebo 物理属性:

| 属性 | 说明 | 不设置的后果 |
|------|------|------------|
| `mu1` / `mu2` | 各向异性库仑摩擦系数 | 脚端"打滑",行为不稳定 |
| `self_collide` | 是否允许 link 间自碰撞 | 腿间可能穿透或过度碰撞 |
| `disableFixedJointLumping` | 禁止 Gazebo 合并固定关节 | 脚端 link 消失,接触检测异常 |
| `kp` / `kd` | 接触刚度/阻尼 | 脚端下陷或抖动 |

建议对脚端/小腿启用自碰撞(`self_collide=1`),对大腿可视情况关闭。对 `*_foot_fixed` 关节必须加 `disableFixedJointLumping`,确保脚端作为独立接触体存在。

Gazebo 插件方面,需要两个关键插件:

```xml
<!-- ros_control 插件:连接 LeggedHWSim 硬件抽象层 -->
<gazebo>
    <plugin name="gazebo_ros_control" filename="liblegged_hw_sim.so">
        <robotNamespace>/</robotNamespace>
        <robotParam>legged_robot_description</robotParam>
        <robotSimType>legged_gazebo/LeggedHWSim</robotSimType>
    </plugin>
</gazebo>

<!-- Ground truth 插件:提供真实状态(cheater 模式使用) -->
<gazebo>
    <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>1000.0</updateRate>
        <bodyName>base</bodyName>
        <topicName>ground_truth/state</topicName>
        <gaussianNoise>0</gaussianNoise>
        <frameName>world</frameName>
    </plugin>
</gazebo>
```

> ⚠️ **工程陷阱：cheater state 仅限仿真调试**
>
> 在仿真环境中，`p3d_base_controller`（或 Gazebo ground truth 插件）可以直接提供机器人的精确位姿和速度——这被称为 **cheater state**（作弊状态）。在开发和调试控制算法时使用 cheater state 可以排除状态估计误差的干扰，快速验证控制逻辑。
>
> 但在**真机部署时必须切换为状态估计器输出**（如 Ch57 中的 ESKF/InEKF）。常见的错误是：仿真中用 cheater state 调通了控制器，直接部署到真机后发现性能急剧下降甚至完全失效——因为状态估计器的噪声、延迟和漂移在仿真中被完全绕过了。
>
> **检查清单**：部署前确认 launch 文件中 `base_state_estimator` 参数已从 `p3d` 切换为 `kalman`/`invariant_ekf`，且 observation 中不包含任何 ground truth 信号。

#### URDF 生成与部署

legged_control 框架要求机器人 URDF 文件存在于 `/tmp/legged_control/<robot_type>.urdf`。源代码提供了 `generate_urdf.sh` 脚本来自动完成这一步。在 launch 文件中集成如下:

```xml
<launch>
    <arg name="robot_type" default="$(env ROBOT_TYPE)"/>

    <!-- 1. 解析 xacro 并挂到 ROS 参数服务器 -->
    <param name="legged_robot_description"
           command="$(find xacro)/xacro $(find dmgo_description)/urdf/robot.xacro
                    robot_type:=$(arg robot_type)"/>

    <!-- 2. 生成 URDF 到 /tmp/legged_control/ -->
    <node name="generate_urdf" pkg="legged_common" type="generate_urdf.sh"
          output="screen"
          args="$(find dmgo_description)/urdf/robot.xacro $(arg robot_type)"/>

    <!-- 3. 加载 IMU/接触传感器配置 -->
    <rosparam file="$(find legged_gazebo)/config/default.yaml" command="load"/>

    <!-- 4. 启动 Gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name"
             value="$(find legged_gazebo)/worlds/empty_world.world"/>
    </include>

    <!-- 5. 在 Gazebo 中生成机器人(带初始关节角度) -->
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
          clear_params="true"
          args="-z 0.5 -param legged_robot_description -urdf
                -model $(arg robot_type)
                -J LF_HAA  0 -J LF_HFE -0.35 -J LF_KFE -2.0
                -J LH_HAA  0 -J LH_HFE -0.35 -J LH_KFE -2.0
                -J RF_HAA  0 -J RF_HFE -0.35 -J RF_KFE -2.0
                -J RH_HAA  0 -J RH_HFE -0.35 -J RH_KFE -2.0"
          output="screen"/>
</launch>
```

其中 `-J` 参数用于设置初始关节角度,使机器人以趴卧姿态生成(而非默认的零位构型)。具体角度值需要根据自己的机器人模型调整。

> **⚠️ 工程陷阱:default.yaml 关键配置**
>
> `legged_gazebo/config/default.yaml` 中的 IMU 名称和接触脚名称**必须与 URDF 中的 link/joint 名称完全匹配**:
>
> ```yaml
> gazebo:
>   delay: 0.009
>   imus:
>     base_imu:
>       frame_id: base_imu    # 必须与 imu.xacro 中的 imu_name 一致
>   contacts: [ "LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT" ]  # 必须与 URDF 中足端 link 名一致
> ```
>
> 同时,`disableFixedJointLumping` 必须正确设置,否则 URDF 解析时固定关节的连杆会被合并,导致碰撞模型和惯性参数错误。

关节名称方面,legged_control 框架内部对关节命名有硬编码假设(如 `LF_HAA`、`LF_HFE`、`LF_KFE` 等)。如果自定义关节名称与代码中的硬编码不一致,控制器将无法正确映射关节,导致力矩输出失败。建议在 SolidWorks 建模阶段就使用标准命名。

### 69.3.6 姿态与零点概念辨析 ⭐⭐

在部署自定义四足机器人时,以下三个概念经常被混淆,导致"机器人乱动/抖动/站不起来"。理清它们是成功部署的前提。

| 概念 | 定义 | 来源 | 修改方式 |
|------|------|------|---------|
| **运动学零点** | 模型中 q=0 对应的关节构型 | URDF `<joint>` 的 `<origin>` 和 `<axis>` | 修改 URDF |
| **机械/传感器零点** | 装配或编码器定义的物理零位 | 电机编码器上电位置 | 硬件标定 |
| **控制零点** | 控制器内部 q=0 的基准 | raw encoder -> model q 的 offset 映射 | 配置 theta_offset |

三者之间的转换公式:

$$\theta_{\text{model}} = \theta_{\text{raw}} + \theta_{\text{offset}}$$

运动学零点不是"绝对零点",而是你人为选择的"参考姿态"——通常选一个对称、接近中位、数值稳定的姿态(比如腿略微弯曲的站立姿态)。OCS2/Pinocchio 的运动学计算默认就基于这个零点。

**初始姿态 vs 参考姿态**

在 legged_control 框架中,还有两个容易混淆的配置:

| 配置 | 所在文件 | 含义 | 典型取值 |
|------|---------|------|---------|
| `initialState` | `task.info` | MPC 优化器启动时的状态猜测 | 趴卧或站立,取决于启动场景 |
| `defaultJointState` + `comHeight` | `reference.info` | 控制器的"期望站立构型" | 标准站立姿态 |

**这两者不是同一件事**。`initialState` 是 MPC 求解器的初始化猜测,如果它与实际机器人姿态差距过大,MPC 求解器可能发散。`defaultJointState` + `comHeight` 则是控制器的参考目标,是机器人"想要"达到的姿态。

如果你希望"初始是趴着,目标是站立",就把 `initialState` 设成趴卧姿态、`defaultJointState` 设成站立姿态。如果启动时就是站立,两个都设成站立即可。

> **⚠️ 工程陷阱:零点和初始姿态配置错误**
>
> 对零点和初始姿态的配置要求很高。一旦配置出现问题就会导致求解出错,从而导致仿真中机器人出现乱动/抖动的情况,无法正常运动。从 SolidWorks 导出的 URDF 中关节原点已经隐含了运动学零点定义,如果在 xacro 中试图修改这个零点,很容易引入不一致。建议直接使用导出的原始 URDF 中的关节状态作为零点,可以保证不会因为零点转换引入额外错误。

### ⚠️ 常见陷阱

> **💡 概念误区:认为 URDF 和 MJCF 的惯性参数含义完全相同**
> - URDF 的 `<inertial>` 是相对于 link 原点的
> - MJCF 的 `<inertial>` 默认是相对于 body 质心的
> - 如果直接复制数值而不做坐标变换,动力学计算会有偏差

> **⚠️ 编程陷阱:MuJoCo 和 Pinocchio 的关节顺序不一致**
> - MuJoCo 的关节顺序取决于 XML 中 `<joint>` 标签的出现顺序
> - Pinocchio 的关节顺序取决于 URDF 中 `<joint>` 的拓扑排序
> - 两者可能不同!必须建立显式的索引映射表

### 练习

**练习 69.3.1** [必做]:加载 Go2 的 URDF 到 Pinocchio,打印所有关节名称和限位。画一张表格,列出每个关节的名称、类型、位置范围、速度范围、力矩限制。

**练习 69.3.2** [必做]:用 Python 编写交叉验证脚本,在 10 个随机关节配置下比较 Pinocchio 和 MuJoCo 的 FK 结果。所有足端位置差异应小于 1mm。

**练习 69.3.3** [进阶]:修改 MJCF 中的 `solref` 和 `solimp` 参数,观察仿真中足地接触的弹跳行为变化。写一段文字解释这些参数的物理含义。

---

## 69.4 站立控制器 ⭐⭐

### 动机:为什么第一步是站立?

站立控制器是整个控制栈的**烟雾测试(smoke test)**——如果机器人连站都站不住,任何上层控制器都没有意义。站立控制器的作用是:

1. **验证数据通路**:URDF → Pinocchio → 关节命令 → 仿真器 → 观测反馈,整条链路是否正确
2. **验证坐标系约定**:关节正方向是否与预期一致
3. **提供调试基线**:后续加 MPC/WBC 时,如果出问题可以退回到站立控制器排查

### 如果跳过站立控制器会怎样?

直接写 MPC+WBC 然后调试,你将面对一个**三重未知**:
- 模型是否正确?
- MPC 公式是否正确?
- WBC 公式是否正确?

三个未知叠加,调试空间是指数级的。先用站立控制器消除第一个未知,再逐步加入 MPC 和 WBC。

> **本质洞察**：站立控制器的价值**不在于它本身的控制性能,而在于它提供了一个"已知正确的基线"**。当你后续加入 MPC 或 WBC 后机器人出了问题,你可以退回到站立控制器确认底层数据通路是否正确——如果站立正常但 MPC 不正常,问题一定在 MPC 层;如果连站立都不正常,问题在模型或硬件接口。这就是软件工程中"二分调试法"在机器人系统中的应用。

### 69.4.1 重力补偿 + PD 控制 ⭐

> **跨领域类比**：重力补偿 + PD 控制之于机器人站立,就像建筑的地基 + 抗震阻尼器之于建筑结构。重力补偿($\tau_g$)相当于地基——它承受了"静态载荷"(重力),让结构在无外力时保持平衡;PD 反馈相当于阻尼器——它抵抗"动态扰动"(推一下或踩到不平),把偏差拉回平衡点。没有地基(没有重力补偿)的建筑会塌,因为阻尼器无法承受持续的重力载荷;没有阻尼器(没有 PD)的建筑会在地震中倒,因为地基无法吸收动态能量。两者缺一不可。

最简单的站立控制器:在站立姿态下做重力补偿 + PD 反馈。

**物理直觉**:机器人在站立姿态下,关节扭矩需要抵消重力引起的力矩。如果我们计算出精确的重力补偿扭矩 $\tau_g$,再加上 PD 反馈修正偏差,机器人就能稳定站立。

**数学公式**:

$$\tau = \tau_g(q) + K_p (q_{\text{des}} - q) + K_d (\dot{q}_{\text{des}} - \dot{q})$$

其中:
- $\tau_g(q)$ = 重力补偿扭矩,由 Pinocchio 的逆动力学在零加速度下计算得到
- $K_p$ = 位置增益矩阵(对角阵)
- $K_d$ = 速度增益矩阵(对角阵)
- $q_{\text{des}}$ = 站立目标关节角度
- $\dot{q}_{\text{des}} = 0$(站立时目标速度为零)

**为什么需要重力补偿?** 如果只用 PD 而不加 $\tau_g$,PD 控制器必须用位置误差来"间接"产生抵消重力的扭矩——这意味着机器人永远无法达到零误差的站立姿态,会有一个**静差(steady-state error)**。加入重力补偿后,PD 只需要处理小扰动,静差消失。

```cpp
// 正确实现:重力补偿 + PD
class StandController {
public:
    StandController(const PinocchioInterface& pin_interface)
        : pin_(pin_interface) {
        // Go2 的站立目标关节角度(弧度)
        // [hip, thigh, calf] x 4 条腿
        q_des_.resize(12);
        q_des_ << 0.0, 0.8, -1.6,   // FL
                  0.0, 0.8, -1.6,    // FR
                  0.0, 1.0, -1.6,    // RL
                  0.0, 1.0, -1.6;    // RR

        // PD 增益与逐关节限幅
        kp_ = Eigen::VectorXd::Constant(12, 50.0);   // Nm/rad
        kd_ = Eigen::VectorXd::Constant(12, 3.0);    // Nm*s/rad
        tau_limit_ = Eigen::VectorXd::Constant(12, 25.0);  // 示例保守值，实机需按关节标定
    }

    Eigen::VectorXd compute(const Eigen::VectorXd& q_full,
                             const Eigen::VectorXd& v_full) {
        // 1. 计算重力补偿
        Eigen::VectorXd tau_gravity = pin_.computeGravityCompensation(q_full);
        // 只取关节部分(跳过浮动基座的 6 维)
        Eigen::VectorXd tau_g = tau_gravity.tail(12);

        // 2. 提取当前关节状态
        Eigen::VectorXd q_joint = q_full.tail(12);
        Eigen::VectorXd v_joint = v_full.tail(12);

        // 3. PD 反馈
        Eigen::VectorXd tau_pd =
            kp_.cwiseProduct(q_des_ - q_joint) +
            kd_.cwiseProduct(-v_joint);

        // 4. 总扭矩 = 重力补偿 + PD
        Eigen::VectorXd tau = tau_g + tau_pd;

        // 5. 扭矩限幅：逐关节、逐平台配置，不使用单一 magic number
        tau = tau.cwiseMax(-tau_limit_).cwiseMin(tau_limit_);

        return tau;
    }

private:
    const PinocchioInterface& pin_;
    Eigen::VectorXd q_des_, kp_, kd_, tau_limit_;
};
```

**对比:两种错误实现**

```cpp
// 错误 1:忘记重力补偿
Eigen::VectorXd tau_wrong = kp_.cwiseProduct(q_des_ - q_joint)
                           + kd_.cwiseProduct(-v_joint);
// 问题:机器人会"蹲下"(因为 PD 无法完全抵消重力)
//       且在不同 q 下表现不一致(重力力矩随姿态变化)

// 错误 2:重力补偿但不限幅
Eigen::VectorXd tau_dangerous = tau_g + tau_pd;  // 没有 clamp!
// 问题:瞬态扰动可能产生远超电机能力的扭矩命令
//       在真机上会触发过流保护甚至烧电机
```

### 69.4.2 在 MuJoCo 中验证站立 ⭐⭐

```python
import mujoco
import mujoco.viewer
import numpy as np

# 加载模型
model = mujoco.MjModel.from_xml_path(
    'mujoco_menagerie/unitree_go2/scene.xml')
data = mujoco.MjData(model)

# 站立目标角度
q_des = np.array([
    0.0, 0.8, -1.6,   # FL
    0.0, 0.8, -1.6,   # FR
    0.0, 1.0, -1.6,   # RL
    0.0, 1.0, -1.6    # RR
])

# PD 增益
kp = np.full(12, 50.0)
kd = np.full(12, 3.0)

def controller(model, data):
    """MuJoCo 控制回调"""
    # 获取当前关节位置和速度
    q = data.qpos[7:]   # 跳过浮动基座(7维: pos + quat)
    dq = data.qvel[6:]  # 跳过浮动基座(6维: lin_vel + ang_vel)

    # MuJoCo 会仿真重力；这里的纯 PD 不是“自带重力补偿”，
    # 而是依靠站立姿态的稳态误差产生支撑扭矩。
    # 加入 Pinocchio 重力补偿后，静差会明显减小。
    tau = kp * (q_des - q) + kd * (0 - dq)

    # 逐关节限幅：示例保守值，实机请用 SDK/URDF/MJCF 参数表覆盖
    tau_limit = np.full(12, 25.0)
    tau = np.clip(tau, -tau_limit, tau_limit)

    # 设置控制信号
    data.ctrl[:] = tau

# 启动仿真
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        controller(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
```

**调参指导**:

| 参数 | 过小的表现 | 过大的表现 | 推荐范围(Go2) |
|------|-----------|-----------|---------------|
| $K_p$ | 机器人软塌,缓慢下沉 | 高频振动("抖动") | 30-80 Nm/rad |
| $K_d$ | 欠阻尼,超调大 | 过阻尼,响应迟钝 | 1-5 Nm*s/rad |

**调参原则**:先设 $K_d = 0$,逐步增大 $K_p$ 直到机器人能站住;然后逐步增大 $K_d$ 直到振动消除。这就是经典的 Ziegler-Nichols 调参思路。

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:MuJoCo 的 qpos 和 qvel 维度不同**
> - `data.qpos` 是 19 维(7 浮动基座 + 12 关节),其中浮动基座用四元数(4 维)
> - `data.qvel` 是 18 维(6 浮动基座 + 12 关节),其中浮动基座用角速度(3 维)
> - **qpos[7:] 和 qvel[6:] 才是关节部分**,索引起点不同!

> **💡 概念误区:认为 PD 增益越大越好**
> - 新手想法:"$K_p$ 开大一点,控制更精确"
> - 实际上:$K_p$ 过大会导致系统带宽超过执行器和仿真器的能力,产生高频振荡
> - 物理原因:实际电机有力矩带宽限制(Go2 的电机带宽约 30-50 Hz),PD 增益隐含的等效带宽 $\omega_n = \sqrt{K_p/J}$ 不能超过电机带宽

### 练习

**练习 69.4.1** [必做]:在 MuJoCo 中实现 Go2 的站立控制器。目标:站立 60 秒不摔,基座高度波动 < 5mm。录屏保存。

**练习 69.4.2** [必做]:给站立的 Go2 施加外力扰动(在 MuJoCo viewer 中 Ctrl+click 拖拽)。记录不同 $K_p$/$K_d$ 下的恢复时间。画出"$K_p$ vs 恢复时间"的曲线。

**练习 69.4.3** [进阶]:实现带重力补偿的站立控制器(用 Pinocchio 的 `computeGeneralizedGravity`),与纯 PD 控制器对比静差。

---

## 69.5 MPC 集成 ⭐⭐

### 动机:站立之后为什么需要 MPC?

站立控制器只能让机器人保持固定姿态——它无法**预测**未来的运动需求。当你想让机器人行走时,需要一个能规划未来 0.5-1 秒质心轨迹和接触力的控制器。这就是 MPC(Model Predictive Control)的角色。

回顾 Ch55 的内容:OCS2 的 MPC 使用 SQP 求解非线性 OCP,计算量大但通用性强。对于 Mini-Legged 项目,我们选择更简单的方案——**SRB(Single Rigid Body)凸 MPC**,其核心思想来自 MIT Mini Cheetah 项目(Di Carlo et al., 2018)。

### 为什么选 SRB 凸 MPC 而不是 OCS2 的 SQP?

| 特性 | SRB 凸 MPC | OCS2 SQP MPC |
|------|-----------|--------------|
| 模型复杂度 | 13 维状态,线性化 | 24+ 维状态,非线性 |
| 求解时间 | <2 ms(QP) | 5-20 ms(SQP 迭代) |
| 实现难度 | 中等(~500 行 C++) | 高(依赖 OCS2 框架) |
| 精度 | 平地足够,粗糙地形受限 | 高精度,适合复杂任务 |
| 教学价值 | 能手推全部数学 | 需要理解 OCS2 抽象层 |

**设计决策**:Mini-Legged 先用 SRB 凸 MPC 跑通全栈,后续可替换为 OCS2 SQP。

### 69.5.1 SRB 模型复习与实现 ⭐⭐

SRB 模型的核心假设:**把整个机器人视为一个刚体**,忽略腿的惯性。这个假设在 Go2 这类小型四足上相当准确(腿质量 < 总质量的 15%)。

**状态向量**(13 维):

$$\mathbf{x} = [\underbrace{\Theta_x, \Theta_y, \Theta_z}_{\text{欧拉角}},\ \underbrace{p_x, p_y, p_z}_{\text{质心位置}},\ \underbrace{\omega_x, \omega_y, \omega_z}_{\text{角速度}},\ \underbrace{\dot{p}_x, \dot{p}_y, \dot{p}_z}_{\text{线速度}},\ \underbrace{1}_{\text{常数项}}]^T$$

**为什么把常数 1 放进状态?** 这是一个工程技巧——把重力这种常量加速度写进线性系统矩阵中，使动力学方程保持仿射项被吸收到状态里的形式：

$$\dot{\mathbf{x}} = A_c \mathbf{x} + B_c \mathbf{u}$$

其中 $\mathbf{u} = [\lambda_1^T, \lambda_2^T, \lambda_3^T, \lambda_4^T]^T \in \mathbb{R}^{12}$ 是四只脚的接触力。

**连续时间动力学矩阵**:

$$A_c = \begin{bmatrix} 0_{3\times3} & 0 & R_z^T & 0 & 0 \\ 0 & 0 & 0 & I_3 & 0 \\ 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf{e}_g \\ 0 & 0 & 0 & 0 & 0 \end{bmatrix}$$

$$B_c = \begin{bmatrix} 0 \\ 0 \\ I_G^{-1}[r_1]_\times & I_G^{-1}[r_2]_\times & I_G^{-1}[r_3]_\times & I_G^{-1}[r_4]_\times \\ \frac{1}{m}I_3 & \frac{1}{m}I_3 & \frac{1}{m}I_3 & \frac{1}{m}I_3 \\ 0 \end{bmatrix}$$

其中 $r_i = p_{\text{foot},i} - p_{\text{com}}$ 是足端相对于质心的向量,$[r]_\times$ 是反对称矩阵,$R_z$ 是 yaw 旋转矩阵(线性化近似),$\mathbf{e}_g = [0, 0, -9.81]^T$。最后一维状态恒为 1，所以 $\dot{v}$ 方程中的最后一列直接贡献重力加速度；不要再把最后一维写成 $g=-9.81$，否则会把重力量级重复乘进去。

**离散化**(零阶保持):

$$A_d = I + A_c \cdot dt, \quad B_d = B_c \cdot dt$$

这是一阶近似,对于 $dt = 20\text{ms}$ 的 MPC 时步足够精确。

```cpp
// C++ 风格算法骨架：展示 SRB MPC 的数据流。
// buildCostMatrix/buildCostVector/buildFrictionConstraints 需要在练习中补全。
class ConvexMpc {
public:
    static constexpr int kStateDim = 13;
    static constexpr int kInputDim = 12;  // 4 feet x 3D force
    static constexpr int kHorizon = 10;
    static constexpr double kDt = 0.02;   // 20ms per step

    struct MpcResult {
        Eigen::Matrix<double, 12, 1> forces;  // 当前时步的接触力
        Eigen::Matrix<double, 13, 1> predicted_state;  // 下一步预测
    };

    MpcResult solve(
        const Eigen::Matrix<double, 13, 1>& x0,
        const Eigen::Matrix<double, 13, Eigen::Dynamic>& x_ref,
        const Eigen::Matrix<double, 3, 4>& foot_positions,
        const Eigen::Matrix<int, 4, Eigen::Dynamic>& contact_schedule
    ) {
        // 1. 构建 QP: min 0.5 z^T H z + g^T z
        //    s.t. A_eq z = b_eq, A_ineq z <= b_ineq
        int n_dec = kInputDim * kHorizon;  // 120 决策变量

        // 2. 构建代价矩阵
        Eigen::MatrixXd H = buildCostMatrix(x_ref);
        Eigen::VectorXd g_vec = buildCostVector(x0, x_ref);

        // 3. 构建约束(摩擦锥 + 接触模式)
        auto [A_ineq, b_ineq] = buildFrictionConstraints(
            contact_schedule, 0.6);  // mu = 0.6

        // 4. 调用 ProxQP 求解
        proxsuite::proxqp::dense::QP<double> qp(n_dec, 0,
            A_ineq.rows());
        qp.init(H, g_vec, std::nullopt, std::nullopt,
                A_ineq, b_ineq);
        qp.solve();

        MpcResult result;
        result.forces = qp.results.x.head(kInputDim);
        result.predicted_state = x0;  // 教学占位；完整实现应由 Ad/Bd 滚动预测得到。
        return result;
    }
};
```

### 69.5.2 OCS2 legged_robot 配置(进阶路径) ⭐⭐⭐

如果你选择使用 OCS2 而非自写凸 MPC,需要配置以下文件:

**task.info**(OCS2 的核心配置文件):

```ini
; OCS2 legged_robot task.info 关键参数

; 模型选择
model_settings
{
  ; "CentroidalModel" 或 "KinoCentroidalModel"
  modelType = CentroidalModel
  ; 步态分辨率
  phaseTransitionStanceTime = 0.1
}

; SQP 求解器参数
sqp
{
  nThreads = 2
  dt = 0.015              ; 每步 15ms
  sqpIteration = 1        ; RTI 模式:只做 1 次 SQP 迭代
  deltaTol = 1e-3
  costTol = 1e-4
}

; MPC 参数
mpc
{
  timeHorizon = 1.0       ; 预测 1 秒
  solutionTimeWindow = 0.2
  coldStart = false
}

; 代价函数权重
; Q 矩阵(状态跟踪):位姿精度 vs 速度精度的权衡
; R 矩阵(输入正则):防止接触力过大
```

**frame_declaration.info**(告诉 OCS2 机器人的拓扑):

```ini
; 为 Go2 配置的 frame declaration
contactNames3DoF
{
  [0] FL_foot
  [1] FR_foot
  [2] RL_foot
  [3] RR_foot
}

; 足端名称必须与 URDF 中的 frame 名称完全匹配!
```

> **⚠️ 编程陷阱:frame_declaration 中的名称与 URDF 不匹配**
> - 错误做法:OCS2 配置文件写 `FL_FOOT`(大写),URDF 里是 `FL_foot`(小写)
> - 现象:OCS2 启动时不报错,但接触力全为零,机器人直接摔
> - 根本原因:OCS2 按名称查找 Pinocchio 的 frame,名称不匹配就找不到接触点
> - 正确做法:用 `grep` 在 URDF 中搜索确切的 frame 名称,复制粘贴到配置文件

### 69.5.3 MPC 与 WBC 的接口设计 ⭐⭐

MPC 和 WBC 运行在不同频率,需要明确定义它们之间传递的数据:

```
MPC (50 Hz)                        WBC (1 kHz)
    |                                   |
    | 输出: MpcSolution                  | 输入: WbcCommand
    | - x_ref[k..k+N]  (状态参考轨迹)   | - com_acc_des (质心加速度)
    | - u_ref[k..k+N]  (接触力参考)     | - omega_dot_des (角加速度)
    | - contact_schedule (接触时序)      | - foot_pos_des (足端位置)
    |                                   | - foot_vel_des (足端速度)
    |---- 通过线程安全缓冲区传递 ------->| - contact_flags (接触状态)
    |                                   | - f_ref (接触力参考)
    |                                   |
    |                                   | 输出:
    |                                   | - tau (12 维关节扭矩)
```

**线程安全通信**:MPC 在独立线程运行,WBC 在实时线程运行。两者之间用无锁的 latest-value buffer 或严格三重缓冲通信,避免互斥锁导致的实时性问题(回顾 Ch55 的双线程架构)。

```cpp
// SPSC latest-value buffer:一个 MPC 写线程,一个 WBC 读线程。
// read() 在收到第一帧前返回 false，避免使用未初始化数据。
template <typename T>
class LatestBuffer {
public:
    void write(T data) {
        const uint8_t next = 1 - write_index_.load(std::memory_order_relaxed);
        slots_[next] = std::move(data);
        write_index_.store(next, std::memory_order_release);
        has_value_.store(true, std::memory_order_release);
    }

    bool read(T& out) const {
        if (!has_value_.load(std::memory_order_acquire)) {
            return false;
        }
        const uint8_t index = write_index_.load(std::memory_order_acquire);
        out = slots_[index];
        return true;
    }

private:
    std::array<T, 2> slots_{};
    std::atomic<uint8_t> write_index_{0};
    std::atomic<bool> has_value_{false};
};
```

严格的三重缓冲也可以实现“写槽、读槽、待交换槽”三者分离，但代码必须保证初始读槽已经写入过、交换后不会把空指针交给写端。本章这里用双槽 latest-value buffer 表达同一个工程意图：WBC 只需要读到 MPC 的**最新完整解**，不需要消费每一帧历史消息。

### ⚠️ 常见陷阱

> **💡 概念误区:认为 MPC 频率越高越好**
> - 新手想法:"MPC 20 Hz 太慢了,提高到 200 Hz 不是更好?"
> - 实际上:MPC 频率受限于求解时间。SRB 凸 MPC 求解 <2ms,理论上可以 500 Hz。但 OCS2 的 SQP 求解需要 5-20ms,所以只能 50-100 Hz。**关键是 MPC 和 WBC 之间的插值质量**,不是 MPC 频率本身
> - 正确理解:MPC 输出的是一条**轨迹**(不是单个点),WBC 在两次 MPC 更新之间沿轨迹插值,所以即使 MPC 只有 30 Hz,WBC 仍然可以在 1 kHz 下平滑执行

> **⚠️ 编程陷阱:SRB 模型的惯性张量用了体坐标系下的值而非世界坐标系**
> - Di Carlo 2018 的原始公式中,$I_G$ 是在**世界坐标系下**表示的惯性张量
> - 即 $I_G^{\text{world}} = R \cdot I_G^{\text{body}} \cdot R^T$
> - 但 SRB 模型的近似是:**在名义姿态下**用 $I_G^{\text{body}}$ 近似 $I_G^{\text{world}}$(假设姿态变化小)
> - 这个近似在 pitch/roll < 15 度时足够好,但大倾斜时会引入误差

### 练习

**练习 69.5.1** [必做]:实现 SRB 凸 MPC(不用 OCS2),在站立状态下验证 MPC 输出的接触力是否合理(每只脚约承担 1/4 体重)。

**练习 69.5.2** [进阶]:给 MPC 输入一个前进速度命令(如 $v_x = 0.3$ m/s),观察 MPC 规划的质心轨迹和接触力变化。绘制 MPC 输出的力随时间变化曲线。

**练习 69.5.3** [思考]:SRB 模型忽略了腿的惯性。对于 Go2(总质量约 15 kg,单腿约 1.5 kg),估算这个近似引入的误差大约是多少百分比。

---

## 69.6 WBC 集成 ⭐⭐

### 动机:MPC 输出了接触力参考,为什么还需要 WBC?

MPC 使用简化模型(SRB),输出的是**质心级**的接触力和轨迹参考。但电机需要的是**关节扭矩**。WBC(Whole-Body Controller)的任务是:把质心级指令转换为关节级命令,同时满足全身动力学约束、摩擦锥约束和关节限位。

用一个类比:MPC 是"战略规划部"(决定大方向),WBC 是"执行部门"(把战略落实为具体行动)。

### 69.6.1 WBC 的 QP 公式 ⭐⭐

回顾 Ch53 的 WBC 框架,Mini-Legged 采用**单层加权 QP**(比 legged_control 的分层 HQP 更简单):

**决策变量**:$\mathbf{z} = [\ddot{q}^T, \lambda^T]^T$

- $\ddot{q} \in \mathbb{R}^{18}$:广义加速度(6 浮动基座 + 12 关节)
- $\lambda \in \mathbb{R}^{12}$:接触力(4 脚 x 3D)

**硬约束**(等式):

1. **浮动基座动力学**(前 6 行):

$$M_b \ddot{q} + h_b = J_{c,b}^T \lambda$$

2. **接触脚零加速度**:

$$J_c \ddot{q} = -\dot{J}_c \dot{q}$$

**硬约束**(不等式):

3. **摩擦锥**(线性化为 4 面锥):

$$|\lambda_x^{(i)}| \leq \mu \lambda_z^{(i)}, \quad |\lambda_y^{(i)}| \leq \mu \lambda_z^{(i)}, \quad \lambda_z^{(i)} \geq f_{\min}$$

**软目标**(加权代价):

$$\min_{\ddot{q}, \lambda} \quad w_1 \|\ddot{p}_G - \ddot{p}_G^{\text{ref}}\|^2 + w_2 \|\dot{\omega} - \dot{\omega}^{\text{ref}}\|^2 + w_3 \|\ddot{p}_{\text{swing}} - \ddot{p}_{\text{swing}}^{\text{ref}}\|^2 + w_4 \|\ddot{q}\|^2 + w_5 \|\lambda - \lambda^{\text{ref}}\|^2$$

**反推关节扭矩**:

$$\tau = S \left(M \ddot{q} + h - J_c^T \lambda\right)$$

其中 $S = [0_{12 \times 6}, I_{12}]$ 是选择矩阵，只取广义力的后 12 维关节部分。这里不能写成 $(S^T)^{-1}$：$S^T \in \mathbb{R}^{18\times12}$ 不是方阵，也不存在普通逆矩阵。浮动基座前 6 行用于欠驱动动力学约束，后 12 行才对应执行器扭矩。

```cpp
// C++ 风格算法骨架：展示 WBC 的求解顺序。
// WbcCommand、assembleCost 和各类约束组装函数需按 Ch53 的矩阵定义实现。
class WholeBodyController {
public:
    Eigen::VectorXd solve(
        const WbcCommand& cmd,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const std::array<bool, 4>& contact_flags
    ) {
        // 1. 更新 Pinocchio 数据
        pin_.computeDynamics(q, v);  // M, h, J_c, dJ_c

        // 2. 确定接触脚数量
        int n_contact = std::count(contact_flags.begin(),
                                    contact_flags.end(), true);
        int n_lambda = 3 * n_contact;
        int n_dec = 18 + n_lambda;

        // 3. 组装 QP
        // 代价: H z + g
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_dec, n_dec);
        Eigen::VectorXd g_vec = Eigen::VectorXd::Zero(n_dec);
        assembleCost(H, g_vec, cmd);

        // 等式约束: A_eq z = b_eq
        // - 浮动基座动力学(6行)
        // - 接触脚零加速度(3*n_contact行)
        auto [A_eq, b_eq] = assembleEqualityConstraints(
            contact_flags);

        // 不等式约束: A_ineq z <= b_ineq
        // - 摩擦锥
        auto [A_ineq, b_ineq] = assembleFrictionConstraints(
            contact_flags, 0.6);

        // 4. 求解 QP(ProxQP)
        proxsuite::proxqp::dense::QP<double> qp(
            n_dec, A_eq.rows(), A_ineq.rows());
        qp.init(H, g_vec, A_eq, b_eq, A_ineq, b_ineq);
        qp.solve();

        // 5. 反推扭矩
        Eigen::VectorXd qdd = qp.results.x.head(18);
        Eigen::VectorXd lambda = qp.results.x.tail(n_lambda);

        Eigen::VectorXd tau_full = pin_.M() * qdd + pin_.h()
            - pin_.Jc(contact_flags).transpose() * lambda;

        return tau_full.tail(12);  // 只返回关节扭矩
    }
};
```

### 69.6.2 WBC 实时性保证 ⭐⭐

WBC 运行在 1 kHz 实时循环中,每次求解必须在 1ms 内完成。关键优化:

1. **预分配内存**:使用 `EIGEN_RUNTIME_NO_MALLOC` 检测动态分配(Ch53 讲过)
2. **Warm starting**:用上一时步的 QP 解作为初始猜测
3. **避免 Eigen 动态大小矩阵的临时对象**

```cpp
// 正确:预分配 + 固定大小
Eigen::Matrix<double, 18, 18> M_;  // 质量矩阵(固定大小)
Eigen::Matrix<double, 18, 1> h_;   // 偏置力(固定大小)

// 错误:每次求解都创建动态矩阵
Eigen::MatrixXd M = pin_.crba(q);  // 每次 new 一个堆对象
// 在 1kHz 循环中,这会导致 ~1000 次/秒的 malloc/free
```

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:WBC 的动力学约束写错维度**
> - 浮动基座动力学只取**前 6 行**(对应 6 维浮动基座的欠驱动方程)
> - 如果错误地用了全 18 行,约束过多,QP 可能无解
> - 正确理解:前 6 行对应"基座被动受力",后 12 行对应"关节主动驱动"

> **💡 概念误区:认为 WBC 的摩擦锥约束是"硬物理限制"**
> - 实际上:摩擦系数 $\mu$ 在仿真中是已知参数,但在真机上是估计值
> - 设 $\mu = 0.6$ 但实际地面可能是 0.3(冰面),WBC 会输出超过真实摩擦锥的力
> - 工程做法:在 WBC 中设 $\mu$ 略低于真实值(保守估计),留安全裕度

### 练习

**练习 69.6.1** [必做]:实现 WBC 并与 MPC 串联。在 MuJoCo 中验证:站立时 WBC 输出的扭矩是否与纯 PD + 重力补偿接近。

**练习 69.6.2** [进阶]:测量 WBC 的 QP 求解时间。画出"接触脚数量 vs 求解时间"的曲线(0 脚 / 2 脚 / 4 脚)。

**练习 69.6.3** [思考]:为什么 legged_control 用分层 HQP 而 Mini-Legged 用加权 QP?在什么情况下加权 QP 会出问题?(提示:想想当两个任务权重相近时...)

---

## 69.7 步态实现 ⭐⭐

### 动机:站立到行走的关键一步

站立时,四只脚始终着地。行走时,必须有脚抬起(swing phase)和落下(stance phase)的循环——这就是**步态(gait)**。步态调度决定了"哪只脚在什么时候抬起/落下",是从站立到行走的核心跨越。

### 69.7.1 基本步态类型 ⭐

| 步态 | 对角支撑 | 特征 | 速度范围 | 稳定性 |
|------|---------|------|---------|--------|
| **Walk** | FL-RR → FR-RL | 每次只抬一只脚,3 脚支撑 | 0-0.5 m/s | 高(静态稳定) |
| **Trot** | FL+RR → FR+RL | 对角两脚同时抬,2 脚支撑 | 0.3-1.5 m/s | 中(动态稳定) |
| **Pace** | FL+RL → FR+RR | 同侧两脚同时抬 | 0.5-1.0 m/s | 低(横向不稳) |
| **Bound** | FL+FR → RL+RR | 前两脚和后两脚交替 | 1.0-3.0 m/s | 低(纵向冲击大) |
| **Gallop** | 非对称 | 四脚依次着地 | 2.0-5.0 m/s | 最低 |

**Mini-Legged 先实现 Trot**,因为它是工程中最常用的步态:
- 两脚支撑提供足够稳定性
- 对角腿交替,天然抵消 yaw 力矩
- 速度范围覆盖大多数室内应用

### 69.7.2 步态调度器实现 ⭐⭐

步态调度器用一个**相位变量** $\phi \in [0, 1)$ 来描述步态周期中的位置:

```cpp
class GaitSchedule {
public:
    struct GaitParams {
        double period;          // 步态周期(秒)
        double duty_factor;     // 支撑相占比(0-1)
        Eigen::Vector4d offsets; // 四腿相位偏移
    };

    GaitSchedule() : params_(trot()) {}

    explicit GaitSchedule(GaitParams params)
        : params_(std::move(params)) {}

    // 预定义步态
    static GaitParams trot() {
        // 腿顺序采用本章内部 [FL, FR, RL, RR]。
        // FL 和 RR 同相，FR 和 RL 同相（对角支撑）。
        return GaitParams{
            0.4,    // 400ms 一个周期
            0.5,    // 50% 支撑,50% 摆动
            Eigen::Vector4d(0.0, 0.5, 0.5, 0.0)
        };
    }

    static GaitParams walk() {
        return GaitParams{
            0.8,
            0.75,   // 75% 支撑(3 脚时间更长)
            Eigen::Vector4d(0.0, 0.5, 0.25, 0.75)
        };
    }

    // 计算当前时刻每条腿的接触状态
    std::array<bool, 4> getContactFlags(double time) const {
        std::array<bool, 4> flags;
        for (int i = 0; i < 4; ++i) {
            double phase = std::fmod(
                time / params_.period + params_.offsets[i], 1.0);
            flags[i] = (phase < params_.duty_factor);  // 支撑相
        }
        return flags;
    }

    // 计算摆动腿的归一化相位(0=刚抬起, 1=即将落下)
    double getSwingPhase(int leg_id, double time) const {
        double phase = std::fmod(
            time / params_.period + params_.offsets[leg_id], 1.0);
        if (phase >= params_.duty_factor) {
            return (phase - params_.duty_factor) /
                   (1.0 - params_.duty_factor);
        }
        return -1.0;  // 不在摆动相
    }

private:
    GaitParams params_;
};
```

### 69.7.3 摆动腿轨迹规划 ⭐⭐

摆动腿需要从当前位置移动到目标落脚点。轨迹规划需要:
1. 抬脚到目标高度(避免磕碰地面)
2. 前移到目标位置
3. 平稳落地(垂直速度小)

**Raibert 启发式落脚点选取**(Raibert, 1986):

$$p_{\text{target}} = p_{\text{hip}} + \frac{T_{\text{stance}}}{2} v_{\text{body}} + K_v (v_{\text{body}} - v_{\text{cmd}})$$

其中:
- $p_{\text{hip}}$:髋关节在地面的投影
- $T_{\text{stance}}$:支撑相时长
- $v_{\text{body}}$:当前体速度
- $v_{\text{cmd}}$:命令速度
- $K_v$:速度反馈增益(典型值 0.03-0.1)

**三次样条摆动轨迹**:

```python
def swing_trajectory(t_phase, start_pos, target_pos, swing_height=0.06):
    """
    抛物线抬脚的摆动轨迹示例
    t_phase: 0 -> 1 (摆动相归一化时间)
    """
    # 水平方向:线性插值
    xy = (1 - t_phase) * start_pos[:2] + t_phase * target_pos[:2]

    # 垂直方向:先保证端点正确，再叠加中间抬脚高度。
    z_base = (1 - t_phase) * start_pos[2] + t_phase * target_pos[2]
    z = z_base + swing_height * 4 * t_phase * (1 - t_phase)
    # 4*t*(1-t) 在 t=0 和 t=1 为 0，在 t=0.5 达到 1

    return np.array([xy[0], xy[1], z])
```

### 69.7.4 步态切换逻辑 ⭐⭐

从站立切换到 trot 不能瞬间完成——需要平滑过渡:

```
站立 -> 预备(降低重心 + 重新分配重量) -> 第一步 -> 稳态 trot
       [~0.5 秒]                        [~1 秒过渡]
```

**工程实现**:

```cpp
enum class GaitState { STAND, TRANSITION, TROT, WALK };

class GaitStateMachine {
public:
    void requestGait(GaitState target) {
        if (current_ == target) return;

        if (current_ == GaitState::STAND && target == GaitState::TROT) {
            // 站立->Trot:先降重心,再开始步态
            transition_timer_ = 0.5;  // 0.5秒过渡
            current_ = GaitState::TRANSITION;
            next_ = target;
        }
    }

    void update(double dt) {
        if (current_ == GaitState::TRANSITION) {
            transition_timer_ -= dt;
            if (transition_timer_ <= 0) {
                current_ = next_;
            }
        }
    }

private:
    GaitState current_ = GaitState::STAND;
    GaitState next_ = GaitState::STAND;
    double transition_timer_ = 0.0;
};
```

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:摆动腿落地瞬间的冲击**
> - 问题:脚从空中高速落地,产生巨大冲击力,导致机器人弹起或不稳
> - 根本原因:轨迹规划没有约束落地时的垂直速度
> - 正确做法:在摆动相最后 10% 阶段,将垂直速度逐渐降为零(软着陆)
> - 进阶做法:落地前 2-3ms 切换到低增益 PD("预接触缓冲"),吸收冲击

> **💡 概念误区:认为步态周期是固定不变的**
> - 实际上:速度越快,步态周期越短(频率越高)。Go2 在 0.3 m/s 时 trot 周期约 0.5s,在 1.5 m/s 时约 0.3s
> - 更高级的实现会根据速度自适应调整步态参数

### 练习

**练习 69.7.1** [必做]:实现 trot 步态调度器 + 摆动腿三次样条轨迹。在 MuJoCo 中让 Go2 做 trot 行走,前进速度 0.3 m/s,录屏保存。

**练习 69.7.2** [进阶]:实现 walk 步态,与 trot 对比在 0.2 m/s 下的稳定性(测量 roll/pitch 的最大偏差)。

**练习 69.7.3** [思考]:为什么 bound 步态在小型四足上很少使用?从能量效率和硬件限制两个角度分析。

---

## 69.8 RL 策略训练 ⭐⭐⭐

### 动机:MPC+WBC 之后,为什么还需要 RL?

MPC+WBC 在平地上表现良好,但面对以下挑战时力不从心:

| 挑战 | MPC+WBC 的困难 | RL 的优势 |
|------|---------------|----------|
| 复杂地形(楼梯、碎石) | 需要精确的地形模型 | 隐式学习地形适应 |
| 未知扰动(被踢、负载变化) | 依赖线性化模型的鲁棒性 | 训练时大量随机化 |
| 高动态运动(翻转、跳跃) | SRB 模型近似失效 | 端到端学习全身动力学 |
| 软硬件差异(sim-to-real) | 需要精确的系统辨识 | Domain Randomization 天然处理 |

**RL 不是替代 MPC+WBC,而是互补**。工业界的趋势是:用 MPC+WBC 提供安全基线,用 RL 提供鲁棒性和适应性。

### 69.8.1 IsaacLab 训练管线概览 ⭐⭐

IsaacLab(原 Isaac Gym / Orbit)是目前最流行的 RL 训练框架,能在单张 GPU 上并行运行 4096 个仿真环境:

```
训练管线:

+-------------------------------------------+
| IsaacLab (GPU 并行仿真)                    |
|                                           |
| +---------+ +---------+ +---------+      |
| | Env 1   | | Env 2   | |...Env N |      | N = 4096
| | Go2     | | Go2     | | Go2     |      |
| | + 随机  | | + 随机  | | + 随机  |      |
| |   地形  | |   摩擦  | |   质量  |      |
| +----+----+ +----+----+ +----+----+      |
|      |           |           |           |
|      +-----------+-----------+           |
|                  |                       |
|          +-------v-------+               |
|          | Observation   |               |
|          | (body vel,    |               |
|          |  joint pos/vel|               |
|          |  commands,    |               |
|          |  gravity proj)|               |
|          +-------+-------+               |
+------------------+---+-------------------+
                   |
           +-------v-------+
           | PPO (rsl_rl)  |
           | Actor-Critic  |
           | MLP [512,256] |
           +-------+-------+
                   | actions: 12 dim
                   | (目标关节角度)
                   v
           下发到所有 N 个环境
```

### 69.8.2 观测空间与动作空间设计 ⭐⭐

**观测空间**(~48 维,取决于具体配置):

| 观测量 | 维度 | 来源 | 作用 |
|--------|------|------|------|
| 基座角速度 | 3 | IMU | 感知旋转 |
| 重力在体坐标系下的投影 | 3 | IMU | 感知倾斜 |
| 速度命令 | 3 | 外部输入 | 目标跟踪 |
| 关节位置(偏差) | 12 | 编码器 | 当前姿态 |
| 关节速度 | 12 | 编码器 | 运动状态 |
| 上一步动作 | 12 | 控制器 | 平滑性 |

**动作空间**(12 维):

RL 策略输出的是**目标关节角度偏差** $\Delta q$,而非直接输出扭矩:

$$q_{\text{target}} = q_{\text{default}} + \Delta q \cdot \text{action\_scale}$$

$$\tau = K_p (q_{\text{target}} - q) + K_d (0 - \dot{q})$$

**为什么输出位置而非扭矩?**
- 位置控制更稳定:即使策略输出垃圾值,PD 环也能提供一定的被动阻尼
- 更容易 sim-to-real:位置控制对模型误差不那么敏感
- 与传统控制框架兼容:可以直接叠加在 MPC 输出上

> **跨领域类比**：RL 策略输出位置偏差而非直接扭矩,就像自动驾驶系统输出"目标车道"而非"方向盘角度"。目标车道(位置)经过底层的 PD 转向控制器才变成方向盘角度(扭矩)——即使上层策略偶尔给出奇怪的车道目标,转向控制器也会平滑、安全地执行,不会瞬间打满方向盘。这个中间层(PD 环/转向控制器)充当了"安全缓冲",把上层策略的不完美与底层执行器的物理极限隔离开来。

### 69.8.3 奖励函数设计 ⭐⭐⭐

奖励函数是 RL 训练中最关键的设计,也是最需要经验的部分。以下是 IsaacLab 中 Go2 locomotion 的典型奖励组成:

| 奖励项 | 公式 | 权重 | 作用 |
|--------|------|------|------|
| **线速度跟踪** | $\exp(-\|v_{xy} - v_{xy}^{\text{cmd}}\|^2 / \sigma^2)$ | 1.0 | 核心目标:跟踪命令 |
| **角速度跟踪** | $\exp(-\|\omega_z - \omega_z^{\text{cmd}}\|^2 / \sigma^2)$ | 0.5 | 转向跟踪 |
| **线速度 z 惩罚** | $-v_z^2$ | -2.0 | 抑制上下弹跳 |
| **角速度 xy 惩罚** | $-\|\omega_{xy}\|^2$ | -0.05 | 抑制 roll/pitch 振荡 |
| **关节加速度惩罚** | $-\|\ddot{q}\|^2$ | -2.5e-7 | 平滑运动 |
| **动作变化率惩罚** | $-\|a_t - a_{t-1}\|^2$ | -0.01 | 抑制抖动 |
| **扭矩惩罚** | $-\|\tau\|^2$ | -1e-5 | 能量效率 |
| **足端气时惩罚** | 脚在空中但不移动 | -1.0 | 避免"悬浮" |
| **足端碰地奖励** | 着地时垂直速度小 | 0.25 | 鼓励软着陆 |

**奖励设计的核心原则**:

1. **正奖励要用指数核(exponential kernel)**:$r = \exp(-e^2/\sigma^2)$ 而非 $r = -e^2$。原因:指数核在误差为零时给最大奖励(=1),在误差大时快速衰减但不为负,避免了"大误差 = 大惩罚 → 策略倾向不动"的问题。

2. **惩罚项的权重要比奖励项小 1-2 个数量级**:惩罚太大会让策略过于保守(站着不动是"安全"的)。

3. **不要一次加入所有奖励项**:先只用速度跟踪 + 存活奖励训练出能走的策略,然后逐步加入平滑性、能效等惩罚。

```python
# IsaacLab 奖励函数配置示例
class Go2RewardsCfg:
    # --- 正奖励(追踪目标) ---
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"std": 0.25}  # sigma
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"std": 0.25}
    )

    # --- 负惩罚(正则化) ---
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2, weight=-2.0
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2, weight=-0.05
    )
    action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2, weight=-0.01
    )
    joint_acc_l2 = RewTerm(
        func=mdp.joint_accel_l2, weight=-2.5e-7
    )
```

### 69.8.4 Domain Randomization(域随机化) ⭐⭐⭐

Domain Randomization 是 sim-to-real 成功的关键——通过在训练时随机化仿真参数,让策略学会对不确定性的鲁棒性:

| 随机化参数 | 范围 | 作用 |
|-----------|------|------|
| 地面摩擦系数 | [0.3, 1.2] | 适应不同地面 |
| 机器人质量 | [+/-20%] | 适应负载变化 |
| 质心位置偏移 | [+/-5 cm] | 适应负载分布 |
| 关节阻尼 | [+/-30%] | 适应电机特性差异 |
| PD 增益 | [+/-20%] | 适应底层控制器差异 |
| 外部推力 | [0, 10 N] 随机 | 抗扰能力 |
| 地形高度 | [0, 10 cm] | 地形适应 |

```python
# IsaacLab Domain Randomization 配置
class Go2DomainRandCfg:
    # 每 episode 开始时随机化一次
    randomize_friction = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "static_friction_range": (0.4, 1.0),
            "dynamic_friction_range": (0.4, 1.0),
        }
    )
    randomize_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={"mass_distribution_params": (-1.5, 1.5)}
    )

    # 每步都随机的(更强的鲁棒性训练)
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5),
                                    "y": (-0.5, 0.5)}}
    )
```

### 69.8.5 训练流程与超参数 ⭐⭐

```bash
# 启动训练。耗时强依赖 IsaacLab 版本、显卡、num_envs、地形复杂度和终止条件；
# 单卡 3090/4090 上平地 Go2 任务通常按“几十分钟到数小时”估算更稳妥。
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 4096 \
    --max_iterations 1500 \
    --headless

# 关键超参数
# PPO 配置(rsl_rl):
#   learning_rate: 1e-3
#   num_mini_batches: 4
#   num_learning_epochs: 5
#   gamma: 0.99
#   lam: 0.95
#   clip_param: 0.2
#   network: MLP [512, 256, 128]
#   activation: ELU
```

**训练监控**:

```bash
# 用 TensorBoard 监控训练
tensorboard --logdir logs/rsl_rl

# 关键指标:
# - episode_reward: 应持续上升
# - episode_length: 应增加(存活更久)
# - tracking_lin_vel: 应趋近 1.0(完美跟踪)
```

IsaacLab/rsl_rl 的日志目录通常是 `logs/rsl_rl/<experiment_name>/<run_name>/...`，`experiment_name` 由任务配置决定，不一定等于 `Isaac-Velocity-Flat-Unitree-Go2-v0` 这个任务 ID。调试时先用 `find logs/rsl_rl -maxdepth 3 -type d` 看实际生成目录，再打开对应 TensorBoard。

**训练完成标志**:
1. episode_reward 收敛(不再明显上升)
2. 在 viewer 中可视化:策略能稳定行走、跟踪速度命令
3. 对随机推力有恢复能力

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:IsaacLab 版本与 rsl_rl 版本不匹配**
> - IsaacLab 绑定特定版本的 rsl_rl,混用版本会导致 tensor shape 不匹配
> - 正确做法:使用 IsaacLab 的 `./isaaclab.sh --install` 自动安装匹配版本

> **🧠 思维陷阱:认为训练越久越好**
> - 新手想法:"训练 10000 iterations 肯定比 1500 好"
> - 实际上:过长训练可能导致**过拟合到训练地形/参数分布**,在真机上反而更差
> - 正确做法:用 early stopping,在 reward 不再上升时停止;或用 checkpoint 保存多个阶段的模型,在真机上分别测试

> **💡 概念误区:认为 RL 可以完全替代传统控制**
> - RL 策略在训练分布外的泛化能力有限
> - 安全关键场景(如真机起身)仍需要传统控制器兜底
> - 工业级方案通常是 RL + 安全监控器(如 CBF/安全集)

### 练习

**练习 69.8.1** [必做]:在 IsaacLab 中训练 Go2 的平地行走策略。记录训练曲线(reward vs iteration),截图保存。

**练习 69.8.2** [进阶]:修改奖励函数:去掉 `action_rate_l2` 惩罚后重新训练。对比有/无动作平滑惩罚的策略在 viewer 中的表现(抖动程度)。

**练习 69.8.3** [研究级]:实现 Teacher-Student 蒸馏:先训练一个有**特权信息**(地面高度、真实摩擦系数)的 teacher 策略,再蒸馏到只有传感器输入的 student 策略。对比 student 与直接训练的策略的鲁棒性。

---

## 69.9 Sim-to-Real 部署 ⭐⭐⭐

### 动机:仿真到真机的最后一公里

> **本质洞察**：Sim-to-Real 的本质不是"让仿真更像真实"（这是一个无底洞），而是**让控制器对"仿真与真实之间的差异"不敏感**。Domain Randomization 的深层逻辑是：与其精确建模一个真实世界（做不到），不如让控制器在**一大族可能的世界**中都能工作——真实世界只要落在这个族里面，控制器就能泛化。这与鲁棒控制理论中的"不确定集"思想一脉相承：你不需要知道干扰的精确值，只需要知道干扰的范围，然后设计在整个范围内都稳定的控制器。

仿真中完美运行的控制器,部署到真机上可能完全失败。**Sim-to-Real gap**(仿真到现实的鸿沟)是机器人学中最具挑战性的问题之一。本节讲解如何系统性地缩小这个鸿沟。

### 如果不做 Sim-to-Real 处理会怎样?

| Sim-to-Real 差异来源 | 仿真中 | 真机上 | 后果 |
|---------------------|--------|--------|------|
| 电机延迟 | 0 ms | 5-15 ms | 控制器相位超前,产生振荡 |
| 地面摩擦 | 精确已知 | 变化的 | 计划的步态打滑 |
| IMU 噪声 | 高斯白噪声 | 有偏置+漂移 | 状态估计漂移 |
| 关节摩擦 | 无 | 存在 | 需要更大扭矩才能启动 |
| 通信延迟 | 0 ms | 1-5 ms | 控制不及时 |

### 69.9.1 Unitree SDK2 接口 ⭐⭐

Unitree Go2 EDU 使用 SDK2(基于 CycloneDDS)进行低层级控制。与旧版 SDK1 的主要区别:

| 特性 | SDK1 (UDP) | SDK2 (CycloneDDS) |
|------|-----------|-------------------|
| 通信协议 | 原始 UDP | DDS (标准中间件) |
| 发现机制 | 固定 IP | 自动发现 |
| ROS 2 兼容 | 需要桥接 | 原生兼容 |
| 多机器人 | 困难 | 天然支持 |

```python
# Python 部署流程骨架(unitree_sdk2_python)：展示数据流，不是完整上机程序。
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
import numpy as np
import time

class Go2Controller:
    def __init__(self, network_interface="eth0"):
        # 初始化 DDS 通信。网卡名必须换成连接 Go2 的实际接口。
        ChannelFactoryInitialize(0, network_interface)
        self.latest_state = None

        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_callback, 1)

        # 加载 ONNX 策略(从 IsaacLab 导出)
        import onnxruntime as ort
        self.policy = ort.InferenceSession("go2_policy.onnx")

        # 默认站立角度
        # 内部顺序使用 [FL, FR, RL, RR]；写入 SDK2 LowCmd 前必须转换到
        # Unitree 当前 JointIndex 顺序 [FR, FL, RR, RL]。
        self.sdk_index_from_internal = np.array([
            3, 4, 5,   # FL -> SDK2 indices
            0, 1, 2,   # FR
            9, 10, 11, # RL
            6, 7, 8    # RR
        ])
        self.default_pos = np.array([
            0.0, 0.8, -1.6,   # FL
            0.0, 0.8, -1.6,   # FR
            0.0, 1.0, -1.6,   # RL
            0.0, 1.0, -1.6    # RR
        ])

    def _state_callback(self, msg: LowState_):
        self.latest_state = msg

    def run(self):
        while True:
            # 1. 读取状态
            state = self.latest_state
            if state is None:
                time.sleep(0.002)
                continue
            obs = self.build_observation(state)

            # 2. 推理
            action = self.policy.run(None,
                {"obs": obs.astype(np.float32)})[0]

            # 3. 转换为关节目标
            q_target = self.default_pos + action * 0.25  # action_scale

            # 4. 发送命令
            cmd = self.build_low_cmd(q_target, kp=50.0, kd=3.0)
            # build_low_cmd() 必须完成内部顺序 -> SDK2 motor_cmd 索引映射，
            # 并设置 mode/q/dq/kp/kd/tau 和 CRC。
            self.cmd_pub.Write(cmd)

            time.sleep(0.002)  # 500 Hz 控制循环
```

`build_low_cmd()` 的核心不是“把 12 个数顺序写进去”，而是按关节名做重排：

下面是伪代码；不同版本的 `unitree_sdk2_python` 字段 setter 可能不同。
```text
def fill_joint_targets(cmd, q_target_internal, kp, kd, sdk_index_from_internal):
    for internal_i, sdk_i in enumerate(sdk_index_from_internal):
        motor = cmd.motor_cmd()[sdk_i]
        set motor.mode to servo/PMSM mode
        set motor.q    to q_target_internal[internal_i]
        set motor.dq   to 0.0
        set motor.kp   to kp
        set motor.kd   to kd
        set motor.tau  to 0.0
```

> **SDK 版本提示**：以上是部署流程骨架，仍省略了 CRC、运动模式切换、急停保护和完整观测构造。Unitree SDK 更新频繁，类名、导入路径和 CRC 写法可能变化，请以 [unitree_sdk2_python 官方仓库](https://github.com/unitreerobotics/unitree_sdk2_python) 的最新示例为准；真正上机前必须加入官方示例中的模式释放和 CRC 计算。

### 69.9.2 ONNX 模型导出 ⭐⭐

IsaacLab 训练的 PyTorch 策略需要导出为 ONNX 格式,以便在真机上高效推理。不要手动 `ActorCritic(...)` 再 `load_state_dict()`：当前 rsl_rl runner 里可能还包含 observation normalizer、动作裁剪和配置参数，手动重建很容易漏掉。

更稳妥的方式是使用 IsaacLab 的 `play.py` 从 checkpoint 加载完整 runner，并让它导出策略：

```bash
cd ~/IsaacLab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --checkpoint logs/rsl_rl/<experiment_name>/<run_name>/model_1500.pt \
    --num_envs 16

# 导出结果通常位于对应 run 目录下的 exported/ 子目录，例如:
# logs/rsl_rl/<experiment_name>/<run_name>/exported/policy.onnx
# logs/rsl_rl/<experiment_name>/<run_name>/exported/policy.pt
```

部署前还要做一次“同一观测输入、PyTorch 输出 vs ONNX Runtime 输出”的数值对比。如果 observation normalizer 没有一起导出，ONNX 在真机上会看到未归一化观测，动作幅度可能完全错误。

### 69.9.3 安全检查清单 ⭐⭐⭐

**在真机上运行任何控制器之前,必须完成以下检查**:

```
  真机部署安全清单

  1. 硬件检查
  [ ] 电池电量 > 50%
  [ ] 所有关节能手动自由转动(无异常卡顿)
  [ ] 急停按钮可达且测试有效
  [ ] 部署区域有软垫(防摔损)

  2. 软件检查
  [ ] 策略在 MuJoCo 中验证通过
  [ ] 逐关节扭矩限幅已设置（来自 SDK/URDF/MJCF 参数表，并保留实机安全裕度）
  [ ] 关节角度限幅已设置
  [ ] 通信延迟 < 5 ms(用 ping 测试)
  [ ] 急停逻辑已实现(任何异常 -> 零扭矩)

  3. 渐进式测试
  [ ] Step 1: 空载上电,检查传感器读数合理
  [ ] Step 2: 发送零扭矩命令,验证通信正常
  [ ] Step 3: 发送站立姿态(低增益 Kp=20),观察缓慢站起
  [ ] Step 4: 逐步增大 Kp 到目标值
  [ ] Step 5: 发送小速度命令(0.1 m/s),观察缓慢移动
  [ ] Step 6: 逐步增大速度到目标值

  4. 异常处理
  [ ] 任何关节扭矩 > 20 Nm 持续 100ms -> 急停
  [ ] 基座 roll/pitch > 45 度 -> 急停
  [ ] 通信中断 > 50 ms -> 零扭矩 + 报警
```

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:真机上的关节顺序与仿真不同**
> - Unitree SDK2 当前 Go2 低层索引是 `FR_Hip, FR_Thigh, FR_Calf, FL_Hip, ...`，与本章内部 `[FL, FR, RL, RR]` 教学顺序不同
> - **必须建立显式映射表**,而不是假设顺序一致
> - 验证方法:逐个关节发送小角度命令,目视确认是哪个关节在动

> **🧠 思维陷阱:在真机上调参**
> - 新手想法:"仿真里差不多了,真机上微调一下就行"
> - 实际上:真机调参的代价极高(每次尝试都可能摔机器人)
> - 正确做法:所有参数调整在仿真中完成;真机上只做**增益缩放**(先用仿真增益的 50% 开始,逐步提高)

### 练习

**练习 69.9.1** [必做(如果有真机)]:按安全检查清单部署站立控制器到真机 Go2。录制 30 秒站立视频。

**练习 69.9.2** [进阶]:部署 RL 策略到真机。先在软垫上测试,再到硬地面。对比仿真和真机的行走表现,列出所有观察到的差异。

**练习 69.9.3** [思考]:如果 RL 策略在真机上的表现明显差于仿真,你会从哪些方面排查?列出至少 5 个可能的原因和对应的诊断方法。

---

## 69.10 性能调优与故障排除 ⭐⭐

### 动机:所有模块都跑通了,但表现不好怎么办?

集成后的调优是最需要经验的环节。本节汇总所有常见故障的诊断和修复方法。

### 69.10.1 常见故障诊断表 ⭐⭐

| 故障现象 | 可能原因 | 诊断方法 | 修复方案 |
|---------|---------|---------|---------|
| 机器人瞬间弹飞 | WBC 扭矩爆表 | 打印 QP 解,检查扭矩值 | 加扭矩限幅;检查 QP 约束是否正确 |
| 机器人缓慢下沉 | 重力补偿不足 | 在站立状态检查 $\tau_g$ 是否正确 | 验证 Pinocchio 模型的质量参数 |
| 行走时 yaw 漂移 | MPC 的 yaw 追踪权重太低 | 检查 Q 矩阵中 yaw 的权重 | 增大 yaw 追踪权重 |
| 脚在空中抖动 | swing foot PD 增益过高 | 降低摆动腿增益后观察 | 降低 $K_p$ 或加低通滤波 |
| 步态不对称 | 关节零位标定偏差 | 比较四腿在同一命令下的角度 | 重新标定关节零位 |
| MPC 求解超时 | QP 维度过大或 warm start 失效 | 测量 QP 求解时间 | 减小 horizon;启用 warm start |
| 状态估计跳变 | KF 观测噪声设置不当 | 画出估计值 vs 真值的曲线 | 调整 R 矩阵(观测噪声) |
| 着地冲击大 | 摆动轨迹末端速度非零 | 检查轨迹规划的边界条件 | 加软着陆段(着地前降速) |

### 69.10.2 调参优先级 ⭐⭐

当需要同时调多个参数时,按以下优先级:

```
优先级 1:安全相关
  +-- 扭矩限幅、关节限位、急停阈值
      |
优先级 2:基础功能
  +-- 站立 PD 增益、重力补偿
      |
优先级 3:MPC 参数
  +-- Q/R 权重、预测步长、摩擦系数
      |
优先级 4:WBC 参数
  +-- 任务权重、摩擦锥安全裕度
      |
优先级 5:步态参数
  +-- 步态周期、duty factor、摆动高度
      |
优先级 6:性能优化
  +-- 线程优先级、内存预分配、QP warm start
```

### 69.10.3 调试工具链 ⭐

```bash
# 1. ROS 2 话题监控
ros2 topic echo /mini_legged/joint_states  # 关节状态
ros2 topic echo /mini_legged/mpc_solution  # MPC 输出
ros2 topic hz /mini_legged/wbc_output      # WBC 频率

# 2. rqt 可视化
rqt_plot /mini_legged/base_pose/position/z  # 基座高度
rqt_plot /mini_legged/joint_torques         # 关节扭矩

# 3. 性能分析
# 使用 Google Benchmark 测量关键函数耗时
# MPC 求解: < 2 ms
# WBC 求解: < 500 us
# 状态估计: < 100 us
# 总 update(): < 1 ms (1 kHz 约束)

# 4. 录制 rosbag 回放
ros2 bag record -a -o mini_legged_debug
ros2 bag play mini_legged_debug
```

### 69.10.4 性能基准测试 ⭐⭐

```cpp
// Google Benchmark 示例
#include <benchmark/benchmark.h>

static void BM_MpcSolve(benchmark::State& state) {
    ConvexMpc mpc;
    auto x0 = getDefaultState();
    auto x_ref = getStandingReference();
    auto foot_pos = getDefaultFootPositions();
    auto schedule = getTrotSchedule();

    for (auto _ : state) {
        auto result = mpc.solve(x0, x_ref, foot_pos, schedule);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_MpcSolve);

static void BM_WbcSolve(benchmark::State& state) {
    WholeBodyController wbc;
    // ... 类似设置 ...
    for (auto _ : state) {
        auto tau = wbc.solve(cmd, q, v, contact_flags);
        benchmark::DoNotOptimize(tau);
    }
}
BENCHMARK(BM_WbcSolve);

BENCHMARK_MAIN();
```

**目标性能基准**:

| 模块 | 目标延迟 | 实际典型值 | 超标处理 |
|------|---------|-----------|---------|
| MPC 求解(SRB 凸 QP) | < 2 ms | 0.5-1.5 ms | 减小 horizon 或简化约束 |
| WBC 求解(ProxQP) | < 500 us | 100-300 us | 预分配内存,warm start |
| 状态估计(Linear KF) | < 100 us | 20-50 us | 基本不会超标 |
| Pinocchio FK/ID | < 200 us | 50-100 us | 预计算,缓存结果 |
| 总 update() | < 1 ms | 0.5-0.8 ms | 检查上述各项 |

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱:Debug 模式下的性能测试无意义**
> - Debug 模式(-O0)下 Eigen 运算慢 5-10 倍
> - WBC 在 Debug 模式下可能需要 3ms(超标!),但 Release 模式只需 200us
> - **性能测试必须在 Release 或 RelWithDebInfo 模式下进行**

> **💡 概念误区:认为 1 kHz 控制频率是"越高越好"**
> - 1 kHz 对于关节级 PD 控制已经足够
> - WBC 在 1 kHz 下运行也够用
> - 但如果你的控制管线总延迟 > 1ms,降到 500 Hz 并保证**稳定的延迟**,比在 1 kHz 下偶尔超时更好
> - **确定性比频率更重要**

### 练习

**练习 69.10.1** [必做]:用 Google Benchmark 测量你的 MPC 和 WBC 求解时间。画出"n_contact(接触脚数) vs 求解时间"的柱状图。

**练习 69.10.2** [进阶]:在 Gazebo 中给行走的 Go2 施加侧向推力(5N 持续 0.5 秒),记录恢复过程。调整 MPC 的 Q 矩阵权重,找到"最快恢复"的参数组合。

---

## 69.11 本章小结

### 知识点总结

| 节 | 核心内容 | 难度 | 关键产出 |
|----|---------|------|---------|
| 69.1 | 项目总览与开发流程 | ⭐ | 架构图、学习路线 |
| 69.2 | 开发环境搭建 | ⭐ | ROS 2 + Pinocchio + OCS2 + MuJoCo + IsaacLab |
| 69.3 | URDF 与 MuJoCo 模型 | ⭐⭐ | 模型加载、URDF→MJCF、交叉验证 |
| 69.4 | 站立控制器 | ⭐⭐ | PD + 重力补偿,数据通路验证 |
| 69.5 | MPC 集成 | ⭐⭐ | SRB 凸 MPC,线程安全通信 |
| 69.6 | WBC 集成 | ⭐⭐ | 加权 QP,扭矩反推,实时性保证 |
| 69.7 | 步态实现 | ⭐⭐ | Trot 步态调度,摆动腿轨迹,步态切换 |
| 69.8 | RL 策略训练 | ⭐⭐⭐ | IsaacLab 训练,奖励设计,Domain Randomization |
| 69.9 | Sim-to-Real 部署 | ⭐⭐⭐ | SDK2 接口,ONNX 导出,安全清单 |
| 69.10 | 性能调优与故障排除 | ⭐⭐ | 故障诊断表,性能基准,调试工具 |

### 完整系统数据流

```
速度命令 (vx, vy, omega_z)
    |
    v
+----------------+
| 步态调度器      | -> contact_schedule[t..t+T]
| (GaitSchedule) | -> swing_targets
+--------+-------+
         |
         v
+----------------+     +-------------------+
| MPC (50 Hz)    |<----| 状态估计 (1 kHz)   |
| SRB 凸 QP      |     | Linear KF          |
|                |     | IMU + Leg Odom     |
| 输出:         |     +-------------------+
|  x_ref, f_ref  |
+--------+-------+
         | Latest Buffer(无锁)
         v
+----------------+
| WBC (1 kHz)    |
| 加权 QP        |
| (ProxQP)       |
|                |
| 输出: tau[12]  |
+--------+-------+
         |
         v
+----------------+     +----------------+
| 扭矩限幅      |---->| 电机驱动        |
| 安全监控       |     | (Go2 SDK2)     |
+----------------+     +----------------+
```

## 累积项目:本章新增模块

**Mini-Legged 项目进度**:

| 章节 | 新增模块 | 代码量(估计) |
|------|---------|-------------|
| Ch47 | PinocchioInterface 封装 | ~200 行 |
| Ch50 | QP 求解器封装(ProxQP) | ~100 行 |
| Ch51 | SRB 模型 | ~300 行 |
| Ch53 | WBC 核心 | ~500 行 |
| Ch55 | MPC 核心 + 双线程架构 | ~800 行 |
| Ch57 | Linear KF 状态估计 | ~300 行 |
| **Ch69** | **完整集成 + 步态 + RL + 部署** | **~3000 行** |
| **总计** | **Mini-Legged 完整控制栈** | **~5200 行 C++ + Python** |

---

## 延伸阅读

### 必读

| 资源 | 难度 | 内容 |
|------|------|------|
| Di Carlo et al., "Dynamic Locomotion in the MIT Cheetah 3 Through Convex MPC", IROS 2018 | ⭐⭐ | SRB 凸 MPC 的原始论文,你的 MPC 实现的理论基础 |
| Kim et al., "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control", IROS 2019 | ⭐⭐ | MIT Mini Cheetah 的 MPC+WBC 实现,工程参考 |
| Rudin et al., "Learning to Walk in Minutes Using Massively Parallel Deep RL", CoRL 2022 | ⭐⭐ | legged_gym / IsaacLab 训练管线的原始论文 |

### 进阶

| 资源 | 难度 | 内容 |
|------|------|------|
| Grandia et al., "Perceptive Locomotion Through Nonlinear Model-Predictive Control", T-RO 2023 | ⭐⭐⭐ | OCS2 + 高程图的集成 |
| Miki et al., "Learning Robust Perceptive Locomotion for Quadrupedal Robots in the Wild", Science Robotics 2022 | ⭐⭐⭐ | Teacher-Student 蒸馏 + 真实部署 |
| Schwarke et al., "RSL-RL: A Learning Library for Robotics Research", arXiv 2025 | ⭐⭐ | rsl_rl 库的设计哲学与最新 API |

### 代码参考

| 仓库 | 用途 |
|------|------|
| [legubiao/ocs2_ros2](https://github.com/legubiao/ocs2_ros2) | OCS2 的 ROS 2 移植版 |
| [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control) | ROS 2 四足控制框架(含 Gazebo Harmonic 支持) |
| [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) | Go2 的 MJCF 模型 |
| [unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | Go2 SDK2 Python 接口 |
| [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) | RL 训练框架 |
| [leggedrobotics/rsl_rl](https://github.com/leggedrobotics/rsl_rl) | 机器人 RL 库 |
