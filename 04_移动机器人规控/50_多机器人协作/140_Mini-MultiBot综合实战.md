# 第 13 章　Mini-MultiBot 综合实战——从零搭一个完整的多机协作系统

**性质**：工程实践教学 | **难度跨度**：⭐⭐ ~ ⭐⭐⭐⭐ | **预计实战**：30-40 小时（2 周）

> **本章性质**：✅ **全方向共享**——这是整个多机器人方向的 **capstone（顶点项目）**。前十二章把"多机协作"这棵知识树的每一根枝桠分别讲透了：第 1 章给了看系统的地图，第 2 章给了连续协调的数学引擎（共识/ADMM），第 3 章给了离散决策层（任务分配/MAPF），第 4 章把这些下沉到连续控制层（分布式 MPC 编队），第 6 章拆掉了"所有机器人都一样"的假设（异构协同），第 10-12 章引入了学习范式（MARL 及其与规控的混合）。本章不再教任何**新理论**——它的唯一使命是把这些散落的工具**焊成一个能跑、能调、能部署的系统**，让你亲手体会"一个真实的多机系统是怎么从一行 `ros2 launch` 长出来的"。

> **一句话定位**：前十二章你学会了多机协作的"零件"，本章教你"装配"。我们会从一个空目录开始，一步步搭起 Mini-MultiBot——一个支持 N 个异构机器人、跑分布式架构、集成任务分配+编队+避障+MARL、能在 Gazebo/Crazyswarm2 仿真也能上真机的完整协作栈。本章给你**可运行的代码框架**和**里程碑式 checklist**，照着走，两周后你会有一个开源在 GitHub 上、自己从头理解每一行的多机系统。

---

## 环境配置指南

> **为什么这一节放在最前面**：工程实践类文档的第一杀手不是算法难，而是**环境装不上**。多机系统比单机更脆——你要同时让 ROS 2、DDS、仿真器、训练框架、多个命名空间下的节点协同工作，任何一个版本错配都会让你卡在"节点起不来"而不是"算法不对"。本节先把地基夯实，后面才谈得上搭楼。

### 系统要求

Mini-MultiBot 的目标是在一台**开发工作站**上完成全部仿真开发，真机部署时再分发到各机器人的板载计算机。开发工作站的推荐配置：

| 部件 | 最低配置 | 推荐配置 | 说明 |
|------|---------|---------|------|
| **操作系统** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | ROS 2 Humble/Iron 的 Tier-1 平台；24.04 对应 Jazzy，本章以 Humble 为基准 |
| **CPU** | 4 核 | 8 核以上 | 多机仿真要同时跑 N 份控制器 + 仿真器，核数直接决定能仿多少机 |
| **内存** | 16 GB | 32 GB | 每个 Gazebo 机器人实例约 0.5-1 GB；8 机仿真 + 训练轻松吃满 16 GB |
| **GPU** | 可选 | NVIDIA RTX 3060+ (8GB+) | MARL 训练（IsaacLab/MQE）必需；纯 MPC 路线可无 GPU |
| **磁盘** | 30 GB 空闲 | 50 GB+ SSD | ROS 2 + Gazebo + IsaacLab + 数据集 |

> **本质洞察**：多机仿真的瓶颈不是"算法算得慢"，而是 **CPU 核数 × 仿真器进程数**。一个朴素的认知误区是以为"加机器人只是多几个话题"——实际上每加一个 Gazebo 机器人，物理引擎要多算一份碰撞与动力学，每加一个控制器节点要多占一个 CPU 调度时隙。这就是为什么本章后面会反复强调"先在 2 机上跑通，再逐步加到 N 机"——不是算法限制，是计算资源的硬约束。

### 版本兼容表

下面这张表是本章所有代码的测试基准。**强烈建议精确锁定到推荐版本**——多机系统对版本错配的容忍度远低于单机，一个 DDS 实现的小版本差异就可能导致跨命名空间的话题发现失败。

| 组件 | 推荐版本 | 最低版本 | 最高测试版本 | 备注 |
|------|---------|---------|------------|------|
| Ubuntu | 22.04 | 20.04 | 24.04 | 20.04→Foxy(EOL)，24.04→Jazzy（API 微调） |
| ROS 2 | Humble | Foxy | Iron | 本章基准 Humble；Iron 完全兼容，仅 launch 语法有细微差异 |
| Python | 3.10 | 3.8 | 3.11 | 跟随 ROS 2 发行版；3.12 与 Humble 的 rclpy 未充分测试 |
| Gazebo | Fortress (Ignition) | Classic 11 | Harmonic | 本章用 Gazebo Fortress + `ros_gz` 桥；Classic 11 需改用 `gazebo_ros` |
| Fast DDS | 2.6.x | 2.3 | 2.10 | ROS 2 Humble 默认 RMW；本章的 Discovery Server 配置依赖 ≥2.6 |
| Nav2 | 1.1.x | 1.0 | 1.2 | 仅"地面机器人导航"扩展任务用到；核心任务不依赖 |
| PyTorch | 2.1.0 | 2.0.0 | 2.2.0 | MARL 路线用；须与 CUDA 版本匹配 |
| CUDA | 12.1 | 11.8 | 12.4 | 跟随 PyTorch 与 GPU 驱动 |
| Crazyswarm2 | 1.0a1 | — | 1.0a1 | UAV 蜂群仿真/实机；跟随 `imrclab/crazyswarm2` main 分支 |
| NumPy | 1.24 | 1.21 | 1.26 | 2.0 与部分 ROS 2 Humble 包有 ABI 冲突，**不要装 NumPy 2.x** |

> **配置陷阱（提前预警，§13.1 详述）**：上表里最容易踩的雷是 **NumPy 2.x**。2024 年 6 月 NumPy 2.0 发布后，大量基于 Humble（编译期链接 NumPy 1.x ABI）的 ROS 2 包会在 `import` 时直接崩溃，报 `_ARRAY_API not found`。本章所有 `pip install` 都应加 `"numpy<2"` 约束。

### 安装步骤

下面分三个"安装层"，每一层装完都给一个**验证命令**——装一层、验一层，不要一口气装完再一起调试。

**第一层：ROS 2 Humble 基础栈**

```bash
# 1. 设置 locale（ROS 2 要求 UTF-8）
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. 添加 ROS 2 apt 源
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS 2 Humble Desktop（含 RViz、demo 节点）
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# 验证：source 后能看到版本号
source /opt/ros/humble/setup.bash
ros2 --version       # 期望输出：ros2 cli ... (Humble)
```

**第二层：仿真器与多机相关包**

```bash
# Gazebo Fortress + ROS-Gazebo 桥
sudo apt install -y ros-humble-ros-gz

# 多机器人会用到的包：导航、TF、命名空间工具、域桥
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  ros-humble-domain-bridge \
  ros-humble-tf2-tools \
  ros-humble-rmw-fastrtps-cpp

# 验证：能启动一个空 Gazebo 世界
ign gazebo --version  # Fortress 对应 Ignition Gazebo 6.x
```

**第三层（可选，MARL 路线）：训练框架**

```bash
# 建议用 conda/venv 隔离 Python 训练环境，避免污染系统 ROS 2 的 Python
python3 -m venv ~/mmb_venv && source ~/mmb_venv/bin/activate
pip install --upgrade pip

# 注意 numpy<2 约束！
pip install "numpy<2" torch==2.1.0 gymnasium==0.29.1 tensorboard

# 验证 CUDA 可用（有 GPU 时）
python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
```

> **配置陷阱**：第三层的 Python 训练环境**绝对不要**和系统 ROS 2 的 Python 环境混用。`rclpy` 是按系统 Python 3.10 编译的，如果你在一个装了不同 NumPy/Python 的 venv 里 `source /opt/ros/humble/setup.bash`，会出现"训练脚本里 import torch 正常，但 import rclpy 段错误"的诡异现象。本章的做法是：**ROS 2 节点用系统 Python，纯训练脚本用独立 venv，二者通过文件（保存的策略权重）或独立进程通信**——这个边界后面 §13.6 会反复出现。

### 创建工作空间

最后，建好 Mini-MultiBot 的 colcon 工作空间——本章所有代码都长在这里：

```bash
mkdir -p ~/mini_multibot_ws/src
cd ~/mini_multibot_ws/src
# 后面每个 §13.x 会往这里加包

# 工作空间根目录建一个基础环境脚本，一次性 source 全部
cat > ~/mini_multibot_ws/mmb_env.sh <<'EOF'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42                 # 给整个项目一个独立域，避免和别人串台
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -f ~/mini_multibot_ws/install/setup.bash ] && source ~/mini_multibot_ws/install/setup.bash
EOF
echo "source ~/mini_multibot_ws/mmb_env.sh" >> ~/.bashrc
```

> **为什么设 `ROS_DOMAIN_ID=42`**：ROS 2 用 DDS 的 domain ID 做**第一层逻辑隔离**——不同 domain ID 的节点彼此完全不可见。在实验室共享网络里，如果你和同学都用默认 domain 0，你们的话题会互相串扰（你的 `/go2_0/cmd_vel` 会被对方的 RViz 订阅到）。给项目一个独立 domain 是多机开发的第一条卫生习惯。这个机制 §13.2 会深入——它正是从"单机随便跑"过渡到"多机要管理通信"的第一道门槛。

---

## Quick Start（5 分钟跑通最小示例）

> 在深入任何理论前，先让你看到"两个机器人在仿真里各自动起来"——建立信心，也验证环境装对了。这个最小示例只用到 ROS 2 的命名空间机制，不依赖本章后面任何代码。

```bash
# 终端 1：启动两个独立命名空间下的 demo talker
source ~/mini_multibot_ws/mmb_env.sh
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/robot_0 -r chatter:=/robot_0/chatter &
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/robot_1 -r chatter:=/robot_1/chatter &

# 终端 2：查看话题树——你应该看到两套独立的话题
source ~/mini_multibot_ws/mmb_env.sh
ros2 topic list
# 期望输出包含：
#   /robot_0/chatter
#   /robot_1/chatter

# 终端 2：只听 robot_0 的话题
ros2 topic echo /robot_0/chatter
# 期望：每秒打印一条 "Hello World: N"，且 N 与 robot_1 独立计数
```

**成功标志**：`ros2 topic list` 里 `robot_0` 和 `robot_1` 各有一套独立话题，互不干扰。这就是多机系统最底层的原理——**用命名空间把 N 份相同的节点图隔离开**。如果你看到了两套独立话题，恭喜，环境就绪，可以正式开始了。

如果这一步失败（只看到一套话题、或话题串了），99% 是 domain ID 没设对或两个终端的环境不一致——回到章末"🔧 故障排查手册"的"故障 1"。

---

## 前置自测

> 📋 **答不出 ≥ 2 题** → 按括号里的指引回对应章节补齐。本章是综合实战，会**大量复用**前十二章的概念但不重新教。欠的账会在搭系统时变成"代码看不懂"。

1. **（接第 1 章）** 多机系统的三大架构（集中式 / 分布式 / 去中心化）各自的信息流是什么？为什么"集中式 $O(N^3)$ 会爆炸"？通信图的代数连通性 $\lambda_2$ 在系统里扮演什么角色？
   （答不出 → 回第 1 章「多机系统全景」§1.1-1.3）

2. **（接第 2 章）** 共识协议 $\dot x = -Lx$ 收敛到什么？ADMM 的三步迭代（x 更新、z 更新、对偶更新）分别在做什么？罚参数 $\rho$ 调大调小各有什么影响？
   （答不出 → 回第 2 章「共识算法与分布式优化」§2.2、§2.4）

3. **（接第 3 章）** 任务分配的指派问题用什么算法最优求解（复杂度多少）？MAPF 里的"顶点冲突"和"边冲突"分别指什么？为什么多机路径规划不是 N 个独立的单体 A\*？
   （答不出 → 回第 3 章「任务分配与路径规划」§3.1、§3.3）

4. **（接第 4/6 章）** 分布式 MPC 编队里，每个机器人本地解什么、邻居间交换什么？异构系统的"能力互补矩阵"刻画了什么？为什么异构系统不是同构系统的简单拼接？
   （答不出 → 回第 4 章「分布式 MPC 多足编队」§4.2、第 6 章「异构多机协同」§6.1）

5. **（接第 10 章）** MARL 的 CTDE（集中训练分布执行）范式是什么意思？为什么多智能体一起学习会破坏单智能体 RL 的收敛保证（非平稳性问题）？MAPPO 的"集中式 Critic"在训练和部署时分别怎么用？
   （答不出 → 回第 10 章「MARL 基础」§10.1、§10.3）

**参考答案要点**（先自己答，再对照）：

1. 集中式：单一节点收集全局信息、统一决策、下发指令（单点故障、$O(N^3)$ 联合优化随机器人数立方增长）；分布式：每个机器人本地决策，靠邻居通信协调（无单点故障，通信轮数正比于图直径）；去中心化：完全对等，无任何全局信息。$\lambda_2$（Fiedler 值）衡量图的连通强度，决定共识/分布式优化的收敛速率——$\lambda_2$ 越大，协调越快。

2. 共识收敛到所有节点状态的**初始平均值**（当权重双随机时）。ADMM：x 更新 = 各 agent 本地解自己的子问题；z 更新 = 全局变量协调（常是邻居解的平均）；对偶更新 = 拉格朗日乘子按约束违反量上升。$\rho$ 大 → 强制一致性快但本地子问题被"拽偏"、易振荡；$\rho$ 小 → 本地解更自由但收敛慢。

3. 匈牙利（Kuhn-Munkres）算法，$O(n^3)$。顶点冲突 = 两机器人同一时刻占同一格；边冲突（交换冲突）= 两机器人同一时刻交换相邻格。多机路径规划不能解耦成 N 个独立 A\*，因为机器人会在共享空间相撞——必须显式处理时空约束（CBS）或加协调机制（优先级规划/PIBT）。

4. 每个机器人本地解一个缩小版 MPC（自己的状态+预测时域），邻居间交换**预测轨迹**（用于一致性约束/耦合力协调）。能力互补矩阵刻画"每个机型擅长哪类子任务"（如 UAV 看得远、四足搬得重、机械臂抓得准）。异构不是简单拼接，因为任务分配、编队几何、观测/动作空间维度都因机型不同而需重写。

5. CTDE：训练时 Critic 可见全局状态（开天眼），执行时每个 Actor 只用自己的局部观测（各自为政）。非平稳性：从单个智能体视角，其他智能体的策略在不断变化，使环境的转移分布随时间漂移，违反了单智能体 RL 假设的马尔可夫平稳性。MAPPO 的集中式 Critic 训练时输入全局状态算优势函数，部署时直接丢弃，只留各自的 Actor。

---

## 本章目标

学完本章后，你应该能够：

1. **从零搭建**一个支持 N 个机器人的 ROS 2 多机工作空间，理解命名空间隔离、多机 TF 树、参数分发的完整机制，并能解释"为什么多机系统要这样组织目录与 launch"。
2. **配置多机通信层**：在默认 DDS（多播发现）、Discovery Server、Domain Bridge 三种方案间做选型，量化各自的带宽/扩展性代价，避免"加到 10 机网络就瘫"的发现风暴。
3. **实现分布式协调内核**：把第 2 章的共识协议、第 3 章的任务分配、第 4 章的 ADMM 编队协调，落成可在 ROS 2 上跨节点运行的代码模块，并接好它们之间的数据流。
4. **集成任务分配 + 编队 + 避障三层**：搭起"先分工（谁去哪）→ 再编队（保持队形）→ 局部避障（ORCA 兜底）"的完整运动栈，并理解三层之间的接口与失效边界。
5. **在两套仿真器上验证**：用 Gazebo（地面异构机）和 Crazyswarm2（空中蜂群）分别跑通协同任务，理解两套仿真器的抽象层级差异与各自适用场景。
6. **打通 MARL 与规控的混合**：把第 10-12 章训练出的 MAPPO 策略接进系统，让"学习的高层决策"驱动"传统的底层控制"，并加 CBF 安全滤波兜底。
7. **完成真机部署与系统级调试**：理解仿真到真机的迁移路径（时间同步、坐标系标定、网络配置），掌握多机系统特有的故障排查方法（话题级、节点级、网络级三层定位）。

---

### 本章知识导航

本章的知识结构不是"一棵新知识树"，而是一张**装配图**——把前十二章的零件按数据流焊接起来。整个系统的骨架是一条从"通信"到"决策"到"运动"到"部署"的流水线：

```
                    Mini-MultiBot 系统装配图
                            │
  ┌─────────────────────────┼─────────────────────────┐
  │                         │                         │
通信层 (§13.2)          决策层 (§13.3-13.4)         运动层 (§13.5)
ROS2 多机/DDS          任务分配+共识+ADMM            编队+ORCA避障
  │                         │                         │
  └──────────┬──────────────┴──────────────┬──────────┘
             │                            │
        仿真验证 (§13.7)              MARL 混合 (§13.6)
     Gazebo / Crazyswarm2          MAPPO 策略 + CBF 滤波
             │                            │
             └────────────┬───────────────┘
                          │
                  实机部署+调试 (§13.8)
                时间同步/标定/三层故障定位
```

| 小节 | 主题 | 难度 | 一句话 |
|------|------|------|--------|
| §13.1 | 项目骨架与工程组织 | ⭐⭐ | 目录结构、colcon、配置即代码——多机项目的"地基浇筑" |
| §13.2 | ROS 2 多机通信层 | ⭐⭐⭐ | 命名空间隔离、多机 TF、DDS 三种发现方案选型 |
| §13.3 | 分布式协调内核 | ⭐⭐⭐⭐ | 共识 + 任务分配落成 ROS 2 节点，跨进程数据流 |
| §13.4 | 任务分配集成 | ⭐⭐⭐ | CBBA/匈牙利接进系统，分配结果驱动运动层 |
| §13.5 | 编队与避障集成 | ⭐⭐⭐ | 三层运动栈：编队几何 → ADMM 协调 → ORCA 兜底 |
| §13.6 | MARL 与规控混合 | ⭐⭐⭐⭐ | MAPPO 策略接入，CBF 安全滤波，学习层↔控制层桥接 |
| §13.7 | 仿真验证 | ⭐⭐⭐ | Gazebo 异构机 + Crazyswarm2 蜂群，两套仿真器对照 |
| §13.8 | 实机部署与系统调试 | ⭐⭐⭐⭐ | Sim2Real 迁移、时间同步、三层故障定位手册 |

**两条实战路线**（按你的方向选）：

- **地面异构线**（四足/轮式/机械臂）：§13.1→§13.2→§13.3→§13.4→§13.5→§13.7(Gazebo)→§13.8，重点是分布式 MPC 编队 + 任务调度。
- **空中蜂群线**（多旋翼）：§13.1→§13.2→§13.5(编队)→§13.7(Crazyswarm2)→§13.8，重点是大规模编队 + 实机蜂群飞行。

无论哪条线，§13.1（骨架）和 §13.2（通信）都是必读地基——它们决定了你的系统"能不能长大"。§13.3、§13.6、§13.8 是三个 ⭐⭐⭐⭐ 硬核小节，值得花最多时间。

---

### 前置知识桥接

本章是十二章的总集成，这里把最关键的前置点重新激活——你不必翻回去就能跟上：

- **回顾第 1 章：三大架构与通信图**。第 1 章我们把任意多机系统拆成"什么架构 × 什么交互 × 什么信息流"。本章 §13.2 把这个**抽象架构**落成**具体的 ROS 2 拓扑**——集中式架构对应"一个 coordinator 节点 + N 个 agent 节点"，分布式架构对应"N 个对等 agent 节点 + 邻居话题连接"。第 1 章的 $\lambda_2$ 在这里变成"邻居话题的连接稠密度"——你会在 launch 文件里亲手决定每个 agent 订阅哪些邻居。
- **回顾第 2 章：共识与 ADMM**。第 2 章证明了共识 $\dot x=-Lx$ 收敛到平均、ADMM 三步迭代切分联合优化。本章 §13.3 把这两套**数学迭代**落成**ROS 2 的话题往返**——共识的"邻居状态平均"变成"订阅邻居状态话题 → 加权求和 → 发布更新"，ADMM 的"协调步"变成"coordinator 收集各 agent 的局部解 → 算平均 → 广播回去"。你会看到教科书上的迭代公式如何变成定时器回调里的几行代码。
- **回顾第 3 章：任务分配与 MAPF**。第 3 章给了匈牙利（集中最优）、CBBA（分布式 50% 界）、CBS（最优 MAPF）。本章 §13.4 直接**复用第 3 章的 `planning/` 模块**——把分配算法包成一个 ROS 2 服务，输入"机器人位置 + 任务列表"，输出"谁去哪"，再把这个分配结果喂给 §13.5 的编队层。
- **回顾第 4 章：分布式 MPC 编队**。第 4 章把 ADMM 用到了真实的多足联合优化（24 维、20 步时域、摩擦锥）。本章 §13.5 把它**简化并系统化**——你不必重新推 SRB 模型，而是把第 4 章的 `admm_coordinator` 接进本章的通信层，让它在 ROS 2 上跨节点跑起来。
- **回顾第 6 章：异构协同**。第 6 章拆掉了"机器人都一样"的假设。本章的双任务之一（Go2 + UAV 联合巡检）就是异构系统的实战——你会处理"四足和无人机的观测/动作空间维度不一致"这个第 6 章讲过的核心困难。
- **回顾第 10-12 章：MARL 及混合**。第 10 章给了 MAPPO 的 CTDE 范式，第 12 章给了 MARL+规控混合。本章 §13.6 把训练好的策略**部署进系统**——这是"训练"（第 10-12 章）和"部署"（本章）的接口，你会亲手处理"Python venv 里训练的策略如何被系统 ROS 2 节点加载"这个工程边界。

### 如果跳过本章会怎样

- **场景一（"每个零件都会，但拼不起来"）**：你把前十二章的算法都实现过——共识能收敛、CBBA 能分配、MPC 编队能跑、MAPPO 能训。但当老板说"给我一个能演示的多机系统"时，你愣住了：这些算法分散在不同的 Python 脚本和 C++ demo 里，没有统一的通信层把它们串起来，没有统一的目录组织，没有一键启动。本章就是教你这个"最后一公里"——把零件焊成产品。这一公里看着简单，实际是工程能力和算法能力的分水岭。
- **场景二（"2 机能跑，5 机就瘫"）**：你天真地把单机代码复制 N 份、改改话题名就上了。2 机时一切正常，加到 8 机时网络开始卡顿、节点随机掉线、TF 报 "lookup would require extrapolation"。你不知道这是 DDS 多播发现风暴 + 时间不同步导致的。§13.2 和 §13.8 会告诉你：多机系统的扩展性问题**不在算法层而在系统层**——发现机制、域隔离、时间同步这些"脏活"才是决定系统能不能长大的关键。

### 预计学习时间

| 模式 | 时长 | 适合 |
|------|------|------|
| 精读+实战（跟着搭完整系统，跑通两个任务，做完里程碑 checklist） | 30-40 小时（2 周） | 第一次做多机综合项目、要交 capstone 的工程师 |
| 速读（读懂系统架构与各层接口，跑通 Quick Start 和一个任务） | 8-10 小时 | 已有多机背景，想快速建立"系统级"视角 |
| 速查（架构图 + 通信选型表 + launch 模板 + 故障排查） | 60-90 分钟 | 搭自己的多机系统时回查工程细节 |

---

## §13.1 项目骨架与工程组织 ⭐⭐

> **这一节解决什么问题**：在写任何算法前，先回答一个被严重低估的问题——**N 个机器人的代码该怎么组织？** 单机项目你可以把所有东西塞进一个 package；多机项目如果还这么干，你会在第三天就被"改一个 agent 的逻辑要动到所有 agent"逼疯。这一节给出 Mini-MultiBot 的目录骨架，并解释每一个设计决策背后的"为什么"。

### 模块功能与在系统中的位置

骨架是整个系统的"承重墙"——它决定了后面每一层代码挂在哪里、彼此怎么依赖。一个好的多机骨架要满足三条：**（1）单机逻辑与多机协调解耦**（改单个 agent 不影响协调层）；**（2）配置与代码分离**（换机器人数量/拓扑不改代码）；**（3）仿真与实机共享同一套控制代码**（只换底层接口）。

### 动机：为什么不能"复制 N 份单机代码"

最朴素的多机做法是：写好单机控制器，复制 N 份，每份改个话题名。这在 2 机时能用，但有三个致命问题：

| 问题 | 后果 | 根因 |
|------|------|------|
| **代码冗余** | 改一处 bug 要改 N 个文件 | 逻辑没有抽象成"参数化的单实例" |
| **拓扑硬编码** | 换 3 机→5 机要重写 launch | 邻居关系写死在代码里 |
| **协调逻辑无处安放** | ADMM/共识的"全局协调步"不知道写哪 | 没有区分"agent 本地逻辑"和"系统协调逻辑" |

> **本质洞察**：多机系统的代码组织，本质是要回答"哪些是**每个机器人各跑一份**的（agent-local），哪些是**全系统只跑一份**的（system-level），哪些是**纯描述、不含逻辑**的（configuration）"。把这三类分清楚，目录结构自然就出来了。这正是下面骨架的设计主线——`controllers/` 是 agent-local，`coordination/` 是 system-level，`config/` 是纯配置。

### 目录骨架详解

下面是 Mini-MultiBot 的完整目录结构。它在第 13 章原始骨架（双 Go2 搬运 + Go2-UAV 巡检）的基础上做了**工程化重组**——把"按文件类型分"改成"按系统角色分"，这样每一层都能独立演进：

```
mini_multibot_ws/src/mini_multibot/
├── mini_multibot_bringup/          # 【系统级】一键启动：把所有节点拼成系统
│   ├── launch/
│   │   ├── bringup_sim.launch.py       # 仿真总启动（Gazebo + N agents + coordinator）
│   │   ├── bringup_real.launch.py      # 实机总启动（同结构，换底层接口）
│   │   ├── spawn_robots.launch.py      # 往仿真器里 spawn N 个机器人
│   │   └── single_agent.launch.py      # 单 agent 子启动（被总启动循环调用）
│   ├── config/
│   │   ├── team_2_go2.yaml              # 队伍描述：2 个 Go2 的拓扑/初始位姿
│   │   ├── team_go2_uav.yaml            # 队伍描述：Go2 + UAV 异构
│   │   └── dds_discovery_server.xml     # DDS 发现服务器配置（§13.2）
│   └── package.xml / CMakeLists.txt
│
├── mini_multibot_msgs/             # 【接口层】自定义消息/服务——系统的"通用语言"
│   ├── msg/
│   │   ├── AgentState.msg               # agent 状态（位姿+速度+健康度）
│   │   ├── Assignment.msg               # 任务分配结果（agent_id → task_id）
│   │   └── ConsensusMsg.msg             # 共识/ADMM 交换的状态向量
│   ├── srv/
│   │   └── AllocateTasks.srv            # 任务分配服务请求/响应
│   └── package.xml / CMakeLists.txt
│
├── mmb_agent/                      # 【agent-local】每个机器人各跑一份的逻辑
│   ├── mmb_agent/
│   │   ├── agent_node.py                # agent 主节点（状态机+生命周期）
│   │   ├── local_controller.py          # 单 agent 控制器基类（被各机型继承）
│   │   ├── go2_controller.py            # Go2 四足控制器（继承 local_controller）
│   │   ├── uav_controller.py            # UAV 控制器（继承 local_controller）
│   │   └── neighbor_comm.py             # 邻居通信封装（订阅邻居状态、发布自身）
│   └── package.xml / setup.py
│
├── mmb_coordination/               # 【system-level】全系统协调逻辑
│   ├── mmb_coordination/
│   │   ├── consensus_core.py            # 共识协议核心（复用第 2 章）
│   │   ├── admm_coordinator.py          # ADMM 分布式协调器（复用第 4 章）
│   │   └── task_allocator.py            # 任务分配服务（复用第 3 章 planning/）
│   └── package.xml / setup.py
│
├── mmb_planning/                   # 【复用第 3 章】任务分配 + MAPF（已有模块直接挪用）
│   └── ...（hungarian.py / cbba.py / cbs.py，本章不重写，import 复用）
│
├── mmb_formation/                  # 【运动层】编队几何 + 局部避障（§13.5）
│   ├── mmb_formation/
│   │   ├── formation_keeper.py          # 编队几何控制（leader-follower / virtual structure）
│   │   └── orca_avoidance.py            # ORCA 局部避障兜底
│   └── package.xml / setup.py
│
├── mmb_learning/                   # 【MARL 路线】策略部署（§13.6，训练脚本在独立 venv）
│   ├── mmb_learning/
│   │   ├── policy_deploy_node.py        # 把训练好的 MAPPO 策略包成 ROS 2 节点
│   │   └── cbf_safety_filter.py         # CBF 安全滤波层
│   └── package.xml / setup.py
│
├── mmb_sim/                        # 【仿真资产】机器人模型、世界文件
│   ├── models/ (go2/ uav/ ...)          # URDF/SDF
│   └── worlds/ (warehouse.sdf ...)      # Gazebo 世界
│
└── mmb_eval/                       # 【评估】benchmark + 可视化（对应评分标准）
    ├── benchmark.py                     # 成功率/跟踪误差/通信量统计
    └── visualize.py                     # radar chart + 多机轨迹可视化
```

把这套结构和第 13 章原始骨架对照，核心变化是**从"按文件类型平铺"升级为"按系统角色分包"**。原始骨架的 `controllers/distributed_mpc/`、`communication/`、`evaluation/` 是按"功能模块"分的——这在小项目够用，但当系统长大，你会发现"一个 agent 的逻辑"散落在 `controllers/`、`communication/` 多个地方。重组后，**改单个 agent 只动 `mmb_agent/`，改协调策略只动 `mmb_coordination/`，换队伍配置只动 `bringup/config/`**——这就是"关注点分离"在多机项目里的具体落地。

### 配置即代码：用 YAML 描述队伍

骨架里最关键的设计是把"队伍长什么样"完全放进 YAML，代码里**不写死任何机器人数量或拓扑**。看 `team_2_go2.yaml`：

```yaml
# config/team_2_go2.yaml —— 描述"双 Go2 搬运"这支队伍
team:
  name: "dual_go2_carry"
  architecture: "distributed"        # centralized / distributed / decentralized

agents:
  - id: 0
    type: "go2"                      # 决定加载哪个 controller 子类
    namespace: "go2_0"
    initial_pose: {x: 0.0, y: 0.5, z: 0.3, yaw: 0.0}
    neighbors: [1]                   # 通信拓扑：agent 0 的邻居是 agent 1
  - id: 1
    type: "go2"
    namespace: "go2_1"
    initial_pose: {x: 0.0, y: -0.5, z: 0.3, yaw: 0.0}
    neighbors: [0]

coordination:
  consensus_rate: 50.0               # 共识/协调步频率 (Hz)
  admm_rho: 1.0                      # ADMM 罚参数（第 2 章的 ρ）
  admm_iterations: 3                 # 每控制周期的 ADMM 迭代轮数
```

每个字段的含义、推荐值、可调范围如下表——这是工程实践类文档的核心要求，**配置项要当知识点讲**：

| 字段 | 含义 | 推荐值 | 可调范围 | 与其他字段的耦合 |
|------|------|--------|---------|----------------|
| `architecture` | 决定 coordinator 节点的行为模式 | `distributed` | 三选一 | `centralized` 时 coordinator 解全局问题；`distributed` 时只做协调步 |
| `neighbors` | 通信拓扑（第 1 章的图 $G$ 的边） | 见拓扑设计 | 任意有向图 | 直接决定 §13.3 共识的收敛速率（$\lambda_2$）；邻居越多越快但通信量越大 |
| `consensus_rate` | 协调步频率 | 50 Hz | 10-100 Hz | 必须 ≤ 控制频率；过高则网络跟不上，过低则协调滞后 |
| `admm_rho` | ADMM 罚参数 | 1.0 | 0.1-10 | 大→强一致快但易振荡，小→收敛慢（第 2 章 §2.4 详述） |
| `admm_iterations` | 每周期 ADMM 轮数 | 3 | 1-10 | 与 `consensus_rate` 乘积是实际通信负载；实时系统通常只能 2-5 轮 |

> **本质洞察**：把拓扑写进 YAML 而非代码，不只是"代码整洁"——它让你能**用同一份代码做实验**：想对比"环形拓扑 vs 全连接拓扑对收敛速率的影响"（第 2 章 $\lambda_2$ 的实验），你只需改 `neighbors` 字段重跑，不用碰一行 Python。配置即代码是多机系统做可复现实验的前提。

### 代码走读：用循环 launch N 个 agent

骨架的灵魂是 launch 文件——它读 YAML，**循环**起 N 个 agent。这是"参数化单实例"思想的直接体现。下面是 `bringup_sim.launch.py` 的核心（完整版在仓库，这里讲关键逻辑）：

```python
# mini_multibot_bringup/launch/bringup_sim.launch.py
import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def load_team_config(team_file):
    """读取队伍 YAML——所有拓扑信息的唯一来源。"""
    cfg_path = os.path.join(
        get_package_share_directory('mini_multibot_bringup'),
        'config', team_file)
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # 1. 加载队伍描述（这里硬选 2-Go2，实际可用 LaunchArgument 切换）
    team = load_team_config('team_2_go2.yaml')
    agents = team['agents']
    coord_cfg = team['coordination']

    actions = []

    # 2. 循环为每个 agent 起一组节点（关键：N 由 YAML 决定，不写死）
    for ag in agents:
        agent_group = GroupAction([
            PushRosNamespace(ag['namespace']),       # 命名空间隔离（§13.2 核心）
            Node(
                package='mmb_agent',
                executable='agent_node',
                name='agent',
                parameters=[{
                    'agent_id': ag['id'],
                    'agent_type': ag['type'],        # 决定加载哪个 controller 子类
                    'neighbors': ag['neighbors'],    # 邻居列表 → neighbor_comm 用
                    'consensus_rate': coord_cfg['consensus_rate'],
                    'admm_rho': coord_cfg['admm_rho'],
                }],
                # 邻居话题 remapping：把 /go2_1/state 映射成本地的 neighbor_0
                remappings=_neighbor_remaps(ag),
            ),
        ])
        actions.append(agent_group)

    # 3. 起一个全系统协调器（system-level，只一份）
    coordinator = Node(
        package='mmb_coordination',
        executable='admm_coordinator',
        name='coordinator',
        parameters=[{
            'n_agents': len(agents),
            'admm_rho': coord_cfg['admm_rho'],
            'admm_iterations': coord_cfg['admm_iterations'],
        }],
    )
    actions.append(coordinator)

    # 4. 把 Gazebo + spawn 机器人的子 launch 也包进来
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mini_multibot_bringup'),
            'launch', 'spawn_robots.launch.py')),
        launch_arguments={'team': 'team_2_go2.yaml'}.items(),
    ))

    return LaunchDescription(actions)


def _neighbor_remaps(agent):
    """为一个 agent 生成邻居话题重映射：邻居 j → neighbor_<k>/state。"""
    remaps = []
    for k, nb_id in enumerate(agent['neighbors']):
        remaps.append((f'neighbor_{k}/state', f'/go2_{nb_id}/state'))
    return remaps
```

逐段解释这段 launch 在做什么——这正是工程实践类文档要求的"代码走读"：

- **`load_team_config`**：所有"系统有几个机器人、谁连谁"的信息只从 YAML 来。这是配置即代码的入口。改队伍 = 改 YAML 文件名（或用 `LaunchArgument` 在命令行传），代码零修改。
- **第 2 步的循环**：这是多机 launch 的核心套路——**用 Python 循环动态生成节点**。每个 agent 被 `PushRosNamespace` 包进自己的命名空间，于是 `mmb_agent` 内部所有话题（如 `cmd_vel`、`state`）都自动加上 `/go2_0/` 前缀。这就是 Quick Start 里"两套独立话题"的原理在真实系统里的应用。
- **`remappings` + `_neighbor_remaps`**：这是"邻居通信"的接线。agent 0 的邻居是 agent 1，于是把 agent 0 内部订阅的 `neighbor_0/state` 重映射到全局话题 `/go2_1/state`。**命名空间负责隔离自身，remapping 负责连接邻居**——这两个机制配合，就实现了第 1 章通信图 $G$ 的边。
- **第 3 步的 coordinator**：注意它**不在任何 agent 命名空间内**——它是 system-level 节点，全系统只一份，监听所有 agent 的状态、跑 ADMM 协调步、广播回去。这对应分布式架构里那个"轻量协调者"（不解全局问题，只做信息聚合）。

### 验证：先让骨架"空转"

搭骨架最忌一口气写完才跑。正确做法是**让骨架先空转**——agent 节点先不放真实控制逻辑，只发布心跳和假状态，验证"N 个节点能起来、命名空间对、邻居话题通"：

```bash
# 构建
cd ~/mini_multibot_ws && colcon build --packages-select \
  mini_multibot_msgs mmb_agent mmb_coordination mini_multibot_bringup
source install/setup.bash

# 起骨架（此时 agent_node 内部是 stub：只发心跳+假位姿）
ros2 launch mini_multibot_bringup bringup_sim.launch.py

# 另开终端验证：应看到两套命名空间下的节点和话题
ros2 node list
#   /go2_0/agent
#   /go2_1/agent
#   /coordinator
ros2 topic list | grep state
#   /go2_0/state
#   /go2_1/state

# 验证邻居连接：agent 0 真的在听 agent 1 吗？
ros2 topic info /go2_1/state
#   Publisher count: 1   ← go2_1 的 agent 在发
#   Subscription count: 1 ← go2_0 的 agent（通过 remapping）在听 ✓
```

**成功标志**：`Subscription count` ≥ 1，证明邻居话题真的连上了。这一步通过，骨架就立住了——后面每一节都是往这个骨架里"填肉"。

### ⚠️ 常见陷阱

```
🔧 配置陷阱：在 launch 循环里忘记 PushRosNamespace，所有 agent 话题撞车
   错误做法：循环起 N 个 Node 但不包 PushRosNamespace，指望靠 name 区分
   现象：N 个 agent 都发到同一个 /state 话题，状态互相覆盖，
        ros2 topic echo /state 看到的数据在 N 个机器人间乱跳
   根本原因：ROS 2 的 name 只改节点名，不改话题名。话题隔离必须靠
            命名空间（namespace）或显式 remapping，而非节点名
   正确做法：每个 agent 用 PushRosNamespace(ag['namespace']) 包起来，
            让其内部所有相对话题自动加前缀。验证方法：ros2 topic list
            必须看到 /go2_0/state 和 /go2_1/state 两个独立话题
```

```
🔧 配置陷阱：邻居 remapping 用相对话题名，结果带上了自己的命名空间前缀
   错误做法：remappings=[('neighbor_0/state', 'go2_1/state')]（无前导 /）
   现象：agent 0（在 /go2_0 命名空间）的订阅被解析成 /go2_0/go2_1/state，
        而 go2_1 实际发布在 /go2_1/state，两者对不上，邻居通信静默失败
   根本原因：相对话题名会被当前命名空间"吃掉"——在 /go2_0 下，相对名
            go2_1/state 展开为 /go2_0/go2_1/state
   正确做法：邻居话题用绝对名（前导 /）：('neighbor_0/state', '/go2_1/state')。
            跨命名空间引用必须用绝对路径。检查方法：ros2 topic info 看
            订阅计数是否 >0
```

```
💡 概念误区：以为"把单机代码复制 N 份"就是多机系统
   新手想法："我有了能跑的单机控制器，复制 N 份改改话题名不就成多机了？"
   实际上：复制 N 份只解决了"N 个机器人各自动"，没有解决"它们如何协调"——
          没有协调层，N 个机器人只是 N 个互不知道对方的孤岛，不是"系统"。
          真正的多机系统 = N 个参数化的 agent 实例 + 一个协调机制（共识/ADMM/分配）
   正确理解：多机系统的价值在"协调"而非"并行"。骨架必须显式区分
            agent-local（各跑一份）和 system-level（协调逻辑），后者才是
            多机的灵魂。这也是为什么本章骨架专门有 mmb_coordination 包
```

### 练习

1. **【配置练习】** 把 `team_2_go2.yaml` 扩展成 `team_4_go2.yaml`（4 个 Go2），拓扑选"环形"（agent i 的邻居是 i-1 和 i+1，模 4）。不改任何 Python 代码，只改 YAML，重跑 `bringup_sim.launch.py`，用 `ros2 node list` 确认 4 个 agent 都起来了。验收标准：`/go2_0` 到 `/go2_3` 四套命名空间齐全，且 `ros2 topic info /go2_1/state` 显示订阅计数为 2（被 agent 0 和 agent 2 监听）。

2. **【管线搭建】** 给骨架加一个"健康监控"节点（system-level，放进 `mmb_coordination`）：订阅所有 agent 的 `/go2_i/state`，若某 agent 超过 1 秒没发状态，打印警告 `[WARN] agent i lost`。提示：用 `MultiThreadedExecutor` 或定时器检查每个 agent 的最后接收时间戳。这个节点后面 §13.8 故障定位会用到。

3. **【综合思考】** 对照第 1 章的三大架构，回答：如果把 `architecture` 改成 `centralized`，骨架里哪些节点的职责要变？coordinator 节点应该从"只做协调步"变成"解什么"？画出集中式与分布式两种架构下的节点-话题图（节点画方框，话题画箭头），标出二者的关键区别。（这是一道跨章综合题，综合第 1 章架构 + 本章骨架。）


## §13.2 ROS 2 多机通信层 ⭐⭐⭐

> **这一节解决什么问题**：§13.1 用命名空间把 N 个 agent 隔离开了，但这只是"逻辑隔离"。当机器人数量从 2 涨到 10、20，一个看不见的敌人会出现——**DDS 发现风暴**：每个节点都要向全网广播"我在这、我发布/订阅这些话题"，发现流量随节点数平方增长，到一定规模网络直接瘫。这一节讲透多机系统的通信底层：命名空间隔离、多机 TF 树管理、以及三种 DDS 发现方案的选型——这是决定你的系统"能不能从 2 机长到 20 机"的关键。

### 模块功能与在系统中的位置

通信层是整个 Mini-MultiBot 的"神经系统"。它在系统里的角色有三层：**（1）隔离**——保证 N 个机器人的同名话题互不串扰；**（2）连接**——让需要协调的 agent 之间能交换状态（邻居通信）；**（3）扩展**——保证加机器人时网络不被发现流量压垮。第 1 章告诉你"用什么架构"，这一节告诉你"这个架构在 ROS 2 上具体怎么联网"。

### 动机：从"能跑"到"跑得动"

回到 Quick Start——两个 talker 在不同命名空间下各自发消息，看起来多机通信很简单。但那是 2 个节点。真实多机系统每个机器人有十几个节点（控制器、状态发布、TF、传感器、邻居通信……），8 个机器人就是上百个节点。这时 ROS 2 默认的**多播发现（multicast discovery）**会暴露它的扩展性问题。

ROS 2 不像 ROS 1 有个中央 master——它用 DDS 做**去中心化的对等发现**：每个节点启动时向全网多播"我来了"，并持续维护与所有其他节点的连接信息。这个设计在单机/少节点时优雅（无单点故障），但代价是：

| 节点数 | 发现流量量级 | 现象 |
|--------|------------|------|
| 2-10 | 可忽略 | 一切正常 |
| 10-50 | 明显 | 启动变慢，`ros2 topic list` 卡顿 |
| 50-200+ | 风暴 | 节点随机掉线、CPU 被发现线程吃满、新节点起不来 |

> **本质洞察**：ROS 2 的去中心化发现是一把双刃剑——它消除了 ROS 1 的 master 单点故障，但把"发现"的成本从"一次性注册到 master"变成了"持续地与全网每个节点维护连接"。发现流量近似随节点数**平方**增长（每个节点要发现其他所有节点）。多机系统的通信优化，本质上就是想办法**把这个平方降下来**——要么减少彼此可见的节点（域隔离），要么改用集中式发现（Discovery Server），要么只桥接必要话题（Domain Bridge）。这三种方案就是下面要讲的。

### 反面：如果不管发现机制会怎样

设想你照着 §13.1 的骨架，天真地把 `team_2_go2.yaml` 扩成 `team_16_go2.yaml`，直接 launch。会发生什么？

- 启动阶段：16 个 agent × 十几个节点 ≈ 200+ 节点同时多播发现，网络瞬间被 DDS 的 `PDP`（参与者发现协议）和 `EDP`（端点发现协议）报文淹没。
- 运行阶段：每个节点维护与其他 ~200 个节点的连接状态，发现线程持续占用 CPU。你会看到 `htop` 里每个进程都有一个吃 CPU 的后台线程。
- 故障阶段：某些节点因为发现超时而认为邻居"掉线"，TF 树出现断点，控制器收不到状态，整个系统进入"薛定谔的连接"——时好时坏。

这不是 bug，是 ROS 2 默认配置在大规模下的固有行为。修复它需要**主动管理发现机制**，下面三种方案各有取舍。

### 方案对比：三种 DDS 发现策略

| 方案 | 原理 | 优点 | 代价 | 适用规模 |
|------|------|------|------|---------|
| **默认多播发现** | 每节点向全网多播，全连接 | 零配置、无单点 | 发现流量 $O(N^2)$ | < 10 机 |
| **Domain ID 隔离** | 每机器人独立 domain，互不可见 | 彻底切断跨机发现流量 | 跨机通信要靠 domain_bridge 显式桥接 | 机间通信稀疏时 |
| **Discovery Server** | 一个中央服务器代理发现，节点只连服务器 | 发现流量降到 $O(N)$，扩展性好 | 服务器是发现的单点（但数据仍 P2P） | 10-100+ 机 |

下面分别讲怎么配。

#### 策略一：Domain ID 隔离 + Domain Bridge

思路：给**每个机器人一个独立的 domain ID**，于是机器人之间在 DDS 层面完全看不见对方——发现流量被彻底切断。但机器人有时确实需要交换信息（邻居状态、共享地图），这部分用 `domain_bridge` **显式桥接**指定话题。

```bash
# 机器人 0 用 domain 100，机器人 1 用 domain 101 —— 它们互相看不见
# 终端（机器人 0 的节点）
ROS_DOMAIN_ID=100 ros2 launch mmb_agent single_agent.launch.py ns:=go2_0

# 终端（机器人 1 的节点）
ROS_DOMAIN_ID=101 ros2 launch mmb_agent single_agent.launch.py ns:=go2_1
```

跨域桥接只搬运邻居通信必需的话题，用一个 YAML 描述桥哪些：

```yaml
# config/domain_bridge.yaml —— 只桥接邻居状态，其余话题不跨域
name: mmb_bridge
topics:
  # 把机器人 0 的 state（domain 100）桥到 domain 101，供机器人 1 订阅
  go2_0/state:
    type: mini_multibot_msgs/msg/AgentState
    from_domain: 100
    to_domain: 101
  go2_1/state:
    type: mini_multibot_msgs/msg/AgentState
    from_domain: 101
    to_domain: 100
```

```bash
# 启动桥（它同时连两个 domain，搬运指定话题）
ros2 run domain_bridge domain_bridge config/domain_bridge.yaml
```

> **本质洞察**：Domain Bridge 的设计哲学是"**默认隔离，按需连接**"——这正好对应第 1 章分布式架构的精神：机器人默认是独立的，只在通信图 $G$ 的边上交换信息。桥接的话题列表，本质就是把第 1 章的邻接矩阵 $A$ 写成了 YAML。这也是为什么分布式系统的通信开销可控——你只桥接 $O(|E|)$ 条边的话题，而非 $O(N^2)$ 的全连接。

#### 策略二：Fast DDS Discovery Server（推荐用于 10+ 机）

思路：保留单一 domain（所有机器人在同一 domain，话题可直接互通），但把"发现"这件事从"每节点向全网多播"改成"每节点只向一个中央 Discovery Server 注册"。**注意：只有发现走服务器，实际数据传输仍是点对点的**——服务器不是数据中转，只是"通讯录"。

```bash
# 1. 启动 Discovery Server（监听一个固定端口，作为全网通讯录）
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 &

# 2. 所有节点通过环境变量指向这个服务器（不再多播）
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

# 3. 现在启动多机系统，发现流量从 O(N^2) 降到 O(N)
ros2 launch mini_multibot_bringup bringup_sim.launch.py
```

发现流量的变化可以量化——这是工程实践要求的"配置→效果"桥接：

| 配置 | 发现报文复杂度 | 16 机系统启动时间（实测量级） |
|------|--------------|----------------------------|
| 默认多播 | $O(N^2)$，每节点发现所有节点 | 30-60 秒，常有节点起不来 |
| Discovery Server | $O(N)$，每节点只连服务器 | 5-10 秒，稳定 |

> **配置陷阱（提前预警，详见陷阱专栏）**：用了 Discovery Server 后，**所有**要互通的终端都必须设 `ROS_DISCOVERY_SERVER`，包括你后开的 `ros2 topic echo` 调试终端。漏设的终端会回退到多播发现，看不到任何走服务器的节点——表现为"我的系统在跑，但 `ros2 node list` 啥也看不到"。

#### 策略三：保持默认（< 10 机时不要过早优化）

如果你的系统就 2-8 个机器人，**别折腾**——默认多播发现完全够用，引入 Discovery Server 反而增加一个要维护的进程和一个潜在故障点。这是工程实践的重要原则：**过早优化是万恶之源**。本章的两个核心任务（双 Go2、Go2+UAV）都是少机场景，用默认发现即可；Discovery Server 留给你做"蜂群扩展"实验时再上。

### 多机 TF 树管理

通信层的另一个核心是 **TF（坐标变换）树**。单机时 TF 很简单——`odom → base_link → 各传感器`。多机时如果不加命名空间，N 个机器人都发布 `odom → base_link`，TF 树会"打架"：tf2 看到多个 publisher 往同一对坐标系发变换，结果是变换值在 N 个机器人间乱跳。

正确的多机 TF 树结构（沿用第 13 章原始骨架的设计）：

```
world                              ← 全局参考系（仿真器或动捕系统提供）
├── go2_0/odom → go2_0/base_link   ← 机器人 0 自己的里程计链
│   └── go2_0/lidar, go2_0/imu ...
├── go2_1/odom → go2_1/base_link   ← 机器人 1 自己的里程计链
│   └── go2_1/lidar, go2_1/imu ...
└── (coordinator 发布 world → go2_i/odom，把各机器人锚到全局系)
```

设计要点：

- **每个 agent 发布自己的 `odom→base_link`**：因为这是 agent-local 信息（自己的里程计），命名空间前缀保证不冲突。
- **谁来发 `world→go2_i/odom`？** 这是"把各机器人放到同一个全局坐标系"的关键变换，是 system-level 信息。仿真里由仿真器提供真值；实机里由动捕（Crazyswarm2 场景）或全局定位（SLAM/UWB）提供。
- **为什么不直接让所有机器人共用一个 `odom`？** 因为里程计是相对的、会漂移，每个机器人的漂移不同。强行共用会导致机器人 A 的漂移污染机器人 B 的定位。

代码上，多机 TF 的命名空间前缀通过 `robot_state_publisher` 的 frame_prefix 参数实现：

```python
# 在 single_agent.launch.py 里，给每个 agent 的 robot_state_publisher 加前缀
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=ns,                       # 如 go2_0
    parameters=[{
        'robot_description': robot_urdf,
        'frame_prefix': f'{ns}/',       # 关键：所有 TF frame 自动加 go2_0/ 前缀
    }],
)
```

> **配置陷阱（提前预警）**：`frame_prefix` 必须以 `/` 结尾（如 `go2_0/`），否则前缀会和 frame 名粘连成 `go2_0base_link`，导致 TF 查找失败。这是多机 TF 最高频的坑。

### 代码走读：邻居通信封装

把通信层的"邻居交换"逻辑封装成一个可复用的类 `NeighborComm`，这是 §13.1 骨架里 `mmb_agent/neighbor_comm.py` 的核心。它的职责：**发布自身状态、订阅所有邻居状态、提供"取最新邻居状态"的接口给上层协调算法用**。

```python
# mmb_agent/mmb_agent/neighbor_comm.py
import rclpy
from rclpy.node import Node
from mini_multibot_msgs.msg import AgentState


class NeighborComm:
    """邻居通信封装：发布自身状态 + 订阅邻居状态。
    这是 §13.3 共识/ADMM 的数据来源——协调算法只管调
    get_neighbor_states()，不必关心 ROS 2 话题细节。
    """

    def __init__(self, node: Node, agent_id: int, neighbor_ids: list):
        self._node = node
        self._agent_id = agent_id
        self._neighbor_ids = neighbor_ids
        self._neighbor_states = {}        # nb_id -> 最新 AgentState

        # 发布自身状态（在本 agent 命名空间下，相对话题 'state'
        # 经 PushRosNamespace 自动变成 /go2_<id>/state）
        self._state_pub = node.create_publisher(AgentState, 'state', 10)

        # 为每个邻居建一个订阅。话题用绝对名（跨命名空间！）
        for k, nb_id in enumerate(neighbor_ids):
            # 注意：这里订阅的是 launch 里 remap 好的 neighbor_<k>/state
            node.create_subscription(
                AgentState,
                f'neighbor_{k}/state',     # 经 remapping 指向 /go2_<nb_id>/state
                self._make_callback(nb_id),
                10)

    def _make_callback(self, nb_id):
        """闭包：把 nb_id 绑进回调，存最新邻居状态。"""
        def _cb(msg):
            self._neighbor_states[nb_id] = msg
        return _cb

    def publish_self(self, state: AgentState):
        """上层控制器算出自身状态后，调这个发出去。"""
        state.agent_id = self._agent_id
        self._state_pub.publish(state)

    def get_neighbor_states(self) -> dict:
        """协调算法（共识/ADMM）取邻居最新状态。
        返回 {nb_id: AgentState}。缺失的邻居（还没收到）不在字典里。
        """
        return dict(self._neighbor_states)

    def all_neighbors_ready(self) -> bool:
        """检查是否已收到所有邻居的状态——协调步开始前的门控。"""
        return len(self._neighbor_states) == len(self._neighbor_ids)
```

逐段解读这个封装的设计决策：

- **为什么封装成类而不是写在节点里？** 因为"邻居通信"是 agent-local 的通用能力，Go2 控制器和 UAV 控制器都要用。封装后，§13.3 的共识、§13.5 的编队都能复用同一个 `NeighborComm`，不必各自重写订阅逻辑。这是"关注点分离"在代码级的落地。
- **`_make_callback` 为什么用闭包？** 因为我们在循环里为每个邻居建订阅，需要把当前的 `nb_id` "冻结"进回调。如果不用闭包直接写 `lambda msg: self._neighbor_states[nb_id]=msg`，Python 的延迟绑定会让所有回调都用循环结束后的最后一个 `nb_id`——这是个经典的 Python 陷阱（下面陷阱专栏详述）。
- **`get_neighbor_states` 为什么返回拷贝？** 防止上层算法在迭代字典时，订阅回调在另一个线程改字典导致 `RuntimeError: dictionary changed size during iteration`。多线程 executor 下这很重要。
- **`all_neighbors_ready` 的作用**：协调算法（如 ADMM）需要所有邻居的当前解才能算协调步。这个门控避免"邻居还没发状态就开始算"导致用到空数据。

### ⚠️ 常见陷阱

```
🔧 配置陷阱：用了 Discovery Server，但调试终端漏设 ROS_DISCOVERY_SERVER
   错误做法：系统用 Discovery Server 启动了，新开一个终端直接跑
            ros2 node list 想看节点
   现象：ros2 node list 返回空，或只看到极少数节点，仿佛系统没起来——
        但系统其实在正常运行
   根本原因：没设 ROS_DISCOVERY_SERVER 的终端回退到默认多播发现，
            它和走服务器发现的节点处于两套"发现域"，互相看不见
   正确做法：把 export ROS_DISCOVERY_SERVER="127.0.0.1:11811" 写进
            mmb_env.sh，确保每个新终端 source 后都指向服务器。
            验证：设了之后 ros2 node list 立刻能看到全部节点
```

```
⚠️ 编程陷阱：循环建邻居订阅时回调闭包用了延迟绑定的循环变量
   错误做法：for nb_id in neighbor_ids:
                node.create_subscription(..., lambda m: store(nb_id, m), 10)
   现象：所有邻居的消息都被存到了同一个 nb_id（最后一个）名下，
        其他邻居的状态永远是空，共识/ADMM 用错数据静默发散
   根本原因：Python 闭包延迟绑定——lambda 捕获的是变量 nb_id 的引用，
            循环结束后所有 lambda 看到的 nb_id 都是最后一次的值
   正确做法：用工厂函数把当前值"冻结"进闭包（见上文 _make_callback），
            或用默认参数 lambda m, nid=nb_id: store(nid, m)。
            检查方法：打印每个邻居订阅实际收到的 agent_id，确认一一对应
```

```
💡 概念误区：以为 Discovery Server 是数据中转，担心它成为带宽瓶颈
   新手想法："所有节点都连一个 Discovery Server，那所有数据不都要
            过这台服务器吗？它会不会被大数据量话题（如点云）压垮？"
   实际上：Discovery Server 只代理"发现"（谁在哪、有什么话题），
          不中转任何实际数据。两个节点一旦通过服务器互相"发现"了，
          它们的数据传输是直接点对点的，完全不经过服务器
   正确理解：把 Discovery Server 想成"通讯录服务"而非"邮局"——它帮你
            找到对方的地址，但你寄信（数据）是直接寄给对方的。所以即使
            桥接 GB 级点云，服务器的负载也几乎为零。这也是为什么它能
            支撑上百机器人
```

### 练习

1. **【配置练习】** 在你的 4-Go2 系统（§13.1 练习 1 的成果）上，先用默认多播发现启动，用 `ros2 multicast receive` 或 Wireshark 观察发现流量；再切换到 Discovery Server 启动，对比启动时间。验收标准：能说出两种方案下 `ros2 node list` 出全部节点所需的时间差，并解释为什么 Discovery Server 更快。

2. **【管线搭建】** 用 `domain_bridge` 实现一个"半隔离"系统：机器人 0、1 在 domain 100，机器人 2、3 在 domain 101，只桥接四个机器人的 `/state` 话题（让协调器能看到全部 4 个）。验收标准：在 domain 100 的终端 `ros2 topic echo /go2_2/state`（跨域）能收到数据，但 `ros2 node list`（同域）看不到 domain 101 的节点。

3. **【综合思考】** 第 1 章讲过通信图的代数连通性 $\lambda_2$ 决定共识收敛速率。现在从**工程**角度回答：如果你想让系统协调更快（大 $\lambda_2$，稠密拓扑），通信层会付出什么代价？如果想省带宽（稀疏拓扑），协调会慢多少？请把"$\lambda_2$ vs 通信开销"这个第 2 章的理论权衡，翻译成"`neighbors` 字段该怎么填"的工程决策。（跨章综合题：第 1/2 章理论 + 本章通信层。）


## §13.3 分布式协调内核 ⭐⭐⭐⭐

> **这一节解决什么问题**：通信层（§13.2）让 agent 能交换信息了，但"交换信息"本身不是协调——协调是**用交换来的信息做某种迭代，让全系统收敛到一致的决策**。这一节把第 2 章的两套数学引擎——**共识协议**和 **ADMM**——从纸面公式落成 ROS 2 上能跨进程运行的代码。这是整个系统从"一堆各自动的机器人"变成"一个协调的系统"的关键一跃，也是本章最硬核的一节。

### 模块功能与在系统中的位置

协调内核（`mmb_coordination` 包）是 Mini-MultiBot 的"大脑皮层"。它在系统里的位置：**上承**通信层（从 `NeighborComm` 拿邻居状态），**下接**运动层（把协调出的结果——一致的编队参考、协调的耦合力——喂给 §13.5 的控制器）。第 2 章给了它的数学灵魂（$\dot x=-Lx$ 和 ADMM 三步），这一节给它工程的躯体。

### 动机：把"迭代公式"变成"定时器回调"

第 2 章你学的共识是一个连续时间微分方程 $\dot x_i = \sum_{j\in\mathcal{N}_i} a_{ij}(x_j - x_i)$，或离散形式 $x_i(k{+}1) = x_i(k) + \epsilon\sum_{j}a_{ij}(x_j(k)-x_i(k))$。这是数学。但机器人不会解微分方程——它跑的是一个**周期性回调**：每隔 20ms，读一次邻居状态，按公式更新一次自己的状态，发出去。

这个"从迭代公式到定时器回调"的翻译，是本节的核心技能。它看似机械，实则藏着多机系统特有的工程难题：邻居状态是**异步到达**的（不像数学里假设的同步）、网络有**延迟和丢包**（第 2 章讲过临界时延 $\tau^\star$）、不同机器人的时钟**不完全同步**。把这些现实塞进那个干净的迭代公式，就是工程化。

> **本质洞察**：分布式协调算法的"分布式"二字，在代码里体现为一件非常具体的事——**没有任何一个节点拥有全局状态**。共识节点只知道自己和邻居的状态；ADMM 的本地节点只解自己的子问题。整个系统的"一致决策"不是某个节点算出来的，而是从 N 个节点的局部迭代中**涌现**出来的。理解这一点，你就理解了为什么协调内核的代码里找不到一个"全局变量"——有的只是每个节点反复地"读邻居、更新自己、发出去"。

### 子模块一：共识协议落地

先把最基础的共识落成代码。场景：N 个机器人要对某个标量/向量达成一致（比如统一编队的朝向、对负载质量的估计、或选举一个临时 leader）。这是第 2 章 $\dot x=-Lx$ 的直接工程化。

#### 配置详解

共识节点的关键配置：

| 配置项 | 含义 | 推荐值 | 可调范围 | 耦合 |
|--------|------|--------|---------|------|
| `consensus_rate` | 迭代频率 (Hz) | 50 | 10-100 | 受网络延迟限制，过高邻居数据跟不上 |
| `epsilon` | 共识步长 $\epsilon$ | 0.3 | 0-1/$d_{max}$ | 超过 $1/d_{max}$（最大度）会发散——第 2 章稳定性条件 |
| `convergence_tol` | 收敛判据 | 1e-3 | — | 相邻两步变化小于此值认为收敛 |

`epsilon` 的取值是第 2 章理论的直接应用——这是工程实践要求的"配置项的默认值来源"：离散共识 $x(k{+}1)=(I-\epsilon L)x(k)$ 收敛要求 $\epsilon < 2/\lambda_{max}(L)$，工程上保守取 $\epsilon < 1/d_{max}$（$d_{max}$ 是最大节点度），因为 $\lambda_{max}(L) \le 2 d_{max}$。

#### 代码走读

```python
# mmb_coordination/mmb_coordination/consensus_core.py
import numpy as np
import rclpy
from rclpy.node import Node
from mmb_agent.neighbor_comm import NeighborComm
from mini_multibot_msgs.msg import AgentState, ConsensusMsg


class ConsensusNode(Node):
    """离散平均共识的 ROS 2 实现——第 2 章 x(k+1)=x(k)+ε·Σ a_ij(x_j-x_i)
    的工程化。每个机器人各跑一份，靠邻居话题交换，最终全网收敛到初值平均。
    """

    def __init__(self):
        super().__init__('consensus')
        # 从参数读拓扑与超参（来自 §13.1 的 YAML）
        self.agent_id = self.declare_parameter('agent_id', 0).value
        self.neighbor_ids = self.declare_parameter('neighbors', []).value
        rate = self.declare_parameter('consensus_rate', 50.0).value
        self.epsilon = self.declare_parameter('epsilon', 0.3).value

        # 本节点的共识状态（这里用 3 维向量举例，如统一的编队中心）
        self.x = np.zeros(3)
        self._init_local_value()        # 用本机的初始观测初始化

        # 共识专用的邻居通信（交换 ConsensusMsg 而非完整 AgentState）
        self._neighbor_x = {}           # nb_id -> np.ndarray
        self._pub = self.create_publisher(ConsensusMsg, 'consensus', 10)
        for k, nb in enumerate(self.neighbor_ids):
            self.create_subscription(
                ConsensusMsg, f'neighbor_{k}/consensus',
                self._make_cb(nb), 10)

        # 核心：周期性迭代
        self.create_timer(1.0 / rate, self._iterate)

    def _init_local_value(self):
        # 实战中这里读本机传感器，比如本机当前位置作为编队中心的初始猜测
        self.x = np.array([float(self.agent_id), 0.0, 0.0])

    def _make_cb(self, nb_id):
        def _cb(msg: ConsensusMsg):
            self._neighbor_x[nb_id] = np.array(msg.value)
        return _cb

    def _iterate(self):
        """一次共识迭代——这就是 ẋ=-Lx 的离散一步。"""
        # 1. 先把自己当前状态发出去（让邻居能用到最新值）
        self._pub.publish(ConsensusMsg(agent_id=self.agent_id,
                                       value=self.x.tolist()))
        # 2. 攒齐邻居数据才更新（异步到达，缺谁就跳过谁，用已有的）
        if not self._neighbor_x:
            return
        # 3. 共识更新：x += ε·Σ(x_j - x_i)
        delta = np.zeros_like(self.x)
        for nb_id in self.neighbor_ids:
            if nb_id in self._neighbor_x:
                delta += (self._neighbor_x[nb_id] - self.x)
        self.x = self.x + self.epsilon * delta
        # 4.（可选）记录收敛性，供 §13.8 调试
        self.get_logger().debug(f'agent {self.agent_id} x={self.x}')
```

逐段拆解这段代码如何把第 2 章的数学变成现实：

- **第 1 步先发后算**：注意迭代里先 `publish` 再更新。这保证邻居总能拿到你"上一步"的状态。如果先更新再发，会引入一拍的额外延迟。这是异步分布式系统的细节——数学公式里假设大家同步，工程里必须显式管理"先发还是先算"。
- **第 2/3 步的"缺谁跳谁"**：数学公式假设每一步都能拿到所有邻居的当前值。但网络异步——某邻居这一拍的消息可能还没到。工程做法是**用手头已有的最新邻居值**（缺失的邻居不计入这一步的 delta）。这会让收敛略慢，但保证系统不会因为等某个邻居而卡死。这是第 2 章理论（同步假设）与工程现实（异步）的桥接点。
- **`epsilon` 的稳定边界**：如果你把 `epsilon` 设成 1.5（超过 $1/d_{max}$），你会亲眼看到 `self.x` 振荡发散——这是第 2 章稳定性条件 $\epsilon < 2/\lambda_{max}(L)$ 被违反的直接后果。建议你真去试一次，把抽象的稳定性条件变成肌肉记忆。

#### 验证：共识收敛实验

```bash
# 起一个 4 机环形拓扑的纯共识实验
ros2 launch mmb_coordination consensus_demo.launch.py team:=team_4_go2.yaml

# 另开终端，记录每个机器人的共识值，看是否收敛到同一个数
ros2 topic echo /go2_0/consensus & ros2 topic echo /go2_1/consensus &
ros2 topic echo /go2_2/consensus & ros2 topic echo /go2_3/consensus &
# 期望：4 个机器人的 value 字段从各自的初值（0,1,2,3）
#       逐步收敛到平均值 1.5，收敛曲线随 ε 和拓扑（λ2）变化
```

**成功标志**：4 个机器人的共识值都收敛到初值平均 1.5（环形拓扑下约几十次迭代）。这就是第 2 章"共识收敛到平均"的眼见为实。你可以改 `team` 的拓扑（环形→全连接）观察收敛变快——这是 $\lambda_2$ 增大的直接效果。

### 子模块二：ADMM 分布式协调器落地

共识让机器人对一个**标量/向量**达成一致。但很多多机问题要协调的不是一个数，而是**每个机器人的整个优化解**——比如双 Go2 搬运时，两个机器人各自的足端力要协调，使它们对负载的合力正好托起负载。这就要用 ADMM（第 2 章 §2.4）。

ADMM 的多机版本结构：每个 agent 解一个**本地子问题**（带一个"向全局变量靠拢"的罚项），coordinator 收集所有本地解、算**全局协调变量**（常是平均）、广播回去，各 agent 再更新**对偶变量**。三步循环，直到本地解与全局变量一致。

#### 配置详解

| 配置项 | 含义 | 推荐值 | 可调范围 | 耦合 |
|--------|------|--------|---------|------|
| `admm_rho` | 罚参数 $\rho$ | 1.0 | 0.1-10 | 第 2 章 §2.4：大→收敛快但易振荡 |
| `admm_iterations` | 每控制周期迭代轮数 | 3 | 1-10 | × `consensus_rate` = 通信负载；实时系统受限 |
| `n_agents` | 参与协调的机器人数 | 来自 YAML | — | 决定 coordinator 收集多少本地解 |

`admm_iterations=3` 的取值是工程权衡——这是配置项的"默认值来源"：理论上 ADMM 要迭代到收敛（几十上百轮），但实时控制每个周期只有几毫秒，跑不了那么多轮。工程做法是**每控制周期只跑固定的少数几轮（warm start：用上周期的解作为这周期的初值）**，靠控制周期间的连续性"摊还"收敛。这是第 4 章分布式 MPC 的标准技巧。

#### 代码走读：本地 ADMM agent

```python
# mmb_coordination/mmb_coordination/admm_local.py（agent 侧，每机一份）
import numpy as np


class ADMMLocalSolver:
    """ADMM 的本地子问题求解器（agent 侧）。
    协调的对象 z 是"共享变量"——例如双机搬运中作用在负载上的合力。
    每个 agent 维护自己对 z 的局部副本 x_i 和对偶变量 u_i。
    """

    def __init__(self, dim: int, rho: float):
        self.rho = rho
        self.x = np.zeros(dim)      # 本地解（如本机分担的负载力）
        self.u = np.zeros(dim)      # 缩放对偶变量（拉格朗日乘子/ρ）
        self.z = np.zeros(dim)      # 当前全局协调变量（从 coordinator 收）

    def local_update(self, local_cost_grad):
        """x 更新：解 argmin_x [ f_i(x) + (ρ/2)||x - z + u||² ]。
        这里用一步梯度近似（实战中 f_i 是本机的 MPC/QP，调真求解器）。
        local_cost_grad: 本地代价 f_i 在当前 x 处的梯度。
        """
        # 近端梯度一步：把本地代价梯度和"向 z 靠拢"的罚项梯度合起来
        grad = local_cost_grad(self.x) + self.rho * (self.x - self.z + self.u)
        self.x = self.x - 0.1 * grad        # 0.1 为本地步长
        return self.x

    def dual_update(self):
        """对偶更新：u = u + (x - z)。约束违反量累积到对偶变量上。"""
        self.u = self.u + (self.x - self.z)

    def set_global(self, z):
        """从 coordinator 接收新的全局协调变量。"""
        self.z = np.array(z)
```

#### 代码走读：coordinator 协调步

```python
# mmb_coordination/mmb_coordination/admm_coordinator.py（system-level，一份）
import numpy as np
import rclpy
from rclpy.node import Node
from mini_multibot_msgs.msg import ConsensusMsg


class ADMMCoordinator(Node):
    """ADMM 的全局协调节点——收集各 agent 的本地解 x_i，算全局 z，广播回去。
    注意：它不解任何优化问题，只做'聚合 + 广播'。这正是分布式架构里
    '轻量协调者'的角色——计算重担在各 agent，它只搬运信息。
    """

    def __init__(self):
        super().__init__('admm_coordinator')
        self.n_agents = self.declare_parameter('n_agents', 2).value
        self.rho = self.declare_parameter('admm_rho', 1.0).value

        self._local_x = {}      # agent_id -> 本地解 x_i
        # 订阅所有 agent 的本地解
        for i in range(self.n_agents):
            self.create_subscription(
                ConsensusMsg, f'/go2_{i}/admm_local',
                self._make_cb(i), 10)
        # 广播全局协调变量 z（所有 agent 都订阅 /shared/admm_global）
        self._z_pub = self.create_publisher(ConsensusMsg, '/shared/admm_global', 10)
        self.create_timer(0.02, self._coordinate)   # 50 Hz 协调步

    def _make_cb(self, agent_id):
        def _cb(msg):
            self._local_x[agent_id] = np.array(msg.value)
        return _cb

    def _coordinate(self):
        """z 更新：全局协调变量 = 各本地解的平均（共识平均型 ADMM）。
        对'合力约束'这类问题，z 可以是带约束的投影，这里用最简平均示例。
        """
        if len(self._local_x) < self.n_agents:
            return      # 等所有 agent 都报了本地解
        # 全局变量 = 平均（这一步对应第 2 章 ADMM 的 z-update）
        z = np.mean(list(self._local_x.values()), axis=0)
        self._z_pub.publish(ConsensusMsg(value=z.tolist()))
```

把这两段代码合起来看，ADMM 的"三步"在系统里是这样分布的——这是理解分布式协调的关键映射：

| ADMM 步骤 | 第 2 章数学 | 在系统里谁做、怎么做 |
|-----------|-----------|---------------------|
| **x 更新** | $x_i = \arg\min f_i(x)+\frac{\rho}{2}\|x-z+u_i\|^2$ | 每个 **agent** 本地做（`ADMMLocalSolver.local_update`），解自己的子问题 |
| **z 更新** | $z = \frac{1}{N}\sum_i(x_i+u_i)$ | **coordinator** 做（`_coordinate`），聚合所有本地解算平均 |
| **u 更新** | $u_i = u_i + (x_i - z)$ | 每个 **agent** 本地做（`dual_update`），累积约束违反 |

> **本质洞察**：ADMM 的精妙在于它把一个**全局耦合**的优化问题（双机合力必须等于负载重量——这把两机绑在一起）切成了 N 个**本地可独立求解**的子问题，耦合只通过"向全局变量 z 靠拢"的罚项隐式存在。coordinator 不需要知道每个 agent 的目标函数 $f_i$ 长什么样——它只看到本地解 $x_i$，算平均。这就是为什么 ADMM 能做分布式：**计算在边缘（agent），协调在中心（coordinator），但中心不需要全局模型**。对比集中式——集中式 coordinator 要知道所有 agent 的完整模型才能解联合问题，这正是第 1 章说的"$O(N^3)$ 爆炸"的来源。

### 数据流全景：协调内核怎么接进系统

把共识和 ADMM 接进 §13.1 骨架后，一个 agent 内部的数据流是这样的（这是理论-工程桥接的关键图）：

```
   传感器/状态估计
        │ (本机状态)
        ▼
   ┌──────────────┐   邻居状态    ┌──────────────┐
   │ NeighborComm │◄──────────────│ 邻居 agent   │
   │  (§13.2)     │──────────────►│              │
   └──────┬───────┘   自身状态     └──────────────┘
          │ 邻居状态
          ▼
   ┌──────────────────────┐
   │  协调内核 (§13.3)      │
   │  ┌────────┐ ┌───────┐ │   本地解 x_i   ┌─────────────┐
   │  │共识     │ │ADMM   │ │──────────────►│ coordinator │
   │  │core    │ │local  │ │◄──────────────│ (z 广播)    │
   │  └────────┘ └───────┘ │   全局变量 z   └─────────────┘
   └──────────┬───────────┘
              │ 协调后的参考（一致的编队中心 / 协调的分担力）
              ▼
       运动层 (§13.5：编队 + 避障)
```

读这张图的关键：**协调内核夹在通信层和运动层之间**。它从通信层吸取邻居信息，吐出"协调好的参考量"给运动层。运动层（下一节就要搭）不必关心协调是怎么发生的——它只管"把协调内核给的参考跟踪好"。这种分层让每一层都能独立测试和替换。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：共识/ADMM 迭代在回调里直接读写共享状态，多线程下数据竞争
   错误做法：用 MultiThreadedExecutor 跑节点，订阅回调写 self.x，
            定时器回调也读写 self.x，二者并发
   现象：偶发的 NaN、收敛曲线出现无法解释的跳变、ros2 launch 偶尔崩溃，
        且难以复现（典型的竞态特征）
   根本原因：多个回调在不同线程并发读写同一个 numpy 数组，
            没有锁保护，出现撕裂读写（torn read/write）
   正确做法：要么用 SingleThreadedExecutor（回调串行，最简单，本章默认）；
            要么对共享状态加 threading.Lock。多机协调对实时性要求没高到
            必须多线程，优先用单线程 executor 避开整类问题
```

```
🔧 配置陷阱：ADMM rho 设得过大，本地子问题被罚项主导，机器人"不听自己的"
   错误做法：为了让一致性快，把 admm_rho 设成 50
   现象：机器人完全跟着全局变量 z 走，忽略自己的本地代价 f_i，
        搬运任务里表现为"两机僵硬地保持队形但都不出力托负载"
   根本原因：罚项 (ρ/2)||x-z+u||² 的权重远大于本地代价 f_i，
            local_update 的梯度被罚项主导，x 退化为"复制 z"
   正确做法：rho 从 1.0 起调，观察"本地代价下降"和"一致性收敛"是否平衡。
            第 2 章给过自适应 rho 的方法（残差比值法）。检查：打印本地代价
            f_i(x) 和一致性残差 ||x-z||，二者应同步下降而非一个压倒另一个
```

```
💡 概念误区：以为 coordinator 节点是"集中式"，违背了分布式的初衷
   新手想法："不是说分布式没有中央节点吗？怎么 ADMM 还有个 coordinator？
            这不就是集中式吗？"
   实际上：coordinator 只做"信息聚合"（算平均），不解任何优化问题，
          不需要任何 agent 的模型。它是 O(N) 的轻量聚合，不是 O(N³) 的
          联合求解。计算的重担完全在各 agent 的本地子问题上
   正确理解：分布式的本质不是"绝对没有中心节点"，而是"没有节点掌握全局
            模型、计算可并行"。ADMM 的 coordinator 是"协调者"不是"决策者"。
            真要做到完全去中心化（连聚合都不要中央节点），可以用第 2 章的
            共识替代 coordinator 的平均——这就是去中心化 ADMM，本章 §13.3
            练习 3 会让你实现
```

### 练习

1. **【管线搭建】** 实现"分布式 leader 选举"：N 个机器人用最大值共识（把平均改成取最大）对"谁的 agent_id 最大"达成一致，最终所有机器人都同意同一个 leader。提示：把 `_iterate` 里的更新规则从"趋向平均"改成"取邻居与自身的最大值"。验收标准：4 机系统中所有机器人的共识值都收敛到 3（最大 id），且收敛轮数 ≈ 图直径。

2. **【性能调优】** 在 ADMM 搬运场景里，固定拓扑，扫描 `admm_rho` ∈ {0.1, 1, 5, 20}，记录每个值下"一致性残差 ||x_i - z|| 降到 1e-2 所需的迭代轮数"和"是否振荡"。画出 rho-收敛轮数曲线，找到你这个问题的最优 rho。验收标准：能复现第 2 章的结论——存在一个中间 rho 使收敛最快，过大过小都更慢。

3. **【综合思考/进阶】** 把 ADMM 的 coordinator 干掉，改成**完全去中心化**：每个 agent 不再把本地解发给中央 coordinator，而是用 §13.3 子模块一的共识，让 agent 们直接对"全局变量 z"做分布式平均（每个 agent 维护自己的 z 估计，靠邻居共识同步）。这样系统就没有任何中央节点了。请说明：去中心化版本相比有 coordinator 的版本，多付出了什么（提示：收敛速度、通信轮数），换来了什么（提示：容错性）。（跨章综合题：第 2 章共识 + 本节 ADMM。）


## §13.4 任务分配集成——把"谁干什么"接进系统 ⭐⭐⭐

> **这一节解决什么问题**：协调内核（§13.3）让机器人能就连续量（编队中心、分担力）达成一致。但在运动之前，有一个更上层的**离散决策**要先做——"哪个机器人去做哪件任务？" 这是第 3 章的任务分配。这一节不重新实现分配算法（第 3 章已经写好了 `mmb_planning/`），而是把它**包装成一个 ROS 2 服务**接进系统，让分配结果驱动下游的编队与运动。重点是**集成的接口设计**，而非算法本身。

### 模块功能与在系统中的位置

任务分配模块（`mmb_coordination/task_allocator.py`）是系统的"调度中枢"。它在数据流里的位置：**输入**当前所有机器人的位置 + 待办任务列表，**输出**一份分配表（agent_id → task_id），**下游**是 §13.5 的编队层（每个机器人知道自己要去哪个任务点后，才能规划编队运动）。第 3 章给了它的算法（匈牙利/CBBA/CBS），这一节给它系统接口。

### 动机：分配为什么要做成"服务"而非"话题"

这是一个值得展开的设计决策——它体现了 ROS 2 通信模式选型的工程思维。

任务分配有一个鲜明特点：**它是请求-响应式的，不是持续流式的**。你不是每 20ms 都要重新分配一次任务（那是控制层的事）；你是在"任务集合变了"（来了新任务、某任务完成、某机器人故障）这些**离散事件**发生时，才需要重新算一次分配。

| 通信模式 | 适合的场景 | 任务分配适合吗 |
|---------|-----------|--------------|
| **Topic（话题）** | 持续的数据流（状态、传感器、控制指令） | ❌ 分配不是连续流 |
| **Service（服务）** | 请求-响应、偶发、需要明确结果 | ✅ 完美匹配 |
| **Action（动作）** | 长时间运行、需要反馈和取消 | △ 分配算她很快，不需要 |

所以我们把分配做成 **Service**：下游需要分配时主动 `call`，分配器算完一次性返回结果。这避免了"话题模式下分配器要无意义地每帧重算"的浪费。

> **本质洞察**：ROS 2 的三种通信模式（话题/服务/动作）对应三种数据时序——**持续流、请求响应、长任务**。多机系统的每个模块该用哪种，取决于它的数据是"流"还是"事件"。状态、控制指令是流（话题）；任务分配、参数查询是事件（服务）；导航到点、抓取是长任务（动作）。选对通信模式，系统的时序就自然清晰——这是比"算法对不对"更底层的工程素养。

### 配置详解：分配服务接口

先定义服务接口 `AllocateTasks.srv`（在 `mmb_msgs` 里）。接口设计是集成的灵魂——它定义了"分配器和系统其余部分如何对话"：

```
# mini_multibot_msgs/srv/AllocateTasks.srv
# ===== 请求 =====
geometry_msgs/Point[] robot_positions    # 各机器人当前位置（按 agent_id 顺序）
geometry_msgs/Point[] task_positions     # 各任务点位置
string method                            # "hungarian" / "cbba" / "greedy"
---
# ===== 响应 =====
int32[] assignment                       # assignment[i] = 机器人 i 被分到的任务 id
float64 total_cost                       # 总代价（用于评估分配质量）
bool success
```

每个字段的设计考量：

| 字段 | 为什么这样设计 |
|------|--------------|
| `robot_positions` 按 agent_id 顺序 | 隐式约定索引即 agent_id，省一个 id 字段；下游用 `assignment[i]` 直接查机器人 i 的任务 |
| `method` 字符串选算法 | 让调用方运行时切换匈牙利/CBBA，不用改服务端代码——对应第 3 章"集中最优 vs 分布次优"的选型 |
| 返回 `total_cost` | 不只给结果，还给质量度量。下游可据此判断"这次分配够好吗"，§13.7 评估也要用 |
| `assignment` 用索引数组 | 紧凑、易解析。`assignment=[2,0,1]` 表示机器人 0→任务 2、机器人 1→任务 0、机器人 2→任务 1 |

### 代码走读：任务分配服务

```python
# mmb_coordination/mmb_coordination/task_allocator.py
import numpy as np
import rclpy
from rclpy.node import Node
from mini_multibot_msgs.srv import AllocateTasks
# 直接复用第 3 章已经实现的算法——不重写！
from mmb_planning.hungarian import hungarian_assignment
from mmb_planning.cbba import cbba_assignment


class TaskAllocatorService(Node):
    """任务分配服务节点。把第 3 章的分配算法包成 ROS 2 服务。
    下游（编队层/上层调度）需要分配时 call 这个服务，一次性拿到结果。
    """

    def __init__(self):
        super().__init__('task_allocator')
        # 创建服务，绑定回调
        self._srv = self.create_service(
            AllocateTasks, 'allocate_tasks', self._on_allocate)
        self.get_logger().info('Task allocator service ready.')

    def _on_allocate(self, request, response):
        """服务回调：收到请求 → 算分配 → 填响应。"""
        # 1. 把 ROS 消息转成算法要的 numpy 代价矩阵
        robots = np.array([[p.x, p.y, p.z] for p in request.robot_positions])
        tasks = np.array([[p.x, p.y, p.z] for p in request.task_positions])
        # 代价 = 机器人到任务的欧氏距离（实战可换成路径长度/能耗）
        cost = np.linalg.norm(
            robots[:, None, :] - tasks[None, :, :], axis=2)   # (n_robot, n_task)

        # 2. 按请求的 method 选算法——第 3 章的集中 vs 分布选型在此落地
        try:
            if request.method == 'hungarian':
                assignment, total = hungarian_assignment(cost)   # 最优, O(n³)
            elif request.method == 'cbba':
                assignment, total = cbba_assignment(cost)        # 分布式, 50%界
            else:  # greedy 兜底
                assignment, total = self._greedy(cost)
            response.assignment = list(map(int, assignment))
            response.total_cost = float(total)
            response.success = True
            self.get_logger().info(
                f'Allocated via {request.method}: {response.assignment}, '
                f'cost={response.total_cost:.2f}')
        except Exception as e:
            response.success = False
            self.get_logger().error(f'Allocation failed: {e}')
        return response

    @staticmethod
    def _greedy(cost):
        """贪婪兜底：每个机器人挑当前最便宜的未分配任务（第 3 章的次优基线）。"""
        n = cost.shape[0]
        assignment = -np.ones(n, dtype=int)
        used = set()
        total = 0.0
        for i in range(n):
            order = np.argsort(cost[i])
            for j in order:
                if j not in used:
                    assignment[i] = j
                    used.add(j)
                    total += cost[i, j]
                    break
        return assignment, total
```

逐段解读这个集成的关键设计：

- **复用而非重写（第 1-2 行 import）**：注意我们从 `mmb_planning` 直接 import 第 3 章的 `hungarian_assignment` 和 `cbba_assignment`。这是 R11 累积项目原则的直接体现——**前面章节的成果作为模块被后面复用**，而不是复制粘贴。第 3 章的 `mmb_planning/` 包就是为此存在的。
- **消息↔算法的转换层（第 1 步）**：服务回调的第一件事是把 ROS 消息（`geometry_msgs/Point[]`）转成算法要的 numpy 数组。这个转换层是集成的常见模式——**算法不应该依赖 ROS 类型，ROS 节点负责类型转换**。这样第 3 章的纯算法代码能脱离 ROS 单独测试。
- **运行时算法切换（第 2 步）**：通过 `request.method` 字符串选算法，把第 3 章"集中最优（匈牙利）vs 分布次优（CBBA）"的选型权交给调用方。这让你能在同一个系统里对比不同分配算法的效果，无需改服务端。
- **异常处理**：分配可能失败（任务数≠机器人数、矩阵奇异）。服务用 `success` 字段而非抛异常，让调用方能优雅处理——这是服务接口设计的良好实践。

### 代码走读：上层调度怎么调这个服务

光有服务还不够，要有人调它。下面是上层调度（可放进 coordinator 或单独的 mission 节点）如何在"任务集合变化"时触发分配：

```python
# 上层调度的核心片段——事件驱动地调用分配服务
class MissionScheduler(Node):
    def __init__(self):
        super().__init__('mission_scheduler')
        self._alloc_client = self.create_client(AllocateTasks, 'allocate_tasks')
        # 发布分配结果给下游编队层（话题模式：结果是要持续告知下游的状态）
        self._assign_pub = self.create_publisher(Assignment, '/shared/assignment', 10)

    def reassign(self, robot_positions, task_positions):
        """任务集变化时调用——重新分配并广播结果。"""
        if not self._alloc_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Allocator service unavailable!')
            return
        req = AllocateTasks.Request()
        req.robot_positions = robot_positions
        req.task_positions = task_positions
        req.method = 'hungarian'        # 少量机器人用最优；蜂群规模换 'cbba'
        future = self._alloc_client.call_async(req)
        future.add_done_callback(self._on_result)

    def _on_result(self, future):
        resp = future.result()
        if resp.success:
            # 把分配结果广播给所有 agent（话题），各 agent 据此知道自己去哪
            msg = Assignment(assignment=resp.assignment, total_cost=resp.total_cost)
            self._assign_pub.publish(msg)
            self.get_logger().info(f'Published assignment: {resp.assignment}')
```

注意这里的**通信模式混搭**——这是工程实践的精髓：

- **调用分配用 Service**（`call_async`）：因为分配是请求-响应的离散事件。
- **广播结果用 Topic**（`_assign_pub`）：因为分配结果是一个**持续有效的状态**，所有 agent 都要知道且要能随时查到最新值。新加入的 agent 也能立刻订阅到当前分配。

> **对比性思维（不是 X 而是 Y）**：一个常见的新手做法是"全用话题"——分配也做成话题，每帧重算。这不是"简化"，而是**用错了通信模式**：它让分配器无意义地每秒算几十次（任务没变也算），既浪费 CPU 又让 CBBA 这种迭代算法无法收敛（每帧重启）。正确的是**分配用服务（事件触发）、结果用话题（状态广播）**——让通信模式匹配数据的时序本质。

### 集成验证：分配→运动的端到端

```bash
# 起带任务分配的系统
ros2 launch mini_multibot_bringup bringup_sim.launch.py \
  team:=team_4_go2.yaml enable_allocation:=true

# 手动触发一次分配（模拟"来了 4 个巡检任务"）
ros2 service call /allocate_tasks mini_multibot_msgs/srv/AllocateTasks \
  "{robot_positions: [{x: 0,y: 0,z: 0},{x: 1,y: 0,z: 0},{x: 0,y: 1,z: 0},{x: 1,y: 1,z: 0}],
    task_positions:  [{x: 5,y: 5,z: 0},{x: 5,y: 0,z: 0},{x: 0,y: 5,z: 0},{x: 3,y: 3,z: 0}],
    method: 'hungarian'}"

# 期望响应：assignment 是一个 4 元素数组，total_cost 是最优总距离
# 验证广播：下游应收到分配
ros2 topic echo /shared/assignment --once
```

**成功标志**：服务返回一个合理的 `assignment`（每个机器人分到不同任务、总代价最小），且 `/shared/assignment` 话题广播了这个结果。把 `method` 改成 `greedy` 重跑，对比 `total_cost`——你会看到贪婪的总代价通常比匈牙利高，这正是第 3 章"贪婪次优"结论的眼见为实。

### ⚠️ 常见陷阱

```
🔧 配置陷阱：服务调用用同步 call() 阻塞了节点的 executor，整个节点卡死
   错误做法：在一个回调里直接 client.call(req)（同步等待）
   现象：调用分配服务后整个节点僵住，其他回调（状态发布、TF）全停，
        系统看起来"死了"
   根本原因：同步 call() 在单线程 executor 里会死锁——它等服务响应，
            但响应的处理也要这个 executor，executor 被自己阻塞
   正确做法：用 call_async() + add_done_callback()（见上文 reassign）。
            ROS 2 里服务调用几乎总该用异步。检查：调用后节点其他话题
            应继续正常发布
```

```
⚠️ 编程陷阱：分配返回的 assignment 索引和 agent_id 对应关系搞反
   错误做法：把 assignment[i] 理解成"任务 i 分给了机器人 assignment[i]"
   现象：机器人朝错误的任务点跑，4 个机器人去重了同一个点或漏掉某点
   根本原因：assignment 数组的语义是"机器人 i → 任务 assignment[i]"，
            而非"任务 i → 机器人 assignment[i]"，两种约定方向相反
   正确做法：在 .srv 注释里明确写死语义（见上文接口定义），
            下游消费时严格按"机器人索引 → 任务值"解析。检查：打印
            (robot_i 位置, task_{assignment[i]} 位置) 对，确认配对合理
```

```
💡 概念误区：以为任务分配做一次就够了，不需要重分配
   新手想法："开始时分配好谁去哪，之后照着执行就行，分配是一次性的"
   实际上：真实系统里任务和机器人都在变——新任务到达、任务完成、
          机器人故障/掉电、环境变化使某分配不再最优。静态一次性分配
          会在第一个意外发生时崩溃
   正确理解：分配是事件驱动的、需要重入的。这正是第 3 章"终身 MAPF
            (lifelong)"的动机。系统要在"任务集变化"和"机器人状态变化"
            事件上重新触发分配（reassign）。本节把分配做成可反复调用的
            服务，正是为了支持这种动态重分配
```

### 练习

1. **【集成测试】** 实现"机器人故障触发重分配"：用 §13.1 练习 2 的健康监控节点，当检测到某 agent 掉线时，自动调用分配服务，把任务在**剩余健康机器人**间重新分配。验收标准：手动 kill 掉一个 agent 节点，系统应在 1-2 秒内重新分配，且死掉机器人的任务被转给活着的机器人。

2. **【性能调优】** 对比匈牙利和 CBBA 在不同规模下的表现：机器人数 N ∈ {4, 8, 16, 32}，记录两种方法的"求解时间"和"总代价"。画出 N-求解时间曲线和 N-代价比（CBBA代价/匈牙利代价）曲线。验收标准：能复现第 3 章结论——匈牙利代价更优但时间随 N 增长更快，CBBA 代价约在最优的 1-2 倍但可扩展。

3. **【综合思考】** 把任务分配（本节）和编队（下一节 §13.5）的接口想清楚：分配输出"机器人 i 去任务点 p_i"，但如果这些机器人还要**保持编队**地一起移动（不是各走各的），分配该输出什么？是"每个机器人一个独立目标点"，还是"整个编队一个目标 + 编队内的相对位置"？请设计这个接口，说明它如何同时满足"任务覆盖"和"编队约束"。（跨章综合题：第 3 章分配 + 第 4 章编队 + 本章集成。）


## §13.5 编队与避障集成——三层运动栈 ⭐⭐⭐

> **这一节解决什么问题**：分配（§13.4）告诉了每个机器人"去哪"，协调内核（§13.3）让它们能就连续量达成一致。现在到了最终落地——**让机器人真的协调地动起来，既保持队形又不撞上彼此和障碍**。这一节搭起运动层的三层栈：**编队几何（决定队形）→ ADMM 协调（处理耦合）→ ORCA 避障（兜底安全）**。重点是这三层如何串接、各自的失效边界、以及"高层期望"如何一步步变成"轮子/腿的指令"。

### 模块功能与在系统中的位置

运动层（`mmb_formation` 包）是系统的"小脑+脊髓"——把上层的抽象意图（"去那个任务点，保持菱形队形"）转成具体的速度/力指令。它在数据流末端：**输入**来自分配层（目标点）和协调内核（一致的编队参数），**输出**给每个机器人的底层控制器（`cmd_vel` 或足端力），**最终**驱动仿真器/真机。

### 动机：为什么运动要分三层

一个自然的问题：为什么不直接写一个控制器，一步到位算出"既到目标、又保队形、又不撞"的指令？因为这三件事的**时间尺度和职责完全不同**，混在一起会变成无法调试的泥潭：

| 层 | 职责 | 时间尺度 | 失效后果 |
|----|------|---------|---------|
| **编队几何层** | 决定理想队形（菱形/直线/环形）、每个机器人的理想位置 | 慢（队形变化是策略级） | 队形不对，但不会撞 |
| **协调层（ADMM）** | 处理机器人间的耦合（搬运合力、避免队形内冲突） | 中（控制周期级，几十 Hz） | 协调不好，队形抖动 |
| **避障层（ORCA）** | 兜底安全，紧急避免碰撞 | 快（反应级，高频） | 直接撞车 |

> **本质洞察**：运动分层的核心是**"慢决策包络快反应"**——编队层做慢的策略决策（什么队形），避障层做快的安全反应（别撞），中间用 ADMM 协调耦合。这种分层不是为了"代码好看"，而是因为**不同时间尺度的控制不能耦合在一个回路里**：如果让编队层去管避障，它的慢更新跟不上突然出现的障碍；如果让避障层去管队形，它的局部视野看不到全局编队目标。每一层只在自己的时间尺度上负责，是控制系统设计的基本原则。这与第 4 章分布式 MPC 的分层（轨迹层 vs WBC 层）是同一个思想。

### 第一层：编队几何

编队几何层回答"理想队形是什么、每个机器人的理想位置在哪"。最常用两种范式（第 2/4 章讲过编队三范式，这里落地两种）：

```python
# mmb_formation/mmb_formation/formation_keeper.py
import numpy as np


class FormationGeometry:
    """编队几何：给定编队中心和朝向，算每个机器人的理想位置。
    支持 leader-follower 和 virtual-structure 两种范式（第 2/4 章）。
    """

    def __init__(self, formation_type='virtual_structure'):
        self.type = formation_type
        # 队形定义：各机器人相对编队中心的偏移（编队坐标系下）
        # 菱形队形示例（4 机）
        self.offsets = {
            0: np.array([1.0, 0.0]),    # 前
            1: np.array([0.0, 1.0]),    # 左
            2: np.array([-1.0, 0.0]),   # 后
            3: np.array([0.0, -1.0]),   # 右
        }

    def desired_position(self, agent_id, center, yaw):
        """算某机器人的理想全局位置。
        center: 编队中心全局坐标 (2,)；yaw: 编队朝向。
        """
        # 把编队坐标系的偏移旋转到全局坐标系
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        return center + R @ self.offsets[agent_id]

    def formation_error(self, agent_id, current_pos, center, yaw):
        """当前位置与理想队形位置的偏差——编队控制的误差信号。"""
        return self.desired_position(agent_id, center, yaw) - current_pos
```

这里的关键设计——**编队中心 `center` 和朝向 `yaw` 从哪来？** 这正是协调内核（§13.3）的产出！在 virtual-structure 范式下，所有机器人要对"编队中心在哪"达成一致——这就用 §13.3 的共识。在 leader-follower 范式下，center 就是 leader 的位置，followers 订阅 leader 的状态。**编队几何层消费协调内核的输出，这就是两层的接口。**

### 第二层：编队 + 任务目标的融合

编队几何给了"保持队形"的速度分量，但机器人还要"朝任务点移动"（§13.4 分配的目标）。这两个目标如何融合？用加权合成——这是一个经典的多目标控制问题：

```python
class FormationController:
    """融合'保持队形'和'奔向任务'两个目标，输出期望速度。"""

    def __init__(self, k_formation=1.0, k_goal=0.8, v_max=1.0):
        self.geom = FormationGeometry()
        self.k_formation = k_formation    # 队形保持增益
        self.k_goal = k_goal              # 目标吸引增益
        self.v_max = v_max                # 速度上限

    def compute_velocity(self, agent_id, current_pos, formation_center,
                         formation_yaw, goal_pos):
        """算期望速度 = 队形保持项 + 目标吸引项。"""
        # 1. 队形保持：拉向理想队形位置
        v_form = self.k_formation * self.geom.formation_error(
            agent_id, current_pos, formation_center, formation_yaw)
        # 2. 目标吸引：拉向分配的任务点（整个编队朝目标走）
        v_goal = self.k_goal * (goal_pos - formation_center)
        # 3. 合成并限幅
        v = v_form + v_goal
        speed = np.linalg.norm(v)
        if speed > self.v_max:
            v = v / speed * self.v_max
        return v        # 这个 v 还要过第三层避障才能下发！
```

> **对比性思维（反事实）**：如果不做队形保持项（`k_formation=0`）会怎样？→ 每个机器人各自直奔任务点，编队立刻散架，退化成 N 个独立机器人。如果不做目标项（`k_goal=0`）？→ 机器人死守队形但整个编队不动，永远到不了任务点。两项的权重比 `k_formation/k_goal` 决定了"队形刚度 vs 任务紧迫"的权衡——任务紧急时调高 k_goal（宁可队形松点也要快到），狭窄通道里调高 k_formation（宁可慢点也要保持队形挤过去）。

### 第三层：ORCA 局部避障兜底

前两层算出的期望速度可能导致碰撞——两个机器人的期望速度可能指向同一点，或某机器人会撞上障碍。最后一层用 **ORCA（Optimal Reciprocal Collision Avoidance，最优互惠碰撞避免）** 做安全兜底：它接收期望速度，输出一个**最接近期望、但保证不碰撞**的安全速度。

ORCA 的核心思想（这里讲原理，完整实现可用 `pyorca`/RVO2 库）：对每个邻居，构造一个"速度禁区"（velocity obstacle 的线性化半平面），机器人的安全速度必须落在所有禁区之外的可行域里。求"离期望速度最近的可行速度"是一个小型线性规划。

```python
# mmb_formation/mmb_formation/orca_avoidance.py
import numpy as np


class ORCAFilter:
    """ORCA 安全滤波：把期望速度投影到'不碰撞'的可行速度集。
    这是运动栈的最后一道防线——无论上层期望什么，这里保证不撞。
    简化版：基于速度障碍的半平面约束 + 最近点投影。
    """

    def __init__(self, robot_radius=0.3, time_horizon=2.0, v_max=1.0):
        self.radius = robot_radius        # 机器人半径（含安全裕度）
        self.tau = time_horizon           # 前瞻时间：多久内不碰撞
        self.v_max = v_max

    def filter(self, pos, v_pref, neighbors):
        """pos: 自身位置；v_pref: 期望速度（前两层算的）；
        neighbors: [(邻居位置, 邻居速度), ...]。
        返回：最接近 v_pref 但满足所有避碰约束的安全速度。
        """
        # 为每个邻居构造一个 ORCA 半平面约束 a·v <= b
        constraints = []
        for nb_pos, nb_vel in neighbors:
            rel_pos = nb_pos - pos
            dist = np.linalg.norm(rel_pos)
            if dist < 1e-6:
                continue
            # 速度障碍的边界法向（指向远离邻居方向）
            n = rel_pos / dist
            # 互惠：各让一半责任（ORCA 的 reciprocal 精髓）
            # 约束：相对速度在 n 方向的投影要 <= 安全阈值
            safe_speed = max(0.0, (dist - 2 * self.radius) / self.tau)
            constraints.append((n, safe_speed + 0.5 * np.dot(nb_vel, n)))

        # 求解：min ||v - v_pref||² s.t. 所有 n·v <= b（小型 QP/LP）
        return self._solve_closest_feasible(v_pref, constraints)

    def _solve_closest_feasible(self, v_pref, constraints):
        """找离 v_pref 最近的满足所有半平面约束的 v（投影法迭代近似）。"""
        v = v_pref.copy()
        for _ in range(20):       # 投影迭代（实战用真 QP 求解器）
            violated = False
            for n, b in constraints:
                if np.dot(n, v) > b:        # 违反约束
                    v = v - (np.dot(n, v) - b) * n   # 投影回半平面
                    violated = True
            # 限幅
            sp = np.linalg.norm(v)
            if sp > self.v_max:
                v = v / sp * self.v_max
            if not violated:
                break
        return v
```

ORCA 的关键设计点——这是理论-工程桥接：

- **互惠（reciprocal）的精髓**：注意约束里的 `0.5`——两个机器人**各承担一半**避让责任。如果每个机器人都假设"对方不动，全靠我躲"，两个机器人会做出对称的过度避让，导致"两人对着躲、反复横跳"（行人迎面避让的尴尬舞蹈）。ORCA 的互惠假设让双方各让一半，避免了这种振荡。这是 ORCA 比朴素速度障碍法（VO）高明的地方。
- **前瞻时间 `tau`**：决定"多远的碰撞才开始反应"。tau 大 → 远远就开始避让（保守、平滑但绕路多）；tau 小 → 临近才避让（激进、路径短但险）。这是配置项，要根据机器人速度和场景调。
- **为什么放在最后一层**：ORCA 只保证"不撞"，不保证"到目标"或"保队形"——它是纯安全兜底。把它放最后，意味着**它有最终否决权**：无论上层多想往某方向走，只要会撞，ORCA 就拦下。这种"安全层在最后、有否决权"的架构，和 §13.6 的 CBF 安全滤波是同一个设计哲学。

### 三层串接：完整的运动栈

把三层串起来，一个 agent 每个控制周期做的事：

```python
# mmb_agent 控制循环里的运动栈串接（伪代码骨架）
def motion_control_step(self):
    # ===== 输入：从各层收集 =====
    my_pos = self.state.position                       # 自身状态
    formation_center = self.consensus.x[:2]            # §13.3 共识出的编队中心
    formation_yaw = self.consensus.x[2]                # 共识出的编队朝向
    my_goal = self.assignment.get_my_goal()            # §13.4 分配的任务点
    neighbors = [(nb.pos, nb.vel)                       # §13.2 邻居状态
                 for nb in self.neighbor_comm.get_neighbor_states().values()]

    # ===== 第一+二层：编队几何 + 目标融合 =====
    v_pref = self.formation_ctrl.compute_velocity(
        self.agent_id, my_pos, formation_center, formation_yaw, my_goal)

    # ===== 第三层：ORCA 安全兜底 =====
    v_safe = self.orca.filter(my_pos, v_pref, neighbors)

    # ===== 输出：下发给底层控制器 =====
    # 轮式：直接是 cmd_vel；四足：v_safe 喂给步态/MPC 转足端力
    self.local_controller.execute(v_safe)
```

这段串接代码是整个运动层的"主旋律"，把前面所有模块焊在一起。读它的关键是看**数据从哪来**：编队中心来自 §13.3 协调内核、目标来自 §13.4 分配、邻居来自 §13.2 通信层。这一行行的数据来源，正是前四节工作的汇合点——这就是"集成"的具体含义。

> **本质洞察**：注意 `v_pref`（期望）和 `v_safe`（安全）的区别——上层算"我想怎么动"，避障层算"我能怎么动"。两者之间的差，就是"安全的代价"。一个健康的系统里，绝大多数时候 `v_safe ≈ v_pref`（畅通无阻），只在临近碰撞时 `v_safe` 才显著偏离 `v_pref`。如果你发现 `v_safe` 长期大幅偏离 `v_pref`，说明环境太挤或队形参数不合理——这是一个有用的系统健康指标，§13.8 会用到。

### 集成验证：编队移动避障

```bash
# 起完整运动栈：4 机菱形编队，朝目标移动，途中有障碍
ros2 launch mini_multibot_bringup bringup_sim.launch.py \
  team:=team_4_go2.yaml world:=warehouse_with_obstacles.sdf

# 给整个编队一个目标点，看它们保持菱形挤过障碍区
ros2 topic pub /shared/formation_goal geometry_msgs/Point "{x: 8.0, y: 0.0, z: 0.0}" --once

# 在 RViz 里观察（或用 mmb_eval/visualize.py）
ros2 launch mmb_eval visualize.launch.py
```

**成功标志**：4 个机器人保持菱形队形（虽然过障碍时会暂时变形）向目标移动，无碰撞，到达后恢复菱形。如果你看到机器人对着障碍"反复横跳"，说明 ORCA 的 tau 太大或互惠没生效——回陷阱专栏。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：ORCA 拿到的邻居速度是"上一帧"的，导致避让滞后
   错误做法：邻居速度直接用订阅到的最新 AgentState.velocity，
            不考虑通信延迟
   现象：机器人避让总慢半拍，高速时仍偶发擦碰，密集场景下避让"卡顿"
   根本原因：邻居状态经网络传来有延迟（第 2 章的 τ），ORCA 用滞后的
            邻居速度构造约束，等于按"邻居过去在哪"避让而非"现在在哪"
   正确做法：对邻居状态做简单外推（用收到的位置+速度+时间戳预测当前位置），
            或在 ORCA 的 tau 里预留通信延迟裕度。检查：高速对向运动时
            两机的最小间距应 ≥ 2×半径
```

```
🔧 配置陷阱：编队增益 k_formation 远大于 ORCA 的避让力度，安全被"挤穿"
   错误做法：为了队形紧，把 k_formation 设很大，ORCA 的限幅 v_max 又很高
   现象：机器人为保持队形硬挤，把 ORCA 的安全约束"压过去"，发生碰撞
   根本原因：三层栈里 ORCA 是最后否决，但如果上层期望速度的幅值远超
            ORCA 能修正的范围（或 ORCA 的 tau/radius 裕度不足），
            安全约束被违反
   正确做法：ORCA 必须有"硬"的最小间距保证（radius 含足够安全裕度），
            且其输出有最终限幅。编队增益再大，也要先过 ORCA 再下发。
            检查：人为制造两机冲突，确认 ORCA 一定拦下（间距不破 2×radius）
```

```
💡 概念误区：以为有了 ORCA 避障就不需要全局路径规划（MAPF）了
   新手想法："ORCA 能避障，那 §13.4 的 MAPF/路径规划是不是多余的？"
   实际上：ORCA 是局部反应式的——它只看邻居和近处障碍，没有全局视野。
          在死胡同、狭长走廊、需要"绕远路让行"的场景，ORCA 会陷入
          局部最优（死锁/活锁），因为它不知道全局拓扑
   正确理解：全局规划（MAPF）和局部避障（ORCA）是互补的两层，不是
            二选一。MAPF 给全局无冲突路径（避免死锁），ORCA 处理
            执行时的局部扰动和动态障碍（MAPF 没预见的）。第 3 章
            §3.8"从规划到执行"讲的就是这个分工。本章的栈里，
            §13.4 是全局层，ORCA 是执行层兜底
```

### 练习

1. **【配置练习】** 在编队移动避障场景里，扫描队形增益比 `k_formation/k_goal` ∈ {0.2, 1, 5}，观察机器人过狭窄障碍时的行为。验收标准：能描述并解释三种比值下的不同行为（散队抢行 / 平衡 / 死守队形挤不过），找到你这个场景的合理比值。

2. **【集成测试】** 把编队范式从 virtual-structure 切换成 leader-follower：指定 agent 0 为 leader（直奔目标，不受队形约束），其余 follower 用 §13.3 订阅 leader 状态作为编队中心。对比两种范式在"leader 突然转向"时的队形保持效果。验收标准：两种范式都能跑通，且能说出 leader-follower 在 leader 故障时的脆弱性（第 6 章讲过）。

3. **【综合思考】** 现在你有了完整的三层运动栈。请把它和第 4 章的分布式 MPC 编队对照：第 4 章用 ADMM 在毫秒级联合优化足端力，本章用"编队几何+ORCA"的分层。两者各适合什么场景？什么时候必须用第 4 章的重型 DMPC（提示：刚性耦合、接触约束），什么时候本章的轻量分层就够（提示：松耦合、点机器人近似）？请给出选型判据。（跨章综合题：第 4 章 DMPC + 本章运动栈。）


## §13.6 MARL 与规控混合——让"学习"驱动"控制" ⭐⭐⭐⭐

> **这一节解决什么问题**：前面的运动栈（§13.5）是纯传统规控——编队几何、ORCA 都是手工设计的规则。但第 10-12 章你训练出了 MAPPO 策略，它能学到"手工规则写不出"的协同行为（如复杂搬运的力协调、对抗环境下的队形调整）。这一节把训练好的策略**部署进系统**，让它驱动底层控制，并用 **CBF（控制屏障函数）安全滤波**给学习策略兜底。重点是两个工程边界：**训练环境↔部署环境**、**学习层↔控制层**。

### 模块功能与在系统中的位置

MARL 部署模块（`mmb_learning` 包）是系统的"高级决策皮层"——它用学习到的策略替代或增强 §13.5 的手工编队逻辑。在数据流里：**输入**各 agent 的观测（本体感知+邻居+任务），**输出**高层动作（期望速度/编队意图），**下游**经 CBF 滤波后给底层控制器。它和 §13.5 的关系是**可替换**——你可以让运动层用纯规控（§13.5），也可以用 MARL（本节），或两者混合。

### 动机：学习能给规控带来什么

一个尖锐的问题：§13.5 的编队+ORCA 已经能让机器人协调移动了，为什么还要上 MARL？因为手工规则有它的天花板：

| 场景 | 手工规控（§13.5）的局限 | MARL 的潜力 |
|------|----------------------|------------|
| **复杂接触协同** | 双机搬运不规则物体，手工力分配难写对 | 从交互中学到隐式的力协调 |
| **对抗/动态环境** | ORCA 在密集动态障碍中易死锁 | 学到预判性的避让策略 |
| **隐式通信** | 手工规则的通信内容要人设计 | 学到该交换什么信息（涌现通信） |
| **泛化** | 换负载/队形要重调参数 | 训练时域随机化后可泛化 |

> **对比性思维（不是 X 而是 Y）**：MARL 不是要**取代**传统规控，而是要**补足**它够不到的地方。一个常见误解是"有了 MARL 就不要 MPC/ORCA 了"——恰恰相反，最稳健的系统是**混合**的：MARL 做高层的、难以手工建模的协同决策，传统规控（CBF/ORCA）做底层的、必须有保证的安全约束。这正是第 12 章"MARL 与传统规控混合"的核心论点，本节是它的工程落地。

### 核心工程边界一：训练环境 ↔ 部署环境

这是 MARL 部署最大的坑，也是工程实践类文档必须讲透的版本/环境问题。回顾 §环境配置——我们特意把训练（独立 venv + PyTorch）和 ROS 2（系统 Python）分开。现在到了它们必须对接的时候。

**问题的本质**：策略在训练环境（IsaacLab/MQE，独立 venv）里学习，但要在部署环境（ROS 2 系统 Python）里执行。这两个环境的 Python、NumPy、甚至 PyTorch 版本可能不同，且不能直接互相 import。

**解决方案：用"权重文件 + 纯推理"解耦**。训练产出的不是"一个能跑的程序"，而是**一个权重文件**（`.pt`/`.onnx`）。部署侧只做纯前向推理，不依赖训练框架：

```
训练侧（独立 venv，§13.6 不管）          部署侧（ROS 2 系统 Python，本节）
  IsaacLab/MQE 训练 MAPPO                 加载 policy.pt
  → 保存 policy.pt（只存网络权重）  ───►   → 纯 torch 前向推理
  → 记录 obs/action 的规格（维度/归一化）   → 按规格构造 obs、解析 action
```

> **本质洞察**：MARL 部署的核心解耦原则是**"训练产出权重，部署只做推理"**。绝不要试图在 ROS 2 节点里 import 整个训练框架（IsaacLab 依赖一大堆 GPU 仿真库，根本装不进部署环境）。策略网络本质上就是几个矩阵乘法——部署侧只需要 PyTorch（甚至只需 ONNX Runtime）做前向。这个解耦让"重型训练"和"轻量部署"各用各的环境，通过一个 `.pt` 文件这一最小接口对接。这也是为什么 §环境配置坚持把两个 Python 环境分开。

### 核心工程边界二：observation 的精确复现

策略对 observation 的**顺序、维度、归一化**极其敏感——训练时 obs 是 `[本体速度(3), 邻居相对位置(2×N), 负载位置(3)]` 这个精确顺序和归一化，部署时必须**逐字节复现**。哪怕你把两个分量的顺序写反，策略的输出就是垃圾。

```python
# mmb_learning/mmb_learning/obs_builder.py
import numpy as np


class ObsBuilder:
    """精确复现训练时的 observation 构造——这是部署成败的关键。
    必须和训练代码里的 obs 定义逐项对齐：顺序、维度、归一化全部一致。
    """

    def __init__(self, n_neighbors, obs_mean, obs_std):
        self.n_neighbors = n_neighbors
        # 训练时保存的归一化参数——必须随权重一起带过来！
        self.obs_mean = np.array(obs_mean)
        self.obs_std = np.array(obs_std)

    def build(self, self_state, neighbor_states, load_state):
        """按训练时的精确顺序拼 obs。任何顺序错误都会让策略失效。"""
        obs = []
        # 1. 本体感知（训练时的顺序：线速度 xyz）
        obs.extend(self_state.velocity)                      # 3 维
        # 2. 邻居相对位置（按 agent_id 升序，训练时怎么排这里就怎么排）
        for nb_id in sorted(neighbor_states.keys()):
            nb = neighbor_states[nb_id]
            rel = np.array(nb.position) - np.array(self_state.position)
            obs.extend(rel[:2])                              # 每邻居 2 维
        # 补齐：邻居数不足时 padding（训练时若固定 n_neighbors 必须 pad）
        while len(obs) < 3 + 2 * self.n_neighbors:
            obs.append(0.0)
        # 3. 负载相对位置
        load_rel = np.array(load_state.position) - np.array(self_state.position)
        obs.extend(load_rel)                                 # 3 维
        # 4. 归一化（用训练时的统计量！否则策略输入分布错位）
        obs = np.array(obs, dtype=np.float32)
        return (obs - self.obs_mean) / (self.obs_std + 1e-8)
```

逐项解读这个 obs 构造为什么必须如此小心——这是 R8 配置详解的精神：

- **顺序必须逐项对齐训练代码**：神经网络的第一层权重是按训练时 obs 的顺序学的。部署时若 obs 顺序变了，等于把权重矩阵的列打乱——输出彻底失效，且**不会报错**（维度对就行），是最难查的 bug。
- **邻居必须固定排序**：训练时邻居按某顺序（如 agent_id 升序）排列，部署时必须一致。Python 字典的迭代顺序不可靠，所以显式 `sorted()`。
- **padding 处理**：如果训练时假设固定邻居数（如总是 3 个邻居），但部署时某机器人只有 2 个邻居（一个掉线），必须 padding 到训练维度，否则维度不匹配直接崩。
- **归一化参数必须随权重带过来**：训练时 obs 做了 `(x-mean)/std` 归一化，这个 mean/std 是训练数据的统计量。部署时必须用**同一组** mean/std——这是最容易漏的，漏了策略输入分布就错位，行为完全不对。

### 代码走读：策略部署节点

```python
# mmb_learning/mmb_learning/policy_deploy_node.py
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mmb_learning.obs_builder import ObsBuilder


class PolicyDeployNode(Node):
    """把训练好的 MAPPO 策略包成 ROS 2 节点。每个 agent 各跑一份
    （CTDE 的'分布执行'：每个 agent 只用自己的 Actor + 局部观测）。
    """

    def __init__(self):
        super().__init__('policy_deploy')
        self.agent_id = self.declare_parameter('agent_id', 0).value
        policy_path = self.declare_parameter('policy_path', '').value
        rate = self.declare_parameter('inference_rate', 20.0).value

        # 1. 加载策略权重（纯推理，不依赖训练框架）
        ckpt = torch.load(policy_path, map_location='cpu')
        self.actor = ckpt['actor']        # 只要 Actor！Critic 是训练时的，部署丢弃
        self.actor.eval()                 # 推理模式，关 dropout/batchnorm 更新
        # obs 构造器：带上训练时保存的归一化参数
        self.obs_builder = ObsBuilder(
            n_neighbors=ckpt['n_neighbors'],
            obs_mean=ckpt['obs_mean'], obs_std=ckpt['obs_std'])

        # 2. 输出：高层动作（这里是期望速度）
        self._cmd_pub = self.create_publisher(Twist, 'policy_cmd', 10)
        # 3. 定时推理（频率远低于底层控制——策略是高层决策）
        self.create_timer(1.0 / rate, self._infer)

    def _infer(self):
        """一次策略推理：收集 obs → 前向 → 发布动作。"""
        # 收集观测（数据来自 §13.2/§13.3 的各模块）
        if not self._ready():
            return
        obs = self.obs_builder.build(
            self._self_state, self._neighbor_states, self._load_state)
        # 前向推理（CTDE 分布执行：只用 Actor，只看自己的局部 obs）
        with torch.no_grad():
            action = self.actor(torch.from_numpy(obs).unsqueeze(0))
            action = action.squeeze(0).numpy()
        # 解析动作为期望速度（按训练时的 action 定义）
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.linear.y = float(action[1])
        cmd.angular.z = float(action[2])
        self._cmd_pub.publish(cmd)        # 注意：这个 cmd 还要过 CBF 滤波！
```

这个部署节点的几个关键工程决策：

- **只加载 Actor，丢弃 Critic（第 1 步）**：这是 CTDE 范式在部署侧的直接体现（第 10 章核心）。训练时 Critic 用全局状态算优势函数，部署时**每个 agent 只用自己的 Actor + 局部观测**——这正是"集中训练、分布执行"的"分布执行"。Critic 在部署时毫无用处，加载它纯属浪费。
- **`map_location='cpu'`**：部署机可能没 GPU（板载计算机常是 CPU）。策略网络小，CPU 推理足够快。这保证了"GPU 训练、CPU 部署"的可行。
- **推理频率远低于控制频率**：策略是**高层决策**（20 Hz），底层控制是**高频执行**（几百 Hz）。策略输出的是"意图"（期望速度），底层控制器负责把意图变成精确的力/力矩。这是第 12 章"分层混合"的标准结构——学习管慢的决策，控制管快的执行。

### CBF 安全滤波：给学习策略兜底

学习策略有一个根本问题——**它没有安全保证**。训练得再好，遇到分布外的情况（OOD）它可能输出危险动作（撞墙、撞同伴）。所以学习策略的输出**必须过一道安全滤波**才能下发。这里用 **CBF（Control Barrier Function，控制屏障函数）**。

CBF 的思想：定义一个"安全集"（如"和邻居距离 ≥ 安全距离"），CBF 保证系统**永远不离开**这个安全集。给定策略的期望动作 $u_{nom}$，CBF 求一个**最接近 $u_{nom}$、但保证安全**的动作 $u_{safe}$——这是一个 QP：

$$ u_{safe} = \arg\min_u \|u - u_{nom}\|^2 \quad \text{s.t.} \quad \dot h(x, u) \ge -\alpha\, h(x) $$

其中 $h(x) \ge 0$ 定义安全集，约束 $\dot h \ge -\alpha h$ 保证 $h$ 不会变负（不离开安全集）。

```python
# mmb_learning/mmb_learning/cbf_safety_filter.py
import numpy as np
from scipy.optimize import minimize


class CBFSafetyFilter:
    """CBF 安全滤波：把策略的标称动作投影到'安全'的动作集。
    安全集：和每个邻居的距离 ≥ d_safe。保证策略永远不会让机器人相撞。
    """

    def __init__(self, d_safe=0.6, alpha=2.0):
        self.d_safe = d_safe      # 安全距离
        self.alpha = alpha        # CBF 增益（class-K 函数斜率）

    def h(self, rel_pos):
        """屏障函数：h = ||rel_pos||² - d_safe²。h≥0 即安全。"""
        return np.dot(rel_pos, rel_pos) - self.d_safe ** 2

    def filter(self, pos, u_nom, neighbors, dt=0.05):
        """u_nom: 策略输出的标称速度；返回满足 CBF 约束的安全速度。"""
        constraints = []
        for nb_pos, nb_vel in neighbors:
            rel = pos - nb_pos
            h_val = self.h(rel)
            # CBF 约束：ḣ = 2·rel·(u - nb_vel) ≥ -α·h
            # 整理成 u 的线性约束： 2·rel·u ≥ -α·h + 2·rel·nb_vel
            grad_h = 2.0 * rel
            b = -self.alpha * h_val + np.dot(grad_h, nb_vel)
            constraints.append({
                'type': 'ineq',
                'fun': (lambda u, g=grad_h, bb=b: np.dot(g, u) - bb)
            })
        # QP：min ||u - u_nom||² s.t. CBF 约束
        res = minimize(lambda u: np.sum((u - u_nom) ** 2),
                       u_nom, constraints=constraints, method='SLSQP')
        return res.x if res.success else self._fallback(pos, neighbors)

    def _fallback(self, pos, neighbors):
        """QP 不可行时的保底：急停（最保守的安全动作）。"""
        return np.zeros(2)
```

CBF 与 ORCA（§13.5）的关系值得辨析——这是系统性分类：

| 维度 | ORCA（§13.5） | CBF（本节） |
|------|--------------|------------|
| **数学形式** | 速度障碍的线性半平面 | 屏障函数的微分不等式 |
| **保证** | 短时域内无碰撞 | 前向不变性（永不离开安全集）——更强 |
| **适用** | 多机互惠避让 | 给任意标称控制器（含学习策略）加安全约束 |
| **在本系统** | 纯规控运动栈的兜底 | 学习策略的兜底 |

> **本质洞察**：CBF 和 ORCA 都是"安全滤波器"——它们都接收一个"期望动作"，输出一个"最接近期望的安全动作"。区别在 CBF 提供更强的理论保证（前向不变性：一旦在安全集内就永远在内），且能套在**任何**标称控制器外面，包括黑盒的学习策略。这就是为什么学习策略要配 CBF：**学习负责"好"（高性能协同），CBF 负责"不坏"（安全保证）**——把性能和安全解耦到两个模块，各自用最擅长的方法（学习 vs 优化）。这是第 12 章混合架构的精髓。

### 完整混合栈：学习 + CBF + 底层控制

```python
# MARL 混合模式下，agent 控制循环的串接
def hybrid_control_step(self):
    # 1. 学习策略给高层意图（§13.6 策略节点，20 Hz）
    u_nom = self.latest_policy_cmd          # 来自 PolicyDeployNode

    # 2. CBF 安全滤波（保证不撞，可高频跑）
    neighbors = [(nb.pos, nb.vel)
                 for nb in self.neighbor_comm.get_neighbor_states().values()]
    u_safe = self.cbf.filter(self.state.position, u_nom, neighbors)

    # 3. 底层控制器执行（轮式直接 cmd_vel；四足转足端力，几百 Hz）
    self.local_controller.execute(u_safe)
```

把这个混合栈和 §13.5 的纯规控栈对照，你会发现结构惊人地一致——都是"高层意图 → 安全滤波 → 底层执行"。区别只在高层意图的来源：§13.5 是手工编队规则，§13.6 是学习策略。这种结构一致性不是巧合，而是**分层控制的通用模式**：无论高层用什么方法（规则/学习/优化），底层的安全兜底和执行是共享的。理解了这一点，你就能在系统里自由切换"纯规控 / 纯学习 / 混合"三种模式而不动底层。

### ⚠️ 常见陷阱

```
💡 概念误区：以为仿真训练好的策略直接部署就能用（忽略 obs 复现）
   新手想法："策略在 IsaacLab 里成功率 95%，部署到 ROS 2 应该也行"
   实际上：部署侧的 obs 构造和训练侧只要有一丁点不一致（顺序、归一化、
          邻居排序、单位），策略输出就是垃圾——且不报错，表现为"机器人
          乱动"，极难定位。这比 Sim2Real 的动力学差异更隐蔽
   正确理解：部署前必须做"obs 一致性验证"——用同一组输入，对比训练侧和
            部署侧的 obs 向量是否逐元素相等。本节的 ObsBuilder 必须和
            训练代码的 obs 定义逐项对齐，归一化参数随权重一起保存和加载
```

```
⚠️ 编程陷阱：部署时忘记 actor.eval()，BatchNorm/Dropout 仍在训练模式
   错误做法：torch.load 后直接推理，不调 .eval()
   现象：策略行为不稳定、同样的 obs 输出却不同、表现明显差于训练时
   根本原因：网络里若有 BatchNorm/Dropout，训练模式下 BatchNorm 用
            当前 batch 统计、Dropout 随机置零。推理时单样本 + 随机性
            导致输出错乱
   正确做法：加载后必须 self.actor.eval()，并用 torch.no_grad() 包推理。
            检查：同一个 obs 多次推理，输出应完全一致（确定性）
```

```
🔧 配置陷阱：CBF 的 alpha 设太大，安全滤波过度保守，机器人寸步难行
   错误做法：为了"绝对安全"，把 CBF 的 alpha 设成 50
   现象：机器人离邻居还很远就急刹/绕大圈，任务完全做不动，
        u_safe 长期严重偏离 u_nom
   根本原因：alpha 是安全集边界的"软硬程度"——alpha 大→约束在远离
            边界时就强烈生效，机器人被过度约束；CBF 退化成"谁都不许靠近"
   正确做法：alpha 从 2-5 起调，平衡"安全裕度"和"任务可行性"。
            监控 ||u_safe - u_nom||：正常应接近 0，只在临近危险时才偏离。
            长期大偏离说明 alpha 过大或 d_safe 过大
```

### 练习

1. **【集成测试】** 做一次"obs 一致性验证"：在训练侧 dump 一组 (输入状态, obs 向量)，在部署侧用 `ObsBuilder` 对同样的输入状态构造 obs，逐元素对比。验收标准：两侧 obs 向量逐元素差 < 1e-5。这是部署任何学习策略前的必做步骤——故意把 ObsBuilder 里两个分量顺序写反，观察策略行为如何崩坏，体会 obs 复现的重要性。

2. **【性能调优】** 对比"纯学习（无 CBF）"和"学习+CBF"两种模式在密集场景的安全性：记录两种模式下的最小机器人间距和碰撞次数。验收标准：纯学习模式偶有碰撞（间距 < d_safe），学习+CBF 模式碰撞次数为 0（间距始终 ≥ d_safe），代价是任务完成时间略增。量化这个"安全的代价"。

3. **【综合思考】** 本章给了三套运动方案：纯规控（§13.5 编队+ORCA）、纯学习（§13.6 MARL）、混合（MARL+CBF）。对照第 13 章评分标准里的"MPC vs MARL 定量对比"，设计一个实验：在"成功率、计算时间、泛化性、安全性、通信量"五个维度上对比这三套方案，画出 radar chart。说明每套方案在哪个维度占优、为什么。（跨章综合题：第 4 章 MPC + 第 10-12 章 MARL + 本章集成，对应评分标准的核心交付物。）


## §13.7 仿真验证——Gazebo 异构机与 Crazyswarm2 蜂群 ⭐⭐⭐

> **这一节解决什么问题**：前六节搭好了系统的全部软件层（通信→协调→分配→运动→学习）。现在要让它在**仿真器**里真的动起来——这是上真机前的最后一道关。本节讲两套仿真器：**Gazebo**（地面异构机，物理逼真）和 **Crazyswarm2**（空中蜂群，轻量大规模）。重点是理解两套仿真器的**抽象层级差异**、各自怎么接进 Mini-MultiBot、以及如何用它们验证协同任务。

### 模块功能与在系统中的位置

仿真器是系统的"虚拟现实"——它替代真机，提供物理动力学、传感器数据、执行器响应。在数据流里，仿真器**替换**了真机的位置：它接收各 agent 的控制指令（`cmd_vel`/足端力），返回机器人状态（位姿/速度/传感器）。Mini-MultiBot 的设计目标之一就是**仿真与真机共享同一套上层代码**——只换底层接口（仿真器 vs 真机驱动）。

### 动机：为什么需要两套仿真器

一个自然的疑问：有了 Gazebo 为什么还要 Crazyswarm2？因为它们的**抽象层级和适用规模完全不同**——这是工程实践的跨平台选型：

| 维度 | Gazebo (Fortress) | Crazyswarm2 |
|------|------------------|-------------|
| **物理逼真度** | 高（完整刚体动力学、接触、传感器噪声） | 低（简化质点/刚体，重在轨迹跟踪） |
| **单机计算成本** | 高（每机一份完整物理） | 低（轻量，可仿上百机） |
| **适用规模** | 2-10 机（受 CPU 限制） | 数十到上百机蜂群 |
| **适用对象** | 地面异构机（四足/轮式/臂） | 多旋翼蜂群 |
| **特色** | 传感器仿真、接触力、Sim2Real 高保真 | 与真实 Crazyflie 固件/硬件无缝切换 |

> **多视角理解（跨平台类比）**：Gazebo 和 Crazyswarm2 的关系，类似"高精度但慢的物理引擎"和"低精度但快的运动学引擎"。Gazebo 像汽车碰撞测试的全物理仿真——每个零件都算，逼真但算得慢；Crazyswarm2 像交通流仿真——把每辆车简化成质点，看的是宏观的群体行为而非单车的精确动力学。选哪个取决于你关心什么：关心"单个机器人的接触/平衡"用 Gazebo，关心"一大群机器人的编队/避障涌现行为"用 Crazyswarm2。这个边界正是它们设计目标的体现——不是哪个更好，而是测试的问题不同。

### Gazebo 路线：地面异构机仿真

Gazebo 路线对应本章核心任务"双 Go2 协同搬运"和"Go2+UAV 联合巡检"。关键是用 `ros_gz` 桥把 Gazebo 和 ROS 2 连起来，并 spawn 多个带命名空间的机器人。

#### 配置详解：多机 spawn

`spawn_robots.launch.py` 负责往 Gazebo 里 spawn N 个机器人，每个带独立命名空间和初始位姿（来自 §13.1 的 YAML）：

```python
# mini_multibot_bringup/launch/spawn_robots.launch.py（核心片段）
from launch_ros.actions import Node


def spawn_one_robot(agent, robot_sdf):
    """为一个 agent 在 Gazebo 里 spawn 一个实例。"""
    ns = agent['namespace']
    pose = agent['initial_pose']
    return [
        # 1. spawn 实体到 Gazebo（用 ros_gz_sim 的 create）
        Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-name', ns,                      # Gazebo 里的实体名
                '-file', robot_sdf,
                '-x', str(pose['x']), '-y', str(pose['y']),
                '-z', str(pose['z']), '-Y', str(pose['yaw']),
            ],
            output='screen',
        ),
        # 2. 为这个机器人起 ros_gz 桥（把 Gazebo 话题桥到 ROS 2 命名空间）
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            namespace=ns,
            arguments=[
                # cmd_vel: ROS 2 → Gazebo（控制指令下行）
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                # odom: Gazebo → ROS 2（状态上行）
                f'/{ns}/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            ],
        ),
    ]
```

每个配置项的讲解：

| 配置/参数 | 含义 | 注意 |
|-----------|------|------|
| `-name ns` | Gazebo 实体名，必须唯一 | 用命名空间名做实体名，保证唯一且对应 |
| `-x/-y/-z/-Y` | spawn 的初始位姿 | 来自 YAML 的 `initial_pose`；多机初始位置不能重叠（否则物理穿模） |
| 桥的 `@type@` 语法 | ROS 类型@桥@Gazebo 类型 | 方向由话题流向决定；类型必须两侧匹配 |
| `namespace=ns` | 桥也在命名空间内 | 保证桥出的话题带 `/go2_0/` 前缀，与 §13.2 的隔离一致 |

#### 验证：Gazebo 双机搬运

```bash
# 起完整 Gazebo 仿真：双 Go2 + 刚性杆负载 + 搬运任务
ros2 launch mini_multibot_bringup bringup_sim.launch.py \
  team:=team_2_go2.yaml world:=dual_go2_carry.sdf

# 触发搬运任务：把负载从 A 搬到 B（5 米外）
ros2 topic pub /shared/carry_goal geometry_msgs/Point "{x: 5.0, y: 0.0, z: 0.0}" --once

# 实时监控搬运质量（对应评分标准：跟踪误差 < 5cm）
ros2 run mmb_eval track_monitor   # 打印负载实际轨迹 vs 期望轨迹的误差
```

**成功标志**（对应第 13 章评分标准）：负载平稳搬运 5 米，跟踪误差 < 5cm，两个 Go2 全程保持对负载的协调受力（ADMM 协调生效）。如果负载剧烈摆动或某 Go2 "甩锅"（不出力），回 §13.3 检查 ADMM 的 rho。

### Crazyswarm2 路线：空中蜂群仿真

Crazyswarm2 是专为 Crazyflie 多旋翼蜂群设计的 ROS 2 栈（IMRCLab 出品）。它的杀手锏是**仿真与真机无缝切换**——同一套代码，改一个 `backend` 参数就能从仿真切到真实 Crazyflie 蜂群飞行。这对应本章的"空中蜂群线"。

#### 配置详解：crazyflies.yaml

Crazyswarm2 的核心配置是 `crazyflies.yaml`——它描述蜂群里有哪些无人机（类比 §13.1 的 `team_*.yaml`）：

```yaml
# crazyflies.yaml —— 描述一个 8 机 Crazyflie 蜂群
robots:
  cf_0:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E700      # 真机的无线电地址（仿真时忽略）
    initial_position: [0.0, 0.0, 0.0]
    type: cf21                            # Crazyflie 2.1
  cf_1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701
    initial_position: [0.5, 0.0, 0.0]
  # ... cf_2 到 cf_7

all:
  firmware_logging:
    enabled: true                         # 把固件日志流成 ROS 2 话题
  broadcasts:
    num_repeats: 15                       # 广播指令重发次数（无线可靠性）
```

关键配置项讲解：

| 配置项 | 含义 | 仿真 vs 真机 |
|--------|------|-------------|
| `enabled` | 是否启用该机 | 调试时可禁用部分机，先小规模验证 |
| `uri` | 真机无线电地址 | 仿真时被忽略，真机时必须和硬件 dongle/地址匹配 |
| `initial_position` | 初始位置 | 仿真即出生点；真机必须和动捕系统标定的实际位置一致 |
| `broadcasts.num_repeats` | 广播重发次数 | 真机用——无线丢包时重发保证指令到达，蜂群越大越重要 |

#### 仿真与真机的统一接口

Crazyswarm2 最精妙的设计——同一套 Python 脚本，仿真和真机都能跑：

```python
# 用 Crazyswarm2 的 crazyflie_py 控制蜂群（仿真/真机通用）
from crazyflie_py import Crazyswarm


def run_swarm_formation():
    # 初始化蜂群（backend 由 launch 参数决定：sim 或 cflib）
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 全体起飞（这就是把本章编队栈接进 Crazyswarm2 的入口）
    allcfs.takeoff(targetHeight=1.0, duration=2.5)
    timeHelper.sleep(3.0)

    # 把 Mini-MultiBot 的编队几何（§13.5）应用到蜂群
    from mmb_formation.formation_keeper import FormationGeometry
    geom = FormationGeometry()
    center = [0.0, 0.0, 1.0]
    for i, cf in enumerate(allcfs.crazyflies):
        target = geom.desired_position(i, center[:2], yaw=0.0)
        cf.goTo([target[0], target[1], 1.0], yaw=0.0, duration=3.0)
    timeHelper.sleep(3.5)

    # 降落
    allcfs.land(targetHeight=0.04, duration=2.5)
```

```bash
# 仿真后端启动（无需真机）
ros2 launch crazyflie launch.py backend:=sim
# 跑编队脚本
ros2 run mmb_formation swarm_formation_demo

# 真机后端：改一个参数 backend:=cflib（需 Crazyradio dongle + 动捕）
# ros2 launch crazyflie launch.py backend:=cflib
```

> **本质洞察**：Crazyswarm2 的 `backend:=sim/cflib` 切换，是"仿真与真机共享代码"的典范实现——它把"机器人在哪、怎么动"抽象成统一的 `Crazyswarm` API，底层是仿真器还是真实无线电对上层完全透明。这正是 Mini-MultiBot 追求的目标：**上层算法（编队、协调）不应该知道自己在仿真还是真机上跑**。把这个边界设计好，你的代码就能"仿真里调好，一键上真机"——这是工程价值的核心。

### 两套仿真器的统一适配

Mini-MultiBot 通过一个**仿真接口抽象层**，让同一套上层代码既能驱动 Gazebo 也能驱动 Crazyswarm2：

```python
# mmb_sim 的接口抽象——上层只认这个接口，不管底层是谁
class SimInterface:
    """仿真接口抽象。Gazebo 后端和 Crazyswarm2 后端都实现这个接口，
    上层（编队/协调）只调这些方法，不关心具体仿真器。"""

    def get_state(self, agent_id):
        """返回 agent 的状态（位姿+速度）。Gazebo 从 odom 话题取，
        Crazyswarm2 从动捕/状态估计取。"""
        raise NotImplementedError

    def send_command(self, agent_id, cmd):
        """下发控制指令。Gazebo 发 cmd_vel，Crazyswarm2 调 cf.cmdVelocity。"""
        raise NotImplementedError
```

这个抽象层是"仿真与真机共享代码"在 Mini-MultiBot 里的具体落地——上层的编队控制器调 `send_command`，至于这个指令是发给 Gazebo 的 `/go2_0/cmd_vel` 还是 Crazyswarm2 的某个 Crazyflie，由具体后端实现决定。换仿真器 = 换 `SimInterface` 的实现，上层零修改。

### ⚠️ 常见陷阱

```
🔧 配置陷阱：Gazebo 多机 spawn 初始位置重叠，机器人开局就穿模爆炸
   错误做法：YAML 里几个机器人的 initial_pose 写得太近（或都写 0,0,0）
   现象：仿真一启动机器人就互相穿透、被物理引擎弹飞，状态瞬间 NaN
   根本原因：Gazebo 的物理引擎检测到初始重叠，会施加巨大的分离力
   正确做法：初始位置间距 ≥ 机器人尺寸 + 裕度。多机 spawn 前在 YAML
            里检查所有 initial_pose 两两距离。检查：spawn 后机器人应
            静止在各自位置，无弹跳
```

```
🔧 配置陷阱：ros_gz 桥的消息类型两侧不匹配，话题桥了但收不到数据
   错误做法：桥语法写成 .../cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
            但 Gazebo 端实际期望的是别的类型
   现象：ros2 topic list 看到桥出的话题，但 echo 没数据，机器人不动
   根本原因：ros_gz 桥要求 ROS 类型和 Gazebo 类型严格对应，写错一侧
            桥静默失败（不报错但不转发）
   正确做法：查 ros_gz_bridge 文档确认类型映射表，用 ign topic -l 看
            Gazebo 端实际话题类型。检查：桥起来后 ros2 topic echo 应有数据
```

```
💡 概念误区：以为 Crazyswarm2 仿真通过了就等于真机能飞
   新手想法："Crazyswarm2 仿真里 8 机编队飞得很好，切 cflib 应该直接能飞"
   实际上：仿真后端用理想质点动力学，没有真实的电池电压衰减、电机不一致、
          无线丢包、动捕遮挡、气流扰动。真机飞行还要处理这些
   正确理解：Crazyswarm2 的仿真验证的是"算法逻辑和编队几何对不对"，
            不验证"真实飞行的鲁棒性"。切真机前要：先单机试飞、再 2-3 机
            小规模、确认动捕标定和无线可靠性后才上全蜂群（§13.8 详述）
```

### 练习

1. **【集成测试】** 在 Gazebo 里跑通"Go2+UAV 联合巡检"（异构任务）：Go2 地面巡检，UAV 空中俯瞰，二者通过 §13.2 通信层共享发现的目标。验收标准：UAV 发现一个目标点后，通过话题告知 Go2，Go2 自主导航过去。这验证了第 6 章异构协同在系统里的落地。

2. **【性能调优】** 测试 Crazyswarm2 的扩展性：从 4 机逐步加到 16、32 机，记录仿真的实时率（real-time factor）。验收标准：找到你的机器上"仿真还能实时跑"的最大机数，并与 Gazebo 的最大机数对比，验证"Crazyswarm2 可仿更大规模"的结论。

3. **【综合思考】** 你现在有 Gazebo 和 Crazyswarm2 两套仿真器接进了系统。如果要验证一个"100 架无人机的大规模编队避障"算法，和一个"双足机器人协同搬运易碎品（需精确接触力）"算法，分别该用哪个仿真器？为什么？如果两个都想验证，系统架构上要怎么设计才能复用大部分代码？（结合本节的 SimInterface 抽象回答。）


## §13.8 实机部署与系统调试 ⭐⭐⭐⭐

> **这一节解决什么问题**：仿真跑通了（§13.7），但仿真到真机有一道鸿沟。多机系统的 Sim2Real 比单机更难——不只是单机的动力学差异，还多了**时间同步、多机坐标系标定、网络配置**这些"多机特有"的脏活。本节讲透从仿真到真机的迁移路径，并给出多机系统特有的**三层故障定位方法**——这是 capstone 真正能"演示"而非"只在仿真里跑"的最后一公里。

### 模块功能与在系统中的位置

实机部署不是一个"模块"，而是把整个系统从开发工作站**分发到多台真实机器人**的过程。在架构上，仿真时所有节点跑在一台机器上；真机时，每个机器人的节点跑在它自己的板载计算机上，通过真实网络通信。这是对前七节所有工作的"终极考验"——任何隐藏的耦合、任何对"单机环境"的隐式假设，都会在这里暴露。

### 动机：仿真到真机，多了哪些"多机特有"的坑

单机的 Sim2Real 你在前面章节学过（动力学差异、传感器噪声、域随机化）。多机系统在此之上，还有三类**只有多机才有**的新问题——这是系统性分类：

| 类别 | 单机有吗 | 多机的新问题 | 后果 |
|------|---------|------------|------|
| **时间同步** | 否（单机一个时钟） | N 台机器各有时钟，不同步则 TF/数据时间戳错乱 | 协调用错时刻的数据，编队抖动 |
| **多机坐标系标定** | 否（单机一个坐标原点） | N 台机器的 `world` 必须是同一个，否则各active在各的世界 | 机器人去错位置、撞车 |
| **网络配置** | 否（单机进程内通信） | 真实无线网络的延迟/丢包/带宽限制 | 发现失败、状态延迟（第 2 章 τ 失稳） |

> **本质洞察**：仿真之所以"骗"过你，是因为它把这三件事**免费**做好了——仿真器里所有机器人共享一个进程的时钟（时间天然同步）、共享一个仿真世界坐标系（坐标天然对齐）、共享进程内通信（网络天然完美）。真机把这三个"免费的午餐"全部收回，你必须**显式地**把它们重新做好。这就是为什么"仿真跑通"和"真机能用"之间隔着一条鸿沟——鸿沟的宽度，正好是这三件被仿真隐藏的事。

### 迁移路径一：时间同步

多机系统的所有协调都依赖"大家对时间的理解一致"。TF 变换有时间戳、传感器数据有时间戳、ADMM 协调要知道"邻居的状态是哪一时刻的"。如果两台机器的时钟差 100ms，机器人 A 会用机器人 B "100ms 前"的状态做协调——这等效于第 2 章讲的通信时延 τ，足以让编队失稳。

解决方案是 **PTP（精确时间协议）或 NTP（网络时间协议）**：

```bash
# 方案 A：NTP（精度毫秒级，够多数多机系统用）
# 选一台机器（或路由器）做时间服务器，其余机器同步到它
sudo apt install -y chrony
# 在各机器人的 /etc/chrony/chrony.conf 里指向同一个时间源：
#   server 192.168.1.1 iburst    # 时间服务器（如主控机或路由器）
sudo systemctl restart chrony
chronyc tracking        # 查看同步状态，Offset 应在毫秒级

# 方案 B：PTP（精度微秒级，高速编队/蜂群推荐，需网卡支持）
sudo apt install -y linuxptp
sudo ptp4l -i eth0 -m   # 在所有机器上跑，自动选主时钟
```

| 方案 | 精度 | 适用 | 代价 |
|------|------|------|------|
| NTP (chrony) | 毫秒级 | 多数地面多机系统 | 几乎无（软件） |
| PTP (linuxptp) | 微秒级 | 高速蜂群、紧耦合编队 | 需网卡硬件时间戳支持 |

> **配置陷阱（提前预警）**：时间不同步是多机真机最隐蔽的杀手——它不会报错，只会让 TF 报 "extrapolation" 警告、让协调悄悄变差。部署的**第一件事**就是确认所有机器的时钟同步（`chronyc tracking` 的 offset 在毫秒级），别等编队抖了才想起来查时钟。

### 迁移路径二：多机坐标系标定

仿真里所有机器人天然在同一个 `world` 坐标系（仿真器定义）。真机里，每台机器人开机时只知道自己的 `odom`（从开机点算起），**不知道自己在全局世界的哪里**。必须有一个机制把各机器人锚定到同一个 `world`——这是 §13.2 多机 TF 树里 `world→go2_i/odom` 那个变换的真机来源。

三种常见方案：

| 方案 | 原理 | 适用 |
|------|------|------|
| **动捕系统（OptiTrack/Vicon）** | 外部相机直接测每机的全局位姿 | 室内、Crazyswarm2 蜂群（精度最高） |
| **共享地图定位（SLAM/AMCL）** | 各机在同一张预建地图里定位 | 室内地面机（无需外部设备） |
| **UWB/GPS-RTK** | 无线测距/卫星定位给全局坐标 | 室外、大范围 |

以动捕为例（Crazyswarm2 标准配置），标定的关键是让动捕的坐标系和系统的 `world` 重合：

```bash
# 动捕系统发布每个机器人的全局位姿到 /go2_i/pose（world 系下）
# 一个静态变换发布器把 odom 锚定到 world：
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --frame-id world --child-frame-id go2_0/odom
# 注意：实际部署中这个变换由"全局定位→odom 的差"动态计算，不是静态
```

> **概念误区（提前预警）**：新手常以为"每个机器人各自 SLAM 就行"——错。各自 SLAM 的机器人都在**各自的** `map` 坐标系里，原点不同、朝向不同，它们眼中的"(5, 0)"是完全不同的物理位置。多机必须有一个**共享的**全局坐标系，要么靠外部动捕，要么靠共享同一张地图。这是 §13.2 强调"`world` 必须是同一个"的真机含义。

### 迁移路径三：网络配置

仿真里通信是进程内的（零延迟、零丢包、无限带宽）。真机用真实无线网络，§13.2 讲的发现风暴、带宽限制全部变成现实问题。真机网络配置的要点：

- **用 Discovery Server**（§13.2 策略二）：真机多用无线，多播发现在无线网络上尤其脆弱（很多 WiFi AP 不可靠转发多播）。Discovery Server 用单播，远比多播在无线下可靠。
- **限制大数据话题跨网**：点云、图像这类大话题尽量本机处理，只把处理结果（如检测到的目标）跨网传。否则无线带宽瞬间被打满，状态话题延迟飙升（第 2 章 τ 失稳）。
- **QoS 配置**：状态话题用 `BEST_EFFORT`（丢了就丢，要最新的）、指令话题用 `RELIABLE`（必须送到）。多机系统的 QoS 不匹配会导致"话题列表里有但收不到"。

```python
# 真机部署：状态话题用 BEST_EFFORT（容忍丢包，要低延迟）
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

state_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,   # 状态：丢了用下一个最新的
    history=HistoryPolicy.KEEP_LAST, depth=1)    # 只留最新一帧
```

### 部署的渐进路径：别一步上全系统

多机真机部署最忌"仿真跑通就把全系统一把推上真机"。正确的是渐进式——这是工程实践的核心方法论：

```
第 1 步：单机真机     —— 一台真机跑通，验证底层驱动/控制接口对
   ↓
第 2 步：双机真机     —— 加第二台，验证时间同步+坐标标定+邻居通信
   ↓
第 3 步：小规模(3-4)  —— 验证协调内核(ADMM/共识)在真实网络下收敛
   ↓
第 4 步：全规模       —— 逐步加到目标机数，监控网络是否成为瓶颈
```

每一步都要**确认通过再进下一步**。在第 2 步暴露的时间同步问题，远比在第 4 步全系统里暴露好查——机器越少，故障越容易定位。

### 多机系统特有的三层故障定位

多机系统出问题时，故障可能在任何一层。盲目调试会浪费大量时间。正确的方法是**自顶向下三层定位**——这是本章最实用的调试方法论，也是 §13.1 健康监控节点的用武之地：

```
       多机系统故障定位决策树
              │
   ┌──────────┴──────────┐
   │ 第一层：话题级        │ ros2 topic hz/echo/info
   │ "数据流通吗？"        │
   └──────────┬──────────┘
              │ 话题没数据/频率不对？
       ┌──────┴───────┐
   是 ↓               ↓ 否（话题正常但行为不对）
┌──────────┐    ┌──────────────┐
│第二层:节点 │    │第三层:算法    │
│"节点活吗?" │    │"逻辑对吗?"    │
│ros2 node  │    │看协调残差/    │
│list/info  │    │obs一致性/     │
│           │    │CBF偏离        │
└──────────┘    └──────────────┘
```

**第一层：话题级——数据流通吗？**

```bash
# 检查关键话题的频率（频率不对 = 上游节点有问题）
ros2 topic hz /go2_0/state         # 期望 50 Hz，明显偏低说明发布节点卡了
ros2 topic hz /go2_0/cmd_vel       # 期望控制频率，没数据说明控制器没输出

# 检查邻居连接（§13.2 的核心）
ros2 topic info /go2_1/state -v    # 看 Publisher/Subscription 数和 QoS
# 常见问题：QoS 不匹配 → 显示有 publisher 但 subscription 收不到
```

**第二层：节点级——节点活吗？**

```bash
ros2 node list                     # 期望的节点都在吗？少了说明某 agent 没起来
ros2 node info /go2_0/agent        # 看这个节点的发布/订阅/服务是否符合预期
# §13.1 练习 2 的健康监控节点在这里发威——它会主动报告哪个 agent 掉线
```

**第三层：算法级——逻辑对吗？**（话题和节点都正常，但行为不对）

这一层是多机特有的"协调正确性"诊断，用前面各节埋的诊断点：

| 症状 | 查什么 | 对应节 |
|------|--------|--------|
| 编队抖动/不收敛 | 共识/ADMM 残差是否下降 | §13.3 |
| 机器人去错位置 | 分配结果 assignment 语义对不对 | §13.4 |
| 机器人对着障碍横跳 | ORCA 互惠/tau 配置 | §13.5 |
| 学习策略行为乱 | obs 一致性、actor.eval() | §13.6 |
| 时间相关的诡异抖动 | 时钟同步 offset | §13.8 |

### 代码走读：系统级健康监控（部署必备）

把 §13.1 练习 2 的健康监控扩展成一个完整的部署期监控节点，它是三层定位的"自动化前哨"：

```python
# mmb_eval/mmb_eval/system_monitor.py
import time
import rclpy
from rclpy.node import Node
from mini_multibot_msgs.msg import AgentState


class SystemMonitor(Node):
    """系统级健康监控——部署期的'仪表盘'。订阅所有 agent 状态，
    检测掉线、时延、协调健康度，是三层故障定位的自动化前哨。
    """

    def __init__(self):
        super().__init__('system_monitor')
        self.n_agents = self.declare_parameter('n_agents', 4).value
        self.timeout = self.declare_parameter('timeout_sec', 1.0).value
        self._last_seen = {}          # agent_id -> 最后接收时间
        self._latency = {}            # agent_id -> 估计的状态时延
        for i in range(self.n_agents):
            self.create_subscription(
                AgentState, f'/go2_{i}/state', self._make_cb(i), 10)
        self.create_timer(1.0, self._check_health)

    def _make_cb(self, agent_id):
        def _cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            self._last_seen[agent_id] = now
            # 用消息时间戳估计端到端时延（依赖时间同步！）
            msg_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._latency[agent_id] = now - msg_t
        return _cb

    def _check_health(self):
        """每秒检查一次，打印仪表盘。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        for i in range(self.n_agents):
            if i not in self._last_seen:
                self.get_logger().warn(f'agent {i}: NEVER SEEN (节点没起来?)')
            elif now - self._last_seen[i] > self.timeout:
                self.get_logger().error(
                    f'agent {i}: LOST ({now - self._last_seen[i]:.1f}s 无状态)')
            elif self._latency.get(i, 0) > 0.2:
                self.get_logger().warn(
                    f'agent {i}: 高时延 {self._latency[i]*1e3:.0f}ms '
                    f'(网络/时钟问题?)')
```

这个监控节点把三层定位的前两层自动化了：**没起来的节点**（第二层）和**掉线/高时延的话题**（第一层）它都会主动报警。注意 `_latency` 的估计**依赖时间同步**——如果时钟没同步，这个时延数字会是错的（甚至负的），这本身就是"时钟没同步"的信号。部署时常开这个监控，比出了问题再手敲 `ros2 topic hz` 高效得多。

### ⚠️ 常见陷阱

```
🔧 配置陷阱：真机部署后忘配时间同步，TF 频繁报 extrapolation 警告
   错误做法：仿真跑通直接上真机，没在各机器人间配 NTP/PTP
   现象：TF 不断报 "lookup would require extrapolation into the future/past"，
        编队时好时坏，协调质量随机波动且无法复现
   根本原因：N 台机器时钟不同步，带时间戳的 TF/状态在不同机器间对不齐，
            tf2 无法在错位的时间线上插值
   正确做法：部署第一步配置 chrony/ptp4l，确认 chronyc tracking 的 offset
            在毫秒级。检查：所有机器 date +%s%N 的差应 < 时延裕度
```

```
🔧 配置陷阱：各机器人各自 SLAM，没有共享 world 坐标系，机器人去错位置
   错误做法：每台机器人独立跑 SLAM，以为坐标自然对齐
   现象：发"去 (5,0)"的指令，机器人们朝不同方向跑——因为各自的 (5,0)
        是不同物理点
   根本原因：独立 SLAM 的每台机器在各自的 map 坐标系，原点/朝向不同，
            没有共享的全局参考系
   正确做法：用动捕、共享预建地图、或 UWB 提供统一 world 系，
            发布一致的 world→go2_i/odom 变换。检查：所有机器人对
            同一个全局点的 TF 查询应得到一致的物理位置
```

```
💡 概念误区：以为多机调试和单机一样，盯着一个机器人看就行
   新手想法："某机器人行为不对，我盯着它的话题/日志调就行"
   实际上：多机系统的故障常是"涌现"的——单个机器人都正常，但它们
          交互出问题（如两机时钟差导致协调发散）。盯单机看不到交互层的 bug
   正确理解：多机调试要有"系统视角"——用 SystemMonitor 看全局健康度，
            用三层定位法（话题→节点→算法）自顶向下排查。很多多机 bug
            （时间不同步、坐标系错位、发现风暴）在单机视角下根本不存在，
            只在"多机交互"这个层面才显现
```

### 练习

1. **【集成测试】** 在仿真里模拟"时间不同步"：给某个 agent 的状态消息时间戳人为加 200ms 偏移，观察编队协调如何劣化、SystemMonitor 是否报高时延。验收标准：能复现"时钟偏移→协调劣化"的因果，并通过 SystemMonitor 的时延报警定位到问题 agent。这让你在没有真机时也能体会时间同步的重要性。

2. **【管线搭建】** 完善 SystemMonitor，加一个"协调健康度"指标：订阅 ADMM 的一致性残差 ||x_i - z||，当残差长期不收敛（如连续 5 秒 > 阈值）时报警 "coordination not converging"。验收标准：人为把 admm_rho 设到发散值，监控应报警，把第三层（算法级）故障也自动化。

3. **【综合思考/Capstone 收官】** 设计一个完整的"仿真→真机"部署检查清单（deployment checklist），覆盖本节三条迁移路径（时间同步、坐标标定、网络配置）和渐进部署四步。这个清单应该让一个没做过多机部署的同学照着走就能避开本节所有陷阱。把它写进你的项目 README——这是 capstone 交付物的一部分（对应评分标准的"文档"项）。


## 本章常见误解汇总

把全章散落的概念误区集中成一张表，便于回查。这些误区的共同特点是——**它们在单机视角下都不存在，只在"多机系统集成"这个层面才显现**：

| 误解 | 正确理解 | 出处 |
|------|---------|------|
| "把单机代码复制 N 份就是多机系统" | 多机价值在协调而非并行；必须显式区分 agent-local 和 system-level，后者才是灵魂 | §13.1 |
| "加机器人只是多几个话题" | 真正的瓶颈是 CPU 核数×仿真进程数、DDS 发现流量 $O(N^2)$ | §13.1/13.2 |
| "Discovery Server 是数据中转，会成带宽瓶颈" | 它只代理发现（通讯录），数据仍点对点；负载几乎为零 | §13.2 |
| "ADMM 有 coordinator 就是集中式" | coordinator 只聚合不决策、不需全局模型，是 $O(N)$ 轻量协调而非 $O(N^3)$ 联合求解 | §13.3 |
| "任务分配做一次就够了" | 分配是事件驱动、需重入的（终身 MAPF）；任务/机器人状态变化要重分配 | §13.4 |
| "有了 ORCA 就不需要全局路径规划" | 局部避障无全局视野、会死锁；MAPF（全局）和 ORCA（执行兜底）互补 | §13.5 |
| "有了 MARL 就不要传统规控" | 最稳健是混合：MARL 做难建模的高层协同，CBF/ORCA 做有保证的底层安全 | §13.6 |
| "仿真训练好的策略直接部署就能用" | obs 顺序/归一化/邻居排序只要差一点策略就失效，且不报错——比动力学 gap 更隐蔽 | §13.6 |
| "Crazyswarm2 仿真过了就等于真机能飞" | 仿真验证算法逻辑，不验证真实鲁棒性（电池/无线/气流） | §13.7 |
| "各机器人各自 SLAM 坐标就对齐了" | 各自 SLAM 在各自 map 系，原点不同；多机必须有共享 world 系 | §13.8 |
| "多机调试和单机一样盯一个机器人看" | 多机故障常是涌现的（交互层），单机视角看不到；要用系统视角+三层定位 | §13.8 |

---

## 本章小结

本章是多机器人方向的 capstone——它不教新理论，而是把前十二章的工具焊成一个完整系统。一条主线贯穿全章：**一个真实的多机系统，是按"通信→协调→决策→运动→学习→部署"这条数据流，由一层层可独立测试、可替换的模块焊接而成的**。每一层都有它要解决的核心问题和特有的陷阱，而集成的艺术在于把这些层的接口设计干净，让数据顺畅地从一层流到下一层。

你应该带走的核心认知有三个。其一，**多机系统的难点不在算法层而在系统层**——算法（共识、ADMM、CBBA、MAPPO）你前面都学过了，本章真正难的是把它们组织成能扩展、能调试、能部署的系统：目录怎么分、通信怎么联、时间怎么同步、坐标怎么对齐、故障怎么定位。其二，**分层与解耦是驾驭复杂度的唯一武器**——无论是运动栈的三层（编队/协调/避障）、还是混合栈的三层（学习/安全/控制）、还是部署的三层故障定位，本质都是"把不同时间尺度、不同职责的事分开，各自用最合适的方法"。其三，**仿真与真机的鸿沟，是仿真免费提供的三件事（时间同步、坐标对齐、完美通信）在真机被收回**——理解这一点，你就知道部署时该显式补做什么。

### 术语速查表

| 术语 | 英文 | 定义 |
|------|------|------|
| 命名空间 | Namespace | ROS 2 给话题/节点加前缀以隔离多个机器人实例的机制 |
| 域隔离 | Domain ID Isolation | 不同 DDS domain ID 的节点完全互不可见，多机的第一层隔离 |
| 发现服务器 | Discovery Server | Fast DDS 的集中式发现机制，把发现流量从 $O(N^2)$ 降到 $O(N)$ |
| 域桥 | Domain Bridge | 在隔离的 DDS domain 之间显式桥接指定话题的工具 |
| 协调内核 | Coordination Kernel | 把共识/ADMM 落成 ROS 2 节点的模块，系统的"协调大脑" |
| 共识 | Consensus | 多机靠邻居通信对某量达成一致的协议（第 2 章） |
| 罚参数 | Penalty Parameter ρ | ADMM 中控制一致性强度的超参（第 2 章） |
| 任务分配 | Task Allocation | 决定"哪个机器人做哪个任务"的离散决策（第 3 章） |
| 编队几何 | Formation Geometry | 定义理想队形与各机器人理想位置的层 |
| ORCA | Optimal Reciprocal Collision Avoidance | 基于速度障碍互惠半平面的局部避障算法 |
| 控制屏障函数 | Control Barrier Function (CBF) | 保证系统前向不变于安全集的约束，给任意控制器加安全保证 |
| 安全滤波 | Safety Filter | 把标称动作投影到安全动作集的模块（ORCA/CBF 都是） |
| CTDE | Centralized Training Decentralized Execution | MARL 范式：训练用全局信息，执行各自用局部观测（第 10 章） |
| 标称动作 | Nominal Action | 上层（策略/规则）给出的期望动作，过安全滤波前的值 |
| 时间同步 | Time Synchronization | 让多机时钟一致（NTP/PTP），多机协调的前提 |
| 实时率 | Real-Time Factor | 仿真时间与真实时间之比，衡量仿真能否实时运行 |

### 知识点总表

| 编号 | 知识点 | 核心要点 | 对应节 | 难度 |
|------|--------|---------|--------|------|
| 1 | 项目骨架组织 | 按系统角色分包：agent-local / system-level / config | §13.1 | ⭐⭐ |
| 2 | 配置即代码 | 用 YAML 描述队伍拓扑，代码零修改换队伍 | §13.1 | ⭐⭐ |
| 3 | 循环 launch N agent | 命名空间隔离 + remapping 连接邻居 | §13.1 | ⭐⭐⭐ |
| 4 | DDS 发现选型 | 多播/域隔离/Discovery Server 三方案权衡 | §13.2 | ⭐⭐⭐ |
| 5 | 多机 TF 树 | 每机自己的 odom→base，共享 world 系 | §13.2 | ⭐⭐⭐ |
| 6 | 共识落 ROS 2 | 迭代公式→定时器回调，异步"缺谁跳谁" | §13.3 | ⭐⭐⭐⭐ |
| 7 | ADMM 落 ROS 2 | x 更新在 agent、z 更新在 coordinator、u 更新在 agent | §13.3 | ⭐⭐⭐⭐ |
| 8 | 通信模式选型 | 分配用 Service（事件）、结果用 Topic（状态） | §13.4 | ⭐⭐⭐ |
| 9 | 算法复用 | import 第 3 章 planning，不重写 | §13.4 | ⭐⭐ |
| 10 | 三层运动栈 | 编队几何→ADMM 协调→ORCA 兜底，慢决策包络快反应 | §13.5 | ⭐⭐⭐ |
| 11 | ORCA 互惠避让 | 各让一半责任，避免对向振荡 | §13.5 | ⭐⭐⭐ |
| 12 | 训练↔部署解耦 | 训练产出权重，部署只做推理；两个 Python 环境 | §13.6 | ⭐⭐⭐⭐ |
| 13 | obs 精确复现 | 顺序/归一化/邻居排序逐项对齐训练 | §13.6 | ⭐⭐⭐⭐ |
| 14 | CBF 安全滤波 | 前向不变性保证，套在学习策略外 | §13.6 | ⭐⭐⭐⭐ |
| 15 | 两套仿真器 | Gazebo 高保真少机 vs Crazyswarm2 轻量大规模 | §13.7 | ⭐⭐⭐ |
| 16 | 仿真接口抽象 | SimInterface 让上层代码不认底层仿真器 | §13.7 | ⭐⭐⭐ |
| 17 | Sim2Real 三件事 | 时间同步、坐标标定、网络配置（仿真免费、真机收回） | §13.8 | ⭐⭐⭐⭐ |
| 18 | 渐进部署 | 单机→双机→小规模→全规模，每步验证 | §13.8 | ⭐⭐⭐ |
| 19 | 三层故障定位 | 话题级→节点级→算法级，自顶向下 | §13.8 | ⭐⭐⭐⭐ |

---

## 累积项目：Mini-MultiBot 完整交付

> 这是整个多机器人方向累积项目的**终点**——前十二章你往这个项目里加的每一个模块（第 2 章的共识、第 3 章的 `planning/`、第 4 章的 ADMM、第 10 章的 MAPPO），在本章全部汇入 Mini-MultiBot。本节给出完整的里程碑式 checklist，照着走就能交付一个开源 capstone。

### 项目里程碑 Checklist

按周组织，每个里程碑有明确的验收标志。**强烈建议严格按顺序推进，每个里程碑确认通过再进下一个**——这正是 §13.8 渐进部署方法论在项目管理上的应用。

**第 1 周：地基与通信（§13.1-13.2）**

- [ ] **M1.1 环境就绪**：按"环境配置指南"装好三层栈，Quick Start 的双 talker 跑通（两套独立话题）。
- [ ] **M1.2 骨架立起**：建好 `mini_multibot` 工作空间，按系统角色分包；骨架空转（stub agent）能起 N 个命名空间节点。验收：`ros2 node list` 看到 `/go2_0/agent`、`/go2_1/agent`、`/coordinator`。
- [ ] **M1.3 配置即代码**：写好 `team_2_go2.yaml`，launch 循环读 YAML 起 agent；改成 `team_4_go2.yaml` 不动代码能起 4 机。验收：邻居话题连接（`ros2 topic info` 订阅计数 > 0）。
- [ ] **M1.4 通信选型**：默认多播跑通 4 机；配通 Discovery Server，对比启动时间。验收：两种方案都能起全部节点，能说出发现流量差异。
- [ ] **M1.5 多机 TF**：每机发布带 `frame_prefix` 的 TF，`world→go2_i/odom` 锚定。验收：`ros2 run tf2_tools view_frames` 看到正确的多机 TF 树，无打架。

**第 2 周上半：协调与决策（§13.3-13.4）**

- [ ] **M2.1 共识落地**：实现 `ConsensusNode`，4 机环形拓扑收敛到初值平均。验收：4 机共识值收敛到 1.5，改全连接拓扑收敛变快。
- [ ] **M2.2 ADMM 落地**：实现本地 solver + coordinator，双机对共享变量协调收敛。验收：一致性残差 ||x_i - z|| 下降到 1e-2，扫 rho 找到最优值。
- [ ] **M2.3 任务分配集成**：把第 3 章算法包成 `AllocateTasks` 服务，匈牙利/CBBA/贪婪可运行时切换。验收：服务返回合理分配，匈牙利代价 < 贪婪代价。
- [ ] **M2.4 动态重分配**：机器人掉线触发重分配，任务转给健康机器人。验收：kill 一个 agent，1-2 秒内任务重分配。

**第 2 周下半：运动、学习与验证（§13.5-13.7）**

- [ ] **M3.1 三层运动栈**：编队几何 + 目标融合 + ORCA 串通，4 机菱形编队过障碍。验收：保持队形移动、无碰撞、到达后恢复队形。
- [ ] **M3.2 任务 A（MPC 方案）**：Gazebo 双 Go2 协同搬运，ADMM 协调耦合力。验收：负载平稳搬运 5 米，跟踪误差 < 5cm（对应评分标准）。
- [ ] **M3.3 任务 B（MARL 方案）**：训练 MAPPO（独立 venv），部署进系统（obs 一致性验证通过），CBF 兜底。验收：搬运成功率 > 80%，无碰撞。
- [ ] **M3.4 MPC vs MARL 对比**：在成功率/计算时间/泛化性/安全性/通信量五维度对比，画 radar chart。验收：radar chart 完成，每维度有定量数据和解释。
- [ ] **M3.5 蜂群验证（可选）**：Crazyswarm2 跑通多机编队（仿真），测扩展性上限。验收：找到能实时仿真的最大机数。

**收官：部署与文档（§13.8）**

- [ ] **M4.1 部署 checklist**：写出覆盖时间同步/坐标标定/网络配置的部署清单。
- [ ] **M4.2 系统监控**：`SystemMonitor` 自动化前两层故障定位 + 协调健康度报警。
- [ ] **M4.3 泛化验证**：至少一种方案验证泛化性（更换负载质量/形状）。验收：换负载后成功率不显著下降。
- [ ] **M4.4 开源交付**：README（架构图 + 关键设计决策 + 部署清单）+ 代码开源到 GitHub。验收：他人 clone 后照 README 能跑通 Quick Start。

### 交付物对照评分标准

| 评分项 | 权重 | 对应里程碑 |
|--------|------|-----------|
| 系统完整性（MPC + MARL 两路可运行） | 30% | M3.2 + M3.3 |
| 任务性能（搬运成功率 > 80%，误差 < 10cm） | 25% | M3.2 + M3.3 |
| 对比分析（MPC vs MARL 定量对比） | 25% | M3.4 |
| 文档（README + 架构图 + 设计决策） | 20% | M4.1 + M4.4 |

### 项目目录最终形态

完整项目代码保存在独立仓库 `mini_multibot/`（建议结构见 §13.1 骨架）。一个健康的最终交付应该是：任何人 clone 后，`colcon build` → `source` → `ros2 launch mini_multibot_bringup bringup_sim.launch.py` 三步就能看到机器人动起来；README 第一屏就是架构图和 Quick Start；每个 `mmb_*` 包有清晰的单一职责。这就是 capstone 的价值——不只是"我会算法"，而是"我能交付一个别人能用、能读懂、能扩展的系统"。

---

## 延伸阅读

按主题分类，标注难度（⭐ 入门 / ⭐⭐ 进阶 / ⭐⭐⭐ 深入）：

**多机 ROS 2 系统搭建**
- ROS 2 官方多机器人导航教程（Nav2 `cloned_multi_tb3_simulation`）⭐ — 多机命名空间与 launch 的权威范例
- Arshad Mehmood 的 TurtleBot3 多机 Gazebo 系列（Medium）⭐⭐ — 多机 spawn、TF、物理引擎配置的实战细节
- "Multi Robot Coordination in ROS 2: From Namespace Isolation to Fleet Management"（Medium）⭐⭐ — 从命名空间到舰队管理的演进

**DDS 与通信扩展性**
- ROS 2 设计文档 "ROS on DDS"（design.ros2.org）⭐⭐ — 理解 ROS 2 为什么选 DDS、发现机制的本质
- Husarnet "Scalable Distributed Robot Fleet With Fast DDS Discovery Server" ⭐⭐ — Discovery Server 的配置与扩展性实测
- Husarnet "Bridge Remote DDS Networks With a DDS Router" ⭐⭐⭐ — 跨网络舰队的话题过滤与桥接

**蜂群仿真与实机**
- Crazyswarm2 官方文档（imrclab.github.io/crazyswarm2）⭐⭐ — 蜂群仿真/实机的完整栈
- CrazySim（gtfactslab/CrazySim）⭐⭐⭐ — 带 Gazebo 物理的 Crazyflie 仿真，介于两套仿真器之间
- "ROS2swarm - A ROS 2 Package for Swarm Robot Behaviors"（arXiv 2405.02438）⭐⭐ — 群体行为的 ROS 2 包

**分布式协调与编队**
- "Decentralized multi-robot formation control ... based on path planning"（Intelligent Service Robotics 2024）⭐⭐⭐ — 编队+ORCA+非凸障碍
- "Safety-Critical Formation Control of Non-Holonomic Multi-Robot Systems"（arXiv 2406.13707）⭐⭐⭐ — CBF 编队的安全保证
- DiRAC "Distributed Robot Awareness and Consensus"（arXiv 2510.16850）⭐⭐⭐ — ROS 2 中分布式共识的架构可扩展性

**前置章节回顾**（系统集成的理论来源）
- 本方向第 2 章「共识算法与分布式优化」— §13.3 协调内核的数学源头
- 本方向第 3 章「任务分配与路径规划」— §13.4 分配集成复用的算法
- 本方向第 4 章「分布式 MPC 多足编队」— §13.5 重型编队方案
- 本方向第 10-12 章「MARL 及混合」— §13.6 学习策略的训练源头

---

## 本章与后续章节的关系

本章是多机器人方向 Part 的**收官**，但不是机器人学习的终点。它向后延伸到几个方向：

| 后续方向 | 关系 | 本章铺垫的知识点 |
|---------|------|----------------|
| 第 14 章「附录」 | 本章用到的工具/数据集/扩展资源的汇总 | 整章的工具链 |
| 真实产品级多机系统 | 本章是"教学级"系统，产品级要加：容错恢复、安全认证、运维监控 | §13.8 的三层故障定位、SystemMonitor |
| 大规模蜂群（100+） | 本章的通信选型（Discovery Server）和轻量仿真（Crazyswarm2）是入口 | §13.2 发现扩展性、§13.7 蜂群仿真 |
| Sim2Real 深化 | 本章讲了多机特有的三件事，单机的域随机化要结合进来 | §13.8 迁移路径 |
| 多机 SLAM / 协同感知 | 本章假设定位已解决（动捕/共享地图），协同感知是独立的大主题 | §13.8 共享 world 系 |

**给读完本章的你**：你现在手里有一个完整的、自己理解每一行的多机系统。它不是终点，而是一个**平台**——往后你想研究任何多机课题（新的协调算法、新的学习方法、新的应用场景），都可以把它接进这个系统验证，而不必从零搭基础设施。这正是 capstone 的长远价值：它给了你一个"做多机研究的起跑线"。

---

## 🔧 故障排查手册

> 本手册聚焦多机系统特有的故障，重点覆盖环境配置、通信、和"多机交互"层面的问题。按"症状→可能原因→排查步骤→相关章节"组织。

### 故障 1：Quick Start 里两个 talker 的话题串了/只看到一套

| 项 | 内容 |
|----|------|
| **症状** | `ros2 topic list` 只看到一套话题，或 `echo` 时两个 talker 的计数混在一起 |
| **可能原因** | ① 两个终端的 `ROS_DOMAIN_ID` 不一致；② 没设命名空间/remapping；③ 系统里有别人的同名节点串扰 |
| **排查步骤** | 1. 两个终端都 `echo $ROS_DOMAIN_ID` 确认一致（都应是 42）；2. 确认启动命令带了 `-r __ns:=/robot_0`；3. `ros2 node list` 看是否有意料外的节点 |
| **相关章节** | §环境配置（domain ID）、§13.1（命名空间） |

### 故障 2：加到 8-16 机，节点随机掉线、启动极慢

| 项 | 内容 |
|----|------|
| **症状** | 少机正常，多机时启动 30+ 秒、`ros2 node list` 卡顿、节点随机消失 |
| **可能原因** | DDS 多播发现风暴——发现流量随节点数 $O(N^2)$ 增长，网络被淹没 |
| **排查步骤** | 1. `htop` 看是否有进程的发现线程吃满 CPU；2. 切换到 Discovery Server（`ROS_DISCOVERY_SERVER`）；3. 确认所有终端都设了 `ROS_DISCOVERY_SERVER`（漏设的看不到节点） |
| **相关章节** | §13.2（DDS 发现选型） |

### 故障 3：邻居通信不通——节点都在但协调不动

| 项 | 内容 |
|----|------|
| **症状** | `ros2 node list` 节点齐全，但共识/ADMM 不收敛，机器人各动各的 |
| **可能原因** | ① 邻居话题 remapping 用了相对名（带上自己命名空间前缀）；② QoS 不匹配（有 publisher 无 subscription 收到）；③ 闭包延迟绑定 bug（所有邻居存到同一 id） |
| **排查步骤** | 1. `ros2 topic info /go2_1/state -v` 看订阅计数和 QoS；2. 确认 remapping 用绝对名（前导 /）；3. 打印每个邻居回调实际收到的 agent_id |
| **相关章节** | §13.2（邻居 remapping、QoS）、§13.3（闭包陷阱） |

### 故障 4：真机部署后 TF 频繁报 extrapolation、编队时好时坏

| 项 | 内容 |
|----|------|
| **症状** | TF 报 "lookup would require extrapolation"，编队质量随机波动，无法复现 |
| **可能原因** | 多机时钟不同步——带时间戳的 TF/状态在不同机器间对不齐 |
| **排查步骤** | 1. 各机器 `chronyc tracking` 看 offset 是否毫秒级；2. 对比各机器 `date +%s%N`；3. 配置 NTP/PTP 并重启同步服务；4. 用 SystemMonitor 看各 agent 时延是否异常 |
| **相关章节** | §13.8（时间同步） |

### 故障 5：仿真训练的策略部署后机器人乱动（不报错）

| 项 | 内容 |
|----|------|
| **症状** | 策略在训练时成功率高，部署后机器人行为混乱，但程序不报错 |
| **可能原因** | ① obs 构造与训练不一致（顺序/归一化/邻居排序）；② 忘记 `actor.eval()`；③ 归一化参数没随权重加载 |
| **排查步骤** | 1. 做 obs 一致性验证（训练侧 dump vs 部署侧 build 逐元素对比）；2. 确认加载后调了 `actor.eval()` + `torch.no_grad()`；3. 确认 obs_mean/obs_std 从 ckpt 加载 |
| **相关章节** | §13.6（obs 复现、eval 模式） |

### 故障 6：Gazebo 多机一启动就穿模爆炸

| 项 | 内容 |
|----|------|
| **症状** | 仿真启动机器人就互相穿透、被弹飞、状态变 NaN |
| **可能原因** | YAML 里多机 `initial_pose` 太近或重叠，物理引擎施加巨大分离力 |
| **排查步骤** | 1. 检查 YAML 所有 initial_pose 两两距离 ≥ 机器人尺寸+裕度；2. 拉开初始位置重新 spawn；3. 确认 SDF 模型尺寸与间距匹配 |
| **相关章节** | §13.7（Gazebo 多机 spawn） |

---

## 版本信息速查

本章所有代码的测试基准版本汇总（详见章首"版本兼容表"）：

| 工具/库 | 版本 | 用途 |
|---------|------|------|
| Ubuntu | 22.04 LTS | 操作系统 |
| ROS 2 | Humble Hawksbill | 核心中间件 |
| Python | 3.10 | ROS 2 节点 + 部署推理 |
| Fast DDS | 2.6.x | 默认 RMW + Discovery Server |
| Gazebo | Fortress (Ignition 6.x) | 地面异构机仿真 |
| ros_gz | (随 Humble) | Gazebo↔ROS 2 桥 |
| Nav2 | 1.1.x | 地面导航（扩展任务） |
| domain_bridge | (随 Humble) | 跨 domain 话题桥接 |
| Crazyswarm2 | 1.0a1 | 空中蜂群仿真/实机 |
| PyTorch | 2.1.0 | MARL 策略推理 |
| CUDA | 12.1 | GPU 训练（部署可纯 CPU） |
| NumPy | 1.24（<2！） | 数值计算（注意 ABI 兼容） |
| Gymnasium | 0.29.1 | 训练环境接口 |
| chrony | (apt) | NTP 时间同步 |
| linuxptp | (apt) | PTP 时间同步（高精度） |

> **版本锁定建议**：多机系统对版本错配敏感，建议在 `requirements.txt`（Python）和 `package.xml`（ROS 2 依赖）里**精确锁定到小版本**。尤其 NumPy 必须锁 `<2`，PyTorch 必须与 CUDA 匹配。整个团队（尤其真机部署时各机器人板载计算机）应统一版本——版本不一致的机器人可能因 DDS 序列化差异而无法通信。
