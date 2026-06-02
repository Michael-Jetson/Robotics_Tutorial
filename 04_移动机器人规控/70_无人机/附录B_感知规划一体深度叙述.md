## 附录B：感知-规划一体深度叙述

> → 对应主大纲章节：D6-D7（环境表示/感知引导规划与自主探索）

### 无人机感知规划、自主探索与环境表征算法十年演进全景报告

**核心结论**：2015–2025 年间，无人机规划领域完成了三次代际跃迁：从 **基于 OctoMap 的被动几何规避**，到 **FUEL/RACER 式的增量前沿 + 分层 TSP 主动探索**，再到 **Splat-Nav/SAFER-Splat 式的 3DGS 可微分表征规划**。贯穿这三代的技术主线有两条：**(1) 环境表征从稀疏八叉树→体素哈希 TSDF/ESDF→机器人中心滑窗栅格→各向异性椭球隐式场**；**(2) 规划范式从贪婪下一最优视点→分层路由 (ATSP/CVRP/MDVRP) + 多机去中心化协同→基于可微分渲染的端到端训练**。对于从 SLAM 转入规划的工程师，关键跨越点是理解 **ikd-Tree / voxblox ESDF / ROG-Map 滑窗栅格三种数据结构的查询语义**，以及 **yaw 作为独立决策变量** 所带来的感知–动作耦合问题。HKU MaRS Lab 的 FAST-LIO2 + ikd-Tree + ROG-Map + SUPER（Science Robotics 2025）已事实上构成华语圈无人机 SLAM + 规划的黄金堆栈；而 Stanford MSL 的 Splat-Nav (T-RO 2025) / SAFER-Splat / SOUS VIDE 则预示了 **"端到端可微分地图 + 控制"** 的下一代架构。

---

### 一、环境表征：从八叉树到高斯椭球的数据结构史

环境表征是一切规划的基石。它的核心指标有四个：**插入复杂度、查询复杂度、内存布局、是否支持距离场梯度**。下表是一张"选型决策图"，文后逐个展开。

| 代表系统 | 结构 | 查询 | 典型用途 | 许可 |
|---|---|---|---|---|
| **OctoMap** (Hornung 2013) | 指针式 8 叉树 + 剪枝 | O(log N) | 教学、小规模室内、基线 | BSD |
| **voxblox** (Oleynikova 2017) | 块哈希 TSDF + 双波前 ESDF | O(1) 块查询 | 有 GPU 预算不足、深度相机、需梯度 | Apache-2 |
| **nvblox** (Millane 2024) | CUDA 块哈希 + PBA ESDF | 片上 O(1) | Jetson、177$\times$ voxblox 速度 | Apache-2 |
| **Ewok Ring Buffer** (Usenko 2017) | 幂 2 环形缓冲 + 位与 | O(1) 位掩 | 纯局部、无 GPU | GPLv3 |
| **ROG-Map** (Ren 2024) | 机器人中心滑窗栅格 + 计数器膨胀 | O(1) 展平数组 | LiDAR 大场景高分辨率 | GPLv3 |
| **ikd-Tree** (Cai 2021) | 增量 kd 树 + 并行再平衡 | O(log n) kNN | 点云 SLAM 内插、碰撞查询 | GPLv2 |

#### OctoMap：概率八叉树的鼻祖

**引用**：Hornung, Wurm, Bennewitz, Stachniss, Burgard, *"OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees"*, Autonomous Robots 34(2): 189–206, 2013. **GitHub**: https://github.com/OctoMap/octomap，**约 2.2k stars**（已超出任务描述中的默认值），谷歌学术被引约 2900 次。

**核心贡献**：用 **八叉树 + 对数几率贝叶斯滤波** 实现概率 3D 占据图。每叶结点储存 log-odds 占据值，允许 **占据/空闲/未知** 三态显式表示（八叉树节点缺失即"未知"）。关键工程技巧是**剪枝**：八个子节点状态一致时合并为父节点，节省磁盘 (.bt 位流 2 bit/子)。

**C++ 模式**：模板化节点 `OcTreeBase<NODE,INTERFACE>`，用户可派生 `OcTreeNodeStamped`、`ColorOcTreeNode` 等；类似 PCL 的点类型扩展思路。`OcTreeKey` 用 `uint16_t[3]` 定点坐标键并提供哈希器；`KeyRay` 预分配避免射线投射时的反复 `new`。附带 `dynamicEDT3D` 做欧氏距离变换——这是 OctoMap 到规划世界的唯一"梯度接口"。

**与 SLAM 的关系**：OctoMap 本身不做位姿估计，要求前端提供位姿；是 ORB-SLAM、RGBDSLAM 最常见的后端可视化地图。**教学价值**：最佳的"体积表示入门"，log-odds 更新、射线投射、稀疏树剪枝都在一千多行代码内讲完，适合作为所有后续方法的性能基线。

#### voxblox：TSDF→ESDF 的 CPU 实时化

**引用**：Oleynikova, Taylor, Fehr, Siegwart, Nieto, *"Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning"*, IROS 2017. **GitHub**: https://github.com/ethz-asl/voxblox，实测 **约 1.6k stars**。

**核心贡献**由两部分组成。**第一**，"分组射线投射 (merged integrator)"：落入同一体素的点合并为一根加权射线，较 OctoMap 式单射线投射提速 5–10$\times$。**第二**，**准欧氏 ESDF 双波前**：下波前用优先队列从 TSDF 零穿越面向 26 邻接体素传播距离，每个体素只存一个 3 分量"父方向"而非完整父索引，节省内存；上波前 (raise queue) 处理被失效的父节点，Oleynikova 特意把两个波前**串行**执行以减少记账复杂度（Lau 2010 原版是交替的）。距离最大误差约 8%，对轨迹优化梯度足够精确。

**内存布局**：`Block<VoxelT>` 是一个 `16³ = 4096` 体素的稠密小数组，多个 `Block` 用 `std::unordered_map<Eigen::Vector3i, BlockPtr>` 哈希组装——**体素哈希**（Nießner 2013 风格）在 CPU 的实现。重要衍生：`c-blox`、`voxgraph` 加入子图 + 位姿图以处理 SLAM 漂移。

**C++ 模式**：重度模板（`Layer<VoxelT>`、`Block<VoxelT>`、`Integrator<VoxelT>`）；Protobuf 序列化；`ThreadSafeIndex` 管理多线程集成；Google style 代码规范 + `ethz-asl/linter` 静态检查。**教学价值**：是"TSDF→ESDF"流水线的标准教材，伴随的 `mav_voxblox_planning` 提供 OMPL RRT*/BIT* + "Loco" 多项式优化 + 稀疏骨架图的完整示例，**适合作为 SLAM 工程师进入规划的第一课**。

#### nvblox：voxblox 的 GPU 再实现

**引用**：Millane, Oleynikova, Wirbel, Steiner, Ramasamy, Tingdahl, Siegwart, *"nvblox: GPU-Accelerated Incremental Signed Distance Field Mapping"*, ICRA 2024, arXiv:2311.00626. **GitHub**: https://github.com/nvidia-isaac/nvblox，**约 1.1k stars**。

nvblox 把 voxblox 的**块哈希**原样搬到 CUDA：块哈希表用 `stdgpu` 的设备端 `unordered_map` 实现，射线投射与 TSDF 更新在 kernel 中并行。ESDF 采用 **PBA (Parallel Banding Algorithm)** 的稀疏块哈希改造：块内 warp 级扫掠 (x,y,z) + 块间边界传播，直到不再有 dirty 块。在 Replica/Redwood 数据集 5 cm 分辨率下 **重建快 177$\times$，ESDF 快 31$\times$**。它是 NVIDIA cuMotion GPU 运动规划器的地图层，**配套 cuVSLAM + Isaac Nav2**。对 SLAM 工程师的价值：它是"经典机器人算法的 GPU 翻版"罕见范例——**同样的逻辑、同样的内存布局，只是把优先队列换成 warp 扫掠**。

#### Ewok 环形缓冲：嵌入式式的极致紧凑

**引用**：Usenko, von Stumberg, Pangercic, Cremers, *"Real-Time Trajectory Replanning for MAVs using Uniform B-splines and 3D Circular Buffer"*, IROS 2017. **GitHub**: https://github.com/VladyslavUsenko/ewok，**约 380 stars**。

核心是 **3D 环形缓冲（环面索引）**：每轴边长是 2 的幂，世界坐标到缓冲索引的映射 `buf_idx = (ix & (N-1), iy & (N-1), iz & (N-1))`——模运算塌缩为位与，零开销。无人机移动时缓冲原点滑动，**旧体素被新体素原地覆盖**（零拷贝滑窗）。模板非类型参数 `_POW`、`_Datatype=int16_t` 让编译器展开内层循环并把模折叠成位掩。这是 **DSP/嵌入式领域的循环缓冲模式在机器人学里的首次 3D 应用**，大多数 SLAM 工程师没见过。Ewok 还包含基于该环形缓冲的 EDT（Felzenszwalb–Huttenlocher 可分离一维变换改造到环面）与 **5–6 阶均匀 B 样条轨迹重规划器**——后者奠定了 Fast-Planner、EGO-Planner 的 B 样条范式。

#### ROG-Map：2024 年 LiDAR 无人机事实标准

**引用**：Ren, Cai, Zhu, Liang, Zhang (HKU MaRS Lab), *"ROG-Map: An Efficient Robocentric Occupancy Grid Map for Large-scene and High-resolution LiDAR-based Motion Planning"*, IROS 2024, arXiv:2302.14819. **GitHub**: https://github.com/hku-mars/ROG-Map，**约 530 stars**。

**三大设计**：**(1) 零拷贝滑窗**——继承 Ewok 的环形索引思想，`SlidingMap` 基类为多层（占据/膨胀/ESDF/前沿）共用。**(2) 计数器式增量膨胀**——每个膨胀体素存一个"被几个占据源体素膨胀到"的计数器。占据体素新增时，$k^3$ 邻居计数 $+1$；占据体素消失（动态物体移走）时计数 $-1$，计数归零才撤销膨胀。这把朴素 $O(k^3 \cdot \Delta N)$ 膨胀降为 **增量 $O(k^3 \cdot \Delta\text{occupied})$**，并**第一次让去膨胀变得正确**（朴素方法做不到）。**(3) 多分辨率计数图**——细 `ProbMap`（0.1 m）+ 粗 `InfMap`（0.2 m）按 2 的幂对齐。

**实测**：0.05 m 分辨率 $30 \times 30 \times 12$ m 局部图 50 Hz，整个地图更新仅 **5.96 ms (20 ms 预算的 29.8%)**，其中膨胀只占 **0.66 ms**。它是 HKU **SUPER**（Science Robotics 2025 高速安全导航）的地图层，事实上成为华语圈 LiDAR 无人机规划的默认地图。**对 SLAM 工程师的价值**：完整展示了"丢掉全局 TSDF、忘掉八叉树，就用机器人中心滑窗稠密栅格"这一 2024 年共识，代码清晰、`SlidingMap` 抽象极具教学性。

#### ikd-Tree：从 SLAM 到规划的共享数据结构

**引用**：Cai, Xu, Zhang, *"ikd-Tree: An Incremental K-D Tree for Robotic Applications"*, arXiv:2102.10808 (2021), 也是 **FAST-LIO2** (TRO 2022) 的核心组件。**GitHub**: https://github.com/hku-mars/ikd-Tree，**约 779 stars**。

**核心算法**：在经典 kd 树基础上支持 **增量插入、惰性删除、箱式删除/重插、降采样、kNN**。惰性标记避免实际移除；节点聚合 `invalid_num`、`tree_size` 让查询跳过已删结点。**箱式操作**是亮点——FAST-LIO2 的滑动局部地图靠 `Delete_Point_Boxes()` 一次调用即可驱逐视场外所有点，毫秒级完成。**在线降采样**：插入时若目标体素已有点，只保留最靠近体素中心者，**免去了另起 VoxelGrid 过滤的时间**。

**活跃再平衡 ($\alpha$-balanced)**：违反阈值时，小子树前台同步重建；大子树则**锁定快照、派发后台线程重建**，主线程此时的插入/删除进入日志队列，重建完毕后原子拼接并回放日志——与 iSAM2 的贝叶斯树重建是同一思想的不同化身。

**意义**：ikd-Tree 是**唯一在 SLAM 与规划中同被复用的数据结构**。对一位 SLAM 工程师，既熟悉（ICP 的 NN 查找），又陌生（并行再平衡、箱删除）；掌握它等于同时打开了 FAST-LIO2 源码与 HKU 规划栈（Bubble Planner、SUPER）的大门。

#### 横向比较：何时选什么？

**规则**总结：(1) **采样/图搜索 + LiDAR** → ROG-Map；(2) **连续时间轨迹优化 + RGB-D 且需要梯度** → voxblox；(3) **Jetson + RGB-D + 极致分辨率** → nvblox；(4) **纯局部 + 无 GPU + 内存紧** → Ewok；(5) **教学基线与小规模室内** → OctoMap；(6) **SLAM 内部与点云碰撞查询共享** → ikd-Tree。**GPU 友好的关键是块哈希布局**——八叉树指针追逐与环形缓冲固定幂 2 边长都不易 SIMT 化，voxblox 当年的设计具有前瞻性。

---

### 二、感知引导规划：yaw 成为一等公民

经典最小急动 (min-snap) 规划把 yaw 视为速度方向的从属量；感知引导规划则把 **yaw 当作独立决策变量**，因为 yaw 就是相机指向。

#### RAPTOR：拓扑路径 + yaw 图搜

**引用**：Zhou, Pan, Gao, Shen, *"RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight"*, IEEE TRO 37(6): 1992–2009, 2021, arXiv:2007.03465. **GitHub**：在 HKUST-Aerial-Robotics/**Fast-Planner** 的 `active_perception` 包内实现，主仓 **约 3.2k stars**。

三个叠加思想构建于 B 样条运动学重规划器之上：**(1) 路径引导优化 (PGO)**——在自由空间拓扑图上生成多条互异引导路径，每条路径用吸引项把 B 样条控制点拉入相应同伦类，避免 ESDF 非凸代价的局部极小。**(2) 风险感知轨迹精修**——惩罚快速穿入未知体素的轨迹段，把无人机"拉回"已观测空间。**(3) yaw 图搜索**——在每个 B 样条节点上离散 yaw 采样，对信息增益（视锥内前沿体素数、遮挡判定）做图搜索，同时加入 yaw 速率/平滑耦合。yaw 与位置轨迹**交替优化**而非联合，保持了 NLP 的可解性。

**C++ 模式**：`plan_env`（占据 + 射线投射增量 ESDF）、`path_searching`（kinodynamic A\* + 拓扑 PRM）、`bspline_opt`（NLopt 解析雅可比）、`active_perception`（yaw + 前沿代价）、`plan_manage`（FSM）。代价是"加权 $\lambda$ 项之和"的典范：`J = w_s·J_s + w_c·J_c + w_f·J_f + w_v·J_v`。**对 SLAM 工程师的价值**：闭环主动建图——yaw 被驱使去观测前沿体素，直接提升后续 SLAM 的建图质量，尽管前端特征跟踪仍由 VINS-Mono/Fusion 负责。

#### PAMPC：把感知塞进 NMPC

**引用**：Falanga, Foehn, Lu, Scaramuzza, *"PAMPC: Perception-Aware Model Predictive Control for Quadrotors"*, IROS 2018, arXiv:1804.04811. **GitHub**: https://github.com/uzh-rpg/rpg_mpc，**约 486 stars**。

PAMPC 是第一篇统一用 **NMPC** 在滚动时间窗内**同时优化动作与感知**的工作。3D 兴趣点 `p_poi` 投影到相机平面 `π(p_poi, T_WB)`：阶段代价加入两个感知项——**视觉中心化**（惩罚投影点到主点的距离平方，等价于最大化 cos 夹角）与**像素速度**（最小化投影点速度，抑制运动模糊 / 跟踪丢失）。非线性动力学由 **ACADO** 用直接多重射击转写为稀疏 QP，用 **qpOASES + RTI (Real-Time Iteration)** 方案每控制步一次 SQP 迭代，**亚毫秒级** ARM 实时。

**三层库架构（高度教学化）**：`mpc_solver`（ACADO 自动生成的纯 C 列优先数组）、`mpc_wrapper`（Eigen 化的薄 C++ 适配层）、`mpc_controller`（ROS 接口）。**与 SLAM 的关系**：PAMPC 是"上游"——它塑造了运动，使下游 VIO 获得好的特征可观性和低像素速度，直接解决快速飞行时 VIO 发散的失效模式。

#### Teach-Repeat-Replan：完整飞行系统

**引用**：Gao, Wang, Zhou, Zhou, Pan, Shen, *"Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments"*, TRO 2020. **GitHub**：https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan，**约 1.1k stars**。

**Teach**：人工操控飞行，在线提取**凸多面体自由空间**，组成安全走廊。**Repeat 全局规划**：Bézier/MINCO 多项式在凸多面体并集里做时空联合优化。**局部重规划**：基于 ESDF 的梯度优化 + 动力学重规划器躲避未建图/动态障碍。**完整栈**：VINS-Fusion + 密集 surfel 建图 + DJI 板载 + 摇杆接口。`polyhedron_generator` 支持 `ENABLE_CUDA` 开关，是学术代码中难得的**可选 GPU 加速**示范。**教学价值**：最完整的"**系统级**"开源工作——一学期研究生课每周拆一个子系统，从 VINS 到凸走廊到 B 样条到 DJI 底层控制。

#### 感知引导规划带来的范式变化

**(1) yaw 作为一等决策变量**——因为 yaw = 相机指向 = 观测什么；**(2) 共视 / 特征覆盖代价**——Zhang & Scaramuzza ICRA 2018 用共视度衡量 VIO 质量；**(3) Fisher 信息代价**——把位姿估计协方差迹/对数行列式加入奖励，闭合主动 SLAM 循环；**(4) 位置与朝向联合优化**——PAMPC 用单 NLP；RAPTOR 用交替；**(5) 未知体素风险**——显式耦合传感器感知速率、重规划率与飞行速度。

---

### 三、自主探索：从贪婪 NBV 到分层覆盖路径

#### nbvplanner：RH-NBV 的祖师

**引用**：Bircher, Kamel, Alexis, Oleynikova, Siegwart, *"Receding Horizon 'Next-Best-View' Planner for 3D Exploration"*, ICRA 2016；期刊扩展 Autonomous Robots 2018. **GitHub**: https://github.com/ethz-asl/nbvplanner，**约 263–400 stars**（近年增长）。

**核心算法**：从当前位姿扩展 RRT，每个候选节点通过**视锥射线投射**计数视场中新观测到的未知体素作为信息增益；分支增益按路径长度指数衰减 `g(n_k)=g(n_{k-1})+Gain(x_k)·exp(-λ·c(σ))`。**只执行最优分支第一条边**，到达后重建 RRT 并保留上轮最优分支种子防止饥饿。终止条件：最大增益低于阈值。

**C++ 模式**：`TreeBase<stateVec>` + `Node<stateVec>` 模板抽象基类（Strategy 模式），`RrtTree : TreeBase` 是默认实现，但同仓配套 "rrt-of-trees" 检视规划器也继承同一接口。`OctomapManager*` 与 `StlMesh*` 通过构造注入（DI）。用 libkdtree 做 RRT 近邻。**教学价值**：读完 Yamauchi 1997 前沿规划后最应读的论文，2500 行核心代码一下午能跑完。

#### mav_active_3d_planning：插件架构的范本

**引用**：Schmid, Pantic, Khanna, Ott, Siegwart, Nieto, *"An Efficient Sampling-Based Method for Online Informative Path Planning in Unknown Environments"*, RA-L 5(2): 1500–1507, 2020. **GitHub**: https://github.com/ethz-asl/mav_active_3d_planning，**约 633 stars**。

**算法**：把 NBVP 的 RRT 升级为 **持久化 RRT\***——树跨迭代保留并随机器人重根，支持**重布线**。两阶段采样（局部球 + 全局）兼顾逃离与覆盖。**迭代射线投射**——相邻射线跳过前面已命中的体素，视点增益计算 ~2$\times$ 加速。所有体素用 **Voxblox TSDF** 并用用户指定 `ValueComputer` 计算全局价值（UCB 等）。

**架构**（学术代码最干净的插件框架之一）：四个虚基类 `TrajectoryGenerator`、`TrajectoryEvaluator`（内含 `SensorModel`/`CostComputer`/`ValueComputer`/`NextSelector`/`EvaluatorUpdater`）、`BackTracker`、`Map`。通过 `ModuleFactory` 在 YAML 中按字符串名实例化——**加新规划器只需派生类，不需重编核心**。包分层 `core / ros / mav / voxblox / app_*` 隔离依赖，是研究代码里少见的工程纪律。**教学价值**：最好的 Factory/Strategy/Decorator 设计模式实例，适合作为软件架构课程的案例。

#### FUEL：分层 + 增量前沿结构

**引用**：Zhou, Zhang, Chen, Shen, *"FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning"*, RA-L 6(2): 779–786, 2021, arXiv:2010.11561. **GitHub**: https://github.com/HKUST-Aerial-Robotics/FUEL，**约 1.3k stars**。

重新定义了单机无人机探索速度（3–8$\times$ 超越 NBVP/经典前沿）。**两大支柱**：

**前沿信息结构 (FIS)**——每个前沿簇 `F_i` 除了 cell 列表外，额外缓存 **PCA 包围盒、候选视点集 VP_i（按覆盖率排序）、到邻近簇的最短路径成本（A\* 于占据栅格）**。地图更新时仅重评估落入更新包围盒的簇（dirty 簇模式），可扩展到大场景、几十 Hz 规划率。

**分层规划**：**(1) 全局 ATSP**——N 个簇最佳视点间的非对称旅行商代价矩阵，用 **LKH (Lin–Kernighan–Helsgaun)** 启发式在毫秒内求解；**(2) 局部视点精修**——半径 5 m 内保留每簇 15 个候选视点，在 DAG 上跑 Dijkstra 选最佳链；**(3) 最小时 B 样条**——三阶均匀 B 样条用 NLopt 同时优化平滑度 / 总时间 / 软障碍距离 / 动力学可行性，凸包性让约束退化为控制点差分的线性形式。

**C++ 特色**：继承 Fast-Planner 架构（`plan_env`/`path_searching`/`bspline_opt`/`plan_manage`/`active_perception`），外部求解器 NLopt + LKH + Armadillo。**教学价值**：**分层探索的最佳教案**——三层分解是可迁移的元模式；向学生展示了运筹学经典（LKH ATSP）如何击败贪婪最近前沿；后续 RACER/FC-Planner/FALCON 都是 FUEL 的增量。

#### GBPlanner / GBPlanner2：子地下世界之王

**引用**：Dang, Tranzatto, Khattak, Mascarich, Alexis, Hutter, *"Graph-based Subterranean Exploration Path Planning using Aerial and Legged Robots"*, J. Field Robotics 37(8): 1363–1388, 2020. **GitHub**: https://github.com/ntnu-arl/gbplanner_ros，**约 770–785 stars**（任务默认值保守估计）。

为 **DARPA SubT** 量身打造。**局部规划器**：每周期在机器人周围局部包围盒内采样顶点 → 加入 **RRG（rapidly-exploring random graph，不是 tree）**→ 运行 Dijkstra → 叶节点做 Voxblox 增益射线投射（含聚类加速 + 叶节点唯一化）→ 选最高增益路径。**全局规划器**：累积稀疏全局图 `G_global`，维护全局前沿顶点。当局部增益为零（死胡同），在全局图上 Dijkstra 到最高增益前沿，**返航路径也复用同一图**——SubT 中的时间约束必需能力。

**工程亮点**：CMake flag `COL_CHECK_METHOD` 编译时切换体素立方/球体-ESDF/射线碰撞检测；`RAY_CAST_METHOD` 切换朴素/迭代；`robot_type` 枚举支持 aerial/ground/legged；YAML 数百个可调参数；多地图后端 (Voxblox / OctoMap) 抽象。Team CERBERUS 四台 ANYmal C 并行跑 GBPlanner2 **赢得 DARPA SubT 决赛**。**教学价值**：工业级研究代码的参数纪律与鲁棒性示范；**局部/全局二分**是"贪婪卡在死胡同"问题的通解。

#### RACER：去中心化多机协同

**引用**：Zhou, Xu, Shen, *"RACER: Rapid Collaborative Exploration With a Decentralized Multi-UAV System"*, IEEE TRO 39(3): 1816–1835, 2023. **2023 IEEE TRO King-Sun Fu 最佳论文奖**. **GitHub**: https://github.com/SYSU-STAR/RACER，**约 715 stars**。

**去中心化协同**的四大创新：**(1) 在线分层栅格 (hgrid)** 双分辨率递归细分工作空间，每栅格跟踪未知体积与覆盖成本。**(2) 成对交互协议**——任意两机进入通讯范围时交换 hgrid 状态并执行本地冲突解决，保证**没有两机指向重叠区域**。通讯可异步有损。**(3) CVRP (带容量车辆路由)** 负载分配——单机被分配的栅格集合不超过"容量"（预期覆盖旅行成本衡量），用 **LKH-3** 经 CVRP→TSP 原子变换求解。**(4) 分层单机规划**——覆盖 TSP → 局部视点精修 → 最小时多项式轨迹。

**C++ 模式**：`ROS1` + `catkin_make`；外部 LKH-3 以子进程方式调用（经典"求解器写 .par/.tsp 文件"模式）；机间通过压缩 `SwarmMsg`（时间戳 hgrid 差分 + 竞价）通讯。**与 SLAM 关系**：依赖 **Omni-Swarm**（HKUST T-RO 2022）去中心化视觉-IMU-UWB 估计器。**教学价值**：去中心化任务分配与经典 OR 路由 (CVRP/TSP) 在实时机器人的完美结合。

#### FC-Planner：骨架引导覆盖

**引用**：Feng, Li, Zhang, Chen, Zhou, Shen, *"FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes"*, ICRA 2024, arXiv:2309.13882. **ICRA 2024 Best UAV Paper Finalist**. **GitHub**: https://github.com/HKUST-Aerial-Robotics/FC-Planner，**约 342 stars**。

面向**已知先验点云**的 3D 重建覆盖规划。**骨架分解 (SSD, ROSA 旋转对称轴)** 从点云提取 1D 拓扑骨架，按骨架分支把场景切成简单子空间——比体素分解便宜得多，对管道/柱/桥等曲线结构具语义意义。**骨架引导视点生成**：每个子空间内做 set-cover 最小视点集，视点姿态用梯度迭代 (`Iterative Updates of Viewpoint Pose`) 精修。**全局覆盖序列**：外层对子空间做 ATSP (LKH)，内层对每个子空间视点做 TSP，**分层组合大幅缩减状态空间**。依赖 Eigen 3.4 / PCL 1.10 / Boost 1.71 / OMPL；附 Marina Bay Sands、HKUST 红鸟雕像 demo。**教学价值**：展示 3D 点云几何处理 (medial axis) 与 set-cover 传感器布放的结合。

#### FALCON：覆盖路径引导 > 前沿选择

**引用**：Zhang*, Chen*, Feng, Zhou, Shen, *"FALCON: Fast Autonomous Aerial Exploration Using Coverage Path Guidance"*, IEEE TRO 41: 1365–1385, 2024, arXiv:2407.00577. **GitHub**: https://github.com/HKUST-Aerial-Robotics/FALCON，**约 270 stars**。

指出经典前沿选择（FUEL, Yamauchi）会导致**来回往返**与锯齿路径。FALCON 把探索重写为 **在线覆盖路径引导**：**(1) 增量可达性感知空间分解**——自由空间连通子区域增量维护为图；**(2) 全局覆盖路径**横跨**整个未探索空间**（不只是前沿），用 LKH 在连通图上求解，作为跨周期的**持久引导**；**(3) 局部前沿访问重排**由全局 CP 段**偏置**，避免重访；**(4) 最小时多项式**。

还贡献了 **EPEE 基准**（相同四旋翼模拟器 + **VECO** 指标：体积/效率/连续性/重叠）。在 classical_office/complex_office/darpa_tunnel/duplex_office/octa_maze/power_plant 上**全面超越 FUEL/TARE/RACER**。工程上依赖 CMake $\ge$ 3.20 / NLopt 2.7.1 / **Open3D 0.18.0 / CUDA `pointcloud_render`**（罕见的 GPU 渲染学术代码）。**教学价值**：完美的"前沿方法之后的下一讲"——为什么贪婪前沿次优，以及怎样用全局引导解决。

#### TARE：分辨率粒度原则

**引用**：Cao, Zhu, Choset, Zhang, *"TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments"*, RSS 2021 (**Best Paper + Best System Paper**)；期刊扩展 Cao et al., *"Representation Granularity Enables Time-Efficient Autonomous Exploration in Large, Complex Worlds"*, **Science Robotics** 8(80), 2023（**注意：任务描述中的"IJRR 2024"实为 Science Robotics 2023**，请在引用时更正）。**GitHub**: https://github.com/caochao39/tare_planner，**约 652 stars**。

**核心思想**：细节仅对近处有用，远处粗糙即可。工作空间被切成固定尺寸子空间：**局部层**（规划地平线立方体内）维护稠密表面、对所有局部视点做精细局部 TSP；**全局层**把每个远处含未观测表面的子空间压成单个节点，做粗糙全局 TSP。局部路径在地平线边界点与全局路径衔接——**在边界点耦合的双分辨率 TSP** 合成一条连续探索路径。求解器：Google **OR-Tools**（仓内自带 AMD64/ARM64 二进制）。**附属的 Autonomous Exploration Development Environment** 提供 Campus $340 \times 340$ m、Indoor、5 层 Garage、Tunnel、Forest、Matterport3D 多场景——已成行业基准。

#### MUI-TARE：未知初始位姿的多机合作

**引用**：Yan, Lin, Ren, Zhao, Yu, Cao, Yin, Zhang, Scherer, *"MUI-TARE: Cooperative Multi-Agent Exploration With Unknown Initial Position"*, RA-L 8(7): 4299–4306, 2023, arXiv:2209.10775. **GitHub**: MetaSLAM/mui-tare（~15 stars）；生产版本 **M-TARE** 在 cmu-exploration.com 以 Docker 分发。

关键贡献：**自适应子图融合**——既不朴素"首次重叠即融合"（假阳性多），也不保守"重走对方大半条轨迹"（慢），而是根据 **AutoMerge 球谐位置描述子的验证增益** 自适应调整所需重叠长度。**分层协同**：子图未融合时各机独立跑 TARE；融合后合作层解 **min-max 多仓库车辆路由 (MD-VRP)**——最小化最长个体行程。2 机比 TRUST-Explorer 快 29%，3 机快 14%。**这是"active SLAM"的清晰范例**——规划器显式触发额外探索动作来改善 SLAM 地图融合质量。

#### 多机协同的通讯与分配策略

| 通讯模型 | 代表 | 特点 |
|---|---|---|
| 全通讯（理想基线） | 经典多机 | 性能上限，现实不可达 |
| 间歇 / 预设会合 | Rendezvous 方法 | 付出等待时间 |
| 机会式追击 (pursuit) | **M-TARE** | 期望信息增益 > 追击成本才追击 |
| 异步成对八卦 | **RACER** | 最适合低占空比电台 |

**任务分配公式**：TSP (FUEL, FALCON 局部, TARE)、ATSP (FC-Planner)、MTSP、MDMTSP/MD-VRP (MUI-TARE, SOAR)、**CVRP (RACER)**、拍卖式（DARPA SubT 早期）。**路由公式 (TSP/VRP) 家族已成主流**，因为 LKH/OR-Tools 足够快、目标直接映射探索时间、分层分解可控规模。

**2025 年前沿趋势**：(1) 覆盖路径引导 > 前沿选择（FALCON 领跑单机）；(2) 分层双分辨率 (TARE/M-TARE) 统治大规模室外；(3) **异构多机** (SOAR：1 台 LiDAR 探索者 + N 台相机摄影师) 崛起；(4) 通讯感知去中心化；(5) **拓扑先验融合**——2025 Drones MDPI 文章与 HKUST **SLABIM** 数据集把 BIM/卫星地图并入；(6) 学习 / LLM 引导探索（SSR-ZSON 叠加零样本语义推理于 TARE）。

---

### 四、3DGS / NeRF 时代：可微分地图与控制的融合

#### Splat-Nav：高斯椭球即安全走廊

**引用**：Chen*, Shorinwa*, Bruno, Swann, Yu, Zeng, Nagami, Dames, Schwager, *"Splat-Nav: Safe Real-Time Robot Navigation in Gaussian Splatting Maps"*, **IEEE TRO 41: 2765–, 2025**, DOI: 10.1109/TRO.2025.3552348, arXiv:2403.02751. **GitHub**: https://github.com/chengine/splatnav，**约 41 stars** (Python/PyTorch 仓)。

**核心洞见**：每个高斯 splat `i` 的几何支撑就是一个 **各向异性椭球** `{x : (x-μᵢ)ᵀΣᵢ⁻¹(x-μᵢ) ≤ k²}`，`k` 取 99% 置信界（可再融入 `αⱼ` 做概率占据椭球）。因此：
- **椭球-椭球碰撞测试**（Gilitschenski & Hanebeck 2012 风格；Splat-Nav Corollary 3 给出闭式变体）：两椭球相交当且仅当 $\lambda\in[0,1]$ 上广义特征值函数最小值 $\le 1$。
- **Splat-Plan**：(1) 用 splat 中心构建栅格 + A\* 获取初始路径；(2) 在每个 A\* 点**膨胀为凸多面体**——沿最近椭球切面朝外扩展直至遇壁——得到**形式安全**的凸多面体走廊；(3) Bézier 曲线用凸包性把走廊约束线性化到控制点，QP 求解。
- **Splat-Loc**：把 splat 解读为彩色点云，RGB-D 全局初始化 + RGB LightGlue PnP 跟踪 + UKF + 走廊内约束 MAP 精修——**25 Hz 定位、>2 Hz 重规划**。

**C++ 工程借鉴**：Splat-Nav 本体是 Python/PyTorch + Nerfstudio + Clarabel (QP) + `dijkstra3d` + `viser`，但其设计模式极其值得 C++ 移植：**CPU 处理几何 / GPU 处理渲染**，K-D 树剪枝 $10^5$ 级 splat 的椭球碰撞，**ROS2** 桥接。**与 SLAM 关系**：假设预建 3DGS，但 Splat-Loc 自成定位系统；自然对接 GSplat-SLAM 前端（Matsuki 2024, GS-SLAM, Gaussian-SLAM, SplatBridge）。**教学价值**：理解 **高斯椭球代数如何替代 ESDF** 的最佳入门。

#### SAFER-Splat：CBF 与在线高斯融合

**引用**：Chen*, Swann*, Yu*, Shorinwa, Murai, Kennedy, Schwager, *"SAFER-Splat: A Control Barrier Function for Safe Navigation with Online Gaussian Splatting Maps"*, arXiv:2409.09868, 2024. **GitHub**: https://github.com/chengine/safer-splat，**约 33 stars**。

定义安全函数 `h(x) = min_i d_E(x, Eᵢ) - r_robot`（机器人中心到最近椭球的欧氏距离减身体半径），嵌入为 **控制屏障函数**：`ḣ(x,u) + α(h(x)) ≥ 0` 是 QP `min‖u-u_ref‖² s.t. CBF 线性约束` 的一条线性约束。用 **Clarabel** 15 Hz 求解；**约束剪枝**（K-NN 于 splat 中心）把 $10^5$ 活跃约束降到 $10^2$–$10^3$。**SplatBridge** 是 NerfBridge 扩展，ROS1/ROS2 把 RGB + ToF + VIO 位姿馈入 Nerfstudio splatfacto **在线训练** 150k–300k 高斯。**亮点**：splat 优化器与 CBF 控制循环**在同一 GPU 上异步并发**，锁守护的参数快照——现代"感知 – 控制协同设计"并发模式范本。比 NeRF-CBF 快 20–50$\times$。

#### FiGS / SOUS VIDE：把 GSplat 当模拟器训练策略

**引用**：Low, Adang, Yu, Nagami, Schwager, *"SOUS VIDE: Cooking Visual Drone Navigation Policies in a Gaussian Splatting Vacuum"*, **RA-L 2025** (IEEE Xplore 10937041), arXiv:2412.16346. **GitHub**: https://github.com/StanfordMSL/SousVide，**约 21 stars**。（FiGS 与 SOUS VIDE 是同一工作：FiGS 是模拟器组件，SV-Net 是策略，SOUS VIDE 是总框架）。

**FiGS**：用 9 维简化四旋翼动力学 + 预建 GSplat 场景实时渲染**板载视角图像 $\le$130 fps**。**专家 MPC → 策略蒸馏**：特权 MPC 驱动 FiGS 生成 10 万–30 万 (图像, 状态, 动作) 元组，带动力学随机化（$\pm$30% 质量、40 m/s 风、60% 亮度、空间扰动）。**SV-Net** 在板载 Orin Nano 以 20 Hz 吃 RGB + 光流 + IMU 输出体率 + 集中推力；**RMA (Rapid Motor Adaptation)** 分支在线推断动力学嵌入。**105 次硬件实验**验证零样本 sim-to-real。

**相关前沿**：**GRaD-Nav**（arXiv:2503.03984, 2025，3DGS 内可微分 RL）、**VISTA**（VLA 策略在 3DGS 内训练）、**FAST-Splat**（开词汇语义 GSplat）、**FalconGym 2.0**（IROS 2025 无人机竞速光实感模拟器）。

#### 3DGS 改变规划范式的六个维度

**(1) 可微分渲染入环**——地图从参数 + 位姿 → 图像是可微函数，解锁像素级梯度策略训练 / 位姿估计 / 在线建图同时规划。**(2) Splat 距离查询替代 ESDF**——每个障碍是**显式各向异性椭球**，点-椭球距离 (GJK 1988)、椭球-椭球重叠（Splat-Nav Cor. 3）、Mahalanobis 距离皆为闭式；K-D 树于 splat 均值上做 O(log N) 最近-splat 查询；**无需体素化、无需射线投射、无需距离场传播**。**(3) 光度反馈**——信息增益 / NBV 可基于 FisherRF 等高斯参数 Fisher 信息；SOUS VIDE 的梯度直接穿过渲染图像回传到动作。**(4) 场景特化 vs 场景不可知**——几何型 (Splat-Nav, SAFER-Splat) 仍场景不可知，学习型 (SOUS VIDE, GRaD-Nav) 则针对特定 GSplat 训练——**范式转变**。**(5) GPU/CPU 并发**——历史上规划器是单线程 CPU 程序；GSplat 强迫"CPU 几何 + GPU 训练/渲染"并发设计。**(6) 概率占据与安全边界**——高斯支撑无界，需按置信等级 (k=3 → 99.7%) 截断；本质比均匀栅格更贴合薄长几何。

---

### 五、SLAM 后端：为规划提供感知管线

#### FAST-LIO2：事实标准

**引用**：Xu, Cai, He, Lin, Zhang, *"FAST-LIO2: Fast Direct LiDAR-Inertial Odometry"*, IEEE TRO 38(4): 2053–2073, 2022. **GitHub**: https://github.com/hku-mars/FAST_LIO，**约 4.6k stars**。

**三大贡献**：(1) **原始点直接扫描-地图配准**（无需边/面特征提取），统一支持 Livox 固态 LiDAR 与旋转 LiDAR；(2) **迭代误差状态卡尔曼滤波 (IESKF)** 在 $SO(3) \times R^n$ 流形上 (IKFoM 工具箱)，新卡尔曼增益形式避免对高维测量协方差求逆，100 Hz 在 ARM 上实时；(3) **ikd-Tree** 增量 kd 树作地图——支持插入/删除/在树上体素降采样/箱式删除/并行再平衡，单帧 ~1.6 ms（对比静态 kd 树线性增长超 100 ms）。

#### Point-LIO：kHz 级控制反馈

**引用**：He, Xu, Chen, Kong, Yuan, Zhang, *"Point-LIO: Robust High-Bandwidth LiDAR-Inertial Odometry"*, Adv. Intell. Syst. 5(7), 2023. **GitHub**: https://github.com/hku-mars/Point-LIO，**约 1.2k stars**。

**逐点 EKF 更新**：在每点采样时刻更新，消去帧内运动畸变，输出 4–8 kHz 里程。**随机过程增广运动学模型**：$\omega$ 与 a 本身建模为随机过程，**IMU 测量被视为模型输出而非输入**——IMU 饱和时滤波仍可跟踪。验证于 PULSAR 单转子 UAV（角速度 > 75 rad/s）。复用 ikd-Tree。**适用场景**：激进/振动重/IMU 饱和飞行（如竞速无人机）；**航测类飞行仍建议 FAST-LIO2**。

#### Faster-LIO：iVox 与 TBB 并行

**引用**：Bai, Xiao, Chen, Wang, Zhang, Gao, *"Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels"*, RA-L 7(2): 4861–4868, 2022. **GitHub**: https://github.com/gaoxiang12/faster-lio，**约 1.4k stars**。作者高翔是《视觉 SLAM 十四讲》著者，仓库自带教学价值。

**iVox** 用 `std::unordered_map<VoxelKey, VoxelNode>` 存稀疏占据体素，两种变体：**Linear iVox**（每体素 `std::list<Point>`，查询扫描 + 6/18/26 邻居）与 **PHC iVox**（伪希尔伯特曲线按局部性组织）。**LRU 缓存淘汰**代替 ikd-Tree 箱删除。**Intel TBB 并行近似 kNN**：牺牲严格保证换来较 FAST-LIO2 快 1.5–2$\times$，固态 LiDAR 1–2 kHz，32 线旋转 LiDAR >200 Hz。**教学价值**：与 FAST-LIO2 **只换了地图结构**，单变量对照实验的完美范本；并演示 TBB 并行模式。

#### 主动 SLAM 与 FAST-LIO2 + ikd-Tree 的华语圈霸权

**主动 SLAM** 概念：在探索增益之外加入位姿协方差迹 / Fisher 信息作为二次目标。**ALCP** (MDPI RS 2022) 给 NBVP 加回环机会增益；**Papachristos 2017** 把协方差传播进 RHP 树；**HKUST ICRA 2022 "Exploration with Global Consistency"** 做实时再积分 + 主动回环。**LiDAR 漂移低**使主动 SLAM 常化为"漂移最小化"而非显式回环闭合。

**FAST-LIO2 + ikd-Tree 为何统治华语圈无人机**：(1) **共享数据结构**——ikd-Tree 是头文件库，规划器可与 LIO **共用同一棵树**；`KD_TREE::Nearest_Search(query, k, res, sq_dist)` 直接成为碰撞查询原语；(2) **固态 LiDAR 首发支持** (Livox Avia/Mid-360)；(3) **ARM 100 Hz** 适配 Manifold-2C / Jetson。HKU/HKUST/ZJU 的 FUEL / RACER / FALCON / ROG-Map / SUPER / Swarm-LIO2 / Bubble Planner / EGO-Planner / GCOPTER / MINCO **全部基于此堆栈**。

**规范管线**：
```
Livox Avia/Mid-360 + IMU  →  FAST-LIO2 (IESKF, state_ikfom)
                                │                │
                                ▼                ▼
                        ikd-Tree 全局点云      控制器 (MPC)
                                │
                                ▼
                        ROG-Map / voxblox / D-Map (稠密占据 + 可选 ESDF)
                                │
                                ▼
                        前端: A*, kinodynamic RRT*, JPS, 拓扑路径
                        后端: B 样条 (Fast-Planner), MINCO/GCOPTER, IPOPT QCQP
                                │
                                ▼
                        轨迹跟踪 + 电机控制
```

#### SLAM 工程师转规划的技能地图

**沿用**：SE(3)/SO(3) 流形线性化；协方差传播与 Jacobian；点云/体素/树数据结构；ROS/C++ 纪律；时间同步。

**新学**：**(1) 构型空间障碍膨胀**（ROG-Map `inflation`）；**(2) ESDF 语义**——TSDF 为重建，**ESDF 为规划**，梯度即最安全方向向量场；**(3) 安全飞行走廊 (SFC)**——GCOPTER 多面体走廊、CIRI 构型空间；**(4) 四旋翼微分平坦性**——位置 + yaw 定义全部状态与控制 (MINCO)；**(5) 前沿与信息增益**——Yamauchi/NBVP/LKH 覆盖 TSP；**(6) 滚动时间窗重规划循环**——MPC 节拍而非测量节拍；**(7) 控制器与可控性**——四旋翼欠驱动 (4 输入 6 DoF)；**(8) 感知引导 yaw 规划**——yaw 从动力学看"免费"，用来指向信息密集方向。

**推荐学习路径**：FAST-LIO2 跑 Livox 包 → ikd-Tree 原论文 + 最小 NN 规划器 → ROG-Map + A\*/RRT\* 示例 → voxblox + `mav_voxblox_planning` → FUEL + FALCON 源码 → GCOPTER/MINCO 数学。

---

### 六、结论：三代范式并存的 2025 年

十年回望，**环境表征是驱动规划演化的第一推动力**。第一代（2013–2017）以 OctoMap / voxblox / Ewok 为代表，**CPU 体素主义**奠定基础；第二代（2018–2023）以 FAST-LIO2 + ikd-Tree + ROG-Map + FUEL + RACER 为代表，**LiDAR 固态化 + 机器人中心滑窗 + 分层 TSP/VRP 路由**成为事实标准，华语圈三强（HKU MaRS、HKUST Aerial、SYSU STAR）在此阶段贡献了占据前沿席位的大部分成果；第三代（2024–2025+）以 Splat-Nav / SAFER-Splat / SOUS VIDE 为代表，**可微分高斯场 + 闭式椭球碰撞 + GPU/CPU 异步**开启端到端范式。

**三代并非替代关系**，而是 **不同约束下的最优选择**——嵌入式低功耗仍属第一代，LiDAR 无人机生产栈稳在第二代，而 Jetson 级 GPU + 科研前沿则走第三代。CMU 的 TARE/M-TARE 独树一帜，以"分辨率粒度"原理在大规模户外场景维持领先；ETH ASL 的 mav_active_3d_planning 则以插件架构成为软件工程教科书。

对一位从 SLAM 转入规划的工程师，最直接的路径是：**先掌握 FAST-LIO2 + ikd-Tree + ROG-Map + FUEL 这条 HKU/HKUST 标准栈**（一学期工作量），再选择进入 GSplat 可微分方向或 TARE 大场景方向。yaw 作为一等决策变量、安全飞行走廊凸分解、微分平坦性——这三个概念是 SLAM 背景之外的必修桥梁。整个领域未解决的问题仍清晰：**长时段无 GNSS 多机异构 SLAM、LLM 引导语义探索、3DGS 场景迁移泛化、纳米无人机群**——这些将是 2026 年后的主战场。

**最后提示**：任务描述中 TARE 期刊扩展应为 **Science Robotics 2023**（Vol. 8 No. 80）而非 IJRR 2024；GBPlanner 当前星数已达约 770–785 而非"200+"；voxblox 当前约 1.6k 星；各仓星数动态变化 $\pm$15% 属正常漂移，不影响技术判断。


