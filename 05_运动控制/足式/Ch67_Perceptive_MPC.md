> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 67 章：Perceptive MPC——让 MPC "长眼睛"的地形感知控制

> **难度**：⭐⭐⭐ | **预计学时**：25-30 小时 | **前置**：Ch55（OCS2 完整栈）、Ch60（感知驱动落脚规划）
>
> **一句话概要**：Perceptive MPC 把高程图、SDF、地形法向量等感知信息嵌入 OCS2 的代价函数和约束中，使 NMPC 能在线生成避障、适应地形的全自由度运动——这是 ETH RSL 在 ANYmal 上实现鲁棒崎岖地形行走的核心技术，也是经典 MPC 与感知融合的标杆工作（Grandia et al. T-RO 2023）。

---

## 前置自测

📋 **答不出 >= 2 题 --> 先回对应章节复习**

1. **[Ch55]** OCS2 的 SQP 求解器如何处理不等式约束？Real-Time Iteration（RTI）只做 1 次 SQP 迭代的前提是什么？
2. **[Ch55]** OCS2 的双线程架构中，MPC 线程和 MRT 线程如何通过 Triple Buffer 通信？为什么这样设计？
3. **[Ch60]** 高程图（Elevation Map）的分辨率、更新频率和查询方式各是什么？为什么高程图适合嵌入 MPC？
4. **[Ch60]** SDF（Signed Distance Field）的正负值分别代表什么？为什么 SDF 的梯度天然可用于优化？
5. **[Ch48]** CppAD 的 `AD<double>` 和 `CppADCodeGen` 各解决什么问题？为什么 MPC 代价函数需要自动微分？

---

## 本章目标

学完本章，你应能：

1. **理解盲 MPC 在崎岖地形上的系统性失效模式**，以及 Perceptive MPC 如何通过感知信息弥补
2. **掌握 Grandia 2023 T-RO 的完整管线**：传感器 --> 高程图 --> 可踩性分类 --> 平面分割 --> SDF --> MPC 约束/代价 --> WBC
3. **能从数学上推导 SDF 碰撞约束如何经一阶线性化变成 MPC 可处理的不等式约束**，以及梯度如何计算
4. **理解地形感知代价函数的设计**：躯体高度跟踪、躯体避障、摆动腿地形回避
5. **能读懂 OCS2 `ocs2_perceptive` 模块的真实接口和本章教学抽象之间的对应关系**，并知道如何添加自定义感知约束
6. **能进行 RL vs MPC 感知运动方案的系统性对比**（Miki 2022 vs Grandia 2023）
7. **掌握部署实践中的延迟预算、降级策略、计算资源分配**

---

## 67.1 从盲 MPC 到感知 MPC ⭐

> **本节解决什么问题**：建立"为什么需要把感知嵌入 MPC"的动机——不是因为技术上能做，而是因为盲 MPC 在真实崎岖地形上会系统性地失效。

### 动机：盲 MPC 在崎岖地形上的三重失效

Ch55 讲了 OCS2 的 NMPC 框架如何在**平地**上为四足机器人生成流畅的全身运动。在实验室环境下，这套控制器表现优异——MPC 以 100 Hz 求解，WBC 以 400 Hz 执行，ANYmal 可以稳健地 trot、pace、甚至跑步。

但当 ANYmal 走出实验室，面对碎石坡、台阶、沟渠时，盲 MPC 会遭遇三重系统性失效：

```
盲 MPC 的三重失效

失效 1：落脚点不可行
  MPC 规划的落脚点基于平地假设 → 实际落在悬崖边/尖石上/沟渠中
  后果：接触力分配失败 → 摩擦锥约束违反 → 滑动/摔倒

失效 2：摆动腿碰撞地形
  MPC 的摆动轨迹按照"抬高 5cm → 前移 → 放下"生成
  不知道路径中间有石头或台阶 → 摆动腿正面撞击障碍
  后果：关节电机过载、减速器冲击、结构损坏

失效 3：躯体高度不匹配
  MPC 维持恒定的躯体高度（如 0.48m）
  上坡时实际地面升高 → 腿几乎伸直，失去调节余量
  下坡时地面降低 → 腿过度弯曲，膝关节力矩过大
  后果：运动学奇异 → MPC 不可行 → 紧急停机
```

这三重失效不是偶发的——在典型的 >10 度坡度或 >5cm 高差条件下容易出现。这意味着**没有感知的 MPC 只能在平地上工作**，这极大限制了腿足机器人的应用范围。

### 如果不用感知 MPC 会怎样——反应式补偿的局限

一个自然的想法是：不改 MPC，在下游用反应式（reactive）策略补偿。比如：

- 脚碰到障碍就抬高（力/力矩反馈）
- 着地冲击过大就缩回（阻抗调节）
- 倾斜超过阈值就减速（安全监控）

Ch60 已经分析过这类反应式策略的三个根本局限——反应滞后、不可逆冲击、无法全局规划。从 MPC 的视角来看,还有两个额外的问题：

**问题 1：MPC 的预测视野被浪费了**。MPC 的核心价值在于它能**预测未来**——典型的预测视野是 1-2 秒，对应 2-4 步。但如果 MPC 不知道未来的地形，这个预测视野里的"地形模型"都是平地假设，等于在预测一个**不存在的未来**。反应式补偿只能在"已经发生"之后起作用，完全浪费了 MPC 的前瞻能力。

**问题 2：MPC 和反应式层会互相冲突**。MPC 生成的轨迹假设平地，反应式层修改了这条轨迹。下一个 MPC 周期看到的状态偏离了自己的预测——MPC 会试图"纠正"回来，而反应式层又会再次修改。两者形成振荡，降低整体性能。

| 方案 | 前瞻能力 | 一致性 | 复杂地形能力 | 计算复杂度 |
|------|---------|--------|------------|-----------|
| 盲 MPC + 反应式补偿 | 无（反应式是事后） | 差（MPC 与反应层冲突） | 有限（仅处理小扰动） | 低 |
| **感知 MPC** | **有**（预测视野内看到真实地形） | **好**（地形信息统一在 MPC 内） | **强**（直接优化避障轨迹） | 中-高 |

### 历史：感知嵌入 MPC 的演进

```
感知 + MPC 融合的里程碑

2014  Fankhauser (ETH)       Elevation Mapping 框架
      → 为后续的感知嵌入提供了标准地形表示
  |
2016  Mastalli et al.        Terrain-aware locomotion planning
      → 离线优化中考虑地形，但不是实时 MPC
  |
2018  Di Carlo (MIT)         Convex MPC for quadrupeds
      → 高效的盲 MPC，成为后续感知扩展的基线
  |
2019  Grandia et al.         Frequency-Aware MPC (RA-L)
      → OCS2 框架成熟，为感知嵌入做好了架构准备
  |
2022  Jenelten (ETH)         TAMOLS (T-RO)
      → 地形感知运动优化，SDF 碰撞回避，但不是在线 MPC
  |
2022  Miki (ETH)             Learning perceptive locomotion (Science Robotics)
      → RL 方案：teacher-student 框架，端到端学习感知运动
  |
2023  Grandia (ETH)          Perceptive Locomotion via NMPC (T-RO)  ← 本章核心
      → 完整的感知-NMPC 管线，100 Hz 实时，ANYmal 实机验证
  |
2025  Corbères et al.        Whole-Body Perceptive MPC + Optimal Region Selection (IEEE Access)
      → 扩展到全身 MPC，结合最优区域选择
  |
2025  Jacquet et al.         Neural NMPC with SDF encoding (IJRR)
      → 用神经网络编码 SDF，实现无地图碰撞避免
```

Grandia 2023 的历史地位在于：它是较早系统性展示**实机 100 Hz 感知 NMPC**的代表性工作之一。之前的工作要么偏离线优化（TAMOLS），要么是 RL 方案（Miki 2022），要么没有把完整感知管线嵌入在线 NMPC。Grandia 2023 把高程图、SDF、平面分割全部实时嵌入 OCS2，在 ANYmal 上验证了台阶攀爬、间隙跨越、stepping stones 等任务。

> **本质洞察**：Perceptive MPC 的核心挑战**不是感知本身,也不是 MPC 本身,而是两者之间的"接口翻译"**——高程图是离散栅格（例如 4cm 分辨率、4m x 4m 局部地图约 100x100 个格子），MPC 的 SQP 求解器要求连续可微或至少分片可微的代价和约束。Grandia 2023 的真正贡献是提出了一套系统的方法把离散地形数据转化为 SQP 可处理的局部模型——包括双线性插值实现地形高度查询、SDF 编码实现碰撞约束线性化、凸区域分解实现落脚约束。没有这套"翻译层",感知和 MPC 就像说不同语言的两个人——各自能力再强也无法协作。

### "Perceptive" 到底意味着什么

"Perceptive MPC"不是简单地"给 MPC 加个摄像头"。它意味着**感知信息成为 MPC 最优控制问题的一等公民**——地形数据直接影响代价函数和约束，而不是在 MPC 外部做预处理。

```
盲 MPC 的 OCP（最优控制问题）：
  min  J = Σ l(x, u)           ← 代价只依赖状态和控制
  s.t. ẋ = f(x, u)            ← 动力学
       h(x, u) >= 0            ← 摩擦锥、关节限位等

感知 MPC 的 OCP：
  min  J = Σ l(x, u, M)        ← 代价还依赖地形 M
  s.t. ẋ = f(x, u)            ← 动力学（不变）
       h(x, u) >= 0            ← 物理约束（不变）
       g(x, M) >= 0            ← 地形约束（新增！）
            ↑
            地形约束：
            - 落脚点必须在可踩区域
            - 摆动腿必须避开地形障碍
            - 躯体高度必须适应地面
```

这里 $M$ 代表地形信息（高程图、SDF、平面参数等）。关键挑战是：$M$ 是**离散栅格数据**，而 MPC 的 SQP 求解器需要**连续可微**的代价和约束。Grandia 2023 的核心贡献就是解决这个"离散感知 vs 连续优化"的桥接问题。

> **跨领域类比**：感知 MPC 中地形数据 $M$ 嵌入优化问题的方式,类似于计算机图形学中纹理映射(Texture Mapping)嵌入渲染管线的方式。在图形学中,纹理是一张离散的像素图,但 GPU 的光栅化管线需要在任意连续坐标处查询颜色——解决方案是双线性插值。感知 MPC 面临完全相同的问题:高程图是离散栅格,SQP 需要在任意连续足端坐标处查询地面高度——解决方案同样是双线性插值。不同之处在于,图形学只需要插值的值,而 MPC 还需要插值的梯度(用于 SQP 的 Jacobian)——这要求插值方案本身是可微的。

### ⚠️ 常见陷阱

> ⚠️ **概念误区：认为"感知 MPC"只是在 MPC 代价里加一个地形罚项**
> **新手想法**：在代价函数里加 $w \cdot (z_{\text{foot}} - z_{\text{terrain}})^2$ 就是 Perceptive MPC 了。
> **实际上**：如果只加代价项而不加约束，MPC 的 SQP 可能找到一条"代价低但物理不可行"的轨迹——比如穿过地形的轨迹。感知 MPC 需要同时处理**代价**（引导优选方向）和**约束**（强制可行性）。Grandia 2023 的关键创新之一，是把落脚区域等局部几何约束转化为线性/凸不等式；而 SDF 避障约束整体仍可能非凸，需要依赖 SQP 的局部线性化和 warm start。

> 💡 **概念误区：认为感知 MPC 需要精确的 3D 地图**
> **新手想法**："要做感知 MPC，先要把 SLAM 做到厘米级精度"
> **实际上**：Grandia 2023 使用的是以机器人为中心的**局部高程图**（4cm 分辨率，4m x 4m 范围），不是全局 SLAM 地图。局部高程图只需要里程计（odometry）来对齐点云，对全局精度要求不高。MPC 的预测视野只有 1-2 秒（约 1-2m 移动），在这个范围内局部高程图的精度完全够用。
> **正确理解**：感知 MPC 需要的是"局部准确"，不是"全局准确"。

> 🧠 **思维陷阱：认为"有了感知 MPC 就不需要反应式层了"**
> **新手想法**："MPC 已经看到地形了，不需要下游补偿"
> **实际上**：感知永远不完美——遮挡、传感器噪声、动态障碍（如其他机器人、人、移动物体）都可能导致 MPC 的地形模型与现实不符。ANYmal 的部署中始终保留了一个**反应式安全层**：当接触力异常、关节力矩过大时，触发紧急保护。感知 MPC 是"主力"，反应式层是"安全网"。

### 练习

1. **[分析题]** 假设一个四足机器人以 0.5 m/s 的速度行走，MPC 预测视野为 1.5 秒。计算 MPC 预测范围内的最大移动距离。如果前方 0.6m 处有一个 10cm 高的台阶，盲 MPC 和感知 MPC 分别会怎么处理？画出两种情况下的摆动腿轨迹对比。

2. **[思考题]** 为什么 Grandia 2023 选择在 MPC 层而不是在 WBC 层嵌入感知信息？如果在 WBC 层处理地形约束，会有什么问题？（提示：考虑 WBC 的优化视野和 MPC 的预测视野的差异。）

3. **[设计题]** 如果你要为一个在仓库环境中工作的四足机器人设计感知 MPC 系统，仓库地面有货架、叉车通道（金属格栅）、偶尔散落的包裹。列出你需要在 MPC 的 OCP 中添加的代价项和约束项，并说明每一项的物理含义。

---

## 67.2 感知 MPC 系统架构 ⭐⭐

> **本节解决什么问题**：从全局视角理解 Grandia 2023 的完整管线——从传感器原始数据到 MPC 求解，中间经过了哪些步骤，每步的频率和延迟是多少。

### 动机：为什么需要理解全管线

很多论文只关注"MPC 算法本身"，对感知前端一笔带过。但在实际部署中，**管线的每一个环节都可能成为瓶颈**。理解全管线意味着：
- 知道系统的端到端延迟预算
- 知道哪些计算可以并行、哪些必须串行
- 知道哪个环节失效时如何降级

### 全管线架构图

Grandia 2023 的管线可以分为三个层次、六个模块：

```
┌─────────────────────────────────────────────────────────────────┐
│                    感知层（Perception, ~20 Hz）                   │
│                                                                   │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────────────┐     │
│  │ LiDAR /  │───>│ Elevation    │───>│ (A) Steppability    │     │
│  │ 深度相机  │    │ Mapping      │    │ Classification      │     │
│  │ 点云      │    │ (grid_map)   │    │ (可踩性分类)         │     │
│  └──────────┘    └──────┬───────┘    └─────────┬───────────┘     │
│                         │                       │                  │
│                         │               ┌───────▼───────────┐     │
│                         │               │ (A) Plane         │     │
│                         │               │ Segmentation      │     │
│                         │               │ (平面分割)         │     │
│                         │               └───────┬───────────┘     │
│                         │                       │                  │
│                   ┌─────▼───────────────────────▼───────────┐     │
│                   │ (B) SDF Precomputation                  │     │
│                   │ + Torso Reference Height                │     │
│                   │ (SDF 预计算 + 躯体参考高度)               │     │
│                   └─────────────────┬───────────────────────┘     │
│                                     │                              │
├─────────────────────────────────────▼──────────────────────────────┤
│                    规划层（Planning, 100 Hz MPC）                   │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────┐      │
│  │ (C) Nonlinear MPC (OCS2 SQP + HPIPM)                    │      │
│  │                                                           │      │
│  │  代价函数:                                                │      │
│  │    - 躯体高度跟踪（terrain-adaptive）                      │      │
│  │    - 速度跟踪、姿态跟踪                                    │      │
│  │                                                           │      │
│  │  约束:                                                    │      │
│  │    - 落脚点约束（凸不等式，来自平面分割）                    │      │
│  │    - 摆动腿 SDF 碰撞约束                                  │      │
│  │    - 摩擦锥、关节限位（物理约束）                           │      │
│  │                                                           │      │
│  │  输出: 最优状态/控制轨迹 x*(t), u*(t)                     │      │
│  └──────────────────────────┬───────────────────────────────┘      │
│                              │                                      │
├──────────────────────────────▼──────────────────────────────────────┤
│                    执行层（Execution, 400 Hz）                       │
│                                                                     │
│  ┌─────────────────┐    ┌───────────────┐    ┌────────────────┐    │
│  │ 状态估计         │    │ 全身力矩控制   │    │ 反应式安全层    │    │
│  │ (IMU + 关节)     │    │ (WBC)         │    │ (力/力矩保护)  │    │
│  └─────────────────┘    └───────────────┘    └────────────────┘    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 频率与延迟预算

管线的各个模块运行在不同的频率上，这是理解系统行为的关键：

| 模块 | 频率 | 延迟 | 运行硬件 | 瓶颈 |
|------|------|------|---------|------|
| 点云获取 | 20-30 Hz | 5-10 ms | 传感器 | 传感器帧率 |
| Elevation Mapping | 20 Hz | 10-20 ms | CPU / GPU | 点云投影+Kalman |
| 可踩性分类 | 20 Hz | 2-5 ms | CPU | 特征计算 |
| 平面分割 | 20 Hz | 5-10 ms | CPU | 连通域标记 |
| SDF 预计算 | 20 Hz | 3-8 ms | CPU | 距离变换 |
| **NMPC 求解** | **100 Hz** | **5-10 ms** | **CPU (专用核)** | **SQP+HPIPM** |
| WBC 执行 | 400 Hz | <1 ms | CPU | QP 求解 |
| 状态估计 | 400 Hz | <1 ms | CPU | Kalman 更新 |

**端到端延迟**：从传感器看到障碍到 MPC 输出避障轨迹，总延迟约为 **30-60 ms**。在 0.5 m/s 行走速度下，这对应 1.5-3 cm 的移动距离——对于 4cm 分辨率的高程图来说可以接受。

### 管线的三个子管线 (A)(B)(C) 详解

Grandia 2023 把感知-规划管线明确分为三个子管线：

**(A) 可踩性分类与平面分割**（Steppability Classification + Plane Segmentation）：
- 输入：高程图的每个格子
- 处理：计算坡度、粗糙度、步高等特征 --> 二值分类（可踩/不可踩）--> 连通域标记 --> 对每个连通域拟合平面
- 输出：每个可踩区域的平面参数（法向量 $\boldsymbol{n}$、平面上一点 $\boldsymbol{p}_0$）

**(B) SDF 预计算与躯体参考**（SDF Precomputation + Torso Reference）：
- 输入：可踩性分类结果 + 高程图
- 处理：对不可踩区域加垂直边距（2cm） --> 膨胀一个格子 --> 计算 2D 距离变换得到 SDF --> 从可踩区域高度计算躯体参考高度
- 输出：2D SDF 场（每个格子到最近不可踩区域的距离）+ 躯体参考高度 $z_{\text{ref}}(x, y)$

**(C) MPC 集成**（Integration into OCP）：
- 输入：(A) 的平面参数、(B) 的 SDF 和参考高度、当前状态估计、用户指令
- 处理：构建 OCP --> SQP+HPIPM 求解
- 输出：全自由度最优轨迹

**为什么 (A)(B) 在 20 Hz 而 (C) 在 100 Hz？** 因为地形变化的时间尺度远慢于机器人运动的时间尺度。高程图在 20 Hz 更新已经足够捕捉地形变化（机器人在两帧之间只移动了 2.5cm @0.5m/s）。而 MPC 需要 100 Hz 来跟踪快速变化的动力学状态。(A)(B) 的结果被缓存，(C) 在每个 MPC 周期直接查询缓存。

### 数据流的线程模型

在 OCS2 的双线程架构（Ch55）基础上，Perceptive MPC 增加了一个感知线程：

```
线程模型（3 线程 + Triple Buffer）

线程 1: 感知线程 (Perception Thread, 20 Hz)
  - 接收点云
  - 更新 Elevation Map
  - 运行 (A)(B) 子管线
  - 把结果写入共享内存（double buffer）

线程 2: MPC 线程 (MPC Thread, 100 Hz)
  - 读取感知线程的最新结果（无锁读）
  - 构建 OCP（代价+约束引用感知数据）
  - SQP 求解
  - 把策略写入 Triple Buffer

线程 3: MRT 线程 (MRT Thread, 400 Hz)
  - 从 Triple Buffer 读取最新策略
  - 线性插值得到当前时刻的参考
  - WBC 执行

关键设计：
  - 感知线程和 MPC 线程之间用 double buffer：
    感知写 buffer A，MPC 读 buffer B → 交换
  - MPC 线程和 MRT 线程之间用 triple buffer（Ch55 已详述）
  - 三个线程完全无锁，无互斥量等待
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：感知线程和 MPC 线程竞争同一块内存**
> **错误做法**：MPC 线程直接引用 Elevation Map 对象，感知线程同时在更新它。
> **现象**：MPC 在一次 SQP 迭代中间，SDF 数据被感知线程更新了 --> 前后不一致 --> 梯度跳变 --> SQP 不收敛。
> **根本原因**：感知更新和 MPC 查询不是原子操作。在一次 MPC 求解的 5-10ms 内，感知线程可能完成一次更新。
> **正确做法**：用 double buffer——感知线程写入"后台 buffer"，写完后原子交换指针。MPC 线程始终读"前台 buffer"，保证一次 SQP 内数据不变。

> 💡 **概念误区：认为感知频率越高越好**
> **新手想法**："为什么不让感知也跑到 100 Hz？"
> **实际上**：感知管线（Elevation Mapping + 分类 + 分割 + SDF）单次计算耗时 20-40 ms，即使用 GPU 也需要 5-10 ms。更重要的是，**20 Hz 的感知更新对 MPC 已经够用**——MPC 的预测视野是 1-2 秒，而高程图在 50ms 内变化微乎其微。提高感知频率只会浪费算力而不提升性能。

> 🧠 **思维陷阱：认为管线是严格串行的**
> **新手想法**："必须等感知更新完，MPC 才能开始求解"
> **实际上**：三个线程是**完全并行**的。MPC 不等感知——如果感知还没更新，MPC 用上一帧的感知数据。这意味着 MPC 看到的地形可能比实际延迟了 50ms，但在通常的行走速度下这完全可接受。只有在极高速运动（>1.5 m/s）时，感知延迟才会成为问题。

### 练习

1. **[计算题]** 假设 ANYmal 以 0.8 m/s 行走，感知管线延迟 40 ms，MPC 求解延迟 8 ms。计算从传感器看到障碍到 MPC 输出避障轨迹的总延迟，以及这段时间内机器人走了多远。如果高程图分辨率是 4 cm，这个延迟对应多少个格子的移动？

2. **[设计题]** 为什么 Grandia 2023 把可踩性分类和 SDF 预计算放在感知线程（20 Hz）而不是 MPC 线程（100 Hz）？如果放在 MPC 线程会怎样？（提示：计算每个 MPC 周期的时间预算。）

3. **[编程题]** 用 C++ 实现一个简化的 double buffer 类：一个写者线程、一个读者线程、无锁交换。测试在写者以 20 Hz、读者以 100 Hz 运行时，读者是否永远读到一致的数据。

---

## 67.3 地形约束构建 ⭐⭐

> **本节解决什么问题**：从高程图原始栅格数据出发，一步步构建 MPC 能用的地形约束——可踩性分类 --> 平面分割 --> 凸不等式约束。

### 动机：为什么不能直接把高程图丢给 MPC

高程图是一个 $N \times M$ 的浮点矩阵，每个格子存一个高度值。MPC 的 SQP 求解器需要的是**连续可微的约束函数** $g(\boldsymbol{x}) \geq 0$。直接把离散栅格数据放入优化器有三个问题：

1. **不可微**：栅格边界是阶跃变化，梯度不存在。SQP 需要约束的雅可比，阶跃函数的雅可比要么是零要么是无穷大，都无法用于求解。

2. **维度爆炸**：一个 100x100 的高程图有 10000 个格子。如果对每个格子都加一个约束，OCP 的约束数量从几十个暴增到上万个——HPIPM 无法实时求解。

3. **非凸**：真实地形的可踩区域是不规则形状，"脚必须落在可踩区域内"是一个非凸约束。MPC 的 SQP 求解器依赖局部凸近似，非凸约束会导致收敛困难。

Grandia 2023 的解决方案：**把全局非凸问题转化为局部凸问题**——对每只脚，找到它当前最可能落脚的平面，用该平面的参数构造少量凸不等式约束。

### 步骤 1：可踩性分类（Steppability Classification）

**目标**：对高程图的每个格子，判断它是否适合落脚。

**判断标准**：一个格子被标记为"可踩"需要同时满足三个条件：

$$\text{steppable}(i,j) = \begin{cases} 1 & \text{if } s(i,j) < s_{\max} \text{ AND } r(i,j) < r_{\max} \text{ AND } \Delta h(i,j) < h_{\max} \\ 0 & \text{otherwise} \end{cases}$$

其中：
- $s(i,j)$：局部坡度（slope），由格子邻域的高度梯度计算
- $r(i,j)$：局部粗糙度（roughness），由格子邻域的高度方差计算
- $\Delta h(i,j)$：台阶高度（step height），格子与邻域最大高差

**典型阈值**（ANYmal 配置）：

| 参数 | 符号 | 典型值 | 物理含义 |
|------|------|--------|---------|
| 最大坡度 | $s_{\max}$ | 30-35 度 | 超过此坡度摩擦锥无法保证 |
| 最大粗糙度 | $r_{\max}$ | 0.05 m | 表面不平度超过 5cm 不稳定 |
| 最大台阶高度 | $h_{\max}$ | 0.15 m | 超过 15cm 关节运动学受限 |

**坡度的计算**：对每个格子 $(i,j)$，用 Sobel 算子或中心差分计算高度梯度：

$$\frac{\partial z}{\partial x} \approx \frac{z(i+1,j) - z(i-1,j)}{2r}, \quad \frac{\partial z}{\partial y} \approx \frac{z(i,j+1) - z(i,j-1)}{2r}$$

其中 $r$ 是栅格分辨率。坡度为：

$$s(i,j) = \arctan\left(\sqrt{\left(\frac{\partial z}{\partial x}\right)^2 + \left(\frac{\partial z}{\partial y}\right)^2}\right)$$

**粗糙度的计算**：取 $(i,j)$ 周围 $k \times k$ 窗口，拟合一个平面 $z_{\text{plane}}$，粗糙度定义为残差的标准差：

$$r(i,j) = \sqrt{\frac{1}{k^2} \sum_{(i',j') \in \text{window}} (z(i',j') - z_{\text{plane}}(i',j'))^2}$$

### 步骤 2：平面分割（Plane Segmentation）

**目标**：把连续的可踩区域分割成若干平面，每个平面用法向量和偏移描述。

这一步为什么重要？因为 MPC 的落脚约束需要知道"脚要落在哪个平面上"——平面的参数（法向量、高度）直接决定了约束的数学形式。

**算法流程**：

```
平面分割算法

Step 1: 连通域标记（Connected Component Labeling）
  输入: 二值可踩性图（0/1）
  方法: 4-连通扫描
  输出: 每个连通区域一个 label

Step 2: 对每个连通域拟合平面
  输入: 某连通域内所有格子的 3D 坐标 {(x_i, y_i, z_i)}
  方法: 计算协方差矩阵 Sigma = (1/N) Sigma (p_i - p_bar)(p_i - p_bar)^T
         最小特征值对应的特征向量 = 平面法向量 n
  检查: 最小特征值 lambda_min < 阈值 → 接受为平面
         lambda_min 过大 → 表面太不平，拒绝

Step 3: 精化分类
  被拒绝的区域标记为"不可踩"
  接受的平面: 记录 (n, p_bar, 边界多边形)
```

**平面参数**：每个被接受的平面由以下参数描述：

$$\text{Plane}_k = (\boldsymbol{n}_k, d_k, \mathcal{B}_k)$$

其中 $\boldsymbol{n}_k$ 是单位法向量，$d_k = \boldsymbol{n}_k \cdot \bar{\boldsymbol{p}}_k$ 是平面偏移，$\mathcal{B}_k$ 是平面的边界（用于后续的 SDF 计算）。

平面方程为：

$$\boldsymbol{n}_k \cdot \boldsymbol{p} = d_k$$

任何在这个平面上的落脚点 $\boldsymbol{p}_{\text{foot}}$ 都应满足此方程（在法向方向上）。

### 步骤 3：落脚点约束的凸不等式形式

**核心问题**：如何把"脚必须落在某个可踩平面上"变成 MPC 能处理的约束？

Grandia 2023 的方法：**为每只脚指定一个目标平面，然后用该平面构造半空间约束（half-space constraint）**。

对于摆动腿 $i$ 在接触时刻 $t_c$ 的落脚约束：

$$\boldsymbol{n}_k^T \boldsymbol{p}_{\text{foot},i}(t_c) = d_k$$

这个等式约束表示"脚必须落在平面上"。但等式约束在有噪声的情况下太严格——实际中用一个**松弛的不等式约束**：

$$|\boldsymbol{n}_k^T \boldsymbol{p}_{\text{foot},i}(t_c) - d_k| \leq \epsilon_z$$

其中 $\epsilon_z$ 是允许的法向偏差（典型 1-2 cm）。

**平面选择**：每个 MPC 周期，根据 Raibert 启发式预测的落脚位置，查询该位置所在的平面。如果预测位置在不可踩区域，则选择最近的可踩平面。这个选择在感知线程以 20 Hz 更新，MPC 线程以 100 Hz 使用（不会每帧都切换平面）。

**为什么这是凸的？** 半空间约束 $\boldsymbol{n}^T \boldsymbol{p} \leq d + \epsilon$ 和 $\boldsymbol{n}^T \boldsymbol{p} \geq d - \epsilon$ 都是线性不等式，天然凸。这使得 HPIPM 可以高效处理——每个落脚约束只增加 2 个线性不等式，计算开销极小。

**与 SDF 约束的配合**：落脚点约束确保脚**最终**落在可踩平面上，SDF 约束确保脚在**摆动过程中**不碰撞地形。两者配合，覆盖了落脚的全过程。

### 对比：直接插值 vs 平面分割

| 方面 | 直接插值高程图 | 平面分割+半空间约束 |
|------|-------------|-------------------|
| 约束光滑性 | 依赖插值质量，边界可能不光滑 | 线性约束，完美光滑 |
| 约束数量 | 每个查询点 1 个 | 每只脚 2 个 |
| 凸性 | 不保证 | 天然凸 |
| 法向量信息 | 需要额外计算 | 平面分割自带 |
| 鲁棒性 | 对高程图噪声敏感 | 平面拟合天然滤噪 |
| 局限 | 无 | 只能描述平面区域 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：平面法向量方向不一致**
> **错误做法**：对不同的可踩区域，法向量可能朝上也可能朝下——取决于特征值分解的符号约定。
> **现象**：某些平面的落脚约束"反了"——MPC 把脚推向地下而不是地面上。
> **根本原因**：特征值分解返回的特征向量只确定了方向，不确定朝向（sign ambiguity）。
> **正确做法**：每次计算法向量后，强制 $n_z > 0$（法向量朝上）。如果 $n_z < 0$，取反 $\boldsymbol{n} \leftarrow -\boldsymbol{n}$。

> 💡 **概念误区：认为所有可踩区域都应该分割为一个平面**
> **新手想法**："弧形坡面也是一个'平面'"
> **实际上**：弧形坡面应该被分割为多个小平面。Grandia 2023 的连通域分割和协方差检查正是为了处理这种情况——如果一个连通域的协方差矩阵最小特征值太大（表面弯曲），该区域会被进一步细分或拒绝。
> **正确理解**：平面分割的"平面"是局部近似。在 4cm 分辨率下，大多数自然地形局部都可以近似为平面。

### 练习

1. **[编程题]** 给定一个 50x50 的高程图矩阵（用 Python/NumPy 随机生成，包含一个台阶和一个斜坡），实现可踩性分类：计算每个格子的坡度和粗糙度，用阈值二值化，可视化结果。

2. **[推导题]** 证明：对于一个水平平面（法向量 $\boldsymbol{n} = [0, 0, 1]^T$），半空间约束 $|\boldsymbol{n}^T \boldsymbol{p}_{\text{foot}} - d| \leq \epsilon_z$ 退化为 $|z_{\text{foot}} - d| \leq \epsilon_z$，即脚的高度必须在平面高度附近。这验证了在平地情况下，Perceptive MPC 的约束等价于盲 MPC 的固定高度约束。

3. **[分析题]** 如果高程图分辨率从 4cm 改为 2cm，平面分割的计算量会怎么变化？连通域标记和协方差计算的复杂度分别是多少？

---

## 67.4 SDF 碰撞避障约束 ⭐⭐⭐

> **本节解决什么问题**：摆动腿在移动过程中如何避开地形障碍——从 SDF 的计算到 MPC 的不等式约束公式化，包括梯度计算和光滑近似。

### 动机：为什么摆动腿需要专门的避障约束

67.3 节解决了"脚落在哪里"的问题。但落脚点是摆动相的**终点**——脚从当前位置到落脚点的**路径**同样需要安全。

```
摆动腿碰撞的典型场景

场景 1: 台阶上方通过
  摆动轨迹: A ──────────── B （低矮弧线）
  台阶:          ████████
  问题: 轨迹穿过台阶实体 → 腿撞台阶

场景 2: 沟渠上方跨越
  摆动轨迹: A ──────────── B
  沟渠:    ██        ██
  问题: 如果抬得不够高，脚尖刮到沟渠边缘

场景 3: 碎石区域
  摆动轨迹: A ── ── ── ── B
  碎石:        ▲  ▲  ▲
  问题: 尖锐碎石可能刮伤腿的侧面
```

盲 MPC 的摆动轨迹是预设的抛物线（抬高-前移-放下），高度固定（如 5cm）。它不知道路径上有什么障碍。感知 MPC 需要一种方法来表达"摆动腿与地形之间的距离不能太近"——这就是 SDF 约束的用途。

### SDF 的数学定义与计算

**定义**：给定地形表面 $\mathcal{S}$，SDF 在任意点 $\boldsymbol{p}$ 的值是该点到最近表面点的有符号距离：

$$\phi(\boldsymbol{p}) = \begin{cases} +\min_{\boldsymbol{q} \in \mathcal{S}} \|\boldsymbol{p} - \boldsymbol{q}\| & \text{if } \boldsymbol{p} \text{ 在自由空间} \\ -\min_{\boldsymbol{q} \in \mathcal{S}} \|\boldsymbol{p} - \boldsymbol{q}\| & \text{if } \boldsymbol{p} \text{ 在障碍内部} \end{cases}$$

- $\phi > 0$：点在自由空间，距离最近障碍表面 $\phi$ 远
- $\phi = 0$：点在障碍表面上
- $\phi < 0$：点在障碍内部（已经穿透）

**SDF 的核心性质——梯度指向远离障碍的方向**：

$$\nabla \phi(\boldsymbol{p}) = \frac{\boldsymbol{p} - \boldsymbol{q}^*}{\|\boldsymbol{p} - \boldsymbol{q}^*\|}$$

其中 $\boldsymbol{q}^*$ 是表面上离 $\boldsymbol{p}$ 最近的点。梯度的物理含义是"离开障碍最快的方向"——这正是优化器需要的信息。当 MPC 的 SQP 计算约束的雅可比时，SDF 的梯度直接提供了"如何修改轨迹来增大与障碍的距离"的方向。

**这为什么如此关键？** 如果用 occupancy grid（二值占据栅格），只知道"某个格子被占据"，没有距离信息也没有梯度。优化器不知道该往哪个方向移动轨迹。SDF 提供了距离和方向，让 SQP 能做有意义的梯度步。

### Grandia 2023 的 2D SDF 计算

在 Grandia 2023 中，SDF 不是直接在 3D 空间计算的——这太昂贵了。取而代之的是**2D SDF**：在高程图的 $(x, y)$ 平面上计算每个格子到最近不可踩区域的水平距离。

**计算步骤**：

```
2D SDF 计算流程

Step 1: 准备二值图
  从可踩性分类得到: steppable(i,j) in {0, 1}
  对所有 steppable = 0 的格子:
    - 加垂直边距: z(i,j) += 2cm（向上抬高不可踩区域）
    - 膨胀一格: 将不可踩标记扩展到相邻格子
  目的: 保守估计，增大安全裕度

Step 2: 距离变换（Distance Transform）
  输入: 二值图（0 = 不可踩/障碍, 1 = 可踩/自由）
  方法: Euclidean Distance Transform (EDT)
  输出: 每个格子到最近 0-格子的 L2 距离
  复杂度: O(N) 使用 Felzenszwalb & Huttenlocher 2012 算法

Step 3: 添加符号
  仅计算到障碍的 EDT 得到的是 unsigned distance；
  若需要真正 signed distance，需要再计算“到自由格”的距离:
    outside = distance_to_obstacle
    inside  = distance_to_free
    sdf(i,j) = outside(i,j) - inside(i,j)
  因此可踩区为正，障碍内部为负，边界附近约为 0。
```

**Euclidean Distance Transform（EDT）**是这里的关键算法。它能在 $O(N)$ 时间内（$N$ 是格子总数）计算所有格子到最近零格子的精确欧氏距离。原理基于"扫描+抛物线包络"：

$$\text{EDT}[i] = \min_j \left( (i - j)^2 + f[j] \right)$$

其中 $f[j]$ 是输入（前一维度的结果）。这个 min 操作的图像是一族抛物线的下包络——可以用线性时间的算法（类似凸包）求解。Felzenszwalb & Huttenlocher 2012 的算法先在 x 方向扫描，再在 y 方向扫描，两次 1D 变换组合得到 2D 结果。

**为什么用 2D 而不是 3D SDF？** 三个原因：

1. **计算量**：3D SDF 需要 $O(N^3)$ 空间和计算，2D 只需 $O(N^2)$。在 4cm 分辨率的 4m x 4m 地图上，2D 是 100x100=10000 格子，3D 是 100x100x50=500000 格子。
2. **更新频率**：2D EDT 在 CPU 上可以亚毫秒完成，3D 需要 GPU。
3. **足够用**：对于四足机器人的摆动腿碰撞回避，水平方向的距离信息比垂直方向更重要——腿主要在水平面上移动，垂直方向用高程图的高度信息就够了。

> **跨领域类比**：SDF 在感知 MPC 中的角色，类似于导航系统中的"距离场地图"——飞行员不需要记住每座山的形状，只需要在每个位置知道"距最近障碍物多远"和"哪个方向远离障碍物"。SDF 把复杂的地形几何压缩为两个信息：距离值（标量）和梯度方向（向量）。这恰好是 MPC 优化器需要的——距离值用于判断约束是否满足，梯度方向用于计算如何修正轨迹以远离障碍物。正如飞行员只需要"terrain proximity warning + escape direction"就能安全飞行，MPC 也只需要 SDF 的值和梯度就能避开障碍。

### SDF 约束的数学公式化

**目标**：在 MPC 的摆动相轨迹上，约束摆动腿末端与地形的距离大于安全裕度 $d_{\min}$。

对于摆动腿 $i$ 在时刻 $t$（摆动相内），约束为：

$$\phi(\boldsymbol{p}_{\text{foot},i}(t)) \geq d_{\min}$$

其中 $\boldsymbol{p}_{\text{foot},i}(t)$ 是脚的位置（通过正运动学从状态 $\boldsymbol{x}$ 计算），$\phi$ 是 SDF 值。

**教学模型中的常见分解**：如果把腿部避障近似为 2D SDF + 高程图高度约束，约束可分解为水平和垂直两个分量：

**水平约束**（使用 2D SDF）：

$$\phi_{2D}(x_{\text{foot}}, y_{\text{foot}}) \geq d_{\min,h}$$

这确保脚在水平方向上远离不可踩区域的边界。

**垂直约束**（使用高程图插值）：

$$z_{\text{foot}} - z_{\text{terrain}}(x_{\text{foot}}, y_{\text{foot}}) \geq d_{\min,v}$$

这确保脚在垂直方向上高于地形表面。$z_{\text{terrain}}$ 通过双线性插值从高程图获得。

### SDF 查询的可微化

SDF 是离散栅格数据。MPC 的 SQP 需要约束对状态的雅可比 $\partial \phi / \partial \boldsymbol{x}$。这需要两步：

**第一步：SDF 的连续化——双线性插值**

给定连续坐标 $(x, y)$，在 SDF 栅格上做双线性插值：

$$\hat{\phi}(x, y) = (1-\alpha)(1-\beta)\phi_{00} + \alpha(1-\beta)\phi_{10} + (1-\alpha)\beta\phi_{01} + \alpha\beta\phi_{11}$$

其中 $\alpha = (x - x_0) / r$，$\beta = (y - y_0) / r$，$\phi_{ij}$ 是四个邻接格子的 SDF 值，$r$ 是栅格分辨率。

**这是连续、分片可微的**。在单个栅格单元内部，对 $x$ 的偏导数：

$$\frac{\partial \hat{\phi}}{\partial x} = \frac{1}{r}\left[ (1-\beta)(\phi_{10} - \phi_{00}) + \beta(\phi_{11} - \phi_{01}) \right]$$

对 $y$ 的偏导数：

$$\frac{\partial \hat{\phi}}{\partial y} = \frac{1}{r}\left[ (1-\alpha)(\phi_{01} - \phi_{00}) + \alpha(\phi_{11} - \phi_{10}) \right]$$

**第二步：链式法则——从 SDF 梯度到状态雅可比**

SQP 需要的是 $\partial \phi / \partial \boldsymbol{x}$（约束对 OCP 状态的雅可比）。通过链式法则：

$$\frac{\partial \phi}{\partial \boldsymbol{x}} = \frac{\partial \hat{\phi}}{\partial \boldsymbol{p}_{\text{foot}}} \cdot \frac{\partial \boldsymbol{p}_{\text{foot}}}{\partial \boldsymbol{x}}$$

其中 $\partial \boldsymbol{p}_{\text{foot}} / \partial \boldsymbol{x}$ 是正运动学的雅可比（Pinocchio 提供），$\partial \hat{\phi} / \partial \boldsymbol{p}_{\text{foot}}$ 来自上面的双线性插值梯度。

**CppAD 与距离场接口的角色**：教学上可以把“正运动学 + 插值”整体写成 `AD<double>` 表达式来理解链式法则。但当前 OCS2 `EndEffectorDistanceConstraintCppAd` 的真实实现更克制：CppADCodeGen 主要用于生成末端执行器正运动学及其雅可比；运行时距离场通过 `DistanceTransformInterface::getValue()` 和 `getLinearApproximation()` 提供 SDF 值与空间梯度，最后显式相乘得到
$$
\frac{\partial \phi}{\partial x}
= \nabla_p \phi(p)^\top \frac{\partial p}{\partial x}.
$$
这样地图和 clearance 可以在运行时更新，而不需要每次地图变化都重新生成 `.so`。

```cpp
// 更接近当前 OCS2 EndEffectorDistanceConstraintCppAd 的结构
auto ee_positions = kinematicsModelPtr_->getFunctionValue(state);
auto ee_jacobians = kinematicsModelPtr_->getJacobian(state);

for (size_t i = 0; i < num_ee; ++i) {
  vector3_t p = ee_positions.segment<3>(3 * i);
  auto [distance, grad_p] = distanceTransform.getLinearApproximation(p);

  g(i) = weight * (distance - clearances(i));
  dfdx.row(i) = weight * grad_p.transpose()
              * ee_jacobians.middleRows<3>(3 * i);
}
```

### 光滑近似：处理 SDF 梯度不连续点

SDF 在某些点的梯度不存在——具体来说，在"等距面"（equidistant loci）上，即到两个最近障碍表面距离相等的点。这些点的 SDF 值连续，但梯度有跳变。

虽然双线性插值能把栅格值变成连续函数，但它只是分片可微，跨格子边界时梯度仍可能跳变。Grandia 2023 还使用了额外的平滑和松弛技巧来降低 SQP 对这些跳变的敏感性：

**技巧 1：高斯模糊 SDF**。在计算 EDT 后，对 SDF 栅格应用一个小核的高斯滤波：

$$\tilde{\phi}(i,j) = \sum_{(i',j') \in \mathcal{N}} w(i',j') \cdot \phi(i',j')$$

这会缓和等距面和栅格边界附近的梯度跳变，让 SQP 看到的局部模型更平滑；代价是 SDF 值不再是严格的几何距离，安全裕度需要留出这部分滤波误差。

**技巧 2：松弛约束的光滑惩罚**。不是用硬约束 $\phi \geq d_{\min}$，而是用光滑的惩罚函数：

$$c(\phi) = \begin{cases} 0 & \text{if } \phi \geq d_{\min} \\ \frac{1}{2\mu}(d_{\min} - \phi)^2 & \text{if } \phi < d_{\min} \end{cases}$$

这是一个二次惩罚，在 $\phi = d_{\min}$ 处连续且一阶可微（值和梯度都为零）。$\mu$ 是松弛参数。OCS2 通过 Augmented Lagrangian 方法（Ch55 已详述）来处理这类松弛约束。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：双线性插值在栅格边界的越界访问**
> **错误做法**：查询坐标 $(x, y)$ 刚好在栅格最后一列时，$i+1$ 越界。
> **现象**：段错误（segfault）或读到垃圾值 --> SDF 约束的梯度异常 --> SQP 发散。
> **正确做法**：调用插值函数之前，由距离场或地图接口把查询坐标裁剪到有效范围，或者在栅格外围加一圈 padding（填充最大安全距离）。当前 OCS2 的 `BilinearInterpolation.h` 只负责给定 `referenceCorner` 和四个角点值后的插值与梯度计算，边界选择应由调用方完成。

> 💡 **概念误区：认为 SDF 约束总是凸的**
> **新手想法**："SDF 本身是连续的，约束 $\phi \geq d$ 应该是凸的"
> **实际上**：SDF 约束一般**不是凸的**。考虑两个障碍之间的狭窄通道：SDF 在通道中心是局部最大值，两侧递减。约束 $\phi \geq d$ 定义的可行域可能是非凸的（两块分离的区域）。这就是为什么 Grandia 2023 用 SQP（局部优化器）而不是全局优化器——SQP 只保证收敛到局部最优，但实时性好。
> **正确理解**：单个障碍外侧的局部线性化常常能给出有效的远离方向，但不要把它理解成全局凸约束。MPC 的初始猜测（warm-start）足够好时，SQP 通常在局部可行区域内工作；初始猜测穿过狭窄通道或障碍边界时，非凸性仍会导致局部最优或收敛失败。

> 🧠 **思维陷阱：认为 2D SDF 足以处理所有碰撞情况**
> **新手想法**："水平距离够了，不需要 3D"
> **反例**：机器人的小腿从一个高台上方摆过——水平距离很远（2D SDF 显示安全），但垂直方向上小腿可能刮到台面。这种情况需要**3D SDF** 或者对小腿的多个检查点做高程图查询。Grandia 2023 为了实时性选择了 2D SDF + 高程图垂直检查的组合，但这在极端地形（如高窄障碍）上可能不足。
> **最新进展**：Jacquet et al. 2025（IJRR）用神经网络编码 3D SDF，在 NMPC 中实现了真正的 3D 碰撞避免。

### 练习

1. **[编程题]** 用 Python 实现 Felzenszwalb & Huttenlocher 的 2D EDT 算法：输入一个二值栅格（0/1），输出每个格子到最近 0-格子的欧氏距离。对一个包含若干矩形障碍的 100x100 栅格测试，用 matplotlib 可视化 SDF 热力图。

2. **[推导题]** 证明双线性插值函数 $\hat{\phi}(x,y)$ 的梯度在栅格内部是连续的，但在栅格边界上（$\alpha = 0$ 或 $\beta = 0$）梯度**可能不连续**。这对 MPC 的 SQP 收敛有什么影响？为什么 Grandia 2023 使用高斯模糊来缓解？

3. **[分析题]** 假设 SDF 的分辨率是 4cm，安全裕度 $d_{\min} = 5$ cm。对于一个宽度为 8cm 的缝隙（两侧都是不可踩区域），SDF 在缝隙中心的值是多少？MPC 能否在这个缝隙中规划出满足约束的轨迹？这说明了 SDF 约束的什么局限性？

---

## 67.5 地形感知代价函数 ⭐⭐

> **本节解决什么问题**：约束保证了可行性（"不撞"、"不踩空"），但 MPC 还需要代价函数来引导**优选方向**——在所有可行轨迹中，哪条最好？

### 动机：约束不够，还需要代价

考虑一个斜坡场景：所有满足落脚约束和 SDF 约束的轨迹都是"可行的"。但有些轨迹让机器人的重心过高（不稳定），有些让膝关节过度弯曲（力矩大），有些让躯体倾斜过大（不舒适）。代价函数的作用是在可行域内选择"最好的"轨迹。

Grandia 2023 在盲 MPC 的代价函数基础上增加了三个地形感知代价项。

### 代价项 1：躯体高度跟踪（Body Height Tracking）

**问题**：盲 MPC 维持固定的躯体高度（如 0.48m）。在斜坡上，这导致上坡时腿伸直、下坡时腿过弯。

**解决**：让躯体高度跟踪**地形自适应的参考高度**：

$$J_{\text{height}} = w_h \left( z_{\text{body}} - z_{\text{ref}}(x_{\text{body}}, y_{\text{body}}) \right)^2$$

其中 $z_{\text{ref}}(x, y)$ 是地形参考高度——从高程图计算的、机器人躯体应该处于的高度。

**参考高度的计算**：取躯体投影下方四个脚的落脚平面高度的加权平均，再加上标称站立高度：

$$z_{\text{ref}}(x, y) = \bar{z}_{\text{terrain}}(x, y) + h_{\text{nominal}}$$

其中 $\bar{z}_{\text{terrain}}$ 是四脚落脚平面高度的均值，$h_{\text{nominal}} = 0.48\text{m}$（ANYmal 的标称站立高度）。

**为什么用平面高度而不是直接查高程图？** 因为高程图在碎石区域有很大的局部波动——如果直接用高程图的高度，参考高度会随碎石一起"抖动"，导致躯体上下振荡。用平面拟合后的高度天然滤掉了局部噪声。

### 代价项 2：躯体避障（Body Clearance）

**问题**：在台阶或大石头旁边行走时，即使脚没有碰到障碍，**躯体**可能撞到。特别是 ANYmal 的躯体宽约 0.5m，在狭窄通道或大石头旁边时需要注意。

**解决**：用 SDF 对躯体位置也加一个代价：

$$J_{\text{clearance}} = w_c \cdot \max(0, d_{\text{body}} - \phi_{2D}(x_{\text{body}}, y_{\text{body}}))^2$$

其中 $d_{\text{body}}$ 是躯体的最小安全距离（考虑躯体宽度），$\phi_{2D}$ 是 2D SDF。当 SDF 值小于安全距离时，代价增大，推动 MPC 让躯体远离障碍。

**注意**：这是一个**代价项**而非约束——允许躯体偶尔靠近障碍（代价增大但不禁止），因为在狭窄环境中完全避免可能导致 MPC 不可行。

### 代价项 3：摆动腿地形回避（Swing Terrain Avoidance）

**问题**：SDF 约束保证了脚不穿透地形，但约束是"硬边界"——MPC 可能让脚刚好擦过地形表面，没有余量。

**解决**：在 SDF 约束之外，额外加一个**软代价**，鼓励摆动腿远离地形表面：

$$J_{\text{swing}} = w_s \sum_{t \in \text{swing}} \max(0, d_{\text{swing}} - \phi(\boldsymbol{p}_{\text{foot}}(t)))^2$$

这是一个"安全裕度代价"——即使满足了 SDF 约束（$\phi \geq d_{\min}$），如果 $\phi < d_{\text{swing}}$（$d_{\text{swing}} > d_{\min}$），也会有额外代价，鼓励 MPC 保持更大裕度。

### 三个代价项的权重调节

| 代价项 | 权重符号 | 典型值范围 | 过大后果 | 过小后果 |
|--------|---------|-----------|---------|---------|
| 躯体高度跟踪 | $w_h$ | 50-200 | 躯体僵硬跟踪地形，忽略其他目标 | 躯体高度不适应地形，腿过伸/过弯 |
| 躯体避障 | $w_c$ | 10-50 | 机器人不敢靠近任何障碍 | 躯体可能碰撞大障碍 |
| 摆动腿回避 | $w_s$ | 20-100 | 摆动高度过大，浪费能量 | 脚擦过地形表面，缺乏安全裕度 |

**调参原则**：在不同地形上需要不同的权重。Grandia 2023 使用了一组**固定权重**在所有测试地形上都表现良好，但最新的工作（如 DTC, Jenelten 2024）开始用 RL 学习权重。

### 代价函数的完整组合

把地形感知代价加到盲 MPC 的代价上：

$$J = \underbrace{J_{\text{vel}} + J_{\text{ori}} + J_{\text{input}}}_{\text{盲 MPC 基础代价}} + \underbrace{J_{\text{height}} + J_{\text{clearance}} + J_{\text{swing}}}_{\text{地形感知代价}}$$

其中基础代价包括：
- $J_{\text{vel}}$：速度跟踪（跟踪用户指令的线速度和角速度）
- $J_{\text{ori}}$：姿态跟踪（保持躯体水平或跟踪参考姿态）
- $J_{\text{input}}$：控制输入正则化（惩罚过大的关节力矩）

**地形感知代价只是"增量"**——它不改变盲 MPC 的基础结构，只是在上面加了三项。这使得系统可以优雅降级：如果感知失效，把地形感知代价的权重设为零，就回到了盲 MPC。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：代价函数中 max(0, x) 的不可微点**
> **错误做法**：直接在代价中用 `std::max(0.0, x)`。
> **现象**：当 $x$ 从负变正时，CppAD 计算的梯度在 $x=0$ 处不连续 --> SQP 的二次子问题不正定 --> 求解失败。
> **正确做法**：用光滑近似，如 softplus：$\text{softplus}(x) = \frac{1}{\beta}\log(1 + e^{\beta x})$，其中 $\beta$ 控制光滑程度。或者用 Huber-like 函数在零点附近做二次过渡。OCS2 的 `SoftConstraintPenalty` 类提供了这种光滑近似。

> 💡 **概念误区：认为代价和约束可以互相替代**
> **新手想法**："既然有 SDF 约束了，为什么还要 SDF 代价？"
> **实际上**：约束定义"可行边界"（绝对不能越过），代价定义"偏好方向"（越远越好）。只有约束没有代价时，MPC 可能让脚刚好擦过地形表面——技术上可行，但缺乏安全裕度，任何扰动都可能导致碰撞。代价在约束之外提供了"软性引导"。
> **类比**：约束是"悬崖边的护栏"，代价是"请远离护栏的警告标志"。

### 练习

1. **[计算题]** 假设 ANYmal 在 15 度斜坡上行走，标称站立高度 0.48m。计算地形自适应参考高度 $z_{\text{ref}}$ 在斜坡上某点的值（已知该点地面高度 0.3m）。与盲 MPC 的固定参考高度 0.48m 相比，差了多少？

2. **[设计题]** 如果你要为一个双足人形机器人设计地形感知代价函数（与四足不同，双足只有两个支撑点），你会怎么修改三个代价项？特别是躯体高度跟踪——双足的参考高度应该怎么计算？

3. **[分析题]** 解释为什么在平地上，三个地形感知代价项的值都趋近于零，感知 MPC 的行为退化为盲 MPC。这是通过什么数学机制保证的？

---

## 67.6 Swing 轨迹地形适应 ⭐⭐

> **本节解决什么问题**：MPC 输出的摆动腿轨迹需要适应地形——在台阶上抬高、在沟渠上跨越、在碎石区域绕行。这里的"适应"不仅是约束层面的（不碰撞），更是轨迹形状层面的（摆动高度、速度、形状应根据地形调整）。

### 动机：固定摆动轨迹的局限

盲 MPC 中的摆动轨迹通常是预设的样条曲线——从起点到终点的抛物线，最大高度固定（如 5cm 或 8cm）。这在平地上足够了，但在崎岖地形上有严重问题：

```
固定摆动轨迹的失效场景

场景 1: 上台阶（10cm 高）
  固定抬高 5cm → 抬高不够 → 脚趾撞台阶面
  需要: 抬高至少 15cm（台阶高度 + 安全裕度）

场景 2: 跨沟渠（8cm 宽，5cm 深）
  固定抬高 5cm → 刚好与沟渠边缘齐平
  任何下垂都会让脚掉进沟渠
  需要: 增大步幅 + 增大抬高

场景 3: 下台阶（10cm 高）
  固定放下高度 → 脚在空中悬停太久 → 冲击力大
  需要: 提前开始下降，减小着地冲击
```

### 地形感知摆动轨迹生成

Grandia 2023 不是预设固定的摆动轨迹，而是让 **MPC 优化器自己决定摆动轨迹**——通过地形约束和代价，SQP 会自动找到避开地形的最优摆动路径。

但 MPC 需要一个好的**初始猜测**（warm-start）。如果初始猜测的摆动轨迹穿过地形，SQP 可能无法收敛（约束违反太大）。因此，感知管线还要提供一个**地形感知的摆动轨迹初始猜测**。

**初始猜测的生成算法**：

```
地形感知摆动轨迹初始猜测

输入: 起点 p_start (当前脚位置)
      终点 p_end (目标落脚点，来自落脚规划)
      地形高程图 M
      SDF 场 phi

Step 1: 计算路径上的最大地形高度
  沿 p_start → p_end 的直线，采样 N 个点
  对每个采样点查询高程图: z_terrain(x_i, y_i)
  z_max = max(z_terrain(x_i, y_i)) for all i

Step 2: 计算安全抬高高度
  h_swing = max(h_min, z_max - min(z_start, z_end) + d_clearance)
  其中:
    h_min = 3cm (最小抬高，平地也需要)
    d_clearance = 5cm (安全裕度)

Step 3: 生成参数化摆动曲线
  使用三段样条:
    段 1 (抬起): p_start → p_mid_up (垂直抬起)
    段 2 (前移): p_mid_up → p_mid_down (水平+向下)
    段 3 (放下): p_mid_down → p_end (垂直放下)
  
  p_mid_up = p_start + [0, 0, h_swing]
  p_mid_down = p_end + [0, 0, h_swing * 0.3]

输出: 摆动轨迹的初始猜测 p_swing(t), t in [t_liftoff, t_touchdown]
```

**MPC 在此基础上进一步优化**：初始猜测提供了一条"大致安全"的轨迹，MPC 的 SQP 会在此基础上微调，考虑动力学约束（关节速度限制、力矩限制）和代价函数（最小化能量、保持平衡）。最终的轨迹通常比初始猜测更平滑、更高效。

### 特殊地形的摆动策略

| 地形 | 关键调整 | 物理原因 |
|------|---------|---------|
| **上台阶** | 大幅增大抬高，先垂直抬起再水平移 | 避免脚趾碰撞台阶面 |
| **下台阶** | 提前下降，减小着地速度 | 减小着地冲击力 |
| **跨沟渠** | 增大步幅 + 增大抬高 | 确保脚完全越过沟渠 |
| **碎石区域** | 中等抬高，快速放下 | 减少空中时间，快速建立支撑 |
| **斜坡** | 抬高略增，倾斜放下方向 | 适应斜坡法向量 |
| **stepping stones** | 精确瞄准，最小水平偏移 | 落脚面积有限，偏差不可接受 |

### 摆动时间分配

地形还影响**摆动相的时间分配**。在复杂地形上，MPC 可能需要：

- **减慢摆动速度**：给足够时间绕开障碍
- **增加触地时的减速段**：减小着地冲击
- **缩短空中时间**：在不稳定地形上，三脚支撑比两脚支撑更稳定，所以应尽快恢复四脚支撑

Grandia 2023 通过 MPC 的代价函数间接控制摆动时间——控制输入正则化惩罚过大的关节速度，使得在需要大幅度移动时自动减慢。但步态时序（liftoff/touchdown 时刻）仍然是预设的（Ch56 的步态调度器决定），不由 MPC 优化。

> **本质洞察**：感知 MPC 的核心工程难题**不是算法本身,而是"离散感知"与"连续优化"之间的阻抗匹配**。高程图是离散的（4cm 栅格）、异步更新的（20 Hz），而 MPC 是连续的（连续状态/输入空间）、同步执行的（100 Hz）。Grandia 2023 的所有工程创新——双线性插值、SDF 预计算、凸区域分解、双线程架构——都是在解决同一个根本问题：如何让离散的感知数据无缝地流入连续的优化管线,既不引入非光滑性（破坏 SQP 收敛），也不引入过大延迟（破坏实时性）。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：摆动轨迹初始猜测穿过地形**
> **错误做法**：初始猜测简单地从起点到终点做线性插值，不考虑中间的地形。
> **现象**：SQP 第一次迭代时约束违反量极大 --> 线搜索步长几乎为零 --> SQP 不收敛 --> MPC 超时 --> 使用上一帧的旧策略 --> 控制器反应迟钝。
> **正确做法**：初始猜测必须"基本安全"——至少不穿过地形。上面的算法通过查询路径上的最大地形高度来保证这一点。

> 💡 **概念误区：认为 MPC 直接优化摆动轨迹的每个点**
> **新手想法**："MPC 在每个时间步都独立优化脚的位置"
> **实际上**：MPC 优化的是**状态轨迹**（包括关节角度和速度）。脚的位置是状态通过正运动学的函数。所以 MPC 是间接优化摆动轨迹——通过约束关节角度来约束脚的运动。这意味着摆动轨迹自动满足运动学约束（关节限位、速度限制），不需要额外检查。

### 练习

1. **[编程题]** 给定一个包含 10cm 台阶的高程图，实现摆动轨迹初始猜测算法：从台阶底部到台阶顶部生成一条地形感知的摆动曲线。用 matplotlib 3D 可视化轨迹和地形。

2. **[分析题]** 在 stepping stones 场景中（离散踏板，间距 30cm，踏板面积 10cm x 10cm），摆动轨迹的水平精度要求是多少？如果 MPC 的控制频率是 100 Hz，每个控制周期的水平位置变化是多少？这说明了 MPC 频率对落脚精度的什么关系？

3. **[思考题]** Grandia 2023 的步态时序是预设的。如果把步态时序也作为 MPC 的优化变量（自由步态时序），Perceptive MPC 会更好还是更差？（提示：考虑优化变量增加对 SQP 收敛速度的影响，以及自由时序对 stepping stones 场景的潜在优势。）

---

## 67.7 OCS2 Perceptive 实现对照 ⭐⭐⭐

> **本节解决什么问题**：把理论落实到代码——对照 OCS2 `ocs2_perceptive` 的真实接口和本章的教学简化模块，理解从距离场到 MPC 约束的代码路径。下面凡是标为“教学简化”的文件名都不是承诺真实仓库中存在同名文件。

### 动机：为什么要读源码

理解算法和理解实现是两回事。论文告诉你"用插值/线性化让距离场可用于 SQP"，但代码告诉你：
- 边界条件怎么处理？
- 距离场的值和梯度由谁提供？
- CppADCodeGen 生成的是整条 SDF 查询链，还是只生成末端运动学？
- clearance、地图和机器人模型变化分别会不会触发重新生成？

### 真实 `ocs2_perceptive` 核心结构

```
ocs2_perceptive/
├── include/ocs2_perceptive/
│   ├── interpolation/
│   │   ├── BilinearInterpolation.h        ← 核心：SDF/高程图的可微查询
│   │   └── TrilinearInterpolation.h       ← 3D 栅格插值
│   ├── distance_transform/
│   │   ├── ComputeDistanceTransform.h     ← EDT 计算模板
│   │   └── DistanceTransformInterface.h   ← 距离场值/梯度接口
│   ├── end_effector/
│   │   ├── EndEffectorDistanceConstraint.h           ← 基础约束类
│   │   └── EndEffectorDistanceConstraintCppAd.h      ← CppAD 可微版本
├── src/
│   └── end_effector/
│       ├── EndEffectorDistanceConstraint.cpp
│       └── EndEffectorDistanceConstraintCppAd.cpp
├── test/
│   └── interpolation/
│       ├── testBilinearInterpolation.cpp
│       └── testTrilinearInterpolation.cpp
└── CMakeLists.txt
```

> **版本提示**：上述结构对应当前 OCS2 main 分支的核心文件。分支和发行版可能变化，源码阅读时以本机 checkout 为准。ANYmal perceptive 示例还会引入 `segmented_planes_terrain_model`、`grid_map_sdf` 等工程包，它们不全在 `ocs2_perceptive` 目录内。

### 真实模块 1：`BilinearInterpolation.h`

这个模块实现双线性插值的值和一阶近似，用来解释 OCS2 感知约束中“地图查询 + 梯度”的核心思想。当前 OCS2 main 分支中的接口并不接收整张栅格图，也不在函数内部查找整数索引；它只接收当前单元的参考角点 `referenceCorner`、四个角点值 `cornerValues` 和查询点 `position`：

```cpp
namespace ocs2::bilinear_interpolation {

template <typename Scalar>
Scalar getValue(
    Scalar resolution,
    const Eigen::Matrix<Scalar, 2, 1>& referenceCorner,
    const std::array<Scalar, 4>& cornerValues,
    const Eigen::Matrix<Scalar, 2, 1>& position);

template <typename Scalar>
std::pair<Scalar, Eigen::Matrix<Scalar, 2, 1>> getLinearApproximation(
    Scalar resolution,
    const Eigen::Matrix<Scalar, 2, 1>& referenceCorner,
    const std::array<Scalar, 4>& cornerValues,
    const Eigen::Matrix<Scalar, 2, 1>& position);

}  // namespace ocs2::bilinear_interpolation
```

把“选哪四个格子”和“在这四个格子内部插值”拆开，是这个接口最重要的设计。距离场实现先在普通运行时代码中决定查询点落在哪个栅格单元，完成边界裁剪并取出四个角点值；随后 `getLinearApproximation()` 只计算当前单元内部的连续值和二维梯度：

```cpp
// 运行时距离场接口内部的典型调用形态，省略边界裁剪和格子索引细节。
Eigen::Vector2d referenceCorner = gridCornerWorldPosition(i, j);
std::array<double, 4> cornerValues = {
    sdf(i, j), sdf(i + 1, j), sdf(i, j + 1), sdf(i + 1, j + 1)
};
Eigen::Vector2d position(x, y);

auto [distance, grad_xy] =
    ocs2::bilinear_interpolation::getLinearApproximation(
        resolution, referenceCorner, cornerValues, position);
```

**关键设计：整数索引与连续插值分离**

整数索引（选择哪四个格子）不是查询坐标的连续函数，它是阶跃函数。真实实现先在普通 `double` 空间确定参考角点和四个角点值，再调用 `bilinear_interpolation::getValue()` 或 `getLinearApproximation()` 计算值与梯度。这样做的含义是：**在当前格子内部线性化，跨格子时由下一次查询重新选择角点**。

不要误解为“CppAD 可以对任意地图索引自动求导”。若在录制函数里从 AD 变量强行取出普通数值，索引会变成 tape 的参数/常量，运行时不能自动跳到新的格子。OCS2 的距离约束通过 `DistanceTransformInterface::getLinearApproximation()` 在运行时提供局部值和梯度，避开了把整张地图录进 AD tape 的问题。

这个设计有一个微妙后果：当查询点移动越过格子边界时，梯度可能跳变（因为参与插值的四个角点变了）。工程上依赖三件事来降低影响：较细的地图分辨率、良好的 warm start，以及对 SDF/惩罚函数的适度平滑。不能把它理解成全局光滑函数。

### 教学简化模块 2：`TeachingSignedDistanceTransform`

这个教学模块解释从二值可踩性图到 2D SDF 的计算链路。当前 OCS2 main 分支没有 `TeachingSignedDistanceTransform` 这个文件，也没有把下面这些步骤封装成同名类；真实代码中请查看 `ComputeDistanceTransform.h`、`DistanceTransformInterface.h`，以及 perceptive ANYmal 示例中的 `SegmentedPlanesSignedDistanceField` / `grid_map_sdf` 相关链路。下面代码是概念伪代码，用来表达数据流，不是可直接编译的源码。

```cpp
// 概念伪代码：说明 SDF 计算流程，不对应 OCS2 中的真实类名。
class TeachingSignedDistanceTransform {
public:
  void compute(const grid_map::GridMap& elevation_map) {
    // 1. 从高程图计算可踩性（坡度+粗糙度+台阶检查）
    Eigen::MatrixXi steppability = classifySteppability(elevation_map);
    
    // 2. 膨胀不可踩区域（安全裕度）
    dilateObstacles(steppability, dilation_radius_);  // 典型 1-2 个格子
    
    // 3. 添加垂直边距
    addVerticalMargin(elevation_map, steppability, vertical_margin_);  // 典型 2cm
    
    // 4. 分别计算到障碍和到自由区的 Euclidean Distance Transform
    Eigen::MatrixXf distance_to_obstacle =
        euclideanDistanceTransform(/*zero set = non-steppable cells*/);
    Eigen::MatrixXf distance_to_free =
        euclideanDistanceTransform(/*zero set = steppable cells*/);
    
    // 5. 添加符号
    for (int i = 0; i < rows_; ++i)
      for (int j = 0; j < cols_; ++j)
        sdf_(i, j) = distance_to_obstacle(i, j) - distance_to_free(i, j);
    
    // 6. 可选：高斯模糊
    if (smooth_sigma_ > 0)
      gaussianBlur(sdf_, smooth_sigma_);
  }
  
  // 查询接口（MPC 线程调用）：真实实现还要做边界裁剪、
  // 角点选择、双线性插值和梯度返回。
  std::pair<float, Eigen::Vector2f> queryLinearApproximation(float x, float y) const {
    return lookupCurrentCellAndInterpolate(sdf_, x, y, resolution_, origin_x_, origin_y_);
  }
  
private:
  Eigen::MatrixXf sdf_;
  float resolution_, origin_x_, origin_y_;
  float dilation_radius_, vertical_margin_, smooth_sigma_;
};
```

### 真实接口 3：`EndEffectorDistanceConstraint(CppAd)`

`EndEffectorDistanceConstraint.h` 和 `EndEffectorDistanceConstraintCppAd.h` 把距离场查询包装成 OCS2 约束接口。非 CppAD 版本使用普通运动学线性化；CppAD 版本用 `CppAdInterface` 生成末端位置关于状态的雅可比。两者共同点是：距离场本身通过 `set(clearance, distanceTransform)` 在运行时注入。

```cpp
// 当前 OCS2 结构的简化版
class EndEffectorDistanceConstraintCppAd : public StateInputConstraint {
public:
  void set(vector_t clearances,
           const DistanceTransformInterface& distanceTransform) {
    clearances_ = std::move(clearances);
    distanceTransformPtr_ = &distanceTransform;
  }

  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t t, const vector_t& state, const vector_t& input,
      const PreComputation& preComp) const override {
    auto eePositions = kinematicsModelPtr_->getFunctionValue(state);
    auto eeJacobians = kinematicsModelPtr_->getJacobian(state);

    VectorFunctionLinearApproximation approx =
        VectorFunctionLinearApproximation::Zero(numEEs, stateDim_, inputDim_);
    for (size_t i = 0; i < numEEs; ++i) {
      auto [distance, grad] =
          distanceTransformPtr_->getLinearApproximation(
              eePositions.segment<3>(3 * i));
      approx.f(i) = weight * (distance - clearances_(i));
      approx.dfdx.row(i) = weight * grad.transpose()
                          * eeJacobians.middleRows<3>(3 * i);
    }
    return approx;
  }

private:
  std::unique_ptr<CppAdInterface> kinematicsModelPtr_;
  const DistanceTransformInterface* distanceTransformPtr_ = nullptr;
  vector_t clearances_;
};
```

### 配置文件解读

OCS2 perceptive 的配置通过 `.info` 文件管理（与 Ch55 中的 `task.info` 类似）：

```ini
; perceptive_task.info 关键配置项

[perception]
elevation_map_topic = "/elevation_mapping/elevation_map"
map_resolution = 0.04          ; 4cm 分辨率
map_size_x = 4.0               ; 4m x 4m 范围
map_size_y = 4.0

[steppability]
max_slope = 0.6                ; ~34 度 (tan 值)
max_roughness = 0.05           ; 5cm
max_step_height = 0.15         ; 15cm

[sdf]
dilation_radius = 1            ; 膨胀 1 个格子
vertical_margin = 0.02         ; 2cm 垂直边距
smooth_sigma = 0.5             ; 高斯模糊标准差（格子数）
min_distance = 0.05            ; 5cm 最小安全距离

[cost_weights]
body_height_tracking = 100.0
body_clearance = 30.0
swing_terrain_avoidance = 50.0

[solver]
mpc_frequency = 100            ; Hz
sqp_iterations = 1             ; RTI: 只做 1 次 SQP 迭代
hpipm_mode = "SPEED"           ; HPIPM 求解模式
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：分清运行时参数和 AD 代码生成内容**
> **错误理解**：修改了 SDF clearance / `min_distance` 后，必须删除 CppADCodeGen 的 `.so` 文件。
> **实际情况**：在 OCS2 的 `EndEffectorDistanceConstraint(CppAd)` 中，clearance 和距离场通过 `set(clearance, distanceTransform)` 在运行时传入；修改这类配置不需要重新生成运动学 `.so`。
> **什么时候才需要重生成**：改变 AD 表达式结构、状态维度、末端数量、机器人模型或被录进 tape 的常量时，旧的生成库才会失效。修改地图、clearance、权重等运行时数据，应优先检查配置是否被节点重新加载，而不是盲目删除代码生成缓存。

> 💡 **概念误区：认为 `ocs2_perceptive` 是独立模块，可以单独使用**
> **新手想法**："我只编译 `ocs2_perceptive` 就够了"
> **实际上**：`ocs2_perceptive` 依赖 `ocs2_core`（基础类型）、`ocs2_oc`（最优控制接口）、`ocs2_pinocchio`（运动学）、`ocs2_robotic_tools`（工具函数）等多个模块。它不是独立可用的——必须在 OCS2 的完整编译环境中使用。推荐用 catkin 或 colcon 编译整个 OCS2 workspace。

### 练习

1. **[源码阅读题]** 克隆 OCS2 仓库（https://github.com/leggedrobotics/ocs2），找到 `ocs2_perceptive/include/ocs2_perceptive/interpolation/BilinearInterpolation.h`。阅读完整实现，回答：(a) `getValue()` 与 `getLinearApproximation()` 的输入分别是什么？(b) 为什么它不接收整张栅格图？(c) 边界裁剪和四个角点选择应该由哪一层代码负责？

2. **[源码阅读题]** 阅读 `EndEffectorDistanceConstraintCppAd.h` 的完整实现。画出从 MPC 调用 `getLinearApproximation()` 到最终得到雅可比矩阵的完整调用链（包括 CppADCodeGen 的 `.so` 加载和调用）。

3. **[实操题]** 修改 OCS2 perceptive 的配置文件，把 `min_distance` 从 5cm 改为 10cm。观察 MPC 在仿真中的行为变化——摆动腿是否抬得更高了？有没有出现 SQP 不收敛的情况？

---

## 67.8 RL vs MPC 感知运动对比 ⭐⭐⭐

> **本节解决什么问题**：系统对比两种感知运动方案——Miki 2022 的 RL 方案（Science Robotics）和 Grandia 2023 的 MPC 方案（T-RO）——帮助理解各自的优势、局限和适用场景。

### 动机：为什么需要对比

2022-2023 年，ETH 同一个实验室（RSL）同时产出了两篇标杆论文，分别代表感知运动控制的两条技术路线。它们的目标相同（让四足机器人在崎岖地形上稳健行走），但方法论截然不同。理解这两条路线的差异和互补性，对于选择自己的研究方向至关重要。

### Miki 2022：RL 方案概述

**论文**：Miki T., Lee J., Hwangbo J., et al. (2022) "Learning robust perceptive locomotion for quadrupedal robots in the wild" -- Science Robotics, Vol. 7, eabk2822.

**核心思想**：用强化学习在仿真中训练一个**端到端**的感知运动策略，直接从本体感知（关节角度、IMU）和外界感知（高程图扫描）映射到关节目标位置。

**架构**：

```
Teacher-Student 训练框架

训练阶段（仿真环境，Isaac Gym）：
  Teacher 策略:
    输入: 特权信息（精确地形、精确状态、接触力）
    输出: 关节目标位置
    训练: PPO，奖励 = 速度跟踪 + 稳定性 + 能耗
    
  Student 策略:
    输入: 可观测信息（关节角/速度、IMU、高程图扫描）
    输出: 关节目标位置
    训练: 行为克隆 Teacher + PPO 微调
    
  关键模块——注意力编码器（Attention-based Encoder）:
    输入: 本体感知 + 外界感知（高程图扫描点）
    输出: 潜在表示 z（用于 Student 策略）
    作用: 学习融合不同模态的感知，自动处理遮挡/噪声

部署阶段（真实 ANYmal）：
  只部署 Student 策略
  输入: 真实传感器数据 → 推理 → 关节 PD 目标
  频率: 100 Hz（神经网络推理）
```

### Grandia 2023：MPC 方案概述

（前面几节已详述，这里做简要总结以便对比。）

**核心思想**：把地形信息显式嵌入 NMPC 的代价函数和约束中，用 SQP+HPIPM 实时求解全自由度最优轨迹。

**架构**：感知管线（20 Hz）--> NMPC（100 Hz）--> WBC（400 Hz）

### 全面对比

| 维度 | Miki 2022 (RL) | Grandia 2023 (MPC) |
|------|----------------|-------------------|
| **方法论** | 端到端学习 | 基于模型的优化 |
| **感知处理** | 隐式（编码器自动学习） | 显式（SDF、平面分割） |
| **动力学模型** | 不需要（仿真中隐式学习） | 需要（Centroidal/全身模型） |
| **训练/调参** | 训练 24-48h（GPU 集群），奖励函数设计 | 无训练，代价/约束权重调参 |
| **部署推理** | 神经网络前向传播（<1ms） | SQP 求解（5-10ms） |
| **可解释性** | 低（黑盒策略） | 高（每个约束/代价都有物理含义） |
| **安全保证** | 无理论保证（依赖训练覆盖） | 约束满足有理论保证（SQP 收敛时） |
| **泛化能力** | 强（训练中见过多样地形） | 弱（依赖精确的感知前端） |
| **极端地形** | 更好（见过极端训练样本） | 受限（SQP 可能不收敛） |
| **新机器人适配** | 需要重新训练（sim-to-real gap） | 更换 URDF + 配置即可 |
| **实时修改** | 困难（需要重新训练） | 容易（修改配置文件） |
| **感知失效处理** | 隐式降级（编码器学会忽略坏数据） | 显式降级（切换到盲 MPC） |

### 更深层的差异分析

**差异 1：如何处理感知不确定性**

MPC 方案把感知数据当作"确定性输入"——高程图说高度是 0.3m，MPC 就假设是 0.3m。如果高程图有 3cm 噪声，MPC 的轨迹就有 3cm 误差。Grandia 2023 通过高斯模糊和安全裕度来部分补偿，但没有**概率框架**。

RL 方案在训练时就见过各种噪声水平的感知数据（domain randomization），编码器自动学会了在不确定时"保守行动"。这种鲁棒性不是设计出来的，而是训练出来的。

**差异 2：如何利用预测视野**

MPC 的核心优势是**多步预测**——在 1-2 秒的视野内优化未来轨迹。这在 stepping stones 等需要前瞻规划的场景中至关重要。

RL 策略通常是**反应式**的——只看当前观测，输出当前动作。虽然 LSTM/Transformer 可以隐式编码短期历史，但没有显式的多步规划能力。

**差异 3：如何应对新场景**

当机器人遇到训练/设计时没有考虑的新场景（如泥地、冰面、动态障碍），两种方案的表现不同：

- **MPC**：如果感知前端能提供正确的地形信息，MPC 自动适应（因为优化是在线的）。但如果感知失效（如泥地没有被正确标记为不可踩），MPC 会生成不安全的轨迹。
- **RL**：如果新场景与训练分布"足够接近"，策略可以泛化。但如果完全 out-of-distribution，策略的行为不可预测——可能突然摔倒，而且没有可解释的原因。

### 混合方案：DTC 和未来方向

鉴于 RL 和 MPC 各有优势，近年的趋势是**混合**：

**DTC（Deep Tracking Control, Jenelten 2024, Science Robotics）**：RL 策略输出**参考轨迹**，MPC 负责跟踪。RL 处理感知理解（高层决策），MPC 处理动力学约束（低层执行）。

**RL-augmented MPC（2023-2025 多项工作）**：用 RL 学习 MPC 的代价函数权重或终端代价，使 MPC 能适应更多场景。

```
混合方案的演进

Level 1: 独立
  RL 和 MPC 各做各的，选一个部署

Level 2: 串联（DTC）
  RL → 参考轨迹 → MPC → 控制
  
Level 3: 融合（RL-augmented MPC）
  RL 学习 MPC 的代价/约束参数 → MPC 优化
  
Level 4: 统一（未来方向）
  可微 MPC 作为 RL 的一个层 → 端到端训练
  (Amos & Kolter 2017 "OptNet", Agrawal 2019 "differentiable MPC")
```

### ⚠️ 常见陷阱

> 🧠 **思维陷阱：认为"RL 比 MPC 好"或"MPC 比 RL 好"**
> **新手想法**："看了 ANYmal Parkour (RL) 的视频，MPC 完全不行啊"
> **实际上**：RL 在极限性能上确实超越 MPC（跳跃、翻越、跑酷）。但在工业部署中，MPC 的优势是**可解释性**和**可调性**——当机器人在客户现场出问题时，工程师可以看 MPC 的约束违反记录找到原因，但 RL 策略只能"重新训练一个版本试试"。
> **正确思维**：选择取决于应用场景。工业巡检（安全优先）--> MPC；极端地形探索（性能优先）--> RL；复杂但需可靠（两者兼顾）--> 混合。

> 💡 **概念误区：认为 Miki 2022 不需要任何模型**
> **新手想法**："RL 是 model-free 的，完全不需要动力学模型"
> **实际上**：Miki 2022 的训练需要一个**精确的仿真环境**（Isaac Gym + ANYmal URDF + 地形生成器）。仿真本身就是一个模型——而且对 sim-to-real 的成功至关重要。如果仿真模型不准确（如接触模型错误），训练出来的策略无法迁移到真实机器人。
> **正确理解**：RL 不是 "model-free"——它只是不在**策略执行时**显式使用模型，但在**训练时**隐式依赖仿真模型。

### 练习

1. **[对比分析题]** 假设你需要为一个在建筑工地巡检的四足机器人选择控制方案。工地有临时堆放的建材（动态障碍）、斜坡、脚手架（狭窄通道）、积水。你会选 RL、MPC 还是混合方案？给出理由，列出各方案的风险。

2. **[设计题]** 设计一个 "Level 3" 的 RL-augmented MPC 系统：RL 学习 MPC 的三个地形感知代价权重（$w_h, w_c, w_s$）。RL 的观测空间、动作空间、奖励函数分别应该是什么？

3. **[论文阅读题]** 阅读 Miki et al. 2022 的 Section III (Method)。回答：(a) Teacher 策略的特权信息包括哪些？(b) 注意力编码器为什么比 MLP 更适合融合多模态感知？(c) Domain randomization 随机化了哪些参数？

---

## 67.9 部署实践 ⭐⭐

> **本节解决什么问题**：从实验室到真实世界的部署，需要解决延迟预算、计算资源分配、降级策略等工程问题。

### 动机：算法正确不等于部署成功

一个在仿真中完美工作的 Perceptive MPC 系统，部署到真实机器人上可能面对：
- 传感器延迟比仿真中大 3 倍
- CPU 负载导致 MPC 频率降到 60 Hz
- 阳光直射导致深度相机失效
- 机器人振动导致点云模糊

### 延迟预算

部署的首要任务是建立**延迟预算表**（latency budget）——每个模块允许多少毫秒：

| 模块 | 预算 | 实测（ANYmal D） | 余量 | 超预算后果 |
|------|------|----------------|------|-----------|
| 点云获取 | 10 ms | 5-8 ms | 充足 | 地图更新延迟 |
| Elevation Mapping | 15 ms | 10-15 ms | 紧张 | 地图过期 |
| 可踩性+分割+SDF | 10 ms | 5-8 ms | 充足 | MPC 用旧数据 |
| **MPC 求解** | **10 ms** | **5-10 ms** | **紧张** | **控制延迟，WBC 用旧轨迹** |
| WBC 求解 | 2.5 ms | 1-2 ms | 充足 | 关节指令延迟 |
| 通信+其他 | 2.5 ms | 1-2 ms | 充足 | -- |
| **总计** | **50 ms** | **30-45 ms** | -- | -- |

**关键瓶颈**：MPC 求解是最紧张的环节。RTI（Real-Time Iteration）只做 1 次 SQP 迭代，把求解时间控制在 10ms 以内。但加入感知约束后，HPIPM 的 QP 子问题变大（多了 SDF 约束），求解时间可能增加 2-3ms。

### 计算资源分配

ANYmal D 搭载 Intel i7 NUC（4 核 8 线程）+ NVIDIA Jetson Xavier NX（GPU）。资源分配如下：

```
CPU 核心绑定（isolcpus 策略）

Core 0-1: 系统 + ROS 通信
Core 2:   感知线程（Elevation Mapping, 分类, SDF）
Core 3:   MPC 线程（SQP + HPIPM）
Core 4:   MRT 线程（WBC + 状态估计）
Core 5-7: 空闲 / 用户程序

GPU (Xavier NX):
  elevation_mapping_cupy（GPU 加速高程图更新）
  深度图像处理

关键: MPC 线程绑定到专用核心，设置 SCHED_FIFO 实时调度
     优先级: MRT > MPC > 感知 > 其他
```

### 降级模式（Degraded Mode）

感知系统不可能永远正常工作。部署必须有降级策略：

| 故障 | 检测方式 | 降级行为 | 恢复条件 |
|------|---------|---------|---------|
| 深度相机失效 | 无点云 >0.5s | 冻结高程图 + 减速 | 点云恢复 |
| 高程图过期 | 时间戳检查 >1s | 切换到盲 MPC | 高程图更新 |
| SDF 计算超时 | watchdog | 用上一帧 SDF | SDF 更新 |
| MPC 不收敛 | SQP 残差过大 | 用上一帧轨迹 + 减速 | SQP 收敛 |
| 所有感知失效 | 多重检测 | 完全盲 MPC + 减速至停 | 人工干预 |

**降级不是失败——是设计**。ANYmal 的工程实践中，约 5-10% 的运行时间处于某种降级模式。重要的是降级是**平滑的**（不是突然切换），并且能**自动恢复**。

### 地图更新频率与 MPC 频率的协调

一个微妙的问题：高程图以 20 Hz 更新，MPC 以 100 Hz 运行。MPC 的 5 次求解共享同一张高程图。如果机器人在这 50ms 内移动了 2.5cm（@0.5m/s），MPC 在第 5 次求解时看到的地形已经"偏移"了半个格子。

**处理方式**：MPC 在查询高程图时，用当前的**状态估计**做坐标变换——把查询点从当前机器人坐标系变换到高程图的坐标系。状态估计以 400 Hz 更新，所以坐标变换是准确的。

```
MPC 查询高程图的坐标变换

每次 MPC 查询 SDF(x_foot, y_foot):
  1. x_foot, y_foot 是在 MPC 模型坐标系中的位置
  2. 用最新的状态估计 T_world_base 变换到世界坐标系
  3. 用高程图的 T_world_map 变换到高程图坐标系
  4. 在高程图坐标系中做双线性插值

注意: 如果 SLAM 发生回环修正, T_world_map 可能突然跳变
     → 用指数平滑过渡, 避免 MPC 看到的地形突然"跳"
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：MPC 线程没有设置实时优先级**
> **错误做法**：用默认的 `SCHED_OTHER` 调度策略运行 MPC 线程。
> **现象**：当系统负载高时（如 RViz 渲染、rosbag 录制），MPC 线程被抢占，求解时间从 8ms 突然跳到 30ms --> MPC 频率降到 33 Hz --> 控制器响应迟钝，机器人走路不稳。
> **正确做法**：用 `pthread_setschedparam` 设置 `SCHED_FIFO` + 高优先级，或者用 `chrt -f 90 ./mpc_node` 启动。同时用 `isolcpus` 把 MPC 绑定到专用 CPU 核心。

> 🧠 **思维陷阱：认为仿真中调好的参数可以直接用于实机**
> **新手想法**："仿真中 $w_h = 100$ 效果完美，实机也用这个"
> **实际上**：仿真的传感器模型（完美点云、零延迟）与实际传感器（噪声、遮挡、延迟）差异显著。通常需要在实机上重新调参——增大安全裕度、降低代价权重（避免过激反应）、增加地图平滑（降噪）。典型的 sim-to-real 调参工作量是仿真调参的 2-3 倍。

### SDF 梯度计算的实现细节 ⭐⭐⭐

SDF 碰撞约束是 Perceptive MPC 的核心。SDF 值通过双线性插值从离散栅格获得：

$$d(x, y) = (1-s)(1-t) \cdot d_{00} + s(1-t) \cdot d_{10} + (1-s)t \cdot d_{01} + st \cdot d_{11}$$

其中 $s = (x - x_0) / \Delta x$，$t = (y - y_0) / \Delta y$。对 $x$ 和 $y$ 的梯度为：

$$\frac{\partial d}{\partial x} = \frac{1}{\Delta x}\left[(1-t)(d_{10} - d_{00}) + t(d_{11} - d_{01})\right]$$

$$\frac{\partial d}{\partial y} = \frac{1}{\Delta y}\left[(1-s)(d_{01} - d_{00}) + s(d_{11} - d_{10})\right]$$

这些梯度先在距离场接口中作为 $\nabla_p d$ 给出，再与末端运动学雅可比相乘传播到 MPC 的决策变量。OCS2 的 `BilinearInterpolation` 提供插值值和局部梯度，`EndEffectorDistanceConstraint(CppAd)` 负责把距离场梯度与末端位置雅可比组合起来。

### 部署延迟优化技巧 ⭐⭐

| 优化手段 | 延迟节省 | 实现难度 | 说明 |
|---------|---------|---------|------|
| SDF 预计算（离线 EDT） | ~5 ms/次 | 低 | 每次高程图更新后离线计算 SDF，MPC 只查表 |
| SDF 分辨率降低（0.04m → 0.08m） | ~2 ms | 低 | 牺牲精度换速度，对大障碍物够用 |
| 感知线程与 MPC 线程异步 | ~3-5 ms | 中 | MPC 不等待最新地图，用上一帧地图 |
| 约束稀疏化（只检查摆动腿） | ~1-2 ms | 低 | 支撑腿已在地面，无需碰撞检查 |
| 初始猜测热启动 | ~2-3 ms | 中 | 用上一次 MPC 的解平移作为初始猜测 |

### 练习

1. **[计算题]** 一个四足机器人搭载 Intel i7-1260P（4 性能核 + 8 能效核）和 NVIDIA Jetson Orin NX。设计 CPU 核心分配方案，并计算每个线程的最大允许延迟（假设 MPC 目标频率 100 Hz、WBC 目标频率 500 Hz）。

2. **[设计题]** 为 Perceptive MPC 系统设计一个完整的降级状态机（state machine）：定义所有降级状态、状态间的转移条件、每个状态下的行为。用 UML 状态图表示。

3. **[分析题]** 如果 SLAM 系统在 MPC 运行期间触发了回环修正，机器人的全局位姿跳变了 5cm。这对 Perceptive MPC 有什么影响？高程图的坐标系也跳变了 5cm 吗？MPC 应该怎么处理？

---

## 67.10 本章小结

### 知识点总结

| 知识点 | 核心内容 | 难度 | 关联章节 |
|--------|---------|------|---------|
| 盲 MPC 的失效模式 | 落脚不可行、摆动碰撞、高度不匹配 | ⭐ | Ch55 |
| 感知 MPC 系统架构 | 三层六模块管线，频率/延迟预算 | ⭐⭐ | Ch55, Ch60 |
| 可踩性分类 | 坡度+粗糙度+台阶检测，阈值判断 | ⭐⭐ | Ch60 |
| 平面分割 | 连通域标记+协方差拟合，凸不等式约束 | ⭐⭐ | -- |
| 2D SDF 计算 | EDT 距离变换，膨胀+边距+高斯模糊 | ⭐⭐ | Ch60 |
| SDF 碰撞约束 | 距离场局部梯度 + 末端运动学雅可比 | ⭐⭐⭐ | Ch48 |
| 地形感知代价 | 高度跟踪、躯体避障、摆动回避 | ⭐⭐ | -- |
| Swing 地形适应 | 查询路径最大高度，调整抬高 | ⭐⭐ | Ch58 |
| OCS2 perceptive 源码 | Bilinear/TrilinearInterpolation, DistanceTransformInterface, EndEffectorDistanceConstraint | ⭐⭐⭐ | Ch55 |
| RL vs MPC 对比 | Miki 2022 vs Grandia 2023，混合方案 | ⭐⭐⭐ | Ch65 |
| 部署实践 | 延迟预算、核心绑定、降级策略 | ⭐⭐ | -- |

### 本章在课程中的位置

```
向前承接:
  Ch55 (OCS2 完整栈) → 本章在 OCS2 上增加感知模块
  Ch60 (感知落脚规划) → 本章把感知从落脚扩展到全 MPC

向后指向:
  Ch68 (legged_control 精读) → legged_control 中的感知集成
  Ch70 (前沿方向) → 感知-MPC-RL 的融合研究
  Ch73 (mobile_manipulator) → 移动机械臂的感知 MPC
```

---

## 累积项目：本章新增模块

**足式累积项目：从零构建四足感知控制器**

本章新增模块：**Perceptive MPC 管线**

```
累积项目进度:
  Ch55: OCS2 基础 MPC (盲)
  Ch56: 步态管理器
  Ch57: 状态估计
  Ch58: 落脚规划 (Raibert)
  Ch60: 高程图 + 可踩性分析
  ────────────────────────────────
  Ch67: [新增] Perceptive MPC 管线
    - 距离场计算/接口模块 (ComputeDistanceTransform + DistanceTransformInterface)
    - SDF 碰撞约束 (EndEffectorDistanceConstraint)
    - 地形感知代价 (高度跟踪 + 躯体避障)
    - 摆动轨迹地形适应 (初始猜测生成器)
    - 降级逻辑 (感知失效 → 盲 MPC)
  ────────────────────────────────
  Ch68: [下一章] legged_control 整合
```

**本章实操目标**：在 OCS2 + Gazebo 仿真中，部署 Perceptive MPC，让四足机器人走过一段包含 5cm 台阶和 15 度斜坡的地形。对比"开感知"和"关感知"的通过率。

---

## 延伸阅读

### 必读

| 文献 | 类型 | 难度 | 核心贡献 |
|------|------|------|---------|
| Grandia R., Jenelten F., Yang S., Farshidian F., Hutter M. (2023) "Perceptive Locomotion Through Nonlinear Model-Predictive Control" -- T-RO, Vol. 39, pp. 3402-3421 | 论文 | ⭐⭐⭐ | 本章核心：完整的感知 NMPC 管线 |
| Miki T., Lee J., Hwangbo J., et al. (2022) "Learning robust perceptive locomotion for quadrupedal robots in the wild" -- Science Robotics, Vol. 7, eabk2822 | 论文 | ⭐⭐⭐ | RL 感知运动的标杆 |
| OCS2 官方文档: https://leggedrobotics.github.io/ocs2/ | 文档 | ⭐⭐ | OCS2 框架使用指南 |

### 进阶

| 文献 | 类型 | 难度 | 核心贡献 |
|------|------|------|---------|
| Jenelten F., He J., Farshidian F., Hutter M. (2024) "DTC: Deep Tracking Control" -- Science Robotics | 论文 | ⭐⭐⭐ | RL+MPC 混合架构 |
| Hoeller D., et al. (2024) "ANYmal parkour" -- Science Robotics | 论文 | ⭐⭐⭐ | 纯 RL 极限感知运动 |
| Corbères A., et al. (2025) "Perceptive Locomotion through Whole-Body MPC and Optimal Region Selection" -- IEEE Access | 论文 | ⭐⭐⭐ | 全身 MPC + 最优区域选择 |
| Jenelten F., et al. (2022) "TAMOLS: Terrain-Aware Motion Optimization for Legged Systems" -- T-RO | 论文 | ⭐⭐⭐ | 地形感知运动优化，SDF 碰撞回避 |

### 前沿

| 文献 | 类型 | 难度 | 核心贡献 |
|------|------|------|---------|
| Jacquet M., Harms M., Alexis K. (2025) "Neural NMPC through signed distance field encoding for collision avoidance" -- IJRR | 论文 | ⭐⭐⭐⭐ | 神经网络 SDF 编码 + NMPC |
| Agarwal A., et al. (2023) "Legged Locomotion in Challenging Terrains using Egocentric Vision" -- CoRL | 论文 | ⭐⭐⭐ | 第一人称视觉驱动运动 |
| Yang R., et al. (2023) "Neural volumetric memory for visual locomotion control" -- CVPR | 论文 | ⭐⭐⭐⭐ | 神经体积记忆 |
| Felzenszwalb P., Huttenlocher D. (2012) "Distance Transforms of Sampled Functions" -- Theory of Computing, Vol. 8, pp. 415-428 | 论文 | ⭐⭐ | EDT 算法的理论基础 |
