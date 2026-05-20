# S1 MuJoCo 核心引擎与 MJCF 建模

> **难度**：⭐⭐-⭐⭐⭐ | **学时**：2 周（20-28 小时） | **性质**：全方向共享基础

**前置依赖**：
- v8 Ch22 Eigen 矩阵运算——mjData 中所有矩阵/向量可通过 NumPy view 访问，零拷贝与 Eigen::Map 的语义完全对应
- 任一仿真器使用经验（Gazebo / IsaacGym / Flightmare / PyBullet 均可）——需要"对比锚点"来理解 MuJoCo 的设计选择

**位置说明**：MuJoCo 是 05\_运动控制所有方向的横切工具——无论你做四足 RL、人形 MPC、灵巧手操作还是生物力学仿真，本章内容都是你的第一层基础设施。后续 S02（MJPC / mjctrl / mink 交互式控制）、S03（MJX / Warp / Isaac Lab GPU 生态）、S04（可微分仿真理论）和 S05（可微分 MPC）全部建立在本章之上。

---

## S1.0 前置自测 ⭐

在开始本章之前，请确认你能回答以下 5 个问题。如果超过 2 个回答不出来，建议先补前置章节。

1. **Eigen::Map\<VectorXd\> 与 VectorXd 的区别是什么？**（提示：零拷贝 vs 深拷贝——这直接对应 Python 中 `d.qpos` 是 view 还是 copy 的问题）
2. **什么是广义坐标 q 与广义速度 v？为什么 nq 不一定等于 nv？**（提示：球关节用四元数表示位置但只有 3 个角速度分量）
3. **正向动力学（FD）和逆向动力学（ID）在数学上分别求解什么？**（提示：FD 给定力求加速度；ID 给定加速度求力）
4. **URDF 文件描述了机器人的哪些信息？不能描述哪些？**（提示：运动学树和惯量有，接触参数和执行器模型没有）
5. **你用过的仿真器（Gazebo/IsaacGym/PyBullet）中，接触碰撞后的行为是什么样的？**（提示：物体弹开、穿透、还是卡住？）

## S1.0.1 本章目标 ⭐

学完本章后，你应该能够：

1. **解释** MuJoCo 选择"Gauss 原理 + 凸优化 + 软约束"而非"LCP + 硬接触"的根本原因，以及这个选择如何影响 RL 训练收敛性、MPC 求解可行性和 sim-to-real 迁移策略
2. **画出** mjModel/mjData 的字段关系图，并与 Pinocchio Model/Data 做逐字段对照
3. **区分** `mj_step`、`mj_forward`、`mj_inverse` 三大核心函数的物理含义和调用时机
4. **编写** 完整的 MJCF 模型文件，包括运动学树、接触参数（solref/solimp）、执行器模型和传感器
5. **诊断** 常见的 MuJoCo 使用错误——修改 `d.qpos` 后忘记调用 `mj_forward`、solref 参数设置不合理导致穿透过大、混淆 `ctrl` 与 `qfrc_applied`
6. **用 Python** 完成基本仿真循环、传感器读取、力平衡验证（正逆动力学一致性检查）

## S1.0.2 本章知识地图 ⭐

这一章不是 MuJoCo API 速查表，而是建立一个仿真工程师需要长期使用的心智模型：**一个 MJCF 文件如何变成可计算的动力学模型，状态如何在 `mjData` 中流动，接触约束如何变成可求解的优化问题，最后控制器如何把力或目标位置写回仿真器**。

```cpp
MJCF / URDF
    │
    │  编译：解析 XML、展开 default、计算惯量、建立名称索引
    ▼
mjModel（只读）
    │
    │  创建运行时状态
    ▼
mjData（可变）
    │
    ├─ qpos / qvel / ctrl             用户可写状态与控制
    ├─ xpos / xquat / site_xpos       运动学输出
    ├─ contact / efc_*                接触与约束空间量
    ├─ qfrc_bias / qfrc_actuator      力学分解
    └─ qacc / sensordata              加速度与传感器输出
    │
    ▼
mj_forward / mj_step / mj_inverse
    │
    ├─ 查询当前物理量：mj_forward
    ├─ 推进一小步时间：mj_step
    └─ 反解需要的广义力：mj_inverse
```

上面这张数据流图的关键节点可以总结为三层：

| 层 | 核心对象 | 角色 | 读/写 |
|---|---------|------|-------|
| 模型层 | `mjModel` | 编译后的物理描述（拓扑、惯量、执行器、传感器、求解器选项） | 只读 |
| 状态层 | `mjData` | 运行时状态 + 中间计算缓冲（位置、速度、接触、力分解） | 可变 |
| 计算层 | `mj_forward` / `mj_step` / `mj_inverse` | 查询物理量、推进时间、反解广义力 | 函数调用 |

把 MuJoCo 和前面学过的 Pinocchio 对照，会发现两者的共同根系：它们都把**模型拓扑**和**运行时缓存**分开。回顾足式/30_Pinocchio深度精读：Pinocchio 的 `Model` 保存关节树、惯量、父子关系，`Data` 保存一次算法调用产生的中间量。MuJoCo 也是同样思路，只是 `mjModel` 还包含接触、执行器、传感器、求解器选项，`mjData` 还保存接触列表、约束力和传感器读数。下面这张表把 MuJoCo 与 Pinocchio 的核心分层做一个快速对照：

| 层面 | MuJoCo | Pinocchio | 关键差异 |
|------|--------|-----------|----------|
| 模型结构 | `mjModel`（含接触/执行器/传感器/求解器） | `Model`（纯运动学+动力学树） | MuJoCo 是完整仿真描述，Pinocchio 是纯动力学库 |
| 运行时缓存 | `mjData`（含接触列表/约束力/传感器读数） | `Data`（含中间矩阵/坐标变换） | MuJoCo 多了物理交互层的缓存 |
| 计算入口 | `mj_step` / `mj_forward` / `mj_inverse` | `forwardKinematics` / `aba` / `rnea` | MuJoCo 包含积分和碰撞，Pinocchio 只做动力学 |

| 学习层次 | 本章对应内容 | 读完后应该能做什么 |
|----------|--------------|--------------------|
| 物理层 | S1.1、S1.5、S1.8 | 解释软接触、Gauss 原理、摩擦锥、约束求解器的关系 |
| 数据层 | S1.2 | 判断一个量应该在 `mjModel` 还是 `mjData` 中寻找 |
| 时间层 | S1.3、S1.6 | 正确选择 `mj_step`、`mj_forward`、`mj_inverse` |
| 建模层 | S1.4、S1.9 | 写出包含接触、执行器、传感器、闭链约束的 MJCF |
| 调参层 | S1.10、S1.13 | 通过症状定位 timestep、solref、friction、condim、执行器配置问题 |

> **本质洞察**：仿真器不是“把牛顿方程算快一点”的黑盒，而是一个**建模假设的集合**。同一个机器人、同一个控制器，在软接触和硬接触中可能表现不同，不是因为某个引擎“对”或“错”，而是因为它们选择了不同的接触物理和数值求解路径。

本章的阅读顺序建议如下：

1. 先读 S1.1，建立“为什么 MuJoCo 与 PhysX/Bullet 不同”的动机。
2. 再读 S1.2，把 `mjModel`/`mjData` 的字段在脑中定位清楚。
3. 接着读 S1.3，掌握 `mj_step`、`mj_forward`、`mj_inverse` 的调用时机。
4. 读 S1.4 和 S1.9，知道 MJCF 如何表达 URDF 表达不了的物理对象。
5. 读 S1.5、S1.8、S1.10，把接触模型从直觉、公式、调参三个层面闭环。
6. 最后完成 S1.11 的综合练习，并用 S1.13 的故障排查表检查自己的模型。

---

## S1.1 MuJoCo 的物理建模哲学——为什么它与 PhysX/Bullet 根本不同 ⭐⭐

> **这一节解决什么问题**：你可能已经用过 Gazebo（底层 ODE/DART）、PyBullet 或 IsaacGym（底层 PhysX），觉得"仿真器不就是解牛顿方程吗，有什么本质区别？"这一节告诉你：**接触的数学建模**是仿真器之间最深层的分水岭，而这个选择直接决定了你能不能做逆动力学、RL 的 reward landscape 是否光滑、以及 sim-to-real 迁移的难度。

### 动机：如果只用 PhysX/Bullet 会怎样？ ⭐⭐

假设你正在训练一个四足机器人的 RL 策略。机器人的脚反复踩地、离地。每次脚触地的瞬间，力是如何计算的？

在 **PhysX/Bullet** 一类偏硬接触的求解框架中：脚触地的瞬间，求解器倾向于通过冲量或速度修正快速消除穿透。工程上这很适合游戏和实时渲染，因为物体“看起来不能互相穿过”很重要；但对机器人控制来说，接触边界的非光滑性会带来三个直接后果：

- **reward 在接触边界震荡**：脚刚碰到地面时，接触力从 0 跳到很大的值，导致 reward 函数不连续。PPO/SAC 的策略梯度在这些不连续点上方差极大，收敛变慢。
- **`mj_inverse` 不可能实现**：硬接触下，给定 (q, v, a) 反解力 tau 是 ill-posed 的——同一个加速度可以对应无穷多个接触力组合（集合值映射）。这意味着你无法在仿真中检查"为什么机器人倒了"。
- **梯度很难稳定使用**：硬接触是非光滑的，不能简单把仿真轨迹当成处处可导函数。MJX-JAX 等可微路线之所以有用，是因为它们在 MuJoCo 风格的软约束模型和 JAX 计算图中提供了更可检查的局部梯度；但接触切换、裁剪和求解器分支附近仍然要做有限差分验证。

在 **MuJoCo**（软约束/凸优化）中：脚触地时，允许微小穿透（通常 < 1mm），穿透量通过弹性-阻尼和阻抗函数产生**连续过渡的接触响应**。这让很多局部分析、逆动力学和可微分仿真工作更容易展开，但它不是“所有状态处处光滑”的保证。

### 来龙去脉：Todorov 2012 论文的核心动机 ⭐⭐

MuJoCo 的设计哲学源自 Emanuel Todorov、Tom Erez、Yuval Tassa 在 2012 年 IROS 发表的论文 *MuJoCo: A physics engine for model-based control*。这篇论文的重点不是“再写一个更快的刚体引擎”，而是把接触、关节限位、摩擦、等式约束统一放进一个适合模型预测控制和轨迹优化的数学框架中。

这不是一个表面 API 差异，而是一个**数学层面的根本差异**。核心洞察是：真实世界的接触本来就不是无限刚性的。橡胶轮胎、硅胶足垫、人类皮肤、桌面微小粗糙度都有形变。把接触建模为“完全刚性、绝对零穿透”是一种理想化；这种理想化在某些碰撞场景很有用，但会让多接触控制问题变成非光滑、难反解、难求导的问题。

**为什么选择凸优化？** 凸优化有三个关键性质使其成为物理引擎的理想数学框架：
- **全局最优有明确含义**：凸优化不会因为初值不同落到完全不同的局部解，接触力的选择有统一准则。
- **逆动力学可以被定义**：软约束下，给定状态和加速度时，约束力不是任意集合中的一个元素，而是求解器模型定义出的结果。
- **数值诊断更可解释**：约束残差、迭代次数、接触力分解都能映射回物理含义，而不只是“求解器又抖了一下”。

### 完整对比：MuJoCo vs PhysX vs Bullet vs Drake ⭐⭐

| 维度 | MuJoCo | PhysX (IsaacGym) | Bullet (PyBullet) | Drake |
|------|--------|-------------------|--------------------|----- |
| **接触模型** | 软约束 / 凸优化 | 硬约束 / TGS 迭代求解 | 硬约束 / SI (Sequential Impulse) | 硬约束 / 时间步进互补 |
| **约束力定义** | 由软约束优化模型定义 | 迭代近似，依赖求解设置 | 迭代近似，依赖求解设置 | 严格但可能涉及互补条件 |
| **逆动力学(有接触)** | `mj_inverse` well-defined | 不可行 | 不可行 | 部分支持（约束雅可比） |
| **可微仿真** | MJX-JAX 可用于局部梯度，MJX-Warp 更偏高吞吐 | 取决于具体 PhysX / Warp 路线 | 不支持 | 有限支持 |
| **penetration** | 允许微小穿透（可调） | 理论零穿透（实际有漂移） | 理论零穿透（实际有漂移） | 理论零穿透 |
| **大步长稳定性** | 5ms 仍稳定 | 需要 <=2ms 或多子步 | 需要 <=4ms | 取决于约束类型 |
| **GPU 并行** | MJX/Warp（千级并行） | 原生 GPU（万级并行） | 不支持 | 不支持 |
| **物理准确度(柔性)** | 优秀（橡胶/硅胶/皮肤） | 差（需手动调参） | 中等 | 中等 |
| **物理准确度(刚性)** | 中等（需调低 solref） | 优秀 | 优秀 | 优秀 |
| **RL 生态** | dm_control / Gymnasium / Playground | IsaacLab / legged_gym | Stable Baselines | 主要用于 MPC |
| **典型用户** | DeepMind / OpenAI / Berkeley | NVIDIA / ETH / 四足公司 | 教学 / 小型项目 | MIT / TRI (丰田) |

### 对 RL / MPC / Sim-to-Real 的具体影响 ⭐⭐

**对 RL 训练的影响**：MuJoCo 的软接触让 reward landscape 在接触边界处更连续——当机器人的脚从"刚好不碰地"过渡到"刚好碰地"时，接触力从 0 **连续增长**而非理想化跳变。这通常会降低策略梯度估计中的接触噪声，让奖励曲线更容易解释。但反过来，如果真实机器人在硬地面上走（如混凝土），过软的仿真接触可能导致 sim-to-real gap——脚在仿真中"陷入"地面获得额外稳定性，真实世界没有这个余量。

**对 MPC 的影响**：MPC 需要在每个时间步求解最优控制问题，通常用到仿真器的梯度（或至少需要仿真器是确定性的、可预测的）。MuJoCo 的软约束模型让接触力在同一个数学框架下被计算，使得基于 MuJoCo 的 iLQG/DDP 类 MPC（如 MJPC）更容易解释残差和收敛状态。偏硬接触的迭代求解框架也能用于控制仿真，但接触切换处的非光滑性会让优化器更敏感。

**对 Sim-to-Real 的影响**：MuJoCo 的 `solref`/`solimp` 参数提供了精细的接触特性调节能力——你可以把仿真中的地面调成"和真实橡胶地垫一样软"或"和混凝土一样硬"。但这要求你知道真实接触面的物理特性。PhysX 的硬接触模型参数更少，但也意味着更少的 sim-to-real 调参空间。**没有哪个仿真器天然 sim-to-real gap 为零——关键是你能否系统地辨识和补偿 gap。**

### ⚠️ 常见陷阱：MuJoCo 的 penetration 不是 bug 是 feature ⭐⭐

新手经常在 MuJoCo viewer 中看到物体"穿透"地面一点点，然后认为仿真有 bug。实际上这正是 MuJoCo 软接触的设计意图——penetration 深度乘以弹性系数产生接触力，就像真实的弹性接触一样。

**如何判断穿透量是否合理？** 经验法则：穿透量应该 < 几何尺寸的 1%。一个 10cm 的盒子穿透 0.5mm 是正常的；穿透 5mm 说明 `solref` 参数设置得太软了。

💡 **概念误区：把 MuJoCo 的软接触等同于"弹簧-阻尼器"**

新手容易把 MuJoCo 的接触理解为"给碰撞面加了一个弹簧"。实际上 `solref` 定义的不是直接的弹簧常数 $k$，而是约束空间中的**参考加速度的时间尺度**——MuJoCo 内部会根据有效质量 $m_{eff}$、阻抗函数 $d(r)$（由 `solimp` 定义）和求解器正则化项共同计算最终的约束力。如果你直接把 `solref[0]` 反推成弹簧刚度 $k = m_{eff}/\text{timeconst}^2$ 再用到其他引擎，参数几乎不可能迁移。正确做法是把 `solref` 理解为"接触恢复的特征时间"，通过穿透深度和接触力曲线来标定，而非反推物理弹簧常数。

🧠 **思维陷阱：认为"软接触 = 物理不准确"**

新手在看到 MuJoCo 允许穿透后，常常本能地认为"硬接触更真实"。这个判断忽略了真实世界的复杂性：橡胶足垫、硅胶指尖、人类皮肤、甚至混凝土表面的微观粗糙度，**都有有限刚度和有限形变**。真正无限刚性的接触在自然界中不存在。MuJoCo 的软接触模型在许多有限刚度的真实材料场景中，反而比理想化的硬接触模型更接近物理现实。选择接触模型时，关键不是"软 vs 硬哪个更对"，而是"你关心的接触场景在刚度谱上处于什么位置"。

### 正确/错误：solref/solimp 参数设置 ⭐⭐

```xml
<!-- ❌ 错误：solref 太软，导致机器人"陷入"地面 -->
<geom name="foot" type="sphere" size="0.03"
      solref="0.1 1.0"/>   <!-- 时间常数 0.1s = 非常软！ -->

<!-- ✅ 正确：橡胶足垫的合理参数 -->
<geom name="foot" type="sphere" size="0.03"
      solref="0.02 1.0"/>   <!-- 时间常数 0.02s = 适中 -->

<!-- ✅ 正确：硬地面接触（金属足/混凝土地面） -->
<geom name="foot" type="sphere" size="0.03"
      solref="0.005 1.0"/>  <!-- 时间常数 0.005s = 硬 -->
```

`solref[0]`（时间常数）的物理直觉：它是接触弹簧恢复到平衡位置所需的特征时间。值越小，接触越硬，穿透越小，但仿真步长也需要相应减小以保持稳定。经验值域：

| 接触场景 | solref[0] 推荐范围 | 备注 |
|---------|-------------------|------|
| 橡胶足垫 / 硅胶 | 0.01 - 0.03 | 可从 0.02 附近开始试验 |
| 硬塑料 / 木头 | 0.005 - 0.01 | 需要 timestep <= 2ms |
| 金属 / 混凝土 | 0.001 - 0.005 | 需要 timestep <= 1ms 或 implicit 积分器 |
| 海绵 / 软体 | 0.05 - 0.2 | 可用大步长 |

🧠 **深层理解**：`solref` 定义的是约束空间中的参考加速度时间尺度，而不是直接给一个普通弹簧的 $k$ 和 $b$。在 `solref = [timeconst, dampratio]` 的正值格式下，可以先用下面的弹簧-阻尼直觉理解：

$$k_{eff} = \frac{m_{eff}}{\text{timeconst}^2}, \quad b_{eff} = \frac{2 \cdot \text{dampratio} \cdot m_{eff}}{\text{timeconst}}$$

这里的 $k_{eff}$ 与 $b_{eff}$ 只是帮助建立量纲直觉。MuJoCo 实际求解时还会经过 `solimp` 定义的阻抗函数 $d(r)$、约束宽度和求解器正则化，因此不能把上式当成“XML 参数到接触刚度”的完整换算公式。工程上更稳妥的做法是：先把 `timeconst` 理解为接触恢复时间，再用穿透深度、接触力曲线和稳定性来标定，而不是直接反推一个物理弹簧常数。

`dampratio = 1.0` 对应接近临界阻尼的恢复行为，是常用起点。`dampratio < 1.0` 更容易出现弹跳，`dampratio > 1.0` 则更偏过阻尼。

**solimp 参数**控制的是接触约束的"硬度曲线"——当穿透从 0 增大时，约束力如何增长。`solimp = [dmin, dmax, width, midpoint, power]` 五个参数定义了一个 S 型曲线：

| solimp 参数 | 含义 | 典型值 |
|------------|------|-------|
| `dmin` | 最小约束阻抗（穿透很小时约束被激活的比例） | 0.9 |
| `dmax` | 最大约束阻抗（穿透较大时约束被激活的比例） | 0.95 |
| `width` | 从 dmin 过渡到 dmax 的穿透范围 | 0.001 |
| `midpoint` | S 型曲线的中点位置 | 0.5 |
| `power` | S 型曲线的陡度 | 2 |

大多数情况下 solimp 使用默认值即可。只有在需要精细控制接触刚度-穿透关系时（如 sim-to-real 标定）才需要调整。**初学者应该优先调 solref，不要同时调 solimp——两者耦合会让调参空间爆炸。**

### S1.1 练习 ⭐⭐

1. **[⭐⭐ 概念题]** 解释为什么 MuJoCo 的软接触能让 `mj_inverse` 在有接触时也 well-defined，而 Bullet 的硬接触不行。提示：从"给定加速度能否唯一确定力"的角度思考。
2. **[⭐⭐ 实验题]** 在 MuJoCo viewer 中加载一个球体自由落体到平面的场景。分别设置 `solref = [0.001, 1.0]`、`[0.02, 1.0]`、`[0.1, 1.0]`，观察穿透深度和弹跳行为的差异。记录每种设置下的最大穿透量。
3. **[⭐⭐⭐ 思考题]** 许多实时引擎通过 Gauss-Seidel 类迭代求解接触。增加迭代次数可以让约束违反更小，但为什么仍然不能简单等同于“物理更真实”？提示：从接触模型、时间步长、摩擦近似和停止准则四个角度分析。

---

## S1.2 mjModel / mjData 双结构设计——与 Pinocchio 的逐字段映射 ⭐⭐

> **这一节解决什么问题**：你可能习惯了 Pinocchio 的 `Model`/`Data` 分离，或者 IsaacGym 的 "gym.create\_sim -> gym.set\_dof\_state" 范式。MuJoCo 的 `mjModel`/`mjData` 也是读写分离设计，但字段更丰富（包含接触、执行器、传感器）。理解这两个结构体的每个字段，是使用 MuJoCo API 的基础。

### 动机：为什么要把模型和数据分开？ ⭐⭐

一个天真的仿真器设计是把所有东西放在一个结构体里——质量、惯量、当前位置、当前速度、接触力全部混在一起。这在单线程时没问题，但一旦你需要：

- **多线程并行仿真**（用 N 个线程各跑一个 rollout 做蒙特卡洛搜索）——每个线程需要自己的状态，但模型参数（质量、关节限位、几何形状）是共享的
- **零内存分配**（`mj_step` 的整个过程不做任何 malloc/free）——预分配所有缓冲区，仿真过程中只填数据
- **域随机化**（训练时随机改变质量/摩擦/执行器增益）——只需复制一份 mjModel 并修改其中几个字段

——你就需要**读写分离**：一个只读的"模型描述"（mjModel）和一个可变的"仿真状态 + 中间计算缓冲"（mjData）。

这个设计模式在机器人学软件中非常普遍：

| 软件 | 只读结构 | 可变结构 | 分离动机 |
|------|---------|---------|---------|
| **MuJoCo** | mjModel | mjData | 线程安全 + 零 malloc |
| **Pinocchio** | Model | Data | 线程安全 + ABI 稳定 |
| **Drake** | MultibodyPlant | Context | 线程安全 + 自动微分 |
| **IsaacGym** | sim (全局) | env\_state (per-env) | GPU 并行 |

💡 **关键概念：mjModel 是编译产物**——MJCF XML 经过 MuJoCo 的编译器变成 mjModel，这个过程是**单向的**。你不能从 mjModel 反向生成原始 XML（`mj_saveLastXML` 会生成等效 XML，但不是原始 XML 的逐字还原）。编译过程做了大量优化：坐标系对齐、惯量对角化、碰撞体包围盒预计算等。这就像 C++ 源码编译成二进制——你可以反汇编但不能还原原始代码。

编译的具体步骤可以用 Python 观察到：

```python
import mujoco

# 方式 1：从 XML 文件编译
m = mujoco.MjModel.from_xml_path("robot.xml")   # MJCF XML -> mjModel

# 方式 2：从 XML 字符串编译（适合程序化生成模型）
xml_string = '<mujoco><worldbody><body><geom size="0.1"/></body></worldbody></mujoco>'
m = mujoco.MjModel.from_xml_string(xml_string)

# 方式 3：从 URDF 编译（MuJoCo 内置 URDF 解析器）
m = mujoco.MjModel.from_xml_path("robot.urdf")

# 编译后可以导出等效 XML（用于检查编译器做了什么转换）
mujoco.mj_saveLastXML("/tmp/compiled_output.xml", m)
```

⚠️ **注意**：MuJoCo 3.2+ 引入了 `MjSpec` API 用于程序化模型编辑（运行时添加/删除刚体和关节），这在域随机化中比重复编译 XML 高效得多。下面是一个最小示例，让你对 MjSpec 的使用方式有个初步印象：

```python
import mujoco

# MjSpec：程序化构建模型，无需手写 XML
spec = mujoco.MjSpec()
spec.worldbody.add_body("box_body")
spec.worldbody.first_body().add_geom(
    name="box", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.1, 0.1]
)
model = spec.compile()   # 编译为 mjModel
data = mujoco.MjData(model)
```

MjSpec 的详细用法（如运行时动态增减刚体、批量修改接触参数做域随机化）是进阶话题，此处不展开。

### mjModel 关键字段（只读，编译自 MJCF/URDF） ⭐⭐

| mjModel 字段 | 含义 | 类型/形状 | Pinocchio Model 对应 | 备注 |
|-------------|------|----------|---------------------|------|
| `nq` | 广义坐标维度 | int | `model.nq` | 含四元数，所以 nq >= nv |
| `nv` | 广义速度维度 | int | `model.nv` | 最小化坐标维度 |
| `nbody` | 刚体数量 | int | `model.nbodies` | 包含 world body |
| `njnt` | 关节数量 | int | `model.njoints` | MuJoCo 含 free joint |
| `nu` | 执行器数量 | int | —（Pinocchio 无） | MuJoCo 独有 |
| `nsensor` | 传感器数量 | int | —（Pinocchio 无） | MuJoCo 独有 |
| `body_mass[i]` | 第 i 个刚体质量 | float | `model.inertias[i].mass()` | |
| `body_inertia[i]` | 惯量对角元（体坐标系） | float[3] | `model.inertias[i].inertia()` | MuJoCo 存储对角化后的惯量 |
| `jnt_type[i]` | 关节类型 | enum | `model.joints[i].shortname()` | hinge/slide/ball/free |
| `opt.gravity` | 重力向量 | float[3] | `model.gravity` | |
| `opt.timestep` | 积分步长 | float | — | Pinocchio 不做积分 |
| `opt.solver` | 接触求解器类型 | enum | — | PGS/CG/Newton |
| `opt.integrator` | 积分器类型 | enum | — | Euler/implicit/RK4 |

### mjData 关键字段（可变，每步更新） ⭐⭐

| mjData 字段 | 含义 | 形状 | Pinocchio Data 对应 | 何时有效 |
|------------|------|------|---------------------|---------|
| `qpos` | 广义坐标 | (nq,) | `data.q` | 随时可读写 |
| `qvel` | 广义速度 | (nv,) | `data.v` | 随时可读写 |
| `qacc` | 广义加速度 | (nv,) | `data.a` | `mj_forward` 后 |
| `ctrl` | 执行器输入 | (nu,) | — | 由用户设置 |
| `qfrc_applied` | 外加广义力 | (nv,) | `data.tau` (RNEA 输入) | 由用户设置 |
| `xpos` | 刚体世界位置 | (nbody, 3) | `data.oMi[i].translation()` | `mj_forward` 后 |
| `xquat` | 刚体世界四元数 | (nbody, 4) | `data.oMi[i].rotation()` | `mj_forward` 后 |
| `contact` | 活跃接触列表 | (ncon,) | — | `mj_forward` 后 |
| `qfrc_bias` | 重力 + 科氏力 | (nv,) | `data.nle` (RNEA 输出) | `mj_forward` 后 |
| `qfrc_constraint` | 接触约束力 | (nv,) | — | `mj_forward` 后 |
| `qfrc_actuator` | 执行器广义力 | (nv,) | — | `mj_forward` 后 |
| `qfrc_inverse` | 逆动力学力 | (nv,) | `data.tau` (RNEA 输出) | `mj_inverse` 后 |
| `sensordata` | 传感器读数 | (nsensordata,) | — | `mj_forward` 后 |

**MuJoCo 独有、Pinocchio 没有的字段**反映了两者的定位差异——MuJoCo 是"全物理仿真器"，Pinocchio 是"纯动力学库"：

- `ctrl`、`qfrc_actuator`——执行器模型是 MuJoCo 的一等公民
- `contact`、`qfrc_constraint`——接触检测和约束求解是内置的
- `sensordata`——传感器抽象是内置的
- `opt.solver`、`opt.integrator`——求解器和积分器配置在模型层面

### Python 零拷贝：d.qpos 是 view 不是 copy ⭐⭐

MuJoCo 的 Python binding 通过 pybind11 把 mjData 中的 C 数组直接暴露为 NumPy 数组，**不做任何内存拷贝**。这意味着修改 NumPy 数组会直接修改底层 C 结构体的内存。这与 Eigen::Map 的语义完全对应——如果你在 C++ 中用过 `Eigen::Map<VectorXd>(data.qpos, nq)`，Python 的 `d.qpos` 就是同一件事的 Python 封装。

```python
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("humanoid.xml")
d = mujoco.MjData(m)
mujoco.mj_step(m, d)

# d.qpos 返回的是底层 C 数组的 NumPy view
qpos_view = d.qpos          # shape (nq,)，指向 mjData 内部的内存
print(qpos_view.base is not None)  # True——说明这是 view，不是独立数组

# ✅ 正确：直接通过索引修改 view，会修改 mjData 内部的数据
d.qpos[2] = 1.0             # 直接修改了 mjData.qpos[2]

# ✅ 正确：如果你需要保存当前状态的快照，必须显式拷贝
saved_state = d.qpos.copy() # 深拷贝——后续 mj_step 不会改变 saved_state
```

下面是最常见的错误写法：

```python
# ❌ 错误：覆盖 Python 变量名不等于修改 mjData
qpos_ref = d.qpos
qpos_ref = np.zeros(m.nq)   # 这只是让 Python 变量 qpos_ref 指向新数组
                              # mjData 内部的 qpos 完全没变！

# ✅ 正确：使用切片赋值来写入底层 C 数组
d.qpos[:] = np.zeros(m.nq)  # 切片赋值 [:] 把数据写入已有数组的内存
```

### Named Access API ⭐⭐

MuJoCo 3.0+ 提供了按名称访问关节/传感器的 API，不需要手动管理索引：

```python
# ❌ 容易出错：通过硬编码索引访问
knee_pos = d.qpos[7]          # 哪个关节？索引可能因模型修改而变化

# ✅ 推荐：通过名称访问——自描述，不受模型修改影响
knee_pos = d.joint('left_knee').qpos
imu_acc  = d.sensor('imu_acc').data

# 当你确实需要索引时（比如构造大型向量做批量运算）
knee_id  = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'left_knee')
knee_qpos_adr = m.jnt_qposadr[knee_id]  # 该关节在 qpos 数组中的起始地址
```

Named access 的实现原理是：mjModel 内部维护了一个名称到 ID 的哈希表（编译时构建），`d.joint('left_knee')` 先查表得到 joint ID，再计算该关节对应的 qpos/qvel 数组索引。单次查表开销约 O(1)，在控制循环中使用完全可接受。

### ⚠️ 常见陷阱：修改 d.qpos 后必须调用 mj_forward ⭐⭐

这是 MuJoCo 新手最常犯的错误：

```python
# ❌ 错误：修改了 qpos 但没有更新依赖量
d.qpos[2] = 1.5           # 修改了广义坐标
print(d.xpos[3])          # 读取刚体世界位置
# ⚠️ 此时 xpos 还是旧值！因为 xpos 是 mj_forward 的输出

# ✅ 正确：修改后调用 mj_forward 更新所有依赖量
d.qpos[2] = 1.5
mujoco.mj_forward(m, d)   # 重新计算：正运动学 -> 碰撞检测 -> 力学
print(d.xpos[3])          # 现在 xpos 反映了新的 qpos
```

**为什么不自动更新？** 因为 `mj_forward` 的计算开销不可忽略（包括正运动学、碰撞检测、约束求解等）。如果每次写 `d.qpos` 都自动触发 `mj_forward`，在你批量设置多个关节时会重复计算多次。MuJoCo 的设计哲学是：**用户显式控制计算时机**，避免隐式的性能开销。这一点与 Pinocchio 完全一致——Pinocchio 中你也必须显式调用 `pin.forwardKinematics(model, data, q)` 来更新 `data.oMi`。

### 💡 线程安全模式与域随机化 ⭐⭐

mjModel/mjData 分离的最大工程收益是**多线程安全**。正确的并行仿真模式：

```python
import mujoco
from concurrent.futures import ThreadPoolExecutor

m = mujoco.MjModel.from_xml_path("humanoid.xml")

def run_rollout(seed):
    d = mujoco.MjData(m)      # 每个线程创建自己的 mjData
    # m 只读所以线程间共享安全，d 只有本线程访问
    for _ in range(1000):
        d.ctrl[:] = policy(d.qpos, d.qvel, seed)
        mujoco.mj_step(m, d)
    return d.qpos.copy()

with ThreadPoolExecutor(8) as pool:
    results = list(pool.map(run_rollout, range(8)))
```

如果你需要域随机化（每个 rollout 用不同的物理参数），则需要复制 mjModel：

```python
import copy
m_base = mujoco.MjModel.from_xml_path("humanoid.xml")

def run_randomized_rollout(seed):
    m_rand = copy.deepcopy(m_base)           # 深拷贝 mjModel
    rng = np.random.default_rng(seed)
    m_rand.body_mass[3] *= rng.uniform(0.8, 1.2)  # 随机化质量
    d = mujoco.MjData(m_rand)
    # ... 仿真循环 ...
```

### S1.2 练习 ⭐⭐

1. **[⭐⭐ 代码题]** 加载 Menagerie 的 `unitree_go2/scene.xml`。打印 `m.nq`、`m.nv`、`m.nbody`、`m.njnt`、`m.nu`。解释为什么 `nq > nv`（提示：Go2 有一个 free joint，用 7 个广义坐标但只有 6 个广义速度）。
2. **[⭐⭐ 对比题]** 用 Pinocchio 加载同一个 Go2 的 URDF。对比 `pinocchio_model.nq` 和 `mujoco_m.nq` 是否一致。如果不一致，分析原因（提示：Pinocchio 默认不加 free joint，需要显式指定 `JointModelFreeFlyer`）。
3. **[⭐⭐ 陷阱验证题]** 写一个脚本：(a) `mj_step` 之后记录 `d.xpos[3].copy()`；(b) 修改 `d.qpos[0] += 0.5` 但不调用 `mj_forward`；(c) 读 `d.xpos[3]` 并与步骤 a 对比——它们应该相同；(d) 调用 `mj_forward` 后再读 `d.xpos[3]`——此时应该不同。用 `np.allclose` 验证你的预期。
4. **[⭐⭐⭐ 线程安全题]** 用 `ThreadPoolExecutor` 创建 4 个线程，共享同一个 mjModel，每个线程各自创建 mjData 并从相同初始条件跑 100 步（`ctrl = 0`）。验证 4 个线程的最终 `qpos` 是否一致。思考：如果不小心在线程间共享了同一个 mjData 会发生什么？
## S1.3 三大核心函数——mj_step / mj_forward / mj_inverse ⭐⭐

上一节建立了 mjModel/mjData 的心智模型。现在进入 MuJoCo 最核心的三个函数——它们定义了仿真引擎"怎样推动物理世界向前走"，是后续所有控制算法（MPC、RL policy 部署、阻抗控制）的前提。

#### 完整调用链 ⭐⭐

```bash
mj_step(m, d)
  ├── mj_step1(m, d)                ← 阶段 1：积分前的状态刷新与控制回调
  │   ├── mj_checkPos / mj_checkVel   检查状态是否数值异常
  │   ├── mj_fwdPosition(m, d)        正运动学 → xpos/xquat/xipos
  │   │   ├── mj_kinematics(m, d)     从 qpos 计算刚体笛卡尔位姿
  │   │   ├── mj_comPos(m, d)         复合刚体质心
  │   │   └── mj_crb(m, d)            复合刚体惯量矩阵 → d.qM
  │   ├── mj_sensorPos(m, d)          位置相关传感器（关节编码器、IMU 姿态等）
  │   ├── mj_collision(m, d)          宽相位 → 窄相位碰撞检测 → d.contact
  │   ├── mj_fwdVelocity(m, d)        科氏力/离心力/被动力 → d.qfrc_bias
  │   ├── mj_sensorVel(m, d)          速度相关传感器
  │   └── mjcb_control(m, d)          若注册控制回调，在这里写入 d.ctrl
  │
  ├── [没有使用 mjcb_control 时，用户代码可在 mj_step1 后写入 d.ctrl]
  │
  └── mj_step2(m, d)                ← 阶段 2：执行器、加速度、约束与 Euler 积分
      ├── mj_fwdActuation(m, d)       ctrl → 执行器力 → d.qfrc_actuator
      ├── mj_fwdAcceleration(m, d)    M * qacc = sum(forces) → d.qacc
      ├── mj_fwdConstraint(m, d)      **接触约束求解** → d.qfrc_constraint
      ├── mj_sensorAcc(m, d)          加速度相关传感器
      ├── mj_checkAcc(m, d)           检查加速度是否数值异常
      ├── mj_Euler(m, d)              split API 固定使用 Euler 积分
      └── d.time += m.opt.timestep     时间前进
```

上面的调用链可以按"阶段-职责-写入字段"三列快速对照：

| 阶段 | 主要职责 | 写入的关键 `mjData` 字段 |
|------|---------|------------------------|
| `mj_step1` | 正运动学、碰撞检测、速度传感器、控制回调 | `xpos/xquat`, `contact`, `qfrc_bias` |
| 用户写入 `d.ctrl` | 设置执行器输入 | `ctrl` |
| `mj_step2` | 执行器力、加速度求解、约束求解、积分 | `qfrc_actuator`, `qacc`, `qfrc_constraint`, `qpos/qvel/time` |

`mj_step` 被拆成 `step1`/`step2` 两个阶段。step1 完成后，位置、速度、碰撞和速度传感器都已刷新，但执行器力、加速度、约束力和积分还没有发生——这是设置 `d.ctrl` 的最佳时机。官方 callback `mjcb_control` 也在这个位置被调用。需要注意：`mj_step1`/`mj_step2` 这个 split API 只对应 Euler 积分路径；如果模型选择 RK4，不应把两段 API 当成完整 RK4 的拆分。

下表总结三大核心函数的职责和典型调用场景：

| 函数 | 输入 | 输出 | 是否推进时间 | 典型用途 |
|------|------|------|:----------:|----------|
| `mj_step` | `(qpos, qvel, ctrl)` | 更新 `(qpos, qvel, qacc, contact, sensordata, ...)` | 是 | 仿真主循环 |
| `mj_forward` | `(qpos, qvel, ctrl)` | 更新 `(qacc, xpos, contact, sensordata, ...)` | 否 | 查询物理量、修改 qpos 后刷新 |
| `mj_inverse` | `(qpos, qvel, qacc)` | `qfrc_inverse` | 否 | 力学诊断、逆动力学验证 |

**`mj_forward`**：执行与 `mj_step` 相同的全部计算，但**跳过积分**——不更新 `qpos`/`qvel`/`time`。用途：给定 `(qpos, qvel, ctrl)` 计算当前加速度 `qacc` 和所有中间量。

**`mj_inverse`**：给定 `(qpos, qvel, qacc)` 反解所需广义力 `qfrc_inverse`。数学上等价于 RNEA，但独特之处在于：**即使存在接触也能给出唯一解**。

#### mj_inverse 的独特价值——PhysX/Bullet 做不到 ⭐⭐⭐

**根本原因**：PhysX/Bullet 的硬接触（LCP/NCP）使接触力成为约束优化的对偶变量——同一状态可能对应多组接触力（集合值映射），无法唯一反解。MuJoCo 的软接触使接触力是 `(qpos, qvel)` 的**光滑、唯一函数**，因此逆动力学有且仅有一个解。

这意味着你可以随时"拆解"机器人运动的力学来源：

```python
import mujoco
import numpy as np

# 加载模型并设置一个非平凡状态
m = mujoco.MjModel.from_xml_path("unitree_go2/scene.xml")
d = mujoco.MjData(m)

# 让机器人站在地面上——先做几步让接触建立
d.qpos[2] = 0.35  # 躯干高度
for _ in range(200):
    mujoco.mj_step(m, d)

# ---- 正动力学 ----
# mj_forward 计算当前 qacc（不推进时间）
mujoco.mj_forward(m, d)
qacc_from_forward = d.qacc.copy()

# ---- 逆动力学 ----
# 给定同样的 (qpos, qvel, qacc)，反解需要的力
mujoco.mj_inverse(m, d)

# 力的分解——这是 PhysX/Bullet 无法提供的
print("=== 力分解（广义力空间，单位 N·m 或 N）===")
print(f"逆动力学总力  qfrc_inverse:    {d.qfrc_inverse[:6]}")
print(f"重力+科氏力   qfrc_bias:       {d.qfrc_bias[:6]}")
print(f"接触约束力    qfrc_constraint: {d.qfrc_constraint[:6]}")
print(f"执行器力      qfrc_actuator:   {d.qfrc_actuator[:6]}")

# 验证正逆动力学一致性：
# mj_inverse 给出的 qfrc_inverse 是“为了产生当前 qacc 所需的非约束广义力”。
# 在 forward dynamics 中，它应与 actuator/applied/passive 三类非约束力相匹配。
non_constraint_force = (d.qfrc_actuator + d.qfrc_applied + d.qfrc_passive)
residual = d.qfrc_inverse - non_constraint_force
print(f"\n正逆动力学残差 (应接近零): max |residual| = {np.max(np.abs(residual)):.2e}")
```

这段代码的输出揭示了一个深刻的物理图景：对于站立的四足机器人，`qfrc_bias` 主要是重力和速度相关偏置，`qfrc_constraint` 是地面接触约束力；它们共同决定当前 `qacc`，但 `qfrc_inverse` 的一致性检查应放在“非约束力”这一侧，而不是把 bias 和 constraint 再加到一起。如果你在 RL 训练中发现机器人"抖动"，可以分别记录 `qfrc_actuator`、`qfrc_constraint` 和 residual：是执行器力在震荡，还是接触求解在跳变，会一眼看出来。

这里最容易混淆的是“力的解释”和“残差的验证”。

解释站立现象时，可以说重力、接触力和执行器力共同形成平衡。

验证正逆动力学一致性时，必须站在 MuJoCo 的方程两侧看问题。

`qfrc_bias` 是被移到动力学方程左侧的偏置项。

`qfrc_constraint` 是约束求解器给出的约束侧贡献。

`qfrc_inverse` 则表示为了产生当前加速度，系统需要的非约束广义力。

因此它应该和 actuator、applied、passive 这些非约束力比较。

如果把 bias 和 constraint 也加进去，表面上是在做“力分解”，实际是在把方程两边的量混到同一侧。

这类错误在调试里很危险。

残差不为零时，你会误以为 MuJoCo 的逆动力学错了。

真正应该检查的是：模型中是否有被动力、外力、执行器传动或接触状态没有被纳入比较。

对腿足机器人尤其要注意 free joint。

浮动基座前 6 个广义速度维度没有直接电机。

如果只看后 12 个关节力矩，可能会错过基座方向的接触力平衡。

更稳健的排查方式是先比较完整 `nv` 维 residual，再按关节、基座和接触状态拆开解释。

这也是 MuJoCo 作为调试工具的价值：它不只告诉你“机器人倒了”，还允许你把倒下前一刻的动力学账本逐项展开。

**实际应用场景**：
- **控制器验证**：你设计了一个 MPC 控制器，它输出关节力矩。用 `mj_inverse` 可以验证："在当前接触条件下，达到目标加速度到底需要多大力？我的控制器输出是否匹配？"
- **力矩估计**：真实机器人通常没有力矩传感器。在仿真中用 `mj_inverse` 生成"真值"力矩数据，训练一个力矩观测器，然后部署到实机
- **系统辨识**：改变模型参数（质量、摩擦），观察 `mj_inverse` 输出如何变化，做灵敏度分析

#### 积分器选择 ⭐⭐

| 积分器 | 稳定性 | 精度 | 速度 | 推荐场景 |
|--------|--------|------|------|---------|
| `mjINT_EULER` | 低 | O(h) | 最快 | 向后兼容旧代码 |
| `mjINT_IMPLICIT` | 高 | O(h) | 中 | 刚性系统（大惯量差异） |
| `mjINT_IMPLICITFAST` | **高** | O(h) | **快** | 接触丰富、又希望控制计算成本的常见选择 |
| `mjINT_RK4` | 中 | O(h^4) | 慢 | 无接触场景需要高精度轨迹 |

`implicitfast` 的思想是在保持隐式速度积分稳定性的同时减少部分雅可比计算成本。是否采用它取决于模型刚度、接触频率、控制频率和可接受的能量误差；具体可用积分器、枚举名和默认设置应以官方文档为准。

#### 从欧拉积分到隐式积分：为什么接触系统需要更稳的时间推进 ⭐⭐

数值积分器解决的是一个非常朴素的问题：已经知道当前状态 $(q_k, v_k)$ 和加速度 $\dot{v}_k$，怎样得到下一步 $(q_{k+1}, v_{k+1})$？最直观的做法是显式欧拉：

$$
v_{k+1} = v_k + h \dot{v}(q_k, v_k)
$$

$$
q_{k+1} = q_k + h v_k
$$

这里 $h$ 是仿真步长。显式欧拉像是“看一眼当前速度和加速度，然后闭着眼走一步”。如果系统很平滑，这一步通常没问题；如果系统里有很硬的弹簧、很强的阻尼、很突然的接触，闭着眼走一步就可能跨过稳定区域。

接触系统的难点在于：接触力本身取决于下一步的位置和速度。脚越陷入地面，接触力越大；接触力越大，下一步速度变化越大；速度变化越大，又会影响下一步穿透深度。这形成了一个强反馈环。

```text
穿透深度 d
  │
  ▼
接触恢复力 f_n = k d + b d_dot
  │
  ▼
广义加速度 qacc
  │
  ▼
下一步速度/位置
  │
  └── 反过来改变下一步穿透深度
```

如果用显式积分，这个反馈环在一步之后才被“发现”；如果步长过大，系统可能已经把物体弹飞。隐式积分的核心思想是：**把下一步的速度也放进方程里一起求**，让求解器提前感知这个反馈。

半隐式欧拉可以写成：

$$
v_{k+1} = v_k + h \dot{v}(q_k, v_{k+1})
$$

$$
q_{k+1} = q_k + h v_{k+1}
$$

这看起来只改了一个下标，但意义很大：速度更新不再完全依赖旧速度，而是要求新速度与新接触力自洽。工程直觉上，这就像你不是先猛踩一脚再看车有没有打滑，而是一边踩一边根据轮胎反馈调整油门。

| 情况 | 显式积分容易出现的现象 | 隐式积分的改善 |
|------|------------------------|----------------|
| 硬接触 | 物体弹飞、接触力尖峰 | 把阻尼反馈纳入当前步 |
| 高增益 PD | 关节振荡、能量注入 | 减少数值能量增长 |
| 质量差异大 | 小质量部件抖动 | 对刚性模态更稳定 |
| 步长过大 | 穿透后再猛烈修正 | 提前求自洽速度 |

反事实地看，如果所有系统都只用 RK4 会怎样？RK4 对平滑 ODE 的精度很高，但每一步需要多次动力学评估，并且接触切换点不是光滑 ODE。对于自由飞行的机械臂，RK4 可能给出更漂亮的轨迹；对于四足机器人连续踩地，花更多计算量去高阶逼近平滑区间，不一定比一个稳定的一阶隐式积分器更划算。

> **本质洞察**：接触仿真中的积分器选择，优先目标不是“局部截断误差阶数最高”，而是“在控制器、接触刚度和步长共同作用下不向系统注入虚假能量”。高阶不等于稳，稳也不等于真实；真实来自模型参数、步长和求解器共同匹配。

#### 积分器选择的工程流程 ⭐⭐

| 观察到的现象 | 先检查 | 再考虑 |
|--------------|--------|--------|
| 关节在无接触时发散 | PD 增益、阻尼、关节限位 | 积分器是否过于显式 |
| 落地瞬间弹飞 | `solref[0]` 太小、步长太大 | 换隐式积分或减小 timestep |
| 自由飞行轨迹误差大 | 步长是否过大 | RK4 或更小步长 |
| 高频抖动但平均姿态正常 | 执行器刚度与接触刚度耦合 | 降低 `kp` 或增大阻尼 |
| 训练速度太慢 | 单步仿真耗时、渲染是否关闭 | 更便宜的积分器或更大批量 |

> **⚠️ 易错陷阱：混淆 `mj_step` 和 `mj_forward`**
>
> `mj_step` 推进时间（更新 qpos/qvel/time），`mj_forward` 不推进时间。初学者常见的两个错误：
>
> 1. **在仿真循环中用 `mj_forward` 代替 `mj_step`**——机器人永远不动，因为状态从未更新。
> 2. **在需要计算中间量时用 `mj_step`**——状态被意外推进了一步，时间线被打乱。
>
> 经验法则：**推进仿真用 `mj_step`，查询物理量用 `mj_forward`**。

---

## S1.4 MJCF 建模语言——超越 URDF 的表达力 ⭐⭐

URDF（Unified Robot Description Format）是 ROS 生态的标准机器人描述格式。如果你从 SLAM 或 ROS 背景转来，URDF 是你已有的知识。MJCF（MuJoCo XML Format）则是 MuJoCo 的原生模型格式。两者的区别不是"语法不同"这么简单——它们的**设计哲学根本不同**。

#### MJCF vs URDF 对比 ⭐⭐

| 维度 | URDF | MJCF |
|------|------|------|
| **设计目标** | 描述运动学树（joint-link 结构） | **描述完整的可仿真物理场景** |
| 接触参数 | 无法表达 | `solref`/`solimp`/`friction`/`condim` |
| 执行器模型 | 无法表达 | `motor`/`position`/`velocity`/`cylinder`/`muscle` |
| 传感器 | 无法表达 | 30+ 种（IMU、力传感器、测距等） |
| 肌腱/线缆 | 无法表达 | 固定肌腱/空间肌腱 |
| 闭环运动链 | 无法表达（只能树结构） | `<equality>` 约束实现闭链 |
| 一个 body 多关节 | 需要插入 dummy link | 直接支持（如球关节 = 3 hinge） |
| 求解器配置 | 无 | `<option solver=... iterations=.../>` |
| 默认值继承 | 无 | `<default>` 层级机制 |
| include/组合 | 无标准方案 | `<include file="..."/>` |
| 场景定义 | 需外部工具（Gazebo SDF） | 地面/光源/相机全在 MJCF 内 |

结论：**URDF 描述的是"机器人长什么样"，MJCF 描述的是"机器人在物理世界中怎么运动"**。这正是为什么 Menagerie 的 MJCF 模型可以"开箱即用"地做仿真，而 URDF 模型需要大量额外配置。

#### 接触参数 solref / solimp——sim-to-real 的关键旋钮 ⭐⭐⭐

MuJoCo 的软接触模型用两组参数控制接触力的生成方式。理解它们的物理含义是调出真实接触行为的关键。

**solref = [timeconst, dampratio]**——控制接触的"软硬程度"

接触被建模为一个弹簧-阻尼系统。当两个几何体发生穿透时，MuJoCo 产生一个将它们推开的力。`solref` 控制这个弹簧的特性：

- `timeconst`（时间常数，单位秒）：弹簧振荡的特征时间。值越大，接触越"软"，穿透恢复越慢。值越小，接触越"硬"，响应越快。
  - 橡胶足垫：`0.02`（软，允许形变）
  - 金属碰撞：`0.002`（硬，快速弹开）
  - 经验规则：设为仿真步长的 5-10 倍
- `dampratio`（阻尼比）：`1.0` = 临界阻尼（无振荡最快收敛），`< 1.0` 会弹跳，`> 1.0` 过阻尼（粘滞感）

**solimp = [dmin, dmax, width, midpoint, power]**——控制约束的"阻抗曲线"

这组参数定义了约束力如何随穿透深度变化的非线性映射：

- `dmin`（最小阻抗，范围 [0,1]）：穿透极浅时的约束刚度比例。通常设 `0.9`
- `dmax`（最大阻抗，范围 [0,1]）：穿透较深时的约束刚度比例。通常设 `0.95`
- `width`（过渡宽度，单位米）：从 dmin 过渡到 dmax 的穿透深度范围。通常设 `0.001`
- `midpoint`/`power`：控制过渡曲线的形状。默认 `0.5` 和 `2` 即可

大多数场景只需要调 `solref` 的两个参数，`solimp` 保持默认值就足够了。

```xml
<!-- 示例：不同材质的接触参数 -->
<!-- 软接触，临界阻尼 -->
<geom name="rubber_foot" type="sphere" size="0.02"
      solref="0.02 1.0"
      solimp="0.9 0.95 0.001 0.5 2"
      friction="0.8 0.005 0.0001" condim="4"/>

<!-- 硬接触 -->
<geom name="metal_tool" type="box" size="0.05 0.02 0.01"
      solref="0.002 1.0"
      friction="0.4 0.001 0.00005" condim="3"/>
```

#### 执行器模型——从 URDF 的空白到精确的驱动建模 ⭐⭐

URDF 完全没有执行器的概念——它只描述关节，不描述驱动关节的"电机"。你在 ROS 中用的 `ros2_control` 里手写的 PID 控制器，在 MuJoCo 中可以直接用 MJCF 声明。

MuJoCo 提供四种基本执行器类型：

| 类型 | MJCF 标签 | ctrl 的物理含义 | 等效控制律 |
|------|----------|---------------|-----------|
| `general` | `<general>` | 用户定义 | 最灵活，可配置任意传递函数 |
| `motor` | `<motor>` | 力/力矩（N 或 N·m） | `tau = gear * ctrl` |
| `position` | `<position>` | 目标位置（rad 或 m） | `tau = kp * (ctrl - q) - kv * qdot` |
| `velocity` | `<velocity>` | 目标速度（rad/s 或 m/s） | `tau = kv * (ctrl - qdot)` |

**从 SLAM/ROS 过来的工程师注意**：如果你在 IsaacGym 或 legged_gym 中看到 PD 控制器写在 Python 代码中（如 `torque = kp * (target - q) - kd * dq`），那等价于 MuJoCo 的 `<position>` 执行器——但 MuJoCo 的做法更优雅，因为 PD 参数（kp/kv）声明在模型文件中，而非散落在训练代码里。

```xml
<!-- 位置伺服——等效于 PD 控制器 -->
<actuator>
  <position name="joint1_servo" joint="joint1"
            kp="100" kv="10"
            ctrlrange="-3.14 3.14"
            forcerange="-87 87"/>   <!-- 力矩限制：电机物理极限 -->
</actuator>

<!-- 力矩执行器——直接施加扭矩 -->
<actuator>
  <motor name="joint1_motor" joint="joint1"
         gear="1" ctrlrange="-87 87"/>
</actuator>
```

#### defaults 机制——层级参数继承 ⭐⭐

当你有 12 个关节（如四足机器人）需要相同的执行器参数时，逐一书写会冗长且易出错。MJCF 的 `<default>` 机制解决了这个问题：

```xml
<mujoco>
  <!-- 层级默认值 -->
  <default>
    <joint damping="0.5" armature="0.01"/>
    <geom friction="0.7 0.005 0.0001" condim="3"
          solref="0.02 1.0"/>

    <default class="leg_motor">
      <position kp="100" kv="10" ctrlrange="-3.14 3.14"/>
      <joint range="-2.0 2.0"/>
    </default>

    <default class="hip_motor">
      <position kp="150" kv="15" ctrlrange="-1.57 1.57"/>
      <joint range="-1.0 1.0"/>
    </default>
  </default>

  <worldbody>
    <body name="trunk" pos="0 0 0.4">
      <joint type="free"/>
      <geom type="box" size="0.2 0.1 0.05" mass="10"/>

      <body name="thigh_FR" pos="0.2 -0.1 0">
        <!-- class="hip_motor" 继承该 default 下的所有参数 -->
        <joint name="hip_FR" type="hinge" class="hip_motor" axis="1 0 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.25" size="0.02" mass="1"/>

        <body name="shank_FR" pos="0 0 -0.25">
          <joint name="knee_FR" type="hinge" class="leg_motor" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.25" size="0.015" mass="0.5"/>
        </body>
      </body>
      <!-- ... 其余三条腿类似 ... -->
    </body>
  </worldbody>

  <actuator>
    <position name="hip_FR"  joint="hip_FR"  class="hip_motor"/>
    <position name="knee_FR" joint="knee_FR" class="leg_motor"/>
    <!-- ... 其余关节类似 ... -->
  </actuator>
</mujoco>
```

子级 `<default>` 继承父级的参数，并可覆盖任何字段。这让大型模型（人形 29-DOF）的 MJCF 文件保持简洁可维护。

#### 结构片段：最小 7-DOF 机械臂 MJCF 的骨架 ⭐⭐

```xml
<mujoco model="simple_7dof_arm">
  <option timestep="0.002" integrator="implicitfast" solver="Newton"/>
  <compiler angle="radian" autolimits="true"/>

  <default>
    <joint armature="0.1" damping="1.0"/>
    <geom condim="3" solref="0.01 1.0" friction="0.6 0.005 0.0001"/>
    <position kp="200" kv="20" forcerange="-87 87"/>
  </default>

  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>
    <body name="base" pos="0 0 0.5">
      <geom type="cylinder" size="0.05 0.05" mass="5"/>
      <body name="link1" pos="0 0 0.05">
        <joint name="j1" type="hinge" axis="0 0 1" range="-2.9 2.9"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.04" mass="3"/>
        <!-- link2..link6 结构相同：hinge joint + capsule geom，交替 axis="0 1 0" / "0 0 1" -->
        <!-- 完整版见练习 1 -->
        <body name="link7_stub" pos="0 0 1.25">
          <joint name="j7" type="hinge" axis="0 0 1" range="-2.9 2.9"/>
          <geom type="sphere" size="0.03" mass="0.3" rgba="1 0 0 1"/>
          <site name="ee_site" pos="0 0 0.03"/>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- 注意：以下执行器引用了 j2-j6 关节，它们定义在上方省略的 link2..link6 中。
       此处仅展示骨架片段，完整模型需补全 link2-link6 及对应的 j2-j6 关节定义。 -->
  <actuator>  <!-- 7 个位置伺服，全部继承 default -->
    <position name="a1" joint="j1"/>
    <position name="a2" joint="j2"/> <!-- ... a3-a6 省略 ... -->
    <position name="a7" joint="j7"/>
  </actuator>
  <sensor>
    <jointpos name="jpos1" joint="j1"/>
    <force name="ee_force" site="ee_site"/>
  </sensor>
</mujoco>
```

注意：这段代码强调 MJCF 的结构组织方式，不是可直接加载的完整模型。完整模型必须补齐 `link2-link6` 与 `j2-j6`，否则执行器引用会失败。这个片段的三个要点是：(1) `<default>` 让同类执行器只需声明 name/joint；(2) `<option>` 明确指定积分器和求解器，避免不同环境中的隐式差异；(3) `<sensor>` 在 URDF 中完全无法表达。

> **⚠️ 易错陷阱：URDF 不能表达闭环链、肌腱和肌肉**
>
> URDF 的拓扑结构被限定为**严格的树**（每个 link 只能有一个 parent joint）。以下结构在 URDF 中无法表达：
>
> - **闭环四连杆机构**（如 ANYmal 的膝关节、许多工业机器人的平行连杆）
> - **肌腱驱动**（如灵巧手的线缆传动：一根线缆连接多个关节）
> - **肌肉模型**（如人体仿真中的 Hill-type muscle）
>
> MJCF 通过 `<tendon>` 和 `<equality>` 标签原生支持这些结构。如果你的机器人有闭链机构（绝大多数工业腿足机器人都有），URDF 只能近似处理（用虚拟弹簧或忽略闭链），MJCF 才能精确建模。

---

## S1.5 接触模型数学 ⭐⭐⭐

理解 MuJoCo 的接触数学不需要读完 Todorov 2012 的全部推导，但以下三个核心概念必须清晰。

#### 软接触：穿透 → 弹性-阻尼力 ⭐⭐⭐

当 geom A 和 geom B 发生穿透（penetration depth = d > 0），MuJoCo 不是像硬接触那样"瞬间弹开"，而是产生一个与穿透深度和穿透速度成正比的法向力：

```text
f_n = k * d + b * d_dot
```

其中弹性系数 k 和阻尼系数 b 由 `solref = [timeconst, dampratio]` 和碰撞点处的等效质量 `m_eff` 决定。MuJoCo 的 solref 定义参考加速度 `aref = -penetration/timeconst^2 - 2*dampratio*velocity/timeconst`，对应的等效弹簧-阻尼参数**与等效质量成正比**：

```text
k_eff = m_eff / timeconst^2
b_eff = 2 * dampratio * m_eff / timeconst
```

`m_eff` 由 MuJoCo 自动从惯量矩阵计算。这个公式的物理直觉：**接触等效于一个弹簧-阻尼器，其固有频率由 timeconst 决定，阻尼特性由 dampratio 决定。刚度与质量成正比意味着：重物体的接触更硬，轻物体的接触更软——这保证了不同质量物体具有相同的恢复时间常数**。

**为什么要允许穿透？** 因为零穿透的硬接触通常需要互补条件（complementarity）来表达“要么分离、要么接触”的逻辑，接触模式切换会使问题非光滑。允许微小穿透（通常希望控制在任务几何精度可接受的范围内）换来了统一的软约束优化模型——这是 MuJoCo 整个架构的基石。

#### 摩擦锥建模：椭圆锥、多面体近似与 Coulomb ⭐⭐⭐

Coulomb 摩擦定律说：切向摩擦力 f_t 的大小不超过 mu * f_n（法向力乘摩擦系数）。在三维空间中，这定义了一个**圆锥**（friction cone）。

不同引擎对摩擦锥的处理方式反映了各自的设计取舍：

| 方法 | 引擎 | 优点 | 缺点 |
|------|------|------|------|
| **多面体近似** | Bullet, ODE | LCP 可直接求解 | 各向异性伪影、棱角效应 |
| **多面体锥或椭圆锥** | **MuJoCo** | 可通过 `<option cone="pyramidal/elliptic">` 选择 | 需要理解 solver 维度和摩擦近似 |
| 无穷面体 | PyBullet 高精度模式 | 接近真实 Coulomb | 计算昂贵 |

MuJoCo 并不固定使用某一种摩擦锥。`cone="elliptic"` 时，可以把切向摩擦理解成椭圆锥：

```text
f_t1^2 / mu1^2 + f_t2^2 / mu2^2 <= f_n^2
```

`cone="pyramidal"` 时，圆锥会被多面体方向近似，求解器看到的是更多线性标量约束。教学上可以把 `elliptic` 理解成更接近连续 Coulomb 锥、把 `pyramidal` 理解成更像传统 LCP/QP 接触方向近似；工程上应以模型中的 `<option cone="...">` 为准，不要只根据 `condim` 推断实际求解维度。

#### condim 的含义 ⭐⭐⭐

MJCF 中的 `condim` 控制接触模型包含哪些物理自由度。它先定义**接触空间维度**；若使用 pyramidal cone，solver 内部的标量约束数还会因为多面体摩擦方向而增加。

| condim | 接触空间含义 | elliptic 下的维度直觉 | pyramidal 下的标量约束直觉 | 典型用途 |
|--------|--------------|------------------------|------------------------------|----------|
| 1 | 仅法向 | 1 | 1 | 冰面、润滑轴承 |
| 3 | 法向 + 2 切向 | 3 | 4 左右 | **大多数场景** |
| 4 | 法向 + 2 切向 + 扭转 | 4 | 6 左右 | 足端防自旋 |
| 6 | 法向 + 2 切向 + 扭转 + 2 滚动 | 6 | 10 左右 | 球形轮、滚珠 |

对于腿足机器人，`condim="3"` 或 `condim="4"` 是合理选择。`condim="4"` 增加的扭转摩擦可以防止足端在地面上自由旋转（spin），这在全身控制中对稳定性有帮助。

---

## S1.6 Python/C API 实战 ⭐⭐

理论清楚后，把完整的仿真循环写一遍。以下代码覆盖了 MuJoCo Python API 的核心使用模式。

#### 完整仿真循环 ⭐⭐

```python
import mujoco
import mujoco.viewer
import numpy as np

# 1. 加载模型，创建数据
m = mujoco.MjModel.from_xml_path("robot.xml")
d = mujoco.MjData(m)

# 检查模型基本信息
print(f"自由度: nq={m.nq}, nv={m.nv}")
print(f"执行器数: nu={m.nu}")
print(f"步长: {m.opt.timestep} s")
print(f"积分器: {m.opt.integrator}")

# 2. 被动查看器（passive viewer）——非阻塞，适合自定义控制循环
with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        # ---- 你的控制器 ----
        d.ctrl[:] = my_policy(d.qpos, d.qvel)

        # ---- 仿真步进 ----
        mujoco.mj_step(m, d)

        # ---- 渲染同步 ----
        viewer.sync()
```

被动查看器 `launch_passive` 是自定义控制循环的标准方式：你在 Python 主循环中控制仿真节奏，viewer 只负责渲染。与之对比，`launch` 是阻塞式的——MuJoCo 接管主循环，适合纯粹的模型查看而非控制开发。

#### 传感器读取与执行器控制 ⭐⭐

```python
# ---- 传感器读取 ----
# 在 MJCF 中定义传感器后，两种访问方式：

# 按名称（推荐——代码可读性高）
imu_acc = d.sensor('imu_acc').data.copy()
foot_force = d.sensor('foot_FR_force').data.copy()

# 按全局数组索引（高频循环中避免字符串查找开销）
all_sensors = d.sensordata  # shape (nsensordata,)

# ---- 执行器控制 ----
# d.ctrl 的物理含义取决于 MJCF 中声明的执行器类型：
#   motor 执行器:    d.ctrl[i] = 期望力矩 (N·m)
#   position 执行器: d.ctrl[i] = 期望关节角度 (rad)
#   velocity 执行器: d.ctrl[i] = 期望关节速度 (rad/s)

# 按名称设置（推荐）
d.actuator('hip_FR').ctrl = 0.5   # 设置 hip_FR 到 0.5 rad

# 按索引设置（高频循环）
d.ctrl[0] = 0.5
```

> **⚠️ 易错陷阱：手动设置 `qpos` 后忘记调用 `mj_forward`**
>
> 这是排名第一的 MuJoCo 新手错误。当你手动修改 `d.qpos`（例如重置机器人位姿）后，所有依赖位置的量（`xpos`、`xquat`、碰撞信息、传感器读数）都是**过期数据**。必须调用 `mj_forward(m, d)` 重新计算：
>
> ```python
> # 错误写法
> d.qpos[:] = new_pose
> print(d.xpos[1])  # 这是旧值！不对应 new_pose
>
> # 正确写法
> d.qpos[:] = new_pose
> mujoco.mj_forward(m, d)  # 重新计算所有位置相关量
> print(d.xpos[1])  # 现在是正确的
> ```
>
> 同理，如果你在 RL 的 `reset()` 函数中设置初始状态，**必须在 reset 末尾调用 `mj_forward`**，否则第一步的 observation 就是错的。这个 bug 极其隐蔽——程序不会报错，只是 observation 滞后一步。

> **⚠️ 易错陷阱：混用旧版 Python 绑定**
>
> 早期教程和开源项目中经常能看到 `import mujoco_py`。新项目应优先查看官方 Python 绑定文档，并以官方文档为准。迁移旧项目时，不要把 `mujoco-py` 的 API 习惯直接套到官方 `mujoco` 包上，尤其是 viewer、模型加载和数组访问部分。

#### C API 最小闭环：加载、控制、步进、释放 ⭐⭐⭐

Python API 适合实验和教学，但真实控制栈经常需要 C++。MuJoCo 的底层接口是 C API，C++ 工程通常直接包含 `mujoco/mujoco.h`，再用 RAII 包一层资源管理。理解 C API 有两个好处：

1. 你能看懂 Python 绑定背后到底调用了什么。
2. 你能在 ROS2 控制节点、实时控制器或 C++ MPC 中直接嵌入仿真。
3. 你能清楚地区分“模型内存”和“状态内存”的生命周期。

最小仿真循环如下：

```cpp
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>

// 用自定义 deleter 管理 mjModel，避免异常路径泄漏内存。
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

// 用自定义 deleter 管理 mjData。mjData 必须和创建它的 mjModel 配套使用。
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

int main() {
  char error[1024] = {0};

  // 1. 从 XML 编译模型。编译失败时 error 会给出具体 XML 行号和原因。
  ModelPtr model(
      mj_loadXML("robot.xml", nullptr, error, sizeof(error)),
      mj_deleteModel);
  if (!model) {
    std::fprintf(stderr, "模型加载失败: %s\n", error);
    return 1;
  }

  // 2. 为这个模型创建一份运行时数据。多线程 rollout 时，每条线程都需要自己的 mjData。
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  if (!data) {
    std::fprintf(stderr, "mjData 创建失败\n");
    return 1;
  }

  // 3. 初始化状态。直接写 qpos/qvel 后，要调用 mj_forward 更新派生量。
  std::fill(data->qpos, data->qpos + model->nq, 0.0);
  std::fill(data->qvel, data->qvel + model->nv, 0.0);
  if (model->nq >= 3) {
    data->qpos[2] = 0.5;  // 示例：把浮动基座高度设到 0.5 m。
  }
  mj_forward(model.get(), data.get());

  // 4. 固定步数仿真。真实工程中这里会接收控制器输出。
  for (int k = 0; k < 1000; ++k) {
    // 先清零控制输入，避免上一周期的控制残留。
    std::fill(data->ctrl, data->ctrl + model->nu, 0.0);

    // 示例：如果有执行器，把第一个执行器给一个小控制量。
    // 这个值的物理含义取决于 MJCF 中 actuator 的类型。
    if (model->nu > 0) {
      data->ctrl[0] = 0.1;
    }

    // 推进一个仿真步。这个调用会更新 qpos/qvel/time，也会刷新接触和传感器。
    mj_step(model.get(), data.get());

    // 每 100 步打印一次状态，避免高频 printf 干扰仿真速度。
    if (k % 100 == 0) {
      std::printf("t = %.3f, qpos[0] = %.6f\n", data->time, data->qpos[0]);
    }
  }

  // 5. unique_ptr 会自动释放 mjData 和 mjModel。
  return 0;
}
```

这段 C++ 代码和 Python 示例一一对应：

| Python | C/C++ | 含义 |
|--------|-------|------|
| `mujoco.MjModel.from_xml_path(path)` | `mj_loadXML(path, ...)` | 编译 XML 为 `mjModel` |
| `mujoco.MjData(m)` | `mj_makeData(m)` | 创建运行时状态 |
| `mujoco.mj_forward(m, d)` | `mj_forward(m, d)` | 刷新派生物理量 |
| `mujoco.mj_step(m, d)` | `mj_step(m, d)` | 推进仿真时间 |
| Python 垃圾回收 | `mj_deleteData` / `mj_deleteModel` | 释放 C 堆内存 |

C++ 中最常见的错误是把 `mjModel*` 和 `mjData*` 的生命周期混在一起：先释放 `mjModel`，再访问 `mjData`；或者复制裸指针到多个对象中，最后重复释放。上面的 `unique_ptr` 不是为了“更现代”，而是为了把所有权表达清楚。

> **本质洞察**：C API 的裸指针并不意味着 MuJoCo 鼓励混乱的资源管理。相反，`mjModel`/`mjData` 的生命周期边界非常清晰：模型先创建、数据依附模型创建；仿真结束时先释放数据，再释放模型。C++ 的 RAII 只是把这个边界写进类型系统。

---

## S1.7 阶段回顾：从模型到仿真循环 ⭐⭐

本章覆盖了 MuJoCo 核心引擎的六个知识块：

| 节 | 核心内容 | 关键收获 |
|----|---------|---------|
| S1.1 | 物理建模哲学 | 软约束 + 优化模型 → 可逆、可诊断、适合控制 |
| S1.2 | mjModel/mjData | 与 Pinocchio Model/Data 同构，但多了接触/执行器/传感器 |
| S1.3 | 三大核心函数 | `mj_step` 推进仿真，`mj_forward` 查询，`mj_inverse` 反解力 |
| S1.4 | MJCF 建模语言 | 超越 URDF——接触参数、执行器、闭链、默认值继承 |
| S1.5 | 接触模型数学 | 弹性-阻尼软接触、摩擦锥选项、solref/solimp 物理含义 |
| S1.6 | Python/C API 实战 | 仿真循环、传感器读取、执行器控制、C++ 资源管理 |

### S1.7 练习 ⭐⭐

**[练习 1 -- URDF 到 MJCF 转换]** ⭐⭐

取 Franka Panda 的 URDF（`franka_description` ROS 包），用以下流程转换为完整的 MJCF：
1. `m = mujoco.MjModel.from_xml_path("panda.urdf")` 加载 URDF
2. `mujoco.mj_saveLastXML("/tmp/panda.xml", m)` 导出 MJCF
3. 打开 `/tmp/panda.xml`，手动添加 7 个 `<position>` 执行器（每个关节一个，kp=200, kv=20）
4. 添加末端力传感器 `<force name="ee_force" site="ee_site"/>`
5. 在 viewer 中验证：拖拽机器人，观察是否有合理的阻力（来自位置伺服）

目的：体会 URDF 到 MJCF 的信息增量——URDF 导出后缺少的执行器和传感器，需要你根据物理知识手动补全。

**[练习 2 -- 完整仿真循环]** ⭐⭐⭐

用 Menagerie 的 `unitree_go2/scene.xml`，实现一个完整的仿真循环：
1. 加载模型并创建 data
2. 在循环中：设置 `d.ctrl` 为零向量（所有关节回零位），调用 `mj_step`，用 `launch_passive` 渲染
3. 记录 10 秒内机器人的质心高度变化（`d.subtree_com[0][2]`），用 matplotlib 画出曲线
4. 观察：机器人从默认位姿落到地面并最终稳定——这条曲线反映了接触动力学

**[练习 3 -- 正逆动力学对比]** ⭐⭐⭐⭐

这是验证你真正理解 MuJoCo 力学体系的练习。在 Go2 站立状态下：
1. 调用 `mj_forward`，记录 `d.qacc`
2. 调用 `mj_inverse`，记录 `d.qfrc_inverse`
3. 打印力分解：`qfrc_bias`、`qfrc_constraint`、`qfrc_actuator`、`qfrc_applied`、`qfrc_passive`
4. 验证 `qfrc_inverse == qfrc_actuator + qfrc_applied + qfrc_passive`（残差接近数值误差）
5. 思考：为什么 `qfrc_bias` 和 `qfrc_constraint` 不应该被加到这个残差右侧？提示：它们已经进入了 MuJoCo 计算当前 `qacc` 的动力学平衡。

**[练习 4 -- 接触参数调优]** ⭐⭐⭐⭐

修改 Go2 的足端 `solref` 参数，做三组对比实验：
1. 极硬接触：`solref="0.001 1.0"`——观察落地时的弹跳和高频振荡
2. 基准接触：`solref="0.02 1.0"`——观察正常的着地阻尼行为
3. 极软接触：`solref="0.1 1.0"`——观察足端明显穿入地面和缓慢回弹

记录每组实验中机器人落地后的质心高度曲线，用 matplotlib 叠加在同一张图上。讨论：对于 sim-to-real 转移，哪组参数更接近真实橡胶足垫的行为？为什么 `solref` 的选择对 RL 策略从仿真迁移到实机有重大影响？

---

## S1.8 从 Gauss 原理到约束求解器 ⭐⭐⭐

S1.1-S1.7 从工程现象和 API 使用层面建立了”MuJoCo 偏好软约束和凸优化”的认知。现在把这个判断推到数学层面：接触力不是凭经验”调出来”的，而是由 Gauss 最小约束原理、惯量矩阵、约束 Jacobian 和摩擦锥共同决定的。

### Gauss 最小约束原理（Gauss's Principle of Least Constraint, 1829） ⭐⭐⭐

经典力学中，Gauss 原理的表述是：**在所有满足约束的加速度中，系统的真实加速度是使"约束偏差"最小的那个**。数学形式如下：

设系统的无约束加速度为 $a_{\text{free}}$（即仅受主动力和惯量决定的加速度），质量矩阵为 $M$（正定对称），则真实加速度 $a^*$ 满足：

$$
a^* = \arg\min_{a \in \mathcal{C}} \frac{1}{2} (a - a_{\text{free}})^T M (a - a_{\text{free}})
$$

其中 $\mathcal{C}$ 是约束允许的加速度集合。

**物理含义的精确解读**：Gauss 原理说的是——"约束做的事情，就是把加速度从无约束轨迹'拉'到最近的允许位置，这里的'最近'用惯量加权的范数来度量"。惯量加权意味着：质量大的自由度更难被约束改变，质量小的自由度更容易被拉偏。这与物理直觉完全一致——你推一堵墙（大惯量），墙几乎不动；你推一个乒乓球（小惯量），球飞走了。

### 从 Gauss 原理到 MuJoCo 的凸优化 ⭐⭐⭐

Todorov 2012 的核心洞察是：把 Gauss 原理直接作为仿真器的**求解器思想**，而非仅作为分析力学的理论工具。为了避免把 MuJoCo 的源码实现误写成一个过度简化的二次规划，可以先用下面的概念式理解：

$$
\min_{a, f} \quad \frac{1}{2}(a-a_{\text{free}})^T M (a-a_{\text{free}}) + \text{constraint regularization}
$$

$$
\text{s.t.} \quad a \text{ 与 } J a \text{ 满足软化后的接触、摩擦和等式约束}
$$

$$
f \in \mathcal{K}
$$

其中各项含义为：

- $M \in \mathbb{R}^{nv \times nv}$：广义质量矩阵，MuJoCo 中通过 composite rigid body algorithm 计算
- $a_{\text{free}}$：不考虑约束时，由执行器力、外力、被动力和偏置项共同决定的自由加速度
- $J$：接触、关节限位和等式约束的 Jacobian
- $f$：约束空间中的力或阻抗变量，受摩擦锥和软约束正则影响

真实实现会在约束空间中引入参考加速度、阻抗函数、正则矩阵和摩擦锥近似；不同 solver 与 `cone` 设置会改变内部线性化和约束数量。因此上式的作用是解释“约束把自由加速度拉回可接受区域”这一核心直觉，而不是作为 MuJoCo 的逐行实现公式。

**约束空间的直觉形式**：若把接触 Jacobian 记为 $J$，约束空间惯量的核心块通常会出现：

$$
A = J M^{-1}J^T
$$

它描述“单位约束力会造成多大的约束空间加速度”。再加上 `solref/solimp` 给出的参考加速度、阻抗和正则项，求解器就在这个约束空间里寻找一组合理的约束响应。若 `cone="pyramidal"`，摩擦锥会被线性方向近似；若 `cone="elliptic"`，摩擦锥更接近连续二阶锥。两者都是凸集合，但内部约束数量和数值行为不同。

### 为什么这是凸的——三个条件的逐一验证 ⭐⭐⭐

1. **目标函数是凸的**：二次项的 Hessian 矩阵 $H = JM^{-1}J^T$ 是半正定的，因为 $M^{-1}$ 正定，对任意向量 $z$，$z^T J M^{-1} J^T z = (J^T z)^T M^{-1} (J^T z) \geq 0$。目标函数是凸二次函数。

2. **约束集合是凸的**：`elliptic` cone 可以写成二阶锥形式，`pyramidal` cone 可以写成线性不等式集合。前者更旋转对称，后者更像多面体方向近似；两者都保持凸性，但 solver 看到的变量和约束结构不同。

3. **可行域非空**：$f = 0$ 总是可行的（无接触力），因此优化问题总有解。

**凸性的工程后果**：优化目标和摩擦锥结构让求解器有明确的下降方向，也让残差、迭代次数和接触力分解具有可解释性。若约束空间惯量退化，还需要正则化和阻抗参数保证数值上可解。MuJoCo 提供多类求解器：

| 求解器 | 算法直觉 | 适用场景 | 主要观察指标 |
|--------|----------|----------|--------------|
| `mjSOL_PGS` | 逐约束投影迭代，像逐个拧紧螺丝 | 小模型、教学验证、保守兼容 | 迭代次数、接触残差 |
| `mjSOL_CG` | 在约束空间做共轭梯度搜索 | 较大稀疏系统、需要较低内存 | 梯度范数、残差下降 |
| `mjSOL_NEWTON` | 用局部二阶信息快速逼近最优 | 接触多且需要高精度的控制仿真 | 线搜索次数、残差、耗时 |

求解器名称、默认值和可用选项会随 MuJoCo 演进而变化，工程中应以官方文档为准。本章关心的是取舍逻辑：PGS 直观且便宜，CG 在较大系统中更节省，Newton 每次迭代更贵但常能更快降低残差。**注意：从 MuJoCo 3.0 开始，默认求解器已从 PGS 切换为 Newton**（`mjSOL_NEWTON`）。如果你在阅读旧教程或代码时看到"MuJoCo 默认用 PGS"的说法，那是 2.x 时代的信息，3.0+ 已不再适用。

### 与 LCP 方案的本质差异 ⭐⭐⭐

传统的硬接触仿真器（ODE/Bullet/PhysX）将接触建模为**线性互补问题（LCP）**：

$$
0 \leq f_n \perp (J_n a + \dot{J}_n v + \text{restitution}) \geq 0
$$

这个 $\perp$ 符号（互补条件）的含义是：**法向力 $f_n$ 和法向间隙加速度不能同时为正**——要么无接触力（分离），要么无间隙加速度（保持接触）。

LCP 的问题在于：

- **多接触时可能无解或多解**——Painleve 悖论提示我们，摩擦接触中的理想刚体模型可能无法给出单一自洽的加速度
- **解关于参数不连续**——微小的几何扰动可能导致接触力跳变，这使得梯度无法穿过 LCP 求解过程

MuJoCo 的凸优化方案**不是 LCP 的近似——它是一种不同的物理建模**。两者在以下情况给出不同结果：

- **刚性碰撞时**：LCP 产生瞬时速度跳变；MuJoCo 产生有限的大加速度（由 solref 控制回弹速度）
- **多接触超定时**：LCP 可能无法分配一致的接触力；MuJoCo 通过最小化 Gauss 目标函数找到"最优折衷"
- **摩擦不确定区域**：LCP 在静-动摩擦切换边界有不连续性；MuJoCo 的软约束和可选摩擦锥近似让切换更容易被数值求解器处理

> **⚠️ 易错陷阱**：一些材料会把 MuJoCo 的软接触简单说成“对硬接触的近似”。这个说法容易误导初学者。更准确的理解是：MuJoCo 选择了不同的接触建模路线。Gauss 原理、软约束和阻抗参数共同定义了一个适合控制与优化的物理模型；在有限刚度的真实材料中，这条路线并不一定比理想刚体模型“更不真实”。

---

## S1.9 MJCF 高级建模：闭链、腱、通用执行器与传感器 ⭐⭐⭐

MJCF 的价值不仅在于能写地面、相机和执行器，还在于能表达 URDF 很难自然表达的结构：闭链机构、腱耦合、带内部状态的执行器和丰富传感器。下面四类机制在腿足、灵巧手和生物力学仿真中非常常见。

### Tendon（腱）——耦合关节驱动 ⭐⭐⭐

MuJoCo 的 `<tendon>` 元素允许定义跨越多个关节的**腱**，实现关节间的运动学耦合。这对于建模**欠驱动手爪**（如 Robotiq 2F-85，一个电机驱动多个指节）至关重要。

腱在物理上是一根（可能非线性的）绳索，连接多个关节。腱的长度是各关节位置的加权和：

$$
L_{\text{tendon}} = \sum_i w_i \cdot q_i
$$

其中 $w_i$ 是关节 $i$ 在该腱中的传动系数。对腱施加力 $F_{\text{tendon}}$，各关节感受到的广义力为 $\tau_i = w_i \cdot F_{\text{tendon}}$。

```xml
<!-- 欠驱动手爪：一根腱同时驱动两个指节 -->
<tendon>
  <fixed name="finger_coupling">
    <joint joint="finger_proximal" coef="1.0"/>
    <joint joint="finger_distal"   coef="0.7"/>
    <!-- distal 以 0.7 的传动比跟随 proximal -->
  </fixed>
</tendon>

<actuator>
  <!-- 执行器作用在腱上，而非单个关节 -->
  <position name="finger_motor" tendon="finger_coupling"
            kp="50" ctrlrange="0 1.2"/>
</actuator>
```

另一种腱类型是**空间腱**（`<spatial>`），它通过固定在刚体上的路径点定义一根空间曲线。空间腱适合建模肌肉骨骼系统（肌腱绕过骨骼的路径随关节角度变化）。

### Equality Constraints——闭链运动学 ⭐⭐⭐

URDF 只能表达开链（树形拓扑）。现实中大量机构是闭链的：四连杆、平行四边形、Stewart 平台、Delta 机器人。MuJoCo 的 `<equality>` 元素支持闭链建模。

两种最常用的等式约束：

- `<connect>`：约束两个刚体上的锚点重合——用于铰接闭链
- `<weld>`：约束两个刚体上的坐标系完全重合（位置+姿态）——用于刚性固连

```xml
<!-- 四连杆（4-bar linkage）——URDF 无法表达 -->
<worldbody>
  <!-- 左侧摇杆：从地面铰接点出发 -->
  <body name="crank" pos="0 0 0.5">
    <joint name="j_crank" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.01"/>
    <!-- 连杆：从摇杆远端铰接 -->
    <body name="coupler" pos="0.3 0 0">
      <joint name="j_coupler" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.25 0 0.15" size="0.01"/>
      <site name="coupler_tip" pos="0.25 0 0.15"/>
    </body>
  </body>

  <!-- 右侧摇杆：从地面另一个铰接点出发 -->
  <body name="rocker" pos="0.4 0 0.5">
    <joint name="j_rocker" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 -0.15 0 0.15" size="0.01"/>
    <site name="rocker_tip" pos="-0.15 0 0.15"/>
  </body>
</worldbody>

<equality>
  <!-- 约束 coupler_tip 和 rocker_tip 重合——闭合四连杆 -->
  <connect body1="coupler" body2="rocker"
           anchor="0.25 0 0.15"
           solref="0.005 1" solimp="0.99 0.99 0.001"/>
  <!-- solref[0]=0.005 → 高刚度，约束几乎无 violation -->
</equality>
```

> **⚠️ 注意**：`<connect>` 约束的 solref/solimp 参数控制约束的"硬度"。对于运动学闭链，应使用高刚度（solref[0] 很小，solimp 接近 1）。如果默认参数导致可见的约束违反（连接点分离），先调小 solref[0]。

### Actuator 高级模型——`<general>` 通用执行器 ⭐⭐⭐

MuJoCo 的 `<motor>` / `<position>` / `<velocity>` 是预定义的快捷方式。真正灵活的是 `<general>` 元素，它支持任意的执行器动力学：

$$
\dot{w} = f_{\text{dyn}}(w, \text{ctrl}) \quad \text{（激活状态动力学）}
$$

$$
\tau = f_{\text{gain}}(w, q, v) \cdot \text{ctrl} + f_{\text{bias}}(w, q, v) \quad \text{（力生成）}
$$

其中 $w$ 是执行器的内部激活状态，$\text{ctrl}$ 是控制输入。通过 `dyntype`、`gaintype`、`biastype` 三个参数的组合，可以表达：

| 执行器类型 | dyntype | gaintype | biastype | 效果 |
|-----------|---------|----------|----------|------|
| 理想力矩电机 | none | fixed | none | $\tau = \text{gain} \cdot \text{ctrl}$ |
| PD 位置伺服 | none | fixed | affine | $\tau = k_p(\text{ctrl} - q) - k_v \dot{q}$ |
| 一阶电机动力学 | integrator | fixed | none | $\dot{w} = \text{ctrl}$, $\tau = \text{gain} \cdot w$ |
| SEA（弹性执行器） | integrator | affine | affine | 弹簧-阻尼耦合 |
| 肌肉模型 | muscle | muscle | muscle | Hill-type 肌肉力学 |

```xml
<!-- 带一阶延迟的电机（模拟电机响应延迟） -->
<actuator>
  <general name="motor_with_delay" joint="knee"
           dyntype="filter" dynprm="0.01"
           gaintype="fixed" gainprm="100"
           biastype="none"
           ctrlrange="-1 1" forcerange="-50 50"/>
  <!-- dynprm=0.01 表示 10ms 的一阶低通滤波时间常数 -->
  <!-- 控制信号经过滤波后才作用于关节 -->
</actuator>
```

### Sensor 全家族——30+ 种传感器 ⭐⭐⭐

MuJoCo 内置的传感器远比大多数用户意识到的丰富。以下按类别列出常用传感器：

**运动学/动力学传感器**：

| 传感器 | XML 标签 | 返回维度 | 说明 |
|--------|---------|---------|------|
| 关节位置 | `<jointpos>` | 1 | 单关节广义坐标 |
| 关节速度 | `<jointvel>` | 1 | 单关节广义速度 |
| 关节力矩 | `<jointactuatorfrc>` | 1 | 作用在关节上的执行器力 |
| 执行器力 | `<actuatorfrc>` | 1 | 执行器输出力 |
| 腱长度 | `<tendonpos>` | 1 | 腱的当前长度 |
| 腱速度 | `<tendonvel>` | 1 | 腱的伸缩速度 |

**惯性测量单元（IMU）族**：

| 传感器 | XML 标签 | 返回维度 | 说明 |
|--------|---------|---------|------|
| 加速度计 | `<accelerometer>` | 3 | 体坐标系线加速度（含重力） |
| 陀螺仪 | `<gyro>` | 3 | 体坐标系角速度 |
| 磁力计 | `<magnetometer>` | 3 | 体坐标系磁场方向 |
| 姿态四元数 | `<framequat>` | 4 | 刚体世界坐标系四元数 |

**接触/力传感器**：

| 传感器 | XML 标签 | 返回维度 | 说明 |
|--------|---------|---------|------|
| 触摸传感器 | `<touch>` | 1 | 绑定 site 的法向接触力标量 |
| 力传感器 | `<force>` | 3 | site 处的力向量 |
| 力矩传感器 | `<torque>` | 3 | site 处的力矩向量 |

```xml
<!-- 完整的足端 IMU + 力传感器配置 -->
<sensor>
  <!-- 躯干 IMU -->
  <accelerometer name="imu_acc" site="torso_imu"/>
  <gyro          name="imu_gyro" site="torso_imu"/>

  <!-- 四足每个脚的触摸传感器 -->
  <touch name="fl_foot_contact" site="fl_foot"/>
  <touch name="fr_foot_contact" site="fr_foot"/>
  <touch name="rl_foot_contact" site="rl_foot"/>
  <touch name="rr_foot_contact" site="rr_foot"/>

  <!-- 关节编码器 -->
  <jointpos name="knee_pos" joint="fl_knee"/>
  <jointvel name="knee_vel" joint="fl_knee"/>

  <!-- 执行器电流反馈 -->
  <actuatorfrc name="knee_torque" actuator="fl_knee_motor"/>
</sensor>
```

在 Python 中读取传感器数据：

```python
mujoco.mj_step(m, d)
imu_acc = d.sensor('imu_acc').data      # shape (3,)
fl_contact = d.sensor('fl_foot_contact').data[0]  # 标量
knee_q = d.sensor('knee_pos').data[0]   # 标量
```

**MuJoCo 3.x Flex/Soft Body 功能**：MuJoCo 3.0 起引入了 `<flex>` 元素，支持有限元风格的软体仿真（如可形变的硅胶手指、软体抓手、织物等）。Flex 使用四面体或壳体网格来描述连续介质的形变，与刚体接触可以在同一个仿真步中统一求解。如果你的项目涉及软体操作（deformable manipulation），这一特性值得关注，但本章不做展开——它的建模和调参方法与刚体接触有显著差异，建议阅读 MuJoCo 官方文档的 Flex 章节。

---

## S1.10 接触模型调参实战：从参数含义到稳定抓取 ⭐⭐⭐

知道 `solref`、`solimp`、`friction`、`condim` 的定义还不够。工程中真正困难的是：看到“物体滑落”“足端抖动”“夹爪弹飞”时，能判断应该改哪个参数，而不是随机试一组数字。

### 调参流程：四步法 ⭐⭐⭐

**第一步：从温和参数开始运行**。可以先从 `solref=[0.02, 1]`、`solimp=[0.9, 0.95, 0.001, 0.5, 2]` 这类常见起点开始，再根据任务精度和材料特性调整；具体默认值以官方文档为准。先运行仿真，观察是否有明显的穿透（penetration）或弹跳（bouncing）。

**第二步：观察 penetration 深度**。在 viewer 中开启 contact force 可视化（快捷键 C），或在代码中检查 `d.contact[i].dist`（负值表示穿透深度）。如果穿透过大（>1mm 对于精密任务），需要增加接触刚度。

**第三步：调 solref[0]（时间常数）**。`solref[0]` 是接触弹簧的特征时间，与接触刚度的关系为 $k \approx 1 / (\text{solref}[0]^2)$。减小 solref[0] 会使接触更硬、穿透更小，但也更容易引起高频振荡。

**第四步：调 solimp[2]（过渡宽度）**。`solimp[2]`（width 参数）控制约束从不活跃到完全活跃的过渡区间。更小的 width 使约束更"开关式"（接近硬接触），更大的 width 使约束更"渐进式"（更光滑但更软）。

### 典型场景参数推荐表 ⭐⭐⭐

| 场景 | solref | solimp | friction | condim | 说明 |
|------|--------|--------|----------|--------|------|
| 刚性地面行走 | [0.02, 1.0] | [0.9, 0.95, 0.001] | [0.8, 0.005, 0.0001] | 3 | 可作为起始点 |
| 橡胶足+平地 | [0.04, 0.8] | [0.8, 0.9, 0.005] | [1.0, 0.005, 0.0001] | 3 | 更软、更高摩擦 |
| 精密装配 | [0.005, 1.0] | [0.95, 0.99, 0.0005] | [0.3, 0.001, 0.0001] | 4 | 高刚度、低摩擦、含扭转 |
| 软体抓取 | [0.05, 0.7] | [0.7, 0.85, 0.01] | [1.2, 0.01, 0.001] | 6 | 软接触、高摩擦、全维度 |
| 金属碰撞 | [0.002, 1.0] | [0.98, 0.99, 0.0002] | [0.3, 0.001, 0.0001] | 3 | 极硬、低能量耗散 |

### condim 选择指南 ⭐⭐⭐

`condim`（contact dimensionality）决定了每个接触点施加力的自由度数目：

| condim | 接触空间 | 物理含义 | 计算成本直觉 | 适用场景 |
|--------|---------|---------|--------------|---------|
| 1 | 法向 | 无摩擦——物体只被推开不被阻止滑动 | 最低 | 流体/低摩擦表面 |
| 3 | 法向 + 2 切向 | 标准滑动摩擦 | 常规 | **绝大多数场景的默认选择** |
| 4 | 法向 + 2 切向 + 扭转 | 加上绕法向的扭转摩擦 | 更高 | 精密装配、圆柱体滚动 |
| 6 | 法向 + 2 切向 + 扭转 + 2 滚动 | 完整接触力学 | 最高 | 球形物体、精确滚动 |

这里的成本只是直觉排序。实际标量约束数量还取决于 `<option cone="pyramidal">` 还是 `<option cone="elliptic">`：pyramidal 会把摩擦锥展开成多个方向约束，`condim=3/4/6` 的 solver 维度通常高于接触空间维度本身。

> **⚠️ 易错陷阱**：给所有接触设 `condim=6` 是常见的"保险式"做法，但代价是接触求解的计算量明显增加。实际上，对于四足行走等场景，`condim=3` 往往已经足够——扭转和滚动摩擦对足端运动的影响可以忽略。**只在确实需要扭转摩擦（如瓶盖拧紧、圆柱体操作）时才用 `condim>=4`。**

### 调参实战：从"物体滑落"到稳定抓取 ⭐⭐⭐

以下是一个典型的抓取任务调参过程，展示如何从"物体总是从手指间滑落"诊断到正确参数：

```python
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("gripper_scene.xml")
d = mujoco.MjData(m)

# 诊断工具：打印所有活跃接触的信息
def diagnose_contacts(m, d):
    for i in range(d.ncon):
        c = d.contact[i]
        geom1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
        geom2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
        print(f"接触 {i}: {geom1} <-> {geom2}")
        print(f"  穿透深度: {-c.dist:.4f} m")  # dist<0 表示穿透
        print(f"  condim: {c.dim}")
        print(f"  摩擦系数: {c.friction[:3]}")

# 运行若干步后诊断
for _ in range(500):
    d.ctrl[:] = 0.8  # 夹紧
    mujoco.mj_step(m, d)
diagnose_contacts(m, d)

# 常见问题诊断清单：
# 1. 法向力很大但物体仍滑落 -> friction 太低，增大 friction[0]
# 2. 穿透深度 > 2mm -> solref[0] 太大（接触太软），减小到 0.01
# 3. 接触力剧烈振荡 -> solref[0] 太小（接触太硬），增大到 0.02
# 4. 物体被"弹飞" -> solimp 的 width 太小，增大 solimp[2]
```

---

## S1.11 综合练习：把建模、接触和诊断串起来 ⭐⭐

- **[A 型 / 练习 5：四连杆 MJCF 建模]** 实现上文给出的四连杆（4-bar linkage）MJCF 模型。(1) 用 `<connect>` 等式约束闭合运动链；(2) 给 crank 关节添加 `<motor>` 执行器，施加正弦力矩 `ctrl = 5*sin(2*pi*t)`；(3) 在 viewer 中验证闭链约束被正确维持（连接点无可见分离）；(4) 打印 `d.eq_active` 检查约束状态。**量化验证**：将 solref[0] 分别设为 0.005、0.02、0.1，测量连接点的最大分离距离（读取两个 site 的 `d.site_xpos` 差的范数），绘制 solref[0] vs 最大分离距离的曲线

- **[A 型 / 练习 6：condim 对抓取的影响]** 使用 Menagerie 的 Robotiq 2F-85 夹爪模型抓取一个圆柱体（`<geom type="cylinder" size="0.02 0.05"/>`）。分别测试 condim=1/3/4/6 四种设置：(1) 每种设置运行 1000 步，记录物体的 z 坐标是否下降（判断滑落）；(2) 用 `time.perf_counter()` 记录每步的仿真时间，对比四种 condim 的计算成本；(3) 绘制条形图：x 轴为 condim 值，左 y 轴为 1000 步后物体 z 坐标下降量（mm），右 y 轴为平均单步仿真时间（us）。**预期结果**：condim=1 因无摩擦必定滑落；condim=3 在圆柱体上可能因缺少扭转摩擦而缓慢旋转滑落；condim=4 稳定抓住；condim=6 同样稳定但计算更慢

- **[B 型 / 练习 7：约束求解器源码精读]** 精读 MuJoCo 源码中 `src/engine/engine_forward.c` 的 `mj_fwdConstraint` 函数（前 200 行）。画出约束求解的流程图，标注以下关键步骤：(1) 约束 Jacobian $J$ 的组装位置；(2) 约束空间惯量矩阵 $A = J M^{-1} J^T$ 的计算位置；(3) 参考加速度 $a_{\text{ref}}$（由 solref 决定）的计算位置；(4) QP 求解器的调用入口——找到 PGS/CG/Newton 三个分支的 `switch` 语句。**进阶**：在代码中找到 condim=3 和 condim=6 时约束矩阵维度的变化——具体是哪个数组的 size 发生了变化？

- **[思考题 / 练习 8]** 如果两个接触点的 Jacobian 行线性相关（即 $J$ 不满秩），$H = JM^{-1}J^T$ 就变成半正定（而非正定），此时接触力是否还唯一？MuJoCo 如何通过阻抗、正则化或求解器设置处理这种退化情况？提示：查阅官方文档中与约束正则化、阻抗和求解器残差相关的参数。追问：正则化是否改变了物理含义？它等价于什么物理模型？

## S1.12 易错陷阱总表 ⭐⭐

前面的各小节已经分散提示了很多坑。这里把它们收束成一张表，方便你在模型表现异常时快速定位。

| 类别 | 错误做法 | 典型现象 | 根本原因 | 正确做法 |
|------|----------|----------|----------|----------|
| 状态更新 | 写 `d.qpos` 后直接读 `d.xpos` | 观测滞后一帧 | 派生量没有重新计算 | 写完状态后调用 `mj_forward` |
| 仿真推进 | 用 `mj_forward` 代替 `mj_step` | 时间不走，机器人不动 | `mj_forward` 不积分 | 推进仿真只用 `mj_step` |
| 控制输入 | 把 `ctrl` 当成关节力矩 | 力矩大小完全不对 | `ctrl` 含义由 actuator 类型决定 | 先查 MJCF actuator 类型 |
| 外力输入 | 把外力写进 `ctrl` | 无执行器关节不受力 | `ctrl` 只经过执行器模型 | 广义外力写 `qfrc_applied` |
| 内存视图 | `saved = d.qpos` 后以为保存了快照 | 后续状态把快照覆盖 | NumPy view 指向同一内存 | 保存状态用 `.copy()` |
| 多线程 | 多条并行仿真共享同一个 `mjData` | 状态互相污染、结果随机 | `mjData` 是可变运行时缓冲 | 每条 rollout 单独创建 `mjData` |
| 接触刚度 | `solref[0]` 极小且 timestep 很大 | 落地弹飞、高频抖动 | 接触刚度超过积分稳定范围 | 减小 timestep 或放软接触 |
| 摩擦维度 | 所有接触都设 `condim=6` | 仿真明显变慢 | 每个接触约束维度变大 | 先用 3，需要扭转再升到 4/6 |
| MJCF 继承 | 滥用 `<default>` 多层覆盖 | 参数来源看不清 | 继承链太深 | 每个类只覆盖少数关键字段 |
| 名称索引 | 在代码里写死 `d.qpos[7]` | 改模型后控制错关节 | 索引随模型结构变化 | 用 named access 或 `mj_name2id` |

三个判断原则可以帮助你从表格中快速筛选：

1. **现象滞后一帧**：优先怀疑没有调用 `mj_forward`。
2. **现象高频抖动**：优先怀疑 timestep、接触刚度、执行器增益耦合。
3. **现象与关节数量相关**：优先怀疑索引、`nq/nv`、free joint、actuator 顺序。

> **本质洞察**：MuJoCo 的大多数“奇怪现象”不是随机的。它们通常来自三类边界被混淆：状态和派生量的边界、模型和数据的边界、控制输入和物理力的边界。把这三条边界画清楚，排查速度会比盲目调参快很多。

## S1.13 🔧 故障排查手册 ⭐⭐

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|----------|----------|----------|
| 机器人重置后第一帧观测错误 | 设置 `qpos/qvel` 后未调用 `mj_forward` | 1. 在 reset 末尾调用 `mj_forward`；2. 打印 reset 前后 `xpos`；3. 用 `.copy()` 保存对比快照 | S1.2, S1.6 |
| 落地瞬间弹飞 | `solref[0]` 太小、timestep 太大、阻尼不足 | 1. 打印最大穿透深度；2. 增大 `solref[0]`；3. 减小 timestep；4. 检查积分器设置 | S1.3, S1.5, S1.10 |
| 足端持续下陷 | 接触太软、质量过大、地面 geom 未参与碰撞 | 1. 检查 `d.ncon` 是否有接触；2. 检查 `contype/conaffinity`；3. 减小 `solref[0]`；4. 检查模型质量量纲 | S1.4, S1.10 |
| 夹爪法向力很大但物体滑落 | 摩擦系数太低或 `condim` 不足 | 1. 打印 `d.contact[i].friction`；2. 增大主摩擦系数；3. 圆柱/旋转物体尝试 `condim=4`；4. 检查夹爪接触点是否在物体上 | S1.5, S1.10 |
| 控制器输出后关节不动 | actuator 未绑定关节、`ctrlrange` 限制、`gear` 太小 | 1. 打印 `m.nu`；2. 用 actuator 名称设置 `ctrl`；3. 检查 `forcerange`；4. 读 `qfrc_actuator` 是否非零 | S1.4, S1.6 |
| 逆动力学残差很大 | 漏掉 `qfrc_applied/passive` 或状态未同步 | 1. 调 `mj_forward` 后再调 `mj_inverse`；2. 把 `qfrc_passive`、`qfrc_applied` 纳入残差；3. 清空外力重新测试 | S1.3 |
| 多线程 rollout 不可复现 | 共享了 `mjData` 或随机种子不独立 | 1. 每条 rollout 单独创建 `mjData`；2. 只共享只读 `mjModel`；3. 显式设置随机种子；4. 禁止跨线程写 `mjModel` | S1.2 |
| XML 加载失败但报错难懂 | include 路径、关节范围、惯量、名称冲突 | 1. 查看 `mj_loadXML` 的 error 缓冲；2. 暂时删除 include 缩小问题；3. 检查重复 name；4. 用 `mj_saveLastXML` 查看编译后模型 | S1.4, S1.6 |

故障排查时建议按“可观测量”而不是按“猜测原因”记录：

| 记录项 | 为什么重要 |
|--------|------------|
| `time`、`qpos`、`qvel` | 判断问题是否随时间发散 |
| `ncon`、`contact[i].dist` | 判断是否真的发生接触 |
| `qfrc_bias`、`qfrc_constraint`、`qfrc_actuator` | 判断力来自重力、接触还是执行器 |
| `solver_niter` 或相近求解器统计量 | 判断约束求解是否收敛，具体字段以官方文档为准 |
| `sensordata` | 判断观测是否与物理状态同步 |

## S1.14 累积项目：MuJoCo 模型诊断脚手架 ⭐⭐

**累积项目进度**：

| 章节 | 新增模块 | 累积能力 |
|------|---------|---------|
| **S01（本章）** | **模型诊断脚手架** `mj_diagnose` | 模型加载、字段检查、力分解、接触诊断、CSV 记录 |
| S02（下章预告） | 交互式控制实验台 | 在诊断脚手架基础上增加 viewer、控制器、日志、回放 |
| S03（后续） | GPU 并行训练环境 | 在实验台基础上增加批量仿真和 RL 训练 |

本章的累积项目不是训练一个策略，而是写一个**模型诊断脚手架**。后续 S02/S03 做交互式控制、GPU 并行训练和可微分仿真时，它会成为第一道防线。

### 项目目标 ⭐⭐

用 Python 或 C++ 实现一个命令行工具：

```bash
mj_diagnose --model unitree_go2/scene.xml --steps 2000 --csv out.csv
```

工具应输出以下内容：

| 模块 | 输出 | 用途 |
|------|------|------|
| 模型摘要 | `nq/nv/nu/nbody/njnt/nsensor` | 确认模型维度 |
| actuator 摘要 | 名称、类型、`ctrlrange`、`forcerange` | 确认控制输入语义 |
| 接触摘要 | 每步 `ncon`、最大穿透深度、接触 geom 名称 | 判断接触是否合理 |
| 力分解 | `qfrc_bias/qfrc_constraint/qfrc_actuator` 范数 | 判断力源 |
| 传感器摘要 | sensor 名称、维度、均值、最大值 | 判断观测是否可用 |
| CSV 记录 | 时间、质心高度、接触数、最大穿透 | 用于画图和回归测试 |

### 实现建议 ⭐⭐

1. 先只支持 Python，快速验证字段和数据格式。
2. 再把核心诊断函数迁移到 C++，用于实时控制工程。
3. 每次修改 MJCF 后，先跑 `mj_diagnose`，再训练 RL 或调 MPC。
4. 把 CSV 画成三张图：质心高度、接触数量、最大穿透深度。
5. 给每个模型保存一份基准曲线，后续改参数时用来判断是否引入异常。

### 验收标准 ⭐⭐

| 标准 | 合格表现 |
|------|----------|
| 可重复 | 同一模型、同一初始状态下多次输出一致 |
| 可解释 | 每个输出字段都有单位和物理含义 |
| 可定位 | 接触异常时能指出 geom 名称 |
| 可扩展 | 能增加新的 sensor 或 force 项而不改主循环 |
| 可对比 | 两次运行的 CSV 能直接画图比较 |

这个项目会迫使你把本章的所有概念串起来：MJCF 编译成 `mjModel`，运行状态保存在 `mjData`，`mj_step` 推进时间，接触参数改变 `d.contact` 和 `qfrc_constraint`，传感器读数来自 `sensordata`。如果这个脚手架写清楚，后续学习 MJPC、MJX 和可微分仿真时，很多问题会提前暴露。

## S1.15 本章小结 ⭐

| 知识点 | 你现在应该掌握的核心判断 | 常见误解 |
|--------|--------------------------|----------|
| MuJoCo 物理哲学 | 软约束和优化模型让接触可诊断、可反解、适合控制 | 把 penetration 一律当成 bug |
| `mjModel` | 只读编译产物，保存拓扑、惯量、执行器、传感器、求解器选项 | 以为它只是 XML 的内存版 |
| `mjData` | 可变运行时状态和中间缓存，每次仿真都被更新 | 在线程间共享同一份 `mjData` |
| `mj_step` | 推进时间，更新状态、接触和传感器 | 用它做查询导致时间意外前进 |
| `mj_forward` | 给定当前状态，刷新派生物理量但不积分 | 用它代替仿真循环 |
| `mj_inverse` | 给定加速度反解广义力，可用于力学诊断 | 只把它理解成 Pinocchio RNEA 的重复 |
| 积分器 | 在稳定性、精度和速度之间取舍 | 认为阶数越高越适合接触 |
| 接触参数 | `solref/solimp/friction/condim` 共同决定穿透、阻尼和摩擦维度 | 同时乱调所有参数 |
| MJCF | 能表达接触、执行器、传感器、闭链、腱和场景 | 只把它看成 URDF 的另一种语法 |
| C/Python API | 同一套模型-数据-函数语义，语言只是封装层 | 以为 Python 行为和 C 内存无关 |

用一句话收束本章：

> MuJoCo 的核心不是某个函数名，而是 **MJCF 编译出的 `mjModel` + 每步更新的 `mjData` + 由软约束优化定义的接触动力学**。理解这三者的关系，才算真正会用 MuJoCo 做机器人控制仿真。

## S1.16 延伸阅读 ⭐

| 资料 | 难度 | 阅读重点 |
|------|------|----------|
| Todorov, Erez, Tassa, *MuJoCo: A physics engine for model-based control*, IROS 2012 | ⭐⭐⭐⭐ | Gauss 原理、接触优化、模型控制动机 |
| [MuJoCo 官方文档：Computation](https://mujoco.readthedocs.io/en/stable/computation/) | ⭐⭐⭐⭐ | 正/逆动力学、约束求解、积分器、接触模型 |
| [MuJoCo 官方文档：XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) | ⭐⭐⭐ | MJCF 标签、`option`、`default`、`actuator`、`sensor`、`equality` |
| [MuJoCo 官方文档：API Reference](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html) | ⭐⭐⭐ | `mj_step`、`mj_forward`、`mj_inverse`、内存管理和枚举 |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | ⭐⭐ | 高质量 MJCF 模型写法，尤其是四足、机械臂和夹爪 |
| [DeepMind Control Suite](https://github.com/google-deepmind/dm_control) | ⭐⭐⭐ | 如何把 MuJoCo 模型包装成连续控制 benchmark |
| [Pinocchio 文档](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/) | ⭐⭐ | 对照理解只读模型与可变数据缓冲 |

阅读顺序建议：

1. 先读官方 XML Reference 中自己模型用到的标签，不要一次性背完整 XML。
2. 再读 Computation 章节，把 `mj_step` 内部流程和本章流程图对应起来。
3. 遇到 API 名称不确定时查 API Reference，尤其是字段、枚举和函数签名。
4. 读 Menagerie 模型时，不要只复制参数；要问每个 `default`、`actuator`、`sensor` 为什么这样写。
5. 读 Todorov 2012 时重点抓住建模哲学，不必第一次就推完所有优化细节。

### 练习：把模型诊断变成日常流程 ⭐

**A 型**：选择一个包含 free joint 的机器人模型，分别打印 `nq`、`nv`、`nu`、`nbody`、`njnt` 和 `nsensor`，解释为什么 `nq` 与 `nv` 不一定相等。

**A 型**：手动修改 `qpos` 中一个关节角，先读取 `xpos`，再调用 `mj_forward` 后再次读取 `xpos`。写出两次结果为什么不同，并说明这类错误在 reset 函数中会造成什么后果。

**B 型**：把同一模型的 timestep 分别设为 0.002、0.005 和 0.01，在固定 PD 增益下记录最大穿透深度、接触数量和质心高度。根据曲线判断不稳定来自积分步长、接触刚度还是控制器增益。

**B 型**：为一个夹爪抓取圆柱体的模型分别设置 `condim=3` 和 `condim=4`，比较物体绕法向旋转时的表现。解释为什么扭转摩擦不是所有接触都需要，但在某些操作任务中非常关键。

**C 型**：设计一个 `mj_diagnose` 输出报告模板，要求报告同时包含模型维度、执行器范围、接触统计、传感器统计和力分解。报告不应只给数值，还要给出每个异常指标对应的排查建议。
