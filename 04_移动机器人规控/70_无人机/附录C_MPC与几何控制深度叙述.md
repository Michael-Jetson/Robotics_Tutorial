## 附录C：MPC与几何控制深度叙述

> → 对应主大纲章节：D1-D2（SE(3)几何控制/MPC实时控制）

### 四旋翼控制的十五年历程：从SE(3)到基础策略

**2025年的共识是混合的，而非单一的。**基于acados/HPIPM的非线性MPC仍是约束关键型飞行的主力，当目标为任务级别（圈速、门点通过）时，强化学习占据主导地位，而二者通过神经动力学MPC和残差策略实现融合。 开源领域的核心位于苏黎世大学RPG实验室，苏黎世联邦理工学院（ASL、ADRL）、香港大学MaRS、Kumar Robotics以及纽约大学ARPL则是其他支柱。 自 2010 年以来，每一项重大进展都是对当年播下的三个理念的深化：Lee–Leok–McClamroch 的 SE(3) 几何追踪、Mellinger–Kumar 的微分平坦性，以及后来演变为 ACADO 并最终成为 acados 的代码生成 SQP-RTI 模式。 发生变化的则是求解器栈（qpOASES → HPIPM）、动作空间（力矩 → 姿态角速度），以及神经网络的部署位置（无 → 成本函数内部 → 动力学内部 → 取代控制器）。

本报告追溯了这一演变历程，修正了经典文献的引用，核查了实际发布的实现版本的星标数量及 C++ 代码模式，并论证了对于从 SLAM 领域转向无人机控制的工程师而言，其思维模型具有惊人的可移植性：**SE(3) 控制器本质上是由 PD 律驱动的 BetweenFactor 残差，而四旋翼 MPC 则是带有输入不等式的固定时滞平滑器**。

### 发展全景一览

| 年份 | 里程碑 | 重要性 |
|---|---|---|
| 2010 | Lee, Leok, McClamroch CDC — SE(3) 空间中的跟踪 | 无欧拉奇点的近全局稳定性 |
| 2011 | Mellinger–Kumar ICRA — 微分平坦性 + 最小瞬变 | 基于位置轨迹的闭式前馈 |
| 2017 | ACADO + qpOASES 四旋翼 NMPC (Kamel 等) | SQP-RTI 使 100 Hz NMPC 机载可行 |
| 2018 | PAMPC（Falanga等人，IROS）；Control Toolbox（Giftthaler等人，SIMPAR） | 感知闭环MPC；基于模板的C++ OCP库 |
| 2021 | CPC（Foehn等人，《Science Robotics》）；GP-MPC（Torrente等人，RA-L） | 时间最优 NLP；学习到的空气动力学残差 |
| 2022 | Agilicious（Foehn 等，Science Robotics）；L1-NMPC（Hanover 等，RA-L）；Neural-Fly（O&#x27;Connell 等，Science Robotics） | 开放式全栈平台；自适应 NMPC；元学习风力抑制 |
| 2023 | Neural-MPC（Salzmann等人，RA-L）；Swift（Kaufmann等人，《自然》）；Reaching-the-Limit（Song等人，《科学机器人》） | 仿生机内的神经网络；强化学习超越人类冠军并胜过MPC |
| 2024–25 | MPCC++（Krinner等人）；Learning-to-Fly（Eschmann等人）；SimpleFlight（Chen等人）；RAPTOR（Eschmann等人） | 安全约束下的时间最优MPC；18秒强化学习训练；基础策略 |

### SE(3)几何控制，取代欧拉流形的流形

**基础参考文献为 Taeyoung Lee、Melvin Leok 和 N. Harris McClamroch 的《基于 SE(3) 的四旋翼无人机几何跟踪控制》（&quot;Geometric Tracking Control of a Quadrotor UAV on SE(3)&quot;），IEEE CDC 2010，第 5420–5425 页（arXiv:1003.2005）。** 它用三个流形原生误差取代了欧拉角PID（该方法在俯仰角 $\pm$90°处会发散）：姿态配置误差 `Ψ(R, R_d) = ½ tr(I − R_dᵀ R) ∈ [0, 2]`、切空间姿态误差 `e_R = ½ vee(R_dᵀR − RᵀR_d)` 以及传递角速度误差 `e_Ω = Ω − RᵀR_d Ω_d`。 该论文证明了在任意子集 `{Ψ < ψ₁ < 2}` 上均具有指数稳定性，仅排除了 $\pi$ 旋转鞍点的测度为零的集合。这是 SO(3) 上连续反馈所能达到的最佳拓扑性能——巴特-伯恩斯坦障碍定理排除了真正的全局渐近稳定性。其优势体现在能够从倒置的初始状态恢复，而这种机动在欧拉 PID 控制下是无法实现的。

**由梅林格（Mellinger）和库马尔（Kumar）于一年后发表的《微分平坦性》（ICRA 2011，最佳论文——自动化领域），实现了规划器与控制器之间的闭环连接。** 这四个平滑输出 `(x, y, z, ψ)` 通过代数运算确定了所有状态和输入，仅在位置和偏航加速度上保留瞬变（snap）：推力方向来自二阶导数，机体角速度来自三阶导数（抖动），机体力矩来自四阶导数（瞬变）。 因此，最小瞬变多项式轨迹会生成一个具有解析前馈的*动态可行*参考轨迹——这与VIO工程师用于IMU预积分的数学机制相同，因为`Ṙ = Rω̂`可积分得到`R ← R · exp(ω̂ Δt)`，即Rodrigues映射。

#### 实际发布的实现

**rpg_quadrotor_control** 仓库（github.com/uzh-rpg/rpg_quadrotor_control，**694★**，C++11，MIT）是 UZH RPG 栈的标准实现。 其架构采用四层级联设计——自动驾驶仪状态机、100 Hz 位置控制器、sbus/RotorS 桥接层以及 1 kHz 的 FCU 速率环路——并暴露了一个带有模板标量类型的 `ControllerBase` 抽象类，从而使 `MpcController<double>` 和 `LargeAngleController<double>` 能够通过策略模式在后台互换。 **mavros_controllers** (github.com/Jaeyoung-Lim/mavros_controllers, **500★**, C++11/14, BSD-3) 是 PX4 的最小化 SE(3) 追踪器： 它在机身速率 + 集体推力设定点处终止（将内循环交由 PX4 的 `mc_rate_control` 处理），通过 `drag_dx/dy/dz` 参数实现 Faessler 旋翼阻力模型，并且是利用现成硬件进行几何控制实验的最快途径。 **KumarRobotics/kr_mav_control**（子仓库中**~61★**， 父仓库 `kr_autonomous_flight` 获 755★) 是 Mellinger/Kumar 代码直接承袭自 GRASP-lab 的成果，其显著特点在于 `SO3Command` 消息类型承载原始 `(f, R_des, Ω_d, Ω̇_d)` 数据，以及一个显式的运行时稳定性阈值——当 `Ψ > 1` 值（即理论稳定域的一半，一个务实的安全裕度）被触发时，该阈值会拒绝位置控制。

**PX4 的 `mc_att_control` 几乎是 SE(3)，但尚未完全达到。** 它采用布雷西亚尼尼（Brescianini）四元数公式，结合了减小俯仰角加偏航混合：`ω_sp = K_P ⊙ (2 · sign(q_{e,w}) · q_{e,xyz})`，该公式在 `π` 误差范围内与李（Lee）的 `e_R` 代数等价，但缺少陀螺前馈 `Ω × JΩ` 以及李亚普诺夫（Lyapunov）吸引盆证明。 它还仅输出机身速率设定值，将 `M = Jω̇ + ω×Jω` 留给内部 PID 控制。这就是为什么针对 PX4 的学术 SE(3) 堆栈始终停留在 `ω_d` 的原因——内环属于 NuttX。

**对 SLAM 工程师的教学价值：**姿态误差 `e_R = ½ vee(R_dᵀR − RᵀR_d)` 是 `log(R_dᵀR)^∨` 的第一阶近似，这恰好是 GTSAM 的 `BetweenFactor<Pose3>` 线性化中使用的最小残差。 帽子/V形算子、指数收缩 `R ← R · exp(δω̂)` 以及 Sophus 的 `SO3d` 均被原样沿用。一句话的洞见：SE(3) 控制器就是 GTSAM 的姿态残差经过 PD 增益处理后的结果。

### 经典MPC时代、ACADO及acados的兴起

四旋翼MPC沿着单一求解器谱系发展成熟。 **ACADO工具包**（Houska, Ferreau, Diehl 2011）引入了带实时迭代（RTI）的SQP——每MPC时钟周期执行一次SQP迭代，分为测量数据到达前的准备阶段（积分动力学、压缩QP）和到达后的反馈阶段（求解QP、返回`u₀*`）。 其C++代码生成器，配合活性集QP求解器**qpOASES**，成为了2014至2020年间几乎所有四旋翼NMPC论文的技术基石。

**rpg_mpc** (github.com/uzh-rpg/rpg_mpc, **486★**, C++11, GPL-3.0) 是 **Falanga, Foehn, Lu, Scaramuzza, &quot;PAMPC: 四旋翼机的感知感知模型预测控制&quot;一文的教科书式范例及参考实现，发表于IROS 2018（arXiv:1804.04811）**。 其问题结构极具启发性：10个状态（位置、速度、四元数），4个输入（总推力 + 3个机身角速度），采用RK2积分的`N=20`射击节点，`~0.1s`步长，以及一个感知成本项——该项会对将3D兴趣点重新投影到相机光轴的行为施加惩罚。 值得借鉴的工程模式是三层架构：由 ACADO 生成的 C 语言求解器、一个 `mpc_wrapper<T>`（它使用 `Eigen::Map<Eigen::Matrix<T, kStateSize, kSamples+1>>` 将求解器的原始列优先数组封装为符合惯例的 Eigen 矩阵，且不进行复制），以及一个 `mpc_controller<T>`（它在 ROS 上发布预测轨迹）。 `Eigen::Map` 的零拷贝模式可直接应用于任何代码生成的求解器集成。

**mav_control_rw** (github.com/ethz-asl/mav_control_rw, **~430★**, C++11) 来自 ETH ASL（Kamel, Stastny, Alexis, Siegwart — ROS 书籍第 2017 章，arXiv:1611.09240），是经典的 **线性与非线性 MPC 对比**。 其 `mav_linear_mpc` 包利用 CVXGEN（Mattingley–Boyd，斯坦福大学）实现悬停状态下的亚毫秒级线性 MPC，而 `mav_nonlinear_mpc` 则采用 ACADO + qpOASES。 更重要的是，该研究提供了一个**基于卡尔曼滤波器的外部扭矩观测器**，通过将估计的力/扭矩偏差叠加到MPC参考信号上，实现了在风载或载荷作用下无偏移的跟踪——这一模式被后续所有论文所效仿。

**Control Toolbox (CT)** (github.com/ethz-adrl/control-toolbox, **1.6k★**, C++14, BSD-2）由 Buchli/Hutter ADRL 团队（**Giftthaler, Neunert, Stäuble, Buchli, SIMPAR 2018, arXiv:1801.04290**）开发，则是一项截然不同的成就。它是现代 C++ 中 *模板化* 最优控制的黄金标准：

```cpp
template<size_t STATE_DIM, size_t CONTROL_DIM, size_t P_DIM, size_t V_DIM,
         typename SCALAR, bool CONTINUOUS>
class iLQR { … };
```

编译时维度在所有位置生成固定大小的 `Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM>`，从而消除了堆内存分配并最大限度地提高了向量化效率。 CppAD + CppADCodegen 提供了针对用户定义成本函数和约束函数的 JIT 编译自动微分功能。该库将 iLQR、GNMS（高斯-牛顿多重射击法）和 DMS 统一在单一的 Riccati 后端之下——这一概念性突破表明 iLQR 是 SQP 多重射击家族的单次射击极限。 v3.0.2-beta 版本（2022年3月）将 HPIPM 作为结构化 QP 后端。根据 README 所述，该项目“自 2021 年 7 月以来几乎未维护”，但仍是关于如何在模板化 C++ 中构建 OCP 代码的最简洁参考。

**acados (github.com/acados/acados, 1.3k★, BSD-2)** 是当前的主流选择，由与 ACADO 相同的弗莱堡研究组开发（Verschueren, Frison, Kouzoupis, Frey, Diehl 等，*Math. Prog. Comp.* 14:147–183, 2022）。 它与 ACADO 的主要区别在于结构：acados 拥有一个 **C 运行时**（而非纯代码生成），并提供 Python/MATLAB/Simulink 前端； 它用**BLASFEO**替代了LAPACK，后者针对嵌入式优化中常见的小型和中型矩阵进行了优化；其默认QP求解器是**HPIPM**，这是一种内点法，通过Riccati递归利用了OCP的块带状KKT结构——在数学上与SLAM平滑器中使用的正向-反向信息滤波递归完全相同。 近期新增功能包括 AS-RTI 四级实时迭代（arXiv:2403.07101, 2024）、多相 OCP（Frey 等，*Optimal Control Applications and Methods*, 2025），以及一个可微分 MPC 层，该层为强化学习（RL）训练提供解的敏感度信息。 2023–2025年间所有主要的四旋翼MPC论文——Salzmann的Neural-MPC、Romero的MPCC/MPCC++以及Agilicious MPC——均可在acados上运行。

#### 四旋翼MPC与机械手MPC的对比

问题结构之间的差异远大于求解器栈之间的差异。对于6自由度姿态控制，四旋翼MPC具有**10–13个状态和4个输入**，因此从结构上来说属于欠驱动系统——位置是通过姿态来控制的，这也是为什么级联架构（位置 → 姿态 → 角速度）被广泛采用的原因。 一个7自由度机械臂具有**14个状态和7个输入**，在关节空间中是完全驱动的，并使用OCS2（github.com/leggedrobotics/ocs2，约1.1k★）或Crocoddyl，配合Pinocchio动力学后端，通过解析导数实现约1微秒的RNEA调用。 四旋翼机的动力学带宽达千赫兹量级（旋翼时间常数为10–50毫秒），而机械臂仅约100赫兹，但两者最终均以50–200赫兹的频率运行MPC。 不确定性结构存在显著差异：四旋翼机面临平滑的、与速度相关的空气动力学效应，这些效应是 GP 或 NN 残差的理想目标；而腿足机器人则面临非平滑的接触脉冲，需要采用具有互补性或增强拉格朗日约束的 DDP。微分平坦性使四旋翼机能够获得闭式前馈，而机械臂通常无法享有这一优势。

### 自适应与鲁棒MPC：吸收模型无法观测的信息

三个开源项目定义了这一领域。**Torrente、Kaufmann、Foehn、Scaramuzza，《四旋翼机的数据驱动MPC》，RA-L 7(2):3769–3776, 2021 (arXiv:2102.05773)** — 实现代码见 github.com/uzh-rpg/data_driven_mpc，**347★**， Python + acados — 通过在机体速度上应用三个轴向稀疏高斯过程来增强名义刚体动力学，使用约20个诱导点以确保评估成本足够低，适合acados运行。该方法在14 m/s速度下将轨迹跟踪误差降低了高达70%，并捕获了在高速下占主导地位的寄生阻力、诱导阻力和旋翼拍动效应。

**Hanover, Foehn, Sun, Kaufmann, Scaramuzza, &quot;性能、精度与载荷： 四旋翼自适应非线性MPC”，RA-L 7(2):690–697，2022年4月（arXiv:2109.04210）**将**L1自适应控制器**置于NMPC之上而非其内部。 该 L1 环路——包含状态预测器、分段常数适应律以及一阶低通滤波器——运行速度足够快，能够吸收旋翼推力层面的匹配与非匹配扰动，而 NMPC 则基于标称模型进行规划。 实验结果令人瞩目：**对未知悬挂载荷（最高达无人机质量的60%）的跟踪，均方根误差（RMSE）降低超过90%**，且无需重新调谐即可达到70公里/小时的最高速度。 据报告，该控制器在 i7-8750H 处理器上的更新时间为 0.82 毫秒——在 0.81 毫秒的 NMPC 基础上几乎无需额外时间——而 GP-MPC 为 4.13 毫秒，MPPI 为 23.13 毫秒。 在标称条件下，L1-NMPC与INDI的均方根误差（RMSE）均在5毫米以内，但当载荷质量发生变化时，L1-NMPC表现更为出色，因为INDI需要针对每种配置进行重新参数化，而L1-NMPC能够在线自适应调整。

**O&#x27;Connell、Shi、Shi等人，《Neural-Fly 助力强风环境下敏捷飞行的快速学习》（Science Robotics 7(66):eabm6597，2022年5月4日）**（加州理工学院，代码见 github.com/aerorobotics/neural-fly）则采用了元学习路径。 DAIML通过6种风况下12分钟的飞行数据，学习出一种抗风特征基`Φ(x) ∈ R^h`；在线训练时仅更新最终层的小量系数，因此该算法可在树莓派4上高效运行。研究报告显示，在加州理工学院1296扇叶风洞中，当风速高达43.6公里/小时时，其跟踪误差比传统方法小2.5–4倍。 **KNODE-MPC**（Chee, Jiahao, Hsieh, RA-L 2022, arXiv:2109.04821）采用基于知识的神经常微分方程作为残差项，相较于 GP-MPC，其模拟性能提升了 60.2%，实际应用性能提升 $\ge$21%。

### Agilicious：作为软件架构的研究平台

**Foehn, Kaufmann, Romero, Penicka, Sun, Bauersfeld, Laengle, Cioffi, Song, Loquercio, Scaramuzza, “Agilicious：基于视觉飞行的开源及开源硬件敏捷四旋翼机”，《Science Robotics》7(67):eabl6259，2022年6月22日**（DOI:10.1126/scirobotics.abl6259；代码库 github.com/uzh-rpg/agilicious，**574★**）与其说是一个单一算法，不如说是一个凝聚了十年经验教训、具有鲜明设计理念的C++17架构。 该平台本身是一架5英寸FPV级竞速四旋翼机：采用Armattan Chameleon机架、HobbyWing XRotor四合一电调、搭载运行**定制版agiNuttx RTOS分支**的BrainFPV Radix飞行控制器（该系统接受单旋翼推力指令而非机身姿态指令），并配备Jetson TX2或Orin NX辅助计算机。 推重比为5.0，最高时速131公里，0–100公里/小时加速仅需1.01秒。

其关键的架构贡献在于**Pilot → Bridge → Guard**模式：

- **Pilot** 是一个状态机，托管一个或多个 **Pipelines**，每个 Pipeline 都是 `{Estimator, Sampler, Reference, Controller, Bridge}` 的组合。控制器（MPC、DFBC、INDI、几何 PID、L1-NMPC）是基于共同抽象基类的策略模式子类，并通过工厂模式从 YAML 配置中实例化。Pipelines 可在运行时热插拔。
- **Bridge** 是一个适配器，用于将控制器输出与硬件隔离：包括 RotorS、Agisim（内部开发的含叶片单元空气动力学模型）、Flightmare、连接至机载版本相同代码的 Laird 无线链路，以及连接至 agiNuttx 的 USB/UART 接口。相同的 C++17 控制器二进制文件可在模拟环境和硬件上运行，无需重新编译。
- **Guard** 是一个并行安全管道，拥有独立的估计器（通常基于动作捕捉数据）。它与用户管道并行运行；若发生不变量违规——姿态偏离、越界、电量不足、控制器失效——Guard 将接管控制并执行故障安全轨迹。正是这一机制让博士生们能够以 70 公里/小时的速度飞行实验性 MPC 系统，而不会损坏硬件。

**Agilicious 与 PX4 的信任边界恰好相反。** PX4 将 STM32 MCU 视为自动驾驶仪，而将辅助计算机视为可选协处理器，所有控制均在 NuttX 上以 $\le$2 kHz 的频率运行。 Agilicious 将 STM32 视为执行器，并将高频控制（100–500 Hz MPC、500 Hz INDI）推送到 Jetson 上，仅将 agiNuttx 用于 2 kHz 的最内层单旋翼推力环路。PX4 侧重于广度（固定翼、垂直起降、漫游车、水下机器人）和认证； 而 Agilicious 则侧重于敏捷性和研究迭代。其公开的 GitHub 仓库主要包含文档——C++17 源代码受 UZH 学术许可协议约束——这正是其星标数量未能充分体现其影响力的原因。

除 MPC 外，Agilicious 还提供了另外两个值得了解的控制器。**DFBC**（Faessler, Franchi, Scaramuzza, RA-L 3(2):620–626, 2018) 证明了带线性旋翼阻力的四旋翼模型具有微分平坦性，并利用该平坦性映射实现闭式前馈控制；其运行速度快于 NMPC 且数值可靠性更高，但在动态不可行参考点上的表现较弱（Sun 等，T-RO 38(6):3357–3373, 2022)。**INDI**（Smeur、Chu、de Croon，《JGCD》39(3):450–461, 2016) 利用了小增量线性化 `ω̇ ≈ ω̇₀ + G(x)(u − u₀)`，因此仅需控制有效性矩阵 `G(x)` ——其余一切均通过测得的角加速度来处理。INDI 本质上对气动模型误差具有鲁棒性，但需要经过良好滤波的陀螺仪导数。

### 规划与控制的融合：当规划即控制

经典的管道式架构——规划器生成轨迹，跟踪器执行该轨迹——存在两个结构性缺陷。规划器忽略了执行器的快速动态特性及延迟。跟踪器无法推断未来的障碍物。近期有三条研究路线将这两者融合在一起。

**Foehn、Romero、Scaramuzza，《四旋翼机航点飞行的时效最优规划》，《Science Robotics》6(56):eabh1221，2021年7月21日（DOI:10.1126/scirobotics.abh1221）**提出了一个单一的大型自然语言处理（NLP）模型——**互补进度约束（CPC）**——该模型能够编码整个多航点飞行路径。 决策变量包括完整的状态轨迹 `x_k ∈ ℝ^13`、单旋翼推力 `u_k ∈ ℝ^4`、各航点进度变量以及节点级时间步长。类似互补性的约束确保每个航点被精确访问一次，但不固定访问时间。目标函数为总飞行时间。 通过 CasADi 结合 IPOPT 求解，该方法生成的真实双稳态执行器调度方案是多项式最小瞬变轨迹无法比拟的——这是首个在苏黎世大学飞行机器竞技场（UZH Flying Machine Arena）的同一赛道上**击败世界级人类飞行员**的优化方法。 参考实现（github.com/uzh-rpg/rpg_time_optimal, 119★, Python）离线运行需耗时数分钟至数小时，这促使了后续实时研究的开展。

**Romero, Sun, Foehn, Scaramuzza, &quot;Model Predictive Contouring Control for Time-Optimal Quadrotor Flight,&quot; IEEE T-RO 38(6):3340–3356, 2022**（arXiv:2108.13205）及其2024年的后续版本**MPCC++（Krinner等人，arXiv:2403.17551）**将这一思路实现了在线化。 在沿参数化参考轨迹 `p_d(θ)` 飞行时，轨迹优化与路径滞后误差被同时最小化——规划与跟踪合二为一。 MPCC++ 引入了 Frenet–Serret 隧道约束以提供递归可行性安全保证，并集成了学习型空气动力学残差及 TuRBO 贝叶斯超参数调优，在 Agilicious 平台上以 100 Hz 运行（N=20），且能在赛道上以超过 80 km/h 的速度无碰撞飞行。

**Liu, Ren, Zhang, &quot;Integrated Planning and Control for Quadrotor Navigation in Presence of Suddenly Appearing Objects and Disturbances,&quot; RA-L 2023 (IEEE Xplore 10238764)** 来自香港大学 MaRS 实验室 (github.com/hku-mars/IPC, **240★**, C++14/17, ROS1）将这一理念从赛道导航扩展至杂乱环境导航。每个预测状态均受硬约束，必须位于**安全飞行走廊**多面体 `A_k x_k ≤ b_k` 内（Liu, Watterson, Mohta, Kumar RA-L 2(3), 2017）； 通过线性化动力学，将整个问题转化为包含数百个变量的稀疏凸优化问题；**OSQP**（Stellato, Banjac, Goulart, Bemporad — 基于ADMM、支持暖启动、具备不可行性检测）通过`osqp-eigen`绑定以**100 Hz**的频率求解该问题。 其结果是能够避开突然出现的横穿无人机和狗，同时抵御风的影响——这些行为是分流管道无法实现的，因为追踪器缺乏对未来障碍物的感知能力。ZJU-FAST-Lab的CMPCC采用MPCC而非追踪MPC，追求相同的理念。

**对于SLAM工程师而言，CPC的NLP在结构上属于束调整。** 状态变量 `x_k` 和输入 `u_k` 扮演相机姿态的角色；拍摄等式约束 `x_{k+1} = f(x_k,u_k)Δt` 扮演重投影残差的角色；KKT 雅可比矩阵在时间维度上呈块三对角结构，这恰好是 iSAM2 海森矩阵的带状结构。 IPOPT 的 MA27/MA57/MUMPS 线性求解器在 Ceres 中扮演着稀疏 Cholesky 矩阵的角色。HPIPM 的 Riccati 递归在数学上与正向-反向信息滤波递归完全一致。其思维模型是：插值节点是关键帧，动力学是强先验，而障碍物则是 SLAM 所不具备的侧面不等式。

### 2023–2025年的前沿领域：神经动力学与学习策略

在Agilicious之后，出现了两个并行的研究方向。第一个方向是将神经网络**嵌入到MPC中**。**Salzmann、Kaufmann、Arrizabalaga、Pavone、Scaramuzza、Ryll的论文《实时神经-MPC： 四旋翼及敏捷机器人平台的深度学习模型预测控制”，RA-L 2023 (arXiv:2203.07747)** 证明，通过局部泰勒逼近和三阶段优化循环，PyTorch动力学模型（其参数容量是解析模型的**4000倍**）能够适配于50 Hz acados OCP中。 相较于标准MPC，**位置跟踪误差最高可降低82%**。ml-casadi桥接器现已合并至**l4casadi**（github.com/Tim-Salzmann/l4casadi），该版本是当前积极维护的后续版本。261★（github.com/TUM-AAS/neural-mpc）上的Neural-MPC是参考实现。

第二种方案则完全用**深度强化学习**取代了MPC。 **Kaufmann, Bauersfeld, Loquercio, Müller, Koltun, Scaramuzza, &quot;利用深度强化学习实现冠军级无人机竞速,&quot; 《自然》620:982–987, 2023年8月30日 (DOI:10.1038/s41586-023-06419-4)** 是具有决定性意义的研究成果： Swift 是一款两层多层感知机（MLP）控制策略，它在模拟环境中通过 PPO 进行训练，并通过卡尔曼滤波器与卷积神经网络（CNN）门控检测器及 RealSense T265 视觉惯性融合（VIO）系统融合。该系统在 Jetson TX2 平台上，使用 870 克的 Agilicious 级无人机，在 8 毫秒的推理预算下，**在一对一的实体竞速中击败了三位人类 FPV 世界冠军**。 动作空间为**CTBR（总推力 + 机身角速度）**。基于真实世界数据识别出的残差模型弥合了模拟与现实之间的差距。

**Song, Romero, Müller, Koltun, Scaramuzza, &quot;Reaching the limit in autonomous racing: Optimal control versus reinforcement learning,&quot; Science Robotics 8(82):eadg1462, 2023年9月13日** 提供的系统性对比，进一步巩固了2025年的发展叙事。 其核心论点令人惊讶：**强化学习的优势不在于对同一目标进行更优的优化，而在于优化一个不同且更优的目标**。MPC（模型预测控制）是在一个本已次优的参考轨迹上最小化轨迹跟踪误差；而强化学习则直接针对整圈赛程优化任务层面的关卡进度，因此能够利用MPC因固定时限和光滑性假设而压制的非平滑执行器边界及未建模效应。 经过训练的强化学习策略实现了峰值加速度 &gt;12 g 和峰值速度 108 km/h，且“在标准工作站上仅需数分钟训练即可达到超人级操控水平”。

**Kaufmann, Bauersfeld, Scaramuzza, &quot;敏捷四旋翼飞行学习控制策略的基准比较,&quot; ICRA 2022 (arXiv:2202.10796)**确立了后续所有论文遵循的动作空间正统范式：对线性速度、集体推力+机身角速度（CTBR）以及单旋翼推力（SRT）进行了基准测试，结果显示**CTBR在模拟到实机的鲁棒性迁移方面胜出**。 CTBR 将电机动力学隐藏在内部速率环路之后，与 Betaflight 和 PX4 固件的接口相匹配，并为 PPO 提供了正确的抽象梯度。

**Huang, Rana, Spitzer, Shi, Boots, &quot;DATT: 深度自适应轨迹跟踪,&quot; CoRL 2023 (arXiv:2310.09053)** 在基于强化学习训练的前馈-反馈策略上叠加 L1 自适应控制，以实现非稳态风条件下的零样本部署，据报告其误差比自适应 MPC 低 34–54%，并在 Crazyflie 2.1 上相较于先前的强化学习基线实现了 50% 的跟踪误差降低。 2025年的**RAPTOR**策略（Eschmann等人，arXiv:2509.11481）将1000个强化学习教师策略蒸馏为单个基于GRU的三层学生模型，仅含**2084个参数**，即可在32克至2.4千克的四旋翼无人机上实现零样本泛化——这已成为无人机控制的基础策略。

#### 那么2025年谁将胜出？

坦率地说，这个问题本身就是错误的。2025年的共识分为三个层次：

- **MPC 胜出**的情形包括：当硬约束（如行驶通道、障碍物、推力限制）至关重要时；当需要性能保证时；当给定参考车辆时；或当目标车辆在模拟中未曾出现时。IPC、MPCC++ 和 L1-NMPC 在这一领域占据优势。
- **强化学习（RL）胜出**：当目标为任务级且具有长时效性（如单圈时间、门点进度）时；当通过领域随机化能弥合仿真与实机的差距时；以及当推理延迟必须控制在毫秒级以下时。Swift、SimpleFlight 和 RAPTOR 在这一领域占据主导地位。
- **混合方法在其他所有场景中均表现优异。** 神经MPC将学习到的动力学嵌入acados模型中。DATT通过L1适应性增强了RL。MPCC++添加了学习到的气动残差。GP-MPC和KNODE-MPC保留名义动力学但学习校正项。

求解器栈正在趋同：MPC采用带HPIPM的acados，RL采用带非对称演员-批评者的PPO，CasADi用于两者的符号微分。 动作空间正趋于统一：CTBR几乎适用于所有场景。心理模型正趋于统一：动力学采用混合模式（解析模型 + 学习残差），感知与成本或观测紧密耦合，安全机制则外化为Guard风格的监督器。

### 反复出现的C++架构模式

有五种惯用模式几乎出现在每个严肃的代码库中，这些正是SLAM工程师应从现有文献中真正汲取的精髓。

**用于运行时控制器切换的策略模式** 出现在 rpg_quadrotor_control 的 `ControllerBase` 中，kr_mav_control 基于 pluginlib 的跟踪器注册表中，以及 Agilicious 的 Controller 抽象基类中。C++ 模板变体使用 `MpcController<SCALAR>` 来支持 float/double 量化实验。 **带 YAML 配置的工厂模式**在 Agilicious 中得到了规范化的体现——每个模块（控制器、估计器、采样器、桥接器、管道）均通过名称注册，并从参数文件中实例化，使得 A/B 测试只需更改配置，无需重新编译。 **用于编译时维度的模板元编程**在 Control Toolbox 中被推向了逻辑极限：无处不在的 `template<size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, ...>`，从而在热循环中生成固定大小的 Eigen 矩阵并实现零堆内存占用。这与 Ceres 和 GTSAM 对成本函子所采用的规范如出一辙。

**用于代码生成求解器数组与惯用 C++ 之间零拷贝交互的 `Eigen::Map`** 借鉴了 rpg_mpc 的 `mpc_wrapper` 模式：ACADO 生成列优先的 C 数组；`Eigen::Map<Eigen::Matrix<T, kStateSize, kSamples+1>>` 对其进行封装，使代码库的其余部分在无需拷贝或分配的情况下直接访问 Eigen 矩阵。 所有代码生成求解器的集成都应效仿此做法。**针对仿真器和硬件的桥接/适配器抽象**——Agilicious的Bridge、ROS2的硬件接口、ros2_control的`ControllerInterface`——将算法代码与硬件变体隔离；同一二进制文件可在RotorS、Flightmare以及硅芯片上运行。

与 Pinocchio/OCS2/ros2_control 栈中的机械臂控制相比，其模式几乎完全一致；差异仅归结于问题结构的差异。机械臂栈依赖 URDF + RBDL/Pinocchio 来解析动力学，因为其中包含数百个连杆；而四旋翼栈则手动推导 50–100 行 CasADi 代码，因为仅涉及一个刚体。 机械手MPC采用ros2_control生命周期实现关节级实时控制；四旋翼MPC则采用Pilot/Pipeline/Bridge三元组，因为多速率结构（MPC为100 Hz，INDI为500 Hz，速率环路为2 kHz）是关键约束条件。 两者均采用 HPIPM、CasADi 及增强拉格朗日约束处理——数值工具集是通用的。

### 结论

在李、勒奥克和麦克拉莫克发表论文十五年后，他们提出的几何控制器仍在运行——它存在于PX4中，存在于Agilicious的几何备用方案中，也存在于每一架业余无人机的姿态环路中——因为欧拉角依然会出现奇异点，四元数依然会产生双重覆盖。差分平坦性仍是计算参考值的方法，因为2011年的代数公式对于名义模型而言是精确的。 ACADO演变为acados，但SQP-RTI的核心思想未变。真正的变革在于其上层架构：L1自适应模型吸收有效载荷，高斯过程学习空气阻力，神经网络嵌入HPIPM，走廊MPC在追踪轨迹的同一循环中避开障碍物，而强化学习策略则直接优化单圈时间，甚至超越了发明无人机竞速的人类。

对于一名SLAM工程师而言，**这些数学概念早已自然对应**。李群残差即姿态误差。束调整即轨迹优化。信息滤波平滑即里卡蒂递归。IMU预积分即运动学积分。 你在离线阶段最小化的 `BetweenFactor<Pose3>`，加上前端的 PD 增益，在线上就变成了 SE(3) 控制器。C++ 惯用语——模板化流形、基于生成求解器的 `Eigen::Map`、基于 Factory + Strategy 的 YAML 替代方案、用于传感器的 Bridge 抽象——与 GTSAM、Ceres 以及结构良好的 VIO 前端中已使用的惯用语完全一致。 真正新颖且值得内化的是 Agilicious 所明确提出的架构纪律：当实验性控制器失效时，可信的守护程序必须能够接管控制，因为在 70 公里/小时的速度下没有第二次机会，而软件架构必须将此设为默认机制，而非事后补救。

2026年的技术前沿不在于选择MPC还是RL，而在于决定栈中的哪一层——动力学、成本、控制器、策略、安全监督器——应当通过学习实现，哪一层应当具备可证明的正确性。


