> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 48 章：CppAD 与 CppADCodeGen 预编译流水线

> **本章定位**：从 Ceres Jet 前向 AD（01_数学/30_优化理论）出发，进入工业级 tape-based 自动微分。CppAD 记录计算图，CppADCodeGen 将其编译为高性能 `.so` 动态库。这条流水线是 OCS2 及自定义实时约束/动力学热路径的常用方案——在合适的模型上，可把数值差分量级的导数计算降到微秒级预编译解析导数。Crocoddyl/Aligator 的主流策略略有不同，本章后半部分单独比较。
>
> **前置依赖**：01_数学/30_优化理论（Ceres Jet AD）、02_C++基础与进阶/10_C++语言核心（编译模型 + dlopen）、02_C++基础与进阶/30_软件工程（CMake 工程化）、足式/30_Pinocchio深度精读（Pinocchio 模板 Scalar 切换）

---

## 前置自测

> 📋 **答不出 $\geq 2$ 题 $\to$ 先回前置章节复习**

1. **[01_数学/30_优化理论]** Ceres 的 `Jet<double, N>` 类型如何实现前向模式自动微分？dual number 的加法和乘法规则是什么？
2. **[02_C++基础与进阶/10_C++语言核心]** `dlopen` 和 `dlsym` 分别做什么？如何在运行时加载一个 `.so` 动态库中的函数？
3. **[02_C++基础与进阶/30_软件工程]** CMake 中 `add_custom_command` 和 `add_custom_target` 的区别是什么？如何让一个目标依赖于代码生成步骤？
4. **[足式/30_Pinocchio深度精读]** Pinocchio 的 `ModelTpl<Scalar>` 如何实现标量参数化？为什么同一份 RNEA 代码可以用 `double` 也可以用 `AD<double>`？
5. **[数值方法]** 有限差分求导 $f'(x) \approx \frac{f(x+h)-f(x)}{h}$ 的误差来源有哪两个？为什么 $h$ 不能太大也不能太小？

---

## 本章目标

学完本章，学员应能：

1. **解释** tape-based 反向 AD 的工作原理——从 CppAD 的 `Independent()` → 前向录制 → `ADFun` 构建 → `Jacobian()`/`Hessian()` 反向求值的完整流程，并与 Ceres Jet 前向 AD（01_数学/30_优化理论）做原理和适用场景的对比
2. **独立搭建** CppADCodeGen 的完整预编译流水线——从 C++ 模板函数 → tape 录制 → C 源码生成 → GCC/Clang 编译为 `.so` → 运行时 `dlopen` 加载并调用，理解每个阶段的输入/输出和可能的失败模式
3. **集成** Pinocchio + CppADCodeGen 实现动力学导数的预编译——将 `pinocchio::ModelTpl<CppAD::cg::CG<double>>` 实例化的 RNEA/ABA 导出为独立 `.so`，在实时控制循环中以 ~1.5 $\mu$s 耗时获取机器精度的 Jacobian
4. **阅读并理解** OCS2 `CppAdInterface` 的核心设计——hash 缓存机制（避免重复编译）、`thread_local` 线程安全模式、与 SQP 求解器的集成接口
5. **做出** AD 方案选型决策——根据问题规模、部署环境和开发迭代速度，在数值差分 / CppAD 解释执行 / CppADCodeGen 预编译 / Pinocchio 解析导数四条路线中选择最合适的方案
6. **诊断** AD 流水线的典型故障——从 tape 为空、`Value()` 脱链、条件分支不可微、`.so` 跨平台 `SIGILL` 到编译时间爆炸，能定位症状并对症修复

### 本章知识导航

本章围绕一条主线展开：**如何把"机器人动力学的导数"从一个每次都要现算的运行时负担，变成一段离线编译好、运行时只需查表执行的机器码**。这条主线串起了六个递进的知识块：

```
                    ┌─────────────────────────────────────────────┐
                    │  主问题：MPC 热路径需要微秒级的精确导数        │
                    └───────────────────┬─────────────────────────┘
                                        │
        ┌───────────────────────────────┼───────────────────────────────┐
        ▼                               ▼                               ▼
  [块1 动机]                       [块2 原理]                      [块3 工具]
  §48.1 三方案对比                 §48.2 前向/反向 AD              §48.3 CppAD API
  数值差分为何不行                  dual number + tape             Independent/ADFun
        │                               │                               │
        └───────────────────────────────┼───────────────────────────────┘
                                        ▼
                                  [块4 代码生成]
                                  §48.4 tape → C → .so
                                  CSE / 编译选项 / CMake
                                        │
              ┌─────────────────────────┼─────────────────────────┐
              ▼                         ▼                         ▼
        [块5 实战集成]             [块6 框架对比]             [块7 选型与前沿]
        §48.5 Pinocchio+CG        §48.6 OCS2 源码           §48.8 选型三角
        §48.5 条件分支三解法       §48.7 Crocoddyl/Aligator  §48.9-48.10 可微仿真/JAX
```

**知识点之间的递进关系**：块 1 给出"为什么"（性能死线），块 2 给出"凭什么能做到机器精度"（AD 数学原理），块 3 给出"用什么工具录制计算图"（CppAD tape），块 4 是全章技术核心（把 tape 翻译成机器码），块 5 把工具用到真实动力学上，块 6-7 横向比较工业框架的不同抉择并展望前沿。

**两条推荐阅读路径**：

- **工程实现路径**（想尽快搭出流水线）：§48.1 → §48.3 → §48.4 → §48.5 →（跳过 §48.2 的推导，先跑通再回头补原理）
- **原理理解路径**（想吃透 AD 与 CodeGen 的本质）：§48.1 → §48.2 → §48.4.5（符号 tape 内部机制）→ §48.6（OCS2 封装）→ §48.8（选型）

### 前置知识桥接

本章站在四块前置知识的肩膀上，这里把它们的核心结论各用 2-3 行重新激活，读者不必翻回前章也能跟上：

- **Ceres Jet 前向 AD（01_数学/30_优化理论）**：`Jet<double, N>` 是带 $N$ 个切线方向的 dual number，把 $x$ 替换为 $x + \dot{x}\varepsilon$（$\varepsilon^2 = 0$）后做一遍普通计算，结果的 $\varepsilon$ 系数就是导数。本章把这套"前向扫描"思想推广到反向模式，并指出它在宽输入场景下的低效，从而引出 tape。
- **编译模型与 `dlopen`（02_C++基础与进阶/10_C++语言核心）**：`.so` 是位置无关的 ELF 共享库，`dlopen` 在运行时把它映射进进程地址空间，`dlsym` 按符号名取函数指针。本章的整条 CodeGen 流水线，终点就是生成一个 `.so` 并用 `dlopen` 加载——理解了这一点，"代码生成"就不再神秘。
- **CMake 工程化（02_C++基础与进阶/30_软件工程）**：`add_custom_command` 声明"如何由输入产出输出文件"，`add_custom_target` 声明"一个可被依赖的逻辑目标"。本章用这两者把"运行代码生成器 → 产出 `.so`"接入构建图，使 URDF 一改就自动重新生成。
- **Pinocchio 标量模板（足式/30_Pinocchio深度精读）**：`ModelTpl<Scalar>` 把质量、惯量、关节轴等全部参数化为模板标量 `Scalar`。正因如此，同一份 RNEA 代码用 `double` 实例化就是普通数值计算，用 `AD<CG<double>>` 实例化就变成可生成代码的符号 tape——这是本章 §48.5 "让 RNEA 在 tape 上跑一遍"的根本前提。

### 如果跳过本章会怎样

- **场景一：MPC 控制器周期性丢帧。** 你用中心差分给 18-DOF 四足算动力学 Jacobian，单次 50 $\mu$s，100 次调用占满 5 ms，叠加 QP 求解后总时长冲破 10 ms 死线，控制频率从 100 Hz 跌到 60 Hz，机器人步态开始抖动。不懂 CodeGen，你会以为是 QP 求解器慢，把精力浪费在错误的地方。
- **场景二：接触切换瞬间优化器发散。** 你把 `if (penetration > 0)` 直接写进 AD 函数，tape 只录了一个分支，接触状态一变导数就完全错误，SQP 收到垃圾梯度后发散，机器人腾空腿"卡死"。不懂 tape 的线性化本质和 `CondExpGt`，这个 bug 几乎无法定位。

### 预计阅读时间

| 模式 | 时间 | 适用对象 |
|:---|:---|:---|
| **精读**（含手算练习 + 跑通流水线） | 25-30 小时 | 要独立实现 MPC 动力学预编译模块 |
| **速读**（读正文 + 看代码不动手） | 6-8 小时 | 要理解 OCS2/Crocoddyl 的 AD 设计 |
| **速查**（按需查表 + 故障排查） | 1 小时 | 已会用，只查 API/编译选项/排错 |

---

## 48.1 为什么需要代码生成——MPC 热路径的 1 $\mu$s 死线 ⭐⭐

> **这一节解决什么问题**：腿足 MPC 需要在 10 ms 内完成数百次动力学求值+求导。数值差分太慢，解释执行的 AD 不够快。只有预编译代码生成才能满足实时性要求。

### 问题场景：Go2 四足机器人 MPC ⭐⭐

以 Unitree Go2 四足机器人为例。Go2 每条腿 3 个关节，共 12 自由度（DOF）。浮动基座引入 6 个额外自由度，因此广义速度维度 $n_v = 18$（配置空间维度 $n_q = 19$，因浮基旋转用四元数表示多 1 维，但 MPC 中的动力学 Jacobian 以 $n_v$ 为维度基准）。MPC 的典型配置为：

- **预测时域**（horizon）：$N = 20$ 步
- **控制频率**：100 Hz，即每 10 ms 必须完成一次完整 MPC 求解
- **每步所需计算**：前向动力学 $f(q, v, u)$ 及其关于 $(q, v, u)$ 的 Jacobian $\frac{\partial f}{\partial (q,v,u)}$

SQP（Sequential Quadratic Programming）求解器在每次迭代中需要对所有 $N$ 个时间步调用一次动力学 + Jacobian。典型的 SQP 内循环迭代 3--5 次，因此单次 MPC 求解需要：

$$
N_{\text{calls}} = N \times N_{\text{iter}} = 20 \times 5 = 100 \text{ 次动力学 + Jacobian 调用}
$$

以下是三种求导方案在 10 ms 预算中的时间占比：

| 求导方案 | 单次 Jacobian 耗时 | 100 次总耗时 | 占 10 ms 预算 | 是否可行 |
|:---|:---|:---|:---|:---|
| 数值差分（中心差分） | ~20 $\mu$s | 2.0 ms | **20%** | 勉强可用 |
| CppAD 解释执行 tape | ~10 $\mu$s | 1.0 ms | 10% | 可行 |
| CppADCodeGen 预编译 `.so` | ~1.5 $\mu$s | 0.15 ms | **1.5%** | 理想 |

20% 看起来还行？但这只是 Jacobian 的时间。MPC 还需要：QP 子问题求解（~3 ms）、约束评估（~1 ms）、线搜索（~0.5 ms）。数值差分的 2 ms 立刻变得紧张。

**对于 H1 这类人形机器人（以 19 个驱动关节为例，浮动基座使 $n_v = 25$，四元数配置维度 $n_q = 26$），情况更加严峻**。数值差分的耗时随输入维度**线性增长**，因为中心差分需要 $2n$ 次函数求值来计算 $n$ 维 Jacobian 的每一列：

| 机器人 | 驱动关节 DOF | $n_q$ / $n_v$ | 线性化输入维度 $n \approx 2n_v + n_u$ | 数值差分（中心） | CppADCodeGen |
|:---|:---|:---|:---|:---|:---|
| Go2 四足 | 12 | 19 / 18 | 48 | ~20 $\mu$s | ~1.5 $\mu$s |
| H1 人形 | 19 | 26 / 25 | 69 | ~55 $\mu$s | ~3.0 $\mu$s |
| Atlas 人形 | 28 | 35 / 34 | 96 | ~90 $\mu$s | ~4.5 $\mu$s |

CodeGen 的增长率远低于线性——编译器将完整 Jacobian 展开为一段连续指令流，分支预测命中率极高，而数值差分每多一个维度就多两次完整的函数求值调用。这就是代码生成的威力所在。

### 三种求导方案对比 ⭐⭐

| 维度 | 数值差分 | CppAD 解释执行 | CppADCodeGen 预编译 |
|:---|:---|:---|:---|
| **原理** | $\frac{f(x+h)-f(x-h)}{2h}$ | 录制 tape，运行时逐条解释执行 | tape $\to$ C 源码 $\to$ `.so` 动态库 |
| **单次耗时** | $O(n) \times t_f$ | $O(1) \times t_{\text{tape}}$ | $O(1) \times t_{\text{native}}$ |
| **精度** | $O(h^2)$ 截断 + $O(\epsilon/h)$ 舍入 | 机器精度（$\sim 10^{-16}$） | 机器精度（$\sim 10^{-16}$） |
| **实现复杂度** | 低（几行代码） | 中（需要模板化函数） | 高（完整编译流水线） |
| **部署依赖** | 无 | 仅 CppAD header-only 库 | GCC/Clang + `dlopen` |
| **适用场景** | 原型验证、低维问题 | 中等规模、快速迭代 | 生产部署、实时 MPC |
| **代码改动后** | 无需额外步骤 | 无需额外步骤 | 需要重新生成 + 编译 |

### 为什么数值差分在 MPC 中不可行 ⭐⭐

数值差分看似简单，实则在 MPC 热路径中有两个致命缺陷。

如果不用自动微分而坚持数值差分，就像试图用尺子量头发丝的直径——工具的精度限制决定了结果的上限，无论你怎么量都无法突破浮点舍入误差的天花板。

**缺陷一：精度与效率的根本矛盾。** 中心差分的总误差由两项组成：

$$
f'(x) = \frac{f(x+h) - f(x-h)}{2h} + \underbrace{\frac{h^2}{6}f'''(\xi)}_{\text{截断误差 (truncation)}} + \underbrace{\frac{\epsilon_{\text{mach}}}{h} \cdot |f(x)|}_{\text{舍入误差 (round-off)}}
$$

其中 $\epsilon_{\text{mach}} \approx 2.2 \times 10^{-16}$（IEEE 754 double 精度）。两项误差的行为相反：

- $h$ 增大 $\to$ 截断误差增大（$O(h^2)$），舍入误差减小（$O(1/h)$）
- $h$ 减小 $\to$ 截断误差减小，舍入误差增大

对总误差 $E(h) \sim c_1 h^2 + c_2 / h$ 取极小值，令 $\frac{dE}{dh} = 0$，得最优步长：

$$
h^* = \left(\frac{c_2}{2 c_1}\right)^{1/3} \approx \left(\frac{3\epsilon_{\text{mach}} |f(x)|}{|f'''(\xi)|}\right)^{1/3} \sim 10^{-5}
$$

此时最优总误差约为 $O(\epsilon_{\text{mach}}^{2/3}) \approx 10^{-11}$。看起来足够？问题在于 $f'''(\xi)$ 和 $|f(x)|$ 对于动力学方程来说并不温和。

**缺陷二：对动力学方程的条件数极度敏感。** 机器人动力学方程 $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$ 中，质量矩阵 $M(q)$ 在接近奇异位型（如膝关节完全伸直、手臂完全展开）时条件数急剧恶化。此时 $f'''(\xi)$ 可以非常大，最优步长 $h^*$ 变得极小。同时 $f(x+h)$ 和 $f(x-h)$ 的有效数字高度重合，相减后有效数字急剧减少——这就是**灾难性相消**（catastrophic cancellation）。

更糟糕的是，即使选对了步长，**中心差分求 $n$ 维 Jacobian 需要 $2n$ 次函数求值**：

$$
J_{ij} = \frac{\partial f_i}{\partial x_j} \approx \frac{f_i(x + h \cdot e_j) - f_i(x - h \cdot e_j)}{2h}, \quad j = 1, \ldots, n
$$

每个偏导数方向 $e_j$ 需要两次完整的动力学求值。对 Go2 的 $n = 48$ 维输入，这就是 96 次完整的 RNEA 调用。

> **常见陷阱**
>
> 数值差分对条件数敏感的动力学方程会产生**灾难性相消误差**。在实际腿足 MPC 中，当膝关节接近完全伸直（$\theta_{\text{knee}} \to 0$ 或 $\pi$）时，逆动力学 Jacobian 的数值差分可以产生 $10^{-3}$ 量级的相对误差——这不是理论上的担忧，而是实际调试中反复出现的问题。自动微分完全消除了这一问题，因为它计算的是解析导数，精度仅受机器精度限制。

### 编译期 vs 运行期：成本搬家的会计学 ⭐⭐⭐

代码生成方案的核心思想，用一句话概括就是：**把成本从"每帧都付的运行期成本"搬到"只付一次的编译期成本"**。要理解这笔账划不划算，得把两类成本分开记账。

设一个 MPC 控制器在部署后总共要运行 $T$ 帧（一台机器人服役期内轻松达到 $10^8$ 帧以上），每帧需要 $K$ 次导数求值。定义：

- $c_{\text{rt}}$：单次运行期导数求值的耗时（解释执行或数值差分）
- $c_{\text{native}}$：单次原生（CodeGen）导数求值的耗时，$c_{\text{native}} \ll c_{\text{rt}}$
- $C_{\text{gen}}$：一次性的代码生成 + 编译成本（秒到分钟级）

两种方案的**总成本**（生命周期视角）为：

$$
\text{Cost}_{\text{解释}} = T \cdot K \cdot c_{\text{rt}}, \qquad
\text{Cost}_{\text{CodeGen}} = C_{\text{gen}} + T \cdot K \cdot c_{\text{native}}
$$

CodeGen 划算的条件是 $\text{Cost}_{\text{CodeGen}} < \text{Cost}_{\text{解释}}$，整理得一个**盈亏平衡帧数**：

$$
T^* = \frac{C_{\text{gen}}}{K \,(c_{\text{rt}} - c_{\text{native}})}
$$

代入 Go2 的典型数字（$C_{\text{gen}} \approx 60$ s $= 6\times10^7\,\mu$s，$K \approx 100$，$c_{\text{rt}} \approx 10\,\mu$s，$c_{\text{native}} \approx 1.5\,\mu$s）：

$$
T^* = \frac{6\times10^7}{100 \times (10 - 1.5)} \approx 7\times10^4 \text{ 帧}
$$

也就是说，**仅仅运行约 12 分钟（100 Hz 下 $7\times10^4$ 帧）后，CodeGen 的一次性编译成本就被摊平了**——此后每一帧都是纯赚。对于服役数年的机器人，这笔账的答案毫无悬念。

> **本质洞察**：代码生成的本质不是"让计算变快"，而是"把同一份计算的成本支付时机前移"。这与编译型语言 vs 解释型语言的取舍是同一个母题——AOT（提前编译）用一次编译成本换来无数次的执行加速，JIT/解释器用即时启动换来每次的解释开销。CppADCodeGen 就是给"求导"这件事做了一次 AOT 编译。

> **对比性思维：什么时候 CodeGen 反而不划算？** 把会计学反过来看 $T^*$ 公式就知道——当 $T$（运行帧数）很小、或 $c_{\text{rt}} - c_{\text{native}}$（速度差）很小时，编译成本摊不平。具体两种场景：(1) **研究早期频繁改模型**——每改一次 URDF 就要重付一次 $C_{\text{gen}}$，而每版只跑几千帧验证就丢弃，$T < T^*$，此时 CppAD 解释执行更优；(2) **模型已有快速解析导数**——$c_{\text{rt}}$（解析）已经很小，CodeGen 省不下多少，不值得引入编译流水线的复杂度。这正是 §48.8 选型决策树的会计学依据。

### 练习 ⭐⭐

1. **[估算题]** H1 人形机器人 19 个驱动关节，浮动基座下 $n_v = 25$，若采用最小坐标状态近似 $n_x = 2n_v = 50$、控制维度 $n_u = 19$，MPC horizon $N=30$，SQP 迭代 4 次。假设 Jacobian 输入维度为 $n = 69$，单次动力学求值耗时 0.3 $\mu$s。分别估算：(a) 数值差分（中心）求一次 Jacobian 的耗时；(b) 单次 MPC 求解中 Jacobian 计算的总耗时；(c) 该总耗时占 10 ms 预算的百分比。对比 CppADCodeGen 方案（假设单次 3 $\mu$s），讨论哪种方案可行。

2. **[编程题]** 用有限差分（前向差分、中心差分）和 CppAD 分别计算 $f(x) = \sin(x^2)$ 在 $x = 1.0$ 处的导数。对有限差分，让步长 $h$ 从 $10^{-1}$ 到 $10^{-16}$ 扫描（每次乘以 $10^{-1}$），计算相对误差 $|f'_{\text{num}} - f'_{\text{exact}}| / |f'_{\text{exact}}|$ 并绘制误差-步长双对数曲线。验证：(a) 前向差分误差呈 V 型，左侧斜率 $+1$，右侧斜率 $-1$；(b) 中心差分的最优精度优于前向差分约 5 个数量级；(c) CppAD 结果与解析值 $f'(1) = 2\cos(1) \approx 1.0806046$ 在机器精度（$\sim 10^{-15}$）内一致。

3. **[会计题]** 用本节的盈亏平衡公式 $T^* = C_{\text{gen}} / \big(K (c_{\text{rt}} - c_{\text{native}})\big)$ 分析两种部署场景。场景甲：H1 人形，$C_{\text{gen}} = 180$ s，$K = 120$，$c_{\text{rt}} = 20\,\mu$s（解释执行），$c_{\text{native}} = 4.5\,\mu$s。场景乙：同一机器人但用数值差分作为基线，$c_{\text{rt}} = 156\,\mu$s。(a) 分别算出 $T^*$，并换算成 100 Hz 下的"摊平时间"（分钟）；(b) 解释为什么相对数值差分基线，CodeGen 摊平得更快；(c) 若研究阶段每改一次模型只跑 5 分钟就丢弃，CodeGen 在哪个场景下仍然不划算？给出定量判据。

---

## 48.2 自动微分基础——从 Dual Number 到 Tape ⭐⭐

> **这一节解决什么问题**：回顾 01_数学/30_优化理论 中的前向模式 AD（Ceres Jet），引入反向模式和 tape-based AD 的核心概念，为 CppAD 的使用打下数学基础。

### 前向模式 AD 回顾：Dual Number ⭐⭐

01_数学/30_优化理论 已经介绍了 Ceres 的 `Jet<double, N>` 类型。这里从数学根基出发，重新推导 dual number 的工作原理，揭示其与链式法则的本质联系。

**Dual number 的定义。** 在实数域 $\mathbb{R}$ 上扩展一个无穷小元 $\varepsilon$，满足：

$$
\varepsilon^2 = 0 \quad \text{但} \quad \varepsilon \neq 0
$$

一个 dual number 写作 $\tilde{a} = a + b\varepsilon$，其中 $a \in \mathbb{R}$ 称为**原值**（primal part），$b \in \mathbb{R}$ 称为**切线**（tangent part / dual part）。

**算术规则**直接由 $\varepsilon^2 = 0$ 推出——不需要任何额外公理：

$$
\begin{aligned}
\text{加法：} \quad & (a_1 + b_1\varepsilon) + (a_2 + b_2\varepsilon) = (a_1 + a_2) + (b_1 + b_2)\varepsilon \\[4pt]
\text{乘法：} \quad & (a_1 + b_1\varepsilon)(a_2 + b_2\varepsilon) = a_1 a_2 + (a_1 b_2 + a_2 b_1)\varepsilon + \underbrace{b_1 b_2 \varepsilon^2}_{=\,0} \\[4pt]
\text{除法：} \quad & \frac{a_1 + b_1\varepsilon}{a_2 + b_2\varepsilon} = \frac{a_1}{a_2} + \frac{b_1 a_2 - a_1 b_2}{a_2^2}\,\varepsilon
\end{aligned}
$$

观察乘法规则中的切线部分 $a_1 b_2 + a_2 b_1$——这恰好是**乘积法则**（product rule）$(fg)' = fg' + f'g$。这不是巧合。对任意可解析函数 $f$，将其在 dual number $a + b\varepsilon$ 处做 Taylor 展开：

$$
f(a + b\varepsilon) = f(a) + f'(a) \cdot b\varepsilon + \frac{f''(a)}{2}(b\varepsilon)^2 + \cdots = f(a) + f'(a)\, b\, \varepsilon
$$

高阶项因 $\varepsilon^2 = 0$ 而精确消失。因此，**dual number 算术自动传播一阶导数，且没有截断误差**。

**从 dual number 到多方向。** Ceres 的 `Jet<double, N>` 是带 $N$ 个独立切线方向的 dual number：

$$
\tilde{a} = a + \sum_{i=1}^{N} b_i \varepsilon_i, \quad \varepsilon_i \varepsilon_j = 0 \;\;\forall\; i, j
$$

其中 $b_i = \frac{\partial a}{\partial x_i}$。一次前向传播同时计算 $a$ 关于所有 $N$ 个输入的偏导数。

**前向模式的计算代价。** 对 $f: \mathbb{R}^n \to \mathbb{R}^m$，一次前向 seed（选取一个切线方向 $\dot{x} = e_j$）计算 Jacobian 的第 $j$ 列。完整 Jacobian 需要 $n$ 次传播。如果用 `Jet<double, N>`，则一次传播内部做 $N$ 个切线方向的并行运算，总计算量为 $O(n \cdot C_f)$，其中 $C_f$ 是函数本身的 FLOP 数。

### 反向模式 AD（反向传播） ⭐⭐⭐

前向模式对每个输入方向做一次传播——如果输入维度 $n$ 很大而输出维度 $m$ 很小，这就浪费了。反向模式从**输出端**出发，一次传播就得到**所有输入方向**的导数信息。

**数学定义。** 给定 $f: \mathbb{R}^n \to \mathbb{R}^m$ 和一个 adjoint（伴随）向量 $\bar{y} \in \mathbb{R}^m$，反向模式计算的是向量-Jacobian 积（VJP）：

$$
\bar{x} = J^T \bar{y} = \left(\frac{\partial f}{\partial x}\right)^T \bar{y} \in \mathbb{R}^n
$$

一次反向传播得到 $\bar{y}^T J$——这是 Jacobian 的一个**行向量**。特别地，当 $m = 1$（标量输出），令 $\bar{y} = 1$，一次反向传播就给出完整梯度 $\nabla_x f \in \mathbb{R}^n$。

**前向 vs 反向模式复杂度对比：**

| | 前向模式 | 反向模式 |
|:---|:---|:---|
| **一次传播计算量** | $J \cdot \dot{x}$（Jacobian 的一**列**） | $\bar{y}^T \cdot J$（Jacobian 的一**行**） |
| **完整 Jacobian 所需传播次数** | $n$ 次 | $m$ 次 |
| **总代价** | $O(n \cdot C_f)$ | $O(m \cdot C_f)$ |
| **额外内存** | $O(n)$（仅需当前切线向量） | $O(C_f)$（需存储完整计算轨迹） |
| **最适用场景** | $m \gg n$（宽输出） | $n \gg m$（宽输入） |

在腿足 MPC 场景下，不同子问题适合不同模式：

- **代价函数** $L: \mathbb{R}^n \to \mathbb{R}$（$m=1$）：反向模式一次传播即得完整梯度，**反向胜出**
- **动力学** $f: \mathbb{R}^{n_x + n_u} \to \mathbb{R}^{n_x}$（$m \approx n/2$）：前向和反向代价相当
- **接触 Jacobian** $J_c: \mathbb{R}^{n_q} \to \mathbb{R}^{3k}$（$k$ 个接触点）：取决于 $n_q$ 和 $3k$ 的大小关系

这与深度学习中的**反向传播**（backpropagation）本质相同。神经网络的 loss 是标量（$m = 1$），参数维度 $n$ 可达数十亿，因此反向模式是唯一可行的选择。PyTorch 的 `loss.backward()` 就是在执行一次反向模式 AD。

**把抽象代价落到 Go2 的具体数字。** 为了让"选哪个模式"从口号变成可计算的决策，拿 Go2 的动力学 Jacobian 算一笔账。其离散动力学 $f$ 把状态+控制映射到下一状态，输入维度 $n = n_x + n_u = 36 + 12 = 48$，输出维度 $m = n_x = 36$。两种模式的传播次数：

$$
N_{\text{fwd}} = n = 48, \qquad N_{\text{rev}} = m = 36, \qquad \frac{N_{\text{fwd}}}{N_{\text{rev}}} = \frac{48}{36} \approx 1.33
$$

反向模式快约 1.33 倍——但**没有数量级差距**，因为这个 Jacobian 接近方形（$m \approx n$）。这解释了为什么前面说动力学"前向反向代价相当"：方形 Jacobian 是模式选择的"中性区"。对比之下，把同样的账套到代价函数 $L: \mathbb{R}^{48} \to \mathbb{R}$（$m=1$）：

$$
N_{\text{fwd}} = 48, \qquad N_{\text{rev}} = 1, \qquad \frac{N_{\text{fwd}}}{N_{\text{rev}}} = 48
$$

反向模式快 **48 倍**——这才是数量级差距。规律一目了然：**Jacobian 越"扁"（$n \gg m$），反向模式的优势越压倒性**。这就是为什么标量代价的梯度永远用反向，而方形动力学 Jacobian 用哪个都行（实践中常让 `func.Jacobian()` 自动按 $\min(n,m)$ 选）。

> **本质洞察**：前向模式的代价钉在输入维度 $n$ 上，反向模式的代价钉在输出维度 $m$ 上——这不是两个并列的算法，而是同一条链式法则从两端展开的对偶。选模式的全部智慧就压缩成一个比较：$n$ 和 $m$ 谁小，就从谁那端扫。所有"标量损失用反向、宽输出用前向"的经验法则，都是这个唯一判据的特例。

### Tape-Based AD：CppAD 的核心机制 ⭐⭐⭐

CppAD 使用 **tape**（记录带）机制实现自动微分。理解 tape 是掌握 CppAD 和 CppADCodeGen 的前提。

**Tape 的本质。** 在函数求值过程中，CppAD 将每一步基本运算（加、减、乘、除、sin、exp 等）连同操作数索引记录到一个顺序数组中。这个数组就是 tape——计算图的线性化存储。

**CppAD 的类型魔法。** `CppAD::AD<double>` 重载了所有算术运算符（`+`, `-`, `*`, `/`）和数学函数（`sin`, `cos`, `exp`, `log` 等）。当你用 `AD<double>` 类型的变量做计算时，每个运算**同时**执行数值计算和 tape 写入：

```
tape 记录格式（概念性）:
[操作类型] [输入槽位1] [输入槽位2] [输出槽位] [常数值]

示例：y = x1 * sin(x2) 的 tape 内容
  Op#0: INPUT   →  slot[0] = x1        // 声明自变量 x1
  Op#1: INPUT   →  slot[1] = x2        // 声明自变量 x2
  Op#2: SIN     slot[1]   →  slot[2]   // slot[2] = sin(x2)
  Op#3: MUL     slot[0] slot[2] → slot[3]  // slot[3] = x1 * sin(x2)
  Op#4: OUTPUT  slot[3]                 // 标记 y = slot[3]
```

**Tape 的生命周期** 分三个阶段：

```
CppAD::Independent(ax)   ←── 开始录制：此后所有 AD<double> 运算写入 tape
       │
       ▼
  ay = f(ax)              ←── 计算过程：运算被执行 AND 被记录
       │
       ▼
CppAD::ADFun<double> func(ax, ay)  ←── 结束录制：tape 被"密封"为 ADFun 对象
       │
       ▼
  func.Forward(...)       ←── 使用阶段：可以反复求值/求导，不再录制
  func.Reverse(...)
  func.Jacobian(...)
```

**`ADFun<double>` 对象** 是 CppAD 的核心数据结构。它持有密封后的 tape，并提供前向/反向模式的求值接口。关键理解：tape 记录的是计算**结构**（哪些运算以什么顺序连接），而不是具体数值。因此同一个 `ADFun` 可以在**任意新的输入点** $x$ 上求值和求导——你只在创建 tape 时需要提供一个初始值，后续求值时这个初始值不起作用。

**内存特征。** tape 大小正比于运算数量，每条记录约 8--16 字节。对于机器人动力学这种"宽而浅"的计算图，tape 通常在 KB 到 MB 量级——完全可接受。

### 深入理解：录制点为什么不等于求值点 ⭐⭐⭐

初学者读到"录制时要给 `ax` 一个初始值"，几乎都会产生一个疑问：**那我录制时填的 $x_0$，是不是就是这个函数"被锁定"的工作点？换个点求导还准吗？** 这个疑问极其常见，也极其重要——答错它会导致你在错误的地方调试半天。

先给结论：**对于不含分支的纯解析函数，录制点完全不影响后续求值点的正确性。** 你可以在 $x_0 = (1, 2)$ 处录制，却在 $x = (100, -7)$ 处求值和求导，结果照样正确。原因在前面已点明——tape 存的是**结构**而非数值。

为什么会这样？换个角度类比就清楚了：tape 之于函数，**正如乐谱之于演奏**。乐谱记录的是"先按哪个键、再按哪个键"的结构，它不绑定任何一次具体演奏的音量和速度。录制 tape 时填的初始值 $x_0$，作用仅仅是"让运算符重载有一组具体的数能跑起来，从而观察到运算的连接关系"——就像作曲家要先在脑中哼一遍才能写下乐谱，但写下的乐谱并不锁定那次哼唱。求值阶段 `Forward(0, x)` 把新的 $x$ 灌进同一张乐谱重新"演奏"一遍，得到新点的函数值；求导阶段沿同一张乐谱传播切线或伴随，得到新点的导数。

> **这个类比的边界**：乐谱类比在"结构与数值分离"这一点上精确成立，但有一处不像——乐谱允许演奏者即兴加花，而 tape 是**绝对刚性**的：录制时是什么运算序列，求值时一字不差地重放，不允许任何运行时的结构变化。这正是下面分支问题的根源。

**反事实推理：什么情况下录制点会"污染"结果？** 唯一的例外是函数里含有**依赖 AD 值的分支**。考虑：

$$
f(x) = \begin{cases} x^2 & x \geq 0 \\ -x & x < 0 \end{cases}
$$

如果你用 `if (Value(ax) >= 0)` 来写它，并在 $x_0 = 1$ 处录制——录制时 $x_0 \geq 0$ 为真，tape 里**只有** $x^2$ 这一支。此后无论你在哪个点求值，tape 永远算 $x^2$ 和导数 $2x$。在 $x = -3$ 处，正确答案是 $f = 3, f' = -1$，但 tape 给你 $f = 9, f' = -6$——**全错**。这时录制点 $x_0 = 1$ 的选择确实"污染"了结果，因为它在录制瞬间替你做了一个本该在运行时做的决定。

把这两种情况并列，规律就清晰了：

| 函数类型 | 录制点是否影响后续求值 | 根本原因 |
|:---|:---|:---|
| 纯解析（无分支，或分支全用 `CondExp`） | **不影响** | tape 结构对所有输入点都正确 |
| 含 `if (AD 值)` 的硬分支 | **严重影响** | tape 只固化了录制点选中的那一支 |

> **本质洞察**：tape 的"录制点无关性"不是一个需要小心维护的脆弱性质，而是 AD 能够工作的**前提**。整个自动微分的威力，正建立在"用一组具体数跑一遍，就能提取出对所有数都成立的导数结构"之上。一旦你用硬分支让结构本身依赖于具体数值，就破坏了这个前提，AD 也就失去了它的根基。这解释了为什么 §48.5 把条件分支列为"实际使用中最容易踩的坑"——它不是一个普通 bug，而是动摇了 AD 工作假设的根本性错误。

工程上由此得到一条朴素但极其有用的自检：**如果你怀疑某个 AD 函数的导数有问题，先问"它的 tape 结构是否依赖于录制点的具体数值？"** 把录制点换成几个差异很大的值各录一次，如果生成的导数随录制点变化，那一定有硬分支藏在里面。

### 计算图视角 ⭐⭐

为了建立直觉，考虑具体例子 $f(x_1, x_2) = x_1 \cdot \sin(x_2) + x_2^2$。

**计算图**（有向无环图，DAG）表示为：

```
  x1          x2
  │            │ \
  │          [sin] [sqr]
  │            │     │
  └──[mul]─────┘     │
       │              │
      [add]───────────┘
       │
       y
```

定义中间变量：$v_1 = \sin(x_2)$，$v_2 = x_2^2$，$v_3 = x_1 \cdot v_1$，$y = v_3 + v_2$。

**前向模式示例。** 选择切线方向 $\dot{x}_1 = 1, \dot{x}_2 = 0$（即求 $\partial y / \partial x_1$），沿图的正方向传播：

$$
\begin{aligned}
\dot{v}_1 &= \cos(x_2) \cdot \dot{x}_2 = 0 \\
\dot{v}_2 &= 2x_2 \cdot \dot{x}_2 = 0 \\
\dot{v}_3 &= \dot{x}_1 \cdot v_1 + x_1 \cdot \dot{v}_1 = 1 \cdot \sin(x_2) + 0 = \sin(x_2) \\
\dot{y} &= \dot{v}_3 + \dot{v}_2 = \sin(x_2)
\end{aligned}
$$

结论：$\frac{\partial y}{\partial x_1} = \sin(x_2)$。要求 $\frac{\partial y}{\partial x_2}$，需要再做一次前向传播（$\dot{x}_1 = 0, \dot{x}_2 = 1$）。

**反向模式示例。** 从输出的 adjoint $\bar{y} = 1$ 出发，沿图的**反方向**传播：

$$
\begin{aligned}
&\bar{y} = 1 \\
&\bar{v}_3 = \bar{y} \cdot \frac{\partial y}{\partial v_3} = 1, \quad \bar{v}_2 = \bar{y} \cdot \frac{\partial y}{\partial v_2} = 1 \\
&\bar{x}_1 = \bar{v}_3 \cdot \frac{\partial v_3}{\partial x_1} = 1 \cdot v_1 = \sin(x_2) \\
&\bar{v}_1 = \bar{v}_3 \cdot \frac{\partial v_3}{\partial v_1} = 1 \cdot x_1 = x_1 \\
&\bar{x}_2 = \bar{v}_1 \cdot \frac{\partial v_1}{\partial x_2} + \bar{v}_2 \cdot \frac{\partial v_2}{\partial x_2} = x_1 \cos(x_2) + 2x_2
\end{aligned}
$$

**一次**反向传播就同时得到了 $\frac{\partial y}{\partial x_1} = \sin(x_2)$ 和 $\frac{\partial y}{\partial x_2} = x_1\cos(x_2) + 2x_2$。这就是反向模式对标量输出函数的效率优势。

CppAD 的 tape 就是这个 DAG 的线性化存储。前向模式对应 `func.Forward()` 接口，反向模式对应 `func.Reverse()` 接口。

> **常见陷阱**
>
> CppAD 的 tape **不能包含依赖于 AD 变量值的条件分支**。例如：
> ```cpp
> AD<double> y;
> if (ax[0] > 0)    // 错误! ax[0] 是 AD 类型，> 运算符被 CppAD 截取为 bool
>     y = ax[0];    // tape 只录制了被选中的这个分支
> else
>     y = -ax[0];   // 这个分支被完全忽略
> ```
> 问题在于 `if` 在录制时根据**初始数值**选择了一个分支。tape 中只包含被选中的分支。当你在不同点（如 $x_0 < 0$）求值时，tape 不会切换分支——函数值和导数在分支边界附近完全错误。
>
> **正确做法**：使用 CppAD 的条件表达式：
> ```cpp
> // CppAD::CondExpGt(left, right, if_true, if_false)
> // 语义：if (left > right) return if_true; else return if_false;
> AD<double> y = CppAD::CondExpGt(ax[0], AD<double>(0.0), ax[0], -ax[0]);
> ```
> `CondExpGt` 在 tape 上记录**两个分支**，求值时根据当前 $x_0$ 的值动态选择正确的分支及其导数。类似的还有 `CondExpLt`、`CondExpEq`、`CondExpLe`、`CondExpGe`。

### 练习 ⭐⭐

1. **[手算题]** 对 $f(x_1, x_2) = x_1 \cdot \exp(x_2) + \log(x_1)$，完成以下步骤：(a) 画出计算图（标注每个中间变量）；(b) 用前向模式分别设 seed $(\dot{x}_1, \dot{x}_2) = (1, 0)$ 和 $(0, 1)$，手算 $\frac{\partial f}{\partial x_1}$ 和 $\frac{\partial f}{\partial x_2}$；(c) 用反向模式设 seed $\bar{y} = 1$，一次传播同时得到两个偏导数；(d) 验证两种模式结果相同；(e) 在 $(x_1, x_2) = (2, 1)$ 处代入数值验证：$\frac{\partial f}{\partial x_1} = e + \frac{1}{2} \approx 3.218$，$\frac{\partial f}{\partial x_2} = 2e \approx 5.436$。

2. **[概念题]** 为什么深度学习框架（PyTorch / JAX）普遍使用反向模式 AD 而不是前向模式？在 MPC 场景下，动力学约束 $x_{k+1} = f(x_k, u_k)$ 的 Jacobian $\frac{\partial f}{\partial (x_k, u_k)} \in \mathbb{R}^{n_x \times (n_x + n_u)}$ 应该用前向还是反向模式？提示：对于 Go2，$n_x = 36$，$n_u = 12$，因此输入维度 $n_x + n_u = 48$，输出维度 $n_x = 36$。前向模式需要 48 次传播，反向模式需要 36 次。哪种更快？如果用稀疏性检测呢？

3. **[判据题]** 用"谁小从谁那端扫"这条唯一判据，判断以下四个函数各应选前向还是反向模式，并算出加速比 $\max(n,m)/\min(n,m)$：(a) H1 接触 Jacobian $J_c: \mathbb{R}^{25} \to \mathbb{R}^{12}$（4 个接触点 $\times$ 3 维）；(b) 单步代价 $L: \mathbb{R}^{67} \to \mathbb{R}$；(c) 一个把 6 维位姿映射到 100 维特征的观测函数 $h: \mathbb{R}^6 \to \mathbb{R}^{100}$；(d) 近方形的整步动力学 $f: \mathbb{R}^{50}\to\mathbb{R}^{50}$。指出哪个属于"模式选择中性区"，为什么在中性区图着色（稀疏性）比模式选择更能决定实际性能。

---

## 48.3 CppAD 核心 API 精读 ⭐⭐

> **这一节解决什么问题**：掌握 CppAD 的核心 API，能独立使用 CppAD 对任意 C++ 函数进行自动微分。

### `AD<double>` 类型与 `Independent` 声明 ⭐⭐

CppAD 的使用遵循一个固定三步模式：声明自变量 $\to$ 录制计算 $\to$ 创建 `ADFun`。以下是最基本的完整示例：

```cpp
#include <cppad/cppad.hpp>
#include <iostream>

using CppAD::AD;
using ADVector = CppAD::vector<AD<double>>;

int main() {
    // ========== 第一步：声明自变量并开始录制 ==========
    size_t n = 2;  // 输入维度
    ADVector ax(n);
    ax[0] = 1.0;   // 初始值——仅用于确定计算图分支走向
    ax[1] = 2.0;   // 后续求导时可以在任意新的点求值
    CppAD::Independent(ax);  // 开始录制 tape！

    // ========== 第二步：计算因变量 ==========
    size_t m = 1;  // 输出维度
    ADVector ay(m);
    ay[0] = ax[0] * CppAD::sin(ax[1]) + ax[1] * ax[1];
    // 注意：必须使用 CppAD::sin 而非 std::sin
    // std::sin 通常没有适合 AD<double> 的重载：轻则编译失败，
    // 重则在显式取 Value/转换后脱离 tape，导致导数错误。

    // ========== 第三步：结束录制，创建 ADFun 对象 ==========
    CppAD::ADFun<double> func(ax, ay);
    // 此刻 tape 被密封。ax 和 ay 恢复为普通变量

    // ========== 使用 ADFun：求值和求导 ==========
    std::vector<double> x = {1.0, 2.0};

    // 函数值
    std::vector<double> y = func.Forward(0, x);
    std::cout << "f(1,2) = " << y[0] << std::endl;
    // 输出: f(1,2) = 1.0*sin(2.0) + 4.0 ≈ 4.909

    // 完整 Jacobian（自动选择最优模式）
    std::vector<double> jac = func.Jacobian(x);
    std::cout << "df/dx1 = " << jac[0] << std::endl;  // sin(2) ≈ 0.909
    std::cout << "df/dx2 = " << jac[1] << std::endl;  // cos(2)+4 ≈ 3.584

    return 0;
}
```

几个必须注意的细节：

- `Independent(ax)` 标记 `ax` 为自变量并开启全局 tape。在此之后、`ADFun` 构造之前，所有涉及 `ax` 元素的运算都被记录。
- **必须**使用 `CppAD::sin`、`CppAD::exp` 等函数。`std::` 版本通常不会绑定到 CppAD 的 AD 重载；如果为了通过编译而显式 `Value()` 或转换成 `double`，该运算会脱离 tape，导数自然错误。
- `ADFun` 构造函数的两个参数标记 tape 的"入口"（自变量）和"出口"（因变量）。

### Forward 模式：切线方向 ⭐⭐⭐

`ADFun::Forward(p, xp)` 计算第 $p$ 阶 Taylor 系数。最常用的是零阶（函数求值）和一阶（方向导数）：

```cpp
// -------- 零阶前向 = 在新点处求函数值 --------
std::vector<double> x = {3.0, 1.0};
std::vector<double> y0 = func.Forward(0, x);
// y0[0] = 3.0 * sin(1.0) + 1.0^2 ≈ 3.524
// 内部状态更新为 x = (3.0, 1.0)

// -------- 一阶前向 = 方向导数 J * dx --------
// 必须在 Forward(0, x) 之后调用
std::vector<double> dx1 = {1.0, 0.0};  // seed: 求 df/dx1
std::vector<double> dy1 = func.Forward(1, dx1);
// dy1[0] = df/dx1|_{(3,1)} = sin(1.0) ≈ 0.841

std::vector<double> dx2 = {0.0, 1.0};  // seed: 求 df/dx2
std::vector<double> dy2 = func.Forward(1, dx2);
// dy2[0] = df/dx2|_{(3,1)} = 3.0*cos(1.0) + 2*1.0 ≈ 3.621

// 用两次 Forward(1, ...) 得到了完整 Jacobian [0.841, 3.621]
// 这就是前向模式的基本循环：对 n 维输入需要 n 次
```

注意 `Forward(1, dx)` **必须**在 `Forward(0, x)` 之后调用——一阶 Taylor 系数依赖于零阶状态。如果忘记先调用 `Forward(0, x)` 就直接调用 `Forward(1, dx)`，结果是未定义的。

### Reverse 模式：伴随方向 ⭐⭐⭐

`ADFun::Reverse(q, w)` 计算 adjoint 向量。对于 $q=1$，它执行 $w^T J$ 的计算：

```cpp
// 先做零阶前向设定求值点
func.Forward(0, x);  // x = {3.0, 1.0}

// -------- 反向模式：w^T * J --------
// 对标量输出，w = {1.0} 给出完整梯度 ∇f
std::vector<double> w = {1.0};
std::vector<double> grad = func.Reverse(1, w);
// grad[0] = df/dx1 = sin(1.0) ≈ 0.841
// grad[1] = df/dx2 = 3.0*cos(1.0) + 2.0 ≈ 3.621

// 一次 Reverse 调用就得到了所有偏导数！
// 对标量输出函数，这比前向模式高效 n 倍（n 是输入维度）
```

### Jacobian 与 Hessian ⭐⭐

CppAD 提供了高层接口，内部自动选择前向或反向模式并管理 seed 循环：

```cpp
// ========== 稠密 Jacobian ==========
std::vector<double> jac = func.Jacobian(x);
// jac 是 m*n 的一维数组（行主序）
// jac[i*n + j] = ∂f_i / ∂x_j

// ========== 稠密 Hessian（仅对 w^T·f 的 Hessian） ==========
std::vector<double> w = {1.0};  // 权重向量
std::vector<double> hes = func.Hessian(x, w);
// hes 是 n*n 的一维数组
// hes[i*n + j] = ∂²(w^T f) / ∂x_i ∂x_j
```

**稀疏 Jacobian——实际机器人应用的性能关键：**

```cpp
// ========== 第一步：稀疏性分析（只需做一次） ==========
size_t n = func.Domain();  // 输入维度
size_t m = func.Range();   // 输出维度

// 用前向模式检测 Jacobian 的稀疏模式
// 创建 n×n 的单位稀疏矩阵作为 seed
CppAD::sparse_rc<std::vector<size_t>> identity;
identity.resize(n, n, n);  // (nr, nc, nnz)
for (size_t k = 0; k < n; k++)
    identity.set(k, k, k);  // 设置第 k 个非零元: (row=k, col=k)

bool transpose  = false;
bool dependency = false;
CppAD::sparse_rc<std::vector<size_t>> jac_pattern;
func.for_jac_sparsity(identity, transpose, dependency, jac_pattern);
// jac_pattern 现在包含 Jacobian 非零元素的 (row, col) 索引对

// ========== 第二步：用稀疏模式计算 Jacobian ==========
CppAD::sparse_rcv<std::vector<size_t>, std::vector<double>> sparse_jac;
CppAD::sparse_jac_work work;  // 工作空间（缓存 coloring 信息）
std::string coloring = "cppad";  // 图着色算法
func.sparse_jac_rev(x, jac_pattern, coloring, sparse_jac, work);
// sparse_jac 只包含非零元素——对稀疏 Jacobian 大幅加速

// work 对象缓存了图着色结果——第二次调用时无需重复着色
func.sparse_jac_rev(x_new, jac_pattern, coloring, sparse_jac, work);
```

**为什么稀疏性如此重要？** 机器人动力学的 Jacobian 有显著的稀疏结构。以串联机械臂为例，关节 $i$ 的位置只依赖于上游关节 $1, 2, \ldots, i$，产生下三角稀疏模式。图着色（graph coloring）将多个不冲突的列合并为一次前向传播，大幅减少所需传播次数：

| 机器人 | Jacobian 维度 | 非零元素比例 | 图着色后传播次数 | 稀疏加速比 |
|:---|:---|:---|:---|:---|
| 7-DOF 机械臂 | $6 \times 7$ | ~60% | 4（vs 7） | ~1.7x |
| Go2 四足 | $24 \times 48$ | ~35% | 12（vs 48） | ~4.0x |
| H1 人形 | $24 \times 67$ | ~25% | 15（vs 67） | ~4.5x |

图着色的直觉值得多说一句：它解决的是"哪些 Jacobian 列可以共用一次前向传播"。两列若在所有行上的非零位置都不重叠（结构上正交），就可以把它们的 seed 叠加进同一次传播——一次算出两列，事后按非零位置拆开即可。把"列"看成图的顶点、"两列在同一行都非零"看成一条边，那么"能共用一次传播的列集合"就是图的一个**独立集**，而把所有列划分进最少的独立集，正是**图着色**问题。着色数就是所需的传播次数。

> **本质洞察**：稀疏 AD 的加速从不来自"少算了某些导数"——所有非零导数一个不少地都算了。它来自"把多个互不冲突的方向塞进同一次传播"，本质是用图论的独立集划分，把 $n$ 次稠密传播压缩成 $\chi$ 次。这与稠密线性代数里用分块把多个右端项一次性回代是同一种"批处理"思想：不改变要算的东西，只改变算的批次组织。所以稀疏性收益的上限，由 Jacobian 的结构（着色数 $\chi$）而非维度 $n$ 决定——一个 $1000\times1000$ 但每行只有 3 个非零的带状 Jacobian，着色数可能只有个位数。

### 内存模型与性能考量 ⭐⭐⭐

CppAD tape 的内存消耗与录制的运算数量线性相关：

| 模型 | DOF | 估计运算数 | tape 内存 | 解释执行 Jacobian |
|:---|:---|:---|:---|:---|
| 3-DOF 平面臂 | 3 | ~500 ops | ~8 KB | ~1 $\mu$s |
| 7-DOF Franka | 7 | ~5,000 ops | ~80 KB | ~5 $\mu$s |
| Go2 四足 | 18 | ~15,000 ops | ~240 KB | ~10 $\mu$s |
| H1 人形 | 25 | ~30,000 ops | ~480 KB | ~20 $\mu$s |

tape 的内存开销在 KB 级别，通常不是瓶颈。真正的瓶颈是**解释执行开销**（interpreter overhead）。每条 tape 指令的执行路径为：

1. 从 tape 数组读取操作码 $\to$ 缓存访问
2. `switch(opcode)` 或虚函数分派 $\to$ **分支预测失败**（操作码序列无规律）
3. 通过索引查找操作数 $\to$ 间接寻址，可能 cache miss
4. 执行基本运算（sin、cos 等） $\to$ 实际有效计算
5. 将结果写回 tape 工作数组

步骤 2 和 3 中的分支预测失败和间接寻址是性能杀手。CPU 的分支预测器面对看似随机的操作码序列无计可施，流水线频繁 flush。这就是 CppAD 解释执行比等价的原生 C 代码慢 5--10 倍的根本原因。

CppADCodeGen 的解决方案：将 tape "展平"为纯 C 代码——**消除所有间接层**。没有 switch，没有索引查找，只有线性的赋值语句。编译器可以自由地进行寄存器分配、指令调度和 SIMD 向量化。这个过程好比把一份用查表法执行的菜谱（"查步骤 3→ 查材料表 → 查火候表"）直接展开为一段连续的操作指令（"放油 → 加盐 5g → 大火 30 秒"），厨师不再需要反复翻书，一气呵成。

> **常见陷阱**
>
> 1. **忘记调用 `Independent()`**：直接用 `AD<double>` 计算并传给 `ADFun` 构造函数 $\to$ tape 为空 $\to$ 运行时 `ADFun` 报错或产生零 Jacobian。调试这类问题非常痛苦，因为错误信息通常是 "zero order forward result has wrong size" 之类不直观的描述。
> 2. **在 tape 录制期间调用 `CppAD::Value()`**：`Value(ax[0])` 提取 `AD<double>` 的数值部分，返回 `double`。这个 `double` 脱离了 tape 的追踪，后续用它参与的计算会被 tape 视为常量乘法/加法。最终导致 Jacobian 缺少对应的偏导数项——一个极难发现的 silent bug。
> 3. **tape 录制期间的线程冲突**：CppAD 使用**线程局部**的全局 tape（每线程一条 tape）。如果在录制期间调用了一个也使用 `AD<double>` 的库函数（如 Pinocchio 的某些内部计算），该函数的运算也会被记录到当前 tape 上——这通常是你想要的。但如果两个线程同时录制，就必须用 CppAD 的多线程 tape 管理机制（`thread_alloc`）。

### 练习 ⭐⭐

1. **[编程题]** 用 CppAD 实现 Rosenbrock 函数 $f(x,y) = (1-x)^2 + 100(y-x^2)^2$ 的梯度和 Hessian 计算。在 $(x,y) = (0.5, 0.5)$ 处求值，并与解析结果对比。解析梯度：$\nabla f = \big(-2(1-x) - 400x(y-x^2),\; 200(y-x^2)\big)$。在 $(0.5, 0.5)$ 处：$\nabla f = (-1 + 400 \times 0.5 \times 0.25,\; 200 \times 0.25) = (49, 50)$。验证 CppAD 的数值结果。

2. **[对比题]** 构造一个 $f: \mathbb{R}^{10} \to \mathbb{R}^{3}$ 的函数（例如 $f_i(x) = \sum_j \sin(x_j + i \cdot x_j^2)$），分别用以下方式计算完整 $3 \times 10$ Jacobian：(a) Forward 模式（10 次 `Forward(1, ...)` 调用）；(b) Reverse 模式（3 次 `Reverse(1, ...)` 调用）；(c) `func.Jacobian(x)` 高层接口。验证三种结果在机器精度内一致。用 `std::chrono::high_resolution_clock` 计时 10000 次重复调用，比较耗时。反向模式是否如理论预期比前向模式快约 $10/3 \approx 3.3$ 倍？

---

## 48.4 CppADCodeGen——从 Tape 到 `.so` 动态库 ⭐⭐⭐

> **这一节解决什么问题**：CppAD 解释执行 tape 仍有显著开销（虚函数调度、分支预测失败）。CppADCodeGen 将 tape 翻译为纯 C 代码，编译为 `.so`，实现零开销原生调用。

### 架构总览 ⭐⭐

CppADCodeGen 的流水线是一个**离线编译**流程，每一步都有明确的产物：

```
C++ 函数 f(x)                          ← 你写的代码
     │
     ▼  用 AD<CG<double>> 类型录制
CppAD tape (符号化)                     ← 内存中的 ADFun<CG<double>> 对象
     │
     ▼  ModelCSourceGen: tape → C 源码
纯 C 源文件 (.c)                        ← /tmp/cppadcg_xxxxxx.c
     │
     ▼  GCC/Clang -O2 -march=native
共享库 (.so)                            ← /tmp/cppadcg_xxxxxx.so
     │
     ▼  dlopen + dlsym
运行时高速调用                           ← model->Jacobian(x) 零开销
```

关键洞察：代码生成和编译在**部署前离线完成**。运行时只有 `dlopen` 加载（一次性，~1 ms）和原生函数调用（每次 ~1.5 $\mu$s）的开销。假如没有代码生成这一步，MPC 热路径每次迭代都必须解释执行 tape 指令——就像每次做菜都要逐行翻阅菜谱查步骤，而不是早已把步骤背熟一气呵成。

### `CG<double>` 标量类型 ⭐⭐⭐

CppADCodeGen 引入了关键类型 `CppAD::cg::CG<double>`。它将 CppAD 的 tape 从"数值记录"升级为"符号记录"——不再存储中间数值，而是存储符号表达式（变量名、运算类型）。

**完整的类型链：**

```
double                              → 普通数值计算，不被追踪
AD<double>                          → CppAD 数值 tape（解释执行）
CG<double>                          → CppADCodeGen 的符号标量
AD<CG<double>>                      → 可生成代码的符号 tape（本节主角）
```

当你用 `AD<CG<double>>` 录制 tape 时，tape 中每个操作存储的不是数值结果，而是一条形如 `v3 = v1 * sin(v2)` 的符号表达式。这些表达式可以直接翻译为 C 语句。

```cpp
// 工业界通用的类型别名（OCS2、Crocoddyl 都用类似约定）
using CGD   = CppAD::cg::CG<double>;      // CodeGen 符号标量
using ADCGD = CppAD::AD<CGD>;             // AD 包装的 CodeGen 标量
```

### 完整流水线代码 ⭐⭐

以下是一个完整的、可编译运行的 CppADCodeGen 示例。这里故意使用一个 2-DOF toy 函数
$y = M(q)v$ 来展示 CodeGen 流水线和 Jacobian 结构；它形状像“质量矩阵乘速度”，但不是完整的机器人动力学扭矩公式：

```cpp
#include <cppad/cg.hpp>
#include <cppad/cg/cppadcg.hpp>
#include <iostream>
#include <chrono>

using CGD   = CppAD::cg::CG<double>;   // CodeGen 符号标量
using ADCGD = CppAD::AD<CGD>;          // 可生成代码的 AD 标量

int main() {
    const size_t n = 4;  // 输入: [q1, q2, dq1, dq2]
    const size_t m = 2;  // 输出: [y1, y2]

    // ============================================================
    // Step 1: 用 AD<CG<double>> 录制符号 tape
    // ============================================================
    CppAD::vector<ADCGD> ax(n), ay(m);
    ax[0] = 1.0; ax[1] = 0.5;   // q1, q2 初始值（仅决定分支走向）
    ax[2] = 0.1; ax[3] = -0.2;  // dq1, dq2
    CppAD::Independent(ax);     // 开始符号录制

    // 2-DOF toy function: y = M(q) * v
    // M(q) = [m1+m2*cos(q2),  m2*sin(q2)]
    //        [m2*sin(q2),     m2         ]
    ADCGD m1 = 2.0, m2 = 1.0;
    ADCGD M00 = m1 + m2 * CppAD::cos(ax[1]);
    ADCGD M01 = m2 * CppAD::sin(ax[1]);
    ADCGD M10 = M01;  // 对称项复用——CodeGen 会检测到这一点
    ADCGD M11 = m2;

    ay[0] = M00 * ax[2] + M01 * ax[3];  // y1
    ay[1] = M10 * ax[2] + M11 * ax[3];  // y2

    // 创建 ADFun（注意模板参数是 CGD）
    CppAD::ADFun<CGD> fun(ax, ay);

    // ============================================================
    // Step 2: 配置代码生成
    // ============================================================
    CppAD::cg::ModelCSourceGen<double> cgen(fun, "matrix_velocity_2dof");
    cgen.setCreateForwardZero(true);     // 生成函数值求值代码
    cgen.setCreateJacobian(true);        // 生成稠密 Jacobian 代码
    cgen.setCreateSparseJacobian(true);  // 生成稀疏 Jacobian 代码
    cgen.setCreateHessian(true);         // 生成 Hessian 代码

    // ============================================================
    // Step 3: 生成 C 源码并编译为 .so 动态库
    // ============================================================
    CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);
    CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen);

    // 配置编译器
    CppAD::cg::GccCompiler<double> compiler;
    compiler.addCompileFlag("-O2");            // 标准优化
    compiler.addCompileFlag("-march=native");  // 使用本机 SIMD 指令集
    // -ffast-math 可能改变 NaN/Inf、结合律和有符号零语义。
    // 安全关键实时路径默认不要开启；只有在单独做过数值回归后才考虑。

    // 执行编译！内部流程：
    //   1. 将符号 tape 翻译为 C 源码 → /tmp/cppadcg_xxx.c
    //   2. 调用 gcc 编译为 .so → /tmp/cppadcg_xxx.so
    //   3. dlopen 加载 .so 到当前进程
    auto dynamicLib = p.createDynamicLibrary(compiler);

    // ============================================================
    // Step 4: 运行时调用预编译函数
    // ============================================================
    auto model = dynamicLib->model("matrix_velocity_2dof");

    std::vector<double> x = {0.8, 0.3, 0.5, -0.1};

    // 函数值——调用的是编译后的原生代码
    std::vector<double> y = model->ForwardZero(x);
    std::cout << "y = [" << y[0] << ", " << y[1] << "]" << std::endl;

    // Jacobian——同样是预编译的原生代码
    std::vector<double> jac = model->Jacobian(x);
    std::cout << "Jacobian (2x4):" << std::endl;
    for (size_t i = 0; i < m; i++) {
        for (size_t j = 0; j < n; j++)
            std::cout << "  " << jac[i * n + j];
        std::cout << std::endl;
    }

    // -------- 性能基准测试 --------
    const int N_BENCH = 100000;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int k = 0; k < N_BENCH; k++)
        model->Jacobian(x);
    auto t1 = std::chrono::high_resolution_clock::now();
    double us_per_call =
        std::chrono::duration<double, std::micro>(t1 - t0).count() / N_BENCH;
    std::cout << "CodeGen Jacobian: " << us_per_call << " us/call" << std::endl;

    return 0;
}
```

### 生成的 C 代码分析 ⭐⭐⭐

CppADCodeGen 将符号 tape 翻译为什么样的 C 代码？以上面 2-DOF toy 函数的 Jacobian 为例，生成代码的结构大致如下（简化后便于阅读）：

```c
/* 由 CppADCodeGen 自动生成 —— 请勿手动编辑 */
/* 模型: matrix_velocity_2dof, Jacobian: 2 输出 x 4 输入 */

void matrix_velocity_2dof_sparse_jacobian(
    const double *x,      /* 输入: [q1, q2, dq1, dq2] */
    double       *jac,    /* 输出: 2x4 Jacobian */
    unsigned long const **row,
    unsigned long const **col,
    unsigned long       *nnz)
{
    double v0, v1, v2;

    // 三角函数只计算一次（公共子表达式消除）
    v0 = cos(x[1]);         // cos(q2)
    v1 = sin(x[1]);         // sin(q2)
    v2 = -v1;               // -sin(q2)

    // dy1/dq1 = 0  → 稀疏格式中直接跳过，不输出
    // dy1/dq2 = -sin(q2)*v1 + cos(q2)*v2
    jac[0] = v2 * x[2] + v0 * x[3];
    // dy1/dv1 = 2.0 + cos(q2)
    jac[1] = 2.0 + v0;
    // dy1/dv2 = sin(q2)
    jac[2] = v1;
    // dy2/dq2 = cos(q2)*v1
    jac[3] = v0 * x[2];
    // dy2/dv1 = sin(q2)
    jac[4] = v1;
    // dy2/dv2 = 1.0（常数）
    jac[5] = 1.0;

    *nnz = 6;  // 8 个元素中只有 6 个非零
    // row 和 col 指向静态数组，描述稀疏结构
}
```

注意生成代码的五个关键特征：

1. **纯 C 语言**——没有模板、没有虚函数、没有类、没有动态内存分配。这使得 GCC 可以进行最激进的优化。
2. **循环完全展开**——所有循环在代码生成阶段已消解为具体的赋值语句。没有循环计数器、没有循环边界检查。
3. **局部变量 `v0`, `v1`, ...`**——所有中间值都是局部 `double`，编译器可以自由地将它们映射到 CPU 寄存器。
4. **公共子表达式消除**（CSE）——`sin(x[1])` 和 `cos(x[1])` 各只计算一次，结果被多处引用。CppADCodeGen 内置了 CSE 优化 pass。
5. **零元素跳过**——Jacobian 中确定为零的元素（如 $\partial\tau_1/\partial q_1 = 0$）在稀疏模式下不占用输出空间。

这些特征使得 GCC `-O2` 优化后的代码性能**接近手写最优 C 代码**。实测表明，CodeGen 生成代码的 FLOP 数通常只比手写多 5--15%（主要来自未完全消除的临时变量）。

### 符号 tape 内部机制——CodeGen 凭什么这么快 ⭐⭐⭐⭐

上一节展示了生成代码"长什么样"，但没有回答一个更深的问题：**从一条线性的 tape，到一段经过公共子表达式消除、死代码删除、拓扑排序的高质量 C 代码，中间到底发生了什么？** 理解这一层，才能解释为什么 CodeGen 的输出常常比初学者手写的 Jacobian 还紧凑，也才能在生成代码异常臃肿时知道去哪里排查。

**第一层：`CG<double>` 把"数值记录"升级为"符号记录"。** 回顾 §48.3，普通 `AD<double>` 的 tape 在每个槽位存的是一个 `double` 数值。而 `CG<double>` 的每个对象内部持有的不是数值，而是一个**表达式图节点**（expression graph node）——它记录"我是哪种运算、我的输入是哪些节点"。当你写 `M00 = m1 + m2 * cos(ax[1])` 时，`CG<double>` 不计算任何数，而是在内部构建出一棵子树：

$$
\texttt{M00} \;\to\; \mathrm{ADD}\big(\,\texttt{m1},\; \mathrm{MUL}(\texttt{m2},\, \mathrm{COS}(\texttt{ax[1]}))\,\big)
$$

整条 tape 录制完毕后，CppADCodeGen 手里握的是一张完整的 DAG（有向无环图）——节点是运算，边是数据依赖。生成 C 代码的过程，本质就是**对这张 DAG 做一次拓扑排序，并为每个节点发射一行赋值语句**。这与编译器把抽象语法树降级为三地址码（three-address code）是同一类操作。

> **本质洞察**：CppADCodeGen 不是"AD 库"，而是一个**领域特定的微型编译器**。它的前端是 C++ 运算符重载（把你的算法"录"成 DAG），中端是一组图优化 pass（CSE、常量折叠、死代码消除），后端是 C 代码发射器。理解了这个三段式结构，就明白了为什么它的产物质量取决于"DAG 优化得多干净"，而非"你写的 C++ 多巧妙"。

**第二层：公共子表达式消除（CSE）。** DAG 表示天然支持节点共享——如果 $\cos(q_2)$ 在 Jacobian 的多个元素中出现，它们指向的是 DAG 里**同一个 COS 节点**，而不是三个独立节点。代码发射时，这个节点只被发射一次（`v0 = cos(x[1])`），后续全部引用 `v0`。这就是 §48.4 生成代码里 `sin`/`cos` 只算一次的来源。CSE 的收益对动力学尤其大，因为：

| 计算对象 | 重复子表达式的来源 | CSE 典型收益 |
|:---|:---|:---|
| RNEA Jacobian | 同一关节的旋转矩阵被多个连杆复用 | 三角函数调用减少 40--70% |
| 接触 Jacobian | 多个接触点共享同一运动链上游 | 中间帧变换共享 |
| 代价函数 Hessian | 二阶导共享一阶导的中间量 | 一阶量被一/二阶导同时引用 |

**第三层：死代码消除（DCE）与 tape 优化。** 如果你录制了输出 `ay`，但某个中间变量 `tmp` 计算了却没有任何输出依赖它，这个节点在 DAG 上是"孤立的"——拓扑排序时根本不会被访问到，对应的 C 代码也就不会生成。这就是为什么 §48.5 中调用 `ad_fun.optimize("no_compare_op")` 很关键：

$$
\text{tape}_{\text{raw}} \xrightarrow{\;\texttt{optimize()}\;} \text{tape}_{\text{opt}} \quad (\text{运算数通常减少 } 10\%\text{--}30\%)
$$

`optimize()` 在 CppAD 层面就先做一轮 CSE + DCE，把 tape 缩小后再交给 CodeGen，等于让后续的 C 代码生成和 GCC 编译都处理更小的输入——既减小生成代码体积，又缩短编译时间。参数 `"no_compare_op"` 表示"优化时把比较运算（`<`、`>`）当作可移除的，因为 CodeGen 路径里它们已经被 `CondExp` 节点接管"。

> **对比性思维：不是"录得越少 tape 越好"，而是"DAG 优化得越彻底越好"。** 初学者常以为要手动减少中间变量来"帮"CodeGen。这是误解——你写 10 个临时变量还是 1 行长表达式，录进 DAG 后结构完全一样，CSE 会自动合并相同子树。真正影响产物质量的是 DAG 的优化 pass，而不是你的源码写法。所以教学上应当**优先写清晰的代码**（多用有名字的中间变量），把性能交给 CodeGen 的优化器。

**第四层：原子函数（atomic functions）——给 DAG"打补丁"。** 有些运算无法或不值得展开成基本运算的 DAG，例如：调用一个外部 LAPACK 的矩阵分解、一段含有大量分支的查表逻辑、或一个本身已经高度优化的手写核函数。CppAD 允许把这类运算封装为**原子操作**（`CppAD::atomic_three` / 旧接口 `atomic_base`），它在 tape 上是一个"黑盒节点"：录制时不展开其内部，但提供自定义的前向/反向传播规则。

```cpp
// 原子函数的概念骨架（教学简化）：把一段不可展开的运算封装为单个 tape 节点
class MyAtomicSolve : public CppAD::atomic_three<double> {
    // forward(): 定义这个黑盒如何做零阶/一阶前向传播
    bool forward(/* ... */) override { /* 自定义函数值 + 切线 */ return true; }
    // reverse(): 定义这个黑盒如何做反向传播（VJP）
    bool reverse(/* ... */) override { /* 自定义伴随回传 */ return true; }
};
```

原子函数的工程意义：**它把"哪些部分用 AD 自动展开、哪些部分用手写解析导数"这条边界，变成可调的设计参数**。这正好呼应了 §48.7 中 Aligator 的混合策略——对刚体动力学这种已有快速解析导数的部分，可以用原子函数把 Pinocchio 的 `computeABADerivatives` 包成一个黑盒节点，让 CodeGen 在外层串接其余部分，而不必把整个 RNEA 都展开成几万行 C 代码。

> **理论-工程桥接**：原子函数在数学上就是"在 DAG 的某个节点处，用你提供的雅可比块替换自动求出的雅可比块"。链式法则在该节点不再向下递归，而是直接采信你给的 $\frac{\partial \text{out}}{\partial \text{in}}$。这与神经网络里"自定义 `autograd.Function` 的 `backward`"是完全相同的机制——PyTorch 用户写 `custom Function` 给某个算子指定反向规则，CppAD 用户写 `atomic_three` 给某个节点指定反向规则，二者是同一个设计模式在两个框架中的体现。

> **常见陷阱**
>
> 1. **误以为生成代码体积正比于 DOF。** 实际上它正比于**优化后 DAG 的节点数**。一个写法糟糕、CSE 失效的 12-DOF 模型，可能生成比 19-DOF 模型更大的代码。排查臃肿代码时，先看 `optimize()` 是否被调用、是否有大量不该出现的重复子表达式（如循环里反复 `cast` 同一常量）。
> 2. **在 tape 录制期间用运行时 `double` 常量参与判断后选择性构建表达式。** 例如 `if (link_mass > 0.01) { ... } else { ... }` 看似在"剪枝"，实则让 DAG 结构依赖于录制时的具体模型参数——换一组惯量参数，生成的 `.so` 就不再正确。模型结构性的分支应在录制前用普通 `double` 决定并固化，数值性的分支必须用 `CondExp`。
> 3. **原子函数的反向规则写错却不报错。** 原子函数的 `reverse()` 是你手写的，CppAD 无法验证其正确性——它会全盘采信。如果伴随回传公式写错，整条链的 Jacobian 在经过该节点后全部错误，且现象隐蔽（函数值对、导数错）。务必用数值差分对原子节点的输入输出单独做一次梯度检查（gradient check）。

### 一阶之上：稀疏 Hessian 与 reverse-over-forward ⭐⭐⭐

MPC 的 SQP 通常只需要一阶导（动力学/约束 Jacobian），但代价函数的 Hessian、以及内点法对障碍项的二阶信息，会用到 Hessian。CppAD 计算 Hessian 的标准做法是**反向套前向**（reverse-over-forward）：先对函数做一次前向获得方向上的一阶信息，再对这个一阶过程做一次反向。形式化地，对标量化的 $w^T f$：

$$
H = \nabla_x^2 \big(w^T f\big), \qquad H_{ij} = \frac{\partial^2 (w^T f)}{\partial x_i \, \partial x_j}
$$

CppAD 通过组合 `Forward(1, ·)`（前向一阶，沿方向 $e_j$）和 `Reverse(2, ·)`（反向二阶）一次得到 Hessian 的第 $j$ 列。完整稠密 Hessian 需要扫描 $n$ 个方向，代价 $O(n \cdot C_f)$。但 Hessian 通常高度稀疏且对称——动力学/代价函数的二阶导往往只有少数变量对之间非零。CppAD 用与稀疏 Jacobian 相同的**图着色**思想压缩 Hessian：

$$
\text{着色后传播次数} = \chi(G_H) \ll n
$$

其中 $\chi(G_H)$ 是 Hessian 稀疏图（变量为顶点、非零 $H_{ij}$ 为边）的**着色数**。对称性还允许只计算下三角，进一步省一半。下表给出典型规模的对比直觉：

| 函数 | 输入维度 $n$ | Hessian 非零比例 | 着色数 $\chi$ | 相对稠密加速 |
|:---|:---|:---|:---|:---|
| 二次代价 $\frac{1}{2}x^T Q x$ | 48 | 取决于 $Q$ 结构 | 视 $Q$ 带宽而定 | 带状时显著 |
| 单步动力学的 $w^T f$ | 48 | ~20% | ~12 | ~4x |
| 多步 rollout 的端到端 Hessian | 数百 | 块带状（block-banded） | 远小于 $n$ | 数十倍 |

```cpp
// 稀疏 Hessian 的标准调用流程（与稀疏 Jacobian 对称）
size_t n = func.Domain();
// Step 1: 检测 Hessian 稀疏模式（哪些 (i,j) 对二阶导非零）
CppAD::sparse_rc<std::vector<size_t>> hes_pattern;
// ... 用 for_hes_sparsity / rev_hes_sparsity 填充 hes_pattern ...

// Step 2: 用图着色 + 对称性压缩计算
CppAD::sparse_rcv<std::vector<size_t>, std::vector<double>> sparse_hes;
CppAD::sparse_hes_work work;        // 缓存着色结果，跨次复用
std::vector<double> w = {1.0};      // 输出加权（标量化）
std::string coloring = "cppad.symmetric";  // 利用对称性的着色
func.sparse_hes(x, w, hes_pattern, coloring, sparse_hes, work);
// sparse_hes 只含非零的下三角元素——对块带状 Hessian 大幅加速
```

> **本质洞察**：Jacobian 用图着色是为了"把互不冲突的列合并到一次传播"，Hessian 用图着色还多了一层对称性可以利用——所以 Hessian 的着色算法（如 `"cppad.symmetric"`、星形着色 star coloring）与 Jacobian 的着色（距离-2 着色）是不同的图论问题。把它们混用（对 Hessian 用非对称着色）不会算错，但会丢掉对称性带来的近 2 倍收益。

### 练习 ⭐⭐⭐

1. **[分析题]** 用 `AD<CG<double>>` 录制 $f(q) = \sum_{i=1}^{4}\big[\cos(q_1+q_2) + \sin(q_2+q_3) \cdot q_i\big]$，生成稀疏 Jacobian 的 C 源码并打开阅读。统计：(a) `cos(q_1+q_2)` 和 `sin(q_2+q_3)` 各被计算了几次——CSE 是否生效？(b) 在录制前先把 `q_1+q_2` 存进一个中间 `ADCGD` 变量，再生成一次，对比两版生成代码的 `cos` 调用次数是否相同，从而验证本节"源码写法不影响 DAG 结构"的论断。
2. **[设计题]** 你有一个含 LAPACK Cholesky 分解的自定义约束，无法直接录进 tape。说明为什么这里适合用 `atomic_three` 封装，并写出该原子节点需要你提供哪两个回调、各自的数学含义（提示：前向传播给函数值与切线，反向传播给 VJP）。如果你把反向回调里的伴随公式写错，会出现什么现象、用什么方法能发现？
3. **[对比题]** 对同一个 $w^T f$，分别用稠密 `Hessian(x, w)` 和稀疏 `sparse_hes(..., "cppad.symmetric", ...)` 计算，记录耗时。当 Hessian 的非零比例从 80% 降到 10% 时，两者的耗时比如何变化？解释为什么稠密方法对稀疏 Hessian 是"浪费"，以及对称着色相比非对称着色省下的那约 2 倍来自哪里。

### 编译优化选项 ⭐⭐

| 编译选项 | 含义 | 对 Jacobian 性能的影响 | 注意事项 |
|:---|:---|:---|:---|
| `-O0` | 无优化 | 基线（仅用于调试） | 变量不会被优化掉，可逐行 GDB |
| `-O2` | 标准优化 | 比 `-O0` 快 3--5x | **推荐默认选项** |
| `-O3` | 激进优化（含自动向量化） | 比 `-O2` 快 0--10% | 编译时间更长 |
| `-Ofast` | `-O3` + `-ffast-math` | 比 `-O3` 快 5--15% | 放宽 IEEE 浮点规范；安全关键路径不作为默认 |
| `-march=native` | 启用本机 CPU 指令集（AVX2/AVX-512） | 10--30% 提升 | `.so` 不可跨 CPU 架构移植 |
| `-ffast-math` | 允许浮点运算重排序 | 5--15% 提升 | 可能改变 NaN/Inf、结合律和有符号零语义；必须经回归验证 |
| `-flto` | 链接时优化 | 5--10% 提升 | 需要 GCC 全链路支持 |

**编译时间**是 CppADCodeGen 的一个实际痛点：

| 模型 | 生成 C 代码行数 | GCC `-O2` 编译时间 | GCC `-O3` 编译时间 |
|:---|:---|:---|:---|
| 3-DOF 平面臂 | ~200 行 | < 1 秒 | < 2 秒 |
| 7-DOF Franka | ~3,000 行 | 5--15 秒 | 15--30 秒 |
| Go2 四足 (18-DOF) | ~15,000 行 | 30--90 秒 | 2--4 分钟 |
| H1 人形 (25-DOF) | ~40,000 行 | 2--5 分钟 | 5--15 分钟 |

编译时间随模型复杂度**超线性增长**。GCC 的寄存器分配（graph coloring NP-hard 的近似算法）和指令调度 pass 在大型单函数上的时间复杂度远超线性。`-O3` 的自动向量化分析进一步加剧了编译时间。

### CMake 集成 ⭐⭐

在实际项目中，代码生成+编译需要集成到 CMake 构建系统。核心模式是 `add_custom_command` + `add_custom_target`：

```cmake
# ==============================================================
# 代码生成器（离线构建工具，输出 .so）
# ==============================================================
add_executable(codegen_dynamics
    src/codegen_dynamics.cpp     # 包含上面的 CppADCodeGen 流水线
)
target_link_libraries(codegen_dynamics
    PRIVATE cppad::cppad         # CppAD header-only
            cppadcg              # CppADCodeGen
            pinocchio::pinocchio # 如果动力学用了 Pinocchio
)

# 自定义命令：运行 codegen 可执行文件，产出 .so
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/generated/dynamics_model.so
    COMMAND $<TARGET_FILE:codegen_dynamics>
            --urdf ${CMAKE_SOURCE_DIR}/urdf/go2.urdf
            --output ${CMAKE_BINARY_DIR}/generated/
    DEPENDS codegen_dynamics
            ${CMAKE_SOURCE_DIR}/urdf/go2.urdf  # URDF 变化时重新生成
    COMMENT "CodeGen: 生成预编译动力学 Jacobian (.so)"
    VERBATIM
)

# 自定义目标：供其他 target 依赖
add_custom_target(generate_dynamics ALL
    DEPENDS ${CMAKE_BINARY_DIR}/generated/dynamics_model.so
)

# ==============================================================
# MPC 主程序（运行时 dlopen 加载 .so）
# ==============================================================
add_executable(mpc_controller src/mpc_controller.cpp)
target_link_libraries(mpc_controller
    PRIVATE ${CMAKE_DL_LIBS}    # -ldl, 提供 dlopen/dlsym
)
add_dependencies(mpc_controller generate_dynamics)
```

注意几个关键设计决策：

- `DEPENDS` 列表同时包含 codegen 可执行文件和 URDF 文件。任何一个变化都会触发重新生成。
- MPC 主程序**不链接** CppAD 或 CppADCodeGen——它只需要 `dlopen` 来加载生成的 `.so`，部署时的依赖极简。
- `ALL` 关键字使得 `make` 默认就会触发代码生成。如果生成耗时过长，可以去掉 `ALL`，改为按需 `make generate_dynamics`。

> **常见陷阱**
>
> 1. **编译时间爆炸**：CppADCodeGen 为 H1 人形（25-DOF）生成的 C 代码可能超过 40,000 行。GCC `-O2` 需要 2--5 分钟，`-O3` 可能超过 10 分钟。**解决方案**：OCS2 框架将生成代码拆分为多个 `.c` 文件（每个 ~5000 行），用 `make -j8` 并行编译。另一种方案是对不同的函数（ForwardZero、Jacobian、Hessian）分别生成和编译。
>
> 2. **平台不可移植**：`.so` 是平台特定的 ELF 二进制文件。x86-64 上编译的 `.so` 不能在 ARM（如 NVIDIA Jetson Orin）上运行。如果 MPC 运行在嵌入式平台上，必须在**目标平台上编译**或使用交叉编译工具链。`-march=native` 更是使得同一 x86 平台的不同微架构（如 Haswell vs Zen4）之间可能不兼容。
>
> 3. **URDF 变化需要重新生成**：机器人模型的任何变化（关节质量、连杆惯量、运动链拓扑）都可能使已生成的 `.so` 失效。OCS2 的 `CppAdInterface` 主要按 `folderName/modelName/cppad_generated/{modelName}_lib.so` 管理生成库，`loadModelsIfAvailable()` 看到库文件存在就会加载；它本身并不会自动比较 URDF 文件内容。工程上应把 URDF、AD 函数结构、编译参数等变化纳入外层构建依赖或改变 `modelName/folderName`，否则可能加载到旧库。

### 练习 ⭐⭐

1. **[编程题]** 对 3-DOF 平面机械臂实现完整的 CppADCodeGen 流水线。正运动学为 $p_x = l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2) + l_3\cos(\theta_1+\theta_2+\theta_3)$，$p_y$ 类似。步骤：(a) 用 `AD<CG<double>>` 录制 FK 函数；(b) 调用 `ModelCSourceGen` 生成 C 源码并保存到文件（用 `cgen.setCreateJacobian(true)` 再调用 `libcgen.getSources()`）；(c) 用 GCC 编译为 `.so`；(d) `dlopen` 加载并调用 `model->Jacobian(q)` 求解析 Jacobian；(e) 与 CppAD 解释执行版本和数值差分版本在精度（相对误差）和速度（$\mu$s/call）上对比。

2. **[分析题]** 在上一题中，找到 CppADCodeGen 生成的 C 源文件（默认在 `/tmp/` 下），阅读 Jacobian 函数的代码。统计：(a) 总代码行数；(b) `sin()`/`cos()` 调用次数——CppADCodeGen 是否做了公共子表达式消除？例如 $\sin(\theta_1+\theta_2)$ 出现在多个 Jacobian 元素中，是否只计算了一次？(c) 手写等价的 Jacobian C 函数，比较 FLOP 数。CppADCodeGen 的代码质量（多余操作的百分比）如何？

3. **[编译实验题]** 对同一个 7-DOF 模型的生成 C 代码，分别用 `-O0`、`-O2`、`-O3`、`-O2 -march=native` 四套选项编译，各测一次"编译耗时"和"运行时单次 Jacobian 耗时"。(a) 画出"编译时间 vs 运行时间"的散点，验证 §48.4 表格的趋势；(b) 解释为什么 `-O0` 运行慢但编译快，而 `-O3` 反之；(c) 若把模型换成 19-DOF 人形，预测哪一项（编译时间/运行时间）会超线性恶化，为什么。

---

## 48.5 Pinocchio + CppAD + CodeGen 完整流水线 ⭐⭐⭐

> **这一节解决什么问题**：把 CppAD 和 CppADCodeGen 应用到真实的 Pinocchio 动力学计算中，实现从 URDF 加载到预编译 `.so` 调用的完整流水线。

### 五步流水线 ⭐⭐⭐

整条流水线的核心思路：**让 Pinocchio 的 RNEA 算法在 CppAD 的 tape 上"跑一遍"**，然后把 tape 翻译成纯 C 代码并编译为共享库。运行时直接调用 `.so`，不再有任何 tape 解释开销。

```
URDF ──→ Pinocchio Model<ADCGScalar> ──→ tape 录制 ──→ C 源码 ──→ .so ──→ MPC 调用
         Step 1                          Step 2        Step 3     Step 4   Step 5
```

**Step 1：用 CodeGen 标量类型加载 URDF**

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/codegen/cppadcg.hpp>

// ---- 类型定义：三层嵌套 ----
// 最内层：CG<double> —— CodeGen 的符号标量
using CGScalar   = CppAD::cg::CG<double>;
// 中间层：AD<CG<double>> —— CppAD 的 AD 标量，包裹 CG
using ADCGScalar = CppAD::AD<CGScalar>;
// Pinocchio 模型模板实例化
using CGModel    = pinocchio::ModelTpl<ADCGScalar>;
using CGData     = pinocchio::DataTpl<ADCGScalar>;

// 先用 double 加载 URDF（Pinocchio 的 parser 只支持 double）
pinocchio::ModelTpl<double> model_d;
pinocchio::urdf::buildModelFromXML(urdf_string, model_d);

// cast 到 CG 标量类型——Pinocchio 支持标量类型转换
CGModel model_cg = model_d.cast<ADCGScalar>();
CGData  data_cg(model_cg);
```

这里的关键在于 Pinocchio 的 `ModelTpl` 是标量类型参数化的模板。`cast<>()` 会把所有内部数据（惯量矩阵、关节轴等）从 `double` 转为 `ADCGScalar`，使得后续算法能在 tape 上录制。

**Step 2：在 tape 上录制 RNEA**

```cpp
// 自变量维度：q(nq) + v(nv) + a(nv)
const int nq = model_cg.nq;
const int nv = model_cg.nv;
const int nx = nq + 2 * nv;

// 声明自变量向量并开始录制
Eigen::Matrix<ADCGScalar, Eigen::Dynamic, 1> ax(nx);
CppAD::Independent(ax);  // ← tape 录制开始

// 从拼接向量中切片出 q, v, a
auto q = ax.segment(0, nq);
auto v = ax.segment(nq, nv);
auto a = ax.segment(nq + nv, nv);

// 在 tape 上执行 RNEA（递推牛顿-欧拉算法）
pinocchio::rnea(model_cg, data_cg, q, v, a);

// 因变量：广义力 tau
Eigen::Matrix<ADCGScalar, Eigen::Dynamic, 1> ay(nv);
for (int i = 0; i < nv; ++i) ay[i] = data_cg.tau[i];

// 构建 ADFun 对象（tape 录制结束）
CppAD::ADFun<CGScalar> ad_fun;
ad_fun.Dependent(ax, ay);
ad_fun.optimize("no_compare_op");  // 优化 tape：移除比较操作
```

> **关键细节**：`optimize("no_compare_op")` 告诉 CppAD 在优化 tape 时忽略条件比较操作。这对 CodeGen 路径很重要，因为条件分支在生成的 C 代码中需要特殊处理。

**Step 3：生成 C 源码**

```cpp
#include <cppad/cg/cppadcg.hpp>

// 创建源码生成器
CppAD::cg::ModelCSourceGen<double> cgen(ad_fun, "rnea_codegen");
cgen.setCreateForwardZero(true);    // 生成 ForwardZero（函数值计算）
cgen.setCreateJacobian(true);       // 生成稠密 Jacobian
cgen.setCreateHessian(false);       // RNEA 一般不需要 Hessian
cgen.setCreateSparseJacobian(true); // 生成稀疏 Jacobian（推荐）

// 包装成库级源码生成器
CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);
```

**Step 4：编译为共享库**

```cpp
// 配置 GCC 编译器
CppAD::cg::GccCompiler<double> compiler;
compiler.setSourcesFolder("/tmp/cppadcg_src");  // 源码临时目录
compiler.addCompileFlag("-O2");                   // 优化级别
compiler.addCompileFlag("-march=native");         // 利用本机 SIMD 指令

// 编译并生成 .so 文件
CppAD::cg::DynamicModelLibraryProcessor<double> processor(libcgen);
auto dynamicLib = processor.createDynamicLibrary(compiler);
// → 输出: /tmp/cppadcg_src/rnea_codegen.so
```

编译时间取决于机器人 DOF：7-DOF 机械臂约 10-30 秒，19-DOF 人形约 1-3 分钟。**这个编译只需做一次**，后续运行直接加载 `.so`。

**Step 5：运行时零开销调用**

```cpp
// 加载已编译的共享库
auto dynamicLib = CppAD::cg::LinuxDynamicLib<double>(
    "/tmp/cppadcg_src/rnea_codegen.so");
auto model = dynamicLib.model("rnea_codegen");

// 在 MPC 循环中调用——与普通函数调用一样快
std::vector<double> x(nx);  // q, v, a 拼接
// ... 从状态估计器填充 x ...

// 计算 RNEA：tau = ID(q, v, a)
std::vector<double> tau = model->ForwardZero(x);

// 计算 RNEA 的 Jacobian：dtau/d[q,v,a]
// 返回 nv x nx 的矩阵（行优先展平）
std::vector<double> J_flat = model->Jacobian(x);

// 稀疏 Jacobian（推荐用于高 DOF 系统）
std::vector<double> vals;
std::vector<size_t> rows, cols;
model->SparseJacobian(x, vals, rows, cols);
```

### 性能基准 ⭐⭐

以下数据在 Intel i7-12700H（单核，`-O2`）上测得，每个函数运行 10000 次取平均：

| 机器人 | DOF | 方案 | RNEA ($\mu s$) | RNEA+Jacobian ($\mu s$) | 加速比 |
|--------|-----|------|-----------|-------------------|--------|
| Panda | 7 | 数值差分 | $1.2 + 7 \times 2.4 = 18$ | 18 | 1x |
| Panda | 7 | CppAD 解释 | 3.5 | 8 | 2.3x |
| Panda | 7 | **CppADCodeGen** | **0.8** | **1.5** | **12x** |
| Go2 | 12 | 数值差分 | $2.0 + 12 \times 4.0 = 50$ | 50 | 1x |
| Go2 | 12 | **CppADCodeGen** | **1.2** | **2.5** | **20x** |
| H1 | 19 | 数值差分 | $4.0 + 19 \times 8.0 = 156$ | 156 | 1x |
| H1 | 19 | **CppADCodeGen** | **2.0** | **4.5** | **35x** |

趋势很明显：**DOF 越大，CodeGen 的加速比越高**。原因是数值差分的开销随 DOF 线性增长（每个维度一次扰动），而 CodeGen 的 Jacobian 是一段展开后的平坦代码，分支预测友好且可被编译器深度优化。

### Tape 录制的条件分支陷阱 ⭐⭐⭐

这是实际使用中最容易踩的坑。CppAD tape 的本质是**一次执行的线性计算图**——它无法表示"运行时根据值选择不同分支"。

**问题场景**：

```cpp
// !! 错误：tape 上的 if 只录制"当时走过的分支"
ADCGScalar force;
if (CppAD::Value(penetration) > 0.0) {  // Value() 提取当前数值
    force = k * penetration;             // 只有这一支被录制到 tape
} else {
    force = ADCGScalar(0.0);             // 这一支完全被忽略
}
```

当 `penetration` 的录制时值 > 0 时，tape 永远记录的是 `force = k * penetration`，即使运行时 `penetration <= 0`，结果也是错的。

**解决方案一：CppAD 条件表达式**

```cpp
// 正确做法：使用 CppAD::CondExpGt
// 语义：如果 penetration > 0，返回 k*penetration，否则返回 0
ADCGScalar force = CppAD::CondExpGt(
    penetration, ADCGScalar(0.0),    // left > right ?
    k * penetration,                  // true_value
    ADCGScalar(0.0)                   // false_value
);
```

`CondExpGt` 在 tape 上录制为一个特殊节点，生成的 C 代码中变成三目运算符 `(a > b) ? c : d`。两个分支的导数都被正确计算。

**解决方案二：多 tape 策略（OCS2 做法）**

对于离散模式切换（例如腿式机器人的接触模式），为每种模式录制独立的 tape：

```cpp
// 接触模式 1：四足全接触
CppAD::ADFun<CGScalar> fun_stance = recordRNEA(model, contacts_all);

// 接触模式 2：对角步态（LF+RH 接触）
CppAD::ADFun<CGScalar> fun_trot = recordRNEA(model, contacts_diag);

// 运行时根据步态调度器选择对应的编译模型
if (gait_phase == STANCE) tau = model_stance->ForwardZero(x);
else                       tau = model_trot->ForwardZero(x);
```

**解决方案三：光滑近似**

如果不用光滑近似直接把 `max(0, x)` 放进 tape，导数在 $x=0$ 处就是未定义的——优化器在接触切换瞬间会收到垃圾梯度信号，导致 SQP 发散。用可微的光滑函数替换不可微的分支：

$$\text{softplus}(x) = \frac{1}{\alpha} \ln(1 + e^{\alpha x}) \approx \max(0, x) \quad (\alpha \to \infty)$$

```cpp
// 光滑的单边接触力近似
ADCGScalar alpha(100.0);  // 光滑度参数，越大越接近 max(0,x)
ADCGScalar force = k * CppAD::log(1.0 + CppAD::exp(alpha * penetration)) / alpha;
```

> **常见陷阱**：Pinocchio 的一些高级功能（如约束动力学 `constraintDynamics()`）在内部使用条件分支，不能直接通过 tape 录制。使用前务必检查 Pinocchio 文档中的 **AD-compatible** 标记。安全的函数包括：`rnea()`、`aba()`、`crba()`、`computeJointJacobians()` 等核心算法。

### 练习 ⭐⭐

1. **[编程题]** 完成 Go2 四足机器人 RNEA 的完整 CodeGen 流水线。分别用 `double` 直接调用、CppAD 解释执行、CppADCodeGen 三种方式计算 RNEA 及其 Jacobian，对比耗时并绘制柱状图。
2. **[分析题]** 将 tape 录制扩展到 ABA（Articulated Body Algorithm，正动力学），比较 RNEA 和 ABA 的 tape 操作数量与生成 C 代码的行数。解释为什么 ABA 的 tape 通常更大。
3. **[调试挑战题]** 给定一段把单边接触力写成 `if (Value(penetration) > 0) force = k*penetration; else force = 0;` 的 AD 代码。(a) 在 $\text{penetration}_0 = 0.01$ 处录制并生成 `.so`，然后在 $\text{penetration} = -0.02$ 处求 $\partial\text{force}/\partial\text{penetration}$，预测 tape 给出的（错误）值并解释来源；(b) 改用本节三种解法各实现一版（`CondExpGt` / 多 tape / softplus），分别给出在接触切换点 $\text{penetration}=0$ 处导数的行为，并说明为什么 softplus 版在 $0$ 处导数连续而 `CondExpGt` 版导数不连续；(c) 讨论"导数不连续"对 SQP 收敛意味着什么，何时该优先选 softplus。

---

## 48.6 OCS2 CppAdInterface 源码精读 ⭐⭐⭐

> **这一节解决什么问题**：OCS2 如何封装 CppAD/CppADCodeGen，使得 MPC 用户无需关心代码生成细节。

### OCS2 定位与架构 ⭐⭐

OCS2（Optimal Control for Switched Systems）是 ETH RSL 开发的 MPC 框架，专为腿式机器人等切换系统设计。其核心设计决策：**把 AD 的全部复杂性封装在 `CppAdInterface` 内部**，上层 MPC 用户只需定义一个 `ad_function(input, output)` 回调函数。

```
用户代码                    OCS2 内部                       运行时
+---------------+       +---------------------+       +---------------+
| 定义           |       | CppAdInterface      |       | MPC 循环      |
| dynamics      | ----> | 1. record tape       | ----> | 调用 .so      |
| cost          |       | 2. generate C        |       | 零开销求值    |
| constraint    |       | 3. compile .so       |       | SQP 求解      |
+---------------+       | 4. cache by hash     |       +---------------+
                        +---------------------+
```

### CppAdInterface 类设计 ⭐⭐⭐

```cpp
// OCS2 源码简化版：ocs2_core/include/.../CppAdInterface.h
class CppAdInterface {
public:
    // AD 函数签名：用户只需实现这个回调
    using ad_function_t = std::function<void(
        const ad_vector_t& input,
        ad_vector_t& output,
        const ad_vector_t& parameters)>;

    // 构造：传入 AD 函数回调和维度信息
    CppAdInterface(ad_function_t adFunction,
                   int inputDim, int outputDim,
                   std::string modelName,
                   std::string folderName = "/tmp/ocs2");

    // 录制 tape -> 生成代码 -> 编译 -> 缓存
    void createModels(ApproximationOrder order = ApproximationOrder::First,
                      bool verbose = true);

    // 加载已编译的模型（hash 匹配则跳过编译）
    void loadModelsIfAvailable(ApproximationOrder order);

    // 运行时求值接口
    vector_t getFunctionValue(const vector_t& input);
    matrix_t getJacobian(const vector_t& input);

private:
    std::shared_ptr<CppAD::cg::LinuxDynamicLib<double>> dynamicLib_;
    std::string libraryFolder_;
    std::string modelName_;
    std::string libraryName_;  // {folder}/{modelName}/cppad_generated/{modelName}_lib
};
```

用户侧的使用极其简洁：

```cpp
// 用户只需写这个 lambda——定义"输入到输出的映射"
auto dynamicsAd = [&](const ad_vector_t& x,
                       ad_vector_t& y,
                       const ad_vector_t& p) {
    // 伪代码：若用浮动基座四元数配置，nq != nv，不能简单令 q_dot = v。
    // 真实 OCS2 动力学通常在状态参数化/映射层处理配置导数。
    // 这里重点展示 ABA 的 AD 包装方式。
    // x = [q; v; u]，y = [v; v_dot] 或项目定义的状态导数
    auto q = x.head(nq);
    auto v = x.segment(nq, nv);
    auto u = x.tail(nv);  // nv 维广义力（含浮动基座 6 维），非 12 维关节力矩
    // 浮动基座的前 6 维由 OCS2 约束/参数化保证为 0
    // 若输入为 12 维关节力矩 u_joint，需构造: tau = [0_6; u_joint]

    // 调用 Pinocchio ABA（正动力学）
    pinocchio::aba(model_ad, data_ad, q, v, u);

    y.head(nv) = v;               // 最小坐标教学写法；四元数浮基需替换为配置积分映射
    y.tail(nv) = data_ad.ddq;     // v_dot = aba(q, v, u)
};

// 一行搞定整个流水线
CppAdInterface adInterface(dynamicsAd, nx + nu, nx, "go2_dynamics");
adInterface.createModels();  // 首次：录制+编译+缓存；后续：直接加载
```

### 自动重编译机制 ⭐⭐⭐

OCS2 的磁盘缓存机制是其工程上的亮点，但要注意它不是“自动感知所有源文件变化”的构建系统：

```
loadModelsIfAvailable() 典型流程：
  |
  +-- 由 folderName 和 modelName 确定 libraryFolder_
  |
  +-- 检查 libraryFolder_/{modelName}_lib.so 是否存在
  |
  +-- [存在] --> dlopen 加载 --> 返回
  |
  +-- [不存在]
        --> record tape --> generate C --> compile .so
        --> 保存到 libraryFolder_ --> 返回
```

**工程效果**：
- 第一次运行：编译可能需要 30 秒 ~ 3 分钟（取决于模型复杂度）
- 后续运行：如果同名库已存在，毫秒级加载
- 修改了 URDF、状态维度、AD 函数结构或被录进 tape 的常量时，必须显式重新生成，或改变 `modelName/folderName`，或由 CMake/脚本删除旧库后再生成

### 多线程安全性 ⭐⭐⭐

OCS2 MPC 运行在双线程模式：

- **MPC_Node**：接收状态估计，求解 MPC 优化问题（约 50-100 Hz）
- **MRT_Node**（Model Reference Tracking）：以 1 kHz 插值 MPC 解并发送关节指令

两个线程都需要调用动力学函数求值。CppAdInterface 的线程安全策略：

```cpp
// 每个线程持有独立的 GenericModel 实例
// 底层 .so 是同一个（只读代码段），evaluation buffer 是线程局部的
thread_local std::unique_ptr<CppAD::cg::GenericModel<double>> threadModel_;

vector_t CppAdInterface::getFunctionValue(const vector_t& input) {
    if (!threadModel_) {
        // 首次调用时从共享库创建线程局部的模型副本
        threadModel_ = dynamicLib_->model(modelName_);
    }
    std::vector<double> x(input.data(), input.data() + input.size());
    auto y = threadModel_->ForwardZero(x);  // 线程安全：各自的 buffer
    return Eigen::Map<vector_t>(y.data(), y.size());
}
```

关键点：**`.so` 中的代码段 `.text` 是只读的**，多线程读同一份代码没有数据竞争。竞争只可能发生在 evaluation buffer（工作内存）上，用 `thread_local` 彻底消除。

### 与 SQP 求解器的集成 ⭐⭐⭐

OCS2 的 SQP 求解器在每次迭代中需要以下导数信息：

| 调用 | 数学含义 | CppAdInterface 方法 |
|------|---------|-------------------|
| 动力学 $f(x_k, u_k)$ | 离散状态转移 | `dynamicsInterface.getFunctionValue()` |
| $\frac{\partial f}{\partial x}, \frac{\partial f}{\partial u}$ | 动力学线性化 | `dynamicsInterface.getJacobian()` |
| 代价 $l(x_k, u_k)$ | 阶段代价 | `costInterface.getFunctionValue()` |
| $\nabla_x l, \nabla_u l$ | 代价梯度 | `costInterface.getJacobian()` |
| 约束 $g(x_k, u_k) \leq 0$ | 不等式约束 | `constraintInterface.getFunctionValue()` |
| $\frac{\partial g}{\partial x}, \frac{\partial g}{\partial u}$ | 约束 Jacobian | `constraintInterface.getJacobian()` |

所有这些都通过同一套 `CppAdInterface` 抽象提供，SQP 求解器完全不知道底层是 CodeGen 还是其他实现。这种解耦是 OCS2 的设计优点。

### 实战调试：一次完整的"梯度对不上"定位流程 ⭐⭐⭐

把前面散落的排错知识串成一个真实的调试故事，演示遇到 AD 导数异常时**应该按什么顺序排查**，而不是盲目改代码。设想你在给 OCS2 的某个自定义足端高度约束写 AD 函数后，SQP 在某些位型下不收敛，怀疑约束 Jacobian 有问题。

**第一步：隔离——把怀疑对象从整个 MPC 里拆出来单独测。** 不要在完整 MPC 里调试 AD——状态太多，因果链太长。直接构造一个独立的 `CppAdInterface`，只包住那个约束函数，在控制循环之外反复调用 `getJacobian(x)`。这一步的本质是把"系统级现象"降维成"函数级现象"。

**第二步：建立金标准——用数值差分作为 ground truth。** AD 的导数对不对，唯一可靠的裁判是有限差分。在同一个 $x$ 上，对约束函数用 `double` 直接求值，做中心差分：

$$
J^{\text{num}}_{ij} = \frac{g_i(x + h e_j) - g_i(x - h e_j)}{2h}, \qquad h \approx 10^{-6}
$$

然后逐元素比较 $\big|J^{\text{AD}}_{ij} - J^{\text{num}}_{ij}\big|$。注意：这里用数值差分不是因为它更准（它更不准），而是因为它的错误模式与 AD **完全独立**——两套独立实现同时算对的概率远高于同时算错。

**第三步：读"错误的形状"——错在哪些元素是关键线索。** 比较结果会落入三种典型形态，每种指向不同病根：

| 错误形态 | 指向的病根 | 下一步动作 |
|:---|:---|:---|
| 某些列**整列为零**（数值差分非零） | 这些输入变量脱离了 tape | 检查这些变量是否经过了 `Value()` 或 `std::` 函数 |
| 全部元素**差一个常数倍** | 单位/缩放问题，或权重 `w` 用错 | 检查输入归一化、参数向量 `p` 的传递 |
| 仅在**特定位型**附近出错，其他点都对 | 硬分支：tape 固化了录制点的分支 | 全文搜 `if`，换成 `CondExp` |

第三种形态正是本案例——约束只在足端接近某高度阈值时出错。这立刻把嫌疑锁定到分支。

**第四步：换录制点验证假设。** 用 §48.2 的自检法：把录制点从原值换成阈值另一侧的值，重新录制并对比导数。如果导数随录制点跳变，假设证实——函数里藏着依赖 AD 值的硬分支（很可能是 `if (foot_height > threshold)` 这类）。

**第五步：修复并回归。** 把硬分支换成 `CondExpGt`，重新生成 `.so`（记得删除旧缓存，否则加载到旧库——这本身又是一个坑），再跑一遍第二步的数值梯度检查。全部位型通过后，才放回完整 MPC 验证收敛性。

> **本质洞察**：这套流程的核心不是任何单一技巧，而是一种**二分查找的调试哲学**——先把问题从系统隔离到函数（缩小空间），再用独立实现建立金标准（确立对错），最后通过"错误的形状"把病根从一大类缩小到一个具体机制。AD 系统的导数 bug 之所以难调，是因为它们"函数值对、导数错"，绕过了大多数断言和单元测试；唯有数值梯度检查这把"独立标尺"能稳定地把它们逼出来。把梯度检查做成默认开启的回归测试，是工程上对抗 AD 隐性 bug 的最有效手段。

### 练习 ⭐⭐

1. **[源码阅读题]** 在 OCS2 源码（`ocs2_core/src/automatic_differentiation/`）中找到 `CppAdInterface` 的完整实现，画出其与 `SystemDynamicsBase`、`CostFunctionBase` 的类关系图。
2. **[思考题]** 如果 `loadModelsIfAvailable()` 只按 `{folderName}/{modelName}` 找到旧的 `.so` 就加载，而没有检查 URDF 文件内容、AD 函数源码或编译参数变化，会导致什么问题？如何用 CMake 依赖、模型内容 hash 或版本化 `modelName` 改进？
3. **[调试题]** 给定一个自定义 AD 代价函数，其某一列 Jacobian 恒为零但数值差分非零。按本节五步流程写出你的排查顺序，并指出最可能的两个根因（提示：`Value()` 与 `std::` 数学函数）。

---

## 48.7 Crocoddyl 与 Aligator 的代码生成方案 ⭐⭐

> **这一节解决什么问题**：与 OCS2 不同，Crocoddyl/Aligator 采用混合策略——手写解析导数为主，选择性使用 CodeGen。

### Crocoddyl：ActionModel + 手写导数 ⭐⭐⭐

Crocoddyl 的设计哲学是**"导数由用户负责"**：

```cpp
// Crocoddyl 的核心抽象
class ActionModelAbstract {
public:
    // calc()：计算状态转移和代价值
    virtual void calc(const Eigen::Ref<const VectorXd>& x,
                      const Eigen::Ref<const VectorXd>& u) = 0;

    // calcDiff()：计算 calc() 对 x, u 的 Jacobian 和 Hessian
    // 用户必须手动实现这个函数!
    virtual void calcDiff(const Eigen::Ref<const VectorXd>& x,
                          const Eigen::Ref<const VectorXd>& u) = 0;
};
```

对于刚体动力学，Crocoddyl 调用 Pinocchio 已经推导好的**解析导数函数**：

```cpp
void ActionModelFreeFwdDynamics::calcDiff(...) {
    // Pinocchio 提供解析推导的动力学导数——无需 AD
    pinocchio::computeABADerivatives(model, data, q, v, tau);
    // data.ddq_dq, data.ddq_dv, data.Minv 直接可用
}
```

**优势**：零编译等待时间，启动即运行，迭代速度快。
**劣势**：只限于 Pinocchio 已推导解析导数的函数；自定义成本/约束需要用户自己手推公式并实现。

> **注意**：Crocoddyl 3.x 进行了重大 API 重构，与 2.x 不向后兼容。新项目建议直接从 3.x 版本开始。

### Aligator (ProxDDP)：并行 Riccati + 选择性 CodeGen ⭐⭐⭐

Aligator 是 INRIA 开发的下一代轨迹优化器，其核心创新在于 **Parallel Riccati Solver**。

传统 DDP/iLQR 的 Riccati 回扫是**严格串行的**——第 $k$ 步的值函数依赖第 $k+1$ 步的结果。这个持续了 30 年的"DDP 天然串行"认知被 Aligator 的并行算法打破：

$$\text{传统 Riccati：} V_k = f(V_{k+1}) \quad \Rightarrow \quad O(N) \text{ 串行步}$$

$$\text{Aligator 并行归约：分块消元} \quad \Rightarrow \quad O(\log N) \text{ 并行步}$$

关键论文：*"Parallel and Proximal Constrained Linear-Quadratic Methods for Real-Time Nonlinear MPC"*, T-RO, March 2025.

在 AD 策略上，Aligator 采用**实用的混合方案**：

- **动力学导数**：调用 Pinocchio 解析导数（`computeABADerivatives` 等）——这是最快的路径
- **复杂成本/约束**：用 CppADCodeGen 自动微分——正确性自动保证
- **简单二次代价**：直接写解析表达式——无需任何 AD 开销

### 为什么解析导数能赢过展平的 CodeGen ⭐⭐⭐

§48.8 会断言"标准刚体动力学导数，Pinocchio 解析导数通常最快"。这里把这个结论的**机理**讲透，因为它直接决定了 Aligator/Crocoddyl 为什么把动力学交给解析导数、而非一律 CodeGen。

刚体动力学的逆/正动力学（RNEA/ABA）本质是一个**沿运动链的递推**：信息从根连杆向叶连杆传一遍（前向递推），再从叶向根传一遍（反向递推）。它们的解析导数（如 `computeRNEADerivatives`）是数学家把这个递推**连同它的导数一起**手工推导出来的——导数计算复用了原算法的中间量（各连杆的空间速度、加速度、力），并保持了 $O(n)$ 的链式递推复杂度。

CodeGen 走的是另一条路：它把整条递推**完全展平**成一段没有循环、没有结构的赋值流。展平之后：

- **递推结构信息丢失**：原本"第 $i$ 个连杆的导数只依赖上游 $1..i$"这种结构，在展平的标量赋值里变成了编译器需要重新从数据依赖中"猜"的东西——而编译器的别名分析（alias analysis）远不如人类对算法语义的理解精确。
- **稀疏递推退化为稠密展开**：解析导数能直接跳过结构性为零的项；CodeGen 虽有 CSE，但对"因递推结构而为零"的高阶项识别能力有限，常常多算。

用一张对比表把两条路线的本质差异钉死：

| 维度 | Pinocchio 解析导数 | CppADCodeGen 展平代码 |
|:---|:---|:---|
| 复杂度结构 | 保持 $O(n)$ 链式递推 | 展平为无结构指令流，依赖编译器再优化 |
| 中间量复用 | 显式复用原算法的速度/力 | 靠 CSE 被动发现，覆盖不全 |
| 稀疏性 | 推导阶段已利用 | 仅靠数值零检测，部分残留 |
| 正确性来源 | 人工推导（有出错风险） | 自动求导（结构性正确） |
| 自定义函数 | 需重新手推 | 零额外成本 |

> **对比性思维：不是"CodeGen 慢"，而是"CodeGen 丢了一份免费的先验知识"。** 对标准动力学，递推结构是数学家已经免费奉送的强先验；解析导数把它用满，CodeGen 把它扔了再让编译器从零捡回来一部分。但反过来，对于**没有现成解析公式**的自定义代价/约束，根本不存在这份先验——此时 CodeGen 的"结构无关、正确性自动"反而成了最大优势。这就是为什么成熟框架（Aligator）的最优策略是**按是否存在解析先验来分流**：有先验走解析，无先验走 CodeGen。

> **理论-工程桥接**：这条机理直接落到 §48.4.5 的原子函数上——理想的工程实现，是用原子函数把 Pinocchio 解析导数"嵌"进 CodeGen 的 DAG：动力学这个黑盒节点采信解析导数（保住递推结构），其余自定义部分照常自动展开。这样既不丢动力学的先验，又享受 CodeGen 对自定义部分的自动化，是"两条路线取长补短"在代码层面的落地。

### OCS2 vs Crocoddyl/Aligator 设计哲学对比 ⭐⭐

| 维度 | OCS2 | Crocoddyl 3.x | Aligator |
|------|------|-----------|---------|
| AD 策略 | 全部 CppADCodeGen | 手写解析为主 | 解析 + 选择性 CodeGen |
| 编译时间 | 首次启动慢（分钟级） | 即时 | 即时 ~ 快速 |
| 导数正确性 | 自动保证 | 依赖实现者正确性 | 自动 + 解析混合 |
| 求解器 | SQP + HPIPM | DDP / iLQR / FDDP | ProxDDP + Parallel Riccati |
| 约束处理 | 软约束 / 增广拉格朗日 | 有限支持 | 近端约束（原生硬约束） |
| 维护状态 | 维护模式 v1.0 | 3.x 活跃开发 | 活跃开发（推荐） |
| 典型用户 | ANYmal 部署 | 研究原型 | 新项目首选 |

### 设计 tradeoff 的深层原因 ⭐⭐⭐

- **OCS2 "全部 CodeGen"**：ETH RSL 的 ANYmal 需要生产级可靠性，不能依赖人工推导正确性。首次启动慢但运行时极快，适合部署场景。
- **Crocoddyl "全部手写"**：LAAS-CNRS 偏重快速迭代，Pinocchio 已有主要算法的解析导数，研究场景频繁修改不想等编译。
- **Aligator 取两者之长**：动力学用解析导数（最快且已验证），自定义部分用 CodeGen（正确性自动保证），并行求解器减少了对单步极致性能的依赖。

### 练习 ⭐⭐

1. **[对比题]** 分析 OCS2 "tape everything" 和 Crocoddyl "hand-derive everything" 两种极端策略的 tradeoff。在项目的什么阶段各自更合适？
2. **[思考题]** Aligator 的并行 Riccati 需要 $O(\log N)$ 步归约。如果每步动力学求导变快（通过 CodeGen），对并行 Riccati 的整体收益影响大吗？为什么？
3. **[机理题]** 用本节"解析导数保留递推结构、CodeGen 展平丢失结构"的论点解释：为什么对标准 RNEA，Pinocchio 的 `computeRNEADerivatives` 常比 CodeGen 快，而对一个自定义的非刚体软接触代价，结论反转？再说明 §48.4.5 的原子函数如何把两者的优点同时拿到——它在 DAG 的哪个节点采信解析导数、在哪些节点继续自动展开？

---

## 48.8 选型三角：AutoDiffXd / CppAD / CppADCodeGen ⭐⭐

> **这一节解决什么问题**：实际工程中如何选择 AD 方案。

### 决策矩阵 ⭐⭐

| 维度 | CppAD 解释 | CppADCodeGen | Drake AutoDiffXd | Pinocchio 解析 | JAX / PyTorch |
|------|-----------|-------------|-----------------|---------------|------------|
| 运行速度 | 中 | 极快 | 慢 | 最快 | GPU 快 / CPU 慢 |
| 首次启动 | 即时 | 分钟级编译 | 即时 | 即时 | JIT 编译（秒级） |
| 实现难度 | 低 | 中 | 低 | 高（需手推公式） | 低（Python） |
| 语言 | C++ | C++ | C++ | C++ | Python |
| 高阶导数 | 支持 | 支持 | 支持 | 部分支持 | 支持 |
| 稀疏性利用 | 自动检测 | 自动检测 | 不支持 | 需手动指定 | JAX 支持 |
| 条件分支 | `CondExp` | `CondExp` | 原生支持 | N/A | 原生支持 |
| MPC 适用性 | 原型验证 | 生产部署 | 原型验证 | 生产部署 | 训练 / 离线规划 |

### 选型决策树 ⭐⭐

```
开始
 |
 +-- 需要 GPU 大规模并行？
 |    \-- 是 --> JAX (differentiable simulation) 或 PyTorch
 |
 +-- 需要 C++ 实时控制 (<1ms)？
 |    |
 |    +-- 有 Pinocchio 解析导数可用？
 |    |    \-- 是 --> 直接用 Pinocchio 解析导数（最快路径）
 |    |
 |    +-- 模型频繁修改？（研究早期）
 |    |    \-- 是 --> CppAD 解释执行（避免等编译）
 |    |
 |    \-- 生产部署 / 模型已稳定？
 |         \-- 是 --> CppADCodeGen（一次编译，持续受益）
 |
 \-- 快速原型验证？
      +-- C++ 生态 --> Drake AutoDiffXd
      \-- Python 生态 --> CasADi 或 JAX
```

### 一个常见误区 ⭐⭐

> **"CppADCodeGen 一定比 Pinocchio 解析导数快"——这是错的。**

Pinocchio 的解析导数（如 `computeABADerivatives`）是数学家手工推导的公式，充分利用了刚体动力学的递推结构和稀疏性。CodeGen 虽然消除了 tape 解释开销，但生成的代码是"展平"的，**丢失了递推结构信息**。

对于标准刚体动力学导数，Pinocchio 解析导数通常是最快的。CodeGen 的优势场景在于：

1. **自定义成本/约束**——没有现成解析公式可用
2. **复合函数**——多步计算串联后的端到端导数
3. **正确性保证**——避免手动推导出错的风险

### 前沿：Enzyme —— LLVM 级 AD ⭐⭐⭐⭐

传统 AD（CppAD、PyTorch autograd）工作在**源码级**或**操作符级**。Enzyme 则直接在 **LLVM IR 级**进行自动微分：

```
源码 --> [Clang 前端] --> LLVM IR --> [Enzyme Pass] --> 带导数的 IR --> [后端] --> 机器码
```

核心优势：

- **无需类型重载**：不需要 `AD<double>` 特殊类型，直接对原始 `double` 代码求导
- **编译器优化后再微分**：先内联、向量化、死代码消除，再做 AD——生成的导数代码更紧凑高效
- **语言无关**：任何编译到 LLVM IR 的语言（C/C++/Rust/Fortran/Julia）都能使用

当前状态（截至 2025）：Julia 生态已较成熟（`Enzyme.jl`），C++ 生态仍处实验阶段，尚无 Pinocchio/Drake 正式集成。关键论文：Moses & Churavy, NeurIPS 2020。如果 Enzyme 在 C++ 生态成熟，可能使 CppADCodeGen 的整条流水线成为历史——直接在编译期对原始 `double` 代码微分即可。

把 Enzyme 和 CppADCodeGen 并排看，二者的分工边界一目了然——它们都在"编译期"工作，但切入点差了一个抽象层：

| 阶段 | CppADCodeGen | Enzyme |
|:---|:---|:---|
| 录制/捕获 | C++ 运算符重载录成 DAG | 复用 Clang 已生成的 LLVM IR |
| 优化时机 | 先微分（DAG），再交 GCC 优化 | **先优化（IR），再微分** |
| 类型侵入 | 必须把代码模板化为 `AD<CG<double>>` | 零侵入，直接吃原始 `double` 代码 |
| 产物 | 独立 `.so`，运行时 `dlopen` | 直接编进同一二进制的导数函数 |

> **对比性思维：CppADCodeGen 与 Enzyme 不是"新旧之争"，而是"在哪一层做 AD"的取舍。** CppADCodeGen 在源码层捕获计算图，优点是可移植（生成的 C 代码到处能编）、可检视（能打开 `.c` 读）；Enzyme 在 IR 层微分，优点是零类型侵入、且"先优化后微分"能让死代码消除和内联**先于** AD 发生，导数代码更紧凑。前者的代价是必须把动力学代码模板化，后者的代价是绑定 LLVM 工具链且产物不易检视。哪个更好，取决于你更看重"可移植可检视"还是"零侵入高紧凑"。

### 练习 ⭐⭐

1. **[设计题]** 你要为一个 12-DOF 轮足机器人开发 MPC 控制器，项目周期 6 个月（前 3 个月原型，后 3 个月部署）。设计你的 AD 方案迁移路径，并说明每个阶段的选择理由。
2. **[调研题]** 调研 Enzyme 在 Pinocchio 或 Drake 中的集成进展，是否有公开的 benchmark 数据？
3. **[对比论证题]** 用本节"在哪一层做 AD"的视角，论证一个判断：对一个已经稳定、需要长期部署且要在 x86 和 ARM 两种平台运行的动力学模型，CppADCodeGen 与 Enzyme 各自的优劣是什么？结合"先优化后微分"与"产物可移植/可检视"两条性质，给出你的选择并说明在什么条件下会反转这个选择。

---

## 48.9 前沿与展望 ⭐⭐⭐

### 可微分仿真 ⭐⭐⭐

传统流水线是"仿真器 + 外部 AD 工具"各自独立。新一代可微分仿真器将 AD 融入物理引擎核心：

**MuJoCo MJX**（2023-2025）：MuJoCo 的 JAX 后端，整个物理引擎端到端可微分。直接对仿真轨迹求梯度，无需 CppAD/CodeGen。主要用于 RL 训练（大规模并行 + GPU 加速），但 MPC 仍需 C++ 实时执行。

**Drake AutoDiffXd 路线**：Drake 所有 System 组件都支持 `AutoDiffXd` 标量类型——"仿真即微分"。优势是与仿真器无缝集成，劣势是运行时性能不如 CodeGen。

可微分仿真与本章 CppADCodeGen 的关系，可以用一句话定位：**两者求的是不同对象的导数**。CppADCodeGen 求的是"一步动力学 $f(x,u)$ 对 $(x,u)$ 的 Jacobian"——这是 MPC 求解器在单个时间步上要的局部线性化。可微分仿真求的是"整条 rollout 轨迹的终态对初始控制序列的梯度"——这是把 $N$ 步动力学链式复合后端到端的导数，用于直接优化整段控制或训练策略。前者是"微观一步"，后者是"宏观全程"；前者服务于实时 MPC 的每帧线性化，后者服务于离线的轨迹优化与策略学习。

> **理论-工程桥接**：可微分仿真在数学上就是把 §48.2 的反向模式 AD 沿时间轴展开——把 $N$ 步动力学看成一个 $N$ 层的"时间网络"，对终态代价做一次跨越所有时间步的反向传播，就得到了对每一步控制的梯度。这与 RNN 的 BPTT（backpropagation through time）是同一个机制。区别仅在于"层"不是神经网络层，而是物理积分步。理解了反向 AD，可微分仿真就不再是一个新概念，而是同一工具在时间维度上的应用。

### GPU 上的 AD ⭐⭐⭐⭐

| 框架 | 特点 | 适用场景 |
|------|------|---------|
| NVIDIA Warp | Python + CUDA codegen，支持可微分仿真 | 变形体、布料仿真 |
| JAX XLA | JIT 编译到 GPU，`jax.grad` 自动微分 | RL 训练、MPC 研究 |
| Taichi | 可微分编程语言，自动并行化 | 物理仿真 + 梯度优化 |

机器人领域的常见模式：**训练用 Python/JAX（GPU），部署用 C++/CppADCodeGen（CPU 实时）**。这导致了"双语言问题"——训练和部署的动力学模型需要维护两套代码。

### 从 MPC 到学习：AD 的角色转变 ⭐⭐⭐

AD 在机器人学中的角色正在从"MPC 的求导工具"演变为贯穿研究方法论的基础设施。

**经典 MPC 流水线**（2015-2022）：$\text{URDF} \xrightarrow{\text{Pinocchio}} f(x,u) \xrightarrow{\text{CppADCodeGen}} \nabla f \xrightarrow{\text{SQP}} u^*$

**可微分 MPC**（2022-）：新目标是 $\frac{\partial u^*}{\partial \theta}$，其中 $\theta$ 为代价函数/约束参数。即不仅对动力学求导，还**对 MPC 求解器本身求导**——允许用真实数据自动调整 MPC 参数。

**关键工作**：
- Amos & Kolter 2017（OptNet）：对 QP 求解器求导
- de Avila Belbute-Peres et al. 2018：可微分物理引擎
- Howell et al. 2022（CALIPSO）：对 cone program 求导
- Aligator 2025：对 proximal DDP 求导

这个趋势意味着：**未来机器人工程师需要同时理解 AD 的"工具层"（CppAD/CodeGen 怎么用）和"方法层"（可微分优化/可微分仿真的原理）。** 假如去掉可微分 MPC 的能力，控制器参数只能靠人工手调或网格搜索——对于腿足机器人这种有数十个调节参数的系统，这在实践中几乎不可行。

> **本质洞察**：CppADCodeGen 的核心价值不仅仅是"加速求导"——它实现了从"数学公式"到"可部署机器码"的全自动桥梁。传统做法是数学家推导公式、工程师手写代码、测试者验证正确性，三个角色各走一遍。CodeGen 把这三步压缩为一步：公式即代码，代码即验证。这消除了"推导正确但实现错误"这一类在机器人控制中极难调试的 bug。

> **本质洞察**：前向 AD 和反向 AD 的选择，本质上是"沿输入方向扫描"还是"沿输出方向扫描"的对偶问题。这个对偶性与线性代数中"行空间 vs 列空间"的关系完全类比：前向模式逐列构建 Jacobian，反向模式逐行构建 Jacobian。选择哪种模式，取决于你的 Jacobian 是"扁的"（列多行少，选反向）还是"高的"（行多列少，选前向）。

### CodeGen 编译优化选项详解 ⭐⭐⭐

CppADCodeGen 生成的 C 源码需要通过 GCC/Clang 编译为共享库。编译选项的选择直接影响生成代码的执行速度和编译时间：

| 编译选项 | 效果 | 适用场景 | 编译时间影响 |
|---------|------|---------|-------------|
| `-O0` | 无优化 | 调试（保留所有变量符号） | 最快（秒级） |
| `-O2` | 标准优化 | **生产部署推荐** | 中等（分钟级） |
| `-O3` | 激进优化（含循环展开、向量化） | 高 DOF 模型追求极致性能 | 慢（可能数十分钟） |
| `-march=native` | 针对当前 CPU 指令集（AVX2/AVX-512） | 同一机器编译和运行时 | 无影响 |
| `-fPIC` | 位置无关代码 | **dlopen 加载时必需** | 无影响 |

OCS2 在处理高 DOF 模型（>20-DOF）时，将生成的 C 代码拆分为多个 `.c` 文件并行编译，避免单个巨大文件导致 GCC 的寄存器分配阶段卡住。拆分阈值通常设为每文件 10000 行。

### 与 CasADi 的简要对比 ⭐⭐

CasADi 是另一个在机器人 MPC 中广泛使用的自动微分框架。两者的定位有根本差异：

| 维度 | CppAD / CppADCodeGen | CasADi |
|------|---------------------|--------|
| **语言** | 纯 C++（header-only） | C++ 核心 + Python 前端 |
| **AD 方式** | Operator overloading（C++ 模板） | 符号计算（表达式图） |
| **与 Pinocchio** | 原生支持（`Scalar = AD<double>`） | 需要 pinocchio.casadi 桥接 |
| **代码生成** | 生成纯 C 代码 → GCC 编译 | 生成 C 代码 或 JIT |
| **NLP 求解** | 不含求解器（需搭配 IPOPT/HPIPM） | **内置 IPOPT/qpOASES 接口** |
| **符号简化** | 无（依赖编译器优化） | 有（CSE、常量折叠） |
| **典型用户** | OCS2（ETH RSL） | acados、MATMPC |
| **学习曲线** | 陡峭（C++ 模板元编程） | 较平缓（Python 交互式） |

**选型建议**：如果你的项目基于 OCS2 生态，CppADCodeGen 是唯一选择（OCS2 的 `CppAdInterface` 深度绑定）。如果你从零开始构建 MPC 且偏好 Python 开发，CasADi + acados 是更友好的组合。两者的数学等价性是完全的——差异仅在工程生态和开发体验上。

---

## 48.10 Python AD 生态：JAX/Diffrax/torch.func 在机器人中的应用趋势 ⭐⭐⭐

### JAX 生态与 CppAD 的定位差异 ⭐⭐⭐

CppAD/CppADCodeGen 是 C++ 世界的 AD 方案，核心优势是与 Pinocchio/OCS2 的零缝集成和微秒级预编译性能。但 2024-2026 年，Python 端的 AD 生态正在快速成熟，尤其是 JAX 生态，它在机器人学研究中的使用率急速上升。

| 维度 | CppAD/CodeGen | JAX (`jax.grad`/`jax.jacobian`) | PyTorch (`torch.func`) |
|------|-------------|-------------------------------|----------------------|
| **语言** | C++ | Python (XLA 编译) | Python (Dynamo 编译) |
| **AD 方式** | Tape + CodeGen | 函数式变换（`grad`/`jacfwd`/`jacrev`） | 函数式变换（`grad`/`jacrev`/`vmap`） |
| **GPU 支持** | 无 | 原生（XLA 后端） | 原生（CUDA 后端） |
| **批量处理** | 手动循环 | `vmap` 自动向量化 | `vmap` 自动向量化 |
| **实时部署** | 微秒级 `.so` | 毫秒级 JIT（首次编译慢） | 毫秒级 JIT |
| **典型用途** | OCS2 MPC 热路径 | MJX 可微仿真、Brax RL | RL 策略训练、可微控制研究 |

### JAX 在机器人 MPC/RL 中的崛起 ⭐⭐⭐

JAX 的 `jax.grad` + `jax.jit` + `jax.vmap` 三件套为机器人学提供了一种全新的开发范式：

- **`jax.grad`**：对任意 Python 函数自动求梯度，无需手动定义 tape 或标量类型替换
- **`jax.jit`**：将 Python 函数编译为 XLA 机器码，性能接近手写 C++
- **`jax.vmap`**：自动向量化——将"对单个样本的函数"变为"对批量样本的函数"，在 GPU 上并行执行

在腿足 RL 训练场景中，`vmap` 的威力尤其突出。传统做法是循环遍历 4096 个并行环境逐一计算动力学，`vmap` 则自动将循环编译为 GPU 并行指令——训练吞吐量提升 10-100 倍。

**Diffrax**（Patrick Kidger, 2022）是 JAX 生态中的微分方程求解器库，支持 ODE/SDE 的可微求解。对于腿足机器人，Diffrax 可以将连续时间动力学 $\dot{x} = f(x, u)$ 的数值积分过程（如 RK4）变为可微的——这意味着你可以直接对"从初始状态经过 N 步积分后的终态"求关于初始控制序列的梯度。这是可微 MPC（Differentiable MPC）的基础。

### torch.func：PyTorch 的函数式 AD 变换 ⭐⭐⭐

PyTorch 在 2.0 版本（2023）引入了 `torch.func` 模块，提供了类 JAX 的函数式 AD 变换：

- **`torch.func.jacrev`**：反向模式 Jacobian，适合 $m \gg n$（输出维度远小于输入维度）的场景
- **`torch.func.jacfwd`**：前向模式 Jacobian，适合 $n \gg m$（输入维度远小于输出维度）的场景
- **`torch.func.vmap`**：自动向量化，与 JAX 的 `vmap` 功能对等

对于已经深度使用 PyTorch 进行 RL 训练（如 rsl_rl、IsaacLab）的团队，`torch.func` 提供了不切换框架就能获得高效 AD 能力的途径。例如，在 RL 的奖励整形中，可以用 `jacrev` 计算策略输出关于观测的 Jacobian，作为正则化信号防止策略过于"激进"。

> **本质洞察**：CppAD/CodeGen 和 JAX/torch.func 解决的是同一个数学问题（自动微分），但面向完全不同的工程约束。CppAD/CodeGen 面向"微秒级实时 C++ 控制循环"，JAX/torch.func 面向"毫秒级 GPU 并行训练/优化"。前者是部署侧的选择，后者是研究侧的选择。未来的趋势是：研究阶段用 JAX/PyTorch 快速验证可微优化思路，部署阶段用 CppAD/CodeGen 生成实时可执行代码。

### 双语言问题与一致性陷阱 ⭐⭐⭐

上一条洞察点出了"研究用 Python、部署用 C++"的趋势，但这条趋势带来一个被严重低估的工程隐患——**双语言问题**（two-language problem）：同一个机器人动力学/代价模型，被用两套语言、两套 AD 框架各实现了一遍。训练侧用 JAX/PyTorch 写一份，部署侧用 Pinocchio+CppAD 写一份。两份代码在数学上"应该"等价，但在工程上几乎不可能自动保持一致。

为什么这是个大问题？把不一致的来源系统性地列出来（系统性分类，而非随意举例）：

| 不一致维度 | 具体表现 | 后果 |
|:---|:---|:---|
| **约定差异** | 四元数 $[w,x,y,z]$ vs $[x,y,z,w]$；广义力是否含浮基 6 维 | 数值整体错位，难察觉 |
| **参数来源** | 训练用一份 URDF，部署用另一份（连杆惯量微调过） | 模型不匹配，sim-to-real 间隙被人为放大 |
| **数值细节** | 接触模型的光滑参数 $\alpha$、积分步长、重力常数取值 | 动力学轨迹缓慢发散 |
| **AD 语义** | JAX 对 `where` 分支天然可微，CppAD 需 `CondExp` | 同一分支两侧导数定义不同 |

> **对比性思维：双语言问题不是"代码重复"那么简单，而是"真理来源分裂"。** 普通代码重复，复制粘贴的两份至少字面一致；双语言问题里，两份代码连语言和框架都不同，没有任何机制保证它们编码的是同一个数学对象。当部署的机器人行为与训练时的仿真不符，你甚至无法判断是"模型不一致"还是"sim-to-real 间隙"——两个原因纠缠在一起，调试成本急剧上升。

应对这个问题，工程上有三条主流思路，各有取舍：

1. **单一真理源 + 双后端**：用一份与语言无关的模型描述（URDF/MJCF）作为唯一真理源，两侧都从它生成代码。Pinocchio 从 URDF 生成 C++，MuJoCo/MJX 从 MJCF 生成 JAX——只要保证 URDF 与 MJCF 由同一工具转换，约定就能对齐。
2. **跨语言桥接**：把 C++ 动力学（Pinocchio）通过绑定暴露给 Python（如 `pinocchio` 的 Python 接口），训练时直接调用与部署相同的实现。代价是失去 GPU 批量并行。
3. **一致性回归测试**：承认两份代码独立存在，但用自动化测试在大量随机状态上比较两侧的函数值与 Jacobian，把"等价性"变成持续验证的断言而非假设。

> **本质洞察**：双语言问题的根源，是"研究侧要 GPU 批量可微、部署侧要 CPU 微秒实时"这两个硬约束目前还无法被单一框架同时满足。CppADCodeGen 和 JAX 各自把一端做到了极致，却在中间留下了一道需要工程纪律来守护的缝。这也是为什么 §48.8 提到的 Enzyme（LLVM 级 AD）令人期待——如果"对原始 `double` 代码直接微分"成熟，理论上同一份 C++ 动力学既能编进实时 `.so`，又能被微分用于离线优化，双语言问题才有望从根上消解。

---

## 本章常见误解汇总

学完全章，把最容易在 AD 流水线上栽跟头的认知误区集中澄清一次。这些误解大多源于把"普通数值计算的直觉"错误地套用到"符号 tape"上。

| 误解 | 为什么是错的 | 正确理解 |
|:---|:---|:---|
| "tape 存的是这次计算的数值结果" | tape 存的是计算**结构**（哪些运算按什么顺序连接），不是具体数值 | 同一个 `ADFun` 可在任意新输入点反复求值/求导；录制时的初始值只决定分支走向 |
| "`std::sin` 和 `CppAD::sin` 只是命名空间不同" | `std::sin` 不绑定 AD 重载，会迫使你 `Value()` 取数，从而脱离 tape | 录制期间所有数学函数必须用 `CppAD::` 版本，否则导数静默丢项 |
| "用 `if (ax[0] > 0)` 写分支只是不够优雅" | tape 只录制录制时选中的那一支，换点求值得到完全错误的导数 | 依赖 AD 值的分支必须用 `CondExpGt` 等条件表达式，两支都进 tape |
| "CppADCodeGen 一定比 Pinocchio 解析导数快" | CodeGen 展平后丢失了刚体动力学的递推结构信息 | 标准刚体动力学导数，Pinocchio 解析导数通常最快；CodeGen 赢在自定义函数和正确性保证 |
| "生成代码越大说明模型越复杂" | 代码体积正比于优化后 DAG 节点数，CSE 失效会让简单模型也膨胀 | 先确认 `optimize()` 已调用、无重复子表达式，再谈模型复杂度 |
| "前向模式总比反向模式慢" | 两者代价分别是 $O(n\,C_f)$ 与 $O(m\,C_f)$，取决于 Jacobian 形状 | 宽输出（$m\gg n$）用前向，宽输入（$n\gg m$）用反向；标量输出必用反向 |
| "代码生成是运行时开销" | 生成 + 编译是**离线一次性**的，运行时只剩 `dlopen` 和原生调用 | 把编译成本算进部署准备阶段，热路径上是零解释开销 |
| "`.so` 编译出来就能到处部署" | `.so` 是平台特定 ELF，`-march=native` 还绑定具体微架构 | 跨平台须在目标平台编译或交叉编译；嵌入式 ARM 不能直接用 x86 的 `.so` |

---

## 研究实践建议

按学习者所处阶段，给出分层的实践路线，避免一上来就陷入大型模型的编译噩梦。

**入门阶段（建立直觉，1-2 天）**

- 从标量函数 $f(x) = x^3 \sin(x)$ 开始，完整走一遍 `Independent → ADFun → ModelCSourceGen → GCC → dlopen` 五步，亲手打开 `/tmp` 下生成的 `.c` 文件读一遍。这一步的目标不是性能，而是**看见**"公式如何变成 C 代码"。
- 立刻用数值差分对生成代码做梯度检查，养成"AD 结果必须与有限差分交叉验证"的肌肉记忆——这个习惯会在后续救你无数次。

**进阶阶段（吃透机制，1 周）**

- 对一个 3-DOF 平面臂分别实现三条路线（数值差分 / CppAD 解释 / CppADCodeGen），亲手测出微秒级耗时差距，把 §48.1 和 §48.5 的性能表"复现"出来——亲眼看到的数据比读到的更可信。
- 刻意制造三类典型 bug 各一次（忘记 `Independent`、误用 `Value()`、用 `if` 取代 `CondExp`），观察它们各自的错误现象，这样真遇到时能凭症状反查原因。

**工程阶段（接入实战，持续）**

- 把动力学预编译模块按累积项目的三阶段接入 Go2，并从一开始就设计**库的版本化命名**（把 DOF、关键参数、生成日期编进 `modelName`），从源头杜绝"加载到旧库"这个高频坑。
- 在引入新模型或修改 URDF 后，固定执行一次"删除缓存 → 重新生成 → 数值梯度检查 → 性能基准"四步流程，把它写成脚本，纳入 CI。
- 对性能敏感的标准动力学路径，做一次"CodeGen vs Pinocchio 解析导数"的实测对比再决定方案，不要凭"代码生成一定快"的直觉下结论。

> **本质洞察**：AD 流水线的工程难点从来不在"怎么求导"——这一步库已经替你做好了。真正消耗工程师时间的是**两类边界问题**：tape 与外部世界的边界（脱链、分支、平台），以及缓存与真实模型的边界（旧库、维度、参数变化）。把精力投在守住这两条边界上，而非反复怀疑导数公式本身，是高效调试 AD 系统的关键心法。

---

## 本章小结

### 核心概念回顾 ⭐

| 概念 | 关键要点 |
|------|---------|
| 前向 AD | dual number 算术，一次扫描得一个方向导数，Ceres `Jet<T,N>` |
| 反向 AD | 伴随方法 (adjoint)，一次回扫得所有输入的偏导，backpropagation |
| CppAD tape | 记录计算图，可前向/反向重放，`ADFun` 封装 |
| CppADCodeGen | tape $\to$ C 源码 $\to$ `.so`，运行时零解释开销 |
| Pinocchio + CG | `ModelTpl<ADCGScalar>` 实现 URDF $\to$ `.so` 的完整流水线 |
| OCS2 CppAdInterface | hash 缓存 + `thread_local` 线程安全 + SQP 无缝集成 |
| Crocoddyl / Aligator | 解析导数优先，CodeGen 为辅，并行 Riccati 求解 |
| 选型三角 | 解释执行（原型）/ CodeGen（生产）/ 解析导数（最快） |

把全章一句话收束：**本章教的不是"如何求导"，而是"如何把求导这件事的成本，从每帧都付的运行期，搬到只付一次的编译期"。** 前向/反向 AD（§48.2）回答了"凭什么能得到机器精度的导数"，tape（§48.3）回答了"如何把计算图捕获下来"，CodeGen（§48.4）回答了"如何把计算图编译成零开销机器码"，Pinocchio 集成（§48.5）把工具用到真实动力学上，OCS2/Crocoddyl/Aligator（§48.6-48.7）展示了工业界在"全部 CodeGen / 全部手写 / 混合"之间的不同抉择，选型与前沿（§48.8-48.10）则把这套工具放回更大的"研究侧 GPU 可微 vs 部署侧 CPU 实时"的版图中定位。读完全章，你应当能在面对一个新的 MPC/WBC 项目时，不靠直觉而靠 $T^*$ 会计学和 Jacobian 形状判据，做出有依据的 AD 方案决策。

### 学习时间估算 ⭐

**1.5 周（25-30 小时）**：

| 内容 | 建议时间 | 对应小节 |
|------|---------|---------|
| AD 原理（dual number、前向/反向模式、录制点无关性） | 3-4 小时 | §48.2 |
| CppAD 基础（tape 概念、`ADFun` API、稀疏 Jacobian） | 4-5 小时 | §48.3 |
| 符号 tape 内部机制（CSE/DCE、原子函数、稀疏 Hessian） | 3-4 小时 | §48.4.5 |
| CppADCodeGen 流水线（代码生成、编译选项、CMake 集成） | 4-5 小时 | §48.4 |
| Pinocchio + CppAD 集成实战（含条件分支三解法） | 5-6 小时 | §48.5 |
| OCS2 `CppAdInterface` 源码阅读 + 实战调试流程 | 4-5 小时 | §48.6 |
| Crocoddyl / Aligator 对比 + 选型/前沿 | 2-3 小时 | §48.7-48.10 |
| 实战练习 + 性能基准 + 累积项目模块 | 4-5 小时 | 全章 |

### 下游章节导引 ⭐

- **$\to$ 足式/90_WBC分层优化与TSID WBC**：全身控制的实时 QP 约束 Jacobian 来自 CppADCodeGen
- **$\to$ 足式/110_OCS2完整栈与双线程MPC OCS2 MPC**：完整 MPC 栈依赖 `CppAdInterface` 提供动力学导数
- **$\to$ 足式/100_DDP家族与Crocoddyl Crocoddyl / Aligator**：DDP 族轨迹优化器的导数来源与方案选择
- **$\to$ S05 可微分仿真 MPC**：AD 从"求导工具"升级为"研究方法论"

---

## 累积项目：四足控制器——CppADCodeGen 动力学导数预编译模块

本章在 足式/30_Pinocchio深度精读 的累积项目基础上，添加**动力学导数的预编译模块**。足式/30_Pinocchio深度精读 完成了 URDF 加载和 FK/RNEA 的 `double` 版本调用；本章将同一套动力学函数通过 CppADCodeGen 导出为高性能 `.so`，供后续 MPC/WBC 实时调用。

**阶段 1：独立函数的 CodeGen 练习**
- 选一个简单的标量函数（如 $f(x) = x^3 \sin(x)$），走通 CppAD tape 录制 → CppADCodeGen C 源码生成 → GCC 编译 → `dlopen` 加载 → 调用的完整流程
- 用数值差分验证生成代码的 Jacobian 精度（误差 $< 10^{-10}$）

**阶段 2：Pinocchio 动力学导数的预编译**
- 用 `pinocchio::ModelTpl<CppAD::cg::CG<double>>` 实例化 Go2 模型
- 录制 RNEA 的 tape，生成 $\partial\tau/\partial q$、$\partial\tau/\partial v$、$\partial\tau/\partial a$ 的 C 源码
- 编译为 `libgo2_dynamics_derivatives.so`，封装为 `PrecompiledDynamics` 类，提供 `getJacobian(q, v, a)` 接口

**阶段 3：性能基准与集成**
- 对比三种方案的单次调用耗时：数值差分、CppAD 解释执行、CodeGen 预编译
- 将 `PrecompiledDynamics` 模块集成到 足式/30_Pinocchio深度精读 的 `RobotModel` 框架中，确保多线程安全（每个线程持有独立的 `dlopen` 句柄或使用 `thread_local`）

> **后续扩展**：足式/90_WBC分层优化与TSID 的 WBC 将调用本模块提供的预编译 Jacobian 构建 QP 约束矩阵；足式/110_OCS2完整栈与双线程MPC 的 OCS2 MPC 将通过 `CppAdInterface` 以相同机制获取动力学导数。

**本章模块的验收标准**（达到即可进入下游章节）：

| 验收项 | 合格判据 | 对应小节 |
|:---|:---|:---|
| 精度 | 预编译 Jacobian 与中心差分在多个随机位型上相对误差 $< 10^{-9}$ | §48.1, §48.5 |
| 性能 | Go2 RNEA+Jacobian 单次调用 $< 3\,\mu$s（$-O2$，单核） | §48.5 |
| 鲁棒性 | 含接触切换的位型用 `CondExp` 后导数无跳变 | §48.5 |
| 线程安全 | 双线程并发调用 1 小时无数据竞争（`thread_local` 隔离 buffer） | §48.6 |
| 可重建 | 删缓存后一条命令重新生成 `.so`，模型名编码了 DOF/版本 | §48.4, §48.6 |

**跨章综合练习**：把本章的 `PrecompiledDynamics` 与 足式/30_Pinocchio深度精读 的 `RobotModel`、以及前置的状态估计接口串成一个最小闭环——从一帧关节传感器读数构造状态向量 $x = [q; v; a]$，调用预编译 `.so` 得到 $\tau$ 与 $\partial\tau/\partial x$，再把 Jacobian 喂给一个手写的单步 QP（最小化 $\|\tau - \tau_{\text{ref}}\|^2$ 受关节力矩限幅约束）。要求：(a) 全链路单次耗时 $< 50\,\mu$s；(b) 用数值差分验证喂给 QP 的 Jacobian 正确；(c) 故意改一次 URDF 连杆质量，验证你的版本化命名机制确实触发了 `.so` 重新生成而非加载旧库。这道题综合了本章（CodeGen）、Pinocchio 章（模型加载）和优化章（QP 构造）三章的知识。

---

## 🔧 故障排查手册

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| `ADFun` 构造时报错 "zero order forward result has wrong size" | 忘记调用 `CppAD::Independent(ax)` 就开始计算，tape 为空 | 检查代码中 `Independent()` 是否在所有 AD 运算之前调用；确认 `ax` 的元素确实参与了 `ay` 的计算 |
| Jacobian 中某些偏导数恒为零（实际不应为零） | tape 录制期间调用了 `CppAD::Value()` 提取 double 值，导致后续运算脱离 tape 追踪 | 全文搜索 `Value(`，确认仅在调试打印中使用；所有参与 `ay` 计算的变量必须保持 `AD<double>` 类型 |
| CodeGen 编译的 `.so` 在目标平台上 `dlopen` 失败（SIGILL 或 "illegal instruction"） | 编译时使用了 `-march=native`，生成了宿主 CPU 的专用指令（如 AVX-512），目标平台不支持 | 去掉 `-march=native`，改用 `-march=x86-64-v2`（兼容大多数现代 x86）；或在目标平台上重新编译 |
| CppADCodeGen 生成 C 代码后 GCC 编译超过 10 分钟未完成 | 高 DOF 模型（>20-DOF）生成的 C 文件过大（>30000 行），GCC 寄存器分配阶段卡顿 | 将优化级别从 `-O3` 降至 `-O2`；或将生成代码拆分为多个 `.c` 文件并行编译（OCS2 的做法） |
| 条件分支处的 Jacobian 在分支边界附近出现跳变或错误值 | tape 中使用了 `if(ax > 0)` 而非 `CppAD::CondExpGt()`，tape 只录制了初始值选中的分支 | 将所有依赖 AD 变量值的 `if/else` 替换为 `CondExpGt` / `CondExpLt` 等条件表达式 |
| `dlopen` 成功但调用 `model->Jacobian(x)` 抛 "input vector size mismatch" | 加载的 `.so` 是用旧的输入维度（如改 URDF 前的 $n_q$）编译的，运行时传入了新维度的 `x` | 删除缓存目录下的旧 `.so` 强制重新生成；或在 `modelName` 中编码维度/版本（如 `go2_dyn_nv18_v2`），让维度变化自动触发新库 |
| 生成的 `.so` 体积异常巨大、编译奇慢，但 DOF 并不高 | 录制时 CSE 失效（如循环内反复 `cast` 常量、未调用 `optimize()`），DAG 节点数爆炸 | 确认 `ad_fun.optimize("no_compare_op")` 已调用；检查是否在 tape 内重复构造相同常量；把不变量提到 `Independent()` 之前用 `double` 算好 |
| RNEA 的 Jacobian 数值对但比 Pinocchio 解析导数慢 2-3 倍 | CodeGen 展平了 RNEA 的递推结构，丢失了 `computeRNEADerivatives` 利用的稀疏递推 | 这是预期行为而非 bug；若该路径是瓶颈，改用 Pinocchio 解析导数，或用原子函数包装解析块（见 §48.4.5） |

> **排查总则**：AD 流水线的 bug 几乎都属于两类——"导数恒为零/缺项"（多半是脱离 tape：忘记 `Independent`、误用 `Value()`、误用 `std::sin`），或"导数在某点错"（多半是分支问题：`if` 取代了 `CondExp`）。遇到导数异常，第一步永远是**用数值差分做一次全量梯度检查**：在多个随机点上比较 CodeGen 导数与中心差分，定位出错的具体行和列，再回到对应的录制代码排查。

---

## API 速查表

本章涉及的核心 API 按"录制 → 求值/求导 → 代码生成 → 运行时调用"四个阶段分组。签名以 CppAD 1.x / CppADCodeGen 当前版本为准，省略部分模板参数以突出语义。

**阶段一：tape 录制**

| API | 作用 | 关键约束 |
|:---|:---|:---|
| `CppAD::Independent(ax)` | 标记 `ax` 为自变量，开启全局 tape 录制 | 必须在所有 AD 运算之前调用 |
| `CppAD::ADFun<Base> f(ax, ay)` | 结束录制，把 tape 密封为可复用的函数对象 | `ax`/`ay` 须为录制起止的自/因变量 |
| `f.Dependent(ax, ay)` | 显式指定因变量（与构造函数等价的两段式写法） | 用于先默认构造再绑定的场景 |
| `f.optimize("no_compare_op")` | 对 tape 做 CSE + DCE，缩小规模 | CodeGen 路径推荐；移除比较运算 |
| `CppAD::CondExpGt(l, r, t, f)` | 可微条件：`l>r ? t : f`，两分支都进 tape | 替代依赖 AD 值的 `if`；另有 `Lt/Le/Ge/Eq` |

**阶段二：求值与求导**

| API | 作用 | 复杂度 |
|:---|:---|:---|
| `f.Forward(0, x)` | 零阶前向 = 在点 `x` 求函数值，并设定后续求导的工作点 | $O(C_f)$ |
| `f.Forward(1, dx)` | 一阶前向 = 方向导数 $J\,dx$（Jacobian 的一列） | $O(C_f)$，须先 `Forward(0,·)` |
| `f.Reverse(1, w)` | 一阶反向 = $w^T J$（Jacobian 的一行）；标量输出取 `w={1}` 得全梯度 | $O(C_f)$ |
| `f.Jacobian(x)` | 稠密 Jacobian（自动选模式），行主序展平 | $O(\min(n,m)\,C_f)$ |
| `f.Hessian(x, w)` | $w^T f$ 的稠密 Hessian | $O(n\,C_f)$ |
| `f.for_jac_sparsity(...)` | 检测 Jacobian 稀疏模式 | 一次性 |
| `f.sparse_jac_rev(x, pat, color, jac, work)` | 用稀疏模式 + 图着色算 Jacobian | $\propto$ 着色数 |
| `f.sparse_hes(x, w, pat, "cppad.symmetric", hes, work)` | 用对称着色算稀疏 Hessian | $\propto$ 着色数 |

**阶段三：代码生成（CppADCodeGen）**

| API | 作用 |
|:---|:---|
| `cg::ModelCSourceGen<double> cgen(adFun, "name")` | 由 `ADFun<CG<double>>` 创建 C 源码生成器 |
| `cgen.setCreateForwardZero(true)` | 生成函数值（ForwardZero）代码 |
| `cgen.setCreateJacobian(true)` / `setCreateSparseJacobian(true)` | 生成稠密 / 稀疏 Jacobian 代码 |
| `cgen.setCreateHessian(true)` | 生成 Hessian 代码 |
| `cg::ModelLibraryCSourceGen<double> libcgen(cgen)` | 把单模型源码包装为库级源码 |
| `cg::GccCompiler<double> compiler` | GCC 编译器封装；`addCompileFlag("-O2")` 等加编译选项 |
| `cg::DynamicModelLibraryProcessor<double> p(libcgen)` | 驱动 "源码 → `.so` → `dlopen`" 全过程 |
| `p.createDynamicLibrary(compiler)` | 执行编译并返回已加载的动态库句柄 |

**阶段四：运行时调用**

| API | 作用 |
|:---|:---|
| `cg::LinuxDynamicLib<double> lib("path/to/x.so")` | 直接加载磁盘上已编译的 `.so` |
| `lib.model("name")` | 取出某个模型的可调用句柄（`GenericModel`） |
| `model->ForwardZero(x)` | 调用预编译的函数值代码 |
| `model->Jacobian(x)` | 调用预编译的稠密 Jacobian 代码 |
| `model->SparseJacobian(x, vals, rows, cols)` | 调用预编译的稀疏 Jacobian 代码 |

---

## 版本信息速查

| 组件 | 本章假设版本 | 备注 |
|:---|:---|:---|
| CppAD | 2024+（coin-or/CppAD） | header-only；`sparse_jac_rev`/`sparse_hes` 接口为新式稀疏 API |
| CppADCodeGen | 2.4+ | 依赖 CppAD；`ModelCSourceGen` / `DynamicModelLibraryProcessor` 接口稳定 |
| Pinocchio | 3.x | `codegen/cppadcg.hpp` 提供 CG 标量支持；`cast<Scalar>()` 切换标量类型 |
| OCS2 | 维护模式 v1.0 | `CppAdInterface` 封装；磁盘缓存按 `folderName/modelName` 组织 |
| Crocoddyl | 3.x | API 与 2.x 不兼容；解析导数优先 |
| Aligator (ProxDDP) | 活跃开发 | Parallel Riccati；解析 + 选择性 CodeGen 混合 |
| GCC / Clang | 9+ / 12+ | 需支持 `-fPIC` `-O2` `-march=native`；高 DOF 建议 `make -j` |

---

## 延伸阅读

| 资料 | 类型 | 难度 | 说明 |
|------|------|------|------|
| Griewank & Walther, *Evaluating Derivatives: Principles and Techniques of Algorithmic Differentiation*, 2nd ed., SIAM 2008 | 教材 | ⭐⭐⭐ | 自动微分领域的权威教材。系统讲解前向模式、反向模式、稀疏性利用和检查点策略，是理解 CppAD tape 机制的理论根基 |
| CppAD 官方文档（[coin-or.github.io/CppAD/doc/](https://coin-or.github.io/CppAD/doc/)） | 文档 | ⭐⭐ | Header-only 库的完整 API 参考。重点关注 `ADFun` 类、`Independent()`、`Jacobian()`/`Hessian()` 接口和 tape 优化选项 |
| Nocedal & Wright, *Numerical Optimization*, 2nd ed., Springer 2006, 附录 B | 教材 | ⭐⭐⭐ | 附录 B 简明介绍了 AD 在数值优化中的角色。将 AD 放在"优化器需要什么样的导数"这个工程语境下讨论，与本章的 MPC 热路径动机高度吻合 |
