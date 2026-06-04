# Lyapunov 稳定性理论

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回前置章节复习）

1. 什么是微分方程 $\dot x = f(x)$ 的平衡点？如何判断平衡点的存在性？
2. 矩阵特征值的实部与线性系统 $\dot x = Ax$ 解的渐近行为有何关系？
3. 什么是正定矩阵？如何判断一个对称矩阵是否正定？
4. 连续函数在紧集上的极值定理（Weierstrass 定理）说了什么？
5. Lipschitz 条件与 ODE 解的存在唯一性（Picard-Lindelof 定理）有何关系？

---

## 本章目标

学完本章后，你将能够：

1. **严格表述**三级稳定性（Lyapunov 稳定、渐近稳定、指数稳定）的 $\varepsilon$-$\delta$ 定义与比较函数等价刻画，并在具体系统上判断属于哪一级。
2. **完整证明** Lyapunov 直接法的稳定性/渐近稳定性定理，理解证明中每一步的物理直觉与数学必要性。
3. **运用 LaSalle 不变集原理**处理 $\dot V$ 仅半负定的实际工程系统（如阻尼机械系统、机器人关节 PD 控制）。
4. **系统构造** Lyapunov 函数：掌握能量法、二次型 + Lyapunov 方程、Krasovskii 法、变量梯度法、SOS 多项式搜索等多种方法。
5. **使用 ISS 框架**分析带外部扰动的非线性系统鲁棒性，并应用小增益定理证明级联系统稳定性。
6. **理解逆定理**（Massera, Kurzweil）的哲学意义：稳定性与 Lyapunov 函数存在性的完全等价。
7. **设计基于 Lyapunov 的控制器**：反步法（Backstepping）递归设计与机器人 PD 控制稳定性证明。

---

## 知识树

```
Lyapunov 稳定性理论
├── 1. 基础概念层
│   ├── 平衡点与自治系统
│   ├── 三级稳定性定义（S / AS / ES）
│   └── 比较函数体系（class K / K∞ / KL）
├── 2. 核心定理层
│   ├── Lyapunov 直接法（稳定 / 渐近稳定 / 全局）
│   ├── LaSalle 不变集原理
│   ├── 指数稳定与 Lyapunov 方程
│   ├── Chetaev 不稳定性定理
│   └── 间接法（线性化判据）
├── 3. 构造方法层
│   ├── 能量法（物理直觉）
│   ├── 二次型 V = x'Px + Lyapunov 方程
│   ├── Krasovskii 法与变量梯度法
│   ├── SOS 多项式搜索
│   └── 学习驱动（Neural Lyapunov）
├── 4. 鲁棒性与互联层
│   ├── ISS（Input-to-State Stability）
│   ├── 小增益定理
│   ├── 级联系统稳定性
│   └── iISS 与实际稳定性
├── 5. 逆定理与完备性
│   ├── Massera 逆定理
│   ├── Kurzweil 逆定理
│   └── 哲学意义：Lyapunov 是"充要"工具
├── 6. 控制设计应用
│   ├── 反步法（Backstepping）
│   ├── 机器人 PD 控制稳定性
│   └── CLF 与 CLF-QP（预告）
└── 7. 典型例题
    ├── 单摆 + 阻尼
    ├── Van der Pol 振子
    └── 机器人关节 PD 控制
```

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含证明和构造练习） | 10-12 小时 | 需要掌握 Lyapunov 函数构造和 ISS 框架的读者 |
| 速读（跳过逆定理和 SOS 细节） | 4-5 小时 | 已有线性系统稳定性基础、需要快速理解非线性判稳方法的读者 |
| 速查（只看定理条件和构造方法表） | 30 分钟 | 遇到具体稳定性分析问题时回来查阅定理条件 |

---

## 7.1 为什么 Lyapunov 稳定性是非线性控制的数学根基 ⭐

### 动机：一个无法回避的问题

假设你设计了一个机器人控制器，让四足机器人在不平地面上行走。你用 MPC 规划了步态，用 WBC 分配了关节力矩。一切在仿真中完美运行。但是——**你怎么证明这个闭环系统不会突然发散？** 怎么证明一个微小的扰动（比如踩到一块石头）不会让机器人摔倒？

这就是**稳定性分析**的核心问题。对线性系统 $\dot x = Ax$，答案很简单：看 $A$ 的特征值是否都在左半平面。但对非线性系统 $\dot x = f(x)$，特征值概念不再适用——系统行为随状态而变化，不同区域可能有完全不同的定性特征。

### 如果没有 Lyapunov 方法会怎样

在 1892 年 Lyapunov 发表博士论文之前，分析非线性系统稳定性的唯一方法是**显式求解微分方程**，然后验证解趋于零。但绝大多数非线性 ODE 没有闭式解！这意味着：

- 单摆方程 $\ddot\theta + \sin\theta = 0$ 无法用初等函数求解（需要椭圆函数）
- 任何含有摩擦、柔性、接触的机器人动力学都不可能解析求解
- 你永远无法"算出"一个复杂系统的轨迹然后验证它收敛

> **本质洞察**：Lyapunov 方法的革命性在于**不需要解微分方程**。它把"求解"问题转化为"构造"问题——找到一个能量函数 $V$ 满足适当条件即可。对非线性系统，这通常意味着从"不可能分析"变为"可以分析"。

### 历史脉络

Lyapunov 的博士论文 "The General Problem of the Stability of Motion"（1892 年，俄文原文发表于 Kharkov 大学）提出了两种方法：

- **第一方法（间接法）**：通过线性化在平衡点附近判断稳定性
- **第二方法（直接法）**：构造能量函数直接判断，无需求解方程

直接法才是真正的突破。它的思想如此朴素：**如果系统有一个"能量"，且这个能量沿轨迹只减不增，那么系统必然趋向能量最低点**。这一思想自提出以来，经历了三次重大跃迁：

| 年代 | 突破 | 意义 |
|------|------|------|
| 1892 | Lyapunov 直接法 | 奠基——无需解方程即可判稳 |
| 1960 | LaSalle 不变集原理 | 救命——$\dot V \le 0$ 半负定也能判渐近稳定 |
| 1989 | Sontag ISS 理论 | 统一——鲁棒性语言的非线性版本 |
| 2000 | Parrilo SOS 编程 | 计算——Lyapunov 函数搜索凸化为 SDP |
| 2019 | Chang-Roohi-Gao Neural Lyapunov | 学习——神经网络 + SMT 验证 |

### 本专题在机器人控制中的三重定位

**第一，它是非线性控制的几何直觉载体**。$V$ 的 level set（等值面）就是"能量等值面"，$\dot V < 0$ 就是轨迹从外层向内层跨越。这种几何叙事贯穿从 LQR 到 MPC 的所有现代控制理论。

**第二，它是"证书"（certificate）思想的原型**。现代 safe RL 论文中的 "barrier certificate""stability certificate""Lyapunov network" 本质都是在把 Lyapunov 条件作为神经网络的学习目标或 SOS 的验证目标。

**第三，它是前沿计算的主战场**。从 Parrilo 2000 的 SOS 到 dReal 的 SMT，从 Neural Lyapunov 到 Contraction Theory，Lyapunov 在过去 25 年被彻底"计算化"。

---

## 7.2 比较函数体系：class-$\mathcal{K}$, $\mathcal{K}_\infty$, $\mathcal{KL}$ ⭐⭐

### 动机：为什么需要比较函数

经典的 $\varepsilon$-$\delta$ 定义虽然严谨，但在实际使用中极其繁琐。每次要证明一个新性质，都需要重新摆弄 $\varepsilon$ 和 $\delta$ 的关系。Hahn 1967 引入比较函数类，目的是**把稳定性条件统一编码为函数不等式**，让不同层级的稳定性只需要改动一两个条件即可区分。

这就像编程中的类型系统：你不需要每次都从位级别操作数据，而是用 int、float、string 等类型抽象。比较函数就是稳定性理论的"类型系统"。

### 三类比较函数的严格定义

**定义（class-$\mathcal{K}$ 函数）**：函数 $\alpha:[0,a) \to [0,\infty)$（$a > 0$ 可为 $\infty$）称为 class-$\mathcal{K}$ 函数，若满足：
1. $\alpha$ 在 $[0,a)$ 上连续
2. $\alpha(0) = 0$
3. $\alpha$ 严格递增

**直觉**：class-$\mathcal{K}$ 函数是"从原点出发、严格上升的曲线"。常见例子：$\alpha(r) = r$, $\alpha(r) = r^2$, $\alpha(r) = \sqrt{r}$, $\alpha(r) = \tan(r)$（$a = \pi/2$）。

**定义（class-$\mathcal{K}_\infty$ 函数）**：$\alpha:[0,\infty) \to [0,\infty)$ 是 class-$\mathcal{K}_\infty$ 函数，若：
1. $\alpha \in \mathcal{K}$（即连续、$\alpha(0)=0$、严格递增）
2. 定义域为 $[0,\infty)$（即 $a = \infty$）
3. $\lim_{r \to \infty} \alpha(r) = \infty$（无界）

**与 $\mathcal{K}$ 的区别**：$\mathcal{K}_\infty$ 额外要求定义域延伸到无穷且函数值也趋向无穷。$\alpha(r) = r/(1+r)$ 属于 $\mathcal{K}$ 但不属于 $\mathcal{K}_\infty$（因为有上界 1）。

**定义（class-$\mathcal{KL}$ 函数）**：$\beta:[0,\infty) \times [0,\infty) \to [0,\infty)$ 是 class-$\mathcal{KL}$ 函数，若：
1. 对固定 $s \ge 0$，$\beta(\cdot, s) \in \mathcal{K}$（关于第一个变量是 class-$\mathcal{K}$）
2. 对固定 $r > 0$，$\beta(r, \cdot)$ 关于第二个变量严格递减
3. $\lim_{s \to \infty} \beta(r, s) = 0$ 对所有 $r > 0$

**直觉**：$\mathcal{KL}$ 函数同时编码了两个物理信息——"初始偏差越大，响应越大"（$\mathcal{K}$ 性质）和"随时间衰减到零"（L 性质）。典型例子：$\beta(r, s) = r \cdot e^{-s}$。

### 比较函数的代数性质

| 性质 | 表述 | 意义 |
|------|------|------|
| 复合封闭 | $\alpha_1, \alpha_2 \in \mathcal{K} \Rightarrow \alpha_1 \circ \alpha_2 \in \mathcal{K}$ | 可以"串联"两个 $\mathcal{K}$ 函数 |
| 逆存在 | $\alpha \in \mathcal{K}_\infty \Rightarrow \alpha^{-1} \in \mathcal{K}_\infty$ | $\mathcal{K}_\infty$ 有逆，$\mathcal{K}$ 一般没有 |
| 求和封闭 | $\alpha_1, \alpha_2 \in \mathcal{K} \Rightarrow \alpha_1 + \alpha_2 \in \mathcal{K}$ | 可以叠加 |
| 夹逼引理 | $\alpha_1(\|x\|) \le V(x) \le \alpha_2(\|x\|)$ 当 $\alpha_1, \alpha_2 \in \mathcal{K}_\infty$ | $V$ 与 $\|x\|$ 等价，保证 radially unbounded |

### 稳定性的比较函数等价刻画

用比较函数重写三级稳定性，消除所有 $\varepsilon$-$\delta$：

| 稳定性级别 | 比较函数刻画 | 物理含义 |
|------------|-------------|----------|
| Lyapunov 稳定 | $\|x(t)\| \le \alpha(\|x(0)\|)$, $\alpha \in \mathcal{K}$ | 初值小则轨迹永远小 |
| 渐近稳定 | $\|x(t)\| \le \beta(\|x(0)\|, t)$, $\beta \in \mathcal{KL}$ | 初值小且随时间衰减 |
| 指数稳定 | $\|x(t)\| \le k\|x(0)\| e^{-\lambda t}$, $k,\lambda > 0$ | 以指数速率衰减 |
| GAS | 同 AS 但对所有 $x(0) \in \mathbb{R}^n$ | 全域吸引 |

> **本质洞察**：$\mathcal{KL}$ 函数把"随时间衰减到零"与"关于初值单调"两个物理直觉打包成一个函数，是现代非线性控制的**通用货币**。所有进阶概念（ISS、iISS、ISpS、practical stability）都是在这套语言上的微调。

### 常见陷阱

⚠️ **概念误区：认为 class-$\mathcal{K}$ 必须无界**

新手常把 $\mathcal{K}$ 和 $\mathcal{K}_\infty$ 混淆。$\alpha(r) = 1 - e^{-r}$ 是 class-$\mathcal{K}$（严格递增、$\alpha(0)=0$、连续）但有上界 1，不属于 $\mathcal{K}_\infty$。只有 $\mathcal{K}_\infty$ 才能保证 Lyapunov 函数的 radially unbounded 性质。

⚠️ **思维陷阱：认为指数稳定一定是"最好的"**

指数稳定给出 $e^{-\lambda t}$ 衰减率，看似最优。但有的系统可以 finite-time stable（有限时间收敛），如 $\dot x = -\text{sign}(x)|x|^{1/2}$，这比指数稳定"更快"。指数稳定只是渐近稳定中一个特别干净的子类，不是最强的。

### 练习

1. 证明 $\alpha(r) = r^p$（$p > 0$）属于 $\mathcal{K}_\infty$。当 $p = 1$ 时它的逆是什么？当 $p = 2$ 时呢？
2. 给出一个属于 $\mathcal{KL}$ 但不具有指数衰减形式的函数。（提示：考虑多项式衰减 $\beta(r,s) = r/(1+s)$）
3. 证明：若 $\alpha_1, \alpha_2 \in \mathcal{K}_\infty$ 且 $\alpha_1(r) \le V(x) \le \alpha_2(r)$（$r = \|x\|$），则 $V(x) \to 0 \Leftrightarrow \|x\| \to 0$。

---

## 7.3 平衡点与稳定性的三级定义 ⭐

### 动机：什么是"稳定"

日常语言中"稳定"是一个模糊概念。当我们说一个杯子在桌子上是"稳定的"，我们模糊地意味着轻轻推一下它不会倒。但要把这个直觉数学化，我们需要精确回答三个层次的问题：

1. **不跑远**（Lyapunov 稳定）：推一下之后，杯子是否永远在桌子上（不掉下去）？
2. **回来**（渐近稳定）：推一下之后，杯子是否最终回到原来的位置？
3. **快速回来**（指数稳定）：回来的速度是否可以量化估计？

这三级稳定性有严格的层次关系：指数稳定 $\Rightarrow$ 渐近稳定 $\Rightarrow$ Lyapunov 稳定。每一级严格强于下一级。

### 形式化定义

考虑自治系统 $\dot x = f(x)$，其中 $f:\mathcal{D} \to \mathbb{R}^n$ 在原点局部 Lipschitz 且 $f(0) = 0$（原点是平衡点）。

**定义 7.3.1（Lyapunov 稳定，S）**：原点称为 Lyapunov 意义下稳定的，若对每个 $\varepsilon > 0$，存在 $\delta(\varepsilon) > 0$ 使得
$$\|x(0)\| < \delta \quad \Rightarrow \quad \|x(t)\| < \varepsilon, \quad \forall t \ge 0.$$

**物理含义**：初始状态足够接近原点时，轨迹永远不会离开任意给定的 $\varepsilon$ 邻域。轨迹被"困住"了，但不一定回到原点——可以一直绕圈。

**定义 7.3.2（渐近稳定，AS）**：原点称为渐近稳定的，若：
1. 原点是 Lyapunov 稳定的
2. 存在 $\delta > 0$ 使得 $\|x(0)\| < \delta \Rightarrow \lim_{t \to \infty} x(t) = 0$

**注意**：渐近稳定**要求先有 Lyapunov 稳定**再加收敛。仅有收敛不够！

反事实：如果收敛但不 Lyapunov 稳定会怎样？考虑系统轨迹先"暴冲"到很远再回来——这在工程中是灾难性的（机器人先摔倒再站起来不算"稳定控制"）。

**定义 7.3.3（指数稳定，ES）**：原点称为指数稳定的，若存在常数 $k > 0$, $\lambda > 0$, $\delta > 0$ 使得
$$\|x(0)\| < \delta \quad \Rightarrow \quad \|x(t)\| \le k\|x(0)\| e^{-\lambda t}, \quad \forall t \ge 0.$$

**定义 7.3.4（全局渐近稳定，GAS）**：原点称为全局渐近稳定的，若：
1. 原点是 Lyapunov 稳定的
2. **对所有** $x(0) \in \mathbb{R}^n$，$\lim_{t \to \infty} x(t) = 0$

GAS 的吸引域是整个 $\mathbb{R}^n$，不存在"逃逸"的初值。

### 三级稳定性的几何图像

类比水球在碗中的运动：

| 稳定性级别 | 碗的类比 | 几何行为 |
|------------|---------|----------|
| 不稳定 | 倒扣的碗 | 水球滚离 |
| Lyapunov 稳定 | 平底碗 | 水球在底部永远滚动但不出碗 |
| 渐近稳定 | 有摩擦的碗 | 水球最终停在碗底 |
| 指数稳定 | 有强摩擦的碗 | 水球以可预测速率停在碗底 |

### 反事实：仅 AS 而不 ES 的系统

系统 $\dot x = -x^3$ 的原点是渐近稳定的（$V = x^2/2$, $\dot V = -x^4 < 0$），但**不是**指数稳定的。因为解为 $x(t) = x_0/\sqrt{1 + 2x_0^2 t}$，衰减速率为 $O(1/\sqrt{t})$——多项式而非指数。这意味着从远处出发时系统收敛极慢。

### 常见陷阱

⚠️ **概念误区：认为收敛就等于渐近稳定**

$\lim_{t \to \infty} x(t) = 0$ **不等于**渐近稳定！必须同时满足 Lyapunov 稳定。经典反例：存在系统使得所有轨迹最终趋向原点，但对任意小的 $\varepsilon$，都存在任意小的初值使轨迹先暴冲到 $\varepsilon$ 之外再回来。

⚠️ **思维陷阱：认为 GAS 蕴含 ES**

GAS 只要求所有初值的轨迹最终收敛，但收敛速率可以是任意慢的多项式衰减。$\dot x = -x^3$ 是 GAS（容易验证）但不是 ES。

### 练习

1. 验证 $\dot x = -x$ 是指数稳定的，给出常数 $k$ 和 $\lambda$。
2. 证明线性系统 $\dot x = Ax$ 的渐近稳定与指数稳定等价（提示：利用矩阵指数 $e^{At}$ 的范数估计）。
3. 举一个 Lyapunov 稳定但不渐近稳定的非线性系统。（提示：保守系统，如简谐振子）

---

## 7.4 Lyapunov 直接法：完整定理与严格证明 ⭐⭐

### 动机：能量下降 $\Rightarrow$ 稳定

Lyapunov 直接法的核心思想可以用一句话概括：**如果你能找到一个"广义能量函数" $V(x)$，证明它沿系统轨迹单调不增（或严格递减），那么系统必然稳定（或渐近稳定）**。

为什么这个思想如此强大？因为它**完全绕开了解微分方程**。你不需要知道 $x(t)$ 的解析表达式——你只需要知道 $V$ 的变化方向。这就像你不需要知道水球在碗中的精确轨迹，只需要知道"海拔在降低"就足以断定它最终到达最低点。

### Lyapunov 函数的形式化定义

**定义 7.4.1**：设 $\mathcal{D} \subset \mathbb{R}^n$ 是包含原点的开集。函数 $V:\mathcal{D} \to \mathbb{R}$ 称为 Lyapunov 函数候选（Lyapunov function candidate），若：
1. $V$ 是 $C^1$ 的（连续可微）
2. $V(0) = 0$
3. $V(x) > 0$ 对所有 $x \in \mathcal{D} \setminus \{0\}$（正定）

若进一步满足 $V(x) \to \infty$ 当 $\|x\| \to \infty$（radially unbounded），则称 $V$ 为全局 Lyapunov 函数候选。

**定义 7.4.2（Lyapunov 导数）**：$V$ 沿系统 $\dot x = f(x)$ 的轨迹导数为
$$\dot V(x) = \nabla V(x) \cdot f(x) = \sum_{i=1}^n \frac{\partial V}{\partial x_i} f_i(x).$$

注意 $\dot V$ 是状态 $x$ 的函数，**不需要知道轨迹** $x(t)$——这正是直接法的力量所在。

### 定理 7.4.1（Lyapunov 稳定性定理，Khalil 3ed Thm 4.1）⭐⭐

**定理陈述**：考虑 $\dot x = f(x)$，$f:\mathcal{D} \to \mathbb{R}^n$ 局部 Lipschitz，$f(0) = 0$。若存在 $C^1$ 函数 $V:\mathcal{D} \to \mathbb{R}$ 满足：

(a) $V(0) = 0$, $V(x) > 0$ 对 $x \ne 0$（$V$ 正定）

(b) $\dot V(x) \le 0$ 对 $x \in \mathcal{D}$（$\dot V$ 负半定）

则原点是 **Lyapunov 稳定的**。

若进一步满足：

(c) $\dot V(x) < 0$ 对 $x \ne 0$（$\dot V$ 负定）

则原点是**渐近稳定的**。

### 完整证明——稳定性部分

**Step 1：构造正不变集。**

取 $c > 0$ 足够小，使得 sublevel set $\Omega_c = \{x \in \mathcal{D}: V(x) \le c\}$ 是紧集且包含在 $\mathcal{D}$ 内。这样的 $c$ 存在是因为：$V$ 连续、$V(0) = 0$、$V(x) > 0$ 对 $x \ne 0$，所以当 $c$ 足够小时，$\Omega_c$ 是原点的一个小紧邻域。

由条件 (b) $\dot V(x) \le 0$，沿系统轨迹 $V(x(t))$ 是单调不增的。因此，若 $x(0) \in \Omega_c$，则
$$V(x(t)) \le V(x(0)) \le c \quad \Rightarrow \quad x(t) \in \Omega_c, \quad \forall t \ge 0.$$
即 $\Omega_c$ 是**正不变集**——轨迹进入后永远不会离开。

**为什么这一步必要？** 正不变性是稳定性的几何本质。没有它，轨迹可能先离开再回来，违反 Lyapunov 稳定的"永远在 $\varepsilon$ 内"要求。

**Step 2：$\varepsilon$-$\delta$ 论证——构造 $\delta$。**

给定任意 $\varepsilon > 0$（小到 $B_\varepsilon = \{x: \|x\| < \varepsilon\} \subset \mathcal{D}$）。

在球面 $S_\varepsilon = \{x: \|x\| = \varepsilon\}$ 上，$V$ 是正值连续函数。由连续函数在紧集上取得最小值（Weierstrass 定理）：
$$m = \min_{\|x\| = \varepsilon} V(x) > 0.$$
$m > 0$ 是因为 $V$ 正定（球面上 $x \ne 0$，故 $V > 0$）。

**Step 3：利用 $V(0) = 0$ 和连续性选取 $\delta$。**

由 $V(0) = 0$ 和 $V$ 的连续性，存在 $\delta > 0$ 使得
$$\|x\| < \delta \quad \Rightarrow \quad V(x) < m.$$
取这样的 $\delta$（并确保 $\delta \le \varepsilon$）。

**Step 4：完成稳定性证明。**

若 $\|x(0)\| < \delta$，则 $V(x(0)) < m$。

由 $\dot V \le 0$，$V(x(t)) \le V(x(0)) < m$ 对所有 $t \ge 0$。

但 Step 2 告诉我们 $V(x) \ge m$ 对所有 $\|x\| = \varepsilon$。

因此 $x(t)$ **永远不能到达球面** $\|x\| = \varepsilon$——即 $\|x(t)\| < \varepsilon$ 永远成立。

这正是 Lyapunov 稳定的 $\varepsilon$-$\delta$ 定义。$\blacksquare$

### 完整证明——渐近稳定性部分

在稳定性基础上，假设 $\dot V(x) < 0$ 对所有 $x \ne 0$ 严格成立。

**Step 5：$V(x(t))$ 单调严格递减并有极限。**

沿轨迹，只要 $x(t) \ne 0$，就有 $\frac{d}{dt}V(x(t)) = \dot V(x(t)) < 0$。因此 $V(x(t))$ 严格递减。

又 $V(x(t)) \ge 0$（正定性），单调递减有下界的序列必有极限：
$$\lim_{t \to \infty} V(x(t)) = c_0 \ge 0.$$

**Step 6：反证 $c_0 = 0$。**

假设 $c_0 > 0$。由于轨迹 $x(t)$ 在紧集 $\Omega_{V(x(0))}$ 内，$\omega$-极限集 $\omega(x_0)$ 非空且紧。

取 $\omega$-极限点 $p \in \omega(x_0)$，即存在 $t_n \to \infty$ 使 $x(t_n) \to p$。

由 $V$ 连续性：$V(p) = \lim_{n \to \infty} V(x(t_n)) = c_0 > 0$。

因此 $p \ne 0$（因为 $V(0) = 0 \ne c_0$）。

由条件 (c)：$\dot V(p) < 0$。

但 $\omega(x_0)$ 是不变集（$\omega$-极限集的基本性质），所以从 $p$ 出发的轨迹永远在 $\omega(x_0)$ 内。这意味着 $V$ 在 $p$ 处继续严格递减——与 $V$ 在 $\omega(x_0)$ 上恒等于 $c_0$ 矛盾！

因此 $c_0 = 0$。

**Step 7：由 $V$ 正定性推出 $x(t) \to 0$。**

$V(x(t)) \to 0$ 结合 $V(x) \ge \alpha_1(\|x\|)$（$\alpha_1 \in \mathcal{K}$，正定性的比较函数刻画）给出
$$\alpha_1(\|x(t)\|) \le V(x(t)) \to 0 \quad \Rightarrow \quad \|x(t)\| \to 0.$$
$\blacksquare$

### 证明中每一步的必要性——反例表

| 条件 | 若移除该条件 | 反例 |
|------|-------------|------|
| $V$ 正定 | 无法从 $V(x) < m$ 推出 $\|x\| < \varepsilon$ | $V = x_1^2$（$x_2$ 方向无约束） |
| $\dot V \le 0$ | 轨迹可能逃出 $\Omega_c$ | 任何不稳定系统如 $\dot x = x$ |
| $\dot V < 0$（渐近部分） | 只得 Lyapunov 稳定，轨迹可能绕而不收敛 | 简谐振子 $\dot x_1 = x_2, \dot x_2 = -x_1$, $V = x_1^2 + x_2^2$, $\dot V = 0$ |
| Radially unbounded（全局） | 大初值轨迹可能逃逸 | $V = x^2/(1+x^2)$（有上界 1） |

### 全局渐近稳定定理（Khalil 3ed Thm 4.2）

**定理 7.4.2**：若 $V:\mathbb{R}^n \to \mathbb{R}$ 满足 (a)-(c) 且是 **radially unbounded**（即 $\|x\| \to \infty \Rightarrow V(x) \to \infty$），则原点是**全局渐近稳定的**。

Radially unbounded 等价于存在 $\alpha_1 \in \mathcal{K}_\infty$ 使 $V(x) \ge \alpha_1(\|x\|)$。它保证了 $\Omega_c$ 对所有 $c > 0$ 都是紧集——无论初值多大，轨迹都被"框住"，然后可以应用上述渐近稳定性证明。

### 反事实：如果不要求 radially unbounded

考虑 $V(x) = x^2/(1+x^2)$。这个函数正定、$V(0) = 0$、但 $V(x) < 1$ 对所有 $x$。如果某个系统的 $\dot V < 0$ 在 $\{V < 0.5\}$ 上成立，那么 $\Omega_{0.5}$ 是有界的——但 $\Omega_1$ 是整个 $\mathbb{R}^n$，不紧！从远处出发的轨迹可能永远不进入 $\Omega_{0.5}$。所以没有 radially unbounded，只能得局部 AS，不能得 GAS。

### 常见陷阱

⚠️ **编程陷阱：计算 $\dot V$ 时忘记链式法则**

$\dot V$ 不是对 $t$ 直接求导。正确计算是：
$$\dot V = \nabla V \cdot f = \frac{\partial V}{\partial x_1} f_1 + \frac{\partial V}{\partial x_2} f_2 + \cdots$$
这是沿向量场 $f$ 的 Lie 导数。新手常犯的错误是把 $V = x_1^2 + x_2^2$ 直接"对 $t$ 求导"得到 $2x_1 + 2x_2$（这是对 $x$ 求梯度，不是 $\dot V$）。

⚠️ **概念误区：$\dot V = 0$ 意味着系统不动**

$\dot V(x) = 0$ 不意味着 $\dot x = 0$！它只意味着轨迹在 $V$ 的等值面上移动。系统完全可以在 $V$ 的 level set 上高速运动而 $V$ 不变——就像卫星在等重力势面上绕行。

⚠️ **思维陷阱：$V$ 正定 + $\dot V$ 负定就一定 GAS**

不对！$V$ 正定 + $\dot V$ 负定只给**局部**渐近稳定。要 GAS 必须加 radially unbounded。直觉：$V$ 在远处可能"躺平"，无法把远处的轨迹"拉"回来。

### 练习

1. 对系统 $\dot x_1 = -x_1 + 2x_1^2 x_2$, $\dot x_2 = -x_2$，取 $V = x_1^2 + x_2^2$，计算 $\dot V$ 并判断能否得出稳定性。（提示：注意 $\dot V$ 的符号取决于状态所在区域）
2. 写出 Lyapunov 稳定性证明中 Step 2 用到的 Weierstrass 定理的精确陈述。为什么需要球面 $S_\varepsilon$ 是紧集？
3. 构造一个满足 $V(0) = 0$, $V(x) > 0$, $\dot V < 0$，但 $V$ 不 radially unbounded 的例子，并说明为什么只能得到局部 AS。

---

## 7.5 LaSalle 不变集原理：半负定 $\dot V$ 的救命稻草 ⭐⭐

### 动机：工程中 $\dot V < 0$ 几乎不可能

在实际机器人系统中，构造满足 $\dot V < 0$（严格负定）的 Lyapunov 函数**极其困难**。最典型的情形：

**机械系统的总能量** $V = T + U$（动能 + 势能）沿轨迹的变化率是
$$\dot V = -\text{耗散功率} \le 0.$$
耗散功率来自摩擦、阻尼等，但它们**只在运动时**耗散能量。当 $\dot q = 0$（系统静止）时，耗散为零——所以 $\dot V = 0$ 不仅在原点成立，在整个 $\{q: \dot q = 0\}$ 集合上都成立！

这意味着 $\dot V$ 只是**半负定**（$\dot V \le 0$），不是严格负定（$\dot V < 0$）。按照直接法，我们只能得到 Lyapunov 稳定——无法判断系统是否最终收敛到平衡点。

但物理直觉告诉我们：有阻尼的摆**必然**停下来。数学上如何证明？

### LaSalle 不变集原理的陈述（Khalil 3ed Thm 4.4）

**定理 7.5.1（LaSalle 不变性原理）**：设 $\Omega \subset \mathcal{D}$ 是**紧正不变集**。$V:\Omega \to \mathbb{R}$ 连续可微且 $\dot V(x) \le 0$ 对 $x \in \Omega$。定义
$$E = \{x \in \Omega: \dot V(x) = 0\},$$
令 $M$ 为 $E$ 中的**最大不变集**。则由 $\Omega$ 内出发的每条轨迹随 $t \to \infty$ 趋向 $M$：
$$x(0) \in \Omega \quad \Rightarrow \quad x(t) \to M \quad (t \to \infty).$$

**推论**：若 $M = \{0\}$（$E$ 中唯一的不变集只有原点），则原点是渐近稳定的。若进一步 $V$ radially unbounded 且 $\Omega = \mathbb{R}^n$，则 GAS。

### 完整证明

**Step 1：$V(x(t))$ 单调不增有极限。**

$\dot V \le 0$ 意味着 $V(x(t))$ 是 $t$ 的单调不增函数。又 $V$ 在紧集 $\Omega$ 上有下界（连续函数在紧集上有最小值），所以
$$c_0 = \lim_{t \to \infty} V(x(t))$$
存在。

**Step 2：$\omega$-极限集非空、紧、不变、在 $\Omega$ 内。**

轨迹 $\{x(t): t \ge 0\}$ 在紧集 $\Omega$ 内，由 Bolzano-Weierstrass 定理，$\omega$-极限集
$$\omega(x_0) = \{y: \exists t_n \to \infty, x(t_n) \to y\}$$
非空且紧。$\omega$-极限集的标准性质保证它是不变集（从 $\omega(x_0)$ 中任一点出发的轨迹永远在 $\omega(x_0)$ 内）。

**Step 3：$V$ 在 $\omega$-极限集上为常数。**

对 $y \in \omega(x_0)$，取 $t_n \to \infty$ 使 $x(t_n) \to y$。由 $V$ 连续性：
$$V(y) = \lim_{n \to \infty} V(x(t_n)) = c_0.$$
因此 $V$ 在整个 $\omega(x_0)$ 上恒等于 $c_0$。

**Step 4：$\omega(x_0) \subset E$。**

由于 $\omega(x_0)$ 是不变集且 $V$ 在其上为常数 $c_0$，沿 $\omega(x_0)$ 中的轨迹 $V$ 保持不变，即
$$\dot V(y) = 0, \quad \forall y \in \omega(x_0).$$
因此 $\omega(x_0) \subset E$。

**Step 5：$\omega(x_0) \subset M$。**

由于 $\omega(x_0)$ 是不变集（Step 2）且包含在 $E$ 中（Step 4），它包含在 $E$ 的最大不变集 $M$ 中。

**Step 6：$x(t) \to M$。**

由 $\omega$-极限集的定义，$\text{dist}(x(t), \omega(x_0)) \to 0$ 当 $t \to \infty$（轨迹趋向其 $\omega$-极限集）。结合 $\omega(x_0) \subset M$，得 $\text{dist}(x(t), M) \to 0$。$\blacksquare$

### 为什么仅负半定 $\dot V$ 就够了——深层原因

LaSalle 原理的力量来自一个深刻的观察：**$\dot V \le 0$ 虽然不能排除轨迹在 $\{V = 0\}$ 上停留，但不变性约束极大限制了轨迹能停留的位置。**

关键逻辑链：
1. 轨迹的 $\omega$-极限集必在 $E = \{\dot V = 0\}$ 上（因为 $V$ 在 $\omega$ 上常数）
2. $\omega$-极限集必须是**不变的**（一旦进入就永不离开）
3. $E$ 内不变集可能很小——很多时候只有原点！

> **本质洞察**：LaSalle 原理的威力不在于 $\dot V$ 的条件放松了，而在于"不变性"是一个极强的额外约束。$\dot V = 0$ 的集合可能很大，但其中"系统轨迹能永远停留"的部分通常很小。

**跨领域类比**：LaSalle 原理像是"消去法"。你知道犯人在城市里（$\Omega$），知道他的活动范围在缩小（$V$ 递减），最终他只能待在"不需要动就能生存"的地方（$M$）。如果这样的地方只有一个（$M = \{0\}$），你就找到他了。

### 经典应用：阻尼单摆

考虑单摆 $\ddot\theta + b\dot\theta + \sin\theta = 0$（$b > 0$ 为阻尼系数）。

状态空间形式：$\dot x_1 = x_2$, $\dot x_2 = -\sin x_1 - bx_2$。

取总能量 $V = \frac{1}{2}x_2^2 + (1 - \cos x_1)$（动能 + 势能）。

计算：$\dot V = x_2(-\sin x_1 - bx_2) + x_2 \sin x_1 = -bx_2^2 \le 0$。

$\dot V$ 只是**半负定**——在 $\{x_2 = 0\}$（整条横轴）上 $\dot V = 0$。

LaSalle 应用：
- $E = \{x: \dot V = 0\} = \{x: x_2 = 0\}$
- 在 $E$ 中寻找不变集：如果 $x_2(t) \equiv 0$，则 $\dot x_2 = -\sin x_1 - b \cdot 0 = -\sin x_1$。但 $x_2 \equiv 0$ 要求 $\dot x_2 = 0$，即 $\sin x_1 = 0$，即 $x_1 = n\pi$。
- 在 $V$ 的一个紧邻域内（$V < 2$，排除 $x_1 = \pi$ 处的鞍点），最大不变集 $M = \{(0,0)\}$。
- 由 LaSalle 原理，原点渐近稳定。

### 为什么不能直接用 $\dot V < 0$ 证明

因为 $\dot V = -bx_2^2$ 在 $x_2 = 0$ 时为零。物理上，当摆到达最高点时速度为零，此刻没有阻尼耗散。但摆不会永远待在 $x_2 = 0$ 的非平衡位置（因为重力会把它拉回来）——这正是 LaSalle 原理利用的"不变性"约束。

### 常见陷阱

⚠️ **概念误区：$E$ 就是最大不变集 $M$**

$E = \{\dot V = 0\}$ 通常比 $M$ 大得多！$M$ 是 $E$ 中系统轨迹能永远停留的那部分。上例中 $E$ 是整条 $x_2 = 0$ 轴，但 $M$ 只有原点（和 $(\pm\pi, 0)$ 等鞍点）。找 $M$ 时必须把 $E$ 中的点代入系统方程检查不变性。

⚠️ **思维陷阱：LaSalle 给出 GAS**

LaSalle 原理需要**紧正不变集** $\Omega$。如果不能构造全局的紧不变集（比如 $V$ 不 radially unbounded），则只能得到局部结论。要 GAS 必须额外验证 $V$ radially unbounded + 全局 $\dot V \le 0$。

⚠️ **编程陷阱：数值仿真中 $\dot V = 0$ 的误判**

在数值仿真中，由于浮点精度，$\dot V$ 可能在 $E$ 附近振荡而非精确为零。不要用 `if dot_V == 0` 判断，应该用容差 `if abs(dot_V) < tol`。

### 练习

1. 对系统 $\dot x_1 = x_2$, $\dot x_2 = -x_1 - x_2^3$，用 LaSalle 原理证明原点 GAS。（提示：取 $V = x_1^2 + x_2^2$）
2. 找出 $E$ 和 $M$，解释为什么 $\dot V$ 半负定却能得到渐近稳定。
3. 如果把阻尼项 $-bx_2$ 改为 $-b\text{sign}(x_2)$（干摩擦），LaSalle 原理还能用吗？为什么？（提示：考虑 $f$ 的连续性）

---

## 7.6 指数稳定与 Lyapunov 方程 ⭐⭐

### 动机：为什么需要量化衰减速率

渐近稳定告诉你"轨迹最终趋向零"，但**多快**趋向零？在实时控制系统中，"最终"可能意味着 0.1 秒也可能意味着 10 小时——这两者在工程中的差别是致命的。指数稳定给出明确的时间常数 $1/\lambda$，让你能精确回答："扰动发生后，系统多久能恢复到指定精度？"

### 指数稳定的 Lyapunov 表征（Khalil 3ed Thm 4.10）

**定理 7.6.1**：原点指数稳定当且仅当存在 $V:\mathcal{D} \to \mathbb{R}$ 及常数 $c_1, c_2, c_3 > 0$ 使得
$$c_1\|x\|^2 \le V(x) \le c_2\|x\|^2 \qquad (\text{二次型上下夹})$$
$$\dot V(x) \le -c_3\|x\|^2. \qquad (\text{二次衰减})$$

**推导收敛速率**：由上下夹 $V \le c_2\|x\|^2$ 和 $\dot V \le -c_3\|x\|^2 \le -(c_3/c_2)V$，得
$$V(x(t)) \le V(x(0)) e^{-(c_3/c_2)t}.$$
再用下界 $c_1\|x\|^2 \le V$：
$$\|x(t)\| \le \sqrt{\frac{c_2}{c_1}} \|x(0)\| e^{-(c_3/2c_2)t}.$$
即 $k = \sqrt{c_2/c_1}$, $\lambda = c_3/(2c_2)$。

### Lyapunov 方程与线性系统

对线性系统 $\dot x = Ax$，取 $V = x^\top P x$（$P = P^\top \succ 0$），则
$$\dot V = x^\top (A^\top P + PA) x.$$
要使 $\dot V = -x^\top Q x$（$Q \succ 0$），需要
$$A^\top P + PA = -Q. \qquad (\text{Lyapunov 方程})$$

**定理 7.6.2（Lyapunov 方程定理）**：$A$ 是 Hurwitz 矩阵（所有特征值严格负实部）**当且仅当**对任意 $Q = Q^\top \succ 0$，Lyapunov 方程 $A^\top P + PA = -Q$ 存在唯一解 $P = P^\top \succ 0$。

**证明方向 $\Leftarrow$**：若存在 $P \succ 0$ 满足方程，则 $V = x^\top Px$ 是 Lyapunov 函数且 $\dot V = -x^\top Qx < 0$，故 $A$ Hurwitz。

**证明方向 $\Rightarrow$**：若 $A$ Hurwitz，可显式构造解
$$P = \int_0^\infty e^{A^\top t} Q e^{At} dt.$$
该积分收敛（因 $A$ Hurwitz 意味着 $e^{At}$ 指数衰减），容易验证它满足方程且正定。

**与 LQR 的联系**：回顾专题 3.5（§3.5.4b）：Riccati 方程 $A^\top P + PA - PBR^{-1}B^\top P + Q = 0$ 是 Lyapunov 方程的非线性推广——多出的 $-PBR^{-1}B^\top P$ 项正是控制优化的"贡献"。当 $B = 0$（无控制输入）时 Riccati 退化为 Lyapunov 方程，这揭示了两者之间的层级关系（详见专题 3.5 表格：Sylvester → Lyapunov → Riccati）。反过来，LQR 的值函数 $V^* = x^\top P^\star x$（$P^\star$ 为 CARE 解）本身就是闭环系统 $\dot x = (A - BK^\star)x$ 的 Lyapunov 函数——Riccati 方程改写为 Lyapunov 形式后恰好给出 $\dot V^* = -x^\top(Q + K^{\star\top}RK^\star)x < 0$。这一联系在后续 CLF-QP（专题 3.8 §2.2）和 MPC 终端代价（Mayne-Rawlings 2000）中反复出现。

### 间接法（线性化判据，Khalil Thm 4.7）

**定理 7.6.3（Lyapunov 间接法）**：设 $A = \frac{\partial f}{\partial x}\big|_{x=0}$（线性化雅可比矩阵）。则：
- 若 $A$ 的所有特征值严格负实部 $\Rightarrow$ 原点**局部指数稳定**
- 若 $A$ 有至少一个正实部特征值 $\Rightarrow$ 原点**不稳定**
- 若 $A$ 有零实部特征值 $\Rightarrow$ **无法判断**，必须用直接法

**第三种情形为何失效？** 线性化只捕捉平衡点附近的一阶行为。当特征值在虚轴上时，一阶近似是中性稳定的，系统的真实行为由高阶项决定——可能稳定也可能不稳定。

**机器人中的典型失效例子**：$\dot x_1 = x_2$, $\dot x_2 = -x_1^3$。线性化为 $A = \begin{pmatrix} 0 & 1 \\ 0 & 0 \end{pmatrix}$，特征值全为零。线性化完全无法判断。但取 $V = x_1^4/4 + x_2^2/2$ 可验证 $\dot V = -x_1^3 x_2 + x_2(-x_1^3) = 0$——不行！需要更巧妙的 $V$ 或用 LaSalle。实际上该系统原点是稳定的（保守系统）。

### 常见陷阱

⚠️ **编程陷阱：解 Lyapunov 方程时矩阵不对称**

MATLAB 的 `lyap(A', Q)` 或 Python 的 `scipy.linalg.solve_continuous_lyapunov(A.T, -Q)` 要求输入正确的转置形式。常见错误是把 $A^\top P + PA = -Q$ 写反为 $AP + PA^\top = -Q$（这是**对偶** Lyapunov 方程，对应可控性 Gramian）。

⚠️ **概念误区：线性化稳定意味着非线性系统全局稳定**

间接法只给出**局部**结论。非线性系统可能在原点附近指数稳定，但远处有另一个吸引子或无界轨迹。例如 $\dot x = -x + x^3$ 在原点局部稳定（线性化 $A = -1$），但 $|x| > 1$ 时不稳定。

### 练习

1. 对 $A = \begin{pmatrix} -1 & 2 \\ 0 & -3 \end{pmatrix}$，解 Lyapunov 方程 $A^\top P + PA = -I$，求 $P$ 并验证 $P \succ 0$。
2. 利用 $P$ 给出 $\|x(t)\|$ 的指数衰减上界。
3. 讨论：如果 $A$ 有一个特征值恰好为零，Lyapunov 方程有解吗？为什么？

---

## 7.7 Chetaev 不稳定性定理 ⭐⭐

### 动机

Lyapunov 方法告诉你"找到 $V$ 使 $\dot V \le 0$ 则稳定"。但如果你找不到这样的 $V$，是系统不稳定还是你构造能力不足？**Chetaev 定理**给出直接判定不稳定性的工具：找到一个"反 Lyapunov 函数"。

**定理 7.7.1（Chetaev 不稳定性定理）**：若存在 $C^1$ 函数 $V:\mathcal{D} \to \mathbb{R}$ 和原点的任意邻域中的开集 $U$，使得：
1. $V(x) > 0$ 在 $U$ 中
2. $V(x) = 0$ 在 $\partial U \cap \mathcal{D}$ 上
3. $\dot V(x) > 0$ 在 $U$ 中

则原点**不稳定**。

**几何直觉**：在原点的某个"锥形区域"内，$V$ 正且沿轨迹递增。轨迹一旦进入这个锥形区域，就像被推离原点的水球，越来越远——无法保持在原点附近。

**应用**：倒立摆上平衡点 $(\theta = \pi, \dot\theta = 0)$ 的不稳定性、机械臂奇异位形分析。

### 练习

1. 用 Chetaev 定理证明 $\dot x_1 = x_2$, $\dot x_2 = x_1$ 的原点不稳定。（提示：取 $V = x_1 x_2$, $U = \{x_1 x_2 > 0\}$）
2. 从线性化的角度验证上题结论（$A$ 有正实部特征值）。

---

## 7.8 Lyapunov 函数的系统构造方法 ⭐⭐⭐

### 动机：Lyapunov 方法的"阿喀琉斯之踵"

Lyapunov 直接法的逻辑极其优美——但它有一个根本性的实践难题：**如何找到 $V$？** 定理说"如果存在这样的 $V$，则系统稳定"，但没有告诉你 $V$ 从哪里来。对复杂非线性系统，构造 Lyapunov 函数更像艺术而非科学。

这一节系统介绍五种构造方法，从最直觉的（能量法）到最计算化的（SOS、Neural），形成一个完整的工具箱。

### 方法一：能量法（物理直觉）⭐

**核心思想**：机械系统的总能量 $V = T + U$（动能 + 势能）天然是 Lyapunov 函数候选。

**适用条件**：
- 系统有明确的物理能量解释
- 耗散机制存在（摩擦、阻尼）

**标准模式**：对 $n$ 自由度机械系统 $M(q)\ddot q + C(q, \dot q)\dot q + g(q) = \tau_d$（$\tau_d$ 为耗散力矩），取
$$V = \frac{1}{2}\dot q^\top M(q) \dot q + U(q),$$
其中 $U(q)$ 是势能且在平衡点 $q^*$ 取最小值。

若 $\tau_d = -D\dot q$（线性阻尼），则
$$\dot V = \dot q^\top M \ddot q + \frac{1}{2}\dot q^\top \dot M \dot q + \dot q^\top \nabla U$$
$$= \dot q^\top (-C\dot q - g + \tau_d) + \frac{1}{2}\dot q^\top \dot M \dot q + \dot q^\top g$$
$$= \dot q^\top (\frac{1}{2}\dot M - C)\dot q + \dot q^\top \tau_d = -\dot q^\top D \dot q \le 0.$$
这里用了关键性质 $\dot M - 2C$ 是反对称矩阵（来自拉格朗日力学的结构）。

**局限**：能量法只给 $\dot V \le 0$ 半负定，需要 LaSalle 辅助判断渐近稳定。

### 方法二：二次型 $V = x^\top P x$ + Lyapunov 方程 ⭐

**适用条件**：线性系统或非线性系统在平衡点附近的局部分析。

**方法**：
1. 计算雅可比 $A = \partial f / \partial x|_{x=0}$
2. 选取 $Q \succ 0$（通常取 $Q = I$）
3. 解 $A^\top P + PA = -Q$ 得 $P \succ 0$
4. $V = x^\top Px$ 是局部 Lyapunov 函数

**对非线性系统的扩展**：$\dot V = x^\top (A^\top P + PA) x + \text{高阶项} = -x^\top Qx + O(\|x\|^3)$。当 $\|x\|$ 足够小时，$-x^\top Qx$ 主导，$\dot V < 0$——这给出局部指数稳定。

### 方法三：Krasovskii 法 ⭐⭐

**定理（Krasovskii）**：若 $f$ 连续可微且 Jacobian $J(x) = \partial f / \partial x$ 满足
$$J(x) + J(x)^\top \preceq -\alpha I, \quad \alpha > 0, \quad \forall x \in \mathcal{D},$$
则 $V = f(x)^\top f(x) = \|f(x)\|^2$ 是 Lyapunov 函数，原点全局渐近稳定。

**直觉**：Krasovskii 法检查的是"向量场本身的长度是否递减"。如果 $J + J^\top$ 一致负定，那么系统的"速度" $\|f(x)\|$ 沿轨迹递减到零——即系统减速到平衡。

**局限**：条件 $J + J^\top \preceq -\alpha I$ 非常强，很多系统不满足（要求雅可比处处负定）。

### 方法四：变量梯度法 ⭐⭐⭐

**思想**：不从"猜 $V$"出发，而是从"猜 $\nabla V$"出发。

假设 $\nabla V(x) = g(x)$（待确定的向量函数），则 $V(x) = \int_0^1 g(\sigma x) \cdot x \, d\sigma$（沿从原点到 $x$ 的直线积分）。

**$V$ 存在的条件**：$g$ 必须是某个标量函数的梯度，即 $\frac{\partial g_i}{\partial x_j} = \frac{\partial g_j}{\partial x_i}$（对称性/可积条件）。

**步骤**：
1. 假设 $\nabla V$ 的参数化形式（含待定系数）
2. 要求可积条件成立
3. 计算 $\dot V = \nabla V \cdot f$ 并要求负定
4. 联合确定参数

### 方法五：SOS 多项式搜索 ⭐⭐⭐

**核心思想**：对多项式系统 $\dot x = f(x)$，参数化 $V$ 为多项式，把 Lyapunov 条件转化为半定规划（SDP）。

**定义**：多项式 $p(x)$ 称为 Sum of Squares (SOS)，若存在多项式向量 $q$ 使 $p = q^\top q = \sum_i q_i^2$。SOS 蕴含 $p(x) \ge 0$（充分条件）。

**算法**：
1. 参数化 $V(x) = m(x)^\top P m(x)$（$m$ 为单项式基，$P \succeq 0$）
2. 要求 $V - \epsilon\|x\|^2$ 是 SOS（保证正定）
3. 要求 $-\dot V - \epsilon\|x\|^2$ 是 SOS（保证 $\dot V$ 负定）
4. 用 SDP 求解器（MOSEK、SCS）求 $P$

**工具链**：SOSTOOLS（MATLAB）、SumOfSquares.jl（Julia + JuMP）、Drake SOS API（Python）。

**优势**：完全自动化，可同时优化吸引域大小。
**局限**：限于多项式系统；随维数和多项式阶数增长计算量快速上升；SOS 是非负性的充分条件（有gap）。

### 方法六：学习驱动（Neural Lyapunov）⭐⭐⭐⭐

**思想**：用神经网络 $V_\theta(x)$ 参数化 Lyapunov 函数，训练 loss = Lyapunov 条件违反量，再用 SMT/SOS 全域验证。

**框架（Chang-Roohi-Gao 2019, NeurIPS）**：
- **Learner**：训练 $V_\theta$ 使 $\max(0, -V + \epsilon\|x\|^2) + \max(0, \dot V + \alpha V)$ 最小
- **Falsifier**：dReal SMT 求解器搜索违反 Lyapunov 条件的 $x$
- 迭代直到 Falsifier 找不到反例

### 各方法对比

| 方法 | 适用范围 | 自动化程度 | 全局性 | 计算量 |
|------|---------|-----------|--------|--------|
| 能量法 | 机械系统 | 手工 | 局部→全局需 LaSalle | 极低 |
| 二次型 | 线性/局部 | 半自动 | 局部 | 低 |
| Krasovskii | $J+J^\top \prec 0$ | 检验型 | 全局 | 低 |
| 变量梯度 | 低维 | 手工 | 视情况 | 中 |
| SOS | 多项式系统 | 全自动 | 可优化 ROA | 高 |
| Neural | 通用 | 全自动 | 需验证 | 很高 |

### 常见陷阱

⚠️ **思维陷阱：认为"找不到 V 就是不稳定"**

逆定理（7.10 节）告诉我们：稳定系统一定存在 Lyapunov 函数。找不到只意味着构造技巧不够或计算能力不足——不能据此判断不稳定。

⚠️ **编程陷阱：SOS 搜索时多项式阶数太低**

如果 $V$ 的阶数选太低（如只用二次多项式），SOS 可能声称"不可行"（找不到满足条件的 $V$）。这不代表系统不稳定——只是搜索空间太小。应逐步增加阶数。

### 练习

1. 用能量法为 $\ddot\theta + 0.5\dot\theta + \sin\theta = 0$（阻尼单摆）构造 Lyapunov 函数并判断稳定性。
2. 对系统 $\dot x_1 = -x_1 + x_2$, $\dot x_2 = -x_1 - x_2$，用 Lyapunov 方程（$Q = I$）求 $V = x^\top Px$ 并计算衰减速率。
3. 描述如何用 SOS 方法求系统 $\dot x_1 = x_2$, $\dot x_2 = -x_1 - x_2 + x_1^2 x_2$ 的 ROA 估计（写出优化问题形式）。

---

## 7.9 逆定理：稳定性与 Lyapunov 函数的完全等价 ⭐⭐⭐

### 动机：Lyapunov 方法是否"完备"

每次用 Lyapunov 方法失败时，你都会问一个根本性问题：**是不是存在某些稳定系统，根本就没有 Lyapunov 函数？** 如果答案是"是"，那 Lyapunov 方法就有内在局限；如果答案是"否"，那它就是一个原则上**万能**的工具。

逆定理回答了这个问题：**任何稳定系统都存在 Lyapunov 函数**。找不到只是人类（或计算机）的构造能力不足。

### Massera 逆定理（1956）

**定理 7.9.1（Massera）**：若 $\dot x = f(x)$ 的原点是渐近稳定的，则存在 $C^\infty$（无穷次可微）Lyapunov 函数 $V$ 满足 $\dot V(x) < 0$ 对 $x \ne 0$。

**原始出处**：J.L. Massera, "Contributions to stability theory", *Annals of Mathematics* 64(1):182-206, 1956。

**证明思路**（仅概述）：利用 $\mathcal{KL}$ 衰减估计 $\|x(t)\| \le \beta(\|x_0\|, t)$ 构造
$$V(x) = \int_0^\infty g(\|\phi(t; x)\|) dt,$$
其中 $\phi(t; x)$ 是从 $x$ 出发的轨迹，$g$ 是适当选取的核函数（使积分收敛且 $V$ 光滑）。

### Kurzweil 逆定理（1956）

**定理 7.9.2（Kurzweil）**：若原点是全局渐近稳定的，则存在全局定义的 $C^\infty$ Lyapunov 函数。

Kurzweil 的结果把 Massera 从局部推广到全局。原文俄文发表于 *Czechoslovak Math. J.* 6(81):217-259, 1956。

### 现代统一结果（Lin-Sontag-Wang 1996）

**定理 7.9.3**：即便对带时变扰动的非自治系统，只要在 $\mathcal{KL}$ 意义下稳定，就存在**光滑** ISS-Lyapunov 函数。

出处：Y. Lin, E.D. Sontag, Y. Wang, "A smooth converse Lyapunov theorem for robust stability", *SIAM J. Control Optim.* 34(1):124-160, 1996。

### 逆定理的哲学价值

> **本质洞察**：逆定理把 Lyapunov 方法从"有时能成功的充分条件"提升为"原则上总能成功的**充要判据**"。这意味着在证明稳定性时**不存在 Lyapunov 以外的"更强方法"**——任何稳定系统都有 Lyapunov 函数，找不到只是算力或构造技巧不足。

**对 Neural Lyapunov 和 SOS 搜索的意义**：逆定理保证了搜索目标的存在性。无论系统多复杂，只要它确实稳定，就存在一个 $V$ 等着被找到。这为自动化搜索（SOS、Neural Lyapunov）奠定了可行性基础——你不是在搜索一个可能不存在的东西。

### 反事实：如果逆定理不成立

假设存在一个稳定系统没有 Lyapunov 函数。那么：
- SOS 搜索永远不可能为它找到证书（目标不存在）
- Neural Lyapunov 训练永远不会收敛
- 你将不得不用其他方法（如直接求解 ODE）验证稳定性——对复杂系统这不可能

幸好，逆定理告诉我们这种情况不会发生。

### 常见陷阱

⚠️ **概念误区：逆定理意味着能构造出 Lyapunov 函数**

逆定理是**存在性定理**，不是构造性定理。Massera/Kurzweil 的证明虽然给出了 $V$ 的积分形式，但计算该积分需要知道系统的精确流（即解方程）——这又回到了起点。逆定理的价值是哲学性的：保证搜索方向正确。

⚠️ **思维陷阱：认为非光滑系统没有逆定理**

Teel-Praly 2000 和 Clarke-Ledyaev-Stern 1998 给出了非光滑系统（Filippov 解意义下）的逆定理推广。只要系统稳定，即使动力学不连续，也存在（Clarke 广义梯度意义下的）Lyapunov 函数。

### 练习

1. 解释为什么 Massera 逆定理的证明需要知道系统的流 $\phi(t; x)$，这与"不解方程"的 Lyapunov 精神是否矛盾？
2. 逆定理保证 $V$ 是 $C^\infty$ 的。但在实际构造中，我们通常只能找到分段光滑或多项式的 $V$。这种差距在实践中有影响吗？
3. 如果系统只是 Lyapunov 稳定（非渐近），是否存在逆定理？（提示：考虑保守系统的 Hamiltonian）

---

## 7.10 ISS：输入到状态稳定性 ⭐⭐⭐

### 动机：真实系统从不是封闭的

到目前为止，我们分析的都是自治系统 $\dot x = f(x)$——没有外部输入。但真实机器人系统**总有外部扰动**：

- 电机噪声、量化误差
- 观测器估计误差
- 地面摩擦系数的随机变化
- 风力、碰撞等外部干扰

这些扰动使系统变为 $\dot x = f(x, u)$，其中 $u(t)$ 是外部输入（扰动）。此时经典 Lyapunov 不再适用——你不能要求轨迹趋向零，因为持续的扰动会阻止这一点。

**ISS（Input-to-State Stability）** 由 Sontag 1989 提出，目的是回答：**在有持续扰动的情况下，轨迹会偏离原点多远？偏移量与扰动大小之间有怎样的定量关系？**

### ISS 的形式化定义

**定义 7.10.1（ISS，Sontag 1989）**：系统 $\dot x = f(x, u)$ 称为 ISS，若存在 $\beta \in \mathcal{KL}$, $\gamma \in \mathcal{K}$ 使得对所有可测有界输入 $u$ 和所有初值 $x_0$：
$$\|x(t)\| \le \beta(\|x_0\|, t) + \gamma\left(\sup_{0 \le \tau \le t} \|u(\tau)\|\right).$$

**物理含义**：
- $\beta(\|x_0\|, t)$：初始状态的影响随时间衰减（如同无扰动时的渐近稳定）
- $\gamma(\|u\|_\infty)$：扰动的影响被 $\gamma$ 函数"限住"——扰动越大偏离越大，但有界

**跨领域类比**：ISS 像一个弹簧-阻尼系统受到外力。弹簧的恢复力确保系统不会无限偏离，阻尼确保暂态衰减。$\gamma$ 就是"弹簧的柔度"——$\gamma$ 越小系统越"硬"，对扰动越鲁棒。

### ISS 的等价刻画

**定理 7.10.2（Sontag-Wang 1995）**：以下条件等价：
1. 系统是 ISS
2. 0-GAS + UBIBS（零输入下全局渐近稳定 + 有界输入有界状态）
3. 存在 ISS-Lyapunov 函数

### ISS-Lyapunov 函数

**定义 7.10.3**：$V:\mathbb{R}^n \to \mathbb{R}$ 称为 ISS-Lyapunov 函数，若存在 $\alpha_1, \alpha_2 \in \mathcal{K}_\infty$, $\alpha_3 \in \mathcal{K}$, $\chi \in \mathcal{K}$ 使得：
$$\alpha_1(\|x\|) \le V(x) \le \alpha_2(\|x\|) \qquad (\text{上下夹})$$
$$\|x\| \ge \chi(\|u\|) \quad \Rightarrow \quad \dot V \le -\alpha_3(\|x\|) \qquad (\text{implication form})$$

等价的**耗散型**表述：
$$\dot V(x, u) \le -\alpha(\|x\|) + \sigma(\|u\|), \quad \alpha \in \mathcal{K}, \sigma \in \mathcal{K}.$$

**直觉**：ISS-Lyapunov 函数说"只要状态足够大（$\|x\| \ge \chi(\|u\|)$），能量就在下降"。这意味着系统最终被拉入一个由扰动大小决定的球 $\|x\| \le \chi(\|u\|_\infty)$ 内。

### ISS-Lyapunov 函数的构造四步套路

1. **选择候选 $V$**：通常取 $u = 0$ 时的 Lyapunov 函数（如 $V = x^\top Px$）
2. **计算 $\dot V$**：沿 $\dot x = f(x, u)$ 求导
3. **分离状态项与输入项**：用 Young 不等式 $ab \le \frac{a^2}{2\epsilon} + \frac{\epsilon b^2}{2}$ 把交叉项拆开
4. **验证 ISS 条件**：确认 $\dot V \le -\alpha(\|x\|) + \sigma(\|u\|)$

**详细例子**：系统 $\dot x = -x^3 + xu$。

取 $V = \frac{1}{2}x^2$。则
$$\dot V = x(-x^3 + xu) = -x^4 + x^2 u.$$

用 Young 不等式处理交叉项：$x^2 u \le x^2 |u| \le \frac{1}{2}x^4 + \frac{1}{2}u^2$。

因此：
$$\dot V \le -x^4 + \frac{1}{2}x^4 + \frac{1}{2}u^2 = -\frac{1}{2}x^4 + \frac{1}{2}u^2.$$

这正是耗散型 ISS 条件，$\alpha(r) = \frac{1}{2}r^4$, $\sigma(r) = \frac{1}{2}r^2$。

ISS 增益：$\chi(r) = r^{1/2}$（由 $\alpha(\chi(r)) = \sigma(r)$ 解出）。

### ISS 小增益定理 ⭐⭐⭐

**定理 7.10.4（Jiang-Teel-Praly 1994）**：考虑两个子系统的反馈互联：
$$\Sigma_1: \dot x_1 = f_1(x_1, x_2), \qquad \Sigma_2: \dot x_2 = f_2(x_1, x_2),$$
若 $\Sigma_1$ 以 $x_2$ 为输入是 ISS（增益 $\gamma_1$），$\Sigma_2$ 以 $x_1$ 为输入是 ISS（增益 $\gamma_2$），且**小增益条件**
$$\gamma_1 \circ \gamma_2(r) < r, \quad \forall r > 0$$
成立，则整体系统 $(\Sigma_1, \Sigma_2)$ 是 ISS。

**物理直觉**：每个子系统的输出被另一个"放大"后反馈。如果合成增益 $\gamma_1 \circ \gamma_2$ 小于恒等函数（衰减），则扰动在循环中越来越小——系统整体稳定。这与经典线性小增益 $\|G_1\|_\infty \|G_2\|_\infty < 1$ 的精神完全一致（详见专题 3.6 §3.6.9 的 $H_\infty$ 小增益定理），但推广到非线性。线性版本中增益退化为 $L_\infty$ 增益 $\gamma_i(r)=\|G_i\|_\infty\cdot r$（附录 B.3 推导），条件变为 $\|G_1\|_\infty\|G_2\|_\infty<1$——这正是鲁棒控制中判断互联稳定性的核心工具。

**机器人中的三大应用**：

1. **观测器-控制器分离**：控制器以估计状态 $\hat x$ 为输入，观测器以控制输出为输入。两者互联的稳定性由小增益保证。

2. **级联系统**：若 $\Sigma_2$ 不反馈到 $\Sigma_1$（$f_1$ 不含 $x_2$），则为单向级联。此时只需 $\Sigma_1$ GAS + $\Sigma_2$ ISS 即可保证整体 GAS。腿足机器人"高层 MPC → 底层 PD"的层级结构天然是级联。

3. **分布式多机器人**：$N$ 个机器人通过网络耦合，每个局部控制器 ISS，增益由拓扑决定。整体稳定性等价于增益矩阵谱半径 < 1。

### iISS（积分 ISS）

**定义**（Sontag 1998）：系统称为 iISS 若
$$\|x(t)\| \le \beta(\|x_0\|, t) + \int_0^t \gamma(\|u(\tau)\|) d\tau.$$

iISS 比 ISS 弱：ISS 要求有界输入 $\Rightarrow$ 有界状态，iISS 只要求积分有限的输入 $\Rightarrow$ 有界状态。对 bilinear 系统等场景更适用。

### 常见陷阱

⚠️ **概念误区：ISS 意味着扰动消失后状态趋向零**

不完全正确。ISS 定义中 $\gamma(\sup\|u\|)$ 项保证的是"有界输入 $\Rightarrow$ 最终有界状态"。若输入为零（$u \equiv 0$），则退化为 $\|x(t)\| \le \beta(\|x_0\|, t) \to 0$——确实趋向零。但若 $u$ 只是减小而非消失，状态会趋向 $\gamma(\limsup\|u\|)$ 邻域而非原点。

⚠️ **思维陷阱：小增益条件对所有系统都容易验证**

计算非线性增益 $\gamma_1, \gamma_2$ 通常本身就很困难（需要找 ISS-Lyapunov 函数）。小增益定理的力量在于**一旦知道增益就能直接判断互联稳定性**，省去了为整体系统重新构造 Lyapunov 函数。

### 练习

1. 对系统 $\dot x = -x + x^2 u$，证明它在 $\|x\| < 1$ 的区域内是 ISS。给出 ISS-Lyapunov 函数和增益函数 $\gamma$。
2. 两个子系统 $\Sigma_1: \dot x_1 = -x_1 + x_2^2$ 和 $\Sigma_2: \dot x_2 = -2x_2 + x_1$，验证小增益条件并判断整体 ISS。
3. 解释为什么机器人"MPC（50Hz）+ PD（1kHz）"的级联结构天然满足级联 ISS 条件。（提示：MPC 输出是慢变的有界参考信号）

---

## 7.11 级联系统稳定性 ⭐⭐⭐

### 动机

现代机器人控制几乎都是**层级结构**：高层规划器输出参考轨迹，底层控制器跟踪该轨迹。这种层级结构在数学上对应**级联系统**：

$$\dot x_1 = f_1(x_1) \qquad (\text{上层，不受下层影响})$$
$$\dot x_2 = f_2(x_1, x_2) \qquad (\text{下层，受上层驱动})$$

核心问题：如果上层 $\dot x_1 = f_1(x_1)$ 单独是 GAS 的，下层 $\dot x_2 = f_2(0, x_2)$（在上层为零输入时）也是 GAS 的，整体系统是否 GAS？

### 级联系统稳定性定理

**定理 7.11.1（Sontag 1989）**：若
1. $\dot x_1 = f_1(x_1)$ 的原点是 GAS 的
2. $\dot x_2 = f_2(0, x_2)$ 的原点是 GAS 的
3. $\dot x_2 = f_2(x_1, x_2)$ 的解对 $x_1$ 是 ISS 的（或满足适当的增长条件）

则级联系统的原点 $(0, 0)$ 是 GAS 的。

**直觉**：上层最终趋向零（条件 1），因此对下层的"驱动"$x_1(t) \to 0$。当驱动消失后，下层也趋向零（条件 2）。条件 3 保证在过渡期间下层不会"爆炸"。

### 机器人中的典型级联结构

| 层级 | 上层 $\Sigma_1$ | 下层 $\Sigma_2$ | 级联条件 |
|------|----------------|----------------|----------|
| MPC + WBC | MPC 输出 CoM 轨迹 | WBC 跟踪关节力矩 | MPC 稳定 + WBC ISS |
| 落脚规划 + 腿控 | Raibert 落脚算法 | 单腿摆动控制 | 落脚稳定 + 腿控 ISS |
| 感知 + 控制 | SLAM 状态估计 | 基于估计的控制器 | SLAM 收敛 + 控制器对估计误差 ISS |

### 常见陷阱

⚠️ **概念误区：条件 1+2 就够了，不需要条件 3**

反例：$\dot x_1 = -x_1$, $\dot x_2 = -x_2 + x_1 x_2^2$。上层 GAS（$x_1 \to 0$ 指数衰减），下层单独 GAS（$\dot x_2 = -x_2$）。但如果 $x_2(0)$ 很大，在 $x_1$ 衰减之前，项 $x_1 x_2^2$ 可能使 $x_2$ 爆炸。需要 ISS 类增长条件来排除这种情况。

### 练习

1. 证明级联系统 $\dot x_1 = -x_1$, $\dot x_2 = -x_2 + x_1$ 的原点 GAS。（提示：先解 $x_1$，代入第二个方程显式求解）
2. 对上述系统，构造一个全局 Lyapunov 函数 $V(x_1, x_2)$。（提示：$V = x_1^2 + (x_2 - \alpha x_1)^2$，选适当 $\alpha$）

---

## 7.12 Lyapunov 与控制设计：反步法（Backstepping）⭐⭐⭐

### 动机

到目前为止，Lyapunov 理论主要用于**分析**——给定系统判断稳定性。但它同样可以用于**设计**——递归地构造控制律和 Lyapunov 函数。反步法（Backstepping）是这一思想最经典的实现。

### 反步法的核心思想

考虑一个"可以从后往前逐步镇定"的系统结构：
$$\dot x_1 = f_1(x_1) + g_1(x_1) x_2$$
$$\dot x_2 = f_2(x_1, x_2) + g_2(x_1, x_2) u$$

**关键观察**：如果 $x_2$ 是"虚拟控制"，能选择 $x_2 = \phi(x_1)$ 使第一个方程稳定，那么真正的控制 $u$ 只需让 $x_2$ 跟踪 $\phi(x_1)$。

### 详细步骤

**Step 1：设计虚拟控制。**

对第一个方程 $\dot x_1 = f_1(x_1) + g_1(x_1) x_2$，假设 $x_2$ 可以自由选择。找 $V_1(x_1)$ 和 $\phi(x_1)$（$\phi(0) = 0$）使得
$$\dot V_1 = \nabla V_1 \cdot [f_1 + g_1 \phi] \le -W_1(x_1) < 0.$$

**Step 2：定义误差变量。**

令 $z_2 = x_2 - \phi(x_1)$（实际 $x_2$ 与虚拟控制的偏差）。

**Step 3：构造增广 Lyapunov 函数。**

$$V_2(x_1, z_2) = V_1(x_1) + \frac{1}{2} z_2^2.$$

**Step 4：计算 $\dot V_2$ 并设计 $u$。**

$$\dot V_2 = \dot V_1 + z_2 \dot z_2 = \dot V_1 + z_2(\dot x_2 - \dot\phi)$$
$$= \dot V_1 + z_2(f_2 + g_2 u - \frac{\partial\phi}{\partial x_1}\dot x_1).$$

选择 $u$ 使 $\dot V_2 \le -W_1(x_1) - W_2(z_2) < 0$。

### 例子：二阶积分器

系统 $\dot x_1 = x_2$, $\dot x_2 = u$。

**Step 1**：取 $V_1 = x_1^2/2$。若 $x_2 = \phi(x_1) = -c_1 x_1$（$c_1 > 0$），则 $\dot V_1 = x_1 x_2 = -c_1 x_1^2 < 0$。

**Step 2**：$z_2 = x_2 - (-c_1 x_1) = x_2 + c_1 x_1$。

**Step 3**：$V_2 = x_1^2/2 + z_2^2/2$。

**Step 4**：
$$\dot V_2 = x_1 \dot x_1 + z_2 \dot z_2 = x_1 x_2 + z_2(u + c_1 x_2)$$
$$= x_1(z_2 - c_1 x_1) + z_2(u + c_1(z_2 - c_1 x_1))$$
$$= -c_1 x_1^2 + x_1 z_2 + z_2 u + c_1 z_2^2 - c_1^2 x_1 z_2.$$

选 $u = -(1 - c_1^2)x_1 - (c_1 + c_2)z_2$（$c_2 > 0$）消去交叉项并使 $\dot V_2 = -c_1 x_1^2 - c_2 z_2^2 < 0$。

### 与机器人控制的联系

反步法的层级结构天然对应机器人的"外环-内环"设计：
- 外环（慢）：位置/姿态控制，虚拟控制 = 期望速度/角速度
- 内环（快）：速度/角速度跟踪，用实际执行器力矩

### 练习

1. 对系统 $\dot x_1 = x_1^2 + x_2$, $\dot x_2 = u$，用反步法设计镇定控制律 $u(x_1, x_2)$ 并给出 Lyapunov 函数。
2. 讨论反步法对系统维数增长时的可扩展性问题（"复杂度爆炸"）。

---

## 7.13 典型例题：Van der Pol 振子 ⭐⭐

### 系统描述

Van der Pol 振子：
$$\dot x_1 = x_2, \qquad \dot x_2 = -x_1 + \mu(1 - x_1^2)x_2, \quad \mu > 0.$$

当 $\mu > 0$ 时，该系统有一个不稳定的平衡点（原点）和一个稳定的极限环。这是非线性振荡的经典范例。

### 原点不稳定性的 Lyapunov 分析

取 $V = \frac{1}{2}(x_1^2 + x_2^2)$。计算：
$$\dot V = x_1 x_2 + x_2(-x_1 + \mu(1-x_1^2)x_2) = \mu(1-x_1^2)x_2^2.$$

在 $|x_1| < 1$ 的带状区域内（且 $x_2 \ne 0$），$\dot V > 0$——能量在增加。

取 Chetaev 函数的区域 $U = \{(x_1, x_2): x_1^2 < 1, x_2 \ne 0, V > 0\}$，可以用 Chetaev 定理证明原点不稳定。

### 极限环的存在性（Poincare-Bendixson 定理）

在 $\|x\|$ 足够大时（$x_1^2 > 1$），$\dot V < 0$——能量在减小。这意味着远处的轨迹被吸引回来。结合原点的不稳定性（近处轨迹被排斥），由 Poincare-Bendixson 定理，存在稳定极限环。

**这就是 Lyapunov 函数给出的全局定性图像**：原点排斥 + 远处吸引 = 极限环。

### 为什么经典 Lyapunov 不能直接处理极限环

经典 Lyapunov 稳定性以**平衡点**为中心。极限环不是平衡点——它是一条**周期轨道**。要分析极限环的稳定性，需要：
- 轨道稳定性（Poincare map + 特征乘子）
- 或 Contraction Theory（7.5 节的 Lohmiller-Slotine 1998）

> **本质洞察**：经典 Lyapunov 方法针对平衡点设计。对更一般的吸引集（极限环、奇异吸引子），需要工具升级——Contraction Theory 是自然的推广。

---

## 7.14 典型例题：机器人关节 PD 控制稳定性证明 ⭐⭐

### 系统描述

$n$ 自由度机器人动力学：
$$M(q)\ddot q + C(q, \dot q)\dot q + g(q) = \tau,$$
其中 $M(q) \succ 0$ 为惯量矩阵，$C$ 为 Coriolis 矩阵，$g$ 为重力项，$\tau$ 为控制力矩。

**PD + 重力补偿控制律**：
$$\tau = g(q) - K_P(q - q_d) - K_D \dot q,$$
其中 $K_P, K_D \succ 0$ 为正定增益矩阵，$q_d$ 为期望关节角度。

### 稳定性证明

令 $e = q - q_d$，则闭环动力学为：
$$M(q)\ddot e + C(q, \dot q)\dot e + K_D \dot e + K_P e = 0.$$

**选取 Lyapunov 函数**（总能量形式）：
$$V = \frac{1}{2}\dot e^\top M(q) \dot e + \frac{1}{2}e^\top K_P e.$$

注意 $V > 0$ 对 $(e, \dot e) \ne 0$（因为 $M \succ 0$, $K_P \succ 0$）。

**计算 $\dot V$**：
$$\dot V = \dot e^\top M \ddot e + \frac{1}{2}\dot e^\top \dot M \dot e + e^\top K_P \dot e.$$

代入 $M\ddot e = -C\dot e - K_D \dot e - K_P e$：
$$\dot V = \dot e^\top(-C\dot e - K_D\dot e - K_P e) + \frac{1}{2}\dot e^\top \dot M \dot e + e^\top K_P \dot e$$
$$= \dot e^\top(\frac{1}{2}\dot M - C)\dot e - \dot e^\top K_D \dot e.$$

**关键性质**：$\dot M - 2C$ 是反对称矩阵（源自拉格朗日力学）。因此 $\dot e^\top(\frac{1}{2}\dot M - C)\dot e = 0$。

最终：
$$\dot V = -\dot e^\top K_D \dot e \le 0. \qquad (\text{半负定！})$$

$\dot V$ 只是半负定——在 $\dot e = 0$ 时为零。

### 应用 LaSalle 不变集原理

$E = \{(e, \dot e): \dot V = 0\} = \{(e, \dot e): \dot e = 0\}$。

在 $E$ 中寻找不变集：若 $\dot e(t) \equiv 0$，则 $\ddot e = 0$，代入动力学方程：
$$0 + 0 + K_P e = 0 \quad \Rightarrow \quad e = 0.$$

因此 $E$ 中最大不变集 $M = \{(0, 0)\}$。

由 LaSalle 原理，$(e, \dot e) \to (0, 0)$，即 $q \to q_d$, $\dot q \to 0$。**PD + 重力补偿控制渐近稳定。**

### 为什么需要重力补偿

如果不加重力补偿（$\tau = -K_P e - K_D \dot e$），闭环方程变为：
$$M\ddot e + C\dot e + K_D\dot e + K_P e = g(q_d) - g(q).$$

右端不为零（除非 $q = q_d$），平衡点不再是 $(0, 0)$ 而是一个非零稳态误差。这就是为什么重力补偿在关节控制中是标配。

### 反事实：如果 $\dot M - 2C$ 不是反对称的

整个证明的关键步骤依赖于 $\dot e^\top(\frac{1}{2}\dot M - C)\dot e = 0$。这一性质来自拉格朗日力学的结构——如果我们随意选择 $C$ 的参数化（注意 $C$ 的选择不唯一），可能破坏这一性质。标准选择是 **Christoffel 符号形式**的 $C$，它保证反对称性。

### 练习

1. 如果把 $K_D$ 换成非线性阻尼 $K_D \text{sign}(\dot e)$（干摩擦模型），上述证明哪一步会出问题？
2. 对 2-DOF 平面机械臂，写出具体的 $M(q)$, $C(q, \dot q)$, $g(q)$ 并验证 $\dot M - 2C$ 反对称性。
3. 如果加入关节柔性（$\tau$ 经过弹簧传到关节），PD + 重力补偿还能保证渐近稳定吗？

---

## 7.15 吸引域估计与 Zubov 方程 ⭐⭐⭐

### 动机

即使证明了渐近稳定，一个关键的工程问题是：**吸引域有多大？** 如果机器人受到大扰动飞出吸引域，控制器就失效了。精确计算吸引域 $\mathcal{R} = \{x_0: x(t; x_0) \to 0\}$ 通常不可能，但 Lyapunov 函数给出**内估计**。

### Lyapunov 内估计

若 $\Omega_c = \{x: V(x) \le c\}$ 紧、包含在 $\dot V < 0$ 成立的区域内，则 $\Omega_c \subset \mathcal{R}$。

**最大估计**：选最大的 $c$ 使 $\Omega_c$ 仍包含在 $\dot V < 0$ 区域内。

**SOS 优化**：对多项式系统，这可以表述为
$$\max_{\rho, V} \quad \rho$$
$$\text{s.t.} \quad V - \epsilon\|x\|^2 \text{ is SOS}$$
$$\quad -\dot V - \lambda(\rho - V) \text{ is SOS} \quad (\lambda \text{ SOS multiplier})$$

这里 S-procedure 把"在 $V \le \rho$ 内 $\dot V < 0$" 编码为 SOS 约束。

### Zubov 方程

**Zubov 方程（Zubov 1964）**精确刻画 ROA 边界：寻找 $V:\mathcal{R} \to [0, 1)$ 和正定 $\phi$ 使得
$$\nabla V(x) \cdot f(x) = -\phi(x)(1-V(x))\sqrt{1 + \|f(x)\|^2}.$$

则 $\mathcal{R} = \{V < 1\}$，$\partial\mathcal{R} = \{V = 1\}$。这是一阶非线性 PDE，通常需数值求解。与 HJB 的联系：Zubov 方程可视为零控制的 HJB 粘性解形式。

### 练习

1. 对 $\dot x = -x + x^3$，用 $V = x^2/2$ 估计吸引域。精确吸引域是什么？
2. 解释 SOS-ROA 优化中 S-procedure 乘子 $\lambda$ 的物理含义。

---

## 7.16 比较定理与估计技术 ⭐⭐

### 动机

Lyapunov 函数告诉你"轨迹趋向零"，但没有告诉你"多快"。比较定理把 Lyapunov 不等式 $\dot V \le -\alpha(V)$ 转化为**显式衰减估计**——精确的时间响应包络。

### 比较定理（Khalil Lemma 4.4）

若标量 ODE $\dot v = -\alpha(v)$, $v(0) = v_0$, $\alpha \in \mathcal{K}$，则存在唯一 $\sigma \in \mathcal{KL}$ 使解 $v(t) = \sigma(v_0, t)$。

**推论**：若 $\dot V(x(t)) \le -\alpha(V(x(t)))$，则 $V(x(t)) \le \sigma(V(x(0)), t)$。

**指数情形**：$\dot V \le -\lambda V$ $\Rightarrow$ $V(t) \le V(0)e^{-\lambda t}$。

### Gronwall-Bellman 不等式

**定理**：若 $u(t) \le c + \int_0^t k(s)u(s)ds$，则 $u(t) \le c \exp(\int_0^t k(s)ds)$。

在以下场景中不可或缺：
- 证明扰动系统保 ISS
- 数值方法误差界
- 观测器收敛速率估计

### 工程估计四步范式

1. 构造 $V$
2. 计算 $\dot V$
3. 用 Young/Cauchy-Schwarz 把交叉项压成 $-\alpha(V) + \sigma(\|u\|)$ 形式
4. 应用比较定理得收敛率

这一范式在 backstepping、自适应控制、observer 设计中反复出现——掌握它就掌握了非线性分析的标准"流水线"。

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关键应用 |
|--------|---------|------|----------|
| 比较函数 | $\mathcal{K}$, $\mathcal{K}_\infty$, $\mathcal{KL}$ 三类 | ⭐⭐ | 稳定性统一语言 |
| 三级稳定性 | S / AS / ES 的 $\varepsilon$-$\delta$ 与比较函数刻画 | ⭐ | 基础判断 |
| Lyapunov 直接法 | 正定 $V$ + 负定/半定 $\dot V$ | ⭐⭐ | 所有稳定性证明的基石 |
| LaSalle 原理 | 半负定 $\dot V$ + 最大不变集 | ⭐⭐ | 机械系统、PD 控制 |
| 指数稳定 | 二次夹 + Lyapunov 方程 | ⭐⭐ | 线性系统、局部分析 |
| Chetaev 定理 | 不稳定性判据 | ⭐⭐ | 平衡点分类 |
| 构造方法 | 能量/二次型/Krasovskii/变量梯度/SOS/Neural | ⭐⭐⭐ | 实际系统分析 |
| 逆定理 | AS $\Rightarrow$ $\exists$ 光滑 $V$ | ⭐⭐⭐ | 哲学完备性 |
| ISS | 带扰动的鲁棒稳定性 | ⭐⭐⭐ | 互联系统、扰动分析 |
| 小增益定理 | $\gamma_1 \circ \gamma_2 < \text{id}$ | ⭐⭐⭐ | 分布式系统、级联系统 |
| 级联系统 | 层级控制结构的稳定性 | ⭐⭐⭐ | MPC + 底层控制 |
| 反步法 | 递归 Lyapunov 设计 | ⭐⭐⭐ | 非线性控制器设计 |
| 吸引域估计 | sublevel set + SOS + Zubov | ⭐⭐⭐ | 安全性验证 |
| 比较定理 | $\dot V \le -\alpha(V)$ 的衰减估计 | ⭐⭐ | 收敛速率量化 |

---

## 累积项目：本章新增模块

**项目：从零构建非线性系统稳定性分析工具箱**

本章新增：
- **模块 7A**：实现 Lyapunov 方程求解器（`solve_lyapunov(A, Q)` → $P$），并验证线性系统的指数衰减率
- **模块 7B**：实现 LaSalle 原理的数值验证——给定系统和 $V$，自动求 $E = \{\dot V = 0\}$ 并检查不变性
- **模块 7C**：用 SymPy 符号计算 $\dot V$ 并判断正/负定性
- **模块 7D**：对机器人 PD 控制的 Lyapunov 稳定性进行数值仿真验证

---

## 延伸阅读

| 资源 | 作者/来源 | 难度 | 内容 |
|------|----------|------|------|
| *Nonlinear Systems* Ch.4 | Khalil, 3ed 2002 | ⭐⭐ | Lyapunov 稳定性经典教材，证明最完整 |
| *Underactuated Robotics* Ch.9 | Tedrake, MIT | ⭐⭐ | 机器人视角 + Drake 代码 |
| *Mathematical Control Theory* Ch.5-7 | Sontag, 2ed 1998 | ⭐⭐⭐ | ISS 创始人视角，免费 PDF |
| *Applied Nonlinear Control* Ch.3-4 | Slotine-Li, 1991 | ⭐ | 工程导向，证明简洁 |
| *Nonlinear Dynamical Systems* | Haddad-Chellaboina, 2008 | ⭐⭐⭐ | 现代综合，涵盖所有变种 |
| Tsukamoto-Chung-Slotine 2021 | *Annual Reviews in Control* | ⭐⭐⭐⭐ | Contraction Theory 最新综述 |
| Dawson-Gao-Fan 2023 | *IEEE Trans. Robotics* | ⭐⭐⭐⭐ | Neural Lyapunov/CBF 综述 |
| DR_CAN B站视频 | 中文讲解 | ⭐ | 中文入门首选 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| 找到 $V$ 但 $\dot V$ 不定号 | $V$ 选取不当 | 1.检查 $V$ 的物理含义 2.尝试加交叉项 3.用变量梯度法系统搜索 | 7.8 |
| $\dot V \le 0$ 但无法判 AS | 需要 LaSalle | 1.求 $E = \{\dot V=0\}$ 2.在 $E$ 中验证不变性 3.检查 $M$ 是否只含原点 | 7.5 |
| Lyapunov 方程 solve 返回非正定 | $A$ 非 Hurwitz | 1.检查 $A$ 特征值 2.确认系统矩阵正确 3.检查输入矩阵符号 | 7.6 |
| SOS 搜索不可行 | 多项式阶数太低 | 1.增加 $V$ 的阶数 2.增加 SOS multiplier 阶数 3.检查系统是否确实稳定 | 7.8 |
| 指数稳定率估计过保守 | $P$ 的条件数太大 | 1.换 $Q$（如取 $Q = I$ 或 $Q = \text{diag}$）2.用数值优化最大化 $\lambda$ | 7.6 |
| 机器人 PD 控制不收敛 | 遗漏重力补偿 | 1.检查是否加了 $g(q)$ 前馈 2.验证 $K_P, K_D$ 正定 3.检查 $\dot M - 2C$ 反对称性 | 7.14 |

---

## 跨章综合练习

**综合题**（需要本章 + 前置线性系统知识）：

对机器人系统 $M(q)\ddot q + C(q, \dot q)\dot q + g(q) = \tau$，设计如下层级控制器并**逐层证明稳定性**：

1. **外环**（位置控制）：设计期望加速度 $\ddot q_d = -K_P e - K_D \dot e$（虚拟控制），用二次 Lyapunov 函数 $V_1 = e^\top K_P e + \dot e^\top M \dot e$ 证明误差系统渐近稳定。

2. **内环**（力矩控制）：$\tau = M(q)\ddot q_d + C(q, \dot q)\dot q + g(q)$（计算力矩法），证明内环精确跟踪（误差为零）。

3. **讨论**：如果内环有建模误差 $\Delta M, \Delta C, \Delta g$，用 ISS 框架分析整体系统的鲁棒性。给出误差界与模型误差大小的定量关系。

---

## 7.17 非自治系统与一致稳定性 ⭐⭐⭐

### 动机：为什么时变系统需要额外小心

到目前为止我们处理的都是自治系统 $\dot x = f(x)$——向量场不显式依赖时间 $t$。但机器人系统中大量场景是**非自治的**：

- 时变轨迹跟踪：期望轨迹 $q_d(t)$ 变化使闭环系统显式含 $t$
- 周期切换系统：步态周期性切换支撑相
- 衰减增益：自适应控制中学习率 $\gamma(t)$ 随时间衰减

对非自治系统 $\dot x = f(x, t)$，经典 Lyapunov 定理需要修改——核心修改是加入**一致性**要求。

### 一致稳定性定义

**定义 7.17.1（一致渐近稳定，UAS）**：$\dot x = f(x, t)$ 的原点一致渐近稳定，若存在 $\beta \in \mathcal{KL}$ 使得对所有初始时刻 $t_0 \ge 0$：
$$\|x(t)\| \le \beta(\|x(t_0)\|, t - t_0), \quad \forall t \ge t_0.$$

关键词"一致"意味着 $\beta$ **不依赖于初始时刻 $t_0$**——无论何时启动，衰减行为都一样。

### 非自治 Lyapunov 定理（Khalil Thm 4.9）

**定理 7.17.2**：若存在 $V(x, t)$ 满足：
$$\alpha_1(\|x\|) \le V(x, t) \le \alpha_2(\|x\|), \quad \alpha_1, \alpha_2 \in \mathcal{K}_\infty \quad (\text{一致上下夹})$$
$$\dot V(x, t) = \frac{\partial V}{\partial t} + \nabla_x V \cdot f(x, t) \le -W(x), \quad W \text{ 正定}$$

则原点一致渐近稳定。

**关键区别**：一致上下夹 $\alpha_1(\|x\|) \le V(x,t) \le \alpha_2(\|x\|)$ **对所有 $t$ 成立**。如果 $V$ 随时间变化太剧烈（如 $V(x,t) = e^t x^2$），则不满足一致上界，定理失效。

### 为什么自治情形不需要"一致"

对自治系统，$V(x)$ 不含 $t$，所以 $\alpha_1(\|x\|) \le V(x) \le \alpha_2(\|x\|)$ 自动对所有 $t$ 一致成立。非自治情形需要显式验证。

### Barbalat 引理——非自治 LaSalle 的替代

LaSalle 不变集原理**仅对自治系统成立**（因为 $\omega$-极限集的不变性依赖于自治性）。对非自治系统，替代工具是 **Barbalat 引理**：

**引理（Barbalat 1959）**：若 $f: [0, \infty) \to \mathbb{R}$ 一致连续且 $\lim_{t \to \infty} \int_0^t f(\tau) d\tau$ 存在有限，则 $\lim_{t \to \infty} f(t) = 0$。

**应用模式**：如果能证明 $\dot V \le 0$ 且 $\ddot V$ 有界（保证 $\dot V$ 一致连续），则 $\dot V(t) \to 0$——进而如果 $\dot V = 0$ 蕴含 $x = 0$，就得到收敛。

### 常见陷阱

⚠️ **概念误区：对时变系统直接套自治定理**

非自治系统的 $V(x, t)$ 必须满足**一致夹**条件。如果 $V = (1 + \sin t) x^2$，虽然正定但上界不一致（$t$ 依赖），定理失效。

⚠️ **思维陷阱：把 Barbalat 引理等同于 LaSalle**

Barbalat 引理的结论是 $\dot V \to 0$（沿时间），不是"轨迹趋向不变集"。它没有 LaSalle 那么强的结论——但对非自治系统是唯一选择。

### 练习

1. 对系统 $\dot x = -(1 + \sin^2 t) x$，构造 Lyapunov 函数并证明一致渐近稳定。
2. 为什么 $V = e^{-t} x^2$ 不能作为一致渐近稳定的 Lyapunov 函数？（提示：检查下界）
3. 用 Barbalat 引理证明自适应系统 $\dot x = -x + \theta y$, $\dot\theta = -xy$ 中 $x(t) \to 0$。

---

## 7.18 切换系统与多 Lyapunov 函数 ⭐⭐⭐

### 动机：腿足机器人的本质是切换系统

四足机器人行走时，支撑相不断切换：左前+右后 → 右前+左后 → ... 每个支撑组合对应不同的动力学模型。数学上，这是**切换系统**：
$$\dot x = f_{\sigma(t)}(x), \quad \sigma: [0, \infty) \to \{1, 2, \ldots, N\}.$$

一个令人不安的事实：**即使每个子系统单独稳定，快速切换也可能导致整体不稳定！**

### 反例：两个稳定系统切换变不稳定

取 $A_1 = \begin{pmatrix} -1 & 10 \\ 0 & -1 \end{pmatrix}$, $A_2 = \begin{pmatrix} -1 & 0 \\ 10 & -1 \end{pmatrix}$。两者都 Hurwitz（特征值均为 $-1$）。但若以足够高的频率在二者之间切换，系统可以发散。

**物理直觉**：每个子系统在"自己的方向"上收缩，但在"垂直方向"上有暂态增长。如果切换在暂态增长阶段就发生，两个子系统轮流在对方的"弱方向"上推动状态增长——像跷跷板一样把状态推到无穷。

### 共同 Lyapunov 函数

**定理 7.18.1**：若存在 $V$ 使得 $\dot V_i = \nabla V \cdot f_i(x) < 0$ 对**所有** $i = 1, \ldots, N$ 和 $x \ne 0$ 成立，则系统在**任意切换信号**下全局渐近稳定。

对线性系统 $\dot x = A_\sigma x$，共同二次 Lyapunov $V = x^\top Px$ 存在当且仅当 LMI
$$A_i^\top P + PA_i \prec 0, \quad \forall i, \quad P \succ 0$$
可行。这是 SDP 问题，可用 MOSEK/SCS 高效求解。

**局限**：共同 $V$ 条件很强，很多实际系统不满足。

### 多 Lyapunov 函数与 Dwell-Time

**Branicky 1998** 引入每个子系统配各自 $V_i$ 的方案：每次重新切入子系统 $i$ 时，要求 $V_i(x(t_k)) \le V_i(x(t_{k-1}))$——即每次"回来"时能量不高于上次离开时。

**Dwell-time 条件**（Hespanha-Morse 1999）：若切换间隔 $\ge \tau_D$，其中
$$\tau_D > \frac{\ln \mu}{\lambda_0},$$
$\mu$ 是 $V_j/V_i$ 的跳变比上界，$\lambda_0$ 是各子系统最小衰减率，则指数稳定。

**Average dwell-time**：放宽为平均切换间隔 $\ge \tau_D$，允许偶尔快速切换。

### 与机器人的联系

- **步态切换**：不同步态相的动力学由不同的 $A_\sigma$ 描述，dwell-time 对应最小支撑相时长
- **接触切换**：足端着地/离地的切换点需要满足 dwell-time 条件
- **Poincare map**：周期步态的稳定性分析常转化为离散 Poincare map 的特征值问题

### 常见陷阱

⚠️ **概念误区：各子系统稳定 $\Rightarrow$ 切换稳定**

这是最常见的错误！如上所述，两个稳定系统快切换可以发散。必须要么有共同 $V$，要么满足 dwell-time 条件。

⚠️ **思维陷阱：dwell-time 越大越安全**

虽然大 dwell-time 保证稳定，但也限制了系统响应速度。机器人步态切换太慢会导致跌倒（来不及抬腿避障）。实际设计需要平衡稳定性与敏捷性。

### 练习

1. 对上述 $A_1, A_2$ 反例，数值仿真验证高频切换导致发散。最小使系统稳定的 dwell-time 是多少？
2. 用 LMI 检查 $A_1, A_2$ 是否存在共同二次 Lyapunov 函数（答案应为"不存在"）。
3. 设计一个腿足机器人步态切换的简化模型（2 维），验证 dwell-time 条件。

---

## 7.19 随机 Lyapunov 与 Ito 公式 ⭐⭐⭐⭐

### 动机：噪声无处不在

真实机器人的传感器有噪声、执行器有扰动、环境有随机性。用确定性 ODE 建模时忽略了这些随机因素。随机微分方程（SDE）
$$dX = b(X)dt + \sigma(X)dW$$
是更忠实的描述，其中 $W$ 是 Wiener 过程（布朗运动）。

### Ito 公式——随机 Lyapunov 的基础

对 $C^2$ 函数 $V$，Ito 公式给出：
$$dV(X_t) = \mathcal{L}V(X_t) dt + \nabla V \cdot \sigma(X_t) dW_t,$$
其中**无穷小生成元**为
$$\mathcal{L}V(x) = \sum_i b_i(x)\frac{\partial V}{\partial x_i} + \frac{1}{2}\sum_{i,j}[\sigma\sigma^\top]_{ij}(x)\frac{\partial^2 V}{\partial x_i \partial x_j}.$$

**关键区别**：与确定性的 $\dot V = \nabla V \cdot f$ 相比，多了**二阶项** $\frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)$。这是 Ito 微积分的本质——布朗运动的二次变差 $(dW)^2 = dt$ 产生额外的漂移效应。

### 随机稳定性层级

| 层级 | 定义 | 强度 |
|------|------|------|
| 概率稳定 | $\forall \varepsilon > 0: P(\|X_t\| > \varepsilon) \to 0$ | 最弱 |
| $p$-阶矩稳定 | $E[\|X_t\|^p] \to 0$ | 中 |
| 几乎必然稳定 | $P(X_t \to 0) = 1$ | 最强 |

三者**不等价**。矩稳定不蕴含几乎必然稳定（反之也不成立）。

### Khasminskii 定理

**定理 7.19.1**：若存在正定 $V$ 使 $\mathcal{L}V \le -\alpha(\|x\|)$（$\alpha \in \mathcal{K}$），则概率稳定。若 $\mathcal{L}V \le -cV$（$c > 0$），则 $p$-阶矩指数稳定。

### 常见陷阱

⚠️ **编程陷阱：随机 Lyapunov 漏掉二阶项**

仅用 $\nabla V \cdot b$ 是把随机系统当确定性处理。Ito 公式中 $\frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)$ **必须**算入。这个二阶项可以是正的也可以是负的——有时噪声反而有助于稳定（noise-induced stabilization）。

### 练习

1. 对一维 SDE $dX = -Xdt + \sigma X dW$（几何布朗运动），计算 $\mathcal{L}V$ 其中 $V = X^2/2$，讨论何时矩稳定。
2. 噪声何时有助于稳定？给出一个确定性不稳定但加噪声后稳定的例子。

---

## 7.20 Contraction Theory 简介 ⭐⭐⭐⭐

### 动机：超越平衡点的稳定性

经典 Lyapunov 分析平衡点附近的稳定性。但很多机器人任务的目标不是"趋向一个点"，而是"跟踪一条轨迹"——例如步态跟踪、末端轨迹跟踪。Contraction Theory 把 Lyapunov 从"平衡点稳定性"推广为"任意两条轨迹之间的增量稳定性"。

### 核心定义

系统 $\dot x = f(x, t)$ 称为**收缩的**（contracting），若存在黎曼度量 $M(x, t) = \Theta^\top \Theta \succ 0$ 使得广义 Jacobian
$$F = (\dot\Theta + \Theta J)\Theta^{-1}, \quad J = \frac{\partial f}{\partial x}$$
满足 $F + F^\top \preceq -2\lambda I$（$\lambda > 0$）。

**结论**：所有轨迹之间的测地距离以指数速率 $e^{-\lambda t}$ 收缩到零——**不需要知道平衡点在哪里**！

### 与 Lyapunov 的关系

| 特性 | Lyapunov | Contraction |
|------|----------|-------------|
| 分析对象 | 平衡点邻域 | 任意两条轨迹 |
| 结论 | 趋向平衡点 | 所有轨迹互相靠拢 |
| 适用 | 平衡点稳定性 | 轨迹跟踪、同步、观测器 |
| 极限环 | 不直接适用 | 可以处理 |

> **本质洞察**：Contraction 是 Lyapunov 的"微分版本"。Lyapunov 看的是 $V(x)$（关于状态），Contraction 看的是 $\delta x^\top M \delta x$（关于两条轨迹的偏差）。当系统有平衡点时，Contraction 退化为 Lyapunov 的指数稳定。

### 在 SLAM 和机器人中的应用

- **EKF-SLAM 收敛性**：Barrau-Bonnabel 2017 用 $SE(3)$ 上的 contraction 证明 IEKF 全局收敛
- **观测器设计**：contraction 观测器不需要精确初始化——任何初始估计都指数收敛到真实状态
- **Neural Contraction Metric (NCM)**：Tsukamoto-Chung-Slotine 2021 用神经网络学习收缩度量，配合 RL 做鲁棒轨迹跟踪

### 练习

1. 验证线性系统 $\dot x = Ax$（$A$ Hurwitz）在欧几里得度量 $M = I$ 下是否收缩。条件是什么？
2. 解释为什么 contraction 可以处理极限环的稳定性而经典 Lyapunov 不行。

---

## 7.21 关键定理速查表

| # | 定理名称 | 条件 | 结论 | 出处 |
|---|---------|------|------|------|
| 1 | Lyapunov 稳定性 | $V > 0$, $\dot V \le 0$ | 原点 Lyapunov 稳定 | Khalil Thm 4.1 |
| 2 | Lyapunov 渐近稳定性 | $V > 0$, $\dot V < 0$ | 原点 AS | Khalil Thm 4.1 |
| 3 | 全局渐近稳定性 | + radially unbounded | GAS | Khalil Thm 4.2 |
| 4 | Chetaev 不稳定性 | 锥形区域内 $V > 0$, $\dot V > 0$ | 不稳定 | Khalil Thm 4.3 |
| 5 | LaSalle 不变性原理 | 紧正不变集, $\dot V \le 0$ | $x(t) \to M$ | Khalil Thm 4.4 |
| 6 | 指数稳定表征 | 二次夹 + $\dot V \le -c_3\|x\|^2$ | ES | Khalil Thm 4.10 |
| 7 | Lyapunov 方程 | $A$ Hurwitz | $\exists P \succ 0: A^\top P + PA = -Q$ | Khalil Thm 4.6 |
| 8 | 间接法 | $A = Df|_0$ 的特征值 | 局部 ES 或不稳定 | Khalil Thm 4.7 |
| 9 | Massera 逆定理 | AS | $\exists C^\infty$ Lyapunov 函数 | Massera 1956 |
| 10 | Kurzweil 逆定理 | GAS | $\exists$ 全局 $C^\infty$ Lyapunov | Kurzweil 1956 |
| 11 | ISS 等价性 | ISS | $\Leftrightarrow$ 0-GAS + UBIBS $\Leftrightarrow$ $\exists$ ISS-Lyapunov | Sontag 1995 |
| 12 | ISS 小增益 | $\gamma_1 \circ \gamma_2 < \text{id}$ | 互联 ISS | Jiang-Teel-Praly 1994 |
| 13 | Artstein CLF | $\exists$ CLF + small control | 可连续镇定 | Artstein 1983 |
| 14 | Sontag 通用公式 | CLF 给定 | 显式镇定律 | Sontag 1989 |
| 15 | Contraction | $F + F^\top \preceq -2\lambda I$ | 所有轨迹指数收敛 | Lohmiller-Slotine 1998 |

---

## 7.22 常见陷阱汇总（14条）

**第一，$\dot V \le 0$ 不等于稳定。** $V$ 本身必须正定（不能只是半正定），否则 $V = 0$ 的非零点使稳定性论证崩溃。

**第二，$\dot V < 0$ 只在某邻域成立不等于 GAS。** 没有 radially unbounded 条件，level set 可能有"泄漏口"。

**第三，半负定 $\dot V$ 时误用渐近稳定。** 必须走 LaSalle，证明 $\{\dot V = 0\}$ 内最大不变集只含原点。

**第四，线性化有零实部特征值时下结论。** 间接法在临界情形**失效**，必须用非线性直接法。

**第五，混淆局部与全局。** "找到 $V$"只证明邻域稳定；要 GAS 必须 $V$ radially unbounded 且 $\dot V < 0$ 在**整个** $\mathbb{R}^n$ 上成立。

**第六，对时变/扰动系统直接套自治定理。** 非自治系统需一致稳定性，$V(x, t)$ 必须有对所有 $t$ 一致的上下夹。

**第七，把 CLF 当 Lyapunov 用。** CLF 是"存在控制使 $\dot V < 0$"，不是沿任意轨迹 $\dot V < 0$。

**第八，忽略 small control property。** Artstein 定理要求 CLF 满足 $\|x\| \to 0$ 时最优 $u \to 0$，否则反馈不连续。

**第九，SOS 搜索找到的 $V$ 未验证全局有效。** SOS 只在某 level set 内保证 $\dot V \le 0$，全局需更高阶证明。

**第十，Neural Lyapunov 训练 loss 为零不等于 Lyapunov 成立。** 必须 SMT/SOS 全域验证才是真正证书。

**第十一，非光滑系统直接用经典 Lyapunov。** Filippov 解需要 Clarke 广义梯度与集值 Lie 导数。

**第十二，切换系统误用"各自稳定"。** 各子系统独立 GAS 不蕴含切换稳定。需共同 $V$ 或 dwell-time。

**第十三，随机 Lyapunov 漏二阶项。** Ito 公式中 $\frac{1}{2}\text{tr}(\sigma\sigma^\top \nabla^2 V)$ 不可遗漏。

**第十四，Contraction 与 Lyapunov 混用。** Contraction 保证任意两轨迹收敛，不预设平衡点；极限环情形 contraction 可成立但经典 GAS 不成立。

---

## 7.23 核心教材与学习资源

### 教材对照

| 教材 | 作者 | 特色 | 难度 | 获取方式 |
|------|------|------|------|----------|
| *Nonlinear Systems* 3ed | Khalil, 2002 | 非线性控制"圣经"，证明完整 | ⭐⭐ | 机构图书馆 |
| *Applied Nonlinear Control* | Slotine-Li, 1991 | 工程导向，适合快速入门 | ⭐ | 图书馆 |
| *Mathematical Control Theory* 2ed | Sontag, 1998 | ISS 创始人视角，免费 PDF | ⭐⭐⭐ | sontaglab.org 免费 |
| *Underactuated Robotics* Ch.9 | Tedrake | 机器人 + Drake 代码 | ⭐⭐ | underactuated.mit.edu |
| *Nonlinear Dynamical Systems* | Haddad-Chellaboina, 2008 | 百科全书式，覆盖所有变种 | ⭐⭐⭐ | Springer |
| *Switching in Systems and Control* | Liberzon, 2003 | 切换系统专著 | ⭐⭐⭐ | Birkhauser |

### 视频课程

- **MIT OCW 6.243J** "Dynamics of Nonlinear Systems"：Slotine 风格，公开讲义
- **Steve Brunton "Control Bootcamp"**：直观讲解，YouTube 公开
- **DR_CAN B站**："李雅普诺夫稳定性判定方法"，中文入门首选（23+ 万播放）

### 工具链

| 工具 | 语言 | 用途 |
|------|------|------|
| SOSTOOLS | MATLAB | SOS 编程 |
| SumOfSquares.jl | Julia | SOS + JuMP 生态 |
| Drake | C++/Python | SOS-ROA + 机器人仿真 |
| dReal4 | C++ | SMT 验证 Neural Lyapunov |
| Neural-Lyapunov-Control | Python | NeurIPS 2019 原作者代码 |
| cvxpy | Python | LMI/SDP 求解 |
| scipy.linalg | Python | solve_continuous_lyapunov |

---

## 7.24 与后续专题的桥梁

**→ CLF/CBF 安全控制（专题 3.8）**：本章的 Lyapunov 函数通过添加"存在控制使 $\dot V<0$"的条件升级为 CLF（附录 E 预告）；CLF 的对偶——CBF——把"$V$ 下降到 0"翻译为"$h$ 不穿过 0"，把 LaSalle 翻译为 Nagumo，比较引理的上界版（$\dot V\le -\gamma V\Rightarrow V(t)\le V(0)e^{-\gamma t}$）翻译为下界版（$\dot h\ge -\alpha(h)\Rightarrow h(t)\ge 0$）。CLF-CBF-QP 是现代安全关键控制的核心范式——它同时要求"系统收敛到目标"（CLF 约束）和"系统永不离开安全集"（CBF 硬约束），冲突时安全优先。

**→ MPC 稳定性证明**：MPC 终端代价 $V_f$ 用 CLF 构造 + 终端集 $\mathcal{X}_f$ 控制不变 → 闭环 Lyapunov 函数 $V_N$ 单调递减（Mayne-Rawlings 2000）。Robust MPC 用 ISS 版本。

**→ DDP/iLQR**：Contraction Theory 与 DDP 的二次近似收敛性直接对接；Neural Contraction Metric 可给 DDP 全局收敛保证。

**→ SLAM 估计器收敛**：EKF 一致性用 Lyapunov；IEKF on Lie groups 用 Contraction 证明收敛。

**→ Safe RL**：Lyapunov-constrained policy optimization、neural certificate RL、shielded RL 都建立在本章之上。

**→ 分布式多智能体**：ISS 小增益定理是 consensus control 的核心；contraction 给出 network synchronization 条件。

---

## 7.25 时间预算

### 档位 3（博士入学，12-15小时）

| 小节 | 内容 | 建议学时 |
|------|------|----------|
| 7.1-7.3 | 动机 + 比较函数 + 三级稳定性 | 2h |
| 7.4 | Lyapunov 直接法完整证明 | 2.5h |
| 7.5 | LaSalle 不变集原理 + 单摆练习 | 2h |
| 7.6-7.7 | 指数稳定 + Lyapunov 方程 + Chetaev | 1.5h |
| 7.8 | Lyapunov 函数构造方法（能量/二次型） | 2h |
| 7.10-7.11 | ISS + 小增益 + 级联 | 2h |
| 7.14 | 机器人 PD 控制稳定性 | 1.5h |
| 自测+练习 | 常见陷阱 + 教材对照阅读 | 1.5h |

### 档位 4（博士毕业+，额外 8-12小时）

| 小节 | 内容 | 建议学时 |
|------|------|----------|
| 7.8 续 | SOS + Neural Lyapunov 实战 | 3h |
| 7.9 | 逆定理深入理解 | 1h |
| 7.12 | 反步法设计 + 练习 | 2h |
| 7.15 | 吸引域 SOS 优化 | 1.5h |
| 7.17 | 非自治 + Barbalat | 1h |
| 7.18 | 切换系统 + dwell-time | 1.5h |
| 7.19-7.20 | 随机 Lyapunov + Contraction 概览 | 2h |

---

## 7.26 总结

**Lyapunov 稳定性理论在 2026 年是一个"完整、计算化、机器学习化"的框架。** 从 1892 年 Lyapunov 的博士论文起，它经历了三次重大跃迁：

1. **LaSalle 1960**：把 $\dot V \le 0$ 的半负定情形"救活"为不变性原理——这让实际机械系统（几乎都只有半负定 $\dot V$）的稳定性证明变得可能。

2. **Sontag 1989 ISS**：把鲁棒性语言统一为 Input-to-State Stability——让互联系统、级联系统、分布式系统的分析有了统一工具。

3. **Parrilo 2000 SOS + Chang 2019 Neural Lyapunov**：把 Lyapunov 函数搜索从"艺术"变为"计算"——SDP 凸化和神经网络逼近让高维复杂系统的稳定性验证自动化。

**对机器人方向博士生的四条核心建议**：

第一，**亲自手算几个 Lyapunov**：单摆、机器人 PD 控制、Van der Pol——不手算永远体会不到"找 $V$"的挑战。

第二，**ISS 是论文写作的万金油**：凡是涉及扰动、噪声、模型误差、级联系统的分析，ISS + 小增益都是论证收敛性的首选。

第三，**LaSalle 是工程实践中最常用的定理**：因为物理系统的 Lyapunov 函数几乎总是只给半负定 $\dot V$，LaSalle 不变集原理是把半负定"升级"为渐近稳定的标准工具。

第四，**掌握本章就掌握了"证明机器人不会坏"的数学工具**。在 AI 安全、具身智能落地日益成为工业刚需的当下，Lyapunov certificate 已从学术追求变成部署标配——这门课是进入现代机器人安全控制研究的通行证。

> **本质洞察**：Lyapunov 稳定性理论的独特魅力在于它同时是"百年经典"和"明天的前沿论文"。从 1892 年到 2026 年，核心思想（能量函数下降）没变，但工具（SOS、Neural Net、SMT 验证）不断进化——这种"经典即前沿"的特性在数学中极为罕见。

---

## 附录 A：LaSalle 不变集原理的详细应用案例 ⭐⭐

### A.1 二自由度机械臂 PD 控制

考虑平面 2-DOF 机械臂，关节角度 $q = (q_1, q_2)^\top$。动力学：
$$M(q)\ddot q + C(q, \dot q)\dot q + g(q) = \tau.$$

其中（设连杆质量 $m_1, m_2$，长度 $l_1, l_2$，质心到关节距离 $l_{c1}, l_{c2}$）：

$$M(q) = \begin{pmatrix} \alpha + 2\beta\cos q_2 & \delta + \beta\cos q_2 \\ \delta + \beta\cos q_2 & \delta \end{pmatrix},$$
$$\alpha = m_1 l_{c1}^2 + m_2(l_1^2 + l_{c2}^2) + I_1 + I_2, \quad \beta = m_2 l_1 l_{c2}, \quad \delta = m_2 l_{c2}^2 + I_2.$$

PD + 重力补偿控制：$\tau = g(q) - K_P(q - q_d) - K_D\dot q$。

**验证 $\dot M - 2C$ 反对称性**：标准 Christoffel 参数化下
$$C(q, \dot q) = \begin{pmatrix} -\beta\sin q_2 \dot q_2 & -\beta\sin q_2(\dot q_1 + \dot q_2) \\ \beta\sin q_2 \dot q_1 & 0 \end{pmatrix}.$$

直接计算 $\dot M$：
$$\dot M = \begin{pmatrix} -2\beta\sin q_2 \dot q_2 & -\beta\sin q_2 \dot q_2 \\ -\beta\sin q_2 \dot q_2 & 0 \end{pmatrix}.$$

验证 $N = \dot M - 2C$：
$$N = \begin{pmatrix} 0 & \beta\sin q_2(2\dot q_1 + \dot q_2) \\ -\beta\sin q_2(2\dot q_1 + \dot q_2) & 0 \end{pmatrix}.$$

确实是反对称矩阵（$N^\top = -N$），因此 $\dot q^\top N \dot q = 0$ 对任意 $\dot q$ 成立——这保证了 $\dot V = -\dot q^\top K_D \dot q$ 的推导成立。

### A.2 带非线性阻尼的单摆

系统：$\ddot\theta + |\dot\theta|\dot\theta + \sin\theta = 0$（二次阻尼）。

$V = \frac{1}{2}\dot\theta^2 + (1 - \cos\theta)$。

$\dot V = \dot\theta\ddot\theta + \dot\theta\sin\theta = \dot\theta(-|\dot\theta|\dot\theta - \sin\theta) + \dot\theta\sin\theta = -|\dot\theta|^3 \le 0$。

$E = \{\dot\theta = 0\}$。在 $E$ 中若 $\dot\theta \equiv 0$，则 $\ddot\theta = 0$，即 $\sin\theta = 0$，即 $\theta = n\pi$。

在 $V < 2$ 的区域内（排除 $\theta = \pi$），最大不变集 $M = \{(0, 0)\}$。由 LaSalle 原理，原点渐近稳定。

### A.3 电路 RLC 系统

并联 RLC 电路，$V_C$ 为电容电压，$I_L$ 为电感电流：
$$C\dot V_C = -V_C/R - I_L + I_s, \qquad L\dot I_L = V_C.$$

令 $I_s = 0$（无外部电流源），取储能函数：
$$V = \frac{1}{2}CV_C^2 + \frac{1}{2}LI_L^2 \qquad (\text{电场 + 磁场能量})$$

$$\dot V = CV_C\dot V_C + LI_L\dot I_L = V_C(-V_C/R - I_L) + I_L V_C = -V_C^2/R \le 0.$$

$E = \{V_C = 0\}$。若 $V_C \equiv 0$，则 $L\dot I_L = V_C = 0$，即 $I_L = \text{const}$。但 $C\dot V_C = -I_L = 0$（由 $V_C \equiv 0$），故 $I_L = 0$。

最大不变集 $M = \{(0, 0)\}$。原点 GAS（$V$ radially unbounded）。

**跨领域类比**：电路中的 $R$ 类似于机械系统中的阻尼 $D$——它们都是耗散元件，只在"运动"（电流/速度）非零时消耗能量。LaSalle 原理处理这类"静止时不耗散"的系统模式是统一的。

---

## 附录 B：ISS 小增益定理的详细证明思路 ⭐⭐⭐

### B.1 证明的核心思想

小增益定理的证明分为两步：

**Step 1：构造全局 ISS-Lyapunov 函数。**

给定两个子系统的 ISS-Lyapunov 函数 $V_1, V_2$，构造互联系统的 Lyapunov 函数
$$V(x_1, x_2) = \max\{\sigma_1(V_1(x_1)), \sigma_2(V_2(x_2))\},$$
其中 $\sigma_1, \sigma_2 \in \mathcal{K}_\infty$ 待确定。

**Step 2：利用小增益条件选取 $\sigma_1, \sigma_2$。**

关键是选 $\sigma_1, \sigma_2$ 使得在 $V$ 达到最大值的区域内，$\dot V < 0$。小增益条件 $\gamma_1 \circ \gamma_2 < \text{id}$ 保证了这样的 $\sigma_1, \sigma_2$ 存在。

### B.2 小增益条件的几何解释

$\gamma_1 \circ \gamma_2(r) < r$ 意味着在 $(r_1, r_2)$ 平面上，曲线 $r_2 = \gamma_1(r_1)$ 和 $r_1 = \gamma_2(r_2)$ 不相交（除原点外）。它们之间的"间隙"正是系统的鲁棒性裕度。

如果 $\gamma_1 \circ \gamma_2(r) = r$（临界情形），则无裕度，系统处于稳定性边界——任何微小模型误差都可能导致不稳定。

### B.3 线性系统的特殊情况

对线性系统 $G_1, G_2$ 互联，ISS 增益退化为 $L_\infty$ 增益：$\gamma_i(r) = \|G_i\|_\infty \cdot r$。小增益条件变为
$$\gamma_1 \circ \gamma_2(r) = \|G_1\|_\infty \|G_2\|_\infty \cdot r < r \quad \Leftrightarrow \quad \|G_1\|_\infty \|G_2\|_\infty < 1.$$
这正是经典 $H_\infty$ 小增益定理。

---

## 附录 C：比较函数的高级性质与常用不等式 ⭐⭐

### C.1 class-$\mathcal{K}$ 函数的上下界关系

**引理（Khalil Lemma 4.3）**：若 $V:\mathbb{R}^n \to \mathbb{R}$ 连续正定且 radially unbounded，则存在 $\alpha_1, \alpha_2 \in \mathcal{K}_\infty$ 使得
$$\alpha_1(\|x\|) \le V(x) \le \alpha_2(\|x\|), \quad \forall x \in \mathbb{R}^n.$$

**证明**：定义 $\alpha_1(r) = \min_{\|x\| = r} V(x)$, $\alpha_2(r) = \max_{\|x\| = r} V(x)$。正定性保证 $\alpha_1(r) > 0$ 对 $r > 0$；连续性保证 $\alpha_1, \alpha_2$ 连续；radially unbounded 保证 $\alpha_1(r) \to \infty$。$\alpha_1$ 严格递增可通过适当的下界替换保证。

### C.2 Young 不等式的多种形式

**标准形式**：$ab \le \frac{a^p}{p} + \frac{b^q}{q}$，$\frac{1}{p} + \frac{1}{q} = 1$, $p, q > 1$。

**带参数形式**：$ab \le \frac{\epsilon a^2}{2} + \frac{b^2}{2\epsilon}$，$\epsilon > 0$ 任意。

**在 ISS 分析中的典型用法**：
$$\nabla V \cdot g(x) u \le |\nabla V \cdot g(x)| \cdot |u| \le \frac{\epsilon}{2}|\nabla V \cdot g(x)|^2 + \frac{1}{2\epsilon}|u|^2.$$

选 $\epsilon$ 使得 $\frac{\epsilon}{2}|\nabla V \cdot g|^2$ 能被 $-\alpha(\|x\|)$ 项吸收，剩余的 $\frac{1}{2\epsilon}|u|^2$ 作为 $\sigma(\|u\|)$。

### C.3 比较引理的离散时间版本

对离散系统 $x_{k+1} = f(x_k)$，若存在 $V$ 使 $V(f(x)) - V(x) \le -\alpha(V(x))$（$\alpha \in \mathcal{K}$, $\alpha(r) < r$），则 $V(x_k) \to 0$。

收敛速率：若 $\alpha(r) = cr$（$0 < c < 1$），则 $V(x_k) \le (1-c)^k V(x_0)$——几何衰减。

---

## 附录 D：Lyapunov 稳定性的 Python 验证代码框架 ⭐⭐

### D.1 符号计算 $\dot V$

```python
import sympy as sp

# 定义状态变量
x1, x2 = sp.symbols('x1 x2')

# 定义系统动力学 f(x)
f1 = x2                          # dot_x1
f2 = -sp.sin(x1) - 0.5 * x2     # dot_x2 (阻尼单摆)

# 定义 Lyapunov 候选函数
V = sp.Rational(1, 2) * x2**2 + (1 - sp.cos(x1))

# 计算 dot_V = dV/dx1 * f1 + dV/dx2 * f2
dV_dx1 = sp.diff(V, x1)
dV_dx2 = sp.diff(V, x2)
dot_V = dV_dx1 * f1 + dV_dx2 * f2

# 化简
dot_V_simplified = sp.simplify(dot_V)
print(f"V = {V}")
print(f"dot_V = {dot_V_simplified}")
# 输出: dot_V = -0.5*x2**2  (半负定，需要 LaSalle)
```

### D.2 数值仿真验证

```python
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def dynamics(t, x):
    """阻尼单摆动力学"""
    x1, x2 = x
    dx1 = x2
    dx2 = -np.sin(x1) - 0.5 * x2
    return [dx1, dx2]

def lyapunov_V(x):
    """Lyapunov 函数"""
    x1, x2 = x
    return 0.5 * x2**2 + (1 - np.cos(x1))

# 从多个初值仿真，验证 V 沿轨迹递减
x0_list = [[1.0, 0.5], [2.0, -1.0], [0.5, 2.0], [-1.5, 1.0]]
t_span = (0, 20)
t_eval = np.linspace(0, 20, 500)

for x0 in x0_list:
    sol = solve_ivp(dynamics, t_span, x0, t_eval=t_eval)
    V_traj = [lyapunov_V(sol.y[:, i]) for i in range(len(sol.t))]
    plt.plot(sol.t, V_traj, label=f'x0={x0}')

plt.xlabel('Time')
plt.ylabel('V(x(t))')
plt.title('Lyapunov Function Along Trajectories (Should Be Non-increasing)')
plt.legend()
plt.grid(True)
plt.show()
```

### D.3 Lyapunov 方程求解

```python
import numpy as np
from scipy.linalg import solve_continuous_lyapunov

# 系统矩阵
A = np.array([[-1, 2],
              [0, -3]])

# 选择 Q = I
Q = np.eye(2)

# 解 A^T P + P A = -Q
# scipy 的约定是 A X + X A^H = Q，所以传入 A^T 和 -Q
P = solve_continuous_lyapunov(A.T, -Q)

print(f"P = \n{P}")
print(f"P 的特征值: {np.linalg.eigvalsh(P)}")  # 应全正
print(f"验证: A^T P + P A + Q = \n{A.T @ P + P @ A + Q}")  # 应近似零

# 计算衰减率
c1 = np.min(np.linalg.eigvalsh(P))
c2 = np.max(np.linalg.eigvalsh(P))
c3 = np.min(np.linalg.eigvalsh(Q))
lambda_rate = c3 / (2 * c2)
k_overshoot = np.sqrt(c2 / c1)
print(f"衰减率 lambda = {lambda_rate:.4f}")
print(f"超调系数 k = {k_overshoot:.4f}")
print(f"||x(t)|| <= {k_overshoot:.2f} * ||x(0)|| * exp(-{lambda_rate:.4f} * t)")
```

**代码说明**：上述代码演示了三种 Lyapunov 分析的计算方法——符号求 $\dot V$、数值仿真验证、Lyapunov 方程求解。在实际研究中，这三者通常配合使用：符号计算给出理论结果，数值仿真提供直觉验证，Lyapunov 方程给出定量衰减估计。

---

## 附录 E：从 Lyapunov 到 Control Lyapunov Function (CLF) ⭐⭐⭐

### E.1 CLF 的精确定义

对控制仿射系统 $\dot x = f(x) + g(x)u$，$V$ 是 CLF 若：
1. $V$ 正定且 radially unbounded
2. $\inf_{u \in \mathbb{R}^m} [L_f V(x) + L_g V(x) u] < 0$ 对 $x \ne 0$

其中 $L_f V = \nabla V \cdot f$, $L_g V = \nabla V \cdot g$ 是 Lie 导数。

**与 Lyapunov 函数的关系**：Lyapunov 函数要求 $\dot V < 0$ 对给定系统成立；CLF 只要求"**存在某个 $u$** 使 $\dot V < 0$"——更弱，但足以保证可镇定性。

### E.2 Sontag 通用公式

**定理（Sontag 1989）**：给定 CLF $V$，令 $a(x) = L_f V(x)$, $b(x) = L_g V(x)$。则

$$u^*(x) = \begin{cases} -\dfrac{a + \sqrt{a^2 + (b b^\top)^2}}{b b^\top} b^\top, & b \ne 0 \\ 0, & b = 0 \end{cases}$$

是连续的镇定反馈律（除原点外），满足 small control property。

**直觉**：当 $b = 0$ 时（控制对 $\dot V$ 无影响），CLF 条件保证 $a < 0$（系统本身就在收敛）。当 $b \ne 0$ 时，公式选择"刚好让 $\dot V$ 负定"的最小能量控制。

### E.3 CLF-QP 控制器

把 CLF 条件写为 QP 约束：
$$\min_{u} \frac{1}{2}\|u\|^2 \quad \text{s.t.} \quad L_f V + L_g V \cdot u \le -\gamma(V(x)),$$
其中 $\gamma \in \mathcal{K}$ 控制收敛速率。

**优势**：
- 最小能量控制（省力）
- 可以加额外约束（力矩饱和、速度限制）
- 用 OSQP 可 1 kHz+ 实时求解
- 是 CLF-CBF-QP（安全控制）的直接基础

### E.4 从 CLF 到 MPC 终端代价

MPC 闭环稳定性的标准方法（Mayne-Rawlings 2000）是取终端代价 $V_f$ 为局部 CLF：
- 终端代价 $V_f$ 满足 $\dot V_f(x, \kappa_f(x)) \le -\ell(x, \kappa_f(x))$ 在终端集 $\mathcal{X}_f$ 内
- 终端集 $\mathcal{X}_f = \{x: V_f(x) \le c\}$ 在局部反馈 $\kappa_f$ 下正不变

这使得 MPC 的值函数 $V_N = \sum \ell + V_f$ 成为闭环 Lyapunov 函数——完美连接了 Lyapunov 理论与最优控制。

---

## 附录 F：补充练习题（跨章综合）⭐⭐

### F.1 综合题：观测器-控制器分离的 ISS 分析

考虑系统 $\dot x = f(x) + g(x)u$，状态反馈 $u = k(\hat x)$ 基于观测器估计 $\hat x$：
$$\dot{\hat x} = f(\hat x) + g(\hat x)k(\hat x) + L(y - h(\hat x)).$$

1. 定义估计误差 $e = x - \hat x$，写出误差动力学。
2. 假设控制器以真实状态为输入时闭环 GAS（Lyapunov 函数 $V_1$），观测器误差动力学 ISS 关于 $u$（ISS-Lyapunov 函数 $V_2$）。说明如何用小增益定理证明整体稳定性。
3. 讨论：线性系统的分离原理（观测器与控制器独立设计）在非线性情形为什么一般不成立？ISS 小增益给出的是什么替代方案？

### F.2 综合题：从能量整形到 Lyapunov 证明

考虑欠驱动机械系统（如 cart-pole）：
$$M(q)\ddot q + C(q, \dot q)\dot q + g(q) = B\tau,$$
其中 $B$ 不满秩（欠驱动）。

1. 为什么能量法不能直接给出 GAS？（提示：不可控自由度无法通过阻尼耗散）
2. 解释"能量整形"（energy shaping）如何修改势能 $U(q) \to U_d(q)$ 使得新平衡点成为能量最小值。
3. 对修改后的系统，写出新的 Lyapunov 函数 $V_d = T + U_d$ 并验证 $\dot V_d \le 0$。
4. 用 LaSalle 原理完成渐近稳定性证明。

### F.3 计算题：SOS-ROA 估计

对系统 $\dot x_1 = x_2$, $\dot x_2 = -x_1 + x_1^3 - x_2$（Duffing 振子变种）：

1. 验证原点是局部渐近稳定的（线性化 + Lyapunov 方程）。
2. 用 $V = x^\top P x$（$P$ 来自 Lyapunov 方程）估计吸引域（找最大 $c$ 使 $\dot V < 0$ 在 $\Omega_c$ 内）。
3. 写出用 SOS 优化改进吸引域估计的完整数学规划。（选择 4 次多项式 $V$，写出 SOS 约束）

---

## 附录 G：Lyapunov 方法的历史演进与学术生态 ⭐

### G.1 从 Lagrange 到 Lyapunov

Lyapunov 稳定性的思想并非凭空出现。它有深厚的历史根源：

**1788 年，Lagrange**：在《分析力学》中已经用能量论证讨论了机械系统平衡的稳定性——"势能极小值处的平衡是稳定的"。这是最早的"能量函数判稳"思想，但仅限于保守系统。

**1877 年，Dirichlet**：给出了 Lagrange 稳定性定理的严格证明（"Dirichlet 稳定性定理"），指出势能严格极小值处平衡稳定。但对非保守系统（有耗散或外力）无法处理。

**1892 年，Lyapunov**：在哈尔科夫大学的博士论文中做了根本性推广——**$V$ 不再局限于物理能量，可以是任何满足条件的标量函数**。这一抽象化是决定性的一步：它把方法从"物理直觉"变为"数学工具"，适用于任何 ODE 系统（不只是力学系统）。

**1960 年，LaSalle**：在 IRE 电路理论期刊上发表不变集原理。同期 Krasovskii（苏联）独立得到类似结果。这一时期美苏冷战，双方学术独立发展但结论惊人一致——说明这些定理是数学的"内在必然"。

**1989 年，Sontag**：ISS 理论的提出标志着 Lyapunov 从"分析工具"向"设计工具"的全面转型。ISS 给出了鲁棒性的统一语言，使得大规模互联系统（网络、多机器人）的分析成为可能。

**2000 年后，计算化时代**：SOS（Parrilo 2000）、Neural Lyapunov（Chang 2019）、Contraction Metrics（Manchester-Slotine 2017）把 Lyapunov 从"纸上的定理"变为"计算机可执行的算法"。

### G.2 当前学术生态图谱

| 方向 | 代表性研究组 | 关键工具 |
|------|------------|---------|
| SOS/ROA 优化 | MIT Tedrake 组, Caltech Ahmadi | Drake, SOSTOOLS |
| Neural Lyapunov | UCSD Gao 组, MIT Fan/REALM | dReal, FOSSIL |
| ISS/互联 | Rutgers Sontag (retired), U. Passau Mironchenko | 理论 |
| Contraction | MIT Slotine 组, Stanford Pavone | Julia, JAX |
| CLF/CBF | Caltech Ames 组, Berkeley Sreenath | OSQP, CasADi |
| 切换/混合系统 | UIUC Liberzon, UT Sanfelice | 理论+仿真 |
| 随机稳定性 | Imperial Astolfi, KTH Hu | 理论 |

### G.3 开放问题（2026 年视角）

1. **高维 SOS 的可扩展性**：当状态维度 $n > 10$ 时，SOS 计算量爆炸。DSOS/SDSOS（Ahmadi 2019）是初步解决方案，但仍有差距。

2. **Neural Lyapunov 的验证 gap**：SMT 验证对深层网络/高维系统仍然困难。如何让验证跟上网络的表达力？

3. **非光滑系统的计算工具**：机器人接触动力学本质非光滑，但现有 SOS/Neural 方法主要针对光滑系统。非光滑 Lyapunov 的计算化是一个开放挑战。

4. **Lyapunov 与大语言模型**：能否让 LLM 辅助构造 Lyapunov 函数？（目前看来符号推理仍然困难）

5. **数据驱动的 Lyapunov**：当系统模型未知时，如何从数据直接学习 Lyapunov 函数？Berkenkamp 2017 的 GP 方法是开端，但理论保证仍不完善。

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| 找不到 Lyapunov 函数就说明系统不稳定 | 逆定理（Massera、Kurzweil）保证稳定系统一定存在 Lyapunov 函数，找不到只是构造能力不足，不能反推不稳定 |
| $\dot V \leq 0$ 就能证明渐近稳定 | $\dot V \leq 0$（半负定）只能证明 Lyapunov 稳定，渐近稳定需要 $\dot V < 0$（负定）或借助 LaSalle 不变集原理排除 $\dot V = 0$ 的非零不变集 |
| 能量函数总是合适的 Lyapunov 函数 | 物理能量是好的起点，但并非总够用——例如无阻尼系统的能量守恒（$\dot V = 0$），无法判断渐近稳定；需要构造修正能量或交叉项 |
| 线性化判据（间接法）适用于所有情况 | 当线性化矩阵有纯虚特征值时，间接法失效——稳定性由非线性项决定，必须用直接法（Lyapunov 函数）判断 |
| 指数稳定和渐近稳定本质上没区别 | 指数稳定给出明确的收敛速率 $\|x(t)\| \leq c \|x_0\| e^{-\alpha t}$，在鲁棒性分析（ISS、小增益定理）中提供定量保证，而渐近稳定只保证最终收敛 |
| ISS 系统在任意大的输入下仍然稳定 | ISS 保证的是"有界输入 $\Rightarrow$ 有界状态"，且当输入消失后状态渐近收敛；但对于极大输入，状态偏移可以任意大 |
| SOS（平方和）方法可以解决任意维度的稳定性验证 | SOS 的计算复杂度随状态维度急剧增长，当 $n > 10$ 时通常不可行；DSOS/SDSOS 和 Neural Lyapunov 是高维替代方案 |
| 全局渐近稳定（GAS）意味着从任何初始状态都以相同速率收敛 | GAS 只保证最终收敛，收敛速率可以随初始状态距离增大而急剧变慢；全局指数稳定（GES）才给出一致的收敛速率保证 |

---

## 附录 H：符号约定与术语对照表

| 符号 | 含义 | 首次出现 |
|------|------|----------|
| $\dot x = f(x)$ | 自治系统 | 7.3 |
| $V(x)$ | Lyapunov 函数候选 | 7.4 |
| $\dot V(x)$ | $V$ 沿轨迹的时间导数 $= \nabla V \cdot f$ | 7.4 |
| $\Omega_c$ | sublevel set $\{x: V(x) \le c\}$ | 7.4 |
| $\mathcal{K}, \mathcal{K}_\infty, \mathcal{KL}$ | 比较函数类 | 7.2 |
| $E$ | $\{x: \dot V(x) = 0\}$ | 7.5 |
| $M$ | $E$ 中最大不变集 | 7.5 |
| $\omega(x_0)$ | 轨迹的 $\omega$-极限集 | 7.5 |
| $\mathcal{R}$ | 吸引域（Region of Attraction） | 7.15 |
| $\gamma \in \mathcal{K}$ | ISS 增益函数 | 7.10 |
| $\chi \in \mathcal{K}$ | ISS 阈值函数 | 7.10 |
| $\mathcal{L}V$ | Ito 无穷小生成元 | 7.19 |
| $M(x, t)$ | 收缩度量（Riemannian metric） | 7.20 |

| 中文术语 | 英文术语 | 缩写 |
|---------|---------|------|
| 李雅普诺夫稳定 | Lyapunov stable | S |
| 渐近稳定 | Asymptotically stable | AS |
| 指数稳定 | Exponentially stable | ES |
| 全局渐近稳定 | Globally asymptotically stable | GAS |
| 输入到状态稳定 | Input-to-State Stability | ISS |
| 正不变集 | Positively invariant set | — |
| 吸引域 | Region of Attraction | ROA |
| 不变集原理 | Invariance principle | — |
| 逆定理 | Converse theorem | — |
| 平方和 | Sum of Squares | SOS |
| 控制李雅普诺夫函数 | Control Lyapunov Function | CLF |
| 小增益定理 | Small-gain theorem | — |
| 反步法 | Backstepping | — |
| 收缩分析 | Contraction analysis | — |
| 驻留时间 | Dwell time | — |
| 级联系统 | Cascade system | — |
| 一致渐近稳定 | Uniformly asymptotically stable | UAS |
| 积分 ISS | integral ISS | iISS |

---

## 附录 I：关键论文引用清单

### 奠基性原著

- **Lyapunov 1892/1992**："The General Problem of the Stability of Motion", trans. A.T. Fuller, *Int. J. Control* 55(3):531-773
- **LaSalle 1960**："Some Extensions of Liapunov's Second Method", *IRE Trans. Circuit Theory* CT-7(4):520-527
- **Massera 1956**："Contributions to stability theory", *Annals of Mathematics* 64(1):182-206
- **Kurzweil 1956**：*Czechoslovak Math. J.* 6(81):217-259

### ISS 系列

- **Sontag 1989**："Smooth stabilization implies coprime factorization", *IEEE TAC* 34(4):435-443
- **Sontag 1998**："Comments on integral variants of ISS", *SCL* 34(1-2):93-100
- **Jiang-Teel-Praly 1994**："Small-gain theorem for ISS systems", *MCSS* 7(2):95-120
- **Lin-Sontag-Wang 1996**："A smooth converse Lyapunov theorem", *SIAM JCO* 34(1):124-160

### 计算方法

- **Parrilo 2000**：Caltech PhD thesis（SOS 奠基）
- **Parrilo 2003**：*Math. Programming* 96(2):293-320
- **Ahmadi-Majumdar 2019**："DSOS and SDSOS", arXiv:1706.02586
- **Chang-Roohi-Gao 2019**："Neural Lyapunov Control", *NeurIPS 2019*

### Contraction 与 CLF

- **Lohmiller-Slotine 1998**："On Contraction Analysis", *Automatica* 34(6):683-696
- **Manchester-Slotine 2017**："Control Contraction Metrics", *IEEE TAC* 62(6):3046-3053
- **Artstein 1983**："Stabilization with relaxed controls", *Nonlinear Analysis* 7(11):1163-1173
- **Sontag 1989**："A universal construction", *SCL* 13(2):117-123

### 切换与随机

- **Branicky 1998**："Multiple Lyapunov functions", *IEEE TAC* 43(4):475-482
- **Hespanha-Morse 1999**："Stability with average dwell-time", *CDC* pp.2655-2660
- **Khasminskii 2012**：*Stochastic Stability of Differential Equations*, Springer 2ed

---
