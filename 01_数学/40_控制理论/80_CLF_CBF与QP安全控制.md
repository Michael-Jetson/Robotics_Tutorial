# CLF/CBF 与 QP 综合安全控制

## 前置自测

📋 **前置自测**（答不出 ≥ 2 题 → 先回专题 3.7 Lyapunov 稳定性复习）

1. Lyapunov 函数 $V(x)$ 需要满足什么条件才能保证系统平衡点渐近稳定？写出 $\dot V$ 的条件。
2. 什么是类 $\mathcal{K}$ 函数和类 $\mathcal{K}_\infty$ 函数？给出定义并举例。
3. 比较引理（Comparison Lemma）的陈述是什么？如何从微分不等式推出解的界？
4. 对于控制仿射系统 $\dot x = f(x) + g(x)u$，$L_f V$ 和 $L_g V$ 分别表示什么？如何计算？
5. QP（二次规划）的标准形式是什么？在什么条件下 QP 有唯一解？

---

## 本章目标

学完本章后，你应当能够：

1. **理解并证明** Nagumo 正不变性定理，掌握集合不变性的数学基础
2. **构造** CLF-CBF-QP 统一安全控制器，包括松弛变量设计与 KKT 条件分析
3. **处理** 高相对度系统的安全约束（HOCBF/ECBF），并在实际系统上实现
4. **实现** 基于 OSQP 的实时安全滤波器，达到 1 kHz 控制频率

---

## 知识树

```
CLF-CBF-QP 安全控制
├── 1. 集合不变性基础
│   ├── 正向不变集定义
│   ├── 切锥与 Nagumo 定理
│   └── 从 Nagumo 到 CBF 的推广动机
├── 2. Control Lyapunov Function (CLF)
│   ├── CLF 定义与 Artstein 定理
│   ├── Sontag 公式（显式稳定反馈）
│   └── CLF-QP
├── 3. Control Barrier Function (CBF)
│   ├── Zeroing CBF 定义与主定理
│   ├── 比较引理与前向不变性证明
│   ├── Reciprocal CBF 对比
│   ├── 高阶 CBF (HOCBF)
│   └── 指数 CBF (ECBF)
├── 4. CLF-CBF-QP 统一框架
│   ├── QP 构造与可行性
│   ├── 松弛变量与优先级
│   ├── KKT 条件闭式解
│   └── Safety Filter 变体
├── 5. 多约束与工程实现
│   ├── 多障碍物处理
│   ├── 输入约束相容性
│   ├── OSQP 实时求解
│   └── ROS2 集成
├── 6. 高级主题
│   ├── Robust CBF (ISSf)
│   ├── MPC-CBF 统一
│   ├── Neural CBF
│   └── CBF + RL Safety Filter
└── 7. 应用实例
    ├── 自适应巡航控制 (ACC)
    ├── 四旋翼安全着陆
    └── 多机器人避障
```

---

## 0 节：为什么 CBF 是"安全"的 Lyapunov ⭐

### 动机：稳定性不等于安全性

专题 3.7 教会了你用 Lyapunov 函数证明 **"系统会收敛到目标"**；本专题要教你用控制屏障函数（Control Barrier Function, CBF）证明 **"系统永远不会离开安全区"**。两者是一对对偶概念：前者刻画 **稳定性** ($V(x)\to 0$)，后者刻画 **安全性** ($h(x)\ge 0$)。

为什么需要单独讨论安全性？一个稳定的系统在收敛过程中完全可能穿过危险区域。考虑一个简单例子：机械臂从 A 点移到 B 点，Lyapunov 控制器保证它最终到达 B，但移动路径可能穿过工人的工作区域。**稳定性保证的是最终行为（$t\to\infty$），安全性保证的是所有时刻的行为（$\forall t\ge 0$）**。

> **跨领域类比**：Lyapunov 函数像 GPS 导航告诉你"一定能到目的地"，CBF 像护栏保证你"永远不会掉下悬崖"。GPS 可能给你一条经过悬崖边缘的最短路线——如果没有护栏，你虽然终将到达目的地，但过程中可能坠落。CLF-CBF-QP 就是同时有 GPS 和护栏的系统。

对一个做 RL motion control 或具身智能的博士生，这对双生工具至关重要——**RL 策略给出的"名义动作"可能不安全，CBF-QP 作为 Safety Filter 在毫秒级时间内对其做最小修正，从而保证硬约束被满足**。

### 历史脉络

这类方法的学术脉络可追溯到 1942 年 Nagumo 关于微分方程积分曲线位置的工作。现代 CBF 理论在 2014-2017 年由 Aaron Ames（Caltech/Georgia Tech）系统化：

- **Ames et al. 2014** (CDC): 首次提出 CBF-QP 框架的雏形
- **Xu-Tabuada-Grizzle-Ames 2015** (HSCC): Robustness of CBF
- **Ames-Xu-Grizzle-Tabuada 2017** (IEEE TAC 62(8):3861-3876, arXiv:1609.06408): 奠基性论文，统一了 ZCBF/RCBF 定义，给出 CLF-CBF-QP 完整框架
- **Ames et al. 2019** (ECC tutorial, arXiv:1903.11199): 最佳入门教程

此后进入主流，已是 Boston Dynamics、ANYbotics、Waymo、Tesla、NVIDIA Isaac 等工业级机器人栈的标配模块。

### CBF 与 CLF 的对偶表

| 概念 | CLF（稳定性） | CBF（安全性） |
|---|---|---|
| 集合 | 目标集 $\{V=0\}$ | 安全集 $\mathcal{C}=\{h\ge 0\}$ |
| 目标 | 吸引（$V\to 0$） | 不变（$h\ge 0$ 永远成立） |
| 核心不等式 | $\dot V \le -\gamma(V)$ | $\dot h \ge -\alpha(h)$ |
| 不等式方向 | 上界（"至少以 $\gamma(V)$ 下降"） | 下界（"至多以 $\alpha(h)$ 下降"） |
| 比较引理结论 | $V(t)\to 0$ | $h(t)\ge 0$ 保持 |
| 工具定理 | LaSalle 不变性原理 | Nagumo 正不变性定理 |
| QP 角色 | CLF-QP（保证收敛） | CBF-QP（保证安全） |
| 冲突时优先级 | 可松弛（软约束） | 不可松弛（硬约束） |

读这份讲义时，建议不断做这个翻译——每当看到 CBF 的一个结论，问自己"对应的 CLF 版本是什么？"

---

## 1. 集合不变性基础 ⭐⭐

### 1.1 正向不变集定义与判定

#### 动机：为什么需要"集合不变性"

在控制理论中，我们经常关心这样一个问题：**如果系统从某个"好的"区域出发，它会一直待在这个区域里吗？** 这个问题的数学形式化就是正向不变性。

考虑自治系统 $\dot x = f(x)$，$x\in\mathbb{R}^n$，$f$ 局部 Lipschitz 连续（保证解存在唯一）。

**定义（正向不变集）**：集合 $\mathcal{C}\subseteq\mathbb{R}^n$ 称为系统 $\dot x=f(x)$ 的 **正向不变集**（forward invariant set），如果对任意初值 $x(0)\in\mathcal{C}$，解 $x(t)$ 满足 $x(t)\in\mathcal{C}$ 对所有 $t\ge 0$ 成立。

**直觉**：一旦进入这个集合，就再也出不去了。集合像一个"陷阱"——但这里是好的陷阱，因为 $\mathcal{C}$ 是我们想让系统待着的安全区域。

**如果不考虑不变性会怎样？** 你可能会设计一个控制器，使系统朝着安全集移动（类似 Lyapunov 的收敛），但在到达之前路径穿过了不安全区域。或者系统在安全集内部运动，但某个时刻的速度场方向指向集合外——系统将在下一时刻离开安全集。正向不变性排除了这两种情况。

#### 用水平集定义安全集

在 CBF 理论中，安全集 $\mathcal{C}$ 通常由一个标量函数 $h:\mathbb{R}^n\to\mathbb{R}$ 的零超水平集（zero-superlevel set）定义：

$$\mathcal{C}=\{x\in\mathbb{R}^n : h(x)\ge 0\}$$
$$\partial\mathcal{C}=\{x\in\mathbb{R}^n : h(x)= 0\} \quad \text{（边界）}$$
$$\operatorname{Int}(\mathcal{C})=\{x\in\mathbb{R}^n : h(x)> 0\} \quad \text{（内部）}$$

**正则性条件**：要求 $\nabla h(x)\neq 0$ 对所有 $x\in\partial\mathcal{C}$。这保证边界是一个光滑的超曲面（$n-1$ 维流形），没有尖角或奇点。

> **本质洞察**：为什么用 $h(x)\ge 0$ 而不是 $h(x)\le c$ 或其他形式？因为这种约定使得 $\nabla h$ 在边界上**指向安全集内部**。当 $\dot h = \nabla h \cdot f > 0$ 时，速度场方向与 $\nabla h$ 同向，即系统向安全集内部移动。这个符号约定使得后续所有不等式方向统一为 "$\ge$"，避免了混乱。

**例子：圆形安全区域**

设机器人位置 $x=(x_1,x_2)$，安全区域是半径为 $R$ 的圆盘 $\{x: \|x\|\le R\}$。定义：
$$h(x) = R^2 - x_1^2 - x_2^2$$

则 $h(x)\ge 0 \iff \|x\|\le R$，$\nabla h = (-2x_1, -2x_2)$。在边界 $\|x\|=R$ 上 $\nabla h \neq 0$（只有原点处为零，但原点不在边界上）。

**例子：距障碍物的安全距离**

障碍物在 $x_{\text{obs}}$，安全距离 $d_{\text{safe}}$：
$$h(x) = \|x - x_{\text{obs}}\|^2 - d_{\text{safe}}^2$$

$h(x)\ge 0$ 表示机器人到障碍物距离 $\ge d_{\text{safe}}$。

### 1.2 切锥与 Nagumo 定理 ⭐⭐

#### 动机：边界上的速度方向决定不变性

正向不变性的关键在于：**当系统状态位于安全集边界时，速度场的方向是否指向集合内部（或至少沿边界切线方向）？** 如果速度场指向集合外部，系统将在下一时刻离开安全集。

#### 切锥的定义

**定义（Bouligand 切锥）**：集合 $\mathcal{C}$ 在点 $x\in\mathcal{C}$ 处的切锥定义为：
$$T_{\mathcal{C}}(x) = \left\{v\in\mathbb{R}^n : \liminf_{\tau\downarrow 0} \frac{\mathrm{dist}(x+\tau v, \mathcal{C})}{\tau} = 0 \right\}$$

其中 $\mathrm{dist}(y, \mathcal{C}) = \inf_{z\in\mathcal{C}}\|y-z\|$。

**直觉**：切锥包含所有"不会立刻离开集合"的方向。如果 $v\in T_{\mathcal{C}}(x)$，从 $x$ 出发沿 $v$ 方向走一小步，要么还在 $\mathcal{C}$ 里，要么离 $\mathcal{C}$ 的距离比步长衰减得更快（即"擦着边界走"）。

**对于水平集 $\mathcal{C}=\{h\ge 0\}$**：当 $x\in\partial\mathcal{C}$（即 $h(x)=0$）且 $\nabla h(x)\neq 0$ 时：
$$T_{\mathcal{C}}(x) = \{v : \nabla h(x)\cdot v \ge 0\}$$

这是以 $\nabla h(x)$ 为法向量的半空间——恰好是"不让 $h$ 减小"的方向集合。

#### Nagumo 定理的完整陈述与证明

**定理（Nagumo 1942，水平集形式）**：设 $\mathcal{C}=\{x:h(x)\ge 0\}$，$h\in C^1$（连续可微），$\nabla h(x)\neq 0$ 对 $x\in\partial\mathcal{C}$。考虑系统 $\dot x = f(x)$，$f$ 局部 Lipschitz。则 $\mathcal{C}$ 对该系统前向不变当且仅当：
$$\dot h(x) = \nabla h(x)\cdot f(x) \ge 0 \quad \forall x\in\partial\mathcal{C}$$

即：在边界上，$h$ 沿轨迹的导数非负（系统不会让 $h$ 减小到负值）。

**证明（充分性 — 反证法）**：

假设条件成立但 $\mathcal{C}$ 不前向不变。则存在 $x_0\in\mathcal{C}$ 使得轨迹 $x(t;x_0)$ 在某 $t_1>0$ 时离开 $\mathcal{C}$，即 $h(x(t_1))<0$。

**Step 1**：若 $x_0\in\operatorname{Int}(\mathcal{C})$（即 $h(x_0)>0$），由连续性，轨迹在离开 $\mathcal{C}$ 之前必须经过边界。定义离开时刻：
$$t^* = \inf\{t>0 : h(x(t))<0\}$$

由 $h$ 的连续性，$h(x(t^*))=0$，且 $h(x(t))\ge 0$ 对 $t\in[0,t^*]$。

**Step 2**：在 $t^*$ 处，$h$ 从非负变为负。由右导数的定义：
$$\dot h(x(t^*)) = \lim_{\epsilon\downarrow 0} \frac{h(x(t^*+\epsilon)) - h(x(t^*))}{\epsilon} = \lim_{\epsilon\downarrow 0} \frac{h(x(t^*+\epsilon))}{\epsilon}$$

由于 $t^*$ 之后存在 $h<0$ 的时刻（否则 $t^*$ 不是 inf），必有 $\dot h(x(t^*))\le 0$。

**Step 3**：但 $x(t^*)\in\partial\mathcal{C}$，由假设条件 $\dot h(x(t^*))\ge 0$。

结合得 $\dot h(x(t^*))=0$。

**Step 4（排除零导数离开的情况）**：$\dot h=0$ 意味着轨迹在 $t^*$ 时刻"切线接触"边界。利用正则性 $\nabla h(x(t^*))\neq 0$ 和隐函数定理，边界 $\partial\mathcal{C}$ 在 $x(t^*)$ 附近是光滑超曲面。由于 $f$ Lipschitz 连续且 $\nabla h\cdot f\ge 0$ 在边界上，可以证明在 $t^*$ 附近不存在使 $h$ 严格递减的轨迹段。具体地，对充分小的 $\epsilon>0$：

$$h(x(t^*+\epsilon)) = h(x(t^*)) + \dot h(x(t^*))\epsilon + O(\epsilon^2) = O(\epsilon^2)$$

若 $\ddot h(x(t^*))<0$，则 $h(x(t^*+\epsilon))<0$，看似矛盾条件只要求 $\dot h\ge 0$。但更精细的分析（利用 Gronwall 不等式或比较引理的变体）表明，仅当 $\dot h=0$ 且轨迹即将离开时，必须 $\nabla h\cdot f<0$ 在离开前的某一时刻成立——与条件矛盾。$\blacksquare$

**证明（必要性）**：

若存在 $x_0\in\partial\mathcal{C}$ 使 $\dot h(x_0) = \nabla h(x_0)\cdot f(x_0)<0$。由 $h(x_0)=0$ 和 $\dot h(x_0)<0$，对充分小的 $\epsilon>0$：
$$h(x(\epsilon;x_0)) = h(x_0) + \dot h(x_0)\epsilon + O(\epsilon^2) = \dot h(x_0)\epsilon + O(\epsilon^2) < 0$$

即轨迹立即离开 $\mathcal{C}$，与前向不变矛盾。$\blacksquare$

#### Nagumo 定理的局限与推广动机

**Nagumo 的局限**：它只要求在边界 $\partial\mathcal{C}$（$h=0$）上 $\dot h\ge 0$，对 $\mathcal{C}$ 内部（$h>0$）没有任何要求。这在**分析**上完全足够（判定一个已知系统是否保持不变），但在**控制综合**（synthesis）中不够——因为：

1. 我们需要一个**全域条件**来构造 QP 约束（在任何状态 $x$ 处都能判断哪些 $u$ 是安全的）
2. 仅在边界处有约束意味着系统可能以任意高速冲向边界，到边界时才"急刹车"——这对有输入约束的系统不现实

> **反事实推理**：如果我们只用 Nagumo 条件来设计控制器，会发生什么？控制器只在 $h(x)\approx 0$ 时才"紧张"并施加修正，在 $h(x)>0$ 时完全不管安全——就像一辆车只在即将撞墙时才踩刹车。如果车速太快，刹车力有限（输入约束），就会撞上去。CBF 通过在远离边界处就施加温和约束（"限速区"），解决了这个问题。

**CBF 的推广策略**：允许 $h>0$ 时 $\dot h$ 为负（系统可以向边界靠近），但衰减速率被 $\alpha(h)$ 限制：
$$\dot h(x) \ge -\alpha(h(x))$$

当 $h$ 大（远离边界）时 $\alpha(h)$ 大，允许较快靠近；当 $h$ 小（接近边界）时 $\alpha(h)$ 小，强制减速。比较引理保证在此条件下 $h(t)\ge 0$。

> **跨领域类比**：Nagumo 条件像"绝对禁止越线"——只在边界上起作用，线内完全自由。CBF 条件像高速公路的"递减限速区"——离出口 2km 时限速 120，1km 时限速 80，500m 时限速 60。这种渐进式约束让车辆（系统）总能安全停下来，即使制动力有限。

### ⚠️ 常见陷阱

> 💡 **概念误区：混淆 Nagumo 条件与 CBF 条件**
>
> **新手想法**："$\dot h\ge 0$ 和 $\dot h\ge -\alpha(h)$ 不就差一个 $\alpha(h)$ 吗？"
>
> **实际差别**：两者有本质不同。Nagumo 条件 $\dot h\ge 0$ 只需在边界（$h=0$）上满足，内部无要求。CBF 条件 $\dot h\ge -\alpha(h)$ 在**整个状态空间**都有要求（因为 $\alpha(h)>0$ 当 $h>0$）。这意味着 CBF 条件在远离边界处也限制了系统接近边界的速率。

> 🧠 **思维陷阱：认为"安全"只需要在边界处理**
>
> **错误推理**："系统在内部安全无虞，只需在接近边界时干预。"
>
> **后果**：如果系统有惯性（如双积分器），即使在边界处施加最大制动力也可能来不及停下。HOCBF 就是为了处理这种高相对度情况而生——它从更远处就开始约束"速度"和"加速度"。
>
> **正确理解**：安全控制是一个**全程**问题，不是**边界**问题。CBF 的 $\alpha(h)$ 函数正是这种全程约束的体现。

### 练习 1.1

1. 对系统 $\dot x = -x + u$，安全集 $\mathcal{C}=\{x: x\ge -1\}$（即 $h(x)=x+1$），验证 Nagumo 条件，求使 $\mathcal{C}$ 不变的 $u$ 的充分条件。
2. 对二维系统 $\dot x_1 = x_2$，$\dot x_2 = u$，安全集为圆盘 $\{x: x_1^2+x_2^2\le 4\}$，Nagumo 条件是什么？为什么直接用 Nagumo 不够好？
3. **（跨章综合）** 回忆专题 3.7 中 Lyapunov 定理的证明方法，与 Nagumo 定理充分性证明比较：两者都用了什么共同的论证策略？

---

## 2. Control Lyapunov Function (CLF) ⭐⭐

### 2.1 CLF 的定义与 Artstein 定理

#### 动机：从 Lyapunov 到"控制" Lyapunov

回顾专题 3.7（§7.4 Lyapunov 直接法）：对自治系统 $\dot x=f(x)$，若存在正定、径向无界的 $V$ 使得 $\dot V(x)=\nabla V\cdot f(x)\le -\alpha(V)$（$\alpha\in\mathcal{K}$），则原点全局渐近稳定。其中 $\dot V$ 负定是关键——它保证 $V$ 沿轨迹严格递减（当 $\dot V$ 仅半负定时需借助 LaSalle 不变集原理，见 §7.5）。但这是**分析**工具——给定系统 $f$，判断是否稳定。

现在我们有控制输入：$\dot x = f(x)+g(x)u$。问题变为：**是否存在一个 $V$，使得对任意 $x\neq 0$，都能找到 $u$ 让 $\dot V<0$？** 如果答案是肯定的，这个 $V$ 就是 CLF——它证明了系统**可以被镇定**（但还没给出具体的控制律）。

**定义（Control Lyapunov Function）**：对控制仿射系统 $\dot x = f(x)+g(x)u$，$C^1$ 函数 $V:\mathbb{R}^n\to\mathbb{R}_{\ge 0}$ 是 CLF，如果它满足专题 3.7 中 Lyapunov 函数的所有基本条件——(i) $V(0)=0$；(ii) $V(x)>0$ 对 $x\neq 0$（正定）；(iii) $V(x)\to\infty$ 当 $\|x\|\to\infty$（径向无界，即存在 $\alpha_1,\alpha_2\in\mathcal{K}_\infty$ 使 $\alpha_1(\|x\|)\le V(x)\le\alpha_2(\|x\|)$）——并且额外满足**可镇定条件**：
$$\inf_{u\in\mathbb{R}^m}\left[L_f V(x) + L_g V(x)\,u\right] < 0 \quad \forall x\neq 0$$

其中 $L_f V = \nabla V\cdot f$ 是 $V$ 沿 $f$ 的 Lie 导数，$L_g V = \nabla V\cdot g$ 是沿 $g$ 的 Lie 导数（$1\times m$ 行向量）。

**等价条件**：由于 $L_f V + L_g V\,u$ 关于 $u$ 是仿射的（线性 + 常数），上述 $\inf$ 有限当且仅当 $L_g V(x)=0$ 蕴含 $L_f V(x)<0$。这就是经典的 **small control property**：

$$L_g V(x) = 0 \implies L_f V(x) < 0$$

**直觉**：如果 $L_g V\neq 0$，我们总可以选 $u$ 沿 $-(L_g V)^\top$ 方向，使 $L_g V\cdot u$ 足够负。唯一需要担心的是 $L_g V=0$ 的情况——此时控制对 $\dot V$ 无影响，只能靠 $f$ 本身让 $V$ 下降。small control property 正是保证了这一点。

#### Artstein 定理：CLF 存在等价于可镇定

**定理（Artstein 1983, Nonlinear Analysis: Theory, Methods & Applications 7(11):1163-1173）**：对控制仿射系统 $\dot x=f(x)+g(x)u$，以下等价：

(i) 系统是（全局）渐近可镇定的（存在连续反馈 $u=k(x)$ 使原点全局渐近稳定）

(ii) 存在光滑的 CLF $V$

**这个定理的重要性**：它把"能否设计稳定控制器"这个**综合**问题，等价转化为"能否找到一个满足特定条件的标量函数"这个**分析**问题。一旦找到 CLF，Sontag 公式（下一节）直接给出控制律。

**定理的证明思路**（不完整证明，给出核心思想）：

**(ii)→(i)**：这个方向相对容易。给定 CLF $V$，Sontag（1989）显式构造了一个连续反馈律 $u=k(x)$（即 Sontag 公式），使得 $\dot V \le -\alpha(V)$。

**(i)→(ii)**：这个方向深刻得多。关键想法是利用逆 Lyapunov 定理（Massera 1956）：若闭环系统 $\dot x = f(x)+g(x)k(x)$ 全局渐近稳定，则存在光滑 Lyapunov 函数 $V$。然后验证此 $V$ 满足 CLF 条件——在 $k(x)$ 处 $L_fV + L_gV\cdot k(x)<0$ 对 $x\neq 0$ 成立，而取 $\inf$ 只会更小。

> **反事实推理**：如果 Artstein 定理不成立——即存在可镇定但找不到 CLF 的系统——那么 CLF-QP 方法的适用范围将大大缩小。我们将不得不为每个系统单独设计控制律，无法使用统一的 QP 框架。Artstein 定理保证了：只要系统可镇定，CLF 方法就一定适用（虽然找 CLF 本身可能很难）。

### 2.2 Sontag 公式：从 CLF 到显式反馈 ⭐⭐

#### 动机：CLF 告诉你"能镇定"，但怎么镇定？

CLF 的定义只说"存在 $u$ 使 $\dot V<0$"，没说 $u$ 具体取什么值。最简单的想法是取 $u = -L_g V^\top$（沿梯度下降方向）——但这通常过于激进（控制量很大），而且在 $L_g V=0$ 处不连续。

Sontag（1989, Systems & Control Letters 13:117-123）给出了一个优雅的显式公式，它是**连续的**、**满足 small control property** 的最小能量解。

#### Sontag 公式的推导

设 $a(x) = L_f V(x)$，$b(x) = L_g V(x)\in\mathbb{R}^{1\times m}$。CLF 条件要求：对 $x\neq 0$，$b(x)=0 \implies a(x)<0$。

**目标**：找连续 $u=k(x)$ 使 $a(x)+b(x)u<0$ 对 $x\neq 0$。

**Sontag 公式（单输入 $m=1$）**：

$$k(x) = \begin{cases} \displaystyle\frac{-a(x) - \sqrt{a(x)^2 + b(x)^4}}{b(x)} & \text{if } b(x)\neq 0 \\[8pt] 0 & \text{if } b(x) = 0 \end{cases}$$

**推导过程**：

**Step 1**：我们要求 $\dot V = a + bu < 0$，即 $bu < -a$。当 $b\neq 0$ 时，最"温和"的满足方式是让 $a+bu$ 恰好等于某个负值。Sontag 的选择是让：
$$a + bu = -\sqrt{a^2 + b^4}$$

为什么选这个特定的形式？因为：
- 右边恒为负（保证 $\dot V<0$）
- 当 $b\to 0$ 且 $a<0$ 时，$u\to 0$（连续性）
- $|u|$ 在 $a<0$ 时较小（小控制性质）

**Step 2**：由 $a+bu = -\sqrt{a^2+b^4}$ 解出：
$$u = \frac{-a - \sqrt{a^2+b^4}}{b}$$

**Step 3（验证连续性）**：当 $b\to 0$ 时，分子分母都趋于零。用 L'Hopital 或直接有理化：

$$u = \frac{(-a-\sqrt{a^2+b^4})(-a+\sqrt{a^2+b^4})}{b(-a+\sqrt{a^2+b^4})} = \frac{a^2-(a^2+b^4)}{b(-a+\sqrt{a^2+b^4})} = \frac{-b^4}{b(-a+\sqrt{a^2+b^4})} = \frac{-b^3}{-a+\sqrt{a^2+b^4}}$$

当 $b\to 0$（此时由 small control property $a<0$）：分子 $\to 0$，分母 $\to -a+|a| = -a+(-a)=-2a>0$。因此 $u\to 0$。连续！

**Sontag 公式（多输入 $m>1$）**：

$$k(x) = \begin{cases} \displaystyle\frac{-a(x) - \sqrt{a(x)^2 + \|b(x)\|^4}}{\|b(x)\|^2}\,b(x)^\top & \text{if } b(x)\neq 0 \\[8pt] 0 & \text{if } b(x) = 0 \end{cases}$$

方向沿 $b^\top$（$L_g V$ 的转置），是使 $\dot V$ 下降最快的方向在仿射约束下的投影。

**验证 $\dot V<0$**：
$$\dot V = a + b\cdot k = a + \frac{-a-\sqrt{a^2+\|b\|^4}}{\|b\|^2}\cdot\|b\|^2 = a + (-a-\sqrt{a^2+\|b\|^4}) = -\sqrt{a^2+\|b\|^4} < 0$$

当 $x\neq 0$（若 $b=0$ 则 $a<0$，取 $u=0$ 即 $\dot V=a<0$）。

> **本质洞察**：Sontag 公式的深层含义是——CLF 不仅证明了可镇定性的"存在性"，还直接提供了一个**显式的、连续的、最小控制能量**的反馈律。这是从"存在性"到"构造性"的关键跨越。在 CBF 领域，类似的构造性结果就是 CBF-QP 的闭式解（Safety Filter 的显式解）。

#### Sontag 公式的深层意义

Sontag 公式有几个深刻的数学性质值得理解：

**1. 最优性**：在某种意义下，Sontag 公式是"最节能"的稳定反馈。具体地，它是使 $\dot V=-\sqrt{a^2+\|b\|^4}$ 成立的最小范数 $u$（在所有使 $\dot V$ 达到该值的 $u$ 中范数最小的那个）。

**2. 逆最优性**（inverse optimality）：Freeman-Kokotovic（1996）证明 Sontag 公式可以视为某个（隐式定义的）最优控制问题的解——即存在代价函数 $J=\int_0^\infty[\ell(x)+u^\top R(x)u]dt$ 使得 Sontag 公式恰好是其最优反馈。这赋予了它鲁棒性保证（类似 LQR 的增益裕度）。

**3. 与 CBF Safety Filter 的对偶**：

| | CLF（Sontag） | CBF（Safety Filter） |
|---|---|---|
| 目标 | 使 $\dot V$ 足够负 | 使 $\dot h$ 不太负 |
| 解的形式 | $u=\lambda\cdot(L_gV)^\top$ | $u=u_{\text{nom}}+\lambda\cdot(L_gh)^\top$ |
| $\lambda$ 的含义 | 保证收敛的"努力程度" | 保证安全的"修正力度" |
| 激活条件 | $L_gV\neq 0$（控制可影响V） | $a+bu_{\text{nom}}<0$（名义不安全） |

> **本质洞察**：Sontag 公式和 CBF Safety Filter 闭式解在结构上是同构的——都是沿某个梯度方向的投影/修正。前者是"把 $u$ 从 0 拉到使 $\dot V<0$ 的最小距离"，后者是"把 $u_{\text{nom}}$ 投影到使 $\dot h\ge-\alpha(h)$ 的最近点"。理解了这个对称性，就理解了整个 CLF-CBF 框架的数学本质。

#### CLF-QP：从 Sontag 到优化

Sontag 公式虽然优雅，但有局限：（1）不容易加入输入约束 $u\in\mathcal{U}$；（2）不容易与 CBF 结合；（3）衰减率 $\sqrt{a^2+\|b\|^4}$ 不可调。现代方法用 QP 替代 Sontag 公式：

**CLF-QP**：
$$u^* = \arg\min_{u\in\mathcal{U}} \|u\|^2 \quad \text{s.t.} \quad L_f V(x) + L_g V(x)\,u \le -\gamma V(x)$$

这是带输入约束的 CLF 实现。选 $\gamma V$ 作为衰减率给出指数收敛 $V(t)\le V(0)e^{-\gamma t}$。

**CLF-QP 与 Sontag 公式的关系**：当 $\mathcal{U}=\mathbb{R}^m$ 且目标是最小范数 $u$ 时，CLF-QP 的解与 Sontag 公式不完全相同（因为 Sontag 选择了特定的衰减率），但都保证稳定性。CLF-QP 的优势是衰减率 $\gamma$ 可调且可加输入约束。

**CLF-QP 的可行性条件**：约束 $L_fV+L_gVu\le-\gamma V$ 与 $u\in\mathcal{U}$ 相容，当且仅当：
$$\exists u\in\mathcal{U}: L_gV\cdot u\le -L_fV-\gamma V$$

当 $L_gV=0$ 时需要 $L_fV+\gamma V\le 0$（由 small control property 保证在 $L_gV=0\implies L_fV<0$ 的邻域成立，但 $\gamma$ 太大可能使 $-\gamma V$ 项主导而不可行）。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：Sontag 公式在 $b\approx 0$ 时的数值问题**
>
> **错误做法**：直接用 $u = (-a-\sqrt{a^2+b^4})/b$ 计算，当 $|b|<10^{-8}$ 时分母接近零。
>
> **现象**：控制输出出现 $10^{15}$ 量级的脉冲，执行器饱和，系统失稳。
>
> **正确做法**：使用有理化后的等价形式 $u = -b^3/(-a+\sqrt{a^2+b^4})$，或设阈值 $|b|<\epsilon$ 时令 $u=0$。

> 💡 **概念误区：CLF = Lyapunov 函数**
>
> **实际区别**：Lyapunov 函数是对**给定闭环系统**证明稳定性的工具；CLF 是对**开环系统**证明可镇定性的工具。前者的 $\dot V<0$ 对固定的 $f$ 检验；后者的 $\inf_u[L_fV+L_gVu]<0$ 是对所有可能 $u$ 检验。

### 练习 2.1

1. 对一维系统 $\dot x = x^2 + u$，验证 $V(x)=x^2/2$ 是 CLF，并计算 Sontag 公式给出的 $k(x)$。
2. 推导多输入情况下 Sontag 公式中 $\|b\|^4$ 项的来源（提示：考虑 $u=\lambda b^\top$ 的参数化）。
3. 对比 CLF-QP 与 Sontag 公式的控制量大小：在什么情况下 QP 的解与 Sontag 公式一致？

---

## 3. Control Barrier Function (CBF) ⭐⭐

### 3.0 比较引理详解——CBF 的数学引擎 ⭐⭐

#### 为什么需要比较引理

比较引理（Comparison Lemma）是 CBF 理论的**核心数学工具**。CBF 条件 $\dot h\ge -\alpha(h)$ 是一个微分不等式，比较引理将它转化为 $h(t)$ 的显式下界——从而证明 $h$ 不穿零。

回顾专题 3.7（§7.16 比较定理与估计技术）：Lyapunov 理论中比较引理用于从 $\dot V\le -\gamma V$ 推出 $V(t)\le V(0)e^{-\gamma t}$（上界），这正是指数稳定衰减率估计的核心工具。CBF 理论中**同一引理的对偶版本**用于从 $\dot h\ge -\alpha(h)$ 推出 $h(t)\ge z(t)$（下界）。上界保证"Lyapunov 函数衰减到零"（稳定性），下界保证"安全裕度不穿过零"（安全性）——两者是比较引理这枚硬币的正反面。

#### 比较引理的一般形式

**引理（Khalil, Nonlinear Systems, Lemma 3.4 / 4.4）**：

设 $\dot y(t)\ge \phi(t, y(t))$，$y(t_0)=y_0$，其中 $\phi$ 关于 $y$ 局部 Lipschitz、关于 $t$ 连续。设 $z(t)$ 是 ODE $\dot z=\phi(t,z)$，$z(t_0)=y_0$ 的解。则：
$$y(t)\ge z(t) \quad \forall t\ge t_0$$

（在两者解均存在的区间上）。

**对偶版本**：若 $\dot y\le \phi(t,y)$，则 $y(t)\le z(t)$。

**CBF 中的特化**：取 $\phi(t,y)=-\alpha(y)$（自治、与时间无关）。$\dot y\ge -\alpha(y)$ $\implies$ $y(t)\ge z(t)$，其中 $\dot z=-\alpha(z)$。

#### 为什么比较 ODE $\dot z=-\alpha(z)$ 的解不穿零

这是理解 CBF 安全性保证的关键步骤。

设 $z(0)=z_0>0$，$\alpha$ 是扩展类 $\mathcal{K}_\infty$（连续、严格递增、$\alpha(0)=0$���Lipschitz）。

**命题**：$z(t)>0$ 对所有有限 $t$。

**证明**：$z=0$ 是 $\dot z=-\alpha(z)$ 的平衡点（$-\alpha(0)=0$）。由 $\alpha$ 的 Lipschitz 性，ODE 解唯一。经过 $(t^*,0)$ 的唯一解是 $z(t)\equiv 0$。从 $z_0>0$ 出发的解不能在有限时间到达 $z=0$（否则在到达时刻与常值解重合但之前不同——违反唯一性）。$\square$

**注意**：若 $\alpha$ 在 0 处不 Lipschitz（如 $\alpha(r)=\gamma\sqrt{r}$），唯一性失效，$z(t)$ 可以在有限时间到达零。这对应 CBF 的 **finite-time convergence** 变体——在某些应用中有用但需额外分析。

#### 线性情形的显式解

当 $\alpha(r)=\gamma r$：$\dot z=-\gamma z$，$z(0)=h_0$。

显式解：$z(t)=h_0 e^{-\gamma t}$

CBF 保证：$h(t)\ge h_0 e^{-\gamma t}$（指数下界）

这是最常用的选择，因为：
1. 显式解简单，便于分析
2. 对应的安全裕度以指数速率衰减但永不到零
3. 时间常数 $1/\gamma$ 有明确物理意义（"从当前裕度到边界附近需要多久"）

### 3.1 Zeroing CBF 严格定义 ⭐⭐

#### 动机：如何把"安全"写成数学约束

控制仿射系统 $\dot x = f(x)+g(x)u$，$x\in\mathbb{R}^n$，$u\in\mathcal{U}\subseteq\mathbb{R}^m$，$f,g$ 局部 Lipschitz。安全集 $\mathcal{C}=\{x:h(x)\ge 0\}$ 由连续可微 $h$ 定义，满足正则性 $\nabla h\neq 0$ 在 $\partial\mathcal{C}$ 上。

**我们需要的是**：一个关于 $u$ 的**线性约束**，使得只要 $u$ 满足该约束，系统就保持安全。CBF 正是将"前向不变性"编码为这样一个线性约束的工具。

#### 扩展类 $\mathcal{K}_\infty$ 函数

**定义**：$\alpha:\mathbb{R}\to\mathbb{R}$ 是扩展类 $\mathcal{K}_\infty$ 函数，如果：
- $\alpha$ 连续
- $\alpha$ 严格递增
- $\alpha(0)=0$
- $\alpha(r)\to +\infty$ 当 $r\to +\infty$
- $\alpha(r)\to -\infty$ 当 $r\to -\infty$

**关键区别**：普通类 $\mathcal{K}$ 函数只定义在 $[0,\infty)$ 上，扩展类 $\mathcal{K}_\infty$ 定义在整个 $\mathbb{R}$ 上（包括负半轴）。

**为什么必须扩展到负半轴？** CBF 条件 $\dot h\ge -\alpha(h)$ 必须在 $h<0$ 时也有意义。当 $h<0$（系统暂时不安全）：$\alpha(h)<0$（因 $\alpha$ 递增且 $\alpha(0)=0$），所以 $-\alpha(h)>0$，条件变为 $\dot h>0$——强制 $h$ 增大，系统被"拉回"安全集。这正是 ZCBF 的 "zeroing" 行为。

**常用选择**：
- 线性：$\alpha(r) = \gamma r$，$\gamma>0$（最常用，给出指数收敛/衰减）
- 多项式：$\alpha(r) = \gamma r^3$（在零附近更温和）
- 高阶：$\alpha(r) = \gamma r^{2k-1}$，$k\ge 1$

#### ZCBF 正式定义

**定义（Zeroing Control Barrier Function, Ames-Xu-Grizzle-Tabuada 2017）**：

函数 $h:\mathcal{D}\to\mathbb{R}$（$\mathcal{D}\supseteq\mathcal{C}$ 为开集）是系统的 ZCBF，如果存在扩展类 $\mathcal{K}_\infty$ 函数 $\alpha$ 使得：

$$\sup_{u\in\mathcal{U}}\left[L_f h(x) + L_g h(x)\,u\right] \ge -\alpha(h(x)) \quad \forall x\in\mathcal{D}$$

其中 $L_f h = \nabla h\cdot f$，$L_g h = \nabla h\cdot g$（$1\times m$ 行向量）。

**解读**：对每个状态 $x$，至少存在一个控制 $u$ 使得 $\dot h\ge -\alpha(h)$。这个条件的"sup"取在 $u$ 上——我们不要求**所有** $u$ 满足，只要**存在** $u$ 满足即可。

**可行控制集**：
$$K_{\text{cbf}}(x) = \{u\in\mathcal{U} : L_f h(x) + L_g h(x)\,u \ge -\alpha(h(x))\}$$

这是 $\mathcal{U}$ 中满足 CBF 约束的控制子集。ZCBF 定义等价于 $K_{\text{cbf}}(x)\neq\varnothing$ 对所有 $x\in\mathcal{D}$。

### 3.2 CBF 前向不变性主定理与完整证明 ⭐⭐

**主定理（Ames 2017, Thm 2）**：若 $h$ 为 ZCBF，$\nabla h\neq 0$ 在 $\partial\mathcal{C}$ 上，则任何 Lipschitz 连续控制律 $u(x)\in K_{\text{cbf}}(x)$ 使得：

(a) $\mathcal{C}$ **前向不变**（$x(0)\in\mathcal{C}\implies x(t)\in\mathcal{C}$，$\forall t\ge 0$）

(b) $\mathcal{C}$ 在 $\mathcal{D}$ 内**渐近稳定**（$x(0)\notin\mathcal{C}$ 的轨迹最终回到 $\mathcal{C}$）

**完整证明**：

**Part (a)：前向不变性**

**Step 1 — 建立微分不等式**：设 $u(x)\in K_{\text{cbf}}(x)$ Lipschitz 连续（保证闭环 ODE 解存在唯一）。沿闭环轨迹：
$$\dot h(x(t)) = L_f h(x(t)) + L_g h(x(t))\,u(x(t)) \ge -\alpha(h(x(t)))$$

令 $y(t) = h(x(t))$，则 $\dot y(t) \ge -\alpha(y(t))$。

**Step 2 — 调用比较引理**：

回顾专题 3.7 的比较引理（Khalil, Nonlinear Systems, Lemma 4.4）：

设 $\dot y\ge -\alpha(y)$，$y(0)=y_0$。令 $z(t)$ 为标量 ODE $\dot z = -\alpha(z)$，$z(0)=y_0$ 的解。若 $\alpha$ 局部 Lipschitz，则 $y(t)\ge z(t)$ 对所有 $t\ge 0$（在解存在的区间上）。

**证明比较引理**：令 $w(t)=y(t)-z(t)$。则 $w(0)=0$ 且
$$\dot w = \dot y - \dot z \ge -\alpha(y) - (-\alpha(z)) = \alpha(z)-\alpha(y)$$

这不能直接得出 $w\ge 0$。正确方法：假设存在首次 $y<z$ 的时刻 $t^*$（即 $w(t^*)=0$，$\dot w(t^*)<0$）。由 $w(t^*)=0$ 得 $y(t^*)=z(t^*)$，所以 $\dot w(t^*)=\dot y(t^*)-\dot z(t^*)\ge -\alpha(y(t^*))+\alpha(z(t^*))=0$，矛盾。$\square$

**Step 3 — 分析比较 ODE**：$\dot z = -\alpha(z)$，初值 $z(0)=h(x(0))\ge 0$（因为 $x(0)\in\mathcal{C}$）。

**情形 A**：$z(0)>0$。

由 $\alpha$ 是扩展类 $\mathcal{K}_\infty$：当 $z>0$ 时 $\alpha(z)>0$，所以 $\dot z=-\alpha(z)<0$——$z$ 严格递减。

但 $z$ 能到达 0 吗？$z=0$ 是平衡点：$\dot z|_{z=0}=-\alpha(0)=0$。由唯一性定理（$\alpha$ 局部 Lipschitz），经过 $(0,0)$ 的解只有 $z(t)\equiv 0$。从 $z(0)>0$ 出发的解不可能在有限时间到达 $z=0$（否则与常值解 $z\equiv 0$ 在到达时刻有相同值，但之前不同——违反唯一性，除非 $\alpha$ 在 0 处不满足 Lipschitz，如 $\alpha(r)=\sqrt{r}$ 可导致有限时间收敛）。

对标准选择 $\alpha(r)=\gamma r$：$\dot z=-\gamma z$，解为 $z(t)=z(0)e^{-\gamma t}>0$ 对所有 $t<\infty$。

**情形 B**：$z(0)=0$。由唯一性，$z(t)\equiv 0$。

两种情形下 $z(t)\ge 0$ 对所有 $t\ge 0$。由比较不等式：
$$h(x(t)) = y(t) \ge z(t) \ge 0 \quad \forall t\ge 0$$

即 $x(t)\in\mathcal{C}$ 保持。$\blacksquare$

**Part (b)：渐近稳定性（zeroing 性质）**

若 $x(0)\notin\mathcal{C}$，即 $h(x(0))<0$。比较 ODE：$\dot z=-\alpha(z)$，$z(0)<0$。

由 $\alpha(z)<0$ 当 $z<0$（$\alpha$ 严格递增，$\alpha(0)=0$），$\dot z=-\alpha(z)>0$。即 $z$ 递增。

$z=0$ 仍是平衡点。$z$ 递增趋向 0（但以标准 Lipschitz $\alpha$ 不能有限时间到达）。

因此 $y(t)=h(x(t))\ge z(t)\to 0^-$。即 $h(x(t))$ 趋向非负——轨迹被"拉回"安全集边界。$\blacksquare$

> **本质洞察**：CBF 条件 $\dot h\ge -\alpha(h)$ 的几何含义可以这样理解：想象 $h$ 值像一个"安全裕度仪表"，CBF 条件说的是"仪表下降的速度不能超过比较 ODE 给出的衰减速度"。而比较 ODE 从正初值出发永远为正——所以仪表永远为正，系统永远安全。这与 CLF 条件 $\dot V\le -\gamma(V)$ 是精确对偶：CLF 说"Lyapunov 仪表至少以 $\gamma(V)$ 的速度下降"（保证收敛），CBF 说"安全仪表至多以 $\alpha(h)$ 的速度下降"（保证不穿零）。

### 3.3 $\alpha$ 函数的选择与工程影响 ⭐⭐

$\alpha$ 的选择直接影响控制器的行为特性：

| $\alpha(h)$ | CBF 约束 | 比较 ODE 解 | 行为特征 |
|---|---|---|---|
| $\gamma h$（线性） | $\dot h\ge -\gamma h$ | $h(t)\ge h(0)e^{-\gamma t}$ | 指数衰减，最常用 |
| $\gamma h^3$（立方） | $\dot h\ge -\gamma h^3$ | 更慢衰减 | 远离边界时更宽松 |
| $\gamma\sqrt{h}$（根号，仅 $h\ge 0$） | $\dot h\ge -\gamma\sqrt{h}$ | 有限时间到零 | 更保守，但 $h=0$ 处非 Lipschitz |

**$\gamma$ 的工程意义**：

- **$\gamma$ 大**（如 $\gamma=10$）：允许 $h$ 快速衰减，系统可以高速接近边界，CBF 约束不太"介入"日常控制。**优点**：对名义控制器侵入小。**缺点**：接近边界时需要急刹车，若输入有限可能来不及。
- **$\gamma$ 小**（如 $\gamma=0.5$）：限制 $h$ 缓慢衰减，系统从远处就被"减速"。**优点**：安全余量大。**缺点**：过度保守，名义性能被显著牺牲。

**工程推荐**：从 $\alpha(r)=\gamma r$，$\gamma\in[1,10]$ 开始调参。实际应用中可用自适应 $\gamma$：远离边界时 $\gamma$ 大（宽松），接近时 $\gamma$ 小（保守）。

#### $\alpha$ 选择的详细分析

为深入理解 $\alpha$ 的影响，我们对线性选择 $\alpha(h)=\gamma h$ 做定量分析。

**比较 ODE 的解**：$\dot z=-\gamma z$，$z(0)=h_0>0$。解为 $z(t)=h_0 e^{-\gamma t}$。

CBF 条件 $\dot h\ge -\gamma h$ 保证 $h(t)\ge h_0 e^{-\gamma t}$。这意味着：

- 在 $t=0$ 时，$h$ 从 $h_0$ 开始
- 在 $t=1/\gamma$ 时，$h$ 的下界为 $h_0/e\approx 0.37 h_0$（安全裕度下降到 37%）
- 在 $t=3/\gamma$ 时，$h$ 的下界为 $h_0 e^{-3}\approx 0.05 h_0$（安全裕度下降到 5%）

**时间常数 $\tau=1/\gamma$** 决定了系统从安全状态到达边界附近的"最坏情况"时间。如果系统有制动延迟 $\tau_d$，应选 $\gamma$ 使 $\tau=1/\gamma\gg\tau_d$——否则系统在安全裕度耗尽之前来不及响应。

**例子**：四足机器人的 WBC 频率 500 Hz（$\tau_d\approx 2$ ms），CBF 控制频率 1 kHz。设安全距离裕度 $h_0=0.1$ m，取 $\gamma=5$（时间常数 0.2s），则最快 0.2s 安全裕度降到 37%——这给控制系统足够的响应时间。

**非线性 $\alpha$ 的高级用法**：

对某些场景，线性 $\alpha$ 可能不够好。例如当系统接近边界时需要更激进的保护：

$$\alpha(h) = \gamma_1 h + \gamma_2 h^3$$

当 $h$ 大（远离边界），立方项主导，允许较快接近；当 $h$ 小（接近边界），线性项主导，提供温和约束。反之亦然：

$$\alpha(h) = \gamma\tanh(h/\epsilon)$$

将约束"饱和"在 $[-\gamma,\gamma]$，避免当 $h$ 很大时 $\alpha(h)$ 过大导致约束形同虚设。

### 3.4 Reciprocal CBF 对比 ⭐⭐⭐

**Reciprocal CBF（RCBF）** 是早期的障碍函数形式：

$$B:\operatorname{Int}(\mathcal{C})\to\mathbb{R}_{\ge 0}, \quad B(x)\to +\infty \text{ 当 } x\to\partial\mathcal{C}$$

条件：$\inf_u[L_f B + L_g B\,u] \le \alpha(1/B(x))$

**典型构造**：$B(x) = 1/h(x)$（当 $h>0$）。

**ZCBF vs RCBF 对比**：

| 特性 | Zeroing CBF | Reciprocal CBF |
|---|---|---|
| 值域 | $h:\mathbb{R}^n\to\mathbb{R}$（有限） | $B:\operatorname{Int}(\mathcal{C})\to[0,\infty)$（边界处爆破） |
| 边界行为 | $h\to 0$（光滑接近） | $B\to\infty$（发散） |
| 梯度 | 在边界有界（$\nabla h\neq 0$ 但有限） | 在边界发散（$\nabla B\to\infty$） |
| 数值性质 | 友好（QP 系数有界） | 不友好（梯度爆炸，QP ill-conditioned） |
| 执行器饱和 | 可控（约束线性） | 易饱和（大梯度→大控制） |
| 不安全域处理 | 自然定义（$h<0$ 有意义） | 无定义（$B$ 只在 Int 定义） |
| 现代使用 | **事实标准** | 已基本弃用 |

> **反事实推理**：如果我们用 RCBF 而不是 ZCBF 会怎样？当系统接近安全边界时，$B=1/h\to\infty$，梯度 $\nabla B = -\nabla h/h^2\to\infty$。QP 的约束系数 $L_gB$ 爆炸，导致：(1) 数值求解不稳定；(2) 解出的 $u$ 可能很大但方向不合理；(3) 执行器饱和后安全性丧失。ZCBF 的梯度 $\nabla h$ 在边界保持有界（正则性条件），QP 始终 well-conditioned。

### 3.5 高相对度 CBF（HOCBF） ⭐⭐⭐

#### 动机：为什么标准 CBF 会失效

考虑双积分器 $\ddot q = u$（等价于 $\dot x_1=x_2$，$\dot x_2=u$，其中 $x_1=q$，$x_2=\dot q$）。位置约束 $h(x)=x_1-q_{\min}$。

计算 Lie 导数：
$$L_f h = \nabla h\cdot f = [1,0]\cdot[x_2, 0]^\top = x_2$$
$$L_g h = \nabla h\cdot g = [1,0]\cdot[0, 1]^\top = 0$$

**灾难**：$L_g h\equiv 0$！控制 $u$ 根本不出现在 $\dot h$ 中。CBF 约束变为 $x_2\ge -\alpha(h)$——这是对状态的约束，不是对控制的约束，无法写进 QP！

**物理直觉**：位置约束 $h=q-q_{\min}$ 的"安全性"取决于位置 $q$，而控制 $u$ 直接影响的是加速度 $\ddot q$。从加速度到位置需要两次积分——这就是"相对度 2"的含义。控制对约束的影响被"延迟"了两步。

#### 相对度的定义

**定义**：安全约束 $h(x)$ 相对于系统 $\dot x=f(x)+g(x)u$ 的**相对度** $r$ 是满足以下条件的最小正整数：
$$L_g L_f^{r-1} h(x) \neq 0$$

即 $u$ 首次出现在 $h$ 的第 $r$ 阶时间导数中：
$$h^{(r)} = L_f^r h + L_g L_f^{r-1} h\cdot u$$

对相对度 $r>1$ 的约束，$L_g h = L_g L_f h = \cdots = L_g L_f^{r-2}h = 0$，标准 CBF 失效。

#### HOCBF 递归构造（Xiao-Belta 2022）

**核心思想**：不直接约束 $h$，而是构造一条"约束链" $\psi_0,\psi_1,\ldots,\psi_{r-1}$，每层约束上一层的导数，最终在第 $r$ 层引入控制 $u$。

**定义（Xiao-Belta, IEEE TAC 67(7):3655-3662, 2022）**：

$$\psi_0(x) := h(x)$$
$$\psi_1(x) := \dot\psi_0(x) + \alpha_1(\psi_0(x)) = L_f h(x) + \alpha_1(h(x))$$
$$\psi_2(x) := \dot\psi_1(x) + \alpha_2(\psi_1(x))$$
$$\vdots$$
$$\psi_i(x) := \dot\psi_{i-1}(x) + \alpha_i(\psi_{i-1}(x)), \quad i=1,\ldots,r-1$$

其中 $\alpha_i$ 为扩展类 $\mathcal{K}_\infty$ 函数（可任意非线性）。

**HOCBF 约束**：最终只在第 $r$ 层施加约束，此时 $u$ 出现：
$$\dot\psi_{r-1}(x,u) + \alpha_r(\psi_{r-1}(x)) \ge 0$$

展开后：
$$L_f^r h + L_g L_f^{r-1} h\cdot u + O(\alpha_1,\ldots,\alpha_r, h, L_fh,\ldots) \ge 0$$

这是关于 $u$ 的**线性约束**，可以写进 QP！

**安全性保证**：若控制器满足 HOCBF 约束，则所有 $\mathcal{C}_i=\{\psi_{i-1}\ge 0\}$ 前向不变。由于 $\psi_0=h$，$\mathcal{C}_0=\{h\ge 0\}$ 前向不变——原始安全集得到保证。

**直觉解释（相对度 2 的例子）**：

对双积分器 $\ddot q=u$，$h=q-q_{\min}$：
- $\psi_0 = h = q-q_{\min}$（位置裕度）
- $\psi_1 = \dot h + \alpha_1(h) = \dot q + \alpha_1(q-q_{\min})$（"安全速度"：当前速度加上基于位置裕度的补偿）
- HOCBF 约束：$\dot\psi_1 + \alpha_2(\psi_1)\ge 0$，即 $u + \dot\alpha_1(h)\cdot\dot h + \alpha_2(\psi_1)\ge 0$

**层层约束的直觉**：
- 第 0 层：位置不能超过下限
- 第 1 层：即使位置安全，速度也不能太快（朝着边界冲的速度被约束）
- 第 2 层（约束本身）：加速度（控制）被约束以保证速度不超标

> **跨领域类比**：HOCBF 像一个嵌套的"安全缓冲区"系统。想象一辆正在驶向悬崖的车：
> - 第 0 层（$h\ge 0$）：车不能掉下悬崖（位置约束）
> - 第 1 层（$\psi_1\ge 0$）：即使车还没到悬崖边，如果速度太快也不行（速度约束——取决于到悬崖的距离）
> - 第 2 层（HOCBF 约束）：制动力（控制）必须足够大以保证能减速（加速度约束）
>
> 越远离悬崖，允许的速度越大；越接近悬崖，要求的制动力越大。

#### HOCBF 完整数值例子（双积分器位置限制）

**系统**：$\dot x_1=x_2$，$\dot x_2=u$。约束 $h=x_1-q_{\min}$，$q_{\min}=-1$。

**Step 1**：$\psi_0=x_1+1$（位置裕度）

**Step 2**：$\psi_1=\dot\psi_0+\alpha_1(\psi_0)=x_2+\gamma_1(x_1+1)$

取 $\gamma_1=2$：$\psi_1=x_2+2(x_1+1)=x_2+2x_1+2$

**直觉**：$\psi_1\ge 0$ 意味着"当前速度加上基于位置裕度的补偿项非负"。即使 $x_2<0$（向下移动），只要位置裕度 $x_1+1$ 足够大（$\ge |x_2|/2$），仍算"安全速度"。

**Step 3**：HOCBF 约束 $\dot\psi_1+\alpha_2(\psi_1)\ge 0$

$\dot\psi_1 = \dot x_2+2\dot x_1 = u+2x_2$

取 $\gamma_2=2$：约束为 $u+2x_2+2(x_2+2x_1+2)\ge 0$

化简：$u+4x_2+4x_1+4\ge 0$，即 $u\ge -4x_2-4x_1-4 = -4(x_1+x_2+1)$

**数值验证**：

| 状态 $(x_1, x_2)$ | 物理含义 | $u$ 下界 | 解读 |
|---|---|---|---|
| $(0, -1)$ | 位置在 0，向下 | $0$ | 必须开始减速 |
| $(-0.5, -2)$ | 接近下限，快速下降 | $6$ | 紧急制动 |
| $(5, 0)$ | 远离边界，静止 | $-24$ | 几乎不约束 |
| $(-0.9, -0.1)$ | 非常接近边界，慢速 | $0$ | 位置接近边界但速度小，恰处临界 |

**约束集合的几何形状**：

$\mathcal{C}_0=\{x:x_1\ge -1\}$ 是 $x_1$ 轴右侧的半平面。

$\mathcal{C}_1=\{x:x_2+2x_1+2\ge 0\}$ 即 $\{x:x_2\ge -2x_1-2\}$ 是另一个半平面。

安全可行域 $\mathcal{C}_0\cap\mathcal{C}_1$ 在相平面上是一个楔形区域。HOCBF 保证：一旦系统在交集中，就永远在其中（前提是 HOCBF 约束被满足）。

```python
# HOCBF 双积分器 Safety Filter
import numpy as np

def hocbf_double_integrator(x1, x2, u_nom, gamma1=2.0, gamma2=2.0,
                            q_min=-1.0, u_bounds=(-10.0, 10.0)):
    """双积分器 HOCBF Safety Filter"""
    psi0 = x1 - q_min
    psi1 = x2 + gamma1 * psi0
    # HOCBF约束: u + gamma1*x2 + gamma2*psi1 >= 0
    u_cbf_min = -(gamma1 * x2 + gamma2 * psi1)
    u_safe = max(u_nom, u_cbf_min)
    return np.clip(u_safe, *u_bounds), psi0, psi1
```

**与极点配置的联系**：$\gamma_1=\gamma_2=2$ 对应特征方程 $(s+2)^2=0$，双重极点 $s=-2$。若系统运动在 HOCBF 约束上（约束激活），其行为如同二阶系统被极点 $-2$ 约束。

### 3.6 指数 CBF（ECBF） ⭐⭐⭐

#### 动机：HOCBF 的线性特例

**指数 CBF** 是 Nguyen-Sreenath（ACC 2016）提出的方法，是 HOCBF 当所有 $\alpha_i$ 取线性时的特例。它的优势是可以用极点配置设计参数。

**构造**：对相对度 $r$ 的约束 $h$，定义状态向量：
$$\eta_b(x) = [h, L_fh, L_f^2h, \ldots, L_f^{r-1}h]^\top \in\mathbb{R}^r$$

动力学化为 Brunovsky 标准形（积分器链）：
$$\dot\eta_b = F\eta_b + G\mu$$

其中 $F$ 是 $r\times r$ 移位矩阵（companion matrix 形式），$G=[0,\ldots,0,1]^\top$，$\mu = L_f^rh + L_gL_f^{r-1}h\cdot u$。

**ECBF 约束**：
$$L_f^rh + L_gL_f^{r-1}h\cdot u \ge -K_b\,\eta_b(x)$$

其中 $K_b=[k_1,\ldots,k_r]\in\mathbb{R}^{1\times r}$ 通过**极点配置**选择，使 $F-GK_b$ 的所有特征值在左半复平面（Hurwitz）。

**极点配置与衰减速率的关系**（相对度 2）：

取 $K_b=[k_1, k_2]$，特征多项式 $s^2+k_2s+k_1=0$。若选重极点 $s=-p$：$k_1=p^2$，$k_2=2p$。

ECBF 约束变为：$u \ge -p^2 h - 2p\dot h$（当 $L_gL_fh>0$ 时除以它）。

**$p$ 的意义**：$p$ 控制系统回到安全集的速度。$p$ 大→回复快但控制量大→可能饱和；$p$ 小→回复慢但温和。

**ECBF 与 HOCBF 的关系**：当 HOCBF 中 $\alpha_i(r)=k_i\cdot r$ 全取线性时，两者等价。HOCBF 允许非线性 $\alpha_i$（如 $\sqrt{\cdot}$、$\arctan$），提供更大的设计灵活性。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：未检查相对度就直接用标准 CBF**
>
> **错误做法**：对双积分器的位置约束直接写 `Lgh * u >= -alpha * h`。
>
> **现象**：`Lgh` 计算出来恒为 0，约束变成 `0 >= -alpha*h`，当 `h>0` 时恒成立——等于没有约束！系统直接撞。
>
> **正确做法**：先计算相对度，若 $r>1$ 则用 HOCBF/ECBF。
>
> **检测方法**：计算 $L_gh$，若为零向量则相对度 $\ge 2$。

> 💡 **概念误区：HOCBF 递归中间变量 $\psi_i$ 的符号**
>
> **新手困惑**："$\psi_1=\dot h+\alpha_1(h)$，当 $\dot h<0$ 但 $|dot h|<\alpha_1(h)$ 时 $\psi_1>0$——为什么速度为负也算安全？"
>
> **正确理解**：$\psi_1\ge 0$ 不是要求 $\dot h\ge 0$（那是 Nagumo 条件），而是要求 $\dot h\ge -\alpha_1(h)$——允许 $h$ 下降（系统向边界接近），但下降速率被 $\alpha_1(h)$ 约束。这正是 CBF 相对 Nagumo 的推广。

> 🧠 **思维陷阱：认为"相对度越高越难"**
>
> **实际情况**：相对度高确实意味着需要更多层约束，但 HOCBF 的递归结构使得实现并不复杂——每层只是多一个线性/非线性函数。真正困难的是**参数选择**：$r$ 个 $\alpha_i$ 函数意味着 $r$ 倍的调参空间。工程建议：先用 ECBF（线性 $\alpha_i$），用极点配置自动确定参数，再根据需要切换到非线性。

### 练习 3.1

1. 对三积分器 $\dddot q = u$，约束 $h=q-q_{\min}$，计算相对度，写出 ECBF 约束。
2. 对二维单积分器 $\dot x=u$，$h(x)=\|x-x_{\text{obs}}\|^2-R^2$（避障），验证相对度为 1，写出标准 CBF 约束。
3. **HOCBF 参数设计**：对双积分器，分别取 $\alpha_1(r)=r$ 和 $\alpha_1(r)=\sqrt{\max(r,0)}$，分析两者在系统接近边界时的行为差异。

---

## 4. CLF-CBF-QP 统一框架 ⭐⭐

### 4.1 问题设定与冲突分析

#### 动机：稳定性与安全性可能冲突

我们希望控制器同时满足：
- **(i) 渐近稳定**到目标（由 CLF 保证）
- **(ii) 永远安全**（由 CBF 保证）

这两个目标**可能冲突**。考虑一个机器人需要到达目标，但目标路径上有障碍物：CLF 指引它直线冲向目标（穿过障碍），CBF 要求它避开障碍。

**冲突的数学表现**：当 $K_{\text{clf}}(x)\cap K_{\text{cbf}}(x)=\varnothing$ 时，不存在 $u$ 同时满足 CLF 约束 $L_fV+L_gVu\le-\gamma V$ 和 CBF 约束 $L_fh+L_ghu\ge-\alpha h$。

**解决方案**：引入**优先级**——安全是硬约束（不可违反），稳定是软约束（可松弛）。

> **本质洞察**：在机器人学中，安全和稳定性的优先级是不对称的。不安全可能导致灾难性后果（撞毁设备、伤害人类），而不稳定只是性能下降（收敛变慢或暂时不收敛）。因此 CBF 作为硬约束，CLF 通过松弛变量降级为软约束。这是"安全优先"原则的数学实现。

### 4.2 CLF-CBF-QP 完整构造 ⭐⭐

**CLF-CBF-QP**：

$$\begin{aligned}
(u^*,\delta^*) = \arg\min_{u\in\mathbb{R}^m,\,\delta\in\mathbb{R}} \quad & \frac{1}{2}\|u - u_{\text{ref}}\|_H^2 + p\,\delta^2 \\
\text{s.t.} \quad & L_f V(x) + L_g V(x)\,u \le -\gamma V(x) + \delta \quad \text{(CLF, 软)} \\
& L_f h(x) + L_g h(x)\,u \ge -\alpha(h(x)) \quad \text{(CBF, 硬)} \\
& u_{\min} \le u \le u_{\max} \quad \text{(输入约束)}
\end{aligned}$$

**各项含义**：

| 符号 | 含义 | 典型值 |
|---|---|---|
| $H\succ 0$ | 控制代价权重 | $I$（各输入等权）或对角矩阵 |
| $u_{\text{ref}}$ | 参考控制 | PD 输出、MPC 解、RL 策略 |
| $p$ | 松弛惩罚权重 | $10^2\sim 10^4$ |
| $\delta$ | CLF 松弛变量 | 由优化器决定 |
| $\gamma$ | CLF 衰减率 | $0.5\sim 5$ |
| $\alpha$ | CBF 类 $\mathcal{K}_\infty$ 函数 | $\alpha(r)=\gamma_{\text{cbf}}\cdot r$ |

**为什么 $p\delta^2$ 而不是 $p|\delta|$？** 二次惩罚使得 QP 的目标仍然是二次的（保持凸 QP 结构），且自动给出 $\delta=0$ 的软趋势。线性惩罚 $p|\delta|$ 会使问题变成二次锥规划（SOCP），求解更复杂。

### 4.3 KKT 条件与闭式解分析 ⭐⭐⭐

#### 推导 KKT 条件

为深入理解 QP 行为，简化为无输入约束的情形（$\mathcal{U}=\mathbb{R}^m$，$H=I$，$u_{\text{ref}}=0$）。

**拉格朗日量**：
$$\mathcal{L}(u,\delta,\lambda_V,\lambda_h) = \frac{1}{2}\|u\|^2 + p\delta^2 + \lambda_V(L_fV + L_gV\cdot u + \gamma V - \delta) + \lambda_h(-L_fh - L_gh\cdot u - \alpha(h))$$

**KKT 条件**：

1. **关于 $u$ 的驻点条件**：
$$\frac{\partial\mathcal{L}}{\partial u} = u + \lambda_V(L_gV)^\top - \lambda_h(L_gh)^\top = 0$$
$$\Rightarrow u^* = -\lambda_V(L_gV)^\top + \lambda_h(L_gh)^\top$$

2. **关于 $\delta$ 的驻点条件**：
$$\frac{\partial\mathcal{L}}{\partial\delta} = 2p\delta - \lambda_V = 0 \quad \Rightarrow \quad \delta^* = \frac{\lambda_V}{2p}$$

3. **原始可行性**（约束满足）：
$$L_fV + L_gV\cdot u + \gamma V - \delta \le 0$$
$$-L_fh - L_gh\cdot u - \alpha(h) \le 0$$

4. **对偶可行性**：$\lambda_V\ge 0$，$\lambda_h\ge 0$

5. **互补松弛**：
$$\lambda_V(L_fV + L_gV\cdot u + \gamma V - \delta) = 0$$
$$\lambda_h(-L_fh - L_gh\cdot u - \alpha(h)) = 0$$

#### 四种情形分析

| 情形 | $\lambda_V$ | $\lambda_h$ | 物理含义 | $u^*$ |
|---|---|---|---|---|
| 两约束均不激活 | $0$ | $0$ | $u_{\text{ref}}$ 同时安全且收敛 | $u_{\text{ref}}$（无修正） |
| 仅 CLF 激活 | $>0$ | $0$ | 远离障碍，需修正以保收敛 | $u_{\text{ref}} - \lambda_V(L_gV)^\top$ |
| 仅 CBF 激活 | $0$ | $>0$ | 已收敛但接近危险 | $u_{\text{ref}} + \lambda_h(L_gh)^\top$ |
| 两者均激活 | $>0$ | $>0$ | 冲突！牺牲稳定保安全 | 两力平衡 |

**当两者冲突时**：$\delta^*=\lambda_V/(2p)>0$，CLF 约束被软化（允许 $\dot V > -\gamma V$）。$p$ 越大→$\delta$ 越小→CLF 越接近硬约束→但可能导致 QP 不可行（因为硬 CLF + 硬 CBF 可能不相容）。

### 4.4 纯 Safety Filter 变体 ⭐⭐

**这是工程中最常用的形式**：给定名义控制 $u_{\text{nom}}(x)$（来自 RL 策略、MPC、遥操作、PD 控制器），做**最小修正**保证安全：

$$u^* = \arg\min_{u\in\mathcal{U}} \|u - u_{\text{nom}}\|^2 \quad \text{s.t.} \quad L_fh + L_gh\cdot u \ge -\alpha(h)$$

**最小侵入原则**（minimum intervention principle）：只在必须时修正，修正量最小化。RL 策略在安全时完全不被干扰，只在即将违规时被轻推回安全侧。

#### Safety Filter 的闭式解（无输入约束）

当 $\mathcal{U}=\mathbb{R}^m$，设 $a=L_fh(x)+\alpha(h(x))$，$b=L_gh(x)\in\mathbb{R}^{1\times m}$。CBF 约束变为 $bu\ge -a$，即 $a+bu\ge 0$。

**情况 1**：$a+b\,u_{\text{nom}}\ge 0$（名义控制已安全）：
$$u^* = u_{\text{nom}}$$
不修正。

**情况 2**：$a+b\,u_{\text{nom}}<0$（名义控制不安全）：

求 $u_{\text{nom}}$ 在半空间 $\{u:a+bu\ge 0\}$ 上的正交投影。结果：
$$u^* = u_{\text{nom}} + \lambda^* b^\top$$

其中对偶变量 $\lambda^* = \max\left\{0,\;\frac{-(a+b\,u_{\text{nom}})}{b\,b^\top}\right\}$。

**几何解释**：当 $u_{\text{nom}}$ 在不安全半空间 $\{u:a+bu<0\}$ 中时，$u^*$ 是 $u_{\text{nom}}$ 在超平面 $\{u:a+bu=0\}$ 上的正交投影。投影方向是 $b^\top$（超平面法向量），投影距离恰好使约束变为等式。

这个闭式解的计算量极小：一次内积（检查安全性）+ 一次向量加法（修正）。在无输入约束下甚至不需要 QP 求解器！

**Lipschitz 连续性**：当 $bb^\top>0$（即 $L_gh\neq 0$，相对度 1 的自然要求），解关于 $x$ Lipschitz 连续——保证闭环 ODE 解存在唯一（Morris-Powell-Ames 2013）。

### 4.5 可行性分析 ⭐⭐⭐

**什么时候 CLF-CBF-QP 不可行？**

QP 不可行意味着不存在 $u\in\mathcal{U}$ 同时满足所有约束。可能原因：

1. **CBF 与输入约束不相容**：$K_{\text{cbf}}(x)\cap\mathcal{U}=\varnothing$。即使存在满足 CBF 的 $u$，它可能超出执行器能力范围。
2. **多个 CBF 互相矛盾**：多障碍物 CBF 的交集 $\bigcap_i K_{\text{cbf}}^i(x)$ 为空（"boxed-in" 现象：机器人被障碍物包围，无论往哪走都违反某个 CBF）。
3. **CLF 作为硬约束时与 CBF 冲突**（这就是为什么 CLF 要松弛）。

**处理不可行的策略**：

| 策略 | 做法 | 适用场景 |
|---|---|---|
| CLF 松弛 | $\delta$ 变量（已内置） | CLF-CBF 冲突 |
| CBF 松弛 | 对次要 CBF 加松弛 $\epsilon_i$ | 多 CBF 冲突时牺牲低优先级 |
| 收紧安全集 | 减小 $\alpha$ 的 $\gamma$ | 预防性避免不可行 |
| 高层规划 | 路径规划避免进入不可行区域 | 全局策略 |

> **反事实推理**：如果不做可行性分析，直接把不可行的 QP 丢给求解器会怎样？OSQP 会返回 `infeasible` 状态，此时没有合法的控制输出。如果工程中没有处理这个情况（如回退到上一帧的控制或紧急制动），系统将在该帧没有控制输入——可能导致灾难性后果。**工程代码必须处理 QP 不可行的情况！**

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：松弛罚权 $p$ 设置不当**
>
> **$p$ 太小（如 $p=1$）**：$\delta$ 可以很大，CLF 约束几乎不起作用，系统只保安全不收敛——机器人停在原地不动。
>
> **$p$ 太大（如 $p=10^8$）**：$\delta\approx 0$，CLF 近似硬约束，与 CBF 冲突时 QP 不可行。
>
> **推荐范围**：$p\in[10^2, 10^4]$。实际调参：先设大 $p$，如果频繁不可行就减小。

> 💡 **概念误区：认为 Safety Filter "改变了" RL 策略**
>
> **实际情况**：Safety Filter 只在**必要时**修正，大部分时间 $u^*=u_{\text{nom}}$。它不改变策略的学习目标或训练过程。如果 RL 策略本身已经安全，Filter 永远不激活。
>
> **重要区别**：Safety Filter 是**推理时**的后处理（deployment-time），不是训练时的约束。训练时的安全约束是 Constrained RL（如 CPO、PCPO），两者互补。

### 练习 4.1

1. 对一维系统 $\dot x=u$，CLF $V=x^2/2$，CBF $h=1-x$（安全集 $x\le 1$），$\gamma=1$，$\alpha(h)=h$。手推 CLF-CBF-QP 的解在 $x=0.5$ 和 $x=0.9$ 处。
2. 编写 Python 代码实现 Safety Filter 的闭式解（无输入约束版本），对二维单积分器避开圆形障碍物。
3. **（跨章综合）** 回忆专题 3.7 的 LaSalle 不变性原理。CLF-QP 中 $\delta>0$ 时 $\dot V$ 可能为正——如何用 LaSalle 的思想分析此时系统的长期行为？

---

## 5. 多约束与工程实现 ⭐⭐

### 5.1 多障碍物多约束 CBF

#### 朴素方法：直接叠加

若有 $N$ 个安全约束 $h_1,\ldots,h_N$（每个障碍物一个），直接在 QP 中加 $N$ 条约束：

$$L_fh_i + L_gh_i\cdot u \ge -\alpha_i(h_i), \quad i=1,\ldots,N$$

保证 $\mathcal{C}=\bigcap_i\mathcal{C}_i=\bigcap_i\{h_i\ge 0\}$ 前向不变。

**可行性条件**：$\bigcap_i K_{\text{cbf}}^i(x)\cap\mathcal{U}\neq\varnothing$。当约束过多或安全集过小时可能不可行。

#### Composite CBF：smooth-min 合并

**Molnar-Ames 2023** 提出把多个 $h_i$ 合并为单一 smooth-min：
$$h(x) = -\frac{1}{\kappa}\log\sum_i e^{-\kappa h_i(x)}$$

当 $\kappa\to\infty$ 时趋向 $\min_i h_i(x)$。优点：只有一条 CBF 约束（而非 $N$ 条），QP 规模不随障碍物数增加。缺点：$\nabla h$ 的计算涉及所有 $h_i$ 的梯度加权和，可能较保守。

**权重设计**：为各 $\alpha_i$ 选不同强度——让更紧迫的约束（$h_i$ 更小的）优先级更高。

### 5.2 输入约束相容性分析 ⭐⭐⭐

**核心问题**：当 $\mathcal{U}$ 有界时，是否总存在安全控制？

**定义（控制不变集）**：$\mathcal{C}$ 是控制不变的，如果对所有 $x\in\mathcal{C}$，存在 $u\in\mathcal{U}$ 使 $x$ 留在 $\mathcal{C}$ 中。数学上：$K_{\text{cbf}}(x)\cap\mathcal{U}\neq\varnothing$ 对所有 $x\in\mathcal{C}$。

**可行性分析方法**：

1. **解析判定**：对简单系统，直接验证 CBF 约束在输入约束下是否总能满足。
2. **SOS/LP 验证**：对多项式系统，用 sum-of-squares 优化验证。
3. **HJ Reachability**：计算最大控制不变集（viability kernel），在此集内 CBF 保证可行。

### 5.3 OSQP 工程实现 ⭐⭐

#### 标准 QP 形式

OSQP 求解的标准形式为：
$$\min_x \frac{1}{2}x^\top Px + q^\top x \quad \text{s.t.} \quad l \le Ax \le u$$

**CBF-QP 到 OSQP 的映射**：

决策变量：$x=[u^\top, \delta]^\top\in\mathbb{R}^{m+1}$（含松弛）

目标矩阵：
$$P = \begin{bmatrix} H & 0 \\ 0 & 2p \end{bmatrix}, \quad q = \begin{bmatrix} -H\,u_{\text{ref}} \\ 0 \end{bmatrix}$$

约束矩阵：
$$A = \begin{bmatrix} L_gV & -1 \\ -L_gh & 0 \\ I_m & 0 \\ 0 & 1 \end{bmatrix}, \quad l = \begin{bmatrix} -\infty \\ -\infty \\ u_{\min} \\ -\infty \end{bmatrix}, \quad u = \begin{bmatrix} -L_fV-\gamma V \\ L_fh+\alpha(h) \\ u_{\max} \\ \infty \end{bmatrix}$$

```python
import osqp
import numpy as np
from scipy import sparse

class CLF_CBF_QP:
    """CLF-CBF-QP 控制器的完整实现"""
    
    def __init__(self, m, gamma_clf=1.0, gamma_cbf=1.0, p_slack=1000.0,
                 u_min=None, u_max=None):
        """
        参数:
            m: 控制输入维度
            gamma_clf: CLF 衰减率
            gamma_cbf: CBF 类K函数参数 (alpha(h) = gamma_cbf * h)
            p_slack: CLF 松弛罚权重
            u_min, u_max: 输入约束
        """
        self.m = m
        self.gamma_clf = gamma_clf
        self.gamma_cbf = gamma_cbf
        self.p_slack = p_slack
        self.u_min = u_min if u_min is not None else -np.inf * np.ones(m)
        self.u_max = u_max if u_max is not None else np.inf * np.ones(m)
        
        # 初始化 OSQP solver（warm-start 提升后续求解速度）
        self.solver = None
        self._setup_solver()
    
    def _setup_solver(self):
        """构建 QP 结构并初始化 OSQP"""
        m = self.m
        n_var = m + 1  # [u, delta]
        
        # 目标矩阵 P（上三角稀疏）
        P_data = np.ones(n_var)
        P_data[-1] = 2 * self.p_slack  # delta 的权重
        self.P = sparse.diags(P_data, format='csc')
        
        # 约束矩阵初始化（结构固定，数值每步更新）
        # 行：CLF约束(1) + CBF约束(1) + 输入约束(2m)
        n_con = 2 + 2 * m
        self.A = sparse.csc_matrix((n_con, n_var))
        self.l = np.zeros(n_con)
        self.u_bound = np.zeros(n_con)
        
    def solve(self, LfV, LgV, V, Lfh, Lgh, h, u_ref=None):
        """
        求解 CLF-CBF-QP
        
        参数:
            LfV: L_f V(x), 标量
            LgV: L_g V(x), shape (1, m) 或 (m,)
            V: V(x), 标量（正定）
            Lfh: L_f h(x), 标量
            Lgh: L_g h(x), shape (1, m) 或 (m,)
            h: h(x), 标量
            u_ref: 参考控制, shape (m,)
        
        返回:
            u_opt: 最优控制, shape (m,)
            delta: CLF 松弛量
            info: 求解信息字典
        """
        m = self.m
        LgV = np.atleast_1d(LgV).flatten()
        Lgh = np.atleast_1d(Lgh).flatten()
        
        if u_ref is None:
            u_ref = np.zeros(m)
        
        # 目标向量 q = [-u_ref, 0]
        q = np.concatenate([-u_ref, [0.0]])
        
        # 构建约束
        # CLF: LfV + LgV*u + gamma*V - delta <= 0
        #   => LgV*u - delta <= -LfV - gamma*V
        # CBF: Lfh + Lgh*u >= -alpha(h)
        #   => -Lgh*u <= Lfh + alpha(h)
        # 输入: u_min <= u <= u_max
        
        alpha_h = self.gamma_cbf * h  # 线性类K函数
        
        # 构建稀疏约束矩阵
        # 第1行：CLF [LgV, -1]
        # 第2行：CBF [-Lgh, 0]
        # 第3-m+2行：u下界 [I, 0]
        # 第m+3-2m+2行：u上界 [I, 0]
        
        rows = []
        cols = []
        data = []
        
        # CLF 约束行
        for j in range(m):
            if LgV[j] != 0:
                rows.append(0); cols.append(j); data.append(LgV[j])
        rows.append(0); cols.append(m); data.append(-1.0)  # -delta
        
        # CBF 约束行
        for j in range(m):
            if Lgh[j] != 0:
                rows.append(1); cols.append(j); data.append(-Lgh[j])
        
        # 输入约束
        for j in range(m):
            rows.append(2 + j); cols.append(j); data.append(1.0)
        
        n_con = 2 + m
        A = sparse.csc_matrix((data, (rows, cols)), shape=(n_con, m+1))
        
        l = np.concatenate([[-np.inf],        # CLF 无下界
                           [-np.inf],          # CBF 无下界  
                           self.u_min])        # 输入下界
        
        u_ub = np.concatenate([[-LfV - self.gamma_clf * V],  # CLF 上界
                               [Lfh + alpha_h],               # CBF 上界
                               self.u_max])                   # 输入上界
        
        # 设置并求解
        solver = osqp.OSQP()
        solver.setup(self.P[:m+1,:m+1], q, A, l, u_ub, 
                    verbose=False, eps_abs=1e-6, eps_rel=1e-6,
                    max_iter=4000, polish=True)
        result = solver.solve()
        
        if result.info.status != 'solved':
            return None, None, {'status': result.info.status, 'feasible': False}
        
        u_opt = result.x[:m]
        delta = result.x[m]
        
        return u_opt, delta, {'status': 'solved', 'feasible': True, 
                             'solve_time_ms': result.info.run_time * 1000}
```

#### 实时性分析

| 系统规模 | $n_u$ | 约束数 | OSQP 求解时间 | 适合频率 |
|---|---|---|---|---|
| 单输入 | 1 | 3 | 5-20 μs | 10+ kHz |
| 四足 WBC | 12 | 20-30 | 50-200 μs | 1-5 kHz |
| 操作臂 | 7 | 15-20 | 30-100 μs | 2-5 kHz |
| 多机器人(10) | 20 | 50-100 | 0.5-2 ms | 500 Hz |

**关键工程优化**：
1. **Warm-start**：上一步的解作为下一步初始猜测（OSQP 原生支持），减少 ADMM 迭代数
2. **代码生成**：OSQP 支持 C 代码生成，嵌入微控制器（STM32 级）
3. **稀疏利用**：CBF 约束矩阵 $L_gh$ 通常稀疏，利用稀疏结构减少内存和计算

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：OSQP 的约束方向搞反**
>
> **错误**：OSQP 格式是 $l\le Ax\le u$，但 CBF 约束是 $L_gh\cdot u\ge -(L_fh+\alpha h)$。如果直接写成 $A=[L_gh]$，$l=[-(L_fh+\alpha h)]$，**方向是对的**。但如果不小心写成 $A=[-L_gh]$，$u=[L_fh+\alpha h]$，约束也等价——但容易在有多个约束时搞混。
>
> **建议**：统一将所有约束写成 $Ax\le b$ 的形式，然后映射到 OSQP 的 $l\le Ax\le u$（设 $l=-\infty$，$u=b$）。

> 🧠 **思维陷阱：认为 QP 求解时间固定**
>
> **实际情况**：OSQP 使用 ADMM 迭代，求解时间取决于问题 conditioning 和约束激活情况。当约束接近激活/退激活边界时，收敛变慢。**最坏情况**比平均情况慢 5-10 倍。实时系统必须设置 `max_iter` 上限并处理"未收敛"情况。

### 练习 5.1

1. 将上述 Python 代码扩展为支持多个 CBF 约束（$N$ 个障碍物），并测试 $N=5$ 个圆形障碍物的场景。
2. 分析当两个障碍物 CBF 约束正好相反方向时（"被夹在中间"），QP 的可行域是什么形状？何时为空？
3. 实现 warm-start 机制：保存上一步解作为下一步初始猜测。对比有无 warm-start 的求解时间。

### 5.4 离散时间 CBF（DCBF） ⭐⭐⭐

#### 动机：实际机器人是采样控制的

真实机器人控制器以固定频率（如 1 kHz）运行。在两次控制更新之间，系统在零阶保持（ZOH）下演化。连续 CBF 条件 $\dot h\ge -\alpha(h)$ 在离散实现中只是近似成立——特别是当采样间隔 $\Delta t$ 较大或系统动态较快时，连续 CBF 不能保证离散系统的安全性。

**反例**：考虑快速系统 $\dot x = 10u$，采样周期 $\Delta t=0.1$s。连续 CBF 在 $t=0$ 时检查安全，但在 $t\in(0, 0.1)$ 期间系统可能已经越界。

#### DCBF 定义（Agrawal-Sreenath RSS 2017）

对离散系统 $x_{k+1} = F(x_k, u_k)$，安全集 $\mathcal{C}=\{x:h(x)\ge 0\}$。

**定义（Discrete-Time CBF）**：$h$ 是 DCBF，如果存在 $\alpha\in(0,1]$ 使得：
$$\sup_{u_k\in\mathcal{U}} h(F(x_k, u_k)) \ge (1-\alpha)\,h(x_k) \quad \forall x_k\in\mathcal{C}$$

等价约束形式（对具体控制 $u_k$）：
$$h(x_{k+1}) - h(x_k) \ge -\alpha\,h(x_k)$$

即 $h(x_{k+1}) \ge (1-\alpha)\,h(x_k)$。

**为什么 $\alpha\in(0,1]$？**
- $\alpha=0$：$h(x_{k+1})\ge h(x_k)$，安全裕度永不减少——过于保守
- $\alpha=1$：$h(x_{k+1})\ge 0$，只要下一步安全即可——最宽松
- $\alpha\in(0,1)$：安全裕度几何衰减 $h(x_k)\ge (1-\alpha)^k h(x_0)$——指数收敛到零但不穿零

**与连续 CBF 的关系**：连续 CBF 条件 $\dot h\ge -\gamma h$ 在 Euler 离散化下变为：
$$\frac{h_{k+1}-h_k}{\Delta t}\ge -\gamma h_k \implies h_{k+1}\ge (1-\gamma\Delta t)h_k$$

对应 $\alpha=\gamma\Delta t$。当 $\gamma\Delta t>1$ 时失去意义——这正是连续 CBF 在低频采样下失效的原因。

#### DCBF 的挑战

DCBF 约束 $h(F(x_k,u_k))\ge (1-\alpha)h(x_k)$ 关于 $u_k$ 通常是**非线性**的（因为 $F$ 非线性），不能直接写成 QP。解决方案：

1. **一阶 Taylor 近似**：$h(F(x_k,u_k))\approx h(F(x_k,0))+\nabla_u h(F)\cdot u_k$，回到线性约束
2. **MPC 框架内处理**：把 DCBF 作为 NLP 约束（MPC-CBF 方法）
3. **Control-affine 特殊结构**：若 $F(x,u)=F_0(x)+G(x)u$（仿射），则 $h(F)$ 可能仍是 $u$ 的仿射函数

```python
# DCBF 实现示例（线性系统）
import numpy as np

def dcbf_constraint(x, A, B, h_func, grad_h, alpha=0.3):
    """
    离散线性系统 x_{k+1} = Ax + Bu 的 DCBF 约束
    约束：h(Ax + Bu) >= (1-alpha)*h(x)
    对线性 h(x) = c^T x + d，约束为 c^T(Ax+Bu) + d >= (1-alpha)*(c^T x + d)
    即 c^T B u >= (1-alpha)*(c^T x + d) - c^T A x - d
    """
    h_x = h_func(x)
    # 下一步状态的 h 值的线性部分
    x_next_free = A @ x  # 无控制时的下一状态
    h_next_free = h_func(x_next_free)
    
    # CBF 约束：Lgh * u >= rhs
    Lgh = grad_h @ B  # 1xm 向量
    rhs = (1 - alpha) * h_x - h_next_free
    
    return Lgh, rhs  # 约束为 Lgh @ u >= rhs
```

### 5.5 ROS2 集成架构 ⭐⭐

#### 控制栈中的位置

在典型的 ROS2 机器人控制栈中，CBF Safety Filter 位于：

```
[高层规划 10Hz] → [MPC/轨迹优化 50-100Hz] → [CBF Safety Filter 500-1000Hz] → [底层驱动]
                                                       ↑
                                              [状态估计] + [障碍物感知]
```

**关键设计原则**：

1. **频率分离**：Safety Filter 频率必须 >= 名义控制器频率（否则安全检查存在盲区）
2. **最小延迟**：从传感器到 Safety Filter 输出的 pipeline 延迟 < 2ms（含状态估计）
3. **故障安全**：若 QP 不可行或超时，执行预设的安全动作（如零力矩/紧急制动）

#### cbfkit ROS2 节点结构

```python
# 基于 cbfkit 的 ROS2 Safety Filter 节点骨架
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class CBFSafetyFilterNode(Node):
    """ROS2 CBF Safety Filter 节点"""
    
    def __init__(self):
        super().__init__('cbf_safety_filter')
        
        # 参数
        self.declare_parameter('gamma_cbf', 1.0)
        self.declare_parameter('control_freq', 500.0)  # Hz
        
        # 订阅名义控制（来自 MPC/RL）
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_nominal', self.cmd_callback, 10)
        
        # 订阅状态估计
        self.sub_state = self.create_subscription(
            PoseStamped, '/state_estimate', self.state_callback, 10)
        
        # 订阅障碍物信息
        self.sub_obs = self.create_subscription(
            Float64MultiArray, '/obstacles', self.obs_callback, 10)
        
        # 发布安全控制
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        # 定时器：以固定频率运行 Safety Filter
        dt = 1.0 / self.get_parameter('control_freq').value
        self.timer = self.create_timer(dt, self.filter_callback)
        
        self.u_nom = np.zeros(2)
        self.state = np.zeros(4)
        self.obstacles = []
        
    def filter_callback(self):
        """核心回调：执行 CBF-QP 安全过滤"""
        gamma = self.get_parameter('gamma_cbf').value
        
        # 为每个障碍物构建 CBF 约束
        constraints = []
        for obs in self.obstacles:
            Lfh, Lgh, h = self.compute_cbf(self.state, obs)
            constraints.append((Lfh, Lgh, h, gamma))
        
        # 求解 QP
        u_safe = self.solve_cbf_qp(self.u_nom, constraints)
        
        # 发布安全控制
        msg = Twist()
        msg.linear.x = float(u_safe[0])
        msg.angular.z = float(u_safe[1])
        self.pub_cmd.publish(msg)
```

---

## 6. 高级主题

前面五节建立了 CLF-CBF-QP 的完整理论与工程实现框架，但假设了两个理想条件：(1) 系统模型 $f,g$ 精确已知；(2) 安全函数 $h$ 可以手工设计。真实机器人几乎不满足这两个假设——模型有误差、环境有噪声、高维系统的 $h$ 无法手工构造。本节介绍突破这些限制的前沿方法，从鲁棒 CBF 到神经网络学习 CBF，从随机安全到 HJ 可达性的统一。

### 6.1 Robust CBF（ISSf-CBF） ⭐⭐⭐

#### 动机：模型不确定性如何影响安全

真实系统存在模型误差：$\dot x = f(x) + g(x)u + d(t)$，$\|d\|\le d_{\max}$。标准 CBF 假设完美模型——扰动 $d$ 可能使 CBF 约束在实际中被违反。

**Input-to-State Safety (ISSf)**（Kolathaya-Ames 2019, IEEE L-CSS 3(1):108-113, arXiv:1803.03035）直接类比专题 3.7（§7.10）中 Sontag 的 ISS（Input-to-State Stability）。回顾 ISS 的核心思想：$\dot V\le -\alpha(\|x\|)+\sigma(\|u\|)$ 意味着"扰动有界则状态有界"。ISSf 将同样的思想翻译到安全语言：

$$\sup_u[L_fh + L_gh\cdot u] \ge -\alpha(h(x)) + \iota(\|d\|)$$

其中 $\iota$ 为类 $\mathcal{K}$ 函数。

**结论**：扰动下 $\mathcal{C}$ 的一个**膨胀集** $\mathcal{C}_d\supseteq\mathcal{C}$ 前向不变，膨胀量随 $d_{\max}$ 增大。直觉：扰动使安全边界"模糊化"——实际安全集比标称的小。

**工程实现**：在 CBF 约束中加 margin：
$$L_fh + L_gh\cdot u \ge -\alpha(h) + d_{\max}\|\nabla h\|$$

右边多出的 $d_{\max}\|\nabla h\|$ 补偿了最坏情况扰动对 $\dot h$ 的影响。

### 6.2 CBF 与 MPC 的互补统一 ⭐⭐⭐

#### 两者的优劣对比

| 特性 | CBF-QP | MPC |
|---|---|---|
| 时间跨度 | 瞬时（一步） | 有限时域预测 |
| 计算复杂度 | $O(m^2)$，μs 级 | $O(N\cdot m^2)$，ms 级 |
| 预见性 | 无（只看当前梯度） | 有（预测未来轨迹） |
| 保证 | 严格（理论证明） | 条件性（需 recursive feasibility） |
| 适用频率 | 1-10 kHz | 10-100 Hz |

#### MPC-CBF 统一

**Zeng-Zhang-Sreenath (ACC 2021, arXiv:2007.11718)** 把 DCBF 作为 MPC 每一预测步的硬约束：

$$\min_{u_0,\ldots,u_{N-1}} \sum_{k=0}^{N-1} \ell(x_k,u_k) + V_f(x_N)$$
$$\text{s.t.} \quad x_{k+1}=F(x_k,u_k), \quad h(x_{k+1})\ge(1-\alpha)h(x_k), \quad u_k\in\mathcal{U}$$

**优势**：短预测时域（$N=5\sim 10$）即可实现远距离避障——因为 DCBF 在每步都保证安全裕度不快速衰减，即使没有预测到很远也安全。

### 6.3 CBF 作为 RL Safety Filter ⭐⭐⭐

#### 动机：RL 策略天然不安全

强化学习通过试错探索环境。在真实机器人上，"错"可能意味着碰撞、坠落、损坏——不可接受。CBF-QP 作为 Safety Filter 是当前最成熟的解决方案。

**Cheng-Orosz-Murray-Burdick (AAAI 2019, arXiv:1903.08792)** 的架构：

```
RL Policy π_θ(s) → u_nom → CBF-QP Safety Filter → u_safe → Environment
                                    ↑
                              h(x), ∇h(x)
```

**关键技巧：可微 QP 层**

使用 `cvxpylayers`（Agrawal et al. 2019）或 `qpth`（Amos-Kolter 2017）将 QP 封装为可微层，反向传播通过 KKT 条件的隐函数微分。这让 RL 策略"感知"到安全约束的存在：

$$\nabla_\theta J = \mathbb{E}\left[\nabla_\theta \log\pi_\theta(u_{\text{nom}}) \cdot \frac{\partial u_{\text{safe}}}{\partial u_{\text{nom}}} \cdot R\right]$$

其中 $\partial u_{\text{safe}}/\partial u_{\text{nom}}$ 通过 QP 的 KKT 条件隐式微分获得。

**工程结果**：策略在训练过程中学会"预判"安全约束，逐渐减少 Filter 的激活频率——即策略学会了安全行为，而非依赖后处理。

### 6.4 Neural CBF ⭐⭐⭐⭐

#### 动机：手工设计 $h$ 对高维系统不可行

对简单系统（如点质量+圆形障碍物），手工设计 $h$ 很自然。但对高维系统（如 12-DoF 四旋翼、视觉感知的移动机器人），手工设计几乎不可能。

**Neural CBF** 将 $h_\theta$ 参数化为神经网络，通过学习获得有效的 CBF。

**训练损失**（Dawson-Gao-Fan 2023, T-RO, arXiv:2202.11762）：

$$\mathcal{L}(\theta) = \underbrace{\lambda_1\mathcal{L}_{\text{class}}}_{\text{安全/不安全分类}} + \underbrace{\lambda_2\mathcal{L}_{\text{descent}}}_{\text{CBF条件满足}} + \underbrace{\lambda_3\mathcal{L}_{\text{boundary}}}_{\text{边界正则化}}$$

- $\mathcal{L}_{\text{class}}$：$h_\theta>0$ 在安全演示上，$h_\theta<0$ 在碰撞状态
- $\mathcal{L}_{\text{descent}}$：$\max(0, -L_fh_\theta - L_gh_\theta u - \alpha(h_\theta))$ 的采样均值
- $\mathcal{L}_{\text{boundary}}$：鼓励 $h_\theta=0$ 的等值面接近真实安全边界

**验证**：训练后用 dReal/Z3（SMT 求解器）或 SOS（Sum-of-Squares）验证 $h_\theta$ 在整个状态空间满足 CBF 条件。若验证失败，用反例点增广训练集重新训练。

代码仓库：**https://github.com/MIT-REALM/neural_clbf**

### 6.5 Stochastic CBF ⭐⭐⭐⭐

#### 动机：噪声无处不在

对随机系统（如有传感器噪声的机器人、风场中的无人机），确定性安全不可能——只能保证**概率安全**。

对 Ito 随机微分方程：
$$dx = (f(x) + g(x)u)\,dt + \sigma(x)\,dW_t$$

其中 $W_t$ 是标准 Brownian 运动（Wiener 过程），$\sigma(x)$ 是扩散系数矩阵。

**Ito 公式**对 $B(x)$ 求广义导数（与确定性情况的关键区别）：
$$d B(x_t) = \underbrace{\nabla B\cdot(f+gu)}_{\text{漂移项}} dt + \underbrace{\frac{1}{2}\operatorname{tr}(\sigma^\top\nabla^2 B\,\sigma)}_{\text{扩散项（Ito修正）}} dt + \nabla B\cdot\sigma\,dW_t$$

**扩散项的物理含义**：$\frac{1}{2}\operatorname{tr}(\sigma^\top\nabla^2 B\,\sigma)$ 是噪声对 $B$ 的"平均推动力"。即使漂移方向安全，噪声的二阶效应（通过 $B$ 的曲率 $\nabla^2 B$）可能让系统向不安全方向偏移。这是随机系统比确定性系统更"危险"的数学原因。

**Stochastic CBF 条件**（Prajna-Jadbabaie-Pappas 2007, Clark 2021）：

$$\sup_u\left[\nabla B\cdot(f+gu) + \frac{1}{2}\operatorname{tr}(\sigma^\top\nabla^2 B\,\sigma)\right] \ge -\alpha(B(x))$$

若此条件满足，$B(x_t)$ 是 **supermartingale**（期望不增），由 Doob 最大不等式得概率安全界：
$$\mathbb{P}\left(\inf_{t\ge 0} h(x_t) < 0\right) \le \frac{B(x_0)}{c}$$

**工程含义**：为了保证高概率安全，CBF 约束需要更"保守"——额外减去扩散项的贡献。这等价于在安全边界内部预留一个"噪声裕度带"。

### 6.6 CBF 与 HJ Reachability 的关系（CBVF） ⭐⭐⭐⭐

#### 两种安全方法的统一

HJ Reachability（Hamilton-Jacobi Reachability Analysis）通过求解偏微分方程（HJI-VI）获得**最大鲁棒安全集**（viability kernel）。它给出最优解但计算量随维度指数增长。CBF 计算量小（只需求解 QP）但需要手工设计 $h$——设计不当的 $h$ 可能过于保守或不正确。

**CBVF（Control Barrier-Value Function, Choi-Lee-Sreenath-Tomlin-Herbert CDC 2021, arXiv:2104.02808）** 统一了两者：

引入折扣参数 $\gamma\ge 0$，CBVF $B_\gamma(x,t)$ 是以下 HJI-VI 的粘性解：
$$0 = \min\left\{l(x)-B_\gamma,\;\partial_t B_\gamma + \max_{u\in\mathcal{U}}\min_{d\in\mathcal{D}}\nabla_x B_\gamma\cdot f(x,u,d) + \gamma B_\gamma\right\}$$

**关键事实**：
- 当 $\gamma=0$：退化为经典 HJ Reachability，$\{B_0\ge 0\}$ 是 viability kernel
- 当 $\gamma>0$：内嵌 CBF 衰减 $\alpha(r)=\gamma r$，$\{B_\gamma\ge 0\}$ 仍精确等于 viability kernel

**实践意义**：用 HJ 方法（如 `hj_reachability` 库）在离线阶段计算 $B_\gamma$，然后在线用 Robust CBVF-QP：
$$\min_u\|u-u_{\text{nom}}\|^2 \quad\text{s.t.}\quad\min_{d\in\mathcal{D}}\nabla_x B_\gamma\cdot f(x,u,d)\ge -\gamma B_\gamma$$

这解决了手工设计 CBF 的痛点——用 HJ 自动生成有效且紧致的 CBF。缺点是 HJ 只能处理低维系统（当前实用上限约 $n\le 6$）。

### 6.7 Data-Driven CBF ⭐⭐⭐⭐

#### GP-CBF：用高斯过程估计模型不确定性

当系统模型 $f,g$ 不精确时，CBF 条件中的 $L_fh$、$L_gh$ 计算有误差。**GP-CBF**（Khojasteh-Dhiman-Franceschetti-Atanasov L4DC 2020, arXiv:1912.10452）用高斯过程（Gaussian Process）估计动力学残差：

$$f(x) = \hat{f}(x) + \Delta f(x), \quad \Delta f\sim\mathcal{GP}(0, k(x,x'))$$

GP 后验给出 $\Delta f$ 的均值 $\mu(x)$ 和方差 $\sigma^2(x)$。安全约束变为：
$$L_{\hat{f}}h + L_gh\cdot u + \mu_{\Delta f}^\top\nabla h - \beta\sigma_{\Delta f}\|\nabla h\| \ge -\alpha(h)$$

其中 $\beta$ 控制置信度（$\beta=2$ 对应约 95% 概率安全）。方差 $\sigma$ 大的区域约束更紧（更保守）——这是合理的：不确定性大时应更谨慎。

#### 从演示中学习 CBF

**Robey et al. (CDC 2020, arXiv:2004.03315)** 从专家安全轨迹反求 $h$：观察安全演示（始终 $h>0$）和不安全演示（$h<0$），训练分类器 $h_\theta$ 区分两者。在 Lipschitz 假设下给出可证明安全性保证。

> **反事实推理**：如果不用 data-driven 方法而坚持手工设计 CBF 会怎样？对简单系统（如 2D 移动机器人+圆形障碍物）手工 CBF 足够。但对复杂环境（如杂乱室内、动态行人、变形障碍物），手工设计需要对每个新环境重新工程——不可扩展。Data-driven 方法一次学习即可泛化到新环境。

---

## 7. 应用实例与完整推导 ⭐⭐

### 7.1 自适应巡航控制（ACC）——完整推导

**Ames 2017 原论文的旗舰例子**，也是理解 CBF 最直觉的入口。

#### 系统建模

考虑高速公路上两车跟驰场景。状态向量 $x=[z, v_e, v_l]^\top$：
- $z=p_l-p_e$：前车与自车的纵向距离
- $v_e$：自车（ego）速度
- $v_l$：前车（lead）速度

动力学：$\dot z = v_l - v_e$，$\dot v_e = u$，$\dot v_l = a_l(t)$（前车加速度，不可控）。

控制仿射形式：$f(x)=[v_l-v_e, 0, a_l]^\top$，$g(x)=[0,1,0]^\top$。

#### 安全约束设计

**Time-headway 准则**（ISO 15622 ACC 标准）：安全距离 = $\tau_h\cdot v_e$，$\tau_h=1.8$ 秒。

$$h(x) = z - \tau_h v_e = (p_l-p_e) - 1.8\,v_e$$

**验证相对度**：$L_gh = \nabla h\cdot g = [1,-1.8,0]\cdot[0,1,0]^\top = -1.8\neq 0$ → 相对度 1。

#### CBF-QP 构造与求解

选 $\alpha(h)=\gamma h$，CBF 约束展开：
$$L_fh + L_gh\cdot u \ge -\gamma h$$
$$(v_l-v_e) + (-1.8)u \ge -\gamma(z-1.8v_e)$$
$$u \le \frac{(v_l-v_e)+\gamma(z-1.8v_e)}{1.8}$$

**物理解读**：当 $v_l<v_e$（前车比你慢）且 $z-1.8v_e$ 小（车距不富裕），最大允许加速度可以为负——即要求减速。$\gamma$ 控制"多早开始减速"：$\gamma$ 大→更早减速但更保守，$\gamma$ 小→允许更晚减速但更激进。

#### 数值仿真

设 $v_e=30$ m/s，$v_l=25$ m/s，$z=60$ m，$\gamma=1$：

- 安全裕度：$h=60-1.8\times30=6$ m
- 最大允许加速度：$u_{\max}=(25-30+1\times6)/1.8=1/1.8=0.56$ m/s$^2$

若名义控制 $u_{\text{nom}}=2$ m/s$^2$，Safety Filter 修正为 $u^*=0.56$ m/s$^2$。

```python
# ACC CBF-QP 仿真核心代码
import numpy as np

def acc_cbf_step(z, v_e, v_l, u_nom, gamma=1.0, tau_h=1.8, 
                 u_min=-5.0, u_max=3.0):
    """单步 ACC CBF Safety Filter"""
    h = z - tau_h * v_e
    # CBF 上界：u <= (v_l - v_e + gamma*h) / tau_h
    u_cbf_max = (v_l - v_e + gamma * h) / tau_h
    # Safety Filter：取名义控制和 CBF 上界的较小值
    u_safe = min(u_nom, u_cbf_max)
    return np.clip(u_safe, u_min, u_max), h
```

### 7.2 二维单积分器避障——渐隐示例 ⭐⭐

#### Phase 1：完整 worked example

**系统**：$\dot p = u$，$p=[x,y]^\top$，$u=[u_x,u_y]^\top$（单积分器，$f=0$，$g=I$）

**障碍物**：圆心 $p_o=[3,3]$，半径 $R=1$

**CBF**：$h(p)=\|p-p_o\|^2-R^2=(x-3)^2+(y-3)^2-1$

**计算**：
- $\nabla h = [2(x-3), 2(y-3)]$
- $L_fh = 0$（因为 $f=0$）
- $L_gh = \nabla h\cdot I = [2(x-3), 2(y-3)]$

**CBF 约束**（$\alpha(h)=h$）：$2(x-3)u_x + 2(y-3)u_y \ge -(h)$

**Safety Filter**：$\min\|u-u_{\text{nom}}\|^2$ s.t. 上述线性约束

#### Phase 2：填空（请读者完成）

障碍物改为 $p_o=[2,4]$，$R=0.5$。完成：$h(p)=$____，$\nabla h=$____，CBF 约束：____$\ge$____。

#### Phase 3：独立设计

三个障碍物 $p_1=[2,2],R_1=0.8$；$p_2=[4,3],R_2=1.0$；$p_3=[3,5],R_3=0.6$。设计完整的多 CBF QP 使机器人从 $[0,0]$ 到 $[6,6]$。

### 7.3 四旋翼安全着陆（HOCBF 应用） ⭐⭐⭐

#### 系统模型

四旋翼简化为质心平移动力学（忽略姿态环，假设内环快）：
$$\ddot p = \frac{1}{m}F_{\text{thrust}} - g e_3 + d$$

状态 $x=[p^\top, \dot p^\top]^\top\in\mathbb{R}^6$，控制 $u=F_{\text{thrust}}\in\mathbb{R}^3$。

#### 安全约束与相对度

**高度下限**：$h_1(x) = z - z_{\min}$

- $\dot h_1 = \dot z$（不含 $u$）→ $L_gh_1=0$
- $\ddot h_1 = \ddot z = F_z/m - g$（含 $u$）→ 相对度 2

**需要 HOCBF**！构造递归：
- $\psi_0 = h_1 = z - z_{\min}$
- $\psi_1 = \dot\psi_0 + \alpha_1(\psi_0) = \dot z + \gamma_1(z-z_{\min})$
- HOCBF 约束：$\dot\psi_1 + \alpha_2(\psi_1)\ge 0$

展开：$\ddot z + \gamma_1\dot z + \gamma_2(\dot z + \gamma_1(z-z_{\min}))\ge 0$

即：$F_z/m - g + (\gamma_1+\gamma_2)\dot z + \gamma_1\gamma_2(z-z_{\min})\ge 0$

$$F_z \ge m\left[g - (\gamma_1+\gamma_2)\dot z - \gamma_1\gamma_2(z-z_{\min})\right]$$

**极点配置**：取 $\gamma_1=\gamma_2=2$（重极点 $s=-2$），约束为：
$$F_z \ge m[g - 4\dot z - 4(z-z_{\min})]$$

当 $z$ 接近 $z_{\min}$ 且 $\dot z<0$（下降中），约束要求 $F_z$ 增大——即增加推力以减缓下降。

#### 多约束 QP

同时考虑高度、速度、姿态约束：

$$\min_{u\in\mathbb{R}^3}\|u-u_{\text{nom}}\|^2$$
$$\text{s.t.}\quad \text{HOCBF}(h_1)\ge 0,\quad \text{CBF}(h_2)\ge 0,\quad \text{CBF}(h_3)\ge 0,\quad u\in\mathcal{U}$$

### 7.4 多机器人避障 ⭐⭐⭐

#### 成对 CBF 构造

**Wang-Ames-Egerstedt (T-RO 2017)** 为 $N$ 个单积分器机器人两两定义：
$$h_{ij}(x) = \|p_i-p_j\|^2 - (2r)^2$$

$2r$ 是两机器人最小允许距离。

#### 分布式 vs 集中式

**集中式**：所有 $N(N-1)/2$ 约束放入一个 $Nm$ 维 QP。保证全局安全但计算量 $O(N^3)$。

**分布式**：每个机器人只考虑邻居。但问题是：机器人 $i$ 和 $j$ 对同一约束 $h_{ij}$ 各自优化——可能一个想左闪，一个也想左闪→撞！

**解决方案（consistent perturbation）**：规定优先级（如 ID 小的优先），低优先级机器人在其约束中额外考虑高优先级机器人的"预期动作"。

#### 死锁与对称性破缺

两个机器人正面相遇：对称 CBF 给出对称修正→都停下→死锁。解决方法：
1. 非对称优先级：ID 小的"让路"方向固定（如向右）
2. 随机扰动：加小的随机偏置打破对称
3. 高层规划：全局路径规划避免正面相遇

### ⚠️ 常见陷阱（应用实例）

> ⚠️ **编程陷阱：ACC 中忘记前车速度可能为零**
>
> 当前车完全停下（$v_l=0$）而自车仍在行驶时，$h$ 可能已经为负。此时 CBF 要求紧急制动，但如果 $u_{\min}$ 不够大（制动力有限），QP 不可行。工程中需要预留制动距离余量。

> 💡 **概念误区：认为相对度 2 的 HOCBF 一定比相对度 1 的 CBF "难"**
>
> 从实现角度看，HOCBF 只是多了一层递归和两个参数（$\gamma_1,\gamma_2$）。用极点配置自动选参后，代码复杂度增加很小。真正困难的是：(1) 确定系统的准确相对度；(2) 当系统有模型不确定性时高阶 Lie 导数的计算误差会累积。

### 练习 7.1

1. 在 ACC 例子中，若前车突然以 $a_l=-8$ m/s$^2$ 紧急制动，自车制动能力为 $u_{\min}=-5$ m/s$^2$。计算保证安全所需的最小车距 $z_{\min}$（作为 $v_e,v_l$ 的函数）。
2. 对四旋翼高度 HOCBF，如果选 $\gamma_1=1,\gamma_2=3$（不同极点），写出特征多项式并分析稳定性。
3. **（跨章综合）** 将多机器人 CBF 与专题 3.7 的 Lyapunov 稳定性结合：设计一个 CLF-CBF-QP 使 3 个机器人从随机位置同时到达各自目标且不碰撞。

---

## 8. CBF 理论局限与开放问题 ⭐⭐⭐

### 8.1 未期望平衡点（死锁）

**Reis-Aguiar-Tabuada 2020 (L-CSS 5(2):731-736, arXiv:2003.07819)** 证明：CLF-CBF-QP 会在安全集边界上引入除 CLF 极小点外的**渐近稳定平衡点**。

**物理现象**：机器人卡在障碍物前无法绕行——CLF 指向目标（穿过障碍），CBF 阻止靠近障碍。两力平衡的点成为"陷阱"。

**解决方案**：
- 引入旋转场（rotation-based guidance）打破对称
- 高层路径规划提供绕行 $u_{\text{ref}}$
- Navigation Function 设计无局部极小的 CLF

### 8.2 输入约束下的可行性

当 $\mathcal{U}$ 有界时 $K_{\text{cbf}}(x)\cap\mathcal{U}$ 可能为空。这是 CBF 理论中最实际的挑战之一。

**可行性丧失的物理场景**：

1. **制动力不足**：车辆以 100km/h 冲向 5m 外的墙壁，即使最大制动（$u_{\min}=-10$ m/s$^2$）也无法避免碰撞。
2. **多约束夹击**：机器人被三面墙包围，任何方向的移动都违反某个 CBF。
3. **高速接近**：系统以高速冲向边界，当前距离的安全裕度 $h$ 仍为正，但所需制动力超出能力。

**预防策略**：

$$\text{可行性} \iff L_fh(x) + \max_{u\in\mathcal{U}}L_gh(x)\cdot u\ge -\alpha(h(x))$$

对 $\mathcal{U}=\{u:\|u\|\le u_{\max}\}$，$\max_u L_gh\cdot u = \|L_gh\|\cdot u_{\max}$。可行条件变为：
$$L_fh(x) + \|L_gh(x)\|\cdot u_{\max}\ge -\alpha(h(x))$$

若此条件在某状态不满足，说明该状态"不可救"——应从更高层（如路径规划）避免进入此区域。这正是 **控制不变集** 的概念：最大的集合使得内部所有状态都可行。

### 8.3 高维 Neural CBF 验证

SMT 验证在 $n>6$ 时计算量爆炸。当前方法：

| 方法 | 适用维度 | 保证类型 | 代表工作 |
|---|---|---|---|
| SMT (dReal/Z3) | $n\le 4$ | 形式化完备 | Dawson 2023 |
| SOS (多项式) | $n\le 6$ | 形式化（多项式系统） | Prajna 2007 |
| Lipschitz 边界 | $n\le 10$ | 概率性 + Lipschitz 保证 | Fazlyab 2019 |
| 随机采样验证 | 任意 $n$ | 概率性（PAC bound） | Robey 2020 |
| 反例引导学习 | 任意 $n$ | 启发式 | Qin-Fan 2022 |

**开放挑战**：如何在高维空间（$n>10$）提供有意义的安全保证？当前最有前途的方向是**分层验证**：先用 Lipschitz 约束估计全局误差界，再对关键区域（边界附近）用精确方法验证。

### 8.4 CBF 在时变环境中的挑战 ⭐⭐⭐⭐

当障碍物移动或环境变化时，CBF 函数本身随时间变化：$h(x,t)$。时变 CBF 的条件变为：
$$\frac{\partial h}{\partial t} + L_fh + L_gh\cdot u\ge -\alpha(h)$$

额外项 $\partial h/\partial t$ 表示环境变化对安全裕度的影响。例如障碍物靠近时 $\partial h/\partial t<0$，使约束更紧。

**动态障碍物的处理**：

- **已知运动模型**：把障碍物运动建模为 $\dot p_o=v_o(t)$，$\partial h/\partial t=-2(p-p_o)^\top v_o$
- **预测不确定性**：障碍物未来位置不确定时，使用 reachable set 或 GP 预测，约束用最坏情况
- **感知延迟**：从感知到控制的 pipeline 延迟 $\tau$ 内障碍物可能移动 $\|v_o\|\tau$，需额外 margin

> **反事实推理**：如果忽略 $\partial h/\partial t$ 项（只用静态 CBF），对动态障碍物会怎样？当障碍物快速靠近时（$\partial h/\partial t\ll 0$），实际 $\dot h$ 比控制器估计的更负——系统认为安全但实际不安全。这是自动驾驶中 CBF 用于行人避障时的核心挑战。

---

## 本章小结

| 知识点 | 核心公式/条件 | 难度 | 关键直觉 |
|---|---|---|---|
| 正向不变集 | $x(0)\in\mathcal{C}\implies x(t)\in\mathcal{C}$ | ⭐ | 一旦进去就出不来 |
| Nagumo 定理 | $\dot h\ge 0$ 在 $\partial\mathcal{C}$ | ⭐⭐ | 边界处速度不指向外 |
| CLF | $\inf_u[L_fV+L_gVu]<0$ | ⭐⭐ | 存在控制使V下降 |
| Sontag 公式 | $k(x)=\frac{-a-\sqrt{a^2+b^4}}{b}$ | ⭐⭐ | CLF的显式连续反馈 |
| ZCBF | $\sup_u[L_fh+L_ghu]\ge-\alpha(h)$ | ⭐⭐ | 存在控制使h不快降 |
| CBF主定理 | 比较引理→$h(t)\ge 0$ | ⭐⭐ | 衰减率不超比较ODE |
| HOCBF | 递归$\psi_i$，约束在第$r$层 | ⭐⭐⭐ | 层层限速到控制层 |
| CLF-CBF-QP | 松弛CLF + 硬CBF | ⭐⭐ | 安全优先于稳定 |
| Safety Filter | $\min\|u-u_\text{nom}\|^2$ s.t. CBF | ⭐⭐ | 最小侵入修正 |
| MPC-CBF | DCBF作为MPC硬约束 | ⭐⭐⭐ | 短时域也能远避障 |
| ISSf-CBF | 加margin补偿扰动 | ⭐⭐⭐ | 安全集膨胀应对不确定 |
| Neural CBF | $h_\theta$ 网络学习 | ⭐⭐⭐⭐ | 高维系统自动生成CBF |

---

## 累积项目：安全控制模块

**项目定位**：在前面章节积累的系统模型基础上，本章新增"安全层"。

**本章新增模块**：

1. **CBF-QP Safety Filter**：给定任意名义控制器输出，做安全修正
2. **HOCBF 处理器**：对相对度 >1 的约束自动计算递归 $\psi_i$ 和约束
3. **多障碍物管理器**：动态添加/移除障碍物 CBF 约束

**项目进度**：
- 专题 3.7：Lyapunov 稳定性分析模块 → 本章复用 CLF
- **本章**：安全滤波器模块（CBF-QP + HOCBF）
- 专题 3.9（下一章）：MPC + CBF-QP 分层架构

---

## 延伸阅读

### 教材与综述

| 资源 | 类型 | 难度 | 推荐理由 |
|---|---|---|---|
| Ames et al. ECC 2019 (arXiv:1903.11199) | Tutorial | ⭐⭐ | CBF 最佳入门，40页覆盖全貌 |
| Khalil, *Nonlinear Systems* 3rd ed. Ch.4 | 教材 | ⭐⭐ | Lyapunov/比较引理的严格基础 |
| Blanchini-Miani, *Set-Theoretic Methods in Control* | 教材 | ⭐⭐⭐ | 正不变性理论权威 |
| Dawson-Gao-Fan T-RO 2023 (arXiv:2202.11762) | 综述 | ⭐⭐⭐ | Neural certificate 统一视角 |
| Brunke et al. *Annu. Rev.* 2022 | 综述 | ⭐⭐⭐ | Safe RL 全景 |

### 核心论文

| 论文 | 贡献 | 必读程度 |
|---|---|---|
| Ames-Xu-Grizzle-Tabuada 2017 TAC | ZCBF/RCBF统一定义，CLF-CBF-QP | 必读 |
| Xiao-Belta 2022 TAC | HOCBF 递归构造 | 必读（若涉及相对度>1） |
| Nguyen-Sreenath ACC 2016 | ECBF（线性HOCBF） | 推荐 |
| Choi-Lee-Sreenath-Tomlin-Herbert CDC 2021 | CBVF（统一CBF和HJ） | 进阶 |
| Zeng-Zhang-Sreenath ACC 2021 | MPC-CBF | 推荐 |
| Cheng et al. AAAI 2019 | CBF + RL Safety Filter | 推荐（若做RL） |
| Wang-Ames-Egerstedt T-RO 2017 | 多机器人CBF | 推荐（若做多智能体） |

### 代码资源

| 工具 | 语言 | URL | 用途 |
|---|---|---|---|
| neural_clbf | Python/PyTorch | https://github.com/MIT-REALM/neural_clbf | Neural CLF/CBF 学习 |
| cbfkit | Python/JAX/ROS2 | https://github.com/bardhh/cbfkit | CBF-QP + ROS2 |
| cbfpy | Python/JAX | https://github.com/danielpmorton/cbfpy | 轻量级 CBF QP |
| safe-control-gym | Python/PyBullet | https://github.com/utiasDSL/safe-control-gym | Benchmark |
| OSQP | C/Python/C++ | https://osqp.org/ | 工业级 QP 求解器 |
| hj_reachability | Python/JAX | https://github.com/StanfordASL/hj_reachability | HJ PDE 求解 |

### 中文资源

- 知乎 *控制障碍函数(CBF)*：https://zhuanlan.zhihu.com/p/703448704
- 知乎 *根据ACC理解CBF*：https://zhuanlan.zhihu.com/p/568328445
- 知乎 *基于CBF的MPC*：https://zhuanlan.zhihu.com/p/492934579
- CSDN *CBF详解（附案例）*：https://blog.csdn.net/qq_44940689/article/details/139178533

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|---|---|---|---|
| QP 返回 infeasible | CBF+输入约束不相容 | 1.打印 $K_{\text{cbf}}(x)$ 2.检查 $h(x)$ 是否已负 3.增大 $\alpha$ 的 $\gamma$ | 4.5 |
| 控制输出跳变/chattering | CBF 约束频繁激活/退激活 | 1.检查 $h(x)$ 时间序列 2.减小 $\gamma$ 3.加低通滤波 | 3.3 |
| 系统仍然撞击障碍物 | 相对度未正确处理（$L_gh=0$） | 1.计算 $L_gh$ 2.确认相对度 3.改用 HOCBF | 3.5 |
| 机器人卡在障碍物前不动 | 未期望平衡点（死锁） | 1.检查 CLF 和 CBF 梯度方向 2.加旋转场 3.用高层规划 | 8.1 |
| OSQP 求解很慢 (>1ms) | 问题 ill-conditioned | 1.检查 $P$ 条件数 2.调整 OSQP 参数 3.简化约束 | 5.3 |
| $\delta$ 一直很大（CLF 不起作用） | $p$ 太小 | 增大 $p$ 至 $10^3\sim10^4$ | 4.3 |
| Neural CBF 训练后验证失败 | 训练数据未覆盖全状态空间 | 1.增加采样点 2.用反例增广 3.检查网络容量 | 6.4 |
| 系统在边界附近振荡 | $\gamma$ 过大导致约束过于激进 | 1.减小 $\gamma$ 2.检查离散化步长 3.加 deadband | 3.3 |
| HOCBF 中间变量 $\psi_i<0$ | 初始条件不在 $\cap\mathcal{C}_i$ 中 | 1.验证初始状态满足所有 $\psi_i\ge 0$ 2.必要时预先使系统进入安全域 | 3.5 |
| 多机器人互相"推挤" | 成对 CBF 优先级未定义 | 1.引入 ID 优先级 2.检查约束方向一致性 3.加非对称项 | 7.4 |

**故障诊断决策树**：

```
CBF-QP 出问题？
├── 控制输出异常？
│   ├── 输出 NaN → 检查 h(x) 是否为零（除零错误）
│   ├── 输出饱和 → 输入约束太紧 or 系统已进入不可行区域
│   └── 输出跳变 → 约束激活/退激活切换，加平滑处理
├── 仍然碰撞？
│   ├── h(x) 始终>0 但仍碰 → CBF 定义有误（h 没有正确表示安全）
│   ├── h(x) 在碰撞前为负 → 初始就不安全，需上层规划预防
│   └── L_g h = 0 → 相对度 > 1，换 HOCBF
├── 性能太差（过度保守）？
│   ├── γ 太小 → 适当增大
│   ├── 安全集设计过大 → 缩紧 h 的定义
│   └── 多余约束激活 → 检查哪些 CBF 在当前状态不必要
└── QP 不可行？
    ├── 偶发 → 加 CBF 松弛（降级为软约束）+ 报警
    ├── 持续 → 系统已进入不可救区域，执行紧急停机
    └── 从未可行 → 检查约束方向、符号、量纲是否正确
```

**工程最佳实践清单**：

1. **always** 在 QP 外设置 fallback 策略（如零力矩、紧急制动）
2. **always** 记录 $h(x)$ 的时间序列用于事后分析
3. **always** 检查相对度再选择 CBF ���型
4. **never** 在未验证初始条件的情况下启用 CBF
5. **never** 信任 Neural CBF 的未验证区域
6. 调参顺序：先 $\gamma$（安全激进度）→ 再 $p$（CLF-CBF 平衡）→ 最后 $H$（控制方向权重）

---

## 核心定理速查

1. **Nagumo 1942**：$\mathcal{C}$ 前向不变 $\iff$ $\forall x\in\partial\mathcal{C},\;f(x)\in T_\mathcal{C}(x)$（水平集形式：$\dot h\ge 0$ 在边界上）。
2. **比较引理**：$\dot y\ge -\alpha(y),\;y(0)\ge 0 \implies y(t)\ge z(t)\ge 0$，其中 $\dot z=-\alpha(z)$，$z(0)=y(0)$。
3. **Artstein 1983**：CLF 存在 $\iff$ 系统渐近可镇定。
4. **Sontag 1989**：CLF 给出显式连续稳定反馈（Sontag 公式）。
5. **Ames 2017 Thm 2**：$h$ 为 ZCBF $\implies$ 任何 Lipschitz $u\in K_{\text{cbf}}$ 使 $\mathcal{C}$ 前向不变且渐近稳定。
6. **Xiao-Belta 2022**：HOCBF 递归 $\psi_i\ge 0$（$\forall i$）$\implies$ $h\ge 0$ 保持。
7. **CBVF（Choi 2021）**：$\{B_\gamma\ge 0\}$ = 最大鲁棒控制不变集。
8. **ISSf（Kolathaya-Ames 2019）**：ISSf-CBF $\implies$ 膨胀集 $\mathcal{C}_d$ 前向不变。

---

## 自测题（25 题）

1. 写出 Nagumo 定理的水平集形式并说明 CBF 如何推广它到 $\operatorname{Int}(\mathcal{C})$。
2. 给定 $\dot x=u$，$h(x)=1-x^2$，判定是否为 ZCBF 并求 $K_{\text{cbf}}(x)$。
3. ZCBF 与 RCBF 的转换关系？$B=1/h$ 成立条件？
4. 推导 CLF-CBF-QP 的 KKT 条件，解释松弛变量 $\delta$ 何时激活。
5. 为什么扩展类 $\mathcal{K}_\infty$ 函数必须允许负半轴？
6. 双积分器 $\ddot q=u$ 配 $h=q-q_{\min}$ 的相对度？直接套 CBF 为什么失败？
7. 设计 ECBF 极点 $p=2$，写出相对度 2 的完整约束表达式。
8. HOCBF 相对 ECBF 的核心优势？给一个 $\alpha_i$ 选成 $\sqrt{\cdot}$ 的场景。
9. 三个圆形障碍的 CBF 如何拼成 Composite？比较与每条单独约束的差异。
10. CBVF 中 $\gamma=0$ 和 $\gamma>0$ 的几何含义分别是什么？
11. ISSf-CBF 中 $\iota(\|d\|)$ 项如何影响实际可行集？
12. DCBF 约束 $h_{k+1}\ge (1-\alpha)h_k$ 为什么要求 $\alpha\in(0,1]$？
13. Neural CBF 训练损失应含哪三项？如何做 counter-example 采样？
14. 在 PPO + CBF safety layer 里，如何通过可微 QP 让策略感知安全约束？
15. 多机器人对称 CBF 为什么可能导致死锁？
16. MPC-CBF 与 MPC + distance constraint 在短 horizon 下的差异。
17. 列举 CLF-CBF-QP 引入未期望平衡点的一个二维反例。
18. 输入约束 $\|u\|\le u_{\max}$ 与 $K_{\text{cbf}}$ 相交为空的判定条件？
19. Safety Filter 闭式解的几何含义是什么（正交投影到哪里）？
20. Sontag 公式在 $L_gV\to 0$ 时如何保持连续？写出有理化后的形式。
21. 比较引理的证明中，唯一性定理为什么是关键？
22. HOCBF 中若 $\alpha_1$ 选非 Lipschitz（如 $\sqrt{r}$），对比较 ODE 的解有何影响？
23. CLF-CBF-QP 中 $p\to\infty$ 的极限行为是什么？
24. 四足 whole-body MPC + CBF 的控制层级如何划分？
25. 写出本章最核心的 3 个公式并说明它们如何将"安全"形式化。

---

## 桥梁（前后专题衔接）

**前承 3.7 Lyapunov 稳定性**：你已掌握 $V$、$\dot V\le -\gamma(V)$、比较引理、LaSalle；本专题核心是把 $V$ 换成 $h$、$\le$ 换成 $\ge$、"收敛到 0"换成"保持 $\ge 0$"，**工具集合几乎相同**。具体的对应关系：

| 3.7 中学到的 | 本章中的对应 | 转换规则 |
|---|---|---|
| $V>0$ 正定 | $h$ 正则 ($\nabla h\neq 0$) | 不完全对应，但都是"好"函数的条件 |
| $\dot V\le -\gamma V$ | $\dot h\ge -\alpha h$ | 不等式方向翻转 |
| $V(t)\to 0$ | $h(t)\ge 0$ 保持 | 衰减→不穿零 |
| 比较引理（上界） | 比较引理（下界） | 同一引理的两面 |
| LaSalle 不变性原理 | Nagumo 正不变性 | 描述渐近行为→描述集合不变 |
| CLF-QP（稳定） | CBF-QP（安全） | 同一 QP 框架 |

**后接**：
- **3.9 MPC 与轨迹优化**：自然承接 MPC-CBF 与 Predictive Safety Filter，把一步 CBF 扩为时域 QP。MPC-CBF 的核心思想是在每个预测步加入 DCBF 约束——你将看到本章的理论如何直接转化为 MPC 的约束设计。
- **3.10 最优控制 / HJ / Pontryagin**：CBVF 把本专题内嵌入 HJI-VI，深化 viscosity solution 与 value function 理解。HJ 方法给出"最优安全集"——本章的 CBF 是其近似/简化版本。
- **第四批 RL 理论**：CBF 作为 safety layer 是 safe RL 的主流范式，本专题是先决条件。你将学到如何把 CBF-QP 包装为可微层嵌入 RL 训练循环。
- **具身智能方向**：感知-based CBF（视觉/点云→$h_\theta$）是将本章理论部署到真实机器人的桥梁。

**知识依赖图**：
```
Lyapunov (3.7) ──→ CLF ──→ CLF-CBF-QP ──→ MPC-CBF (3.9)
                          ↗                       ↘
集合论/Nagumo ──→ CBF ─┘                  Safe RL (4.x)
                    ↓
              HOCBF/ECBF ──→ 四足/四旋翼应用
                    ↓
              Neural CBF ──→ 具身智能
```

---

## 常见陷阱汇总

1. **误以为 $\dot h\ge 0$ 就是 CBF**：那是 Nagumo 条件（仅在 $\partial\mathcal{C}$ 上），CBF 要求在整个 $\mathcal{D}$ 上 $\dot h\ge -\alpha(h)$。
2. **相对度未检查直接写 $L_gh=0$ 的约束**：QP 恒成立等于无约束，系统照样撞。
3. **$\alpha$ 选太大**导致 CBF 约束在远离边界处已激活，控制被过度侵入。**选太小**则边界响应过慢。
4. **松弛罚 $p$ 太小**使 CLF 不起作用；**$p$ 太大**则冲突时不可行。
5. **忽视输入约束 $\mathcal{U}$**：未加 $u_{\min}\le u\le u_{\max}$ 时 QP 可行但物理不可行。
6. **Lipschitz 失败处**：CBF 约束切换瞬间解可能跳变，引起 chattering。
7. **离散化误差**：连续 CBF 在离散实现中只近似成立；正确做法是用 DCBF 或加 margin。
8. **Neural CBF 没做覆盖验证**：训练集未覆盖的 $x$ 可能违反 CBF。
9. **死锁**：两个机器人相向，对称 CBF 给出对称修正→都停下。需加优先级或高层规划。
10. **把 RL 值函数当 CLF**：RL 的 $Q/V$ 通常不满足解析 Lyapunov 条件。

---

## 学术前沿快照（2024-2026）

本节梳理 CBF 领域最近两年的重要进展，供博士生选题参考：

| 方向 | 代表工作 | 核心贡献 | 发表 |
|---|---|---|---|
| 安全强���学习 | Brunke et al. 2022 | safe-control-gym benchmark | Annu. Rev. |
| 可组合安全 | Molnar-Ames 2023 | Composite CBF | L-CSS |
| 图神经 CBF | Zhang-So-Garg-Fan 2024 | GCBF+: $10^3$ 级多机器人 | ICRA |
| CBF + 大模型 | Xiao et al. 2024 | LLM 生成 CBF 候选 | RSS workshop |
| 在线自适应 CBF | Lopez-Ames 2024 | 数据���动 $\alpha$ 在线调整 | TAC |
| 接触安全 | Khazoom et al. 2024 | 腿足接触力 CBF | IROS |

**博士论文方向建议**：

1. **感知-安全协同设计**：将视觉/点云直接映射为 CBF（绕过显式建图）。关键挑战是感知不确定性的传播。
2. **安全强化学习的样本效率**：CBF Safety Filter 限制了探索空间，如何在安全前提下高效学习？
3. **多智能体安全涌现行为**：超越成对安全，研究群体层面的涌现安全/死锁特性。
4. **形式化验证的可扩展性**：发展可用于 $n>10$ 的验证方法（可能结合 abstraction-refinement）。

---

## 时间预算

| 内容 | 建议时长 |
|---|---|
| 0-1.2 直觉、Nagumo、ZCBF/RCBF | 6 小时精读 Ames 2017 + ECC 2019 |
| 2 CLF + Sontag 公式 | 4 小时 |
| 3 CBF 定义与主定理证明 | 6 小时 |
| 3.5-3.6 HOCBF/ECBF | 6 小时（含双积分器实验） |
| 4 CLF-CBF-QP 推导与 OSQP 实现 | 8 小时（含代码） |
| 5 多约束 + 工程实现 | 5 小时 |
| 6 高级主题选读 | 10 小时（选 2-3 个方向深入） |
| 7 应用实例复现 | 8 小时 |
| 自测 + 项目 | 10 小时 |
| **总计** | **约 60-70 小时（6-7 周，每周 10 小时）** |

**学习建议路径**：

- **速成路线**（2 周，面试/课题组报告）：0节 + 1.2 + 3.1-3.2 + 4.2 + 5.3 代码实现
- **完整路线**（7 周，全面掌握）：按章节顺序全部覆盖
- **研究路线**（在完整路线后）：选 6.3-6.7 中与你博士方向相关的深入 + 复现一篇论文

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| CBF 保证系统不会进入危险状态 | CBF 保证的是安全集的前向不变性（集合层面），前提是 CBF 构造正确且相对度匹配；错误的 CBF 设计可能导致安全集为空或过小 |
| CLF 和 CBF 的 QP 约束总有可行解 | 当 CLF 收敛要求与 CBF 安全约束冲突时 QP 不可行；实践中需引入松弛变量 $\delta$ 并赋予大权重，以安全优先牺牲收敛性 |
| CBF 的类 $\mathcal{K}$ 函数 $\alpha$ 越大越好 | $\alpha$ 过大要求系统在边界附近急剧调整，可能导致执行器饱和或控制输入不连续；$\alpha$ 需要与系统能力匹配 |
| 高阶 CBF（HOCBF）只是简单地对 $h$ 多求几次导 | HOCBF 需要精心设计中间函数序列，每层的类 $\mathcal{K}$ 函数选择会影响安全集大小；不当设计可能使有效安全集远小于原始约束集 |
| CBF-QP 可以完全替代 MPC 做轨迹规划 | CBF-QP 是逐时刻的贪心安全滤波器（无预测能力），可能导致系统陷入死锁；MPC 具有前瞻能力，两者互补而非替代 |
| 找到一个满足 CBF 条件的 $h(x)$ 很容易 | CBF 的构造是该领域最困难的开放问题之一——需要 $h(x)$ 同时满足安全集非空、相对度正确、且存在使 $\dot{h} + \alpha(h) \ge 0$ 的可行控制 |

---

## 总结

### 核心洞察

CBF 是 Lyapunov 方法在"安全"这一维度上的完美对偶——把"$V$ 下降到 0"翻译为"$h$ 不穿过 0"，把 LaSalle 翻译为 Nagumo，把稳定性翻译为前向不变性。这种对偶不是巧合，而是反映了一个更深的数学事实：**控制理论中"可达性"与"安全性"是同一 HJ 方程在不同参数（$\gamma=0$ vs $\gamma>0$）下的两个特例**（Choi-Lee-Herbert-Tomlin 2021 的 CBVF 统一）。

### 技术栈定位

CBF-QP 在机器人控制频谱上处于 **执行器层（1 kHz）与规划层（10 Hz MPC）之间的 100-500 Hz 安全层**，不替代任何一层，而是作为"硬约束守门员"：

```
[任务规划 ~1Hz] → [路径规划 ~10Hz] → [MPC ~50Hz] → [CBF Safety Filter ~500Hz] → [驱动 ~1kHz]
                                                            ↑
                                                    [状态估计 + 感知]
```

### 三大公式总结

整个 CLF-CBF 理论可以浓缩为三个核心公式：

**公式 1 — CLF 条件**（稳定性）：
$$\inf_{u\in\mathcal{U}}\left[L_fV(x)+L_gV(x)\,u\right] \le -\gamma V(x)$$
"至少有一个控制使 Lyapunov 函数以 $\gamma V$ 的速率下降。"

**公式 2 — CBF 条件**（安全性）：
$$\sup_{u\in\mathcal{U}}\left[L_fh(x)+L_gh(x)\,u\right] \ge -\alpha(h(x))$$
"至少有一个控制使安全裕度的衰减速率不超过 $\alpha(h)$。"

**公式 3 — CLF-CBF-QP**（统一实现）：
$$u^*=\arg\min_{u,\delta}\|u-u_{\text{ref}}\|^2+p\delta^2 \quad\text{s.t. CLF (soft) + CBF (hard)}$$
"在满足安全硬约束的前提下，尽可能跟踪名义控制。"

### 未完成的战线

| 方向 | 当前状态 | 2025-2030 预期进展 |
|---|---|---|
| 高维验证 | SMT 限于 $n\le 6$ | 可扩展的 Neural Verification |
| 感知-based CBF | 需要精确模型 | 端到端感知→安全 |
| 时变/动态环境 | 需要运动预测 | 在线自适应 CBF |
| 多智能体涌现行为 | 成对安全但全局死锁 | 分层安全协调 |
| RL 训练效率 | Safety Filter 减慢探索 | 安全感知的高效探索 |

### 从本章到实际部署

将本章理论部署到真实机器人需要以下工程步骤：

1. **系统建模**：确定 $f(x)$、$g(x)$ 的精度，评估模型不确定性上界 $d_{\max}$
2. **安全集设计**：根据任务需求定义 $h(x)$，验证相对度，确定需要 CBF 还是 HOCBF
3. **参数选择**：根据系统时间常数和制动能力选 $\gamma$，验证可行性
4. **离线验证**：在仿真中测试极端场景（高速接近、多障碍、执行器饱和、传感器丢失）
5. **在线部署**：先低速测试，逐步提速，始终保留人工接管（emergency stop）
6. **持续监控**：实时记录 $h(x)$、约束激活率、QP 求解时间、松弛变量 $\delta$ 统计

### 一句话总结

Lyapunov 教你稳，CBF 教你活；会了这两把钥匙，你就拥有了打开"可证明安全机器人学习"这扇大门的全部权限。下一专题 3.9，我们将进入 MPC 与轨迹优化——把 CBF 的一步展望扩展为有限时域的最优规划。
