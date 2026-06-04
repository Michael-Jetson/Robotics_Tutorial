# 共轭函数与 Proximal 算子

## 前置自测

> 📋 **前置自测**（答不出 ≥ 2 题 → 先回凸分析基础复习）
>
> 1. 什么是凸函数的次梯度？$0 \in \partial f(x^*)$ 意味着什么？
> 2. 凸函数在 ri(dom f) 上一定连续吗？闭凸函数需要满足什么条件？
> 3. 什么是支撑函数 $\sigma_C(y) = \sup_{x \in C}\langle y, x \rangle$？它与指示函数有什么关系？
> 4. $\mu$-强凸的一阶条件是什么？它与 $L$-光滑有什么"对偶"关系？
> 5. 写出 $\|x\|_1$ 在 $x = 0$ 处的次微分。

## 本章目标

学完本章后，你将能够：(1) 对常见凸函数独立推导其共轭函数的闭式解，理解共轭的几何意义（切线包络 / 截距的 sup）；(2) 证明 Fenchel-Moreau 定理（$f^{**} = f$ 对闭凸函数成立）并理解其意义；(3) 计算5种核心 proximal 算子（软阈值、块软阈值、投影、二次、SVT）；(4) 证明 Moreau 分解定理并理解其在 ADMM 中的角色；(5) 理解 proximal 算子作为 firmly nonexpansive 映射的性质及其对算法收敛的意义。

---

## 2.1 共轭函数的定义与几何直觉 ⭐⭐

### 动机

在凸优化中，我们经常需要把一个问题"翻转"到另一个等价视角——这就是对偶。**共轭函数（Legendre-Fenchel 变换）是实现这种翻转的核心数学工具。** 它把函数空间中的 $f$ 映射到"对偶函数空间"中的 $f^*$，使得很多困难问题（如非光滑优化）在对偶空间变得简单。

对机器人工程师而言，共轭函数不是抽象数学玩具：
- OSQP/ADMM 求解器底层的每一步迭代都涉及共轭运算
- Robust SLAM 的 GNC（Graduated Non-Convexity）本质是 Moreau 光滑化，其基础是共轭
- RL 中的 mirror descent / natural gradient 建立在 Fenchel 对偶视角上

### 如果不学共轭函数会怎样

当你试图理解 ADMM 为什么能把一个复杂问题拆成简单子问题时，如果不懂共轭函数，你会发现：
- 对偶变量的更新步骤似乎"凭空出现"
- proximal 算子的定义看起来像"魔法"
- 强对偶何时成立、对偶间隙为什么为零，都无法从第一性原理理解

共轭函数就是把这些"魔法"还原为"数学"的工具。

### 历史背景

共轭函数的概念可以追溯到 Adrien-Marie Legendre (1752-1833) 的经典 Legendre 变换——它在热力学（从能量到自由能的转换）和哈密顿力学（从 Lagrangian 到 Hamiltonian）中有核心应用。Legendre 变换要求函数严格凸且光滑，Werner Fenchel (1905-1988) 在 1949 年把它推广到一般凸函数，允许非光滑和扩展实值函数——这就是 Legendre-Fenchel 共轭。

**从物理到数学**：在热力学中，内能 $U(S,V)$ 是熵 $S$ 和体积 $V$ 的函数。Legendre 变换把 $U$ 转成 Helmholtz 自由能 $F(T,V) = U - TS$——从"自然变量 $(S,V)$"转为"可控变量 $(T,V)$"。在凸分析中，同样的数学操作把函数从"点值表示"转为"斜率表示"。

### 定义

**定义 2.1.1**（Legendre-Fenchel 共轭）：设 $f: \mathbb{R}^n \to \mathbb{R} \cup \{+\infty\}$。$f$ 的共轭函数 $f^*: \mathbb{R}^n \to \mathbb{R} \cup \{+\infty\}$ 定义为：

$$f^*(y) = \sup_{x \in \text{dom}(f)} \left[ \langle y, x \rangle - f(x) \right]$$

**为什么取 sup**：$\langle y, x \rangle - f(x)$ 可以理解为"用斜率为 $y$ 的仿射函数 $\langle y, x \rangle$ 逼近 $f(x)$时的最大间隙"。对每个斜率 $y$，共轭 $f^*(y)$ 告诉我们这个间隙有多大——等价地，斜率为 $y$ 的支撑超平面能"推到"多高。

### 经典 Legendre 变换 vs Legendre-Fenchel 共轭

在 Fenchel 推广之前，Legendre 变换只适用于严格凸可微函数。两者的区别值得仔细理解：

| 特征 | 经典 Legendre 变换 | Legendre-Fenchel 共轭 |
|------|-------------------|---------------------|
| 适用范围 | 严格凸、可微 | 任意函数（含非凸、非光滑） |
| 定义 | $f^*(y) = y \cdot x(y) - f(x(y))$，$y = f'(x)$ | $f^*(y) = \sup_x[\langle y,x\rangle - f(x)]$ |
| 求解方式 | 解方程 $y = \nabla f(x)$ | 取上确界 |
| 可逆性 | 总可逆（严格凸保证） | 仅对闭凸函数可逆 ($f^{**}=f$) |
| 典型应用 | 哈密顿力学、热力学 | 凸优化、对偶理论 |

**从 Lagrangian 到 Hamiltonian**（物理学视角）：

在分析力学中，Lagrangian $L(q, \dot{q})$ 是广义坐标 $q$ 和广义速度 $\dot{q}$ 的函数。Legendre 变换把速度变量 $\dot{q}$ 转换为动量变量 $p = \partial L/\partial \dot{q}$：

$$H(q, p) = p \cdot \dot{q}(p) - L(q, \dot{q}(p))$$

这正是 $L$ 关于 $\dot{q}$ 的 Legendre 变换。$H$ 是 Hamiltonian——它把 Lagrange 方程（二阶 ODE）转化为 Hamilton 方程（一阶 ODE 组），这在数值积分（辛积分器）和控制理论（最优控制的 Pontryagin 最大原理）中至关重要。

> **本质洞察**：Legendre 变换的物理意义是"变量替换"——从一组自然变量转到另一组更方便的变量。在热力学中，从 $(S, V)$（熵、体积）转到 $(T, P)$（温度、压力）。在凸优化中，从 $x$（原始变量）转到 $y$（对偶变量）。共轭函数就是实现这种转换的数学工具。

### 几何直觉：截距的上确界 ⭐⭐

$f^*(y)$ 的几何意义是什么？考虑斜率为 $y$ 的仿射函数 $\ell(x) = \langle y, x \rangle - b$。我们要找最大的 $b$ 使得 $\ell(x) \leq f(x)$ 对所有 $x$ 成立（即 $\ell$ 是 $f$ 的下方支撑线）。

$\ell(x) \leq f(x) \Leftrightarrow b \leq f(x) - \langle y, x \rangle \Leftrightarrow b \leq \inf_x [f(x) - \langle y, x \rangle] = -f^*(y)$

所以 $f^*(y) = -\inf_x[f(x) - \langle y, x \rangle] = \sup_x[\langle y, x \rangle - f(x)]$，即 $f^*(y)$ 是所有斜率为 $y$ 的支撑仿射函数中，**截距的负数的最大值**。

**切线包络解释**：$f^*$ 把 $f$ 的所有切线（支撑超平面）的信息编码成一个函数。给定斜率 $y$，$f^*(y)$ 告诉你在这个斜率下，支撑线能"推到"多高。整个 $f$ 可以从其所有切线恢复——这就是 $f^{**} = f$ 的几何本质。

> **本质洞察**：共轭变换把函数从"点值表示"（给定 $x$，$f(x)$ 是多少）转换为"切线表示"（给定斜率 $y$，最紧的下方支撑线截距是多少）。这两种表示对闭凸函数是等价的——这就是 Fenchel-Moreau 定理。

**跨领域类比**：共轭变换之于凸分析，就像 Fourier 变换之于信号处理。Fourier 变换把信号从"时域"（时间点的值）转到"频域"（频率成分的幅度），共轭变换把函数从"原始空间"（点值）转到"对偶空间"（斜率-截距）。两者都是可逆的变换（对"好的"函数），且在各自的对偶空间中，某些运算变得简单。

### Young-Fenchel 不等式 ⭐⭐

**定理 2.1.2**：对任何函数 $f$ 和任何 $x, y$：

$$f(x) + f^*(y) \geq \langle x, y \rangle$$

等号成立当且仅当 $y \in \partial f(x)$。

**证明**：由定义 $f^*(y) = \sup_z [\langle y, z \rangle - f(z)] \geq \langle y, x \rangle - f(x)$，移项即得。

等号条件：$f^*(y) = \langle y, x \rangle - f(x)$ 意味着 $x$ 是 $\sup_z[\langle y, z \rangle - f(z)]$ 的取到者，即 $f(z) \geq f(x) + \langle y, z - x \rangle$ 对所有 $z$ 成立，这正是 $y \in \partial f(x)$。

**为什么这个不等式重要**：它是所有对偶理论的出发点。弱对偶不等式、Lagrange 对偶下界——都可以从 Fenchel-Young 不等式推出。

### 典型函数的共轭推导 ⭐⭐

**例 1：二次函数** $f(x) = \frac{1}{2}x^\top Q x$（$Q \succ 0$）

$$f^*(y) = \sup_x \left[ y^\top x - \frac{1}{2}x^\top Q x \right]$$

对 $x$ 求导令为零：$y - Qx = 0 \Rightarrow x = Q^{-1}y$

代回：$f^*(y) = y^\top Q^{-1}y - \frac{1}{2}(Q^{-1}y)^\top Q (Q^{-1}y) = \frac{1}{2}y^\top Q^{-1} y$

**结论**：$(\frac{1}{2}x^\top Q x)^* = \frac{1}{2}y^\top Q^{-1} y$。特别地，$(\frac{1}{2}\|x\|^2)^* = \frac{1}{2}\|y\|^2$——平方欧几里得范数是自共轭的。

**例 2：范数** $f(x) = \|x\|_p$（$p \geq 1$）

$$f^*(y) = \sup_x [\langle y, x \rangle - \|x\|_p]$$

由 Holder 不等式：$\langle y, x \rangle \leq \|y\|_q \|x\|_p$（$\frac{1}{p} + \frac{1}{q} = 1$）

因此 $\langle y, x \rangle - \|x\|_p \leq (\|y\|_q - 1)\|x\|_p$

若 $\|y\|_q > 1$：取 $\|x\|_p \to \infty$，$\sup = +\infty$
若 $\|y\|_q \leq 1$：上界为 $0$，且在 $x = 0$ 时取到 $0$
若 $\|y\|_q = 1$：由 Holder 等号条件，$\sup = 0$

**结论**：$\|x\|_p$ 的共轭是对偶范数单位球的指示函数 $\delta_{\{\|y\|_q \leq 1\}}$。

特别地：$\|\cdot\|_1^* = \delta_{\|\cdot\|_\infty \leq 1}$，$\|\cdot\|_2^* = \delta_{\|\cdot\|_2 \leq 1}$。

**例 3：指示函数** $f(x) = \delta_C(x)$

$$f^*(y) = \sup_{x \in C} \langle y, x \rangle = \sigma_C(y)$$

指示函数的共轭就是支撑函数。这连接了凸分析基础章节的两个概念。

**例 4：负对数** $f(x) = -\log x$（$x > 0$）

$$f^*(y) = \sup_{x > 0} [yx + \log x]$$

对 $x$ 求导：$y + 1/x = 0 \Rightarrow x = -1/y$（需要 $y < 0$）

代回：$f^*(y) = y \cdot (-1/y) + \log(-1/y) = -1 - \log(-y)$

**结论**：$(-\log x)^* = -1 - \log(-y)$，$\text{dom}(f^*) = \{y < 0\}$。

**例 5：log-sum-exp** $f(x) = \log(\sum_i e^{x_i})$

其共轭是概率单纯形上的负熵：$f^*(y) = \sum_i y_i \log y_i$（当 $y \in \Delta_n$），否则 $+\infty$。

**推导**：需要求 $\sup_x [\langle y, x \rangle - \log\sum_i e^{x_i}]$。

对 $x_j$ 求导：$y_j - \frac{e^{x_j}}{\sum_i e^{x_i}} = 0$，即 $y_j = p_j$（softmax 输出）。

这要求 $y_j > 0$ 且 $\sum_j y_j = 1$（$y \in \Delta_n$）。

代回：$\langle y, x \rangle - \log\sum e^{x_i} = \sum y_j x_j - \log\sum e^{x_j}$。

由 $y_j = e^{x_j}/\sum e^{x_k}$，得 $x_j = \log y_j + \log\sum e^{x_k}$。

$\sum y_j x_j = \sum y_j \log y_j + \log\sum e^{x_k}$。

因此 $f^*(y) = \sum y_j \log y_j$（负熵）。

这个结果连接了凸优化和信息论——soft-max 的共轭是负熵。在 RL 中，soft-value iteration 的 Bellman 方程 $V(s) = \log\sum_a \exp(Q(s,a))$ 正是 log-sum-exp，其共轭自然引出策略的熵正则化。

**例 6：Huber 函数** $\rho_\delta(x) = \begin{cases} x^2/2 & |x| \leq \delta \\ \delta|x| - \delta^2/2 & |x| > \delta \end{cases}$

**推导**：

当 $|y| \leq \delta$ 时：$\sup_x[yx - \rho_\delta(x)]$ 的极值在光滑区域取到（$|x| \leq \delta$），求导 $y = x$，代回 $f^*(y) = y^2/2$。

当 $|y| > \delta$ 时：在线性区域 $\rho_\delta(x) = \delta|x| - \delta^2/2$，$\sup_x[yx - \delta|x| + \delta^2/2]$。若 $y > \delta$，取 $x \to +\infty$ 得 $\sup = +\infty$。

**结论**：$\rho_\delta^*(y) = \begin{cases} y^2/2 & |y| \leq \delta \\ +\infty & |y| > \delta \end{cases} = \frac{y^2}{2} + \delta_{[-\delta,\delta]}(y)$

**意义**：Huber 函数是 $\frac{x^2}{2}$ 和 $\delta\|x\|$ 的 infimal 卷积，其共轭是 $\frac{y^2}{2}$（二次的共轭）和 $\delta_{[-\delta,\delta]}$（范数共轭的缩放）之和——符合 infimal 卷积的共轭 = 和的规则。

**例 7：$\ell_1$ 范数的平方** $f(x) = \frac{1}{2}\|x\|_1^2$

$$f^*(y) = \frac{1}{2}\|y\|_\infty^2$$

**证明思路**：$\|x\|_1^2/2$ 可以写成 $\|x\|_1$ 的 Moreau 包络的某种变换。更直接地，利用 $\|x\|_1 = \max_{\|z\|_\infty \leq 1} z^\top x$，代入并交换 sup 和 min。

**例 8：矩阵函数** $f(X) = -\log\det X$（$X \succ 0$）

$$f^*(Y) = -\log\det(-Y) - n \quad (Y \prec 0)$$

**推导**：$\sup_{X \succ 0} [\text{tr}(YX) + \log\det X]$。对 $X$ 求导：$Y + X^{-1} = 0$，即 $X = -Y^{-1}$（需要 $Y \prec 0$）。

代回：$\text{tr}(Y \cdot (-Y^{-1})) + \log\det(-Y^{-1}) = -n - \log\det(-Y)$。

这个结果在最大似然估计、实验设计和控制综合中经常出现。

### 共轭运算的基本性质

| 性质 | 陈述 | 应用 |
|------|------|------|
| 永远凸 | $f^*$ 总是凸的（即使 $f$ 不凸） | 对偶问题永远凸 |
| 永远闭 | $f^*$ 总是闭的（lsc） | 对偶问题结构好 |
| 仿射共轭 | $(f(Ax+b))^*(y) = f^*(A^{-\top}y) - b^\top A^{-\top}y$ | 变量替换 |
| 缩放 | $(\lambda f)^*(y) = \lambda f^*(y/\lambda)$（$\lambda > 0$） | 正则化参数变换 |
| 平移 | $(f(\cdot) + \langle a, \cdot\rangle + b)^*(y) = f^*(y-a) - b$ | 仿射项吸收 |
| 可分 | $(f_1 \oplus f_2)^*(y_1, y_2) = f_1^*(y_1) + f_2^*(y_2)$ | ADMM 分解 |
| Infimal 卷积 | $(f_1 \square f_2)^* = f_1^* + f_2^*$ | Moreau 包络 |

**性质：共轭永远凸的证明**：$f^*(y) = \sup_x[\langle y, x \rangle - f(x)]$。对每个固定 $x$，$\langle y, x \rangle - f(x)$ 是关于 $y$ 的仿射函数（因此凸）。逐点 sup 保凸，故 $f^*$ 凸。

这个性质极其重要：即使原函数 $f$ 是非凸的，其共轭 $f^*$ 仍然是凸的。但此时 $f^{**} \neq f$——$f^{**}$ 只是 $f$ 的闭凸包（最大的不超过 $f$ 的闭凸函数）。

**Infimal 卷积的详细解释** ⭐⭐⭐：

**定义**：两个函数的 infimal 卷积（inf-convolution）为：
$$(f_1 \square f_2)(x) = \inf_{x_1 + x_2 = x} [f_1(x_1) + f_2(x_2)] = \inf_z [f_1(z) + f_2(x-z)]$$

**关键性质**：$(f_1 \square f_2)^* = f_1^* + f_2^*$。

**证明**：
$$(f_1 \square f_2)^*(y) = \sup_x \left[\langle y, x \rangle - \inf_z [f_1(z) + f_2(x-z)]\right]$$
$$= \sup_x \sup_z \left[\langle y, x \rangle - f_1(z) - f_2(x-z)\right]$$
$$= \sup_z \left[\langle y, z \rangle - f_1(z)\right] + \sup_w \left[\langle y, w \rangle - f_2(w)\right] \quad \text{（令 } w = x-z \text{）}$$
$$= f_1^*(y) + f_2^*(y)$$

**为什么 infimal 卷积重要**：Moreau 包络恰好是一个 infimal 卷积！
$$M_f^\lambda(v) = \inf_x \left[f(x) + \frac{1}{2\lambda}\|x-v\|^2\right] = (f \square \frac{1}{2\lambda}\|\cdot\|^2)(v)$$

因此 $(M_f^\lambda)^* = f^* + \frac{\lambda}{2}\|\cdot\|^2$——Moreau 包络的共轭就是原函数的共轭加上一个二次项！这解释了为什么 Moreau 包络总是光滑的——加了二次项使共轭变成强凸的，由强凸-光滑对偶，原函数就变成光滑的。

### 共轭计算的系统方法 ⭐⭐

计算共轭函数有几种标准策略：

**方法 1：直接求导**（最常用）。设 $f$ 可微，在 $f^*(y) = \sup_x[\langle y, x \rangle - f(x)]$ 中对 $x$ 求导令为零：
$$y = \nabla f(x^*) \quad \Rightarrow \quad x^* = (\nabla f)^{-1}(y)$$

代回：$f^*(y) = \langle y, (\nabla f)^{-1}(y) \rangle - f((\nabla f)^{-1}(y))$。

**方法 2：Fenchel-Young 等号条件**。若知道 $y \in \partial f(x)$，则 $f^*(y) = \langle x, y \rangle - f(x)$。

**方法 3：利用共轭运算规则**。把函数分解为简单函数的组合，利用缩放、平移、可分性等规则。

**方法 4：对偶范数关系**。范数 $\|\cdot\|$ 的共轭是对偶范数单位球的指示函数。

### ⚠️ 常见陷阱

> 💡 **概念误区：认为"共轭的共轭一定回到原函数"**
> - $f^{**} = f$ 仅当 $f$ 是 proper + closed + convex 时成立
> - 非凸 $f$ 的 $f^{**}$ 只是 $f$ 的闭凸包（最大的不超过 $f$ 的闭凸函数）
> - 这个条件在实际中通常满足，但自定义函数需要检查

> 🧠 **思维陷阱：混淆"共轭"和"proximal"**
> - 共轭 $f^*(y) = \sup_x[\langle y,x \rangle - f(x)]$ 是一个函数变换
> - Proximal $\text{prox}_f(v) = \arg\min_x[f(x) + \frac{1}{2}\|x-v\|^2]$ 是一个点映射
> - 二者有深刻联系（通过 Moreau 分解），但不是同一个东西
> - 常见错误：把 $x\log x$ 的共轭 $e^{y-1}$ 当成其 proximal（实际需要 Lambert W 函数）

### 练习

1. 推导 Huber 函数 $\rho_\delta(x)$ 的共轭。（答案：$\frac{1}{2}y^2 + \delta_{[-\delta,\delta]}(y)$）
2. 证明仿射函数 $f(x) = a^\top x + b$ 的共轭是 $\delta_{\{a\}}(y) - b$。
3. 推导 hinge loss $f(x) = \max(0, 1-x)$ 的共轭，并给出 $\text{dom}(f^*)$。

---

## 2.2 Fenchel-Moreau 定理与双共轭 ⭐⭐

### 动机

共轭变换是可逆的吗？对哪些函数 $f^{**} = f$？这个问题的答案决定了对偶理论何时能"完美还原"原问题。

### 定理

**定理 2.2.1**（Fenchel-Moreau 定理）：对 proper 函数 $f$：

$$f^{**} = f \quad \Longleftrightarrow \quad f \text{ 是闭凸函数（proper + lsc + convex）}$$

若 $f$ 不是闭凸的，$f^{**}$ 等于 $f$ 的**闭凸包**——最大的不超过 $f$ 的闭凸函数。

### 证明（$\Leftarrow$ 方向的核心思想）

需要证明：若 $f$ 是闭凸的，则 $f^{**}(x) = f(x)$ 对所有 $x$。

**Step 1**：首先 $f^{**}(x) \leq f(x)$ 总成立（由 Fenchel-Young 不等式，对所有 $y$：$f^{**}(x) = \sup_y[\langle x, y \rangle - f^*(y)] \leq f(x)$，最后一步是对 $y$ 取 $y \in \partial f(x)$ 时等号成立）。

实际上更精确：$f^{**}(x) \leq f(x)$ 是因为 $f^{**}$ 是所有不超过 $f$ 的仿射函数的逐点上确界，而 $f$ 自身也是这些仿射函数的上界。

**Step 2**：证明 $f^{**}(x) \geq f(x)$。若 $f^{**}(x_0) < f(x_0)$，则点 $(x_0, f^{**}(x_0))$ 在 $\text{epi}(f)$ 的严格下方。由分离超平面定理（$\text{epi}(f)$ 是闭凸集），存在仿射函数 $\ell(x) = a^\top x + b$ 使得 $\ell(x_0) > f^{**}(x_0)$ 且 $\ell(x) \leq f(x)$ 对所有 $x$。

但 $\ell(x) \leq f(x)$ 意味着 $\ell$ 也不超过 $f^{**}$（因为 $f^{**}$ 是所有 $\leq f$ 的仿射函数的 sup），所以 $\ell(x_0) \leq f^{**}(x_0)$，矛盾。

### 意义

Fenchel-Moreau 定理说：**闭凸函数的世界在共轭变换下是封闭的**——做两次共轭回到原函数。这使得"在原空间做优化"和"在对偶空间做优化"完全等价。

### 共轭-次微分互逆定理 ⭐⭐

**定理 2.2.2**：设 $f$ 是闭凸函数。以下三个条件等价：

$$y \in \partial f(x) \quad \Longleftrightarrow \quad x \in \partial f^*(y) \quad \Longleftrightarrow \quad f(x) + f^*(y) = \langle x, y \rangle$$

**推论**：若 $f$ 可微且严格凸，则 $\nabla f^* = (\nabla f)^{-1}$——共轭的梯度是原函数梯度的逆映射。

**推论的完整证明**：

设 $f$ 可微且严格凸。由共轭-次微分互逆：$y = \nabla f(x) \Leftrightarrow x = \nabla f^*(y)$。

第一个等价来自：$y \in \partial f(x)$，由 $f$ 可微，$\partial f(x) = \{\nabla f(x)\}$，故 $y = \nabla f(x)$。

由等价条件：$x \in \partial f^*(y)$。由 $f$ 严格凸，$\nabla f$ 是单射，故 $f^*$ 也可微（$\partial f^*(y)$ 是单点集），$\nabla f^*(y) = x$。

因此 $\nabla f^*(y) = x = (\nabla f)^{-1}(y)$。

**应用实例**：对 $f(x) = \frac{1}{p}\|x\|^p$（$p > 1$），$\nabla f(x) = \|x\|^{p-2}x$。

其共轭 $f^*(y) = \frac{1}{q}\|y\|^q$（$\frac{1}{p}+\frac{1}{q}=1$），$\nabla f^*(y) = \|y\|^{q-2}y = (\nabla f)^{-1}(y)$。

**机器人应用**：这个互逆关系是 mirror descent（镜像下降）和自然梯度法的数学基础。在信息几何中，指数族分布的自然参数和均值参数之间的转换就是共轭梯度的逆映射。

### Fenchel-Moreau 的深层意义 ⭐⭐

**闭凸函数的仿射表示**：$f^{**} = f$ 意味着闭凸函数可以写成其所有仿射下界的逐点上确界：
$$f(x) = \sup_{y}[\langle y, x \rangle - f^*(y)]$$

**几何解释**：每个 $y$ 对应一条支撑超平面 $\ell_y(x) = \langle y, x \rangle - f^*(y)$。所有这些超平面从下方"包围"函数 $f$，它们的上包络恰好等于 $f$。这是"切线包络"的精确数学表述。

**反事实推理**：如果 $f$ 不闭凸，$f^{**}$ 只恢复 $f$ 的"最好的闭凸逼近"——即把 $f$ 的非凸部分和不连续部分"填平"后的版本。例如：
- 非凸函数的 $f^{**}$ 是其凸包络（把凹的部分拉直）
- 在某点向下跳的函数，$f^{**}$ 把跳跃"补"上

### 闭凸函数的判定

| 函数类型 | 是否闭凸 | 理由 |
|---------|---------|------|
| 连续凸函数（dom 闭凸） | 是 | 连续 → lsc |
| 范数 $\|x\|_p$ | 是 | 连续凸 |
| 指示函数 $\delta_C$（$C$ 闭凸） | 是 | $C$ 闭 → $\delta_C$ lsc |
| 任何 $f^*$ | 是 | 共轭永远闭凸（仿射的 sup） |
| $f(x) = e^x$, $-\log x$ | 是 | 连续凸（在定义域上） |
| 自定义分段函数 | 需检查 | 验证 epi(f) 是否闭集 |

### ⚠️ 常见陷阱

> 💡 **概念误区：$f$ 的共轭一定取有限值**
> - 对非强制（non-coercive）的 $f$，$f^*$ 可能在某些点取 $+\infty$
> - 例如 $f(x) = a^\top x + b$ 的共轭 $f^*(y) = \delta_{\{a\}}(y) - b$，只在 $y = a$ 处有限
> - 这是正常的——$\text{dom}(f^*)$ 不必等于全空间

### 练习

1. 验证 Fenchel-Moreau：对 $f(x) = |x|$，计算 $f^*$ 和 $f^{**}$，验证 $f^{**} = f$。
2. 给出一个凸但不闭的函数 $f$，计算 $f^{**}$ 并验证 $f^{**} \neq f$。
3. 利用共轭-次微分互逆定理：若 $f(x) = \frac{1}{2}\|x\|^2$，验证 $\nabla f^* = (\nabla f)^{-1} = \text{id}$。

---

## 2.3 强凸-光滑对偶 ⭐⭐

### 动机

强凸性和光滑性在凸分析基础章节是独立引入的两个概念。但共轭变换揭示了它们之间深刻的**对偶关系**：一个函数的强凸性等价于其共轭的光滑性，反之亦然。

### 定理

**定理 2.3.1**（强凸-光滑对偶）：设 $f$ 是闭凸函数。

$$f \text{ 是 } \mu\text{-强凸} \quad \Longleftrightarrow \quad f^* \text{ 是 } \frac{1}{\mu}\text{-光滑}$$

### 证明（$\Rightarrow$ 方向）

设 $f$ 是 $\mu$-强凸。需要证明 $f^*$ 是 $\frac{1}{\mu}$-光滑，即 $\|\nabla f^*(y_1) - \nabla f^*(y_2)\| \leq \frac{1}{\mu}\|y_1 - y_2\|$。

**Step 1**：由 $f$ 是 $\mu$-强凸，$f$ 的最小值存在且唯一（对每个线性扰动）。特别是，$f^*(y) = \sup_x[\langle y, x \rangle - f(x)]$ 的 sup 被取到，即 $\nabla f^*(y)$ 存在且 $\nabla f^*(y) = \arg\max_x [\langle y, x \rangle - f(x)] = \arg\min_x [f(x) - \langle y, x \rangle]$。

**Step 2**：设 $x_i = \nabla f^*(y_i)$（$i = 1, 2$）。由共轭-次微分互逆：$y_i \in \partial f(x_i)$。由 $f$ 可微（强凸在 ri(dom f) 上 Gateaux 可微）：$y_i = \nabla f(x_i)$。

**Step 3**：由 $f$ 的 $\mu$-强凸性（强单调性）：
$$\langle \nabla f(x_1) - \nabla f(x_2), x_1 - x_2 \rangle \geq \mu\|x_1 - x_2\|^2$$
$$\langle y_1 - y_2, \nabla f^*(y_1) - \nabla f^*(y_2) \rangle \geq \mu\|\nabla f^*(y_1) - \nabla f^*(y_2)\|^2$$

由 Cauchy-Schwarz：
$$\|y_1 - y_2\| \cdot \|\nabla f^*(y_1) - \nabla f^*(y_2)\| \geq \mu\|\nabla f^*(y_1) - \nabla f^*(y_2)\|^2$$

除以 $\|\nabla f^*(y_1) - \nabla f^*(y_2)\|$（假设非零）：
$$\|\nabla f^*(y_1) - \nabla f^*(y_2)\| \leq \frac{1}{\mu}\|y_1 - y_2\|$$

这正是 $\frac{1}{\mu}$-Lipschitz 梯度。

### 对偶表

| $f$ 的性质 | $f^*$ 的性质 |
|-----------|-------------|
| $\mu$-强凸 | $\frac{1}{\mu}$-光滑 |
| $L$-光滑 | $\frac{1}{L}$-强凸 |
| 非光滑 | 非强凸 |
| 定义域有界 | 处处有限 |

**反事实推理**：如果不知道这个对偶关系会怎样？你会以为强凸和光滑是两个"独立"的概念，需要分别证明算法的收敛性。但有了对偶关系，你可以把"在原空间利用强凸性"转化为"在对偶空间利用光滑性"，这正是 mirror descent 和对偶加速方法的核心思想。

### 证明（$\Leftarrow$ 方向）⭐⭐⭐

设 $f^*$ 是 $\frac{1}{\mu}$-光滑。需要证明 $f$ 是 $\mu$-强凸。

**Step 1**：由 $f^*$ 是 $\frac{1}{\mu}$-光滑，$\nabla f^*$ 是 $\frac{1}{\mu}$-Lipschitz 的。

**Step 2**：由共轭关系，$\nabla f = (\nabla f^*)^{-1}$。

**Step 3**：$\frac{1}{\mu}$-Lipschitz 的逆映射是 $\mu$-强单调的。具体地，对任意 $x_1, x_2$ 和 $y_i = \nabla f(x_i)$：

由 $\nabla f^*$ 的 $\frac{1}{\mu}$-Lipschitz：$\|x_1 - x_2\| = \|\nabla f^*(y_1) - \nabla f^*(y_2)\| \leq \frac{1}{\mu}\|y_1-y_2\|$。

即 $\|y_1 - y_2\| \geq \mu\|x_1-x_2\|$。

由 co-coercivity（$f^*$ 凸且 $\frac{1}{\mu}$-光滑）：
$$\langle x_1-x_2, y_1-y_2\rangle = \langle \nabla f^*(y_1)-\nabla f^*(y_2), y_1-y_2\rangle \geq \mu\|\nabla f^*(y_1)-\nabla f^*(y_2)\|^2 = \mu\|x_1-x_2\|^2$$

这就是 $\nabla f$ 的 $\mu$-强单调性，等价于 $f$ 的 $\mu$-强凸性。

### 对偶表的完整推导

| $f$ 的性质 | $f^*$ 的性质 | 推导来源 |
|-----------|-------------|---------|
| $\mu$-强凸 | $\frac{1}{\mu}$-光滑 | 强单调性 → Lipschitz 逆映射 |
| $L$-光滑 | $\frac{1}{L}$-强凸 | 对称性（交换 $f$ 和 $f^*$ 的角色） |
| 非光滑 | 非强凸 | 非光滑 $\Leftrightarrow$ $\nabla f$ 不 Lipschitz |
| dom($f$) 有界 | $f^*$ 处处有限 | sup 在有界集上总有限 |
| $f$ 处处有限 | dom($f^*$) 有界 | 对偶关系 |
| 可分 $f = \sum f_i$ | 可分 $f^* = \sum f_i^*$ | 逐分量取 sup |

### 条件数的对偶不变性

若 $f$ 是 $\mu$-强凸且 $L$-光滑（条件数 $\kappa = L/\mu$），则 $f^*$ 是 $\frac{1}{L}$-强凸且 $\frac{1}{\mu}$-光滑（条件数 $\kappa^* = \frac{1/\mu}{1/L} = L/\mu = \kappa$）。

> **本质洞察**：条件数在共轭变换下不变！这意味着"在原空间做优化"和"在对偶空间做优化"面临相同的困难度——条件数相同，所需迭代次数也相同。这解释了为什么对偶加速方法（如 mirror descent）不能通过简单地"切换到对偶空间"来改善条件数——真正的加速需要利用额外的结构。

### 练习

1. 验证：$f(x) = \frac{\mu}{2}\|x\|^2$ 是 $\mu$-强凸的，其共轭 $f^*(y) = \frac{1}{2\mu}\|y\|^2$ 是 $\frac{1}{\mu}$-光滑的。
2. 若 $f$ 是 $\mu$-强凸且 $L$-光滑，证明 $f^*$ 是 $\frac{1}{L}$-强凸且 $\frac{1}{\mu}$-光滑的。$f^*$ 的条件数是什么？
3. 利用对偶关系解释：为什么给 LASSO 的 $\ell_1$ 项加上 elastic net 正则 $\frac{\mu}{2}\|x\|^2$ 会使对偶问题变得光滑？
4. （Moreau 包络改善条件数）设 $f$ 既是 $\mu$-强凸又是 $L$-光滑的。证明其 Moreau 包络 $M_f^{\lambda}$ 是 $\frac{1}{\lambda}$-光滑且 $\frac{\mu}{1+\lambda\mu}$-强凸的；进而取 $\lambda = 1/\mu$，说明 $M_f^{1/\mu}$ 的光滑常数为 $\mu$、强凸常数为 $\mu/2$，故条件数为 $2$——与 $f$ 本身的 $L/\mu$ 相比大幅改善。（这正是 proximal / Moreau–Yosida 正则化能加速病态问题的根源。）

---

## 2.4 Proximal 算子的定义与存在唯一性 ⭐⭐

### 动机

共轭函数提供了对偶视角，但要落地到算法，我们需要一个能"计算"的工具。**Proximal 算子就是把共轭-对偶理论变成可执行迭代的桥梁。**

在光滑优化中，梯度下降每步做 $x_{k+1} = x_k - \eta \nabla f(x_k)$。但如果 $f$ 不可微（如 $\|x\|_1$），梯度不存在，怎么办？Proximal 算子给出了答案——它是梯度步的一种"隐式推广"。

### 定义

**定义 2.4.1**（Proximal 算子）：设 $f: \mathbb{R}^n \to \mathbb{R} \cup \{+\infty\}$ 是 proper closed convex 函数。$f$ 的 proximal 算子是映射 $\text{prox}_f: \mathbb{R}^n \to \mathbb{R}^n$：

$$\text{prox}_f(v) = \arg\min_x \left[ f(x) + \frac{1}{2}\|x - v\|^2 \right]$$

**直觉**：proximal 算子寻找一个点 $x$，它在"靠近 $v$"（由 $\frac{1}{2}\|x-v\|^2$ 惩罚）和"使 $f$ 小"之间取得最佳折中。参数 $\lambda$ 控制折中的比例（带参数版本 $\text{prox}_{\lambda f}$）。

**跨领域类比**：
- proximal 算子之于非光滑优化，就像梯度步之于光滑优化
- proximal 算子像一个"带弹簧的投影"——在 $f = \delta_C$（指示函数）时退化为正交投影 $\Pi_C$，在一般情况下加了一个"软性"的 $f$ 惩罚
- 在物理学中，proximal 算子类似"最小作用量原理"——在"接近当前位置"和"降低势能 $f$"之间取折中

### Proximal 算子的历史

Proximal 算子由法国数学家 Jean-Jacques Moreau (1923-2014) 在 1962 年引入，最初称为 proximity operator（近端算子）。Moreau 的动机是把投影算子推广到非线性情形——投影是距离最近的点（$\arg\min \|x-v\|^2$ s.t. $x \in C$），proximal 是"广义距离"最近的点（$\arg\min f(x) + \frac{1}{2}\|x-v\|^2$）。

Rockafellar (1976) 提出了 proximal point algorithm，但直到 2009-2014 年 proximal 方法才在信号处理和机器学习社区大规模流行——ISTA/FISTA (Beck-Teboulle 2009)、ADMM (Boyd-Parikh 2011)、proximal 算法综述 (Parikh-Boyd 2014) 是三个里程碑。

### Proximal 算子作为梯度步的推广 ⭐⭐

**显式梯度步**（光滑函数）：
$$x_{k+1} = x_k - \eta\nabla f(x_k) = \arg\min_x \left[f(x_k) + \nabla f(x_k)^\top(x - x_k) + \frac{1}{2\eta}\|x - x_k\|^2\right]$$

右边是 $f$ 的一阶 Taylor 展开加二次正则化。

**隐式 proximal 步**（一般凸函数）：
$$x_{k+1} = \text{prox}_{\eta f}(x_k) = \arg\min_x \left[f(x) + \frac{1}{2\eta}\|x - x_k\|^2\right]$$

右边直接用 $f(x)$ 而不是其线性近似。

**为什么隐式步更好**：隐式步不需要 $f$ 可微（处理 $\|x\|_1$ 等非光滑函数），且数值上更稳定（二次正则化保证子问题强凸）。代价是需要求解一个优化子问题——但对很多常见 $f$，这个子问题有闭式解（即 prox 的闭式）。

### 带参数的 Proximal 算子 ⭐⭐

在很多文献中，proximal 算子的参数位置不统一，需要特别注意：

**约定 1**（本教程使用）：$\text{prox}_{\lambda f}(v) = \arg\min_x [f(x) + \frac{1}{2\lambda}\|x-v\|^2]$

**约定 2**（部分文献）：$\text{prox}_f^\lambda(v) = \arg\min_x [\lambda f(x) + \frac{1}{2}\|x-v\|^2]$

两者的关系：$\text{prox}_{\lambda f}(v) = \text{prox}_f^\lambda(v)$（实际上是同一个东西的不同记法）。

**参数 $\lambda$ 的意义**：$\lambda$ 越大，proximal 步越"大胆"——更多地降低 $f$，更少地关心"靠近 $v$"。$\lambda \to 0$ 时 $\text{prox}_{\lambda f}(v) \to v$（不动），$\lambda \to \infty$ 时 $\text{prox}_{\lambda f}(v) \to \arg\min f$（直接跳到最优）。

### 存在唯一性定理 ⭐⭐

**定理 2.4.2**：若 $f$ 是 proper closed convex，则对每个 $v \in \mathbb{R}^n$，$\text{prox}_f(v)$ 存在且唯一。

**证明**：

**存在性**：$g(x) = f(x) + \frac{1}{2}\|x-v\|^2$ 是 proper（$f$ proper 且二次项有限）、闭（$f$ 闭且二次项连续）、且 $1$-强凸的（$\frac{1}{2}\|x-v\|^2$ 贡献 $\mu = 1$ 的强凸性）。强凸函数是 coercive 的（$g(x) \to +\infty$ 当 $\|x\| \to \infty$），故 $\inf g$ 被取到。

**唯一性**：$g$ 是严格凸的（甚至强凸），最小值点唯一。

### Proximal 算子与次微分的关系 ⭐⭐

**定理 2.4.3**（Proximal resolvent）：

$$p = \text{prox}_f(v) \quad \Longleftrightarrow \quad v - p \in \partial f(p) \quad \Longleftrightarrow \quad \text{prox}_f = (I + \partial f)^{-1}$$

**证明**：$p = \text{prox}_f(v)$ 意味着 $p$ 最小化 $f(x) + \frac{1}{2}\|x-v\|^2$。由 Fermat 规则：
$$0 \in \partial\left[f(x) + \frac{1}{2}\|x-v\|^2\right]\bigg|_{x=p} = \partial f(p) + (p - v)$$

即 $v - p \in \partial f(p)$，等价于 $v \in p + \partial f(p) = (I + \partial f)(p)$，即 $p = (I + \partial f)^{-1}(v)$。

**为什么这很重要**：这个等价关系把"最优性条件"（$0 \in \partial f(x^*)$）和"不动点"（$x^* = \text{prox}_{\lambda f}(x^*)$）联系起来。所有 proximal 算法的收敛性证明都基于这个对应。

> **本质洞察**：proximal 算子 = 次微分 resolvent = 隐式梯度步 = 含"吸引势"的投影。这四种解释是同一个数学对象的四张面孔，理解它们的等价性是掌握 proximal 算法族的关键。

### ⚠️ 常见陷阱

> 💡 **概念误区："PPO 的 proximal 就是 Moreau 意义的 prox"**
> - PPO（Proximal Policy Optimization）的"proximal"来自 KL trust region / clipped surrogate
> - 与 Moreau-Yosida proximal 算子**没有直接数学关系**，只是术语巧合
> - Schulman 2017 的命名来源是 TRPO 的 trust-region 思想，不是 Moreau 1965 的 proximity operator

> ⚠️ **编程陷阱：proximal gradient 中 prox 的参数**
> - $\text{prox}_{\lambda f}(v) = \arg\min_x [f(x) + \frac{1}{2\lambda}\|x-v\|^2]$（注意 $\lambda$ 在分母）
> - 很多论文/代码中 $\lambda$ 的位置不一致（有的写 $\lambda f$，有的写 $\frac{1}{\lambda}$ 在二次项前）
> - 统一约定：$\text{prox}_{\lambda f}$ 表示对 $\lambda f$ 取 prox，二次项系数为 $\frac{1}{2}$

### Proximal 算子的四种等价描述 ⭐⭐

以下四种描述是同一个数学对象的不同视角，理解它们的等价性是掌握 proximal 理论的关键：

| 视角 | 数学描述 | 直觉 |
|------|---------|------|
| **优化** | $\text{prox}_f(v) = \arg\min_x[f(x) + \frac{1}{2}\|x-v\|^2]$ | 在靠近 $v$ 和降低 $f$ 之间折中 |
| **次微分** | $p = \text{prox}_f(v) \Leftrightarrow v-p \in \partial f(p)$ | 残差是次梯度 |
| **Resolvent** | $\text{prox}_f = (I + \partial f)^{-1}$ | 单调算子的预解式 |
| **不动点** | $x^* = \arg\min f \Leftrightarrow x^* = \text{prox}_{\lambda f}(x^*)$ | 最优解是不动点 |

**为什么需要四种视角**：

- 优化视角：计算 prox（解子问题）
- 次微分视角：推导 prox 的闭式解（通过最优性条件）
- Resolvent 视角：连接单调算子理论，分析收敛性
- 不动点视角：设计算法（迭代到不动点）

### 练习

1. 证明：$\text{prox}_{\delta_C}(v) = \Pi_C(v)$（指示函数的 proximal 就是投影）。
2. 证明：$\text{prox}_{\frac{1}{2}\|\cdot\|^2}(v) = \frac{v}{2}$。
3. 用 resolvent 关系证明 Fermat 规则的等价形式：$x^* = \arg\min f \Leftrightarrow x^* = \text{prox}_{\lambda f}(x^*)$ 对任何 $\lambda > 0$。
4. 对 $f(x) = \delta_{\{x \geq 0\}}(x) + |x-3|$（非负约束 + 绝对值），计算 $\text{prox}_f(v)$ 并画出函数图形。

---

## 2.5 五个必记 Proximal 算子的闭式解 ⭐⭐

### 动机

Proximal 算子的实用性取决于我们能否高效计算它。幸运的是，很多常见函数的 proximal 有闭式解。掌握这5个核心闭式就足以覆盖绝大多数机器人优化场景。

### Proximal 算子的代数运算规则 ⭐⭐

在计算复杂函数的 prox 之前，先掌握一组代数规则，可以把复杂的 prox 分解为简单 prox 的组合。

**规则 1：可分性**。若 $f(x) = \sum_i f_i(x_i)$（各分量独立），则：
$$(\text{prox}_f(v))_i = \text{prox}_{f_i}(v_i)$$

逐分量独立计算。这就是为什么 $\ell_1$ 的 prox 可以逐分量做软阈值。

**规则 2：仿射变换**。若 $g(x) = f(ax + b)$（$a \neq 0$），则：
$$\text{prox}_g(v) = \frac{1}{a}\left(\text{prox}_{a^2f}\left(av + b\right) - b\right)$$

**规则 3：加仿射项**。$g(x) = f(x) + \langle a, x \rangle + c$，则：
$$\text{prox}_{\lambda g}(v) = \text{prox}_{\lambda f}(v - \lambda a)$$

仿射项只是把输入平移了 $\lambda a$。

**规则 4：加二次项**。$g(x) = f(x) + \frac{\rho}{2}\|x - a\|^2$，则：
$$\text{prox}_{\lambda g}(v) = \text{prox}_{\frac{\lambda}{1+\lambda\rho}f}\left(\frac{v + \lambda\rho a}{1 + \lambda\rho}\right)$$

**规则 5：正交变换**。若 $U$ 是正交矩阵，$g(x) = f(Ux)$，则：
$$\text{prox}_g(v) = U^\top \text{prox}_f(Uv)$$

这解释了为什么核范数的 prox 可以在 SVD 域中做——SVD 的酉变换不影响 prox 的结构。

**规则 6：Moreau 分解**。已知 $\text{prox}_f$ 可以推出 $\text{prox}_{f^*}$：
$$\text{prox}_{f^*}(v) = v - \text{prox}_f(v)$$

### Proximal 算子的数值计算 ⭐⭐

当 prox 没有闭式解时，需要数值求解 $\min_x [f(x) + \frac{1}{2\lambda}\|x-v\|^2]$。

**方法 1：内部 Newton 法**。目标函数是 $\lambda$-强凸的（当 $f$ 凸时），Newton 法二次收敛。通常 3-5 次迭代即可达到机器精度。

**方法 2：二分法**（标量情形）。对 $\min_x [f(x) + \frac{1}{2\lambda}(x-v)^2]$，最优性条件 $0 \in \partial f(x) + \frac{1}{\lambda}(x-v)$，由单调性可用二分法。

**方法 3：CVXPY**。用 CVXPY 求解 prox 子问题，适合原型验证。

```python
import cvxpy as cp

def prox_cvxpy(f_cvxpy, v, lam):
    """通用 prox 计算（用 CVXPY，慢但通用）"""
    x = cp.Variable(len(v))
    prob = cp.Problem(cp.Minimize(f_cvxpy(x) + 0.5/lam * cp.sum_squares(x - v)))
    prob.solve()
    return x.value
```

### 2.5.1 软阈值（$\ell_1$ 范数的 prox）⭐⭐

$$\text{prox}_{\lambda\|\cdot\|_1}(v) = \text{sign}(v) \odot \max(|v| - \lambda, 0)$$

**推导**：由于 $\|\cdot\|_1$ 可分（逐分量独立），只需解标量问题：
$$p_i = \arg\min_{x_i} \left[ \lambda|x_i| + \frac{1}{2}(x_i - v_i)^2 \right]$$

由 Fermat 规则：$0 \in \lambda \partial|x_i| + (x_i - v_i)$

**情况 1**：$x_i > 0$。则 $\partial|x_i| = \{1\}$，得 $\lambda + x_i - v_i = 0$，即 $x_i = v_i - \lambda$。需要 $x_i > 0$，即 $v_i > \lambda$。

**情况 2**：$x_i < 0$。则 $\partial|x_i| = \{-1\}$，得 $-\lambda + x_i - v_i = 0$，即 $x_i = v_i + \lambda$。需要 $x_i < 0$，即 $v_i < -\lambda$。

**情况 3**：$x_i = 0$。则 $\partial|x_i| = [-1, 1]$，需要 $v_i \in [-\lambda, \lambda]$。

合并：$p_i = \text{sign}(v_i) \max(|v_i| - \lambda, 0)$。

**这就是"软阈值"**——值的绝对值小于 $\lambda$ 的分量被"杀死"为零（产生稀疏性），大于 $\lambda$ 的分量被"收缩" $\lambda$。

### 2.5.2 块软阈值（$\ell_2$ 范数的 prox）⭐⭐

$$\text{prox}_{\lambda\|\cdot\|_2}(v) = \left(1 - \frac{\lambda}{\|v\|_2}\right)_+ v = \begin{cases} \left(1 - \frac{\lambda}{\|v\|_2}\right) v & \|v\|_2 > \lambda \\ 0 & \|v\|_2 \leq \lambda \end{cases}$$

**推导**：需解 $\min_x \lambda\|x\|_2 + \frac{1}{2}\|x-v\|^2$。

由旋转不变性，最优解在 $v$ 的方向上：$x = \alpha v/\|v\|$（$\alpha \geq 0$）。

代入变为标量问题：$\min_{\alpha \geq 0} \lambda\alpha + \frac{1}{2}(\alpha - \|v\|)^2$。

求导：$\lambda + \alpha - \|v\| = 0 \Rightarrow \alpha = \|v\| - \lambda$。若 $\|v\| \leq \lambda$ 则 $\alpha = 0$。

**区别于软阈值**：$\ell_2$ 的 prox 是"整组杀死或整组收缩"（group lasso 中用），而 $\ell_1$ 是逐分量独立操作。

### 2.5.3 凸集投影 ⭐

$$\text{prox}_{\delta_C}(v) = \Pi_C(v) = \arg\min_{x \in C} \|x - v\|^2$$

常见投影：
- **非负象限**：$\Pi_{\mathbb{R}^n_+}(v) = \max(v, 0)$（逐元素取正部）
- **Box**：$\Pi_{[l,u]}(v) = \min(\max(v, l), u)$
- **$\ell_2$ 球**：$\Pi_{B(0,r)}(v) = v \cdot \min(1, r/\|v\|)$
- **概率单纯形**：$\Pi_\Delta(v) = \max(v - \tau\mathbf{1}, 0)$，$\tau$ 由 $\mathbf{1}^\top \Pi_\Delta(v) = 1$ 确定（$O(n\log n)$ 排序算法）

### 2.5.4 二次函数的 prox

$$\text{prox}_{\lambda \cdot \frac{1}{2}x^\top Q x}(v) = (I + \lambda Q)^{-1} v$$

**推导**：$\arg\min_x [\frac{\lambda}{2}x^\top Q x + \frac{1}{2}\|x-v\|^2]$。

求导令零：$\lambda Q x + x - v = 0$，即 $(I + \lambda Q)x = v$。

### 2.5.5 奇异值阈值（SVT，核范数的 prox）⭐⭐⭐

$$\text{prox}_{\lambda\|\cdot\|_*}(V) = U \cdot \text{diag}((\sigma_i - \lambda)_+) \cdot W^\top$$

其中 $V = U \cdot \text{diag}(\sigma_i) \cdot W^\top$ 是 SVD。

**直觉**：核范数 $\|X\|_* = \sum_i \sigma_i(X)$ 对奇异值的作用就像 $\ell_1$ 对向量分量的作用——SVT 就是在奇异值上做软阈值。

**机器人应用**：低秩矩阵补全（视觉 SLAM 中的结构恢复）、鲁棒 PCA（动态背景分离）。

**组稀疏 vs 元素稀疏的对比**：

| 正则化 | prox 操作 | 稀疏模式 | 应用场景 |
|--------|---------|---------|---------|
| $\|x\|_1$ | 逐元素软阈值 | 元素级稀疏 | LASSO, compressed sensing |
| $\sum_j\|x_{G_j}\|_2$ | 块软阈值 | 组级稀疏 | Group LASSO, 多任务学习 |
| $\|X\|_*$ | SVT | 奇异值稀疏（低秩） | 矩阵补全, 鲁棒 PCA |
| $\|x\|_2^2$ | 简单缩放 | 无稀疏（全局收缩） | Ridge 回归 |

### Proximal 速查表

| $f(x)$ | $\text{prox}_{\lambda f}(v)$ | 复杂度 |
|---------|------------------------------|--------|
| $\|x\|_1$ | $\text{sign}(v)\odot(|v|-\lambda)_+$ | $O(n)$ |
| $\|x\|_2$ | $(1-\lambda/\|v\|)_+ v$ | $O(n)$ |
| $\delta_C$ | $\Pi_C(v)$ | 视 $C$ 而定 |
| $\frac{1}{2}x^\top Q x$ | $(I+\lambda Q)^{-1}v$ | $O(n^3)$ 或预分解 |
| $\|X\|_*$ | SVT | $O(\min(m,n)mn)$ |
| $\frac{1}{2}\|x\|^2$ | $v/(1+\lambda)$ | $O(n)$ |
| $\|x\|_\infty$ | $v - \lambda\Pi_{\|\cdot\|_1 \leq 1}(v/\lambda)$ | $O(n\log n)$ |

### ⚠️ 常见陷阱

> 💡 **概念误区：范数 vs 范数平方的 prox 差异很大**
> - $\text{prox}_{\lambda\|\cdot\|_2}(v) = (1 - \lambda/\|v\|)_+ v$（块软阈值，有死区）
> - $\text{prox}_{\lambda \cdot \frac{1}{2}\|\cdot\|^2}(v) = v/(1+\lambda)$（简单缩放，无死区）
> - 混淆两者是 ADMM 实现中最常见的 bug 之一

> ⚠️ **编程陷阱：概率单纯形投影不是"代数闭式"**
> - 虽然形式上是 $\max(v - \tau\mathbf{1}, 0)$，但 $\tau$ 需要通过排序算法确定
> - 实现时用 Held et al. 1974 的 $O(n\log n)$ 算法或 Condat 2016 的 $O(n)$ 算法
> - 不要试图"解析求解" $\tau$——它是分段线性函数的根

### 练习

1. 从软阈值公式出发，用 Moreau 分解（下一节）推导 $\ell_\infty$ 球投影的公式。
2. 验证：对 $v = (3, -1, 0.5)^\top$ 和 $\lambda = 1$，手动计算 $\text{prox}_{\lambda\|\cdot\|_1}(v)$ 和 $\text{prox}_{\lambda\|\cdot\|_2}(v)$。
3. 实现 SVT 并对随机矩阵验证：$\text{prox}_{\lambda\|\cdot\|_*}(V)$ 的奇异值确实是 $(\sigma_i - \lambda)_+$。

---

## 2.6 Moreau 包络与光滑化 ⭐⭐

### 动机

非光滑函数（如 $\|x\|_1$）不能直接用梯度法。但如果能把它"光滑化"成一个近似函数，同时保持最优解不变，就可以用梯度法了。**Moreau 包络就是这样一个"通用光滑化"工具。**

### 定义

**定义 2.6.1**（Moreau 包络 / Moreau-Yosida 正则化）：

$$M_f^\lambda(v) = \inf_x \left[ f(x) + \frac{1}{2\lambda}\|x - v\|^2 \right] = f(\text{prox}_{\lambda f}(v)) + \frac{1}{2\lambda}\|\text{prox}_{\lambda f}(v) - v\|^2$$

### 关键性质 ⭐⭐

**定理 2.6.2**：设 $f$ 是 proper closed convex，$\lambda > 0$。

1. $M_f^\lambda$ 是凸的且处处可微（即使 $f$ 不可微！）
2. 梯度公式：$\nabla M_f^\lambda(v) = \frac{1}{\lambda}(v - \text{prox}_{\lambda f}(v))$
3. 梯度 Lipschitz：$\nabla M_f^\lambda$ 是 $\frac{1}{\lambda}$-Lipschitz 的
4. $\inf M_f^\lambda = \inf f$，且 $\arg\min M_f^\lambda = \arg\min f$
5. 当 $\lambda \to 0$：$M_f^\lambda(v) \to f(v)$（逐点收敛）
6. $M_f^\lambda(v) \leq f(v)$（Moreau 包络总不超过原函数）
7. 若 $f$ 是 $\mu$-强凸，则 $M_f^\lambda$ 是 $\frac{\mu}{1+\lambda\mu}$-强凸的

**直觉**：Moreau 包络是 $f$ 的"模糊版本"——它在每一点取 $f$ 在附近的加权最小值。$\lambda$ 越大，"模糊半径"越大，函数越光滑（但也越远离原函数）。$\lambda \to 0$ 时回到原函数。

**跨领域类比**：Moreau 包络像图像处理中的高斯模糊——用一个二次核（代替高斯核）对函数做"平滑"。$\lambda$ 相当于模糊半径。模糊后的图像没有尖锐的边缘（对应光滑性），但整体形状不变（最小值点不变）。

### 性质的完整证明

**性质 6 的证明**（$M_f^\lambda \leq f$）：
$$M_f^\lambda(v) = \inf_x \left[f(x) + \frac{1}{2\lambda}\|x-v\|^2\right] \leq f(v) + \frac{1}{2\lambda}\|v-v\|^2 = f(v)$$

取 $x = v$ 即得上界。

**性质 4 的证明**（最小值点不变）：

$(\Rightarrow)$：设 $x^* = \arg\min f$。则 $M_f^\lambda(x^*) = \inf_x[f(x) + \frac{1}{2\lambda}\|x-x^*\|^2] \leq f(x^*) = \inf f$。又 $M_f^\lambda(v) \geq \inf f$（因为 $f(x) + \frac{1}{2\lambda}\|x-v\|^2 \geq \inf f$）。因此 $M_f^\lambda(x^*) = \inf f = \inf M_f^\lambda$。

$(\Leftarrow)$：设 $v^* = \arg\min M_f^\lambda$。则 $\nabla M_f^\lambda(v^*) = 0$，即 $\frac{1}{\lambda}(v^* - \text{prox}_{\lambda f}(v^*)) = 0$，故 $v^* = \text{prox}_{\lambda f}(v^*)$。由 Fermat 规则 $v^* = \text{prox}_{\lambda f}(v^*) \Leftrightarrow 0 \in \partial f(v^*) \Leftrightarrow v^* = \arg\min f$。

**性质 1 的证明**（凸性与光滑性）：

凸性来自 infimal 卷积的保凸性。光滑性是核心——我们通过 infimal 卷积的共轭来证明。

$M_f^\lambda = f \square \frac{1}{2\lambda}\|\cdot\|^2$，其共轭为 $(M_f^\lambda)^* = f^* + \frac{\lambda}{2}\|\cdot\|^2$。

$f^* + \frac{\lambda}{2}\|\cdot\|^2$ 是 $\lambda$-强凸的。由强凸-光滑对偶，$(M_f^\lambda)^{**} = M_f^\lambda$ 是 $\frac{1}{\lambda}$-光滑的。

**梯度公式的证明**：

设 $p = \text{prox}_{\lambda f}(v)$。对 $v$ 做微小扰动 $v + \Delta v$：

$$M_f^\lambda(v + \Delta v) \leq f(p) + \frac{1}{2\lambda}\|p - v - \Delta v\|^2$$
$$= M_f^\lambda(v) - \frac{1}{\lambda}(p - v)^\top \Delta v + \frac{1}{2\lambda}\|\Delta v\|^2$$

即 $M_f^\lambda(v + \Delta v) - M_f^\lambda(v) \leq \frac{1}{\lambda}(v - p)^\top \Delta v + O(\|\Delta v\|^2)$。

类似地可以证明下界，得到 $\nabla M_f^\lambda(v) = \frac{1}{\lambda}(v - p)$。

### Moreau 包络的具体计算 ⭐⭐

**例 1**：$f(x) = |x|$

$$M_f^\lambda(v) = \min_x \left[|x| + \frac{1}{2\lambda}(x-v)^2\right]$$

最优 $x^* = \text{prox}_{\lambda|\cdot|}(v) = \text{sign}(v)\max(|v|-\lambda, 0)$。

- 若 $|v| \leq \lambda$：$x^* = 0$，$M_f^\lambda(v) = 0 + \frac{v^2}{2\lambda} = \frac{v^2}{2\lambda}$
- 若 $|v| > \lambda$：$x^* = v - \lambda\text{sign}(v)$，$M_f^\lambda(v) = |v| - \lambda + \frac{\lambda^2}{2\lambda} = |v| - \frac{\lambda}{2}$

合并：$M_f^\lambda(v) = \begin{cases} \frac{v^2}{2\lambda} & |v| \leq \lambda \\ |v| - \frac{\lambda}{2} & |v| > \lambda \end{cases}$

这正是 **Huber 函数**（参数 $\delta = \lambda$）！即：**$|x|$ 的 Moreau 包络是 Huber 函数**。

**直觉验证**：$|x|$ 在 $x=0$ 处不可微。Moreau 包络用一段抛物线替换了尖角处，使之光滑——$\lambda$ 越大，抛物线段越宽，函数越光滑。

**例 2**：$f(x) = \delta_{[-1,1]}(x)$

$$M_f^\lambda(v) = \min_{x \in [-1,1]} \frac{1}{2\lambda}(x-v)^2 = \frac{1}{2\lambda}\text{dist}(v, [-1,1])^2$$

- 若 $|v| \leq 1$：$M_f^\lambda(v) = 0$
- 若 $v > 1$：$M_f^\lambda(v) = \frac{(v-1)^2}{2\lambda}$
- 若 $v < -1$：$M_f^\lambda(v) = \frac{(v+1)^2}{2\lambda}$

这是 box 约束的"软化"——违反约束的程度用二次惩罚衡量。

### Moreau 包络与 Nesterov 光滑化的联系 ⭐⭐⭐

Nesterov (2005) 提出了一种通用的非光滑函数光滑化方法。对 $f(x) = \max_{u \in Q} [\langle Ax, u \rangle - \phi(u)]$（很多非光滑函数都可以写成这种 max 结构），添加强凸正则化：

$$f_\mu(x) = \max_{u \in Q} [\langle Ax, u \rangle - \phi(u) - \frac{\mu}{2}\|u\|^2]$$

$f_\mu$ 是 $f$ 的光滑近似，误差 $|f_\mu - f| \leq \frac{\mu}{2}D^2$（$D$ 是 $Q$ 的直径）。

**与 Moreau 包络的关系**：Nesterov 光滑化可以看作在对偶空间做 Moreau 包络！$-\phi(u) - \frac{\mu}{2}\|u\|^2 = -M_\phi^\mu(u)$... 更精确地说，Nesterov 的 $f_\mu$ 等价于 $f$ 的 Moreau 包络在对偶空间的应用。

### 机器人应用：GNC 与 Moreau 包络 ⭐⭐⭐

**Graduated Non-Convexity（GNC）** 是 robust SLAM 的核心技术（Yang-Carlone 2020，ICRA Best Paper）。其数学本质是：

1. 把非凸的鲁棒损失（如 Geman-McClure）写成 Moreau 包络的形式
2. 通过调节参数 $\mu$（类似 $\lambda$），从凸松弛逐步过渡到原非凸问题
3. 每一步都是凸优化，最终逼近全局最优

> **本质洞察**：Moreau 包络不只是"光滑化工具"——它是连接凸优化和非凸优化的桥梁。通过 continuation method（逐步减小 $\lambda$），可以把非凸问题的求解分解为一系列凸问题。这正是 GNC 的工作原理。

### ⚠️ 常见陷阱

> 💡 **概念误区："Moreau 包络 $M_f^\lambda$ 就是 $f$ 加上二次项"**
> - $M_f^\lambda(v) = \min_x[f(x) + \frac{1}{2\lambda}\|x-v\|^2]$ 是对 $x$ 取 min 后的结果
> - 它不是简单地把 $\frac{1}{2\lambda}\|x-v\|^2$ 加到 $f$ 上——加法后的函数是 $v$ 和 $x$ 的联合函数，取 min 后才变成只关于 $v$ 的函数
> - Moreau 包络 $\leq f$ 处处成立（因为取了 min），而"$f$ + 二次项" $> f$

### 练习

1. 计算 $f(x) = |x|$ 的 Moreau 包络 $M_f^\lambda(v)$，画出不同 $\lambda$ 下的图形。
2. 验证 $f(x) = \|x\|_1$ 的 Moreau 包络的梯度是 $\frac{1}{\lambda}(v - \text{prox}_{\lambda\|\cdot\|_1}(v))$。
3. 对 $f(x) = \delta_{[-1,1]}(x)$，计算 Moreau 包络并验证它就是 Huber 函数的一个变体。

---

## 2.7 Moreau 分解定理 ⭐⭐

### 动机

Moreau 分解是 proximal 理论中最优美的结果之一。它把任意向量 $v$ "分解"为两个 proximal 的和，就像正交分解把向量分成投影和余量。

### 定理

**定理 2.7.1**（Moreau 分解）：对任何 proper closed convex 函数 $f$ 和任何 $v \in \mathbb{R}^n$：

$$v = \text{prox}_f(v) + \text{prox}_{f^*}(v)$$

**缩放版本**：

$$v = \text{prox}_{\lambda f}(v) + \lambda \cdot \text{prox}_{f^*/\lambda}(v/\lambda)$$

### 证明

设 $p = \text{prox}_f(v)$。由 resolvent 关系：$v - p \in \partial f(p)$。

由共轭-次微分互逆：$v - p \in \partial f(p) \Leftrightarrow p \in \partial f^*(v - p)$。

再用 resolvent 关系：$p \in \partial f^*(v - p) \Leftrightarrow v - (v-p) \in \partial f^*(v-p) \Leftrightarrow v - p = \text{prox}_{f^*}(v)$。

因此 $v = p + (v - p) = \text{prox}_f(v) + \text{prox}_{f^*}(v)$。

### 应用：从一个 prox 推出另一个

**例：从 $\ell_\infty$ 球投影推出 $\ell_1$ 的 prox**

由 $\|\cdot\|_1^* = \delta_{\|\cdot\|_\infty \leq 1}$，Moreau 分解给出：
$$v = \text{prox}_{\lambda\|\cdot\|_1}(v) + \lambda \cdot \Pi_{\|\cdot\|_\infty \leq 1}(v/\lambda)$$

因此 $\text{prox}_{\lambda\|\cdot\|_1}(v) = v - \lambda \cdot \Pi_{\|\cdot\|_\infty \leq 1}(v/\lambda)$

这给出了软阈值的另一种推导方式——从对偶空间的投影出发。

**例：从 $\ell_2$ 球投影推出 $\ell_2$ 范数的 prox**

由 $\|\cdot\|_2^* = \delta_{\|\cdot\|_2 \leq 1}$：
$$\text{prox}_{\lambda\|\cdot\|_2}(v) = v - \lambda \cdot \Pi_{\|\cdot\|_2 \leq 1}(v/\lambda) = v - \lambda \cdot \frac{v/\lambda}{\max(1, \|v/\lambda\|)} = v \cdot \left(1 - \frac{\lambda}{\max(\lambda, \|v\|)}\right)$$

即块软阈值公式。

### Moreau 分解的多种推导方式 ⭐⭐

**方式 1**（通过 resolvent，已在上面给出）：利用 $v - p \in \partial f(p)$ 和共轭-次微分互逆。

**方式 2**（通过 Moreau 包络）：

由 $M_f^\lambda$ 的梯度公式：$\nabla M_f^\lambda(v) = \frac{1}{\lambda}(v - \text{prox}_{\lambda f}(v))$

和 Moreau 包络的共轭关系 $(M_f^\lambda)^* = f^* + \frac{\lambda}{2}\|\cdot\|^2$，可以推出：

$$\nabla (M_f^\lambda)^*(y) = \nabla f^*(y) + \lambda y$$

由共轭的梯度互逆关系，$v$ 和 $y = \nabla M_f^\lambda(v)$ 满足 $v = \nabla(M_f^\lambda)^*(y)$。展开并简化即得 Moreau 分解。

**方式 3**（直接验证）：

设 $p = \text{prox}_f(v)$，$q = v - p$。需验证 $q = \text{prox}_{f^*}(v)$。

$q = \text{prox}_{f^*}(v) \Leftrightarrow v - q \in \partial f^*(q) \Leftrightarrow p \in \partial f^*(q)$。

由共轭-次微分互逆：$p \in \partial f^*(q) \Leftrightarrow q \in \partial f(p)$。

而 $q = v - p \in \partial f(p)$ 正是 resolvent 关系。

### 在 ADMM 中的角色

ADMM 的每步迭代本质上是：
1. 对原函数取 prox（$x$-更新）
2. 对偶变量更新（$u$-更新）
3. 对约束/惩罚取 prox（$z$-更新）

Moreau 分解保证了这些 prox 之间的"能量守恒"——原空间的 prox 和对偶空间的 prox 合起来恰好等于输入。这就是 ADMM 的对偶残差和原残差平衡的数学根源。

### Moreau 分解的计算意义

在实际计算中，Moreau 分解的价值在于：**如果你知道一个函数的 prox，就自动知道其共轭的 prox**。

例如：$\|\cdot\|_1$ 的 prox 是软阈值 $S_\lambda(v)$。由 Moreau 分解：

$$\text{prox}_{\lambda\|\cdot\|_\infty^*}(v) = v - \lambda S_{1/\lambda}(v/\lambda) = v - S_\lambda(v)$$

但 $\|\cdot\|_1^* = \delta_{\|\cdot\|_\infty \leq 1}$（指示函数），其 prox 就是 $\ell_\infty$ 球投影。

因此：$\Pi_{\|\cdot\|_\infty \leq 1}(v) = v - S_1(v)$——从软阈值推出了 $\ell_\infty$ 球投影！

这种"买一送一"的性质在算法实现中非常有用——不需要为每个函数单独推导 prox，只要知道一侧的 prox 就能通过 Moreau 分解得到另一侧。

### 练习

1. 利用 Moreau 分解，从概率单纯形投影推导 max 函数 $f(x) = \max_i x_i$ 的 prox。
2. 验证 Moreau 分解：对 $f = \frac{1}{2}\|\cdot\|^2$，有 $f^* = \frac{1}{2}\|\cdot\|^2$。代入 $\lambda=1$ 的分解式 $v = \operatorname{prox}_f(v) + \operatorname{prox}_{f^*}(v)$，由 $\operatorname{prox}_f(v) = v/2$、$\operatorname{prox}_{f^*}(v) = v/2$ 得 $v = v/2 + v/2 = v$，验证成立。再用缩放版 $v = \operatorname{prox}_{\lambda f}(v) + \lambda\,\operatorname{prox}_{f^*/\lambda}(v/\lambda)$ 对一般 $\lambda>0$ 重新验证，体会 $\lambda$ 参数的作用。
3. 证明缩放版 Moreau 分解从基本版推导。

---

## 2.8 Firmly Nonexpansive 性质与收敛保证 ⭐⭐⭐

### 动机

proximal 算子为什么能保证算法收敛？答案在于它具有 **firmly nonexpansive** 性质——这是一种比 Lipschitz 连续更强的"收缩"性质。

### 定义与定理

**定义 2.8.1**（Firmly nonexpansive）：映射 $T: \mathbb{R}^n \to \mathbb{R}^n$ 是 firmly nonexpansive 的，如果：

$$\|T(x) - T(y)\|^2 \leq \langle x - y, T(x) - T(y) \rangle, \quad \forall x, y$$

**定理 2.8.2**：$\text{prox}_f$ 是 firmly nonexpansive 的。

**证明**：设 $p = \text{prox}_f(x)$，$q = \text{prox}_f(y)$。由 resolvent：
$$x - p \in \partial f(p), \quad y - q \in \partial f(q)$$

由次梯度的单调性（$f$ 凸 $\Rightarrow$ $\partial f$ 单调）：
$$\langle (x-p) - (y-q), p - q \rangle \geq 0$$

展开：$\langle x - y, p - q \rangle - \|p - q\|^2 \geq 0$

即 $\|p - q\|^2 \leq \langle x - y, p - q \rangle$。

### 推论

1. **Nonexpansive**：$\|T(x) - T(y)\| \leq \|x - y\|$（由 Cauchy-Schwarz 立即得到）
2. **1/2-averaged**：$T = \frac{1}{2}I + \frac{1}{2}N$，其中 $N$ 是 nonexpansive 的
3. **Proximal point algorithm 收敛**：$x_{k+1} = \text{prox}_{\lambda f}(x_k)$ 收敛到 $\arg\min f$

**收敛率**：对一般凸函数，proximal point algorithm 以 $O(1/k)$ 速率收敛；对 $\mu$-强凸函数，以线性速率 $(1/(1+\lambda\mu))^k$ 收敛。

### Nonexpansive 映射的等价刻画 ⭐⭐⭐

以下条件等价：

1. $T$ 是 firmly nonexpansive：$\|Tx - Ty\|^2 \leq \langle x-y, Tx-Ty \rangle$
2. $2T - I$ 是 nonexpansive：$\|(2T-I)x - (2T-I)y\| \leq \|x-y\|$
3. $T$ 是 $\frac{1}{2}$-averaged：$T = \frac{1}{2}I + \frac{1}{2}N$，其中 $N$ 是 nonexpansive
4. $I - T$ 也是 firmly nonexpansive

**证明 (1)$\Leftrightarrow$(4)**：设 $S = I - T$。
$$\|Sx - Sy\|^2 = \|(x-y) - (Tx-Ty)\|^2 = \|x-y\|^2 - 2\langle x-y, Tx-Ty\rangle + \|Tx-Ty\|^2$$

由 $T$ firmly nonexpansive：$\|Tx-Ty\|^2 \leq \langle x-y, Tx-Ty\rangle$。

代入：$\|Sx-Sy\|^2 \leq \|x-y\|^2 - 2\|Tx-Ty\|^2 + \|Tx-Ty\|^2 = \|x-y\|^2 - \|Tx-Ty\|^2$

即 $\|Sx-Sy\|^2 + \|Tx-Ty\|^2 \leq \|x-y\|^2$。

反之类似。因此 $T$ 和 $I-T$ 的 firmly nonexpansive 性质是对称的——这对应了 Moreau 分解中 $\text{prox}_f$ 和 $\text{prox}_{f^*}$ 的对称角色。

### Proximal Point Algorithm（PPA）⭐⭐

**算法**：$x_{k+1} = \text{prox}_{\lambda f}(x_k)$。

**定理 2.8.3**：若 $\arg\min f \neq \emptyset$，则 PPA 生成的序列 $\{x_k\}$ 弱收敛到 $\arg\min f$ 中的某一点。

**证明核心**：设 $x^* \in \arg\min f$。由 firmly nonexpansive：
$$\|x_{k+1} - x^*\|^2 = \|\text{prox}_{\lambda f}(x_k) - \text{prox}_{\lambda f}(x^*)\|^2 \leq \langle x_k - x^*, x_{k+1} - x^*\rangle$$

展开右边并用 Cauchy-Schwarz：$\|x_{k+1}-x^*\|^2 \leq \|x_k-x^*\| \cdot \|x_{k+1}-x^*\|$

因此 $\|x_{k+1}-x^*\| \leq \|x_k-x^*\|$——距离单调不增。

更精确地：$\|x_{k+1}-x^*\|^2 \leq \|x_k-x^*\|^2 - \|x_{k+1}-x_k\|^2$

因此 $\sum_k \|x_{k+1}-x_k\|^2 < \infty$，即 $x_{k+1}-x_k \to 0$。

由 resolvent 关系，$x_k - x_{k+1} \in \lambda\partial f(x_{k+1})$。当 $x_k - x_{k+1} \to 0$ 时，$0 \in \partial f(x^*)$（利用次微分的闭图性），即极限点是最小值点。

**PPA 的实际价值**：PPA 本身不是高效算法（每步需要精确解一个 proximal 子问题）。但它是 ADMM 和 proximal gradient 的"母算法"——ADMM 可以看作 PPA 应用于对偶问题，proximal gradient 可以看作 PPA 的近似版本（用梯度步近似光滑部分的 prox）。

### 与 ADMM 收敛的联系

ADMM 的收敛性证明的核心步骤就是：证明每步迭代可以写成 averaged operator 的复合，因此 Krasnosel'skii-Mann 定理保证收敛。firmly nonexpansive 是这一切的数学基石。

**Krasnosel'skii-Mann 定理**：设 $T$ 是非扩张的（nonexpansive），$\text{Fix}(T) \neq \emptyset$。则迭代 $x_{k+1} = (1-\alpha_k)x_k + \alpha_k T(x_k)$（$\alpha_k \in (0,1)$，$\sum \alpha_k(1-\alpha_k) = \infty$）弱收敛到 $T$ 的不动点。

ADMM 的每步迭代可以写成 $x_{k+1} = T_{\text{ADMM}}(x_k)$，其中 $T_{\text{ADMM}}$ 是两个 firmly nonexpansive 映射的复合的 $\frac{1}{2}$-averaged 化——因此 Krasnosel'skii-Mann 适用。

### ⚠️ 常见陷阱

> 💡 **概念误区：firmly nonexpansive $=$ 收缩映射**
> - 收缩映射要求 $\|Tx-Ty\| \leq c\|x-y\|$（$c < 1$），保证线性收敛
> - Firmly nonexpansive 只要求 $c = 1$（不严格收缩），收敛可以是次线性的
> - 强凸函数的 $\text{prox}$ 是收缩映射（$c = 1/(1+\lambda\mu) < 1$），保证线性收敛
> - 一般凸函数的 $\text{prox}$ 只是 nonexpansive，收敛率 $O(1/k)$

> 🧠 **思维陷阱：以为 PPA 不需要精确 prox**
> - 理论上 PPA 要求精确计算 $\text{prox}_{\lambda f}$
> - 不精确 PPA（inexact PPA）允许近似计算，但需要误差序列可求和 $\sum \epsilon_k < \infty$
> - ADMM 的 $x$-步通常不精确解——这相当于不精确 PPA，需要控制近似误差

### 练习

1. 证明：firmly nonexpansive 蕴含 nonexpansive（提示：Cauchy-Schwarz）。
2. 对 $f(x) = |x|$，直接验证 $\text{prox}_f = \text{sign}(v)(|v|-1)_+$ 是 firmly nonexpansive 的。
3. 证明：proximal point algorithm $x_{k+1} = \text{prox}_{\lambda f}(x_k)$ 对强凸 $f$ 线性收敛，收敛因子为 $1/(1+\lambda\mu)$。
4. 证明 $T$ firmly nonexpansive $\Leftrightarrow$ $2T-I$ nonexpansive。

---

## 2.9 共轭函数典型例子大全 ⭐⭐

### 完整共轭-prox 对照表

| $f(x)$ | $f^*(y)$ | $\text{prox}_{\lambda f}(v)$ |
|---------|----------|-------------------------------|
| $\frac{1}{2}x^\top Q x$ ($Q \succ 0$) | $\frac{1}{2}y^\top Q^{-1}y$ | $(I + \lambda Q)^{-1}v$ |
| $\frac{1}{2}\|x\|^2$ | $\frac{1}{2}\|y\|^2$ | $v/(1+\lambda)$ |
| $\|x\|_p$ | $\delta_{\|y\|_q \leq 1}$ | 视 $p$ 而定 |
| $\|x\|_1$ | $\delta_{\|y\|_\infty \leq 1}$ | 软阈值 |
| $\|x\|_2$ | $\delta_{\|y\|_2 \leq 1}$ | 块软阈值 |
| $\delta_C(x)$ | $\sigma_C(y)$ | $\Pi_C(v)$ |
| $\sigma_C(x)$ | $\delta_C(y)$ | $v - \lambda\Pi_C(v/\lambda)$ |
| $\langle a, x \rangle + b$ | $\delta_{\{a\}}(y) - b$ | $v - \lambda a$ |
| $-\log x$ | $-1-\log(-y)$, $y<0$ | $(v+\sqrt{v^2+4\lambda})/2$ |
| $e^x$ | $y\log y - y$, $y>0$ | Lambert W 需要 |
| log-sum-exp | 负熵（单纯形上） | 无初等闭式 |
| Huber$_\delta$ | $\frac{1}{2}y^2 + \delta_{[-\delta,\delta]}(y)$ | 收缩公式 |
| $\|X\|_*$ | $\delta_{\|Y\|_{op} \leq 1}$ | SVT |
| hinge $\max(0,1-x)$ | $y$ ($y \in [-1,0]$) | 分段公式 |

### 两条核心恒等式

**Moreau 分解**：$v = \text{prox}_f(v) + \text{prox}_{f^*}(v)$

**共轭梯度关系**：对闭真凸严格凸可微 $f$：$\nabla f^* = (\nabla f)^{-1}$

这两条公式贯穿整个 proximal 算法理论，是从一个 prox/共轭推出另一个的万能工具。

---

## 2.10 Fenchel 对偶问题 ⭐⭐

### 动机

回顾：Lagrange 对偶处理的是"约束优化"。但很多问题的结构不是"目标+约束"，而是"两个函数的和"：$\min f(x) + g(Ax)$。Fenchel 对偶直接利用共轭函数处理这种结构，无需引入 Lagrange 乘子。

### Fenchel 对偶定理

**定理 2.10.1**（Fenchel 对偶）：考虑原问题

$$p^* = \inf_x [f(x) + g(Ax)]$$

其 Fenchel 对偶问题为：

$$d^* = \sup_y [-f^*(-A^\top y) - g^*(y)]$$

**弱对偶**：$d^* \leq p^*$ 总成立。

**强对偶条件**：若 $\text{ri}(\text{dom}\,f) \cap A^{-1}(\text{ri}(\text{dom}\,g)) \neq \emptyset$（即存在点 $\bar{x}$ 使 $\bar{x} \in \text{ri}(\text{dom}\,f)$ 且 $A\bar{x} \in \text{ri}(\text{dom}\,g)$），则 $p^* = d^*$ 且对偶最优被取到。

### 证明（弱对偶）

对任意可行 $x$ 和对偶变量 $y$：

$$f(x) + g(Ax) = f(x) + g(Ax) + \langle y, Ax \rangle - \langle y, Ax \rangle$$

由 Fenchel-Young 不等式：
- $f(x) \geq \langle -A^\top y, x \rangle - f^*(-A^\top y)$（取 $s = -A^\top y$）
- $g(Ax) \geq \langle y, Ax \rangle - g^*(y)$

相加：$f(x) + g(Ax) \geq -f^*(-A^\top y) - g^*(y)$

对 $x$ 取 inf，对 $y$ 取 sup：$p^* \geq d^*$。

### Fenchel 对偶的应用实例

**例：LASSO 的 Fenchel 对偶**

原问题：$\min_x \frac{1}{2}\|Ax-b\|^2 + \lambda\|x\|_1$

写成 $f(x) + g(Ax)$ 形式：$f(x) = \lambda\|x\|_1$，$g(u) = \frac{1}{2}\|u-b\|^2$，$A$ 是设计矩阵。

计算共轭：
- $f^*(s) = \delta_{\|s\|_\infty \leq \lambda}$（$\ell_1$ 范数共轭是 $\ell_\infty$ 球指示函数的缩放）
- $g^*(y) = \frac{1}{2}\|y\|^2 + b^\top y$

对偶问题：$\max_y -\delta_{\|A^\top y\|_\infty \leq \lambda} - \frac{1}{2}\|y\|^2 - b^\top y$

即：$\max_y -\frac{1}{2}\|y\|^2 - b^\top y$ s.t. $\|A^\top y\|_\infty \leq \lambda$

等价地：$\min_y \frac{1}{2}\|y+b\|^2$ s.t. $\|A^\top y\|_\infty \leq \lambda$

这是一个**光滑目标 + 多面体约束**的 QP！比原问题（非光滑目标）更容易求解。

**关键观察**：$\ell_1$ 正则化的对偶把非光滑性"转移"到了约束中——原问题的非光滑目标变成了对偶问题的简单约束。这是 Fenchel 对偶的威力。

### 对偶证书（Dual Certificate）⭐⭐⭐

在 sparse recovery 和 compressed sensing 中，Fenchel 对偶给出了一个强大的理论工具——**对偶证书**。

考虑 LASSO 的最优性条件：$x^*$ 是最优解当且仅当存在对偶变量 $y^*$ 使得：
$$A^\top y^* \in \lambda \partial\|x^*\|_1$$

即 $(A^\top y^*)_i = \lambda\text{sign}(x_i^*)$（$x_i^* \neq 0$ 时）且 $|(A^\top y^*)_i| \leq \lambda$（$x_i^* = 0$ 时）。

**含义**：如果能构造一个满足这些条件的 $y^*$，就证明了 $x^*$ 确实是最优解——$y^*$ 就是"最优性证书"。

**在 compressed sensing 中的应用**：稀疏恢复 $\min \|x\|_1$ s.t. $Ax = b$ 的对偶是 $\max b^\top y$ s.t. $\|A^\top y\|_\infty \leq 1$。对偶最优 $y^*$ 必须满足 $A^\top y^*$ 在 $x^*$ 的支撑（非零位置）上取值 $\pm 1$——这就是 Fuchs (2004) 的 irrepresentable condition 的来源。

**几何直觉**：$\|A^\top y\|_\infty \leq 1$ 定义了 $y$ 空间中的一个多面体。$b^\top y$ 在这个多面体上的最大值点就是对偶最优解。对偶最优的 $A^\top y^*$ 恰好"接触"了 $\ell_\infty$ 球在 $x^*$ 支撑位置上的面——这是互补松弛的几何表现。

### Fenchel 对偶的一般性强对偶条件

**定理**（Fenchel 强对偶）：对 $\inf_x [f(x) + g(Ax)]$，若以下条件之一成立则强对偶成立：

| 条件 | 名称 | 适用场景 |
|------|------|---------|
| $\exists \bar{x} \in \text{ri(dom} f) : A\bar{x} \in \text{ri(dom} g)$ | Rockafellar 正则条件 | 最一般 |
| $f$ 或 $g$ 连续 | 连续性条件 | 实际中最常见 |
| $\text{dom}(f) \cap A^{-1}(\text{dom}\,g)$ 有相对内部点 | 简化正则条件 | 有限维 |

**注意**：这些条件比 Lagrange 对偶的 Slater 条件更精细——它们直接作用在函数的定义域上，而不是约束函数的值上。

### Fenchel 对偶与 Lagrange 对偶的关系

Lagrange 对偶是 Fenchel 对偶的特殊情形。把约束 $f_i(x) \leq 0$ 写成指示函数 $g(u) = \delta_{u \leq 0}$，$u = F(x) = (f_1(x),...,f_m(x))$。当 $F$ 仿射时，Fenchel 对偶直接退化为 Lagrange 对偶。

> **本质洞察**：Fenchel 对偶的核心思想是"分裂"——把复杂目标 $f(x) + g(Ax)$ 分成两部分，分别取共轭，然后在对偶空间"组装"。这与 ADMM 的分裂思想完全一致——ADMM 就是 Fenchel 对偶的算法化。

### 练习

1. 推导 group LASSO $\min \frac{1}{2}\|Ax-b\|^2 + \lambda\sum_j\|x_{G_j}\|_2$ 的 Fenchel 对偶。
2. 对约束 LASSO $\min \|x\|_1$ s.t. $\|Ax-b\| \leq \epsilon$，推导其 Fenchel 对偶并解释几何含义。
3. 证明：当 $A = I$（单位矩阵）时，Fenchel 对偶简化为 $d^* = \sup_y [-f^*(-y) - g^*(y)]$。

---

## 2.11 Proximal Gradient 算法 ⭐⭐

### 动机

很多实际问题有"光滑+非光滑"的复合结构：$\min f(x) + g(x)$，其中 $f$ 是 $L$-光滑的（如 $\frac{1}{2}\|Ax-b\|^2$），$g$ 是非光滑但 prox 容易计算的（如 $\lambda\|x\|_1$）。Proximal gradient 方法利用了这种结构——对光滑部分做梯度步，对非光滑部分做 prox 步。

### 算法

**定义**：设 $\min F(x) = f(x) + g(x)$，$f$ 是 $L$-光滑凸，$g$ 是 proper closed convex。

**Proximal gradient 迭代**：
$$x_{k+1} = \text{prox}_{\eta g}(x_k - \eta \nabla f(x_k))$$

其中 $\eta \leq 1/L$（步长不超过光滑部分的逆 Lipschitz 常数）。

**等价形式**（更容易理解）：
$$x_{k+1} = \arg\min_x \left[ g(x) + \frac{1}{2\eta}\left\|x - (x_k - \eta\nabla f(x_k))\right\|^2 \right]$$

**直觉**：先对 $f$ 做一个梯度步得到中间点 $z_k = x_k - \eta\nabla f(x_k)$，然后对 $g$ 做 prox，把中间点"拉"向使 $g$ 小的方向。

**跨领域类比**：proximal gradient 就像"分工合作"——光滑部分由梯度负责（擅长处理光滑），非光滑部分由 prox 负责（擅长处理非光滑）。两者交替执行，各司其职。

### 收敛性定理 ⭐⭐

**定理 2.10.1**：设 $f$ 是 $L$-光滑凸，$g$ 是 proper closed convex，$\eta = 1/L$。则 proximal gradient 满足：

$$F(x_k) - F(x^*) \leq \frac{L\|x_0 - x^*\|^2}{2k}$$

若 $f$ 还是 $\mu$-强凸，则线性收敛：
$$F(x_k) - F(x^*) \leq (1 - \mu/L)^k (F(x_0) - F(x^*))$$

### 证明核心步骤 ⭐⭐

**Step 1**：定义 proximal gradient 的"充分下降引理"。设 $p = \text{prox}_{\eta g}(x - \eta\nabla f(x))$。则：
$$F(p) \leq F(x) - \frac{\eta}{2}\left\|\frac{x - p}{\eta}\right\|^2 + \frac{L\eta - 1}{2}\|x - p\|^2/\eta^2 \cdot \eta$$

当 $\eta = 1/L$ 时简化为 $F(p) \leq F(x) - \frac{1}{2L}\|G_L(x)\|^2$，其中 $G_L(x) = L(x - p)$ 是"广义梯度"。

**Step 2**：利用凸性下界和上面的下降引理，通过 telescoping 得到 $O(1/k)$ 收敛。

**Step 3**：强凸情形用 PL 不等式的推广（$\|G_L(x)\|^2 \geq 2\mu(F(x) - F(x^*))$）得到线性收敛。

### ISTA 与 FISTA ⭐⭐

**ISTA**（Iterative Shrinkage-Thresholding Algorithm）就是 proximal gradient 在 LASSO 上的特化：
$$x_{k+1} = \text{prox}_{\eta\lambda\|\cdot\|_1}(x_k - \eta A^\top(Ax_k - b)) = S_{\eta\lambda}(x_k - \eta A^\top(Ax_k - b))$$

其中 $S_\tau$ 是软阈值算子。

**FISTA**（Beck-Teboulle 2009）加入 Nesterov 动量：
$$y_{k+1} = x_k + \frac{k-1}{k+2}(x_k - x_{k-1})$$
$$x_{k+1} = \text{prox}_{\eta g}(y_{k+1} - \eta\nabla f(y_{k+1}))$$

收敛率从 $O(1/k)$ 加速到 $O(1/k^2)$——与 Nesterov 加速梯度法一致。

**为什么 FISTA 重要**：在 compressed sensing、稀疏恢复、低秩矩阵补全中，FISTA 是标准算法。SLAM 中的 robust estimation（Yang-Carlone 2020 的 GNC）的内层优化也可以用 FISTA 求解。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：ISTA/FISTA 中步长 $\eta$ 的确定**
> - $\eta \leq 1/L$，其中 $L = \lambda_{\max}(A^\top A)$（最大特征值）
> - 计算 $\lambda_{\max}$ 可能很贵。实践中用 backtracking line search 更常见
> - FISTA 的动量系数 $\frac{k-1}{k+2}$ 不能用固定值替代——它的递增性是加速的关键

> 💡 **概念误区：proximal gradient 只适用于 $\ell_1$ 正则化**
> - proximal gradient 适用于任何 $f + g$ 结构（$f$ 光滑，$g$ 有闭式 prox）
> - 包括但不限于：$\ell_1$（软阈值）、group lasso（块软阈值）、核范数（SVT）、指示函数（投影）
> - 只要 $g$ 的 prox 能高效计算，proximal gradient 就能用

### 练习

1. 对 LASSO $\min \frac{1}{2}\|Ax-b\|^2 + \lambda\|x\|_1$，实现 ISTA 和 FISTA，比较收敛速度。
2. 对 group LASSO $\min \frac{1}{2}\|Ax-b\|^2 + \lambda\sum_j \|x_{G_j}\|_2$，写出 proximal gradient 的每步更新公式。
3. 证明 FISTA 的 $O(1/k^2)$ 收敛率（提示：使用 Beck-Teboulle 2009 的 Lyapunov 函数 $t_k^2(F(x_k)-F^*) + \frac{L}{2}\|v_k - x^*\|^2$）。

---

## 2.11 ADMM 推导与收敛 ⭐⭐

### 动机

**交替方向乘子法（ADMM）** 是凸优化中最实用的算法之一。OSQP（MPC 求解器）、SCS（SDP 求解器）、CVXPY 的默认后端——都基于 ADMM。理解 ADMM 的数学原理需要共轭函数和 proximal 算子这两个工具。

### 问题结构

ADMM 求解以下分裂问题：
$$\min_{x, z} \; f(x) + g(z) \quad \text{s.t. } Ax + Bz = c$$

其中 $f$ 和 $g$ 都是 proper closed convex。

**为什么要分裂**：直接最小化 $f(x) + g(Ax)$ 可能很难（如 $f$ 和 $g$ 的 prox 分别容易计算，但合在一起不行）。通过引入辅助变量 $z = Ax$，把问题拆成两个"简单"子问题。

### Augmented Lagrangian

ADMM 基于增广 Lagrangian：
$$L_\rho(x, z, y) = f(x) + g(z) + y^\top(Ax + Bz - c) + \frac{\rho}{2}\|Ax + Bz - c\|^2$$

其中 $y$ 是对偶变量（Lagrange 乘子），$\rho > 0$ 是惩罚参数。

增广 Lagrangian 比普通 Lagrangian 多了二次惩罚项 $\frac{\rho}{2}\|Ax+Bz-c\|^2$。这个惩罚项**不改变最优解**（在最优解处约束 $Ax+Bz=c$ 满足，惩罚项为零），但**改善了收敛性**——它使子问题更容易求解（添加了强凸性）。

### ADMM 迭代

**Step 1**（$x$-更新）：
$$x_{k+1} = \arg\min_x L_\rho(x, z_k, y_k) = \arg\min_x \left[f(x) + \frac{\rho}{2}\|Ax + Bz_k - c + y_k/\rho\|^2\right]$$

**Step 2**（$z$-更新）：
$$z_{k+1} = \arg\min_z L_\rho(x_{k+1}, z, y_k) = \arg\min_z \left[g(z) + \frac{\rho}{2}\|Ax_{k+1} + Bz - c + y_k/\rho\|^2\right]$$

**Step 3**（$y$-更新 / 对偶上升）：
$$y_{k+1} = y_k + \rho(Ax_{k+1} + Bz_{k+1} - c)$$

### 缩放形式（更简洁）

定义 $u = y/\rho$（缩放对偶变量），ADMM 变为：
$$x_{k+1} = \text{prox}_{f/\rho}(z_k - u_k) \quad \text{（$A = I, B = -I, c = 0$ 的简化情形）}$$
$$z_{k+1} = \text{prox}_{g/\rho}(x_{k+1} + u_k)$$
$$u_{k+1} = u_k + (x_{k+1} - z_{k+1})$$

**直觉**：$x$ 步优化 $f$（考虑与 $z$ 的一致性），$z$ 步优化 $g$（考虑与 $x$ 的一致性），$u$ 步累积约束违反量。当 $x_k = z_k$ 时达到收敛。

### 收敛性定理

**定理 2.11.1**：设 $f, g$ 是 proper closed convex，Lagrange 对偶问题有解。则 ADMM 满足：

1. **残差收敛**：原始残差 $\|Ax_k + Bz_k - c\| \to 0$，对偶残差 $\|\rho A^\top B(z_k - z_{k-1})\| \to 0$
2. **目标收敛**：$f(x_k) + g(z_k) \to p^*$
3. **对偶收敛**：$y_k \to y^*$（对偶最优解）

**证明思路**：定义 Lyapunov 函数 $V_k = \frac{1}{\rho}\|y_k - y^*\|^2 + \rho\|Bz_k - Bz^*\|^2$，证明 $V_k$ 单调不增且 $\sum_k r_k^2 < \infty$。

### ADMM 在 OSQP 中的应用 ⭐⭐

OSQP 求解 QP：$\min \frac{1}{2}x^\top Px + q^\top x$ s.t. $l \leq Ax \leq u$。

写成 ADMM 形式：$f(x) = \frac{1}{2}x^\top Px + q^\top x$，$g(z) = \delta_{[l,u]}(z)$，约束 $Ax = z$。

- $x$-更新：解线性系统 $(P + \rho A^\top A)x = -q + \rho A^\top(z-u)$
- $z$-更新：$z = \Pi_{[l,u]}(Ax + u)$（box 投影，$O(m)$）
- $u$-更新：$u \leftarrow u + Ax - z$

**OSQP 的工程优化**：
1. 对 $P + \rho A^\top A$ 做预分解（Cholesky/LDL），$x$-更新只需回代
2. Ruiz 均衡预处理改善条件数
3. 自适应 $\rho$ 调节加速收敛
4. Polishing 阶段提高精度（从 ADMM 的 $O(1/k)$ 提升到机器精度）

### Moreau 分解在 ADMM 中的角色 ⭐⭐⭐

ADMM 的收敛证明的核心步骤依赖 Moreau 分解。考虑 $z$-更新和 $u$-更新的组合：

$$z_{k+1} = \text{prox}_{g/\rho}(x_{k+1} + u_k)$$
$$u_{k+1} = u_k + x_{k+1} - z_{k+1} = (x_{k+1} + u_k) - z_{k+1}$$

由 Moreau 分解：$(x_{k+1} + u_k) = z_{k+1} + u_{k+1}$，即 $z_{k+1} = \text{prox}_{g/\rho}(v)$，$u_{k+1} = \text{prox}_{(g/\rho)^*}(v)$，其中 $v = x_{k+1} + u_k$。

这意味着：$z$-步在原空间做 prox，$u$-步自动在对偶空间做 prox。两者"加起来等于输入"——这就是 ADMM 的"能量守恒"。

**反事实推理**：如果没有 Moreau 分解，我们无法保证 $z$ 和 $u$ 的更新是"协调"的——可能 $z$ 步把变量推向一个方向，$u$ 步推向相反方向，算法就不收敛了。Moreau 分解保证了它们的"推力"加起来恰好等于输入 $v$，不会产生额外的"力"，因此 Lyapunov 函数单调不增。

### ADMM 的参数选择 ⭐⭐

| 参数 | 选择指导 | 对收敛的影响 |
|------|---------|-------------|
| $\rho$ | $\rho \sim \sqrt{\|P\|/\|A^\top A\|}$ | 太大 → 原残差慢；太小 → 对偶残差慢 |
| 自适应 $\rho$ | Boyd 2011 的规则：当 $\|r\|/\|s\| > \mu$，$\rho \leftarrow \tau\rho$ | 平衡原对偶残差 |
| 终止条件 | $\|r_k\| \leq \epsilon_{\text{pri}}$ 且 $\|s_k\| \leq \epsilon_{\text{dual}}$ | 绝对+相对精度 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：ADMM 中 $\rho$ 的影响**
> - $\rho$ 太小：$x$-更新的二次项太弱，子问题"太软"，收敛慢
> - $\rho$ 太大：$x$-更新的二次项主导，$f$ 的信息被淹没，也收敛慢
> - 最佳 $\rho$ 依赖问题结构——自适应调节是最佳实践

> 💡 **概念误区：ADMM 一定比其他方法快**
> - ADMM 的优势是**分裂**——可以把大问题拆成简单子问题
> - 对本身就容易解的问题（如小规模 QP），active-set（qpOASES）或 IPM 可能更快
> - ADMM 的 $O(1/k)$ 收敛率是中等精度的——高精度需要 polishing 或换方法

### ADMM 的变体与扩展 ⭐⭐

**多块 ADMM**：原始 ADMM 把问题分成两块（$x$ 和 $z$）。对 $\min \sum_{i=1}^N f_i(x_i)$ s.t. $\sum_i A_i x_i = b$，可以扩展为 $N$ 块。但注意：$N \geq 3$ 时直接推广的多块 ADMM 不一定收敛（Chen et al. 2016 的反例）。需要加 proximal 项或用 Jacobi-ADMM 等变体。

**Linearized ADMM**：当 $x$-更新中的二次子问题不容易精确求解时（如 $A$ 很大），可以线性化 $\frac{\rho}{2}\|Ax - z + u\|^2$ 中的 $A$ 项：
$$x_{k+1} = \text{prox}_{\tau f}(x_k - \tau A^\top(Ax_k - z_k + u_k))$$

这等价于对 $x$ 做 proximal gradient 步，避免了解大型线性系统。

**Consensus ADMM**：对分布式优化 $\min \sum_i f_i(x)$，引入局部副本 $x_i$：
$$\min \sum_i f_i(x_i) \quad \text{s.t. } x_i = z \; (\forall i)$$

ADMM 的 $x_i$-更新可以在各个节点并行执行，$z$-更新是简单的求平均。这是分布式机器学习中 ADMM 应用的基础。

### 从 Moreau 包络到 ADMM 的完整推导链

让我们把本章的核心概念串联成一条完整的推导链：

1. **共轭函数** $f^*$ 把 $f$ 从点值空间映射到对偶空间
2. **Fenchel-Moreau** $f^{**} = f$ 保证了这个映射可逆
3. **Moreau 包络** $M_f^\lambda$ 光滑化了 $f$（通过 infimal 卷积）
4. **Moreau 分解** $v = \text{prox}_f(v) + \text{prox}_{f^*}(v)$ 建立了原-对偶 prox 的联系
5. **Firmly nonexpansive** 保证了 prox 迭代的收敛性
6. **ADMM** 的每步本质是 prox 步 + Moreau 分解的对偶更新
7. **OSQP** 是 ADMM 在 QP 上的工程化实现

每一步都建立在前一步之上——这就是为什么理解共轭和 proximal 的理论对于理解现代求解器如此重要。

### 练习

1. 对 LASSO $\min \frac{1}{2}\|Ax-b\|^2 + \lambda\|z\|_1$ s.t. $x = z$，写出 ADMM 的完整三步迭代。
2. 解释 OSQP 中 polishing 阶段的原理——如何从 ADMM 的近似解出发，用几步 Newton 迭代达到高精度。
3. 实现一个简单的 ADMM 求解器，解 $\min \|x\|_1$ s.t. $Ax = b$。与 CVXPY 的结果比较。
4. 对 consensus ADMM，推导 $z$-更新为什么是各局部变量 $x_i$ 的平均。

---

## 2.12 Forward-Backward Splitting 与算子分裂 ⭐⭐⭐

### 动机

Proximal gradient 和 ADMM 都是**算子分裂**方法的特例。理解统一的分裂框架能帮助你在面对新问题时选择最合适的算法。

### 单调算子框架

回顾：凸优化 $\min F(x)$ 等价于求解包含 $0 \in \partial F(x)$。

如果 $F = f + g$（$f$ 光滑，$g$ 非光滑），则 $0 \in \nabla f(x) + \partial g(x)$，即：
$$0 \in (A + B)(x) \quad \text{其中 } A = \nabla f, \; B = \partial g$$

这是一个**单调包含**问题（$A$ 是 Lipschitz 单调的，$B$ 是最大单调的）。

### Forward-Backward Splitting（前向-后向分裂）

**迭代**：$x_{k+1} = (I + \gamma B)^{-1}(I - \gamma A)(x_k) = \text{prox}_{\gamma g}(x_k - \gamma \nabla f(x_k))$

这正是 proximal gradient！"前向"步是 $I - \gamma A$（显式梯度），"后向"步是 $(I + \gamma B)^{-1}$（隐式 prox）。

### Douglas-Rachford Splitting ⭐⭐⭐

当两个算子都是"硬的"（都需要 prox 但没有光滑部分）时，forward-backward 退化为 proximal point。Douglas-Rachford 提供了另一种分裂：

$$y_{k+1} = \text{prox}_g(2\text{prox}_f(x_k) - x_k)$$
$$x_{k+1} = x_k + y_{k+1} - \text{prox}_f(x_k)$$

**与 ADMM 的关系**：ADMM 就是 Douglas-Rachford 应用于对偶问题！具体地，对 $\min f(x) + g(z)$ s.t. $x = z$，ADMM 的对偶形式等价于对对偶函数 $-f^*(-y) - g^*(y)$ 做 Douglas-Rachford 分裂。

### 算子分裂方法族谱

```
单调包含 0 in (A + B)(x)
├── Forward-Backward (A Lipschitz, B maximal monotone)
│   ├── Proximal Gradient (A = grad f, B = subdiff g)
│   │   ├── ISTA (g = lam*||x||_1)
│   │   └── FISTA (加速版)
│   └── 投影梯度法 (g = delta_C)
├── Douglas-Rachford (A, B 都是 maximal monotone)
│   └── ADMM (对偶版本)
├── Peaceman-Rachford (Douglas-Rachford 的"全步"版)
└── Proximal Point (A = 0)
    └── 原始-对偶混合梯度法 (PDHG)
```

> **本质洞察**：所有这些算法的收敛性都基于一个统一的数学事实——firmly nonexpansive 映射的不动点迭代收敛（Krasnosel'skii-Mann 定理）。proximal 算子是 firmly nonexpansive 的（Ch 2.8 证明），因此所有基于 prox 的算法都有收敛保证。

### 练习

1. 证明 ADMM 等价于对偶问题上的 Douglas-Rachford 分裂。
2. 对 $\min f(x) + g(x)$（两者都非光滑但 prox 容易计算），比较 proximal point、Douglas-Rachford、ADMM 三种方法的每步计算量和收敛速度。
3. 实现 PDHG（Primal-Dual Hybrid Gradient）方法求解 $\min_x \max_y \langle Ax, y \rangle + f(x) - g^*(y)$。

---

## 2.13 Proximal 算子在机器人学中的应用 ⭐⭐

### MPC 求解器中的 Proximal 算子

OSQP 每步迭代包含一个投影操作 $z = \Pi_{[l,u]}(Ax+u)$——这就是 $\text{prox}_{\delta_{[l,u]}}$。投影的高效性（$O(m)$）是 OSQP 适合嵌入式 MPC 的关键原因。

### Robust SLAM 中的 Moreau 包络

GNC 算法（Yang-Carlone 2020）处理 SLAM 中的外点（outlier）：
1. 把鲁棒核函数（如 truncated LS）写成 Moreau 包络 $M_\rho^\mu(r) = \min_w [\rho(w) + \frac{\mu}{2}(w-r)^2]$
2. 从大 $\mu$（接近 LS，凸）到小 $\mu$（接近原始鲁棒核，非凸）逐步调节
3. 每步求解是凸优化，最终逼近非凸问题的全局最优

### 稀疏控制中的软阈值

在 $\ell_1$ 正则化的模型预测控制（sparse MPC）中，目标函数加入 $\lambda\|u\|_1$ 来鼓励稀疏控制输入（节省执行器能量）。求解器的每步都涉及软阈值操作。

**具体场景**：四足机器人的步态优化中，某些关节在特定相位不需要主动驱动（如摆动腿）。$\ell_1$ 正则化自然地让这些关节的力矩为零，减少了能量消耗。

### 接触力优化中的锥投影

在全身控制（WBC）中，接触力必须在摩擦锥内：$\|f_t\| \leq \mu f_n$。这是一个二阶锥约束。

投影到摩擦锥 $\Pi_{\text{SOC}}$ 有闭式解：
$$\Pi_{\text{SOC}}(f_t, f_n) = \begin{cases} (f_t, f_n) & \|f_t\| \leq \mu f_n \\ (0, 0) & \|f_t\| \leq -f_n/\mu \\ \frac{1}{1+\mu^2}\left(f_t(1-\frac{\mu f_n}{\|f_t\|}), f_n + \mu\|f_t\|\right) & \text{otherwise} \end{cases}$$

这个投影操作在每个 ADMM 迭代的 $z$-步中使用，计算量极小（$O(1)$ per contact）。

### MPC-QP 的 warm-start 策略

在 MPC 中，相邻时间步的 QP 变化很小（只是 horizon 平移了一步）。利用上一步的 ADMM 解作为新问题的初始点（warm-start），通常只需 5-20 次 ADMM 迭代即可达到精度——比冷启动快 5-10 倍。

warm-start 的数学保证来自 proximal 算子的连续性：当问题参数（$q, l, u$）变化很小时，新的最优解和旧的最优解很近，ADMM 从旧解出发只需少量迭代。

### 低秩补全中的 SVT

在视觉 SLAM 的结构恢复中，观测矩阵可能有缺失值。通过核范数最小化 $\min \|X\|_*$ s.t. $\mathcal{P}_\Omega(X) = \mathcal{P}_\Omega(M)$，可以补全缺失值。proximal gradient 的每步需要 SVT 操作。

---

## 2.15 Bregman Proximal 与 Mirror Descent ⭐⭐⭐

### 动机

标准 proximal 算子使用欧氏距离 $\|x-v\|^2$ 作为正则化项。但在某些问题中（如概率单纯形上的优化），欧氏距离不是最自然的"距离"——Bregman 散度可能更合适。

### Bregman 散度

**定义**：设 $\phi$ 是严格凸可微函数。$\phi$ 生成的 Bregman 散度为：
$$D_\phi(x, y) = \phi(x) - \phi(y) - \langle \nabla\phi(y), x - y \rangle$$

**直觉**：$D_\phi(x, y)$ 是 $\phi(x)$ 与其在 $y$ 处的一阶 Taylor 展开之差——"函数值"减去"线性近似值"。由 $\phi$ 严格凸，这个差总是正的。

**常见 Bregman 散度**：

| $\phi(x)$ | $D_\phi(x, y)$ | 名称 |
|-----------|----------------|------|
| $\frac{1}{2}\|x\|^2$ | $\frac{1}{2}\|x-y\|^2$ | 欧氏距离的平方 |
| $\sum x_i \log x_i$ | $\sum x_i \log(x_i/y_i)$ | KL 散度 |
| $-\sum \log x_i$ | $\sum (x_i/y_i - \log(x_i/y_i) - 1)$ | Burg 散度 |

### Bregman Proximal 算子

$$\text{prox}_f^{D_\phi}(v) = \arg\min_x \left[f(x) + D_\phi(x, v)\right]$$

当 $\phi = \frac{1}{2}\|\cdot\|^2$ 时退化为标准 proximal。

### Mirror Descent

Mirror descent（镜像下降）是 Bregman proximal gradient：

$$x_{k+1} = \arg\min_x \left[\langle \nabla f(x_k), x \rangle + \frac{1}{\eta}D_\phi(x, x_k)\right]$$

**为什么叫"镜像"**：等价形式 $\nabla\phi(x_{k+1}) = \nabla\phi(x_k) - \eta\nabla f(x_k)$——先通过 $\nabla\phi$ 映射到"对偶空间"，做梯度步，再通过 $(\nabla\phi)^{-1} = \nabla\phi^*$ 映射回"原空间"。$\nabla\phi$ 就像一面"镜子"，在原空间和对偶空间之间来回映射。

**在单纯形上的优化**：取 $\phi(x) = \sum x_i\log x_i$（负熵），mirror descent 在概率单纯形 $\Delta_n$ 上的更新变为：

$$x_{i,k+1} = \frac{x_{i,k} \exp(-\eta \nabla_i f(x_k))}{\sum_j x_{j,k} \exp(-\eta \nabla_j f(x_k))}$$

这是 exponentiated gradient（指数梯度）——它自然保持概率约束（$x \geq 0$，$\sum x_i = 1$），无需投影！

**与共轭函数的关系**：$\nabla\phi^* = (\nabla\phi)^{-1}$（共轭-次微分互逆）是 mirror descent 的数学基础。选择不同的 $\phi$ 就是选择不同的"镜子"——欧氏空间用 $\frac{1}{2}\|x\|^2$（恒等镜子），单纯形用负熵（softmax 镜子）。

### 练习

1. 验证 KL 散度 $D_{KL}(x\|y) = \sum x_i\log(x_i/y_i)$ 是由 $\phi(x) = \sum x_i\log x_i$ 生成的 Bregman 散度。
2. 推导 mirror descent 在概率单纯形上的指数梯度更新公式。
3. 对 $\min_{x \in \Delta_n} c^\top x$（线性规划在单纯形上），比较投影梯度下降和 mirror descent 的每步复杂度。

---

## 2.16 Proximal 算子的不动点解释 ⭐⭐

### 不动点等价性

**定理**：$x^*$ 是 $\arg\min f$ 当且仅当 $x^* = \text{prox}_{\lambda f}(x^*)$ 对任何 $\lambda > 0$。

**证明**：$x^* = \text{prox}_{\lambda f}(x^*) \Leftrightarrow 0 \in \partial f(x^*) \Leftrightarrow x^* = \arg\min f$。

**直觉**：最优解是 proximal 映射的不动点——在最优解处，"靠近自己"和"降低 $f$"的目标完全一致，proximal 步不会移动。

### 从不动点到算法

不动点等价性直接给出了三族算法：

1. **Proximal Point Algorithm**：$x_{k+1} = \text{prox}_{\lambda f}(x_k)$（直接迭代不动点映射）
2. **Proximal Gradient**：$x_{k+1} = \text{prox}_{\eta g}(x_k - \eta\nabla f(x_k))$（光滑+非光滑分裂）
3. **ADMM**：多个 prox 交替迭代

所有这些算法的收敛性都可以通过不动点理论的统一框架来证明：映射的 nonexpansive/averaged 性质 → Krasnosel'skii-Mann 定理 → 收敛到不动点 = 最优解。

### 与控制理论的联系

在控制论中，不动点对应系统的平衡点。proximal 算子的迭代 $x_{k+1} = T(x_k)$ 可以看作一个离散动力系统，不动点就是系统的稳态。firmly nonexpansive 性质保证了系统是全局渐近稳定的——从任意初始状态出发都会收敛到平衡点。

这个类比解释了为什么 proximal 方法在控制工程中如此受欢迎——它们提供了与控制系统稳定性分析相同味道的收敛保证。

---

## 2.17 Proximal 算子的高级性质 ⭐⭐⭐

### Moreau 恒等式

**定理**：对 proper closed convex $f$ 和 $\lambda > 0$：

$$\frac{1}{2}\|v\|^2 = M_f^\lambda(v) + M_{f^*}^{1/\lambda}\left(\frac{v}{\lambda}\right) \cdot \lambda$$

更精确地写：
$$\frac{1}{2}\|v\|^2 = \left[f(\text{prox}_{\lambda f}(v)) + \frac{1}{2\lambda}\|\text{prox}_{\lambda f}(v) - v\|^2\right] + \lambda\left[f^*\left(\frac{v - \text{prox}_{\lambda f}(v)}{\lambda}\right) + \frac{\lambda}{2}\left\|\frac{v-\text{prox}_{\lambda f}(v)}{\lambda} - \frac{v}{\lambda}\right\|^2\right]$$

这个恒等式把 $\|v\|^2/2$ 分解为两个 Moreau 包络的和——一个在原空间，一个在对偶空间。

### Proximal 算子的连续性

**定理**：$\text{prox}_f$ 是连续映射。

**证明**：由 firmly nonexpansive：$\|\text{prox}_f(v_1) - \text{prox}_f(v_2)\| \leq \|v_1 - v_2\|$，这比连续性更强——它是 Lipschitz 连续的（Lipschitz 常数 $\leq 1$）。

### Proximal 算子的微分性质

**定理**：若 $f$ 二次可微且 $\nabla^2 f(x) \succ 0$，则 $\text{prox}_f$ 在 $v$ 处可微，且：

$$\nabla \text{prox}_f(v) = (I + \nabla^2 f(\text{prox}_f(v)))^{-1}$$

**推论**：$\nabla \text{prox}_f(v)$ 的特征值在 $[0, 1]$ 中——这是 firmly nonexpansive 的微分版本。

**证明**：由隐函数定理。$p(v) = \text{prox}_f(v)$ 满足 $v - p \in \partial f(p)$，即 $v = p + \nabla f(p)$（当 $f$ 可微时）。对 $v$ 求微分：$I = \nabla p + \nabla^2 f(p) \cdot \nabla p = (I + \nabla^2 f(p)) \nabla p$，故 $\nabla p = (I + \nabla^2 f(p))^{-1}$。

### Proximal 算子与距离函数

**定理**：$\text{dist}(v, \text{lev}_\alpha f)^2 \leq 2\lambda(M_f^\lambda(v) - \alpha)$

其中 $\text{lev}_\alpha f = \{x : f(x) \leq \alpha\}$ 是 $f$ 的 $\alpha$-下水平集。

**含义**：Moreau 包络值越接近 $\alpha$，$v$ 越接近 $\alpha$-下水平集。这为 proximal 算法提供了距离下水平集的估计。

### 非凸 Proximal 的推广 ⭐⭐⭐⭐

在非凸优化中，proximal 算子仍然有定义：$\text{prox}_f(v) = \arg\min_x [f(x) + \frac{1}{2}\|x-v\|^2]$。但：

1. 解可能不唯一（不再是单值映射）
2. 不再是 firmly nonexpansive
3. 算法收敛性需要额外条件（如 KL 性质）

**KL（Kurdyka-Lojasiewicz）性质**：若 $f$ 满足 $\|\partial f(x)\| \geq \phi(f(x) - f(x^*))$（某个递减函数 $\phi$），则非凸 proximal gradient 仍然收敛。很多实际函数（如深度学习中的损失函数）满足 KL 性质。

**在机器人学中的意义**：SLAM 后端的非凸优化（pose graph）、轨迹优化、RL 的策略优化——都是非凸问题。KL proximal 理论为这些问题的算法收敛性提供了理论保证（虽然只保证收敛到临界点，不保证全局最优）。

### 练习

1. 验证 Moreau 恒等式：对 $f(x) = |x|$，$\lambda = 1$，$v = 2$，计算等式两边并验证相等。
2. 对 $f(x) = \frac{1}{2}x^\top Qx$（$Q \succ 0$），计算 $\nabla\text{prox}_f(v)$ 并验证特征值在 $[0,1]$ 中。
3. 给出一个不满足 KL 性质的非凸函数例子。（提示：考虑在最小值附近非常"平坦"的函数。）

---

## 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| "共轭的共轭一定回到原函数" | $f^{**} = f$ 仅当 $f$ 是 proper + closed + convex 时成立。非凸 $f$ 的 $f^{**}$ 只是 $f$ 的闭凸包 |
| "共轭函数和 proximal 算子是一回事" | 共轭 $f^*$ 是函数变换（输入函数，输出函数），proximal 是点映射（输入点，输出点）。二者通过 Moreau 分解联系但概念不同 |
| "PPO 的 proximal 就是 Moreau 意义的 prox" | PPO 的"proximal"来自 KL trust region / clipped surrogate，与 Moreau-Yosida proximal 算子没有直接数学关系 |
| "范数和范数平方的 prox 差不多" | $\text{prox}_{\lambda\|\cdot\|_2}$ 是块软阈值（有死区），$\text{prox}_{\lambda\|\cdot\|^2/2}$ 是简单缩放（无死区）。混淆两者是 ADMM 实现中最常见的 bug |
| "prox 没有闭式解就不能用近端方法" | 很多没有闭式 prox 的函数可以用 Newton 法在 3-5 步内求解 prox 子问题（因为子问题强凸）。关键是 prox 代价相对于梯度代价的比值 |
| "强凸和光滑是两个独立概念" | 共轭变换揭示了深刻的对偶关系：$f$ $\mu$-强凸 $\Leftrightarrow$ $f^*$ $1/\mu$-光滑。条件数在共轭下不变 |
| "Moreau 包络改变了最优解" | Moreau 包络 $M_f^\lambda$ 与原函数 $f$ 有相同的最小值点和最小值。它只是把函数"光滑化"了，没有改变优化的目标 |
| "ADMM 是凭空发明的算法" | ADMM 等价于 Douglas-Rachford splitting 作用在对偶问题上（Eckstein-Bertsekas 1992）。理解这个等价性是理解 ADMM 收敛的关键 |

---

## 本章小结

| 概念 | 核心陈述 | 难度 | 关键应用 |
|------|---------|------|---------|
| Legendre-Fenchel 共轭 | $f^*(y) = \sup_x[\langle y,x\rangle - f(x)]$ | ⭐⭐ | 对偶理论基础 |
| Fenchel-Young 不等式 | $f(x)+f^*(y) \geq \langle x,y\rangle$ | ⭐⭐ | 弱对偶的根源 |
| Fenchel-Moreau 定理 | $f^{**}=f$ 对闭凸函数 | ⭐⭐ | 对偶可逆性 |
| 共轭-次微分互逆 | $y \in \partial f(x) \Leftrightarrow x \in \partial f^*(y)$ | ⭐⭐ | Mirror descent |
| 强凸-光滑对偶 | $\mu$-强凸 $\Leftrightarrow$ $1/\mu$-光滑共轭 | ⭐⭐ | 对偶加速 |
| Proximal 算子 | $\text{prox}_f(v) = \arg\min[f(x)+\frac{1}{2}\|x-v\|^2]$ | ⭐⭐ | ADMM/splitting 核心 |
| Moreau 包络 | 光滑化 + 梯度 = $(v-\text{prox})/\lambda$ | ⭐⭐ | GNC/Nesterov smoothing |
| Moreau 分解 | $v = \text{prox}_f(v) + \text{prox}_{f^*}(v)$ | ⭐⭐ | Prox 互推工具 |
| Firmly nonexpansive | $\|\text{prox}(x)-\text{prox}(y)\|^2 \leq \langle x-y, \text{prox}(x)-\text{prox}(y)\rangle$ | ⭐⭐⭐ | 算法收敛保证 |
| Fenchel 对偶 | $p^* = \inf[f(x)+g(Ax)]$, $d^* = \sup[-f^*(-A^\top y)-g^*(y)]$ | ⭐⭐ | LASSO 对偶 |
| Proximal gradient | $x_{k+1} = \text{prox}_{\eta g}(x_k - \eta\nabla f(x_k))$ | ⭐⭐ | ISTA/FISTA |
| ADMM | $x$-更新 + $z$-更新 + $u$-更新 | ⭐⭐ | OSQP/SCS |
| Forward-Backward | 光滑前向步 + 非光滑后向步 | ⭐⭐⭐ | 统一分裂框架 |
| Douglas-Rachford | 两个 prox 的交替组合 | ⭐⭐⭐ | ADMM 的对偶形式 |
| Bregman proximal | 用 Bregman 散度替代欧氏距离 | ⭐⭐⭐ | Mirror descent |
| Prox 不动点 | $x^* = \arg\min f \Leftrightarrow x^* = \text{prox}_{\lambda f}(x^*)$ | ⭐⭐ | 算法设计出发点 |

---

## 本章知识地图

```
共轭函数与 Proximal 算子
├── 共轭函数理论 (§2.1-2.3, 2.9-2.10)
│   ├── Legendre-Fenchel 变换 → 点值表示 ↔ 切线表示
│   ├── Fenchel-Young 不等式 → 弱对偶的根源
│   ├── Fenchel-Moreau 定理 → f** = f (闭凸)
│   ├── 共轭-次微分互逆 → Mirror descent
│   ├── 强凸-光滑对偶 → 条件数不变
│   ├── 共轭计算实例大全 → 速查表
│   └── Fenchel 对偶 → LASSO 对偶 / 对偶证书
├── Proximal 算子 (§2.4-2.5, 2.16)
│   ├── 定义 → 隐式梯度步 = 次微分 resolvent
│   ├── 存在唯一性 → 强凸子问题
│   ├── 代数运算规则 → 可分/仿射/正交
│   ├── 5 个核心闭式 → 软阈值/块阈值/投影/二次/SVT
│   └── 不动点解释 → 算法设计出发点
├── Moreau 理论 (§2.6-2.7)
│   ├── Moreau 包络 → 通用光滑化 / GNC
│   ├── 梯度公式 $\nabla M_f^\lambda(v) = (v - \text{prox}_{\lambda f}(v))/\lambda$
│   ├── Moreau 分解 $\text{prox}_f(v) + \text{prox}_{f^*}(v) = v$
│   └── Infimal 卷积 $(f \square g)^* = f^* + g^*$
├── 收敛理论 (§2.8)
│   ├── Firmly nonexpansive → prox 的核心性质
│   ├── Averaged operator → Krasnosel'skii-Mann
│   └── PPA 收敛 → 所有 proximal 算法的母定理
├── 算法 (§2.11-2.13)
│   ├── Proximal gradient → ISTA / FISTA
│   ├── ADMM → OSQP / SCS
│   ├── Forward-Backward → 统一分裂框架
│   └── Douglas-Rachford → ADMM 的对偶形式
└── 推广 (§2.15)
    └── Bregman proximal → Mirror descent → 单纯形优化
```

---

## 跨章综合练习

> 📝 **综合练习**（需要本章 + 凸分析基础的知识）

1. **从分离定理到 Fenchel-Moreau**：回顾 Ch1 的分离超平面定理。解释为什么闭凸函数的 epigraph 是闭凸集，以及分离定理如何给出 $f^{**} \geq f$ 的关键步骤。

2. **从强凸性到 ADMM 收敛**：设 $f$ 是 $\mu$-强凸的。
   (a) 计算 $\text{prox}_{\lambda f}$ 的 contraction factor（收缩因子）。
   (b) 说明为什么 ADMM 对强凸的 $f$ 有线性收敛率。
   (c) 如果 $f$ 只是凸（不强凸），收敛率变成什么？

3. **从条件数到算法选择**：对 $\min \frac{1}{2}\|Ax-b\|^2 + \lambda\|x\|_1$（LASSO）：
   (a) 计算光滑部分的 $L$（$L = \|A^\top A\|_2$）。
   (b) 如果 elastic net $+ \frac{\mu}{2}\|x\|^2$，新问题的条件数是什么？
   (c) 用条件数解释为什么 elastic net 的 ISTA 比纯 LASSO 的 ISTA 收敛更快。

4. **Moreau 包络的全链路验证**：对 $f(x) = \|x\|_1$：
   (a) 计算 $\text{prox}_{\lambda f}(v)$（软阈值）。
   (b) 计算 $M_f^\lambda(v)$（Moreau 包络的闭式）。
   (c) 验证 $\nabla M_f^\lambda(v) = \frac{1}{\lambda}(v - \text{prox}_{\lambda f}(v))$。
   (d) 验证 $\nabla M_f^\lambda$ 的 Lipschitz 常数是 $\frac{1}{\lambda}$。
   (e) 用 Moreau 分解验证 $\text{prox}_{\lambda\|\cdot\|_\infty^*}(v)$。

---

## 向后指向：从共轭/proximal 到对偶理论

| 本章概念 | 下一章（对偶理论）的应用 |
|---------|---------------------|
| Fenchel-Young 不等式 | 弱对偶 $g(\lambda) \leq p^*$ 的直接来源 |
| $f^{**} = f$ (闭凸函数) | 强对偶 = 对偶可逆性 |
| 强凸-光滑对偶 | 对偶问题的条件数分析 |
| Moreau 包络 | ADMM / augmented Lagrangian 的光滑化 |
| Proximal 算子 | ADMM 每步子问题的求解 |
| Firmly nonexpansive | ADMM 收敛保证的数学基石 |
| Infimal 卷积 | Fenchel 对偶的标准形式 |

---

## 累积项目：本章新增模块

**项目：Proximal 算子库与优化算法实现**

本章新增：
- 实现5个核心 prox 的闭式解（软阈值、块软阈值、投影、二次、SVT）
- 实现 Moreau 包络及其梯度计算
- 实现 proximal gradient 算法（ISTA）求解 LASSO
- 实现 FISTA（加速版本），比较收敛速度
- 实现简易 ADMM 求解 $\min f(x) + g(z)$ s.t. $x = z$

```python
import numpy as np

def prox_l1(v, lam):
    """L1 范数的 proximal: 软阈值"""
    return np.sign(v) * np.maximum(np.abs(v) - lam, 0)

def prox_l2(v, lam):
    """L2 范数的 proximal: 块软阈值"""
    norm_v = np.linalg.norm(v)
    if norm_v <= lam:
        return np.zeros_like(v)
    return (1 - lam / norm_v) * v

def moreau_envelope(f, prox_f, v, lam):
    """计算 Moreau 包络值"""
    p = prox_f(v, lam)
    return f(p) + 0.5 / lam * np.linalg.norm(p - v)**2

def moreau_gradient(prox_f, v, lam):
    """Moreau 包络的梯度"""
    p = prox_f(v, lam)
    return (v - p) / lam

def prox_nuclear(V, lam):
    """核范数的 proximal: 奇异值阈值 (SVT)"""
    U, s, Vt = np.linalg.svd(V, full_matrices=False)
    s_thresh = np.maximum(s - lam, 0)
    return U @ np.diag(s_thresh) @ Vt

def ista(A, b, lam, x0, n_iter=100):
    """ISTA 求解 LASSO: min 0.5||Ax-b||^2 + lam*||x||_1"""
    L = np.linalg.norm(A.T @ A, 2)  # 最大特征值
    eta = 1.0 / L
    x = x0.copy()
    history = []
    for k in range(n_iter):
        grad = A.T @ (A @ x - b)
        x = prox_l1(x - eta * grad, eta * lam)
        obj = 0.5 * np.linalg.norm(A @ x - b)**2 + lam * np.linalg.norm(x, 1)
        history.append(obj)
    return x, history

def fista(A, b, lam, x0, n_iter=100):
    """FISTA: 加速版 ISTA"""
    L = np.linalg.norm(A.T @ A, 2)
    eta = 1.0 / L
    x = x0.copy()
    y = x0.copy()
    t = 1.0
    history = []
    for k in range(n_iter):
        grad = A.T @ (A @ y - b)
        x_new = prox_l1(y - eta * grad, eta * lam)
        t_new = (1 + np.sqrt(1 + 4 * t**2)) / 2
        y = x_new + (t - 1) / t_new * (x_new - x)
        x = x_new
        t = t_new
        obj = 0.5 * np.linalg.norm(A @ x - b)**2 + lam * np.linalg.norm(x, 1)
        history.append(obj)
    return x, history

def admm_lasso(A, b, lam, rho=1.0, n_iter=100):
    """ADMM 求解 LASSO: min 0.5||Ax-b||^2 + lam*||z||_1 s.t. x=z"""
    n = A.shape[1]
    x = np.zeros(n)
    z = np.zeros(n)
    u = np.zeros(n)
    # 预计算 (A'A + rho*I)^{-1}
    ATA = A.T @ A
    AtA_rhoI_inv = np.linalg.inv(ATA + rho * np.eye(n))
    Atb = A.T @ b
    history = []
    for k in range(n_iter):
        # x-更新: (A'A + rho*I) x = A'b + rho*(z - u)
        x = AtA_rhoI_inv @ (Atb + rho * (z - u))
        # z-更新: 软阈值
        z = prox_l1(x + u, lam / rho)
        # u-更新: 对偶上升
        u = u + x - z
        obj = 0.5 * np.linalg.norm(A @ x - b)**2 + lam * np.linalg.norm(z, 1)
        history.append(obj)
    return x, z, history
```

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|------|------|
| Parikh & Boyd《Proximal Algorithms》(2014) | ⭐⭐⭐ | Proximal 算法百科，含 prox 表 |
| Beck《First-Order Methods》Ch 4, 6 | ⭐⭐⭐ | 最现代算法导向的自学教材 |
| Bauschke & Combettes Ch 12-13, 23 | ⭐⭐⭐⭐ | Moreau/prox 最权威理论参考 |
| Rockafellar《Convex Analysis》§12, §23-26 | ⭐⭐⭐⭐⭐ | 共轭理论圣经 |
| Combettes & Pesquet "Proximal Splitting" | ⭐⭐⭐ | Splitting 方法族谱 |
| Boyd-Parikh-Chu "ADMM" (2011) | ⭐⭐⭐ | ADMM 工程圣经，5000+ 引用 |
| Moreau "Proximité et dualité" (1965) | ⭐⭐⭐⭐⭐ | proximal 算子的原始论文 |
| Ryu & Boyd "Primer on Monotone Operator Methods" | ⭐⭐⭐ | 算子分裂的现代入门 |
| Condat "Fast Projection onto the Simplex" (2016) | ⭐⭐ | 单纯形投影的 O(n) 算法 |
| Yang & Carlone "GNC" (ICRA 2020) | ⭐⭐⭐⭐ | Moreau 包络在鲁棒 SLAM 中的应用 |
| Nesterov "Smooth Minimization" (2005) | ⭐⭐⭐ | Nesterov 光滑化 = 对偶空间 Moreau |
| Bubeck "Convex Optimization" (2015) | ⭐⭐⭐ | Mirror descent 的最清晰推导 |
| Amari "Information Geometry" (2016) | ⭐⭐⭐⭐ | 共轭函数在信息几何中的角色 |
| Boyd & Vandenberghe Ch 3.3 | ⭐⭐ | 共轭函数的工程入门 |

---

## 共轭函数与 Proximal 算子的历史演进 ⭐

| 年代 | 里程碑 | 贡献者 |
|------|--------|--------|
| 1788 | Lagrange 力学中的 Legendre 变换 | Legendre |
| 1949 | Legendre-Fenchel 共轭推广到非光滑函数 | Fenchel |
| 1951 | Fenchel 对偶理论 | Fenchel |
| 1962 | Proximity operator（近端映射）定义 | Moreau |
| 1965 | Moreau 分解定理 | Moreau |
| 1970 | 《Convex Analysis》系统化共轭理论 | Rockafellar |
| 1976 | Proximal point algorithm | Rockafellar |
| 1979 | Douglas-Rachford splitting | Lions & Mercier |
| 1992 | ADMM 的现代形式 | Glowinski & Le Tallec |
| 2005 | Nesterov 光滑化方法 | Nesterov |
| 2009 | FISTA（加速 proximal gradient） | Beck & Teboulle |
| 2011 | ADMM 工程综述（5000+ 引用） | Boyd, Parikh et al. |
| 2014 | Proximal Algorithms 综述 | Parikh & Boyd |
| 2017 | OSQP 发布（ADMM-based QP solver） | Stellato et al. |
| 2020 | GNC 用 Moreau 包络做鲁棒 SLAM | Yang & Carlone |

**历史启示**：从 Moreau (1962) 定义 proximal 算子到 OSQP (2017) 成为 MPC 标准求解器，经历了半个多世纪。关键转折点是 Boyd 团队的工程化工作——ADMM 综述 (2011) 和 OSQP (2017) 把抽象的算子理论变成了嵌入式控制器上的实时代码。

**两个学派的融合**：法国学派（Moreau, Lions, Combettes）发展了 proximal/splitting 的数学理论；美国学派（Boyd, Parikh）把它变成了工程工具。本章试图把两者结合——用工程直觉引入概念，用法国学派的严格性证明。

---

## 共轭/Proximal 核心公式速查 ⭐⭐

以下是本章最重要的公式，按使用频率排序：

### 必记的 5 个公式

**1. Fenchel-Young 不等式**：
$$f(x) + f^*(y) \geq \langle x, y \rangle, \quad \text{等号} \Leftrightarrow y \in \partial f(x)$$

**2. Moreau 分解**：
$$v = \text{prox}_f(v) + \text{prox}_{f^*}(v)$$

**3. Proximal resolvent**：
$$p = \text{prox}_f(v) \quad \Leftrightarrow \quad v - p \in \partial f(p)$$

**4. Moreau 包络梯度**：
$$\nabla M_f^\lambda(v) = \frac{1}{\lambda}(v - \text{prox}_{\lambda f}(v))$$

**5. 强凸-光滑对偶**：
$$f \text{ 是 } \mu\text{-强凸} \quad \Leftrightarrow \quad f^* \text{ 是 } \frac{1}{\mu}\text{-光滑}$$

### 必记的 3 个 prox 闭式

**软阈值**：$\text{prox}_{\lambda\|\cdot\|_1}(v) = \text{sign}(v) \odot \max(|v|-\lambda, 0)$

**块软阈值**：$\text{prox}_{\lambda\|\cdot\|_2}(v) = (1-\lambda/\|v\|)_+ \cdot v$

**投影**：$\text{prox}_{\delta_C}(v) = \Pi_C(v)$

### 必记的 3 个共轭

$(\frac{1}{2}\|x\|^2)^* = \frac{1}{2}\|y\|^2$ （自共轭）

$(\|\cdot\|_p)^* = \delta_{\|\cdot\|_q \leq 1}$ （$\frac{1}{p}+\frac{1}{q}=1$）

$(\delta_C)^* = \sigma_C$ （指示 ↔ 支撑）

### 核心算法

**Proximal Gradient**：$x_{k+1} = \text{prox}_{\eta g}(x_k - \eta\nabla f(x_k))$ ，收敛率 $O(1/k)$

**FISTA**：加 Nesterov 动量 → $O(1/k^2)$

**ADMM**：$x$-步 (prox $f$) → $z$-步 (prox $g$) → $u$-步 (对偶上升)

### 算法选择决策指南

```
你的问题是什么结构？
├── min f(x)，f 光滑凸
│   └── 梯度下降 / Nesterov 加速（Ch1）
├── min f(x) + g(x)，f 光滑、g 有闭式 prox
│   ├── 精度要求中等 → ISTA / proximal gradient
│   └── 需要加速 → FISTA
├── min f(x) + g(Ax)，f/g 都有闭式 prox
│   └── ADMM
├── min f(x) s.t. Ax = b，f 有闭式 prox
│   └── ADMM（引入辅助变量）
├── min f(x) + g(x)，两者都非光滑、都有 prox
│   └── Douglas-Rachford splitting
└── 概率单纯形上的优化
    └── Mirror descent（Bregman proximal）
```

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| ADMM 不收敛 | prox 计算有误 | 1. 验证 prox 公式正确 2. 检查 $\lambda$ 参数位置 3. 验证 firmly nonexpansive | §2.5, §2.8 |
| ADMM 收敛很慢 | $\rho$ 选择不当 | 1. 尝试自适应 $\rho$ 2. 检查原/对偶残差的比值 3. 预处理问题矩阵 | §2.11 |
| Proximal gradient 振荡 | 步长太大 | 1. 确认 $\eta \leq 1/L$ 2. 尝试 backtracking 3. 检查光滑部分的 $L$ 估计 | §2.10 |
| Proximal gradient 不下降 | 梯度或 prox 实现有误 | 1. 用数值差分验证梯度 2. 验证 prox 满足 resolvent 关系 3. 检查目标函数定义 | §2.4, §2.10 |
| FISTA 比 ISTA 慢 | 动量参数错误 | 1. 检查 $t_k$ 的更新公式 2. 确认 $\beta_k = (t_k-1)/t_{k+1}$（不是固定值）3. 检查是否重启了 | §2.10 |
| 对偶间隙不收到零 | Fenchel 对偶条件不满足 | 1. 检查 ri(dom f) 和 ri(dom g) 的交集 2. 验证 Slater 条件 3. 检查函数是否真的闭凸 | §2.2 |
| LASSO 解不稀疏 | $\lambda$ 太小或 prox 实现有误 | 1. 增大 $\lambda$ 2. 验证软阈值实现 3. 检查收敛判据是否达到 | §2.5.1 |
| Moreau 光滑化后梯度不连续 | $\lambda$ 设置或 prox 实现有误 | 1. Moreau 包络的梯度应该是 $1/\lambda$-Lipschitz 的 2. 数值验证梯度连续性 3. 检查 prox 是否用对了 | §2.6 |
| 共轭计算结果与预期不符 | 求导时遗漏了定义域约束 | 1. 检查 sup 是否在 dom(f) 上取 2. 检查极值点是否在定义域内 3. 对边界情况单独讨论 | §2.1 |
| SVT 结果不是低秩的 | $\lambda$ 太小 | 1. 增大 $\lambda$ 2. 检查 SVD 计算精度 3. 对大矩阵用 truncated SVD | §2.5.5 |

### 调试 prox 实现的标准流程

1. **验证不动点**：$x^* = \arg\min f \Leftrightarrow x^* = \text{prox}_{\lambda f}(x^*)$。代入你的实现检查。
2. **验证 resolvent**：检查 $v - \text{prox}_f(v) \in \partial f(\text{prox}_f(v))$。
3. **验证 Moreau 分解**：检查 $v = \text{prox}_f(v) + \text{prox}_{f^*}(v)$。
4. **验证非扩张性**：对随机点对检查 $\|\text{prox}(x) - \text{prox}(y)\| \leq \|x-y\|$。
5. **与 CVXPY 对比**：用 CVXPY 求解 $\min f(x) + \frac{1}{2}\|x-v\|^2$，与你的 prox 输出比较。

### 常见 ADMM 实现错误检查清单

| 检查项 | 正确行为 | 常见错误 |
|--------|---------|---------|
| 原残差 $r_k = Ax_k + Bz_k - c$ | 单调递减趋于 0 | 不减→ $\rho$ 太大或 $x$-更新有误 |
| 对偶残差 $s_k = \rho A^\top B(z_k - z_{k-1})$ | 单调递减趋于 0 | 不减→ $\rho$ 太小或 $z$-更新有误 |
| 目标值 | 单调不增（大致） | 振荡→ prox 实现有误 |
| $x_k - z_k$ | 趋于 0（consensus） | 不趋于 0 → 约束未满足 |
| $u_k$ | 收敛到有限值 | 发散 → 原问题不可行 |
| $(P + \rho A^\top A)$ 的 Cholesky | 成功 | 失败 → $P$ 不 PSD 或 $\rho$ 太小 |

### ADMM 收敛速度的理论保证

| $f, g$ 的性质 | 收敛率 | 改善方法 |
|-------------|--------|---------|
| 一般凸 | $O(1/k)$ | 自适应 $\rho$，polishing |
| $f$ 或 $g$ 强凸 | $O(1/k)$ 但常数更小 | 利用强凸性加速 |
| $f, g$ 都强凸 | 线性 $O((1-c)^k)$ | 理论最佳 |
| $f$ 二次 + $g$ polyhedral | 有限步 | 等价于 active-set |

**Nishihara et al. (2015)** 证明了：对于 $f$ 强凸 + $g$ 也强凸的情况，ADMM 的线性收敛率可以精确刻画为 $\sqrt{1 - 1/(1 + \kappa)}$，其中 $\kappa$ 是条件数相关量。

### 共轭函数的信息论视角 ⭐⭐⭐

共轭函数在信息论中有天然的对应关系：

| 信息论概念 | 共轭函数表述 |
|---------|------------|
| 负熵 $\sum p_i\log p_i$ | log-sum-exp 的共轭 |
| Shannon 熵 $H(p)$ | $H^*(y)$：log-sum-exp 的变体 |
| 交叉熵损失 | 与负熵的共轭关系 |
| KL 散度 | Bregman 散度（$\phi = $ 负熵） |

**log-sum-exp 和 softmax 的对偶**：

log-sum-exp $f(x) = \log\sum e^{x_i}$ 是概率论中 "自由能" 的离散版本。它的梯度是 softmax $\nabla f(x) = \text{softmax}(x)$。

其共轭是负熵（限制在概率单纯形上）：$f^*(p) = \sum p_i \log p_i$（$p \in \Delta_n$）。

共轭的梯度：$\nabla f^*(p) = (\log p_1 + 1, ..., \log p_n + 1)$，即 $(\nabla f)^{-1} = \nabla f^* \Leftrightarrow \text{softmax}^{-1}(p) = \log p + \text{const}$。

这解释了为什么 softmax 和 log-softmax 在深度学习中如此自然——它们是一对共轭函数的梯度。

### Proximal 算子在深度学习中的应用 ⭐⭐

虽然深度学习主要使用（随机）梯度下降，但 proximal 方法在以下场景中发挥重要作用：

1. **稀疏训练**：proximal gradient 用于训练稀疏神经网络（$\ell_1$ 权重正则化）
2. **约束学习**：投影梯度（proximal with $\delta_C$）保持权重在约束集内（如谱范数约束 → $\|W\|_2 \leq 1$）
3. **对偶方法**：augmented Lagrangian / ADMM 用于分布式训练
4. **Natural gradient**：mirror descent（Bregman proximal）在 Fisher 信息度量下做策略优化

**反事实推理**：如果没有 proximal 理论，深度学习中的这些技术就缺乏理论保证。例如，为什么 $\ell_1$ 正则化训练能产生稀疏网络？答案是 proximal gradient（ISTA）的收敛性 + 软阈值的稀疏性质——两者都是本章建立的理论。

### 符号表

| 符号 | 含义 | 首次出现 |
|------|------|---------|
| $f^*(y)$ | $f$ 的 Legendre-Fenchel 共轭函数 | §2.1 |
| $\partial f(x)$ | $f$ 在 $x$ 处的次微分（次梯度集合） | §前置自测 |
| $\sigma_C(y)$ | 凸集 $C$ 的支撑函数 $\sup_{x \in C}\langle y, x \rangle$ | §前置自测 |
| $\delta_C(x)$ | 凸集 $C$ 的指示函数（$x \in C$ 时为 $0$，否则 $+\infty$） | §2.1（例 3） |
| $\operatorname{prox}_f(v)$ | $f$ 的 proximal 算子 $\arg\min_x [f(x) + \frac{1}{2}\|x-v\|^2]$ | §2.4 |
| $M_f^\lambda(v)$ | $f$ 的 Moreau 包络（Moreau-Yosida 正则化） | §2.6 |
| $\mu$ | 强凸参数 | §2.3 |
| $L$ | Lipschitz 光滑参数 | §2.3 |
| $\kappa$ | 条件数 $\kappa = L / \mu$ | §2.3 |
| $\mathcal{S}_\lambda(\cdot)$ | 软阈值算子 $\operatorname{prox}_{\lambda\|\cdot\|_1}$ | §2.5 |
| $f^{**}$ | $f$ 的双共轭（二次 Legendre-Fenchel 变换） | §2.2 |
| $\rho$ | ADMM 的惩罚参数 | §ADMM 收敛诊断 |

---
