## 博士前数学路线图·第五批·子专题 E：Certifiable Perception 与 SDP 松弛——SE-Sync、TEASER++ 与全局最优证书

### 前置自测

> 答不出 2 题以上，建议先回 B（因子图优化）和 C（iSAM2/增量优化）复习。

1. Gauss-Newton 方法中 $J^\top J\delta=-J^\top r$ 的正规方程从哪里来？它近似了什么？
2. 半正定矩阵 $X\succeq 0$ 的定义是什么？它等价于什么特征值条件？
3. SLAM 中 pose graph optimization 的优化变量和约束分别是什么？为什么它是非凸的？
4. 什么是 Lagrange 对偶？强对偶和弱对偶的区别？Slater 条件的作用？
5. $SO(3)$ 上的 $R^\top R=I$ 约束为什么让优化变成 QCQP？

---

### 本章目标

学完本章后，你应当能够：

1. 解释为什么局部方法（GN/LM/iSAM2）对 SLAM/感知问题不能保证全局最优，给出具体反例
2. 写出 rotation averaging 或 PGO 问题的 QCQP 形式，推导其 Shor SDP 松弛
3. 理解对偶证书 $S=C-\sum\lambda_i A_i\succeq 0$ 如何验证全局最优性，解释"认证"的充分性逻辑
4. 描述 Burer-Monteiro 分解如何把 $O(n^2)$ SDP 变量降为 $O(np)$，理解 Riemannian Staircase
5. 区分 SE-Sync（PGO）和 TEASER++（点云配准）的问题形式和求解策略差异

---

### 知识树总览

```text
Certifiable Perception 与 SDP 松弛
├── 动机：局部方法的根本局限（§E.1）
│   ├── 非凸性来源：SO(d) 流形约束
│   ├── Carlone-Censi 2^n 局部极小定理
│   └── 工业补救（随机重启）的理论困境
├── 理论工具链
│   ├── SDP 基础（§E.3）
│   │   ├── 标准形式 / 对偶 / KKT / Slater
│   │   └── 内点法复杂度
│   ├── QCQP → Shor 松弛（§E.4）
│   │   ├── 变量提升 Z=xx^T → Z≽0, rank(Z)=1
│   │   └── 松弛紧 ⟺ rank-d 最优解
│   ├── 对偶证书（§E.5）
│   │   └── S≽0 ⟹ 当前解全局最优
│   └── Burer-Monteiro 分解（§E.6）
│       ├── Z=YY^T 降维
│       ├── Stiefel 流形 + Riemannian trust-region
│       └── BVB 定理：几乎所有代价无伪鞍点
├── 核心算法
│   ├── SE-Sync（§E.7-E.12）：PGO 全局最优
│   │   ├── Problem 1-3 的推导链
│   │   ├── Riemannian Staircase
│   │   └── 对偶证书验证
│   └── TEASER++（§E.13-E.15）：鲁棒配准
│       ├── TLS + GNC 去外点
│       ├── 旋转子问题 SDP
│       └── DRS 认证
├── 扩展与前沿（§E.16-E.19）
│   ├── Rotation Averaging
│   ├── PACE（category-level 姿态）
│   ├── STRIDE（general POP）
│   └── DC²-PGO（分布式可认证）
└── 边界与开放问题
    ├── 松弛紧的经验阈值
    ├── Incremental Certifiable（开放）
    └── Learning + Certificates（开放）
```

---

**与主干内容的关系**：B 和 C 建立了因子图上的 Gauss-Newton/LM/iSAM2 求解框架，但这些方法都是局部迭代——收敛到哪个极小点取决于初始化。本专题提出一个根本性的追问：对 SLAM/几何感知中的非凸优化问题，能否在多项式时间内**证明**当前解是全局最优？Certifiable Perception 通过 SDP 松弛与对偶证书给出了肯定回答（在松弛紧时）。这是从 B/C 的"局部好解"迈向"可证全局最优"的理论升级，也与 F（鲁棒估计）互补——F 处理外点问题，E 处理非凸性问题。建议在完成 B/C 后阅读本专题。

> **底线陈述（BLUF）**：5-B/5-C/5-F 给出的所有方法（GN、LM、iSAM2、GNC-TLS、M-estimator）本质都是**局部**迭代——即便收敛也只能保证**一阶 KKT** 或**二阶充分**条件，无法排除"好的局部最优"。5-E 给出的 **certifiable perception** 范式改变了这一局面：对 **rotation averaging、pose graph optimization (PGO)、点云配准 (point-cloud registration)、category-level 姿态估计** 等非凸问题，通过 **Shor 半定松弛（SDP relaxation）+ 对偶证书（dual certificate）**，在**多项式时间**内**后验验证**当前解是原非凸 MLE 的**全局最优解**。关键技术是 (1) **Lagrangian 对偶**把 QCQP 松弛成 SDP（Shor 1987）；(2) **Burer-Monteiro 分解** $Z=YY^\top$ 把 $O(n^2)$ 的 SDP 变量降到 $O(np)$，在 **Stiefel 流形**上用 **Riemannian trust-region** 求解；(3) 在紧光滑流形、约束资格条件成立且秩选择满足条件时，**Boumal-Voroninski-Bandeira** 定理给出“几乎所有代价”下二阶临界点全局最优的保证；(4) **对偶证书** $S=Q-\Lambda^\star\succeq0$ 可作为全局最优的充分认证条件。**SE-Sync**（Rosen et al. IJRR 2019）在 sphere2500/torus/garage 等标准数据集上 100% 获得证书，比 GTSAM Powell's Dog-Leg 快 **平均 3×、最高 27×**；**TEASER++**（Yang et al. T-RO 2021）在 **>99% 外点**下仍能恢复正确配准。代价是：目前 certifiable 方法主要支持 **batch**、尚不原生支持 **incremental**，且松弛紧性对噪声/外点水平存在**经验阈值**。这是 SLAM/几何感知的**状态估计终点**——当 SDP 紧且证书成立时，估计问题在多项式时间内可解到全局最优，理论上不再有"局部极小"之虞。

---

### §E.1 局部方法的根本局限——为何要追求全局最优 ⭐⭐

5-B 建立的 Gauss-Newton / Levenberg-Marquardt、5-C 建立的 iSAM2、5-D 建立的 InEKF、5-F 建立的 GNC-TLS，全部都是**局部**方法。收敛性分析的终点永远是"收敛到某个局部最优"——**收敛到哪个**取决于初值。对凸问题这无所谓（局部=全局），但 **SLAM/geometric perception 几乎全部非凸**：

1. **SO(d) 流形非凸**：$R^\top R = I$ 是二次等式约束（QCQP），可行集在 $\mathbb{R}^{d\times d}$ 中是非凸子流形。
2. **旋转与平移耦合**：按本文采用的相对旋转约定，PGO 目标函数 $\|R_j - R_i\tilde R_{ij}\|_F^2$ 展开后包含 $R_i^\top R_j$ 交叉项——这是关于 $R_i, R_j$ 矩阵元素的双线性项（二次形式），使目标函数连同 $R^\top R=I$ 约束构成 QCQP。
3. **重投影残差**：BA 里 $\pi(K[R|t]X)$ 是高度非线性。

**Carlone-Censi (T-RO 2014)** 对 2D 平面 PGO 给出了最惊人的结果：将旋转参数化为角度 $\theta_i \in (-\pi, \pi]$ 后，经典的线性正则化会在 $\mathbb{R}$ 上产生 **$2^n$ 个局部极小**——它们对应整数晶格上 $\theta_i$ 的不同"卷绕"选择。这意味着对一个 $n=1000$ 的 pose graph，GN 有 $\approx 10^{300}$ 个局部极小池可能跌入。**Carlone 等 T-RO 2016** 进一步推导出：**回路边测量误差超过某阈值（经验上 $\sigma_\theta > 0.3$ rad）时，LM 收敛到正确解的概率骤降**。

实际场景中坏初值极为常见：**大回环闭合**（loop closure）、**全局重定位**（global relocalization）、**多地图合并**（multi-session merge）、**kidnapped robot** 问题都会让 GN 从远离真值的点启动。工业界的补救是"多次随机重启 + 选最低代价"，但这既**无理论保证**又**耗时**。

5-F 的 GNC-TLS 在高外点率（>80%）下**经验上**能恢复正确拓扑，但**不提供证书**——你无法区分它是找到了全局最优还是找到了一个恰好"看起来不错"的局部最优。这就是 5-E 要回答的终极问题：**能否在多项式时间内证明当前估计是全局最优？**

**参考**：Carlone et al. "Planar Pose Graph Optimization: Duality, Optimal Solutions, and Verification" *IEEE T-RO* 32(3):545-565, 2016；Carlone "Estimation Contracts for Outlier-Robust Geometric Perception" *FnT in Robotics* 11(2-3):90-224, 2023 (arXiv:2208.10521)。

---

### §E.2 Certifiable Perception 的形式化定义 ⭐⭐

**Yang-Carlone** 在 TPAMI 2023（arXiv:2109.03349）给出精确定义：

> **定义（certifiable algorithm）**：一个算法 $\mathcal{A}$ 对非凸问题 $\min_x f(x)$ 称为 **certifiable**，若在多项式时间内 $\mathcal{A}$ 返回候选解 $\hat x$ 以及一个布尔标志 `cert`，使得：当 `cert = true` 时，$\hat x$ **可验证**是原问题的全局最优解；当 `cert = false` 时，$\hat x$ 可能不是（但也不一定不是）全局最优。

关键要点三条：

1. **不保证一定找到全局最优**——有些高噪声/高外点实例中 `cert = false`，算法"放弃认证"。
2. **保证当 `cert = true` 时的解确实是全局最优**——认证是**单向的、充分的**。
3. **松弛紧 (tightness)** 是认证成功的关键：若某个 SDP 松弛 $\text{SDP}_{\text{opt}} = \text{QCQP}_{\text{opt}}$（对偶间隙为零），则从 SDP 解可以构造 QCQP 的全局最优解。

**Yang-Carlone 三层架构（TPAMI 2023）**：

| 层 | 对象 | 作用 |
|---|---|---|
| 1 | 非凸原问题（MLE/MAP over $SO(d)^n$ 或 $SE(d)^n$ 或 TLS） | 真要解的问题 |
| 2 | SDP 松弛（凸化 via Shor lifting） | 凸且有全局最优 |
| 3 | 对偶证书 $S = Q - \Lambda^\star \succeq 0$ | 验证松弛紧 & 恢复原解 |

**松弛紧的判据**：SDP 最优解 $Z^\star$ 恰好是 rank-$d$（对 $SO(d)$ 问题）或 rank-1（对标准 QCQP）。在相应 KKT 条件、互补松弛和具体问题结构成立时，低秩解、PSD 对偶证书、零对偶间隙和原问题全局最优可恢复之间会形成严密联系；工程上最直接可用的是 **PSD 对偶证书给出充分认证条件**。这是整个 certifiable perception 的核心判据。

**参考**：Yang \& Carlone "Certifiably Optimal Outlier-Robust Geometric Perception: Semidefinite Relaxations and Scalable Global Optimization" *IEEE TPAMI* 2023；Rosen et al. "Advances in Inference and Representation for SLAM" *ARCRAS* 4:215-242, 2021 (arXiv:2103.05041) 的 certifiability 综述节。

---

### §E.3 Semidefinite Programming 基础 ⭐⭐⭐

**标准 SDP**（Boyd-Vandenberghe *Convex Optimization* §4.6, p.168，conic 形式 eq.4.52）：

$$
\boxed{\;\min_{X \in \mathbb{S}^n}\; \langle C, X \rangle \quad \text{s.t.}\quad \langle A_i, X\rangle = b_i,\ i=1..m,\ X \succeq 0\;}
$$

其中 $\mathbb{S}^n$ 是 $n\times n$ 对称矩阵空间，$\langle A, B\rangle := \operatorname{tr}(A^\top B)$ 是 **trace inner product**，$X \succeq 0$ 表示**半正定**。

**SDP 的核心性质**：

1. **凸性**：$\mathbb{S}^n_+$ 是凸锥（pointed, closed, self-dual w.r.t. trace 内积）；目标函数线性、约束线性。
2. **强对偶与可达性有充分条件**——Slater 条件常用来保证强对偶和相应侧最优值可达；具体原问题解是否存在，还要结合可行集闭有界性、目标下有界性或 level-bounded 条件判断。
3. **多项式时间可解**：内点法复杂度 $O(\sqrt{n}\log(1/\epsilon))$ 次迭代（Nesterov-Nemirovski 1994），每次迭代 $O(n^3 + mn^2 + m^2 n^2 + m^3)$。

**对偶 SDP**（Boyd eq.4.53）：

$$
\max_{y \in \mathbb{R}^m}\; b^\top y \quad \text{s.t.}\quad C - \sum_{i=1}^m y_i A_i \succeq 0.
$$

**弱对偶**总成立：$\langle C, X\rangle - b^\top y = \langle X, C - \sum y_i A_i\rangle \ge 0$（两个半正定矩阵内积非负）。

**强对偶（Slater 条件）**（Boyd §5.9, pp.265-267）：若存在**严格内点** $X \succ 0$ 原可行，或存在 $y$ 使 $C - \sum y_i A_i \succ 0$ 对偶可行，则 $p^\star = d^\star$ 且最优在严格可行侧可达。SE-Sync 这类块对角约束 SDP 通常可以直接构造或验证 Slater 条件；一般几何感知 SDP 仍需逐问题检查，不能仅凭“来自 SLAM”就默认强对偶成立。

**KKT 条件（SDP 版本）**：$(X^\star, y^\star, S^\star)$ 最优 $\iff$
$$
\langle A_i, X^\star\rangle = b_i,\quad \sum y_i^\star A_i + S^\star = C,\quad X^\star \succeq 0,\quad S^\star \succeq 0,\quad X^\star S^\star = 0.
$$
最后一条 $X^\star S^\star = 0$ 是**矩阵互补松弛**，蕴含 $\operatorname{rank}(X^\star) + \operatorname{rank}(S^\star) \le n$。

**核心教材**：Boyd-Vandenberghe *Convex Optimization* 2004, §4.6 (pp.167-172), §5.1-5.5 (pp.215-244), §5.9 (pp.265-267), §A.5 (pp.646-651)；Vandenberghe-Boyd "Semidefinite Programming" *SIAM Review* 38(1):49-95, 1996。

#### SDP 的物理/几何直觉

SDP 的约束 $X\succeq 0$ 的几何含义是什么？

**直觉 1：相关矩阵**。如果 $X$ 是一个 correlation matrix（对角为 1），$X\succeq 0$ 表示它可以被写成 $X=V^\top V$，即存在一组向量 $v_1,...,v_n$ 使得 $X_{ij}=\langle v_i, v_j\rangle$。MaxCut SDP 松弛（Goemans-Williamson 1995）正是利用这个几何：把离散 $\pm 1$ 变量松弛为单位球上的连续向量，$X_{ij}=\langle v_i, v_j\rangle$ 编码了"两个节点有多接近"。

**直觉 2：旋转的 Gram matrix**。对 rotation averaging，$Z=RR^\top$ 其中 $R=[R_1^\top,...,R_n^\top]^\top\in\mathbb{R}^{3n\times 3}$。$Z\succeq 0$ 是 rank-3 的自然放松——允许"旋转"生活在更高维空间中。松弛紧意味着：最优的"高维旋转"恰好可以投影回 $SO(3)^n$。

**直觉 3：不确定性椭球**。$X\succeq 0$ 定义了一个椭球 $\{z : z^\top X^{-1} z \le 1\}$。SDP 约束可以理解为"在所有满足线性约束的椭球中找代价最小的"。

> **跨领域类比**：SDP 松弛之于非凸 QCQP，如同线性规划（LP）松弛之于整数规划（IP）。
> 在 IP 中，变量被限制为 $\{0,1\}$（组合约束）；LP 松弛允许 $[0,1]$（连续化）。
> 在 QCQP 中，变量被限制为 $X=xx^\top$（rank-1，极度非凸）；SDP 松弛允许 $X\succeq 0$（半正定锥，凸）。
> 两者的共同哲学是：**用凸的外逼近包住非凸可行集，在凸问题上求全局最优，再检查解是否"恰好"落在原始可行集中**。
> LP 松弛成功时 = 整数解恰好是 LP 最优点（integrality gap = 0）；
> SDP 松弛成功时 = 低秩解恰好是 SDP 最优点（tightness / zero duality gap）。

#### 对偶证书的直觉——为什么 $S\succeq 0$ 就能证明全局最优？

这是整个 certifiable perception 中最关键的逻辑步骤，值得从多个角度理解。

**推导路径**：

设原问题（QCQP）为 $\min_x f(x)$ s.t. $g_i(x)=0$。Lagrangian 为

$$L(x,\lambda) = f(x) - \sum_i \lambda_i g_i(x)$$

对偶函数为

$$d(\lambda) = \min_x L(x,\lambda)$$

**弱对偶**：对任何可行 $x_0$（即 $g_i(x_0)=0$），

$$f(x_0) = L(x_0,\lambda) \ge \min_x L(x,\lambda) = d(\lambda)$$

因此 $d(\lambda)$ 是 $f$ 的下界。

**对偶证书的逻辑**：如果找到了一个 $\lambda^\star$ 使得 $d(\lambda^\star)=f(\hat{x})$（某个可行点 $\hat{x}$ 的目标值），则：

$$f(\hat{x}) = d(\lambda^\star) \le f(x) \quad \forall \text{ feasible } x$$

这证明了 $\hat{x}$ 是全局最优——因为对偶下界恰好等于 $\hat{x}$ 的目标值。

**在 SDP 语言中**：$S=C-\sum\lambda_i^\star A_i$ 是"对偶松弛矩阵"。$S\succeq 0$ 等价于 $d(\lambda^\star)$ 有界。如果同时 $S\hat{x}=0$（互补松弛），那么

$$f(\hat{x}) = \langle C, \hat{x}\hat{x}^\top\rangle = \langle S+\sum\lambda_i^\star A_i, \hat{x}\hat{x}^\top\rangle = \hat{x}^\top S\hat{x} + \sum\lambda_i^\star b_i = 0 + d(\lambda^\star)$$

等式成立。这就是**对偶证书**的完整逻辑。

> **本质洞察**：对偶证书的力量在于它提供了一个**与求解路径无关的验证**。
> 无论你用什么方法找到候选解 $\hat{x}$（GN、LM、随机搜索、甚至猜测），
> 只要你能同时找到一个 $\lambda^\star$ 使得 $S\succeq 0$ 且 $S\hat{x}=0$，
> 就能证明 $\hat{x}$ 是全局最优。这把"求解"和"验证"完全解耦——
> 求解可以用启发式（如 Riemannian optimization on BM），验证是严格数学。

**工程含义**：在 SE-Sync 中，实际求解步骤是在 Stiefel 流形上跑 Riemannian trust-region（非凸优化，可能陷入鞍点）。但只要求解完成后，检查 $\lambda_{\min}(S)\ge -\epsilon$，就能**后验确认**解的全局最优性。这就是"certifiable"的含义——**不信任求解器，信任证书**。

---

### §E.4 QCQP -> SDP 的 Shor 松弛 ⭐⭐⭐

**二次约束二次规划 (QCQP)**（Boyd eq.4.34, p.152）：

$$
\min_{x \in \mathbb{R}^n}\ x^\top C x\quad \text{s.t.}\quad x^\top A_i x = b_i,\ i=1..m.
$$

当 $C, A_i$ 有一个不定（indefinite），QCQP 一般**NP-hard**（例如 MaxCut、Boolean quadratic program）。

**Shor lifting**（Shor 1987；Luo-Ma-So-Ye-Zhang *IEEE SPM* 2010）。观察恒等式：
$$
x^\top C x = \operatorname{tr}(C x x^\top),\qquad x^\top A_i x = \operatorname{tr}(A_i x x^\top).
$$

令 $X := xx^\top$。则 $X \succeq 0$ 且 $\operatorname{rank}(X) = 1$；反之 $X \succeq 0$、$\operatorname{rank}(X)=1$ $\iff$ $X = xx^\top$ for some $x$。于是 QCQP **等价于**：

$$
\min_{X \in \mathbb{S}^n_+}\ \operatorname{tr}(CX)\quad \text{s.t.}\quad \operatorname{tr}(A_i X) = b_i,\ \boxed{\operatorname{rank}(X) = 1}.
$$

**Shor 松弛**：**扔掉** rank-1 约束。得到：

$$
\boxed{\ (\text{SDR})\quad \min_{X}\ \operatorname{tr}(CX)\ \ \text{s.t.}\ \operatorname{tr}(A_i X) = b_i,\ X \succeq 0.\ }
$$

**关键性质**：

| 性质 | 说明 |
|---|---|
| 凸性 | SDR 是 SDP，凸，多项式时间可解 |
| 下界 | $p^\star_{\text{SDR}} \le p^\star_{\text{QCQP}}$（松弛后可行域变大） |
| **紧 (tight)** | 若 SDR 最优解 $X^\star$ **rank 1**，则 $X^\star = x^\star (x^\star)^\top$，$x^\star$ 是 QCQP 全局最优；此时 $p^\star_{\text{SDR}} = p^\star_{\text{QCQP}}$ |

**Lagrangian 对偶推导 Shor 松弛**：对 QCQP 写 Lagrangian
$$
L(x, y) = x^\top(C - \textstyle\sum_i y_i A_i)x + b^\top y.
$$
对 $x$ 求下确界：$g(y) = b^\top y$ 若 $C - \sum y_i A_i \succeq 0$，否则 $-\infty$。对偶
$$
\max_y\ b^\top y\ \ \text{s.t.}\ C - \sum y_i A_i \succeq 0
$$
正是 SDR 的对偶。**Lagrangian 对偶紧 ⟺ SDR 紧 ⟺ SDR 有 rank-1 最优解**——这是 certifiable perception 反复使用的核心结论。

> **概念误区**：初学者常认为"SDP 松弛 = 扔掉约束让问题变容易"。实际上 Shor 松弛只扔掉了 rank-1 约束（$X=xx^\top$），所有原始等式约束 $\operatorname{tr}(A_iX)=b_i$ 都保留。松弛后可行域变大（从 rank-1 子集扩大到整个 PSD 锥截面），因此最优值只能变小或不变——这是"下界"性质的来源。

**非齐次 QCQP**：若目标含一次项 $2q^\top x$，lift 到 $y = [x; 1]$，$Y = yy^\top$，加约束 $Y_{n+1,n+1} = 1$，rank-1；扔 rank 得 Shor。

**MaxCut 作为原型**（Goemans-Williamson *JACM* 1995）：
$$
\max \sum_{(i,j) \in E} \tfrac{w_{ij}}{2}(1 - X_{ij})\ \text{s.t.}\ X_{ii} = 1,\ X \succeq 0.
$$
随机超平面舍入给出 **0.87856 近似保证**——这是 Shor 松弛的"教科书例子"，SE-Sync 和 TEASER 的 SDP 本质上都是它的"结构化孪生"。

---

### §E.5 Lagrangian 对偶证书——certifiable perception 的认证器 ⭐⭐⭐

**对偶证书 (dual certificate)** 是整个框架的"签字笔"。设 QCQP 目标 $x^\top C x$，等式约束 $x^\top A_i x = b_i$。拉格朗日乘子 $\lambda \in \mathbb{R}^m$。**KKT 条件**给出：若 $x^\star$ 是全局最优且强对偶成立，则存在 $\lambda^\star$ 使

$$
\boxed{\;S(\lambda^\star) := C - \sum_{i=1}^m \lambda_i^\star A_i \succeq 0\quad \text{且}\quad S(\lambda^\star)\, x^\star = 0.\;}
$$

**对偶证书验证流程**：

1. 用任意方法（GN/LM/Burer-Monteiro）得到候选解 $\hat x$。
2. 从 $\hat x$ 的一阶 KKT 条件**显式构造** $\hat \lambda$（对 SE-Sync 是 SymBlockDiag 操作，见 §E.8）。
3. 构造 $S(\hat \lambda) = C - \sum \hat \lambda_i A_i$。
4. 用 **Lanczos** 或 **LOBPCG** 计算 $\lambda_{\min}(S(\hat \lambda))$。
5. 检查候选解满足原问题可行性，并检查互补/驻点条件，例如 $S(\hat \lambda)\hat x\approx0$ 或等价的 primal-dual gap 足够小。
6. **若 $\lambda_{\min} \ge -\epsilon$**（$\epsilon$ 数值容差，如 $10^{-8}$）且第 5 步通过：认证 $\hat x$ 为全局最优。
7. **若 $\lambda_{\min} < -\epsilon$** 或互补条件失败：松弛不紧或 $\hat x$ 不是局部最优；报告"无法认证"。

**为什么这是充分的？** 若 $S(\hat \lambda) \succeq 0$，则对任意可行 $x$：
$$
f(x) = x^\top C x = x^\top S(\hat \lambda) x + \sum \hat \lambda_i\, x^\top A_i x \ge \sum \hat \lambda_i b_i = f(\hat x)
$$
（最后一步用了 $S(\hat \lambda) \hat x = 0$ 推出的 $\hat x^\top S(\hat \lambda) \hat x = 0$）。这就证明 $\hat x$ 全局最优。

**Boyd §5.5.3 (p.243)**：对 differentiable 问题，强对偶下 KKT 是最优的必要充分条件。Slater 成立时 KKT ⟺ primal-dual 最优。

**Lasserre hierarchy**（Lasserre *Moments, Positive Polynomials* 2010）把 Shor 松弛作为 **moment-SOS hierarchy 的第一层**；更高层可以为更广泛的多项式问题提供更紧松弛，但计算代价指数增长。许多标准 SLAM 松弛在常见噪声水平下第一层已经足够，但是否成立仍要以秩条件和对偶证书为准。

---

### §E.6 SDP 求解器概览 ⭐⭐⭐

**内点法 (IPM)**：MOSEK、SeDuMi、SDPT3——用 log-det 自洽障碍函数 $\phi(X) = -\log\det X$，Newton 迭代追中心路径。**实用规模**：单块 PSD 矩阵边 $n \lesssim 2\text{-}5 \times 10^3$；约束数 $m \lesssim 10^4$。内存 $O(n^2 + m^2)$，时间每步 $O(n^3 + mn^2 + m^2 n^2)$。对 SLAM 场景（$n$ = 万级位姿）**完全不可行**。

**一阶法**：**SCS** (O'Donoghue-Chu-Parikh-Boyd *JOTA* 2016, `github.com/cvxgrp/scs`)——ADMM + 齐次自对偶嵌入；每步一次缓存 LDLᵀ 因子化 + 锥投影；支持 $10^5$–$10^6$ 变量，但精度止于 $10^{-3}$–$10^{-5}$，**对证书验证精度不够**。

**Burer-Monteiro**（Burer-Monteiro *Math. Prog.* 95(2):329-357, 2003）——**SLAM 的杀手锏**。

> **想法**：既然（Barvinok-Pataki 1998）SDP 最优解的秩满足 $\frac{r(r+1)}{2} \le m$，即 $r \le \sqrt{2m}$，那就**直接以秩为 $p$ 的 $Y \in \mathbb{R}^{n \times p}$ 参数化** $X = YY^\top$：
> $$
> (\text{BM}_p)\quad \min_{Y \in \mathbb{R}^{n \times p}}\ \langle C, YY^\top\rangle\ \text{s.t.}\ \langle A_i, YY^\top\rangle = b_i,\ i=1..m.
> $$

- 变量存储从 $O(n^2)$ 降至 $O(np)$，$p \approx \sqrt{2m} \ll n$。
- 自动满足 $X \succeq 0$（因 $YY^\top \succeq 0$ 总成立）。
- **但非凸**——等式约束从线性变成了二次（$Y^\top Y$ 带来）。

**Boumal-Voroninski-Bandeira 定理**（NeurIPS 2016；*CPAM* 73(3):581-608, 2020, arXiv:1804.02008）：

> **定理**：设 $\mathcal{M}_p = \{Y : \mathcal{A}(YY^\top) = b\}$ 是紧的光滑流形（CQ 成立），且 $p(p+1)/2 > m$，即 $p > \sqrt{2m}$。则：
>
> **(a) 秩亏版本**：对**任意** $C$，若 $Y \in \mathcal{M}_p$ 是 (BM$_p$) 的**秩亏二阶临界点**（row rank $< p$），则 $X = YY^\top$ 是 (SDP) 的**全局最优**。
>
> **(b) 几乎处处版本**：对**几乎所有** $C$（Lebesgue-零测集外），(BM$_p$) 的**每个**二阶临界点都是全局最优。

**SE-Sync 的 Riemannian Staircase** 利用 (a)：从小 $p$ 开始，若二阶临界点秩亏则完成；否则 $p \leftarrow p+1$ 继续爬。实际数据中 $p = d+1$ 或 $d+2$ 往往已经足够，但这属于经验现象，不是一般 SDP 的理论保证。

**反例**：Waldspurger-Waters (*SIAM J. Optim.* 30(3):2577-2602, 2020) 证明 BVB 的 $p \gtrsim \sqrt{2m}$ 界**本质上紧**；低于它时可构造带伪局部极小的 $C$。O'Carroll-Srinivas-Vijayaraghavan (NeurIPS 2022) 显示 MaxCut 上 BM 可在 $p = n/2$ 以下失败——所以 BM 在**一般 SDP** 上**没有保证**，但在 SE-Sync 的特殊结构下 empirical tightness 100%。

**Riemannian 优化视角**：BM 约束 $\langle A_i, YY^\top\rangle = b_i$ 构成光滑嵌入流形 $\mathcal{M}_p$。在其上做 **Riemannian trust-region (RTR)**（Absil-Baker-Gallivan *FoCM* 2007）——用 retraction 代替测地线，用 truncated CG 求解子问题。Manopt (`manopt.org`)、Pymanopt、ROPTLIB 是通用实现。

---

### §E.7 SE-Sync 问题设定——从 PGO 到 certifiable 求解 ⭐⭐⭐

**Rosen-Carlone-Bandeira-Leonard** "SE-Sync: A Certifiably Correct Algorithm for Synchronization over the Special Euclidean Group" *IJRR* 38(2-3):95-125, 2019 (arXiv:1612.07386)。

**输入**：连通图 $G = (V, E)$，$|V| = n$；对每边 $(i,j) \in E$ 有相对测量 $(\tilde R_{ij}, \tilde t_{ij}) \in SE(d)$ 及精度 $\kappa_{ij}, \tau_{ij}$。

**噪声模型**（paper eq.10）：
$$
\tilde t_{ij} = \bar t_{ij} + t^\epsilon,\ t^\epsilon \sim \mathcal{N}(0, \tau_{ij}^{-1} I_d);\qquad \tilde R_{ij} = \bar R_{ij} R^\epsilon,\ R^\epsilon \sim \text{Langevin}(I_d, \kappa_{ij})
$$
**Langevin (isotropic)**：$p(X; M, \kappa) = \frac{1}{c_d(\kappa)} \exp(\kappa \operatorname{tr}(M^\top X))$；对 $d=3$，$c_3(\kappa) = \exp(\kappa)(I_0(2\kappa) - I_1(2\kappa))$，渐近 $\text{SD}[\theta] \approx 1/\sqrt{2\kappa}$。

**Problem 1（MLE）**：
$$
\min_{\substack{t_i \in \mathbb{R}^d\\ R_i \in SO(d)}}\ \sum_{(i,j) \in \vec{E}} \Big[\kappa_{ij}\|R_j - R_i \tilde R_{ij}\|_F^2 + \tau_{ij}\|t_j - t_i - R_i \tilde t_{ij}\|_2^2\Big].
$$
非凸（$SO(d)$ 约束）。

**关键认识**：$t_i$ 无约束，可**解析消去**；化约后是一个"纯旋转"的 QCQP。这是 SE-Sync 全部技术的起点。

---

### §E.8 SE-Sync 的九步松弛链——从 Problem 1 到 Problem 9 ⭐⭐⭐⭐

**Rosen et al. 2019 的九步 reformulation**（这是全文最关键的数学链条）：

**Problem 2（QP 形式）**：把 $[t; \operatorname{vec}(R)]$ 拼成长向量，目标写成 $[t; \operatorname{vec}(R)]^\top (M \otimes I_d) [t; \operatorname{vec}(R)]$（paper eq.18），$M$ 是 $2\times 2$ 块矩阵，由加权平移 Laplacian $L(W_\tau)$、旋转 connection Laplacian $L(\tilde G^\rho)$、相对平移矩阵 $\tilde V$、对角 $\tilde \Sigma$ 构成。

**Problem 3（仅旋转 MLE，消去 $t$）**：用平移无约束的最优性条件，得 $t^\star = -\operatorname{vec}(R^\star \tilde V^\top L(W_\tau)^\dagger)$（paper eq.21）。这个负号来自 Rosen 论文对 $\tilde V$ 的定义；若改写为 §E.31 的 $\min_t\|Bt-c(R)\|^2$ 记号，等价表达为 $t^\star=(B^\top B)^\dagger B^\top c(R)$。回代得
$$
\min_{R \in SO(d)^n}\ \operatorname{tr}(\tilde Q\, R^\top R),\qquad \tilde Q = L(\tilde G^\rho) + \tilde \Sigma - \tilde V^\top L(W_\tau)^\dagger \tilde V.
$$
（paper eq.20）。$\tilde Q$ 是 $dn \times dn$ 对称半正定矩阵——**certifiable pose graph 核心矩阵**。

**Problem 4（simplified MLE）**：用 $\tilde Q_\tau = \tilde T^\top \Omega^{1/2} \Pi \Omega^{1/2} \tilde T$（eq.24），$\Pi = I_m - \Omega^{1/2} A^\top L^{-\top} L^{-1} A \Omega^{1/2}$ 为 $\ker(A(\vec G)\Omega^{1/2})$ 上的正交投影，通过对关联矩阵 $A(\vec G)\Omega^{1/2} = LQ_1$ 做**稀疏 LQ 分解**获得稀疏表示。

**Problem 5（正交松弛）**：$SO(d) \to O(d)$，把行列式约束放松：
$$
\min_{R \in O(d)^n}\ \operatorname{tr}(\tilde Q\, R^\top R).
$$
（eq.25）。$O(d)$ 包含反射；rank-$d$ 的 SDR 最优解可能对应 $O(d)$ 而非 $SO(d)$，但通过取多数块符号修复。

**Problem 6（原始 SDP，Lagrangian 对偶）**：用乘子 $\Lambda \in \text{SBD}(d, n)$（对称块对角矩阵）对 $R_i^\top R_i = I_d$ 构造对偶：
$$
\max_{\Lambda \in \text{SBD}(d,n)}\ \operatorname{tr}(\Lambda)\quad \text{s.t.}\quad \tilde Q - \Lambda \succeq 0.
$$
（eq.32）。

**Problem 7（中心 SDP 松弛）**：Problem 6 的对偶：
$$
\boxed{\;\min_{Z \in \mathbb{S}^{dn}_+}\ \operatorname{tr}(\tilde Q Z)\quad \text{s.t.}\quad Z_{ii} = I_d,\ \forall i=1..n.\;}
$$
（eq.33）。这是 SE-Sync 真正求解的 SDP。相当于 MaxCut SDP（$X_{ii}=1$）的**矩阵值**推广（$Z_{ii} = I_d$）。

**Problem 8（Burer-Monteiro NLP）**：$Z = Y^\top Y$，$Y \in \mathbb{R}^{r \times dn}$，$r \ll dn$：
$$
\min_{Y}\ \operatorname{tr}(\tilde Q Y^\top Y)\quad \text{s.t.}\quad Y_i^\top Y_i = I_d.
$$
（eq.35）。

**Problem 9（Riemannian 形式）**：约束 $Y_i \in \text{St}(d, r)$（Stiefel 流形，$d \le r$）：
$$
\boxed{\;\min_{Y \in \text{St}(d, r)^n}\ \operatorname{tr}(\tilde Q Y^\top Y).\;}
$$
（eq.38）。这是在 $n$ 个 Stiefel 流形乘积上的 Riemannian 优化，**实际求解的问题**。

---

### §E.9 SE-Sync 的四大紧性定理 ⭐⭐⭐⭐

**定理 1（精确恢复等价性，paper Thm.1）**：若 $Z^\star$ 是 Problem 7 最优解且因子化为 $Z^\star = R^{\star\top} R^\star$ 其中 $R^\star \in O(d)^n$，则 $R^\star$ 是 Problem 5 全局最优。若进一步 $R^\star \in SO(d)^n$，则 $R^\star$ 是 Problem 4 全局最优，$(t^\star, R^\star)$ 是 Problem 1 全局 MLE。**证明**：SDP 对偶给出 $p^\star_6 \le p^\star_7$，松弛关系给出 $p^\star_7 \le p^\star_5 \le p^\star_4$；若强对偶且秩条件成立，就得到从松弛解回到原问题的全局最优证书。

**引理 6（KKT 一阶条件）**：若 $R^\star$ 是 Problem 5 最优，则存在 $\Lambda^\star \in \text{SBD}(d, n)$ 满足
$$
(\tilde Q - \Lambda^\star) R^{\star\top} = 0,\qquad \Lambda^\star = \text{SymBlockDiag}_d(\tilde Q\, R^{\star\top} R^\star).
$$
**这给出从 primal 解 $R^\star$ 直接构造对偶证书 $\Lambda^\star$ 的显式公式**。

**定理 7（充分条件）**：令 $S := \tilde Q - \Lambda^\star$ 为**对偶证书矩阵**。若
$$
S \succeq 0
$$
则 $Z^\star = R^{\star\top} R^\star$ 是 Problem 7 最优解。若进一步 $\operatorname{rank}(S) = dn - d$，则 $Z^\star$ 是**唯一**最优（用 Alizadeh-Haeberly-Overton *SIOPT* 1997 的 primal-dual 非退化）。**这就是对偶证书验证的全部内容**。

**定理 10（无噪声精确性）**：无噪声下 $\tilde Q = Q$，Problem 7 紧且唯一。用 connection Laplacian 性质：$L(W_\rho) \otimes I_d = S\, L(G_\rho)\, S^{-1}$，$S = \operatorname{Diag}(R_1,...,R_n)$；$\lambda_{d+1}(L(G_\rho)) = \lambda_2(L(W_\rho)) > 0$（$G$ 连通）；$\ker L(G_\rho) = \{R^\top v : v \in \mathbb{R}^d\}$（引理 8）。

**定理 12（误差上界）**：
$$
d_O(R, R^\star) \le \sqrt{\frac{4dn\,\|\tilde Q - Q\|_2}{\lambda_{d+1}(Q)}}.
$$
其中 $d_O$ 是 $O(d)^n / O(d)$ 商空间上的 gauge-invariant Procrustes 距离。

**命题 2（带噪声精确恢复）**：存在 $\beta = \beta(Q) > 0$ 使 $\|\tilde Q - Q\|_2 < \beta$ 时 Problem 7 **唯一**最优 $Z^\star = R^{\star\top} R^\star$，$R^\star \in SO(d)^n$ 是 Problem 4 最优。证明通过 $C = \tilde Q - \Lambda^\star$ 特征值连续性 + $O(d)$ 分量 $\sqrt{2}$-分离性给出两个阈值 $\beta_1, \beta_2$，取 $\beta = \min(\beta_1, \beta_2)$。证明模板改编自 Bandeira-Boumal-Singer *Math. Prog.* 2017 angular synchronization tightness。

**命题 3（BVB 秩亏 → 全局最优，paper Prop.3）**：若 $Y \in \text{St}(d, r)^n$ 是 Problem 9 的**行秩亏二阶临界点**，则 $Y$ 是 Problem 9 全局最优，$Z^\star = Y^\top Y$ 是 Problem 7 全局最优——**这是 Riemannian Staircase 的理论根据**。

---

### §E.10 SE-Sync 算法（三阶段 + 对偶证书验证） ⭐⭐⭐

**Algorithm 1 — Riemannian Staircase**：
```text
输入: Q̃, 初始秩 r₀ (默认 5, 至少 d+1)
for r = r₀, r₀+1, ..., r_max:
    用 RTR 求解 Problem 9 在 St(d,r)ⁿ 上的 Y*_r
    if rank(Y*_r) < r:        # 秩亏 → BVB 命题 3
        return Y* := Y*_r
    else:
        # "爬楼梯": 往 Y*_r 追加一行零,升维至 St(d,r+1)ⁿ
        Y_{r+1,init} = [Y*_r; 0]
return failure
```

**Algorithm 2 — Rounding（恢复 $SO(d)$ 解）**：
1. 对 $Y^\star$ 做 rank-$d$ truncated SVD：$Y^\star = U_d \Xi_d V_d^\top$。
2. $\hat R = \Xi_d V_d^\top$。
3. 若 $< n/2$ 个 $3\times 3$ 块 $\hat R_i$ 有正行列式，则右乘 $\operatorname{Diag}(1,...,1,-1)$ 翻号。
4. 对每个 $\hat R_i$ 做 special-orthogonal Procrustes 投影到 $SO(d)$（SVD 把奇异值设为 $(1,...,1,\operatorname{sign}\det)$）。

**Algorithm 3 — SE-Sync 主流程**：
1. 构造稀疏 $\tilde Q = L(\tilde G^\rho) + \tilde Q_\tau$，其中 $\tilde Q_\tau$ 用 $A(\vec G)\Omega^{1/2} = LQ_1$ 稀疏 LQ 分解构造 $\Pi$。
2. 运行 Algorithm 1 得 $Y^\star$；SDP 下界 $p^\star_{\text{SDP}} = \operatorname{tr}(\tilde Q Y^{\star\top} Y^\star)$。
3. Algorithm 2 舍入得 $\hat R$；由 eq.21 恢复 $\hat t$。
4. **验证证书**：计算 $\Lambda^\star = \text{SymBlockDiag}(\tilde Q \hat R^\top \hat R)$，$S = \tilde Q - \Lambda^\star$；用 **Lanczos** (库：Spectra C++) 求 $\lambda_{\min}(S)$。若 $\lambda_{\min} \ge -\epsilon$ **且** $\operatorname{tr}(\tilde Q \hat R^\top \hat R) - p^\star_{\text{SDP}} \le \epsilon$，认证全局最优。

**RTR 细节**：Riemannian 梯度通过 Stiefel 切空间投影 $\operatorname{grad} f(Y) = P_Y \nabla f(Y)$（eq.42-43）；精确 Hessian 可用（eq.44）；truncated CG 求解二次子问题；retraction 用 QR 或极分解。

**实现库**：SE-Sync C++ 使用 Eigen (稠密线性代数)、SuiteSparse/CHOLMOD (稀疏 Cholesky)、**ROPTLIB** (Riemannian optimization, Huang-Absil-Gallivan)、**Spectra** (Lanczos) —— 参见 `github.com/david-m-rosen/SE-Sync`。

---

### §E.11 SE-Sync 实测性能 ⭐⭐

**Cube 合成实验**（$n=1000$，$\kappa=16.67$（约 $14°$ rotation std），$\tau=75$，回环概率 $p_{LC}=0.1$，50 次 Monte-Carlo）：SE-Sync 在旋转噪声 RMS 误差 **$\le 20°$** 时 **100% 获得证书**——这比典型传感器噪声高一个数量级。

**2D SLAM benchmarks（paper Table 1）**：

| 数据集 | n poses / m edges | PDL-GN (s) | SE-Sync (s) | 加速比 |
|---|---|---|---|---|
| manhattan | 3500 / 5453 | — | — | — |
| city | 10000 / 20687 | — | — | — |
| csail | 1045 / 1172 | — | — | — |
| intel | 1728 / 2512 | — | — | — |
| ais2klinik | 15115 / 16727 | **12.472** | **1.981** | **6.3×** |

**3D SLAM benchmarks（paper Table 2）**：

| 数据集 | n / m | PDL-GN (s) | SE-Sync (s) | 加速比 |
|---|---|---|---|---|
| sphere | 2500 / 4949 | — | — | ~2× |
| torus | 5000 / 9048 | — | — | ~2-3× |
| grid | 8000 / 22236 | **46.343** | **1.717** | **27×** |
| garage | 1661 / 6275 | — | — | <1 (garage 是唯一 GN 更快的) |
| cubicle | 5750 / 16869 | — | — | ~3× |
| rim | 10195 / 29743 | — | — | ~4× |

**平均加速 ≈ 3.05×（排除 grid）**。所有测试用例中，相对次优性 $(F(\tilde Q \hat R^\top \hat R) - p^\star_{\text{SDP}})/p^\star_{\text{SDP}}$ 在 $10^{-14}$-$10^{-15}$ 量级——**数值上严格认证全局最优**。

**经验紧性**：在标准 SLAM 噪声下（rotation std $< 0.3$ rad，translation std 合理），SDP 松弛 **100% 紧**（$r_0 = 5$ 时已够；秩亏临界点 $Y^\star$ 的秩恰为 $d$）。

**硬件**：Dell Precision 5510、Intel Xeon E3-1505M @ 2.80 GHz、16 GB、Ubuntu 16.04；对比 GTSAM 4.0 Powell's Dog-Leg。

---

### §E.12 SE-Sync 代码映射与生态系统 ⭐⭐

**主库**：`github.com/david-m-rosen/SE-Sync`

核心头文件与类：
- `include/SESync/SESync.h` / `src/SESync.cpp`：主入口函数 `SESync::SESync(const measurements_t&, const SESyncOpts&)`。
- `SESync/SESyncProblem.h/.cpp`：`class SESyncProblem` 封装 $\tilde Q$ 稀疏构造、RTR 的代价/梯度/Hessian 回调。
- `SESync/SESync_types.h`：`RelativePoseMeasurement`、`Measurements` 类型。
- `SESync/SESync_utils.h`：稀疏矩阵工具（构造 $L(\tilde G^\rho)$ 等）。
- `SESync/SESyncResult.h`：返回 `SESyncResult { SDPval, Fxhat, xhat, y (Stiefel factor), suboptimality_bound, total_computation_time, termination_status }`。

关键参数（`SESyncOpts`）：`r0`（初始秩）、`rmax`（爬楼梯上限）、`rel_func_decrease_tol`、`min_eig_num_tol`、`max_eig_iterations`（Lanczos）、`initialization`（`CHORDAL` / `RANDOM` / `ODOMETRY`）。

**依赖**：Eigen ≥ 3.3、SuiteSparse、ROPTLIB（子模块）、Spectra（最小特征值）。

**Kimera-RPGO**（`github.com/MIT-SPARK/Kimera-RPGO`）：把 SE-Sync 作为 **Pose Graph Optimization 后端**，结合 **Pairwise Consistency Maximization (PCM)**（5-F 外点过滤）实现**鲁棒 + certifiable** 流水线——先 PCM 去外点，再 SE-Sync 求全局最优。代表了工业 SLAM 后端的"当前最佳实践"。

**与 iSAM2 集成模式**：SE-Sync 目前是**batch**，不支持增量。实际系统中典型做法是：
1. iSAM2 在线增量求解；
2. 每 $N$ 个关键帧或大回环闭合后触发 SE-Sync batch；
3. 用 SE-Sync 结果作为 `PriorFactor` 注入 GTSAM，重启动 iSAM2。

**Cartan-Sync**（Briales-González-Jiménez *IROS* 2017 / RA-L）：与 SE-Sync 平行发展，**不消去 $t$**，而是直接在 Cartan motion group $SE(d)^n$ 上构造 SDP。数学上等价（Rosen et al. 在 IJRR 2019 讨论节中证明），但 Cartan-Sync 的**变量维度更大**（每个位姿 $d+d^2$ vs SE-Sync $d^2$ 后消去），实际中 SE-Sync 更快。

**DC²-PGO**（Tian-Khosoussi-Rosen-How *T-RO* 37(6):2137-2156, 2021, arXiv:1911.05864）：**分布式** certifiable PGO，机器人团队中每台只存自己的位姿与邻居通信；用 Riemannian Block Coordinate Descent + 分布式 Riemannian Staircase + 分布式证书验证。2021 King-Sun Fu 最佳论文荣誉奖。

---

### §E.13 TEASER / TEASER++ 问题设定 ⭐⭐⭐

**Yang-Shi-Carlone** "TEASER: Fast and Certifiable Point Cloud Registration" *IEEE T-RO* 37(2):314-333, 2021 (arXiv:2001.07715)。

**输入**：两组 3D 对应点 $\{(a_i, b_i)\}_{i=1}^N$，$a_i, b_i \in \mathbb{R}^3$，其中**未知比例**的外点。生成模型：
$$
b_i = s^\circ R^\circ a_i + t^\circ + o_i + \epsilon_i,\qquad o_i = 0 \text{ (内点)}\ \text{or}\ \text{任意 (外点)},\ \epsilon_i \sim \mathcal{N}(0, \beta_i^2 I).
$$

**估计器（Truncated Least Squares, TLS）**：
$$
\boxed{\;\min_{s > 0, R \in SO(3), t \in \mathbb{R}^3}\ \sum_{i=1}^N \min\Big(\frac{\|b_i - sRa_i - t\|^2}{\beta_i^2},\ \bar c^2\Big)\;}
$$

$\bar c$ 是**截断常数**（truncation constant），通常 $=1$。TLS 对外点**自适应不敏感**：残差超过 $\bar c \beta_i$ 时代价被"封顶"，外点不再拉动拟合。

**TLS vs Huber vs GM（回顾 5-F）**：

| 损失 | 形式 | 外点影响 |
|---|---|---|
| L2 | $r^2$ | 无限大（外点支配） |
| Huber | $r^2$ if $|r| \le \delta$ else $2\delta\|r\|-\delta^2$ | 线性 |
| Geman-McClure | $r^2/(1+r^2/c^2)$ | 趋于常数 |
| **TLS** | $\min(r^2, \bar c^2)$ | **严格常数**（封顶） |

TLS 的好处：**内点与外点完全解耦**——内点仍是 L2（有严格 MLE 意义），外点无贡献。坏处：非光滑、非凸。

---

### §E.14 TEASER 三步解耦架构 ⭐⭐⭐

TEASER 的**核心洞察**：通过**测量不变量**把耦合的 $(s, R, t)$ 估计**串行解耦**。

**第一步：尺度估计 via TRIMs**
- **TIM** (Translation-Invariant Measurement)：$\bar a_{ij} := a_j - a_i$，$\bar b_{ij} := b_j - b_i$。因 $b_j - b_i = sR(a_j - a_i) + (o_j - o_i) + (\epsilon_j - \epsilon_i)$，$t$ 消失。
- **TRIM** (Translation-Rotation-Invariant)：$s_{ij} := \|\bar b_{ij}\|/\|\bar a_{ij}\| = s + o_{ij}^s + \epsilon_{ij}^s$，$R$ 也消失——**纯尺度标量 TLS**：
$$
\hat s = \arg\min_s \sum_{ij} \min((s_{ij} - s)^2/\sigma_{ij}^2,\ \bar c^2).
$$
**Adaptive voting** 在多项式时间内求全局最优（一维问题，排序 + 扫描所有候选区间）。

**第二步：旋转估计 via TIMs（最难）**
把 $\bar b_{ij} = \hat s R \bar a_{ij} + o_{ij}^R + \epsilon_{ij}^R$ 代入 TLS：
$$
\hat R = \arg\min_{R \in SO(3)} \sum_{ij} \min(\|\bar b_{ij} - \hat s R \bar a_{ij}\|^2/\delta_{ij}^2,\ \bar c^2).
$$
这是**外点鲁棒 Wahba 问题**（outlier-robust Wahba），可表为 **TLS-QCQP** 再 Shor 松弛。具体地，引入二进制变量 $\theta_i \in \{-1, +1\}$ 标记内/外点，把 min 写成 $\min(\alpha, \beta) = \frac{1}{2}((\alpha + \beta) - |\alpha - \beta|)$ 的 QCQP 形式，再用单位四元数 $q$ 表示 $R$（$\|q\|=1$ 二次约束）。**QUASAR**（Yang-Carlone *ICCV* 2019, arXiv:1905.12536）是前身。

**Shor SDP 松弛**：提升 $X = zz^\top$（$z = [q; \theta \otimes q]$），$X \succeq 0$，drop rank。**紧性定理（TEASER Thm.2）**：噪声低于某阈值、外点比例不太高时 SDR 紧。实验上 **>99% 外点**仍紧。

**Max-clique 图剪枝**：TIM 兼容性定义为 $|s_{ij} - \hat s| \le \tau$；构造兼容性图 $G_c = (V, E_c)$；**Maximum Clique** 中所有 TIM 两两兼容 → 几乎全内点。用 **PMC** (Parallel Maximum Clique, Rossi-Ahmed SDM 2015) 求解。剪枝后外点率从 99% 降到 <10%，SDP 变得极易求解。

**第三步：平移估计 via component-wise adaptive voting**
给定 $\hat s, \hat R$，$t_x, t_y, t_z$ 分量间解耦（单轴上的 TLS），又是**一维 TLS**，多项式可解。

**TEASER++ 的实际策略**：第二步 SDP 太慢（内点法 $O(n^{6.5})$）；实际用 **GNC-TLS**（5-F 的方法）作为**快速求解器**，再用 **Douglas-Rachford Splitting (DRS)** 作为**后验证书**——若 DRS 迭代收敛到 SDP 最优则认证。"Certify only when you need it"。

**紧性实验**：Bunny 数据集 1000 对应、99% 外点，TEASER SDP 恢复 rotation error < 1°，100% 紧；RANSAC-1K 完全失败。

---

### §E.15 TEASER++ 代码 API ⭐⭐

**主库**：`github.com/MIT-SPARK/TEASER-plusplus`（MIT license；~2.2k stars Apr 2026）。C++ 核心，Python/MATLAB 绑定。

**核心类**：`teaser::RobustRegistrationSolver`

**Params 字段**（关键）：
| 字段 | 默认 | 含义 |
|---|---|---|
| `noise_bound` | 0.03 | 测量噪声 $\beta$ 上界（米） |
| `cbar2` | 1.0 | 截断常数 $\bar c^2$ |
| `estimate_scaling` | true | 是否估计尺度 |
| `rotation_estimation_algorithm` | GNC_TLS | `GNC_TLS` / `FGR` |
| `rotation_gnc_factor` | 1.4 | GNC 连续化因子 $\mu$（5-F） |
| `rotation_max_iterations` | 100 | GNC 最大迭代 |
| `rotation_cost_threshold` | 0.005 | GNC 停机阈值 |
| `rotation_tim_graph` | CHAIN | TIM 图形状（`CHAIN` / `COMPLETE` / `FCGR`） |
| `inlier_selection_mode` | PMC_EXACT | Max-clique 求解器：`PMC_EXACT` / `PMC_HEU` / `KCORE_HEU` |
| `kcore_heuristic_threshold` | 0.5 | k-core 启发阈值 |
| `max_clique_time_limit` | 3600 | PMC 时限（秒） |
| `max_clique_num_threads` | 0 | PMC 并行线程 |

**典型调用（C++）**：
```cpp
teaser::RobustRegistrationSolver::Params params;
params.noise_bound = 0.05;
params.cbar2 = 1.0;
params.estimate_scaling = false;
params.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
params.rotation_gnc_factor = 1.4;
params.rotation_max_iterations = 100;

teaser::RobustRegistrationSolver solver(params);
solver.solve(src_cloud, dst_cloud);  // Eigen::Matrix<double,3,N>
auto solution = solver.getSolution();
// solution.scale, solution.rotation, solution.translation
```

**Python（teaserpp_python）**：
```python
import teaserpp_python as tp
s = tp.RobustRegistrationSolver.Params()
s.noise_bound = 0.05
s.cbar2 = 1.0
s.rotation_estimation_algorithm = \
    tp.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
solver = tp.RobustRegistrationSolver(s)
solver.solve(src, dst)
sol = solver.getSolution()
```

**FPFH + 3DSmoothNet 流水线**（`examples/teaser_cpp_fpfh/`）：
```text
点云 → FPFH 特征 → 互最近邻匹配（得到噪声对应）→ TEASER++ → (R,t,s)
```
在 3DMatch 基准上 **平均运行 85 ms**，成功率（rot err < 10° 且 trans err < 30cm）：

| 场景 | RANSAC-1K | RANSAC-10K | TEASER++ | TEASER++ (CERT) |
|---|---|---|---|---|
| Kitchen | 90.9 | 96.4 | 97.7 | **99.2** |
| Home 1 | 91.0 | 92.3 | 92.3 | 97.5 |
| Home 2 | 73.1 | 73.1 | 82.7 | **90.0** |
| Hotel 1 | 88.1 | 92.0 | 96.9 | **98.8** |
| Hotel 2 | 80.8 | 84.6 | 88.5 | **94.9** |

**CERT** 列：只统计 DRS 认证为全局最优的子集——认证后可靠性显著提升（Home 2 从 82.7% → 90.0%），代价是拒绝未认证输出。

**依赖**：CMake, Eigen3, Boost, PMC (max clique), pybind11, Open3D (Python 示例)。

---

### §E.16 Rotation Averaging 的 SDP 松弛 ⭐⭐⭐⭐

**Eriksson-Olsson-Kahl-Chin** "Rotation Averaging with the Chordal Distance: Strong Duality, Tight Relaxation, and Certifiable Algorithms" *IEEE TPAMI* 43(1):256-268, 2021（CVPR 2018 会议版）。

**问题**：给定相对旋转 $\tilde R_{ij}$，最小化 **chordal distance**
$$
\min_{R_i \in SO(d)} \sum_{(i,j)} \|R_j - R_i\tilde R_{ij}\|_F^2.
$$

**关键贡献**：证明这是 SE-Sync 去掉平移因子后得到的**纯旋转**版本；在 chordal distance 下 **Lagrangian 强对偶成立**的**显式噪声阈值**；给出基于 $\lambda_{\min}$ 的 certifier。这里不是令 $\tau\to\infty$：$\tau$ 是平移测量精度，趋于无穷表示平移约束被无限强化，而不是消失。

这是 Carlone-Tron-Daniilidis-Dellaert *CVPR* 2015 "Initialization techniques for 3D SLAM" 的**理论化**——后者给了经验 chordal 初始化，前者证明它在低噪声下是**全局最优**。

---

### §E.17 Category-Level 姿态估计——PACE ⭐⭐⭐⭐

**Shi-Yang-Carlone** "Optimal Pose and Shape Estimation for Category-level 3D Object Perception" *RSS* 2021 Best Paper Finalist (arXiv:2104.08383)。

**问题**：给定一组同类别 3D 关键点观测和类别先验（CAD 模型的主动形状空间 $\sum_k \beta_k B_k$），**联合**估计物体姿态 $(R, t)$ 和形状系数 $\beta$：
$$
\min_{R, t, \beta \ge 0,\,\|\beta\|_1 = 1} \sum_i \Big\|y_i - R\Big(\sum_k \beta_k B_k^{(i)}\Big) - t\Big\|^2.
$$
在 simplex 约束下 $\|\beta\|_1=1$ 已经是常数；若要表达形状先验，应使用真正有作用的二次先验或类别协方差项，例如 $\lambda\|\beta-\bar\beta\|^2$ 或 $\lambda\beta^\top\Sigma_\beta^{-1}\beta$。
四次耦合（$R \cdot \beta$）→ TLS + Shor SDP 松弛 → 全局最优。

**PACE3D\*** 和 **PACE2D\***（带 2D 投影）都是 certifiable。**ROBIN**（Shi-Yang-Carlone *ICRA* 2021）是**基于图的 TLS 外点剪枝**，类似 TEASER 的 max-clique 但用 **compatibility hypergraph** 推广到非对偶不变量情形。

---

### §E.18 Outlier-Robust 估计的统一 SDP 框架 ⭐⭐⭐⭐

**Yang-Carlone** "Certifiably Optimal Outlier-Robust Geometric Perception" *IEEE TPAMI* 2023 (arXiv:2109.03349)。

**统一化贡献**：TLS、Max-consensus、Geman-McClure、Tukey 等损失都可以写成**多项式优化问题 (POP)**；引入 **sparse semidefinite relaxation (SSR)**——只用 Shor 的**对角块**稀疏结构；用 **STRIDE** 求解器。

**STRIDE 架构**：
1. **Global step**：SDP 对偶下降（SCS/MOSEK）获得候选最优值的下界；
2. **Local step**：Riemannian/NLP 局部搜索获得原问题候选解（上界）；
3. **Certify**：若上下界间隙 $< \epsilon$，认证全局最优；否则返回 global step。

**实验规模**：6 个感知问题（点云配准、mesh registration、绝对姿态估计、3 种旋转 averaging）在 60-90% 外点下 SSR 经验紧；STRIDE 比 MOSEK 快 **~100×**；**唯一**能以高精度求解数十万约束规模的 SDP 求解器。

**GNC vs SDP 的理论联系**（Yang-Antonante-Tzoumas-Carlone *RA-L* 2020, arXiv:1909.08605）：GNC-TLS 可看作 SDP 对偶下降的"一阶近似"——GNC 每次外层迭代相当于在当前权重下求解凸子问题，而 SDP 对偶给出**全局**权重。在低外点率下两者收敛到同一解；但高外点率下 GNC 可能陷入局部，SDP/STRIDE 仍全局。

**Estimation Contracts**（Carlone *FnT in Robotics* 11(2-3):90-224, 2023, arXiv:2208.10521）：给 certifiable 方法建立**非渐近 error bound** 形式化——"estimation contract" 是一对 (输入假设, 输出误差界)。例如"若噪声 $\le \sigma$、外点率 $\le \rho$，则 $\|\hat R - R^\circ\|_F \le f(\sigma, \rho, n)$"。这是 certifiable perception 的**理论完备化**。

---

### §E.19 SLAM 中的更多 certifiable 方法 ⭐⭐⭐

| 方法 | 论文 | 贡献 |
|---|---|---|
| **Cartan-Sync** | Briales-González-Jiménez *RA-L/IROS* 2017 | 在 $SE(d)^n$ 上直接 SDP，不消 $t$ |
| **SE-Sync** | Rosen et al. *IJRR* 2019 | 消 $t$ + Stiefel 流形 BM |
| **DC²-PGO** | Tian et al. *T-RO* 2021 | 分布式 certifiable PGO |
| **PACE** | Shi-Yang-Carlone *RSS* 2021 | 形状 + 姿态联合 |
| **QUASAR** | Yang-Carlone *ICCV* 2019 | 外点鲁棒 Wahba |
| **TEASER** | Yang-Shi-Carlone *T-RO* 2021 | 点云配准 |
| **STRIDE** | Yang-Carlone *TPAMI* 2023 | 通用 SDP 求解器 |
| **Anonymous bearing** | Fan-Murphey *RA-L* 2019 | 未知对应的 mutual localization |

**Rosen-Doherty-Terán Espinoza-Leonard** "Advances in Inference and Representation for SLAM" *ARCRAS* 4:215-242, 2021 (arXiv:2103.05041) 是目前**最权威**的 certifiable SLAM 综述，section on "Certifiably Correct Estimation" 给出完整技术路线图。

---

### §E.20 核心方法对比表 ⭐⭐

| 方法 | 问题类型 | 全局最优 | 需初值 | 鲁棒性 | 复杂度 | 规模 | 代表论文 | 代码 |
|---|---|---|---|---|---|---|---|---|
| GN/LM | 非线性 LSQ | ✗ 局部 | **是** | 无 | $O(n^{1.5})$ per iter | $10^6$ | GTSAM, Ceres, g2o | 成熟 |
| iSAM2 | 增量 NLSQ | ✗ 局部 | 是 | 无 | 局部更新；2D pose graph 常见约 $O(\sqrt N)$，最坏退化为 batch | $10^6$ | Kaess et al. | GTSAM |
| **SE-Sync** | PGO | ✓ **证书** | **不** | 无 | Stiefel RTR | $10^4$-$10^5$ | Rosen *IJRR* 2019 | `david-m-rosen/SE-Sync` |
| **Cartan-Sync** | PGO | ✓ 证书 | 不 | 无 | $SE(d)$ BM | $10^4$ | Briales *RA-L* 2017 | (部分开源) |
| GNC-TLS | 鲁棒 LSQ | ✗ | 是（可差） | **高** | GN 内层 + 外层连续化 | $10^5$ | Yang *RA-L* 2020 | TEASER/Kimera |
| **TEASER++** | 3D 配准 | ✓ 证书（DRS 验证） | 不 | **极高（>99%外点）** | ms 级 | $10^3$-$10^4$ 对应 | Yang *T-RO* 2021 | `MIT-SPARK/TEASER-plusplus` |
| **STRIDE** | 通用 TLS | ✓ 证书 | 不 | 高 | global-local 迭代 | $10^5$-$10^6$ 约束 | Yang *TPAMI* 2023 | `MIT-SPARK/CertifiablyRobustPerception` |
| **DC²-PGO** | 分布式 PGO | ✓ 证书 | 不 | 无 | 分布式 RBCD | $10^4$/node | Tian *T-RO* 2021 | 开源 |
| Kimera-RPGO | PCM + SE-Sync | ✓ + 鲁棒 | 不 | 高 | 组合 | $10^4$ | Rosinol *ICRA* 2020 | `MIT-SPARK/Kimera-RPGO` |

---

### §E.21 核心文献表（含具体位置） ⭐

**教材**：
- **Boyd-Vandenberghe** *Convex Optimization* CUP 2004：§4.4 QCQP (pp.152-160), §4.6 SDP (pp.167-172), §5 Duality (pp.215-273), §5.2.3 Slater (p.226), §5.5 KKT (pp.241-244), §5.9 conic duality (pp.265-267), §A.5 PSD (pp.646-651). PDF: `web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf`.
- **Absil-Mahony-Sepulchre** *Optimization Algorithms on Matrix Manifolds* Princeton 2008：Stiefel manifold (Ch.3.3.2), retraction (Ch.4), RTR (Ch.7), vector transport (Ch.8).
- **Lasserre** *Moments, Positive Polynomials and Their Applications* ICP 2010：moment/SOS hierarchy（Shor 是第一层）。
- **Carlone** *Estimation Contracts for Outlier-Robust Geometric Perception* *FnT in Robotics* 11(2-3), 2023（arXiv:2208.10521）：certifiable + robust 的统一教科书。

**核心论文**：
- Vandenberghe-Boyd "Semidefinite Programming" *SIAM Review* 38(1):49-95, 1996：SDP 经典综述。
- Shor "Quadratic optimization problems" *Soviet J. CSS* 25:1-11, 1987：原始 QCQP 对偶。
- Goemans-Williamson "Improved approximation algorithms for maximum cut..." *JACM* 42(6):1115-1145, 1995：0.878 近似。
- Luo-Ma-So-Ye-Zhang "Semidefinite relaxation of quadratic optimization problems" *IEEE SPM* 27(3):20-34, 2010：SDR 综述。
- Burer-Monteiro *Math. Program.* 95(2):329-357, 2003; 103:427-444, 2005：BM 方法。
- **Boumal-Voroninski-Bandeira** "Deterministic guarantees for Burer-Monteiro factorizations" *CPAM* 73(3):581-608, 2020 (arXiv:1804.02008)：$p(p+1)/2 > m$ 界。
- Waldspurger-Waters *SIOPT* 30(3):2577-2602, 2020：BM 界紧性（反例）。
- Bandeira-Boumal-Singer "Tightness of the maximum likelihood SDP for angular synchronization" *Math. Prog.* 2017：noiseless tight 证明模板。
- Carlone-Censi *IEEE T-RO* 30(2):475-492, 2014：$2^n$ 局部极小。
- Carlone-Calafiore-Tommolillo-Dellaert "Planar Pose Graph Optimization" *IEEE T-RO* 32(3):545-565, 2016：2D PGO SDP 紧性 + SZEP。
- **Rosen-Carlone-Bandeira-Leonard** "SE-Sync" *IJRR* 38(2-3):95-125, 2019 (arXiv:1612.07386)：3D PGO certifiable。
- Briales-González-Jiménez "Cartan-Sync" *IEEE RA-L* 2(4):2127-2134, 2017。
- Eriksson-Olsson-Kahl-Chin "Rotation Averaging with Chordal Distance" *IEEE TPAMI* 43(1):256-268, 2021。
- **Yang-Shi-Carlone** "TEASER" *IEEE T-RO* 37(2):314-333, 2021 (arXiv:2001.07715)。
- Yang-Carlone "QUASAR" *ICCV* 2019 (arXiv:1905.12536)：外点鲁棒 Wahba。
- Yang-Antonante-Tzoumas-Carlone "GNC" *IEEE RA-L* 2020 (arXiv:1909.08605) — ICRA 2020 Robot Vision Best Paper。
- **Yang-Carlone** "Certifiably Optimal Outlier-Robust Geometric Perception" *IEEE TPAMI* 2023 (arXiv:2109.03349)：STRIDE + SSR。
- Shi-Yang-Carlone "PACE" *RSS* 2021 (arXiv:2104.08383)。
- Tian-Khosoussi-Rosen-How "DC²-PGO" *IEEE T-RO* 37(6):2137-2156, 2021 (arXiv:1911.05864)。
- Rosen-Doherty-Terán Espinoza-Leonard "Advances in Inference and Representation for SLAM" *ARCRAS* 4:215-242, 2021 (arXiv:2103.05041)：综述。

---

### §E.22 C++/Python 库映射 ⭐⭐

| 库 | 语言 | 功能 | 规模 |
|---|---|---|---|
| **SE-Sync** (`david-m-rosen/SE-Sync`) | C++ | 3D PGO certifiable | $n \le 10^5$ |
| **TEASER++** (`MIT-SPARK/TEASER-plusplus`) | C++/Py/MATLAB | 3D 配准 certifiable | $10^4$ 对应 |
| **CertifiablyRobustPerception** (`MIT-SPARK`) | MATLAB/Py | STRIDE + SSR | 通用 TLS |
| **Kimera-RPGO** (`MIT-SPARK/Kimera-RPGO`) | C++ | PCM + SE-Sync | SLAM 后端 |
| **Manopt / Pymanopt** (`manopt.org`) | MATLAB/Py | Riemannian 优化通用 | BM 开发 |
| **ROPTLIB** (`whuang08/ROPTLIB`) | C++ | Riemannian 优化 | SE-Sync 依赖 |
| **MOSEK** (`mosek.com`) | C/Py/MATLAB | 商业 IPM SDP | $n \lesssim 5 \times 10^3$ |
| **SeDuMi / SDPT3** | MATLAB | 经典 IPM SDP | 中等规模 |
| **SCS** (`cvxgrp/scs`) | C/Py | 一阶 ADMM SDP | $10^5$-$10^6$ 低精度 |
| **CVXPY / YALMIP** | Py/MATLAB | 凸建模前端 | 原型验证 |
| **Spectra** | C++ | Lanczos 特征值 | 证书验证 |
| **PMC** | C++ | Max-clique | TEASER 剪枝 |
| **GTSAM** | C++/Py | 因子图（与 SE-Sync 对接） | $10^6$ |
| **Ceres / g2o** | C++ | NLSQ | 对比基线 |

---

### §E.23 十条常见陷阱 ⭐⭐

**陷阱 1 — SDP 松弛不一定紧**。Shor 给出 $p^\star_{\text{SDR}} \le p^\star_{\text{QCQP}}$，**相等只是经验结果**。旋转噪声 $\sigma_\theta > 0.3$ rad 或外点率 >95% 时 SE-Sync/TEASER 可能不紧；此时证书 $\lambda_{\min} < 0$，`cert = false`。必须做 **rejection**，不能把未认证输出当全局最优报告。

**陷阱 2 — Burer-Monteiro 秩 $p$ 选择影响收敛**。$p$ 太小（$< \sqrt{2m}$）时 BVB 保证失效，可能有伪局部极小；$p$ 太大浪费计算。SE-Sync Riemannian Staircase 从 $p_0 = 5$ 开始自适应爬楼梯——**不要固定 $p = d$**，会遇到秩不足导致非全局的临界点。

**陷阱 3 — SE-Sync 对平移的处理**。SE-Sync 把 $t$ **完全解析消去**（Problem 3），只解旋转 SDP 再回代。Cartan-Sync 保留 $t$ 但变量量翻倍。若你实现时忘了消 $t$，复杂度骤升。同时注意：消去公式 $t^\star = -\operatorname{vec}(R^\star \tilde V^\top L(W_\tau)^\dagger)$ 需要 $L(W_\tau)$ **伪逆**——连通图保证 kernel 是 1 维（$\mathbf{1}$ 向量），数值上需选 anchor pose（$t_1 := 0$）。

**陷阱 4 — TEASER 尺度估计对外点敏感**。TRIMs $s_{ij}$ 需要两点（$i, j$ 都是内点）才无偏；内点比例 $\rho$ 时两点都内点的比例 $\rho^2$。外点率 95% 时只有 $0.05^2 = 0.25\%$ 的 TIM 可用——所以 TEASER 必须先做 **max-clique 剪枝**。跳过剪枝直接 adaptive voting 会失败。

**陷阱 5 — SDP 求解器精度设置影响证书验证**。SCS 默认精度 $10^{-3}$，对 $\lambda_{\min} \ge -\epsilon$ 判定不够；需要 $\epsilon_{\text{dual}} \le 10^{-8}$ 级才稳定。MOSEK 默认达得到；SCS 需手动设 `eps_abs = eps_rel = 1e-9` 且问题规模 $\lesssim 10^4$ 才有效。大规模问题必须用 Burer-Monteiro。

**陷阱 6 — 内点法对 SLAM 规模不可行**。$n = 10^4$ 位姿 → SDP 变量 $Z$ 维 $3n \times 3n = 30000 \times 30000$ → 稠密 IPM 单步 $O(9 \times 10^{18})$ 乘法，显存 $O(10^{10})$ 字节——**完全不可行**。BM 是唯一路径。

**陷阱 7 — certifiable 方法目前是 batch 的**。SE-Sync 每次都要重算 $\tilde Q$ 稀疏分解、RTR 全部迭代——不支持 iSAM2 风格的增量更新。**实际集成模式**：iSAM2 做在线 + 定期触发 SE-Sync batch（例如每 100 关键帧或大回环后），把 SE-Sync 结果注入 `PriorFactor` 重启 iSAM2。研究前沿：**Incremental SE-Sync**（open problem）。

**陷阱 8 — TEASER++ 的 noise_bound 对成功率极敏感**。`noise_bound` 设得太小 → 真内点被误判外点；太大 → 真外点混入。实际应用要基于传感器 spec（LiDAR $\pm$ 3cm、FPFH 匹配噪声）分析性设定。经验：3DMatch 用 `noise_bound = 0.05`；KITTI LiDAR 用 `0.02`。

**陷阱 9 — BM 二阶临界点 $\ne$ 全局最优（非秩亏时）**。BVB 定理 (a) 要求**秩亏**二阶临界点；RTR 收敛到秩满的二阶临界点时，根据 (b) 对"几乎所有 $C$"仍全局最优，但存在 Lebesgue 零测集反例（Waldspurger-Waters）。**实践**：若 $Y^\star$ 满秩，爬一级楼梯 $p \leftarrow p+1$ 再优化；仍满秩则报告不紧。

**陷阱 10 — 对偶证书验证的数值误差**。$\lambda_{\min}(S)$ 的 Lanczos 估计有绝对误差 $\epsilon_{\text{Lanczos}} \approx 10^{-10}\|\tilde Q\|_2$。若 $\tilde Q$ 条件数 $\kappa(\tilde Q) = 10^{12}$（大规模、差尺度测量），数值上无法区分 $\lambda_{\min} = 0$ 与 $\lambda_{\min} = -10^{-6}$。**稳健做法**：同时检查 $(F(\tilde Q \hat R^\top \hat R) - p^\star_{\text{SDP}})/p^\star_{\text{SDP}} < 10^{-6}$（相对 suboptimality）作为 **primal-dual gap** 验证，与 $\lambda_{\min}$ 双重认证。

---

### §E.24 对偶证书的完整手推：从一个 QCQP 到全局最优证明 ⭐⭐⭐⭐

对偶证书最容易被误解成"求解器输出的一个诊断数"。

实际上它是一段完整的数学证明。

这一节用最小形式把证明写出来。

考虑等式约束 QCQP：

$$
\begin{aligned}
p^\star=\min_x\quad & x^\top Qx\\
\text{s.t.}\quad & x^\top A_i x=b_i,\qquad i=1,\ldots,m.
\end{aligned}
$$

设已经有一个候选可行解 $\hat x$。

目标是证明：

$$
\forall x\ \text{可行},\qquad x^\top Qx\ge \hat x^\top Q\hat x.
$$

这就是全局最优。

##### 第一步：构造 Lagrangian

把约束移到目标中：

$$
L(x,\lambda)
=
x^\top Qx-\sum_{i=1}^{m}\lambda_i(x^\top A_i x-b_i).
$$

整理：

$$
L(x,\lambda)
=
x^\top\left(Q-\sum_i\lambda_i A_i\right)x
+\sum_i\lambda_i b_i.
$$

定义

$$
S(\lambda)=Q-\sum_i\lambda_i A_i.
$$

则

$$
L(x,\lambda)=x^\top S(\lambda)x+\lambda^\top b.
$$

##### 第二步：如果 $S(\lambda)\succeq0$，就得到全局下界

若 $S(\lambda)\succeq0$，则对任意 $x$：

$$
x^\top S(\lambda)x\ge0.
$$

因此

$$
L(x,\lambda)\ge \lambda^\top b.
$$

对任意可行 $x$，约束满足 $x^\top A_i x=b_i$，所以

$$
L(x,\lambda)=x^\top Qx.
$$

于是

$$
x^\top Qx\ge \lambda^\top b.
$$

这说明 $\lambda^\top b$ 是原问题最优值的下界。

##### 第三步：让下界碰到候选解

如果还能满足

$$
S(\lambda)\hat x=0,
$$

则

$$
\hat x^\top Q\hat x
=
\hat x^\top S(\lambda)\hat x+\lambda^\top b
=
\lambda^\top b.
$$

结合第二步：

$$
\forall x\ \text{可行},\qquad
x^\top Qx\ge \lambda^\top b=\hat x^\top Q\hat x.
$$

因此 $\hat x$ 是全局最优。

这就是证书。

它只需要验证三件事：

| 条件 | 含义 | 工程检查 |
|---|---|---|
| primal feasible | $\hat x$ 满足原约束 | 投影到 $SO(3)$、检查块正交 |
| stationarity | $S(\hat\lambda)\hat x=0$ | KKT 残差范数 |
| dual feasible | $S(\hat\lambda)\succeq0$ | 最小特征值 $\lambda_{\min}\ge-\epsilon$ |

只要三件事成立，证明就完成。

无需相信求解器。

证书是可独立验证的数学对象。

> **本质洞察**：可认证感知的"认证"不是说优化过程可靠，而是说候选解携带了一个可检查的全局下界；当这个下界等于候选解代价时，任何其他解都不可能更好。

##### 一个二维单位圆例子

考虑

$$
\min_{x\in\mathbb R^2} x^\top Qx
\quad\text{s.t.}\quad x^\top x=1.
$$

这是 Rayleigh quotient。

全局最优是 $Q$ 的最小特征向量。

约束矩阵为 $A=I$，$b=1$。

Lagrangian 为

$$
L(x,\lambda)=x^\top(Q-\lambda I)x+\lambda.
$$

若 $\hat x$ 是最小特征向量，$Q\hat x=\lambda_{\min}(Q)\hat x$。

取 $\hat\lambda=\lambda_{\min}(Q)$。

则

$$
S=Q-\hat\lambda I\succeq0,
$$

并且

$$
S\hat x=0.
$$

因此对任意单位向量 $x$：

$$
x^\top Qx
=
x^\top Sx+\hat\lambda
\ge
\hat\lambda
=
\hat x^\top Q\hat x.
$$

这个例子就是 SE-Sync 证书的低维影子。

SE-Sync 只是把"单位圆约束"换成了每个旋转块的 $R_i^\top R_i=I$，把标量 $\lambda$ 换成块对角矩阵 $\Lambda$。

##### 为什么 $S\hat x=0$ 可从 KKT 构造

若 $\hat x$ 是一阶驻点，则

$$
\nabla_x L(\hat x,\lambda)
=
2S(\lambda)\hat x=0.
$$

因此乘子 $\lambda$ 要让 $\hat x$ 落在 $S$ 的零空间中。

在 SE-Sync 中，这个条件给出显式公式：

$$
\Lambda^\star
=
\operatorname{SymBlockDiag}_d(\tilde Q\,\hat R^\top\hat R).
$$

它的作用与 Rayleigh quotient 中的

$$
\hat\lambda=\hat x^\top Q\hat x
$$

完全类似。

都是从候选解的一阶最优性反推对偶变量。

##### 与 SDP 松弛的关系

Shor 松弛的 primal 是

$$
\min_Z \langle Q,Z\rangle
\quad\text{s.t.}\quad
\langle A_i,Z\rangle=b_i,\quad Z\succeq0.
$$

其 dual 是

$$
\max_\lambda \lambda^\top b
\quad\text{s.t.}\quad
Q-\sum_i\lambda_iA_i\succeq0.
$$

也就是找一个 PSD 的 $S(\lambda)$。

所以对偶证书既是原 QCQP 的全局下界，也是 SDP 对偶可行解。

当候选解 $\hat x$ 使 $Z=\hat x\hat x^\top$ 可行，并且 primal value 等于 dual value，就有零对偶间隙。

这同时证明：

1. SDP 松弛紧；
2. $\hat x$ 是 QCQP 全局最优；
3. $Z=\hat x\hat x^\top$ 是 SDP 的 rank-1 最优解。

##### 常见误区

| 误区 | 正确理解 |
|---|---|
| 只要求解 SDP 就一定全局最优 | SDP 给下界，只有 rank 条件或证书通过才恢复原问题全局解 |
| $\lambda_{\min}<0$ 说明候选解一定错 | 只能说明当前证书失败，可能是数值误差或松弛不紧 |
| 证书由同一求解器给出所以不独立 | $S\succeq0$ 可由独立 Lanczos/特征值检查 |
| BM 非凸所以没有意义 | BM 负责找候选，证书负责验证候选 |

##### 练习

1. 对 $Q=\begin{bmatrix}2&1\\1&3\end{bmatrix}$，手算单位圆 QCQP 的对偶证书。
2. 证明若 $S\succeq0$ 且 $S\hat x=0$，则 $\hat x$ 的代价等于对偶下界。
3. 解释为什么 $S$ 的零空间维度通常等于 gauge 维度，而不是 0。

---

### §E.25 SE-Sync 中消去平移的逐步推导：为什么证书只看旋转 ⭐⭐⭐⭐

SE-Sync 最重要的技巧是先消去平移。

这一步常被写成一个公式，但它直接决定了问题规模。

单条边 $(i,j)$ 的平移残差为

$$
r^t_{ij}=t_j-t_i-R_i\tilde t_{ij}.
$$

平移代价为

$$
F_t(t,R)
=
\sum_{(i,j)\in E}\tau_{ij}\|t_j-t_i-R_i\tilde t_{ij}\|^2.
$$

对固定的旋转 $R$，这是关于 $t$ 的线性最小二乘。

因此可解析求最优 $t^\star(R)$。

##### 矩阵化

令 $t=[t_1^\top,\ldots,t_n^\top]^\top\in\mathbb R^{dn}$。

对每条有向边 $e=(i,j)$，关联矩阵 $A$ 的一行在 $i$ 列为 $-1$，在 $j$ 列为 $+1$。

加权后，有

$$
(A\otimes I_d)t
-
\tilde T\,\operatorname{vec}(R)
$$

其中 $\tilde T$ 把每条边的 $R_i\tilde t_{ij}$ 收集起来。

平移代价写为

$$
F_t(t,R)
=
\|(W_\tau^{1/2}A\otimes I_d)t
-W_\tau^{1/2}\tilde T\,\operatorname{vec}(R)\|^2.
$$

为简化记号，设

$$
B=W_\tau^{1/2}(A\otimes I_d),\qquad
c(R)=W_\tau^{1/2}\tilde T\,\operatorname{vec}(R).
$$

问题变成

$$
\min_t \|Bt-c(R)\|^2.
$$

正规方程为

$$
B^\top Bt=B^\top c(R).
$$

$B^\top B$ 是加权图 Laplacian 与 $I_d$ 的 Kronecker 积：

$$
B^\top B=L(W_\tau)\otimes I_d.
$$

由于全局平移 gauge，连通图的 Laplacian 有一维零空间 $\mathbf 1$。

因此用伪逆或固定一个 anchor：

$$
t^\star(R)
=
(B^\top B)^\dagger B^\top c(R).
$$

##### 回代得到纯旋转二次型

线性最小二乘的最小残差平方可以写成投影形式：

$$
\min_t\|Bt-c\|^2
=
\|(I-BB^\dagger)c\|^2.
$$

定义投影矩阵

$$
\Pi=I-BB^\dagger.
$$

则

$$
F_t^\star(R)
=
c(R)^\top\Pi c(R).
$$

由于 $c(R)$ 对 $\operatorname{vec}(R)$ 是线性的，存在矩阵 $\tilde Q_\tau$ 使

$$
F_t^\star(R)
=
\operatorname{tr}(\tilde Q_\tau R^\top R).
$$

旋转测量项本来也可写成

$$
F_R(R)=\operatorname{tr}(L(\tilde G^\rho)R^\top R).
$$

两者相加：

$$
\boxed{
\tilde Q
=
L(\tilde G^\rho)+\tilde Q_\tau.
}
$$

于是 SE(d) PGO 被化为纯旋转 QCQP：

$$
\min_{R_i\in SO(d)}
\operatorname{tr}(\tilde Q R^\top R).
$$

平移在求出旋转后回代：

$$
t^\star
=
(B^\top B)^\dagger B^\top c(R^\star).
$$

##### 为什么这一步重要

| 若不消去平移 | 消去平移后 |
|---|---|
| 优化变量含 $t$ 与 $R$ | 只剩 $R$ |
| SDP 变量更大 | SDP/BM 变量显著缩小 |
| 平移 gauge 直接进入证书 | 证书集中在旋转块 |
| Cartan-Sync 风格 | SE-Sync 风格 |

这不是近似。

因为平移子问题对固定旋转是凸二次，解析消元得到的是原问题的 exact variable projection。

##### 稀疏实现细节

直接形成 $\Pi=I-BB^\dagger$ 会很稠密。

SE-Sync 使用稀疏分解绕开显式投影。

令

$$
B=LQ_1
$$

是稀疏 LQ 分解。

则 $\Pi$ 可通过 $Q_2Q_2^\top$ 的隐式形式应用。

在代码中，真正需要的是 $\tilde Q$ 与向量相乘、代价、梯度和 Hessian-vector product。

这些操作都可以通过稀疏矩阵和三角求解完成。

##### 常见误区

| 误区 | 后果 | 正确做法 |
|---|---|---|
| 显式求 $L(W_\tau)^\dagger$ | 稠密矩阵爆内存 | 固定 gauge 或稀疏 LQ/Cholesky |
| 忘记平移 gauge | 正规方程奇异 | 锚定一个 pose 或使用伪逆 |
| 把消平移当近似 | 误判 SE-Sync 精度 | 这是固定 $R$ 下的精确最小化 |
| 只看旋转不回代平移 | 输出不完整 PGO | 旋转认证后必须恢复 $t^\star$ |

##### 练习

1. 对三节点一维平移链写出 $A$ 与 $L=A^\top A$，手算固定 $R$ 时的 $t^\star$。
2. 证明 $\min_t\|Bt-c\|^2=\|(I-BB^\dagger)c\|^2$。
3. 解释为什么图不连通时 SE-Sync 需要对每个连通分量分别处理 gauge。

---

### §E.26 证书验证的数值流程：$\lambda_{\min}$、rank 与 primal-dual gap ⭐⭐⭐

数学上只要 $S\succeq0$。

计算机上只能验证近似半正定。

因此可认证算法必须定义数值容差。

##### 最小特征值检查

证书矩阵为

$$
S=\tilde Q-\Lambda^\star.
$$

它通常很大但稀疏。

不需要完整特征分解，只需要最小特征值。

常用 Lanczos 或 LOBPCG 计算

$$
\lambda_{\min}(S).
$$

若

$$
\lambda_{\min}(S)\ge -\epsilon_{\text{eig}},
$$

则视为 dual feasible。

典型容差不是固定的绝对数，而应与矩阵尺度相关：

$$
\epsilon_{\text{eig}}
=
10^{-8}\max(1,\|\tilde Q\|_2).
$$

如果 $\tilde Q$ 条件数很差，单看 $\lambda_{\min}$ 容易误判。

##### Rank 检查

SDP 解 $Z=Y^\top Y$ 应该是 rank $d$。

BM 求得的是 $Y\in\mathbb R^{r\times dn}$。

若奇异值满足

$$
\sigma_{d+1}(Y)\le \epsilon_{\text{rank}}\sigma_d(Y),
$$

则认为秩亏到 $d$。

Riemannian Staircase 的逻辑是：

1. 若 $Y$ 秩亏，依据 BVB 命题得到 SDP 全局最优。
2. 从 $Y$ 舍入恢复 $R$。
3. 再用对偶证书验证 $R$。

秩亏是理论 sufficient condition。

对偶证书是工程最终检查。

两者最好同时满足。

##### Primal-dual gap

primal 候选代价为

$$
p=\operatorname{tr}(\tilde Q \hat R^\top\hat R).
$$

dual 下界为

$$
d=\operatorname{tr}(\Lambda^\star).
$$

相对间隙为

$$
\text{gap}
=
\frac{p-d}{\max(1,|p|)}.
$$

若

$$
\text{gap}\le\epsilon_{\text{gap}},
$$

说明下界与候选解相遇。

如果 $\lambda_{\min}$ 略负但 gap 很小，可能是数值误差。

如果 $\lambda_{\min}$ 接近 0 但 gap 大，说明 KKT 或舍入不一致。

##### KKT 残差

还应检查

$$
\|S\hat R^\top\|_F.
$$

它衡量 stationarity。

若这个值大，说明 $\Lambda^\star$ 没有让候选解落在证书矩阵零空间中。

可能原因：

| 原因 | 现象 |
|---|---|
| RTR 没收敛 | gradient norm 大 |
| rounding 破坏最优性 | $Y$ 到 $R$ 的投影误差大 |
| 数值尺度差 | KKT 残差集中在低权重边 |
| 外点未处理 | 证书矩阵出现明显负特征值 |

##### 认证结果的三种状态

可认证算法返回的不应只是成功/失败。

更清晰的是三态：

| 状态 | 条件 | 解释 |
|---|---|---|
| Certified | feasibility、stationarity、PSD、gap 均过阈 | 原问题全局最优 |
| Uncertified feasible | 候选解可行但证书失败 | 可能是局部解，也可能松弛不紧 |
| Numerical failure | 可行性或 KKT 严重不满足 | 求解器未收敛或模型错误 |

工程系统应只把 Certified 结果作为强全局结论。

Uncertified feasible 可以作为初值或候选，但不能写成全局最优。

##### 与日志指标的对应

| 指标 | 应看什么 |
|---|---|
| `lambda_min(S)` | 是否 $\ge -\epsilon$ |
| `suboptimality_bound` | primal-dual gap |
| `rank(Y)` | 是否降到 $d$ |
| `grad_norm` | RTR 是否收敛 |
| `rounding_error` | BM 解是否接近 rank-$d$ |
| `num_staircase_steps` | 是否需要升秩 |

##### 练习

1. 给定一组奇异值 $[10,9,8,10^{-7},10^{-8}]$，判断 $d=3$ 时是否 rank deficient。
2. 解释为什么 $\lambda_{\min}=-10^{-10}$ 不一定意味着证书失败。
3. 设计一个日志表，记录 SE-Sync 每级 staircase 的 rank、cost、grad norm 与 $\lambda_{\min}$。

---

### §E.27 Certifiable 与鲁棒方法的边界：SDP、GNC、RANSAC、PCM 如何组合 ⭐⭐⭐

可认证方法解决的是非凸全局性。

鲁棒方法解决的是外点污染。

两者经常同时需要，但不能混为一谈。

##### 四类方法的角色

| 方法 | 解决的问题 | 输出 |
|---|---|---|
| RANSAC | 从大量候选对应中快速找一个内点一致模型 | 初值或内点集合 |
| PCM | 对回环集合做图一致性筛选 | 一组两两一致回环 |
| GNC-TLS | 在连续优化中逐步二值化内外点权重 | 鲁棒局部/准全局解 |
| SDP/证书 | 验证非凸问题的全局最优 | 全局最优证明或拒绝 |

RANSAC 与 PCM 是优化前的组合筛选。

GNC 是优化中的权重退火。

SDP 证书是优化后的全局验证。

##### 典型 SLAM 后端组合

多机器人地图合并中，候选回环外点率可能超过 80%。

直接 SE-Sync 会把错误回环当成真实测量，得到一个被污染的全局最优。

这不是 SE-Sync 失败，而是模型错了。

正确流程是：

```text
候选回环
  -> 几何验证 / RANSAC
  -> PCM 最大团筛选
  -> GNC-TLS 进一步软/硬加权
  -> SE-Sync 或 batch LM 求解
  -> 对偶证书验证
```

如果 PCM 已经把外点清干净，SE-Sync 的证书说明"在剩余测量模型下全局最优"。

如果外点仍在，证书只会证明一个错误模型的全局最优。

这点非常关键。

证书认证的是优化问题，不认证数据关联的真实性。

##### TEASER++ 的组合逻辑

TEASER++ 内部也体现了这种分层：

1. 用 TIM/TRIM 构造不变量；
2. 用最大团剪枝减少外点；
3. 用 GNC-TLS 快速求旋转；
4. 必要时用 DRS/SDP 做认证。

它不是"只靠 SDP 打败 RANSAC"。

真正强的是：不变量把问题解耦，图剪枝降低外点率，GNC 给快速候选，证书给最终可信度。

##### RANSAC 的工程边界

RANSAC 的成功概率为

$$
P=1-(1-w^s)^N,
$$

其中 $w$ 是内点比例，$s$ 是最小样本数，$N$ 是迭代次数。

反解：

$$
N=\frac{\log(1-P)}{\log(1-w^s)}.
$$

若 $w=0.1$，$s=3$，想要 $P=0.99$：

$$
N\approx\frac{\log(0.01)}{\log(1-0.001)}\approx4603.
$$

若 $w=0.01$，$s=3$：

$$
N\approx4.6\times10^6.
$$

这说明高外点率下 RANSAC 迭代数爆炸。

TEASER/PCM 的价值就在于用一致性结构替代盲随机采样。

##### 选择建议

| 场景 | 推荐 |
|---|---|
| 点云粗配准，外点未知 | TEASER++，必要时 RANSAC 仅作前筛 |
| 单机器人回环，外点 <30% | Huber/DCS + iSAM2 |
| 多机器人回环，外点 50-90% | PCM -> GNC-TLS -> SE-Sync |
| 需要论文级全局结论 | SDP/dual certificate 必须报告 |
| 实时控制闭环 | 先用鲁棒局部法，证书放异步线程 |

##### 常见误区

| 误区 | 正确理解 |
|---|---|
| 有证书就不用 RANSAC/PCM | 证书不判断数据关联真假 |
| RANSAC 找到模型就是全局最优 | 它只是随机采样的高概率方法 |
| GNC-TLS 总能认证 | GNC 本身不提供证书，需 SDP/STRIDE |
| PCM 最大团一定是真内点 | 对称环境中错误回环也可能相互一致 |

##### 练习

1. 计算 RANSAC 在 $w=0.05,s=4,P=0.99$ 时需要多少次迭代。
2. 构造一个错误回环彼此一致的对称走廊例子，说明 PCM 最大团可能失败。
3. 解释"SE-Sync 认证的是剩余 PGO，而不是原始前端匹配"这句话。

---

### §E.28 工程边界与开放问题：可认证方法什么时候不应强行使用 ⭐⭐⭐

Certifiable perception 的地位很高，但工程使用必须谨慎。

##### 边界 1：松弛不紧

当噪声很大、外点未剔除、图结构弱连通时，SDP 可能不紧。

表现为：

$$
\operatorname{rank}(Z^\star)>d
$$

或

$$
\lambda_{\min}(S)<-\epsilon.
$$

此时不能把舍入后的解称为全局最优。

它仍可能是很好的初值，但证书失败必须如实报告。

##### 边界 2：batch 计算限制

SE-Sync、STRIDE、TEASER 的认证步骤多为 batch。

在线 SLAM 每帧运行完整认证通常不现实。

合理架构是异步：

| 线程 | 工作 |
|---|---|
| 实时线程 | iSAM2 / ESKF / local BA |
| 后端线程 | GNC/SE-Sync 周期性优化 |
| 验证线程 | dual certificate / DRS / rank check |
| 回注线程 | 用 certified 解重置地图或加先验 |

##### 边界 3：模型族限制

当前最成熟的证书集中在：

1. rotation averaging；
2. pose graph optimization；
3. point cloud registration；
4. category-level keypoint pose。

带投影的完整 BA、动态物体、多模态语义关联、神经隐式地图仍缺少同等级成熟证书。

原因是这些问题含有更复杂的有理函数、遮挡离散变量和非多项式模型。

##### 边界 4：数值尺度

对偶证书依赖半正定检查。

若测量权重跨越 $10^{12}$，证书矩阵会严重病态。

工程中要做：

1. 所有残差白化；
2. 平移与旋转权重按传感器噪声设置；
3. 统一单位；
4. 对极小/极大权重裁剪；
5. 报告条件数或 Lanczos 残差。

##### 边界 5：全局最优不等于任务成功

如果前端给了错误数据关联，优化器可全局最优地满足错误约束。

如果地图环境本身不可观，证书只能证明在 gauge 商空间上的最优。

如果传感器模型错了，高斯 MLE 的全局最优也可能偏离真实世界。

因此可认证方法应与数据关联、观测性分析、鲁棒估计一起使用。

##### 开放问题

| 问题 | 难点 |
|---|---|
| Incremental SE-Sync | 如何增量更新 $\tilde Q$、$Y^\star$ 与证书 |
| Certifiable BA | 投影残差有理函数与深度可见性 |
| Learning + certificates | 神经网络输出不确定性难给硬界 |
| Dynamic scenes | 数据关联与运动分割离散变量 |
| Real-time certificates | PSD 检查与 BM 优化耗时 |

##### 故障排查表

| 症状 | 可能原因 | 排查 |
|---|---|---|
| $\lambda_{\min}$ 明显负 | 松弛不紧或外点未剔除 | 先 PCM/GNC，降低噪声权重 |
| rank 高于 $d$ | BM 秩不足或问题本身不紧 | 提高 staircase rank，检查外点 |
| gap 小但位姿错 | 数据关联错误被全局满足 | 检查前端匹配与 PCM clique |
| 内存爆炸 | 显式 SDP 变量过大 | 使用 BM，不显式形成 $Z$ |
| SE-Sync 比 LM 慢 | 图小、噪声低、LM 初值好 | 只在大回环或需证书时调用 |

##### 练习

1. 设计一个异步 SLAM 架构：实时 iSAM2，每 200 个关键帧触发 SE-Sync，并说明如何回注结果。
2. 给出一个错误数据关联但可被全局优化完美满足的例子，说明证书边界。
3. 讨论完整 BA 为什么比 PGO 更难做 SDP 认证。

---

### §E.29 学习时间预算（档位 4：博士毕业级） ⭐

| 模块 | 内容 | 时间 |
|---|---|---|
| §E.1-E.2 | 局部极限 + certifiable 定义 | 1 h |
| §E.3-E.5 | SDP / QCQP / Shor / 对偶证书 | 3-4 h |
| §E.6 | BM + BVB + Riemannian | 2-3 h |
| §E.7-E.12 | **SE-Sync 九步链 + 四大定理 + 代码** | **4-5 h**（核心） |
| §E.13-E.15 | TEASER++ | 2-3 h |
| §E.16-E.19 | 扩展方法（Rot avg, PACE, STRIDE, DC²-PGO） | 2 h |
| §E.20-E.30 | 对比、陷阱、自测 | 1-2 h |
| **合计** | | **15-20 h** |

**前置**：5-B（因子图 + LSQ）、5-C（iSAM2）、5-F（鲁棒估计）必须先完成。**强烈推荐**：在 sphere2500、torus、parking-garage 上跑一遍 SE-Sync 源码，打印 $\lambda_{\min}(S)$ 观察 100% 紧；在 Bunny 上跑 TEASER++ 观察 99% 外点下的正确配准。

---

### §E.30 自测题（≥6 道） ⭐⭐

1. **写出 3D rotation averaging 的 QCQP 形式**并推导其 Shor SDP 松弛（提示：$R = [R_1^\top, ..., R_n^\top]^\top$ 堆叠；约束 $R_i^\top R_i = I_3$；松弛变量 $Z = RR^\top$ 用块对角约束 $Z_{ii} = I_3$）。对比 MaxCut SDP 的 $X_{ii} = 1$。

2. **推导 Shor 松弛的对偶**。写出 QCQP $\min x^\top Cx$ s.t. $x^\top A_i x = b_i$ 的 Lagrangian、对偶函数、对偶问题；说明"对偶紧 ⟺ SDR 紧 ⟺ SDR 有 rank-1 最优解"的等价性。

3. **解释对偶证书如何验证全局最优**。给定候选解 $\hat x$，如何构造 $\hat \lambda$ 使 $S(\hat \lambda) = C - \sum \hat \lambda_i A_i$？验证 $S \succeq 0$ 需要什么条件？为什么 $S \succeq 0$ 就证明 $\hat x$ 全局最优？给出完整推理。

4. **手推 SE-Sync 的 Problem 3**。从 Problem 1 出发，利用平移 $t$ 的无约束最优性 $\partial L / \partial t = 0$，显式消去 $t$，导出 $\tilde Q = L(\tilde G^\rho) + \tilde \Sigma - \tilde V^\top L(W_\tau)^\dagger \tilde V$。解释为什么 $\tilde Q$ 是稀疏的。

5. **证明 Burer-Monteiro 的 Barvinok-Pataki 秩界**。即：若 SDP 可行集非空且 bounded，则存在最优 $X^\star$ 满足 $\frac{r(r+1)}{2} \le m$，$r = \operatorname{rank}(X^\star)$。（提示：顶点可达性 + $\mathbb{S}^n_+$ 面的维度）。

6. **实操**：下载 `github.com/david-m-rosen/SE-Sync`，在 **sphere2500** 数据集上运行；记录 $p^\star_{\text{SDP}}$、$F(\tilde Q \hat R^\top \hat R)$、$\lambda_{\min}(S)$、总耗时；与 GTSAM 4.0 PDL-GN 对比（对同样 chordal 初始化）。报告：相对 suboptimality gap 是否 $\le 10^{-10}$？SE-Sync 加速比？Riemannian Staircase 爬了几级？

7. **实操**：下载 `MIT-SPARK/TEASER-plusplus`，生成 Bunny 点云的 **99% 外点**合成对应（1000 点，10 真对应 + 990 随机），运行 TEASER++；记录 rotation error、translation error、耗时、DRS 认证状态。与 RANSAC-10K 对比。尝试降到 99.5% 外点，观察何时 TEASER++ 开始失败。

8. **理论题**：证明 SE-Sync Theorem 12 的误差界 $d_O(R, R^\star) \le \sqrt{4dn\|\tilde Q - Q\|_2 / \lambda_{d+1}(Q)}$。提示：用 Davis-Kahan $\sin\Theta$ 定理 + $O(d)^n/O(d)$ 商空间上的 Procrustes 距离。

9. **对比题**：列出 SE-Sync vs g2o LM 在 **parking-garage** 数据集（1661 poses, 6275 edges）上的精度和速度差异；**garage 是 IJRR 2019 Table 2 中唯一 PDL-GN 略快于 SE-Sync 的数据集**——解释为什么（提示：garage 图结构很稀疏但 loop closure 少，PDL-GN 的 Hessian 很容易分解）。

10. **开放题**：目前 certifiable 方法都是 batch 的。设想一个 **incremental SE-Sync** 算法：当新测量到来时，能否复用前一次的 $Y^\star$ 和 $\Lambda^\star$ 做 warm-start？Riemannian Staircase 的秩 $p$ 是否需要重新确定？如何 incremental 地更新对偶证书？这是当前开放问题之一。

---

### 结语：几何感知的"状态估计终点"

5-E 是第五批的**终点**，也是整个经典状态估计范式的**理论顶峰**。它回答了一个曾被视为"只能经验处理"的问题：**非凸 SLAM/感知问题，能否在多项式时间内保证全局最优？** 答案通过 **Shor 松弛 + Burer-Monteiro + 对偶证书**的三步组合给出**肯定的**回答——**当松弛紧时**。

这一回答的深远意义在于三点：**第一，它把状态估计的"收敛到对的地方"问题从"工程调参"变成"数学定理"**——SE-Sync 给了存在性 $\beta$，TEASER 给了外点率阈值，STRIDE 给了统一 POP 框架；**第二，它重塑了工业 SLAM 后端**——Kimera-RPGO 的"PCM + SE-Sync"已成为鲁棒后端范式，TEASER++ 取代 RANSAC 成为配准默认选项；**第三，它为下一代 SLAM 定义了问题**——**incremental certifiable**、**deep learning + SDP**、**neural implicit + certifiable pose**、**分布式多机器人 certifiable**（DC²-PGO 是起点）是目前最热的研究前沿。

但也需诚实指出局限：**松弛不总是紧**（$\sigma_\theta > 0.3$ rad 或外点 >99.5% 时失败）、**batch 限制**（与 iSAM2 的无缝集成是开放问题）、**变量类型受限**（目前主要是 $SO(d)/SE(d)/SO(2)^n$ 等李群，尚未涵盖所有感知问题）、**BM 理论有反例**（Waldspurger-Waters 显示 $p$ 下界本质紧）。5-F 的 GNC-TLS 仍是高外点率 + 无紧性时的"求生工具"，与 certifiable 方法**互补**而非替代。

**理论与工程的汇合处**：当你在 sphere2500 上跑 SE-Sync 看到 $\lambda_{\min}(S) = 4.7 \times 10^{-11}$ 的一瞬间，这不只是一个数值——**这是 150 年来对"非凸估计能否全局最优"的争论的一个具体回答**。Gauss 在 1795 年给了最小二乘，Kalman 在 1960 年给了递推，Kaess 在 2008 年给了增量；2019 年 Rosen 等人给了**证书**。第五批至此结束。接下来的第六批及以后，我们要探索的是这条"证书之路"如何延伸到学习系统、连续时间轨迹、以及更本质的不可观测性与可观测性分析。

---

### §E.31 Certifiable Perception 的完整方法论总结 ⭐⭐⭐

把本章的所有技术组件放在一起，形成一个完整的方法论：

**Step 1：问题形式化**

把几何感知问题写成 QCQP 或更一般的 POP（Polynomial Optimization Problem）形式：

$$\min_{x} x^\top C x \quad \text{s.t.} \quad x^\top A_i x = b_i,\quad i=1,...,m$$

对 PGO：$x$ 包含所有旋转矩阵的列向量堆叠；$C$ 编码测量残差；$A_i$ 编码 $R_i^\top R_i=I_d$ 约束。

**Step 2：构造 SDP 松弛**

通过 Shor lifting $Z=xx^\top\to Z\succeq 0$，得到凸 SDP：

$$\min_Z \langle C, Z\rangle \quad \text{s.t.} \quad \langle A_i, Z\rangle = b_i,\quad Z\succeq 0$$

松弛去掉了 rank-1 约束（使凸化成为可能），但引入了松弛间隙（relaxation gap）的可能。

**Step 3：高效求解**

不直接求解 $O(n^2)$ 变量的 SDP（内点法太慢），而用 Burer-Monteiro 分解 $Z=YY^\top$，$Y\in\mathbb{R}^{n\times p}$，在 Stiefel 流形上用 Riemannian trust-region 求解。实际复杂度从 $O(n^6)$ 降到 $O(n \cdot p^2 \cdot \text{nnz})$，对 PGO 约 $O(n \cdot d^2 \cdot M)$（$M$ 边数）。

**Step 4：恢复原始解**

从 BM 最优 $Y^\star$ 恢复旋转：$R^\star = \text{ProjectToSO}(Y^\star)$（Procrustes 投影或 rounding）。平移从 $\partial L/\partial t=0$ 的闭式解回代。

**Step 5：对偶证书验证**

构造对偶变量 $\lambda^\star$（通常从 BM 的 KKT 条件提取），计算 $S=C-\sum\lambda_i^\star A_i$，检查 $\lambda_{\min}(S)\ge -\epsilon$。若通过 → 解经过认证是全局最优。若不通过 → 报告不确定（可能是松弛不紧、外点太多、或数值精度不够）。

**方法论的适用边界**：

| 条件 | 方法有效 | 方法可能失效 |
|------|---------|-------------|
| 噪声水平 | $\sigma_\theta < 0.3$ rad（PGO） | $\sigma_\theta > 0.3$ rad |
| 外点率 | 先用 PCM/GNC 清理到 <5% | >5% 残余外点可能使松弛不紧 |
| 问题规模 | $n<10^4$ poses（BM 可处理） | $n>10^5$ 可能需要分布式 |
| 变量类型 | $SO(d)$, $SE(d)$, Stiefel | 一般非线性约束（如 BA 的投影） |
| 时间预算 | 秒级（batch 后处理） | 毫秒级实时不可行 |

**与其他方法的互补关系**：

```text
实时前端                  → Visual/LiDAR Odometry（局部精确）
实时后端                  → iSAM2 + Huber/DCS（增量、局部最优）
周期性鲁棒化              → PCM + GNC-TLS（去外点）
全局验证（本章核心）      → SE-Sync + 对偶证书（全局最优 + 认证）
分布式协作               → DC²-PGO（多机器人分布式认证）
```

> **反事实推理——如果没有对偶证书会怎样？** 那么 SE-Sync 只是"另一个非凸优化器"——在 BM 分解后的 Riemannian 优化仍然可能陷入鞍点（虽然 BVB 定理说"几乎所有代价"下二阶临界点 = 全局最优，但"几乎所有"不等于"你的实例"）。对偶证书的价值在于：**它把"几乎所有"变成"你的实例确实是"**——从概率保证升级为确定性保证。这就是 certifiable 方法与纯 Riemannian 优化的本质区别。

---

### §E.32 从 MaxCut 到 SE-Sync：SDP 松弛的历史演进 ⭐⭐⭐⭐

SE-Sync 的 SDP 松弛不是凭空出现的，它有清晰的学术演进脉络：

| 时间 | 工作 | 贡献 | 与 SLAM 的关系 |
|------|------|------|---------------|
| 1987 | Shor | Shor 松弛：非凸二次 → SDP | 理论基础 |
| 1995 | Goemans-Williamson | MaxCut SDP + 0.878 近似比 | 首次展示 SDP 松弛的实际威力 |
| 2003 | Burer-Monteiro | 低秩分解 $Z=YY^\top$ 降维 | 使大规模 SDP 可解 |
| 2014 | Carlone-Censi | 2D PGO 的 Lagrangian 对偶 | 首次把 SDP 思想引入 SLAM |
| 2016 | Carlone et al. | 松弛紧性条件 + 噪声阈值 | 理解何时证书可用 |
| 2016 | Bandeira et al. | $\mathbb{Z}_2$ 同步的 SDP 紧性 | 数学基础（随机矩阵理论） |
| 2017 | Briales-Gonzalez | SE(3) 对偶四元数 SDP | 稀疏 SDP 构造 |
| 2019 | Rosen et al. | **SE-Sync**：BM + Staircase + 证书 | 第一个实用的 certifiable PGO |
| 2021 | Yang et al. | **TEASER++**：TLS + GNC + 证书 | 配准问题的认证 |
| 2021 | Tian et al. | **DC²-PGO**：分布式认证 | 多机器人协作 |
| 2023 | Yang-Carlone | TPAMI 统一框架 | 方法论集大成 |
| 2024 | Dumbgen et al. | **STRIDE**：general POP | 超越 QCQP 到多项式 |

这条脉络展示了一个清晰的**从纯数学到工程落地**的过程。Shor 1987 的理论经过 30 年才变成 Rosen 2019 的实用工具——中间的关键突破是 Burer-Monteiro（降维使大规模可解）和 Carlone-Censi（把 SDP 思想引入 SLAM 社区）。

**对博士生的研究启示**：这条演进路线表明，"从数学工具到工程突破"需要两个条件同时满足：（1）数学工具本身够强（SDP 比 LP 紧得多），（2）计算可行性突破（BM 使 SDP 可扩展到 $10^4$ 规模）。当前的开放问题（incremental certifiable、learning + certificates）也需要同时在两个维度上推进。

---

### §E.33 对偶证书的数值实践：如何判断"接近零"是否够好 ⭐⭐⭐

理论上，松弛紧 ⟺ $\lambda_{\min}(S)=0$。实际中由于浮点精度，$\lambda_{\min}(S)$ 永远不会精确为零。工程判断标准如下：

**判据层次**：

| 指标 | 计算方式 | 阈值建议 | 含义 |
|------|---------|---------|------|
| 绝对 $\lambda_{\min}$ | Lanczos 或 ARPACK | $> -10^{-7}$ | 证书近似成立 |
| 相对 gap | $\frac{p_{\text{SDP}} - d_{\text{SDP}}}{|p_{\text{SDP}}|+1}$ | $< 10^{-6}$ | 对偶间隙相对很小 |
| 相对 suboptimality | $\frac{f(\hat{x}) - d_{\text{SDP}}}{|d_{\text{SDP}}|+1}$ | $< 10^{-4}$ | 候选解接近全局最优 |

**数值陷阱**：

1. **大规模问题 Lanczos 精度**：sphere2500（$n=7500$）的 $\lambda_{\min}$ 计算用 Lanczos 迭代（ARPACK），精度受矩阵条件数影响。如果 $S$ 本身条件数很大（$\kappa>10^{10}$），Lanczos 可能给出不可靠的 $\lambda_{\min}$。此时应多跑几次随机初始化。

2. **残差白化影响**：如果测量权重跨越多个数量级（如 GPS cm 级 + LiDAR mm 级 + IMU），$\tilde{Q}$ 矩阵的条件数会很大，证书计算容易病态。建议统一白化后再构造 SDP。

3. **证书失败不等于解不好**：$\lambda_{\min}<0$ 只说明"无法认证全局最优"，不说明解是错的。实际中很多 $\lambda_{\min}\approx -10^{-3}$ 的情况下，BM 解仍然非常接近全局最优（suboptimality gap < 0.01%）。

**SE-Sync 源码中的实际判断逻辑**（`SESync.cpp`）：

```cpp
// Rosen 的实现
double min_eigenvalue = computeMinEigenvalue(S);
bool certified = (min_eigenvalue > -certification_tolerance);
// certification_tolerance 默认 1e-4
```

> **思维陷阱**：看到"证书通过"就认为估计结果一定正确。证书认证的是**优化问题的全局最优性**，不认证**模型的正确性**。如果数据关联错误（把两个不同地方当成同一地方），SE-Sync 会全局最优地"满足"这个错误约束——给出一个数学上最优但物理上荒谬的解。因此 certifiable perception 必须与鲁棒前端（PCM、GNC、几何验证）配合使用。

**BVB 定理的直觉**（Boumal-Voroninski-Bandeira 2020）：为什么 BM 分解（非凸优化）几乎总是找到全局最优？

核心定理的非正式表述：对"几乎所有" $C$ 矩阵（在某种 measure-zero 排除集之外），当 BM 秩参数 $p$ 满足 $\frac{p(p+1)}{2}>m$（$m$ 为约束数）时，BM 问题的**所有二阶临界点都是全局最优**。换言之，没有"伪鞍点"（strict saddle that is not global optimum）。

**直觉**：当 $p$ 足够大时，BM 的可行集"足够圆"（类似于高维球面上的凸优化），使得任何局部最优都自动是全局最优。这类似于高维空间中的"blessing of dimensionality"——更多维度意味着更少的局部极小陷阱。

**对 SE-Sync 的含义**：3D PGO 的 $d=3$（旋转矩阵列数），BVB 要求 $p\ge d$。SE-Sync 默认从 $p=d$ 开始尝试（Riemannian Staircase 的起始），如果 RTR 不收敛则提升到 $p=d+1$。实践中对所有标准数据集（sphere2500、torus、parking-garage），$p=d$ 就够了——这说明 BVB 定理的 measure-zero 排除集在实际 SLAM 问题中几乎不出现。

**Waldspurger-Waters 反例**（2020）：存在满足所有 BVB 前提条件的 SDP，在 $p$ 恰好等于 Barvinok-Pataki 秩界时，BM 有伪鞍点。这说明 BVB 定理的 $p$ 下界是**本质紧**的——不能进一步放松。但对实际 SLAM 问题，这个反例极为人工，实践中不需要担心。

### 常见陷阱与故障排查

⚠️ **陷阱一：把 SDP 原问题和对偶问题的不等式方向写反。** 对最小化原问题，弱对偶给出“对偶最优值不超过原问题最优值”。

⚠️ **陷阱二：把 rotation averaging 理解成 $\tau\to\infty$。** $\tau$ 是平移精度，趋于无穷会强化平移约束；纯旋转问题是直接去掉平移因子。

⚠️ **陷阱三：在 simplex 上再加 $\|\beta\|_1$ 正则。** 若 $\beta\ge0,\|\beta\|_1=1$，这个正则项是常数，不改变优化解。

| 故障排查现象 | 可能原因 | 处理方式 |
|---|---|---|
| 证书矩阵最小特征值略为负 | 数值容差或松弛不紧 | 比较 relative gap，调 RTR 精度 |
| 平移回代方向与论文相反 | incidence matrix 或 $\tilde V$ 约定不同 | 固定一个两节点例子手算符号 |
| SDP 解不能恢复 rank-$d$ 旋转 | 噪声过大或外点未处理 | 先做 PCM/GNC，或报告松弛不紧 |

> **本质洞察**：certifiable perception 的核心不是“换一个优化器”，而是把候选解和一个对偶证书配对；解给出估计值，证书回答“为什么它已经是全局最优”。

### 练习

1. 对一个 rank-1 Shor 松弛例子写出原问题、SDP 原问题和 SDP 对偶，标出弱对偶方向。
2. 构造两节点 PGO，分别采用 $R_j-R_i\tilde R_{ij}$ 与 $R_j-\tilde R_{ij}R_i$，说明相对旋转约定如何变化。
3. 给定一个 simplex 变量 $\beta$，证明 $\lambda\|\beta\|_1$ 是常数项，并设计一个真正会改变解的二次形状先验。

---

### 本章小结

| 核心概念 | 一句话要义 | 关键公式/条件 | 工程工具 |
|---------|-----------|-------------|---------|
| Certifiable Algorithm | 多项式时间返回解 + 布尔证书 | `cert=true` ⟹ 全局最优 | SE-Sync / TEASER++ |
| QCQP | 二次目标 + 二次等式约束（如 $R^\top R=I$） | $\min x^\top Cx$ s.t. $x^\top A_i x=b_i$ | PGO / RA 的原始形式 |
| Shor 松弛 | 变量提升 $Z=xx^\top\to Z\succeq 0$ 去掉 rank-1 | $\min\langle C,Z\rangle$ s.t. $\langle A_i,Z\rangle=b_i$ | SDP 求解器 |
| 松弛紧 (tightness) | SDP 最优值 = 原问题最优值 | $Z^\star$ 的秩恰好为 $d$ | $\lambda_{\min}(S)\ge 0$ |
| 对偶证书 | $S=C-\sum\lambda_i^\star A_i\succeq 0$ 验证全局最优 | 互补松弛 $XS=0$ | Lanczos 检查 $\lambda_{\min}$ |
| Burer-Monteiro | $Z=YY^\top$ 把 SDP 降维到 Stiefel 流形 | BVB 定理：$p>\sqrt{2m}$ 时无伪鞍 | Riemannian trust-region |
| Riemannian Staircase | 从低秩 $p=d$ 开始，需要时逐步提升 | 实践中几乎不需要 $p>d+1$ | SE-Sync 默认策略 |
| SE-Sync | PGO 的 certifiable 求解器 | $\tilde{Q}$ 矩阵 + 旋转恢复 + 平移回代 | `github.com/david-m-rosen/SE-Sync` |
| TEASER++ | >99% 外点下的鲁棒配准 | TLS + GNC + DRS 认证 | `MIT-SPARK/TEASER-plusplus` |
| DC²-PGO | 分布式可认证 PGO | RBCD + 分布式证书验证 | Kimera-Multi |

---

### 累积项目：本章新增模块

**项目名称**：从零构建 Mini-LIO 后端 → 添加全局验证层

**本章新增**：SE-Sync 证书验证模块

在前序章节中，累积项目已完成：
- 5-A：Kalman 滤波递推器
- 5-B：Batch 因子图 GN 求解器
- 5-C：iSAM2 增量后端

本章新增：
1. 对 iSAM2 后端输出的 pose graph，调用 SE-Sync 做全局验证
2. 报告对偶证书 $\lambda_{\min}(S)$ 和 relative suboptimality gap
3. 当证书通过时，把 SE-Sync 解注入 iSAM2 作为先验
4. 当证书失败时，报告失败并建议增加鲁棒处理

```python
# 累积项目骨架
import numpy as np
# 假设有 SE-Sync Python binding 或调用命令行

class CertifiableVerifier:
    def __init__(self, se_sync_path):
        self.se_sync_path = se_sync_path

    def verify_pose_graph(self, poses, edges, noise_models):
        """
        输入 pose graph，输出：
        - is_certified: bool
        - global_solution: 全局最优解（如果认证通过）
        - lambda_min: 对偶证书最小特征值
        - relative_gap: 相对次优间隙
        """
        # TODO: 学生实现
        # 1. 导出 g2o 或自定义格式
        # 2. 调用 SE-Sync
        # 3. 解析输出，检查 lambda_min >= -tolerance
        # 4. 如果认证通过，恢复 R 和 t
        pass

    def inject_to_isam2(self, isam_backend, global_solution):
        """将 SE-Sync 全局解注入 iSAM2"""
        # TODO: 作为强先验因子添加
        pass
```

---

### 延伸阅读

| 资源 | 难度 | 内容 | 建议阅读方式 |
|------|------|------|------------|
| Rosen et al. "SE-Sync" IJRR 2019 | ⭐⭐⭐⭐ | PGO certifiable 完整算法 | §III-V + 附录定理证明 |
| Yang & Carlone "Certifiably Optimal..." TPAMI 2023 | ⭐⭐⭐⭐ | 统一框架 + 外点处理 | §II-IV 理论框架 |
| Yang et al. "TEASER" T-RO 2021 | ⭐⭐⭐ | 鲁棒配准 + TLS + GNC | §III-IV 算法描述 |
| Carlone "Estimation Contracts" FnT 2023 | ⭐⭐⭐⭐ | 可认证感知综述 | 全文通读了解全景 |
| Boyd-Vandenberghe *Convex Optimization* 2004 | ⭐⭐⭐ | SDP/对偶/Slater 基础 | §4.6, §5.9 |
| Boumal et al. "Non-convex BM" JMLR 2020 | ⭐⭐⭐⭐ | BVB 定理正式证明 | Thm 2 + 讨论 |
| Tian et al. "DC²-PGO" T-RO 2021 | ⭐⭐⭐⭐ | 分布式可认证 PGO | §III-IV 分布式算法 |
| Briales-Gonzalez "SE(3) dual quaternion SDP" CVPR 2017 | ⭐⭐⭐⭐ | 稀疏 SDP 与 SE-Sync 等价 | 松弛构造 |
| Cifuentes et al. FOCM 2022 | ⭐⭐⭐⭐ | QCQP 松弛紧性条件 | 理论最强结果 |
| SE-Sync 源码 `github.com/david-m-rosen/SE-Sync` | ⭐⭐⭐ | C++ 参考实现 | 跑 sphere2500 + 看 `SESync.cpp` |

---

### 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| $\lambda_{\min}(S)$ 明显为负（如 $<-0.1$） | 松弛不紧：噪声过大或大量外点 | 1.先用 PCM/GNC 去外点 2.降低噪声权重 3.检查数据关联 | §E.28 |
| BM 解的 rank 高于 $d$ | Staircase 没收敛或问题结构不紧 | 1.提高 RTR 精度 2.升高 staircase 起始 rank 3.检查是否有孤立节点 | §E.6 |
| SE-Sync 比 LM 慢 | 图小且噪声低，LM 初值好直接收敛 | 1.只在大回环或需证书时调用 SE-Sync 2.混合策略：LM 日常 + SE-Sync 验证 | §E.20 |
| 对偶 gap 很小但位姿明显错误 | 数据关联错误被全局优化完美满足 | 1.检查前端匹配质量 2.PCM clique 分析 3.证书只保证"满足当前约束的最优"不保证约束正确 | §E.28 |
| 内存爆炸（>32 GB） | 显式形成了 $Z\in\mathbb{R}^{n\times n}$ | 1.使用 BM 分解（永远不显式存 $Z$） 2.检查代码是否误调用了 full SDP solver | §E.6 |
| 平移回代结果错误 | $\tilde{V}$ 矩阵构造的符号约定错 | 1.用两节点小例手算验证 2.对照 SE-Sync 源码的 incidence matrix 构造 | §E.9 |
| Riemannian 梯度不下降 | Stiefel 流形上的 retraction 或投影错 | 1.检查 $Y^\top Y$ 是否保持 block-diagonal 2.验证 `Proj_T(grad)` 的正交性 | §E.6 |

---

### 跨章综合练习 ⭐⭐⭐

**题目**：综合 5-B（因子图优化）+ 5-C（iSAM2）+ 5-E（本章 Certifiable）+ 5-F（鲁棒估计）的知识：

1. **混合架构设计**：设计一个完整的鲁棒 SLAM 后端：实时层用 iSAM2 + Huber，周期性验证层用 PCM + GNC-TLS + SE-Sync。画出数据流图，标注每个模块的输入/输出接口、触发条件（如每 200 个关键帧触发一次）、以及结果如何回注 iSAM2。

2. **松弛紧性的数值探索**：生成 sphere2500 数据集的合成版本，逐步增大旋转噪声 $\sigma_\theta\in\{0.01, 0.05, 0.1, 0.2, 0.3, 0.5\}$ rad。对每个噪声水平跑 SE-Sync，记录 $\lambda_{\min}(S)$。画出 $\lambda_{\min}$ vs $\sigma_\theta$ 曲线，验证 Carlone T-RO 2016 的"$\sigma_\theta>0.3$ 时松弛可能不紧"的经验阈值。

3. **GNC + SE-Sync 流水线**：对一个含 30% 错误回环的 pose graph，先用 GNC-TLS（5-F）识别外点并剔除，再用 SE-Sync（5-E）对剩余内点求全局最优并获得证书。与直接在含外点图上跑 SE-Sync（不剔除外点）对比——后者的证书会成功吗？为什么？

### 术语对照表

| 英文术语 | 中文 | 首次出现节 |
|---------|------|-----------|
| Semidefinite Programming (SDP) | 半定规划 | §E.4 |
| Semidefinite Relaxation | 半定松弛 | §E.4 |
| Burer-Monteiro (BM) Factorization | Burer-Monteiro 分解 | §E.6 |
| Riemannian Staircase | Riemannian 阶梯法 | §E.6 |
| Certifiably Optimal | 可认证最优 | §E.2 |
| Duality Gap | 对偶间隙 | §E.5 |
| Pairwise Consistency Maximization (PCM) | 成对一致性最大化 | §E.28 |
| Truncated Least Squares (TLS) | 截断最小二乘 | §E.28 |
| Quadratically Constrained QP (QCQP) | 二次约束二次规划 | §E.4 |
| Strong Duality | 强对偶性 | §E.5 |
| Stiefel Manifold | Stiefel 流形 | §E.6 |
| Riemannian Trust-Region (RTR) | Riemannian 信赖域 | §E.6 |
| Sparse Bounded Degree SOS (SBSOS) | 稀疏有界度平方和 | §E.14 |
| Rotation Averaging | 旋转平均 | §E.3 |
| Pose Graph Optimization (PGO) | 位姿图优化 | §E.3 |
| Dual Certificate | 对偶证书 | §E.5 |
| Rank-Restricted BM | 秩受限 BM 分解 | §E.6 |
| Incidence Matrix | 关联矩阵 | §E.9 |

### 本章方法适用性速查

| 场景 | 推荐方法 | 原因 |
|------|---------|------|
| 小规模 PGO（<100 节点）| 直接 SDP solver | 规模小，精确求解可行 |
| 中等规模（100-5000 节点）| SE-Sync (BM + Staircase) | 扩展性好，有紧性证书 |
| 大规模（>5000 节点）| Distributed DC2-PGO | 可分布式并行，内存可控 |
| 含外点 | GNC-TLS + SE-Sync | 先去外点再求全局最优 |
| 实时在线 | iSAM2 日常 + SE-Sync 定期验证 | 兼顾实时性与全局最优性 |

---
