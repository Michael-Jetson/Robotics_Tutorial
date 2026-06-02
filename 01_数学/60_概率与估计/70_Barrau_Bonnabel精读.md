## 博士前数学路线图·第五批·子专题 D：Barrau-Bonnabel TAC 2017 逐定理精读——群-仿射、对数线性与稳定性

### 前置自测

> 答不出 2 题以上，建议先回 A3（流形滤波族）和李群基础章节复习。

1. 矩阵李群上的 $\exp$ 和 $\log$ 映射分别做什么？在 $SO(3)$ 上写出 Rodrigues 公式。
2. 什么是左不变误差 $\eta^L=\chi^{-1}\hat\chi$？它与标准 EKF 的 $e=x-\hat x$ 有什么区别？
3. 标准 EKF 的雅可比 $A(\hat x,u)$ 依赖什么量？这会导致什么工程问题？
4. BCH 公式 $\log(e^Xe^Y)=X+Y+\frac{1}{2}[X,Y]+\cdots$ 的前两项是什么？什么条件下高阶项消失？
5. $SE_2(3)$ 群的矩阵形式是什么？它比 $SE(3)$ 多了什么结构？

---

**与主干内容的关系**：本专题是 A3（流形滤波族）的理论纵深。A3 在工程层面引入了 InEKF 的算法流程与 $SE_2(3)$ IMU 实现，但对 group-affine 定理只借用结论不展开证明。本专题回到 Barrau-Bonnabel TAC 2017 原论文，逐定理精读其数学核心——group-affine 条件的三等价刻画、对数线性性质的 BCH 推导、以及基于 Deyst-Price 1968 的 Lyapunov 稳定性证明。这些证明为 A3 中"InEKF 为什么比 EKF 好"提供了严格的数学答案，也为理解后续 Equivariant Filter（EqF）推广奠定基础。建议在完成 A3 后阅读本专题。

---

### 本章目标

学完本章后，你应当能够：

1. 在黑板上独立复现 Theorem 1（group-affine 三等价）的完整证明，理解每一步的代数动机
2. 解释 Theorem 2（log-linear）为什么是**精确的**而非近似的——具体说明 BCH 中交叉项如何消失
3. 区分"在李群上写 EKF"与"Invariant EKF"的本质差异——后者要求 group-affine + compatible observation
4. 对 $SE_2(3)$ IMU 系统逐块验证 group-affine 成立，理解 $f_u(e)\neq 0$（重力项）的"仿射"含义
5. 理解 bias 增广为什么破坏 group-affine，掌握两种修复路径（Tangent Group / EqF）

---

### 知识树总览

```text
Barrau-Bonnabel TAC 2017 精读
├── 前置基础
│   ├── 矩阵李群 / 李代数 / exp / log（A3 前置）
│   ├── 左/右不变误差定义（§D.2）
│   ├── 伴随表示 Ad / ad（§D.2）
│   └── EKF 雅可比 A(x̂,u) 依赖估计的问题（A3）
├── 核心定理链
│   ├── Theorem 1：group-affine 三等价（§D.3-D.5）
│   │   ├── (i) 左不变误差自治
│   │   ├── (ii) 右不变误差自治
│   │   └── (iii) f(ab)=f(a)b+af(b)-af(e)b
│   ├── 流的群同态性质（§D.7 Lemma）
│   ├── Theorem 2：log-linear 精确性（§D.8-D.9）
│   │   └── BCH + [X,X]=0 消除交叉项
│   └── Theorem 4：沿任意轨迹的稳定性（§D.10-D.12）
│       └── Deyst-Price 1968 LTV-KF → 群上升
├── 应用实例
│   ├── SE₂(3) IMU 动力学逐块验证（§D.26）
│   ├── 不变观测分类（左/右）（§D.15-D.16）
│   └── Bias negative result（§D.17）
└── 后续发展
    ├── Tangent Group / Two-Frames Group（Chauchat 2022）
    ├── Equivariant Filter（Mahony-van Goor 2022）
    └── 与优化方法 (iSAM2/InEKF) 的互补关系
```

> **底线陈述（BLUF）**：Barrau-Bonnabel TAC 2017 的真正贡献不是"把 EKF 搬到李群"，而是发现了一类闭合的非线性系统——group-affine 系统——使 EKF 在该类上同时获得三件平凡 EKF 永远做不到的事：误差 ODE 自治（Theorem 1）、误差对数严格满足线性时变 ODE（Theorem 2，**精确，不是近似**——但需 $\log$ 映射在收敛域内；对 $SO(3)$ 旋转误差接近 $\pi$ 时有拓扑限制）、收敛半径独立于初始化时刻（Theorem 4）。**为什么重要**：标准 EKF 的雅可比 $A(\hat x_t,u_t)$ 依赖估计，导致 Riccati 协方差传播失真、虚假可观性、filter divergence；几十年来工程界用 FEJ/OC-EKF 等经验修正缓解。InEKF 从代数结构本身解决问题——$A(u_t)$ 仅依赖输入，**Riccati 在传播段精确**，无需任何外部一致性修正。这一突破直接落地于 $SE_2(3)$ 上的 IMU 导航（§V）、SLAM、Hartley IJRR 2020 的 Cassie 双足机器人（位置 RMSE 降低 ~30%）、以及 Mahony-van Goor 的 Equivariant Filter 推广。**本文档的边界与分工**：5-A3 已给出 InEKF 的算法流程、工程实现与 $SE_2(3)$ IMU 验证（概念借用 group-affine 定理的结论，但不展开证明）；本子任务（5-D）专注于原论文的定理级精读——Theorem 1 三等价证明、Proposition 2 流的群同态性、Theorem 2 的 BCH 推导、Theorem 3/4 基于 Deyst-Price 1968 的 Lyapunov 稳定性证明、不变观测的代数分类、Bias 的 negative result 代数证明、以及与 Bonnabel-Martin-Rouchon 2008 (BMR)、Mahony-van Goor EqF 的关系。读者完成本文档后，应能在黑板上独立复现 Theorem 1、Theorem 2 的完整证明。

---

### §D.1 论文整体结构与核心贡献三件套 ⭐⭐

主论文 **Barrau, A. & Bonnabel, S., "The Invariant Extended Kalman Filter as a Stable Observer", IEEE TAC 62(4):1797–1812, 2017**（arXiv:1410.1465v4, 2015-10-19）按 arXiv 版的章节编排为：

- **§I Introduction**：动机——EKF 稳定性证明（Reif-Unbehauen 1999、Krener 2003）依赖 Riccati 协方差 $P_t$ 的谱条件，"沿任意轨迹收敛"对一般非线性观测器是极罕见的性质。
- **§II A special class of multiplicative systems**：
  - §II.1 引例（非完整车，Proposition 1）
  - §II.2 矩阵李群上误差自治系统（**Theorem 1**，eq.(7) **group-affine**）
  - §II.3 误差传播的对数线性性质（**Theorem 2**）
- **§III Invariant Extended Kalman Filtering**：IEKF 通用结构（LIEKF/RIEKF）、增益 tuning、紧凑方程 (36)/(37)；**Proposition 2** 出现在 §III.1.1；**Theorem 3** = Deyst-Price 1968 重述；**Theorem 4** 主稳定性定理；**Theorem 5** 工程化充分条件。
- **§IV Simplified car example**：SE(2) 独轮车，Propositions 3, 4。
- **§V Navigation on flat earth**：$SE_2(3)$ 上 IMU + landmark，Proposition 5 + Theorem 6。
- **附录 A**（李群基础）、**附录 B**（Theorem 2 证明）、**附录 C**（Theorem 4 证明）、**附录 D**（§IV 证明）。

**核心贡献三件套**：

1. **Group-affine 条件**（Theorem 1, eq.(7)）：完全刻画使左/右不变误差演化自治的系统类，**严格扩张**了 Bonnabel-Martin-Rouchon 2008 的"左不变 + 右不变"框架——$SE_2(3)$ 上带重力 $g$ 的 IMU 动力学 $f(e)\neq 0$，既非左不变也非右不变，但仍 group-affine。
2. **Log-linear 性质**（Theorem 2）：对 group-affine 系统，存在仅依赖 $u_t$ 的矩阵 $A_t$ 使得 $\dot\xi=A_t\xi$ **精确**成立（在 $\log$ 映射的收敛域内；对 $SO(3)$ 旋转误差接近 $\pi$ 时有拓扑限制——$\log$ 出现多值性，BCH 公式的收敛性要求 $\|\xi\|$ 在单射半径内）。
3. **沿任意轨迹的稳定性**（Theorem 4）：在均匀可观条件下 IEKF 渐近稳定，**收敛半径 $\varepsilon$ 独立于初始化时间 $t_0$**。

**历史脉络**：Bonnabel CDC 2007（SO(3) 左不变 EKF）→ Bonnabel-Martin-Rouchon TAC 2008（symmetry-preserving observers，Cartan moving frame）→ Bonnabel-Martin-Salaün CDC 2009（IEKF 原型）→ **Barrau PhD 2015**（HAL: tel-01344622，最详细证明）→ Barrau-Bonnabel TAC 2017（正式出版）→ Barrau-Bonnabel ARCR 2018（综述）→ Hartley IJRR 2020（接触辅助 InEKF + Cassie 实验）→ Mahony-van Goor TAC 2022 / van Goor-Mahony T-RO 2023（EqF 推广）。

---

### §D.2 符号约定与对数坐标（严格按 TAC 2017 §II.2） ⭐⭐

设 $G\subset GL_n(\mathbb{R})$ 为矩阵李群，李代数 $\mathfrak{g}\subset\mathbb{R}^{N\times N}$，$\dim\mathfrak{g}=d$。选定线性同构 $L_{\mathfrak g}:\mathbb{R}^d\to\mathfrak{g}$，**指数映射**定义为
$$
\exp(\xi) := \mathrm{expm}\bigl(L_{\mathfrak g}(\xi)\bigr)\in G,\qquad \xi\in\mathbb{R}^d.
$$
在 $\eta\in G$ 充分接近单位元 $e=\mathrm{Id}$ 时，$\eta=\exp(\xi)$ 中的 $\xi$ 称为**对数坐标**。

**伴随表示**：$\mathrm{Ad}_g:\mathbb{R}^d\to\mathbb{R}^d$ 定义为 $g\,L_{\mathfrak g}(\xi)\,g^{-1}=L_{\mathfrak g}(\mathrm{Ad}_g\,\xi)$；其李代数版本 $\mathrm{ad}_x\xi$ 定义为 $[L_{\mathfrak g}(x),L_{\mathfrak g}(\xi)]=L_{\mathfrak g}(\mathrm{ad}_x\xi)$。

**真值与估计**：在 §II.2 抽象阶段，作者**不区分**真值与估计，取 $(\chi_t,\bar\chi_t)$ 为系统 (4) 的两条任意轨迹，定义
$$
\boxed{\;\eta^L_t := \chi_t^{-1}\bar\chi_t\quad\text{(left-invariant)},\qquad \eta^R_t := \bar\chi_t\chi_t^{-1}\quad\text{(right-invariant)}\;}
$$
进入 §III IEKF 阶段后，$\chi_t$ 解读为**真值**，$\hat\chi_t$ 为**估计**，复用 $\eta^L=\chi^{-1}\hat\chi$、$\eta^R=\hat\chi\chi^{-1}$（eqs. (18), (25)）。

**重要警告**：Hartley IJRR 2020 部分章节、Bonnabel CDC 2007 旧惯例、以及一些 MEKF 文献把右不变误差写作 $\hat\chi\chi^{-1}$ 而左不变误差写作 $\chi\hat\chi^{-1}$（**估计在左**）——与 TAC 2017 恰好对称换位。交叉阅读时**必须**以 TAC 2017 (18)、(25) 为基准重新换算。"左/右不变"的命名理由：$\eta^L=\chi^{-1}\hat\chi$ 在**左平移** $(\chi,\hat\chi)\mapsto(\Gamma\chi,\Gamma\hat\chi)$ 下不变。

---

### §D.3 Theorem 1 的精确陈述 ⭐⭐⭐

**Theorem 1（Barrau-Bonnabel TAC 2017, §II.2, p.1800）**　对动力学
$$
\frac{d}{dt}\chi_t = f_{u_t}(\chi_t)\tag{4}
$$
下列三条件**等价**：

**(i)** 左不变误差具状态轨迹无关传播：存在 $g^L_{u_t}$ 使得 $\dfrac{d}{dt}\eta^L_t = g^L_{u_t}(\eta^L_t)$。

**(ii)** 右不变误差具同样性质：$\dfrac{d}{dt}\eta^R_t = g^R_{u_t}(\eta^R_t)$。

**(iii) (Group-affine 条件)** 对所有 $t>0$，所有 $a,b\in G$，
$$
\boxed{\;f_{u_t}(ab) = f_{u_t}(a)\,b + a\,f_{u_t}(b) - a\,f_{u_t}(e)\,b\;}\tag{7}
$$
此时 (eqs. (8)-(9))
$$
g^L_{u_t}(\eta) = f_{u_t}(\eta) - f_{u_t}(e)\,\eta,\qquad g^R_{u_t}(\eta) = f_{u_t}(\eta) - \eta\,f_{u_t}(e).
$$

> **术语对照**：(iii) 在 Barrau PhD 2015 Ch.3 称为"autonomous error propagation characterization"；Hartley 2020 Theorem 3.1 命名为 "group-affine property"。TAC 2017 正文未单独命名 (7)，但所有后续文献统一使用 "group-affine dynamics"。注意 $f_u(e)$ **不必为零**——这是"仿射"而非"线性"的本质来源（重力 $g$ 即 $f(e)$ 的非零项）。

---

### §D.4 Theorem 1 的完整证明（教学版，逐步展开） ⭐⭐⭐⭐

**预备引理**：对任意可微曲线 $\chi_t\in G$，
$$
\frac{d}{dt}(\chi_t^{-1}) = -\chi_t^{-1}\dot\chi_t\,\chi_t^{-1}.\tag{$\star$}
$$
*证*：从 $\chi_t\chi_t^{-1}=I$ 求导，$\dot\chi_t\chi_t^{-1}+\chi_t\frac{d}{dt}\chi_t^{-1}=0$，左乘 $\chi_t^{-1}$ 即得。$\square$

#### (i) ⟹ (iii)：从"轨迹无关"反推 group-affine

**Step 1**　设 $\chi_t,\bar\chi_t$ 为 (4) 的两条轨迹，$\eta_t=\chi_t^{-1}\bar\chi_t$（省略上标 $L$）。由 ($\star$)：
$$
\frac{d}{dt}\eta_t = -\chi_t^{-1}\dot\chi_t\chi_t^{-1}\bar\chi_t + \chi_t^{-1}\dot{\bar\chi}_t.
$$

**Step 2**　代入动力学 $\dot\chi_t=f_{u_t}(\chi_t)$, $\dot{\bar\chi}_t=f_{u_t}(\bar\chi_t)$，并把 $\chi_t^{-1}\bar\chi_t=\eta_t$、$\bar\chi_t=\chi_t\eta_t$ 回代：
$$
\frac{d}{dt}\eta_t = -\chi_t^{-1}f_{u_t}(\chi_t)\,\eta_t + \chi_t^{-1}f_{u_t}(\chi_t\eta_t).\tag{10}
$$
此为精确等式，尚未使用 (i)。

**Step 3**　假设 (i) 成立，即存在 $g_{u_t}$ 使 $\dot\eta_t=g_{u_t}(\eta_t)$。(10) 对所有 $\chi_t\in G$、所有 $\eta_t\in G$ 成立。取特殊值 $\chi_t=e$：
$$
g_{u_t}(\eta_t) = -f_{u_t}(e)\,\eta_t + f_{u_t}(\eta_t).\tag{11}
$$
此即 (8)。

**Step 4**　把 (11) 代回 (10) 左端，并把一般 $\chi_t$ 写作 $a$、$\eta_t$ 写作 $b$：
$$
-f_{u_t}(e)\,b + f_{u_t}(b) = -a^{-1}f_{u_t}(a)\,b + a^{-1}f_{u_t}(ab).
$$
左乘 $a$ 并整理：
$$
\boxed{\;f_{u_t}(ab) = f_{u_t}(a)\,b + a\,f_{u_t}(b) - a\,f_{u_t}(e)\,b.\;}
$$
即 (7)。$\square$

#### (iii) ⟹ (i)：从 group-affine 推回轨迹无关

由 (iii) 取 $a=\chi_t,b=\eta_t$：$f_{u_t}(\chi_t\eta_t)=f_{u_t}(\chi_t)\eta_t+\chi_t f_{u_t}(\eta_t)-\chi_t f_{u_t}(e)\eta_t$。代入 (10)：
$$
\frac{d}{dt}\eta_t = -\chi_t^{-1}f_{u_t}(\chi_t)\eta_t + \chi_t^{-1}\bigl[f_{u_t}(\chi_t)\eta_t + \chi_t f_{u_t}(\eta_t) - \chi_t f_{u_t}(e)\eta_t\bigr].
$$
逐项化简：$-\chi_t^{-1}f_{u_t}(\chi_t)\eta_t$ 与 $+\chi_t^{-1}f_{u_t}(\chi_t)\eta_t$ 互相抵消，剩余 $f_{u_t}(\eta_t)-f_{u_t}(e)\eta_t = g^L_{u_t}(\eta_t)$。$\chi_t$ **完全消除**，(i) 得证。$\square$

#### (ii) ⟺ (iii) 的对称证明

对 $\tilde\eta_t=\bar\chi_t\chi_t^{-1}$：
$$
\dot{\tilde\eta}_t = f_{u_t}(\bar\chi_t)\chi_t^{-1} - \bar\chi_t\chi_t^{-1}f_{u_t}(\chi_t)\chi_t^{-1}.
$$
代入 $\bar\chi_t=\tilde\eta_t\chi_t$，取 $\chi_t=e$ 得 (9) $g^R_{u_t}(\tilde\eta)=f_{u_t}(\tilde\eta)-\tilde\eta f_{u_t}(e)$。(iii) ⟹ (ii) 用 (7) 取 $a=\tilde\eta_t,b=\chi_t$ 平行展开即可。$\square$

> **关键引理：$f_u(e)$ 的"常数漂移项"角色**。若 $f_u(e)=0$，group-affine 退化为左-右不变的纯乘法形式；一般情况下 $f_u(e)$ 扮演"常数项"，类比 $\mathbb{R}^n$ 上仿射函数的 $f(0)$。

---

### §D.5 Group-Affine 的代数直觉 ⭐⭐⭐

#### 向量空间退化（Remark 2）

在 $\mathbb{R}^n$（加法群）上 (7) 退化为 $f_u(a+b)=f_u(a)+f_u(b)-f_u(0)$。令 $g(x):=f_u(x)-f_u(0)$，则 $g(a+b)=g(a)+g(b)$，即 $g$ 线性，$f_u(x)=Ax+B$ 即**仿射函数**。这正解释了 "group-affine" 的命名。

#### 三类典型 group-affine 动力学

**纯左不变** $f_u(\chi)=\chi\omega_u$（$\omega_u\in\mathfrak{g}$）：
$$
f_u(a)b+af_u(b)-af_u(e)b = a\omega_u b + ab\omega_u - a\omega_u b = ab\omega_u = f_u(ab).\;\checkmark
$$

**纯右不变** $f_u(\chi)=v_u\chi$：同理 $\checkmark$。

**左右组合（Remark 1）** $f_{v,\omega}(\chi)=v_u\chi+\chi\omega_u$：
$$
\begin{aligned}
\text{RHS}&=(v_u a+a\omega_u)b+a(v_u b+b\omega_u)-a(v_u+\omega_u)b\\
&=v_u ab+a\omega_u b+av_u b+ab\omega_u-av_u b-a\omega_u b = v_u ab+ab\omega_u = f(ab).\;\checkmark
\end{aligned}
$$

> **概念误区**：初学者常认为"group-affine 就是左不变加右不变"。实际上 group-affine 严格扩张了左右不变的并集——$SE_2(3)$ 上 IMU 动力学既非左不变也非右不变，但仍是 group-affine。区分这三者的方法是检查 $f(e)$ 是否为零：$f(e)=0$ 时退化为左右不变，$f(e)\neq 0$ 时"仿射"项登场。

#### 最重要的例子：$SE_2(3)$ 上 IMU 动力学

状态 $\chi=\begin{pmatrix}R&v&p\\0&1&0\\0&0&1\end{pmatrix}\in SE_2(3)$，输入陀螺 $\omega$、加速度计 $a_m$、重力 $g\in\mathbb{R}^3$。动力学
$$
\dot R=R(\omega)_\times,\quad\dot v=g+R\,a_m,\quad\dot p=v.
$$
矩阵化（§V eq.(49)）：
$$
f_{\omega,a_m}(\chi)=\begin{pmatrix}R(\omega)_\times & g+Ra_m & v\\ 0&0&0\\0&0&0\end{pmatrix}.
$$
此 $f$ 既非左不变也非右不变（重力项 $g$ 出现在 $f(e)\neq 0$ 中），但仍 group-affine。最直接的代数验证：把 $f$ 写成 $f_{\omega,a_m}(\chi) = A\chi + \chi B$，其中 $A$ 携带重力（$f(e)$ 的非零部分），$B$ 携带 body-frame 输入；这是 Remark 1 左右组合形式的特例，(7) 自动成立。

> **教学要点**：$SE_2(3)$ 把 $v,p$ 嵌入为"虚拟平移"，使得 IMU 动力学呈现 $A\chi+\chi B$ 的左右组合 + 常数漂移结构。$f(e)=A\neq 0$ 对应重力——这是"仿射"一词最直观的工程体现。Barrau PhD 2015 Ch.4 与 Hartley IJRR 2020 Appendix B 给出逐块严格验证。

---

### §D.6 Proposition 2：流的群同态性 ⭐⭐⭐⭐

**Proposition 2（TAC 2017 §III.1.1, p.1804，与任务书的强化版本）**　考虑 group-affine 动力学 (14) 的 LIEKF 传播步 (16)。设 $\xi_t$ 满足线性 ODE
$$
\frac{d}{dt}\xi_t = A_{u_t}\xi_t,\qquad A_{u_t}\text{ 由 } g^L_{u_t}(\exp\xi)=L_{\mathfrak g}(A_{u_t}\xi)+O(\|\xi\|^2)\text{ 定义,}\tag{20}
$$
$\eta_t$ 满足非线性误差 ODE (19) $\dot\eta_t=g^L_{u_t}(\eta_t)$。则若在 $t_{n-1}$ 处 $\eta_{t_{n-1}}=\exp(\xi_{t_{n-1}})$，则
$$
\boxed{\;\eta_t = \exp(\xi_t),\qquad \forall\, t_{n-1}\le t<t_n\;}
$$
**且即使初始误差任意大亦成立**。

**与 Theorem 2 的关系**：Proposition 2 是 Theorem 2 在 IEKF 传播步的直接应用。任务书表述的"存在仅依赖 $u_t$ 的流映射 $g_t:G\to G$ 使 $\eta_t=g_t(\eta_0)$"是 Theorem 1 + Picard-Lindelöf 的即时推论；Proposition 2 进一步把流 $g_t$ 通过 $\exp$ **线性化**为 (20)。

---

### §D.7 Proposition 2 的证明：流的群同态结构 ⭐⭐⭐⭐

#### Part A：流映射 $g_t$ 的存在（Theorem 1 + Picard-Lindelöf）

由 Theorem 1，group-affine ⟺ $\dot\eta_t=g^L_{u_t}(\eta_t)$ 右端只依赖 $\eta_t$。$g^L_{u_t}$ 局部 Lipschitz（由 $f_u$ 光滑性自动保证）→ Picard-Lindelöf 给出**唯一**流映射
$$
g_t:G\to G,\qquad g_t(\eta_0):=\eta_t.
$$
$g_t$ 仅依赖输入 $\{u_s\}_{0\le s\le t}$，**与 $\chi_t,\hat\chi_t$ 无关**。

#### Part B：$g_t$ 是 $G$ 的群同态（Lemma 2）

**Lemma**　对 $\dot\eta=g^L_u(\eta)$ 的流 $\Phi_t$（即 $g_t$），有 $\Phi_t(\eta_0\eta_0')=\Phi_t(\eta_0)\Phi_t(\eta_0')$。

*证*：$g^L_u$ 满足关键 Leibniz 性质（来自 (8) 与 (7)）：
$$
g^L_u(ab) = f_u(ab) - f_u(e)ab \stackrel{(7)}{=} f_u(a)b + af_u(b) - af_u(e)b - f_u(e)ab = g^L_u(a)b + ag^L_u(b).\tag{57}
$$
对 $\Phi_t(\eta_0)\Phi_t(\eta_0')$ 求导：
$$
\frac{d}{dt}[\Phi_t(\eta_0)\Phi_t(\eta_0')] = g^L_u(\Phi_t(\eta_0))\Phi_t(\eta_0') + \Phi_t(\eta_0)g^L_u(\Phi_t(\eta_0')) \stackrel{(57)}{=} g^L_u(\Phi_t(\eta_0)\Phi_t(\eta_0')).
$$
两侧在 $t=0$ 处都等于 $\eta_0\eta_0'$，由 ODE 解的唯一性 $\Phi_t(\eta_0\eta_0')=\Phi_t(\eta_0)\Phi_t(\eta_0')$。$\square$

#### Part C：log-linear 精确性

**关键步骤（BCH 平凡化）**：由 Lemma + 归纳，$\Phi_t(\eta_0^p)=\Phi_t(\eta_0)^p$ 对 $p\in\mathbb{Z}$ 成立。对任意 $n\in\mathbb{N}$：
$$
\Phi_t(\exp(\xi_0)) = \Phi_t\bigl([\exp(\xi_0/n)]^n\bigr) = \Phi_t(\exp(\xi_0/n))^n.
$$
设 $F_t:=D\Phi_t|_e$，泰勒展开
$$
\Phi_t(\exp(\xi_0/n)) = \exp\bigl(\tfrac{1}{n}F_t\xi_0 + r_t(\xi_0/n)\bigr),\qquad r_t(\zeta)=O(\|\zeta\|^2).
$$
**关键 BCH 平凡化**：$n$ 个相同因子 $X_n=\frac{1}{n}F_t\xi_0+r_t(\xi_0/n)$ 的 $\exp$ 乘积，由 BCH 公式 $\log(e^Xe^Y)=X+Y+\frac{1}{2}[X,Y]+\cdots$，**所有交叉项含 $[X_n,X_n]=0$**，故
$$
[\exp(X_n)]^n = \exp(nX_n) = \exp\bigl(F_t\xi_0 + n\cdot r_t(\xi_0/n)\bigr).
$$
由 $r_t(\zeta)=O(\|\zeta\|^2)$，$n\cdot r_t(\xi_0/n)=n\cdot O(1/n^2)=O(1/n)\to 0$，故
$$
\boxed{\;\Phi_t(\exp(\xi_0)) = \exp(F_t\xi_0).\;}
$$
两边对时间求导 + 在 $\xi_0\to 0$ 一阶比较：$\dot F_t=A_{u_t}F_t$，$F_0=I$。即 $\xi_t=F_t\xi_0$ 满足 (20)。$\square$

**几何直觉**：流 $\Phi_t$ 是 $G$ 上的**群同态**（保群运算），由其在单位元邻域的导数 $A_t$ 完全决定。所有 group-affine 动力学的"非线性"只是表面，骨架是 $\mathfrak g\to\mathfrak g$ 的线性映射 $A_t$——典型的李群"局部到整体"原理。

---

### §D.8 Theorem 2 精确陈述 ⭐⭐⭐

**Theorem 2（Log-Linear Property of the Error，TAC 2017 §II.3, eq.(13), p.1801-1802）**　设 $\chi_t,\bar\chi_t$ 为 group-affine 动力学 (12) 的任意两条轨迹，$\eta^i_t$（$i\in\{L,R\}$）为不变误差。取 $\xi^i_0\in\mathbb{R}^d$ 使 $\eta^i_0=\exp(\xi^i_0)$。设 $A^i_{u_t}$ 由
$$
g^i_{u_t}(\exp(\xi)) = L_{\mathfrak g}(A^i_{u_t}\xi) + O(\|\xi\|^2)
$$
定义。若 $\xi^i_t$ 满足线性 ODE
$$
\frac{d}{dt}\xi^i_t = A^i_{u_t}\xi^i_t,\tag{13}
$$
则**对任意 $t\ge 0$ 且对任意可由所选 log/exp 分支表示的初始误差**都有 $\eta^i_t = \exp(\xi^i_t)$。

#### 三层"不可思议"之处

1. **$A^i_{u_t}$ 仅依赖输入 $u_t$**——它是 $g^i_u$ 在 $e$ 处的李代数雅可比，与 $\chi_t,\hat\chi_t$ 均无关。
2. **(13) 是精确 ODE，不是近似**——虽然 $A^i_u$ 是用一阶展开定义，但 group-affine 结构使 BCH 高阶项**恰好相消**。
3. **不要求小误差线性化**——线性关系不依赖 $\|\xi\|$ 足够小，但仍受 log/exp 分支、拓扑和误差可表示性的限制；这不是滤波器全局收敛保证，而是误差传播方程的结构性精确性。

#### $A^i_{u_t}$ 的具体公式

$$
\boxed{\;A^L_u = D\bigl[L_{\mathfrak g}^{-1}\circ g^L_u\circ \exp\bigr]\Big|_{\xi=0}.\;}
$$
对 $SE_2(3)$ IMU 导航的具体计算（§V）给出
$$
A_t = \begin{pmatrix} 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \\ (g)_\times & 0_{3\times 3} & 0_{3\times 3} \\ 0_{3\times 3} & I_3 & 0_{3\times 3} \end{pmatrix},\qquad\text{完全独立于 }\hat\chi_t.
$$
注意 $A_t$ **下三角且严格幂零**——这是 $SE_2(3)$ 在 IMU 导航中数值稳定性极佳的结构性原因。

---

### §D.9 Theorem 2 的 BCH 证明 ⭐⭐⭐⭐

完整证明在附录 B（约 1.5 页代数）。结构由三条引理 + 主定理构成，核心已在 §D.7 Part C 给出。这里再总结 BCH 的两层关键作用：

**第一层：群同态性消除 $\chi$ 依赖**。Lemma（§D.7）的证明用到 $g^L_u$ 的 Leibniz 性 (57)——这正是 group-affine 条件 (7) 平移到 $g^L$ 之后的结果。无 group-affine，$g^L_u$ 不满足 (57)，流不是群同态，整个论证崩塌。

**第二层：相同因子的 BCH 平凡化**。$[\exp(X_n)]^n = \exp(nX_n)$ 利用 $[X_n,X_n]=0$ 与 Jacobi 恒等式。这看似初等，但是连接"局部线性化 $A_t$" 与"全局精确解 $\xi_t$"的桥梁——证明的"魔术"全在这里。

#### Chirikjian Vol.2 的工具

$\exp$ 的微分 $d\exp_\xi:\mathfrak g\to T_{\exp\xi}G$ 的级数公式（Ch.5）：
$$
d\exp_\xi = \sum_{k\ge 0}\frac{(-\mathrm{ad}_\xi)^k}{(k+1)!} = \frac{\mathrm{Id}-e^{-\mathrm{ad}_\xi}}{\mathrm{ad}_\xi},
$$
是展开 $\dot\eta = d\exp_\xi(\dot\xi)$ 的工具。对真正幂零的李代数，级数会在有限阶截断，所有计算闭式可得。$\mathfrak{se}(3)$ 与 $\mathfrak{se}_2(3)$ 含有旋转部分，整体并不是幂零李代数，因此不能简单说 BCH 有限阶截断；它们的闭式更多来自 $SO(3)$ 旋转指数、左 Jacobian 及块结构。

#### 为什么 nilpotent 李代数特别适用

$\mathfrak{se}(3)$ 的平移子空间是阿贝尔的，但旋转对平移的反复伴随作用不会有限阶消失；$\mathfrak{se}_2(3)$ 也应按半直积结构理解。Theorem 2 的"无穷 BCH 相消"不是靠位姿群幂零性，而是靠 group-affine 条件与引理给出的代数抵消。对导航/位姿估计而言，计算友好性来自矩阵群闭式、Adjoint 块结构和 $SO(3)$ Jacobian，而不是把整个位姿李代数当成幂零代数。

> 注意：log-linear 精确性**本身不需幂零**（仅需 group-affine + Lemma + 极限），幂零只是**额外的计算友好性**。

---

### §D.10 为什么 Log-Linear 是 InEKF 的根本性突破 ⭐⭐

#### 标准 EKF 的致命缺陷

对一般非线性 $\dot x=f(x,u)$，标准 EKF 误差 $e=x-\hat x$ 满足
$$
\dot e = f(x,u)-f(\hat x,u) = \underbrace{\partial_x f|_{\hat x,u}}_{A(\hat x,u)} e + O(\|e\|^2).
$$
**雅可比 $A(\hat x,u)$ 依赖估计**——估计漂移 → $A$ 偏离真值雅可比 $A(x,u)$ → Riccati $\dot P=AP+PA^\top+Q$ 传播失真 → Kalman 增益 $L=PH^\top S^{-1}$ 不匹配真实观测 → 估计更漂离 → **正反馈 → filter divergence**。

这正是 EKF 稳定性证明（Reif-Unbehauen 1999, Krener 2003, Boutayeb-Rafaralahy-Darouach 1997）必须**假设** $\gamma_{\min}I\preceq P_t\preceq\gamma_{\max}I$ 的根源——而此假设本身在实际中**无法先验验证**。

#### InEKF 的结构性突破

对 group-affine 系统，$\dot\xi = A_{u_t}\xi$ 中 $A_{u_t}$ **仅依赖输入**。Riccati
$$
\dot P = A_{u_t}P + P A_{u_t}^\top + \hat Q_t \tag{35/37}
$$
在传播段**精确**。Hartley IJRR 2020 §3.3 总结此关键性质："the linearized error dynamics ... depend only on the control inputs $u_t$ and known quantities, not on the current state estimate $\hat X_t$"。

#### 对比 FEJ 与 OC-EKF

Huang-Mourikis-Roumeliotis 的 **First-Estimates Jacobian (FEJ)** 与 **Observability-Constrained EKF (OC-EKF)** 通过**人为冻结**雅可比在某参考点防止虚假可观性。这是工程经验修正：
- 冻结雅可比**失真**（牺牲二阶项准确性）；
- 与系统几何无关，只是经验技巧。

InEKF 从**对称性**导出"$A$ 与估计无关"是**结构性性质**，并非经验修正。其结果是 InEKF **自动**享有 OC-EKF 类似的可观性一致性（Hartley 2020 §III.D），且**观测雅可比 $H$ 也独立于估计**（见 §D.15）。

#### 对 EKF-SLAM 假可观性问题的解决

EKF-SLAM 中由于雅可比依赖估计，yaw（沿重力方向旋转）虚假可观（Julier-Uhlmann 2001; Huang et al. 2010），导致长期一致性丧失。**Brossard-Barrau-Bonnabel** 证明 InEKF-SLAM 误差方程 log-linear，**自动**保持 yaw 不可观（与物理一致），无需任何外部修正——这是 InEKF 在 SLAM 工程中的最大卖点之一。

---

### §D.11 可观性条件与 Deyst-Price 1968 ⭐⭐⭐

**可观性 Gramian** 对线性时变系统 $\dot x=A_t x$, $y_n=Hx_{t_n}$，定义状态转移 $\dot\Phi^t_{t_0}=A_t\Phi^t_{t_0}$, $\Phi^{t_0}_{t_0}=I$，则
$$
W_O(t_0,t_f) = \sum_{t_n\in[t_0,t_f]}(\Phi^{t_n}_{t_0})^\top H^\top N_n^{-1} H \Phi^{t_n}_{t_0}.
$$
**均匀可观（uniformly observable）**：存在常数 $\beta_1,\beta_2,M>0$ 使 $\beta_1 I\preceq W_O(t_n-M,t_n)\preceq\beta_2 I$ 对所有 $n$ 成立。

**Theorem 3（TAC 2017 p.1804，即 Deyst-Price 1968）**　对 LTV 离散观测的连续 KF，若 (i) 一步转移可逆性、(ii) 过程噪声正定 $Q_s\succeq\delta_2 I$、(iii) 观测噪声正定 $N_n\succeq\delta_3 I$、(iv) 过程 Gramian 上下有界、(v) 观测 Gramian 上下有界 这五条件成立，则存在 $\gamma_{\min},\gamma_{\max}>0$ 使 $\gamma_{\min}I\preceq P_t^{-1}\preceq\gamma_{\max}I$，且 Lyapunov 函数 $V(P,\xi)=\xi^\top P^{-1}\xi$ 沿 KF 轨迹**指数衰减**。

---

### §D.12 Theorem 4：连续 IEKF 的渐近稳定性 ⭐⭐⭐

**Theorem 4（TAC 2017 p.1804，主定理）**　考虑 group-affine 系统 (14) 配合左不变观测 (15)（resp. 右不变观测 (22)）。若 Theorem 3 的稳定性条件 (i)-(v) 沿**真实轨迹 $\chi_t$** 线性化后的系统成立，则 LIEKF（resp. RIEKF）(36) 所定义的估计 $\hat\chi_t$ 是 $\chi_t$ 的渐近稳定观测器。**且收敛半径 $\varepsilon>0$ 在整条轨迹上一致成立（独立于初始化时间 $t_0$）**。

#### 为什么 $\varepsilon$ 独立于 $t_0$ 是不平凡的

Reif-Unbehauen 1999 EKF 稳定性结果给出
$$
\varepsilon(t_0)\le \frac{\text{const}}{\sup_{t\ge t_0}\|A(\hat x_t,u_t)\|\cdot\gamma_{\max}(t_0,\cdot)},
$$
$\varepsilon$ **随 $t_0$ 变化**——因真实轨迹上雅可比幅度随时间变化，二阶余项也随之膨胀。

InEKF 中 $A_{u_t}$ 仅依赖输入 $u_t$，只要 $u_t$ 在整条轨迹上**均匀有界**，BCH 展开下高阶余项可被**单一常数**控制——$\varepsilon$ 只需小到这个常数水平，**与 $t_0$ 无关**。这是附录 C 证明 Lemma 4-7 的核心数学内容。

---

### §D.13 Theorem 4 离散时间版本 ⭐⭐⭐⭐

虽然 TAC 2017 的 Theorem 4 已经处理"连续传播 + 离散观测"的混合情形，**严格纯离散** IEKF 的 log-linear 证明由 **Barrau-Bonnabel SCL 2019 "Linear observed systems on groups"** 给出。

**离散 group-affine** 的等价刻画：$F_n(ab) = F_n(a)F_n(e)^{-1}F_n(b)$，保证离散误差也满足 log-linear。

**Riccati 离散化**：若测量间隔 $\Delta t=t_{n+1}-t_n$，$\Phi_n=\exp_m(A_{u_t}\Delta t)$（输入分段常数），则
$$
P^-_{n+1}=\Phi_n P^+_n\Phi_n^\top+\hat Q_n,\quad S_n=HP^-_nH^\top+\hat N_n,\quad L_n=P^-_nH^\top S_n^{-1},\quad P^+_n=(I-L_nH)P^-_n.
$$

**警告**：若 $A$ 在采样间隔内确为常数，$\Phi_n=\exp(A\Delta t)$ 正是误差线性系统的精确状态转移；被大步长破坏的不是 group-affine 结构本身，而是把输入、噪声或状态积分粗暴冻结的一阶近似。准确离散化需要同时处理状态在群上的数值积分和噪声协方差积分；工程上可用 RKMK (Runge-Kutta-Munthe-Kaas) 等保李群积分器，或对 IMU 积分采用 Sola "micro Lie theory" 推荐的离散指数映射方案。

---

### §D.14 Theorem 4 证明思路（附录 C） ⭐⭐⭐⭐

附录 C 由 Lemma 4-7 + 主结论组成，核心论证逻辑：

#### Step 1：二阶余项的代数刻画

Update 阶段 $\xi^+_n = (I-L_nH)\xi_n + r_n(\xi_n)$，其中 $r_n$ 来自
$$
\exp[(I-L_nH)\xi - r_n(\xi)] = \exp[\xi]\exp[-L_nH(\exp[\xi]b - b)].
$$
由 BCH 公式 $\log(e^Xe^Y)=X+Y+\frac{1}{2}[X,Y]+\cdots$，得 $r_n(\xi)=O(\|\xi\|\|L_n\xi\|)$，即**二阶**。

#### Step 2：Duhamel 分解

引入线性部分状态转移 $\Psi^t_{t_0}$（满足 $\dot\Psi^t_{t_0}=A_{u_t}\Psi^t_{t_0}$, $\Psi^{t_n+}_{t_0}=(I-L_nH)\Psi^{t_n}_{t_0}$），按 Duhamel 分解：
$$
\xi_t = \Psi^t_{t_0}\xi_0 + \sum_{t_n<t}\Psi^t_{t_n}\,r_n(\xi_{t_n}).\tag{58}
$$

#### Step 3：四条引理

**Lemma 4（封闭邻域条件迁移）**：真实轨迹 $\chi_t$ 上 (i)-(v) 成立 → 估计轨迹 $\hat\chi_t$ 上修改常数 $\hat\delta_i = \delta_i/k^2$ 等成立，前提 $\|\xi_s\|<\varepsilon$。由 $\mathrm{Ad}_{e^{-\xi}}\to I$ 的连续性构造此 $\varepsilon$。

**Lemma 5（一阶增长控制）**：在 $[0,2M\Delta T]$ 上 $\|\xi_{t_0+s}\|\le l_1(\|\xi_{t_0}\|)$，$l_1(x)=O(x)$。

**Lemma 6（Lyapunov 二阶控制）**：$V_{t_0+s}(\xi_{t_0+s})\le V_{t_0+s}(\Psi^{t_0+s}_{t_0}\xi_{t_0})+l_2(\|\xi_{t_0}\|)$，$l_2(x)=O(x^2)$。

**Lemma 7（最终二阶增长控制）**：在每段 $M$ 次 update 之间，由 Deyst-Price 的 $V^+_{t_{n+N}}-V^+_{t_n}\le -\beta_3\|\Psi^{t_{n+N}}_{t_n}\xi\|^2$ 获得**严格单减线性主项**，二阶修正 $l_2$ 被 $\beta^k$ 吸收。

#### Step 4：最终结论

代入 $l_2(x)<\frac{\beta^k}{2}x$ 于小邻域，反证 $t=\inf\{s:\|\xi_{t_0+s}\|\ge\frac{3}{4}\varepsilon'\}<\infty$ 矛盾 → 解整条轨迹保持于 $\varepsilon'/2$ 邻域；级数 $\sum\|\xi^+_{t_{n_0+iM}}\|^2$ 收敛 → $\xi^+\to 0$。附带结论 $\|P_t-\tilde P_t\|\to 0$（$\tilde P_t$ 是沿真实轨迹的 Riccati 解）。

#### 与 EKF 稳定性证明的核心差异表

| 项目 | 标准 EKF（Reif-Unbehauen） | InEKF（本文） |
|---|---|---|
| 雅可比 $A$ | $A(\hat x_t,u_t)$ 依赖估计 | $A(u_t)$ 仅依赖输入 |
| 高阶余项 | $O(\|e\|^2)$ 系数随 $\hat x$ 变 | propagation 中**为零**；update 中**一致有界** |
| 收敛半径 | 依赖 $t_0$ | **独立于 $t_0$** |
| $P_t$ 边界 | 需假设 | 由 Deyst-Price 在真实轨迹上保证 |
| 是否需要 FEJ/OC | 是（工程经验修正） | 否（结构性） |

---

### §D.15 左不变观测 vs 右不变观测的代数分类 ⭐⭐⭐

#### 严格定义（TAC 2017 §III-B, eq.(15)/(22)）

**左不变观测**：$Y_n = \chi_{t_n}\,d + V_n$，$d\in\mathbb{R}^N$ 是已知的 **body frame** 参考向量。"左不变"含义：观测在右平移 $(\chi,\hat\chi)\mapsto(\chi\Gamma^{-1},\hat\chi\Gamma^{-1})$ 下不变；其 innovation 与 $\eta^L=\chi^{-1}\hat\chi$ 兼容。

**右不变观测**：$Y_n = \chi_{t_n}^{-1}\,d + V_n$，$d\in\mathbb{R}^N$ 是已知的 **world/fixed frame** 参考向量。其 innovation 与 $\eta^R=\hat\chi\chi^{-1}$ 兼容。

#### 兼容性引理

- 左不变观测 → $\hat\chi^{-1}Y - d = \eta^{L,-1}d - d$ 仅含 $\eta^L$ → 选 LIEKF（更新公式 eq.(17) $\hat\chi^+=\hat\chi\exp(L_n(\hat\chi^{-1}Y-d))$）。
- 右不变观测 → $\hat\chi Y - d = \eta^R d - d$ 独立于 $\chi$ → 选 RIEKF（eq.(24)）。

**手性错配后果**：若对 body-frame 观测用 RIEKF，innovation 残差残留 $\chi$ 依赖项，Jacobian $H$ 重新依赖估计，Theorem 4 失效。

#### Jacobian $H$ 独立于估计——一致性最后一块拼图

对左不变观测一阶展开（§III-C, eq.(33)-(34)）：
$$
(\eta^L)^{-1}d - d = -L_{\mathfrak g}(\xi)\,d + O(\|\xi\|^2)\;\Rightarrow\; H\xi=-(L_{\mathfrak g}(\xi)d^1,\dots,L_{\mathfrak g}(\xi)d^k)^\top.
$$
**$H$ 是 $\xi$ 的线性算子，矩阵仅依赖已知参考向量 $\{d^i\}$，完全独立于估计与真值**。若噪声协方差也采用固定或不变的表达，配合 (35) 的 Riccati，Kalman 增益 $L_n$ 不需要通过当前估计重新计算观测 Jacobian；若噪声需要通过 $\operatorname{Ad}_{\hat\chi}$ 或当前姿态换坐标，增益仍可能间接依赖估计。这正是 InEKF 相对标准 EKF 在更新步的本质优势与工程边界。

#### 完整分类表

| 传感器 | 观测方程 | 参考向量坐标系 | 类别 |
|---|---|---|---|
| GPS（world-frame 位置） | $Y=\chi\cdot(0,\dots,0,1)^\top$ | body/selector 固定向量 | **左不变** |
| Body-frame velocity（pitot/Doppler） | $Y=R^\top v=\chi^{-1}\cdot(0,v,0,1)^\top$ | world（body 读取） | **右不变** |
| Landmark（相机看 world 中 $p_k$） | $Y_k=\chi^{-1}\cdot p_k^{\mathrm{hom}}$ | world-fixed | **右不变** |
| Contact kinematics（foot-world 刚接触） | $h(q)=R^\top(p_c-p)$ | world-fixed | **右不变** |
| Magnetometer（body 读 world $m_\oplus$） | $Y=R^\top m_\oplus + V$ | world | **右不变** |
| Body-frame accel（重力 $g$） | $Y=R^\top g + V$ | world | **右不变** |
| Pose measurement（motion capture） | $Y=\chi$ | — | 自伴 |

**记忆法则**：参考向量 $d$ 若"长在 world 里、在 body 里读"（magnetometer, body velocity, landmark），归**右不变**；$d$ 若"长在 body 里、被 $\chi$ 送到 world"（GPS 的 origin 在 body 中已知为 $(0,1)^\top$），归**左不变**。

> **常见陷阱**：把 landmark 观测 $Y=R^\top(p_k-p)$ 当成左不变。它其实是**右不变**，因 $p_k$ 在 world 中已知；这正是 §IV range-and-bearing、§V flat-earth navigation 使用 RIEKF 的根源。

---

### §D.16 Fundamental Observations 与 ICRA 2020 扩展 ⭐⭐⭐⭐

#### 定义

Barrau-Bonnabel ICRA 2020 (arXiv:2003.03908) 与后续 TAC 2022 把 §III-B 抽象化：**fundamental observation** 是形如 $y_n=\chi_{t_n}^{\pm 1}\star d$ 的观测（$\star$ 为群在向量空间 $V$ 上的线性作用），使得 innovation $\hat\chi^{\pm 1}\star y - d$ 在 noise-free 极限下仅是真实误差 $\eta^{L/R}$ 的函数。

代数刻画：存在群作用 $\rho:G\times V\to V$ 及 $d\in V$ 使 $h(\chi)=\rho(\chi,d)$，且其线性化 $H$ 仅依赖 $d$。

#### 相对 TAC 2017 的扩展

TAC 2017 限定 $\rho$ 为标准矩阵左乘/右乘 $V=\mathbb{R}^N$。ICRA 2020 后允许 $V$ 是非平凡 $G$-表示（张量积、对称乘），从而把 GNSS lever-arm（天线杆臂）、极线约束、坐标系间相对姿态吸收进 fundamental 框架。

#### 一致性的双重要求

"observation is fundamental" 与 "dynamics is group-affine" 是**两个独立条件**。完整一致性（$F$ 与 $H$ 都独立于估计）需**二者同时成立**。

#### 例：$SE_2(3)$ 上 GPS 是 fundamental left-invariant

令 $e=(0,0,0,0,1)^\top$ 标记 "position slot"：$\chi e=(p^\top,0,1)^\top$，$Y_n=\chi_{t_n}e+V_n$。Jacobian $H=(0_{3\times 3},0_{3\times 3},I_3)$（Hartley IJRR 2020 §IV.C）。配合 §V 的 group-affine IMU 动力学，propagation 与 update 均满足 log-linear 一致性。

---

### §D.17 Imperfect IEKF 与 IMU Bias——Negative Result ⭐⭐⭐

#### 带 bias 的 IMU 动力学

真实角速度 $\omega=\omega_m-b_g-w^g$，比力 $a=a_m-b_a-w^a$。"自然"扩增 $X=(\chi,b)\in SE_2(3)\times\mathbb{R}^6$，bias 加性演化 $\dot b=w^b$。pose 动力学
$$
\dot\chi = \chi\,\nu(\omega_m-b_g, a_m-b_a) + \text{重力项}.
$$
关键观察：$\nu$ 相对 $b$ **仿射非线性**（通过 $\omega_m-b_g$ 进入）。

#### Negative Result 的精确陈述

**Barrau PhD 2015 + Hartley 2020 §IV**：在 $G=SE_2(3)\times\mathbb{R}^6$（或其他 pose×bias 直积/半直积）上，**不存在自然的 $f$** 使
$$
f_u(XX')=f_u(X)X'+Xf_u(X')-Xf_u(e)X'\qquad(*)
$$
对所有 $X,X'$ 成立。

#### 可教学的代数证明（最小例：$SO(3)\times\mathbb{R}^3$ + gyro bias）

为便于黑板讲解，取最小 non-trivial 情形：$X=(R,b_g)\in SO(3)\times\mathbb{R}^3$，群律 $(R_1,b_1)(R_2,b_2)=(R_1R_2,b_1+b_2)$，$e=(I,0)$。动力学
$$
f_u(R,b_g)=(R(\omega_m-b_g)_\times,\,0).
$$

**代入 $(*)$**：

- LHS：$f_u(XX')=f_u(R_1R_2,b_1+b_2)=(R_1R_2(\omega_m-b_1-b_2)_\times,0)$。
- $f_u(X)X'=(R_1(\omega_m-b_1)_\times,0)\cdot(R_2,b_2)=(R_1(\omega_m-b_1)_\times R_2,0)$。
- $Xf_u(X')=(R_1,b_1)\cdot(R_2(\omega_m-b_2)_\times,0)=(R_1R_2(\omega_m-b_2)_\times,0)$。
- $Xf_u(e)X'=(R_1(\omega_m)_\times R_2,0)$。

**合并 RHS**：用 $(\omega_m)_\times R = R(R^\top\omega_m)_\times$ 化简，
$$
\text{RHS}_{(1,1)} = R_1(\omega_m-b_1)_\times R_2 + R_1R_2(\omega_m-b_2)_\times - R_1(\omega_m)_\times R_2.
$$
化简前两项与第三项的差：$R_1(\omega_m-b_1)_\times R_2 - R_1(\omega_m)_\times R_2 = -R_1(b_1)_\times R_2 = -R_1R_2(R_2^\top b_1)_\times$，故
$$
\text{RHS} = R_1R_2\bigl[(\omega_m-b_2)_\times - (R_2^\top b_1)_\times\bigr].
$$
而 LHS$=R_1R_2(\omega_m-b_1-b_2)_\times$。等式成立 ⟺
$$
(b_1)_\times = (R_2^\top b_1)_\times\quad\forall R_2\in SO(3),
$$
此需 $b_1=0$ 或所有 $R_2$ 保留 $b_1$——**显然不成立**（取任意非单位旋转 + $b_1\neq 0$ 即矛盾）。$\square$

**结论**：$\omega\to\omega-b_g$ 引入"旋转与加性 bias 不可交换的耦合"，使 group-affine 在 $X=e$ 邻域即失败。加速度计 bias 同理。

#### Imperfect IEKF 的工程妥协

Hartley IJRR 2020 §IV 处理：
- pose 用 right-invariant 误差 $\eta=\hat\chi\chi^{-1}\in SE_2(3)$；
- bias 用加性误差 $\tilde b=\hat b - b\in\mathbb{R}^6$；
- 联合线性化 $\epsilon=(\xi,\tilde b)\in\mathbb{R}^{15}$。

连续误差雅可比（arXiv:1904.09251 eq.(50)）：
$$
F=\begin{pmatrix}F_{\mathrm{pp}}(u) & F_{\mathrm{pb}}(\hat\chi)\\ 0 & 0\end{pmatrix},
$$
- **pose-pose 块** $F_{\mathrm{pp}}$：独立于估计；
- **pose-bias 块** $F_{\mathrm{pb}}$：形如 $\mathrm{diag}(-\hat R,-(\hat v)_\times\hat R,-(\hat p)_\times\hat R)$ ——**依赖 $\hat\chi$**，是 group-affine 被破坏的直接代数体现。

**代价**：一致性不再严格，Theorem 4 全局结论不再直接成立。但 pose-pose 的线性性仍带来巨大优势。

#### Hartley Cassie biped 实验（IJRR 2020 §VI）

- 60 次 Monte-Carlo（初始 yaw $\pm 45^\circ$）：imperfect InEKF 位置 RMSE 相较 quaternion-EKF **降低 ~30%**，yaw RMSE 降低 >50%，收敛盆地显著更大。
- Cassie 双足真机 3 分钟行走：InEKF + bias 漂移 ~0.3%，QEKF ~0.5%；更重要的是 Jacobian 不依赖估计使 tuning 更鲁棒。

#### 近期补救

- **TG-VEKF / TFG**（Chauchat-Barrau-Bonnabel arXiv:2201.04426 "Two-Frames Group"，Luo-Wu-Mangelson 2023）：在 **tangent group** 上重定义 bias 群律（半直积），$\omega=0$ 时恢复 group-affine。
- **EqF**（§D.19）：完全绕开 group-affine 需求，只要求 equivariance。

---

### §D.18 与 Bonnabel CDC 2007 的关系 ⭐⭐⭐

Bonnabel CDC 2007 "Left-invariant EKF and attitude estimation" 是 IEKF 源头：在 $G=SO(3)$ 上做姿态估计，首次提出"左不变误差 $\eta=R^\top\hat R$ 随 $\omega$-driven 动力学独立于真实 $R$"。

**TAC 2017 相对 CDC 2007 的三个关键推广**：

1. **Group-affine 条件 (7)**：SO(3) 上左不变动力学 $\dot R=R\omega_\times$ 自然满足，但 $SE_2(3)$ 带重力 IMU 动力学既非左不变也非右不变，需 group-affine 这个**更宽**的代数条件。
2. **Log-linear (Theorem 2)**：SO(3) 上由 Rodrigues 公式可见，一般李群需 BCH + Ad 证明。
3. **$SE_k(d)$ 族群结构**：把 pose + $k$ 个向量量塞入单一 matrix group，使 velocity、landmarks、contacts 共用 $\chi$。CDC 2007 无此构造。

**SO(3) 平凡但一般情形需新工具的项**：fundamental observation 概念、$A_t$ 中重力 $(g)_\times$ 与位置 $(p)_\times$ 的耦合、误差 log 的高阶项消除。

---

### §D.19 与 Equivariant Filter (EqF) 的关系 ⭐⭐⭐⭐

#### 基本设定

Mahony-van Goor-Hamel "EqF" IEEE TAC 2022 (arXiv:2010.14666, CDC 2020) 把 InEKF 推广到**齐性空间** $\mathcal{M}=G/H$：
- 状态 $\xi\in\mathcal{M}$ 未必是李群；
- 传递作用 $\phi:G\times\mathcal{M}\to\mathcal{M}$；
- 动力学 $\dot\xi=f(\xi,u)$ 要求 **equivariance**：$D\phi_X\cdot f(\xi,u)=f(\phi_X(\xi),\psi_X(u))$。

#### EqF 与 InEKF 的关系

- **InEKF 是 EqF 的特例**：当 $\mathcal{M}=G$、$\phi$ 为群乘法、动力学 group-affine 时。
- EqF **只要求 equivariance**，不要求 group-affine——允许 lift 到更大对称群（通常 $\dim G>\dim\mathcal{M}$）。
- 线性化在 **fixed origin $\xi_\circ\in\mathcal{M}$**（而非沿估计轨迹），使误差坐标具**全局、估计无关**的线性化。

#### 处理 bias：EqF 通过 lift 而非扩增李群

Fornasier et al. "Overcoming Bias" (RAL 2022, arXiv:2209.12038) 与 EqVIO (arXiv:2205.01980)：把 IMU bias 视为"virtual input extension"，扩充对称群 $G=SE_2(3)\ltimes\mathbb{R}^6$（半直积，$SO(3)$ 作用在 gyro bias 上）。关键：定义 group action 使 bias 与 pose 以 $(R,b_g)\mapsto R^\top b_g$ 方式耦合——恰好补偿 §D.17 negative result 中 $(b_1)_\times\neq(R_2^\top b_1)_\times$ 的不可交换性。

**EqVIO 实验**：EuRoC V1_01 上 APE ~0.06m，V2_03 上 ~0.27m，优于 ROVIO/OpenVINS。

#### 理论关系总结

| 属性 | InEKF (2017) | EqF (2022) |
|---|---|---|
| 状态空间 | 李群 $G$ | 齐性空间 $\mathcal{M}=G/H$ |
| 代数条件 | group-affine | equivariance |
| 线性化点 | 沿估计轨迹 | fixed origin |
| $F$ 独立估计 | propagation 是 | 总是 |
| $H$ 独立估计 | fundamental obs. 时是 | output equivariance 时是 |
| 处理 bias | imperfect（失真） | 通过 lift 恢复一致 |

---

### §D.20 Bonnabel-Martin-Rouchon 2008 Symmetry-Preserving Observers ⭐⭐⭐⭐

#### 框架概述

BMR IEEE TAC 53(11):2514-2526, 2008 承接 Olver 的 Cartan moving frame 方法，给出系统满足对称 $\phi_g\cdot x=\phi_g(x)$ 时**保对称的非线性观测器**：
$$
\dot{\hat x}=f(\hat x,u)+\sum_i K_i(\hat x)E_i(\hat x,u,y),
$$
$K_i$ 与不变输出误差 $E_i$ 均由 Cartan 标架构造以保整个观测器在群作用下等变。

#### 不变观测器的 Cartan moving frame

选归一化截面 $\mathcal{N}\subset X$ → 对每个 $x$ 求唯一 $g(x)\in G$ 使 $\phi_{g(x)}(x)\in\mathcal{N}$ → 不变误差 $E(x,y)$ 与不变增益 $K(x)$ 通过 $g(x)$ 的 transport 构造。

#### IEKF 是其 Kalman-like 版本

- BMR 2008 提供观测器族**结构**（无特定增益准则）；
- Bonnabel CDC 2007 → Barrau-Bonnabel TAC 2017 把**增益选取改为 Kalman-Riccati**，使 IEKF 成为 BMR 框架在"最小均方误差 + 李群 group-affine 动力学"下的具体实现。

#### 为什么 Kalman 化需要 group-affine

Kalman 滤波对协方差精确传播的依赖来自线性化：$\dot P=AP+PA^\top+Q$ 要求 linearized error 方程**不含 $O(\|\xi\|^2)$ 余项**。若动力学仅"对称但非 group-affine"，linearized error 方程含随轨迹变动的二阶项，$P$ 演化失真，Deyst-Price 1968 失效。Group-affine + Theorem 2 共同保证 $\dot\xi=A_{u_t}\xi$ **精确成立**，使 $P$ 严格对应原非线性误差统计。

---

### §D.21 定理索引表 ⭐⭐

| 定理 | 前置条件 | 核心结论 | 证明位置 | 关键工具 |
|---|---|---|---|---|
| **Thm 1** (eq.(7)) | 矩阵李群 + 光滑 $f_u$ | 三等价：(i) $\dot\eta^L$ 自治 ⟺ (ii) $\dot\eta^R$ 自治 ⟺ (iii) group-affine | TAC p.1800-1801, §II.2 | 引理 ($\star$) + (10) 取 $\chi=e$ |
| **Prop 2** | group-affine | 流 $\Phi_t$ 是群同态；$\eta_t=\exp(F_t\xi_0)$ 精确 | TAC p.1804, §III.1.1 | Picard-Lindelöf + Lemma (57) |
| **Thm 2** (eq.(13)) | group-affine | $\dot\xi=A_{u_t}\xi$ **精确**（任意大误差） | TAC p.1801-1802; 证明附录 B p.1810 | BCH 平凡化 $[\exp X_n]^n=\exp(nX_n)$ |
| **Thm 3** (Deyst-Price 1968) | LTV + 五条件 (i)-(v) | 线性 KF $V=\xi^\top P^{-1}\xi$ 指数衰减 | TAC p.1804 | Lyapunov + Riccati 上下界 |
| **Thm 4** (主结果) | group-affine + Thm 3 条件沿真实轨迹 | LIEKF/RIEKF 渐近稳定，$\varepsilon$ 独立 $t_0$ | TAC p.1804; 证明附录 C p.1811, Lemma 4-7 | Duhamel (58) + BCH 二阶余项一致界 |
| **Thm 5** (工程化判据) | 线性化 $(A,H,B,D)$ 可检测 + 可达 | 自动满足 (i)-(v) | TAC p.1805 | 标准 LTV 理论 |
| **Thm 6** ($SE_2(3)$ 应用) | 三个非共线 landmark | flat-earth navigation IEKF 稳定 | TAC §V, p.1809 | Thm 5 应用 |

---

### §D.22 核心教材/论文索引 ⭐

| 文献 | 用途 | 关键章节 |
|---|---|---|
| **Barrau-Bonnabel TAC 2017** (arXiv:1410.1465v4) | 主论文 | §II-III, 附录 B-C |
| **Barrau PhD 2015** (HAL: tel-01344622) | 最详细证明 | Ch.3-4 |
| **Bonnabel CDC 2007** (DOI 10.1109/CDC.2007.4434662) | SO(3) 前驱 | 全文 |
| **Bonnabel-Martin-Rouchon TAC 2008** (53(11):2514-2526) | symmetry-preserving 框架 | §III-IV |
| **Hartley et al. IJRR 2020** (arXiv:1904.09251) | Contact-Aided InEKF + Cassie 实验 | §III-IV, §VI |
| **Mahony-van Goor-Hamel TAC 2022** (arXiv:2010.14666) | EqF 推广 | §III-V |
| **van Goor-Mahony T-RO 2023** (arXiv:2205.01980) | EqVIO 实验 | §IV-VI |
| **Barrau-Bonnabel ARCR 2018** | 综述（推荐入门） | 全文 |
| **Barrau-Bonnabel SCL 2019** | 离散版严格证明 | 全文 |
| **Barfoot 2ed §8.4** | 工程实现 | §8.4 |
| **Chirikjian Vol.2** | BCH 公式背景 | Ch.5-7 |
| **Solà arXiv:1812.01537** | micro Lie theory，符号统一 | §3-§4 |
| **Deyst-Price 1968 IEEE TAC** (13:702-705) | KF 稳定性原始结果 | 全文 |

---

### §D.23 常见理解错误（Common Pitfalls，10 条） ⭐⭐

1. **"Log-linear 是线性化近似"**——错。Theorem 2 断言 $\xi_t=\log(\eta_t)$ **精确**满足 $\dot\xi=A_{u_t}\xi$，BCH 高阶项通过群同态性 + 相同因子 $[X,X]=0$ **完全消除**。$A_{u_t}$ 虽由一阶导定义，但其解通过 $\exp$ 与真正非线性 $\eta_t$ **任意大误差精确重合**。

2. **"InEKF 完全消除所有非线性"**——错。Log-linear **只在传播段精确**。Update 阶段 $\xi^+=(I-LH)\xi+r(\xi)$ 仍有 $r(\xi)=O(\|\xi\|\|L\xi\|)$ 二阶项（附录 C.3）。Theorem 4 的非平凡之处是论证 update 二阶项**一致有界**而不破坏稳定性。

3. **"Bias 可以用 group-affine 处理"**——错。Barrau PhD 2015 negative result（§D.17 代数证明）：在 $SE_2(3)\times\mathbb{R}^6$ 上 $\omega-b_g$ 替换引入 $(R_2^\top b_1)_\times\neq(b_1)_\times$ 矛盾。"Imperfect" InEKF 的 $F_{\mathrm{pb}}$ 块不可避免地依赖估计。

4. **"InEKF 自动全局收敛"**——错。Theorem 4 仍是**局部**收敛（半径 $\varepsilon$ 内），但 $\varepsilon$ 独立于 $t_0$（沿任意轨迹一致）。这与 EKF "$\varepsilon(t_0)$ 随时间膨胀" 形成对比，但绝不等同于全局收敛。

5. **"Theorem 2 的 $A_t$ 与 EKF 的 Jacobian 是同一个东西"**——错。EKF Jacobian $A(\hat x,u)$ 依赖估计；InEKF $A(u_t)$ 仅依赖输入。两者**几何来源完全不同**：EKF 是状态空间切空间的雅可比，InEKF 是李代数同态 $g^L_u$ 在单位元的微分。

6. **"Log-linear 意味 Kalman 在李代数上是后验最优"**——错。Log-linear 仅说明误差**传播精确**（Riccati 在传播段无失真），并不说明 IEKF 是 MMSE 后验最优。最优性需要更强假设（如高斯分布在李代数上的精确对应），TAC 2017 未声称此结论。

7. **"Group-affine 要求动力学本身是线性的"**——错。Group-affine 是在群上的"仿射"性质（(7) 式），不是欧氏空间线性。$SE_2(3)$ 上 IMU 动力学高度非线性（含 $R\omega_\times,Ra_m,(g)_\times$ 耦合），但仍 group-affine。

8. **"Proposition 2 的 $g_t$ 与 $\exp(A_t)$ 是同一映射"**——部分错。$g_t:G\to G$ 是群上的非线性流（群同态）；$F_t=\exp(\int A_s\,ds)$ 是李代数 $\mathfrak g\simeq\mathbb{R}^d$ 上的线性映射。两者通过 $g_t(\exp\xi_0)=\exp(F_t\xi_0)$ 关联，但**$g_t$ 不等于** $\exp\circ F_t\circ\log$ 当作群上映射来看（除非通过 $\exp$ 参数化）。

9. **"左/右不变误差选择只是符号惯例"**——错。选择由**观测的不变性**决定（§D.15）：左不变观测 $Y=\chi d$ → LIEKF；右不变观测 $Y=\chi^{-1}d$ → RIEKF。手性错配会让 $H$ 重新依赖估计，Theorem 4 失效。Landmark 是**右不变**，GPS 位置是**左不变**——这是工程上最常见的误判。

10. **"$A_t$ 不依赖 $\hat\chi$ 意味 $P_t$ 也完全不依赖 $\hat\chi$"**——错。$A_{u_t}$ 不依赖 $\hat\chi$，但**调谐矩阵 $\hat Q_t,\hat N_n$** 依赖 $\hat\chi$（通过 $\mathrm{Ad}_{\hat\chi}$ 把传感器坐标系噪声变换到滤波器坐标系）。$P_t$ 仍温和依赖 $\hat\chi$，Lemma 4 量化此微扰不破坏稳定性。

---

### §D.24 学习时间预算（档位 4） ⭐

| 任务 | 时长 |
|---|---|
| TAC 2017 全文精读（§I-§V + 附录 A-C） | 5-6h |
| Barrau PhD 2015 Ch.3-4 精读（最详细 BCH 证明） | 4-5h |
| Hartley IJRR 2020 §III-IV 交叉对照 | 2h |
| Bonnabel CDC 2007 + BMR TAC 2008 历史脉络 | 1-2h |
| Mahony-van Goor EqF TAC 2022 关系阅读 | 1-2h |
| 自测题 + Cassie/EqVIO 实验复现思考 | 2-3h |
| **合计** | **15-20h** |

---

### §D.25 自测题（8 道，难度递进）

**Q1**　从 group-affine 定义 (7) 出发，独立推导 (i)⟹(iii) 的完整代数（不查阅本文档）。提示：Step 1 用 ($\star$)，Step 2 代入动力学，Step 3 取 $\chi=e$，Step 4 把一般 $\chi$ 重命名为 $a$。

**Q2**　用 $SE_2(3)$ 上 IMU 动力学 $\dot R=R\omega_\times$, $\dot v=g+Ra$, $\dot p=v$ 验证 group-affine 的速度块 (1,2)。展示如何把 $f$ 写成 $A\chi+\chi B$ 形式并验证 (7) 自动成立。

**Q3**　推导 InEKF 在 $SE_2(3)$ 上对 IMU 的 $A$ 矩阵：
$$
A_t=\begin{pmatrix}0&0&0\\(g)_\times&0&0\\0&I_3&0\end{pmatrix}.
$$
验证 $A_t$ **下三角**且**严格幂零**（$A_t^3=0$）。这对 BCH 截断有什么意义？

**Q4**　解释为什么 Theorem 2 的 log-linear 是**精确的**而非近似。具体说明 BCH 公式在 $[\exp(X_n)]^n=\exp(nX_n)$ 这一步如何"魔术般"消除所有交叉项。提示：$[X_n,X_n]=0$ + Jacobi 恒等式。

**Q5**　证明 bias 建模破坏 group-affine（Barrau PhD negative result）。具体计算：在 $SO(3)\times\mathbb{R}^3$ 上验证 $f_u(R,b_g)=(R(\omega_m-b_g)_\times,0)$ 不满足 (7)。给出导致矛盾的关键代数步 $(b_1)_\times\neq(R_2^\top b_1)_\times$。

**Q6**　对比 InEKF 稳定性证明（Theorem 4，附录 C）与 EKF 稳定性证明（Reif-Unbehauen 1999）的关键差异。具体回答：(a) 雅可比依赖性如何不同；(b) 二阶余项如何 bound；(c) 收敛半径 $\varepsilon$ 与 $t_0$ 关系如何不同。

**Q7**　判断以下传感器是左不变还是右不变观测，并写出对应观测方程：(a) GPS 位置；(b) 相机看 world-fixed landmark；(c) body-frame velocity（pitot tube）；(d) magnetometer。说明判断依据。

**Q8**　证明 InEKF 的流 $\Phi_t=g_t$ 是 $G$ 的群同态（即 §D.7 Lemma）。关键步骤：用 group-affine 推出 $g^L_u(ab)=g^L_u(a)b+ag^L_u(b)$（即 Leibniz 性 (57)），然后对 $\Phi_t(\eta_0)\Phi_t(\eta_0')$ 求导用 ODE 解的唯一性。

---

### §D.26 从定理回到 $SE_2(3)$：逐块验证 group-affine ⭐⭐⭐

前文已经说明 $SE_2(3)$ IMU 动力学可以写成 $A\chi+\chi B$，从而自动满足 group-affine。本节把这个结论逐块展开，因为很多工程错误都发生在"我知道它成立，但不知道每个块为什么成立"这个断层上。

令

$$
\chi=
\begin{bmatrix}
R&v&p\\
0&1&0\\
0&0&1
\end{bmatrix},\qquad
\chi_i=
\begin{bmatrix}
R_i&v_i&p_i\\
0&1&0\\
0&0&1
\end{bmatrix}.
$$

群乘法给出

$$
\chi_1\chi_2=
\begin{bmatrix}
R_1R_2&R_1v_2+v_1&R_1p_2+p_1\\
0&1&0\\
0&0&1
\end{bmatrix}.
$$

IMU 动力学为

$$
f_{\omega,a}(\chi)=
\begin{bmatrix}
R\omega^\wedge&g+Ra&v\\
0&0&0\\
0&0&0
\end{bmatrix}.
$$

单位元处

$$
f_{\omega,a}(I)=
\begin{bmatrix}
\omega^\wedge&g+a&0\\
0&0&0\\
0&0&0
\end{bmatrix}.
$$

要验证

$$
f(\chi_1\chi_2)=f(\chi_1)\chi_2+\chi_1f(\chi_2)-\chi_1f(I)\chi_2,
$$

只需检查三块：旋转块、速度列、位置列。

**旋转块**：

左边为

$$
R_1R_2\omega^\wedge.
$$

右边旋转块为

$$
R_1\omega^\wedge R_2+R_1R_2\omega^\wedge-R_1\omega^\wedge R_2
=R_1R_2\omega^\wedge.
$$

旋转块成立。注意抵消项正是 group-affine 中 $-\chi_1f(I)\chi_2$ 的作用；没有它，左右组合会多出一项。

**速度列**：

左边速度列为

$$
g+R_1R_2a.
$$

右边第一项 $f(\chi_1)\chi_2$ 的速度列为

$$
R_1\omega^\wedge v_2+g+R_1a.
$$

右边第二项 $\chi_1f(\chi_2)$ 的速度列为

$$
R_1(g+R_2a)+v_1\cdot0=R_1g+R_1R_2a.
$$

右边第三项 $\chi_1f(I)\chi_2$ 的速度列为

$$
R_1\omega^\wedge v_2+R_1(g+a).
$$

三项合并：

$$
R_1\omega^\wedge v_2+g+R_1a+R_1g+R_1R_2a-R_1\omega^\wedge v_2-R_1g-R_1a
=g+R_1R_2a.
$$

速度列成立。这里最容易看出"仿射"的意义：世界系重力 $g$ 不随 $R_1$ 旋转，若把它当成普通左/右不变项会出错；group-affine 的减项正好消掉多余的 $R_1g$。

**位置列**：

左边位置列为

$$
R_1v_2+v_1.
$$

右边第一项的位置列为

$$
R_1\omega^\wedge p_2+v_1.
$$

右边第二项的位置列为

$$
R_1v_2.
$$

右边第三项的位置列为

$$
R_1\omega^\wedge p_2.
$$

合并后为

$$
R_1\omega^\wedge p_2+v_1+R_1v_2-R_1\omega^\wedge p_2
=R_1v_2+v_1.
$$

位置列成立。由此三块全部验证完毕。

> **本质洞察**：$SE_2(3)$ 的矩阵形式不是为了"好看"，而是为了让速度和位置成为群乘法的一部分。
> 一旦 $v,p$ 进入群结构，IMU 的积分链 $\dot p=v,\ \dot v=g+Ra$ 就能被 group-affine 恒等式捕获。

这个逐块验证也说明了为什么普通 $SE(3)$ 不够。$SE(3)$ 只含 $R,p$，速度 $v$ 仍在群外，$\dot p=v$ 不能作为群乘法的一列自然出现。$SE_2(3)$ 把速度列并入矩阵，正是为惯性导航量身定制的扩展群。

---

### §D.26b Group-Affine 条件的几何直觉 ⭐⭐⭐

前面的逐块验证是代数层面的。这里给出 group-affine 的几何含义——为什么这个代数条件如此"自然"。

**回顾 group-affine 恒等式**：

$$f_u(ab) = f_u(a)\,b + a\,f_u(b) - a\,f_u(e)\,b$$

**几何解读**：这个恒等式说的是：**复合状态 $ab$ 在动力学 $f$ 下的速度，可以完全分解为"$a$ 的速度作用在 $b$ 上"加上"$b$ 的速度被 $a$ 变换后"再减去一个"重复计算修正项"。**

更精确地，把 $f_u(\chi)$ 理解为状态 $\chi$ 处的切向量（velocity field），则：

- $f_u(a)\,b$：从 $a$ 的角度看，$b$ 只是一个"被动搭载"的参考系——$a$ 的运动自然把 $b$ 带走
- $a\,f_u(b)$：从 $b$ 的角度看，它有自己的动力学 $f_u(b)$，但需要左乘 $a$ 变换到复合参考系
- $-a\,f_u(e)\,b$：双重计数修正——$f_u(e)$ 的贡献在前两项中各算了一次（一次作为 $a$ 的一部分、一次作为 $b$ 的一部分），需要减去一次

**类比到有限维空间的仿射映射**：考虑 $\mathbb{R}^n$ 上的仿射映射 $f(x)=Ax+c$。它满足

$$f(x+y) = A(x+y)+c = (Ax+c)+Ay = f(x)+Ay \neq f(x)+f(y)$$

但修正后：$f(x+y) = f(x)+f(y)-f(0)$——因为 $f(x+y)=Ax+Ay+c=f(x)+f(y)-(Ax+Ay+2c)$... 不完全对应。

更准确的类比是：在群上，"仿射"意味着 $f$ 在群的左/右平移之间保持特定的"兼容性"。纯左不变系统 $f(\chi)=\chi V$（$V\in\mathfrak{g}$ 固定）满足 $f(ab)=abV=f(a)b$——这是 group-affine 的特例（$f(e)=V$，第三项 $-af(e)b=-aVb$ 消掉了第二项 $af(b)=abV$）。纯右不变系统 $f(\chi)=W\chi$ 类似。**group-affine 是同时具有"左部分"和"右部分"的一般形式**——$f(\chi)=A\chi+\chi B$，其中 $A=f(e)-B$。

> **跨领域类比**：group-affine 之于李群动力学，如同仿射变换之于欧氏空间。
> 仿射变换 $f(x)=Ax+b$ 比纯线性映射（$b=0$）更广，但仍保持许多好性质（平行线映射到平行线）。
> group-affine 动力学比纯左/右不变动力学更广（允许 $f(e)\neq 0$，如重力），
> 但仍保持关键性质（误差自治、log-linear）。
> 正如仿射空间是向量空间的"最小推广"，group-affine 是不变系统的"最小推广"。

**为什么"仿射"而非"线性"**：group-affine 中 $f_u(e)$ 项的存在是核心。若 $f_u(e)=0$（如纯陀螺 $\dot R=R\omega^\wedge$），系统既是左不变也是右不变，group-affine 退化为最简单的情况。但 IMU 导航中重力 $g$ 出现在 $\dot v = g + Ra$ 中——在单位元处 $f(I)$ 的速度列为 $g+a\neq 0$。这个非零的 $f(e)$ 是"仿射"一词的来源，也是 Barrau 2015 PhD 论文的核心发现：**以前认为只有左不变或右不变系统才能做 InEKF，但 IMU 导航两者都不是——它是 group-affine 的，因为重力使 $f(e)\neq 0$。** Theorem 1 证明了这类系统仍然具有完整的 InEKF 理论保证。

---

### §D.27 Log-linear 的线性系统含义：不仅是漂亮定理 ⭐⭐⭐

Theorem 2 说：若 $\eta_t=\exp(\xi_t)$，则 $\xi_t$ 满足

$$
\dot\xi_t=A_t\xi_t.
$$

这句话有三个层次，必须分开理解。

**层次 1：误差传播自治。**

Theorem 1 已经给出

$$
\dot\eta_t=g_{u_t}(\eta_t),
$$

它只依赖 $\eta_t$，不依赖真值 $\chi_t$ 或估计值 $\hat\chi_t$。这一步消除了"沿哪条轨迹线性化"的问题。

**层次 2：指数坐标下线性。**

一般李群上的自治误差方程 $\dot\eta=g(\eta)$ 并不意味着 $\xi=\log\eta$ 线性。非线性项通常会通过 BCH 进入 $\dot\xi$。group-affine 更强，它使误差流 $\Phi_t$ 成为群同态：

$$
\Phi_t(\eta_1\eta_2)=\Phi_t(\eta_1)\Phi_t(\eta_2).
$$

群同态由单位元处导数决定，这使得

$$
\Phi_t(\exp\xi)=\exp(F_t\xi),
$$

其中 $F_t=D\Phi_t|_e$。对时间求导就得到线性 ODE。

**层次 3：Kalman 传播精确。**

若 $\xi$ 精确满足 LTV 系统，那么传播段的均值与协方差更新就是标准线性 KF 的精确传播：

$$
\dot P=A_tP+PA_t^\top+Q_t.
$$

这不是把非线性系统强行近似为线性系统，而是误差变量本身在对数坐标里线性。普通 EKF 与 InEKF 的差异就在这里。

以 $SE_2(3)$ IMU 为例，常见右不变误差的 $A$ 可写为

$$
A=
\begin{bmatrix}
0&0&0\\
g^\wedge&0&0\\
0&I&0
\end{bmatrix}.
$$

于是

$$
A^2=
\begin{bmatrix}
0&0&0\\
0&0&0\\
g^\wedge&0&0
\end{bmatrix},\qquad
A^3=0.
$$

矩阵指数截断为

$$
\Phi(\Delta t)=I+A\Delta t+\frac12A^2\Delta t^2.
$$

逐块写出传播：

$$
\delta\theta_{k+1}=\delta\theta_k,
$$

$$
\delta v_{k+1}=\delta v_k+g^\wedge\delta\theta_k\Delta t,
$$

$$
\delta p_{k+1}=\delta p_k+\delta v_k\Delta t+\frac12g^\wedge\delta\theta_k\Delta t^2.
$$

这个形式的物理意义很清楚：姿态误差本身不被理想陀螺传播放大；姿态误差通过重力方向错误进入速度；速度误差再进入位置。它与经典惯导误差方程一致，但这里它是由李群误差和 group-affine 定理推出的。

若加入 bias，上述 $A$ 会出现依赖估计的项，$A^3=0$ 的简洁结构也会被破坏。这不是实现细节，而是 D.17 negative result 的工程表现。

---

### §D.27b InEKF vs 标准 EKF：为什么 A 矩阵的差异如此关键 ⭐⭐⭐

这一节用具体数值说明 InEKF 的 log-linear 性质带来的实际性能差异。

**标准 EKF 的 Jacobian**：对状态 $x=[R,v,p]$，标准 EKF 定义误差为 $\delta x = x - \hat{x}$（或流形上的加法扰动）。传播段 Jacobian 为

$$A_{\text{EKF}}(\hat{x},u) = \frac{\partial f}{\partial x}\Big|_{\hat{x},u}$$

关键：$A_{\text{EKF}}$ **依赖当前估计 $\hat{x}$**。具体地，对 IMU 导航：

$$A_{\text{EKF}} = \begin{bmatrix}
-\omega_m^\wedge & 0 & 0 \\
-\hat{R}(a_m)^\wedge & 0 & 0 \\
0 & I & 0
\end{bmatrix}$$

其中 $\hat{R}$ 是当前估计的旋转矩阵。

**InEKF 的 Jacobian**：

$$A_{\text{InEKF}}(u) = \begin{bmatrix}
0 & 0 & 0 \\
g^\wedge & 0 & 0 \\
0 & I & 0
\end{bmatrix}$$

关键差异：$A_{\text{InEKF}}$ **只依赖输入 $u$（通过 $g$）和已知常数**，不依赖估计。

**实际后果**：

1. **Riccati 协方差传播精度**：EKF 的 $\dot{P} = A(\hat{x})P + PA(\hat{x})^\top + Q$ 中，$A$ 是近似的（因为 $\hat{x}\neq x$）。InEKF 的 $\dot{P} = A(u)P + PA(u)^\top + Q$ 中，$A$ 是精确的（Theorem 2）。这意味着 InEKF 的协方差**精确反映真实不确定性**，而 EKF 的协方差可能过度乐观或过度保守。

2. **FEJ 不再需要**：标准 EKF 在 SLAM/VIO 中需要 FEJ（First Estimate Jacobian）来避免"虚假可观性"——即估计值变化导致原本不可观的方向变得看起来可观。InEKF 的 $A$ 天然不依赖估计，自动避免了这个问题。

3. **初始化鲁棒性**：Theorem 4 保证 InEKF 的收敛半径 $\varepsilon$ 不依赖初始化时刻 $t_0$——无论机器人在第 1 秒还是第 100 秒开始运行，收敛域相同。EKF 的收敛域随 $\hat{x}$ 偏离真值而缩小。

**数值实例**（Hartley IJRR 2020, Table II）：在 Cassie 双足机器人实验中，InEKF 相比 MSCKF（使用标准 EKF Jacobian + FEJ）：
- 位置 RMSE 降低 ~30%
- 姿态 RMSE 降低 ~25%
- 在快速运动（跑步、跳跃）中改善更显著——因为大估计误差使 EKF 的 $A(\hat{x})$ 偏差更大

> **反事实推理——如果 InEKF 的 $A$ 也依赖估计会怎样？** 那么 Theorem 2 的"精确线性"就退化为"一阶近似"——与标准 EKF 无异。所有关于 Riccati 精确性、初始化无关稳定性的结论都失效。系统回到"需要 FEJ + OC-EKF 修补"的状态。这就是为什么 group-affine 条件如此珍贵——它是从根本上消除"线性化依赖估计"问题的代数条件，而不是工程修补。

---

### §D.28 不变观测与可观性：从 $H$ 独立于估计到 yaw 零空间 ⭐⭐⭐

传播段的 log-linear 只解决了一半问题。若观测 Jacobian 仍强依赖估计并破坏 gauge 对称性，滤波器仍可能过度自信。因此 TAC 2017 把 invariant observation 与 group-affine dynamics 放在同一框架下。

考虑 landmark/body-frame 观测

$$
y=R^\top(\ell-p)+n.
$$

对整体平移变换

$$
p\mapsto p+t,\qquad \ell\mapsto\ell+t,
$$

观测不变：

$$
R^\top((\ell+t)-(p+t))=R^\top(\ell-p).
$$

对绕重力方向整体 yaw 旋转，若相机、IMU 与地图一起变换，纯相对观测也不改变。这说明全局平移与全局 yaw 是物理不可观方向。

在线性化系统中，设不可观方向矩阵为 $N$。一致性要求

$$
HN=0,\qquad H\Phi N=0,\qquad H\Phi_{k,0}N=0.
$$

标准 EKF 的困难在于 $H(\hat x_k)$ 和 $\Phi(\hat x_k)$ 依赖不同时间的估计点，$N$ 在堆叠可观性矩阵中不再保持同一含义。

InEKF 的处理方式是把误差定义在群上，使全局变换变成误差坐标中的对称方向。对 fundamental observation，观测残差在单位元附近线性化：

$$
z\approx H\xi+n,
$$

其中 $H$ 不依赖当前全局位姿估计。这样，若物理观测对某个群作用不敏感，线性化后的 $H$ 也保留这一不敏感性。

这一点对 VIO/SLAM 非常关键。yaw 漂移本来不可避免，但滤波器应该知道"我不知道 yaw"。若滤波器把 yaw 协方差压小，它会在后续地图更新中拒绝真实的新息，导致轨迹看起来短期平滑、长期不一致。

从诊断角度，建议在实现中加入两个单元测试：

1. 构造全局平移方向 $N_t$，验证每个 landmark residual 的 $HN_t$ 数值接近 0。
2. 构造小 yaw 方向 $N_\psi$，在无磁力计/无绝对航向观测时验证堆叠观测对该方向不敏感。

若测试失败，不要先调噪声。应先检查误差定义、Jacobian 符号、左右扰动约定和 landmark/pose 块排序。

### §D.29 IMU、SLAM 与接触辅助：同一个代数模板 ⭐⭐

Barrau-Bonnabel 论文的一个重要价值，是把看似不同的机器人估计问题压到同一个代数模板：

| 场景 | 群状态中的额外列 | 观测形式 | 工程解释 |
|---|---|---|---|
| IMU 导航 | $v,p$ | GNSS/高度/速度 | 惯性积分链 |
| landmark SLAM | $\ell_i$ | $R^\top(\ell_i-p)$ | 地图点相对机器人 |
| contact-aided odometry | $d_i$ | $R^\top(d_i-p)$ | 接触足端相对 base |
| multi-frame clone | $p_j,R_j$ 的扩展状态 | 相对约束 | 滑窗内历史位姿 |

landmark 与 contact point 的数学形式几乎一样：

$$
y_i=R^\top(s_i-p)+n_i,
$$

其中 $s_i$ 可以是地图点 $\ell_i$，也可以是接触足端 $d_i$。差异只在过程模型：

$$
\dot\ell_i=0
$$

表示地图点静止；

$$
\dot d_i=0\quad\text{during contact}
$$

表示足端在接触相相对世界静止；

$$
\dot d_i\ \text{reset}
$$

表示接触切换时需要重新初始化该列。

这个统一视角对腿式机器人尤其重要。腿式里程计不是"把腿部运动学硬塞进 EKF"，而是把足端接触点看作短寿命 landmark。接触成立时，它提供类似 landmark 的相对位置观测；接触解除时，该列的物理意义消失，需要边缘化或重置。

如果不按这个模板思考，常见错误是把足端位置当作普通欧氏 bias 一直估计。这样会让已经离地的脚继续对 base pose 施加约束，接触切换后产生不连续误差。正确做法是把接触状态作为观测可用性的开关，并在新接触建立时用当前运动学初始化新的接触点列。

### §D.30 公式复核清单：哪些地方最容易差一个符号 ⭐⭐

本专题公式多，最容易出现的错误不是大定理，而是局部符号。实现或授课前建议逐项复核：

| 公式点 | 易错形式 | 复核方法 |
|---|---|---|
| 左/右不变误差 | 把 $\chi^{-1}\hat\chi$ 与 $\hat\chi\chi^{-1}$ 混用 | 对整体左乘/右乘测试是否不变 |
| group-affine 减项 | 漏掉 $-af(e)b$ | 令 $a=e$ 或 $b=e$ 检查退化 |
| $SE_2(3)$ Adjoint | $v^\wedge R$ 与 $p^\wedge R$ 块顺序错 | 对照李代数排序 $[\omega,\nu_v,\nu_p]$ |
| IMU $A$ 矩阵 | $g^\wedge$ 符号反 | 用小 roll 误差检查重力投影方向 |
| landmark Jacobian | 姿态块 $-y^\wedge$ 写成 $+y^\wedge$ | 对一个小 yaw 数值差分 |
| reset Jacobian | 左扰动/右扰动符号相反 | 注入一个已知小角并比较新旧误差 |
| bias 增广 | 误以为仍 perfect group-affine | 检查 $f(ab)$ 是否出现无法抵消的 bias 交叉项 |

这些复核点也是后续数学复查的重点。特别是 $SE_2(3)$ 的符号取决于采用 Barrau-Bonnabel、Hartley 还是库实现的切空间排序；同一公式在不同排序下块位置会改变，但几何含义不能改变。

---

### §D.31 稳定性定理的工程读法：$\varepsilon$ 独立于 $t_0$ 为什么重要 ⭐⭐⭐

Theorem 4 中最容易被忽略的一句是：收敛半径 $\varepsilon$ 独立于初始化时间 $t_0$。这句话看起来像技术条件，实际非常工程化。

普通 EKF 的稳定性证明通常依赖沿某条真实轨迹的局部线性化：

$$
\dot e=A(\hat x_t,u_t)e+\mathcal O(\|e\|^2).
$$

二阶项的系数会随轨迹、估计值和时间变化。若机器人已经运行很久，$\hat x_t$ 可能进入一个与初始证明区域完全不同的状态区域。此时早期得到的局部收敛半径不再直接适用。

InEKF 的传播段不同：

$$
\dot\xi=A(u_t)\xi.
$$

传播没有二阶余项；余项主要来自观测更新中的局部线性化。若系统满足均匀可观性和噪声有界条件，Deyst-Price 型 LTV-KF 稳定性可以在任意时间窗口上重复使用，因此收敛半径不依赖"从什么时候开始滤波"。

对机器人而言，这意味着：

| 场景 | 普通局部 EKF 的担忧 | InEKF 稳定性结论的意义 |
|---|---|---|
| 摔倒后重新站起 | 初始姿态误差大，轨迹与正常行走差异大 | 只要误差在统一邻域内，局部收敛论证仍适用 |
| 长时间巡检 | 运行时间越久，线性化区域越难事先界定 | 可用滑动时间窗口的均匀条件描述 |
| 接触频繁切换 | 每次接触改变观测组合 | 传播结构不随估计漂移改变 |
| 地图增量扩展 | 状态维度和 landmark 集变化 | 只要扩展群与观测保持不变结构，可复用同一分析框架 |

这并不表示 InEKF 全局收敛。$\log$ 映射仍有单射半径限制，观测仍可能退化，接触误检仍会破坏模型。它的意义更精确：在可观性和局部误差条件满足时，收敛邻域不随初始化时刻漂移，这是普通 EKF 很难保证的性质。

### §D.32 读论文时的最小复现路线 ⭐⭐

若要真正掌握 Barrau-Bonnabel TAC 2017，不建议从头到尾只读文字。更有效的复现路线是把定理变成 5 个可计算实验。

| 实验 | 要验证的命题 | 最小做法 |
|---|---|---|
| group-affine 代数实验 | $f(ab)=f(a)b+af(b)-af(e)b$ | 随机生成两个 $SE_2(3)$ 状态，逐块比较两边 |
| log-linear 传播实验 | $\eta_t=\exp(\Phi_t\xi_0)$ | 随机初始误差，非线性传播真值与估计，再比较 $\log\eta$ |
| 幂零矩阵实验 | $A^3=0$ | 计算 $A,A^2,A^3$，验证 $\Phi=I+A\Delta t+\frac12A^2\Delta t^2$ |
| 不可观方向实验 | $HN=0$ | 构造全局平移/yaw 扰动，对 landmark residual 做数值差分 |
| bias 反例实验 | bias 破坏 group-affine | 在 $SO(3)\times\mathbb R^3$ 上随机取 $b_1,b_2,R_1,R_2$，检查恒等式残差 |

这些实验不需要完整机器人系统。它们只需要矩阵指数、对数、随机状态采样和数值差分。若这些实验不通过，直接集成到 VIO/LIO 中只会把代数错误藏得更深。

### §D.33 离散时间 InEKF 完整实现 ⭐⭐⭐

#### 动机

§D.13 给出了离散 IEKF 的 Riccati 方程框架，但仅列出公式而未给出完整的 predict-update 循环伪代码。工程实现中最常见的困惑不是单个公式，而是"这些公式按什么顺序组合"。本节给出**右不变 InEKF**（RIEKF）的完整离散 predict-update 循环，包含 Exp-based 状态传播、不变创新量计算和协方差更新，并对比连续时间公式讨论"何时用哪种"。

#### 33.1 离散预测步（Prediction）

设时刻 $t_k$ 的估计为 $\hat{\chi}_k^+ \in G$，协方差为 $P_k^+$。给定输入 $u_k$（如 IMU 角速度 $\omega_k$ 和加速度 $a_k$），预测步包含两部分：

**（A）状态传播**——在群上用 Exp 映射前推：

$$
\hat{\chi}_{k+1}^- = f_\Delta(\hat{\chi}_k^+, u_k)
$$

对 $SE_2(3)$ 上的 IMU 导航，展开为：

$$
\hat{R}_{k+1}^- = \hat{R}_k^+ \cdot \mathrm{Exp}(\omega_k \Delta t)
$$

$$
\hat{v}_{k+1}^- = \hat{v}_k^+ + (g + \hat{R}_k^+ a_k) \Delta t
$$

$$
\hat{p}_{k+1}^- = \hat{p}_k^+ + \hat{v}_k^+ \Delta t + \frac{1}{2}(g + \hat{R}_k^+ a_k) \Delta t^2
$$

注意旋转部分使用 $\mathrm{Exp}$（指数映射）而非欧拉角积分——这保持了 $\hat{R}$ 始终在 $SO(3)$ 上，无需额外重正交化。

**（B）协方差传播**——使用仅依赖输入的 $A_t$ 矩阵：

$$
\Phi_k = \exp(A_{u_k} \Delta t) = I + A_{u_k}\Delta t + \frac{1}{2}A_{u_k}^2\Delta t^2
$$

回顾 §D.27：对 $SE_2(3)$ IMU 系统 $A_{u_k}^3 = 0$，上式是**精确**的矩阵指数（不是截断近似！）。

$$
P_{k+1}^- = \Phi_k P_k^+ \Phi_k^\top + Q_d
$$

其中 $Q_d$ 是离散化后的过程噪声协方差。对 RIEKF，$Q_d$ 需要通过 $\mathrm{Ad}_{\hat{\chi}}$ 把 body-frame IMU 噪声搬到右不变误差坐标中：

$$
Q_d = \mathrm{Ad}_{\hat{\chi}_k^+} \cdot Q_{\text{body}} \cdot \mathrm{Ad}_{\hat{\chi}_k^+}^\top \cdot \Delta t
$$

> **反事实推理**：如果不用 $\Phi_k = \exp(A\Delta t)$ 而改用一阶欧拉 $\Phi_k \approx I + A\Delta t$，在 $\Delta t = 0.01\,\text{s}$ 时误差多大？$A^2\Delta t^2/2$ 的贡献约为 $g \cdot \Delta t^2 / 2 \approx 4.9 \times 10^{-4}$，看似很小，但经过 $K = 100$ 步累积后位置误差可达 $\sim 0.05\,\text{m}$。对足式机器人精确里程计，这是不可接受的。幸运的是 $A^3 = 0$ 保证了二阶截断即精确。

#### 33.2 离散更新步（Update）

收到量测 $Y_n$ 时（如 GPS 位置、contact kinematics）：

**（A）计算不变创新量**：

对右不变观测 $Y_n = \hat{\chi}_{t_n}^{-1} d + V_n$，创新量为：

$$
z_n = \hat{\chi}_n^{-} Y_n - d
$$

这个残差只依赖不变误差 $\eta^R = \hat{\chi}\chi^{-1}$——在无噪声极限下 $z_n = (\eta^R - I)d$，仅是误差 $\eta^R$ 的函数。

**（B）计算 Kalman 增益**：

$$
S_n = H P_n^- H^\top + N_n, \qquad L_n = P_n^- H^\top S_n^{-1}
$$

$H$ 由不变观测的线性化给出（§D.15），**不依赖当前估计**——这是 InEKF 的核心优势。对 GPS 观测 $Y = \chi e$（左不变，LIEKF 更合适），$H = (0_{3\times3}, 0_{3\times3}, I_3)$。

**（C）状态更新**——在群上用 Exp：

$$
\hat{\chi}_n^+ = \hat{\chi}_n^- \cdot \mathrm{Exp}(L_n z_n)
$$

对 RIEKF，更新是**右乘** $\mathrm{Exp}$；对 LIEKF，更新是**左乘**：

$$
\hat{\chi}_n^+ = \mathrm{Exp}(L_n z_n) \cdot \hat{\chi}_n^-
$$

**（D）协方差更新**——标准 Joseph 形式保证正定：

$$
P_n^+ = (I - L_n H) P_n^- (I - L_n H)^\top + L_n N_n L_n^\top
$$

简化形式 $P_n^+ = (I - L_n H)P_n^-$ 在数值上不保证正定，工程中**必须**使用 Joseph 形式。

#### 33.3 完整 predict-update 伪代码

```text
算法：Right-Invariant EKF (RIEKF) 离散实现
──────────────────────────────────────────
输入：初始估计 χ̂₀, P₀; IMU 数据流 {ωₖ, aₖ}; 量测流 {Yₙ, tₙ}
输出：每个时刻的估计 χ̂ₖ 和协方差 Pₖ

初始化：χ̂ ← χ̂₀, P ← P₀

FOR each IMU measurement (ωₖ, aₖ) at rate 1/Δt:
    // ——— 预测步 ———
    1. 状态前推：
       R̂ ← R̂ · Exp(ωₖΔt)
       v̂ ← v̂ + (g + R̂aₖ)Δt
       p̂ ← p̂ + v̂Δt + ½(g + R̂aₖ)Δt²

    2. 误差转移矩阵（SE₂(3) 幂零结构）：
       Φ ← I₉ + AΔt + ½A²Δt²
       其中 A = [[0, 0, 0], [g×, 0, 0], [0, I₃, 0]]

    3. 协方差传播：
       Qd ← Ad(χ̂) · Q_body · Ad(χ̂)ᵀ · Δt
       P ← ΦPΦᵀ + Qd

    // ——— 更新步（仅在收到量测时执行）———
    IF 量测 Yₙ 到达 at time tₙ:
        4. 不变创新量：
           z ← χ̂·Yₙ - d       （右不变观测）

        5. Kalman 增益：
           S ← HPHᵀ + N
           L ← PHᵀS⁻¹

        6. 状态更新：
           χ̂ ← χ̂ · Exp(Lz)   （RIEKF 右乘）

        7. 协方差更新（Joseph 形式）：
           P ← (I-LH)P(I-LH)ᵀ + LNLᵀ

RETURN χ̂, P
```

#### 33.4 连续时间 vs 离散时间：何时用哪种

| 场景 | 推荐形式 | 理由 |
|------|----------|------|
| IMU 高频积分（200-1000 Hz）| 离散时间 | 输入本身是采样值，$\Phi = \exp(A\Delta t)$ 精确 |
| 连续动力学仿真 + 稀疏量测 | 连续-离散（CD）混合 | 精确积分 Riccati ODE，避免离散化误差 |
| 理论分析 / 稳定性证明 | 连续时间 | Theorem 2/4 的原始形式，数学更简洁 |
| 大步长离散化（$\Delta t > 0.05\,\text{s}$）| 需要谨慎 | ZOH 假设被破坏，考虑多步 RK4 积分 |

**Barrau-Bonnabel SCL 2019 的离散 group-affine**条件为 $F_n(ab) = F_n(a)F_n(e)^{-1}F_n(b)$，保证离散误差也满足 log-linear。当输入在 $[t_k, t_{k+1}]$ 内分段常数时，$\Phi_k = \exp(A_{u_k}\Delta t)$ 自动满足此条件。但若输入快速变化（如高频振动），应考虑在 $\Delta t$ 内做 Runge-Kutta-Munthe-Kaas（RKMK）保李群积分。

⚠️ **陷阱：更新步中用错乘法方向**

- 错误做法：RIEKF 中把状态更新写成 $\hat{\chi}^+ = \mathrm{Exp}(Lz) \cdot \hat{\chi}^-$（左乘）。
- 后果：增益方向与协方差定义不匹配，滤波器在前几步"看似正常"但逐渐发散，NEES 持续偏高。
- 根本原因：RIEKF 的误差定义为 $\eta^R = \hat{\chi}\chi^{-1}$，更新必须在**右侧**施加修正才能正确减小右不变误差。
- 正确做法：RIEKF 右乘，LIEKF 左乘。记忆法则：误差"从哪边定义"，更新就"从哪边乘"。

---

### §D.34 接触辅助 InEKF（足式机器人） ⭐⭐⭐

#### 动机：腿式机器人的状态估计挑战

足式机器人不同于轮式车辆——没有连续的轮速计，取而代之的是间歇性的脚地接触。接触提供了富信息的相对位置约束，但接触的离散切换使得观测模型呈现"时有时无"的特征。Hartley, Ghaffari, Eustice, Grizzle（IJRR 2020）提出的 **Contact-Aided Invariant EKF**，把足端接触点视为扩展群 $SE_{N+2}(3)$ 的"额外列"，将接触运动学纳入 InEKF 的统一代数框架。

#### 34.1 Hartley IJRR 2020 框架概述

**核心思想**：当机器人的第 $i$ 条腿处于接触状态时，接触点 $d_i$ 在世界系中静止（$\dot{d}_i = 0$）。这与 SLAM 中的静态地标完全类似——$d_i$ 扮演"短寿命 landmark"的角色。当接触解除时，该点不再提供约束，应被边缘化或重置。

**状态表示**——把接触点嵌入扩展群 $SE_{N+2}(3)$：

$$
\chi = \begin{bmatrix}
R & v & p & d_1 & \cdots & d_N \\
0 & 1 & 0 & 0 & \cdots & 0 \\
0 & 0 & 1 & 0 & \cdots & 0 \\
\vdots & & & & \ddots & \vdots \\
0 & 0 & 0 & 0 & \cdots & 1
\end{bmatrix} \in SE_{N+2}(3)
$$

其中 $N$ 是当前活跃接触点数量（对四足机器人最多 $N=4$）。$R \in SO(3)$ 是体帧朝向，$v, p, d_i \in \mathbb{R}^3$ 分别是速度、位置和各接触点的世界坐标。

> **跨领域类比**：把接触点放进群矩阵，本质上和 SLAM 中把 landmark 放进状态向量一样——只是 landmark 寿命很长（地图持久存在），而接触点寿命很短（一个步态周期）。数学结构完全一致：$\dot{d}_i = 0$ vs $\dot{\ell}_j = 0$，观测方程 $y_i = R^\top(d_i - p) + n$ vs $y_j = R^\top(\ell_j - p) + n$。这个统一视角是 §D.29 "同一个代数模板"的具体工程体现。

#### 34.2 接触作为"零速度"量测

当第 $i$ 条腿接触地面时，前向运动学（FK）提供了从 base 到足端的相对位置：

$$
y_i^{\text{FK}} = R^\top(d_i - p) = J_i(q)\,q_{\text{leg},i}
$$

这是一个**右不变观测**（参考向量 $d_i$ 在世界系中，body frame 读取）。对应的不变残差为：

$$
z_i = \hat{\chi} \cdot Y_i - e_i
$$

其中 $e_i$ 是 $SE_{N+2}(3)$ 中第 $i$ 个接触列的选择器向量。由 §D.15 的分析，此观测的 Jacobian $H_i$ **不依赖当前估计**。

**零速度约束**：接触期间 $\dot{d}_i = 0$——这不仅是位置约束，更隐含了足端速度为零。速度约束可作为额外量测纳入：

$$
v_{\text{foot},i} = v + \omega \times (d_i - p) + R \dot{J}_i q_{\text{leg}} = 0
$$

这为状态估计提供了丰富的可观性——即使没有 GPS，仅靠 IMU + 接触运动学就能估计机器人位姿。

#### 34.3 为什么 contact-aided InEKF 优于 ESKF

Hartley IJRR 2020 通过 60 次 Monte Carlo 实验（初始 yaw 误差 $\pm 45°$）在 Cassie 双足机器人上验证了 InEKF 相对于标准 quaternion-EKF 的优势：

| 指标 | Quaternion-EKF | Contact-Aided InEKF | 改善幅度 |
|------|---------------|---------------------|---------|
| 位置 RMSE | ~0.5% 行走距离 | ~0.3% 行走距离 | **~30%** |
| Yaw RMSE | 较大 | 降低 >50% | **>50%** |
| 收敛盆地 | 小，依赖初始化精度 | 显著更大 | 鲁棒性提升 |
| NEES 一致性 | 长时间后偏高 | 始终在置信带内 | 结构性改善 |

性能差异的根本原因可以从 §D.10 的理论框架理解：

1. **传播 Jacobian $A$ 不依赖估计**：InEKF 中 $A$ 仅含重力 $g$ 和输入 $\omega, a$，即使估计姿态有偏差，$A$ 不会跟着偏。QEKF 的 $A(\hat{q}, u)$ 会因姿态误差而系统性偏离真值。
2. **观测 Jacobian $H$ 不依赖估计**：接触观测是 fundamental right-invariant，$H$ 仅依赖选择器向量 $e_i$。QEKF 中 $H$ 依赖 $\hat{R}$，在大角度误差时失真。
3. **无虚假可观性**：QEKF 中不同时刻的 $A(\hat{x})$ 和 $H(\hat{x})$ 使用不同的估计，导致堆叠可观性矩阵的零空间不一致（$HN \neq 0$），使 yaw 虚假可观。InEKF 中 $A(u)$ 和 $H$ 都不依赖估计，零空间自动一致。

> **本质洞察**：Contact-aided InEKF 的力量不是来自"接触信息更丰富"（QEKF 也用同样的接触信息），而是来自**误差定义与群结构的匹配**。把接触点嵌入 $SE_{N+2}(3)$ 后，接触观测自然成为 fundamental right-invariant observation，享有 Theorem 4 的全部保证。同样的接触信息在 QEKF 中无法享有这些结构性优势。

#### 34.4 接触状态切换与重置

足式行走中接触状态频繁切换。当第 $i$ 条腿从摆动切换到接触时：

1. **初始化新接触列**：用当前估计的 base 位姿 $(\hat{R}, \hat{p})$ 和前向运动学 $\hat{d}_i = \hat{p} + \hat{R} \cdot \text{FK}_i(\hat{q})$ 初始化 $d_i$
2. **扩展协方差**：在 $P$ 矩阵中增加新行列，初始协方差来自运动学和位姿估计的不确定性传播
3. **增加观测维度**：$H$ 矩阵增加对应行

当接触解除时：

1. **边缘化接触列**：从状态和协方差中移除 $d_i$，保留其与其他状态的交叉协方差对剩余状态的信息贡献（Schur 补）
2. **或直接删除**：工程中常简单地移除接触列和对应协方差块，牺牲少量信息但避免矩阵求逆

**开源实现**：`github.com/UMich-BipedLab/Contact-Aided-Invariant-EKF` 提供了完整的 C++ 实现和 ROS wrapper（`github.com/RossHartley/invariant-ekf-ros`），支持 Cassie 和 Mini Cheetah 的实验数据。

> **与运动控制/足式方向的联系**：Contact-aided InEKF 不仅是数学工具，更是足式机器人运动控制栈的关键组件。MIT Mini Cheetah 的控制架构（Bledt et al., IROS 2018）中，状态估计器的输出直接驱动 MPC 的状态反馈和 WBC 的参考轨迹。InEKF 的一致性（NEES 不偏高）意味着协方差矩阵可靠——这对 MPC 中约束的概率化处理（chance constraint）至关重要。详见运动控制/足式方向的相关章节。

#### 34.5 练习

1. **（⭐⭐⭐）** 写出 $SE_4(3)$（四足机器人四个接触点）的矩阵形式和群乘法规则。验证 $\dot{d}_i = 0$ 对应的动力学仍然满足 group-affine。
2. **（⭐⭐⭐）** 分析接触切换时刻的协方差不连续性：边缘化 vs 直接删除在信息论上的差异是什么？在什么条件下直接删除是合理的近似？
3. **（⭐⭐⭐⭐）** Hartley IJRR 2020 Table I 给出了 Cassie 实验的具体参数。如果 Cassie 的接触检测有 10ms 延迟，如何在 InEKF 框架中处理这个延迟？提示：考虑 state augmentation 或 measurement delay compensation。

---

### §D.35 InEKF 实用调参与 Allan 方差 ⭐⭐⭐

#### 动机

前面各节的理论保证（Theorem 2, 4）建立在"噪声参数正确"的前提上。但工程中 $Q_t$（过程噪声）和 $N_n$（量测噪声）的设置往往是最耗时的步骤。本节建立从 Allan 方差到 InEKF 噪声参数的完整转换链，结合开源实现的实践经验给出调参指南。

#### 35.1 从 Allan 方差到 InEKF 噪声参数

**连续时间过程噪声** $Q_c$：InEKF 的连续传播 $\dot{P} = A P + P A^\top + Q_c$ 中，$Q_c$ 来自 IMU 白噪声：

$$
Q_c = \begin{bmatrix}
\sigma_g^2 I_3 & 0 & 0 \\
0 & \sigma_a^2 I_3 & 0 \\
0 & 0 & 0_{3\times3}
\end{bmatrix} \in \mathbb{R}^{9 \times 9}
$$

其中 $\sigma_g$ 和 $\sigma_a$ 分别是陀螺和加速度计的**噪声密度**（从 Allan 方差的 ARW 斜率提取或直接查数据手册）。

**注意坐标系**：上述 $Q_c$ 定义在 body frame 中。对 RIEKF，需要通过 Adjoint 搬运到右不变误差坐标：

$$
\hat{Q}_c = \mathrm{Ad}_{\hat{\chi}} \cdot Q_c^{\text{body}} \cdot \mathrm{Ad}_{\hat{\chi}}^\top
$$

这里 $\hat{Q}_c$ 温和依赖估计 $\hat{\chi}$（通过 $\hat{R}$），但不影响 Theorem 4 的稳定性结论（Lemma 4 量化了此微扰的有界性，见 §D.14）。

**量测噪声** $N_n$：

| 量测类型 | 噪声来源 | 典型设置方法 |
|---------|---------|-------------|
| GPS 位置 | 接收机精度 | 数据手册给出 CEP → $N_{\text{GPS}} = \sigma_{\text{GPS}}^2 I_3$ |
| Contact kinematics | 编码器精度 + 运动学误差 | FK 精度 + 关节编码器分辨率传播 |
| 磁力计 | 硬铁/软铁干扰 | 标定后残差的经验方差 |
| Barometer | 温度/气流影响 | 数据手册 + 静态残差统计 |

#### 35.2 从开源实现中学到的实践经验

**`invariant-ekf` ROS 包**（Hartley, `github.com/RossHartley/invariant-ekf`）的默认参数设置策略：

1. **过程噪声初始值**：从 IMU 数据手册的噪声密度出发，**乘以 2-5 倍安全系数**。理由：数据手册给出的是理想条件下的典型值，实际工况（振动、温度变化）下噪声通常更大。
2. **量测噪声初始值**：从传感器精度出发，**不加安全系数或仅加小量**。理由：量测噪声过大会使滤波器过于保守，错过有用信息。
3. **Bias 过程噪声**：设为 $\sigma_b \sim 10^{-4}$ 至 $10^{-3}\,\text{rad/s}/\sqrt{\text{Hz}}$（陀螺）和 $10^{-3}$ 至 $10^{-2}\,\text{m/s}^2/\sqrt{\text{Hz}}$（加速度计），具体值从 Allan 方差的 RRW 斜率提取。
4. **调参方向**：先保证 NEES $\bar{\varepsilon} \approx n$（§20 的一致性检验），再优化 RMSE。

**调参口诀**（来自 Hartley 实验室的经验总结）：

```text
NEES 偏高（过度自信）→ 增大 Q 或减小 N
NEES 偏低（过度保守）→ 减小 Q 或增大 N
位置漂移但 NEES 正常 → 检查 bias 是否被正确估计
接触切换时跳变    → 检查 FK 精度和接触检测延迟
```

#### 35.3 InEKF vs ESKF 的实用对比

在实际部署中，选择 InEKF 还是 ESKF（Error-State KF）取决于系统特征：

| 对比维度 | InEKF | ESKF（Sola 2017） |
|---------|-------|-------------------|
| Jacobian 依赖性 | $A(u)$ 不依赖估计 | $A(\hat{x}, u)$ 依赖估计 |
| 一致性 | 结构性保证（group-affine） | 需 FEJ/OC-EKF 修正 |
| Bias 处理 | Imperfect（$F_{pb}$ 依赖估计） | 自然融入（但整体依赖估计） |
| 调参鲁棒性 | 更鲁棒（收敛盆地大） | 对初始化更敏感 |
| 实现复杂度 | 需要李群运算库 | 向量空间运算即可 |
| 计算开销 | 略高（$\mathrm{Ad}$ 运算） | 略低 |
| 适用场景 | 长时间运行、大初始误差 | 短时间、已有好初始化 |

> **本质洞察**：InEKF 和 ESKF 的核心差异不是"算法步骤不同"，而是"误差定义不同"。ESKF 把误差定义为切空间中的加性扰动 $\delta x$，误差方程 $\dot{\delta x} = A(\hat{x})\delta x + \cdots$ 中 $A$ 依赖估计；InEKF 把误差定义为群元素 $\eta = \hat{\chi}\chi^{-1}$，误差方程 $\dot{\xi} = A(u)\xi$ 中 $A$ 不依赖估计。两种误差定义都是数学上合理的，但后者在 group-affine 系统上享有结构性优势。选择哪种，取决于你的系统是否 group-affine——如果是，InEKF 优于 ESKF；如果不是（如复杂的 bias/时延/外参耦合），两者差异不大，ESKF 可能因实现简单而更实用。

近期的 **CT-ESKF**（Covariance Transformation-based ESKF，arXiv:2511.00453, 2025）提出了一种统一框架：通过协方差变换把不同误差定义下的 ESKF 算法统一起来，实验表明在含全局和体帧混合观测的导航系统中，CT-ESKF 可能优于 InEKF 和原始 ESKF。这提示"最佳误差定义"可能取决于具体的观测组合。

⚠️ **陷阱：在 bias 项上也期望 InEKF 的结构性优势**

- 错误想法："InEKF 的 $A$ 不依赖估计，所以带 bias 的 InEKF 也一定比 ESKF 好。"
- 后果：过度信任 InEKF 的一致性保证，忽略了 $F_{pb}$ 块对估计的依赖（§D.17）。
- 根本原因：bias 破坏 group-affine（negative result），带 bias 的 InEKF 是 "imperfect" 的——pose 块享有优势，但 bias 块退化为普通 EKF 水平。
- 正确认识：InEKF 在 pose 估计上有结构性优势（实测 RMSE 降低 ~30%），但 bias 估计精度与 ESKF 相当。整体性能仍优于纯 ESKF，但不是"完美"的。

#### 35.4 练习

1. **（⭐⭐）** 查找 ADIS16470 数据手册，提取陀螺和加速度计的噪声密度。将其转换为 InEKF 连续时间过程噪声 $Q_c$ 的对角元素（注意单位转换）。
2. **（⭐⭐⭐）** 对一个 SO(3) + bias 的简单 IMU 估计问题，分别实现 InEKF 和 ESKF，运行 $N=50$ 次 Monte Carlo。画出两者的平均 NEES 曲线，验证 InEKF 是否更一致。特别观察：在 bias 初始误差很大时，两者的差异是否更显著？
3. **（⭐⭐⭐）** 解释为什么 `invariant-ekf` ROS 包建议对过程噪声乘以安全系数但对量测噪声不加。从 Bayesian 滤波的角度分析：过度自信和过度保守哪个后果更严重？

---

### §D.36 与 A3/A4 的闭环关系 ⭐⭐

本专题的定理在 A3/A4 中分别承担不同角色：

| 文档 | 使用方式 | 读者应带走的能力 |
|---|---|---|
| A3 | 把 MEKF/ESKF/InEKF/UKF-M 放到流形滤波谱系 | 能写出误差定义和基本算法 |
| A4 | 把 InEKF 放入 Kalman 族全景选型 | 能判断何时用迭代、何时用不变结构、何时用平滑 |
| D | 证明 group-affine 与 log-linear 的数学来源 | 能解释为什么该结构带来一致性与稳定性优势 |
| D §D.33-35 | 离散实现 + 接触辅助 + 实用调参 | 能从数据手册出发实现完整 InEKF 并通过一致性检验 |

如果只读 A3，容易把 InEKF 当作一个工程算法；如果只读 D，容易停在定理层。两者合起来才完整：A3/A4 告诉你何时用，D 告诉你为什么成立以及哪里会失效。§D.33-35 新增的实用内容则补全了"怎么落地"这个最后环节——从离散算法伪代码到 IMU 噪声参数提取，再到足式机器人的接触建模。

最后再强调一个边界：group-affine 是强条件，不是所有机器人估计问题都满足。遇到复杂 bias、外参、时间延迟、滚动快门、非刚性接触时，应先分析对称性是否仍存在；若不存在，应转向 EqF、滑窗优化或混合建模，而不是把所有状态强行塞进一个矩阵李群。

---

### §D.37 最小单元测试设计：把定理变成可运行检查 ⭐⭐⭐

理论文档最怕停留在"证明看懂了"。对 InEKF 来说，很多符号错误只有在数值测试中才会暴露。下面给出一组最小单元测试设计，用于把本文的定理级结论转成工程检查。

**测试 1：群乘法闭合。**

随机生成两个 $SE_2(3)$ 元素 $X_1,X_2$，检查 $X_1X_2$ 的结构是否仍为

$$
\begin{bmatrix}
R&v&p\\
0&1&0\\
0&0&1
\end{bmatrix}.
$$

同时检查 $R^\top R=I$ 与 $\det R=1$。若这一步失败，后面的 group-affine 测试没有意义。

**测试 2：group-affine 残差。**

定义

$$
\mathcal R(X_1,X_2)=f(X_1X_2)-f(X_1)X_2-X_1f(X_2)+X_1f(I)X_2.
$$

随机采样 $X_1,X_2,\omega,a$，验证 $\|\mathcal R\|$ 接近机器精度。为了定位错误，应分别打印旋转块、速度列、位置列的范数，而不是只打印整体 Frobenius 范数。

**测试 3：log-linear 传播。**

随机取小误差 $\xi_0$，令

$$
\eta_0=\exp(\xi_0^\wedge).
$$

用非线性误差 ODE 传播 $\eta_t$，同时用线性 ODE 传播

$$
\xi_t=\Phi_t\xi_0.
$$

检查

$$
\|\log(\eta_t)^\vee-\xi_t\|
$$

是否在数值积分误差范围内。这个测试能同时发现 $A$ 矩阵符号错误和 $\exp/\log$ 排序错误。

**测试 4：观测不可观方向。**

对 landmark residual

$$
r=R^\top(\ell-p)-y
$$

构造全局平移扰动

$$
p\mapsto p+\epsilon t,\qquad \ell\mapsto \ell+\epsilon t.
$$

用数值差分验证

$$
\frac{r(\epsilon)-r(0)}{\epsilon}\approx 0.
$$

再构造绕重力方向的小 yaw 扰动，对无绝对航向观测的系统执行同样检查。

**测试 5：bias 反例。**

在 $SO(3)\times\mathbb R^3$ 上定义

$$
f(R,b)=\left(R(\omega_m-b)^\wedge,0\right).
$$

随机采样 $(R_1,b_1),(R_2,b_2)$，计算 group-affine 残差。该残差一般不为零。这个测试的意义不是让实现通过，而是提醒工程师：bias 增广后的系统不再享有 perfect InEKF 的完整结论。

这五个测试覆盖了本文的主链条：群结构、group-affine、log-linear、不可观方向、bias 边界。若它们都通过，再进入完整滤波器实现；若其中任意一个失败，应先修代数层，而不是调 $Q/R$。

---

### §D.29c 三大证明的统一结构：从代数到分析 ⭐⭐⭐⭐

回顾全文的三大定理，它们形成了一条严密的逻辑链：

**第一环：代数条件 → 微分几何后果（Theorem 1）**

Group-affine 条件 (7) 是一个纯代数恒等式。它的后果是：误差 ODE 自治——不依赖真值轨迹。证明方法是直接计算（求导 + 代入 + 化简），属于微分方程定性理论中"自治系统判定"的范畴。

关键技巧：利用群逆的求导公式 ($\star$) 把 $\dot\eta$ 展开为 $f$ 的组合，再用 group-affine 恒等式把结果化为仅含 $\eta$ 的表达式。

**第二环：微分几何后果 → 分析后果（Theorem 2）**

误差自治 + 群同态性质 → 在对数坐标下精确线性。证明方法是 BCH 公式 + 群同态的"无交叉项"性质。

关键技巧：$[\exp(X_n)]^n = \exp(nX_n)$ 中 $[X_n, X_n]=0$ 自动消除所有 BCH 交叉项。这一步把无穷级数的交叉项分析简化为一个代数事实。

**第三环：分析后果 → 稳定性保证（Theorem 4）**

精确线性 ODE → Deyst-Price LTV-KF 稳定性定理直接适用 → 收敛半径 $\varepsilon$ 不依赖 $t_0$。

关键技巧：因为误差 ODE 精确线性，Riccati 协方差传播精确，可以把 Deyst-Price 1968 的 LTV Kalman 滤波稳定性结果"原封不动"应用在对数坐标中。标准 EKF 无法做到这一点，因为它的线性化是近似的——余项破坏了 Deyst-Price 定理的前提条件。

**三环的逻辑链**：

$$\underbrace{\text{group-affine (代数)}}_{\text{Thm 1}} \xrightarrow{\text{流的群同态}} \underbrace{\text{log-linear (分析)}}_{\text{Thm 2}} \xrightarrow{\text{Deyst-Price}} \underbrace{\text{稳定性 (控制论)}}_{\text{Thm 4}}$$

每一环都依赖前一环，但提供了不同层面的保证。工程师可以根据需求选择"停在哪一环"：

- 只需传播段好结构 → 验证 Theorem 1 即可
- 需要协方差精确 → 验证 Theorem 2（需要 $\log$ 在收敛域内）
- 需要初始化无关的稳定性 → 需要 Theorem 4 的全部条件（均匀可观 + 有界输入）

**与 5-C iSAM2 的对偶关系**：

iSAM2 和 InEKF 从不同角度解决同一个 SLAM 估计问题：

| 方面 | iSAM2（5-C） | InEKF（5-D） |
|------|-------------|-------------|
| 视角 | 全轨迹 MAP 优化 | 递推滤波（当前状态边缘） |
| 优势 | 保留全图可重线性化 | 传播段精确、无 FEJ 需求 |
| 线性化 | 在当前估计处（可能不精确） | 在群结构下精确（group-affine） |
| 回环处理 | 原生支持（添加因子） | 需要外部因子图或 reset |
| 一致性 | 通过 fluid relin 近似保证 | group-affine 结构保证 |
| 计算量 | $O(\sqrt{N})$ 每步 | $O(d^2)$ 每步（$d$ 为状态维度） |
| 适用 | 需全轨迹一致的场景 | 需高频递推、低延迟的场景 |

**互补使用模式**：InEKF 做 400 Hz IMU 传播（精确 Riccati），iSAM2 做 10 Hz 关键帧平滑（全图一致）。前者的输出作为后者的"预测因子"，后者的更新反馈给前者的线性化点。这正是 Kimera-VIO 的架构思路。

> **本质洞察**：iSAM2 的力量来自"稀疏图结构"，InEKF 的力量来自"代数对称性"。
> 两者不矛盾——前者解决"哪些变量该更新"（结构层面），后者解决"怎么做更新最精确"（代数层面）。
> 最强的估计器将两者结合：在 group-affine 结构上定义因子图，用 iSAM2 管理稀疏性，用 InEKF 保证传播精度。

---

### 结论：从代数对称性到稳定性的统一图景

Barrau-Bonnabel TAC 2017 的最深刻洞见是：**李群上一类闭合的"广义仿射"动力学（group-affine），通过 BCH 与流的群同态性质，在指数坐标下恰好坍缩为线性时变 ODE**。这一坍缩不是"近似"而是"精确"——因为 group-affine 条件 (7) 强制流 $\Phi_t$ 满足群同态 $\Phi_t(ab)=\Phi_t(a)\Phi_t(b)$（§D.7 Lemma），群同态由其在单位元的导数完全决定，BCH 中的所有"非线性交叉项"在 $[\exp(X_n)]^n$ 的处理中由于 $[X,X]=0$ 自动消失。

这一**代数事实**直接产生**统计后果**：Riccati 协方差在传播段精确传播（§D.10），无需 FEJ/OC-EKF 这类外部一致性修正；进而产生**几何后果**：Theorem 4 的稳定性证明把 Deyst-Price 1968 LTV-KF 结果通过 BCH "上升"到李群，得到收敛半径 $\varepsilon$ **独立于初始化时刻**——这是 Reif-Unbehauen 1999 EKF 稳定性永远无法达到的。

**未竟之处**：Bias 的代数 negative result（§D.17）说明 group-affine 框架本身有边界。突破口有二：(a) **Tangent group / Two-Frames Group**（Chauchat-Barrau-Bonnabel 2022）局部恢复 group-affine；(b) **Equivariant Filter**（Mahony-van Goor 2022）放弃 group-affine 转向更弱的 equivariance，把 bias 通过对称群 lift 完全吸收。EqVIO 在 EuRoC 上的优秀表现表明 EqF 是 InEKF 的自然继承者。

---

### §D.29b 从 InEKF 到 Equivariant Filter：理论演进路线图 ⭐⭐⭐⭐

InEKF 的成功在于找到了一类特殊系统（group-affine）使得 EKF 的线性化在对数坐标下精确。但现实中很多系统不满足 group-affine（如带 bias 的 IMU、带弹性关节的机械臂、非平坦地球模型等）。自然的问题是：**能否放宽 group-affine 条件，仍保留部分优势？**

Mahony-van Goor 2022 的回答是：把 group-affine 放宽为 **equivariance**（等变性）。

**关键区别**：

| 性质 | InEKF (group-affine) | EqF (equivariant) |
|------|---------------------|-------------------|
| 系统类要求 | $f(ab)=f(a)b+af(b)-af(e)b$ | 存在群作用 $\phi$ 使 $f$ 等变 |
| 误差定义 | 固定为左/右不变误差 | 由对称群 $G$ 的作用自然定义 |
| A 矩阵独立性 | 严格不依赖估计 | 一般仍可能依赖估计，但结构更好 |
| 适用范围 | 窄：仅 group-affine 系统 | 广：任何具有对称性的系统 |
| bias 处理 | 不兼容（negative result） | 可通过 lift 到更大对称群吸收 |

**EqF 的核心思想**：不再要求系统"是群上的动力学"，而是要求系统具有某种**对称性**——存在一个李群 $G$ 作用在状态空间 $\mathcal{M}$ 上，使得动力学 $f$ 与观测 $h$ 在这个群作用下**等变**。EqF 在**齐次空间** $\mathcal{M}\cong G/H$ 上定义误差，利用等变性使误差传播获得更好的结构。

**历史演进**：

```text
Bonnabel CDC 2007          → SO(3) 左不变 EKF（最简情况）
    ↓
BMR TAC 2008               → 一般对称保持观测器（Cartan moving frame）
    ↓
Barrau-Bonnabel TAC 2017   → group-affine 条件（Theorem 1-4）
    ↓
Mahony-van Goor TAC 2022   → Equivariant Filter（放弃 group-affine，
                              转向更弱的 equivariance）
    ↓
van Goor-Mahony T-RO 2023  → EqVIO（EqF 在视觉-惯性系统上的落地）
    ↓
Fornasier et al. RA-L 2024 → Overcoming bias on Lie groups（EqF + bias）
```

每一步都在"放宽条件"和"保持优势"之间权衡。InEKF 的条件最强但结论最好（精确 log-linear）；EqF 的条件最弱但结论也最弱（只保证误差结构"比标准 EKF 好"，不保证精确线性）。

**对工程师的实际建议**：

| 系统类型 | 推荐方法 | 原因 |
|---------|---------|------|
| 无 bias 的 IMU 导航 | InEKF（$SE_2(3)$） | 完美满足 group-affine |
| 带 bias 的 IMU 导航 | Imperfect InEKF | 实际中 bias 变化慢，近似成立 |
| 带 bias + 高精度需求 | EqF 或 Tangent Group InEKF | 理论上更优，但实现复杂 |
| 视觉-惯性里程计 | EqVIO 或 InEKF + MSCKF | EqVIO 处理 feature 更自然 |
| 接触辅助导航（双足） | InEKF + $SE_k(3)$ | Hartley 2020 验证有效 |
| 非平坦地球模型 | EqF | group-affine 在非平坦地球不成立 |

---

**对工程师的启示**：当你在 $SE_2(3)$/SE(3) 上设计估计器，**首先**验证动力学是否 group-affine（写成 $A\chi+\chi B$ 形式），**然后**确保观测是 fundamental（$Y=\chi^{\pm 1}d$），**最后**正确选择 LIEKF/RIEKF 手性。三者全满足时，InEKF 提供超越 EKF 的根本性优势。任一不满足，要么调整状态群表示（如把 landmark/contact 塞入 $SE_k(3)$），要么转向 EqF 框架。

**设计 InEKF 的工程决策流程**：

```text
系统识别：写出动力学 f_u(χ)
    │
    ▼
group-affine 测试：验证 f(ab)=f(a)b+af(b)-af(e)b
    │
    ├── 通过 → 选择 LIEKF 或 RIEKF（取决于观测类型）
    │           │
    │           ├── 观测 Y=χd → RIEKF
    │           └── 观测 Y=χ⁻¹d → LIEKF
    │
    └── 不通过 → 分析原因
                  │
                  ├── bias 破坏 → 方案 A: imperfect InEKF
                  │                 方案 B: Tangent Group
                  │                 方案 C: EqF
                  │
                  └── 结构性不满足 → 标准 EKF + FEJ
                                    或 EqF（如果有对称性）
```

### 常见陷阱与故障排查

⚠️ **陷阱一：把 $\exp(A\Delta t)$ 本身当成破坏 group-affine 的来源。** 若 $A$ 常值，它是误差线性系统的精确转移；真正危险的是冻结输入、粗糙积分或错误噪声离散化。

⚠️ **陷阱二：innovation 中误用真值 $\chi$。** 实现只能用 $\hat\chi$ 和量测 $Y$ 构造残差；写成 $\chi^{-1}Y-d$ 会在数学上消掉误差。

⚠️ **陷阱三：看见李群就默认 InEKF 适用。** 必须同时满足 group-affine 动力学和 compatible observation；bias 增广往往破坏完整结论。

| 故障排查现象 | 可能原因 | 处理方式 |
|---|---|---|
| Jacobian 仍依赖当前估计 | 观测手性选错或系统不 group-affine | 检查 $Y=\chi d$ 还是 $Y=\chi^{-1}d$ |
| 大步长传播 NEES 异常 | 状态积分或噪声积分近似过粗 | 使用更细 IMU 积分或 Van Loan 离散化 |
| 加入 bias 后理论结论失效 | bias 破坏 group-affine | 采用 imperfect InEKF、tangent group 或 EqF |

> **本质洞察**：InEKF 的力量来自对称性，而不是来自“在李群上写 EKF”。只有动力学、误差和观测三者共同尊重群结构，log-linear 定理才真正成立。

### 练习

1. 对左不变观测 $Y=\chi d$，从 $\hat\chi^{-1}Y-d$ 推导它只依赖 $\eta^L=\chi^{-1}\hat\chi$。
2. 写出一个带 bias 的姿态系统，验证 group-affine 恒等式在哪个项上失败。
3. 用常值 $A$ 的线性误差系统比较 Euler 离散和 $\exp(A\Delta t)$ 离散在大步长下的差异。

---

### 本章小结

| 核心概念 | 一句话要义 | 关键公式 | 工程意义 |
|---------|-----------|---------|---------|
| Group-affine (Thm 1) | 使不变误差自治的系统类 | $f(ab)=f(a)b+af(b)-af(e)b$ | 判断 InEKF 是否适用的充要条件 |
| Log-linear (Thm 2) | 误差对数满足精确线性 ODE | $\dot\xi=A_t\xi$，$A_t$ 只含 $u_t$ | Riccati 传播精确，无需 FEJ |
| 流的群同态 (Lemma) | group-affine 流保持群乘法 | $\Phi_t(ab)=\Phi_t(a)\Phi_t(b)$ | BCH 交叉项消失的代数根源 |
| 稳定性 (Thm 4) | 收敛半径 $\varepsilon$ 不依赖 $t_0$ | Deyst-Price Lyapunov | 初始化时刻无关的鲁棒性保证 |
| 左/右不变误差 | $\eta^L=\chi^{-1}\hat\chi$, $\eta^R=\hat\chi\chi^{-1}$ | 乘法误差 vs 加法误差 | 选择手性影响观测 Jacobian |
| 不变观测 | $Y=\chi d$ (左) 或 $Y=\chi^{-1}d$ (右) | Innovation 只依赖 $\eta$ | 观测 Jacobian 不依赖估计 |
| $SE_2(3)$ 群 | 把速度纳入群结构的惯性导航群 | 5x5 矩阵包含 $R,v,p$ | IMU 导航的自然群表示 |
| Bias negative result | 陀螺 bias 破坏 group-affine | $(b_1)_\times\neq(R_2^\top b_1)_\times$ | bias 需要额外处理方案 |
| Imperfect InEKF | group-affine 近似成立时仍用 InEKF | 工程上仍优于 EKF | bias 当作慢变参数 |
| Equivariant Filter | 不要求 group-affine 的推广 | equivariance + symmetry group | InEKF 的自然继承者 |

---

### 累积项目：本章新增模块

**项目名称**：手写矩阵库 → 李群滤波器

**本章新增**：InEKF 传播段的代数验证工具

在前序章节中，累积项目已完成：
- Ch1：向量类与基本运算
- Ch2：矩阵乘法与 LU 分解
- Ch3：SVD 与最小二乘
- Ch4：$SO(3)$ / $SE(3)$ 表示与 exp/log

本章新增：
1. 实现 $SE_2(3)$ 群的矩阵表示（5x5 矩阵）
2. 实现 group-affine 残差测试函数 $\mathcal{R}(X_1,X_2)$
3. 实现 log-linear 传播测试：对比非线性误差 ODE 与线性 $\dot\xi=A_t\xi$ 的传播结果
4. 构造 IMU 动力学 $f_{\omega,a}(\chi)$ 并逐块验证 group-affine

```python
import numpy as np
from scipy.linalg import expm, logm

class SE23:
    """SE_2(3) 群的矩阵表示"""
    def __init__(self, R, v, p):
        assert R.shape == (3,3)
        self.mat = np.eye(5)
        self.mat[:3,:3] = R
        self.mat[:3,3] = v
        self.mat[:3,4] = p

    def __matmul__(self, other):
        """群乘法"""
        result = SE23.__new__(SE23)
        result.mat = self.mat @ other.mat
        return result

    def inv(self):
        """群逆"""
        # TODO: 学生填充——利用 R^T, -R^T v, -R^T p
        pass

    @staticmethod
    def exp_map(xi):
        """李代数 → 群：xi ∈ R^9"""
        # TODO: 学生实现 SE_2(3) 的指数映射
        pass

def group_affine_residual(f_u, X1, X2):
    """计算 group-affine 残差 R(X1,X2) = f(X1@X2) - f(X1)@X2 - X1@f(X2) + X1@f(I)@X2"""
    I = np.eye(5)
    lhs = f_u(X1.mat @ X2.mat)
    rhs = f_u(X1.mat) @ X2.mat + X1.mat @ f_u(X2.mat) - X1.mat @ f_u(I) @ X2.mat
    return np.linalg.norm(lhs - rhs, 'fro')
```

---

### 延伸阅读

| 资源 | 难度 | 内容 | 建议阅读方式 |
|------|------|------|------------|
| Barrau-Bonnabel TAC 2017 (arXiv:1410.1465v4) | ⭐⭐⭐⭐ | 原论文，核心定理 + 证明 | 精读 §II + 附录 B/C |
| Barrau PhD 2015 (HAL: tel-01344622) | ⭐⭐⭐⭐ | 最详细证明版本 | Ch.3-4，BCH 展开细节 |
| Hartley et al. IJRR 2020 | ⭐⭐⭐ | Cassie 实验 + 接触辅助 InEKF | §III-IV 实现细节 |
| Bonnabel-Martin-Rouchon TAC 2008 | ⭐⭐⭐ | 不变观测器历史基础 | §II-III 对称性框架 |
| Bonnabel CDC 2007 | ⭐⭐ | SO(3) 左不变 EKF 最早版本 | 短文，快速阅读 |
| Mahony-van Goor TAC 2022 | ⭐⭐⭐⭐ | Equivariant Filter 推广 | Thm 1-3 的证明框架 |
| van Goor-Mahony T-RO 2023 (EqVIO) | ⭐⭐⭐ | EqF 在 VIO 上的工程实现 | 与 InEKF 的对比实验 |
| Chauchat-Barrau-Bonnabel 2022 | ⭐⭐⭐⭐ | Two-Frames Group 处理 bias | Tangent group 构造 |
| Deyst-Price TAC 1968 | ⭐⭐⭐⭐ | LTV-KF 稳定性原始论文 | Thm 1-2（被 Barrau 引用） |
| Reif-Unbehauen TAC 1999 | ⭐⭐⭐⭐ | EKF 稳定性经典论文 | 对比 InEKF 证明的差异 |

---

### 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| 传播段 Jacobian 依赖估计值 | 系统不满足 group-affine 或观测手性选错 | 1.逐块验证 group-affine 残差 2.检查 $Y=\chi d$ vs $Y=\chi^{-1}d$ | §D.3, §D.26 |
| 加入 bias 后 NEES 发散 | bias 破坏了 group-affine 完整性 | 1.用 imperfect InEKF（忽略 bias 的 group-affine 影响） 2.增大 Q_bias 3.考虑 EqF | §D.17 |
| 旋转误差接近 $\pi$ 时滤波器跳变 | $\log$ 映射在 $\pi$ 处多值/不连续 | 1.检查四元数表示是否跨越对径 2.加入抗包裹逻辑 3.降低初始化误差 | §D.8 注记 |
| 线性误差传播与非线性不一致 | BCH 收敛域外或积分步长过大 | 1.运行 log-linear 测试（§D.22 Test 3） 2.缩小时间步长 3.检查 $\|\xi\|$ 是否在单射半径内 | §D.9, §D.22 |
| 不可观方向上协方差不收敛 | 全局平移/yaw 方向无绝对观测 | 1.用数值零空间测试（§D.22 Test 4） 2.加入 GPS/磁力计约束 3.设置不可观子空间初始 P 为大值 | §D.16 |

---

### 跨章综合练习 ⭐⭐⭐

**题目**：综合 5-A3（InEKF 工程实现）+ 5-D（本章定理精读）+ 5-B（因子图优化）的知识：

1. **InEKF vs 因子图的统一视角**：对一个 $SE_2(3)$ IMU + GPS landmark 系统，写出 InEKF 的传播/更新方程（来自 5-A3），再写出等价的因子图表示（来自 5-B）。证明：当 InEKF 只保留最新状态时，它等价于 iSAM2 Bayes 树退化为单节点链的情况。
2. **group-affine 与 iSAM2 的互补性**：解释为什么 InEKF 在传播段提供精确 Riccati 但只保留当前状态估计，而 iSAM2 保留全轨迹但传播段可能有线性化误差。设计一个混合方案：InEKF 做高频传播（400 Hz），iSAM2 做低频全轨迹平滑（10 Hz 关键帧），两者通过什么接口连接？
3. **Theorem 2 的数值验证**：在 Python 中实现 $SE_2(3)$ 上的 IMU 动力学，随机生成初始误差 $\xi_0$，分别用（a）非线性误差 ODE 数值积分和（b）$\exp(\int_0^t A_\tau d\tau \cdot \xi_0)$ 的线性传播，验证两者一致到机器精度。讨论：当 $\|\xi_0\|$ 增大到多少时，两者开始偏离？这与 $\log$ 映射的单射半径有什么关系？

### 术语对照表

| 英文术语 | 中文 | 首次出现节 |
|---------|------|-----------|
| Group-Affine System | 群仿射系统 | §D.3 |
| Autonomous Error Equation | 自治误差方程 | §D.9 |
| Log-Linearity Property | 对数线性性质 | §D.9 |
| Left/Right-Invariant Error | 左/右不变误差 | §D.5 |
| Two-Frames Group | 双参考系群 | §D.17 |
| Equivariant Filter (EqF) | 等变滤波器 | §D.19 |
| Imperfect InEKF | 不完美不变 EKF | §D.17 |
| Injectivity Radius | 单射半径 | §D.22 |
| Observability Rank Condition | 可观性秩条件 | §D.16 |
| Canonical Connection | 典范联络 | §D.19 |

### 本章核心定理速查

| 定理 | 核心结论 | 工程意义 |
|------|---------|---------|
| Theorem 1 (Group-Affine) | $f(\chi a, \chi b) = \chi f(a,b) + (I-\chi)d$ | 保证误差传播与估计值无关 |
| Theorem 2 (Log-Linearity) | 误差 ODE 在 Lie 代数上精确线性 | EKF 传播段无线性化近似误差 |
| Theorem 3 (Convergence) | 满足 group-affine + 可观 → 指数稳定 | InEKF 的理论收敛保证 |

---
