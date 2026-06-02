## 博士前数学路线图 · 第零层 · 任务 B2：测度论与 Lebesgue 积分

> **本任务在路线图中的位置**：B2 是第零层实分析支柱的第二块基石（B1 实分析基础 → **B2 测度论与 Lebesgue 积分** → B3 泛函分析），也是向第一层概率论（C1）与流形积分（Layer-1 李群/黎曼几何）过渡的必经通道。建议在掌握 B1 中 ε-δ 分析、一致收敛、紧性、Baire 纲定理之后进入本任务。

---

### 引言：为什么机器人学博士生必须严肃学测度论

机器人学的数学语言到了 2020 年代已经几乎完全被测度论重写。一个在 $\mathrm{SE}(3)$ 上滑行的无人机、一个在稀疏地图上闭环的 SLAM 前端、一个用 MPPI 做模型预测控制的四足、一个用 actor–critic 学习操作技能的机械臂——它们共同的数学骨骼是 **概率测度 $P$ 在可测空间 $(\Omega,\mathcal{F})$ 上的演化**，而非中学式的\"概率密度 $p(x)$\"。密度只是相对 Lebesgue 测度的 **Radon–Nikodym 导数** $p=\dfrac{dP}{d\lambda}$，一旦遇到离散数据关联 + 连续位姿的混合状态、或在 $\mathrm{SO}(3)$ 这种非欧流形上的姿态，没有 Lebesgue 测度做参照物，密度连定义都谈不上。

具体到四个核心场景：(i) **SLAM 后验** $\pi_t(A)=\mathbb{P}(X_{0:t}\in A\mid Z_{1:t})$ 是 $(\mathrm{SE}(3)^{t+1},\mathcal{B})$ 上的概率测度，贝叶斯更新 $\dfrac{d\pi_t}{d\pi_{t-1}}\propto p(z_t\mid x_t)$ 本质是 R–N 导数的递推；(ii) **Kalman 滤波** 是 $L^2(\Omega,\mathcal{F},\mathbb{P})$ 中向观测子空间 $H_t=\overline{\operatorname{span}}\{1,y_1,\dots,y_t\}$ 的正交投影 $\hat x_{t|t}=\Pi_{H_t} x_t$，投影的存在性依赖 $L^2$ 的**完备性**（Riesz–Fischer 定理），而完备性在 Riemann 积分下根本不成立；(iii) **粒子滤波** 的经验测度 $\mu_t^N=\tfrac1N\sum_i\delta_{x_t^{(i)}}$ 弱收敛到 $\pi_t$，收敛速度 $\|\mu_t^N(\varphi)-\pi_t(\varphi)\|_{L^2}\le C_t\|\varphi\|_\infty/\sqrt N$ 的证明核心工具就是**控制收敛定理（DCT）**；(iv) **采样运动规划** 的 RRT* 渐近最优性定理依赖\"正 Lebesgue 测度的管道几乎必然被均匀采样命中\"这一 Borel–Cantelli 论证。

测度论是这些语言的统一底座。如果 B2 学得松散，博士生会发现：读不懂 Del Moral 的 *Feynman–Kac Formulae*、看不清 Crisan–Doucet 证明中 DCT 在哪一步起作用、推不出 Munos–Szepesvári 的拟合价值迭代误差界，也没法把 Chirikjian 的 $\mathrm{SE}(3)$ 上 Gauss 分布相对 Haar 测度的密度想清楚。因此本任务不是\"数学素养装饰\"，而是**未来五年所有概率、滤波、控制、学习论文的前置库函数**。

本任务建议学时：**8–12 周**，主教材 Folland 2e（Ch 1–3, 6, 7）配 Tao *Introduction to Measure Theory* 做动机驱动，参考 Cohn 2e 做概率论过渡与 Haar 测度，参考 Rudin RC 看 von Neumann 证明的优雅。本大纲分 17 节，每节标注🔵核心或🟣进阶，并用⚡推导密集或📖概念铺垫标签。

---

### §B2.1 从 Riemann 到 Lebesgue 的动机 🔵核心 📖概念铺垫

**来源领域**：实分析史 · 积分理论的危机与重建  
**前置依赖**：B1 中 Riemann 积分、一致收敛、Cantor 集  
**主参考**：Tao §1.1；Stein–Shakarchi Ch 1 §1；Royden–Fitzpatrick Ch 2 开篇  
**黑盒标注**：选择公理（用于 §B2.5 的 Vitali 不可测集，此处可暂时当作给定）

**Riemann 可积的 Lebesgue 判据**给出 Riemann 积分失败的病理学诊断：有界函数 $f:[a,b]\to\mathbb{R}$ 可 Riemann 积分当且仅当其不连续点集为 **Lebesgue 零测集**。这句话本身就用了 Lebesgue 测度——说明\"什么使 Riemann 失败\"需要新工具来表达。

**三个经典反例驱动整个理论**：(1) **Dirichlet 函数** $\mathbf{1}_\mathbb{Q}(x)$ 处处不连续，不 Riemann 可积，但 Lebesgue 积分 $\int_{[0,1]}\mathbf{1}_\mathbb{Q}\,d\lambda=0$ 因 $\mathbb{Q}\cap[0,1]$ 是 Lebesgue 零测；(2) **Cantor 函数**（魔鬼楼梯）连续单调递增、$c'=0$ 几乎处处、$c(1)-c(0)=1$，暴露出 Riemann 版 FTC 与\"连续+有界变差\"脱节；(3) **Cantor 集特征函数** $\mathbf{1}_C$ 是 Riemann 可积的（零测），但它的一个\"肥胖 Cantor 集\"（Smith–Volterra–Cantor，测度 $1/2$）变体 $\mathbf{1}_{C_+}$ 就 Riemann 不可积——小幅调整就破坏可积性，理论极其脆弱。

**极限交换的失败**：Riemann 下不存在\"只要 $f_n\to f$ 逐点且都可积就 $\int f_n\to\int f$\"这样的定理。反例：用 $\mathbb{Q}\cap[0,1]$ 的枚举 $\{q_k\}$ 定义 $f_n(x)=\mathbf{1}_{\{q_1,\dots,q_n\}}(x)$，则 $f_n$ 处处 Riemann 可积且 $\int f_n=0$，但 $f_n\uparrow \mathbf{1}_\mathbb{Q}$，极限函数竟然不 Riemann 可积。这一现象正是 Lebesgue 在 1902 博士论文中用\"水平切片\"替代 Riemann 的\"垂直切片\"的动机：**按值域分层**先收集 $\{f\in[k/n,(k+1)/n)\}$ 的测度再求和，对\"定义域上多么病理的函数\"都鲁棒，代价是需要先把\"什么叫集合的测度\"严格化——这就是下文 §B2.2–§B2.5 的任务。

**机器人学启示**：连续状态空间上观测似然 $p(z\mid x)$ 可能在障碍边界出现跳跃、在遮挡阴影出现间断，Riemann 框架下 $\int p(z\mid x)\,dP(x)$ 可能根本没定义；但 Lebesgue 框架下只要 $p$ 可测就能积分，这是粒子滤波工作在复杂机器人场景的根本保证。

---

### §B2.2 σ-代数与可测空间 🔵核心 📖概念铺垫

**来源领域**：集合论 · 测度论公理基础  
**前置依赖**：B1 集合运算、B1.5 开集/闭集  
**主参考**：Folland §1.2；Cohn §1.1, §1.6；Tao §1.4  
**黑盒标注**：无

**σ-代数** $\mathcal{M}\subseteq 2^X$ 是满足 (i) $X\in\mathcal{M}$ (ii) 补封闭 (iii) **可数**并封闭的集族。可数并比有限并强，正是让\"极限事件\"可度量的关键。生成 σ-代数 $\sigma(\mathcal{E})=\bigcap\{\mathcal{M}\supseteq\mathcal{E}:\mathcal{M}\text{ 为 σ-代数}\}$ 给出包含 $\mathcal{E}$ 的最小 σ-代数；特别地，**Borel σ-代数** $\mathcal{B}(X):=\sigma(\tau_X)$（拓扑空间开集生成）是一切连续函数、一切开/闭集可测的最小共同框架。

**π-λ 定理（Dynkin 系统定理）** 是测度论唯一性证明的瑞士军刀。设 $\mathcal{P}$ 为 $X$ 上的 π-系（对有限交封闭），$\mathcal{L}$ 为 λ-系（含 $X$、差封闭、可数递增极限封闭），若 $\mathcal{P}\subseteq\mathcal{L}$ 则 $\sigma(\mathcal{P})\subseteq\mathcal{L}$。应用模板：要证两测度 $\mu=\nu$ 于 $\sigma(\mathcal{P})$，只需证其在 π-系 $\mathcal{P}$ 上相等并验证 $\{\mu=\nu\}$ 是 λ-系。此定理避开了直接描述 $\sigma(\mathcal{P})$ 的困难（后者通常无法显式枚举）。

**机器人应用**：**信息 σ-代数** $\mathcal{F}_t:=\sigma(z_{1:t},u_{1:t})$ 精确编码\"t 时刻已知信息\"。一个估计器 $\hat x_t$ 被称为**因果的 (causal)** 当且仅当它 $\mathcal{F}_t$-可测——换言之，它只依赖 $\mathcal{F}_t$ 下可区分的信息。滤波被严格定义为条件期望 $\hat x_{t|t}=\mathbb{E}[X_t\mid\mathcal{F}_t]$，这个定义本身就要求 $\mathcal{F}_t$ 是 σ-代数而不仅是代数——否则条件期望的 Radon–Nikodym 构造（§B2.12）不成立。**σ-代数链** $\mathcal{F}_1\subseteq\mathcal{F}_2\subseteq\cdots$ 构成\"信息过滤\"（filtration），这是鞅论与随机最优控制的载体。

---

### §B2.3 测度的定义与基本性质 🔵核心 ⚡推导密集

**来源领域**：测度论公理化  
**前置依赖**：§B2.2  
**主参考**：Folland §1.3；Cohn §1.2；Rudin §1.18  
**黑盒标注**：无

**测度** $\mu:\mathcal{M}\to[0,\infty]$ 满足 $\mu(\varnothing)=0$ 与**可数可加性**：对两两不交的 $\{A_j\}\subseteq\mathcal{M}$，$\mu(\bigcup A_j)=\sum\mu(A_j)$。**有限测度** $\mu(X)<\infty$；**σ-有限** $X=\bigcup X_n$，$\mu(X_n)<\infty$；**概率测度** $\mu(X)=1$。

**基本性质**都从可数可加性推出：(i) 单调性 $A\subseteq B\Rightarrow\mu(A)\le\mu(B)$；(ii) 可数次可加性 $\mu(\bigcup A_j)\le\sum\mu(A_j)$；(iii) **由下连续性** $A_n\uparrow A\Rightarrow\mu(A_n)\uparrow\mu(A)$（总成立）；(iv) **由上连续性** $A_n\downarrow A$ 且 $\mu(A_1)<\infty\Rightarrow\mu(A_n)\downarrow\mu(A)$（**有限性不可省**，反例 $A_n=[n,\infty)$）。

**Borel–Cantelli 引理**：若 $\sum\mu(A_n)<\infty$，则 $\mu(\limsup A_n)=\mu(\bigcap_k\bigcup_{n\ge k}A_n)=0$。证明骨架：$\mu(\bigcup_{n\ge k}A_n)\le\sum_{n\ge k}\mu(A_n)\to 0$，对 $k$ 取上连续性极限。此引理在随机逼近（§B2.6 机器人应用）中用于证\"坏事件发生次数 a.s. 有限\"。

**测度完备化**：给定 $(X,\mathcal{M},\mu)$，令 $\bar{\mathcal{M}}=\{A\cup N:A\in\mathcal{M}, N\subseteq M\in\mathcal{M}, \mu(M)=0\}$，定义 $\bar\mu(A\cup N)=\mu(A)$。则 $(\bar{\mathcal{M}},\bar\mu)$ 完备（零测集的一切子集可测）。Lebesgue 测度即 Borel 测度的完备化。

**机器人应用**：概率测度 $P$ 是测度论的特例——Kolmogorov 1933 公理化把概率论还原为\"归一化测度论\"。所有概率公理都是测度公理的特化：$P(A\cup B)=P(A)+P(B)$（不交）来自可数可加性，$P(A_n\uparrow A)\Rightarrow P(A_n)\uparrow P(A)$ 保证了\"观测到越多数据，事件发生概率的估计稳定\"。**σ-有限性**的要求也自然——虽然 $P$ 概率测度必然是 σ-有限（取 $X$ 自身），但像 Lebesgue 测度 $\lambda$ 作为参考测度时必须用 $\mathbb{R}^n=\bigcup_n[-n,n]^n$ 的分解。

---

### §B2.4 外测度与 Carathéodory 扩张定理 🔵核心 ⚡推导密集

**来源领域**：测度构造论  
**前置依赖**：§B2.2, §B2.3  
**主参考**：Folland §1.4；Cohn §1.3；Halmos §II.10–§III.13；Tao §1.7  
**黑盒标注**：无

**外测度** $\mu^*:2^X\to[0,\infty]$ 三公理：$\mu^*(\varnothing)=0$、单调、可数次可加。外测度不一定可加，但可用 **Carathéodory 条件** 筛选\"好集合\"：
$$
A\text{ 为 }\mu^*\text{-可测} \;\Longleftrightarrow\; \mu^*(E)=\mu^*(E\cap A)+\mu^*(E\cap A^c)\;\forall E\subseteq X.
$$

**Carathéodory 定理**：$\mu^*$-可测集全体 $\mathcal{M}^*$ 是 σ-代数，$\mu^*|_{\mathcal{M}^*}$ 是完备测度。

**完整证明骨架**：
1. **外测度基本性质** 由定义直接。
2. **$\mathcal{M}^*$ 为代数**：$\varnothing\in\mathcal{M}^*$、补封闭显然；有限并封闭通过两次 Carathéodory 分裂（对 $E\cap(A\cup B)$）得到。
3. **可数并与可数可加性**：对不交 $\{A_j\}\subseteq\mathcal{M}^*$，令 $B_n=\bigcup_{j\le n}A_j$。用测试集 $E\cap B_n$ 归纳证 $\mu^*(E\cap B_n)=\sum_{j\le n}\mu^*(E\cap A_j)$；取 $n\to\infty$ 结合次可加性得不交并的可数可加性与 $\bigcup A_j\in\mathcal{M}^*$。
4. **完备性**：$\mu^*(N)=0\Rightarrow\mu^*(E\cap N)=0$ 对一切 $E$，故 $N\in\mathcal{M}^*$。

**Hahn–Kolmogorov 扩张定理**：代数 $\mathcal{A}$ 上的预测度 $\mu_0$（即 $\mathcal{A}$ 上可数可加）通过
$$\mu^*(E)=\inf\Big\{\sum_j\mu_0(A_j):A_j\in\mathcal{A},E\subseteq\bigcup A_j\Big\}$$
生成外测度，Carathéodory 得到 σ-代数 $\mathcal{M}^*\supseteq\sigma(\mathcal{A})$ 与扩张 $\mu=\mu^*|_{\mathcal{M}^*}$。**σ-有限性下扩张唯一**（π-λ 定理证明）。

**陷阱**：无 σ-有限性唯一性失效。反例：$X=\mathbb{Q}\cap[0,1]$ 上的半开区间代数，$\mu_0($非空$)=\infty$，此预测度可扩张为多个不同测度。

**机器人应用**：Carathéodory 扩张是 Lebesgue 测度与一切乘积测度（SLAM 联合分布、马尔可夫链转移核在 $\prod_t\mathrm{SE}(3)$ 上的测度）存在性的唯一构造机制。Kolmogorov 扩张定理——构造无限时间轴随机过程测度——是 Carathéodory 在乘积空间的直接推广。

---

### §B2.5 Lebesgue 测度的构造与性质 🔵核心 ⚡推导密集

**来源领域**：经典实分析  
**前置依赖**：§B2.4  
**主参考**：Folland §1.5；Stein–Shakarchi Ch 1 §3；Royden–Fitzpatrick Ch 2；Tao §1.2  
**黑盒标注**：选择公理（用于 Vitali 集）

从半开区间 $(a,b]\subseteq\mathbb{R}$ 的长度 $\ell((a,b])=b-a$ 出发，推广到有限不交并的代数 $\mathcal{A}_0$ 上的预测度 $m_0$；经 Carathéodory 扩张得**Lebesgue 测度** $m$ 于 $\mathcal{L}\supseteq\mathcal{B}(\mathbb{R})$。

**关键性质**：
1. **正则性**：每 $E\in\mathcal{L}$ 满足 $m(E)=\inf\{m(U):U\supseteq E\text{ 开}\}=\sup\{m(K):K\subseteq E\text{ 紧}\}$（外正则 + 内正则）。
2. **平移不变** $m(E+x)=m(E)$ 且 **$m$ 是 $\mathbb{R}^n$ 上唯一满足正则性 + 平移不变 + $m([0,1]^n)=1$ 的测度**（Haar 测度唯一性的原型）。
3. **线性变换**：$m(AE)=|\det A|\,m(E)$ 对 $A\in\mathrm{GL}(n)$。这一公式在李群上推广为 Haar 测度的模函数（§B2.16 机器人应用）。
4. **Vitali 不可测集**（需选择公理）：在 $[0,1]$ 上等价关系 $x\sim y\Leftrightarrow x-y\in\mathbb{Q}$，从每等价类选一代表构成 $V$；则 $V\notin\mathcal{L}$，因平移 $\{V+q\}_{q\in\mathbb{Q}\cap[-1,1]}$ 不交、覆盖 $[0,1]$、若可测则和为 $\sum m(V)\in\{0,\infty\}$ 都与 $1\le m(\cdots)\le 3$ 矛盾。
5. **Cantor 集** $C\subseteq[0,1]$ 是不可数紧完集但 $m(C)=0$；**Cantor–Lebesgue 函数** $c:[0,1]\to[0,1]$ 连续、单调递增、几乎处处 $c'=0$，却 $c(1)-c(0)=1$——这是 §B2.15 FTC 的最核心反例。
6. **Borel ⊊ Lebesgue**：$|\mathcal{B}(\mathbb{R})|=\mathfrak{c}$ 而 $|\mathcal{L}|=2^{\mathfrak{c}}$，Cantor 集的子集给出非 Borel 的 Lebesgue 可测集。

**机器人应用**：**概率密度** $p:\mathbb{R}^n\to[0,\infty)$ 的数学本质是 R–N 导数 $p=\dfrac{dP}{dm}$。Gauss 分布 $\mathcal{N}(\mu,\Sigma)$（$\Sigma\succ 0$）相对 $m$ 绝对连续，其密度即熟知的
$$p(x)=(2\pi)^{-n/2}(\det\Sigma)^{-1/2}\exp\bigl(-\tfrac12(x-\mu)^\top\Sigma^{-1}(x-\mu)\bigr).$$
一旦 $\Sigma$ 奇异（退化 Gauss，例如约束后的位姿分布），密度不再存在——必须回到测度层面处理。

---

### §B2.6 可测函数 🔵核心 📖概念铺垫

**来源领域**：测度论基础  
**前置依赖**：§B2.2, §B2.5  
**主参考**：Folland §2.1；Rudin §1.3–§1.14；Cohn §2.1  
**黑盒标注**：无

**可测函数**：$f:(X,\mathcal{M})\to(Y,\mathcal{N})$ 可测当且仅当 $f^{-1}(\mathcal{N})\subseteq\mathcal{M}$。对 $\mathbb{R}$-值函数等价于 $\{f>a\}\in\mathcal{M}$ 对一切 $a\in\mathbb{R}$。

**封闭性**：可测函数在 $(+,-,\times,\div)$、$\sup_n f_n$、$\inf_n f_n$、$\limsup f_n$、$\liminf f_n$、**逐点极限**下封闭；连续函数复合可测函数仍可测；两个可测函数的极限 a.e. 存在处可测。

**简单函数逼近定理**：对非负可测 $f:X\to[0,\infty]$，存在简单函数列 $0\le\varphi_n\nearrow f$ 逐点。构造：$\varphi_n(x)=k/2^n$ 若 $k/2^n\le f(x)<(k+1)/2^n$，$k<n\cdot 2^n$；$\varphi_n(x)=n$ 若 $f(x)\ge n$。这是 Lebesgue 积分\"三阶段定义\"的脚手架。

**Littlewood 三原则** 给出直观总结：每可测集近似于开集 + 闭集；每可测函数近似于连续函数；每点态收敛近似于一致收敛。精确化为三定理：

**Egorov 定理**（$\mu(X)<\infty$）：$f_n\to f$ a.e. $\Rightarrow$ 对任 $\varepsilon>0$ 存在 $E_\varepsilon$，$\mu(E_\varepsilon^c)<\varepsilon$ 且 $f_n\to f$ 在 $E_\varepsilon$ 上一致。证明骨架：令 $E_{n,k}=\bigcup_{m\ge n}\{|f_m-f|>1/k\}$，由 $f_n\to f$ a.e. 得 $\mu(E_{n,k})\downarrow 0$；选 $n_k$ 使 $\mu(E_{n_k,k})<\varepsilon/2^k$，取并的补即得。**有限测度不可省**（反例 $f_n=\mathbf{1}_{[n,n+1]}$）。

**Lusin 定理**：$f:\mathbb{R}\to\mathbb{R}$ Lebesgue 可测、有限 a.e. $\Rightarrow$ 对任 $\varepsilon>0$ 存在闭集 $F$，$m(F^c)<\varepsilon$ 且 $f|_F$ 连续。

**机器人应用**：SLAM 似然模型 $p(z\mid x)$ 典型是分段光滑（遮挡边界有跳跃），在 Lebesgue 意义下可测但不连续；Lusin 定理保证\"除 ε 测度的集合外\"可视为连续——这正是 EKF 线性化、UKF σ 点抽样、粒子重采样等\"局部近似\"的理论基础。

---

### §B2.7 Lebesgue 积分的构造 🔵核心 ⚡推导密集

**来源领域**：积分论  
**前置依赖**：§B2.3, §B2.6  
**主参考**：Folland §2.2–§2.3；Rudin §1.23–§1.33；Cohn §2.2–§2.3  
**黑盒标注**：无

**三阶段构造**：
1. **非负简单函数** $\varphi=\sum_{i=1}^n c_i\mathbf{1}_{A_i}$（标准型，$A_i$ 不交）：$\int\varphi\,d\mu:=\sum_i c_i\mu(A_i)$，$0\cdot\infty:=0$ 约定。
2. **非负可测函数** $f\ge 0$：$\int f\,d\mu:=\sup\{\int\varphi\,d\mu:\varphi\text{ 简单},0\le\varphi\le f\}$。
3. **一般可测函数** $f=f^+-f^-$：若 $\int f^+$ 与 $\int f^-$ 不同时 $\infty$ 则 $\int f:=\int f^+-\int f^-$；若 $\int|f|<\infty$ 称 $f\in L^1(\mu)$。

**基本性质**：线性（非负函数需单独证，借 MCT）、单调性、**三角不等式** $|\int f|\le\int|f|$、零测集上积分为零、$\int f=\int g$ 若 $f=g$ a.e.。

**Layer-cake 表示**（对非负 $f$）：
$$
\int f\,d\mu=\int_0^\infty\mu(\{f>t\})\,dt=\int_0^\infty\mu(\{f\ge t\})\,dt.
$$
证明骨架：用 Fubini 在 $X\times[0,\infty)$ 上对 $\mathbf{1}_{\{(x,t):t<f(x)\}}$ 做累次积分。此公式在证明 $L^p$-范数的替代刻画与 Markov 不等式时极有用。

**可积性刻画**：$f\in L^1\iff|f|\in L^1\iff\int|f|\,d\mu<\infty$。在 $(X,\mathcal{M},\mu)$ 上 $L^1(\mu)$ 是向量空间，$\|f\|_1:=\int|f|\,d\mu$ 是一个模去\"a.e. 零\"等价关系后的真范数。

**机器人应用**：期望 $\mathbb{E}[X]=\int_\Omega X\,dP$ 正是测度 $P$ 下的 Lebesgue 积分；当 $X$ 有密度 $p_X=dP_X/dm$ 时退化为熟悉的 $\int x\,p_X(x)\,dx$。Markov 不等式 $P(|X|\ge t)\le\mathbb{E}|X|/t$ 即 Layer-cake 的立即推论，支撑粒子滤波的集中不等式。

---

### §B2.8 三大收敛定理 🔵核心 ⚡推导密集

**来源领域**：积分论核心  
**前置依赖**：§B2.7  
**主参考**：Folland §2.3；Rudin §1.26–§1.34；Cohn §2.4  
**黑盒标注**：无

**单调收敛定理（MCT / Beppo Levi）**：$f_n\ge 0$ 可测、$f_n\nearrow f$ a.e. $\Rightarrow\int f_n\,d\mu\nearrow\int f\,d\mu$。

证明骨架：$\int f_n\le\int f$ 得上界。下界用**α-技巧**：固定 $\alpha\in(0,1)$ 与简单 $\varphi\le f$，令 $E_n=\{f_n\ge\alpha\varphi\}\nearrow X$；由测度下连续 $\int f_n\ge\alpha\int_{E_n}\varphi\to\alpha\int\varphi$。取 $\alpha\uparrow 1$ 再对 $\varphi$ 取 sup 得 $\lim\int f_n\ge\int f$。

非负性不可省：$f_n=-\tfrac1n\mathbf{1}_{[0,n]}\nearrow 0$ 但 $\int f_n=-1\not\to 0$。

**Fatou 引理**：$f_n\ge 0$ 可测 $\Rightarrow\int\liminf f_n\,d\mu\le\liminf\int f_n\,d\mu$。

证明骨架：$g_k=\inf_{n\ge k}f_n\nearrow\liminf f_n$，$g_k\le f_n\forall n\ge k$，故 $\int g_k\le\inf_{n\ge k}\int f_n$；对 $g_k$ 用 MCT 得 $\int\liminf f_n=\lim\int g_k\le\liminf\int f_n$。

严格不等常见：**行进帽子** $f_n=\mathbf{1}_{[n,n+1]}$，$\liminf f_n=0$ 而 $\int f_n=1$。

**控制收敛定理（DCT / Lebesgue）**：$f_n\to f$ a.e.、$|f_n|\le g\in L^1$ $\Rightarrow$ $f\in L^1$，$\int f_n\to\int f$，$\int|f_n-f|\to 0$。

证明骨架：对 $g+f_n\ge 0$ 用 Fatou 得 $\int f\le\liminf\int f_n$；对 $g-f_n\ge 0$ 用 Fatou 得 $\limsup\int f_n\le\int f$。合并。$L^1$ 收敛：对 $2g-|f_n-f|\ge 0$ 用 Fatou。

**控制函数必要性**的经典反例：
- 行进帽子 $f_n=\mathbf{1}_{[n,n+1]}\to 0$ 逐点，$\int f_n=1$，找不到 $L^1$ 控制（$g\ge 1$ 于 $[0,\infty)$ 使 $\int g=\infty$）。
- 高瘦帽子 $f_n=n\mathbf{1}_{(0,1/n]}\to 0$ 逐点，$\int f_n=1$，最优 $g(x)=\tfrac1x\notin L^1(0,1)$。

**推广**：
- **Scheffé 引理**：$f_n,f\ge 0$、$f_n\to f$ a.e.、$\int f_n\to\int f<\infty\Rightarrow\int|f_n-f|\to 0$。
- **Pratt 引理（广义 DCT）**：$|f_n|\le g_n$、$g_n\to g$ a.e.、$\int g_n\to\int g<\infty\Rightarrow\int f_n\to\int f$。

**机器人应用**：DCT 是**粒子滤波收敛性证明的核心工具**。经验测度 $\mu_t^N=\tfrac1N\sum\delta_{x_t^{(i)}}$，对有界可测 $\varphi$ 证 $\mu_t^N(\varphi)\to\pi_t(\varphi)$ a.s.：由强大数定律逐点收敛，用 $|\varphi|\le\|\varphi\|_\infty$ 做控制函数，DCT 给出积分收敛。Del Moral 的 Feynman–Kac 框架下 $L^2$ 收敛率 $\mathbb{E}|\mu_t^N(\varphi)-\pi_t(\varphi)|^2\le C_t\|\varphi\|_\infty^2/N$ 的逐步证明中，每一个似然归一化步骤都用 DCT 交换极限与积分。类似地，**随机梯度下降**的几乎必然收敛中\"残差趋 0\"这一步也依赖 Scheffé 引理处理 $L^1$ 损失。

---

### §B2.9 Riemann 积分与 Lebesgue 积分的比较 🔵核心 📖概念铺垫

**来源领域**：积分理论对比  
**前置依赖**：§B2.7, §B2.8  
**主参考**：Folland §2.7；Cohn §2.5；Bartle §16

**Lebesgue 可积判据（Riemann 版）**：有界 $f:[a,b]\to\mathbb{R}$ 为 Riemann 可积当且仅当其不连续点集为 Lebesgue 零测集。证明思路：上下 Darboux 和之差等于振荡函数的上积分，有限 iff $\{f\text{ 不连续}\}$ 可被有限开区间列以任意小总长覆盖。

**一致性定理**：若 $f:[a,b]\to\mathbb{R}$ Riemann 可积，则 $f$ 也 Lebesgue 可积且两个积分值相等。这一致性保证了所有经典微积分计算无须重做。

**Lebesgue 优势**：(i) 更强收敛定理（MCT/DCT）Riemann 下根本不成立；(ii) **完备的 $L^p$ 空间**——Riemann 可积函数在 $\|\cdot\|_p$ 下不完备，Cauchy 列可能收敛到非 Riemann 可积的极限；(iii) 乘积空间与 Fubini 定理的简洁形式；(iv) 无界区间与无界函数的自然处理（无须**广义** Riemann 积分的权宜）。

**反向不等式**：Lebesgue 可积 $\not\Rightarrow$ Riemann 可积。Dirichlet 函数是经典反例。条件收敛积分如 $\int_0^\infty\tfrac{\sin x}{x}\,dx$ 作为广义 Riemann 积分存在但**非 Lebesgue 可积**（因 $\int|\sin x/x|\,dx=\infty$）——此类\"震荡式\"积分需 **Henstock–Kurzweil 积分** 或在复分析中作为反常积分处理。

---

### §B2.10 积测度与 Fubini–Tonelli 定理 🔵核心 ⚡推导密集

**来源领域**：多变量积分  
**前置依赖**：§B2.4, §B2.8  
**主参考**：Folland §2.5；Rudin Ch 8；Cohn Ch 5；Stein–Shakarchi Ch 2 §3  
**黑盒标注**：单调类定理（§B2.2 的派生工具）

**积 σ-代数** $\mathcal{A}\otimes\mathcal{B}:=\sigma(\{A\times B:A\in\mathcal{A},B\in\mathcal{B}\})$。**积测度** $\mu\times\nu$ 通过在可测矩形代数上定义 $(\mu\times\nu)(A\times B)=\mu(A)\nu(B)$ 并由 Carathéodory 扩张得到，σ-有限下唯一。

**单调类定理**：若 $\mathcal{A}_0$ 为代数，则 $\mathcal{M}(\mathcal{A}_0)=\sigma(\mathcal{A}_0)$，其中 $\mathcal{M}$ 为单调类包含 $\mathcal{A}_0$。这是证\"所有可测集满足某性质\"的标准归纳工具。

**Tonelli 定理**（非负可测，σ-有限）：$f:X\times Y\to[0,\infty]$ 为 $\mathcal{A}\otimes\mathcal{B}$-可测。则切片 $x\mapsto\int f(x,y)\,d\nu(y)$ 为 $\mathcal{A}$-可测（对称），且
$$
\int_{X\times Y}f\,d(\mu\times\nu)=\int_X\!\!\int_Y\! f(x,y)\,d\nu(y)\,d\mu(x)=\int_Y\!\!\int_X\! f(x,y)\,d\mu(x)\,d\nu(y).
$$

证明骨架：(1) **指示函数** $f=\mathbf{1}_E$：$\mathcal{M}=\{E:\text{两累次等于}(\mu\times\nu)(E)\}$ 含矩形、是单调类 ⇒ $\mathcal{M}=\mathcal{A}\otimes\mathcal{B}$；(2) **简单函数**线性；(3) **非负可测**用简单函数 $\varphi_n\nearrow f$ 与 MCT 三次。

**Fubini 定理**（绝对可积）：$f\in L^1(\mu\times\nu)$ $\Rightarrow$ 切片 $f(x,\cdot)\in L^1(\nu)$ a.e. $x$，累次积分存在且等于重积分。证法：先对 $|f|$ 用 Tonelli 得有限，再对 $f^+,f^-$ 分别用 Tonelli 做差。

**不可交换累次积分反例**：$f(x,y)=\dfrac{x^2-y^2}{(x^2+y^2)^2}$ 在 $(0,1]^2$ 上；$\int_0^1\!\int_0^1 f\,dy\,dx=\pi/4$，$\int_0^1\!\int_0^1 f\,dx\,dy=-\pi/4$；此时 $|f|\notin L^1$，Fubini 前提失效。**σ-有限性反例**：$[0,1]$ 上 Lebesgue × 计数测度，对角线 $\Delta$ 两累次积分分别为 $0$ 与 $1$。

**机器人应用**：**SLAM 地图边际化** $p(x_{0:T}\mid z_{1:T})=\int p(x_{0:T},m\mid z_{1:T})\,dm$ 本质是 Fubini——将联合后验对地图变量做边缘化。**FastSLAM 的 Rao–Blackwellization** $p(x_{0:t},m\mid z_{1:t})=p(x_{0:t}\mid z_{1:t})\,p(m\mid x_{0:t},z_{1:t})$ 的条件期望分解依赖 Fubini 交换积分顺序，从而把高维粒子代价压到低维；**因子图消息传递**的求和–积操作（\"sum-product\"）每一步都是 Fubini 的应用。警示：若不检查 $\int\int|f|<\infty$（例如未归一化位势或重尾似然），交换积分可得到错误结果。

---

### §B2.11 符号测度与 Hahn–Jordan 分解 🔵核心 ⚡推导密集

**来源领域**：测度论深化  
**前置依赖**：§B2.3  
**主参考**：Folland §3.1；Rudin §6.1–§6.5；Cohn §4.1

**符号测度** $\nu:\mathcal{M}\to[-\infty,\infty]$ 可数可加（至多取 $+\infty$ 或 $-\infty$ 之一）。**正/负集**：$A$ 为 $\nu$-正集若 $\nu(E)\ge 0$ 对一切 $E\subseteq A$ 可测；负集对称。

**Hahn 分解定理**：对每符号测度 $\nu$ 存在可测划分 $X=P\cup N$（$P\cap N=\varnothing$），$P$ 为正集、$N$ 为负集；分解在零测集意义下唯一。

证明骨架：设 $\nu$ 不取 $-\infty$。令 $m=\inf\{\nu(E):E\in\mathcal{M}\}\ge-\infty$，取 $E_n$ 使 $\nu(E_n)\to m$。归纳构造\"更负\"集：对每 $E_n$ 中 $\nu$-正子集排除；用 Hahn 的极值构造 $N=\liminf E_n$ 或类似，证 $N$ 为负集、$m=\nu(N)$、$P=N^c$ 为正集。

**Jordan 分解**：$\nu=\nu^+-\nu^-$，其中 $\nu^+(E):=\nu(E\cap P)$、$\nu^-(E):=-\nu(E\cap N)$ 均为正测度且**互奇异** $\nu^+\perp\nu^-$。**全变差** $|\nu|:=\nu^++\nu^-$；$\|\nu\|:=|\nu|(X)$ 给出符号测度空间 $M(X)$ 的范数。

**划分刻画**：$|\nu|(E)=\sup\{\sum_i|\nu(E_i)|:\{E_i\}\text{ 为 }E\text{ 的可测有限划分}\}$，与泛函分析中向量测度全变差定义吻合。

**机器人应用**：奖励塑形 (reward shaping) 中差值奖励 $r'(s,a,s')=r(s,a,s')+\gamma\Phi(s')-\Phi(s)$ 可视为符号测度；Hahn 分解用于界定\"塑形正部/负部\"的效果。更重要的是：符号测度是下节 Radon–Nikodym 定理证明的技术前提。

---

### §B2.12 Radon–Nikodym 定理与 Lebesgue 分解 🔵核心 ⚡推导密集

**来源领域**：测度论最核心定理  
**前置依赖**：§B2.11, §B2.13 部分（Hilbert 证明）  
**主参考**：Rudin §6.10（von Neumann 证）；Folland §3.2；Cohn §4.2；Stein–Shakarchi Ch 6 §4  
**黑盒标注**：$L^2$ 的 Riesz 表示定理（Hilbert 空间版本，将在 §B2.13 展开）

**绝对连续** $\nu\ll\mu$：$\mu(E)=0\Rightarrow\nu(E)=0$。**互奇异** $\nu\perp\mu$：存在 $X=A\cup B$ 使 $\nu(A)=\mu(B)=0$。

**Radon–Nikodym 定理**：$\mu,\nu$ σ-有限，$\nu\ll\mu$ $\Rightarrow$ 存在非负可测 $f:X\to[0,\infty)$，μ-a.e. 唯一，使 $\nu(E)=\int_E f\,d\mu\forall E$；记 $f=\dfrac{d\nu}{d\mu}$。

**Lebesgue 分解定理**：$\mu,\nu$ σ-有限 $\Rightarrow$ 唯一分解 $\nu=\nu_{ac}+\nu_s$，$\nu_{ac}\ll\mu$，$\nu_s\perp\mu$。

**von Neumann 证明**（优雅、短）：
1. σ-有限归约为 $\mu,\nu$ 有限。
2. 令 $\varphi=\mu+\nu$，考虑 $L^2(\varphi)$ 上线性泛函 $\Lambda g=\int g\,d\nu$，$|\Lambda g|\le\nu(X)^{1/2}\|g\|_{L^2(\varphi)}$ 有界。
3. 由 Hilbert 空间 Riesz 表示存在 $h\in L^2(\varphi)$ 使 $\int g\,d\nu=\int gh\,d\varphi=\int gh\,d\mu+\int gh\,d\nu$，即 $\int g(1-h)\,d\nu=\int gh\,d\mu$。
4. 代入 $g=\mathbf{1}_E$ 分析得 $0\le h\le 1$ φ-a.e.；令 $A=\{h<1\}$、$B=\{h=1\}$。
5. 在 $B$ 上：$\int_B(1-h)\,d\nu=0=\int_B h\,d\mu=\mu(B)$，故 $\mu(B)=0$，$\nu_s:=\nu\cdot\mathbf{1}_B\perp\mu$。
6. 在 $A$ 上：迭代代入 $g=\mathbf{1}_E(1+h+\cdots+h^n)$，取极限（MCT）得 $\nu_{ac}(E)=\int_E\dfrac{h}{1-h}\mathbf{1}_A\,d\mu$，即 $f=\dfrac{h}{1-h}\mathbf{1}_A$。

**经典证明**（Folland/Cohn）：用 Hahn 分解 + 可容许函数族 $\mathcal{F}=\{f\ge 0:\int_E f\,d\mu\le\nu(E)\forall E\}$ 上确界。对 $g=\sup\mathcal{F}$ 若残差 $\nu-g\mu\ne 0$，用 Hahn 分解对 $\nu-g\mu-t\mu$ 找正部集 $P$ ($\mu(P)>0$)，则 $g+t\mathbf{1}_P\in\mathcal{F}$ 矛盾极大性。

**链式法则**：$\lambda\ll\nu\ll\mu\Rightarrow\dfrac{d\lambda}{d\mu}=\dfrac{d\lambda}{d\nu}\cdot\dfrac{d\nu}{d\mu}$ a.e.

**σ-有限不可省反例**：$[0,1]$ 上 Lebesgue $\mu$ 与计数测度 $\nu$，$\mu\ll\nu$，但没有 $f$ 满足 $\mu(E)=\sum_{x\in E}f(x)$。

**机器人应用**（本节的机器人密度最高）：
- **贝叶斯更新** $\dfrac{dP(\cdot\mid z)}{dP}(x)=\dfrac{p(z\mid x)}{\int p(z\mid x')\,dP(x')}$ 就是 R–N 导数——先验到后验的密度比。
- **重要性采样权** $w^{(i)}\propto\dfrac{d\pi}{dq}(x^{(i)})$ 是 R–N 导数的样本估计；自归一化重要性采样 $\hat{\mathbb{E}}_\pi[\varphi]=\dfrac{\sum w^{(i)}\varphi(x^{(i)})}{\sum w^{(i)}}$ 的无偏性/一致性证明依赖 R–N 的乘法与链式性质。
- **Girsanov 定理**（受控扩散下的测度变换）给出受控 Wiener 测度 $\mathbb{Q}$ 相对被动 Wiener 测度 $\mathbb{P}$ 的 R–N 导数 $\dfrac{d\mathbb{Q}}{d\mathbb{P}}=\exp(\int_0^T u_s^\top dW_s-\tfrac12\int_0^T|u_s|^2\,ds)$——这是路径积分控制 (PI², MPPI) 的核心。
- **Fisher 信息与 KL 散度** $D(P\|Q)=\int\log\dfrac{dP}{dQ}\,dP$ 只有当 $P\ll Q$ 时有限，定义了信息几何与 TRPO/PPO 的信赖域。

---

### §B2.13 $L^p$ 空间 🔵核心 ⚡推导密集

**来源领域**：泛函分析交界  
**前置依赖**：§B2.7, §B2.8  
**主参考**：Folland Ch 6；Rudin Ch 3；Royden–Fitzpatrick Ch 7；Stein–Shakarchi Vol III Ch 1 §3

**定义**：$L^p(X,\mu)$（$1\le p<\infty$）为 $\|f\|_p:=(\int|f|^p\,d\mu)^{1/p}<\infty$ 的可测函数空间（模 a.e. 零）；$L^\infty:=\{f:\|f\|_\infty:=\operatorname{ess\,sup}|f|<\infty\}$。

**Young 不等式**：$a,b\ge 0$、$\tfrac1p+\tfrac1q=1$（$1<p<\infty$）$\Rightarrow ab\le\tfrac{a^p}{p}+\tfrac{b^q}{q}$。证明：凸性或取 $\log$。

**Hölder 不等式**：$\int|fg|\,d\mu\le\|f\|_p\|g\|_q$。证明：若 $\|f\|_p,\|g\|_q>0$ 有限，对归一化 $\tilde f=f/\|f\|_p$、$\tilde g=g/\|g\|_q$ 用 Young 逐点积分。

**Minkowski 不等式**：$\|f+g\|_p\le\|f\|_p+\|g\|_p$。证明：$|f+g|^p\le|f+g|^{p-1}(|f|+|g|)$，对两项分别用 Hölder。

**Riesz–Fischer 定理（$L^p$ 完备性）**：$1\le p\le\infty$ 下 $L^p$ 为 Banach 空间。

$1\le p<\infty$ 的证明骨架：
1. 取 Cauchy 列 $\{f_n\}$ 的**速收子列** $\{f_{n_k}\}$ 使 $\|f_{n_{k+1}}-f_{n_k}\|_p<2^{-k}$。
2. 令 $g_K=\sum_{k=1}^K|f_{n_{k+1}}-f_{n_k}|$，由 Minkowski $\|g_K\|_p\le 1$；$g:=\sup_K g_K$，由 MCT $\int g^p\le 1$，故 $g\in L^p$ 且 $g<\infty$ a.e.
3. 于 $\{g<\infty\}$ 上 $\sum(f_{n_{k+1}}-f_{n_k})$ 绝对收敛，定义 $f:=\lim f_{n_k}$ a.e.
4. $|f-f_{n_k}|^p\le(|f|+g)^p\in L^1$，由 DCT $\|f_{n_k}-f\|_p\to 0$。
5. Cauchy + 子列收敛 $\Rightarrow$ 整列收敛。

$p=\infty$：可列零集外一致 Cauchy 列直接给出一致极限。

**对偶性**：σ-有限 + $1\le p<\infty$ $\Rightarrow$ $(L^p)^*\cong L^q$（$\tfrac1p+\tfrac1q=1$），同构映射 $g\mapsto\Lambda_g(f)=\int fg\,d\mu$。证明用 Radon–Nikodym：有界线性泛函 $\Lambda$ 定义集函数 $\nu_\Lambda(E)=\Lambda(\mathbf{1}_E)$，证 $\nu_\Lambda\ll\mu$，R–N 得密度 $g$。

**稠密性**：简单函数在 $L^p$ 中稠密（$p<\infty$）；在 $\mathbb{R}^n$ 上 $C_c^\infty$ 在 $L^p$ 中稠密（$p<\infty$）。

**机器人应用**：**Kalman 滤波是 $L^2$ 中的正交投影**。令 $L^2(\Omega,\mathcal{F},\mathbb{P})$ 为平方可积随机变量的 Hilbert 空间，内积 $\langle X,Y\rangle=\mathbb{E}[XY]$。给定观测子空间 $H_t=\overline{\operatorname{span}}\{1,y_1,\dots,y_t\}\subset L^2$，MMSE 估计量 $\hat x_{t|t}=\Pi_{H_t}x_t$ 是正交投影——**投影的存在性**依赖 $L^2$ 的完备性（Riesz–Fischer）；**新息序列** $\tilde y_t=y_t-\Pi_{H_{t-1}}y_t$ 的递推正交化给出 Kalman 增益 $K_t$。Gauss 假设下正交投影恰等于条件期望 $\mathbb{E}[x_t\mid y_{1:t}]$；非 Gauss 下二者分离，Kalman 只是最优**线性** MMSE。更广地，强化学习中**价值函数** $V\in L^2(\mu)$ 的最小二乘 TD (LSTD) 学习 = 在有限维子空间上对 Bellman 算子的 Galerkin 投影；$L^p$ 完备性保证迭代极限存在。

---

### §B2.14 收敛模式与相互关系 🔵核心 📖概念铺垫

**来源领域**：测度论收敛分析  
**前置依赖**：§B2.8, §B2.13  
**主参考**：Folland §2.4；Bartle Ch 7；Royden–Fitzpatrick §5

**四种收敛**：
1. **几乎处处 (a.e.)**：$f_n\to f$ 在一个余集为零测的集合上。
2. **依测度 (in measure)**：$\forall\varepsilon>0,\mu(\{|f_n-f|>\varepsilon\})\to 0$。
3. **$L^p$ 中 (norm)**：$\|f_n-f\|_p\to 0$。
4. **一致**：$\sup_x|f_n(x)-f(x)|\to 0$。

**蕴含关系图**（$\mu$ 有限时）：
$$
\text{一致}\Rightarrow L^\infty\Rightarrow L^p\Rightarrow L^1\Rightarrow\text{依测度}; \quad \text{a.e.}\overset{\text{Egorov}}{\Rightarrow}\text{依测度};\quad L^p\Rightarrow\text{依测度}
$$
且 $L^p$ 或依测度 $\Rightarrow$ 存在**子列** a.e. 收敛（Riesz 子列定理）。

**反例填充每个缺口**：
- a.e. ⇏ $L^p$：高瘦帽子 $f_n=n\mathbf{1}_{(0,1/n]}\to 0$ a.e.、$\|f_n\|_1=1$。
- $L^p$ ⇏ a.e.：**打字机序列** $\mathbf{1}_{[k/2^j,(k+1)/2^j]}$ 以字典序遍历，$\|\cdot\|_1\to 0$ 但处处不收敛。
- 依测度 ⇏ a.e.：同打字机序列。
- 无限测度下 a.e. ⇏ 依测度：$f_n=\mathbf{1}_{[n,n+1]}$ on $\mathbb{R}$。

**一致可积 (UI)**：$\{f_n\}\subset L^1$ 为 UI 若 $\lim_{M\to\infty}\sup_n\int_{\{|f_n|>M\}}|f_n|\,d\mu=0$。

**Vitali 收敛定理**：$\mu(X)<\infty$ + $f_n\to f$ 依测度 + UI $\Rightarrow$ $f\in L^1$ 且 $\|f_n-f\|_1\to 0$。Vitali 比 DCT 更通用（$f$ 存在 $L^1$ 控制 $\Rightarrow$ UI，反之不然）。

**机器人应用**：随机逼近算法的**几乎必然 (a.s.) 收敛 vs 依概率收敛**意味着不同实用保证。SGD 在非凸目标下经典结果 $\theta_n\xrightarrow{a.s.}\theta^\ast$（Robbins–Monro、Tsitsiklis 1994）保证\"几乎每一次训练轨迹都收敛\"——这对实际部署的鲁棒性至关重要。依概率收敛只保证\"多次训练的集合中大部分收敛\"。actor–critic 的双时间尺度分析（Konda–Tsitsiklis 2003）更需要 UI 来处理参数跳跃下的极限一致性。在机器人策略学习中强调 a.s. 收敛而非仅依概率，因为单次部署即决定系统成败。

---

### §B2.15 微分与 FTC 🟣进阶 ⚡推导密集

**来源领域**：实分析与测度论交界  
**前置依赖**：§B2.5, §B2.7  
**主参考**：Folland §3.4–§3.5；Rudin Ch 7；Stein–Shakarchi Ch 3；Wheeden–Zygmund Ch 7  
**黑盒标注**：无

**Hardy–Littlewood 极大函数** $Mf(x)=\sup_{r>0}\dfrac{1}{|B(x,r)|}\int_{B(x,r)}|f(y)|\,dy$，对 $f\in L^1_{\text{loc}}(\mathbb{R}^n)$ 定义。

**Vitali 覆盖引理**（有限版）：$\{B_1,\dots,B_N\}\subset\mathbb{R}^n$ 有限球族 $\Rightarrow$ 存在不交子族 $\{B_{i_j}\}$ 使 $\bigcup B_k\subseteq\bigcup 3B_{i_j}$。证明：按半径降序贪心选取。

**弱 (1,1) 极大不等式**：$m(\{Mf>\alpha\})\le\dfrac{3^n}{\alpha}\|f\|_1$。证明：对紧 $K\subseteq\{Mf>\alpha\}$，每 $x\in K$ 存在 $r_x$ 使 $\int_{B(x,r_x)}|f|>\alpha|B(x,r_x)|$；取有限覆盖用 Vitali 抽不交族，总测度估计 $m(K)\le 3^n\sum|B_{i_j}|\le\tfrac{3^n}{\alpha}\|f\|_1$。

**Lebesgue 微分定理**：$f\in L^1_{\text{loc}}(\mathbb{R}^n)\Rightarrow$ a.e. $x$ 为 **Lebesgue 点**，即
$$
\lim_{r\to 0}\frac{1}{|B(x,r)|}\int_{B(x,r)}|f(y)-f(x)|\,dy=0.
$$

证明骨架：连续函数在 $L^1$ 中稠密；对 $g\in C_c$ 由一致连续性处处为 Lebesgue 点；对残差 $f-g$ 用极大不等式估计 bad 集 $\{Mf>\alpha\}\cup\{|f-g|>\alpha\}$ 测度 $\le C\|f-g\|_1/\alpha$，$\varepsilon\to 0$ 令 $\|f-g\|_1\to 0$。

**绝对连续函数 (AC)** $f:[a,b]\to\mathbb{R}$：$\forall\varepsilon\exists\delta$ 使任不交区间族 $\{(a_i,b_i)\}$ 总长 $<\delta\Rightarrow\sum|f(b_i)-f(a_i)|<\varepsilon$。**有界变差 (BV)**：$V_a^b f:=\sup_\pi\sum|f(x_{i+1})-f(x_i)|<\infty$。AC $\subsetneq$ BV $\subsetneq$ 连续。

**Lebesgue FTC**：$f$ 在 $[a,b]$ 上 AC $\iff$ $f'$ 存在 a.e.、$f'\in L^1$ 且 $f(x)-f(a)=\int_a^x f'(t)\,dt$ $\forall x\in[a,b]$。

反例核心：**Cantor 函数** $c$ 连续、单调递增、$c'=0$ a.e.，但 $c(1)-c(0)=1\ne 0=\int_0^1 c'$——这说明连续 + BV 不够，必须 AC。AC 正是使\"零导数 a.e. ⇒ 常函数\"成立的精确条件。

**Radon–Nikodym 与 FTC 的统一**：$f$ 在 $[a,b]$ 上 AC $\iff$ $f$ 诱导的 Lebesgue–Stieltjes 测度 $\mu_f\ll m$ $\iff$ $f'=\dfrac{d\mu_f}{dm}$ 为 R–N 导数。

**机器人应用**：Hardy–Littlewood 极大函数在**视觉 SLAM 的局部亮度归一化、LiDAR 点云局部密度估计**中直接出现——"在半径 $r$ 的邻域内的平均值\"就是 $M$ 算子。更深的联系：**随机逼近的 ODE 方法**（Borkar 2008）中证 $\theta_n\to\theta^\ast$ a.s. 借助 Lebesgue 微分保证轨迹\"几乎处处\"对应其 ODE 极限。Cantor 函数反例提醒我们：机器人轨迹若仅连续而非 AC，即使导数 a.e. 为零也可能发生位移——这在分形路径、分段常数控制下必须小心。

---

### §B2.16 局部紧 Hausdorff 空间上的 Radon 测度与 Riesz 表示定理 🟣进阶 ⚡推导密集

**来源领域**：泛函分析 · 测度与拓扑交界  
**前置依赖**：§B2.4, §B2.5, §B2.13；点集拓扑中的局部紧 Hausdorff、Urysohn 引理  
**主参考**：Rudin Ch 2；Folland Ch 7；Cohn Ch 7；Royden–Fitzpatrick Ch 21  
**黑盒标注**：Urysohn 引理（LCH 版）

**Radon 测度**：$(X,\mathcal{B}(X),\mu)$，$X$ 为 LCH，$\mu$ 满足 (i) 紧集有限、(ii) 外正则（任 Borel 集）、(iii) 内正则（开集；σ-紧下对所有 Borel 集）。

**Riesz–Markov–Kakutani 表示定理**：$X$ 为 LCH，$\Lambda:C_c(X)\to\mathbb{R}$ 为正线性泛函（$f\ge 0\Rightarrow\Lambda f\ge 0$）$\Rightarrow$ 存在唯一 Radon 测度 $\mu$ 使 $\Lambda f=\int f\,d\mu\forall f\in C_c(X)$。

证明骨架：
1. **开集赋测**：$\mu(U)=\sup\{\Lambda f:0\le f\le 1,\operatorname{supp}f\subseteq U\}$。
2. **外测度**：$\mu^*(E)=\inf\{\mu(U):U\supseteq E\text{ 开}\}$。
3. **Carathéodory 可测性**：用 Urysohn 引理构造分离紧集与开集外部的函数 $\varphi$，通过 $\Lambda\varphi$ 验证 $\mu^*(E)=\mu^*(E\cap A)+\mu^*(E\cap A^c)$ 对开集 $A$ 成立，从而 Borel 集皆可测。
4. **正则性**：开集内正则由定义；σ-紧下传递到 Borel 集。
5. **积分表示**：对 $0\le f\le 1$ 做水平集分层 $f\approx\sum_{k=1}^N\tfrac1N\mathbf{1}_{\{f>k/N\}}$ 并用 Urysohn 光滑化，由 $\Lambda$ 线性 + 极限过渡得 $\Lambda f=\int f\,d\mu$。
6. **唯一性**：若 $\mu_1,\mu_2$ 表示同一 $\Lambda$，对紧 $K$ 由 Urysohn 刻画 $\mu_i(K)$，再由正则性传导到 Borel 集。

**对偶形式**：$C_0(X)^*\cong M(X)$（有限符号 Radon 测度空间，范数为全变差）。

**Haar 测度存在性**：$G$ 为 LCH 拓扑群。通过\"比率平均\"在 $C_c(G)$ 上构造左不变正线性泛函（Weil 1940；用 Tychonoff 紧性取极限）；Riesz 表示定理输出左不变 Radon 测度——**Haar 测度**，至多正常数倍唯一。

**机器人应用**（本节最核心意义）：$\mathrm{SO}(3)$、$\mathrm{SE}(3)$、一般李群 $G$ 上的积分没有 Lebesgue 测度可用（非欧流形），必须用 Haar 测度。具体后果：
- **各向同性姿态先验**：无偏好的姿态估计取归一化 Haar 测度 $dR$ 于 $\mathrm{SO}(3)$，$\operatorname{Vol}(\mathrm{SO}(3))=8\pi^2$。
- **群卷积** $(p\ast q)(g)=\int_G p(h)\,q(h^{-1}g)\,d\mu_H(h)$ 用于滤波/控制中不确定性组合。
- **$\mathrm{SE}(3)$ 上 Gauss**（Chirikjian, Barfoot–Furgale 2014）：$\xi\sim\mathcal{N}(0,\Sigma)$ 定义在李代数 $\mathfrak{se}(3)$，通过指数 $T=\exp(\xi^\wedge)\cdot T_0$ 映射到群；其密度相对 Haar 测度具显式形式。
- **Peter–Weyl 定理**把紧群 $L^2(G,\mu_H)$ 分解为不可约表示直和，支撑球面卷积 CNN（Cohen–Welling 2016）、旋转等变特征、姿态图谱滤波。

Riesz 定理是这一切的存在性证明机制——没有它，Haar 测度只是"愿景\"。

---

### §B2.17 与后续任务的接口总结 🔵核心 📖概念铺垫

本节把 B2 的产出接到路线图下游四条主线上，示意哪些工具将在何处再次出现。

**→ B3 泛函分析**：$L^p$（§B2.13）是 Banach 空间最核心的例子；$L^2$ 是无限维 Hilbert 空间的范式。B3 将在此基础上展开 Banach–Alaoglu、Hahn–Banach、开映射、闭图像等支柱定理；Riesz 表示定理（§B2.16）的对偶形式将作为一般 Banach 对偶理论的具体化。

**→ C1 概率论**：测度论直接变身为概率论的语言字典——**随机变量 ≡ 可测函数**、**期望 ≡ 积分**、**独立 ≡ 联合测度 = 积测度的边缘分解**、**条件期望 ≡ σ-子代数上 R–N 导数**、**鞅 ≡ 适应过程的条件期望塔**、**特征函数 ≡ Fourier 变换**。C1 的鞅收敛定理、大数定律、中心极限定理将大量调用 DCT 与 Fatou。

**→ Layer-1 流形积分与李群**：Riesz 表示定理（§B2.16）直通 Haar 测度；黎曼流形上的体积形式是 $n$-维坐标图局部与 $\sqrt{\det g}\,dx^1\cdots dx^n$ 的积分，整体化为流形上的 Radon 测度。李群指数映射与 Baker–Campbell–Hausdorff 将与 Haar 测度交互定义 Gauss–$\mathrm{SE}(3)$ 分布。

**→ 第二层 SLAM / 状态估计 / 控制 / 学习**：
- **DCT**（§B2.8）→ 粒子滤波收敛（Crisan–Doucet 2002；Del Moral 2004）；
- **R–N 导数**（§B2.12）→ 贝叶斯更新、重要性采样、Girsanov（PI²/MPPI）、KL/Fisher（TRPO/PPO）；
- **Fubini**（§B2.10）→ SLAM 边际化、Rao–Blackwellization (FastSLAM)、因子图 sum-product；
- **$L^2$ 投影**（§B2.13）→ Kalman/EKF/UKF、LSTD/LSPI；
- **Haar 测度**（§B2.16）→ 姿态估计、群卷积 CNN、姿态图谱。

**建议学习后立即阅读的三篇桥梁论文**：(1) Crisan & Doucet, \"A Survey of Convergence Results on Particle Filtering Methods for Practitioners\", *IEEE T-SP* 2002；(2) Barfoot & Furgale, \"Associating Uncertainty with Three-Dimensional Poses for Use in Estimation Problems\", *IEEE T-RO* 2014；(3) Munos & Szepesvári, \"Finite-Time Bounds for Fitted Value Iteration\", *JMLR* 2008。读懂这三篇标志着 B2 达标。

---

### 核心教材深度对照表

| 教材 | 覆盖范围 | 构造 Lebesgue 测度 | Radon–Nikodym 证明 | Riesz 表示 | 特色 | 对机器人博士生适用度 |
|---|---|---|---|---|---|---|
| **Folland 2e** (Wiley 1999) | Ch 1–3, 6, 7, 10–11 | Carathéodory 标准路线 (§1.4–§1.5) | 经典 Hahn 分解 + 上确界 (§3.2) | Ch 7 独立章节 | 现代广覆盖，450+ 习题，后有概率、Haar | ★★★★★ **首选主教材** |
| **Rudin RC 3e** (1987) | Ch 1–3, 6–8 | **反向路线**：经 Riesz 表示倒推 Lebesgue (§2.14) | **von Neumann Hilbert 证** (§6.10) | §2.14 作为起点 | 优雅简洁，连通复变 | ★★★★ 参考精读 |
| **Stein–Shakarchi III** (PUP 2005) | Ch 1–3, 6 | $\mathbb{R}^n$ 几何路线，开矩形覆盖 | von Neumann 风格 (Ch 6 §4) | Ch 6 间接 | 几何直觉、连通 Fourier | ★★★★ 搭配使用 |
| **Cohn 2e** (Birkhäuser 2013) | Ch 1–5, 7, 9, 10 | 标准 Carathéodory | 经典 Hahn 分解 (§4.2) | Ch 7 详尽 | **Ch 10 概率 + 鞅 + Brownian**；Ch 9 Haar | ★★★★★ **概率过渡最佳** |
| **Halmos** (1950) | 全书 | σ-ring 扩张 | Hahn 分解 | Ch X（Baire σ-ring） | 历史地位、σ-ring 框架 | ★★ 仅历史参考 |
| **Royden–Fitzpatrick 4e** | Ch 2–8, 17–22 | 单变量先行 | §18 Vitali 风格 | §21.4–§21.5 | 渐进温和，Vitali 收敛 | ★★★ 回退方案 |
| **Tao GSM 126** (AMS 2011) | 全书 | **Jordan → Lebesgue → Carathéodory** 动机驱动 | 未完整给 | 未涵盖 | blog 风格历史驱动 | ★★★★ **动机阅读首选** |
| **Bartle** (Wiley 1966/95) | 全书 | Part II 具体 Lebesgue | 经典 §8 | §9 仅 $C[0,1]$ | 最简洁入门 | ★★★ 快速入门 |

**12 周学习路径建议**：
- Week 1–2：Tao §1.1–§1.3（Jordan/Lebesgue 动机）
- Week 3–4：Folland Ch 1（§B2.2–§B2.5）
- Week 5–6：Folland Ch 2（§B2.6–§B2.10）
- Week 7–8：Folland Ch 3（§B2.11–§B2.12, §B2.15）+ Tao §1.6
- Week 9–10：Folland Ch 6（§B2.13–§B2.14）
- Week 11：Folland §7.1–§7.2（§B2.16）+ Cohn Ch 10（概率衔接）
- Week 12：Crisan–Doucet 2002、Barfoot–Furgale 2014、Munos–Szepesvári 2008 三篇桥梁论文

---

### 关键定理清单

| # | 定理 | 核心条件 | 结论 | 本大纲位置 | Folland 编号 | Rudin 编号 |
|---|---|---|---|---|---|---|
| 1 | Carathéodory 扩张 | 代数上预测度，σ-有限 | 唯一 σ-代数扩张 | §B2.4 | 1.11, 1.14 | §2.20 |
| 2 | 单调收敛 (MCT) | $f_n\ge 0,f_n\nearrow f$ | $\int f_n\nearrow\int f$ | §B2.8 | 2.14 | 1.26 |
| 3 | Fatou 引理 | $f_n\ge 0$ 可测 | $\int\liminf\le\liminf\int$ | §B2.8 | 2.18 | 1.28 |
| 4 | 控制收敛 (DCT) | $f_n\to f$ a.e., $|f_n|\le g\in L^1$ | $\int f_n\to\int f$, $L^1$ 收敛 | §B2.8 | 2.24 | 1.34 |
| 5 | Fubini–Tonelli | σ-有限 + (非负 / 绝对可积) | 累次 = 重积分 | §B2.10 | 2.36–2.37 | 8.8–8.9 |
| 6 | Radon–Nikodym | σ-有限, $\nu\ll\mu$ | $d\nu/d\mu$ 存在唯一 a.e. | §B2.12 | 3.8 | 6.10 |
| 7 | Lebesgue 分解 | σ-有限 | $\nu=\nu_{ac}+\nu_s$ 唯一 | §B2.12 | 3.8 | 6.10 |
| 8 | Hahn–Jordan 分解 | 符号测度 | $\nu=\nu^+-\nu^-$, $\nu^+\perp\nu^-$ | §B2.11 | 3.3 | 6.2, 6.6 |
| 9 | Riesz–Fischer | $1\le p\le\infty$ | $L^p$ 完备 | §B2.13 | 6.6, 6.8 | 3.11 |
| 10 | Hölder 不等式 | $1/p+1/q=1$ | $\int\|fg\|\le\|f\|_p\|g\|_q$ | §B2.13 | 6.2 | 3.5 |
| 11 | $(L^p)^*\cong L^q$ | σ-有限, $1\le p<\infty$ | 等距同构 | §B2.13 | 6.15 | 6.16 |
| 12 | Egorov 定理 | $\mu(X)<\infty$, $f_n\to f$ a.e. | 一致收敛于大集合 | §B2.6 | 2.33 | — |
| 13 | Lusin 定理 | $f$ Lebesgue 可测 | 闭集上连续 | §B2.6 | 7.10 | 2.24 |
| 14 | Hardy–Littlewood 弱 (1,1) | $f\in L^1$ | $m\{Mf>\alpha\}\le 3^n\|f\|_1/\alpha$ | §B2.15 | 3.17 | 7.4 |
| 15 | Lebesgue 微分 | $f\in L^1_{loc}$ | a.e. 点为 Lebesgue 点 | §B2.15 | 3.21 | 7.7 |
| 16 | Lebesgue FTC | $f$ AC | $f(x)-f(a)=\int_a^x f'$ | §B2.15 | 3.35 | 7.20 |
| 17 | Riesz–Markov–Kakutani | LCH, 正线性 $\Lambda$ on $C_c$ | 唯一 Radon 测度表示 | §B2.16 | 7.2 | 2.14 |
| 18 | Borel–Cantelli | $\sum\mu(A_n)<\infty$ | $\mu(\limsup A_n)=0$ | §B2.3 | — | — |

---

### 经典论文与里程碑文献

**数学史里程碑**：
1. Lebesgue, H., *Intégrale, longueur, aire*, 博士论文, 1902（Lebesgue 测度与积分的首次系统构造）。
2. Carathéodory, C., "Über das lineare Maß von Punktmengen", *Nachr. Akad. Wiss. Göttingen*, 1914（外测度与 μ\*-可测性）。
3. Radon, J., "Theorie und Anwendungen der absolut additiven Mengenfunktionen", *Sitzungsber. Akad. Wiss. Wien*, 122:1295–1438, 1913；Nikodym, O., "Sur une généralisation des intégrales de M. J. Radon", *Fund. Math.*, 15:131–179, 1930。
4. Riesz, F., "Sur les opérations fonctionnelles linéaires", *C. R. Acad. Sci. Paris*, 149:974–977, 1909（C[a,b] 对偶）；Kakutani, S., "Concrete representation of abstract (M)-spaces", *Ann. Math.*, 42:994–1024, 1941（LCH 版）。
5. Fubini, G., "Sugli integrali multipli", *Rend. Acc. Naz. Lincei*, 16:608–614, 1907；Tonelli, L., "Sull'integrazione per parti", *Rend. Acc. Naz. Lincei*, 18:246–253, 1909。
6. Haar, A., "Der Maßbegriff in der Theorie der kontinuierlichen Gruppen", *Ann. Math.*, 34:147–169, 1933；Weil, A., *L'intégration dans les groupes topologiques et ses applications*, Hermann, 1940（存在性的 Riesz 路径）。

**机器人与学习桥梁**：
7. Kalman, R. E., "A New Approach to Linear Filtering and Prediction Problems", *J. Basic Eng.*, 82(1):35–45, 1960。
8. Robbins, H. & Monro, S., "A Stochastic Approximation Method", *Ann. Math. Stat.*, 22(3):400–407, 1951。
9. Tsitsiklis, J. N., "Asynchronous Stochastic Approximation and Q-Learning", *Machine Learning*, 16:185–202, 1994。
10. Crisan, D. & Doucet, A., "A Survey of Convergence Results on Particle Filtering Methods for Practitioners", *IEEE T-SP*, 50(3):736–746, 2002。
11. Del Moral, P., *Feynman–Kac Formulae: Genealogical and Interacting Particle Systems with Applications*, Springer, 2004。
12. Montemerlo, Thrun, Koller, Wegbreit, "FastSLAM: A Factored Solution to SLAM", *AAAI* 2002；FastSLAM 2.0, *IJCAI* 2003。
13. Karaman, S. & Frazzoli, E., "Sampling-based Algorithms for Optimal Motion Planning", *IJRR*, 30(7):846–894, 2011。
14. Todorov, E., "Efficient Computation of Optimal Actions", *PNAS*, 106(28):11478–11483, 2009；Theodorou, Buchli, Schaal, "A Generalized Path Integral Control Approach to RL", *JMLR*, 11:3137–3181, 2010。
15. Munos, R. & Szepesvári, C., "Finite-Time Bounds for Fitted Value Iteration", *JMLR*, 9:815–857, 2008。
16. Barfoot, T. & Furgale, P., "Associating Uncertainty with 3D Poses for Use in Estimation Problems", *IEEE T-RO*, 30(3):679–693, 2014。
17. Chirikjian, G. S., *Stochastic Models, Information Theory, and Lie Groups*, Vols. 1–2, Birkhäuser, 2009 & 2012。

---

### 结语：从测度到机器人的三层跃迁

B2 的学习体验应该是**三次认知跃迁**：第一次发生在 §B2.5——意识到\"密度\"只是 R–N 导数，概率分布的本体是测度；第二次发生在 §B2.8——理解 DCT 如何让\"极限与积分交换\"在机器人蒙特卡洛算法中变成定量收敛率；第三次发生在 §B2.16——看到 Riesz 表示定理如何凭空\"造出\" $\mathrm{SO}(3)$ 上的 Haar 测度，让整个李群机器人学成立。

带着这三次跃迁的收获进入 B3 与 C1，会发现泛函分析里 Banach–Alaoglu 不再是\"抽象\"的（它是粒子滤波弱紧性的来源），而鞅收敛不再是\"概率论装饰\"（它是 Q-learning a.s. 收敛证明的核心）。测度论不是数学修养，是**机器人学博士未来五年论文的脚手架**；B2 完成得好坏，直接决定之后能否独立评估一篇滤波/控制/学习论文的数学正确性。

建议学习者在每一节结束时问自己三个问题：(i) 这一节的核心定理失效会导致哪个机器人算法出错？(ii) 若把定理条件削弱一步（去掉 σ-有限、去掉控制函数、去掉完备性），反例是什么？(iii) 我能在一张 A4 纸上把主要证明骨架默写出来吗？三问皆\"是\"，即可进入下一节。
---

