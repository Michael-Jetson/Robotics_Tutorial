## 博士前数学路线图·第五批·子专题 C：iSAM2 与 Bayes 树——增量式平滑与建图的数据结构与算法

---

### 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回 5-B 复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **MAP = WNLS**：写出 SLAM 中 MAP 估计等价的加权非线性最小二乘问题。残差 $r_k(x)=h_k(x)-z_k$ 的权重矩阵是什么？ | 5-B §B.2 |
| 2 | **Gauss-Newton 正规方程**：写出 GN 的正规方程 $(J^\top\Omega J)\delta=-J^\top\Omega r$，解释为什么 $H=J^\top\Omega J$ 是近似 Hessian，以及它相比真 Hessian 忽略了什么。 | 5-B §B.4 |
| 3 | **Schur 补与边缘化**：对 BA 问题，Schur 消元路标的含义是什么？写出 Reduced Camera System $S=H_{pp}-H_{pl}H_{ll}^{-1}H_{pl}^\top$ 的概率解释（边缘化联合分布中的路标）。 | 5-B §B.10 |
| 4 | **稀疏 Cholesky 与排序**：什么是 fill-in？为什么消元顺序影响 fill-in 数量？COLAMD 和 METIS 各解决什么问题？ | 5-B §B.11–B.13 |
| 5 | **因子图语义**：画一个 3 pose + 2 landmark 的因子图，标出变量节点和因子节点。消元一个 landmark 在图上的操作是什么？ | 5-B §B.1, §B.3 |

---

### 本章目标

学完本章后，你应当能够：

1. 从因子图出发，手动执行变量消元并构造完整的 Bayes 树，理解每个团的 frontal/separator 划分及其概率含义
2. 解释 iSAM2 的三大核心机制（findAffectedTop、Fluid Relinearization、Wildfire 回代）的数学依据和工程实现
3. 在 GTSAM（C++/Python）中正确配置和使用 `ISAM2`，理解关键参数（`relinearizeThreshold`、`relinearizeSkip`、`wildfireThreshold`、`factorization`）的物理含义与工程权衡
4. 区分 iSAM2 与 EKF-SLAM、滑窗 BA、Batch LM 的适用场景，做出工程选型决策
5. 诊断 iSAM2 运行时的典型故障（发散、非叶边缘化报错、SmartFactor 冲突等），并应用正确的修复策略

---

### 知识树总览

```text
iSAM2 与 Bayes 树
├── 前置基础
│   ├── MAP = WNLS（5-B）
│   ├── Gauss-Newton 正规方程（5-B）
│   ├── Schur 补与边缘化（5-B）
│   └── 稀疏 Cholesky 与排序（5-B）
├── 核心数据结构
│   ├── 因子图 → Bayes 网 → Bayes 树（§C.3-C.5）
│   ├── 消元树与超节点等价（§C.5）
│   └── Bayes 树关键性质：局部性、可分离性、叶剪枝（§C.6）
├── iSAM2 算法
│   ├── 总体流程七步（§C.8）
│   ├── Fluid Relinearization（§C.9, §C.28）
│   ├── Wildfire 回代（§C.10, §C.27）
│   └── 收敛性与 Dogleg 增强（§C.12）
├── 复杂度分析
│   ├── Lipton-Rose-Tarjan 嵌套剖分（§C.13）
│   └── 2D O(√N) vs 3D O(N^{2/3})（§C.13）
├── 工程实践
│   ├── GTSAM API 与参数配置（§C.14-C.15）
│   ├── LIO-SAM / Kimera-VIO 案例（§C.16-C.17）
│   └── 15 条常见陷阱（§C.23）
└── 前沿扩展
    ├── 并行推断与分布式 SLAM（§C.18）
    ├── 连续时间 GP-SLAM（§C.19）
    └── 与 SE-Sync / GNC / InEKF 的桥梁（§C.31）
```

本章处于**因子图优化系列**的核心位置：它以 5-B（批量因子图求解）为前置，向前承接 EKF/RTS（5-A）的滤波视角，向后连通鲁棒估计（5-F）、全局最优证书（5-E）、以及不变滤波理论（5-D）。

---

> **底线陈述（BLUF）**：iSAM2 把全图批量优化问题改写成**沿 Bayes 树自底向上局部编辑 + 沿根向下野火回代**的增量算法。**Bayes 树 = 弦化因子图的团树 + 按消元顺序有根化 + 每团存条件密度 $p(F_k\mid S_k)$**，它本质上等价于稀疏 Cholesky 因子 $R$ 的超节点结构（Liu 1990、Kaess 2012 §3.2）。三把核心钥匙把批量变增量：**(i) findAffectedTop + removeTop**（只拆受影响路径上的团）、**(ii) Fluid Relinearization**（只对 $|\delta x_j|>\beta$ 的变量重新线性化）、**(iii) Wildfire 回代**（只沿变化显著的子树向下传播新 $\delta$）。每步复杂度在 2D pose graph 上为 $O(\sqrt N)$、在 3D BA 上为 $O(N^{2/3})$——这两个著名的界不是 iSAM 原创，而是 Lipton-Rose-Tarjan 1979 的嵌套剖分定理 + Krauthausen-Dellaert 2006 的 SLAM 应用。工程上 iSAM2 是 **GTSAM 的旗舰求解器**，也是 LIO-SAM、Kimera-VIO、LAMP、Hydra 等系统的后端引擎；它**不是 LM**（无阻尼 $\lambda$），大回环需连跑多次 `update()` 或切 Dogleg；它与 SmartFactor、`marginalizeLeaves` 组合时有一串著名陷阱（GTSAM Issues #595/#1101/#1976）。本文把从 §C.1 到 §C.31 的全部机制、推导、代码映射、陷阱、对比一次讲清，目标档位为博士入学 + 档位 4 进阶。

---

### §C.1 Batch SLAM 的计算瓶颈：为什么必须增量 ⭐⭐

5-B 建立的 MAP 因子图求解是**批量**（batch）的：每收到新一帧，把新因子追加进 $\mathcal F$，对整张图重新做 Gauss-Newton（GN）或 Levenberg-Marquardt（LM）。这条路线的复杂度取决于信息矩阵 $\Lambda = J^\top\Sigma^{-1}J$ 的稀疏 Cholesky：

- **稠密上界**：$O(N^3)$，其中 $N$ 是状态维度。对 KITTI 一段 4500 帧的 pose graph（$N\approx 2.7\times 10^4$），单次 Cholesky 在桌面 CPU 需 ~秒级。
- **稀疏上界（2D pose graph）**：$O(N^{3/2})$（Lipton-Rose-Tarjan 1979 + COLAMD/METIS 嵌套剖分）；**3D BA**：$O(N^2)$。
- **实时要求**：LiDAR SLAM 10 Hz、VIO 30–100 Hz，每帧预算 10–100 ms。纯 batch 到 $N\sim 10^4$ 后就卡住。

**增量式平滑（Incremental Smoothing）的目标**：新来一批因子 $\mathcal F_\text{new}$，**只更新受影响的变量子集**——而不是重算整图。核心观察：在 pose graph 中，新的里程计因子 $f(x_{k-1}, x_k)$ **局部**地耦合两个变量；即使出现回环因子 $f(x_k, x_m)$（$m\ll k$），受影响的也只是**沿 Bayes 树从 $x_k$ 到 $x_m$ 公共祖先的路径**。数据结构若能天然编码这种"变化局部性"，算法就能做到期望 $O(\sqrt N)$ 甚至 $O(\log N)$ 每帧。这正是 Bayes 树的设计目的。

**对比 EKF-SLAM**：EKF 把所有 landmark 塞进一个稠密协方差 $\Sigma\in\mathbb R^{N\times N}$，每步 $O(N^2)$；而且一次线性化不可撤销，长期漂移无解。iSAM2 既保留全图（可再线性化），又只付局部代价——两全其美。

> **本质洞察**：增量式平滑的核心不是"更快地做同一件事"，而是**利用概率图结构天然编码的条件独立性**。
> 在 Bayes 树中，给定 separator 变量的值，子树与图的其余部分**条件独立**。
> 新因子只打破局部条件独立关系——这种"破坏的局部性"正是 $O(\sqrt{N})$ 复杂度的来源。

**反事实推理——如果不用增量，纯 batch 会怎样？** 考虑一个自动驾驶场景：4500 帧 KITTI 序列，每帧一个 6-DoF pose，总状态维度 $N=27000$。Batch Cholesky 在 COLAMD 排序下需要 $O(N^{3/2})\approx 4.6\times 10^6$ 次浮点运算。桌面 CPU（10 GFLOPS）耗时约 $0.5$ ms/帧——看起来可行。但再加入 $5000$ 个 landmark（每个 3-DoF），$N$ 跳到 $42000$，Hessian 不再是 pose graph 的带状结构而是 BA 的 arrow-head 结构，Schur 补后仍需 $O(N_{pose}^2 \cdot N_{lm})\approx 10^{11}$ 次运算。此时 iSAM2 的局部更新（每步只消元受影响的 $\sim 50$ 个团）节省了 3-4 个数量级。

**历史演进脉络**：

| 时间 | 工作 | 核心贡献 | 局限 |
|------|------|---------|------|
| 1986 | Smith-Self-Cheeseman | EKF-SLAM：第一次把 SLAM 形式化为状态估计 | $O(N^2)$ 稠密协方差 |
| 1997 | Lu-Milios | "全局一致 pose graph"：全轨迹关系表示 | 没有高效求解 |
| 2002 | Montemerlo et al. | FastSLAM：粒子滤波分离 pose 与 map | 粒子退化、高维不适 |
| 2004 | Dellaert | SqrtSAM：稀疏 $R$ 因子直接解 batch | 每帧 $O(N^{3/2})$ |
| 2006 | Dellaert-Kaess | SqrtSAM = 因子图上的变量消元 | 仍需 batch |
| 2008 | Kaess et al. | **iSAM1**：Givens QR 增量更新 $R$ | 周期性批量尖峰 |
| 2010 | Kaess et al. | **Bayes 树**：消元产物的概率数据结构 | 纯理论，还未完整算法化 |
| 2012 | Kaess et al. | **iSAM2**：Bayes 树 + fluid relin + wildfire | 当前工业标准 |
| 2017 | Dellaert-Kaess | FnT 综述：统一因子图范式 | 教材，非新算法 |

这条脉络清晰地展示了：**从 EKF 的"全耦合"到 iSAM2 的"局部编辑"，本质上是对条件独立结构的逐步开发**。EKF 无视所有结构，把 $N$ 个变量捆成一个稠密块；SqrtSAM ���现了稀疏性但只能 batch 利用；iSAM1 发现了增量更新 $R$ 的可能但无法避免周期性 batch；iSAM2 最终通过 Bayes 树把"哪些团受影响"的判断编码进数据结构本身，实现了真正���义上的"只更新必须更新的部分"。

---

### §C.2 iSAM1 的贡献与局限 ⭐⭐⭐

**Kaess, Ranganathan, Dellaert, "iSAM: Incremental Smoothing and Mapping Using the Square Root Information Filter", IEEE TRO 24(6):1365–1378, 2008** 是第一次把增量平滑落地的工作。

#### 核心思想：Square Root Information Filter (SRIF) + Givens 增量 QR

把信息矩阵写成 $\Lambda = R^\top R$，$R$ 上三角。GN 子问题变为解 $R\,\delta = d$。新测量对应新行 $w^\top$ 加在 $R$ 底部、新 RHS $\gamma$ 加在 $d$ 末尾：

```python
def isam1_update(R, d, w, gamma):
    R = vstack([R, w.T]); d = append(d, gamma)
    for k in nonzero_columns(w):
        c, s = givens(R[k,k], w[k])      # cos/sin
        R[k,:], w[:] = rotate(R[k,:], w[:], c, s)
        d[k], gamma  = rotate(d[k], gamma, c, s)
    # 舍弃 w（已全 0）
    return R, d
```

每行通常只需常数次 Givens 旋转（TRO §III-A,B，p.1368–1370）。

#### 局限三连（iSAM2 正是为解决这三点而生）

1. **周期性批量**（TRO §III-D）：每 $N_\text{batch}\approx 100$ 帧要做一次 **批量 COLAMD 重排序 + 全部重线性化 + 重做 QR**。图上画出每帧用时曲线呈周期性尖峰（Kaess 2012 Fig. 14a）。
2. **重线性化只在批量步**：两次批量之间，线性化点与最新估计偏离，$\chi^2$ 漂移（Kaess 2010 Fig. 7）。
3. **边缘化难支持**：Givens QR 的 $R$ 没有自然的"剪叶"操作；滑窗 SLAM 只能退回批量 Schur 补。

**教材**：Dellaert & Kaess, *"Factor Graphs for Robot Perception"*, FnT in Robotics 6(1-2), 2017, §5.1–5.3 完整讲解 iSAM1；Kaess PhD 论文（GaTech 2008）Ch.5 有最细节版。

---

### §C.3 从因子图到 Bayes 网再到 Bayes 树 ⭐⭐

#### 符号设定

因子图 $G=(\mathcal F, \Theta, \mathcal E)$ 上的联合分布（Kaess 2012, Eq. 1, p. 2）：

$$f(\Theta)=\prod_i f_i(\Theta_i),\qquad \Theta^*=\arg\max_\Theta f(\Theta).$$

Gaussian 因子 $f_i(\Theta_i)\propto \exp(-\tfrac12\|h_i(\Theta_i)-z_i\|^2_{\Sigma_i})$，线性化得 $\arg\min_\Delta \|A\Delta-b\|^2$，其中 $\Sigma_i^{-1/2}$ 已吸进 $A$ 的行块。

#### 变量消元（Variable Elimination，Kaess 2012 Alg. 2）

按给定消元顺序 $\pi$，逐个消元变量 $\theta_j$：

```text
def eliminate_variable(G, θ_j):
    # 1. 收集邻居因子；定义 separator
    adj = {f_i ∈ G : θ_j ∈ scope(f_i)}
    S_j = (⋃ scope(f_i) for f_i in adj) \ {θ_j}
    # 2. 构造联合（未归一化）密度
    f_joint(θ_j, S_j) = ∏_{f_i ∈ adj} f_i
    # 3. 链式规则分解
    P(θ_j | S_j), f_new(S_j) = chain_rule(f_joint)
    # 4. 更新图 + 追加条件到 Bayes 网
    G.factors = (G.factors \ adj) ∪ {f_new}
    BayesNet.append( P(θ_j | S_j) )
```

**Gaussian 情形**（Kaess 2012 Eq. 9–11, p. 5–6）：设 $A_j=[a\,|\,A_S]$ 是所有含 $\Delta_j$ 因子 Jacobian 拼块，

$$f_\text{joint}(\Delta_j, s_j)\propto \exp\!\Big(-\tfrac12\|a\Delta_j + A_S s_j - b\|^2\Big).$$

应用 Gram-Schmidt 一步：

$$P(\Delta_j|s_j)\propto \exp\!\Big(-\tfrac12\|a\|^2(\Delta_j + r\,s_j - d)^2\Big), \quad r\triangleq a^\dagger A_S,\; d\triangleq a^\dagger b,\; a^\dagger=(a^\top a)^{-1}a^\top.$$

若按 QR 记号写，就是 $R_{jj}=\|a\|$，条件密度中保留 $R_{jj}(\Delta_j+r s_j-d)$。只写括号内的均值关系可以解释 Bayes 网结构，但不能代表完整的信息量。

余因子：$f_\text{new}(s_j)=\exp(-\tfrac12\|A' s_j-b'\|^2)$，$A'\triangleq A_S - a r$，$b'\triangleq b - a d$。**这正是 $A$ 的稀疏 QR 的一行** $(r, d)\to R$。

#### Bayes 网的弦性 + 团分解 → Bayes 树

- 消元产物是**有向无环图**（DAG）：每个节点 $\theta_j$ 存储 $P(\theta_j \mid S_j)$。Kaess 2012 §3.2 证明这个 DAG 是**弦图**（chordal）——每个 $\geq 4$ 环有弦。
- 弦图的团可用 **最大基数搜索**（MCS, Tarjan-Yannakakis 1984）以**反消元顺序**发现。
- **Bayes 树的正式定义**（Kaess 2012 §3.2, p. 6）：
  > A Bayes tree is a **directed tree** where the nodes represent **cliques** $C_k$ of the underlying chordal Bayes net. Each clique stores one conditional density $P(F_k\mid S_k)$, where the **separator** $S_k = C_k\cap\Pi_k$（$\Pi_k$ 是父团），**frontal variables** $F_k = C_k\setminus S_k$。写作 $C_k = F_k : S_k$。根 $F_r$ 有 $S_r=\emptyset$。

联合分布分解（Kaess 2012 Eq. 13）：

$$\boxed{P(\Theta) = \prod_k P(F_k\mid S_k)}$$

---

### §C.4 Bayes 树的构造算法 ⭐⭐⭐

输入：因子图 $G$ + 消元顺序 $\pi$（由 COLAMD/METIS 决定）。

**步骤**（Kaess 2012 Alg. 3, p. 6）：

1. **符号消元**：对 $G$ 按 $\pi$ 执行变量消元，得到 Bayes 网的所有条件密度 $P(\theta_j\mid S_j)$。
2. **反向扫描条件密度**（按消元顺序的逆序）：
   ```
   for each P(θ_j | S_j) in reverse order:
       if S_j == {}:
           start new root clique F_r containing θ_j
       else:
           C_p ← parent clique containing first-eliminated(S_j) as frontal
           if F_p ∪ S_p == S_j:
               insert θ_j into C_p          # 合并进已有团
           else:
               start new clique C' = (θ_j : S_j) as child of C_p
   ```
3. **结果**：有根有向树，每个团存储 `[F_k, S_k, P(F_k|S_k) as R-block]` 及子节点指针。

**直觉**：该算法本质上是**超节点检测**——把连续消元且共享 separator 的变量打包成一个稠密块（即 Cholesky 的 supernode）。这不仅是概率图技巧，也是 CHOLMOD、PaStiX、MUMPS 等稀疏矩阵库的标准 speedup。

---

### §C.5 Bayes 树与稀疏 Cholesky 的等价 ⭐⭐⭐

这是**全章最重要的数学桥梁**。5-B 讲过 Schur 补 = 消元；本节把消元产物的**结构**与 Cholesky $R$ 的**稀疏模式**对应。

#### 消元树（Elimination Tree，Liu 1990）

**定义**（Davis 2006, Ch. 4, §4.1）：对 Cholesky 因子 $L$（$A = LL^\top$，或等价上三角 $R$，$A = R^\top R$），节点 $i$ 的父为

$$\mathrm{parent}(i) = \min\{\, j > i : L_{ji} \neq 0\,\}.$$

**定理（Liu 1990, Thm 2.4）**：忽略数值消去，$L_{ji}\neq 0$ 意味着 $j$ 位于 $i$ 在消元树 $\mathcal T(A)$ 中的祖先路径上；反过来，祖先关系描述的是传递依赖或可达性，不要求每一个祖先位置都出现直接非零元。等价地，$\mathcal T(A)$ 是填充图 $G^+(A)$ 的生成树，即有向图 $G(L^\top)$ 的**传递约简**（transitive reduction）。

#### 超节点（supernode）= Bayes 树的团

**定义**：连续若干列 $\{i_s,\ldots,i_{s+t-1}\}$ 在 $L$ 中具有**相同（higher adjacency 意义下）**的非零结构，并在消元树中构成链。Pothen & Sun 1990 证明：

$$\text{supernode of }L \;\Longleftrightarrow\; \text{maximal clique of }G^+(A) \;\Longleftrightarrow\; \text{Bayes 树团 }C_k.$$

#### 三条教学用等价陈述

1. **Bayes 树的 cliques = Cholesky 超节点**。
2. **Bayes 树的祖先关系 = 稀疏回代的传递依赖**：$R_{ij}\neq 0$ 给出直接依赖，沿这些直接依赖向上闭包才形成 Bayes 树中的祖先关系。
3. **Bayes 树野火回代路径 = 稀疏回代 `R\d` 的依赖链**。

#### 行结构定理（Gilbert-Ng 1993）

$$\mathrm{struct}(R_{i,:}) = \{i\} \cup \bigcup_{k:\,L_{ki}\neq 0,\,k<i} \mathrm{struct}(R_{k,:}) \cap \{j:j\geq i\}.$$

等价地，**沿消元树从 $i$ 向上的所有祖先** 给出 $R_{i,:}$ 的非零模式（row subtree）。应用于 Bayes 树：从团 $C_k$ 到根的路径上的变量集合 = $R$ 中 $F_k$ 行块的非零列集合。

**教材参考**：Davis *Direct Methods for Sparse Linear Systems* (SIAM 2006) §4.1–§4.3；Kaess PhD 论文 Ch.4；Gilbert "Predicting structure in sparse matrix computations" SIMAX 1994。

#### 为什么这个等价如此重要——三重解读

理解 "Bayes 树 = 超节点 = 弦图团" 的等价性是掌握 iSAM2 的关键。这个等价从三个方向给出洞察：

**从概率角度**：Bayes 树的每个团存储一个条件密度 $P(F_k\mid S_k)$。联合分布是所有团条件密度的乘积。这意味着：给定 separator $S_k$ 的值，frontal $F_k$ 与树中非后代节点**条件独立**。这是 iSAM2 局部更新的概率根基。

**从线性代数角度**：稀疏 Cholesky 因子 $R$ 的每个超节点对应一组列，它们共享相同的非零行结构。回代 $R\delta=d$ 时，超节点内部用稠密 BLAS-3 操作（Level-3 BLAS: `dtrsv`/`dgemm`），超节点之间只传递 separator 部分。这正是 Bayes 树 wildfire 回代在矩阵运算层面的实现。

**从图论角度**：弦图（chordal graph）的完美消元序列保证消元过程不产生"意外"的新边（即 fill-in 只出现在已有边的传递闭包内）。最大基数搜索（MCS）能在 $O(N+M)$ 时间内发现弦图的所有最大团，并按反消元顺序组织成团树（clique tree）。Bayes 树就是这棵团树的有根版本。

> **跨领域类比**：Bayes 树之于概率图推断，如同 B+ 树之于数据库查询。B+ 树把有序数据组织成"只有叶子存数据、内部节点存索引"的结构，使得插入/删除/查找都是 $O(\log N)$。Bayes 树把联合分布组织成"只有叶子存局部信息、根存全局耦合"的结构，使得增量更新只需沿受影响路径操作。两者的共同设计哲学是：**把全局结构分层编码，使局部操作不需要触碰全体**。区别在于 B+ 树的更新路径总是 $O(\log N)$（平衡树），而 Bayes 树的更新路径长度取决于图的拓扑——pose graph 约 $O(\sqrt{N})$（平面图分隔符），BA 约 $O(N^{2/3})$。

---

### §C.6 Bayes 树的关键性质 ⭐⭐

| 性质 | 陈述 | 用途 |
|------|------|------|
| **局部性** | 新因子 $f(x_a, x_b)$ 只影响从团 $C(x_a)$、$C(x_b)$ 到**最低公共祖先**（LCA）路径上的所有团 | iSAM2 findAffectedTop 的根据 |
| **可分离性** | 任何子树在给定其 separator 后，与树的其余部分条件独立 | 并行消元、DDF-SAM 多机器人 |
| **增量可编辑** | 拆开受影响路径 → 产生 orphan 子树（附带缓存边缘因子）→ 重新消元 top → 挂回 orphan | iSAM2 removeTop + graft 的依据 |
| **叶剪枝 = 精确边缘化** | 叶团的 frontal 变量的信息 **已完全向上传递** 给父团 separator，直接删叶 = 零误差边缘化 | marginalizeLeaves、Fixed-Lag Smoother |
| **消元顺序自由度** | 同一 Bayes 树可对应多个 $R$（Kaess 2012 §3.2 末）——子团排序可任意 | 解释 Bayes 树比 $R$ 结构更稳 |

**叶剪枝定理（非正式）**：若 $F_k$ 全是叶团 frontal，

$$\int P(\Theta)\,dF_k \;=\; \underbrace{\int P(F_k\mid S_k)\,dF_k}_{=1}\cdot\prod_{j\neq k} P(F_j\mid S_j) \;=\; \prod_{j\neq k} P(F_j\mid S_j).$$

——零计算。这是 Bayes 树对 junction tree 的一个突出优势（信息只向上流）。

---

### §C.7 图示与直觉：5 变量 pose graph 完整变换 ⭐⭐

**因子图**（3 pose + 2 landmark + 1 prior）：

```text
  [prior]──x1──[odo]──x2──[odo]──x3
             \  |    /          |
              [meas]─l1          [meas]─l2
```

6 个因子：$f(x_1),\ f(x_1,x_2),\ f(x_2,x_3),\ f(l_1,x_1),\ f(l_1,x_2),\ f(l_2,x_3)$。

**消元顺序**：$l_1, l_2, x_1, x_2, x_3$（先消 landmark 再消 pose，典型 BA 顺序）。

1. 消 $l_1$：吸收 $f(l_1,x_1), f(l_1,x_2)$；产出 $P(l_1\mid x_1,x_2)$ + 新因子 $f'(x_1,x_2)$。
2. 消 $l_2$：产出 $P(l_2\mid x_3)$ + $f'(x_3)$。
3. 消 $x_1$：吸收 $f(x_1), f(x_1,x_2), f'(x_1,x_2)$；产出 $P(x_1\mid x_2)$ + $f''(x_2)$。
4. 消 $x_2$：产出 $P(x_2\mid x_3)$ + $f''(x_3)$。
5. 消 $x_3$：产出 $P(x_3)$（先验）。

**Bayes 树**（团发现用 MCS）：

```text
              ┌──────────────┐
              │  {x2, x3}    │     ← 根团（F_r={x2,x3}, S_r=∅）
              └───┬──────┬───┘
                  │      │
        ┌─────────▼───┐  │  ┌─▼──────────┐
        │ {l1,x1 | x2}│  │  │ {l2 | x3}  │
        └─────────────┘  │  └────────────┘
                         │
                  （在此根团外不再展开——已经是最简）
```

**读法**：`F : S` 表示 frontals : separator。左团 $C_1 = \{l_1, x_1, x_2\}$，$F_1=\{l_1, x_1\}$，$S_1=\{x_2\}\subset F_r$。右团 $C_2 = \{l_2, x_3\}$，$F_2=\{l_2\}$，$S_2=\{x_3\}$。

**"根是最全局、叶是最局部" 的深义**：
- 根团的 frontal 是**最后消元**的变量——它们携带了整张图回传的所有信息。
- 叶团的 frontal 是**最先消元**的变量——它们只与局部邻居耦合。
- 这就是为什么"回环会传到根"（全局耦合）、"里程计更新只到叶附近"（局部耦合）。

**添加新回环因子 $f'(x_1, x_3)$**（参见 Kaess 2012 Fig. 6）：$x_1$ 和 $x_3$ 的 root-路径 = 整棵左支 + 根。步骤：

1. 剥离左团与根 → "top" 子因子图 + 新因子；右团 $\{l_2\mid x_3\}$ 成 **orphan**（带缓存边缘因子）。
2. 重新消元（含重排序），形成新 Bayes 子树。
3. orphan 重新挂回。

新根变为 $\{x_1, x_2, x_3\}$，左孩子 $\{l_1\mid x_1, x_2\}$，右孩子（orphan）$\{l_2\mid x_3\}$。

---

### §C.8 iSAM2 总体流程（Kaess 2012 Alg. 8） ⭐⭐

```python
# 状态：Bayes 树 T、非线性因子集 F、线性化点 Θ、增量 δ
# 每步输入：新因子 F_new、新变量初值 Θ_new
def iSAM2_step(T, F, Θ, δ, F_new, Θ_new):
    F = F ∪ F_new
    Θ = Θ ∪ Θ_new                              # 初始化新变量
    # (1) fluid relinearization：找需要重线性化的变量
    Θ, M = fluid_relinearize(Θ, δ, β)          # Alg. 5
    # (2) 标记 redo-top：新因子 affected ∪ relin affected
    J = M ∪ affected_by(F_new)
    # (3) removeTop + relinearize + reorder + eliminate + graft
    T = update_top(T, F, J)                    # Alg. 6
    # (4) wildfire partial back-substitution
    δ = partial_solve(T, α)                    # Alg. 7
    return T, Θ, δ
```

**七步拆解**（对应问题描述 §C.8 的 step 1–8）：

1. **addVariables / addFactors**：把 $F_\text{new}$ 和 $\Theta_\text{new}$ 加入。
2. **findAffectedVariables**：从新因子涉及的变量 + relin 变量出发，沿 Bayes 树 **parent 指针** 向根游走——路径上的所有团都进入 "top"。
3. **removeTop**：拆下 top 中的团；它们的孩子成 orphan，保留 **cached marginal factors**（避免重消元整棵子树）。
4. **relinearize**：在最新 $\Theta$ 处重算 top 涉及的非线性因子的 Jacobian（只对受影响子集）。
5. **reorder**（可选）：对 top 内变量做局部 COLAMD/CCOLAMD 重排序，**最近访问变量约束到排序末尾**（推向根，使下次 top 更小）。
6. **eliminate**：对 top 因子集 + orphan 的缓存边缘因子执行变量消元 → 新 Bayes 子树。
7. **graft**：把 orphan 重新挂回新子树。
8. **update $\delta$**：沿受影响团向下执行 **wildfire 回代**，非路径团保持旧 $\delta$。

---

### §C.9 Fluid Relinearization ⭐⭐⭐

**动机**：完整重线性化 = batch GN，每步 $O(N^{3/2})$。但实测 80–90% 的变量 $\|\delta x_j\|$ 很小，根本无需重线性化。**Fluid Relinearization** 的中心思想：

$$J = \{\, j : |\delta_j| \geq \beta\,\} \;\; \Rightarrow\;\; \Theta[J]\leftarrow \Theta[J]\oplus \delta[J],\;\; M = \bigcup_{j\in J}\{\text{所有含 }j\text{ 的团 + 它们的祖先}\}.$$

```python
def fluid_relinearize(Θ, δ, β):
    J = {j for j in Θ if abs(δ[j]) >= β}
    Θ[J] = Θ[J] ⊕ δ[J]
    M = { clique_and_ancestors(j) for j in J }
    return Θ, M
```

> **概念陷阱：把 fluid relinearization 理解为 FEJ。** Fluid relinearization 和 FEJ（First Estimate Jacobian）都涉及"线性化点"，但目标完全不同。Fluid relinearization 是增量 GN 的效率启发式——变化大的变量刷新 Jacobian，变化小的沿用旧值，从而减少受影响团。FEJ 的目标是固定特定 Jacobian 求值点以维护滤波器在不可观子空间的一致性。iSAM2 的选择性重线性化不等于 FEJ 的一致性保证。

**默认值**（GTSAM `ISAM2Params.h`）：
- `relinearizeThreshold = 0.1`（标量）——大致是 "旋转 0.1 rad 或 平移 0.1 m"；
- 也可按变量符号分别设：`FastMap<char, Vector>`，对 pose (`X`) 设 `Vector6(0.1,0.1,0.1,0.1,0.1,0.1)`；
- `relinearizeSkip = 10`：每 10 次 `update()` 才触发一次 relin 检查（默认；实时系统常改为 1）。

**实测影响**（Kaess 2012 Fig. 9）：$\beta=0.1$ 对 $\chi^2$ 几乎无影响，受影响矩阵元数下降到 batch 的 5–15%。

**与 FEJ（First Estimate Jacobian）的关系**：fluid relinearization 不是 FEJ。它是增量 Gauss-Newton 的**选择性重线性化启发式**：变化大的变量刷新线性化点，变化小的变量沿用旧线性化点，从而减少受影响矩阵元。FEJ 的目标是固定特定 Jacobian 求值点以维护不可观测子空间；iSAM2 的目标是高效更新 Bayes 树。二者都涉及"线性化点"，但不能把 iSAM2 的重线性化直接理解为一致性修正。

---

### §C.10 Wildfire 传播（Partial State Update） ⭐⭐⭐

**动机**：批量 GN 解完 $R\delta = d$ 后一次更新所有变量；iSAM2 只沿"受影响路径"向下传播。

```python
def partial_solve(T, α):                   # 从根开始
    def visit(clique C = F:S):
        δ_F_new = solve_local(P(F|S), δ_S) # 团内回代（Eq. 12）
        changed_F = {j ∈ F : |δ_F_new[j] - δ_F[j]| > α}
        δ[F] = δ_F_new
        for child D of C:
            if (D.separator ∩ changed_F) != {}:
                visit(D)                  # 野火蔓延到子树
            # else: 子树保持旧 δ（野火熄灭）
    visit(T.root)
```

**running intersection property** 给出野火策略的结构基础：若当前团的 separator 变量和条件密度都几乎没有改变，下游子树的增量通常也很小，可以停止传播。这里是数值启发式而非严格等式；阈值过大时会牺牲精度，阈值过小则接近全量回代。

**GTSAM 参数**：`ISAM2GaussNewtonParams::wildfireThreshold = 0.001`（默认）；Dogleg 版默认 $10^{-5}$。

**收敛性**：单次 `update()` 内的 wildfire 并不等价于 batch GN；但多次 `update()` 之后，wildfire 将累积传播到全树。Kaess 2012 §IV-C 证明：在每次 update 内部**迭代** fluid relinearization 直到 $J=\emptyset$，等价于 batch GN 的若干步。

**退化条件**：当新回环导致大范围的变量变动（$\delta$ 遍及全树），wildfire 就"烧到根再烧到每片叶"——等价于 batch GN。这就是 **iSAM2 最坏情况 = batch** 的本质。

---

### §C.11 增量边缘化（Incremental Marginalization） ⭐⭐⭐

#### 叶团的零开销边缘化

若要边缘化的 key 集合 $K$ **正好是叶团的全部 frontal**，直接删除该叶即可：

$$\int P(\Theta)\,dF_k = \int P(F_k\mid S_k)\,dF_k \cdot \prod_{j\neq k}P(F_j\mid S_j) = \prod_{j\neq k}P(F_j\mid S_j).$$

**分隔集 $S_k$ 的信息已完全位于父团的 $P(F_{\pi(k)}\mid S_{\pi(k)})$**，无需任何重新计算。

#### 非叶变量：必须先"旋转到叶"

通过 `constrainedKeys` 的 Group 机制：

```cpp
// GTSAM IncrementalFixedLagSmoother::createOrderingConstraints
FastMap<Key,int> constrained;
for (Key k : allKeys)              constrained[k] = 1;   // 正常 Group
for (Key k : marginalizableKeys)   constrained[k] = 0;   // 优先消元 → 落到叶
isam.update(newFactors, newValues, FactorIndices(), constrained);
isam.marginalizeLeaves(marginalizableKeys);
```

**COLAMD 会在 Group 0 内部做最小度排序、但保证 Group 0 全部消元在 Group 1 之前**——即被边缘化的变量最早消元（落到叶端），然后真正删掉。

> **编程陷阱：`constrainedKeys` 组号反了导致 fill-in 爆炸或边缘化失败。** 若把要边缘化的 key 设为 Group 1（高优先级保留），现有 key 设 Group 0，则边缘化目标反而被推向根（最后消元），形成巨大 separator，不仅无法删叶还会让 Bayes 树退化。正确做法：边缘化 key = Group 0（最先消元，落到叶），其余 = Group 1。照搬 `IncrementalFixedLagSmoother::createOrderingConstraints` 的逻辑即可。

#### `IncrementalFixedLagSmoother` = iSAM2 + 定期边缘化

文件：`gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.{h,cpp}`（4.3 后迁入主 gtsam）。

工作流：

1. 维护 `KeyTimestampMap`：每次 `update` 写入新 key 的时间戳。
2. 找超时 key：`timestamp < now - lag` → `marginalizableKeys`。
3. 构造 `constrainedKeys`（Group 0 = 超时 key，Group 1 = 其余）。
4. `isam_.update(factors, values, timestamps, constrainedKeys, ...)`。
5. `isam_.marginalizeLeaves(marginalizableKeys)`。
6. `eraseKeysBefore(timestamp)` 清 timestampKeyMap。

#### SmartFactor 与边缘化的冲突（GTSAM Issues #595, #1101, #1976）

**SmartFactor**（如 `SmartProjectionPoseFactor`）把 3D landmark **按需 Schur 消去**——因子只显式连接 pose keys，内部维护动态的观测列表。这与 `marginalizeLeaves` 的假设冲突：

1. `marginalizeLeaves` 假设受影响团的 key 顺序使得"要边缘化的 key 都排在被保留 key 之前"。SmartFactor 每次 linearize 时内部重排 keys → 该假设可能破坏（Issue #1101：*"the logic for splitting a clique assumes that keys are ordered ... This is not always true"*）。
2. SmartFactor 内部的 Schur 补让 landmark 隐式依赖多个 pose → 该 landmark **不是叶**、也不在显式 key 列表中，但其信息散布各团，marginalize 时重算冲突。
3. `cacheLinearizedFactors = true` 让旧 Jacobian 被复用，但 SmartFactor 的 Jacobian pattern 会随观测变化 → 缓存失效时不易察觉。

**解决策略**：
- 边缘化前把相关 SmartFactor **显式 linearize 为 `JacobianFactor`**（"固化"），冻结 key 顺序；
- 或 `params.cacheLinearizedFactors = false`；
- Kimera-VIO 采用第一种：`deleteOldSmartFactors` 后 `marginalize`。

---

### §C.12 与 batch GN/LM 的关系 ⭐⭐

> **思维陷阱：以为 iSAM2 内置了 LM 阻尼。** 这是使用 GTSAM 最常见的误解之一。iSAM2 默认使用的是纯 Gauss-Newton，没有任何阻尼参数 $\lambda$。这意味着在大回环或坏初值场景下，单次 `update()` 可能因为 GN 步过大而跳出正确 basin。如果你在 iSAM2 回环后发现轨迹"抖了一下然后恢复"或"抖了一下然后发散"，第一件事应该检查是否需要多次空 `update()` 或切换到 `ISAM2DoglegParams`。

**iSAM2 不同于 LM**。它没有阻尼参数 $\lambda$；默认 **Gauss-Newton**（`ISAM2GaussNewtonParams`）。那么非线性失败时怎么办？

| 失败模式 | iSAM2 应对 |
|---------|-----------|
| 小扰动、好初值 | 正常收敛（二次收敛率） |
| 大回环后多变量偏离线性点 | 单次 `update()` 不够；连续跑多次（LIO-SAM 回环后跑 6 次） |
| 严重非线性 / 坏初值 | 切换 `ISAM2DoglegParams`：信赖域 Powell dogleg（Rosen et al. 2012），在 Bayes 树上实现 |
| 病态 Hessian | `factorization = QR` 代替 `CHOLESKY`（慢 ~3× 但稳健） |
| 外点型噪声 | 配 `noiseModel::Robust(Cauchy/Huber/DCS)` 或 `GncOptimizer` + 批量 |

**为什么 SLAM 中通常收敛**：
- 里程计 / scan-matching / IMU 预积分提供**位姿好初值**；
- Landmark 由首次观测 back-project 得良好初值；
- 回环前误差在局部近似范围内；
- Fluid relinearization + 多次 update 足以拉回最优；
- 保底方案 RISE（Rosen-Kaess-Leonard, TRO 2014）在 iSAM2 之上加 trust region。

**收敛速率的严格分析**：对增量 GN 的每次 update，在良好线性化点附近（$\|\delta\|$ 小）��GN 具有**二次收敛率**（Nocedal-Wright *Numerical Optimization* 2006, Thm 10.2）：

$$\|\delta^{(k+1)}\| \le C\|\delta^{(k)}\|^2$$

其中常数 $C$ 取决于残差的 Hessian。在 iSAM2 中，每次 `update()` 等价于一步 GN（或 Dogleg），因此：

- **纯里程计区间**：$\|\delta\|$ 很小（初值来自预积分，误差 $\sim 10^{-2}$ m/rad），一次 update 足够
- **回环闭合后**：$\|\delta\|$ 可能很大（如 1m 级的闭合误差），需要多次 update 才能收敛
- **多次空 update 的数学含义**：每次空 update 在当前 $\Theta$ 处重新线性化并求解，等价于 GN 的多次迭代

LIO-SAM 回环后跑 6 次空 update 的经验来自：闭合误差通常 $<5$ m，6 次二次收���可把误差降到 $5\times(C\cdot 5)^{2^6}\approx 10^{-10}$ m（假设 $C\approx 0.1$），远低于传感器精度。

**Dogleg 信赖域的数学形式**：当 GN 步 $\delta_{gn}$ 超出信赖域半径 $\Delta$ 时，Powell Dogleg 取

$$\delta_{dl} = \begin{cases}
\delta_{gn} & \text{if } \|\delta_{gn}\|\le\Delta \\
\alpha\delta_{sd} + (1-\alpha)\delta_{gn} & \text{interpolation on the dogleg path}
\end{cases}$$

其中 $\delta_{sd}=-\alpha_{sd}J^\top r$（梯度方向的 Cauchy 点），$\alpha$ 由 $\|\delta_{dl}\|=\Delta$ 确定。在 Bayes 树上实现 Dogleg 需要存储额外的 `deltaNewton_` 和 `RgProd_`（梯度投影），这就是 `ISAM2::deltaNewton_` 成员的用途。

**Rosen-Kaess-Leonard RISE (TRO 2014)**：在 iSAM2 之上加信赖域控制——如果单步 update 使非线性���差增加，缩小信赖域并重试。这比切换到完整 Dogleg 更轻量（不需要每步都计算 Cauchy 点），但代价是偶尔触发 batch-like 行为。

> **反事实推理——如果 iSAM2 用 LM 而非 GN 会怎样？** LM 需要调节阻尼参数 $\lambda$：成功则减小，失败则增大。但 iSAM2 的 Bayes 树结构假设 Hessian 不变（只做局部更新）—��每次改变 $\lambda$ 等于修改所有因子的"虚拟 prior"（$\lambda I$ 加在对角），这破坏了局部性，使每步必须 relinearize 全图。因此 iSAM2 选择 GN（无 $\lambda$）作为默认，在需要鲁棒性时用 Dogleg（信赖域）而非 LM（阻尼矩阵）。这是**算法-数据结构协同设计**的典型例子：数据结构的局部性假设约束了可用的优化策略。

---

### §C.13 复杂度分析 ⭐⭐⭐

#### 理论界（Kaess 2012 §IV-C）

- **完全批量 Cholesky**：2D pose graph $O(N^{3/2})$，3D BA $O(N^2)$——来自 **Lipton-Rose-Tarjan 1979 嵌套剖分**（对平面图/网格图）。
- **iSAM2 每步**：$O(|\text{受影响团}|^{1.5})$；典型 SLAM 图受影响团数 $\approx$ 路径长度 = separator 长度 = $O(\sqrt N)$（2D）或 $O(N^{2/3})$（3D）。
- **结论（Kaess 2012, §IV-C）**：*"this bound does not depend on the number of loop closings"*——只要回环保持局部，每步仍 $O(\sqrt N)$。

#### Lipton-Rose-Tarjan 嵌套剖分定理的直觉

这个 1979 年的定理是理解 SLAM 求解器复��度的数学基石。核心思想如下：

**平面分隔符定理（Planar Separator Theorem, Lipton-Tarjan 1979）**：任何 $N$ 节点平面图都存在一个大小为 $O(\sqrt{N})$ 的**分隔符集合** $S$，使得删除 $S$ 后图分裂为两��大小各不超过 $\frac{2}{3}N$ 的子图。

**对 SLAM 的意义**：2D pose graph 本质上是平面图（或近似平面图——少量回环跨越平面不影响渐近行为）。当我们对信息矩阵做 Cholesky 分解时，消元一个变量 $x_i$ 会在其所有邻居之间引入 fill-in 边。嵌套剖分排序（Nested Dissection）递归地找分隔符、先消元两侧的子图、最后消元分隔符——这保证了 fill-in 被控制在 $O(N\log N)$ 个非零元内，总 Cholesky 运算量为 $O(N^{3/2})$。

**推广到 3D**：3D 网格图的分隔符大小为 $O(N^{2/3})$（George 1973），对应 Cholesky 复杂度 $O(N^2)$。iSAM2 每步只消元受影响子集（路径长度 $\approx$ 分隔符大小），因此每步为 $O(\sqrt{N})$（2D）或 $O(N^{2/3})$（3D）。

**为什么这个界"不取决于回环数"**：回环因子 $f(x_i, x_j)$ 影响的 Bayes 树路径是从 $x_i$ 和 $x_j$ 到它们的最低公共祖先（LCA）。对**局部回环**（$x_j$ 在 $x_i$ 附近），LCA 很浅，路径很短。只有**全局回环**（如起点-终点闭合）才会让路径延伸到根。多数实际 SLAM 场景的回环是局部的，因此平均每步远低于最坏情况。

**经验验证**：Kaess 2012 Fig. 11 在 M3500 数据集（3500 poses, 5453 edges）上统计每步受影响 clique 数，经验拟合为 $\approx 3.2\sqrt{N}$——与理论预测高度吻合。

#### 对比表

| 方法 | 每步复杂度 | 空间 | 备注 |
|------|----------|------|------|
| **EKF-SLAM** | $O(N^2)$ | $O(N^2)$ | 稠密 $\Sigma$ |
| **iSAM1 (Givens)** | $O(\text{fill})$ + 周期性 batch 尖峰 | $O(\text{nnz}(R))$ | 批量重排序周期 $\sim 100$ 帧 |
| **iSAM2 2D pose graph** | $\approx O(\sqrt N)$ | $O(\text{nnz}(R))$ | 好局部性下 |
| **iSAM2 3D BA** | $\approx O(N^{2/3})$ | $O(\text{nnz}(R))$ | 同上 |
| **iSAM2 最坏情况** | $O(N^{3/2})$ | 同上 | 全图回环 = batch |
| **滑窗 BA (VINS-Mono)** | $O(k^3), k$ 窗长 | $O(k^2)$ | 显式 Schur 边缘化 |
| **Batch LM (g2o)** | $O(N^{3/2})$ 每次 | 同上 | 全图重优化 |

> **跨领域类比**：iSAM2 的复杂度结构类似于数据库的 B 树索引更新。在 B 树中，插入一条记录最坏需要分裂 $O(\log N)$ 个节点，但平均只影响 $O(1)$ 个���节点。同理，iSAM2 中添加一个里程计因子平均只影响 $O(1)$ 个叶团；添加一个回环因子影响 $O(\sqrt{N})$ 个团（路径到 LCA）。两者都利用了"树状层次结构把局部操作控制在子树内"的设计原理。

---

### §C.14 GTSAM ISAM2 API ⭐⭐

**头文件**：`gtsam/nonlinear/ISAM2.h`、`ISAM2Params.h`、`ISAM2Result.h`、`ISAM2-impl.h`。

#### ISAM2Params 关键字段

| 字段 | 默认 | 含义 |
|------|------|------|
| `optimizationParams` | `ISAM2GaussNewtonParams()` | `boost::variant<GaussNewtonParams, DoglegParams>` |
| `relinearizeThreshold` | `0.1` | double 或 `FastMap<char,Vector>`（分符号设） |
| `relinearizeSkip` | `10` | 每多少 `update()` 检查一次 relin |
| `enableRelinearization` | `true` | 全局开关 |
| `evaluateNonlinearError` | `false` | 若 true 则 `ISAM2Result.errorBefore/After` 有值 |
| `factorization` | `CHOLESKY` | `{CHOLESKY, QR}` |
| `cacheLinearizedFactors` | `true` | 缓存 Jacobian；SmartFactor 场景建议 false |
| `findUnusedFactorSlots` | `false` | 删因子后复用 slot |
| `enableDetailedResults` | `false` | 填 `ISAM2Result::detail` |
| `enablePartialRelinearizationCheck` | `false` | 只查部分子树 |

#### update 完整签名

```cpp
ISAM2Result ISAM2::update(
    const NonlinearFactorGraph& newFactors            = {},
    const Values&               newTheta              = {},
    const FactorIndices&        removeFactorIndices   = {},
    const std::optional<FastMap<Key,int>>&  constrainedKeys = {},
    const std::optional<FastList<Key>>&     noRelinKeys     = {},
    const std::optional<FastList<Key>>&     extraReelimKeys = {},
    bool                        force_relinearize     = false);
```

#### ISAM2Result 字段

- `variablesRelinearized`, `variablesReeliminated`, `factorsRecalculated`, `cliques`；
- `newFactorsIndices`（新 slot 号，删除时必需）；
- `errorBefore` / `errorAfter`（仅 `evaluateNonlinearError=true`）；
- `detail`（仅 `enableDetailedResults=true`）。

#### 其他核心方法

```cpp
Values calculateEstimate() const;                       // 全量估计
template<class T> T calculateEstimate(Key) const;       // 单变量
Matrix marginalCovariance(Key) const;                   // 单变量边缘协方差
void marginalizeLeaves(const FastList<Key>& leafKeys,
                       FactorIndices* marginalFactors = nullptr,
                       FactorIndices* deletedFactors  = nullptr);
const NonlinearFactorGraph& getFactorsUnsafe() const;
const VectorValues& getDelta() const;
const Values& getLinearizationPoint() const;
const VariableIndex& getVariableIndex() const;
```

#### C++ 最小示例

```cpp
#include <gtsam/nonlinear/ISAM2.h>
using namespace gtsam;

ISAM2Params p;
p.relinearizeThreshold   = 0.1;
p.relinearizeSkip        = 1;
p.factorization          = ISAM2Params::CHOLESKY;
p.cacheLinearizedFactors = true;
p.findUnusedFactorSlots  = true;
p.evaluateNonlinearError = true;
// Dogleg 更鲁棒
ISAM2DoglegParams dl; p.setOptimizationParams(dl);

ISAM2 isam(p);
NonlinearFactorGraph g; Values v0;
// ... 添加因子和初值
auto res = isam.update(g, v0);
Values est = isam.calculateEstimate();
Matrix C   = isam.marginalCovariance(X(k));
```

#### Python 绑定（gtsam ≥ 4.2）

```python
import gtsam
from gtsam.symbol_shorthand import X

params = gtsam.ISAM2Params()
params.setRelinearizeThreshold(0.1)
params.setRelinearizeSkip(1)
isam = gtsam.ISAM2(params)

graph = gtsam.NonlinearFactorGraph()
values = gtsam.Values()
# ... 构造因子和初值
result = isam.update(graph, values)
estimate = isam.calculateEstimate()
cov      = isam.marginalCovariance(X(0))
```

---

### §C.15 ISAM2 内部数据结构 ⭐⭐⭐

| 成员 | 类型 | 作用 |
|------|------|------|
| `bayesTree` | `BayesTree<GaussianBayesTreeClique>` | Bayes 树主体 |
| `theta_` | `Values` | 当前线性化点 $\Theta$ |
| `delta_` | `VectorValues` | 当前增量 $\delta$ |
| `deltaNewton_`, `RgProd_` | `VectorValues` | Dogleg 专用：Newton 步 + 梯度投影 |
| `variableIndex_` | `VariableIndex` | 因子→变量的反向索引 |
| `deltaReplacedMask_` | `KeySet` | 标记本轮已 relin 的变量 |
| `nonlinearFactors_` | `NonlinearFactorGraph` | 原始因子图存档（供重线性化） |
| `linearFactors_` | `GaussianFactorGraph` | 缓存的线性化因子（若 `cacheLinearizedFactors=true`） |
| `fixedVariables_` | `KeySet` | 已 marginalize、不允许 relin 的变量 |

**`GaussianBayesTreeClique`** 每个节点含：
- `GaussianConditional`：条件密度 $P(F_k\mid S_k)$，等价于 $R$ 矩阵的行块 $[R_{FF}\,R_{FS}]$ + RHS $d_F$；
- `children`：子团指针；
- `cachedFactor` / `cachedSeparatorMarginal`：缓存边缘因子，用于 orphan 挂回。

---

### §C.16 LIO-SAM 中的 iSAM2 使用 ⭐⭐

**文件**：`LIO-SAM/src/mapOptmization.cpp`（~1779 行，commit master）。

#### ISAM2 配置（构造函数内）

```cpp
ISAM2Params parameters;
parameters.relinearizeThreshold = 0.1;   // 默认
parameters.relinearizeSkip      = 1;     // 比默认 10 激进 10×
isam = new ISAM2(parameters);
```

没设 `factorization`（= CHOLESKY）、没设 Dogleg → **默认 GaussNewton**。前端 `imuPreintegration.cpp` 的 `ISAM2 optimizer;` 默认构造。

#### 每帧流程（`saveKeyFramesAndFactor()` ~行 1333–1375）

```cpp
addOdomFactor();        // PriorFactor on X(0) + BetweenFactor<Pose3>(X(i-1),X(i))
addGPSFactor();         // GPSFactor 或 PriorFactor<Pose3>（仅约束平移）
// addLoopFactor() 在 correctPoses() 路径
isam->update(gtSAMgraph, initialEstimate);
isam->update();         // 第 2 次（无新因子，仅再跑一轮）

if (aLoopIsClosed) {    // 回环后
    isam->update(); isam->update(); isam->update();
    isam->update(); isam->update();   // 共 6 次
}
gtSAMgraph.resize(0);
initialEstimate.clear();
isamCurrentEstimate = isam->calculateEstimate();
```

#### 因子类型

- `gtsam/slam/PriorFactor.h` —— 首帧 + GPS 平移先验；
- `gtsam/slam/BetweenFactor.h` —— 相邻关键帧 + ICP 回环 (`Pose3`)；
- `gtsam/navigation/GPSFactor.h` —— WGS84→ENU 后 3D 位置；
- `gtsam/navigation/CombinedImuFactor.h` —— IMU 预积分（在前端）。

#### **从不调用 `marginalizeLeaves`**

LIO-SAM **持全轨迹在 iSAM2**，随地图增大 `calculateEstimate()` 线性增长——这就是 README 建议 `rosbag play -r 1` 的根因（"heavy iSAM optimization"）。

---

### §C.17 Kimera-VIO vs ORB-SLAM3 的 BA 策略 ⭐⭐⭐

| 组件 | 求解器 | 策略 |
|------|-------|------|
| **Kimera-VIO BackEnd** | iSAM2 via `gtsam::IncrementalFixedLagSmoother` | 固定延迟（~1 秒）平滑；`relinearizeThreshold=0.01`，`relinearizeSkip=1`，`cacheLinearizedFactors=1`；用 `SmartStereoProjectionPoseFactor` |
| **Kimera-RPGO** | **非 iSAM2**：LM（默认）/ GN + `GncOptimizer` | 批量 PGO + PCM 外点剔除 + GNC |
| **LIO-SAM** | iSAM2 纯用 | 不做 marginalize，保留全图 |
| **ORB-SLAM3 Tracking** | g2o motion-only BA | 6-DoF pose 优化当前帧 |
| **ORB-SLAM3 Local BA** | g2o LM + Schur 补 | 共视图（covisibility）局部 BA |
| **ORB-SLAM3 Loop Closing** | g2o Sim(3) PGO + Full BA | 三段式：Sim(3) 对齐 → Essential Graph PGO → Full BA |

#### 为什么 ORB-SLAM3 不用 iSAM2？

1. **拓扑剧烈变动**：相机-landmark 可见性随 tracking 随机变化，iSAM2 的 `cacheLinearizedFactors` 假设因子稳定；频繁 remove+reinsert 代价高。
2. **问题规模**：ORB-SLAM3 可持万级 MapPoint + 千级 KF，相机-点结构用 **Schur 补** 比一般消元快得多（$O(N_{KF}^3)$ vs $O(N^{3/2})$）。
3. **回环策略不同**：ORB-SLAM3 **Essential Graph（稀疏子图） + Sim(3) PGO + Full BA** 三段式，把重投影 BA 与位姿图解耦；iSAM2 在同一张图中处理大回环反而容易触发数值病态。
4. **边缘化语义**：滑窗 VIO 想保留旧信息作先验；ORB-SLAM3 认为"忘掉比记错好"，iSAM2 的"全图保留"优势在它的设计里不必要。
5. **工程惯性**：ORB-SLAM 自 2014 绑 g2o，自定义 `EdgeSE3ProjectXYZ` 等；切 iSAM2 重写收益不明。

**结论**：**iSAM2 强在全轨迹一致增量平滑**（LIO-SAM/Kimera-VIO）；**不强在超大规模视觉 BA**（ORB-SLAM3 路线）。两者是不同问题的最优解。

---

### §C.18 Bayes 树上的并行推断（档位 4）

#### 子树条件独立

**性质**：在 Bayes 树中，给定 separator $S_k$，frontal $F_k$ 与非后代 separator 变量条件独立。因此 **sibling 子树可并行消元 / 并行回代**。

#### DDF-SAM

- **DDF-SAM**（Cunningham, Paluri, Dellaert, IROS 2010）：每个机器人维护局部 SAM，用 constrained factor graph 通过 Schur 边缘化生成"condensed"邻域因子并交换。
- **DDF-SAM 2.0**（Cunningham, Indelman, Dellaert, ICRA 2013）：引入 **anti-factor**（信息减法 / down-dating）避免重复计数，给出去中心化增量式平滑。

#### Certifiable 分布式 PGO

**Tian, Khosoussi, Rosen, How, T-RO 37(6):2137–2156, 2021** "Distributed Certifiably Correct Pose-Graph Optimization" (**DC²-PGO**)：

1. 采用稀疏 SDP 松弛（Briales-Gonzalez-Jimenez 2017），证明与 SE-Sync 的 dense 松弛有相同 exactness 保证；
2. Riemannian Block Coordinate Descent (RBCD) 在 product-of-manifolds 上分布式 Burer-Monteiro；
3. 分布式 dual certificate 验证 $S(X^\star)\succeq 0$；
4. 2021 T-RO King-Sun Fu Memorial Best Paper Honorable Mention。

#### Kimera-Multi

**Tian et al., T-RO 38(4):2022–2038, 2022**（Best Paper Award）：8 机器人、8 km 轨迹、实时户外；局部 Kimera-VIO + 语义 3D mesh + 分布式 place recognition + DC²-PGO + 分布式 GNC 外点剔除 + mesh deformation。

---

### §C.19 连续时间 SLAM 与 GP 先验（档位 4）

#### 核心结果

**Barfoot, Tong, Särkkä, RSS 2014** 把轨迹视作 1D Gauss 过程 $x(t)$，先验由 LTV-SDE 生成：

$$\dot x(t) = A(t) x(t) + v(t) + L(t) w(t),\qquad w(t)\sim\mathcal{GP}(0, Q_c \delta(t-t')).$$

**定理**：若先验由上述线性 SDE 生成，则在测量时间 $\{t_k\}$ 上，先验精度矩阵 $K^{-1}$ 为 **block-tridiagonal**（Markov），MAP 信息矩阵仍为"带状 + 测量稀疏"，标准 Cholesky 在 $O(N)$ 完成。

#### White-Noise-on-Acceleration (WNOA)

状态 $[p(t), \dot p(t)]$，$\ddot p = w$。离散化：

$$Q_k = \begin{bmatrix}\tfrac13\Delta t_k^3 Q_c & \tfrac12\Delta t_k^2 Q_c\\ \tfrac12\Delta t_k^2 Q_c & \Delta t_k Q_c\end{bmatrix},\quad \Phi_k = \begin{bmatrix}I & \Delta t_k I\\ 0 & I\end{bmatrix}.$$

GP 因子 $\phi_k = \|\Phi_k x_{k-1} - x_k\|^2_{Q_k^{-1}}$——正是常速度因子。Anderson-Barfoot IROS 2015 推广到 $SE(3)$（"Full STEAM Ahead"）；Mukadam et al. RSS 2017 给出增量版本，速度比裸 iSAM2 快约 3×。

#### STEAM 库

UToronto ASRL 维护的 C++ 库，实现上述 GP 因子，支持 GP 内插任意时刻状态，与 GTSAM/iSAM2 兼容。

---

### §C.20 Bayes 树 vs 其他增量方法对比 ⭐⭐⭐

记 $N$=状态数，$M$=测量数，$k$=滑窗长度，$L$=受影响子树大小（典型 $O(\sqrt N)$ 2D / $O(N^{2/3})$ 3D）。

| 方法 | 代表文献 | 时间（每步） | 空间 | 保留全图 | 回环 | 精度 | 难度 | 典型应用 |
|------|---------|------------|------|--------|------|------|------|---------|
| **iSAM2 (Bayes Tree)** | Kaess IJRR 2012 | $O(L)\approx O(\sqrt N)$ 2D | $O(\text{nnz}(R))$ | ✔ | ✔ 原生 | MLE 高（局部） | 中高 | GTSAM, LIO-SAM, Kimera-VIO |
| **iSAM1 (Givens QR)** | Kaess TRO 2008 | $O(\text{fill})$ + batch 尖峰 | $O(\text{nnz}(R))$ | ✔ | ✔ | 高 | 中 | 早期 SAM |
| **EKF-SLAM** | Smith-Self-Cheeseman 1987 | $O(N^2)$ | $O(N^2)$ | 当前状态 | ✔ 但一致性差 | 中 | 低 | 历史 |
| **VINS-Mono 滑窗 BA** | Qin 2018 | $O(k^3)$ | $O(k^2)$ | ✘ | 外挂 4-DoF PGO | 中高 | 中高 | AR/VR, MAV |
| **MSCKF** | Mourikis 2007 | $O(M_f^2)$ | $O(k)$ | ✘ | ✘（原版） | 中 | 中 | 手机 VIO |
| **ORB-SLAM3 Local BA** | Campos TRO 2021 | $O(k_c^3)$ | $O(k_c^2)$ | KF 池 | ✔ Essential + Full | 高 | 高 | 单/双/RGBD + IMU |
| **Batch LM (SqrtSAM)** | Dellaert-Kaess 2006 | $O(N^{3/2})$ 2D | $O(\text{nnz}(R))$ | ✔ | ✔ | 高 | 中 | 离线 BA |
| **SE-Sync** | Rosen IJRR 2019 | 低秩 SDP | $O(\text{nnz})$ | ✔（batch） | ✔ | **全局+证书** | 高 | 离线 PGO |
| **DC²-PGO / Kimera-Multi** | Tian TRO 2021/2022 | RBCD 本地 $O(N_i)$ | 分布 | ✔ | ✔ + 外点 | 全局+证书 | 很高 | 多机器人 CSLAM |

**说明**：iSAM2 的 $O(\sqrt N)$ 特指稀疏 2D pose graph 平均情况；最坏情况（密集回环）退化为 $O(N^{3/2})$。MSCKF 无法回环因其把 landmark 滑出后擦除。

---

### §C.21 核心教材/论文速查表

| 资源 | 章节 / 页 | 作用 |
|------|----------|------|
| Kaess PhD 论文 (GaTech 2008) | Ch.4–5 | iSAM1 完整推导 |
| Kaess et al. "iSAM" IEEE TRO 24(6):1365–1378, 2008 | §III-A–D | Givens 增量 QR |
| Kaess et al. "Bayes Tree" WAFR 2010, pp.157–173 | 全文 | Bayes 树首次提出 |
| Kaess et al. "iSAM2" IJRR 31(2):216–235, 2012 | §III, §IV, Alg. 2–8 | iSAM2 完整算法 |
| Dellaert & Kaess "Factor Graphs for Robot Perception" FnT 6(1-2):1–139, 2017 | §5 | 最新教材，含增量 |
| Davis *Direct Methods for Sparse Linear Systems* SIAM 2006 | Ch.3–4 | 消元树、超节点、COLAMD |
| Liu "Role of Elimination Trees" SIMAX 11(1):134–172, 1990 | Thm 2.4 | 消元树基本定理 |
| Lipton-Rose-Tarjan "Generalized Nested Dissection" SINUM 16(2):346–358, 1979 | Thm 1 | 2D $O(N^{3/2})$ |
| Barfoot *State Estimation for Robotics* 2ed 2024 | Ch.9 | 批量 SAM + GP |
| Rosen et al. "SE-Sync" IJRR 38(2-3):95–125, 2019 | §III–V | 全局最优证书 |

---

### §C.22 C++/Python 库映射

| GTSAM 类 | 文件 | 用途 |
|---------|------|------|
| `ISAM2` | `gtsam/nonlinear/ISAM2.h` | 主求解器 |
| `ISAM2Params` | `gtsam/nonlinear/ISAM2Params.h` | 配置 |
| `ISAM2Result` | `gtsam/nonlinear/ISAM2Result.h` | 返回值 |
| `ISAM2GaussNewtonParams` / `ISAM2DoglegParams` | 同上 | 内层优化 |
| `BayesTree<CLIQUE>` | `gtsam/inference/BayesTree.h` | Bayes 树模板 |
| `GaussianBayesTree` / `GaussianBayesTreeClique` | `gtsam/linear/GaussianBayesTree.h` | Gaussian 专化 |
| `EliminationTree` | `gtsam/inference/EliminationTree.h` | 消元树 |
| `SymbolicBayesTree` | `gtsam/symbolic/SymbolicBayesTree.h` | 纯结构版 |
| `VariableIndex` | `gtsam/inference/VariableIndex.h` | 因子→变量反向索引 |
| `IncrementalFixedLagSmoother` | `gtsam_unstable/nonlinear/` (4.3+ 主 gtsam) | 固定延迟平滑 |
| `BatchFixedLagSmoother` | 同上 | 批量固定延迟 |

| 外部库 | 关键文件 | iSAM2 使用模式 |
|-------|---------|-------------|
| LIO-SAM | `src/mapOptmization.cpp` 行 ~1333–1375 | 纯 iSAM2，回环后连跑 6 次 update |
| Kimera-VIO | `src/backend/VioBackend.cpp` | `IncrementalFixedLagSmoother` |
| Kimera-RPGO | `include/KimeraRPGO/RobustSolver.h` | **非 iSAM2**，LM + GncOptimizer |
| Hydra / Kimera-PGMO | `kimera_pgmo/` | 批量 LM |
| LAMP | `loop_closure/` + `pose_graph_solver/` | GTSAM LM 批量 |

---

### §C.23 常见陷阱（15 条）

1. **`relinearizeThreshold` 过松（0.1）** 在慢动作下 $\delta$ 几乎不越阈 → 地图越用越"懒"。**对策**：LiDAR/视觉设 0.01–0.1；分符号设。

2. **`relinearizeSkip` 过大**（默认 10）导致即使 $\delta$ 超阈也要等 10 次 update 才重线性化。**对策**：实时系统设 1（LIO-SAM/Kimera/Visual-ISAM2 均如此）。

3. **`marginalizeLeaves` 非叶报错** "Requesting to marginalize variables that are not leaves, the ISAM2 object is now in an inconsistent state so should no longer be used"——消元顺序错致目标 key 还挂子树。**对策**：配合 `constrainedKeys` Group 0 强制推向叶；或 `force_relinearize`；bug 复现时绕开批量边缘化。相关 Issue #1101：三处 bug（nullptr 检查缺失、`nToRemove` 错、clique 键顺序假设错）。

4. **SmartFactor 与边缘化冲突**（Issue #595 `InconsistentEliminationRequested`、#1101、#1976）：SmartProjectionPoseFactor 在 pose 被边缘化后仍引用该 key，因子拓扑变动与 `cacheLinearizedFactors=true` 不兼容。**对策**：`cacheLinearizedFactors=false`；或边缘化前删除并重建该 smart factor；Kimera-VIO 用 `deleteOldSmartFactors` → `marginalize`。

5. **`update(g, v)` 缺初值**：`ValuesKeyDoesNotExist` / `IndeterminantLinearSystemException`。**对策**：新因子引用的每个 key，要么已在 isam 里，要么必须出现在 `newTheta`；先 `insert` 再加因子。

6. **`constrainedKeys` 组号错致 fill-in 爆炸**：把要边缘化的 key 设 Group 1（高），现有 key 设 Group 0 → 边缘化 key 被推到根，消元后生成稠密 separator。**对策**：边缘化 key = Group 0，其余 = Group 1；照搬 `IncrementalFixedLagSmoother::createOrderingConstraints`。

7. **iSAM2 非 LM，大回环 GN 一步过头**：下次 update 发散。**对策**：① 多次 `update()`（LIO-SAM 6 次）；② 切 Dogleg；③ `force_relinearize=true`；④ 回环因子包 `noiseModel::Robust(Cauchy/DCS)`（SC-LIO-SAM 默认 Cauchy）。

8. **初值差 + wildfire 阈值致发散**：野火只更新 $\delta$ 大的子树；初值远离真值时小子树 $\delta$ 在阈下被跳过 → 估计冻结。**对策**：`wildfireThreshold ≤ 0`（全量回代）；或 `force_relinearize`。

9. **`errorBefore/errorAfter` 为空**：默认 `evaluateNonlinearError=false`。**对策**：开启（略拖速度）。

10. **`cacheLinearizedFactors` 权衡**：SmartFactor、可变维 OrientedPlane3Factor、GNC 权重变化等"Jacobian 结构动态变"场景会复用旧 Jacobian → 数值错。**对策**：此类因子关闭缓存；或每次 update 删旧 slot 插新。

11. **iSAM2 非线程安全**：`calculateEstimate()` 与 `update()` 并行会破坏 Bayes 树。**对策**：mutex 互斥；LIO-SAM 用 `std::mutex mtx`。

12. **GN vs Dogleg 选择**：GN 在好初值下最快；Dogleg 信赖域鲁棒但慢 1.3–1.5×。**建议**：外点多场景 Dogleg；纯 LIO 平稳用 GN（LIO-SAM 选择）。

13. **`PriorFactor` 太松致 Indeterminant**：首帧 $\sigma$=1e6、其余 odom，某些方向欠约束（GPSFactor 无 yaw、Pose3 z 自由）→ `IndeterminantLinearSystemException`（Issue #17, #561, #666）。**对策**：首帧 $\sigma$ 给小量（LIO-SAM: 1e-2 rad/m）；IMU gravity 约束 roll/pitch；缺 yaw 时用磁 / GPS 航向。

14. **`findUnusedFactorSlots=false` + 频繁 remove**：slot 单调增长，`getFactorsUnsafe()` 变慢。**对策**：长跑系统一律 `findUnusedFactorSlots=true`。

15. **IMU 偏置陪跑 + 回环**：回环只改 $X$，不更 $V/B$ → 下一帧 IMU 预积分与新 $X$ 不一致。**对策**：LIO-SAM `correctPoses()` 回环后同步刷 $X$ 并重置 IMU 预积分窗口。

---

### §C.24 学习时间预算

| 阶段 | 时间 | 建议内容 |
|------|------|--------|
| **档位 3（博士入学）** | **20–25 h** | 精读 Kaess 2012 IJRR 全文；Dellaert & Kaess FnT §5；跑通 GTSAM `examples/VisualISAM2Example.cpp` 与 Python `ISAM2_SmartFactorStereoExample.py`；完成 §C.25 自测题 1–5 |
| **档位 4（进阶）** | **+8–12 h** | Kaess 博士论文 Ch.4；Davis 2006 Ch.4；Barfoot RSS 2014（GP-SLAM）；阅读 LIO-SAM mapOptmization.cpp + Kimera-VIO VioBackend.cpp；实现 IncrementalFixedLagSmoother demo；尝试复现 Issue #1101 bug；完成 §C.25 自测题 6–8 |

---

### §C.25 自测题（≥ 8 道）

1. **画 4 节点 pose graph 的完整变换**：4 个 pose + 1 个回环 $f(x_1, x_4)$，消元顺序 $x_1, x_2, x_3, x_4$。画出：(a) 因子图；(b) Bayes 网；(c) Bayes 树；(d) $R$ 矩阵的稀疏模式。验证 $R$ 非零结构 = Bayes 树祖先关系。

2. **证明叶变量边缘化 = 删叶**：设 $F_k$ 是叶团 frontal。证明 $\int P(\Theta)\,dF_k = \prod_{j\neq k}P(F_j\mid S_j)$。关键步骤：利用 $\int P(F_k\mid S_k)\,dF_k = 1$ 与联合分解 $P(\Theta)=\prod_j P(F_j\mid S_j)$。

3. **GTSAM Python 实现增量 pose graph**：用 `gtsam.ISAM2` 实现 10 帧 Pose2 里程计 + 第 10 帧与第 1 帧回环。逐帧 `addFactor + update + calculateEstimate`。对比 `relinearizeThreshold ∈ {0.001, 0.01, 0.1, 1.0}` 的 ATE 与每步耗时。

4. **Fluid relinearization 的 tradeoff**：若 $\beta\to 0$，iSAM2 等价于什么？若 $\beta\to \infty$，等价于什么？请用 GN 迭代次数 + 受影响团数 两个维度解释。

5. **Wildfire 退化为 batch 的条件**：给出回环拓扑使得 wildfire 必须传播到全树。提示：回环涉及 Bayes 树两个遥远子树的 LCA = 根。

6. **IncrementalFixedLagSmoother + SmartFactor**：为什么 `SmartProjectionPoseFactor` 会让 `marginalizeLeaves` 失败？写出两种绕过方案（`cacheLinearizedFactors=false` / 边缘化前固化为 JacobianFactor），讨论其影响。

7. **iSAM2 vs VINS-Mono**：在 KITTI 07 序列上跑 LIO-SAM（iSAM2）与 VINS-Fusion（滑窗 BA）。记录每帧耗时与总 ATE。解释：为什么 iSAM2 在回环后精度更高，但滑窗 BA 在短序列更快？

8. **复杂度对比**：在 2D pose graph（M3500, $N=3500$ pose、$M\approx 5500$ 约束）与 3D BA（KITTI 06, $N\approx 10^4$ pose）上，用 GTSAM ISAM2 统计每步受影响团数与 Cholesky flops。验证 $O(\sqrt N)$ vs $O(N^{2/3})$ 的经验复杂度标度。

9. **（档位 4）** 证明 Lipton-Rose-Tarjan 定理在 2D pose graph 上给出 $O(N^{3/2})$ batch Cholesky bound。提示：pose graph 近似平面图 + 平面分隔符定理。

10. **（档位 4）** 在 Bayes 树上实现并行野火回代（使用 OpenMP 或 TBB）。测量 sibling 子树并行带来的 speedup。

---

### §C.26 从滤波链到 Bayes 树：增量平滑的范式切换 ⭐⭐⭐

5-B 已经把滤波和平滑放在同一个后验上比较。

这里进一步说明：Bayes 树不是凭空出现的数据结构，而是平滑后验在消元顺序下的概率骨架。

先看最简单的链式系统：

$$
x_0 \rightarrow x_1 \rightarrow x_2 \rightarrow \cdots \rightarrow x_T.
$$

若只保留当前边际，Kalman 滤波递推为

$$
p(x_k\mid z_{1:k})
\propto
p(z_k\mid x_k)
\int p(x_k\mid x_{k-1})p(x_{k-1}\mid z_{1:k-1})\,dx_{k-1}.
$$

这个积分每次只向前传一条消息。

若保留全轨迹并做 MAP，得到的是

$$
\min_{x_{0:T}}
\frac12\|x_0-\bar x_0\|_{P_0^{-1}}^2
+\frac12\sum_{k=1}^{T}\|x_k-f(x_{k-1},u_{k-1})\|_{Q^{-1}}^2
+\frac12\sum_{k=1}^{T}\|z_k-h(x_k)\|_{R^{-1}}^2.
$$

线性化后，Hessian 是块三对角。

对这个块三对角系统做 Cholesky，得到的消元树就是一条链。

这条链的 Bayes 树也退化为一条链。

因此：

| 结构 | 线性高斯链中的表现 |
|---|---|
| Kalman filter | 从左到右的前向消息 |
| RTS smoother | 前向滤波 + 从右到左回代 |
| Batch Cholesky | 分解块三对角信息矩阵 |
| Bayes tree | 一条由条件密度组成的链 |
| iSAM2 wildfire | 链上只回代发生变化的一段 |

这给出一个重要的直觉：

> **本质洞察**：Bayes 树可以看作 RTS 平滑在一般稀疏图上的推广。链式图的"前向滤波、后向平滑"只有一条路径；一般 SLAM 图有许多路径，Bayes 树把这些路径组织成可局部更新的条件独立子树。

二维 pose graph 不是链。

回环会把远处节点连接起来，使消元后出现团。

但消元后的联合分布仍可写为条件密度乘积：

$$
P(\Theta)=\prod_k P(F_k\mid S_k).
$$

这就是 Bayes 树。

其中 $F_k$ 是该团真正要回代求解的 frontal 变量，$S_k$ 是它依赖的 separator 变量。

给定父团 separator 的值，子树与其余图条件独立。

这正是增量更新能够局部化的概率原因。

##### 一个小型链例子

设消元顺序为 $x_0,x_1,x_2,x_3$。

消元后得到

$$
P(x_0,x_1,x_2,x_3)
=
P(x_0\mid x_1)P(x_1\mid x_2)P(x_2\mid x_3)P(x_3).
$$

Bayes 树为

```text
{x3}
  |
{x2 | x3}
  |
{x1 | x2}
  |
{x0 | x1}
```

新量测只连接 $x_3$ 与新变量 $x_4$ 时，受影响区域在链尾。

##### 从链到树：回环如何破坏链式结构

现在在链的基础上加一条回环因子 $f(x_0, x_3)$。

消元 $x_0$ 时，由于 $x_0$ 同时连接 $x_1$（里程计）和 $x_3$（回环），产出因子覆盖 $\{x_1, x_3\}$——这就是 **fill-in**。

消元后的 Bayes 树不再是单链，而是：

```text
{x3}
  ├── {x2 | x3}
  │     └── {x1 | x2, x3}   ← fill-in 造成 x3 出现在 separator
  │            └── {x0 | x1, x3}
  ...
```

注意 $x_3$ 穿透了多层——从根一直延伸到叶的 separator。这就是为什么"全图回环"（连接第一帧和最后一帧）的代价最高：它迫使整条链的每个团都把首帧变量包含在 separator 中，Bayes 树退化为"每团都依赖根"的星形结构。

> **本质洞察**：回环在 Bayes 树上的代价**不取决于回环本身的测量质量**，而取决于**回环连接的两个变量在消元顺序中的距离**。
> 两个相邻帧的回环几乎免费（LCA 就是父团）；两个远帧的回环代价正比于路径长度。
> 这就是为什么 SLAM 系统对回环的处理策略差异巨大——频繁短距离回环（如走廊来回）很廉价，
> 而长距离闭合（如绕行一圈回到起点）需要特殊处理（多次 update、Dogleg、甚至 batch refinement）。

##### 二维 pose graph 的典型 Bayes 树形态

对一个机器人在 $L\times L$ 网格中走 $N$ 步的 2D pose graph：

- **无回环**：Bayes 树退化为链，高度 $H=N$，每团大小 $|C_k|=2$（当前 pose + 前一 pose 的 separator）
- **少量局部回环**：链上偶尔分叉，出现几个宽团（$|C_k|\approx 3$-5），高度仍 $\approx N$
- **嵌套剖分排序**（COLAMD/METIS）：树高 $H\approx\log N$，根团大小 $|C_{root}|\approx\sqrt{N}$（对应最大分隔符）
- **全连通回环**（最坏情况）：根团包含所有变量，树退化为一个团——等价于稠密 Cholesky

实际 SLAM 系统的 Bayes 树通常介于第二和第三种之间。COLAMD 排序的目标正是让树"又矮又宽"——减少每步 wildfire 传播的路径长度。

##### Bayes 树高度与 iSAM2 性能的关系

iSAM2 每步的受影响团数上界是**从受影响叶到根的最长路径**。因此：

$$\text{每步复杂度} \le O(\text{路径长度} \times \text{平均团大小}^2)$$

对 COLAMD 排序的 2D pose graph，路径长度 $\approx O(\log N)$-$O(\sqrt{N})$（取决于回环位置），团大小 $\approx O(\sqrt{N})$（根团最大）。组合给出平均 $O(\sqrt{N})$ 每步的经验复杂度。

这个分析也解释了为什么 **METIS 嵌套剖分在大图上优于 COLAMD**：METIS 显式优化分隔符大小，产出更平衡的树（高度更低、根团更小），而 COLAMD 只做贪心最小度排序，可能产出不平衡的深树。GTSAM 默认用 COLAMD（速度快），但在 $N>10^4$ 的大图上建议切换到 METIS。

##### 练习

1. 对 5 节点链 + 一条 $f(x_0, x_4)$ 回环，分别用"自然顺序"和"COLAMD 顺序"消元，画出两种 Bayes 树。比较根团大小和树高。
2. 设计一个 10 节点图使得 Bayes 树在某种消元顺序下退化为一个稠密团。
3. 在 GTSAM 中对 M3500 数据集分别用 COLAMD 和 METIS 排序，统计根团大小和每步平均受影响 clique 数。

新回环连接 $x_0$ 与 $x_4$ 时，影响会沿链一路传到根。

这就是 iSAM2 回环后可能退化为 batch 的最小例子。

##### 为什么不是维护协方差

EKF-SLAM 维护的是协方差 $\Sigma$。

若出现回环，协方差的交叉项会把所有变量耦合起来。

Bayes 树维护的是 square-root information 条件密度。

它保存的是稀疏因子 $R$ 的结构，而不是稠密 $\Sigma$。

从求解角度看：

$$
\Sigma = \Lambda^{-1}
$$

即便 $\Lambda$ 稀疏，$\Sigma$ 通常稠密。

因此增量平滑选择维护 $\Lambda$ 的平方根结构，而不是维护 $\Sigma$。

##### 工程上的范式切换

| 问题 | 滤波思路 | Bayes 树思路 |
|---|---|---|
| 新里程计 | 更新当前边际 | 添加局部因子，更新叶附近团 |
| 大回环 | 改当前状态并传播协方差 | 拆受影响 top，重消元并回代 |
| 旧状态需要修正 | 不自然，除非保存历史 | 原生支持 |
| 协方差查询 | 当前状态方便，历史困难 | 需局部 marginal covariance |
| 实时性 | 每步固定 | 平均局部，最坏 batch |

这解释了 LIO-SAM 一类系统的设计。

IMU 前端用 ESKF 传播高频状态。

关键帧后端用 iSAM2 保留全局轨迹。

前者解决高频预测，后者解决全局一致性。

##### 常见误区

| 误区 | 正确说法 |
|---|---|
| Bayes 树是另一种滤波器 | 它是全局平滑后验的条件分解 |
| iSAM2 每次只优化新变量 | 它会重消元受影响团，必要时影响旧变量 |
| 野火不更新就意味着变量不重要 | 只意味着本次 separator 变化低于阈值 |
| 增量一定比 batch 快 | 大回环、差初值、全图重线性化时会接近 batch |

##### 练习

1. 对 $x_0-x_1-x_2-x_3$ 链写出 Bayes net 分解，并画出 Bayes 树。
2. 在链尾添加 $x_4$，说明受影响团有哪些。
3. 添加回环 $f(x_0,x_4)$，说明为什么 LCA 变成根，更新范围扩大。

---

### §C.27 团内条件密度的线性代数：$P(F\mid S)$ 到局部回代 ⭐⭐⭐⭐

每个 Bayes 树团存储一个 Gaussian conditional。

把 frontal 变量记为 $F$，separator 变量记为 $S$。

消元后，该团的条件密度可写成

$$
P(\delta_F\mid \delta_S)
\propto
\exp\left(
-\frac12\|R_{FF}\delta_F+R_{FS}\delta_S-d_F\|^2
\right).
$$

这里 $R_{FF}$ 是上三角或块上三角矩阵。

给定父团已经算出的 $\delta_S$，该团的最优 frontal 增量为

$$
\boxed{
\delta_F
=
R_{FF}^{-1}(d_F-R_{FS}\delta_S).
}
$$

这就是 Bayes 树回代。

它与普通三角回代完全相同，只是按团块执行。

##### 从 QR 到条件密度

设某个团装配后的局部线性系统为

$$
A_F\delta_F + A_S\delta_S \approx b.
$$

对 $A_F$ 做 QR：

$$
A_F=Q
\begin{bmatrix}
R_{FF}\\0
\end{bmatrix}.
$$

左乘 $Q^\top$：

$$
\begin{bmatrix}
R_{FF}\\0
\end{bmatrix}\delta_F
+
\begin{bmatrix}
\bar A_S\\A'_S
\end{bmatrix}\delta_S
\approx
\begin{bmatrix}
\bar b\\b'
\end{bmatrix}.
$$

上半部分给出条件密度：

$$
R_{FF}\delta_F+\bar A_S\delta_S-\bar b.
$$

下半部分不再含 $\delta_F$，成为 separator 上的新因子：

$$
A'_S\delta_S-b'.
$$

这就是消元的代数全过程。

Bayes 树把上半部分存在当前团，把下半部分传给父团。

##### 与 Cholesky 的关系

若从正规方程出发，局部 Hessian 写成

$$
\begin{bmatrix}
H_{FF} & H_{FS}\\
H_{SF} & H_{SS}
\end{bmatrix}.
$$

消去 $F$ 后，父团收到的 update 是 Schur 补：

$$
H'_{SS}
=
H_{SS}-H_{SF}H_{FF}^{-1}H_{FS}.
$$

QR 视角避免显式形成 $H=A^\top A$，数值更稳。

Cholesky 视角更容易解释 fill-in。

GTSAM 中 `factorization=QR` 更稳定但更慢，`CHOLESKY` 更快但对病态系统更敏感。

选择不是理论偏好，而是数值条件的工程判断。

##### 团的 frontal 与 separator 为什么分开

设团 $C_k=\{x_1,x_2,x_5,x_9\}$。

若父团包含 $\{x_5,x_9\}$，则

$$
F_k=\{x_1,x_2\},\qquad S_k=\{x_5,x_9\}.
$$

团内条件密度是

$$
P(x_1,x_2\mid x_5,x_9).
$$

这意味着：只要父团给出 $x_5,x_9$ 的增量，当前团就能独立求 $x_1,x_2$。

兄弟团之间不需要互相通信，因为它们只共享父团 separator。

这正是 Bayes 树可并行的原因。

##### 局部回代的误差传播

回代公式中只有 $\delta_S$ 会影响子团。

若父团 separator 的变化很小：

$$
\|\delta_S^{new}-\delta_S^{old}\|<\alpha,
$$

子团新的 frontal 变化也受限于

$$
\|\delta_F^{new}-\delta_F^{old}\|
\le
\|R_{FF}^{-1}R_{FS}\|\cdot
\|\delta_S^{new}-\delta_S^{old}\|.
$$

Wildfire threshold 的直觉就在这里。

若 separator 变化小，整棵子树的变化通常也小，可以暂时不更新。

但这个界包含 $\|R_{FF}^{-1}R_{FS}\|$。

如果局部系统病态，这个放大因子可能很大，小 separator 变化仍可能造成大 frontal 变化。

因此在高精度离线优化或病态几何场景中，应把 wildfire threshold 设得更小，甚至设为 0 做全量回代。

##### 数值边界

| 现象 | 可能原因 | 建议 |
|---|---|---|
| `IndeterminantLinearSystemException` | $R_{FF}$ 奇异或近奇异 | 检查 gauge、先验、退化观测 |
| QR 能跑而 Cholesky 失败 | $J^\top J$ 病态放大条件数 | 切 QR 或增强白化尺度 |
| 回代后局部变量跳变 | separator 变化被放大 | 降低 wildfireThreshold |
| 团很大 | 排序不佳或大回环制造 separator | 用 constrained COLAMD/METIS |

##### 练习

1. 给定 $R_{FF}=\begin{bmatrix}2&1\\0&3\end{bmatrix}$、$R_{FS}=\begin{bmatrix}1\\2\end{bmatrix}$、$d_F=[1,0]^\top$，手算 $\delta_F$ 关于标量 $\delta_S$ 的表达式。
2. 对一个局部因子矩阵 $[A_F\ A_S]$ 做 QR，标出哪部分是 conditional，哪部分是传给父团的 factor。
3. 解释为什么 `factorization=QR` 在病态情况下更稳，但通常比 Cholesky 慢。

---

### §C.28 Fluid Relinearization 的数学细节：什么时候旧线性化点仍可信 ⭐⭐⭐⭐

iSAM2 的核心节省来自一个判断：

不是每个变量每一帧都需要重线性化。

设非线性残差为 $r_i(\Theta_i)$。

在旧线性化点 $\bar\Theta$ 处，有

$$
r_i(\bar\Theta\oplus\delta)
\approx
r_i(\bar\Theta)+J_i(\bar\Theta)\delta.
$$

如果当前增量 $\delta$ 很小，Taylor 余项近似为

$$
\frac12\delta^\top
\nabla^2 r_i(\bar\Theta)
\delta.
$$

余项是二阶小量。

这就是阈值规则的数学依据：

$$
\|\delta_j\| < \beta_j
\quad\Rightarrow\quad
\text{暂时保留旧 Jacobian}.
$$

但 $\beta_j$ 不能只设一个标量。

Pose3 的切向量包含旋转和平移：

$$
\delta\xi=[\omega_x,\omega_y,\omega_z,\rho_x,\rho_y,\rho_z]^\top.
$$

旋转 $0.1$ rad 与平移 $0.1$ m 的物理意义不同。

实际工程中更合理的是按变量类型设置阈值：

| 变量 | 建议阈值含义 |
|---|---|
| Pose3 旋转 | $0.01$-$0.1$ rad |
| Pose3 平移 | $0.01$-$0.1$ m |
| IMU bias gyro | 按随机游走 sigma |
| IMU bias acc | 按随机游走 sigma |
| landmark inverse depth | 按深度不确定性 |

GTSAM 支持 `FastMap<char, Vector>`，可以对 `X`、`V`、`B` 等符号分别给阈值向量。

##### 为什么需要先把 $\delta$ 吸收到线性化点

若某变量超过阈值，iSAM2 做

$$
\Theta_j \leftarrow \Theta_j\oplus\delta_j,\qquad
\delta_j \leftarrow 0.
$$

这一步不是状态更新的普通形式，而是重新定义局部坐标原点。

随后涉及该变量的因子在新 $\Theta_j$ 处重新线性化。

如果不吸收 $\delta$，继续在远离当前估计的切空间里线性化，会出现两个问题：

1. Lie 群 retraction 的局部坐标不再准确；
2. Jacobian 描述的是旧切平面，不再描述当前残差曲率。

这与 IEKF 的 iterated update 很像。

区别是 IEKF 对当前时刻反复重线性化，iSAM2 对整棵 Bayes 树中越界的变量局部重线性化。

##### RelinearizeSkip 的利弊

`relinearizeSkip=10` 表示每 10 次 update 才检查一次阈值。

这节省了遍历变量的成本。

但它也意味着某变量在第 1 次 update 后已经越界，仍可能拖到第 10 次才被刷新。

在线 LiDAR/VIO 通常设为 1，因为关键帧频率不高，且前端约束非线性较强。

大规模离线 pose graph 可设更大，以减少管理开销。

| 参数 | 变小 | 变大 |
|---|---|---|
| `relinearizeThreshold` | 更接近 batch GN，耗时更高 | 更懒，可能精度下降 |
| `relinearizeSkip` | 每帧检查，响应快 | 检查少，可能延迟刷新 |
| `wildfireThreshold` | 回代更完整 | 回代更少，可能局部冻结 |

##### 与鲁棒核权重变化的关系

鲁棒核和 GNC 会改变因子权重。

如果因子权重变化，但 iSAM2 复用缓存的线性因子，旧 Jacobian 与旧权重可能继续参与。

普通 Huber/DCS 在每次 linearize 时可重算权重，仍可与 iSAM2 配合。

GNC 不适合直接嵌入 iSAM2，是因为 GNC 的外层 $\mu$ 会让所有因子的权重调度同时改变。

这不是局部拓扑变化，而是全图目标函数变化。

因此 GNC 应作为 batch 或周期性后端使用。

##### 线性化误差的监控

GTSAM 默认不计算 nonlinear error，因为代价较高。

开发阶段应打开

```cpp
params.evaluateNonlinearError = true;
```

并观察：

$$
\Delta F = F_{\text{after}}-F_{\text{before}}.
$$

若单次 update 后 error 上升明显，有几种可能：

| 可能原因 | 处理 |
|---|---|
| GN 步过大 | 切 Dogleg 或多次 update |
| 阈值太松 | 降低 relinearizeThreshold |
| 外点回环 | 加 DCS/Huber 或先 PCM |
| 排序把关键变量推到根 | 检查 constrainedKeys |
| 初值太差 | batch LM/GNC/SE-Sync 初始化 |

##### 常见误区

| 误区 | 说明 |
|---|---|
| 阈值越小越好 | 会退化为 batch，实时性下降 |
| 阈值越大越快且无损 | 非线性残差会积累线性化误差 |
| 只需看最终 ATE | 中途 error 增长可能预示后端不稳定 |
| Dogleg 可替代重线性化 | Dogleg 控制步长，重线性化控制模型准确性 |

##### 练习

1. 对重投影残差写出 Taylor 余项的二阶量级，解释远点与近点哪个更怕旧线性化点。
2. 在 GTSAM 中分别设置 `relinearizeThreshold=0.001,0.1,1.0`，记录 `variablesRelinearized`。
3. 对同一数据集开启与关闭 `evaluateNonlinearError`，观察每帧耗时变化。

---

### §C.29 iSAM2 局部编辑的完整 worked example ⭐⭐⭐⭐

这一节把 `update()` 过程用一个具体图跑一遍。

考虑 6 个 pose 的链：

```text
x1 -- x2 -- x3 -- x4 -- x5 -- x6
```

当前 Bayes 树的简化形状为

```text
          {x5,x6}
          /     \
   {x3,x4 | x5}  {l6 | x6}
       /
 {x1,x2 | x3}
```

现在加入回环因子

$$
f_{\text{loop}}(x_2,x_6).
$$

第一步，找到涉及变量所在团：

| 变量 | 所在团 |
|---|---|
| $x_2$ | $\{x1,x2\mid x3\}$ |
| $x_6$ | 根团 $\{x5,x6\}$ |

第二步，从这两个团向根走，取路径并集。

受影响 top 为

```text
{x1,x2 | x3}
{x3,x4 | x5}
{x5,x6}
```

右侧子树 `{l6 | x6}` 不在 top 内，但它的父团被拆掉，因此临时成为 orphan。

第三步，`removeTop` 把 top 中所有团拆成一个因子集合。

这个因子集合包含：

1. top 内原条件密度转回的 Gaussian factors；
2. 新回环因子；
3. orphan 的 cached marginal factor，用于保持 `{l6 | x6}` 与新父团的连接。

第四步，在 top 相关变量的新线性化点处重算 Jacobian。

注意 orphan 子树内部不重线性化。

它只通过 cached factor 与 top 交互。

第五步，对 top 内变量做局部重排序。

这里可能得到新顺序：

$$
x_1,x_2,x_3,x_4,x_5,x_6.
$$

也可能因为回环强耦合而把 $x_2,x_6$ 推向根。

iSAM2 的启发式会尽量把最近受影响的变量放到排序末端，使它们靠近根，减少后续更新的 top 大小。

第六步，重新消元 top 因子集，生成新 Bayes 子树。

可能得到

```text
       {x2,x5,x6}
       /       \
{x1 | x2}   {x3,x4 | x5}
                 \
               orphan {l6 | x6}
```

第七步，把 orphan 按 separator 挂回。

若 orphan 的 separator 是 $\{x_6\}$，它必须挂到包含 $x_6$ 的最近团下。

第八步，从新子树根开始 wildfire 回代。

若 $\delta_{x6}$ 变化大，orphan 也会被访问。

若 $\delta_{x6}$ 变化小，orphan 保持旧解。

##### 这个例子说明了什么

| 观察 | 含义 |
|---|---|
| 新因子不一定只影响附近变量 | 回环可沿 Bayes 树路径影响到根 |
| orphan 不丢信息 | 它通过 cached marginal factor 保持与 top 的关系 |
| 局部重排序很关键 | 不重排会让未来 top 变大 |
| wildfire 只改数值不改结构 | 结构更新在 removeTop/eliminate/graft 阶段完成 |

##### 与 Batch 的差别

Batch 做法是：

1. 重新线性化所有因子；
2. 对全图重新排序；
3. 重新 Cholesky；
4. 全量回代。

iSAM2 做法是：

1. 只线性化 top 涉及因子；
2. 只对 top 重排序；
3. 只重消元 top；
4. 只沿变化传播回代。

当 top 远小于全树时，收益巨大。

当 top 接近全树时，iSAM2 退化为 batch，并多出管理开销。

##### 工程可观测指标

`ISAM2Result` 中这些字段可用于判断更新范围：

| 字段 | 含义 | 异常信号 |
|---|---|---|
| `variablesRelinearized` | 本次重线性化变量数 | 长期为 0 可能阈值太松 |
| `variablesReeliminated` | 本次重消元变量数 | 大回环后接近全图属正常 |
| `factorsRecalculated` | 重算线性因子数 | SmartFactor 变化会升高 |
| `cliques` | 受影响团数 | 持续增大说明排序或图结构恶化 |
| `errorBefore/After` | 非线性误差 | after 上升需检查初值与外点 |

##### 练习

1. 对上述 Bayes 树添加 $f(x_1,x_6)$，重新画 top 与 orphan。
2. 若 orphan separator 为 $\{x_4,x_6\}$，说明它应挂到哪个新团下。
3. 写伪代码统计每次 update 的 top 占全树比例，并用它判断是否需要周期性 batch reorder。

---

### §C.30 工程边界：什么时候 iSAM2 不是最合适的后端 ⭐⭐⭐

iSAM2 很强，但它不是所有状态估计问题的默认答案。

正确使用它需要知道边界。

##### 边界 1：目标函数频繁全局变化

GNC、可学习鲁棒权重、全局温度退火等方法会改变所有因子的权重。

这等价于每个因子的线性化都可能失效。

Bayes 树的局部性假设被破坏。

此时更适合 batch LM/GN 外层循环。

典型管线是：

```text
在线: iSAM2 + Huber/DCS
回环后: 导出 pose graph
离线/周期性: PCM -> GNC-TLS -> LM
再注入: 用优化结果重置 iSAM2
```

##### 边界 2：视觉 BA 中 landmark 拓扑高速变化

单目/双目视觉 SLAM 的 MapPoint 会频繁创建、删除、合并。

每个关键帧的可见性也在变化。

这让因子图结构比 LiDAR pose graph 更不稳定。

ORB-SLAM 系列选择 g2o 局部 BA + Schur 补，而不是 iSAM2，原因就在这里。

iSAM2 更适合 keyframe pose graph、VIO fixed-lag、LiDAR odometry + loop factors 这类拓扑相对稳定的问题。

##### 边界 3：强非线性与坏初值

iSAM2 默认 GN，没有 LM 阻尼。

若初值远离真值，单次 update 可能把线性模型用到错误区域。

可选策略：

| 场景 | 策略 |
|---|---|
| 大回环但外点少 | 连续多次 `update()` + `force_relinearize` |
| 初值较差 | 切 `ISAM2DoglegParams` |
| 外点不确定 | 回环先 PCM/DCS |
| 全局错位 | batch SE-Sync 或 GNC 初始化 |

##### 边界 4：边缘化变量不是叶

Fixed-lag smoothing 需要删旧变量。

Bayes 树只允许安全地删叶团 frontal。

若目标变量不是叶，需要用 `constrainedKeys` 把它们推到消元顺序前端。

若 SmartFactor 或动态因子破坏 key 顺序假设，`marginalizeLeaves` 可能失败。

工程上要么固化 SmartFactor，要么关闭缓存并重建相关因子。

##### 边界 5：协方差查询不是免费

iSAM2 的主结构是 square-root information。

查询某个变量的 marginal covariance 需要局部消元或回代。

若每帧查询大量变量协方差，耗时可能接近一次额外求解。

因此在线系统常只查询关键变量，或用近似边际。

##### 参数选型决策表

| 系统 | 推荐参数 |
|---|---|
| LiDAR pose graph | `relinearizeSkip=1`, threshold 0.01-0.1, CHOLESKY |
| VIO fixed-lag | threshold 更小，关注 bias，必要时 QR |
| 大回环系统 | Dogleg 或回环后多次 update |
| SmartFactor 密集系统 | 小心 `cacheLinearizedFactors` |
| 调试阶段 | 打开 nonlinear error 与 detailed result |

##### 故障排查表

| 症状 | 可能原因 | 排查步骤 |
|---|---|---|
| 回环后轨迹只局部变形 | update 次数不足或 wildfire 阈值过大 | 多跑空 update，降低 wildfire |
| 每帧越来越慢 | Bayes 树根团变大或 factor slot 不复用 | 打印 clique 数，开启 `findUnusedFactorSlots` |
| 边缘化报非叶错误 | constrained ordering 错 | 检查 Group 0/1，确认目标 key 在叶团 |
| Cholesky 崩溃 | gauge、退化或坏回环 | 加先验、切 QR、检查 robust 权重 |
| SmartFactor 相关崩溃 | 缓存线性因子与 key 顺序冲突 | 固化或重建 smart factors |

##### 练习

1. 设计一个实验：逐渐增加错误回环比例，比较 iSAM2+DCS 与 batch GNC-TLS 的成功率。
2. 在 LIO-SAM 风格的图中，每次回环后分别跑 1、3、6 次空 update，比较 ATE 与耗时。
3. 对 fixed-lag smoother 构造一个非叶边缘化失败案例，并用 constrainedKeys 修复。

---

### §C.31 与其他子任务的桥梁

| 方向 | 关系 | 关键映射 |
|------|------|---------|
| ← **5-B（因子图与非线性最小二乘）** | **消元 = Schur 补 = 边缘化** 产出的是 Bayes 树 | 5-B 的 batch GN/LM 直接在 Bayes 树上退化为一次 wildfire |
| ← **5-A（RTS / 卡尔曼滤波 / MHE）** | **RTS = 特例块三对角的野火回代** | Bayes 树在链状结构（纯里程计）上是一条链，iSAM2 退化为增量 RTS |
| → **5-F（鲁棒代价 + 可切换约束）** | iSAM2 + Robust noise model（Huber/Cauchy/DCS）处理坏回环 | `noiseModel::Robust` + ISAM2；GNC 需要批量 |
| → **5-E（certifiable 全局最优）** | iSAM2 局部最优 → SE-Sync/DC²-PGO 提供全局证书 | 组合 workflow：iSAM2 实时 + 定期 SE-Sync refinement 注入 prior |
| → **5-D（InEKF 理论精读）** | Bayes 树给出优化视角，InEKF 给出群结构下的滤波一致性视角 | 同一个估计问题的 batch / recursive 两种语言 |
| → **后续专题（多机器人 / 分布式）** | Bayes 树子树独立性 → DDF-SAM / Kimera-Multi | Separator 交换 + anti-factor |
| → **后续专题（连续时间 GP-SLAM）** | GP 先验 → block-tridiagonal → iSAM2 增量兼容 | STEAM + GTSAM |

---

### 总结：从批量到增量的思维转变

**最后一句话的升华**：iSAM2 不是一个"更快的 GN"，而是一种**对全图 MAP 问题结构的重新解读**——它告诉我们，MAP 解的本质是一棵**概率上的树**，而不是一个需要每次全解的矩阵方程。从这个角度看：

- **Kalman 滤波** 是 Bayes 树退化为一条链的情形；
- **RTS 平滑** 是沿链的双向回代；
- **Batch GN** 是对整棵树做一次全遍历；
- **iSAM2** 是在树上做"只读到变化处为止"的懒惰求值（lazy evaluation）。

理解这点后，**每当面对一个状态估计问题**，应先问：
1. 概率图的稀疏结构是什么？
2. Bayes 树长什么样？
3. 新因子影响哪些团？
4. 哪些团可以懒更新、哪些必须重线性化？

这条思维链路把"估计 = 求解方程"升华为"估计 = 在稀疏树上做消息传递"，而后者在机器人学、计算机视觉、SLAM、多传感器融合、分布式估计等所有场景都统一适用。这正是 Dellaert & Kaess 2017 在 *Factor Graphs for Robot Perception* 开篇所主张的**"图模型就是机器人感知的一等语言"**。

5-C 到此结束。接下来的三个专题（D、E、F）是相对独立的前沿深入：5-D 回到 A3 的 InEKF 理论基础，对 Barrau-Bonnabel TAC 2017 做逐定理精读；5-E 引入 Certifiable Perception 与 SDP 松弛，探索全局最优保证；5-F 讨论鲁棒代价函数（Huber、DCS、GNC）如何与因子图/iSAM2 组合，处理 SLAM 中不可避免的外点与错误回环。三者都以 A/B/C 建立的滤波与优化框架为前提。

---

### §C.32 iSAM2 的局限与开放问题 ⭐⭐⭐

尽管 iSAM2 是当前增量 SLAM 后端的事实标准，它仍有明确的局限。理解这些局限对于正确选型和未来研究都至关重要。

**局限一：局部最优性**

iSAM2 本质是增量 Gauss-Newton——它收敛到**局部最优**而非全局最优。在以下场景中局部最优问题严重：

- 大角度回环（如机器人转了 360 度后闭合）：GN 一步的线性近似可能跨越多个极值点
- 多义性场景（如对称走廊）：存在多个代价相近的局部极小
- 坏数据关联（如错误回环）：可能收敛到错误拓扑对应的极小点

5-E 的 SE-Sync 通过 SDP 松弛和对偶证书解决了 PGO 的全局最优问题，但它是 batch 方法。**Incremental Certifiable SLAM 是当前的开放问题之一**。一种工程上可行的混合策略是：实时用 iSAM2，周期性触发 SE-Sync 验证/修正，再把结果注入 iSAM2 作为先验。

**局限二：不原生支持 GNC**

GNC（Graduated Non-Convexity）需要在整个因子图上同步调节鲁棒核的参数 $\mu$，这本质上是 batch 外层循环。iSAM2 的 fluid relinearization 假设因子图的目标函数在相邻帧之间变化较小——GNC 的全局 $\mu$ 退火违反了这个假设。因此在线系统只能用 Huber/DCS（逐因子独立降权），离线/周期性后端用 GNC。

**局限三：边缘化信息不可恢复**

`marginalizeLeaves` 把叶团的信息编码进父团的 separator 上的 prior。一旦边缘化，即使后来发现该信息有误（如后来检测到外点），也无法"撤销"——边缘化是不可逆操作。这与 ORB-SLAM3 的"宁可忘掉也不记错"哲学形成对比。

**局限四：变量维度变化困难**

iSAM2 假设每个变量的维度在创建后不变。若需要动态改变变量（如 landmark 从逆深度参数化切换到 3D 点参数化），需要删除旧变量、重新插入新变量、重建相关因子——操作代价高且容易出错。

**开放问题汇总**：

| 问题 | 难点 | 潜在方向 |
|------|------|---------|
| Incremental Certifiable | SDP 松弛与增量 Bayes 树结合 | warm-start Burer-Monteiro；增量对偶证书更新 |
| iSAM2 + GNC | 全局权重退火 vs 局部更新 | 分区域退火；双线程（iSAM2 实时 + GNC 后台） |
| 语义因子 | 离散-连续混合变量 | Hybrid Bayes Tree（GTSAM 开发中） |
| 大规模长时运行 | 内存/计算��性增长 | 分层地图；稀疏化子图 |
| 分布式多机器人 | 各机器人 Bayes 树同步 | DC2-PGO + anti-factor |

---

### §C.33 设计决策总结：何时用 iSAM2、何时不用 ⭐⭐

这是工程选型的核心问��。以下决策流程图基于实际系统的经验总结：

**用 iSAM2 的典型场景**：
- LiDAR SLAM 后端（LIO-SAM、SC-LIO-SAM、LeGO-LOAM-v2）：关键帧频率 2-10 Hz，pose graph 结构稀疏，偶尔回环
- 固定延迟 VIO（Kimera-VIO、OKVIS2）：滑窗 + 边缘化，每帧添加少量因子
- 多传感器融合：IMU + GPS + LiDAR 异步因子，需要全��迹一致估计
- 在线语义建图（Hydra、Kimera-Multi）：需要随时查询历史 pose 的边缘协方差

**不适合用 iSAM2 的场景**：
- 纯视觉 BA（大量 landmark 频繁出入）：prefer Schur 补 + 局部 BA（ORB-SLAM3 路线）
- 需要全局最优保证：prefer SE-Sync + batch LM
- 高外点率需要 GNC：prefer batch LM + GncOptimizer
- 仅需当前帧位姿（不需全轨迹）：prefer MSCKF 或 motion-only BA
- 超大规模离线建图（百万帧）：prefer hierarchical 方法 + 子图分割

**决策核心原则**：
1. 需要**全轨迹一致估计** + **实时增量** → iSAM2
2. 需要**全局最优** → batch + 证书
3. 变量拓扑**频繁剧烈变化** → 不适合 iSAM2 的缓存机制
4. 纯**滤波**足矣（不需全轨迹） → EKF/MSCKF 更简单高效
5. **离线后处理**不在乎实时性 → batch LM（g2o/Ceres）即可，代码更简单
6. **多机器人分布式**场景 → DC2-PGO 或 DDF-SAM2，iSAM2 仅作为单机器人本地后端

**工程选型速查表**：

| 应用场景 | 推荐后端 | 原因 |
|---------|---------|------|
| 自动驾驶 LiDAR SLAM | iSAM2（LIO-SAM 模式） | 关键帧稀疏、增量自然、需全轨迹 |
| AR/VR VIO | iSAM2 Fixed-Lag（Kimera 模式） | 低延迟、滑窗边缘化控制内存 |
| 无人机视觉 SLAM | 滑窗 BA（VINS-Mono 模式） | 特征频繁进出、不需全轨迹 |
| 手持三维重建 | ORB-SLAM3 + batch BA | 需要精确地图点、Schur 补高效 |
| 多机器人协作建图 | 本地 iSAM2 + 全局 DC2-PGO | 兼顾实时性与全局一致 |
| 后处理精修（ground truth 生成） | SE-Sync + batch LM | 需要证书保证全局最优 |
| 稀疏环境长走廊 | iSAM2 + SC/DCS | 回环少但一旦闭合误差大 |
| 密集特征环境（仓库） | iSAM2 + SmartFactor | 大量 landmark，注意缓存冲突 |
| 水下 SLAM（声学） | iSAM2 + 强先验 | 测量稀疏但噪声大，需要鲁棒核 |

> **反事实推理——如果所有 SLAM 系统都用 iSAM2 会��样？** ORB-SLAM3 的视觉 BA 每帧涉及 ~200 个 map point 的可见性变化——每帧都要大量 remove + reinsert factor，iSAM2 的 `cacheLinearizedFactors` 反复失效，性能可能反而比直接 batch Schur 补更差。这解释了为什么 ORB-SLAM3 选择 g2o 而非 GTSAM/iSAM2。**正确的工程直觉是：当因子图拓扑稳定时，iSAM2 的增量缓存优势最大；当拓扑频繁变动时，batch 方法的简单性反而更高效。**

---

### §C.34 信息形式与协方差形式的对偶视角 ⭐⭐⭐

理解 Bayes 树需要区分两种表示联合高斯分布的方式，以及它们���自擅长的操作。

**信息形式（Information Form / Canonical Form）**：

$$p(\Theta)\propto\exp\!\Big(-\frac{1}{2}\Theta^\top\Lambda\Theta+\eta^\top\Theta\Big)$$

其中 $\Lambda=\Sigma^{-1}$ 是信息矩阵（精度矩阵），$\eta=\Lambda\mu$ 是信息向量。SLAM 的信息矩阵天然稀疏——因子 $f_i(\Theta_i)$ 只在 $\Theta_i$ 对应的行/列上贡献非零块。

**协方差形式（Covariance Form / Moment Form）**：

$$p(\Theta)\propto\exp\!\Big(-\frac{1}{2}(\Theta-\mu)^\top\Sigma^{-1}(\Theta-\mu)\Big)$$

均值 $\mu=\Lambda^{-1}\eta$，协方差 $\Sigma=\Lambda^{-1}$。协方差矩阵通常**稠密**（$\Sigma_{ij}\neq 0$ 只要 $\Theta_i$ 和 $\Theta_j$ 在图上可达）。

**操作效率对比**：

| 操作 | 信息形式 | 协方差形式 |
|------|---------|-----------|
| 添加新因子 | $\Lambda\mathrel{+}= J_i^\top\Omega_i J_i$ — **局部加法** $O(1)$ | 需重算整个 $\Sigma$ — $O(N^2)$ |
| 边缘化变量 $x_j$ | **Schur 补** $O(N_j^2 N_{-j})$ — 产生 fill-in | 直接删行/列 — $O(1)$ |
| 条件化（给定 $x_j$ 的值） | 直接删行/列 — $O(1)$ | Schur 补 $O(N)$ |
| 查询单变量边缘协方差 | 需要求解/部分求逆 — $O(\text{path length}^2)$ | 直接读取 $\Sigma_{jj}$ — $O(1)$ |

**Bayes 树的定位**：Bayes 树同时编码了 $R$（信息的平方根 $\Lambda=R^\top R$）和��件密度（类似协方差的分层表达）。它在信息形式下操作（添加因子 = 局部��改、消元 = QR），但也能高效回答协方差查询（沿路径回代计算 `marginalCovariance`）。

**EKF vs iSAM2 的本质差异**从此表一目了然：EKF 维护协方差形式（方便预测/更新但代价 $O(N^2)$），iSAM2 维护信息形式的平方根因子（方便添加因子但边缘协方差查询代价正比于 Bayes 树路径长度）。

| 系统 | 表示形式 | 添加新观测 | 查询任意变量协方差 | 边缘化 |
|------|---------|-----------|----------------|--------|
| EKF | $\Sigma$ (协方差) | $O(N^2)$ | $O(1)$ | $O(N^2)$ |
| SEIF | $\Lambda$ (信息) | $O(1)$ | $O(N^2)$ 求逆 | $O(N)$ Schur |
| iSAM2 | $R$ (Cholesky = Bayes 树) | $O(\sqrt{N})$ | $O(\text{path})$ | $O(1)$ 删叶 |

> **跨领域类比**：信息形式与协方差形式的对偶关系类似于频域与时域的 Fourier 对偶。
> 在时域中卷积复杂但点乘简单；在频域中点乘简单但卷积需要逆变换。
> 同理，信息形式中添加约束（"乘法"）简单但查询边缘（"积分"）复杂；
> 协方差形式中查询边缘简单但添加约束复杂。
> Bayes 树的精妙之处在于：它用**树结构**把两种操作的代价都压缩到了中间地带—— $O(\sqrt{N})$。

### 常见陷阱与故障排查

⚠️ **陷阱一：只保留 conditional 均值关系，丢掉信息尺度。** Gaussian 消元中的 $R_{jj}$ 或 $\|a\|$ 决定条件密度方差，不能从公式里省掉。

⚠️ **陷阱二：把消元树祖先关系等同于每个 Cholesky 非零元。** 非零元给出直接依赖，祖先关系是传递闭包。

⚠️ **陷阱三：把 wildfire threshold 当成严格数学剪枝。** 它是数值启发式，阈值越大越快但误差越可能累积。

| 故障排查现象 | 可能原因 | 处理方式 |
|---|---|---|
| 增量更新后局部轨迹抖动 | relinearization threshold 过大 | 降低阈值，比较 batch 解 |
| Bayes 树根团过大 | ordering 不合适或回环集中 | 尝试 COLAMD/METIS，检查 separator |
| 鲁棒核与 iSAM2 效果不稳定 | 权重变化和增量重线性化交互 | 在线用 Huber/DCS，周期性 batch GNC |

> **本质洞察**：iSAM2 的速度来自“只更新受影响的团”，而不是改变了 MAP 问题本身；Bayes 树是稀疏消元结构的概率表达。

### 练习

1. 对一个三变量线性高斯系统手算一次消元，保留 $R_{jj}$，写出完整 conditional。
2. 画一个 5 节点链加一条回环的消元树，标出直接非零依赖和祖先传递依赖。
3. 改变 wildfire threshold，记录 `calculateEstimate()` 耗时和与 batch 解的差异。

---

### 本章小结

| 核心概念 | 一句话要义 | 关键公式/定理 | 工程映射 |
|---------|-----------|-------------|---------|
| Bayes 树 | 消元产物的概率有根树——每团存一个条件密度 | $P(\Theta)=\prod_k P(F_k\mid S_k)$ | `GaussianBayesTree` |
| 超节点等价 | Bayes 树团 = Cholesky 超节点 = 弦图最大团 | Liu 1990 Thm 2.4 | CHOLMOD supernode |
| Fluid Relinearization | 只对 $\|\delta_j\|>\beta$ 的变量重算 Jacobian | Taylor 余项为二阶小量 | `relinearizeThreshold` |
| findAffectedTop | 新因子/relin 变量沿 parent 指针到 LCA 的路径 | 局部性：回环仅影响 root-path | `ISAM2::update` 内部 |
| removeTop + graft | 拆出受影响子树 → 重消元 → orphan 挂回 | orphan 携带 cached marginal | `ISAM2-impl.h` |
| Wildfire 回代 | 沿变化显著的子树向下传播新 $\delta$ | $\|\delta_F^{new}-\delta_F^{old}\|\le\|R_{FF}^{-1}R_{FS}\|\cdot\Delta\delta_S$ | `wildfireThreshold` |
| 复杂度界 | 2D pose graph $O(\sqrt{N})$，3D BA $O(N^{2/3})$ | Lipton-Rose-Tarjan 1979 嵌套剖分 | COLAMD/METIS 排序 |
| 叶剪枝 = 边缘化 | 删叶团 = 精确边缘化 frontal 变量 | $\int P(F_k\mid S_k)dF_k=1$ | `marginalizeLeaves` |
| 非 LM | iSAM2 默认 GN（无阻尼）；大回环需多次 update 或 Dogleg | 回环后 LIO-SAM 跑 6 次空 update | `ISAM2DoglegParams` |
| GNC 不兼容 | GNC 是 batch outer-loop；在线用 Huber/DCS | 全图权重同时变化 $\neq$ 局部更新 | 周期性 batch GNC |

---

### 累积项目：本章新增模块

**项目名称**：从零构建 Mini-LIO 后端

**本章新增**：iSAM2 增量优化后端

在前序章节中，累积项目已完成：
- Ch1（5-A）：Kalman 滤波递推器
- Ch2（5-B）：Batch 因子图 GN 求解器

本章新增模块：
1. 用 GTSAM Python 绑定实现一个 `ISAM2Backend` 类
2. 支持逐帧添加 `BetweenFactor<Pose3>`（里程计）和 `PriorFactor`（GPS）
3. 实现回环检测后的多次空 `update()` 策略
4. 对比不同 `relinearizeThreshold` 下的 ATE 曲线
5. 在 M3500 数据集上验证每步耗时与 $\sqrt{N}$ 的标度关系

```python
# 累积项目骨架（学生需填充 TODO 部分）
import gtsam
from gtsam.symbol_shorthand import X

class ISAM2Backend:
    def __init__(self, relin_threshold=0.1):
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(relin_threshold)
        params.setRelinearizeSkip(1)
        self.isam = gtsam.ISAM2(params)
        self.frame_id = 0

    def add_odom(self, odom_pose, noise_model):
        """添加里程计因子"""
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        # TODO: 构造 BetweenFactor 并添加
        # TODO: 为新变量提供初值
        pass

    def add_loop_closure(self, from_id, to_id, relative_pose, noise_model):
        """添加回环因子并多次 update"""
        graph = gtsam.NonlinearFactorGraph()
        # TODO: 添加回环 BetweenFactor
        # TODO: 多次空 update（参考 LIO-SAM 策略）
        pass

    def get_trajectory(self):
        """返回全轨迹估计"""
        return self.isam.calculateEstimate()
```

---

### 延伸阅读

| 资源 | 难度 | 内容 | 建议阅读方式 |
|------|------|------|------------|
| Kaess et al. "iSAM2" IJRR 2012 | ⭐⭐⭐ | iSAM2 完整算法 + 复杂度分析 | 精读 §III-IV + Alg. 2-8 |
| Dellaert & Kaess "Factor Graphs for Robot Perception" FnT 2017 | ⭐⭐ | 统一教材 | §5 增量式部分 |
| Kaess PhD 论文 (GaTech 2008) | ⭐⭐⭐ | iSAM1 最详细推导 | Ch.4-5 |
| Davis *Direct Methods for Sparse Linear Systems* SIAM 2006 | ⭐⭐⭐⭐ | 消元树/超节点/稀疏 Cholesky 数学 | Ch.3-4 |
| Liu "Role of Elimination Trees" SIMAX 1990 | ⭐⭐⭐⭐ | 消元树基本定理 | Thm 2.4 及其证明 |
| Lipton-Rose-Tarjan 1979 "Generalized Nested Dissection" | ⭐⭐⭐⭐ | 2D $O(N^{3/2})$ 界的数学来源 | Thm 1 |
| Barfoot *State Estimation for Robotics* 2ed 2024 | ⭐⭐ | 批量 SAM + GP-SLAM 入门 | Ch.9 |
| Rosen et al. "SE-Sync" IJRR 2019 | ⭐⭐⭐⭐ | 全局最优 PGO + 对偶证书 | §III-V |
| LIO-SAM 源码 `mapOptmization.cpp` | ⭐⭐ | iSAM2 工程最佳实践 | 行 1333-1375 |
| Kimera-VIO `VioBackend.cpp` | ⭐⭐⭐ | Fixed-lag + SmartFactor | 边缘化逻辑 |
| GTSAM Issues #595, #1101, #1976 | ⭐⭐ | SmartFactor + 边缘化陷阱 | 问题复现与修复 |

---

### 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| `IndeterminantLinearSystemException` | 首帧先验过松/欠约束方向/gauge 自由度 | 1.检查 PriorFactor sigma 2.确认 roll/pitch 有 IMU 约束 3.确认 yaw 有航向源 | §C.23 #13 |
| 回环后轨迹仅局部变形、远处不动 | wildfire 阈值过大或 update 次数不足 | 1.多跑 3-6 次空 update 2.降低 `wildfireThreshold` 3.开启 `force_relinearize` | §C.10, §C.23 #7 |
| 每帧耗时单调递增 | Bayes 树根团膨胀/factor slot 未复用 | 1.打印 clique 数量 2.开启 `findUnusedFactorSlots` 3.检查是否需要 constrained ordering | §C.23 #14 |
| `marginalizeLeaves` 报"not leaves"错误 | constrained ordering 未将目标 key 推至叶 | 1.检查 Group 0/1 设置 2.确认目标 key 无子树 3.参考 `IncrementalFixedLagSmoother` 实现 | §C.23 #3 |
| SmartFactor 相关崩溃 | cached linear factor 与 key 生命周期冲突 | 1.关闭 `cacheLinearizedFactors` 2.边缘化前固化 SmartFactor 为 JacobianFactor 3.删旧重建 | §C.23 #4 |
| 鲁棒核加了但外点仍拉坏图 | Huber/DCS 阈值单位理解错误 | 1.确认残差已白化 2.对照 §F.23 的三库映射表 3.先试保守阈值再逐步收紧 | §F.23, §C.23 |
| Dogleg 比 GN 更慢但无精度提升 | 场景简单、初值好，信赖域无必要 | 1.切回 GN 2.仅在回环密集场景用 Dogleg | §C.12, §C.23 #12 |

---

### 跨章综合练习 ⭐⭐⭐

**题目**：综合 5-A（Kalman 滤波）+ 5-B（因子图优化）+ 5-C（iSAM2）的知识，完成以下分析：

设一个 1D 位置估计问题：状态 $x_k\in\mathbb{R}$，运动模型 $x_k = x_{k-1}+u_k+w_k$（$w_k\sim\mathcal{N}(0,Q)$），观测 $z_k=x_k+v_k$（$v_k\sim\mathcal{N}(0,R)$），$k=1,...,100$。

1. **Kalman 滤波视角**（5-A）：写出 $P_k$ 的递推公式，证明 $P_k\to P_\infty = \frac{-Q+\sqrt{Q^2+4QR}}{2}$ 稳态。
2. **Batch 因子图视角**（5-B）：写出 100 步的信息矩阵 $\Lambda\in\mathbb{R}^{100\times 100}$，证明它是三对角矩阵。画出其稀疏模式。
3. **iSAM2 视角**（5-C）：对上述链式因子图构造 Bayes 树（消元顺序 $x_1,...,x_{100}$），证明 Bayes 树退化为一条链。解释为什么 iSAM2 在此退化情况下等价于 Kalman 滤波。
4. **回环的影响**：现在加入一条额外因子 $f(x_1, x_{100})$。画出新的 Bayes 树结构。解释为什么这条回环使得 iSAM2 必须从 $x_1$ 的团一直更新到根（即 wildfire 传遍全树）。这正是"回环代价高"的数学本质。

### 延伸思考：iSAM2 的工程边界

在工程实践中，iSAM2 并非万能。以下场景需要特别警惕：

- **长走廊问题**：当机器人沿长走廊行驶且无回环时，Bayes 树退化为长链，每次更新需从叶向根传播，复杂度退化为 $O(n)$。此时滑窗边缘化（Fixed-Lag Smoother）比 iSAM2 更高效。
- **高动态地图**：若环境中大量路标频繁出现/消失，factor 的增删操作密集，`findUnusedFactorSlots` 的开销可能显著。需要定期对 Bayes 树做 full batch relinearization 来整理内部结构。
- **多机协同**：分布式 iSAM2 需要解决 Bayes 树的跨机器同步问题。Cunningham et al. (ICRA 2010) 的 DDF-SAM 提供了一种通过条件独立性分割因子图的方案。

---
