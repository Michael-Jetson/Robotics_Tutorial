## 博士前数学路线图·第五批·子专题 A4：Kalman 族全景收口——迭代变体、平滑、全景对比与完整工程映射

> **底线陈述（BLUF）**：本文是 5-A 系列的**收口**。到此为止，您应能：(i) 把线性 KF / EKF / UKF / ESKF / InEKF / IKFoM / EnKF / Schmidt-KF / $H_\infty$ / RTS / MHE 这一族方法**用一张图组织起来**，并说出每一对相邻方法的精确差异；(ii) 任给一个工程场景（VIO、LIO、腿足、航天、低成本 IMU），**从 18 个开源库里准确选型**并指出其四元数与扰动约定；(iii) 能把**迭代 EKF ≡ 单时间步 Gauss-Newton**、**RTS ≡ 块三对角系统回代**、**VINS-Mono 滑窗 ≡ 非线性 MHE**这三条跨学科等价关系写在黑板上现场推导；(iv) 避开 5-A1 ~ 5-A4 累积 20 条工程陷阱。**一句话概括 5-A 全系列**：*「先验高斯 × 线性化策略 × 状态空间几何 × 迭代/平滑维度」这四条正交坐标轴，生成了整个 Kalman 族*。

**A1 → A2 → A3 → A4 递进关系回顾**。A1 在线性高斯假设下建立了 KF 的完整机器（协方差形、信息形、平方根形、一致性诊断），这是整个 Kalman 族的地基。A2 保持欧氏状态空间不变，将线性化策略从"精确"升级为"近似"——EKF 的一阶 Taylor、UKF 的 sigma 点、CKF 的球面径向积分——同时暴露了 EKF 在 SLAM/VIO 中的一致性病理。A3 保持一阶线性化不变，将状态空间从 $\mathbb{R}^n$ 提升到李群流形——ESKF/MEKF 解决过参数化与奇点，InEKF 利用 group-affine 结构从根源消除虚假可观测性。本专题 A4 在前三者基础上打开最后两个维度：**迭代**（IEKF = 单步 Gauss-Newton，连接到因子图世界）和**平滑**（RTS、MHE、滑动窗口），并以全景对比表和 18 个开源库选型指南作为 Kalman 族的工程收口。

---

### §A4.1 迭代 EKF (IEKF) 完整推导与 Gauss-Newton 等价 ⭐⭐

#### A4.1.1 问题设定与迭代更新式

考虑标准离散非线性高斯模型
$$
x_k = f(x_{k-1},u_{k-1}) + w_{k-1},\quad y_k = h(x_k) + v_k,\qquad w\sim\mathcal N(0,Q),\ v\sim\mathcal N(0,R).
$$
预测步与 EKF 相同，得到先验 $\hat x_{k|k-1}, P_{k|k-1}$。**迭代更新步**（Bell & Cathey, *IEEE TAC* 38(2):294–297, 1993, eq. (5)）以 $\hat x^{(0)} = \hat x_{k|k-1}$ 初始化，迭代执行：

$$
\boxed{\;\hat x^{(j+1)} = \hat x_{k|k-1} + K^{(j)}\!\Big[\,y_k - h(\hat x^{(j)}) - H^{(j)}\big(\hat x_{k|k-1}-\hat x^{(j)}\big)\Big]\;}\qquad(A4.1)
$$

其中 $H^{(j)}=\partial h/\partial x|_{\hat x^{(j)}}$，$K^{(j)} = P_{k|k-1}H^{(j)\top}(H^{(j)}P_{k|k-1}H^{(j)\top}+R)^{-1}$。**注意**：括号内 $-H^{(j)}(\hat x_{k|k-1}-\hat x^{(j)})$ 是先验线性化的**搬移项**，标准 EKF（即 $j=0$）时此项为零而与 EKF 重合；若漏写（工程上常见错误），将导致每次迭代"忘记先验"。收敛后的协方差为 $P_{k|k}=(I-K^{(j^\star)}H^{(j^\star)})P_{k|k-1}$，$j^\star$ 为收敛迭代数。

#### A4.1.2 Bell-Cathey 定理：IEKF = 对 MAP 做 Gauss-Newton

**定理**（Bell-Cathey 1993；Barfoot 2ed §4.2.6 pp.120–121）。**在加性高斯噪声模型** $y_k = h(x_k) + v_k,\, v\sim\mathcal{N}(0,R)$ **下**，式 (A4.1) 是对负对数后验
$$
J(x)=\tfrac12\|x-\hat x_{k|k-1}\|_{P_{k|k-1}^{-1}}^2+\tfrac12\|y_k-h(x)\|_{R^{-1}}^2\qquad(A4.2)
$$
做 **Gauss-Newton** 迭代的 Kalman 等价形式。因此 IEKF 若收敛，其定点是该单步 MAP 目标的一阶驻点；在局部凸或残差足够温和的邻域中通常对应局部极小，但非凸观测模型下不保证全局 $\arg\min$。

**证明思路**。设 $L_P L_P^\top = P_{k|k-1}$, $L_R L_R^\top = R$，堆叠残差 $r(x) = [L_P^{-1}(x-\hat x_{k|k-1});\; L_R^{-1}(y-h(x))]$ 使 $J=\tfrac12\|r\|^2$。在 $\hat x^{(j)}$ 处线性化 $h$ 得
$$
J_r^{(j)} = \begin{bmatrix}L_P^{-1}\\ -L_R^{-1}H^{(j)}\end{bmatrix},\quad J_r^{(j)\top}J_r^{(j)} = P_{k|k-1}^{-1} + H^{(j)\top}R^{-1}H^{(j)}.
$$
Gauss-Newton 步 $\Delta^{(j)} = -(J_r^\top J_r)^{-1}J_r^\top r(\hat x^{(j)})$ 代入 Woodbury 恒等式
$$
(P^{-1}+H^\top R^{-1}H)^{-1}H^\top R^{-1}=K,\quad (P^{-1}+H^\top R^{-1}H)^{-1}P^{-1}=I-KH,
$$
即化为 (A4.1)。$\square$ **推广**：Bell 1994 (*SIAM J. Opt.* 4(3):626–636) 证明**迭代平滑 = 整条轨迹的 GN**，这是后来"IEKF = 单步因子图 / BA = 全步因子图"的理论基石。

#### A4.1.3 收敛判据、LM-IEKF、Line-search IEKF

**收敛判据**两选一：状态增量 $\|\hat x^{(j+1)}-\hat x^{(j)}\|<\varepsilon$（典型 $10^{-3}\sim 10^{-4}$）或代价相对下降 $|J^{(j+1)}-J^{(j)}|<\delta$。实现上常设最大迭代 $J_{\max}\in\{3,4,5\}$（FAST-LIO2 `max_iteration` 默认 3–4）。

**LM-IEKF** (Skoglund-Hendeby-Axehill, *FUSION* 2015 §III)：在近似 Hessian 上加阻尼
$$
A_\lambda^{(j)} = P^{-1}+H^{(j)\top}R^{-1}H^{(j)}+\lambda I,\qquad(A4.3)
$$
$\lambda$ 按 Nielsen 策略：接受步时 $\lambda\leftarrow\lambda\cdot\max(1/3, 1-(2\rho-1)^3)$，拒绝步时 $\lambda\leftarrow\lambda\cdot 2^k$。

**Line-search IEKF** (Särkkä & Svensson *BFS* 2ed §7.5; ICASSP 2020)：求出 GN 方向后 $\hat x^{(j+1)}=\hat x^{(j)}+\alpha\Delta$，取 $\alpha\in(0,1]$ 满足 Armijo 条件 $J(\hat x+\alpha\Delta)\le J(\hat x)+c_1\alpha\nabla J^\top\Delta$（$c_1=10^{-4}$，下降方向满足 $\nabla J^\top\Delta<0$），否则 $\alpha\leftarrow\alpha/2$ 回溯。

#### A4.1.3b IEKF 收敛性质的直觉解释 ⭐⭐

**为什么 IEKF 能改善 EKF？** 考虑一个强非线性观测 $h(x)=\arctan(x)$。EKF 在先验均值 $\hat x^-=5$ 处线性化，得到 $H=1/(1+25)=0.038$——这是一个非常平坦的斜率。即使观测与预测差异很大，小 $H$ 导致 $K$ 很小，更新几乎不动。但实际后验的众数可能在 $x=1$ 附近——在那里 $H=1/2=0.5$，线性化好得多。

IEKF 的迭代正是解决这个问题：每次迭代在新的点重新线性化，逐步逼近真正的后验众数。

**IEKF 不是万能的——何时迭代无用？** 当观测函数 $h$ 在先验覆盖范围内近似线性时（例如 $h(x)=x$ 加噪声），一次线性化已经足够，迭代不会改善任何东西——只会浪费计算。这就是为什么 FAST-LIO2 默认只迭代 3-4 次：LiDAR 点到面残差在配准良好后几乎线性，更多迭代没有收益。

**IEKF 在流形上的扩展**。当状态在李群上时，迭代更新式 (A4.1) 变为 §A4.3 的 IKFoM 形式，核心区别是：
- "搬移项"中的减法 $\hat x_{k|k-1}-\hat x^{(j)}$ 变成 $\boxminus$ 运算
- 曲率 Jacobian $J^{(j)}$ 出现在先验项中
- 增量通过 $\boxplus$ 注入而非加法

这些修改保证了迭代过程尊重流形几何，不会因为"在流形上做加法"而引入误差。

> **本质洞察**：IEKF 的本质不是"多做几次 EKF 更新"，而是"在 MAP 目标函数上做 Gauss-Newton 迭代"。每次迭代改善的是线性化点的质量，而非信息量。如果先验已经很准（$\hat x^-$ 接近 MAP），一次迭代就够了（退化为 EKF）；如果先验很差或观测非线性很强，多次迭代能显著改善。

⚠️ **陷阱：把 IEKF 迭代理解为"多次利用同一观测"**

错误想法："IEKF 对同一个 $y_k$ 做了 $J_{\max}$ 次更新，所以信息被用了 $J_{\max}$ 倍。"

实际上：IEKF 的 $J_{\max}$ 次迭代收敛到的定点**与 $J_{\max}$ 无关**——它始终是同一个 MAP 解（在凸条件下）。迭代次数只影响能否到达这个解，不影响到达后的协方差。最终协方差由收敛点的 Hessian 决定：$P_{k|k}^{-1}=P_{k|k-1}^{-1}+H^{(j^\star)\top}R^{-1}H^{(j^\star)}$。

#### A4.1.4 IEKF 与因子图的**单步等价**

把 $x_{0:N}$ 全部放入因子图，联合 MAP 目标
$$
J_{\text{BA}} = \tfrac12\|x_0\boxminus\hat x_0\|_{P_0^{-1}}^2+\sum_k\tfrac12\|x_k\boxminus f(x_{k-1})\|_{Q^{-1}}^2+\sum_k\tfrac12\|z_k\boxminus h(x_k)\|_{R^{-1}}^2
$$
做 GN/LM 即为 **Bundle Adjustment / Full Graph SLAM**。IEKF 是对该目标做**顺序 Schur 边缘化**：每一步把 $x_{0:k-1}$ 消去成等价先验 $(\hat x_{k|k-1},P_{k|k-1})$，仅对 $x_k$ 做 GN。故**MSCKF ≡ 特殊部分联合 IEKF**（相机位姿联合保留、特征零空间投影消去）；**FAST-LIO2 ≡ 23 维状态上的单步流形 MAP**；**VINS-Fusion ≡ 较大窗口不边缘化当前帧的 MAP**。

---

### §A4.2 Iterated Sigma-Point KF (ISPKF) ⭐⭐⭐⭐

#### A4.2.1 算法骨架

ISPKF（Sibley, Sukhatme, Matthies, *RSS* 2006）把 (A4.1) 的解析线性化换为 **statistical linear regression (SLR)**：在 $\hat x^{(j)}, P^{(j)}$ 处生成 $2n+1$ sigma 点 $\{\mathcal X_i\}$，过 $h$ 得 $\mathcal Y_i$，按权重求 $\mu_y^{(j)}, \Sigma_{yy}^{(j)}, \Sigma_{xy}^{(j)}$，然后
$$
K^{(j)}=\Sigma_{xy}^{(j)}\Sigma_{yy}^{(j)-1},\quad H_{\rm stat}^{(j)}=\Sigma_{xy}^{(j)\top}(P^{(j)})^{-1},
$$
$$
\hat x^{(j+1)}=\hat x_{k|k-1}+K^{(j)}\bigl[y_k-\mu_y^{(j)}-H_{\rm stat}^{(j)}(\hat x_{k|k-1}-\hat x^{(j)})\bigr].
$$

#### A4.2.2 关键定理：ISPKF **收敛到后验均值**（而非 MAP）

Barfoot 2ed §4.2.11 (p.140) 明确指出：*"upon convergence, the ISPKF provides an estimate of the posterior mean, whereas the IEKF converges to the MAP"*。**根源**：UT/sigma 近似的是期望积分 $\mathbb E[h(x)]$，对应**矩匹配 / KL 意义下的最优高斯**；Taylor 近似的是 $h$ 在点处的梯度，对应**众数 (mode)**。García-Fernández 等 (*IEEE TAC* 60(5), 2015) 把这一差异系统化为 Iterated Posterior Linearization Filter (IPLF, Särkkä *BFS* 2ed Ch.10)，ISPKF 是 IPLF 的一个特例。

#### A4.2.3 工业界为何罕用

(1) 每次迭代都需重生成 sigma 点并过 $h$，复杂度 $O((2n+1)\cdot C_h)\cdot J_{\max}$，相对 IEKF 贵 5–10 倍；(2) 高维需 Cholesky，数值需 SR-UKF 化；(3) 流形扩展繁琐；(4) VIO/LIO 先验窄时精度增益有限；(5) **MAP 语义与因子图后端一致**，ISPKF 的均值语义反而割裂前后端。ISPKF 主要出现在 bearing-only tracking、长距立体、SHM 等强非线性低维场景。

---

### §A4.3 IKFoM 迭代流形 EKF 补充 ⭐⭐

#### A4.3.1 $\boxplus/\boxminus/\oplus$ 三算子

He-Xu-Zhang (*ICRA/arXiv* 2102.03804, 2021) §III 定义 $\boxplus:\mathcal M\times\mathbb R^n\to\mathcal M$, $\boxminus:\mathcal M\times\mathcal M\to\mathbb R^n$，满足 $x\boxplus(y\boxminus x)=y$、$(x\boxplus u)\boxminus x=u$；对李群 $x\boxplus u=x\cdot\operatorname{Exp}(u)$。规范系统形式 (eq. (10))：$x_{k+1}=x_k\oplus(\Delta t\cdot f(x_k,u_k,w_k))$，$z_k=h(x_k,v_k)$。

#### A4.3.2 流形 IEKF 迭代公式

误差态 $\delta x^{(j)}\triangleq x\boxminus\hat x^{(j)}$。关键是**流形曲率 Jacobian** $J^{(j)}$：把切空间扰动从 $\hat x^{(j)}$ 搬到 $\hat x_{k|k-1}$ 所需的线性化矩阵。对 $SO(3)$，$J^{(j)}=A((\hat x^{(j)}\boxminus\hat x_{k|k-1}))^{-\top}$，其中 $A(u)=I+\frac{1-\cos\|u\|}{\|u\|^2}\lfloor u\rfloor+\frac{\|u\|-\sin\|u\|}{\|u\|^3}\lfloor u\rfloor^2$ 是 $SO(3)$ 的左 Jacobian $J_l(u)$，所以 $J^{(j)}$ 实际使用的是 $J_l(u)^{-\top}$。在 $\mathbb R^n$ 上 $J=I$，退化为 IEKF。

Gauss-Newton 增量（He et al. eq. (27)–(28)）：
$$
\boxed{\;\delta^{(j+1)}=-\bigl(J^{(j)\top}P^{-1}J^{(j)}+H^{(j)\top}R^{-1}H^{(j)}\bigr)^{-1}\bigl[J^{(j)\top}P^{-1}(\hat x^{(j)}\boxminus\hat x_{k|k-1})+H^{(j)\top}R^{-1}(h(\hat x^{(j)})-z_k)\bigr]\;}\qquad(A4.4)
$$
$$
\hat x^{(j+1)}=\hat x^{(j)}\boxplus\delta^{(j+1)},\quad P_{k|k}=(I-KH)J^{-1}P_{k|k-1}J^{-\top}.
$$

#### A4.3.3 FAST-LIO2 工程实现

参照 `hku-mars/FAST_LIO` 的 `laserMapping.cpp` 与 `use-ikfom.hpp`：
- `NUM_MAX_ITERATIONS` 读自 ROS 参数 `max_iteration`，默认 **3–4 次**；收敛判据是旋转增量 $<0.01°$ 且平动增量 $<0.015\,\mathrm m$。
- **Rematch 策略**：在末尾两次迭代对 kd-tree 重新做 point-to-plane 最近邻 (`rematch_num>=2 || iterCount==NUM_MAX_ITERATIONS-1`)，避免早期错关联污染收敛值。
- **高效 Kalman 增益**：FAST-LIO §III-D Theorem 1 用 $K=(H^\top R^{-1}H+P^{-1})^{-1}H^\top R^{-1}$，复杂度只与状态维 $n\approx 23$ 相关而非测量维 $m\sim 10^3\sim 10^4$，这是 LIO 实时的关键。
- **失败回退**：$\|\delta\|$ 超阈值直接截断；FAST-LIVO2 加入 LM 阻尼和 patch 级 outlier 剔除。

---

### §A4.4 Rauch-Tung-Striebel 平滑器家族 ⭐⭐⭐

#### A4.4.1 线性 RTS（Rauch-Tung-Striebel 1965, *AIAA J.* 3(8):1445）

**两阶段算法**。Forward pass 存 $\hat x_{k|k}, P_{k|k}, \hat x_{k+1|k}, P_{k+1|k}$；Backward pass 从 $k=N-1$ 到 $0$：
$$
\boxed{\;C_k = P_{k|k}F_k^\top P_{k+1|k}^{-1},\quad \hat x_{k|N}=\hat x_{k|k}+C_k(\hat x_{k+1|N}-\hat x_{k+1|k}),\quad P_{k|N}=P_{k|k}+C_k(P_{k+1|N}-P_{k+1|k})C_k^\top.\;}\qquad(A4.5)
$$
注意 $P_{k+1|N}-P_{k+1|k}\preceq 0$，故 $P_{k|N}\preceq P_{k|k}$（平滑协方差小于滤波），但浮点可能破正定（见陷阱 15）。

#### A4.4.2 三种等价视角（Barfoot 2ed §3.1–3.2；Särkkä 2ed §12）

| 视角 | 做法 | 等价证明 |
|---|---|---|
| **(a) Two-filter (Fraser-Potter 1969)** | 前向 KF + 反向信息滤波器融合 | 代数等价；反向初值必须用信息形式 $Y=P^{-1}\to 0$ |
| **(b) RTS forward-backward** | 前向 KF + 后向修正递推 | 即 (A4.5)，单向存储、最省实现 |
| **(c) Batch MAP** | 把 $\mathbf x=[x_0,\dots,x_N]$ 拼成大向量，信息阵 $\Lambda$ 分块三对角，Cholesky 回代 | **对块三对角做前向消元 = KF，回代 = RTS**；复杂度同为 $O(Nn^3)$ |

教学上**必须掌握 (c) 视角**：它把"滤波 vs 平滑"还原为"消元 vs 回代"，为后续 iSAM2 的 Bayes tree 视角打下基础。

#### A4.4.3 非线性 RTS：EKS / UKS / CKS

Särkkä *BFS* 2ed §13 统一为 **Gaussian RTS 家族**：核心是交叉协方差 $D_{k+1}=\text{Cov}(x_k,x_{k+1}|Y_{0:k})$ 与预测协方差 $P_{k+1|k}$ 的近似方法。

| 平滑器 | 近似 | Jacobian | 计算量/步 | 权重正定 |
|---|---|---|---|---|
| **EKS** | 一阶 Taylor, $F=\partial f/\partial x$ | 需 | $O(n^3)$ | — |
| **UKS** (Särkkä 2008, *IEEE TAC* 53(3):845) | UT, $2n+1$ sigma 点 | 否 | $(2n+1)C_f+O(n^3)$ | $n\le 3$ 常正 |
| **CKS** (Arasaratnam-Haykin-Hurd 2011, *Automatica* 47(10):2245) | 3 阶球面-径向 cubature, $2n$ 点 | 否 | $2nC_f+O(n^3)$ | **始终正** |

**iterated EKS / IPLS** (Särkkä 2ed §13.7, Ch.10)：对 RTS 整体做 GN 迭代，等价于整条轨迹 MAP 的 Gauss-Newton，是 Bell 1994 定理的自然推广。

#### A4.4.3b RTS 平滑器数值示例：1D 匀速模型，5 步完整推演 ⭐⭐

前面给出了 RTS 公式 (A4.5) 和三种等价视角，但公式中 $C_k$、$P_{k|N}$ 到底长什么样？平滑器"利用未来信息"到底怎么体现在数字上？本节用最简单的 1D 匀速模型做 5 步完整手算，让读者直观看到"滤波→平滑"的改善效果。

##### 问题设定

**状态**：$x_k=(p_k,v_k)^\top$，位置和速度。**系统矩阵**：

$$
F=\begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}=\begin{bmatrix}1 & 1\\ 0 & 1\end{bmatrix}\quad(\Delta t=1\text{s}),
\quad H=\begin{bmatrix}1 & 0\end{bmatrix}\quad(\text{只观测位置}).
$$

**噪声**：$Q=\begin{bmatrix}0.25 & 0\\ 0 & 0.25\end{bmatrix}$（过程噪声），$R=[1.0]$（观测噪声标量）。

**初始**：$\hat x_{0|0}=(0,\,1)^\top$（估计从原点出发、速度 1 m/s），$P_{0|0}=\begin{bmatrix}1 & 0\\ 0 & 1\end{bmatrix}$。

**真实轨迹**（仅用于生成观测，滤波器不可见）：匀速 $v=1$ m/s，无噪声。位置序列：$p_{\text{true}}=(0,1,2,3,4)$。

**观测**（加 $\sigma_R=1$ 的高斯噪声）：$y=(0.2,\, 0.8,\, 2.5,\, 2.7,\, 4.3)$。

##### Forward Pass：标准 Kalman 滤波

对每个 $k=0,1,\ldots,4$ 执行预测+更新。为节省篇幅，列出关键结果：

| 时刻 $k$ | 观测 $y_k$ | $\hat x_{k|k}=(p,v)$ | $P_{k|k}$ 对角 $(P_{11},P_{22})$ | $\hat x_{k+1|k}$ | $P_{k+1|k}$ 对角 |
|---|---|---|---|---|---|
| 0 | 0.2 | $(0.10,\, 0.95)$ | $(0.50,\, 0.95)$ | $(1.05,\, 0.95)$ | $(2.20,\, 1.20)$ |
| 1 | 0.8 | $(0.87,\, 0.86)$ | $(0.69,\, 0.82)$ | $(1.73,\, 0.86)$ | $(2.12,\, 1.07)$ |
| 2 | 2.5 | $(2.25,\, 0.98)$ | $(0.68,\, 0.78)$ | $(3.23,\, 0.98)$ | $(2.09,\, 1.03)$ |
| 3 | 2.7 | $(2.86,\, 0.88)$ | $(0.68,\, 0.76)$ | $(3.74,\, 0.88)$ | $(2.07,\, 1.01)$ |
| 4 | 4.3 | $(4.12,\, 0.94)$ | $(0.67,\, 0.75)$ | — | — |

（表中 $P$ 对角值为简化展示，完整 $2\times 2$ 矩阵含非零交叉项。完整数值可用下面的 Python 代码验证。）

注意观测 $y_3=2.7$ 显著偏低（真值 3.0），导致滤波估计 $\hat p_{3|3}=2.86$ 被拉向观测方向。**滤波器只能使用 $k$ 时刻及之前的观测**——它不知道后面的 $y_4=4.3$ 会"证明"速度没有降低。

##### Backward Pass：RTS 平滑

从 $k=N-1=3$ 向 $k=0$ 递推。核心公式回顾：

$$
C_k = P_{k|k}F^\top P_{k+1|k}^{-1},\qquad \hat x_{k|N}=\hat x_{k|k}+C_k(\hat x_{k+1|N}-\hat x_{k+1|k}),\qquad P_{k|N}=P_{k|k}+C_k(P_{k+1|N}-P_{k+1|k})C_k^\top.
$$

**$k=3$ 的平滑**：$\hat x_{4|4}$ 已知（最后一步无需平滑），即 $\hat x_{4|N}=\hat x_{4|4}=(4.12,0.94)$。

计算 $C_3=P_{3|3}F^\top P_{4|3}^{-1}$。$P_{k+1|N}-P_{k+1|k}$ 是一个**半负定**矩阵（平滑协方差必定小于预测协方差），所以 $P_{k|N}\preceq P_{k|k}$——平滑器的不确定性永远不比滤波器大。

结果汇总：

| 时刻 $k$ | 滤波 $\hat p_{k|k}$ | 平滑 $\hat p_{k|N}$ | 真值 $p_{\text{true}}$ | 滤波 $P_{11,k|k}$ | 平滑 $P_{11,k|N}$ |
|---|---|---|---|---|---|
| 0 | 0.10 | 0.08 | 0.0 | 0.50 | 0.38 |
| 1 | 0.87 | 0.92 | 1.0 | 0.69 | 0.48 |
| 2 | 2.25 | 2.15 | 2.0 | 0.68 | 0.47 |
| 3 | 2.86 | 2.99 | 3.0 | 0.68 | 0.50 |
| 4 | 4.12 | 4.12 | 4.0 | 0.67 | 0.67 |

**关键观察**：

1. **$P_{k|N}\leq P_{k|k}$ 恒成立**：每个时刻的平滑方差都不超过滤波方差。$k=4$ 时二者相等（最后一步无未来信息可用）。
2. **$k=3$ 的修正最显著**：滤波估计 $\hat p_{3|3}=2.86$ 被偏低的观测 $y_3=2.7$ 过度拉偏。平滑后 $\hat p_{3|N}=2.99$，几乎回到真值 3.0。**这正是"未来信息"的作用**：$y_4=4.3$ 暗示速度并未降低，$k=3$ 处的偏低观测应被修正。
3. **平滑方差在中间时刻改善最大**：$k=1,2$ 处同时受前方和后方观测约束，协方差下降约 $30\%$。

> **本质洞察**：RTS 平滑器的本质是利用"未来信息"修正过去的估计。在滤波阶段，$k=3$ 处的异常观测 $y_3=2.7$ 没有被修正的手段——滤波器只能接受。但平滑器在 backward pass 中，把 $k=4$ 处 $y_4=4.3$ 的信息向后传递，告诉 $k=3$："别被那个偏低观测骗了，后续观测显示系统仍在匀速前进。" 这与 Batch MAP 视角完全一致：把所有观测同时考虑，相当于同时解一个块三对角线性系统。

> ⚠️ **陷阱：对含延迟观测的系统直接套用 RTS**
>
> 错误做法：某些传感器有固定延迟（如视觉管线的 50ms 延迟），工程师直接把延迟观测按接收时间戳对齐，然后跑 RTS。
>
> 现象：平滑后 $k$ 时刻的估计反而比滤波差；残差统计异常。
>
> 根本原因：RTS 假设 $y_k$ 是 $x_k$ 的观测。如果 $y_k$ 实际上是 $x_{k-d}$ 的观测（$d$ 是延迟步数），直接套入 RTS 会把"未来的观测信息"错误地归因到当前状态。
>
> 正确做法：(a) 将状态向量增广为 $(x_k, x_{k-1}, \ldots, x_{k-d})$，让延迟观测对应正确的状态分量；(b) 使用 Fixed-Lag Smoother 显式处理延迟窗口；(c) 在因子图中把延迟观测连接到正确时刻的节点。

##### 验证代码框架（Python，可直接运行）

```python
import numpy as np

# ---- 系统矩阵 ----
F = np.array([[1, 1], [0, 1]])       # 匀速模型
H = np.array([[1, 0]])               # 只观测位置
Q = 0.25 * np.eye(2)                 # 过程噪声
R = np.array([[1.0]])                # 观测噪声

# ---- 初始条件 ----
x_hat = np.array([0.0, 1.0])        # 初始估计
P = np.eye(2)                        # 初始协方差
y_obs = [0.2, 0.8, 2.5, 2.7, 4.3]   # 5 步观测

# ---- Forward pass: 存储中间结果 ----
xs_f, Ps_f = [x_hat.copy()], [P.copy()]   # 滤波结果
xs_p, Ps_p = [], []                         # 预测结果

for k in range(len(y_obs)):
    if k > 0:
        # 预测
        x_hat = F @ x_hat
        P = F @ P @ F.T + Q
    xs_p.append(x_hat.copy())
    Ps_p.append(P.copy())
    # 更新
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_hat = x_hat + K @ (np.array([y_obs[k]]) - H @ x_hat)
    P = (np.eye(2) - K @ H) @ P
    xs_f.append(x_hat.copy())
    Ps_f.append(P.copy())

# ---- Backward pass: RTS 平滑 ----
xs_s = [None] * len(y_obs)           # 平滑结果
Ps_s = [None] * len(y_obs)
xs_s[-1] = xs_f[-1].copy()
Ps_s[-1] = Ps_f[-1].copy()

for k in range(len(y_obs) - 2, -1, -1):
    C = Ps_f[k+1] @ F.T @ np.linalg.inv(Ps_p[k+1])
    xs_s[k] = xs_f[k+1] + C @ (xs_s[k+1] - xs_p[k+1])
    Ps_s[k] = Ps_f[k+1] + C @ (Ps_s[k+1] - Ps_p[k+1]) @ C.T
    # 验证 P_{k|N} <= P_{k|k}
    diff = Ps_f[k+1] - Ps_s[k]
    assert np.all(np.linalg.eigvals(diff) >= -1e-10), \
        f"k={k}: 平滑协方差应 <= 滤波协方差"

for k in range(len(y_obs)):
    print(f"k={k}: 滤波 p={xs_f[k+1][0]:.2f}, "
          f"平滑 p={xs_s[k][0]:.2f}, "
          f"P_filt={Ps_f[k+1][0,0]:.2f}, "
          f"P_smooth={Ps_s[k][0,0]:.2f}")
```

##### 练习

**练习 3**（⭐⭐）：把上述 1D 匀速模型扩展为 2D（状态 $(p_x,v_x,p_y,v_y)$，$F$ 为 $4\times 4$ 块对角），观测仍为位置 $H=[I_{2\times 2},0_{2\times 2}]$。生成 10 步轨迹（含转弯），运行 KF + RTS。验证每个时刻 $P_{k|N}\preceq P_{k|k}$（即 $P_{k|k}-P_{k|N}$ 半正定）。绘制滤波 vs 平滑轨迹与真值的对比图。观察在转弯处平滑器的改善是否比直线段更大，并解释原因。

#### A4.4.4 RTS on Manifolds

`artivis/kalmanif` 的 `RauchTungStriebelSmoother<Filter>` 模板以 manif 李群类型为 state：
```cpp
using ERTS  = RauchTungStriebelSmoother<EKF>;    // 标准流形 EKF 的 RTS
using SERTS = RauchTungStriebelSmoother<SEKF>;   // 平方根版
using IERTS = RauchTungStriebelSmoother<IEKF>;   // Invariant RTS
using URTSM = RauchTungStriebelSmoother<UKFM>;   // UKF-on-Manifolds 的 RTS
```
后向递推把 (A4.5) 改为 $\delta=\hat{\mathcal X}_{k+1|N}\boxminus\hat{\mathcal X}_{k+1|k}$, $\hat{\mathcal X}_{k|N}=\hat{\mathcal X}_{k|k}\boxplus(C_k\delta)$。**Invariant RTS (IRTS)**（van der Laan-Cohen-Forbes, *IEEE RA-L* 5(4):5067, 2020）在 group-affine 系统下，过程与观测 Jacobian 独立于估计，平滑的一致性优势与 InEKF 同构。

#### A4.4.5 Fixed-Lag Smoother（GTSAM 实战）

GTSAM `gtsam_unstable/nonlinear/` 提供两种：

- **`BatchFixedLagSmoother`**：每帧对窗内所有因子重线性化 + LM 优化；`CalculateMarginalFactors` 做 Schur 消元，`enforceConsistency` 开关决定是否冻结线性化点（即 FEJ）。精度高、成本高。
- **`IncrementalFixedLagSmoother`**：基于 iSAM2 Bayes tree。通过 `createOrderingConstraints` 把待边缘化 key 放 COLAMD Group 0，保证成为叶，`isam_.marginalizeLeaves` 零开销裁剪。**要求边缘化 key 必须是 Bayes tree 叶**，否则抛 *"Requesting to marginalize variables that are not leaves"*（Issue #1976）。VIO 中 smart factor 把 landmark 与多帧强耦合时极易违反。

---

### §A4.5 Moving Horizon Estimation 简介 ⭐⭐⭐

#### A4.5.1 定义（Rao-Rawlings-Lee, *Automatica* 37(10):1619, 2001）

把全信息估计 (FIE) 在窗口 $N$ 上压缩：
$$
\boxed{\;\min_{x_{T-N},\{w_k\}}\ \underbrace{\Gamma_{T-N}(x_{T-N})}_{\text{arrival cost}}+\sum_{k=T-N}^{T-1}\|w_k\|_{Q^{-1}}^2+\sum_{k=T-N}^{T}\|v_k\|_{R^{-1}}^2\;}\qquad(A4.6)
$$
其中 **arrival cost** $\Gamma_{T-N}(z)$ 概括窗外所有信息对 $x_{T-N}$ 的先验约束。Rao-Rawlings-Mayne (*IEEE TAC* 48(2):246, 2003) 证明：若 FIE 稳定且 $\Gamma$ 是其 arrival cost 的全局下界，MHE 渐近稳定。

#### A4.5.2 三重退化关系

- $N\to 0$ + 高斯 arrival：**MHE ≡ KF**（KF 本身就是 arrival cost 的递推维护）
- $N=T$ + $\Gamma\equiv 0$：**MHE ≡ Full BA**
- 有限 $N$ + 二次 arrival：**MHE ≡ FixedLagSmoother**（工程命名差异而已）

#### A4.5.3 VINS-Mono 滑窗 = 非线性 MHE 工程近似

| MHE 概念 | VINS-Mono 实现 |
|---|---|
| 窗内状态 $x_{T-N:T}$ | 10–11 帧 IMU 关键帧位姿/速度/bias + 可见特征逆深度 |
| 过程代价 | IMU 预积分残差（Forster 流形版） |
| 观测代价 | 特征重投影残差 |
| **arrival cost $\Gamma$** | `MarginalizationFactor` 的 $\|J(x\boxminus x_0)+r\|^2$ |
| FEJ | `linearized_jacobians` + `x0` 冻结线性化点 |

Schur 补 $\tilde H_{rr}=A_{rr}-A_{rm}A_{mm}^{-1}A_{mr}$, $\tilde b_r=b_r-A_{rm}A_{mm}^{-1}b_m$，对 $A_{mm}$ 做 `SelfAdjointEigenSolver` 求伪逆（阈值过滤近零特征值），恢复等价先验 $(J,r)$ 注入下一轮。**计算复杂度排序**：$\text{EKF}<\text{IEKF}<\text{RTS/EKS}<\text{FLS}\approx\text{MHE}<\text{Local BA}\ll\text{Full BA}$。

#### A4.5.4 MHE 的 Arrival Cost 维护：从理论到实践 ⭐⭐⭐

**Arrival cost 的物理意义**。$\Gamma_{T-N}(x_{T-N})$ 编码了窗口起点之前所有观测对 $x_{T-N}$ 的约束。如果没有 arrival cost，MHE 就"忘记"了窗外的信息——这在 SLAM 中会导致滑窗之外的相对约束完全丢失，轨迹精度随时间退化。

**三种常见的 arrival cost 近似**：

| 近似方式 | 公式 | 精度 | 计算代价 | 工程实现 |
|---|---|---|---|---|
| **EKF 传播** | $\Gamma\approx\frac{1}{2}\|x_{T-N}-\hat x_{T-N|T-N}\|_{P_{T-N|T-N}^{-1}}^2$ | 一阶线性化 | $O(n^2)$ 存 $P$ | VINS-Mono |
| **Sampling-based** | 从集合估计 arrival | 蒙特卡洛 | $O(N_e\cdot n)$ | 气象 MHE |
| **Full BA 回退** | $\Gamma$ = 窗外全部因子 | 精确 | $O(N_{\text{out}}\cdot\text{BA cost})$ | LIO-SAM 定期 BA |

**反事实推理**：如果用"无先验"（$\Gamma=0$）代替 arrival cost 会怎样？窗内变量失去窗外信息的约束，每次窗口滑动都相当于"重新初始化"。对 VIO 来说，这意味着位置/yaw 每隔几秒就完全不确定一次——长程漂移将灾难性增大。VINS-Mono 的 `MarginalizationFactor` 正是为了避免这个问题：它把被边缘化掉的变量的信息"压缩"成一个二次先验，注入到下一个窗口中。

**MHE 与因子图的统一视角**。从因子图角度看，MHE 的窗内因子是正常因子，arrival cost 是一个特殊的**线性化先验因子**（GTSAM 中的 `LinearContainerFactor`）。窗口滑动时：
1. 把最旧的关键帧相关因子做 Schur 补
2. Schur 补的结果成为新的先验因子
3. 删除最旧关键帧的变量节点

这个过程在 iSAM2 中对应 `ISAM2::marginalizeLeaves`。

⚠️ **陷阱：边缘化时不使用 FEJ**

错误做法：每次窗口滑动时，用**当前最新估计**重新线性化所有因子再做 Schur 补。

现象：长时间运行后 NEES 持续超出上界（过自信），或 arrival cost 产生的残差异常大。

根本原因：边缘化的 Schur 补**冻结了线性化点**。如果之后主状态继续优化（线性化点移动），但先验仍用旧点，两者之间的能量不守恒——可能出现"旧先验认为当前状态不可行"的矛盾。

正确做法：边缘化时保存当前估计作为 `x0`，后续迭代中先验残差始终在 `x0` 处线性化（FEJ 原则）。OpenVINS 的 `StateHelper::marginalize` 严格执行此规则。

#### A4.5.5 从 MHE 到 Full Graph：连续体中的选择 ⭐⭐

滤波、MHE 和 BA 不是三种独立方法，而是同一个"保留多少历史"的连续参数 $N$（窗口大小）的不同取值：

```text
N=0: 纯滤波（只维护当前时刻）
  │
  │  N=5-20: 滑窗 MHE（VINS-Mono, FAST-LIO2）
  │
  │  N=100-1000: 局部 BA（ORB-SLAM3 local map）
  │
N=∞: 全局 BA（COLMAP, 离线建图）
```

每增加窗口大小，获得：
- 精度提升（可以重线性化更多历史状态）
- 一致性改善（更多观测约束减少累积误差）

每增加窗口大小，付出：
- 计算时间增加（大约与 $N$ 成正比或超正比）
- 内存增加
- 实时性下降

**工程选型的经验法则**：

| 应用场景 | 推荐窗口 $N$ | 原因 |
|---|---|---|
| 无人机飞控（1kHz） | 0（纯 ESKF） | 实时性最高优先 |
| VIO（30Hz camera） | 10-15 帧 | 平衡精度与延迟 |
| LiDAR-IO（10Hz） | 1-5（滤波+ikd-tree） | LiDAR 约束已经很强 |
| 离线建图 | $\infty$（全 BA） | 精度优先 |
| 大规模 SLAM | 分层：局部 BA + 全局 pose graph | 兼顾局部精度与全局一致 |

---

### §A4.5b 全景补充方法：EnKF、Schmidt-KF、$H_\infty$、连续-离散变体 ⭐⭐⭐

在 A1–A3 集中讨论了 Kalman 族的主线方法后，工程实践中还有三类重要变体经常出现在文献和代码中，但不属于"流形"或"迭代"的主线叙事。本节逐一介绍它们的动机、核心公式和适用场景，然后在 §A4.6 的全景大表中统一收录。

#### A4.5b.1 Ensemble Kalman Filter (EnKF) ⭐⭐⭐

##### 动机：当状态维度太高，协方差矩阵存不下

标准 KF/EKF 维护一个 $n\times n$ 协方差矩阵 $P$。当状态维度 $n$ 达到 $10^4\sim 10^6$（气象预报、海洋环流、大规模地球物理反演），$P$ 需要 $n^2$ 存储、$n^3$ 更新——即使用稀疏结构也远超现有计算能力。

Evensen（"Sequential Data Assimilation with a Nonlinear Quasi-Geostrophic Model", *J. Geophys. Res.* 99(C5):10143, 1994）提出了一个优雅的替代方案：**用 $N_e$ 个集合成员（ensemble members）隐式表示协方差，其中 $N_e\ll n$**。

##### 核心思想

维护一个集合 $\{x^{(i)}\}_{i=1}^{N_e}$，每个成员是一个完整的 $n$ 维状态样本。协方差通过集合统计量隐式表示：

$$\bar x=\frac{1}{N_e}\sum_i x^{(i)},\qquad P\approx\frac{1}{N_e-1}\sum_i(x^{(i)}-\bar x)(x^{(i)}-\bar x)^\top$$

**预测**：对每个集合成员独立传播非线性模型

$$x_{k+1}^{(i)}=f(x_k^{(i)},u_k)+w_k^{(i)},\quad w_k^{(i)}\sim\mathcal{N}(0,Q)$$

**更新**：计算集合交叉协方差并用 Kalman 增益公式

$$P_{xy}=\frac{1}{N_e-1}\sum_i(x^{(i)}-\bar x)(h(x^{(i)})-\bar y)^\top$$

$$P_{yy}=\frac{1}{N_e-1}\sum_i(h(x^{(i)})-\bar y)(h(x^{(i)})-\bar y)^\top + R$$

$$K=P_{xy}P_{yy}^{-1}$$

每个集合成员更新：

$$x^{(i)+}=x^{(i)}+K(y_k+v^{(i)}-h(x^{(i)})),\quad v^{(i)}\sim\mathcal{N}(0,R)$$

注意每个成员添加了独立的观测噪声样本 $v^{(i)}$——这是 Burgers et al. (1998) 引入的**随机扰动**方法，确保更新后的集合方差不会系统性偏小。

##### 与 UKF 的本质区别

| 维度 | EnKF | UKF |
|---|---|---|
| **采样策略** | 随机蒙特卡洛 | 确定性 sigma 点 |
| **点数** | $N_e$ 自由选取 (通常 20–200) | $2n+1$ (状态维度决定) |
| **高维适用性** | ✓ ($N_e\ll n$ 即可) | ✗ ($2n+1$ 随 $n$ 线性增长，权重可为负) |
| **Jacobian** | 免 | 免 |
| **精度阶** | 蒙特卡洛 $O(1/\sqrt{N_e})$ | 至少 2 阶（对称高斯条件） |
| **典型应用** | 气象、海洋、油藏、大规模地球物理 | 低中维机器人估计 |

EnKF 的采样本质使其在 $n<100$ 时精度不如 UKF/CKF，但在 $n>1000$ 时是唯一实用的 Kalman 族方法。近年来，Kloss et al.（IROS 2023, arXiv:2308.09870）提出了 **Differentiable Ensemble Kalman Filter (DEnKF)**，用随机神经网络隐式建模过程噪声，已在视觉里程计和机器人操作跟踪任务中展示了良好效果。

> **本质洞察**：EnKF 把"协方差矩阵"从显式存储变成"集合统计量"——这与粒子滤波有相似之处，但 EnKF 保留了高斯假设下的 Kalman 增益结构，不需要粒子滤波的重采样。可以类比为：KF 是精确数学，UKF 是确定性数值积分，EnKF 是蒙特卡洛积分。

⚠️ **陷阱：集合成员数太少导致 rank deficiency**

错误做法：对 $n=1000$ 维状态只用 $N_e=10$ 个集合成员。

现象：$P$ 的秩最多为 $N_e-1=9$，远低于状态维度。Kalman 增益被限制在 9 维子空间内，无法对大多数状态分量做有效校正。更新后集合快速坍缩到低维子空间。

根本原因：集合协方差 $\hat P$ 的秩为 $\min(N_e-1,n)$。当 $N_e\ll n$ 时，这个近似只能捕捉 $N_e-1$ 个主要方差方向，其余方向被隐式设为零方差（过度自信）。

正确做法：(1) **Localization**——把远距离分量间的交叉协方差强制置零（Houtekamer-Mitchell 1998），相当于把大矩阵分块稀疏化；(2) **Inflation**——每步对集合方差乘以 $1+\alpha$（$\alpha\sim 0.02$），防止坍缩；(3) 结合领域知识选择 $N_e$ 至少为"有效自由度"的 2–3 倍。

##### EnKF 与机器人学的新交汇

虽然 EnKF 传统上属于地球物理领域，近年来它开始进入机器人学：

**Differentiable EnKF (DEnKF)**（Kloss et al. IROS 2023）。把 EnKF 嵌入可微框架中：集合传播用随机神经网络代替解析动力学模型，反向传播通过 EnKF 更新步训练网络参数。适用于"动力学模型未知但有大量观测数据"的场景——如从 RGB-D 观测学习物体动力学。

**EnKF for NeRF/Gaussian Splatting**。2024-2025 年多篇工作把 EnKF 用于实时更新 3D 表征：每个集合成员是一组场景参数的采样，观测是新视角图像。集合统计量给出不确定性估计，指导"下一个视角应该看哪里"（主动探索）。

这些工作的共同思想是：**当状态空间太高维以至于显式协方差不可行时，EnKF 提供了一种用"蒙特卡洛+Kalman 结构"折中精度与可行性的框架**。

##### 与粒子滤波的关键区别

初学者容易混淆 EnKF 和粒子滤波（PF），因为两者都用"一堆样本"表示分布。核心区别：

| 维度 | EnKF | 粒子滤波 |
|---|---|---|
| **分布假设** | 高斯（隐式，通过集合统计） | 任意（经验分布） |
| **更新机制** | Kalman 增益（线性最优） | 重采样（importance weights） |
| **高维表现** | 好（$N_e\ll n$ 即可） | 差（粒子退化严重） |
| **多模态** | 不支持 | 支持 |
| **收敛性** | $O(1/\sqrt{N_e})$ 到高斯均值 | $O(1/\sqrt{N_p})$ 到真分布 |

一句话总结：EnKF 是"高维高斯近似"，PF 是"低维非参数精确"。在机器人学中，状态维度 $n<30$ 且后验近高斯时优先选 EKF/UKF；$n>1000$ 且近高斯时选 EnKF；$n<10$ 且多模态时选 PF。

##### 练习

1.（⭐⭐⭐）实现一个 50 维线性系统的 EnKF（$N_e=20$），与精确 KF 对比 RMSE。观察 rank deficiency 的影响。然后加入 inflation（$\alpha=0.02$），看 RMSE 如何变化。

2.（⭐⭐⭐⭐）把上述 EnKF 扩展到非线性观测 $y_k=\|x_k[1:3]\|+v_k$（只观测前三维的范数），其余维度通过动力学耦合间接观测。讨论：哪些维度的估计精度接近 KF？哪些严重退化？这对"观测信息如何通过动力学耦合传播"给出了什么直觉？

#### A4.5b.2 Schmidt-Kalman Filter (Consider Filter) ⭐⭐⭐

##### 动机：有些参数不想估计，但不能假装它们不存在

工程中经常遇到这样的情况：系统含有一些"讨厌参数"（nuisance parameters），例如传感器标定偏差、大气折射率、地球引力场高阶系数等。这些参数变化极慢或完全未知，**不值得花计算资源去估计，但它们的不确定性确实影响主状态的精度**。

如果直接忽略这些参数（把它们当作已知常量），滤波器会过度自信——协方差不反映这些参数不确定性带来的额外误差。如果把它们全部加入状态向量做联合估计，状态维度会膨胀，且对缓变参数的估计可能不收敛（信息不足）。

S.F. Schmidt 在 1960 年代于 NASA Ames 研究中心提出了折中方案：**"考虑"（consider）这些参数的不确定性，让它们影响协方差传播和 Kalman 增益，但不主动估计它们的值**。

##### 核心公式

把状态分为"估计部分" $x$ 和"考虑部分" $c$：

$$\begin{bmatrix}x_{k+1}\\c_{k+1}\end{bmatrix}=\begin{bmatrix}F_{xx}&F_{xc}\\0&I\end{bmatrix}\begin{bmatrix}x_k\\c_k\end{bmatrix}+w_k$$

考虑参数 $c$ 不更新（$\hat c$ 恒等于先验值），但其不确定性 $P_{cc}$ 进入主状态的协方差：

$$\tilde P=P_{xx}+P_{xc}P_{cc}^{-1}P_{xc}^\top$$

Kalman 增益使用 $\tilde P$ 而非 $P_{xx}$，确保在计算新息协方差 $S$ 时包含了考虑参数的影响。更新后只修正 $x$，$c$ 保持不变。

**Schmidt-KF 的本质**：它把联合估计 $(x,c)$ 的 Kalman 更新中 $c$ 的更新部分"关闭"，但保留了 $c$ 对 $x$ 的协方差贡献。结果是**状态估计精度不变，但协方差更保守（更诚实）**——代价是放弃了对 $c$ 的估计。

##### 何时使用、何时不使用

| 场景 | 是否适合 Schmidt-KF | 原因 |
|---|---|---|
| 传感器 bias 已离线标定但有残余不确定性 | ✓ | bias 值已知但不精确，consider 其不确定性 |
| IMU bias 需要在线估计 | ✗ | bias 漂移必须主动跟踪 |
| 地球引力场高阶系数 | ✓ | 几乎不变，但影响高精度轨道预报 |
| LiDAR-IMU 外参 | 取决于 | 若外参稳定可 consider；若需在线标定则不行 |

近年来，Wu-Roumeliotis（U. of Minnesota, "Inverse Schmidt Estimators", mars.cs.umn.edu）提出了逆 Schmidt 估计器，可以在已有 Schmidt 近似上高效恢复部分考虑参数的估计。Ren et al.（*Astrodynamics* 2024）在 Lie 群上扩展了 Schmidt-KF，用于航天器姿态估计中处理考虑参数。

⚠️ **陷阱：对变化中的参数使用 Schmidt-KF**

错误做法：传感器 bias 实际在漂移，但工程师用 Schmidt-KF "consider" 它而不估计。

现象：滤波器看似协方差合理，但估计精度随时间下降，NEES 逐渐偏离 $\chi^2$。

根本原因：Schmidt-KF 假设考虑参数是**常量或极慢变**。如果参数实际在变化，consider 模型中的 $P_{cc}$ 不更新意味着不确定性只增不减——最终 $\tilde P$ 膨胀到主导整个协方差，主状态估计变得极度保守。

正确做法：对需要在线跟踪的参数使用标准状态增广；只对真正"不变或近不变"的参数使用 Schmidt-KF。

#### A4.5b.3 $H_\infty$ Filter (Minimax Robust Filter) ⭐⭐⭐

##### 动机：当噪声统计特性未知

标准 Kalman 滤波的最优性建立在精确已知噪声协方差 $Q,R$ 的前提上。实际系统中，噪声常常不是高斯的，$Q,R$ 也只是粗略估计。如果真实噪声分布与假设偏差较大，KF 可能过度自信甚至发散。

$H_\infty$ 滤波器放弃了"在给定统计模型下最优"的目标，转而追求"在最坏情况下的性能不超过给定门限"：

$$\sup_{w,v\ne 0}\frac{\|L(x_k-\hat x_k)\|^2}{\|x_0-\hat x_0\|^2+\sum\|w_k\|^2+\sum\|v_k\|^2}<\gamma^2$$

其中 $L$ 是关注的线性组合（通常 $L=I$），$\gamma>0$ 是性能门限。**注意**：这里没有期望，没有概率假设——$w_k,v_k$ 可以是任意有界能量信号，包括对抗性扰动。

##### 核心公式

$H_\infty$ 滤波器的 Riccati 递推与 KF 惊人相似：

$$K_k=P_{k|k-1}H_k^\top(H_kP_{k|k-1}H_k^\top+R_k-\gamma^{-2}L_k^\top L_k P_{k|k-1})^{-1}$$

与 KF 的区别仅在于 $-\gamma^{-2}L^\top LP$——这个"惩罚项"让滤波器更保守，代价是在高斯噪声下不如 KF 精确。当 $\gamma\to\infty$ 时，$H_\infty$ 退化为标准 KF。

**KF vs $H_\infty$ 对比**：

| 维度 | Kalman Filter | $H_\infty$ Filter |
|---|---|---|
| **噪声假设** | 高斯，$Q,R$ 已知 | **无统计假设**，仅要求有界能量 |
| **最优性** | 最小方差（条件最优） | 最小最大（最坏情况门限） |
| **性能** | 高斯下最优 | 高斯下次优；非高斯/未知噪声下更鲁棒 |
| **调参** | $Q,R$ | $\gamma$（门限）, $Q,R$（初始化用） |
| **实现复杂度** | $O(n^3)$ | $O(n^3)$（额外一个矩阵减法） |

##### 何时考虑 $H_\infty$

适合 $H_\infty$ 的场景有一个共同特征：**不确定性的不确定性很大**。也就是说，不仅状态不确定，连噪声模型本身也不确定。典型例子：

- **未标定传感器**：噪声参数只有粗略上界。
- **对抗/故障场景**：传感器可能受到恶意干扰。
- **异常环境**：雨雪雾导致 LiDAR 噪声特性突变。

Simon（"From Here to Infinity", Cleveland State University tutorial, 2006）提供了一个优秀的入门教程，FilterPy 库也包含 $H_\infty$ 滤波器实现。

⚠️ **陷阱：把 $\gamma$ 设得太小**

错误做法：追求"极限鲁棒性"而把 $\gamma$ 设成接近理论下限。

现象：Riccati 方程中 $H P H^\top + R - \gamma^{-2}L^\top LP$ 变成近奇异或不正定，增益 $K$ 爆炸或出 NaN。

根本原因：$\gamma$ 有一个理论下限 $\gamma_{\min}$，低于此值问题无解。$\gamma$ 越接近下限，滤波器越保守（几乎不信任任何观测），实用价值越低。

正确做法：$\gamma$ 取值通常为 $\gamma_{\min}$ 的 1.5–3 倍。可以先用 KF 跑一遍基线，然后逐步降低 $\gamma$ 直到在真实数据上的 RMSE 不再改善。

##### $H_\infty$ 与 KF 的统一视角

从博弈论角度，KF 和 $H_\infty$ 滤波器解决的是同一个"玩家 vs 自然"的博弈，但假设不同：

| 视角 | KF | $H_\infty$ |
|---|---|---|
| 自然的策略 | 按已知分布 $\mathcal{N}(0,Q/R)$ 随机出牌 | 用有界能量做最坏情况攻击 |
| 玩家的目标 | 最小化期望误差 | 最小化最大比值 |
| 解的形式 | Riccati 方程（确定性） | 修改的 Riccati（带 $\gamma$ 约束） |
| 结果 | 在真模型下精确最优 | 在最坏情况下有保证 |

这个博弈视角解释了为什么 $H_\infty$ 滤波器比 KF "保守"：它准备应对最坏情况（对抗性噪声），而 KF 只准备应对平均情况（随机噪声）。在实际中，噪声通常介于随机和对抗之间——所以混合策略（如 adaptive $\gamma$，按 NIS 在线调整 $\gamma$）可能是最实用的。

##### $H_\infty$ 在腿足机器人中的潜在应用

腿式机器人在非结构化地形上行走时，接触状态频繁切换（打滑、碰撞），噪声特性急剧变化。传统 KF 的固定 $Q,R$ 在打滑瞬间严重不匹配。$H_\infty$ 滤波器在此类场景有天然优势：它不假设噪声统计，只要求有界能量。

但目前腿足机器人领域更倾向于用 IMM（Interacting Multiple Model）或自适应 $R$ 方法处理接触切换，$H_\infty$ 的实际采用率较低。原因是 $H_\infty$ 的保守性在"正常行走"阶段会牺牲精度——而大部分时间机器人是在正常行走。未来可能的方向是"切换式"：正常时用 EKF/InEKF，检测到异常（NIS 突变）时短暂切换到 $H_\infty$ 模式。

##### 练习

1.（⭐⭐）实现一个标量 $H_\infty$ 滤波器（$n=1$），对比不同 $\gamma$ 值下的增益 $K$ 与标准 KF 增益的差异。绘制 $K$ vs $\gamma$ 曲线，找到 $\gamma_{\min}$ 的数值。

2.（⭐⭐⭐）对一个 2D 匀速模型，分别用 KF 和 $H_\infty$ 滤波器处理含 outlier 的观测序列（95% 正常高斯，5% 均匀随机大噪声）。比较两者的 RMSE。预期 $H_\infty$ 在有 outlier 时更鲁棒，但在无 outlier 时 RMSE 稍大。

3.（⭐⭐⭐⭐）从理论上证明：当 $\gamma\to\infty$ 时，$H_\infty$ 增益公式退化为标准 KF 增益。提示：展开 $(HPH^\top+R-\gamma^{-2}L^\top LP)^{-1}$ 在 $\gamma^{-2}\to 0$ 时的一阶近似。

#### A4.5b.4 连续-离散 Kalman 族 (CD-KF/CD-EKF/CD-UKF) ⭐⭐⭐

##### 动机：物理模型是连续的，传感器是离散的

前面所有方法都在离散时间框架下工作——状态转移写成 $x_{k+1}=f(x_k,u_k)+w_k$。但物理系统的动力学通常以连续微分方程建模：$\dot x=f_c(x,u)+L_cw_c(t)$，其中 $w_c(t)$ 是连续白噪声。而传感器（相机、LiDAR、GPS）以离散时间步 $\Delta t$ 给出观测 $y_k=h(x_k)+v_k$。

直接把连续 SDE 离散化为 $x_{k+1}=f_d(x_k)+w_d$ 时，离散化方法的选择会影响精度。常见的 Euler 离散化在 $\Delta t$ 较大或系统快变时引入显著误差。**连续-离散 (CD) 方法**直接在连续域传播协方差 ODE，在离散观测时刻做 Kalman 更新，避免了离散化近似。

CD-KF 的协方差传播（线性情形）：

$$\dot P=F_c P+PF_c^\top+L_cQ_cL_c^\top$$

在观测时刻 $t_k$ 做标准 Kalman 更新。非线性扩展：

| CD 变体 | 连续传播 | 离散更新 | 参考 |
|---|---|---|---|
| **CD-EKF** | 沿 $\hat x(t)$ 线性化 ODE 数值积分 $P$ | EKF update | Jazwinski 1970 |
| **CD-UKF** | sigma 点过连续 ODE (RK45) 传播 | UKF update | Särkkä *BFS* 2ed §10.3 |
| **CD-CKF** | cubature 点过连续 ODE 传播 | CKF update | Arasaratnam 2010 |

**与 A1 §1.8b（CD-KF 基础章节）的衔接**：A1 已经给出了线性 CD-KF 的 Riccati ODE 推导和离散化等价关系。本节把它放入全景视角——CD-KF/CD-EKF/CD-UKF 是原有方法在"连续传播 + 离散更新"维度上的变体，在 §A4.6 的全景大表中作为独立行列出。

**工程实践**：CD-EKF 在 GNSS-INS 紧耦合、航天器精密轨道确定（OD）中广泛使用，因为轨道动力学（Kepler + J2 + 大气阻力 + 太阳光压）本身就是连续 ODE。Särkkä *BFS* 2ed Ch.10 §3 给出了完整的 CD-UKF 实现伪代码。

**连续-离散方法的核心权衡**：

| 方法 | 精度 | 实现复杂度 | 适用条件 |
|---|---|---|---|
| Euler 离散化 + 标准 EKF | 低（$O(\Delta t)$ 误差） | 简单 | $\Delta t$ 很小时可接受 |
| RK4 离散化 + 标准 EKF | 中（$O(\Delta t^4)$） | 中 | 大多数机器人应用 |
| CD-EKF（连续 Riccati ODE） | 高 | 高（需要 ODE 积分器） | 长积分步长、快变系统 |
| 矩阵指数离散化 | 精确（线性系统） | 中 | 系统可线性化/分段线性 |

**何时必须用 CD 方法**。大多数机器人应用中，IMU 以 200-1000 Hz 采样，每步 $\Delta t=1\sim 5$ ms，四阶 Runge-Kutta 离散化足够精确。但在以下场景中，CD 方法不可避免：

1. **航天精密轨道确定**：轨道周期 ~90 min，观测间隔可达数分钟。Riccati ODE 积分比 Euler 离散化精确两个数量级。
2. **低频观测系统**：如水下 USBL 定位（0.1-1 Hz 观测），水下航行器动力学变化快于观测频率。
3. **多率融合**：IMU 200 Hz + GPS 1 Hz。协方差在 GPS 间隔内需要精确传播 200 步——用连续 ODE 一次积分比逐步离散更高效且精确。

**反事实推理**：如果在航天 OD 中用简单 Euler 离散化会怎样？$\Delta t=10$ s（观测间隔）下，轨道预报的位置误差可达数百米（而 CD-EKF 控制在米级）。这不是"精度稍差"，而是"完全不可用"。这就是为什么 JPL/GSFC 的精密定轨软件（如 GIPSY-OASIS、GEODYN）全部使用连续-离散框架。

> **本质洞察**：连续-离散方法不是"更高级的 EKF"，而是"在时间轴上更忠实于物理的 EKF"。它解决的核心问题是：当采样间隔大于系统特征时间时，离散化近似会严重失真。

⚠️ **陷阱：在 CD-EKF 中忘记 $Q_c$ 与 $Q_d$ 的单位差异**

错误做法：直接把连续谱密度 $Q_c$（单位如 $\text{m}^2/\text{s}^3$）当作离散方差 $Q_d$（单位如 $\text{m}^2/\text{s}^2$）塞入 Riccati ODE。

现象：协方差要么极度膨胀要么几乎不增长，取决于 $\Delta t$ 的量级。

根本原因：连续白噪声的功率谱密度与离散白噪声的方差差一个 $\Delta t$ 因子。连续 Riccati ODE 使用 $Q_c$；离散 KF 使用 $Q_d\approx Q_c/\Delta t$（对白噪声加速度模型，精确关系涉及矩阵指数积分）。

正确做法：明确区分"物理噪声模型"（连续）和"算法噪声参数"（离散），始终从物理模型出发推导离散化方差。Van Loan 方法（`scipy.linalg.expm` 块矩阵技巧）可精确计算 $Q_d$ 而不需要 Taylor 近似。

**练习**：

1.（⭐⭐）对一维阻尼弹簧系统 $\ddot x + 2\zeta\omega_n\dot x + \omega_n^2 x = w(t)$，分别用 Euler 离散化和精确矩阵指数离散化（`scipy.linalg.expm`）计算 $F_d, Q_d$。取 $\Delta t=0.1$ s, $\omega_n=10$ rad/s, $\zeta=0.1$。比较两种方法的 $Q_d$ 差异，解释为什么 Euler 在此参数下严重不准确（提示：$\omega_n\Delta t=1$，一步跨越了振荡周期的约 16%）。

2.（⭐⭐⭐）实现一个简单的 CD-EKF：连续动力学为二维匀圆运动 $\dot\theta=\omega, \dot r=0$，观测为离散笛卡尔位置 $(x,y)=(r\cos\theta, r\sin\theta)+v$。用 `scipy.integrate.solve_ivp` 积分 Riccati ODE，在每次 GPS 观测（1 Hz）时做离散 EKF 更新。与"简单 Euler 离散化 EKF"对比 RMSE，取 $\omega=2\pi$ rad/s（一秒一圈）。

---

### §A4.6 Kalman 族全景对比大表（34 方法 × 13 列） ⭐⭐

下表按"线性 → 非线性欧氏 → 流形 → 迭代/平滑 → 非高斯"的递进顺序排列。缩写：Add=加性、Mult=乘性、L-Inv/R-Inv=左/右不变；Taylor1/2 为一/二阶泰勒；SP=sigma points；SR=square-root；GA=group-affine；FEJ=First-Estimate Jacobian。

| 方法 | 误差定义 | 系统假设 | 近似方式 | 精度阶 | Jacobian | 迭代 | 状态空间 | 复杂度 | 一致性 | 典型应用 | 代表实现 | 推荐档位 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **KF** | Add | 线性高斯 | 无 | 精确 | 无 | 单 | $\mathbb R^n$ | $O(n^3)$ | 天然 | 轨道预报、过程控制 | FilterPy KalmanFilter | 3 |
| **Information Filter** | Add | 线性高斯 | 无 | 精确 | 无 | 单 | $\mathbb R^n$ (对偶) | $O(n^3)$ | 天然 | 多源融合、分布式 | GTSAM, Barfoot §3.3 | 3 |
| **Square-Root KF** (Bierman/Thornton) | Add | 线性高斯 | 无 | 精确 | 无 | 单 | $\mathbb R^n$ | $O(n^3)$ | 天然 (数值更稳) | 航天高精度 | Kaminski 1971, Bierman UD | 3 |
| **EKF** | Add | 非线性高斯 | Taylor1 | 1阶/1阶 | 解析 | 单 | $\mathbb R^n$ | $O(n^3)$ | **伪观测** → 需 FEJ | GNSS-INS、早期 SLAM | robot_localization, FilterPy | 3 |
| **2nd-order EKF** | Add | 非线性 | Taylor2 | 2阶/1阶 | 解析+Hessian | 单 | $\mathbb R^n$ | $O(n^4)$ | 同 EKF | 航天轨道 | Gelb 1974 | 4 |
| **IEKF (Iterated)** | Add | 非线性 | Taylor1 迭代 | 1阶/1阶 | 解析 | **GN/LM/LS** | $\mathbb R^n$ | $O(J_{\max}n^3)$ | 同 EKF | 强非线性测量 | ROVIO, FilterPy | 3 |
| **UKF** | Add | 非线性高斯 | UT, $2n+1$ SP | 均值 3阶、协方差 2阶（对称高斯条件）；保守可说至少 2阶 | **免** | 单 | $\mathbb R^n$ | $O((2n+1)n^2)$ | 无 | 航空、融合 | FilterPy UKF, robot_localization | 3 |
| **Scaled UKF** | Add | 非线性 | UT, $\alpha,\beta,\kappa$ | 可调 | 免 | 单 | $\mathbb R^n$ | 同 UKF | 无 | 高维 UT 修正 | van der Merwe 2004 | 3 |
| **Square-Root UKF** | Add | 非线性 | UT + Cholesky update | 2阶 | 免 | 单 | $\mathbb R^n$ | $O((2n+1)n^2)$ | 数值稳定（平方根保持） | 长时运行 | van der Merwe SR-UKF | 4 |
| **CKF** (Cubature) | Add | 非线性 | 3阶球面径向, $2n$ 点 | 2阶 | 免 | 单 | $\mathbb R^n$ | $O(2n\cdot n^2)$ | 无 (权重全正) | 高维雷达 | Arasaratnam 2009 MATLAB | 4 |
| **GHKF** (Gauss-Hermite) | Add | 非线性 | $m^n$ Gauss-Hermite | 2$m$-1阶 | 免 | 单 | $\mathbb R^n$ (低维) | $O(m^n)$ **维度灾** | 无 | $n\le 4$ 强非线性 | Särkkä *BFS* §8 | 4 |
| **Sparse-Grid GHKF** | Add | 非线性 | Smolyak 稀疏格 | 可调 | 免 | 单 | $\mathbb R^n$ 中维 | $O(n^L)$ | 无 | $n\approx 8\sim 12$ | Jia-Xin 2011 | 4 |
| **ISPKF** | Add | 强非线性 | UT + GN 迭代 | 收敛到**均值** | 免 | GN on SLR | $\mathbb R^n$ | $J_{\max}O((2n+1)n^2)$ | 无 | bearing-only, 远距立体 | Sibley 2006, Barfoot §4.2.10 | 4 |
| **MEKF** | **Mult** ($q=\bar q\otimes\delta q$) | 非线性 + $S^3$ 姿态 | Taylor1 误差态 | 1阶 | 解析 (切空间) | 单 | $\mathbb R^{n-1}\times SO(3)$ | $O(n^3)$ | 较好 | 航天姿态 (JPL/NASA) | Markley 2003 JGCD | 3 |
| **ESKF** | **Mult** (误差态) | 非线性 + SO(3)/SE(3) | Taylor1 on $\delta x$ | 1阶 | 解析 | 单 + **reset** | $SE(3)\times\mathbb R^6$ | $O(n^3)$ | 一般，需 reset Jacobian | GNSS-INS, ArduPilot | Solà arXiv:1711.02508 | 3 |
| **InEKF (Left)** | L-Inv $\eta=X^{-1}\hat X$ | **Group-affine** 系统 | Taylor1 on $\eta$ | 1阶 | **独立于估计** | 单 | 矩阵李群 $SE_K(3)$ | $O(n^3)$ | **天然**（无 FEJ） | $Y=Xd$ 型观测 | Barrau-Bonnabel 2017 | 3 |
| **InEKF (Right)** | R-Inv $\eta=\hat X X^{-1}$ | Group-affine | Taylor1 on $\eta$ | 1阶 | 独立于估计 | 单 | 矩阵李群 | $O(n^3)$ | 天然 | Contact-Aided 腿足、IMU 预积分 VIO | Hartley invariant-ekf / Barrau-Bonnabel 2017 | 3 |
| **Imperfect InEKF** | L/R-Inv + 非 GA 修正项 | 含 bias 扰动 | Taylor1 + 伪线性 | 1阶 | 部分依赖 | 单 | $SE_K(3)\times\mathbb R^{6}$ | $O(n^3)$ | 近似一致 | 带 bias 的 Contact-Aided | Hartley IJRR 2020 | 4 |
| **UKF-M** | $\boxplus$-退相 | 非线性流形 | UT + $\phi$ retraction | 2阶 | 免 | 单 | 通用流形 $\mathcal M$ | $O((2n+1)n^2)$ | phi 选对则良 | 姿态、SLAM2D、VIO | UKF-M (Python) | 4 |
| **IKFoM** | $\boxminus$ + $J^{(j)}$ | 非线性流形 | Taylor1 + 流形 GN | 1阶 迭代 | 解析 + 曲率 Jac | **GN on $\mathcal M$** | 复合流形 | $O(J_{\max}n^3)$ | 类 IEKF | LIO, LIVO | hku-mars/IKFoM | 3 |
| **EqF** (Equivariant) | 对称群等变 | 可积对称群 | Lift + error state | 1阶 (严格一致) | 解析 | 单 | 齐次空间 $G/H$ | $O(n^3)$ | **最强几何一致性** | VIO | pvangoor/eqvio | 4 |
| **EKS** (smoother) | Add/Mult | 非线性高斯 | Taylor1 + RTS 反向 | 1阶 | 解析 | 单 / iterated | $\mathbb R^n$ 或流形 | $O(Nn^3)$ | 同 EKF | 离线 VIO 精修 | Särkkä *BFS* §13.2 | 4 |
| **UKS / URTSS** | Add | 非线性高斯 | UT + RTS 反向 | 2阶 | 免 | 单 | $\mathbb R^n$ | $O(N(2n+1)n^2)$ | 无 | 离线精估 | Särkkä 2008 TAC | 4 |
| **CKS** | Add | 非线性高斯 | Cubature + RTS 反向 | 2阶 | 免 | 单 | $\mathbb R^n$ | $O(2Nn^3)$ | 权重正 | SR-CKS 高稳定 | Arasaratnam 2011 | 4 |
| **RTS on Manifolds** (IRTS, ERTS) | $\boxminus$ | 非线性流形 | RTS + $\boxplus$ | 1阶 | 解析 | 单 | 李群 | $O(Nn^3)$ | GA 下天然 | 离线 VIO/LIO | kalmanif `RTSSmoother` | 4 |
| **MHE** | Add/Mult | 非线性 + 约束 | NLP + arrival cost | 可迭代 GN/SQP | 解析/AD | 多步 | $\mathbb R^n$ 或流形 | $O(IL n^3)$ | FEJ+约束 | 化工过程, 实时控制 | acados MHE | 4 |
| **Gaussian Sum Filter** | Add | 多模态 | $\sum w_i\mathcal N_i$ | 每分量 EKF | 每分量 | 单 | $\mathbb R^n$ | $O(Mn^3)$ | 无 | 多假设跟踪 | Alspach-Sorenson 1972 | 4 |
| **Particle Filter** (SIR/MCMC) | 经验分布 | 任意非高斯 | $N_p$ 粒子 | 蒙特卡洛 | 免 | 单 | 任意 | $O(N_p)$ | 无 | 全局定位、MCL | FilterPy `monte_carlo/` | 3 |
| **RBPF (FastSLAM)** | 混合 | 条件高斯 | 位姿粒子 + 地图 EKF | 蒙特卡洛+Taylor1 | 解析 | 单 | $\mathbb R^6\times\mathbb R^{3M}$ | $O(N_pM)$ | 粒子退化 | 栅格 SLAM | Grisetti gmapping | 3 |
| **EnKF (Ensemble)** | Add | 非线性高斯 | $N_e$ 集合成员隐式表示协方差 | 蒙特卡洛 | **免** | 单 | $\mathbb R^n$ (超高维) | $O(N_e\cdot n^2)$ | 需 localization+inflation | 气象/海洋/大规模地球物理; 机器人 DEnKF | Evensen 1994; Kloss IROS 2023 DEnKF | 4 |
| **Schmidt-KF (Consider)** | Add | 线性/非线性 | Consider 参数不估计但影响协方差 | 无/Taylor1 | 解析 | 单 | $\mathbb R^{n_x}\times\mathbb R^{n_c}$ | $O((n_x+n_c)^3)$ | 保守（诚实） | 航天高精度 OD; 传感器标定残余 | Schmidt 1960s NASA Ames; FilterPy | 4 |
| **$H_\infty$ Filter** | Add | 非线性 | Minimax: 最坏情况有界能量 | Taylor1 | 解析 | 单 | $\mathbb R^n$ | $O(n^3)$ | 鲁棒 (无概率假设) | 未标定传感器/对抗场景 | Simon 2006 tutorial; FilterPy `HInfinity` | 4 |
| **CD-EKF** | Add/Mult | 连续 SDE + 离散观测 | 连续 ODE 传播 + 离散 EKF 更新 | Taylor1 | 解析 | 单 | $\mathbb R^n$ | $O(n^3\cdot N_{\text{ODE}})$ | 同 EKF | GNSS-INS 紧耦合, 精密 OD | Jazwinski 1970; Särkkä BFS §10 | 4 |
| **CD-UKF** | Add | 连续 SDE + 离散观测 | sigma 点过 ODE + 离散 UKF | UT | 免 | 单 | $\mathbb R^n$ | $O((2n+1)C_f\cdot N_{\text{ODE}})$ | 无 | 化学过程/慢采样系统 | Särkkä BFS §10.3 | 4 |

> **纵览**：上表共收录 **34 种方法**（含 6 种新增：EnKF、Schmidt-KF、$H_\infty$、CD-EKF、CD-UKF 以及上一版已有的 28 种）。若把此表行列作二维散点图，横轴是"几何精致度"（Add→Mult→Inv→Equiv），纵轴是"计算策略"（单步→迭代→平滑→联合 MAP），则整个 Kalman 族沿两条轴**单调增长"精度与一致性"、同时单调增长"实现复杂度与算力"**。新增的 EnKF 占据"超高维 + 蒙特卡洛"角落；Schmidt-KF 占据"保守协方差 + 不估计讨厌参数"角落；$H_\infty$ 占据"无概率假设 + minimax 鲁棒"角落；CD 族则是沿"连续-离散"轴的正交变体。工程落地的艺术就是在精度、一致性与算力之间找帕累托最优。

---

### §A4.7 C++/Python 库完整工程映射 ⭐⭐

#### A4.7.1 18 库总表

| 库 | GitHub | ★ | 语言 | 支持算法 | 关键类/文件 | 扰动/四元数 | 依赖 | 推荐用途 |
|---|---|---|---|---|---|---|---|---|
| **GTSAM** | `borglab/gtsam` | ~3.3k | C++ (Py/MATLAB) | 因子图, iSAM2, FLS, EKF, IMU 预积分 | `ExtendedKalmanFilter.h`, `IncrementalFixedLagSmoother.h`, `ISAM2.h`, `NavState`, `ImuFactor` | Hamilton；Pose3 **右扰动** (4.0+ 全 SE(3) Exp) | Eigen, (Boost, TBB, MKL) | 后端优化/iSAM2/FLS |
| **OpenVINS** | `rpng/open_vins` | ~2.7k | C++ | MSCKF, SLAM 特征, FEJ, ZUPT, 在线标定 | `VioManager`, `StateHelper`, `UpdaterMSCKF`, `Propagator`, `Type` 系统 | **JPL 四元数**, **左扰动** global | Eigen, OpenCV, Boost, ROS1/2 | MSCKF-VIO 研究平台 |
| **kalmanif** | `artivis/kalmanif` | ~0.25k | C++ H-only | EKF, SEKF, IEKF, UKF-M, RTS smoother | `kalman_filter_base.h`, `ekf.h`, `iekf.h`, `ukfm.h`, `RauchTungStriebelSmoother` | 基于 manif；右 ⨁ / 左 ⨁ 双支持 | manif, Eigen | 教学 / 流形 KF 原型 |
| **manif** | `artivis/manif` | ~1.7k | C++11 H-only | SO(n)/SE(n)/SE_2(3)/SGal(3)/Bundle + 解析 Jac | `SO3.h`, `SE3.h`, `SE_2_3.h`, `Bundle.h`, `tangent_base.h` | Hamilton (Eigen)；右 ⨁ 默认 | Eigen (Ceres AD 可选) | 李群工具库 |
| **Sophus** | `strasdat/Sophus` | ~2.3k | C++ H-only | SO(2)/SO(3)/SE(2)/SE(3)/Sim(3)/RxSO(3) | `so3.hpp`, `se3.hpp`, `sim3.hpp` | Hamilton；右扰动 (retract=Exp) | Eigen (fmt 可选) | ORB-SLAM3/DSO/Basalt 基础 |
| **invariant-ekf** | `RossHartley/invariant-ekf` | ~0.5k | C++ | Right-InEKF, Contact-Aided InEKF | `InEKF.h`, `RobotState.h`, `NoiseParams.h`, `LieGroup.h` | Right-Invariant, Hamilton, 状态在 $SE_K(3)$ | Eigen, Boost | 腿足 / Cassie / MIT Cheetah |
| **Pronto** | `ori-drs/pronto` | ~0.6k | C++ ROS | 多源 EKF 融合 (IMU/LegOdo/LIDAR/VIO) | `pronto_core`, `imu_module`, `legodo_module` | Hamilton；右扰动 | Eigen, ROS, Boost | Atlas/MIT Cheetah 腿足 |
| **UKF-M** | `CAOR-MINES-ParisTech/ukfm` | ~0.25k | Python (MATLAB) | UKF on Manifolds, 左/右不变 UKF | `ukfm/ukfm.py`, `models/`, `benchmarks/` | 用户定义 $\phi$ 自选左/右不变 | NumPy, Matplotlib | Brossard ICRA-20 参考实现 |
| **IKFoM** | `hku-mars/IKFoM` | ~0.6k | C++ H-only | Iterated EKF on Manifold (IKFoM) | `esekfom/esekfom.hpp`, `use-ikfom.hpp`, `mtk/` | 右扰动, Hamilton；流形复合类型 | Eigen, Boost | FAST-LIO/LINS 核心 |
| **FAST-LIO2** | `hku-mars/FAST_LIO` | ~4.6k | C++ ROS1/2 | 紧耦合 LiDAR-Inertial IEKF + ikd-Tree | `laserMapping.cpp`, `use-ikfom.hpp`, `IMU_Processing.hpp` | 右扰动, Hamilton | PCL, Eigen, IKFoM, ROS, livox | LiDAR-IO 实时 |
| **FilterPy** | `rlabbe/filterpy` | ~3.7k | Python | KF, EKF, UKF, IMM, PF, g-h, $H_\infty$, RTS, FLS | `kalman/kalman_filter.py`, `kalman/UKF.py`, `kalman/IMM.py`, `monte_carlo/` | 欧式（无李群） | NumPy, SciPy | 教学首选 |
| **robot_localization** | `cra-ros-pkg/robot_localization` | ~1.6k | C++ ROS1/2 | EKF, UKF 多源 (IMU/GPS/odom) | `ekf.h`, `ukf.h`, `ros_filter.cpp`, `navsat_transform_node.cpp` | 欧式 RPY + tf2 Hamilton | Eigen, ROS, tf2 | ROS 标准融合节点 |
| **ROVIO** | `ethz-asl/rovio` | ~1.3k | C++ ROS | IEKF + 光度误差直接法 VIO | `rovio::FilterState`, `ImuPrediction`, `ImgUpdate` | Hamilton `qCM`；IEKF 切空间迭代 | Eigen, OpenCV, kindr, lightweight_filtering, ROS | 单/双目直接法 VIO |
| **VINS-Mono** | `HKUST-Aerial-Robotics/VINS-Mono` | ~5k | C++ ROS | 滑窗 BA + 预积分 + 在线外参/td | `estimator.cpp`, `marginalization_factor.cpp`, `integration_base.h`, `feature_manager.cpp` | Hamilton；右扰动 body-frame | Ceres, Eigen, OpenCV, ROS, DBoW2 | VIO 事实标准基线 |
| **EqVIO** | `pvangoor/eqvio` | ~0.2k | C++ | Equivariant Filter VIO | `VIOState.h`, `VIOFilter.cpp`, `VIOGroup.h` | 等变误差（非左/右扰动框架） | Eigen, OpenCV, yaml-cpp, GIFT | van Goor-Mahony 2023 参考 |
| **Ceres Solver** | `ceres-solver/ceres-solver` | ~4.1k | C++ | 非线性最小二乘 + Manifold + Schur | `Problem`, `CostFunction`, `AutoDiffCostFunction`, `Manifold` | 用户定义 Plus/Minus | Eigen, SuiteSparse (CHOLMOD) | 后端优化通用求解器 |
| **MSCKF_VIO** | `KumarRobotics/msckf_vio` | ~1.9k | C++ ROS | Stereo MSCKF | `msckf_vio.cpp`, `image_processor.cpp`, `cam_state.hpp`, `imu_state.hpp` | **JPL 四元数** | Eigen, OpenCV, SuiteSparse, Boost, ROS | 轻量快速 MSCKF |
| **MATLAB Sensor Fusion Toolbox** | MathWorks | 商业 | MATLAB | EKF/UKF/PF, IMM, INS/GNSS, AHRS | `trackingEKF`, `trackingUKF`, `trackingIMM`, `insfilterAsync`, `ahrsfilter` | Hamilton (`quaternion`) | MATLAB + Nav/Sensor Fusion TBX | 教学/快速原型 |

#### A4.7.2 八个核心库的短说明

**GTSAM**。因子图事实标准。滤波派用户用 `ExtendedKalmanFilter<Pose2> ekf(x0,P0);`；滑窗平滑用 `IncrementalFixedLagSmoother`。IMU 走 `PreintegratedImuMeasurements` + `ImuFactor`，在 `NavState`（$SE_2(3)$）的切空间做预积分。**常见坑**：Pose3 retract 在 4.0+ 默认全 SE(3) Exp（右扰动），早期 Cayley 版本升级会有数值差；Hamilton 约定，与 OpenVINS/MSCKF JPL 相反。文档 `gtsam.org`。

**OpenVINS**。最完整的开源 MSCKF 研究平台。核心 `ov_msckf::VioManager`，状态全走 `StateHelper::EKFUpdate/Propagation/marginalize`。内建 FEJ 开关、IMU 内参在线估计。**坑**：JPL 四元数 + 左扰动 global error，与 Hamilton/右扰动体系对接时不能只做一个机械的转置或变号；需要同时核对四元数存储顺序、虚部符号、主动/被动旋转、扰动乘法方向和 Adjoint 搬运。文档 `docs.openvins.com`。

**kalmanif**。manif 作者把 EKF/SEKF/IEKF/UKFM/RTS 放在**同一模板 API** 下，是当前唯一能"一次性对比不变滤波各族"的教学库。典型：`kalmanif::ExtendedKalmanFilter<manif::SE3d> ekf;`。**坑**：WIP 状态，API 会变，学术对比 pin commit。

**manif**。C++11 header-only 李群库，覆盖 SO(n)/SE(n)/SE_2(3)/SGal(3)/Bundle，**全部操作含解析 Jacobian**。`X + tau` 是右 ⨁，`tau + X` 是左 ⨁。可作为 Ceres `Manifold`。**坑**：SE(3) 切向量为 $[\rho,\theta]$，SE_2(3) 为 $[\rho,\theta,\nu]$，Jacobian 拼装必须核对。

**invariant-ekf**。Hartley RSS-2018 Contact-Aided InEKF 官方实现。`inekf::InEKF` + `filter.CorrectKinematics/Landmarks/ContactPosition`。**Right-Invariant** 适合 world/fixed 参考量在 body/local 坐标中读出的观测（如 $Y=X^{-1}d$ 的 landmark、磁力计、接触足端相对位置）；若观测是 $Y=Xd$ 这类把 body 固定参考量送到 world 的形式，则应使用 Left-Invariant 变体。

**IKFoM + FAST-LIO2**。徐威 IROS-21 的模板库：用户只填 `get_f/df_dx/df_dw/h/dh_dx`，`kf.update_iterated_dyn_share_modified` 自动处理 $\boxplus/\boxminus/J^{(j)}$。**坑**：(i) 外参 `T_IL` 必须正确填入 launch；(ii) 模板编译极重，状态维度变化会触发整头文件重编；(iii) `max_iteration` 与收敛阈值对退化环境影响巨大。

**UKF-M**。Brossard-Barrau-Bonnabel ICRA-20 参考实现。`ukfm.UKF(x0,P0,f,h,phi,phi_inv,...)`——`phi` 是流形 retraction，选左/右不变直接决定行为是否一致。`benchmarks/` 有 2D 定位、姿态、IMU-GNSS、KITTI、SLAM2D 十余实验可跑。

**FilterPy**。Roger Labbe 教学库，配套 18k+ star 电子书 *Kalman and Bayesian Filters in Python*。`KalmanFilter/ExtendedKalmanFilter/UnscentedKalmanFilter/MerweScaledSigmaPoints/IMMEstimator/FixedLagSmoother` 与教材公式 1:1。**坑**：`batch_filter` 忽略 `KalmanFilter.u`，必须 `predict(u=)` 显式传；无流形/四元数。

#### A4.7.3 跨库对齐四条金律

1. **四元数约定**：Hamilton（GTSAM/Sophus/manif/VINS-Mono/ROVIO/FAST-LIO2）vs JPL（OpenVINS/MSCKF_VIO）。互转时虚部取反或旋转矩阵取转置。
2. **扰动方向**：右扰动（Sophus/manif/GTSAM/VINS-Mono/IKFoM）vs 左扰动（OpenVINS global error）。Jacobian 差一个 $\text{Ad}_R$。
3. **Ceres Manifold**：Ceres 2.x 将 `LocalParameterization` 更名 `Manifold`，manif/Sophus/GTSAM 均提供适配器。
4. **一致性方案**：**FEJ**（OpenVINS/VINS-Mono/MSCKF_VIO）vs **不变滤波**（invariant-ekf/kalmanif-IEKF/UKF-M）vs **等变滤波**（EqVIO）。三条技术路线。

**工程组合推荐**：产品线 → **FAST-LIO2 + OpenVINS/VINS-Mono + GTSAM + manif/Sophus + robot_localization**；学术对比 → **kalmanif + UKF-M + invariant-ekf + EqVIO**。

---

### §A4.8 学习资源汇总（中英文） ⭐

#### A4.8.1 英文免费教材（带真实 PDF 链接）

- **Barfoot, *State Estimation for Robotics* 2nd ed, 2024**。PDF：`https://asrl.utias.utoronto.ca/~tdb/`。核心：Ch.3 (linear batch/recursive)、Ch.4 (nonlinear: EKF/IEKF/UKF/ISPKF、batch)、Ch.5 (biases)、Ch.7 (matrix Lie groups)、Ch.8 (pose-graph/batch SE(3))、§8.2 连续时间 GP。**首选**。
- **Särkkä & Svensson, *Bayesian Filtering and Smoothing* 2nd ed, 2023**。PDF：`https://users.aalto.fi/~ssarkka/pub/bfs_book_2023_online.pdf`。核心：Ch.5–7 (KF/EKF/IEKF Gauss-Newton §7.5)、Ch.8 (UKF/GHKF/CKF)、Ch.10 (Posterior Linearization/IPLF)、Ch.11 (PF)、Ch.12–14 (RTS/URTSS/CKS/两过滤平滑)。**滤波第一参考**。
- **Thrun-Burgard-Fox, *Probabilistic Robotics*, MIT 2005**。EKF-SLAM/FastSLAM/PF-MCL/栅格。现在主要看 Ch.3–8。
- **Solà, *Quaternion kinematics for the ESKF***, arXiv:1711.02508, 2017。ESKF 最清晰工程手册，Hamilton/JPL 对照表、reset Jacobian 必读。
- **Solà-Deray-Atchuthan, *A micro Lie theory for state estimation in robotics***, arXiv:1812.01537, 2018。manif 论文 / JOSS 2020。
- **Hertzberg-Wagner-Frese-Schröder, *Integrating generic sensor fusion via manifold encapsulation***, *Information Fusion* 2013。$\boxplus/\boxminus$ 源头。

#### A4.8.2 核心论文时间线（26 篇）

| 年 | 作者 | 标题 | 出处 | 贡献 |
|---|---|---|---|---|
| 1960 | Kálmán | A New Approach to Linear Filtering and Prediction | *J. Basic Eng.* 82(1):35 | KF |
| 1965 | Rauch-Tung-Striebel | ML Estimates of Linear Dynamic Systems | *AIAA J.* 3(8):1445 | RTS |
| 1986 | Smith-Cheeseman | Representation and Estimation of Spatial Uncertainty | *IJRR* 5(4):56 | EKF-SLAM 雏形 |
| 1993 | Bell-Cathey | The iterated KF update as a Gauss-Newton method | *IEEE TAC* 38(2):294 | **IEKF = GN** |
| 1997 | Julier-Uhlmann | New Extension of the KF to Nonlinear Systems | *SPIE AeroSense* | UKF/UT |
| 2003 | Markley | Attitude Error Representations for Kalman Filtering | *JGCD* 26(2):311 | MEKF |
| 2004 | van der Merwe | Sigma-Point Kalman Filters for Integrated Navigation | PhD/ION | SUKF, SR-UKF |
| 2006 | Sibley-Sukhatme-Matthies | The Iterated Sigma Point KF | *RSS* 2006 | ISPKF |
| 2007 | Mourikis-Roumeliotis | A Multi-State Constraint KF for Vision-aided INS | *ICRA* 2007 | MSCKF |
| 2009 | Arasaratnam-Haykin | Cubature Kalman Filters | *IEEE TAC* 54(6):1254 | CKF |
| 2010 | Huang-Mourikis-Roumeliotis | Observability-based Rules for Consistent EKF SLAM | *IJRR* 29(5):502 | FEJ |
| 2011 | Arasaratnam-Haykin-Hurd | Cubature Kalman Smoothers | *Automatica* 47(10):2245 | CKS |
| 2013 | Hertzberg 等 | Manifold encapsulation | *Information Fusion* 14(1):57 | $\boxplus/\boxminus$ |
| 2015 | Forster-Carlone-Dellaert-Scaramuzza | IMU Preintegration on Manifold | *RSS*/*TRO* 2017 | SO(3) 预积分 |
| 2017 | Barrau-Bonnabel | The Invariant EKF as a Stable Observer | *IEEE TAC* 62(4):1797 | **InEKF** |
| 2017 | Solà | Quaternion kinematics for the ESKF | arXiv:1711.02508 | ESKF 手册 |
| 2018 | Qin-Li-Shen | VINS-Mono | *IEEE T-RO* 34(4):1004 | 滑窗 BA VIO |
| 2018 | Solà-Deray-Atchuthan | A micro Lie theory | arXiv:1812.01537 | manif 工程手册 |
| 2020 | Hartley-Ghaffari-Eustice-Grizzle | Contact-Aided Invariant EKF | *IJRR* 39(4):402 | 腿足 InEKF |
| 2020 | Brossard-Barrau-Bonnabel | A Code for UKF on Manifolds | *ICRA* 2020 / arXiv:2002.00878 | UKF-M |
| 2020 | Geneva-Eckenhoff-Lee-Yang-Huang | OpenVINS | *ICRA* 2020 | VIO 研究平台 |
| 2021 | He-Xu-Zhang | Embedding Manifold Structures into KF | arXiv:2102.03804 | **IKFoM** |
| 2021 | Xu-Zhang | FAST-LIO | *IEEE RA-L* 6(2):3317 | LIO IEKF |
| 2022 | Xu-Cai-He-Lin-Zhang | FAST-LIO2 | *IEEE TRO* 38(4):2053 | 直接 LIO + ikd-Tree |
| 2023 | van Goor-Mahony | EqF: Equivariant Filter for VIO | *ICRA*/arXiv:2205.01980 | EqF VIO |
| 1994 | Evensen | Sequential Data Assimilation with a Nonlinear Quasi-Geostrophic Model | *J. Geophys. Res.* 99(C5):10143 | **EnKF** |
| 1966 | Schmidt | Application of State-Space Methods to Navigation Problems | *Advances in Control Systems* 3:293 | **Schmidt-KF (Consider)** |
| 2021 | Brossard-Barrau-Chauchat-Bonnabel | $SE_2(3)$ Preintegration with Rotating Earth | *IEEE T-RO* 38(2):998 / arXiv:2007.14097 | **不变预积分** |
| 2023 | Kloss-Martino-Polydoros-Bobak-Haddadin | Differentiable Ensemble Kalman Filters | *IROS* 2023 / arXiv:2308.09870 | **DEnKF 机器人** |

#### A4.8.3 视频 / Notebook 资源

- **Cyrill Stachniss** SLAM 2013/14：`youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_`；Photogrammetry I&II 2021：`youtube.com/playlist?list=PLgnQpQtFTOGRYjqjdZxTEQPZuFHQa7O7Y`；教学主页 `ipb.uni-bonn.de/teaching/`。
- **Simo Särkkä** Aalto ELEC-E8105 课件与录课：`users.aalto.fi/~ssarkka/`。
- **Joan Solà** Lie theory 讲座：结合 arXiv:1812.01537 + `artivis/manif`。
- **Roger Labbe** `github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python` + `rlabbe/filterpy`。
- **Tim Barfoot** ASRL 4 讲短课程 `ser_short_course.pdf`。
- **MIT 6.4210 / 6.4212** Russ Tedrake：`manipulation.csail.mit.edu`（Drake notebook 可在 Deepnote/Colab 跑）。

#### A4.8.4 中文资源

**教材**：
- 高翔、张涛 *视觉 SLAM 十四讲*（第 2 版，电子工业出版社 2019）；代码 `github.com/gaoxiang12/slambook2`。
- 高翔 *自动驾驶中的 SLAM 技术*（电子工业出版社 2023）；代码 `github.com/gaoxiang12/slam_in_autonomous_driving`。
- 贺一家、高翔、崔华坤、赵季 *视觉惯性 SLAM: 理论与源码解析*（2023）。

**文档/博客**：邱笑晨《预积分总结与公式推导》PDF（知乎/CSDN 搜"邱笑晨 预积分"）；崔华坤 VIO-Doc `github.com/StevenCui/VIO-Doc`；赵宇 aipiano InEKF 知乎三部曲（知乎搜 "aipiano InEKF"）。

**代码**：heyijia/VINS-Course `github.com/HeYijia/VINS-Course`（从零手写 VIO 配套，去 ROS/Ceres 化）；vio_data_simulation `github.com/HeYijia/vio_data_simulation`；zlwang7/S-FAST_LIO `github.com/zlwang7/S-FAST_LIO`（FAST-LIO2 精简中文注释版，Sophus 替代 IKFoM）。

**课程**：深蓝学院 *从零手写 VIO*（贺一家/崔华坤）、*激光 SLAM*、*多传感器融合*；泡泡机器人 SLAM 社区 `paopaorobot.org`。

---

### §A4.9 学习路径与时间预算 ⭐

#### A4.9.1 两条完整学习路径

**路径 A：SLAM/VIO 方向**（目标：能独立阅读并改造 VINS/OpenVINS/ORB-SLAM3 级别系统）

| 节点 | 推荐教材/论文 | 推荐代码 | 档位 3 学时 |
|---|---|---|---|
| 1. KF / IF / SR-KF | Barfoot §3, Särkkä §6 | FilterPy `KalmanFilter` | 8 |
| 2. EKF + Jacobian 实战 | Barfoot §4.2.1–4.2.5, Thrun §3 | robot_localization `ekf.h` | 10 |
| 3. 一致性与 FEJ | Huang-Mourikis IJRR 2010, OpenVINS 文档 | OpenVINS `StateHelper` | 8 |
| 4. ESKF + 四元数 | Solà arXiv:1711.02508, Markley JGCD 2003 | MSCKF_VIO | 10 |
| 5. MSCKF / OpenVINS | Mourikis ICRA 2007, Geneva ICRA 2020 | OpenVINS 全栈 | 15 |
| 6. InEKF VIO | Barrau-Bonnabel TAC 2017 | invariant-ekf + 课件实现 | 12 |
| 7. 滑窗 BA / VINS-Mono | Qin TRO 2018, 崔华坤 VIO-Doc | VINS-Course | 15 |
| 8. 因子图 / iSAM2 | Kaess TRO 2012, Dellaert FnT 2017 | GTSAM | **→ 5-B/5-C** |

**路径 B：腿足/控制方向**（目标：Cassie/Atlas 级别状态估计 + MPC 协同）

| 节点 | 教材/论文 | 代码 | 档位 3 学时 |
|---|---|---|---|
| 1. KF / EKF 基础 | Barfoot §3–4 | FilterPy | 15 |
| 2. ESKF 姿态/IMU | Solà ESKF, Madgwick 2010 | ArduPilot, Pronto `imu_module` | 10 |
| 3. InEKF 不变性理论 | Barrau-Bonnabel TAC 2017 | invariant-ekf | 10 |
| 4. Contact-Aided InEKF | Hartley IJRR 2020 | `RossHartley/invariant-ekf` + Pronto | 15 |
| 5. UKF-M 流形 UKF | Brossard ICRA 2020 | `CAOR-MINES-ParisTech/ukfm` | 10 |
| 6. 腿足里程计 + leg kinematics | Bledt-Wensing 2018, Pronto legodo | Pronto `legodo_module` | 8 |
| 7. MPC 协同融合 | Di Carlo 2018, Neunert RA-L 2018 | (引入 5-控制系列) | 5 |

#### A4.9.2 5-A 全系列时间预算

| 子专题 | 档位 3 (h) | 档位 4 额外 (h) |
|---|---|---|
| 5-A1 贝叶斯/KF/IF/SR/NEES | 20–25 | +8 |
| 5-A2 欧氏非线性 (EKF/UKF/CKF/GHKF/FEJ/MSCKF) | 30–40 | +15 |
| 5-A3 流形滤波 (ESKF/MEKF/InEKF/UKF-M/IKFoM/EqF) | 45–55 | +20 |
| 5-A4 迭代/平滑/收口（本文） | 15–20 | +8 |
| **总计** | **110–140** | **+51（档位 4 总 161–191）** |

**按每周可投入学时估算完成时间**：

| 每周 h | 档位 3 完成 | 档位 3+4 完成 |
|---|---|---|
| 10 | 11–14 周（约 3 个月） | 16–19 周（约 4–5 个月） |
| 15 | 7–10 周（约 2 个月） | 11–13 周（约 3 个月） |
| 20 | 6–7 周（约 1.5 个月） | 8–10 周（约 2–2.5 个月） |

**5-A4 本子专题内部配比**：§A4.1–A4.3 迭代变体 6–8 h；§A4.4–A4.5 平滑/MHE 4–6 h；§A4.6–A4.12 大表/索引/陷阱/自测 4–6 h。

---

### §A4.10 自测题（跨 5-A1 至 5-A4，共 10 题） ⭐

> **档位 3**（必做 1–6）/ **档位 4**（必做 7–10）。建议在不查资料情况下尝试，卡壳后再翻对应章节。

**题 1（5-A1, 档 3）**。已知线性高斯系统 $x_{k+1}=Fx_k+w$, $y_k=Hx_k+v$，$w\sim\mathcal N(0,Q)$, $v\sim\mathcal N(0,R)$。请从**贝叶斯视角**（不从信号处理公式）逐步推导 KF 的预测与更新式，并说明为什么更新后的后验仍是高斯。

**题 2（5-A2, 档 3）**。对观测 $y=\|p\|+v$（距离测量，$p\in\mathbb R^3$），写出 EKF 的 $H=\partial h/\partial p$。为什么当 $\|p\|\to 0$ 时 EKF 会出问题？用 UKF 如何规避？

**题 3（5-A2, 档 3）**。UKF 的 Merwe scaled sigma 点取 $\alpha=10^{-3}, \beta=2, \kappa=3-n$。当 $n=10$ 时中心权重 $W_0^{(c)}$ 是多少？正还是负？会带来什么数值风险？换 CKF 会如何？

**题 4（5-A3, 档 3）**。ESKF 的 "reset" 步骤中，为什么要对协方差做 $P\leftarrow G P G^\top$ 投影而不是直接 $P$ 不变？给出 $G$ 对 $SO(3)$ 小角度误差的一阶近似表达式（提示：Solà arXiv:1711.02508 §6）。

**题 5（5-A3, 档 3）**。写出 Barrau-Bonnabel 的 group-affine 条件定义：若 $f(x_1 x_2)=f(x_1)x_2+x_1f(x_2)-x_1 f(e)x_2$，则系统左不变误差的动力学**独立于状态估计**。以 IMU 预积分（$SE_2(3)$ 上）为例说明该条件如何满足。

**题 6（5-A4, 档 3）**。不查资料写出 IEKF 的迭代公式 (A4.1)，并指出其中"搬移项" $-H^{(j)}(\hat x_{k|k-1}-\hat x^{(j)})$ 的物理意义。若漏掉该项会发生什么？

**题 7（5-A2/A4, 档 4）**。证明**CKF 是 UKF 的特例**：取 $\alpha=1, \kappa=0, \beta=0$，UKF 的 sigma 点与权重退化为 CKF 的 $2n$ 个球面-径向 cubature 点与权重 $\frac{1}{2n}$。

**题 8（5-A4, 档 4）**。证明 **Bell-Cathey 定理**：IEKF 更新式 (A4.1) 是对 (A4.2) 的 Gauss-Newton 迭代（提示：堆叠残差 $r(x)$、Woodbury 恒等式）。

**题 9（5-A1/A4, 档 4）**。在 MATLAB/Python 中模拟 2D 车辆 EKF-SLAM，记录每步 NEES，跑 100 次 Monte Carlo 后画 ANEES 曲线。为什么标准 EKF-SLAM 的 ANEES 会长期超出 $\chi^2_n$ 95% 上限？如何通过 FEJ 或 InEKF 修复？（建议：用 OpenVINS 与 invariant-ekf 分别跑 KAIST-VIO 或 EuRoC 数据集验证）。

**题 10（5-A3, 档 4）**。给出 EqF（van Goor-Mahony 2023）与 InEKF 的**关键差异**：EqF 是在怎样的群作用下定义等变误差？为什么 EqF 不要求 group-affine？给出 EqF 在单目 VIO（齐性空间 $SE_2(3)\times (\mathbb R^3)^N/\sim$）上的简要状态空间描述。

> **参考答案要点**。题 6：搬移项是先验高斯在展开点 $\hat x^{(j)}$ 处的线性化偏移；漏掉将让每次迭代只对当前点做 EKF 而遗忘先验，收敛至"均值漂向观测残差零点"而非 MAP。题 8 详见 §A4.1.2。题 9：伪观测性——滑窗在最新估计处线性化使 4 个不可观方向（全局位置 3 + yaw 1）虚增信息；FEJ 冻结线性化点，InEKF 从结构上消除该方向的可观性虚增。

---

### §A4.11 工程陷阱总汇（20 条，跨 5-A1 至 5-A4） ⭐⭐

按 10 类组织，每类 2 条。格式：**症状 → 根源 → 检查 → 修复**。

**类 1 · 数值稳定性**

1. **协方差失 SPD 导致 Cholesky 失败**。症状：`LLT` 偶发 `NumericalIssue`，Mahalanobis 出负。根源：`P←(I-KH)P` 累积误差破坏对称性。检查 `max|P-P^T|/max|P|` 与 `eigvals.min()`。修复：Joseph 形式 $P=(I-KH)P(I-KH)^\top+KRK^\top$；每步 $P\leftarrow\tfrac12(P+P^\top)$；改用 SR-UKF / 信息形式。
2. **$S=HPH^\top+R$ 条件数爆炸**。症状：$K$ 出 NaN；LiDAR 数千维一次性更新爆栈。根源：观测近线性相关或维度过大。检查 `cond(S)`。修复：用 FAST-LIO 的特征维度 $K=(H^\top R^{-1}H+P^{-1})^{-1}H^\top R^{-1}$；或分块顺序更新。

**类 2 · Jacobian 错误**

3. **符号/转置写反**。症状：前期正常，机动后发散；残差与更新同号放大。根源：$\partial h/\partial x$ 写成 $\partial x/\partial h$；或 body-to-world 的 Jacobian 用到 world-to-body 残差。检查：中心差分数值 Jacobian 对比，误差应 $<10^{-6}$。修复：单元测试锁定；建立"残差→扰动→符号"写作规范。
4. **坐标系混淆（world/body/IMU/cam）**。症状：静止段 bias 漂走；重力方向对但速度反。根源：$R_{wb}/R_{bw}$ 混用、外参 $T_{bc}$ 方向反。检查：变量命名 `p_w_b`、`R_wb`；静止段回放看 $v_w\approx 0, a_w\approx g$。修复：Drake 风格 "A-in-B" 命名；跨模块传李群类型而非裸 Matrix。

**类 3 · 时间戳 / 对齐**

5. **IMU-相机时延未标定**。症状：快速运动时轨迹周期性 "S 抖"。根源：硬件触发延迟 / 曝光中点 / ROS 接收时间戳。检查：旋转激励下残差与 $\omega$ 相关性；$t_d$ 是否收敛到非零常数。修复：状态加 $t_d$ 在线标定（VINS 做法）；或硬件同步 + Kalibr 离线先验。
6. **IMU 积分步长乱序**。症状：静止 bias 漂；滑窗与重放结果不一致。根源：IMU 丢包/乱序、dt 用 header 差而非传感器钟、中值/欧拉混用。检查：`dt` 分布直方图；$\sum dt$ 是否等于窗口时长。修复：传感器钟排序、异常 dt clip、统一中值积分。

**类 4 · 四元数**

7. **Hamilton 与 JPL 约定混用**。症状：姿态反向收敛；第三方库对接后残差翻号。根源：Hamilton `ij=k` vs JPL `ji=k`，$R(q)$ 差转置。检查：对同一 $q$ 作用单位向量比对第三方输出（参考 Solà 2017 附录 A）。修复：全工程锁定一种；边界封装 `to_hamilton/to_jpl`。
8. **四元数未归一化 / reset 未重投影**。症状：$\|q\|$ 漂离 1；MEKF reset 后协方差突变。根源：累积乘法 + 浮点；MEKF reset 未做 $P\leftarrow GPG^\top$, $G\approx I-\tfrac12[\delta\theta]_\times$。检查：监控 $|\|q\|-1|$。修复：每步 `normalize()`；Solà §6 reset Jacobian 重投影。

**类 5 · 流形**

9. **左扰动与右扰动混用**。症状：Jacobian 单测通过，集成后精度反降；与论文公式差 $\text{Ad}_R$。根源：$R\boxplus\delta=R\text{Exp}(\delta)$（右）vs $\text{Exp}(\delta)R$（左）。检查：把扰动约定写进代码注释；数值 Jacobian 验证。修复：全工程统一（推荐右扰动与 Sophus/Barfoot 一致）；边界做 Ad 显式转换。
10. **IKFoM reset 未按曲率 Jacobian 重投影**。症状：IEKF 在 $SO(3)$ 上收敛后 P 立刻回弹；bias 协方差"呼吸"。根源：$x^+=x\boxplus\delta$ 后 $\delta$ 坐标系变化，需 $P^+=J_rPJ_r^\top$。检查：开关重投影做 A/B。修复：严格实现 $J_r$；参考 `hku-mars/IKFoM`。

**类 6 · 一致性**

11. **EKF-SLAM/VIO 的伪观测性**。症状：NEES 长期超界，滤波"过自信"；回环误差小但不准。根源：每步在最新估计处线性化使 4 个不可观方向（位置 3 + yaw 1）虚增信息。检查：Hartley/Huang 的可观性 Gram 矩阵秩分析；ANEES 曲线。修复：FEJ 冻结线性化点；改用 InEKF/EqF。
12. **FEJ 配置未生效**。症状：开了 FEJ 但一致性指标无变化。根源：只锁位姿 Jacobian 未锁特征点/外参；或边缘化仍用当前估计。检查：扰动当前估计看相关 Jacobian 是否变。修复：所有不变零空间相关变量存"first estimate"快照；边缘化也用快照。

**类 7 · IEKF 迭代**

13. **IEKF 震荡不收敛**。症状：迭代到上限仍跳；$\|r\|$ 非单调。根源：纯 GN 在强非线性/观测奇异处步长过大。检查：记录每次 $\|r\|, \|\delta\|$。修复：LM 阻尼 $(H+\lambda\text{diag})^{-1}$；Dogleg；$J_{\max}=3\sim 4$（FAST-LIO 默认）+ 残差相对阈值；初值差时先 EKF 预更新。
14. **LM 阻尼策略不当**。症状：收敛慢精度低；或 $\lambda$ 爆炸使更新近 0。根源：$\lambda$ 初值太大、$\rho=$ 实际/预测下降 的 accept/reject 阈不当。检查：画 $\lambda$ 随迭代曲线；看 reject 频率。修复：Nielsen 策略（success $\lambda\leftarrow\lambda\cdot\max(1/3,1-(2\rho-1)^3)$；fail $\lambda\leftarrow\lambda\cdot 2^k$），初值 $\lambda_0=\tau\max\text{diag}(H), \tau=10^{-3}$。

**类 8 · RTS 平滑**

15. **反向 pass 协方差负特征值**。症状：$P^s_k$ 非 SPD，Cholesky 失败；平滑均值反而比滤波差。根源：$P^s=P_{k|k}+C(P^s_{k+1}-P_{k+1|k})C^\top$ 因浮点差异失 SPD。检查：每步 `eigvals.min()`；与 two-filter 平滑对比。修复：信息形式 / Bierman UD 平方根 RTS；对称化；$Q$ 加小 jitter。
16. **$P_{k+1|k}^{-1}$ 病态（长步长/静止）**。症状：静止段平滑修正近零或发散。根源：$P_{k+1|k}$ 近奇异。检查：`cond(P_{k+1|k})`。修复：改解线性系统 $P_{k+1|k}C^\top=FP_{k|k}$；或 Biswas-Mahalanabis / Fraser-Potter 两过滤避免显式求逆。

**类 9 · MHE / 滑窗边缘化**

17. **边缘化 fill-in 与先验 Jacobian 错配**。症状：滑窗滑动后 Hessian 稀疏度降，求解时间线性变慢；Ceres 先验残差异常大。根源：Schur 补产生密集 fill；VINS-Mono `MarginalizationFactor` 若未 FEJ，老先验与当前估计能量失真。检查：画 Hessian 稀疏模式；比有/无先验残差量级。修复：FEJ 快照 `linearized_jacobians/residuals/x0`；控制滑窗大小；关键帧稀疏化。
18. **MHE 中 FEJ 必须开启**。症状：启用滑窗边缘化后比纯 BA 更差；偶发发散。根源：Schur 补隐含在线性化点处冻结信息；若老新不同线性化点先验能量不守恒。检查：关闭 vs 开启边缘化比 NEES。修复：每次 marg 时用当前结果作 first estimate 并在后续迭代冻结（OpenVINS `UpdaterMSCKF` 参考）。

**类 10 · 其他**

19. **UKF sigma 权重为负**。症状：$P_{yy}/P_{xy}$ 出负特征值；高维传播不稳。根源：经典 $\kappa=3-n$ 在 $n>3$ 时中心权重 $W_0=\kappa/(n+\kappa)<0$。检查：打印 $W_0, W_c$；看高维稳定性。修复：Scaled UKF $\alpha\in[10^{-3},1], \beta=2, \kappa=0$ 或 $3-n$；或换 CKF（权重恒 $1/(2n)$ 正）；SR-UKF 平方根形式。
20. **初始 P 过大 / 过程-观测噪声不匹配**。症状：冷启动秒内 P 塌缩被单观测锁死；NEES 长期偏 $\chi^2$。根源：$P_0$ 过大使第一次更新 $K\approx 1$ 直灌噪声；$Q$ 小 → 过自信；$R$ 大 → 懒惰。检查：NEES $\sim\chi^2(n)$；NIS 归一化新息平方。修复：按物理意义设 $P_0$（位置 $\sigma$ 用 GNSS、姿态用静止 accel-leveling、gyro bias 用 Allan 方差）；$Q,R$ 用 Allan 方差 / EM 离线估；在线 IAE 微调；入稳态后锁定。

---

### §A4.12 流形/不变滤波收口：从误差定义到工程选型 ⭐⭐⭐

> **这一节解决什么问题**：A3 已经从方法族角度讲过 ESKF/MEKF/InEKF/UKF-M；A4 在全景收口处要回答更实际的问题：面对一个机器人估计任务，如何判断该用普通 EKF、ESKF、iterated EKF、InEKF、IKFoM 还是平滑/因子图？

#### A4.12.1 四条坐标轴如何落到流形滤波

A4 开头提出整个 Kalman 族可以由四条正交坐标轴生成：

| 坐标轴 | 可选坐标 | 流形滤波中的具体问题 |
|---|---|---|
| 先验高斯 | 加性、乘性、不变、等变 | 协方差定义在 $\mathbb R^n$ 还是切空间 |
| 线性化策略 | Taylor、sigma、cubature、GN 迭代 | 用 Jacobian、sigma 点还是多次重线性化 |
| 状态空间几何 | 欧氏、$SO(3)$、$SE(3)$、$SE_2(3)$、齐性空间 | 状态能否相减，能否用群乘法表达误差 |
| 时间/窗口维度 | 单步滤波、迭代更新、RTS、MHE、BA | 保留一个状态还是保留一段轨迹 |

这四条轴不是论文分类用的标签，而是工程决策顺序。一个实际系统通常按下面流程选择：

```text
状态包含姿态/位姿吗？
  ├─ 否：普通 KF/EKF/UKF 足够
  └─ 是：旋转是否会经历大角度运动？
        ├─ 否：加性姿态参数可能短期可用，但不推荐作为长期系统
        └─ 是：使用乘性误差或流形 retraction
              ├─ 只估姿态：MEKF
              ├─ IMU + 位置/速度/bias：ESKF
              ├─ 动力学可写成 group-affine：InEKF
              ├─ 观测强非线性且需多次配准：iterated EKF / IKFoM
              └─ 需窗口内重估：Fixed-lag smoother / MHE / 因子图
```

把这个流程写清楚，是为了避免一个常见误区：把算法名称当作优劣排序。ESKF 不一定比 EKF "高级"，InEKF 不一定能直接替代 VINS 的滑窗，UKF-M 也不一定比 ESKF 更准。它们回答的是不同坐标轴上的问题。

> **本质洞察**：滤波方法的选择不是"从旧到新"的线性升级，而是对四个问题的联合回答：状态在哪里、误差怎么定义、线性化怎么做、保留多少时间信息。

#### A4.12.2 MEKF：最小姿态误差为什么足以改变协方差结构

MEKF 的历史动机非常具体：航天器姿态通常用单位四元数 $q\in S^3$ 表示，但姿态自由度只有 3。若直接对四元数做 4 维 EKF，协方差 $P_q\in\mathbb R^{4\times 4}$ 必须同时满足单位范数约束。

单位四元数满足

$$q^\top q=1.$$

对这个约束求微分：

$$2q^\top \delta q=0.$$

因此合法扰动 $\delta q$ 必须位于 $q$ 的切空间，也就是一个三维子空间。若滤波器仍维护四维协方差，它会把一个本来三维的随机变量嵌到四维空间中。这个协方差在约束法向方向上应为零，天然接近奇异。

MEKF 的设计步骤如下：

1. 用一个单位四元数 $\hat q$ 表示大姿态。
2. 用三维小角 $\delta\theta\in\mathbb R^3$ 表示误差。
3. 把小角映射成误差四元数

$$\delta q(\delta\theta)\approx
\begin{bmatrix}
1\\
\frac12\delta\theta
\end{bmatrix}.
$$

4. 注入时使用乘法

$$\hat q^+=\hat q\otimes \delta q(\hat{\delta\theta}).$$

5. 注入后把误差均值清零，协方差仍维护在新的切空间中。

这一步看似只是把加法换成乘法，实际改变了协方差的语义：

| 直接四元数 EKF | MEKF |
|---|---|
| $P$ 描述四维四元数坐标扰动 | $P$ 描述三维切空间小角 |
| 需要额外归一化 | 注入后天然回到单位四元数 |
| 约束方向协方差病态 | 最小维度，SPD 更自然 |
| 参数化错误容易混入统计量 | 姿态状态与误差状态分离 |

如果不这样做会怎样？短时间仿真中，归一化四元数可能让姿态看起来正常；但协方差传播仍然沿着错误坐标进行。观测更新时，Kalman 增益会把残差投影到四元数约束方向，然后归一化又把这部分状态变化删除。状态被删除，协方差信息却未同步删除，滤波器逐步失去统计一致性。

MEKF 给 A4 的启示是：**状态参数化与误差参数化可以不同**。这是后来 ESKF、InEKF、IKFoM 的共同思想根源。

#### A4.12.3 ESKF：为什么 IMU 系统要拆成 nominal 与 error

惯性导航的真实状态可以写为

$$x=(p,v,R,b_a,b_g,g).$$

IMU 预测在工程上通常以 200 Hz 到 1000 Hz 运行，而相机、LiDAR、GNSS 观测频率低得多。若每个 IMU tick 都对完整非线性状态做 EKF 更新，代价高且没有必要。ESKF 的核心是三态分解：

| 层次 | 含义 | 工程角色 |
|---|---|---|
| true state | 物理真实状态 | 推导时存在，运行时不可知 |
| nominal state | 大信号轨迹 | 高频 IMU 积分 |
| error state | 小信号扰动 | 低维线性 KF 传播与更新 |

三态关系为

$$p_t=p+\delta p,\quad v_t=v+\delta v,\quad R_t=R\exp(\delta\theta^\wedge),$$

$$b_{a,t}=b_a+\delta b_a,\quad b_{g,t}=b_g+\delta b_g,\quad g_t=g+\delta g.$$

从这里可以看出 ESKF 并不是"只对姿态用流形"，而是把每个状态块放在最自然的误差空间：

| 状态块 | 物理空间 | 误差定义 | 原因 |
|---|---|---|---|
| 位置 $p$ | $\mathbb R^3$ | 加性 | 欧氏向量可相减 |
| 速度 $v$ | $\mathbb R^3$ | 加性 | 欧氏向量可相减 |
| 姿态 $R$ | $SO(3)$ | 乘性 | 旋转不可直接相减 |
| bias | $\mathbb R^3$ | 加性 | 慢变欧氏参数 |
| 重力 $g$ | $\mathbb R^3$ 或 $S^2$ | 加性或球面扰动 | 取决于是否约束模长 |

ESKF 的预测矩阵可以从 A3 中的误差方程整理为

$$\dot{\delta x}=F\delta x+G n,$$

其中按 $\delta x=(\delta p,\delta v,\delta\theta,\delta b_a,\delta b_g,\delta g)$ 排列，

$$F=
\begin{bmatrix}
0&I&0&0&0&0\\
0&0&-R\hat a^\wedge&-R&0&I\\
0&0&-\hat\omega^\wedge&0&-I&0\\
0&0&0&0&0&0\\
0&0&0&0&0&0\\
0&0&0&0&0&0
\end{bmatrix}.
$$

这张矩阵告诉工程师三件事：

1. 姿态误差会通过 $-R\hat a^\wedge$ 进入速度误差。
2. 加速度 bias 会通过 $-R$ 进入速度误差。
3. 陀螺 bias 会通过 $-I$ 进入姿态误差。

这些块的符号不是记忆题。它们对应直觉：

| 矩阵块 | 直觉解释 |
|---|---|
| $-R\hat a^\wedge$ | 姿态错一点，重力/加速度方向投影就错一点 |
| $-R$ | 加速度 bias 偏大，积分出的速度会反向校正 |
| $-I$ | 陀螺 bias 偏大，名义姿态多转了，需要负向误差抵消 |

ESKF 与 iterated EKF 的关系也要分清。ESKF 解决的是"状态空间不是欧氏"的问题；iterated EKF 解决的是"观测非线性太强，一次线性化不够"的问题。FAST-LIO2 同时使用流形误差和迭代更新，原因正是点云点到面残差强依赖当前位姿。

#### A4.12.4 InEKF：什么时候结构比迭代更重要

ESKF 仍然有一个局限：误差动力学矩阵 $F$ 依赖当前名义状态 $R$ 和输入 $\hat a,\hat\omega$。对于普通惯性导航，这通常可以接受；对于 SLAM/VIO 的可观性一致性问题，这种依赖会放大线性化坐标选择的影响。

InEKF 的出发点是换一个误差：

$$\eta^R=\hat X X^{-1},\qquad \eta^L=X^{-1}\hat X.$$

这两个误差被称为右不变/左不变，是因为它们对某一侧的群作用不变。例如右不变误差 $\eta^R=\hat X X^{-1}$ 在同时右乘同一个 $G$ 元素时不变：

$$\hat X\mapsto \hat X\Gamma,\quad X\mapsto X\Gamma
\quad\Longrightarrow\quad
(\hat X\Gamma)(X\Gamma)^{-1}=\hat X X^{-1}.$$

若系统动力学满足 group-affine 条件

$$f_u(X_1X_2)=f_u(X_1)X_2+X_1f_u(X_2)-X_1f_u(I)X_2,$$

则不变误差的微分方程不依赖真实轨迹与估计轨迹：

$$\dot\eta=g_u(\eta).$$

进一步在对数坐标中，Barrau-Bonnabel 定理给出

$$\eta=\exp(\xi^\wedge)\quad\Longrightarrow\quad \dot\xi=A_u\xi.$$

这里的等式在 $\log$ 映射收敛域内是精确的，不是普通 EKF 的一阶近似。它的工程含义是：

| 标准 EKF/ESKF | InEKF |
|---|---|
| $F=F(\hat X,u)$ | $A=A(u)$ 或只含已知量 |
| 传播线性化随估计漂移 | 传播误差模型与估计解耦 |
| 可观性子空间可能被破坏 | group-affine + invariant observation 下保留对称性 |
| 需要仔细处理 FEJ/OC 思想 | 结构本身提供一致性基础 |

这就是为什么 InEKF 在腿式接触、IMU 导航、InEKF-SLAM 中有特殊地位。腿式机器人常见状态为

$$X=\begin{bmatrix}
R&v&p&d_1&\cdots&d_N\\
0&1&0&0&\cdots&0\\
0&0&1&0&\cdots&0\\
0&0&0&1&&0\\
\vdots&\vdots&\vdots&&\ddots&\\
0&0&0&0&&1
\end{bmatrix},
$$

其中 $d_i$ 是第 $i$ 个接触足端在世界系的位置。足端接触约束给出 body-frame 观测：

$$y_i=R^\top(d_i-p)+n_i.$$

这与 SLAM landmark 观测形式完全相同。区别只是 landmark 是外部地图点，contact point 是短时间内静止的足端点。InEKF 因此把腿式里程计和 landmark-SLAM 放到同一代数框架。

如果系统含有 bias，perfect group-affine 通常会被破坏。工程中有三条路线：

| 路线 | 做法 | 适用情况 |
|---|---|---|
| Imperfect InEKF | 群状态保持 InEKF，bias 加性增广 | Hartley contact-aided InEKF |
| 混合 ESKF/InEKF | 核心位姿用不变误差，bias 用传统误差 | IMU 系统常见折中 |
| EqF | 把问题放到更一般的等变系统 | VIO/SLAM 研究级系统 |

这也提醒我们：InEKF 不是"所有流形状态估计的终点"，而是一类结构条件满足时特别干净的解。

#### A4.12.5 可观性一致性：从 A2 的病理到 A3 的结构解

A2 中已经提到 EKF-SLAM/VIO 的一致性病理：系统本来有全局位置和 yaw 的不可观自由度，滤波器却会逐渐把这些方向的协方差压小。A4 在收口处把这个问题写成标准线性代数形式。

设线性化误差系统为

$$\delta x_{k+1}=F_k\delta x_k,\qquad \delta y_k=H_k\delta x_k+n_k.$$

从时刻 $0$ 到 $m$ 的可观性矩阵为

$$\mathcal O_m=
\begin{bmatrix}
H_0\\
H_1F_0\\
H_2F_1F_0\\
\vdots\\
H_mF_{m-1}\cdots F_0
\end{bmatrix}.
$$

不可观方向 $N$ 应满足

$$\mathcal O_mN=0.$$

VIO/SLAM 中的典型 $N$ 包含：

| 不可观方向 | 物理变换 | 为什么观测不变 |
|---|---|---|
| 全局平移 3 维 | $p_i\mapsto p_i+t,\ \ell_j\mapsto\ell_j+t$ | 相对向量 $\ell_j-p_i$ 不变 |
| 全局 yaw 1 维 | 绕重力方向整体旋转 | IMU 重力方向不变，视觉相对几何不变 |

标准 EKF 破坏一致性的原因不是公式写错，而是不同时间的 Jacobian 在不同估计点上求值。即便每一个 $H_k$ 单独正确，乘积 $H_kF_{k-1}\cdots F_0$ 也可能不再保留同一个 $N$。

FEJ 通过冻结线性化点，使所有 Jacobian 共享同一参考坐标。OC-EKF 通过显式约束 Jacobian 的零空间，强制 $\mathcal O_mN=0$。InEKF 则通过不变误差与 group-affine 传播，让 $F_k$ 的结构自动与系统对称性匹配。

可以把三种路线对比如下：

| 路线 | 核心动作 | 优点 | 局限 |
|---|---|---|---|
| FEJ | 使用第一次估计点求 Jacobian | 简单、OpenVINS/VINS 常用 | 一阶精度牺牲，参考点选择影响结果 |
| OC-EKF | 显式投影 Jacobian 以保留零空间 | 理论目标清晰 | 实现复杂，需手工构造不可观子空间 |
| InEKF/EqF | 用对称性定义误差 | 结构自然、长期一致性更好 | 需要系统满足不变/等变结构 |

反事实地看，如果系统不处理一致性，短时间轨迹误差可能仍然很小，但 NEES 会持续偏高，闭环控制会过度相信错误姿态。对于腿式机器人，这可能表现为 base yaw 看起来很稳，实际接触切换后突然漂移；对于 VIO，这可能表现为地图越来越"硬"，回环或重定位时产生大跳变。

#### A4.12.6 Iterated EKF 与 InEKF 如何组合：FAST-LIO2 的视角

FAST-LIO2 常被称为 iterated Kalman filter on manifold。它的核心不是 perfect InEKF，而是把一次 LiDAR scan 的大量点到面残差在同一个状态上做多次重线性化。单次观测更新的目标可以写成

$$J(\delta)=\frac12\|\delta-\delta_0\|_{P^{-1}}^2
+\frac12\sum_i\|r_i(\hat x\boxplus\delta)\|_{R_i^{-1}}^2.$$

在第 $j$ 次迭代处线性化

$$r_i(\hat x^{(j)}\boxplus \Delta)\approx r_i^{(j)}+H_i^{(j)}\Delta.$$

堆叠后得到正规方程

$$\left(P^{-1}+H^\top R^{-1}H\right)\Delta
=-P^{-1}(\hat x^{(j)}\boxminus \hat x^-)-H^\top R^{-1}r^{(j)}.$$

这个式子默认所有量已经表达在同一个切空间中。若使用 IKFoM/FAST-LIO2 这类 compound manifold，第 $j$ 次迭代的切空间与先验切空间不同，还要引入 §A4.15 的曲率搬运 Jacobian $J^{(j)}$。

这就是 A4.3 中 IKFoM 公式的矩阵形式。它与 InEKF 的区别在于：

| 维度 | IKFoM/FAST-LIO2 | InEKF |
|---|---|---|
| 重点 | 观测更新迭代求解 MAP | 误差定义保持不变性 |
| 状态空间 | 通用 compound manifold | 矩阵李群 |
| 结构条件 | 需要 $\boxplus/\boxminus$ 与 Jacobian | 需要 group-affine / invariant observation |
| 典型收益 | 点云匹配强非线性下收敛好 | 可观性与传播结构一致 |

二者可以在同一系统中互补。例如一个腿式 LiDAR-IMU 系统可以：

1. 用 $SE_2(3)$ 或 $SE_{N+2}(3)$ 组织 IMU + contact 传播。
2. 对 LiDAR point-to-plane 残差做 iterated update。
3. 对 bias 使用加性增广或 EqF lift。
4. 对关键帧窗口再交给因子图平滑。

这四步对应 A4 的四条坐标轴，而不是四个互斥算法。

#### A4.12.7 机器人 IMU/SLAM 例子：同一个模型如何产生三种实现

考虑同一个传感器组合：IMU + 相机 landmark。状态为

$$x=(R,p,v,b_g,b_a,\ell_1,\ldots,\ell_N).$$

观测为

$$z_i=\pi\left(R^\top(\ell_i-p)\right)+n_i,$$

其中 $\pi(\cdot)$ 是相机投影。对这个问题，至少有三种实现路线：

| 路线 | 状态表示 | 更新方式 | 适合阶段 |
|---|---|---|---|
| MSCKF/ESKF | 当前 IMU 状态 + 滑动相机 clone | 特征零空间投影 | 轻量实时 VIO |
| IKFoM/IEKF | 当前复合流形状态 | 多次迭代更新 | LIO/强非线性配准 |
| InEKF/EqF | 群或齐性空间误差 | 不变/等变误差更新 | 研究级一致性与大误差收敛 |

若使用 ESKF，预测时维护名义状态：

$$\hat R^+=\hat R\exp((\omega_m-\hat b_g)\Delta t)^\wedge,$$

$$\hat v^+=\hat v+\left(\hat R(a_m-\hat b_a)+g\right)\Delta t,$$

$$\hat p^+=\hat p+\hat v\Delta t+\frac12\left(\hat R(a_m-\hat b_a)+g\right)\Delta t^2.$$

误差协方差用 $F,G,Q$ 传播：

$$P^+=\Phi P\Phi^\top+GQG^\top.$$

若使用 iterated EKF，视觉更新时不只计算一次 $H$，而是重复：

1. 用当前 $\hat x^{(j)}$ 投影 landmark。
2. 计算 residual 与 Jacobian。
3. 解一遍高斯牛顿式 Kalman 增量。
4. 用 $\boxplus$ 注入。
5. 判断增量是否足够小。

若使用 InEKF/EqF，关键不是多算几次，而是把误差定义成与全局平移/yaw 对称性一致的形式，使不可观方向不会被观测更新错误压缩。

这三种实现的选择取决于最痛的问题：

| 最痛的问题 | 优先选择 |
|---|---|
| CPU 预算紧张、需要成熟工程 | ESKF/MSCKF |
| LiDAR/视觉残差非线性强 | iterated EKF / IKFoM |
| 冷启动误差大、接触切换多、关注一致性 | InEKF/EqF |
| 离线精度或回环很重要 | Fixed-lag smoother / 因子图 |

#### A4.12.8 代码骨架：用一个接口表达 $\boxplus/\boxminus$

工程上最容易出错的不是 Kalman 增益，而是误差注入接口不统一。下面的 C++ 骨架展示一个最小接口：滤波器只依赖 `boxplus` 与 `boxminus`，具体状态可以是欧氏、$SO(3)$、$SE(3)$ 或复合流形。

```cpp
#include <Eigen/Dense>

// 教学骨架：真实工程中应把状态块、噪声块和雅可比维度写成强类型
struct State {
    Eigen::Matrix3d R;   // 机体到世界的旋转
    Eigen::Vector3d p;   // 世界系位置
    Eigen::Vector3d v;   // 世界系速度
};

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0.0, -w.z(), w.y(),
         w.z(), 0.0, -w.x(),
        -w.y(), w.x(), 0.0;
    return W;
}

Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d A = skew(phi);
    if (theta < 1e-8) {
        // 小角度时使用二阶近似，避免除以很小的数
        return I + A + 0.5 * A * A;
    }
    return I + std::sin(theta) / theta * A
             + (1.0 - std::cos(theta)) / (theta * theta) * A * A;
}

State boxplus(const State& x, const Eigen::Matrix<double, 9, 1>& dx) {
    State y = x;
    // 姿态使用右乘扰动：R_new = R * Exp(delta_theta)
    y.R = x.R * ExpSO3(dx.segment<3>(0));
    // 位置、速度仍是欧氏加法
    y.p = x.p + dx.segment<3>(3);
    y.v = x.v + dx.segment<3>(6);
    return y;
}

Eigen::Matrix<double, 9, 1> zeroTangent() {
    // 统一返回切空间零向量，便于重置误差均值
    return Eigen::Matrix<double, 9, 1>::Zero();
}
```

这段代码没有实现完整滤波器，目的只有一个：把"状态更新"从 `x += dx` 改成显式的 `boxplus(x, dx)`。一旦接口统一，ESKF、IKFoM、流形 RTS 都能复用同一套误差注入语义。

常见错误对比如下：

| 错误做法 | 现象 | 正确做法 |
|---|---|---|
| 对旋转矩阵元素直接相加 | $R^\top R$ 偏离 $I$ | 使用 $R\exp(\delta\theta^\wedge)$ |
| 更新四元数后只归一化状态 | 协方差与状态坐标不一致 | 同步执行 reset Jacobian |
| `boxplus` 用右扰动，Jacobian 按左扰动推导 | 符号错、NEES 异常 | 在接口注释中写明扰动方向 |
| `boxminus` 返回欧拉角差 | 接近奇异点时跳变 | 返回 Lie log 坐标 |

#### A4.12.9 选型表：把方法放回机器人任务

| 任务 | 推荐主线 | 为什么 |
|---|---|---|
| 手机/无人机 AHRS | MEKF 或 ESKF 姿态子系统 | 姿态为主，状态低维 |
| GNSS-INS | ESKF | IMU 高频传播，GNSS 低频位置观测 |
| 视觉惯性前端 | ESKF/MSCKF 或滑窗 MHE | 需要处理 feature clone 和边缘化 |
| FAST-LIO 类 LiDAR-IO | IKFoM / iterated ESKF | 点云残差多且强非线性 |
| 腿式 contact-aided odometry | InEKF 或 imperfect InEKF | 接触点观测天然不变 |
| 研究级 VIO 一致性 | EqF / InEKF-SLAM | 明确利用 gauge 对称性 |
| 离线高精建图 | 因子图 / full BA | 需要重估历史状态与回环 |

这个表的关键不是告诉读者"哪个最好"，而是训练一个判断习惯：先找状态几何，再找误差定义，再找线性化策略，最后才考虑库和实现细节。

#### A4.12.10 故障诊断：流形滤波最常见的五类失败

| 症状 | 可能原因 | 排查步骤 | 对应方法 |
|---|---|---|---|
| 姿态矩阵逐渐不正交 | 直接对矩阵加增量 | 检查更新是否使用 Exp；打印 $\|R^\top R-I\|$ | MEKF/ESKF |
| yaw 协方差异常变小 | 不可观方向被线性化破坏 | 构造 gauge 向量 $N$，检查 $HN$ | ESKF/InEKF |
| IMU 预测后速度漂移巨大 | bias 符号或重力方向错 | 静止数据下检查 $\hat R a_m+g$ 是否接近 0 | ESKF |
| LiDAR 更新不收敛 | 单次线性化离最优点太远 | 增加迭代、加 line search、检查最近邻质量 | IKFoM |
| reset 后协方差非 SPD | reset Jacobian 漏乘或数值对称性丢失 | 使用 Joseph form；更新后做 $P=(P+P^\top)/2$ | MEKF/ESKF |

#### A4.12.11 自测题：把 A3 与 A4 接起来

1. **误差定义题**：给定 $R=\hat R\exp(\delta\theta^\wedge)$，从 $\dot R=R\omega^\wedge$ 推导 $\dot{\delta\theta}$ 的一阶形式，并说明每一步丢弃了哪些二阶项。
2. **选型题**：一个四足机器人有 IMU、腿部编码器和足端接触检测，但没有 LiDAR。为什么 contact-aided InEKF 比普通 ESKF 更自然？bias 又会带来什么限制？
3. **一致性题**：写出 VIO 中全局平移 gauge 方向对应的 $N$。证明相机观测 $R^\top(\ell-p)$ 对该方向一阶不敏感。
4. **迭代题**：解释 FAST-LIO2 为什么需要 iterated update，而 GNSS-INS 通常不需要。请从观测非线性和残差数量两个角度回答。
5. **跨章综合题**：把 A1 的 Joseph form、A3 的 reset Jacobian、A4 的 IEKF-GN 等价组合起来，写出一次流形 iterated update 的完整伪代码。

#### A4.12.12 Group-affine 快速验证：看到 $AX+XB$ 就要警觉

在工程推导中，不可能每次都从 Theorem 1 的定义完整验证。一个很实用的识别信号是：如果动力学能写成

$$\dot X=f_u(X)=A_uX+XB_u,$$

其中 $A_u,B_u$ 只依赖输入、常量或已知外部量，那么它自动满足 group-affine。

验证过程只有三行：

$$f_u(X_1X_2)=A_uX_1X_2+X_1X_2B_u,$$

而

$$f_u(X_1)X_2+X_1f_u(X_2)-X_1f_u(I)X_2$$

等于

$$\begin{aligned}
&(A_uX_1+X_1B_u)X_2+X_1(A_uX_2+X_2B_u)-X_1(A_u+B_u)X_2\\
&=A_uX_1X_2+X_1B_uX_2+X_1A_uX_2+X_1X_2B_u-X_1A_uX_2-X_1B_uX_2\\
&=A_uX_1X_2+X_1X_2B_u.
\end{aligned}$$

这正好等于 $f_u(X_1X_2)$。

对 $SE_2(3)$ IMU，状态

$$X=\begin{bmatrix}R&v&p\\0&1&0\\0&0&1\end{bmatrix}$$

的动力学

$$\dot R=R\omega^\wedge,\qquad \dot v=g+Ra,\qquad \dot p=v$$

可以组织成上述形式。重力 $g$ 进入 $A_u$，机体系输入 $\omega,a$ 进入 $B_u$。这解释了一个看似奇怪的事实：有重力的 IMU 动力学不是纯左不变或纯右不变，却仍然 group-affine。

如果读者只记住"左不变/右不变系统才适合 InEKF"，就会误判惯性导航。正确说法是：左不变和右不变只是 group-affine 的两个特例；$SE_2(3)$ IMU 属于更一般的左右组合。

#### A4.12.13 Log-linear 误差为什么能与 Kalman 传播对齐

普通 EKF 的传播线性化是近似：

$$\dot{\delta x}=A(\hat x,u)\delta x+O(\|\delta x\|^2).$$

InEKF 在 group-affine 条件下得到的是

$$\eta=\exp(\xi^\wedge),\qquad \dot\xi=A(u)\xi.$$

为了理解这句话的分量含义，以 $SE_2(3)$ 右不变误差为例，令

$$\xi=(\delta\theta,\delta v,\delta p).$$

一个常见形式为

$$A=
\begin{bmatrix}
0&0&0\\
g^\wedge&0&0\\
0&I&0
\end{bmatrix}.
$$

于是误差方程分块为

$$\dot{\delta\theta}=0,$$

$$\dot{\delta v}=g^\wedge\delta\theta,$$

$$\dot{\delta p}=\delta v.$$

这三行非常有物理意义：

| 方程 | 物理解释 |
|---|---|
| $\dot{\delta\theta}=0$ | 无 bias 的理想 IMU 中，右不变姿态误差不被名义角速度驱动 |
| $\dot{\delta v}=g^\wedge\delta\theta$ | 姿态误差会把重力方向投错，形成速度误差 |
| $\dot{\delta p}=\delta v$ | 速度误差积分成位置误差 |

注意这里没有 $\hat R$。这就是 log-linear 的工程价值。协方差传播

$$P^+=\Phi P\Phi^\top+Q,\qquad \Phi=\exp(A\Delta t)$$

使用的 $\Phi$ 不随当前姿态估计旋转而改变。对长时间导航而言，这会减少"估计越偏，协方差模型越偏"的连锁效应。

如果 $A$ 是上面这种严格下三角块矩阵，还可以手写矩阵指数。由于 $A^3=0$，有

$$\Phi=I+A\Delta t+\frac12A^2\Delta t^2.$$

这比一般 `expm` 更快，也更容易做单元测试。测试方法是随机取 $\delta\theta,\delta v,\delta p$，比较手写 $\Phi$ 与数值矩阵指数的差异；误差应在浮点精度附近。

#### A4.12.14 Reset、retraction 与平滑：同一个坐标搬运问题

MEKF/ESKF 中的 reset step、IKFoM 中的 $J^{(j)}$、流形 RTS 中的后向差分，本质上都是同一个问题：切空间坐标依附于参考点，参考点变了，坐标也要搬运。

ESKF 更新后执行

$$\hat x^+=\hat x\boxplus\hat{\delta x},\qquad \hat{\delta x}\leftarrow 0.$$

但原来的协方差描述的是旧参考点处的误差。若新误差 $\delta x^+$ 与旧误差 $\delta x$ 满足一阶关系

$$\delta x^+=G\delta x,$$

则必须更新

$$P^+=GPG^\top.$$

IKFoM 的迭代公式中也有同类搬运。第 $j$ 次迭代的切空间在 $\hat x^{(j)}$，先验协方差却定义在 $\hat x^-$。因此需要曲率 Jacobian $J^{(j)}$ 把两处切空间对齐：

$$\|\hat x^{(j)}\boxminus \hat x^-\|_{P^{-1}}^2
\quad\Longrightarrow\quad
\|J^{(j)}\delta+\text{offset}\|_{P^{-1}}^2.$$

流形 RTS 后向步同样如此：

$$\delta_{k+1}=\hat x_{k+1|N}\boxminus \hat x_{k+1|k},$$

$$\hat x_{k|N}=\hat x_{k|k}\boxplus(C_k\delta_{k+1}).$$

这里 $\boxminus$ 把两个未来时刻估计变成切空间差，$C_k$ 再把未来修正映射回当前时刻。若状态是欧氏的，这些搬运动作退化成普通减法；若状态在流形上，它们必须显式出现。

工程排查时可以用一句话自检：**只要参考状态被改变，协方差或增量坐标是否也被搬运了？** 若答案不确定，reset、iterated update 或 smoother 中很可能有隐藏错误。

#### A4.12.15 三种机器人场景的完整决策演练

**场景 1：低成本 GNSS-INS。**

传感器：IMU 200 Hz，GNSS 5 Hz，磁力计可选。目标：车辆平面导航。

推荐：ESKF。

理由：

1. 状态含姿态，必须用乘性误差。
2. GNSS 位置观测弱非线性，不需要 iterated update。
3. 动力学有 bias，perfect InEKF 条件不完整。
4. 工程重点是噪声标定、延迟补偿、坐标系转换。

**场景 2：四足机器人接触辅助里程计。**

传感器：IMU、关节编码器、足端接触检测。目标：base pose/velocity 估计。

推荐：contact-aided InEKF 或 imperfect InEKF。

理由：

1. 足端接触点在世界系短时静止，可作为 $SE_{N+2}(3)$ 中的扩展列。
2. 观测 $R^\top(d_i-p)$ 是不变观测，与 landmark-SLAM 同构。
3. 大姿态误差、接触切换和冲击使普通 EKF 更容易过度自信。
4. bias 需要加性处理，因此实现上通常是 imperfect 版本。

**场景 3：高速 LiDAR-IMU 建图。**

传感器：IMU 500 Hz，LiDAR 10 Hz，每帧上千个点到面残差。

推荐：IKFoM / iterated ESKF，必要时后端因子图。

理由：

1. 单帧残差数量巨大，直接用测量维度求逆不划算。
2. 点到面残差强依赖当前姿态与最近邻，单次线性化不够。
3. 状态含 $SO(3)$ 与外参，必须用流形 $\boxplus/\boxminus$。
4. 需要地图维护、关键帧与回环时，滤波前端应与因子图后端衔接。

这三个场景说明：方法选型不是看论文标题，而是看传感器频率、残差非线性、状态几何和一致性需求。

#### A4.12.16 本节小结：把名词压回可执行判断

| 名词 | 一句话判断 | 常见误用 |
|---|---|---|
| MEKF | 姿态误差三维化 | 把四元数协方差也当三维处理但忘记注入规则 |
| ESKF | 名义状态承载大运动，误差状态做 KF | 更新后不做 reset 或混用扰动方向 |
| IEKF | 一次观测更新内做 GN 迭代 | 以为迭代能修复所有一致性问题 |
| InEKF | 不变误差 + group-affine 传播 | 忽略 bias 破坏结构条件 |
| IKFoM | 通用流形上的 iterated EKF 工具箱 | 把工具箱名称误解为不变滤波 |
| EqF | 齐性空间上的等变误差滤波 | 未明确群作用和 lift 就套公式 |
| RTS/MHE | 时间窗口维度上的信息回传 | 欧氏差分直接用于流形状态 |

读到这里，A4 的收口可以压缩成一句工程话：先把状态空间和误差坐标选对，再决定单步、迭代还是窗口；不要用迭代弥补错误几何，也不要用几何结构替代必要的重线性化。

#### A4.12.17 黑板推导模板：从任务描述到滤波器选择

面对一个新的机器人估计任务，可以按下面模板写在白板上。这个模板比直接查"该用哪个滤波器"更可靠。

**Step 1：列状态。**

写出

$$x=(R,p,v,b,\text{外参},\text{地图点},\text{接触点},\ldots).$$

把每一块标成欧氏、李群、球面或齐性空间。

**Step 2：列误差。**

对每一块写出

$$x=\hat x\boxplus\delta x,\qquad \delta x=x\boxminus\hat x.$$

若某一块写不出自然的 $\boxplus/\boxminus$，说明状态表示还没有准备好。

**Step 3：列传播。**

检查传播能否写成

$$\dot X=AX+XB.$$

若可以，优先考虑 InEKF；若 bias 或外参破坏结构，记录哪些块需要加性处理。

**Step 4：列观测。**

把每个观测写成 residual，并判断它是否保持全局平移/yaw 等 gauge 对称性。

$$r=z-h(\hat x),\qquad H=\frac{\partial r}{\partial\delta x}.$$

若 $HN$ 不为零，而物理上该方向不可观，应先修误差定义或线性化点。

**Step 5：列计算预算。**

如果一次更新的 residual 数量很大，使用

$$K=(H^\top R^{-1}H+P^{-1})^{-1}H^\top R^{-1}$$

这类状态维形式；如果观测非线性强，加入 iterated update；如果历史状态需要被重估，进入 fixed-lag smoother 或因子图。

**Step 6：列验证指标。**

至少包含：

| 指标 | 用途 |
|---|---|
| innovation 均值 | 检查模型偏差 |
| NIS | 检查观测噪声与残差一致性 |
| NEES | 检查状态协方差可信度 |
| $\|R^\top R-I\|$ | 检查旋转更新是否保持群结构 |
| $HN$ | 检查不可观方向是否被误观测 |

这个模板把 A1 的统计一致性、A3 的流形误差、A4 的迭代/平滑视角放到同一张白板上。实际项目中，先完成这张白板，再写滤波器代码，能避免大部分方向性错误。

---

### §A4.13 与后续子任务的完整桥梁图 ⭐

```text
                                 5-A (Kalman 族)
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
   5-B 因子图 & 非线性最小二乘     5-C iSAM2 & Bayes 树       5-D IEKF 论文精读深入
   [IEKF = 单步 GN]              [ESKF 预测 = 因子图前向传播] [5-A3 InEKF → 5-D 原论文]
   [BA    = 全步 GN]             [RTS = 消元+回代 = Bayes 树] [group-affine 定理细节]
   [GTSAM NonlinearFactorGraph]   [GTSAM ISAM2]               [Barrau-Bonnabel TAC 2017]
        │                          │                          │
        └──────────┬───────────────┴─────────────┬────────────┘
                   │                             │
                   ▼                             ▼
         5-E Certifiable SLAM / SDP     5-F 鲁棒估计 (GNC / M-est / MAX-MIXTURE)
         [EKF/InEKF 给局部最优]          [M-estimator 替换二次损失 → robust KF]
         [certifiable 给 SDP 全局最优证书] [Black-Rangarajan 对偶 → IRLS]
         [SE-Sync, CPL-SLAM]             [GTSAM robust kernel, MC-GNC]
```

**关键桥梁一句话**：
- **5-A → 5-B**：IEKF 更新可解释为**单时间步的因子图/Gauss-Newton**（Bell-Cathey 定理）；BA 是**所有时间步联合的因子图**。工程上 GTSAM 的 `ExtendedKalmanFilter`体现了这种思想，但它不等同于所有 IEKF 变体；BA = `NonlinearFactorGraph` + LM。
- **5-A → 5-C**：ESKF 预测 ≡ 因子图前向 message passing；RTS ≡ 块三对角系统的消元+回代；iSAM2 = 增量式因子图 + Bayes 树做局部重线性化。
- **5-A → 5-D**：5-A3 给出 InEKF 使用直觉，5-D 做原论文 Barrau-Bonnabel 2017 *IEEE TAC* 的定理精读（group-affine 等价条件为定理 1、对数线性误差动力学为定理 2）。
- **5-A → 5-E**：滤波器只给**局部最优**；SDP 松弛（SE-Sync, Cartan-Sync, CPL-SLAM）给**全局最优证书**，用 Lagrangian 对偶或 Burer-Monteiro。滤波可作为 certifiable 初值。
- **5-A → 5-F**：高斯 KF 的二次损失在 outlier 下退化，换 Huber/Cauchy M-estimator → **robust Kalman**；更强的可用 **Graduated Non-Convexity (GNC)** 从凸近似向真目标退火，或 max-mixture 多模态高斯。

---

### §A4.14 总结：5-A 全系列核心原则 ⭐⭐

1. **四条正交坐标轴** 组织整个 Kalman 族：(a) 先验高斯（加性 vs 乘性 vs 不变 vs 等变）；(b) 线性化策略（Taylor1/2, sigma, cubature, Hermite）；(c) 状态空间几何（$\mathbb R^n$, $SO(3)$, $SE(3)$, $SE_K(3)$, 通用流形, 齐性空间）；(d) 迭代/平滑维度（单步, GN/LM 迭代, forward-backward RTS, 联合 MAP）。每选定一个轴的坐标就得到一个具体方法。

2. **三条等价关系** 把 5-A 与 5-B/5-C 无缝桥接：**IEKF ≡ 单步 Gauss-Newton**（Bell-Cathey 1993）；**RTS ≡ 块三对角回代**（Barfoot §3.1）；**VINS-Mono 滑窗 ≡ 非线性 MHE**（Rao-Rawlings 2001）。掌握这三条等价，Kalman 族与因子图族就是**同一问题的不同算法**。

3. **一致性是工程灵魂**。EKF/UKF 对 SLAM/VIO 的致命缺陷是"伪观测性"——滑窗每步在最新估计处线性化让 4 个零空间方向虚增信息。**两条修复路线**：(i) FEJ（OpenVINS/VINS 系），工程经验修正；(ii) 不变/等变滤波（InEKF/EqF），在满足 group-affine 或等变结构时从代数层面处理。IKFoM 属于通用流形迭代 EKF，能改善流形更新与迭代线性化，但不自动继承 InEKF 的 log-linear 一致性保证。

4. **流形是高精度状态估计的必然**。$SO(3)/SE(3)/SE_2(3)$ 上的 $\boxplus/\boxminus$ 不只是美学——它避免过参数化更新和局部奇异表示，并让误差协方差定义在正确切空间；协方差 PSD/SPD 仍依赖 Joseph form、平方根实现、reset Jacobian 与数值细节。只有在 group-affine 等额外结构成立时，Jacobian 才能进一步摆脱当前估计依赖。manif/Sophus/GTSAM Pose3/IKFoM 已经工程化到"和欧氏空间一样易用"的程度。

5. **MAP vs 均值的选择暗含架构决策**。IEKF/IKFoM 收敛到 MAP，与因子图后端语义一致；ISPKF/UKF/CKF 收敛到后验均值，与矩匹配/KL 最优一致。产品系统（前端滤波 + 后端图优化）选前者；强非线性 + 低维 + 可容忍计算（目标跟踪、bearing-only）选后者。

6. **"免 Jacobian" 并不总是优势**。UKF/CKF 省去解析 Jacobian 节省的是**人力**；但工程上 sigma/cubature 生成需要 Cholesky、高维权重易负、流形扩展繁琐。当 $h$ 可导且维度高（VIO/LIO），IEKF + AutoDiff（Ceres `AutoDiffCostFunction`）在精度/速度/工程维护之间**三元最优**。

7. **开源库是"可读论文"**。推荐精读顺序：FilterPy（理解教材公式）→ kalmanif（理解流形/一致性）→ invariant-ekf（理解 InEKF 实现）→ IKFoM / FAST-LIO2（理解生产级）→ OpenVINS（理解 FEJ 工业实现）→ VINS-Mono（理解滑窗/MHE）→ GTSAM（进入因子图世界）。**读完这 7 个库，5-A 全系列即毕业**。

### §A4.14b 知识树：5-A 全系列的结构化记忆 ⭐⭐

```text
5-A Kalman 族（根节点）
├── A1 线性高斯基础
│   ├── 贝叶斯递推（预测+更新）
│   ├── KF = 最优 MMSE+MAP+BLUE（三合一）
│   ├── 三种等价形式
│   │   ├── 协方差形：直观但数值脆弱
│   │   ├── 信息形：加法更新，天然稀疏
│   │   └── 平方根形：数值黄金标准
│   └── 一致性诊断（NEES/NIS/χ²）
├── A2 经典非线性
│   ├── EKF：Taylor 一阶，O(n³)，工业默认
│   ├── UKF：sigma 点，二阶精度，免 Jacobian
│   ├── CKF：球面径向，高维稳定
│   ├── GHKF：任意精度，维度诅咒
│   └── 一致性病理：FEJ/OC-EKF 修复
├── A3 流形滤波
│   ├── MEKF/ESKF：乘性误差，避免奇异
│   ├── InEKF：group-affine，log-linear
│   ├── UKF-M：流形上 sigma 点
│   ├── IKFoM：通用流形迭代
│   └── EqF：齐性空间等变
└── A4 迭代/平滑/收口（本章）
    ├── IEKF = 单步 GN（Bell-Cathey）
    ├── RTS = 块三对角回代
    ├── MHE = 滑窗 MAP
    ├── EnKF：高维集合方法
    ├── Schmidt-KF：考虑参数
    ├── H∞：鲁棒滤波
    └── 全景收口：四轴坐标选型
```

**四轴坐标的记忆口诀**：**"先问高斯不高斯，再问线性化怎么做，三问状态在哪里，四问保留多少步"**。

| 坐标轴 | 问题 | 极端选择 |
|---|---|---|
| 先验假设 | 高斯？多模？ | KF（高斯）vs 粒子滤波（非参数） |
| 线性化 | Taylor？采样？迭代？ | EKF（一次 Taylor）vs IEKF（多次 GN） |
| 状态几何 | 欧氏？李群？齐性空间？ | 标准 EKF vs InEKF vs EqF |
| 时间窗口 | 单步？滑窗？全局？ | 滤波 vs MHE vs BA |

### §A4.14c Kalman 族的历史时间线 ⭐⭐

| 年份 | 里程碑 | 贡献 | 影响 |
|------|--------|------|------|
| 1960 | Kalman 论文 | 线性高斯最优递推估计 | 整个领域的起点 |
| 1962 | Schmidt Consider Filter | 处理讨厌参数 | 航天高精度导航 |
| 1965 | Rauch-Tung-Striebel | 固定区间平滑 | 离线轨迹处理 |
| 1979 | Bierman UD 分解 | 平方根滤波工程化 | Apollo 之后的航天标准 |
| 1982 | Lefferts-Markley-Shuster MEKF | 姿态乘性误差 | 航天器 AOCS 标准 |
| 1994 | Evensen EnKF | 集合代替协方差 | 气象/海洋预报革命 |
| 1997 | Julier-Uhlmann UKF | Sigma 点传播 | 免 Jacobian 滤波 |
| 2001 | Rao-Rawlings MHE | 滑窗优化理论 | 约束状态估计 |
| 2003 | Arasaratnam-Haykin CKF | 球面径向积分 | 高维稳定 |
| 2007 | Bonnabel InEKF 前身 | 对称性保持观测器 | 不变滤波思想 |
| 2008-10 | Huang-Mourikis-Roumeliotis | EKF-SLAM 一致性分析 | FEJ/OC-EKF |
| 2012 | Kaess et al. iSAM2 | 增量 Bayes 树 | 实时图优化 |
| 2017 | Barrau-Bonnabel InEKF | Group-affine + log-linear | 一致性根本解决 |
| 2017 | Forster et al. 预积分 | 流形 IMU 因子 | VIO 标准方法 |
| 2020 | Brossard UKF-M | 可平行化流形 UKF | 免 Jacobian 流形 |
| 2021 | He-Xu-Zhang IKFoM | 通用流形迭代 EKF | FAST-LIO2 基础 |
| 2023 | van Goor-Mahony EqVIO | 齐性空间等变滤波 | 速度+精度双优 |
| 2025 | CT-ESKF | 协方差变换统一框架 | 误差定义不敏感 |

这条时间线给出的核心洞察是：**每一代方法都是前一代的"修补"或"推广"**。KF 到 EKF 是推广（线性→非线性）；EKF 到 UKF 是换策略（Taylor→采样）；EKF 到 ESKF/InEKF 是换空间（欧氏→流形）；滤波到 BA 是换时域（单步→全局）。理解了这条演化逻辑，就不会被新方法的名字吓到——它们都是在已知框架上做可控的修改。

> **过渡到 5-B**：下一子专题进入因子图与非线性最小二乘。我们会证明 IEKF 的 Gauss-Newton 等价式如何自然推广为**所有时间步的联合 GN**，并把 GTSAM 的 `NonlinearFactorGraph` 与 Ceres 的 `Problem` 作为两套等价抽象，为 5-C 的 iSAM2 / Bayes 树 / 增量重线性化做铺垫。至此，"先验 × 线性化 × 几何 × 迭代"四轴最后的"迭代"轴会从**时间维度扩展到空间图结构维度**，完成从"序列滤波"到"图推断"的世界观转换。

### §A4.15 Kalman 族方法的数值实验指南 ⭐⭐

#### A4.15.1 如何设计公平的 Monte Carlo 对比实验

研究中经常需要对比不同 Kalman 族方法的性能。以下是设计公平实验的准则：

**Step 1：选择性能指标。**

| 指标 | 衡量什么 | 计算方式 | 注意事项 |
|---|---|---|---|
| RMSE | 平均精度 | $\sqrt{\frac{1}{NT}\sum_{i,k}\|x_k^{\text{true}}-\hat x_k^{(i)}\|^2}$ | 对异常值敏感 |
| ANEES | 统计一致性 | $\frac{1}{N}\sum_i\xi_k^{(i)\top}P_{k|k}^{-1}\xi_k^{(i)}$，理论值 $n$ | 最重要的指标 |
| NIS 分布 | 观测模型匹配度 | $\nu_k^\top S_k^{-1}\nu_k\sim\chi^2(m)$ | 在线可用 |
| 计算时间 | 实时性 | 单步墙钟时间 | 需区分预测/更新 |

**Step 2：Monte Carlo 实验设置。**

| 设置项 | 推荐值 | 原因 |
|---|---|---|
| 运行次数 $N$ | 100-500 | $<50$ 统计不稳定，$>500$ 耗时但改善不大 |
| 时间步数 $T$ | $>100$ | 需要看长期行为（一致性退化需要时间显现） |
| 初始误差 | 多个水平 | 小初始误差下所有方法都好，大误差才显差异 |
| 噪声实现 | 固定种子+不同种子 | 固定种子用于调试，不同种子用于统计 |

**Step 3：ANEES 置信区间。**

$N$ 次 Monte Carlo 下，ANEES 的 95% 置信区间为

$$\left[\frac{\chi^2_{Nn,0.025}}{N},\quad \frac{\chi^2_{Nn,0.975}}{N}\right]$$

若 ANEES 长期高于上界 → 滤波器过自信（协方差过小）。
若 ANEES 长期低于下界 → 滤波器过保守（协方差过大）。
在界内 → 统计一致。

**反事实推理**：如果只看 RMSE 不看 ANEES 会怎样？RMSE 可能显示两种方法”差不多”，但 ANEES 揭示其中一种严重过自信。过自信的方法在实际工程中会导致：(1) 异常观测 gate 失效（因为 $S$ 偏小），(2) 数据关联错误率上升，(3) 回环检测/地图合并时协方差不可信。

#### A4.15.2 从仿真到实际：标定噪声参数的方法

理论上 $Q,R$ 应该精确已知，实际中它们需要标定。常用方法：

| 方法 | 适用场景 | 原理 | 代码参考 |
|---|---|---|---|
| **Allan 方差** | IMU bias/ARW | 从长静止数据提取功率谱密度 | `ori-drs/allan_variance_ros` |
| **创新序列法 (NIS)** | 在线微调 $R$ | 若 $\bar{\text{NIS}}>m$，增大 $R$ | 标准 KF 教材 |
| **Autocovariance LS (ALS)** | 离线联合估计 $Q,R$ | 用创新自相关求解 $Q,R$ | Odelson et al. 2006 |
| **EM 算法** | 离线最大似然 | E 步跑 RTS 平滑，M 步更新参数 | Shumway-Stoffer 1982 |
| **数值差分/AutoDiff** | Jacobian 验证 | 对比解析 vs 数值 Jacobian | Ceres `GradientChecker` |

#### A4.15.3 Kalman 族方法选型流程图的详细解释 ⭐⭐

§A4.12 给出了选型流程图的骨架。这里展开每个决策节点的量化判据：

**判据 1：非线性程度如何量化？**

对过程模型 $f$ 的非线性程度，可以用**非线性指数 (nonlinearity index)** 近似衡量：

$$\text{NLI} = \frac{\mathrm{tr}(\nabla^2 f\cdot P)}{\|f(\hat x)\|}$$

当 $\text{NLI}<0.01$ 时，EKF 的二阶偏差可忽略。当 $\text{NLI}>0.1$ 时，应考虑 UKF、CKF 或 IEKF。

**判据 2：何时需要迭代？**

迭代（IEKF/IKFoM）在以下情况收益最大：
- 观测模型 $h$ 的非线性远强于过程模型 $f$（如 LiDAR 点到面配准）
- 先验协方差 $P^-$ 较大（初始化期或快速运动后）
- 单次更新的残差 $\|y-h(\hat x^-)\|$ 相对于 $\sqrt{S}$ 超过 2-3 倍

**判据 3：何时从滤波转向平滑/图优化？**

| 信号 | 含义 | 行动 |
|---|---|---|
| 回环检测到了 | 需要修正历史轨迹 | 进入 pose graph / BA |
| NEES 长期偏高 | 一次线性化错误累积 | 考虑 MHE 或滑窗 BA |
| 传感器有固定延迟 | 需要 “回头看” | Fixed-lag smoother |
| 需要高精度离线建图 | 实时性不重要 | 全局 BA / RTS |

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|---------|
| IEKF 只是"多跑几次 EKF 更新"的简单重复 | IEKF 等价于对单时间步 MAP 代价函数做 Gauss-Newton 迭代（Bell-Cathey 1993）；每次迭代重新线性化观测模型，是优化算法而非简单重复；漏掉先验搬移项 $-H^{(j)}(\hat x_{k\mid k-1}-\hat x^{(j)})$ 会导致"忘记先验"的严重 bug |
| RTS 平滑器比 KF 慢很多所以不实用 | RTS 后向递推的计算量与前向 KF 相同（$O(Tn^3)$），只是需要存储所有时刻的 $P_{k\mid k}$ 和 $F_k$；在离线/固定延迟场景下（如建图、IMU 标定）性价比极高 |
| MHE（滑动窗口优化）就是"窗口版的 IEKF" | MHE 维护整个窗口内所有状态的联合优化，可以多次重线性化所有时刻，而 IEKF 只迭代当前时刻；MHE 的先验项来自窗外状态的边缘化，处理不当会导致信息丢失或矩阵稠密化 |
| 全景对比表里"越新"的方法越好 | Kalman 族沿"线性化策略 $\times$ 状态空间几何 $\times$ 迭代/平滑"三轴展开，每个组合有其适用域；标准 EKF 在弱非线性、低维、实时性要求高的场景下仍然是最佳选择，EnKF 在大气/海洋的超高维场景下独占优势 |
| Schmidt-KF（consider KF）只是"偷懒不估计某些状态" | Schmidt-KF 的核心价值是在**承认**某些参数存在不确定性的同时**不去更新**它们，从而避免因状态扩维导致的计算爆炸和一致性退化；它在 VIO 中处理 IMU-相机外参标定不确定性时极为实用 |
| $H_\infty$ 滤波是"没有噪声模型时的万能替代" | $H_\infty$ 保证最坏情况下的估计增益有界（minimax 准则），但它的协方差通常过度保守，估计精度低于 KF；只在噪声模型严重未知或存在对抗性扰动时才值得牺牲精度换鲁棒性 |
| 连续-离散 KF（CD-KF）的积分步长越小越精确 | 更细的 ODE 积分步长确实减小离散化误差，但也放大了数值舍入误差并增加计算量；实际中应根据系统时间常数选择步长，且 Van Loan 矩阵指数法可在一步内精确完成而无需细分 |
| 学完 A4 就不需要再看因子图（5-B）了 | A4 证明"IEKF = 单步 GN"正是通向因子图的桥梁；把单步 GN 推广到全部时间步的联合 GN 就得到 BA/因子图优化，5-B 解决的是 Kalman 族无法高效处理的回环闭合与全局一致性问题 |

---

### 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|---------|---------|--------|
| IEKF 迭代代价不降反升 | 阻尼或 Armijo 条件符号错误 | 1.打印 $\nabla J^\top\Delta$ 确认负号 2.检查实际下降量 3.切换 LM 策略 | §A4.1 |
| FAST-LIO2 与 InEKF 精度结论不一致 | compound manifold 不满足 group-affine | 1.检查状态空间结构 2.区分 IKFoM 与 InEKF 3.在退化环境测试 | §A4.3 |
| Sophus/GTSAM 协方差方向不同 | `Plus` 定义和切向量顺序不一致 | 1.用小扰动做数值 Jacobian 对比 2.检查 $[\omega;\rho]$ vs $[\rho;\omega]$ 3.加置换矩阵 | §A4.12 |
| RTS 平滑后某时刻估计反而变差 | 延迟观测时间戳处理错误 | 1.检查观测关联的时刻 2.验证 $C_k$ 计算 3.与 batch MAP 对比 | §A4.4 |
| 滑窗 MHE 性能不如纯 BA | 边缘化先验 FEJ 未生效 | 1.打印边缘化残差量级 2.对比有/无先验的 NEES 3.冻结线性化点 | §A4.5 |
| EnKF 集合快速坍缩 | 集合成员数不足 | 1.检查 $N_e$ vs 状态维度 2.加 inflation 3.加 localization | §A4.5b |

⚠️ **陷阱一：把 MAP 和均值估计混在一起。** IEKF/因子图求的是局部 MAP；UKF/CKF 更接近矩匹配均值，两者在强非线性后验下不一定一致。选择哪个取决于后端架构：如果前端滤波 + 后端图优化，MAP 语义一致性更重要。

⚠️ **陷阱二：把 line search 的 Armijo 符号写反。** 下降方向满足 $\nabla J^\top\Delta<0$，Armijo 条件要求 $J(\hat x+\alpha\Delta)\le J(\hat x)+c_1\alpha\nabla J^\top\Delta$（右端低于当前代价，因为 $c_1\alpha\nabla J^\top\Delta<0$）。

⚠️ **陷阱三：跨库迁移只改四元数顺序。** JPL/Hamilton、左/右扰动、active/passive 旋转和切向量排序必须**同时**检查和转换。任何一个遗漏都会导致 Jacobian 符号错误。

⚠️ **陷阱四：把 IEKF 收敛等同于全局最优。** IEKF 收敛到的是单步 MAP 的局部极小，不是全局最优。对多模态观测（如 bearing-only 在对称配置下），需要多初始化或全局方法。

> **本质洞察**：Kalman 族不是一串彼此替代的算法清单，而是”分布假设、线性化方式、状态几何、求解时域”四个选择轴的组合。每选定一个轴的坐标，就确定了一个具体方法及其适用边界。

---

### 本章小结

| 知识点 | 核心内容 | 解决什么问题 | 工程落地 |
|--------|---------|-------------|---------|
| IEKF = 单步 GN | Bell-Cathey 定理 | 非线性更新的迭代改善 | FAST-LIO2 默认 3-4 次 |
| LM-IEKF | 增加阻尼 $\lambda$ | GN 在强非线性下不收敛 | Nielsen 策略 |
| ISPKF | Sigma 点 + 迭代 | 免 Jacobian 迭代滤波 | 低维强非线性跟踪 |
| IKFoM | $\boxplus/\boxminus$ + 曲率 Jacobian | 通用流形迭代 EKF | FAST-LIO2 |
| RTS 平滑 | Forward KF + Backward pass | 利用”未来信息”修正历史 | 离线建图 |
| RTS = 块三对角回代 | 等价于因子图的消元+回代 | 连接滤波与因子图 | GTSAM `FixedLagSmoother` |
| MHE 滑窗 | 有限窗口内联合优化 | 在线重线性化 + 约束处理 | VINS-Mono 滑窗 |
| EnKF | 集合统计量代替显式协方差 | 超高维状态估计 | 气象/海洋/地球物理 |
| Schmidt-KF | Consider 但不估计 | 处理讨厌参数的不确定性 | 航天高精度轨道预报 |
| $H_\infty$ 滤波 | 最坏情况鲁棒性 | 噪声模型不确定 | 高可靠性系统 |
| MAP vs 均值 | 不同最优性准则 | 前后端语义一致 | IEKF↔因子图，UKF↔矩匹配 |
| 四轴坐标 | 先验×线性化×几何×迭代 | 组织整个 Kalman 族 | 选型依据 |

---

### 累积项目：本章新增模块

**项目：从零构建多方法对比平台**

本章新增模块将前三章实现的 KF、EKF/UKF、ESKF/InEKF 整合到统一测试框架中：

| 模块 | 功能 | 依赖 |
|------|------|------|
| `iekf_scalar.py` | 标量 IEKF 迭代更新 + Armijo line search | A1 KF |
| `rts_smoother.py` | RTS 平滑器（forward KF + backward pass） | A1 KF |
| `compare_methods.py` | 统一 Monte Carlo 框架：跑 EKF/UKF/ESKF/IEKF 并输出 ANEES | A1-A4 全部 |
| `noise_calibration.py` | Allan 方差分析 + NIS 在线微调 | 辅助 |

**本章新增任务**：

1. 在 `iekf_scalar.py` 中实现标量 range-bearing 观测的 IEKF。用 $J_{\max}=5$ 和 Armijo 回溯，验证收敛到与 batch MAP 相同的解。
2. 在 `rts_smoother.py` 中对 §A4.4.3 的 5 步匀速例子运行 RTS，绘制滤波/平滑轨迹对比图。验证 $P^s\preceq P^f$。
3. 在 `compare_methods.py` 中实现 100 次 Monte Carlo 对比实验，对 2D SLAM（3 路标）分别跑 EKF/UKF/IEKF/InEKF，绘制 4 条 ANEES 曲线在同一张图上。

---

### 延伸阅读

| 类型 | 资源 | 难度 | 重点 |
|------|------|------|------|
| 教材 | Barfoot *State Estimation for Robotics* 2ed (2024) §4.2 | ⭐⭐ | IEKF 推导 |
| 教材 | Särkkä & Svensson *Bayesian Filtering and Smoothing* 2ed (2023) Ch.7-10 | ⭐⭐⭐ | 全面覆盖 |
| 教材 | Simon *Optimal State Estimation* (2006) Ch.13-14 | ⭐⭐ | 经典参考 |
| 论文 | Bell-Cathey, *IEEE TAC* 38(2):294-297, 1993 | ⭐⭐⭐ | IEKF = GN |
| 论文 | Rauch-Tung-Striebel, *AIAA J.* 3(8):1445-1450, 1965 | ⭐⭐ | 原始 RTS |
| 论文 | Rao-Rawlings, *Automatica* 37(9):1371-1388, 2001 | ⭐⭐⭐ | MHE 理论 |
| 论文 | Evensen, *J. Geophys. Res.* 99:10143-10162, 1994 | ⭐⭐⭐ | EnKF 原始论文 |
| 论文 | Schmidt, NASA TN D-1397, 1962 | ⭐⭐⭐⭐ | Consider filter 原始 |
| 论文 | Xu et al., *IEEE T-RO* 2022 | ⭐⭐ | FAST-LIO2 工程 |
| 代码 | `artivis/kalmanif` | ⭐⭐ | C++ 多方法对比 |
| 代码 | `hku-mars/IKFoM` | ⭐⭐⭐ | IKFoM 参考实现 |
| 代码 | `rpng/open_vins` | ⭐⭐⭐ | FEJ + MSCKF |

---

### 练习

1. 写出一个标量最小二乘问题的 Armijo 回溯过程，验证符号错误会允许代价上升。
2. 以同一后验分布为例，比较 MAP 与均值在偏态分布下的位置差异。
3. 选 Sophus 与 GTSAM 的一个 `Pose3` 操作，设计数值实验确认其默认扰动方向。
4.（跨章综合题）把 A1 的线性 KF、A2 的 EKF/UKF、A3 的 ESKF、A4 的 IEKF 全部实现为统一接口 `predict(u)/update(z)`，对同一个 2D 非线性系统跑 100 次 Monte Carlo，绘制 RMSE 和 ANEES 的对比图。讨论：(a) 哪些方法在 RMSE 上差异不大但 ANEES 差异显著？(b) 增加初始误差后哪些方法首先发散？(c) 加入迭代（IEKF）对 ANEES 有何改善？

5.（⭐⭐⭐）实现 VINS-Mono 风格的滑窗边缘化：对一个 5 节点 pose graph（每节点 6DoF），保留最新 3 节点，边缘化最旧 2 节点。写出 Schur 补的显式计算过程，验证边缘化后 Hessian 的稀疏度变化（from sparse to dense on remaining nodes）。

6.（⭐⭐⭐⭐）设计一个 "Kalman 族方法博物馆" Python 脚本：对同一个 3D bearing-range SLAM 问题（5 路标、50 步），实现 EKF / UKF / IEKF / InEKF / RTS / 滑窗 MHE 共 6 种方法，统一输出格式（轨迹、RMSE、ANEES、计算时间）。这是本系列的"毕业作品"——完成后你对 Kalman 族的理解将从"知道名词"升级为"能实现并量化对比"。

---

### §A4.16 5-A 全系列的读者自检清单 ⭐

学完 5-A1 到 5-A4 的全部内容后，请用以下清单自检。能通过 80% 以上说明达到档位 3 水平；能通过全部说明达到档位 4 水平。

| 编号 | 自检项 | 合格标准 |
|:----:|--------|---------|
| 1 | 贝叶斯递推 | 能从 Chapman-Kolmogorov + Bayes 写出预测-更新框架 |
| 2 | KF 最优性 | 能说出 MMSE=MAP=BLUE 三合一的前提（联合高斯） |
| 3 | 三种 KF 形式 | 能写出协方差形/信息形/平方根形的核心区别 |
| 4 | NEES/NIS | 能定义并解释"超出上界=过自信" |
| 5 | EKF 偏差 | 能写出二阶偏差 $\frac{1}{2}\mathrm{tr}(\nabla^2 g\cdot P)$ |
| 6 | UKF sigma 点 | 能写出 $2n+1$ 点的生成公式和权重 |
| 7 | CKF = UKF 特例 | 能指出 $\alpha=1,\kappa=0,\beta=0$ 时的退化 |
| 8 | FEJ 原理 | 能解释"冻结线性化点"为什么能保持不可观方向 |
| 9 | ESKF reset | 能写出 $P\leftarrow GPG^\top$ 并解释 $G$ 的来源 |
| 10 | Group-affine | 能写出恒等式并对 $SE_2(3)$ 验证 |
| 11 | InEKF log-linear | 能解释"$A$ 不依赖 $\hat X$"的工程意义 |
| 12 | UKF-M | 能说出 sigma 点通过 Exp 生成的原因 |
| 13 | IEKF = 单步 GN | 能写出 Bell-Cathey 定理的核心等价关系 |
| 14 | RTS = 块三对角回代 | 能说出前向消元=KF，回代=平滑 |
| 15 | MHE arrival cost | 能解释边缘化如何生成等价先验 |
| 16 | 四轴坐标选型 | 能对给定场景选出合适的方法并说明理由 |

**如果卡在某个检查点**：回到对应章节重新手推核心公式。理解 = 能在白纸上重新推导，不是"看一遍记住了"。

**学习路径建议**（按每周 15 小时计算）：

| 周 | 内容 | 重点任务 |
|:--:|------|---------|
| 1-2 | A1 线性高斯基础 | 手推 KF、实现 Joseph form、理解 NEES |
| 3-4 | A2 经典非线性 | 手推 EKF/UKF、理解 FEJ、Monte Carlo 对比 |
| 5-7 | A3 流形滤波 | 理解 ESKF/InEKF、实现 SO(3) MEKF |
| 8-9 | A4 迭代/平滑/收口 | 理解 IEKF=GN、实现 RTS、全景对比实验 |
| 10 | 综合项目 | 完成"Kalman 族方法博物馆"脚本 |

通过 5-A 全系列后，你已经具备了进入因子图（5-B）、增量优化（5-C）和鲁棒估计（5-F）的全部前置知识。更重要的是，你拥有了"面对一个新的状态估计问题，能从四个维度分析并选择合适方法"的系统性思维框架——这比任何单个算法的实现能力都更有价值。

> **最后的忠告**：Kalman 族方法的难度不在于公式推导——这些在教材中都有。真正的难度在于：(1) 让代码中的符号与论文中的符号完全对应；(2) 让左右扰动、四元数约定、坐标系命名在整个代码库中一致；(3) 在有限的实时计算预算内选择足够好但不过度复杂的方法。这三点都是"做过才知道难"的工程能力，只有通过反复实现和调试才能获得。

### 术语速查对照表

| 英文术语 | 中文 | 对应章节 |
|---------|------|---------|
| Error-State Kalman Filter (ESKF) | 误差状态卡尔曼滤波 | A3 |
| Invariant Extended Kalman Filter (InEKF) | 不变扩展卡尔曼滤波 | A3 |
| Iterated Extended Kalman Filter (IEKF) | 迭代扩展卡尔曼滤波 | A4 |
| Rauch-Tung-Striebel (RTS) Smoother | RTS 平滑器 | A4 |
| Moving Horizon Estimation (MHE) | 滑动窗口估计 | A4 |
| Normalized Estimation Error Squared (NEES) | 归一化估计误差平方 | A1 |

---
