## 专题4：雅可比矩阵体系与BCH公式

> **前置要求**：已完成专题3（李群基础与SO(3)/SE(3)的exp/log/Adjoint闭式）
> **核心档位**：档位3（能推导核心定理） | **进阶档位**：档位4（能证明收敛性、理解前沿）
> **建议时长**：档位3约30–40小时，档位4额外15–20小时，建议2–3周完成

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含练习与手推） | 8–10 小时 | 需要独立推导所有 Jacobian 的读者 |
| 速读（跳过推导细节） | 3–4 小时 | 已有李群基础、只需补全 BCH 与 convention 知识的读者 |
| 速查（只看表格和速查卡） | 30–45 分钟 | 遇到具体 Jacobian 公式或 convention 问题时回来查 |

---

### 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回专题 3 复习**

| 编号 | 问题 | 答不出→回顾 |
|:----:|------|------------|
| 1 | 写出 SO(3) 的 Rodrigues 公式 $\exp(\theta n^\wedge) = I + \sin\theta\,n^\wedge + (1-\cos\theta)(n^\wedge)^2$。它的推导依赖反对称矩阵的什么代数性质？ | 专题 3 第 3 节 |
| 2 | SE(3) 指数映射中平移项为什么是 $V\rho$ 而不是 $\rho$？$V$ 矩阵的物理含义是什么？ | 专题 3 第 5.2 节 |
| 3 | Adjoint $\text{Ad}_T$ 把什么空间的向量变换到什么空间？写出 SE(3) 的 Adjoint 矩阵（平移在前排序）。 | 专题 3 第 6 节 |
| 4 | 左扰动 $T' = \exp(\delta\xi^\wedge) T$ 和右扰动 $T' = T \exp(\delta\xi^\wedge)$ 的区别是什么？扰动向量分别表达在哪个坐标系？ | 专题 3 第 8 节 |
| 5 | BCH 公式的一阶近似 $\log(\exp(X)\exp(Y)) \approx X + Y$ 在什么条件下成立？如果 $[X,Y] \neq 0$ 会出现什么额外项？ | 专题 3 第 7 节 |

---

#### 0. 为什么本专题是整个SLAM数学体系的枢纽 ⭐

所有非线性最小二乘——Bundle Adjustment、Pose-Graph Optimization、VIO预积分——最终都归结为**在李群上对残差求Jacobian**。Jacobian 在李群优化中的角色，可以类比微积分中导数在 Newton 法中的角色：Newton 法用 $f'(x)$ 把非线性方程 $f(x) = 0$ 局部线性化为 $f(x) + f'(x)\delta x = 0$，而李群优化用 Jacobian 把非线性残差 $r(X)$ 在群上局部线性化为 $r + J \delta\xi = 0$。区别在于：Newton 法的导数定义在欧氏空间，$\delta x$ 直接加到 $x$ 上；李群上的 Jacobian 定义在切空间，$\delta\xi$ 必须通过 Exp 映射搬回群上。这个额外步骤正是 $J_l$ 和 $J_r$ 存在的根本原因——它们度量了从切空间到群上的"搬运成本"。专题3给出了exp/log/Adjoint的闭式表达，但实际优化中需要的是这些映射**对扰动参数的导数**，这正是本专题要建立的完整体系。**左Jacobian $\mathbf{J}_l$ 和右Jacobian $\mathbf{J}_r$** 是exp映射对其李代数参数的微分，是整个体系的核心算子；**BCH公式**回答"两个exp相乘等于什么exp"，是误差传播与IMU预积分的数学基础；**Adjoint Jacobian** 则充当左/右扰动模型之间的翻译词典，让你能在不同代码库的convention之间自由切换。完成本专题后，你将能**独立推导SLAM论文中出现的所有Jacobian表达式**。

---

#### 1. 核心章节清单（档位3必学） ⭐

##### §4.1 扰动模型的两大流派：左扰动 vs 右扰动 ⭐⭐

这是整个Jacobian体系的起点。在李群上做优化时，"对姿态求导"意味着在当前估计 $\bar{X}$ 附近施加微小扰动 $\delta\boldsymbol{\xi}$，然后令 $\delta\boldsymbol{\xi}\to 0$ 求极限。扰动放在哪一侧，直接决定了Jacobian的形式。

| 扰动类型 | 公式 | 物理含义 | 典型使用者 |
|:---------|:-----|:---------|:----------|
| **左扰动** | $X = \mathrm{Exp}(\delta\boldsymbol{\xi})\cdot\bar{X}$ | 扰动在世界/基座坐标系 | Barfoot书、Eade笔记、Sophus典型用法 |
| **右扰动** | $X = \bar{X}\cdot\mathrm{Exp}(\delta\boldsymbol{\xi})$ | 扰动在Body坐标系 | Solà论文、manif库、GTSAM、Forster预积分 |
| Central（较少见） | $X = \mathrm{Exp}(\tfrac{1}{2}\delta\boldsymbol{\xi})\cdot\bar{X}\cdot\mathrm{Exp}(\tfrac{1}{2}\delta\boldsymbol{\xi})$ | 对称扰动 | 少数不确定性传播工作 |

**关键点**：两种扰动通过Adjoint互转——$\mathrm{Exp}(\delta\boldsymbol{\xi}_\text{left})\cdot\bar{T} = \bar{T}\cdot\mathrm{Exp}(\mathrm{Ad}(\bar{T})^{-1}\cdot\delta\boldsymbol{\xi}_\text{left})$。选定convention后须全程一致，混用是最常见的错误来源。

##### §4.2 左Jacobian $\mathbf{J}_l$ 与右Jacobian $\mathbf{J}_r$ 的推导 ⭐⭐

**核心定义**（以右Jacobian为例）：

$$\mathrm{Exp}(\boldsymbol{\phi}+\delta\boldsymbol{\phi}) \approx \mathrm{Exp}(\boldsymbol{\phi})\cdot\mathrm{Exp}(\mathbf{J}_r(\boldsymbol{\phi})\,\delta\boldsymbol{\phi})$$

对应地，左Jacobian满足：

$$\mathrm{Exp}(\boldsymbol{\phi}+\delta\boldsymbol{\phi}) \approx \mathrm{Exp}(\mathbf{J}_l(\boldsymbol{\phi})\,\delta\boldsymbol{\phi})\cdot\mathrm{Exp}(\boldsymbol{\phi})$$

**推导路径**：从exp的Taylor展开出发，利用 $\mathrm{ad}$ 算子（李括号的矩阵表示），可以证明：

$$\mathbf{J}_l(\boldsymbol{\phi}) = \int_0^1 \exp(t\,\mathrm{ad}(\boldsymbol{\phi}))\,dt = \sum_{k=0}^{\infty}\frac{\mathrm{ad}(\boldsymbol{\phi})^k}{(k+1)!}$$

$$\mathbf{J}_r(\boldsymbol{\phi}) = \int_0^1 \exp(-t\,\mathrm{ad}(\boldsymbol{\phi}))\,dt = \mathbf{J}_l(-\boldsymbol{\phi})$$

**核心关系**：$\mathbf{J}_r(\boldsymbol{\phi}) = \mathbf{J}_l(-\boldsymbol{\phi})$，以及 $\mathbf{J}_l(\boldsymbol{\phi}) = \mathrm{Ad}(\mathrm{Exp}(\boldsymbol{\phi}))\cdot\mathbf{J}_r(\boldsymbol{\phi})$。

学习建议：先理解 $\mathrm{ad}$ 算子在SO(3)上就是叉乘矩阵 $[\boldsymbol{\phi}]_\times$，再手推级数前三项，体会为什么会出现 $\sin\theta/\theta$ 等系数。

##### §4.3 SO(3)上的闭式Jacobian ⭐⭐

由于SO(3)上 $\mathrm{ad}(\boldsymbol{\phi})^3 = -\theta^2\,\mathrm{ad}(\boldsymbol{\phi})$（其中 $\theta=\|\boldsymbol{\phi}\|$），级数可以封闭求和，得到 **$3\times3$ 闭式**：

$$\mathbf{J}_l(\boldsymbol{\phi}) = \frac{\sin\theta}{\theta}\mathbf{I} + \left(1-\frac{\sin\theta}{\theta}\right)\mathbf{a}\mathbf{a}^T + \frac{1-\cos\theta}{\theta}[\mathbf{a}]_\times$$

其中 $\mathbf{a}=\boldsymbol{\phi}/\theta$ 为单位轴。右Jacobian只需把 $\boldsymbol{\phi}$ 取负即可。

| 表达式 | 闭式要点 | 数值注意事项 |
|:------|:--------|:-----------|
| $\mathbf{J}_l(\boldsymbol{\phi})$ | $\sin\theta/\theta$ 项、$(1-\cos\theta)/\theta$ 项 | $\theta\to 0$ 时须用Taylor展开避免0/0 |
| $\mathbf{J}_r(\boldsymbol{\phi})$ | $= \mathbf{J}_l(-\boldsymbol{\phi})$ | 同上 |
| $\mathbf{J}_l^{-1}(\boldsymbol{\phi})$ | $= \mathbf{I} - \tfrac{1}{2}[\boldsymbol{\phi}]_\times + e_\theta[\boldsymbol{\phi}]_\times^2$，含 $\cot(\theta/2)$ 项 | 需单独推导/查表 |
| $\mathbf{J}_r^{-1}(\boldsymbol{\phi})$ | $= \mathbf{J}_l^{-1}(-\boldsymbol{\phi})$ | 验证：$\mathbf{J}_r\cdot\mathbf{J}_l^{-1}$ 应等于 $\mathrm{Ad}(\mathrm{Exp}(\boldsymbol{\phi}))^{-1}$ |

**自测**：手推 $\mathbf{J}_l$ 闭式——这是本专题最核心的一道推导练习（参考Solà论文Appendix B、Barfoot §8.2.3、Eade exp_diff.pdf §5.1）。

##### §4.4 SE(3)上的Jacobian ⭐⭐⭐

SE(3)的Jacobian是 **$6\times6$ 块矩阵**。以下采用 Sola/Barfoot 约定，切向量排序为 $[\boldsymbol{\rho}, \boldsymbol{\phi}]$（**平移在前**），结构为：

$$\mathbf{J}_l^{\mathrm{SE}(3)} = \begin{bmatrix} \mathbf{J}_l^{\mathrm{SO}(3)} & \mathbf{Q}_l \\ \mathbf{0} & \mathbf{J}_l^{\mathrm{SO}(3)} \end{bmatrix}$$

> **排序约定警告**：若使用"旋转在前" $[\boldsymbol{\phi}, \boldsymbol{\rho}]$ 排序（GTSAM 约定），则 $\mathbf{Q}_l$ 出现在左下而非右上：$\mathbf{J}_l = \begin{bmatrix} \mathbf{J}_l^{\mathrm{SO}(3)} & \mathbf{0} \\ \mathbf{Q}_l & \mathbf{J}_l^{\mathrm{SO}(3)} \end{bmatrix}$。切向量排序是 SE(3) 工程中最常见的 bug 来源之一。

**Q矩阵** 编码了旋转与平移之间的耦合项，其表达式涉及 $\boldsymbol{\phi}$（旋转部分）和 $\boldsymbol{\rho}$（平移部分）的多项交叉乘积。Q矩阵与专题3中的V矩阵密切相关——V矩阵是exp映射中从 $\boldsymbol{\rho}$ 到平移分量的线性映射，而Q矩阵出现在对 $\boldsymbol{\phi}$ 微分时的耦合效应中。

此处闭式表达式冗长但结构清晰，建议直接参考Solà论文Appendix D或Barfoot §8.2.4的Q矩阵定义，不必死记。实际工程中优先使用manif库的 `w.ljac()` / `w.rjac()` 来获取 $6\times6$ 数值结果。

##### §4.5 常用表达式的Jacobian速查 ⭐⭐

以下是SLAM/VIO中反复出现的基本Jacobian，建议逐一推导一遍后作为速查表保存：

| 表达式 | Jacobian（右扰动） | 应用场景 |
|:------|:------------------|:--------|
| $\mathbf{R}^T\mathbf{p}$ 对 $\delta\boldsymbol{\theta}$（右扰动 $R \leftarrow R\cdot\mathrm{Exp}(\delta\theta)$） | $[\mathbf{R}^T\mathbf{p}]_\times$ | 旋转一个向量对旋转扰动 |
| $\mathbf{T}\cdot\mathbf{p}$ 对 $\delta\boldsymbol{\xi}$（右扰动 $T \leftarrow T\cdot\mathrm{Exp}(\delta\xi)$） | $\begin{bmatrix}\mathbf{R} & -\mathbf{R}[\mathbf{p}]_\times\end{bmatrix}$（$3\times6$ 矩阵，平移在前排序） | 重投影误差中最常见的项 |
| $\mathrm{Log}(\mathbf{R}_1^T\mathbf{R}_2)$ 对 $\delta\boldsymbol{\theta}_1$（右扰动 $R_1 \leftarrow R_1\cdot\mathrm{Exp}(\delta\theta)$） | $-\mathbf{J}_l^{-1}(\mathrm{Log}(\mathbf{R}_1^T\mathbf{R}_2))$ | PGO中的旋转残差 |
| $\mathbf{T}_1^{-1}\cdot\mathbf{T}_2$ 对 $\delta\boldsymbol{\xi}_1$ 和 $\delta\boldsymbol{\xi}_2$ | 含Adjoint的between factor | PGO / loop closure |
| $\mathrm{Exp}(\mathbf{X})\cdot\mathbf{Y}\cdot\mathrm{Exp}(-\mathbf{X})$ 对 $\mathbf{X}$ | $\mathrm{ad}(\mathbf{Y})$ | 伴随表示的微分 |

推导思路统一：施加扰动 → 展开到一阶 → 提取 $\delta\boldsymbol{\xi}$ 前面的矩阵。高翔《十四讲》第7讲PnP重投影误差给出了完整的工程示例。

##### §4.6 Adjoint Jacobian：扰动模型间的桥梁 ⭐⭐⭐

$\mathrm{Ad}(\mathbf{T})$ 本质上就是**从右扰动到左扰动的Jacobian**。对于SO(3)，$\mathrm{Ad}(\mathbf{R})=\mathbf{R}$（一个巧合性简化）。对于SE(3)，Adjoint是 $6\times6$ 矩阵且含平移-旋转耦合项。以下采用"平移在前" $[\boldsymbol{v}, \boldsymbol{\omega}]$ 排序（Sola/Barfoot 约定）：$\mathrm{Ad}(\mathbf{T}) = \begin{bmatrix}\mathbf{R} & [\mathbf{t}]_\times\mathbf{R}\\\mathbf{0}&\mathbf{R}\end{bmatrix}$。若用"旋转在前" $[\boldsymbol{\omega}, \boldsymbol{v}]$ 排序（GTSAM 约定），耦合项 $[\mathbf{t}]_\times\mathbf{R}$ 的位置从右上变为左下。

当你读到一篇用左扰动写的论文、而代码库用右扰动时，转换公式为：$\mathbf{J}^\text{left} = \mathrm{Ad}(\bar{\mathbf{T}})\cdot\mathbf{J}^\text{right}$。这是跨convention的"翻译词典"。

##### §4.7 BCH公式及其应用 ⭐⭐

**完整形式**：$\mathrm{Log}(\mathrm{Exp}(\mathbf{X})\cdot\mathrm{Exp}(\mathbf{Y})) = \mathbf{X}+\mathbf{Y}+\tfrac{1}{2}[\mathbf{X},\mathbf{Y}]+\tfrac{1}{12}\big([\mathbf{X},[\mathbf{X},\mathbf{Y}]]-[\mathbf{Y},[\mathbf{X},\mathbf{Y}]]\big)+\cdots$

**工程中最常用的一阶近似**（$\delta\mathbf{Y}$ 为小量）：

$$\mathrm{Log}(\mathrm{Exp}(\boldsymbol{\phi})\cdot\mathrm{Exp}(\delta\boldsymbol{\phi})) \approx \boldsymbol{\phi} + \mathbf{J}_r^{-1}(\boldsymbol{\phi})\,\delta\boldsymbol{\phi}$$

**在IMU预积分中的直接应用**（Forster TRO 2017）：预积分的关键在于将旋转增量的噪声 $\delta\boldsymbol{\phi}$ 从exp内部"提取"出来。通过BCH一阶展开，$\mathrm{Exp}(\boldsymbol{\phi}+\delta\boldsymbol{\phi})\approx\mathrm{Exp}(\boldsymbol{\phi})\cdot\mathrm{Exp}(\mathbf{J}_r(\boldsymbol{\phi})\,\delta\boldsymbol{\phi})$，使测量值与噪声解耦，进而实现协方差的线性传播——这正是"预积分"之所以可行的数学核心。

##### §4.8 Convention差异大全 ⭐⭐

| 维度 | Sophus | manif | GTSAM | Ceres |
|:----|:-------|:------|:------|:------|
| 默认扰动方向 | 左（典型用法） | **右** | **右** | 用户自定义 |
| Jacobian定义域 | 内部参数向量（四元数分量） | **切空间扰动** | **切空间扰动** | ambient参数 |
| SE(3)切向量序 | $[\boldsymbol{\rho},\boldsymbol{\phi}]$（平移在前） | $[\boldsymbol{\rho},\boldsymbol{\phi}]$ | $[\boldsymbol{\phi},\boldsymbol{\rho}]$（**旋转在前**） | 用户定义 |
| J_l/J_r方法 | `leftJacobian` 静态方法（无rightJacobian） | `w.ljac()`、`w.rjac()` 及逆 | `Expmap`/`Logmap` 的OptionalJacobian | 通过Jet自动传播 |
| Ceres集成 | 需手写Manifold wrapper | 原生提供CeresLocalParameterization | 自有因子图框架 | 原生Manifold接口 |

**文献convention对照**：Barfoot书主用左扰动、Solà论文主用右扰动、Eade笔记用左扰动、Forster预积分用右扰动、Lynch-Park《Modern Robotics》用screw body/spatial Jacobian。阅读论文时**第一步永远是确认convention**。

---

#### 2. 进阶章节（档位4选学） ⭐⭐⭐⭐

| 主题 | 内容概要 | 推荐资源 |
|:----|:--------|:--------|
| BCH完整级数展开 | 无穷项的递归结构 | Hall《Lie Groups》Ch.5 |
| BCH收敛性证明 | 需要泛函分析/Banach代数工具 | Hall Ch.5、Chirikjian Vol.2 Ch.5–7 |
| Magnus展开 | 时变系统 $\dot{X}=A(t)X$ 的解写成单个exp | 与BCH互补，控制理论中常见 |
| 高阶Jacobian（Hessian on Lie groups） | 二阶导数在Gauss-Newton加速中的角色 | 前沿研究，尚无标准教材 |
| Retraction-induced Jacobian | 不同retraction对优化收敛速度的影响 | 与专题2联系，Absil书 |
| Zassenhaus公式 | BCH的"对偶"：$e^{A+B}=e^A e^B e^{C_2} e^{C_3}\cdots$ | Hall Ch.5 |
| Dynkin公式 | BCH的另一封闭表达 | 纯数学兴趣 |
| dLog（log映射的Jacobian） | $d\,\mathrm{Log}(\mathbf{X})$ 与 $\mathbf{J}_l^{-1}$ 的关系 | Eade exp_diff.pdf |

---

#### 3. 核心教材深度对照 ⭐

| 教材 | 核心章节 | 特色 | 免费获取 |
|:----|:--------|:----|:--------|
| **Barfoot《State Estimation for Robotics》2nd ed. (2024)** | Ch.8（§8.1.5 BCH, §8.1.9 Jacobians, §8.2.3–4 SO(3)/SE(3)线性化）; Ch.9 §9.4 预积分; App.B SE(3) Jacobian恒等式 | 工程视角最完整，所有推导配协方差传播；**2nd ed.新增**等变性§8.4、预积分§9.4 | asrl.utias.utoronto.ca/~tdb |
| **Solà et al.《A micro Lie theory》(arXiv 1812.01537, v9)** | §II-G Jacobian定义; §III 全部基本Jacobian块; App. B SO(3)闭式; App. D SE(3)闭式含Q矩阵 | **机器人社区事实标准**，17页覆盖全部内容，7个Boxed Example; 与manif库一一对应 | arxiv.org/abs/1812.01537 |
| **Eade笔记** | lie.pdf（V矩阵、Adjoint、Jacobian）; exp_diff.pdf（exp映射导数=左Jacobian的完整推导） | 极致紧凑的速查手册，含SO(3)/SE(3)/Sim(3)全部闭式 | ethaneade.com/lie.pdf, ethaneade.com/exp_diff.pdf |
| **Chirikjian《Stochastic Models》Vol.2** | Ch.5–7 Jacobian与BCH的严格数学处理 | 数学最严谨，难度最高 | 参考用 |
| **Lynch & Park《Modern Robotics》** | Ch.3末尾 screw theory body/spatial Jacobian | 与控制社区的旋量理论衔接 | modernrobotics.org |
| **Hall《Lie Groups, Lie Algebras》** | Ch.5 BCH公式证明 | 仅档位4 | 大学图书馆 |

---

#### 4. 最关键的论文 ⭐

**必读**（档位3）：
- **Solà et al. (2018/2021)** — 全部Jacobian的统一框架与闭式汇总
- **Forster et al. TRO 2017** "On-Manifold Preintegration" — BCH/右Jacobian在VIO预积分中的完整实战；Appendix含全部解析Jacobian
- **Barfoot & Furgale TRO 2014** "Associating Uncertainty with 3D Poses" — Jacobian在不确定性传播（compounding/fusing/projection）中的系统应用
- **Eade "Derivative of the exponential map" (2018)** — exp_diff.pdf，$D_\text{exp}$的完整推导

**推荐**（档位3+/档位4）：
- **Bloesch et al. (2016)** "A Primer on the Differential Calculus of 3D Orientations" — 表示无关的抽象框架，绿色=通用恒等式/红色=四元数实现，含完整IMU建模示例
- **Carpentier & Mansard RSS 2018** "Analytical Derivatives of Rigid Body Dynamics Algorithms" — Pinocchio的理论基础，RNEA/ABA解析导数O(n)复杂度

---

#### 5. 与C++库的具体映射 ⭐⭐

| 库 | Jacobian获取方式 | 典型工作流 |
|:--|:----------------|:----------|
| **manif** | 所有操作（compose/inverse/exp/log/act）均支持传入Jacobian输出指针；`w.rjac()`/`w.ljac()`/`w.rjacinv()`/`w.ljacinv()` | 与Solà论文一一对应，右扰动convention；支持Ceres的Jet类型 |
| **Sophus** | `SO3::leftJacobian(omega)` 和 `leftJacobianInverse(omega)` 静态方法（无rightJacobian，需取负）；`Dx_exp_x` 是对内部参数的导数，**不是**切空间Jacobian | SE(3)的exp/log内部使用leftJacobian；注意Sophus2（farm-ng-core）已是完全重写，API不兼容 |
| **GTSAM** | `Compose(p,q,Hp,Hq)` / `Expmap(v,Hv)` / `Logmap(p,Hp)` 均接受OptionalJacobian；Expression框架实现反向模式块AD | 右扰动convention；**Pose3切向量序为旋转在前** $[\boldsymbol{\phi},\boldsymbol{\rho}]$，与manif/Sophus相反 |
| **Ceres** | Jet类型前向AD自动传播Jacobian；`Manifold::PlusJacobian` 提供 $\partial(x\oplus\delta)/\partial\delta$；内置`EigenQuaternionManifold` | 通常不需手动推导；内部链式法则：$J_\text{total}=J_\text{cost}\times J_\text{plus}$ |
| **Pinocchio** | `computeJointJacobians`/`computeFrameJacobian`（运动学）；`computeRNEADerivatives`/`computeABADerivatives`（动力学） | LOCAL/WORLD/LOCAL_WORLD_ALIGNED三种参考系；几何Jacobian（不是解析Jacobian） |

---

#### 6. 在SLAM/VIO/BA中的直接应用 ⭐⭐

- **BA重投影误差Jacobian**：$\partial e_\text{reproj}/\partial\delta\boldsymbol{\xi}$ 是 $2\times 6$ 矩阵，由 $\mathbf{T}\cdot\mathbf{p}$ 对扰动的Jacobian经相机投影链式法则得到（高翔《十四讲》第7讲）
- **PGO的between factor Jacobian**：$\partial\mathrm{Log}(\mathbf{T}_i^{-1}\mathbf{T}_j\cdot\mathbf{T}_{ij}^{-1})/\partial\delta\boldsymbol{\xi}_i$ 和 $\partial/\partial\delta\boldsymbol{\xi}_j$，是 $6\times6$ 矩阵，依赖Adjoint和 $\mathbf{J}_r^{-1}$
- **IMU预积分协方差传播**：Forster论文的核心——通过BCH一阶近似把噪声从exp内部提取出来，用 $\mathbf{J}_r$ 做线性化传播 $15\times 15$ 协方差矩阵
- **IEKF的更新步Jacobian**：InEKF（不变扩展卡尔曼滤波器）的优越性正是因为其Jacobian对状态无关——利用了Adjoint的群结构
- **VIO Marginalization**：Schur消元时需要所有因子的Jacobian，FEJ（First Estimate Jacobian）策略要求固定线性化点处的Jacobian以保持一致性

---

#### 7. 关键定理清单 ⭐⭐

| # | 定理/恒等式 | 档位 |
|:--|:----------|:-----|
| 1 | $\mathbf{J}_l(\boldsymbol{\phi})=\sum_{k=0}^\infty \mathrm{ad}(\boldsymbol{\phi})^k/(k+1)!=\int_0^1 \exp(t\,\mathrm{ad}(\boldsymbol{\phi}))\,dt$ | 3 |
| 2 | $\mathbf{J}_r(\boldsymbol{\phi})=\mathbf{J}_l(-\boldsymbol{\phi})$ | 3 |
| 3 | $\mathbf{J}_l(\boldsymbol{\phi})=\mathrm{Ad}(\mathrm{Exp}(\boldsymbol{\phi}))\cdot\mathbf{J}_r(\boldsymbol{\phi})$ | 3 |
| 4 | SO(3)闭式 $\mathbf{J}_l$, $\mathbf{J}_r$, $\mathbf{J}_l^{-1}$, $\mathbf{J}_r^{-1}$ | 3 |
| 5 | SE(3)上的Q矩阵与 $6\times6$ 块Jacobian结构 | 3 |
| 6 | BCH前三阶：$\mathbf{X}+\mathbf{Y}+\tfrac{1}{2}[\mathbf{X},\mathbf{Y}]+\cdots$ | 3 |
| 7 | BCH一阶Jacobian形式：$\mathrm{Log}(\mathrm{Exp}(\boldsymbol{\phi})\mathrm{Exp}(\delta\boldsymbol{\phi}))\approx\boldsymbol{\phi}+\mathbf{J}_r^{-1}\delta\boldsymbol{\phi}$ | 3 |
| 8 | BCH完整级数的存在性与收敛性（紧致李群） | 4 |

---

#### 8. 学习资源汇总 ⭐

##### 英文免费资源

- Barfoot书PDF：asrl.utias.utoronto.ca/~tdb（持续更新的工作草稿）
- Solà论文：arxiv.org/abs/1812.01537（v9, 2021）
- Eade笔记：ethaneade.com/lie.pdf + ethaneade.com/exp_diff.pdf
- manif文档与cheat sheet：github.com/artivis/manif + 仓库内paper/Lie_theory_cheat_sheet.pdf
- Bloesch Primer：arxiv.org/abs/1606.05285

##### 视频

- **Joan Solà** "Lie theory for the roboticist" YouTube讲座（约1小时，含manif演示）：youtube.com/watch?v=nHOcoIyJj2o
- **高翔** 深蓝学院SLAM课程第3讲（李群与李代数+批量估计，约70min）：bilibili.com/video/BV1nh411B7mv
- **高翔** 《十四讲》完整视频（第二版）：bilibili.com/video/BV1Z5411t7oB
- bilibili "一起读书"系列第4讲（手写推导，2万播放）：bilibili.com/video/BV1dD4y177Ay

##### 中文文本资源

- **高翔《视觉SLAM十四讲》第4讲**（§4.3 BCH与扰动模型是核心）+ slambook2 GitHub代码
- **weihao-ysgs 博客园**：IMU预积分Jacobian的最完整中文推导（cnblogs.com/weihao-ysgs/p/IMU-Pre-Integration.html）
- **PetWorm/IMU-Preintegration-Propogation-Doc** GitHub：基于泡泡机器人邱笑晨博士文档的修订版
- **贺一家 CSDN博客**（heyijia0327）：Lie group in CV、marginalization、FEJ系列
- **知乎高赞**：搜索"SLAM 李群 雅可比"，推荐 zhuanlan.zhihu.com/p/658948878（基于Solà论文的详细解读）

---

#### 9. 自测题目（5题） ⭐⭐

1. **手推SO(3)上 $\mathbf{J}_l$ 和 $\mathbf{J}_r$ 的闭式**：从 $\mathrm{ad}$ 算子级数出发，利用 $[\boldsymbol{\phi}]_\times^3=-\theta^2[\boldsymbol{\phi}]_\times$ 封闭求和（档位3核心）
2. **证明 $\mathbf{J}_r(\boldsymbol{\phi})=\mathbf{J}_l(-\boldsymbol{\phi})$**：从积分定义出发做变量替换（档位3）
3. **推导BA重投影误差对SE(3)姿态的Jacobian**：用右扰动模型，对 $\pi(\mathbf{T}\cdot\mathbf{P}_w)$ 求 $\partial e/\partial\delta\boldsymbol{\xi}$，得到 $2\times 6$ 矩阵（档位3实战）
4. **用BCH公式推导** $\mathrm{Log}(\mathrm{Exp}(\delta\boldsymbol{\phi})\cdot\mathrm{Exp}(\boldsymbol{\phi}))$ **在小 $\delta\boldsymbol{\phi}$ 下的一阶表达**（档位3）
5. **实现SE(3) between factor的Jacobian**：$r=\mathrm{Log}(\mathbf{T}_{ij}^{-1}\cdot\mathbf{T}_i^{-1}\cdot\mathbf{T}_j)$，分别对 $\delta\boldsymbol{\xi}_i$ 和 $\delta\boldsymbol{\xi}_j$ 求导（档位3工程）

---

#### 10. 常见陷阱 ⭐⭐

- **左右扰动混用**——最高频错误，尤其是从论文A搬公式到用论文B convention的代码时
- **分子布局 vs 分母布局混淆**——Solà/manif用分子布局，部分控制教材用分母布局，Jacobian会转置
- **$\theta\to 0$ 忘记Taylor展开**——$\sin\theta/\theta$ 在 $\theta=0$ 附近需要切换到 $1-\theta^2/6+\cdots$，否则出现NaN
- **误以为SO(3)的简化适用于SE(3)**——SO(3)上 $\mathrm{Ad}(\mathbf{R})=\mathbf{R}$ 是巧合；SE(3)的Adjoint含 $[\mathbf{t}]_\times\mathbf{R}$ 耦合项
- **直接数值反转 $\mathbf{J}_l$ 而不用闭式 $\mathbf{J}_l^{-1}$**——在 $\theta$ 接近 $2\pi$ 时条件数极差
- **Forster与Barfoot的convention不同**——Forster用右扰动+平移在后，Barfoot主用左扰动+不同的Q矩阵定义
- **Sophus版本陷阱**——Sophus 1.x（C++14, strasdat/Sophus）与Sophus2（C++20, farm-ng-core）API完全不兼容；GTSAM Pose3的切向量序是旋转在前，与manif/Sophus相反

---

### 12. 从扰动定义到 Jacobian 的完整推导链 ⭐⭐

前面给出了 Jacobian 与 BCH 的速查框架。

这一节按推导顺序把公式连起来。

目标不是背公式，而是让你在看到任意李群残差时都能自己推导。

核心流程只有五步：

```text
明确状态属于哪个李群
明确扰动乘在左边还是右边
明确切向量排序
施加小扰动并保留一阶项
把小扰动前面的线性系数读成 Jacobian
```

#### 12.1 前置自测 ⭐

进入推导前，先回答：

1. $\operatorname{Exp}(\phi+\delta)$ 能否直接写成 $\operatorname{Exp}(\phi)\operatorname{Exp}(\delta)$？
2. $\operatorname{Exp}(\delta)\operatorname{Exp}(\phi)$ 与 $\operatorname{Exp}(\phi)\operatorname{Exp}(\delta)$ 的一阶差别在哪里？
3. 左扰动 $X^+=\operatorname{Exp}(\delta)X$ 与右扰动 $X^+=X\operatorname{Exp}(\delta)$ 的坐标系含义是什么？
4. $SO(3)$ 中 $\operatorname{Ad}_R=R$ 为什么是简化，而不是一般规律？
5. $SE(3)$ 的切向量采用 $[\rho,\phi]$ 还是 $[\phi,\rho]$ 会改变矩阵块位置吗？

如果第 1 个问题回答为"可以"，说明还没有真正理解 BCH。

---

### 13. 为什么两个小旋转不能简单相加 ⭐⭐

#### 13.1 欧氏直觉的诱惑 ⭐

在 $\mathbb{R}^n$ 中：

$$
(x+a)+b=x+(a+b)
$$

因此两个增量合成就是相加。

很多人自然希望在李群上也有：

$$
\operatorname{Exp}(a)\operatorname{Exp}(b)
=
\operatorname{Exp}(a+b)
$$

这只在 Abelian 群中成立。

旋转群不是 Abelian 群。BCH 公式的额外修正项可以类比合力的向量加法。在一维情况下，两个力 $f_1 + f_2$ 就是简单相加。但在三维空间中，两个力矩的复合不仅是向量加法——如果力矩的方向不同，它们的复合效果还取决于作用顺序和耦合效应（想象先绕 $x$ 轴再绕 $y$ 轴转和反过来转的结果不同）。BCH 公式中的 $\frac{1}{2}[A,B]$ 项正是这种"顺序依赖"的数学度量。在信号处理中也有类似现象：两个线性时不变滤波器的串联与顺序无关（因为卷积交换），但两个非线性滤波器的串联结果取决于顺序——BCH 就是把这种非交换性精确量化。

一般有：

$$
R_x(\alpha)R_y(\beta)\neq R_y(\beta)R_x(\alpha)
$$

因此指数里的增量合成必然出现额外项。

#### 13.2 用矩阵指数看问题 ⭐⭐

矩阵指数展开：

$$
e^Ae^B
=
\left(I+A+\frac{1}{2}A^2+\cdots\right)
\left(I+B+\frac{1}{2}B^2+\cdots\right)
$$

保留到二阶：

$$
e^Ae^B
=
I+A+B+\frac{1}{2}A^2+AB+\frac{1}{2}B^2+O(3)
$$

如果它等于 $e^C$，且假设：

$$
C=A+B+C_2+O(3)
$$

其中 $C_2$ 是二阶项，则：

$$
e^C
=
I+C+\frac{1}{2}C^2+O(3)
$$

代入：

$$
e^C
=
I+A+B+C_2+\frac{1}{2}(A+B)^2+O(3)
$$

展开平方：

$$
\frac{1}{2}(A+B)^2
=
\frac{1}{2}A^2+\frac{1}{2}AB+\frac{1}{2}BA+\frac{1}{2}B^2
$$

与 $e^Ae^B$ 的二阶项比较：

$$
C_2+\frac{1}{2}AB+\frac{1}{2}BA=AB
$$

所以：

$$
C_2=\frac{1}{2}(AB-BA)=\frac{1}{2}[A,B]
$$

因此：

$$
\log(e^Ae^B)
=
A+B+\frac{1}{2}[A,B]+O(3)
$$

这就是 BCH 公式最重要的第一层。

> **本质洞察**：BCH 公式的额外项不是数学装饰，而是非交换性的度量。若 $[A,B]=0$，二阶修正消失；若 $[A,B]\neq0$，简单相加就一定遗漏旋转顺序效应。

#### 13.3 三阶项为什么长得复杂 ⭐⭐⭐

完整 BCH 的前三阶为：

$$
\log(e^Ae^B)
=
A+B+\frac{1}{2}[A,B]
+\frac{1}{12}[A,[A,B]]
+\frac{1}{12}[B,[B,A]]
+O(4)
$$

也常写成：

$$
A+B+\frac{1}{2}[A,B]
+\frac{1}{12}\big([A,[A,B]]-[B,[A,B]]\big)
+O(4)
$$

因为：

$$
[B,[B,A]]=-[B,[A,B]]
$$

这些嵌套括号表示：

```text
不仅 A 与 B 的顺序重要，
A 与 AB-BA 的顺序也重要，
B 与 AB-BA 的顺序也重要。
```

在机器人中，通常不需要背完整无穷级数。

但必须知道保留到哪一阶对应什么近似精度。

> 💡 **概念误区**：认为"BCH 高阶项在工程中永远可以忽略"
>
> **新手想法**："反正我们只用一阶近似，三阶及以上的项没有实际意义。"
>
> **实际上**：一阶近似 $\log(\exp(A)\exp(B)) \approx A + B$ 只在 $A$ 或 $B$ 足够小时成立。当两个旋转都不小（如 IMU 长时间积分或快速步态切换）时，忽略 $\frac{1}{2}[A,B]$ 项会引入系统性偏差。在预积分中，如果两个关键帧间隔过长导致旋转增量很大，一阶 BCH 近似可能不够——Forster 2017 论文的实现实际上在关键帧间隔时重置预积分，部分原因就是避免 BCH 高阶误差的累积。

---

### 14. 左 Jacobian 与右 Jacobian 的来源 ⭐⭐

#### 14.1 右 Jacobian 的定义 ⭐⭐

考虑指数映射的扰动：

$$
\operatorname{Exp}(\phi+\delta)
$$

我们希望把它写成：

$$
\operatorname{Exp}(\phi)\operatorname{Exp}(\epsilon)
$$

其中 $\epsilon$ 是与 $\delta$ 一阶相关的小量。

定义右 Jacobian：

$$
\epsilon=J_r(\phi)\delta+O(\|\delta\|^2)
$$

也就是：

$$
\operatorname{Exp}(\phi+\delta)
\approx
\operatorname{Exp}(\phi)\operatorname{Exp}(J_r(\phi)\delta)
$$

它回答的问题是：

```text
指数坐标里的小变化 delta，
等价于在当前 Exp(phi) 右侧乘上多大的小扰动？
```

#### 14.2 左 Jacobian 的定义 ⭐⭐

左 Jacobian 类似：

$$
\operatorname{Exp}(\phi+\delta)
\approx
\operatorname{Exp}(J_l(\phi)\delta)\operatorname{Exp}(\phi)
$$

它回答：

```text
指数坐标里的小变化 delta，
等价于在当前 Exp(phi) 左侧乘上多大的小扰动？
```

两者差异来自李群的非交换性。

左/右 Jacobian 的角色可以类比广义相对论中的平行移动（parallel transport）。在平坦的欧氏空间中，一个向量可以直接从一个点搬到任何其他点而不改变——这就是为什么 $\mathbb{R}^n$ 中 $J_l = J_r = I$。但在弯曲的流形上，把切向量从一个点搬到另一个点时，向量会被曲率"扭转"。$J_l$ 和 $J_r$ 正是度量这种扭转的矩阵：它们把单位元处的小扰动 $\delta$ 搬运到 $\text{Exp}(\phi)$ 附近时告诉你搬运过程中发生了多少形变。在 SO(3) 中这种形变由 $\sin\theta/\theta$ 等系数控制，物理上对应旋转群的曲率。

#### 14.3 用 BCH 推导右 Jacobian 的级数 ⭐⭐⭐

设：

$$
\operatorname{Exp}(\phi+\delta)
=
\operatorname{Exp}(\phi)\operatorname{Exp}(\epsilon)
$$

两边取 log：

$$
\phi+\delta
=
\log(\operatorname{Exp}(\phi)\operatorname{Exp}(\epsilon))
$$

对右侧用 BCH，并只保留关于 $\epsilon$ 的一阶项：

$$
\phi+\delta
=
\phi+\epsilon
+\frac{1}{2}[\phi,\epsilon]
+\frac{1}{12}[\phi,[\phi,\epsilon]]
-\frac{1}{720}[\phi,[\phi,[\phi,[\phi,\epsilon]]]]
+\cdots
$$

因此：

$$
\delta
=
J_r^{-1}(\phi)\epsilon
$$

其中：

$$
J_r^{-1}(\phi)
=
I+\frac{1}{2}\operatorname{ad}_\phi
+\frac{1}{12}\operatorname{ad}_\phi^2
-\frac{1}{720}\operatorname{ad}_\phi^4+\cdots
$$

所以：

$$
\epsilon=J_r(\phi)\delta
$$

#### 14.4 积分表达式 ⭐⭐⭐

更紧凑的结果是：

$$
J_r(\phi)
=
\int_0^1 \exp(-s\,\operatorname{ad}_\phi)\,ds
$$

展开指数：

$$
\exp(-s\,\operatorname{ad}_\phi)
=
\sum_{k=0}^{\infty}
\frac{(-s)^k}{k!}\operatorname{ad}_\phi^k
$$

对 $s$ 从 0 到 1 积分：

$$
J_r(\phi)
=
\sum_{k=0}^{\infty}
\frac{(-1)^k}{(k+1)!}
\operatorname{ad}_\phi^k
$$

同理：

$$
J_l(\phi)
=
\int_0^1 \exp(s\,\operatorname{ad}_\phi)\,ds
=
\sum_{k=0}^{\infty}
\frac{1}{(k+1)!}
\operatorname{ad}_\phi^k
$$

因此：

$$
J_r(\phi)=J_l(-\phi)
$$

#### 14.5 Adjoint 关系 ⭐⭐⭐

还有一个重要关系：

$$
J_l(\phi)=\operatorname{Ad}_{\operatorname{Exp}(\phi)}J_r(\phi)
$$

对矩阵李群：

$$
\operatorname{Ad}_{\operatorname{Exp}(\phi)}
=
\exp(\operatorname{ad}_\phi)
$$

因此：

$$
\operatorname{Ad}_{\operatorname{Exp}(\phi)}J_r(\phi)
=
\exp(\operatorname{ad}_\phi)
\int_0^1 \exp(-s\operatorname{ad}_\phi)\,ds
$$

令 $u=1-s$：

$$
=
\int_0^1 \exp((1-s)\operatorname{ad}_\phi)\,ds
=
\int_0^1 \exp(u\operatorname{ad}_\phi)\,du
=
J_l(\phi)
$$

这说明左/右 Jacobian 不是两套无关公式，而是同一微分对象在不同平凡化下的表达。

**如果没有左/右 Jacobian 会怎样？** 假设你在 VIO 系统中传播 IMU 旋转噪声，但忽略了 Jacobian，直接把噪声 $\delta\phi$ 加到旋转向量上：$\phi' = \phi + \delta\phi$。在低速运动（$\|\phi\| \ll 1$）时，$J_r \approx I$，误差不明显，系统看起来"工作正常"。但当无人机高速旋转（$\|\phi\| > 1$ rad）时，$J_r$ 与 $I$ 的差异可达 20-30%，噪声协方差会被系统性低估。后果是：滤波器的一致性检验（NEES）会超出卡方分布置信区间，表现为过度自信——估计值看起来精度很高但实际误差很大。更严重的是，VIO 的 bias 估计会因为不一致的协方差传播而漂移，最终导致定位发散。这不是理论风险，而是在实际飞行测试中反复出现过的问题。

---

### 15. SO(3) Jacobian 的闭式与小角度展开 ⭐⭐

#### 15.1 从级数到闭式 ⭐⭐

在 $SO(3)$ 中：

$$
\operatorname{ad}_\phi=[\phi]_\times
$$

记：

$$
\theta=\|\phi\|
$$

叉乘矩阵满足：

$$
[\phi]_\times^3=-\theta^2[\phi]_\times
$$

因此任何高阶幂都能化成：

$$
I,\quad [\phi]_\times,\quad [\phi]_\times^2
$$

的线性组合。

右 Jacobian 闭式为：

$$
J_r(\phi)
=
I
-\frac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2
$$

左 Jacobian 闭式为：

$$
J_l(\phi)
=
I
+\frac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2
$$

这与前面的关系 $J_r(\phi)=J_l(-\phi)$ 一致。

#### 15.2 用单位轴写法核对 ⭐⭐

令：

$$
\phi=\theta a,\qquad \|a\|=1
$$

则：

$$
[\phi]_\times=\theta[a]_\times
$$

代入左 Jacobian：

$$
J_l(\phi)
=
I
+\frac{1-\cos\theta}{\theta}[a]_\times
+\frac{\theta-\sin\theta}{\theta}[a]_\times^2
$$

利用：

$$
[a]_\times^2=aa^\top-I
$$

可写成：

$$
J_l(\phi)
=
\frac{\sin\theta}{\theta}I
+\left(1-\frac{\sin\theta}{\theta}\right)aa^\top
+\frac{1-\cos\theta}{\theta}[a]_\times
$$

这与常见教材中的轴角形式一致。

#### 15.3 逆 Jacobian ⭐⭐⭐

左 Jacobian 的逆为：

$$
J_l^{-1}(\phi)
=
I-\frac{1}{2}[\phi]_\times
+\left(
\frac{1}{\theta^2}
-\frac{1+\cos\theta}{2\theta\sin\theta}
\right)[\phi]_\times^2
$$

右 Jacobian 的逆为：

$$
J_r^{-1}(\phi)=J_l^{-1}(-\phi)
$$

即：

$$
J_r^{-1}(\phi)
=
I+\frac{1}{2}[\phi]_\times
+\left(
\frac{1}{\theta^2}
-\frac{1+\cos\theta}{2\theta\sin\theta}
\right)[\phi]_\times^2
$$

#### 15.4 小角度展开 ⭐⭐

当 $\theta$ 很小时，直接计算会遇到 $0/0$。

应使用 Taylor：

$$
\frac{1-\cos\theta}{\theta^2}
=
\frac{1}{2}-\frac{\theta^2}{24}+\frac{\theta^4}{720}+\cdots
$$

$$
\frac{\theta-\sin\theta}{\theta^3}
=
\frac{1}{6}-\frac{\theta^2}{120}+\frac{\theta^4}{5040}+\cdots
$$

因此：

$$
J_l(\phi)
=
I+\frac{1}{2}[\phi]_\times+\frac{1}{6}[\phi]_\times^2+O(\theta^3)
$$

$$
J_r(\phi)
=
I-\frac{1}{2}[\phi]_\times+\frac{1}{6}[\phi]_\times^2+O(\theta^3)
$$

⚠️ **陷阱：小角度分支不是性能优化，而是正确性要求**

如果 $\theta$ 接近 0 时仍直接计算闭式系数，浮点消减会让 Jacobian 出现 NaN 或精度损失。

这会在 IMU 预积分和高频姿态估计中频繁触发。

---

### 16. SE(3) Jacobian 的块结构 ⭐⭐⭐

#### 16.1 切向量排序必须先声明 ⭐⭐

本节采用：

$$
\xi=
\begin{bmatrix}
\rho\\
\phi
\end{bmatrix}
$$

其中 $\rho$ 是平移扰动，$\phi$ 是旋转扰动。

这是 Solà/manif/Sophus 常见排序。

GTSAM 的 `Pose3` 常使用旋转在前：

$$
\xi_{\text{gtsam}}=
\begin{bmatrix}
\phi\\
\rho
\end{bmatrix}
$$

两者通过置换矩阵互换。

如果不声明排序，SE(3) Jacobian 公式没有唯一含义。

#### 16.2 SE(3) 指数的回顾 ⭐⭐

对：

$$
\xi^\wedge=
\begin{bmatrix}
\phi^\wedge & \rho\\
0 & 0
\end{bmatrix}
$$

指数映射为：

$$
\operatorname{Exp}(\xi)=
\begin{bmatrix}
R & t\\
0 & 1
\end{bmatrix}
$$

其中：

$$
R=\operatorname{Exp}(\phi)
$$

$$
t=V(\phi)\rho
$$

且：

$$
V(\phi)=J_l^{SO(3)}(\phi)
$$

在平移在前约定下，SE(3) 左 Jacobian 有块结构：

$$
J_l^{SE(3)}(\rho,\phi)
=
\begin{bmatrix}
J_l^{SO(3)}(\phi) & Q_l(\rho,\phi)\\
0 & J_l^{SO(3)}(\phi)
\end{bmatrix}
$$

右 Jacobian 为：

$$
J_r^{SE(3)}(\xi)=J_l^{SE(3)}(-\xi)
$$

#### 16.3 Q 矩阵的含义 ⭐⭐⭐

$Q_l$ 描述：

```text
旋转扰动变化时，
平移部分 t=V(phi)rho 如何随之变化。
```

如果只看 $SO(3)$，旋转和平移不存在耦合。

但在 $SE(3)$ 中，平移向量的表达与旋转密切相关。

因此 6x6 Jacobian 不能简单写成两个 $SO(3)$ Jacobian 的对角拼接。

> ⚠️ **思维陷阱**：把 SE(3) Jacobian 写成 SO(3) Jacobian 的对角扩展
>
> **错误做法**：$J_l^{SE(3)} = \text{diag}(J_l^{SO(3)}, J_l^{SO(3)})$，认为旋转和平移的 Jacobian 互不影响。
>
> **现象**：位姿图优化的 between factor 在纯旋转或纯平移边上看起来正确，但在包含旋转和平移的一般边上出现系统性偏差，优化收敛变慢或结果不精确。
>
> **根本原因**：SE(3) 不是 SO(3) 和 $\mathbb{R}^3$ 的直积。旋转扰动 $\delta\phi$ 的变化会通过 $Q_l$ 矩阵影响平移分量 $t = V(\phi)\rho$，而忽略 $Q_l$ 就是假设旋转和平移完全解耦——这与半直积的代数结构矛盾。
>
> **正确做法**：使用完整的 6x6 块结构，包含非对角的 $Q_l$ 块。工程中优先使用 manif 库的 `w.ljac()` 获取数值结果，避免手写。

在工程中，$Q_l$ 公式冗长，手写容易错。

更好的做法是：

1. 明确切向量排序。
2. 使用经过测试的库实现。
3. 用有限差分单元测试验证自写公式。

#### 16.4 Adjoint 的块结构 ⭐⭐

对：

$$
T=
\begin{bmatrix}
R & t\\
0 & 1
\end{bmatrix}
$$

采用平移在前 $[\rho,\phi]$ 排序：

$$
\operatorname{Ad}_T=
\begin{bmatrix}
R & [t]_\times R\\
0 & R
\end{bmatrix}
$$

采用旋转在前 $[\phi,\rho]$ 排序：

$$
\operatorname{Ad}_T=
\begin{bmatrix}
R & 0\\
[t]_\times R & R
\end{bmatrix}
$$

这两个公式都正确。

它们只是坐标排序不同。

⚠️ **陷阱：把两个正确公式混在一起会得到错误代码**

很多 SE(3) bug 不是公式本身错，而是从一本书抄了平移在前的 Adjoint，又在 GTSAM 的旋转在前向量上使用。

这种错误不会编译失败，但会让协方差传播、between factor 和预积分 Jacobian 全部偏掉。

---

### 17. 常用残差 Jacobian 的推导模板 ⭐⭐

#### 17.1 推导模板 ⭐⭐

每次推导残差 Jacobian，都按以下模板：

```text
Step 1: 写出残差 r(X)
Step 2: 写出扰动模型 X(delta)
Step 3: 代入 r(X(delta))
Step 4: 展开到一阶
Step 5: 把 delta 前的系数读成 J
```

这比直接背公式可靠。

#### 17.2 点变换残差 ⭐⭐

设：

$$
y=Rp+t
$$

采用左扰动：

$$
T^+=\operatorname{Exp}(\delta\xi)T
$$

其中：

$$
\delta\xi=
\begin{bmatrix}
\delta\rho\\
\delta\phi
\end{bmatrix}
$$

小扰动作用在点上：

$$
y^+
\approx
(I+\delta\phi^\wedge)(Rp+t)+\delta\rho
$$

因此：

$$
y^+
\approx
y+\delta\rho+\delta\phi^\wedge y
$$

利用：

$$
\delta\phi^\wedge y=-y^\wedge\delta\phi
$$

得到：

$$
\frac{\partial y}{\partial\delta\xi}
=
\begin{bmatrix}
I & -[y]_\times
\end{bmatrix}
$$

这里的 $y=Rp+t$。

如果使用右扰动，结果会不同。

> ⚠️ **编程陷阱**：左扰动和右扰动的 Jacobian 差一个 Adjoint，但新手常把两者的公式混用
>
> **错误做法**：从用左扰动的论文（如 Barfoot 书）抄了 $\frac{\partial y}{\partial\delta\xi} = [I, -[y]_\times]$ 的结果，直接用在右扰动代码库（如 manif/GTSAM）中。
>
> **现象**：代码编译通过，数值差分验证时某些列的符号相反或大小不同。
>
> **根本原因**：左扰动对应 $T' = \text{Exp}(\delta)T$，右扰动对应 $T' = T\text{Exp}(\delta)$。两者的 Jacobian 相差 $\text{Ad}_T$ 或 $\text{Ad}_T^{-1}$，不是简单的转置或取负。
>
> **正确做法**：每次推导 Jacobian 时从第一步写清楚扰动方向，用数值差分验证每一列。

#### 17.3 between factor 的结构 ⭐⭐⭐

位姿图残差常写：

$$
r_{ij}
=
\operatorname{Log}(Z_{ij}^{-1}T_i^{-1}T_j)
$$

这里有三层非线性：

1. 逆映射 $T_i^{-1}$。
2. 群乘法 $T_i^{-1}T_j$。
3. 对数映射 $\operatorname{Log}$。

推导时不要试图一步完成。

应逐层施加扰动并使用：

$$
T\operatorname{Exp}(\delta)
=
\operatorname{Exp}(\operatorname{Ad}_T\delta)T
$$

以及：

$$
\operatorname{Log}(\operatorname{Exp}(\phi)\operatorname{Exp}(\delta))
\approx
\phi+J_r^{-1}(\phi)\delta
$$

最终 Jacobian 必然包含：

```text
Adjoint: 负责把扰动搬到同一侧；
J_r^{-1} 或 J_l^{-1}: 负责 Log 的微分。
```

#### 17.4 IMU 预积分中的 BCH ⭐⭐⭐

IMU 旋转递推中有：

$$
\Delta R_{k+1}
=
\Delta R_k\operatorname{Exp}((\omega_k-b_g)\Delta t)
$$

噪声进入指数内部：

$$
\operatorname{Exp}(\phi+\delta)
$$

为了传播协方差，需要把它变成当前名义旋转右侧的小扰动：

$$
\operatorname{Exp}(\phi+\delta)
\approx
\operatorname{Exp}(\phi)\operatorname{Exp}(J_r(\phi)\delta)
$$

这一步正是右 Jacobian 的定义。

没有 BCH，就无法系统地把噪声从指数内部搬出来。

> **本质洞察**：BCH 是预积分能够线性传播噪声的原因。预积分不是简单把 IMU 增量累加——它要在李群乘法中追踪噪声如何被旋转、搬运、耦合。BCH 把"指数内部的随机小量"变成"乘法右侧的切空间小扰动"，从而让协方差递推成为线性系统。没有 BCH 的一阶展开 $\text{Exp}(\phi + \delta) \approx \text{Exp}(\phi)\text{Exp}(J_r(\phi)\delta)$，噪声 $\delta$ 就无法从指数内部"提取"出来，预积分的整个理论框架就不成立。

---

### 17A. IMU 预积分完整 Jacobian 推导 ⭐⭐⭐

#### 动机：为什么需要预积分

在视觉惯性里程计（VIO）中，IMU 以 200-1000 Hz 的频率输出加速度和角速度，而视觉关键帧通常在 10-30 Hz。两个关键帧之间可能有几十甚至上百个 IMU 测量。如果在每次优化迭代中都重新积分这些 IMU 数据，计算代价极高，因为每改变一次关键帧位姿的估计，所有中间 IMU 积分都要重算。

Forster et al.（TRO 2017, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"）提出了一个关键洞察：IMU 增量可以预先积分成只依赖两个关键帧之间的**相对运动**，而不依赖关键帧在世界坐标系中的绝对位姿。当优化器改变关键帧位姿时，预积分量不需要重新计算——只需要通过简单的 bias 校正来更新。

这个"预先积分"能够成立的数学基础，正是本专题前面建立的 BCH 公式和右 Jacobian。

#### Forster 框架：预积分测量量

设两个关键帧时刻为 $i$ 和 $j$，之间有 $N$ 个 IMU 测量。IMU 的测量模型为：

$$
\omega_m = \omega + b_g + n_g, \qquad
a_m = R_{WB}^\top (a_W - g) + b_a + n_a
$$

其中 $\omega_m$ 和 $a_m$ 分别是陀螺仪和加速度计的测量值，$b_g, b_a$ 是缓慢变化的偏差，$n_g, n_a$ 是白噪声。

从连续时间运动方程离散化，可以定义三个**预积分测量量**：

$$
\Delta R_{ij} = \prod_{k=i}^{j-1} \operatorname{Exp}\big((\omega_k - b_g) \Delta t\big)
$$

$$
\Delta v_{ij} = \sum_{k=i}^{j-1} \Delta R_{ik} (a_k - b_a) \Delta t
$$

$$
\Delta p_{ij} = \sum_{k=i}^{j-1} \left[ \Delta v_{ik} \Delta t + \frac{1}{2} \Delta R_{ik} (a_k - b_a) \Delta t^2 \right]
$$

这三个量只依赖 IMU 测量值和偏差估计，不依赖世界坐标系中的姿态、速度和位置。这就是"预积分"名称的由来。

它们与关键帧状态之间的关系为：

$$
\Delta R_{ij} \approx R_i^\top R_j
$$

$$
\Delta v_{ij} \approx R_i^\top (v_j - v_i - g \Delta t_{ij})
$$

$$
\Delta p_{ij} \approx R_i^\top \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right)
$$

> **本质洞察**：预积分的本质是把 IMU 积分从"世界坐标系中的绝对积分"变成"第 $i$ 帧坐标系中的相对积分"。通过左乘 $R_i^\top$ 并减去重力效应，所有 IMU 增量都被表达在第 $i$ 帧的局部坐标系中。这样，当优化器改变 $R_i, v_i, p_i$ 的估计时，$\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij}$ 不受影响——它们已经"预先积分好了"。

#### 噪声传播与 $J_r$ 的角色

回顾专题 3 第 11.1 节中提到的 IMU 旋转噪声传播。含噪声的旋转递推为：

$$
\Delta \tilde{R}_{k+1} = \Delta \tilde{R}_k \operatorname{Exp}\big((\omega_k - b_g - n_g^k) \Delta t\big)
$$

将 $n_g^k$ 视为小量，利用 BCH 一阶近似（正是本专题 §4.7 和 §14.3 的核心结果）：

$$
\operatorname{Exp}\big((\omega_k - b_g) \Delta t - n_g^k \Delta t\big)
\approx
\operatorname{Exp}\big((\omega_k - b_g) \Delta t\big) \cdot \operatorname{Exp}\big(-J_r(\phi_k) n_g^k \Delta t\big)
$$

其中 $\phi_k = (\omega_k - b_g) \Delta t$ 是第 $k$ 步的名义旋转增量。

这一步就是右 Jacobian 在预积分中的直接应用——它把"指数内部的噪声"搬运到"乘法右侧的切空间小扰动"。如果忽略 $J_r$（令 $J_r = I$），相当于忽略了旋转对噪声的搬运效应。在低速运动下（$\|\phi_k\| \ll 1$），$J_r \approx I$，忽略可以接受；但在高动态场景下（无人机快速转弯、四足机器人步态切换），$\|\phi_k\|$ 可能达到 0.1-0.5 rad，$J_r$ 与 $I$ 的差异不可忽略。

#### $15\times15$ 协方差传播矩阵的逐步推导

定义误差状态向量为 $\delta = [\delta\phi, \delta v, \delta p, \delta b_g, \delta b_a]^\top \in \mathbb{R}^{15}$，其中：

- $\delta\phi \in \mathbb{R}^3$：旋转预积分误差（$\Delta \tilde{R} = \Delta R \operatorname{Exp}(\delta\phi)$）
- $\delta v \in \mathbb{R}^3$：速度预积分误差
- $\delta p \in \mathbb{R}^3$：位置预积分误差
- $\delta b_g \in \mathbb{R}^3$：陀螺仪偏差误差
- $\delta b_a \in \mathbb{R}^3$：加速度计偏差误差

误差状态的递推方程为：

$$
\delta_{k+1} = A_k \delta_k + B_k n_k
$$

其中 $n_k = [n_g^k, n_a^k, n_{bg}^k, n_{ba}^k]^\top$ 是噪声向量，$A_k$ 是 $15 \times 15$ 系统矩阵，$B_k$ 是噪声输入矩阵。

**$A_k$ 矩阵的逐块推导**：

| 块位置 | 表达式 | 来源 |
|--------|--------|------|
| $\frac{\partial \delta\phi_{k+1}}{\partial \delta\phi_k}$ | $\Delta R_k^\top = \operatorname{Exp}(-\phi_k)$ | 旋转误差的右乘传播 |
| $\frac{\partial \delta\phi_{k+1}}{\partial \delta b_g}$ | $-J_r(\phi_k) \Delta t$ | BCH 一阶近似，$J_r$ 的定义 |
| $\frac{\partial \delta v_{k+1}}{\partial \delta\phi_k}$ | $-\Delta R_k [a_k - b_a]_\times \Delta t$ | 旋转误差耦合到加速度方向 |
| $\frac{\partial \delta v_{k+1}}{\partial \delta v_k}$ | $I$ | 速度误差直接传播 |
| $\frac{\partial \delta v_{k+1}}{\partial \delta b_a}$ | $-\Delta R_k \Delta t$ | 加速度偏差通过旋转作用 |
| $\frac{\partial \delta p_{k+1}}{\partial \delta\phi_k}$ | $-\frac{1}{2} \Delta R_k [a_k - b_a]_\times \Delta t^2$ | 类似速度，多一个 $\frac{1}{2}\Delta t$ |
| $\frac{\partial \delta p_{k+1}}{\partial \delta v_k}$ | $I \cdot \Delta t$ | 位置是速度的积分 |
| $\frac{\partial \delta p_{k+1}}{\partial \delta p_k}$ | $I$ | 位置误差直接传播 |
| $\frac{\partial \delta p_{k+1}}{\partial \delta b_a}$ | $-\frac{1}{2} \Delta R_k \Delta t^2$ | 加速度偏差的位置效应 |
| $\frac{\partial \delta b_g}{\partial \delta b_g}$ | $I$ | 偏差为随机游走 |
| $\frac{\partial \delta b_a}{\partial \delta b_a}$ | $I$ | 偏差为随机游走 |

写成完整矩阵：

$$
A_k =
\begin{bmatrix}
\Delta R_k^\top & 0 & 0 & -J_r(\phi_k)\Delta t & 0\\
-\Delta R_k [a_k-b_a]_\times \Delta t & I & 0 & 0 & -\Delta R_k \Delta t\\
-\frac{1}{2}\Delta R_k [a_k-b_a]_\times \Delta t^2 & I\Delta t & I & 0 & -\frac{1}{2}\Delta R_k \Delta t^2\\
0 & 0 & 0 & I & 0\\
0 & 0 & 0 & 0 & I
\end{bmatrix}
$$

其中 $\Delta R_k = \operatorname{Exp}(\phi_k)$ 是第 $k$ 步的名义旋转增量。

协方差递推为：

$$
\Sigma_{k+1} = A_k \Sigma_k A_k^\top + B_k Q_d B_k^\top
$$

其中 $Q_d = \text{diag}(\sigma_g^2 I, \sigma_a^2 I, \sigma_{bg}^2 I, \sigma_{ba}^2 I) \cdot \Delta t$ 是离散化的噪声协方差。

#### 预积分测量对偏差的 Jacobian

当陀螺仪或加速度计偏差的估计在优化过程中被更新时（从 $\bar{b}$ 变为 $\bar{b} + \delta b$），预积分量需要相应修正。完全重新积分代价太高，Forster 使用**一阶偏差校正**：

$$
\Delta R_{ij}(\bar{b}_g + \delta b_g) \approx \Delta R_{ij}(\bar{b}_g) \cdot \operatorname{Exp}\left(\frac{\partial \Delta R_{ij}}{\partial b_g} \delta b_g\right)
$$

$$
\Delta v_{ij}(\bar{b} + \delta b) \approx \Delta v_{ij}(\bar{b}) + \frac{\partial \Delta v_{ij}}{\partial b_g} \delta b_g + \frac{\partial \Delta v_{ij}}{\partial b_a} \delta b_a
$$

$$
\Delta p_{ij}(\bar{b} + \delta b) \approx \Delta p_{ij}(\bar{b}) + \frac{\partial \Delta p_{ij}}{\partial b_g} \delta b_g + \frac{\partial \Delta p_{ij}}{\partial b_a} \delta b_a
$$

这些 Jacobian 在预积分过程中递推计算：

$$
\frac{\partial \Delta R_{k+1}}{\partial b_g}
=
-\Delta R_{k+1,k}^\top J_r(\phi_k) \Delta t \cdot I
+ \Delta R_{k+1,k}^\top \frac{\partial \Delta R_k}{\partial b_g}
$$

$$
\frac{\partial \Delta v_{k+1}}{\partial b_g}
=
\frac{\partial \Delta v_k}{\partial b_g}
- \Delta R_k [a_k - b_a]_\times \frac{\partial \Delta R_k}{\partial b_g} \Delta t
$$

$$
\frac{\partial \Delta v_{k+1}}{\partial b_a}
=
\frac{\partial \Delta v_k}{\partial b_a}
- \Delta R_k \Delta t
$$

注意 $J_r(\phi_k)$ 在 $\frac{\partial \Delta R}{\partial b_g}$ 的递推中直接出现——这正是 BCH 公式和右 Jacobian 在预积分中的具体体现。如果忽略 $J_r$，偏差校正的精度就会下降，特别是在高角速度运动中。

#### 数值算例：10 步 IMU 积分

为了建立具体直觉，考虑一个简化的 IMU 积分场景。

**设定**：
- 采样间隔 $\Delta t = 0.01$ s（100 Hz）
- 10 步积分，总时间 0.1 s
- 角速度（恒定）：$\omega_m = [0.0, 0.0, 1.0]^\top$ rad/s（绕 $z$ 轴旋转）
- 加速度（恒定）：$a_m = [0.0, 0.0, 9.81]^\top$ $\text{m/s}^2$（静止，只有重力）
- 陀螺仪偏差 $b_g = 0$，加速度计偏差 $b_a = 0$
- 重力 $g = [0, 0, -9.81]^\top$ $\text{m/s}^2$

**Step 1**：计算每步旋转增量：

$$
\phi_k = \omega_m \Delta t = [0, 0, 0.01]^\top \text{ rad}
$$

$$
\theta = \|\phi_k\| = 0.01 \text{ rad}
$$

**Step 2**：计算 $J_r(\phi_k)$（使用小角度近似，因为 $\theta = 0.01$ 很小）：

$$
J_r \approx I - \frac{1}{2}[\phi_k]_\times + \frac{1}{6}[\phi_k]_\times^2
\approx I - 0.005 \begin{bmatrix} 0 & -1 & 0\\ 1 & 0 & 0\\ 0 & 0 & 0 \end{bmatrix}
$$

$J_r$ 与 $I$ 的差异约为 $O(0.005)$，即 0.5%。

**Step 3**：10 步后的旋转预积分：

$$
\Delta R_{ij} = \prod_{k=0}^{9} \operatorname{Exp}(\phi_k) = \operatorname{Exp}([0, 0, 0.1]^\top)
$$

总旋转角度为 $0.1$ rad $\approx 5.7°$。

**Step 4**：10 步后的速度预积分（静止场景中 $R_i^\top(a - g) = 0$，因此）：

$$
\Delta v_{ij} = \sum_{k=0}^{9} \Delta R_{ik} (a_k - b_a) \Delta t
$$

在这个简化场景中，$a_m - b_a = [0, 0, 9.81]^\top$ 而 $R_i^\top g = [0, 0, 9.81]^\top$（假设初始姿态为单位阵），所以比力（specific force）在世界坐标中恰好被重力抵消，预积分速度接近零。这与直觉一致——物体静止不动。

**Step 5**：协方差传播。假设陀螺仪噪声密度 $\sigma_g = 1.7 \times 10^{-4}$ rad/s/$\sqrt{\text{Hz}}$，加速度计噪声密度 $\sigma_a = 2.0 \times 10^{-3}$ $\text{m/s}^2/\sqrt{\text{Hz}}$。10 步后旋转预积分的协方差约为：

$$
\Sigma_{\Delta\phi} \approx \sum_{k=0}^{9} (J_r(\phi_k))^2 \sigma_g^2 \Delta t \approx 10 \times 1 \times (1.7 \times 10^{-4})^2 \times 0.01 \approx 2.9 \times 10^{-10} \text{ rad}^2
$$

标准差约为 $1.7 \times 10^{-5}$ rad $\approx 0.001°$。这是 0.1 秒积分的旋转不确定性——非常小，说明短时间内 IMU 旋转积分是可靠的。

> 💡 **概念误区**：认为"预积分在优化中从不需要重新计算"
>
> **新手想法**："预积分的全部意义就是避免重新积分，所以任何情况下都不应该重算。"
>
> **实际上**：一阶偏差校正 $\Delta R(\bar{b} + \delta b) \approx \Delta R(\bar{b}) \cdot \operatorname{Exp}(\frac{\partial\Delta R}{\partial b_g} \delta b_g)$ 是一阶近似，当 $\delta b$ 足够大时精度不够。Forster 论文建议：当偏差更新量超过某个阈值时，应当丢弃当前预积分量并从新的偏差估计重新积分。工程中的常见阈值是偏差变化超过 $5\sigma_{bias}$。
>
> **正确做法**：记录预积分时使用的偏差 $\bar{b}$，每次优化后检查 $\|\hat{b} - \bar{b}\|$ 是否超出线性化有效范围。未超出时使用一阶校正，超出时重新积分。

> ⚠️ **编程陷阱 A**：混淆机体坐标系和世界坐标系的加速度
>
> **错误做法**：在递推 $\Delta v$ 时，直接把 IMU 原始加速度 $a_m$ 累加到 $\Delta v$ 中，忘记乘以旋转 $\Delta R_k$。
>
> **现象**：静止场景下 $\Delta v$ 不为零，而且随积分时间线性增长。
>
> **根本原因**：$\Delta v_{ij} = \sum \Delta R_{ik} (a_k - b_a) \Delta t$ 中的 $\Delta R_{ik}$ 是把加速度从机体坐标系旋转到第 $i$ 帧的坐标系。忘记这一步，等于假设 IMU 的机体系和世界系方向始终一致。
>
> **正确做法**：每步都要用当前的旋转预积分 $\Delta R_{ik}$ 去旋转加速度，然后再累加。

> ⚠️ **编程陷阱 B**：在积分前忘记减去重力
>
> **错误做法**：预积分公式中的加速度是"比力"（specific force）$a_k - b_a$，但在构建残差时忘记在世界坐标系中减去重力：$\Delta v_{ij} \approx R_i^\top (v_j - v_i - g \Delta t_{ij})$。
>
> **现象**：系统在水平运动时"大致正确"，但在有明显俯仰变化时位姿估计严重漂移。
>
> **根本原因**：加速度计测量的是"比力"（合外力减去重力的效应），不是惯性加速度。预积分是在无重力参考系下进行的，重力效应在构建残差时才通过 $-g\Delta t_{ij}$ 和 $-\frac{1}{2}g\Delta t_{ij}^2$ 补偿回来。
>
> **正确做法**：牢记预积分量**不包含**重力——重力只在最终残差中出现。检查方法：静止场景下，$\Delta v_{ij}$ 应接近零（不是接近 $g\Delta t$），$\Delta p_{ij}$ 也应接近零（不是接近 $\frac{1}{2}g\Delta t^2$）。

#### 练习

1. 从 $\Delta \tilde{R}_{k+1} = \Delta \tilde{R}_k \operatorname{Exp}\big((\omega_k - b_g - n_g^k)\Delta t\big)$ 出发，利用 BCH 一阶近似，推导旋转预积分误差 $\delta\phi$ 的递推方程。明确指出 $J_r(\phi_k)$ 出现在哪一步、为什么出现。 ⭐⭐⭐

2. 用上述数值算例的参数，假设陀螺仪偏差为 $b_g = [0.001, 0, 0]^\top$ rad/s。计算 10 步后旋转预积分对 $b_g$ 的 Jacobian $\frac{\partial \Delta R_{ij}}{\partial b_g}$ 的近似值（提示：每步的贡献为 $-J_r(\phi_k)\Delta t$，总计约 $-0.1 I$），并解释当 $b_g$ 被更新 $\delta b_g = [0.0005, 0, 0]^\top$ 时，旋转预积分量应如何修正。 ⭐⭐⭐

---

### 18. 数值验证与单元测试 ⭐⭐

#### 18.1 有限差分验证模板 ⭐⭐

解析 Jacobian 写完后必须验证。

对残差：

$$
r(X)
$$

右扰动数值差分：

$$
J_{\cdot i}
\approx
\frac{
r(X\operatorname{Exp}(\epsilon e_i))
-r(X\operatorname{Exp}(-\epsilon e_i))
}{2\epsilon}
$$

左扰动则改成：

$$
J_{\cdot i}
\approx
\frac{
r(\operatorname{Exp}(\epsilon e_i)X)
-r(\operatorname{Exp}(-\epsilon e_i)X)
}{2\epsilon}
$$

两者不能混用。

#### 18.2 C++ 伪代码 ⭐⭐

```cpp
// 使用右扰动检查一个 SE(3) 残差的 Jacobian。
// delta 的维度和排序必须与解析 Jacobian 完全一致。
Eigen::MatrixXd NumericalJacobianRight(const SE3& T) {
  const double eps = 1e-6;
  Eigen::VectorXd r0 = Residual(T);
  Eigen::MatrixXd J(r0.size(), 6);

  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> d;
    d.setZero();
    d(i) = eps;

    SE3 Tp = T * SE3::Exp(d);   // 右扰动
    SE3 Tm = T * SE3::Exp(-d);  // 右扰动

    J.col(i) = (Residual(Tp) - Residual(Tm)) / (2.0 * eps);
  }

  return J;
}
```

这段测试要和三件事绑定：

1. 扰动方向。
2. 切向量排序。
3. residual 的局部坐标定义。

如果任何一个不一致，有限差分会指出错误，但不会告诉你是哪一层错。

#### 18.3 故障排查表 ⭐⭐

| 现象 | 常见原因 | 检查方式 | 修复方式 |
|------|----------|----------|----------|
| 解析 Jacobian 与数值差分差一个负号 | 左/右扰动反了 | 改变扰动乘法侧测试 | 统一扰动约定 |
| 平移列和旋转列位置互换 | $[\rho,\phi]$ 与 $[\phi,\rho]$ 混用 | 打印单位扰动的物理效果 | 引入置换矩阵或改排序 |
| 小角度时出现 NaN | 闭式系数 $0/0$ | 测试 $\theta=10^{-9}$ | 加 Taylor 分支 |
| 大角度接近 $\pi$ 时不连续 | Log 分支切换 | 扫描角度从 $-\pi$ 到 $\pi$ | 避免跨分支或重置线性化点 |
| 协方差传播不对称 | Jacobian 或 Adjoint 排序错 | 检查 $\Sigma-\Sigma^\top$ | 对称化只是临时止血，根因仍要修 |
| IMU 预积分漂移异常 | $J_r$ 与 $J_l$ 用错 | 检查噪声是左乘还是右乘 | 按 Forster/Barfoot 约定重推 |

---

### 练习：雅可比矩阵与 BCH 公式 ⭐⭐

1. 从矩阵指数二阶展开出发，手推 BCH 的 $\frac{1}{2}[A,B]$ 项。
2. 从积分定义证明 $J_r(\phi)=J_l(-\phi)$。
3. 利用 $[\phi]_\times^3=-\theta^2[\phi]_\times$，推导 $SO(3)$ 左 Jacobian 闭式。
4. 写出 $J_l(\phi)$ 在 $\theta\to0$ 时的三阶 Taylor 展开，并说明为什么要在代码里分支。
5. 采用平移在前排序，写出 $SE(3)$ 的 Adjoint；再改为旋转在前排序，说明块位置如何变化。
6. 对 $y=Rp+t$，分别用左扰动和右扰动推导 $y$ 对 $\delta\xi$ 的 Jacobian。
7. 实现一个有限差分测试，验证你写的 `between_factor_jacobian`。
8. 阅读一个开源库的 Pose3 Jacobian 接口，记录它使用的扰动方向和切向量排序。

### 跨章综合题 ⭐⭐⭐

给定位姿图残差：

$$
r=\operatorname{Log}(Z^{-1}T_i^{-1}T_j)
$$

请完成：

1. 说明 $T_i,T_j,Z$ 所在李群。
2. 选择右扰动模型并写出 $T_i^+,T_j^+$。
3. 用 Adjoint 把扰动搬到残差右侧。
4. 用 BCH 一阶公式把 $\operatorname{Log}$ 的扰动提出来。
5. 写出 Jacobian 中必然出现 $J_r^{-1}(r)$ 的原因。
6. 结合专题 5，说明如果边噪声协方差定义在另一坐标系，为什么需要 Adjoint 搬运。

---

### 19. 与后续专题的桥梁 ⭐

本专题是"数学工具→工程应用"的关键枢纽，向后辐射到路线图的几乎每一个方向：

- **→ 专题5（李群上的不确定性）**：Jacobian直接用于协方差传播 $\boldsymbol{\Sigma}_Y=\mathbf{J}\boldsymbol{\Sigma}_X\mathbf{J}^T$，这是Barfoot-Furgale 2014论文的核心
- **→ 专题6（等变理论）**：Jacobian在等变性保持中的角色——InEKF之所以优于EKF，是因为其Jacobian具有对称群不变结构
- **→ 第二批（流形优化）**：流形上的Gauss-Newton需要本专题的Jacobian来构造法方程 $\mathbf{J}^T\mathbf{J}\delta\boldsymbol{\xi}=-\mathbf{J}^T\mathbf{r}$
- **→ 第四批（动力学）**：Pinocchio的RNEA/ABA解析导数以本专题的李群微积分为基础
- **→ 第五批（SLAM后端）**：所有pose-graph、BA、VIO后端求解都直接依赖本专题的Jacobian公式

---

### 19bis. ad 算子的深入理解 ⭐⭐⭐

#### 19bis.1 ad 算子是什么 ⭐⭐⭐

$\operatorname{ad}$ 算子是 Lie bracket 的矩阵表示。

对李代数元素 $X$，$\operatorname{ad}_X$ 定义为：

$$
\operatorname{ad}_X(Y) = [X, Y]
$$

它是一个从李代数到自身的线性映射。

在 SO(3) 上，Lie bracket 是向量叉乘：

$$
[\phi, \psi] = \phi \times \psi
$$

因此 $\operatorname{ad}_\phi$ 的矩阵表示就是叉乘矩阵：

$$
\operatorname{ad}_\phi = [\phi]_\times = \begin{pmatrix} 0 & -\phi_3 & \phi_2 \\ \phi_3 & 0 & -\phi_1 \\ -\phi_2 & \phi_1 & 0 \end{pmatrix}
$$

在 SE(3) 上（平移在前排序 $\xi = [\rho, \phi]^\top$）：

$$
\operatorname{ad}_\xi = \begin{pmatrix} [\phi]_\times & [\rho]_\times \\ 0 & [\phi]_\times \end{pmatrix}
$$

#### 19bis.2 ad 算子的幂次与循环性 ⭐⭐⭐

SO(3) 上的关键性质：

$$
(\operatorname{ad}_\phi)^3 = -\theta^2 \operatorname{ad}_\phi \qquad (\theta = \|\phi\|)
$$

这是因为 $[\phi]_\times^3 = -\theta^2 [\phi]_\times$（源自反对称矩阵的循环性）。

此性质使得涉及 $\operatorname{ad}$ 的无穷级数可以封闭求和——这正是 $J_l$、$J_r$ 闭式的来源。

SE(3) 上没有如此简洁的循环性，因为 $6\times6$ 的 ad 矩阵结构更复杂。SE(3) 的 Jacobian 闭式需要对旋转和平移部分分别处理，最终得到 $6\times6$ 块矩阵形式。

#### 19bis.3 ad、Ad 和群运算的关系 ⭐⭐⭐

三者的关系形成一个完整的链条：

| 对象 | 层级 | 作用 | 公式 |
|------|------|------|------|
| 群乘法 $L_g$ | 群 $G$ | 左平移 | $L_g(h) = gh$ |
| Adjoint $\operatorname{Ad}_g$ | 李代数 $\mathfrak{g}$ | 群元素对切向量的共轭 | $\operatorname{Ad}_g X = g X g^{-1}$ |
| ad 算子 $\operatorname{ad}_X$ | 李代数 $\mathfrak{g}$ | 切向量对切向量的括号 | $\operatorname{ad}_X Y = [X,Y]$ |

它们满足：

$$
\operatorname{Ad}_{\exp(X)} = \exp(\operatorname{ad}_X) = \sum_{k=0}^{\infty} \frac{(\operatorname{ad}_X)^k}{k!}
$$

这个等式极为重要——它把群上的 Adjoint 与李代数上的 ad 通过指数映射联系起来。

在 SO(3) 上的具体表现：

$$
\operatorname{Ad}_R \omega = R\omega \qquad (\text{向量被旋转})
$$

$$
\exp(\operatorname{ad}_\phi) = \exp([\phi]_\times) = R(\phi) \qquad (\text{Rodrigues 公式!})
$$

因此 $\operatorname{Ad}_{\exp(\phi^\wedge)}$ 作用在 $\mathbb{R}^3$ 上就是旋转矩阵 $R(\phi)$ 本身。

**类比理解**：如果把李群比作地球，李代数比作某点的切平面，那么 $\operatorname{Ad}$ 是"把一张切平面上的箭头用群运算搬到另一张切平面"，$\operatorname{ad}$ 是"同一张切平面内两个箭头的交互作用"。$J_l$ 和 $J_r$ 则度量了"从切平面到地球表面"的搬运过程中距离的变形程度。

⚠️ **概念误区：认为 Ad 和 ad 是同一个东西的大小写差异**

这是初学者最常犯的混淆。$\operatorname{Ad}_g$ 是群元素 $g$ 对整个李代数的线性变换（在 SE(3) 中是 $6\times6$ 矩阵）；$\operatorname{ad}_X$ 是李代数元素 $X$ 对其他李代数元素的线性变换（也是 $6\times6$ 矩阵）。前者的参数是群元素，后者的参数是李代数元素。两者通过 $\operatorname{Ad}_{\exp X} = \exp(\operatorname{ad}_X)$ 联系，但在数值上完全是不同的矩阵。

为了进一步澄清这个区别，考虑一个具体的 SE(3) 例子。设 $T = (R, t)$ 是一个位姿，$\xi = [\rho, \phi]^\top$ 是一个切向量。$\operatorname{Ad}_T$ 把切向量 $\xi$ 从一个坐标系搬到另一个坐标系——它的矩阵元素包含 $R$ 和 $[t]_\times R$，因此依赖群元素 $T$ 的具体值。而 $\operatorname{ad}_\xi$ 描述切向量 $\xi$ 对其他切向量的"搅动效应"——它的矩阵元素只包含 $[\phi]_\times$ 和 $[\rho]_\times$，不依赖任何群元素。在 SLAM 后端中，$\operatorname{Ad}_T$ 出现在 between factor 的 Jacobian 中（用于把扰动搬到同一侧），$\operatorname{ad}_\xi$ 出现在 $J_l$ 和 $J_r$ 的级数展开中（用于计算指数映射的微分）。混淆两者会导致 Jacobian 矩阵的维度和数值都出错。

#### 19bis.4 Jacobi 恒等式与李代数结构 ⭐⭐⭐⭐

Lie bracket 满足 Jacobi 恒等式：

$$
[X,[Y,Z]] + [Y,[Z,X]] + [Z,[X,Y]] = 0
$$

这不是显然的恒等式——它是李代数定义的一部分。

在 SO(3) 中，Jacobi 恒等式等价于向量三重叉乘恒等式：

$$
a \times (b \times c) + b \times (c \times a) + c \times (a \times b) = 0
$$

Jacobi 恒等式保证了 BCH 公式的存在性和自洽性。如果没有它，$\log(\exp X \exp Y)$ 的级数展开不一定收敛到李代数中的元素。

**如果 Jacobi 恒等式不成立会怎样？** 考虑一个假想的"伪李代数"，其 bracket 定义为 $[X,Y] = XY$（不减去 $YX$）。这个 bracket 不满足 Jacobi 恒等式。如果用它构造 BCH 级数，展开出的无穷级数不能保证收敛到代数中的元素——换句话说，$\exp(X)\exp(Y)$ 可能无法写成 $\exp(Z)$ 的形式。Jacobi 恒等式本质上保证了李代数的"结构常数"足够自洽，使得群上的乘法可以在代数层面用级数精确表达。在工程中我们不需要验证 Jacobi 恒等式（SO(3) 和 SE(3) 自动满足），但理解它的角色有助于区分"合法的李群操作"和"数学上不保证正确的近似"。

---

### 20. BCH 公式的完整展开与高阶项 ⭐⭐⭐

#### 20.1 BCH 公式的前四阶完整展开 ⭐⭐⭐

Baker-Campbell-Hausdorff 公式回答了一个核心问题：给定两个李代数元素 $X, Y$，$\log(\exp(X)\exp(Y))$ 等于什么？

完整展开到四阶：

$$
\log(\exp X \exp Y) = X + Y + \frac{1}{2}[X,Y] + \frac{1}{12}([X,[X,Y]] + [Y,[Y,X]]) - \frac{1}{24}[Y,[X,[X,Y]]] + \cdots
$$

各阶的含义：

| 阶数 | 项 | 物理含义 |
|------|-----|----------|
| 1阶 | $X + Y$ | 交换假设（如果群是 Abel 的就止步于此） |
| 2阶 | $\frac{1}{2}[X,Y]$ | 非交换性的一阶修正 |
| 3阶 | $\frac{1}{12}([X,[X,Y]]+[Y,[Y,X]])$ | 嵌套交换子（曲率效应） |
| 4阶 | $-\frac{1}{24}[Y,[X,[X,Y]]]$ | 更高阶非线性 |

**在 SO(3) 上**，Lie bracket 就是叉乘：$[\phi, \psi] = \phi \times \psi$。因此：

$$
\log(\exp(\phi^\wedge)\exp(\psi^\wedge))^\vee \approx \phi + \psi + \frac{1}{2}\phi\times\psi + \frac{1}{12}(\phi\times(\phi\times\psi) + \psi\times(\psi\times\phi)) + \cdots
$$

当 $\phi$ 和 $\psi$ 都很小时（$\|\phi\|, \|\psi\| \ll 1$），高阶项可忽略：

$$
\log(\exp(\phi^\wedge)\exp(\psi^\wedge))^\vee \approx \phi + \psi
$$

这就是为什么"小扰动近似可加"——但只在一阶成立。

#### 20.2 BCH 在 IMU 预积分中的应用 ⭐⭐⭐

Forster et al. (2017) 的 IMU 预积分核心依赖 BCH 公式。

预积分的旋转部分需要处理：

$$
\Delta R_{ij} = \prod_{k=i}^{j-1} \exp((\tilde\omega_k - b_g)\Delta t)
$$

当陀螺 bias $b_g$ 改变时，不想重新积分整个序列。用 BCH 一阶近似：

$$
\Delta R_{ij}(b_g + \delta b_g) \approx \Delta R_{ij}(b_g) \cdot \exp\left(\frac{\partial \Delta R_{ij}}{\partial b_g}\delta b_g\right)
$$

其中的偏导数正是右 Jacobian 的累积：

$$
\frac{\partial \Delta R_{ij}}{\partial b_g} = -\sum_{k=i}^{j-1} \left(\prod_{l=k+1}^{j-1}\exp(\omega_l\Delta t)\right)^{-1} J_r(\omega_k\Delta t)\Delta t
$$

如果没有 BCH 公式和右 Jacobian，每次 bias 更新都需要重新积分数百个 IMU 样本——在实时 VIO 中完全不可接受。

> **本质洞察**：BCH 公式的工程价值不在于精确计算 $\log(\exp X \exp Y)$（这可以直接先乘后取 log），而在于提供了关于参数（如 bias）的一阶/二阶展开，使得"参数变化→结果变化"可以用 Jacobian 线性化表达。这正是优化和估计所需要的。

#### 20.3 BCH 的收敛性 ⭐⭐⭐⭐

BCH 级数不一定收敛。在什么条件下收敛？

**充分条件**（Dynkin 1947）：当 $\|X\| + \|Y\| < \log 2$（在任何亚乘范数下）时，BCH 级数绝对收敛。

对 SO(3) 而言，这意味着 $\|\phi\| + \|\psi\| < \log 2 \approx 0.693$ 弧度（约 $39.7°$）保证收敛。

在 SLAM 中，两个相邻帧之间的相对旋转通常远小于这个阈值。因此 BCH 的一阶截断在正常操作条件下是安全的。

但在大运动场景（如快速转弯、跟踪丢失后重定位）中，相对旋转可能超过收敛域。此时不应使用 BCH 展开，而应直接计算矩阵乘法后取 Log。

**BCH 收敛域与工程实践的关系**：

| 场景 | 典型相对旋转量 | 是否在收敛域内 | 推荐做法 |
|------|--------------|---------------|---------|
| 相邻帧 VIO（10ms 间隔） | $< 0.05$ rad | 远在域内 | 一阶 BCH 足够 |
| 相邻关键帧（0.5s 间隔） | $< 0.3$ rad | 在域内 | 一阶 BCH 安全 |
| 回环闭合修正 | $0.1 - 1.0$ rad | 可能接近边界 | 直接矩阵乘法 + Log |
| 重定位 / 大回环 | $> 1.0$ rad | 超出收敛域 | 必须直接计算 |
| 高速无人机转弯（IMU 积分） | $0.5 - 2.0$ rad | 可能超出 | 需要重置预积分 |

这张表解释了 Forster 预积分论文中"关键帧间隔不能过长"的数学原因——不仅是数值积分误差累积，BCH 一阶近似本身也会因为旋转增量过大而失效。

从另一个角度看，BCH 收敛域的大小本质上由李代数的结构常数决定。SO(3) 的结构常数是 Levi-Civita 符号 $\epsilon_{ijk}$，对应的 $\operatorname{ad}$ 算子范数与旋转角成正比。当旋转角足够大时，$\operatorname{ad}^k$ 的贡献不再快速衰减，级数收敛性不再有保证。这与 Taylor 级数在收敛半径之外发散的原理完全类似——BCH 级数的收敛半径 $\log 2$ 正是此类分析的结果。

---

### 21. $J_l^{-1}$ 和 $J_r^{-1}$ 的完整推导 ⭐⭐⭐

#### 21.1 为什么需要逆 Jacobian ⭐⭐⭐

在 SLAM 的法方程中，残差的 Jacobian 常包含 $J_r^{-1}$ 或 $J_l^{-1}$。

回忆 BCH 的一阶近似（右扰动形式）：

$$
\log(\exp(\phi)\exp(\delta\phi)) \approx \phi + J_r^{-1}(\phi)\delta\phi
$$

这个公式中出现的恰好是 $J_r^{-1}$，不是 $J_r$ 本身。

直觉理解：$J_r(\phi)$ 把切空间的增量 $\delta\phi$ "翻译"成群上的乘法效果；$J_r^{-1}(\phi)$ 做反向翻译——从群上观测到的变化推回切空间的增量。后者正是 SLAM 残差求导所需要的。

#### 21.2 SO(3) 上 $J_l^{-1}$ 的闭式 ⭐⭐⭐

$$
J_l^{-1}(\phi) = I - \frac{1}{2}[\phi]_\times + \left(\frac{1}{\theta^2} - \frac{1+\cos\theta}{2\theta\sin\theta}\right)[\phi]_\times^2
$$

其中 $\theta = \|\phi\|$。第三个系数可以化简为 $\frac{1}{\theta^2}(1 - \frac{\theta}{2}\cot\frac{\theta}{2})$。

验证方法：$J_l(\phi) \cdot J_l^{-1}(\phi)$ 应等于 $I$。

当 $\theta \to 0$：

$$
J_l^{-1}(\phi) \approx I - \frac{1}{2}[\phi]_\times + \frac{1}{12}[\phi]_\times^2
$$

当 $\theta \to \pi$：$\cot(\theta/2) \to 0$，系数趋于 $1/\theta^2$——有界，不发散。

⚠️ **数值陷阱：$J_l^{-1}$ 在 $\theta = 2k\pi$（$k \ne 0$）处有奇异**

$\sin\theta = 0$ 导致分母为零。在 SLAM 中这对应 $360°$ 旋转——实践中很少出现，但自动化测试应覆盖此边界。

---

### 21bis. SE(3) Jacobian 的 6x6 块结构详解 ⭐⭐⭐

#### 21bis.1 SE(3) 左 Jacobian 的块分解 ⭐⭐⭐

SE(3) 的左 Jacobian 是 $6\times6$ 矩阵（平移在前排序 $\xi = [\rho, \phi]^\top$）：

$$
\mathbf{J}_l(\xi) = \begin{pmatrix} J_l^{(tt)} & J_l^{(t\omega)} \\ \mathbf{0} & J_l^{(\omega\omega)} \end{pmatrix}
$$

其中：
- $J_l^{(\omega\omega)} = J_l^{SO(3)}(\phi)$：SO(3) 的左 Jacobian（已知闭式）
- $J_l^{(tt)}$：平移对平移的耦合（也等于 $J_l^{SO(3)}(\phi)$）
- $J_l^{(t\omega)}$：平移对旋转的交叉项（最复杂的块）

交叉项 $J_l^{(t\omega)}$ 的表达式（Barfoot Ch.8）：

$$
J_l^{(t\omega)} = Q_l(\xi)
$$

$Q_l$ 是一个 $3\times3$ 矩阵，其表达式涉及 $\rho$、$\phi$、$\theta$ 和多个三角函数的组合。完整公式较长（约 10 个项），但可以通过以下结构记忆：

$$
Q_l = \frac{1}{2}[\rho]_\times + c_1([\phi]_\times[\rho]_\times + [\rho]_\times[\phi]_\times + [\phi]_\times[\rho]_\times[\phi]_\times) + \cdots
$$

核心认知：$Q_l$ 的存在反映了 SE(3) 是半直积而非直积的事实。如果旋转和平移完全解耦，这个块会是零矩阵。

#### 21bis.2 右 Jacobian 与左 Jacobian 的关系 ⭐⭐⭐

$$
\mathbf{J}_r(\xi) = \mathbf{J}_l(-\xi)
$$

这对 SE(3) 同样成立。另外：

$$
\mathbf{J}_l(\xi) = \operatorname{Ad}_{\operatorname{Exp}(\xi)} \cdot \mathbf{J}_r(\xi)
$$

在代码中，通常只实现一个（比如 $J_l$），另一个通过取负或乘 Adjoint 得到。

#### 21bis.3 实用计算策略 ⭐⭐

在工程中，SE(3) 的 $6\times6$ Jacobian 计算有三种策略：

| 策略 | 精度 | 计算量 | 适用场景 |
|------|------|--------|----------|
| 完整闭式（Barfoot/Sola） | 精确 | $O(1)$ 但公式复杂 | 需要解析 Hessian |
| 一阶近似 $J \approx I + \frac{1}{2}\operatorname{ad}_\xi$ | $O(\|\xi\|^2)$ | 极低 | 小扰动、快速原型 |
| 数值有限差分 | 取决于 $\epsilon$ | $O(12)$ 次 Exp/Log | 验证、非性能关键路径 |

> **本质洞察**：在 SLAM 后端优化中，Jacobian 在每次迭代都要重新计算（因为线性化点变了）。如果迭代接近收敛（$\|\xi\|$ 已经很小），一阶近似 $J \approx I$ 足够准确。只有在初始残差大或需要二次收敛时，完整闭式才有必要。这就是为什么很多工程实现在前几次迭代用完整 Jacobian，后期自动退化到恒等近似。

#### 21bis.4 SymPy 验证 SE(3) Jacobian 块结构 ⭐⭐

```python
import sympy as sp

# 验证 SE(3) 左 Jacobian 的下三角块结构
# 即证明旋转部分不受平移影响
rho1, rho2, rho3 = sp.symbols('rho1 rho2 rho3')
phi1, phi2, phi3 = sp.symbols('phi1 phi2 phi3')

# ad 矩阵（平移在前排序）
def se3_ad(rho, phi):
    phi_x = sp.Matrix([[0, -phi[2], phi[1]],
                       [phi[2], 0, -phi[0]],
                       [-phi[1], phi[0], 0]])
    rho_x = sp.Matrix([[0, -rho[2], rho[1]],
                       [rho[2], 0, -rho[0]],
                       [-rho[1], rho[0], 0]])
    return sp.BlockMatrix([[phi_x, rho_x],
                           [sp.zeros(3), phi_x]])

# J_l 的级数前两项
rho = [rho1, rho2, rho3]
phi = [phi1, phi2, phi3]
ad = se3_ad(rho, phi)
# J_l ≈ I + 1/2 ad + 1/6 ad^2 + ...
# 验证右下角块只依赖 phi（不含 rho）
```

这段代码可以确认：$\mathbf{J}_l$ 的右下 $3\times3$ 块确实只取决于 $\phi$，不依赖 $\rho$。这是 SE(3) 半直积结构的直接体现。

---

### 22. 本章知识树总结 ⭐

```text
雅可比矩阵与 BCH 公式
├── 扰动模型
│   ├── 左扰动 (world frame)
│   ├── 右扰动 (body frame)
│   └── Adjoint 互转
├── 左/右 Jacobian
│   ├── 积分定义
│   ├── 级数展开
│   ├── SO(3) 闭式
│   ├── SE(3) 6x6 块结构
│   └── 逆 Jacobian 闭式
├── BCH 公式
│   ├── 一阶：X + Y
│   ├── 二阶：+ 1/2 [X,Y]
│   ├── 三阶与四阶
│   ├── 收敛域
│   └── IMU 预积分应用
├── 常用 Jacobian
│   ├── 点变换 y = Rp + t
│   ├── Between factor
│   ├── IMU preintegration
│   └── 投影/重投影
├── Convention 对照
│   ├── 切向量排序
│   ├── 四元数约定
│   └── 库间转换
└── 验证方法
    ├── 有限差分测试
    ├── 属性验证 (J_r = J_l(-φ))
    └── 数值边界测试
```

---

### 23. 本章小结 ⭐

| 核心概念 | 一句话定义 | 工程对应 | 闭式条件 |
|----------|-----------|----------|----------|
| 左 Jacobian $J_l$ | $\exp(\phi+\delta\phi) \approx \exp(J_l\delta\phi)\exp(\phi)$ | 左扰动模型的导数 | SO(3): 含 $\sin\theta/\theta$ |
| 右 Jacobian $J_r$ | $\exp(\phi+\delta\phi) \approx \exp(\phi)\exp(J_r\delta\phi)$ | 右扰动模型的导数 | $J_r(\phi) = J_l(-\phi)$ |
| $J_l^{-1}$ / $J_r^{-1}$ | BCH 一阶近似中的系数 | SLAM 残差 Jacobian | 含 $\cot(\theta/2)$ |
| BCH 一阶 | $\log(\exp X\exp Y) \approx X+Y$ | 小扰动近似可加 | $\|X\|+\|Y\| \ll 1$ |
| BCH 二阶 | $\approx X+Y+\frac{1}{2}[X,Y]$ | 非交换修正 | $[X,Y] = X\times Y$ (SO3) |
| 有限差分验证 | $(r(x\oplus\epsilon) - r(x\ominus\epsilon))/(2\epsilon)$ | Jacobian 正确性检查 | $\epsilon \sim 10^{-7}$ |

---

### 24. 累积项目：本章新增模块 ⭐

**项目方向**：手写几何验证库

本章新增：**Jacobian 与 BCH 验证工具**

```python
import numpy as np

def so3_left_jacobian(phi):
    """SO(3) 左 Jacobian 闭式"""
    theta = np.linalg.norm(phi)
    if theta < 1e-10:
        return np.eye(3) + 0.5 * SO3.hat(phi)
    n = phi / theta
    N = SO3.hat(n)
    # J_l = sin(θ)/θ I + (1-sin(θ)/θ)nn^T + (1-cos(θ))/θ N
    return (np.sin(theta)/theta) * np.eye(3) + \
           (1 - np.sin(theta)/theta) * np.outer(n, n) + \
           ((1-np.cos(theta))/theta) * N

def so3_right_jacobian(phi):
    """SO(3) 右 Jacobian: J_r(φ) = J_l(-φ)"""
    return so3_left_jacobian(-phi)

def bch_first_order(phi, psi):
    """BCH 一阶近似"""
    return phi + psi

def bch_second_order(phi, psi):
    """BCH 二阶近似"""
    return phi + psi + 0.5 * np.cross(phi, psi)

# 验证 J_r(φ) = J_l(-φ)
phi = np.array([0.3, -0.5, 0.7])
Jr = so3_right_jacobian(phi)
Jl_neg = so3_left_jacobian(-phi)
print("J_r(φ) = J_l(-φ)?", np.allclose(Jr, Jl_neg))  # True

# 验证 BCH 展开精度
psi = np.array([0.05, -0.03, 0.02])
# 精确值
from scipy.linalg import logm
exact = SO3.log(SO3.exp(phi) @ SO3.exp(psi))
bch1 = bch_first_order(phi, psi)
bch2 = bch_second_order(phi, psi)
print("BCH 1阶误差:", np.linalg.norm(exact - bch1))
print("BCH 2阶误差:", np.linalg.norm(exact - bch2))
```

---

### 延伸阅读 ⭐

| 资源 | 难度 | 核心价值 |
|------|------|----------|
| Sola "A micro Lie theory" (2018) Appendix B | ⭐⭐ | SO(3)/SE(3) Jacobian 闭式全表 |
| Barfoot "State Estimation for Robotics" 2nd ed. Ch.8 | ⭐⭐⭐ | SE(3) Jacobian 与误差传播 |
| Eade "Lie Groups for 2D and 3D Transformations" | ⭐⭐ | 简洁工程导向推导 |
| Rossmann "Lie Groups: An Introduction Through Linear Groups" | ⭐⭐⭐ | BCH 公式的严格证明 |
| Hall "Lie Groups, Lie Algebras, and Representations" Ch.5 | ⭐⭐⭐⭐ | BCH 收敛性定理的完整证明 |
| Dynkin "Calculation of BCH coefficients" (1947) | ⭐⭐⭐⭐ | BCH 系数的组合学公式 |
| Wikipedia "Baker-Campbell-Hausdorff formula" | ⭐⭐ | 高阶展开项的参考 |
| Forster et al. "On-Manifold Preintegration" (T-RO 2017) | ⭐⭐⭐ | BCH 在预积分中的核心应用 |
| manif 库 Jacobian 测试代码 | ⭐⭐ | 有限差分验证的工程范例 |

---

### 🔧 故障排查手册 ⭐

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|----------|----------|--------|
| 解析 Jacobian 与数值差分差一个负号 | 左/右扰动选反 | 1.改变扰动乘法侧 2.检查残差定义方向 3.与 Sola 论文公式逐项对比 | §4.1 |
| 平移列和旋转列互换 | 切向量排序不一致 | 1.打印单位扰动效果 2.对照库文档 3.用置换矩阵转换 | §4.8 |
| 小角度时 Jacobian 出现 NaN | $\sin\theta/\theta$ 的 0/0 | 1.测试 $\theta=10^{-12}$ 2.加 Taylor 分支 3.与常值极限对比 | §4.3 |
| 大角度接近 $\pi$ 时跳变 | Log 分支切换或 $J^{-1}$ 奇异 | 1.扫描 $\theta$ 从 0 到 $\pi$ 2.检查 $\cot(\theta/2)$ 3.避免 $2\pi$ 附近 | §21 |
| IMU 预积分随时间漂移 | $J_r$ 与 $J_l$ 混用 | 1.确认噪声模型左/右 2.检查 bias Jacobian 符号 3.对照 Forster 公式 | §20.2 |
| 协方差传播后矩阵不正定 | Jacobian 方向/排序错 | 1.检查 $J\Sigma J^\top$ 对称性 2.打印特征值 3.用数值 Jacobian 替换排查 | §18 |

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|----------|
| "左 Jacobian 和右 Jacobian 是两套独立的公式" | 它们是同一微分对象在不同平凡化（左/右）下的表达，满足 $J_r(\phi) = J_l(-\phi)$ 和 $J_l = \mathrm{Ad} \cdot J_r$ |
| "BCH 公式的高阶项在工程中永远可以忽略" | 一阶近似只在扰动量足够小时成立；当旋转增量较大（如 IMU 长时间积分）时，$\frac{1}{2}[A,B]$ 项不可忽略 |
| "SE(3) 的 Jacobian 是两个 SO(3) Jacobian 的对角拼接" | SE(3) 是半直积，$6\times6$ Jacobian 含非对角的 $Q_l$ 块，编码旋转与平移的耦合 |
| "SO(3) 上 $\mathrm{Ad}_R = R$ 的简化可以推广到 SE(3)" | 这是 SO(3) 的巧合；SE(3) 的 Adjoint 是 $6\times6$ 矩阵，含 $[t]_\times R$ 耦合项 |
| "不同库的 Jacobian 可以直接互相抄用" | 切向量排序（$[\rho,\phi]$ vs $[\phi,\rho]$）和扰动方向（左 vs 右）不同，会导致矩阵块位置互换和符号差异 |
| "小角度分支只是性能优化" | 小角度分支是正确性要求：$\sin\theta/\theta$ 在 $\theta \to 0$ 时的浮点消减会产生 NaN |
| "$J_l^{-1}$ 可以通过数值反转 $J_l$ 得到" | 在 $\theta$ 接近 $2\pi$ 时条件数极差，应使用闭式 $J_l^{-1}$ 表达式 |

---

#### 总结 ⭐

本专题将专题3建立的李群 exp/log 框架转化为可微分的计算工具。核心脉络是一条清晰的链条：**扰动模型（§4.1）定义了"对什么求导" → $\mathbf{J}_l$/$\mathbf{J}_r$（§4.2--4.4）给出 exp 映射本身的导数 → 常用 Jacobian（§4.5）将基本导数组合成 SLAM 中的实际残差导数 → BCH 公式（§4.7）处理两个 exp 的乘积 → Adjoint（§4.6）在不同 convention 间翻译 → convention 对照表（§4.8）保证代码实现不出错**。

掌握这条链条后，你面对任何 SLAM/VIO 论文中的 Jacobian 推导都不会再感到陌生——它们都是这几个基本积木的组合。

回顾全部四个专题的知识流向：专题1提供了"弯曲空间上如何做微积分"的底层语言；专题2把"切空间增量 → 流形状态"的更新方式形式化为 Retraction；专题3在流形上叠加群结构，建立了 $SO(3)$ 和 $SE(3)$ 的 exp/log/Adjoint 工具箱；本专题进一步提供了这些工具对参数的导数——Jacobian 和 BCH。四个专题合在一起，构成了机器人状态估计的完整数学基座：状态存储在流形上（专题1），优化增量在切空间中求解并通过 Retraction 回到流形（专题2），状态更新和误差定义使用群运算（专题3），线性化和噪声传播使用 Jacobian 和 BCH（专题4）。

建议学习节奏为：第一周精读 Sola 论文全文并手推 SO(3) 闭式，第二周对照 Barfoot Ch.8 完成 SE(3) Jacobian 和 BCH，第三周用 manif 库验证所有公式并推导 Forster 预积分的 Jacobian 作为综合练习。

完成本专题后，下一站是专题5——将 Jacobian 和 BCH 工具应用于李群上的不确定性传播与概率分布，这是 InEKF 和预积分理论的直接入口。

从工程角度而言，本专题最核心的产出是两个能力：

1. **独立推导能力**：面对任意一个涉及 $SO(3)$ 或 $SE(3)$ 的残差函数，能从第一步（写出扰动模型）到最后一步（读出 Jacobian 矩阵），全程不依赖背公式。这意味着即使遇到论文中使用的新 convention 或非标准残差定义，也能自行推导而非查表。

2. **调试验证能力**：写完解析 Jacobian 后，能用数值有限差分独立验证每一列的正确性——包括正确设置扰动方向、切向量排序和步长大小。这个能力在 SLAM/VIO 系统开发中几乎每天都会用到。

这两个能力的组合，意味着你不再是公式的搬运工，而是公式的制造者。面对新的传感器模型、新的运动约束或新的代价函数形式，你可以从零推导出正确的 Jacobian 并用系统化的方法验证它——这正是从"使用现有框架"到"设计新框架"的关键跨越。

### 符号表

| 符号 | 含义 | 首次出现 |
|------|------|---------|
| $\mathbf{J}_r(\phi)$ | 右 Jacobian（右扰动下 exp 映射的导数） | §14.1 |
| $\mathbf{J}_l(\phi)$ | 左 Jacobian（左扰动下 exp 映射的导数） | §14.2 |
| $\mathbf{J}_r^{-1}(\phi)$ | 右 Jacobian 的逆矩阵 | §15.3 |
| $\phi$ | 旋转向量 $\phi = \theta n \in \mathbb{R}^3$ | §14.1 |
| $\delta\phi$ | 右扰动增量（$R' = R \exp(\delta\phi^\wedge)$） | §14.1 |
| $\operatorname{Ad}_R$ | $SO(3)$ 的 Adjoint（$\operatorname{Ad}_R = R$） | §14.5 |
| $[A, B]$ | 矩阵交换子（Lie bracket）$AB - BA$ | §13.2 |
| $Q_l$ | $SE(3)$ 左 Jacobian 中的旋转-平移耦合块 | §本章常见误解汇总 |
| $\theta$ | 旋转角度 $\theta = \|\phi\|$ | §15.1 |
| $\operatorname{BCH}(\phi_1, \phi_2)$ | Baker-Campbell-Hausdorff 公式 $\log(\exp(\phi_1^\wedge)\exp(\phi_2^\wedge))^\vee$ | §13 |

---
