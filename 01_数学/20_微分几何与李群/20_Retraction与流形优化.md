## 专题2：Retraction理论与流形优化基础

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含练习与推导验证） | 8–10 小时 | 需要理解 Retraction 框架并实现流形优化算法的读者 |
| 速读（跳过推导细节） | 3–4 小时 | 已有优化基础、只需补全流形优化视角的读者 |
| 速查（只看表格和算法伪代码） | 30–40 分钟 | 遇到具体 Retraction 选型或 Armijo 条件问题时回来查 |

### 前置自测

📋 **前置自测**（答不出 >= 2 题 → 先回专题1「光滑流形一般理论」复习）

1. 切空间 $T_xM$ 的定义是什么？它与流形 $M$ 的区别在哪里？
2. 光滑映射 $F: M \to N$ 的切映射 $dF_p$ 把什么映射到什么？
3. 为什么在 $SO(3)$ 上不能直接做 $R_{k+1} = R_k + \Delta R$ 来更新旋转？
4. 梯度下降公式 $x_{k+1} = x_k - \alpha \nabla f(x_k)$ 隐含了什么关于状态空间的假设？
5. Riemannian 度量（内积）为什么在流形优化中不可或缺？

#### 0 为什么这个专题是连接几何与优化的桥梁 ⭐

**Retraction 是"切空间→流形"的回退映射（retraction map），它让流形上的优化算法成为可能。** 欧氏空间中梯度下降的更新 $x_{k+1} = x_k - \alpha\nabla f(x_k)$ 在流形上失效——因为 $x_k - \alpha\nabla f(x_k)$ 通常不在流形上。Retraction 正是解决这一矛盾的核心工具：先在切空间做"欧氏式"的一步，再通过 retraction 映射回流形。

这里需要建立的第一个关键认知是：**李群的 exp 映射只是 retraction 的一个特例**，而且常常是计算成本最高的选择。实践中 QR retraction、polar retraction、Cayley map 在 Stiefel 流形和 SO(n) 上比矩阵指数快一个数量级。第二个关键认知是：**李群指数映射（代数对象）和 Riemannian exp 映射（测地线的几何对象）是两件事**，二者仅在 bi-invariant 度量下恰好重合。

本专题在路线图中的位置十分明确：承接专题1（光滑流形一般理论）提供的切空间与光滑映射语言，为后续专题3（李群）提供"exp 只是 retraction 特例"的视角，同时是第二批"凸优化/非线性优化"中 manifold optimization 部分的完整前置。在机器人工程中，本专题的知识直接支撑 **ICP 对齐优化、PGO 中的 pose 优化（GTSAM 的 retract 接口）、SE-Sync 的 SDP 松弛求解，以及等变学习中的流形参数优化**。

---

#### 1 核心章节清单（档位3必学，25-35小时） ⭐

##### 1.1 Retraction 的一般定义（Absil-Mahony-Sepulchre 框架） ⭐⭐

- **Retraction** $R_x: T_xM \to M$ 的严格定义：满足 $R_x(0_x)=x$ 且 $\mathrm{D}R_x(0_x) = \mathrm{id}_{T_xM}$ 的光滑映射
- 一阶 retraction 为何对一阶优化（梯度下降）已经足够——因为一步迭代的误差为 $O(\|\xi\|^2)$，不影响一阶收敛性
- 二阶 retraction 的额外条件：与测地线的二阶相切，Newton 方法需要此精度
- 核心直觉：retraction 是对 exp 映射的"廉价近似"，精度–速度权衡是实际算法设计的核心

**资源**：Boumal 书 Ch.3 §3.5–3.6；Absil 书 Ch.4 Definition 4.1.1；Absil Ch.4 免费 PDF（press.princeton.edu/absil）

##### 1.2 常见 retraction 的例子与计算成本 ⭐⭐

| 流形 | Retraction 类型 | 计算复杂度 | 备注 |
|------|----------------|-----------|------|
| $S^{n-1}$（单位球面） | 归一化 $R_x(\xi) = (x+\xi)/\|x+\xi\|$ | $O(n)$ | 最便宜 |
| $S^{n-1}$ | Exponential map | $O(n)$ | 需要三角函数，略贵 |
| $\mathrm{St}(k,n)$（Stiefel 流形） | QR retraction | $O(nk^2)$ | **实践首选**，仅需薄 QR 分解 |
| $\mathrm{St}(k,n)$ | Polar retraction | $O(nk^2)$ | 精度优于 QR |
| $\mathrm{St}(k,n)$ | Cayley retraction | $O(nk^2)$ | 保结构 |
| $\mathrm{St}(k,n)$ | Exponential map | $O(n^3)$ 或迭代 | **最贵**，通常不必要 |
| $\mathrm{Gr}(k,n)$（Grassmann 流形） | 列空间投影 | $O(nk^2)$ | 自然商结构诱导 |
| $\mathrm{SO}(n)$ | Cayley map $(I-\xi/2)^{-1}(I+\xi/2)$ | $O(n^3)$ | 不需 exp |
| $\mathrm{SO}(n)$ | Matrix exponential | $O(n^3)$，常数更大 | Padé 近似或 scaling-squaring |
| $\mathrm{SE}(3)$ | 分解 retraction（旋转 + 平移分别处理） | $O(1)$ | GTSAM/Ceres 默认 |
| $\mathrm{SE}(3)$ | Group exponential（Rodrigues 闭式） | $O(1)$，常数更大 | 闭式可用但含三角函数 |

**核心洞察**：retraction 的选择直接影响每步迭代的计算量，在大规模 PGO（数万 pose）中累积差异显著。

##### 1.3 Vector transport ⭐⭐⭐

- **定义**：两个切空间 $T_xM$ 与 $T_{R_x(\xi)}M$ 之间的线性映射，作为 parallel transport 的一阶近似
- **为什么需要**：共轭梯度法需要将上一步梯度"搬运"到新切空间做 $\beta$ 系数计算；动量方法同理
- Vector transport 与 retraction 的相容性条件（differentiated retraction 自然诱导 vector transport）
- 实践中常用的三种方式：投影型、differentiated retraction 型、parallel transport 型

**资源**：Absil 书 Ch.8；Boumal 书 Ch.10；Absil ICIAM'07 vector transport 报告（perso.uclouvain.be/pa.absil/Talks/ICIAM070717\_oom\_05.pdf）

##### 1.4 Riemannian 度量的引入 ⭐⭐

- 流形上定义梯度依赖内积：$\langle \mathrm{grad}\,f(x), \xi \rangle_x = \mathrm{D}f(x)[\xi]$，没有度量就没有梯度
- **Riemannian 度量张量** $g_x: T_xM \times T_xM \to \mathbb{R}$，逐点光滑变化的内积
- **诱导度量 vs 内在度量**：嵌入子流形从环境空间继承度量（如 Stiefel 用 $\mathrm{tr}(\xi^T\eta)$）；商流形需要 horizontal lift 定义度量
- SO(3)/SE(3) 上的 **bi-invariant metric**：$\langle \Omega_1, \Omega_2 \rangle = \mathrm{tr}(\Omega_1^T \Omega_2)$，此度量下李群 exp = Riemannian exp

##### 1.5 Riemannian gradient 与测地线 ⭐⭐

- **Riemannian gradient** 的定义：欧氏梯度在切空间上的正交投影 $\mathrm{grad}\,f(x) = \mathrm{Proj}_{T_xM}(\nabla f(x))$（对嵌入子流形）
- 为什么 $\nabla f$ 不等于 $\mathrm{grad}\,f$：前者可能有法向分量，后者严格在切空间内
- Exponential map 作为测地线 retraction：$\mathrm{Exp}_x(\xi)$ = 从 $x$ 出发沿 $\xi$ 方向走单位时间的测地线终点
- **关键区分**：李群 $\exp$（代数定义，$e^{tX}$ 是单参数子群）vs Riemannian $\mathrm{Exp}$（几何定义，测地线），二者在 bi-invariant metric 下重合，但一般旋量群的 left-invariant metric 下不等

##### 1.6 流形上的一阶优化算法 ⭐⭐

**Riemannian Gradient Descent 完整算法**：
```
输入：流形 M, 目标函数 f, 初始点 x_0, retraction R
for k = 0, 1, 2, ...
    计算 Riemannian 梯度 η_k = grad f(x_k)
    选择步长 α_k（Armijo on manifold）
    更新 x_{k+1} = R_{x_k}(-α_k η_k)
end
```

- **Armijo on manifold**：沿 retraction 曲线 $t \mapsto R_x(-t\,\mathrm{grad}\,f)$ 做回溯线搜索，要求 $f(R_x(-t\,\eta)) \leq f(x) - c\,t\,\|\eta\|^2$
- **全局收敛性**：在紧流形上，Armijo RGD 的梯度范数收敛到零；迭代复杂度 $O(1/\varepsilon^2)$ 达到 $\|\mathrm{grad}\,f\| \leq \varepsilon$，与欧氏 GD 的 sharp rate 匹配
- 与欧氏 GD 对比：核心差异在于 (1) 梯度需投影, (2) 更新需 retraction, (3) 步长条件沿 retraction 曲线检验

---

#### 2 进阶章节清单（档位4选学，额外15-25小时） ⭐⭐⭐⭐

| 主题 | 核心内容 | 推荐资源 |
|------|---------|---------|
| 二阶 retraction 与测地线高阶逼近 | 二阶条件的严格定义；对 Newton 方法收敛阶的影响 | Absil 书 Ch.5-6 |
| Riemannian Hessian 的两种定义 | 经由 Levi-Civita connection 或经由 retraction pullback | Boumal 书 Ch.5 |
| Parallel transport vs vector transport | 严格几何定义与数值近似的差异 | Boumal 书 Ch.10 |
| Riemannian Newton 与 Trust-region | Riemannian 截断 Newton-CG；RTR 算法 | Absil 书 Ch.6-7；Boumal 书 Ch.6 |
| 商流形（Stiefel/Grassmann 的商结构） | Horizontal lift、水平空间上的梯度/Hessian | Boumal 书 Ch.9；Absil 书 Ch.3.4 |
| Burer-Monteiro 非凸 landscape 理论 | 为什么 SE-Sync 能达到全局最优——smooth SDP 无杂散局部极小 | Boumal, Voroninski, Bandeira (NIPS 2016) |
| Non-smooth Riemannian optimization | RL policy on SO(3) 的 sub-gradient 方法 | 近年 NeurIPS/ICML 论文 |
| Riemannian SGD 及其 ML 应用 | 收敛分析；超双曲嵌入的训练 | Bonnabel (2013)；Bécigneul & Ganea (ICLR 2019) |

---

#### 3 核心教材深度指南 ⭐

##### 3.1 Boumal《An Introduction to Optimization on Smooth Manifolds》(2023) ⭐

**定位**：当代流形优化的最佳入门教材，无需几何或优化先修。

| 项目 | 详情 |
|------|------|
| 出版 | Cambridge University Press, 2023 |
| **免费 PDF** | nicolasboumal.net/book（预出版版，含 errata 批注） |
| 配套视频 | **EPFL MATH-512**（2023 春），14 周 $\times$ 90 分钟完整录像，附习题解答 |
| edX 课程 | 前 6 周内容，edx.org 可注册旁听 |
| Bilibili | **BV1hHFJe8E7z**，66 集完整搬运，约 3100 播放 |
| 档位3 必读 | Ch.1-4（几何基础 + 一阶算法）+ Ch.7 选读（球面、Stiefel、SO(n) 的例子） |
| 档位4 追加 | Ch.5-6（二阶几何 + Newton/TR）+ Ch.9（商流形）+ Ch.10-11（transport, 测地凸性） |
| 难度 | ★★★☆☆ 教学友好，自包含；每章末有习题 |

##### 3.2 Absil-Mahony-Sepulchre《Optimization Algorithms on Matrix Manifolds》(2008) ⭐⭐

**定位**：流形优化的奠基之作，理论严谨度更高，更偏数学。

| 项目 | 详情 |
|------|------|
| **免费全文** | press.princeton.edu/absil（Princeton University Press 官方，各章 PDF 独立下载） |
| 配套网站 | sites.uclouvain.be/absil/amsbook/（含报告幻灯片、Grenoble'08 暑期学校材料） |
| 档位3 必读 | Ch.3（一阶几何）+ Ch.4（线搜索 + retraction 定义 Definition 4.1.1） |
| 档位4 追加 | Ch.5-7（二阶几何、Newton、Trust-region）+ Ch.8（超线性算法） |
| 难度 | ★★★★☆ 比 Boumal 更抽象，适合参考而非首次学习 |
| 与 Boumal 的关系 | Boumal 书是其"现代化教学版"，读 Boumal 为主、Absil 为辅是最优路径 |

##### 3.3 Nocedal & Wright《Numerical Optimization》（欧氏对照） ⭐

流形上每种算法都有欧氏对应物。对照阅读可加深理解：**Ch.3（线搜索 → Riemannian Armijo）**、**Ch.4（Trust-region → Riemannian TR）**、Ch.5（CG → Riemannian CG + vector transport）、Ch.6（BFGS → Riemannian BFGS）。

---

#### 4 关键定理清单 ⭐⭐

| # | 定理 | 档位 | 来源 |
|---|------|------|------|
| 1 | 一阶 retraction 充分性：一阶 retraction 保证 RGD 的一阶收敛性 | 3 | Absil Ch.4 |
| 2 | Riemannian Armijo 线搜索的全局收敛定理 | 3 | Boumal Ch.4 Thm 4.4 |
| 3 | RGD 迭代复杂度 $O(1/\varepsilon^2)$（达到 $\varepsilon$-一阶临界点） | 3 | Boumal, Absil, Cartis (IMA J. 2019) |
| 4 | 嵌入子流形的 Riemannian 梯度 = 欧氏梯度的正交投影 | 3 | Boumal Ch.3 |
| 5 | Riemannian Trust-region 的全局收敛 + 超线性局部收敛 | 4 | Absil Ch.7 |
| 6 | 二阶 retraction 的误差阶定理（与测地线的 $O(\|\xi\|^3)$ 偏差） | 4 | Absil Ch.5 |
| 7 | 商流形上的水平梯度定理 | 4 | Boumal Ch.9 |
| 8 | Burer-Monteiro 无杂散局部极小（smooth SDP 的全局最优性） | 4 | Boumal, Voroninski, Bandeira (NIPS 2016) |

---

#### 5 关键论文清单 ⭐⭐

**奠基论文**

- Absil, Mahony, Sepulchre.《Optimization Algorithms on Matrix Manifolds》Princeton, 2008. Ch.4 引入 retraction 框架。免费全文在线
- Absil, Malick. "Projection-like Retractions on Matrix Manifolds." SIAM J. Optim., 2012. 系统发展投影型 retraction 构造理论
- Edelman, Arias, Smith. "The Geometry of Algorithms with Orthogonality Constraints." SIAM J. Matrix Anal. Appl., 1998. Stiefel/Grassmann 几何的开山论文，~2700+ 引用。arXiv: physics/9806030

**SE-Sync 系列（机器人社区最关键应用）**

- Rosen, Carlone, Bandeira, Leonard. "SE-Sync: A Certifiably Correct Algorithm for Synchronization over the Special Euclidean Group." IJRR, 38(2-3):95-125, 2019. arXiv: 1612.07386. **WAFR 2016 最佳论文**
- Dellaert, Rosen 等. "Shonan Rotation Averaging." ECCV 2020. 旋转平均的全局最优算法

**Burer-Monteiro 理论基础**

- Boumal, Voroninski, Bandeira. "The Non-convex Burer-Monteiro Approach Works on Smooth Semidefinite Programs." NIPS 2016. arXiv: 1606.04970. SE-Sync 的理论基石
- 期刊扩展版：Comm. Pure Appl. Math., 2020

**机器学习中的流形优化**

- Bonnabel. "Stochastic Gradient Descent on Riemannian Manifolds." IEEE Trans. Autom. Control, 2013. Riemannian SGD 奠基。arXiv: 1111.5280
- Bécigneul, Ganea. "Riemannian Adaptive Optimization Methods." ICLR 2019. arXiv: 1810.00760. Riemannian Adam/Adagrad
- Boumal, Absil, Cartis. "Global Rates of Convergence for Nonconvex Optimization on Manifolds." IMA J. Numer. Anal., 2019. 建立 $O(1/\varepsilon^2)$ sharp rate。arXiv: 1605.08101

**综述**

- Hu, Liu, Wen, Yuan. "A Brief Introduction to Manifold Optimization." J. Oper. Res. Soc. China, 2020.（袁亚湘院士团队，中文社区权威入口）
- Fei et al. "A Survey of Geometric Optimization for Deep Learning." ACM Computing Surveys, 2025. arXiv: 2302.08210

---

#### 6 软件库与工具 ⭐

##### 研究工具

| 库 | 语言 | 定位 | 与本专题的关系 | GitHub / 网址 |
|---|------|------|-------------|------|
| **Manopt** v8.0 | MATLAB | 流形优化参考实现 | 每个流形提供多种 retraction 选择，`egrad2rgrad` 接口 | manopt.org |
| **Pymanopt** | Python | Manopt 的 Python 移植 | 支持 NumPy/JAX/PyTorch 自动微分后端 | github.com/pymanopt/pymanopt |
| **Geomstats** v2.8 | Python | 几何统计 + ML on manifolds | 提供 exp/log/parallel transport，scikit-learn 兼容 API | github.com/geomstats/geomstats |
| **Geoopt** v0.5 | PyTorch | 几何深度学习 | `retr()` 与 `expmap()` 分离设计；`RiemannianAdam` 优化器 | github.com/geoopt/geoopt |
| **ROPTLIB** | C++ | 高性能流形优化 | 显式 `Retraction()` / `DiffRetraction()` 接口，含等距性验证工具 | github.com/whuang08/ROPTLIB |

##### 机器人工程工具

| 库 | 语言 | retraction 接口 | 典型用途 |
|---|------|----------------|---------|
| **GTSAM** | C++/Python | `retract()` / `localCoordinates()`，Rot3 支持 Cayley/Expmap 可选 | 因子图 SLAM、iSAM2 |
| **Ceres** ≥2.1 | C++ | `Manifold::Plus()` / `Minus()`（取代旧 LocalParameterization） | 非线性最小二乘、BA |
| **Pinocchio** | C++/Python | `integrate()` / `difference()`（配置空间 Lie 群操作） | 刚体动力学、运动规划 |
| **SE-Sync** | C++/Python | 内部使用 Stiefel 上 Riemannian TNT | 可认证全局最优 PGO |
| **manif** | C++11 | `exp()` / `log()` + 解析 Jacobian，兼容 Ceres | 状态估计、Kalman |
| **smooth** | C++20 | `rplus` / `rminus` Manifold concept | 李群样条、控制 |

---

#### 7 学习资源汇总 ⭐

##### 视频课程（推荐顺序）

1. **Boumal EPFL MATH-512**（2023）— 14 周完整课程，配套习题解答。nicolasboumal.net/book/#lectures。Bilibili 搬运：BV1hHFJe8E7z（66 集）
2. **Simons Institute 1h talk** — 浓缩入门。YouTube: youtube.com/watch?v=UjaoZE0GBpg
3. **SIAM Optimization 2023 Minitutorial** — $2\times90$ 分钟，含 Max-Cut 代码示例
4. **IROS'22 Tutorial "Riemann and Gauss meet Asimov"**（Noémie Jaquier）— 机器人学习视角。Bilibili: BV1WtWae7ErR

##### 中文社区资源

- **知乎**：邓康康「黎曼优化的一点理解」（62 赞，入门首选）；「袁亚湘院士团队流形优化综述」推荐文；「论文阅读：A brief introduction to manifold optimization」（详细数学笔记）；Manopt 使用教程（含代码）
- **CSDN**：「流形优化全网最通俗版本详解」（应用导向，含波束成形示例）
- **北京大学**：文再文「最优化：建模、算法与理论」课程（Bilibili: BV1Kc411i7kJ），教材被 100+ 高校采用，含 Stiefel 流形优化章节
- **中科大**：王作勤微分流形课程讲义 staff.ustc.edu.cn/~wangzuoq/Courses/18F-Manifolds/（几何基础补充）

##### 英文博客与补充资源

- Agustinus Kristiadi: "Optimization and Gradient Descent on Riemannian Manifolds"（从零推导 RGD）
- Andreas Bloch: "Stochastic Gradient Descent on Riemannian Manifolds"（乘积流形示例）
- MIT VNAV Lecture 18-19: Optimization on Manifolds（机器人应用视角笔记）
- **Awesome-Riemannian-Optimization** GitHub 论文集：github.com/andyjm3/Awesome-Riemannian-Optimization

---

#### 8 学习时间预算与节奏 ⭐

| 阶段 | 内容 | 时间 | 建议节奏 |
|------|------|------|---------|
| 档位3 Phase 1 | Retraction 定义 + 常见例子（§1.1-1.2） | 8-10 h | 第 1 周：读 Boumal Ch.3，做球面/Stiefel 上的 retraction 练习 |
| 档位3 Phase 2 | Riemannian 度量 + gradient（§1.4-1.5） | 7-9 h | 第 2 周：读 Boumal Ch.3 后半 + Ch.5 前半，配合 MATH-512 视频 |
| 档位3 Phase 3 | 一阶优化算法 + vector transport（§1.3, 1.6） | 10-12 h | 第 3 周：读 Boumal Ch.4 + Ch.10 选读，用 Pymanopt 实现 RGD |
| 档位4 | 二阶方法 + 商流形 + Burer-Monteiro + RSGD | 15-25 h | 第 4-5 周：Boumal Ch.5-6, 9；读 SE-Sync 和 Burer-Monteiro 论文 |

**总计**：档位3 约 **25-35 小时**（每天 2 小时，约 2-3 周）；档位4 额外 **15-25 小时**。

---

#### 9 自测题目 ⭐⭐

| # | 题目 | 档位 | 考察点 |
|---|------|------|--------|
| 1 | 证明球面 $S^{n-1}$ 上的归一化映射 $R_x(\xi)=(x+\xi)/\|x+\xi\|$ 满足一阶 retraction 的两个条件 | 3 | Retraction 定义的验证 |
| 2 | 在 Stiefel 流形 $\mathrm{St}(k, \mathbb{R}^n)$ 上实现 QR retraction，数值验证 $R_X(0)=X$ 和 $\mathrm{D}R_X(0)=\mathrm{id}$ | 3 | 编程 + 数值微分验证 |
| 3 | 推导球面 $S^{n-1}$ 上 $f(x)=x^TAx$ 的 Riemannian gradient，并实现 Armijo RGD 求最大特征值 | 3 | 投影梯度 + 完整算法实现 |
| 4 | 对比实现 SO(3) 上 Cayley map 和 Rodrigues 公式的计算效率（FLOP 计数 + wall-clock） | 3 | Retraction 成本的实际感知 |
| 5 | 实现简化版 rotation averaging：给定 $n$ 个相对旋转测量 $\tilde{R}_{ij}$，用 Riemannian GD 在 $\mathrm{SO}(3)^n$ 上最小化 $\sum \|R_i^T R_j - \tilde{R}_{ij}\|_F^2$ | 4 | SE-Sync 的简化版，综合检验 |

---

#### 10 与后续专题的桥梁 ⭐

本专题在整个路线图中起承上启下的枢纽作用。**向后看**：专题1提供的切空间和光滑映射语言是 retraction 定义的地基。**向前看**：

- **→ 专题3（李群与李代数）**：建立"exp 只是 retraction 的特例"后，李群专题可聚焦于群结构本身（伴随表示、BCH 公式），而非将 exp 神秘化
- **→ 专题4（雅可比与微分）**：retraction 的微分 $\mathrm{D}R_x(\xi)$ 正是流形版雅可比；differentiated retraction 自然诱导 vector transport
- **→ 专题6（等变理论）**：商流形的 horizontal lift、对称性在 retraction 设计中的角色（如 Grassmann 作为 Stiefel 的商）
- **→ 第二批（凸优化/非线性优化）**：流形优化是非凸优化的重要分支；Armijo 线搜索和 trust-region 的流形版是欧氏版的自然推广
- **→ 第五批（SE-Sync/certifiable SLAM）**：Burer-Monteiro + Riemannian Staircase 的完整数学基础在本专题建立

---

#### 11 常见陷阱 ⭐

**陷阱1：把 RGD 理解为"先梯度下降再投影"。** 正确理解是：先在切空间计算 Riemannian gradient（已经是投影后的），然后经 retraction 回到流形。顺序和概念都不同于"project-after-step"。

**陷阱2：忽视 vector transport 在动量方法中的必要性。** 欧氏空间中 $v_{k+1}=\beta v_k + \nabla f(x_{k+1})$ 的 $v_k$ 和 $\nabla f(x_{k+1})$ 在同一空间；流形上不然，必须将 $v_k$ transport 到新切空间。

**陷阱3：混淆李群 exp 和 Riemannian Exp。** 李群 $\exp(tX)$ 定义为单参数子群生成元；Riemannian $\mathrm{Exp}_x(\xi)$ 定义为测地线终点。二者仅在 bi-invariant metric 下重合。在 left-invariant metric 下（如机器人中常用的 SE(3) 度量），两者不同。

**陷阱4：误以为 retraction 必须是 exp。** 这是最常见的误解。一阶优化只需一阶 retraction，QR/polar/Cayley 往往更快且数值更稳定。

**陷阱5：不理解商流形上的优化需要 horizontal lift。** Grassmann 流形是 Stiefel 的商；直接在 Stiefel 上做 RGD 会沿 fiber 方向浪费计算，需要 horizontal lift 约束到水平子空间。

---

### 12. Retraction 的完整教学主线 ⭐

前面的章节给出了 Retraction 的定义、资源和算法清单。

这一节把它们串成一条从动机到工程实现的连续链条。

本专题的核心问题可以写成一句话：

```text
状态必须留在流形上，但优化增量必须在线性空间里求解。
```

这句话同时解释了 Retraction 的必要性、流形优化的算法结构，以及 Ceres/GTSAM/Pinocchio 中 `Plus`、`retract`、`integrate` 这些接口为什么存在。

#### 12.1 前置自测 ⭐

学习本节前，先检查五个问题：

1. 在单位球面 $S^{n-1}$ 上，为什么 $x-\alpha\nabla f(x)$ 通常不再满足单位长度？
2. 为什么梯度必须属于 $T_xM$，而不是任意环境空间方向？
3. 为什么一阶优化只需要 Retraction 的一阶正确性？
4. 为什么 Newton 或 trust-region 更关心二阶 Retraction？
5. 为什么 Ceres 的 `Manifold::Plus(x, delta)` 不是普通加法？

如果第 3 个问题不清楚，很容易把 Retraction 神秘化为"必须使用指数映射"。

这会导致实现过度复杂。

---

### 13. 从欧氏梯度下降的失败开始 ⭐

#### 13.1 欧氏梯度下降隐含什么 ⭐

在 $\mathbb{R}^n$ 中，最简单的优化问题是：

$$
\min_x f(x)
$$

梯度下降写作：

$$
x_{k+1}=x_k-\alpha_k\nabla f(x_k)
$$

这个公式隐含三件事：

| 隐含前提 | 在 $\mathbb{R}^n$ 中为什么成立 | 在流形上为什么危险 |
|----------|-------------------------------|--------------------|
| $x_k$ 和 $\nabla f(x_k)$ 可相加 | 点和向量都用同一坐标空间表示 | 点属于 $M$，速度属于 $T_xM$ |
| 更新后仍合法 | $\mathbb{R}^n$ 对加法封闭 | $M$ 通常不对加法封闭 |
| 梯度方向可全局比较 | 所有切空间天然相同 | 不同点的切空间不同 |

因此欧氏梯度下降不是普适算法。

它是流形优化在最简单流形 $\mathbb{R}^n$ 上的特例。

Retraction 可以类比导航系统中的"贴地飞行"：飞行员根据仪表（梯度）判断方向，先在平坦的仪表平面（切空间）上规划下一步，然后"贴回"地形表面（流形）。仪表显示是线性的，地形是弯曲的，Retraction 正是那个把仪表规划投射回真实地形的过程。类比的边界在于：飞机可以选择贴地飞行也可以离地，但流形优化的状态必须严格停留在流形上。

#### 13.2 单位圆上的反面例子 ⭐

考虑：

$$
M=S^1=\{x\in\mathbb{R}^2\mid \|x\|=1\}
$$

设目标函数为：

$$
f(x)=-a^\top x
$$

它希望 $x$ 朝向给定方向 $a$。

欧氏梯度是：

$$
\nabla f(x)=-a
$$

若直接更新：

$$
x^+=x+\alpha a
$$

一般有：

$$
\|x^+\|^2
=
\|x\|^2+2\alpha a^\top x+\alpha^2\|a\|^2
$$

除非恰好满足特殊条件，否则 $\|x^+\|\neq 1$。

这说明直接欧氏更新破坏约束。

一种自然想法是更新后归一化：

$$
x^+=\frac{x+\alpha a}{\|x+\alpha a\|}
$$

这已经是 Retraction 的雏形。

🧠 **思维陷阱：认为"更新后归一化"就够了**

归一化只是一种特定的 Retraction（球面上的）。对于更复杂的流形（如 Stiefel 流形、Grassmann 流形），不存在简单的"归一化"操作。盲目推广"算完加法再投影回约束集"会遇到投影不唯一、投影计算量大、投影破坏搜索方向等问题。正确的思维方式是：先在切空间中计算合法方向，再用适合该流形的 Retraction 更新。

但还差一步：

方向 $a$ 不一定在切空间。

正确的下降方向应该先投影到：

$$
T_xS^1=\{\xi\in\mathbb{R}^2\mid x^\top\xi=0\}
$$

得到：

$$
\operatorname{grad}f(x)
=
(I-xx^\top)\nabla f(x)
$$

然后更新：

$$
x^+=R_x(-\alpha\operatorname{grad}f(x))
$$

其中：

$$
R_x(\xi)=\frac{x+\xi}{\|x+\xi\|}
$$

这就是流形梯度下降的完整雏形。

> **本质洞察**：Retraction 不是为了"修补"欧氏更新，而是把"局部线性计算"与"全局几何约束"分工清楚。线性代数在切空间里做，状态合法性由 Retraction 保证。

---

### 14. Retraction 的正式定义与每个条件的含义 ⭐⭐

#### 14.1 定义 ⭐⭐

设 $M$ 是光滑流形。

对每个 $x\in M$，Retraction 是一个光滑映射：

$$
R_x:T_xM\to M
$$

满足：

$$
R_x(0_x)=x
$$

以及：

$$
DR_x(0_x)=\operatorname{id}_{T_xM}
$$

这两个条件非常少。

它们正是 Retraction 的力量所在。

Retraction 的两个条件可以类比 Taylor 展开的零阶和一阶：$R_x(0) = x$ 相当于函数在展开点的值正确（零阶），$DR_x(0) = \mathrm{id}$ 相当于斜率正确（一阶）。正如一阶 Taylor 近似对小 $h$ 足够好一样，一阶 Retraction 对小步长的优化迭代也足够好。这不是巧合——背后的数学原因完全一致。

#### 14.2 条件一：零增量不动 ⭐⭐

第一个条件：

$$
R_x(0_x)=x
$$

保证没有优化增量时，状态不变。

如果这个条件不成立，算法会出现荒唐行为：

```text
即使梯度为零，状态也会被更新到另一个点。
```

这会破坏临界点概念。

#### 14.3 条件二：一阶导数是恒等映射 ⭐⭐

第二个条件：

$$
DR_x(0_x)=\operatorname{id}
$$

表示 Retraction 在零附近的一阶行为与切空间自身一致。

用局部坐标理解：

$$
R_x(\xi)=x+\xi+O(\|\xi\|^2)
$$

这说明小增量 $\xi$ 的一阶效果没有被扭曲。

如果一阶导数不是恒等映射，比如：

$$
DR_x(0)=A
$$

则算法以为自己走了 $\xi$，实际一阶走了 $A\xi$。

这会改变下降方向、收敛速率甚至稳定性。

**如果 Retraction 不满足一阶条件会怎样？** 考虑一个"坏"的回退映射 $\tilde{R}_x(\xi) = x + 2\xi + O(\|\xi\|^2)$（一阶导数是 $2I$ 而非 $I$）。算法以为走了步长 $\alpha$，实际一阶走了 $2\alpha$。结果是 Armijo 线搜索的充分下降估计失效，原本应该收敛的步长变得过大，迭代可能发散。更隐蔽的情况是 $DR_x(0) = A$（某个非恒等矩阵），此时下降方向被隐式旋转，梯度下降不再沿最陡下降方向移动。

#### 14.4 为什么一阶条件已经够用 ⭐⭐

一阶梯度下降的收敛性只依赖一阶下降：

$$
f(R_x(-\alpha\eta))
=
f(x)-\alpha\langle \operatorname{grad}f(x),\eta\rangle
+O(\alpha^2)
$$

当 $\eta=\operatorname{grad}f(x)$ 时：

$$
f(R_x(-\alpha\eta))
=
f(x)-\alpha\|\eta\|^2+O(\alpha^2)
$$

只要 $\alpha$ 足够小，一阶负项主导二阶误差。

因此一阶 Retraction 足以保证下降。

这解释了为什么 QR Retraction、归一化 Retraction 能用于一阶优化。

它们不必等于测地线指数映射。

💡 **提示：Retraction 的标准比你想象得低**

很多初学者以为流形优化必须精确沿测地线走。

实际上，一阶算法只要求局部一阶正确。

这就是工程上大量使用廉价 Retraction 的原因。

---

### 15. Exponential Map 与 Retraction 的区别 ⭐⭐

#### 15.1 Riemannian Exp ⭐⭐

若 $M$ 上有 Riemannian 度量，可以定义测地线。

给定：

$$
\xi\in T_xM
$$

令 $\gamma(t)$ 是满足：

$$
\gamma(0)=x,\qquad \dot{\gamma}(0)=\xi
$$

的测地线。

Riemannian 指数映射定义为：

$$
\operatorname{Exp}_x(\xi)=\gamma(1)
$$

它表示：

```text
从 x 出发，以初速度 xi 沿最直的曲线走 1 个单位时间。
```

#### 15.2 李群 Exp ⭐⭐

在李群上，指数映射：

$$
\exp:\mathfrak{g}\to G
$$

来自矩阵指数或单参数子群。

它是代数对象。

它不一定等于 Riemannian Exp。

两者相等需要额外条件，例如 bi-invariant metric。

#### 15.3 Retraction 更一般 ⭐⭐

Retraction 只要求：

$$
R_x(0)=x,\qquad DR_x(0)=I
$$

因此：

$$
\operatorname{Exp}_x(\xi)
$$

是 Retraction，但 Retraction 不必是 Exp。

三者关系可以整理为：

| 映射 | 需要什么结构 | 几何含义 | 计算代价 |
|------|--------------|----------|----------|
| 普通 Retraction | 光滑流形 | 一阶合法更新 | 可很低 |
| Riemannian Exp | Riemannian 度量 | 沿测地线走 | 常较高 |
| 李群 exp | 群结构 | 李代数到群 | 依群而定 |

#### 15.4 反事实：如果坚持使用 Exp 会怎样 ⭐⭐

在 $SO(3)$ 和 $SE(3)$ 中，指数映射有闭式，使用它很合理。

但在 Stiefel 流形：

$$
\operatorname{St}(p,n)=\{X\in\mathbb{R}^{n\times p}\mid X^\top X=I\}
$$

精确测地线 Exp 通常比 QR Retraction 贵。

若每次迭代都用精确 Exp，大规模问题会显著变慢。

而一阶优化的收敛性并不需要这种精确性。

这就是 Retraction 框架的工程价值。

---

### 16. 常见 Retraction 逐个推导 ⭐⭐

#### 16.1 球面归一化 Retraction ⭐⭐

在单位球面：

$$
S^{n-1}=\{x\mid x^\top x=1\}
$$

切空间为：

$$
T_xS^{n-1}=\{\xi\mid x^\top \xi=0\}
$$

定义：

$$
R_x(\xi)=\frac{x+\xi}{\|x+\xi\|}
$$

验证第一个条件：

$$
R_x(0)=\frac{x}{\|x\|}=x
$$

验证第二个条件。

令：

$$
g(t)=R_x(t\xi)=\frac{x+t\xi}{\|x+t\xi\|}
$$

因为 $x^\top\xi=0$：

$$
\|x+t\xi\|^2=1+t^2\|\xi\|^2
$$

所以：

$$
\|x+t\xi\|=1+\frac{1}{2}t^2\|\xi\|^2+O(t^4)
$$

因此：

$$
g(t)
=(x+t\xi)\left(1-\frac{1}{2}t^2\|\xi\|^2+O(t^4)\right)
=x+t\xi+O(t^2)
$$

于是：

$$
g'(0)=\xi
$$

所以 $DR_x(0)=I$。

#### 16.2 SO(3) 上的指数 Retraction ⭐⭐

对 $R\in SO(3)$，右扰动更新：

$$
R_R(\delta\theta)=R\exp(\delta\theta^\wedge)
$$

零增量：

$$
R_R(0)=R\exp(0)=R
$$

一阶展开：

$$
\exp(\delta\theta^\wedge)
=I+\delta\theta^\wedge+O(\|\delta\theta\|^2)
$$

所以：

$$
R_R(\delta\theta)
=R+R\delta\theta^\wedge+O(\|\delta\theta\|^2)
$$

这正好落在：

$$
T_RSO(3)=\{R\Omega\mid \Omega^\top=-\Omega\}
$$

的一阶方向中。

#### 16.3 SO(3) 上的 Cayley Retraction ⭐⭐⭐

Cayley 映射定义为：

$$
\operatorname{Cay}(\Omega)
=(I-\frac{1}{2}\Omega)^{-1}(I+\frac{1}{2}\Omega)
$$

其中 $\Omega^\top=-\Omega$。

当 $I-\frac{1}{2}\Omega$ 可逆时，$\operatorname{Cay}(\Omega)\in SO(3)$。

小量展开：

$$
(I-\frac{1}{2}\Omega)^{-1}
=I+\frac{1}{2}\Omega+\frac{1}{4}\Omega^2+\cdots
$$

因此：

$$
\operatorname{Cay}(\Omega)
=
\left(I+\frac{1}{2}\Omega+O(\|\Omega\|^2)\right)
\left(I+\frac{1}{2}\Omega\right)
=I+\Omega+O(\|\Omega\|^2)
$$

所以 Cayley 映射也是一阶 Retraction。

它不覆盖旋转角为 $\pi$ 的情形。

但作为局部优化更新通常足够。

#### 16.4 Stiefel 流形的 QR Retraction ⭐⭐⭐

Stiefel 流形：

$$
\operatorname{St}(p,n)=\{X\in\mathbb{R}^{n\times p}\mid X^\top X=I\}
$$

给定切向量 $\Xi\in T_X\operatorname{St}(p,n)$。

对 $X+\Xi$ 做 thin QR 分解：

$$
X+\Xi=QR
$$

取：

$$
R_X(\Xi)=Q
$$

因为 $Q^\top Q=I$，所以结果在 Stiefel 流形上。

当 $\Xi$ 很小时，QR 分解中的 $Q$ 与 $X+\Xi$ 的差别是二阶量。

因此一阶条件成立。

这就是很多矩阵流形算法选择 QR Retraction 的原因。

> **本质洞察**：Retraction 的多样性不是理论上的冗余，而是工程上的自由度。对于同一个流形，不同的 Retraction 在计算代价、数值稳定性和几何精度之间取不同的权衡。选择 Retraction 就像选择数值积分器——Euler 够用就不必用 Runge-Kutta，QR 够用就不必用矩阵指数。

⚠️ **陷阱：把 QR 的符号约定忘掉**

QR 分解不唯一。

若 $R$ 的对角线允许负号，$Q$ 可能发生列符号翻转。

优化中必须固定约定，通常要求 $R$ 对角线为正。

否则 Retraction 可能不连续。

---

### 17. Riemannian Gradient 的推导 ⭐⭐

#### 17.1 微分先于梯度 ⭐⭐

函数：

$$
f:M\to\mathbb{R}
$$

在点 $x$ 处的微分是：

$$
df_x:T_xM\to\mathbb{R}
$$

它是一个 1-形式。

梯度是切向量：

$$
\operatorname{grad}f(x)\in T_xM
$$

要从 $df_x$ 得到梯度，需要内积：

$$
\langle\cdot,\cdot\rangle_x
$$

定义为：

$$
\langle \operatorname{grad}f(x),\eta\rangle_x
=
df_x[\eta]
$$

这说明梯度依赖 Riemannian 度量。

Riemannian gradient 的投影公式可以类比受约束系统中的力分解：一个物体被约束在光滑曲面上运动（如珠子在铁丝上），重力可以分解为沿曲面的分力（切向分量，真正让物体运动）和垂直于曲面的法向分力（被约束力抵消）。Riemannian gradient 就是"沿流形的有效下降力"——法向分量被约束消灭，只留下切向分量推动优化。

#### 17.2 嵌入子流形中的投影公式 ⭐⭐

若 $M$ 嵌入在欧氏空间中，且使用诱导度量。

欧氏梯度为：

$$
\nabla \bar{f}(x)
$$

其中 $\bar{f}$ 是 $f$ 的局部延拓。

Riemannian gradient 是欧氏梯度在切空间上的正交投影：

$$
\operatorname{grad}f(x)
=
\operatorname{Proj}_{T_xM}\nabla \bar{f}(x)
$$

以球面为例：

$$
T_xS^{n-1}=\{\xi\mid x^\top\xi=0\}
$$

投影矩阵为：

$$
P_x=I-xx^\top
$$

因此：

$$
\operatorname{grad}f(x)
=(I-xx^\top)\nabla \bar{f}(x)
$$

#### 17.3 为什么法向分量必须去掉 ⭐⭐

欧氏梯度可以分解为：

$$
\nabla \bar{f}=g_T+g_N
$$

其中：

$$
g_T\in T_xM,\qquad g_N\in (T_xM)^\perp
$$

沿流形上的任何可行小运动 $\eta\in T_xM$，有：

$$
g_N^\top\eta=0
$$

因此法向分量不改变一阶目标值。

它只会把更新推出流形。

所以流形优化中真正有意义的是切向分量。

**如果不去掉法向分量会怎样？** 直接使用欧氏梯度 $\nabla f$ 而不投影到切空间，更新 $x^+ = R_x(-\alpha \nabla f)$ 中的 $\nabla f$ 包含法向分量。Retraction 会把法向分量"吸收"回流形，但这种吸收是非线性的、不可控的——它可能导致步长效率降低（法向分量白白浪费计算资源）、收敛方向偏移（法向分量经 Retraction 的非线性投射后产生意想不到的切向位移），甚至在曲率大的区域引起数值不稳定。

💡 **提示：约束优化中的 Lagrange multiplier 与投影梯度是同一件事的两种表达**

在等式约束 $c(x)=0$ 下，KKT 条件要求梯度落在约束 Jacobian 的行空间。

等价地，Riemannian gradient 为零。

前者从约束优化写法出发，后者从子流形几何出发。

---

### 18. 流形上的线搜索 ⭐⭐

#### 18.1 欧氏 Armijo 条件 ⭐⭐

欧氏空间中，沿下降方向 $\eta$ 做线搜索：

$$
x^+=x+\alpha\eta
$$

Armijo 条件为：

$$
f(x+\alpha\eta)
\le
f(x)+c\alpha\nabla f(x)^\top\eta
$$

若 $\eta=-\nabla f(x)$，右侧为：

$$
f(x)-c\alpha\|\nabla f(x)\|^2
$$

#### 18.2 流形 Armijo 条件 ⭐⭐

在流形上，线变成 Retraction 曲线：

$$
\gamma(\alpha)=R_x(\alpha\eta)
$$

Armijo 条件写为：

$$
f(R_x(\alpha\eta))
\le
f(x)+c\alpha\langle \operatorname{grad}f(x),\eta\rangle_x
$$

如果：

$$
\eta=-\operatorname{grad}f(x)
$$

则：

$$
f(R_x(-\alpha\operatorname{grad}f(x)))
\le
f(x)-c\alpha\|\operatorname{grad}f(x)\|_x^2
$$

这与欧氏形式完全平行。

唯一变化是：

```text
直线 x + alpha eta
被 Retraction 曲线 R_x(alpha eta) 取代。
```

#### 18.3 伪代码 ⭐⭐

```cpp
// Riemannian gradient descent on a manifold.
// 这里的 State 可以是 SO3、SE3、球面点或其他流形状态。
State x = initial_state;

for (int iter = 0; iter < max_iter; ++iter) {
  Tangent g = riemannian_gradient(x);  // 已经投影到 T_x M
  double alpha = initial_step;

  while (true) {
    State x_trial = retract(x, -alpha * g);  // 通过 Retraction 回到流形
    double lhs = cost(x_trial);
    double rhs = cost(x) - c * alpha * inner(x, g, g);

    if (lhs <= rhs) {
      x = x_trial;
      break;
    }

    alpha *= 0.5;  // 回溯线搜索
  }
}
```

⚠️ **编程陷阱：在回溯线搜索中忘记重新计算代价函数**

流形上 `cost(x_trial)` 的计算可能涉及约束检查（如矩阵正交性验证）。如果 Retraction 实现有 bug（返回的点不严格在流形上），代价函数可能返回 NaN 或异常值，但回溯线搜索会静默接受一个"足够下降"的坏点。建议在 debug 模式下加入流形约束残差检查。

这段代码展示了流形优化的三层结构：

1. `riemannian_gradient` 负责切空间方向。
2. `retract` 负责合法状态更新。
3. `inner` 负责度量。

---

### 19. Gauss-Newton 与 LM 在流形上怎么变 ⭐⭐⭐

#### 19.1 最小二乘问题 ⭐⭐⭐

机器人后端优化常写为：

$$
\min_{x\in M}\frac{1}{2}\|r(x)\|^2
$$

其中残差：

$$
r:M\to\mathbb{R}^m
$$

在当前点 $x$ 附近，用 Retraction 建立局部模型：

$$
\hat{r}_x(\xi)=r(R_x(\xi))
$$

这是从切空间 $T_xM$ 到 $\mathbb{R}^m$ 的普通函数。

因此可以对 $\xi=0$ 线性化：

$$
\hat{r}_x(\xi)\approx r(x)+J\xi
$$

其中：

$$
J=D(r\circ R_x)(0)
$$

#### 19.2 Gauss-Newton 步 ⭐⭐⭐

局部二次模型：

$$
\min_\xi \frac{1}{2}\|r(x)+J\xi\|^2
$$

法方程：

$$
J^\top J\xi=-J^\top r
$$

求得 $\xi^\star$ 后更新：

$$
x^+=R_x(\xi^\star)
$$

这说明：

```text
法方程永远在切空间里求解；
状态更新永远通过 Retraction 回到流形。
```

#### 19.3 LM 阻尼 ⭐⭐⭐

Levenberg-Marquardt 在切空间里加阻尼：

$$
(J^\top J+\lambda I)\xi=-J^\top r
$$

这里的 $I$ 是切空间坐标中的单位矩阵。

如果切空间不同维度或不同尺度混合，例如 $SE(3)$ 的平移与旋转，就需要注意尺度权重。

在位姿优化中，旋转单位是 rad，平移单位是 m。

直接使用同一个阻尼可能导致平移和旋转权重不合理。

⚠️ **陷阱：把环境空间维度当成切空间维度**

单位四元数存储为 4 维。

但 $SO(3)$ 的切空间是 3 维。

Ceres 中四元数参数块大小是 4，局部增量大小是 3。

如果把 Jacobian 写成对 4 维自由变量求导，再不处理单位约束，就会得到错误的优化问题。

**如果不区分环境维度和切空间维度会怎样？** 对四元数直接构造 $4 \times 4$ 的 $J^\top J$ 矩阵并求解 4 维增量，看似可行但有两个严重问题：(1) 法方程变成奇异或接近奇异的——因为有一个方向（四元数范数方向）被约束锁死，该方向的 Hessian 信息缺失；(2) 求得的增量包含法向分量，直接加到四元数后范数偏离 1，需要额外归一化步骤。正确做法是在 3 维切空间中求解增量，Ceres 的 `Manifold` 接口正是自动处理这种维度缩减。

---

### 20. 工程接口对照 ⭐

#### 20.1 Ceres ⭐

Ceres 的 `Manifold` 接口核心是：

```cpp
class Manifold {
 public:
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;

  virtual bool Minus(const double* y,
                     const double* x,
                     double* y_minus_x) const = 0;
};
```

这里：

| 接口 | 几何含义 |
|------|----------|
| `Plus(x, delta)` | $R_x(\delta)$ |
| `Minus(y, x)` | $R_x^{-1}(y)$ 的局部近似 |
| `AmbientSize()` | 存储维度 |
| `TangentSize()` | 切空间维度 |

四元数的典型情况是：

```text
AmbientSize = 4
TangentSize = 3
```

#### 20.2 GTSAM ⭐

GTSAM 使用：

```text
retract(delta)
localCoordinates(other)
```

几何意义为：

$$
\operatorname{retract}_x(\delta)=R_x(\delta)
$$

以及：

$$
\operatorname{localCoordinates}_x(y)\approx R_x^{-1}(y)
$$

Pose3 的局部坐标是 6 维。

但要特别注意 GTSAM 的切向量排序通常是旋转在前。

#### 20.3 Pinocchio ⭐

Pinocchio 对配置空间提供：

```text
integrate(model, q, v)
difference(model, q0, q1)
```

它们处理的不只是 $SE(3)$，还包括机械臂关节、浮动基、球关节等混合配置空间。

这正是 Retraction 思想在刚体动力学库中的体现。

#### 20.4 Sophus / manif ⭐

Sophus 和 manif 更接近李群接口：

| 库 | 更新风格 | 注意点 |
|----|----------|--------|
| Sophus | 常见为左乘或用户自定义 | 注意 `leftJacobian` 命名 |
| manif | right-plus 清晰 | 与 Solà 论文约定一致 |
| GTSAM | right-plus | Pose3 排序旋转在前 |
| Ceres | 用户自定义 | 必须写清 Plus/Minus |

💡 **提示：接口名不同，本质相同**

`Plus`、`retract`、`integrate`、`boxplus` 都在表达同一件事：

```text
给定流形点 x 和切空间增量 delta，返回新的流形点。
```

真正需要警惕的是扰动方向、切向量排序和局部坐标定义。

---

### 21. 故障排查表 ⭐

| 现象 | 可能原因 | 检查方法 | 修复思路 |
|------|----------|----------|----------|
| 优化一步后状态非法 | 直接做欧氏加法 | 检查约束残差，如 $\|q\|-1$ 或 $R^\top R-I$ | 使用 Retraction 或库的 manifold 接口 |
| 代价下降很慢 | Retraction 一阶正确但尺度不合适 | 检查平移/旋转量纲和步长 | 调整度量或归一化残差 |
| LM 对旋转过度阻尼 | 切空间坐标尺度混合 | 看阻尼矩阵是否同权处理 m 和 rad | 使用尺度矩阵或信息矩阵 |
| 四元数优化发散 | 4 维环境变量和 3 维切空间混淆 | 检查参数块维度与局部维度 | 使用 EigenQuaternionManifold |
| Stiefel 优化列符号跳变 | QR Retraction 约定不连续 | 检查 $R$ 对角符号 | 固定 QR 对角线为正 |
| Riemannian CG 表现异常 | 没有 vector transport | 检查上一轮方向是否直接复用 | 把旧方向搬运到新切空间 |
| 论文公式搬到代码符号相反 | 左/右 Retraction 约定不同 | 写出 $x^+=\operatorname{Exp}(\delta)x$ 还是 $x^+=x\operatorname{Exp}(\delta)$ | 全项目统一约定并加测试 |

---

### 22. 学习中的易错概念辨析 ⭐⭐

⚠️ **陷阱：Retraction 不是 Projection 的同义词**

Projection 通常指从环境空间投影到流形。

Retraction 的定义域是切空间。

对嵌入子流形，有些 Retraction 可由投影构造。

但在抽象流形、商流形或李群中，Retraction 不必表现为环境投影。

⚠️ **陷阱：梯度投影和状态投影不是一回事**

梯度投影把欧氏梯度投到 $T_xM$。

状态投影把环境点拉回 $M$。

一个发生在速度空间，一个发生在状态空间。

混淆二者会导致算法解释混乱。

⚠️ **陷阱：`Minus` 不总是简单的反向 `Plus`**

在非线性流形上，`Minus(y, x)` 通常是局部坐标：

$$
\delta\approx R_x^{-1}(y)
$$

它只在局部唯一。

例如旋转角接近 $\pi$ 时，log 映射可能有分支歧义。

🧠 **本质洞察：流形优化不是把欧氏优化推倒重来**

它保留欧氏优化中最强大的部分：线性化、二次模型、线搜索、信赖域、稀疏法方程。

它只替换一件事：

```text
把全局加法替换为局部切空间增量加 Retraction。
```

这也是它能进入大型工程库的原因。

---

### 练习：Retraction 与流形优化

1. 在草稿纸上证明球面归一化 Retraction 满足 $R_x(0)=x$ 与 $DR_x(0)=I$。
2. 对 $SO(3)$，从 $\exp(\delta^\wedge)=I+\delta^\wedge+O(\|\delta\|^2)$ 推导右乘指数更新的一阶正确性。
3. 写出 $S^{n-1}$ 上 $f(x)=x^\top Ax$ 的 Riemannian gradient，并说明为什么要投影欧氏梯度。
4. 对同一个 $S^2$ 优化问题，比较归一化 Retraction 与精确测地线 Exp 的一步差异阶数。
5. 解释 Ceres 中四元数 `AmbientSize=4`、`TangentSize=3` 的几何原因。
6. 用伪代码写出流形 Gauss-Newton：输入 `residual(x)`、`jacobian_at_zero(x)`、`retract(x, delta)`。
7. 在 Stiefel 流形上说明 QR Retraction 为什么需要固定符号约定。
8. 比较 GTSAM 的 `retract/localCoordinates` 与 Ceres 的 `Plus/Minus`，指出它们在几何上对应什么。

### 跨章综合题

考虑一个位姿图优化问题，变量为 $T_i\in SE(3)$，边残差为：

$$
r_{ij}(T_i,T_j)=\operatorname{Log}(Z_{ij}^{-1}T_i^{-1}T_j)
$$

请完成：

1. 说明变量所在流形是什么。
2. 说明每个优化增量所在切空间维数。
3. 写出一次 Gauss-Newton 迭代中线性化、求解、更新的流程。
4. 解释为什么不能写 $T_i^+=T_i+\delta_i$。
5. 结合专题 4，说明残差 Jacobian 为什么依赖左右扰动约定。
6. 结合专题 5，说明边噪声协方差为什么也定义在切空间中。

---

### 23. Riemannian 共轭梯度与 L-BFGS ⭐⭐⭐

#### 23.1 为什么梯度下降不够：病态问题的困境 ⭐⭐⭐

前面 §18 介绍的 Riemannian 梯度下降（RGD）在理论上有全局收敛保证，但在实际大规模问题中常常**收敛极慢**。原因与欧氏空间中完全一致：当目标函数的 Hessian 条件数 $\kappa = \lambda_{\max}/\lambda_{\min}$ 很大时，梯度方向与 Newton 方向偏差严重，迭代沿"窄谷"反复锯齿震荡。

回顾 Nocedal & Wright《Numerical Optimization》Ch.5 的经典分析：欧氏空间中最速下降的迭代复杂度为 $O(\kappa \log(1/\varepsilon))$，而共轭梯度为 $O(\sqrt{\kappa} \log(1/\varepsilon))$——条件数 $\kappa = 10^4$ 时，CG 快两个数量级。流形上的情况完全平行。

用一个类比来说明：梯度下降就像在山谷中只看脚下最陡的方向走路——如果山谷又长又窄，你会不断在两侧山壁之间来回碰撞，前进极慢。共轭梯度则像一位有记忆的登山者——他记住了上次走过的方向，这次故意选择一个"不重复上次错误"的方向，从而更快穿过窄谷。L-BFGS 更进一步，像一个带笔记本的登山者，记录最近几次的地形曲率信息，据此估计最优前进方向。

#### 23.2 Riemannian 共轭梯度（Riemannian CG） ⭐⭐⭐

**核心思想**：在切空间中构造共轭方向，用 vector transport 把上一步方向搬运到新切空间。

欧氏共轭梯度的更新公式为：

$$
d_k = -\nabla f(x_k) + \beta_k d_{k-1}
$$

其中 $\beta_k$ 的选择决定了具体算法变体。流形上的困难在于：$\nabla f(x_k) \in T_{x_k}M$ 与 $d_{k-1} \in T_{x_{k-1}}M$ **不在同一个切空间**，不能直接相加。

解决方案是引入 **vector transport**（回顾 §1.3）：令 $\mathcal{T}_{x_{k-1} \to x_k}$ 把上一步方向搬运到新切空间。Riemannian CG 的完整更新为：

$$
d_k = -\operatorname{grad} f(x_k) + \beta_k \, \mathcal{T}_{x_{k-1} \to x_k}(d_{k-1})
$$

$$
x_{k+1} = R_{x_k}(\alpha_k d_k)
$$

两种经典的 $\beta_k$ 选择：

| 变体 | $\beta_k$ 公式 | 特点 |
|------|---------------|------|
| **Fletcher-Reeves** | $\displaystyle\frac{\|\operatorname{grad} f(x_k)\|^2}{\|\operatorname{grad} f(x_{k-1})\|^2}$ | 实现简单，但可能产生不良搜索方向 |
| **Polak-Ribière** | $\displaystyle\frac{\langle \operatorname{grad} f(x_k),\, \operatorname{grad} f(x_k) - \mathcal{T}({\operatorname{grad} f(x_{k-1})})\rangle}{\|\operatorname{grad} f(x_{k-1})\|^2}$ | 实践中更稳定，梯度变化小时自动退化为梯度下降 |

注意 Polak-Ribière 公式中 $\operatorname{grad} f(x_{k-1})$ 同样需要通过 vector transport 搬运到 $T_{x_k}M$，才能与 $\operatorname{grad} f(x_k)$ 做差。

**如果不做 vector transport 会怎样？** 假设直接把 $d_{k-1}$（属于 $T_{x_{k-1}}M$）当作 $T_{x_k}M$ 中的向量使用——这在欧氏空间中没问题（因为所有切空间同构于 $\mathbb{R}^n$），但在弯曲流形上，$d_{k-1}$ 可能不在 $T_{x_k}M$ 中。用它做 retraction 更新 $R_{x_k}(\alpha d_{k-1})$ 会产生两个问题：(1) 搜索方向不合法，retraction 可能行为异常；(2) 共轭性被破坏，算法退化甚至发散。这就是 §21 故障排查表中"Riemannian CG 表现异常"一项的根本原因。

Sato 与 Iwai（SIAM J. Optim., 2022）建立了 Riemannian CG 方法的统一收敛分析框架，证明在适当的 $\beta_k$ 选择和 Wolfe 条件下，Riemannian CG 具有全局收敛性，且在强凸情况下达到 $O(\sqrt{\kappa} \log(1/\varepsilon))$ 的迭代复杂度。

#### 23.3 Riemannian L-BFGS ⭐⭐⭐⭐

**动机**：Newton 方法需要精确 Hessian，代价太高。L-BFGS 用最近 $m$ 步的梯度差来近似 Hessian 逆，在欧氏空间中是大规模优化的标准选择。

流形上的困难同样是切空间不一致：需要把历史梯度对 $(s_k, y_k)$ 全部 transport 到当前切空间。Riemannian L-BFGS 的关键步骤如下：

1. 计算 Riemannian 梯度 $g_k = \operatorname{grad} f(x_k)$
2. 将存储的 $m$ 对向量 $(s_i, y_i)$（$i = k-m, \ldots, k-1$）全部 transport 到 $T_{x_k}M$
3. 用 two-loop recursion 计算近似 Newton 方向 $H_k g_k$
4. 沿该方向做 Wolfe 线搜索并通过 retraction 更新

其中 $s_k = \mathcal{T}(\alpha_k d_k)$，$y_k = \operatorname{grad} f(x_{k+1}) - \mathcal{T}(\operatorname{grad} f(x_k))$。

Huang, Absil 与 Gallivan（SIAM J. Optim., 2018）发表的"A Riemannian BFGS Method for Nonconvex Optimization Problems"证明了 Riemannian BFGS 在非凸情况下的全局收敛性和在非退化极小值点附近的超线性收敛。

#### 23.4 收敛速率对比 ⭐⭐⭐

| 算法 | 每步代价 | 迭代复杂度 | 存储 | 适用场景 |
|------|---------|-----------|------|---------|
| **RGD（最速下降）** | 1 次梯度 + 1 次 retraction | $O(\kappa/\varepsilon)$ | $O(n)$ | 简单验证、小问题 |
| **Riemannian CG** | 1 次梯度 + 1 次 transport + 1 次 retraction | $O(\sqrt{\kappa}\log(1/\varepsilon))$ | $O(n)$ | 大规模一阶方法首选 |
| **Riemannian L-BFGS** | 1 次梯度 + $m$ 次 transport + 1 次 retraction | 近超线性 | $O(mn)$ | 中等规模、病态问题 |
| **Riemannian Newton** | 1 次 Hessian 求解 + 1 次 retraction | 二阶局部收敛 | $O(n^2)$ | 小规模高精度 |
| **Riemannian Trust-Region** | 截断 CG 近似 Hessian | 超线性局部收敛 | $O(n)$ | 综合性能最优 |

> **本质洞察**：流形优化的算法谱系与欧氏优化完全平行——GD、CG、L-BFGS、Newton、Trust-Region 各有流形版。差别仅在于每步需要 retraction（替代加法）和 vector transport（替代向量直接复用）。理解了欧氏版本，流形版本只需加上这两个"几何税"。

#### 23.5 Pymanopt 代码示例 ⭐⭐⭐

```python
import pymanopt
from pymanopt.manifolds import Sphere
from pymanopt.optimizers import ConjugateGradient

# 球面上的 Rayleigh 商优化（求最大特征值）
n = 100
A = np.random.randn(n, n)
A = (A + A.T) / 2  # 对称矩阵

manifold = Sphere(n)  # S^{n-1}

@pymanopt.function.numpy(manifold)
def cost(x):
    return -x @ A @ x  # 最小化负 Rayleigh 商 = 最大化特征值

problem = pymanopt.Problem(manifold, cost)

# Riemannian CG 求解，Pymanopt 自动处理梯度投影和 vector transport
optimizer = ConjugateGradient(verbosity=2)
result = optimizer.run(problem)
print(f"最大特征值: {-result.cost:.6f}")
```

**参考文献**：Absil, Mahony, Sepulchre, *Optimization Algorithms on Matrix Manifolds*, Princeton University Press, 2008. Boumal, *An Introduction to Optimization on Smooth Manifolds*, Cambridge University Press, 2023, Ch.8-10.

⚠️ **编程陷阱：Pymanopt 中忘记指定 retraction 类型**

Pymanopt 对大多数流形有默认 retraction（如 Sphere 用归一化、Stiefel 用 QR）。但如果研究 retraction 选择对收敛的影响，需要显式设置。不同 retraction 可能导致相同 CG 算法在迭代次数上差 2-3 倍——原因不是收敛理论变了，而是 retraction 曲线与测地线的偏差影响了线搜索的效率。

💡 **概念误区：认为 Riemannian CG 比 RGD "总是更好"**

- 错误想法：CG 的理论收敛速率更快，所以应该总是用 CG。
- 实际情况：CG 的优势体现在**病态问题**（$\kappa \gg 1$）上。对条件良好的问题（$\kappa \approx 1$），CG 的额外 vector transport 计算反而是浪费。此外，CG 对线搜索参数更敏感，在非光滑或高噪声目标函数上可能表现不如简单的 RGD。
- 正确做法：先用 RGD 跑一个 baseline，观察收敛曲线。如果出现明显锯齿或收敛停滞，再换 CG 或 L-BFGS。

---

### 24. 测地凸性与全局最优保证 ⭐⭐⭐⭐

#### 24.1 从欧氏凸性到测地凸性 ⭐⭐⭐⭐

在欧氏空间中，凸函数的定义依赖直线段：

$$
f(\lambda x + (1-\lambda)y) \le \lambda f(x) + (1-\lambda)f(y)
$$

在流形上没有"直线段"，但有**测地线**。测地凸性（geodesic convexity）把直线段替换为测地线段：

$$
f(\gamma(\lambda)) \le \lambda f(\gamma(1)) + (1-\lambda)f(\gamma(0))
$$

其中 $\gamma$ 是从 $\gamma(0)=x$ 到 $\gamma(1)=y$ 的最短测地线。

**为什么这个定义重要？** 欧氏凸优化的所有美好性质——局部极小即全局极小、梯度下降全局收敛到最优解——都可以推广到测地凸函数上。但流形上的函数往往不是全局测地凸的（例如 $SO(3)$ 上的函数只在半径 $\pi$ 的测地球内有凸性保证），因此需要仔细分析凸性成立的区域。

测地凸性与欧氏凸性的关系可以这样理解：欧氏空间是曲率为零的平坦流形，测地线就是直线，测地凸性退化为欧氏凸性。正曲率流形（如球面）上，测地线"弯曲"程度更大，凸性区域更小；负曲率流形（如双曲空间）上，测地线发散更快，但凸性条件可能更容易满足。

Boumal 书 Ch.11 对测地凸性有完整的处理，包括强测地凸性（geodesic strong convexity）和其对收敛速率的改善。

#### 24.2 旋转平均的测地凸性 ⭐⭐⭐⭐

旋转平均（rotation averaging）问题是：给定一组相对旋转测量 $\tilde{R}_{ij}$，求绝对旋转 $R_i \in SO(3)$ 使得

$$
\min_{R_1,\ldots,R_n \in SO(3)} \sum_{(i,j)\in\mathcal{E}} w_{ij} \, d^2(R_i^{-1}R_j, \tilde{R}_{ij})
$$

其中 $d(\cdot,\cdot)$ 是 $SO(3)$ 上的测地距离 $d(R_1,R_2) = \|\operatorname{Log}(R_1^{-1}R_2)\|$。

**关键结果**（Hartley et al., IJCV 2013；Boumal 2020）：当所有噪声旋转 $\tilde{R}_{ij}^{-1} R_i^{-1} R_j$ 的测地距离小于 $\pi/2$ 时，上述目标函数在真实解的测地球内是**测地凸的**。这意味着任何从该球内出发的 Riemannian 梯度下降都能收敛到全局最优。

> **本质洞察**：旋转平均的测地凸性告诉我们，SLAM 后端中的旋转优化在"噪声不太大"的条件下没有伪局部极小值。这解释了为什么 Gauss-Newton 在实践中很少陷入错误解——不是因为运气好，而是因为问题在合理噪声水平下本就是凸的。

**如果噪声超过凸性半径会怎样？** 当相对旋转测量的误差超过 $\pi/4$（约 45 度）时，测地凸性开始失效，目标函数可能出现伪局部极小值。此时传统迭代方法（Gauss-Newton、LM）可能收敛到错误解。这正是 SE-Sync 和 Shonan Rotation Averaging 的核心应用场景——它们通过 SDP 松弛提供全局最优性证书，即使在凸性失效的区域也能识别全局最优。

#### 24.3 SE-Sync 中的对偶证书 ⭐⭐⭐⭐

SE-Sync（Rosen, Carlone, Bandeira, Leonard, IJRR 2019）的全局最优保证来自 SDP 对偶理论：

1. **原始问题**：在 $SO(d)^n$ 上最小化二次代价（位姿图 MLE）
2. **SDP 松弛**：将正交约束放松为半正定约束 $X \succeq 0$
3. **对偶问题**：构造一个对偶可行解
4. **松弛精确性条件**：当原始解 $X^\star$ 的秩等于 $d$ 时，SDP 松弛是 tight 的

Rosen 等人证明，当测量噪声低于某个阈值时，SDP 松弛精确成立，此时可以从 SDP 解直接恢复全局最优旋转，并给出**可验证的最优性证书**——对偶间隙（duality gap）为零。

这与测地凸性的联系是：SDP 松弛的精确性条件本质上等价于对偶问题在测地凸区域内可行——当噪声足够小使得凸性成立时，松弛自动 tight。

**Shonan Rotation Averaging**（Dellaert, Rosen 等, ECCV 2020）提供了一种优雅的实现策略："Riemannian Staircase"——从 $SO(3)$ 出发，逐步提升到 $SO(p)$（$p > 3$），在更高维空间中用 Riemannian trust-region 求解，直到验证全局最优。这个过程每一步都在 Stiefel 流形上做优化，直接使用本专题 §16.4 的 QR retraction 和 §23 的 Riemannian CG/trust-region。

#### 24.4 与可认证感知的连接 ⭐⭐⭐⭐

测地凸性和 SDP 松弛理论为整个**可认证感知（certifiable perception）**领域提供了数学基础。这些方法不仅用于旋转平均，还扩展到：

- **点云配准**：TEASER++（Yang, Shi, Carlone, T-RO 2021）在 99% 离群值下仍可全局最优
- **相对位姿估计**：GNC（Yang 等, RA-L 2020, ICRA 最佳论文）通过 graduated non-convexity 逐步逼近鲁棒代价
- **多机器人 SLAM**：分布式 certifiable 方法

这些方法的共同数学工具正是本专题建立的 retraction、Riemannian trust-region 和 Stiefel 流形优化。

⚠️ **概念误区：认为"可认证"等于"一定正确"**

- 错误想法：SE-Sync 给出的证书说明解一定是物理真实的。
- 实际上：证书只保证解是**该数学模型下的全局最优**。如果数据关联错误（把 A 楼的特征匹配到 B 楼）、噪声模型不准确（假设高斯但实际有离群值）、或外参标定有偏差，全局最优解仍然是错的——只是在错误模型下最优而已。
- 正确理解：可认证方法解决的是"给定正确模型，能否高效找到全局最优"的**计算问题**，不是"模型是否正确"的**建模问题**。

---

### 25. 实战案例：旋转平均与 SE-Sync ⭐⭐⭐

#### 25.1 问题表述 ⭐⭐⭐

旋转平均是 Structure-from-Motion 和多视角 SLAM 中的基础子问题：给定一个图 $\mathcal{G} = (\mathcal{V}, \mathcal{E})$，其中节点 $i \in \mathcal{V}$ 对应未知的绝对旋转 $R_i \in SO(3)$，边 $(i,j) \in \mathcal{E}$ 对应观测到的相对旋转 $\tilde{R}_{ij} \approx R_i^{-1} R_j$。

代价函数为：

$$
C(R_1,\ldots,R_n) = \sum_{(i,j)\in\mathcal{E}} \|R_i \tilde{R}_{ij} - R_j\|_F^2
$$

这是 $SO(3)^n$ 上的优化问题。每个 $R_i$ 必须严格满足正交约束 $R_i^\top R_i = I$，$\det R_i = 1$。

#### 25.2 Retraction 选择如何影响收敛 ⭐⭐⭐

在 $SO(3)$ 上做梯度下降时，每步需要一次 retraction。回顾 §16 中 SO(3) 的两种 retraction：

| Retraction | 计算 | 覆盖范围 | 数值行为 |
|-----------|------|---------|---------|
| Rodrigues Exp | $R \exp(\delta^\wedge)$，含 $\sin/\cos$ | 完整 $SO(3)$ | 精确但含三角函数 |
| Cayley map | $R(I - \frac{1}{2}\delta^\wedge)^{-1}(I + \frac{1}{2}\delta^\wedge)$ | 旋转角 $< \pi$ | 仅需矩阵求逆，无三角函数 |

在大规模旋转平均（$n > 10000$ 节点）中，每步迭代需要对所有节点执行 retraction。若使用 Rodrigues Exp，$n = 50000$ 时需要 $50000$ 次 $\sin/\cos$ 计算；Cayley map 仅需 $3 \times 3$ 矩阵求逆（对 SO(3) 有闭式），通常快 1.5-2 倍。由于旋转平均中步长通常很小（远离 $\pi$ 边界），Cayley map 的覆盖范围限制几乎不成问题。

> **本质洞察**：Retraction 的选择是"精度-速度"权衡的工程决策。在大规模问题中，这个决策的累积效应可以决定算法是否能实时运行。对 SLAM 后端优化（通常需要在 100ms 内完成），用 Cayley 替代 Exp 可能是实现实时性的关键一步。

#### 25.3 从旋转平均到完整 PGO ⭐⭐⭐

旋转平均只优化旋转分量。完整的位姿图优化（PGO）还包含平移：

$$
\min_{T_i \in SE(3)} \sum_{(i,j)} \|r_{ij}(T_i, T_j)\|_{\Sigma_{ij}^{-1}}^2
$$

SE-Sync 的策略是**分步求解**：先用旋转平均求全局最优旋转（利用 SDP 松弛保证全局性），再在已知旋转下用线性最小二乘求平移。这种分步策略之所以可行，是因为旋转平均的全局最优性保证了后续平移求解的基础是正确的。

GTSAM 中实现的接口直接体现了本专题的数学：

```cpp
// GTSAM 中的旋转平均接口
#include <gtsam/sfm/ShonanAveraging.h>

// 构建旋转平均问题
gtsam::ShonanAveraging3 shonan(measurements);

// 从 SO(3)^n 出发，逐步提升到 SO(p)^n
auto result = shonan.run();  // 内部使用 Riemannian trust-region + QR retraction

// 结果包含全局最优旋转和最优性证书
double min_eigenvalue = shonan.computeMinEigenValue(result);
// min_eigenvalue >= 0 意味着 SDP 松弛 tight，解是全局最优
```

这段代码背后的数学正是本专题全部内容的工程落地：retraction（§16）用于流形更新、Riemannian gradient（§17）用于计算下降方向、trust-region（§2 进阶）用于步长控制、Stiefel 流形优化（§16.4 的 QR retraction）用于 Burer-Monteiro 分解。

⚠️ **编程陷阱：Shonan Rotation Averaging 中 $p$ 的初始值设太大**

- 错误做法：直接从 $p = 10$ 开始求解，希望"一步到位"。
- 后果：$SO(10)^n$ 上的优化问题维度是 $SO(3)^n$ 的 $\binom{10}{2}/\binom{3}{2} \approx 15$ 倍。计算量暴增，可能比简单的 Gauss-Newton + 初始化策略还慢。
- 正确做法：Riemannian Staircase 从 $p = 3$ 开始，检查最优性证书。如果证书通过（最小特征值 $\ge 0$），直接返回；否则将 $p$ 加 1 重试。实践中 90% 以上的问题在 $p = 3$ 或 $p = 4$ 就通过了。

#### 25.4 练习 ⭐⭐⭐

1. （手推）对两个旋转 $R_1, R_2 \in SO(3)$，写出代价 $\|R_1 \tilde{R}_{12} - R_2\|_F^2$ 关于 $R_1$ 右扰动 $\delta_1$ 的一阶展开。指出 Riemannian gradient 的形式。
2. （编程）用 Pymanopt 实现简单的旋转平均：随机生成 $n = 20$ 个旋转和它们之间的带噪声相对旋转，分别用 RGD 和 CG 求解，比较迭代次数和收敛速度。
3. （思考）SE-Sync 论文中的 SDP 松弛精确性依赖噪声水平。查阅 Rosen et al. (IJRR 2019) 的 Theorem 3，说明精确性条件与测量噪声的定量关系。结合 §24.2 的测地凸性讨论，解释两个结果的联系。

---

### 25bis. Retraction 的数值实现细节 ⭐⭐

#### 25bis.1 数值稳定性：小角度与大角度 ⭐⭐

SO(3) 上的 Rodrigues 指数映射：

$$
\exp(\phi^\wedge) = I + \frac{\sin\theta}{\theta}\phi^\wedge + \frac{1-\cos\theta}{\theta^2}(\phi^\wedge)^2
$$

当 $\theta = \|\phi\| \to 0$ 时，$\sin\theta/\theta$ 和 $(1-\cos\theta)/\theta^2$ 都会出现 0/0 形式。

直接计算会导致数值不稳定。正确做法是使用 Taylor 展开：

$$
\frac{\sin\theta}{\theta} = 1 - \frac{\theta^2}{6} + \frac{\theta^4}{120} - \cdots
$$

$$
\frac{1-\cos\theta}{\theta^2} = \frac{1}{2} - \frac{\theta^2}{24} + \frac{\theta^4}{720} - \cdots
$$

实践中当 $\theta < 10^{-8}$ 时切换到 Taylor 展开（保留到 $\theta^4$ 项），可保证全范围内的数值精度。

类似问题出现在 Log 映射中：

$$
\theta = \arccos\left(\frac{\operatorname{tr}(R)-1}{2}\right)
$$

当 $R$ 接近 $I$ 时，$\operatorname{tr}(R) \approx 3$，$\arccos(1) = 0$，需要用 Taylor 展开替代。

当 $R$ 接近 $-I$（旋转角接近 $\pi$）时，出现另一种退化：轴方向变得不唯一（$\pi$ 绕任何法向轴旋转得到相同矩阵），此时需要特殊处理分支。

| 情况 | $\theta$ 范围 | 数值风险 | 处理策略 |
|------|-------------|----------|----------|
| 接近恒等 | $\theta < 10^{-8}$ | $\sin\theta/\theta$ 的 0/0 | Taylor 展开 |
| 正常范围 | $10^{-8} < \theta < \pi - 10^{-6}$ | 无 | 直接公式 |
| 接近 $\pi$ | $\theta > \pi - 10^{-6}$ | 轴方向退化 | 特征值分解或四元数路径 |

⚠️ **编程陷阱：忽略 Rodrigues 公式的小角度分支**

很多实现（包括某些版本的 OpenCV）在 $\theta \to 0$ 时不做 Taylor 切换，导致在接近零旋转处返回 NaN 或巨大数值。在迭代优化中，优化收敛时 $\delta\phi \to 0$ 恰好触发此分支——如果没有正确处理，会在最后几步迭代中突然发散。

#### 25bis.2 Cayley Map 的优势与限制 ⭐⭐

Cayley retraction 避免了三角函数：

$$
\operatorname{cay}(\Omega) = (I - \tfrac{1}{2}\Omega)^{-1}(I + \tfrac{1}{2}\Omega)
$$

对 SO(3)，$3\times3$ 矩阵求逆有闭式（Cramer 法则），完全避免迭代。

与 Rodrigues 的对比：

$$
\operatorname{cay}(\theta n^\wedge) = I + \frac{4}{4+\theta^2}\left(\theta n^\wedge + \frac{\theta^2}{2}(n^\wedge)^2\right)
$$

$$
\exp(\theta n^\wedge) = I + \sin\theta\, n^\wedge + (1-\cos\theta)(n^\wedge)^2
$$

两者的系数在 $\theta$ 小时非常接近：$4/(4+\theta^2) \approx 1 - \theta^2/4$ vs $\sin\theta/\theta \approx 1 - \theta^2/6$。

但 Cayley map 有覆盖限制：它无法表示旋转角恰好为 $\pi$ 的旋转（此时分母 $I - \frac{1}{2}\Omega$ 奇异）。在优化中步长通常远小于 $\pi$，这个限制几乎不构成实际问题。

**如果优化步长接近 $\pi$ 会怎样？** 这意味着当前估计与最优解相差几乎 $180°$ 旋转。此时算法本身已经有问题——如此大的步长说明初始化极差或问题几何严重非凸。正确做法不是换用 exp retraction，而是改善初始化。

#### 25bis.3 SE(3) 上的分解 Retraction ⭐⭐

SE(3) 上最常用的不是完整的群指数映射，而是分解 retraction：

$$
R_T(\xi) = \begin{pmatrix} R \cdot \exp(\phi^\wedge) & t + R\rho \\ 0 & 1 \end{pmatrix}
$$

其中 $\xi = (\rho, \phi)^\top \in \mathbb{R}^6$。

这不是 SE(3) 的群指数映射——群指数映射中平移项是 $t + V\rho$（$V$ 是与 $\phi$ 相关的矩阵）。分解 retraction 省略了 $V$ 矩阵的计算，把旋转和平移解耦处理。

GTSAM 和 Ceres 默认使用的分解 retraction。误差是 $O(\|\phi\|\|\rho\|)$——当旋转扰动和平移扰动都小时，误差是两者乘积的量级，通常在优化收敛阶段完全可忽略。

> **本质洞察**：完整的 SE(3) 指数映射和分解 retraction 之间的差异，反映了一个更深的数学事实：SE(3) 不是 SO(3) 与 $\mathbb{R}^3$ 的直积，而是半直积。两者在李代数层面有耦合（体现为 $V$ 矩阵），但这个耦合是二阶效应。对一阶优化方法，忽略它完全合法。

---

### 26. Retraction 的收敛性理论 ⭐⭐⭐

#### 26.1 一阶 Retraction 保证一阶收敛 ⭐⭐⭐

Riemannian 梯度下降使用一阶 retraction 时的收敛保证：

设 $f: M \to \mathbb{R}$ 满足 $L$-Lipschitz 梯度（在 Riemannian 意义下），使用固定步长 $\alpha = 1/L$，则：

$$
\min_{k=0,\ldots,K-1} \|\operatorname{grad} f(x_k)\| \le \sqrt{\frac{2L(f(x_0) - f^\star)}{K}}
$$

这与欧氏梯度下降的收敛率 $O(1/\sqrt{K})$ 完全一致。

关键点在于：一阶 retraction 的定义条件 $DR_x(0) = \operatorname{id}$ 保证了局部误差为 $O(\|\xi\|^2)$，而一阶方法已经足够——因为梯度下降本身就是一阶方法，它对步长精度的要求只到一阶。

**如果使用零阶"retraction"（比如简单的归一化投影）会怎样？** 零阶近似的误差是 $O(\|\xi\|)$，与步长本身同阶。这意味着每步更新中，retraction 引入的误差与优化方向的增益同量级——等于一步前进、一步后退，梯度下降可能完全不收敛。这就是为什么 retraction 的一阶条件 $DR_x(0) = \operatorname{id}$ 是不可协商的最低要求。

#### 26.2 二阶 Retraction 与 Newton 方法 ⭐⭐⭐⭐

对于 Riemannian Newton 方法或 trust-region 方法，需要二阶 retraction。

二阶 retraction 的额外条件是与测地线在二阶相切：

$$
R_x(t\xi) = \operatorname{Exp}_x(t\xi) + O(t^3)
$$

Newton 方法需要 Hessian 信息，步长由二阶 Taylor 展开确定。如果 retraction 只有一阶精度，Newton 步的二阶修正会被 retraction 误差"吃掉"，导致丧失二次收敛性。

| 优化方法 | 需要的 Retraction 阶数 | 收敛率 |
|----------|----------------------|--------|
| 梯度下降 | 一阶 | $O(1/K)$（强凸）或 $O(1/\sqrt{K})$ |
| 共轭梯度 | 一阶 + vector transport | $O(1/K^2)$（理想情况） |
| Newton | 二阶 | 局部二次 |
| Trust-region | 二阶 | 局部二次 + 全局线性 |

> **本质洞察**：retraction 的阶数与优化方法的阶数必须匹配。用高阶方法配低阶 retraction 是浪费计算——如同用精密天平称重，但把东西放在晃动的桌面上。反过来，用低阶方法配高阶 retraction 则是过度设计——精确的 retraction 带来的二阶精度在一阶方法中根本用不到。

#### 26.3 全局收敛与测地强凸 ⭐⭐⭐⭐

局部收敛保证通常已够工程使用。但对可认证方法，需要全局结果。

测地强凸函数 $f$ 在 $M$ 上满足：

$$
f(\operatorname{Exp}_x(t\xi)) \ge f(x) + t\langle\operatorname{grad}f(x), \xi\rangle + \frac{\mu}{2}t^2\|\xi\|^2
$$

则梯度下降以线性速率收敛到全局唯一极小值：

$$
f(x_k) - f^\star \le (1 - \mu/L)^k (f(x_0) - f^\star)
$$

测地强凸在曲率为正的流形上成立范围有限——$SO(3)$ 上的二次代价在旋转角小于 $\pi/2$ 时是强凸的，超出此范围可能出现多个局部极小。

---

### 27. Retraction 在深度学习中的应用 ⭐⭐⭐

#### 27.1 正交约束层 ⭐⭐⭐

深度学习中越来越多的架构要求权重满足正交约束：

- 正交 RNN（Arjovsky et al., ICML 2016）：防止梯度爆炸/消失
- Cayley 参数化（Helfrich et al., ICML 2018）：用 Cayley retraction 保持正交性
- 等变网络的 Wigner D 矩阵层（e3nn）

训练过程本质上是 Stiefel 流形上的优化：

$$
\min_{W \in St(k,n)} \mathcal{L}(W; \text{data})
$$

梯度更新使用 retraction 保持约束：

$$
W_{t+1} = R_{W_t}(-\alpha \operatorname{grad}\mathcal{L}(W_t))
$$

常用 QR retraction 或 Cayley retraction（避免计算昂贵的矩阵指数）。

#### 27.2 Geoopt 与 PyTorch 中的流形优化 ⭐⭐⭐

Geoopt 是 PyTorch 生态中的流形优化库，直接实现了本专题的数学：

```python
# Geoopt 实现球面上的优化
import torch
import geoopt

# 定义球面流形
sphere = geoopt.Sphere()

# 创建流形上的参数
x = geoopt.ManifoldParameter(torch.randn(3), manifold=sphere)
# 自动投影到球面

# 定义优化器（Riemannian Adam）
optimizer = geoopt.optim.RiemannianAdam([x], lr=0.01)

# 优化循环
for step in range(100):
    loss = some_function(x)  # 目标函数
    loss.backward()          # 欧氏梯度
    optimizer.step()         # 内部：投影到切空间 + retraction
    optimizer.zero_grad()
```

Geoopt 支持的流形包括：Sphere、Stiefel、Grassmann、SPD matrices、Poincare ball。每个流形都实现了 `retr`（retraction）和 `transp`（vector transport）接口。

**与 Pymanopt 的区别**：Pymanopt 面向传统优化（CG、trust-region），适合中小规模精确求解；Geoopt 面向深度学习（SGD、Adam），适合大规模随机优化。

---

### 28. 本章知识树总结 ⭐

```text
Retraction 与流形优化
├── 动机：欧氏更新离开约束集
│   └── 需要"切空间→流形"的回退映射
├── Retraction 严格定义
│   ├── 一阶条件：R_x(0)=x, DR_x(0)=id
│   ├── 二阶条件：与测地线二阶相切
│   └── 核心直觉：exp 的廉价近似
├── 常见 Retraction 实例
│   ├── 球面：归一化
│   ├── Stiefel：QR / Polar / Cayley
│   ├── SO(n)：Cayley map / matrix exp
│   └── SE(3)：分解 retraction / group exp
├── Vector Transport
│   ├── 把切向量从旧切空间搬到新切空间
│   └── CG/momentum 方法必需
├── Riemannian 度量
│   ├── 定义梯度的前提
│   ├── 诱导度量 vs 内在度量
│   └── bi-invariant metric 的特殊性
├── 一阶优化算法
│   ├── Riemannian GD
│   ├── Riemannian CG
│   └── Riemannian SGD / Adam
├── 二阶优化算法
│   ├── Riemannian Newton
│   ├── Riemannian trust-region
│   └── 商流形上的方法
├── 收敛性理论
│   ├── retraction 阶数与方法阶数匹配
│   ├── 测地强凸 → 线性收敛
│   └── 全局保证 → SDP 松弛
├── 工程落地
│   ├── GTSAM retract 接口
│   ├── Ceres Manifold::Plus
│   ├── Pymanopt / Geoopt
│   └── SE-Sync / Shonan
└── 前沿
    ├── 可认证感知
    ├── 深度学习中的正交约束
    └── 分布式流形优化
```

---

### 29. 本章小结 ⭐

| 核心概念 | 一句话定义 | 工程对应 | 关键公式 |
|----------|-----------|----------|----------|
| Retraction | 切空间到流形的局部光滑映射，一阶近似 exp | GTSAM `retract` | $R_x(0)=x,\; DR_x(0)=\operatorname{id}$ |
| Vector transport | 不同切空间之间的线性搬运 | CG/momentum 中的方向更新 | $\mathcal{T}_\xi: T_xM \to T_{R_x(\xi)}M$ |
| Riemannian gradient | 欧氏梯度在切空间上的正交投影 | 流形上的下降方向 | $\operatorname{grad}f = \operatorname{Proj}_{T_xM}(\nabla f)$ |
| Riemannian Hessian | 切空间上的二阶算子 | Newton 步长 | $\operatorname{Hess}f[\xi] = \nabla_\xi \operatorname{grad}f$ |
| 测地凸性 | 沿测地线的凸性 | 全局最优保证 | 沿 $\gamma$ 的 $f$ 是凸的 |
| QR retraction | Stiefel 上最实用的 retraction | Shonan 内部实现 | $R_X(\xi) = \operatorname{qf}(X+\xi)$ |
| Cayley map | 无需三角函数的 SO(n) retraction | 大规模旋转优化 | $(I-\xi/2)^{-1}(I+\xi/2)$ |

---

### 30. 累积项目：本章新增模块 ⭐

**项目方向**：手写几何验证库

本章新增模块：**球面与 SO(3) 上的 Retraction 实现**

```python
# 验证不同 retraction 的一阶精度
import numpy as np
from scipy.linalg import expm

def so3_hat(omega):
    """向量 -> 反对称矩阵"""
    return np.array([[0, -omega[2], omega[1]],
                     [omega[2], 0, -omega[0]],
                     [-omega[1], omega[0], 0]])

def retraction_exp(R, delta):
    """SO(3) 指数映射 retraction"""
    return R @ expm(so3_hat(delta))

def retraction_cayley(R, delta):
    """SO(3) Cayley retraction"""
    W = so3_hat(delta)
    I = np.eye(3)
    return R @ np.linalg.solve(I - 0.5*W, I + 0.5*W)

# 验证：两种 retraction 在小步长下一致
R = np.eye(3)  # 从单位元出发
delta = np.array([0.01, 0.02, -0.01])  # 小扰动

R_exp = retraction_exp(R, delta)
R_cay = retraction_cayley(R, delta)
print("两种 retraction 差异:", np.linalg.norm(R_exp - R_cay))
# 应为 O(||delta||^3) ~ 1e-7 级别
```

后续模块预告：
- 专题3：实现完整的 SO(3)/SE(3) 李群操作
- 专题4：实现左/右 Jacobian 闭式并验证

---

### 延伸阅读 ⭐

| 资源 | 难度 | 核心价值 |
|------|------|----------|
| Boumal《An Introduction to Optimization on Smooth Manifolds》(2023) | ⭐⭐⭐ | 现代流形优化标准教材，免费在线 |
| Absil, Mahony, Sepulchre《Optimization on Matrix Manifolds》(2008) | ⭐⭐⭐ | 经典参考，Ch.4 retraction 定义开创性 |
| Boumal EPFL MATH-512 视频课程（14周） | ⭐⭐ | 最权威入门，含习题解答 |
| Rosen et al.《SE-Sync》(IJRR 2019) | ⭐⭐⭐⭐ | 可认证 SLAM 的数学核心 |
| Geomstats Python 库 | ⭐⭐ | 流形操作可视化与实验 |
| Geoopt（PyTorch 流形优化） | ⭐⭐⭐ | 深度学习中的流形约束 |
| Pymanopt（Python 流形优化框架） | ⭐⭐ | 支持 CG/trust-region/SD |
| Manopt（Matlab/Julia 原版） | ⭐⭐⭐ | 最早的流形优化工具箱 |
| Manifolds.jl（Julia 流形计算） | ⭐⭐⭐ | 最丰富的 retraction 实现集合 |
| Boumal "Introduction to smooth manifolds" SIAM OP 2023 幻灯片 | ⭐⭐ | 精美可视化，适合快速回顾 |
| Ruda Zhang et al. "A Survey of Numerical Methods on Manifolds" (2020) | ⭐⭐⭐ | retraction 对比的系统综述 |

**SymPy 验证提示**：

流形优化中公式较多，建议用 SymPy 验证以下关键公式：
- Riemannian 梯度 = 欧氏梯度的正交投影（用具体例子如球面）
- QR retraction 确实满足一阶条件（Taylor 展开验证 $DR_X(0) = \operatorname{id}$）
- Cayley map 与 Rodrigues Exp 的 Taylor 展开前几项是否一致

```python
import sympy as sp

# 验证 Cayley 与 Exp 的小角度等价性
theta = sp.Symbol('theta')
# Cayley 系数
cay_coeff = 4 / (4 + theta**2)
# Exp (sin/theta) 系数
exp_coeff = sp.sin(theta) / theta
# Taylor 展开对比
print("Cayley:", sp.series(cay_coeff, theta, 0, n=5))
print("Exp:   ", sp.series(exp_coeff, theta, 0, n=5))
# 差异从 theta^2 阶开始
diff = sp.series(cay_coeff - exp_coeff, theta, 0, n=5)
print("差异:", diff)  # O(theta^2) 项不同 -> retraction 只保证一阶一致
```

这验证了核心定理：Cayley map 是一阶 retraction（与 Exp 在 $O(\|\xi\|)$ 处一致），但不是二阶 retraction（$O(\|\xi\|^2)$ 项不同）。因此 Cayley 适合梯度下降，但不适合 Newton 方法。

| Dellaert et al.《Shonan Rotation Averaging》(ECCV 2020) | ⭐⭐⭐⭐ | Riemannian Staircase 实现 |
| Yang, Carlone《TEASER++》(T-RO 2021) | ⭐⭐⭐⭐ | 可认证点云配准 |
| Sato, Iwai "A Riemannian CG Method" (SIAM J. Optim. 2022) | ⭐⭐⭐⭐ | CG 收敛分析统一框架 |
| Huang, Absil, Gallivan "RBFGS" (SIAM J. Optim. 2018) | ⭐⭐⭐⭐ | Riemannian BFGS 非凸全局收敛 |

---

### 🔧 故障排查手册 ⭐

| 症状 | 可能原因 | 排查步骤 | 相关节 |
|------|----------|----------|--------|
| Retraction 后约束违反增大 | 实现错误或步长过大 | 1.检查 $\|R^\top R - I\|$ 2.缩小步长 3.换用 exp retraction 对照 | §1.1-1.2 |
| 收敛比欧氏慢很多 | 度量选择不当 | 1.检查 Riemannian gradient 方向 2.尝试 preconditioned 度量 3.对比欧氏投影法 | §1.4-1.5 |
| CG 在流形上不收敛 | Vector transport 实现有误 | 1.验证 $\beta$ 系数公式 2.检查 transport 后向量长度是否保持 3.退化到 RGD 确认 | §1.3 |
| Trust-region 内循环不收敛 | Hessian 近似不准 | 1.用有限差分验证 Hessian-vector product 2.检查 retraction 是否二阶 3.增大 trust-region 半径 | §26 |
| 大规模问题内存溢出 | 存储了完整 Hessian | 1.改用 L-BFGS on manifold 2.只存近期梯度差 3.用截断 CG 避免显式 Hessian | §1.6 |
| SE-Sync 报"SDP 不 tight" | 噪声过大或数据关联错误 | 1.检查相对旋转的残差分布 2.降低噪声 3.加入鲁棒核 4.增大 Staircase 的 $p$ | §24-25 |
| Shonan 在 $p=3$ 就卡住 | 初始化太差 | 1.用 chordal relaxation 提供初始值 2.增大 trust-region 初始半径 3.检查图连通性 | §25 |

---

### 31. 跨章综合题 ⭐⭐

**综合题 A**（结合专题1切空间 + 本专题 Retraction）：

在 Stiefel 流形 $V_2(\mathbb{R}^4) = \{X \in \mathbb{R}^{4\times2} \mid X^\top X = I_2\}$ 上，考虑代价函数 $f(X) = \operatorname{tr}(X^\top A X)$，其中 $A$ 是对称正定矩阵。

1. 写出 $V_2(\mathbb{R}^4)$ 的切空间 $T_X V_2 = \{\Delta \mid X^\top\Delta + \Delta^\top X = 0\}$。
2. 计算欧氏梯度 $\nabla f = 2AX$，然后投影到切空间得到 Riemannian 梯度。
3. 用 QR retraction 实现一步梯度下降：$X_{\text{new}} = \operatorname{qf}(X - \alpha\operatorname{grad}f)$。
4. 解释为什么这个问题的全局最优解是 $A$ 的最小两个特征值对应的特征向量（PCA 的几何解释）。
5. 与专题1的正则值定理联系：解释为什么 $V_2(\mathbb{R}^4)$ 是 $\mathbb{R}^{4\times2}$ 中 $f(X) = X^\top X$ 在 $I_2$ 处的正则水平集。

**综合题 B**（结合本专题 + 机器人运动学）：

一个 6-DOF 机械臂的末端位姿在 SE(3) 上，当前末端位姿为 $T_{\text{current}}$，目标位姿为 $T_{\text{target}}$。

1. 定义 SE(3) 上的代价函数 $f(T) = \|\operatorname{Log}(T^{-1} T_{\text{target}})\|^2$。解释为什么用 Log 距离而非 Frobenius 范数 $\|T - T_{\text{target}}\|_F^2$。
2. 这个代价函数的 Riemannian 梯度是什么？（提示：与 Log 映射本身相关。）
3. 如果用右扰动模型 $T' = T \cdot \operatorname{Exp}(\delta\xi)$，一步 Riemannian 梯度下降的更新公式是什么？
4. 讨论当 $T_{\text{current}}$ 与 $T_{\text{target}}$ 之间旋转角接近 $\pi$ 时，优化可能出现的数值问题。
5. 对比使用群指数映射 retraction 和分解 retraction 对 IK 收敛性的影响。

**综合题 C**（Retraction 选择的工程决策）：

在一个有 10000 个位姿节点的 SLAM 后端中，每次迭代对所有节点的旋转分量执行 retraction。

1. 估算使用 Rodrigues Exp（含三角函数）和 Cayley map（仅矩阵求逆）的每步迭代计算量差异。
2. 如果要求 100ms 内完成 10 次迭代，哪种 retraction 更可能满足实时性要求？给出估算依据。
3. 在什么情况下必须切换到 Rodrigues Exp？（提示：考虑步长大小和 Cayley 的覆盖范围限制。）
4. （思考）分解 retraction 和群指数映射 retraction 之间的差异是几阶量？在一次 Gauss-Newton 迭代中，这个差异对最终收敛结果有影响吗？为什么？
5. 比较 Rodrigues 指数映射和 Cayley retraction 在 $\theta = 0.01, 0.1, 1.0, 2.5$ rad 处的 Frobenius 范数差异，验证差异随 $\theta$ 增大的趋势。
6. 证明 QR retraction 在 Stiefel 流形上的一阶正确性：令 $g(t) = \operatorname{qf}(X + t\Xi)$，利用 QR 分解的隐函数定理论证 $g'(0) = \Xi$。

**综合题 D**（结合本专题 Retraction 理论 + 专题3 李群 + 专题4 Jacobian）：

在 VIO 系统中，预积分因子连接两个关键帧 $T_i, T_j \in SE(3)$。优化器使用 Gauss-Newton 迭代，每步需要：(a) 在切空间中求解法方程得到增量 $\delta\xi$；(b) 通过 retraction 更新位姿 $T_i^+ = T_i \cdot \operatorname{Exp}(\delta\xi_i)$。

1. 说明这里使用的是哪种 retraction（群指数映射还是分解 retraction），并写出两者的公式差异。
2. 预积分残差中出现了 $J_r^{-1}$（专题4内容），解释为什么 retraction 的选择会影响残差 Jacobian 中 $J_r^{-1}$ 的精确值。
3. 如果将群指数映射替换为 Cayley retraction，预积分残差的数值结果会改变吗？为什么？（提示：考虑残差本身使用 $\operatorname{Log}$。）

---

### 本章常见误解汇总

| 误解 | 正确理解 |
|------|----------|
| "流形优化必须使用指数映射" | 一阶优化只需一阶 Retraction；QR/polar/Cayley 往往更快且数值更稳定 |
| "Riemannian 梯度下降就是先梯度下降再投影" | 正确顺序是先在切空间计算 Riemannian gradient（已是投影后的），然后经 Retraction 回到流形 |
| "李群 exp 和 Riemannian Exp 是同一回事" | 李群 $\exp(tX)$ 是单参数子群生成元，Riemannian $\mathrm{Exp}_x(\xi)$ 是测地线终点；仅在 bi-invariant metric 下重合 |
| "更新后归一化就是通用的 Retraction" | 归一化只适用于球面；对 Stiefel、Grassmann 等流形，不存在简单的"归一化"操作 |
| "Vector transport 在动量方法中不重要" | 欧氏空间中 $v_k$ 和 $\nabla f(x_{k+1})$ 在同一空间；流形上不然，必须将 $v_k$ transport 到新切空间 |
| "Retraction 的二阶正确性对所有算法都必要" | 一阶 Retraction 对梯度下降已经足够；只有 Newton/trust-region 等二阶方法才需要二阶 Retraction |
| "商流形上可以直接做梯度下降" | 需要 horizontal lift 约束到水平子空间，否则会沿 fiber 方向浪费计算 |

---

#### 总结

Retraction 理论是将欧氏优化算法"提升"到流形上的数学桥梁。**对机器人工程师而言，这不是抽象的纯数学——GTSAM 的 `retract` 接口、Ceres 的 `Manifold::Plus`、SE-Sync 的可认证全局最优，底层都是 retraction 在工作。** 学习路径建议以 Boumal 书 + EPFL 视频为主线（Bilibili 有完整搬运），Absil 书作参考，配合 Pymanopt/Geoopt 的编程实践。档位3 聚焦一阶 retraction 定义、常见例子、Riemannian gradient 和 RGD 算法，确保能独立实现球面和 SO(3) 上的优化；档位4 扩展到二阶方法、商流形和 Burer-Monteiro 理论，为理解 SE-Sync 的全局最优性证明打下基础。完成本专题后，进入专题3（李群）将拥有一个更成熟的视角：不再把 exp 当作唯一选择，而是看到它在 retraction 大家族中的特殊位置。

### 符号表

| 符号 | 含义 | 首次出现 |
|------|------|---------|
| $R_x$ | Retraction 映射，$R_x: T_xM \to M$ | §14.1 |
| $T_xM$ | 流形 $M$ 在 $x$ 处的切空间 | §12 |
| $\mathrm{Exp}_x(\xi)$ | Riemannian 指数映射（测地线终点） | §15.1 |
| $\exp$ | 李群指数映射（矩阵指数） | §15.2 |
| $\operatorname{grad} f(x)$ | Riemannian 梯度 | §14.4 |
| $\xi$ | 切空间中的增量向量 | §14.1 |
| $\alpha$ | 线搜索步长 | §14.4 |
| $DR_x(0_x)$ | Retraction 在零向量处的微分 | §14.1 |
| $\operatorname{qf}(A)$ | QR 分解中的 Q 因子（QR retraction） | §综合题D |
| $\mathcal{T}_{\xi}$ | 向量迁移（vector transport） | §本章常见误解汇总 |
| $St(k, n)$ | Stiefel 流形 $\{X \in \mathbb{R}^{n \times k} \mid X^\top X = I_k\}$ | §综合题A |

---
