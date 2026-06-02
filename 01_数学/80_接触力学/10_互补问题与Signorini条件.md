## 专题 1 · 互补问题（LCP/NCP）与 Signorini 条件

接触动力学的数学核心可以压缩成一条链：**Signorini ⇒ LCP/NCP ⇒ 微分包含 ⇒ Moreau扫动过程**。这条主线把足式机器人的落地冲击、灵巧抓取的摩擦建模、乃至软体操控的FEM接触patch都统一在同一个凸分析与互补理论框架之下。本文给出一份可直接用于博士前自学或课程设计的大纲：数学内核、定理清单、教材对照评分、关键论文、开源工具、中英文资源与6–8周学习路径。重点在于——让读者清楚"为什么Coulomb摩擦不是P-matrix"、"Painlevé悖论为何需要测度解"、"SAP/TAMSI与Stewart-Trinkle的理论分野在哪里"——这些问题直接决定了Drake、MuJoCo、Siconos在仿真失败模式上的本质差异。

### 数学主干：从LCP到测度微分包含

#### LCP的代数-几何结构与QP-KKT等价

**线性互补问题**。给定 $M\in\mathbb{R}^{n\times n}$ 与 $q\in\mathbb{R}^n$，求 $(w,z)$ 满足
$$w=Mz+q,\quad w\ge 0,\ z\ge 0,\ w^{\mathsf T}z=0,$$
逐分量写作 $0\le z\perp(Mz+q)\ge 0$。**互补锥** $C_M(\alpha)$ 的 $2^n$ 个组合是否覆盖 $\mathbb{R}^n$（$M\in Q$）且不重叠，分别对应存在性与唯一性。

**与QP/KKT的等价**。凸QP $\min\tfrac12 x^{\mathsf T}Qx+c^{\mathsf T}x,\ Ax\ge b,\ x\ge 0$ 的KKT条件直接写成LCP，$M=\begin{pmatrix}Q & -A^{\mathsf T}\\ A & 0\end{pmatrix}$。机器人中多接触刚体动力学的法向互补条件天然产生这个形式；MLCP加入等式约束（$z$ 自由分量），对应双侧关节限位+单侧接触的混合场景。

#### Signorini的三级形式与VI等价

设间距 $g(q)\ge 0$、法向力 $\lambda_N\ge 0$。**位置级**（Signorini 1933/Fichera 1963）：$0\le g(q)\perp\lambda_N\ge 0$；**速度级**（Moreau）：$g=0$ 时 $0\le \dot g\perp\lambda_N\ge 0$；**加速度级**：$g=\dot g=0$ 时 $0\le\ddot g\perp\lambda_N\ge 0$。三级间的跃升改变了微分指标与冲量结构。等价的法锥形式为 $-\lambda_N\nabla g\in N_K(q)$，$K=\{q:g(q)\ge 0\}$；这也等价于变分不等式 $\langle M\ddot q-F,v-\dot q\rangle\ge 0,\ \forall v\in T_K(q)$。

#### Coulomb摩擦：2D→LCP，3D→SOCCP

Coulomb模型 $\|\lambda_T\|\le\mu\lambda_N$ 配合最大耗散原理 $\lambda_T\in\arg\max_{\|\tau\|\le\mu\lambda_N}(-u_T^{\mathsf T}\tau)$，即 $-u_T\in N_{\mathcal{C}_\mu}(\lambda_T)$。**2D线性化**（Stewart–Trinkle 1996）：切向锥在2D是区间 $[-\mu\lambda_N,\mu\lambda_N]$，引入 $\beta^\pm\ge 0$ 与松弛 $\sigma\ge 0$ 把整个时间步写为LCP。**3D则天然是二阶锥互补问题（SOCCP）**：$\mathcal{L}_\mu\ni\lambda\perp u\in\mathcal{L}_\mu^\circ$。多边形化（Anitescu-Potra 1997）把SOCCP降为LCP，但引入各向异性和 $O(K)$ 变量膨胀。Jordan代数框架给出统一处理。

#### Lemke算法与存在唯一性定理清单

Lemke通过引入人工变量 $z_0$ 与覆盖向量 $d>0$ 执行互补主元，终止于解或射线发散。下表是机器人接触所必须掌握的六条定理：

| 定理 | 精确陈述 | 机器人关联 | 出处 |
|---|---|---|---|
| **Lemke终止性** | $M$ copositive-plus ⇒ 算法要么给出LCP解，要么有限步内证不可行 | 无摩擦接触矩阵 $JM^{-1}J^{\mathsf T}$ 为PSD，Lemke保证成功；加入Coulomb则破坏 | Lemke *Manag. Sci.* 11 (1965) 681 |
| **Cottle-Dantzig存在性** | $M$ strictly copositive（或copositive-plus + $q\in K(M)$）⇒ LCP$(q,M)$ 有解 | Anitescu-Potra时间步LCP存在性基础 | Cottle-Dantzig *LAA* 1 (1968) 103 |
| **P-matrix唯一性** | $M\in\mathbf{P}$ ⇔ 互补锥划分 $\mathbb{R}^n$ ⇔ $\forall q$ 唯一解 | 满秩无摩擦Delassus算子是P-matrix，接触力唯一 | Samelson-Thrall-Wesler *Proc. AMS* 9 (1958) 805（正主子式刻画）；LCP唯一解↔P-matrix的完整等价证明归功于 Cottle (1966) 与 Ingleton (1966)；参见 Cottle-Pang-Stone Ch.3 的完整历史归属 |
| **Signorini-VI等价（Fichera）** | 椭圆PDE的Signorini条件等价于凸锥 $K$ 上VI；Stampacchia给出存在唯一 | 软体接触FEM、抓取接触patch | *Rend. Accad. Lincei* 34 (1964) 138 |
| **Painlevé悖论** | 存在 $(\mu,q,\dot q)$ 使刚体+Coulomb加速度级LCP无解或多解；均匀杆在特定接触倾角下的临界值 $\mu_P=4/3$（Genot-Brogliato 1999；此值非普适常数，取决于杆的质量分布、接触几何与倾角；Stewart 2000 给出另一配置下 $\mu>4/(3\sqrt{3})\approx 0.77$ 的不一致阈值） | 拖擦末端、粉笔跳动、jamming；时间步测度解消解（Stewart 1998） | Painlevé *CRAS* 121 (1895) 112 |
| **Moreau viability** | BV闭凸值 $C(t)$ ⇒ 扫动 $-dx\in N_{C(t)}(x)$ 存在BV右连续唯一解；二阶版本支持冲量跳跃 | Siconos的Moreau-Jean方案；颗粒、软体、多接触 | Moreau *JDE* 26 (1977) 347 |

**为何Coulomb不是P-matrix**——2D Stewart-Trinkle矩阵因摩擦非对称块含 $\mu$ 与 $D$，当 $\mu$ 大时出现负主子式；3D锥投影亦非单调。唯一性丧失产生三种后果：**多解（jamming）、无解（Painlevé不一致）、突发不可解需冲量解**。这三种模式对应所有仿真器失败的根本原因。

#### NCP函数与半光滑Newton

**Fischer-Burmeister** $\varphi_{\mathrm{FB}}(a,b)=\sqrt{a^2+b^2}-a-b$ 满足 $\varphi=0\Leftrightarrow a\ge 0,b\ge 0,ab=0$。强半光滑、merit函数 $\Psi=\tfrac12\|\Phi\|^2$ 连续可微且梯度Lipschitz，光滑化版本驱动内点算法。**Sun-Sun 2005**（*Math. Prog.* 103:575）证明其在Euclidean Jordan代数上的SOC/SDP版本全局强半光滑，为3D摩擦SOCCP的Newton二次收敛奠定基础。

#### Moreau扫动过程与SCL 2006框架

Brogliato-Daniilidis-Lemaréchal-Acary（*Systems & Control Letters* 2006）把刚体Signorini+Coulomb、理想二极管电路、塑性流统一写成微分变分不等式 $M\ddot q+h=F+J^{\mathsf T}\lambda,\ \lambda\in -N_{\mathcal{C}_\mu}(J\dot q+e)$，等价于测度微分包含。Moreau扫动 $-\dot x\in N_{C(t)}(x)$ 的二阶版本直接建模单侧约束 $-M\ddot q\in N_{T_K(q)}(\dot q^+)$。配套的**Moreau-Jean时间步进**在速度级离散，每步是LCP/QP，是Siconos核心。

### 五本教材的难度与覆盖对照

下表基于Springer/SIAM官方介绍、Amazon/AbeBooks真实用户评论与社区引用密度给出判断。难度1-5（1为研究生入门，5为前沿专著）。

| 教材 | 年/页 | 难度 | 本主题应读 | 优缺点与真实评价 |
|---|---|---|---|---|
| **Cottle-Pang-Stone《The Linear Complementarity Problem》** SIAM Classics 60 | 2009/761 | 3 | Ch.1-5全部；Ch.3存在唯一性、Ch.4 Lemke | 1994年Lanchester Prize；Amazon实读者："heartily recommend…thorough yet overall points easy to grasp"。**最权威的LCP参考**，但偏运筹背景，机器人动机弱。习题丰富无正式解答。 |
| **Facchinei-Pang《Finite-Dimensional VI & CP》** Springer两卷 | 2003/共约1200 | 5 | Vol.I Ch.1-3理论；Vol.II Ch.7-9算法 | *Quarterly of Applied Math.*："comprehensive state-of-the-art treatment"。Amazon："both volumes require mathematical maturity…indispensable"。**VI/NCP百科全书**，第二卷含FB/半光滑Newton全套。 |
| **Brogliato《Nonsmooth Mechanics》3rd** Springer CCE | 2016/629 | 4 | Ch.5非光滑Lagrangian、Ch.6多重冲击 | *Math. Reviews* Panagiotopoulos："excellent in combining rigorous mathematics with a great number of examples…allowing the reader to understand the basic concepts"。1300+参考文献，**机器人工程师的首选**，涵盖控制与稳定性。 |
| **Stewart《Dynamics with Inequalities》** SIAM | 2011/ | 4 | 刚体摩擦接触、Painlevé、测度解 | "the first book that comprehensively addresses dynamics with inequalities"。把有限维与无穷维统一处理，Painlevé悖论的测度解证明首次系统呈现。 |
| **Acary-Brogliato《Numerical Methods for Nonsmooth Dyn. Sys.》** Springer LNACM 35 | 2008/526 | 4 | Part II时间步进、Part IV Siconos | *ZAMM* Wieners："valuable and concise contribution…formal mathematical style with precise definitions…accessible for both researchers in mechanics and in mathematics"。**算法+Siconos手册**，与前四本互补。 |

**推荐组合**：入门用Cottle-Pang-Stone建立LCP直觉 → Brogliato 3rd作为力学主线 → Acary-Brogliato作为算法实现手册 → Facchinei-Pang作为理论查阅百科 → Stewart 2011深入Painlevé与测度解。

### 必读经典论文清单

**奠基论文**（1895–2000）。Painlevé "Sur les lois du frottement de glissement" *CRAS* 121 (1895) 112——刚体摩擦不一致的原始反例。Moreau "Unilateral contact and dry friction in finite freedom dynamics" *CISM* 302 (1988)——测度微分包含与扫动过程建模接触。Stewart-Trinkle "An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and Coulomb friction" *IJNME* 39 (1996) 2673，[DOI](https://doi.org/10.1002/(SICI)1097-0207(19960815)39:15%3C2673::AID-NME972%3E3.0.CO;2-I)——首个保证非穿透的时间步LCP格式，705+引用，ODE/Bullet/dVC的理论祖本。Pang-Trinkle "Complementarity formulations and existence of solutions of dynamic multi-rigid-body contact problems with Coulomb friction" *Math. Prog.* 73 (1996) 199——多体接触+Coulomb的存在性。Anitescu-Potra "Formulating dynamic multi-rigid-body contact problems with friction as solvable LCPs" *Nonlinear Dynamics* 14 (1997) 231——**凸松弛的开端**，后来成为MuJoCo/Drake SAP的理论源头。Génot-Brogliato "New results on Painlevé paradoxes" *Eur. J. Mech. A/Solids* (1999)——Painlevé悖论的完整相图。Stewart "Convergence of a time-stepping scheme for rigid-body dynamics and resolution of Painlevé's problem" *ARMA* 145 (1998) 215——**测度解存在性的严格证明**。

**现代与统一框架**（2000–2025）。Brogliato-Daniilidis-Lemaréchal-Acary相关*SCL* 2006工作——DVI/DCS/Moreau-Jean的统一。Anitescu "Optimization-based simulation of nonsmooth rigid multibody dynamics" *Math. Prog.* 105 (2006) 113——凸QP表述的大规模仿真。Le Lidec-Jallet-Montaut-Laptev-Schmid-Carpentier "Contact Models in Robotics: a Comparative Analysis" arXiv:[2304.06372](https://arxiv.org/abs/2304.06372)（2024 *IEEE T-RO*）——**当下最权威的机器人接触模型横向对比**，配套开源ContactBench基准。Castro-Permenter-Han "An unconstrained convex formulation of compliant contact"（Drake SAP的原始论文），以及Castro-Han-Masterjohn "Irrotational Contact Fields" arXiv:[2312.03908](https://arxiv.org/abs/2312.03908)——SAP/可微接触的最新实现。Posa-Cantu-Tedrake *IJRR* 33 (2014) 69——**contact-implicit trajectory optimization**，把LCP约束嵌入SQP。Howell等人"Predictive Sampling"与Aydinoglu-Posa共识互补控制（2024 *T-RO*）——实时CI-MPC。

### 学习资源：课程、开源与中英文博客

**英文课程与讲义**。Russ Tedrake *Underactuated Robotics* 的 [Planning and Control through Contact](https://underactuated.mit.edu/contact.html) 章节最适合首读——覆盖混合系统、接触约束、摩擦锥、冲量。Michael Posa DAIR Lab 的 [publications页面](https://dair.seas.upenn.edu/publications/) 与 [CV](https://dair.seas.upenn.edu/assets/pdf/posa_cv.pdf) 系统整理了过去十年接触控制的最佳实践，包含博士论文级别的免费教材资源。Stéphane Caron 的 [scaron.info](https://scaron.info) 博客对接触力锥、摩擦、QP有干净的工程笔记。YouTube上Tedrake与Posa的talks是动态补充。

**开源代码库**。[SICONOS (siconos/siconos)](https://github.com/siconos/siconos) 是INRIA的C++/Python非光滑动力学框架，支持LCP/MLCP/SOCCP/VI全套求解器（Lemke、PGS、FB-Newton、PATH接口），对应Acary-Brogliato 2008整本书；其[LCP求解器文档](https://nonsmooth.gricad-pages.univ-grenoble-alpes.fr/siconos/users_guide/problems_and_solvers/lcp.html)直接列出基于Facchinei-Pang的FB-LSA实现。[Drake (RobotLocomotion/drake)](https://drake.mit.edu) 的SAP/TAMSI求解器与Hydroelastic接触是工业级的manipulation仿真范式，对应[TRI "Rethinking Contact Simulation" 博客](https://medium.com/toyotaresearch/rethinking-contact-simulation-for-robot-manipulation-434a56b5ec88)深度解释了为何从点接触转向压力场；配套[Drake v1.5 Release Notes](https://drake.mit.edu/release_notes/v1.5.0.html)记录了SAP凸求解器的引入。[Simple-Robotics/ContactBench](https://github.com/Simple-Robotics/ContactBench) 是Le Lidec et al. 2024的基准实现。MuJoCo的软约束求解器、Bullet的MLCP、PATH Solver（Ferris-Munson的商用NCP求解器）各自代表不同建模哲学，ContactBench恰好做了定量对比。

**中文资源较稀缺，但工程入门可用**。知乎与CSDN上有[机器人力觉/阻抗-导纳控制](https://zhuanlan.zhihu.com/p/148556816)、[机械臂抓取流程](https://zhuanlan.zhihu.com/p/6116318359)、[协作机器人接触安全](https://zhuanlan.zhihu.com/p/53513669)、[机械臂运动规划中的接触任务](https://zhuanlan.zhihu.com/p/1898703023968092774)等工程向长文，但真正讲LCP/NCP/Signorini数学内核的中文一手资料极少——这是这套教学大纲的价值所在。建议中文读者以Brogliato 2016或Acary-Brogliato 2008为主干，辅以INRIA Siconos的Python教程。

### 八周自学路径

第一至二周打基础：Cottle-Pang-Stone Ch.1-2，理解LCP定义、互补锥、Lemke主元流程；完成Ch.4若干习题并用Python复现Lemke。第三至四周转入机械接触：Brogliato Ch.1-5 + Stewart-Trinkle 1996原文，手推2D滑块LCP矩阵并在Siconos跑bouncing ball示例。第五周深入Painlevé：Stewart 1998 ARMA + Génot-Brogliato 1999，理解为何需要测度解。第六周处理3D摩擦：Anitescu-Potra 1997 + Sun-Sun 2005 Jordan代数框架，用Drake SAP或MuJoCo复现滑动实验。第七周读当代综述Le Lidec et al. 2024与ContactBench代码，形成对六大仿真器失败模式的直观判断。第八周进入contact-implicit控制：Posa-Cantu-Tedrake 2014 IJRR + 最新CI-MPC论文（Aydinoglu 2024）。这条路径把从1895年Painlevé到2024年CI-MPC的130年工作压缩进两个月，每一步都有可运行的代码验证。

### 贯穿全局的几个认知转折

第一，**LCP不是唯一出口**。凸松弛（Anitescu 2006、Todorov MuJoCo、Castro SAP）用可解性换放弃严格Coulomb，在manipulation场景往往效果更好；这正是为什么Drake默认不走Stewart-Trinkle路线。第二，**Painlevé悖论不是奇例而是常态**。任何足端拖擦、指尖侧推都可能触发；工程上靠compliance正则化（penalty/hydroelastic）或测度解格式（Moreau-Jean）回避。第三，**矩阵类层级**（P ⊂ P₀、PSD ⊂ copositive-plus）不是抽象分类，它直接决定算法是否收敛、解是否唯一——工程师判断"为什么我的仿真炸了"必须从矩阵类入手。第四，**SOCCP与Jordan代数**是3D摩擦的正确数学语言，多边形近似只是工程权宜之计。第五，**Moreau的测度微分包含**把Signorini、Coulomb、冲击、电路二极管、塑性流统一——这种统一性本身就是机器人博士应有的思维高度。

掌握这五点，读者就能在TRI博客、DAIR Lab论文、Drake源码、Siconos代码之间自由穿行，并判断一个新的"可微接触"或"学习接触"论文究竟在既有理论图景中补了哪块拼图。

---

