## 第八批 · 专题 8.4：Diffusion Models 数学基础

> **定位**：扩散模型（Diffusion Models）在两年内从图像生成工具演变为机器人策略的核心骨架——Diffusion Policy（Chi et al. RSS 2023）和 $\pi_0$（Black et al. 2024）分别用 DDPM 和 Flow Matching 生成连续动作序列。本专题提供理解这些方法所需的**全部数学**：从 Anderson 1982 的反向时间 SDE 定理，到 Score Matching、ELBO 推导、收敛性理论、Flow Matching，再到 SE(3) 等变扩散，并打通到机器人采样式 MPC（MPPI = 单步去噪）与扩散式时空规划。
>
> **本专题类型**：理论教学（数学推导与理论分析为核心，几乎无代码）。少量代码仅用于数值验证推导结论，不承担讲解功能。
>
> **风格**：先动机后理论——每个关键定理给出"为什么需要它 → 不用它会怎样 → 谁怎么想到的 → 精确陈述 + 完整推导 → 机器人解读"的五段式展开，配合多视角类比、反事实推理与本质洞察。
>
> **与机器人的关系**：◉ **全方向核心**——Diffusion Policy 是操纵 SOTA；$\pi_0$ 用 Flow Matching 实现 50 Hz VLA 控制；SE(3)-DiffusionFields 用于 6-DoF 抓取生成；Diffuser 把轨迹优化化为去噪；而采样式 MPC 的 MPPI 更新本身就是扩散反向过程的单步。

---

### 前置自测 ⭐

> 📋 **怎么用这份自测**：下面 5 道题考查本专题的硬前置（随机微积分、ODE、测度论、概率）。答不出 **$\ge 2$ 题**，强烈建议先回到 `01_数学/95_随机分析/10_随机微分方程基础.md` 补齐 Itô 积分与 Fokker-Planck，再读本专题——否则反向 SDE 的每一步推导都会卡住。第 5 题不是前置，而是一个"先记下你的直觉、本专题会给你反直觉答案"的引子。

| 编号 | 问题 | 答不出 → 回顾 |
|:----:|------|------------|
| 1 | 什么是布朗运动 $W_t$？写出它的三条定义性质（独立增量、增量分布、连续路径），并解释 $\mathbb{E}[(W_t-W_s)^2]=t-s$ 为何成立。 | 随机分析 9.1 §布朗运动 |
| 2 | 写出 Itô 公式（一维）：对光滑函数 $f(t,x)$ 和 $\mathrm{d}X_t=\mu\,\mathrm{d}t+\sigma\,\mathrm{d}W_t$，$\mathrm{d}f(t,X_t)=?$ 那个"多出来的二阶项" $\tfrac12\sigma^2 f_{xx}\,\mathrm{d}t$ 从哪里来？ | 随机分析 9.1 §Itô 公式 |
| 3 | 给定线性 SDE $\mathrm{d}X_t=-\tfrac12\beta X_t\,\mathrm{d}t+\sqrt\beta\,\mathrm{d}W_t$（常数 $\beta$），它的解 $X_t$ 服从什么分布？均值和方差随 $t$ 如何演化？ | 随机分析 9.1 §OU 过程 |
| 4 | 什么是概率密度的 KL 散度 $\mathrm{KL}(p\Vert q)$？它非负吗？为什么 $\mathrm{KL}(p\Vert q)=0 \iff p=q$（几乎处处）？ | 测度论 B2 / 信息论 |
| 5 | 你听说"扩散模型表达能力很强、能拟合任意多峰分布"。凭直觉猜：这种表达能力是因为每一步去噪用了比高斯更复杂的分布，还是另有来源？（本专题 §8.4.3、§8.4.8 会给出答案，先记下你的猜测。） | —— |

**参考答案要点**（自评用，不必逐字对照）：

1. $W_0=0$；增量 $W_t-W_s\sim\mathcal{N}(0,t-s)$ 且与历史独立；路径连续。方差 $=t-s$ 是定义的一部分，物理上对应"扩散距离的平方正比于时间"。
2. $\mathrm{d}f=\big(f_t+\mu f_x+\tfrac12\sigma^2 f_{xx}\big)\mathrm{d}t+\sigma f_x\,\mathrm{d}W_t$。二阶项来自 $(\mathrm{d}W_t)^2=\mathrm{d}t$（布朗运动的二次变差非零）——这是 Itô 微积分与普通微积分的根本分野。
3. $X_t\sim\mathcal{N}\big(e^{-\beta t/2}x_0,\,1-e^{-\beta t}\big)$（从确定初值出发）。均值指数衰减到 0，方差从 0 单调升到 1——这正是 VP-SDE 的转移核，本专题 §8.4.1 会反复用。
4. $\mathrm{KL}(p\Vert q)=\int p\log\tfrac pq$；由 Jensen 不等式 $\mathrm{KL}\ge0$；等号当且仅当 $p=q$ a.e.（$\log$ 严格凹，Jensen 取等需被积变量几乎处处为常数）。
5. 表达能力**不**来自单步用更复杂的分布——扩散每一步仍是高斯。它来自"多步去噪的复合"：许多个简单高斯步首尾相接，能雕出任意复杂的多峰分布。这与 `04_移动机器人规控/20_采样式MPC/60_扩散启发采样MPC.md` 中"MPPI 单步是高斯、多级退火才突破高斯天花板"是同一个洞察。

---

### 本章目标

学完本专题后，你应该能够：

1. **写出前向扩散的 Itô SDE**（VP/VE 两种形式），并从线性 SDE 解的一般理论推导出闭式转移核 $p_{t|0}(x_t|x_0)$，解释噪声调度（noise schedule）如何控制信噪比；
2. **陈述并理解 Anderson 反向时间 SDE 定理**（1982），说清它为什么把"生成"问题化归为"估计 score 函数 $\nabla\log p_t$"这一件事；
3. **从头推导 DDPM 的 ELBO**，并把均值参数化重写为噪声预测（$\varepsilon$-prediction），得到简化损失 $L_\text{simple}$，解释为什么"预测噪声"比"预测均值"在工程上更稳；
4. **理解 Score Matching 家族的三种形式**（Hyvärinen 显式 / Vincent 去噪 / Sliced 切片）及其等价性，能用分部积分独立证明 Vincent 去噪定理；
5. **陈述 Tweedie 公式**并用它统一 $\varepsilon$-预测、数据预测、score 预测三种参数化——理解它们是同一个量的三种坐标；
6. **理解扩散模型收敛理论**的三源误差分解（初始化失配 / score 估计误差 / 离散化误差）与 Girsanov 证明骨架，知道 Chen 2023 的 $d^2$ 与 Benton 2024 的 $d$-线性界差在哪；
7. **掌握 Flow Matching**：理解 Lipman 条件 Flow Matching 等价恒等式（CFM 与 FM 同梯度）、最优传输直线路径、Rectified Flow 的 reflow，并理解 Stochastic Interpolants 如何把 DDPM 与 FM 统一为同一连续族的两端；
8. **将理论桥接到机器人**：理解 Diffusion Policy / $\pi_0$ / Diffuser / SE(3)-DiffusionFields 的数学本质，并能解释 MPPI 反向退火与扩散反向 SDE 的同构（呼应采样式 MPC 专题）。

---

### 本章知识导航

本专题的数学骨架可以用一句话概括：**前向加噪是简单的（线性 SDE，闭式可解）；难的是反向去噪，而反向去噪的全部信息都浓缩在一个函数 $\nabla\log p_t(x)$ 里——这个函数叫 score，估计它就是训练，沿它走就是采样。** 整章围绕这条主线展开。

知识结构分四个板块，递进关系如下：

```
板块 A：前向与反向的连续时间理论（§8.4.1–8.4.2）
    前向 SDE（加噪）──闭式转移核──→ Anderson 反向 SDE（去噪）──→ 概率流 ODE
         │                                      │
         │                                      ▼
板块 B：把"反向"变成"可训练"（§8.4.3–8.4.4）
    DDPM 离散视角：ELBO ──重参数化──→ $\varepsilon$-预测 ←──Tweedie──→ score 预测
    Score Matching：显式 ──分部积分──→ 去噪 DSM ──Sliced──→ 大规模可行
         │
         ▼
板块 C：统一与保证（§8.4.5–8.4.7）
    收敛理论（Girsanov + 三源误差）│ Flow Matching（ODE 视角）│ 测度论骨架（Fokker-Planck/JKO）
         │
         ▼
板块 D：机器人落地（§8.4.8）
    Diffusion Policy（SDE/DDPM）│ $\pi_0$（ODE/FM）│ Diffuser（规划=去噪）│ SE(3) 扩散
              ▲                                                              
              └──── 与 04_采样式MPC：MPPI = 单步去噪、退火 = 反向扩散 ────┘
```

| 知识点 | 依赖 | 与其他知识点的关系 |
|------|------|------------------|
| §8.4.1 前向 SDE | 随机分析（Itô） | 全章地基，提供闭式转移核 |
| §8.4.2 Anderson 反向 SDE | §8.4.1 + Fokker-Planck | 把"生成"化归为"估计 score"——全章枢纽 |
| §8.4.3 DDPM ELBO/$\varepsilon$-预测 | §8.4.1 + 变分推断 | 反向过程的离散、可训练版本 |
| §8.4.4 Score Matching | §8.4.2 | 提供训练 score 的损失函数 |
| §8.4.5 收敛理论 | §8.4.2 + Girsanov | 回答"训练好 score 后采样能多准" |
| §8.4.6 Flow Matching | §8.4.2（概率流 ODE） | 反向过程的确定性、ODE 版本 |
| §8.4.7 测度论骨架 | §8.4.1–8.4.6 | 把前面所有视角统一到测度空间几何 |
| §8.4.8 机器人应用 | 全部 | 落地与跨章桥接 |

**推荐阅读路径**：

- **主干优先**（首次学习）：§8.4.1 → §8.4.2 → §8.4.3 → §8.4.4 → §8.4.6 → §8.4.8，跳过 §8.4.5（收敛理论）和 §8.4.7（测度论骨架）的证明细节。这条路径让你最快理解"训练什么、怎么采样、机器人怎么用"。
- **理论深读**（二刷或做研究）：补上 §8.4.5 的 Girsanov 证明和 §8.4.7 的 JKO 梯度流——它们回答"为什么这套方法有数学保证"，是写理论论文的基础。
- **机器人优先**（只想用 Diffusion Policy）：先读 §8.4.8，遇到不懂的术语再回头查 §8.4.3（$\varepsilon$-预测）和 §8.4.6（FM）。

> 注意：本导航只展示**结构**，不展开具体内容。每个板块的完整推导见对应小节。

---

### 前置知识桥接

本专题站在三条前置链的交汇点上，这里用 2-3 行各激活一次核心要点，让你不翻回去也能跟上：

- **回顾随机分析 9.1（`01_数学/95_随机分析/10_随机微分方程基础.md`）**：那里我们严格构造了 Itô 积分 $\int_0^t f\,\mathrm{d}W_s$，证明了 **Itô 等距** $\mathbb{E}\big[\big(\int f\,\mathrm{d}W\big)^2\big]=\int\mathbb{E}[f^2]\,\mathrm{d}t$，并推导了 **Itô 公式** $\mathrm{d}f=(f_t+\mu f_x+\tfrac12\sigma^2 f_{xx})\mathrm{d}t+\sigma f_x\mathrm{d}W$ 与 **Fokker-Planck 方程** $\partial_t p=-\partial_x(\mu p)+\tfrac12\partial_x^2(\sigma^2 p)$。本专题把这些工具直接用于扩散过程：闭式转移核（§8.4.1）用 Itô 等距算方差，反向 SDE（§8.4.2）用 Fokker-Planck 的时间反演，收敛证明（§8.4.5）用 Itô 积分的路径测度。

- **回顾测度论 B2 与信息论**：那里我们定义了概率密度、Lebesgue 积分与 KL 散度，并证明了 $\mathrm{KL}\ge0$（Gibbs 不等式）。本专题用 KL 散度写 ELBO（§8.4.3）、度量收敛（§8.4.5），用全变差距离 $\mathrm{TV}$ 与数据处理不等式 $\mathrm{TV}^2\le\tfrac12\mathrm{KL}$ 把 KL 界翻译成采样精度界。

- **回顾 ODE 基础 B4**：那里我们学了线性 ODE 的解结构与数值积分（Euler、Runge-Kutta）。本专题的概率流 ODE（§8.4.2、§8.4.6）就是一个向量场 $\dot x=v(x,t)$，采样即数值积分；SDE 是 ODE 加上布朗噪声项的推广。

> [!WARNING]
> **Itô 前置断层警告（务必先读）**
>
> 早期版本的机器人数学路线图中**没有**任何专题系统覆盖随机微积分。现在这一缺口已由 `01_数学/95_随机分析/10_随机微分方程基础.md`（专题 9.1）系统补全——它正是为本专题铺路而写。**学习本专题前，请确认你已掌握**：布朗运动定义、Itô 积分与等距、Itô 公式、OU 过程求解、Fokker-Planck 推导。若这些概念陌生，请先完成专题 9.1（约 8–10 小时），否则 §8.4.2 起的每一步推导都会悬空。

---

### 如果跳过本章会怎样

下面两个具体场景说明不学本专题会在哪里卡住：

- **场景 1：你要复现或改进 Diffusion Policy**。论文里写"训练目标是 $\mathbb{E}\|\varepsilon-\varepsilon_\theta(A^{(k)},k,o)\|^2$"，但不解释为什么"预测噪声"等价于"学策略分布"。不学 §8.4.3–8.4.4，你会把它当成一个玄学损失函数照抄，调参时不知道噪声调度、采样步数、DDPM/DDIM 的取舍背后的数学约束——一旦动作维度变化或多峰性消失，你无从判断是哪一环出了问题。

- **场景 2：你要在采样式 MPC 里用扩散思想**（如 Model-Based Diffusion、DIAL-MPC）。`04_移动机器人规控/20_采样式MPC/60_扩散启发采样MPC.md` 的核心论点是"MPPI 一步更新 = 扩散单步去噪、退火 = 完整反向扩散"。如果你不理解本专题 §8.4.2 的反向 SDE 和 §8.4.4 的 score 估计，这个同构对你只是一句口号——你无法理解为什么"多级退火能突破单级 MPPI 的高斯天花板"，也无法把图像扩散里的噪声调度经验迁移到力矩级控制。

---

### 预计阅读时间

| 阅读方式 | 时间 | 适合谁 |
|---------|------|--------|
| 精读（含推导手算与练习） | 14–18 小时 | 需要深入理解、要做扩散相关研究的读者 |
| 速读（跳过 §8.4.5/§8.4.7 证明细节） | 6–8 小时 | 已有 SDE 基础、想快速建立全景的读者 |
| 速查（只看表格、定理速查表、四重视角统一表） | 40 分钟 | 遇到具体问题（如"$\varepsilon$ 和 score 怎么换算"）回来查 |

> **额外提示**：若随机微积分前置未补，需先加 8–10 小时完成专题 9.1。本专题与 `01_数学/90_深度学习数学/50_具身智能VLA框架.md`、`04_移动机器人规控/20_采样式MPC/60_扩散启发采样MPC.md` 强耦合，建议三者交替阅读。

---

### 科研发展脉络

理论教学的第一要义是"追溯来龙去脉"——不是直接给出最终形式，而是还原前人如何一步步想到这套方法。扩散模型不是 2020 年凭空出现的，它有一条清晰的两段式血脉：**一条来自统计物理与随机控制（反向时间 SDE）**，**一条来自统计推断（score matching 避开配分函数）**，两条线在 2020–2021 年汇合，催生了 DDPM 与 Score SDE。

*（引用计数截至 2025 年中，仅供量级参考。）*

| 阶段 | 年代 | 代表工作 | Venue | 引用 | 核心贡献 |
|------|------|---------|-------|------|---------|
| **G1 理论根基** | 1982 | Anderson | Stoch. Proc. Appl. | ~1,100 | 反向时间 SDE 定理：任何前向扩散都有配套反向扩散 |
| | 1986 | Haussmann & Pardoux | Ann. Probab. | ~700 | 时间反演的严格条件（Anderson 的测度论加固） |
| | 2005 | Hyvärinen | JMLR | ~1,800 | Score Matching：用分部积分避开配分函数 |
| | 2011 | Vincent | Neural Computation | ~2,500 | 去噪 Score Matching = Score Matching（DDPM 训练的数学基础） |
| **G2 深度生成** | 2015 | Sohl-Dickstein et al. | ICML | ~5,500 | 非平衡热力学 → 深度生成：首次把扩散用于学习数据分布 |
| | 2019 | Song & Ermon (NCSN) | NeurIPS | ~4,000 | 多尺度噪声扰动 + Langevin 采样（VE-SDE 前身） |
| | 2020 | Ho, Jain, Abbeel | NeurIPS | ~20,000 | **DDPM**：$\varepsilon$-预测 + 简化损失，图像质量首次比肩 GAN |
| | 2021 | Song et al. | ICLR Outstanding | ~10,000 | **Score SDE**：VP/VE SDE 统一框架 + 概率流 ODE |
| | 2021 | Song, Meng, Ermon | ICLR | ~8,000 | **DDIM**：非马氏确定性采样，10–50 步生成 |
| **G3 收敛理论** | 2023 | Chen, Chewi, Li, Li, Salim | ICLR Top-5% | ~600 | 最小假设下 TV 收敛：$\tilde{O}(L^2 d/\varepsilon^2)$ |
| | 2024 | Benton et al. | ICLR Spotlight | ~300 | 近 $d$-线性 KL 界：$\tilde{O}(d/\varepsilon^2)$，随机局部化 |
| **G4 Flow Matching** | 2023 | Lipman, Chen et al. | ICLR | ~2,000 | 条件 Flow Matching：仿真无关的 CNF 训练 |
| | 2023 | Liu, Gong, Liu | ICLR Spotlight | ~1,200 | Rectified Flow：直线路径 + 单步生成 |
| | 2023 | Albergo, Vanden-Eijnden | arXiv/ICLR | ~500 | Stochastic Interpolants：DDPM 与 FM 的统一族 |
| **G5 机器人应用** | 2022 | Janner et al. (Diffuser) | ICML Long Talk | ~1,500 | 轨迹优化 = 去噪；规划 = classifier guidance |
| | 2023 | Chi et al. (Diffusion Policy) | RSS / IJRR | ~2,500 | 视觉运动策略的扩散建模，15 任务 +46.9% |
| | 2023 | Urain et al. (SE(3)-DiffusionFields) | ICRA | ~300 | SE(3) 上的扩散抓取生成 |
| | 2024 | Black et al. ($\pi_0$) | arXiv | ~200 | VLA + Flow Matching：50 Hz 实时控制 |
| | 2024 | Pan et al. (Model-Based Diffusion) | arXiv/CoRL | ~50 | 无需数据的扩散式轨迹优化（与采样式 MPC 直连） |

**这条脉络的两次关键汇合**：

1. **2011 年 Vincent 的去噪 Score Matching**把"学 score"从"需要算 Hessian trace（不可扩展）"变成"学一个去噪器（可扩展）"——这是 DDPM 训练目标的数学前身，但当时无人意识到它能生成高质量图像。
2. **2020–2021 年 Ho 与 Song 的汇合**：Ho 从变分推断（ELBO）出发得到 $\varepsilon$-预测损失，Song 从 score matching + 反向 SDE 出发得到 score 预测；Tweedie 公式（§8.4.6）随后揭示**两者预测的是同一个量的不同坐标**。这次汇合统一了"变分"与"score"两个看似无关的视角。

> **本质洞察**：扩散模型的历史告诉我们一个反复出现的模式——**一个 1982 年的纯理论结果（Anderson 反向 SDE）和一个 2011 年的统计技巧（去噪 DSM）各自沉睡多年，直到深度神经网络提供了"足够好的 score 估计器"才被激活。** 这不是巧合：反向 SDE 把生成化归为 score 估计，但 score 是一个高维向量场，只有神经网络的逼近能力（专题 8.1）才让这个化归变得可计算。理论、统计、算力三者缺一不可。

---

### 教学目标与核心定理速览

下表是本专题要讲透的核心定理。"深度"列标注讲解程度：**完**=完整证明、**骨**=证明骨架、**陈**=精确陈述+引用。"必完证"的五个定理（#2/#4/#5/#6/#9）是博士生必须能独立复述证明的。

| # | 定理/结果 | 深度 | 所在节 | 机器人关键词 |
|---:|---|:---:|:---:|---|
| 1 | VP/VE SDE 的闭式转移核 | 完 | §8.4.1 | OU 过程、噪声调度 |
| 2 | **Anderson 反向时间 SDE 定理 (1982)** | 完 | §8.4.2 | 从噪声恢复数据的数学基础 |
| 3 | 概率流 ODE（Song et al. 2021） | 骨 | §8.4.2 | DDIM / Flow Matching 的几何基础 |
| 4 | **DDPM ELBO 与 $\varepsilon$-预测等价** | 完 | §8.4.3 | 训练目标推导 |
| 5 | **Score Matching = 去噪 Score Matching** | 完 | §8.4.4 | 避免 Hessian trace |
| 6 | **Tweedie 公式** | 完 | §8.4.4 | 三种参数化的统一 |
| 7 | **Chen et al. 2023 TV 收敛定理** | 骨 | §8.4.5 | Girsanov + 三源误差 |
| 8 | Benton et al. 2024 $d$-线性 KL 界 | 陈 | §8.4.5 | 随机局部化 |
| 9 | **Lipman CFM 等价恒等式** | 完 | §8.4.6 | Flow Matching 训练 |
| 10 | Rectified Flow 凸代价不增定理 | 骨 | §8.4.6 | 直线路径、单步生成 |
| 11 | Fokker-Planck = $W_2$ 梯度流 (JKO) | 骨 | §8.4.7 | 测度论基础 |
| 12 | Girsanov 定理 | 骨 | §8.4.7 | 收敛证明核心工具 |

---

## §8.4.1 前向扩散：把数据"溶解"成噪声 ⭐⭐

### 动机：我们究竟想做什么

生成模型的终极目标只有一句话：**给定一堆样本 $x_1,\dots,x_N\sim p_\text{data}$（一批专家演示轨迹、一批 RGB 图、一批抓取位姿），学会从同一个分布 $p_\text{data}$ 里采出新样本。** 注意这里没有要求"算出 $p_\text{data}(x)$ 的数值"——我们只要会**采样**就够了。机器人语境下这一点尤其关键：Diffusion Policy 要的不是"这个动作序列的概率是多少"，而是"给我一个在当前观测下合理的动作序列"。

直接学 $p_\text{data}$ 难在哪？数据分布是高维（一段 16 步、7 自由度的动作序列就是 112 维）、多峰（同一个抓取任务有从左、从右两种合理路径）、形状未知的怪物。历史上人们尝试过三条路，每条都撞墙：

| 路线 | 核心思想 | 撞的墙 |
|------|---------|--------|
| 显式密度（VAE、归一化流） | 直接参数化 $p_\theta(x)$ 并最大化似然 | VAE 的 ELBO 有难以收紧的间隙；归一化流要求可逆+雅可比可算，限制了网络结构 |
| 隐式密度（GAN） | 学一个生成器 $G:z\mapsto x$，用判别器对抗训练 | 训练不稳定、模式崩溃（multi-modal 的动作分布被压成单峰）|
| 自回归（PixelCNN、自回归 Transformer） | 把联合分布拆成条件链 $\prod p(x_i\mid x_{<i})$ | 采样必须逐维串行，高维时极慢；强加了人为的维度顺序 |

扩散模型走的是第四条路，它的核心赌注是一个看似绕远、实则极其聪明的迂回：

> **不直接学"如何从噪声造出数据"，而是先研究"如何把数据逐步毁成噪声"，再学会把这个毁灭过程倒放。**

为什么这个迂回是天才的？因为"毁灭"方向可以设计得**极其简单**——简单到有闭式解、无需训练；而所有的学习负担都被转移到"倒放"上，且倒放被证明只依赖一个可学的量（score）。本节先把"毁灭"方向彻底讲透；§8.4.2 再处理"倒放"。

### 如果不这样做会怎样：一个失败的直觉方案

设想一个朴素方案：直接训一个网络 $G_\theta$ 把标准高斯 $z\sim\mathcal N(0,I)$ 映射到数据。这正是 GAN/VAE 解码器在做的事。问题在于，这要求**单次**前向就完成"从无结构噪声到高度结构化数据"的全部跳跃。对一个 112 维、有多个分离峰的动作分布，这个映射本身极度非线性、极度病态——网络容量稍有不足，要么只覆盖一个峰（模式崩溃），要么在峰之间"抹平"产生不合法的中间动作（对机器人是危险的）。

> **本质洞察**：扩散模型的全部威力，来自把"一次困难的跳跃"拆成"许多次容易的小步"。每一小步只需要去掉一点点噪声——这是一个良态（well-conditioned）的局部回归问题；而许多小步的**复合**却能雕刻出任意复杂的多峰分布。这与前置自测第 5 题的答案完全一致：表达能力不来自单步用了复杂分布（每步仍是高斯），而来自复合的深度。把它和你已知的东西对照——这正是深度网络"深"的意义在概率分布上的翻版：单层线性变换平庸，多层复合却万能。

类比标注边界：这个"小步复合"与**数值积分把一段曲线轨迹拆成许多小直线段**有**像**的一面——都是用大量简单局部操作逼近一个复杂全局对象；但**不像**之处在于，数值积分逼近的是一条确定曲线，而扩散逼近的是一个概率分布的演化，每一步还注入了随机性。不要把这个类比延伸到"误差会像 Euler 法那样线性累积"——扩散的误差分析（§8.4.5）要用 Girsanov 而非简单的局部截断误差求和。

### 历史：从非平衡热力学到 SDE

扩散这个词不是比喻，而是字面意义的物理过程。1905 年 Einstein 用扩散方程解释布朗运动：一滴墨水在水中，分子的无规则碰撞让浓度分布 $p(x,t)$ 按热方程 $\partial_t p=\tfrac12\partial_x^2 p$ 演化，最终铺满整个容器变成均匀分布。Sohl-Dickstein 等人(ICML 2015)做了一个反向的提问：**如果数据分布是初始的"墨滴"，让它扩散成均匀噪声很容易；那么能不能学会这个扩散过程的逆，从均匀噪声"反扩散"回墨滴形状？** 他们借用非平衡热力学中的思想首次实现了这一点，但当时图像质量平平，无人重视。直到 2020 年 Ho 等人的 DDPM 和 2021 年 Song 等人的 Score SDE 把它打磨到超越 GAN，扩散才爆发。

本节采用 Song et al. (2021) 的连续时间 SDE 视角，因为它最统一：DDPM 的离散链(§8.4.3)、NCSN 的多尺度噪声、DDIM 的确定性采样，全都是这个 SDE 框架的特例。

### 理论：前向 SDE 的一般形式

我们把"逐步加噪"写成一个连续时间的随机过程。设数据 $x_0\sim p_\text{data}$，定义一族随机变量 $\{x_t\}_{t\in[0,T]}$，由如下 Itô 随机微分方程(SDE)驱动：

$$
\mathrm{d}x_t = f(x_t,t)\,\mathrm{d}t + g(t)\,\mathrm{d}W_t,\qquad x_0\sim p_\text{data}
$$

其中：

- $f(x_t,t)\in\mathbb R^d$ 称为**漂移系数**(drift)，决定确定性的"拉拽"方向；
- $g(t)\in\mathbb R$ 称为**扩散系数**(diffusion coefficient)，决定注入噪声的强度；
- $W_t$ 是标准 $d$ 维布朗运动(回顾前置自测 1:$W_0=0$，增量独立且 $W_t-W_s\sim\mathcal N(0,(t-s)I)$，路径连续)。

> **回顾随机分析 9.1**:SDE $\mathrm{d}x_t=f\,\mathrm{d}t+g\,\mathrm{d}W_t$ 的含义不是逐点求导(布朗运动处处不可导！)，而是积分形式 $x_t=x_0+\int_0^t f\,\mathrm{d}s+\int_0^t g\,\mathrm{d}W_s$ 的简写，其中第二个积分是 Itô 积分。我们在那里证明的 **Itô 等距** $\mathbb E[(\int g\,\mathrm{d}W)^2]=\int \mathbb E[g^2]\,\mathrm{d}s$ 马上就要用来算方差。

我们记 $p_t$ 为 $x_t$ 的边际密度(marginal density),$p_{t|0}(x_t\mid x_0)$ 为给定起点的**转移核**(transition kernel)。整个前向过程的设计目标是：

1. $p_0=p_\text{data}$(起点是数据);
2. $p_T\approx\pi$，一个**与数据无关的、易采样的**先验分布(终点是纯噪声，通常 $\pi=\mathcal N(0,I)$)。

这两个条件是反向过程能工作的前提：反向时我们从 $\pi$ 起步(因为它易采样)，所以必须保证前向真的把数据推到了 $\pi$。

### 两种主流选择：VP-SDE 与 VE-SDE

漂移和扩散系数怎么选？Song et al. 给出两个典范，对应两种历史路线的连续化：

**(1) VP-SDE(Variance Preserving，方差保持),DDPM 的连续极限：**

$$
\boxed{\ \mathrm{d}x_t = -\tfrac12\beta(t)\,x_t\,\mathrm{d}t + \sqrt{\beta(t)}\,\mathrm{d}W_t\ }
$$

漂移项 $-\tfrac12\beta(t)x_t$ 是一个**线性回拉**(把 $x_t$ 往原点拽)，扩散项 $\sqrt{\beta(t)}$ 注入噪声。$\beta(t)>0$ 是噪声调度(noise schedule)。这正是前置自测第 3 题的 OU 过程的时变版本。

**(2) VE-SDE(Variance Exploding，方差爆炸),NCSN 的连续极限：**

$$
\boxed{\ \mathrm{d}x_t = \sqrt{\tfrac{\mathrm{d}\,[\sigma^2(t)]}{\mathrm{d}t}}\,\mathrm{d}W_t\ }
$$

漂移为零($f=0$)，只靠不断增大的噪声 $\sigma(t)$ 把数据淹没。

> **本质洞察**:VP 和 VE 的区别可以用一句话点破——**VP 边加噪边把信号按比例缩小(保持总方差有界),VE 信号不缩、只让噪声方差爆炸式增长**。前者像"把一杯浓咖啡不断兑水同时也倒掉一部分，杯子总量不变"；后者像"咖啡不动，往一个无限大的池子里持续注入清水直到浓度趋零"。两者最终都让信号被噪声主导，殊途同归。这个"保持 vs 爆炸"的二分法不是任意的，它对应反向采样时数值稳定性的不同权衡(VP 的有界方差更利于神经网络处理)。

类比边界：把 VP-SDE 类比成 OU 过程**像**在"线性回拉 + 高斯噪声 → 高斯平稳分布",**不像**在标准 OU 的系数是常数而这里 $\beta(t)$ 随时间增大；不要把"OU 有平稳分布"直接套用——VP 在有限时间 $T$ 并未真正到达平稳分布，只是近似($p_T\approx\mathcal N(0,I)$ 有一个 $e^{-\bar\beta}$ 量级的残差，见下)。

### 核心推导：VP-SDE 的闭式转移核

前向过程"简单"的全部含金量在于：**转移核 $p_{t|0}$ 有闭式高斯解，无需任何模拟即可直接采样 $x_t$。** 这是训练能高效进行的命门——训练时我们要对随机的 $t$ 采样 $x_t$，如果每次都要从 $0$ 数值积分到 $t$，训练将慢到不可行。现在我们从头推这个闭式解。

**假设**:VP-SDE,$x_0$ 给定(先看确定初值，再对 $x_0$ 取期望即得一般情形)。目标是求 $x_t$ 的分布。

**Step 1：线性 SDE 的解是高斯的。** 关键观察：VP-SDE 的漂移 $-\tfrac12\beta(t)x_t$ 对 $x_t$ 是**线性**的，扩散系数 $\sqrt{\beta(t)}$ 不依赖 $x_t$。这类 SDE 叫**线性 SDE**，它的解必然是高斯过程(因为高斯初值经线性变换+加高斯噪声仍是高斯)。所以我们只需求出均值 $m(t)=\mathbb E[x_t]$ 和方差 $\Sigma(t)=\mathrm{Cov}(x_t)$ 两个量，分布就完全确定了。

**Step 2：求均值。** 对 SDE 两边取期望。Itô 积分项 $\int g\,\mathrm{d}W$ 是鞅，期望为零，于是噪声项消失：

$$
\frac{\mathrm{d}}{\mathrm{d}t}\,\mathbb E[x_t] = -\tfrac12\beta(t)\,\mathbb E[x_t]
\quad\Longrightarrow\quad
\frac{\mathrm{d}m}{\mathrm{d}t}=-\tfrac12\beta(t)\,m
$$

这是一个一阶线性 ODE，分离变量 $\frac{\mathrm{d}m}{m}=-\tfrac12\beta(t)\,\mathrm{d}t$，积分得：

$$
m(t) = x_0\,\exp\!\Big(-\tfrac12\int_0^t\beta(s)\,\mathrm{d}s\Big)
$$

为记号简洁，定义**累积噪声** $\displaystyle\bar\beta(t):=\int_0^t\beta(s)\,\mathrm{d}s$，则 $m(t)=x_0\,e^{-\bar\beta(t)/2}$。**物理意义**：信号(均值)按 $e^{-\bar\beta(t)/2}$ 指数衰减——$t$ 越大，原始数据的"影子"越淡，这就是"信号被溶解"的精确刻画。

> **阶段小结**：到这里我们用"取期望消掉鞅项"得到了均值的指数衰减 $m(t)=x_0 e^{-\bar\beta(t)/2}$。接下来要做的是求方差——这一步需要用到 Itô 公式处理 $x_t^2$，是整个推导的技术核心。

**Step 3：求方差。** 方差不能简单取期望得到，因为 $\mathbb E[x_t^2]$ 涉及二阶项，必须动用 Itô 公式。考虑函数 $\phi(x)=x^2$(为清晰起见看一维分量；高维各分量独立，同理)。Itô 公式(回顾前置自测 2)：对 $\mathrm{d}x_t=f\,\mathrm{d}t+g\,\mathrm{d}W_t$,

$$
\mathrm{d}\phi(x_t)=\Big(\phi'f+\tfrac12\phi'' g^2\Big)\mathrm{d}t+\phi' g\,\mathrm{d}W_t
$$

代入 $\phi'=2x_t$, $\phi''=2$, $f=-\tfrac12\beta x_t$, $g=\sqrt\beta$:

$$
\mathrm{d}(x_t^2)=\Big(2x_t\cdot(-\tfrac12\beta x_t)+\tfrac12\cdot 2\cdot\beta\Big)\mathrm{d}t+2x_t\sqrt\beta\,\mathrm{d}W_t
=\big(-\beta x_t^2+\beta\big)\mathrm{d}t+2x_t\sqrt\beta\,\mathrm{d}W_t
$$

**注意那个 $+\beta$ 项**：它正是 Itô 公式的二阶修正项 $\tfrac12\phi''g^2=\tfrac12\cdot2\cdot\beta=\beta$ 的产物，来自 $(\mathrm dW_t)^2=\mathrm dt$。这是普通微积分**没有**的项——若用普通链式法则你会漏掉它，从而算错方差。取期望(再次消掉鞅项):

$$
\frac{\mathrm d}{\mathrm dt}\,\mathbb E[x_t^2]=-\beta(t)\,\mathbb E[x_t^2]+\beta(t)
$$

这是一阶线性非齐次 ODE。用积分因子 $e^{\bar\beta(t)}$:$\frac{\mathrm d}{\mathrm dt}\big(e^{\bar\beta}\mathbb E[x_t^2]\big)=\beta e^{\bar\beta}=\frac{\mathrm d}{\mathrm dt}e^{\bar\beta}$。积分(初值 $\mathbb E[x_0^2]=x_0^2$):

$$
e^{\bar\beta(t)}\mathbb E[x_t^2]-x_0^2=e^{\bar\beta(t)}-1
\ \Longrightarrow\
\mathbb E[x_t^2]=x_0^2 e^{-\bar\beta(t)}+\big(1-e^{-\bar\beta(t)}\big)
$$

方差 $=\mathbb E[x_t^2]-m(t)^2=x_0^2 e^{-\bar\beta}+(1-e^{-\bar\beta})-x_0^2 e^{-\bar\beta}=1-e^{-\bar\beta(t)}$。漂亮地，$x_0^2$ 项相消，方差与 $x_0$ 无关！

**结论(VP 转移核):**

$$
\boxed{\ p_{t|0}(x_t\mid x_0)=\mathcal N\!\Big(x_t;\ \underbrace{x_0\,e^{-\bar\beta(t)/2}}_{\text{衰减的信号}},\ \underbrace{\big(1-e^{-\bar\beta(t)}\big)I}_{\text{增长的噪声}}\Big)\ }
$$

DDPM 文献里习惯把这个**累积保留率**记作 $\bar\alpha_t:=e^{-\bar\beta(t)}$(头上的横线表示它是"从 $0$ 累积到 $t$"的量；§8.4.3 离散版会看到它等于单步保留率的连乘 $\prod_s\alpha_s$)，则转移核写成更眼熟的形式：

$$
x_t=\sqrt{\bar\alpha_t}\,x_0+\sqrt{1-\bar\alpha_t}\,\varepsilon,\qquad \varepsilon\sim\mathcal N(0,I)
$$

这就是**前向加噪的重参数化(reparameterization)公式**——训练时给定 $x_0$，采一个 $\varepsilon$，一行代码就得到任意时刻的 $x_t$。它是后续一切的基石。本章自始至终用带横线的 $\bar\alpha_t$ 表示"累积"量、用不带横线的 $\alpha_t$ 表示"单步"量(后者在 §8.4.3 离散链里登场)，请勿混淆。

> **本质洞察**:"方差与 $x_0$ 无关"这件事不是巧合，而是 VP-SDE 名字("方差保持")的来源，也是它优于 VE 的工程关键。它意味着不论数据点在哪，加噪到时刻 $t$ 后的不确定性都一样大——神经网络只需学"在固定噪声水平下去噪"，而不必为不同数据点适配不同的噪声尺度。把它和你已知的 batch normalization 对照：两者都是在"消除尺度差异让网络只关注本质信号"。

### 信噪比：理解噪声调度的统一视角

把转移核的两部分相除，定义**信噪比**(Signal-to-Noise Ratio):

$$
\mathrm{SNR}(t)=\frac{\|\text{信号}\|^2}{\text{噪声方差}}=\frac{\bar\alpha_t}{1-\bar\alpha_t}=\frac{e^{-\bar\beta(t)}}{1-e^{-\bar\beta(t)}}
$$

- $t\to0$:$\bar\beta\to0$,$\mathrm{SNR}\to\infty$(几乎是干净数据);
- $t\to T$:$\bar\beta$ 很大，$\mathrm{SNR}\to0$(几乎纯噪声)。

> **本质洞察**：不同论文的噪声调度(DDPM 的线性 $\beta$、Improved DDPM 的余弦调度、EDM 的对数正态)表面上千差万别，但 Kingma et al. (VDM, 2021) 证明了一个深刻的统一：**只要两个调度诱导的 $\mathrm{SNR}(t)$ 端点相同，它们训练出的模型(在连续时间极限下)等价。** 换句话说，调度的"形状"只影响训练时各噪声水平被采样的权重，不改变模型本身的最优解。这把一大堆看似独立的工程技巧统一成了"如何分配训练预算到不同 SNR"这一个问题。这与机器人课程学习(curriculum learning)里"按难度分配训练样本"是同构的思想。

### $p_T$ 真的等于 $\mathcal N(0,I)$ 吗？反事实检验

反事实推理：**如果 $T$ 不够大会怎样？** 由 $m(T)=x_0 e^{-\bar\beta(T)/2}$，均值并非严格为零，而是残留 $e^{-\bar\beta(T)/2}$ 倍的数据信息；方差 $1-e^{-\bar\beta(T)}$ 也并非严格为 $1$。所以 $p_T$ 只是**近似** $\mathcal N(0,I)$，误差量级 $\sim e^{-\bar\beta(T)/2}$。

这个残差正是 §8.4.5 收敛理论里"初始化失配误差"(initialization error)的来源：反向采样从真正的 $\mathcal N(0,I)$ 起步，而理想的反向过程应从 $p_T$ 起步，两者的差距会传播到生成质量。工程上通过把 $\bar\beta(T)$ 取得足够大(让 $e^{-\bar\beta(T)/2}$ 小于 $10^{-3}$)来压制它。**这是理论与工程桥接的典型例子**：一个推导中"约等于"的符号，在实践中对应一个必须调够的超参数。

### ⚠️ 常见陷阱

💡 **概念误区：把前向过程当成"需要训练的网络"**
- 新手想法："前向扩散是不是也要学一个网络把数据变成噪声？"
- 实际上：前向过程**完全没有可学参数**。$f,g$ 是人为设计的固定函数，转移核有闭式高斯解。所有学习都发生在反向过程。
- 为什么重要：理解这一点，你才明白为什么扩散训练只需要一个去噪网络，而不是 GAN 那样的两个对抗网络。前向的"免费"正是扩散稳定性的来源。

💡 **概念误区：漏掉 Itô 公式的二阶项导致方差算错**
- 新手想法：用普通链式法则算 $\mathrm d(x_t^2)=2x_t\,\mathrm dx_t$。
- 实际上：必须用 Itô 公式，多出 $\tfrac12\phi''g^2\,\mathrm dt=\beta\,\mathrm dt$ 项。漏掉它会得到方差 $=x_0^2(e^{-\bar\beta}-1)<0$ 的荒谬结果(负方差)。
- 根本原因：布朗运动二次变差 $(\mathrm dW)^2=\mathrm dt$ 非零。这是随机微积分区别于普通微积分的根本。
- 正确做法：凡是对随机过程的非线性函数求微分，一律上 Itô 公式。

🧠 **思维陷阱：认为 VE 和 VP 哪个"更好"有绝对答案**
- 新手想法："方差爆炸听起来不稳定，VP 肯定全面优于 VE。"
- 实际上：两者是不同权衡。VE 在高分辨率图像上历史表现优异(NCSN++/EDM),VP 在低维结构化数据(如机器人动作)上更常用。EDM(Karras et al. 2022)证明二者可在统一参数化下相互转换，"好坏"取决于预处理(preconditioning)和采样器，而非 SDE 本身。
- 正确思维：选 SDE 时问的是"我的数据维度、动态范围、采样预算如何"，而不是"哪个 SDE 是 SOTA"。

🧠 **思维陷阱：把 $t$ 当成"训练迭代步"而非"噪声水平"**
- 新手想法：看到 $t\in[0,T]$ 以为是优化的时间步。
- 实际上：$t$ 是**扩散时间/噪声水平**，与神经网络的训练 epoch 毫无关系。一次训练迭代里，$t$ 是随机采样的(每个样本配一个随机噪声水平)。
- 为什么重要：混淆这两个"时间"会让你完全读不懂训练循环——训练时同一个 batch 里不同样本处在完全不同的 $t$。

### 练习

1.(推导题，草稿纸上完成)对 **VE-SDE** $\mathrm dx_t=\sqrt{\mathrm d[\sigma^2(t)]/\mathrm dt}\,\mathrm dW_t$，仿照正文 Step 2–3 推导其转移核。提示：漂移为零，均值恒为 $x_0$；用 Itô 等距直接算方差，应得 $p_{t|0}=\mathcal N(x_0,(\sigma^2(t)-\sigma^2(0))I)$。解释为什么这叫"方差爆炸"以及为什么它的 $p_T$ 仍依赖 $x_0$(与 VP 对比)。

2.(开放思考题)VP 转移核中信号系数 $\sqrt{\bar\alpha_t}$ 与噪声系数 $\sqrt{1-\bar\alpha_t}$ 满足 $(\sqrt{\bar\alpha_t})^2+(\sqrt{1-\bar\alpha_t})^2=1$。这个"勾股关系"在 Flow Matching(§8.4.6)的直线插值里会变成线性关系 $\alpha+(1-\alpha)=1$。请猜想：这两种"信号-噪声配比"路径在 $(\sqrt{\bar\alpha},\sqrt{1-\bar\alpha})$ 平面上分别是什么几何曲线？(答案：VP 走单位圆弧，FM 走直线——这正是 FM"路径更直、采样更省步"的几何根源，§8.4.6 详解。)

3.(跨章综合题，综合随机分析 9.1 + 本节)给定 $\beta(t)=\beta_\text{min}+t(\beta_\text{max}-\beta_\text{min})$(DDPM 线性调度，$t\in[0,1]$)。(a) 算出 $\bar\beta(t)$ 的解析式；(b) 写出 $\bar\alpha_t=e^{-\bar\beta(t)}$;(c) 用 Fokker-Planck 方程验证：你算出的高斯转移核 $p_{t|0}$ 确实满足 VP-SDE 对应的前向 Fokker-Planck 方程 $\partial_t p=\tfrac12\beta\partial_x(xp)+\tfrac12\beta\partial_x^2 p$。这一步把"用 SDE 解算分布"和"用 PDE 描述分布演化"两个视角对上，是理解 §8.4.2 反向 SDE 的关键热身。

---

## §8.4.2 Anderson 反向时间 SDE：全章枢纽 ⭐⭐⭐

### 动机：前向容易，反向才是生成

§8.4.1 把数据溶解成噪声，这个方向"免费且简单"。但我们真正想要的是**反过来**：从 $x_T\sim\mathcal N(0,I)$ 出发，一步步去噪，最终得到 $x_0\sim p_\text{data}$。这才是"生成"。问题是：**前向 SDE 倒过来跑，还是一个 SDE 吗？如果是，它的漂移和扩散系数是什么？**

这不是显然的。直觉上，把布朗运动"倒放"会很诡异——布朗运动的路径处处不可导、时间方向上没有记忆，你凭什么相信"倒放"还能写成一个良态的 SDE？更麻烦的是，反向过程似乎需要"预知未来"(知道当前噪点最终会去噪成哪个数据点)，这违反因果性。

Anderson(1982)的定理给出了一个惊人干净的答案，它是整章的**枢纽**：把生成这件事化归为估计**一个函数**。理解了这一节，你就理解了扩散模型为什么能工作；后面所有章节(DDPM、Score Matching、Flow Matching)都是在回答"如何估计这个函数"。

### 如果没有这个定理会怎样

反事实推理：**假如反向过程不是 SDE，我们会面临什么？** 那我们就只能用前向核 $p_{t|0}$ 配贝叶斯公式硬算反向条件分布 $p_{0|t}(x_0\mid x_t)\propto p_{t|0}(x_t\mid x_0)p_\text{data}(x_0)$。但这个式子里有 $p_\text{data}$——正是我们不知道、要去学的东西！而且这个后验通常是高度复杂的多峰分布(给定一个噪点，它可能来自多个不同的数据峰)，没有闭式。于是"反向"会和"直接学 $p_\text{data}$"一样难，迂回毫无意义。

Anderson 定理的威力恰恰在于：它证明反向过程是 SDE，而这个 SDE 的系数只依赖**边际** $p_t$ 的对数梯度(score)，不依赖那个棘手的后验。score 是一个**确定的向量场**，可以用神经网络回归——这就把"学一个复杂的多峰条件分布"换成了"学一个向量场"，后者是良态的监督学习问题。

> **本质洞察**:Anderson 定理是整个扩散模型的"阿基米德支点"。它把一个看似不可能的任务(从噪声逆向生成数据)撬动成一个标准的回归任务(拟合一个向量场 $s(x,t)\approx\nabla\log p_t(x)$)。**生成的全部困难被压缩进一个函数的估计。** 这就是为什么本专题反复强调：估计 score 就是训练，沿 score 走就是采样。把它和你已知的对照——这与最优控制里"用值函数的梯度($\nabla V$，即最优策略方向)把'寻找最优轨迹'化归为'估计一个标量场'"是同一种数学杠杆(这个类比在 §8.4.8 桥接 MPPI 时会精确化)。

### 历史：一个沉睡 38 年的定理

Brian D. O. Anderson 是控制论学者(不是机器学习背景),1982 年在 *Stochastic Processes and their Applications* 发表 "Reverse-time diffusion equation models"，出于滤波与平滑(smoothing)理论的需要——估计理论里常需要"反向时间滤波器"。Haussmann & Pardoux(1986)随后给出了时间反演的严格测度论条件。这个结果在随机过程圈内是标准工具，但在机器学习界沉睡了近 40 年，直到 Song et al.(2021)把它请出来作为 Score SDE 的理论支柱。

> **本质洞察(重申章首脉络)**：这是科学史上反复出现的模式——一个纯理论结果(Anderson 1982)和一个统计技巧(Vincent 2011 去噪 DSM)各自沉睡，直到深度网络提供了"足够好的 score 估计器"才被同时激活。理论提供"化归路径"，统计提供"训练目标"，算力提供"逼近能力"，三者缺一不可。这提醒我们：**很多 SOTA 方法的数学内核早已存在，瓶颈往往不在数学而在"是否有足够强的函数逼近器去填那个被理论留空的洞"。**

### 理论：Anderson 反向时间 SDE 定理

> **定理(Anderson 1982)**。设前向过程 $\mathrm dx_t=f(x_t,t)\,\mathrm dt+g(t)\,\mathrm dW_t$,$t\in[0,T]$，边际密度为 $p_t$(满足适当正则性：$p_t$ 处处正、光滑、衰减足够快)。则存在一个**反向时间**的扩散过程 $\{\bar x_t\}$，它与前向过程有**相同的边际分布**($\bar x_t\sim p_t$ 对所有 $t$)，且满足反向 SDE
> $$
> \boxed{\ \mathrm d\bar x_t=\Big[f(\bar x_t,t)-g(t)^2\,\nabla_x\log p_t(\bar x_t)\Big]\mathrm dt+g(t)\,\mathrm d\bar W_t\ }
> $$
> 其中 $\mathrm dt$ 是**负**的时间增量(从 $T$ 流向 $0$),$\bar W_t$ 是反向时间的布朗运动。函数 $\nabla_x\log p_t(x)$ 称为时刻 $t$ 的 **score(分数函数)**。

这个公式信息量极大，逐项拆解：

- $f(\bar x_t,t)$:**保留**了前向的漂移(在反向时间里它现在推着过程往回走);
- $-g(t)^2\nabla\log p_t$:**新增**的"修正漂移"，正是 score 乘以 $g^2$。它把过程往"高概率区域"拉(score 指向密度增大最快的方向);
- $g(t)\mathrm d\bar W_t$：扩散系数**不变**，仍是 $g(t)$。

> **本质洞察**：反向 SDE 与前向 SDE 的唯一结构差异，就是漂移项里多了 $-g^2\nabla\log p_t$ 这一坨。这意味着：**前向过程"不知道"数据长什么样(它只是无脑加噪)；反向过程要重建数据，就必须注入关于数据分布的全部知识，而这些知识恰好、且仅仅通过 score 这一个量进入。** score 就是数据分布在每个噪声水平下留下的"指纹"。把它与导航类比对照：前向是"随波逐流被洋流冲散"，反向是"看着指南针(score)逆流游回港口"——指南针的读数就是 score，它在每个位置都指向"离数据更近"的方向。类比边界：指南针指向固定磁极，而 score 指向的"数据"方向随 $t$ 和位置都在变，且越接近 $t=0$ 指向越尖锐(多峰分裂)。

### 核心推导：用 Fokker-Planck 时间反演证明 Anderson 定理(完整证明)

我们给出 Song et al. 附录采用的、基于 Fokker-Planck 方程的自洽证明。**证明策略**：两个扩散过程若诱导相同的边际密度演化(同一个 PDE)，则它们边际等价。我们写下反向过程应满足的边际 PDE，反解出它的系数必须是 Anderson 公式那样，从而验证。

**假设**：前向 $\mathrm dx_t=f\,\mathrm dt+g\,\mathrm dW_t$，边际 $p_t(x)$。一维记号(高维同理，把 $\partial_x$ 换成 $\nabla\cdot$ 和 $\nabla$)。

**Step 1：前向 Fokker-Planck(回顾随机分析 9.1)。** 前向边际密度 $p_t$ 满足 Fokker-Planck 方程(也叫 Kolmogorov 前向方程):

$$
\partial_t p_t(x)=-\partial_x\big[f(x,t)p_t(x)\big]+\tfrac12\partial_x^2\big[g(t)^2 p_t(x)\big]
\tag{FP-fwd}
$$

第一项是漂移导致的概率流(probability current)散度，第二项是扩散导致的概率铺开。因 $g$ 不依赖 $x$，第二项 $=\tfrac12 g^2\partial_x^2 p_t$。

**Step 2：把 FP 改写成连续性方程(概率流形式)。** 任何 FP 方程都能写成连续性方程 $\partial_t p=-\partial_x J$，其中 $J$ 是概率流。整理 (FP-fwd):

$$
\partial_t p_t=-\partial_x\Big[\underbrace{f p_t-\tfrac12 g^2\partial_x p_t}_{=:J(x,t)}\Big]
$$

这一步用到 $\tfrac12 g^2\partial_x^2 p_t=\partial_x(\tfrac12 g^2\partial_x p_t)$。

**Step 3：关键代数——把扩散项重写为"有效漂移"。** 用对数导数恒等式

$$
\partial_x p_t=p_t\,\partial_x\log p_t=p_t\,(\nabla\log p_t)
$$

(这正是 score 登场的地方！)代入 $J$:

$$
J=f p_t-\tfrac12 g^2 p_t\nabla\log p_t=\Big(f-\tfrac12 g^2\nabla\log p_t\Big)p_t
$$

于是前向 FP 等价地写成一个**纯漂移(零扩散)**的连续性方程：

$$
\partial_t p_t=-\partial_x\Big[\big(\underbrace{f-\tfrac12 g^2\nabla\log p_t}_{\text{概率流 ODE 速度场}}\big)p_t\Big]
\tag{$\star$}
$$

> **阶段小结**：到这里我们做了一件极漂亮的事——把"漂移 + 扩散"的前向 FP，通过 score 恒等式 $\partial_x p=p\,\partial_x\log p$，等价改写成一个**没有扩散项**的纯输运方程 ($\star$)。这个无扩散版本马上会给出概率流 ODE；而要得到反向 SDE，我们还需要把一部分扩散"加回来"。下面分两路收尾。

**Step 4a：概率流 ODE(确定性反向)。** 方程 ($\star$) 说：边际 $p_t$ 的演化等价于一群粒子按确定性 ODE

$$
\boxed{\ \frac{\mathrm dx}{\mathrm dt}=f(x,t)-\tfrac12 g(t)^2\nabla\log p_t(x)=:v(x,t)\ }
\tag{PF-ODE}
$$

输运的结果(每个粒子是一条确定轨迹，无随机性)。这就是 **概率流 ODE**(Probability Flow ODE, Song et al. 2021)：它与前向/反向 SDE **共享完全相同的边际分布 $p_t$**，但轨迹是光滑确定的。把时间从 $T$ 积分到 $0$，从 $\mathcal N(0,I)$ 的样本出发，就确定性地得到数据样本。DDIM(§下文)和 Flow Matching(§8.4.6)本质上都是在解这个 ODE。

**Step 4b：反向 SDE(随机反向)。** 现在我们要"加回"扩散。诀窍：任意 $\lambda\ge0$，我们可以把 ($\star$) 中的纯漂移拆成"漂移 + 扩散 + 反扩散修正"，使得加入强度为 $\lambda$ 的扩散后边际不变。取 $\lambda=g^2$(即恢复原扩散强度)，可以验证(留作练习，用与 Step 3 相反的代数把 $\tfrac12 g^2\partial_x p$ 拆回去)反向过程

$$
\mathrm d\bar x_t=\Big[f-g^2\nabla\log p_t\Big]\mathrm dt+g\,\mathrm d\bar W_t
$$

诱导的 FP 方程恰好也是 (FP-fwd)(在反向时间里)，故边际与前向一致。这就是 **Anderson 反向 SDE**。$\blacksquare$

> **本质洞察(概率流 ODE 与反向 SDE 的统一)**:Step 4a 和 4b 揭示了一个深刻的"族"结构。Song et al. 进一步证明，对任意 $0\le\lambda\le 1$，反向过程
> $$\mathrm d\bar x_t=\big[f-\tfrac{1+\lambda^2}{2}g^2\nabla\log p_t\big]\mathrm dt+\lambda g\,\mathrm d\bar W_t$$
> 都有**相同的边际 $p_t$**!$\lambda=1$ 是 Anderson SDE(全噪声),$\lambda=0$ 是概率流 ODE(无噪声)，中间是连续过渡。**同一个 score 函数，配不同的随机性强度 $\lambda$，给出一整族采样器**——这正是 DDPM(随机)、DDIM(确定)、以及各种"churn"采样器(EDM)能共用同一个训练好的模型的根本原因。训练一次，采样千变万化。这与机器人里"同一个学到的动力学模型，既能跑确定性 MPC 也能跑随机 MPPI"是同构的(§8.4.8 详述)。

### 概率流 ODE 与反向 SDE 的实践权衡：不是"哪个更好"而是"换什么"

很多人误以为 ODE 采样(确定)一定优于 SDE 采样(随机)。这是"不是 X 而是 Y"型误区。真相是各有得失：

| 维度 | 反向 SDE($\lambda=1$) | 概率流 ODE($\lambda=0$) |
|------|----------------------|------------------------|
| 轨迹 | 随机、粗糙 | 确定、光滑 |
| 采样步数 | 需要多步(随机性要求小步长) | 可用高阶 ODE 求解器，少步数(DDIM 10–50 步) |
| 似然计算 | 不可直接算 | 可！用瞬时变量替换公式算精确对数似然 |
| 误差自我纠正 | 有(噪声把过程拉回高密度区，纠正累积误差) | 无(误差沿确定轨迹累积) |
| 多样性 | 高 | 受初值唯一决定，可重复 |

> **本质洞察**:SDE 采样的随机性不是缺点而是**纠错机制**——每一步注入的噪声会把偏离高密度流形的过程重新"踹回"流形(因为 score 项把它往高概率区拉，而噪声提供探索)。ODE 没有这个纠错，所以 score 估计误差会沿轨迹无情累积。这解释了一个反直觉现象：在 score 估计不够准时，**随机采样反而比确定采样质量更高**。机器人语境：Diffusion Policy 用随机 DDPM 采样，部分原因正是它对 score 网络的不完美更鲁棒；而追求实时性的 $\pi_0$ 用 ODE(Flow Matching)少步采样，接受了"需要更准的速度场"的代价。

### ODE 的额外馈赠：精确算对数似然

上表"似然计算：可！"那一格值得展开——这是概率流 ODE 相对反向 SDE 的独门优势，也是为什么做密度估计/异常检测时偏爱 ODE。

关键工具是**瞬时变量替换公式**(instantaneous change of variables, Chen et al. Neural ODE 2018)：若 $x$ 沿 ODE $\dot x=v(x,t)$ 演化，则它的对数密度沿轨迹的变化率等于速度场散度的负值：

$$
\frac{\mathrm d}{\mathrm dt}\log p_t(x_t)=-\nabla\cdot v(x_t,t)
$$

**直觉**：散度 $\nabla\cdot v$ 衡量流"局部膨胀/收缩"的速率。流膨胀(散度正)→ 概率被摊薄 → 密度下降；流收缩(散度负)→ 概率被压缩 → 密度上升。这正是连续版的"雅可比行列式追踪体积变化"(归一化流的核心)，只不过连续时间下变成了散度的积分。

**算法**：要算某个数据点 $x_0$ 的对数似然 $\log p_0(x_0)$，沿概率流 ODE 从 $t=0$ 积分到 $t=T$(把数据流回噪声)，同时累加散度：

$$
\log p_0(x_0)=\log p_T(x_T)+\int_0^T\nabla\cdot v(x_t,t)\,\mathrm dt
$$

终点 $\log p_T(x_T)$ 是已知的(标准高斯，可解析算)，散度积分沿轨迹数值累加(高维用 Hutchinson 迹估计器随机化散度，避免 $O(d)$ 成本)。

> **本质洞察(为什么 ODE 能算似然而 SDE 不能)**：随机过程的单条轨迹是"一对多"的(同一起点能到不同终点)，无法定义沿轨迹的确定密度变化；而 ODE 是确定流，"一对一"，每个点有唯一轨迹，密度沿轨迹的变化由散度精确刻画。**这是确定性的"代价与馈赠"的另一面：§8.4.2 前面说 ODE 因确定性而无纠错(代价)，这里说 ODE 因确定性而能算似然(馈赠)——同一个'确定'属性，在采样鲁棒性上是劣势，在密度估计上是优势。** 这再次说明 ODE/SDE 不是谁优谁劣，而是不同任务选不同工具。机器人语境：若你要用扩散模型做"这个动作序列正常吗"的异常检测(算 $\log p(A)$ 判断 OOD)，必须走 ODE 路线；若只是生成动作，SDE/ODE 皆可。

### 把抽象公式落地：VP-SDE 的反向 SDE 显式形式

Anderson 公式是一般 $f,g$ 的。我们把它代到 VP-SDE($f=-\tfrac12\beta(t)x$,$g=\sqrt{\beta(t)}$)，得到可直接离散实现的反向 SDE。代入 $\lambda=1$ 的 Anderson 公式：

$$
\mathrm d\bar x_t=\Big[-\tfrac12\beta(t)\bar x_t-\beta(t)\nabla\log p_t(\bar x_t)\Big]\mathrm dt+\sqrt{\beta(t)}\,\mathrm d\bar W_t
$$

把 score 用神经网络 $s_\theta\approx\nabla\log p_t$ 替换，并对时间从 $T$ 到 $0$ 做 Euler-Maruyama 离散(步长 $\Delta t$，反向所以时间倒流)，得到实际采样的迭代：

$$
\bar x_{t-\Delta t}=\bar x_t+\Big[\tfrac12\beta(t)\bar x_t+\beta(t)\,s_\theta(\bar x_t,t)\Big]\Delta t+\sqrt{\beta(t)\,\Delta t}\,z,\qquad z\sim\mathcal N(0,I)
$$

**逐项工程解读**：$+\tfrac12\beta\bar x_t\Delta t$ 抵消前向的回拉(把被拽向原点的过程拉回来);$+\beta\,s_\theta\,\Delta t$ 是 score 引导，把过程推向高密度区(数据);$\sqrt{\beta\Delta t}\,z$ 注入新噪声(纠错探索)。**这三项各司其职，正是反向 SDE 数学结构在代码里的字面映射**——当你在 diffusers 库的采样循环里看到三行加法，它们一一对应这三项。

> **阶段小结**：到这里，从抽象的 Anderson 定理，我们已经落地到一行可执行的采样更新。剩下唯一的未知是 $s_\theta$——它怎么训练出来，是下两节的事。

### 与 NCSN/退火 Langevin 的连接：另一条通往同一采样器的路

Song & Ermon(NCSN, 2019)早于 Score SDE，从一条完全不同的路得到了"沿 score 走 + 注噪"的采样器，值得对照(它也是 VE-SDE 的前身，且与 §8.4.8 的 MPPI 退火直接同构)。

**Langevin 动力学**：若已知某分布 $p$ 的 score $\nabla\log p$，则如下迭代(Langevin MCMC)的平稳分布恰是 $p$:

$$
x_{k+1}=x_k+\tfrac{\eta}{2}\nabla\log p(x_k)+\sqrt\eta\,z_k,\qquad z_k\sim\mathcal N(0,I)
$$

直觉：漂移项 $\tfrac{\eta}{2}\nabla\log p$ 把粒子推向高密度区(梯度上升)，噪声项防止它塌缩到单个最大值点(保持采样多样性而非求最优)。这与上面 VP 反向 SDE 的离散形式**结构同构**——都是"score 漂移 + 高斯噪声"。

**问题**：直接对数据分布 $p_\text{data}$ 跑 Langevin 会失败，原因有二：(1) 真实数据集中在低维流形上，流形外 score 估计极差(没有训练样本);(2) 多峰分布的峰间低密度区，Langevin 混合极慢(困在一个峰里，这正是 §8.4.5 说的"无 LSI"困境)。

**NCSN 的解法——退火(annealing)**：不直接采 $p_\text{data}$，而是采一串噪声水平递减的加噪分布 $p_{\sigma_1},p_{\sigma_2},\dots,p_{\sigma_L}$($\sigma_1$ 大、$\sigma_L\to0$)。先在大噪声(平滑、单峰、易采)下跑几步 Langevin，把结果当下一个(更小噪声)分布的初值，逐级退火直到 $\sigma_L$:

> **本质洞察(退火 = 反向扩散的离散精神)**:NCSN 的"多噪声水平退火 Langevin"和 Score SDE 的"反向 SDE 从 $T$ 积分到 $0$"是**同一件事的两种语言**——噪声水平 $\sigma$ 就是扩散时间 $t$，退火就是反向积分。大噪声端分布平滑单峰、易混合；逐级降噪相当于反向去噪，峰在"还连通"时被引导到正确吸引域，降到 $\sigma_L$ 时各峰已分离锁定。**这就是扩散为什么能采多峰而朴素 Langevin 不能的全部秘密：不是在固定分布上硬采，而是沿一条从易到难的噪声路径走。** 这个洞察是 §8.4.5"无需 LSI"和 §8.4.8"多级退火 MPPI 突破高斯天花板"的共同源头——三者是同一思想在采样理论、收敛理论、机器人控制三个领域的化身。

类比边界：退火 Langevin 与金属退火(物理)**像**在都"高温/大噪声时全局探索，降温/小噪声时局部锁定";**不像**在物理退火是被动随机游走找能量极小，而这里有 score 主动引导且每个噪声水平的目标分布被前向过程精确设计——不是找单个极小值，而是采出整个多峰分布。

### 一个还没解决的问题：score 从哪来？

Anderson 定理把生成化归为"知道 score $\nabla\log p_t$"。但 $p_t$ 是数据分布加噪后的边际，我们并不知道它的解析式(它涉及对未知 $p_\text{data}$ 的卷积)，自然也不知道它的对数梯度。**这正是 §8.4.3(DDPM)和 §8.4.4(Score Matching)要解决的全部内容：如何用神经网络 $s_\theta(x,t)$ 估计这个未知的 score。** 本节是"为什么估计 score 就够了"的答案，下两节是"如何估计"的答案：本节制造了对 score 估计的需求，下两节正好满足它。

### ⚠️ 常见陷阱

💡 **概念误区：以为反向过程需要知道 $p_{0|t}$ 后验**
- 新手想法："反向就是给定 $x_t$ 推 $x_0$，那不就是要算后验 $p(x_0\mid x_t)$ 吗？"
- 实际上：Anderson 定理只需要**边际** score $\nabla\log p_t(x_t)$，不需要完整后验。score 是边际密度的对数梯度，可由去噪目标学到(§8.4.4)；后验是复杂多峰分布，无需显式建模。
- 为什么重要：这正是迂回的精髓——用易学的边际 score 替代难学的后验。

💡 **概念误区：把 score $\nabla_x\log p_t$ 当成对参数 $\theta$ 的梯度**
- 新手想法：看到 $\nabla\log p$ 联想到极大似然里的对数似然梯度 $\nabla_\theta\log p_\theta$。
- 实际上：score 是对**数据变量 $x$** 求梯度，不是对参数。它是一个 $\mathbb R^d\to\mathbb R^d$ 的向量场，告诉你"在 $x$ 点，往哪个方向走密度增长最快"。
- 根本原因：统计学里"score"一词历史上指 $\nabla_\theta\log p_\theta$(Fisher score),Hyvärinen 2005 把它借用到 $\nabla_x\log p$，机器学习沿用了后者。同名不同物。
- 正确做法：在扩散语境永远把 score 理解为"数据空间的密度梯度向量场"。

🧠 **思维陷阱：认为概率流 ODE 是"反向 SDE 的近似"**
- 新手想法："ODE 去掉了噪声项，应该是 SDE 的简化近似，精度更低。"
- 实际上：ODE 与 SDE 在**边际分布层面完全等价**(共享所有 $p_t$)，不是近似关系。差异只在单条轨迹的随机性，不在边际。用精确 score 时，两者采出的样本都严格服从 $p_\text{data}$。
- 为什么重要：理解了这点，你才不会以为"用 ODE 采样是在牺牲质量换速度"——速度提升来自 ODE 轨迹光滑可用高阶求解器，而非牺牲了分布正确性(误差来自有限步+score 不准，不来自 ODE 本身)。

🧠 **思维陷阱：把"反向时间布朗运动 $\bar W$"和前向 $W$ 当成同一个**
- 新手想法："倒着跑，布朗运动也倒着用同一个 $W$。"
- 实际上：$\bar W$ 是相对反向时间滤波(filtration)的新布朗运动，不是前向 $W$ 的逐路径倒放。布朗运动的时间反演涉及测度变换(Haussmann-Pardoux 1986 的技术核心)。
- 正确做法：实现层面无需纠结——离散化时反向每步独立采新高斯噪声即可；但理论上要知道这是个独立的反向鞅，否则在 Girsanov 证明(§8.4.7)里会犯错。

### 练习

1.(证明题，草稿纸上完成)补全正文 Step 4b 跳过的代数：从概率流形式 ($\star$)，验证反向 SDE $\mathrm d\bar x_t=[f-g^2\nabla\log p_t]\mathrm dt+g\,\mathrm d\bar W_t$ 诱导的 FP 方程确实等于 (FP-fwd)。提示：写出该反向 SDE 的 FP，用 $\partial_x p=p\nabla\log p$ 把 $g^2\nabla\log p\cdot p$ 项与扩散项 $\tfrac12 g^2\partial_x^2 p$ 合并。

2.(推导题)对一般 $\lambda\in[0,1]$ 族 $\mathrm d\bar x=[f-\tfrac{1+\lambda^2}{2}g^2\nabla\log p_t]\mathrm dt+\lambda g\,\mathrm d\bar W$，证明其 FP 方程与前向一致(故边际不变)。这一步让你亲手验证"同一 score、不同噪声强度共享边际"这个统一结论。

3.(开放思考题)概率流 ODE 速度场 $v=f-\tfrac12 g^2\nabla\log p_t$ 中，前向漂移 $f$ 和 score 项符号相反地"拔河"。在 VP-SDE 下($f=-\tfrac12\beta x$)，写出 $v$ 的显式表达，并解释：在 $t\to0$(接近数据)时哪一项主导？在 $t\to T$(接近噪声)时哪一项主导？这个主导权的切换如何对应"先粗定位数据峰、再精修细节"的采样直觉？(此题为 §8.4.6 Flow Matching 的速度场做铺垫。)

---

## §8.4.3 DDPM：从 ELBO 到 $\varepsilon$-预测 ⭐⭐⭐

### 动机：连续 SDE 很美，但怎么训练？

§8.4.2 告诉我们采样只需 score。但 Anderson 是连续时间的优雅理论，而真正点燃扩散模型革命的 DDPM(Ho et al. 2020)走的是**离散时间 + 变分推断**的路子，且早于 Song 的 SDE 框架。为什么要单独学 DDPM 的离散视角？三个理由：

1. **工程实现是离散的**:Diffusion Policy、$\pi_0$ 的早期版本、几乎所有代码库都用离散步 $t\in\{1,\dots,N\}$。读懂论文里的 $L_\text{simple}=\mathbb E\|\varepsilon-\varepsilon_\theta\|^2$ 必须懂 DDPM 推导。
2. **它从"概率建模第一性原理"出发**:DDPM 把扩散看成一个**层次隐变量模型**(hierarchical latent variable model)，用 ELBO(证据下界)训练——这条路不依赖 SDE，提供了独立的、更"机器学习"的理解。
3. **它揭示了"$\varepsilon$-预测"这个工程上至关重要的参数化**——为什么"预测噪声"比"预测均值"或"预测数据"训练更稳，是一个有深刻数学根源的工程智慧。

> **本质洞察(两条路殊途同归)**:DDPM(变分/离散)和 Score SDE(score/连续)是同一座山的两条登山道。Ho 从 ELBO 出发得到"预测噪声 $\varepsilon$"的损失；Song 从反向 SDE 出发得到"预测 score"的损失。§8.4.4 的 Tweedie 公式会证明：**$\varepsilon$、score、干净数据 $x_0$ 三者是同一个量的三种坐标，差一个确定的线性变换。** 所以两条路训练出的是本质相同的模型。先把这个结论记住，本节末和 §8.4.4 会精确兑现它。这种"不同出发点殊途同归"在物理里也常见——Lagrange 力学和 Hamilton 力学描述同一个世界。

### 如果不用 ELBO 直接最大似然会怎样

反事实：我们想最大化 $\log p_\theta(x_0)$，但 $p_\theta(x_0)=\int p_\theta(x_{0:T})\,\mathrm dx_{1:T}$ 是对所有中间隐变量 $x_{1:T}$ 的高维积分，**无法解析、无法直接优化**(这是所有隐变量模型的通病)。强行蒙特卡洛估计这个积分，方差爆炸到无法训练。

ELBO 的作用就是绕过这个不可解的积分：用 Jensen 不等式构造一个**可计算的下界**，最大化下界来间接最大化似然。这与 VAE 用 ELBO 是同一招——但扩散的 ELBO 有一个 VAE 没有的巨大优势：**后验 $q(x_{1:T}\mid x_0)$ 是已知的固定高斯链(就是前向加噪！)，不需要学一个编码器去近似它。** 这消除了 VAE 中"编码器近似后验"的一大误差来源，是扩散 ELBO 比 VAE 更紧、训练更稳的根本。

类比边界：扩散是 VAE 的"无限层、固定编码器"极限——**像**在都用 ELBO、都有隐变量；**不像**在 VAE 的编码器是学出来的单层、扩散的"编码器"是写死的多步高斯链。不要把"VAE 模糊"的印象套到扩散——正因为编码器固定，扩散没有 VAE 那种后验坍缩(posterior collapse)问题。

### 历史

DDPM 直接继承 Sohl-Dickstein et al.(2015)的非平衡热力学框架，但 Ho et al.(2020)做了三个关键改进让质量飞跃：(i) 把均值参数化重写为 $\varepsilon$-预测；(ii) 用简化损失 $L_\text{simple}$(去掉 ELBO 各项的权重)替代完整 ELBO，发现反而更好；(iii) 用 U-Net + 注意力的架构。这三点里，(i)(ii) 是数学，本节讲透；(iii) 是架构，属深度学习常识。

### 理论：前向链与反向链

DDPM 把 $[0,T]$ 离散成 $N$ 步，记 $x_0,x_1,\dots,x_N$。沿用 §8.4.1 的记号但改用离散下标。**前向链**(固定，无参数)是马氏链：

$$
q(x_t\mid x_{t-1})=\mathcal N\big(x_t;\sqrt{1-\beta_t}\,x_{t-1},\beta_t I\big),\qquad
q(x_{1:N}\mid x_0)=\prod_{t=1}^N q(x_t\mid x_{t-1})
$$

记 $\alpha_t:=1-\beta_t$,$\bar\alpha_t:=\prod_{s=1}^t\alpha_s$(注意：这里 $\bar\alpha_t$ 是离散累乘，对应 §8.4.1 连续的 $e^{-\bar\beta(t)}$)。由高斯链的可加性(把 §8.4.1 的重参数化离散化)，给定 $x_0$ 可一步跳到任意 $x_t$:

$$
\boxed{\ q(x_t\mid x_0)=\mathcal N\big(x_t;\sqrt{\bar\alpha_t}\,x_0,(1-\bar\alpha_t)I\big)\ \Longleftrightarrow\ x_t=\sqrt{\bar\alpha_t}\,x_0+\sqrt{1-\bar\alpha_t}\,\varepsilon\ }
\tag{FWD}
$$

**反向链**(可学，带参数 $\theta$)也设为高斯马氏链(这是建模假设，其合理性见下文"为什么反向也是高斯"):

$$
p_\theta(x_{t-1}\mid x_t)=\mathcal N\big(x_{t-1};\mu_\theta(x_t,t),\Sigma_\theta(x_t,t)\big),\qquad
p_\theta(x_{0:N})=p(x_N)\prod_{t=1}^N p_\theta(x_{t-1}\mid x_t)
$$

其中 $p(x_N)=\mathcal N(0,I)$。训练目标：让 $p_\theta(x_0)$ 尽量大。

> **为什么反向单步也能设成高斯？** 这是一个关键且常被跳过的点。反向单步 $p(x_{t-1}\mid x_t)$ 一般不是高斯。但 Sohl-Dickstein 证明：**当每步噪声 $\beta_t$ 足够小(步数 $N$ 足够多)时，反向单步条件分布趋于高斯。** 直觉：小步加噪近似可逆，逆也接近线性高斯。这就是为什么 DDPM 需要很多步(原版 $N=1000$)——不是为了"加够噪"，而是为了让"每步反向是高斯"这个假设成立。**这又一次印证了前置自测第 5 题：表达力来自多步复合，单步必须简单(高斯)。** 步数少了(如 DDIM 想 10 步)，就不能再依赖马氏高斯假设，必须换确定性 ODE 视角(§下文 DDIM)。

### 核心推导：ELBO 的分解(完整证明)

**目标**：最大化 $\log p_\theta(x_0)$。**Step 1:Jensen 构造下界。** 引入固定后验 $q(x_{1:N}\mid x_0)$ 作为重要性分布：

$$
\log p_\theta(x_0)=\log\int p_\theta(x_{0:N})\mathrm dx_{1:N}
=\log\,\mathbb E_{q}\!\left[\frac{p_\theta(x_{0:N})}{q(x_{1:N}\mid x_0)}\right]
\ge\mathbb E_q\!\left[\log\frac{p_\theta(x_{0:N})}{q(x_{1:N}\mid x_0)}\right]=:\mathcal L_\text{ELBO}
$$

不等号是 Jensen($\log$ 凹)。最大化 $\mathcal L_\text{ELBO}$ 即间接最大化似然，等价于最小化 $-\mathcal L_\text{ELBO}$。

**Step 2：展开比值。** 代入两条链的乘积形式：

$$
-\mathcal L_\text{ELBO}=\mathbb E_q\!\left[-\log p(x_N)-\sum_{t=1}^N\log\frac{p_\theta(x_{t-1}\mid x_t)}{q(x_t\mid x_{t-1})}\right]
$$

**Step 3：关键技巧——用贝叶斯把 $q(x_t\mid x_{t-1})$ 改写为以 $x_0$ 为条件的可逆形式。** 直接的 $q(x_t\mid x_{t-1})$ 难配对，因为反向是 $p_\theta(x_{t-1}\mid x_t)$。我们用贝叶斯引入 $x_0$:

$$
q(x_t\mid x_{t-1})=q(x_t\mid x_{t-1},x_0)=\frac{q(x_{t-1}\mid x_t,x_0)\,q(x_t\mid x_0)}{q(x_{t-1}\mid x_0)}
$$

(第一个等号因前向马氏性，$x_t$ 给定 $x_{t-1}$ 后与 $x_0$ 无关。)代入并让求和中的 $q(x_t\mid x_0)/q(x_{t-1}\mid x_0)$ 形成望远镜求和(telescoping)，整理后(标准但繁琐的代数，逐项配对)得到 DDPM 论文的核心分解：

$$
\boxed{\ -\mathcal L_\text{ELBO}=\underbrace{D_\text{KL}\big(q(x_N\mid x_0)\,\|\,p(x_N)\big)}_{L_N\text{：先验匹配}}+\sum_{t=2}^N\underbrace{\mathbb E_q D_\text{KL}\big(q(x_{t-1}\mid x_t,x_0)\,\|\,p_\theta(x_{t-1}\mid x_t)\big)}_{L_{t-1}\text{：去噪匹配}}\underbrace{-\mathbb E_q\log p_\theta(x_0\mid x_1)}_{L_0\text{：重构}}\ }
$$

**逐项物理意义**:

- $L_N$：前向终点 $q(x_N\mid x_0)$ 与先验 $\mathcal N(0,I)$ 的 KL。无参数(前向固定)，只衡量"加噪是否充分"，训练中是常数，可忽略。这正是 §8.4.1 末"$p_T$ 残差"的离散对应。
- $L_{t-1}$($t=2,\dots,N$):**主力项**。要求反向单步 $p_\theta(x_{t-1}\mid x_t)$ 匹配**前向后验** $q(x_{t-1}\mid x_t,x_0)$。后者是可解析计算的高斯(下面算)!
- $L_0$：最后一步从 $x_1$ 重构 $x_0$ 的对数似然。

> **阶段小结**：到这里我们把"不可解的似然"拆成了一串 KL 散度，且每个 KL 都是两个高斯之间的 KL——而两个高斯的 KL 有闭式！主力项 $L_{t-1}$ 要求反向网络匹配前向后验 $q(x_{t-1}\mid x_t,x_0)$。下一步：算出这个后验，然后把 KL 写成均值的平方差。

**Step 4：计算前向后验 $q(x_{t-1}\mid x_t,x_0)$。** 这是一个高斯(高斯的贝叶斯组合仍高斯)。用 (FWD) 和贝叶斯，配方(complete the square)可得：

$$
q(x_{t-1}\mid x_t,x_0)=\mathcal N\big(x_{t-1};\tilde\mu_t(x_t,x_0),\tilde\beta_t I\big)
$$

$$
\tilde\mu_t(x_t,x_0)=\frac{\sqrt{\bar\alpha_{t-1}}\,\beta_t}{1-\bar\alpha_t}\,x_0+\frac{\sqrt{\alpha_t}\,(1-\bar\alpha_{t-1})}{1-\bar\alpha_t}\,x_t,
\qquad
\tilde\beta_t=\frac{1-\bar\alpha_{t-1}}{1-\bar\alpha_t}\beta_t
$$

**Step 5:KL 化为均值平方差。** 两个同协方差高斯 $\mathcal N(\tilde\mu_t,\tilde\beta_t I)$ 与 $\mathcal N(\mu_\theta,\tilde\beta_t I)$(DDPM 固定 $\Sigma_\theta=\tilde\beta_t I$)的 KL 散度有闭式：

$$
L_{t-1}=\mathbb E_q\left[\frac{1}{2\tilde\beta_t}\big\|\tilde\mu_t(x_t,x_0)-\mu_\theta(x_t,t)\big\|^2\right]+\text{const}
$$

所以训练目标就是让网络预测的均值 $\mu_\theta$ 逼近解析后验均值 $\tilde\mu_t$。**到此为止是"均值预测"参数化。** 下面是 Ho et al. 的点睛之笔。

### 核心推导：从均值预测到 $\varepsilon$-预测(完整证明)

**关键代换**：由 (FWD),$x_0=\frac{1}{\sqrt{\bar\alpha_t}}(x_t-\sqrt{1-\bar\alpha_t}\,\varepsilon)$。把它代入 $\tilde\mu_t$ 的表达式，经代数化简(把 $x_0$ 用 $x_t,\varepsilon$ 表示再合并 $x_t$ 系数)，得到一个只含 $x_t$ 和噪声 $\varepsilon$ 的形式：

$$
\tilde\mu_t(x_t,x_0)=\frac{1}{\sqrt{\alpha_t}}\Big(x_t-\frac{\beta_t}{\sqrt{1-\bar\alpha_t}}\,\varepsilon\Big)
$$

**这个形式提示我们：与其让网络预测均值 $\mu_\theta$，不如让它预测噪声 $\varepsilon$!** 设网络输出 $\varepsilon_\theta(x_t,t)$，并令反向均值取同样结构：

$$
\mu_\theta(x_t,t)=\frac{1}{\sqrt{\alpha_t}}\Big(x_t-\frac{\beta_t}{\sqrt{1-\bar\alpha_t}}\,\varepsilon_\theta(x_t,t)\Big)
$$

代入 Step 5 的 $L_{t-1}$,$x_t$ 项相消，只剩 $\varepsilon$ 与 $\varepsilon_\theta$ 的差：

$$
L_{t-1}=\mathbb E_{x_0,\varepsilon}\left[\frac{\beta_t^2}{2\tilde\beta_t\,\alpha_t(1-\bar\alpha_t)}\big\|\varepsilon-\varepsilon_\theta(\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon,\,t)\big\|^2\right]
$$

**Step 6：简化损失。** Ho et al. 发现，**丢掉前面那个复杂的权重系数**(令其为 1)，训练效果反而更好(它等价于对不同 $t$ 重新加权，强调了更难的高噪声步):

$$
\boxed{\ L_\text{simple}(\theta)=\mathbb E_{t\sim\mathcal U\{1,N\},\,x_0\sim p_\text{data},\,\varepsilon\sim\mathcal N(0,I)}\Big[\big\|\varepsilon-\varepsilon_\theta(\sqrt{\bar\alpha_t}\,x_0+\sqrt{1-\bar\alpha_t}\,\varepsilon,\,t)\big\|^2\Big]\ }
$$

$\blacksquare$ 这就是你在 Diffusion Policy 论文里看到的那个损失！训练流程极简：(1) 采数据 $x_0$;(2) 采时间步 $t$;(3) 采噪声 $\varepsilon$;(4) 合成 $x_t$;(5) 网络预测 $\varepsilon_\theta$;(6) 最小化 $\|\varepsilon-\varepsilon_\theta\|^2$。

### 为什么"预测噪声"比"预测均值/数据"工程上更稳

"预测什么"是一个有标准答案的工程问题，而这个答案并不直观。三种等价参数化(§8.4.4 Tweedie 会证它们等价)：预测均值 $\mu_\theta$、预测数据 $x_0$、预测噪声 $\varepsilon$。数学上等价，工程上 $\varepsilon$-预测胜出，原因：

1. **目标尺度恒定**:$\varepsilon\sim\mathcal N(0,I)$ 在所有 $t$ 都是单位方差。而 $x_0$ 的尺度依数据而定、$\mu_\theta$ 的尺度依 $t$ 剧烈变化。恒定尺度让网络输出归一化、梯度稳定。
2. **高噪声区不退化**:$t\to N$ 时 $x_t$ 几乎纯噪声，预测 $x_0$ 几乎是"无中生有"(输入无信息)，网络只能输出数据均值，损失饱和；但预测 $\varepsilon$ 时，网络只需"认出输入里的噪声成分"，任务在所有 $t$ 都良态。
3. **与 score 的简单关系**：由 Tweedie(§8.4.4),$\nabla\log p_t(x_t)=-\varepsilon_\theta/\sqrt{1-\bar\alpha_t}$,$\varepsilon$-预测直接给出 score，无缝接入 §8.4.2 的采样器。

> **本质洞察**:$\varepsilon$-预测的优越性揭示了一个普适的工程原则——**回归目标的"条件数/尺度一致性"比目标的"语义直观性"更重要**。"预测干净数据"语义上最直观，却因尺度随噪声水平剧变而难训；"预测噪声"语义上绕，却因目标恒为单位高斯而稳。这与机器人控制里"宁可回归归一化的残差/增量，也不回归绝对量"是同一智慧(例如学动作增量 $\Delta a$ 而非绝对位置，正是 Diffusion Policy 的常见做法)。

### 噪声调度的选择：线性 vs 余弦，以及它对训练的真实影响

DDPM 原文用**线性调度** $\beta_t$ 从 $10^{-4}$ 线性增到 $0.02$。Nichol & Dhariwal(Improved DDPM, 2021)发现它有个缺陷，并提出**余弦调度**——这是一个能体现"调度如何影响训练"的具体案例。

**线性调度的问题**：用 §8.4.1 的 SNR 视角看，线性 $\beta_t$ 让累积 $\bar\alpha_t$ 在末段下降过快——到 $t$ 接近 $N$ 时，$\bar\alpha_t$ 几乎已是 $0$，信息早就被噪声淹没。这意味着**最后那批高噪声步几乎是"纯噪声去纯噪声"，对学习几乎无贡献，白白浪费了训练预算**；同时低频结构信息在前段就过早丢失，损害质量。

**余弦调度的修正**:Nichol-Dhariwal 直接设计 $\bar\alpha_t$ 的形状(而非 $\beta_t$):

$$
\bar\alpha_t=\frac{f(t)}{f(0)},\qquad f(t)=\cos^2\!\Big(\frac{t/N+s}{1+s}\cdot\frac{\pi}{2}\Big)
$$

($s$ 是小偏移防 $t=0$ 处奇异)。它让 $\bar\alpha_t$ 在两端平缓、中段下降——使各噪声水平被"更均匀地"利用，中段(感知上最重要)分配到更多有效训练。

> **本质洞察(调度 = 训练预算在噪声水平上的分配)**：回顾 §8.4.1 的 VDM 统一结论——只要 SNR 端点相同，模型最优解不变，调度只影响"各噪声水平被采样的权重"。所以**选调度本质上是在回答"我要在哪些噪声水平上花更多训练力气"**。线性调度在高噪声端浪费预算；余弦调度把预算挪到中段。**这不是"哪个调度更对"，而是"训练预算往哪分配"的工程决策**——和机器人课程学习"按难度分配训练样本"、主动学习"在信息量大的样本上多花标注"是同一思想。把调度理解成"预算分配器"而非"加噪配方"，你才能针对自己的数据(如机器人动作的 SNR 特性可能与图像很不同)定制调度。

类比边界：噪声调度与"考试复习计划"**像**在都"决定在哪个难度区间投入多少时间";**不像**在复习计划影响你最终会什么(学到的内容)，而调度(在 VDM 意义下)不改变模型能学到的最优解，只改变到达它的优化效率。不要以为"换个好调度能让模型学到本质上更强的东西"——它只是让你更快、更稳地到达同一个最优。

**机器人语境的提示**：图像扩散的调度经验未必直接适用于机器人动作。动作序列的"频率成分"和"有效 SNR 范围"与自然图像差异很大(动作更平滑、低频主导)，所以 Diffusion Policy 等工作常需重新调整调度或干脆用更少的扩散步数 + DDIM。盲目照搬图像调度是常见失误。

### DDIM：把 DDPM 从 1000 步压到 50 步

DDPM 采样要 $N=1000$ 步，太慢——机器人控制要 50 Hz，根本等不起。Song, Meng, Ermon(ICLR 2021)的 DDIM(Denoising Diffusion Implicit Models)给出救命方案。核心洞察：**$L_\text{simple}$ 只依赖边际 $q(x_t\mid x_0)$，不依赖前向是不是马氏链！** 所以可以构造一族**非马氏**前向过程，它们有相同的边际(故可复用同一个训练好的 $\varepsilon_\theta$)，但反向可以"跳步"。

DDIM 反向更新(用预测的 $\hat x_0=\frac{1}{\sqrt{\bar\alpha_t}}(x_t-\sqrt{1-\bar\alpha_t}\varepsilon_\theta)$):

$$
x_{t-1}=\sqrt{\bar\alpha_{t-1}}\,\hat x_0+\sqrt{1-\bar\alpha_{t-1}-\sigma_t^2}\,\varepsilon_\theta(x_t,t)+\sigma_t\,z,\quad z\sim\mathcal N(0,I)
$$

参数 $\sigma_t$ 控制随机性：$\sigma_t=$（特定值）退化回 DDPM(随机);$\sigma_t=0$ 给出**完全确定**的采样——而这个确定版本正是 §8.4.2 概率流 ODE 的离散化(用 Euler 解 PF-ODE)!

> **本质洞察(DDIM = 概率流 ODE 的离散积分)**:DDIM 的 $\sigma_t=0$ 版本不是一个独立技巧，而是 §8.4.2 那个 $\lambda=0$ 概率流 ODE 的一阶数值积分。这就解释了 DDIM 为什么能少步：**确定性 ODE 轨迹光滑，可以用大步长积分而不发散**(就像解光滑 ODE 可以用大步长 RK4)；而 DDPM 的随机轨迹粗糙，大步长会引入过大误差。这把"DDPM vs DDIM"从两个孤立算法统一成"同一 score、$\lambda=1$ 随机积分 vs $\lambda=0$ 确定积分"。一图胜千言：训练得到 $\varepsilon_\theta$ → 它给出 score → 你自由选择用哪种积分器采样。

### ⚠️ 常见陷阱

💡 **概念误区：以为 DDPM 的 1000 步是"加噪需要这么多步"**
- 新手想法："步数多是为了把数据彻底变成噪声。"
- 实际上：加噪充分性由 $\bar\alpha_N\approx0$ 保证，几十步的累积噪声就够。1000 步是为了让"反向单步是高斯"这个建模假设成立(小 $\beta_t$)。DDIM 用 ODE 视角绕过马氏假设，故能跳步。
- 为什么重要：理解了这点，你才知道"减步数"要换 DDIM/ODE 思路，而不是简单调大 $\beta_t$(那会破坏高斯假设)。

💡 **概念误区：把 $L_\text{simple}$ 里去掉的权重当成"近似/不严谨"**
- 新手想法："扔掉 ELBO 的权重系数，损失就不是真的下界了，这不是不严谨吗？"
- 实际上：$L_\text{simple}$ 确实不再是 ELBO，但它是一个**重新加权的**目标，Ho et al. 实证它的样本质量更好(它降低了低噪声步的权重，让网络多关注感知上重要的中噪声步)。Kingma VDM 进一步给出连续时间下"权重不影响最优解、只影响优化路径"的理论。
- 正确做法：追求似然(如做密度估计)时用带权完整 ELBO；追求样本质量(如生成、机器人策略)时用 $L_\text{simple}$。

🧠 **思维陷阱：把反向链高斯假设当成"扩散的固有限制"**
- 新手想法："既然反向单步是高斯，扩散模型表达力是不是受限于高斯？"
- 实际上：单步高斯，但 $N$ 步复合后的边际 $p_\theta(x_0)$ 可以是任意复杂多峰分布(前置自测 5 的答案)。表达力在复合，不在单步。
- 为什么重要：这消除了"高斯=简单=表达力弱"的错误推断，也解释了为什么 Diffusion Policy 能建模多峰动作分布(从左/从右抓取)，而高斯策略(单峰)做不到。

🧠 **思维陷阱：混淆 DDIM 的 $\sigma_t=0$ 与"没有噪声所以没有多样性"**
- 新手想法："DDIM 确定性采样，那生成的样本岂不是都一样？"
- 实际上：DDIM 的随机性全部来自**初值** $x_N\sim\mathcal N(0,I)$。不同初值经确定 ODE 映到不同样本，多样性来自初值采样而非过程噪声。固定初值才会得到固定输出(这恰是 DDIM 可用于插值、可复现的优点)。
- 正确做法：要多样性就采多个初值；要可复现/可控就固定初值——ODE 视角让"初值→样本"成为确定映射。

### 练习

1.(推导题，草稿纸上完成)补全 Step 4 的配方：用 (FWD) 写出 $q(x_t\mid x_0)$ 和 $q(x_{t-1}\mid x_0)$，结合马氏前向核 $q(x_t\mid x_{t-1})$，通过贝叶斯 $q(x_{t-1}\mid x_t,x_0)\propto q(x_t\mid x_{t-1})q(x_{t-1}\mid x_0)$ 配方，独立推出 $\tilde\mu_t$ 和 $\tilde\beta_t$。这是 DDPM 推导的技术核心，务必亲手算一遍。

2.(推导题)从 $\tilde\mu_t(x_t,x_0)=\frac{\sqrt{\bar\alpha_{t-1}}\beta_t}{1-\bar\alpha_t}x_0+\frac{\sqrt{\alpha_t}(1-\bar\alpha_{t-1})}{1-\bar\alpha_t}x_t$ 出发，代入 $x_0=\frac{1}{\sqrt{\bar\alpha_t}}(x_t-\sqrt{1-\bar\alpha_t}\varepsilon)$，化简验证得到正文的 $\tilde\mu_t=\frac{1}{\sqrt{\alpha_t}}(x_t-\frac{\beta_t}{\sqrt{1-\bar\alpha_t}}\varepsilon)$。注意系数合并时要用 $\bar\alpha_t=\alpha_t\bar\alpha_{t-1}$。

3.(跨章综合题，综合 §8.4.2 + §8.4.3)证明 DDIM 的 $\sigma_t=0$ 更新在连续极限($N\to\infty$，步长 $\to0$)下趋于概率流 ODE (PF-ODE) 的 Euler 离散。提示：把 $\hat x_0$ 和 $\varepsilon_\theta$ 用 score 表示(用 Tweedie 关系 $\varepsilon_\theta=-\sqrt{1-\bar\alpha_t}\,s_\theta$)，再把 DDIM 更新做一阶 Taylor 展开，对照 PF-ODE 的 Euler 步。这道题把 DDPM 的离散世界和 SDE 的连续世界正式焊接起来。

---

## §8.4.4 Score Matching 家族与 Tweedie 公式 ⭐⭐⭐⭐

### 动机：怎么学一个我们写不出来的梯度

§8.4.2 把生成化归为"知道 score $\nabla\log p_t(x)$";§8.4.3 从变分角度给出了 $\varepsilon$-预测损失。本节回答最根本的问题：**给定一堆样本，怎么训练一个网络 $s_\theta(x)\approx\nabla\log p(x)$，而我们根本不知道 $p$ 的解析式？**

天真的想法：最小化 $\mathbb E_{p}\|s_\theta(x)-\nabla\log p(x)\|^2$。但这个目标里有未知的 $\nabla\log p(x)$——我们没有它的标签，无法直接回归！这就是 score 估计的核心困难。本节讲三种破解它的方法，以及把它们串起来的 Tweedie 公式。

为什么这是博士级(⭐⭐⭐⭐)内容？因为它涉及一个深刻的数学魔术：**通过分部积分，把"含未知 $\nabla\log p$ 的目标"变形成"不含它、只含 $s_\theta$ 自身导数的目标"**——未知量神奇地消失了。理解这个魔术，是理解所有现代生成模型(扩散、score-based、能量模型)训练原理的关键。

### 如果绕不开配分函数会怎样

反事实推理，从更深的历史动机讲起。假设我们想用能量模型(energy-based model)建密度：$p_\theta(x)=\frac{1}{Z_\theta}e^{-E_\theta(x)}$，其中 $Z_\theta=\int e^{-E_\theta(x)}\mathrm dx$ 是**配分函数**(partition function)。极大似然要算 $\log p_\theta=-E_\theta-\log Z_\theta$，而 $\log Z_\theta$ 是高维积分，**算不出来也没法求导**——这是能量模型几十年的死穴(要用 MCMC 近似，极慢)。

Hyvärinen(2005)的天才一招：**看 score 而非密度。** 因为 score 是对 $x$ 求梯度：

$$
\nabla_x\log p_\theta(x)=\nabla_x\big(-E_\theta(x)-\log Z_\theta\big)=-\nabla_x E_\theta(x)
$$

$\log Z_\theta$ 不依赖 $x$，求梯度直接**消失**!**配分函数的诅咒被 score 一笔勾销。** 这是 score 之所以是扩散模型核心对象的更深层理由：它天然免疫归一化常数。

> **本质洞察**:score 的全部魔力源于一个简单事实——**对 $x$ 求梯度会杀死任何不依赖 $x$ 的归一化常数。** 密度 $p(x)$ 必须归一化(积分为 1)，这个全局约束是万恶之源；而 score $\nabla\log p$ 是"局部斜率"，不受全局归一化牵连。这把"建模归一化密度"(全局难题)换成了"建模一个无约束向量场"(局部易事)。把它和你已知的对照：这与物理里"力是势能的梯度，而势能的零点(常数)无所谓"是同一回事——我们永远只能测量力(score)，测不到势能的绝对值($\log Z$)。

### 历史

Hyvärinen(JMLR 2005)提出显式 score matching，但它要算 score 的雅可比迹(Hessian trace)，高维不可行。Vincent(Neural Computation 2011)证明了去噪 score matching(DSM)等价于显式 SM，把目标变成"学一个去噪器"——可扩展。Song & Ermon(NeurIPS 2019)的切片 score matching 提供另一条可扩展路。这三者构成"score matching 家族"。Tweedie 公式更古老，源自 Maurice Tweedie(1947),Robbins(1956)用于经验贝叶斯，Efron(2011)发扬，在扩散里被用来统一各种参数化。

### 理论一：显式 Score Matching(Hyvärinen 2005)

我们想最小化与真实 score 的 $L^2$ 距离(**显式 SM 目标**,explicit score matching):

$$
J_\text{ESM}(\theta)=\tfrac12\,\mathbb E_{x\sim p}\big\|s_\theta(x)-\nabla\log p(x)\big\|^2
$$

含未知 $\nabla\log p$。Hyvärinen 的定理：

> **定理(Hyvärinen 2005)**。在 $p$ 光滑且边界衰减($p(x)s_\theta(x)\to0$ 当 $\|x\|\to\infty$)的条件下，
> $$
> J_\text{ESM}(\theta)=\mathbb E_{x\sim p}\Big[\tfrac12\|s_\theta(x)\|^2+\nabla\cdot s_\theta(x)\Big]+\text{const}
> $$
> 其中 $\nabla\cdot s_\theta=\sum_i\partial_{x_i}[s_\theta]_i=\mathrm{tr}(\nabla s_\theta)$ 是散度(雅可比的迹),const 不依赖 $\theta$。

**证明(完整，用分部积分)**：展开平方：

$$
J_\text{ESM}=\mathbb E_p\Big[\tfrac12\|s_\theta\|^2\Big]-\mathbb E_p\big[\langle s_\theta,\nabla\log p\rangle\big]+\underbrace{\mathbb E_p\big[\tfrac12\|\nabla\log p\|^2\big]}_{\text{const，不含}\theta}
$$

只需处理中间的交叉项。把期望写成积分，用 $\nabla\log p=\frac{\nabla p}{p}$:

$$
\mathbb E_p\langle s_\theta,\nabla\log p\rangle=\int p(x)\Big\langle s_\theta(x),\frac{\nabla p(x)}{p(x)}\Big\rangle\mathrm dx=\int\langle s_\theta(x),\nabla p(x)\rangle\mathrm dx
$$

对每个分量 $i$ 做分部积分(integration by parts):$\int [s_\theta]_i\,\partial_{x_i}p\,\mathrm dx=\big[[s_\theta]_i\,p\big]_{-\infty}^{\infty}-\int p\,\partial_{x_i}[s_\theta]_i\,\mathrm dx$。边界项由衰减假设为零，故：

$$
\int\langle s_\theta,\nabla p\rangle\mathrm dx=-\int p\,(\nabla\cdot s_\theta)\,\mathrm dx=-\mathbb E_p[\nabla\cdot s_\theta]
$$

代回：$J_\text{ESM}=\mathbb E_p[\tfrac12\|s_\theta\|^2+\nabla\cdot s_\theta]+\text{const}$。$\blacksquare$

> **本质洞察**：这个证明就是前面说的"魔术"的兑现——**分部积分把"含未知 $\nabla\log p$ 的内积项"变成了"只含 $s_\theta$ 散度的项"，未知量被边界项(为零)吞掉了。** 代价是出现了散度 $\nabla\cdot s_\theta=\mathrm{tr}(\nabla s_\theta)$，需要算 score 网络的雅可比迹。在 $d$ 维下，精确算这个迹要 $d$ 次反向传播，$d=10^5$(图像)时完全不可行。**这就是显式 SM 的阿喀琉斯之踵，催生了下面两个可扩展替代。**

### 理论二：去噪 Score Matching(Vincent 2011)—— DDPM 的数学心脏

Vincent 的破局思路：**不直接对干净数据 $p$ 做 score matching，而是对加了高斯噪声的 $p_\sigma$ 做。** 加噪后，score 有了闭式标签！

设加噪 $\tilde x=x+\sigma\varepsilon$($\varepsilon\sim\mathcal N(0,I)$)，即 $q_\sigma(\tilde x\mid x)=\mathcal N(\tilde x;x,\sigma^2 I)$，加噪边际 $p_\sigma(\tilde x)=\int q_\sigma(\tilde x\mid x)p(x)\mathrm dx$。关键：**条件 score 是显式的**:

$$
\nabla_{\tilde x}\log q_\sigma(\tilde x\mid x)=\nabla_{\tilde x}\Big(-\frac{\|\tilde x-x\|^2}{2\sigma^2}\Big)=-\frac{\tilde x-x}{\sigma^2}=-\frac{\varepsilon}{\sigma}
$$

> **定理(Vincent 2011，去噪 SM = 显式 SM)**。
> $$
> \underbrace{\mathbb E_{p_\sigma}\big\|s_\theta(\tilde x)-\nabla\log p_\sigma(\tilde x)\big\|^2}_{J_\text{ESM on }p_\sigma}=\underbrace{\mathbb E_{x\sim p,\,\tilde x\sim q_\sigma(\cdot\mid x)}\big\|s_\theta(\tilde x)-\nabla_{\tilde x}\log q_\sigma(\tilde x\mid x)\big\|^2}_{J_\text{DSM}}+\text{const}
> $$
> 即：用**可计算的条件 score** $\nabla\log q_\sigma(\tilde x\mid x)=-\varepsilon/\sigma$ 作标签做回归，与对未知边际 score $\nabla\log p_\sigma$ 做回归，有相同的最优解。

**证明(完整，独立用分部积分)**：只需证两个目标的交叉项相等(平方项中 $\|s_\theta\|^2$ 项两边相同，$\|\nabla\log\cdot\|^2$ 项都是不含 $\theta$ 的常数)。左边交叉项：

$$
\mathbb E_{p_\sigma}\langle s_\theta(\tilde x),\nabla\log p_\sigma(\tilde x)\rangle=\int p_\sigma(\tilde x)\Big\langle s_\theta(\tilde x),\frac{\nabla p_\sigma(\tilde x)}{p_\sigma(\tilde x)}\Big\rangle\mathrm d\tilde x=\int\langle s_\theta(\tilde x),\nabla p_\sigma(\tilde x)\rangle\mathrm d\tilde x
$$

代入 $p_\sigma(\tilde x)=\int q_\sigma(\tilde x\mid x)p(x)\mathrm dx$，梯度可进积分：$\nabla p_\sigma(\tilde x)=\int p(x)\nabla_{\tilde x}q_\sigma(\tilde x\mid x)\mathrm dx$。再用 $\nabla q_\sigma=q_\sigma\nabla\log q_\sigma$:

$$
\text{左交叉项}=\iint p(x)q_\sigma(\tilde x\mid x)\big\langle s_\theta(\tilde x),\nabla\log q_\sigma(\tilde x\mid x)\big\rangle\mathrm dx\,\mathrm d\tilde x=\mathbb E_{x,\tilde x}\langle s_\theta(\tilde x),\nabla\log q_\sigma(\tilde x\mid x)\rangle
$$

这正是右边目标的交叉项。两交叉项相等，故两目标差一个常数，同最优解。$\blacksquare$

代入 $\nabla\log q_\sigma(\tilde x\mid x)=-\varepsilon/\sigma$,DSM 目标变成：

$$
\boxed{\ J_\text{DSM}=\mathbb E_{x,\varepsilon}\Big\|s_\theta(x+\sigma\varepsilon)+\frac{\varepsilon}{\sigma}\Big\|^2\ }
$$

**这就是 DDPM 损失的本质！** 令 $s_\theta=-\varepsilon_\theta/\sigma$(即用网络预测噪声 $\varepsilon_\theta\approx\varepsilon$)，目标变成 $\mathbb E\|\varepsilon_\theta-\varepsilon\|^2/\sigma^2$——正是 §8.4.3 的 $L_\text{simple}$(差一个 $1/\sigma^2$ 权重)。**§8.4.3 从变分推断、本节从 score matching，两条完全独立的路殊途同归到同一个损失。** 章首承诺的"两条登山道汇合"在此精确兑现。

> **本质洞察(去噪即 score 估计)**:DSM 揭示了一个惊人的等价——**学会"从带噪样本去噪"(预测加进去的 $\varepsilon$)就等于学会了"加噪分布的 score"。** 去噪和 score 估计是同一件事的两个名字。直觉：最优去噪器告诉你"往哪个方向移动 $\tilde x$ 能最像干净数据"，而这个方向正是密度上升最快的方向(score)。这个等价是 DDPM 能用一个朴素的"去噪 MSE"训练却隐式学到完整数据分布的根本原因。它也连接了一个老领域：图像去噪(经典信号处理)和现代生成建模，原来是同一枚硬币的两面。

类比边界：把 DSM 类比成"教学生通过'改错'学知识"——**像**在都是从"被破坏的版本"恢复"正确版本";**不像**在改错通常有唯一正确答案，而去噪的"正确答案"是一个分布(给定噪点，干净数据有多种可能)，网络学的是这个后验的均值(下面 Tweedie 会精确化)。

### 理论三：切片 Score Matching(Song et al. 2019，简述)

显式 SM 的散度 $\nabla\cdot s_\theta$ 难算。Sliced SM 用随机投影估计它：$\nabla\cdot s_\theta=\mathbb E_{v}[v^\top(\nabla s_\theta)v]$($v$ 随机单位向量)，而 $v^\top(\nabla s_\theta)v$ 可由一次"前向模式"自动微分(雅可比-向量积)算出，$O(1)$ 次而非 $O(d)$ 次。它不需要加噪(可估计干净数据的 score)，但方差比 DSM 大。三者关系：

| 方法 | 标签来源 | 可扩展性 | 是否需加噪 | 主要用途 |
|------|---------|---------|-----------|---------|
| 显式 SM(Hyvärinen) | 分部积分消去 $\nabla\log p$，留散度 | 差($O(d)$ 算 Hessian 迹) | 否 | 理论基准 |
| 去噪 SM(Vincent) | 条件 score $-\varepsilon/\sigma$ | 好(纯回归) | 是 | **扩散模型训练** |
| 切片 SM(Song) | 随机投影估散度 | 中(投影方差) | 否 | 估计干净数据 score |

### 理论四：Tweedie 公式 —— 三种参数化的统一(完整证明)

我们已多次提到 $\varepsilon$、score、$x_0$ 三种预测目标等价。Tweedie 公式给出精确的换算，是本专题"四重视角统一"的数学基石。

> **定理(Tweedie 公式)**。设 $\tilde x=x+\sigma\varepsilon$,$\varepsilon\sim\mathcal N(0,I)$,$x\sim p$，加噪边际 $p_\sigma$。则后验均值(最优去噪)由边际 score 给出：
> $$
> \boxed{\ \mathbb E[x\mid\tilde x]=\tilde x+\sigma^2\,\nabla_{\tilde x}\log p_\sigma(\tilde x)\ }
> $$

**证明(完整)**：加噪边际 $p_\sigma(\tilde x)=\int p(x)\,\frac{1}{(2\pi\sigma^2)^{d/2}}e^{-\|\tilde x-x\|^2/(2\sigma^2)}\mathrm dx$。对 $\tilde x$ 求梯度，梯度作用在高斯核上($\nabla_{\tilde x}e^{-\|\tilde x-x\|^2/2\sigma^2}=-\frac{\tilde x-x}{\sigma^2}e^{\cdots}$):

$$
\nabla p_\sigma(\tilde x)=\int p(x)\Big(-\frac{\tilde x-x}{\sigma^2}\Big)\frac{e^{-\|\tilde x-x\|^2/2\sigma^2}}{(2\pi\sigma^2)^{d/2}}\mathrm dx=-\frac{1}{\sigma^2}\Big(\tilde x\,p_\sigma(\tilde x)-\int x\,p(x)q_\sigma(\tilde x\mid x)\mathrm dx\Big)
$$

两边除以 $p_\sigma(\tilde x)$，并注意后验 $p(x\mid\tilde x)=\frac{p(x)q_\sigma(\tilde x\mid x)}{p_\sigma(\tilde x)}$:

$$
\nabla\log p_\sigma(\tilde x)=\frac{\nabla p_\sigma}{p_\sigma}=-\frac{1}{\sigma^2}\Big(\tilde x-\int x\,p(x\mid\tilde x)\mathrm dx\Big)=-\frac{1}{\sigma^2}\big(\tilde x-\mathbb E[x\mid\tilde x]\big)
$$

移项即得 $\mathbb E[x\mid\tilde x]=\tilde x+\sigma^2\nabla\log p_\sigma(\tilde x)$。$\blacksquare$

**统一三种坐标**(在 VP 记号下，$\tilde x=x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon$，等价方差 $\sigma_t^2=1-\bar\alpha_t$，信号缩放 $\sqrt{\bar\alpha_t}$)。三个量通过下列恒等式互相确定：

$$
\boxed{\ \underbrace{s_\theta(x_t,t)}_{\text{score}}=-\frac{\varepsilon_\theta(x_t,t)}{\sqrt{1-\bar\alpha_t}}=\frac{\sqrt{\bar\alpha_t}\,\hat x_0(x_t,t)-x_t}{1-\bar\alpha_t}\ }
$$

| 预测目标 | 网络输出 | 由 score 换算 | 工程特点 |
|---------|---------|--------------|---------|
| **噪声 $\varepsilon$**(DDPM) | $\varepsilon_\theta$ | $\varepsilon_\theta=-\sqrt{1-\bar\alpha_t}\,s_\theta$ | 尺度恒定，最稳，主流 |
| **score**(NCSN/SDE) | $s_\theta$ | —— | 直接进采样器 |
| **数据 $x_0$**(部分实现) | $\hat x_0$ | $\hat x_0=\frac{1}{\sqrt{\bar\alpha_t}}(x_t+(1-\bar\alpha_t)s_\theta)$ | 高噪声退化，但低噪声直观 |
| **velocity $v$**(Flow/v-pred) | $v_\theta$ | $v=\sqrt{\bar\alpha_t}\varepsilon-\sqrt{1-\bar\alpha_t}x_0$ | §8.4.6，数值最稳之一 |

> **本质洞察(同一个量的不同坐标)**:Tweedie 公式证明了 $\varepsilon$、score、$x_0$ 不是三个不同的学习目标，而是**同一个后验信息在三组坐标下的表示**，彼此差一个**确定的、已知的线性变换**(只依赖 $\bar\alpha_t$，不依赖数据)。这意味着：训练时选哪个目标，在"能学到什么"层面完全等价，差异**纯粹在数值条件**(梯度尺度、高低噪声区的退化)。这是本专题最重要的统一性结论——它让你在读任何扩散论文时，看到 $\varepsilon$-pred / x0-pred / v-pred / score 都能瞬间翻译成同一个对象。把它和你已知的对照：这与线性代数里"同一个向量在不同基下有不同坐标，但向量本身不变"是同一种思想——选基(参数化)是为了数值方便，不改变被表示的对象。

### ⚠️ 常见陷阱

💡 **概念误区：以为 score matching 需要知道真实 score 做标签**
- 新手想法："回归 score，总得有 score 的真值标签吧？"
- 实际上：显式 SM 通过分部积分消去了真值 score(代价是散度项);DSM 用**条件** score $-\varepsilon/\sigma$ 做标签——这个是已知的(就是你加进去的噪声)！整个 score matching 的精髓就是"绕开不可知的边际 score"。
- 为什么重要：理解了这点，你才明白 DDPM 训练里"标签就是采样的噪声 $\varepsilon$"为什么合理。

💡 **概念误区：把 Tweedie 的 $\mathbb E[x\mid\tilde x]$ 当成"唯一的干净数据"**
- 新手想法："去噪器输出 $\mathbb E[x\mid\tilde x]$，那不就是恢复了原始数据吗？"
- 实际上：它是后验**均值**，不是某个具体样本。给定一个噪点，可能对应多个干净数据(多峰后验),Tweedie 给的是它们的加权平均。在高噪声区，这个均值可能是个"模糊的平均脸"，不对应任何真实样本。
- 根本原因：去噪是一个一对多的逆问题，均值未必在数据流形上。
- 正确做法：单步 Tweedie 去噪只在低噪声区接近真实样本；高噪声区要靠多步迭代(逐步缩小后验的不确定性)才能采到清晰样本。

💡 **概念误区：显式 SM 和 DSM "学到的是不同的 score"**
- 新手想法："显式 SM 学干净数据 score,DSM 学加噪 score，它们不一样。"
- 实际上：在同一个 $\sigma$ 下，Vincent 定理证明二者学到**相同**的 $\nabla\log p_\sigma$(加噪边际 score)。DSM 不是在学干净数据 score——它学的就是加噪边际的 score，与显式 SM on $p_\sigma$ 一致。
- 正确做法：扩散模型要的本来就是各噪声水平 $p_t$ 的 score(反向 SDE 每个 $t$ 都要),DSM 正好提供，无需干净数据 score。

🧠 **思维陷阱：把"配分函数被消去"误推为"扩散模型能算似然"**
- 新手想法："score 避开了 $Z$，所以扩散模型能直接算 $p(x)$?"
- 实际上：score 避开 $Z$ 让**训练**可行，但单凭 score 不能直接给出归一化密度值。算似然要用概率流 ODE 的瞬时变量替换公式(§8.4.2 表格)，沿轨迹积分散度，是另一套机制。
- 为什么重要：不要混淆"训练不需要 $Z$"和"推理能算密度"——前者是 score matching 的功劳，后者是 ODE 视角的额外馈赠。

### 练习

1.(证明题，草稿纸上完成)独立重证 Vincent 定理：不看正文，从 $J_\text{ESM on }p_\sigma$ 出发，展开平方，用 $p_\sigma=\int q_\sigma p$ 和 $\nabla q_\sigma=q_\sigma\nabla\log q_\sigma$ 把交叉项变成 $J_\text{DSM}$ 的交叉项。这是博士生必须能独立复述的核心证明(本专题"必完证"之一)。

2.(推导题)用 Tweedie 公式独立推导 score 与 $\varepsilon$ 的关系。提示：在 VP 设定 $x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon$ 下，先把 Tweedie 写成 $\mathbb E[x_0\mid x_t]$ 形式(注意信号缩放 $\sqrt{\bar\alpha_t}$ 要折算)，再用 $x_0=\frac{1}{\sqrt{\bar\alpha_t}}(x_t-\sqrt{1-\bar\alpha_t}\varepsilon)$ 对照，得到 $s_\theta=-\varepsilon_\theta/\sqrt{1-\bar\alpha_t}$。

3.(开放思考题)显式 SM 的散度项 $\nabla\cdot s_\theta$ 在 $d=10^5$ 时为何不可行？切片 SM 用随机投影把它变成 $O(1)$ 次 JVP，代价是引入估计方差。请分析：当数据维度从 $d=7$(单步机器人动作)增长到 $d=10^5$(高分辨率图像)时，显式 SM、DSM、切片 SM 三者的适用性如何变化？为什么机器人低维动作场景下三者差异不如图像场景明显？(此题训练你在"维度"这个维度上做方法选型。)

---

## §8.4.5 收敛理论：训练好 score 后，采样能多准？ ⭐⭐⭐⭐

### 动机：一个让人不安的循环论证

到目前为止，我们的逻辑是：Anderson 说"知道 score 就能完美生成";score matching 说"我们能学到近似 score $s_\theta\approx\nabla\log p_t$"。但这里有个裂缝：**我们只能学到近似 score(有误差)，还得用有限步离散采样(又有误差)，从一个近似的初值 $\mathcal N(0,I)$ 起步(再有误差)。这三重误差叠加后，采出的样本分布 $\hat p$ 离真实 $p_\text{data}$ 到底有多远？会不会误差雪崩，前面所有理论都白搭？**

这正是收敛理论(convergence theory)要回答的。它给出形如"$\text{dist}(\hat p,p_\text{data})\le$ 某个随维度 $d$、步数 $N$、score 误差 $\varepsilon_\text{score}$ 增长的界"的定量保证。对机器人尤其重要：如果你要把 Diffusion Policy 用在安全攸关的场合，你需要知道"采样步数减半，分布偏差会增大多少"，而不是凭感觉调参。

为什么是研究级(⭐⭐⭐⭐)？因为它动用随机分析最重的武器——Girsanov 定理(测度变换)。这一节只给证明骨架，完整证明在专业论文里要十几页。但理解骨架足以让你读懂收敛论文、知道各个界的强弱。

### 如果没有收敛保证会怎样

反事实：假如扩散模型只有"经验上效果好"而无收敛理论，会发生两件坏事。其一，无法**比较**方法优劣——你说 DDIM 50 步够好，凭什么？多大维度下够？没有界就只能 case-by-case 实验。其二，无法**指导设计**——score 网络要训到多准才"够用"？步数和精度怎么权衡？这些都需要把误差和最终偏差的关系写成不等式。收敛理论把扩散从"炼丹"推向"工程科学"。

> **本质洞察**：收敛理论的价值不在那个具体的界(常数往往很松)，而在它告诉你**误差的来源结构和缩放规律**。知道"误差随维度 $d$ 线性增长还是平方增长"比知道"误差 $\le 0.037$"重要得多——前者指导你"维度翻倍要把 score 训多准、步数加多少"，后者只是一个用不上的数字。这与机器人里"我们关心控制误差随质量误差是线性还是二次放大，而非某个具体仿真数值"是同一种思维。

### 历史：从指数界到近线性界

早期收敛分析(2022 前)需要强假设(如对数 Sobolev 不等式 LSI、数据分布的强凸性)，界随维度指数爆炸，实用性差。突破来自：**Chen, Chewi, Li, Li, Salim(ICLR 2023)**在最小假设下(只要 score 的 $L^2$ 误差有界 + 数据有有限二阶矩，**不需要** LSI、不需要光滑)证明了多项式收敛。**Benton et al.(ICLR 2024)**用随机局部化(stochastic localization)进一步把维度依赖改进到近线性 $\tilde O(d)$。这是 2023–2024 年扩散理论的核心进展。

### 理论：三源误差分解

把采样误差分解成三个可独立分析的来源。设理想反向 SDE(用真 score、连续、从 $p_T$ 起步)产生 $p_\text{data}$；实际算法产生 $\hat p$。误差三来源：

| 误差源 | 来自哪里 | 怎么控制 | 对应前文 |
|--------|---------|---------|---------|
| **(E1) 初始化失配** | 从 $\mathcal N(0,I)$ 而非真实 $p_T$ 起步 | 增大 $T$(让前向更充分混合)，误差 $\sim e^{-T}$ 指数小 | §8.4.1 末的 $p_T$ 残差、§8.4.3 的 $L_N$ 项 |
| **(E2) score 估计误差** | $s_\theta\ne\nabla\log p_t$，有 $L^2$ 误差 $\varepsilon_\text{score}^2=\mathbb E\|s_\theta-\nabla\log p_t\|^2$ | 训更久/更大网络/更多数据 | §8.4.4 的训练质量 |
| **(E3) 离散化误差** | 有限 $N$ 步，每步用 Euler-Maruyama 近似 | 增加步数 $N$，误差 $\sim 1/N$ 或更好 | DDIM 减步的代价 |

> **本质洞察(三源各管一段)**：这个分解的威力在于**正交化**——三个误差源对应三个独立的"旋钮"，互不纠缠。E1 调 $T$,E2 调训练，E3 调步数。调参时不该乱试，而该问"我的瓶颈是哪一源"：样本整体偏离数据流形 → 多半 E2(score 不准)；样本有但多样性差/有伪影 → 多半 E3(步数不够)；样本系统性偏向某方向 → 多半 E1(混合不足)。这把"调参玄学"变成"诊断-对症"。机器人故障诊断里"先定位是感知、规划还是控制出问题，再针对性修"是完全相同的方法论。

### 理论：Chen et al. 2023 主定理(陈述 + 骨架)

> **定理(Chen et al. 2023，非正式)**。设数据分布有有限二阶矩，score 估计满足 $L^2$ 误差 $\frac{1}{N}\sum_k\mathbb E\|s_\theta(\cdot,t_k)-\nabla\log p_{t_k}\|^2\le\varepsilon_\text{score}^2$，前向用 VP-SDE。则用 $N$ 步指数积分器采样，输出 $\hat p$ 满足
> $$
> \mathrm{TV}(\hat p,\,p_\text{data})\ \lesssim\ \underbrace{\sqrt{\mathrm{KL}(p_\text{data}\|\gamma)}\,e^{-T}}_{\text{E1: 初始化}}+\underbrace{\sqrt{T}\,\varepsilon_\text{score}}_{\text{E2: score}}+\underbrace{\frac{L\sqrt{d}\,T}{\sqrt N}}_{\text{E3: 离散化}}
> $$
> 其中 $\gamma=\mathcal N(0,I)$,$L$ 是 score 的 Lipschitz 常数。综合优化得迭代复杂度 $N=\tilde O(L^2 d/\varepsilon^2)$ 达到 $\mathrm{TV}\le\varepsilon$。

**证明骨架(Girsanov 三步)**:

**Step 1(测度变换，Girsanov 定理)**。理想反向 SDE 和近似反向 SDE 只差漂移项里的 score：真 $\nabla\log p_t$ vs 近似 $s_\theta$。Girsanov 定理(§8.4.7 详述)告诉我们：**两个只差漂移的扩散过程，其路径测度的 KL 散度等于漂移差的平方在路径上的积分**:

$$
\mathrm{KL}\big(\mathbb P_\text{ideal}\,\|\,\mathbb P_\text{approx}\big)=\tfrac12\,\mathbb E\!\int_0^T g(t)^2\big\|s_\theta(x_t,t)-\nabla\log p_t(x_t)\big\|^2\,\mathrm dt
$$

这一步是核心：它把"两个分布有多远"翻译成"score 误差的时间积分"——E2 误差的来源就在这里精确量化。

**Step 2(数据处理不等式)**。路径测度的 KL 控制终点(即生成样本)边际的 KL:$\mathrm{KL}(\hat p\|p_\text{data})\le\mathrm{KL}(\mathbb P_\text{approx}\|\mathbb P_\text{ideal})$(边际是路径的函数，数据处理不等式让 KL 不增)。再用 **Pinsker 不等式** $\mathrm{TV}\le\sqrt{\tfrac12\mathrm{KL}}$ 转成全变差距离(回顾前置桥接：$\mathrm{TV}^2\le\tfrac12\mathrm{KL}$)。

**Step 3(三项累加)**。把 E1(初值用 $\gamma$ 代替 $p_T$，贡献 $e^{-T}$ 项，来自前向的指数混合)、E2(Step 1 的 score 积分)、E3(Euler 离散每步的局部误差累加，贡献 $L\sqrt d\,T/\sqrt N$)三部分用三角不等式合并。$\blacksquare$(骨架)

> **阶段小结**:Girsanov 把"分布距离"换成"score 误差的路径积分"(E2)，数据处理+Pinsker 把路径 KL 降到样本 TV，最后三角不等式拼起三源。整条链的命门是 Step 1——**这是 Girsanov 在扩散收敛证明中不可替代的角色：只有它能把"两条随机轨迹的分布差"精确写成"漂移差的二次型积分"。**

### Chen 的 $d^2$ vs Benton 的 $d$：差在哪

学习目标 6 要求知道两个界的差异。Chen et al. 2023 原始界在某些设定下维度依赖是 $\tilde O(d^2)$(或 $d$ 的较高次，取决于度量和假设);Benton et al. 2024 用**随机局部化**(stochastic localization，把采样看成一个信息逐步揭示的鞅过程)证明了**近线性** $\tilde O(d/\varepsilon^2)$ 的 KL 界。

> **本质洞察(为什么维度依赖从 $d^2$ 降到 $d$ 是大事)**：差一个 $d$ 因子，在 $d=10^4$ 时就是一万倍的步数差异——这决定了理论是"实用"还是"空谈"。Benton 的改进核心是更精细地处理了离散化误差：不再对每步用最坏情况的 Lipschitz 界(那会乘出额外的 $d$)，而是用鞅结构让误差在维度上"平均掉"。**这印证了一个理论研究的通则：维度依赖的阶数，几乎总取决于你用多粗的不等式处理误差累积——粗放的逐步三角不等式给高次，精细的鞅/集中不等式给低次。** 这对机器人高维动作空间(长 horizon $\times$ 多自由度)的扩散规划有直接意义：它说明扩散在高维下原则上不会"维度诅咒"式爆炸。

### 把三源误差当"预算"用：一个工程化的思考框架

收敛界除了理论意义，还能当成**误差预算分配器**来用——把抽象的不等式直接翻译成调参的行动指南。假设你要把生成分布的总误差控制在 $\mathrm{TV}\le\varepsilon_\text{total}$，主定理给出三项之和：

$$
\varepsilon_\text{total}\approx\underbrace{c_1 e^{-T}}_{\text{E1}}+\underbrace{c_2\sqrt{T}\,\varepsilon_\text{score}}_{\text{E2}}+\underbrace{c_3\frac{L\sqrt d\,T}{\sqrt N}}_{\text{E3}}
$$

**配平原则**：让三项大致相等(任一项远大于其他都是浪费——比如把 score 训得超准但步数太少，E2 极小而 E3 主导，白训了)。由此反推三个旋钮：

1. **定 $T$**：由 E1 $=e^{-T}\approx\tfrac13\varepsilon_\text{total}$ 得 $T\approx\log(3c_1/\varepsilon_\text{total})$——只需对数级别，$T$ 取 $5\sim10$ 量级通常够(这解释了为何前向"混合"不必太久)。
2. **定 score 精度**：由 E2 $=c_2\sqrt T\,\varepsilon_\text{score}\approx\tfrac13\varepsilon_\text{total}$ 得 $\varepsilon_\text{score}\approx\varepsilon_\text{total}/(3c_2\sqrt T)$——这是"score 要训到多准才不浪费"的定量答案。
3. **定步数**：由 E3 $\approx\tfrac13\varepsilon_\text{total}$ 得 $N\approx(3c_3 L\sqrt d\,T/\varepsilon_\text{total})^2$——步数随维度 $\sqrt d$、随精度 $1/\varepsilon$ 平方增长。

> **本质洞察(配平比单项优化更重要)**：新手常犯的错是"死磕一个旋钮"——把网络训到极致(E2→0)却用 5 步采样(E3 巨大)，或反之。**收敛界告诉你：总误差由三项之和决定，木桶效应，最大的那项卡住一切。** 正确做法是先诊断哪项主导(§8.4.5 的诊断表)，再针对性投入。这与机器人系统集成"先找瓶颈子系统再优化，而非平均用力"是完全相同的工程方法论。把抽象的收敛界翻译成"三个旋钮的配平"，它就从论文里的不等式变成了你调参时的行动指南。

### 一个反直觉推论：为什么不需要 LSI

Chen et al. 最震撼之处是**不需要对数 Sobolev 不等式(LSI)**。LSI 大致要求分布"没有被高能垒分隔的远距离峰"。早期分析都依赖它，因而排除了多峰分布。Chen 等人证明扩散收敛**不需要** LSI——这正是扩散能处理高度多峰分布(如机器人多模态动作)的理论根据。

> **本质洞察**：多峰分布对很多采样方法(如 Langevin MCMC)是噩梦——粒子会困在一个峰里出不来(混合时间随峰间能垒指数爆炸，这正是 LSI 失效的场景)。扩散为什么免疫？因为它**不是在固定分布上跑 MCMC，而是沿着一个从单峰高斯逐步"分裂"出多峰的路径走**——在高噪声端分布是单峰高斯(易采)，随着去噪逐步分裂出各个峰，粒子在峰还"连通"时就被引导到正确的吸引域。**这是扩散相对传统 MCMC 采样的根本优势，也是 §8.4.8 里"多级退火 MPPI 突破单级高斯天花板"的理论根源。** 类比边界：这像"退火"(annealing)——高温时势垒被抹平易探索，降温时锁定到正确极小值；但不像物理退火的是，扩散的"温度路径"是被前向 SDE 精确设计的，且有 score 主动引导而非被动随机游走。

### ⚠️ 常见陷阱

💡 **概念误区：把收敛界的常数当真**
- 新手想法："定理说 $N=\tilde O(d/\varepsilon^2)$，我代入数字算出需要 $10^9$ 步，扩散没法用啊？"
- 实际上：这些界的常数极松(为覆盖最坏情况)，实际所需步数远小于界的预测(DDIM 几十步就好)。界的价值在**缩放规律**($N$ 怎么随 $d,\varepsilon$ 变)，不在绝对数值。
- 正确做法：用界做相对比较和趋势预测，绝不用它定具体超参。

💡 **概念误区：以为 score 误差 $\varepsilon_\text{score}$ 可以无限减小**
- 新手想法："E2 项 $\sqrt T\varepsilon_\text{score}$，只要训得好 $\varepsilon_\text{score}\to0$ 就行。"
- 实际上：$\varepsilon_\text{score}$ 有下界——有限数据下，score 在低密度区(数据稀疏处)本质上估不准；且高噪声小 $t$ 区 score 量级大、难拟合。存在不可约的估计误差。
- 为什么重要：这解释了为什么实践中盲目增大网络/训练时间收益递减，以及为什么"在哪些 $t$ 区间加权训练"(回顾 §8.4.3 $L_\text{simple}$ 重加权)很关键。

🧠 **思维陷阱：认为 ODE 采样(确定)误差一定小于 SDE(随机)**
- 新手想法："ODE 没有随机性，误差应该更可控。"
- 实际上：收敛分析显示，score 不准时，**SDE 的随机性提供纠错**(把偏离流形的轨迹拉回，见 §8.4.2)，反而可能比 ODE 更鲁棒；ODE 误差沿确定轨迹无纠正地累积。但 ODE 在 score 很准时可用高阶求解器，离散误差更低。
- 正确做法：score 估计质量是分水岭——粗 score 用 SDE(纠错)，精 score 用 ODE(高阶少步)。

### 练习

1.(推导题，草稿纸上完成)从 Girsanov 的 KL 公式 $\mathrm{KL}=\tfrac12\mathbb E\int_0^T g^2\|s_\theta-\nabla\log p_t\|^2\mathrm dt$ 出发，假设 score 误差在所有 $t$ 均匀有界 $\|s_\theta-\nabla\log p_t\|^2\le\delta^2$，且 VP-SDE 的 $g^2=\beta(t)$，推出 KL $\le\tfrac12\delta^2\int_0^T\beta\,\mathrm dt=\tfrac12\delta^2\bar\beta(T)$，再用 Pinsker 给出 TV 界。这让你亲手走通"score 误差 → 分布距离"的核心一步。

2.(开放思考题)三源误差中，增大 $T$ 减小 E1 但增大 E2(更长的时间积分累积更多 score 误差)和 E3(更长 horizon 更多离散步)。请论证存在一个"最优 $T$"，并定性描述它如何依赖 score 质量 $\varepsilon_\text{score}$ 和步数预算 $N$。(这是真实系统调参的核心权衡。)

3.(跨章综合题，综合 §8.4.2 + §8.4.5 + 04 采样式 MPC)在采样式 MPC(MPPI)中，"温度"参数控制探索-利用权衡，多级退火 = 逐步降温。把它类比扩散的噪声水平 $t$：论证"单级 MPPI 困在单峰"对应扩散里"只在低噪声端采样会困在初值附近的峰"，而"多级退火"对应"从高噪声端走完整反向过程"。用本节的"多峰不需 LSI"洞察解释为什么多级退火能突破单级的高斯天花板。

---

## §8.4.6 Flow Matching：把反向过程拉成直线 ⭐⭐⭐

### 动机：ODE 视角下，能不能训得更简单、采得更快

§8.4.2 给出了概率流 ODE $\dot x=v(x,t)$，采样即沿速度场积分。但我们训练时学的是 score(经由 $\varepsilon$-预测)，再换算成速度场。Flow Matching(Lipman et al. 2023)问了一个更直接的问题：**能不能跳过 score，直接学那个速度场 $v(x,t)$？而且，能不能选一条比扩散"弯曲的圆弧路径"更直的路径，让采样只需极少步(甚至单步)?**

这个动机对机器人是命脉级的。$\pi_0$(Black et al. 2024)要在真实机器人上做 50 Hz 控制——每 20 毫秒生成一个动作序列。DDPM 的几十到上千步采样太慢；Flow Matching 的直线路径让采样压到几步，这才使实时 VLA 控制可行。理解 FM 就是理解当前最先进机器人策略($\pi_0$、$\pi_{0.5}$)的生成引擎。

> **本质洞察(连续性方程是统一一切的母方程)**:Flow Matching 和扩散看似两套体系，但 §8.4.2 Step 3 已经埋下伏笔——任何边际密度演化 $p_t$ 都可写成连续性方程 $\partial_t p_t=-\nabla\cdot(v_t\,p_t)$，其中 $v_t$ 是某个速度场。**扩散、score-based、Flow Matching 的全部区别，仅在于选了不同的 $p_t$ 路径(连接 $p_\text{data}$ 与噪声的不同"插值")和对应的 $v_t$。** score 视角和速度场视角是同一个连续性方程的两种参数化。这是本专题"四重视角统一"的几何顶点。把它和你已知的对照：这与流体力学里"用速度场描述流动 vs 用密度梯度描述流动"是同一种对偶。

### 如果坚持用扩散的弯曲路径会怎样

反事实推理。扩散的概率流 ODE 轨迹是弯的(VP 的信号-噪声配比走单位圆弧，回顾 §8.4.1 练习 2)。弯曲轨迹意味着：用大步长数值积分时，直线段会"切过"弯道，误差大——所以扩散 ODE 想少步就得用高阶求解器(DPM-Solver 等)，且仍有精度上限。

如果路径本身就是**直线**呢？直线的速度场恒定(沿轨迹不变)，那么哪怕用最粗的一阶 Euler、哪怕只走一步，也精确！**这就是 Flow Matching 追求"直线路径"的全部动机：把采样的数值难度从源头消除。** Rectified Flow(Liu et al. 2023)更进一步，通过"reflow"迭代把路径反复拉直，逼近单步生成。

类比边界：把"弯路 vs 直路"类比成"开车走盘山公路 vs 走隧道直达"——**像**在都连接起点终点、直路明显更省"步数(油耗)";**不像**在物理隧道是固定的，而 FM 的"直路"是在概率分布空间里的直线(每个样本配一条直线)，且只有在条件路径层面是直的，边际速度场仍可能弯(下面详述这个微妙之处)。

### 历史

Chen et al.(NeurIPS 2018)的 Neural ODE 和 FFJORD 提出连续归一化流(CNF)，但训练要"模拟 ODE + 算雅可比迹"，极慢。Lipman et al.(ICLR 2023)的 Flow Matching 给出**仿真无关(simulation-free)**的训练目标——无需在训练时积分 ODE，这是关键突破。Liu et al.(Rectified Flow)和 Albergo & Vanden-Eijnden(Stochastic Interpolants)同期给出相关框架。Stochastic Interpolants 尤其重要：它证明 DDPM 和 FM 是同一连续族的两端。

### 理论：从条件路径构造边际速度场

**目标**：学一个速度场 $v_\theta(x,t)$，使其诱导的 ODE 把先验 $p_0=\mathcal N(0,I)$(注意 FM 习惯 $t:0\to1$,$t=0$ 是噪声、$t=1$ 是数据，与扩散时间方向相反)输运到 $p_1=p_\text{data}$。

朴素目标(**Flow Matching 损失**，但不可算):

$$
\mathcal L_\text{FM}(\theta)=\mathbb E_{t,\,x\sim p_t}\big\|v_\theta(x,t)-u_t(x)\big\|^2
$$

其中 $u_t$ 是真实边际速度场。问题同 score matching:$u_t$ 是未知的边际量，我们没有它的标签。Lipman 的破解：**用条件路径(conditional path)。**

**构造**：对每个数据点 $x_1\sim p_\text{data}$，设计一条从噪声到该点的**条件概率路径** $p_t(x\mid x_1)$ 和对应的**条件速度场** $u_t(x\mid x_1)$。最自然的选择是**高斯条件路径**:

$$
p_t(x\mid x_1)=\mathcal N\big(x;\,t\,x_1,\,(1-t)^2 I\big)
$$

即均值从 $0$ 线性移到 $x_1$，标准差从 $1$ 线性降到 $0$。其采样 $x=t\,x_1+(1-t)\,x_0$($x_0\sim\mathcal N(0,I)$)——**这是一条从噪声样本 $x_0$ 到数据 $x_1$ 的直线！** 沿这条直线求时间导数，条件速度场是常向量：

$$
u_t(x\mid x_1)=\frac{\mathrm d}{\mathrm dt}\big(t x_1+(1-t)x_0\big)=x_1-x_0
$$

**条件 Flow Matching 损失**(可算！):

$$
\boxed{\ \mathcal L_\text{CFM}(\theta)=\mathbb E_{t\sim\mathcal U[0,1],\,x_1\sim p_\text{data},\,x_0\sim\mathcal N(0,I)}\big\|v_\theta(\underbrace{t x_1+(1-t)x_0}_{x},\,t)-\underbrace{(x_1-x_0)}_{u_t(x\mid x_1)}\big\|^2\ }
$$

这个目标完全可算：采数据 $x_1$、采噪声 $x_0$、采时间 $t$、合成 $x$、回归 $v_\theta\to x_1-x_0$。极其简洁——比 DDPM 还简单(没有 $\bar\alpha_t$ 那些系数)!

### 核心推导：CFM 与 FM 同梯度(Lipman 等价恒等式，完整证明)

但这里有个看似致命的问题：我们训练 $v_\theta$ 去匹配**条件**速度场 $u_t(x\mid x_1)=x_1-x_0$，可采样时需要的是**边际**速度场 $u_t(x)$。给定一个中间点 $x$，它可能落在许多条不同 $(x_0,x_1)$ 直线上，对应不同的 $x_1-x_0$——条件速度场是"多值"的，边际速度场是它们的某种平均。凭什么用条件目标训练能学到边际速度场？

Lipman 的等价恒等式回答了这个，它是本专题"必完证"之一：

> **定理(Lipman et al. 2023,CFM = FM 同梯度)**。边际速度场 $u_t(x)$ 是条件速度场关于后验的期望：
> $$
> u_t(x)=\mathbb E_{x_1\sim p(x_1\mid x)}\big[u_t(x\mid x_1)\big]=\int u_t(x\mid x_1)\,\frac{p_t(x\mid x_1)p_\text{data}(x_1)}{p_t(x)}\,\mathrm dx_1
> $$
> 且 $\mathcal L_\text{FM}$ 与 $\mathcal L_\text{CFM}$ 关于 $\theta$ 的梯度相同(两目标差一个不依赖 $\theta$ 的常数)。因此**最小化可算的 $\mathcal L_\text{CFM}$ 等价于最小化不可算的 $\mathcal L_\text{FM}$**。

**证明(完整)**：先证边际速度场表达式。边际密度 $p_t(x)=\int p_t(x\mid x_1)p_\text{data}(x_1)\mathrm dx_1$ 满足连续性方程。边际速度场 $u_t$ 由连续性方程 $\partial_t p_t=-\nabla\cdot(u_t p_t)$ 定义。我们验证上式的 $u_t$ 满足它。对边际密度求时间导，用每个条件路径满足自己的连续性方程 $\partial_t p_t(x\mid x_1)=-\nabla\cdot(u_t(\cdot\mid x_1)p_t(\cdot\mid x_1))$:

$$
\partial_t p_t(x)=\int\partial_t p_t(x\mid x_1)p_\text{data}(x_1)\mathrm dx_1=-\int\nabla\cdot\big(u_t(x\mid x_1)p_t(x\mid x_1)\big)p_\text{data}(x_1)\mathrm dx_1
$$

把散度提到积分外(线性):$=-\nabla\cdot\int u_t(x\mid x_1)p_t(x\mid x_1)p_\text{data}(x_1)\mathrm dx_1$。要让它等于 $-\nabla\cdot(u_t(x)p_t(x))$，只需

$$
u_t(x)p_t(x)=\int u_t(x\mid x_1)p_t(x\mid x_1)p_\text{data}(x_1)\mathrm dx_1
\ \Longrightarrow\
u_t(x)=\int u_t(x\mid x_1)\frac{p_t(x\mid x_1)p_\text{data}(x_1)}{p_t(x)}\mathrm dx_1
$$

正是后验期望(注意 $\frac{p_t(x\mid x_1)p_\text{data}(x_1)}{p_t(x)}=p(x_1\mid x)$)。**边际速度场表达式得证。**

再证两损失同梯度。展开两个平方目标，$\|v_\theta\|^2$ 项：在 $\mathcal L_\text{FM}$ 里是 $\mathbb E_{p_t(x)}\|v_\theta\|^2$，在 $\mathcal L_\text{CFM}$ 里是 $\mathbb E_{x_1,x}\|v_\theta\|^2=\mathbb E_{p_t(x)}\|v_\theta\|^2$(因对 $x_1$ 边缘化后 $x$ 的分布就是 $p_t$),**两者相同**。交叉项：

$$
\mathcal L_\text{FM}\text{ 交叉}: -2\mathbb E_{p_t(x)}\langle v_\theta(x,t),u_t(x)\rangle=-2\int p_t(x)\langle v_\theta,u_t(x)\rangle\mathrm dx
$$

代入刚证的 $u_t(x)p_t(x)=\int u_t(x\mid x_1)p_t(x\mid x_1)p_\text{data}(x_1)\mathrm dx_1$:

$$
=-2\int\langle v_\theta(x,t),\int u_t(x\mid x_1)p_t(x\mid x_1)p_\text{data}(x_1)\mathrm dx_1\rangle\mathrm dx=-2\,\mathbb E_{x_1,x}\langle v_\theta,u_t(x\mid x_1)\rangle
$$

这正是 $\mathcal L_\text{CFM}$ 的交叉项！平方项与交叉项都相同，故 $\mathcal L_\text{FM}=\mathcal L_\text{CFM}+\text{const}$，同梯度。$\blacksquare$

> **本质洞察(条件化的普适威力)**:Lipman 等价和 Vincent 去噪 SM(§8.4.4)是**同一个数学策略的两次应用**——都是"边际量不可算，但条件量可算，且边际量是条件量关于后验的期望，故用条件量做回归目标能学到边际量"。这个"条件化技巧"是现代生成建模的核心引擎：回归一个随机的、多值的条件目标($x_1-x_0$ 或 $-\varepsilon/\sigma$)，由于平方损失的最优解是条件期望，网络自动学到了那个不可算的边际量。**一旦你看穿这一点，DDPM、Score Matching、Flow Matching 在你眼里就是同一个把戏的三次变奏。** 这是整个专题最高层的统一洞察(本专题要求的"本质洞察 > 引用块"之一)。

### 为什么"直线"省步：把数值积分误差算出来

前面反复说"直线路径省采样步数"，这里把它量化，让直觉变成不等式——直接把理论结论连到工程上的采样步数。采样是解 ODE $\dot x=v(x,t)$。一阶 Euler 法每步的**局部截断误差**(local truncation error)正比于轨迹的二阶导(曲率):

$$
\text{每步误差}\approx\tfrac12\|\ddot x\|\,(\Delta t)^2,\qquad \ddot x=\frac{\mathrm d}{\mathrm dt}v(x_t,t)
$$

- **若轨迹是直线**:$\ddot x=0$，局部误差为零——**哪怕 $\Delta t$ 取满(一步走完),Euler 也精确！** 这就是 Rectified Flow 追求单步生成的数学根据。
- **若轨迹弯曲**(如扩散的圆弧):$\ddot x\ne0$，误差随 $(\Delta t)^2$ 增长，要小误差就得小 $\Delta t$、多步数。全局误差 $\sim N\cdot(\Delta t)^2=N\cdot(T/N)^2=T^2/N$，即步数 $N$ 翻倍误差减半。

> **本质洞察(曲率即步数成本)**：采样步数预算和路径曲率是一对此消彼长的量——**轨迹越直(曲率越小)，达到同等精度所需步数越少，极限直线时单步即可。** 这把"为什么 FM 比扩散省步"从一句口号变成精确陈述：不是 FM 算法更聪明，而是它选的概率路径在数据空间里更直，降低了数值积分的曲率成本。这也解释了为什么扩散要少步必须上高阶求解器(DPM-Solver 用二阶/三阶项抵消曲率)，而 FM + Rectified 直接从源头消除曲率。机器人语境：$\pi_0$ 选 FM 的本质，是用"训练时把路径拉直"换取"推理时少步"，而推理步数正是 50 Hz 实时控制的硬约束。

但要注意一个微妙处(承接 §8.4.6 陷阱 2):**单次 CFM 训练得到的边际速度场轨迹并非真直线**(条件路径直，但边际是条件的后验平均，平均后会弯)。所以"单步生成"需要 Rectified Flow 的 reflow 反复拉直，而非 CFM 一训即得。

### 最优传输耦合：为什么"配对方式"也影响直度

CFM 默认独立采 $x_0\sim\mathcal N(0,I)$ 和 $x_1\sim p_\text{data}$(独立耦合)。但 Tong et al.(2023)指出：**改变 $(x_0,x_1)$ 的配对方式(耦合 $\gamma$)，能让边际速度场更直。** 用最优传输(OT)耦合——即按"搬运成本最小"配对噪声和数据，而非随机配对——诱导的边际路径更接近直线、交叉更少。

工程实现：在每个 minibatch 内，用 Sinkhorn 或匈牙利算法求该批 $\{x_0^{(i)}\}$ 与 $\{x_1^{(j)}\}$ 之间的近似 OT 匹配，再按匹配配对做 CFM。这叫 **minibatch-OT CFM**。

> **本质洞察(耦合的自由度)**:Lipman 等价恒等式(§8.4.6)对**任意**满足边际约束的耦合都成立——这意味着耦合 $\gamma$ 是一个"免费的设计自由度"。独立耦合最简单但路径交叉多(边际弯);OT 耦合让"不相撞"(直线不交叉)，边际更直、采样更省步。**这揭示了 Flow Matching 框架的一个深层灵活性：同一对边际 $(p_0,p_\text{data})$，不同耦合给出不同直度的路径，而 OT 耦合是其中最直的。** 这把"最优传输"从抽象数学直接变成"少几步采样"的工程收益，也是 §8.4.7 $W_2$ 几何(OT 是 $W_2$ 测地线)在 FM 里的落地。

### 与扩散的精确关系：Stochastic Interpolants

FM 的高斯条件路径 $x=tx_1+(1-t)x_0$ 是线性插值；扩散的 VP 路径 $x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon$ 是"圆弧插值"(系数平方和为 1)。Albergo & Vanden-Eijnden 的 Stochastic Interpolants 用统一形式 $x_t=\alpha(t)x_1+\sigma(t)x_0$ 涵盖两者：

| 路径 | $\alpha(t)$ | $\sigma(t)$ | 几何 | 采样代表 |
|------|------------|------------|------|---------|
| Flow Matching(OT/线性) | $t$ | $1-t$ | 直线 | 少步 ODE、单步(Rectified) |
| 扩散 VP | $\sqrt{\bar\alpha_t}$ | $\sqrt{1-\bar\alpha_t}$ | 单位圆弧 | DDPM(多步随机)/DDIM |

而且，**给定边际速度场就能反推 score，反之亦然**:$\nabla\log p_t(x)$ 和 $u_t(x)$ 通过 $\alpha,\sigma$ 线性互换(同 Tweedie 精神)。所以 FM 训练的速度场可以转成 score 去跑 SDE 采样，扩散的 score 也能转成速度场跑 ODE——**它们是同一对象的不同坐标，可自由互转。** 这是 §8.4.7 测度论统一的具体体现。

### Rectified Flow：把路径反复拉直(骨架)

Rectified Flow(Liu et al. 2023)的洞察：第一次 FM 训出的速度场，其诱导的 ODE 轨迹仍可能弯曲(因为边际速度场是条件直线的平均，平均后会弯)。**Reflow**：用训好的模型采一批 $(x_0,x_1)$ 配对(把噪声映到数据)，然后**用这些配对重新做 FM 训练**——新的配对让直线路径更"不交叉"，诱导的边际速度场更直。迭代数次，轨迹趋于直线，可单步生成。

> **定理(Rectified Flow 凸代价不增，骨架)**。reflow 操作不增加任何凸传输代价 $\mathbb E[c(x_1-x_0)]$($c$ 凸)，且严格减少轨迹的"弯曲度"(除非已是直线)。证明骨架：reflow 保持边际不变(仍连接 $p_0,p_\text{data}$)，而重新配对让传输更接近最优传输(OT)耦合，凸代价由 Jensen 不增。$\blacksquare$(骨架)

> **本质洞察**:Rectified Flow 揭示了"路径直度"和"采样步数"是同一枚硬币——**轨迹越直，数值积分误差越小，所需步数越少，极限是单步。** 而拉直的代价是多轮 reflow 训练(训练换推理速度)。机器人语境：$\pi_0$ 用 FM 的直线路径换来 50 Hz 实时性，本质上是把"采样步数预算"这个推理瓶颈，通过 FM 的直线设计转移到了训练阶段。这与工程里"预计算/缓存换运行时速度"是同一权衡。

### ⚠️ 常见陷阱

💡 **概念误区：以为条件速度场 $x_1-x_0$ 就是采样时用的速度场**
- 新手想法："训练目标是 $x_1-x_0$，采样时直接走 $x_1-x_0$ 方向？"
- 实际上：采样用的是**边际**速度场 $v_\theta(x,t)$(网络输出)，它是条件速度场关于后验的期望(Lipman 恒等式)。$x_1-x_0$ 只是训练标签，采样时我们没有 $x_1$(那正是要生成的！)。
- 为什么重要：混淆这两者会让你以为"采样需要知道终点 $x_1$"，陷入循环。网络的作用正是从当前 $x$ 估计出"平均该往哪走"。

💡 **概念误区：认为 FM 的直线路径让边际速度场也是常数**
- 新手想法："条件路径是直线、条件速度场是常数 $x_1-x_0$，那边际速度场也该是常数。"
- 实际上：边际速度场是许多条直线速度的后验加权平均，**这个平均随 $x,t$ 变化，通常不是常数**(故边际 ODE 轨迹仍弯)。Rectified Flow 的 reflow 就是为了把它拉直。
- 根本原因：平均"抹平"了方向，只有当所有经过 $x$ 的直线方向一致(OT 耦合)时边际才直。

🧠 **思维陷阱：把 FM 和扩散当成竞争的两套理论**
- 新手想法："Flow Matching 是不是要取代扩散？学哪个？"
- 实际上：二者是同一连续性方程的不同路径选择(Stochastic Interpolants 统一)，速度场与 score 可自由互转。FM 不是"取代"，而是"换了条更直的路径 + 直接学速度场"。同一个网络，换个目标和路径就在两者间切换。
- 正确做法：理解统一框架，按需选路径(要随机纠错/多样性 → 扩散 SDE；要少步实时 → FM ODE)。

### 练习

1.(证明题，草稿纸上完成)独立重证 Lipman 等价恒等式：不看正文，从连续性方程出发证明边际速度场 $u_t(x)=\mathbb E_{p(x_1\mid x)}[u_t(x\mid x_1)]$，再证 $\mathcal L_\text{FM}$ 与 $\mathcal L_\text{CFM}$ 同梯度。这是本专题"必完证"之一，务必能独立复述。

2.(推导题)对高斯条件路径 $p_t(x\mid x_1)=\mathcal N(x;\alpha(t)x_1,\sigma(t)^2 I)$(一般 $\alpha,\sigma$)，推导条件速度场 $u_t(x\mid x_1)$。提示：从 $x=\alpha(t)x_1+\sigma(t)x_0$ 求 $\dot x$，再用 $x_0=(x-\alpha x_1)/\sigma$ 消去 $x_0$，得到 $u_t(x\mid x_1)=\dot\alpha x_1+\frac{\dot\sigma}{\sigma}(x-\alpha x_1)$。验证 FM 线性路径($\alpha=t,\sigma=1-t$)给出 $x_1-x_0$。

3.(跨章综合题，综合 §8.4.4 Tweedie + §8.4.6)证明：在高斯条件路径下，FM 的边际速度场 $u_t(x)$ 与扩散 score $\nabla\log p_t(x)$ 通过 $u_t(x)=\frac{\dot\alpha}{\alpha}x+\big(\sigma^2\frac{\dot\alpha}{\alpha}-\sigma\dot\sigma\big)\nabla\log p_t(x)$ 互换(系数仅依赖 $\alpha,\sigma$)。提示：用 §8.4.4 Tweedie 把 $\nabla\log p_t$ 换成 $\mathbb E[x_1\mid x]$，代入第 2 题的 $u_t(x\mid x_1)$ 取后验期望。这道题正式焊接"速度场视角"和"score 视角"。

---

## §8.4.7 测度论骨架：Fokker-Planck、Wasserstein 梯度流与 Girsanov ⭐⭐⭐⭐

### 动机：把前面所有视角"装进同一个几何"

前六节我们见了四种视角：SDE(随机轨迹)、ODE(确定轨迹)、score(密度梯度)、velocity(速度场)。它们反复被证明等价，但"为什么必然等价"还缺一个最高层的统一解释。本节给出这个解释：**把概率分布 $p_t$ 看成一个无穷维空间(概率测度空间 $\mathcal P_2$)里的一个点，$p_t$ 随时间的演化就是这个空间里的一条曲线；而扩散/FM 不过是让这条曲线"走得快"或"走直线"的不同方式。** 这个视角叫**最优传输几何**(optimal transport geometry)，它的核心定理是 JKO(Jordan-Kinderlehrer-Otto):**Fokker-Planck 方程是某个能量泛函在 Wasserstein 度量下的梯度流。**

这一节是博士级(⭐⭐⭐⭐)的"理论顶板"，主干学习可跳过(导航已注明)。但它回答了最深的"为什么"，且 Girsanov 定理是 §8.4.5 收敛证明的引擎，值得专门讲清。

### 如果只停留在 SDE/PDE 层面会怎样

反事实：不上升到测度几何，我们就只能"逐个验证"各视角等价(像前面那样一次次算 FP 方程对上)，却看不到**它们为什么是同一回事的不同侧面**。测度几何提供"上帝视角"：一旦知道"$p_t$ 是 $\mathcal P_2$ 中的曲线、速度场是它的切向量、score 是某泛函的梯度"，所有等价关系都成了"同一条曲线的不同坐标描述"，无需逐一验证。这就像从"逐个验证旋转矩阵性质"上升到"理解 SO(3) 是李群"(呼应李群专题)——抽象层级提高后，零散事实变成必然结构。

> **本质洞察**：数学的威力常来自"把动态对象嵌入正确的几何空间"。把概率分布的演化嵌入 Wasserstein 空间后，"扩散"这个随机过程变成了一条确定的测地线式曲线，"加噪"变成"沿熵增方向流动","去噪/生成"变成"逆着这条曲线爬回去"。**Wasserstein 几何是连接随机分析、偏微分方程、最优传输、信息论的枢纽，而扩散模型恰好坐落在这个枢纽上——这是它理论如此丰富的根本原因。** 把它和你已知的对照：这与广义相对论"把引力几何化为时空曲率"是同一种思想——把"力/演化"翻译成"几何"，让动力学变成沿几何走。

### 理论：Wasserstein 距离与 $\mathcal P_2$ 空间

**2-Wasserstein 距离**：两个概率分布 $\mu,\nu$ 之间的"最优搬运成本":

$$
W_2(\mu,\nu)^2=\inf_{\gamma\in\Pi(\mu,\nu)}\int\|x-y\|^2\,\mathrm d\gamma(x,y)
$$

$\Pi(\mu,\nu)$ 是所有边际为 $\mu,\nu$ 的耦合(联合分布)。直觉：把一堆沙子($\mu$)搬成另一堆形状($\nu$),$W_2^2$ 是最小的"搬运量 $\times$ 距离平方"。配上 $W_2$，概率测度空间 $\mathcal P_2$ 成为一个(无穷维)度量空间，甚至有黎曼结构(Otto 微积分)。

> **本质洞察(为什么是 $W_2$ 而非 KL)**：为什么用 Wasserstein 而非更常见的 KL?**KL 散度不是距离(不对称、不满足三角不等式)，且对"分布的几何位移"不敏感**——两个分离的窄峰，无论离多远 KL 都是无穷(若支撑不交)。而 $W_2$ 真正度量"形状要移动多远"，对几何位移线性敏感。扩散把数据"搬"到噪声再"搬"回来，本质是几何搬运，故 $W_2$ 才是它的自然度量。这解释了为什么 FM 的最优传输路径(直线)在 $W_2$ 几何里是测地线——直线是搬运成本最小的路径。

### 理论：JKO 定理 —— Fokker-Planck 是 $W_2$ 梯度流(骨架)

> **定理(Jordan-Kinderlehrer-Otto 1998，骨架)**。考虑自由能泛函 $\mathcal F[p]=\underbrace{\int U\,p\,\mathrm dx}_{\text{势能}}+\underbrace{\int p\log p\,\mathrm dx}_{\text{熵(负)}}$。则 Fokker-Planck 方程 $\partial_t p=\nabla\cdot(p\nabla U)+\Delta p$ 是 $\mathcal F$ 在 $W_2$ 度量下的**梯度流**:
> $$
> \partial_t p_t=-\nabla_{W_2}\mathcal F[p_t]
> $$
> 即 $p_t$ 沿着自由能下降最陡的方向(在 Wasserstein 几何意义下)演化。等价地，它是如下隐式时间离散(JKO 格式)的极限：
> $$
> p_{k+1}=\arg\min_p\Big\{\mathcal F[p]+\frac{1}{2\tau}W_2(p,p_k)^2\Big\}
> $$

**证明骨架**:JKO 格式是"邻近点算法"(proximal point)在 $W_2$ 空间的实例——每步在"降低自由能"和"不要离上一步太远(按 $W_2$)"之间权衡。令步长 $\tau\to0$，用 Otto 微积分计算 $\mathcal F$ 的 $W_2$ 梯度：势能项的梯度给出漂移 $\nabla\cdot(p\nabla U)$，熵项的梯度给出扩散 $\Delta p$。两者相加正是 Fokker-Planck。关键引理是"$W_2$ 梯度 = $\nabla(\frac{\delta\mathcal F}{\delta p})$"(变分导数的梯度)。$\blacksquare$(骨架)

**对扩散的意义**：前向 VP-SDE 的 Fokker-Planck 正是某个自由能的 $W_2$ 梯度流——前向过程是"自由能下降(熵增)"，数据分布被熵驱动着流向高斯(最大熵分布)。**反向生成则是逆着这个梯度流爬升**，而 score 正是这个梯度流的"方向信息"。这给了 score 一个变分解释：它是自由能泛函变分导数的梯度。

> **本质洞察(前向是熵增，反向是抗熵)**:JKO 视角下，前向扩散 = 系统沿自由能梯度自发演化到平衡(熵最大的高斯)，这是热力学第二定律的字面体现；反向生成 = 逆熵而行，把无序噪声重新组织成有序数据——这需要"外部信息"输入，而这个信息恰恰就是 score(经由神经网络从数据学到)。**生成模型在做的，是用学到的 score 当"麦克斯韦妖"，局部地逆转熵增，把噪声雕成数据。** 这个热力学图景把 §8.4.1 开头"墨水扩散"的物理直觉提升到了精确的变分原理高度。类比边界：与麦克斯韦妖**像**在都用信息逆转熵增；**不像**在妖是无成本的理想装置，而 score 的获取(训练)消耗了大量数据和算力——信息不是免费的(对应 Landauer 原理)，没有违反热二定律。

### 理论：Girsanov 定理 —— 测度变换的引擎(骨架)

Girsanov 是 §8.4.5 收敛证明的核心工具，这里补上它本身。

> **定理(Girsanov，骨架)**。设两个 SDE 只差漂移项：$\mathrm dx_t=b_1\,\mathrm dt+g\,\mathrm dW_t$ 与 $\mathrm dx_t=b_2\,\mathrm dt+g\,\mathrm dW_t$(同扩散系数 $g$)，诱导路径测度 $\mathbb P_1,\mathbb P_2$。则它们相互绝对连续，且 Radon-Nikodym 导数(似然比)为
> $$
> \frac{\mathrm d\mathbb P_1}{\mathrm d\mathbb P_2}=\exp\Big(\int_0^T\tfrac{b_1-b_2}{g}\,\mathrm dW_t-\tfrac12\int_0^T\tfrac{\|b_1-b_2\|^2}{g^2}\,\mathrm dt\Big)
> $$
> 因此两测度的 KL 散度为
> $$
> \boxed{\ \mathrm{KL}(\mathbb P_1\|\mathbb P_2)=\tfrac12\,\mathbb E_{\mathbb P_1}\!\int_0^T\frac{\|b_1(x_t,t)-b_2(x_t,t)\|^2}{g(t)^2}\,\mathrm dt\ }
> $$

**证明骨架**：核心是"改变漂移相当于改变概率测度"。构造指数鞅(Doléans-Dade 指数)$Z_t=\exp(\int\theta_s\mathrm dW_s-\tfrac12\int\theta_s^2\mathrm ds)$,$\theta=(b_1-b_2)/g$;Novikov 条件保证 $Z_t$ 是真鞅；在新测度 $\mathrm d\mathbb P_1=Z_T\mathrm d\mathbb P_2$ 下，$\tilde W_t=W_t-\int\theta_s\mathrm ds$ 是布朗运动，从而 $x_t$ 在 $\mathbb P_1$ 下满足漂移 $b_1$ 的 SDE。KL 由 $\mathbb E[\log Z_T]$ 算出，鞅项期望为零，只剩二次项。$\blacksquare$(骨架)

**对扩散的意义**：这正是 §8.4.5 Step 1 用的公式——令 $b_1=$ 理想反向漂移(用真 score)、$b_2=$ 近似漂移(用 $s_\theta$)，漂移差 $b_1-b_2=g^2(\nabla\log p_t-s_\theta)$，代入得 KL $=\tfrac12\mathbb E\int g^2\|s_\theta-\nabla\log p_t\|^2\mathrm dt$。**Girsanov 是把"score 误差"翻译成"分布距离"的唯一桥梁**，没有它收敛理论无从谈起。

> **本质洞察(Girsanov 的不可替代性)**：为什么收敛证明非 Girsanov 不可？因为我们要比较的是**两个随机过程的整条轨迹分布**，而不是某一时刻的边际。逐时刻比较边际会丢掉轨迹的时间相关性，给出过松的界。Girsanov 直接在"路径测度"层面工作，把"两条随机轨迹分布的差"一步到位写成"漂移差的二次型时间积分"——这是任何逐点方法都做不到的。**它体现了随机分析的精髓：在路径空间(而非状态空间)上思考。** 这与机器人里"比较两条轨迹要在轨迹空间用泛函度量，而非逐点比位置"是同一层级的思维跃迁。

### 四重视角统一总表

至此，本专题的全部视角可以装进一张表——不是随意罗列，而是按"用什么对象描述演化"做穷举式分类：

| 视角 | 核心对象 | 演化方程 | 训练目标 | 采样 | 代表方法 |
|------|---------|---------|---------|------|---------|
| **SDE(随机)** | 随机轨迹 $x_t$ | Anderson 反向 SDE | $\varepsilon$-预测 / DSM | 随机积分(Euler-Maruyama) | DDPM、NCSN |
| **ODE(确定)** | 确定轨迹 $x_t$ | 概率流 ODE | 同上(共享 score) | 确定积分(可高阶) | DDIM、DPM-Solver |
| **score(梯度)** | 密度梯度场 $\nabla\log p_t$ | Fokker-Planck | score matching | 任选 SDE/ODE | Score SDE |
| **velocity(流)** | 速度场 $u_t$ | 连续性方程 | CFM | ODE 积分 | Flow Matching、$\pi_0$ |
| **测度(几何)** | 测度曲线 $p_t\in\mathcal P_2$ | $W_2$ 梯度流(JKO) | —— | —— | 理论统一框架 |

**它们靠什么互转**:Tweedie / 线性插值系数(§8.4.4、§8.4.6)在 score、$\varepsilon$、velocity 间换算；连续性方程把 score 与 velocity 联系；Anderson 定理把 SDE 与 ODE 联系；JKO 把这一切嵌入 $W_2$ 几何。**一句话：同一条 $\mathcal P_2$ 曲线，五种坐标。**

### ⚠️ 常见陷阱

💡 **概念误区：把 $W_2$ 梯度流的"梯度"当成普通函数梯度**
- 新手想法："梯度流不就是 $\dot x=-\nabla f$ 那种吗？"
- 实际上：$W_2$ 梯度是**无穷维测度空间**里的梯度(Otto 微积分)，涉及变分导数 $\frac{\delta\mathcal F}{\delta p}$ 再取空间梯度。它作用在分布上，不是在欧氏点上。
- 为什么重要：不理解这点会把 JKO 误读成普通 ODE，错失"分布演化是测度空间测地流"的核心图景。

💡 **概念误区：以为 Girsanov 对任意两个 SDE 都成立**
- 新手想法："Girsanov 能比较任何两个 SDE。"
- 实际上：Girsanov 要求两 SDE **扩散系数相同**(只能差漂移)。若扩散系数也不同，两测度可能相互奇异(不绝对连续),KL 为无穷，Girsanov 失效。
- 根本原因：扩散系数决定路径的"二次变差"，不同二次变差的测度无法相互绝对连续。
- 正确做法：用 Girsanov 前先确认扩散系数一致——扩散模型里前向反向 $g$ 相同，正好满足。

🧠 **思维陷阱：认为测度论视角"只是花哨的重新包装"**
- 新手想法："前面用 SDE/PDE 都推出来了，$\mathcal P_2$ 几何是不是多余的装饰？"
- 实际上：测度视角给出新工具和新结果——JKO 启发了"扩散即梯度流"的采样器设计，$W_2$ 几何给出 FM 直线路径的最优性证明，Otto 微积分是分析收敛速率的利器。它不是包装，是生产力。
- 正确做法：主干学习可跳过，但做扩散理论研究必须掌握——它是当前理论论文的通用语言。

### 练习

1.(推导题，草稿纸上完成)对一维高斯 $\mu=\mathcal N(m_1,\sigma_1^2)$, $\nu=\mathcal N(m_2,\sigma_2^2)$，证明 $W_2(\mu,\nu)^2=(m_1-m_2)^2+(\sigma_1-\sigma_2)^2$。提示：一维最优耦合是分位数匹配(单调重排)。对比 KL 散度的表达式，体会"$W_2$ 对均值位移线性敏感、KL 对方差比敏感"的差异。

2.(证明题)用 Girsanov 的 KL 公式独立推出 §8.4.5 Step 1 的 score 误差积分：设理想反向漂移 $b_1=f-g^2\nabla\log p_t$、近似漂移 $b_2=f-g^2 s_\theta$，代入 $\mathrm{KL}=\tfrac12\mathbb E\int\frac{\|b_1-b_2\|^2}{g^2}\mathrm dt$，验证得 $\tfrac12\mathbb E\int g^2\|s_\theta-\nabla\log p_t\|^2\mathrm dt$。这把 §8.4.5 的"黑箱公式"亲手打开。

3.(开放思考题)JKO 把前向扩散解释为自由能 $\mathcal F[p]=\int Up+\int p\log p$ 的梯度流。请论证：若把势能 $U$ 设为 $\tfrac12\|x\|^2$(对应 VP 的线性回拉)，平衡分布(梯度流终点，$\nabla_{W_2}\mathcal F=0$)恰为 $\mathcal N(0,I)$。这从变分原理解释了"为什么 VP 前向必然收敛到标准高斯"。再思考：若换一个 $U$，前向会收敛到什么？(这是设计非高斯先验扩散的理论入口。)

---

## §8.4.8 机器人落地：从图像扩散到动作生成 ⭐⭐⭐

### 动机：为什么生成模型适合做机器人策略

前七节建立了完整的扩散数学。本节回答"这套数学怎么用到机器人"，并把前文反复埋下的跨章伏笔(MPPI = 单步去噪等)精确兑现。

先说一个根本问题：机器人策略 $a=\pi(o)$(给观测 $o$ 出动作 $a$)，为什么要用生成模型而非普通回归网络？答案在**多峰性**(multimodality)。考虑一个抓取任务：杯子在正前方，从左手绕或从右手绕都合理。普通回归网络用 MSE 损失，学到的是所有合理动作的**平均**——而"左绕"和"右绕"的平均是"直直撞上去"，一个灾难性的动作。

> **本质洞察(MSE 平均 vs 分布建模)**：这是扩散策略压倒性优势的根源——**回归网络学的是条件均值 $\mathbb E[a\mid o]$，生成网络学的是条件分布 $p(a\mid o)$。** 当 $p(a\mid o)$ 多峰时，均值落在峰之间的低密度区(无效甚至危险动作)。扩散策略不取平均，而是从分布里采样，每次采到某一个峰(这次左绕、下次右绕)，每个都是有效动作。这与前置自测 5、§8.4.3 的"多步复合出多峰"一脉相承：扩散能表达多峰，所以能避免"平均成灾难"。把它和你已知的对照——这正是 §8.4.5 "扩散不需 LSI、天然处理多峰"的洞察在机器人决策层面的兑现。类比边界：与"分类 vs 回归"**像**在都区分"选一个模式"和"取连续平均";**不像**在分类的模式是离散预设的，而扩散的模式是从连续动作空间里自动浮现的。

### 应用一：Diffusion Policy(Chi et al. RSS 2023)—— DDPM 做视觉运动策略

**核心思想**：把"给观测生成动作序列"建模为**条件**扩散。动作序列 $A=(a_t,a_{t+1},\dots,a_{t+H-1})$(预测未来 $H$ 步)是被生成的对象 $x_0$，观测 $o$(几帧图像+本体状态)是条件。直接套用 §8.4.3 的条件 DDPM:

$$
L=\mathbb E_{A^0,k,\varepsilon,o}\Big[\big\|\varepsilon-\varepsilon_\theta(\underbrace{\sqrt{\bar\alpha_k}A^0+\sqrt{1-\bar\alpha_k}\varepsilon}_{A^k},\,k,\,o)\big\|^2\Big]
$$

**这就是 §8.4.3 的 $L_\text{simple}$ 加了观测条件 $o$。** 现在你应该完全理解每个符号：$A^0$ 是干净动作序列(专家演示),$k$ 是扩散步，$A^k$ 是加噪动作，$\varepsilon_\theta$ 是去噪网络(以观测 $o$ 为条件)。训练就是 §8.4.4 的去噪 score matching；采样就是 §8.4.2 的反向过程(DDPM 用随机 SDE 离散)。

**与本专题数学的逐条对应**：

| Diffusion Policy 的做法 | 本专题对应理论 | 为什么这样做 |
|------------------------|--------------|------------|
| 预测噪声 $\varepsilon$ 而非动作 | §8.4.3 $\varepsilon$-预测、§8.4.4 Tweedie | 尺度恒定、训练稳 |
| 用 DDPM 随机采样 | §8.4.2 反向 SDE($\lambda=1$) | 随机性纠错，对不完美 score 鲁棒 |
| 预测整段动作序列(action chunking) | 多峰序列 = 高维多峰分布 | §8.4.3 复合出多峰、§8.4.5 不需 LSI |
| Receding horizon 执行(只执行前几步) | —— | 闭环重规划，呼应 MPC |
| 100 步 DDPM 或 10 步 DDIM | §8.4.3 DDIM = PF-ODE 离散 | 实时性与质量权衡 |

> **本质洞察(action chunking 的扩散根据)**:Diffusion Policy 一次生成一整段动作序列(而非单步动作)，这不是工程随意选择，而是扩散数学的自然结果。**单步动作分布可能简单，但"一段时间相关的动作序列"是高维强相关的多峰分布——正是扩散最擅长、回归最无力的对象。** 而且整段生成自动保证了动作的时间一致性(不会前后矛盾)，因为它建模的是序列的联合分布而非逐步条件。这与 §8.4.3 "反向链建模联合分布"直接对应。

### 应用二：$\pi_0$(Black et al. 2024)—— Flow Matching 做实时 VLA

$\pi_0$ 是视觉-语言-动作(Vision-Language-Action, VLA)模型，要在真实机器人上 50 Hz 控制。它的动作生成头用 **Flow Matching**(§8.4.6)而非 DDPM，核心原因就是 §8.4.6 讲的：**FM 的直线路径让采样只需极少步($\pi_0$ 用约 10 步 Euler 积分)，满足实时性。**

$\pi_0$ 的动作流匹配损失(对应 §8.4.6 的 $\mathcal L_\text{CFM}$ 加条件):

$$
L=\mathbb E_{t,A^1,A^0,o,\ell}\big\|v_\theta(\underbrace{tA^1+(1-t)A^0}_{A^t},\,t,\,o,\ell)-(A^1-A^0)\big\|^2
$$

$A^1$ 是专家动作序列、$A^0\sim\mathcal N(0,I)$ 是噪声、$o$ 是视觉观测、$\ell$ 是语言指令。采样时从 $A^0$ 出发，用学到的速度场 $v_\theta$ 积分 ODE 到 $A^1$。

**Diffusion Policy(DDPM)vs $\pi_0$(FM)的对比**(不是哪个更好，而是各自换取了什么):

| 维度 | Diffusion Policy(DDPM) | $\pi_0$(Flow Matching) |
|------|------------------------|-------------------|
| 反向过程 | 随机 SDE($\lambda=1$) | 确定 ODE($\lambda=0$) |
| 路径 | VP 圆弧 | 直线(OT) |
| 采样步数 | 100(DDPM)/10–50(DDIM) | ~10(直线路径省步) |
| 对 score 误差 | 鲁棒(随机纠错) | 需更准的速度场 |
| 实时性 | 中 | 高(50 Hz) |
| 训练目标 | $\varepsilon$-预测 | velocity 预测 |

> **本质洞察**:Diffusion Policy 和 $\pi_0$ 的选择差异，完美映射了 §8.4.2 的 $\lambda$ 族——前者选 $\lambda=1$(随机端，要鲁棒)，后者选 $\lambda=0$(确定端，要快)。**同一套扩散数学，通过调 $\lambda$(随机性强度)和路径(圆弧 vs 直线)，适配了"离线模仿要鲁棒"和"在线控制要实时"两种截然不同的机器人需求。** 这是理论统一性的最佳实战注脚：你不需要学两套方法，你学的是一个连续谱，两个机器人系统只是谱上的两点。

### 应用三：Diffuser(Janner et al. ICML 2022)—— 规划即去噪

Diffuser 把**轨迹优化**(trajectory optimization)重新表述为扩散。它对整条状态-动作轨迹 $\tau=(s_0,a_0,s_1,a_1,\dots)$ 做扩散建模，生成 = 去噪出一条可行轨迹。

**最妙的是它处理"目标/约束"的方式——classifier guidance(分类器引导)**：想让生成的轨迹满足某目标(如到达终点、避开障碍、最大化回报)，不必重训模型，只需在反向采样的每一步，把 score 加上目标函数 $\mathcal J$ 的梯度：

$$
\tilde s(\tau,t)=\underbrace{\nabla\log p_t(\tau)}_{\text{可行性 score}}+\underbrace{w\,\nabla_\tau\mathcal J(\tau)}_{\text{目标引导}}
$$

第一项把轨迹拉向"动力学可行"(数据流形)，第二项把它拉向"高回报"。这是**贝叶斯**的：$p(\tau\mid\text{目标})\propto p(\tau)\,e^{\mathcal J(\tau)}$，取对数梯度即上式。

> **本质洞察(规划 = 可行性先验 $\times$ 目标似然)**:Diffuser 揭示了一个深刻的统一——**轨迹优化的两大诉求(可行性 + 最优性)恰好对应贝叶斯的先验 $\times$ 似然。** 扩散模型当"可行性先验"(它学了什么样的轨迹是物理可行的)，目标函数当"似然",classifier guidance 把两者在 score 层面相加。这把"硬约束优化"软化成"概率推断"，而且约束和目标可以在采样时灵活组合(换个 $\mathcal J$ 就换个任务，无需重训)。**这正是连接"生成模型"与"最优控制/规划"的桥梁，也是 §8.4.2 那个"score 像值函数梯度"类比的精确兑现**——这里 $\nabla\mathcal J$ 就字面上是回报的梯度，和 score 相加共同决定轨迹走向。

### 引导(Guidance)：条件生成的数学，所有机器人应用的公共底层

上面 Diffuser 的 classifier guidance 只是冰山一角。引导是把"无条件扩散"变成"可控条件扩散"的通用机制，Diffusion Policy 的观测条件、Diffuser 的目标、文本到图像的提示词，本质都是引导。这里系统讲清它，因为它是所有机器人扩散应用的公共底层。

**核心恒等式——条件 score 的贝叶斯分解。** 我们想从条件分布 $p(x\mid c)$ 采样($c$ 是条件：观测、目标、指令)。对它取对数梯度，用贝叶斯 $p(x\mid c)\propto p(x)p(c\mid x)$:

$$
\nabla_x\log p_t(x\mid c)=\underbrace{\nabla_x\log p_t(x)}_{\text{无条件 score}}+\underbrace{\nabla_x\log p_t(c\mid x)}_{\text{条件似然梯度}}
$$

**这个分解是引导的全部数学基础。** 它说：条件 score = 无条件 score + 一个"把 $x$ 往'更符合条件 $c$'方向推"的修正项。两种实现路线：

**(1) Classifier Guidance(分类器引导，Dhariwal & Nichol 2021)**：单独训一个分类器 $p_\phi(c\mid x_t)$(在带噪 $x_t$ 上判断条件)，用它的梯度做修正项。加一个**引导强度** $w$ 放大效果：

$$
\tilde s(x,t)=\nabla\log p_t(x)+w\,\nabla_x\log p_\phi(c\mid x)
$$

Diffuser 的 $\nabla\mathcal J$ 就是这一路($\mathcal J=\log p(c\mid x)$,$c$="高回报")。缺点：要额外训一个噪声鲁棒的分类器。

**(2) Classifier-Free Guidance(无分类器引导，Ho & Salimans 2022)**：不训分类器，而是训一个**同时支持有/无条件**的网络——训练时随机丢弃条件($c$ 以一定概率置空 $\varnothing$)，得到 $\varepsilon_\theta(x,t,c)$ 和 $\varepsilon_\theta(x,t,\varnothing)$，采样时把两者外推，用它们的**差**当条件似然梯度的代理：

$$
\boxed{\ \tilde\varepsilon_\theta(x,t,c)=(1+w)\,\varepsilon_\theta(x,t,c)-w\,\varepsilon_\theta(x,t,\varnothing)\ }
$$

$w=0$ 是标准条件生成，$w>0$ **过度强化**条件(更贴合 $c$ 但多样性下降)。这是当今文生图、VLA 条件控制的主流(实现简单、无需额外分类器)。它从贝叶斯分解到外推公式的完整推导，以及在 Diffusion Policy/$\pi_0$ 上的机器人解读，留到 §8.4.8 应用七(那里 $c$ 就是观测/语言条件)——本处先建立"引导 = 无条件 score + 条件修正"的统一图景。

> **本质洞察(引导强度 $w$ 是"保真度 vs 多样性"旋钮)**：为什么 $w>0$ 有意义？标准条件采样($w=0$)从真实 $p(x\mid c)$ 采；$w>0$ 实际是从 $p(x\mid c)^{1+w}/p(x)^{w}$(把条件似然抬 $1+w$ 次方)采——**这个分布比真实条件分布更"尖锐"地集中在'最符合条件'的样本上**。代价是牺牲多样性(分布变窄)。这与统计里的"温度缩放/退火"是同一旋钮：$w$ 大 = 低温 = 尖锐 = 高保真低多样；$w$ 小 = 高温 = 弥散 = 低保真高多样。机器人语境：Diffusion Policy 调 $w$ 就是在"严格服从当前观测(可能过拟合)"和"保留动作多样性(更鲁棒)"之间权衡。把它和你已知的对照——这与强化学习里 softmax 策略的温度参数控制探索-利用，是字面相同的数学旋钮。

类比边界：引导强度 $w$ 与"放大镜倍率"**像**在都"放大某个方向(符合条件的部分)";**不像**在放大镜不改变图像分布，而 $w$ 实际改变了被采样的概率分布(把条件似然提到 $1+w$ 次方)，过大会"放大到失真"(样本不再真实)。不要把 $w$ 当成无副作用的"调好一点"旋钮——它有保真-多样性的本质权衡。

**与 Diffusion Policy 的连接**:Diffusion Policy 用的是"条件扩散"(观测 $o$ 直接作为网络输入)，属于把条件**内建**进网络的做法(等价于 classifier-free 的 $\varepsilon_\theta(\cdot,c)$ 分支)，通常不额外放大($w=0$)，因为动作要的是真实多峰分布而非"过度自信"的单峰。而 Diffuser 用 classifier guidance(目标作为外部 $\mathcal J$)，因为目标(回报)是采样时才指定的、可变的。**两种引导对应"条件是否在训练时已知"：训练时已知(观测)→ 内建条件；采样时才定(目标)→ 外部 guidance。** 这是一个清晰的工程选型准则。

### 应用四：SE(3)-DiffusionFields(Urain et al. ICRA 2023)—— 流形上的扩散

抓取位姿不是欧氏向量，而是 $SE(3)$ 元素(回顾李群专题：$SE(3)$ 是 3D 刚体位姿的李群，弯曲空间，不能直接加噪)。SE(3)-DiffusionFields 把扩散搬到 $SE(3)$ 流形上，生成 6-DoF 抓取位姿分布。

**核心挑战与本专题的连接**:§8.4.1 的前向加噪 $x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon$ 是欧氏的——在 $SE(3)$ 上不成立(两个位姿不能这样线性组合，会跌出流形)。解决方案用李群专题的工具：在**李代数** $\mathfrak{se}(3)$(切空间，局部欧氏)里加噪，通过指数映射搬回流形：

$$
T_t=T_0\,\exp\big((\sqrt{1-\bar\alpha_t}\,\xi)^\wedge\big),\quad \xi\sim\mathcal N(0,I)\in\mathbb R^6
$$

score 也定义在李代数上($\nabla\log p_t$ 变成流形上的黎曼梯度)。

> **本质洞察(扩散 + 几何 = 流形扩散)**:SE(3) 扩散是本专题与李群专题的交汇——**扩散的"在切空间加噪、几何上演化"和李群的"在李代数线性化、群上运算"，是同一个'局部欧氏'哲学的两次应用。** §8.4.1 我们说前向加噪要简单(高斯)，李群专题说扰动要放在李代数(切空间)；流形扩散把两者结合：在 $\mathfrak{se}(3)$(切空间)加高斯噪、用指数映射搬到 $SE(3)$。这呼应了李群专题"在弯曲群空间保存状态、在切空间表示扰动"的核心思想。类比边界：与"地图投影"(李群专题用过的类比)**像**在都用局部平面近似弯曲空间；**不像**在扩散还要在这个局部图上做一整套加噪-去噪动力学，比单纯的坐标投影复杂一层。

### 应用五：MPPI = 单步去噪(与 04 采样式 MPC 的精确桥接)

这是本专题最重要的跨章桥接，呼应 `04_移动机器人规控/20_采样式MPC/60_扩散启发采样MPC.md`。**论点：采样式 MPC 的 MPPI 更新，数学上就是扩散反向过程的单步去噪；而多级退火 MPPI 就是完整反向扩散。**

回顾 MPPI(Model Predictive Path Integral)：采 $N$ 条带噪控制序列 $U^{(i)}=\bar U+\delta U^{(i)}$，按代价 $S^{(i)}$ 加权平均更新：

$$
\bar U^{\text{new}}=\sum_i w_i\,U^{(i)},\quad w_i=\frac{\exp(-S^{(i)}/\lambda)}{\sum_j\exp(-S^{(j)}/\lambda)}
$$

**桥接(精确对应)**：把"低代价 = 高概率"翻译，定义目标分布 $p(U)\propto\exp(-S(U)/\lambda)$。则 MPPI 的加权平均更新正是在估计 $\mathbb E_{p}[U]$ 附近的一步——而由 Tweedie 公式(§8.4.4)$\mathbb E[U\mid\text{noisy}]=\text{noisy}+\sigma^2\nabla\log p$,**MPPI 的"加权平均"就是用样本估计 Tweedie 后验均值，即一步去噪！** 温度 $\lambda$ 对应噪声水平，MPPI 的高斯采样对应扩散单步的高斯。

| MPPI 概念 | 扩散对应 | 本专题节 |
|----------|---------|---------|
| 代价 $S(U)$ | 负对数目标密度 $-\lambda\log p(U)$ | §8.4.4 |
| softmax 加权平均 | Tweedie 后验均值 = 单步去噪 | §8.4.4 Tweedie |
| 温度 $\lambda$ | 噪声水平 $\sigma_t^2$ | §8.4.1 |
| 高斯采样扰动 | 单步高斯反向核 | §8.4.3 |
| 单级 MPPI | 固定噪声水平的单步去噪 | §8.4.2 |
| 多级退火($\lambda$ 递减) | 完整反向扩散($t:T\to0$) | §8.4.2 反向 SDE |

> **本质洞察(MPPI 是扩散的特例，退火是反向扩散)**：这个同构不是表面类比而是数学等式——**单级 MPPI 困在高斯天花板，正因为它只做"一个固定噪声水平的单步去噪"，而单步去噪只能输出后验均值(单峰高斯)；多级退火 MPPI 突破天花板，正因为它做了"噪声水平递减的多步去噪"，而多步复合能雕出多峰(前置自测 5、§8.4.3)。** 所以 Model-Based Diffusion、DIAL-MPC 这些"扩散启发的 MPC"不是把两个领域硬凑，而是认识到它们本就是同一套数学。这意味着：图像扩散里的噪声调度、采样器经验，可以直接迁移到力矩级控制——这是 04 专题的核心论点，现在你有了完整数学根据。把它和你已知的对照：这与"卡尔曼滤波是贝叶斯滤波的高斯特例"是同一种"具体算法是一般框架的特例"的认知，看穿后两个领域的工具库瞬间打通。

### 应用六：与 VLA 动作头的桥接(呼应 06 具身智能)

呼应 `01_数学/90_深度学习数学/50_具身智能VLA框架.md` 和 06 具身 VLA 章节：现代 VLA($\pi_0$、RDT-1B、Octo 等)的"动作头"(action head)普遍用扩散/FM 生成连续动作。本专题为理解这些动作头提供数学：

- **为什么 VLA 动作头用扩散而非离散 token**：连续动作的多峰分布(§8.4.8 开头)，离散化会丢精度且 token 化的动作仍难表达多峰；扩散直接在连续空间建模多峰。
- **为什么 $\pi_0$ 选 FM**:50 Hz 实时(§8.4.6 直线路径少步)。
- **大模型骨干 + 扩散头的分工**:VLM 骨干(语言-视觉理解)提供条件 $o,\ell$，扩散头(本专题)负责把条件映射为动作分布并采样。骨干学"理解"，头学"生成"——对应本专题"条件扩散"的条件 $o,\ell$ 与生成对象 $A$ 的分离。

> **本质洞察**:VLA = "大模型理解 + 扩散生成"的分工，反映了一个深刻的架构原则——**理解(从高维感知/语言抽取语义)和生成(从语义采出多峰动作)是两类不同的计算，适合不同的模型。** Transformer 骨干擅长前者(注意力聚合信息)，扩散头擅长后者(多步复合采多峰)。把它们拼接，各司其职。这与人脑"感知皮层理解 + 运动皮层生成动作"的分工是松散类比(边界：神经科学证据远未到一一对应，此处仅作架构直觉，不可当生物学事实)。

### 应用七：Classifier-Free Guidance —— 现代 VLA 的条件引擎

§8.4.8 应用三的 Diffuser 用 **classifier guidance**(分类器引导)：需要一个**外部**目标/分类器 $\mathcal J$，在采样时加它的梯度。但这有两个工程痛点：(i) 要单独训一个能在**各噪声水平**上都准的分类器 $\mathcal J(\tau,t)$(在加噪轨迹上分类，本身难训);(ii) 引导强度大时容易被分类器的对抗梯度带偏。现代 Diffusion Policy 与 $\pi_0$ 普遍改用 **classifier-free guidance(CFG, 无分类器引导)**(Ho & Salimans 2022)——它把"条件"直接焊进去噪网络，无需外部分类器。

**动机(机器人版)**:Diffusion Policy 要的"条件"是观测 $o$(图像+本体),$\pi_0$ 要的是 $(o,\ell)$(加语言)。我们希望生成的动作**强烈服从**这个条件(别忽略当前看到的障碍)，又不想为此训一个"动作-观测匹配度分类器"。CFG 的思路是：**让同一个网络既学条件生成 $\varepsilon_\theta(x_t,t,c)$、又学无条件生成 $\varepsilon_\theta(x_t,t,\varnothing)$(训练时以一定概率把条件 $c$ 置空)，采样时把两者外推。**

**推导(从贝叶斯到外推)**：我们想强化条件 $c$ 的影响，等价于从一个"被锐化的"后验 $p(x\mid c)^{1+w}/p(x)^{w}$ 采样(把条件似然抬 $1+w$ 次方)。取对数梯度(score 层面)，用贝叶斯 $p(x\mid c)\propto p(x)p(c\mid x)$:

$$
\nabla_x\log\frac{p(x\mid c)^{1+w}}{p(x)^{w}}=(1+w)\nabla_x\log p(x\mid c)-w\nabla_x\log p(x)
$$

换成 $\varepsilon$(用 §8.4.4 Tweedie $\varepsilon=-\sqrt{1-\bar\alpha_t}\,\nabla\log p$)，得到 **CFG 的去噪公式**:

$$
\boxed{\ \tilde\varepsilon_\theta(x_t,t,c)=(1+w)\,\varepsilon_\theta(x_t,t,c)-w\,\varepsilon_\theta(x_t,t,\varnothing)\ }
$$

$w=0$ 退化为普通条件生成；$w>0$ 把动作往"更符合观测 $c$"的方向外推(放大条件与无条件的差)。注意它**不需要外部分类器**——条件和无条件用的是**同一个网络**，只是把 $c$ 喂进去或置空。

> **本质洞察(CFG = 用"差"放大条件信号)**:CFG 的精妙在于——**$\varepsilon_\theta(x_t,t,c)-\varepsilon_\theta(x_t,t,\varnothing)$ 这个差，恰好是"条件 $c$ 给去噪方向带来的增量"**(由上面贝叶斯推导，它正比于 $\nabla\log p(c\mid x)$，即"似然 score")。沿这个差的方向多走 $w$ 步，就放大了条件的影响，而无需显式建模 $p(c\mid x)$。**这把 Diffuser 那个"需要外部分类器梯度 $\nabla\mathcal J$"的需求，变成了"同一网络两次前向之差"——分类器被隐式吸收进了条件/无条件的对比里。** 把它和你已知的对照：这与控制里"用前馈+反馈之差提取扰动信息"是同一种"用两个估计的差分离出感兴趣分量"的思想。类比边界：CFG 的"差"提取的是条件信号(确定性的语义增量)，与控制的扰动估计**像**在都靠差分离信号；**不像**在 CFG 两次前向共享网络权重、只差输入，而前馈/反馈是两套不同机制。

**机器人意义**:CFG 让 Diffusion Policy/$\pi_0$ 能用一个旋钮 $w$ 调"动作有多听观测的话"——$w$ 适中时动作既服从当前场景又保留多样性；$w$ 过大则可能过度锐化、丢多样性甚至放大网络在条件上的偏差(见下方陷阱)。这是 Diffuser 的 classifier guidance(需外部 $\mathcal J$、适合"加一个临时目标如到达某点")与 CFG(条件内生、适合"始终服从观测/语言")的分工：前者灵活加**任务级目标**，后者强化**固有条件**。

### 应用八：一次 Diffusion Policy 推理的端到端走查

把前面所有数学串成一次具体的推理，帮你把抽象公式落到"代码里到底发生了什么"。设动作维度 $d_a=7$(7-DoF 机械臂)、预测时域 $H=16$、用 DDIM 采 $K'=10$ 步、执行时域 $H_a=8$。

**输入**：当前观测 $o$(两帧 RGB + 关节角)。**目标**：产生接下来 $8$ 步要执行的动作。

| 步骤 | 发生了什么 | 对应理论 |
|:----:|-----------|---------|
| 0 | 视觉编码器把 $o$ 编成条件向量 $c$(整段去噪共享，缓存一次) | §8.4.8 应用二(条件缓存) |
| 1 | 采初始噪声 $A^{(K)}\sim\mathcal N(0,I)\in\mathbb R^{16\times7}$(整段动作序列) | §8.4.1 先验 $\pi$ |
| 2 | 对去噪步 $k=K,\dots,1$(DDIM 跳步，共 10 个 $k$)：网络输出 $\varepsilon_\theta(A^{(k)},k,c)$ | §8.4.3 $\varepsilon$-预测 |
| 3 | (可选 CFG)$\tilde\varepsilon=(1+w)\varepsilon_\theta(\cdot,c)-w\varepsilon_\theta(\cdot,\varnothing)$ | §8.4.8 应用七 |
| 4 | 由 $\tilde\varepsilon$ 算 $\hat A_0=\frac{1}{\sqrt{\bar\alpha_k}}(A^{(k)}-\sqrt{1-\bar\alpha_k}\,\tilde\varepsilon)$(Tweedie 预测干净序列) | §8.4.4 Tweedie |
| 5 | DDIM 更新到 $A^{(k-1)}$(确定性，$\sigma_k=0$,= 概率流 ODE 一步) | §8.4.3 DDIM=PF-ODE |
| 6 | 10 步后得 $A^{(0)}\in\mathbb R^{16\times7}$——一整段 16 步动作 | §8.4.2 反向过程终点 |
| 7 | 只取前 $H_a=8$ 步执行，丢弃后 8 步 | §8.4.8 滚动时域 |
| 8 | 执行完 8 步，拿新观测 $o'$，回到步骤 0 重新去噪 | §8.4.8 闭环重规划 |

**关键观察(把数学焊到工程)**：整个流程里，**真正"学到的东西"只有 $\varepsilon_\theta$ 这一个网络**——它隐式编码了"在观测 $c$ 下，加噪动作 $A^{(k)}$ 该往哪去噪"的全部分布信息(经 Tweedie 等价于条件 score)。步骤 4–5 全是**确定的代数**(无可学参数)，步骤 1 的随机性是多样性的唯一来源(§8.4.3 DDIM 多样性来自初值)。所以"调 Diffusion Policy"本质是调三件事：(a) $\varepsilon_\theta$ 训得多好(E2 误差，§8.4.5);(b) 用几步采样(E3 误差);(c) CFG 强度 $w$ 和滚动时域 $H_a$。

> **本质洞察(走查揭示的极简性)**：这次端到端走查暴露了扩散策略一个反直觉的简洁——**一个看似复杂的"多步迭代生成多峰动作序列"的系统，可学部分只有一个去噪网络，其余全是固定代数。** 这与"复杂行为来自简单规则的多次迭代"(元胞自动机、深度网络的深度)是同一种"简单部件 $\times$ 多步复合 = 复杂能力"的体现，也正是本专题反复强调的核心(前置自测 5)。理解了这点，你调参时就不会乱动那些"固定代数"部分，而会聚焦真正的三个旋钮。

### 应用九：VLA 动作头的生成方法选型全景(系统分类)

把本节学的方法收束成一张"选型地图"——现代具身智能(VLA)模型的动作头，本质都在回答同一个问题：**用哪种生成方式把"理解到的语义"变成"连续多峰动作"?** 下表不是罗列名字，而是按"建模对象 $\times$ 生成范式 $\times$ 采样代价"三维归类。

| 模型/方法 | 动作生成范式 | 视角(§8.4.7 表) | 采样代价 | 选这条路的理由 |
|----------|------------|------------------|---------|--------------|
| Diffusion Policy | 条件 DDPM(动作序列) | SDE / score | 中(10–100 步) | 离线模仿，要多峰 + 鲁棒，频率要求不极端 |
| $\pi_0$ / $\pi_{0.5}$ | 条件 Flow Matching | ODE / velocity | 低(~10 步) | 实时 VLA(50 Hz)，直线路径少步是命门 |
| RDT-1B | 条件 DDPM(双臂动作) | SDE / score | 中 | 大规模双臂操纵，沿用扩散的多峰建模 |
| Octo | 扩散动作头(可选) | SDE / score | 中 | 通用机器人 Transformer，模块化动作头 |
| $\pi_0$-FAST / 自回归 VLA | 离散动作 token 自回归 | (非扩散) | 取决于 token 数 | 复用 LLM 自回归机栈，但连续性/多峰受限 |
| Diffuser | 条件 DDPM(状态-动作轨迹) | SDE + guidance | 中 | 规划场景，需灵活加任务级目标(classifier guidance) |

**怎么读这张表(给选型者的决策框架)**:

1. **建模对象决定空间**：动作序列(欧氏)→ 直接扩散/FM；位姿(SE(3))→ 流形扩散(应用四)；状态-动作轨迹 → Diffuser 式规划。
2. **频率需求决定范式**：要实时(>30 Hz)→ FM 直线路径(ODE 少步)；可离线/低频 → DDPM(SDE 多步，换鲁棒+多样)。这正是 §8.4.2 $\lambda$ 族两端的工程投影。
3. **条件类型决定引导**：固有条件(观测/语言)→ CFG(应用七)；临时任务目标 → classifier guidance(应用三)。
4. **连续 vs 离散**：连续动作的精度和多峰几乎总让扩散/FM 胜过离散 token 自回归(§8.4.8 开头)；自回归的唯一优势是直接复用 LLM 基础设施。

> **本质洞察(动作头选型 = 在统一框架里选坐标和工作点)**：这张表最深的含义是——**所有这些"不同的 VLA 动作头"，在 §8.4.7 的四重视角统一框架里，都只是同一套生成数学的不同坐标(score/velocity)+ 不同路径(圆弧/直线)+ 不同采样器(SDE/ODE)+ 不同引导(CFG/classifier)的组合。** 你不需要把它们当成 N 个要分别学的"方法"，而是一个连续设计空间里的 N 个工作点。一旦你内化了本专题的统一视角，读任何一个新 VLA 的动作头，都能在 30 秒内把它定位到这个空间的某个角落，并立刻知道它的取舍。**这就是"理解一个框架胜过记住十个方法"的字面兑现**，也是本专题把"图像扩散数学"讲透后，留给机器人读者的最高价值——一张能容纳未来新方法的地图，而非一份会过时的方法清单。

### ⚠️ 常见陷阱

💡 **概念误区：以为 Diffusion Policy 比回归策略"只是更复杂"**
- 新手想法："回归网络也能出动作，扩散只是把它复杂化。"
- 实际上：本质差异是"学均值 vs 学分布"。多峰任务下回归的均值是无效动作(峰之间)，扩散采样每次得一个有效模式。这是能力差异，不是复杂度差异。
- 为什么重要：理解了才知道何时必须用扩散(多峰、多解任务)，何时回归够用(单峰、确定任务)。

💡 **概念误区：把 classifier guidance 的引导权重 $w$ 当成可随意调大**
- 新手想法："想更接近目标，把 $w$ 调大就行。"
- 实际上：$w$ 过大会让目标项压倒可行性 score，生成的轨迹"高回报但动力学不可行"(机器人执行会失败)。$w$ 是可行性与最优性的权衡旋钮，有上限。
- 根本原因：两项 score 在拔河，失衡则一方崩溃(§8.4.8 Diffuser 公式)。
- 正确做法：从小 $w$ 起调，监控生成轨迹的可行性(违反动力学约束的程度)。

🧠 **思维陷阱：认为 MPPI 和扩散的同构"只是类比，实践无用"**
- 新手想法："MPPI = 单步去噪听起来很妙，但我做 MPC 又不真的训神经网络。"
- 实际上：这个同构有实际产出——它告诉你 (a) 单级 MPPI 的局限(高斯天花板)来自哪、怎么破(多级退火);(b) 可以把扩散的噪声调度经验迁移到 MPPI 的温度退火；(c) Model-Based Diffusion 等新方法的设计原理。这是方法迁移的桥，不是空谈。
- 正确做法：做采样式 MPC 时，用"反向扩散"视角思考退火调度，往往比纯试错高效。

🧠 **思维陷阱：把"SE(3) 扩散"当成"欧氏扩散直接套用"**
- 新手想法："位姿是 6 个数，直接对这 6 个数做扩散不就行了？"
- 实际上：位姿在 $SE(3)$ 流形上，直接对欧拉角/四元数做欧氏加噪会破坏流形结构(跌出 $SO(3)$，呼应李群专题开头的反面案例)。必须在李代数加噪、指数映射搬回。
- 为什么重要：这是李群专题"旋转不能当欧氏向量"的教训在扩散里的重演——几何结构必须尊重。

### 练习

1.(开放思考题)Diffusion Policy 用 action chunking(一次生成 $H$ 步)。请论证：为什么生成整段序列比"逐步生成单个动作"更能保证时间一致性？用 §8.4.3"反向链建模联合分布"解释。再思考：$H$ 太大或太小各有什么问题？(联系 receding horizon。)

2.(跨章综合题，综合本节 + §8.4.4 + 04 采样式 MPC)严格推导 MPPI 更新与 Tweedie 单步去噪的对应：定义 $p(U)\propto e^{-S(U)/\lambda}$，设当前均值 $\bar U$ 为"带噪观测"，证明 MPPI 的 softmax 加权平均 $\sum_i w_i U^{(i)}$ 是 $\mathbb E[U\mid\bar U]$ 的蒙特卡洛估计，并用 Tweedie 把它写成 $\bar U+\sigma^2\nabla\log p_\sigma(\bar U)$ 的形式。指出温度 $\lambda$ 与噪声 $\sigma^2$ 的对应关系。

3.(跨章综合题，综合本节 + 李群专题 + §8.4.1)设计一个 $SO(3)$ 上的旋转扩散方案：写出在 $\mathfrak{so}(3)$(反对称矩阵，回顾李群专题 hat 运算)加噪、用矩阵指数 $\exp$ 搬回 $SO(3)$ 的前向过程，并说明 score 应定义在何处、反向采样如何用 retraction 更新。指出与欧氏 VP-SDE 相比，哪一步发生了本质改变，哪一步可以照搬。

4.(推导题，草稿纸上完成)从 CFG 公式 $\tilde\varepsilon=(1+w)\varepsilon_\theta(x_t,t,c)-w\varepsilon_\theta(x_t,t,\varnothing)$ 出发，用 §8.4.4 Tweedie 把它换回 score 形式，验证它等价于 $\nabla\log\big[p(x\mid c)^{1+w}p(x)^{-w}\big]$。再论证：$w\to\infty$ 时采样会发生什么(提示：后验被无限锐化，多样性如何变？这对机器人多峰动作意味着什么风险？)。

5.(开放思考题，综合应用七 + 应用三)Diffuser 用 classifier guidance(外部 $\mathcal J$),Diffusion Policy/$\pi_0$ 用 classifier-free guidance(条件内生)。请分维度对比二者：何时该用哪个？(提示：从"条件是固定的固有量(观测)还是临时的任务目标(到达某点)""是否愿意额外训一个噪声鲁棒的分类器"两个维度分析。)给出一个适合 classifier guidance、一个适合 CFG 的机器人场景。

6.(跨章综合题，综合应用八 + §8.4.5)在应用八的端到端走查里，把采样步数从 10 步减到 3 步。请用 §8.4.5 三源误差分解论证：哪一源误差会显著增大？如果发现 3 步采样的动作在段接合处跳变，你应该先怀疑是 score 不准(E2)还是步数不够(E3)？给出一个能区分二者的诊断实验(提示：固定网络只改步数，看质量是否随步数单调改善)。

---

## 本章常见误解汇总

下面把全专题最易绊倒人的误解集中成一张表，作为通读后的"查漏卡"。每条都给出**误解**与**正确理解**的对照，出处指回对应小节，便于回查。

| # | 常见误解 | 正确理解 | 出处 |
|---:|---------|---------|------|
| 1 | 前向加噪过程也要训练一个网络 | 前向**无任何可学参数**,$f,g$ 固定、转移核有闭式高斯解，全部学习在反向 | §8.4.1 |
| 2 | 扩散的表达力来自每步用了比高斯更复杂的分布 | 每步仍是高斯，表达力来自**多步复合**——简单步首尾相接雕出多峰 | §8.4.1、§8.4.3、前置自测 5 |
| 3 | 用普通链式法则算 $\mathrm d(x_t^2)$ 即可 | 必须用 Itô 公式，二阶项 $\tfrac12\phi''g^2$ 来自 $(\mathrm dW)^2=\mathrm dt$，漏掉得负方差 | §8.4.1 |
| 4 | $t\in[0,T]$ 是训练迭代步 | $t$ 是**扩散时间/噪声水平**，一个 batch 内不同样本配不同随机 $t$ | §8.4.1 |
| 5 | 反向过程需要知道后验 $p_{0\mid t}(x_0\mid x_t)$ | 只需**边际 score** $\nabla\log p_t$，后验那个复杂多峰量被绕开 | §8.4.2 |
| 6 | score $\nabla_x\log p_t$ 是对参数 $\theta$ 的梯度 | 是对**数据变量 $x$** 的梯度，一个 $\mathbb R^d\to\mathbb R^d$ 向量场；与 Fisher score 同名不同物 | §8.4.2 |
| 7 | 概率流 ODE 是反向 SDE 的近似、精度更低 | 二者**边际分布完全等价**(共享所有 $p_t$)，差异只在单条轨迹随机性 | §8.4.2 |
| 8 | ODE 采样一定比 SDE 采样准 | score 不准时 SDE 的随机性**提供纠错**反而更鲁棒；score 准时 ODE 可高阶少步 | §8.4.2、§8.4.5 |
| 9 | DDPM 的 1000 步是"加噪需要这么多步" | 加噪几十步即足；1000 步是为让"反向单步是高斯"的建模假设成立(小 $\beta_t$) | §8.4.3 |
| 10 | DDIM 确定性采样所以没有多样性 | 多样性来自**初值** $x_N\sim\mathcal N(0,I)$；固定初值才固定输出 | §8.4.3 |
| 11 | score matching 需要真实 score 当标签 | 显式 SM 用分部积分消去真值；DSM 用**条件 score** $-\varepsilon/\sigma$(=你加的噪声)当标签 | §8.4.4 |
| 12 | Tweedie 的 $\mathbb E[x\mid\tilde x]$ 是某个具体干净样本 | 是后验**均值**，高噪声区是"模糊平均脸"，未必在数据流形上 | §8.4.4 |
| 13 | $\varepsilon$-预测、score、$x_0$ 是三个不同的学习目标 | 是同一量的**三种坐标**，差一个只依赖 $\bar\alpha_t$ 的确定线性变换 | §8.4.4 Tweedie |
| 14 | 收敛界的具体常数可用来定超参 | 常数极松，只有**缩放规律**($N$ 随 $d,\varepsilon$ 怎么变)有用 | §8.4.5 |
| 15 | 条件速度场 $x_1-x_0$ 就是采样时用的速度场 | 采样用**边际**速度场 $v_\theta$(后验期望);$x_1-x_0$ 只是训练标签 | §8.4.6 |
| 16 | FM 直线条件路径 → 边际速度场也是常数 | 边际是条件直线的后验平均，**随 $x,t$ 变、通常仍弯**;reflow 才拉直 | §8.4.6 |
| 17 | Flow Matching 天生比扩散快 | 快来自**选了直线(OT)路径**，非框架自带；扩散选近直路径也能快 | §8.4.6 |
| 18 | Girsanov 对任意两个 SDE 都成立 | 要求两 SDE **扩散系数相同**，否则测度可能相互奇异、KL 为无穷 | §8.4.7 |
| 19 | $W_2$ 梯度流的"梯度"是普通函数梯度 | 是**无穷维测度空间**的梯度(Otto 微积分)，含变分导数再取空间梯度 | §8.4.7 |
| 20 | Diffusion Policy 的扩散步 $k$ 等于动作时间步 $H$ | 两个**正交维度**:$H$ 是动作物理时长，$k$ 是去噪迭代数；每步 $k$ 处理整段 $H$ 序列 | §8.4.8 |
| 21 | 位姿是 6 个数，直接对它们做欧氏扩散即可 | 位姿在 $SE(3)$ 流形，须在李代数加噪、指数映射搬回，否则跌出流形 | §8.4.8 |
| 22 | MPPI = 单步去噪只是类比、实践无用 | 是数学等式；它解释单级 MPPI 的高斯天花板、指导多级退火调度 | §8.4.8 |
| 23 | CFG 的引导强度 $w$ 越大越好 | $w$ 过大会过度锐化后验、丢多样性、放大网络在条件上的偏差；是有上限的旋钮 | §8.4.8 应用七 |

---

## 本章小结

本专题用一条主线贯穿：**前向加噪简单(线性 SDE 闭式可解)，反向去噪难，而反向去噪的全部信息浓缩在 score $\nabla\log p_t$ 一个函数里——估计它就是训练，沿它走就是采样。** 围绕这条主线，我们走完了"连续理论(§8.4.1–8.4.2)→ 可训练化(§8.4.3–8.4.4)→ 统一与保证(§8.4.5–8.4.7)→ 机器人落地(§8.4.8)"四个板块。下面用三张表把它沉淀为可查的速查卡。

### 符号表

本专题新引入的核心数学符号汇总(按首次出现排序):

| 符号 | 含义 | 首见 |
|------|------|------|
| $p_\text{data}$ | 数据分布(目标采样分布) | §8.4.1 |
| $x_t,\,\{x_t\}_{t\in[0,T]}$ | 前向扩散过程在时刻 $t$ 的随机变量 | §8.4.1 |
| $f(x,t),\,g(t)$ | 前向 SDE 的漂移系数、扩散系数 | §8.4.1 |
| $W_t,\,\bar W_t$ | 前向、反向时间的标准布朗运动 | §8.4.1、§8.4.2 |
| $p_t$ | $x_t$ 的边际密度 | §8.4.1 |
| $p_{t\mid0}(x_t\mid x_0)$ | 给定起点的前向转移核 | §8.4.1 |
| $\beta(t),\,\bar\beta(t)$ | 噪声调度及其累积 $\int_0^t\beta$ | §8.4.1 |
| $\bar\alpha_t$ | 累积保留率：连续为 $e^{-\bar\beta(t)}$，离散为 $\prod_s\alpha_s$ | §8.4.1、§8.4.3 |
| $\alpha_t=1-\beta_t$ | 单步保留率(离散链) | §8.4.3 |
| $\mathrm{SNR}(t)=\bar\alpha_t/(1-\bar\alpha_t)$ | 信噪比 | §8.4.1 |
| $\nabla_x\log p_t(x)$ | score(密度对数梯度向量场) | §8.4.2 |
| $v(x,t),\,u_t(x)$ | 概率流 ODE / Flow Matching 的速度场 | §8.4.2、§8.4.6 |
| $\lambda$(采样族) | 反向过程随机性强度($1$=SDE,$0$=ODE) | §8.4.2 |
| $J(x,t)$ | 概率流(probability current) | §8.4.2 |
| $\mu_\theta,\,\Sigma_\theta$ | DDPM 反向链的均值/协方差网络 | §8.4.3 |
| $\tilde\mu_t,\,\tilde\beta_t$ | 前向后验 $q(x_{t-1}\mid x_t,x_0)$ 的均值/方差 | §8.4.3 |
| $\varepsilon_\theta(x_t,t)$ | 噪声预测网络($\varepsilon$-预测) | §8.4.3 |
| $L_\text{simple},\,\mathcal L_\text{ELBO}$ | DDPM 简化损失、证据下界 | §8.4.3 |
| $\sigma_t$(DDIM) | DDIM 随机性参数($0$=确定) | §8.4.3 |
| $s_\theta(x,t)$ | score 估计网络 | §8.4.4 |
| $J_\text{ESM},\,J_\text{DSM}$ | 显式/去噪 score matching 目标 | §8.4.4 |
| $q_\sigma(\tilde x\mid x)$ | 加噪条件核 $\mathcal N(\tilde x;x,\sigma^2I)$ | §8.4.4 |
| $\mathbb E[x\mid\tilde x]$ | 后验均值(Tweedie 最优去噪) | §8.4.4 |
| $\varepsilon_\text{score}$ | score 估计的 $L^2$ 误差 | §8.4.5 |
| $\mathrm{TV},\,\mathrm{KL}$ | 全变差距离、KL 散度 | §8.4.5 |
| $\mathbb P_\text{ideal},\,\mathbb P_\text{approx}$ | 理想/近似反向过程的路径测度 | §8.4.5、§8.4.7 |
| $\mathcal L_\text{FM},\,\mathcal L_\text{CFM}$ | (条件)Flow Matching 损失 | §8.4.6 |
| $u_t(x\mid x_1)$ | 条件速度场 | §8.4.6 |
| $\alpha(t),\sigma(t)$(插值) | Stochastic Interpolants 的信号/噪声系数 | §8.4.6 |
| $W_2(\mu,\nu)$ | 2-Wasserstein 距离 | §8.4.7 |
| $\mathcal P_2$ | 有限二阶矩概率测度空间 | §8.4.7 |
| $\mathcal F[p]$ | 自由能泛函(势能 + 负熵) | §8.4.7 |
| $\nabla_{W_2}$ | Wasserstein 梯度(Otto 微积分) | §8.4.7 |
| $\frac{\mathrm d\mathbb P_1}{\mathrm d\mathbb P_2}$ | 路径测度的 Radon-Nikodym 导数 | §8.4.7 |
| $A,\,A^{(k)}$ | 动作序列及其加噪版本(Diffusion Policy) | §8.4.8 |
| $H,\,H_a$ | 预测时域、执行时域 | §8.4.8 |
| $c=\text{VLM}(\cdot)$ | $\pi_0$ 的视觉-语言条件 | §8.4.8 |
| $T\in SE(3),\,\xi\in\mathfrak{se}(3)$ | 抓取位姿、其李代数扰动 | §8.4.8 |
| $S(U),\,w_i,\,\lambda$(MPPI) | 控制序列代价、softmax 权重、温度 | §8.4.8 |

### 定理速查表

本专题核心定理/公式与一句话说明(按出现顺序；深度标注沿用速览表的 完/骨/陈):

| 定理/公式 | 一句话说明 | 深度 | 对应节 |
|----------|----------|:---:|--------|
| VP/VE 闭式转移核 | 线性 SDE 解是高斯，$x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon$，方差与 $x_0$ 无关 | 完 | §8.4.1 |
| 信噪比统一调度 | SNR 端点相同的调度训练等价(VDM)，调度只改训练权重 | 陈 | §8.4.1 |
| **Anderson 反向 SDE(1982)** | 反向漂移 $=f-g^2\nabla\log p_t$，把生成化归为估计 score | 完 | §8.4.2 |
| 概率流 ODE | $\dot x=f-\tfrac12 g^2\nabla\log p_t$，与 SDE 共享边际、轨迹确定 | 骨 | §8.4.2 |
| $\lambda$ 采样族 | 同一 score 配不同随机强度 $\lambda$，共享边际(DDPM↔DDIM) | 骨 | §8.4.2 |
| **DDPM ELBO 分解** | 似然下界拆成高斯间 KL 之和($L_N+\sum L_{t-1}+L_0$) | 完 | §8.4.3 |
| **$\varepsilon$-预测等价** | 均值预测代换 $x_0$ 后变 $\|\varepsilon-\varepsilon_\theta\|^2$，尺度恒定更稳 | 完 | §8.4.3 |
| DDIM = PF-ODE 离散 | $\sigma_t=0$ 的 DDIM 是概率流 ODE 的一阶 Euler，故能少步 | 骨 | §8.4.3 |
| **Hyvärinen 显式 SM** | 分部积分把含未知 $\nabla\log p$ 的目标变成 $\tfrac12\|s\|^2+\nabla\cdot s$ | 完 | §8.4.4 |
| **Vincent 去噪 SM** | DSM = 显式 SM(on $p_\sigma$)，用条件 score $-\varepsilon/\sigma$ 当标签 | 完 | §8.4.4 |
| **Tweedie 公式** | $\mathbb E[x\mid\tilde x]=\tilde x+\sigma^2\nabla\log p_\sigma$，统一 $\varepsilon$/score/$x_0$/v 四坐标 | 完 | §8.4.4 |
| **Chen et al. 2023 TV 界** | 最小假设(无 LSI)下三源误差 TV 收敛，$N=\tilde O(L^2d/\varepsilon^2)$ | 骨 | §8.4.5 |
| Benton et al. 2024 KL 界 | 随机局部化把维度依赖改进到近线性 $\tilde O(d/\varepsilon^2)$ | 陈 | §8.4.5 |
| **Lipman CFM 等价** | 边际速度场 = 条件速度场后验期望；CFM 与 FM 同梯度 | 完 | §8.4.6 |
| Rectified Flow 不增代价 | reflow 不增凸传输代价、拉直轨迹，逼近单步生成 | 骨 | §8.4.6 |
| Stochastic Interpolants | $x_t=\alpha(t)x_1+\sigma(t)x_0$ 统一 FM(直线)与扩散(圆弧) | 陈 | §8.4.6 |
| **JKO 定理(1998)** | Fokker-Planck = 自由能在 $W_2$ 度量下的梯度流 | 骨 | §8.4.7 |
| **Girsanov 定理** | 只差漂移的两 SDE，路径 KL $=\tfrac12\mathbb E\int\|b_1-b_2\|^2/g^2\,\mathrm dt$ | 骨 | §8.4.7 |
| MPPI = 单步去噪 | softmax 加权平均 = Tweedie 后验均值；退火 = 反向扩散 | 骨 | §8.4.8 |
| Classifier-Free Guidance | $\tilde\varepsilon=(1+w)\varepsilon_\theta(\cdot,c)-w\varepsilon_\theta(\cdot,\varnothing)$，无分类器放大条件 | 完 | §8.4.8 |

### 知识点总表

| # | 知识点 | 核心要点 | 对应节 | 难度 |
|---:|-------|---------|--------|:---:|
| 1 | 前向扩散 SDE | 数据按线性 SDE 溶解成噪声；VP(方差保持)/VE(方差爆炸)两典范；转移核闭式高斯 | §8.4.1 | ⭐⭐ |
| 2 | Anderson 反向 SDE | 反向也是 SDE，漂移多 $-g^2\nabla\log p_t$；生成 = 估计 score(全章枢纽) | §8.4.2 | ⭐⭐⭐ |
| 3 | 概率流 ODE 与 $\lambda$ 族 | 确定性 ODE 与随机 SDE 共享边际；一个 score 给出整族采样器 | §8.4.2 | ⭐⭐⭐ |
| 4 | DDPM ELBO | 离散隐变量模型，似然下界拆成高斯间 KL 之和 | §8.4.3 | ⭐⭐⭐ |
| 5 | $\varepsilon$-预测与 $L_\text{simple}$ | 预测噪声而非均值/数据，尺度恒定、高噪声不退化 | §8.4.3 | ⭐⭐⭐ |
| 6 | DDIM | 非马氏确定采样，= PF-ODE 离散，10–50 步 | §8.4.3 | ⭐⭐⭐ |
| 7 | 显式/切片 Score Matching | 分部积分消去真值 score，代价是散度(Hessian 迹) | §8.4.4 | ⭐⭐⭐⭐ |
| 8 | 去噪 Score Matching | 去噪 = score 估计；DDPM 损失的数学心脏 | §8.4.4 | ⭐⭐⭐⭐ |
| 9 | Tweedie 公式 | 后验均值由 score 给出；四种参数化统一 | §8.4.4 | ⭐⭐⭐⭐ |
| 10 | 收敛三源误差 | 初始化/score/离散化三旋钮正交；Girsanov 证明骨架 | §8.4.5 | ⭐⭐⭐⭐ |
| 11 | 维度依赖($d^2$→$d$) | 阶数取决于误差累积用多粗的不等式；扩散不维度诅咒 | §8.4.5 | ⭐⭐⭐⭐ |
| 12 | Flow Matching / CFM | 直接学速度场，条件路径可算，直线路径少步 | §8.4.6 | ⭐⭐⭐ |
| 13 | Rectified Flow / SI | reflow 拉直路径逼近单步；插值系数统一 FM 与扩散 | §8.4.6 | ⭐⭐⭐ |
| 14 | Wasserstein 几何与 JKO | 分布演化 = $\mathcal P_2$ 测地流；前向 = 自由能梯度流(熵增) | §8.4.7 | ⭐⭐⭐⭐ |
| 15 | Girsanov 定理 | 路径测度 KL = 漂移差二次型积分；收敛证明引擎 | §8.4.7 | ⭐⭐⭐⭐ |
| 16 | Diffusion Policy | 动作序列扩散，多峰、滚动时域；DDPM 损失原样搬 | §8.4.8 | ⭐⭐⭐ |
| 17 | $\pi_0$ Flow Matching | VLM 条件 + FM 动作头，直线路径换 50 Hz 实时 | §8.4.8 | ⭐⭐⭐ |
| 18 | Diffuser / guidance | 规划 = 去噪；classifier guidance = 可行性先验 $\times$ 目标似然 | §8.4.8 | ⭐⭐⭐ |
| 19 | SE(3) 扩散 | 李代数加噪 + 指数映射；扩散 $\times$ 几何 | §8.4.8 | ⭐⭐⭐ |
| 20 | MPPI = 单步去噪 | softmax 平均 = Tweedie；退火 = 反向扩散(打通 04 专题) | §8.4.8 | ⭐⭐⭐ |

---

## 累积项目：本章新增模块

> **项目主线(数学方向贯穿项目)**：本项目的数学方向以"从零搭一套可运行的机器人生成式策略数学工具箱"为累积目标。前面专题已搭好随机分析(9.1 的 Itô 积分/Fokker-Planck 模块)、深度学习数学(8.1 逼近论、8.3 优化)。本专题贡献"扩散/流匹配核心模块"，为后续 VLA 专题(8.5)和采样式 MPC(04 章)直接复用。

**本章新增模块：`diffusion_core`(纯数学验证，非生产代码)。** 理论教学的代码只做数值验证(占比 <15%)，不承担讲解。本模块包含三个可手算或几十行脚本验证的子件，每个都对应本章一个"必完证"定理，目的是让你**用数值实验确认推导没错**:

| 子模块 | 验证什么 | 对应节 | 验证方式 |
|--------|---------|--------|---------|
| `verify_transition_kernel` | VP 转移核 $x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\varepsilon$ 的均值/方差 | §8.4.1 | 蒙特卡洛模拟前向 SDE(Euler-Maruyama 细步)，统计 $\mathbb E[x_t]$、$\mathrm{Var}[x_t]$，对照闭式 $x_0 e^{-\bar\beta/2}$、$1-e^{-\bar\beta}$ |
| `verify_tweedie` | Tweedie $\mathbb E[x\mid\tilde x]=\tilde x+\sigma^2\nabla\log p_\sigma$ | §8.4.4 | 取一个已知混合高斯 $p$(score 可解析)，加噪后数值算后验均值，对照 Tweedie 右端 |
| `verify_dsm_equals_esm` | DSM 与 ESM(on $p_\sigma$)同最优解 | §8.4.4 | 在 1D 混合高斯上，分别用两目标拟合一个小 MLP score 网络，对照二者收敛到同一 $\nabla\log p_\sigma$ |

**本章给项目加的能力**：学完本章，你的工具箱具备了"把任意低维数据分布(如一批 2D 多峰点、一批机器人动作样本)训练成扩散/FM 采样器"的数学完备性——你知道前向怎么设计(§8.4.1)、损失怎么写(§8.4.3/8.4.4/8.4.6)、采样怎么积分(§8.4.2)、误差怎么估(§8.4.5)。后续 8.5 VLA 专题会在此基础上加"条件注入(视觉/语言)"模块，04 采样式 MPC 会复用"Tweedie 单步去噪"模块。

**建议的累积验证练习(贯穿前后章)**：用本章的 `diffusion_core` 在一个 **2D 双月牙(two-moons)分布**上训练一个最小扩散采样器，确认它能采出双月牙(而高斯拟合只会得到一个糊成一团的椭圆)。保留这个脚本——8.5 专题会让你把"双月牙"换成"给定图像条件的动作分布"，验证条件扩散；04 专题会让你把同一 score 网络的 Tweedie 去噪嵌进 MPPI 循环。这条线让你亲眼见证"同一套扩散数学，换个数据/换个条件/换个采样器，就横跨生成、策略、规控三个场景"。

---

## 延伸阅读

按"原始论文 / 理论深化 / 机器人应用 / 教材综述"四类整理，标注难度(⭐ 入门 ~ ⭐⭐⭐⭐ 研究级)。

**原始论文(必读，按本章脉络)**

- ⭐⭐ Ho, Jain, Abbeel. *Denoising Diffusion Probabilistic Models*. NeurIPS 2020. arXiv:2006.11239. — DDPM 原作，§8.4.3 的来源，工程实现的起点。
- ⭐⭐⭐ Song, Sohl-Dickstein, Kingma, Kumar, Ermon, Poole. *Score-Based Generative Modeling through Stochastic Differential Equations*. ICLR 2021(Outstanding Paper). arXiv:2011.13456. — Score SDE,§8.4.1/8.4.2 的 SDE 统一框架与概率流 ODE。
- ⭐⭐⭐ Song, Meng, Ermon. *Denoising Diffusion Implicit Models*. ICLR 2021. arXiv:2010.02502. — DDIM,§8.4.3 的少步确定采样。
- ⭐⭐ Vincent. *A Connection Between Score Matching and Denoising Autoencoders*. Neural Computation 2011. — §8.4.4 去噪 SM 定理的原始证明(DDPM 的数学心脏)。
- ⭐⭐⭐⭐ Hyvärinen. *Estimation of Non-Normalized Statistical Models by Score Matching*. JMLR 2005. — 显式 score matching 与分部积分技巧的鼻祖。
- ⭐⭐⭐ Lipman, Chen, Ben-Hamu, Nickel, Le. *Flow Matching for Generative Modeling*. ICLR 2023. arXiv:2210.02747. — §8.4.6 的 CFM 等价恒等式来源。
- ⭐⭐⭐ Liu, Gong, Liu. *Flow Straight and Fast: Learning to Generate and Transfer Data with Rectified Flow*. ICLR 2023(Oral). arXiv:2209.03003. — Rectified Flow 与 reflow。
- ⭐⭐⭐⭐ Albergo, Vanden-Eijnden. *Stochastic Interpolants: A Unifying Framework for Flows and Diffusions*. JMLR 2025(arXiv:2303.08797, 2023). — §8.4.6 统一 FM 与扩散的理论。

**理论深化**

- ⭐⭐ Anderson. *Reverse-Time Diffusion Equation Models*. Stochastic Processes and their Applications, 1982. — §8.4.2 反向 SDE 定理的源头(沉睡 38 年的理论)。
- ⭐⭐⭐⭐ Chen, Chewi, Li, Li, Salim, Zhang. *Sampling is as Easy as Learning the Score*. ICLR 2023. arXiv:2209.11215. — §8.4.5 最小假设 TV 收敛(无 LSI)。
- ⭐⭐⭐⭐ Benton, De Bortoli, Doucet, Deligiannidis. *Nearly $d$-Linear Convergence Bounds for Diffusion Models via Stochastic Localization*. ICLR 2024(Spotlight). arXiv:2308.03686. — §8.4.5 近线性维度依赖。
- ⭐⭐⭐⭐ Jordan, Kinderlehrer, Otto. *The Variational Formulation of the Fokker-Planck Equation*. SIAM J. Math. Anal., 1998. — §8.4.7 JKO 定理(Fokker-Planck = $W_2$ 梯度流)的奠基作。
- ⭐⭐⭐ Kingma, Salimans, Poole, Ho. *Variational Diffusion Models*. NeurIPS 2021. arXiv:2107.00630. — §8.4.1 信噪比统一调度、§8.4.3 权重不影响最优解。
- ⭐⭐⭐ Karras, Aittala, Aila, Laine. *Elucidating the Design Space of Diffusion-Based Generative Models (EDM)*. NeurIPS 2022. arXiv:2206.00364. — §8.4.1 VP/VE 统一与预处理，采样器设计实战圣经。

**机器人应用**

- ⭐⭐⭐ Chi, Xu, Feng, Cousineau, Du, Burchfiel, Tedrake, Song. *Diffusion Policy: Visuomotor Policy Learning via Action Diffusion*. RSS 2023 / IJRR 2024. arXiv:2303.04137. — §8.4.8 Diffusion Policy。
- ⭐⭐⭐ Black 等(Physical Intelligence). *$\pi_0$: A Vision-Language-Action Flow Model for General Robot Control*. 2024. arXiv:2410.24164. — §8.4.8 $\pi_0$,Flow Matching 实时 VLA。
- ⭐⭐⭐ Janner, Du, Tenenbaum, Levine. *Planning with Diffusion for Flexible Behavior Synthesis*. ICML 2022(Long Talk). arXiv:2205.09991. — §8.4.8 Diffuser，规划即去噪 + classifier guidance。
- ⭐⭐⭐⭐ Urain, Funk, Peters, Chalvatzaki. *SE(3)-DiffusionFields: Learning Smooth Cost Functions for Joint Grasp and Motion Optimization through Diffusion*. ICRA 2023. arXiv:2209.03855. — §8.4.8 流形扩散与 6-DoF 抓取。
- ⭐⭐⭐ Pan, Yi, Shi, Qu. *Model-Based Diffusion for Trajectory Optimization*. NeurIPS 2024. arXiv:2407.01573. — §8.4.8 MPPI 桥接，无数据扩散式轨迹优化。
- ⭐⭐⭐ Ho, Salimans. *Classifier-Free Diffusion Guidance*. NeurIPS 2021 Workshop / arXiv:2207.12598 (2022). — §8.4.8 应用七，现代条件扩散(Diffusion Policy/$\pi_0$)的主流引导方式。

**教材 / 综述 / 优质讲义**

- ⭐⭐ Yang Song. *Generative Modeling by Estimating Gradients of the Data Distribution*(博客). — score-based 生成的最佳入门讲解，配 §8.4.2/8.4.4。
- ⭐⭐⭐ Lilian Weng. *What are Diffusion Models?*(博客). — DDPM/DDIM/score SDE 的公式推导汇编，适合配 §8.4.3 速查。
- ⭐⭐⭐⭐ Chen, Lu 等关于扩散收敛理论的综述(arXiv 持续更新)— §8.4.5 的进阶阅读。
- ⭐⭐⭐ Lipman 等。 *Flow Matching Guide and Code*(2024 讲义/代码). — §8.4.6 的系统化教程，含可运行实现。

---

## 本章与后续章节的关系

本专题是"扩散/流匹配数学"的中枢，下游多个专题直接复用本章成果。下表说明每个后续专题如何依赖本章、本章哪个知识点为其铺垫。

| 后续章节 | 与本章的关系 | 本章哪个知识点为其铺垫 |
|---------|------------|---------------------|
| `01_数学/90_深度学习数学/50_具身智能VLA框架.md`(8.5) | VLA 动作头普遍用扩散/FM，本章提供其全部生成数学；8.5 在此基础上加"视觉-语言条件注入"与大模型骨干 | §8.4.3 $\varepsilon$-预测、§8.4.6 Flow Matching、§8.4.8 $\pi_0$/Diffusion Policy |
| `04_移动机器人规控/20_采样式MPC/60_扩散启发采样MPC.md` | 该专题核心论点"MPPI = 单步去噪、退火 = 反向扩散"由本章 §8.4.8 提供完整数学根据；Model-Based Diffusion 等方法的原理在此 | §8.4.4 Tweedie、§8.4.2 反向 SDE、§8.4.8 MPPI 桥接 |
| `04_移动机器人规控` 的轨迹优化/规划专题 | Diffuser 把规划表述为去噪 + classifier guidance，提供"生成式规划"视角与传统优化互补 | §8.4.8 Diffuser、§8.4.2 score=值函数梯度类比 |
| 操纵/抓取专题(6-DoF 抓取生成) | SE(3) 扩散在抓取位姿生成上的应用，需本章流形扩散 + 李群专题联合 | §8.4.8 SE(3)-DiffusionFields、§8.4.1 前向加噪 |
| 强化学习专题(若涉及扩散策略/Q-learning 引导) | 扩散策略、diffusion Q-learning 等把本章生成模型与 RL 结合 | §8.4.8 Diffusion Policy、§8.4.8 classifier guidance |
| 李群/微分几何专题(反向桥接) | 本章 SE(3) 扩散反过来为李群专题提供"在流形上做生成"的应用实例 | §8.4.8 流形扩散 |

**前置依赖回顾(本章依赖谁)**：本章重度依赖 `01_数学/95_随机分析/10_随机微分方程基础.md`(9.1,Itô 积分/公式、Fokker-Planck、OU 过程)、`01_数学/90_深度学习数学/` 的逼近论与优化专题(8.1/8.3，神经网络逼近 score 的能力)、以及李群专题(SE(3) 扩散用)。若这些前置陌生，务必先补——本章 §8.4.2 起的推导全部建立在它们之上。

---

## 🔧 故障排查手册

理论专题的"故障"既包括推导/数学层面的常见出错(概念误用导致结论错)，也包括把理论落到机器人/数值实现时反复出现的症状。下面给出 7 个高频场景的结构化排查表(症状 → 可能原因 → 排查步骤 → 相关章节)，覆盖任务点名的采样发散、方差爆炸、低维动作过拟合、等变性破坏、步数-质量权衡，并补充另外两个高频坑。

### 场景 1：反向采样发散(样本变成 NaN 或越来越大)

| 项 | 内容 |
|----|------|
| **症状** | 反向采样跑到中后期，$x_t$ 数值爆炸成 $\infty$/NaN，或样本远离数据范围(范数越走越大)。 |
| **可能原因** | (a) score 网络在低密度区/高噪声小 $t$ 区估计极不准，被 $-g^2\nabla\log p_t$ 放大；(b) 离散步长过大(尤其 SDE),Euler-Maruyama 在 score 陡峭处超调；(c) 错误地用了正号 score(漂移符号写反);(d) 时间方向搞反(从 $t=0$ 而非 $T$ 起步)。 |
| **排查步骤** | 1. 先排除符号：核对反向漂移是 $f-g^2 s_\theta$(§8.4.2),score 把过程往高密度拉，符号错会把它往外推→必发散。2. 把步数翻倍(或用更小步长)，若发散消失→是离散化(E3);3. 检查 score 在大 $\|x\|$ 处的输出范数是否异常大(应随 $\|x\|$ 增长但有界);4. 改用 ODE 确定采样(§8.4.2)，若稳定→说明是 SDE 随机性 + 大步长共振，需减步长或用 churn 控制。 |
| **相关章节** | §8.4.2(反向 SDE 符号/方向)、§8.4.5(离散化误差 E3)、§8.4.3(DDIM/步长) |

### 场景 2：方差爆炸 / 信号被淹没得太快或不够

| 项 | 内容 |
|----|------|
| **症状** | (VE 路线)训练/采样时数值范围跨好几个数量级，网络难处理；或前向到 $T$ 时 $p_T$ 仍明显带数据信息(没混合充分)，反向起点失配。 |
| **可能原因** | (a) 用了 VE-SDE 但没做 EDM 式预处理(preconditioning)，网络输入未归一化；(b) 噪声调度 $\bar\beta(T)$ 取太小，$e^{-\bar\beta(T)/2}$ 不够小，$p_T\ne\mathcal N(0,I)$;(c) 把 VP 的经验(有界方差)直接套到 VE。 |
| **排查步骤** | 1. 算 $e^{-\bar\beta(T)/2}$ 是否 $<10^{-3}$(§8.4.1 反事实检验)，不够则增大 $\bar\beta(T)$ 或 $T$;2. VE 路线务必加输入/输出缩放(EDM 预处理)，让网络在所有噪声水平看到归一化量；3. 低维结构化数据(机器人动作)优先选 VP(有界方差更稳，§8.4.1 陷阱);4. 用 `verify_transition_kernel`(累积项目)确认实际方差与闭式一致，排除调度实现 bug。 |
| **相关章节** | §8.4.1(VP/VE、SNR、$p_T$ 残差)、延伸阅读 EDM |

### 场景 3：低维动作上过拟合 / 多样性崩塌(机器人最常见)

| 项 | 内容 |
|----|------|
| **症状** | Diffusion Policy 在小数据集(几十条演示)上训练后，采样动作几乎不变(多样性消失)，或对训练轨迹过拟合、泛化差。 |
| **可能原因** | (a) 动作维度低($d\sim10^2$)+ 数据少 → 模型容量相对过剩，记住了训练样本；(b) 噪声调度让低噪声步权重过高，网络专攻"精修"而没学到分布的"骨架";(c) score 在数据稀疏区无约束(§8.4.5 不可约误差)，低维下尤其暴露；(d) 把图像扩散的大网络/长训练直接搬到低维动作。 |
| **排查步骤** | 1. 缩小网络、加正则/EMA，低维任务不需要图像级容量(§8.4.8"动作维度低让扩散更便宜"的反面：容量也要按维度缩);2. 检查多峰是否保留——在一个已知多峰任务上看采样是否覆盖多个峰(若塌成一峰→过拟合或调度问题);3. 调整噪声采样权重，让中/高噪声步(分布骨架)获得足够训练(§8.4.3 $L_\text{simple}$ 重加权);4. 用 DDPM 随机采样而非 DDIM，随机性能恢复部分多样性(§8.4.2)。 |
| **相关章节** | §8.4.8(Diffusion Policy)、§8.4.5(不可约 score 误差)、§8.4.3(调度权重) |

### 场景 4：等变性 / 流形结构被破坏(SE(3)/SO(3) 抓取)

| 项 | 内容 |
|----|------|
| **症状** | 生成的抓取位姿不是合法的 $SE(3)$ 元素(旋转矩阵非正交、行列式 $\ne1$)，或对输入做一个全局旋转后输出不随之等变旋转(本该等变却乱了)。 |
| **可能原因** | (a) 直接对欧拉角/四元数/旋转矩阵的 9 个数做欧氏 VP 加噪 → 跌出 $SO(3)$ 流形(§8.4.8 陷阱);(b) score 没定义在李代数/切空间，而在嵌入的欧氏坐标里；(c) 网络架构本身不等变(用了破坏对称的全连接)，即使流形处理对了也无等变性。 |
| **排查步骤** | 1. 确认加噪在李代数 $\mathfrak{se}(3)/\mathfrak{so}(3)$ 做、用指数映射搬回(§8.4.8 应用四);2. 反向更新用 retraction(指数映射)而非欧氏加法，每步后投影回流形(或本就用群运算保证不跌出);3. 单独测等变性：对输入施加群作用 $g$，检查输出是否变成 $g\cdot$ 原输出(等变定义)；若流形对了但等变错→是网络架构问题，需用等变层(回顾几何深度学习/李群专题);4. 用李群专题的 hat/vee、$\exp/\log$ 单元测试确认流形运算实现无误。 |
| **相关章节** | §8.4.8(SE(3) 扩散)、李群专题($\mathfrak{se}(3)$、指数映射、retraction) |

### 场景 5：步数-质量权衡失衡(减步就崩 / 多步也不好)

| 项 | 内容 |
|----|------|
| **症状** | 想减少采样步数提速，但一减步样本质量骤降(段接合处跳变、伪影)；或步数已经很多但质量仍上不去。 |
| **可能原因** | (a) 用 DDPM 随机采样还想少步——随机轨迹粗糙，大步长误差大(§8.4.3);(b) 路径弯曲(VP 圆弧)用一阶 Euler 少步切弯道(§8.4.6);(c) 多步也不好 → 瓶颈在 score 估计(E2)而非离散化(E3)，加步数(降 E3)对 E2 主导的误差无效。 |
| **排查步骤** | 1. 先用三源误差(§8.4.5)定位瓶颈：固定 score 网络，只增步数，质量饱和不再升→瓶颈是 E2(score)，该去训练而非加步；质量随步数持续升→瓶颈是 E3(离散)，可换更好求解器；2. 要少步：换 ODE 确定采样 + 高阶求解器(DPM-Solver)，或换 Flow Matching 直线路径(§8.4.6)甚至 Rectified Flow reflow(逼近单步);3. 不要靠调大 $\beta_t$ 来减步(破坏反向高斯假设，§8.4.3 陷阱);4. 实时控制(如 $\pi_0$)直接选 FM 直线路径，从架构上解决(§8.4.8)。 |
| **相关章节** | §8.4.5(三源误差定位)、§8.4.6(直线路径/Rectified Flow)、§8.4.2(SDE vs ODE) |

### 场景 6：训练损失正常下降但采样质量差(损失-质量脱节)

| 项 | 内容 |
|----|------|
| **症状** | $\|\varepsilon-\varepsilon_\theta\|^2$ 损失降得很好，但采样出来的样本/动作质量差。 |
| **可能原因** | (a) 损失被低噪声步主导(那里好学、loss 小)，高噪声步(决定分布骨架)欠拟合——平均损失漂亮但关键区域差；(b) 训练/采样的噪声调度不一致(实现 bug);(c) 条件注入有误(观测 $o$ 没真正影响 $\varepsilon_\theta$)，退化成无条件生成。 |
| **排查步骤** | 1. 按 $t$ 分桶看损失：若高噪声 $t$ 的损失明显偏大→重加权强调高噪声步(§8.4.3);2. 核对训练与采样用同一 $\bar\alpha_t$ 表(最常见的低级 bug);3. 测条件有效性：固定噪声、换不同观测 $o$，看输出是否随之变化，不变→条件通路断了；4. 用 Tweedie(§8.4.4)从 $\varepsilon_\theta$ 反推 $\hat x_0$ 可视化，直接看网络"以为的干净样本"对不对。 |
| **相关章节** | §8.4.3(调度/重加权)、§8.4.4(Tweedie 可视化诊断)、§8.4.8(条件注入) |

### 场景 7：把 score 误当成对参数的梯度 / 配分函数误用(纯数学陷阱)

| 项 | 内容 |
|----|------|
| **症状** | 推导卡壳或得出矛盾结论：比如试图"对 $\theta$ 求 $\nabla\log p$"、或以为训练绕开了 $Z$ 就能直接算似然。 |
| **可能原因** | (a) 把 $\nabla_x\log p$(数据梯度，扩散 score)与 $\nabla_\theta\log p_\theta$(Fisher score)混淆(§8.4.2 陷阱);(b) 以为 score matching 消去配分函数 $Z$ 后就能算密度(其实只是训练不需 $Z$，算似然要 ODE 散度积分，§8.4.4 陷阱)。 |
| **排查步骤** | 1. 凡见 $\nabla\log p$，先确认对谁求导：扩散里**永远对 $x$**(向量场)，不是对 $\theta$;2. 区分"训练可行"(score 避开 $Z$)与"能算似然"(需概率流 ODE 的瞬时变量替换，§8.4.2 表);3. 若要密度估计，走 ODE 似然路径，别指望 score 直接给归一化密度。 |
| **相关章节** | §8.4.2(score 定义)、§8.4.4(配分函数与似然)、§8.4.2(ODE 似然) |

---

## 数值算例：单步去噪 $\leftrightarrow$ score $\leftrightarrow$ 后验均值(手算一遍)

前面 §8.4.3 与 §8.4.4 反复强调"$\varepsilon$、score、$\hat x_0$、后验均值是同一信息的不同坐标，差一个只依赖 $\bar\alpha_t$ 的确定线性变换"。这个结论抽象，初学时容易当口号略过。本节用一组**具体数字**把它兑现：给定一个噪声样本和网络的一次 $\varepsilon$ 输出，我们手算出 $\hat x_0$、score、DDPM 后验均值，并用**两条独立路径交叉验证**——亲眼看到它们必然一致，正是理解"同一个量的不同坐标"的最快方式。建议你先遮住答案自己算，再对照。

为便于笔算，取一维标量、整洁的完全平方数。给定量：

$$
\bar\alpha_t=0.36,\quad \bar\alpha_{t-1}=0.49,\quad x_t=1.2,\quad \varepsilon_\theta(x_t,t)=0.5 .
$$

由此预先算出本节要反复用的派生量(都用 §8.4.1 的关系 $\alpha_t=\bar\alpha_t/\bar\alpha_{t-1}$、$\beta_t=1-\alpha_t$):

$$
\sqrt{\bar\alpha_t}=0.6,\quad 1-\bar\alpha_t=0.64,\quad \sqrt{1-\bar\alpha_t}=0.8,\quad
\alpha_t=\tfrac{0.36}{0.49}\approx0.7347,\quad \beta_t\approx0.2653,\quad \sqrt{\alpha_t}\approx0.8571,\quad 1-\bar\alpha_{t-1}=0.51 .
$$

**第 1 步：从 $\varepsilon$ 预测反推干净数据估计 $\hat x_0$。** 用 §8.4.3 的去噪公式 $\hat x_0=\frac{1}{\sqrt{\bar\alpha_t}}\big(x_t-\sqrt{1-\bar\alpha_t}\,\varepsilon_\theta\big)$:

$$
\hat x_0=\frac{1.2-0.8\times0.5}{0.6}=\frac{0.8}{0.6}=\frac{4}{3}\approx1.3333 .
$$

直觉检验：网络认为 $x_t=1.2$ 里混入了正噪声($\varepsilon_\theta=0.5>0$)，所以去噪后的"干净值"$1.333$ 比把 $x_t$ 直接除以信号缩放 $\sqrt{\bar\alpha_t}$ 得到的 $2.0$ 更靠近原点——噪声被正确扣除了。

**第 2 步：算 score，并用两条路径交叉验证。** §8.4.4 的 Tweedie 关系（正文 boxed 公式）给出 score 的两种等价写法：

$$
s_\theta=-\frac{\varepsilon_\theta}{\sqrt{1-\bar\alpha_t}}
\qquad\text{以及}\qquad
s_\theta=\frac{\sqrt{\bar\alpha_t}\,\hat x_0-x_t}{1-\bar\alpha_t}.
$$

- 路径 A（由 $\varepsilon$）：$s_\theta=-\dfrac{0.5}{0.8}=-0.625$。
- 路径 B（由 $\hat x_0$）：$s_\theta=\dfrac{0.6\times1.3333-1.2}{0.64}=\dfrac{0.8-1.2}{0.64}=\dfrac{-0.4}{0.64}=-0.625$。

两者**严格相等**——这不是巧合，而是 §8.4.4 那条线性变换的必然结果。score 为负，方向把 $x_t$ 往 $\hat x_0$（更靠近高密度区）拉，符合"score 指向对数概率上升方向"。

**第 3 步：算 DDPM 后验均值 $\tilde\mu_t$，再用两条路径交叉验证。** §8.4.3 给了两种等价形式。先用 $x_0$ 形式（代入 $\hat x_0$ 作为模型对 $x_0$ 的估计）:

$$
\tilde\mu_t=\underbrace{\frac{\sqrt{\bar\alpha_{t-1}}\,\beta_t}{1-\bar\alpha_t}}_{c_0}\,\hat x_0+\underbrace{\frac{\sqrt{\alpha_t}\,(1-\bar\alpha_{t-1})}{1-\bar\alpha_t}}_{c_x}\,x_t,
\quad
c_0=\frac{0.7\times0.2653}{0.64}\approx0.2902,\;\;
c_x=\frac{0.8571\times0.51}{0.64}\approx0.6830 .
$$

$$
\tilde\mu_t\approx0.2902\times1.3333+0.6830\times1.2\approx0.3869+0.8196\approx1.2065 .
$$

再用 §8.4.3 化简后的 $\varepsilon$ 形式 $\tilde\mu_t=\frac{1}{\sqrt{\alpha_t}}\big(x_t-\frac{\beta_t}{\sqrt{1-\bar\alpha_t}}\,\varepsilon_\theta\big)$ 独立复算：

$$
\tilde\mu_t=\frac{1}{0.8571}\Big(1.2-\frac{0.2653}{0.8}\times0.5\Big)=\frac{1}{0.8571}\big(1.2-0.1658\big)=\frac{1.0342}{0.8571}\approx1.2065 .
$$

两条路径给出**同一个** $\tilde\mu_t\approx1.2065$——这正面兑现了 §8.4.3 "把 $x_0$ 代换成 $x_t,\varepsilon$ 后两式恒等"的代数化简。后验方差为 $\tilde\beta_t=\frac{1-\bar\alpha_{t-1}}{1-\bar\alpha_t}\beta_t=\frac{0.51}{0.64}\times0.2653\approx0.2114$（标准差 $\approx0.4598$），所以 DDPM 的下一步采样为 $x_{t-1}=\tilde\mu_t+\sqrt{\tilde\beta_t}\,z\approx1.2065+0.4598\,z$（$z\sim\mathcal N(0,1)$）。

**这个算例教会你三件事**（呼应 §8.4.4 的本质洞察）:

1. **三种坐标确实是同一个量**：$\varepsilon_\theta=0.5$、$\hat x_0=1.333$、$s_\theta=-0.625$ 三个数字彼此由 $\bar\alpha_t$ 唯一确定，知道任一个就能算出另两个——网络"预测哪一个"在信息上完全等价。
2. **后验均值靠近 $x_t$ 而非 $\hat x_0$**：注意 $\tilde\mu_t\approx1.21$ 离 $x_t=1.2$ 很近，离 $\hat x_0=1.33$ 较远（系数 $c_x\approx0.68>c_0\approx0.29$）。这是去噪"小步走"的体现——单步只朝 $\hat x_0$ 挪一点点，靠几十上千步累积才到达干净样本，对应 §8.4.4 "单步 Tweedie 只在低噪声区接近真实样本"的告诫。
3. **调试抓手**：把这条手算链固化进代码断言（给定 $\bar\alpha_t,x_t,\varepsilon_\theta$，两路算 score 与 $\tilde\mu_t$ 必须一致），就是 §8.4.4 练习与故障排查场景 6 中"用 Tweedie 反推 $\hat x_0$ 做诊断"的最小可执行版本——一旦两路不等，立刻锁定是 $\bar\alpha_t$ 表或系数实现写错了。

> **练习（把数字换一组）**：取 $\bar\alpha_t=0.25$、$x_t=2.0$、$\varepsilon_\theta=-1.0$，自己重走三步。验证 $\hat x_0=\frac{2.0-\sqrt{0.75}\times(-1.0)}{0.5}$，并确认两路 score 与（取你自定的 $\bar\alpha_{t-1}$ 后）两路 $\tilde\mu_t$ 仍各自一致。若你能不查公式独立完成，说明 §8.4.3–8.4.4 的坐标变换已真正内化。

---

## 研究实践建议

**给新手(第一次学扩散/要复现 Diffusion Policy)**:

- **先走主干路径**:§8.4.1 → 8.4.2 → 8.4.3 → 8.4.4 → 8.4.6 → 8.4.8，跳过 §8.4.5/8.4.7 的证明。目标是先打通"训练什么($\varepsilon$/score/velocity)、怎么采样(SDE/ODE)、机器人怎么用"，建立可操作的整体图景，再回头补理论顶板。
- **亲手算三个"必完证"**:DDPM ELBO→$\varepsilon$(§8.4.3)、Vincent 去噪 SM(§8.4.4)、Lipman CFM 等价(§8.4.6)。这三个证明是"条件化技巧"的三次变奏，算通一个，另两个会豁然开朗——它们是你读任何扩散论文的解码钥匙。
- **用最小数值实验锚定直觉**：跑累积项目的 2D 双月牙采样器(几十行)。亲眼看到"高斯拟合糊成一团 vs 扩散采出两个月牙"，比读十遍"扩散能多峰"都管用。
- **从 $\varepsilon$-预测入手，别一上来碰 score/velocity 互转**：工程上 $\varepsilon$-预测最稳、最主流(§8.4.3)，先把它用熟，再用 Tweedie(§8.4.4)理解它和 score/velocity 的换算。
- **机器人方向先做低维**：在 7-DoF 动作上做 Diffusion Policy，比图像扩散便宜得多(§8.4.8)，迭代快、容易调通，适合建立信心。

**给有经验者(要做扩散相关研究/改进方法)**:

- **必补理论顶板**:§8.4.5(Girsanov + 三源误差)和 §8.4.7(JKO/$W_2$ 几何)是当前理论论文的通用语言。不掌握它们，你能用扩散但读不懂收敛/采样器设计的前沿工作，也写不出有理论贡献的论文。
- **把"四重视角统一表"(§8.4.7)当工作记忆**：读任何新方法，先归位——它在 SDE/ODE/score/velocity/测度 哪一列？用什么路径(线性/圆弧/其他)？这能让你 5 分钟看穿一篇"新"扩散论文的本质是不是旧框架换坐标。
- **维度依赖意识**：做高维(长 horizon $\times$ 多自由度)机器人规划时，记住 §8.4.5 的"维度依赖阶数取决于误差分析的精细度"。这指导你判断一个方法在你的维度下是否会爆炸，以及该往哪改(更精细的误差累积处理)。
- **路径设计是杠杆**：要实时(机器人控制)就从路径直度下手(FM/Rectified Flow,§8.4.6)，而非盲目换框架名。"步数预算"问题的真正旋钮是路径几何 + 求解器阶数。
- **跨领域迁移**：认清 MPPI = 单步去噪(§8.4.8)后，主动把图像扩散十年的红利(噪声调度、采样器、引导技巧)迁移到采样式 MPC/轨迹优化——这是当前一个出论文的富矿(Model-Based Diffusion、DIAL-MPC 等)。
- **流形扩散是蓝海**:SE(3)/SO(3) 上的等变扩散(§8.4.8)在抓取、装配、多体位姿生成上仍有大量未解问题，且需要扩散 $\times$ 李群 $\times$ 几何深度学习的交叉功底——壁垒高、空间大。

---

## 版本信息速查

本专题为纯理论数学专题，不绑定特定软件版本；下表汇总正文涉及的关键文献年份/会议与(若有)代表实现，便于核对与溯源。

| 主题 | 关键文献 | 年份 / Venue | 代表实现(参考) |
|------|---------|-------------|----------------|
| 反向时间 SDE | Anderson | 1982 / Stoch. Proc. Appl. | —— |
| 时间反演严格条件 | Haussmann & Pardoux | 1986 / Ann. Probab. | —— |
| 显式 Score Matching | Hyvärinen | 2005 / JMLR | —— |
| 去噪 Score Matching | Vincent | 2011 / Neural Computation | —— |
| 非平衡热力学扩散 | Sohl-Dickstein 等 | 2015 / ICML | —— |
| NCSN(VE 前身) | Song & Ermon | 2019 / NeurIPS | `ermongroup/ncsn` |
| DDPM | Ho, Jain, Abbeel | 2020 / NeurIPS | `hojonathanho/diffusion` |
| Score SDE / 概率流 ODE | Song 等 | 2021 / ICLR(Outstanding) | `yang-song/score_sde` |
| DDIM | Song, Meng, Ermon | 2021 / ICLR | `ermongroup/ddim` |
| Variational Diffusion(SNR 统一) | Kingma 等 | 2021 / NeurIPS | —— |
| EDM(VP/VE 统一、预处理) | Karras 等 | 2022 / NeurIPS | `NVlabs/edm` |
| Flow Matching | Lipman 等 | 2023 / ICLR | `facebookresearch/flow_matching` |
| Rectified Flow | Liu, Gong, Liu | 2023 / ICLR(Oral) | `gnobitab/RectifiedFlow` |
| Stochastic Interpolants | Albergo, Vanden-Eijnden | 2023 / arXiv,2025 / JMLR | —— |
| 收敛理论(TV，无 LSI) | Chen, Chewi 等 | 2023 / ICLR | —— |
| 收敛理论($d$-线性 KL) | Benton 等 | 2024 / ICLR(Spotlight) | —— |
| JKO($W_2$ 梯度流) | Jordan, Kinderlehrer, Otto | 1998 / SIAM J. Math. Anal. | —— |
| Diffuser(规划即去噪) | Janner 等 | 2022 / ICML(Long Talk) | `jannerm/diffuser` |
| Diffusion Policy | Chi 等 | 2023 / RSS,2024 / IJRR | `real-stanford/diffusion_policy` |
| SE(3)-DiffusionFields | Urain 等 | 2023 / ICRA | `robotgradient/grasp_diffusion` |
| $\pi_0$(FM VLA) | Black 等(Physical Intelligence) | 2024 / arXiv | `Physical-Intelligence/openpi` |
| Model-Based Diffusion(MPPI 桥接) | Pan 等 | 2024 / NeurIPS | `LeCAR-Lab/model-based-diffusion` |

> **引用与版本声明**：本专题引用计数(科研脉络表)截至 2025 年中，仅供量级参考；论文年份以正式发表/arXiv 首发为准。代表实现列仅为帮助读者定位参考代码，非官方背书，具体以各仓库 README 为准。本文档遵循 CC BY 4.0。

---

> **专题完**。回到开篇那句主线：**前向加噪简单，反向去噪难，而反向去噪的全部信息浓缩在 score $\nabla\log p_t$ 一个函数里——估计它就是训练，沿它走就是采样。** 走完八节，你应当能够：写出前向 VP/VE SDE 与闭式转移核(§8.4.1)；独立陈述并证明 Anderson 反向 SDE(§8.4.2)；从 ELBO 推出 $\varepsilon$-预测、理解 DDPM/DDIM 的取舍(§8.4.3)；独立证明 Vincent 去噪 SM 与 Tweedie 公式，把 $\varepsilon$/score/$x_0$/velocity 自由互译(§8.4.4)；说清三源误差与 Girsanov 证明骨架、知道 $d^2$ 与 $d$-线性界差在哪(§8.4.5)；独立证明 Lipman CFM 等价、理解 Rectified Flow 与 Stochastic Interpolants 的统一(§8.4.6)；用 $W_2$ 几何与 JKO 把所有视角装进一个框架(§8.4.7)；并把这一切桥接到 Diffusion Policy、$\pi_0$、Diffuser、SE(3) 扩散、CFG 与采样式 MPC(MPPI = 单步去噪)(§8.4.8)。
>
> 若其中任一环仍模糊，回到对应小节的"动机 → 反面 → 历史 → 理论"重走一遍——理解的裂缝，十之八九不在最后的公式，而在最初"为什么需要它"那一步没补上。这套数学的真正馈赠，不是某一个具体方法，而是 §8.4.7 那张"四重视角统一表"和 §8.4.8 应用九那张"动作头选型地图"——它们能容纳你未来遇到的每一个新方法，让你在 30 秒内把它定位到设计空间的某个角落。**理解一个框架，胜过记住十个方法。**

---
