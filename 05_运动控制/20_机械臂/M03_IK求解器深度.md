> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# M03 IK求解器——从解析解到优化式逆运动学

> **性质**: ❌ 纯机械臂 | **时长**: 1.5 周 (15-20 学时)
> **前置**: M01(Pinocchio 深度精读), M02(动力学库对比); 02_基础/C++ 并发编程
> **下游**: M04(碰撞检测) → M07(OMPL采样规划) → M14(MoveIt2 MTC 集成)

---

## M03.0 前置自测

开始本章之前，请独立回答以下 5 题。若答不出 3 题以上，建议先回顾 M01 和线性代数基础。

| # | 问题 | 期望答案要点 |
|---|------|-------------|
| 1 | 正运动学 FK(q) 的输入输出分别是什么？对串联链而言，输出唯一吗？ | 输入: 关节角 $q \in \mathbb{R}^n$；输出: 末端位姿 $T \in SE(3)$；给定 q 时输出唯一，这是函数的确定性；但 FK 通常**不是单射**，不同构型可能到达同一末端位姿 |
| 2 | Jacobian 矩阵 $J(q) \in \mathbb{R}^{6 \times n}$ 的物理意义是什么？当 $\det(JJ^T) \to 0$ 时意味着什么？ | $J$ 将关节速度 $\dot{q}$ 映射到末端速度 $v = J\dot{q}$；行列式趋零即**奇异构型**（singularity），此时某方向上运动能力丧失 |
| 3 | 给定 $T_1, T_2 \in SE(3)$，$\mathrm{Log}(T_1^{-1} T_2) \in \mathbb{R}^6$ 的几何含义？ | 从 $T_1$ 到 $T_2$ 的"切空间误差向量"。在 Pinocchio `Motion::toVector()` 约定下，前 3 维是线速度/平移相关分量，后 3 维是角速度/旋转相关分量；使用其他库时必须先确认 6D 向量排列 |
| 4 | `std::thread t(func, args...); t.join();` 的含义？`std::atomic<bool>` 相比普通 `bool` 有何保证？ | `thread` 启动新线程执行 `func`，`join` 阻塞等待其完成；`atomic<bool>` 保证**无数据竞争**的跨线程可见性（memory ordering） |
| 5 | 给定 $A \in \mathbb{R}^{m \times n}$（$m < n$），Moore-Penrose 伪逆 $A^\dagger$ 给出的解有何特殊性？ | $A^\dagger b$ 是满足 $Ax = b$ 的所有解中**2-范数最小**的那个（minimum-norm solution） |

---

## 本章目标

学完本章后，你应该能够：

1. **形式化** 逆运动学问题——理解解的存在性、唯一性、冗余分解，能判断给定机械臂结构是否有闭式解
2. **选型** 七大 IK 求解器（opw / IKFast / ik_geo / KDL / TRAC-IK / pick-ik / BioIK），根据精度、速度、成功率和约束需求做出合理选择
3. **实现** 基于 Pinocchio 的阻尼 Jacobian IK 和零空间投影 IK，理解 DLS 阻尼因子对奇异性处理的作用
4. **精读** TRAC-IK 的双线程竞速源码，理解 `std::thread` + `std::atomic` 在实时 IK 中的工程应用
5. **配置** MoveIt2 的 IK 插件机制（pluginlib），能在 KDL / TRAC-IK / pick-ik / IKFast 之间零代码切换
6. **理解** Differentiable IK 和 Learning-based IK 的前沿方向及其与传统方法的关系

---

## M03.1 逆运动学问题形式化 ⭐

### 动机：为什么 IK 是机械臂的核心问题？

用户告诉机器人："把杯子放到桌上 (x, y, z) 位置"——这是一个**任务空间**描述。但电机只认**关节角**。从任务空间到关节空间的映射，就是逆运动学（Inverse Kinematics, IK）。

> **跨领域类比**：IK 问题类似于导航中的"路径逆问题"。GPS 给你目标坐标（任务空间），但你的车只接受方向盘角度和油门（关节空间）。正运动学 FK 就是"给定一系列方向盘操作，车到了哪里"——这很容易计算。IK 是反过来——"要到某个位置，应该怎么打方向盘"——这困难得多，因为可能有多条路径（多解），也可能根本到不了（无解）。

> **反事实推理**：如果我们不做 IK 而直接在关节空间规划会怎样？这意味着用户必须自己指定 7 个关节角度——即使是经验丰富的机器人工程师也很难直觉地知道"把杯子放到桌上某点"对应什么关节角。更关键的是，任务空间的约束（末端沿直线移动、保持水平姿态）在关节空间中是高度非线性的耦合约束，直接在关节空间处理这些约束几乎不可行。IK 是连接"人类直觉"和"机器控制"的桥梁。

> **本质洞察**：IK 的数学本质是一个**非线性方程组的逆映射问题**。正运动学 $FK: \mathbb{R}^n \to SE(3)$ 是一个光滑映射，但它通常不是双射——逆映射可能不存在（超出工作空间）、不唯一（多解）、或在奇异点处不连续。IK 的所有困难都根源于此。

### 形式化定义

**正运动学**（Forward Kinematics, FK）:

$$
T = FK(q), \quad q \in \mathbb{R}^n, \quad T \in SE(3)
$$

对串联链而言，FK 是一个**连续满射**（surjective 到工作空间 $\mathcal{W} \subseteq SE(3)$），但**不是单射的逆方向**。

**逆运动学**（Inverse Kinematics, IK）:

$$
\text{给定 } T_{\text{target}} \in SE(3), \quad \text{求 } q^* \in \mathbb{R}^n \quad \text{使得 } FK(q^*) = T_{\text{target}}
$$

### 解空间分析

IK 的困难在于解的结构极其复杂：

| 自由度关系 | 解的性质 | 典型情形 |
|-----------|---------|---------|
| $n = 6$（恰好约束） | 有限个解（0, 2, 4, 8, 16 个） | 6-DOF 工业臂 |
| $n > 6$（冗余） | **无穷多解**（$(n-6)$ 维流形） | 7-DOF 协作臂 |
| $n < 6$（欠约束） | 通常无精确解，只能满足部分约束 | 4-DOF SCARA |

**存在性问题**:
- 目标在**工作空间边界**外 → 无解
- 目标恰好在边界上 → 奇异构型，仅有有限解且 Jacobian 退化
- 目标在工作空间内部 → 通常有解（6-DOF）或无穷多解（7-DOF+）

**唯一性问题**（6-DOF 为例）:

```
肘上(elbow-up) vs 肘下(elbow-down)
肩前(shoulder-front) vs 肩后(shoulder-back)  
腕翻(wrist-flip) vs 腕正(wrist-noflip)

→ 最多 2^3 = 8 种构型组合
```

### 三大求解范式

```
┌──────────────────────────────────────────────────────────────┐
│                    IK 求解范式全景                              │
├──────────────┬─────────────────────┬─────────────────────────┤
│  解析/闭式   │    数值迭代          │    优化/进化              │
│ (Analytical) │  (Numerical)        │  (Optimization)         │
├──────────────┼─────────────────────┼─────────────────────────┤
│ opw_kinem.   │  KDL (J-pinv NR)    │  TRAC-IK (KDL+SQP)     │
│ IKFast       │  Pinocchio+Jacobian │  pick-ik (L-BFGS)       │
│ ik_geo       │                     │  BioIK (evolutionary)   │
├──────────────┼─────────────────────┼─────────────────────────┤
│ ~0.1-5 us    │  ~1-2 ms            │  ~50-500 us             │
│ 100% 成功    │  30-60% (KDL NR 无重启) │  95-99.5% 成功          │
│ 仅限特定结构  │  通用                │  通用 + 可加约束         │
└──────────────┴─────────────────────┴─────────────────────────┘
```

### 历史脉络

| 年份 | 里程碑 | 意义 |
|------|--------|------|
| 1968 | Pieper 博士论文 | 证明 3 轴交汇 → 闭式解存在 |
| 1969 | Whitney 提出 Jacobian 伪逆 IK | 数值迭代方法奠基 |
| 2010 | Diankov (IKFast, OpenRAVE) | 符号消元自动代码生成 |
| 2014 | Brandstotter (OPW) | 参数化闭式解——工业臂通用 |
| 2015 | Beeson & Ames (TRAC-IK) | 双线程竞速，成功率质变 |
| 2017 | Starke (BioIK) | 进化算法+多目标约束 |
| 2022 | Elias & Wen (ik_geo) | 几何子问题分解，通用闭式 |
| 2024 | PickNik (pick-ik) | L-BFGS + Pinocchio, MoveIt2 原生 |

---

## M03.2 解析解——Pieper条件与OPW参数化 ⭐⭐⭐

### 动机：为什么工业臂几乎都有闭式解？

工厂产线要求 IK 在**亚微秒**内完成（伺服周期 1ms 内需调用数百次）。数值迭代太慢。解析解是唯一选择——而巧妙的**结构设计**正是为了让解析解存在。

### Pieper 定理 (1968)

**定理**: 若 6-DOF 串联臂满足以下条件之一，则其 IK 有封闭解：
1. **三轴交汇**（three consecutive axes intersect at a single point）——"球腕"
2. **三轴平行**（three consecutive axes are parallel）

**物理直觉**: 当后三轴交于一点（腕心, wrist center）时，可以**解耦**位置和姿态：
- 前 3 关节（肩+肘）决定**腕心位置** → 3 维位置方程
- 后 3 关节（腕）决定**末端姿态** → 3 维姿态方程

解耦后，每组方程独立求解，复杂度从 $6 \times 6$ 耦合降为 $3 + 3$。

### 为什么主流工业臂都满足 Pieper 条件？

| 品牌 | 型号 | 结构 | 满足条件 |
|------|------|------|---------|
| ABB | IRB 6700 | 球腕 | 后 3 轴交汇 |
| KUKA | KR 210 | 球腕 | 后 3 轴交汇 |
| Fanuc | LR Mate 200 | 球腕 | 后 3 轴交汇 |
| UR | UR5/UR10 | 平行+正交 | 轴 2,3,4 平行 |
| Motoman | GP8 | 球腕 | 后 3 轴交汇 |

> **设计哲学**: 工业臂的**机械结构**是为闭式 IK 而生的。不是"碰巧"有闭式解，而是**刻意设计**使其有解。

### OPW 参数化 (Orthogonal Parallel Wrist)

Brandstotter et al. 2014 提出的统一参数化方法，用 **7 个几何参数** 描述绝大多数工业臂：

$$
\text{OPW 参数}: \{a_1, a_2, b, c_1, c_2, c_3, c_4\} + \text{关节偏移/符号}
$$

```
           c1
    ┌──────┤
    │      │
    │  a1  │        c2         c3         c4
    │──────┼────────┼──────────┼──────────┤
    │      │        │          │          │
    ▼      ▼        ▼          ▼          ▼
  base   joint1   joint2     joint3     wrist_center
            │
            ├── a2 (垂直偏移)
            │
            └── b  (侧向偏移, 如 UR 系列有)
```

**求解过程**（UR5 为例，完整推导）:

**Step 1: 腕心位置解耦**

目标位姿 $T_{\text{target}} = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}$，腕心位置：

$$
p_w = p - c_4 \cdot R \cdot \hat{z}_6
$$

其中 $\hat{z}_6$ 是末端工具方向（通常为 $[0,0,1]^T$）。

**Step 2: 求解 $\theta_1$（底座旋转）**

$$
\theta_1 = \text{atan2}(p_{w,y}, \, p_{w,x}) - \text{atan2}\!\left(b, \, \pm\sqrt{p_{w,x}^2 + p_{w,y}^2 - b^2}\right)
$$

两个解对应 shoulder-left 和 shoulder-right。

**Step 3: 求解 $\theta_5$（腕翻转）**

$$
\theta_5 = \pm \arccos\!\left(\frac{p_{w,x}\sin\theta_1 - p_{w,y}\cos\theta_1 - b}{c_4}\right)
$$

**Step 4: 求解 $\theta_6$**

利用已知的 $\theta_1, \theta_5$，从旋转矩阵中提取 $\theta_6$：

$$
\theta_6 = \text{atan2}\!\left(\frac{-r_{01}\sin\theta_1 + r_{11}\cos\theta_1}{\sin\theta_5}, \; \frac{r_{00}\sin\theta_1 - r_{10}\cos\theta_1}{\sin\theta_5}\right)
$$

**Step 5: 求解 $\theta_3$（肘关节，余弦定理）**

$$
\cos\theta_3 = \frac{\|p_{w,\text{proj}}\|^2 - a_2^2 - c_3^2}{2 a_2 c_3}
$$
$$
\theta_3 = \pm\arccos(\cos\theta_3)
$$

两个解对应 elbow-up 和 elbow-down。

**Step 6: 求解 $\theta_2, \theta_4$**

每步利用已知角消元，最终得到**最多 8 组解**。

### opw_kinematics 代码实践

```cpp
#include <opw_kinematics/opw_kinematics.h>
#include <array>
#include <iostream>
#include <cmath>

// UR5 的 OPW 参数 (单位: 米)
opw_kinematics::Parameters<double> make_ur5_params() {
    opw_kinematics::Parameters<double> p;
    p.a1 =  0.0;       // base 到 shoulder 的 x 偏移
    p.a2 = -0.42500;   // upper arm 长度 (负号表示方向)
    p.b  =  0.0;       // 侧向偏移 (UR5 为零)
    p.c1 =  0.08946;   // base 到 shoulder 的 z 偏移
    p.c2 =  0.10915;   // shoulder 到 elbow 偏移
    p.c3 =  0.09465;   // wrist 1 到 wrist 2
    p.c4 =  0.0823;    // wrist 2 到 flange
    // 关节偏移 (UR 系列有 +/-pi/2 偏移)
    p.offsets = {0, -M_PI/2, 0, -M_PI/2, 0, 0};
    p.sign_corrections = {1, 1, 1, 1, 1, 1};
    return p;
}

int main() {
    auto params = make_ur5_params();

    // 目标位姿 (4x4 齐次变换矩阵, 列主序存储)
    // 这里用一个简单的前伸姿态
    std::array<double, 16> target = {
         1,  0,  0,  0,   // 第一列 (R 的第一列)
         0,  0,  1,  0,   // 第二列
         0, -1,  0,  0,   // 第三列
         0.4, 0.1, 0.3, 1 // 第四列 (位置 + 齐次坐标)
    };

    // 求解: 返回 Solutions<double> — 即 std::array<std::array<double,6>, 8>
    // 无效解对应的 joint 值为 NaN，需逐一检查
    auto solutions = opw_kinematics::inverse(params, target);

    int n_valid = 0;
    for (int i = 0; i < 8; ++i) {
        // 检查该组解是否有效（任一关节为 NaN 则无效）
        bool valid = true;
        for (int j = 0; j < 6; ++j) {
            if (std::isnan(solutions[i][j])) { valid = false; break; }
        }
        if (!valid) continue;
        ++n_valid;
        std::cout << "  解 " << i << ": [";
        for (int j = 0; j < 6; ++j) {
            double angle_deg = solutions[i][j] * 180.0 / M_PI;
            std::cout << angle_deg << "deg";
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]\n";
    }
    std::cout << "找到 " << n_valid << " 组有效 IK 解\n";

    // 验证: 正运动学检验 (forward 接受 std::array<T,6>，返回 Transform<T>)
    auto fk_result = opw_kinematics::forward(params, solutions[0]);

    // 计算位置误差
    double pos_err = std::sqrt(
        std::pow(fk_result[12] - target[12], 2) +
        std::pow(fk_result[13] - target[13], 2) +
        std::pow(fk_result[14] - target[14], 2));
    std::cout << "FK 验证位置误差: " << pos_err << " m\n";
    // 期望: < 1e-10 (机器精度)

    return 0;
}
// 编译: g++ -std=c++17 -O3 -I/path/to/opw_kinematics/include ur5_ik.cpp
// 性能: 单次求解 ~0.1 us (含所有 8 解)
```

### 解析解的局限

| 条件 | 能否用解析解？ | 推荐方案 |
|------|--------------|---------|
| 6-DOF 球腕工业臂 | 完美适用 | opw_kinematics 或 IKFast |
| 6-DOF 非标准结构 | 可能 | ik_geo (通用几何分解) |
| 7-DOF 冗余臂 (Panda, iiwa) | 通常不直接得到有限孤立解 | pick-ik / TRAC-IK / 指定冗余参数后的 IKFast |
| 带关节限位约束 | 部分 | 解析求解后筛选 |
| 需要避障等额外约束 | 不可 | pick-ik / BioIK |

> **Pitfall Box**: 解析解返回"所有数学上的解"，但不一定每个解都物理可行（可能超出关节限位）。实际使用时需要**后验筛选** + **选优**（如选最近当前构型的解，避免大关节跳变）。

---

## M03.3 Jacobian伪逆IK (KDL方法) ⭐⭐

### 动机：当没有闭式解时怎么办？

通用 7-DOF 臂对 6D 位姿 IK 通常是一维解族，而不是 6-DOF 工业臂那样的有限个孤立解。因此工程上很少直接追求"列出所有闭式解"。特定几何结构可以做解析或半解析参数化，IKFast 也可以通过指定 free joint 采样/参数化来生成求解器；但对 Panda、iiwa 这类冗余臂，**数值迭代**仍是最通用的后备方案——像 Newton-Raphson 求解非线性方程组一样，逐步逼近目标，同时把避障、关节限位、姿态偏好写成约束或代价。

### 算法推导

**目标**: 令误差趋零

$$
e(q) = \text{Log}\!\left(FK(q)^{-1} \cdot T_{\text{target}}\right) \in \mathbb{R}^6
$$

其中 $\text{Log}: SE(3) \to \mathfrak{se}(3) \cong \mathbb{R}^6$ 是对数映射（将位姿差转化为切空间向量）。

**线性化**: 在当前 $q_k$ 处泰勒展开。注意这里有两个常见符号约定：

如果 $J_e = \partial e / \partial q$ 是**误差函数本身的 Jacobian**，则

$$
e(q_k + \Delta q) \approx e(q_k) + J_e(q_k) \Delta q = 0
\quad\Rightarrow\quad
\Delta q = -J_e^\dagger e(q_k)
$$

很多 IK 实现采用另一种写法：用空间/体雅可比 $J$ 将关节速度映射到"减少误差所需的 twist"，先构造

$$
v_{\text{cmd}} = K e(q_k)
$$

再令

$$
\Delta q = J^\dagger v_{\text{cmd}}
$$

这两种写法的符号差异来自 $J_e$ 与几何雅可比 $J$ 的定义方向不同。实现时必须保持误差定义、Jacobian 参考系和更新符号一致；否则会把 Newton 步变成远离目标的正反馈。

Pinocchio 的 CLIK 示例采用更严格的 SE(3) 线性化语义：

$$
iM_d = FK(q_k)^{-1} T_{\text{target}}, \quad
e = \log(iM_d)
$$

此时 `log6` 给出的误差在当前末端的 body frame 中表达，不能直接拿几何 Jacobian 当作 $\partial e/\partial q$。应先计算 `LOCAL` 几何 Jacobian $J_L$，再用对数映射的雅可比修正：

$$
J_e = -J_{\log 6}(iM_d^{-1}) J_L, \quad
\Delta q = -J_e^\top (J_eJ_e^\top + \lambda^2 I)^{-1} e
$$

其中 $J^\dagger = J^T(JJ^T)^{-1}$ 是 Moore-Penrose 伪逆（当 $m < n$ 时给出最小范数解）。

**迭代**:

$$
q_{k+1} = \operatorname{integrate}(q_k, \alpha \cdot \Delta q), \quad \text{直到 } \|e(q_k)\| < \varepsilon
$$

### KDL 实现: ChainIkSolverPos_NR

KDL (Orocos Kinematics and Dynamics Library) 的 `ChainIkSolverPos_NR` 正是上述算法的直接实现：

```cpp
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <iostream>

int main() {
    // 构建 KDL chain (以 3-DOF 平面臂为例演示结构)
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                     KDL::Frame(KDL::Vector(0.5, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                     KDL::Frame(KDL::Vector(0.4, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                     KDL::Frame(KDL::Vector(0.3, 0, 0))));

    // FK 求解器
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // 速度级 IK (Jacobian 伪逆)
    KDL::ChainIkSolverVel_pinv ik_vel_solver(chain);

    // 位置级 IK (Newton-Raphson, 包裹速度求解器)
    // 参数: FK求解器, 速度IK求解器, 最大迭代次数, 收敛精度
    KDL::ChainIkSolverPos_NR ik_pos_solver(chain, fk_solver,
                                            ik_vel_solver, 100, 1e-6);

    // 求解
    KDL::JntArray q_init(chain.getNrOfJoints());  // 初始猜测 (全零)
    KDL::JntArray q_result(chain.getNrOfJoints());
    KDL::Frame target_frame(KDL::Rotation::RPY(0, 0, 0.5),
                            KDL::Vector(0.8, 0.2, 0));

    int ret = ik_pos_solver.CartToJnt(q_init, target_frame, q_result);

    if (ret >= 0) {
        std::cout << "KDL IK 成功! q = [";
        for (unsigned i = 0; i < q_result.rows(); ++i)
            std::cout << q_result(i) << (i < q_result.rows()-1 ? ", " : "");
        std::cout << "]\n";
    } else {
        std::cout << "KDL IK 失败! 错误码: " << ret << "\n";
        // ret = -1: 超过最大迭代次数 (未收敛)
        // ret = -3: 奇异点 (Jacobian 退化)
    }
    return 0;
}
```

### 问题诊断: KDL 的三大致命缺陷

**缺陷 1: 奇异点发散**

当 $\det(JJ^T) \to 0$ 时，$J^\dagger = J^T(JJ^T)^{-1}$ 的条目趋向无穷：

$$
\|J^\dagger\| \to \infty \quad \Rightarrow \quad \|\Delta q\| \to \infty
$$

关节角一步跳出合理范围，迭代炸掉。

**缺陷 2: 局部极小值陷阱**

若初始 $q_0$ 不佳，梯度方向可能引导到鞍点或局部极小值——末端"卡住"不再移动，但并未到达目标。

**缺陷 3: 无关节限位处理**

$q_{k+1}$ 可能超出物理关节限位 $[q_{\min}, q_{\max}]$，但 KDL NR 不做钳位。

### 阻尼最小二乘法 (Damped Least-Squares, DLS)

解决奇异点发散的经典方法——**Levenberg-Marquardt** 正则化：

$$
\Delta q = -J_e^T(J_eJ_e^T + \lambda^2 I)^{-1} \cdot e
$$

- 当 $\lambda = 0$ 时退化为标准伪逆
- $\lambda > 0$ 时，即使 $JJ^T$ 奇异也不会发散
- 代价是**精度降低**（末端不会精确到达目标，存在稳态误差 $O(\lambda)$）

**自适应阻尼**: 根据 $\sigma_{\min}(J)$（最小奇异值）动态调整 $\lambda$：
- 远离奇异 ($\sigma_{\min}$ 大) → $\lambda \approx 0$（保精度）
- 接近奇异 ($\sigma_{\min}$ 小) → $\lambda$ 增大（保稳定）

$$
\lambda = \begin{cases} 0 & \sigma_{\min} \geq \sigma_0 \\ \lambda_{\max}\left(1 - \frac{\sigma_{\min}^2}{\sigma_0^2}\right) & \sigma_{\min} < \sigma_0 \end{cases}
$$

### 使用 Pinocchio 实现完整阻尼 Jacobian IK

```cpp
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Dense>
#include <iostream>

namespace pin = pinocchio;

// 阻尼伪逆 IK 求解器
// 返回 true 表示收敛成功
bool solve_ik_damped(
    const pin::Model& model,
    pin::Data& data,
    pin::FrameIndex frame_id,         // 末端 frame
    const pin::SE3& T_target,         // 目标位姿
    Eigen::VectorXd& q,               // in: 初始猜测; out: IK 解
    double eps = 1e-4,                 // 收敛阈值
    int max_iter = 200,               // 最大迭代次数
    double dt = 1.0,                  // 步长
    double lambda = 1e-4              // 阻尼系数
) {
    const int nv = model.nv;  // 速度空间维度
    Eigen::MatrixXd J(6, nv);
    J.setZero();

    for (int i = 0; i < max_iter; ++i) {
        // 正运动学
        pin::forwardKinematics(model, data, q);
        pin::updateFramePlacement(model, data, frame_id);
        const pin::SE3& T_current = data.oMf[frame_id];

        // Pinocchio CLIK 语义: iMd = current^{-1} * target
        pin::SE3 iMd = T_current.actInv(T_target);
        Eigen::Matrix<double, 6, 1> err =
            pin::log6(iMd).toVector();

        // 收敛判断
        if (err.norm() < eps) {
            return true;  // 成功!
        }

        // LOCAL 几何 Jacobian 需要通过 Jlog6 转成误差 Jacobian。
        pin::computeFrameJacobian(model, data, q, frame_id,
                                   pin::LOCAL, J);
        Eigen::MatrixXd J_task =
            -pin::Jlog6(iMd.inverse()) * J;

        // 阻尼伪逆: dq = -J_task^T (J_task J_task^T + lambda^2 I)^{-1} e
        Eigen::MatrixXd JJt = J_task * J_task.transpose(); // 6x6
        JJt.diagonal().array() += lambda * lambda;      // 正则化
        Eigen::VectorXd delta_q =
            -J_task.transpose() * JJt.ldlt().solve(err);

        // 更新关节角 (流形积分, 正确处理四元数关节)
        q = pin::integrate(model, q, dt * delta_q);
    }
    return false;  // 超过最大迭代次数, 未收敛
}

int main() {
    // 加载 URDF
    pin::Model model;
    pin::urdf::buildModel("panda.urdf", model);
    pin::Data data(model);

    // 末端 frame
    pin::FrameIndex ee_id = model.getFrameId("panda_hand");

    // 目标位姿 (前伸 0.4m, 侧移 0.2m, 抬高 0.5m)
    pin::SE3 T_target(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.4, 0.2, 0.5)
    );

    // 初始关节角 (中间位置)
    Eigen::VectorXd q = pin::neutral(model);

    bool success = solve_ik_damped(model, data, ee_id, T_target, q);
    if (success) {
        std::cout << "IK 收敛成功! q = " << q.transpose() << "\n";
        // 验证
        pin::forwardKinematics(model, data, q);
        pin::updateFramePlacement(model, data, ee_id);
        auto err = pin::log6(data.oMf[ee_id].actInv(T_target)).toVector();
        std::cout << "最终误差范数: " << err.norm() << "\n";
    } else {
        std::cout << "IK 未收敛! 请检查目标是否在工作空间内。\n";
    }
    return 0;
}
```

> **Pitfall Box**: `pin::integrate` 用于流形上的积分（处理四元数关节等非欧几里得关节）。如果用简单的 `q += delta_q`，对含浮动基座或球关节的模型会出错。对于纯旋转关节的臂（如 Panda），两者等价，但养成使用 `integrate` 的习惯更安全。

---

## M03.4 TRAC-IK——双线程竞速 ⭐⭐⭐

### 动机：KDL 只有 60% 成功率，能否做到 99%+？

Beeson & Ames (2015, Humanoids) 的关键洞察：

> KDL 的 Newton-Raphson **快但不稳**（奇异点失败）；SQP 优化**稳但慢**。如果让两者**同时跑**，谁先成功就用谁的结果——就能同时获得速度和鲁棒性。

这就是 **TRAC-IK** (Tolerant IK for Real-time Arm Control) 的核心思想——**并行竞速模式(race pattern)**。

### 架构设计

```
┌─────────────────────────────────────────────────┐
│              TRAC_IK::CartToJnt()                │
│                                                  │
│  ┌─────────────┐          ┌───────────────────┐ │
│  │  Thread 1   │          │    Thread 2       │ │
│  │  KDL NR     │          │    NLopt SQP      │ │
│  │  + Random   │  race!   │  min ||FK(q)-T||  │ │
│  │   Restart   │◄────────►│  s.t. q_lo <= q   │ │
│  │             │          │       <= q_hi     │ │
│  └──────┬──────┘          └────────┬──────────┘ │
│         │                          │             │
│         ▼                          ▼             │
│    ┌──────────────────────────────────────┐      │
│    │   std::atomic<bool> solved_{false}   │      │
│    │   (第一个设为 true 的线程赢)           │      │
│    └──────────────────────────────────────┘      │
│                                                  │
│  结果: 先完成者的 q_solution                       │
└─────────────────────────────────────────────────┘
```

### Thread 1: KDL Newton-Raphson with Random Restarts

KDL 原版只从一个初始点开始迭代。TRAC-IK 改进：
- 若第一次 NR 迭代失败 → **随机重启**（在关节限位内均匀采样新 $q_0$）
- 循环直到找到解或被对方线程终止
- 关键优化：每次随机重启只跑少量迭代（~10次），快速试错

### Thread 2: SQP (Sequential Quadratic Programming)

将 IK 建模为**带约束非线性优化**：

$$
\min_{q} \quad \|FK(q) - T_{\text{target}}\|^2
$$
$$
\text{s.t.} \quad q_{\min} \le q \le q_{\max}
$$

使用 NLopt 库的 SLSQP 算法。虽然每步比 NR 慢，但：
- **严格尊重关节限位**（可行域内搜索）
- **全局搜索能力更强**（SQP 的二次子问题有全局性质）
- 对奇异构型不敏感（不依赖 $J^\dagger$）

### 核心代码解析

```cpp
// trac_ik_lib/src/trac_ik.cpp (教学简化版, 保留核心竞速逻辑)
#include <thread>
#include <atomic>
#include <mutex>
#include <random>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <nlopt.hpp>

class TRAC_IK {
public:
    // === 跨线程共享状态 ===
    std::atomic<bool> solved_{false};  // 竞速标志
    KDL::JntArray solution_;           // 最终解
    std::mutex sol_mutex_;             // 保护 solution_

    // 关节限位
    KDL::JntArray q_lower_, q_upper_;

    // KDL 求解器 (Thread 1 用)
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> kdl_solver_;

    // 超时 (默认 5ms)
    double timeout_sec_ = 0.005;

    int CartToJnt(const KDL::JntArray& q_init,
                  const KDL::Frame& target,
                  KDL::JntArray& result) {
        solved_.store(false, std::memory_order_release);

        // 启动双线程竞速
        std::thread t1(&TRAC_IK::runKDL, this,
                       std::cref(q_init), std::cref(target));
        std::thread t2(&TRAC_IK::runSQP, this,
                       std::cref(q_init), std::cref(target));

        t1.join();  // 等待两个线程都结束
        t2.join();

        if (solved_.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lk(sol_mutex_);
            result = solution_;
            return 0;  // 成功
        }
        return -1;  // 超时, 两者都未找到解
    }

private:
    void runKDL(const KDL::JntArray& q_init,
                const KDL::Frame& target) {
        std::mt19937 rng(std::random_device{}());
        KDL::JntArray q_start = q_init;
        KDL::JntArray q_out(q_init.rows());

        auto start_time = std::chrono::steady_clock::now();

        while (!solved_.load(std::memory_order_relaxed)) {
            // 超时检查
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > std::chrono::duration<double>(timeout_sec_))
                return;

            // 尝试 KDL NR 求解
            int ret = kdl_solver_->CartToJnt(q_start, target, q_out);

            if (ret >= 0) {
                // 找到解! 抢占标志
                std::lock_guard<std::mutex> lk(sol_mutex_);
                if (!solved_.load(std::memory_order_relaxed)) {
                    solution_ = q_out;
                    solved_.store(true, std::memory_order_release);
                }
                return;
            }

            // 失败 → 在关节限位内随机重启
            for (unsigned j = 0; j < q_start.rows(); ++j) {
                std::uniform_real_distribution<double> dist(
                    q_lower_(j), q_upper_(j));
                q_start(j) = dist(rng);
            }
        }
    }

    void runSQP(const KDL::JntArray& q_init,
                const KDL::Frame& target) {
        const unsigned n = q_init.rows();

        // NLopt: SLSQP (Sequential Least-Squares QP)
        nlopt::opt opt(nlopt::LD_SLSQP, n);

        // 关节限位作为 box constraints
        std::vector<double> lb(n), ub(n);
        for (unsigned j = 0; j < n; ++j) {
            lb[j] = q_lower_(j);
            ub[j] = q_upper_(j);
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // 目标函数: ||FK(q) - target||^2
        // (这里简化, 实际通过 lambda 传递 target 和 FK)
        opt.set_min_objective([](const std::vector<double>& q,
                                 std::vector<double>& grad,
                                 void* data) -> double {
            // ... FK(q) 计算, 误差平方和 ...
            return error_sq;
        }, nullptr);

        opt.set_xtol_rel(1e-6);
        opt.set_maxeval(200);

        // 初始猜测
        std::vector<double> q(n);
        for (unsigned j = 0; j < n; ++j)
            q[j] = q_init(j);

        double min_val;
        try {
            opt.optimize(q, min_val);
        } catch (...) { return; }

        // 检查是否收敛到足够精度
        if (min_val < 1e-6 && !solved_.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lk(sol_mutex_);
            if (!solved_.load(std::memory_order_relaxed)) {
                for (unsigned j = 0; j < n; ++j)
                    solution_(j) = q[j];
                solved_.store(true, std::memory_order_release);
            }
        }
    }
};
```

### C++ 并发机制详解

| 机制 | 作用 | 如果不用会怎样？ |
|------|------|----------------|
| `std::atomic<bool>` | 无锁跨线程标志 | 普通 `bool` 在 ARM/RISC-V 上可能出现 torn read 或 stale cache |
| `std::mutex` + `lock_guard` | 保护 `solution_` 写入 | 两线程同时写 → data race → undefined behavior |
| `memory_order_release` | 写者保证：之前的写入对读者可见 | 读者可能看到部分更新的 solution (torn state) |
| `memory_order_relaxed` | 热循环中避免 memory fence 开销 | 用 `seq_cst` 也正确，但每次循环多一个 full fence |
| Double-check pattern | 避免先到者被后到者覆盖 | 两个线程可能互相覆盖结果 (虽然都是合法解) |

### 性能实测数据 (Beeson & Ames 2015)

| 机器人 | KDL 成功率 | TRAC-IK 成功率 | TRAC-IK 平均时间 |
|--------|-----------|---------------|-----------------|
| UR5 (6-DOF) | 62.5% | **99.7%** | 0.32 ms |
| Jaco (6-DOF) | 53.8% | **99.5%** | 0.41 ms |
| Panda (7-DOF) | 58.2% | **99.8%** | 0.52 ms |
| Atlas arm (7-DOF) | 45.1% | **98.9%** | 0.61 ms |
| Baxter (7-DOF) | 49.3% | **99.4%** | 0.48 ms |

### ROS2 集成 (Jazzy/Kilted/Rolling)

```yaml
# kinematics.yaml (MoveIt2 配置文件)
panda_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05  # 50ms 超时 (内部有更细粒度超时)
  kinematics_solver_attempts: 3    # 失败后重试次数
  # TRAC-IK 特有参数
  solve_type: Distance  # Speed | Distance | Manip1 | Manip2
  epsilon: 1e-5         # 收敛精度
```

**solve_type 选择指南**:

| 模式 | 含义 | 适用场景 |
|------|------|---------|
| `Speed` | 返回第一个找到的解 | 规划阶段（只要可行解） |
| `Distance` | 选择离 seed 最近的解 | 轨迹跟踪（避免跳变） |
| `Manip1` | 选择 $\sqrt{\det(JJ^T)}$ 最大的解 | 需要高可操作性 |
| `Manip2` | 选择条件数最小的解 | 避免接近奇异 |

> **Pitfall Box**: `solve_type: Distance` 适合轨迹跟踪但**不适合**初始规划——因为它偏好当前构型附近的解，可能遗漏更好的全局解。规划阶段建议用 `Speed`。

---

## M03.5 IKFast——符号消元代码生成 ⭐⭐⭐

### 动机：能否让计算机自动推导闭式解？

手工推导 6-DOF IK 闭式解（如 M03.2 所示）极其繁琐。Diankov (2010, CMU PhD) 的天才想法：

> 把 FK 方程组输入**符号计算系统**，用 Groebner 基等代数几何工具**自动消元**，然后将结果**直接输出为 C++ 代码**。

生成的代码：无循环、无迭代、纯代数表达式 → **~1 us 求解**。

### 工作流程

```
┌─────────┐     ┌──────────────────┐     ┌───────────────────────┐
│  URDF   │────►│   OpenRAVE       │────►│ 生成的 C++ 代码        │
│ (机器人  │     │  符号消元引擎     │     │ ikfast_xxx.cpp         │
│  描述)   │     │  (Python/SymPy)  │     │ (数百~数千行纯代数式)   │
└─────────┘     └──────────────────┘     └───────────────────────┘
      │                                           │
      │       离线 (一次性, 分钟~小时)               │  在线 (每次 ~1us)
      └───────────────────────────────────────────┘
```

### 代码生成步骤

```bash
# 1. 准备环境 (Docker 推荐, OpenRAVE 依赖极其复杂)
docker pull personalrobotics/openrave:0.9-bionic

# 2. URDF → Collada DAE (OpenRAVE 输入格式)
rosrun collada_urdf urdf_to_collada ur5.urdf ur5.dae

# 3. 运行 IKFast 代码生成 (可能需要数分钟)
python3 `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py \
    --robot=ur5.dae \
    --iktype=transform6d \
    --baselink=0 \
    --eelink=6 \
    --savefile=ikfast_ur5.cpp

# 4. 编译 (无任何运行时依赖!)
g++ -std=c++11 -O3 -DIKFAST_NO_MAIN -shared \
    -fPIC ikfast_ur5.cpp -o libikfast_ur5.so

# 5. 或者直接嵌入你的项目
g++ -std=c++11 -O3 -DIKFAST_NO_MAIN \
    -c ikfast_ur5.cpp -o ikfast_ur5.o
```

### 生成代码结构解析

```cpp
// ikfast_ur5.cpp (自动生成, 教学简化展示)
// 实际生成代码可达 5000-10000 行纯代数表达式!

#define IKFAST_HAS_LIBRARY
#include "ikfast.h"  // 定义 IkReal, IkSolution 等基本类型

// 编译期常量: 机器人参数已内嵌
static const IkReal d1 = 0.08946;   // DH 参数
static const IkReal a2 = -0.42500;
static const IkReal a3 = -0.39225;
// ... (所有几何参数硬编码)

// 主求解函数
bool ComputeIk(const IkReal* eetrans,    // 末端位置 [x,y,z]
               const IkReal* eerot,       // 末端旋转 [3x3, 行主序]
               const IkReal* pfree,       // 自由关节值 (6-DOF 时为 nullptr)
               IkSolutionListBase<IkReal>& solutions) {
    IkReal x0, x1, x2, x3, x4, x5;  // 中间变量 (实际有数百个)

    // ===== 第一阶段: 腕心位置 =====
    // 从末端位姿反推腕心坐标
    x0 = eetrans[0] - 0.0823 * eerot[2];   // p_wx
    x1 = eetrans[1] - 0.0823 * eerot[5];   // p_wy
    x2 = eetrans[2] - 0.0823 * eerot[8];   // p_wz

    // ===== 第二阶段: theta1 (底座) =====
    IkReal x3_sq = x0*x0 + x1*x1;
    if (x3_sq < 1e-12) return false;  // 奇异: 腕心在 Z 轴上
    // 分支 1: theta1 = atan2(y, x)
    // 分支 2: theta1 = atan2(y, x) + pi
    // (生成的代码会枚举所有有效分支)

    // ===== 第三阶段: theta5 (腕部) =====
    // ... (数十行三角函数表达式) ...

    // ===== 第四阶段: theta6, theta3, theta2, theta4 =====
    // ... (数百行代数表达式, 全部是 +, -, *, /, atan2, sqrt, sin, cos) ...

    // ===== 枚举所有有效解 =====
    for (/* each valid branch combination */) {
        std::vector<IkReal> sol(6);
        sol[0] = theta1; sol[1] = theta2;
        sol[2] = theta3; sol[3] = theta4;
        sol[4] = theta5; sol[5] = theta6;
        solutions.AddSolution(sol, {});
    }
    return solutions.GetNumSolutions() > 0;
}

// 正运动学 (也自动生成)
void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot) {
    // DH 正运动学的展开形式
    // ... 同样是纯代数表达式 ...
}
```

### IKFast 的优势与局限

| 维度 | 优势 | 局限 |
|------|------|------|
| 速度 | ~1 us (纯代数, 无迭代) | — |
| 完备性 | 返回**所有**数学解 | — |
| 依赖 | **零运行时依赖** | 代码生成环境复杂 (OpenRAVE) |
| 适用范围 | — | 仅 6-DOF (或手动指定自由关节) |
| 可维护性 | — | 生成代码不可读/不可调试 |
| 灵活性 | — | 无法融入避障等约束 |
| 趋势 | — | 正被 ik_geo 逐步取代 |

### MoveIt2 中使用 IKFast

```bash
# MoveIt2 提供了 IKFast 生成器脚本 (简化 OpenRAVE 交互)
ros2 run moveit_kinematics create_ikfast_moveit_plugin.py \
    --robot_name=ur5 \
    --ikfast_plugin_pkg=ur5_ikfast_plugin \
    --base_link_name=base_link \
    --eef_link_name=tool0
```

```yaml
# kinematics.yaml
ur5_arm:
  kinematics_solver: ur5_ikfast_plugin/IKFastKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005  # 5ms 超时 (IKFast 实际 <0.01ms)
```

> **Pitfall Box**: IKFast 生成代码时假定 URDF 中的 link/joint 变换是固定的。如果后续修改了 URDF（如添加 tool frame、改变基座安装方式），**必须重新生成** IKFast 代码。这是一个常见的"配置漂移"bug 来源。

---

## M03.6 ik_geo——几何子问题分解 ⭐⭐⭐

### 动机：能否不依赖 OpenRAVE，也不手推每种机器人的闭式解？

IKFast 的痛点是：每换一个机器人就要重新跑代码生成（且环境难配）。Elias & Wen (2022, ASME J. Mechanisms and Robotics; 2025 Mechanism and Machine Theory, Volume 209, Article 105971 扩展版) 提出了一种**统一的几何分解框架**：

> 任何 6-DOF 串联臂的 IK 都可以分解为若干"**规范子问题**"(canonical subproblems) 的序列组合。每个子问题有已知的闭式解。不需要为每个机器人做符号消元。

### 核心思想: Product of Exponentials

用**旋量指数积**(Product of Exponentials, PoE) 表示 FK：

$$
T(\theta) = e^{[\xi_1]\theta_1} \cdot e^{[\xi_2]\theta_2} \cdots e^{[\xi_6]\theta_6} \cdot M = T_{\text{target}}
$$

其中 $\xi_i \in \mathbb{R}^6$ 是第 $i$ 个关节的旋量，$M$ 是零位时的末端位姿。

通过**策略性地左乘/右乘**已知的逆变换，将方程逐步简化为标准子问题形式。

### 六种规范子问题

| 子问题 | 方程形式 | 几何含义 | 未知数 |
|--------|---------|---------|--------|
| **SP1** | $e^{[\hat\omega]\theta} p = q$ | 点 $p$ 绕轴 $\omega$ 旋转到 $q$ | 1 个 $\theta$ |
| **SP2** | $e^{[\hat\omega_1]\theta_1} e^{[\hat\omega_2]\theta_2} p = q$ | 依次绕两轴旋转 | 2 个 $\theta$ |
| **SP3** | $\|e^{[\hat\omega]\theta} p - q\| = d$ | 旋转后到 $q$ 的距离为 $d$ | 1 个 $\theta$ |
| **SP4** | $h^T e^{[\hat\omega]\theta} p = d$ | 旋转后在 $h$ 方向的投影为 $d$ | 1 个 $\theta$ |
| **SP5** | $h_1^T e^{[\hat\omega_1]\theta_1} p_1 = h_2^T e^{[\hat\omega_2]\theta_2} p_2$ | 两轴旋转后投影相等 | 2 个 $\theta$ |
| **SP6** | $\|e^{[\hat\omega_1]\theta_1} p_1 - e^{[\hat\omega_2]\theta_2} p_2\| = d$ | 两轴旋转后间距为 $d$ | 2 个 $\theta$ |

### SP1 详细推导

**问题**: 给定旋转轴 $\omega$（单位向量）, 轴上一点 $r$, 初始点 $p$, 目标点 $q$, 求 $\theta$ 使得绕 $\omega$ 旋转 $\theta$ 后 $p$ 到达 $q$。

**解法**: 将 $p$ 和 $q$ 投影到垂直于 $\omega$ 的平面：

$$
u = (p - r) - \omega \cdot [\omega^T (p - r)]
$$
$$
v = (q - r) - \omega \cdot [\omega^T (q - r)]
$$

这两个向量在垂直于 $\omega$ 的平面内，且旋转不改变到轴的距离：

$$
\theta = \text{atan2}(\omega^T(u \times v), \; u^T v)
$$

**存在条件**: $\|u\| = \|v\|$（若不等则无解——点到轴的距离不同，旋转无法到达）。

### SP2 推导思路

**问题**: $e^{[\hat\omega_1]\theta_1} e^{[\hat\omega_2]\theta_2} p = q$

**策略**: 引入中间点 $c = e^{[\hat\omega_2]\theta_2} p$，则问题分解为：
- $e^{[\hat\omega_1]\theta_1} c = q$ → SP1（但 $c$ 未知）
- $c$ 在以 $\omega_2$ 轴上某点为圆心的圆上
- $q$ 在以 $\omega_1$ 轴上某点为圆心的圆上
- 两圆的交点即为 $c$ → 几何求交

最多 **2 组解**（两圆交于 0 或 2 点）。

### 求解流程示例 (球腕 6-DOF)

```
原始方程: e^{xi1*t1} e^{xi2*t2} e^{xi3*t3} e^{xi4*t4} e^{xi5*t5} e^{xi6*t6} M = T_target

Step 1: 令 T_goal = T_target * M^{-1}
        e^{xi1*t1} ... e^{xi6*t6} = T_goal

Step 2: 利用 axes 4,5,6 交汇于腕心 p_w
        两边作用于 p_w:
        e^{xi1*t1} e^{xi2*t2} e^{xi3*t3} p_w = T_goal * p_w
                                                 ^^^^^^^^^^^^
                                                 已知的点 q
        这是 "3 轴依次旋转一个点到目标" 问题
        → 分解为 SP3 + SP2 (或 SP2 + SP1 组合)
        → 求出 theta1, theta2, theta3

Step 3: 已知 theta1, theta2, theta3
        → 左边 e^{xi1*t1} e^{xi2*t2} e^{xi3*t3} 已知 (记为 T_123)
        → T_123^{-1} * T_goal = e^{xi4*t4} e^{xi5*t5} e^{xi6*t6}
        → 同样用 SP2 + SP1 求出 theta4, theta5, theta6
```

对于**非球腕**结构，ik_geo 会自动选择不同的子问题分解策略。论文证明：任何 6-DOF 串联链都可以用 SP1-SP6 的某种组合覆盖。

### ik_geo 代码使用

```cpp
// ik_geo (C++17, Eigen-based)
// 参考 GitHub: rpiRobotics/ik-geo 及 Verdant-Robotics/ik_geo
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

// 子问题求解器 (核心函数)
namespace ik_geo {

// SP1: 绕轴 omega 旋转, 使 p → q
// 返回角度 theta (弧度), 若无解返回 NaN
double subproblem1(const Eigen::Vector3d& omega,
                   const Eigen::Vector3d& r,
                   const Eigen::Vector3d& p,
                   const Eigen::Vector3d& q) {
    Eigen::Vector3d u = (p - r) - omega * omega.dot(p - r);
    Eigen::Vector3d v = (q - r) - omega * omega.dot(q - r);

    // 存在性检查
    if (std::abs(u.norm() - v.norm()) > 1e-10)
        return std::nan("");  // 无解

    double theta = std::atan2(omega.dot(u.cross(v)), u.dot(v));
    return theta;
}

// SP3: 绕轴旋转使距离等于 d
// 可能有 0 或 2 个解
std::vector<double> subproblem3(const Eigen::Vector3d& omega,
                                const Eigen::Vector3d& r,
                                const Eigen::Vector3d& p,
                                const Eigen::Vector3d& q,
                                double d) {
    Eigen::Vector3d u = (p - r) - omega * omega.dot(p - r);
    Eigen::Vector3d v = (q - r) - omega * omega.dot(q - r);

    double u_norm = u.norm();
    double v_norm = v.norm();

    // 余弦定理: d^2 = u^2 + v^2 - 2*u*v*cos(theta - phi)
    double d_sq_proj = d*d - std::pow(omega.dot(p - q), 2);
    if (d_sq_proj < 0) return {};  // 无解

    double cos_val = (u_norm*u_norm + v_norm*v_norm - d_sq_proj)
                   / (2 * u_norm * v_norm);
    if (std::abs(cos_val) > 1.0 + 1e-10) return {};  // 无解

    cos_val = std::clamp(cos_val, -1.0, 1.0);
    double phi = std::atan2(omega.dot(u.cross(v)), u.dot(v));
    double delta = std::acos(cos_val);

    return {phi + delta, phi - delta};  // 两个解
}

// 完整 6-DOF IK (球腕结构)
struct RobotParams {
    std::array<Eigen::Vector3d, 6> omega;  // 关节轴方向
    std::array<Eigen::Vector3d, 6> r;      // 关节轴上一点
    Eigen::Vector3d p_wrist;               // 腕心位置 (零位)
    Eigen::Matrix4d M_home;                // 零位末端位姿
};

std::vector<std::array<double,6>> solve_ik_spherical_wrist(
    const RobotParams& robot,
    const Eigen::Matrix4d& T_target)
{
    std::vector<std::array<double,6>> all_solutions;

    // 目标中的腕心位置
    Eigen::Matrix4d T_goal = T_target * robot.M_home.inverse();
    Eigen::Vector3d q_wrist = T_goal.block<3,3>(0,0) * robot.p_wrist
                            + T_goal.block<3,1>(0,3);

    // === 求解前 3 个关节 (腕心位置) ===
    // 使用 SP3 求 theta3, 然后 SP2 求 theta1, theta2
    // (具体组合取决于轴的排列)

    auto theta3_solutions = subproblem3(
        robot.omega[2], robot.r[2],
        robot.p_wrist, q_wrist,
        /* distance */ 0.0  // 简化示意
    );

    for (double t3 : theta3_solutions) {
        // 对每个 theta3, 用 SP1/SP2 求 theta1, theta2
        // ...

        // === 求解后 3 个关节 (腕部姿态) ===
        // 已知前 3 轴旋转 → 计算残余旋转
        // 用 SP2 + SP1 分解为 theta4, theta5, theta6
        // ...

        all_solutions.push_back({t1, t2, t3, t4, t5, t6});
    }

    return all_solutions;
}

}  // namespace ik_geo
```

### ik_geo vs IKFast vs opw 三方对比

| 维度 | ik_geo | IKFast | opw_kinematics |
|------|--------|--------|----------------|
| 求解模式 | 运行时子问题组合 | 离线生成纯代数代码 | 参数化公式 |
| 代码量 | ~500 行通用代码 | ~5000 行/机器人 | ~300 行 |
| 代码生成 | **不需要** | 需要 OpenRAVE | **不需要** |
| 运行时间 | ~5 us | ~1 us | ~0.1 us |
| 通用性 | 任意 6-DOF | 任意 6-DOF (需重生成) | 仅 OPW 结构 |
| 依赖 | Eigen | 无 | Header-only |
| 可调试性 | 高 | 极低 | 高 |
| ROS2 插件 | 暂无 (2025) | moveit ikfast | opw_ros2 |

> **趋势判断 (2025)**: 新的 6-DOF IK 项目推荐优先考虑顺序：
> 1. **opw_kinematics** (如果是 OPW 结构) — 最快、最简单
> 2. **ik_geo** (通用 6-DOF) — 一份代码适配所有机器人
> 3. **IKFast** (需要极致 1us 速度且已有生成代码) — 遗留系统

---

## M03.7 pick-ik——优化式IK (L-BFGS + Pinocchio) ⭐⭐

### 动机：如何让 IK 求解器"知道"障碍物在哪里？

解析解和基础数值法只能满足 $FK(q) = T$。但实际应用中，我们经常需要额外约束：
- 解**远离**关节限位（安全裕量）
- 解**靠近**当前构型（轨迹平滑）
- 解**远离**障碍物（避碰）
- 解具有**高可操作性**（远离奇异）

只有**优化式 IK** 能自然融入这些额外代价。

### pick-ik 定位 (PickNik Robotics, 2024)

pick-ik 是 MoveIt2 生态中 PickNik Robotics 推出的现代 IK 求解器：
- **后端**: Pinocchio (FK + Jacobian 计算, 业界最快)
- **局部优化器**: L-BFGS (拟牛顿法, 只需梯度)
- **全局搜索**: Memetic algorithm (进化 + 局部优化混合)
- **接口**: MoveIt2 `KinematicsBase` 插件, 即插即用
- **语言**: 纯 C++17, 利用 `std::span`, `std::optional` 等现代特性

### 优化问题建模

$$
\min_{q} \quad \underbrace{w_p \cdot \|p_{FK}(q) - p_{\text{target}}\|^2 + w_o \cdot \|R_{FK}(q) \ominus R_{\text{target}}\|^2}_{\text{位姿误差}} + \sum_i \lambda_i \cdot c_i(q)
$$

$$
\text{s.t.} \quad q_{\min} \le q \le q_{\max}
$$

其中 $c_i(q)$ 是用户定义的代价项：

| 代价项 | 表达式 | 效果 |
|--------|--------|------|
| `center_joints` | $\|q - q_{\text{mid}}\|^2$ | 避免接近限位 |
| `minimal_displacement` | $\|q - q_{\text{seed}}\|^2$ | 平滑, 避免跳变 |
| `avoid_singularity` | $-\sqrt{\det(JJ^T)}$ | 远离奇异构型 |
| 自定义 | $f(q)$ | 避障、能量等 |

### L-BFGS 求解器原理

L-BFGS (Limited-memory Broyden-Fletcher-Goldfarb-Shanno) 是拟牛顿法的一种，核心特点：

**标准 Newton 法**: $\Delta q = -H^{-1} \nabla f$，需要 Hessian $H \in \mathbb{R}^{n \times n}$（$O(n^2)$ 存储 + $O(n^3)$ 求逆）

**BFGS**: 用梯度差近似 Hessian 逆，不需要二阶导数

**L-BFGS**: 只存储最近 $m$ 步的梯度差信息（通常 $m = 5 \sim 20$），内存 $O(mn)$

```
L-BFGS 单步迭代:
───────────────────────────────────────────
输入: 当前点 q_k, 梯度 g_k = nabla f(q_k)
      历史存储: {s_i, y_i}_{i=k-m}^{k-1}
      其中 s_i = q_{i+1} - q_i, y_i = g_{i+1} - g_i

1. 两重循环 (Two-Loop Recursion):
   r = g_k
   for i = k-1, ..., k-m:
       alpha_i = rho_i * s_i^T r
       r = r - alpha_i * y_i
   r = H0 * r   (H0 = gamma_k * I, 初始 Hessian 估计)
   for i = k-m, ..., k-1:
       beta = rho_i * y_i^T r
       r = r + (alpha_i - beta) * s_i
   search_direction = -r

2. 线搜索: 找 alpha 满足 Wolfe 条件
   q_{k+1} = q_k + alpha * search_direction

3. 更新存储:
   s_k = q_{k+1} - q_k
   y_k = g_{k+1} - g_k
   若 |存储| > m: 丢弃最旧的一对
───────────────────────────────────────────
```

**为什么 L-BFGS 适合 IK？**
- IK 的 cost function 梯度可通过 Jacobian **解析计算**（$\nabla \|e\|^2 = 2J^T e$）
- 不需要 Hessian（避免计算 $\frac{\partial^2 FK}{\partial q^2}$）
- 超线性收敛（比梯度下降快得多）
- 内存效率高（7-DOF: 仅需存储 ~10 个 7维向量）

### pick-ik 代码使用 (C++ API)

```cpp
// 教学伪代码，实际 pick-ik 通过 MoveIt2 KinematicsBase 插件接口调用
// (以下接口为概念性展示，不存在 pick_ik::solve() 自由函数)
#include <pick_ik/fk_moveit.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_solver.hpp>
#include <pick_ik/robot.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include <iostream>

int main() {
    // === 1. 加载机器人模型 ===
    pinocchio::Model model;
    pinocchio::urdf::buildModel("panda.urdf", model);

    // === 2. 配置求解器参数 ===
    pick_ik::RobotConfig robot_config;
    robot_config.joint_names = {"panda_joint1", "panda_joint2",
        "panda_joint3", "panda_joint4", "panda_joint5",
        "panda_joint6", "panda_joint7"};
    robot_config.frame_name = "panda_hand";

    pick_ik::SolverConfig solver_config;
    solver_config.mode = pick_ik::SolverMode::Global;  // 或 Local
    solver_config.position_threshold = 1e-4;   // 位置精度 (m)
    solver_config.orientation_threshold = 1e-3; // 姿态精度 (rad)
    solver_config.max_iterations = 100;
    solver_config.num_threads = 4;             // 并行线程数 (global mode)
    solver_config.population_size = 16;        // 种群大小

    // === 3. 定义目标 (位姿 + 额外代价) ===
    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    target_pose.translation() = Eigen::Vector3d(0.4, 0.2, 0.5);

    // 额外代价: 关节居中
    auto center_cost = [&](const Eigen::VectorXd& q) -> double {
        Eigen::VectorXd q_mid = (model.upperPositionLimit
                                + model.lowerPositionLimit) / 2.0;
        return 0.01 * (q - q_mid).squaredNorm();
    };

    // === 4. 求解 ===
    Eigen::VectorXd q_seed = Eigen::VectorXd::Zero(7);  // 初始猜测

    auto result = pick_ik::solve(model, robot_config, solver_config,
                                 target_pose, q_seed, {center_cost});

    if (result.success) {
        std::cout << "pick-ik 成功!\n";
        std::cout << "  解: " << result.q.transpose() << "\n";
        std::cout << "  迭代: " << result.iterations << "\n";
        std::cout << "  耗时: " << result.solve_time_us << " us\n";
        std::cout << "  位置误差: " << result.position_error << " m\n";
    } else {
        std::cout << "pick-ik 未收敛 (达到最大迭代或超时)\n";
    }
    return 0;
}
```

### MoveIt2 配置 (最常用方式)

```yaml
# panda_moveit_config/config/kinematics.yaml
panda_arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05   # 50ms 超时
  # === pick-ik 特有参数 ===
  mode: global                       # global (memetic) | local (L-BFGS)
  position_threshold: 0.001          # 位置收敛阈值 (m)
  orientation_threshold: 0.01        # 姿态收敛阈值 (rad)
  cost_threshold: 0.01               # 总代价阈值
  approximate_solution: false        # 是否接受近似解
  # L-BFGS 参数
  max_iterations: 100                # 最大迭代次数
  # Memetic (global mode) 参数
  memetic_num_threads: 4             # 并行种群线程
  memetic_population_size: 16        # 种群大小
  memetic_stop_on_first: true        # 第一个解即停
  memetic_elite_size: 4              # 精英保留数
  # 代价权重
  center_joints_weight: 0.01         # 关节居中代价
  minimal_displacement_weight: 0.0   # 最小位移代价 (默认关闭)
```

### pick-ik 的两种模式对比

| 特性 | `local` (L-BFGS) | `global` (Memetic) |
|------|-------------------|-------------------|
| 算法 | 纯 L-BFGS 局部优化 | 进化搜索 + L-BFGS 精炼 |
| 速度 | ~20-50 us | ~100-500 us |
| 全局性 | 依赖初始猜测 | 多点并行探索 |
| 线程 | 单线程 | 多线程 (可配) |
| 适用 | 轨迹跟踪、实时 | 规划阶段、初始求解 |

> **Pitfall Box**: `mode: global` 使用多线程。在实时控制循环 (1kHz) 中使用时要注意：(1) 线程亲和性——确保 IK 线程不抢占控制线程的 CPU 核; (2) 优先级反转——mutex 可能导致低优先级线程阻塞高优先级线程。实时场景建议 `mode: local` + 好的 seed。

---

## M03.8 冗余分解与零空间投影 ⭐⭐⭐

### 动机：7-DOF 臂有无穷多 IK 解，如何选"最好"的？

7-DOF 臂（如 Franka Panda, KUKA iiwa）比完成 6-DOF 任务多出 1 个自由度。这意味着在满足末端位姿的前提下，还有 **1 维自由度**可以用来优化其他目标——这就是**冗余度**(redundancy)。

### 数学基础

对 7-DOF 臂，Jacobian $J \in \mathbb{R}^{6 \times 7}$：

**零空间**(Null Space):

$$
\mathcal{N}(J) = \{v \in \mathbb{R}^7 \mid Jv = 0\}, \quad \dim(\mathcal{N}) = 7 - \text{rank}(J) = 1
$$

(当 $J$ 满秩时)

**零空间投影矩阵**:

$$
N = I_7 - J^\dagger J \in \mathbb{R}^{7 \times 7}
$$

**物理意义**: 对任意 $v \in \mathbb{R}^7$，$Nv$ 是 $v$ 在零空间上的分量——它产生关节运动但**不改变末端位姿**（$J \cdot Nv = 0$）。

### 冗余分解公式

$$
\dot{q} = \underbrace{J^\dagger \cdot v_{\text{task}}}_{\text{主任务: 末端运动}} + \underbrace{(I - J^\dagger J) \cdot \dot{q}_0}_{\text{子任务: 零空间优化}}
$$

其中 $\dot{q}_0$ 是**任意**的关节速度——被投影到零空间后，不会干扰主任务。

聪明的选择：$\dot{q}_0 = k \cdot \nabla \phi(q)$（某个标量目标函数的梯度），在执行主任务的同时**梯度下降/上升**优化子目标。

### 常用子任务目标

| 目标 | 代价函数 $\phi(q)$ | 梯度 $\nabla\phi$ | 效果 |
|------|-----------|-------------------|------|
| 关节居中 | $\frac{1}{2}\|q - q_{\text{mid}}\|^2$ | $q - q_{\text{mid}}$ | 远离关节限位 |
| 可操作性 | $\sqrt{\det(JJ^T)}$ | (需数值或解析) | 远离奇异构型 |
| 避障 | $\min_i d_i(q)$ | 最近障碍方向梯度 | 远离障碍物 |
| 能量最小 | $\frac{1}{2}\|\tau(q)\|^2$ | (需动力学) | 减少力矩 |
| 自碰撞避免 | $\min_{i \ne j} d(L_i, L_j)$ | link 间距离梯度 | 避免自碰 |

### 完整实现 (Pinocchio)

```cpp
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include <algorithm>  // std::clamp

namespace pin = pinocchio;

// 带零空间优化的冗余 IK 求解器
bool solve_ik_nullspace(
    const pin::Model& model,
    pin::Data& data,
    pin::FrameIndex frame_id,
    const pin::SE3& T_target,
    Eigen::VectorXd& q,                                     // in/out
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> null_gradient,
    double eps = 1e-4,
    int max_iter = 500,
    double dt = 0.5,
    double lambda = 1e-4,
    double null_gain = 0.1
) {
    const int nv = model.nv;
    Eigen::MatrixXd J(6, nv);

    for (int iter = 0; iter < max_iter; ++iter) {
        // FK + Jacobian
        pin::forwardKinematics(model, data, q);
        pin::updateFramePlacement(model, data, frame_id);
        pin::computeFrameJacobian(model, data, q, frame_id,
                                   pin::LOCAL, J);

        // 主任务误差
        pin::SE3 iMd = data.oMf[frame_id].actInv(T_target);
        Eigen::Matrix<double, 6, 1> err =
            pin::log6(iMd).toVector();

        if (err.norm() < eps) return true;  // 收敛

        Eigen::MatrixXd J_task =
            -pin::Jlog6(iMd.inverse()) * J;

        // 阻尼伪逆 J_pinv = J_task^T (J_task J_task^T + lambda^2 I)^{-1}
        Eigen::MatrixXd JJt = J_task * J_task.transpose();
        JJt.diagonal().array() += lambda * lambda;
        Eigen::MatrixXd J_pinv = J_task.transpose() * JJt.inverse();

        // 主任务关节速度
        Eigen::VectorXd dq_primary = -J_pinv * err;

        // 零空间投影: N = I - J_pinv * J_task
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nv, nv)
                          - J_pinv * J_task;

        // 子任务: 零空间内优化
        Eigen::VectorXd grad = null_gradient(q);
        Eigen::VectorXd dq_null = N * grad;

        // 合成
        Eigen::VectorXd dq = dq_primary + null_gain * dq_null;

        // 流形积分
        q = pin::integrate(model, q, dt * dq);

        // 关节限位钳位
        for (int i = 0; i < model.nq; ++i) {
            q[i] = std::clamp(q[i],
                              model.lowerPositionLimit[i],
                              model.upperPositionLimit[i]);
        }
    }
    return false;  // 未收敛
}

int main() {
    // 加载 Panda 7-DOF 模型
    pin::Model model;
    pin::urdf::buildModel("panda.urdf", model);
    pin::Data data(model);
    pin::FrameIndex ee_id = model.getFrameId("panda_hand");

    // 子任务: 关节居中 (梯度方向朝 q_mid)
    Eigen::VectorXd q_mid = (model.upperPositionLimit
                            + model.lowerPositionLimit) / 2.0;

    auto center_gradient = [&](const Eigen::VectorXd& q) -> Eigen::VectorXd {
        // 梯度方向: 朝向中心 (即下降方向)
        return -(q - q_mid);
    };

    // 目标位姿
    pin::SE3 T_target(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
        Eigen::Vector3d(0.4, 0.0, 0.5)
    );

    Eigen::VectorXd q = pin::neutral(model);  // 零位出发

    bool ok = solve_ik_nullspace(model, data, ee_id, T_target,
                                 q, center_gradient);
    if (ok) {
        std::cout << "零空间 IK 成功!\n";
        std::cout << "  解: " << q.transpose() << "\n";

        // 验证: 解到中间位置的距离
        double dist_to_mid = (q - q_mid).norm();
        std::cout << "  到中间位置距离: " << dist_to_mid << "\n";

        // 对比: 不用零空间优化时的距离
        Eigen::VectorXd q2 = pin::neutral(model);
        // (不带 null_gradient 求解...)
        // 可以观察到 dist_to_mid 更小
    }
    return 0;
}
```

### 多优先级任务层级 (Siciliano 1991)

当有多个子任务时，可以建立**严格优先级层级**：

$$
\dot{q} = J_1^\dagger v_1 + N_1 \left[ (J_2 N_1)^\dagger (v_2 - J_2 J_1^\dagger v_1) + N_2' \left[ \ldots \right] \right]
$$

其中 $N_1 = I - J_1^\dagger J_1$，$N_2' = I - (J_2 N_1)^\dagger (J_2 N_1)$。

**优先级示例**:
- Level 1 (最高): 末端位姿跟踪 → $J_1 = J_{\text{ee}}$
- Level 2: 避碰 → $J_2 = J_{\text{collision}}$
- Level 3 (最低): 关节居中 → $\nabla\phi$

低优先级任务**只在高优先级的零空间中**执行——绝不干扰高优先级目标。

> **Pitfall Box**: 多层优先级的零空间计算存在**算法奇异性**——当高优先级任务的零空间维度骤变时（如接近奇异构型），投影矩阵不连续，导致关节速度跳变。工程中常用**加权伪逆**(`Weighted Least-Norm`) 做平滑过渡：
> $$\dot{q} = J_W^\dagger v + (I - J_W^\dagger J) W^{-1} \nabla\phi$$
> 其中 $W$ 是正定权重矩阵，通过连续调整 $W$ 避免不连续。

---

## M03.9 MoveIt2 IK插件机制 ⭐⭐

### 动机：如何在不改代码的情况下切换 IK 求解器？

MoveIt2 通过 **pluginlib** 实现运行时动态加载 IK 求解器。用户只需改 YAML 配置文件，即可切换 KDL / TRAC-IK / pick-ik / IKFast / BioIK——无需重新编译任何 C++ 代码。

### KinematicsBase 接口

```cpp
// 核心虚函数接口 (简化版)
// moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h

namespace kinematics {

class KinematicsBase {
public:
    virtual ~KinematicsBase() = default;

    // === 初始化 (MoveIt2 加载插件时调用) ===
    virtual bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const moveit::core::RobotModel& robot_model,
        const std::string& group_name,       // "panda_arm"
        const std::string& base_frame,       // "panda_link0"
        const std::vector<std::string>& tip_frames,  // ["panda_hand"]
        double search_discretization
    ) = 0;

    // === 核心: 单目标 IK ===
    virtual bool getPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,     // 目标
        const std::vector<double>& ik_seed_state,    // 初始猜测
        std::vector<double>& solution,               // 输出解
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const KinematicsQueryOptions& options = {}
    ) const = 0;

    // === 核心: 带回调的搜索式 IK ===
    // 回调用于碰撞检测——找到解后验证是否无碰撞
    virtual bool searchPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        double timeout,
        std::vector<double>& solution,
        const IKCallbackFn& solution_callback,  // 碰撞检测回调
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const KinematicsQueryOptions& options = {}
    ) const = 0;

    // === FK (通常委托给内部模型) ===
    virtual bool getPositionFK(
        const std::vector<std::string>& link_names,
        const std::vector<double>& joint_angles,
        std::vector<geometry_msgs::msg::Pose>& poses
    ) const = 0;

    // === 查询接口 ===
    virtual const std::vector<std::string>& getJointNames() const = 0;
    virtual const std::vector<std::string>& getLinkNames() const = 0;
};

}  // namespace kinematics
```

### pluginlib 动态加载流程

```
1. CMake 编译时: 生成 .so 动态库
2. package.xml 中声明 <export> 标签
3. plugin_description.xml 描述类名映射
4. MoveIt2 启动时:
   a) 读取 kinematics.yaml → 获取插件名
   b) pluginlib::ClassLoader 加载 .so
   c) 实例化插件, 调用 initialize()
   d) 之后通过 KinematicsBase* 多态调用 IK
```

```xml
<!-- plugin_description.xml 示例 (TRAC-IK) -->
<library path="libtrac_ik_kinematics_plugin">
  <class name="trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin"
         type="trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin"
         base_class_type="kinematics::KinematicsBase">
    <description>TRAC-IK solver for MoveIt2</description>
  </class>
</library>
```

### 各 IK 插件配置速查

```yaml
# ===== KDL (默认, 不推荐生产使用) =====
panda_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3

# ===== TRAC-IK (通用场景首选) =====
panda_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solve_type: Distance
  epsilon: 1e-5

# ===== pick-ik (现代优化, 支持额外约束) =====
panda_arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  mode: global
  position_threshold: 0.001
  orientation_threshold: 0.01
  center_joints_weight: 0.01

# ===== IKFast (极速闭式, 需预生成代码) =====
panda_arm:
  kinematics_solver: panda_ikfast_plugin/IKFastKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005

# ===== BioIK (多目标进化, 最灵活) =====
panda_arm:
  kinematics_solver: bio_ik/BioIKKinematicsPlugin
  kinematics_solver_timeout: 0.1
  mode: bio2
  center_joints_weight: 1.0
```

### 编写自定义 IK 插件框架

```cpp
// my_ik_plugin/src/my_ik_plugin.cpp
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace my_ik {

class MyIKPlugin : public kinematics::KinematicsBase {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node,
                    const moveit::core::RobotModel& robot_model,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::vector<std::string>& tip_frames,
                    double search_discretization) override {
        // 加载 Pinocchio 模型
        // 读取自定义参数 (从 ROS2 parameter server)
        // 初始化内部求解器
        storeValues(robot_model, group_name, base_frame,
                    tip_frames, search_discretization);
        return true;
    }

    bool getPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        std::vector<double>& solution,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
    ) const override {
        // 1. Pose msg → Eigen/SE3
        // 2. 调用你的 IK 算法
        // 3. 填充 solution
        // 4. 设置 error_code
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        return true;
    }

    bool searchPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        double timeout,
        std::vector<double>& solution,
        const IKCallbackFn& solution_callback,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
    ) const override {
        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::duration<double>(timeout);

        while (std::chrono::steady_clock::now() < deadline) {
            if (getPositionIK(ik_pose, ik_seed_state, solution,
                              error_code, options)) {
                // 通过回调验证 (碰撞检测等)
                if (!solution_callback || solution_callback(ik_pose, solution, error_code)) {
                    return true;
                }
            }
            // 随机扰动 seed, 重试
        }
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
        return false;
    }

    bool getPositionFK(/* ... */) const override {
        // Pinocchio FK
        return true;
    }
};

}  // namespace my_ik

// 注册插件
PLUGINLIB_EXPORT_CLASS(my_ik::MyIKPlugin, kinematics::KinematicsBase)
```

---

## M03.10 性能光谱与选型 ⭐⭐

### 完整性能对比表

| 求解器 | 平均时间 | 成功率 | 约束类型 | 冗余臂 | ROS2 | 代码规模 |
|--------|---------|--------|----------|--------|------|---------|
| **opw_kinematics** | ~0.1 us | 100% | 关节限位 | -- | ros2_opw | ~500行 |
| **IKFast** | ~1 us | 100% | 关节限位 | 部分 | moveit2 | 生成~5k行 |
| **ik_geo** | ~5 us | 100% | 关节限位 | -- | 无(2025) | ~500行 |
| **pick-ik local** | ~50 us | 95% | 任意代价 | 7-DOF | moveit2 | ~3k行 |
| **pick-ik global** | ~200 us | 98% | 任意代价 | 7-DOF | moveit2 | ~3k行 |
| **TRAC-IK** | ~500 us | 99.5% | 关节限位 | 7-DOF | moveit2 | ~2k行 |
| **KDL** | ~2 ms | 60% | 关节限位 | 7-DOF | 默认 | ~200行 |
| **BioIK** | ~5 ms | 99% | 任意代价 | 7-DOF+ | moveit2 | ~5k行 |

### 对数尺度速度谱

```
时间 (对数)          求解器                 适用场景
─────────────────────────────────────────────────────────
  0.1 us  ▓▓         opw_kinematics      高频伺服 (10kHz)
    1 us  ▓▓▓        IKFast              实时控制 (1kHz)
    5 us  ▓▓▓▓       ik_geo              快速闭式 (1kHz)
   50 us  ▓▓▓▓▓▓     pick-ik (local)     轨迹跟踪 (100Hz)
  200 us  ▓▓▓▓▓▓▓    pick-ik (global)    规划求解
  500 us  ▓▓▓▓▓▓▓▓   TRAC-IK            通用规划
    2 ms  ▓▓▓▓▓▓▓▓▓  KDL                仅教学用
    5 ms  ▓▓▓▓▓▓▓▓▓▓ BioIK              多目标复杂约束
─────────────────────────────────────────────────────────
         │           │           │
      可用于          可用于       仅规划阶段
    1kHz伺服       100Hz跟踪      (离线)
```

### 选型决策流程图

```
                      你的机器人?
                          │
          ┌───────────────┼────────────────┐
          ▼               ▼                ▼
    6-DOF 标准        7-DOF 冗余       非标准/特殊
   (OPW/球腕)       (Panda, iiwa)     (蛇形,人形)
          │               │                │
    ┌─────┤         ┌─────┤          ┌─────┤
    ▼     ▼         ▼     ▼          ▼     ▼
  高频?  通用?    有额外   通用?     BioIK  自定义
    │     │      约束?     │        (进化)  优化
    ▼     ▼       │       ▼
   opw  ik_geo    ▼     TRAC-IK
 (0.1us) (5us) pick-ik  (500us)
                (50us)
```

### 场景化选型指南

**场景 1: 高频伺服控制 (1-10 kHz 控制循环)**
- 必选: opw_kinematics (OPW 结构) 或 IKFast (6-DOF 通用)
- 理由: 控制周期 0.1-1 ms，IK 必须 <10 us
- 禁用: TRAC-IK, BioIK (太慢, 抢占实时线程)

**场景 2: MoveIt2 运动规划 (离线/准实时)**
- 首选: TRAC-IK (稳定、高成功率)
- 升级: pick-ik (需要额外约束时)
- 理由: 规划阶段容忍 ms 级延迟，但需要 >99% 成功率

**场景 3: 7-DOF 臂在狭窄空间操作**
- 首选: pick-ik (global mode) + 避碰 cost
- 备选: BioIK + collision cost
- 理由: 需要在 IK 阶段就考虑碰撞

**场景 4: 工业产线 (大批量、固定机型)**
- 首选: opw_kinematics (一次配参, 永久使用)
- 备选: IKFast (一次生成, 零维护)
- 理由: 确定性 + 可预测延迟最重要

**场景 5: 快速原型/研究**
- 首选: pick-ik (配置简单, 功能全)
- 备选: TRAC-IK (最成熟)
- 理由: 不想折腾代码生成或参数标定

> **Pitfall Box**: 不要只看"平均时间"——**最坏情况延迟** (worst-case latency) 对实时系统更关键。TRAC-IK 平均 500us，但偶尔达 5ms（SQP 线程慢收敛）。实时系统应设**硬超时** + **降级策略**（超时则用上一帧的解 + 阻尼插值）。

---

## 实战练习

### A 型 (动手实践) ⭐

**A1: opw_kinematics 基准测试**

为 UR5 配置 OPW 参数，生成 1000 个工作空间内随机目标位姿，统计：
- 每个目标的 8 组解中，有多少满足关节限位 $[-2\pi, 2\pi]$？
- 选最近当前构型的解时，关节最大跳变量 $\max_j |\Delta q_j|$ 的分布

**A2: 阻尼系数对比实验**

用 M03.3 的 Pinocchio Jacobian IK 代码求解 Panda 臂，对比：
- `lambda = 0` vs `lambda = 0.01` vs `lambda = 0.1` 的收敛曲线
- 在奇异构型附近（肘完全伸直 $q_4 \approx 0$）各自的稳定性

**A3: MoveIt2 三插件对比**

在 MoveIt2 中配置 KDL / TRAC-IK / pick-ik，对 100 个随机目标：
- 画出成功率柱状图
- 画出求解时间分布的 box plot
- 标注每个失败 case 对应的目标位姿特征（工作空间边界？奇异？）

**A4: 零空间可视化**

实现 M03.8 的零空间 IK，用 Panda 臂在 RViz 中可视化：
- 固定末端位姿，连续改变零空间参数 $\alpha \in [-1, 1]$
- 观察肘关节如何在不改变末端的情况下"绕圈"运动

### B 型 (源码精读) ⭐⭐

**B1: TRAC-IK 时序分析**

精读 `trac_ik_lib/src/trac_ik.cpp` 的 `CartToJnt()` 主函数，画完整时序图：
- 两线程的生命周期（创建 → 计算 → 终止）
- `atomic<bool>` 的所有读/写时刻及 memory order
- `mutex` 的 lock/unlock 范围
- 思考：改用 `memory_order_seq_cst` 对吞吐量的影响（估算 fence 成本）

**B2: pick-ik cost 机制**

阅读 pick-ik 的 `src/goal.cpp`，回答：
- cost function 如何注册？支持动态添加吗？
- L-BFGS 的梯度是数值差分还是解析？（提示：看 Pinocchio Jacobian 的使用）
- `memetic` 模式中 "elite" 个体如何保留？

**B3: ik_geo 子问题实现**

阅读 ik_geo 的 SP2 实现代码，写出完整数学推导（包括两圆求交的几何条件）。

### C 型 (思考题) ⭐⭐⭐

**C1: 多求解器竞速扩展**

TRAC-IK 用 2 个线程竞速。如果扩展到 4 个（NR + SQP + L-BFGS + 进化），分析：
- CPU 资源争抢（4 线程抢 4 核 vs 8 核的区别）
- 线程启动开销 (~10us) vs 边际收益
- 是否存在"diminishing returns"？最优线程数如何确定？

**C2: 双臂 IK**

为移动机械臂 (7-DOF 臂 + 3-DOF 底盘 = 10-DOF) 设计 IK 系统：
- 哪些方法可直接用？哪些需要修改？
- 如何处理底盘运动与臂运动的解耦/耦合？

**C3: pick-ik vs BioIK**

pick-ik 的 L-BFGS + memetic 和 BioIK 的 memetic evolutionary 本质区别：
- L-BFGS 是**梯度引导**的局部优化——需要光滑 cost 和解析梯度
- BioIK 是**无梯度**的全局搜索——适合非光滑/离散 cost
- pick-ik 更快的原因？（Pinocchio FK 快 + L-BFGS 收敛快 + C++17 效率）

---

## 章节总结

### 核心收获

| # | 要点 |
|---|------|
| 1 | IK 是"多对一映射"的逆问题——解可能不存在、有限个、或无穷多 |
| 2 | 解析解 (opw/IKFast/ik_geo) 适用于 6-DOF 标准结构——亚微秒级，确定性最强 |
| 3 | TRAC-IK 的竞速模式是 `std::thread + std::atomic` 的教科书案例 |
| 4 | pick-ik 代表 IK 的现代范式——优化框架 + Pinocchio 后端 + 任意约束 |
| 5 | 零空间投影是冗余臂的核心技术——主任务 + 子任务分层 |
| 6 | MoveIt2 pluginlib 机制允许运行时热切换 IK 求解器 |

### 1.5 周学习路线

| 天数 | 内容 | 时长 |
|------|------|------|
| Day 1 | M03.0-M03.1 前置自测 + 问题形式化 | 2h |
| Day 2 | M03.2 解析解理论 + OPW 推导 + opw 实操 | 3h |
| Day 3 | M03.3 Jacobian IK 推导 + Pinocchio 实现 | 3h |
| Day 4 | M03.4 TRAC-IK 源码精读 + 并发分析 | 3h |
| Day 5 | M03.5 IKFast 代码生成原理 + 实操 | 2h |
| Day 6 | M03.6 ik_geo 子问题推导 | 2h |
| Day 7 | M03.7 pick-ik L-BFGS + MoveIt2 配置 | 2h |
| Day 8 | M03.8 零空间 IK 理论 + 实现 | 3h |
| Day 9 | M03.9 MoveIt2 插件机制 + M03.10 选型 | 2h |
| Day 10 | 综合练习 + 基准测试实验 | 3h |

**总计: ~25 学时** (含练习)

### 下游衔接

```
M03 (IK 求解器)
  │
  ├──► M04 (碰撞检测) — IK 解的碰撞验证, searchPositionIK 回调
  ├──► M07 (OMPL采样规划) — IK 作为 goal state 采样器
  ├──► M08 (轨迹优化) — IK 约束嵌入 NLP
  ├──► M10 (时间参数化) — IK 解序列 → 时间最优轨迹
  └──► M14 (MoveIt2 MTC) — IK 插件在 Task Constructor 管线中的角色
```

### 关键参考文献

1. **Beeson, P. & Ames, B.** (2015). *TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics*. IEEE-RAS Humanoids Conference.
2. **Diankov, R.** (2010). *Automated Construction of Robotic Manipulation Programs*. PhD Thesis, Carnegie Mellon University. Chapter 4: IKFast.
3. **Elias, A. & Wen, J.T.** (2022). *Canonical Subproblems for Robot Inverse Kinematics*. ASME Journal of Mechanisms and Robotics, 14(2).
4. **Elias, A. & Wen, J.T.** (2025). *IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition*. Mechanism and Machine Theory, Volume 209, Article 105971 (扩展版).
5. **Brandstotter, M., Angerer, A., Hofbaur, M.** (2014). *An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist*. Austrian Robotics Workshop.
6. **Pieper, D.L.** (1968). *The Kinematics of Manipulators Under Computer Control*. PhD Thesis, Stanford University.
7. **Siciliano, B.** (1991). *Kinematic Control of Redundant Robot Manipulators: A Tutorial*. Journal of Intelligent and Robotic Systems, 3, 201-212.
8. **Starke, S. et al.** (2017). *Memetic Evolution for Generic Full-Body Inverse Kinematics in Robotics and Animation*. IEEE T-EC.

---

## M03.11 前沿方向：Differentiable IK 与 Learning-based IK ⭐⭐⭐⭐

前十节覆盖了从解析解到优化式 IK 的完整经典方法谱系。近年来，可微分优化和深度学习为 IK 问题带来了新的求解范式——它们不是替代传统方法，而是在特定场景下提供互补的能力。

### Differentiable IK——用自动微分求解 IK

传统数值 IK 依赖 Jacobian 伪逆或 L-BFGS，但这些方法对目标函数的选择和调参敏感。Differentiable IK 的核心思想是：将 IK 问题嵌入一个可微优化框架，通过自动微分（AD）直接计算精确梯度，避免 Jacobian 伪逆的数值问题。

**核心公式**：将 IK 视为隐式层（implicit layer）

$$q^* = \arg\min_q \|FK(q) - T_{\text{target}}\|^2 + \lambda R(q)$$

通过隐函数定理，可以求解 $\frac{\partial q^*}{\partial T_{\text{target}}}$——即目标位姿变化时最优关节角如何变化。这使得 IK 可以作为可微模块嵌入端到端的学习/优化管线中。

**与传统方法的关系**：

| 维度 | 传统 Jacobian IK | Differentiable IK |
|------|----------------|-------------------|
| 梯度来源 | 手工推导的 Jacobian $J(q)$ | AD 自动计算（CppAD/JAX/PyTorch） |
| 目标函数 | 固定为 $\|J\dot{q} - v\|^2$ | 任意可微目标（含碰撞代价、操作性指标等） |
| 求解器 | Newton-Raphson / DLS | 梯度下降 / L-BFGS / 任意一阶优化器 |
| 可嵌入学习管线 | 不可微 | 可微——反向传播穿过 IK 层 |
| 工具 | Pinocchio `computeFrameJacobian` | Pinocchio CppAD/CasADi, JAX, Theseus |

**Theseus**（Meta AI, 2022）是目前最成熟的可微优化库之一，直接支持将 IK 作为可微层使用。

### Learning-based IK——数据驱动的 IK

传统 IK 求解器基于精确的运动学模型 $FK(q)$。Learning-based IK 的动机是：

1. **模型不精确**：磨损、弹性变形、电缆拖拽导致实际 FK 与名义 FK 有偏差
2. **约束复杂**：考虑碰撞、操作性、关节极限等约束的 IK 求解耗时
3. **分布预测**：给定目标位姿，预测 IK 解的**分布**而非单一解

典型方法包括：

- **IKFlow**（Ames et al., 2022）：用 Normalizing Flows 学习 IK 解的条件分布 $p(q | T_{\text{target}})$，一次前向传播生成多样化的 IK 解
- **Neural IK**：训练神经网络直接回归 $q = f_\theta(T_{\text{target}})$，推理速度极快（~10 $\mu$s），但精度依赖训练数据覆盖率

> **本质洞察**：Learning-based IK **不是** 要替代传统 IK——它适合"需要快速生成多个候选解"的场景（如操作规划中的目标采样），而传统方法适合"需要高精度单一解"的场景（如实时伺服控制）。两者的关系是互补而非替代。

### 练习

1. ⭐⭐⭐ 用 Pinocchio CppAD 实现一个简单的 Differentiable IK：用 AD 自动计算 FK 误差对 $q$ 的梯度，然后用梯度下降求解。与手工 Jacobian 伪逆的结果对比。
2. ⭐⭐⭐⭐ 调研 Theseus 或 JAX-based Differentiable IK 的实现，分析其在 GPU 上的批量 IK 求解性能。

---

## 累积项目：IK 求解器模块

### 本章新增

```
mini-manip/
├── src/
│   ├── load_panda.cpp              ← M01
│   ├── dynamics_benchmark.cpp      ← M02
│   └── ik_solver.cpp               ← M03 新增：多种 IK 求解器统一接口
├── include/
│   ├── dynamics_interface.hpp      ← M02
│   └── ik_interface.hpp            ← M03 新增：IK 求解器抽象接口
├── config/
│   └── ik_plugins.yaml             ← M03 新增：MoveIt2 IK 插件配置
└── CMakeLists.txt                  ← M03 更新：添加 TRAC-IK / pick-ik 依赖
```

### 具体任务

1. 实现 `IKInterface` 抽象类，封装 `solve(T_target) -> q` 接口
2. 实现三个后端：Pinocchio DLS IK、TRAC-IK wrapper、pick-ik wrapper
3. 编写 benchmark：随机采样 1000 个可达目标，对比三个后端的成功率和平均耗时
4. （跨章综合题）结合 M01 的 FK 和 M02 的性能测量方法，验证 IK 解的 FK 还原精度

---

## 延伸阅读

| 资源 | 难度 | 说明 |
|------|:---:|------|
| Beeson & Ames (2015) "TRAC-IK" | ⭐⭐ | 双线程竞速 IK 原论文，IEEE-RAS Humanoids |
| Diankov (2010) "IKFast" PhD Thesis Ch4 | ⭐⭐⭐ | 符号消元代码生成的完整推导 |
| Elias & Wen (2022/2025) "ik_geo" | ⭐⭐⭐ | 几何子问题分解——通用闭式 IK 的现代方法 |
| Brandstotter et al. (2014) "OPW" | ⭐⭐ | 工业臂参数化闭式解 |
| Siciliano (1991) "Kinematic Control of Redundant Manipulators" | ⭐⭐⭐ | 零空间投影和优先级控制的经典教程 |
| Starke et al. (2017) "BioIK" | ⭐⭐⭐ | 进化算法 IK——无梯度全局搜索 |
| Pieper (1968) PhD Thesis | ⭐⭐⭐⭐ | 闭式 IK 存在性的奠基工作 |
| pick-ik GitHub + MoveIt2 文档 | ⭐⭐ | L-BFGS + Pinocchio 的现代 IK 实现 |
| Lynch & Park (2017) "Modern Robotics" Ch6 | ⭐⭐ | IK 的教科书级讲解 |

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| KDL IK 成功率极低（<50%） | Newton-Raphson 无随机重启，初始猜测陷入局部极值 | 1. 切换到 TRAC-IK（双线程竞速+随机重启） 2. 检查目标是否在工作空间内 3. 增大迭代次数不是根本解决方案 | M03.3-M03.4 |
| TRAC-IK 在某些目标附近极慢（>10 ms） | SQP 线程在奇异构型附近收敛慢 | 1. 打印 SQP 迭代次数 2. 检查目标是否靠近工作空间边界 3. 适当放宽容差 $\epsilon$ | M03.4 |
| IKFast 生成代码编译失败 | OpenRAVE 版本不兼容或机器人结构不满足代码生成条件 | 1. 确认机器人是 6-DOF 且满足 Pieper 条件 2. 使用 Docker 运行 OpenRAVE 3. 考虑改用 ik_geo（不依赖 OpenRAVE） | M03.5-M03.6 |
| pick-ik 返回的解违反关节限位 | 优化问题中关节限位约束权重太低 | 1. 检查 `kinematics.yaml` 中的 `joint_limit_weight` 2. 增大约束权重 3. 检查 URDF 中的关节限位值是否正确 | M03.7 |
| 零空间 IK 子任务与主任务冲突 | 零空间投影矩阵计算错误或优先级设置不当 | 1. 验证 $N = I - J^+ J$ 的幂等性（$N^2 = N$） 2. 打印主任务误差确认不被子任务干扰 3. 检查 Jacobian 坐标系是否一致 | M03.8 |

---

> **完成本章后，你应能**: 为任何机械臂选择最合适的 IK 求解器并理解其算法本质；在 MoveIt2 中配置和切换 IK 插件；实现带零空间优化的冗余 IK；具备编写自定义 IK 插件的工程能力。
