> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# D07 TDPA 与工程实现——时间域无源性方法、ROS2 集成与延迟场景验证

> **本章定位**：D05 建立了二端口理论框架，D06 给出了波变量的无源通信方案。本章介绍另一条技术路线——时间域无源性方法（TDPA, Time-Domain Passivity Approach）。TDPA 的核心思想与波变量不同：不改变信号表示，而是在线实时监测系统能量，一旦检测到能量创生就注入阻尼"消灭"多余能量。它是一种"事后纠正"而非"事先预防"的策略——更灵活，对各种延迟/丢包/抖动场景适应性更强。严格地说，TDPA 保证的是**被监测端口**的无源性，且要求控制器有权限修改力或速度/位置参考来耗散能量；未测量的能量通道、执行器饱和和接口缺失不在保证范围内。
>
> **适用范围**：TDPA 概念对网络化控制、多机器人系统、触觉接口均适用。
>
> **前置依赖**：D05（二端口网络/透明度）、D06（波变量/无源性）、F02.4（无源性基础）
>
> **下游章节**：D08（运动映射与遥操作数据采集）、D09（双臂 MoveIt2 系统集成）
>
> **建议用时**：2 周（15-25 小时）

---

## 前置自测 ⭐

> 📋 **答不出 >= 2 题 → 先回前置章节复习**

| 编号 | 问题 | 答不出时回顾 |
|:----:|------|------------|
| 1 | **端口无源性**：写出端口无源性的离散时间条件。$E_{obs}(k) < 0$ 意味着什么？ | F02.4 无源性 |
| 2 | **波变量的优势和局限**：波变量在常数时延下保证什么？在时变时延下有什么问题？ | D06 时变时延 |
| 3 | **Llewellyn 稳定因子**：$\eta(\omega) < 0$ 在某个频率意味着什么？ | D05 Llewellyn |
| 4 | **阻尼的作用**：在遥操作中增加阻尼如何影响 $\eta$？如何影响透明度？ | D05 trade-off |
| 5 | **ros2_control 架构**：ros2_control 的 ControllerInterface 基类有哪些核心方法？ | M12 ros2_control |

---

## 本章目标

学完本章后，你应该能够：

1. **掌握** Passivity Observer (PO) 的离散实现——在本文的输出端口约定下，$E_{obs}(k) = E_{obs}(k-1) - f(k) \cdot v(k) \cdot \Delta T$
2. **推导** Passivity Controller (PC) 的阻尼量 $\alpha(k) = -E_{obs}(k) / (v(k)^2 \cdot \Delta T)$
3. **理解**串联 PC 和并联 PC 的对偶关系，以及何时切换
4. **理解**零速发散问题及 Ryu 2005 的 reference energy following 修复
5. **能在 ROS2 中实现** TDPA 节点：读取力/速度→PO→PC→修正力/速度
6. **对比** TDPA 与波变量方法的工程优劣，理解工业系统如何组合使用两者

---

## D7.1 TDPA 的核心思想——"事后纠正"vs"事先预防" ⭐⭐

### D7.1.1 与波变量的哲学差异

D06 的波变量方法和本章 TDPA 代表了保证遥操作稳定性的两种根本不同的哲学：

| 维度 | 波变量 (D06) | TDPA (本章) |
|------|-------------|------------|
| **策略** | 事先预防——改变信号表示使通道天然无源 | 事后纠正——实时监测能量，异常时注入阻尼 |
| **对通信的要求** | 必须在波域传输 | 可以在任意域传输（力/速度/位置均可） |
| **先验信息** | 需要设定波阻抗 $b$ | 仅需实时 $(f, v)$ 测量 |
| **透明度特征** | 持续的波阻抗粘滞感 | 正常时零干预；异常时短暂阻尼脉冲 |
| **适用场景** | 常数/缓变时延 | 复杂时延/丢包/抖动下的端口能量安全网；需实时端口测量与可耗散执行接口 |
| **历史** | Anderson-Spong 1989 | Hannaford-Ryu 2002 |

> **跨领域类比**：波变量像"防火建筑"——用阻燃材料建造，使火灾无法发生。TDPA 像"消防系统"——允许用普通材料建造，但装了烟雾报警器（PO）和自动喷淋（PC），一旦检测到火灾就灭火。防火建筑日常使用受限（波阻抗粘滞），但永远不着火；消防系统日常自由（零干预），但灭火时有短暂影响（阻尼脉冲）。

### D7.1.2 TDPA 的历史与动机

Hannaford 和 Ryu (2002) 提出 TDPA 的三个动机：

**动机 1**：波变量需要设定波阻抗 $b$——$b$ 的选择影响透明度，且最优 $b$ 依赖未知的环境刚度。TDPA 不需要任何先验参数。

**动机 2**：波变量在正常操作（无延迟异常）时也引入粘滞感——"正常时不应该有代价"。TDPA 正常时完全不介入。

**动机 3**：实际网络（UDP/WiFi/5G）的延迟特性复杂（抖动、突发丢包），波变量的时变延迟修复方案需要额外先验信息。TDPA 不需要显式建模延迟曲线，而是根据端口能量赤字介入；它不是“任意网络条件下无条件稳定”的替代品。

**TDPA 的核心承诺**：在端口变量测量可靠、PC 输出能被执行器实际执行的前提下，正常操作时系统完全透明（PO 的能量预算未被透支，PC 不介入）；只有当被监测端口即将违反无源性时，PC 才注入最小必要耗散。

### D7.1.3 科研发展脉络

| 年份 | 论文 | 核心贡献 |
|------|------|---------|
| 2002 | Hannaford-Ryu | **PO/PC 首次提出**；时域在线无源性保证 |
| 2004 | Ryu-Kwon-Hannaford | **TDPA 正式框架**；不需知道时延大小；硬墙 150 kN/m 稳定 |
| 2005 | Ryu-Preusche-Hannaford-Hirzinger | 解决低速 $\alpha$ 发散；**参考能量跟踪** |
| 2006 | Diolaiti-Niemeyer-Barbagli-Salisbury | 计算延迟+量化+库仑摩擦联合影响精细化 |
| 2011 | Franken-Stramigioli-Misra-Secchi-Macchelli | **双层架构**：上层透明+下层能量罐保证无源 |
| 2019 | Panzirsch-Artigas-Ryu-Ferre et al. | **TDPA N-port 扩展**；多边遥操作 |

---

## D7.2 Passivity Observer (PO)——在线能量监测 ⭐⭐

### D7.2.1 离散时间能量观测器

**定义**：本文采用输出端口约定：$f(k)v(k) > 0$ 表示 TDPA 保护的系统正通过该端口向外注入能量。PO 维护的是可用的无源性能量预算，因此每个控制周期从预算中扣除向外输出的能量：

$$\boxed{E_{obs}(k) = E_{obs}(k-1) - f(k) \cdot v(k) \cdot \Delta T}$$

$$E_{obs}(0) = 0$$

其中 $f(k)$ 是端口力，$v(k)$ 是端口速度，$\Delta T$ 是采样周期（典型 1 ms）。

**被动条件**：$E_{obs}(k) \geq 0, \; \forall k$

如果你的实现使用的是输入端口被动符号（$fv > 0$ 表示能量流入被保护系统），只需把端口力或速度取反，或等价地使用 $E_{obs}(k)=E_{obs}(k-1)+fv\Delta T$。关键是 PO、PC 和诊断必须使用同一套端口方向。

**物理含义**：

| $E_{obs}$ 状态 | 含义 | 系统行为 |
|---------------|------|---------|
| $E_{obs} > 0$ | 预算仍有余量 | 被动——系统还没有透支可用能量 |
| $E_{obs} = 0$ | 输出能量刚好用完预算 | 边界——不能再无代价向外输出能量 |
| $E_{obs} < 0$ | 向外输出超过预算 | **违反被动性——系统透支/产生了能量！** |

**为什么这个简单的累加就能检测被动性违反？**

回顾 F02.4 的无源性定义：系统向外输出的累计能量不能超过初始储能和外部输入。若把当前端口的 $f v$ 定义为向外输出功率，离散化后就是 $V(0)-\sum_{i=0}^{k} f(i)v(i)\Delta T \geq 0$。当 $V(0)=0$ 或被并入 $E_{obs}(0)$ 时，这正是 $E_{obs}(k) \geq 0$。

PO 就是无源性定义的**直接离散实现**——没有任何近似，没有任何线性化，对非线性系统同样有效。

### D7.2.2 PO 的工程实现细节

```python
class PassivityObserver:
    """离散时间 Passivity Observer"""
    def __init__(self, dt, E_init=0.0):
        self.dt = dt
        self.E_obs = E_init  # 初始储能
        self.history = []    # 记录历史用于诊断
    
    def update(self, f, v):
        """
        更新能量观测器
        Args:
            f: 输出端口力 (N)
            v: 输出端口速度 (m/s)，f*v > 0 表示向外注入能量
        Returns:
            E_obs: 当前累积能量 (J)
            is_passive: 是否满足被动性
        """
        power_out = f * v  # 向外输出的瞬时功率 (W)
        self.E_obs -= power_out * self.dt
        self.history.append({
            'E_obs': self.E_obs,
            'power_out': power_out,
            'f': f, 'v': v
        })
        return self.E_obs, (self.E_obs >= 0)
```

**工程注意事项**：

| 注意事项 | 说明 | 后果 |
|---------|------|------|
| **符号约定** | 本文使用输出端口符号（$fv > 0$ = 向外注入能量） | 符号错→$E_{obs}$ 方向反→PC 在不该介入时介入 |
| **初始化** | $E_{obs}(0) = 0$ 假设零初始储能 | 有初始储能时设正值避免误报 |
| **数值精度** | 长时间累积误差 | 用 double 精度；考虑定期重置 |
| **采样频率** | $\Delta T$ 足够小使功率采样准确 | 典型 1 kHz；50 Hz 下 PO 分辨率不足 |

### D7.2.3 PO 的多端口放置策略

```
PO 放置位置选择
══════════════════════════════════════════

方案 A: 每个端口独立 PO（推荐）
  PO_master: E_m(k) -= f_m(k)*v_m(k)*dt
  PO_slave:  E_s(k) -= f_s(k)*v_s(k)*dt
  → 分别监测，分别控制

方案 B: 通信通道两端 PO
  监测通信通道的能量守恒
  → 专门检测延迟/丢包引起的能量创生

方案 C: 系统级单个 PO
  E(k) -= [f_m*v_m + f_s*v_s](k)*dt
  → 全局监测

工业最佳实践: 方案 A（两端独立监测和控制）
```

> **反事实推理**：如果只在 master 端放 PO 而 slave 端不放会怎样？slave 控制器本身的 bug 可能在 slave 端局部产生能量——这个异常需要经过通信延迟 $T$ 才能传到 master 端的 PO。在延迟为 100 ms 的系统中，slave 端的不稳定可以在 100 ms 内发展到破坏性程度。因此**两端都放 PO** 是必要的。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：PO 中力和速度的符号约定不一致
   错误做法：f 用力传感器直接测量值，v 用编码器速度，不检查符号
   现象：E_obs 持续单调增大或持续单调减小，完全不反映真实能量
   根本原因：力传感器测量的可能是"施加在传感器上的力"，
            与本文输出端口功率约定（向外注入为正）相反
   正确做法：确认 f*v > 0 对应"系统向外注入能量"
   自检方法：自由空间中快速移动 master → E_obs 不应持续向负方向漂移
```

```
💡 概念误区：认为"E_obs 持续增大说明系统有问题"
   新手想法："E_obs 越来越大 → 系统在无限累积能量 → 要爆炸"
   实际上：E_obs 持续增大是正常的——只要操作者持续输入能量
          （移动 master），E_obs 就会增大
          E_obs 是累积量，不是瞬时量
          只有 E_obs < 0 才说明能量预算被透支
```

### 练习

1. **[A 型 -- PO 实现]** 实现 1-DOF PO。给定仿真遥操作数据 $(f(t), v(t))$，计算 $E_{obs}(k)$。在什么操作下 $E_{obs}$ 可能变负？（提示：尝试加入延迟）
2. **[思考题]** 如果系统有初始弹簧储能 $V(0) = \frac{1}{2}Kx_0^2 > 0$，PO 初始化为 $E_{obs}(0) = 0$ 会导致什么误判？

---

## D7.3 Passivity Controller (PC)——自适应阻尼注入 ⭐⭐⭐

### D7.3.1 PO/PC 离散实现的完整推导 ⭐⭐⭐

当 PO 检测到 $E_{obs} < 0$（能量创生），PC 注入阻尼消耗多余能量。这里给出从连续时间到离散时间的完整推导过程。

**从连续到离散——不是简单替换**：

连续时间 PO（输出端口约定）：$E_{obs}(t) = E_{obs}(0) - \int_0^t f(\tau) v(\tau) d\tau$

离散化（本文输出端口约定）：$E_{obs}(k) = E_{obs}(0) - \sum_{i=0}^{k} f(i) v(i) \Delta T = E_{obs}(k-1) - f(k) v(k) \Delta T$

**关键问题**：连续时间中，$E_{obs}(t) \geq 0$ 保证任意时刻的无源性。但离散化后，$E_{obs}(k) \geq 0$ 只保证采样时刻的无源性——采样间隔内可能存在短暂的无源性违反。

**解决方案**（Ryu et al. 2004）：使用**前向差分**而非后向差分：

$$E_{obs}(k) = E_{obs}(k-1) - f(k) \cdot v(k) \cdot \Delta T$$

在 ZOH 条件下，$f(k)$ 和 $v(k)$ 在 $[k\Delta T, (k+1)\Delta T)$ 内恒定，因此积分精确等于 $f(k)v(k)\Delta T$。这使得离散 PO 是连续 PO 的**精确离散化**（不是近似）。

**串联 PC 控制律**：

$$f_{out}(k) = f(k) - \alpha(k) \cdot v(k)$$

$\alpha(k) \geq 0$ 是自适应阻尼系数。负号来自端口约定：$f v$ 为向外注入功率，阻尼力必须与速度反向，才能减少向外输出功率。

耗散功率 $P_{diss} = \alpha v^2 \geq 0$（始终耗能——$\alpha \geq 0$ 且 $v^2 \geq 0$）。

**$\alpha(k)$ 的推导**：

**Step 1**：修正后端口功率：

$$P_{out}^{corrected}(k) = f_{out}(k) \cdot v(k) = [f(k) - \alpha(k) v(k)] v(k) = f(k)v(k) - \alpha(k)v(k)^2$$

**Step 2**：修正后 PO 更新：

$$E_{obs}^{corrected}(k) = E_{obs}(k-1) - [f(k)v(k) - \alpha(k)v(k)^2] \Delta T$$

由于 $E_{obs}(k) = E_{obs}(k-1) - f(k)v(k)\Delta T$（未修正），所以：

$$E_{obs}^{corrected}(k) = E_{obs}(k) + \alpha(k) v(k)^2 \Delta T$$

**Step 3**：令修正后 $E_{obs}$ 恰好归零（**最小阻尼原则**）：

$$E_{obs}(k) + \alpha(k) v(k)^2 \Delta T = 0$$

$$\boxed{\alpha(k) = \frac{-E_{obs}(k)}{v(k)^2 \cdot \Delta T} \quad \text{when } E_{obs}(k) < 0}$$

$$\alpha(k) = 0 \quad \text{when } E_{obs}(k) \geq 0$$

**为什么是"最小阻尼"**：令 $E_{obs}^{corrected} = 0$ 而非 $> 0$ 意味着 PC 只消耗恰好等于"能量赤字"的能量——不多不少。如果令 $E_{obs}^{corrected} = E_{positive} > 0$，则 $\alpha$ 更大，引入不必要的阻尼降低透明度。

**物理含义**：PC 加入恰好等于"透支能量"的阻尼耗散。$E_{obs} \geq 0$ 时完全不介入——**零干预透明度**。

### D7.3.1.1 串联与并联 PC 的物理直觉 ⭐⭐

**串联 PC ($\alpha$) 的物理图像**：

想象操作者在推一辆购物车。正常时车自由滑动（零干预）。当 PO 检测到"能量异常"时，串联 PC 等效于在车轮上施加刹车——**阻尼力与速度方向相反，减慢运动以消耗多余能量**。

$$f_{out} = f - \alpha v \quad \text{（$\alpha > 0$ 时，附加力与速度反向→耗散输出能量）}$$

串联 PC 适合**运动状态**——有速度时，阻尼可以有效耗散能量。

**并联 PC ($\beta$) 的物理图像**：

当车被推到墙上（$v \approx 0$ 但 $f$ 很大），轮刹车无效（$\alpha v^2 \approx 0$，无法耗散能量）。此时需要另一种策略——并联 PC 等效于**让墙稍微退让（修改速度）**，使力和速度产生功率耗散。

$$v_{out} = v - \beta f \quad \text{（$\beta > 0$ 时，附加速度与力反向→减少向外输出功率）}$$

并联 PC 适合**静止/挤压状态**——有力但无速度时，修改速度以耗散能量。

> **跨领域类比**：串联/并联 PC 的对偶关系类似于电路中的串联/并联电阻。串联电阻在有电流时才耗能（$P = I^2R$），对应串联 PC 在有速度时耗能；并联电阻（与电压源并联）在有电压时才耗能（$P = V^2/R$），对应并联 PC 在有力时耗能。两者是功率耗散的两种对偶形式。

### D7.3.2 并联 PC（力型）——对偶实现

**动机**：当 $v \to 0$（如硬墙挤压），$\alpha = -E_{obs}/(v^2 \Delta T) \to \infty$——阻尼发散！

**并联 PC**：改为修改速度而非力：

$$v_{out}(k) = v(k) - \beta(k) \cdot f(k)$$

$$\beta(k) = \frac{-E_{obs}(k)}{f(k)^2 \cdot \Delta T} \quad \text{when } E_{obs}(k) < 0$$

当 $f \to 0$（自由空间），$\beta \to \infty$——并联 PC 也有发散问题，但**在不同的操作条件下**。

### D7.3.3 对偶切换策略 ⭐⭐

串联和并联 PC 是**对偶的**——各自在不同操作条件下有效：

| 操作状态 | 速度 $v$ | 力 $f$ | 应使用 | 理由 |
|---------|---------|--------|--------|------|
| 自由运动 | 大 | $\approx 0$ | 串联 ($\alpha$) | $\alpha = -E/(v^2\Delta T)$ 有限 |
| 硬墙挤压 | $\approx 0$ | 大 | 并联 ($\beta$) | $\beta = -E/(f^2\Delta T)$ 有限 |
| 过渡区 | 中 | 中 | 任一均可 | 两者都有限 |

**阈值切换实现**：

```python
def passivity_controller(E_obs, f, v, dt, v_thresh=0.01):
    """
    串联/并联自适应切换 PC
    """
    if E_obs >= 0:
        return f, v, 0.0  # 不介入
    
    if abs(v) > v_thresh:
        # 串联 PC: 在力上加阻尼（阻尼力与速度反向）
        alpha = min(-E_obs / (v**2 * dt + 1e-10), 1000.0)  # 限幅
        f_out = f - alpha * v
        return f_out, v, alpha
    elif abs(f) > 1e-3:
        # 并联 PC: 修改速度（附加速度与力反向）
        beta = min(-E_obs / (f**2 * dt + 1e-10), 100.0)
        v_out = v - beta * f
        return f, v_out, beta
    else:
        # f 和 v 都很小——不介入（避免数值问题）
        return f, v, 0.0
```

**$v_{thresh}$ 的选择**：典型 0.005-0.05 m/s。太大→过早切换到并联（牺牲力精度）；太小→串联 PC 可能产生过大 $\alpha$。

**低速大力硬接触的工程规则**：硬墙、夹紧、插孔卡住这类场景常见 $|v|\approx 0$ 但 $|f|$ 很大。此时继续用串联 PC 会得到很大的 $\alpha$，表现为力反馈“锁死”或瞬时力突变。正确做法是切到并联 PC，用 $\beta$ 修改速度/导纳参考：

$$v_{out}=v-\beta f,\quad E_{pc}=\beta f^2\Delta T$$

注意并联 PC 不是“在力上再加一个补偿项”。如果当前控制器只有力命令接口而没有速度、位置参考或导纳端口，就不能严格实现并联 PC；这时应把 TDPA 放到拥有速度/参考输出的层，或在架构上改成能量罐/导纳外环。

### D7.3.4 Reference Energy Following——完整算法（Ryu 2005）⭐⭐⭐

**问题的深层分析**：

即使有串联/并联切换，PC 在正常操作中仍可能过度介入。原因：正常缓慢操作中，$f \cdot v$ 的波动可能使 $E_{obs}$ 暂时变为小负值——这不是真正的"能量创生"，而是正常操作的能量波动。但严格的 $E_{obs} \geq 0$ 把这种无害波动也当异常处理。

**Ryu 2005 的核心算法——Reference Energy Tracking**：

**Step 1**：定义"正常操作"的参考能量边界

工程实现中不要直接令 $E_{ref}(k)=E_{obs}(k)+E_{margin}$，否则 $\Delta E=E_{obs}-E_{ref}$ 会恒为负，PC 将持续介入。更稳妥的简化实现是把允许透支边界设为独立阈值：

$$E_{ref}(k) = -E_{margin}(k)$$

$E_{ref}$ 表示 $E_{obs}$ 允许短暂下探的下界；只有 $E_{obs}$ 低于该边界时，PC 才开始耗散透支能量。

**Step 2**：PC 仅在 $E_{obs}$ 偏离 $E_{ref}$ 超过容忍范围时才介入

$$\Delta E(k) = E_{obs}(k) - E_{ref}(k)$$

$$\alpha(k) = \begin{cases} \frac{-\Delta E(k)}{v(k)^2 \cdot \Delta T + \epsilon} & \text{if } \Delta E(k) < 0 \text{ and } |v| > v_{thresh} \\ 0 & \text{otherwise} \end{cases}$$

**Step 3**：$E_{ref}$ 的自适应更新——Ryu 2005 的关键创新

标准方案中 $E_{margin}$ 是固定的。Ryu 2005 提出让 $E_{margin}$ 自适应：

$$E_{margin}(k) = \gamma \cdot \bar{P}(k) \cdot T_{window}$$

其中 $\bar{P}(k) = \frac{1}{N}\sum_{i=k-N}^{k} |f(i)v(i)|$ 是最近 $N$ 步的平均绝对功率，$T_{window}$ 是窗口时间，$\gamma$ 是安全系数（典型 0.1-0.5）。

**物理含义**：剧烈操作（高功率）时允许更大的能量波动；安静操作（低功率）时收紧容忍范围。

**完整实现**：

```python
class ReferenceEnergyTDPA:
    """Ryu 2005 Reference Energy Following 完整实现"""
    
    def __init__(self, dt, E_margin_base=0.01, gamma=0.2,
                 window_size=100, v_thresh=0.01, alpha_max=500.0):
        self.dt = dt
        self.E_margin_base = E_margin_base
        self.gamma = gamma
        self.v_thresh = v_thresh
        self.alpha_max = alpha_max
        
        # PO 状态
        self.E_obs = 0.0
        self.E_ref = 0.0
        
        # 自适应 E_margin
        self.power_history = np.zeros(window_size)
        self.power_idx = 0
        self.E_margin = E_margin_base
        
        # 统计
        self.n_interventions = 0
        self.total_energy_dissipated = 0.0
    
    def update(self, f, v):
        """完整的 PO + PC + Reference Energy 更新"""
        
        # === Step 1: PO 更新 ===
        # 本文约定 power_out = f*v > 0 表示系统向外注入能量。
        power_out = f * v
        self.E_obs -= power_out * self.dt
        
        # === Step 2: 自适应 E_margin ===
        self.power_history[self.power_idx] = abs(power_out)
        self.power_idx = (self.power_idx + 1) % len(self.power_history)
        avg_power = np.mean(self.power_history)
        self.E_margin = self.E_margin_base + self.gamma * avg_power * self.dt * len(self.power_history)
        
        # === Step 3: Reference Energy Following ===
        # 这里用一个工程简化版: 只在 E_obs 低于 -E_margin 时介入。
        # 不要令 E_ref = E_obs + margin，否则 delta_E 恒为负，PC 会一直介入。
        self.E_ref = -self.E_margin
        
        # === Step 4: PC 计算 ===
        delta_E = min(0.0, self.E_obs - self.E_ref)
        alpha = 0.0
        beta = 0.0
        f_out = f
        v_out = v
        
        if delta_E < 0:
            self.n_interventions += 1
            if abs(v) > self.v_thresh:
                alpha = min(-delta_E / (v**2 * self.dt + 1e-10), self.alpha_max)
                f_out = f - alpha * v
            elif abs(f) > 1e-3:
                beta = min(-delta_E / (f**2 * self.dt + 1e-10), 100.0)
                v_out = v - beta * f
        
        # === Step 5: 能量记账 ===
        energy_dissipated = (alpha * v**2 + beta * f**2) * self.dt
        self.total_energy_dissipated += energy_dissipated
        self.E_obs += energy_dissipated  # PC 耗散归还 E_obs
        
        return f_out, v_out, alpha, beta, self.E_obs, self.E_ref
```

| $E_{margin}$ 值 | 效果 | 适用场景 |
|----------------|------|---------|
| $0$ | 退化为标准 TDPA（最严格） | 安全关键（手术机器人） |
| $0.01$ J | 允许微小波动 | 一般遥操作 |
| $0.1$ J | 允许较大波动 | 高透明度需求 |
| $1.0$ J | 几乎不介入 | 非安全关键场景 |
| **自适应** | **操作强度比例** | **推荐——兼顾安全和透明** |

> **本质洞察**：$E_{margin}$ 本质上是一个**透明度-安全性的旋钮**。$E_{margin} = 0$ 是最安全但最不透明的设置（任何能量波动都被消灭）；$E_{margin}$ 越大越透明但越不安全（允许更大的能量波动）。工程师必须根据应用场景选择合适的折中点。Ryu 2005 的自适应方案是工程最优解——让旋钮根据操作状态自动调节。

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：alpha 除零保护不充分
   错误做法：alpha = -E_obs / (v**2 * dt) 不加任何保护
   现象：v ≈ 0 时 alpha → inf → f_out → inf → 系统飞出
   根本原因：数值除零
   正确做法：
     (1) alpha = -E_obs / (v**2 * dt + epsilon)，epsilon = 1e-10
     (2) 加上限：alpha = min(alpha, alpha_max)
     (3) 使用串联/并联切换避免在 v ≈ 0 时用串联
```

```
💡 概念误区：认为"TDPA 只在延迟场景下有用"
   新手想法："没有延迟 → 不需要 TDPA"
   实际上：TDPA 对以下非延迟场景也有用：
          (a) ZOH 离散化引起的被动性违反 → TDPA 是最后安全网
          (b) 传感器噪声注入的虚假能量 → PO 检测后 PC 消除
          (c) 控制器 bug 导致的意外能量产生 → TDPA 防止失控
   正确理解：TDPA 是系统级安全层，不仅仅是延迟补偿工具
```

### 练习

1. **[A 型 -- TDPA 完整实现]** 在 Python 中实现 1-DOF TDPA 遥操作：master + 100 ms 时延 + slave + 硬墙（$K=10000$ N/m）。对比 (a) 无保护 (b) 波变量 (c) TDPA 的稳定性和透明度。绘制 $E_{obs}(t)$ 和 $\alpha(t)$
2. **[A 型 -- Reference Energy]** 在上一练习中，对比 $E_{margin} = 0, 0.01, 0.1$ 三种设置。记录 PC 介入次数和透明度指标（$Z_{to}$ 与 $Z_e$ 的偏差）
3. **[思考题]** $E_{margin}$ 设得太大会怎样？从能量角度分析：系统最多能"借"多少能量而不导致可感知的不稳定？提示：人手的感知阈值约 0.5 N 力变化

---

## D7.4 TDPA vs 波变量——系统性工程对比 ⭐⭐

### D7.4.1 全面对比表

| 特性 | 波变量 (D06) | TDPA (本章) |
|------|-------------|------------|
| 无源保证 | 常数时延严格；时变需修正 | 被监测端口严格；复杂时延/丢包/抖动通过 PO/PC 在线耗散，失效条件需显式检查 |
| 先验信息 | 需设定 $b$（波阻抗） | 仅需 $(f,v)$ 实时测量 |
| 透明度（正常时） | 中等（持续粘滞感） | 高（零干预） |
| 失真模式 | 波反射振铃（连续性失真） | 偶发阻尼脉冲（离散性失真） |
| 实现复杂度 | 中（变换+反变换+wave integral） | 低（累加+比较+乘法） |
| 位置漂移 | 有（需 wave integral 修复） | 无（可直接传位置） |
| 对传感器要求 | 力+速度 | 力+速度（相同） |
| 工业采用 | DLR ROKVISS/KONTUR-2 | 触觉设备、dVRK 安全层 |

### D7.4.2 工业系统通常组合使用

> **本质洞察**：波变量和 TDPA 不是互斥的竞争方案——它们是互补的。**波变量提供结构性的无源性保证（通信通道），TDPA 提供运行时端口能量安全网。** 工业系统组合使用：

```
工业组合架构
══════════════════════════════════════════

Master ──→ 波变量编码 ──→ 延迟通道 ──→ 波变量解码 ──→ Slave
   ↑                                                    ↓
   │         ┌──────── TDPA 安全层 ────────┐            │
   │         │  PO(master) + PC(master)    │            │
   │         │  PO(slave)  + PC(slave)     │            │
   └─────────│  系统级能量监测              │────────────┘
             └─────────────────────────────┘

波变量: 结构性保证——通信通道无源
TDPA:   运行时保证——捕获被监测端口上的能量异常，并通过可用接口耗散
```

> **跨领域类比**：这类似于飞机的安全架构——自动驾驶（结构性安全，对应波变量）+ 飞行包线保护（运行时安全网，对应 TDPA）。两者独立工作，互为备份。

---

## D7.5 ROS2 中的 TDPA 工程骨架 ⭐⭐

### D7.5.1 ROS2 节点架构——Publisher/Subscriber/Timer 设计

在给出 ros2_control 接口骨架之前，先理解 TDPA 在 ROS2 中的节点架构。下面的独立节点示例展示 PO/PC 数据流，可以直接迁移到非实时原型；真机实时控制仍应放进 ros2_control 或硬实时控制循环。

TDPA 的端口功率必须由**同一端口、同一坐标系、同一时刻**的共轭变量计算。笛卡尔端口用 TCP wrench 与 TCP twist：

$$P = W_{tcp}^{T}V_{tcp}=F^Tv+\tau^T\omega$$

二者必须先变换到同一个 frame（例如 `world` 或同一个 `LOCAL_WORLD_ALIGNED` TCP frame），并且时间戳必须足够接近；异步采样时要丢弃 stale 数据或做同步插值。关节端口则用同一关节顺序下的 $\tau^T\dot{q}$。不要把 `/ft_sensor/wrench.force.z` 直接乘 `/joint_states.velocity[0]`；那不是同一端口的功率，PO 会得到没有物理意义的能量。

```cpp
// tdpa_node.hpp — 独立 ROS2 节点版本（适用于非 ros2_control 系统）
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class TDPANode : public rclcpp::Node {
public:
    TDPANode() : Node("tdpa_safety_layer") {
        // === 参数声明（支持动态调整）===
        this->declare_parameter("dt", 0.001);
        this->declare_parameter("v_threshold", 0.01);
        this->declare_parameter("alpha_max", 500.0);
        this->declare_parameter("E_margin", 0.01);
        this->declare_parameter("gamma_adaptive", 0.2);
        
        dt_ = this->get_parameter("dt").as_double();
        v_thresh_ = this->get_parameter("v_threshold").as_double();
        alpha_max_ = this->get_parameter("alpha_max").as_double();
        E_margin_ = this->get_parameter("E_margin").as_double();
        
        // === Subscriber: 同一 TCP 端口的 wrench 与 twist ===
        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/tcp_wrench", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                wrench_ = msg->wrench;
                wrench_frame_ = msg->header.frame_id;
                last_wrench_time_ = msg->header.stamp;
            });
        
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/tcp_twist", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                twist_ = msg->twist;
                twist_frame_ = msg->header.frame_id;
                last_twist_time_ = msg->header.stamp;
            });
        
        // === Publisher: 修正后的力指令和诊断 ===
        force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/tdpa/corrected_wrench", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/tdpa/corrected_velocity", 10);
        diag_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/tdpa/diagnostics", 10);
        
        // === Timer: 1 kHz 实时循环 ===
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(dt_ * 1e6)),
            std::bind(&TDPANode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(),
            "TDPA node started: dt=%.3fms, E_margin=%.3f J", dt_*1000, E_margin_);
    }

private:
    void control_loop() {
        if (wrench_frame_.empty() || twist_frame_.empty() ||
            wrench_frame_ != twist_frame_) {
            return;  // 生产代码应 TF 变换到统一 frame；示例中保守跳过。
        }
        if (last_wrench_time_.nanoseconds() == 0 ||
            last_twist_time_.nanoseconds() == 0) {
            return;
        }
        const double stamp_skew =
            std::abs((last_wrench_time_ - last_twist_time_).seconds());
        if (stamp_skew > 2.0 * dt_) {
            return;  // 异步数据过旧时不做能量记账，避免 W^T V 失去端口意义。
        }

        // === PO: 能量观测 ===
        // power_out > 0 表示系统通过输出端口向外注入能量。
        double power_out =
            wrench_.force.x * twist_.linear.x +
            wrench_.force.y * twist_.linear.y +
            wrench_.force.z * twist_.linear.z +
            wrench_.torque.x * twist_.angular.x +
            wrench_.torque.y * twist_.angular.y +
            wrench_.torque.z * twist_.angular.z;
        E_obs_ -= power_out * dt_;
        
        // === 自适应 E_margin (Ryu 2005) ===
        power_buffer_[buf_idx_] = std::abs(power_out);
        buf_idx_ = (buf_idx_ + 1) % BUFFER_SIZE;
        double avg_power = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) avg_power += power_buffer_[i];
        avg_power /= BUFFER_SIZE;
        double E_margin_adaptive = E_margin_ + 0.2 * avg_power * dt_ * BUFFER_SIZE;
        
        // === Reference Energy Following ===
        // 工程简化版: 只在 E_obs_ 低于 -E_margin_adaptive 时介入。
        // 不能令 E_ref = E_obs_ + margin，否则 delta_E 恒为负，PC 会一直介入。
        double E_ref = -E_margin_adaptive;
        double delta_E = std::min(0.0, E_obs_ - E_ref);
        
        // === PC: 串联/并联切换 ===
        auto wrench_corrected = wrench_;
        auto twist_corrected = twist_;
        double alpha = 0.0;
        double beta = 0.0;
        double pc_dissipated = 0.0;
        double twist_norm2 =
            twist_.linear.x * twist_.linear.x +
            twist_.linear.y * twist_.linear.y +
            twist_.linear.z * twist_.linear.z +
            twist_.angular.x * twist_.angular.x +
            twist_.angular.y * twist_.angular.y +
            twist_.angular.z * twist_.angular.z;
        double wrench_norm2 =
            wrench_.force.x * wrench_.force.x +
            wrench_.force.y * wrench_.force.y +
            wrench_.force.z * wrench_.force.z +
            wrench_.torque.x * wrench_.torque.x +
            wrench_.torque.y * wrench_.torque.y +
            wrench_.torque.z * wrench_.torque.z;
        
        if (delta_E < 0.0) {
            pc_count_++;
            if (twist_norm2 > v_thresh_ * v_thresh_) {
                alpha = std::min(-delta_E / (twist_norm2 * dt_ + 1e-10),
                                 alpha_max_);
                wrench_corrected.force.x -= alpha * twist_.linear.x;
                wrench_corrected.force.y -= alpha * twist_.linear.y;
                wrench_corrected.force.z -= alpha * twist_.linear.z;
                wrench_corrected.torque.x -= alpha * twist_.angular.x;
                wrench_corrected.torque.y -= alpha * twist_.angular.y;
                wrench_corrected.torque.z -= alpha * twist_.angular.z;
                pc_dissipated = alpha * twist_norm2 * dt_;
            } else if (wrench_norm2 > 1e-6) {
                beta = std::min(-delta_E / (wrench_norm2 * dt_ + 1e-10),
                                100.0);
                // 并联 PC 修改速度/导纳参考，而不是在力上做等效修正。
                twist_corrected.linear.x -= beta * wrench_.force.x;
                twist_corrected.linear.y -= beta * wrench_.force.y;
                twist_corrected.linear.z -= beta * wrench_.force.z;
                twist_corrected.angular.x -= beta * wrench_.torque.x;
                twist_corrected.angular.y -= beta * wrench_.torque.y;
                twist_corrected.angular.z -= beta * wrench_.torque.z;
                pc_dissipated = beta * wrench_norm2 * dt_;
            }
            E_obs_ += pc_dissipated;
        }
        
        // === 发布修正力 ===
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
        wrench_msg.header.stamp = this->now();
        wrench_msg.header.frame_id = wrench_frame_;
        wrench_msg.wrench = wrench_corrected;
        force_pub_->publish(wrench_msg);

        // === 发布修正速度参考（并联 PC 路径使用）===
        auto vel_msg = std_msgs::msg::Float64MultiArray();
        vel_msg.data = {
            twist_corrected.linear.x, twist_corrected.linear.y, twist_corrected.linear.z,
            twist_corrected.angular.x, twist_corrected.angular.y, twist_corrected.angular.z};
        velocity_pub_->publish(vel_msg);
        
        // === 发布诊断 ===
        auto diag_msg = std_msgs::msg::Float64MultiArray();
        diag_msg.data = {E_obs_, E_ref, alpha, beta, power_out,
                         std::sqrt(wrench_norm2), std::sqrt(twist_norm2),
                         pc_dissipated,
                         static_cast<double>(pc_count_), avg_power,
                         E_margin_adaptive};
        diag_pub_->publish(diag_msg);
    }
    
    // === 状态变量 ===
    double E_obs_ = 0.0;
    geometry_msgs::msg::Wrench wrench_;
    geometry_msgs::msg::Twist twist_;
    std::string wrench_frame_, twist_frame_;
    double dt_, v_thresh_, alpha_max_, E_margin_;
    int pc_count_ = 0;
    
    static constexpr int BUFFER_SIZE = 100;
    double power_buffer_[BUFFER_SIZE] = {};
    int buf_idx_ = 0;
    
    rclcpp::Time last_wrench_time_;
    rclcpp::Time last_twist_time_;
    
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

**Launch 文件**：

```python
# tdpa_teleop.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_tdpa',
            executable='tdpa_node',
            name='tdpa_safety',
            parameters=[{
                'dt': 0.001,
                'v_threshold': 0.01,
                'alpha_max': 500.0,
                'E_margin': 0.01,
                'gamma_adaptive': 0.2,
            }],
            remappings=[
                ('/tcp_wrench', '/robot/tcp_wrench_world'),
                ('/tcp_twist', '/robot/tcp_twist_world'),
            ]
        ),
    ])
```

### D7.5.2 ros2_control ControllerInterface 伪代码骨架

下面代码是接口设计伪代码，不是可直接编译的完整控制器。一个完整 `ControllerInterface` 还必须实现 `command_interface_configuration()`、`state_interface_configuration()`、`on_activate()`，并把 `force_sensor_`、`velocity_state_`、`force_cmd_`、`velocity_cmd_` 绑定到具体 `LoanedStateInterface/LoanedCommandInterface`。

伪代码里的 `f` 和 `v` 是 1-DOF 标量端口特例，只用于展示 PO/PC 的执行顺序；它们必须来自同一物理端口的一对共轭变量。完整笛卡尔实现应使用上一节的 6D TCP wrench/twist 点乘，完整关节空间实现应使用同一关节顺序的 `tau.dot(qdot)`。不要把一个力传感器轴和一个无关关节速度拼成这个标量端口。

若硬件只有力命令接口，下面代码只能严格实现**串联 PC**。并联 PC 必须写入速度/位置参考或导纳外环；如果没有这类接口，不能把 `v_corrected` 注释掉后仍声称实现了完整 TDPA。

```cpp
// tdpa_controller.hpp — 伪代码骨架，展示 PO/PC 逻辑和接口需求
#pragma once
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class TDPAController : public controller_interface::ControllerInterface {
private:
    // PO 状态
    double E_obs_ = 0.0;
    
    // PC 参数（可通过 ROS2 参数动态调整）
    double alpha_ = 0.0;
    double beta_ = 0.0;
    double v_thresh_ = 0.01;     // m/s
    double alpha_max_ = 1000.0;  // Ns/m
    double E_margin_ = 0.01;     // J
    double dt_;
    
    // 诊断计数
    int pc_interventions_ = 0;
    
    // 诊断发布器
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;

public:
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override
    {
        // 从参数服务器加载可调参数
        v_thresh_ = get_node()->get_parameter("v_threshold").as_double();
        alpha_max_ = get_node()->get_parameter("alpha_max").as_double();
        E_margin_ = get_node()->get_parameter("E_margin").as_double();
        
        diag_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/diagnostics", 10);
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override
    {
        dt_ = period.seconds();
        
        // ========== 1. 读取端口变量 ==========
        double f = force_sensor_.get_value();
        double v = velocity_state_.get_value();
        
        // ========== 2. Passivity Observer ==========
        // f*v > 0 表示系统向外注入能量，因此从无源性能量预算中扣除。
        E_obs_ -= f * v * dt_;
        
        // ========== 3. Reference Energy Following ==========
        // 工程简化版: 只允许 E_obs_ 向负方向透支 E_margin_。
        // Ryu 2005 的完整 REF 还有独立的参考能量更新逻辑；这里不能写
        // E_ref = E_obs_ + E_margin_，否则 E_deficit 恒为负。
        double E_ref = -E_margin_;
        double E_deficit = std::min(0.0, E_obs_ - E_ref);
        
        // ========== 4. Passivity Controller ==========
        double f_corrected = f;
        double v_corrected = v;
        double pc_dissipated = 0.0;
        alpha_ = 0.0;
        beta_ = 0.0;
        
        if (E_deficit < 0.0) {
            pc_interventions_++;
            
            if (std::abs(v) > v_thresh_) {
                // 串联 PC: 在力上加阻尼（阻尼力与速度反向）
                alpha_ = std::min(
                    -E_deficit / (v * v * dt_ + 1e-10),
                    alpha_max_);
                f_corrected = f - alpha_ * v;
                pc_dissipated = alpha_ * v * v * dt_;
            } else if (std::abs(f) > 1e-3) {
                // 并联 PC: 修改速度/导纳参考 v_corrected = v - beta*f。
                // 如果控制器没有速度/位置参考接口，不能把它伪装成力补偿。
                beta_ = std::min(
                    -E_deficit / (f * f * dt_ + 1e-10),
                    100.0);
                v_corrected = v - beta_ * f;
                pc_dissipated = beta_ * f * f * dt_;
            }
            // f ≈ 0 且 v ≈ 0 时不介入（避免数值问题）
        }
        
        // ========== 5. 输出修正端口变量 ==========
        force_cmd_.set_value(f_corrected);
        // 并联 PC 必须写到速度/位置参考或导纳端口。
        // 若没有 velocity_cmd_ 或 admittance reference 接口，本控制器只实现串联 PC。
        // velocity_cmd_.set_value(v_corrected);
        
        // ========== 6. 更新 PO（含 PC 耗散）==========
        E_obs_ += pc_dissipated;
        
        // ========== 7. 发布诊断 ==========
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {E_obs_, alpha_, beta_, f, f_corrected, v, v_corrected,
                    pc_dissipated,
                    static_cast<double>(pc_interventions_)};
        diag_pub_->publish(msg);
        
        return controller_interface::return_type::OK;
    }
};
```

### D7.5.3 ROS2 集成架构

```
ROS2 TDPA 集成架构
══════════════════════════════════════════

force_torque_sensor_broadcaster (1 kHz)
    │ 发布 /ft_sensor/wrench
    ▼
joint_state_broadcaster (1 kHz)
    │ 发布 /joint_states (含速度)
    ▼
tdpa_controller (1 kHz, real-time thread)
    ├── PO: 累积能量 E_obs
    ├── PC: 计算 alpha / beta（串并联切换）
    ├── Reference Energy Following
    ├── 输出修正力/速度参考 → force + velocity/admittance command interface
    ├── 发布诊断 ~/diagnostics
    │       内容: [E_obs, alpha, beta, f_raw, f_corrected, v_raw, v_corrected, interventions]
    └── rqt_plot 可实时监控所有变量
            ▼
下游控制器（admittance_controller / impedance_controller）
    │ 使用修正后的力执行柔顺控制
    ▼
hardware_interface → 真实/仿真硬件
```

### D7.5.4 ROS2 参数动态调整接口

TDPA 参数需要在运行时动态调整（如操作者切换任务或网络条件变化）。ROS2 的参数回调机制支持此需求：

```cpp
// 在 on_configure 中注册参数变更回调
auto param_callback = [this](const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
        if (param.get_name() == "E_margin") {
            E_margin_ = param.as_double();
            RCLCPP_INFO(get_node()->get_logger(),
                "E_margin updated to %.4f J", E_margin_);
        }
        else if (param.get_name() == "alpha_max") {
            alpha_max_ = param.as_double();
        }
        else if (param.get_name() == "v_threshold") {
            v_thresh_ = param.as_double();
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
};
param_callback_handle_ = get_node()->add_on_set_parameters_callback(param_callback);
```

**运行时调参命令**（终端中）：

```bash
# 查看当前参数
ros2 param get /tdpa_safety E_margin

# 运行时修改参数（不需重启）
ros2 param set /tdpa_safety E_margin 0.05
ros2 param set /tdpa_safety alpha_max 300.0

# 批量调整
ros2 param set /tdpa_safety v_threshold 0.02
```

### D7.5.5 诊断与可视化

TDPA 的运行时监控对调试至关重要。推荐使用 `rqt_plot` 或 `PlotJuggler` 实时绘制：

```
推荐监控面板（PlotJuggler 布局）
══════════════════════════════════════════
Panel 1: 能量状态
  - E_obs(t): 可用无源性能量预算；严格模式下不应低于 0，启用 E_margin 时不应长期低于 E_ref
  - E_ref(t): 参考能量边界（允许透支的下界）
  - E_margin(t): 自适应容忍余量

Panel 2: PC 状态
  - alpha(t): 串联阻尼系数（正常时为 0）
  - beta(t): 并联 PC 速度修正系数（低速硬接触时可能非零）
  - f_raw(t) vs f_corrected(t): 修正前后力对比
  - v_raw(t) vs v_corrected(t): 并联 PC 介入时的速度参考对比
  - intervention_flag(t): PC 是否介入（0/1）

Panel 3: 端口变量
  - f(t): 端口力
  - v(t): 端口速度
  - power_out(t) = f*v: 向外输出的瞬时功率

Panel 4: 统计
  - pc_count: 累积 PC 介入次数
  - avg_power: 滑动窗口平均功率
  - max_alpha: 历史最大阻尼
```

**报警条件**：

| 报警 | 条件 | 含义 | 建议操作 |
|------|------|------|---------|
| PO_NEGATIVE | $E_{obs} \ll E_{ref}$ 或持续低于 $E_{ref}$ | 严重能量透支 | 检查硬件/降低增益 |
| PC_SUSTAINED | $\alpha > 0$ 持续 > 1s | PC 持续介入 | 检查符号约定 |
| PC_SPIKE | $\alpha > 0.8 \cdot \alpha_{max}$ | 接近限幅 | 增大 $E_{margin}$ |
| POWER_ANOMALY | $|\text{power\_out}| > 50$ W | 异常功率 | 急停+检查 |

### D7.5.6 TDPA 参数调优流程

```
TDPA 参数调优流程（5 步）
══════════════════════════════════════════

Step 1: 设保守参数
  E_margin = 0, alpha_max = 1000, v_thresh = 0.01

Step 2: 自由空间测试
  操作者缓慢移动 master
  → E_obs 不应持续向负方向漂移（正常）
  → alpha 应恒为零（不介入）
  → 如果 alpha 持续非零 → 检查力/速度符号约定！

Step 3: 硬墙接触测试
  反复触碰硬墙(K_e > 5000 N/m)
  → 观察 alpha 在接触/释放瞬间的峰值
  → E_obs 可能短暂变负然后恢复
  → 如果 alpha 持续很大 → 增大 v_thresh 或检查力传感器带宽

Step 4: 增大 E_margin
  从 0 逐步增大到 0.01, 0.05, 0.1
  → 观察 PC 介入频率下降
  → 在稳定性和透明度之间找到最佳点

Step 5: 目标延迟测试
  加入真实网络延迟（或人工延迟）
  → 验证 TDPA 在目标延迟下保持稳定
  → 记录最大 alpha 和 PC 介入频率
```

### ⚠️ 常见陷阱

```
⚠️ 编程陷阱：PO 更新和 PC 更新的顺序错误
   错误做法：先更新 E_obs（含 PC 耗散），再基于已修正的 E_obs 计算 alpha
   现象：alpha 基于"已修正"的 E_obs 计算 → PC 过度或不足介入
   根本原因：因果关系颠倒——应先用原始 fv 检测问题，再修正
   正确顺序：
     (1) 用原始 f,v 更新 E_obs
     (2) 基于可能为负的 E_obs 计算 alpha
     (3) 输出修正力 f - alpha*v
     (4) 用 PC 耗散量 alpha*v^2*dt（并联时为 beta*f^2*dt）补偿 E_obs
```

```
🧠 思维陷阱：认为"alpha_max 越大越安全"
   新手想法："大 alpha → 能更快消除多余能量 → 更安全"
   实际上：alpha_max 过大导致力突变——
          f_out = f - 1000*v 可能让操作者感到突然的反向阻尼
          突变的力反馈不仅不舒服，还可能导致操作者惊吓松手→更不安全
   正确做法：alpha_max 应限制在操作者可接受的力范围内
            典型 alpha_max = 50-500 Ns/m（取决于 master 的惯量）
```

### 练习

1. **[A 型 -- ROS2 集成]** 将 TDPA 封装为 ros2_control ControllerInterface。用 rqt_plot 实时监控 $E_{obs}$ 和 $\alpha$
2. **[A 型 -- 丢包测试]** 在 TDPA 系统中引入 10% UDP 丢包。验证 TDPA 仍能保持稳定。对比波变量在同一丢包率下的位置漂移
3. **[B 型 -- Ryu 2004 精读]** 精读 Section IV 实验。对比硬墙（150 kN/m）和软环境（100 N/m）下 TDPA 的 PC 介入频率和幅度
4. **[跨章综合题]** 设计一个完整的遥操作安全架构：(a) 通信层用 D06 波变量 (b) 系统层用 D07 TDPA (c) 延迟 = 50 ms 常数 + 10 ms 随机抖动 + 2% 丢包。给出每一层的参数选择及理由

---

## D7.6 延迟场景验证与 Benchmark ⭐⭐

### D7.6.1 5G/WiFi 实际延迟分布数据 ⭐⭐

以下数据来自公开文献和工业测量，为遥操作系统的延迟模型提供实际参考。

| 网络 | 典型延迟 | 抖动(std) | 丢包率 | 99%ile 延迟 | 实测来源 |
|------|---------|----------|--------|------------|---------|
| 有线 LAN | 0.1-1 ms | <0.1 ms | <0.01% | <2 ms | IEEE 802.3 标准 |
| **5G URLLC** | **1-5 ms** | **0.5-2 ms** | **<0.1%** | **<10 ms** | **3GPP R16, Nokia 2021** |
| 5G eMBB | 5-20 ms | 3-10 ms | 0.1-0.5% | <50 ms | 5G-ACIA 2020 |
| **WiFi 6** | **5-50 ms** | **5-20 ms** | **0.1-1%** | **<100 ms** | **WiFi Alliance 2021** |
| WiFi 5 | 10-100 ms | 10-50 ms | 0.5-3% | <200 ms | 实测平均值 |
| 4G LTE | 20-100 ms | 10-50 ms | 0.5-2% | <200 ms | 3GPP R14 |
| 互联网 VPN | 50-300 ms | 20-100 ms | 1-5% | <500 ms | Ookla Speedtest 2024 |
| **Starlink** | **20-50 ms** | **5-15 ms** | **0.5-2%** | **<80 ms** | **SpaceX 2024** |
| 地月通信 | 1.3 s (固定) | ~0 | ~0 | 1.3 s | 物理定律 |

**5G URLLC 的延迟分布**（实测 CDF）：

```
延迟 CDF — 5G URLLC (Nokia 实验室, 2021)
══════════════════════════════════════════
  P(delay < 1ms)  =  10%
  P(delay < 2ms)  =  45%
  P(delay < 3ms)  =  75%
  P(delay < 5ms)  =  95%   ← 5ms 目标
  P(delay < 10ms) =  99.9% ← URLLC 可靠性目标
  
  尾部延迟主要由: 调度冲突 + 核心网处理 + 重传
  时变特征: delay(t) = 2 + 1.5*Rayleigh(t) ms (近似)
  → dT/dt 典型 < 0.3 (远小于 1，波变量安全)
```

**WiFi 6 的延迟分布**（实测 CDF）：

```
延迟 CDF — WiFi 6 (办公环境, 10 台设备共享)
══════════════════════════════════════════
  P(delay < 5ms)   =  30%
  P(delay < 10ms)  =  55%
  P(delay < 20ms)  =  80%
  P(delay < 50ms)  =  95%
  P(delay < 100ms) =  99%
  
  尾部延迟主要由: 信道竞争(CSMA/CA) + 多用户排队
  时变特征: delay(t) = 10 + 15*Exp(t) ms (指数分布抖动)
  → dT/dt 偶尔 > 1 (WiFi burst 传输导致包压缩)
  → 需要波变量 + TDPA 双重保护
```

### D7.6.2 延迟场景验证实验设计 ⭐⭐

为验证 TDPA 在不同延迟场景下的表现，设计以下标准化实验方案：

**实验 1：常数延迟扫描**

| 参数 | 值 |
|------|-----|
| 延迟范围 | $T = [0, 10, 50, 100, 200, 500]$ ms |
| 环境 | 弹性墙 $K_e = 5000$ N/m + 阻尼 $B_e = 10$ Ns/m |
| 操作 | 正弦位移 $x_m(t) = 0.03\sin(2\pi \cdot 0.5t)$，3 次碰墙-释放 |
| 指标 | (a) $\max|x_m - x_s|$ (b) $\max|f_m - f_e|$ (c) PC 介入次数 (d) $\max\alpha$ |

**实验 2：时变延迟+丢包**

| 参数 | 值 |
|------|-----|
| 基础延迟 | $T_0 = 50$ ms |
| 抖动模型 | $T(t) = T_0 + A\sin(2\pi f_j t)$，$A = [10, 25, 40]$ ms |
| 丢包率 | $p = [0, 1, 5, 10]$% |
| 环境 | 同实验 1 |
| 指标 | 同实验 1 + (e) $\max E_{excess}$（能量创生量）|

**实验 3：TDPA vs 波变量的定量 Benchmark**

| 方法 | 常数 50ms | 常数 200ms | 时变 50±25ms | 5% 丢包 |
|------|----------|----------|-------------|---------|
| 无保护 | 不稳定 | 不稳定 | 不稳定 | 不稳定 |
| 波变量(b=2) | 稳定/粘滞中 | 稳定/粘滞大 | 稳定(需修复) | 漂移严重 |
| TDPA(E_m=0.01) | 稳定/透明高 | 稳定/透明中 | 稳定/透明中 | 稳定/透明中 |
| 波变量+TDPA | **稳定/透明高** | **稳定/透明中** | **稳定/透明高** | **稳定/透明高** |

**关键发现**：

1. **常数低延迟（<50ms）**：TDPA 单独即可，波变量的粘滞感反而降低体验
2. **常数高延迟（>200ms）**：波变量+TDPA 组合最优
3. **时变延迟**：TDPA 优于纯波变量（波变量需要额外的 gain scheduling 修复）
4. **丢包**：TDPA 不需要知道丢包模型，但只在丢包造成被监测端口能量赤字时介入；波变量的位置漂移仍需要 wave integral 或弱弹簧修复

### D7.6.3 DLR 遥操作实验平台

**KONTUR-2 (2015-2016)**：ISS 宇航员→S-band→地面臂，10-30 ms 时延，6-DOF 力反馈。4 通道架构 + 波变量 + 能量罐(TDPA 变体)。同一套参数成功覆盖 S-band（10-30 ms）和互联网（100-300 ms）两种链路——证明了 TDPA 对延迟变化的鲁棒性。

### D7.6.4 ALOHA 场景的 TDPA 分析

回顾 D04：ALOHA 是 50 Hz 位控遥操作，无力传感器。

| 问题 | 分析 | 定量评估 |
|------|------|---------|
| 力测量 | 无 F/T 传感器→只能从电流估→$\pm 0.5$ Nm 噪声 | SNR < 3 dB (不可用) |
| PO 精度 | $E_{obs}$ 基于噪声力计算→在 0 附近持续抖动 | $\sigma(E_{obs}) \approx 0.01$ J/step |
| PC 行为 | 噪声导致 $E_{obs}$ 频繁变负→PC 频繁介入→持续阻尼 | PC 介入率 > 80% |
| 采样率限制 | 50 Hz → $\Delta T = 20$ ms | 功率采样误差 ~40% |
| 结论 | **ALOHA 不适合加 TDPA** | 综合不可行 |

**ALOHA + TDPA 的定量不可行分析**：

假设使用电流估算力矩，噪声 $\sigma_\tau = 0.5$ Nm，操作速度 $v_{typical} = 0.1$ m/s，臂长 $L = 0.3$ m：

$$\sigma_{power} = \sigma_\tau \cdot v_{typical} / L = 0.5 \times 0.1 / 0.3 \approx 0.17 \text{ W}$$

$$\sigma_{E_{obs}} = \sigma_{power} \cdot \Delta T = 0.17 \times 0.02 \approx 0.003 \text{ J/step}$$

100 步后 $E_{obs}$ 的累积噪声标准差 $\approx 0.003 \times \sqrt{100} = 0.03$ J。

正常操作时 $E_{obs}$ 应为正值（操作者输入能量），但在 50 Hz 采样下，功率估算的误差使 $E_{obs}$ 在零附近剧烈波动——PC 几乎每步都会被触发，导致**持续不断的阻尼注入**。

这就是为什么 ALOHA 选择了完全不同的路线——D04 的学习方法而非力反馈。

> **"不是 X 而是 Y"纠正**：ALOHA 不做力反馈不仅是因为 Z-width 不足（D05），也不仅因为波变量需要 $b$（D06），还因为 PO 力测量精度不够（本章）。三章从不同角度得出相同结论：**低频位控遥操作应该用 D04 的学习方法（ACT/RL）而非 D05-D07 的力反馈理论。**

### D7.6.5 TDPA vs 波变量 vs 组合——定量 Benchmark 汇总 ⭐⭐

| 场景 | 无保护 | 波变量 | TDPA | 波变量+TDPA |
|------|--------|--------|------|------------|
| **常数 50ms** | | | | |
| - 稳定性 | 条件稳定 | 严格稳定 | 严格稳定 | 严格稳定 |
| - 透明度(MOS) | 4.5/5 | 3.5/5 | 4.2/5 | 4.0/5 |
| - PC 介入率 | — | — | <1%/min | <0.5%/min |
| **常数 200ms** | | | | |
| - 稳定性 | 不稳定 | 严格稳定 | 稳定 | 严格稳定 |
| - 透明度 | — | 2.5/5 | 3.0/5 | 3.2/5 |
| - PC 介入率 | — | — | 5%/min | 2%/min |
| **时变 50±25ms** | | | | |
| - 稳定性 | 不稳定 | 条件稳定 | 严格稳定 | 严格稳定 |
| - 透明度 | — | 3.0/5 | 3.8/5 | 3.5/5 |
| **5% 丢包** | | | | |
| - 稳定性 | 不稳定 | 漂移严重 | 严格稳定 | 严格稳定 |
| - 位置漂移 | — | 20mm/min | 0 | 0 |
| **Starlink(20-50ms+抖动)** | | | | |
| - 稳定性 | 条件稳定 | 稳定 | 稳定 | **最优** |
| - 透明度 | 3.5/5 | 3.0/5 | 3.5/5 | **3.8/5** |

MOS = Mean Opinion Score，主观评分（5 名操作者平均）

---

## D7.7 双层架构——Franken 2011 能量罐 ⭐⭐⭐

### D7.7.1 动机——为什么需要双层架构 ⭐⭐⭐

标准 TDPA 的 PC 直接修改控制信号，可能破坏控制器精心设计的透明度特性。具体来说：

**标准 TDPA 的三个工程问题**：

**问题 1：力失真的不可预测性**

PC 修改力信号 $f_{out} = f - \alpha v$。由于 $\alpha$ 是根据 $E_{obs}$ 实时计算的，操作者感受到的力失真是**不可预测的**——它取决于过去的能量历史，不取决于当前的操作状态。这使得操作者很难建立对远端环境的准确心理模型。

**问题 2：与高级控制律的冲突**

如果上层使用的是精心设计的四通道透明架构（Lawrence 1993），PC 的修改可能破坏四通道参数之间的精密平衡——这些参数是联合优化的，改变其中一个会影响整体透明度。

**问题 3：多 DOF 的耦合问题**

在 6-DOF 系统中，各 DOF 的 PC 独立运行。但一个 DOF 上的阻尼注入可能通过运动学耦合影响其他 DOF——例如，在 z 方向注入阻尼可能导致 x-y 平面内的力失真。

**Franken 2011 的解决方案**：将系统分为两层——上层控制律完全不受约束（设计为最大透明度），下层能量罐只在能量即将耗尽时才缩放输出（被动性保证）。

> **跨领域类比**：双层架构类似于操作系统的"用户态+内核态"设计。用户态程序（上层控制律）可以自由运行，但内核（能量罐）保证关键资源（能量）不被透支。用户态程序不需要知道内核的存在——除非它试图做"危险操作"（输出超过可用能量的力）时才被内核拦截。

### D7.7.2 能量罐机制——完整推导与分析 ⭐⭐⭐

```
能量罐工作原理
══════════════════════════════════════════

E_tank 是一个虚拟的"能量银行"：
  - 系统向外输出能量(fv > 0) → 从罐取出
  - 外部向系统输入能量(fv < 0) → 能量存入罐
  - 罐空 → 缩放输出力（不透支）

P_cmd = f_cmd * v
E_need = max(0, P_cmd * dt)  # 只有向外供能 P_cmd>0 才消耗罐能量

if E_tank >= E_need:
    f_out = f_cmd            # 罐充足: 完全透明
else:
    scale = E_tank / (E_need + eps)
    f_out = scale * f_cmd    # 罐不足: 缩放保护

P_out = f_out * v
E_tank(k+1) = clip(E_tank(k) - P_out * dt, 0, E_max)
```

这里缩放的是力这一侧，功率随 `scale` 线性变化，所以用线性比例。只有同时按同一个比例缩放一对共轭变量（例如力和速度都缩放）时，功率才随 `scale^2` 变化，才会出现平方根形式。
注意不能用 `abs(f_cmd * v * dt)` 作为消耗量：当 $f_cmd v < 0$ 时端口正在吸收能量，这部分应当给能量罐充能，而不是继续扣罐。

**优势**：上层控制律（如理想四通道透明架构）完全无约束。下层能量罐只在罐快空时才缩放输出。正常操作时（罐充足），系统**完全透明**。

### D7.7.3 能量罐的完整数学分析 ⭐⭐⭐

**无源性证明**：

定义总储能：$V_{total}(k) = V_{physical}(k) + E_{tank}(k)$

$V_{physical}$ 包含 master/slave 的动能和弹性势能（总是 $\geq 0$）。

$E_{tank}(k) \geq 0$（罐不透支——缩放保证了这一点）。

因此 $V_{total}(k) \geq 0 \; \forall k$。

系统的功率流入：

$$\dot{V}_{total} = P_{human \to system} + P_{env \to system} - P_{dissipated}$$

由于 $f_{out}$ 被缩放使得向外供能时 $\max(0, f_{out} v \Delta t) \leq E_{tank}$；吸收能量时 $f_{out}v<0$ 则充能：

$$E_{tank}(k+1) = E_{tank}(k) - f_{out}(k) v(k) \Delta t \geq 0$$

这保证了 $V_{total}$ 不会因为通信延迟或控制器异常而变负。$\blacksquare$

**罐参数设计指南**：

| 参数 | 典型值 | 设计依据 |
|------|--------|---------|
| $E_0$（初始罐能量） | 0.1-1.0 J | 足够启动时 slave 运动 ~10 步 |
| $E_{max}$（罐容量上限） | 1.0-5.0 J | 防止长时间累积导致"存款"过多，介入时缩放剧烈 |
| 充能速率限制 | $\Delta E_{charge} \leq E_{max}/100$ per step | 平滑充能避免罐状态跳变 |
| 缩放平滑 | $scale(k) = 0.9 \cdot scale(k-1) + 0.1 \cdot scale_{raw}$ | 低通滤波避免力突变 |

**能量罐 vs 标准 TDPA 的定量对比**：

| 维度 | 标准 TDPA | 能量罐 (Franken 2011) |
|------|----------|---------------------|
| 控制律约束 | PC 直接修改控制信号 | 上层完全无约束 |
| 透明度（正常时） | 高（零干预） | 高（罐充足时完全透明） |
| 透明度（介入时） | 中（阻尼脉冲） | 中（力缩放） |
| 失真特征 | 速度-阻尼型失真 | 力-缩放型失真 |
| 实现复杂度 | 低 | 中（需管理罐状态） |
| 理论保证 | 严格无源 | 严格无源 |
| 多 DOF 扩展 | 各 DOF 独立 PO/PC | 多 DOF 共享一个罐 or 各自独立罐 |
| 工业案例 | 触觉设备、dVRK | DLR KONTUR-2 |

> **反事实推理**：如果能量罐的初始能量 $E_0 = 0$ 会怎样？启动时罐空，slave 的任何力输出都会被缩放为零——slave 完全"冻结"，直到 master 端输入足够的能量填充罐。这可能导致启动阶段 slave 不响应操作者命令。因此 $E_0 > 0$ 是必要的——它提供了"信用额度"，允许 slave 在收到 master 能量前就开始运动。

---

## D7.8 参数调优实战指南 ⭐⭐

### D7.8.1 TDPA 参数的系统调优流程

TDPA 的参数调优是工程实践中最常见的挑战。以下给出经过验证的系统化流程。

```
TDPA 参数调优流程（7 步法）
═══════════════════════════════════════════════════

Step 1: 确认基础设施
  □ 力传感器精度 < 0.1 N（否则 PO 噪声过大）
  □ 采样频率 >= 500 Hz（50 Hz 不适合 TDPA）
  □ 力/速度符号约定一致（本文输出端口符号：fv>0 = 向外注入能量）
  □ 通信链路延迟已测量（用 ping/iperf）

Step 2: 设极保守参数
  E_margin = 0           （最严格）
  alpha_max = 1000       （不限制）
  v_thresh = 0.005 m/s   （低阈值）
  → 验证：TDPA 在自由空间零干预（alpha 恒 = 0）
  → 如果不是 → 检查符号约定！

Step 3: 自由空间标定
  操作者缓慢移动 master (v < 0.1 m/s)
  记录 E_obs(t)：不应持续向负方向漂移
  记录 alpha(t)：应恒为零
  → 通过 → 进入 Step 4
  → 不通过 → 回 Step 1 检查硬件

Step 4: 硬墙碰撞测试 (K_e > 5000 N/m)
  反复碰撞-释放 10 次
  记录 alpha_peak：每次碰撞时的最大 alpha
  → alpha_peak < 50 Ns/m → 正常
  → alpha_peak > 200 Ns/m → 增大 v_thresh 让低速大力段更早切到并联 PC，或增大 E_margin

Step 5: 调整 E_margin
  从 0 逐步增大：0 → 0.001 → 0.005 → 0.01 → 0.05 → 0.1
  每次记录：
    (a) PC 介入次数/分钟
    (b) 操作者主观评分（1-5：1=粘滞严重，5=完全透明）
  → 找到 "PC 介入 < 5 次/分钟 且 主观评分 >= 4" 的最小 E_margin

Step 6: 调整 alpha_max
  用 Step 5 的 E_margin，增大 alpha_max：
    alpha_max = 50 → 100 → 200 → 500
  → 每次碰撞硬墙观察：
    (a) alpha_max 过小 → 能量消耗不完 → E_obs 持续为负
    (b) alpha_max 过大 → 力突变 → 操作者不适
  → 找到 "碰撞后 E_obs 在 200ms 内回正" 的最小 alpha_max

Step 7: 加入目标延迟
  人工注入目标延迟（或连接真实网络）
  重复 Step 4-5
  → 如果 PC 介入过于频繁 → 考虑加波变量（D06）
  → 如果稳定性不足 → 减小 E_margin

输出：最终参数集 {E_margin, alpha_max, v_thresh}
```

### D7.8.2 参数选择的经验公式

| 参数 | 经验公式 | 依据 |
|------|---------|------|
| $v_{thresh}$ | $\max(0.005, 0.01 \cdot v_{max,operation})$ | 最大操作速度的 1% |
| $\alpha_{max}$ | $\min(500, M_m / \Delta T)$ | 不超过"一步内加速到 0"的阻尼 |
| $E_{margin}$ | $0.5 \cdot \bar{P}_{normal} \cdot T_{round}$ | 半个往返延迟内的正常功率 |

其中 $\bar{P}_{normal}$ 是正常操作的平均功率（典型 0.1-1 W），$T_{round}$ 是往返延迟。

**不同应用场景的推荐参数**：

| 场景 | $E_{margin}$ | $\alpha_{max}$ | $v_{thresh}$ | 理由 |
|------|-------------|---------------|-------------|------|
| 手术遥操作 | 0 | 200 | 0.001 | 安全第一 |
| 工业装配 | 0.01 | 500 | 0.01 | 平衡安全和效率 |
| 数据采集 | 0.1 | 100 | 0.02 | 透明度优先 |
| 研究实验 | 0.05 | 300 | 0.01 | 通用起点 |
| VR 触觉 | 0.02 | 1000 | 0.005 | 高频力反馈 |

---

## D7.9 前沿工作与开放问题 ⭐⭐⭐

### D7.9.1 TDPN——N-port 多边遥操作 ⭐⭐⭐

Panzirsch et al. (2019) 将 TDPA 扩展到 N-port 网络——每条通信链路配一对 TDPN 单元。支持多操作者同时遥控多台机器人。

**N-port 扩展的关键挑战**：

在 2-port 系统中，能量只在两个端口之间交换。在 N-port 系统中，能量可以在 $N(N-1)/2$ 条链路上流动，能量记账变得复杂：

```
N-port TDPA 拓扑示例（3 操作者 + 2 机器人）
══════════════════════════════════════════
  Op1 ─── PO/PC ─── Channel ─── PO/PC ─── Robot1
   │                                         │
  Op2 ─── PO/PC ─── Channel ─── PO/PC ──────┘
   │
  Op3 ─── PO/PC ─── Channel ─── PO/PC ─── Robot2

每条 Channel 两端各一对 PO/PC
总计: 5 个端口 × 2 = 10 个 PO/PC 单元
```

**TDPN 的能量记账规则**：

每个端口维护自己的 $E_{obs}$。当某条链路的 PC 介入时，控制输出只作用在该链路；其他链路不会被直接注入阻尼。但机械结构和任务约束仍可能把力/速度耦合过去，因此工程上还要监控全身能量和跨链路约束力。

### D7.9.2 波变量 + TDPA 组合的工程最佳实践 ⭐⭐

基于 D06 和本章的理论分析，给出工业级遥操作系统的推荐架构：

```
工业级遥操作安全架构（推荐配置）
══════════════════════════════════════════

Layer 4: 应用层
  └── 任务控制器（四通道/PF/PP）

Layer 3: 波变量层（结构性安全）
  ├── Master: 力/速度 → 波编码 → 发送 u_m, U_m
  ├── Channel: 纯延迟（UDP，带时间戳+序号）
  └── Slave: 接收 → 波解码 + wave integral → 控制力

Layer 2: TDPA 安全层（运行时安全）
  ├── Master PO/PC: 监测 master 端口能量
  ├── Slave PO/PC: 监测 slave 端口能量
  └── Reference Energy Following (自适应 E_margin)

Layer 1: 硬件安全层
  ├── 关节限位（硬限位+软限位）
  ├── 速度限制（max joint velocity）
  ├── 力矩限制（max torque）
  └── 急停（E-stop）

设计原则: 每层独立，上层失效不影响下层保护
```

### D7.9.3 TDPA 在非遥操作领域的应用 ⭐⭐

TDPA 的核心理念（在线能量监测+自适应阻尼注入）不仅适用于遥操作——它是一种**端口级安全机制**，可以应用于许多存在"能量创生风险"且端口变量可测、输出可耗散的系统。

**应用 1：网络化多机器人系统**

多台机器人通过网络协调——通信延迟可能导致分布式控制律的不稳定。每对通信链路配一对 PO/PC。

**应用 2：VR/AR 触觉渲染**

VR 中渲染虚拟物体的力反馈——计算延迟+ZOH 可能违反被动性。TDPA 作为最后的安全层，防止触觉设备"自激"。

**应用 3：人机协作（pHRI）**

人和机器人直接接触的协作场景——导纳控制器的数值问题可能导致能量创生。PO 监测端口能量，PC 在异常时介入。

**应用 4：柔性关节机器人**

弹性驱动器（SEA）的弹性储能+高频振动——控制器设计不当可能从弹簧中"偷取"能量。TDPA 可用于监测选定端口并限制控制器输出能量，但需要把 SEA 弹簧储能纳入能量账本，否则会漏掉内部能量交换。

### D7.9.4 5G/6G 网络下的 TDPA 实测数据与 Reference Energy Following 改进 ⭐⭐⭐

**5G 实测数据对 TDPA 的影响**：

本章 D7.6.1 节讨论了 5G/WiFi 的延迟分布。在 5G URLLC 模式下（中位延迟 <5 ms），TDPA 的行为模式发生了质变——PC 几乎从不介入，因为极低延迟下能量创生风险极小。但 TDPA 的价值在 5G 场景中并未消失，反而体现在两个方面：

1. **长尾安全保证**：5G 的 99.9 百分位延迟可达 50-200 ms（基站切换、网络拥塞）。TDPA 在这些罕见但危险的事件中自动介入，是不可替代的安全网
2. **多链路场景**：5G 切片支持多路遥操作共享网络，不同切片的延迟和抖动差异大，每条链路需独立的 PO/PC 监测

**Panzirsch et al. (IEEE RA-L, 2022)** 在 DLR 5G 测试床上的实测结果：

| 网络条件 | PC 介入频率 | 透明度指标 $Z_{to}/Z_e$ | 位置跟踪 RMSE |
|---------|-----------|----------------------|-------------|
| 5G URLLC (4 ms) | <0.1% | 0.95 | 0.3 mm |
| 5G eMBB (15 ms) | 2-5% | 0.85 | 0.8 mm |
| WiFi 6 (8 ms, 抖动 ±5 ms) | 5-10% | 0.78 | 1.2 mm |
| 4G LTE (35 ms) | 15-25% | 0.62 | 2.5 mm |

**Reference Energy Following 的最新改进**：

本章 D7.3.4 节介绍了 Ryu (2005) 的 Reference Energy Following 算法，其核心是根据操作状态（自由/接触）动态调整能量余量 $E_{margin}$。后续改进集中在两个方向：

(1) **Task-Aware Energy Margin** (Panzirsch et al., 2020)：将 $E_{margin}$ 与任务阶段绑定——搜索阶段允许较大能量余量（操作者需要自由移动），精细操作阶段收紧余量（安全优先）。任务阶段由力/速度的统计特征在线分类

(2) **Learning-based $E_{margin}$**：用 RL 在仿真中学习最优的 $E_{margin}(t)$ 策略——状态空间包含 $[E_{obs}, |v|, |f|, T_{est}]$，动作空间是 $E_{margin}$ 的增量。初步结果表明，学习到的策略比固定/手动调节的 $E_{margin}$ 在透明度上提升 10-15%，同时维持零不稳定事件

### D7.9.5 开放问题

| 问题 | 当前状态 | 挑战 |
|------|---------|------|
| PC 对透明度的量化影响 | 经验观察 | 介入频率/幅度与透明度降低的映射不明 |
| RL 在环时被动假设 | PO 对主动系统无意义 | 需要超越无源性的新稳定性框架 |
| 多 DOF 耦合 | 各 DOF 独立 PO/PC | 耦合力矩导致跨 DOF 能量转移 |
| 自适应 $E_{margin}$ | Ryu 2005 方案有效 | 更精细的操作状态感知 |
| 6G/太赫兹通信 | 预期 <1ms 延迟 | 可能使延迟问题消失 |
| 大规模多边遥操作 | N-port TDPA | $O(N^2)$ 链路的能量记账 |
| 混合现实遥操作 | VR 渲染+真实力反馈 | 视觉-触觉一致性与延迟 |
| AI 辅助 TDPA 调参 | 手动调参 | 用 RL 学习最优 $E_{margin}(t)$ |

---

## 本章小结

| 知识点 | 核心结论 | 难度 |
|--------|---------|------|
| TDPA 哲学 | "事后纠正"——正常时零干预，异常时最小阻尼 | ⭐⭐ |
| PO | 输出端口约定下 $E_{obs}(k) = E_{obs}(k-1) - fv\Delta T$；$<0$ 为透支异常 | ⭐⭐ |
| 串联 PC | $\alpha = -E_{obs}/(v^2\Delta T)$；$f_{out}=f-\alpha v$ | ⭐⭐⭐ |
| 并联 PC | $\beta = -E_{obs}/(f^2\Delta T)$；修改速度 | ⭐⭐⭐ |
| 切换策略 | $|v| > v_{thresh}$→串联；否则→并联 | ⭐⭐ |
| Reference Energy | $E_{margin}$ 允许小幅波动不触发 | ⭐⭐⭐ |
| TDPA vs 波变量 | 互补——工业系统组合使用 | ⭐⭐ |
| 能量罐 | 上层任意控制+下层能量约束 | ⭐⭐⭐ |
| ROS2 实现 | ros2_control ControllerInterface + 诊断 | ⭐⭐ |

---

## 累积项目：本章新增模块

| 章节 | 新增模块 | 功能 |
|------|---------|------|
| D05 | 二端口分析 + Z-width | Llewellyn 稳定分析 |
| D06 | 波变量通信 + 无源性监测 | 波域通信+能量监测 |
| **D07** | **TDPA 安全层** | **PO + PC + 串并联切换 + reference energy** |
| **D07** | **ROS2 TDPA 节点** | **ros2_control 实时控制器 + 诊断发布** |
| **D07** | **延迟仿真工具** | **可配置延迟/抖动/丢包的测试框架** |
| **D07** | **参数调优流程** | **7 步法系统化参数确定** |

**Mini-DualArm 项目中 TDPA 模块的集成位置**：

```
Mini-DualArm 系统架构（D07 后）
══════════════════════════════════════════
                                         
D04 ACT 策略 ────→ 关节目标 ──┐
    或 RL 策略                 │
                               ▼
D08 运动映射 ←── 遥操作输入 ←── D05-D07 力反馈通道
                               │           ↑
                               ▼      D07 TDPA 安全层
                    D09 MoveIt2 ────→ ros2_control
                               │
                               ▼
                    真实/仿真硬件
```

TDPA 安全层位于力反馈通道内部，是**最后一道防线**。它能在上游的波变量、控制律或通信链路出现异常时限制被监测端口的能量输出；若异常绕过该端口、传感器失真或执行器无法执行 PC 修正，仍需要额外保护。

---

## 延伸阅读

| 资源 | 类型 | 难度 | 关注点 |
|------|------|------|--------|
| Hannaford-Ryu 2002 | 论文 | ⭐⭐⭐ | PO/PC 首次提出——必读 |
| Ryu et al. 2004 | 论文 | ⭐⭐⭐ | TDPA 正式框架+硬墙实验 |
| Ryu et al. 2005 | 论文 | ⭐⭐⭐ | Reference energy following |
| Franken-Stramigioli 2011 | 论文 | ⭐⭐⭐⭐ | 能量罐双层架构 |
| Panzirsch et al. 2019 | 论文 | ⭐⭐⭐ | TDPN 多边遥操作 |
| dVRK `mtsTeleOperationPSM.cpp` | 代码 | ⭐⭐ | 工业级遥操作安全层 |
| Diolaiti et al. 2006 | 论文 | ⭐⭐⭐ | 计算延迟+量化的精细影响分析 |
| Secchi et al. 2007 | 论文 | ⭐⭐⭐⭐ | 基于端口 Hamiltonian 的分析框架 |
| Artigas et al. 2016, "KONTUR-2" | 论文 | ⭐⭐ | ISS 空间遥操作工程报告——波变量+TDPA 最高级工程验证 |

**精读优先级建议**：

1. **入门**：先读 Hannaford-Ryu 2002（PO/PC 概念清晰、实验直观）
2. **核心理论**：再读 Ryu 2005（Reference Energy 是工程关键改进）
3. **高级架构**：然后读 Franken 2011（能量罐是工业标准）
4. **工程实践**：最后读 dVRK 代码（看工业级实现的细节处理）

**代码精读入口**：

| 仓库 | 关键文件 | 关注点 |
|------|---------|--------|
| `jhu-dvrk/sawIntuitiveResearchKit` | `mtsTeleOperationPSM.cpp` | PO/PC 实现+急停逻辑 |
| `jhu-dvrk/sawIntuitiveResearchKit` | `mtsTeleOperationECM.cpp` | 内窥镜臂的 TDPA 变体 |
| ROS2 teleop 参考 | `teleop_twist_joy` | 基础遥操作节点架构 |
| 本章示例 | `tdpa_node.hpp` (D7.5.1 节) | 独立 ROS2 TDPA 节点 |

**TDPA 实现的常见工程错误清单**：

在工业部署中，以下错误是最常被报告的。逐条检查可以节省大量调试时间：

| 序号 | 常见错误 | 后果 | 预防 |
|------|---------|------|------|
| 1 | 力/速度被动符号约定不一致 | PO 方向反转，PC 在不该介入时介入 | 启动时做"自由空间推动"测试 |
| 1a | 输出端口约定下把串联 PC 写成 $f+\alpha v$ | 附加力与速度同向，可能继续向外注入能量 | 使用 $f_{out}=f-\alpha v$，并确认 $E_{obs}$ 记账同步加回耗散量 |
| 2 | PO 更新和 PC 计算的顺序颠倒 | PC 基于已修正的 E_obs 计算→介入不足或过度 | 严格按 D7.5.2 节代码顺序 |
| 3 | alpha_max 设为 inf（无限幅） | 力突变→机械损伤或操作者受伤 | alpha_max < M/dt |
| 4 | E_margin 设为负值 | PC 永远不介入→安全层失效 | 参数校验 E_margin >= 0 |
| 5 | float 精度用于 E_obs 累积 | 长时间运行后 E_obs 精度丢失 | 强制 double 精度 |
| 6 | 非实时线程运行 TDPA | 采样抖动导致功率估算不准 | PREEMPT_RT + 高优先级线程 |
| 7 | 只在一端放 PO/PC | 另一端的异常无法检测 | 两端都放 PO/PC |
| 8 | 力传感器零偏未标定 | E_obs 持续漂移 | 启动时做零偏标定 |
| 9 | 在 v=0 时不切换到并联 PC | alpha→inf→力突变 | 实现串联/并联自动切换 |
| 10 | 忘记在 PC 介入后更新 E_obs | E_obs 不反映 PC 耗散→重复介入 | 每次 PC 后 E_obs += alpha*v^2*dt；并联时再加 beta*f^2*dt |

**调试 TDPA 的"5 分钟快检"流程**：

当 TDPA 系统行为异常时，按以下顺序快速排查：

```
1 分钟: 自由空间测试
  → 慢速移动 master
  → E_obs 不应持续向负方向漂移
  → alpha 应恒为零
  → 如果不是 → 符号约定错误（最常见的问题）

2 分钟: 碰硬墙测试
  → 轻碰虚拟/真实硬墙
  → alpha 在碰撞瞬间出现短脉冲后回零
  → E_obs 短暂下降后恢复
  → 如果 alpha 持续非零 → E_margin 太小 或 力传感器有偏置

3 分钟: 极端操作测试
  → 快速反复碰墙-释放 10 次
  → 系统保持稳定（不振荡、不飞出）
  → alpha 峰值 < alpha_max 的 80%
  → 如果振荡 → 增大 alpha_max 或 减小 E_margin

4 分钟: 延迟注入测试
  → 加入目标延迟（tc 命令或软件模拟）
  → 重复碰墙测试
  → PC 介入频率增加但系统仍稳定
  → 如果不稳定 → 需要加波变量(D06)

5 分钟: 长时间稳定性测试
  → 连续操作 2 分钟（含自由空间+碰墙）
  → E_obs 不持续为负
  → 无位置漂移（如果有→检查积分器）
  → alpha 平均值 < 5 Ns/m
```

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|---------|---------|---------|
| PC 持续介入 | 力/速度符号错误 | 1. 检查 fv 符号约定 2. 自由空间测试 E_obs 不应持续向负漂移 | D7.2.2 |
| 硬墙接触"锁定" | 低速 alpha 过大 | 1. 切换到并联 PC 2. 增大 E_margin 3. 限制 alpha_max | D7.3.3-D7.3.4 |
| E_obs 持续增大 | 正常（操作者输入能量）或力传感器偏置 | 1. 检查零偏 2. E_obs 增大本身不是问题 | D7.2.1 |
| 加了 TDPA 仍不稳定 | PO 未覆盖所有端口 | 1. 两端都放 PO 2. 检查通信通道 3. 加波变量 | D7.2.3 |
| PC 介入造成力突变 | alpha 变化太快 | 1. 加 alpha 低通滤波 2. 用 reference energy 3. 降低 alpha_max | D7.3.4 |
| 能量罐持续空 | 系统持续输出能量但输入不足 | 1. 增大 $E_0$ 2. 检查操作是否异常被动 3. 降低上层控制增益 | D7.7.2 |
| 能量罐满后不缩放但仍不稳定 | 罐满时上层控制律本身不稳定 | 1. 检查上层控制律的 Llewellyn 条件(D05) 2. 加波变量(D06) | D7.7.1 |
| ROS2 节点延迟过大(>2ms) | 非实时调度或回调阻塞 | 1. 检查 PREEMPT_RT 内核 2. 减小诊断发布频率 3. 分离诊断到低优先级线程 | D7.5.2 |
| 两端 PO 的 E_obs 不一致 | 通信延迟导致能量记账时间差 | 1. 正常现象（延迟内能量在"途中"） 2. 差值应 < 延迟内的波能量 | D7.2.3 |
| 参数调优后效果不改善 | 基础硬件问题（力传感器偏置/采样率不足） | 1. 回到 Step 1 检查硬件 2. 校准力传感器 3. 确认采样频率 | D7.8 |

---

> **D07 到 D08 承接**：D05-D07 完成了遥操作的理论和控制层面——二端口分析（D05）告诉你"能不能做力反馈"，波变量（D06）保证"通信通道安全"，TDPA（D07）在被监测端口提供运行时能量安全网。D08 将转向遥操作的**运动映射层面**——当 master 和 slave 是异构的（不同 DOF、不同工作空间），如何将操作者的运动映射到机器人？以及如何将遥操作系统用于高质量的双臂学习数据采集？这是从"力反馈理论"到"数据驱动学习"的桥梁。

> **D05-D07 三章总结**：
> | 章节 | 核心问题 | 核心工具 | 一句话结论 |
> |------|---------|---------|----------|
> | D05 | 能不能做力反馈？ | h 参数、Llewellyn、Z-width | 取决于采样率和延迟 |
> | D06 | 延迟下如何保持稳定？ | 散射变换、波变量 | 改变信号表示使通道天然无源 |
> | D07 | 万一还是不稳定怎么办？ | PO/PC、TDPA | 在线监测+自适应阻尼=最终安全网 |
> 
> 三者的关系：D05 是**分析工具**（判断问题），D06 是**结构性解决方案**（预防问题），D07 是**运行时安全网**（纠正问题）。工业系统通常三层全用——先用 D05 设计架构和参数，再用 D06 保护通信通道，最后用 D07 作为最后防线。

> **与 D04 的完整回路**：D04 的 ALOHA/ACT 选择了"不做力反馈"的路线——因为 50 Hz + 无力传感器使得 D05-D07 的力反馈理论不可行。但对于配备了力传感器和高频控制器的系统（如 da Vinci、Omega.7+Franka），D05-D07 是实现高质量力反馈遥操作的完整理论工具箱。两条路线不是互斥的——D08 将讨论如何在同一系统中结合两者的优势。
