> 本文档属于 [Robotics Tutorial](https://github.com/Michael-Jetson/Robotics_Tutorial) 项目，作者：Pengfei Guo，达妙科技。采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 协议，转载请注明出处。

# 第 64 章 RL 的 C++ 部署——TorchScript / LibTorch + ONNX Runtime + 实时推理

> **难度**: ⭐⭐ ~ ⭐⭐⭐ | **预计学时**: 25-35 小时(1.5 周) | **text:code = 6:4**（工程实践为主章节）
>
> **一句话概要**: 训好的 RL 策略是 Python 对象——要在 1 kHz 实时循环里以 $< 1$ ms 推理,必须导出为 TorchScript/ONNX,用 LibTorch/ONNX Runtime 在 C++ 中加载,并解决零拷贝数据转换、内存预分配、安全降级等工程问题。

**向前承接**: Ch63 训好了 Go2 的 RL 策略(`.pth` checkpoint)——一个包含 Actor 网络权重的 Python 对象,此时它只能在 Python + PyTorch 环境中运行。Ch61 搭好了实时 C++ 和 ros2_control 框架——其中 `update()` 以 1 kHz 运行,要求每次调用的延迟严格低于 1 ms。Ch62 提供了硬件接口(Unitree SDK)——通过 UDP 协议以 500 Hz 与电机通信。本章把三者打通——策略从 Python 进入 C++ 实时循环。

**向后指向**: Ch65 将探索 RL+MPC 混合范式——其中 RL 部分的部署正是本章内容。Ch69(Mini-Legged 综合实战)将整合本章的部署技术完成真机验证。

---

## 前置自测

📋 **答不出 $\geq$ 2 题 → 先回对应章节复习**

1. **[Ch61]** 什么是 `SCHED_FIFO`?为什么实时控制线程需要用它而不是默认的 CFS 调度器?
2. **[Ch61]** `mlockall(MCL_CURRENT | MCL_FUTURE)` 的作用是什么?不调用它会导致什么实时性问题?
3. **[Ch63]** PPO 训练后得到的 `.pth` checkpoint 文件包含什么内容?为什么不能直接在 C++ 中加载 `.pth`?
4. **[C++ 基础]** `std::memcpy` 与 `std::copy` 在连续内存上的性能差异是什么?什么时候用哪个?
5. **[Ch62]** ros2_control 的 `update()` 函数运行在什么线程上?它的调用频率由什么决定?

---

## 本章目标

学完本章,你应能:

1. 理解从"Python 训好"到"C++ 部署"的完整流水线及其每一步的工程考量
2. 掌握 TorchScript(trace vs script)两种导出方式的内部机制和适用场景
3. 用 LibTorch C++ API 加载并运行 RL 策略,理解 Eigen 与 Tensor 之间的零拷贝转换
4. 用 ONNX Runtime 作为跨框架推理引擎,理解 ONNX 格式的内部结构
5. 在 ros2_control 的 `update()` 中集成 RL 推理,满足 $< 1$ ms 延迟约束
6. 设计完整的安全降级机制——NaN 检测、数值范围检查、备用策略切换
7. 精读 rl_sar 源码,理解腿足 RL 部署的工业级实现

---

## 64.1 RL 模型部署的基本问题 ⭐

### 动机:为什么不直接在 Python 里跑 RL 推理?

一个直觉的问题:Python + PyTorch 能跑推理,为什么还要折腾 C++?

答案在于**实时性和可靠性**的双重约束。让我们系统对比两种方案:

| 维度 | Python + PyTorch | C++ + LibTorch/ONNX |
|------|-----------------|---------------------|
| **推理延迟** | 1-5 ms(含 GIL、解释器开销) | 50-200 $\mu$s |
| **延迟确定性** | 不确定(GC 暂停、GIL 竞争) | 确定(无 GC,可 SCHED_FIFO) |
| **依赖大小** | ~2 GB(Python + PyTorch + CUDA) | ~200 MB(LibTorch)或 ~50 MB(ONNX) |
| **与 ros2_control 集成** | 需要 ROS 2 Python wrapper | 原生 C++ 插件 |
| **GIL 问题** | 阻塞实时线程 | 无 GIL |
| **部署复杂度** | 低(直接用 Python) | 中(需要导出+编译) |

Python 方案在研究环境下可行(如 Gazebo 仿真里跑 RL),但在**真机部署**时有三个致命问题:

1. **GIL (Global Interpreter Lock)**: Python 的 GIL 让多线程形同虚设。如果 RL 推理线程和 ROS 2 通信线程共享 GIL,推理可能在任意时刻被阻塞。
2. **垃圾回收 (GC)**: Python 的 GC 可能在任意时刻暂停程序数十毫秒——Ch61 讲过,这对 1 kHz 控制循环是致命的(5 ms 的暂停就可能让机器人摔倒)。
3. **延迟不确定性**: Python 解释器对每行代码的执行时间不做保证,无法满足硬实时约束。

因此,生产级腿足 RL 部署必须走 C++ 路线。本章的全部内容就是解决"如何优雅地把 Python 训好的模型搬到 C++ 实时循环中"这个工程问题。

> **跨领域类比**：RL 模型从 Python 到 C++ 的部署,类似于芯片设计从 RTL 仿真到流片的过程——仿真环境（Python/PyTorch）中功能验证通过后,必须经过综合、布局布线、时序收敛等一系列工程步骤才能变成可在真实硬件上运行的芯片（C++ 二进制）。两者的共同挑战是:**功能等价性验证**（Python 和 C++ 的推理结果必须一致）和**时序约束满足**（推理延迟必须在截止时间内）。但不同于芯片流片的不可逆性,RL 模型部署可以反复迭代——这是软件部署的优势。

### 如果不用 C++ 部署会怎样

在真实工程中遇到过的问题:

| 问题 | Python 部署的后果 | C++ 部署的解决 |
|------|------------------|--------------|
| GIL 竞争 | RL 推理被 ROS callback 阻塞 300 ms,机器人失控摔倒 | 无 GIL,独立线程不受干扰 |
| GC 暂停 | 控制循环突然停顿 50 ms,摆动腿未能按时着地 | 无 GC(RAII 管理生命周期) |
| 依赖冲突 | PyTorch 版本与 ROS 2 的 Python 环境冲突,安装耗时数天 | LibTorch 是独立 C++ 库,与 ROS 2 无冲突 |
| 嵌入式部署 | Jetson Orin 的 Python 环境配置极其痛苦(交叉编译、conda 不兼容) | 交叉编译 C++ 二进制,干净部署 |

### 两大部署方案概览

把 PyTorch `nn.Module` 变成 C++ 可执行的模型,有两条主路:

**方案 A:TorchScript + LibTorch**(PyTorch 生态内)

```
Python 训练 → torch.jit.trace/script → policy.pt → LibTorch C++ 加载推理
```

优势:与 PyTorch 完全兼容,API 熟悉。劣势:依赖较大(~200 MB)。

**方案 B:ONNX + ONNX Runtime**(跨框架)

```
Python 训练 → torch.onnx.export → policy.onnx → ONNX Runtime C++ 加载推理
```

优势:框架无关,依赖小(~50-100 MB),可接入 TensorRT/OpenVINO 加速。劣势:新算子支持滞后 PyTorch 几个月。

两种方案的详细对比见 64.4 节末尾。先从方案 A 的第一步——TorchScript 导出开始。

### ⚠️ 常见陷阱

> ⚠️ **思维陷阱:认为 Python 部署"够用"**
> **新手想法**: "我在 Gazebo 里用 Python 跑得好好的,真机也可以。"
> **实际情况**: Gazebo 是软实时(偶尔丢帧只是仿真不准确),真机是硬实时(丢帧 = 摔倒 = 硬件损坏)。Python 的不确定性延迟在软实时下可以接受,在硬实时下不可接受。
> **判断标准**: 如果控制频率 $\geq$ 200 Hz 且真机部署,必须 C++。如果只在仿真中验证算法,Python 可以。

> ⚠️ **概念误区:混淆训练和推理的计算量**
> **新手想法**: "训练要用 GPU 几十小时,推理肯定也很慢。"
> **实际情况**: 训练慢是因为反向传播 + 4096 并行环境 + 数千次迭代。推理只是一个小 MLP 的**单次前向传播**:48 维输入 → 256 → 256 → 12 维输出,约 50k 次浮点乘加。现代 CPU 的单核算力约 10 GFLOPS,理论上 $50000 / (10 \times 10^9) \approx 5 \mu s$。实际框架开销让延迟到 50-100 $\mu$s,但仍然极快。
> **关键认知**: RL 策略的推理**极其轻量**,瓶颈不在计算本身,而在框架调用开销。

> ⚠️ **编程陷阱:在 ROS 2 Python 节点中直接调用 PyTorch**
> **错误做法**: 在 `rclpy` 节点的 `timer_callback` 中调 `model.forward(obs)`。
> **后果**: `timer_callback` 与 ROS 2 的 executor 共享 GIL。如果有其他 Python callback(如 subscriber),它们可能抢占 GIL,导致推理延迟不可预测。
> **正确做法**: 用 C++ ros2_control Controller(Ch61 讲过),RL 推理在 Controller 的 `update()` 中原生运行,不经过 Python。

### 练习

**练习 64.1.1**: 估算一个 MLP [48 → 256 → 256 → 12] 的前向传播需要多少次浮点乘加(FLOPs)。具体计算:第一层 $48 \times 256$ 次乘法 + $256$ 次加法;第二层 $256 \times 256$;第三层 $256 \times 12$。假设 CPU 能做 10 GFLOPS,理论推理时间是多少微秒?

**练习 64.1.2**: 列出 Python + PyTorch 推理链路上的所有开销来源(从调用 `model.forward(obs)` 到返回结果)。提示:考虑 Python 解释器、PyTorch dispatch、tensor 创建、autograd 图构建等。哪些开销在 C++ 中可以消除?

---

部署的第一步是把 PyTorch 模型导出为可序列化的格式。下一节深入 TorchScript 的内部机制。

## 64.2 TorchScript 导出:从 `.pth` 到 `.pt` ⭐⭐

### 动机:为什么 `.pth` 不能直接用?

普通 PyTorch checkpoint `.pth` 文件存的是 `state_dict`——即模型参数(权重矩阵和偏置向量)的字典。它**不包含网络结构定义**。要使用 `.pth`,必须先在 Python 中定义网络类(`class ActorCritic(nn.Module): ...`),然后 `model.load_state_dict(torch.load("model.pth"))`。

这意味着:加载 `.pth` 需要 Python 解释器 + 模型定义源代码 + PyTorch 完整环境。在 C++ 中无法直接使用。

TorchScript `.pt` 文件存的是**网络结构 + 权重 + 优化信息**,可以在没有 Python 的环境中独立加载和执行。这曾经是从 Python 到 C++ 的主桥梁。需要注意的是,PyTorch 官方已经将 TorchScript 标记为 deprecated,新项目应优先评估 `torch.export`、ONNX 或 AOTInductor 等路线；本节保留 TorchScript,是因为大量腿足 RL 项目和旧版部署代码仍在使用 LibTorch/TorchScript。

### TorchScript 的内部机制

TorchScript 不是简单的"序列化"——它是一个完整的**中间表示(IR)和编译器**。理解其内部机制有助于排查导出问题。

TorchScript IR 是一个 **静态单赋值(SSA, Static Single Assignment)** 格式的有向无环图(DAG)。其核心组件:

- **Graph**: 根容器,表示一个函数的完整计算图
- **Node**: 表示一个操作(如 `aten::linear`、`aten::elu`、`prim::Constant`)
- **Value**: 表示数据流(输入 tensor、中间结果、输出 tensor),每个 Value 有显式类型
- **Block**: 控制流的基本单元(对应 `if/else` 分支)

IR 经过的**优化 pass**(在 `torch/csrc/jit/passes/` 中实现):

| 优化 Pass | 作用 | 对 RL 策略的影响 |
|-----------|------|-----------------|
| **常量折叠 (Constant Folding)** | 预计算只依赖常量的子图 | 将 bias 的 reshape 等常量操作消除 |
| **死代码消除 (DCE)** | 删除未使用的计算 | 删除 Critic 相关的死代码(如果导出时包含了) |
| **公共子表达式消除 (CSE)** | 合并重复计算 | 如果多个奖励项共享子表达式(不常见) |
| **算子融合 (Operator Fusion)** | 多个算子合并为一个高效内核 | **Linear + ELU 可能被融合**,显著加速 |

这些优化让 TorchScript 导出的模型比 Python eager 模式快 20-50%。

### 两种导出方法

**方法 1:`torch.jit.trace`(推荐用于 RL 策略)**

```python
import torch

# 1. 加载训好的模型
model = ActorCritic(num_obs=48, num_actions=12)
model.load_state_dict(torch.load("checkpoint.pth"))
model.eval()  # 必须!切换到 eval 模式

# 2. 只导出 Actor 部分(部署不需要 Critic)
actor = model.actor

# 3. 用示例输入 trace
example_input = torch.randn(1, 48)  # batch=1, obs_dim=48
traced_actor = torch.jit.trace(actor, example_input)

# 4. 保存
traced_actor.save("policy.pt")

# 5. 验证一致性
with torch.no_grad():
    out_orig = actor(example_input)
    out_traced = traced_actor(example_input)
    max_err = (out_orig - out_traced).abs().max().item()
    assert max_err < 1e-6, f"误差过大: {max_err:.2e}"
    print(f"Trace 验证通过, 最大误差: {max_err:.2e}")
```

**trace 的原理**: 用 `example_input` 做一次完整的前向传播,**记录**所有经过的算子及其数据依赖关系,生成计算图(TorchScript IR)。本质上是一次"录像"——只录制实际执行的路径。

**trace 的局限**:
- **不记录控制流**: `if/else`、`while` 等分支不被记录——只记录 trace 时实际走过的那条路径。如果另一个输入会走不同分支,trace 结果就是错的。
- **不记录动态 shape**: 如果模型对不同 batch size 有不同行为,需要额外处理。

**对 RL 策略为什么 trace 就够了**: RL 策略网络通常是纯前馈 MLP(Linear → ELU → Linear → ELU → Linear),没有条件分支,输入 shape 固定(batch=1,obs_dim=48)。trace 完美适用。

**方法 2:`torch.jit.script`(编译式,更严格)**

```python
scripted_actor = torch.jit.script(actor)
scripted_actor.save("policy.pt")
```

**script 的原理**: 直接**编译** Python 源代码为 TorchScript IR。支持 `if/else`、`for`、`while` 等控制流,但要求代码符合 TorchScript 的语法子集(不支持某些 Python 特性,如动态类型、列表推导式等)。

**选型建议**:
- 纯 MLP 策略(无分支,无 RNN)→ **trace**(简单可靠)
- 有 RNN(GRU/LSTM)或条件分支 → **script**(更完整)
- 不确定 → 先尝试 trace;如果 trace 后的输出不正确,再用 script

### 完整的导出脚本

```python
"""export_policy.py — 导出 rsl_rl 训好的策略为 TorchScript"""
import torch
import torch.nn as nn
import argparse

def export_policy(checkpoint_path, output_path, obs_dim=48, act_dim=12):
    # 1. 重建 Actor 网络结构(必须和训练时一致!)
    # rsl_rl 默认: 3 层 MLP, 隐藏维度 256, 激活函数 ELU
    actor = nn.Sequential(
        nn.Linear(obs_dim, 256),
        nn.ELU(),           # 注意: rsl_rl 默认用 ELU, 不是 ReLU!
        nn.Linear(256, 256),
        nn.ELU(),
        nn.Linear(256, 256),
        nn.ELU(),
        nn.Linear(256, act_dim),
    )

    # 2. 从 checkpoint 提取 Actor 权重
    ckpt = torch.load(checkpoint_path, map_location='cpu')
    actor_state = {}
    for key, val in ckpt['model_state_dict'].items():
        if key.startswith('actor.'):
            actor_state[key.replace('actor.', '')] = val
    actor.load_state_dict(actor_state)

    # 3. 切换到 eval 模式
    actor.eval()

    # 4. Trace
    example = torch.randn(1, obs_dim)
    traced = torch.jit.trace(actor, example)

    # 5. 冻结(将参数内联为常量,进一步优化)
    traced = torch.jit.freeze(traced)

    # 6. 保存
    traced.save(output_path)
    print(f"导出成功: {output_path}")

    # 7. 加载验证
    with torch.no_grad():
        out_orig = actor(example)
        loaded = torch.jit.load(output_path)
        out_loaded = loaded(example)
        max_err = (out_orig - out_loaded).abs().max().item()
        print(f"最大误差: {max_err:.2e}")
        assert max_err < 1e-5, f"误差过大: {max_err}"

    # 8. 多输入验证(确保不是偶然一致)
    for i in range(10):
        test_input = torch.randn(1, obs_dim)
        err = (actor(test_input) - loaded(test_input)).abs().max().item()
        assert err < 1e-5, f"输入 {i} 误差过大: {err}"
    print("10 个随机输入验证全部通过")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", required=True, help="checkpoint .pth 路径")
    parser.add_argument("--out", default="policy.pt", help="输出 .pt 路径")
    parser.add_argument("--obs_dim", type=int, default=48)
    parser.add_argument("--act_dim", type=int, default=12)
    args = parser.parse_args()
    export_policy(args.ckpt, args.out, args.obs_dim, args.act_dim)
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:导出时未切换到 eval 模式**
> **错误做法**: 直接 `torch.jit.trace(model, example)`,model 仍在 train 模式。
> **后果**: 如果网络有 BatchNorm 或 Dropout,train 模式和 eval 模式行为不同。train 模式下 BatchNorm 使用 mini-batch 统计量(随机的),eval 模式使用全局统计量(固定的)。导出的模型在不同输入上可能给出不一致的输出。
> **正确做法**: 导出前必须 `model.eval()`。

> ⚠️ **编程陷阱:rsl_rl 的激活函数是 ELU 不是 ReLU**
> **错误做法**: 导出时用 `nn.ReLU()` 重建网络结构。
> **后果**: 权重加载不会报错(因为激活函数没有可学习参数,`state_dict` 不含激活函数的 key),但推理结果完全错误。ELU 对负值输出 $\alpha(e^x - 1)$,ReLU 直接截断为 0——行为差异巨大。
> **正确做法**: 检查 rsl_rl 训练配置中的 `activation` 参数。默认是 `nn.ELU()`。

> ⚠️ **概念误区:认为 trace 和 script 在所有情况下效果相同**
> **新手想法**: "都能导出 TorchScript,用哪个都一样。"
> **实际情况**: 对于纯 MLP 两者确实等价。但如果网络有数据依赖的分支(如 `if obs[0] > 0: use_path_A() else: use_path_B()`),trace 只记录一条路径——另一条路径会被丢弃。script 则编译所有分支。
> **最佳实践**: 先尝试 trace;导出后用多个随机输入验证输出一致性。如果发现不一致,说明有条件分支,需要用 script。

### 练习

**练习 64.2.1**: 写一个完整的 `export.py` 脚本,加载你在 Ch63 训好的 Go2 checkpoint,导出为 TorchScript `.pt` 文件。用 10 个随机输入验证导出前后的输出误差均 < 1e-6。

**练习 64.2.2**: 用 `torch.jit.trace` 导出一个包含 `if-else` 分支的简单网络(`if x.sum() > 0: return linear1(x) else: return linear2(x)`)。观察导出的模型在两种输入(正/负)上是否都正确。然后用 `torch.jit.script` 重新导出,对比结果。

**练习 64.2.3**: 查看导出的 `.pt` 文件的 TorchScript IR:用 `traced.graph` 打印计算图。能否在图中找到 ELU 算子?`torch.jit.freeze` 前后图有什么变化?

---

TorchScript 把模型从 Python 解放出来。下一步是在 C++ 中加载并运行它——这需要 LibTorch。

> **本质洞察**：TorchScript / ONNX 等模型导出格式的本质是**将计算图与编程语言解耦**——就像 PDF 将文档排版与创作工具解耦一样。Python 是创作工具（训练方便），C++ 是阅读器（运行高效），而导出格式是两者之间的"通用表示层"。理解了这一点，你就能预判所有导出问题的根源：凡是无法被序列化为纯计算图的 Python 动态特性（控制流、动态 shape、自定义算子），都会在导出时出问题。

## 64.3 LibTorch C++ 加载与推理 ⭐⭐

### 动机:LibTorch 是什么?

LibTorch 是 PyTorch 的 **C++ 运行时**。它提供了与 Python PyTorch 几乎对等的 API:tensor 创建/操作、自动微分、JIT 模型加载。但它是纯 C++ 库,不依赖 Python 解释器。

LibTorch 与 PyTorch 版本号同步,可从 PyTorch 官网下载预构建包,按平台区分:CPU / CUDA 11.8 / CUDA 12.x / ARM64(Jetson)等变体。建议使用与训练端 PyTorch 相同的主版本号,以避免算子兼容性问题。

### LibTorch 与 Python PyTorch API 的对照

学过 Python PyTorch 的开发者可以快速上手 LibTorch——两者 API 高度对称:

| 操作 | Python PyTorch | LibTorch (C++) |
|------|---------------|----------------|
| 创建零 tensor | `torch.zeros(3, 3)` | `torch::zeros({3, 3})` |
| 指定数据类型 | `dtype=torch.float32` | `torch::dtype(torch::kFloat32)` |
| 指定设备 | `device='cuda:0'` | `torch::Device(torch::kCUDA, 0)` |
| 加载 TorchScript | `torch.jit.load("p.pt")` | `torch::jit::load("p.pt")` |
| 前向推理 | `model(input)` | `module.forward({input}).toTensor()` |
| 禁用梯度 | `with torch.no_grad():` | `torch::NoGradGuard no_grad;` |
| tensor→指针 | `t.data_ptr()` | `t.data_ptr<float>()` |
| 从外部数据创建 | `torch.from_numpy(arr)` | `torch::from_blob(ptr, shape)` |

### 最小化 C++ 推理代码

下面是一个完整的、可在 ros2_control Controller 中使用的 RL 推理类。每一行都有详细注释解释"为什么这样写":

```cpp
#include <torch/script.h>   // TorchScript 加载
#include <torch/torch.h>     // tensor 操作
#include <Eigen/Dense>       // 机器人状态用 Eigen 表示
#include <string>
#include <chrono>

class RLPolicy {
 public:
  /**
   * 构造函数: 加载模型 + 预分配内存 + warmup
   *
   * 为什么在构造函数中做所有初始化?
   * 因为构造函数在 ros2_control 的 on_configure() 阶段调用,
   * 此时不在实时循环中,可以有较大延迟(秒级)。
   * 而 forward() 在 update() 实时循环中调用,必须极快。
   */
  RLPolicy(const std::string& model_path,
           int obs_dim, int act_dim,
           const std::string& device = "cpu")
      : obs_dim_(obs_dim), act_dim_(act_dim) {

    // 1. 加载 TorchScript 模型
    try {
      module_ = torch::jit::load(model_path);
    } catch (const c10::Error& e) {
      throw std::runtime_error(
          "Failed to load TorchScript model: " + std::string(e.what()));
    }
    module_.eval();  // 确保 eval 模式(关闭 dropout/batchnorm 的训练行为)

    // 2. 设置设备
    // ⚠️ 实时部署建议使用 CPU：CUDA 的 host↔device memcpy 延迟不确定，
    // 不适合 1 kHz 硬实时循环。GPU 推理适合 >10ms 预算的场景。
    device_ = torch::Device(device);
    module_.to(device_);

    // 3. 预分配 tensor(关键的实时编程技巧,Ch61)
    // 启动时分配一次,运行时只填充数据,永不释放/重建
    input_tensor_ = torch::zeros(
        {1, obs_dim_},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_)
    );

    // 4. 预分配 Eigen 输出向量
    action_ = Eigen::VectorXd::Zero(act_dim_);

    // 5. 预分配 float 缓冲区(用于 double→float 转换)
    obs_float_buffer_.resize(obs_dim_, 0.0f);

    // 6. Warmup(消除 JIT 首次编译延迟)
    warmup(20);
  }

  /**
   * 前向推理——在 1 kHz 实时循环中调用
   * 设计目标: < 200 us 延迟, 零堆分配
   */
  const Eigen::VectorXd& forward(const Eigen::VectorXd& obs) {
    // Step 1: Eigen (double) → float 缓冲区
    // 为什么不直接用 from_blob? 因为 Eigen 是 double, 网络要 float
    for (int i = 0; i < obs_dim_; ++i) {
      obs_float_buffer_[i] = static_cast<float>(obs[i]);
    }

    // Step 2: 填充预分配的 input tensor(零拷贝,只覆盖数据)
    std::memcpy(input_tensor_.data_ptr<float>(),
                obs_float_buffer_.data(),
                obs_dim_ * sizeof(float));

    // Step 3: 前向推理(核心,~50-100 us)
    torch::NoGradGuard no_grad;  // 禁用 autograd,节省 ~20% 时间
    auto output = module_.forward({input_tensor_}).toTensor();

    // Step 4: tensor (float) → Eigen (double)
    auto output_accessor = output.cpu().accessor<float, 2>();
    for (int i = 0; i < act_dim_; ++i) {
      action_[i] = static_cast<double>(output_accessor[0][i]);
    }

    return action_;
  }

  /**
   * 获取推理延迟统计
   */
  double getLastInferenceTimeUs() const { return last_inference_us_; }

 private:
  void warmup(int n) {
    // LibTorch 第一次前向传播会触发 JIT 编译:
    // TorchScript IR → 当前 CPU/GPU 的优化机器码
    // 这个编译可能耗时 100-500 ms
    // 在启动阶段做 n 次 warmup,确保实时循环中不触发编译
    torch::NoGradGuard no_grad;
    for (int i = 0; i < n; ++i) {
      module_.forward({input_tensor_});
    }
  }

  torch::jit::script::Module module_;
  torch::Device device_{torch::kCPU};
  torch::Tensor input_tensor_;
  Eigen::VectorXd action_;
  std::vector<float> obs_float_buffer_;
  int obs_dim_;
  int act_dim_;
  double last_inference_us_ = 0.0;
};
```

### Eigen 与 Tensor 的数据转换:细节与陷阱

Eigen 和 PyTorch Tensor 的内存布局有关键差异:

| 维度 | Eigen | PyTorch Tensor |
|------|-------|---------------|
| **默认存储顺序** | 列优先(Column-major) | 行优先(Row-major, C-contiguous) |
| **默认数据类型** | `double` (float64) | `float` (float32) |
| **内存管理** | 栈分配(固定大小)或堆(动态大小) | 堆分配,引用计数 |

**方案 A:零拷贝(`from_blob`)——快但有陷阱**

```cpp
// Eigen::VectorXf → torch::Tensor (零拷贝)
Eigen::VectorXf obs_f(48);
// ... 填充 obs_f ...

auto tensor = torch::from_blob(
    obs_f.data(),     // 指向 Eigen 内存的指针
    {1, 48},          // shape
    torch::kFloat32   // dtype
);
// 注意: tensor 不拥有这块内存!
// 如果 obs_f 被销毁或超出作用域,tensor 变成悬垂引用。
// 在实时循环中, obs_f 是成员变量(不会销毁),所以安全。
```

**方案 B:拷贝(`memcpy`)——安全且对小 tensor 不比 from_blob 慢多少**

```cpp
// 预分配 tensor(构造时)
torch::Tensor input = torch::zeros({1, 48}, torch::kFloat32);

// 每次循环: 拷贝数据到预分配 tensor
std::memcpy(input.data_ptr<float>(), obs_float.data(), 48 * sizeof(float));
// 48 个 float = 192 字节, memcpy 耗时 < 1 us
```

**推荐方案**: 对于 RL 策略部署(obs_dim < 200),方案 B(memcpy 到预分配 tensor)更安全且性能充足。方案 A 的零拷贝优势在 < 200 维时可以忽略。

### 性能数据

典型 MLP [48 → 256 → 256 → 256 → 12] 的单次推理延迟:

| 平台 | CPU 推理 | GPU 推理 | 说明 |
|------|---------|---------|------|
| x86 i7-12700 | ~50 $\mu$s | ~200 $\mu$s | GPU 反而更慢(核启动+PCIe 拷贝) |
| Jetson Orin (ARM) CPU | ~100 $\mu$s | — | ARM CPU 较慢 |
| Jetson Orin GPU | — | ~150 $\mu$s | GPU 开销依然存在 |
| Jetson Nano (ARM) CPU | ~300 $\mu$s | — | 老平台,较慢但够用 |

**关键发现**: 对于参数量 < 100K 的小 MLP,**CPU 推理通常比 GPU 推理更快**。原因:GPU 的核启动(kernel launch ~10 $\mu$s)和数据拷贝(CPU→GPU→CPU,经 PCIe 总线)的固定开销超过了 GPU 并行计算的收益。只有当网络较大(参数量 > 1M,如 CNN 视觉策略)时,GPU 加速才有意义。

> **本质洞察**：RL 策略部署的性能瓶颈**不是计算本身,而是框架调用开销**。一个 [48→256→256→12] 的 MLP 前向传播理论上只需 ~5 $\mu$s 的浮点运算,但 LibTorch/ONNX Runtime 的框架开销（tensor 元数据管理、算子调度、内存管理）将实际延迟推高到 50-100 $\mu$s——框架开销是计算本身的 10-20 倍。这解释了为什么"预分配 + 零拷贝 + warmup"这些看似琐碎的工程技巧如此关键——它们消除的不是计算量,而是框架开销。

### 实时循环集成

```cpp
// 在 ros2_control Controller 的 update() 里集成 RL 推理
controller_interface::return_type
RLController::update(const rclcpp::Time& time,
                     const rclcpp::Duration& period) {
  // 1. 从 state_interfaces 读取传感器数据
  readSensorData();  // 填充 joint_positions_, joint_velocities_, imu_ 等

  // 2. 构建观测向量(必须与训练时的观测构建一致!)
  Eigen::VectorXd obs = buildObservation(
      joint_positions_, joint_velocities_,
      imu_angular_vel_, projected_gravity_,
      velocity_command_, last_action_
  );  // shape: (obs_dim_,)

  // 3. RL 推理 (~50-100 us)
  const Eigen::VectorXd& action = policy_->forward(obs);

  // 4. 动作后处理
  // action 是关节位置偏移量, 加上默认站立姿态 -> 关节目标位置
  Eigen::VectorXd q_des = default_joint_pos_ + action * action_scale_;

  // 5. 安全检查 (64.7 节详述)
  if (!safety_guard_->checkAction(q_des, joint_positions_, last_q_des_)) {
    q_des = safety_guard_->getFallbackAction(last_safe_q_des_, joint_positions_);
  }

  // 6. 写入 command_interfaces
  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(q_des[i]);
  }

  last_action_ = action;
  last_safe_q_des_ = q_des;
  return controller_interface::return_type::OK;
}
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:每次推理都创建新的 tensor**
> **错误做法**: 在 `update()` 中 `auto tensor = torch::zeros({1, 48});` 或 `torch::from_blob(...)`。
> **问题**: 即使 `from_blob` 不分配 tensor 数据内存,它也会创建 TensorImpl 元数据对象(涉及堆分配+引用计数)。在 1 kHz 循环中,每秒 1000 次堆分配 → 内存碎片化 + 可能触发 GC(如果用了 shared_ptr)。
> **正确做法**: 构造时预分配 tensor,每次循环只用 `memcpy` 覆盖数据。

> ⚠️ **编程陷阱:`from_blob` 的生命周期陷阱**
> **错误做法**: `from_blob` 引用了一个函数局部变量的数据:
> ```cpp
> torch::Tensor makeTensor() {
>     float data[48] = {...};
>     return torch::from_blob(data, {1, 48}); // data 在栈上,函数返回后失效!
> }
> ```
> **后果**: 返回的 tensor 指向已销毁的栈内存——未定义行为(可能段错误,也可能读到"看起来正确"的垃圾数据,后者更难调试)。
> **正确做法**: 确保 `from_blob` 引用的数据在 tensor 使用期间始终有效(如类成员变量)。或者直接用 `memcpy` 拷贝到预分配 tensor。

> ⚠️ **概念误区:在实时循环中用 GPU 推理小 MLP**
> **新手想法**: "GPU 更快,推理当然要用 GPU。"
> **实际情况**: 对于 < 100K 参数的 MLP,GPU 推理的固定开销(核启动 + PCIe 数据往返)~200 $\mu$s,远大于 CPU 推理时间 ~50 $\mu$s。如果整个控制循环在 CPU 上,GPU 推理意味着每步都有 CPU→GPU→CPU 的数据往返,反而更慢。
> **正确做法**: 小网络用 CPU 推理。大网络(如 CNN 视觉策略)或已有 GPU 数据管道时才用 GPU。

### 练习

**练习 64.3.1**: 写一个最小 C++ 程序:在 Python 中创建 MLP [3→8→1],导出为 TorchScript;在 C++ 中加载,传入 `[1, 2, 3]`,验证输出与 Python 一致(误差 < 1e-6)。给出完整的 `CMakeLists.txt`。

**练习 64.3.2**: 在练习 64.3.1 基础上,用 `std::chrono::high_resolution_clock` 测量 1000 次推理的平均延迟和 P99 延迟。对比有/无 `torch::NoGradGuard` 的性能差异(百分比)。

**练习 64.3.3**: 实现 Eigen→Tensor 的两种转换方式(零拷贝 `from_blob` vs 拷贝 `memcpy`),各测 10000 次,对比单次转换延迟。对于 48 维向量,差异有多大?

---

LibTorch 与 PyTorch 生态紧密耦合。如果你想脱离 PyTorch、追求更小的部署包或使用 TensorRT 加速,ONNX Runtime 是另一个强力选择。

## 64.4 ONNX + ONNX Runtime 部署 ⭐⭐

### 动机:为什么考虑 ONNX?

ONNX(Open Neural Network Exchange)是微软、Meta 等联合推出的**跨框架模型表示格式**。使用 ONNX 的三大动机:

1. **框架无关**: 不依赖 PyTorch——未来如果团队换 JAX/TensorFlow 训练,部署代码不变
2. **部署包小**: ONNX Runtime ~50-100 MB vs LibTorch ~200 MB——在嵌入式平台(Jetson)上更友好
3. **多加速后端**: 可以接入 TensorRT(NVIDIA GPU 极致优化)、OpenVINO(Intel CPU)、CoreML(Apple M 系列芯片)等——同一个 ONNX 模型,不同平台用不同后端

ONNX Runtime 保持频繁的发布节奏,请以官方 GitHub Releases 页面确认当前版本及其编译器和 CUDA 版本要求。

### ONNX 格式的内部结构

ONNX 文件是一个 **protobuf** 序列化的计算图。理解其结构有助于排查导出和推理问题:

```
ONNX Model (protobuf 序列化)
├── ir_version: 9
├── opset_import: [{domain: "", version: 17}]
├── graph: GraphProto
│   ├── name: "policy"
│   ├── input: [ValueInfoProto("obs", [1, 48], FLOAT)]
│   ├── output: [ValueInfoProto("action", [1, 12], FLOAT)]
│   ├── node: [                       # 计算图的算子节点
│   │   NodeProto(op="Gemm", in=["obs","w0","b0"], out=["gemm0"]),  # Linear
│   │   NodeProto(op="Elu", in=["gemm0"], out=["elu0"]),
│   │   NodeProto(op="Gemm", in=["elu0","w1","b1"], out=["gemm1"]),
│   │   ...
│   ]
│   └── initializer: [               # 模型权重(作为常量嵌入)
│       TensorProto("w0", [48, 256], data=[...]),
│       TensorProto("b0", [256], data=[...]),
│       ...
│   ]
```

每个 `NodeProto` 对应一个 ONNX 算子(如 `Gemm` = General Matrix Multiply = Linear 层,`Elu` = ELU 激活)。算子的语义由 opset 版本决定。

### ONNX 导出

```python
import torch

model = load_trained_actor()  # 加载训好的 Actor 网络
model.eval()

example_input = torch.randn(1, 48)

torch.onnx.export(
    model,
    example_input,
    "policy.onnx",
    export_params=True,        # 将权重嵌入 ONNX 文件
    opset_version=17,          # opset 17 支持 ELU 等算子
    input_names=["obs"],       # 输入名称(C++ 推理时要用)
    output_names=["action"],   # 输出名称
    dynamic_axes=None,         # RL 用固定 batch size=1
    do_constant_folding=True,  # 常量折叠优化
)

# 验证
import onnx
onnx_model = onnx.load("policy.onnx")
onnx.checker.check_model(onnx_model)
print("ONNX 模型验证通过")

# 用 onnxruntime 在 Python 中验证数值一致性
import onnxruntime as ort
import numpy as np
sess = ort.InferenceSession("policy.onnx")
onnx_out = sess.run(["action"], {"obs": example_input.numpy()})[0]
torch_out = model(example_input).detach().numpy()
max_err = np.abs(onnx_out - torch_out).max()
print(f"ONNX vs PyTorch 最大误差: {max_err:.2e}")
assert max_err < 1e-5
```

### C++ ONNX Runtime 推理

```cpp
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#include <vector>

class ONNXPolicy {
 public:
  ONNXPolicy(const std::string& model_path, int obs_dim, int act_dim)
      : obs_dim_(obs_dim), act_dim_(act_dim),
        env_(ORT_LOGGING_LEVEL_WARNING, "rl_policy"),
        session_(env_, model_path.c_str(), Ort::SessionOptions{}) {

    // 预分配缓冲区
    input_data_.resize(obs_dim, 0.0f);
    input_shape_ = {1, static_cast<int64_t>(obs_dim)};
    action_ = Eigen::VectorXd::Zero(act_dim);

    // 创建 MemoryInfo(描述内存在 CPU 上)
    memory_info_ = Ort::MemoryInfo::CreateCpu(
        OrtDeviceAllocator, OrtMemTypeCPU
    );
  }

  const Eigen::VectorXd& forward(const Eigen::VectorXd& obs) {
    // 1. Eigen (double) → float 缓冲区
    for (int i = 0; i < obs_dim_; ++i) {
      input_data_[i] = static_cast<float>(obs[i]);
    }

    // 2. 创建 Ort::Value(引用预分配缓冲区,不拷贝)
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_data_.data(), input_data_.size(),
        input_shape_.data(), input_shape_.size()
    );

    // 3. 推理
    const char* input_names[] = {"obs"};
    const char* output_names[] = {"action"};
    auto output_tensors = session_.Run(
        Ort::RunOptions{nullptr},
        input_names, &input_tensor, 1,
        output_names, 1
    );

    // 4. 提取输出
    float* out_ptr = output_tensors[0].GetTensorMutableData<float>();
    for (int i = 0; i < act_dim_; ++i) {
      action_[i] = static_cast<double>(out_ptr[i]);
    }

    return action_;
  }

 private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_{nullptr};
  std::vector<float> input_data_;
  std::vector<int64_t> input_shape_;
  Eigen::VectorXd action_;
  int obs_dim_, act_dim_;
};
```

### ONNX Runtime C++ API 详解 ⭐⭐

ONNX Runtime 的 C++ API 与 LibTorch 有显著风格差异。理解其核心概念对于正确使用至关重要：

**四大核心对象**：

| 对象 | 生命周期 | 职责 | 类比 |
|------|---------|------|------|
| `Ort::Env` | 全局唯一 | 运行时环境，管理线程池和日志 | 类似 CUDA context |
| `Ort::SessionOptions` | 创建 Session 前设置 | 配置优化级别、执行后端 | 类似 CMake 选项 |
| `Ort::Session` | 每个模型一个 | 持有加载的模型，执行推理 | 类似 `torch::jit::Module` |
| `Ort::Value` | 每次推理创建 | 张量数据容器（可引用外部内存） | 类似 `torch::Tensor` |

**Session Options 的关键配置**：

```cpp
Ort::SessionOptions opts;

// 优化级别：ORT_ENABLE_ALL 启用所有图优化（算子融合、常量折叠等）
opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

// 线程数：RL 推理通常设 1（避免多线程调度开销）
opts.SetIntraOpNumThreads(1);
opts.SetInterOpNumThreads(1);

// 执行模式：顺序执行（确定性延迟）
opts.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);

// 可选：启用 CUDA 加速（Jetson 上推荐搭配 TensorRT）
// OrtCUDAProviderOptions cuda_opts;
// cuda_opts.device_id = 0;
// opts.AppendExecutionProvider_CUDA(cuda_opts);
```

**零拷贝推理模式**：ONNX Runtime 支持直接引用外部内存创建张量，避免数据拷贝：

```cpp
// 零拷贝模式：Ort::Value 直接引用 input_data_ 的内存
// 推理时不需要 memcpy，节省 ~5-10 us
Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info_,
    input_data_.data(),           // 直接引用已有缓冲区
    input_data_.size(),           // 元素数量
    input_shape_.data(),          // shape 数组
    input_shape_.size()           // shape 维度数
);
// 注意：input_data_ 的生命周期必须覆盖整个 Run() 调用
// 如果 input_data_ 在 Run() 返回前被销毁，行为未定义
```

### TorchScript vs ONNX 部署的定量对比 Benchmark ⭐⭐

以下基准测试在三个平台上对比了两种部署方案，使用的模型是典型的腿足 RL 策略（3 层 MLP [256, 128, 12]，输入 48 维，输出 12 维）：

**平台 A：Intel i7-12700H（台式机/NUC）**

| 指标 | LibTorch (CPU) | ONNX Runtime (CPU) | ONNX + OpenVINO |
|------|---------------|-------------------|-----------------|
| 平均延迟 | ~48 $\mu$s | ~35 $\mu$s | ~28 $\mu$s |
| P99 延迟 | ~82 $\mu$s | ~55 $\mu$s | ~42 $\mu$s |
| 最大延迟 | ~350 $\mu$s | ~120 $\mu$s | ~95 $\mu$s |
| 内存占用 | ~210 MB | ~65 MB | ~70 MB |

**平台 B：Jetson Orin NX（15W 模式）**

| 指标 | LibTorch (CPU) | LibTorch (CUDA) | ONNX (CPU) | ONNX + TensorRT |
|------|---------------|----------------|-----------|-----------------|
| 平均延迟 | ~95 $\mu$s | ~120 $\mu$s | ~65 $\mu$s | ~25 $\mu$s |
| P99 延迟 | ~180 $\mu$s | ~250 $\mu$s | ~100 $\mu$s | ~45 $\mu$s |
| GPU 占用 | 0% | ~5% | 0% | ~3% |

**关键发现**：
- 对于小 MLP，**CPU 推理通常比 GPU 推理更快**（LibTorch CUDA 的约 120 $\mu$s > CPU 的约 95 $\mu$s）——因为 CPU-GPU 数据传输开销大于 GPU 计算节省
- ONNX + TensorRT 在 Jetson 上通常是最优选择（约 25 $\mu$s），因为 TensorRT 将 MLP 编译为 Jetson GPU 架构的优化 kernel
- LibTorch 的 P99 延迟波动较大（最大延迟约 350 $\mu$s vs ONNX 的约 120 $\mu$s），可能原因是 LibTorch 的内存分配器偶尔触发 GC

**Jetson 部署实战的工程细节**：

```bash
# Jetson 上安装 ONNX Runtime + TensorRT
# 1. 确认 JetPack 版本（决定 CUDA 和 TensorRT 版本）
cat /etc/nv_tegra_release
dpkg -l | grep tensorrt

# 2. 安装 ONNX Runtime（需要匹配 JetPack 版本的预构建包）
pip3 install onnxruntime-gpu  # Python 端验证用

# 3. C++ 端：从 ONNX Runtime GitHub Releases 下载 aarch64 预构建包
# 注意选择与 JetPack CUDA 版本匹配的 release
```

**TensorRT Engine 缓存**：TensorRT 将 ONNX 模型编译为针对特定 GPU 架构的优化 engine。首次编译需要 30-60 秒，但编译结果可以缓存：

```cpp
// 启用 TensorRT 的 engine 缓存
OrtTensorRTProviderOptions trt_opts;
trt_opts.trt_engine_cache_enable = 1;
trt_opts.trt_engine_cache_path = "/tmp/trt_cache/";
// 首次 Run() 会触发 TensorRT 编译（30-60s），后续直接从缓存加载
```

> **⚠️ 陷阱：TensorRT engine 不跨 GPU 架构兼容**
>
> 在 Jetson Orin NX（Ampere 架构）上编译的 TensorRT engine 不能在 Jetson Xavier NX（Volta 架构）上运行。每次更换硬件或更新 JetPack 版本，都需要重新编译 engine。正确做法是在部署脚本中检测缓存是否有效，无效则自动重新编译。

### LibTorch vs ONNX Runtime:完整工程权衡

| 维度 | LibTorch | ONNX Runtime |
|------|----------|--------------|
| **生态绑定** | PyTorch 原生 | 跨框架(PyTorch/TF/JAX) |
| **依赖大小** | ~200 MB (CPU) | ~50-100 MB |
| **推理速度 (小 MLP)** | ~50 $\mu$s | ~30-80 $\mu$s |
| **多后端加速** | CPU + CUDA | CPU + CUDA + TensorRT + OpenVINO + CoreML |
| **新 Op 支持** | 即时(与 PyTorch 同步) | 滞后 PyTorch 数月 |
| **C++ API 风格** | 类 PyTorch(熟悉) | 独立 API(需要学习) |
| **Jetson 支持** | 官方预构建 | 官方预构建 + TensorRT 后端 |
| **版本兼容性** | TorchScript 有版本要求 | ONNX opset 向后兼容好 |

**选型推荐**:

| 场景 | 推荐 | 理由 |
|------|------|------|
| 研究项目/快速迭代 | LibTorch | API 与 PyTorch 一致,迭代最快 |
| 生产部署/资源受限 | ONNX Runtime | 更小、更快、更易部署 |
| Jetson GPU 加速 | ONNX + TensorRT | TensorRT 在 NVIDIA GPU 上有极致优化 |
| 多框架团队 | ONNX Runtime | 不绑定 PyTorch |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:ONNX opset 版本不够高**
> **错误做法**: 导出时用 `opset_version=9`,但网络用了 opset 9 不支持的算子。
> **后果**: 导出报错 `Unsupported: ONNX export of operator xxx`。
> **正确做法**: 使用能覆盖模型所用全部算子的最低 opset 版本,导出后用 `onnx.checker.check_model()` 验证完整性。

> ⚠️ **概念误区:认为 ONNX Runtime 一定比 LibTorch 快**
> **新手想法**: "ONNX Runtime 更轻量,推理肯定更快。"
> **实际情况**: 对于小 MLP(< 100K 参数),两者速度差异在 20% 以内,取决于平台和编译优化。ONNX Runtime 的真正优势在 TensorRT/OpenVINO 加速——但这些对小网络的加速也有限。
> **最佳实践**: 对你的具体模型和目标平台做 benchmark,不凭假设选型。

### 练习

**练习 64.4.1**: 把 Ch63 训好的策略同时导出为 TorchScript (.pt) 和 ONNX (.onnx)。在 C++ 中分别加载两者,用 10 个随机输入验证输出一致性(互相之间误差 < 1e-5)。

**练习 64.4.2**: 在同一平台上,对 LibTorch 和 ONNX Runtime 做 1000 次推理的 benchmark,记录平均延迟、P99 延迟、最大延迟。哪个更快?差距多大?

---

选好了推理引擎,下一步是精读一个完整的开源部署框架——rl_sar,理解工业级部署的全部细节。

## 64.5 rl_sar 源码精读:观测一致性是 sim-to-real 的关键 ⭐⭐

### 动机:为什么精读 rl_sar?

rl_sar (`fan-ziqi/rl_sar`) 是目前最活跃的开源腿足 RL C++ 部署框架。"sar" 代表 "Simulation And Real"——它同时支持仿真验证和真机部署。截至 2026 年,rl_sar 支持 Unitree A1/Go1/Go2/Go2W/B2/B2W/G1 等多款机器人,构建方式包括 ROS 1、ROS 2 和纯 CMake。

同一作者维护的 `robot_lab` 仓库提供了 IsaacLab 上的训练端,与 rl_sar 形成完整的 train-deploy 流水线。

rl_sar 的核心价值:**它已经踩过了部署中的所有坑**——观测归一化、关节顺序映射、动作缩放、安全检查。精读它比从零踩坑高效得多。

### 观测构建的核心逻辑

```cpp
// rl_sar 的观测构建(核心,简化版)
torch::Tensor RL::ComputeObservation() {
    // 每个观测项都乘以 obs_scale —— 与训练时必须完全一致
    torch::Tensor obs = torch::cat({
        // 1. 基座角速度(从 IMU 读取,转换到身体坐标系)
        QuatRotateInverse(base_quat_, base_ang_vel_) * obs_scales_.ang_vel,

        // 2. 投影重力(判断身体倾斜程度)
        QuatRotateInverse(base_quat_, gravity_vec_),

        // 3. 速度命令(vx, vy, wz)
        commands_ * commands_scale_,

        // 4. 关节位置偏差(相对于默认站立姿态)
        (dof_pos_ - default_dof_pos_) * obs_scales_.dof_pos,

        // 5. 关节速度
        dof_vel_ * obs_scales_.dof_vel,

        // 6. 上一步动作
        actions_,
    }, /*dim=*/1);  // 拼接为 (1, obs_dim) tensor

    // clip 到合理范围(防止异常传感器值导致网络输出异常)
    obs = torch::clamp(obs, -clip_obs_, clip_obs_);
    return obs;
}
```

### 观测归一化一致性——sim-to-real 最容易出错的地方

训练和部署的观测归一化**必须完全一致**。这不是"大致对就行"——任何一个 scale 参数的不一致都会导致策略行为完全错误:

| 观测项 | 训练时处理 | 部署时处理 | 不一致的后果 |
|--------|----------|----------|-------------|
| 关节位置 | `(q - q_default) * 1.0` | 必须相同 | `q_default` 不一致 → 策略感知到"所有关节都偏了" |
| 关节速度 | `dq * 0.05` | 必须相同 | scale 不一致 → 策略对速度感知失调 |
| 角速度 | `omega * 0.25` | 必须相同 | scale 差异 → 过度/不足响应旋转 |
| 命令 | `[vx*2.0, vy*2.0, wz*0.25]` | 必须相同 | 命令 scale 错误 → 机器人速度异常 |
| 重力投影 | 用四元数旋转 | 必须相同 | 旋转方向错误 → 策略以为身体倒了 |

**实操清单**:导出策略后,把训练配置文件中**每一个** `obs_scales` 参数抄到 C++ 部署代码中。不要凭记忆,逐项对照。

> **跨领域类比**：观测归一化一致性的要求,就像编译器的 ABI（Application Binary Interface）兼容性——训练端和部署端必须就"每个字段的含义、顺序、缩放"达成完全一致的协议。如果训练端认为观测向量的第 4-6 位是 `[vx*2.0, vy*2.0, wz*0.25]` 但部署端写成了 `[vx, vy, wz]`,就像 C++ 编译的 struct padding 不一致一样——表面上能运行,但数据在语义上全乱了。与 ABI 不同的是,obs scale 不一致**不会触发任何编译或运行时错误**,它只会让策略行为诡异地"差一点",这才是最难调试的。

### 关节顺序映射——另一个常见的"隐形杀手"

不同系统的关节编号顺序可能不同:

| 系统 | 关节顺序(示例) |
|------|--------------|
| IsaacGym/IsaacLab | FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_..., RR_... |
| Unitree SDK | FR_0, FR_1, FR_2, FL_0, FL_1, FL_2, RR_..., RL_... |
| MuJoCo | 取决于 URDF 中的定义顺序 |

如果不做关节顺序映射,策略给前左腿的命令会发到前右腿——机器人的行为会像"左右脑对调"一样混乱。rl_sar 通过 `robot_config.yaml` 中的 `joint_reorder` 字段解决这个问题。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:训练和部署的关节顺序不一致**
> **错误做法**: 假设 IsaacGym 和 Unitree SDK 的关节编号顺序一样。
> **后果**: 策略给前左腿的命令发到了前右腿,机器人动作完全混乱,立刻摔倒。
> **根本原因**: 不同仿真器和 SDK 的关节编号约定不同,没有统一标准。
> **正确做法**: 明确记录训练时的关节顺序,在部署代码中维护映射表。

> ⚠️ **编程陷阱:忘记减去 `default_dof_pos`**
> **错误做法**: 把关节绝对位置直接作为观测传入网络。
> **后果**: 训练时观测是"关节位置相对于默认站立姿态的偏差"(范围约 $\pm 0.5$ rad),部署时传入绝对位置(范围约 $0 \sim 3$ rad)。数值范围完全不同,网络输出毫无意义。
> **正确做法**: `obs_joint_pos = (q_measured - q_default) * obs_scale`。

> ⚠️ **思维陷阱:认为 rl_sar 开箱即用**
> **新手想法**: "fork rl_sar,换上我的 .pt 文件,改一下机器人名字就完了。"
> **实际情况**: 每个训练配置的观测项数量、观测顺序、归一化参数、动作缩放、关节顺序都可能不同。rl_sar 的默认配置是为特定训练配置定制的——换了你的训练配置,必须逐项核对。
> **正确做法**: 精读 rl_sar 的 `ComputeObservation()`,与你的训练配置**逐项对比**。

### 练习

**练习 64.5.1**: fork rl_sar,对照你在 Ch63 训练时的 legged_gym/IsaacLab 配置文件,逐项修改 rl_sar 的配置。列出你修改的每一项及其原因。

**练习 64.5.2**: 写一个 Python 验证脚本:用训练环境生成一组 (obs, action) 数据对,然后在 C++ rl_sar 中用相同的 obs 推理,比较 action 输出。误差应 < 1e-5。

---

精读了部署逻辑后,下一节深入实时推理的工程细节——这些看似细小的点决定了部署是否真正可靠。

## 64.6 实时推理的工程细节 ⭐⭐⭐

### 动机:推理正确了 $\neq$ 部署成功

即使推理结果与 Python 完全一致,部署仍然可能失败——因为实时系统对**延迟确定性**有极高要求(Ch61 完整讲述)。以下是实时推理中必须解决的工程问题。

### 细节 1:内存预分配(零运行时分配)

实时循环中的**一切内存都必须在启动时预分配**:

```cpp
// ❌ 错误:每次循环都创建新对象
void update() {
    auto tensor = torch::zeros({1, 48});   // 堆分配!
    std::vector<float> result(12);          // 可能堆分配!
    auto output = module_.forward({tensor}); // 内部可能分配!
}

// ✅ 正确:启动时分配,运行时只填充
class RLController {
    torch::Tensor input_tensor_;    // 构造时分配
    Eigen::VectorXd action_;        // 构造时分配
    std::vector<float> obs_buf_;    // 构造时分配

    void init() {
        input_tensor_ = torch::zeros({1, 48}, torch::kFloat32);
        action_.resize(12);
        obs_buf_.resize(48, 0.0f);
    }

    void update() {
        // 只填充数据,不创建新对象
        std::memcpy(input_tensor_.data_ptr<float>(),
                     obs_buf_.data(), 48 * sizeof(float));
        // ... 推理 ...
    }
};
```

### 细节 2:避免 CPU-GPU 数据往返

如果整个控制循环在 CPU 上(ros2_control 的 `update()` 在 CPU 线程),就**不要用 GPU 推理小模型**:

```
CPU 控制循环:
  读传感器(CPU) → 构建 obs(CPU) → 拷贝到 GPU → GPU 推理 → 拷贝回 CPU → 发命令(CPU)
                                     ↑________50 us________↑  ↑_______50 us_______↑
                                     PCIe 往返开销比推理本身还大!
```

对小 MLP,CPU 推理 ~50 $\mu$s < GPU 推理 + 数据往返 ~200 $\mu$s。

### 细节 3:线程亲和(Thread Affinity)

把 RL 推理线程绑定到 `isolcpus` 隔离的 CPU 核心(Ch61 详述):

```cpp
// 设置当前线程亲和到 CPU 核心 3(已通过 grub isolcpus=3 隔离)
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(3, &cpuset);
pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

隔离核心不被其他进程使用,确保 RL 推理不会因缓存被刷出(cache thrashing)而延迟波动。

### 细节 4:消除首次推理的 JIT 编译延迟

LibTorch 第一次前向传播会触发 JIT 编译——将 TorchScript IR 编译为当前平台的优化机器码。延迟可达 **100-500 ms**。

```cpp
// 在 on_configure() 或 on_activate() 阶段做 warmup
// 不是在 update() 中!
void warmupPolicy() {
    torch::NoGradGuard no_grad;
    auto dummy = torch::zeros({1, obs_dim_}, torch::kFloat32);
    for (int i = 0; i < 20; ++i) {
        module_.forward({dummy});
    }
    // 此后 update() 中的推理延迟将稳定在 50-100 us
}
```

某些 cuDNN 算子即使 warmup 也需要第一次实际调用来编译。确保在**允许较大延迟的生命周期阶段**(如 `on_activate`)完成所有 warmup。

### 细节 5:延迟监控与统计

生产部署中,持续监控推理延迟:

```cpp
struct LatencyStats {
    double sum = 0, sum_sq = 0;
    double max_val = 0;
    int count = 0;
    std::vector<double> percentile_buf;

    void update(double value_us) {
        sum += value_us;
        sum_sq += value_us * value_us;
        max_val = std::max(max_val, value_us);
        count++;
        percentile_buf.push_back(value_us);
    }

    double mean() const { return sum / count; }
    double p99() {
        std::sort(percentile_buf.begin(), percentile_buf.end());
        return percentile_buf[static_cast<int>(0.99 * count)];
    }
};
```

### 完整延迟预算分析

一个 1 kHz 控制循环(1000 $\mu$s 周期)的延迟预算:

| 步骤 | 典型延迟 | 占比 |
|------|---------|------|
| 读取传感器 (state_interfaces) | 10-30 $\mu$s | 1-3% |
| 构建观测向量 | 5-10 $\mu$s | 0.5-1% |
| **RL 推理** | **50-100 $\mu$s** | **5-10%** |
| 动作后处理 + 安全检查 | 5-10 $\mu$s | 0.5-1% |
| 写入命令 (command_interfaces) | 10-30 $\mu$s | 1-3% |
| **总计** | **80-180 $\mu$s** | **8-18%** |
| **余量** | **820-920 $\mu$s** | **82-92%** |

余量充足(> 80%)——这是 RL 部署相比 MPC 部署的巨大优势。MPC 求解一次可能需要 10-50 ms(Ch55),需要双线程 Triple Buffer 架构;而 RL 推理在单线程内就能轻松完成。

### 延迟监控的生产级实现 ⭐⭐

在生产系统中，仅知道"平均延迟正常"是不够的——必须持续监控 P99 延迟和最大延迟，及时发现偶发的延迟尖刺。以下是一个轻量级的延迟统计类：

```cpp
class LatencyMonitor {
 public:
  void recordSample(double latency_us) {
    samples_[idx_ % kWindowSize] = latency_us;
    idx_++;
    if (idx_ % kReportInterval == 0) report();
  }

 private:
  void report() {
    std::vector<double> sorted(samples_.begin(), samples_.end());
    std::sort(sorted.begin(), sorted.end());
    double avg = std::accumulate(sorted.begin(), sorted.end(), 0.0) / kWindowSize;
    double p50 = sorted[kWindowSize / 2];
    double p99 = sorted[(int)(kWindowSize * 0.99)];
    double max_val = sorted.back();

    RCLCPP_INFO_THROTTLE(logger_, clock_, 5000,
        "Inference latency [us]: avg=%.1f p50=%.1f p99=%.1f max=%.1f",
        avg, p50, p99, max_val);

    // 安全警报：P99 超过 500us 时告警
    if (p99 > 500.0) {
      RCLCPP_WARN(logger_, "P99 latency %.1f us exceeds 500 us threshold!", p99);
    }
  }

  static constexpr int kWindowSize = 1000;
  static constexpr int kReportInterval = 1000;
  std::array<double, kWindowSize> samples_{};
  size_t idx_ = 0;
};
```

这个监控类应在 `update()` 函数中包裹推理调用，在开发和部署阶段始终开启。当 P99 延迟超过阈值时自动告警——这比事后分析 rosbag 高效得多。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:在实时循环中打印日志**
> **错误做法**: `std::cout << "action: " << action.transpose() << std::endl;`
> **后果**: `cout` + `endl` 涉及 I/O 系统调用和缓冲区 flush,可能阻塞数十毫秒(Ch61 详述)。
> **正确做法**: 使用限频警告(`RCLCPP_WARN_THROTTLE`)或无锁队列将日志发送到非实时线程。

> ⚠️ **编程陷阱:推理前不关闭 autograd**
> **错误做法**: 不使用 `torch::NoGradGuard`。
> **后果**: LibTorch 会构建计算图(为 backward 准备),即使你永远不会调 backward()。增加 ~20-30% 推理时间 + 额外内存分配。
> **正确做法**: 所有推理代码包在 `torch::NoGradGuard` 作用域内。

### 练习

**练习 64.6.1**: 在你的 RL 推理代码中添加 `LatencyStats` 监控。运行 10000 次推理,绘制延迟直方图。P50、P99、P999 分别是多少?是否有异常尖峰?分析原因。

**练习 64.6.2**: 对比有/无 `torch::NoGradGuard` 的推理延迟差异(百分比)。用 1000 次推理的均值对比。

---

推理正确且延迟达标后,还需要最后一层保护:如果 RL 模型输出了异常值怎么办?

## 64.7 安全降级:RL 部署的最后一道防线 ⭐⭐⭐

### 动机:RL 策略不是万无一失的

RL 策略是神经网络——它没有形式化的安全保证。在训练分布外的输入(out-of-distribution, OOD)上,输出完全不可预测。在真机上,异常输出的后果:

| 异常类型 | 后果 |
|---------|------|
| NaN 输出 | PD 控制器收到 NaN → 扭矩变 NaN → 电机可能释放最大扭矩 |
| 超出关节限位 | 电机堵转 → 齿轮组受极大应力 → 永久损坏 |
| 动作剧烈跳变 | 电机在 1 ms 内尝试转动大角度 → 极大电流冲击 |
| 持续异常输出 | 机器人缓慢偏离稳定姿态 → 几秒后摔倒 |

**安全降级不是可选项——它是生产级部署的必备组件。**

如果不做安全降级会怎样？2022 年某开源项目的真实案例：RL 策略在仿真中表现优秀,但部署到真机后,IMU 因电磁干扰偶发返回 NaN。NaN 传入网络后输出也是 NaN,PD 控制器收到 NaN 目标位置后将其解释为极大值,导致所有关节瞬间以最大扭矩运动——机器人像弹射一样跳起后摔碎。全过程不到 5 ms,人类来不及按急停键。一个 NaN 检查就能避免的事故,造成了数千元的硬件损失。

### 多层安全检查

```cpp
class SafetyGuard {
 public:
  struct CheckResult {
    bool safe;
    std::string reason;
  };

  CheckResult check(const Eigen::VectorXd& action,
                     const Eigen::VectorXd& current_pos,
                     const Eigen::VectorXd& last_action) {
    // Layer 1: NaN 检查 —— 最严重,立即降级
    if (action.hasNaN()) {
      return {false, "NaN detected"};
    }

    // Layer 2: 关节限位检查
    for (int i = 0; i < action.size(); ++i) {
      if (action[i] < lower_[i] || action[i] > upper_[i]) {
        return {false, "Joint " + std::to_string(i) + " out of limits"};
      }
    }

    // Layer 3: 动作变化率检查(防止跳变)
    double max_delta = (action - last_action).cwiseAbs().maxCoeff();
    if (max_delta > max_delta_per_step_) {
      return {false, "Action rate " + std::to_string(max_delta) + " too high"};
    }

    // Layer 4: 动作幅值检查
    if (action.norm() > max_action_norm_) {
      return {false, "Action norm too large"};
    }

    return {true, ""};
  }

 private:
  Eigen::VectorXd lower_, upper_;
  double max_delta_per_step_ = 0.5;   // rad/step
  double max_action_norm_ = 10.0;
};
```

### 降级策略的层次设计

```
正常运行
    │
    │ 检测到异常
    ▼
┌─────────────────────────┐
│ Level 1: Hold Last      │ ← 使用最近一次安全动作
│   触发: 偶尔一次异常     │    (如推理偶尔超时)
│   恢复: 下次正常即恢复   │
└──────────┬──────────────┘
           │ 连续 N 次异常 (N ~ 10-50)
           ▼
┌─────────────────────────┐
│ Level 2: Stand          │ ← 缓慢回到站立姿态
│   触发: 策略持续异常      │    q_cmd += 0.001 * (q_default - q_cmd)
│   恢复: 异常消失后缓慢   │    插值过渡, 不是突变
└──────────┬──────────────┘
           │ 持续异常 > 5 秒
           ▼
┌─────────────────────────┐
│ Level 3: Damping        │ ← 纯阻尼模式
│   触发: 策略完全失效      │    tau = -Kd * dq (让关节慢慢停)
│   恢复: 需要人工重启      │
└──────────┬──────────────┘
           │ 硬件级保护 (Ch62)
           ▼
┌─────────────────────────┐
│ Level 4: E-Stop         │ ← 电机驱动器看门狗
│   触发: 软件完全失控      │    50ms 无有效命令 → 自动切断
│   恢复: 手动复位          │
└─────────────────────────┘
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:安全检查后直接 clamp 而不是降级**
> **错误做法**: `action = action.cwiseMax(lower).cwiseMin(upper);`
> **后果**: clamp 后的动作可能不是 RL 策略"想要"的——比如策略想向前走但某个关节被 clamp 了,结果变成不协调的姿态。clamp 不保证结果行为的合理性。
> **正确做法**: 检测到超限就切换到**已知安全的降级策略**(如站立),而不是"修正"异常动作。

> ⚠️ **思维陷阱:认为安全降级只是"加个 NaN 检查"**
> **新手想法**: "加一行 `if (isnan) use_last_action` 就够了。"
> **实际情况**: NaN 只是最极端的异常。更常见的是"数值在合理范围内但行为不合理"——比如策略持续输出让机器人缓慢倾斜的动作,5 秒后才摔倒。这需要**多层检查**(限位 + 变化率 + 统计分布)和**分级降级**(不是非黑即白)。

> ⚠️ **概念误区:软件安全检查做好了就不需要硬件保护**
> **新手想法**: "代码里检查了所有异常情况,硬件保护是多余的。"
> **实际情况**: 如果控制程序本身崩溃(段错误、assert 失败),所有软件保护都失效。电机驱动器的看门狗(Ch62)是独立于控制软件的最后防线——如果驱动器在 50 ms 内没收到有效命令,自动进入阻尼模式。
> **正确做法**: 软件保护(本节) + 硬件看门狗(Ch62),两层独立。

### 练习

**练习 64.7.1**: 实现上述 `SafetyGuard` 类,集成到你的 ros2_control Controller 中。故意让 RL 模型输出 NaN(在 Python 中把某个权重设为 NaN 再导出),验证安全降级是否正确触发。

**练习 64.7.2**: 设计一个"动作分布偏移检测器":从训练 rollout 数据中统计策略输出动作的均值 $\mu$ 和标准差 $\sigma$(每个关节独立)。部署时如果动作连续 10 步超出 $\mu \pm 3\sigma$,触发 Level 2 降级。实现并测试。

---

## 64.8 从训练到部署的完整流水线 ⭐⭐

前面七节覆盖了部署的每个环节。这里把完整流水线串起来,并讨论 Jetson 平台部署和 sim-to-real 调优。

### 完整 8 步 Pipeline

```
┌──────────────────────────────────────────────┐
│ 1. 训练 (Ch63): IsaacLab + rsl_rl → ckpt.pth │
├──────────────────────────────────────────────┤
│ 2. 导出 (64.2): torch.jit.trace → policy.pt  │
│    验证: Python 输出一致性 < 1e-6            │
├──────────────────────────────────────────────┤
│ 3. 配置对齐 (64.5): 逐项核对训练/部署参数     │
│    obs_scales, joint_order, action_scale,    │
│    default_dof_pos, clip_obs, clip_actions   │
├──────────────────────────────────────────────┤
│ 4. C++ 仿真验证 (64.3): rl_sar + Gazebo     │
│    策略行为与 Python play.py 一致            │
├──────────────────────────────────────────────┤
│ 5. 实时性验证 (64.6): cyclictest + benchmark │
│    P99 推理延迟 < 500 us                     │
├──────────────────────────────────────────────┤
│ 6. 安全验证 (64.7): 注入 NaN/超限/跳变      │
│    安全降级正确触发                          │
├──────────────────────────────────────────────┤
│ 7. 真机部署: ros2_control + Unitree SDK      │
│    先低频(100 Hz) → 逐步提高到目标频率      │
├──────────────────────────────────────────────┤
│ 8. sim-to-real 调优(数天到数周)             │
│    观察真机行为 → 调参 → 必要时重训          │
└──────────────────────────────────────────────┘
```

### Jetson 平台部署要点

许多腿足机器人使用 NVIDIA Jetson Orin 作为机载计算平台:

| 维度 | x86 台式机 | Jetson Orin |
|------|-----------|-------------|
| **CPU 架构** | x86_64 | ARM64 (aarch64) |
| **LibTorch** | 官网下载 x86 版 | 需要 ARM64 版或交叉编译 |
| **CUDA** | 独立安装 | JetPack 预装,版本与 JetPack 绑定 |
| **功耗** | 不限 | 15-40W,需功耗管理 |
| **推荐推理引擎** | LibTorch(方便) | **ONNX + TensorRT**(针对 Jetson GPU 优化) |

在 Jetson 上,ONNX + TensorRT 组合通常最优:TensorRT 会将 ONNX 模型编译为 Jetson GPU 架构的高度优化 engine。

### Jetson 部署的完整工程清单 ⭐⭐

**Jetson 功耗模式选择**：Jetson Orin 支持多种功耗模式，不同模式下 CPU/GPU 频率不同，直接影响推理延迟：

| 功耗模式 | CPU 频率 | GPU 频率 | RL 推理延迟 | 适用场景 |
|---------|---------|---------|-----------|---------|
| MAXN (50W) | 2.2 GHz (全核) | 1.3 GHz | ~60 $\mu$s | 实验室测试 |
| 30W | 1.5 GHz (6核) | 930 MHz | ~90 $\mu$s | 正常部署 |
| 15W | 1.0 GHz (4核) | 612 MHz | ~150 $\mu$s | 长续航任务 |

```bash
# 设置 Jetson 功耗模式
sudo nvpmodel -m 0   # MAXN (50W)
sudo nvpmodel -m 1   # 30W
sudo jetson_clocks    # 锁定最高频率（避免动态降频导致延迟抖动）
```

**交叉编译 vs 本地编译**：

| 方式 | 优点 | 缺点 | 推荐场景 |
|------|------|------|---------|
| Jetson 本地编译 | 简单，无兼容性问题 | 编译慢（ARM 性能低） | 小项目、初次尝试 |
| Docker 交叉编译 | 快（x86 编译） | 需要配置 sysroot | 持续集成（CI） |
| NVIDIA SDK Manager | 官方工具链 | 仅支持 Ubuntu | 正式产品 |

**TensorRT engine 缓存策略**：TensorRT 首次加载 ONNX 时会做模型优化和 kernel 选择，耗时 30-60 秒。生产部署中必须缓存优化后的 engine：

```cpp
// 检查 TensorRT engine 缓存是否有效
bool engineCacheValid(const std::string& cache_path,
                      const std::string& onnx_path) {
    // 1. 缓存文件是否存在
    if (!std::filesystem::exists(cache_path)) return false;
    // 2. ONNX 文件是否比缓存新（模型更新后需重新编译）
    auto onnx_time = std::filesystem::last_write_time(onnx_path);
    auto cache_time = std::filesystem::last_write_time(cache_path);
    return cache_time > onnx_time;
}
```

> **⚠️ 陷阱：Jetson 上 GPU 内存与系统内存共享**
>
> 与台式机（独立显存）不同，Jetson 的 GPU 和 CPU 共享 8-16 GB 内存。如果同时运行 elevation_mapping_cupy（GPU 高程图）+ LibTorch（RL 推理）+ ROS2 节点（系统开销），很容易耗尽内存。正确做法：用 `tegrastats` 实时监控内存，为每个 GPU 用户设置内存上限。

### sim-to-real 调优

第 8 步是最耗时的环节。即使仿真验证完美,真机还需要反复调试:

| 症状 | 可能原因 | 解决方法 |
|------|---------|---------|
| 机器人抖动 | action_scale 太大,或 PD 的 Kd 太小 | 降低 action_scale 或增大 Kd |
| 走路太慢 | commands_scale 不正确 | 检查训练配置中的命令缩放 |
| 容易摔倒 | DR 范围不够大 | 回 Ch63 加大 DR(摩擦、延迟) |
| 脚步不协调 | 关节顺序映射错误 | 检查 joint_reorder 配置 |
| 电机过热 | default_dof_pos 不正确 | 偏差导致 PD 持续输出大扭矩 |

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱:Jetson 上 LibTorch 版本与训练端不匹配**
> **错误做法**: 训练用 PyTorch 2.x.0 导出 TorchScript,Jetson 的 LibTorch 是旧版 2.y.0（主版本不一致）。
> **后果**: TorchScript 有版本兼容性要求,可能加载失败或行为不一致。
> **正确做法**: 确保版本一致(至少主版本号一致),或使用 ONNX 格式(版本兼容性更好)。

> ⚠️ **思维陷阱:跳过仿真验证直接上真机**
> **新手想法**: "Python 里验证过了,直接上真机。"
> **实际情况**: Python→C++ 转换有太多细节可能出错。先在 Gazebo 的 C++ 仿真中验证策略行为与 Python 一致——这一步可以发现 80% 的部署 bug,成本远低于真机调试。

### 练习

**练习 64.8.1**: 按照 8 步 Pipeline,完成从 Ch63 训好的 Go2 策略到 Gazebo 仿真部署的全流程。记录每一步花费的时间和遇到的问题。

**练习 64.8.2**: 设计自动化 sim-to-sim 验证:在 Python(IsaacLab)和 C++(rl_sar + Gazebo)中用相同初始状态和命令序列跑同一策略 1000 步,比较关节轨迹的均方误差。

---

## 常见故障与排查

| 症状 | 可能原因 | 排查步骤 | 相关小节 |
|------|---------|---------|---------|
| ONNX/TorchScript 模型加载失败,报 `Invalid model` 或 `Unsupported op` | 导出时 opset 版本过低,或 PyTorch 与 LibTorch 版本不匹配 | 1. 使用能覆盖模型算子的最低 opset 版本导出,用 `onnx.checker.check_model()` 验证 2. 确认 LibTorch/ORT 版本与训练端 PyTorch 主版本一致 | 64.2, 64.4 |
| 推理延迟远超预期（>1 ms）,或首次推理特别慢 | 未做 warmup 导致 JIT 首次编译延迟,或未关闭 autograd,或在实时循环中创建新 tensor | 1. 检查是否在 `on_configure()` 阶段做了 20 次 warmup 2. 确认使用了 `torch::NoGradGuard` 3. 用 `LatencyStats` 统计 P99 延迟,区分首次 vs 稳态 | 64.6 |
| 部署后策略行为与 Python 不一致（机器人抖动、不走路、走偏） | 观测归一化参数不一致（obs_scales/default_dof_pos/关节顺序不匹配） | 1. 逐项对比训练配置中的 `obs_scales` 和部署代码 2. 打印部署端和训练端的 obs 向量,逐维对比 3. 检查关节顺序映射表 | 64.5 |
| RL 推理输出 NaN 或极大值,触发安全降级 | 模型文件损坏、输入包含 NaN（IMU 故障）、或导出时激活函数不匹配（ELU vs ReLU） | 1. 在 Python 中重新加载 .pt/.onnx 验证 2. 在推理前打印输入 obs 检查 NaN 3. 确认网络结构中激活函数与训练一致 | 64.2, 64.7 |
| Action clipping 后机器人姿态不协调（能站但走路别扭） | action_scale 或 clip_actions 参数与训练不一致,导致动作范围被错误截断 | 1. 对比训练配置中的 `clip_actions` 和部署代码 2. 在 Python play.py 中打印 action 的实际范围,确认部署端 clip 范围覆盖它 3. 暂时放大 clip 范围观察行为变化 | 64.5, 64.7 |

---

## 本章小结

| 知识点 | 核心内容 | 难度 | 关键技术 |
|--------|---------|------|---------|
| 64.1 部署基本问题 | Python vs C++、GIL/GC、两大方案 | ⭐ | LibTorch / ONNX Runtime |
| 64.2 TorchScript | trace vs script、IR 优化 pass、导出脚本 | ⭐⭐ | `torch.jit.trace` + `freeze` |
| 64.3 LibTorch | C++ 加载推理、Eigen 转换、预分配 | ⭐⭐ | `torch::jit::load` + `memcpy` |
| 64.4 ONNX Runtime | 导出、protobuf 结构、C++ 推理 | ⭐⭐ | `torch.onnx.export` + `Ort::Session` |
| 64.5 rl_sar 精读 | 观测归一化、关节映射、配置一致性 | ⭐⭐ | 配置对齐是 sim-to-real 关键 |
| 64.6 实时工程 | 预分配、warmup、线程亲和、延迟监控 | ⭐⭐⭐ | P99 < 500 $\mu$s |
| 64.7 安全降级 | NaN/限位/跳变/统计、4 级降级 | ⭐⭐⭐ | Hold → Stand → Damp → E-Stop |
| 64.8 完整流水线 | 8 步 pipeline、Jetson、sim-to-real 调优 | ⭐⭐ | 调优是最耗时环节 |

---

## 累积项目:本章新增模块

**四足站立控制器进度更新**:

```
Ch47: 加载 URDF + Pinocchio 建模           [完成]
Ch49: 正/逆运动学 + 接触 Jacobian           [完成]
Ch50: QP 求解器集成                         [完成]
Ch52: 摩擦锥约束                            [完成]
Ch53: WBC 实现 (加权 QP)                    [完成]
Ch55: OCS2 MPC 集成                         [完成]
Ch61: 实时部署框架                          [完成]
Ch62: 硬件接口 + Unitree SDK                [完成]
Ch63: RL 策略训练                           [完成]
Ch64: RL 策略 C++ 部署                      <-- 本章新增
  |-- TorchScript 导出脚本 (export_policy.py)
  |-- LibTorch C++ 推理类 (RLPolicy)
  |-- ONNX Runtime C++ 推理类 (ONNXPolicy)
  |-- ros2_control Controller 集成 (RLController)
  |-- SafetyGuard 安全降级模块
  |-- 推理延迟 benchmark 工具 (LatencyStats)
  |-- rl_sar 配置对齐检查
```

**本章新增的具体任务**:
1. 将 Ch63 训好的 Go2 策略导出为 TorchScript `.pt` 和 ONNX `.onnx`
2. 编写 C++ 推理类,验证输出与 Python 一致(误差 < 1e-6)
3. 集成到 ros2_control Controller 的 `update()` 函数
4. 实现 `SafetyGuard` 安全降级模块(4 层检查 + 4 级降级)
5. 在 Gazebo 中验证完整的 train→export→deploy pipeline
6. 做推理延迟 benchmark,确认 P99 < 500 $\mu$s

---

## 延伸阅读

### 必读 ⭐⭐

| 资料 | 类型 | 说明 |
|------|------|------|
| LibTorch 官方教程 (`pytorch.org/tutorials/advanced/cpp_frontend.html`) | 文档 | C++ 前端入门,含模型加载和推理 |
| ONNX Runtime 官方文档 (`onnxruntime.ai/docs/`) | 文档 | C++ API 参考,版本 1.25.x |
| rl_sar (`github.com/fan-ziqi/rl_sar`) | 代码 | 腿足 RL C++ 部署的最佳开源参考 |
| robot_lab (`github.com/fan-ziqi/robot_lab`) | 代码 | 配套 IsaacLab 训练端 |

### 核心论文 ⭐⭐⭐

| 资料 | 类型 | 说明 |
|------|------|------|
| Hwangbo J. et al. (2019) "Learning agile and dynamic motor skills for legged robots", Science Robotics 4(26), eaau5872 | 论文 | 首次 RL 真机部署(ANYmal),含 actuator network |
| Rudin N. et al. (2022) "Learning to Walk in Minutes", CoRL | 论文 | legged_gym 框架,含部署流程 |

### 深入技术 ⭐⭐⭐⭐

| 资料 | 类型 | 说明 |
|------|------|------|
| PyTorch JIT Overview (`github.com/pytorch/pytorch/blob/main/torch/csrc/jit/OVERVIEW.md`) | 文档 | TorchScript IR 和优化 pass 的官方详解 |
| LibTorch Stable ABI (PyTorch 2.10+) | 文档 | ABI 稳定性保证,简化版本管理 |
| ONNX Runtime 1.25 Release Notes | 文档 | CUDA Plugin EP,第三方加速后端插件化 |

---

## 与其他章节衔接

**向前承接**:
- Ch61(实时 C++)→ 64.6 RL 推理的实时工程要求
- Ch62(硬件栈)→ 64.5 关节顺序映射、看门狗保护
- Ch63(RL 训练)→ 64.2 模型导出、64.5 归一化一致性

**向后指向**:
- Ch64 → Ch65 RL+MPC 混合(RL 部分的部署正是本章内容)
- Ch64 → Ch69 Mini-Legged 综合实战(真机部署完整验证)
- Ch64 安全降级 → Ch70 研究方向(安全 RL、运行时验证)

---

分别掌握了 MPC(Ch55)和 RL(Ch63-64)之后,自然的问题是:能否融合两者的优势?Ch65 正是腿足控制最活跃的研究前沿——OCS2 MPC-Net、Cafe-MPC VWBC、RAMBO、残差 RL 四条 RL+MPC 混合路线都要讲透。Ch66 则衔接到感知的工程侧——讲清楚 Grid Map、Elevation Mapping 等数据结构和工程实现,为 Ch67 的 Perceptive MPC 算法做好数据层准备。
