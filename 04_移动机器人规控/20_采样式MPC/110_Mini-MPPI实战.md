# 第 10 章：Mini-MPPI 综合实战

**性质**：✅ **全方向共享**

前 9 章回答了"是什么""为什么""怎么改进"——从路径积分的理论基础（Ch1），到工程可部署的完整算法与 GPU 并行化（Ch2），再到六大变体（Ch3）、CEM 统一视角（Ch5）、学习世界模型（Ch7）、腿足全身控制（Ch8）等各个维度。但到目前为止，你读的都是**别人写好的代码**和**别人跑出的数字**。本章回答最后一个问题：**你自己能不能从零写出一个完整的、能跑的 CUDA MPPI 控制器？**

这中间隔着三类东西。其一是**工程架构**——前面各章的代码片段散落在不同节里，每段各管一件事（动力学、kernel、权重计算），但一个真正能编译、能运行的项目需要把它们组装成一个有结构的 CMake 工程，该怎么组织目录、怎么分模板？其二是**跨环境集成**——Ch2 的 rollout kernel 用的是自写的简单动力学，真实项目常常要接上 MuJoCo 这样的物理引擎，而 MuJoCo 有自己的 C API、线程模型和数据结构，把 MPPI 和它缝在一起绝非简单替换。其三是**性能验证**——Ch2 引用的是 MPPI-Generic 论文的基准数字，但你自己的实现到底快不快、K 和 T 怎么选、GPU 比 CPU 快多少，必须自己量、自己画曲线、自己分析瓶颈。

学完本章，你将拥有一份自己从零搭建的 Mini-MPPI 项目——它不是某个库的 wrapper，而是你亲手写的每一行 kernel、每一个模板参数、每一条 CMakeLists 指令。这个过程会把前 9 章的知识从"读过"变成"用过"，从"理解"变成"会做"。

---

## 📋 前置自测

> 答不出 **$\geq 2$ 题**，先回对应章节补课，再读本章。本章是综合实战，默认你已掌握前 9 章的核心内容。

1. **（Ch2 §2.1）** 写出 MPPI 权重公式 $w_k$，并解释 $\rho = \min_k S_k$ 减法的作用。如果不减 $\rho$，在代价 $S_k \approx 1200$ 时会发生什么？
2. **（Ch2 §2.2）** CUDA 的 rollout kernel 中，"每 thread 一条轨迹"和"每 warp/block 一条轨迹"两种线程映射各有什么优缺点？MPPI-Generic 倾向哪种？为什么？
3. **（Ch2 §2.1）** warm-start shift 的操作是什么？它本质上改善了采样的什么属性？如果每周期从零冷启动，会有什么后果？
4. **（Ch3）** MPPI 的六大变体中，有色噪声采样（colored noise）解决的是什么问题？它和 Savitzky-Golay 事后平滑有什么本质区别？
5. **（Ch5 CEM）** CEM（Cross-Entropy Method）和 MPPI 都是采样式 MPC，它们在权重计算上的核心区别是什么？CEM 用精英样本拟合分布，MPPI 用什么？

---

## 本章目标

学完本章后，你应该能够：

1. **从零搭建**一个模板化的 CUDA MPPI 项目——包括目录结构、CMakeLists 配置、动力学/代价/控制器的模板分离，并说清每个设计决策的理由；
2. **手写**至少两种动力学模型（CartPole swing-up + Quadrotor/Go2 hover），理解 `__host__ __device__` 双标注的工程原因，并规避 Eigen 在 CUDA 中的常见陷阱；
3. **实现并调优**一个完整的 CUDA rollout kernel——包括线程映射、shared memory 归约、register pressure 管理，并能用 nsys/ncu 定位瓶颈；
4. **编写**完整的 MPPI 主循环——包括 cuRAND 噪声生成、GPU 或 CPU 权重归约、warm-start shift、Savitzky-Golay 平滑集成，并理解每步的算法动机与工程权衡；
5. **集成** MuJoCo 物理引擎——理解 mjData 的线程安全模型、mjModel 共享模式、batch rollout 的数据组织，能在 MPPI rollout 中调用 MuJoCo C API；
6. **设计并执行**系统化的性能基准测试——K/T 扫参、GPU vs CPU 对比、nsys timeline 分析、ncu kernel profiling——并能从数据中得出可操作的调优结论。

---

## 本章知识导航

本章七节是一条**从架构到部署再到验证**的递进链：先把项目骨架搭好（§10.1），再填入动力学模型（§10.2）和 GPU kernel（§10.3），然后写好主循环把它们串起来（§10.4），接上 MuJoCo 引擎（§10.5），最后用基准测试验证性能（§10.6）并学会调试故障（§10.7）。

| 节 | 知识点 | 难度 | 它解决什么 | 依赖 |
|----|--------|------|-----------|------|
| §10.1 | 项目架构、CMake 配置、模板化设计 | ⭐⭐ | 把散落的代码片段组织成可编译的工程 | Ch2 §2.3 模板设计 |
| §10.2 | 动力学模型封装、`__host__ __device__`、Eigen-CUDA 互操作 | ⭐⭐ | 让同一份动力学代码在 CPU 测试和 GPU kernel 里都能跑 | §10.1、Ch2 §2.2 |
| §10.3 | CUDA rollout kernel、线程映射、shared memory、register pressure | ⭐⭐⭐ | 把 $K$ 条 rollout 并行到 GPU 上 | §10.2、Ch2 §2.2 |
| §10.4 | MPPI 主循环、权重计算、warm-start、SGF 平滑 | ⭐⭐ | 把 kernel 和算法逻辑串成完整控制器 | §10.3、Ch2 §2.1 |
| §10.5 | MuJoCo C API、mjData 线程安全、batch rollout | ⭐⭐ | 用工业级物理引擎替换手写动力学 | §10.4 |
| §10.6 | K/T 扫参、GPU vs CPU 对比、nsys/ncu profiling | ⭐⭐⭐ | 量化性能、定位瓶颈、指导调优 | §10.3–§10.5 |
| §10.7 | CUDA 调试、数值发散诊断、可视化调试 | ⭐⭐ | 排查实战中的常见故障 | 全章 |

> **关于本章的代码**：本章给出的代码是**完整可编译的工程级 C++/CUDA 代码**，与前面章节的教学示意片段不同。每段代码前会解释"为什么这样写"，代码后会给出"如果不这样做会怎样"。你可以直接按照本章的架构搭建自己的 Mini-MPPI 项目。

### 前置知识桥接

**回顾 Ch2 §2.1：MPPI 完整算法。** Ch2 把 Ch1 的 20 行核心补全成了可部署的算法：$\rho$ 减法保数值稳定、warm-start shift 复用上周期计算、Savitzky-Golay 平滑驯服控制抖动、$\lambda$/$\Sigma$ 调探索与利用、ESS 当探针监控采样质量。本章的 §10.4 要把这些细节从伪代码落实为真正能跑的 C++ 代码。

**回顾 Ch2 §2.2：CUDA 并行化策略。** Ch2 讲透了两种线程映射（每 thread 一条 vs 每 warp/block 一条）和三级内存层级（register / shared / global），但只给了教学示意的 kernel 骨架。本章的 §10.3 要把它写成能编译、能 profile 的真实 kernel。

**回顾 Ch2 §2.3：MPPI-Generic 的模板化设计。** Ch2 分析了 Georgia Tech 的 MPPI-Generic 库如何用 C++ 模板把 Dynamics / Cost / Sampler / Controller 四维正交解耦。本章的 §10.1 要借鉴这个思路，设计自己的模板化项目架构——不是照抄 MPPI-Generic，而是理解其设计哲学后做一个精简版。

**回顾 Ch3：六大变体。** Ch3 介绍了有色噪声、协方差自适应、基于梯度的采样引导等变体，它们本质上都在"让提议分布更聪明、提高样本效率"。本章的实现先用标准白噪声采样跑通全流程，但架构设计要**预留**变体扩展的接口——这就是模板化的价值。

**回顾 Ch5：CEM 统一视角。** Ch5 把 CEM 和 MPPI 统一在"采样-评估-更新分布"的框架下，区别在于更新方式（CEM 用精英样本拟合、MPPI 用指数加权）。本章的代价函数模板设计要能同时支持这两种更新策略，为后续扩展留空间。

### 如果跳过本章会怎样

- **场景一**：你读完了 Ch2 的 CUDA kernel 教学示意，觉得"我理解了"，但从没自己写过一个能编译的 kernel。当你要给自己的机器人写 MPPI 控制器时，发现连 CMakeLists 怎么配 CUDA 都不知道，Eigen 的 `Matrix` 在 `__device__` 函数里报一堆编译错误，`cudaMalloc` 的指针类型怎么传给模板函数也搞不定——**读懂和写出之间隔着一条深沟**，本章就是在过这条沟。
- **场景二**：你照着 Ch2 的片段拼了一个 MPPI，仿真里看着还行，但一接 MuJoCo 发现 mjData 在多线程下崩溃——因为你不知道 mjData 不是线程安全的，需要每线程一份拷贝（§10.5）。或者你想量化性能，却不知道 nsys 和 ncu 怎么用，只好靠 `clock()` 计时，结果因为忘了 `cudaDeviceSynchronize()` 而测出了虚假的"超快"时间。

### 预计学习时间

| 模式 | 时间 | 适合 |
|------|------|------|
| 精读 + 动手实现（从零搭建完整项目、跑通两个任务、做完基准测试） | 30–40 小时（2 周） | 要自己写 MPPI 控制器的工程师 |
| 速读 + 参考实现（读懂架构和设计决策、在提供的代码基础上修改） | 10–15 小时 | 需要快速上手改别人代码的研究者 |
| 速查（查特定模块的实现、某个 CUDA 错误的排查） | 1–2 小时 | 已实现过、回来查细节 |

---

## 环境配置指南

本章是**工程实践**章，不配好环境一行代码都跑不起来。与 Python 训练管线（pip install 即可）不同，CUDA + C++ + Eigen + MuJoCo 这套原生工具链的版本耦合更紧、踩坑更隐蔽——一个 CUDA 与驱动的小版本错配就能让你卡一下午。本节把"装什么、装哪个版本、怎么验证装对了"一次说清。

### 系统要求

| 项目 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 20.04 / 22.04（推荐）或 Windows 10/11 + WSL2 | 本章命令以 Ubuntu 为准；MuJoCo 的 GPU 渲染在原生 Linux 上最省心 |
| GPU | NVIDIA，计算能力 $\geq 6.0$（Pascal 及以后） | $\geq 7.0$（Volta/Turing 及以后）才有完整的独立线程调度（independent thread scheduling），warp 内同步语义更安全 |
| 显存 | $\geq 4$ GB | $K=8192, T=64, m=4$ 的噪声数组约 8 MB，本身不大；显存主要被 MuJoCo 模型与渲染占用 |
| NVIDIA 驱动 | $\geq 525$（对应 CUDA 12.x） | 驱动版本决定可用的**最高** CUDA 运行时版本，反之不约束——这是新手最常错配的一环（见下文本质洞察） |
| CPU | $\geq 4$ 核（MuJoCo OpenMP batch rollout 用） | §10.5 的 CPU batch rollout 吃多核；核越多 MuJoCo rollout 越快 |

> **本质洞察：驱动版本与 CUDA Toolkit 版本是"地基与楼层"的关系，不是"必须相等"的关系。** 很多新手以为"CUDA 12.4 的代码必须装 CUDA 12.4 的驱动"，于是疯狂折腾驱动。真相是：NVIDIA 驱动向后兼容（backward compatible）——一个支持 CUDA 12.6 的新驱动，可以跑用 CUDA 11.8、12.0、12.4 编译的任何程序。所以正确策略是**装尽可能新的驱动，然后 Toolkit 版本只要 $\leq$ 驱动支持的上限即可**。`nvidia-smi` 右上角显示的 "CUDA Version: 12.6" 是**驱动支持的上限**，不是你装的 Toolkit 版本——这个误解是 90% 的"CUDA 版本对不上"困惑的根源。

### 版本兼容表

| 组件 | 推荐版本 | 最低版本 | 最高测试版本 | 备注 |
|------|---------|---------|------------|------|
| NVIDIA 驱动 | 535 | 525 | 555 | 决定 CUDA 运行时上限；装最新稳定版即可 |
| CUDA Toolkit | 12.4 | 11.8 | 12.6 | nvcc 编译器 + cuRAND 随之安装；$\leq$ 驱动上限 |
| CMake | 3.27 | 3.18 | 3.30 | 3.18 才有成熟的 `enable_language(CUDA)` 一等支持 |
| GCC / G++ | 11 | 9 | 12 | 必须与 CUDA 版本兼容；CUDA 12.4 不支持 GCC 13+ |
| Eigen | 3.4.0 | 3.4.0 | 3.4.0 | 3.4 才正式标注 CUDA 设备代码支持；3.3 有坑 |
| MuJoCo | 3.1.x | 2.3.0 | 3.2.x | 3.0+ 起官方开源、C API 稳定（§10.5） |
| Python（可视化） | 3.10 | 3.8 | 3.12 | 仅 `python/visualize.py` 用，与 C++ 主程序解耦 |
| matplotlib / numpy | 3.8 / 1.26 | 3.5 / 1.21 | 3.9 / 2.0 | 可视化依赖；numpy 2.0 已测试通过 |

> **版本锁定建议**：CUDA 和 GCC 在**大版本**粒度锁定（如固定 CUDA 12.x + GCC 11），因为它们的 ABI（应用二进制接口）跨大版本不保证兼容；Eigen 在**精确版本**锁定（3.4.0），因为它是 header-only，换版本就是换源码，3.3→3.4 在 CUDA 设备代码上有 breaking change（3.4 之前 `Eigen::Matrix` 在 `__device__` 函数里需要额外补丁）；MuJoCo 在**小版本**粒度锁定（3.1.x），3.0 是 C API 的分水岭（2.x 闭源、API 不同）。

### GCC 与 CUDA 的版本耦合陷阱

这是原生 CUDA 工具链**最容易卡住新手**的一环，值得单独展开。nvcc 编译 `.cu` 文件时，会把主机端（host）代码交给系统的 host 编译器（Linux 上是 g++）。但 nvcc 对 host 编译器的版本有**严格上限**——每个 CUDA 版本只支持到某个 GCC 版本，超过就直接报错：

```
#error -- unsupported GNU version! gcc versions later than 12 are not supported!
```

| CUDA Toolkit | 支持的最高 GCC | 常见错配场景 |
|--------------|---------------|-------------|
| 11.8 | GCC 11 | Ubuntu 22.04 默认 GCC 11，恰好兼容 |
| 12.0–12.3 | GCC 12 | 系统升级到 GCC 13 后 nvcc 直接拒绝编译 |
| 12.4–12.6 | GCC 13 | Ubuntu 24.04 默认 GCC 13，需 CUDA $\geq$ 12.4 |

**反事实：如果忽略这个耦合会怎样。** 你在 Ubuntu 24.04（默认 GCC 13）上装了 CUDA 12.0（最高支持 GCC 12），`cmake` 配置阶段不会报错（CMake 只检查 nvcc 存在），但一 `make` 就撞上 `unsupported GNU version`。新手往往以为是 CUDA 没装好，重装三遍 CUDA 仍然失败——根因是 GCC 太新。**正确做法**有两条路：(1) 升级 CUDA 到支持当前 GCC 的版本；(2) 装一个旧版 GCC 并显式告诉 CMake 用它：

```bash
# 路径 2：安装并指定旧版 GCC（以 GCC 11 为例）
sudo apt install gcc-11 g++-11
# CMake 配置时显式指定 CUDA 的 host 编译器
cmake -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-11 ..
```

### 安装步骤（Ubuntu 22.04）

```bash
# ---- 1. 安装 NVIDIA 驱动（如果尚未安装）----
# 用 ubuntu-drivers 自动选择推荐版本，避免手动下错
sudo ubuntu-drivers autoinstall
sudo reboot
# 重启后验证：应看到 GPU 型号和 "CUDA Version: 12.x"（驱动上限）
nvidia-smi

# ---- 2. 安装 CUDA Toolkit 12.4 ----
# 从 NVIDIA 官网下载对应 .deb（这里用 network 安装方式）
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-4
# 把 nvcc 加入 PATH（写进 ~/.bashrc 永久生效）
export PATH=/usr/local/cuda-12.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH
# 验证：应输出 "Cuda compilation tools, release 12.4"
nvcc --version

# ---- 3. 安装 CMake、Eigen、构建工具 ----
sudo apt install cmake build-essential libeigen3-dev
# 验证 Eigen 版本（应 >= 3.4）
pkg-config --modversion eigen3

# ---- 4.（可选，§10.5 需要）安装 MuJoCo ----
# MuJoCo 3.x 提供预编译动态库，解压即用
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.6/mujoco-3.1.6-linux-x86_64.tar.gz
tar -xzf mujoco-3.1.6-linux-x86_64.tar.gz -C ~/.local/

# ---- 5.（可选）可视化环境 ----
pip install numpy matplotlib
```

**为什么用 network 安装方式而非 runfile。** CUDA 有两种安装包：`.deb`（network/local）和 `.run`（runfile）。runfile 会试图同时装驱动和 Toolkit——如果你已经用 `ubuntu-drivers` 装好了驱动，runfile 可能覆盖它导致冲突。`.deb` network 方式只装 Toolkit、不碰驱动，更安全。这正是上面"驱动与 Toolkit 解耦"原则的实践。

### Quick Start（5 分钟跑通最小示例）

下面给出一个**最小可编译**的 Mini-MPPI 骨架——不含完整的 CartPole 控制逻辑，只验证"CUDA + Eigen + cuRAND 这套工具链在你的机器上能编译、能启动 kernel、能拿回结果"。跑通它，说明环境配好了；跑不通，对照上文版本表排查。

```bash
# ---- 1. 建立最小项目 ----
mkdir -p ~/mini-mppi-hello/{src,build} && cd ~/mini-mppi-hello
```

最小 `src/hello_mppi.cu`（一个把 Eigen 状态搬进 kernel、用 cuRAND 生成噪声、跑一步 Euler 积分并回传的"冒烟测试"）：

```cpp
// src/hello_mppi.cu —— 工具链冒烟测试
#include <cstdio>
#include <Eigen/Dense>
#include <curand.h>
#include <cuda_runtime.h>

// 最小 rollout kernel：每线程对一个标量状态做一步加噪积分
__global__ void smoke_kernel(const float* noise, float* out, int K) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= K) return;                       // 越界保护——CUDA 铁律
  Eigen::Matrix<float, 2, 1> x;             // 固定大小 Eigen，栈/寄存器分配
  x << 1.0f, 0.0f;                          // 初始状态
  float u = noise[k];                       // 取本线程的噪声当控制
  x(1) = x(1) + u * 0.01f;                  // 一步 Euler（sinf 等同理用 f 后缀）
  out[k] = x(1);                            // 回写
}

int main() {
  const int K = 1024;
  float *noise_d, *out_d;
  cudaMalloc(&noise_d, K * sizeof(float));
  cudaMalloc(&out_d,   K * sizeof(float));

  // cuRAND 在 GPU 上直接生成 K 个标准正态噪声
  curandGenerator_t gen;
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen, 42);
  curandGenerateNormal(gen, noise_d, K, 0.0f, 1.0f);

  smoke_kernel<<<(K + 255) / 256, 256>>>(noise_d, out_d, K);
  cudaError_t err = cudaDeviceSynchronize();  // 同步并捕获 kernel 执行期错误
  if (err != cudaSuccess) {
    printf("CUDA error: %s\n", cudaGetErrorString(err));
    return 1;
  }

  float out_h[8];
  cudaMemcpy(out_h, out_d, 8 * sizeof(float), cudaMemcpyDeviceToHost);
  printf("OK. first 3 outputs: %.4f %.4f %.4f\n", out_h[0], out_h[1], out_h[2]);

  curandDestroyGenerator(gen);
  cudaFree(noise_d); cudaFree(out_d);
  return 0;
}
```

```bash
# ---- 2. 一行编译（不写 CMake，先验证工具链）----
# -arch=sm_XX 改成你 GPU 的计算能力（nvidia-smi 查型号后查表）
nvcc -arch=sm_75 --expt-relaxed-constexpr \
     -I/usr/include/eigen3 \
     src/hello_mppi.cu -lcurand -o build/hello_mppi

# ---- 3. 运行 ----
./build/hello_mppi
# 预期输出（具体数值因随机种子而定，但应是三个有限的小数）：
# OK. first 3 outputs: 0.0148 -0.0061 0.0233
```

**成功标志**：打印出 `OK.` 加三个有限小数。这一行输出同时验证了五件事：nvcc 能编译、Eigen 头文件能被 nvcc 解析（`--expt-relaxed-constexpr` 生效）、kernel 能启动（`-arch` 匹配硬件）、cuRAND 能生成噪声（链接到 `-lcurand`）、GPU→CPU 回传正常。**失败排查**：编译期报 `constexpr` 错 → 漏了 `--expt-relaxed-constexpr`；报 `unsupported GNU version` → GCC 太新（见上文）；运行期报 `no kernel image is available` → `-arch` 与 GPU 不匹配（章末故障排查手册场景 6）；报 `cannot find -lcurand` → CUDA 库路径没进 `LD_LIBRARY_PATH`。

跑通这个 30 行的冒烟测试后，你就可以放心进入 §10.1，把它扩展成有目录结构、有 CMake、有模板化动力学的完整 Mini-MPPI 项目。

---

## §10.1 项目架构与设计决策 ⭐⭐

### 动机：为什么需要想清楚目录结构

把 Ch2 的所有代码片段——动力学模型、CUDA kernel、权重计算、控制循环、可视化——堆进一个 `main.cu` 里，能不能跑？能。但这样做会怎样？

第一，**改一个动力学模型，整个 kernel 要重新编译**。CUDA 编译本就比纯 C++ 慢得多（nvcc 的预处理、PTX 生成、设备代码链接），如果所有东西都在一个文件里，改一行 CartPole 的参数就要等 nvcc 重新编译整个 kernel——在 RTX 3080 上，一个中等复杂度的 `.cu` 文件编译需要 10-30 秒，而纯 C++ 的 `.cpp` 通常 1-3 秒。开发迭代速度直接降 10 倍。

第二，**无法复用**。你写了 CartPole，下一个任务换成四旋翼，发现 kernel 里硬编码了 `STATE_DIM=4`、`CONTROL_DIM=1`，改成 12 和 4 后又要改一堆相关的数组大小、线程映射——本质上是在重写，不是在复用。

第三，**测试困难**。动力学模型的正确性应该在 CPU 上单独测试（给一个已知的状态和控制，验证输出），但如果它和 kernel 糅在一起，只能通过跑整个控制循环来间接验证——这是用大炮打蚊子，一旦输出不对，不知道是动力学错、kernel 错、还是权重计算错。

> **类比：为什么餐厅要把厨房分成切配、炒锅、凉菜？** 不分也能做饭，但一个人在同一块砧板上切肉切菜切水果，效率低、易交叉污染、出问题找不到源头。分开后各司其职、互不干扰，哪道菜出问题一眼就知道是哪个档口。软件架构的模块化也是同理——每个模块有清晰的接口和职责，独立开发、独立测试、独立替换。

### 如果不模板化会怎样

这是一个值得亲身体验的反事实。假设你不用模板，而是为 CartPole 写一个 `rollout_kernel_cartpole`，为四旋翼写一个 `rollout_kernel_quadrotor`——两个 kernel 的结构几乎一模一样（生成噪声、前向积分 T 步、累加代价），唯一的区别是动力学和代价函数。你会发现自己在做**复制-粘贴式编程**：两份 kernel 有 80% 相同的代码，改一个 bug 要同时改两处，忘了一处就埋下不一致的隐患。

模板的价值正在于此：**把变化的部分（动力学、代价）抽成模板参数，不变的部分（kernel 结构、权重计算、warm-start）写一次用到底**。这正是 Ch2 §2.3 分析的 MPPI-Generic 的设计哲学——Dynamics、Cost、Sampler、Controller 四个维度用模板正交解耦，换一个机器人只需换一个模板参数，kernel 结构不动。

### 实现：项目目录结构

**为什么这样组织。** 目录结构的设计原则是**按职责分离、按编译单元归组**。头文件模板放 `include/`（不参与编译，只在被 `#include` 时实例化），CUDA 编译单元放 `src/`（nvcc 编译），纯 CPU 代码也放 `src/`（g++ 编译），Python 可视化放 `python/`（完全独立，不参与 C++ 构建）。这样 nvcc 只需要编译 `.cu` 文件，纯 C++ 部分用更快的编译器处理。

```
mini-mppi/
├── include/
│   ├── dynamics/           # 动力学模型（头文件模板）
│   │   ├── cartpole.hpp    #   CartPole：STATE_DIM=4, CONTROL_DIM=1
│   │   └── quadrotor.hpp   #   Quadrotor：STATE_DIM=12, CONTROL_DIM=4
│   ├── cost/               # 代价函数（头文件模板）
│   │   ├── cartpole_cost.hpp
│   │   └── quadrotor_cost.hpp
│   ├── mppi/               # MPPI 核心（CUDA kernel + 控制器）
│   │   ├── rollout_kernel.cuh  # rollout kernel 模板（CUDA）
│   │   └── mppi_controller.hpp # MPPI 主循环（CPU 端调度）
│   └── utils/              # 工具
│       ├── cuda_utils.cuh  #   CUDA 错误检查宏、内存管理
│       └── eigen_cuda.hpp  #   Eigen-CUDA 互操作辅助
├── src/
│   ├── main.cpp            # 控制循环入口（纯 CPU，调用控制器）
│   └── mppi_kernel.cu      # CUDA kernel 编译单元（实例化模板）
├── python/
│   └── visualize.py        # 结果可视化（matplotlib）
├── CMakeLists.txt          # 构建配置
├── README.md
└── configs/
    ├── cartpole.yaml       # CartPole 超参数（K, T, λ, Σ...）
    └── quadrotor.yaml      # Quadrotor 超参数
```

**如果不这样做会怎样。** 把所有 `.hpp` 和 `.cu` 堆在同一层目录里，当文件超过 10 个时就会发现"这个 `utils.hpp` 到底是给动力学用的还是给 kernel 用的？"；当两个人同时开发 CartPole 和 Quadrotor 任务时，会在同一个目录里频繁冲突。按职责分目录后，开发 CartPole 的人只碰 `dynamics/cartpole.hpp` 和 `cost/cartpole_cost.hpp`，与 Quadrotor 互不干扰。

### 实现：CMakeLists 配置

**为什么需要专门讲 CMake。** CUDA + C++ + Eigen 的构建配置是新手最容易卡住的地方。CMake 对 CUDA 的支持在 3.18 之后才算成熟（`enable_language(CUDA)` + `set_target_properties` 设置 CUDA 标准），之前需要 `find_package(CUDA)` 走另一条路径，两种方式的语法完全不同，网上教程混杂，新手很容易抄错。

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.18)
project(mini_mppi LANGUAGES CXX CUDA)

# --- C++ 和 CUDA 标准 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

# --- Eigen（header-only，只需 include 路径）---
find_package(Eigen3 3.4 REQUIRED)

# --- CUDA 编译选项 ---
# -arch=sm_75 是 RTX 2080 的计算能力；改成你 GPU 的实际值
# --expt-relaxed-constexpr 允许 constexpr 函数在 device 代码中使用
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_75 --expt-relaxed-constexpr")

# --- 可执行目标 ---
add_executable(mini_mppi
    src/main.cpp          # 纯 C++，g++ 编译
    src/mppi_kernel.cu    # CUDA，nvcc 编译
)

target_include_directories(mini_mppi PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(mini_mppi PRIVATE
    Eigen3::Eigen
    curand              # cuRAND 用于设备端噪声生成
)

# --- 让 nvcc 看到 Eigen 头文件 ---
# Eigen 是 header-only，但 nvcc 的 include 路径有时需要显式指定
set_target_properties(mini_mppi PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON    # 允许跨文件的 device 函数调用
)
```

**代码后的解读。** 这份 CMakeLists 有几个关键决策值得展开。

其一，`LANGUAGES CXX CUDA` 同时声明两种语言——CMake 会自动用 `g++` 编译 `.cpp`，用 `nvcc` 编译 `.cu`，无需手动指定编译器。

其二，`-arch=sm_75` 指定了 GPU 的计算能力（Compute Capability）。这个值必须与你的 GPU 匹配：RTX 2080 是 `sm_75`，RTX 3080 是 `sm_86`，RTX 4090 是 `sm_89`。如果写错——比如你的 GPU 是 `sm_86` 却写了 `sm_75`——代码能编译但跑不出最优性能（新架构的特性用不上），严重时甚至编译出的 kernel 与硬件不兼容直接崩。

其三，`--expt-relaxed-constexpr` 是 Eigen 在 CUDA 中能用的前提条件之一。Eigen 的很多函数用了 `constexpr`，但标准 CUDA 对 `constexpr` 在设备代码中的使用有限制，这个 flag 放宽了限制。不加它，你会在编译 `Eigen::Matrix` 的设备函数时遇到一堆"calling a constexpr __host__ function from a __device__ function is not allowed"的错误。

**如果不这样做会怎样。** 最常见的错误是用旧式的 `find_package(CUDA)` + `cuda_add_executable` 语法——它在 CMake 3.18+ 中已被标记为过时（deprecated），与新版 CMake 的 CUDA 支持冲突，导致编译选项丢失、链接失败。另一个常见错误是忘了 `CUDA_SEPARABLE_COMPILATION ON`——当你在 `.cu` 文件里调用另一个 `.cuh` 文件中定义的 `__device__` 函数时，缺少分离编译会导致链接期找不到符号（`undefined reference to __device_stub__`）。

### ⚠️ 常见陷阱

**编程陷阱：CMake 的 `-arch` 参数与 GPU 不匹配。**
- **错误描述**：CMakeLists 里写了 `-arch=sm_75`，但实际 GPU 是 RTX 3080（`sm_86`）。
- **现象/后果**：编译通过，但运行时出现 `CUDA error: no kernel image is available for execution on the device`，kernel 直接不执行。或者编译出的代码能跑但用不上新架构的 tensor core / shared memory 特性，性能打折。
- **根本原因**：`-arch=sm_XX` 指定的是 PTX 编译的目标架构，GPU 硬件只能执行与自己架构兼容的二进制。低版本的 PTX 可以被 JIT 为高版本硬件执行（forward compatible），但反过来不行。
- **正确做法**：运行 `nvidia-smi` 查看 GPU 型号，查表确认计算能力，填入正确的 `-arch` 值。或者用 `-gencode arch=compute_75,code=sm_75 -gencode arch=compute_86,code=sm_86` 同时生成多架构二进制（fat binary），兼容多种 GPU。

**编程陷阱：所有代码堆在一个 `.cu` 文件里，改一行等 30 秒编译。**
- **错误描述**：把动力学、代价、kernel、主循环全写进 `main.cu`，每次修改都触发整个文件的 nvcc 重编译。
- **现象/后果**：开发迭代极慢——改一个 CartPole 的物理参数（纯 C++ 改动）也要等 nvcc 编译全部 CUDA 代码。在一天的开发中，累计等待编译的时间可达数小时。
- **根本原因**：nvcc 编译比 g++ 慢一个量级（要做 PTX 生成、设备代码优化、主机/设备代码分离等额外步骤），把不需要 nvcc 的代码（纯 C++ 的控制循环、参数解析）混进 `.cu` 文件是不必要的代价。
- **正确做法**：只把真正需要 CUDA 的代码（kernel 实例化、cuRAND 调用）放在 `.cu` 里，其余都放 `.cpp`。通过 `CUDA_SEPARABLE_COMPILATION` 和头文件模板实现跨文件协作。

**概念陷阱：过度设计架构，还没写一行 kernel 就花一周搭框架。**
- **错误描述**：一上来就设计 5 层抽象（Sampler 基类、Dynamics 基类、Cost 基类、Controller 基类、Planner 基类），每层都有虚函数、工厂模式、注册宏，还没有一行能跑的代码。
- **现象/后果**：架构搭了一周，发现 CUDA 不支持虚函数（`__device__` 函数不能是 virtual），整套设计要推翻重来。或者架构能跑但过于复杂，调试时在 5 层抽象里追踪一个 bug 要跳 20 个文件。
- **根本原因**：CUDA 的编程模型与 CPU 的 OOP 范式有本质差异——设备代码在编译期就要确定类型（模板而非虚函数），运行时多态在 GPU 上代价极高甚至不可用。过早设计 CPU 风格的类继承体系，必然与 CUDA 的限制冲突。
- **正确做法**：先用最简单的结构（一个 kernel + 一个 dynamics struct + 一个 cost struct）跑通 CartPole，确认能正确控制后，再逐步提取模板、抽象出可复用的组件。"先跑通再重构"比"先设计再实现"在 CUDA 项目中更有效。

### 练习

1. **（架构设计题）** 假设你要在 Mini-MPPI 中新增一个"差速驱动小车"（differential drive）任务，STATE_DIM=3（x, y, theta），CONTROL_DIM=2（左右轮速度）。在上面的目录结构中，你需要新增哪些文件？需要修改哪些已有文件？如果架构设计得好，应该只需要新增 2 个文件、修改 0 个已有文件——验证你的架构是否达到了这个标准。

2. **（构建调试题）** 给你一个错误日志：`nvcc fatal: Unsupported gpu architecture 'compute_99'`。请解释这个错误的含义，说明如何查询你的 GPU 支持的计算能力，并给出修复方案。进一步：如果你的代码要同时支持实验室的 RTX 2080（sm_75）和服务器上的 A100（sm_80），CMakeLists 应该怎么写？

3. **（设计权衡题）** MPPI-Generic 用 C++ 模板实现多态（编译期确定类型），而不是用虚函数（运行时多态）。解释为什么 CUDA 环境下模板比虚函数更合适。思考：如果未来 CUDA 完全支持设备端虚函数调用，你会改用虚函数吗？为什么？

---

## §10.2 动力学模型封装 ⭐⭐

### 动机：为什么动力学模型是 MPPI 系统的第一个要写的组件

MPPI 的 rollout kernel 需要反复调用动力学模型——每条轨迹 $T$ 步，每步一次 `step(x, u, dt)`，$K$ 条轨迹就是 $K \times T$ 次调用。动力学模型是整个系统中**调用频率最高**的组件，它的正确性和性能直接决定了 MPPI 的行为和速度。

更重要的是，动力学模型必须**同时在 CPU 和 GPU 上可用**。在 CPU 上，你需要它来做单元测试（给一个已知状态和控制，验证输出是否符合物理直觉）、做 ground truth 对比（用 RK4 高精度积分作为参考）；在 GPU 上，rollout kernel 要在每个线程里调用它。如果你写了两份——CPU 版和 GPU 版——它们之间的不一致将是一个几乎无法调试的噩梦。

> **类比：同一张菜谱在家里做和在餐厅做。** 你希望菜谱（动力学方程）是一份，不管在哪个厨房（CPU/GPU）用都是同一份——而不是家里和餐厅各写一份、出了差异时不知道谁对。`__host__ __device__` 双标注就是让"同一份菜谱"在两个厨房都能用的工程手段。

### 如果不做双标注会怎样

写两份动力学代码（一份纯 CPU 的 `cartpole_cpu.hpp`，一份纯 GPU 的 `cartpole_gpu.cuh`），后果是确定性的：

| 问题 | 现象 |
|------|------|
| 改了 CPU 版忘改 GPU 版 | CPU 测试通过但 GPU rollout 用的是旧动力学，控制行为莫名其妙不对 |
| 两版的浮点行为不一致 | CPU 用 `double`、GPU 用 `float`，积分结果有微妙差异，难以判断是"正常精度差"还是"代码 bug" |
| 维护成本翻倍 | 新增一个参数（如阻尼系数），两个文件都要改，忘一个就是不一致 |

解决方案就是 `__host__ __device__` 双标注——让同一份代码在编译 `.cpp` 时被 g++ 当作普通 C++ 函数编译（忽略 CUDA 标注），在编译 `.cu` 时被 nvcc 同时编译为主机版和设备版。

### 实现：CartPole 动力学模型

**为什么从 CartPole 开始。** CartPole（倒立摆）是控制领域的"Hello World"：状态维度低（$n=4$：位置、速度、角度、角速度），控制维度极低（$m=1$：水平力），动力学方程有解析形式（不依赖数值求解器），swing-up 任务的成功/失败判定直观（摆起来了没有）。先在 CartPole 上跑通全流程，再换成高维的四旋翼，是最低风险的开发路径。

**为什么这样写。** 下面这段代码有几个关键设计决策。第一，用模板参数 `STATE_DIM` 和 `CONTROL_DIM` 而非硬编码——当你换成四旋翼时，改的是模板参数而非代码逻辑。第二，`Eigen::Matrix<float, STATE_DIM, 1>` 使用固定大小矩阵——固定大小矩阵在编译期确定维度，数据分配在栈（CPU）或寄存器（GPU）上，没有堆分配，在高频调用的 rollout 循环里这一点至关重要。第三，物理参数作为 `struct` 成员而非全局变量——方便每个 rollout 使用不同的参数（模型不确定性的采样，§10.5 MuJoCo 集成时会用到）。

```cpp
// include/dynamics/cartpole.hpp
#pragma once
#include <Eigen/Dense>
#include <cmath>

template<int STATE_DIM = 4, int CONTROL_DIM = 1>
struct CartPoleDynamics {
  // ---- 类型别名：让 kernel 通过 Dynamics::State 统一访问 ----
  static constexpr int state_dim = STATE_DIM;
  static constexpr int ctrl_dim  = CONTROL_DIM;
  using State   = Eigen::Matrix<float, STATE_DIM, 1>;
  using Control = Eigen::Matrix<float, CONTROL_DIM, 1>;

  // ---- __host__ __device__ 双标注 ----
  // 同一份代码在 CPU 上单元测试、在 GPU kernel 里调用
  __host__ __device__
  State step(const State& x, const Control& u, float dt) const {
    // 状态解包：x = [位置, 速度, 角度, 角速度]
    float theta  = x(2), dtheta = x(3);
    float sin_t  = sinf(theta), cos_t = cosf(theta);

    // CartPole 解析动力学（Euler 积分）
    float denom    = m_cart + m_pole * sin_t * sin_t;
    float ddx      = (u(0) + m_pole * sin_t *
                      (l * dtheta * dtheta + g * cos_t)) / denom;
    float ddtheta  = (-u(0) * cos_t
                      - m_pole * l * dtheta * dtheta * sin_t * cos_t
                      - (m_cart + m_pole) * g * sin_t) / (l * denom);

    // 前向 Euler 积分
    State x_new;
    x_new << x(0) + x(1) * dt,
             x(1) + ddx * dt,
             x(2) + x(3) * dt,
             x(3) + ddtheta * dt;
    return x_new;
  }

  // ---- 物理参数（struct 成员，非全局变量）----
  float m_cart = 1.0f;    // 小车质量 [kg]
  float m_pole = 0.1f;    // 摆杆质量 [kg]
  float l      = 0.5f;    // 摆杆半长 [m]
  float g      = 9.81f;   // 重力加速度 [m/s²]
};
```

**代码后的解读。** 这段代码看似简单，但有几个"看不见的设计"值得深挖。

其一，`sinf` / `cosf` 而非 `sin` / `cos`。在 CUDA 设备代码中，`sinf` 是原生的单精度正弦函数，直接映射到 GPU 的特殊函数单元（SFU），一个周期完成；而 `sin` 在设备代码中可能被解析为双精度版本，双精度在消费级 GPU 上的吞吐量只有单精度的 1/32（RTX 3080 的 fp64 吞吐量仅 0.56 TFLOPS vs fp32 的 29.8 TFLOPS）。这不是吹毛求疵——在 $K=4096, T=64$ 的 rollout 循环里，动力学的三角函数被调用 $4096 \times 64 = 262144$ 次，单次慢 32 倍的影响是巨大的。

其二，前向 Euler 积分而非 RK4。Euler 积分每步只调一次动力学函数（$O(1)$），RK4 每步要调四次（$O(4)$）。在 MPPI 的 rollout 里，精度要求不高——rollout 本身带着随机噪声，Euler 的截断误差被噪声淹没了；而 RK4 的 4 倍计算量直接反映在 kernel 时间上。所以工程 MPPI 几乎都用 Euler。不过要注意：如果你的 `dt` 很大或动力学很刚（stiff），Euler 可能发散——这时要减小 `dt` 而非换 RK4（后者也不适合刚系统，需要隐式方法）。

其三，`const State& x` 的引用传递。在 CPU 上这避免了拷贝；在 GPU 上，Eigen 的固定大小矩阵足够小（4 个 float = 16 字节），编译器通常会把它优化到寄存器里，引用传递在设备代码中的实际行为可能与 CPU 不同（nvcc 可能会自动内联整个函数并消除引用），但写法上保持一致性是好习惯。

### 实现：Quadrotor 动力学模型

**为什么还需要第二个模型。** CartPole 的 STATE_DIM=4 太低了——在 GPU 上几乎看不出线程映射策略的差异（4 个寄存器 vs 12 个寄存器对占用率的影响可以忽略）。Quadrotor 的 STATE_DIM=12（位置 3 + 速度 3 + 欧拉角 3 + 角速度 3）才是真实机器人的典型维度，能暴露出寄存器压力、warp divergence 等 GPU 效率问题。

```cpp
// include/dynamics/quadrotor.hpp
#pragma once
#include <Eigen/Dense>
#include <cmath>

template<int STATE_DIM = 12, int CONTROL_DIM = 4>
struct QuadrotorDynamics {
  static constexpr int state_dim = STATE_DIM;
  static constexpr int ctrl_dim  = CONTROL_DIM;
  using State   = Eigen::Matrix<float, STATE_DIM, 1>;
  using Control = Eigen::Matrix<float, CONTROL_DIM, 1>;

  __host__ __device__
  State step(const State& x, const Control& u, float dt) const {
    // 状态解包：[px,py,pz, vx,vy,vz, roll,pitch,yaw, wx,wy,wz]
    float roll = x(6), pitch = x(7), yaw = x(8);
    float sr = sinf(roll),  cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);
    float sy = sinf(yaw),   cy = cosf(yaw);

    // 控制：u = [总推力, 力矩x, 力矩y, 力矩z]
    float F  = u(0);   // 总推力（沿机体 z 轴）
    float tx = u(1), ty = u(2), tz = u(3);   // 三轴力矩

    // 位置导数 = 速度
    // 速度导数 = 旋转矩阵 * 推力/质量 + 重力
    float ax = F / mass * (cr*sp*cy + sr*sy);
    float ay = F / mass * (cr*sp*sy - sr*cy);
    float az = F / mass * (cr*cp) - g;

    // 角速度导数（简化为对角惯量）
    float dwx = (tx + (Iyy - Izz) * x(10) * x(11)) / Ixx;
    float dwy = (ty + (Izz - Ixx) * x(9)  * x(11)) / Iyy;
    float dwz = (tz + (Ixx - Iyy) * x(9)  * x(10)) / Izz;

    State x_new;
    // 位置
    x_new(0) = x(0) + x(3) * dt;
    x_new(1) = x(1) + x(4) * dt;
    x_new(2) = x(2) + x(5) * dt;
    // 速度
    x_new(3) = x(3) + ax * dt;
    x_new(4) = x(4) + ay * dt;
    x_new(5) = x(5) + az * dt;
    // 欧拉角（简化：小角度近似下角速度≈欧拉角导数）
    x_new(6)  = x(6)  + x(9)  * dt;
    x_new(7)  = x(7)  + x(10) * dt;
    x_new(8)  = x(8)  + x(11) * dt;
    // 角速度
    x_new(9)  = x(9)  + dwx * dt;
    x_new(10) = x(10) + dwy * dt;
    x_new(11) = x(11) + dwz * dt;

    return x_new;
  }

  // ---- 物理参数 ----
  float mass = 1.0f;     // 总质量 [kg]
  float g    = 9.81f;    // 重力加速度
  float Ixx  = 0.01f;    // 惯量矩阵对角元
  float Iyy  = 0.01f;
  float Izz  = 0.02f;
};
```

**代码后的解读与反事实。** Quadrotor 的 `step` 函数比 CartPole 复杂得多——12 个状态分量的更新涉及 6 次三角函数（`sinf/cosf` 各 3 次）和大量乘法。在 GPU kernel 中，这意味着**每个线程的寄存器需求显著增加**：CartPole 的 `step` 只需约 10-15 个寄存器，Quadrotor 可能需要 40-60 个。

这直接影响 GPU 的**占用率（occupancy）**。以 RTX 3080 为例，每个 SM 有 65536 个寄存器；如果每线程用 60 个寄存器、block size 为 256，则一个 block 需要 $256 \times 60 = 15360$ 个寄存器，每个 SM 最多驻留 4 个 block（$65536/15360 \approx 4.3$），对应 1024 个线程——而最大支持 2048 线程，占用率只有 50%。

这就是 Ch2 §2.2 提到的"寄存器压力"在实际代码中的体现——**换一个更复杂的动力学模型，可能不需要改 kernel 的一行代码，但 GPU 性能会因为寄存器压力而下降**。§10.6 的 profiling 会用 ncu 实测这个占用率。

### Eigen 在 CUDA 中的陷阱

**为什么 Eigen 和 CUDA 的结合这么多坑。** Eigen 是为 CPU 设计的——它大量使用 C++ 的表达式模板（expression template）、SIMD 向量化（SSE/AVX）、惰性求值（lazy evaluation）。这三者在 CUDA 设备代码中要么不可用、要么行为不同、要么会导致隐蔽的性能问题。

**陷阱一：动态大小矩阵在设备代码中堆分配。** `Eigen::MatrixXf`（动态大小）在构造时会 `new` 一块堆内存。在 CPU 上这是正常操作；在 GPU 的 `__device__` 函数里，"堆分配"意味着调用设备端的 `malloc`——这是一个**极其昂贵**的操作（通过全局原子操作竞争分配设备堆内存），在 $K \times T$ 次调用的 rollout 循环里，它会让 kernel 慢 10-100 倍，甚至直接耗尽设备堆空间导致 `cudaErrorMemoryAllocation`。

```cpp
// ❌ 错误：设备代码中使用动态大小矩阵
__device__ void bad_step() {
    Eigen::MatrixXf x(12, 1);   // 堆分配！设备端 malloc，极慢
    // ...
}

// ✅ 正确：使用固定大小矩阵
__device__ void good_step() {
    Eigen::Matrix<float, 12, 1> x;   // 栈分配（GPU 上即寄存器/局部内存）
    // ...
}
```

**如果犯了这个错误，现象是什么？** kernel 不会报编译错误（nvcc 允许设备端 `malloc`），但运行时间从正常的 1-2 ms 暴增到 500 ms 甚至更长，GPU 利用率反而很低（大量线程在等待堆分配的锁）。用 ncu 可以看到 `__device_malloc` 的调用次数异常高——这是唯一的诊断线索，如果你不知道去找它，这个 bug 可以隐藏数周。

**陷阱二：Eigen 的 SIMD 指令在设备代码中无效。** Eigen 的核心性能优化依赖 SSE/AVX 向量化——在 CPU 上，一条 AVX 指令可以同时处理 8 个 float。但 GPU 没有 SSE/AVX（GPU 的并行是线程级的 SIMT，不是指令级的 SIMD），所以 Eigen 在设备代码中编译时，SIMD 优化被默默关闭，代码退化成标量循环。

这意味着：**Eigen 在 GPU 上不会让你的动力学更快，它只是一个方便的容器**（让你用 `x(0)` 而非 `x[0]`、用 `x_new << ...` 而非逐元素赋值）。对于 STATE_DIM <= 12 的小矩阵，Eigen 的容器开销可以忽略；但不要期望 Eigen 在 GPU 上提供任何性能加速——它不会。

**陷阱三：在 `__device__` 函数中使用 Eigen 的动态方法。** 某些 Eigen 方法（如 `x.resize(n)`、`x.conservativeResize(n)`）在动态矩阵上调用时会触发堆分配/释放，在设备代码中同样致命。即使你的矩阵是固定大小的，如果你不小心调用了 `.eval()`（Eigen 的表达式模板求值函数），它在某些情况下也可能创建临时对象——不过对于固定大小矩阵，临时对象也是栈分配的，问题不大。

**最佳实践总结。**

```cpp
// Eigen 在 CUDA 中的安全用法清单：
// 1. 只用 Eigen::Matrix<float, N, 1>（固定大小、float）
// 2. 不用 Eigen::MatrixXf / Eigen::VectorXf（动态大小）
// 3. 用 sinf/cosf 不用 sin/cos（单精度设备函数）
// 4. CMake 加 --expt-relaxed-constexpr（让 Eigen 的 constexpr 通过）
// 5. CMake 加 -DEIGEN_NO_CUDA（可选：关闭 Eigen 的 CUDA 检测，
//    避免某些版本的 Eigen 在设备代码中引入不兼容的路径）
```

### ⚠️ 常见陷阱

**编程陷阱：在 CUDA device 函数中使用 `Eigen::VectorXf`（动态大小矩阵）。**
- **错误描述**：动力学的 `step` 函数里声明 `Eigen::VectorXf x(12)` 而非 `Eigen::Matrix<float, 12, 1> x`。
- **现象/后果**：编译通过（nvcc 不会拒绝设备端堆分配），但 kernel 执行时间暴增 10-100 倍，GPU 利用率极低。用 `nsys` 看到 kernel 时间从 ~1 ms 变成 ~500 ms；用 `ncu` 看到 `__device_malloc` 调用次数 = K * T * state_dim。
- **根本原因**：`VectorXf` 构造时在设备堆上分配内存（设备端 `malloc`），这是一个全局原子操作，数千线程竞争同一把锁。
- **正确做法**：所有在 `__device__` 函数中使用的 Eigen 类型必须是固定大小：`Eigen::Matrix<float, N, 1>`。

**编程陷阱：在 device 代码中使用 `sin`/`cos` 而非 `sinf`/`cosf`。**
- **错误描述**：从纯 CPU 代码直接复制动力学方程，没有把 `sin`/`cos` 改成 `sinf`/`cosf`。
- **现象/后果**：编译通过（nvcc 会把 `sin` 解析为设备端双精度版本），但在消费级 GPU 上（fp64 吞吐量仅 fp32 的 1/32），kernel 速度慢 5-30 倍，且占用的寄存器数翻倍（双精度寄存器宽度是单精度的 2 倍）。
- **根本原因**：消费级 GPU 的双精度单元极少（设计给图形渲染，不需要 fp64）；即使在数据中心 GPU（A100）上，fp64 吞吐也只有 fp32 的 1/2。
- **正确做法**：设备代码中所有数学函数用 `f` 后缀版本（`sinf`、`cosf`、`sqrtf`、`expf`），除非你有明确的双精度需求。

**概念陷阱：期望 Eigen 在 GPU 上提供 SIMD 加速。**
- **错误描述**：认为"用 Eigen 就自动快了"——在 CPU 上 Eigen 通过 SSE/AVX 确实有 4-8 倍加速，于是期望 GPU 上也一样。
- **现象/后果**：费心把所有标量运算都写成 Eigen 矩阵形式（"向量化"），实测发现并没有更快，反而因为 Eigen 的表达式模板在设备代码中引入了额外开销。
- **根本原因**：GPU 没有 SSE/AVX，并行粒度是线程（SIMT），Eigen 的 SIMD 后端在设备代码中被关闭。对于 state_dim <= 12 的小向量，Eigen 在 GPU 上只是一个"语法糖容器"，不是性能加速器。
- **正确做法**：在 GPU 上把 Eigen 当作"方便读写"的容器即可，不要为了"向量化"而扭曲代码结构。真正的 GPU 加速来自线程级并行（K 条 rollout 同时跑），不是指令级的 SIMD。

### 练习

1. **（实现题）** 在 CPU 上为 CartPole 动力学写一个单元测试：从静止的竖直下垂状态 $x = [0, 0, \pi, 0]$ 开始，施加零控制 $u = 0$，积分 100 步（$dt = 0.01$），验证摆杆是否保持在 $\theta \approx \pi$（稳定平衡点）。然后从竖直向上 $x = [0, 0, 0, 0]$ 加微小扰动 $x = [0, 0, 0.01, 0]$，验证系统是否自然倒下（$\theta$ 增大）。这个测试能否同时验证 CPU 和 GPU 版本的一致性？

2. **（分析题）** Quadrotor 的 `step` 函数使用了前向 Euler 积分和欧拉角表示。列出这两个选择各自的局限性（提示：Euler 积分在大 dt 下不稳定；欧拉角在 pitch = 90 度时有万向锁）。如果你要改用 RK4 积分和四元数表示，`step` 函数的计算量会增加多少？在 MPPI rollout 的上下文中，这个增加值不值得？

3. **（设计扩展题）** 设计一个 `Go2Dynamics` struct（宇树 Go2 四足机器人，12 关节），STATE_DIM=36（机体 6DoF + 速度 6 + 关节角 12 + 关节角速度 12），CONTROL_DIM=12。你不需要写完整的动力学方程（那需要完整的 URDF 和接触模型），只需要设计 struct 的接口（`State`/`Control` 类型别名、`step` 函数签名、物理参数成员），并回答：STATE_DIM=36 对 GPU 的寄存器压力有多大影响？每线程大约需要多少个寄存器？RTX 3080 的每 SM 占用率会降到多少？

---

## §10.3 CUDA Rollout Kernel ⭐⭐⭐

### 动机：把"K 条独立 rollout"真正同时跑起来

§10.2 准备好了动力学模型——一个能在 CPU 和 GPU 上都调用的 `step` 函数。现在要写的是 MPPI 系统中**计算量最大、也最能从 GPU 并行中获益**的组件：rollout kernel。

回顾 Ch2 §2.2 的核心论点：MPPI 的 $K$ 条 rollout 彼此完全独立——第 $k$ 条轨迹的前向积分不依赖第 $j$ 条的任何中间结果。这种"尴尬并行"结构是 GPU 的完美负载。本节要做的，就是把 Ch2 的教学示意 kernel 写成一个**真正能编译、能 profile、能调优**的实现。

> **本质洞察**：rollout kernel 的核心设计张力，不在于"能不能并行"——那是天然的——而在于**内存层级的选择**和**线程映射的粒度**。同一份动力学代码，用不同的线程映射跑，性能差异可达 3-5 倍。这不是算法层面的差异，是纯粹的**硬件工程**。

### 如果不做 GPU 并行会怎样

重述 Ch2 §2.2 的基准数字，这是本节存在的根本理由：

| 实现 | K=2048, T=64, 12 维状态 | 每次迭代耗时 | 50 Hz 可行？ |
|------|------------------------|------------|-------------|
| CPU 串行（Eigen, i7-12700） | 一条接一条跑 | ~80 ms | 否（差 4 倍） |
| GPU 并行（RTX 3080） | 2048 条同时跑 | ~1.5 ms | 是（余量充裕） |

### 实现：rollout kernel 模板

**为什么这样写。** 这个 kernel 是 Ch2 §2.2 教学示意的工程版本，关键改进有三处。第一，用模板参数 `Dynamics` 和 `CostFunc` 替换硬编码的动力学和代价——换任务只需换模板参数，kernel 结构不动。第二，加入 `__restrict__` 指针修饰（restrict pointer）——告诉编译器这些指针指向的内存不重叠，允许更激进的优化（如寄存器缓存、指令重排）。第三，用 `constexpr int` 代替 kernel 参数传维度——编译期已知的维度让编译器能展开循环、优化寄存器分配。

```cpp
// include/mppi/rollout_kernel.cuh
#pragma once
#include <cuda_runtime.h>

// ---- CUDA 错误检查宏（开发期必需，发布期可编译掉）----
#define CUDA_CHECK(call) do {                                    \
  cudaError_t err = (call);                                      \
  if (err != cudaSuccess) {                                      \
    printf("CUDA error at %s:%d — %s\n",                        \
           __FILE__, __LINE__, cudaGetErrorString(err));          \
    exit(EXIT_FAILURE);                                          \
  }                                                              \
} while(0)

// ---- Rollout Kernel：每 thread 一条轨迹（方案 A）----
// 教学优先的实现：简单、易懂、易调试。
// 性能瓶颈在高维状态下的寄存器压力——§10.6 会量化。
template<class Dynamics, class CostFunc>
__global__ void rollout_kernel(
    const float* __restrict__ x0,      // 初始状态 [state_dim]
    const float* __restrict__ u_mean,  // 均值控制序列 [T * ctrl_dim]
    const float* __restrict__ noise,   // 噪声 [K * T * ctrl_dim]
    float* __restrict__ costs,         // 输出代价 [K]
    int K, int T, float dt)
{
  // ---- 线程映射：第 k 个线程算第 k 条 rollout ----
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= K) return;    // 越界保护（K 不一定是 blockDim 的整数倍）

  // ---- 动力学和代价函数实例（编译期确定类型）----
  Dynamics dyn;
  CostFunc cost_fn;

  // ---- 从 global memory 加载初始状态到寄存器 ----
  // 这一步只做一次，之后状态在寄存器里就地更新
  constexpr int SD = Dynamics::state_dim;
  constexpr int CD = Dynamics::ctrl_dim;
  typename Dynamics::State x;
  for (int i = 0; i < SD; ++i) x(i) = x0[i];

  // ---- 前向 rollout：T 步串行积分 ----
  float total_cost = 0.0f;
  for (int t = 0; t < T; ++t) {
    // 组装微扰控制 v = u_mean + noise
    typename Dynamics::Control u;
    for (int d = 0; d < CD; ++d) {
      u(d) = u_mean[t * CD + d]
           + noise[k * T * CD + t * CD + d];
    }
    // 前向动力学
    x = dyn.step(x, u, dt);
    // 累加运行代价
    total_cost += cost_fn.running(x, u);
  }
  // 终端代价
  total_cost += cost_fn.terminal(x);

  // ---- 写回 global memory（每条 rollout 只写一次）----
  costs[k] = total_cost;
}
```

**代码后的深入分析。**

**线程映射分析。** 这个 kernel 用的是"每 thread 一条轨迹"（方案 A），Ch2 §2.2 分析过其优缺点。优点是实现极简——每个线程独立完成一整条 rollout，无需 warp 内协作，无需 shared memory。缺点是**寄存器压力**：每个线程独占保存整条轨迹的当前状态 $x_t$（STATE_DIM 个 float）、当前控制 $v_t$（CTRL_DIM 个 float）、以及动力学计算的中间变量。

对 CartPole（SD=4, CD=1），寄存器需求约 15-20 个，RTX 3080 每 SM 65536 寄存器、block size 256 时占用率 ~100%，没有问题。对 Quadrotor（SD=12, CD=4），寄存器需求飙升到 50-70 个，占用率降到 ~50%——虽然还在可接受范围，但已经能在 profiling 中看到明显的寄存器溢出（register spilling to local memory）。

**如果不做越界保护 `if (k >= K) return;` 会怎样？** 如果 K=2000、block size=256，grid 需要 $\lceil 2000/256 \rceil = 8$ 个 block，总共启动 $8 \times 256 = 2048$ 个线程，其中 48 个是"多出来的"。没有越界保护，这 48 个线程会用越界的 index 读取 `noise` 数组之外的内存——读到垃圾值，算出错误的代价，写入 `costs` 数组之外的内存（如果 `costs` 恰好只分配了 K 个 float），造成**内存越界写入（buffer overwrite）**。CUDA 不像 CPU 那样会给你一个 segfault——设备端的内存越界通常**不报错、不崩溃**，只是默默地写坏了别的数据。这是 CUDA 编程中最危险的 bug 之一：表面上程序正常运行，但结果是错的，且每次运行结果可能不同（取决于被写坏的是哪块内存）。

**内存访问模式分析。** kernel 中有两类 global memory 访问：

第一类是 `x0[i]` 的读取——所有 K 个线程都读同一份 `x0`，这是一个**广播（broadcast）**模式。CUDA 的 L1/L2 缓存会把 `x0` 缓存住，K 个线程实际上只产生一次 global memory 事务。

第二类是 `noise[k * T * CD + t * CD + d]` 的读取——每个线程 k 读自己那条 rollout 的噪声。这里的访存模式取决于线程编号与数据布局的关系。当 `t` 和 `d` 固定时，相邻线程 `k` 和 `k+1` 读取的地址差 `T * CD` 个 float——这是一个**非合并（non-coalesced）**的访问模式，因为 warp 内 32 个相邻线程读取的地址不连续（间距 = T * CD），无法合并成一次内存事务，而是拆成多次事务。

优化方式是**转置噪声数组的布局**：把 `noise` 从 `[K, T, CD]` 改成 `[T, CD, K]` 或 `[T, K, CD]`，使得同一 warp 的相邻线程读取的地址连续（合并访问，coalesced access）。这个优化在 K 大、T 大时效果显著（减少 global memory 事务数 4-8 倍），但增加了代码复杂度。本章先用简单布局跑通，§10.6 的 profiling 会量化这个瓶颈并指导是否值得优化。

### Shared Memory 优化：把 u_mean 缓存到 block 内

**为什么需要 shared memory。** 上面的 kernel 中，`u_mean[t * CD + d]` 在每个时步被 block 内所有线程读取——同一个 block 的 256 个线程都在读同一份 `u_mean`。虽然 L1 缓存会帮忙，但显式地把 `u_mean` 加载到 shared memory 可以：(1) 保证命中而不是"期望缓存帮忙"；(2) 减少对 L1 缓存容量的竞争（把 L1 留给 `noise` 数组的读取）。

```cpp
// ---- 优化版：用 shared memory 缓存 u_mean ----
template<class Dynamics, class CostFunc>
__global__ void rollout_kernel_smem(
    const float* __restrict__ x0,
    const float* __restrict__ u_mean,
    const float* __restrict__ noise,
    float* __restrict__ costs,
    int K, int T, float dt)
{
  constexpr int CD = Dynamics::ctrl_dim;
  constexpr int SD = Dynamics::state_dim;

  // ---- 把 u_mean 加载到 shared memory ----
  // u_mean 大小 = T * CD 个 float，block 内所有线程共享
  extern __shared__ float s_u_mean[];
  int tid = threadIdx.x;
  int total_u = T * CD;
  // 协作加载：每个线程搬一部分
  for (int i = tid; i < total_u; i += blockDim.x) {
    s_u_mean[i] = u_mean[i];
  }
  __syncthreads();   // 确保加载完毕后再开始 rollout

  int k = blockIdx.x * blockDim.x + tid;
  if (k >= K) return;

  Dynamics dyn;
  CostFunc cost_fn;
  typename Dynamics::State x;
  for (int i = 0; i < SD; ++i) x(i) = x0[i];

  float total_cost = 0.0f;
  for (int t = 0; t < T; ++t) {
    typename Dynamics::Control u;
    for (int d = 0; d < CD; ++d) {
      // 从 shared memory 读 u_mean，从 global memory 读 noise
      u(d) = s_u_mean[t * CD + d]
           + noise[k * T * CD + t * CD + d];
    }
    x = dyn.step(x, u, dt);
    total_cost += cost_fn.running(x, u);
  }
  total_cost += cost_fn.terminal(x);
  costs[k] = total_cost;
}
```

**如果不用 shared memory 会怎样。** 在 CartPole（CD=1, T=30）的规模下，`u_mean` 只有 30 个 float = 120 字节，L1 缓存命中率几乎 100%，shared memory 优化的收益微乎其微（<1%）。但在 Quadrotor（CD=4, T=64）的规模下，`u_mean` 有 256 个 float = 1024 字节，当多个 warp 同时竞争 L1 缓存时，缓存逐出（eviction）会导致重复的 global memory 访问。此时 shared memory 缓存可以带来 5-15% 的 kernel 时间减少。§10.6 的 ncu profiling 会给出精确数字。

**shared memory 启动语法。** 注意 kernel 启动时需要指定 shared memory 大小：

```cpp
// 启动 kernel 时的第三个 <<< >>> 参数 = shared memory 字节数
int smem_bytes = T * CD * sizeof(float);
int blocks = (K + 255) / 256;
rollout_kernel_smem<CartPoleDynamics<>, CartPoleCost>
    <<<blocks, 256, smem_bytes>>>(x0_d, u_mean_d, noise_d, costs_d, K, T, dt);
```

### 代价函数模板：rollout kernel 的另一半

**为什么现在才讲代价函数。** 上面的 kernel 反复调用 `cost_fn.running(x, u)` 和 `cost_fn.terminal(x)`，但 `CostFunc` 究竟长什么样一直没展开——因为它的接口设计要等 kernel 结构定下来才说得清。代价函数和动力学是 rollout kernel 的两块拼图：动力学决定"状态怎么演化"，代价决定"这条轨迹好不好"。两者都用模板参数注入，kernel 结构对它们一无所知——这正是 §10.1 模板化设计哲学的兑现。

**接口约定。** 一个合法的 `CostFunc` 只需提供两个 `__host__ __device__` 方法：`running(x, u)` 返回单步运行代价 $q(x_t, u_t)$，`terminal(x)` 返回终端代价 $\phi(x_T)$。回顾 Ch2 §2.1 的代价结构 $S_k = \phi(x_T) + \sum_{t} q(x_t, u_t)$——kernel 里 `total_cost += running(...)` 累加运行项、最后 `+= terminal(...)` 加终端项，正是这个求和的逐字实现。

```cpp
// include/cost/cartpole_cost.hpp
#pragma once
#include <Eigen/Dense>
#include <cmath>

// CartPole swing-up 代价：目标是把摆杆从下垂（θ=π）摆到竖直（θ=0）
struct CartPoleCost {
  using State   = Eigen::Matrix<float, 4, 1>;
  using Control = Eigen::Matrix<float, 1, 1>;

  // ---- 运行代价：每个时步都累加 ----
  __host__ __device__
  float running(const State& x, const Control& u) const {
    // 用 (1 - cosθ) 而非 θ² 度量"离竖直有多远"——它在 θ=0 处为 0、
    // θ=π 处取最大值 2，且对 θ 的 2π 周期性天然兼容（θ 和 θ+2π 等价）。
    // 若改用 θ²，则 θ=2π（同样是竖直）会被错判为高代价——这是新手常踩的坑。
    float angle_cost = 1.0f - cosf(x(2));        // 角度偏差 ∈ [0, 2]
    float pos_cost   = x(0) * x(0);              // 小车别跑太远
    float vel_cost   = 0.1f * x(3) * x(3);       // 抑制角速度（鼓励停在顶点）
    float ctrl_cost  = 0.001f * u(0) * u(0);     // 控制能量正则
    return w_angle * angle_cost + w_pos * pos_cost
         + w_vel * vel_cost + ctrl_cost;
  }

  // ---- 终端代价：只在轨迹末端加一次，权重更大以"锁定"目标 ----
  __host__ __device__
  float terminal(const State& x) const {
    float angle_cost = 1.0f - cosf(x(2));
    return w_terminal * angle_cost
         + w_vel_term * (x(1) * x(1) + x(3) * x(3));   // 末端速度归零
  }

  // ---- 权重（struct 成员，便于按任务调）----
  float w_angle    = 1.0f;
  float w_pos      = 0.5f;
  float w_vel      = 1.0f;
  float w_terminal = 50.0f;    // 终端权重远大于运行权重——见下文解读
  float w_vel_term = 5.0f;
};
```

**代码后的解读。** 三个设计决策值得展开。

其一，**为什么用 $1-\cos\theta$ 而不是 $\theta^2$**。这是 R6B"不是 X 而是 Y"的典型场景。$\theta^2$ 看似直观（偏差越大代价越大），但它把角度当成了无界的实数——而角度是周期的（$\theta$ 与 $\theta+2\pi$ 是同一姿态）。如果 swing-up 过程中摆杆转了一整圈到 $\theta=2\pi$（物理上等于竖直、代价应为 0），$\theta^2=4\pi^2\approx 39$ 却给出巨大的虚假代价，误导优化器去"解开"这一圈。而 $1-\cos\theta$ 自带 $2\pi$ 周期，$\theta=0$ 与 $\theta=2\pi$ 都取 0——它度量的是真正的"姿态差"，不是"数值差"。

其二，**为什么终端权重 $w_\text{terminal}=50$ 远大于运行权重**。回顾 Ch2 §2.1：运行代价累加 $T$ 次、终端代价只加 1 次。如果两者权重相同，终端项会被 $T$ 倍的运行项淹没，优化器只顾"路上每步都还行"而不在乎"最终停没停在顶点"。放大终端权重，等于告诉优化器"过程可以差一点，但结局必须站住"——这对 swing-up 这种"目标态导向"任务至关重要。

其三，**$\sin\theta$、$\cos\theta$ 在代价里同样要用 `cosf`**。和动力学一节同理——代价函数也在 kernel 里被 $K\times T$ 次调用，用双精度 `cos` 会拖慢整个 kernel。

> **本质洞察：代价函数是 MPPI 唯一的"任务规约"——它替代了传统最优控制里需要手工推导的一切。** LQR 要你给出 $Q$、$R$ 矩阵并求解 Riccati 方程；MPC 要你写出约束的解析形式。MPPI 把这一切压缩成一个标量函数 `running + terminal`——你只需要回答"这条轨迹有多好"，剩下的交给采样。这是 MPPI"无梯度、无模型可微性要求"的根源：代价函数甚至可以是不可导的（如"撞墙 +1000 否则 0"），因为采样只需要比较代价的**大小**，不需要它的**梯度**。

### 线程映射方案 B：每 warp 一条轨迹

**为什么需要第二种映射。** 前面所有 kernel 用的都是"每 thread 一条轨迹"（方案 A），Ch2 §2.2 和本节的 warp divergence 分析多次提到"方案 B（每 warp 一条）能从根本上避免分歧"——但一直没给实现。现在补上，让你能亲手对比两种映射的性能差异。

**核心思想。** 方案 A 让一个线程独扛一整条 rollout，导致状态向量全压在该线程的寄存器里（高维状态 → 寄存器溢出）。方案 B 反过来：让一个 **warp（32 线程）协同处理同一条** rollout——状态向量的各分量分摊到 warp 的不同 lane 上，每个线程只持有状态的一部分。这样单线程寄存器需求大降，且同一 warp 处理同一条轨迹，永远走同一条代价分支——**warp divergence 从结构上被消除**。

```cpp
// ---- 方案 B：每 warp 一条轨迹（warp-per-rollout）----
// 适合高维状态（如 Quadrotor SD=12）——状态分摊到 warp 的 32 个 lane，
// 降低单线程寄存器压力，且同一条 rollout 永不发生 warp 内分歧。
template<class Dynamics, class CostFunc>
__global__ void rollout_kernel_warp(
    const float* __restrict__ x0,
    const float* __restrict__ u_mean,
    const float* __restrict__ noise,
    float* __restrict__ costs,
    int K, int T, float dt)
{
  constexpr int SD = Dynamics::state_dim;
  constexpr int CD = Dynamics::ctrl_dim;

  // ---- 线程到 warp 的映射 ----
  int global_tid = blockIdx.x * blockDim.x + threadIdx.x;
  int warp_id    = global_tid / 32;     // 第几个 warp = 第几条 rollout
  int lane       = global_tid % 32;     // warp 内的 lane 编号 [0,31]
  int k = warp_id;
  if (k >= K) return;

  Dynamics dyn;
  CostFunc cost_fn;

  // ---- 状态放 shared memory，整个 warp 协作读写 ----
  // 每个 warp 在 shared memory 里有一份 state（SD 个 float）
  extern __shared__ float s_state[];
  int warps_per_block = blockDim.x / 32;
  int local_warp = threadIdx.x / 32;
  float* x = &s_state[local_warp * SD];   // 本 warp 的状态切片

  // lane 0..SD-1 各负责加载一个状态分量（其余 lane 空闲）
  if (lane < SD) x[lane] = x0[lane];
  __syncwarp();                           // warp 内同步（比 __syncthreads 轻）

  float total_cost = 0.0f;
  for (int t = 0; t < T; ++t) {
    // 整条 rollout 的 step 由 lane 0 串行算（动力学是强耦合的递推，
    // 难以在 lane 间细粒度并行——这是方案 B 的固有局限，见下文权衡）
    if (lane == 0) {
      typename Dynamics::State xv, xn;
      for (int i = 0; i < SD; ++i) xv(i) = x[i];
      typename Dynamics::Control u;
      for (int d = 0; d < CD; ++d)
        u(d) = u_mean[t * CD + d] + noise[k * T * CD + t * CD + d];
      xn = dyn.step(xv, u, dt);
      for (int i = 0; i < SD; ++i) x[i] = xn(i);
      total_cost += cost_fn.running(xn, u);
    }
    __syncwarp();      // 等 lane 0 写完 shared state，其余 lane 才能继续
  }
  if (lane == 0) {
    typename Dynamics::State xv;
    for (int i = 0; i < SD; ++i) xv(i) = x[i];
    total_cost += cost_fn.terminal(xv);
    costs[k] = total_cost;     // 每条 rollout 由 lane 0 写一次
  }
}
```

**代码后的诚实权衡。** 这个方案 B 实现刻意保留了一个"教学诚实"的细节：动力学 `step` 由 lane 0 串行算，其余 31 个 lane 在 `__syncwarp()` 处等待。**这是因为 CartPole/Quadrotor 这类低维动力学的 `step` 是强耦合的递推**——$x_{t+1}$ 的每个分量都依赖 $x_t$ 的多个分量，难以干净地在 lane 间切分。换句话说，对**低维**系统，方案 B 的"warp 协作"收益有限，甚至可能因为 31 个 lane 空转而比方案 A 更慢。

那方案 B 什么时候真正划算？答案是**高维、可向量化**的动力学——比如状态有几十上百维、`step` 里有大段可并行的矩阵-向量乘（如神经网络世界模型的前向，Ch7 TD-MPC 的场景）。这时把矩阵乘的各行分到 32 个 lane 上并行，warp 协作的收益才显现。

> **本质洞察：线程映射没有"更优"，只有"更匹配"。** 方案 A（每线程一条）匹配低维、廉价的动力学——简单、寄存器够用、并行度直接等于 K。方案 B（每 warp 一条）匹配高维、可向量化的动力学——单线程寄存器压力小、无 warp 分歧，但要求 `step` 内部有可挖掘的并行性。MPPI-Generic 的做法是**两者都支持**，让用户按动力学复杂度选——这正是 Ch2 §2.3 "Sampler/Controller 维度正交"设计能容纳不同映射的价值。本章先用方案 A 跑通，理解方案 B 的存在是为了在高维场景下有牌可打。

**方案 A vs 方案 B 对比表。**

| 维度 | 方案 A（每线程一条） | 方案 B（每 warp 一条） |
|------|---------------------|----------------------|
| 并行度 | $K$ 条 = $K$ 个线程 | $K$ 条 = $K$ 个 warp = $32K$ 个线程 |
| 单线程寄存器压力 | 高（整条状态压在一个线程） | 低（状态摊到 shared memory） |
| Warp divergence | 有（32 条不同 rollout 走不同分支） | 无（同 warp 同 rollout） |
| 实现复杂度 | 低（最直观） | 高（lane 映射 + `__syncwarp`） |
| 适用动力学 | 低维、廉价 `step`（CartPole） | 高维、可向量化 `step`（神经网络模型） |
| shared memory 用量 | 仅 `u_mean`（可选） | 每 warp 一份 state（必需） |

### Register Pressure 与 Warp Divergence 分析

**Register Pressure（寄存器压力）。** 每个 CUDA 线程有有限的寄存器资源（由编译器分配）。当 `step` 函数中的中间变量太多时，编译器会被迫把一部分寄存器**溢出到局部内存（local memory）**——而局部内存实际上是在 global memory 上的，速度慢 100 倍以上。

用 nvcc 的 `--ptxas-options=-v` 标志可以在编译期看到每个 kernel 使用的寄存器数量：

```bash
# 编译时查看寄存器使用
nvcc --ptxas-options=-v -c mppi_kernel.cu
# 输出示例：
# ptxas info: Used 47 registers, 4096+0 bytes smem, 380 bytes cmem[0]
```

如果你看到 `Used 47 registers` 对于 Quadrotor 的 kernel，意味着每线程 47 个寄存器、block size 256 时总需 $256 \times 47 = 12032$ 个——RTX 3080 每 SM 65536 个寄存器，最多驻留 $\lfloor 65536/12032 \rfloor = 5$ 个 block = 1280 个线程，占用率 = $1280/2048 = 62.5\%$。还行，但已经不是满载。

如果看到 `Used 72 registers`（比如你不小心用了 `double` 或 Eigen 的动态矩阵），则每 block 需要 $256 \times 72 = 18432$ 个寄存器，每 SM 只驻留 3 个 block = 768 个线程，占用率跌到 37.5%——GPU 一半以上的算力在空转。

**Warp Divergence（warp 分歧）。** 同一 warp 的 32 个线程必须执行同一条指令（SIMT 模型）。如果代价函数里有 `if-else` 分支（比如"如果碰到障碍加 1000 否则加 0"），同一 warp 内有的线程进 if、有的进 else，硬件就要**串行地执行两条分支**——性能降为原来的 1/2。

在"每 thread 一条 rollout"的映射下，同一 warp 的 32 个线程算的是 32 条**不同的** rollout。如果部分 rollout 碰了障碍、部分没碰，同一 warp 就出现分歧。这是"每 thread 一条"方案的固有缺陷——Ch2 §2.2 提到的"每 warp 一条"方案，让同一 warp 的线程协同处理**同一条** rollout，从根本上避免了这种分歧。

**如何量化 warp divergence？** 用 `ncu --metrics smsp__inst_executed_pipe_branch_divergent` 可以测量分歧分支的指令数。如果这个数字接近 0，说明没有分歧；如果它占总指令的 5% 以上，就值得考虑改映射策略或简化分支。

### ⚠️ 常见陷阱

**编程陷阱：rollout kernel 缺少越界保护 `if (k >= K) return;`。**
- **错误描述**：kernel 的第一行直接开始读状态，没有检查线程 index 是否越界。
- **现象/后果**：当 K 不是 blockDim 的整数倍时（几乎总是这样），多出来的线程读写越界内存。CUDA **不会报 segfault**——结果是无声的数据损坏（silent data corruption），表现为每次运行结果不同、偶尔出现 NaN、偶尔看起来正常。这是最难调试的 CUDA bug 之一。
- **根本原因**：CUDA 的内存保护模型比 CPU 宽松——设备端的越界读写在很多 GPU 上不会触发硬件错误，而是读到垃圾值或写坏相邻的分配。
- **正确做法**：kernel 第一行必须有 `if (k >= K) return;`，这是 CUDA 编程的**铁律**。用 `compute-sanitizer`（CUDA 自带的内存检查工具）在开发期检测越界。

**编程陷阱：忘记 `__syncthreads()` 导致 shared memory 竞态。**
- **错误描述**：用 shared memory 缓存 `u_mean` 后，没有在协作加载和使用之间加 `__syncthreads()`。
- **现象/后果**：部分线程在其他线程还没加载完 `u_mean` 时就开始读取 shared memory——读到的是旧值（0 或垃圾），导致控制序列错误。更隐蔽的是，这个 bug 是**非确定性的**：同一份代码运行 10 次可能 8 次正确 2 次错误（取决于 warp 调度的时序），极难复现。
- **根本原因**：CUDA 的 warp 调度不保证顺序——同一 block 内的不同 warp 可能在任意时刻执行任意指令，`__syncthreads()` 是唯一能保证"所有线程都到达这一点"的同步原语。
- **正确做法**：每次协作加载 shared memory 后都要 `__syncthreads()`；循环内如果有 shared memory 的写后读也要加。宁可多加（性能损失极小）也不要少加（正确性无保证）。

**性能陷阱：noise 数组的非合并访存（non-coalesced access）。**
- **错误描述**：noise 数组按 `[K, T, CD]` 布局，kernel 中 warp 内相邻线程 k, k+1 读取的地址间距为 `T * CD`，无法合并为一次内存事务。
- **现象/后果**：global memory 事务数高出预期 4-8 倍（用 ncu 的 `l1tex__t_sectors_pipe_lsu_mem_global_op_ld` 指标可观测），kernel 时间被内存带宽瓶颈限制。
- **根本原因**：GPU 的 global memory 以 128 字节的 cache line 为单位传输；warp 内 32 个线程如果读取的地址连续（合并），一次事务传 128 字节即可服务 32 个 float（32 * 4 = 128）；如果地址不连续，每个线程的读取可能落在不同的 cache line 上，需要多次事务。
- **正确做法**：把 noise 数组的布局从 `[K, T, CD]` 改为 `[T, K, CD]` 或在 kernel 内先把 noise 转置到 shared memory。这是一个"值不值得做"的工程权衡——先 profile 确认内存带宽是瓶颈后再优化。

### 练习

1. **（实现题）** 在上面的 rollout kernel 中，用 `nvcc --ptxas-options=-v` 编译 CartPole 和 Quadrotor 两个版本，分别记录寄存器使用量。计算在 RTX 3080（每 SM 65536 寄存器、最大 2048 线程）上、block size = 256 时的理论占用率（occupancy）。如果 Quadrotor 版的占用率低于 50%，尝试用 `__launch_bounds__(256, min_blocks)` 提示编译器限制寄存器使用。

2. **（分析题）** 把 CartPole 的代价函数改成含分支的版本：`if (|theta| > pi/2) cost += 1000; else cost += theta^2;`。用 `ncu` 测量 warp divergence 指标，与无分支版本对比。估算：当 K=2048 时，大约有多少条 rollout 会"碰到" |theta| > pi/2 的分支？这个比例如何影响 warp divergence 的程度？

3. **（优化题）** 把 noise 数组的布局从 `[K, T, CD]` 改为 `[T, K, CD]`，修改 kernel 中对应的索引计算。用 nsys 对比优化前后的 kernel 执行时间和 global memory 吞吐量。在 K=2048, T=64, CD=4 的配置下，预期加速比是多少？实测是多少？解释差异。

---

## §10.4 MPPI 主循环与权重计算 ⭐⭐

### 动机：把 kernel 和算法逻辑串成完整控制器

§10.3 写好了 rollout kernel——给它初始状态和噪声，它输出 K 个代价。但这只是 Ch2 §2.1 完整伪代码的**第 1 步**。一个能工作的 MPPI 控制器还需要：第 2 步权重计算（$\rho$ 减法 + 归一化）、第 3 步加权更新控制序列、第 4 步可选的 SGF 平滑、第 5 步执行 $u_0$ + warm-start shift。本节把这五步串成一个完整的 `MPPIController` 类。

回顾 Ch2 §2.1 的完整伪代码（此处重述其核心结构）：

```text
每个控制周期：
  1. GPU 并行 rollout → 得到 costs[K]
  2. ρ = min(costs); w_k = exp(-(costs[k] - ρ) / λ); η = Σ w_k
  3. u_t += Σ_k (w_k/η) * ε_k(t)
  4. （可选）U ← SGFilter(U)
  5. 执行 u₀；U ← shift_left(U)
```

### 如果不做 warm-start 会怎样

Ch2 §2.1 已经用 2-D 导航的例子定量证明了这一点：warm-start 版 47 步到达目标，冷启动版 120 步预算内**未到达**。在真实的 CartPole swing-up 任务中，差距更大——冷启动版在 50 Hz 的控制频率下根本无法在 20 ms 内从零收敛到一个像样的控制序列，摆杆永远摆不起来。

### 实现：MPPIController 类

**为什么把权重计算放在 CPU 上。** 权重计算涉及三个操作：求 min（$\rho$）、逐元素指数、求 sum（$\eta$）。这三个操作的输入大小是 K（几千个 float），计算量极小（微秒级），远不值得单独写一个 GPU kernel——kernel 的启动开销本身就有几微秒，和计算量相当。所以工程上通常把 K 个代价从 GPU 拷贝回 CPU（一次 `cudaMemcpy`，K=4096 时约 16 KB，传输时间 ~5 微秒），在 CPU 上算权重。

不过，如果控制更新（第 3 步的加权求和）也在 CPU 上做，那 K 条 rollout 的噪声也得拷贝回 CPU——噪声数组 $K \times T \times m$ 在 K=4096, T=64, m=4 的配置下约 4 MB，拷贝开销 ~200 微秒，这就不可忽视了。更高效的做法是把第 3 步也放在 GPU 上——写一个 `update_kernel` 在设备端完成加权求和——但这增加了实现复杂度。本章先用 CPU 版跑通全流程，§10.6 会量化瓶颈并判断是否值得优化。

```cpp
// include/mppi/mppi_controller.hpp
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <curand.h>
#include "rollout_kernel.cuh"

template<class Dynamics, class CostFunc>
class MPPIController {
public:
  static constexpr int SD = Dynamics::state_dim;
  static constexpr int CD = Dynamics::ctrl_dim;

  MPPIController(int K, int T, float dt, float lambda, float sigma)
    : K_(K), T_(T), dt_(dt), lambda_(lambda), sigma_(sigma),
      u_mean_(T * CD, 0.0f),
      costs_h_(K),
      weights_(K),
      noise_h_(K * T * CD)
  {
    // ---- GPU 内存一次性分配（不在控制循环里 malloc！）----
    CUDA_CHECK(cudaMalloc(&x0_d_,     SD * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&u_mean_d_, T * CD * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&noise_d_,  K * T * CD * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&costs_d_,  K * sizeof(float)));

    // ---- cuRAND 初始化（设备端噪声生成器）----
    curandCreateGenerator(&gen_, CURAND_RNG_PSEUDO_DEFAULT);
    curandSetPseudoRandomGeneratorSeed(gen_, 42);
  }

  ~MPPIController() {
    cudaFree(x0_d_); cudaFree(u_mean_d_);
    cudaFree(noise_d_); cudaFree(costs_d_);
    curandDestroyGenerator(gen_);
  }

  // ---- 核心：一次 MPPI 更新 ----
  void update(const Eigen::Matrix<float, SD, 1>& x0) {
    // ==== Step 1：生成噪声 + 启动 rollout ====
    // 1a. 把初始状态和当前控制序列传到 GPU
    CUDA_CHECK(cudaMemcpy(x0_d_, x0.data(),
               SD * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(u_mean_d_, u_mean_.data(),
               T_ * CD * sizeof(float), cudaMemcpyHostToDevice));

    // 1b. 在 GPU 上生成 K*T*CD 个标准正态噪声，然后乘以 sigma
    curandGenerateNormal(gen_, noise_d_, K_ * T_ * CD, 0.0f, sigma_);

    // 1c. 启动 rollout kernel
    int block_size = 256;
    int num_blocks = (K_ + block_size - 1) / block_size;
    rollout_kernel<Dynamics, CostFunc><<<num_blocks, block_size>>>(
        x0_d_, u_mean_d_, noise_d_, costs_d_, K_, T_, dt_);

    // 1d. 同步等待 kernel 完成
    CUDA_CHECK(cudaDeviceSynchronize());

    // ==== Step 2：把代价拷贝回 CPU，计算权重 ====
    CUDA_CHECK(cudaMemcpy(costs_h_.data(), costs_d_,
               K_ * sizeof(float), cudaMemcpyDeviceToHost));

    // ρ 减法（Ch2 §2.1 的数值稳定化——不做就 NaN！）
    float rho = *std::min_element(costs_h_.begin(), costs_h_.end());
    float eta = 0.0f;
    for (int k = 0; k < K_; ++k) {
      weights_[k] = expf(-(costs_h_[k] - rho) / lambda_);
      eta += weights_[k];
    }
    // 归一化
    for (int k = 0; k < K_; ++k) weights_[k] /= eta;

    // ==== Step 3：加权更新控制序列 ====
    // 把噪声拷贝回 CPU（这是性能瓶颈！见正文分析）
    CUDA_CHECK(cudaMemcpy(noise_h_.data(), noise_d_,
               K_ * T_ * CD * sizeof(float), cudaMemcpyDeviceToHost));

    for (int t = 0; t < T_; ++t) {
      for (int d = 0; d < CD; ++d) {
        float du = 0.0f;
        for (int k = 0; k < K_; ++k) {
          du += weights_[k]
              * noise_h_[k * T_ * CD + t * CD + d];
        }
        u_mean_[t * CD + d] += du;
      }
    }

    // ==== Step 4（可选）：Savitzky-Golay 平滑 ====
    // 对每个控制维度独立做 SGF
    for (int d = 0; d < CD; ++d) {
      savgol_smooth(d);
    }

    // ==== Step 5：warm-start shift ====
    // 左移一位，末尾补零
    std::rotate(u_mean_.begin(),
                u_mean_.begin() + CD,
                u_mean_.end());
    std::fill(u_mean_.end() - CD, u_mean_.end(), 0.0f);
  }

  // ---- 获取当前要执行的控制 ----
  Eigen::Matrix<float, CD, 1> get_control() const {
    Eigen::Matrix<float, CD, 1> u;
    // 注意：update() 最后做了 shift，所以 u_mean_[0:CD] 已经是
    // 原始序列的 u_1（shift 之前的 u_0 已经被移走了）
    // 实际使用中应在 shift 之前取 u_0——见练习 1
    for (int d = 0; d < CD; ++d) u(d) = u_mean_[d];
    return u;
  }

private:
  int K_, T_;
  float dt_, lambda_, sigma_;

  // CPU 端
  std::vector<float> u_mean_;     // 控制序列 [T * CD]
  std::vector<float> costs_h_;    // rollout 代价（CPU 副本）[K]
  std::vector<float> weights_;    // 权重 [K]
  std::vector<float> noise_h_;    // 噪声（CPU 副本）[K * T * CD]

  // GPU 端
  float* x0_d_;
  float* u_mean_d_;
  float* noise_d_;
  float* costs_d_;

  curandGenerator_t gen_;

  // ---- Savitzky-Golay 平滑（窗口=5，2 阶多项式）----
  void savgol_smooth(int ctrl_dim_idx) {
    // SG 滤波的卷积系数（窗口 5，2 阶多项式）
    // 由 (A^T A)^{-1} A^T 投影矩阵的中心行给出
    static const float coef[5] = {
      -3.0f/35, 12.0f/35, 17.0f/35, 12.0f/35, -3.0f/35
    };
    std::vector<float> buf(T_);
    for (int t = 0; t < T_; ++t) {
      buf[t] = u_mean_[t * CD + ctrl_dim_idx];
    }
    // 端点不动，中间做卷积
    for (int t = 2; t < T_ - 2; ++t) {
      float val = 0.0f;
      for (int j = -2; j <= 2; ++j) {
        val += coef[j + 2] * buf[t + j];
      }
      u_mean_[t * CD + ctrl_dim_idx] = val;
    }
  }
};
```

**代码后的深入分析。**

**性能瓶颈：噪声回传。** 上面代码中最大的性能问题是 Step 3 的 `cudaMemcpy` 把 K * T * CD 个 float 从 GPU 拷贝回 CPU。以 K=4096, T=64, CD=4 为例，这是 $4096 \times 64 \times 4 \times 4 = 4194304$ 字节 = 4 MB。通过 PCIe 3.0 传输 4 MB 约需 250 微秒——这比 rollout kernel 本身的执行时间（~1.5 ms）还短，但占了总周期时间的一个不小比例。

更好的做法是把 Step 3 的加权更新也放在 GPU 上：写一个 `update_control_kernel`，在设备端直接用 `weights_d_` 和 `noise_d_` 完成加权求和，只把更新后的 `u_mean`（T * CD 个 float，通常 < 1 KB）拷贝回来。这样每周期的 CPU-GPU 传输量从 ~4 MB 降到 ~1 KB，减少约 4000 倍。但这是一个"先跑通再优化"的典型场景——先用简单但慢的 CPU 版确认算法正确，再写 GPU 版优化性能。

**关于 cuRAND 的使用。** `curandGenerateNormal` 在 GPU 上直接生成高斯随机数，不需要 CPU 端生成再拷贝——这正是 Ch2 §2.2 强调的"噪声应在设备上生成"的实现。cuRAND 使用 XORWOW 伪随机数生成器，每个线程的状态由 seed + offset 确定，K * T * CD 个随机数的生成在 GPU 上只需约 10-50 微秒（远快于 CPU 端 `std::normal_distribution`）。

**warm-start 的 `std::rotate` 实现。** `std::rotate` 是 C++ 标准库的原地旋转函数，它把 `[first, middle)` 和 `[middle, last)` 两段交换位置——效果就是"左移 CD 个元素、末尾 CD 个元素挪到前面"。紧接着的 `std::fill` 把末尾 CD 个元素置零（中性填充）。这两步合起来就是 Ch2 §2.1 的 warm-start shift。

> **反事实：如果每周期从零冷启动。** 把 `std::rotate` 和 `std::fill` 注释掉，改成 `std::fill(u_mean_.begin(), u_mean_.end(), 0.0f)`——每周期从零控制序列开始优化。在 CartPole swing-up 任务中，K=2048、T=30 的配置下，warm-start 版约 3 秒内摆起来；冷启动版在 10 秒内未能成功。差距的根因就是 Ch2 §2.1 的本质洞察：warm-start 改善的是提议分布的**位置**，比增加 K（改善**数量**）重要得多。

### 数值稳定性深挖：$\rho$ 减法、log-sum-exp 与下溢/上溢

前置自测第 1 题问"不减 $\rho$ 会怎样"——这一节把它从"会 NaN"的口头结论，落实成**浮点位级别的机理**，并给出比朴素 $\rho$ 减法更稳健的 log-sum-exp 实现。

**朴素权重计算为什么会炸。** 权重公式 $w_k = \exp(-S_k/\lambda)$。假设代价尺度 $S_k\approx 1200$、温度 $\lambda=1$，则 $w_k=e^{-1200}$。IEEE 754 单精度 `float` 的最小正规数约 $1.18\times 10^{-38}$，对应指数下限约 $e^{-87}$——$e^{-1200}$ 远在其下，**直接下溢为 0**。于是所有 $w_k=0$，归一化时 $\eta=\sum w_k=0$，最后 $w_k/\eta=0/0=$ NaN。这就是章首前置自测第 1 题答案的浮点机理：不是"数太小算不准"，而是"小到被截断成精确的 0，再除以 0 得 NaN"。

**$\rho$ 减法为什么救场。** 取 $\rho=\min_k S_k$，改算 $w_k=\exp(-(S_k-\rho)/\lambda)$。因为 $S_k-\rho\geq 0$ 且最小值恰为 0，所以指数 $-(S_k-\rho)/\lambda\in(-\infty, 0]$，$w_k\in(0,1]$——**代价最低的那条 rollout 权重恰好等于 1**，其余在 $(0,1)$ 间。$\eta=\sum w_k\geq 1$ 永远非零，除法安全。注意 $\rho$ 减法**不改变归一化后的权重**：$\frac{e^{-(S_k-\rho)/\lambda}}{\sum_j e^{-(S_j-\rho)/\lambda}}=\frac{e^{-S_k/\lambda}e^{\rho/\lambda}}{e^{\rho/\lambda}\sum_j e^{-S_j/\lambda}}=\frac{e^{-S_k/\lambda}}{\sum_j e^{-S_j/\lambda}}$——分子分母的公共因子 $e^{\rho/\lambda}$ 约掉了。这正是它的精妙：**只挪动了未归一化权重的尺度，不动最终概率**。

> **本质洞察：$\rho$ 减法就是 softmax 数值稳定化的"减最大值"技巧，只是符号反了一下。** 机器学习里算 softmax $\frac{e^{z_k}}{\sum e^{z_j}}$，标准做法是减去 $\max_j z_j$ 防上溢。MPPI 里 $z_k=-S_k/\lambda$，"减最大 $z$"等价于"减最小 $S$"——也就是减 $\rho$。理解了这个对应，你就明白 MPPI 的权重计算本质上是一个 **temperature 为 $\lambda$ 的 softmax over 负代价**——它和深度学习里见过无数次的那个 softmax 是同一个东西。这也解释了 Ch5 把 MPPI 和 CEM 统一在"采样-加权"框架下的合理性：CEM 用硬阈值（精英/非精英 0-1 权重），MPPI 用软的 softmax 权重，二者是同一谱系的两端。

**更稳健的写法：log-sum-exp。** $\rho$ 减法解决了下溢，但还有一个隐患——如果你需要的不是权重本身，而是归一化常数 $\eta$ 的**对数**（例如做模型证据估计、或自适应 $\lambda$ 时要比较不同 $\lambda$ 下的 $\log\eta$），直接 $\log(\sum e^{\cdot})$ 仍可能在求和阶段溢出。通用的数值稳定原语是 **log-sum-exp（LSE）**：

$$\operatorname{LSE}(a_1,\dots,a_K) = \log\sum_{k} e^{a_k} = a^* + \log\sum_k e^{a_k - a^*}, \quad a^* = \max_k a_k.$$

在 MPPI 里 $a_k=-S_k/\lambda$，$a^*=-\rho/\lambda$（因为 $\rho=\min S_k$ 对应最大的 $a_k$）。把 $a^*$ 提到对数外面，括号内每项 $e^{a_k-a^*}\in(0,1]$，求和不溢出。下面给出 LSE 版的权重计算，它在数学上与 $\rho$ 减法等价，但额外提供了稳定的 $\log\eta$：

```cpp
// ---- log-sum-exp 版权重计算（比朴素 ρ 减法更通用）----
// 输入：costs_h_[K]，温度 lambda_
// 输出：weights_[K]（归一化），以及 log_eta（归一化常数的对数）
float compute_weights_lse(float lambda) {
  // a_k = -S_k / lambda；a* = max_k a_k = -rho / lambda
  float a_max = -std::numeric_limits<float>::infinity();
  for (int k = 0; k < K_; ++k) {
    float a = -costs_h_[k] / lambda;
    a_max = std::max(a_max, a);          // 等价于找 rho = min S_k
  }
  // sum_exp = Σ exp(a_k - a*)；每项 ∈ (0,1]，不会上溢
  double sum_exp = 0.0;                  // 用 double 累加，进一步防误差累积
  for (int k = 0; k < K_; ++k) {
    float a = -costs_h_[k] / lambda;
    weights_[k] = expf(a - a_max);       // 未归一化权重 ∈ (0,1]
    sum_exp += weights_[k];
  }
  float log_eta = a_max + logf((float)sum_exp);   // log Σ exp(a_k)，稳定
  // 归一化：w_k /= Σ
  float inv = 1.0f / (float)sum_exp;
  for (int k = 0; k < K_; ++k) weights_[k] *= inv;
  return log_eta;
}
```

**为什么 `sum_exp` 用 `double` 累加。** 这是一个容易忽略但真实存在的精度问题：$K$ 可达 8192，把 8192 个 `float` 累加，后期的小项加到已经很大的部分和上会因尾数位不够而被"吃掉"（catastrophic cancellation 的近亲——加法吸收）。用 `double` 中间累加，再转回 `float`，几乎零成本地消除这一误差。这也是为什么生产级 MPPI 库（如 MPPI-Generic）的归约普遍用更宽的累加类型。

### 把加权更新搬上 GPU：reduction kernel 与 ESS

§10.4 开头分析过：CPU 版的最大瓶颈是 Step 3 把 $K\times T\times m\approx 4$ MB 噪声回传 CPU。现在兑现"写一个 `update_control_kernel` 在设备端完成加权求和"的承诺——这样每周期只回传 $T\times m<1$ KB 的更新后控制序列，传输量降约 4000 倍。

**设计要点：两阶段归约。** GPU 上的加权求和 $\Delta u_t=\sum_k \hat w_k\,\epsilon_k(t)$ 是一个**按 $(t,d)$ 分组、沿 $k$ 求和**的归约。最直接的并行方案：让每个 $(t,d)$ 对由一个 block 负责，block 内 256 个线程协作沿 $k$ 累加，用 shared memory 做 block 内归约。

```cpp
// ---- GPU 端加权更新 kernel ----
// grid = T * CD 个 block（每个 block 算一个 (t,d) 的 Δu），block = 256 线程
// 输出 delta_u[T*CD]，控制器拿回后做 u_mean += delta_u
__global__ void update_control_kernel(
    const float* __restrict__ weights,  // [K] 归一化权重（已在 GPU 上）
    const float* __restrict__ noise,    // [K * T * CD]（rollout 时已在 GPU）
    float* __restrict__ delta_u,        // [T * CD] 输出
    int K, int T, int CD)
{
  int td = blockIdx.x;                  // 第几个 (t,d) 对 ∈ [0, T*CD)
  int t  = td / CD;
  int d  = td % CD;
  int tid = threadIdx.x;

  // ---- 每个线程沿 k 方向做跨步累加（grid-stride over K）----
  float partial = 0.0f;
  for (int k = tid; k < K; k += blockDim.x) {
    partial += weights[k] * noise[k * T * CD + t * CD + d];
  }

  // ---- block 内 shared memory 归约 ----
  extern __shared__ float sdata[];
  sdata[tid] = partial;
  __syncthreads();
  // 树形归约：256 → 128 → ... → 1
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (tid < s) sdata[tid] += sdata[tid + s];
    __syncthreads();
  }
  // lane 0 写出本 (t,d) 的加权和
  if (tid == 0) delta_u[td] = sdata[0];
}
```

**代码后的解读。** 三处工程细节。其一，**grid-stride 累加**（`k += blockDim.x`）让 256 个线程覆盖任意大的 $K$——$K=8192$ 时每个线程累加 32 项。其二，**树形归约**把 256 个部分和在 $\log_2 256=8$ 步内合成一个，每步线程数减半。注意每步后的 `__syncthreads()` 不可省（§10.3 陷阱已警告：shared memory 写后读必须同步）。其三，**为什么不用一个 `atomicAdd` 直接累加到 `delta_u[td]`**——几千个线程对同一地址做原子加会严重串行化（争抢同一把锁），树形归约把争抢限制在 block 内的 shared memory，快得多。

**启动与回传。** 配合这个 kernel，控制器的 Step 3 改写为：

```cpp
// ==== Step 3（GPU 版）：设备端加权更新，只回传 delta_u ====
// 前提：weights_ 已拷到 weights_d_，noise_d_ 还在 GPU 上（rollout 时生成）
CUDA_CHECK(cudaMemcpy(weights_d_, weights_.data(),
           K_ * sizeof(float), cudaMemcpyHostToDevice));
int smem = 256 * sizeof(float);
update_control_kernel<<<T_ * CD, 256, smem>>>(
    weights_d_, noise_d_, delta_u_d_, K_, T_, CD);
CUDA_CHECK(cudaDeviceSynchronize());
// 只回传 T*CD 个 float（<1 KB），而非 4 MB 噪声
CUDA_CHECK(cudaMemcpy(delta_u_h_.data(), delta_u_d_,
           T_ * CD * sizeof(float), cudaMemcpyDeviceToHost));
for (int i = 0; i < T_ * CD; ++i) u_mean_[i] += delta_u_h_[i];
```

> **本质洞察：MPPI 的全部计算都"尴尬并行"，但归约是唯一需要线程协作的环节——它决定了你对 GPU 编程的理解深度。** rollout 是纯并行（线程互不通信），新手都能写对；但加权求和需要"把 K 个线程的结果合并成一个"，这里才真正考验对 shared memory、`__syncthreads`、原子操作的掌握。一个工程师会不会写 MPPI，看 rollout 看不出来，看他怎么写这个归约就知道了。这也是为什么本节把它单独拎出来讲——它是从"会调 API"到"懂 GPU"的分水岭。

**ESS（有效样本数）监控。** 权重算出来后，最重要的健康探针是 ESS（Effective Sample Size, 有效样本数）：

$$\text{ESS} = \frac{1}{\sum_{k=1}^{K} \hat w_k^2}, \qquad \text{ESS}\in[1, K].$$

它的含义：$K$ 个加权样本"相当于"多少个等权样本。两个极端——若某条 rollout 独占全部权重（$\hat w_1=1$，其余 0），$\text{ESS}=1$（提议分布只有一条样本起作用，等于没采样）；若权重完全均匀（$\hat w_k=1/K$），$\text{ESS}=K$（所有样本平等贡献，但这意味着代价没区分度、$\lambda$ 太大）。健康区间是 $\text{ESS}/K\in[5\%, 50\%]$（章末速查表的调参依据正源于此）。

```cpp
// ---- ESS 监控：在 compute_weights 后调用 ----
float compute_ess() const {
  double sum_sq = 0.0;
  for (int k = 0; k < K_; ++k) sum_sq += (double)weights_[k] * weights_[k];
  return (float)(1.0 / sum_sq);     // weights_ 已归一化，∑w=1
}
```

**ESS 怎么用：它是 $\lambda$ 的"活体探针"。** ESS 不只是事后诊断，更可驱动**自适应温度**。如果某周期 ESS 突然跌到 $K$ 的 1%（权重塌缩到极少数 rollout），说明 $\lambda$ 相对当前代价尺度太小——可临时调大 $\lambda$ 恢复权重多样性。反过来 ESS 贴近 $K$（权重太平），说明 $\lambda$ 太大、采样在"和稀泥"，可调小 $\lambda$ 加大区分度。一个简单的自适应规则：

```cpp
// ---- 基于 ESS 的自适应温度（每周期可选执行）----
void adapt_lambda() {
  float ess = compute_ess();
  float ratio = ess / K_;
  if      (ratio < 0.05f) lambda_ *= 1.2f;   // 权重太尖 → 升温（更软）
  else if (ratio > 0.50f) lambda_ *= 0.9f;   // 权重太平 → 降温（更尖）
  // 夹紧到合理范围，防止失控漂移
  lambda_ = std::clamp(lambda_, lambda_min_, lambda_max_);
}
```

**为什么不直接固定一个"完美"的 $\lambda$。** 因为代价尺度在一次任务里会变。CartPole swing-up 初期摆杆下垂，代价 $\sim 2T$（角度项满额）；摆起来后代价 $\sim 0$。同一个 $\lambda$ 在初期可能让权重塌缩、在末期可能让权重太平——固定 $\lambda$ 是"刻舟求剑"。ESS 自适应让温度随代价尺度自动伸缩，这正是 Ch3 提到的"让提议分布更聪明"在温度维度上的体现。

### 协方差自适应：让噪声幅度随收敛自动收缩

固定噪声标准差 $\sigma$（章首速查表建议"执行器范围 10%-50%"）也有同样的"刻舟求剑"问题：swing-up 初期需要**大** $\sigma$ 充分探索（找到摆起来的剧烈动作），收敛后需要**小** $\sigma$ 精细微调（大 $\sigma$ 会让控制在最优点附近抖动）。协方差自适应（covariance adaptation）让 $\sigma$ 随收敛程度自动收缩。

**与 CEM 的联系与区别（R6A 双重解读）。** Ch5 讲过 CEM 用精英样本的**样本协方差**直接拟合下一轮的采样分布——这是"硬"自适应。MPPI 没有精英集，但可以用**加权样本协方差**做"软"自适应：用归一化权重 $\hat w_k$ 对噪声 $\epsilon_k$ 加权，估计当前控制序列周围"好样本"的散布，据此更新 $\sigma$。

$$\hat\sigma_d^2 \;=\; \sum_{k=1}^{K} \hat w_k\,\big(\epsilon_k(d)\big)^2 \quad\text{(每个控制维度 } d \text{ 独立估计)}.$$

```cpp
// ---- 加权协方差自适应（对角、逐控制维度）----
// 用归一化权重对噪声平方加权，估计"高权重样本"的散布
void adapt_sigma() {
  std::vector<double> var(CD, 0.0);
  for (int k = 0; k < K_; ++k) {
    float w = weights_[k];
    for (int t = 0; t < T_; ++t)
      for (int d = 0; d < CD; ++d) {
        float e = noise_h_[k * T_ * CD + t * CD + d];
        var[d] += (double)w * e * e;     // 加权平方，∑_t 上摊到全时域
      }
  }
  for (int d = 0; d < CD; ++d) {
    float new_sigma = sqrtf((float)(var[d] / T_));   // 时域平均
    // 指数滑动平均，避免单周期噪声估计的剧烈跳变
    sigma_vec_[d] = (1.0f - beta_) * sigma_vec_[d] + beta_ * new_sigma;
    // 下限保护：σ 不能塌到 0（否则彻底停止探索，陷在局部最优）
    sigma_vec_[d] = std::max(sigma_vec_[d], sigma_floor_);
  }
}
```

**代码后的两个关键决策。** 其一，**指数滑动平均（EMA, $\beta\approx 0.1$）而非直接替换**——单周期的加权协方差估计噪声很大（毕竟只有 $K$ 个样本），直接拿它当新 $\sigma$ 会让噪声幅度剧烈跳变、控制不稳。EMA 让 $\sigma$ 平滑演化。其二，**$\sigma$ 下限保护（`sigma_floor_`）至关重要**——这是 R6B 反事实的经典场景：

> **反事实：如果不设 $\sigma$ 下限会怎样。** 协方差自适应一旦收敛，加权协方差会持续缩小，$\sigma\to 0$。一旦 $\sigma=0$，所有 rollout 的噪声 $\epsilon_k=0$，K 条轨迹**退化成同一条**——提议分布坍缩成一个点，MPPI 彻底失去探索能力。此后哪怕环境突变（如 CartPole 被外力推一下）需要重新探索，$\sigma=0$ 也再也"长不回来"，控制器卡死在过时的解上。下限保护 `sigma_floor_` 保证永远留一点探索余量——这与强化学习里熵正则（entropy bonus）防止策略过早收敛是同一种智慧：**永远不要让探索归零**。

**自适应的开关哲学。** 和 SGF 一样，ESS 自适应温度和协方差自适应都是**可选增强**，不是必需。本章的基础实现先用固定 $\lambda$、固定 $\sigma$ 跑通——确认算法正确后，再按需开启自适应。过早引入自适应会让调试复杂化（一个 bug 你分不清是 rollout 错、权重错、还是自适应规则把 $\lambda/\sigma$ 带跑偏了）。这再次呼应 §10.1 的"先跑通再优化"。

### Savitzky-Golay 平滑集成的设计决策

**为什么 SGF 放在 Step 4 而非 Step 3 之后？** 逻辑上，SGF 应该在加权更新**完成后**、warm-start shift **之前**——因为你要平滑的是"已经加权更新过的当前控制序列"，而不是"shift 之后给下一周期用的序列"。如果放在 shift 之后，你平滑的对象包含了末尾那个新填充的零——这个零不是优化出来的控制值，把它纳入平滑会拉低末端几步的控制幅度。

**SGF 的超参选择。** 代码里用的是窗口 5、2 阶多项式——这是一个保守的选择。回顾 Ch2 §2.1 的分析：窗口越宽平滑越狠但越容易抹平真实特征；阶数越高越保形但去噪越弱。$\text{window}=5, \text{poly}=2$ 在 $T=30$ 的时域下约覆盖 17% 的时步，平滑强度适中。如果你的 $T$ 更大（如 T=100），可以适当增大窗口到 7 或 9。

### ⚠️ 常见陷阱

**编程陷阱：在控制循环里重复调用 `cudaMalloc` / `cudaFree`。**
- **错误描述**：每个控制周期（50 Hz = 每 20 ms 一次）都 `cudaMalloc` 分配噪声数组、用完后 `cudaFree` 释放。
- **现象/后果**：`cudaMalloc` 的延迟约 100-500 微秒，`cudaFree` 约 50-200 微秒，每周期额外开销 150-700 微秒——在 20 ms 的预算里占 1-3.5%。更糟的是，频繁的分配/释放会导致 GPU 内存碎片化（fragmentation），一段时间后可能出现 `cudaErrorMemoryAllocation` 即使总空闲内存充足。
- **根本原因**：`cudaMalloc` 不是 `malloc`——它要走 GPU 驱动的内存管理器，涉及页表更新和 TLB 刷新，代价远高于 CPU 端的 `malloc`。
- **正确做法**：在构造函数中一次性分配所有 GPU 缓冲区，在析构函数中统一释放。控制循环里只做 `cudaMemcpy` 和 kernel 启动，不做内存管理。

**编程陷阱：忘了 `cudaDeviceSynchronize()`，测出虚假的"超快"时间。**
- **错误描述**：用 CPU 端的 `clock()` 或 `chrono` 计时，在 kernel 启动后立刻停表，没有等 kernel 完成。
- **现象/后果**：测出的"kernel 时间"是 kernel **启动**的时间（几微秒），不是 kernel **执行**的时间（几毫秒）。于是得出"GPU 比 CPU 快 1000 倍"的虚假结论。
- **根本原因**：CUDA kernel 启动是**异步的**——`<<<...>>>` 调用立刻返回 CPU，kernel 在 GPU 上异步执行。CPU 端的计时在 kernel 还没跑完时就停了。
- **正确做法**：在 kernel 启动后、停表前调用 `cudaDeviceSynchronize()`（或使用 CUDA event 计时）。或者直接用 `nsys` 做 timeline profiling，它能精确测量 kernel 的设备端执行时间。

**概念陷阱：把 SGF 当作 MPPI 的必要组成部分而非可选项。**
- **错误描述**：认为"没有 SGF 的 MPPI 是不完整的"，在所有场景下都开启 SGF。
- **现象/后果**：在仿真环境或低通执行器上，SGF 是多余的——它白白破坏了 MPPI 的信息论最优性（Ch2 §2.1 的本质洞察），可能让控制稍微变迟钝（尖锐特征被平滑掉了）。
- **根本原因**：SGF 是"理论最优让位于执行器现实"的工程妥协——只有当执行器真的吃不消高频抖动时才需要。Ch2 区分了两种平滑：采样层面（不破坏最优性）和事后平滑（破坏最优性），SGF 属于后者。
- **正确做法**：先不开 SGF 跑一遍，观察控制输出的抖动程度；如果抖动伤到执行器再开。在仿真开发阶段完全不需要 SGF。

### 练习

1. **（Bug 修复题）** 上面代码中的 `get_control()` 函数有一个微妙的 bug：它在 `update()` 里先做加权更新、再做 warm-start shift，然后 `get_control()` 返回的是 shift 后的 `u_mean_[0:CD]`——这已经是原始序列的 $u_1$ 而不是 $u_0$。设计两种修复方案：(A) 在 shift 之前保存 $u_0$；(B) 把 shift 移到下一次 `update()` 的开头。分析两种方案的优缺点。

2. **（性能分析题）** 用 `nsys profile ./mini_mppi` 生成 timeline，找到每个控制周期中以下步骤的耗时：kernel 执行、costs 回传（D→H）、noise 回传（D→H）、CPU 权重计算、CPU 加权更新。画出一个饼图，识别最大的瓶颈。然后设计一个优化方案：如果把加权更新也放在 GPU 上（`update_control_kernel`），预期能减少多少百分比的周期时间？

3. **（调参实验题）** 在 CartPole swing-up 任务中，固定 K=2048、T=30，分别用 $\lambda = 0.1, 1, 10, 100$ 跑 10 次，记录：(a) 每次运行的 ESS = $1/\sum_k \hat{w}_k^2$；(b) 首次摆起（$|\theta| < 0.1$）的时间步数；(c) 控制信号 $u_0$ 的均值和方差。用数据回答：$\lambda$ 太小或太大各自导致什么后果？ESS 落在 K 的多少百分比范围内时性能最好？

---

## §10.5 MuJoCo 集成 ⭐⭐

### 动机：为什么要接上物理引擎

§10.2 的手写动力学模型简单直接，但有两个根本局限。第一，**接触动力学**——CartPole 和 Quadrotor 都是"非接触"的系统（没有碰撞、摩擦），而真实机器人（腿足、操作）的核心就是接触。手写一个准确的接触动力学方程极其困难——要处理碰撞检测、摩擦锥、弹性变形等，这是一个专门的研究领域。物理引擎如 MuJoCo（Multi-Joint dynamics with Contact）已经把这一切封装好了。

第二，**模型复杂度**——Go2 四足机器人有 12 个驱动关节、4 条腿各 3 段连杆、机体 6 自由度，总共 36 维状态（Ch8 腿足全身 MPPI 的基础），手写其动力学方程不仅费力而且容易出错。MuJoCo 通过 URDF/XML 模型描述文件自动生成完整的动力学，用户只需要提供机器人的几何参数和关节约束。

> **类比：手写 SQL 查询 vs 用 ORM。** 手写动力学像手写 SQL——对简单查询（CartPole）足够，但表一多、关系一复杂就不可维护。MuJoCo 像 ORM——它把复杂的"物理引擎底层"封装成简洁的 API，你只需要定义"模型长什么样"，它自动生成"模型怎么动"。

### MuJoCo C API 基础

**为什么用 C API 而非 Python API。** MuJoCo 提供两套接口：C API（`mujoco.h`）和 Python API（`mujoco` package）。Python API 方便原型开发，但 MPPI 的 rollout 循环要求每秒调用动力学几十万次（K=2048, T=64 = 131072 次/周期，50 Hz = 655 万次/秒），Python 的函数调用开销（~1 微秒/次）在这个频率下会成为瓶颈。C API 的函数调用开销约 10 纳秒，差 100 倍。

MuJoCo C API 的核心是两个数据结构：

```cpp
#include <mujoco/mujoco.h>

// mjModel：模型描述（只读），包含几何、质量、关节约束等
// 从 XML 文件加载，全程序共享一份
mjModel* m = mj_loadXML("quadrotor.xml", NULL, error, 1000);

// mjData：仿真状态（可读写），包含位置、速度、控制、力等
// 每个 rollout 线程需要自己的一份（不是线程安全的！）
mjData* d = mj_makeData(m);

// 一步仿真：读取 d->ctrl（控制输入），更新 d->qpos/qvel（状态）
mj_step(m, d);    // 调用这一行就完成一步积分
```

### mjData 的线程安全问题

**为什么这是一个必须讲清的陷阱。** `mjData` 包含了仿真的全部可变状态——位置、速度、控制、内部缓冲区（接触点列表、约束雅可比矩阵等）。**`mjData` 不是线程安全的**——两个线程同时对同一个 `mjData` 调用 `mj_step` 会导致数据竞争（data race），结果不可预测。

这对 MPPI 来说是一个结构性问题：MPPI 需要同时跑 K 条 rollout，每条 rollout 在不同的控制序列下积分 T 步——如果用 MuJoCo 做动力学，就需要**每条 rollout 有自己的 `mjData` 副本**。

```cpp
// ---- 正确：每条 rollout 一份 mjData ----
std::vector<mjData*> d_pool(K);
for (int k = 0; k < K; ++k) {
    d_pool[k] = mj_makeData(m);    // 分配 K 份独立的 mjData
}

// rollout：每个线程 k 用 d_pool[k]
// （注意：这是 CPU 并行，不是 GPU——MuJoCo 的 mj_step 是 CPU 函数）
#pragma omp parallel for
for (int k = 0; k < K; ++k) {
    mjData* d = d_pool[k];
    // 设置初始状态
    mj_copyData(d, m, d_init);    // 从初始状态拷贝
    for (int t = 0; t < T; ++t) {
        // 设置控制
        for (int j = 0; j < m->nu; ++j)
            d->ctrl[j] = u_mean[t * m->nu + j] + noise[k * T * m->nu + t * m->nu + j];
        mj_step(m, d);    // 一步仿真
        costs[k] += compute_cost(d);
    }
}
```

**如果不做每线程一份 mjData 会怎样。** 直接在多线程中共享同一个 `mjData*`，调用 `mj_step`。后果：

| 现象 | 原因 |
|------|------|
| 仿真结果每次不同（非确定性） | 两个线程同时修改 `d->qpos`，写入顺序不确定 |
| 偶尔崩溃（segfault） | `mj_step` 内部动态分配的接触点列表被两个线程同时修改，指针失效 |
| 控制行为诡异（"鬼手"现象） | 线程 A 设置的 `d->ctrl` 被线程 B 覆盖，线程 A 积分时用了线程 B 的控制 |

这些 bug 极难调试——它们是非确定性的（取决于线程调度时序），运行 10 次可能只有 1 次出问题，且 segfault 的 call stack 指向 MuJoCo 内部深处，让你误以为是 MuJoCo 的 bug。

**mjModel 为什么可以共享。** `mjModel` 是只读的——它描述的是模型的**结构**（几何形状、质量参数、关节约束），在仿真过程中不变。多个 `mjData` 可以安全地共享同一个 `mjModel*`，因为 `mj_step(m, d)` 只读取 `m`、只写入 `d`。

### batch rollout 模式

**为什么 MuJoCo rollout 通常在 CPU 上做。** MuJoCo 的 `mj_step` 是 CPU 函数——它没有 GPU 版本。MuJoCo 3.0+ 引入了 MJX（MuJoCo XLA），可以在 GPU 上跑批量仿真，但那是 Python/JAX 接口，不是 C API。在 C/C++ 中使用 MuJoCo，rollout 必须在 CPU 上执行。

这意味着 MuJoCo 集成的 MPPI 架构是**混合式**的：

```text
CPU                                    GPU
 │                                      │
 ├── MuJoCo batch rollout (K条, OpenMP) │
 │   └── costs[K]                       │
 │                                      │
 ├── 权重计算 (rho, exp, sum)           │
 ├── 加权更新 (u_mean += ...)           │
 ├── SGF 平滑                           │
 └── warm-start shift                   │
```

对比 §10.3 的纯 CUDA 架构（rollout 在 GPU 上），MuJoCo 版的 rollout 时间通常慢 10-50 倍——但获得了接触动力学的准确性。工程上的取舍是：对**非接触**系统（CartPole、Quadrotor），用 CUDA kernel + 手写动力学获得最快速度；对**接触**系统（腿足、操作），用 MuJoCo 获得准确性，代价是速度，通过 OpenMP 多线程部分弥补。

```cpp
// ---- MuJoCo batch rollout（OpenMP 并行）----
void mujoco_batch_rollout(
    const mjModel* m,
    const mjData* d_init,          // 初始状态
    const float* u_mean,           // [T * nu]
    const float* noise,            // [K * T * nu]
    float* costs,                  // [K]
    int K, int T,
    std::vector<mjData*>& d_pool)  // 预分配的 mjData 池
{
  #pragma omp parallel for schedule(dynamic)
  for (int k = 0; k < K; ++k) {
    mjData* d = d_pool[k];
    mj_copyData(d, m, d_init);    // 重置到初始状态

    float cost = 0.0f;
    for (int t = 0; t < T; ++t) {
      // 设置微扰控制
      int nu = m->nu;
      for (int j = 0; j < nu; ++j) {
        d->ctrl[j] = u_mean[t * nu + j]
                   + noise[k * T * nu + t * nu + j];
      }
      mj_step(m, d);              // MuJoCo 一步
      cost += mujoco_running_cost(m, d);
    }
    cost += mujoco_terminal_cost(m, d);
    costs[k] = cost;
  }
}
```

**代码后的解读。** `schedule(dynamic)` 告诉 OpenMP 用动态调度——因为不同 rollout 的计算时间可能不同（有的碰到复杂接触、有的没碰），动态调度让先完成的线程去拿新任务，而非所有线程等最慢的那个（静态调度的缺点）。

### ⚠️ 常见陷阱

**编程陷阱：多线程共享同一个 `mjData`，导致非确定性崩溃。**
- **错误描述**：为了省内存，K 条 rollout 共享一个 `mjData*`，在 OpenMP 循环里直接用。
- **现象/后果**：仿真结果每次不同（data race on `d->qpos`/`d->qvel`），偶尔 segfault（接触列表被并发修改），极难复现（取决于线程调度时序）。
- **根本原因**：`mjData` 包含大量可变状态（位置、速度、接触缓冲区），`mj_step` 会修改它们。两个线程同时修改同一个 `mjData` 是未定义行为。
- **正确做法**：每个线程预分配一份独立的 `mjData`（`mj_makeData` 返回新对象），仿真结束后释放。K=4096 时 4096 个 `mjData` 的内存开销约 40-400 MB（取决于模型复杂度），通常可接受。

**编程陷阱：忘记在每条 rollout 前 `mj_copyData` 重置状态。**
- **错误描述**：只在第一条 rollout 前设置初始状态，后续 rollout 继承上一条的终态。
- **现象/后果**：第 k 条 rollout 的初始状态不是 $x_0$ 而是第 $k-1$ 条的终态——K 条 rollout 不再独立，违反了 MPPI 的核心假设，加权更新的结果毫无意义。
- **根本原因**：`mj_step` 是有状态的——它就地修改 `mjData`，不会自动重置。
- **正确做法**：每条 rollout 开始前调用 `mj_copyData(d_pool[k], m, d_init)` 把状态重置到初始条件。

**性能陷阱：MuJoCo rollout 的 K 条全串行执行。**
- **错误描述**：没有用 OpenMP 或 std::thread，K=2048 条 rollout 在单线程上串行跑。
- **现象/后果**：单次 `mj_step` 约 10-50 微秒（取决于模型复杂度），$2048 \times 64 = 131072$ 次调用 = 1.3-6.5 秒/周期——远超 20 ms 的实时预算。
- **根本原因**：没有利用 CPU 的多核并行。`mj_step` 本身是单线程的，但 K 条 rollout 之间是独立的，可以分配到多核上并行。
- **正确做法**：用 OpenMP `#pragma omp parallel for` 并行化 K 条 rollout。8 核 CPU 约 8 倍加速，$6.5 / 8 \approx 0.8$ 秒/周期——仍然比 GPU 慢很多，但对于接触系统这是目前可行的方案。MuJoCo MJX + JAX 可以在 GPU 上做 batch rollout，但需要 Python 生态。

### 练习

1. **（集成题）** 在 Mini-MPPI 项目中集成 MuJoCo：(a) 用 MuJoCo 自带的 CartPole XML 模型替换手写动力学；(b) 写一个 CPU batch rollout 函数；(c) 比较 MuJoCo 版和手写版的仿真轨迹是否一致（给相同的初始状态和控制序列，比较 T 步后的终态差异）。差异来自何处？

2. **（性能分析题）** 在 K=1024、T=30 的配置下，分别测量：(a) CUDA kernel + 手写动力学的 rollout 时间；(b) MuJoCo + OpenMP（8 线程）的 rollout 时间。计算加速比。在你的 CPU/GPU 硬件上，K 需要多大时 MuJoCo 版才会无法满足 50 Hz 实时？

3. **（设计题）** 设计一个"混合"架构：用手写的简化动力学在 CUDA kernel 中做快速 rollout（获得近似代价），然后用 MuJoCo 对 top-10% 的 rollout 做精确 re-simulation（验证近似代价是否准确）。这种"粗筛+精验"的策略能否在保持速度的同时提高模型准确性？分析其优缺点。

---

## §10.6 性能基准测试 ⭐⭐⭐

### 动机：为什么必须自己做基准测试

Ch2 §2.2 引用的性能数字（GPU 约 50 倍加速）来自 MPPI-Generic 论文——它是在特定硬件（Jetson AGX Xavier）、特定模型（AutoRally）上测的。你的 Mini-MPPI 用不同的 GPU、不同的动力学、不同的 kernel 实现，性能可以差 2-10 倍。**不自己测，就不知道自己的实现到底行不行**。

基准测试不是"跑完记个数"——它是一种**系统化的实验方法论**：控制变量、扫参数、画曲线、定位瓶颈。做好了，它告诉你"K 取多少就够了""GPU 比 CPU 快了多少""瓶颈在内存还是在计算"；做不好，它只告诉你一个没有上下文的数字（"1.5 ms"），你不知道它是好是坏、能否改进。

### K/T 扫参方法论

**为什么要扫 K 和 T。** K（采样数）和 T（时域长度）是 MPPI 最重要的两个"资源旋钮"——它们直接决定了计算量（$\propto K \times T$）和控制质量。但它们的边际收益是递减的：K 从 256 加到 2048，控制质量大幅提升；从 2048 加到 16384，提升微乎其微但计算量翻了 8 倍。扫参的目的是找到**拐点**——在那之后加资源不值得了。

**实验设计。** 绘制以下四组曲线：

```text
实验 1：K vs 计算时间（固定 T=30）
  K = [256, 512, 1024, 2048, 4096, 8192]
  分别测量 CPU（单线程）和 GPU 的单次 update() 时间
  每个 K 跑 100 次取中位数（避免首次 JIT 编译和缓存冷启动的影响）

实验 2：K vs 最终代价（固定 T=30）
  K = [256, 512, 1024, 2048, 4096]
  每个 K 做 50 次独立实验（不同随机种子）
  记录每次实验的最终代价，画均值 ± 标准差

实验 3：T vs 最终代价（固定 K=2048）
  T = [10, 20, 30, 50, 100]
  同上 50 次独立实验

实验 4：λ 敏感性（固定 K=2048, T=30）
  λ = [0.01, 0.1, 1, 10, 100]
  同上 50 次独立实验
  额外记录每次的 ESS
```

**为什么跑 100 次取中位数而非单次。** GPU kernel 的执行时间有波动（$\pm$10-20%），来源包括：GPU 动态频率调节（boost clock vs base clock）、L2 缓存状态、GPU 上其他进程的干扰。中位数比均值更鲁棒——它不受偶尔出现的异常值（如系统中断导致的延迟）影响。

### GPU vs CPU 对比

**实验设计。** 用同一份动力学（CartPole 和 Quadrotor 各一组），在 CPU（单线程 Eigen）和 GPU（CUDA kernel）上分别跑 rollout，固定 T=30，扫 K。

预期结果模式：

```text
K=256:   CPU ~2 ms,   GPU ~0.5 ms   → 加速比 4×（GPU 没吃饱）
K=1024:  CPU ~8 ms,   GPU ~0.7 ms   → 加速比 11×（GPU 开始吃饱）
K=4096:  CPU ~32 ms,  GPU ~1.5 ms   → 加速比 21×（GPU 接近饱和）
K=8192:  CPU ~64 ms,  GPU ~3.0 ms   → 加速比 21×（线性缩放区）
```

**为什么小 K 时加速比低。** GPU 有固定的启动开销——kernel 启动（~5 微秒）、cuRAND 初始化、内存拷贝。当 K 小时，这些固定开销占总时间的比例大，计算量不够"淹没"开销，加速比自然低。这也回答了"K 最小能取多少"的问题：如果你的 GPU 在 K=256 时只快 4 倍，而 CPU 版已经够快（2 ms < 20 ms 预算），那 GPU 的意义主要不是"加速"，而是"释放 CPU 算力给感知和估计"。

### Profiling 工具：nsys 和 ncu

**nsys（Nsight Systems）：时间线分析。** nsys 生成整个应用的时间线——CPU 线程在干什么、GPU kernel 何时启动何时结束、内存拷贝何时发生。它回答的是**"时间花在了哪一步"**的宏观问题。

```bash
# 生成 timeline 报告
nsys profile -o report ./mini_mppi
# 用 nsys-ui 可视化（或导出文本报告）
nsys stats report.nsys-rep
```

从 nsys timeline 中你应该能看到：

```text
|--- cudaMemcpy H→D (x0, u_mean) ---|
|--- curandGenerateNormal ------------|
|--- rollout_kernel ==================|   ← 主要计算
|--- cudaMemcpy D→H (costs) ---------|
|--- CPU 权重计算 ---------|
|--- cudaMemcpy D→H (noise) =========|   ← 可能的瓶颈
|--- CPU 加权更新 =========|              ← 可能的瓶颈
```

**ncu（Nsight Compute）：kernel 微架构分析。** ncu 深入单个 kernel 内部——寄存器使用、shared memory 使用、占用率、内存带宽利用率、指令吞吐率。它回答的是**"这个 kernel 为什么慢/能否更快"**的微观问题。

```bash
# 分析单个 kernel 的详细指标
ncu --set full -k rollout_kernel ./mini_mppi
```

关键指标和解读：

| 指标 | 含义 | 健康范围 | 不健康意味着 |
|------|------|---------|-------------|
| Occupancy（占用率） | 实际活跃 warp / SM 最大支持 warp | > 50% | 寄存器压力大，需减少每线程寄存器 |
| L1/L2 Hit Rate（缓存命中率） | 内存请求在缓存中命中的比例 | > 80% | 访存模式不合并，需要优化数据布局 |
| Memory Throughput | 实际内存吞吐 / 峰值内存带宽 | > 60% | 如果计算密集型 kernel 的内存吞吐高，说明瓶颈在内存，需要减少访存 |
| Warp Branch Divergence | 分歧分支的 warp 比例 | < 5% | 代价函数的 if 分支导致 warp 内分歧 |

### ⚠️ 常见陷阱

**实验陷阱：计时时忘了 `cudaDeviceSynchronize()`，得到虚假的快时间。**
- **错误描述**：用 `auto start = chrono::high_resolution_clock::now();` 在 kernel 启动后立刻 `auto end = ...`，没有等 kernel 完成。
- **现象/后果**：测出的"kernel 时间"是 ~5 微秒（kernel 启动的异步返回时间），实际执行时间可能是 ~1.5 毫秒——差 300 倍。发表基准测试论文时用了这个数字，reviewer 一看就知道你没做 synchronize。
- **根本原因**：CUDA kernel 启动是异步的——CPU 调用 `<<<...>>>` 后立刻返回，不等 GPU 完成。这是 CUDA 的**设计特性**（允许 CPU 和 GPU 重叠执行），但在计时时必须手动同步。
- **正确做法**：在 kernel 启动后、停表前加 `cudaDeviceSynchronize()`。或者用 CUDA event（`cudaEventRecord` + `cudaEventElapsedTime`），它在 GPU 端精确计时，不受 CPU 影响。

**实验陷阱：只跑一次就报数字，没有统计量。**
- **错误描述**：K=2048 跑了一次，得到 1.3 ms，就在报告里写"GPU 耗时 1.3 ms"。
- **现象/后果**：下一次跑可能是 1.8 ms（GPU boost clock 降频了），再下一次可能是 1.1 ms（L2 缓存热了）。单次数字没有统计意义。
- **根本原因**：GPU 执行时间受多种因素影响——频率调节、缓存状态、系统中断、其他 GPU 进程。
- **正确做法**：每个配置跑 100 次，报告中位数和 25th-75th 百分位数。前 10 次是"热身"（warming up），不计入统计。

**概念陷阱：只看加速比不看绝对时间。**
- **错误描述**："GPU 比 CPU 快 50 倍"——但实际上 CPU 版只需 5 ms（已经满足 20 ms 的实时预算），GPU 版 0.1 ms。50 倍加速比很好看，但在这个场景下 CPU 已经够用了。
- **现象/后果**：为了追求加速比而上 GPU，增加了系统复杂度（CUDA 依赖、GPU 驱动兼容、功耗），但实际收益为零。
- **根本原因**：加速比是相对指标，绝对时间才是决定"能否实时"的标准。50 Hz 控制要求每周期 < 20 ms，只要满足这个约束，用 CPU 就够了。
- **正确做法**：先看绝对时间是否满足实时约束；不满足时才上 GPU。GPU 的真正价值不只是"快"，还有"释放 CPU 算力给感知和规划"——这个价值在绝对时间表里看不出来，但在系统级架构中很重要。

### 练习

1. **（基准测试题）** 对 CartPole 任务做完整的 K 扫参实验（K=256/512/1024/2048/4096），分别测量 GPU 和 CPU 版的 `update()` 时间（各 100 次取中位数），画出 K vs 时间的双对数图。找出 GPU 的加速比开始饱和的 K 值。

2. **（Profiling 题）** 用 `nsys profile` 对 K=2048, T=30 的一次控制周期做 timeline 分析。识别以下阶段的耗时占比：(a) kernel 执行；(b) H→D 数据传输；(c) D→H 数据传输；(d) CPU 计算。画出饼图。如果把 D→H 的 noise 传输消除（通过 GPU 端加权更新），总耗时能减少多少百分比？

3. **（分析题）** 用 `ncu` 对 Quadrotor 的 rollout kernel（K=2048, T=64）做微架构分析。报告：(a) 占用率 %;  (b) 每线程寄存器数；(c) L1/L2 缓存命中率；(d) global memory 吞吐量占峰值带宽的比例。根据这些指标判断：你的 kernel 是**计算瓶颈（compute bound）**还是**内存瓶颈（memory bound）**？相应地，应该从哪个方向优化？

---

## §10.7 调试与故障排查 ⭐⭐

### 动机：CUDA + 数值计算 = 调试地狱

CUDA 编程的调试难度远高于纯 CPU 编程——原因有三。第一，**GPU 上的错误是静默的**：内存越界不报 segfault、竞态条件不报 data race，结果就是"程序跑完了但输出是错的"，且每次可能不同。第二，**printf 调试在 GPU 上不可靠**：CUDA 的 `printf` 要走一个有限大小的缓冲区（默认 1 MB），K=4096 个线程同时 printf 会溢出缓冲区，丢失输出。第三，**数值问题和 CUDA 问题交织**：一个 NaN 可能来自动力学方程的数值不稳定，也可能来自 kernel 的内存越界读到了垃圾值——症状一样，根因完全不同。

### CUDA 常见错误与诊断

**错误类型一：kernel 启动后 cudaGetLastError 报错。**

```cpp
// 启动 kernel 后检查错误
rollout_kernel<...><<<blocks, threads>>>(args...);
cudaError_t err = cudaGetLastError();
if (err != cudaSuccess) {
    printf("Kernel launch failed: %s\n", cudaGetErrorString(err));
}
```

常见的启动错误包括：
- `cudaErrorInvalidConfiguration`：block size 超过 GPU 限制（通常 1024）或 shared memory 超过 SM 容量（通常 48 KB）
- `cudaErrorLaunchOutOfResources`：每线程寄存器需求太大，一个 block 都放不下

**错误类型二：kernel 执行中出错（异步错误）。**

```cpp
// 同步后检查执行期间的错误
cudaDeviceSynchronize();
err = cudaGetLastError();
// 常见：cudaErrorIllegalAddress（设备端野指针）
//      cudaErrorAssert（设备端 assert 失败）
```

**利器：compute-sanitizer。** CUDA 自带的内存/竞态检查工具，类似 CPU 上的 Valgrind/ASan。

```bash
# 检查内存越界
compute-sanitizer --tool memcheck ./mini_mppi

# 检查竞态条件
compute-sanitizer --tool racecheck ./mini_mppi

# 检查未初始化内存
compute-sanitizer --tool initcheck ./mini_mppi
```

`compute-sanitizer` 会把程序慢 10-50 倍，但能精确定位"哪个线程在哪一行访问了非法地址"——这是排查"静默内存错误"的唯一可靠手段。开发期间应该**常规性地**在 sanitizer 下跑一遍。

### 数值发散诊断

**症状：控制输出突然变成 NaN 或 Inf。** 这是 MPPI 最常见的运行时故障，可能来自三个源头：

| 源头 | 检查方法 | 典型根因 |
|------|---------|---------|
| 权重计算 | 检查 `rho` 和 `eta` 是否 finite | 没做 $\rho$ 减法 → $e^{-S/\lambda}$ 下溢为 0 → $0/0$ = NaN |
| 动力学积分 | 在 kernel 中对 `x` 加 `isfinite` 检查 | dt 太大导致 Euler 积分发散；或除以零（如 $\text{denom}=0$ 当 $\sin\theta=0$ 且 $m_\text{pole}=0$） |
| 代价函数 | 检查 `total_cost` 是否 finite | 代价中有 $\log(0)$、$\sqrt{-1}$、或障碍罚值导致 Inf |

**诊断流程图。**

```text
控制输出 NaN？
  ├── 检查 weights：eta == 0？
  │     ├── 是 → 权重全下溢 → 检查是否做了 ρ 减法
  │     └── 否 → 权重正常
  ├── 检查 costs[K]：有 NaN/Inf？
  │     ├── 是 → rollout 内部出了 NaN → 进 kernel 调试
  │     │     ├── 动力学发散？→ 减小 dt / 检查方程
  │     │     └── 代价函数 NaN？→ 检查 log/sqrt 的定义域
  │     └── 否 → costs 正常
  └── 检查 noise：有 NaN？
        ├── 是 → cuRAND 初始化有问题 → 检查 gen_ 是否已创建
        └── 否 → 噪声正常 → bug 在加权更新步骤
```

### 可视化调试

**为什么可视化是 MPPI 调试的第一工具。** MPPI 的中间变量是高维的（K 条轨迹、T 步时域、多维状态），用 printf 看数字几乎不可能发现模式。但画出来一目了然：

```python
# python/visualize.py —— MPPI 调试可视化
import numpy as np
import matplotlib.pyplot as plt

def plot_mppi_debug(data_file):
    """读取 MPPI 一次 update() 的中间数据，画四张调试图。"""
    data = np.load(data_file)
    costs   = data['costs']       # [K]
    weights = data['weights']     # [K]
    trajs   = data['trajs']       # [K, T, state_dim]
    u_mean  = data['u_mean']      # [T, ctrl_dim]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 图 1：代价分布直方图
    ax = axes[0, 0]
    ax.hist(costs, bins=50, density=True)
    ax.axvline(costs.min(), color='r', label=f'rho={costs.min():.1f}')
    ax.set_xlabel('Total Cost')
    ax.set_title('Cost Distribution')
    ax.legend()

    # 图 2：权重分布（CDF）—— 检查 ESS
    ax = axes[0, 1]
    sorted_w = np.sort(weights)[::-1]
    cumsum_w = np.cumsum(sorted_w)
    ax.plot(np.arange(len(sorted_w)), cumsum_w)
    ax.axhline(0.5, color='r', linestyle='--', label='50% weight')
    ess = 1.0 / np.sum(weights**2)
    ax.set_title(f'Cumulative Weight (ESS={ess:.0f}/{len(weights)})')
    ax.set_xlabel('Sorted Rollout Index')
    ax.legend()

    # 图 3：K 条轨迹在 2D 空间的投影
    ax = axes[1, 0]
    for k in range(min(100, len(trajs))):
        alpha = max(0.01, weights[k] * 10)
        ax.plot(trajs[k, :, 0], trajs[k, :, 1],
                alpha=min(alpha, 1.0), color='blue', linewidth=0.5)
    ax.set_title('Sampled Trajectories (brightness = weight)')
    ax.set_xlabel('x'); ax.set_ylabel('y')

    # 图 4：控制序列 u_mean
    ax = axes[1, 1]
    for d in range(u_mean.shape[1]):
        ax.plot(u_mean[:, d], label=f'u_{d}')
    ax.set_title('Control Sequence (u_mean)')
    ax.set_xlabel('Time Step')
    ax.legend()

    plt.tight_layout()
    plt.savefig('mppi_debug.png', dpi=150)
    plt.show()
```

**每张图能看出什么问题：**

| 图 | 健康样子 | 不健康意味着 |
|----|---------|------------|
| 代价分布 | 右偏分布（大多代价适中，少数很高） | 所有代价都很高 → 提议分布太差；代价全一样 → 代价函数没区分度 |
| 权重 CDF | 10-20 条 rollout 占 50% 权重 | 1 条占 99% → $\lambda$ 太小；全均匀 → $\lambda$ 太大 |
| 轨迹投影 | 高权重轨迹聚集在目标方向 | 轨迹四散 → $\Sigma$ 太大；轨迹扎堆在错误方向 → warm-start 指向了过时的解 |
| 控制序列 | 平滑、有物理意义的形状 | 高频锯齿 → 没做 SGF；首尾剧烈跳变 → warm-start 末位填充有问题 |

### ⚠️ 常见陷阱

**调试陷阱：在 GPU kernel 里用 `printf` 调试，输出丢失或乱序。**
- **错误描述**：在 kernel 里加 `printf("k=%d cost=%f\n", k, total_cost);`，K=4096 时只看到几百行输出（而非 4096 行），且顺序杂乱。
- **现象/后果**：以为某些线程没执行，误判为 kernel 的线程映射有 bug——实际上所有线程都执行了，只是 printf 的输出缓冲区溢出了。
- **根本原因**：CUDA 的设备端 printf 使用一个固定大小的环形缓冲区（默认 1 MB），4096 条 ~40 字节的输出 = ~160 KB，通常没问题；但如果你在循环内部也 printf（T 步 $\times$ K 条 = 12 万行），160 KB 缓冲区就远远不够了。输出不保证顺序（不同 warp 的 printf 是非确定性交错的）。
- **正确做法**：(1) 只对少数线程 printf：`if (k < 5) printf(...);`；(2) 增大 printf 缓冲：`cudaDeviceSetLimit(cudaLimitPrintfFifoSize, 10 * 1024 * 1024);`（10 MB）；(3) 更好的方式是把中间数据写到一个 `debug_buffer_d_` 数组里，kernel 结束后整块拷贝回 CPU 分析。

**调试陷阱：NaN 的传染性——一个 NaN 污染整条计算链。**
- **错误描述**：控制输出是 NaN，但不知道 NaN 最先在哪里产生。
- **现象/后果**：NaN 有传染性——`NaN + 1 = NaN`、`NaN * 0 = NaN`、`min(NaN, 1) = NaN`（IEEE 754 语义），一旦产生就会从 rollout 内部一路传到权重、传到控制更新、传到输出，最终**所有**输出都是 NaN。倒着追踪 NaN 的源头需要在每一步加 `isfinite` 检查。
- **根本原因**：IEEE 754 浮点标准规定 NaN 参与任何运算的结果仍是 NaN——这是一个设计特性（让 NaN 像"毒药"一样传播，确保不会被静默忽略），但在调试时意味着你必须找到**第一个**产生 NaN 的运算。
- **正确做法**：在 kernel 中加分段检查——(1) 加载初始状态后检查 `isfinite(x)`；(2) 每步 `step` 后检查 `isfinite(x)`；(3) 每步代价后检查 `isfinite(total_cost)`。第一个 `!isfinite` 的点就是 NaN 的源头。开发期间这些检查可以用宏控制（`#ifdef DEBUG`），release 时编译掉。

### 练习

1. **（调试题）** 故意在 CartPole 的 `step` 函数中引入一个 bug：把 `denom = m_cart + m_pole * sin_t * sin_t` 改成 `denom = m_pole * sin_t * sin_t`（去掉 `m_cart`）。运行 Mini-MPPI，观察：(a) 多少步后出现 NaN？(b) NaN 最先在哪个变量产生？(c) 用上面的诊断流程图定位根因。

2. **（工具题）** 用 `compute-sanitizer --tool memcheck` 运行你的 Mini-MPPI。如果没有内存错误，故意去掉 kernel 的越界保护 `if (k >= K) return;`，再跑一遍 sanitizer。观察 sanitizer 报告的错误类型和位置。

3. **（可视化题）** 用上面的 `visualize.py` 为 CartPole swing-up 的一次 MPPI 更新画四张调试图。然后把 $\lambda$ 改成一个极小值（如 0.001），再画一遍。对比两组图，说明 $\lambda$ 过小如何反映在权重 CDF 和轨迹投影上。

---

## 本章小结

本章是前 9 章知识的"落地考试"——把散落在理论、算法、GPU、变体各章中的知识组装成一个从零开始、能编译、能运行、能验证的完整 MPPI 控制器。回顾七节的主线：

| 节 | 核心动作 | 关键设计决策 | 与前置章节的联系 |
|----|---------|------------|----------------|
| §10.1 | 搭项目骨架 | 模板化设计，按编译单元分文件 | Ch2 §2.3 MPPI-Generic 的四维解耦 |
| §10.2 | 写动力学模型 | `__host__ __device__` 双标注，固定大小 Eigen | Ch2 §2.2 寄存器与内存层级 |
| §10.3 | 写 rollout kernel | 线程映射、shared memory、register pressure | Ch2 §2.2 CUDA 并行化策略 |
| §10.4 | 写 MPPI 主循环 | $\rho$ 减法、warm-start、SGF 集成 | Ch2 §2.1 完整伪代码的全部细节 |
| §10.5 | 接 MuJoCo 引擎 | mjData 线程安全、batch rollout | Ch8 腿足全身 MPPI 的动力学基础 |
| §10.6 | 做性能基准测试 | K/T 扫参、nsys/ncu profiling | Ch2 §2.2 的基准数字需要你自己验证 |
| §10.7 | 调试与排查 | compute-sanitizer、NaN 追踪、可视化 | 全书所有数值细节的实战检验 |

> **本质洞察**：从零搭建 Mini-MPPI 的过程揭示了一个规律——**算法论文里看似"显然"的步骤，在工程实现中各个都是坑**。论文说"采样 K 条 rollout"——但你要决定线程映射、内存布局、寄存器预算。论文说"计算权重"——但你要处理数值下溢、决定 CPU 还是 GPU 做归约。论文说"用 MuJoCo"——但你要解决 mjData 的线程安全。这些"论文不讲的东西"恰恰是工程师的核心价值：**把算法变成能跑在真实硬件上的系统**。

---

## 速查表

### MPPI 控制器实现清单

```text
□ 项目结构
  □ include/dynamics/ — 动力学模型（__host__ __device__ 模板）
  □ include/cost/     — 代价函数（running + terminal）
  □ include/mppi/     — rollout kernel + controller
  □ src/main.cpp      — 控制循环（纯 C++）
  □ src/mppi_kernel.cu — CUDA 编译单元
  □ CMakeLists.txt    — CXX + CUDA 混合编译

□ 动力学模型
  □ 固定大小 Eigen：Matrix<float, N, 1>，不用 MatrixXf
  □ sinf/cosf/sqrtf，不用 sin/cos/sqrt
  □ __host__ __device__ 双标注
  □ CPU 单元测试通过

□ Rollout Kernel
  □ if (k >= K) return; 越界保护
  □ __restrict__ 指针修饰
  □ 初始状态从 global 到 register（一次读取）
  □ 代价累加在寄存器（total_cost）
  □ 最终代价一次写入 global（costs[k]）
  □ nvcc --ptxas-options=-v 检查寄存器数
  □ compute-sanitizer 检查内存

□ MPPI 主循环
  □ 构造函数中一次性 cudaMalloc
  □ cuRAND 设备端噪声生成
  □ ρ = min(costs) 减法（必须！）
  □ 权重归一化 w_k /= eta
  □ 加权更新 u_t += Σ w_k * ε_k(t)
  □ SGF 平滑（可选）
  □ warm-start shift: rotate + fill
  □ cudaDeviceSynchronize() 在计时前

□ 性能基准
  □ K 扫参：256 / 512 / 1024 / 2048 / 4096
  □ 每配置 100 次，取中位数
  □ GPU vs CPU 对比
  □ nsys timeline + ncu kernel profiling
```

### 超参数速查

| 参数 | 推荐范围 | 调参依据 |
|------|---------|---------|
| K（采样数） | 512–4096 | 越大越好但递减；拐点看 K vs 代价曲线 |
| T（时域长度） | 20–100 | 够看到关键后果；配合终端代价 |
| $\lambda$（温度） | 取决于代价尺度 | 用 ESS 当探针；ESS/K 在 5%–50% |
| $\sigma$（噪声标准差） | 执行器范围的 10%–50% | 与执行器物理极限匹配 |
| dt（积分步长） | 0.01–0.05 s | Euler 稳定性限制；dt 太大会发散 |
| block_size | 128–256 | 256 是安全默认值；高寄存器压力时降到 128 |

---

## 故障排查手册

### 场景 1：控制输出全是 NaN

| 步骤 | 检查内容 | 工具 |
|------|---------|------|
| 1 | 权重 `eta` 是否为 0 | 在权重计算后加 `assert(eta > 0)` |
| 2 | `costs[K]` 中是否有 NaN/Inf | `assert(isfinite(costs[k]))` for all k |
| 3 | 是否做了 $\rho$ 减法 | 检查代码中 `rho = *min_element(...)` 是否存在 |
| 4 | 噪声数组是否有 NaN | `curandGenerateNormal` 后检查 |
| 5 | 动力学 `step` 是否数值稳定 | 单独在 CPU 上跑 T 步检查 |

**最常见根因**：没做 $\rho$ 减法（占 NaN 案例的 70%+）。

### 场景 2：kernel 启动失败（cudaErrorInvalidConfiguration）

| 步骤 | 检查内容 |
|------|---------|
| 1 | block_size 是否超过 1024 |
| 2 | shared memory 请求是否超过 SM 容量（48 KB） |
| 3 | 每线程寄存器数 $\times$ block_size 是否超过 SM 寄存器总数 |

### 场景 3：结果正确但 GPU 没有预期的加速

| 步骤 | 检查内容 | 工具 |
|------|---------|------|
| 1 | 计时是否包含了 `cudaDeviceSynchronize` | 代码审查 |
| 2 | 是否在循环里重复 `cudaMalloc`/`cudaFree` | nsys timeline |
| 3 | 噪声是否在 CPU 生成然后拷贝到 GPU | nsys 的 memcpy 统计 |
| 4 | `-arch` 是否与 GPU 架构匹配 | `nvidia-smi` vs CMakeLists |
| 5 | Eigen 是否用了动态矩阵（设备端堆分配） | ncu 检查 `__device_malloc` 调用 |

### 场景 4：CartPole 摆不起来

| 步骤 | 检查内容 |
|------|---------|
| 1 | 动力学方程是否正确（在 CPU 上单独测试） |
| 2 | 代价函数是否鼓励 swing-up（$\theta=0$ 代价最低？） |
| 3 | K 和 T 是否足够（K < 256 或 T < 15 时可能不够） |
| 4 | 是否做了 warm-start（不做则收敛极慢） |
| 5 | $\sigma$ 是否太小（探索不足，找不到 swing-up 的动作） |
| 6 | 控制是否被钳位到合理范围 |

### 场景 5：MuJoCo batch rollout 崩溃（segfault）

| 步骤 | 检查内容 |
|------|---------|
| 1 | 是否每线程一份 `mjData`（共享 = 竞态） |
| 2 | 每条 rollout 前是否 `mj_copyData` 重置状态 |
| 3 | `mj_loadXML` 是否成功（检查返回值和 error 字符串） |
| 4 | XML 模型中的执行器数量（`m->nu`）是否与 CONTROL_DIM 匹配 |

### 场景 6：编译/运行报架构或编译器版本错误

环境配置阶段最高频的两类拦路虎，集中排查（详细机理见章首"环境配置指南"）：

| 症状 | 可能原因 | 排查步骤 | 相关位置 |
|------|---------|---------|---------|
| `nvcc fatal: Unsupported gpu architecture 'compute_XX'` | `-arch` 写了 nvcc 不认识的计算能力 | 升级 CUDA Toolkit，或把 `-arch` 改成当前 Toolkit 支持的值 | 环境配置指南、§10.1 |
| 运行期 `no kernel image is available for execution on the device` | `-arch` 低于实际 GPU 或不匹配 | `nvidia-smi` 查 GPU 型号 → 查表得计算能力 → 填对 `-arch`；或用 `-gencode` 做 fat binary | §10.1 陷阱 |
| `#error -- unsupported GNU version! gcc versions later than N are not supported` | 系统 GCC 比 CUDA 支持的上限新 | 升级 CUDA，或装旧 GCC 并 `-DCMAKE_CUDA_HOST_COMPILER=g++-11` | 环境配置指南（GCC-CUDA 耦合） |
| `cannot find -lcurand` | CUDA 库路径未进 `LD_LIBRARY_PATH` | `export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH` | 环境配置指南安装步骤 |
| `constexpr __host__ function ... not allowed` in Eigen | 漏了 `--expt-relaxed-constexpr` | CMake 的 `CMAKE_CUDA_FLAGS` 加上该 flag | §10.1 CMake 配置 |

**最常见根因**：CUDA Toolkit 与系统 GCC 大版本错配（占环境类故障的 50%+），根源是误以为"装最新的就行"——而 nvcc 对 host 编译器有严格上限。

### 场景 7：自适应让 $\lambda$ 或 $\sigma$ 漂移失控

开启 §10.4 的 ESS 自适应温度 / 协方差自适应后出现的新故障：

| 症状 | 可能原因 | 排查步骤 |
|------|---------|---------|
| 控制越来越"软"、响应迟钝 | $\lambda$ 单调上升未夹紧 | 检查 `std::clamp(lambda_, lambda_min_, lambda_max_)` 是否生效 |
| K 条轨迹塌缩成一条、失去探索 | $\sigma\to 0$ 无下限保护 | 检查 `sigma_floor_` 是否设置且 `std::max` 生效 |
| $\sigma$ 单周期剧烈跳变 | 直接替换而非 EMA | 加权协方差用指数滑动平均（$\beta\approx 0.1$）平滑 |
| ESS 始终 $\approx K$ 或 $\approx 1$ | 自适应步长太小/代价尺度异常 | 先关自适应、固定 $\lambda$ 跑通，确认基础算法正确 |

---

## 延伸阅读

### 核心参考

| 资源 | 关联章节 | 价值 |
|------|---------|------|
| Williams 等, *MPPI: From Theory to Parallel Computation*, JGCD 2017 | §10.3, §10.6 | GPU rollout kernel 的工程设计原版——register/shared/global 三级内存策略的详细分析 |
| Vlahov 等, *MPPI-Generic: A CUDA Library for Stochastic Trajectory Optimization*, arXiv 2024 | §10.1, §10.3 | 模板化设计的工业参考——Dynamics/Cost/Sampler/Controller 四维解耦 |
| Williams 等, *Aggressive Driving with MPPI*, ICRA 2016 | §10.4 | 首次实车 GPU MPPI——warm-start、$\rho$ 减法等工程细节的起源 |
| MuJoCo 官方文档, https://mujoco.readthedocs.io | §10.5 | mjModel/mjData API 参考、线程安全说明、XML 模型格式 |
| NVIDIA Nsight Systems/Compute 文档 | §10.6 | nsys/ncu 的使用教程和指标解读 |

### 进阶方向

| 方向 | 关键资源 | 与本章的关系 |
|------|---------|-------------|
| GPU 端加权更新（消除 noise 回传瓶颈） | MPPI-Generic 源码中的 `update_control_kernel` | §10.4 的性能瓶颈解决方案 |
| MuJoCo MJX + JAX 做 GPU batch rollout | MuJoCo 3.0 文档、Google DeepMind brax | §10.5 的 GPU 替代方案 |
| 有色噪声 / 协方差自适应采样 | Ch3 六大变体 | §10.3 kernel 的采样策略扩展 |
| RL 学到的值函数作为终端代价 | Ch7 TD-MPC | §10.4 的终端代价设计 |
| 实车部署（ROS 2 集成、实时 Linux） | AutoRally 开源平台（Goldfain 2019） | 本章的下一步——从仿真到真机 |

### 评分标准（用于自评或课程评分）

| 维度 | 权重 | 评分要点 |
|------|------|---------|
| 代码质量 | 40% | 零堆分配热路径、CUDA 错误检查完备、模板化设计可复用、代码风格一致 |
| 任务性能 | 30% | CartPole swing-up 和 Quadrotor/Go2 hover 的成功率和最终代价 |
| 基准测试 | 20% | K/T 扫参曲线完整、GPU vs CPU 对比有统计量、nsys/ncu 分析有发现 |
| 文档 | 10% | README 含关键设计决策、如何复现、已知局限 |

---

## 版本信息速查

本章代码与命令在以下版本组合下验证通过。复现遇到问题时，先核对此表（版本错配是工程实践最高频的"伪 bug"来源）。

| 工具 / 库 | 验证版本 | 作用 | 备注 |
|----------|---------|------|------|
| NVIDIA 驱动 | 535.x | 决定 CUDA 运行时上限 | `nvidia-smi` 右上角的 CUDA 版本是驱动上限，非 Toolkit 版本 |
| CUDA Toolkit | 12.4 | nvcc 编译器、cuRAND | 须 $\leq$ 驱动上限；与 GCC 大版本耦合 |
| cuRAND | 随 CUDA 12.4 | 设备端噪声生成 | `CURAND_RNG_PSEUDO_DEFAULT`（XORWOW）；批量正态可换 Philox（$4\times32\times10$）提效 |
| GCC / G++ | 11.4 | nvcc 的 host 编译器 | CUDA 12.4 支持上限 GCC 13；本章用 11 求稳 |
| CMake | 3.27 | 构建系统 | $\geq 3.18$ 才有一等 CUDA 支持 |
| Eigen | 3.4.0 | 固定大小矩阵容器 | 精确锁定；3.3 在设备代码有坑 |
| MuJoCo | 3.1.6 | 接触动力学引擎（§10.5） | 3.0+ 起开源、C API 稳定 |
| Python | 3.10 | 可视化脚本（`visualize.py`） | 与 C++ 主程序解耦，版本宽松 |
| numpy / matplotlib | 1.26 / 3.8 | 调试可视化 | numpy 2.0 亦已验证 |

**一行环境自检命令**（跑通即说明工具链齐备）：

```bash
nvidia-smi | grep "CUDA Version" && nvcc --version | grep release && \
  pkg-config --modversion eigen3 && cmake --version | head -1
```

---

**预计学习时间**：精读 + 动手实现全流程约 30–40 小时（2 周）。
