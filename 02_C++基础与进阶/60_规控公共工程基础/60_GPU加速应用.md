# GPU 加速与 CUDA 在机器人中的应用

> 本章定位：让 GPU 从“看起来很快的硬件”变成“有清晰数据边界、同步边界和延迟预算的机器人计算模块”。
> 机器人中使用 GPU 的核心问题不是能不能写 kernel，而是数据在哪里、何时同步、是否批量足够大、错过 deadline 时如何降级。

## 前置自测

1. PCIe 传输为什么可能让小矩阵 GPU 加速变慢？
2. CUDA kernel launch 是同步还是异步？
3. `cudaDeviceSynchronize()` 放在控制循环中有什么风险？
4. pinned memory 和 pageable memory 的差异是什么？
5. TensorRT 相比 libtorch 的工程成本和性能收益分别是什么？

## 本章目标

学完本章后，你应该能判断一个机器人计算任务是否值得上 GPU。
你应该能写出带 stream、显式同步和错误检查的 CUDA 调用骨架。
你还应该能解释 CUDA Graphs、Thrust、TensorRT、cuRobo 在机器人系统中的位置，以及 GPU 与实时控制之间的边界。

---

## 6.1 何时该上 GPU ⭐

这一节解决的问题是：避免把 GPU 当作默认加速按钮。

### 动机：GPU 擅长吞吐，不天然擅长低延迟

GPU 有大量并行计算单元。
它适合大批量、规则、并行度高的任务。
但机器人控制常有小矩阵、强时序依赖和严格 deadline。
把 3x3 矩阵乘法搬到 GPU 通常会变慢，因为数据传输和 kernel 启动开销远大于计算本身。

GPU 可以类比高速铁路。
一次运很多人非常高效。
但如果只送一个人去隔壁楼，进站安检和候车时间会超过步行。

### 决策表

| 因素 | 偏向 CPU | 偏向 GPU |
|------|----------|----------|
| 数据规模 | 小矩阵、单查询 | 大图像、点云、批量轨迹 |
| 并行度 | 序列依赖强 | 每个元素或种子独立 |
| 数据位置 | 已在 CPU | 已在 GPU 感知或学习管线 |
| 延迟要求 | 单次 < 1 ms 且批量小 | 可批量或可流水化 |
| 开发成本 | 简单 C++ 可维护 | 有明确性能瓶颈 |

### 反面失败：把 WBC 小 QP 放到 GPU

一个 36 变量 WBC QP 在 CPU 上可能几十微秒到几百微秒。
如果每周期把 Hessian、约束、状态从 CPU 拷到 GPU，再启动 kernel，再把解拷回 CPU，很可能比 CPU 求解慢。
更糟的是同步点会把控制线程卡住。

除非你能把整个控制管线留在 GPU，或者一次批量求解大量 QP，否则小规模在线控制不应默认 GPU 化。

> 本质洞察：GPU 加速的第一问题不是“计算能不能并行”，而是“数据是否已经在 GPU 上，以及结果是否必须立刻回到 CPU”。
> 数据搬运和同步经常比浮点运算更贵。

### 适合 GPU 的机器人任务

| 任务 | GPU 价值 | 原因 |
|------|----------|------|
| 图像神经网络推理 | 高 | 卷积和矩阵乘大规模并行 |
| 点云体素化 | 中到高 | 点级并行，但有写冲突 |
| 批量 IK | 高 | 多种子独立求解 |
| 批量碰撞检测 | 高 | 多轨迹、多时间步并行 |
| 全局采样规划 | 中到高 | 多样本并行 |
| 小矩阵状态估计 | 低 | 数据小、同步贵 |
| 1 kHz WBC 小 QP | 低 | 延迟和同步主导 |

### 练习

1. 对一个 640x480 深度图滤波任务估算像素数、每像素计算量和传输成本，判断是否适合 GPU。
2. 对一个 12 关节 WBC 小矩阵计算估算 CPU 浮点量，解释为什么 GPU 不划算。
3. 找一个已经在 GPU 上产生的数据源，说明如何减少 CPU 往返。

---

## 6.2 CUDA 执行模型与同步边界 ⭐⭐

这一节解决的问题是：CUDA 调用到底什么时候完成，何处会阻塞 CPU。

### 异步 kernel launch

CUDA kernel launch 通常对 CPU 异步。
CPU 发起 kernel 后会继续向下执行。
真正需要结果时，才必须同步。

```cuda
#include <cuda_runtime.h>

__global__ void scaleKernel(float* data, int n, float scale) {
    // 每个线程处理一个元素，线程索引映射到数组下标。
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        data[i] *= scale;
    }
}
```

```cpp
#include <cuda_runtime.h>
#include <stdexcept>

void checkCuda(cudaError_t err) {
    if (err != cudaSuccess) {
        // 把 CUDA 错误码转成异常，避免调用者忽略失败。
        throw std::runtime_error(cudaGetErrorString(err));
    }
}

void launchScale(float* d_data, int n, float scale, cudaStream_t stream) {
    const int block = 256;
    const int grid = (n + block - 1) / block;
    scaleKernel<<<grid, block, 0, stream>>>(d_data, n, scale);
    // kernel launch 本身异步。
    // 这里检查的是 launch 配置错误，不代表 kernel 已经执行完成。
    checkCuda(cudaGetLastError());
}
```

### 同步点

| API | 同步范围 | 风险 |
|-----|----------|------|
| `cudaDeviceSynchronize()` | 整个设备 | 最粗，会等待所有 stream |
| `cudaStreamSynchronize(stream)` | 指定 stream | 较可控，但仍阻塞 CPU |
| `cudaEventSynchronize(event)` | 等待事件 | 可测时延，但仍阻塞 |
| `cudaMemcpy` 默认方向 | 常为同步 | 可能隐式等待 |
| `cudaMemcpyAsync` | 异步，需要 pinned memory 和 stream | 仍需后续同步 |

控制线程中最危险的是无意识同步。
例如读取 GPU 结果、默认 stream 拷贝或调用全设备同步，都可能造成不可预测等待。

### 用 event 查询而不是阻塞

```cpp
#include <cuda_runtime.h>

enum class GpuJobState {
    // 控制线程根据状态决定使用新结果、等待下一周期或进入降级路径。
    kRunning,
    kReady,
    kFailed
};

GpuJobState pollJob(cudaEvent_t done_event) {
    const cudaError_t status = cudaEventQuery(done_event);
    if (status == cudaSuccess) {
        return GpuJobState::kReady;
    }
    if (status == cudaErrorNotReady) {
        return GpuJobState::kRunning;
    }
    return GpuJobState::kFailed;
}
```

这个模式适合控制线程。
控制线程查询 GPU 作业是否完成。
如果未完成，使用上一帧结果或降级策略，而不是阻塞等待。

### stream 的生命周期

```cpp
#include <cuda_runtime.h>

class CudaStreamOwner {
public:
    CudaStreamOwner() {
        // 非阻塞 stream 避免默认 stream 带来的隐式同步。
        checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
        owned_ = true;
    }

    ~CudaStreamOwner() {
        // 用显式 owned_ 标志判断所有权，而不是 stream_ != nullptr：
        // cudaStream_t 的 0（默认流/cudaStreamLegacy）是合法句柄，
        // 用 nullptr 判断会把“持有默认流”误判为“未持有”，进而漏销毁或误销毁。
        if (owned_) {
            cudaStreamDestroy(stream_);
        }
    }

    cudaStream_t get() const { return stream_; }

private:
    cudaStream_t stream_{nullptr};
    bool owned_{false};
};
```

stream 是资源。
应在初始化阶段创建，在退出阶段销毁。
不要在高频循环中反复创建销毁 stream。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | kernel 后立即读 CPU 结果 | 隐式同步卡顿 | GPU 尚未完成 | event 查询或流水化 |
| 编程 | 控制循环 `cudaDeviceSynchronize` | 偶发大延迟 | 等待全设备所有任务 | 限定 stream 或降级 |
| 概念 | 以为 launch 完成等于计算完成 | 读到旧数据 | CUDA 异步执行 | 明确同步点 |
| 工程 | 循环里创建 stream | 延迟抖动 | 资源分配 | 初始化阶段创建 |

### 练习

1. 写一个 kernel 后不同步就读取结果的示例，解释为什么结果不可靠。
2. 把 `cudaDeviceSynchronize` 改成 event 查询，并设计未完成时的降级输出。
3. 对两个 stream 中的任务分别插入 event，测量是否并发。

---

## 6.3 内存传输、pinned memory 与数据布局 ⭐⭐⭐

这一节解决的问题是：为什么 GPU 计算快，整体 pipeline 仍可能慢。

### 传输成本

CPU 与独立 GPU 之间通常通过 PCIe 传输。
传输有固定开销和带宽限制。
小数据传输会被固定开销主导。
频繁小拷贝通常比一次大拷贝更差。

### pinned memory

pageable host memory 需要驱动临时锁页或中转。
pinned memory 允许 DMA 更高效，并且是 `cudaMemcpyAsync` 真正异步的重要条件。

```cpp
#include <cuda_runtime.h>
#include <cstddef>

class PinnedFloatBuffer {
public:
    explicit PinnedFloatBuffer(std::size_t n) : n_(n) {
        // 锁页内存支持更高效的异步 H2D/D2H 传输。
        checkCuda(cudaHostAlloc(&ptr_, n_ * sizeof(float), cudaHostAllocDefault));
    }

    ~PinnedFloatBuffer() {
        if (ptr_ != nullptr) {
            cudaFreeHost(ptr_);
        }
    }

    float* data() { return ptr_; }
    std::size_t size() const { return n_; }

private:
    float* ptr_{nullptr};
    std::size_t n_{0};
};
```

pinned memory 不是越多越好。
锁定太多页面会影响系统内存管理。
应只为高频传输缓冲分配。

### 结构数组与数组结构

点云常见两种布局：

```text
AoS: [x y z intensity][x y z intensity]...
SoA: [x x x ...][y y y ...][z z z ...][intensity ...]
```

GPU 通常更喜欢 coalesced 访问。
如果每个线程读取连续的 x 数组，SoA 更友好。
但 ROS 消息常是 AoS。
是否转换取决于后续计算是否足够多，能否抵消转换成本。

### 统一内存

Unified Memory 简化编程，但页面迁移可能在第一次访问时发生。
对实时路径，隐式迁移是风险。
生产系统中应明确数据驻留位置和传输时机。

### 练习

1. 比较 1000 次小 `cudaMemcpyAsync` 与 1 次大拷贝的耗时。
2. 用 pinned memory 和普通 `std::vector` 分别做 H2D 传输，比较带宽。
3. 对点云 AoS 和 SoA 两种布局写访问 kernel，分析内存合并访问。

---

## 6.4 CUDA Graphs、Thrust 与批量规划 ⭐⭐

这一节解决的问题是：怎样降低大量小 kernel 的启动开销，并快速搭建并行流水线。

### CUDA Graphs

多个 kernel 和拷贝构成固定流程时，可以捕获为 graph。
后续回放 graph 比逐个 launch 更低开销。

```cpp
#include <cuda_runtime.h>

class GraphRunner {
public:
    GraphRunner() = default;

    // 持有 cudaGraph_t / cudaGraphExec_t 两个裸资源，拷贝会导致双重销毁，
    // 因此禁用拷贝构造与拷贝赋值（需要转移所有权时另行实现移动语义）。
    GraphRunner(const GraphRunner&) = delete;
    GraphRunner& operator=(const GraphRunner&) = delete;

    void capture(cudaStream_t stream, float* data, int n) {
        checkCuda(cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal));
        launchScale(data, n, 0.5f, stream);
        launchScale(data, n, 2.0f, stream);
        checkCuda(cudaStreamEndCapture(stream, &graph_));
        // CUDA 12.0 起，带 errorNode/logBuffer 的 5 参数重载已废弃；
        // 使用 3 参数版本 cudaGraphInstantiate(&exec, graph, flags)，flags 传 0。
        checkCuda(cudaGraphInstantiate(&exec_, graph_, 0));
    }

    void run(cudaStream_t stream) {
        // graph 回放仍是异步。
        // 是否等待完成由调用者用 event 或 stream 同步决定。
        checkCuda(cudaGraphLaunch(exec_, stream));
    }

    ~GraphRunner() {
        if (exec_) cudaGraphExecDestroy(exec_);
        if (graph_) cudaGraphDestroy(graph_);
    }

private:
    cudaGraph_t graph_{nullptr};
    cudaGraphExec_t exec_{nullptr};
};
```

cuRobo 等 GPU 运动规划系统会利用类似思路把固定计算图反复回放。
这对于“多种子、多时间步、多碰撞体”的批量规划非常合适。

### Thrust

Thrust 提供类似 STL 的 GPU 并行算法。
它适合快速验证并行数据处理。
但在严格延迟路径中，应理解每个算法背后的临时分配和同步。

```cpp
#include <thrust/device_vector.h>
#include <thrust/transform.h>

void squareInPlace(thrust::device_vector<float>& v) {
    // Thrust 在设备端并行执行逐元素平方，适合快速验证 GPU 数据处理。
    thrust::transform(v.begin(), v.end(), v.begin(),
                      [] __device__ (float x) { return x * x; });
}
```

### 批量 IK 与碰撞检测

GPU 规划常把一个问题扩展成许多独立种子。
例如 512 条轨迹、64 个时间步、每个时间步检查多个球体碰撞。
每个线程处理一个轨迹点或碰撞对。

这种结构非常适合 GPU。
反过来，如果只有一条轨迹一个时间步，GPU 很难发挥。

### 练习

1. 把两个固定 kernel 串联捕获成 CUDA Graph，比较重复 1000 次的 launch 开销。
2. 用 Thrust 对 1M 个浮点数做变换，再用手写 kernel 对比。
3. 设计一个批量碰撞检测数据布局，让线程连续读取球心和半径。

---

## 6.5 深度学习部署：libtorch、TensorRT 与同步边界 ⭐⭐

这一节解决的问题是：神经网络推理怎样放进机器人软件，而不阻塞控制线程。

### 工具选择

| 工具 | 优势 | 代价 | 适合 |
|------|------|------|------|
| libtorch | API 接近 PyTorch，迁移快 | 性能不是极限 | 研究原型、复杂模型 |
| TorchScript | 部署简单 | 算子和优化受限 | 中等性能部署 |
| TensorRT | 延迟和吞吐优秀 | 构建 engine、精度校准、插件成本 | 生产推理 |
| Torch-TensorRT | 支持子图加速 | 需处理回退 | 渐进优化 |

### 控制边界

学习策略输出常进入控制系统。
如果推理线程偶发超时，控制器必须有降级策略。

| 策略 | 含义 | 适合 |
|------|------|------|
| 使用上一帧动作 | 短时推理未完成 | 输出平滑策略 |
| 切换经典控制器 | 推理连续超时 | 安全关键系统 |
| 降低模型频率 | 推理慢但可用 | 高层策略 |
| 降低输入分辨率 | 感知推理慢 | 视觉前端 |

### TensorRT 生命周期

TensorRT engine 构建应在离线或启动阶段完成。
运行阶段只做输入拷贝、enqueue、输出读取。
不要在控制循环中构建 engine。

```cpp
// 概念骨架：展示生命周期，不展开 TensorRT 具体 API。
class InferenceRunner {
public:
    bool configure() {
        // 加载 engine、分配 GPU/CPU 缓冲、创建 stream。
        return true;
    }

    bool enqueueAsync(cudaStream_t stream) {
        // 只提交推理任务，不等待完成。
        // context_->enqueueV3(stream);
        return true;
    }

    GpuJobState poll(cudaEvent_t done) {
        return pollJob(done);
    }
};
```

### 练习

1. 为一个策略网络设计推理线程与控制线程的数据交换，要求控制线程不等待 GPU。
2. 比较 libtorch 与 TensorRT 部署步骤，列出可能影响数值一致性的环节。
3. 设计连续 5 帧推理超时后的安全降级策略。

---

## 6.6 CPU SIMD 仍然重要 ⭐⭐

这一节解决的问题是：为什么上 GPU 前应该先确认 CPU SIMD 和算法结构已用好。

### VAMP 的启发

一些运动规划系统利用 CPU SIMD 就能把碰撞检测和 FK 做到极低延迟。
这说明 GPU 不是唯一加速路线。
当问题规模小、数据在 CPU、延迟要求极严时，CPU SIMD 可能更好。

### CPU 优化优先级

| 层级 | 动作 | 原因 |
|------|------|------|
| 算法 | 降低复杂度 | 最大收益 |
| 数据布局 | 连续内存、减少 cache miss | 避免内存瓶颈 |
| 批量化 | 一次处理多个查询 | 提升向量化 |
| SIMD | AVX/NEON | 利用硬件 |
| GPU | 大规模并行 | 最高开发成本 |

### 练习

1. 对一个碰撞检测函数先做数据布局优化，再考虑 GPU，比较收益。
2. 用 Eigen 或手写 SIMD 批量计算 4 个 3D 点距离，说明 CPU 向量化思路。
3. 制定一个性能优化顺序，要求每一步都有测量指标。

---

## 6.7 GPU 计算模型：线程、warp、block 与占用率 ⭐⭐⭐

这一节解决的问题是：把 CUDA kernel 从“能跑”推进到“知道为什么快或慢”。

### 从 CPU 线程直觉切换到 GPU 吞吐直觉

CPU 的核心数量少，但每个核心很强，擅长复杂分支、低延迟响应和大缓存复用。
GPU 的核心数量多，但单个线程很轻，依靠大量线程隐藏内存延迟。
这意味着 GPU 编程的基本问题不是“开几个线程”，而是“能否提供足够多、足够相似、内存访问足够规律的工作”。

可以把 CPU 类比为经验丰富的技师团队，每个人能处理复杂任务；GPU 类比为大型流水线，单个工位能力有限，但成千上万件相似工件同时流动时吞吐极高。
机器人任务中，图像像素、点云点、轨迹种子、碰撞对都像流水线工件。
小 QP、单个 6x6 矩阵分解、控制状态机则像需要技师判断的任务，不适合搬上流水线。

| CUDA 层级 | 直觉 | 典型大小 | 设计关注点 |
|-----------|------|----------|------------|
| thread | 一个轻量工作单元 | 处理一个像素/点/碰撞对 | 分支少、寄存器够用 |
| warp | 线程调度基本单位 | 通常 32 个线程 | 分支一致、内存合并 |
| block | 同一共享内存区域内的线程组 | 128/256/512 常见 | 同步和共享内存 |
| grid | 一次 kernel 的所有 block | 覆盖全数据 | 任务拆分和边界检查 |
| stream | 有序任务队列 | 多个 kernel/拷贝 | 异步流水化 |

如果不理解这些层级，kernel 可能“看起来并行”，实际却因为 warp 分支发散、全局内存离散访问或 block 配置不合适而很慢。

### warp 分支发散

一个 warp 内的线程如果走不同分支，硬件通常需要分批执行不同路径。
这会把并行变成串行片段。
图像阈值、点云过滤、碰撞检测都容易出现这种情况。

```cuda
#include <cuda_runtime.h>

__global__ void thresholdDepth(const float* depth,
                               unsigned char* mask,
                               int n,
                               float min_depth,
                               float max_depth) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;  // 边界线程直接退出，避免越界访问。
    }

    const float z = depth[i];
    // 每个线程独立判断深度是否有效，输出 0/1 掩码。
    mask[i] = (z >= min_depth && z <= max_depth) ? 1 : 0;
}
```

这段代码虽然有分支，但分支很短，通常可以接受。
真正危险的是一个分支做大量计算，另一个分支几乎不做事。
例如碰撞检测中，有些轨迹点进入复杂网格查询，有些直接跳过，warp 内线程执行时间差异会很大。
工程上常用的处理方式是先用简单 kernel 做筛选，把需要复杂处理的索引压缩出来，再对压缩后的集合运行第二个 kernel。

### 全局内存合并访问

GPU 全局内存带宽很高，但需要线程访问地址连续，才能合并成少量内存事务。
如果一个 warp 的线程分别访问相隔很远的地址，就会浪费带宽。
这正是点云 AoS/SoA 选择的重要原因。

```cuda
#include <cuda_runtime.h>

struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

__global__ void badReadXFromAoS(const PointXYZI* points, float* xs, int n) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        // AoS 中相邻 x 之间隔着 y/z/intensity，读取 x 字段不完全连续。
        xs[i] = points[i].x;
    }
}

__global__ void goodReadXFromSoA(const float* x, float* xs, int n) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        // SoA 中 x 数组连续，warp 内线程访问更容易合并。
        xs[i] = x[i];
    }
}
```

SoA 并不总是绝对更好。
如果后续每个线程都需要同一个点的 x/y/z/intensity，AoS 可能更方便，尤其在 CPU 端也是 AoS 的情况下。
决策标准仍是端到端：转换成本、后续 kernel 数量、访问字段数量、是否需要回传 CPU。

### occupancy 的含义

occupancy 通常指一个 SM 上活跃 warp 数占理论最大 warp 数的比例。
它不是越高越好。
如果 kernel 被内存延迟限制，较高 occupancy 有助于隐藏延迟。
如果 kernel 寄存器很多、计算密集，适当降低 occupancy 也可能更快。

| 限制因素 | 现象 | 调整方向 |
|----------|------|----------|
| block 太小 | 活跃线程不足 | 增大 block 或合并任务 |
| 寄存器太多 | 可驻留 warp 少 | 简化局部变量、拆分 kernel |
| 共享内存太多 | 每个 SM block 数少 | 减少 tile 或分阶段计算 |
| 全局内存离散 | 带宽低 | 改布局、合并访问 |
| 分支发散 | warp 利用率低 | 分流任务、压缩索引 |

反事实地看，如果只追求 occupancy，可能把一个清晰的 kernel 拆得过碎，增加 launch 次数和全局内存往返。
机器人系统更关心端到端 deadline，而不是某个 profiling 指标单独好看。

> **本质洞察**：GPU 性能不是由“线程数量”单独决定，而是由并行度、访存规律、分支一致性、同步边界和数据驻留共同决定。
> CUDA 调优的核心是让硬件持续有规则工作可做，同时减少 CPU 与 GPU 之间的等待。

### kernel 配置骨架

```cpp
#include <cuda_runtime.h>
#include <stdexcept>

void checkCudaStatus(cudaError_t err) {
    if (err != cudaSuccess) {
        throw std::runtime_error(cudaGetErrorString(err));
    }
}

void launchThreshold(const float* d_depth,
                     unsigned char* d_mask,
                     int n,
                     float min_depth,
                     float max_depth,
                     cudaStream_t stream) {
    const int block = 256;                 // 常用起点：每个 block 256 个线程。
    const int grid = (n + block - 1) / block;
    thresholdDepth<<<grid, block, 0, stream>>>(d_depth, d_mask, n, min_depth, max_depth);

    // 检查启动配置错误；真正的运行错误需要在后续同步或事件查询时暴露。
    checkCudaStatus(cudaGetLastError());
}
```

这个骨架体现三个原则。
第一，kernel 自己做边界检查，不要求数据长度恰好整除 block。
第二，launch 只提交到 stream，不在这里全设备同步。
第三，错误检查和同步语义分开，避免把每个 launch 都变成阻塞点。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | block/grid 写死且无边界检查 | 越界访问 | 数据长度变化 | kernel 内 `if (i<n)` |
| 编程 | 每个线程做复杂不同分支 | warp 利用率低 | 分支发散 | 先筛选再分流处理 |
| 概念 | occupancy 越高越好 | 调优方向错误 | 指标不等于目标 | 看端到端延迟 |
| 思维 | 把 CPU 线程模型套到 GPU | 任务太少仍上 GPU | 吞吐模型不同 | 批量化并减少同步 |

### 练习

1. 对同一个深度阈值 kernel 试验 block=64/128/256/512，记录端到端时间。
2. 把点云 AoS 转成 SoA，再运行只读取 x 的 kernel，比较转换前后的总时间。
3. 设计一个两阶段碰撞检测：第一阶段筛选候选，第二阶段只处理候选索引。

---

## 6.8 内存传输与流水线设计 ⭐⭐⭐

这一节解决的问题是：怎样把 H2D、kernel、D2H 组织成不阻塞控制线程的流水线。

### 数据驻留比单次加速更重要

官方 CUDA 最佳实践长期强调：应尽量减少主机与设备之间的数据传输，必要时把中间数据留在设备端连续处理。
这条原则在机器人系统里尤其重要。
如果相机图像已经由 GPU 解码，视觉网络也在 GPU 上推理，后续代价地图或轨迹评分最好继续留在 GPU。
反过来，如果状态估计、WBC 和执行器接口都在 CPU 上，小规模矩阵任务搬到 GPU 只会增加往返。

| 数据路径 | 适合 GPU 化程度 | 原因 |
|----------|----------------|------|
| 相机图像 GPU 解码 → CNN → 深度后处理 | 高 | 数据天然在 GPU |
| LiDAR CPU 驱动 → 小规模滤波 → CPU EKF | 低到中 | 传输可能主导 |
| GPU 批量规划 → CPU 只取最优轨迹 | 高 | 回传结果很小 |
| CPU WBC → GPU 小 QP → CPU 力矩 | 低 | 往返和同步主导 |

### 三缓冲流水线

单缓冲的写法通常是：拷贝输入、跑 kernel、拷贝输出、同步等待。
这会让 CPU 和 GPU 互相等。
更好的方式是使用多缓冲，把第 $k$ 帧的计算、第 $k+1$ 帧的输入拷贝、第 $k-1$ 帧的输出读取重叠起来。

```text
时间轴:
CPU:  准备帧 k+1     读取帧 k-1     控制降级判断
GPU:  H2D(k+1)  kernel(k)  D2H(k-1)
```

三缓冲不是为了增加复杂度，而是为了消除“必须等当前帧完成”的假设。
机器人控制线程每周期只查询最新可用结果。
如果 GPU 结果还没完成，就使用上一帧结果或经典控制器输出。

### pinned memory 生命周期

pinned memory 分配和释放都比普通内存重，不应放在高频循环里。
它应在初始化阶段按最大尺寸分配，在运行中复用。

```cpp
#include <cuda_runtime.h>
#include <cstddef>
#include <stdexcept>

class PinnedByteBuffer {
public:
    explicit PinnedByteBuffer(std::size_t bytes) : bytes_(bytes) {
        // 初始化阶段锁页，支持高效异步传输。
        checkCuda(cudaHostAlloc(&ptr_, bytes_, cudaHostAllocDefault));
    }

    ~PinnedByteBuffer() {
        if (ptr_ != nullptr) {
            // 退出阶段释放锁页内存。
            cudaFreeHost(ptr_);
        }
    }

    void* data() { return ptr_; }
    const void* data() const { return ptr_; }
    std::size_t bytes() const { return bytes_; }

private:
    void* ptr_{nullptr};
    std::size_t bytes_{0};
};
```

pinned memory 不是越多越好。
锁定太多页面会降低系统内存管理效率，也可能影响其他实时进程。
经验上只锁定高频、固定大小、确实需要异步传输的缓冲。

### 异步流水线骨架

```cpp
#include <cuda_runtime.h>
#include <array>

struct GpuFrameSlot {
    PinnedByteBuffer h_input;
    PinnedByteBuffer h_output;
    void* d_input{nullptr};
    void* d_output{nullptr};
    cudaEvent_t done{nullptr};

    GpuFrameSlot(std::size_t input_bytes, std::size_t output_bytes)
        : h_input(input_bytes), h_output(output_bytes) {
        // 设备缓冲也在初始化阶段分配，运行期只复用。
        checkCuda(cudaMalloc(&d_input, input_bytes));
        checkCuda(cudaMalloc(&d_output, output_bytes));
        checkCuda(cudaEventCreateWithFlags(&done, cudaEventDisableTiming));
    }

    ~GpuFrameSlot() {
        if (done != nullptr) cudaEventDestroy(done);
        if (d_input != nullptr) cudaFree(d_input);
        if (d_output != nullptr) cudaFree(d_output);
    }
};

void submitFrame(GpuFrameSlot* slot,
                 std::size_t input_bytes,
                 std::size_t output_bytes,
                 cudaStream_t stream) {
    // H2D 异步拷贝，要求 host 输入缓冲来自 pinned memory。
    checkCuda(cudaMemcpyAsync(slot->d_input,
                              slot->h_input.data(),
                              input_bytes,
                              cudaMemcpyHostToDevice,
                              stream));

    // 这里应调用实际 kernel；示例中省略具体算法。
    // launchKernel(slot->d_input, slot->d_output, stream);

    // D2H 异步拷贝，把结果写回 pinned 输出缓冲。
    checkCuda(cudaMemcpyAsync(slot->h_output.data(),
                              slot->d_output,
                              output_bytes,
                              cudaMemcpyDeviceToHost,
                              stream));

    // 事件记录在 stream 尾部，表示本帧所有 GPU 工作完成。
    checkCuda(cudaEventRecord(slot->done, stream));
}
```

这段代码故意没有 `cudaStreamSynchronize`。
提交函数只负责把任务放进 stream。
控制线程用 `cudaEventQuery` 查询完成状态。
这样 GPU 慢一帧时，不会把控制线程拖进不可预测等待。

### 默认 stream 的隐式同步

默认 stream 的行为容易制造隐藏同步。
如果一个库内部使用默认 stream，而你的代码使用非阻塞 stream，二者之间可能出现意外等待。
多库集成时，最好明确每个 GPU 库是否支持传入 stream。

| 库/模块 | 需要确认的问题 |
|---------|----------------|
| TensorRT | enqueue 是否使用调用者 stream |
| OpenCV CUDA | 操作是否绑定指定 stream |
| 自写 kernel | 是否全程使用非阻塞 stream |
| 第三方规划库 | 是否内部调用全设备同步 |
| 日志/可视化 | 是否触发 D2H 同步读取 |

如果无法控制第三方库的同步行为，就不要把它放在硬实时控制线程中。
可以把它放在感知线程或规划线程，通过时间戳和双缓冲把结果交给控制器。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 循环中 `cudaHostAlloc` | 周期抖动 | 锁页分配很重 | 初始化阶段分配 |
| 编程 | `cudaMemcpyAsync` 用普通 `std::vector` | 不能真正重叠 | host 内存未锁页 | 使用 pinned memory |
| 概念 | 认为异步提交等于数据可读 | 读到旧结果 | GPU 尚未完成 | event 查询 |
| 思维 | 强行等待当前帧 | 控制线程卡住 | 缺少降级策略 | 多缓冲和上一帧输出 |

### 练习

1. 实现双缓冲或三缓冲 GPU 图像处理流水线，要求控制线程只查询事件不阻塞。
2. 在同一机器上比较 pageable 和 pinned memory 的 H2D/D2H 带宽，并记录缓冲大小对结果的影响。
3. 搜索项目中所有默认 stream 和 `cudaDeviceSynchronize()` 使用点，说明哪些必须移出控制线程。

---

## 6.9 CUDA kernel 设计：以深度图和点云为例 ⭐⭐⭐

这一节解决的问题是：如何把机器人感知中的规则数据处理写成可维护的 CUDA kernel。

### 深度图滤波的并行结构

深度图天然适合 GPU。
一张 640x480 图像有 307200 个像素。
如果每个像素的处理只依赖局部邻域，线程之间几乎独立。
这类任务常见于深度阈值、双边滤波、法向估计、代价地图投影。

| 算法 | 每线程工作 | 访存模式 | 注意点 |
|------|------------|----------|--------|
| 阈值滤波 | 读一个像素写一个 mask | 连续 | 简单高吞吐 |
| 3x3 均值 | 读邻域写中心 | 局部 | 边界处理 |
| 双边滤波 | 读邻域和权重 | 局部更多 | 可用 shared memory |
| 点云投影 | 一个点投影到像素 | 写冲突 | 需要原子或分阶段 |

### 3x3 深度均值 kernel

```cuda
#include <cuda_runtime.h>

__global__ void meanDepth3x3(const float* depth_in,
                             float* depth_out,
                             int width,
                             int height,
                             float invalid_value) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) {
        return;  // 超出图像范围的线程不做计算。
    }

    float sum = 0.0f;
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                const float z = depth_in[ny * width + nx];
                if (z > 0.0f) {
                    sum += z;
                    ++count;
                }
            }
        }
    }

    // 邻域内没有有效深度时，写入约定的无效值。
    depth_out[y * width + x] = (count > 0) ? (sum / count) : invalid_value;
}
```

这个 kernel 的教学价值在于完整展示边界检查和无效值处理。
它不是最极致性能版本。
如果要进一步优化，可以把 tile 加 halo 放进 shared memory，减少全局内存重复读取。
但在机器人系统里，先写出正确、可测、可降级的版本更重要。

### 点云体素化的写冲突

点云体素化常见做法是每个线程处理一个点，计算它落在哪个 voxel。
问题是多个点可能写同一个 voxel，导致写冲突。
最简单的方法是使用原子操作更新计数或最小深度。

```cuda
#include <cuda_runtime.h>

__global__ void countPointsPerVoxel(const float* xs,
                                    const float* ys,
                                    const float* zs,
                                    int* voxel_counts,
                                    int n_points,
                                    float voxel_size,
                                    int dim_x,
                                    int dim_y,
                                    int dim_z) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_points) {
        return;  // 每个线程处理一个点。
    }

    const int ix = static_cast<int>(floorf(xs[i] / voxel_size));
    const int iy = static_cast<int>(floorf(ys[i] / voxel_size));
    const int iz = static_cast<int>(floorf(zs[i] / voxel_size));

    if (ix < 0 || ix >= dim_x || iy < 0 || iy >= dim_y || iz < 0 || iz >= dim_z) {
        return;  // 体素范围外的点直接忽略。
    }

    const int linear = (iz * dim_y + iy) * dim_x + ix;
    atomicAdd(&voxel_counts[linear], 1);  // 多个点可能落到同一体素，必须原子累加。
}
```

原子操作不是不能用，而是要知道它的代价。
如果大量点集中在少数 voxel，原子冲突会严重。
可以先按 voxel id 排序，再做 reduce；也可以分块统计局部直方图，再合并。
选择哪种方案取决于点云密度、voxel 分辨率和延迟预算。

### shared memory 何时值得使用

shared memory 适合一个 block 内线程重复读取同一片数据。
图像卷积、局部邻域滤波、距离场 tile 查询都可能受益。
但 shared memory 会增加代码复杂度，也会限制 occupancy。
如果每个元素只读取一次，shared memory 通常没有意义。

| 是否使用 shared memory | 判断依据 |
|------------------------|----------|
| 值得 | 同一数据被 block 内多个线程重复读取 |
| 值得 | 全局内存访问可通过 tile 变连续 |
| 不值得 | 每个线程只读一次且访问已连续 |
| 不值得 | 共享内存占用导致活跃 block 明显减少 |

### kernel 错误定位

CUDA 错误常延迟暴露。
kernel 内越界可能在后续 `cudaMemcpy` 或同步时才报。
调试阶段可以使用更强同步，生产路径再移除。

```cpp
#include <cuda_runtime.h>
#include <stdexcept>

void debugSynchronizeKernel(cudaStream_t stream) {
    // 调试阶段限定同步当前 stream，便于把错误定位到最近的 kernel。
    checkCuda(cudaStreamSynchronize(stream));
}

void productionRecordEvent(cudaEvent_t event, cudaStream_t stream) {
    // 运行阶段记录事件，让控制线程查询，不主动阻塞。
    checkCuda(cudaEventRecord(event, stream));
}
```

调试和运行策略不同并不矛盾。
调试时追求错误定位清晰，运行时追求不阻塞控制线程。
关键是不要把调试同步遗留在高频路径中。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 点云体素写入不用原子 | 计数随机错误 | 多线程写冲突 | 原子、排序 reduce 或分块合并 |
| 编程 | 图像边界不检查 | 偶发非法访问 | 邻域越界 | 显式处理边界 |
| 概念 | shared memory 必然更快 | 代码更慢 | 重用不足或 occupancy 降低 | 先测全局内存版本 |
| 思维 | 只看 kernel 时间 | pipeline 仍慢 | 忽略传输和同步 | 测端到端 |

### 练习

1. 实现 3x3 深度均值滤波，并比较 CPU/OpenMP/CUDA 三种端到端时间。
2. 对点云体素计数构造“均匀分布”和“集中在少数 voxel”两种输入，观察原子冲突影响。
3. 把一个图像滤波 kernel 改成 shared memory tile 版本，说明收益来自哪一类访存减少。

---

## 6.10 机器人感知、规划与策略部署案例 ⭐⭐⭐

这一节解决的问题是：GPU 在机器人系统里到底应该落在哪些模块，而不是停留在抽象讨论。

### 案例一：视觉感知流水线

典型视觉机器人流水线：

```text
相机采集
  ↓
图像解码/去畸变
  ↓
神经网络推理
  ↓
深度/语义/关键点后处理
  ↓
局部地图或任务状态
  ↓
规划与控制
```

如果相机驱动、图像预处理、网络推理都支持 GPU，数据应该尽量留在 GPU。
CPU 只接收压缩后的任务状态，例如目标位姿、障碍物列表或低维特征。
这能把 D2H 传输从整张图像降到几十个浮点数。

| 输出类型 | 回传大小 | 控制意义 |
|----------|----------|----------|
| 全分辨率语义图 | 大 | 适合建图线程 |
| 障碍物多边形 | 中 | 适合局部规划 |
| 目标 6D 位姿 | 小 | 适合视觉伺服 |
| 策略网络动作 | 很小 | 适合高层命令 |

### 案例二：批量轨迹评分

采样式局部规划经常生成几百到几千条候选轨迹。
每条轨迹有几十个时间步，每个时间步需要查询距离场、动态障碍物和代价函数。
这正是 GPU 擅长的“多轨迹、多时间步、多查询”结构。

```cuda
#include <cuda_runtime.h>

__global__ void scoreTrajectories(const float* states,
                                  const float* sdf,
                                  float* costs,
                                  int n_traj,
                                  int horizon,
                                  int sdf_width,
                                  int sdf_height,
                                  float resolution) {
    const int traj = blockIdx.x;
    const int t = threadIdx.x;
    if (traj >= n_traj || t >= horizon) {
        return;  // 一个 block 处理一条轨迹，线程处理不同时间步。
    }

    const int state_index = (traj * horizon + t) * 3;
    const float x = states[state_index + 0];
    const float y = states[state_index + 1];

    const int ix = static_cast<int>(x / resolution);
    const int iy = static_cast<int>(y / resolution);

    float cost = 0.0f;
    if (ix < 0 || ix >= sdf_width || iy < 0 || iy >= sdf_height) {
        cost = 1e6f;  // 地图外轨迹给高代价。
    } else {
        const float dist = sdf[iy * sdf_width + ix];
        cost = (dist > 0.0f) ? (1.0f / (dist + 1e-3f)) : 1e6f;
    }

    // 简化示例：每个时间步写独立代价，后续再做并行规约。
    costs[traj * horizon + t] = cost;
}
```

这个例子没有在单个 kernel 内完成规约，是为了保持结构清楚。
工程版本可以用 block 内规约把一条轨迹的代价合成一个数，也可以用 CUB/Thrust 做后处理。
关键是把问题排列成“一个轨迹一个 block”或“一个轨迹点一个线程”的规则形态。

### 案例三：批量 IK 与碰撞检测

机械臂抓取常同时评估许多候选末端位姿和许多 IK 初值。
CPU 单次 IK 延迟可能很低，但 1024 个初值乘以多组候选就会变成吞吐问题。
GPU 的适用条件是：每个种子独立、迭代次数上限固定、失败分支可接受。

| 子任务 | GPU 化方式 | 注意点 |
|--------|------------|--------|
| 多初值 IK | 每个线程或 warp 一个种子 | 控制最大迭代次数 |
| 自碰撞粗筛 | 每个线程一个球体对 | 数据布局连续 |
| 距离场查询 | 每个轨迹点查询 SDF | 纹理或缓存友好 |
| 最优候选选择 | 并行 reduce | 回传最小索引 |

如果只有一个目标、一个初值、一个机械臂，GPU 不会神奇地降低延迟。
批量才是关键。

### 案例四：强化学习策略部署

腿足机器人策略网络通常以较低频率输出期望速度、足端目标或关节目标，再由经典控制器在高频闭环中执行。
GPU 适合运行较大的感知网络或批量策略评估。
但最终关节力矩通常仍由 CPU 上的实时控制器产生。

| 部署结构 | GPU 角色 | CPU 角色 | 适用场景 |
|----------|----------|----------|----------|
| 视觉策略 | 图像编码和策略推理 | 状态融合和低层控制 | 视觉导航 |
| 特权训练后部署 | 离线训练用 GPU | 在线策略可 CPU 或 GPU | 低维状态腿足 |
| 批量仿真训练 | 大量环境并行 | 记录与调度 | 策略训练 |
| 在线残差策略 | 输出修正项 | MPC/WBC 保底 | 安全敏感控制 |

策略部署的关键不是“网络在哪跑”，而是“超时时系统怎么办”。
如果策略输出未及时到达，控制器必须有上一帧保持、经典控制器接管、速度限幅或紧急停止逻辑。

```cpp
#include <cuda_runtime.h>

struct PolicyOutput {
    float command[12];
    double stamp_sec{0.0};
    bool valid{false};
};

class PolicyBridge {
public:
    PolicyOutput readForControl(double now_sec, double max_age_sec) const {
        // 控制线程只读取最近完成的输出，不等待 GPU 推理。
        PolicyOutput out = latest_;
        if (!out.valid || now_sec - out.stamp_sec > max_age_sec) {
            out.valid = false;  // 输出过旧时，上层应切换保底控制。
        }
        return out;
    }

    void publishFromInference(const PolicyOutput& out) {
        // 推理线程在 GPU 任务完成后发布结果。
        latest_ = out;
    }

private:
    PolicyOutput latest_;
};
```

示例省略了锁或无锁缓冲实现，因为不同系统的线程模型不同。
核心原则是控制线程不等待推理线程。
如果共享数据跨线程访问，工程中需要用双缓冲、原子序号或实时安全队列保护一致性。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 推理线程和控制线程共享裸缓冲 | 偶发读到半帧数据 | 缺少发布协议 | 双缓冲或原子序号 |
| 编程 | 轨迹评分每条轨迹单独 launch | 启动开销巨大 | 批量没有合并 | 一个 kernel 覆盖全部轨迹 |
| 概念 | 认为策略输出能直接替代低层控制 | 机器人动作不稳定 | 缺少高频反馈 | 策略给目标，控制器闭环执行 |
| 思维 | 感知结果必须每帧最新 | 控制线程等待 | 实时性优先级错误 | 使用时间戳和过期策略 |

### 练习

1. 设计一个视觉策略部署结构，要求图像和网络推理留在 GPU，只把低维命令传给 CPU 控制器。
2. 为 1024 条轨迹、64 个时间步的局部规划设计数据布局和 kernel 映射方式。
3. 制定策略网络连续超时 1/3/10 帧时的分级降级逻辑。

---

## 6.11 GPU 与实时控制的系统边界 ⭐⭐⭐

这一节解决的问题是：GPU 任务如何进入机器人软件架构，而不破坏实时控制的确定性。

### 控制频率分层

机器人软件天然分层。
不是所有模块都需要 1 kHz，也不是所有模块都能容忍 100 ms。
GPU 任务通常适合感知、规划、学习推理和批量仿真，不适合直接卡在电机控制闭环里。

| 层级 | 典型频率 | 可接受延迟 | GPU 适合度 |
|------|----------|------------|------------|
| 电流环/驱动 | 5-20 kHz | 微秒级 | 很低 |
| 关节控制/WBC | 500 Hz-1 kHz | 1-2 ms | 低 |
| MPC/局部规划 | 20-100 Hz | 5-50 ms | 中到高 |
| 感知网络 | 10-60 Hz | 10-100 ms | 高 |
| 离线训练/批量仿真 | 非实时 | 吞吐优先 | 很高 |

GPU 的输出越靠近执行器，越需要严格降级。
例如视觉网络给出的目标位姿可以慢一帧；关节力矩不能慢一帧。
如果一个 GPU 任务失败，系统应退化为保守行为，而不是让控制线程等待。

### deadline 与 freshness

实时系统看两个量：deadline 和 freshness。
deadline 问“这次计算是否在规定时间内完成”。
freshness 问“这个结果是否足够新”。
GPU 异步任务可能在 deadline 后完成，此时结果虽然数值正确，但对当前状态已经过期。

| 结果状态 | 处理 |
|----------|------|
| 准时且新 | 正常使用 |
| 准时但状态时间戳不匹配 | 重新投影或降低权重 |
| 超时但很快完成 | 供下一周期参考 |
| 连续超时 | 降级或关闭该模块 |
| CUDA 错误 | 切换保底控制并记录错误 |

### 非阻塞控制读取

```cpp
#include <cuda_runtime.h>

enum class ResultStatus {
    kReady,
    kRunning,
    kFailed,
    kExpired
};

ResultStatus queryGpuResult(cudaEvent_t done,
                            double result_stamp,
                            double now,
                            double max_age) {
    const cudaError_t status = cudaEventQuery(done);
    if (status == cudaErrorNotReady) {
        return ResultStatus::kRunning;  // GPU 尚未完成，控制线程不等待。
    }
    if (status != cudaSuccess) {
        return ResultStatus::kFailed;   // CUDA 运行错误，进入保底路径。
    }
    if (now - result_stamp > max_age) {
        return ResultStatus::kExpired;  // 结果完成但已经过期。
    }
    return ResultStatus::kReady;
}
```

这个函数把“完成”和“可用”分开。
完成只说明 GPU 工作结束，可用还要求时间戳满足控制需求。
这是机器人系统和离线批处理最大的差异之一。

### GPU 资源竞争

一台机器人上可能同时运行视觉网络、局部地图、规划器和日志可视化。
如果它们共享同一块 GPU，任何一个模块的峰值负载都可能影响其他模块。
因此 GPU 模块需要资源预算：最大显存、最大运行时间、最大输入尺寸和降频策略。

| 资源 | 预算方式 |
|------|----------|
| 显存 | 启动阶段一次性分配，运行期禁止无限增长 |
| stream | 按模块固定创建，避免循环创建 |
| 输入尺寸 | 限制图像分辨率、点云数量、轨迹数量 |
| 推理时间 | 记录分位数和最坏时间 |
| 错误恢复 | CUDA 错误后隔离模块或重建上下文 |

如果不做资源预算，GPU 很容易变成系统中“看不见的共享瓶颈”。
某次视觉输入分辨率升高，可能让规划器错过 deadline。
某个调试可视化读回大纹理，可能把控制线程间接拖慢。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 控制线程等待 GPU 推理 | 力矩输出抖动 | 阻塞破坏实时性 | 查询事件并降级 |
| 编程 | 多模块无显存预算 | 运行一段时间后失败 | 资源竞争 | 启动阶段预分配 |
| 概念 | 计算完成就一定可用 | 使用过期感知结果 | 忽略时间戳 | 检查 freshness |
| 思维 | 只优化平均时间 | 偶发 deadline miss | 尾部延迟未控制 | 记录 P95/P99/最坏值 |

### 练习

1. 为一个 50 Hz GPU 局部规划器设计 deadline、freshness 和连续超时降级规则。
2. 记录 GPU 模块 10 分钟运行的 P50/P95/P99/最大延迟，判断是否满足机器人任务。
3. 设计一个多 GPU 模块资源预算表，包含显存、stream、输入尺寸和最大运行时间。

---

## 6.12 什么时候不该上 GPU ⭐⭐⭐

这一节解决的问题是：把“不上 GPU”也作为一种明确的工程决策，而不是失败或保守。

### 不上 GPU 的五类信号

GPU 加速的价值来自吞吐和数据驻留。
如果一个任务没有足够并行度，或者结果必须立即回到 CPU，GPU 可能是错误选择。

| 信号 | 解释 | 例子 |
|------|------|------|
| 数据很小 | launch 和传输超过计算 | 3x3/6x6 小矩阵 |
| 强序列依赖 | 后一步依赖前一步结果 | 单条链式递推 |
| CPU 已有低延迟实现 | 优化空间不在硬件 | WBC 小 QP |
| 必须同步回 CPU | 异步优势消失 | 每周期读取力矩 |
| 开发维护成本过高 | bug 面和部署复杂度上升 | 小团队非瓶颈模块 |

不上 GPU 不是放弃性能。
很多时候正确路径是先改算法、改数据结构、减少拷贝、使用 CPU SIMD、批量化，再评估 GPU。
如果这些步骤已经满足 deadline，GPU 化只会增加系统复杂度。

### 决策流程

```text
任务是否已经超过 deadline？
├── 否 → 不上 GPU，保留测量基线
└── 是
    ├── 是否有足够并行度？
    │   ├── 否 → 优先算法/CPU SIMD/缓存优化
    │   └── 是
    ├── 数据是否已在 GPU 或可长时间驻留？
    │   ├── 否 → 估算传输与同步成本
    │   └── 是
    ├── 结果是否必须本周期回 CPU？
    │   ├── 是 → 谨慎，设计降级或保留 CPU 路径
    │   └── 否 → 适合异步 GPU 流水线
    └── 是否能维护、测试、部署？
        ├── 否 → 保持 CPU 实现
        └── 是 → 建立 GPU 版本并测端到端收益
```

### 小 QP 为什么通常不适合 GPU

以 36 变量 WBC QP 为例。
CPU 上成熟求解器可能几十到几百微秒完成。
GPU 版本需要：

1. 把状态、雅可比、Hessian、约束传到 GPU。
2. 启动一个或多个 kernel。
3. 求解后把关节力矩或解向量传回 CPU。
4. 同步等待结果，因为执行器需要当前周期输出。

即使 kernel 本身很快，步骤 1、2、3、4 的固定成本也会吞掉收益。
除非一次求解大量 QP，或者整个 WBC 和执行接口都在 GPU 上，否则这类任务应留在 CPU。

### CPU 路径的优化顺序

| 顺序 | 动作 | 为什么先做 |
|------|------|------------|
| 1 | 测端到端时间 | 找真实瓶颈 |
| 2 | 降低算法复杂度 | 收益最大 |
| 3 | 固定尺寸和预分配 | 消除 malloc 和动态分派 |
| 4 | 改数据布局 | 提高 cache 命中 |
| 5 | 使用 SIMD/并行 CPU | 成本低于 GPU |
| 6 | 批量化后评估 GPU | 确保并行度足够 |

这与上一章 Eigen 的实时审计相互衔接。
如果 CPU 代码还在控制循环里动态分配、反复构造稀疏结构、显式求逆，那么直接上 GPU 是绕过真正问题。

### 保留 CPU 回退路径

只要 GPU 结果参与机器人运动，就应保留 CPU 回退路径。
回退路径不一定性能相同，但必须物理安全。
例如视觉规划失效时降到低速避障；策略推理失效时切换到站立控制；GPU 距离场不可用时使用保守膨胀障碍。

```cpp
#include <Eigen/Dense>

enum class PlannerMode {
    kGpuNominal,
    kCpuFallback,
    kHoldLastSafeCommand
};

struct MotionCommand {
    Eigen::Vector3d base_velocity = Eigen::Vector3d::Zero();
    bool safe{true};
};

MotionCommand chooseCommand(PlannerMode mode,
                            const MotionCommand& gpu_cmd,
                            const MotionCommand& cpu_cmd,
                            const MotionCommand& last_safe) {
    if (mode == PlannerMode::kGpuNominal && gpu_cmd.safe) {
        return gpu_cmd;  // GPU 结果准时且安全，正常使用。
    }
    if (mode == PlannerMode::kCpuFallback && cpu_cmd.safe) {
        return cpu_cmd;  // GPU 不可用时，使用 CPU 保守规划。
    }
    return last_safe;    // 两者都不可用时，保持上一条安全命令。
}
```

这段代码的重点是把模式切换显式化。
不要让异常、超时或空指针自然传播到控制输出。
安全相关系统应该把“没有 GPU 结果”视为正常工况之一，而不是罕见故障。

### 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 删除 CPU 版本只保留 GPU | GPU 错误时无输出 | 没有回退路径 | 保留保守 CPU 路径 |
| 编程 | 小任务每周期 H2D/D2H | 比 CPU 慢 | 固定成本主导 | 留在 CPU 或批量化 |
| 概念 | GPU 快等于系统快 | 端到端不变甚至变慢 | 同步和传输未计入 | 测完整 pipeline |
| 思维 | 为了新技术改架构 | 维护成本上升 | 目标不清 | 以 deadline 和风险决策 |

### 练习

1. 选一个项目中的小矩阵任务，估算 FLOPs、传输字节数和同步次数，判断是否适合 GPU。
2. 为 GPU 局部规划器设计 CPU 回退路径，要求能连续运行 10 秒而不依赖 GPU 输出。
3. 写一份“不上 GPU”的技术说明，包含测量数据、瓶颈分析和替代优化方案。

---

## 6.13 cuBLAS/cuSOLVER 在机器人动力学中的应用 ⭐⭐⭐

这一节解决的问题是：当矩阵运算规模足够大或需要批量处理时，如何用 NVIDIA 数学库替代手写 kernel。

### 动机：不要重复发明矩阵乘法

机器人动力学中充满矩阵运算：质量矩阵 $M(q)$ 的组装、雅可比矩阵 $J$ 的计算、线性方程组 $M \ddot{q} = \tau$ 的求解。当这些运算在 CPU 上以单次执行时，Eigen 已经足够高效。但当你需要批量处理时——例如 GPU 并行仿真中 256 个环境同时计算动力学、或 MPC 展开中 50 个时间步各自需要一次矩阵分解——CPU 的单次低延迟优势就被批量需求淹没了。

cuBLAS 提供 GPU 上的密集线性代数运算（矩阵乘、三角分解等）。cuSOLVER 提供分解和求解功能（LU、Cholesky、QR、SVD 等）。两者都有批量（batched）版本，专门为"大量小矩阵"场景设计。

可以把 cuBLAS 类比为 GPU 上的 BLAS/LAPACK。你不会在 CPU 上手写矩阵乘法内循环——Eigen 底层调用 BLAS 完成这件事。同样，你也不应该在 GPU 上手写矩阵乘法 kernel——cuBLAS 的实现经过 NVIDIA 多年优化，利用了 Tensor Core、内存层次和指令调度的细节。

### 批量矩阵运算的场景

| 场景 | 矩阵大小 | 批量数 | 为什么需要批量 |
|------|----------|--------|----------------|
| GPU 并行仿真 | 6x6 到 30x30 | 256-4096 | 每个仿真环境独立计算动力学 |
| MPC 展开 | 状态维度 n | 10-50 | 每个时间步的 Riccati 递推 |
| 批量 IK | 6x6 雅可比 | 512-2048 | 多初值并行求解 |
| 蒙特卡洛碰撞检测 | 3x3 或 4x4 | 数千-数万 | 大量候选姿态的变换 |
| 神经网络训练 | 隐藏层维度 | batch size | 这是 GPU 的传统强项 |

### cuBLAS 批量矩阵乘法示例

```cpp
#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <vector>
#include <stdexcept>

void checkCublas(cublasStatus_t status) {
    if (status != CUBLAS_STATUS_SUCCESS) {
        throw std::runtime_error("cuBLAS error");
    }
}

// 批量计算 C[i] = A[i] * B[i]，i = 0..batch_count-1
// 适合 GPU 并行仿真中每个环境独立计算雅可比乘以速度向量。
void batchedMatmul(cublasHandle_t handle,
                   int m, int n, int k,
                   const float** d_A_array,
                   const float** d_B_array,
                   float** d_C_array,
                   int batch_count) {
    const float alpha = 1.0f;
    const float beta = 0.0f;
    // cublasSgemmBatched 一次调用完成所有批次的矩阵乘法。
    // 比循环调用 cublasSgemm 减少大量 kernel launch 开销。
    checkCublas(cublasSgemmBatched(
        handle,
        CUBLAS_OP_N, CUBLAS_OP_N,
        m, n, k,
        &alpha,
        d_A_array, m,     // A 矩阵数组
        d_B_array, k,     // B 矩阵数组
        &beta,
        d_C_array, m,     // C 矩阵数组
        batch_count));
}
```

### cuSOLVER 批量 Cholesky 分解

在机器人动力学中，质量矩阵 $M(q)$ 是对称正定的。对称正定矩阵最高效的分解方式是 Cholesky 分解。cuSOLVER 提供批量 Cholesky 分解，适合 GPU 并行仿真中每个环境独立求解 $M \ddot{q} = f$。

```cpp
#include <cusolverDn.h>
#include <cuda_runtime.h>
#include <stdexcept>

void checkCusolver(cusolverStatus_t status) {
    if (status != CUSOLVER_STATUS_SUCCESS) {
        throw std::runtime_error("cuSOLVER error");
    }
}

// 批量 Cholesky 分解：对 batch_count 个 n×n 对称正定矩阵同时分解。
// 适合 GPU 并行仿真中每个环境的质量矩阵分解。
void batchedCholesky(cusolverDnHandle_t handle,
                     int n,
                     float** d_A_array,
                     int* d_info,
                     int batch_count) {
    // 查询工作空间大小。
    int work_size = 0;
    checkCusolver(cusolverDnSpotrfBatched(
        handle,
        CUBLAS_FILL_MODE_LOWER,
        n,
        d_A_array,
        n,
        d_info,
        batch_count));
    // d_info[i] == 0 表示第 i 个矩阵分解成功。
    // d_info[i] > 0 表示第 i 个矩阵不是正定的（工程上应处理这种情况）。
}
```

### 批量 vs 单次的选型判断

| 因素 | 偏向单次 CPU | 偏向批量 GPU |
|------|-------------|-------------|
| 矩阵数量 | 1 | 数百以上 |
| 矩阵大小 | 很小（3x3, 6x6） | 中等以上或数量补偿 |
| 结果是否需要立即回 CPU | 是 | 否，后续计算仍在 GPU |
| 是否在 GPU 并行仿真环境中 | 否 | 是 |
| 开发维护成本 | 低 | 中到高 |

反事实地看，如果每个仿真环境的动力学计算独立地在 CPU 上运行，256 个环境就需要 256 次串行的 Cholesky 分解。即使每次只需要 10 微秒，总计也要 2.56 毫秒。而 cuSOLVER 的批量版本可以在一次 kernel launch 中完成所有 256 次分解，延迟可能只有几百微秒。

### ⚠️ 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 忘记检查 cuSOLVER 的 info 数组 | 分解失败但继续使用结果 | 矩阵不正定时分解无意义 | 每次分解后检查 info |
| 编程 | cuBLAS 矩阵用行优先布局 | 结果错误 | cuBLAS 默认列优先（Fortran 风格） | 注意布局或使用转置标志 |
| 概念 | 对 3x3 小矩阵也用 cuBLAS | 比 CPU 慢 | launch 开销超过计算 | 小矩阵留在 CPU 或 warp 内手动计算 |
| 思维 | 认为 cuBLAS 自动最优 | 性能不如预期 | 没有使用批量 API 或数据布局不连续 | 使用 batched API + 连续内存布局 |

### 练习

1. 用 cuBLAS 批量矩阵乘法计算 512 个 6x6 矩阵与 6x1 向量的乘积，与 CPU 循环版本比较延迟。
2. 用 cuSOLVER 批量 Cholesky 分解求解 256 个独立的 $Ax = b$ 系统，验证结果的数值精度。
3. 估算一个 12 关节机器人的质量矩阵大小（12x12），判断 256 个并行环境下 GPU 批量分解是否有优势。

---

## 6.14 CUDA Graphs 深入：原理与高级用法 ⭐⭐⭐

这一节解决的问题是：CUDA Graphs 如何从根本上降低 kernel launch 开销，以及什么场景值得使用。

### Kernel Launch 开销的本质

每次 CUDA kernel launch，CPU 需要完成一系列工作：验证参数、构造 launch 描述符、通过驱动提交到 GPU 命令队列。这些步骤有固定开销，通常在几微秒量级。当 kernel 数量很多、每个 kernel 计算量很小时，这些固定开销累积起来可能超过实际计算时间。

可以把 kernel launch 类比为快递下单。每单的打包、写地址、叫快递员的流程都有固定成本。如果你每次只寄一封信，流程成本远超信件本身。但如果你把 100 封信打成一个包裹一次寄出，流程成本就被分摊了。CUDA Graphs 就是"打包 100 次 launch 为一次回放"。

### CUDA Graphs 的工作原理

```text
普通执行：
  CPU: launch_k1 → launch_k2 → launch_k3 → ... → launch_kN
  每次 launch 都有几微秒的 CPU 端开销

Graph 捕获：
  CPU: begin_capture → launch_k1 → launch_k2 → ... → launch_kN → end_capture
  GPU 不执行任何 kernel，只记录拓扑结构

Graph 实例化：
  CPU: instantiate(graph) → 生成可执行的 graph_exec
  一次性优化和验证

Graph 回放：
  CPU: graph_launch(graph_exec, stream)
  一次 API 调用替代 N 次 launch，CPU 开销几乎为零
```

### Graph 的限制与适用条件

CUDA Graphs 不是万能的。它有几个重要限制。

| 限制 | 原因 | 工程影响 |
|------|------|----------|
| 捕获阶段不能分配/释放 GPU 内存 | Graph 记录的是操作序列，不能有非确定性操作 | 所有缓冲必须在捕获前分配 |
| 不支持条件分支 | Graph 是静态拓扑 | 不适合输入大小变化的场景 |
| 输入数据可以变，但缓冲地址不能变 | Graph 记录的是指针 | 使用固定缓冲，每帧覆写数据 |
| 不支持主机回调（CUDA 12 之前） | 需要纯 GPU 操作 | 不能在 graph 中间做 CPU 判断 |

> **本质洞察**：CUDA Graphs 的本质是把"CPU 控制 GPU 的命令流"转变为"GPU 自主重放预录的命令流"。
> 它消除了 CPU 作为中间人的开销，但代价是执行路径必须在捕获时确定。
> 这是典型的"确定性换性能"权衡。

### cuRobo 中的 CUDA Graphs 应用

cuRobo（NVIDIA 的 GPU 加速运动规划库）大量使用 CUDA Graphs。运动规划中的正向运动学、碰撞检测和轨迹优化是固定计算图：输入是关节角度数组，经过固定的 FK-碰撞检测-代价计算-梯度计算步骤，输出是代价和梯度。每次优化迭代的计算步骤相同，只有输入数据变化。

这正是 CUDA Graphs 的理想场景。cuRobo 在第一次前向传播时捕获整个计算图，后续迭代只需要回放。

```cpp
#include <cuda_runtime.h>

class MotionPlannerGraph {
public:
    void captureForwardPass(cudaStream_t stream,
                            float* d_joint_angles,
                            float* d_costs,
                            int n_trajectories,
                            int n_timesteps) {
        // 第一次执行时捕获完整的前向计算图。
        checkCuda(cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal));

        // FK → 碰撞检测 → 代价计算 → 梯度计算
        launchFK(d_joint_angles, d_ee_poses_, n_trajectories * n_timesteps, stream);
        launchCollision(d_ee_poses_, d_distances_, n_trajectories * n_timesteps, stream);
        launchCostEval(d_distances_, d_costs, n_trajectories, n_timesteps, stream);
        launchGradient(d_costs, d_gradients_, n_trajectories, n_timesteps, stream);

        checkCuda(cudaStreamEndCapture(stream, &graph_));
        // 3 参数版本（CUDA 12.0+）；旧的 5 参数重载已废弃。
        checkCuda(cudaGraphInstantiate(&exec_, graph_, 0));
        captured_ = true;
    }

    void replayIteration(cudaStream_t stream) {
        // 后续迭代只需回放，CPU 端开销几乎为零。
        // 输入数据已经被覆写到 d_joint_angles 缓冲中。
        checkCuda(cudaGraphLaunch(exec_, stream));
    }

    ~MotionPlannerGraph() {
        if (exec_) cudaGraphExecDestroy(exec_);
        if (graph_) cudaGraphDestroy(graph_);
    }

private:
    cudaGraph_t graph_{nullptr};
    cudaGraphExec_t exec_{nullptr};
    bool captured_{false};
    // 所有中间缓冲在捕获前分配，地址固定。
    float* d_ee_poses_{nullptr};
    float* d_distances_{nullptr};
    float* d_gradients_{nullptr};
};
```

### Graph 更新：输入变化但拓扑不变

CUDA 11.1 引入了 `cudaGraphExecUpdate` API，允许在不重新实例化的情况下更新 graph 中某些 kernel 的参数。这进一步降低了"拓扑不变但参数变化"场景的开销。

### ⚠️ 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 在捕获阶段调用 `cudaMalloc` | 捕获失败 | Graph 不支持非确定性操作 | 预分配所有缓冲 |
| 编程 | 每帧重新捕获和实例化 | 开销比普通 launch 还大 | 捕获和实例化本身有成本 | 捕获一次，重复回放 |
| 概念 | 对变长输入使用 Graph | 捕获的 grid 大小不匹配 | Grid/block 在捕获时固定 | 固定最大尺寸，用掩码处理短输入 |
| 思维 | 所有 kernel 序列都用 Graph | 不必要的复杂度 | 只有 kernel 数量多且固定时收益明显 | 先测量 launch 开销占比 |

### 练习

1. 将一个包含 5 个 kernel 的固定流水线捕获为 CUDA Graph，比较 1000 次回放与 1000 次普通 launch 的总耗时。
2. 在 CUDA Graph 中尝试调用 `cudaMalloc`，观察并解释捕获错误。
3. 设计一个"批量轨迹优化"场景，说明 Graph 如何消除每次迭代的 CPU 端开销。

---

## 6.15 PyTorch C++（LibTorch）推理部署 ⭐⭐⭐

这一节解决的问题是：如何把 PyTorch 训练的模型以 C++ 部署到机器人系统中。

### 为什么需要 C++ 部署

Python 推理在研究原型中完全可行。但机器人系统常有以下需求使 C++ 成为更好的选择。

| 需求 | Python 的限制 | C++ 的优势 |
|------|-------------|-----------|
| 延迟确定性 | GIL、GC 导致抖动 | 无 GIL，手动内存管理 |
| 嵌入式部署 | Python 运行时开销 | 精简依赖 |
| 与 C++ 控制栈集成 | 跨语言调用开销 | 同一进程同一语言 |
| 实时线程隔离 | Python 不适合实时线程 | 可控的线程模型 |

### LibTorch 推理骨架

```cpp
#include <torch/script.h>
#include <torch/torch.h>
#include <memory>
#include <vector>
#include <stdexcept>

class PolicyInference {
public:
    explicit PolicyInference(const std::string& model_path) {
        // 加载 TorchScript 模型，应在启动阶段完成。
        model_ = torch::jit::load(model_path);
        model_.to(torch::kCUDA);
        model_.eval();
        // Warm up：第一次推理可能触发 JIT 编译和内存分配。
        warmUp();
    }

    std::vector<float> infer(const std::vector<float>& observation) {
        // 从观测向量创建输入 tensor。
        auto options = torch::TensorOptions()
            .dtype(torch::kFloat32)
            .device(torch::kCUDA);
        auto input = torch::from_blob(
            const_cast<float*>(observation.data()),
            {1, static_cast<long>(observation.size())},
            torch::TensorOptions().dtype(torch::kFloat32))
            .to(torch::kCUDA);

        // 推理时禁用梯度计算，减少内存和计算开销。
        torch::NoGradGuard no_grad;
        auto output = model_.forward({input}).toTensor();

        // 结果拷贝回 CPU。对低维输出（如 12 个关节目标），D2H 开销很小。
        auto cpu_output = output.to(torch::kCPU).contiguous();
        const float* data = cpu_output.data_ptr<float>();
        return std::vector<float>(data, data + cpu_output.numel());
    }

private:
    void warmUp() {
        // 用随机输入运行一次，触发所有延迟初始化。
        std::vector<float> dummy(48, 0.0f);
        infer(dummy);
    }

    torch::jit::script::Module model_;
};
```

### TorchScript 模型导出

```python
import torch

class SimplePolicy(torch.nn.Module):
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(obs_dim, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, act_dim),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)

# 训练后导出为 TorchScript。
model = SimplePolicy(48, 12)
model.eval()
scripted = torch.jit.script(model)
scripted.save("policy.pt")
# 导出后的 .pt 文件可以被 LibTorch C++ 直接加载。
```

### LibTorch vs TensorRT 的选型

| 维度 | LibTorch | TensorRT |
|------|----------|----------|
| 部署复杂度 | 低，直接加载 .pt | 高，需要构建 engine |
| 推理延迟 | 中等 | 低（显著优化） |
| 精度控制 | FP32/FP16 | FP32/FP16/INT8，需要校准 |
| 算子覆盖 | 与 PyTorch 一致 | 部分算子需要插件 |
| 模型更新 | 替换 .pt 文件 | 重新构建 engine |
| 适合阶段 | 研究和快速迭代 | 生产和嵌入式部署 |

反事实地看，如果只用 TensorRT，每次修改网络结构都需要重新构建 engine，调试迭代效率很低。如果只用 LibTorch，生产部署时可能无法达到延迟目标。因此工程上常见的路径是：训练阶段用 PyTorch，验证阶段用 LibTorch 快速集成，量产阶段用 TensorRT 优化。

### ⚠️ 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 推理时未关闭梯度 | 显存占用增加、推理慢 | 默认记录计算图用于反向传播 | 使用 `torch::NoGradGuard` |
| 编程 | 未 warm up 就测量延迟 | 首次推理慢 10 倍 | JIT 编译和 CUDA 初始化 | 启动阶段至少推理 3 次 |
| 概念 | 认为 TorchScript 支持所有 PyTorch 操作 | 导出失败 | 部分动态控制流和 Python 操作不支持 | 用 `torch.jit.script` 提前验证 |
| 工程 | 推理和控制在同一线程 | 控制频率被推理延迟拉低 | 推理延迟不确定 | 推理在独立线程，通过双缓冲传结果 |

### 练习

1. 导出一个简单策略网络为 TorchScript，用 LibTorch C++ 加载并推理，比较 Python 和 C++ 的推理延迟。
2. 设计推理线程和控制线程的数据交换，要求控制线程不等待推理完成。
3. 在 LibTorch 推理中加入 CUDA event 计时，记录 100 次推理的 P50/P95/最大延迟。

---

## 6.16 TensorRT 推理优化流程 ⭐⭐⭐⭐

这一节解决的问题是：如何把 TensorRT 从"知道很快"推进到"能完成从 ONNX 到部署的完整流程"。

### TensorRT 的优化步骤

TensorRT 不直接运行 PyTorch 或 TensorFlow 模型。它需要一个转换和优化过程。

```text
训练框架（PyTorch/TF）
  ↓ 导出
ONNX 中间表示
  ↓ TensorRT 解析
网络定义（INetworkDefinition）
  ↓ 构建器优化
   - 层融合（Conv+BN+ReLU → 一个 kernel）
   - 精度校准（FP16/INT8）
   - 内存优化（激活重用）
   - kernel 自动调优（多种实现中选最快）
  ↓
Engine 文件（序列化）
  ↓ 部署时加载
ExecutionContext → enqueue 推理
```

### ONNX 导出

```python
import torch
import torch.onnx

model = SimplePolicy(48, 12)
model.eval()

dummy_input = torch.randn(1, 48)
torch.onnx.export(
    model,
    dummy_input,
    "policy.onnx",
    input_names=["observation"],
    output_names=["action"],
    dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
    opset_version=17,
)
# 导出后用 onnxruntime 或 polygraphy 验证数值一致性。
```

### Engine 构建

TensorRT engine 构建应在离线阶段完成，而不是在机器人启动时。构建过程可能需要几分钟，因为 TensorRT 会尝试多种 kernel 实现并选择最快的。

```bash
# 使用 trtexec 工具从 ONNX 构建 FP16 engine。
trtexec --onnx=policy.onnx \
        --saveEngine=policy_fp16.engine \
        --fp16 \
        --workspace=1024 \
        --verbose
```

engine 文件与 GPU 型号和 TensorRT 版本绑定。在 Jetson 上构建的 engine 不能在桌面 GPU 上运行，反之亦然。因此 CI 或部署脚本需要在目标硬件上重新构建 engine。

### 推理部署骨架

```cpp
// 概念骨架：展示 TensorRT 推理的生命周期管理。
// 真实代码需要处理 builder、network、parser 的创建和销毁。
class TrtInference {
public:
    bool loadEngine(const std::string& engine_path) {
        // 加载序列化的 engine 文件。
        // 创建 IRuntime → 反序列化 → 创建 ICudaEngine → 创建 IExecutionContext。
        // 分配输入输出 GPU 缓冲。
        return true;
    }

    void enqueueAsync(const float* h_input,
                      float* h_output,
                      std::size_t input_bytes,
                      std::size_t output_bytes,
                      cudaStream_t stream) {
        // H2D 拷贝输入 → context->enqueueV3(stream) → D2H 拷贝输出。
        // 全部异步提交到 stream，不阻塞 CPU。
    }

    void synchronize(cudaStream_t stream) {
        // 推理完成后同步，只在需要结果时调用。
        cudaStreamSynchronize(stream);
    }
};
```

### FP16 与 INT8 精度校准

TensorRT 支持降低精度以提高吞吐量。FP16 通常几乎无精度损失，速度提升约 2 倍。INT8 需要校准数据，但速度可提升约 4 倍。

| 精度 | 速度提升 | 精度影响 | 额外工作 |
|------|----------|----------|----------|
| FP32 | 基线 | 无 | 无 |
| FP16 | ~2x | 通常可忽略 | 构建时加 `--fp16` |
| INT8 | ~4x | 需要验证 | 需要校准数据集（代表性输入样本） |

对于机器人策略网络，FP16 通常是安全的默认选择。INT8 需要用训练数据的子集做校准，并验证输出动作的数值差异是否在可接受范围内。

### ⚠️ 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 每次启动都重新构建 engine | 启动时间很长 | engine 构建需要几分钟 | 离线构建，启动时只加载 |
| 编程 | 在 A 卡上构建 engine 在 B 卡上运行 | 加载失败或性能差 | engine 与 GPU 绑定 | 在目标硬件构建 |
| 概念 | INT8 直接使用不校准 | 精度严重下降 | 量化需要代表性数据 | 提供校准数据集 |
| 工程 | ONNX 导出后不验证数值一致性 | 推理结果与 PyTorch 不同 | 导出过程可能引入数值差异 | 用 polygraphy 比较 |

### 练习

1. 将一个策略网络导出为 ONNX，用 `trtexec` 构建 FP16 engine，并比较 LibTorch 和 TensorRT 的推理延迟。
2. 为 TensorRT 推理设计与控制线程的数据交换，要求推理超时时使用上一帧输出。
3. 用 polygraphy 比较 PyTorch、ONNX Runtime 和 TensorRT 三者的输出差异。

---

## 6.17 GPU 内存管理：统一内存 vs 显式传输 ⭐⭐⭐

这一节解决的问题是：统一内存简化了编程，但在机器人系统中什么时候应该使用它，什么时候不该。

### 统一内存的工作原理

CUDA 统一内存（Unified Memory）让 CPU 和 GPU 共享同一个虚拟地址空间。用 `cudaMallocManaged` 分配的内存，CPU 和 GPU 都能直接访问。驱动程序在后台通过页面迁移（page migration）把数据移到需要它的处理器上。

```cpp
#include <cuda_runtime.h>

float* managed_data = nullptr;
// 分配统一内存，CPU 和 GPU 都可以通过同一指针访问。
cudaMallocManaged(&managed_data, n * sizeof(float));

// CPU 直接写入。
for (int i = 0; i < n; ++i) {
    managed_data[i] = static_cast<float>(i);
}

// GPU kernel 直接读取同一指针。
myKernel<<<grid, block>>>(managed_data, n);
cudaDeviceSynchronize();

// CPU 直接读取结果，无需显式 D2H 拷贝。
float result = managed_data[0];
```

### 统一内存 vs 显式传输的权衡

| 维度 | 统一内存 | 显式传输 |
|------|----------|----------|
| 编程复杂度 | 低，无需手动管理传输 | 高，需要管理 H2D/D2H |
| 首次访问延迟 | 页面迁移可能在第一次访问时发生 | 传输时机完全可控 |
| 性能可预测性 | 低，迁移时机取决于驱动 | 高，传输在显式 API 调用时发生 |
| 适合场景 | 原型、稀疏访问、不确定谁需要数据 | 生产系统、实时路径、确定性需求 |
| 调试难度 | 性能问题难定位（隐式迁移） | 传输开销可直接测量 |

反事实地看，如果统一内存的页面迁移完全可预测且零开销，那么显式传输就没有存在意义。但现实中，页面迁移可能在控制循环的任意位置被触发，造成不可预测的延迟尖峰。这对机器人实时路径是不可接受的。

> **本质洞察**：统一内存是编程便利性工具，不是性能优化工具。
> 它消除了"思考数据在哪里"的负担，但也消除了"控制数据何时移动"的能力。
> 对于需要延迟确定性的机器人系统，这种控制权的丧失往往不可接受。

### 内存管理策略总结

| 策略 | 适合 | 不适合 |
|------|------|--------|
| `cudaMalloc` + 显式 `cudaMemcpy` | 生产系统、确定性传输 | 快速原型 |
| `cudaMallocManaged` | 原型验证、稀疏访问模式 | 实时控制路径 |
| `cudaHostAlloc`（pinned memory） | 高频异步传输缓冲 | 大量内存需求 |
| `cudaMallocHost` + `cudaMemcpyAsync` | 流水线化传输 | 小数据低频场景 |

### ⚠️ 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 编程 | 在实时路径中使用统一内存 | 偶发延迟尖峰 | 页面迁移在不可预测时刻发生 | 实时路径用显式传输 |
| 概念 | 统一内存等于共享内存 | 混淆概念 | 统一内存是虚拟地址共享，不是 SM 的 shared memory | 区分两个完全不同的概念 |
| 工程 | 统一内存分配过多 | 系统内存压力 | 驱动需要管理页面映射和迁移 | 控制分配总量 |
| 思维 | 先用统一内存写完再优化 | 重构成本高 | 统一内存和显式传输的代码结构不同 | 对性能敏感路径一开始就用显式传输 |

### 练习

1. 用统一内存和显式传输分别实现同一个 kernel 的数据传输，比较 10000 次迭代的延迟分布（P50/P95/P99）。
2. 在统一内存上使用 `cudaMemPrefetchAsync` 预迁移数据，观察是否能消除首次访问的延迟尖峰。
3. 为一个机器人感知流水线设计内存管理策略：哪些缓冲用 pinned memory，哪些用 device memory，为什么不用统一内存。

---

## 6.18 工程案例：GPU 加速 MPC 与动力学计算 ⭐⭐⭐⭐

这一节解决的问题是：把本章所有知识综合应用到一个完整的 GPU 加速机器人控制管线中。

### 场景：GPU 并行仿真中的 MPC

在强化学习训练中，GPU 并行仿真（如 Isaac Gym / Isaac Lab）同时模拟数千个机器人环境。每个环境需要独立的动力学计算和控制。这是 GPU 计算在机器人中最自然的大规模应用场景。

```text
训练循环（每步）：
  GPU 仿真引擎
    ├── 每个环境独立计算物理（GPU 并行）
    ├── 每个环境独立观测（GPU 并行）
    └── 策略网络批量推理（GPU 并行）
         ↓
  动作 → 仿真引擎执行
```

### GPU 批量 MPC 的数据布局

```text
环境数 N = 2048
状态维度 nx = 13 (位置 3 + 四元数 4 + 速度 3 + 角速度 3)
控制维度 nu = 12 (四足 12 关节)
预测步数 H = 20

数据布局（SoA 风格）：
  states: [N × H × nx] → float[2048][20][13]
  controls: [N × H × nu] → float[2048][20][12]
  costs: [N × H] → float[2048][20]
  gradients: [N × H × nu] → float[2048][20][12]
```

这里的关键设计决策是把环境维度放在最外层。这样一个 kernel 可以用 blockIdx 索引环境，用 threadIdx 索引时间步或状态维度，实现高效的 GPU 并行。

### 批量动力学前向传播

```cuda
#include <cuda_runtime.h>

// 简化的单刚体动力学前向传播。
// 每个线程块处理一个环境，线程处理不同时间步。
__global__ void batchDynamicsForward(
    const float* states,      // [N, H, nx]
    const float* controls,    // [N, H, nu]
    float* next_states,       // [N, H, nx]
    int N, int H, int nx, int nu,
    float dt) {
    const int env = blockIdx.x;
    const int t = threadIdx.x;
    if (env >= N || t >= H) return;

    const int state_idx = (env * H + t) * nx;
    const int ctrl_idx = (env * H + t) * nu;
    const int next_idx = (env * H + t) * nx;

    // 简化示例：线性动力学 x_{t+1} = x_t + f(x_t, u_t) * dt
    // 真实系统中这里应该是完整的刚体动力学方程。
    for (int i = 0; i < nx && i < nu; ++i) {
        next_states[next_idx + i] = states[state_idx + i]
            + controls[ctrl_idx + i] * dt;
    }
    // 超出控制维度的状态保持不变。
    for (int i = nu; i < nx; ++i) {
        next_states[next_idx + i] = states[state_idx + i];
    }
}
```

### 端到端 GPU MPC 管线

```text
数据全部在 GPU 上：
  1. 从仿真引擎获取当前状态 → 已在 GPU
  2. 批量前向动力学 → GPU kernel
  3. 批量代价计算 → GPU kernel
  4. 批量梯度计算 → GPU kernel（或自动微分）
  5. 批量控制更新 → GPU kernel
  6. 最优控制输入 → 留在 GPU，直接送回仿真引擎

CPU 全程不参与！没有 H2D/D2H 传输！
```

这正是 GPU MPC 的理想形态：数据从头到尾留在 GPU。CPU 只负责启动训练循环和记录日志。任何 D2H 传输都会成为瓶颈。

### 从训练到部署：GPU MPC 的现实约束

在真实机器人上部署 GPU MPC 时，情况不同于训练时的大规模并行。

| 维度 | 训练时（GPU 仿真） | 部署时（真实机器人） |
|------|-------------------|---------------------|
| 环境数 | 2048-8192 | 1 |
| GPU 利用率 | 高 | 低（除非批量采样） |
| 数据来源 | GPU 仿真引擎 | CPU 传感器驱动 |
| 延迟要求 | 吞吐优先 | 严格 deadline |
| 数据传输 | 无 | H2D/D2H 每周期一次 |

在真实机器人上使用 GPU MPC 只有在以下条件满足时才有价值：
1. MPC 使用采样法，需要大量并行轨迹评估。
2. 模型复杂，单次前向传播计算量大。
3. GPU 上已有其他模块（如感知网络），数据可以共享。

如果 MPC 是基于梯度的优化且问题规模小，CPU 实现通常更适合。

### 本案例中各章节知识的综合运用

| 本章知识 | 在 GPU MPC 中的体现 |
|----------|-------------------|
| 何时上 GPU（6.1） | 训练时 2048 环境适合，单机器人需要评估 |
| CUDA 同步（6.2） | event 查询 + 降级策略 |
| 内存传输（6.3, 6.8） | pinned memory + 三缓冲流水线 |
| CUDA Graphs（6.4, 6.14） | 固定计算图回放 |
| 推理部署（6.5, 6.15, 6.16） | 策略网络推理 |
| GPU 计算模型（6.7） | 批量矩阵的 warp 和 block 设计 |
| cuBLAS/cuSOLVER（6.13） | 批量 Cholesky 分解 |
| 内存管理（6.17） | 显式传输 + 预分配 |
| CPU 回退（6.12） | GPU 失败时切换 CPU MPC |

### ⚠️ 常见陷阱

| 类型 | 错误做法 | 现象 | 根本原因 | 正确做法 |
|------|----------|------|----------|----------|
| 概念 | 训练时的 GPU MPC 架构直接搬到部署 | 单环境 GPU 利用率极低 | 训练和部署的并行度完全不同 | 评估部署时是否有足够并行度 |
| 编程 | 每个 MPC 步骤单独 launch kernel | launch 开销累积 | 小 kernel 数量多 | 使用 CUDA Graphs 或合并 kernel |
| 工程 | GPU MPC 没有 CPU 回退 | GPU 故障时机器人失控 | 单一控制路径 | 保留 CPU MPC 作为保底 |
| 思维 | 为了用 GPU 而选择采样 MPC | 问题本身适合梯度优化 | 技术选择应服务于问题 | 先选方法再选硬件 |

### 练习

1. （跨章综合题）设计一个 GPU 加速的采样式 MPC 系统，包括数据布局、kernel 映射、CUDA Graphs 捕获、event 查询和 CPU 回退路径。
2. 比较 1024 条轨迹的 GPU 批量前向动力学与 CPU 循环版本的延迟，分析传输开销在总时间中的占比。
3. 为一个四足机器人的实机部署设计 GPU 使用策略：哪些模块在 GPU，哪些在 CPU，边界如何交换数据。

---

## 本章小结

| 知识点 | 关键结论 | 工程动作 |
|--------|----------|----------|
| GPU 选型 | 数据位置和批量决定收益 | 先估算传输和同步成本 |
| CUDA 同步 | launch 异步，读取结果需同步 | 用 stream/event 明确边界 |
| 内存传输 | pinned memory 支持高效异步拷贝 | 只锁定关键缓冲 |
| CUDA Graphs | 固定流程回放更低开销 | 适合批量规划流水线 |
| 推理部署 | TensorRT 快但工程成本高 | 推理线程与控制线程分离 |
| CPU SIMD | 小规模低延迟仍可能更优 | 上 GPU 前先剖析 CPU |
| GPU 计算模型 | 性能由并行度、访存和同步共同决定 | 关注 warp、block、occupancy 和端到端时间 |
| 感知与规划案例 | 图像、点云、批量轨迹最适合 GPU | 数据尽量驻留 GPU，只回传低维结果 |
| 实时边界 | 完成不等于可用 | 用 deadline、freshness 和降级策略约束 |
| 不上 GPU | 小任务、强同步、低并行度应留在 CPU | 先优化算法、布局、SIMD 和批量化 |

## 累积项目：GPU 感知规划加速模块

本章新增模块是 `gpu_planning_accel`。

阶段 1：实现一个深度图滤波 kernel，使用 stream 异步提交。
阶段 2：使用 pinned memory 管理输入输出缓冲。
阶段 3：加入 event 查询，控制线程未等到结果时使用上一帧。
阶段 4：把固定 kernel 序列捕获为 CUDA Graph。
阶段 5：与 CPU SIMD 实现做对比，报告端到端延迟而不只报告 kernel 时间。

## 延伸阅读

| 资料 | 难度 | 阅读目的 |
|------|------|----------|
| CUDA C Programming Guide | ⭐⭐ | 理解执行模型和内存 |
| CUDA Graphs 文档 | ⭐⭐⭐ | 学习固定流水线回放 |
| NVIDIA TensorRT 文档 | ⭐⭐⭐ | 部署低延迟推理 |
| Thrust/CCCL 文档 | ⭐⭐ | 快速构建 GPU 算法 |
| cuRobo 论文与文档 | ⭐⭐⭐ | 学习 GPU 运动规划结构 |

## 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 处理 |
|------|----------|----------|------|
| GPU 版本比 CPU 慢 | 数据太小或传输主导 | 分离测 kernel 和端到端时间 | 批量化或留在 CPU |
| 控制线程偶发卡住 | 隐式同步或全设备同步 | 搜索同步 API 和默认 memcpy | 用 event 查询和 stream |
| `cudaMemcpyAsync` 不异步 | 使用 pageable memory | 检查 host buffer 来源 | 换 pinned memory |
| 结果偶发旧值 | kernel 未完成就读取 | 插入 event 验证 | 明确同步或轮询 |
| CUDA Graph 回放失败 | 捕获流程中有不支持操作 | 检查 capture 错误码 | 简化 graph 或分段捕获 |
| TensorRT 首次推理很慢 | engine 构建或 lazy 初始化 | 分离 configure 和 run | 启动阶段 warm up |
