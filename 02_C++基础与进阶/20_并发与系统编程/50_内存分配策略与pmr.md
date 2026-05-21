# 内存分配策略与 `std::pmr`

> **难度**：⭐⭐⭐⭐ | **建议用时**：2 周 | **前置要求**：C++语言进阶/RAII与智能指针 RAII，C++语言进阶/移动语义与完美转发 移动语义，线程管理与互斥同步-实时约束与高性能数据传递 并发、原子、并行与实时约束

---

## 前置自测

答不出两题以上，建议先复习 RAII、容器扩容、移动语义、实时路径和对象生命周期。
内存分配不是“调用 `new` 得到一块内存”这么简单。
它涉及分配器锁、碎片、缓存局部性、对象生命周期、容器传播规则和实时长尾。

1. `std::vector<T>::push_back` 什么时候会触发重新分配？
2. 为什么 `reserve()` 只能避免容量扩张，不能避免所有分配？
3. `std::pmr::vector<T>` 和 `std::vector<T>` 的元素布局是否不同？
4. `monotonic_buffer_resource` 为什么分配很快？为什么它不适合长期逐个释放对象？
5. `unsynchronized_pool_resource` 和 `synchronized_pool_resource` 的差别是什么？
6. 对象池和内存池有什么区别？
7. 每帧临时点云适合哪类分配策略？长期地图点适合哪类策略？
8. 为什么在 1kHz 实时循环中调用默认分配器会带来长尾风险？
9. `pmr` 能否自动让容器跨线程安全？
10. 如果一个 `pmr::vector` 比普通 `vector` 更慢，可能是什么原因？

---

## 本章目标

学完本章，你将能够：

- 解释动态内存分配在 SLAM 和控制系统中造成长尾、碎片和锁竞争的机制。
- 区分普通 allocator、`std::pmr::polymorphic_allocator`、memory resource、对象池和帧级分配器。
- 使用 `std::pmr::monotonic_buffer_resource` 构造帧级临时内存区域。
- 使用 `std::pmr::unsynchronized_pool_resource` 管理单线程小对象高频分配。
- 判断什么时候需要 `synchronized_pool_resource`，什么时候应避免跨线程共享分配资源。
- 为点云滤波、特征匹配、临时残差块、局部地图更新选择不同内存策略。
- 编写固定容量对象池，并用 RAII 句柄表达对象归还。
- 设计分配次数统计和 benchmark，避免只凭感觉谈性能。
- 完成 Mini SLAM Frame Arena：用 `pmr` 管理一帧内的临时点云、匹配对和残差缓存。

**知识树**：

```text
内存分配策略与 pmr
├── 为什么分配是瓶颈（35.1）
│   ├── malloc 多层架构：系统调用 → 用户态缓存 → 线程本地
│   ├── 最坏情况 vs 平均情况
│   └── 分配策略与生命周期对齐
├── pmr 基础模型（35.2 - 35.3）
│   ├── memory_resource / polymorphic_allocator / pmr 容器
│   ├── 值语义 allocator vs 引用语义 resource
│   └── monotonic_buffer_resource（bump allocation）
├── 池化分配（35.4 - 35.5）
│   ├── pool_resource：按大小类复用
│   ├── 对象池：类型化 + 稳定地址 + RAII 句柄
│   └── 碎片理论（Robson 下界）
├── 工程实践（35.6 - 35.11）
│   ├── 容器传播规则
│   ├── 内存增长监控
│   ├── 多线程 resource 隔离
│   ├── 第三方库边界（Eigen/PCL/ROS2）
│   └── benchmark 方法
└── 决策框架
    └── 生命周期 → 实时性 → 地址稳定性 → 策略选择
```

**本章在课程中的位置**：实时约束与高性能数据传递 已经说明实时路径不应做无界分配。
本章继续回答：如果算法确实需要大量临时对象，怎样把分配行为移到可控边界内。
缓存优化与数据布局 会进一步讨论缓存布局。
本章关注的是“内存从哪里来、什么时候释放、释放成本是否可控”。

---

## 35.1 为什么分配会成为 SLAM 的性能瓶颈 ⭐⭐

> **这一节解决什么问题**：pmr 不是"更快的 allocator"——而是让你控制内存生命周期策略。为什么 SLAM 后端优化时 `malloc` 是瓶颈？

### 动机：为什么 SLAM 后端优化时 malloc 成为瓶颈？

在进入技术细节之前，先理解一个真实的工程场景：SLAM 后端优化（如 Ceres Solver、GTSAM 的 Levenberg-Marquardt 迭代）为什么会被 `malloc` 拖慢？

回顾 SLAM 后端优化的核心循环：每次迭代需要构建雅可比矩阵 $J$、计算 $J^T J$（Hessian 近似）和 $J^T r$（梯度），然后求解线性系统 $H \Delta x = -g$。对于一个包含 1000 个位姿节点和 5000 条约束的因子图，每次迭代需要创建 5000 个残差块对象，每个残差块包含一个小矩阵（如 3x6 或 6x6）和一个残差向量。如果这些临时对象全部通过 `new` 分配，就是 5000 次 `malloc` 调用。

**量化这个问题**：假设每次 `malloc` 的快速路径耗时 50ns（线程本地缓存命中），5000 次就是 0.25ms。看起来不多。但如果其中 1% 的调用（50 次）命中了慢速路径（需要从全局 free list 分配或触发系统调用），每次耗时跳到 5 微秒，这 50 次就额外增加了 0.25ms。再加上 5000 次 `free` 的开销（约 0.15ms），以及碎片化导致的缓存效率下降（L1/L2 cache miss 率上升），总影响可能达到 1-2ms。在 10Hz 的后端优化中（每帧预算 100ms、一次迭代预算约 10-20ms），1-2ms 的分配开销占了 5-20%——这不是微不足道的开销。更关键的是，这 5000 个临时对象在每次迭代结束后全部销毁，下次迭代又重新创建——它们"同生同死"，但默认分配器不知道这一点，仍然在逐个管理每个对象的内存块。

这就是 pmr 要解决的核心问题：**不是让每次分配更快，而是让分配策略与对象的生命周期模式匹配**。如果一批对象同生同死，就不应该逐个分配和释放——应该用一块连续内存统一管理，结束时一次性回收。

### 工程问题：每帧都在创建大量临时对象

一个典型 LiDAR 前端每帧可能创建：

| 临时对象 | 数量级 | 生命周期 |
|----------|--------|----------|
| 去畸变点云 | 10万点 | 当前帧 |
| 下采样点云 | 1万到5万点 | 当前帧 |
| 最近邻索引 | 1万到5万项 | 当前帧 |
| 残差块 | 1万到5万项 | 当前优化迭代 |
| 有效匹配对 | 1万项 | 当前帧 |
| 统计缓存 | 数十个 | 当前帧 |

这些对象有共同特征：

1. 数量大。
2. 生命周期短。
3. 同一帧内一起创建。
4. 帧结束后整体失效。

如果每个容器和小对象都走默认分配器，系统会产生大量 `malloc/free`。
平均耗时可能不高。
但长尾、碎片和锁竞争会影响实时稳定性。

### 反面失败：每个阶段返回新的 `std::vector`

```cpp
std::vector<PointXYZI> removeInvalid(const std::vector<PointXYZI>& input);
std::vector<PointXYZI> deskew(const std::vector<PointXYZI>& input);
std::vector<PointXYZI> voxelFilter(const std::vector<PointXYZI>& input);

PointCloud preprocess(const PointCloud& raw) {
    auto valid = removeInvalid(raw.points);
    auto deskewed = deskew(valid);
    auto filtered = voxelFilter(deskewed);
    return PointCloud{std::move(filtered), raw.stamp_ns};
}
```

这段代码清楚，但可能分配三次大数组。
如果每个函数内部还不断 `push_back`，会继续触发多次扩容。
在桌面离线处理里可能可以接受。
在实时前端里，这些分配会变成尾延迟来源。

### 抽象不变量：分配策略要匹配生命周期

内存策略的第一问题不是“哪个最快”。
而是：

```text
这批对象什么时候一起死亡？
是否需要逐个释放？
是否跨线程？
最大容量能否估计？
是否处在实时路径？
```

不同生命周期对应不同策略：

| 生命周期 | 推荐策略 |
|----------|----------|
| 当前表达式 | 栈对象 |
| 当前函数 | 局部容器 + reserve |
| 当前帧整体释放 | `monotonic_buffer_resource` |
| 高频小对象反复创建销毁 | pool resource 或对象池 |
| 长期地图对象 | 稳定容器、对象池或普通分配器 |
| 跨线程共享 | 明确所有权，避免共享 resource |
| 硬实时循环 | 预分配固定容量 |

### 规则推导：从系统调用到用户态缓存——`malloc` 的完整分配器架构

要理解为什么 `malloc` 在实时路径中是危险的，需要理解它内部的完整架构。`malloc` 不是一个简单函数——它是一个多层缓存系统，每一层都有自己的开销和最坏情况。

**第一层：操作系统接口（brk/mmap）**。进程的堆内存最终来自操作系统。Linux 上有两种方式获取内存：`brk` 系统调用移动进程数据段的末端（连续扩展堆），`mmap` 系统调用映射新的虚拟内存区域（可以在地址空间任意位置创建）。glibc 的 `malloc` 对小分配使用 `brk`（快，但只能连续扩展），对大分配（默认阈值 128KB）使用 `mmap`（灵活，但系统调用开销更大）。关键问题是：**系统调用会陷入内核态**，耗时在微秒到数十微秒级别，而且如果操作系统需要回收页面或压缩内存，可能延迟到毫秒级。更糟糕的是，`mmap` 返回的页面默认是"虚拟的"——第一次写入时才会分配物理页面（demand paging），这一次页错误（page fault）的成本可能达到几十微秒。

**第二层：用户态内存分配器（tcmalloc/jemalloc/glibc ptmalloc）**。为了避免每次 `malloc` 都触发系统调用，所有现代分配器都在用户态维护一个缓存系统。分配器一次从操作系统获取大块内存（通常 64KB-4MB），然后在用户态管理这些内存的分配和回收。不同分配器的策略差异很大：

- **glibc ptmalloc2**（Linux 默认）：使用 arena（竞技场）机制。每个线程尝试使用自己的 arena，减少锁竞争。每个 arena 维护多个 free list（按大小分类的空闲块链表）。小对象从 fastbin（单链表，LIFO，无锁快速路径）分配；中等对象从 smallbin/largebin 分配；大对象直接 `mmap`。问题在于：ptmalloc 的 arena 数量有上限（默认 CPU 核数 * 8），线程数超过 arena 数量时就会发生 arena 竞争——多个线程争抢同一个 arena 的锁。
- **tcmalloc**（Google）：更激进的线程本地缓存。每个线程有一个 thread-local cache，小对象分配完全不需要锁。只有当 thread-local cache 耗尽时才从中央 free list 获取一批对象（此时需要锁）。这让小对象的快速路径耗时降低到约 20-50ns。
- **jemalloc**（Facebook/FreeBSD）：类似思路但更注重碎片控制。使用 slab 分配器理念，按大小类（size class）组织内存页，同一大小类的对象从同一个 slab 分配，减少外部碎片。

**第三层：线程本地缓存**。tcmalloc 和 jemalloc 的核心优化就是线程本地缓存——每个线程维护一个私有的小对象缓存池。分配时先检查本地缓存，如果有空闲对象就直接返回（无锁，约 20ns）。只有缓存为空时才需要访问全局结构（有锁，约 100-500ns）。释放时对象也是先放入本地缓存，满了再批量归还。这就是为什么 `malloc` 在平均情况下很快——大多数分配命中了线程本地缓存。

**那么 `malloc` 的最坏情况发生在什么时候？**

1. **线程本地缓存为空**：需要从全局 arena 获取一批对象，要加锁。
2. **arena 没有合适大小的空闲块**：需要合并碎片或从操作系统获取新内存。
3. **操作系统需要回收内存**：`mmap` 可能触发内核的内存回收机制（kswapd、direct reclaim），延迟可达毫秒级。
4. **首次写入新页面**：触发 page fault，内核分配物理页面，可能需要清零。
5. **内存碎片严重**：长期运行后，空闲块被切碎，分配器需要搜索更长的 free list 或合并更多碎片块。

一次动态分配可能包含上述任何组合。在平均路径上，这些都可能很快。但实时系统关心最坏情况。默认分配器的目标是通用性，不是让某个控制周期的最坏情况可证明。

> 读到这里你可能会问："既然 tcmalloc/jemalloc 这么快，为什么实时路径还要避免 `malloc`？" 因为"快"和"有界"是不同的性质。tcmalloc 的平均分配耗时可能只有 30ns，但当线程本地缓存恰好用完时，一次分配可能跳到 500ns 甚至更多（如果触发了系统调用）。在 1kHz 控制循环的 1ms 预算中，一次 500 微秒的 `malloc` 就吃掉了一半时间。而你无法预测哪一次 `malloc` 会命中这个最坏情况——这正是实时系统不能接受的"不可预测性"。

### 工程边界：`reserve()` 是必要但不充分

`reserve()` 能避免 vector 容量不足时重新分配：

```cpp
std::vector<PointXYZI> filtered;
filtered.reserve(raw.size());
```

但它不能解决：

1. vector 本身首次分配。
2. 多个临时 vector 分别分配。
3. 内部元素如果也分配。
4. map/set 节点分配。
5. 小对象频繁 new/delete。
6. 分配器内部锁和碎片。

所以 `reserve()` 是第一步。
`pmr` 和对象池是进一步把分配来源纳入系统设计。

### 代码验证：计数分配器的最小模型

为了优化分配，先要能观察分配。
可以写一个计数 memory resource：

```cpp
#include <atomic>
#include <cstddef>
#include <memory_resource>

class CountingResource : public std::pmr::memory_resource {
public:
    explicit CountingResource(std::pmr::memory_resource* upstream =
                                  std::pmr::get_default_resource())
        : upstream_(upstream) {}

    std::size_t allocations() const {
        return allocations_.load(std::memory_order_relaxed);
    }

    std::size_t bytes() const {
        return bytes_.load(std::memory_order_relaxed);
    }

private:
    void* do_allocate(std::size_t bytes, std::size_t alignment) override {
        allocations_.fetch_add(1, std::memory_order_relaxed);
        bytes_.fetch_add(bytes, std::memory_order_relaxed);
        return upstream_->allocate(bytes, alignment);
    }

    void do_deallocate(void* p,
                       std::size_t bytes,
                       std::size_t alignment) override {
        upstream_->deallocate(p, bytes, alignment);
    }

    bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override {
        return this == &other;
    }

    std::pmr::memory_resource* upstream_;
    std::atomic<std::size_t> allocations_{0};
    std::atomic<std::size_t> bytes_{0};
};
```

用它包住容器，就能知道某段代码到底分配了几次。
没有测量，优化很容易变成猜测。

> **本质洞察**：内存分配策略的核心问题不是"用哪个分配器最快"，而是**让内存的生命周期和算法的生命周期对齐**。如果一批临时对象同生同死，就应该用同一块内存统一管理；如果对象需要独立创建销毁，就应该用池化分配器复用。分配策略是生命周期分析的自然产物。

内存分配策略和仓库管理是同构问题：`malloc` 就像每次需要存货就去建一个新仓库——灵活但成本高。`reserve` 就像提前租好一个大仓库——省了反复找仓库的时间，但仓库大小要提前估计。`monotonic_buffer_resource` 就像一个"只进不出"的流水线仓库——货物按顺序堆放，不需要逐件取出，一轮结束后整体清空。对象池就像标准化储物柜——格子大小固定，归还后下一个用户立刻可用。

如果不做任何分配优化，直接使用默认分配器处理所有场景会怎样？对于桌面离线处理，通常完全可以接受——现代 `malloc` 实现（如 tcmalloc、jemalloc）在平均路径上非常快。但对于实时路径，问题不在平均速度，而在最坏情况：默认分配器可能在任意时刻触发系统调用（`mmap`/`munmap`）、页错误或内部锁竞争，这些操作的耗时从微秒到毫秒不等，且无法预测。

> ⚠️ **编程陷阱：每帧创建新 `std::vector` 导致反复分配**
> **错误做法**：每帧的预处理函数返回一个新的 `std::vector<PointXYZI>`，函数结束后局部 vector 析构释放内存，下一帧重新分配。
> **现象**：perf 显示大量时间花在 `malloc/free` 上，p99 延迟随运行时间缓慢上升。
> **根本原因**：频繁分配释放导致内存碎片化。碎片化后 `malloc` 需要搜索更长的 free list 或向系统申请新页。
> **正确做法**：复用容器——在帧间保留 `vector` 对象，每帧开始时 `clear()` 而不是重新创建。`clear()` 只修改 size 不释放 capacity。

> 💡 **概念误区：认为 `new`/`delete` 是 O(1) 操作**
> **新手想法**："`malloc` 不就是返回一个指针吗？应该很快。"
> **实际上**：`malloc` 内部需要搜索 free list、可能加锁、可能触发系统调用。其平均耗时通常在 50-200ns，但最坏情况可以达到数百微秒甚至更长。对于 1kHz 控制循环（1ms 预算），一次 500 微秒的 `malloc` 就占用了一半预算。

> 🧠 **思维陷阱：认为"先优化分配器再优化算法"**
> **新手想法**："分配太慢了，我先换成自定义分配器。"
> **实际上**：如果代码每帧分配 100 次，最有效的优化可能不是让每次分配更快，而是减少分配次数——通过容器复用、`reserve`、固定容量结构等手段。只有分配次数已经降到最低后，才值得考虑分配器本身的性能。
> **正确思维**：先用 `CountingResource` 或 profiler 统计分配次数和字节数，找到热点，然后按"减少次数 > 选择策略 > 优化分配器"的顺序推进。

### 练习

1. **[诊断题]** 用 `CountingResource` 包住一段点云预处理代码（去畸变 + 下采样 + 匹配），统计总分配次数和字节数。哪个阶段的分配最多？
2. **[设计题]** 根据对象生命周期分析，为以下数据选择合适的分配策略：(a) 当前帧临时点云；(b) 滑窗内的关键帧列表；(c) 全局地图点；(d) 单次优化迭代的临时雅可比块。
3. **[估算题]** 一个 10Hz LiDAR 前端每帧创建 5 个临时 `std::vector`，每个约 100K 元素。估算每秒的 `malloc/free` 调用次数（考虑扩容）。如果改用帧级 arena，这个数字降为多少？

---

## 35.2 `std::pmr` 的基本模型 ⭐⭐

### 工程问题：容器类型不应被分配策略污染

传统 allocator 是容器类型的一部分：

```cpp
std::vector<Point, MyAllocator<Point>>
```

这意味着换分配器会改变容器类型。
函数签名、类成员、模板推导都会受到影响。

`std::pmr` 的目标是把”容器装什么”和”从哪里分配内存”解耦。
`std::pmr::vector<T>` 的分配策略由运行时的 `memory_resource*` 决定。

### pmr 的设计哲学：为什么 allocator 是值语义而 resource 是引用语义？

`std::pmr` 的设计看似只是”换了一种 allocator”，但它背后有深思熟虑的语义设计——理解这些设计决策对于正确使用 pmr 至关重要。

**C++98 allocator 模型的困境**：C++98 的 allocator 被设计为容器的模板参数，这意味着它是容器类型的一部分。`std::vector<int, MyAlloc<int>>` 和 `std::vector<int, YourAlloc<int>>` 是**不同类型**。你不能把前者赋值给后者，不能在接受前者的函数参数中传入后者。这对库设计是灾难性的——如果一个函数接受 `std::vector<int>`，调用者就不能传入使用自定义分配器的 vector。实际效果是：几乎没有人在标准容器中使用自定义 allocator，因为一旦使用，就必须在所有接口中传播 allocator 类型参数。

**pmr 的解决方案：类型擦除**。`std::pmr::polymorphic_allocator<T>` 内部持有一个 `memory_resource*` 指针。不管底层 resource 是 `monotonic_buffer_resource`、`pool_resource` 还是你自定义的 resource，allocator 的类型始终是 `polymorphic_allocator<T>`。这就是”多态”（polymorphic）的含义——分配策略的多态性通过虚函数实现，而不是通过模板参数。

**为什么 allocator 是值语义？** `polymorphic_allocator` 是一个轻量值对象——它只包含一个 `memory_resource*` 指针。复制一个 allocator 就是复制一个指针，开销极小。这让容器可以自由地持有 allocator 的副本。值语义还有一个重要含义：两个持有相同 `memory_resource*` 的 allocator 被视为”等价”——它们分配的内存可以互相释放。这允许容器在移动和赋值时正确处理分配器兼容性。

**为什么 resource 是引用语义？** `memory_resource` 是一个抽象基类，用户通过继承它来实现自定义分配策略。它是引用语义的——你不能复制一个 `monotonic_buffer_resource`（因为它管理着内部状态：当前指针位置、已分配块列表等），只能通过指针或引用访问它。这个设计分离了两个关注点：allocator 回答”从哪里分配”（通过持有 resource 指针），resource 回答”怎么分配”（通过虚函数实现）。

**所有权模型的关键后果**：因为 allocator 只持有 resource 的**指针**而不是 resource 本身，所以 allocator 不拥有 resource。这意味着：

1. **resource 的生命周期必须覆盖所有使用它的 allocator 和容器**。如果 resource 先于容器析构，容器的后续操作（析构元素、释放内存）会通过悬空指针访问已销毁的 resource——未定义行为。
2. **容器不会在析构时释放 resource**。`pmr::vector` 析构时会释放自己的元素内存（通过 resource 的 `deallocate` 方法），但不会析构或释放 resource 对象本身。resource 的生命周期由外部管理。
3. **多个容器可以共享同一个 resource**。这是 pmr 的核心优势之一——一帧内的所有临时容器可以共享同一个 `monotonic_buffer_resource`，帧结束时一次性释放。

> 读到这里你可能会问：”为什么不让容器通过 `shared_ptr` 管理 resource 的生命周期？” 因为 `shared_ptr` 本身需要原子引用计数操作，而 pmr 的设计目标之一就是在实时路径中避免不必要的开销。用裸指针 + 显式的声明顺序管理生命周期，虽然更需要程序员注意，但零运行时开销。对于硬实时路径，这个权衡是值得的。

### 抽象不变量：allocator 是接口，memory resource 是实际来源

`pmr` 有三层：

| 层 | 作用 |
|----|------|
| `std::pmr::memory_resource` | 抽象内存来源 |
| `std::pmr::polymorphic_allocator<T>` | 把 resource 适配给容器 |
| `std::pmr::vector<T>` 等容器别名 | 使用 polymorphic allocator 的标准容器 |

`pmr::vector<T>` 的元素仍然连续存储。
它和 `std::vector<T>` 的主要区别不是布局，而是分配器来源。

### 代码验证：最小 `pmr::vector`

```cpp
#include <array>
#include <cstddef>
#include <memory_resource>
#include <vector>

struct PointXYZI {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
};

void usePmrVector() {
    // std::byte 只保证 1 字节对齐；作为 arena 底层缓冲时需要给出通用对象对齐。
    alignas(std::max_align_t) std::array<std::byte, 1024 * 1024> buffer{};
    std::pmr::monotonic_buffer_resource arena(buffer.data(), buffer.size());

    std::pmr::vector<PointXYZI> points{&arena};
    points.reserve(10000);
    points.push_back(PointXYZI{1.0f, 2.0f, 3.0f, 0.5f});
}
```

这里 `points` 的元素内存来自 `arena`。
`points` 析构时会调用元素析构。
但 `monotonic_buffer_resource` 不会逐块把内存还给上游。
它会在 resource 析构或 `release()` 时整体释放。

### 工程边界：resource 必须活得比容器久

错误写法：

```cpp
std::pmr::vector<PointXYZI> makeVector() {
    alignas(std::max_align_t) std::array<std::byte, 4096> buffer{};
    std::pmr::monotonic_buffer_resource arena(buffer.data(), buffer.size());
    std::pmr::vector<PointXYZI> points{&arena};
    points.push_back(PointXYZI{});
    return points;
}
```

返回的 `points` 保存了指向 `arena` 的分配器。
但 `arena` 已经析构。
后续 vector 析构或扩容都会使用悬空 resource 指针。

这类错误很隐蔽。
`pmr` 容器不会拥有 resource。
resource 生命周期必须由外部保证。

> ⚠️ **编程陷阱：pmr 容器返回后 resource 已析构（UAF）**
> **错误做法**：在函数内创建 `monotonic_buffer_resource` 和 `pmr::vector`，然后返回这个 vector。
> **现象**：返回的 vector 在后续使用时崩溃，或 TSan 报告 use-after-free。
> **根本原因**：`pmr::vector` 保存了指向 resource 的指针，但 resource 是栈上局部对象，函数返回后已析构。vector 后续的任何操作（析构、扩容、元素访问）都通过悬空指针访问已销毁的 resource。
> **正确做法**：确保 resource 的生命周期严格覆盖所有使用它的容器。最简单的方式是把它们放在同一个对象中，resource 作为成员声明在容器之前（C++ 按声明顺序构造，逆序析构）。

> 💡 **概念误区：认为 `pmr::vector<T>` 和 `std::vector<T>` 布局不同**
> **新手想法**："`pmr::vector` 是特殊容器，元素存储方式可能不一样。"
> **实际上**：`pmr::vector<T>` 就是 `std::vector<T, std::pmr::polymorphic_allocator<T>>`。元素仍然连续存储。区别只在于内存从哪里分配，而不是怎么存储。

### 练习

1. **[调试题]** 下面的代码有什么生命周期问题？如何修复？
```cpp
std::pmr::vector<int> makeVec() {
    std::array<std::byte, 4096> buf{};
    std::pmr::monotonic_buffer_resource arena(buf.data(), buf.size());
    std::pmr::vector<int> v{&arena};
    v.push_back(42);
    return v;  // 问题在哪？
}
```
2. **[设计题]** 设计一个 `FrameWorkspace` 类，把 resource 和多个 pmr 容器放在同一个对象中。说明成员声明顺序为什么重要。

### 规则推导：把 resource 放在拥有者里

更安全的结构是把 resource 和容器放在同一个拥有者对象中，并保证声明顺序：

```cpp
#include <array>
#include <memory_resource>

class FrameWorkspace {
public:
    FrameWorkspace()
        : arena_(buffer_.data(), buffer_.size()),
          points_(&arena_),
          matches_(&arena_) {}

    std::pmr::vector<PointXYZI>& points() {
        return points_;
    }

    std::pmr::vector<Match>& matches() {
        return matches_;
    }

    void clearFrame() {
        // 容器先重新绑定到 arena，再 release 旧帧的整块临时内存。
        points_ = std::pmr::vector<PointXYZI>{&arena_};
        matches_ = std::pmr::vector<Match>{&arena_};
        arena_.release();
    }

private:
    alignas(std::max_align_t) std::array<std::byte, 4 * 1024 * 1024> buffer_{};
    std::pmr::monotonic_buffer_resource arena_;
    std::pmr::vector<PointXYZI> points_;
    std::pmr::vector<Match> matches_;
};
```

成员析构顺序与声明顺序相反。
这里 `points_` 和 `matches_` 先析构，`arena_` 后析构。
这正是我们需要的顺序。

---

## 35.3 `monotonic_buffer_resource`：帧级临时内存 ⭐⭐

### 工程问题：一帧内创建的临时对象通常一起死亡

点云前端常有这种生命周期：

```text
帧开始
    创建去畸变点云
    创建下采样点云
    创建匹配对
    创建残差缓存
    创建临时排序 key
帧结束
    这些临时对象全部失效
```

这非常适合 monotonic 分配。
它只向前移动指针。
不单独释放每个小块。
帧结束整体释放。

### 反面失败：把 monotonic 用作长期容器分配器

```cpp
std::pmr::monotonic_buffer_resource global_arena;
std::pmr::vector<MapPoint> map_points{&global_arena};

void removeBadPoints() {
    eraseBadPoints(map_points);
}
```

`erase` 会析构元素，但 monotonic resource 不会回收对应内存。
长期地图不断增删时，内存只增不减。
这不是泄漏，而是资源策略不匹配生命周期。

monotonic 适合”批量释放”。
不适合”长期反复增删”。

> **本质洞察**：`monotonic_buffer_resource` 的高效来自一个根本性的放弃——**放弃逐个释放的能力，换取分配时只需要移动指针**。这和 arena allocator 在游戏引擎中的使用完全一致：一帧内所有临时对象共享一块内存，帧结束后整块回收。代价是帧内任何单个对象的 `delete` 都不会真正回收内存。

如果把 `monotonic_buffer_resource` 用于长期地图容器会怎样？每次删除地图点时，元素的析构函数会被调用（对象被”逻辑”销毁），但对应的内存不会被回收给 resource。经过数千次增删后，monotonic resource 的指针只会向前移动，占用的内存只会增长，最终可能耗尽 buffer 并向上游 resource 申请更多内存。这不是内存泄漏（resource 析构时会全部释放），但在长期运行的机器人系统中，它会导致 RSS 持续上升直到系统被 OOM killer 终止。

> ⚠️ **编程陷阱：monotonic resource 用于长期容器导致内存只增不减**
> **错误做法**：用全局 `monotonic_buffer_resource` 作为地图点容器的分配来源，并频繁增删地图点。
> **现象**：程序 RSS 持续上升，几小时后被操作系统杀死。
> **根本原因**：monotonic resource 不回收单个 deallocate 的内存。每次 `erase` 析构元素但不释放空间，新 `push_back` 继续消耗新空间。
> **正确做法**：长期增删容器使用 `pool_resource`、普通分配器或对象池。monotonic 只用于帧级”同生同死”的临时数据。

### 练习

1. **[实验题]** 创建一个 `monotonic_buffer_resource`（初始 1MB），反复 `push_back` 然后 `erase` 前半部分元素，观察 resource 实际使用量是否下降。用 `CountingResource` 作为上游记录溢出次数。
2. **[设计题]** 一个 SLAM 系统有两种临时数据：(a) 每帧的去畸变点云（帧结束整体释放）；(b) 滑窗内的关键帧描述子（逐帧增删）。为这两种数据分别选择分配策略并解释原因。

### 抽象不变量：monotonic 的释放单位是整个 resource——指针碰撞（bump allocation）原理

`monotonic_buffer_resource` 之所以快，不是因为它用了什么巧妙的数据结构，而是因为它**把分配操作简化到了理论最小值**——指针碰撞（bump allocation，也叫线性分配或 arena allocation）。

**普通分配器分配一次需要做什么？** 回顾 35.1 节的分析：`malloc` 需要搜索 free list 找到合适大小的空闲块、可能需要拆分块、更新前后块的元数据、可能需要加锁（多线程场景）、可能需要合并相邻空闲块。即使是最优化的 tcmalloc，线程本地缓存命中时也需要更新链表指针和大小类索引。

**bump allocator 分配一次只需要什么？** 一次加法和一次比较：

```text
分配 N 字节：
    new_pointer = current_pointer + align(N)
    if (new_pointer > buffer_end):
        分配失败或向上游申请
    else:
        result = current_pointer
        current_pointer = new_pointer
        return result
```

这就是全部操作。没有 free list 搜索，没有块拆分，没有元数据更新。在单线程场景下，甚至不需要任何原子操作或锁。一次分配的成本是几条 CPU 指令（一次加法、一次对齐调整、一次比较）——约 2-5 纳秒，比 `malloc` 的快速路径（约 20-50ns）快一个数量级。

**量化对比**：回到 35.1 节的 SLAM 后端场景——5000 个临时残差块，每个约 256 字节：

| 分配策略 | 单次分配耗时 | 5000 次分配总耗时 | 释放策略 | 5000 次释放总耗时 | 合计 |
|---------|------------|------------------|---------|------------------|------|
| `malloc`（快速路径） | ~50ns | ~250 微秒 | 逐个 `free` | ~150 微秒 | ~400 微秒 |
| `malloc`（含 1% 慢速） | ~50-5000ns | ~500 微秒 | 逐个 `free` | ~200 微秒 | ~700 微秒 |
| bump allocator | ~3ns | ~15 微秒 | 一次 `release()` | ~0.01 微秒 | ~15 微秒 |

bump allocator 比 `malloc` 快速路径快约 **25 倍**，比含慢速路径的情况快约 **45 倍**。而且 bump allocator 的耗时完全可预测——没有最坏情况的不确定性。对于 10ms 的迭代预算，从 700 微秒降到 15 微秒意味着把分配开销从 7% 降到 0.15%。

**为什么能这么快？因为放弃了逐个释放的能力。** 普通分配器之所以复杂，是因为它要支持任意顺序的 `free`：对象 A 先分配、后释放，对象 B 后分配、先释放——分配器必须跟踪每个块的边界和状态。bump allocator 通过一个根本性的设计取舍消除了这个复杂度：**`deallocate` 是空操作（no-op）**。当你"释放"一个对象时，对象的析构函数会被调用（语义上对象被销毁），但内存不会被回收给 resource——current_pointer 不会后退。内存只有在 `release()` 被调用或 resource 析构时才会**整体回收**——current_pointer 回到 buffer 起始位置。

**这个权衡在什么场景下合理？** 恰好就是 SLAM 帧处理的场景：一帧内创建大量临时对象（去畸变点云、下采样点云、匹配对、残差块），帧结束后这些对象全部失效。它们"同生同死"——不需要逐个释放，一次性回收就够了。这种生命周期模式在游戏引擎（每帧临时对象）、Web 服务器（每请求临时对象）和编译器（每编译单元临时 AST 节点）中都很常见，所以 bump allocator / arena allocator 在这些领域被广泛使用。

`monotonic_buffer_resource` 的心智模型：

```text
buffer begin
    allocated block 1
    allocated block 2
    allocated block 3
current pointer ->
free area
buffer end
```

分配就是移动 current pointer。
单个 deallocate 通常不回收。
`release()` 或 resource 析构时整体回收。

### 代码验证：帧级 arena

```cpp
#include <array>
#include <cstddef>
#include <memory_resource>

class FrameArena {
public:
    FrameArena()
        : resource_(buffer_.data(), buffer_.size()) {}

    std::pmr::memory_resource* resource() {
        return &resource_;
    }

    void reset() {
        resource_.release();
    }

private:
    alignas(std::max_align_t) std::array<std::byte, 8 * 1024 * 1024> buffer_{};
    std::pmr::monotonic_buffer_resource resource_;
};
```

使用：

```cpp
void processFrame(const RawCloud& raw, FrameArena& arena) {
    arena.reset();

    std::pmr::vector<PointXYZI> deskewed{arena.resource()};
    std::pmr::vector<PointXYZI> filtered{arena.resource()};
    std::pmr::vector<Match> matches{arena.resource()};

    deskewed.reserve(raw.size());
    filtered.reserve(raw.size() / 4);
    matches.reserve(raw.size() / 4);

    deskew(raw, deskewed);
    voxelFilter(deskewed, filtered);
    findMatches(filtered, matches);
}
```

`arena.reset()` 必须发生在上一帧所有临时容器都销毁或不再使用之后。
不要把 `deskewed` 的引用存到帧外。

### 工程边界：上游 resource 与溢出策略

如果初始 buffer 不够，`monotonic_buffer_resource` 默认会向上游 resource 继续申请内存。
这对普通程序方便，但对实时路径可能不符合预期。

可以选择：

1. 允许上游分配，记录分配次数。
2. 使用 `null_memory_resource()` 作为上游，让溢出直接失败。
3. 根据最大点数增大 buffer。
4. 在非实时路径处理超大帧。

示例：

```cpp
std::pmr::monotonic_buffer_resource arena(
    buffer.data(),
    buffer.size(),
    std::pmr::null_memory_resource());
```

如果 buffer 不够，会抛出 `std::bad_alloc`。
这比在实时路径偷偷向系统分配更可控。

---

## 35.4 pool resource：小对象反复分配 ⭐⭐

### 内存碎片化的理论基础：为什么 malloc 长期运行会变慢

在讨论 pool resource 之前，需要理解它要解决的核心问题——内存碎片化。碎片化不是一个模糊的概念，而是有精确定义和理论分析的现象。

**外部碎片（external fragmentation）** 指的是空闲内存总量足够，但没有单个连续块能满足分配请求。例如，总共有 10MB 空闲内存，但分散在 1000 个不连续的 10KB 块中——如果请求分配 20KB，没有一个块够大。外部碎片是 `malloc` 最头疼的问题。

**内部碎片（internal fragmentation）** 指的是分配的内存块比实际需要的大。例如，请求 7 字节但分配器给了 8 字节（因为对齐要求或大小类量化）。多出的 1 字节是内部碎片。内部碎片浪费内存但不影响分配成功率。

**碎片化为什么随时间恶化？** 考虑一个反复分配-释放不同大小对象的程序。假设交替分配 16 字节和 64 字节的对象，然后只释放 16 字节的对象。内存中会出现大量 16 字节的"洞"，被 64 字节的占用块隔开。这些洞太小，无法满足新的 64 字节分配请求。随着时间推移，这种"瑞士奶酪"效应越来越严重——空闲内存越来越碎，可用的连续块越来越小。

理论上，Robson (1977) 证明了一个重要的结论：对于任何内存分配策略（不仅仅是特定的分配器实现），如果分配大小在 $[1, M]$ 范围内变化，那么最坏情况下的碎片开销为 $\Omega(M \cdot \log_2 M)$ 倍——也就是说，为了满足所有请求，分配器可能需要的内存是实际使用内存的 $M \cdot \log_2 M$ 倍。这个下界是**所有策略都无法逾越的**。

**pool allocator 如何绕过碎片问题？** 关键洞察是：如果所有对象大小相同，外部碎片就**完全消失**。pool allocator 把内存预先切分成固定大小的块（slot），每个块大小等于对象大小。分配就是从空闲链表取一个块，释放就是把块归还空闲链表。因为所有块大小相同，任何空闲块都能满足任何分配请求——不存在"块太小"的问题。

这就是 `std::pmr::pool_resource` 的核心思想。它维护多个 pool，每个 pool 管理一种大小类（size class）的对象。请求到来时，先找到对应大小类的 pool，然后从该 pool 的空闲链表分配。因为每个 pool 内部的块大小统一，外部碎片被限制在大小类之间的量化误差（内部碎片）上。

**pool 分配的时间复杂度分析**：

| 操作 | malloc (平均) | malloc (最坏) | pool (平均) | pool (最坏) |
|------|-------------|-------------|------------|------------|
| 分配 | ~50ns | 数百微秒 | ~10-20ns | ~10-20ns |
| 释放 | ~30ns | 数十微秒 | ~5-10ns | ~5-10ns |
| 碎片增长 | 随时间增加 | 不可预测 | 无外部碎片 | 固定 |

pool 分配的最坏情况几乎等于平均情况——因为操作就是链表头部的一次指针读取和更新。这种**可预测性**对实时系统比绝对速度更重要。

### 工程问题：并不是所有对象都按帧一起死亡

有些小对象会反复创建和销毁：

1. 特征点对象。
2. 匹配候选。
3. 图优化临时节点。
4. 回环候选记录。
5. 小型消息包装对象。

它们的生命周期不一定完全一致。
monotonic 会浪费内存。
pool resource 更适合。

### 抽象不变量：pool 把相同大小的块复用

池化分配器通常维护多个大小类：

```text
8 bytes pool
16 bytes pool
32 bytes pool
64 bytes pool
...
```

分配时选择合适大小类。
释放时把块放回对应池。
下一次同类分配可以复用。

### `unsynchronized_pool_resource`

`unsynchronized_pool_resource` 不提供内部线程安全。
它适合：

1. 单线程前端临时对象。
2. 每个线程一个独立 resource。
3. 外部已经保证不会并发访问的场景。

```cpp
#include <memory_resource>
#include <vector>

void buildCandidates(const std::vector<Feature>& features) {
    std::pmr::unsynchronized_pool_resource pool;
    std::pmr::vector<Candidate> candidates{&pool};

    for (const Feature& feature : features) {
        for (const auto& match : queryCandidates(feature)) {
            candidates.push_back(match);
        }
    }
}
```

不要把同一个 `unsynchronized_pool_resource` 交给多个线程同时使用。
这不是性能建议，而是正确性要求。

### `synchronized_pool_resource`

`synchronized_pool_resource` 内部同步，适合多个线程共享。
但共享也意味着锁竞争。
如果每个线程都大量分配小对象，通常更好的做法是：

```text
每个线程一个 unsynchronized_pool_resource
线程结束后合并结果
```

只有当对象确实需要跨线程共享分配来源时，才考虑 synchronized 版本。

### 工程边界：pool 解决分配成本，不解决对象生命周期

pool resource 管的是内存块。
对象构造、析构、所有权仍然由容器和代码负责。
如果对象里有指针、文件句柄、GPU 资源，pool 不会自动帮你管理它们。

这也是 RAII 仍然重要的原因。
分配策略只回答“内存从哪里来”。
RAII 回答”资源什么时候释放”。

> ⚠️ **编程陷阱：多线程共享 `unsynchronized_pool_resource`**
> **错误做法**：`std::pmr::unsynchronized_pool_resource pool;` 被多个 OpenMP 线程同时使用。
> **现象**：偶发崩溃，堆损坏，TSan 报大量 data race。
> **根本原因**：`unsynchronized_pool_resource` 顾名思义不做内部同步。多线程同时调用 `allocate`/`deallocate` 会破坏其内部 free list 结构。
> **正确做法**：每个线程使用独立的 `unsynchronized_pool_resource`，或使用 `synchronized_pool_resource`（接受其锁开销）。

> 🧠 **思维陷阱：认为”用了 pmr 就一定更快”**
> **新手想法**：”pmr 是 C++17 的新特性，一定比普通 allocator 快。”
> **实际上**：pmr 通过虚函数分派到底层 resource，这引入了一次间接调用的开销。如果底层 resource 是默认分配器（`new_delete_resource`），pmr 甚至比直接使用 `std::vector` 更慢。pmr 的价值在于让你**选择合适的 resource**（如 monotonic、pool），而不是 pmr 本身自动更快。
> **正确思维**：pmr 是策略框架，不是加速器。加速来自选择正确的 resource，而不是换一个容器类型。

### 练习

1. **[对比题]** 在同一个场景下分别使用 `unsynchronized_pool_resource` 和 `synchronized_pool_resource`，用 benchmark 比较单线程和 8 线程下的分配吞吐量。解释差异原因。
2. **[设计题]** 一个并行点云匹配算法中，每个线程会创建和销毁大量小型 `Candidate` 对象。应该使用全局 `synchronized_pool_resource` 还是每线程独立的 `unsynchronized_pool_resource`？分析各方案的内存占用和性能。

---

## 35.5 对象池：比内存池更具体的所有权模型 ⭐⭐⭐

### 从内存池到对象池：抽象层级的提升

pool resource 管理的是"字节块"——它不关心这些字节会被用来构造什么类型的对象。对象池则更进一步——它管理的是"特定类型的对象实例"。这个抽象层级的提升有几个重要的理论意义。

**类型化池的优势根源：消除分配器的大小类匹配开销**。通用分配器（包括 `pool_resource`）需要为每次分配请求查找"哪个大小类能容纳请求的字节数"。这个查找通常是 $O(1)$（通过位移或表查找），但仍然有分支和间接访问。对象池完全消除了这个步骤——因为所有对象大小完全相同，"查找大小类"退化为"取池中下一个空闲 slot"。

**稳定地址（stable address）的价值**。在 SLAM 中，很多数据结构通过指针或引用互相关联：地图点被多个关键帧引用，关键帧被多个因子引用。如果这些对象存储在 `std::vector` 中，vector 的扩容会导致所有元素地址改变——所有指向这些元素的外部指针都会失效。对象池预先分配固定容量的内存块，对象的地址在整个生命周期内不变。这不是性能优化，而是**正确性保证**——在指针密集的图结构中，稳定地址往往是必需的。

**对象池与 `std::deque` 的对比**。`std::deque` 也提供稳定地址（插入不导致已有元素移动），但它有两个缺点：(1) 释放的 slot 不能直接复用——deque 只能在两端增删，不能在中间释放后复用；(2) 没有容量控制——deque 可以无限增长，不适合实时系统的容量约束需求。对象池通过维护显式的 free list 解决了 slot 复用问题，通过固定容量解决了边界控制问题。

### 工程问题：有些对象需要固定容量和稳定地址

对象池不是通用分配器。
它通常直接管理某种对象：

```text
FixedPool<MapPoint, 100000>
FixedPool<FeatureTrack, 20000>
FixedPool<KeyFrame, 500>
```

对象池的优势：

1. 容量固定。
2. 分配释放 O(1)。
3. 地址可稳定。
4. 溢出策略明确。
5. 容易统计使用量。

代价是类型固定、实现复杂、容量需要设计。

### 反面失败：池满时偷偷 fallback 到 `new`

```cpp
MapPoint* allocate() {
    if (free_list_empty()) {
        return new MapPoint();
    }
    return takeFromPool();
}
```

这破坏了对象池的实时意义。
池满时系统又回到了默认分配器。
更合理的是返回失败，并让上层降级：

```text
不再创建新地图点
提高下采样阈值
跳过非关键帧插入
触发维护线程清理坏点
```

### 代码验证：固定对象池

下面的实现展示了对象池的核心思想：用一个固定大小的数组作为存储，用 free list（空闲链表）跟踪哪些 slot 可用。分配就是从 free list 取一个 slot（O(1)），释放就是把 slot 还回 free list（O(1)）。与 `malloc` 相比，这里没有大小类匹配、没有碎片合并、没有锁——因为所有 slot 大小相同，且对象池通常只在一个线程中使用。注意 `Handle` 类使用 RAII 自动归还 slot，避免手动归还导致的泄漏。

```cpp
#include <array>
#include <cstddef>
#include <new>
#include <optional>
#include <utility>

template <class T, std::size_t Capacity>
class FixedObjectPool {
public:
    FixedObjectPool(const FixedObjectPool&) = delete;
    FixedObjectPool& operator=(const FixedObjectPool&) = delete;
    FixedObjectPool(FixedObjectPool&&) = delete;
    FixedObjectPool& operator=(FixedObjectPool&&) = delete;

    class Handle {
    public:
        Handle() = default;

        Handle(FixedObjectPool* pool, std::size_t index)
            : pool_(pool), index_(index) {}

        ~Handle() {
            reset();
        }

        Handle(const Handle&) = delete;
        Handle& operator=(const Handle&) = delete;

        Handle(Handle&& other) noexcept
            : pool_(std::exchange(other.pool_, nullptr)),
              index_(std::exchange(other.index_, 0)) {}

        Handle& operator=(Handle&& other) noexcept {
            if (this != &other) {
                reset();
                pool_ = std::exchange(other.pool_, nullptr);
                index_ = std::exchange(other.index_, 0);
            }
            return *this;
        }

        T& get() {
            return *pool_->ptr(index_);
        }

        T* operator->() {
            return &get();
        }

        T& operator*() {
            return get();
        }

        explicit operator bool() const {
            return pool_ != nullptr;
        }

    private:
        void reset() {
            if (pool_) {
                // 生命周期边界：
                // Handle 不拥有 pool；pool 必须活得比所有 Handle 久。
                // 否则这里会通过悬空 pool_ 归还对象。
                pool_->destroy(index_);
                pool_ = nullptr;
            }
        }

        FixedObjectPool* pool_ = nullptr;
        std::size_t index_ = 0;
    };

    template <class... Args>
    std::optional<Handle> tryCreate(Args&&... args) {
        if (free_count_ == 0) {
            return std::nullopt;
        }

        const std::size_t index = free_[--free_count_];
        ::new (static_cast<void*>(ptr(index))) T(std::forward<Args>(args)...);
        occupied_[index] = true;
        return Handle(this, index);
    }

    FixedObjectPool() {
        for (std::size_t i = 0; i < Capacity; ++i) {
            free_[i] = Capacity - 1 - i;
        }
        free_count_ = Capacity;
    }

    ~FixedObjectPool() {
        for (std::size_t i = 0; i < Capacity; ++i) {
            if (occupied_[i]) {
                destroy(i);
            }
        }
    }

private:
    using Storage = std::aligned_storage_t<sizeof(T), alignof(T)>;

    T* ptr(std::size_t index) {
        return std::launder(reinterpret_cast<T*>(&storage_[index]));
    }

    void destroy(std::size_t index) {
        ptr(index)->~T();
        occupied_[index] = false;
        free_[free_count_++] = index;
    }

    std::array<Storage, Capacity> storage_{};
    std::array<std::size_t, Capacity> free_{};
    std::array<bool, Capacity> occupied_{};
    std::size_t free_count_ = 0;
};
```

这个对象池是教学实现。
它不是线程安全的。
并发使用时需要外部同步，或为每个线程准备独立池。
对象池本身也禁止拷贝和移动，因为内部原始存储、占用标志和 `Handle` 中的裸 `pool_` 指针共同构成一个不可平凡复制的所有权关系。

### 工程边界：RAII 句柄与长期图结构可能冲突

对象池 `Handle` 析构即归还对象。
这适合临时对象。
但地图点可能被图结构长期引用。
如果需要长期共享，句柄生命周期必须和图结构关系一致。

两种常见设计：

1. 池拥有对象，图里保存稳定 ID 或非拥有指针。
2. 图拥有 RAII 句柄，删除节点时归还对象。

不要让对象池自动回收仍被地图引用的对象。

对象池和内存池的关系，类似于"汽车租赁公司"和"停车场"的关系。停车场（内存池）只管空间：给你一个车位，你走了还回来。汽车租赁公司（对象池）还管车本身：给你一辆初始化好的车，你还的时候要把车恢复到可用状态。停车场不关心你停什么车；租赁公司关心每辆车的状态。

> ⚠️ **编程陷阱：对象池 Handle 被拷贝导致 double-free**
> **错误做法**：意外拷贝了一个对象池 `Handle`，两个 Handle 析构时都归还同一个对象。
> **现象**：同一个槽位被放入 free list 两次。后续两次 `tryCreate` 返回指向同一块内存的 Handle，两个"不同"对象共享内存。
> **根本原因**：Handle 的拷贝语义和移动语义必须正确——应该 delete 拷贝构造和拷贝赋值，只允许移动。
> **正确做法**：`Handle(const Handle&) = delete; Handle& operator=(const Handle&) = delete;`，并正确实现移动构造和移动赋值（源 Handle 置空）。

### 练习

1. **[代码题]** 为 `FixedObjectPool` 增加 `activeCount()` 和 `capacity()` 方法，并写测试验证：创建 3 个对象后 `activeCount()==3`，移动一个 Handle 后仍然是 3，Handle 析构后变为 2。
2. **[设计题]** 一个图优化系统中，地图点通过对象池管理，图中的边持有地图点的非拥有指针。当地图点被池回收时，如何确保所有引用该点的边也被更新或删除？讨论至少两种方案。

---

## 35.6 pmr 容器传播规则与接口设计 ⭐⭐⭐

### 工程问题：容器从哪个 resource 分配，不能靠猜

`pmr` 容器保存一个 `memory_resource*`。
当容器拷贝、移动、赋值时，分配器传播规则会影响结果。
如果不理解，可能出现：

1. 临时容器分配到了默认 resource。
2. 返回值持有了错误 resource。
3. 内层容器没有使用外层 resource。
4. 函数接口丢失 resource 信息。

### 反面失败：外层 pmr，内层普通 vector

```cpp
struct FrameScratch {
    std::pmr::vector<std::vector<Match>> buckets;
};
```

`buckets` 的外层数组来自 pmr。
但每个内层 `std::vector<Match>` 仍使用默认分配器。
如果每个 bucket 都 push 匹配项，仍然会大量默认分配。

应改为：

```cpp
struct FrameScratch {
    std::pmr::vector<std::pmr::vector<Match>> buckets;
};
```

但还要保证内层 vector 构造时也拿到正确 resource。

### 抽象不变量：resource 应沿数据结构向内传播

如果一个结构内部含有多个 pmr 容器，推荐显式传入 resource：

```cpp
struct FrameScratch {
    explicit FrameScratch(std::pmr::memory_resource* resource)
        : points(resource),
          matches(resource),
          buckets(resource) {}

    std::pmr::vector<PointXYZI> points;
    std::pmr::vector<Match> matches;
    std::pmr::vector<std::pmr::vector<Match>> buckets;
};
```

创建内层 vector 时：

```cpp
void resizeBuckets(FrameScratch& scratch,
                   std::size_t n,
                   std::pmr::memory_resource* resource) {
    scratch.buckets.clear();
    scratch.buckets.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        scratch.buckets.emplace_back(resource);
    }
}
```

这样内外层都使用同一 resource。

### 工程边界：不要把短生命周期 resource 泄露到返回值

如果函数内部创建 frame arena，就不要返回依赖它的 pmr 容器。
可选设计：

1. 调用者提供 resource。
2. 函数返回普通 owning 容器。
3. 函数只在回调作用域内使用 pmr 容器。
4. 把 resource 和容器一起封装进返回对象。

示例：

```cpp
struct MatchResult {
    std::shared_ptr<std::pmr::memory_resource> resource;
    std::pmr::vector<Match> matches;
};
```

这个设计让 resource 生命周期覆盖 `matches`。
但引入共享所有权和堆分配。
是否值得，要看结果是否需要跨帧保存。

---

## 35.7 内存增长监控：RSS、容量和高水位 ⭐⭐

### 工程问题：内存问题不总是崩溃，而是慢慢变差

长期运行的机器人系统可能运行数小时。
内存问题常表现为：

1. RSS 持续增长。
2. 分配延迟越来越大。
3. 地图越来越大导致缓存 miss 增加。
4. 后台维护线程越来越慢。
5. 最终被系统杀死或进入 swap。

因此内存策略必须配合监控。

### 反面失败：只检查有没有泄漏

没有传统泄漏，也可能有内存增长问题：

```text
关键帧从不边缘化
地图点只增加不删除
历史点云全保留
monotonic resource 用于长期容器
vector capacity 高水位永不下降
```

这些都不是“忘记 delete”。
但它们会让进程内存持续上升。

### 抽象不变量：区分 size、capacity 和实际 RSS

| 指标 | 含义 |
|------|------|
| `vector.size()` | 当前元素数量 |
| `vector.capacity()` | 已保留容量 |
| resource 已分配字节 | 该资源向上游拿过多少 |
| RSS | 进程实际驻留物理内存 |
| 地图对象数量 | 业务层容量 |

`clear()` 只把 size 变为 0。
通常不释放 capacity。
这对复用很好。
但如果输入峰值很大，capacity 会保持高水位。

### 代码验证：容量监控

```cpp
#include <cstddef>
#include <iostream>
#include <vector>

template <class T>
void printVectorMemory(const std::vector<T>& v, const char* name) {
    const std::size_t size_bytes = v.size() * sizeof(T);
    const std::size_t capacity_bytes = v.capacity() * sizeof(T);
    std::cout << name
              << " size_bytes=" << size_bytes
              << " capacity_bytes=" << capacity_bytes
              << "\n";
}
```

RSS 可以从系统接口读取。
教学项目里可以先记录容器容量和对象数量。
真实部署再接入平台监控。

### 工程边界：`shrink_to_fit` 不是实时操作

`shrink_to_fit` 可能重新分配并移动元素。
它不适合实时路径。
如果需要释放峰值容量，应在非实时维护阶段进行：

```text
检测到长期低负载
暂停相关数据通道
在后台重建容器
替换旧容器
恢复处理
```

内存回收也要设计时机。
不是随手调用一个函数。

### 内存分配策略的完整决策树

到目前为止，本章介绍了多种分配策略。为了帮助在实际工程中快速选择，下面给出一个结构化的决策框架。选择分配策略的核心依据是**对象的生命周期模式**和**所处路径的实时性要求**。

**第一层判断：是否在实时热路径上？**

如果在硬实时路径（如 1kHz 控制循环）上，首选方案是**完全避免运行时分配**——所有内存在初始化阶段分配完毕，运行时只使用固定容量结构（固定大小数组、环形缓冲、预分配对象池）。这是最保守也最可预测的策略。

如果在准实时路径（如 SLAM 前端，允许偶尔的小抖动）上，可以使用 pmr 策略来控制分配行为——关键是确保分配操作的最坏情况耗时可接受。

如果不在实时路径上（后台线程、离线处理），默认分配器通常完全足够。

**第二层判断：对象的生命周期模式是什么？**

| 生命周期模式 | 特征 | 推荐策略 | 理由 |
|------------|------|---------|------|
| 整帧同生同死 | 一帧内创建的所有临时对象在帧结束时一起释放 | `monotonic_buffer_resource` | 分配只需指针碰撞，帧结束一次性释放 |
| 高频创建销毁 | 对象被频繁创建和销毁，但每个对象生命周期不长 | `pool_resource` 或对象池 | 消除碎片，分配释放均为 O(1) |
| 长期持有 | 对象长期存在，偶尔增删 | 普通分配器 + `reserve` | 长期对象不需要频繁分配 |
| 固定数量 | 对象数量有已知上限 | 预分配数组或对象池 | 容量可控，地址稳定 |
| 跨线程共享 | 对象被多个线程访问 | 每线程独立 resource + 合并 | 避免 resource 内部锁竞争 |

**第三层判断：是否需要地址稳定性？**

如果其他数据结构通过指针指向这些对象（如因子图中的指针关联），需要地址稳定——排除 `std::vector`（可能扩容导致地址变化），使用对象池或 `std::deque`。

**常见错误：策略与生命周期不匹配的后果**

| 错误组合 | 后果 |
|---------|------|
| monotonic + 长期增删 | RSS 持续上升 |
| 普通 vector + 硬实时 | 偶发尖峰延迟 |
| unsynchronized pool + 多线程 | 数据竞争崩溃 |
| 全局 synchronized pool + 高频分配 | 锁竞争成为瓶颈 |
| 对象池 + 容量估计不足 | 运行时分配失败 |

---

## 35.8 Mini SLAM Frame Arena

### 工程问题：把本章策略落到一帧处理流程

本章项目为 Mini SLAM 增加帧级内存工作区。
目标：

1. 一帧内临时点云、匹配对、残差缓存都来自 frame arena。
2. 帧结束统一释放临时内存。
3. 长期地图对象不使用 frame arena。
4. 记录每帧分配次数和峰值容量。

### 模块结构

```text
mini_slam_memory/
  include/
    frame_arena.hpp
    frame_workspace.hpp
    counting_resource.hpp
    fixed_object_pool.hpp
    memory_metrics.hpp
  tests/
    test_frame_arena_lifetime.cpp
    test_object_pool.cpp
    test_allocation_count.cpp
```

### `frame_workspace.hpp`

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <memory_resource>

class FrameWorkspace {
public:
    FrameWorkspace()
        : arena_(buffer_.data(),
                 buffer_.size(),
                 std::pmr::null_memory_resource()),
          deskewed_(&arena_),
          filtered_(&arena_),
          matches_(&arena_),
          residuals_(&arena_) {}

    void beginFrame() {
        // 先让旧容器脱离旧 arena 存储，再整体释放上一帧的临时内存。
        deskewed_ = std::pmr::vector<PointXYZI>{&arena_};
        filtered_ = std::pmr::vector<PointXYZI>{&arena_};
        matches_ = std::pmr::vector<Match>{&arena_};
        residuals_ = std::pmr::vector<Residual>{&arena_};
        arena_.release();
    }

    std::pmr::vector<PointXYZI>& deskewed() {
        return deskewed_;
    }

    std::pmr::vector<PointXYZI>& filtered() {
        return filtered_;
    }

    std::pmr::vector<Match>& matches() {
        return matches_;
    }

    std::pmr::vector<Residual>& residuals() {
        return residuals_;
    }

private:
    // std::byte 默认只保证 1 字节对齐；arena 底层缓冲需要显式给出普通类型可用的对齐。
    alignas(std::max_align_t) std::array<std::byte, 8 * 1024 * 1024> buffer_{};
    std::pmr::monotonic_buffer_resource arena_;
    std::pmr::vector<PointXYZI> deskewed_;
    std::pmr::vector<PointXYZI> filtered_;
    std::pmr::vector<Match> matches_;
    std::pmr::vector<Residual> residuals_;
};
```

注意 `beginFrame()` 的顺序。
先让旧容器脱离旧存储，再释放 arena。
更严格的实现可以把每帧临时容器放进局部作用域，避免跨帧持有。

### 帧处理示例

```cpp
void processFrame(const RawCloud& raw, FrameWorkspace& workspace) {
    workspace.beginFrame();

    auto& deskewed = workspace.deskewed();
    auto& filtered = workspace.filtered();
    auto& matches = workspace.matches();
    auto& residuals = workspace.residuals();

    deskewed.reserve(raw.size());
    filtered.reserve(raw.size() / 4);
    matches.reserve(raw.size() / 4);
    residuals.reserve(raw.size() / 4);

    deskew(raw, deskewed);
    voxelFilter(deskewed, filtered);
    findMatches(filtered, matches);
    buildResiduals(matches, residuals);
    solvePose(residuals);
}
```

这个流程没有把临时容器返回到帧外。
如果后续模块需要保存结果，应复制或移动到长期拥有者。
不要保存指向 frame arena 内存的引用。

### 测试：arena 溢出应可见

```cpp
#include <cassert>
#include <exception>

void testArenaOverflow() {
    FrameWorkspace workspace;
    workspace.beginFrame();

    bool failed = false;
    try {
        workspace.deskewed().reserve(1000000000);
    } catch (const std::bad_alloc&) {
        failed = true;
    }

    assert(failed);
}
```

使用 `null_memory_resource` 作为上游后，容量不足会明确失败。
这比静默走系统分配器更适合实时边界测试。

### 验收标准

| 项目 | 标准 |
|------|------|
| 临时容器来源 | 全部来自 `FrameWorkspace` |
| 长期地图对象 | 不依赖 frame arena |
| 溢出策略 | 可观测，不静默系统分配 |
| 分配次数 | 可通过 `CountingResource` 统计 |
| 生命周期 | 不返回 arena 内存引用 |
| 性能对比 | 与普通 vector 版本比较 p50/p99 |

---

## 35.9 多线程中的 resource 隔离 ⭐⭐⭐

### 工程问题：分配器本身也会成为共享资源

并行编程框架 讲过并行框架。
如果并行循环里的每个任务都向同一个分配资源申请内存，分配资源就变成共享热点。
这和多个线程同时写同一个 vector 是同类问题，只是共享点藏在 allocator 后面。

错误直觉是：

```text
我已经用了 pmr，所以分配一定更快。
```

正确问题应该是：

```text
这些线程是否共享同一个 resource？
这个 resource 是否线程安全？
线程安全的代价是否可接受？
能不能每个线程独立分配，最后合并？
```

### 反面失败：并行循环共享 `unsynchronized_pool_resource`

```cpp
std::pmr::unsynchronized_pool_resource pool;
std::pmr::vector<Match> all_matches{&pool};

#pragma omp parallel for
for (int i = 0; i < static_cast<int>(queries.size()); ++i) {
    std::pmr::vector<Match> local{&pool};
    findMatches(queries[static_cast<std::size_t>(i)], local);
    append(all_matches, local);
}
```

这段代码有两个问题：

1. 多个线程同时访问 `unsynchronized_pool_resource`，不安全。
2. 多个线程同时追加 `all_matches`，也不安全。

把默认分配器换成 pmr，并没有改变并发语义。
它只是把内存来源换了。

### 抽象不变量：并行热路径优先线程本地 resource

更合理的结构：

```text
每个线程一个 local resource
每个线程一个 local result
并行结束后合并 result
```

这和 并行编程框架 的线程本地归约同构。
区别只是归约对象从数值变成容器。

### 代码验证：OpenMP 线程本地 arena

```cpp
#include <array>
#include <cstddef>
#include <memory_resource>
#include <omp.h>
#include <vector>

struct ThreadScratch {
    alignas(std::max_align_t) std::array<std::byte, 1024 * 1024> buffer{};
    std::pmr::monotonic_buffer_resource arena;
    std::pmr::vector<Match> matches;

    ThreadScratch()
        : arena(buffer.data(), buffer.size(), std::pmr::null_memory_resource()),
          matches(&arena) {}

    void reset() {
        matches = std::pmr::vector<Match>{&arena};
        arena.release();
    }
};

std::vector<Match> parallelFindMatches(const std::vector<Query>& queries) {
    const int threads = omp_get_max_threads();
    std::vector<ThreadScratch> scratch(static_cast<std::size_t>(threads));

    #pragma omp parallel
    {
        const int tid = omp_get_thread_num();
        ThreadScratch& local = scratch[static_cast<std::size_t>(tid)];
        local.reset();

        #pragma omp for schedule(dynamic, 32)
        for (int i = 0; i < static_cast<int>(queries.size()); ++i) {
            findMatches(queries[static_cast<std::size_t>(i)], local.matches);
        }
    }

    std::vector<Match> merged;
    std::size_t total = 0;
    for (const ThreadScratch& item : scratch) {
        total += item.matches.size();
    }
    merged.reserve(total);
    for (const ThreadScratch& item : scratch) {
        merged.insert(merged.end(), item.matches.begin(), item.matches.end());
    }
    return merged;
}
```

这段示例的关键不是 OpenMP。
关键是每个线程只访问自己的 arena 和 matches。
合并发生在并行区域之后。

### 工程边界：线程本地 arena 的容量也要设计

每线程 1MB，8 线程就是 8MB。
如果每个线程容量都按最坏情况配置，内存可能浪费。
可以考虑：

1. 根据输入规模动态选择线程数。
2. 每线程 arena 只承担小对象，点云大数组另行处理。
3. 对超大帧回退到非实时路径。
4. 记录每线程峰值使用量，调整容量。

`pmr` 让你控制分配来源。
它不会自动帮你做容量规划。

### `synchronized_pool_resource` 的适用边界

如果多个线程确实需要共享 resource，可以使用 `synchronized_pool_resource`。
但它内部同步，仍可能产生竞争。

适合场景：

1. 低频共享小对象。
2. 分配不在实时路径。
3. 实现简单性比极限性能更重要。
4. 对象生命周期跨线程，难以拆成本地 arena。

不适合场景：

1. 每个点、每个残差都分配。
2. 高频控制循环。
3. 线程数多且分配密集。
4. 明明可以局部化，仍共享一个资源。

---

## 35.10 `pmr` 与 Eigen、PCL、ROS2 的边界 ⭐⭐⭐

### 分配器传播的理论困境：为什么自定义分配器在实践中很少被使用

在讨论 pmr 与第三方库的边界之前，有必要理解一个更深层的问题：为什么 C++ 社区在 1998 年就引入了 allocator 机制，但直到 2017 年的 pmr 出现之前，几乎没有人在生产代码中使用自定义 allocator？

**根本原因是"分配器传播"问题**。C++98 的 allocator 是容器类型的一部分，这导致了一个**传染性**问题：如果你的一个数据结构使用了自定义 allocator，那么所有持有它的数据结构也必须感知这个 allocator 类型。考虑一个场景：你想让 SLAM 前端的临时点云使用 arena 分配器。点云是 `std::vector<PointXYZI, ArenaAlloc<PointXYZI>>`。但这个 vector 被存放在一个 `Frame` 结构体中，`Frame` 被存放在 `std::deque<Frame>` 中，`deque` 被存放在 `SlidingWindow` 类中……如果用 C++98 allocator，每一层容器都需要知道 allocator 类型——要么也使用 arena（不合适，因为滑窗是长期数据结构），要么接受类型不匹配（不能互相赋值）。这个"类型传染"让自定义 allocator 在嵌套数据结构中几乎不可用。

**pmr 如何解决这个问题？** `polymorphic_allocator` 通过类型擦除把 allocator 类型统一为一个——不管底层是 monotonic、pool 还是默认 allocator，容器类型都是 `pmr::vector<T>`。这意味着一个函数可以接受任何 pmr 容器，不需要模板化 allocator 类型。"类型传染"被消除了。

但 pmr 只解决了标准库容器的问题。第三方库有自己的内存管理方式，通常不接受外部分配器。

### 工程问题：不是所有库都接受你的分配器

`std::pmr` 对标准库容器很好用。
但机器人项目大量使用第三方库：

1. Eigen。
2. PCL。
3. OpenCV。
4. ROS2 message。
5. Ceres/GTSAM/g2o。
6. CUDA/LibTorch。

这些库不一定直接支持 `std::pmr::memory_resource`。
因此 `pmr` 不是全项目一键替换方案。
它更适合你自己控制的数据结构和标准库容器。

### Eigen 的边界

固定大小 Eigen 类型通常不分配：

```cpp
Eigen::Matrix<double, 6, 6> H;
Eigen::Matrix<double, 6, 1> b;
```

动态大小 Eigen 类型会分配：

```cpp
Eigen::MatrixXd J(rows, cols);
Eigen::VectorXd r(rows);
```

Eigen 有自己的内存管理和对齐规则。
不要简单假设 `pmr` 能接管 Eigen 内部分配。
如果实时路径需要动态矩阵，常见策略是：

1. 固定最大维度，用固定大小或栈上矩阵。
2. 初始化阶段分配动态矩阵，运行时只 resize 到不超过容量的范围。
3. 使用 Eigen 的运行时 malloc 检查辅助定位误分配。
4. 把大规模线性代数放到非实时优化线程。

### PCL 的边界

PCL 点云通常基于 `std::vector<PointT>`。
某些类型和 API 可以通过 allocator 参数定制，某些路径不方便。
更常见的工程做法是：

1. 在自己模块中使用 `pmr::vector<PointT>` 做临时处理。
2. 进入 PCL API 前转换或复用 PCL 容器。
3. 尽量减少在 PCL 内部反复创建临时对象。
4. 对关键热点进行封装，避免每帧重复分配。

如果直接使用 `pcl::PointCloud<PointT>`，先做 `reserve()` 和容器复用。
这仍然是有效优化。
不必为了使用 `pmr` 把所有 PCL 代码重写。

### ROS2 message 的边界

ROS2 消息类型通常有 allocator 模板参数。
但实际项目中还涉及：

1. RMW 实现。
2. loaned message 支持。
3. intra-process communication。
4. publisher/subscription 的生命周期。
5. executor 调度。

因此 ROS2 里的内存优化要结合通信路径。
只在本地算法容器里使用 `pmr`，不能自动让 DDS 传输零拷贝。
实时约束与高性能数据传递 已经强调：loaned message 是否真正零拷贝，以当前 ROS2 和 RMW 官方文档为准。

### 反面失败：为了统一分配器而破坏库接口

有些代码会为了追求“全 pmr”，把清晰的库接口改得很复杂：

```cpp
void process(std::pmr::vector<Point>& points,
             std::pmr::memory_resource* resource,
             ExternalLibraryContext& context);
```

如果外部库内部仍然默认分配，这种接口复杂化收益有限。
更好的策略是先定位分配热点：

1. 哪个函数分配次数最多？
2. 哪个分配在实时路径？
3. 哪个容器容量反复增长？
4. 哪个临时对象生命周期最清楚？

只在这些位置引入 `pmr`。

> ⚠️ **编程陷阱：外层 pmr 容器内嵌普通 vector，分配不一致**
> **错误做法**：`std::pmr::vector<std::vector<Match>> buckets;`——外层用 pmr，内层仍走默认分配器。
> **现象**：`CountingResource` 显示 pmr 分配次数很少，但系统整体 `malloc` 次数仍然很高。
> **根本原因**：内层 `std::vector<Match>` 使用默认分配器。pmr 只影响外层容器的元素存储，不会自动传播到内层容器。
> **正确做法**：使用 `std::pmr::vector<std::pmr::vector<Match>>`，并确保内层容器构造时也传入正确的 resource。

### 练习

1. **[调试题]** `FrameScratch` 中使用了 `std::pmr::vector<std::pmr::vector<Match>> buckets`。写一个函数正确地为 `buckets` 添加 N 个空的内层 vector，每个都使用同一个 resource。
2. **[设计题]** 一个函数内部使用 frame arena 做临时计算，但需要把最终结果返回给调用者。列出至少三种设计方案，分析各自的生命周期风险。

### 抽象不变量：pmr 是局部工具，不是架构目标

架构目标是：

```text
实时路径无不可控分配
临时内存生命周期清楚
长期对象容量受控
跨库边界所有权明确
```

`pmr` 是实现这些目标的工具之一。
对象池、固定容量数组、`reserve()`、容器复用、分区加载也同样重要。

### 代码验证：把 pmr 限定在自有模块内部

```cpp
class FeatureMatcher {
public:
    explicit FeatureMatcher(std::pmr::memory_resource* resource)
        : candidates_(resource),
          accepted_(resource) {}

    MatchView match(const FeatureSet& source, const FeatureSet& target) {
        candidates_.clear();
        accepted_.clear();

        buildCandidates(source, target, candidates_);
        filterCandidates(candidates_, accepted_);
        // MatchView 是非拥有视图，只在下一次 match()、对象析构或 accepted_ 重分配前有效。
        // FeatureMatcher 也不是线程安全对象，不应跨线程共享同一个实例并并发调用 match()。
        return MatchView{accepted_.data(), accepted_.size()};
    }

private:
    std::pmr::vector<Candidate> candidates_;
    std::pmr::vector<Match> accepted_;
};
```

外部只看到 `MatchView`。
内部用 `pmr` 控制临时分配。
这样不会把分配器细节扩散到全系统每个接口。
但 `MatchView` 不是所有权对象。
它适合“立刻消费结果”的同步接口，不适合缓存到异步任务里。

### 工程边界：`pmr::string` 与日志

`std::pmr::string` 可以从自定义 resource 分配。
但这不意味着实时路径可以安全拼接字符串。
字符串格式化仍然有计算成本，也可能触发溢出分配。
实时路径应记录固定结构化事件，而不是依赖 pmr 字符串。

`pmr` 解决内存来源。
它不改变“日志格式化不适合硬实时路径”的事实。

---

## 35.11 benchmark：怎样比较分配策略 ⭐⭐

### 工程问题：分配优化很容易被错误 benchmark 误导

常见错误：

1. 只测一次。
2. 输入规模太小。
3. 没有预热。
4. 把构造 resource 的成本混进每次操作。
5. 没有统计分配次数。
6. 只看平均值，不看 p99。
7. 编译优化级别不一致。

分配策略比较要贴近真实生命周期。
帧级 arena 就应该按“每帧 reset + 多个容器分配 + 帧结束释放”测。
对象池就应该按“反复创建和释放小对象”测。

### 抽象不变量：benchmark 的生命周期要匹配生产代码

错误 benchmark：

```cpp
for (int i = 0; i < repeats; ++i) {
    std::pmr::monotonic_buffer_resource arena;
    std::pmr::vector<Point> points{&arena};
    fill(points);
}
```

如果生产代码中 arena 是长期复用的，这个 benchmark 把 arena 构造成本重复算入。

更贴近帧级工作区：

```cpp
FrameWorkspace workspace;
for (int i = 0; i < repeats; ++i) {
    processFrame(input, workspace);
}
```

这样测到的是每帧 reset 和容器使用成本。

### 代码验证：分配次数对比

```cpp
void compareAllocationCount(const RawCloud& raw) {
    CountingResource counting;
    alignas(std::max_align_t) std::array<std::byte, 8 * 1024 * 1024> buffer{};
    std::pmr::monotonic_buffer_resource arena(
        buffer.data(), buffer.size(), &counting);

    {
        std::pmr::vector<PointXYZI> points{&arena};
        points.reserve(raw.size());
        deskew(raw, points);
    }

    std::cout << "allocations=" << counting.allocations()
              << " bytes=" << counting.bytes()
              << "\n";
}
```

这里的 `counting` 只统计 arena 向上游申请的次数。
固定 buffer 足够大时，理想情况下上游分配次数应为 0。
如果上游分配次数增加，说明 buffer 不够或某个容器没有正确使用预期 resource。

### 结果解释

如果 `pmr` 没有明显加速，不一定说明它没用。
可能原因：

1. 原代码已经 `reserve()` 且分配次数很少。
2. 计算成本远大于分配成本。
3. 输入规模太小。
4. buffer 太小导致频繁上游分配。
5. 多线程共享 resource 产生竞争。
6. benchmark 把构造成本算进热路径。

分配优化的目标有时不是降低平均耗时，而是降低 p99 和分配次数。

---

## 🔧 故障排查手册

| 现象 | 常见原因 | 检查方法 | 修复方向 |
|------|----------|----------|----------|
| pmr 容器崩溃 | resource 已析构 | 检查 resource 生命周期 | 让 resource 活得更久 |
| 内存持续增长 | monotonic 用于长期容器 | 记录 RSS 和对象数 | 改 pool 或普通分配 |
| 仍有大量 malloc | 内层容器没用 pmr | 计数 resource | 向内传播 resource |
| 实时路径偶发 bad_alloc | arena 太小 | 记录峰值容量 | 增大容量或降级 |
| pmr 比普通 vector 慢 | buffer 太小导致上游分配 | 统计上游次数 | 调整 buffer 和 reserve |
| 多线程偶发崩溃 | 共享 unsynchronized resource | TSan | 每线程独立 resource |
| 对象池重复释放 | 句柄移动/析构语义错误 | 单元测试移动句柄 | 修正 RAII handle |
| clear 后内存不降 | capacity 保留 | 打印 capacity | 非实时阶段重建容器 |
| pool 满后延迟升高 | fallback 到系统分配 | 注入容量压力 | 返回失败并降级 |
| 地图点悬空 | 池回收仍被引用对象 | 所有权图检查 | 用 ID 或集中所有权 |

### 策略选择自检清单

在决定使用哪种分配策略前，按下面顺序检查：

1. 这批对象是否能在同一时刻整体释放？
2. 最大容量是否能从传感器频率、点数或窗口大小推导出来？
3. 对象是否需要逐个析构并归还？
4. 分配是否发生在实时路径？
5. 分配是否跨线程？
6. 容器是否有内层容器或字符串字段？
7. 第三方库是否接受自定义 allocator？
8. 如果容量不足，系统应该丢弃、降级、还是失败？
9. 是否有测试能注入容量溢出？
10. 是否记录了分配次数、上游分配字节数和 p99 延迟？

如果前两个问题回答“是”，优先考虑 frame arena。
如果第三个问题回答“是”，优先考虑 pool resource 或对象池。
如果第四个问题回答“是”，避免隐式上游分配。
如果第五个问题回答“是”，优先做线程本地 resource，而不是共享同步 resource。

这个清单比“默认使用 pmr”更重要。
分配策略是生命周期设计的结果，不是代码风格选择。

---

## 35.13 练习与跨章综合题

1. 为 `FrameArena` 增加容量压力测试：输入点数超过预估上限时，要求抛出明确错误或返回降级状态，而不是偷偷走系统分配。
2. 用 `CountingResource` 比较普通 `std::vector`、预先 `reserve()` 的 `std::vector` 和 `std::pmr::vector` 的上游分配次数。
3. 修改 `FixedObjectPool`，增加 `activeCount()` 和 `capacity()`，并写测试验证移动 `Handle` 不会重复释放。
4. 设计一个线程本地 scratch 方案：每个工作线程有自己的 arena，最后合并匹配结果。说明为什么不能共享一个 `unsynchronized_pool_resource`。
5. 跨章综合题：结合 实时约束与高性能数据传递 的实时路径要求和 软件工程/代码质量测试与调试 的测试方法，为点云前端设计一组“无动态分配”门禁。至少包含接口限制、容量溢出测试、benchmark 和故障日志字段。

这些练习的目标是把“分配策略”落实到生命周期、容量和错误处理，而不是只替换容器类型。

---

## 35.14 本章小结

内存分配策略的核心不是“把 `std::vector` 换成 `pmr::vector`”。
真正要做的是让内存生命周期和算法生命周期一致。

本章主线可以总结为：

1. 高频实时路径不应依赖默认分配器的长尾行为。
2. 当前帧临时对象适合 `monotonic_buffer_resource`。
3. 高频小对象适合 pool resource 或对象池。
4. 长期地图对象不适合帧级 monotonic arena。
5. `pmr` 容器不拥有 resource，resource 生命周期必须显式设计。
6. 内层容器也要传递 resource，否则默认分配仍会出现。
7. 优化分配前先统计分配次数和容量高水位。

如果说 实时约束与高性能数据传递 解决的是“实时路径不能做什么”，本章解决的是“确实需要临时内存时应该从哪里来”。
下一章会继续向下走到 CPU 缓存：即使分配次数降下来了，数据布局仍然可能决定系统是否稳定高速。

---

## 延伸阅读

1. **C++ 标准库 `std::pmr` 文档（C++17/20）** ⭐⭐——重点阅读 `memory_resource`、`monotonic_buffer_resource`、`pool_resource` 的精确语义与线程安全保证。
2. **Pablo Halpern, "Allocators: The Good Parts", CppCon 2017** ⭐⭐——pmr 设计者之一的演讲，解释了 polymorphic allocator 解决的核心问题。
3. **Robson, "Worst Case Fragmentation of First Fit and Best Fit", Computer Journal 1977** ⭐⭐⭐⭐——内存碎片的理论下界证明，理解为什么 pool allocator 能绕过碎片问题。
4. **C++ Core Guidelines R 系列（资源管理）和 P 系列（性能）** ⭐——RAII、`unique_ptr`/`shared_ptr` 与自定义 allocator 的最佳实践。
5. **游戏引擎 frame allocator / arena allocator 设计资料** ⭐⭐——Dice/EA 的 Frostbite 引擎、Epic 的 UE 线性分配器等，与 SLAM 的帧级 arena 同构。
6. **Linux 内存观测工具：`/proc/self/status`、`pmap`、`perf`、heap profiling** ⭐⭐——实际定位 RSS 增长和分配热点的工具链。

---

## 35.15 分配器感知的容器设计模式 ⭐⭐⭐

### 工程问题：第三方库不支持自定义分配器

pmr 容器在项目内部可以统一使用，但实际系统中经常需要与不接受分配器参数的第三方库交互。PCL 的 `pcl::PointCloud<T>` 内部使用标准 `std::vector`，ROS2 消息类型使用 `rosidl_runtime_cpp::BoundedVector`，Eigen 的 `MatrixXd` 使用自己的内存分配机制。在这些边界上，pmr 的优势无法直接穿透。

处理这个问题的正确策略不是试图把 pmr 强推到所有组件中，而是在可控边界内使用 pmr，在库交互边界做显式转换。

```cpp
// 内部处理使用 pmr::vector，避免实时路径 malloc
void processFrame(const std::vector<Eigen::Vector3d>& pcl_cloud,
                  std::pmr::memory_resource* scratch) {
    // 从 PCL 输入转换到 pmr 容器
    std::pmr::vector<Eigen::Vector3d> working_set(scratch);
    working_set.assign(pcl_cloud.begin(), pcl_cloud.end());

    // 内部处理：所有临时分配走 scratch arena
    std::pmr::vector<int> indices(scratch);
    std::pmr::vector<float> distances(scratch);
    // ... 处理逻辑 ...

    // 结果写回标准容器供 PCL/ROS2 使用
    result_cloud.assign(working_set.begin(), working_set.end());
}
```

这种"外层标准容器 + 内层 pmr 容器"的模式在实际项目中很常见。它的开销是边界处的一次数据拷贝，但内层的所有临时分配都走 arena，避免了实时路径上的 malloc。如果转换开销过大（如百万点云），可以考虑在非实时阶段完成格式转换，实时路径中直接操作 pmr 容器。

### 分配器传播规则的工程影响

pmr 容器有一个重要的语义细节：分配器**不随拷贝传播**。当你拷贝一个 `pmr::vector` 时，新容器使用的是默认 resource，而非源容器的 resource。

```cpp
std::pmr::monotonic_buffer_resource arena(1024 * 1024);
std::pmr::vector<int> v1(&arena);
v1.push_back(42);

auto v2 = v1;  // v2 使用 std::pmr::get_default_resource()，不是 arena！
// v2 的后续分配走系统 malloc，可能在实时路径引入延迟
```

这是 pmr 设计的有意选择——`allocator_traits` 的 `propagate_on_container_copy_assignment` 对 pmr 返回 `false`。理由是：拷贝后的容器应当独立于源容器的 resource 生命周期，否则源 resource 析构后，所有拷贝都悬空。

工程上的应对方式是：在需要保持同一 resource 时使用移动而非拷贝；在必须拷贝时显式传递目标 resource。

```cpp
// 显式传递 resource 的拷贝
std::pmr::vector<int> v3(v1.begin(), v1.end(), &arena);  // v3 也使用 arena
```

### 内存碎片的量化诊断

内存碎片是长期运行系统的隐形杀手，但它很难直接观测。以下方法可以间接诊断碎片程度：

| 指标 | 含义 | 获取方式 |
|------|------|---------|
| RSS 增长率 | 进程实际占用的物理内存随时间增长 | `/proc/self/status` 中的 `VmRSS` |
| 分配/释放比率 | 分配次数远大于释放次数说明可能有泄漏 | `CountingResource` 统计 |
| 高水位差 | 活跃分配量远小于 RSS 说明碎片严重 | 分配器统计 vs RSS 对比 |
| malloc_info | glibc 提供的分配器内部状态 | `malloc_info(0, stdout)` 输出 XML |

当 RSS 持续增长但活跃对象数量稳定时，最可能的原因是碎片。解决方案是按本章介绍的策略做分配器替换：高频短期对象用 monotonic arena，固定大小对象用 pool，长期对象用标准分配器但做容量预留。

> **反事实推理**：如果 C++ 标准库不提供 pmr，工程师能怎么做？
> 历史上的做法是手写内存池或使用第三方分配器（如 jemalloc、tcmalloc）。
> 这些通用分配器优化了多线程争用和碎片，但无法像 pmr 那样按算法生命周期定制策略。
> pmr 的价值不是比 jemalloc 更快——而是让"分配策略"成为可编程的、与算法结构匹配的设计决策。

---

## 35.16 pmr 与 C++23/26 的演进方向 ⭐⭐⭐

C++23 和 C++26 对内存管理引入了若干相关改进：

**`std::generator`（C++23）**：协程生成器支持自定义分配器，pmr 的协程帧可以分配在 arena 上，避免协程创建时的堆分配。这对实时路径中的惰性数据流很有价值——比如一个点云过滤管线可以用协程表达，且帧分配走 monotonic arena。

**`std::inplace_vector`（C++26 提案 P0843）**：固定最大容量的栈上 vector，完全避免堆分配。对于已知上限的小型容器（如 ICP 迭代中的对应点对），`inplace_vector` 是比 pmr 更简洁的选择。它填补了 `std::array`（固定大小）和 `std::vector`（可变大小但堆分配）之间的空白。

**`std::hive`（C++26 提案 P0447）**：稳定地址的无序容器，专为频繁插入/删除设计。在 SLAM 地图中管理动态 MapPoint 集合时，`std::hive` 可能比 `std::vector<std::unique_ptr<MapPoint>>` 更适合——它保证元素地址不因其他元素的插入/删除而失效。

这些新组件不是 pmr 的替代品，而是互补工具。选择标准始终是生命周期匹配：

- 容量编译期已知且小 → `std::array` 或 `std::inplace_vector`
- 当前帧临时数据 → `pmr::vector` + monotonic arena
- 长期动态集合 → `std::hive` 或标准容器 + 预分配
- 高频固定大小对象 → pool resource 或对象池

---

## 35.17 tcmalloc 和 jemalloc 与 pmr 的关系 ⭐⭐

### 通用分配器替换 vs 算法级定制

在讨论 pmr 之前，很多项目通过链接时替换系统 `malloc` 来获得性能提升。Google 的 tcmalloc 和 Facebook 的 jemalloc 是两个最流行的替代分配器。它们通过线程本地缓存和智能的 size class 分配策略，减少了多线程环境下的锁争用和碎片。

| 分配器 | 核心策略 | 优势 | 局限 |
|--------|---------|------|------|
| glibc malloc | 单 arena + per-thread arena | 通用性好 | 高并发下争用，碎片随时间增长 |
| tcmalloc | 线程本地缓存 + 中心堆 + 页堆 | 小对象分配极快 | 大型对象仍走通用路径 |
| jemalloc | 多 arena + slab 分配 | 碎片控制好，可配置 | 配置复杂度高 |
| pmr | 程序员按场景选择策略 | 可以精确匹配算法生命周期 | 需要手动管理 resource 生命周期 |

通用分配器和 pmr 解决的问题不同。tcmalloc/jemalloc 优化的是"所有 `malloc` 调用的平均性能"，而 pmr 解决的是"特定代码路径的最坏情况延迟"。对于实时机器人系统，两者可以组合使用：全局替换 tcmalloc 改善非实时路径的平均性能，在实时路径上使用 pmr arena 消除最坏情况的 malloc 延迟。

```bash
# 链接时替换——对应用代码透明
# 方式一：CMake 链接
target_link_libraries(my_slam PRIVATE tcmalloc)

# 方式二：LD_PRELOAD（不修改构建系统）
LD_PRELOAD=/usr/lib/libtcmalloc.so ./my_slam
```

通用分配器替换是"零成本"的优化——不需要修改任何源代码。但它无法消除实时路径上的长尾延迟，因为即使是 tcmalloc，分配大对象或跨线程回收时仍然可能出现几百微秒的延迟。pmr 通过预分配彻底消除了这些不可控路径。

### 分配器选择的决策树

```
需要优化分配性能？
├── 非实时代码？
│   ├── 全局替换 tcmalloc/jemalloc（零改动，即时收益）
│   └── 如果仍有瓶颈 → profile 后按热点做针对性 pmr
├── 实时路径？
│   ├── 对象生命周期 = 当前帧？→ monotonic arena
│   ├── 对象生命周期 > 帧但 < 窗口？→ pool resource
│   ├── 对象生命周期 = 全程？→ 预分配 + reserve
│   └── 不确定？→ 先用 CountingResource 统计分配模式
└── 嵌入式（无 tcmalloc/jemalloc）？
    └── 自定义固定大小分配器或裸 arena
```

> **类比**：tcmalloc 像一个高效的通用快递公司——无论寄什么都比普通快递快。
> pmr 像你在仓库里设置了专门的传送带——快递公司根本不需要介入，包裹直接从源头到目的地。
> 对于大多数包裹，快递公司够好了。但对于"必须在 1 毫秒内到达"的紧急包裹，专用传送带才能保证时效。

---

## 35.18 分配策略的自动化测试 ⭐⭐⭐

### 工程问题：分配策略的正确性难以通过代码审查验证

pmr 的 resource 生命周期、容量边界和传播规则很难通过阅读代码来验证。需要专门的测试手段来确保分配策略在各种边界条件下正确工作。

### 关键测试模式

**1. 容量压力测试**：验证 arena 容量不足时的行为。

```cpp
TEST(FrameArena, OverflowBehavior) {
    char buffer[1024];  // 故意设置极小的 arena
    std::pmr::monotonic_buffer_resource arena(buffer, sizeof(buffer));

    std::pmr::vector<double> vec(&arena);
    // 当 arena 耗尽时，上游 resource（默认 new_delete_resource）接管
    // 在实时路径中，这是不可接受的——应该触发明确的错误
    EXPECT_THROW({
        for (int i = 0; i < 10000; ++i) vec.push_back(i);
    }, std::bad_alloc);
    // 注意：默认行为是退到系统分配而非抛异常
    // 要测试溢出失败，需要自定义 upstream 为 null_memory_resource
}
```

**2. 零分配门禁测试**：验证实时路径确实没有动态分配。

```cpp
// CountingResource 用于审计分配行为
class CountingResource : public std::pmr::memory_resource {
    std::atomic<int> alloc_count_{0};
    std::pmr::memory_resource* upstream_;

protected:
    void* do_allocate(std::size_t bytes, std::size_t align) override {
        ++alloc_count_;
        return upstream_->allocate(bytes, align);
    }
    void do_deallocate(void* p, std::size_t bytes, std::size_t align) override {
        upstream_->deallocate(p, bytes, align);
    }
    bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override {
        return this == &other;
    }

public:
    explicit CountingResource(std::pmr::memory_resource* upstream)
        : upstream_(upstream) {}
    int count() const { return alloc_count_.load(); }
};

TEST(RealtimePath, ZeroAllocation) {
    CountingResource counter(std::pmr::null_memory_resource());
    // 设为默认 resource——任何未经 arena 的分配都会被捕获
    std::pmr::set_default_resource(&counter);

    // 初始化阶段完成所有预分配
    auto pipeline = createPipeline();

    // 模拟 100 帧实时处理
    for (int frame = 0; frame < 100; ++frame) {
        pipeline.processFrame(testData[frame]);
    }

    EXPECT_EQ(counter.count(), 0) << "实时路径存在未预期的动态分配";
}
```

**3. 生命周期测试**：验证 resource 析构后没有悬空访问。

用 AddressSanitizer（ASan）配合自定义 resource 可以检测 use-after-free。在 resource 析构时将管理的内存填充为特征值（如 `0xDEADBEEF`），后续访问会被 ASan 或值检查捕获。

这些测试应当作为 CI 的一部分持续运行，防止后续代码修改意外引入实时路径上的动态分配。

### 故障排查手册（补充条目）

| 症状 | 可能原因 | 排查步骤 | 处理 |
|------|----------|----------|------|
| pmr 容器在多线程下偶发数据损坏 | 多线程共享同一 `unsynchronized_pool_resource` | 用 TSan 检测，检查 resource 是否跨线程共享 | 每个线程使用独立的 resource 实例 |
| arena 容量估算不准导致生产环境溢出 | 测试数据规模小于生产环境 | 在生产环境录制峰值容量，用 CountingResource 统计 | 按 p99 峰值的 2 倍预留容量 |
| 切换到 pmr 后内存占用反而增大 | arena 预留了固定大小但实际使用量远小于预留量 | 比较 pmr 和标准分配器下的 RSS | 减小预留量或使用按需增长的 pool |
| pmr 容器析构时性能差 | `monotonic_buffer_resource` 不逐个析构元素中的非平凡类型 | 检查容器元素是否有非平凡析构函数 | 对非平凡类型使用 `pool_resource` 而非 `monotonic` |
| 分配器不匹配导致容器赋值失败 | 两个 pmr 容器使用不同的 resource | 检查赋值两侧的 `get_allocator().resource()` 是否相同 | 使用 `swap`（要求相同 resource）或显式构造新容器 |
