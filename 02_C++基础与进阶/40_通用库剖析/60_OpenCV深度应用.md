# OpenCV 在 SLAM 中的深度应用

> **难度**：⭐⭐～⭐⭐⭐ | **建议用时**：2周 | **前置要求**：通用库·Eigen Eigen深入

---

## 前置自测

> 📋 答不出 ≥ 2 题 → 先回 通用库·Eigen 复习 Eigen 基础

1. `Eigen::Matrix3d` 是行优先还是列优先存储？为什么 Eigen 选择这种默认方式？
2. 什么是 Shi-Tomasi 角点？它与 Harris 角点的评分函数有什么区别？
3. 光流法的亮度恒定假设（Brightness Constancy Assumption）用数学如何表达？
4. 本质矩阵 $E$ 和基础矩阵 $F$ 的关系是什么？各自有几个自由度？
5. RANSAC 的基本思想是什么？为什么多视图几何中几乎必须使用 RANSAC？

---

## 知识树

```
OpenCV 在 SLAM 中的深度应用
├── 数据基础
│   └── 28.1  cv::Mat 核心与 Eigen 互操作（内存模型、零拷贝）
├── 视觉前端
│   ├── 28.2  图像预处理（CLAHE、高斯模糊、金字塔）
│   ├── 28.3  特征检测（FAST、Shi-Tomasi、ORB）
│   └── 28.4  光流跟踪（LK、setMask 均匀化）
├── 几何与标定
│   ├── 28.5  多视图几何（F/E 矩阵、PnP、三角化、RANSAC）
│   └── 28.6  畸变校正（相机模型、去畸变）
├── 深度学习
│   ├── 28.7  DNN 模块与深度学习特征
│   └── 28.12 DNN 推理部署进阶
└── 工程实践
    ├── 28.8  SLAM 代码精读（ORB-SLAM3 / VINS-Mono）
    ├── 28.9  实战
    ├── 28.10 视觉前端工程边界与验证清单
    └── 28.11 ArUco/AprilTag 标记检测
```

---

## 本章目标

学完本章，你将能够：
- 理解 `cv::Mat` 的内存模型，并在 SLAM 工程中实现 `cv::Mat` 与 `Eigen` 矩阵的零拷贝互转
- 复现 ORB-SLAM3 和 VINS-Mono 中的图像预处理流水线（CLAHE、高斯模糊、金字塔构建）
- 掌握 FAST 角点检测和 Shi-Tomasi 角点检测在 SLAM 前端的具体用法与调参策略
- 实现 VINS-Mono 风格的稀疏光流跟踪前端，包括 `setMask` 均匀化机制
- 使用 `findFundamentalMat`、`findEssentialMat`、`solvePnPRansac`、`triangulatePoints` 完成完整的多视图几何流水线

---

## 28.1 cv::Mat 核心与 Eigen 互操作 ⭐⭐

> 本节解决的问题：SLAM 系统中，OpenCV 负责图像处理，Eigen 负责线性代数运算——两者的数据如何高效互通？

### 动机：为什么必须理解 cv::Mat 的内存模型

在任何视觉 SLAM 系统中，你一定会遇到这样的场景：OpenCV 的 `cv::Mat` 存储了相机图像或特征点坐标，而后端优化需要用 Eigen 矩阵做矩阵运算。如果你不理解两者的内存布局差异，数据转换时就会踩入"行优先 vs 列优先"、"浅拷贝 vs 深拷贝"的陷阱，轻则程序崩溃，重则数值静默错误——后者在 SLAM 中尤为致命，因为一个微小的旋转矩阵转置错误可能在几百帧后累积成巨大的轨迹漂移。

### cv::Mat 的内存布局

`cv::Mat` 是 OpenCV 中最核心的数据结构，几乎所有图像数据和矩阵数据都通过它承载。要正确使用它，必须理解四个关键属性：

| 属性 | 含义 | 典型值 |
|------|------|--------|
| `data` | 指向第一个像素的 `uchar*` 指针 | 堆上分配的连续内存 |
| `rows`, `cols` | 矩阵的行数和列数 | 图像: 480×640 |
| `step` | 每行数据占用的**字节数**（含对齐填充） | 640×3=1920 或 1920+padding |
| `type()` | 数据类型编码（深度+通道数） | `CV_8UC3`=16, `CV_64FC1`=6 |

**数据类型编码规则**：`CV_{位深}{S/U/F}C{通道数}`。例如 `CV_8UC3` 表示 8 位无符号整数、3 通道（彩色图像）；`CV_64FC1` 表示 64 位浮点数、单通道（对应 `double` 矩阵）。

SLAM 中最常用的类型对照表：

| OpenCV 类型 | C++ 原生类型 | Eigen 对应类型 | SLAM 用途 |
|------------|-------------|---------------|----------|
| `CV_8UC1` | `unsigned char` | — | 灰度图像 |
| `CV_8UC3` | `unsigned char[3]` | — | BGR 彩色图像 |
| `CV_32FC1` | `float` | `Eigen::MatrixXf` | 光流计算中间结果 |
| `CV_64FC1` | `double` | `Eigen::MatrixXd` | 位姿矩阵、本质矩阵 |

**`step` 与内存连续性**：`step` 不一定等于 `cols × elemSize()`。当 `cv::Mat` 是另一个矩阵的 ROI（感兴趣区域）时，`step` 会大于实际列数所需的字节——因为 ROI 的每行之间夹着原始矩阵的其他列。用 `isContinuous()` 检查内存是否连续。这一点在向 Eigen 传递数据时至关重要：非连续的 `cv::Mat` 不能直接用 `Eigen::Map` 映射。

**像素寻址公式**：对于一个 `type()` 为 `CV_<depth>C<n>` 的 `cv::Mat`，第 `(i, j)` 个像素在内存中的地址为：

$$\text{addr}(M_{i,j}) = M.\text{data} + M.\text{step}[0] \times i + M.\text{step}[1] \times j$$

其中 `step[0]` 是行步长（字节），`step[1]` 是单个像素的字节数（`elemSize()`）。对于连续矩阵，`step[0] == cols * step[1]`；对于 ROI 子矩阵，`step[0] > cols * step[1]`。

### 浅拷贝与深拷贝：SLAM 中最隐蔽的 Bug 来源

`cv::Mat` 的赋值运算符和拷贝构造函数都执行**浅拷贝**——它们只复制矩阵头（维度、类型、指针等元数据），不复制像素数据。多个 `cv::Mat` 可以指向同一块数据内存，通过引用计数管理生命周期。

```cpp
// ===== 浅拷贝（默认行为）=====
cv::Mat img1 = cv::imread("frame_001.png", cv::IMREAD_GRAYSCALE);
cv::Mat img2 = img1;           // 浅拷贝！img2 和 img1 共享同一块内存
img2.at<uchar>(0, 0) = 255;   // 修改 img2 会同时修改 img1！

// ===== 深拷贝 =====
cv::Mat img3 = img1.clone();   // 深拷贝：分配新内存，复制所有像素
cv::Mat img4;
img1.copyTo(img4);             // 另一种深拷贝方式

// ===== ROI：浅拷贝的特殊形式 =====
cv::Mat roi = img1(cv::Rect(100, 100, 200, 200));  // ROI 也是浅拷贝
// roi 的 data 指向 img1 内部，step = img1.step（而非 200×elemSize）
// roi.isContinuous() == false
```

**为什么 OpenCV 选择浅拷贝作为默认行为？** 因为图像数据通常很大（一张 1080p 灰度图 = 2 MB），如果每次赋值都深拷贝，传参和返回值的开销会无法接受。浅拷贝 + 引用计数是一种高效的所有权共享机制，类似于 `std::shared_ptr`。

**为什么 SLAM 中这很重要？** VINS-Mono 的 `feature_tracker.cpp` 在处理每帧图像时，会将当前帧赋值给 `prev_img`，然后接收新帧到 `cur_img`。如果不理解浅拷贝机制，可能导致 `prev_img` 被新帧覆盖，光流跟踪失败。正确做法是：

```cpp
// ❌ 错误：prev_img 和 cur_img 指向同一内存
prev_img = cur_img;
cur_img = new_frame;  // prev_img 也变成了 new_frame！

// ✅ 正确：深拷贝保存上一帧
prev_img = cur_img.clone();
cur_img = new_frame;
```

### Eigen::Map：零拷贝映射的核心武器

`Eigen::Map` 可以将一块已有的内存"包装"为 Eigen 矩阵，**不复制任何数据**。这在 SLAM 的高频计算中（如每帧数百个特征点的坐标变换）至关重要——避免了不必要的 `memcpy`。

要使用 `cv::eigen2cv` 和 `cv::cv2eigen`，需要包含头文件 `<opencv2/core/eigen.hpp>`。这个头文件必须在包含 Eigen 头文件**之后**引入：

```cpp
#include <Eigen/Dense>             // 必须先包含 Eigen
#include <opencv2/core/eigen.hpp>  // 再包含 OpenCV 的 Eigen 互操作头文件
```

三种互转方式的对比：

```cpp
// ===== 方式 A：Eigen::Map 零拷贝（推荐高性能场景）=====
cv::Mat R_cv(3, 3, CV_64FC1);  // 3×3 double 矩阵
// ... 填充 R_cv ...

// 关键：cv::Mat 是行优先（Row-Major），Eigen 默认是列优先（Col-Major）
// 必须指定 Eigen::RowMajor！
Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_eigen(
    R_cv.ptr<double>());
// 现在 R_eigen 和 R_cv 共享内存，修改一个会影响另一个

// ===== 方式 B：cv::eigen2cv / cv::cv2eigen（安全但有拷贝）=====
Eigen::Matrix3d R_eigen_src = Eigen::Matrix3d::Identity();
cv::Mat R_cv_dst;
cv::eigen2cv(R_eigen_src, R_cv_dst);  // Eigen → cv::Mat，有内存拷贝

Eigen::Matrix3d R_eigen_dst;
cv::cv2eigen(R_cv_dst, R_eigen_dst);  // cv::Mat → Eigen，有内存拷贝

// ===== 方式 C：手动逐元素复制（最慢，仅用于调试）=====
for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
        R_cv.at<double>(i, j) = R_eigen_src(i, j);
```

**三种方式对比：**

| 方式 | 是否拷贝 | 行/列优先处理 | 性能 | 适用场景 |
|------|---------|-------------|------|---------|
| `Eigen::Map` | 否（零拷贝） | 需手动指定 `RowMajor` | 最快 | 高频循环（光流、特征匹配） |
| `eigen2cv`/`cv2eigen` | 是 | 自动处理 | 中等 | 偶尔转换（初始化、结果输出） |
| 手动逐元素 | 是 | 手动处理 | 最慢 | 调试验证 |

**行优先 vs 列优先的内存差异图解**：

```text
3×3 矩阵 M = | a b c |
              | d e f |
              | g h i |

行优先（cv::Mat）内存布局：  [a][b][c][d][e][f][g][h][i]
列优先（Eigen 默认）内存布局：[a][d][g][b][e][h][c][f][i]

如果不指定 RowMajor，Eigen::Map 把 cv::Mat 的 [a][b][c][d]...
按列优先读成：
  第 1 列 = [a, b, c]  → 实际上是第 1 行！
  第 2 列 = [d, e, f]  → 实际上是第 2 行！
结果：读到的矩阵是 M 的转置！
```

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：忘记 Row-Major vs Col-Major 导致旋转矩阵被转置**
>
> **错误做法**：直接用 `Eigen::Map<Eigen::Matrix3d>` 映射 `cv::Mat`
>
> **现象**：旋转矩阵 R 被"静默转置"——因为 `cv::Mat` 按行存储 `[r00, r01, r02, r10, r11, ...]`，而 Eigen 默认按列读取，读到的是 `[r00, r10, r20, r01, ...]`。程序不会崩溃，但位姿计算结果完全错误。在短序列上可能"看起来差不多"，在长序列上轨迹会严重漂移。
>
> **根本原因**：`cv::Mat` 是行优先存储（Row-Major），`Eigen::Matrix` 默认是列优先存储（Col-Major）。两者的内存布局在逻辑矩阵元素上是转置关系。
>
> **正确做法**：使用 `Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>`，或使用 `cv::cv2eigen()` 让 OpenCV 自动处理布局转换。
>
> **自检方法**：转换后立即检查 `R * R.transpose()` 是否为单位阵——但注意，正交矩阵转置后仍正交，所以这个检查无法发现转置错误！更好的检查是打印矩阵内容逐元素对比，或者用已知旋转（如绕 Z 轴转 90 度）验证变换结果是否符合预期。

> ⚠️ **编程陷阱：对非连续 cv::Mat 使用 Eigen::Map**
>
> **错误做法**：对 ROI 子矩阵直接 `Eigen::Map`
>
> **现象**：`Eigen::Map` 假定内存连续，但 ROI 的行间有间隔（`step > cols*elemSize`），导致 Eigen 读取到错位数据。结果矩阵的第 2 行开始就全是错位值。
>
> **根本原因**：ROI 的每行末尾存在"间隙"——原始矩阵中属于 ROI 外部列的数据。`Eigen::Map` 不知道这些间隙的存在。
>
> **正确做法**：先调用 `isContinuous()` 检查，非连续时先 `.clone()` 再映射。或使用 `Eigen::Map` 的 `Stride` 模板参数指定步长：
> ```cpp
> Eigen::Map<Eigen::MatrixXd, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
>     mapped(roi.ptr<double>(), roi.rows, roi.cols,
>            Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(
>                roi.step1(), 1));
> ```

> 💡 **概念误区：认为 `cv::eigen2cv` 会自动匹配数据类型**
>
> **新手想法**："反正 `eigen2cv` 会自动转，我不用管类型"
>
> **实际上**：`eigen2cv` 按照 Eigen 矩阵的标量类型设置输出 `cv::Mat` 的类型。如果你的 Eigen 矩阵是 `Matrix3f`（float），输出就是 `CV_32FC1`；是 `Matrix3d`（double），输出就是 `CV_64FC1`。后续如果传给期望 `CV_32F` 的 OpenCV 函数（如 `triangulatePoints`）就会报错。始终在转换后检查 `mat.type()`，或显式调用 `mat.convertTo(dst, CV_32F)` 统一类型。

### 练习

1. **零拷贝验证**：创建一个 `cv::Mat R_cv(3,3,CV_64FC1)` 并填入已知旋转矩阵（如绕 Z 轴旋转 45 度）。分别用 `Eigen::Map`（带 `RowMajor`）和 `cv2eigen` 转换为 Eigen 矩阵，验证两者结果一致。然后通过 `Eigen::Map` 修改 `R_eigen(0,0)` 的值，检查 `R_cv.at<double>(0,0)` 是否同步变化。

2. **ROI 陷阱复现**：从一张 640×480 灰度图中取一个 `cv::Rect(100,100,200,200)` 的 ROI，检查 `isContinuous()` 的返回值。尝试对该 ROI 直接 `Eigen::Map`（不带 Stride），观察结果是否正确。然后 `.clone()` 后再映射，对比差异。

3. **性能基准测试**：在一个循环中将 1000 个 `4×4` 的 `cv::Mat`（`CV_64FC1`）转换为 `Eigen::Matrix4d`，分别用三种方式（`Eigen::Map`、`cv2eigen`、逐元素拷贝）计时，比较性能差异。提示：使用 `cv::getTickCount()` 计时。

---

## 28.2 图像预处理 ⭐⭐

> 本节解决的问题：原始相机图像受光照变化、噪声、畸变等影响，SLAM 前端如何对图像进行标准化处理以提高特征检测和跟踪的鲁棒性？

### 动机：为什么 SLAM 需要图像预处理

想象你在地下停车场用手机做视觉定位——灯光昏暗且不均匀，有些区域几乎全黑，有些区域被车灯直射过曝。如果直接在这种原始图像上检测特征点，暗区几乎检测不到任何特征，亮区的特征也被过曝"冲掉"。SLAM 系统在这种场景下会迅速丢失跟踪。

这就是为什么所有工业级 SLAM 系统都有图像预处理步骤——通过 CLAHE（对比度受限自适应直方图均衡化）增强低光照区域的对比度，通过高斯模糊抑制噪声，通过图像金字塔实现多尺度检测。

**如果不做预处理会怎样？** 在 TUM-VI 数据集的 `corridor` 序列中（长走廊、光照极暗），不使用 CLAHE 的 ORB-SLAM3 在第 200 帧左右就会因为特征点不足（<30 个）而丢失跟踪。启用 CLAHE 后，特征点数量稳定在 800-1200 个，系统全程不丢失。这就是预处理的价值。

### cvtColor：颜色空间转换

SLAM 前端几乎全部工作在灰度图上（特征检测、光流跟踪都只需要灰度信息），因此第一步通常是将 BGR 彩色图转为灰度图：

```cpp
cv::Mat color_img = cv::imread("frame.png");  // 默认读入 BGR！不是 RGB
cv::Mat gray;
cv::cvtColor(color_img, gray, cv::COLOR_BGR2GRAY);
```

**为什么是 BGR 而不是 RGB？** 这是 OpenCV 的历史遗留设计。1990 年代 OpenCV 最初开发时，Windows 平台的 BMP 格式和摄像头驱动普遍使用 BGR 字节序，OpenCV 就沿用了这个约定。虽然反直觉，但已经成为 OpenCV 的核心约定——几乎所有 OpenCV 函数（包括 `imshow`、`imwrite`、`cvtColor`）都假定输入是 BGR 格式。

**灰度转换的数学公式**：

$$\text{gray} = 0.299 \times R + 0.587 \times G + 0.114 \times B$$

这个权重来自人眼对不同颜色的敏感度（ITU-R BT.601 标准）——人眼对绿色最敏感，对蓝色最不敏感。如果你用 `COLOR_RGB2GRAY` 处理 `cv::imread` 读入的 BGR 图像，R 和 B 通道会交换，变成 $0.299B + 0.587G + 0.114R$，灰度值会有微小差异（因为大多数自然场景中 R 和 B 相关性高），但在色彩偏差大的场景中可能影响特征检测的一致性。

### CLAHE：自适应直方图均衡化

CLAHE（Contrast Limited Adaptive Histogram Equalization）是 SLAM 中处理光照不均的标准手段。理解它需要先理解两个前置概念：

**全局直方图均衡化的问题**：全局均衡化将整张图的直方图拉伸到 [0, 255]，在光照均匀的场景中效果很好。但在光照不均匀时，暗区虽然被提亮了，噪声也被严重放大；亮区的细节被压缩。

**CLAHE 的解决思路**：
1. 将图像分成 `tileGridSize × tileGridSize`（默认 8×8）个小块（tile）
2. 对每个 tile 独立做直方图均衡化
3. 关键创新——**对比度裁剪（Clip Limiting）**：如果某个 tile 的直方图中某灰度级的像素数超过 `clipLimit`，超出部分被"裁剪"下来，均匀分配到所有灰度级。这防止了噪声区域被过度增强
4. 用双线性插值平滑相邻 tile 边界的过渡，避免块效应

**SLAM 系统中的实际参数对比：**

| 系统 | clipLimit | tileGridSize | 使用场景 | 设计理由 |
|------|-----------|-------------|---------|---------|
| ORB-SLAM3 | 3.0 | 8×8 | TUM-VI 低光照 | 保守增强，保护描述子稳定性 |
| OpenVINS | 10.0 | 8×8 | 极暗环境 | 激进增强，光流对灰度变化鲁棒 |
| OpenCV 默认 | 40.0 | 8×8 | 通用场景 | 通常过大，SLAM 中不建议 |

**为什么 ORB-SLAM3 用 3.0 而 OpenVINS 用 10.0？** 这背后是特征描述方式的差异：
- ORB-SLAM3 使用 BRIEF 描述子，通过比较像素点对的灰度大小生成二进制串。CLAHE 过强会改变局部灰度排序关系，导致同一物理点在不同帧的描述子发生大量比特翻转，匹配失败率上升。
- OpenVINS 使用光流跟踪，光流只依赖局部灰度梯度的方向和大小，对灰度的绝对值变化不敏感。因此可以用更大的 `clipLimit` 来最大程度提升暗区对比度。

```cpp
// ===== ORB-SLAM3 风格 CLAHE =====
cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
cv::Mat enhanced;
clahe->apply(gray, enhanced);

// ===== OpenVINS 风格 CLAHE =====
cv::Ptr<cv::CLAHE> clahe_vins = cv::createCLAHE(10.0, cv::Size(8, 8));
clahe_vins->apply(gray, enhanced);

// ===== 不推荐：OpenCV 默认值太大 =====
// cv::Ptr<cv::CLAHE> clahe_default = cv::createCLAHE();  // clipLimit=40.0
// 在 SLAM 中会导致暗区噪声被严重放大
```

**CLAHE 的计算开销**：在 640×480 图像上大约 1-2 ms（现代 CPU），在 1920×1080 上约 5-8 ms。ORB-SLAM3 在配置文件中提供了开关，只在低光照数据集上启用 CLAHE。在光照良好的室外环境中关闭可节省计算量。

### 高斯模糊：描述子计算前的降噪

ORB-SLAM3 在计算 ORB 描述子之前，会对图像做 7×7 的高斯模糊（$\sigma=2$）：

```cpp
// ORB-SLAM3 ORBextractor.cc 中的调用
cv::GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
```

**高斯核的数学定义**：

$$G(x,y) = \frac{1}{2\pi\sigma^2} \exp\left(-\frac{x^2+y^2}{2\sigma^2}\right)$$

离散化后，7×7 高斯核（$\sigma=2$）的权重矩阵大致为：

```text
核大小 7×7, σ=2 的近似权重分布：
0.002  0.006  0.013  0.016  0.013  0.006  0.002
0.006  0.019  0.039  0.048  0.039  0.019  0.006
0.013  0.039  0.080  0.098  0.080  0.039  0.013
0.016  0.048  0.098  0.120  0.098  0.048  0.016
0.013  0.039  0.080  0.098  0.080  0.039  0.013
0.006  0.019  0.039  0.048  0.039  0.019  0.006
0.002  0.006  0.013  0.016  0.013  0.006  0.002
```

中心权重最大（0.120），边缘权重趋近于零——这就是"高斯"模糊的本质：距离越远的像素对中心像素的贡献越小。

**为什么是 7×7 而不是 3×3 或 5×5？** 核大小通常取 $\lceil 6\sigma \rceil$ 的最近奇数。$\sigma=2$ 对应 $6\sigma=12$，完整应该是 13×13，但 7×7 已经截断了高斯尾部（保留 >95% 的能量），是计算效率和模糊效果的折中。ORB 描述子的 BRIEF 模式在 31×31 patch 内采样，点对间距通常 5-15 像素——7×7 模糊恰好消除了像素级噪声，同时不会模糊掉这些采样点之间的灰度差异。

**重要区分**：ORB-SLAM3 对检测和描述子使用**不同的图像**。FAST 角点检测在原始（或 CLAHE 增强后的）图像上进行——因为高斯模糊会降低角点响应值。描述子计算在模糊后的图像上进行——因为 BRIEF 需要稳定的灰度比较。

### buildOpticalFlowPyramid：预构建图像金字塔

OpenVINS 的 `TrackKLT.cpp` 在跟踪前会预构建图像金字塔，而不是让 `calcOpticalFlowPyrLK` 内部自动构建：

```cpp
// OpenVINS TrackKLT.cpp 风格
std::vector<cv::Mat> imgpyr;
cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels);
// imgpyr 包含 2*(pyr_levels+1) 个 cv::Mat：
// imgpyr[0] = 原始图像
// imgpyr[1] = 第 0 层的 Scharr 梯度
// imgpyr[2] = 下采样 2x 的图像
// imgpyr[3] = 第 1 层的 Scharr 梯度
// ...
```

**为什么要预构建而不是让 `calcOpticalFlowPyrLK` 自动构建？** 两个重要原因：

1. **复用金字塔**：同一帧图像可能被多次用于跟踪。当前帧作为"参考帧"时，它的金字塔会在下一帧到来时被复用——避免重复计算。VINS-Mono 没有预构建金字塔，每次调用 `calcOpticalFlowPyrLK` 都要重新构建，浪费了约 30% 的光流计算时间。
2. **自定义相机模型**：OpenVINS 支持等距（equidistant）、径向-切向（radtan）等多种畸变模型。预构建金字塔后，可以在自定义的去畸变流程中独立使用各层。

**金字塔的计算开销**：每一层的像素数是上一层的 $\frac{1}{4}$（长宽各减半），因此总计算量是原图的 $\sum_{k=0}^{L} \frac{1}{4^k} = \frac{4}{3}(1 - \frac{1}{4^{L+1}}) \approx \frac{4}{3}$ 倍。金字塔并不显著增加计算量，但能处理大位移——这是一个非常划算的投资。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：BGR/RGB 混淆导致颜色通道错位**
>
> **错误做法**：用 ROS 的 `cv_bridge` 接收图像（ROS 的 `sensor_msgs/Image` 常用 `rgb8` 编码），然后直接用 `cv::COLOR_BGR2GRAY` 转灰度。
>
> **现象**：灰度转换公式中 R 和 B 通道权重互换。在大多数场景下视觉差异很小（自然场景中 R、B 通道相关性高），但在红色/蓝色物体附近，灰度值偏差可达 10-20 个灰度级，导致帧间特征检测的响应值不一致。
>
> **根本原因**：`cv::imread` 输出 BGR，ROS `sensor_msgs::Image` 可以是 RGB 或 BGR（取决于编码字段），第三方相机 SDK 可能是任意格式。
>
> **正确做法**：在管线入口明确检查图像编码格式。ROS 中用 `cv_bridge::toCvCopy(msg, "mono8")` 直接获取灰度图最安全。

> 💡 **概念误区：认为 CLAHE 的 clipLimit 越大增强效果越好**
>
> **新手想法**："低光照环境下把 clipLimit 调到最大不就亮了吗？"
>
> **实际上**：`clipLimit` 控制的是每个 tile 直方图中允许的最大计数值。超过此值的像素数量被"裁剪"并均匀重分配到所有灰度级。过大的 `clipLimit`（如默认 40.0）会让暗区的传感器噪声被放大为"虚假特征"——FAST 检测器会在噪声纹理上检测出大量不可重复的角点，这些角点在下一帧就消失了，导致跟踪率急剧下降。
>
> **自检方法**：分别用 `clipLimit` = 1.0, 3.0, 10.0, 40.0 处理一张低光照图像，对比 FAST 检测到的特征点数量和帧间重复率。好的 `clipLimit` 应该是"特征点数量增加但重复率不下降"的平衡点。

> 🧠 **思维陷阱：认为高斯模糊应该用于所有图像处理步骤**
>
> **新手想法**："既然模糊能降噪，那 FAST 检测、光流跟踪之前都应该先模糊一下"
>
> **实际上**：ORB-SLAM3 的 7×7 高斯模糊**仅用于描述子计算**，不用于 FAST 角点检测也不用于光流跟踪。FAST 检测需要锐利的角点响应，模糊后角点会被"钝化"，检出率下降。光流跟踪需要准确的灰度梯度，模糊会降低梯度幅值。只有 BRIEF 描述子的二进制比较需要稳定的灰度——所以只在描述子计算前模糊。

### 练习

1. **CLAHE 参数实验**：对 TUM-VI 数据集的低光照序列（如 `room1`），分别用 `clipLimit` = 1.0, 3.0, 10.0, 40.0 进行 CLAHE 增强，用 `cv::FAST(img, kps, 20, true)` 检测每张图的特征点数量，绘制"clipLimit vs 特征点数量"曲线。观察是否存在"收益递减"的拐点。

2. **金字塔可视化**：用 `buildOpticalFlowPyramid` 构建 4 层金字塔（`pyr_levels=3`），将每层图像缩放到相同大小后水平拼接显示。计算每层相对于原图的像素总数比值，验证总计算量约为原图的 $\frac{4}{3}$ 倍（$1 + \frac{1}{4} + \frac{1}{16} + \frac{1}{64}$）。

3. **预处理流水线复现**：实现 ORB-SLAM3 的完整预处理流水线：`cvtColor(BGR2GRAY)` → `CLAHE(3.0, 8×8)` → `GaussianBlur(7×7, σ=2)`。对比直接使用原始灰度图和经过预处理后的图像，分别用 `cv::ORB::create(1000)` 做 ORB 检测+匹配（BFMatcher + Hamming distance），统计正确匹配数的提升百分比。

---

## 28.3 特征检测 ⭐⭐

> 本节解决的问题：如何从预处理后的图像中高效、均匀地检测出高质量的角点特征，作为 SLAM 跟踪和匹配的基础？

### 动机：特征检测是视觉 SLAM 的"眼睛"

视觉 SLAM 的前端需要在每帧图像中找到一组"有辨识度"的像素位置（特征点），用于帧间跟踪（光流法）或帧间匹配（描述子法）。特征点的质量直接决定了位姿估计的精度——如果检测到的全是纹理平坦区域的噪声点，后端优化再强大也无力回天。

**如果不做均匀化会怎样？** 在一个典型的室内场景中，书架区域可能检测出 500 个角点，旁边的白墙只有 2 个。如果全部保留书架的 500 个点，不仅计算浪费（这 500 个点提供的几何约束高度相关），而且 SLAM 在白墙方向上完全没有约束——当相机朝白墙方向移动时，估计的位移误差会非常大。均匀分布的特征点才能为所有方向提供均衡的约束。

### cv::FAST：亚毫秒级角点检测

FAST（Features from Accelerated Segment Test）是最快的角点检测器之一。它的算法思想可以追溯到 Edward Rosten 和 Tom Drummond 在 2006 年的论文。核心思想极其简洁：

**算法步骤**：
1. 取候选像素 $p$，灰度值为 $I_p$
2. 考察以 $p$ 为中心的 Bresenham 圆（半径 3，共 16 个像素）
3. 如果圆上有连续 $N$（FAST-12 中 $N=12$）个像素的灰度值都满足 $I_i > I_p + t$（全亮）或 $I_i < I_p - t$（全暗），则 $p$ 是角点
4. **高速测试（Speed Test）**：先检查位置 1、5、9、13（圆的四个端点）。至少有 3 个满足条件才可能有连续 12 个——如果不满足，直接跳过。这使得大多数非角点像素只需 4 次比较即可排除

```cpp
// 基本 FAST 检测
std::vector<cv::KeyPoint> keypoints;
cv::FAST(gray, keypoints, 20, true);  // threshold=20, nonmaxSuppression=true
```

**ORB-SLAM3 的双阈值策略（先 20 后 7）**：

ORB-SLAM3 在 `ORBextractor.cc` 中实现了一个精巧的双阈值 FAST 检测策略。这是理解工业级 SLAM 设计的关键细节：

```text
算法流程：
1. 将图像金字塔的每一层分割为约 30×30 像素的 cell
2. 对每个 cell，先用高阈值 iniThFAST=20 检测 FAST 角点
3. 如果某个 cell 检测到 0 个角点，降低阈值到 minThFAST=7 重新检测
4. 收集所有 cell 的角点，送入四叉树做均匀化分布
```

**为什么是阈值 20 和 7？** 这两个值经过大量实验确定：
- **20**：在正常光照和纹理条件下，阈值 20 可以过滤掉大部分噪声响应，只保留灰度对比度显著的真实角点。在 640×480 的典型室内图像上，阈值 20 通常检测出 2000-5000 个角点。
- **7**：这是 FAST 的实用下限——低于 7 时，传感器噪声（特别是 CMOS 在低光照下的读出噪声，标准差约 2-5 灰度级）会被大量误检为角点。阈值 7 在弱纹理区域（如走廊尽头的墙面交界处）仍能检出有用的角点。

**为什么不直接用低阈值（7）检全部？** 因为低阈值在高纹理区域会检测出成千上万的角点，计算开销爆炸且绝大多数冗余。"先高后低、按 cell 自适应"的策略是一种高效的空间自适应机制——在高纹理区域用高标准精选，在低纹理区域降低标准兜底。

**四叉树均匀化（Quadtree Distribution）**：

收集到所有 FAST 角点后，ORB-SLAM3 用 `DistributeOctTree` 将角点均匀化分布：

```text
四叉树均匀化步骤（DistributeOctTree）：
1. 初始化：将整张图像作为根节点，根据宽高比分为若干初始节点
2. 递归细分：遍历每个节点——如果包含 >1 个角点，将其分为 4 个子节点
   （左上、右上、左下、右下），角点按坐标分配到对应子节点
3. 终止条件：节点总数 >= 目标特征数 N，或所有节点都只含 0 或 1 个角点
4. 选择：从每个包含角点的叶节点中，选择 FAST 响应值最大的那一个

结果：每个叶节点覆盖图像的一小块区域，且只保留最强角点
     → 特征点在图像上均匀分布，同时优先保留高质量角点
```

这种设计的优势在于：即使场景一半是高纹理的书架、一半是低纹理的白墙，最终特征点也会在两个区域均匀分布，保证 SLAM 在所有方向上都有视觉约束。

### goodFeaturesToTrack：VINS-Mono 的选择

VINS-Mono 选择 Shi-Tomasi 角点（`goodFeaturesToTrack`）而非 FAST 的原因值得深入理解。

**Shi-Tomasi 评分的数学原理**：对像素 $p$ 的邻域，构建结构张量（Structure Tensor / Second Moment Matrix）：

$$\mathbf{M} = \sum_{(x,y) \in W} \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$$

其中 $I_x, I_y$ 是灰度梯度。$\mathbf{M}$ 的特征值 $\lambda_1 \geq \lambda_2$ 描述了局部灰度变化的两个主方向：
- $\lambda_1$ 大, $\lambda_2$ 大 → 角点（两个方向都有梯度变化）
- $\lambda_1$ 大, $\lambda_2 \approx 0$ → 边缘（只有一个方向有变化）
- 都小 → 平坦区域

Harris 角点用 $R = \lambda_1 \lambda_2 - k(\lambda_1 + \lambda_2)^2$ 作为评分；Shi-Tomasi 直接用 $\lambda_{\min} = \lambda_2$ 作为评分——更简洁且理论上更优（Shi 和 Tomasi 在 1994 年证明了 $\lambda_{\min}$ 是特征跟踪稳定性的最佳指标）。

**为什么光流跟踪偏好 Shi-Tomasi？** 回顾 28.4 节的 LK 光流方程——解的稳定性取决于 $\mathbf{A}^T \mathbf{A} = \mathbf{M}$ 的可逆性，即 $\lambda_{\min}$ 要足够大。`goodFeaturesToTrack` 直接筛选 $\lambda_{\min}$ 大的点，保证检测到的每个角点都适合光流跟踪。FAST 只检测灰度圆周跳变，不保证结构张量的条件数。

VINS-Mono `feature_tracker.cpp` 中的关键参数：

```cpp
// VINS-Mono parameters.cpp 中定义的参数
int MAX_CNT = 150;          // 最大特征点数量
double MIN_DIST = 30;       // 特征点最小间距（像素）
double qualityLevel = 0.01; // Shi-Tomasi 评分阈值（相对于最强角点的比例）

// feature_tracker.cpp 中的调用——每帧只补充缺失的特征点
int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
if (n_max_cnt > 0) {
    cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt,
                            qualityLevel, MIN_DIST, mask);
}
```

**参数设计的工程考量：**

- **`MAX_CNT=150`**：VINS-Mono 是 VIO 系统，IMU 提供了帧间角速度和加速度先验，视觉只需提供足够的平移约束。150 个点在 640×480 图像上已能覆盖大部分区域。实验表明，从 150 增加到 300 个点，VIO 精度提升不到 5%，但光流计算时间几乎翻倍。反之，低于 80 个点时，在严重遮挡场景下可能丢失跟踪。
- **`qualityLevel=0.01`**：这是相对阈值——如果最强角点的 Shi-Tomasi 评分为 $S_{\max}$，则只保留评分 $\geq 0.01 \times S_{\max}$ 的角点。0.01 非常宽松，意味着"只要有一点角点特性就接受"。VINS-Mono 宁可多检测弱角点，也不错过有用特征——后续有 `setMask` 做均匀化和 RANSAC 做外点剔除。
- **`MIN_DIST=30`**：两个角点至少相隔 30 像素。30 像素约是光流 21×21 窗口的 1.4 倍，确保相邻特征点的光流搜索窗口不会严重重叠——重叠的窗口提供的是冗余约束，不值得额外计算。

### cv::ORB::create()：OpenVINS 的直接使用 vs ORB-SLAM3 的自定义实现

这是一个教学中经常被混淆的关键事实：

| 系统 | 是否使用 `cv::ORB::create()` | 原因 |
|------|---------------------------|------|
| ORB-SLAM3 | **否**，完全自研 | 需要双阈值 FAST、四叉树均匀化、自定义 BRIEF 模式 |
| OpenVINS | **是**，在 `TrackDescriptor` 模式 | 主要用光流跟踪，ORB 描述子只作为辅助重定位 |
| stella_vslam | **否**，也自研了 ORB 提取 | 沿用 OpenVSLAM 的设计 |

```cpp
// OpenVINS 直接使用 OpenCV ORB 的方式
auto orb = cv::ORB::create(
    500,                    // nfeatures: 最大特征数
    1.2f,                   // scaleFactor: 金字塔尺度因子
    8,                      // nlevels: 金字塔层数
    31,                     // edgeThreshold: 边缘排除阈值
    0,                      // firstLevel: 金字塔起始层
    2,                      // WTA_K: BRIEF 描述子每次比较的点数
    cv::ORB::HARRIS_SCORE,  // scoreType: 用 Harris 评分排序
    31,                     // patchSize: BRIEF 描述子 patch 大小
    20                      // fastThreshold: FAST 阈值
);
std::vector<cv::KeyPoint> kps;
cv::Mat desc;
orb->detectAndCompute(gray, cv::noArray(), kps, desc);
```

**ORB-SLAM3 自研实现的核心优势**：

1. **双阈值自适应**：`cv::ORB` 全图使用同一个 `fastThreshold`，无法按区域自适应
2. **四叉树均匀化**：`cv::ORB` 使用简单的 Harris 评分排序截断前 N 个点，不保证空间均匀性
3. **逐层控制**：ORB-SLAM3 根据金字塔层级分配不同的目标特征数（上层更多，下层更少），`cv::ORB` 的分配策略不够灵活
4. **硬编码 BRIEF 模式**：ORB-SLAM3 使用特定的 256 个点对模式（`bit_pattern_31_` 数组），经过实验优化以最大化描述子的区分度

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：检测完特征后忘记非极大值抑制（NMS）**
>
> **错误做法**：`cv::FAST(img, kps, 20, false)`——第四个参数 `nonmaxSuppression=false`
>
> **现象**：一个物理角点周围可能有 5-10 个像素都满足 FAST 判据，产生"特征簇"。这些高度相关的点被当作独立特征，不仅浪费计算资源，还可能导致位姿估计中的秩亏——多个点实际上只提供一个独立约束。
>
> **根本原因**：FAST 的 Bresenham 圆只有 3 像素半径，相邻像素的检测窗口高度重叠，如果一个像素是角点，邻居很可能也是。
>
> **正确做法**：始终启用 NMS（`nonmaxSuppression=true`），或使用 ORB-SLAM3 的四叉树做更激进的均匀化。`goodFeaturesToTrack` 的 `minDistance` 参数天然实现了 NMS 功能。

> 💡 **概念误区：认为 FAST 检测的是"边缘"**
>
> **新手想法**："FAST 检测圆周上连续亮/暗的像素，那沿着边缘不也有一侧亮一侧暗吗？"
>
> **实际上**：边缘上的像素确实有一侧亮一侧暗，但不会形成连续 12 个像素**全亮**或**全暗**的模式。边缘上的 Bresenham 圆大约一半亮一半暗（形成两段），每段约 8 个像素，达不到 12 的连续要求。角点处的圆则不同——两个方向都有灰度跳变，可以形成一大段连续的亮（或暗）区域。

> 🧠 **思维陷阱：认为"ORB-SLAM3 用 ORB，所以 cv::ORB 就是 ORB-SLAM3 的检测器"**
>
> **实际上**：ORB-SLAM3 **不使用** `cv::ORB::create()`。它在 `ORBextractor.cc` 中从头实现了整套流水线（约 1000 行代码）。如果你试图通过调用 `cv::ORB` 来"复现"ORB-SLAM3 的前端，结果会有显著差异——`cv::ORB` 没有双阈值策略、没有四叉树均匀化。阅读源码和论文时务必区分"算法思想同名"和"代码实现相同"。

### 练习

1. **双阈值 FAST 实验**：对同一张图像分别用 `threshold=20` 和 `threshold=7` 做 FAST 检测，统计角点数量。然后实现 ORB-SLAM3 的双阈值策略：将图像分成 30×30 的 cell，先在每个 cell 中用 20 检测，空白 cell 用 7 重检。对比三种策略（单阈值 20、单阈值 7、双阈值）的角点总数和空间均匀性（可用每个 cell 中的角点数标准差衡量）。

2. **goodFeaturesToTrack 参数敏感性**：固定 `qualityLevel=0.01`，分别用 `MIN_DIST` = 10, 20, 30, 50 像素检测特征点。绘制"MIN_DIST vs 特征点数量"曲线。计算每种设置下特征点到最近邻的平均距离，验证它是否接近 `MIN_DIST`。

3. **cv::ORB vs 自定义实现对比**：使用 `cv::ORB::create(1000)` 检测特征点，然后自己实现"FAST(threshold=20) + 非极大值抑制 + 简单网格均匀化（将图像分为 10×10 网格，每格保留响应最强的 10 个点）"。可视化两者的空间分布差异，计算特征点到最近邻的平均距离。

---

## 28.4 光流跟踪 ⭐⭐

> 本节解决的问题：如何利用连续帧间的灰度一致性，不计算描述子就追踪特征点的运动？

### 动机：为什么不用描述子匹配而用光流

描述子匹配（如 ORB 匹配）的流程：对每个特征点计算 256 位描述子 → 构建 KD-tree 或暴力搜索最近邻 → 用汉明距离筛选匹配。对 1000 个特征点，仅描述子计算就需要约 5 ms，加上匹配约 2-3 ms，总共 7-8 ms。

光流跟踪的流程：对每个特征点在新帧中搜索灰度一致的位置。对 150 个特征点（VINS-Mono 的设定），`calcOpticalFlowPyrLK` 只需约 2-3 ms——速度快一倍以上，且能达到亚像素精度。

**光流法的适用条件**：帧率足够高（>15 Hz）、帧间位移足够小（<50 像素）。在 VIO 场景中（IMU 提供了帧间先验），这两个条件通常都满足。这就是 VINS-Mono 和 OpenVINS 选择光流的原因。

**光流法的局限**：不能做回环检测（只能跟踪连续帧，无法匹配时间上相隔很远的帧）、不能做重定位（跟踪丢失后无法恢复）。ORB-SLAM3 选择描述子匹配正是因为它需要这些能力。

### LK 光流理论：从亮度恒定假设到最小二乘解

**亮度恒定假设（Brightness Constancy Assumption）**：

设像素 $(x, y)$ 在时刻 $t$ 的灰度值为 $I(x, y, t)$，假设该像素在短时间 $\delta t$ 内移动到 $(x+\delta x, y+\delta y)$ 且灰度不变：

$$I(x, y, t) = I(x+\delta x, y+\delta y, t+\delta t)$$

对右侧做一阶泰勒展开：

$$I(x+\delta x, y+\delta y, t+\delta t) \approx I(x,y,t) + \frac{\partial I}{\partial x} \delta x + \frac{\partial I}{\partial y} \delta y + \frac{\partial I}{\partial t} \delta t$$

代入亮度恒定方程，消去 $I(x,y,t)$：

$$I_x \, \delta x + I_y \, \delta y + I_t \, \delta t = 0$$

两边除以 $\delta t$，令 $u = \frac{\delta x}{\delta t}$，$v = \frac{\delta y}{\delta t}$ 为光流速度：

$$I_x u + I_y v + I_t = 0 \quad \Longleftrightarrow \quad \nabla I \cdot \begin{bmatrix} u \\ v \end{bmatrix} = -I_t$$

这就是**光流约束方程**。它只给出了 $(u, v)$ 的一个线性约束——一个方程两个未知数，欠定！这就是所谓的**孔径问题（Aperture Problem）**：仅从一个像素看，无法区分沿等灰度线的不同运动方向。

**Lucas-Kanade 方法的核心思想**：

为了解决欠定问题，LK 方法假设以像素 $(x, y)$ 为中心的 $w \times w$ 窗口内，所有像素共享相同的光流 $(u, v)$。这是一个"局部刚体运动"假设——窗口足够小时，场景表面近似平面，窗口内所有点的运动一致。

窗口内 $n = w \times w$ 个像素各提供一个方程，组成超定方程组 $\mathbf{A} \mathbf{d} = \mathbf{b}$：

$$\underbrace{\begin{bmatrix} I_x(p_1) & I_y(p_1) \\ I_x(p_2) & I_y(p_2) \\ \vdots & \vdots \\ I_x(p_n) & I_y(p_n) \end{bmatrix}}_{\mathbf{A} \in \mathbb{R}^{n \times 2}} \underbrace{\begin{bmatrix} u \\ v \end{bmatrix}}_{\mathbf{d}} = \underbrace{-\begin{bmatrix} I_t(p_1) \\ I_t(p_2) \\ \vdots \\ I_t(p_n) \end{bmatrix}}_{\mathbf{b}}$$

最小二乘解为：

$$\mathbf{d} = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T \mathbf{b}$$

其中 $\mathbf{A}^T \mathbf{A}$ 就是**结构张量（Structure Tensor）**：

$$\mathbf{M} = \mathbf{A}^T \mathbf{A} = \begin{bmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{bmatrix}$$

$\mathbf{M}$ 可逆要求两个特征值 $\lambda_1, \lambda_2$ 都足够大——这恰好就是 Shi-Tomasi "Good Features to Track" 的定义。所以说，**"好的角点"在数学上就是"适合光流跟踪的点"**，这不是巧合，而是 Shi-Tomasi 方法的设计初衷。

**为什么需要金字塔？** LK 方法的泰勒展开 $I(x+\delta x) \approx I(x) + I_x \delta x$ 只在 $\delta x$ 很小（<< 1 个像素）时准确。当帧间位移超过几像素时，一阶近似失效，迭代可能不收敛或收敛到错误位置。

金字塔策略的思路：
1. 构建 $L$ 层金字塔：$I^0$（原图）, $I^1$（缩小 2×）, ..., $I^L$（缩小 $2^L$×）
2. 在最粗层 $I^L$ 上，像素位移也缩小 $2^L$ 倍，回到"小位移"范围
3. 在 $I^L$ 上用 LK 估计粗略光流 $\mathbf{d}^L$
4. 将 $\mathbf{d}^L$ 乘以 2 后传到 $I^{L-1}$ 层作为初始猜测，再迭代细化
5. 逐层向下，直到 $I^0$ 得到最终的亚像素精度光流

### calcOpticalFlowPyrLK：VINS-Mono 的核心跟踪函数

```cpp
// VINS-Mono feature_tracker.cpp 中的光流跟踪调用
std::vector<uchar> status;
std::vector<float> err;
cv::calcOpticalFlowPyrLK(
    prev_img, cur_img,          // 上一帧和当前帧图像
    prev_pts, cur_pts,          // 输入上一帧特征点，输出当前帧跟踪结果
    status,                      // 每个点的跟踪状态（1=成功, 0=失败）
    err,                         // 每个点的跟踪误差（SSD 残差）
    cv::Size(21, 21),           // 搜索窗口大小
    3                            // 金字塔层数（0=不用金字塔）
);
```

**参数深入解析：**

| 参数 | VINS-Mono 值 | 含义 | 设计权衡 |
|------|-------------|------|---------|
| `winSize` | 21×21 | LK 窗口大小（像素） | 大窗口 → 容忍大位移、抗噪声；但窗口内光流一致假设更难成立 |
| `maxLevel` | 3 | 金字塔层数 | 3 层 → 最粗层缩 $2^3=8$ 倍 → 能处理 ~80 像素的大位移 |
| `criteria` | 默认 | 迭代终止条件 | 默认 `TermCriteria(COUNT+EPS, 30, 0.01)`: 最多 30 次或精度 0.01 |
| `flags` | 默认 0 | 特殊标志 | 可设 `OPTFLOW_USE_INITIAL_FLOW` 使用 `cur_pts` 的初始值作为猜测 |

**为什么窗口是 21×21？** 窗口大小的选择是多个因素的权衡：

- **太小（如 7×7）**：只有 49 个像素参与求解，信噪比低。且 $\mathbf{A}^T \mathbf{A}$ 的条件数容易很大（当窗口刚好覆盖边缘而非角点时），导致解不稳定。
- **太大（如 51×51）**：2601 个像素参与求解，但"窗口内光流一致"的假设在深度不连续边界（如物体边缘）处不成立——前景和背景有不同的运动，大窗口会把两种运动"平均"掉。
- **21×21 是经验最佳值**：包含 441 个像素，信噪比足够；窗口约 0.03 倍图像宽度，在 30 Hz 帧率下足以覆盖正常运动范围。

### 前向-后向一致性检查（Forward-Backward Consistency Check）

这是检测光流跟踪失败最有效的方法之一，由 Zdenek Kalal 在 2010 年（TLD 跟踪器论文中）推广。其思想极其直观：

如果跟踪器"真的找到了"正确的对应点，那么从反方向跟回去应该能回到起点。如果回不到，说明跟踪出了问题。

```cpp
// 前向-后向一致性检查的完整实现
std::vector<cv::Point2f> pts_B, pts_back;
std::vector<uchar> status_fwd, status_back;
std::vector<float> err_fwd, err_back;

// 前向跟踪：A → B
cv::calcOpticalFlowPyrLK(img_A, img_B, pts_A, pts_B,
                          status_fwd, err_fwd, cv::Size(21,21), 3);

// 后向跟踪：B → A
cv::calcOpticalFlowPyrLK(img_B, img_A, pts_B, pts_back,
                          status_back, err_back, cv::Size(21,21), 3);

// 一致性检查
for (size_t i = 0; i < pts_A.size(); i++) {
    if (status_fwd[i] && status_back[i]) {
        float dx = pts_A[i].x - pts_back[i].x;
        float dy = pts_A[i].y - pts_back[i].y;
        float fb_error = dx*dx + dy*dy;
        if (fb_error > 1.0) {  // 阈值：1 像素的平方距离
            status_fwd[i] = 0;  // 标记为跟踪失败
        }
    }
}
```

**关键事实：VINS-Mono 并未实现前向-后向一致性检查。** 这是源码中一个经常被误引用的设计选择。VINS-Mono 转而依赖 `findFundamentalMat` 的 RANSAC 来剔除外点（详见 28.5 节）。

**为什么 VINS-Mono 不用前向-后向检查？**
1. **计算开销**：前向-后向需要两次光流计算，在嵌入式平台（VINS-Mono 的目标平台之一）上时间翻倍
2. **已有替代方案**：RANSAC 基于全局几何约束剔除外点，比局部的前向-后向检查更全面
3. **冗余性**：对于已经通过 RANSAC 的点，前向-后向检查很少能额外发现问题

**OpenVINS 的不同选择**：OpenVINS 在 `TrackKLT.cpp` 中实现了前向-后向一致性检查，因为：
1. OpenVINS 面向更强计算力的平台（桌面/笔记本），翻倍的开销可以接受
2. 前向-后向检查能在 RANSAC 之前就剔除大量外点，提高 RANSAC 的效率和精度

### VINS-Mono setMask：特征点均匀化的精巧设计

VINS-Mono 的 `setMask()` 是其光流前端最精巧的设计之一——一个简单而高效的特征点管理策略。让我们逐步拆解：

**核心思路**：优先保留"跟踪时间长"的老特征点，新特征点只填补老特征点覆盖不到的空白区域。

```text
setMask() 算法流程：
1. 创建全白（255）mask 图像，大小与原图相同
2. 将所有当前帧的特征点按 track_cnt（跟踪次数）降序排列
   → 跟踪 30 帧的"老兵" > 跟踪 2 帧的"新兵"
3. 遍历排好序的特征点：
   a. 检查该点位置在 mask 上是否为 255（未被占用）
   b. 如果是 255 → 保留该点，并在 mask 上画实心黑圆：
      cv::circle(mask, pt, MIN_DIST, 0, -1)
      （半径 MIN_DIST=30 像素的圆形区域变为 0）
   c. 如果不是 255 → 该点被优先级更高的点"占位"，丢弃
4. 用剩余的 mask（非零区域）作为 goodFeaturesToTrack 的检测区域，
   补充新特征点
```

**为什么按 track_cnt 降序是合理的？**

一个被连续跟踪了 30 帧的特征点有极高的价值：
- 它提供了长基线的视差观测，对 SLAM 后端的三角化精度至关重要
- 它是稳定的物理点（不是噪声或运动物体），已经被多帧验证
- 它与 IMU 预积分的时间窗口有大量重叠，能提供丰富的视觉-惯性约束

一个只跟踪了 2 帧的新点还未经验证——可能是噪声、可能在下一帧就因遮挡消失。给老点更高优先级是"经验优先"的合理策略。

```cpp
// VINS-Mono setMask() 核心代码还原
void FeatureTracker::setMask() {
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // 构建 (track_cnt, point, id) 三元组并按 track_cnt 降序排列
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back({track_cnt[i], {forw_pts[i], ids[i]}});
    std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });

    // 清空并重新填充
    forw_pts.clear(); ids.clear(); track_cnt.clear();
    for (auto &it : cnt_pts_id) {
        if (mask.at<uchar>(it.second.first) == 255) {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // 画实心黑圆：阻止后续更低优先级的点占据周围 MIN_DIST 范围
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    // 之后用此 mask 调用 goodFeaturesToTrack 补充新特征
}
```

**圆形 mask 的几何意义**：`cv::circle(mask, pt, 30, 0, -1)` 在特征点周围画一个半径 30 像素的实心黑圆。这意味着：
- 任意两个保留点之间距离 $\geq 30$ 像素
- 新检测的特征点也必须距离所有已有点 $\geq 30$ 像素
- 效果：在整张图像上形成近似均匀的特征点网格

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：不检查 status 数组就使用光流跟踪结果**
>
> **错误做法**：`calcOpticalFlowPyrLK` 返回后，直接使用 `cur_pts` 的所有点。
>
> **现象**：`status[i]==0` 的点表示跟踪失败——例如特征点运动到图像外、被遮挡、亮度剧烈变化。这些点在 `cur_pts` 中的坐标可能是随机值或上一帧的原始坐标。使用这些错误坐标计算位姿会引入巨大误差。
>
> **根本原因**：`calcOpticalFlowPyrLK` 不会从输出数组中删除失败的点——它保留所有点的顺序，只通过 `status` 标记成功/失败。
>
> **正确做法**：遍历 `status` 数组，只保留 `status[i]==1` 的点。同时要同步更新所有关联数组（`ids`, `track_cnt`, `prev_pts` 等）。VINS-Mono 用 `reduceVector(vec, status)` 辅助函数完成这一步。
>
> **自检方法**：跟踪后打印 `status` 中为 0 的比例。正常情况下（帧间位移小、图像质量好），成功率应在 80-95%。如果低于 50%，说明参数设置有问题或帧间位移太大。

> ⚠️ **编程陷阱：光流跟踪后不检查点是否在图像边界内**
>
> **错误做法**：只检查 `status`，不检查 `cur_pts[i]` 的坐标范围。
>
> **现象**：某些情况下 `calcOpticalFlowPyrLK` 返回 `status[i]=1`，但跟踪到的点坐标已超出图像边界（x<0 或 x>=cols）。后续在该坐标访问像素或计算描述子导致数组越界。
>
> **正确做法**：VINS-Mono 在 `reduceVector` 后还用 `inBorder()` 检查每个点：
> ```cpp
> bool inBorder(const cv::Point2f &pt) {
>     const int BORDER_SIZE = 1;
>     return BORDER_SIZE <= pt.x && pt.x < COL - BORDER_SIZE
>         && BORDER_SIZE <= pt.y && pt.y < ROW - BORDER_SIZE;
> }
> ```

> 💡 **概念误区：认为 VINS-Mono 实现了前向-后向一致性检查**
>
> **新手想法**："VINS-Mono 是经典光流 SLAM，肯定用了前向-后向检查吧？很多博客都这么写。"
>
> **实际上**：VINS-Mono 的 `feature_tracker.cpp` 源码中**没有**前向-后向一致性检查。它使用 `findFundamentalMat` 的 RANSAC 来剔除外点。OpenVINS 的 `TrackKLT.cpp` 才实现了前向-后向检查。很多 SLAM 教程和博客将两个系统的技术细节混为一谈——这提醒我们，理解 SLAM 系统的最可靠方式始终是阅读源码，而非二手资料。

> 🧠 **思维陷阱：认为"光流法比描述子匹配法差"**
>
> **新手想法**："ORB-SLAM3 比 VINS-Mono 有名，ORB-SLAM3 用描述子，所以描述子更好。"
>
> **实际上**：光流和描述子适用于不同场景。光流在高帧率（>15 Hz）、小位移、连续跟踪中效率更高、精度也好（天然亚像素级）。描述子匹配在低帧率、大位移、回环检测、重定位中不可替代。VINS-Mono（光流）和 ORB-SLAM3（描述子）的性能差异主要来自系统整体设计（VIO vs 纯视觉、滑动窗口 vs 全局BA），而非前端匹配方式。选择方法时应根据系统需求而非"哪个更高级"。

### 练习

1. **光流窗口大小实验**：用 `calcOpticalFlowPyrLK` 跟踪一组特征点，分别用窗口大小 7×7、15×15、21×21、31×31，金字塔层数=3。对每种窗口大小，计算跟踪成功率（`status==1` 的比例）和前向-后向误差（实现一致性检查计算每个点的 FB 误差）。绘制"窗口大小 vs 成功率"和"窗口大小 vs 平均 FB 误差"曲线。

2. **前向-后向一致性检查实现与分析**：在 VINS-Mono 光流跟踪基础上添加前向-后向一致性检查。分别用阈值 $\epsilon$ = 0.5, 1.0, 2.0, 5.0 像素，统计被剔除的点比例。将被剔除的点标红、被保留的点标绿，可视化在图像上——被剔除的点是否多集中在遮挡边界、运动物体区域？

3. **setMask 均匀化效果量化**：实现 VINS-Mono 的 `setMask()`。在一张纹理不均匀的图像上（一半书架一半白墙），分别用 (a) `goodFeaturesToTrack(150)` 不带 mask 和 (b) `setMask` + `goodFeaturesToTrack` 补充至 150 个点。将图像分为 8×8 网格，统计每格中的特征点数。计算 64 个网格中特征点数的标准差（越小越均匀）和覆盖率（非空网格数/64）。

---

## 28.5 多视图几何 ⭐⭐⭐

> 本节解决的问题：给定两帧图像中的匹配特征点对，如何恢复相机的相对运动（旋转和平移），以及如何将 2D 匹配点三角化为 3D 空间点？

### 动机：从 2D 到 3D 的桥梁

到目前为止，特征检测和光流跟踪给了我们大量的 2D-2D 点对——"帧 A 中的 $(u_1, v_1)$ 对应帧 B 中的 $(u_2, v_2)$"。但 SLAM 需要的是 3D 信息：相机移动了多少？场景点在空间中的位置？多视图几何就是解决"2D → 3D"这个根本问题的数学工具箱。

**如果没有多视图几何**，单目 SLAM 就不可能存在——一张 2D 图像已经完全丢失了深度信息（射线上所有点投影到同一个像素），只有通过两帧（或多帧）之间的几何约束，才能利用三角测量原理恢复深度。这就是为什么多视图几何被称为视觉 SLAM 的"理论基石"。

### 核心数学：本质矩阵与基础矩阵

**极几何约束（Epipolar Constraint）** 是所有多视图几何方法的出发点。

**直觉理解**：空间中一个点 $P$ 在相机 1 中投影到像素 $\mathbf{p}_1$。从相机 1 的光心看，$\mathbf{p}_1$ 对应一条射线——$P$ 一定在这条射线上。这条射线在相机 2 的图像中投影为一条直线（**极线**）。因此，$P$ 在相机 2 中的投影 $\mathbf{p}_2$ 一定在这条极线上。这就是极几何约束的几何含义。

**数学推导**：

设两个相机的相对变换为旋转 $R \in SO(3)$ 和平移 $t \in \mathbb{R}^3$（从相机 1 到相机 2）。空间点 $P$ 在两个相机坐标系下分别为 $P_1$ 和 $P_2$：

$$P_2 = R P_1 + t$$

两侧左乘 $[t]_\times$（即对两侧与 $t$ 做叉积）：

$$[t]_\times P_2 = [t]_\times R P_1 + \underbrace{[t]_\times t}_{= t \times t = 0}$$

因此 $[t]_\times P_2 = [t]_\times R P_1$。

两侧左乘 $P_2^T$：

$$\underbrace{P_2^T [t]_\times P_2}_{= P_2 \cdot (t \times P_2) = 0} = P_2^T [t]_\times R P_1$$

左侧为零（向量与其叉积的点积恒为零），因此：

$$P_2^T \underbrace{[t]_\times R}_{= E} P_1 = 0$$

由于 $P_i = \lambda_i \mathbf{x}_i$（$\mathbf{x}_i$ 是归一化相机坐标，$\lambda_i$ 是深度），代入后深度因子消去，得到：

$$\boxed{\mathbf{x}_2^T \, E \, \mathbf{x}_1 = 0}$$

其中 **本质矩阵（Essential Matrix）** 为：

$$E = [t]_\times R$$

反对称矩阵 $[t]_\times$ 的定义：

$$[t]_\times = \begin{bmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{bmatrix}$$

**基础矩阵（Fundamental Matrix）**：

如果匹配点 $\mathbf{p}_1, \mathbf{p}_2$ 是像素坐标（未归一化），由 $\mathbf{x}_i = K_i^{-1} \mathbf{p}_i$ 代入极几何约束：

$$(K_2^{-1} \mathbf{p}_2)^T E (K_1^{-1} \mathbf{p}_1) = 0$$

$$\mathbf{p}_2^T \underbrace{K_2^{-T} E K_1^{-1}}_{= F} \mathbf{p}_1 = 0$$

即 $\boxed{F = K_2^{-T} E K_1^{-1}}$。当两帧使用同一相机时 $K_1 = K_2 = K$。

**$E$ 和 $F$ 的性质对比：**

| 性质 | 本质矩阵 $E$ | 基础矩阵 $F$ |
|------|-------------|-------------|
| 自由度 | 5（3 旋转 + 3 平移 - 1 尺度） | 7（9 元素 - 1 尺度 - 1 行列式约束） |
| 秩 | 2 | 2 |
| 奇异值 | $[\sigma, \sigma, 0]$（两个相等非零奇异值） | $[\sigma_1, \sigma_2, 0]$（无特殊约束） |
| 最少求解点数 | 5 对（五点法） | 7 或 8 对 |
| 前提条件 | 需要已知相机内参 $K$ | 不需要内参 |
| SLAM 推荐 | 优先使用（已知 $K$ 时更鲁棒） | 内参未知时的替代方案 |

### findFundamentalMat：VINS-Mono 的外点剔除利器

VINS-Mono 在光流跟踪后，使用 `findFundamentalMat` + RANSAC 来剔除跟踪失败的外点：

```cpp
// VINS-Mono feature_tracker.cpp 中的 RANSAC 外点剔除
if (forw_pts.size() >= 8) {  // 基础矩阵至少需要 8 对点（八点法）
    std::vector<cv::Point2f> un_prev_pts, un_forw_pts;
    // 将像素坐标转换为归一化坐标（去畸变 + 除以内参）
    for (unsigned int i = 0; i < prev_pts.size(); i++) {
        // ... undistortedPoints() 去畸变并归一化 ...
        un_prev_pts.push_back(cv::Point2f(x1, y1));
        un_forw_pts.push_back(cv::Point2f(x2, y2));
    }

    std::vector<uchar> status;
    cv::findFundamentalMat(un_prev_pts, un_forw_pts,
                           cv::FM_RANSAC,     // 使用 RANSAC 方法
                           F_THRESHOLD,        // 极线距离阈值（默认 1.0）
                           0.99,               // 置信度（99%）
                           status);            // 输出：内点(1) / 外点(0)

    // 根据 status 剔除外点
    reduceVector(forw_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}
```

**一个微妙但重要的细节**：VINS-Mono 传入 `findFundamentalMat` 的是**归一化坐标**（已去内参），而非原始像素坐标。当 $K = I$（内参为单位阵）时，$F = K^{-T} E K^{-1} = E$。所以 VINS-Mono 实际上是在用 `findFundamentalMat` 估计本质矩阵。这可能是历史原因——VINS-Mono 开发时（约 2017 年），`findEssentialMat` 在 OpenCV 中的接口不如 `findFundamentalMat` 成熟。

**RANSAC 工作流程回顾**：

```text
RANSAC 估计基础矩阵的迭代过程：
1. 随机采样 8 对匹配点（八点法最小集）
2. 用这 8 对点估计候选 F：构建 8×9 线性方程组，SVD 求解
3. 对所有 N 对点计算极线距离：d_i = |p2^T · F · p1| / ||F·p1||_2
4. 统计 d_i < threshold 的内点数 n_inlier
5. 如果 n_inlier 是目前最大的，保存该 F 和内点集
6. 重复步骤 1-5，迭代次数由 k = log(1-confidence) / log(1-w^8) 决定
   其中 w = n_inlier / N 是当前最佳内点比例
7. 最终用所有内点重新估计 F（用所有内点的八点法，更稳定）
```

### findEssentialMat + recoverPose：从匹配到位姿

在已知相机内参的 SLAM 场景中，`findEssentialMat` 比 `findFundamentalMat` 更推荐——$E$ 只有 5 个自由度（vs $F$ 的 7 个），可以用五点法求解，RANSAC 每次只需采样 5 对点，迭代更快、对外点更鲁棒。stella_vslam 在地图初始化时使用这对函数：

```cpp
// stella_vslam 风格的初始化
cv::Mat E, R, t, mask;

// 步骤 1：估计本质矩阵 E
E = cv::findEssentialMat(
    pts1, pts2,              // 匹配点对（像素坐标）
    K,                        // 3×3 相机内参矩阵
    cv::RANSAC,              // 使用 RANSAC
    0.999,                    // 置信度 99.9%
    1.0,                      // 阈值：1.0 像素
    mask                      // 输出内点掩码
);

// 步骤 2：从 E 分解出 R, t
int inlier_count = cv::recoverPose(E, pts1, pts2, K, R, t, mask);
// R: 3×3 旋转矩阵（从相机 1 到相机 2）
// t: 3×1 平移向量（归一化到单位长度 ||t||=1）
// inlier_count: 通过正深度检验的内点数
```

**`recoverPose` 的四解问题与正深度检验**：

从 $E$ 的 SVD 分解 $E = U \Sigma V^T$（其中 $\Sigma = \text{diag}(\sigma, \sigma, 0)$），可以得到两个 $R$ 和两个 $t$，共 4 种组合：

$$R_1 = U W V^T, \quad R_2 = U W^T V^T, \quad t_1 = +U_3, \quad t_2 = -U_3$$

其中 $W = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$，$U_3$ 是 $U$ 的第三列。

4 种 $(R, t)$ 组合中，只有 1 种能让三角化的 3D 点同时在两个相机前方（正深度）。`recoverPose` 对每种组合三角化若干点，选择正深度点最多的组合。返回值 `inlier_count` 就是该最佳组合的正深度内点数——如果这个数太小（如 <10 或 < 总点数的 50%），说明初始化失败（可能是纯旋转运动、退化场景等）。

**尺度不确定性——单目 SLAM 的根本限制**：

`recoverPose` 返回的平移 $t$ 是**单位向量**（$\|t\|=1$）。这不是算法的缺陷，而是单目相机的物理限制：从 $\mathbf{x}_2^T E \mathbf{x}_1 = 0$ 来看，$E$ 乘以任意非零标量不改变约束，因此 $t$ 的绝对尺度是不可观测的。

直觉上：一个小物体在近处的拍摄结果与一个大物体在远处完全相同——单目相机无法区分这两种情况。这就是为什么纯单目 SLAM 有"尺度漂移"问题，也是 VINS-Mono 需要 IMU 来恢复真实尺度的根本原因。

### solvePnPRansac：从已知 3D 地图点恢复位姿

当 SLAM 已经建立了 3D 地图后，新来一帧图像时可以直接通过 3D-2D 对应估计相机位姿——这就是 PnP（Perspective-n-Point）问题。与 $E$/$F$ 需要两帧匹配不同，PnP 只需要一帧图像和已知的 3D 点。

```cpp
// stella_vslam 重定位中使用 PnP
cv::Mat rvec, tvec;  // 旋转向量（Rodrigues）和平移向量
cv::Mat inliers;     // 内点索引

bool success = cv::solvePnPRansac(
    objectPoints,              // N×3 的 3D 世界坐标 (CV_64FC1 或 CV_32FC1)
    imagePoints,               // N×2 的 2D 像素坐标
    K,                          // 3×3 相机内参矩阵
    distCoeffs,                 // 畸变系数（可以是空 Mat）
    rvec,                       // 输出：3×1 Rodrigues 旋转向量
    tvec,                       // 输出：3×1 平移向量
    false,                      // useExtrinsicGuess：是否用 rvec/tvec 初始值
    100,                        // iterationsCount：RANSAC 最大迭代次数
    8.0,                        // reprojectionError：重投影误差阈值（像素）
    0.99,                       // confidence：置信度
    inliers,                    // 输出：内点索引
    cv::SOLVEPNP_ITERATIVE     // 求解方法
);

// 将 Rodrigues 旋转向量转为旋转矩阵
cv::Mat R_cv;
cv::Rodrigues(rvec, R_cv);  // 3×1 向量 → 3×3 矩阵
```

**关键事实：ORB-SLAM3 不使用 `cv::solvePnPRansac`。** ORB-SLAM3 在 `PnPsolver.cc` 中自研了 EPnP 和 MLPnP 算法，原因包括：

1. **自定义 RANSAC 框架**：ORB-SLAM3 使用概率性终止条件——根据当前内点比例动态调整迭代次数，比 OpenCV 的固定迭代更高效
2. **EPnP 效率**：EPnP 将 N 个 3D 点表示为 4 个虚拟控制点的加权和，将 $O(N)$ 问题转化为 $O(1)$ 问题，比迭代法快得多
3. **MLPnP 广角支持**：MLPnP（Maximum Likelihood PnP）支持非中心投影模型（鱼眼、全景相机），而 OpenCV 的 `solvePnP` 只支持针孔模型
4. **与系统深度集成**：ORB-SLAM3 的 PnP 求解器需要与词袋模型、共可见图等系统组件紧密配合

stella_vslam 同样自研了基于 opengv 的 PnP 求解器（`pnp_solver.cc`），设计哲学与 ORB-SLAM3 类似——追求与自身 RANSAC 框架的深度集成。

**OpenCV 提供的 PnP 求解方法对比：**

| 方法 | 最少点数 | 特点 | 适用场景 |
|------|---------|------|---------|
| `SOLVEPNP_ITERATIVE` | 4 | Levenberg-Marquardt 迭代优化 | 通用，需要好的初始值 |
| `SOLVEPNP_EPNP` | 4 | 非迭代，O(N) 线性求解 | 速度快，不需要初始值 |
| `SOLVEPNP_P3P` | 3 | 最小集求解，4 个候选解 | RANSAC 内部使用，采样效率最高 |
| `SOLVEPNP_AP3P` | 3 | P3P 的改进版 | RANSAC 内部使用，数值更稳定 |

### triangulatePoints：从 2D-2D 恢复 3D 空间坐标

当有两帧图像的匹配点和已知的相机位姿时，可以通过三角化恢复 3D 点坐标。这是 SLAM 初始化和新地图点创建的核心操作。

```cpp
// 三角化的完整示例
// P1, P2 是 3×4 的投影矩阵：P = K * [R|t]
cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);     // 第一帧：[I|0]
cv::Mat Rt;
cv::hconcat(R, t, Rt);                             // 拼接 [R|t]
cv::Mat P2 = K * Rt;                                // 第二帧：[R|t]

cv::Mat points4D;  // 输出：4×N 齐次坐标（CV_32F 或 CV_64F）
cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

// 齐次坐标 → 欧氏坐标
std::vector<cv::Point3f> points3D;
for (int i = 0; i < points4D.cols; i++) {
    float w = points4D.at<float>(3, i);
    if (std::abs(w) < 1e-6) continue;  // w≈0 表示点在无穷远
    float X = points4D.at<float>(0, i) / w;
    float Y = points4D.at<float>(1, i) / w;
    float Z = points4D.at<float>(2, i) / w;
    if (Z > 0)  // 只保留相机前方的点
        points3D.push_back(cv::Point3f(X, Y, Z));
}
```

**`triangulatePoints` 的数学原理（DLT 方法）**：

对于匹配点 $\mathbf{p}_1 = (u_1, v_1, 1)^T$ 和 $\mathbf{p}_2 = (u_2, v_2, 1)^T$，投影方程为：

$$\lambda_1 \mathbf{p}_1 = P_1 \mathbf{X}, \quad \lambda_2 \mathbf{p}_2 = P_2 \mathbf{X}$$

其中 $\mathbf{X} = (X, Y, Z, 1)^T$ 是 3D 点的齐次坐标。为了消去未知的深度 $\lambda$，利用 $\mathbf{p} \times (P \mathbf{X}) = 0$（向量与其标量倍的叉积为零），每对匹配点给出：

$$\begin{bmatrix} u_1 \mathbf{p}_1^{3T} - \mathbf{p}_1^{1T} \\ v_1 \mathbf{p}_1^{3T} - \mathbf{p}_1^{2T} \\ u_2 \mathbf{p}_2^{3T} - \mathbf{p}_2^{1T} \\ v_2 \mathbf{p}_2^{3T} - \mathbf{p}_2^{2T} \end{bmatrix} \mathbf{X} = \mathbf{0}$$

其中 $\mathbf{p}_i^{kT}$ 是投影矩阵 $P_i$ 的第 $k$ 行。这是 $4 \times 4$ 的齐次线性方程组 $A\mathbf{X} = \mathbf{0}$，通过 SVD 分解 $A = U \Sigma V^T$，$\mathbf{X}$ 取 $V$ 的最后一列（对应最小奇异值）。

这就是 DLT（Direct Linear Transform）方法——简洁高效，但对噪声敏感。更鲁棒的方法（如 optimal triangulation）需要考虑两个视图的重投影误差最小化。

**三角化精度与基线的关系**：

三角化的深度估计精度正比于基线长度 $b$（两帧之间的平移距离），反比于深度 $Z$ 的平方：

$$\sigma_Z \propto \frac{Z^2}{b \cdot f}$$

其中 $f$ 是焦距。这意味着：
- **基线太短**（如连续两帧之间位移仅 1 cm）→ 深度不确定性极大，三角化的 3D 点"散布"在很长的射线上
- **基线太长**（如两帧之间旋转超过 30 度）→ 匹配点很少或匹配不准确

这就是为什么 SLAM 初始化通常需要等到相机有足够平移后才触发——ORB-SLAM3 会检查视差角是否超过阈值（通常 > 1 度）。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：triangulatePoints 输出的齐次坐标忘记归一化**
>
> **错误做法**：直接取 `points4D` 的前三行作为 (X, Y, Z) 坐标。
>
> **现象**：得到的 3D 点坐标完全错误——可能差几个数量级。因为齐次坐标 $(X, Y, Z, W)$ 中 $W$ 通常不为 1（SVD 输出的向量是归一化到 $\|\mathbf{X}\|=1$ 的，不是 $W=1$）。
>
> **正确做法**：对每一列，除以第四个分量 $W$。同时检查 $|W| < \epsilon$ 的退化情况（点在无穷远处）和 $Z/W < 0$ 的情况（点在相机后方）。

> ⚠️ **编程陷阱：recoverPose 返回的 t 是单位向量，不能直接当真实平移用**
>
> **错误做法**：`cv::recoverPose(E, pts1, pts2, K, R, t, mask); trajectory.push_back(t);`
>
> **现象**：单目 SLAM 的轨迹长度与真实轨迹不一致——第一段平移被归一化为 1.0，后续帧的平移也各自被归一化，导致轨迹形状可能大致正确但尺度逐帧跳变。
>
> **根本原因**：极几何约束 $\mathbf{x}_2^T E \mathbf{x}_1 = 0$ 中，$E$ 的尺度是任意的——乘以任意非零标量不改变约束。$\|t\|=1$ 是归一化约定，不是真实尺度。
>
> **正确做法**：在 VIO 系统中，用 IMU 积分提供的平移量恢复真实尺度。在纯单目系统中，需通过已知物体尺寸、地面平面约束、或其他先验信息确定尺度。

> 💡 **概念误区：认为 findEssentialMat 和 findFundamentalMat 效果完全一样**
>
> **新手想法**："反正 $E$ 和 $F$ 只差一个内参变换，用哪个都行吧？"
>
> **实际上**：当已知内参时，$E$ 只有 5 个自由度（RANSAC 用五点法，每次采样 5 对点），$F$ 有 7 个自由度（需要 7-8 对点）。$E$ 的低自由度带来两个优势：(1) RANSAC 每次迭代用更少的点，在相同置信度下需要更少的迭代次数；(2) 解空间更小，对外点更鲁棒。在已知内参的 SLAM 场景中，应优先使用 `findEssentialMat`。

> 🧠 **思维陷阱：认为"ORB-SLAM3 肯定用了 cv::solvePnP"**
>
> **新手想法**："PnP 是标准算法，ORB-SLAM3 不可能自己写吧？"
>
> **实际上**：ORB-SLAM3 的 `PnPsolver.cc`（约 800 行）完全自研了 EPnP 和 MLPnP 实现，**不调用任何 OpenCV 的 solvePnP 函数**。自研的原因是需要与系统的自定义 RANSAC 框架（概率性终止、自定义阈值）和非针孔相机模型深度集成。stella_vslam 同样自研了 PnP 求解器（基于 opengv 库），而非直接调用 `cv::solvePnPRansac`。不同系统在"自研 vs 调库"上的选择反映了"灵活性 vs 维护成本"的工程权衡——不存在绝对的好坏。

### 练习

1. **$E$ 和 $F$ 的数值验证**：用 `findEssentialMat` 和 `findFundamentalMat` 分别估计同一组匹配点的矩阵。验证关系 $F = K^{-T} E K^{-1}$（在数值误差范围内，Frobenius 范数差 < 0.01）。对 $E$ 做 SVD 分解，检查奇异值是否满足 $[\sigma, \sigma, 0]$ 的约束（两个非零奇异值的比值应接近 1.0）。

2. **完整初始化流水线**：实现一个简单的单目 VO 初始化流程：(1) `goodFeaturesToTrack` 检测 200 个特征 → (2) `calcOpticalFlowPyrLK` 跟踪到下一帧 → (3) `findEssentialMat` 估计 $E$ → (4) `recoverPose` 分解 $R, t$ → (5) `triangulatePoints` 获得 3D 点云。用两张有明显平移基线的图像测试，用 matplotlib（Python 端）或 PCL（C++ 端）可视化三角化得到的 3D 点云和两个相机位姿。

3. **PnP 精度分析**：生成一组已知的 3D-2D 对应——从已知位姿投影 100 个随机 3D 点得到 2D 像素坐标，然后添加不同水平的高斯噪声（$\sigma$ = 0.5, 1.0, 2.0, 5.0 像素）。用 `solvePnPRansac` 估计位姿，计算旋转误差（角度）和平移误差（欧氏距离）。分别测试 `SOLVEPNP_ITERATIVE`、`SOLVEPNP_EPNP`、`SOLVEPNP_AP3P` 三种方法，绘制"噪声水平 vs 误差"曲线，观察哪种方法对噪声最鲁棒。
## 28.6 畸变校正 ⭐⭐

> **本节解决的问题**：真实相机镜头会引入畸变，使得直线在图像上弯曲、特征点位置偏移。如果不做畸变校正，SLAM 中的多视图几何（本质矩阵、PnP、三角化）全部会因为输入点不准而崩溃。本节系统讲解 SLAM 中三种主流畸变校正方案及其在真实项目中的选型逻辑。

### 28.6.1 相机模型回顾：从针孔到鱼眼

**针孔模型（Pinhole）** 是最简单的投影模型。一个三维点 $\mathbf{P} = [X, Y, Z]^T$ 经过针孔投影后，其归一化坐标为：

$$
x = X / Z, \quad y = Y / Z
$$

然后经过内参矩阵 $K$ 映射到像素坐标：

$$
u = f_x \cdot x + c_x, \quad v = f_y \cdot y + c_y
$$

但真实镜头存在径向畸变和切向畸变。OpenCV 的标准畸变模型（也叫 Brown-Conrady 模型）用 5 个参数 $(k_1, k_2, p_1, p_2, k_3)$ 描述：

$$
\begin{aligned}
r^2 &= x^2 + y^2 \\
x_d &= x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2(r^2 + 2x^2) \\
y_d &= y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2p_2 xy
\end{aligned}
$$

这里 $(x_d, y_d)$ 是畸变后的归一化坐标。**去畸变就是从 $(x_d, y_d)$ 反解出 $(x, y)$**——注意这是一个非线性逆问题，没有解析解，OpenCV 内部用迭代法求解。

**鱼眼模型** 的视场角（FOV）可以超过 180 度，针孔模型的 $r = f \tan\theta$ 在 $\theta \to 90°$ 时趋于无穷，根本无法表达。鱼眼镜头使用等距投影（equidistant projection）：

$$
r = f \cdot \theta
$$

**Kannala-Brandt（KB）模型** 是 ORB-SLAM3 对鱼眼相机的默认选择，使用 8 个参数（因此也叫 KB8）。它对投影角度 $\theta$ 做多项式展开：

$$
d(\theta) = \theta + k_1 \theta^3 + k_2 \theta^5 + k_3 \theta^7 + k_4 \theta^9
$$

其中 $\theta = \arctan(r)$，$r = \sqrt{x^2 + y^2}$ 是三维点在归一化平面上的距离。投影到像素的坐标为：

$$
u = f_x \cdot d(\theta) \cdot \frac{x}{r} + c_x, \quad v = f_y \cdot d(\theta) \cdot \frac{y}{r} + c_y
$$

| 模型 | 参数数量 | 适用 FOV | 代表 SLAM 项目 |
|------|---------|---------|---------------|
| Pinhole + Brown-Conrady | 5 (k1,k2,p1,p2,k3) | < 120° | ORB-SLAM3 (Pinhole), VINS-Mono |
| KB8 (Kannala-Brandt) | 4 (k1-k4) + fx,fy,cx,cy | > 150° | ORB-SLAM3 (KannalaBrandt8) |
| OpenCV Fisheye | 4 (k1-k4) | > 150° | OpenVINS, Basalt |
| MEI (Unified) | 5 + xi | 全向 | OKVIS |

**为什么 ORB-SLAM3 有两套畸变校正代码？** 因为针孔模型和鱼眼模型的数学完全不同。如果用针孔模型的 `cv::undistortPoints` 处理鱼眼图像，畸变系数的含义完全对不上——径向畸变用的是 $r^2, r^4, r^6$ 级数，而 KB 模型用的是 $\theta^3, \theta^5, \theta^7, \theta^9$ 级数。混用会导致校正后的点位置偏差极大，SLAM 直接跟丢。

### 28.6.2 方案一：undistortPoints（逐点校正）

这是最直接的方式——对每个特征点单独做去畸变。ORB-SLAM3 在 `Frame::UndistortKeyPoints()` 中使用此方案处理 Pinhole 模型：

```cpp
// ORB-SLAM3 Frame.cc 中 Pinhole 模型的去畸变
// 输入：mvKeysUn 中存储了带畸变的特征点像素坐标
// 输出：将像素坐标转为去畸变后的归一化坐标，再重投影回像素

// Step 1: 将 N 个特征点打包成 Nx2 的 Mat
cv::Mat mat(N, 2, CV_32F);
for (int i = 0; i < N; i++) {
    mat.at<float>(i, 0) = mvKeysUn[i].pt.x;  // 畸变像素 u
    mat.at<float>(i, 1) = mvKeysUn[i].pt.y;  // 畸变像素 v
}

// Step 2: reshape 为 Nx1x2，这是 undistortPoints 要求的格式
mat = mat.reshape(2);  // channels=2, rows=N

// Step 3: 调用 OpenCV 去畸变
cv::undistortPoints(
    mat,           // 输入：畸变的像素坐标
    mat,           // 输出：去畸变的归一化坐标（如果不传 P）
    mK,            // 相机内参矩阵 3x3
    mDistCoef,     // 畸变系数 [k1, k2, p1, p2, k3]
    cv::Mat(),     // R：不做旋转校正
    mK             // P：重投影矩阵，传 K 使输出仍为像素坐标
);

// Step 4: reshape 回来，取出结果
mat = mat.reshape(1);  // channels=1, cols=2
for (int i = 0; i < N; i++) {
    mvKeysUn[i].pt.x = mat.at<float>(i, 0);
    mvKeysUn[i].pt.y = mat.at<float>(i, 1);
}
```

**函数签名深入理解**：

```cpp
void cv::undistortPoints(
    InputArray src,         // 畸变点，Nx1x2 或 1xNx2
    OutputArray dst,        // 去畸变点
    InputArray cameraMatrix,// 内参 K
    InputArray distCoeffs,  // 畸变系数
    InputArray R = noArray(),  // 可选的旋转矩阵（立体校正用）
    InputArray P = noArray()   // 可选的新投影矩阵
);
```

**P 参数的关键作用**：
- 如果 `P = noArray()`：输出是**归一化坐标** $(x, y)$，即 $Z=1$ 平面上的坐标
- 如果 `P = K`：输出是**去畸变后的像素坐标** $(u, v)$
- 如果 `P = P_new`（`getOptimalNewCameraMatrix` 的输出）：输出是重投影到新内参下的像素坐标

ORB-SLAM3 传了 `P = mK`，所以输出直接是去畸变后的像素坐标，可以直接用于后续的特征匹配和多视图几何。

**内部算法**：`undistortPoints` 内部对每个点执行迭代反解。对于 Brown-Conrady 模型，它大约迭代 5-10 次牛顿法。OpenCV 源码（`modules/calib3d/src/undistort.dispatch.cpp`）中可以看到迭代终止条件是残差小于阈值或达到最大迭代次数。

### 28.6.3 方案二：initUndistortRectifyMap + remap（预计算查找表）

逐点校正在特征法 SLAM 中够用（每帧只有几百个特征点），但在**直接法**或**稠密法**中，需要对图像的每一个像素都做去畸变——一帧 640x480 的图像就有 30 万像素，逐点迭代太慢了。

**解决方案**：预计算一张"查找表"（映射表），记录去畸变后图像的每个像素对应原始图像的哪个位置，然后用 `remap` 高效完成像素重采样。这就是 FAST-LIVO2 采用的方案。

```cpp
// FAST-LIVO2 风格的预计算映射表方案
// 只在初始化时调用一次（相机参数不变）

cv::Mat map1, map2;

// Step 1: 计算映射表
cv::initUndistortRectifyMap(
    K,                  // 原始内参
    distCoeffs,         // 畸变系数
    cv::Mat(),          // R：不做旋转（单目情况）
    K,                  // 新内参（可用 getOptimalNewCameraMatrix 调整）
    imageSize,          // 输出图像尺寸
    CV_16SC2,           // map1 的数据类型（定点数，速度最快）
    map1, map2          // 输出映射表
);

// Step 2: 每帧调用 remap（极快，纯查表+插值）
cv::Mat undistorted;
cv::remap(
    distortedImage,     // 输入：畸变图像
    undistorted,        // 输出：去畸变图像
    map1, map2,         // 预计算的映射表
    cv::INTER_LINEAR    // 双线性插值
);
```

**为什么 `CV_16SC2` 比 `CV_32FC1` 快？**

`CV_16SC2` 是定点数（fixed-point）表示：map1 存储整数部分（最近邻像素坐标），map2 存储小数部分（插值权重）。这样 `remap` 内部只需要做整数寻址 + 查表插值，避免了浮点除法。在 ARM 平台（如 Jetson）上，定点数方案比浮点方案快 2-3 倍。

| 方案 | 适用场景 | 每帧耗时 (640x480) | 代表项目 |
|------|---------|-------------------|---------|
| undistortPoints | 特征法，几百个点 | ~0.1 ms | ORB-SLAM3 |
| initUndistortRectifyMap + remap | 直接法/稠密法，整幅图像 | ~1-2 ms（预计算后） | FAST-LIVO2 |
| undistort（封装函数） | 简单场景 | ~3-5 ms（每次重算） | 教学演示 |

### 28.6.4 方案三：cv::fisheye::undistortPoints（鱼眼校正）

ORB-SLAM3 在 `KannalaBrandt8` 相机模型下使用此函数：

```cpp
// ORB-SLAM3 KannalaBrandt8::unproject 的核心逻辑
// 输入：畸变的像素坐标
// 输出：归一化三维射线方向

// 注意：cv::fisheye 命名空间下的函数，畸变系数含义不同！
cv::fisheye::undistortPoints(
    distortedPts,       // 输入：畸变像素坐标 Nx1x2
    undistortedPts,     // 输出：去畸变归一化坐标
    K,                  // 内参
    D,                  // 鱼眼畸变系数 [k1, k2, k3, k4]（4个！不是5个）
    cv::Mat(),          // R
    cv::Mat()           // P（不传，输出归一化坐标）
);
```

**关键区别**：`cv::fisheye::undistortPoints` 和 `cv::undistortPoints` 虽然名字相似，但内部算法完全不同：

| 比较项 | cv::undistortPoints | cv::fisheye::undistortPoints |
|--------|--------------------|-----------------------------|
| 畸变模型 | Brown-Conrady (r^2, r^4, r^6) | Kannala-Brandt (theta^3, theta^5, theta^7, theta^9) |
| 畸变系数数量 | 4-14 个 | 恰好 4 个 |
| 反解算法 | 牛顿迭代 | 牛顿迭代（对 theta 求解） |
| 适用 FOV | < 120 度 | > 150 度，可超 180 度 |
| 命名空间 | cv:: | cv::fisheye:: |

**ORB-SLAM3 的一个争议**：在 GitHub Issue #303 中，有开发者质疑 ORB-SLAM3 在 KB8 模式下是否正确使用了 `cv::fisheye::undistortPoints`。实际上 ORB-SLAM3 对 KB8 模型实现了自己的 `unproject` 方法，用牛顿迭代直接反解 $d(\theta)$ 多项式，而不是直接调用 OpenCV 的鱼眼函数。这是因为 KB8 和 OpenCV `cv::fisheye` 实际上使用相同的 4 参数公式（都到 $\theta^9$）。ORB-SLAM3 自研 `unproject` 的原因是为了与其自定义相机模型抽象层深度集成，而非多项式阶数差异。

### 28.6.5 立体校正（Stereo Rectification）

双目 SLAM 需要将左右相机图像校正到同一水平面上，使得对应点的搜索从二维退化为一维（沿水平极线搜索）。OpenCV 提供了完整的立体校正流水线：

```cpp
// Step 1: 立体标定（获取外参）
cv::stereoCalibrate(
    objectPoints, imagePointsL, imagePointsR,
    K_L, D_L, K_R, D_R,
    imageSize, R, T, E, F,
    cv::CALIB_FIX_INTRINSIC  // 已知内参时固定
);

// Step 2: 计算校正变换
cv::Mat R1, R2, P1, P2, Q;
cv::stereoRectify(
    K_L, D_L, K_R, D_R,
    imageSize, R, T,
    R1, R2, P1, P2, Q,       // 输出
    cv::CALIB_ZERO_DISPARITY, // 主点对齐
    0                         // alpha=0 裁掉黑边
);

// Step 3: 预计算映射表（左右各一套）
cv::Mat map1L, map2L, map1R, map2R;
cv::initUndistortRectifyMap(K_L, D_L, R1, P1, imageSize, CV_16SC2, map1L, map2L);
cv::initUndistortRectifyMap(K_R, D_R, R2, P2, imageSize, CV_16SC2, map1R, map2R);

// Step 4: 每帧校正（只需 remap）
cv::remap(imgL, rectifiedL, map1L, map2L, cv::INTER_LINEAR);
cv::remap(imgR, rectifiedR, map1R, map2R, cv::INTER_LINEAR);
```

**校正后有什么好处？**
- 左右图像的对应点在**同一行**上（极线水平）
- 可以用 `cv::StereoBM` 或 `cv::StereoSGBM` 直接计算视差图
- ORB-SLAM3 双目模式中，校正后的图像使得特征匹配只需在同一行搜索，效率提升 10 倍以上

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱：undistortPoints 的输入格式错误**
> - **错误做法**：传入 `Nx2` 的 `cv::Mat`，期望得到正确结果
> - **现象**：编译通过但输出全是 0 或 NaN
> - **根本原因**：`undistortPoints` 要求输入是 `Nx1` 的 2 通道 Mat（即 `CV_32FC2`），或者 `1xN` 的 2 通道 Mat。`Nx2` 的单通道 Mat 会被错误解析
> - **正确做法**：用 `mat.reshape(2)` 将 `Nx2` 单通道转为 `Nx1` 双通道，或者直接用 `std::vector<cv::Point2f>`（OpenCV 自动处理）

> **💡 概念误区：认为 cv::undistortPoints 和 cv::fisheye::undistortPoints 可以互换**
> - **新手想法**："反正都是去畸变，换个函数也行吧"
> - **实际上**：两个函数使用完全不同的数学模型。针孔模型的畸变系数 $[k_1, k_2, p_1, p_2, k_3]$ 如果传给鱼眼函数，会被解释为 $[k_1, k_2, k_3, k_4]$——含义完全错误
> - **后果**：校正后的点位置偏差可达数十像素，SLAM 初始化必然失败
> - **正确做法**：先确认相机模型类型，再选择对应的函数

> **🧠 思维陷阱：认为"去畸变越精确越好"**
> - **新手想法**："我应该用 14 参数模型获得最精确的校正"
> - **实际上**：更多参数不一定更好。参数过多时标定容易过拟合——在标定板图像上精度极高，但在其他场景泛化性差。ORB-SLAM3 对 Pinhole 只用 5 个参数，对 KB8 只用 4 个畸变参数，这是实践中的平衡
> - **正确思维**：先用标准模型标定，检查重投影误差；如果误差 < 0.5 像素就够了，不需要更复杂的模型

### 练习

1. **[编程]** 用 `cv::undistortPoints` 对一组特征点做去畸变。分别传 `P = noArray()` 和 `P = K`，验证输出的区别。解释为什么归一化坐标 $(x, y)$ 满足 $u = f_x \cdot x + c_x$。

2. **[编程]** 实现预计算映射表方案：用 `initUndistortRectifyMap` 生成映射表，然后用 `remap` 对整幅图像去畸变。对比 `CV_16SC2` 和 `CV_32FC1` 两种 map 类型的处理速度（用 `cv::getTickCount` 计时）。

3. **[思考]** 为什么 FAST-LIVO2 选择预计算映射表方案而不是逐点校正？如果 FAST-LIVO2 改用特征法（提取 ORB 特征），还需要用映射表方案吗？分析两种方案的计算量差异。

---

## 28.7 DNN 模块与深度学习特征 ⭐⭐⭐

> **本节解决的问题**：传统手工特征（ORB、FAST、BRIEF）在光照变化大、纹理弱、重复纹理等场景下表现不佳。深度学习特征（SuperPoint、SuperGlue、LightGlue）在这些场景下显著优于传统方法，但如何在 C++ SLAM 系统中高效部署深度学习模型？OpenCV 的 DNN 模块提供了一个轻量级入口。

### 28.7.1 cv::dnn 模块概述

OpenCV 的 DNN 模块是一个**推理引擎**（inference-only），不支持训练，但支持加载主流框架导出的模型：

```cpp
// 加载 ONNX 模型的基本流程
cv::dnn::Net net = cv::dnn::readNetFromONNX("superpoint.onnx");

// 设置后端和目标设备
net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);  // 或 DNN_BACKEND_CUDA
net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);         // 或 DNN_TARGET_CUDA

// 准备输入 blob
cv::Mat blob = cv::dnn::blobFromImage(
    grayImage,        // 输入图像
    1.0 / 255.0,      // 缩放因子（归一化到 [0,1]）
    cv::Size(640, 480),// 目标尺寸
    cv::Scalar(0),     // 均值减除
    false,             // swapRB
    false              // crop
);

net.setInput(blob);

// 前向推理
cv::Mat output = net.forward("output_name");
```

**为什么 SLAM 社区关注 cv::dnn？** 因为它是**零依赖**的——不需要安装 PyTorch、TensorFlow 或 ONNX Runtime，只需要 OpenCV 本身。对于嵌入式平台（如 Jetson Nano），减少依赖意味着减少编译时间和磁盘占用。

### 28.7.2 SuperPoint + LightGlue 的 C++ 部署

**LightGlue-OnnxRunner** 是目前社区最成熟的 C++ 方案之一，它展示了如何在 C++ 中部署 SuperPoint（特征提取）+ LightGlue（特征匹配）的完整流水线。

**架构模式**：

```text
┌──────────┐    ┌──────────────┐    ┌───────────┐
│ 原始图像  │───>│ SuperPoint   │───>│ keypoints │
│ (灰度)   │    │ (ONNX模型)   │    │ descriptors│
└──────────┘    └──────────────┘    └─────┬─────┘
                                          │
┌──────────┐    ┌──────────────┐    ┌─────▼─────┐
│ 原始图像  │───>│ SuperPoint   │───>│ keypoints │
│ (灰度)   │    │ (ONNX模型)   │    │ descriptors│
└──────────┘    └──────────────┘    └─────┬─────┘
                                          │
                ┌──────────────┐    ┌─────▼─────┐
                │ LightGlue    │<───│ 拼接输入   │
                │ (ONNX模型)   │    └───────────┘
                └──────┬───────┘
                       │
                ┌──────▼───────┐
                │ 匹配对 + 置信度│
                └──────────────┘
```

LightGlue-OnnxRunner 提供两种推理模式：
- **端到端模式（end-to-end）**：将 SuperPoint + LightGlue 合并为一个 ONNX 模型，一次推理完成
- **解耦模式（decouple）**：SuperPoint 和 LightGlue 分别是独立的 ONNX 模型，灵活组合

```cpp
// LightGlue-OnnxRunner 解耦模式的核心流程（简化）
// 注意：实际项目使用 ONNX Runtime 而非 cv::dnn，
// 因为 cv::dnn 不支持 LightGlue 中的某些算子

// 1. SuperPoint 特征提取
Ort::Session sp_session(env, "superpoint.onnx", session_options);
auto sp_output = sp_session.Run(run_options, input_names, &input_tensor, 1,
                                output_names, num_outputs);
// sp_output 包含 keypoints (Nx2) 和 descriptors (Nx256)

// 2. LightGlue 匹配
// 将两幅图像的 keypoints + descriptors 拼接为 LightGlue 的输入
Ort::Session lg_session(env, "lightglue.onnx", session_options);
auto lg_output = lg_session.Run(run_options, lg_input_names, lg_inputs, num_inputs,
                                lg_output_names, num_outputs);
// lg_output 包含 matches (Mx2) 和 match_scores (Mx1)
```

**为什么 LightGlue-OnnxRunner 不用 cv::dnn 而用 ONNX Runtime？** 因为 LightGlue 模型内部使用了 `ScatterND`、`GridSample` 等算子，OpenCV 4.x 的 DNN 模块尚未完全支持。这是 cv::dnn 的主要局限——算子覆盖率不如 ONNX Runtime 或 TensorRT。

### 28.7.3 CPU vs GPU 与趋势分析

| 推理后端 | 延迟 (SuperPoint 640x480) | 优点 | 缺点 |
|----------|--------------------------|------|------|
| cv::dnn (CPU) | ~30-50 ms | 零依赖，跨平台 | 算子支持有限 |
| ONNX Runtime (CPU) | ~15-25 ms | 算子全面，优化好 | 额外依赖 |
| ONNX Runtime (CUDA) | ~3-8 ms | 速度快 | 需要 NVIDIA GPU |
| TensorRT | ~1-4 ms | 最快（FP16/INT8 量化） | 仅 NVIDIA，编译慢 |

**趋势判断**：
1. **OpenCV 正在追赶**：OpenCV GSoC 2025-2026 项目中，将 LightGlue、ALIKED、XFeat 集成到 DNN 模块是优先任务之一
2. **ONNX 成为桥梁**：PyTorch -> ONNX -> C++ 推理 是目前最主流的部署路径
3. **混合前端是趋势**：用深度学习做特征提取 + 传统方法做几何验证（如 RANSAC），兼顾精度与可解释性
4. **端侧部署**：在 Jetson Orin、RK3588 等边缘 AI 芯片上，TensorRT / RKNN 成为 SLAM 深度学习特征的首选推理后端

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱：cv::dnn::blobFromImage 的归一化参数搞错**
> - **错误做法**：不做归一化或归一化范围不对（[0,255] vs [0,1] vs [-1,1]）
> - **现象**：模型输出全是零或数值异常
> - **根本原因**：深度学习模型训练时的输入预处理必须在推理时严格复现。SuperPoint 训练时归一化到 [0,1]，如果推理时不除以 255，输入分布完全不同
> - **正确做法**：查阅模型的训练代码或文档，确认 `scalefactor`、`mean`、`swapRB` 参数

> **💡 概念误区：认为深度学习特征一定比传统特征好**
> - **新手想法**："SuperPoint 在论文里碾压 ORB，所以应该全面替换"
> - **实际上**：SuperPoint 在**困难场景**（光照变化、模糊）下优势明显，但在正常光照下 ORB 的速度快 10-100 倍，精度也足够。ORB-SLAM3 用自定义 ORB 在 TUM/KITTI 上就能跑出 SOTA 结果
> - **正确思维**：根据部署平台和场景选择。嵌入式 + 正常光照 -> ORB；高端 GPU + 困难场景 -> SuperPoint/LightGlue

### 练习

1. **[编程]** 用 `cv::dnn::readNetFromONNX` 加载一个简单的图像分类模型（如 MobileNet），完成前向推理。观察 `blobFromImage` 的 `scalefactor` 和 `mean` 对输出的影响。

2. **[调研]** 阅读 LightGlue-OnnxRunner 的 GitHub 仓库（`github.com/OroChippw/LightGlue-OnnxRunner`），画出其推理流水线图。思考：如果要集成到 ORB-SLAM3 中替换 ORB 前端，需要修改哪些模块？

3. **[思考]** 为什么 OpenCV DNN 模块选择"只做推理、不做训练"？这个设计决策对 SLAM 开发者意味着什么？

---

## 28.8 SLAM 代码精读 ⭐⭐⭐

> **本节解决的问题**：前面几节讲了 OpenCV 的各个 API，但在真实 SLAM 系统中，这些 API 是怎样被组合使用的？本节精读三个代表性项目的核心文件，展示 OpenCV API 在工程中的完整编排逻辑。

### 28.8.1 ORB-SLAM3 ORBextractor.cc 完整解析

`ORBextractor.cc` 是 ORB-SLAM3 中最核心的文件之一（约 1200 行），它**没有直接调用 `cv::ORB::create()`**，而是从头实现了一套定制化的 ORB 特征提取流水线。

**为什么不直接用 OpenCV 的 ORB？** 因为 OpenCV 的 `cv::ORB::create()` 不提供以下关键能力：
- 双阈值 FAST 策略（先 20 后 7 的退化机制）
- 四叉树均匀化（保证特征在图像上均匀分布）
- 自定义 IC_Angle 方向计算（比 OpenCV 默认实现更高效）
- 硬编码 BRIEF 采样模式（确保跨平台一致性）

**代码结构精读**：

```text
ORBextractor.cc 逻辑流程：
+--------------------------------------------------+
| 1. 构建图像金字塔 ComputePyramid()               |
|    - nlevels=8 层，scaleFactor=1.2                |
|    - 每层做 GaussianBlur(7, 7, 2, 2)             |
|    - 边界扩展 EDGE_THRESHOLD=19 像素             |
+--------------------------------------------------+
| 2. 逐层提取 FAST 角点                             |
|    - 将每层图像划分为 30x30 像素的 cell           |
|    - 每个 cell 先用 iniThFAST=20 检测             |
|    - 如果检测到 0 个角点 -> 退化到 minThFAST=7    |
+--------------------------------------------------+
| 3. 四叉树均匀化 DistributeOctTree()              |
|    - 将所有角点放入根节点                          |
|    - 递归分裂：每个节点 -> 4 个子节点              |
|    - 分裂终止：节点数 >= 目标特征数 nFeatures      |
|    - 每个叶子节点保留响应值最大的 1 个角点         |
+--------------------------------------------------+
| 4. 计算方向 IC_Angle()                            |
|    - 使用 Intensity Centroid 方法                  |
|    - 在 PATCH_SIZE=31 的圆形区域内计算灰度重心    |
|    - 方向 = atan2(m01, m10)                       |
|    - 预计算的 umax[] 数组定义圆形边界             |
+--------------------------------------------------+
| 5. 计算 BRIEF 描述子 computeDescriptors()         |
|    - 256 对采样点，硬编码在 bit_pattern_31_[] 中   |
|    - 根据 Step 4 的方向旋转采样模式               |
|    - 输出 32 字节（256 bit）二进制描述子           |
+--------------------------------------------------+
```

**关键代码片段——双阈值 FAST**：

```cpp
// ORBextractor.cc 中的双阈值策略（简化）
void ORBextractor::ComputeKeyPointsOctTree(
    vector<vector<KeyPoint>>& allKeypoints)
{
    const int CELL_SIZE = 30;  // 每个 cell 的像素边长

    for (int level = 0; level < nlevels; ++level) {
        // 将当前层划分为 cell 网格
        for (int i = 0; i < nRows; i++) {
            for (int j = 0; j < nCols; j++) {
                // 提取当前 cell 的 ROI
                cv::Mat cellImage = mvImagePyramid[level](cellROI);

                // 第一次尝试：高阈值
                cv::FAST(cellImage, cellKeypoints, iniThFAST, true);

                // 如果高阈值没检测到任何角点 -> 降低阈值重试
                if (cellKeypoints.empty()) {
                    cv::FAST(cellImage, cellKeypoints, minThFAST, true);
                }
                // 即使 minThFAST 也没检测到，该 cell 就跳过
            }
        }
        // 四叉树均匀化
        allKeypoints[level] = DistributeOctTree(/*...*/);
    }
}
```

**为什么是 30x30 的 cell？** 太小的 cell（如 10x10）会导致 FAST 检测不到有意义的角点（角点需要周围 16 个像素的上下文）；太大的 cell（如 100x100）则失去了分区检测的意义（弱纹理区域被强纹理区域压制）。30x30 是经验值，约等于 FAST 检测半径的 2 倍。

### 28.8.2 VINS-Mono feature_tracker.cpp 完整流程

VINS-Mono 的光流前端是**教科书级别的 KLT 跟踪实现**，完整流程如下：

```text
feature_tracker.cpp 的 readImage() 完整流程：
+--------------------------------------------------+
| 1. CLAHE 均衡化                                   |
|    clahe = cv::createCLAHE(3.0, cv::Size(8,8))   |
|    clahe->apply(img, img)                         |
+--------------------------------------------------+
| 2. KLT 光流跟踪                                   |
|    cv::calcOpticalFlowPyrLK(                      |
|        prev_img, cur_img,                         |
|        prev_pts, cur_pts,                         |
|        status, err,                               |
|        cv::Size(21, 21),   // 搜索窗口            |
|        3                   // 金字塔层数           |
|    )                                              |
+--------------------------------------------------+
| 3. 边界检查 + 状态过滤                             |
|    for each point:                                |
|      if (status[i] == 0) remove                   |
|      if (pt outside image) remove                 |
+--------------------------------------------------+
| 4. RANSAC 外点剔除                                |
|    cv::findFundamentalMat(                        |
|        prev_pts, cur_pts,                         |
|        cv::FM_RANSAC, 1.0, 0.99, status          |
|    )                                              |
|    // 用基础矩阵 RANSAC 剔除不符合对极约束的点     |
+--------------------------------------------------+
| 5. setMask 均匀化                                  |
|    - 按 track_cnt 降序排列（跟踪次数越多越优先）   |
|    - 对每个保留点画 cv::circle(mask, pt,           |
|      MIN_DIST, 0, -1)  // 填充黑色圆             |
|    - 新检测的点只能出现在 mask 非零区域            |
+--------------------------------------------------+
| 6. 补充新特征                                     |
|    n_max_cnt = MAX_CNT - cur_pts.size()           |
|    cv::goodFeaturesToTrack(                       |
|        img, new_pts, n_max_cnt,                   |
|        0.01,        // qualityLevel               |
|        MIN_DIST,    // minDistance                 |
|        mask         // 只在非零区域检测            |
|    )                                              |
+--------------------------------------------------+
| 7. 去畸变 + 计算归一化坐标                         |
|    cv::undistortPoints(pts, un_pts, K, D)         |
|    // 或通过自定义相机模型反投影                    |
+--------------------------------------------------+
```

**VINS-Mono 的 setMask 是一个非常聪明的设计**。它解决的核心问题是：如果纹理丰富的区域（如桌面上的书本）吸引了大量特征，而纹理弱的区域（如白墙）没有特征，那么相机在白墙方向上的位姿约束就很弱。setMask 通过"已有特征点周围画黑洞"的方式，强制 `goodFeaturesToTrack` 在空白区域补充新特征，实现均匀分布。

**一个重要缺陷**：VINS-Mono 的光流跟踪**没有实现正反向一致性检查**（forward-backward check）。所谓正反向检查是：先从帧 A 跟踪到帧 B，再从帧 B 反向跟踪回帧 A，如果来回的位置偏差超过阈值就认为跟踪失败。OpenVINS 的 `TrackKLT.cpp` 实现了这个检查，稳健性更好。

### 28.8.3 OpenVINS TrackKLT.cpp 的改进

OpenVINS 相比 VINS-Mono 的主要改进：

```cpp
// OpenVINS TrackKLT.cpp 的关键改进

// 改进 1: 预构建金字塔（避免重复计算）
std::vector<cv::Mat> imgpyr;
cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels);
// 存储金字塔，下一帧作为 prev_pyr 直接复用
// VINS-Mono 每次调用 calcOpticalFlowPyrLK 都重建金字塔

// 改进 2: CLAHE clipLimit=10.0（更激进的均衡化）
auto clahe = cv::createCLAHE(10.0, cv::Size(8, 8));
// VINS-Mono 用 clipLimit=3.0
// 更高的 clipLimit 在低光照下增强对比度更明显

// 改进 3: 自定义相机模型输出 Jacobian
// OpenVINS 不依赖 cv::undistortPoints
// 而是实现了自己的 camera model 类，
// 可以输出去畸变的 Jacobian（用于协方差传播）
```

### 28.8.4 SLAM OpenCV TOP 10 函数

基于对 ORB-SLAM3、VINS-Mono、OpenVINS、FAST-LIVO2、stella_vslam 等项目的源码调研，整理出 SLAM 中使用频率最高的 10 个 OpenCV 函数：

| 排名 | 函数 | 用途 | 代表项目 |
|------|------|------|---------|
| 1 | `cv::Mat` 构造/操作 | 图像与矩阵的统一容器 | 所有项目 |
| 2 | `cv::FAST` | 角点检测 | ORB-SLAM3 (双阈值) |
| 3 | `calcOpticalFlowPyrLK` | KLT 光流跟踪 | VINS-Mono, OpenVINS |
| 4 | `goodFeaturesToTrack` | Shi-Tomasi 角点检测 | VINS-Mono (补充特征) |
| 5 | `findFundamentalMat` | RANSAC 外点剔除 | VINS-Mono |
| 6 | `undistortPoints` | 逐点去畸变 | ORB-SLAM3 (Pinhole) |
| 7 | `cv::resize` | 图像金字塔缩放 | ORB-SLAM3 (金字塔) |
| 8 | `createCLAHE` + `apply` | 自适应直方图均衡化 | ORB-SLAM3, OpenVINS |
| 9 | `GaussianBlur` | 描述子前平滑 | ORB-SLAM3 (7x7 sigma=2) |
| 10 | `cvtColor` | 彩色->灰度转换 | 所有视觉前端 |

**补充说明**：
- `initUndistortRectifyMap` + `remap` 在直接法项目（FAST-LIVO2）中排名靠前
- `findEssentialMat` + `recoverPose` 在需要初始化的单目系统（stella_vslam）中关键
- `solvePnPRansac` 是 OpenCV 标准 PnP 接口（注：stella_vslam 实际使用自研 opengv PnP，ORB-SLAM3 自研 EPnP/MLPnP），但 ORB-SLAM3 **自研了 EPnP/MLPnP 而不调用此函数**

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱：calcOpticalFlowPyrLK 的 status 向量未正确处理**
> - **错误做法**：只检查 `status[i] == 1` 就认为跟踪成功
> - **现象**：跟踪点漂移到图像外部或聚集到一起
> - **根本原因**：`status == 1` 只表示 KLT 迭代收敛了，不代表跟踪正确。点可能收敛到错误位置（如运动模糊区域），也可能跑到图像边界外
> - **正确做法**：检查 status + 边界检查 + 正反向一致性检查 + RANSAC 外点剔除，四层过滤缺一不可

> **🧠 思维陷阱：认为"ORB-SLAM3 自定义实现说明 OpenCV 不好"**
> - **新手想法**："ORB-SLAM3 不用 cv::ORB::create()，说明 OpenCV 的 ORB 实现质量差"
> - **实际上**：OpenCV 的 ORB 是通用实现，适合大多数场景。ORB-SLAM3 自定义是为了极致优化——双阈值退化、四叉树均匀化、硬编码采样模式这些都是针对 SLAM 场景的特化。如果你的项目不需要这些优化，`cv::ORB::create()` 完全够用
> - **正确思维**：先用 OpenCV 标准接口原型验证，确认瓶颈后再考虑自定义实现

### 练习

1. **[代码阅读]** 打开 ORB-SLAM3 的 `ORBextractor.cc`，找到 `DistributeOctTree` 函数。画出当 nFeatures=100 时，四叉树的分裂过程示意图（前 3 轮即可）。

2. **[编程]** 模仿 VINS-Mono 的 setMask 策略，实现一个 `uniformFeatureDetection` 函数：输入图像和目标特征数，输出均匀分布的特征点集合。对比有/无 setMask 时特征分布的可视化效果。

3. **[对比分析]** OpenVINS 的 `buildOpticalFlowPyramid` 预构建金字塔相比 VINS-Mono 直接调用 `calcOpticalFlowPyrLK` 有什么性能优势？写一个基准测试验证。

---

## 28.9 实战 ⭐⭐

> **本节解决的问题**：将前面学到的 API 串联成完整的视觉处理流水线。通过四个由浅入深的实战项目，掌握 OpenCV 在 SLAM 前端中的核心编程范式。

### 28.9.1 实战一：ORB 检测 + 匹配 + RANSAC 流水线

这是特征法 SLAM 前端的最小完整原型：

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

int main() {
    // ---- 1. 读取两幅图像 ----
    cv::Mat img1 = cv::imread("frame1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread("frame2.png", cv::IMREAD_GRAYSCALE);

    // ---- 2. ORB 特征检测与描述 ----
    auto orb = cv::ORB::create(
        1000,       // nfeatures：最多提取 1000 个特征
        1.2f,       // scaleFactor：金字塔缩放因子
        8,          // nlevels：金字塔层数
        31,         // edgeThreshold：边缘排除区域
        0,          // firstLevel
        2,          // WTA_K：BRIEF 每次比较的点数
        cv::ORB::HARRIS_SCORE,  // scoreType
        31,         // patchSize：描述子计算的 patch 大小
        20          // fastThreshold：FAST 角点阈值
    );

    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desc1, desc2;
    orb->detectAndCompute(img1, cv::noArray(), kp1, desc1);
    orb->detectAndCompute(img2, cv::noArray(), kp2, desc2);

    std::cout << "Image 1: " << kp1.size() << " keypoints\n";
    std::cout << "Image 2: " << kp2.size() << " keypoints\n";

    // ---- 3. 暴力匹配 (Hamming 距离) ----
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);  // crossCheck=true
    std::vector<cv::DMatch> matches;
    matcher.match(desc1, desc2, matches);

    // ---- 4. 比率测试过滤（替代方案：knnMatch + Lowe's ratio test）----
    // crossCheck 已做了初步过滤，这里再按距离排序取前 N 个
    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch& a, const cv::DMatch& b) {
                  return a.distance < b.distance;
              });
    int numGoodMatches = std::min((int)matches.size(), 200);
    matches.resize(numGoodMatches);

    // ---- 5. RANSAC 外点剔除 ----
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : matches) {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }

    std::vector<uchar> inlierMask;
    cv::Mat F = cv::findFundamentalMat(
        pts1, pts2,
        cv::FM_RANSAC,
        3.0,         // RANSAC 阈值（像素）
        0.99,        // 置信度
        inlierMask   // 内点 mask
    );

    int numInliers = cv::countNonZero(inlierMask);
    std::cout << "Matches: " << matches.size()
              << " -> Inliers: " << numInliers << "\n";

    // ---- 6. 可视化 ----
    std::vector<cv::DMatch> inlierMatches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (inlierMask[i])
            inlierMatches.push_back(matches[i]);
    }
    cv::Mat vis;
    cv::drawMatches(img1, kp1, img2, kp2, inlierMatches, vis);
    cv::imwrite("matches_result.png", vis);

    return 0;
}
```

**编译命令**：

```bash
g++ -std=c++17 orb_pipeline.cpp -o orb_pipeline \
    $(pkg-config --cflags --libs opencv4)
```

### 28.9.2 实战二：LK 光流简单 VO

用 KLT 光流跟踪实现两帧之间的位姿估计——这是 VINS-Mono 前端的精简版：

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap("video.mp4");
    cv::Mat prevFrame, prevGray;
    cap >> prevFrame;
    cv::cvtColor(prevFrame, prevGray, cv::COLOR_BGR2GRAY);

    // ---- 1. 检测初始特征点 ----
    std::vector<cv::Point2f> prevPts;
    cv::goodFeaturesToTrack(
        prevGray, prevPts,
        200,       // maxCorners
        0.01,      // qualityLevel
        30         // minDistance
    );

    cv::Mat frame, gray;
    while (cap.read(frame)) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ---- 2. KLT 光流跟踪 ----
        std::vector<cv::Point2f> currPts;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(
            prevGray, gray,
            prevPts, currPts,
            status, err,
            cv::Size(21, 21),  // winSize（与 VINS-Mono 相同）
            3                  // maxLevel
        );

        // ---- 3. 过滤：仅保留跟踪成功且在图像内的点 ----
        std::vector<cv::Point2f> goodPrev, goodCurr;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i] &&
                currPts[i].x >= 0 && currPts[i].x < gray.cols &&
                currPts[i].y >= 0 && currPts[i].y < gray.rows) {
                goodPrev.push_back(prevPts[i]);
                goodCurr.push_back(currPts[i]);
            }
        }

        // ---- 4. 用基础矩阵 RANSAC 再过滤 ----
        if (goodPrev.size() >= 8) {
            std::vector<uchar> ransacMask;
            cv::findFundamentalMat(goodPrev, goodCurr,
                                   cv::FM_RANSAC, 1.0, 0.99, ransacMask);
            std::vector<cv::Point2f> filteredPrev, filteredCurr;
            for (size_t i = 0; i < ransacMask.size(); i++) {
                if (ransacMask[i]) {
                    filteredPrev.push_back(goodPrev[i]);
                    filteredCurr.push_back(goodCurr[i]);
                }
            }
            goodPrev = filteredPrev;
            goodCurr = filteredCurr;
        }

        std::cout << "Tracked: " << goodCurr.size() << " points\n";

        // ---- 5. 补充特征（模仿 VINS-Mono 的 setMask 策略）----
        if (goodCurr.size() < 100) {
            cv::Mat mask = cv::Mat::ones(gray.size(), CV_8U) * 255;
            for (const auto& pt : goodCurr) {
                cv::circle(mask, pt, 30, 0, -1);  // 已有点周围画黑洞
            }
            std::vector<cv::Point2f> newPts;
            cv::goodFeaturesToTrack(gray, newPts,
                                    200 - (int)goodCurr.size(),
                                    0.01, 30, mask);
            goodCurr.insert(goodCurr.end(), newPts.begin(), newPts.end());
        }

        // 更新上一帧
        gray.copyTo(prevGray);
        prevPts = goodCurr;
    }
    return 0;
}
```

### 28.9.3 实战三：findEssentialMat + recoverPose 恢复轨迹

在实战二的基础上加入位姿恢复——从二维像素对应关系恢复三维运动：

```cpp
// 已知相机内参（从标定获得）
double fx = 718.856, fy = 718.856;
double cx = 607.1928, cy = 185.2157;
cv::Mat K = (cv::Mat_<double>(3, 3) <<
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);

// 累积位姿
cv::Mat R_total = cv::Mat::eye(3, 3, CV_64F);
cv::Mat t_total = cv::Mat::zeros(3, 1, CV_64F);
std::vector<cv::Point3d> trajectory;

// 在跟踪循环中（接实战二的光流跟踪后）：
if (goodPrev.size() >= 5) {  // 五点法最少需要 5 对点
    // Step 1: 估计本质矩阵
    cv::Mat inlierMask;
    cv::Mat E = cv::findEssentialMat(
        goodCurr, goodPrev,  // 注意顺序：当前帧在前
        K,                    // 相机内参
        cv::RANSAC,           // 方法
        0.999,                // 置信度
        1.0,                  // 阈值（像素）
        inlierMask            // 内点 mask
    );

    // Step 2: 从本质矩阵恢复 R, t
    cv::Mat R, t;
    int numPoseInliers = cv::recoverPose(
        E, goodCurr, goodPrev, K, R, t, inlierMask
    );
    // R: 3x3 旋转矩阵
    // t: 3x1 平移向量（单位向量，尺度未知！）

    // Step 3: 累积位姿（注意：单目 VO 平移尺度未知）
    // 这里简单假设每帧运动尺度相同
    double scale = 1.0;  // 实际应从 GPS/IMU/地面真值获得
    R_total = R * R_total;
    t_total = t_total + scale * R_total * t;

    trajectory.push_back(cv::Point3d(
        t_total.at<double>(0),
        t_total.at<double>(1),
        t_total.at<double>(2)
    ));
}
```

**关键理解——为什么单目 VO 的平移尺度是未知的？**

本质矩阵 $E = [t]_\times R$ 的分解结果中，$t$ 只是一个单位向量（$\|t\| = 1$）。这是因为从二维图像对应关系无法恢复绝对尺度——一个小物体靠近相机和一个大物体远离相机产生的光流完全相同。这就是单目 SLAM 的**尺度模糊性**（scale ambiguity），也是 VINS-Mono 需要 IMU 的根本原因之一。

### 28.9.4 实战四：完整相机标定

SLAM 的一切建立在相机标定的基础上。下面是基于棋盘格的完整标定流程：

```cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // ---- 配置 ----
    cv::Size boardSize(9, 6);       // 棋盘格内角点数
    float squareSize = 0.025f;      // 格子边长（米）

    // ---- 1. 准备三维标定点 ----
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            objp.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // ---- 2. 读取标定图像，检测角点 ----
    std::vector<std::string> imageFiles;
    cv::glob("calib_images/*.png", imageFiles);

    cv::Size imageSize;
    for (const auto& file : imageFiles) {
        cv::Mat img = cv::imread(file);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        imageSize = gray.size();

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners);

        if (found) {
            // 亚像素精化（关键！不做这步标定精度会差很多）
            cv::cornerSubPix(gray, corners, cv::Size(11, 11),
                             cv::Size(-1, -1),
                             cv::TermCriteria(
                                 cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                 30, 0.001));
            objectPoints.push_back(objp);
            imagePoints.push_back(corners);
        }
    }

    std::cout << "Using " << objectPoints.size() << " images for calibration\n";

    // ---- 3. 标定 ----
    cv::Mat K, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(
        objectPoints, imagePoints, imageSize,
        K, distCoeffs, rvecs, tvecs
    );
    std::cout << "RMS reprojection error: " << rms << " pixels\n";
    std::cout << "Camera matrix:\n" << K << "\n";
    std::cout << "Distortion coeffs: " << distCoeffs << "\n";

    // ---- 4. 验证——计算重投影误差分布 ----
    double totalErr = 0;
    for (size_t i = 0; i < objectPoints.size(); i++) {
        std::vector<cv::Point2f> reprojPts;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                          K, distCoeffs, reprojPts);
        double err = cv::norm(imagePoints[i], reprojPts, cv::NORM_L2);
        totalErr += err * err;
    }
    std::cout << "Mean reprojection error: "
              << std::sqrt(totalErr / (objectPoints.size() * boardSize.area()))
              << " pixels\n";

    // 保存标定结果
    cv::FileStorage fs("calibration.yaml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << K;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs.release();

    return 0;
}
```

**标定质量判断标准**：
- 重投影误差 < 0.5 像素：**优秀**，可用于高精度 SLAM
- 重投影误差 0.5-1.0 像素：**合格**，适合一般 VIO
- 重投影误差 > 1.0 像素：**需要重新标定**，检查标定板平整度和图像覆盖率

### ⚠️ 常见陷阱

> **⚠️ 编程陷阱：findEssentialMat 的点顺序搞反**
> - **错误做法**：`findEssentialMat(prevPts, currPts, K, ...)` 然后用 `recoverPose` 的 R, t 累积位姿
> - **现象**：轨迹方向反了或旋转不对
> - **根本原因**：`findEssentialMat` 和 `recoverPose` 的点顺序决定了 R, t 表示的是从哪个帧到哪个帧的运动。搞反后 R 变成了逆旋转
> - **正确做法**：明确约定并保持一致。建议统一用 (curr, prev) 顺序，得到的 R, t 表示从 prev 到 curr 的运动

> **💡 概念误区：认为标定一次就永远有效**
> - **新手想法**："我在实验室标定好了，拿到户外直接用"
> - **实际上**：温度变化会导致镜头焦距微变（热膨胀），机械冲击会改变相机-IMU 外参。工业级 SLAM 系统通常内置在线标定模块
> - **正确做法**：每次更换环境或经过剧烈运动后重新检查标定质量（拍标定板看重投影误差）

### 练习

1. **[编程]** 完成实战一的完整代码，在 KITTI 数据集的两帧图像上运行。对比 `crossCheck=true` 和 Lowe's ratio test（knnMatch + ratio < 0.75）两种匹配策略的内点数量和匹配质量。

2. **[编程]** 在实战三的基础上，添加正反向一致性检查：先从帧 A 跟踪到帧 B，再从帧 B 反向跟踪回帧 A，如果往返误差 > 1 像素则剔除。对比添加前后的轨迹精度。

3. **[完整项目]** 用实战四的标定结果，结合实战三的 VO 流水线，在自己拍摄的视频上运行单目 VO。将输出轨迹可视化（用 `cv::viz` 或保存为 TUM 格式导入 `evo` 工具评估）。

---

## 28.10 视觉前端工程边界与验证清单 ⭐⭐

> **这一节解决什么问题**：OpenCV 前端函数很容易调用成功，但 SLAM 需要的是几何上可信的输出。本节把 `cv::Mat` 生命周期、像素坐标/归一化坐标、RANSAC mask、畸变模型和多线程边界统一成上线前检查表。

### 动机：图像 API 成功不等于几何正确

回顾 通用库·Eigen：`Eigen::Map` 的布局错会让矩阵解释错误；OpenCV 中同样存在 `cv::Mat` 浅拷贝、ROI stride、数据类型和连续性问题。回顾 通用库·Ceres：优化器会相信前端残差；如果前端把畸变像素直接送进本质矩阵估计，后端收到的就是带系统偏差的约束。

如果不做视觉边界验证会怎样？`findEssentialMat()` 可能返回一个矩阵，`recoverPose()` 也可能返回一个 `R,t`，但三角化点大量在相机后方；LK 光流可能返回 status=1，却跟踪到动态物体；标定 YAML 读入成功，却把鱼眼模型当针孔模型使用。

> **本质洞察**：视觉 SLAM 前端的核心边界是"像素观测必须在正确相机模型下进入几何方程"。OpenCV 负责计算，工程代码负责保证输入坐标、畸变状态、数据类型和 mask 语义正确。

### cv::Mat 生命周期与布局边界

| 边界 | 风险 | 推荐验证 |
|------|------|----------|
| 浅拷贝 | 修改一个 Mat 影响另一个 Mat | 需要独立数据时使用 `clone()` |
| ROI | `isContinuous()` 可能为 false | 传给 Eigen/自定义 SIMD 前检查 stride |
| 类型 | `CV_8U`、`CV_32F`、`CV_64F` 混用 | 每个模块入口 `CV_Assert(type)` |
| 通道 | 灰度/彩色通道数不匹配 | `channels()` 检查 |
| 生命周期 | `Mat` 引用外部 buffer | 外部 buffer 必须长于 Mat 使用期 |

这与 通用库·Eigen 的 `Map` 完全类比：`cv::Mat` 头部是一个引用计数视图，数据可能被共享。不要把它当作总是深拷贝的图像对象。

### 多视图几何输入边界

| API | 输入坐标要求 | 常见错误 |
|-----|--------------|----------|
| `findFundamentalMat` | 原始像素坐标 | 误传归一化坐标导致阈值尺度错 |
| `findEssentialMat` | 像素坐标 + K，或归一化坐标 + 单位 K | 去畸变和 K 使用重复 |
| `recoverPose` | 与 E 估计一致的坐标约定 | mask 没有同步过滤 |
| `triangulatePoints` | 投影矩阵与点坐标在同一坐标系 | 用像素点配归一化投影矩阵 |
| `solvePnPRansac` | 3D 点单位与相机坐标一致 | 世界系/相机系方向混淆 |

本质矩阵估计尤其容易犯"双重去畸变"错误：先调用 `undistortPoints` 得到归一化点，又把原始相机矩阵 `K` 传给 `findEssentialMat`。这样几何尺度被重复校正，RANSAC 阈值也不再对应像素误差。

### 几何结果验证模板

下面的函数在 `recoverPose` 后检查三件事：内点数、旋转矩阵正交性、三角化正深度比例。它不会替代完整 BA，但能在前端阶段挡住明显坏的两视图约束。

```cpp
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

void validateRelativePose(const cv::Mat& R,
                          const cv::Mat& t,
                          int inlier_count,
                          int total_matches) {
    if (R.empty() || t.empty()) {
        throw std::runtime_error("recoverPose returned empty R or t");
    }
    if (R.type() != CV_64F || t.type() != CV_64F) {
        throw std::runtime_error("R and t should be CV_64F for geometry checks");
    }

    const double inlier_ratio = static_cast<double>(inlier_count) /
                                std::max(1, total_matches);
    if (inlier_count < 30 || inlier_ratio < 0.3) {
        throw std::runtime_error("too few geometric inliers");
    }

    cv::Mat should_be_identity = R.t() * R;
    const double orth_error = cv::norm(should_be_identity - cv::Mat::eye(3, 3, CV_64F));
    const double det_R = cv::determinant(R);
    if (orth_error > 1e-6 || std::abs(det_R - 1.0) > 1e-6) {
        throw std::runtime_error("R is not a valid rotation matrix");
    }
}
```

对象生命周期边界：`cv::Mat` 以 const 引用传入，只在函数内读取；函数不保存对 Mat 数据的引用。若你需要把 `R` 转成 Eigen 并长期保存，应立即拷贝到 `Eigen::Matrix3d`，不要保存指向 `Mat::data` 的 `Map`。

### RANSAC mask 的语义边界

OpenCV 多个函数都会返回 mask，但 mask 的语义取决于函数：

| 函数 | mask 含义 | 后续处理 |
|------|-----------|----------|
| `calcOpticalFlowPyrLK` | 光流跟踪是否成功 | 还要做前后向一致性和边界检查 |
| `findFundamentalMat` | 满足极线几何的匹配 | 同步过滤 `pts1/pts2/id` |
| `findEssentialMat` | E 矩阵 RANSAC 内点 | 传给 `recoverPose` 继续 cheirality 检查 |
| `solvePnPRansac` | PnP 内点索引或 mask | 用内点重算位姿或进入 BA |

不要把一个 mask 套到已经重新排序的点集上。SLAM 前端通常还维护特征 id、生命周期、速度估计等并行数组；过滤时必须同步更新，否则观测和 id 会错位。

### DNN 与传统前端的工程边界

SuperPoint/LightGlue 等深度特征可以提升弱纹理和大视角变化下的匹配，但它们引入新的边界：

| 边界 | 传统特征 | 深度特征 |
|------|----------|----------|
| 延迟 | CPU 可控，通常毫秒级 | 依赖 GPU/ONNX/TensorRT |
| 可解释性 | 角点/描述子可视化直观 | 网络输出需要额外验证 |
| 部署 | OpenCV core/features2d 足够 | 模型文件、backend、precision |
| 失败模式 | 纹理弱、重复纹理 | 域外图像、模型版本不一致 |

在实时 SLAM 中，DNN 前端应有传统特征回退路径。GPU 繁忙、模型加载失败或输入分辨率不匹配时，系统应降级而不是阻塞整个跟踪线程。

### 练习

1. **坐标边界题**：同一组匹配点，分别用"原始像素 + K"和"undistortPoints 归一化 + 单位 K"估计本质矩阵，验证结果一致。再故意双重使用 K，观察误差。
2. **mask 同步题**：构造 `pts1/pts2/ids/track_cnt` 四个并行数组，用 RANSAC mask 过滤，写单元测试确保 id 与点仍然对应。
3. **跨章综合题**：把 `recoverPose` 的相对位姿输出转换为 通用库·李群manif 的 `SE3d`，作为 通用库·Ceres 位姿图边加入优化，比较加入前后的轨迹闭合误差。

---

## 28.11 ArUco/AprilTag 标记检测 ⭐⭐

### 动机：为什么 SLAM 系统需要人工标记？

在大多数场景下，SLAM 依靠自然特征（角点、边缘、纹理）进行定位。但在以下场景中，自然特征不足以提供可靠定位：

- **室内弱纹理环境**：白墙、单色地板、重复纹理走廊
- **标定场景**：多传感器外参标定、手眼标定
- **精确定位需求**：工业机器人的末端定位精度 < 1mm
- **初始化和重定位**：SLAM 系统冷启动或跟踪丢失后的恢复

ArUco 和 AprilTag 是两种广泛使用的人工标记系统。OpenCV 4.x 的 `aruco` 模块提供了两者的完整实现。

### ArUco 与 AprilTag 的对比

| 特性 | ArUco | AprilTag |
|------|-------|----------|
| 字典大小 | 多种（4x4 到 7x7，50-1000 个） | 多种（16h5, 25h9, 36h11 等） |
| 检测速度 | 快（OpenCV 原生） | 略慢（但更鲁棒） |
| 鲁棒性 | 对遮挡敏感 | 对遮挡和光照变化更鲁棒 |
| 误检率 | 较低 | 更低（Hamming distance 设计） |
| 位姿估计 | `cv::aruco::estimatePoseSingleMarkers` | 同一 API |

```cpp
#include <opencv2/aruco.hpp>

// ArUco 标记检测与位姿估计
void detectAndEstimatePose(const cv::Mat& image,
                           const cv::Mat& camera_matrix,
                           const cv::Mat& dist_coeffs) {
    // 创建 ArUco 字典
    auto dictionary = cv::aruco::getPredefinedDictionary(
        cv::aruco::DICT_6X6_250);
    auto parameters = cv::aruco::DetectorParameters::create();

    // 检测标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

    if (!ids.empty()) {
        // 位姿估计（每个标记独立）
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            corners, 0.05f,  // 标记边长（米）
            camera_matrix, dist_coeffs,
            rvecs, tvecs);

        // rvecs[i] 和 tvecs[i] 是第 i 个标记相对相机的位姿
        // 可转换为 SE(3) 用于 SLAM 回环或重定位
    }
}
```

**在 SLAM 中的典型用途**：ORB-SLAM3 支持 ArUco 标记作为地图中的固定路标（map markers），提供绝对位姿约束。这在大型室内场景的多房间建图中非常有用——每个房间放一个 ArUco 标记就能提供全局一致性约束。

### OpenCV 与 Isaac ROS 视觉管道的关系 ⭐⭐⭐

NVIDIA Isaac ROS 是面向机器人的 GPU 加速视觉计算框架。它和 OpenCV 不是替代关系，而是互补的：

| 层次 | 工具 | 角色 |
|------|------|------|
| 基础图像处理 | OpenCV | 格式转换、ROI 裁剪、简单滤波 |
| GPU 加速处理 | Isaac ROS / VPI | 特征检测、光流、畸变校正 |
| DNN 推理 | TensorRT (via Isaac) | SuperPoint/SuperGlue 推理 |
| 3D 感知 | Isaac ROS SLAM | Visual SLAM、深度估计 |

在实时系统中，OpenCV 通常负责 CPU 上的轻量操作和数据格式转换，而计算密集型任务（如大分辨率图像的特征检测）交给 Isaac ROS 的 GPU 管道。两者通过 `cv_bridge`（ROS 1）或 `image_transport`（ROS 2）桥接数据。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：ArUco 标记检测中忘记去畸变**
>
> **错误做法**：直接在原始畸变图像上做 `estimatePoseSingleMarkers`
>
> **现象**：角点位置因畸变偏移，位姿估计误差随标记离图像中心的距离增大。在鱼眼镜头上误差可达数十度。
>
> **正确做法**：先用 `cv::undistort` 或 `cv::remap` 去畸变，再做标记检测。或者将去畸变集成到位姿估计的投影模型中。

### 练习

1. **实践题**：打印 5 个 ArUco 标记（DICT_6X6_250），贴在桌面上，编写程序实时检测所有标记并在画面上绘制坐标轴。

2. **定位题**：把 ArUco 标记的世界坐标设为已知，用 `solvePnP` 计算相机位姿，与 `estimatePoseSingleMarkers` 的结果比较。

---

## 28.12 DNN 推理部署进阶 ⭐⭐⭐

### 动机：从"能跑"到"能用"

28.7 节介绍了 OpenCV DNN 模块的基本用法。但在实时机器人系统中，仅仅能加载和运行模型是不够的——还需要考虑推理延迟、GPU/CPU 后端选择、模型格式转换和推理结果的后处理。

### OpenCV DNN 的后端与目标设备

OpenCV 4.x 支持多种推理后端：

| 后端 | 设备 | 优势 | 适用场景 |
|------|------|------|---------|
| `DNN_BACKEND_OPENCV` | CPU | 无额外依赖 | 轻量部署、开发调试 |
| `DNN_BACKEND_CUDA` | NVIDIA GPU | 高吞吐 | Jetson/桌面 GPU |
| `DNN_BACKEND_INFERENCE_ENGINE` | Intel CPU/GPU/VPU | OpenVINO 优化 | Intel 平台 |
| `DNN_BACKEND_VKCOM` | Vulkan GPU | 跨平台 GPU | 非 NVIDIA GPU |

```cpp
// 选择 CUDA 后端
auto net = cv::dnn::readNetFromONNX("superpoint.onnx");
net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);  // 或 DNN_TARGET_CUDA_FP16

// 输入预处理
cv::Mat blob = cv::dnn::blobFromImage(
    gray, 1.0/255.0,             // 归一化到 [0, 1]
    cv::Size(640, 480),          // 网络输入尺寸
    cv::Scalar(0),               // 均值
    true, false);                // swapRB, crop
net.setInput(blob);

// 前向推理
cv::Mat output = net.forward("output_name");
```

**OpenCV DNN vs TensorRT vs ONNX Runtime**

| 框架 | 优势 | 劣势 |
|------|------|------|
| OpenCV DNN | 无额外依赖，接口简洁 | 性能不如专用框架 |
| TensorRT | NVIDIA GPU 最优性能 | 仅 NVIDIA，编译繁琐 |
| ONNX Runtime | 跨平台，多后端 | 额外依赖 |

> **本质洞察**：在 SLAM 系统中，DNN 推理通常不是瓶颈——传统特征检测和匹配每帧只需 5-15ms，DNN 特征（SuperPoint）在 GPU 上也是 5-10ms。真正的挑战在于**系统集成**：DNN 前端和传统前端如何共存、GPU 资源如何在 DNN 和其他视觉任务（如立体匹配、光流）之间分配、模型更新如何不中断实时管道。

### ⚠️ 常见陷阱

> ⚠️ **编程陷阱：OpenCV DNN 的 CUDA 后端需要编译时启用**
>
> **错误做法**：用 `apt install libopencv-dev` 安装的 OpenCV，直接设置 `DNN_BACKEND_CUDA`
>
> **现象**：运行时报错 "CUDA backend is not available"。
>
> **根本原因**：Ubuntu 软件源中的 OpenCV 通常不编译 CUDA 支持。需要从源码编译 OpenCV，并在 CMake 中启用 `-DWITH_CUDA=ON -DWITH_CUDNN=ON`。
>
> **正确做法**：检查 `cv::getBuildInformation()` 的输出，确认 CUDA 后端是否可用。

### 练习

1. **部署题**：将一个 ONNX 格式的图像分类模型加载到 OpenCV DNN 中，分别用 CPU 和 CUDA 后端测量推理时间。

2. **集成题**：设计一个 SLAM 前端架构，同时支持传统 FAST 特征和 SuperPoint DNN 特征，通过配置文件切换。要求 DNN 失败时自动回退到传统方法。

---

## 本章小结

本章系统讲解了 OpenCV 在 SLAM 中的核心应用。下表汇总了每个知识点与 SLAM 项目的映射关系：

| 节号 | 主题 | 核心 API | SLAM 项目应用 |
|------|------|---------|--------------|
| 28.1 | cv::Mat 核心 | Mat 构造/clone/ROI/Eigen互转 | 所有项目的基础数据结构 |
| 28.2 | 图像预处理 | cvtColor/createCLAHE/GaussianBlur | ORB-SLAM3 (CLAHE+Blur), OpenVINS (CLAHE clipLimit=10) |
| 28.3 | 特征检测 | FAST/goodFeaturesToTrack/ORB::create | ORB-SLAM3 (FAST双阈值), VINS-Mono (Shi-Tomasi) |
| 28.4 | 光流跟踪 | calcOpticalFlowPyrLK/buildOpticalFlowPyramid | VINS-Mono (21x21,3层), OpenVINS (预构建金字塔) |
| 28.5 | 多视图几何 | findFundamentalMat/findEssentialMat/solvePnPRansac | VINS-Mono (F RANSAC), stella_vslam (E+自研PnP) |
| 28.6 | 畸变校正 | undistortPoints/fisheye::undistortPoints/remap | ORB-SLAM3 (Pinhole+KB8), FAST-LIVO2 (remap) |
| 28.7 | DNN 模块 | dnn::readNetFromONNX/blobFromImage | LightGlue-OnnxRunner (SuperPoint+LightGlue) |
| 28.8 | 代码精读 | 综合 | ORB-SLAM3/VINS-Mono/OpenVINS 三大项目 |
| 28.9 | 实战 | 综合 | ORB Pipeline / LK VO / E+Pose / 标定 |
| 28.11 | ArUco/AprilTag | aruco::detectMarkers/estimatePose | 标定、重定位、固定路标 |
| 28.12 | DNN 部署进阶 | CUDA/OpenVINO 后端、模型优化 | 深度特征实时部署 |

**核心洞察**：
1. OpenCV 在 SLAM 中的角色是**基础设施层**——提供图像处理、特征检测、几何验证的基本工具
2. 顶级 SLAM 系统（ORB-SLAM3）会在 OpenCV 基础上做大量定制化，但底层仍然依赖 OpenCV 的核心数据结构和基础算法
3. 光流法（VINS-Mono）和特征法（ORB-SLAM3）对 OpenCV 的使用模式截然不同——前者重跟踪，后者重检测+匹配
4. 深度学习特征正在进入 SLAM，但短期内不会完全替代传统方法
5. ArUco/AprilTag 标记为弱纹理环境和精确定位场景提供了可靠的补充方案

---

## 累积项目：Mini-LIO 视觉前端模块

> **本章新增模块**：为 Mini-LIO 项目添加视觉前端，实现图像特征的检测、跟踪和去畸变。

在前面章节中，Mini-LIO 已经实现了点云读取（通用库·PCL PCL）、体素滤波、ICP 配准等激光前端模块。本章新增的视觉前端将为后续的视觉-激光融合打下基础。

**新增代码结构**：

```text
mini_lio/
├── src/
│   ├── lidar_frontend/          # 已有：激光前端
│   │   ├── voxel_filter.cpp
│   │   └── icp.cpp
│   └── visual_frontend/         # 本章新增
│       ├── feature_tracker.h    # 特征跟踪器接口
│       ├── feature_tracker.cpp  # KLT 光流跟踪实现
│       ├── camera_model.h       # 相机模型（Pinhole/Fisheye）
│       ├── camera_model.cpp     # 去畸变实现
│       └── image_processor.cpp  # CLAHE + 金字塔预处理
├── config/
│   └── camera.yaml              # 相机内参和畸变系数
└── CMakeLists.txt               # 添加 OpenCV 依赖
```

**功能清单**：

| 模块 | 功能 | 对应本章知识点 |
|------|------|---------------|
| `image_processor` | CLAHE 均衡化 + 图像金字塔 | 28.2 图像预处理 |
| `feature_tracker` | KLT 跟踪 + setMask 均匀化 + RANSAC 外点剔除 | 28.4 光流跟踪 + 28.5 多视图几何 |
| `camera_model` | Pinhole/Fisheye 去畸变 + 预计算映射表 | 28.6 畸变校正 |

**下一步**（软件工程/设计模式与高级惯用法-31）：在软件架构章节中，将视觉前端与激光前端封装为 ROS2 节点，实现多传感器消息同步与融合。

---

## 延伸阅读

### 论文

| 论文 | 相关性 | 难度 |
|------|--------|------|
| Rublee et al., "ORB: An efficient alternative to SIFT or SURF", ICCV 2011 | ORB 描述子原始论文，理解 28.3/28.8 的基础 | ⭐⭐ |
| Shi & Tomasi, "Good Features to Track", CVPR 1994 | VINS-Mono 特征检测的理论基础 | ⭐⭐ |
| Lucas & Kanade, "An Iterative Image Registration Technique", 1981 | KLT 光流的奠基论文 | ⭐⭐⭐ |
| DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description", CVPRW 2018 | 深度学习特征的代表作 | ⭐⭐⭐ |
| Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed", ICCV 2023 | 最新特征匹配，理解 28.7 的基础 | ⭐⭐⭐ |
| Kannala & Brandt, "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses", PAMI 2006 | KB 模型原始论文 | ⭐⭐⭐⭐ |

### 源码推荐阅读

| 文件 | 项目 | 推荐阅读顺序 |
|------|------|-------------|
| `ORBextractor.cc` | ORB-SLAM3 | 第 1 个读，理解特征提取的完整流程 |
| `feature_tracker.cpp` | VINS-Mono | 第 2 个读，理解光流前端的完整流程 |
| `TrackKLT.cpp` | OpenVINS | 第 3 个读，对比 VINS-Mono 的改进 |
| `Frame.cc` (UndistortKeyPoints) | ORB-SLAM3 | 对照 28.6 理解去畸变实现 |
| `modules/calib3d/src/fisheye.cpp` | OpenCV | 深入理解鱼眼去畸变的内部实现 |

### 官方文档

- [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — 畸变校正与多视图几何的完整 API 参考
- [OpenCV DNN Module](https://docs.opencv.org/4.x/d2/d58/tutorial_table_of_content_dnn.html) — DNN 模块教程与模型 Zoo
- [OpenCV Fisheye Camera Model](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html) — 鱼眼相机模型的专用 API
- [LightGlue-OnnxRunner GitHub](https://github.com/OroChippw/LightGlue-OnnxRunner) — C++ 部署 SuperPoint+LightGlue 的参考实现
- [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) — ONNX 格式导出与 TensorRT 优化

---

## 🔧 故障排查手册

| 症状 | 可能原因 | 排查步骤 | 相关章节 |
|------|----------|----------|----------|
| 修改 ROI 后原图也被改变 | `cv::Mat` 浅拷贝和 ROI 共享底层数据 | 1. 检查是否需要 `clone()`；2. 打印 `data` 指针；3. 对独立图像显式深拷贝 | 通用库·Eigen, 通用库·OpenCV |
| LK 光流跟踪点大量丢失 | 金字塔层数/窗口大小不合适，图像曝光变化，初值越界 | 1. 可视化 status；2. 加前后向一致性；3. 调整 winSize/maxLevel；4. 做 CLAHE | 通用库·OpenCV |
| `findEssentialMat` 内点很多但位姿错误 | 坐标去畸变/K 使用不一致，RANSAC 阈值尺度错误 | 1. 明确输入是像素还是归一化点；2. 避免双重去畸变；3. 检查 `recoverPose` 正深度数量 | 通用库·李群manif, 通用库·OpenCV |
| 三角化点大量在相机后方 | `R,t` 方向反了，投影矩阵和点坐标不在同一坐标系 | 1. 检查 $T_{cw}$ vs $T_{wc}$；2. 用 cheirality 检查；3. 对一个合成点做投影回放 | 通用库·李群manif, 通用库·OpenCV |
| 鱼眼图像去畸变后边缘严重拉伸 | 针孔畸变模型误用于鱼眼，或 K/D 参数顺序错 | 1. 区分 `cv::fisheye` 和普通 calib3d；2. 检查 D 维度；3. 用标定板重投影误差验证 | 通用库·OpenCV |
| DNN 特征前端偶发卡顿 | GPU backend 初始化、模型输入尺寸变化、线程争用 | 1. 预热模型；2. 固定输入分辨率；3. 设置传统特征回退路径；4. 分离推理线程 | 通用库·OpenCV |
