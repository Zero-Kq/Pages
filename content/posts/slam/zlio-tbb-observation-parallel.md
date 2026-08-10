+++
title = 'ZLIO 观测模型：用 TBB 并行累加点面信息矩阵'
description = "说明 ZLIO 为何用 TBB 替代 OpenMP、任务窃取调度是什么，以及 parallel_for 配合线程局部累加器的用法"
date = '2026-08-10'
draft = false
tags = ["slam", "优化", "C++", "学习笔记"]
categories = ["SLAM"]
toc = true
math = true
+++

## 介绍

ZLIO 把点面观测热路径上的并行，从 OpenMP 换成了 Intel TBB。观测侧要对当前帧大量点做近邻搜索、平面拟合和雅可比外积累加，属于典型的「数据并行」：点与点之间互不依赖，适合多线程分摊。

本文主要讲：为什么用 TBB、它和 OpenMP 差在哪、任务窃取是什么、以及在观测累加里怎么用。

## 为什么用 TBB

点面观测里，每个点大致做同一套工作：变换到世界系、查近邻、拟合平面、算残差、把 $J J^\top$ 和 $-J r$ 累进六维法方程。点数通常上千，单点耗时不均（有的点 KNN 失败很快返回，有的要做 QR），更适合「按区间切分、空闲线程去接活」的调度，而不是固定开 2 个线程硬扫一遍。

选用 TBB 主要有三点：

- **任务调度更适合负载不均**：`parallel_for` 会把下标区间切成 `blocked_range`，线程做完自己的块可以继续拿新块，减少尾部空转
- **C++ 写法更自然**：用 lambda / 函数对象表达并行体，而不是在循环上挂 `#pragma`；局部状态、归约也能用标准容器式接口表达
- **与 SuperLIO 对齐**：同一类「按点并行 → 线程局部累加 → 主线程归并」的路径，便于对照和移植

旧版把线程数写死成 `MP_PROC_NUM=2`，在多核机器上往往吃不满；TBB 默认按硬件并发度调度，也可用环境变量 `TBB_NUM_THREADS` 限制。

## TBB 和 OpenMP 的区别

两者都能做循环并行，但模型不同：

| 维度 | OpenMP | TBB |
| ---- | ------ | --- |
| 形态 | 编译器指令 + 运行时 | 纯 C++ 模板库 |
| 典型写法 | `#pragma omp parallel for` | `tbb::parallel_for(range, body)` |
| 调度 | 静态 / 动态等子句，偏「把循环切开」 | 任务窃取，偏「把工作切成可窃取任务」 |
| 线程数控制 | `omp_set_num_threads` / 环境变量 | 运行时自动，或 `TBB_NUM_THREADS` |
| 归约 | `reduction(+:x)` 等内置子句 | 常用线程局部存储，再手动合并 |
| 嵌套并行 | 有，但易踩坑 | 任务组合更统一，嵌套相对自然 |
| 依赖 | 编译器支持 OpenMP | 链接 `libtbb` |

直观对比：

```cpp
// OpenMP：在循环上挂指令
#pragma omp parallel for
for (int i = 0; i < n; ++i) {
  work(i);
}
```

```cpp
// TBB：把区间和函数对象交给调度器
tbb::parallel_for(tbb::blocked_range<int>(0, n),
                  [&](const tbb::blocked_range<int> &r) {
                    for (int i = r.begin(); i < r.end(); ++i) {
                      work(i);
                    }
                  });
```

对 ZLIO 这类「单点耗时波动大、最后还要累加矩阵」的场景，TBB 的任务窃取和「线程局部累加器」组合更顺手；OpenMP 当然也能做，但旧实现把线程数钉死，且归约主要靠按点写独立槽位，扩展性一般。

## 任务窃取是什么

**任务窃取（work stealing）**是一种多线程调度方式：每个线程先处理自己队列里的任务；做完后去「偷」其他线程尚未完成的任务，减少有人空闲、有人堵死的情况。

### 直观例子

假设 4 个线程扫 4000 个点，粗切成 4 段，每段 1000 点：

| 线程 | 分到的点 | 实际情况 |
| ---- | -------- | -------- |
| T0 | 0–999 | 很多点 KNN 失败，很快扫完 |
| T1 | 1000–1999 | 正常 |
| T2 | 2000–2999 | 大量点要做 QR，很慢 |
| T3 | 3000–3999 | 正常 |

如果是**静态均分、不能偷活**：T0 早早空闲，整帧仍要等最慢的 T2。

如果是**任务窃取**：区间还会再切成更小的块（TBB 的 `blocked_range` 会自动细分）。T0 做完自己的块后，从 T2 队列尾部拿走一块未做的点继续算，大家一起消化慢任务。

### 和普通并行 for 的差别

| 调度方式 | 行为 | 负载不均时 |
| -------- | ---- | ---------- |
| 静态调度 | 事先按块分好，谁分到难活谁做完 | 易出现尾部空转 |
| 任务窃取 | 工作切成可转移的小任务，空闲线程去接别人剩的活 | 利用率更高 |

TBB 的调度器围绕「任务」构建：`parallel_for` 不是简单开 N 个线程各跑一截，而是把范围拆成可窃取任务，用窃取来平衡。这也是观测里单点耗时波动大时，TBB 比「固定线程数硬切」更合适的原因。

补充一点：C++17 的 `std::execution::par` 只保证「可以并行」，并不规定必须用任务窃取，也不规定块怎么切；底层实现因编译器而异。TBB 则明确提供这套模型，调度行为更可预期。

## 如何使用

观测累加可以拆成三步：准备线程局部累加器 → `parallel_for` 扫活跃点 → 主线程归并。

### 线程局部累加器

每个线程维护自己的 $6 \times 6$ / $6 \times 1$ 和有效点计数，避免热循环里对全局矩阵加锁：

```cpp
struct ThreadAccumulator {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 6, 6> hth = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> htr = Eigen::Matrix<double, 6, 1>::Zero();
  int effective_count = 0;
};

tbb::enumerable_thread_specific<ThreadAccumulator> thread_accumulators;
```

`local()` 第一次调用时为当前线程创建副本；并行阶段只写自己的那份。

### parallel_for

```cpp
tbb::parallel_for(
    tbb::blocked_range<int>(0, static_cast<int>(active_indices_.size())),
    [&](const tbb::blocked_range<int> &_range) {
      ThreadAccumulator &accumulator = thread_accumulators.local();
      for (int i = _range.begin(); i < _range.end(); ++i) {
        process_point(i, accumulator); // KNN / 拟合 / 累加 J*J^T 与 -J*r
      }
    });
```

单点累加形式：

$$
J = \begin{bmatrix} p_b \times (R^\top n) \\ n \end{bmatrix},\quad
H^\top H \mathrel{+}= J J^\top,\quad
-H^\top r \mathrel{+}= -J\, r
$$

### 归并结果

```cpp
for (const ThreadAccumulator &accumulator : thread_accumulators) {
  _hth += accumulator.hth;
  _htr += accumulator.htr;
  effective_count += accumulator.effective_count;
}
```

把 $O(N)$ 次对全局矩阵的争用，收成 $O(\text{线程数})$ 次合并。IESKF 后续只消费固定尺寸的 `hth` / `htr`，不再拼与点数成正比的稠密 $H$、$K$。

### 依赖与限制线程数

```cmake
find_package(TBB REQUIRED)
target_link_libraries(${PROJECT_NAME}_lib TBB::tbb ...)
```

```bash
sudo apt install libtbb-dev
# 可选：限制线程数做对比实验
export TBB_NUM_THREADS=4
```

## 参考

- **oneTBB 文档**：https://oneapi-src.github.io/oneTBB/
- **tbb::parallel_for**：https://oneapi-src.github.io/oneTBB/main/tbb_userguide/parallel_for_is_a_concept.html
- **enumerable_thread_specific**：https://oneapi-src.github.io/oneTBB/main/tbb_userguide/usable_thread_local_storage.html
- **OpenMP 规范**：https://www.openmp.org/specifications/
- **Intel oneTBB GitHub**：https://github.com/uxlfoundation/oneTBB
