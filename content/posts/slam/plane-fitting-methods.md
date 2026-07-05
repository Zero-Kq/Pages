+++
title = '平面拟合方法'
description = "本文将详细介绍如何估计平面参数，并聚焦于两种常用的实现方法：PCA（主成分分析）法和线性最小二乘法。"
date = '2026-07-05'
draft = false
tags = ["slam", "学习笔记"]
categories = ["SLAM"]
slug = ""
aliases = []
series = []
externalLink = ""
toc = true
math = true
+++

## 介绍

在三维点云处理与空间几何分析中，平面拟合是一项基础且关键的任务。给定一组三维点坐标 $\mathbf{p}_i = (x_i, y_i, z_i)^\top$，本文将探讨如何估计平面参数，并重点对比两种最常用的实现方法：**PCA（主成分分析）法**和**线性最小二乘法**。

---

## 平面方程的几种形式

在讨论具体算法前，先回顾一下平面的常见数学表达：

### 一般式（最常用）

$$
ax + by + cz + d = 0
$$

记 $\mathbf{n} = (a, b, c)^\top$ 为平面的法向量。若法向量为单位向量，即 $\|\mathbf{n}\| = 1$，则 $|ax + by + cz + d|$ 就是任意一点到平面的**几何距离**。

### 点法式

$$
\mathbf{n}^\top (\mathbf{p} - \mathbf{c}) = 0
$$

- $\mathbf{c}$：平面上的一点（常取点云的质心）
- $\mathbf{n}$：单位法向量

### 参数形式

$$
\mathbf{p}(u, v) = \mathbf{c} + u\mathbf{e}_1 + v\mathbf{e}_2
$$

其中 $\mathbf{e}_1, \mathbf{e}_2$ 为平面内两个相互正交的方向向量。

---

## 点到平面的距离

给定一个查询点 $\mathbf{p}$ 和拟合出的平面 $(\mathbf{n}, \mathbf{c})$，该点到平面的有符号距离为：

$$
r = \mathbf{n}^\top (\mathbf{p} - \mathbf{c})
$$

在点云配准及位姿优化任务中，$r$ 常常被用作**观测残差**进行最小化。

---

## 方法一：PCA / 协方差特征分解

### 核心思路

如果一组点云近似落在某个平面上，那么它们在沿平面法线方向上的方差一定是最小的。因此，对去中心化后的点集进行 PCA（主成分分析），其**最小特征值对应的特征向量**即为平面的法向量。

### 计算步骤

假设共有 $N$ 个点 $\mathbf{p}_1, \dots, \mathbf{p}_N$。

**Step 1：计算质心**

$$
\bar{\mathbf{p}} = \frac{1}{N}\sum_{i=1}^{N}\mathbf{p}_i
$$

**Step 2：构建协方差矩阵**

$$
\mathbf{C} = \frac{1}{N}\sum_{i=1}^{N}(\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{p}_i - \bar{\mathbf{p}})^\top
$$

在实际工程中，为了支持数据的增量更新，常采用等价的增量形式：

$$
\mathbf{C} = \frac{\mathbf{P}}{N} - \bar{\mathbf{p}}\bar{\mathbf{p}}^\top,\quad
\mathbf{P} = \sum_i \mathbf{p}_i\mathbf{p}_i^\top,\quad
\mathbf{v} = \sum_i \mathbf{p}_i
$$

**Step 3：特征分解**

对协方差矩阵进行特征分解：

$$
\mathbf{C} = \mathbf{V}\mathbf{\Lambda}\mathbf{V}^\top,\quad
\lambda_0 \le \lambda_1 \le \lambda_2
$$

- **法向**：$\mathbf{n} = \mathbf{v}_0$（即最小特征值 $\lambda_0$ 对应的特征向量）
- **平面上一点**：$\mathbf{c} = \bar{\mathbf{p}}$
- **参数 d**：$d = -\mathbf{n}^\top \mathbf{c}$

**Step 4：平面有效性检验**

并非所有点云都能拟合出有效平面。通常需要设定阈值进行判断，例如：
如果最小特征值足够小（如 `λ₀ < 0.01`）且显著小于次小特征值（如 `λ₀ < 0.1 × λ₁`），则认为平面有效；否则认为点云分布不够“平”，应拒绝该拟合结果。

### 方法特点

| 优点 | 缺点 |
|------|------|
| 支持增量维护，无需保存每个历史点坐标 | 拟合的是**整块点集**的平均平面，对混合结构（如墙角、边缘）容易失效 |
| 适合大规模点云统计，抗噪性相对较好 | 至少需要足够多的点（一般建议 $N \ge 5$） |
| 计算过程稳定，实现简单 | - |

---

## 方法二：线性最小二乘

### 核心思路

将平面的一般方程 $ax + by + cz + d = 0$ 在 $d \neq 0$ 时进行归一化：

$$
\frac{a}{d}x + \frac{b}{d}y + \frac{c}{d}z = -1
$$

令未知数向量 $\mathbf{x} = (a/d,\, b/d,\, c/d)^\top$，对于每个点 $\mathbf{p}_i = (x_i, y_i, z_i)^\top$，有：

$$
\begin{bmatrix} x_i & y_i & z_i \end{bmatrix}
\begin{bmatrix} x_1 \\ x_2 \\ x_3 \end{bmatrix}
= -1
$$

将 $N$ 个点堆叠起来，可得到一个超定线性方程组 $\mathbf{A}\mathbf{x} = \mathbf{b}$：

$$
\underbrace{\begin{bmatrix}
x_1 & y_1 & z_1 \\
x_2 & y_2 & z_2 \\
\vdots & \vdots & \vdots \\
x_N & y_N & z_N
\end{bmatrix}}_{\mathbf{A}\,(N\times 3)}
\mathbf{x}
=
\underbrace{\begin{bmatrix} -1 \\ -1 \\ \vdots \\ -1 \end{bmatrix}}_{\mathbf{b}}
$$

### 计算步骤

**Step 1：最小二乘求解**

通过最小二乘法求解该超定方程：

$$
\mathbf{x} = \arg\min_{\mathbf{x}} \|\mathbf{A}\mathbf{x} - \mathbf{b}\|_2^2
$$

**Step 2：还原平面参数 $(a,b,c,d)$**

设求得的解为 $\mathbf{w} = \mathbf{x}$，先得到未归一化的法向 $\tilde{\mathbf{n}} = \mathbf{w}$。然后对其进行归一化：

$$
\mathbf{n} = \frac{\tilde{\mathbf{n}}}{\|\tilde{\mathbf{n}}\|},\quad
d = \frac{1}{\|\tilde{\mathbf{n}}\|}
$$

这样得到的平面系数 $[a, b, c, d]^\top$ 满足单位法向量的前提。

**Step 3：内点严格检验**

通常会对每个参与拟合的点进行距离检查，以确保拟合质量：

$$
|a x_i + b y_i + c z_i + d| \le \epsilon
$$

若任一点超出误差允许范围（如 $\epsilon = 0.1\text{m}$），则拒绝该平面。

### 方法特点

| 优点 | 缺点 |
|------|------|
| 极其适合少量局部点（如 4～5 个 KNN 近邻点） | 需要显式保存并计算点坐标，不能仅依靠统计量 |
| 平面高度贴合查询点的**局部切面** | 若点共线或处于特定退化分布，矩阵 $\mathbf{A}$ 会病态导致求解失败 |
| 内点检验机制更为严格 | 相比增量 PCA，每次都需要进行矩阵求解 |

---

## 两种方法对比汇总

| 项目 | PCA（协方差特征分解） | 线性最小二乘法 |
|------|-------------|----------------------|
| **最少点数** | 建议 $\ge 5$ | 4～5 个即可 |
| **输入数据** | 区域内点集的统计量（可增量更新） | 局部查询到的近邻点坐标 |
| **平面含义** | 区域点集的主导平面 | 局部的切平面 |
| **有效性判断** | 协方差矩阵的特征值阈值 | 每个点到平面的实际几何距离 |

---

## 退化情况与处理对策

在实际数据中，点云的分布往往不理想。下面列出了常见的退化情况及对应表现：

| 点分布形态 | 现象表现 | 处理策略 |
|--------|------|------|
| **共线** | 协方差矩阵秩不足，法向不唯一（PCA 中 $\lambda_0 \approx \lambda_1 \approx 0$） | 检验失败，丢弃该结果 |
| **单点 / 两点** | 自由度不足，无法确定唯一平面 | 数量不足直接跳过拟合 |
| **球状 / 团状** | 三个方向上的方差接近（$\lambda_0$ 不够小） | 特征值检验失败，拒绝平面 |
| **两面混合（如墙角）** | 会拟合出一个“折中”的错误平面 | 残差过大，通过几何检验剔除 |

---

## 最小示例

### PCA 法代码示例

```python
import numpy as np

points = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.01],
    [0.0, 1.0, -0.01],
    [1.0, 1.0, 0.0],
    [0.5, 0.5, 0.0],
])

center = points.mean(axis=0)
cov = (points - center).T @ (points - center) / len(points)
eigval, eigvec = np.linalg.eigh(cov)  # 默认升序排列

normal = eigvec[:, 0]                 # 最小特征值对应的方向
d = -normal @ center

print("normal:", normal)
print("center:", center)
print("d:", d)
```

### 最小二乘法代码示例

```python
import numpy as np

points = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.01],
    [0.0, 1.0, -0.01],
    [1.0, 1.0, 0.0],
])

A = points
b = -np.ones(len(points))

# 最小二乘求解
w = np.linalg.lstsq(A, b, rcond=None)[0]
n = w / np.linalg.norm(w)
d = 1.0 / np.linalg.norm(w)

a, b_, c = n
print(f"拟合平面: {a:.4f}x + {b_:.4f}y + {c:.4f}z + {d:.4f} = 0")
```

---

## 在实际应用中的工作流程

无论是基于网格化的点云处理，还是基于 KNN 的局部处理，平面拟合最终往往都服务于状态估计或位姿优化。一般流程如下：

```plaintext
点云数据输入
       │
       ├─► [PCA法] 统计点集特征 → 提取法向 n 与中心 c
       │         │
       │         └─► 计算残差 r = nᵀ(p - c)，构建优化目标
       │
       └─► [最小二乘法] 提取局部近邻点 → 解算平面系数 abcd
                 │
                 └─► 计算残差 r = a·x + b·y + c·z + d，构建优化目标
```

**关于优化雅可比的补充：**
在进行位姿优化时，假设点 $\mathbf{p}$ 经过位姿（旋转 $\mathbf{R}$，平移 $\mathbf{t}$）变换，已知目标平面的单位法向 $\mathbf{n}$，其对位姿增量的雅可比通常遵循统一的形式：
- 对旋转：$\frac{\partial r}{\partial \delta\boldsymbol{\theta}} = -\mathbf{n}^\top \mathbf{R}\,[\mathbf{p}]_\times$
- 对平移：$\frac{\partial r}{\partial \delta\mathbf{t}} = \mathbf{n}^\top$

---

## 小结

1. **平面拟合的本质**是：从一组空间点坐标中估计平面的法向量 $\mathbf{n}$ 和位置参数（$d$ 或中心点 $\mathbf{c}$）。
2. **PCA 法**考察的是点云在哪个方向最为“扁平”，它非常适合多点聚集的区域，且易于实现增量式统计。
3. **线性最小二乘法**通过建立方程组求解，更擅长对少量局部近邻点进行快速的切平面拟合。
4. 拟合算法本身无法辨别噪声和非平面结构，因此拟合完成后**必须引入几何检验**（如特征值比例、点面距离等）剔除退化情况或外点，才能保证下游任务的稳定。

---

## 参考

- 无
