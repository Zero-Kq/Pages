+++
title = 'IESKF 观测更新：6×6 累加两种推导路径'
description = "从 18 维 IESKF 点面观测出发，推导并对比两种 6×6 信息累加路径，解释合成观测与直接更新的等价条件"
date = '2026-07-10'
draft = false
weight = 4
tags = ["slam", "优化", "学习笔记", "C++"]
categories = ["SLAM"]
toc = true
math = true
+++

## 介绍

在 LIO 的 [IESKF]({{< ref "ieskf-standard-workflow.md" >}}) 点面观测中，每行雅可比只有前 6 列非零，因此 $H^\top H$ 与 $H^\top Z$ 的信息可压缩为 $6 \times 6$ 与 $6 \times 1$ 的汇总量。本文记录并推导由此产生的两种等价计算路径：

| 路径 | 简述 |
|------|------|
| **路径 A** | 累加 $\mathrm{HTVH}$/$\mathrm{HTVr}$ → Cholesky 还原为 6 行合成观测 $H_s, Z_s$ → 走标准更新公式 |
| **路径 B** | 累加 $\mathrm{HTVH}$/$\mathrm{HTVr}$ → 在更新阶段直接消费汇总量，不显式构造 $H$、$Z$、$K$ |

两者观测模型相同，在保留迭代误差（IE）的 $\mathrm{left}+\mathrm{right}$ 形式、使用相同噪声模型和同一轮线性化结果时，与逐点构建 $V$ 行 $H$ 的基准写法代数等价。下文以固定外参的 18 维 LIO 状态为例，逐步展开推导。

## 1. 背景：18 维状态 vs 6 维观测

考虑 18 维误差状态：

| 下标 | 状态 | LiDAR 点面观测 |
|------|------|----------------|
| 0–2 | 旋转 $\delta\theta$ | **有** |
| 3–5 | 位置 $\delta p$ | **有** |
| 6–8 | 速度 $\delta v$ | **无** |
| 9–11 | 陀螺零偏 $b_g$ | **无** |
| 12–14 | 加计零偏 $b_a$ | **无** |
| 15–17 | 重力 $g$ | **无** |

本例将 LiDAR–IMU 外参固定，采用姿态右扰动 $R\boxplus\delta\theta=R\operatorname{Exp}([\delta\theta]_\times)$。点到平面残差直接依赖 $R,p$，每行雅可比 $h_i\in\mathbb R^{1\times18}$ **只有前 6 列非零**；若外参也参与估计，就不能沿用此稀疏结构。

记 $J_i \in \mathbb{R}^6$ 为 $h_i$ 前 6 列的转置：

$$
J_i = \begin{bmatrix} p_{\text{imu},i} \times (R^\top n_i) \\ n_i \end{bmatrix}
$$

其中 $p_{\mathrm{imu},i}$ 是运动补偿后、已用固定外参转到 IMU 系的点，$n_i,c_i$ 是世界系匹配平面的单位法向和面上一点。预测的有符号残差为：

$$
r_i = n_i^\top (R \, p_{\mathrm{imu},i} + p - c_i)
$$

理想观测为零，即 $0\simeq r_i+h_i\Delta x$。因此更新的观测项为 $-KZ$；若程序采用“观测减预测”的残差定义，符号需同步改变。

## 2. 基准写法：逐点构建 $V$ 行 $H$

### 2.1 观测矩阵

$$
H \in \mathbb{R}^{V \times 18}, \quad Z \in \mathbb{R}^{V \times 1}
$$

$$
h_{i,\theta} = -n_i^\top R \, [p_{\text{imu},i}]_\times, \quad h_{i,p} = n_i^\top
$$

### 2.2 IESKF 迭代更新

以本文示例的观测噪声方差 $\sigma^2 = 0.001$ 为例，信息形式为：

$$
P_{\text{inv}} = \left(\frac{P_{\text{in}}}{\sigma^2}\right)^{-1}
$$

$$
M = H^\top H + P_{\text{inv}} \quad (18 \times 18)
$$

$$
K = M^{-1} H^\top \quad (18 \times V)
$$

这里 $P_{\mathrm{in}}=J_{\mathrm{inv}}P^-J_{\mathrm{inv}}^\top$ 是传播先验 $P^-$ 在当前迭代切空间中的表达，$\delta=x^{(j)}\boxminus x^-$，$J_{\mathrm{inv}}$ 是从先验切空间到当前迭代切空间的雅可比逆。各轮使用同一个 $P^-$，但重新计算当前的 $H,Z,J_{\mathrm{inv}}$。IE 形式的两项更新为：

$$
\text{left} = -K Z
$$

$$
\text{right} = -(I - KH) \, J_{\text{inv}} \, \delta
$$

$$
\Delta x = \text{left} + \text{right}
$$

$$
P \leftarrow (I - KH) \, P_{\text{in}}
$$

### 2.3 可压缩的 6 维汇总量

由于 $H$ 每行只有前 6 列非零，有：

$$
\mathrm{HTVH} = \sum_i J_i J_i^\top = \text{（} H^\top H \text{ 的左上角 } 6 \times 6 \text{ 块）} \quad (6 \times 6)
$$

$$
\mathrm{HTVr} = \sum_i J_i r_i = \text{（} H^\top Z \text{ 的前 6 维）} \quad (6 \times 1)
$$

**公共累加逻辑**（两种路径在观测构建阶段相同）：

```cpp
Eigen::Vector6d J;
J.head<3>() = p_imu.cross(R.transpose() * normal);
J.tail<3>() = normal;

HTVH += J * J.transpose();   // 6×6
HTVr += J * residual;        // 6×1
```

**噪声约定**：未加权累加只适用于各有效点独立且具有同一方差 $\sigma^2$ 的情形。把正规方程同乘 $\sigma^2$ 后，$P_{\mathrm{inv}}=\sigma^2P_{\mathrm{in}}^{-1}$，故累加时不能再重复乘 $1/\sigma^2$。不同方差或相关观测应使用 $H^\top R_m^{-1}H$ 与 $H^\top R_m^{-1}Z$。

## 3. 路径 A：累加后 Cholesky 还原合成观测

### 3.1 思路

点云循环内只累加 $\mathrm{HTVH}$/$\mathrm{HTVr}$；循环结束后，用 Cholesky 分解构造 6 行合成观测矩阵，再代入第 2 节的标准更新公式。观测构建与滤波更新在接口上仍可保持分离。

### 3.2 Cholesky 构造

要求合成观测 $H_s, Z_s$ 满足：

$$
H_s^\top H_s \text{ 的 } 6 \times 6 \text{ 块} = \mathrm{HTVH}, \quad H_s^\top Z_s \text{ 的前 6 维} = \mathrm{HTVr}
$$

设 $\mathrm{HTVH}=LL^\top$，其中 $L$ 为下三角阵。为了使 $H_s^\top H_s=LL^\top$，合成观测的左块必须取 $L^\top$，随后解 $LZ_s=\mathrm{HTVr}$：

$$
\mathrm{HTVH} = L L^\top
$$

$$
H_s = \begin{bmatrix} L^\top & 0_{6 \times 12} \end{bmatrix} \in \mathbb{R}^{6 \times 18}
$$

$$
L Z_s = \mathrm{HTVr}
$$

```cpp
Eigen::LLT<Eigen::Matrix6d> llt(HTVH);
if (llt.info() != Eigen::Success) return false;

Eigen::Matrix6d L = llt.matrixL();
H_synth = Eigen::MatrixXd::Zero(6, 18);
H_synth.block<6, 6>(0, 0) = L.transpose();
Z_synth = L.triangularView<Eigen::Lower>().solve(HTVr);
```

### 3.3 代入标准更新

将 $H_s \in \mathbb{R}^{6 \times 18}$、$Z_s \in \mathbb{R}^{6}$ 代入第 2.2 节公式即可，$K$ 的维度由 $18 \times V$ 降为 $18 \times 6$：

```cpp
K = (H_s.transpose() * H_s + (P_in / sigma2).inverse()).inverse() * H_s.transpose();
left  = -K * Z_s;
right = -(I - K * H_s) * J_inv * delta;
P_out = (I - K * H_s) * P_in;
```

### 3.4 等价性

因 $H_s^\top H_s = H^\top H$ 且 $H_s^\top Z_s = H^\top Z$，故信息矩阵 $M$、左项 $KZ$、右项涉及的 $KH$ 与 $V$ 行基准写法相同。合成残差的 $Z_s^\top Z_s$ 不必等于原始 $Z^\top Z$；该项与状态增量无关，不影响本轮最优增量或协方差。

### 3.5 路径 A 的特点

| 代数层面 | 计算层面 |
|----------|----------|
| 与 $V$ 行 $H$ 严格等价 | 仍需显式构造 $6 \times 18$ 的 $H_s$ 并计算 $KH$ |
| 更新公式形式不变，便于对照推导 | Cholesky 要求 $\mathrm{HTVH}$ 正定，退化时分解可能失败 |
| 合成观测保留了"先建 $H$ 再更新"的结构 | 比路径 B 多一步矩阵分解 |

## 4. 路径 B：更新阶段直接消费 $\mathrm{HTVH}$/$\mathrm{HTVr}$

### 4.1 思路

观测构建阶段只输出汇总量 $\mathrm{HTVH}$/$\mathrm{HTVr}$，更新阶段用信息滤波形式直接计算 $\Delta x$ 和 $P$，不再分配 $H$、$Z$、$K$。

### 4.2 核心等价关系

定义 18 维嵌入矩阵（观测信息只在左上角）：

$$
\Lambda = \begin{bmatrix} \mathrm{HTVH} & 0 \\ 0 & 0 \end{bmatrix} \quad (18 \times 18)
$$

则有：

$$
H^\top H = \Lambda, \quad H^\top Z = \begin{bmatrix} \mathrm{HTVr} \\ \mathbf{0} \end{bmatrix}
$$

信息矩阵：

$$
P_{\text{inv}} = \left(\frac{P_{\text{in}}}{\sigma^2}\right)^{-1}
$$

$$
M = \Lambda + P_{\text{inv}}
$$

$$
Q = M^{-1} \quad \text{（对应基准写法中 } K = M^{-1} H^\top \text{ 里的 } M^{-1} \text{）}
$$

### 4.3 保留 IE 的 $\mathrm{left}+\mathrm{right}$ 形式

**$\mathrm{left}$ 项**（等价于 $-KZ$）：

$$
\text{left} = -Q \begin{bmatrix} \mathrm{HTVr} \\ \mathbf{0} \end{bmatrix}
$$

```cpp
M = (P_in / sigma2).inverse();
M.block<6, 6>(0, 0) += HTVH;
Q = M.inverse();

error_dx.setZero();
error_dx.head<6>() = HTVr;
left = -Q * error_dx;
```

**$\mathrm{right}$ 项**（等价于 $-(I-KH) J_{\text{inv}} \delta$）：

$$
KH = Q \Lambda
$$

$$
\text{right} = -(I - Q\Lambda) \, J_{\text{inv}} \, \delta
$$

```cpp
Lambda.setZero();
Lambda.block<6, 6>(0, 0) = HTVH;

KH = Q * Lambda;
right = -(I - KH) * J_inv * delta;
dx = left + right;
```

**协方差更新**（等价于 $(I-KH)P_{\text{in}}$）：

```cpp
P_out = (I - KH) * P_in;
```

### 4.4 等价性推导

**$\mathrm{left}$：**

$$
-KZ = -M^{-1} H^\top Z = -M^{-1} \begin{bmatrix} \mathrm{HTVr} \\ \mathbf{0} \end{bmatrix} = -Q \, \text{error\_dx}
$$

**$\mathrm{right}$：**

$$
KH = M^{-1} H^\top H = M^{-1} \Lambda = Q \Lambda
$$

故 $\text{right}$ 与 $P$ 的更新与基准写法一致。

**18 维状态的间接修正：** $Q$ 为 $18 \times 18$ 满阵，$\text{left}$ 虽只在前 6 维注入观测残差，速度、零偏、重力等分量仍通过 $P$ 的交叉协方差被间接修正。

### 4.5 仅保留观测项的简化写法

有些实现只用观测项修正当前状态。沿用本文“$Z$ 为预测残差”的符号约定，令

$$
\eta=\begin{bmatrix}\mathrm{HTVr}\\\mathbf 0\end{bmatrix},
\qquad
Q=(\sigma^2P_{\mathrm{in}}^{-1}+\Lambda)^{-1},
\qquad
\Delta x_{\mathrm{simple}}=-Q\eta.
$$

它相当于保留第 4.3 节的 $\mathrm{left}$，省略 $\mathrm{right}=-(I-Q\Lambda)J_{\mathrm{inv}}\delta$。首轮 $\delta=0$ 时两式相同；后续迭代一般不同。若代码把残差定义为“观测减预测”，观测项的符号会反过来，不能仅凭正负号判断是否等价。

| 更新形式 | 与基准 IE 写法的关系 |
|----------|----------------------|
| 路径 B + $\mathrm{left}+\mathrm{right}$（4.3 节） | 保留同一先验项时，代数等价 |
| 仅 $-Q\eta$ | 首轮相同；后续迭代通常不等价 |

### 4.6 路径 B 的特点

| 代数层面 | 计算层面 |
|----------|----------|
| 保留 $\mathrm{left}+\mathrm{right}$ 时与基准等价 | 不显式分配 $H$、$Z$、$K$，矩阵规模最小 |
| 直接揭示 $H^\top H$、$H^\top Z$ 的压缩结构 | 需完整理解 $Q$、$Q\Lambda$ 与 $KH$ 的对应关系 |
| 无合成观测矩阵，推导更抽象 | 无法从汇总量还原逐点残差向量 |

### 4.7 单轮迭代的完整流程

```cpp
// 迭代更新（单轮示意）
const auto x_prop = x_est;  // x_est 此时为传播预测值
const auto P_prop = P;      // P 此时为传播先验
for (int iter = 0; iter < max_iter; ++iter) {
  delta = computeErrorState(x_est, x_prop);
  J_inv = computeJacobianInverse(...);
  P_in = J_inv * P_prop * J_inv.transpose();

  // 观测构建：逐点累加
  HTVH.setZero();
  HTVr.setZero();
  buildObservation(x_est, HTVH, HTVr);

  // 信息滤波更新
  M = (P_in / sigma2).inverse();
  M.block<6, 6>(0, 0) += HTVH;
  Q = M.inverse();

  error_dx.setZero();
  error_dx.head<6>() = HTVr;
  left = -Q * error_dx;

  Lambda.setZero();
  Lambda.block<6, 6>(0, 0) = HTVH;
  KH = Q * Lambda;
  right = -(I - KH) * J_inv * delta;

  dx = left + right;
  x_est = boxplus(x_est, dx);
  // 判断收敛；下一轮使用同一个 x_prop、P_prop ...
}
P = (I - KH) * P_in;  // 最后一轮线性化结果
```

此流程用显式逆表示代数关系；实际求解宜选合适的矩阵分解。若有效点不足，应跳过本次观测更新；流形状态在提交协方差时还应遵循所用误差坐标的重置约定。

## 5. 三种写法对比

设有效观测点数 $V$，状态维度 $n = 18$，迭代轮数 $T$。

| 项目 | 基准：$V$ 行 $H$ | 路径 A：6 行合成 $H_s$ | 路径 B：直接 $\mathrm{HTVH}$ |
|------|------------------|------------------------|-------------------|
| 存储 $H$ | $V \times n$ | $6 \times n$ | **无** |
| 存储 $K$ | $n \times V$ | $n \times 6$ | **无** |
| Cholesky | 无 | **需要** | 不需要 |
| 显式计算 $H^\top H$ | $O(V n^2)$ | $O(6^2)$ | **跳过**（已累加） |
| 计算 $KH$ | $O(n^2 V)$ | $O(n^2 \cdot 6)$ | $Q\Lambda$，$O(n^2 \cdot 6)$ |
| 与基准代数等价 | 基准 | **是** | **是**（保留 $\mathrm{left}+\mathrm{right}$） |

## 6. 数值差异与退化

| 方面 | 路径 A | 路径 B（$\mathrm{left}+\mathrm{right}$） |
|------|--------|----------------------|
| 浮点累加顺序 | 极小差异 | 极小差异 |
| Cholesky 分解的数值误差 | 有 | 无合成观测分解，但仍有信息矩阵求解误差 |
| $\mathrm{HTVH}$ 仅半正定 | 普通 Cholesky 可能失败 | 若 $P_{\mathrm{in}}$ 正定、$\sigma^2>0$，$M=\Lambda+\sigma^2P_{\mathrm{in}}^{-1}$ 仍正定 |
| 逐点残差可还原 | 否 | 否 |

## 7. 优化结果对比

在实际 LIO 系统中按路径 B 改造后，对 IESKF 两阶段耗时进行了分段统计：

| 指标 | 含义 |
| ---- | ---- |
| `calc_ms` | 观测模型阶段：体素查找、点面匹配、雅可比与 $\mathrm{HTVH}$/$\mathrm{HTVr}$ 累加 |
| `mat_ms` | 更新阶段：信息矩阵求解、$\mathrm{left}$/$\mathrm{right}$ 计算、协方差更新 |

下图记录了约 781 帧运行过程中，基准写法与路径 B 的耗时曲线对比：

![IESKF 路径 B 优化前后耗时对比](../ieskf-timing-comparison.png)

从曲线可以观察到：

- **`calc_ms`（观测构建）**：路径 B（红线）整体低于基准（绿线）。基准多在 15–28 ms 波动，路径 B 多在 7–15 ms，约降低一半。
- **`mat_ms`（矩阵更新）**：路径 B（浅蓝线）几乎贴近零轴；基准（黄线）仍在 5–13 ms 波动。更新阶段耗时下降最为显著。

以第 307 帧为例：

| 指标 | 基准 | 路径 B | 变化 |
| ---- | ---- | ------ | ---- |
| `calc_ms` | 23.02 ms | 9.48 ms | 约 **−59%** |
| `mat_ms` | 8.17 ms | 0.09 ms | 约 **−99%** |
| 合计 | 31.19 ms | 9.57 ms | 约 **−69%** |

这与第 5 节的复杂度分析相符：路径 B 不再分配 $V$ 行 $H$ 和 $K$，避免了显式构造 $KH$ 时随 $V$ 增长的矩阵乘法；若利用 $\Lambda$ 的稀疏块，$Q\Lambda$ 也只需使用前 6 列。`calc_ms` 的下降还包含观测构建阶段的改动，不能仅归因于滤波求解式。图中的数字是这次运行的示例，不代表不同数据、硬件或点数下都有相同降幅。

## 8. 总结

| 路径 | 核心思想 |
|------|----------|
| **路径 A** | 用 Cholesky 将 $6 \times 6$ 信息还原为 6 行合成观测，再走标准 $K$、$KH$ 公式 |
| **路径 B** | 将 $H^\top H$、$H^\top Z$ 视为可压缩量，用 $Q$ 与 $Q\Lambda$ 直接完成 IE 更新 |
| **共同认识** | 压缩的是**信息矩阵结构**，而非观测几何本身；点面匹配仍须逐点完成 |

理解这两条路径的关键，在于认清 $H$ 的稀疏列结构与 $H^\top H$ 低秩块之间的对应关系。路径 A 更贴近"先观测、后滤波"的教科书形式；路径 B 则直接暴露了信息滤波中"只需汇总量"的本质。实测中路径 B 使 `mat_ms` 降至亚毫秒级，单帧 IESKF 总耗时可降低约 60% 以上。

## 参考

- **IESKF 标准流程**：[从 IMU 预测到迭代观测更新]({{< ref "ieskf-standard-workflow.md" >}})

- **ieskf_lio 项目**：https://github.com/Zero-Kq/ieskf_lio
- **Super-LIO ESKF 实现**：https://github.com/Liansheng-Wang/Super-LIO/blob/ros2/src/super_lio/src/lio/ESKF.cpp
- **FAST-LIO2**：https://github.com/hku-mars/FAST_LIO
