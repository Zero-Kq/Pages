+++
title = 'IESKF 观测优化：6×6 累加两种方案'
description = "对比 IESKF 点面观测中基于 6×6 HTVH/HTVr 累加的方案 A（Cholesky 压成 6 行 H/Z）与方案 B（update 直接消费），分析等价性与实现代价"
date = '2026-03-11'
draft = false
tags = ["slam", "优化", "学习笔记", "C++"]
categories = ["SLAM"]
toc = true
math = true
+++

## 介绍

本文对比两种基于 $6 \times 6$ 累加的 IESKF 观测优化方案：

| 方案 | 简述 |
|------|------|
| **方案 A** | 累加 HTVH/HTVr → Cholesky 压成 6 行 H/Z → `update()` **不改** |
| **方案 B** | 累加 HTVH/HTVr → `update()` **直接消费**，不建 H/Z/K |

两者观测模型相同，均与现版 V 行 H 在代数上等价（方案 B 若保留 ZLIO 的 IE `left+right` 形式）。

## 1. 背景：18 维状态 vs 6 维观测

ZLIO 状态为 18 维：

| 下标 | 状态 | LiDAR 点面观测 |
|------|------|----------------|
| 0–2 | 旋转 $\delta\theta$ | **有** |
| 3–5 | 位置 $\delta p$ | **有** |
| 6–8 | 速度 $\delta v$ | **无** |
| 9–11 | 陀螺零偏 $b_g$ | **无** |
| 12–14 | 加计零偏 $b_a$ | **无** |
| 15–17 | 重力 $g$ | **无** |

点到平面残差只依赖 $R, p$，每行雅可比 $h_i \in \mathbb{R}^{1 \times 18}$ **只有前 6 列非零**。

记 $J_i \in \mathbb{R}^6$ 为 $h_i$ 前 6 列的转置：

$$
J_i = \begin{bmatrix} p_{\text{imu},i} \times (R^\top n_i) \\ n_i \end{bmatrix}
$$

残差：

$$
r_i = n_i^\top (R \, p_{\text{imu},i} + p - c_i)
$$

## 2. 原版计算（V 行 H，现版基准）

### 2.1 观测矩阵

$$
H \in \mathbb{R}^{V \times 18}, \quad Z \in \mathbb{R}^{V \times 1}
$$

$$
h_{i,\theta} = -n_i^\top R \, [p_{\text{imu},i}]_\times, \quad h_{i,p} = n_i^\top
$$

### 2.2 IESKF 更新（现版 `ieskf.cpp`）

噪声 $R = 0.001$（写死在代码中），信息形式：

$$
P_{\text{inv}} = \left(\frac{P_{\text{in}}}{0.001}\right)^{-1}
$$

$$
M = H^\top H + P_{\text{inv}} \quad (18 \times 18)
$$

$$
K = M^{-1} H^\top \quad (18 \times V)
$$

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

$$
\text{HTVH} = \sum_i J_i J_i^\top = \text{（} H^\top H \text{ 的左上角 } 6 \times 6 \text{ 块）} \quad (6 \times 6)
$$

$$
\text{HTVr} = \sum_i J_i r_i = \text{（} H^\top Z \text{ 的前 6 维）} \quad (6 \times 1)
$$

**公共累加代码**（两种方案在 `calculate` / `accumulate` 中相同）：

```cpp
Eigen::Vector6d J;
J.head<3>() = p_imu.cross(R.transpose() * normal);
J.tail<3>() = normal;

HTVH += J * J.transpose();   // 6×6
HTVr += J * residual;        // 6×1
```

**注意**：累加时 **不要** 额外乘 $w=1000$；权重由 `update()` 里的 `0.001` 体现。

## 3. 方案 A：累加 + Cholesky 压成 6 行 H/Z

### 3.1 思路

接口 `calculate(State, Z, H)` **不变**。循环内只累加 HTVH/HTVr，循环结束后用 Cholesky 构造 6 行合成观测，再交给现版 `update()`。

### 3.2 Cholesky 构造

要求：

$$
H_{\text{synth}}^\top H_{\text{synth}} \text{ 的 } 6 \times 6 \text{ 块} = \text{HTVH}, \quad H_{\text{synth}}^\top Z_{\text{synth}} \text{ 的前 6 维} = \text{HTVr}
$$

步骤：

$$
\text{HTVH} = L L^\top
$$

$$
H_{\text{synth}} = \begin{bmatrix} L & 0_{6 \times 12} \end{bmatrix} \in \mathbb{R}^{6 \times 18}
$$

$$
L^\top Z_{\text{synth}} = \text{HTVr}
$$

```cpp
Eigen::LLT<Eigen::Matrix6d> llt(HTVH);
if (llt.info() != Eigen::Success) return false;

Eigen::Matrix6d L = llt.matrixL();
_H = Eigen::MatrixXd::Zero(6, 18);
_H.block<6, 6>(0, 0) = L;
_Z = L.transpose().triangularView<Eigen::Upper>().solve(HTVr);
```

### 3.3 update() 不变

```cpp
// H: 6×18, Z: 6×1 → K: 18×6
K = (H.transpose() * H + (P_in_update / 0.001).inverse()).inverse() * H.transpose();
left  = -K * Z;
right = -(I - K * H) * J_inv * error_state;
P_ = (I - K * H) * P_in_update;
```

### 3.4 等价性

因 $H_{\text{synth}}^\top H_{\text{synth}} = H_{\text{full}}^\top H_{\text{full}}$ 且 $H_{\text{synth}}^\top Z_{\text{synth}} = H_{\text{full}}^\top Z_{\text{full}}$，故 $M$、$KZ$、$KH$ 与 V 行版本相同。

### 3.5 方案 A 特点

| 优点 | 缺点 |
|------|------|
| `update()` 零改动，风险低 | 仍要建 6×18 的 H、算 $H^\top H$、$KH$ |
| 接口兼容 | Cholesky 要求 HTVH 正定 |
| 与现版严格等价 | 比方案 B 多一步分解 |

## 4. 方案 B：累加 + update 直接吃 HTVH/HTVr

### 4.1 思路

观测接口改为输出汇总量，**不再构造 H、Z、K**：

```cpp
// 新接口（替代 calculate）
virtual bool accumulate(const State18& state,
                        Eigen::Matrix6d& HTVH,
                        Eigen::Vector6d& HTVr) = 0;
```

`update()` 用信息滤波形式，直接从 HTVH/HTVr 算 $\Delta x$ 和 $P$。

### 4.2 核心等价关系

定义 18 维嵌入矩阵（观测信息只在左上角）：

$$
\Lambda = \begin{bmatrix} \text{HTVH} & 0 \\ 0 & 0 \end{bmatrix} \quad (18 \times 18)
$$

则：

$$
H^\top H = \Lambda, \quad H^\top Z = \begin{bmatrix} \text{HTVr} \\ \mathbf{0} \end{bmatrix}
$$

信息矩阵与增益（**不显式求 K**）：

$$
P_{\text{inv}} = \left(\frac{P_{\text{in}}}{0.001}\right)^{-1}
$$

$$
M = \Lambda + P_{\text{inv}}
$$

$$
Q = M^{-1} \quad \text{（与现版 } K = M^{-1} H^\top \text{ 中的 } M^{-1} \text{ 相同）}
$$

### 4.3 保留 ZLIO IE 形式（与现版等价）

**left 项**（等价于 $-KZ$）：

$$
\text{left} = -Q \begin{bmatrix} \text{HTVr} \\ \mathbf{0} \end{bmatrix}
$$

```cpp
Eigen::Matrix<double, 18, 18> P_inv =
    (P_in_update / 0.001).inverse();
P_inv.block<6, 6>(0, 0) += HTVH;

Eigen::Matrix<double, 18, 18> Q = P_inv.inverse();

Eigen::Matrix<double, 18, 1> error_dx =
    Eigen::Matrix<double, 18, 1>::Zero();
error_dx.head<6>() = HTVr;

Eigen::Matrix<double, 18, 1> left = -Q * error_dx;
```

**right 项**（等价于 $-(I-KH) J_{\text{inv}} \delta$）：

$$
KH = Q \Lambda
$$

$$
\text{right} = -(I - Q\Lambda) \, J_{\text{inv}} \, \delta
$$

```cpp
Eigen::Matrix<double, 18, 18> Lambda =
    Eigen::Matrix<double, 18, 18>::Zero();
Lambda.block<6, 6>(0, 0) = HTVH;

Eigen::Matrix<double, 18, 18> KH = Q * Lambda;
Eigen::Matrix<double, 18, 1> right =
    -(Eigen::Matrix<double, 18, 18>::Identity() - KH)
    * J_inv * error_state;

Eigen::Matrix<double, 18, 1> update_x = left + right;
```

**协方差更新**（等价于 $(I-KH)P_{\text{in}}$）：

```cpp
P_ = (Eigen::Matrix<double, 18, 18>::Identity() - KH) * P_in_update;
```

### 4.4 等价性推导

**left：**

$$
-KZ = -M^{-1} H^\top Z = -M^{-1} \begin{bmatrix} \text{HTVr} \\ \mathbf{0} \end{bmatrix} = -Q \, \text{error\_dx}
$$

**right：**

$$
KH = M^{-1} H^\top H = M^{-1} \Lambda = Q \Lambda
$$

故 $\text{right}$、$P$ 更新与 V 行版本一致。

**状态仍是 18 维更新：** $Q$ 为 $18 \times 18$ 满阵，$\text{left}$ 虽只在前 6 维注入观测残差，**v、$b_g$、$b_a$、$g$ 仍通过 $P$ 交叉项被间接修正**。

### 4.5 Super-LIO 风格（不等价于现版 ZLIO）

Super-LIO 的 `ESKF::UpdateObserve` 使用更简形式：

$$
P_k^{-1} \mathrel{+}= \text{HTVH}, \quad Q_k = (P_k^{-1})^{-1}
$$

$$
\text{error\_dx.head}(6) = \text{HTVr}, \quad \Delta x = Q_k \cdot \text{error\_dx}
$$

$$
P \leftarrow (I - Q_k \cdot \text{temp\_cov}) P_k, \quad \text{temp\_cov 的 } 6 \times 6 \text{ 块} = \text{HTVH}
$$

**没有** ZLIO 的 `right = -(I-KH) J_inv δ` 项。若直接照搬 Super-LIO 更新而去掉 `right`，**迭代收敛行为可能与现版不同**（通常仍可用，但需 bag 回归验证）。

| 更新形式 | 与现版 ZLIO 关系 |
|----------|------------------|
| 方案 B + `left + right`（4.3 节） | **代数等价** |
| Super-LIO 式 `dx = Q * error_dx` | **不等价**，需单独验证 |

### 4.6 方案 B 特点

| 优点 | 缺点 |
|------|------|
| 不分配 H/Z/K，矩阵运算最少 | 需改 `CalcZHInterface` 和 `update()` |
| 无 Cholesky，无正定失败风险 | 实现时符号/`right` 项易抄错 |
| `ieskf_mat_ms` 预期下降最明显 | 无逐点残差向量（调试需另存） |

### 4.7 方案 B 完整伪代码（单轮迭代）

```cpp
bool IESKF::update(...) {
  for (int i = 0; i < iter_times_; ++i) {
    error_state = getErrorState18(x_k_k, x_);
    J_inv = ...;
    P_in_update = J_inv * P_ * J_inv.transpose();

    Eigen::Matrix6d HTVH = Eigen::Matrix6d::Zero();
    Eigen::Vector6d HTVr = Eigen::Vector6d::Zero();
    if (!calc_zh_ptr_->accumulate(x_k_k, HTVH, HTVr)) return false;

    Eigen::Matrix<double, 18, 18> P_inv =
        (P_in_update / 0.001).inverse();
    P_inv.block<6, 6>(0, 0) += HTVH;
    Eigen::Matrix<double, 18, 18> Q = P_inv.inverse();

    Eigen::Matrix<double, 18, 1> error_dx =
        Eigen::Matrix<double, 18, 1>::Zero();
    error_dx.head<6>() = HTVr;
    Eigen::Matrix<double, 18, 1> left = -Q * error_dx;

    Eigen::Matrix<double, 18, 18> Lambda =
        Eigen::Matrix<double, 18, 18>::Zero();
    Lambda.block<6, 6>(0, 0) = HTVH;
    Eigen::Matrix<double, 18, 18> KH = Q * Lambda;
    Eigen::Matrix<double, 18, 1> right =
        -(Eigen::Matrix<double, 18, 18>::Identity() - KH)
        * J_inv * error_state;

    update_x = left + right;
    // 更新 x_k_k，收敛判断 ...
  }
  P_ = (I - KH) * P_in_update;  // 用最后一轮 KH
}
```

## 5. 三种方案对比总表

设有效点 $V \approx 5000$，迭代 4 轮，点云 $N \approx 25000$。

| 项目 | 现版 V 行 H | 方案 A：6 行 H | 方案 B：直接 HTVH |
|------|-------------|----------------|-------------------|
| 改 `calculate` 接口 | — | 否 | **是** → `accumulate` |
| 改 `update()` | — | 否 | **是** |
| 存储 H | $V \times 18$ | $6 \times 18$ | **无** |
| 存储 K | $18 \times V$ | $18 \times 6$ | **无** |
| Cholesky | 无 | **需要** | 不需要 |
| $H^\top H$ 显式计算 | $O(V \cdot 18^2)$ | $O(6^2)$ via $H^\top H$ | **跳过**（已累加） |
| $KH$ 显式计算 | $O(18^2 \cdot V)$ | $O(18^2 \cdot 6)$ | $Q \Lambda$，$O(18^2 \cdot 6)$ |
| calculate 全点 find | 有 | 有 | 有 |
| 与现版代数等价 | 基准 | **是** | **是**（保留 left+right） |
| 实现风险 | — | 低 | 中 |

## 6. 数值差异与退化

| 方面 | 方案 A | 方案 B（left+right） |
|------|--------|----------------------|
| 浮点累加顺序 | 极小差异 | 极小差异 |
| Cholesky 误差 | 有 | **无** |
| HTVH 非正定 | Cholesky 失败，需 fallback | 仅影响 $P_{\text{inv}}$ 求逆稳定性，一般仍可算 |
| 无法还原逐点残差 | 是 | 是 |

## 7. 改哪些文件

| 文件 | 方案 A | 方案 B |
|------|--------|--------|
| `lio_zh_voxel_model.h` | 累加 + Cholesky 填 6 行 H/Z | 改为 `accumulate()` |
| `ieskf.h` | 不改 | 新接口 + `update()` 重写 |
| `ieskf.cpp` | 不改 | 按 4.3 节实现 |
| `frontend.cpp` | 不改 | 不改（仍调 `update()`） |

## 8. 实现检查清单

### 公共

- [ ] `J` 定义与现版 H 前 6 列一致
- [ ] 累加时不重复乘 $w=1000$
- [ ] 用同一 bag 对比轨迹 APE/RPE

### 方案 A

- [ ] Cholesky 失败时有 fallback
- [ ] 输出 H 为 6×18 后 `update()` 行为不变

### 方案 B

- [ ] `left = -Q * error_dx`，`error_dx.head(6) = HTVr`
- [ ] `KH = Q * Lambda`，`Lambda` 仅 6×6 块非零
- [ ] `P = (I - KH) * P_in`，与现版一致
- [ ] 若改用 Super-LIO 式更新，单独标注并做回归

## 9. 总结

| 方案 | 一句话 |
|------|--------|
| **方案 A** | 累加后 Cholesky 成 6 行 H/Z，接口不变，与 V 行等价，省矩阵但仍有 $KH$ |
| **方案 B** | 累加后 $Q$、$Q\Lambda$ 直接算 left/right，不建 H/K，**省得最多**，需改 `update` |
| **共同局限** | 均不减少 calculate 里每轮全点体素 find（`ieskf_calc_ms` 主因） |

**推荐路径：** 若求稳，先做方案 A；确认收益后再上方案 B 进一步压低 `ieskf_mat_ms`。

## 参考

- **ieskf_lio 项目**：https://github.com/Zero-Kq/ieskf_lio
- **ZLIO 观测模块 lioZH.hpp**：https://github.com/Zero-Kq/ieskf_lio/blob/master/include/ieskf_slam/modules/lioZH.hpp
- **ZLIO IESKF 更新 ieskf.cpp**：https://github.com/Zero-Kq/ieskf_lio/blob/master/src/ieskf_slam/ieskf.cpp
- **Super-LIO ESKF 实现**：https://github.com/Liansheng-Wang/Super-LIO/blob/ros2/src/super_lio/src/lio/ESKF.cpp
