+++
title = 'LIO学习笔记03'
description = "ZLIO 中 IMU 处理的核心流程：初始化、前向传播和后向传播去畸变"
date = '2026-06-14'
draft = false
tags = ["slam", "学习笔记"]
categories = ["SLAM"]
weight = 1
slug = ""
aliases = []
series = []
externalLink = ""
toc = true
math = true
+++

## 介绍

在 LIO-SLAM 系统中，IMU（惯性测量单元）扮演着至关重要的角色。IMU 提供高频（通常 200-400Hz）的角速度和线加速度测量，用于：

1. **状态初始化**：确定系统的初始重力方向、零偏和缩放系数
2. **状态预测（前向传播）**：在两帧 LiDAR 之间递推位姿，为滤波器提供预测值
3. **点云去畸变（后向传播）**：将一帧扫描周期内的点云补偿到同一时刻，消除运动畸变

ZLIO 中 IMU 处理的核心流程如下：

```
IMU 初始化 → 前向传播 (递推状态 + 记录位姿序列) → 后向传播 (点云去畸变)
```

涉及的核心文件：

| 文件 | 作用 |
|------|------|
| `imu_type.h` | IMU 数据结构定义 |
| `imu_process.h/cpp` | IMU 前向/后向传播与去畸变 |
| `ieskf.h/cpp` | IESKF 滤波器，包含 predict 步 |
| `frontend.h/cpp` | 前端模块，协调 IMU 初始化与数据同步 |
| `math_utils.h` | 李群李代数数学工具 |

---

## IMU 数据结构

IMU 数据结构定义在 [imu_type.h](src/zlio/include/core/type/imu_type.h) 中：

```cpp
class Imu {
 public:
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(); // 线加速度 (m/s^2)
  Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();    // 角速度 (rad/s)
  double timestamp = 0.0;                                 // 时间戳 (s)
};
```

IMU 数据以**测量组 (MeasureGroup)** 的形式与点云同步后一起处理，定义在 [measure_group_type.h](src/zlio/include/core/type/measure_group_type.h)：

```cpp
struct MeasureGroup {
  double lidar_begin_time = 0.0; // 点云开始时间 (s)
  double lidar_end_time = 0.0;   // 点云结束时间 (s)
  std::deque<Imu> imus;          // 该帧点云时间段内的 IMU 数据
  PointCloud cloud;              // 点云数据
};
```

**设计要点**：

- 一帧 LiDAR 点云的时间跨度内会有多条 IMU 数据，它们被同步打包到 `MeasureGroup` 中
- 点云中每个点的时间信息存储在 `curvature` 字段中，表示相对于点云起始时刻的偏移（秒）

---

## IMU 初始化

### 原理

IMU 初始化的目的是确定系统的初始状态，包括：

1. **重力方向**：IMU 测量的加速度包含重力分量，通过平均多帧加速度数据可以确定重力方向
2. **加速度计缩放系数**：实际 IMU 可能存在缩放误差，通过比较测量值模长与理论重力值 `g=9.81` 可以计算缩放系数
3. **陀螺仪零偏**：静止状态下角速度的平均值即为零偏的估计

**数学原理**：

在静止或匀速运动假设下，加速度计测量值为：

$$a_{measured} = R^T \cdot (-g) + b_a + n_a$$

其中 $R$ 是 IMU 到世界系的旋转，$g$ 是重力加速度向量。当载体静止时：

$$\|a_{measured}\| \approx g = 9.81 \text{ m/s}^2$$

因此缩放系数为：

$$s = \frac{g}{\|a_{mean}\|}$$

重力方向可由平均加速度方向确定：

$$g_{vec} = -\frac{a_{mean}}{\|a_{mean}\|} \cdot g$$

### 代码实现

初始化逻辑在 [frontend.cpp:initState()](src/zlio/src/core/modules/frontend.cpp#L49-L80) 中实现：

```cpp
void FrontEnd::initState(MeasureGroup &_mg) {
  static int imu_count = 0;
  static Eigen::Vector3d mean_acc{0, 0, 0};

  // 累加 IMU 数据
  for (size_t i = 0; i < _mg.imus.size(); i++) {
    imu_count++;
    auto x = ieskf.getX();
    mean_acc += _mg.imus[i].acceleration;  // 累加加速度
    x.bg += _mg.imus[i].gyroscope;         // 累加角速度（用于零偏估计）
    ieskf.setX(x);
  }

  // 累计 5 个 IMU 数据后完成初始化
  if (imu_count >= 5) {
    auto x = ieskf.getX();
    mean_acc /= double(imu_count);   // 计算平均加速度
    x.bg /= double(imu_count);       // 计算平均角速度（零偏）

    // 计算缩放系数: 实际重力值 / 测量值模长
    imu_scale_ = GRAVITY / mean_acc.norm();
    imu_process_ptr_->imu_scale = imu_scale_;

    // 初始化 ImuProcess 的历史状态
    imu_process_ptr_->last_imu = _mg.imus.back();
    imu_process_ptr_->last_lidar_end_time = _mg.lidar_end_time;

    // 确定重力向量（注意符号为负，与 FAST-LIO 公式一致）
    x.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
    ieskf.setX(x);
    imu_inited_ = true;
  }
}
```

**关键步骤解读**：

| 步骤 | 操作 | 作用 |
|------|------|------|
| 1 | 累加 `acceleration` 和 `gyroscope` | 收集初始化所需的统计数据 |
| 2 | `imu_count >= 5` 判断 | 确保有足够数据进行平均，避免噪声影响 |
| 3 | `imu_scale_ = GRAVITY / mean_acc.norm()` | 计算加速度计缩放系数 |
| 4 | `x.gravity = -mean_acc / mean_acc.norm() * GRAVITY` | 确定重力方向（世界系下） |
| 5 | `last_imu` / `last_lidar_end_time` | 初始化 ImuProcess 的递推起点 |

**重力符号说明**：`x.gravity` 存储的是**世界系下的重力加速度向量**，取负号是因为在 IESKF 的状态递推公式中：

$$\dot{v} = R \cdot (a_m - b_a) + g$$

其中 $g$ 是重力加速度（方向向下），与 FAST-LIO 的公式约定一致。

---

## 前向传播 (Forward Propagation)

### 原理

前向传播的核心任务是：利用 IMU 数据在两帧 LiDAR 之间递推系统状态，并记录递推过程中的中间位姿序列，为后续的点云去畸变提供位姿参考。

**运动模型**（连续时间）：

$$\dot{R} = R \cdot [\omega_m - b_g]_\times$$

$$\dot{v} = R \cdot (a_m - b_a) + g$$

$$\dot{p} = v$$

其中 $[\cdot]_\times$ 表示反对称矩阵（叉积的矩阵形式）。

**离散化**（中值积分）：

对于相邻两个 IMU 时刻 $k$ 和 $k+1$，取两端测量值的均值：

$$\bar{\omega} = \frac{1}{2}(\omega_k + \omega_{k+1})$$

$$\bar{a} = \frac{1}{2}(a_k + a_{k+1})$$

状态更新：

$$R_{k+1} = R_k \cdot \exp([\bar{\omega} - b_g]_\times \cdot \Delta t)$$

$$v_{k+1} = v_k + (R_k \cdot (\bar{a} - b_a) + g) \cdot \Delta t$$

$$p_{k+1} = p_k + v_k \cdot \Delta t$$

> **注意**：代码中的位置更新使用了简化的欧拉积分 `p += v * dt`，而非二阶积分 `p += v*dt + 0.5*a*dt^2`。这是因为前向传播的主要目的是记录位姿序列用于去畸变，去畸变阶段会使用更精确的二次积分。

### 代码实现

前向传播实现在 [imu_process.cpp:forwardPropagate()](src/zlio/src/core/modules/imu_process.cpp#L37-L103) 中：

```cpp
void ImuProcess::forwardPropagate(MeasureGroup &_mg, IESKF::Ptr _ieskf_ptr,
                                    std::vector<IMUPose6d> &_poses_out) {
  // ===== 步骤 1: 点云按时间排序 =====
  // curvature 字段存储的是相对点云起始时刻的时间偏移 (秒)
  std::sort(_mg.cloud.cloud_ptr->points.begin(), _mg.cloud.cloud_ptr->points.end(),
            [](const PointType &_x, const PointType &_y) {
              return _x.curvature < _y.curvature;
            });

  // ===== 步骤 2: 准备 IMU 数据 =====
  // 将上一帧最后一条 IMU 数据插入队列头部，保证时间连续性
  auto v_imu = _mg.imus;
  v_imu.push_front(last_imu);

  const double &pcl_beg_time = _mg.lidar_begin_time;
  const double &pcl_end_time = _mg.lidar_end_time;
  auto imu_state = _ieskf_ptr->getX();

  // ===== 步骤 3: 记录递推起始状态 =====
  _poses_out.clear();
  _poses_out.emplace_back(0.0, acc_s_last, angvel_last,
                          imu_state.velocity, imu_state.position, imu_state.rotation);

  // ===== 步骤 4: 逐段递推 =====
  Eigen::Vector3d angvel_avr, acc_avr;
  double dt = 0.0;
  Imu in;

  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    // 中值积分：取两端均值
    angvel_avr = 0.5 * (head.gyroscope + tail.gyroscope);
    acc_avr = 0.5 * (head.acceleration + tail.acceleration) * imu_scale;

    // 计算时间步长，处理与上一帧的衔接
    if (head.timestamp < last_lidar_end_time) {
      dt = tail.timestamp - last_lidar_end_time;  // 跨帧衔接
    } else {
      dt = tail.timestamp - head.timestamp;        // 正常步长
    }
    if (dt <= 0.0) continue;

    // 执行 IESKF 预测步
    in.acceleration = acc_avr;
    in.gyroscope = angvel_avr;
    _ieskf_ptr->predict(in, dt);

    // 获取更新后的状态
    imu_state = _ieskf_ptr->getX();

    // 计算世界系下的加速度（用于去畸变）
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rotation * (acc_avr - imu_state.ba);
    acc_s_last += imu_state.gravity;  // 补偿重力

    // 记录该时刻位姿，时间戳为相对于点云起始时刻的偏移
    double offs_t = tail.timestamp - pcl_beg_time;
    _poses_out.emplace_back(offs_t, acc_s_last, angvel_last,
                            imu_state.velocity, imu_state.position, imu_state.rotation);
  }

  // ===== 步骤 5: 补齐到点云结束时刻 =====
  double dt_final = pcl_end_time - v_imu.back().timestamp;
  if (dt_final > 0.0) {
    _ieskf_ptr->predict(in, dt_final);
  }

  // 更新历史状态
  last_imu = _mg.imus.back();
  last_lidar_end_time = pcl_end_time;
}
```

**IMUPose6d 结构体**记录了每个递推时刻的关键信息：

```cpp
struct IMUPose6d {
  double time;            // 相对点云起始时刻的时间偏移 (s)
  Eigen::Vector3d acc;    // 世界系下的加速度（已补偿重力和零偏）
  Eigen::Vector3d angvel; // 角速度（已减去零偏）
  Eigen::Vector3d vel;    // 世界系速度
  Eigen::Vector3d pos;    // 世界系位置
  Eigen::Quaterniond rot; // 世界系旋转
};
```

**IESKF 预测步**实现 [ieskf.cpp:predict()](src/zlio/src/core/modules/ieskf.cpp#L48-L80)：

```cpp
void IESKF::predict(const Imu &_imu, double _dt) {
  // 1. 状态递推 (Nominal State Prediction)
  Eigen::Vector3d angvel = _imu.gyroscope - x_.bg;   // 去零偏
  Eigen::Vector3d acc = _imu.acceleration - x_.ba;    // 去零偏

  auto rotation_mat = x_.rotation.toRotationMatrix();

  // 旋转更新: R_{k+1} = R_k * exp([ω]× * dt)
  x_.rotation = Eigen::Quaterniond(rotation_mat * so3Exp(angvel * _dt));
  x_.rotation.normalize();

  // 位置和速度更新
  x_.position += x_.velocity * _dt;
  x_.velocity += (rotation_mat * acc + x_.gravity) * _dt;

  // 2. 协方差传播 (Covariance Propagation)
  // P_{k+1} = Fx * P_k * Fx^T + Fw * Q * Fw^T
  Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
  Eigen::Matrix<double, 18, 12> Fw = Eigen::Matrix<double, 18, 12>::Zero();

  // Fx: 状态转移矩阵（误差状态对误差状态的雅可比）
  Fx.block<3, 3>(0, 0) = so3Exp(-angvel * _dt);           // δR 对 δR
  Fx.block<3, 3>(0, 9) = -A_T(-angvel * _dt) * _dt;      // δbg 对 δR
  Fx.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * _dt; // δv 对 δp
  Fx.block<3, 3>(6, 0) = -rotation_mat * skewSymmetric(acc) * _dt; // δR 对 δv
  Fx.block<3, 3>(6, 12) = -rotation_mat * _dt;            // δba 对 δv
  Fx.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * _dt; // δg 对 δv

  // Fw: 噪声转移矩阵（误差状态对噪声的雅可比）
  Fw.block<3, 3>(0, 0) = -A_T(-angvel * _dt) * _dt;      // 陀螺仪噪声
  Fw.block<3, 3>(6, 3) = -rotation_mat * _dt;              // 加速度计噪声
  Fw.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * _dt; // 零偏随机游走
  Fw.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * _dt;

  P_ = Fx * P_ * Fx.transpose() + Fw * Q_ * Fw.transpose();
}
```

**18 维状态向量布局**：

| 索引 | 维度 | 状态 | 说明 |
|------|------|------|------|
| 0-2 | 3 | δθ (旋转误差) | SO3 李代数表示 |
| 3-5 | 3 | δp (位置误差) | |
| 6-8 | 3 | δv (速度误差) | |
| 9-11 | 3 | δb_g (陀螺仪零偏) | |
| 12-14 | 3 | δb_a (加速度计零偏) | |
| 15-17 | 3 | δg (重力误差) | |

**12 维噪声向量布局**：

| 索引 | 维度 | 噪声源 |
|------|------|--------|
| 0-2 | 3 | 陀螺仪测量噪声 |
| 3-5 | 3 | 加速度计测量噪声 |
| 6-8 | 3 | 陀螺仪零偏随机游走 |
| 9-11 | 3 | 加速度计零偏随机游走 |

---

## 后向传播与去畸变 (Undistortion)

### 原理

由于 LiDAR 扫描一帧点云需要一定时间（例如 100ms），在这段时间内载体是运动的，因此同一帧点云中的不同点实际上是在不同位姿下采集的。这会导致点云出现**运动畸变**。

去畸变的目标是：将一帧点云中的所有点变换到**同一时刻**（通常是扫描结束时刻），消除运动带来的畸变。

**数学推导**：

设扫描结束时刻为 $C$，点 $P_i$ 的采集时刻为 $t_i$，则：

- $R_i, p_i$：时刻 $t_i$ 的位姿（由前向传播插值得到）
- $R_C, p_C$：扫描结束时刻 $C$ 的位姿
- $P_i^{L}$：点在采集时刻雷达坐标系下的坐标

点在世界系下的坐标：

$$P_i^{G} = R_i \cdot P_i^{L} + p_i$$

变换到结束时刻雷达系：

$$P_i^{C} = R_C^T \cdot (P_i^{G} - p_C) = R_C^T \cdot (R_i \cdot P_i^{L} + p_i - p_C)$$

**旋转插值**：在 $t_i$ 时刻的旋转通过角速度插值得到：

$$R_i = R_{head} \cdot \exp([\omega \cdot (t_i - t_{head})]_\times)$$

其中 $R_{head}$ 是该 IMU 段起始时刻的旋转，$\omega$ 是该段的平均角速度。

**位置插值**：使用二阶积分：

$$p_i = p_{head} + v_{head} \cdot \Delta t + \frac{1}{2} a \cdot \Delta t^2$$

其中 $a$ 是世界系下的加速度（已补偿重力和零偏）。

### 代码实现

去畸变实现在 [imu_process.cpp:undistortPcl()](src/zlio/src/core/modules/imu_process.cpp#L111-L151) 中：

```cpp
void ImuProcess::undistortPcl(MeasureGroup &_mg,
                               const std::vector<IMUPose6d> &_poses_in,
                               IESKF::Ptr _ieskf_ptr) {
  auto &pcl_out = *(_mg.cloud.cloud_ptr);
  auto it_pcl = pcl_out.points.end() - 1;  // 从最后一个点开始（时间最晚）

  // 获取扫描终点 C 的位姿
  auto imu_state_final = _ieskf_ptr->getX();

  // 从后往前遍历 IMU 位姿段
  for (auto it_kp = _poses_in.end() - 1; it_kp != _poses_in.begin(); it_kp--) {
    auto head = it_kp - 1;  // 当前段起始
    auto tail = it_kp;       // 当前段结束

    Eigen::Matrix3d R_imu = head->rot.toRotationMatrix();
    Eigen::Vector3d vel_imu = head->vel;
    Eigen::Vector3d pos_imu = head->pos;
    Eigen::Vector3d acc_imu = tail->acc;        // 使用尾部的加速度
    Eigen::Vector3d angvel_avr = tail->angvel;  // 使用尾部的角速度

    // 遍历落在该时间段内的点
    for (; it_pcl->curvature > head->time; it_pcl--) {
      double dt = it_pcl->curvature - head->time;

      // 插值计算点时刻的旋转: R_i = R_head * exp([ω * dt]×)
      Eigen::Matrix3d R_i(R_imu * so3Exp(angvel_avr * dt));

      // 点在采集时刻雷达系下的坐标
      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);

      // 计算点时刻的位置偏移（相对于扫描终点）
      // T_ei = p_i^G - p_C^G
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt
                           - imu_state_final.position);

      // 变换到扫描终点时刻的雷达系
      // P_C = R_G^C * (R_i * P_i + T_ei)
      Eigen::Vector3d P_compensate =
          imu_state_final.rotation.conjugate() * (R_i * P_i + T_ei);

      // 写回点云
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}
```

**去畸变流程图**：

```
时间轴: ─────────────────────────────────────────────►
        |  IMU段0  |  IMU段1  |  IMU段2  |  IMU段3  |
        ↑                                               ↑
     扫描开始                                         扫描结束 (C)

点云点:   •  • •  • ••  •  •• •  •  •• •  • •  •
          ──────────────────────────────────────────→
          从后往前遍历点，按时间分配到对应 IMU 段

对每个点 P_i:
  1. 确定所在 IMU 段 [head, tail]
  2. dt = P_i.curvature - head.time
  3. R_i = R_head * exp(ω * dt)          // 旋转插值
  4. p_i = p_head + v*dt + 0.5*a*dt²     // 位置插值
  5. P_out = R_C^T * (R_i * P_i + p_i - p_C)  // 变换到终点系
```

**为什么从后往前遍历？**

- 点云已按时间排序（前向传播的第一步）
- 从后往前遍历可以保证每个点只需检查当前 IMU 段，时间复杂度为 O(N)
- 如果从前往后遍历，每个点都需要重新查找所属的 IMU 段

---

## 数学工具

IMU 处理依赖的核心数学工具定义在 [math_utils.h](src/zlio/include/core/math/math_utils.h) 中：

### 反对称矩阵 (Skew Symmetric Matrix)

向量 $v = [v_1, v_2, v_3]^T$ 的反对称矩阵：

$$[v]_\times = \begin{bmatrix} 0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0 \end{bmatrix}$$

作用：将叉积转化为矩阵乘法 $a \times b = [a]_\times \cdot b$

### SO3 指数映射 (Rodrigues 公式)

将旋转向量 $\theta \cdot n$ 转换为旋转矩阵：

$$\exp([\theta n]_\times) = I + \sin\theta \cdot [n]_\times + (1 - \cos\theta) \cdot [n]_\times^2$$

```cpp
static inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d &_v) {
  double theta = _v.norm();
  if (theta < 1e-7) return Eigen::Matrix3d::Identity();  // 小角度近似
  Eigen::Vector3d n = _v / theta;
  Eigen::Matrix3d n_skew = skewSymmetric(n);
  return Eigen::Matrix3d::Identity() + std::sin(theta) * n_skew
       + (1.0 - std::cos(theta)) * n_skew * n_skew;
}
```

### SO3 对数映射

将旋转矩阵转换回旋转向量：

$$\theta = \arccos\left(\frac{tr(R) - 1}{2}\right)$$

$$[\theta n]_\times = \frac{\theta}{2\sin\theta}(R - R^T)$$

### 右雅可比 A_T

用于 IESKF 协方差传播中的误差状态转移：

$$J_r(\theta n) = I + \frac{1 - \cos\theta}{\theta^2} [n]_\times + \frac{\theta - \sin\theta}{\theta^3} [n]_\times^2$$

---

## 完整数据流

将所有部分串联起来，IMU 处理在 ZLIO 系统中的完整数据流如下：

```
┌─────────────────────────────────────────────────────────────────┐
│                        FrontEnd::track()                         │
│                                                                  │
│  ┌──────────────────┐                                            │
│  │ syncMeasureGroup │ ← 同步 IMU 和点云数据                      │
│  └────────┬─────────┘                                            │
│           │                                                      │
│           ▼                                                      │
│  ┌──────────────────┐     No     ┌───────────────┐              │
│  │ imu_inited_?     │ ─────────→ │  initState()  │              │
│  └────────┬─────────┘            │  · 累加 acc    │              │
│           │ Yes                  │  · 累加 gyro   │              │
│           │                      │  · 计算 gravity │              │
│           │                      │  · 计算 scale   │              │
│           │                      └───────┬───────┘              │
│           ▼                              │                      │
│  ┌──────────────────────────────────────────┐                   │
│  │ imu_process_ptr_->propagate(mg, ieskf)   │                   │
│  │                                           │                   │
│  │  ┌─────────────────────────────────────┐ │                   │
│  │  │ 1. forwardPropagate()               │ │                   │
│  │  │    · 点云按时间排序                   │ │                   │
│  │  │    · 逐段中值积分递推                 │ │                   │
│  │  │    · 调用 ieskf->predict() 更新状态   │ │                   │
│  │  │    · 记录 IMUPose6d 位姿序列          │ │                   │
│  │  └──────────────┬──────────────────────┘ │                   │
│  │                 │                         │                   │
│  │                 ▼                         │                   │
│  │  ┌─────────────────────────────────────┐ │                   │
│  │  │ 2. undistortPcl()                   │ │                   │
│  │  │    · 从后往前遍历点云                 │ │                   │
│  │  │    · 旋转插值 + 位置二阶积分          │ │                   │
│  │  │    · 变换到扫描终点坐标系             │ │                   │
│  │  └─────────────────────────────────────┘ │                   │
│  └───────────────────────────────────────────┘                   │
│           │                                                      │
│           ▼                                                      │
│  ┌──────────────────┐                                            │
│  │ IESKF::update()  │ ← 观测更新（点到面残差）                    │
│  └──────────────────┘                                            │
└─────────────────────────────────────────────────────────────────┘
```

### 关键设计总结

| 设计点 | 选择 | 原因 |
|--------|------|------|
| 积分方式 | 中值积分 | 比欧拉积分精度更高，比 RK4 计算量小 |
| 时间衔接 | `last_lidar_end_time` | 保证帧间 IMU 数据不丢失不重复 |
| 去畸变基准 | 扫描结束时刻 | 与 IESKF 更新步的状态时间对齐 |
| 遍历方向 | 从后往前 | 点云已排序，单次遍历即可完成去畸变 |
| 旋转表示 | 四元数 + SO3 李代数 | 避免万向锁，指数映射计算高效 |
| 状态维度 | 18 维 | 包含重力估计，提高鲁棒性 |

---

## 参考

- **FAST-LIO 源码**：https://github.com/hku-mars/FAST_LIO
- **IESKF 理论**：Iterated Extended Kalman Filter for SLAM
- **SO3 李代数**：State Estimation for Robotics (Timothy D. Barfoot)
