+++
title = "LIO 学习笔记 03"
description = "关于 LIO SLAM 系统的学习"
date = 2026-04-20
draft = true
tags = ["slam", "学习笔记", "C++"]
categories = ["SLAM"]
math = true
+++

## 介绍

这一节主要是关于IMU数据处理，滤波的方法速度更快和据说无人机上对IMU的鲁棒性比优化方法好，而且能实现高频输出，所以先实现滤波方法；
优化方法的精度更高，对随机白噪声更鲁棒
这一章主要包含初始化、ESKF预测和点云去畸变处理。

## 主要依赖

- Pangolin：3D 可视化库
- glog：Google 日志库

## 初始化

定义重力常量 G：

```cpp
static constexpr float G_ = 9.81f;
constexpr float IMUProcess::G_;
```

获取静态下 IMU 的加速度和陀螺仪的值，并判断重力方向。

> 类成员 static 不需要对象也能访问，所有对象共享同一份，需要在类外定义，所以 `G_` 在类里面声明了还需在外面定义。

## 前向传播（预测）和后向传播（去畸变）

### 前向传播

将 ESKF 数据转换到 IMU 结构体：

```cpp
Eigen::Matrix3f R0 = eskf.GetRot().cast<float>();
Eigen::Vector3f pos0 = eskf.GetPos().cast<float>();
Eigen::Vector3f vel0 = eskf.GetVel().cast<float>();
Eigen::Vector3f bg0 = eskf.GetBg().cast<float>();
Eigen::Vector3f ba0 = eskf.GetBa().cast<float>();
```

> `cast<float>()` 是 Eigen 库的类型转换方法，将数据转换成 float。

进行均值滤波，取当前帧和下一帧 IMU 的中间值作为这段时间的均值做积分：

```cpp
// 矩形积分（只用端点）
acc_integrated = head.acc * dt;
// 梯形积分（用均值）
acc_integrated = (head.acc + tail.acc) / 2.0 * dt;
// 辛积分（待研究）
acc_integrated = (head.acc + tail.acc + 4 * mid.acc) / 6.0 * dt;
```

> IMU 频率高、梯形积分计算简单开销小。

进行状态预测，调用 ESKF 的预测方程，输入噪声矩阵、IMU 的加速度计和陀螺仪、时间差，状态保存在 ESKF 中。将所有的 IMU 预测后的位姿数据存储起来用于去畸变：

```cpp
// 调用 ESKF 预测 (使用预创建的Q矩阵)
Eigen::Vector3d gyro = imu.gyro.cast<double>();
Eigen::Vector3d acc = imu.acc.cast<double>();
eskf.Predict(dt, Q_, gyro, acc);

// 保存IMU pose (用于去畸变)
Pose6D pose;
pose.offset_time = imu.timestamp;
pose.acc = eskf.GetRot().cast<float>() * (imu.acc - eskf.GetBa().cast<float>()) + gravity_.cast<float>();
pose.gyro = imu.gyro.cast<float>() - eskf.GetBg().cast<float>();
pose.vel = eskf.GetVel().cast<float>();
pose.pos = eskf.GetPos().cast<float>();
pose.rot = eskf.GetRot().cast<float>();
imu_poses_.push_back(pose);
```

噪声矩阵配置：

| 位置 | 噪声 | 配置参数 | 参考值 |
|------|------|----------|--------|
| block(0,0) | gyro_noise | gyro_noise | 0.1 |
| block(3,3) | acc_noise | acc_noise | 0.1 |
| block(6,6) | gyro_bias_noise | gyro_bias_noise | 0.0001 |
| block(9,9) | acc_bias_noise | acc_bias_noise | 0.00001 |

### 反向传播

遍历所有比当前 IMU 时间戳晚的点，将该点通过变换矩阵投影到 LiDAR 的帧末时刻：

```cpp
// 获取结束时刻状态用于去畸变
Eigen::Vector3f final_pos = eskf.GetPos().cast<float>();
Eigen::Matrix3f final_rot = eskf.GetRot().cast<float>();

auto it_pcl = pcl_out.points.end() - 1;
for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); --it_kp) {
    auto head = it_kp - 1;
    auto tail = it_kp;

    // 遍历点云，将时间戳大于head时刻的点投影到tail时刻
    for (; it_pcl->timestamp > head->offset_time; --it_pcl) {
        double dt = it_pcl->timestamp - head->offset_time;

        // 计算时刻t_i的位姿
        Eigen::Matrix3f R_i = head->rot * SO3Utils::ExpF(head->gyro, static_cast<float>(dt));
        Eigen::Vector3f T_ei = head->pos + head->vel * static_cast<float>(dt) +
                                0.5f * head->acc * static_cast<float>(dt * dt) - final_pos;

        // 获取原始点
        Eigen::Vector3f P_i(it_pcl->x, it_pcl->y, it_pcl->z);

        // 投影到帧末时刻
        Eigen::Matrix3f R_lidar_imu = extrinsic_.rotation.matrix();
        Eigen::Vector3f T_lidar_imu = extrinsic_.translation;

        Eigen::Vector3f P_compensate =
            R_lidar_imu.transpose() *
            (final_rot.transpose() *
                    (R_i * (R_lidar_imu * P_i + T_lidar_imu) + T_ei) -
                T_lidar_imu);

        // 更新点坐标
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin()) {
            break;
        }
    }

    if (it_pcl == pcl_out.points.begin()) {
        break;
    }
}
```

## 参考

- 无
