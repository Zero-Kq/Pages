+++
title = "Gtsam 学习笔记"
description = "关于 Gtsam 优化库的学习"
date = 2026-03-19
draft = false
tags = ["优化", "学习笔记", "C++"]
categories = ["SLAM"]
math = true
+++

## 介绍Gtsam

Gtsam（General and Tunable Structure-from-Motion）是一个用于结构化从运动学（SFM）的优化库。Gtsam提供了一种基于图优化的算法，用于解决各种类型的结构化从运动学问题。Gtsam支持多种优化算法，如Levenberg-Marquardt、Gauss-Newton、Dogleg、Gauss-Newton-Dogleg等，以及多种优化目标，如最小二乘、最小 Trust Region等。Gtsam提供了多种数据结构，如变量、因子、 优化问题等，以及多种工具函数，如线性方程求解、矩阵运算、向量运算等。（AI生成）

## Gtsam库
https://github.com/borglab/gtsam

## Gtsam的使用

### 快速使用

1. 引入头文件
```c++
#include <gtsam/slam/PriorFactor.h>      // 先验因子，为Pose提供绝对约束（如首帧初始位姿）
#include <gtsam/nonlinear/ISAM2.h>       // 增量平滑与建图优化器，支持增量式求解因子图
#include <gtsam/nonlinear/Values.h>      // 优化变量值的容器，存储键值对形式
#include <gtsam/slam/BetweenFactor.h>   // 里程计/回环约束因子，表示两Pose间相对变换
#include <gtsam/inference/Symbol.h>     // GTSAM符号系统，用于生成因子图变量唯一键 ，当直接用帧创建key时无需引入
#include <gtsam/geometry/Pose3.h>        // 3D位姿（Rot3旋转+Point3位置），表示机器人姿态
```

2. 创建变量（Values）
```c++
// 使用 Symbol 创建唯一键：'x' + 0 = 键 "x0"
gtsam::Key key1 = gtsam::symbol('x', 0);
gtsam::Key key2 = gtsam::symbol('x', 1);

//直接用帧数也可以
gtsam::Key key1 = 0;           // 直接用整数

// 创建 Pose3 位姿（旋转 + 平移）
gtsam::Rot3 rotation = gtsam::Rot3::Ypr(0.0, 0.0, 0.0);  // yaw, pitch, roll
gtsam::Point3 translation(0.0, 0.0, 0.0);                // x, y, z
gtsam::Pose3 pose1(rotation, translation);

// 存入 Values 容器
gtsam::Values initial_values;
initial_values.insert(key1, pose1);        // 初始值
initial_values.insert(key2, pose1);         // 第二帧初始值
```

3. 创建噪声模型（NoiseModel）
```c++
// 高斯噪声模型（6维：旋转3维 + 平移3维）
gtsam::Vector6 variance = (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished();
gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Covariance(
    gtsam::Matrix6::Identity() * variance.asDiagonal());

// 或使用对角噪声模型
auto odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
```

4. 创建因子（Factor）
```c++
gtsam::NonlinearFactorGraph graph;

// 先验因子（为第一帧提供绝对约束）
graph.add(gtsam::PriorFactor<gtsam::Pose3>(key1, pose1, priorNoise));

// 里程计因子（两帧之间的相对变换）
gtsam::Pose3 odom_delta = pose1.between(pose2);  // 计算相对变换
graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, odom_delta, odometryNoise));
```

5. 创建 ISAM2 优化器
```c++
gtsam::ISAM2Params params;
params.relinearizeThreshold = 0.01;   // 重新线性化阈值
params.relinearizeSkip = 10;          // 跳帧数
gtsam::ISAM2 isam(params);
isam.update(graph, initial_values);    // 首次更新
```

6. 增量优化
```c++
// 添加新帧时
gtsam::Key key3 = gtsam::symbol('x', 2);
initial_values.clear();
initial_values.insert(key3, pose_guess);  // 新帧初始值

// 更新优化器（增量式，只优化新节点）
isam.update(graph, initial_values);

// 获取优化结果
gtsam::Values result = isam.calculateEstimate();

// 读取特定节点
gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(key3);
```

### 完整示例

```c++
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>

int main() {
    // 1. 创建变量
    gtsam::Key key1 = gtsam::symbol('x', 0);
    gtsam::Key key2 = gtsam::symbol('x', 1);
    gtsam::Pose3 pose1;                        // 单位位姿
    gtsam::Pose3 pose2(gtsam::Rot3::Ypr(0.1, 0, 0), gtsam::Point3(1, 0, 0));

    // 2. 创建噪声模型
    auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
    auto odomNoise = gtsam::noiseModel::Isotropic::Sigma(6, 0.05);

    // 3. 创建因子图
    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(key1, pose1, priorNoise));  // 先验
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, pose1.between(pose2), odomNoise)); // 里程计

    // 4. 创建初始值
    gtsam::Values initial_values;
    initial_values.insert(key1, pose1);
    initial_values.insert(key2, pose2);

    // 5. ISAM2 优化
    gtsam::ISAM2 isam;
    isam.update(graph, initial_values);
    gtsam::Values result = isam.calculateEstimate();

    // 6. 读取结果
    gtsam::Pose3 optimized_pose2 = result.at<gtsam::Pose3>(key2);
    std::cout << "Optimized pose: " << optimized_pose2 << std::endl;

    return 0;
}
```

### 注意事项

1. 每一个Key必须有初始值
2. 一个图必须创建先验约束
3. 不能有孤立节点，必须与其他节点建立约束

### 优化效果

原始x轴位移
```c++
vector<double> cumulative_x = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10
  }
```
优化后

![优化效果对比](/posts/slam/Figure.png)

### 关键函数速查

| 函数 | 说明 |
|------|------|
| `gtsam::symbol( 'x', id)` | 创建变量键，如 'x0', 'x1' |
| `Pose3::Ypr(yaw, pitch, roll)` | 从欧拉角创建旋转 |
| `pose1.between(pose2)` | 计算 pose1 到 pose2 的相对变换 |
| `graph.add(Factor(...))` | 添加因子到因子图 |
| `isam.update(graph, values)` | 增量更新优化器 |
| `isam.calculateEstimate()` | 获取所有优化结果 |
| `result.at<Pose3>(key)` | 获取特定键的优化值 |

### 引用文献
[无]

