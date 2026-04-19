+++
title = "LIO 学习笔记 01"
description = "关于 LIO SLAM 系统的学习"
date = 2026-04-14
draft = false
tags = ["slam", "学习笔记", "c++"]
categories = ["SLAM"]
math = true
+++

## 介绍

打算实现读书时应该完成的工作，写一个自己的 SLAM，来对 SLAM 的实现有一个更深层次的了解，项目参考了 [FAST_LIO2](https://github.com/hku-mars/FAST_LIO)、[Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM)、[lightning-lm](https://github.com/gaoxiang12/lightning-lm)

## 数据结构定义和数据缓存

首先是对数据结构体进行创建，点、点云、IMU、同步后的数据体，为了结构清楚，我单独写一个头文件common.h方便查看和管理
点和点云通过PCL注册，hku是通过PointXYZNomal的曲率存储时间戳，lm是通过重新注册一个
IMU 包含时间戳、加速度计、陀螺仪
同步后的数据measuregroup 包含点云 两帧点云之间的所有IMU
之所以单独定义结构体，更多的是增强独立性，这样其他如ros或者雷达调用接口就可调用该里程计

数据缓存
imu_buffer_.push_back(imu)
lidar_buffer_.push_back(cloud)

### 点云创建和注册

```cpp
// PCL兼容点类型 (必须在全局或pcl命名空间定义)
struct PointXYZIT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    double timestamp;
    PointXYZIT() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp))
```

### 注意

每个数据在存储时会用到 `std::lock_guard<std::mutex> lock(mtx_buffer_);` 因为回调函数出发存储，避免和其他位置同时操作缓存导致错误

> **说明**：`std::lock_guard` 能自动解锁，在作用域结束、return、异常时自动解锁，作用域是从声明开始到所在的大括号结束

## ROS接口

创建ROS接口实现，算法模块与ROS系统解耦，虽然现在基本是在ROS系统中运行，但是解耦后更方便移植在ROS1和ROS2中，以及万一之后在其他嵌入式上运行

主要是对应的话题回调函数中调用slam借口实现

RosApp::CloudCallback
RosApp::IMUCallback

开始我自己创建的点云结构体，没有用PCL，所以不是很理解time_buffer的作用，三个参考项目都加了time_buffer，time_buffer中存储的是话题的时间戳，在时间同步时作为点云的扫描开始时间，那为什么不直接在点云存储时间戳，我就省略了time_buffer
我知道了，因为他们创建的点云结构体没有时间戳，我是自己定义的点云，考虑到点云只在同步时才需要时间，确实没必要多一个成员

fastlio对livox雷达会进行IMU和雷达的时间软同步处理，不是很知道区别对待的原图，其他的算法似乎没有

参考算法似乎对不同雷达还会有不同的处理，但是最后还是只用到了XYZI，以及用curvature记录不同雷达的点云时间偏移，lm则是用的动态平均值处理的点云时间偏移，计划先用lm的方法，后面再两个都加上吧

lm在IMU回调中还额外EKF处理了，估计是为了高频输出，后续再进行添加吧

回调函数中还通过跳点逻辑进行了降采样，我之前都是用的体素滤波，非常耗时

```cpp
inline PointCloudType DownSamplePointCloud(const PointCloudType& cloud, int point_filter_num) {
    PointCloudType downsampled;
    downsampled.reserve(cloud.size() / point_filter_num + 1);

    for (size_t i = 0; i < cloud.points.size(); i += point_filter_num) {
        downsampled.points.push_back(cloud.points[i]);
    }
    return downsampled;
}
```

跳点常用间隔参考：

| 项目 | LiDAR 类型 | point_filter_num |
|------|------------|------------------|
| FAST-LIO | AVIA | 3 |
| | Velodyne | 4 |
| | Mid360 | 3 |
| | Horizon | 3 |
| | SE3 | 1（不降采样） |
| Voxel-SLAM | Mid360 | 3 |
| | AVIA | 3 |
| | Velodyne | 3 |
| | Ouster | 1 |
| | Hesai | 1 |
| lightning-lm | Livox | 4 |
| | NCLT | 6 |
| | Robosense | 6 |
| | UTBM | 10 |


还有自定义点云使用相关函数如体素滤波需要在头文件中宏定义

```cpp
#define PCL_NO_PRECOMPILE  // 禁用PCL预编译，启用自定义点云类型模板实例化
```

不然编译会报错链接失败之类的

## 数据同步

三个好像都差不多，简单的逻辑，以雷达点云为标准，如果IMU数据不够（时间比雷达的时间早）就返回false，然后吧两帧之间的IMU及当前点云帧全部放入一个结构体中，同步的数据结构除了点云和IMU容器，还需要点云的开始扫描和结束扫描时间，用于去畸变。

## 参考

- **FAST_LIO源码**：https://github.com/hku-mars/FAST_LIO
- **Voxel-SLAM源码**：https://github.com/hku-mars/Voxel-SLAM
- **lightning-lm源码**：https://github.com/gaoxiang12/lightning-lm
