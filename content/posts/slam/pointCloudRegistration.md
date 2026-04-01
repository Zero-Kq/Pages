+++
title = '点云配准'
description = "关于常用的点云配准对比使用"
date = '2026-04-01'
draft = true
tags = ["点云", "学习笔记", "C++"]
categories = ["SLAM"]
# weight = 1
# slug = ""
# aliases = []
# series = []
# externalLink = ""
# toc = true
# math = false
+++

## 点云配准方法介绍

常见的有ICP、GICP、NDT等，快捷实现基本基于PCL库中自带的点云处理实现

### GICP

原理：

#### 头文件包含

```c++
#include <pcl/point_types.h>          // 点类型定义
#include <pcl/point_cloud.h>          // 点云容器
#include <pcl/registration/gicp.h>    // GICP 算法核心
#include <pcl/registration/icp.h>     // ICP 基类（部分功能依赖）
#include <pcl/features/normal_3d.h>   // 法向量估计（GICP 需要）
#include <pcl/filters/voxel_grid.h>   // 下采样（可选）
#include <pcl/io/pcd_io.h>            // PCD 文件读写
```



