+++
title = '点云配准'
description = "关于常用的点云配准方法对比使用"
date = '2026-04-01'
draft = false
tags = ["学习笔记", "C++", "SLAM", "雷达"]
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

常见的有 ICP、GICP、NDT 等，快捷实现基本基于 PCL 库中自带的点云处理实现。

### GICP

#### 原理

Generalized Iterative Closest Point (GICP) 是一种改进的 ICP 算法，通过协方差矩阵优化对应点匹配，提升配准精度。

#### 头文件

```cpp
#include <pcl/point_types.h>          // 点类型定义
#include <pcl/point_cloud.h>          // 点云容器
#include <pcl/registration/gicp.h>    // GICP 算法核心
#include <pcl/registration/icp.h>     // ICP 基类（部分功能依赖）
#include <pcl/features/normal_3d.h>    // 法向量估计（GICP 需要）
#include <pcl/filters/voxel_grid.h>   // 下采样（可选）
#include <pcl/io/pcd_io.h>            // PCD 文件读写
```

#### 参数配置

| 参数 | 值 | 说明 |
|------|-----|------|
| setMaximumIterations | 500 | 最大迭代次数，达到后停止配准 |
| setTransformationEpsilon | 1e-10 | 变换矩阵变化的收敛阈值，小于此值认为已收敛 |
| setEuclideanFitnessEpsilon | 0.0001 | 点到点欧氏距离误差的收敛阈值 |
| setMaxCorrespondenceDistance | 50.0 | 对应点搜索半径上限，超过则不考虑 |
| setCorrespondenceRandomness | 20 | 每次迭代随机采样对应点数量，影响计算效率和精度 |
| setRANSACOutlierRejectionThreshold | 0.05 | RANSAC 离群点阈值，超过此距离的点被标记为异常值 |
| setRANSACIterations | 1000 | RANSAC 迭代次数 |
| setTransformationRotationEpsilon | 1e-10 | 旋转矩阵变化的收敛阈值 |
| setUseReciprocalCorrespondences | false | 是否使用互对应关系（source-target双向匹配） |

---

## 测试结果

通过将目标点云变换作为原始点云进行测试，实际变换为 T=[10, 10, 10]m, R=[90, 0, 0]deg。

初始让 AI 以 Score 为基准进行调参，获得"最优参数"，但实际误差依旧很大

#### 测试概述

| 分类 | 参数 | 值 |
|------|------|-----|
| 配置 | max_iterations | 500 |
| 配置 | transformation_epsilon | 1e-10 |
| 配置 | euclidean_fitness_epsilon | 0.0001 |
| 配置 | max_correspondence_distance | 50.0 |
| 配置 | correspondence_randomness | 20 |
| 配置 | ransac_iterations | 1000 |
| 配置 | ransac_outlier_rejection_threshold | 0.05 |
| 配置 | transformation_rotation_epsilon | 1e-10 |
| 配置 | use_reciprocal_correspondences | false |
| 输出 | Score | 0.891884 |
| 输出 | Time | 862.025 ms |
| 输出 | Trans Error | 2.14007 m |
| 输出 | Rot Error | 2.03481 deg |

#### 效果展示

![GICP 效果展示](/posts/slam/GICP.png)

图例：绿色=Target cloud，白色=Source cloud (before)，蓝色=Registration result

#### 测试分析

通过遍历调参优化 Score 指标，发现 `mcd=50, cr=20` 时 Score 可降至 0.89，但实际 Trans Error 仍达 2.14m。

**Score vs Trans/Rot Error 差异原因**：

1. **Score 是局部误差，Trans/Rot Error 是全局误差**
   - GICP 优化的是点到邻近点平面的距离（局部对齐质量）
   - 即使对应关系完全错误，只要局部结构相似，Score 也可能很低

2. **对称性欺骗**
   - 例如 180 度旋转后，点云局部结构仍然相似
   - GICP 可能收敛到"错误但局部最优"的解

3. **初始估计敏感**
   - GICP 是局部优化算法
   - 从单位矩阵出发，没有好的初始估计
   - 可能收敛到错误的局部最优而非全局最优

4. **mcd 太大的影响**
   - mcd=50 太大，允许更远的对应点匹配
   - 可能接受错误匹配，陷入局部最优

---

然后以 Trans Error 和 Rot Error 作为基准进行参数调整

#### 测试概述

| 分类 | 参数 | 值 |
|------|------|-----|
| 配置 | max_iterations | 500 |
| 配置 | transformation_epsilon | 1e-10 |
| 配置 | euclidean_fitness_epsilon | 0.0001 |
| 配置 | max_correspondence_distance | 17 |
| 配置 | correspondence_randomness | 18.5 |
| 配置 | ransac_iterations | 1000 |
| 配置 | ransac_outlier_rejection_threshold | 0.05 |
| 配置 | transformation_rotation_epsilon | 1e-10 |
| 配置 | use_reciprocal_correspondences | false |
| 输出 | Score | 2.9e-07 |
| 输出 | Time | 777 ms |
| 输出 | Trans Error | 0.0002 m |
| 输出 | Rot Error | 0 deg |

#### 效果展示

![最优error GICP 效果展示](/posts/slam/GICP_minerror.png)

#### 分析总结

| mcd | cr | Trans Error | Rot Error | Score |
|-----|----|-------------|-----------|-------|
| 50 | 20 | 2.14m | 2.03deg | 0.89 |
| **17** | **18.5** | **0.0002m** | **0deg** | **2.9e-07** |

优化目标从 Score 改为 Trans/Rot Error 后，找到真正有效的参数组合。

**关键发现**：

1. **Score 与 Trans/Rot Error 无直接关联**
   - Score 是点到平面距离的 RMSE（局部对齐质量）
   - Trans/Rot Error 是全局位姿精度
   - Score 低不等于配准准确

2. **陷入错误局部最优的原因**
   - mcd=50 太大：允许更远的对应点匹配，接受错误匹配
   - 初始估计敏感：从单位矩阵出发，没有好的初始估计
   - 对称性欺骗：180 度旋转后局部结构仍相似

3. **核心参数**：mcd 和 cr 决定配准成败
   - mcd 太大：接受错误匹配
   - mcd 太小：找不到正确对应
   - cr 偏离 18-20：陷入局部最优

**参数对耗时和分数的影响**：

| 参数 | 影响程度 | 对分数的影响 | 对耗时的影响 |
|------|---------|-------------|-------------|
| max_correspondence_distance | ★★★★★ | 决定是否收敛到正确解 | mcd 较小时耗时增加 |
| correspondence_randomness | ★★★★★ | 决定是否收敛到正确解 | cr 太大或太小都会增加耗时 |
| ransac_outlier_rejection_threshold | ☆☆☆☆☆ | 无影响 | 无影响 |
| ransac_iterations | ☆☆☆☆☆ | 无影响 | 无影响 |
| max_iterations | ☆☆☆☆☆ | 无影响 | 无影响 |
| transformation_epsilon | ☆☆☆☆☆ | 无影响 | 无影响 |
| euclidean_fitness_epsilon | ☆☆☆☆☆ | 无影响 | 无影响 |
| transformation_rotation_epsilon | ☆☆☆☆☆ | 无影响 | 无影响 |
| use_reciprocal_correspondences | ☆☆☆☆☆ | 无影响 | 无影响 |

**耗时分析**：

| mcd | cr | Time | Trans Error |
|-----|----|------|-------------|
| 5 | 18 | 889ms | 17.3m |
| 17 | 18.5 | 777ms | 0.0002m |
| 20 | 18 | 461ms | 0.0012m |
| 50 | 20 | 519ms | 20.1m |

- **追求精度**：mcd=17, cr=18.5，耗时约 777ms
- **追求速度**：mcd=20, cr=18，耗时约 461ms，精度仍可达标

**最优配置**：

```yaml
max_correspondence_distance: 17
correspondence_randomness: 18.5
max_iterations: 500
```

---

### NDT + GICP

#### 参数配置

| 参数 | 值 | 说明 |
|------|-----|------|
| ndt.transformation_epsilon | 0.005 | NDT 变换矩阵收敛阈值 |
| ndt.step_size | 1 | NDT 步长 |
| ndt.resolution | 150 | NDT 栅格分辨率 |
| ndt.max_iterations | 300 | NDT 最大迭代次数 |
| gicp.max_iterations | 200 | GICP 最大迭代次数 |
| gicp.transformation_epsilon | 1e-10 | GICP 变换矩阵收敛阈值 |
| gicp.euclidean_fitness_epsilon | 0.0001 | GICP 欧氏距离收敛阈值 |
| gicp.max_correspondence_distance | 17.0 | GICP 对应点搜索半径上限 |
| gicp.correspondence_randomness | 50 | GICP 随机对应点数量 |
| gicp.ransac_outlier_rejection_threshold | 0.05 | GICP RANSAC 离群点阈值 |

#### 测试概述

| 分类 | 参数 | 值 |
|------|------|-----|
| 点数 | Target/Source | 21528 points |
| 实际变换 | T | [10, 10, 10] m |
| 实际变换 | R | [90, 0, 0] deg |
| NDT | transformation_epsilon | 0.005 |
| NDT | step_size | 1 |
| NDT | resolution | 150 |
| NDT | max_iterations | 300 |
| GICP | max_iterations | 200 |
| GICP | transformation_epsilon | 1e-10 |
| GICP | euclidean_fitness_epsilon | 0.0001 |
| GICP | max_correspondence_distance | 17.0 |
| GICP | correspondence_randomness | 50 |
| GICP | ransac_outlier_rejection_threshold | 0.05 |
| 输出 | NDT Time | 5023.26 ms |
| 输出 | GICP Time | 198.718 ms |
| 输出 | Total Time | 5225.32 ms |
| 输出 | Score | 4.43e-07 |
| 输出 | Trans Error | 0.0002 m |
| 输出 | Rot Error | 0 deg |

#### 效果展示

![NDT+GICP 匹配效果展示](/posts/slam/NDT+GICP.png)

#### 总结分析

NDT的耗时过长，并不适合进行slam的回环配准，可用于初始化配准，因为GICP需要一个比较好的初始位姿，但是按纯GICP配准中，似乎初始位姿不好也能实现配准？
