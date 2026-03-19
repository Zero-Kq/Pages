+++
title = "Gtsam 学习笔记"
description = "关于 Gtsam 优化库的学习"
date = 2026-03-19
draft = true
tags = ["Hugo", "PaperMod", "学习笔记"]
categories = ["SLAM"]
+++

## 介绍Gtsam

Gtsam（General and Tunable Structure-from-Motion）是一个用于结构化从运动学（SFM）的优化库。Gtsam提供了一种基于图优化的算法，用于解决各种类型的结构化从运动学问题。Gtsam支持多种优化算法，如Levenberg-Marquardt、Gauss-Newton、Dogleg、Gauss-Newton-Dogleg等，以及多种优化目标，如最小二乘、最小 Trust Region等。Gtsam提供了多种数据结构，如变量、因子、 优化问题等，以及多种工具函数，如线性方程求解、矩阵运算、向量运算等。（AI生成）

## Gtsam的使用

### 快速使用

1. 引入头文件
```c++
#include <gtsam/slam/PriorFactor.h>
```

2. 创建变量

