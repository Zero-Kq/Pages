+++
title = "LIO 学习笔记 02"
description = "关于 LIO SLAM 系统的学习"
date = 2026-04-18
draft = false
tags = ["slam", "学习笔记", "C++"]
categories = ["SLAM"]
math = true
+++

## 介绍

本文主要介绍 LIO SLAM 系统中的可视化工具 Pangolin 和日志工具 glog 的使用方法。

## 主要依赖

- Pangolin：3D 可视化库
- glog：Google 日志库

## 可视化

可视化功能在 `visualize.cpp` 中独立实现，创建单独的显示线程，使用原子布尔进行线程控制：

```cpp
std::atomic<bool> exit_flag_{false};
```

### 头文件

```cpp
#include <pangolin/pangolin.h>
#include <GL/gl.h>
```

### 原子布尔

原子布尔需要进行读写操作：

```cpp
exit_flag_.store(true);   // 写入
exit_flag_.load();        // 读取
```

原子布尔有**锁**机制，会强制同步线程的缓存值到主内存。

### 可视化窗口设置

```cpp
pangolin::CreateWindowAndBind("ZLIO Visualize", 1920, 1080);
```

投影参数配置：

```cpp
pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1920, 1080, 5000, 5000, 960, 540, 1, 1000),
    pangolin::ModelViewLookAt(0, -10, 20, 0, 0, 0, pangolin::AxisY));
```

| 参数 | 值 | 说明 |
|------|-----|------|
| ProjectionMatrix 第1/2个参数 | 1920, 1080 | 屏幕分辨率 |
| ProjectionMatrix 第3/4个参数 | 5000, 5000 | fx, fy 焦距 |
| ProjectionMatrix 第5/6个参数 | 960, 540 | cx, cy 主点（屏幕中心） |
| ProjectionMatrix 第7/8个参数 | 1, 1000 | near, far 显示深度 |
| ModelViewLookAt 第1/2/3个参数 | 0, -10, 20 | 相机位置 |
| ModelViewLookAt 第4/5/6个参数 | 0, 0, 0 | 相机观察目标点 |
| ModelViewLookAt 第7个参数 | pangolin::AxisY | Y 轴向上 |

### 按钮设置

创建左侧菜单面板：

```cpp
CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 第一个参数 | 0.0 | 左边界：屏幕最左边 |
| 第二个参数 | 1.0 | 右边界：屏幕最右边 |
| 第三个参数 | 0.0 | 下边界：屏幕最下面 |
| 第四个参数 | pangolin::Attach::Pix(180) | 上边界：离底部 180 像素 |

其他 Attach 用法：

| 用法 | 说明 |
|------|------|
| `Attach::Pix(180)` | 固定像素值 |
| `Attach::Fam(1.0)` | 相对比例（1.0 = 100%） |
| `Attach::Ref(0.5)` | 相对于父容器 |

配置按钮：

```cpp
pangolin::Var<bool> menu_reset_view("menu.Reset View", false, false);    // 重置视角按钮
pangolin::Var<float> menu_intensity("menu.Intensity", 1.0f, 0.1f, 10.0f); // 点云亮度调节
```

> 按钮无法修改 UI，据 AI 反馈可以和 ImGui 进行组合，后续再尝试。

### 显示

使用 while 循环进行显示，通过线程休眠控制帧率：

```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(30));
```

| 睡眠时间 | 约等于帧率 |
|----------|------------|
| sleep_for(16ms) | ~60 FPS |
| sleep_for(30ms) | ~33 FPS |
| sleep_for(100ms) | ~10 FPS |

#### 点云绘制

```cpp
void Visualize::RenderCurrentScan(float intensity_scale) {
    std::lock_guard<std::mutex> lock(mtx_cloud_);
    if (current_cloud_.points.empty()) {
        return;
    }

    // 清除并预分配变换后的点云数据缓冲区
    cloud_xyz_.clear();
    cloud_color_.clear();
    cloud_xyz_.reserve(current_cloud_.points.size());
    cloud_color_.reserve(current_cloud_.points.size());

    // 遍历每个点，进行坐标系变换和颜色计算
    for (const auto& pt : current_cloud_.points) {
        // 将点从机体坐标系变换到世界坐标系
        Eigen::Vector3f pt_body(pt.x, pt.y, pt.z);
        Eigen::Vector3f pt_world = TransformPoint(pt_body, current_pose_);

        cloud_xyz_.push_back(pt_world);

        // 根据点云强度值计算灰度颜色，并应用菜单缩放
        // intensity 归一化到 [0, 1] 范围
        float intensity = pt.intensity / 255.0f * intensity_scale;
        intensity = std::min(intensity, 1.0f);
        cloud_color_.push_back(Eigen::Vector3f(intensity, intensity, intensity));
    }

    // 使用 OpenGL 渲染点云
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < cloud_xyz_.size(); ++i) {
        const auto& pt = cloud_xyz_[i];
        const auto& color = cloud_color_[i];
        glColor3f(color.x(), color.y(), color.z());
        glVertex3f(pt.x(), pt.y(), pt.z());
    }
    glEnd();
}
```

#### 轨迹绘制

```cpp
void Visualize::RenderTrajectory() {
    std::lock_guard<std::mutex> lock(mtx_traj_);
    if (trajectory_pts_.empty()) {
        return;
    }

    // 设置线宽和颜色，绘制轨迹线
    glLineWidth(2.0f);
    glColor3f(traj_color_[0], traj_color_[1], traj_color_[2]);
    glBegin(GL_LINE_STRIP);
    for (const auto& pt : trajectory_pts_) {
        glVertex3f(pt.x(), pt.y(), pt.z());
    }
    glEnd();

    // 在轨迹终点绘制红色标记点，表示当前位置
    if (!trajectory_pts_.empty()) {
        const auto& last_pt = trajectory_pts_.back();
        glPointSize(5.0f);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        glVertex3f(last_pt.x(), last_pt.y(), last_pt.z());
        glEnd();
    }
}
```

#### 机体坐标轴绘制

```cpp
void Visualize::RenderCar() {
    std::lock_guard<std::mutex> lock(mtx_state_);

    // 从导航状态获取旋转矩阵和位置
    Eigen::Matrix3f R = nav_state_.orientation.matrix();
    Eigen::Vector3f t = nav_state_.position;

    // 计算车体坐标系三个轴的世界坐标表示
    // X轴: 前进方向(1,0,0), Y轴: 右侧(0,1,0), Z轴: 上方(0,0,1)
    Eigen::Vector3f forward = R * Eigen::Vector3f(1, 0, 0) * car_size_;
    Eigen::Vector3f right = R * Eigen::Vector3f(0, 1, 0) * car_size_ * 0.5f;
    Eigen::Vector3f up = R * Eigen::Vector3f(0, 0, 1) * car_size_ * 0.3f;

    glLineWidth(2.0f);

    // 使用 GL_LINES 绘制三条坐标轴
    glBegin(GL_LINES);
    // X 轴（前进方向）- 红色
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(t.x(), t.y(), t.z());
    glVertex3f(t.x() + forward.x(), t.y() + forward.y(), t.z() + forward.z());

    // Y 轴 - 绿色
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(t.x(), t.y(), t.z());
    glVertex3f(t.x() + right.x(), t.y() + right.y(), t.z() + right.z());

    // Z 轴 - 蓝色
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(t.x(), t.y(), t.z());
    glVertex3f(t.x() + up.x(), t.y() + up.y(), t.z() + up.z());
    glEnd();
}
```

## log 日志打印

日志打印直接设置参数并调用即可：

```cpp
#include <glog/logging.h>    // 头文件

// 初始化日志系统
google::InitGoogleLogging("zlio_test");
// 设置日志保存路径
google::SetLogDestination(google::GLOG_INFO, "/home/yj/ZLIO_ws/logs/zlio_test");
// 设置日志级别（小于该级别的日志不输出）
FLAGS_stderrthreshold = google::GLOG_WARNING;
// 立即刷新日志到文件
FLAGS_logbufsecs = 0;

// 使用日志宏
LOG(INFO) << "程序开始";      // 写入 INFO 日志
LOG(WARNING) << "警告";       // 写入 WARNING 日志
LOG(ERROR) << "错误";         // 写入 ERROR 日志

// 关闭 google 日志
google::ShutdownGoogleLogging();
```

只要链接到同一个程序，引用头文件 `#include <glog/logging.h>` 就可以直接调用宏进行日志打印。

## 参考

- 无
