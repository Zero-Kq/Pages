+++
title = '无人机仿真环境搭建指南'
description = "介绍 ROS、PX4-Autopilot、Gazebo、MAVROS 与 QGroundControl 的安装与配置流程，用于搭建完整的无人机仿真开发环境"
date = '2026-03-11'
draft = false
tags = ["学习笔记", "工具"]
categories = ["ROS"]
toc = true
+++

## 介绍

本文记录无人机仿真开发环境的完整搭建流程，涵盖 ROS 安装、PX4-Autopilot 源码编译、Gazebo 仿真环境配置、MAVROS 通信中间件安装，以及 QGroundControl 地面站部署。

## 1. 安装 ROS

使用鱼香 ROS 一键安装脚本：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 2. 安装 PX4-Autopilot

### 2.1 下载源码

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
```

### 2.2 安装依赖

解压缩后执行：

```bash
cd ~/Desktop/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

### 2.3 添加环境路径

编辑 `~/.bashrc` 文件，添加以下内容：

```bash
source ~/桌面/PX4-Autopilot/Tools/setup_gazebo.bash ~/桌面/PX4-Autopilot ~/桌面/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/桌面/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/桌面/PX4-Autopilot/Tools/sitl_gazebo
# 具体路径取决于实际文件环境
```

使环境变量生效：

```bash
source ~/.bashrc
```

此时开启终端会打印以下信息：

```terminal
GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH
GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH
LD_LIBRARY_PATH $LD_LIBRARY_PATH
```

可通过注释 `setup_gazebo.bash` 最后三句关闭打印。

### 2.4 编译 PX4 SITL

在 PX4-Autopilot 目录下执行：

```bash
make px4_sitl_default gazebo
```

## 3. 安装 MAVROS

### 3.1 二进制安装

```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
```

### 3.2 安装 GeographicLib 数据集

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

由于网络问题容易失败，可通过 GitHub 镜像获取：

```bash
git clone https://github.91chi.fun/https://github.com/wyfroom/mavros-GeographicLib.git
```

将 `Geographiclib` 文件夹放在 `/usr/share` 路径下。

## 4. 安装 QGroundControl

### 4.1 配置环境

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

### 4.2 下载启动程序

从 QGroundControl 发布页面下载 `QGroundControl.AppImage`：

```text
# 下载地址
# https://github.com/mavlink/qgroundcontrol/releases/tag/v4.3.0
```

下载后设置为可执行文件：

```bash
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

## 参考

- **PX4-Autopilot 源码**：https://github.com/PX4/PX4-Autopilot
- **鱼香 ROS 一键安装**：http://fishros.com/
- **MAVROS 文档**：https://github.com/mavlink/mavros
- **QGroundControl 发布页**：https://github.com/mavlink/qgroundcontrol/releases/tag/v4.3.0
