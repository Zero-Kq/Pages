+++
title = 'Ubuntu ROS 环境安装与配置指南'
description = "介绍 Ubuntu 下 ROS2 Foxy 的安装步骤、系统环境检查、ROS1 与 ROS2 切换配置，以及编译命令和终端指令对照"
date = '2026-03-12'
draft = false
tags = ["linux", "学习笔记"]
categories = ["ROS"]
toc = true
+++

## 介绍

本文介绍在 Ubuntu 系统上安装与配置 ROS 环境的完整流程，包括系统编码与软件源检查、ROS2 Foxy 安装、ROS1 与 ROS2 环境切换、编译命令差异及常用终端指令对照。

## 1. 检查系统设置

### 1.1 检查编码 UTF-8

终端输入：

```bash
locale
```

出现如下输出信息说明编码为 UTF-8：

```terminal
LANG=zh_CN.UTF-8
LANGUAGE=zh_CN:en_US:en
LC_CTYPE="zh_CN.UTF-8"
LC_NUMERIC=zh_CN.UTF-8
LC_TIME=zh_CN.UTF-8
LC_COLLATE="zh_CN.UTF-8"
LC_MONETARY=zh_CN.UTF-8
LC_MESSAGES="zh_CN.UTF-8"
LC_PAPER="zh_CN.UTF-8"
LC_NAME="zh_CN.UTF-8"
LC_ADDRESS="zh_CN.UTF-8"
LC_TELEPHONE="zh_CN.UTF-8"
LC_MEASUREMENT="zh_CN.UTF-8"
LC_IDENTIFICATION="zh_CN.UTF-8"
LC_ALL=
```

### 1.2 检查是否启用 Ubuntu Universe 存储库

终端输入：

```bash
apt-cache policy | grep universe
```

若输出如下则表示已启用：

```terminal
500 http://security.ubuntu.com/ubuntu focal-security/universe i386 Packages
     release v=20.04,o=Ubuntu,a=focal-security,n=focal,l=Ubuntu,c=universe,b=i386
500 http://security.ubuntu.com/ubuntu focal-security/universe amd64 Packages
    release v=20.04,o=Ubuntu,a=focal-security,n=focal,l=Ubuntu,c=universe,b=amd64
```

若无输出，执行以下命令启用：

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 1.3 设置软件包签名密钥

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 2. 安装

```bash
sudo apt install ros-foxy-desktop
```

## 3. 配置环境变量

为方便在 ROS1 和 ROS2 之间切换，可在 `~/.bashrc` 中添加以下函数：

```bash
# >>> type in ros to choose which to initialize >>>
ros(){
  echo "ros: noetic(1) foxy(2)?"
  read edition
  if [ "$edition" -eq "1" ];then
    source /opt/ros/noetic/setup.bash

    echo using ROS1 Noetic
  else
    source /opt/ros/foxy/setup.bash

    echo using ROS2 Foxy
  fi
}
# <<< type in ros/ros2 to initialize <<<
```

根据需要修改工作空间路径，然后执行 `source ~/.bashrc` 使配置生效。

## 4. 编译命令差异

```bash
# ROS1
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]...
# catkin_create_pkg test_pkg std_msgs rospy roscpp
catkin_make
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

# ROS2
colcon build
# 只编译指定包
colcon build --packages-select PACKAGE_NAME
# 忽略指定包
colcon build --packages-ignore PACKAGE_NAME
# 遇到编译错误继续编译其他模块
colcon build --continue-on-error
```

## 5. 工作空间配置

```bash
source ~/catkin_ws/devel/setup.bash
source ~/colcon_ws/install/setup.bash
```

## 6. 终端命令对照

| ROS1 命令 | ROS2 命令 |
| --------- | --------- |
| `rosrun` | `ros2 run` |
| `rosnode` | `ros2 node` |
| `roslaunch` | `ros2 launch` |
| `rosparam` | `ros2 param` |
| `rospkg` | `ros2 pkg` |
| `rosservice` | `ros2 service` |
| `rossrv` | `ros2 srv` |
| `rostopic` | `ros2 topic` |
| `rosaction` | `ros2 action` |

## 参考

- **ROS2 官方安装文档**：https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- **ROS Noetic 安装文档**：http://wiki.ros.org/noetic/Installation/Ubuntu
- **colcon 编译工具文档**：https://colcon.readthedocs.io/
