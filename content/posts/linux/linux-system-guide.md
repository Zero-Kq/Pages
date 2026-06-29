+++
title = "Linux 系统配置与问题解决指南"
description = "Linux 系统中常见问题的解决方案，包括 NVIDIA 驱动安装、桌面环境故障排除、WPS 字体配置、输入法设置、终端配置以及 GNOME 桌面美化等内容"
date = 2024-02-11
draft = false
tags = ["Linux", "Ubuntu", "实用工具"]
categories = ["学习笔记"]
+++

## 1. Nvidia 驱动配置

### 1.1 禁用开源 nouveau 驱动

与 NVIDIA 专有驱动冲突，核心步骤如下：

```bash
# 编辑黑名单文件
sudo nano /etc/modprobe.d/blacklist-nouveau.conf
```

写入以下内容：

```text
blacklist nouveau
options nouveau modeset=0
```

更新并重启：

```bash
sudo update-initramfs -u
sudo reboot
```

验证是否禁用：

```bash
lsmod | grep nouveau
```

查看可用驱动版本：

```bash
ubuntu-drivers devices
```

安装指定版本驱动：

```bash
sudo apt install nvidia-driver-535 nvidia-settings -y
```

重启使配置生效：

```bash
sudo reboot
nvidia-smi
```

### 1.2 避免自动安装问题

```bash
sudo ubuntu-drivers autoinstall
```

> **注意**：此命令会安装新内核，可能导致兼容性问题，建议手动安装。

---

## 2. 桌面环境故障排除

### 2.1 桌面无法进入（左上角光标闪烁）

可能原因：`/var/log/syslog.1` 过大导致系统卡顿或无法继续引导。

**排查与处理步骤：**

首先检查磁盘与内存使用情况：

```bash
df -h        # 查看磁盘使用情况
free -h      # 查看内存使用情况
```

若确定是日志文件过大，可清空（谨慎操作）：

```bash
sudo sh -c 'echo > /var/log/syslog.1'
```

使用 `logrotate` 管理日志大小（可编辑 rsyslog 的 logrotate 配置）：

编辑文件：`/etc/logrotate.d/rsyslog`

示例配置（控制 `/var/log/syslog` 大小与轮转）：

```text
/var/log/syslog
{
        size 200M
        rotate 4
        daily
        missingok
        notifempty
        delaycompress
        compress
        postrotate
                /usr/lib/rsyslog/rsyslog-rotate
        endscript
}
```

保存后退出编辑器（例如 `nano` 使用 `Ctrl+O` 保存，`Ctrl+X` 退出）。

重启 rsyslog 服务：

```bash
sudo systemctl restart rsyslog
```

---

## 3. WPS Office 配置

### 3.1 WPS 字体缺失问题

符号或特殊字体缺失的解决方案：

参考文档：[WPS 字体配置详解](https://www.cnblogs.com/librarookie/p/14655922.html)

**方法一**（解压到 WPS 的字体目录）：

```bash
sudo unzip wps_symbol_fonts.zip -d /usr/share/fonts/wps-office
# 若目录不存在，可先创建：sudo mkdir -p /usr/share/fonts/wps-office
```

**方法二**（通用流程：解压 → 生成索引 → 更新缓存）：

```bash
# 将字体文件放入任意系统字体目录（如 /usr/share/fonts 或 ~/.local/share/fonts）
sudo mkfontscale
sudo mkfontdir
sudo fc-cache -fv
```

> **说明**：完成后重启 WPS 或重新登录会话以确保字体生效。

### 3.2 安装宋体（Simsun）

参考文档：[Ubuntu 安装宋体详细教程](https://blog.csdn.net/LclLsh/article/details/132509872)

可从可信网站下载 `Simsun.ttf`，然后安装：

```bash
sudo mkdir -p /usr/share/fonts/chinese # 在 fonts 目录下创建自己的目录
sudo cp Simsun.ttf /usr/share/fonts/chinese # 将字体文件拷贝过去
sudo mkfontscale
sudo mkfontdir
sudo fc-cache -fv
```

完成后重新启动需要使用该字体的程序。

---

## 4. 输入法配置

修改候选词数量与显示可以提升输入体验。

![修改候选词和候选词数量](/images/posts/linux/image.png)

根据不同输入法（如 Fcitx、IBus）在设置中调整候选词数量与样式。

---

## 5. 终端配置

### 5.1 安装 Terminator 终端

```bash
sudo apt-get update
sudo apt-get install terminator
```

安装后右键终端窗口可以进入首选项，调整配色、字体和布局。

### 5.2 设置右键打开 Terminator

在文件管理器右键菜单中集成 Terminator 的步骤：

**1. 安装插件：**

```bash
sudo apt-get install nautilus-actions filemanager-actions
```

**2. 运行配置工具：**

```bash
fma-config-tool
```

**3. 新建动作：**

![新建动作界面](/images/posts/linux/image-1.png)

**4. 新建命令**（设置执行路径与参数）：

![新建命令界面](/images/posts/linux/image-2.png)

- 路径：`/usr/bin/terminator`
- 参数：`--working-directory=%d/%b`

**5. 修改 Filemanager 配置**（点击左上角 Filemanager…tool 的 Preference）：

![Filemanager 配置界面](/images/posts/linux/image-3.png)

**6. 重启文件管理器**以应用更改：

```bash
nautilus -q
```

---

## 6. GNOME 桌面美化

### 6.1 安装扩展管理器

使用 GNOME 扩展管理器来安装美化扩展：

```bash
sudo apt install gnome-shell-extension-manager
```

### 6.2 推荐扩展

在 `gnome-shell-extension-manager` 中搜索并安装以下扩展：

- **Dash to Dock**：用于管理并美化 Dock（任务栏）
- **Hide Top Bar**：隐藏或自动折叠顶部栏以节省屏幕空间

### 6.3 管理 Ubuntu Dock

若需要删除默认的 Ubuntu Dock，使用：

```bash
sudo apt remove -y gnome-shell-extension-ubuntu-dock
```

如需恢复或重新安装：

```bash
sudo apt install -y gnome-shell-extension-ubuntu-dock
```

### 6.4 解决双侧边栏冲突

Ubuntu Dock 和 Dash to Dock 冲突导致，需要卸载 Ubuntu Dock 然后安装 Dash to Dock：

```bash
sudo apt install git make gettext # 安装依赖
git clone https://github.com/micheleg/dash-to-dock.git -b gnome-3.38
cd dash-to-dock/
make
make install
```

---

## 7. 多媒体播放支持

### 7.1 MP4 播放支持

安装 VLC 播放器及相关编解码器：

```bash
sudo apt install ubuntu-restricted-extras
sudo apt-get install vlc
```

---

## 8. 系统恢复

### 8.1 设置黑屏崩溃恢复

当遇到黑屏崩溃时，使用以下步骤恢复：

1. 按 `Ctrl+Alt+F3` 进入终端
2. 执行系统升级和重新安装桌面环境：

```bash
sudo apt upgrade
sudo apt install --reinstall ubuntu-desktop
```

---

## 参考文档

- [NVIDIA 驱动安装官方指南](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)
- [Ubuntu 日志管理最佳实践](https://ubuntu.com/server/docs/log-files)
- [WPS Linux 版字体配置](https://www.cnblogs.com/librarookie/p/14655922.html)
- [Ubuntu 安装宋体详细教程](https://blog.csdn.net/LclLsh/article/details/132509872)
- [GNOME Extensions 官方文档](https://extensions.gnome.org/)
- [Terminator 终端配置指南](https://terminator-gtk3.readthedocs.io/en/latest/)
