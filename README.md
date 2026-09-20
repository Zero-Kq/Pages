# Zero 的个人博客

基于 [Hugo](https://gohugo.io/) 与 [PaperMod](https://github.com/adityatelange/hugo-PaperMod) 主题搭建的静态站点，用于记录技术学习笔记与成长历程。

## 在线访问

**主页：** [https://Zero-Kq.github.io/Pages/](https://Zero-Kq.github.io/Pages/)

## 内容概览

站点主要涵盖以下方向的学习笔记：

- SLAM / 点云配准 / LIO
- Linux 系统与桌面
- ROS
- Web / Hugo
- 感知与部署

## 本地开发

安装 Hugo 和 Node.js 22（包含 npm）。为避免不同 Hugo 版本的渲染差异，
本地应与 `.github/workflows/gh-pages.yml` 中的 `HUGO_VERSION` 保持一致（当前为 0.166.0）。

在 PowerShell 中运行：

```powershell
cd D:\Project\pages\Pages
npm ci
npm run dev
```

打开 http://localhost:1313/。保存文章、样式或模板后，会自动重新构建并刷新页面。
按 Ctrl+C 停止。端口被占用时，先停止旧的 `hugo server`。
Windows 下也会自动寻找 WinGet 安装的 Hugo，无需手动补 PATH。
其他安装位置可通过环境变量 `HUGO_BINARY` 指定可执行文件完整路径。

本地与 GitHub Actions 共用 `scripts/build.mjs`：Hugo 构建（包含草稿）后，
执行同一个 Shiki 高亮脚本。预览使用本地网址，并额外注入自动刷新脚本；
文章和代码高亮使用相同处理流程。不要使用普通 `hugo server` 代替这个预览命令，
因为它不会执行 Shiki。修改预览服务脚本本身后需要重启服务。

### Windows 启动，局域网访问

在 Windows PowerShell 中进入项目目录，设置 **运行预览服务的 Windows 电脑的 IPv4 地址**：

```powershell
cd Z:\Pages
npm.cmd ci
$env:HOST = "0.0.0.0"
$env:PORT = "1313"
$env:BASE_URL = "http://192.168.31.251:1313/"
npm.cmd run dev
```

`Z:\Pages` 是当前映射的项目目录，其他电脑请替换为自己的实际路径。
`192.168.31.251` 是本次检查时 Windows 主机的局域网地址；可运行 `ipconfig`
查看以太网或 Wi-Fi 的 IPv4，地址变化后请替换。即使项目文件存放在网络共享中，
访问地址也应使用运行 Node.js 的电脑 IP。`npm.cmd ci` 只需首次运行或依赖锁文件变化后执行。

本机和同一局域网的其他 Windows 电脑、手机都打开 `http://192.168.31.251:1313/`。
保持终端运行，按 Ctrl+C 停止。不要将 `BASE_URL` 设置为 `0.0.0.0`；它只是监听地址。
修改 `BASE_URL` 后需要重启服务。Windows 预览输出至本机 `%TEMP%\pages-preview\` 下按项目隔离的目录，避免 Hugo 重复写入网络共享盘时出现路径错误；Linux 仍使用项目中的 `.preview/`。

如果 Windows 防火墙提示，请允许 Node.js 通过专用网络。若本机能打开但其他设备不能，
先确认设备在同一局域网、Windows 当前网络为可信的专用网络，再在管理员 PowerShell 中放行该端口：

```powershell
New-NetFirewallRule -DisplayName "Pages preview 1313 (LAN)" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1313 -Profile Private -RemoteAddress LocalSubnet
```

如果提示找不到 `npm.cmd`，请安装 Node.js 22（包含 npm）并重新打开终端。
若依赖已安装且 `node` 可用，也可以用 `node scripts/preview.mjs` 启动。
对于 UNC 共享路径，建议先映射为盘符再使用 npm，避免 npm 的 Windows 命令解释器不支持 UNC 工作目录。

### Ubuntu 启动，Windows 访问

在 Ubuntu 终端进入项目实际目录，先按 Ctrl+C 停止原来的预览进程，然后运行：

```bash
npm ci
BASE_URL=http://192.168.31.102:1313/ npm run dev
```

在 Windows 浏览器打开 http://192.168.31.102:1313/。如果 Ubuntu 的 IP 改变，
同时修改启动命令和浏览器地址。`BASE_URL` 用于生成正确的页面链接和资源地址。
预览默认监听 `0.0.0.0:1313`，允许局域网设备访问；只需本机访问时可设置 `HOST=127.0.0.1`。
可通过 `PORT` 修改端口，同时应更新 `BASE_URL` 中的端口。

构建生产站点（输出至 `public/`）：

```powershell
npm run build
```

推送到 `main` 分支后，GitHub Actions 会自动部署到 GitHub Pages。
