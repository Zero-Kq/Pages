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

构建生产站点（输出至 `public/`）：

```powershell
npm run build
```

推送到 `main` 分支后，GitHub Actions 会自动部署到 GitHub Pages。
