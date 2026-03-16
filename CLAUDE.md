# CLAUDE.md

本文档为 Claude Code (claude.ai/code) 在本项目中工作提供指导。

## 项目概述

这是一个使用 **PaperMod 主题** 的 **Hugo 静态站点**。站点通过 GitHub Actions 在推送到 main 分支时自动部署到 **GitHub Pages**。

## 语言偏好

与用户交流时使用**中文**。

## 命令

### 开发

```bash
# 本地启动站点并实时重载
hugo server

# 为生产环境构建站点
hugo

# 构建包含草稿的站点（用于预览未发布的内容）
hugo --buildDrafts
```

### GitHub Actions 部署

站点会在推送到 main 分支时自动部署。手动触发：
1. 前往 GitHub 的 Actions 页面
2. 选择 "Deploy Hugo PaperMod Site to Pages"
3. 点击 "Run workflow"（可选择指定 Hugo 版本）

默认使用 Hugo 版本 `0.146.0`。

## 架构

### 目录结构

- `content/` - Markdown 内容文件（文章、页面）
- `layouts/` - 自定义 Hugo 模板（覆盖主题）
- `themes/PaperMod/` - 主题文件（本地克隆，非 git 子模块）
- `static/` - 静态资源（图片、CSS、JS）
- `assets/` - 构建资源（由 Hugo Pipes 处理）
- `data/` - Hugo 数据文件（YAML、JSON、TOML）
- `i18n/` - 国际化文件
- `archetypes/` - 内容脚手架模板

### 配置

- `hugo.toml` - Hugo 主配置
- `.github/workflows/gh-pages.yml` - GitHub Actions 工作流

### 内容组织

内容使用 Hugo 标准组织方式。Markdown 文件的 front matter 控制页面属性（标题、日期、草稿状态等）。

### 主题自定义

要自定义 PaperMod，在 `layouts/` 中创建与主题结构相同的覆盖文件。将自定义 CSS 放入 `assets/css/`，然后在 `hugo.toml` 中引用或通过 Hugo 资源管道引入。
