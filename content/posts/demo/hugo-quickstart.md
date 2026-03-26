+++
title = "Hugo 快速入门指南"
description = "快速上手 Hugo 静态网站生成器"
date = 2026-03-15
draft = false
tags = ["Hugo", "教程"]
categories = ["技术"]
+++

# Hugo 快速入门指南

## 安装 Hugo

在 Windows 上可以使用 Chocolatey 或 Scoop 安装：

```bash
scoop install hugo
```

## 创建新站点

```bash
hugo new site my-site
```

## 添加主题

```bash
cd my-site
git init
git submodule add https://github.com/adityatelange/hugo-PaperMod.git themes/PaperMod
```

## 启动开发服务器

```bash
hugo server
```

## 构建生产版本

```bash
hugo
```

这样就可以生成静态网站文件了！
