+++
title = 'GitHub 基础使用指南'
description = "介绍如何使用 Personal Access Token 克隆私人仓库、保存 Git 凭据，以及配置全局与局部用户名和邮箱"
date = '2026-03-11'
draft = false
tags = ["工具", "学习笔记"]
categories = ["其他"]
toc = true
+++

## 介绍

本文介绍 GitHub 日常开发中的基础操作，包括使用 Personal Access Token 克隆私人仓库、保存 Git 凭据避免重复输入，以及配置全局与局部的用户名和邮箱。

## 1. 拉取私人仓库

### 1.1 创建 Personal Access Token

1. 登录 GitHub，点击右上角头像 → `Settings`
2. 左侧菜单选择 `Developer settings`
3. 选择 `Personal access tokens` → `Fine-grained tokens`
4. 点击 `Generate new token` → `Generate new token (classic)`
5. 设置名称（如：我的开发令牌）
6. 选择过期时间（建议选择较长时间，如 90 天或自定义）
7. 选择仓库范围 `All repositories` 所有仓库，或者其他选项
8. 勾选权限：
   - 选择对应的权限领域 `repositories`（仓库）、`Account`（账户）
   - 仓库中选择 `Contents` 可读写，其它权限按需设置
9. 点击 `Generate token`
10. **重要**：复制生成的 token（只显示一次，请妥善保存）

### 1.2 使用 Token 克隆仓库

在命令行中执行：

```bash
git clone https://github.com/用户名/仓库名.git
```

当提示输入用户名和密码时：

| 提示项 | 填写内容 |
| ------ | -------- |
| Username | 你的 GitHub 用户名 |
| Password | 刚才复制的 Personal Access Token（不是你的 GitHub 密码） |

示例：

```text
# Username: zhangsan
# Password: ghp_xxxxxxxxxxxxxxxxxxxx（你的 token）
```

## 2. Git 保存密码

在 `git pull` 输入用户名和密码之后，执行以下命令保存凭据：

```bash
git config --global credential.helper store
```

执行后，Git 会将凭据写入 `~/.git-credentials`，后续操作无需重复输入。

## 3. 设置用户名与邮箱

### 3.1 全局配置

适用于当前设备上的所有仓库：

```bash
git config --global user.name "你的用户名"
git config --global user.email "zhangsan@example.com"
```

### 3.2 局部配置

仅在当前仓库目录下生效，会覆盖全局配置：

```bash
git config user.name "zhangsan"
git config user.email "zhangsan@example.com"
```

### 3.3 查看配置

| 命令 | 说明 |
| ---- | ---- |
| `git config user.name` | 查看当前仓库的用户名 |
| `git config user.email` | 查看当前仓库的邮箱 |
| `git config --global user.name` | 查看全局用户名 |
| `git config --global user.email` | 查看全局邮箱 |

## 参考

- **GitHub Personal Access Token 文档**：https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens
- **Git 凭据存储文档**：https://git-scm.com/docs/gitcredentials
- **Git 配置文档**：https://git-scm.com/docs/git-config
