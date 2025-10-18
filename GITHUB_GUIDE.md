# GitHub 上传指南

本指南将指导您如何将RoboCup 2025项目上传到GitHub并进行管理。

## 📋 准备工作

### 1. 检查文件

在上传前，确保以下文件已准备好：

```bash
cd /path/to/2025参赛项目

# 检查核心文档
ls -l README.md LICENSE .gitignore
ls -l INSTALLATION.md QUICK_START.md CONTRIBUTING.md

# 检查目录README
ls -l catkin_ws/README.md
ls -l Mydataset/README.md
ls -l docs/README.md
```

### 2. 清理不必要的文件

```bash
# 删除编译文件
cd catkin_ws
rm -rf build/ devel/ logs/

# 清理临时文件
find . -name "*.pyc" -delete
find . -name "__pycache__" -delete
find . -name "*.log" -delete
```

### 3. 检查文件大小

```bash
# 查找大文件（>50MB）
find . -type f -size +50M

# 这些文件可能需要Git LFS：
# - YOLO模型权重 (*.pt)
# - 训练数据集图像
# - 视频文件
```

## 🚀 初始化Git仓库

### 步骤1: 初始化

```bash
cd /path/to/2025参赛项目

# 初始化Git仓库
git init

# 配置用户信息
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

### 步骤2: 添加文件

```bash
# 添加所有文件（.gitignore会自动排除）
git add .

# 检查将要提交的文件
git status
```

### 步骤3: 首次提交

```bash
# 提交
git commit -m "Initial commit: RoboCup 2025 Multi-UAV System

- 完整的6机协同追踪系统
- YOLOv11目标检测
- 安全航点规划
- 协同避免重复机制
- 完整的文档体系

Copyright © 2025 DHU AI Innovation Lab"
```

## 🌐 创建GitHub仓库

### 在GitHub上创建

1. 登录 GitHub（个人账号）
2. 点击 "New repository"
3. 填写信息：
   - **Repository name**: `robocup2025` 或其他名称
   - **Description**: "RoboCup 2025 Multi-UAV Cooperative Tracking System - DHU AI Lab"
   - **Visibility**: 
     - **Private** ✅ 推荐（内部使用）
     - Public ❌ 不推荐（包含敏感信息）
   - **不要** 初始化README、.gitignore或LICENSE（我们已有）

4. 点击 "Create repository"

**注意**：
- 本项目由个人GitHub账号维护
- 东华大学人工智能创新实验室没有公开GitHub组织
- 实验室内部协作使用飞书

### 连接远程仓库

```bash
# 添加远程仓库
git remote add origin https://github.com/lansijian/aerial-vision-system.git

# 验证
git remote -v
```

### 推送代码

```bash
# 推送到GitHub
git push -u origin main

# 如果分支名是master
# git branch -M main
# git push -u origin main
```

## 🔒 设置访问权限

### Private仓库权限管理

1. 进入仓库 Settings
2. 选择 "Manage access"
3. 点击 "Invite a collaborator"
4. 添加实验室成员：
   - 输入GitHub用户名或邮箱
   - 选择权限级别：
     - **Write** - 可以推送代码
     - **Read** - 只能查看

### 团队管理

建议创建GitHub Team：

1. 组织设置 → Teams
2. 创建 "RoboCup 2025 Team"
3. 添加成员
4. 将team添加到仓库

## 📦 处理大文件（Git LFS）

### 安装Git LFS

```bash
# Ubuntu
sudo apt install git-lfs

# 初始化
git lfs install
```

### 配置LFS

```bash
# 追踪大文件类型
git lfs track "*.pt"
git lfs track "*.pth"
git lfs track "*.h5"
git lfs track "*.onnx"

# 添加.gitattributes
git add .gitattributes
git commit -m "Configure Git LFS for model files"
```

### 添加模型文件

```bash
# 添加YOLO模型
git add catkin_ws/src/yolov11_ros/weights/*.pt
git commit -m "Add YOLO model weights"
git push
```

## 📝 仓库设置

### 1. 设置README

GitHub会自动显示根目录的README.md作为首页。

### 2. 添加描述和标签

在仓库Settings中：
- **Description**: "RoboCup 2025 Multi-UAV Cooperative Tracking System - DHU AI Lab"
- **Topics**: `robocup`, `ros`, `px4`, `yolo`, `multi-uav`, `rescue-simulation`

### 3. 设置分支保护

Settings → Branches → Add rule:

```
Branch name pattern: main

✅ Require pull request reviews before merging
✅ Require status checks to pass before merging
✅ Include administrators
```

### 4. 禁用Wiki和Projects

如果不需要：
- Settings → Features
- 取消 "Wikis"
- 取消 "Projects"

## 🔄 日常使用流程

### 开发流程

```bash
# 1. 创建功能分支
git checkout -b feature/new-feature

# 2. 进行修改
# ... 编辑代码 ...

# 3. 提交更改
git add .
git commit -m "feat: add new feature"

# 4. 推送分支
git push origin feature/new-feature

# 5. 在GitHub上创建Pull Request

# 6. 代码审查通过后合并
```

### 同步代码

```bash
# 获取最新代码
git pull origin main

# 解决冲突（如有）
git mergetool

# 推送修改
git push origin main
```

## 📊 管理Releases

### 创建版本发布

1. 进入 "Releases"
2. 点击 "Draft a new release"
3. 填写信息：
   - **Tag**: `v1.0.0`
   - **Release title**: "RoboCup 2025 - Version 1.0.0"
   - **Description**: 版本说明
4. 附加文件（可选）：
   - 训练好的模型
   - 使用手册PDF
5. 发布

### 版本号规范

使用语义化版本：`v主版本.次版本.修订号`

- `v1.0.0` - 初始发布
- `v1.1.0` - 新增功能
- `v1.0.1` - Bug修复

## 🛡️ 安全注意事项

### 不要提交的内容

❌ **绝对不要提交**:
```
- 密码和API密钥
- 私人配置文件
- 大型数据集（使用Git LFS或外部存储）
- 编译文件和临时文件
- 个人身份信息
```

### 检查敏感信息

```bash
# 搜索可能的敏感信息
git grep -i "password"
git grep -i "api_key"
git grep -i "secret"
```

### 移除已提交的敏感文件

```bash
# 从历史中移除文件
git filter-branch --tree-filter 'rm -f path/to/sensitive/file' HEAD

# 或使用BFG Repo-Cleaner
java -jar bfg.jar --delete-files sensitive_file.txt
```

## 📚 文档维护

### 保持文档更新

```bash
# 修改文档后提交
git add README.md docs/
git commit -m "docs: update documentation"
git push
```

### 自动生成文档

可以使用GitHub Actions自动生成文档网站。

## 🤝 团队协作

### Issue管理

创建Issue模板：

`.github/ISSUE_TEMPLATE/bug_report.md`:
```markdown
---
name: Bug报告
about: 报告一个bug
---

**Bug描述**
简要描述bug

**复现步骤**
1. ...
2. ...

**预期行为**
应该发生什么

**实际行为**
实际发生了什么

**环境信息**
- OS: Ubuntu 20.04
- ROS: Noetic
- PX4: v1.12.3
```

### Pull Request模板

`.github/PULL_REQUEST_TEMPLATE.md`:
```markdown
## 变更说明

简要描述此PR的变更

## 变更类型

- [ ] Bug修复
- [ ] 新功能
- [ ] 代码优化
- [ ] 文档更新

## 测试

- [ ] 已通过单元测试
- [ ] 已通过仿真测试
- [ ] 已更新文档

## 相关Issue

Closes #(issue编号)
```

## 🔍 监控和统计

### GitHub Insights

查看仓库统计：
- Contributors - 贡献者
- Commits - 提交历史
- Code frequency - 代码频率
- Network - 分支网络

### 添加Badges

在README.md中添加徽章：

```markdown
[![License](https://img.shields.io/badge/License-DHU--AIIL-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Melodic-green.svg)](http://wiki.ros.org/melodic)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
```

## 📱 移动端访问

GitHub移动应用支持：
- 查看代码
- 审查PR
- 管理Issues
- 接收通知

## ⚙️ GitHub Actions (可选)

创建自动化工作流：

`.github/workflows/test.yml`:
```yaml
name: ROS Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v2
      - name: Install ROS
        run: |
          sudo apt update
          sudo apt install ros-noetic-desktop-full
      - name: Build
        run: |
          source /opt/ros/noetic/setup.bash
          cd catkin_ws
          catkin_make
      - name: Test
        run: |
          source catkin_ws/devel/setup.bash
          catkin_make run_tests
```

## 📞 获取帮助

### GitHub资源

- [GitHub文档](https://docs.github.com/)
- [Git教程](https://git-scm.com/book/zh/v2)
- [GitHub Skills](https://skills.github.com/)

### 常见问题

**Q: Push被拒绝？**
```bash
# 先pull再push
git pull --rebase origin main
git push origin main
```

**Q: 忘记添加.gitignore？**
```bash
# 清除缓存
git rm -r --cached .
git add .
git commit -m "chore: update .gitignore"
```

## ✅ 上传检查清单

上传前确认：

- [ ] 所有敏感信息已移除
- [ ] .gitignore配置正确
- [ ] README.md完整清晰
- [ ] LICENSE文件存在
- [ ] 大文件已配置LFS
- [ ] 文档已更新
- [ ] 代码已测试
- [ ] 提交信息规范
- [ ] 仓库设置为Private
- [ ] 团队成员已添加

## 📄 许可证说明

在GitHub上明确标注：

> **Copyright © 2025 东华大学人工智能创新实验室**
>
> 本项目仅供实验室内部使用和2025年RoboCup参赛。
> 未经许可，禁止任何形式的复制、分发或商业使用。

---

**上传完成后，记得通知团队成员！** 🎉

*最后更新: 2025年10月18日*
