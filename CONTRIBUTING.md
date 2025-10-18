# 贡献指南

感谢您对RoboCup 2025多无人机协同追踪系统的关注！

## ⚠️ 重要说明

本项目为**东华大学人工智能创新实验室**内部项目，**仅限实验室成员贡献**。

## 👥 适用对象

- ✅ 东华大学人工智能创新实验室成员
- ✅ RoboCup 2025参赛团队成员
- ✅ 经实验室授权的合作者

## 📝 贡献准则

### 1. 开始之前

在开始贡献前，请确保：

1. ✅ 您已是实验室正式成员
2. ✅ 已阅读并理解项目文档
3. ✅ 已配置好开发环境
4. ✅ 了解项目的总体架构

### 2. 开发流程

#### 步骤1: 创建分支

```bash
# 更新主分支
git checkout main
git pull origin main

# 创建功能分支
git checkout -b feature/your-feature-name

# 或修复分支
git checkout -b fix/bug-description
```

**分支命名规范**:
- `feature/xxx` - 新功能
- `fix/xxx` - Bug修复
- `optimize/xxx` - 性能优化
- `docs/xxx` - 文档更新
- `refactor/xxx` - 代码重构

#### 步骤2: 进行开发

```bash
# 进行代码修改
# ...

# 测试你的修改
catkin build
roslaunch yolov11_ros multi_drone_system.launch
```

#### 步骤3: 提交代码

```bash
# 添加修改
git add .

# 提交（使用规范的提交信息）
git commit -m "feat: 添加新功能XXX"
```

#### 步骤4: 推送并请求审查

```bash
# 推送到远程
git push origin feature/your-feature-name

# 在GitHub/GitLab上创建Pull Request/Merge Request
```

#### 步骤5: 代码审查

- 至少需要1名团队成员审查
- 通过所有自动测试
- 解决所有审查意见
- 获得批准后才能合并

## 📋 代码规范

### Python代码规范

遵循PEP 8规范：

```python
# 好的例子
class DroneController:
    """无人机控制器类"""
    
    def __init__(self, drone_id):
        """初始化控制器
        
        Args:
            drone_id: 无人机ID
        """
        self.drone_id = drone_id
        self.state = "IDLE"
    
    def takeoff(self, altitude):
        """起飞到指定高度
        
        Args:
            altitude: 目标高度（米）
        """
        rospy.loginfo(f"[Drone {self.drone_id}] Taking off to {altitude}m")
        # 实现代码...
```

**关键要点**:
- 使用4空格缩进
- 类名使用大驼峰 (CamelCase)
- 函数名使用小写+下划线 (snake_case)
- 添加文档字符串
- 有意义的变量名

### C++代码规范

```cpp
// 使用Google C++规范
class DroneController {
 public:
  DroneController(int drone_id);
  ~DroneController();
  
  void Takeoff(double altitude);
  
 private:
  int drone_id_;
  std::string state_;
};
```

### ROS代码规范

```python
# 话题命名
/namespace/node_name/topic_name

# 参数命名
~private_param
global_param

# 服务命名
/namespace/service_name
```

## 🧪 测试要求

### 单元测试

为新功能添加测试：

```python
# test/test_drone_controller.py
import unittest
from drone_controller import DroneController

class TestDroneController(unittest.TestCase):
    def test_takeoff(self):
        controller = DroneController(0)
        result = controller.takeoff(3.0)
        self.assertTrue(result)
```

### 集成测试

```bash
# 运行所有测试
catkin run_tests

# 运行特定包的测试
catkin run_tests yolov11_ros
```

### 仿真测试

每次修改后必须测试：

1. 启动完整系统
2. 验证所有无人机正常起飞
3. 检查检测和追踪功能
4. 确认协同机制工作
5. 观察至少5分钟无异常

## 📝 提交信息规范

使用约定式提交（Conventional Commits）：

### 格式

```
<类型>(<范围>): <简短描述>

<详细描述>

<页脚>
```

### 类型

- `feat`: 新功能
- `fix`: Bug修复
- `docs`: 文档更新
- `style`: 代码格式（不影响功能）
- `refactor`: 重构
- `perf`: 性能优化
- `test`: 测试相关
- `chore`: 构建/工具相关

### 示例

```bash
# 简单提交
git commit -m "feat: 添加紧急后退功能"

# 详细提交
git commit -m "fix: 修复无人机解锁失败问题

修复了MAVROS端口配置错误导致的解锁失败。
将fcu_url端口从14540改为24540+drone_id。

Closes #123"
```

## 📚 文档要求

### 代码文档

- 所有公共函数必须有文档字符串
- 复杂算法需要注释说明
- 重要参数需要说明单位和范围

### 用户文档

添加新功能时需要更新：

- `README.md` - 如果影响使用方式
- `TECHNICAL.md` - 技术实现细节
- `docs/` - 相关专题文档

### 变更日志

在 `CHANGELOG.md` 中记录：

```markdown
## [v10.3.0] - 2025-10-18

### 新增
- 添加紧急后退功能

### 修复
- 修复解锁失败问题

### 优化
- 提高检测速度
```

## 🔍 代码审查清单

审查者需要检查：

### 功能性
- [ ] 代码实现了预期功能
- [ ] 没有引入新的Bug
- [ ] 边界情况已处理

### 代码质量
- [ ] 遵循代码规范
- [ ] 命名清晰易懂
- [ ] 没有重复代码
- [ ] 适当的错误处理

### 测试
- [ ] 包含必要的测试
- [ ] 所有测试通过
- [ ] 仿真验证通过

### 文档
- [ ] 代码有适当注释
- [ ] 更新了相关文档
- [ ] 提交信息清晰

## 🚫 不允许的操作

以下操作**严格禁止**：

1. ❌ 直接推送到 `main` 分支
2. ❌ 提交未测试的代码
3. ❌ 删除他人代码而不沟通
4. ❌ 提交包含密码/密钥的代码
5. ❌ 提交大型二进制文件（>10MB）
6. ❌ 修改版权声明
7. ❌ 未经许可分享代码

## 🎯 优先级指南

### 高优先级

- 🔴 影响比赛的Bug
- 🔴 系统崩溃问题
- 🔴 性能严重下降

### 中优先级

- 🟡 功能增强
- 🟡 代码优化
- 🟡 文档完善

### 低优先级

- 🟢 代码美化
- 🟢 注释补充
- 🟢 非关键优化

## 📞 沟通渠道

### 遇到问题？

1. 🔍 先查看文档和已知问题
2. 💬 在团队群里讨论
3. 📧 联系项目负责人
4. 🐛 创建Issue（内部系统）

### 建议新功能？

1. 📝 写下详细的需求说明
2. 💭 在团队会议上讨论
3. ✅ 获得批准后开始开发

## 🏆 贡献者

感谢以下团队成员的贡献：

- [请团队成员添加姓名]

## 📖 参考资源

- [PEP 8 Python代码规范](https://pep8.org/)
- [Google C++规范](https://google.github.io/styleguide/cppguide.html)
- [约定式提交](https://www.conventionalcommits.org/zh-hans/)
- [ROS代码规范](http://wiki.ros.org/CppStyleGuide)

## 📄 许可证

贡献的代码将遵循项目许可证：

**东华大学人工智能创新实验室专有许可证**

详见: [LICENSE](LICENSE)

---

**再次感谢您的贡献！让我们一起打造优秀的系统！** 🚁
