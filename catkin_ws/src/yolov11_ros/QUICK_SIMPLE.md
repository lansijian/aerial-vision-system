# 极简追踪 - 快速参考

## 🎯 唯一目标
**保持YOLO检测框在图像中心**

---

## 📐 核心算法（仅2行）

```python
yaw_rate = -Kp_yaw * u_offset        # 偏航控制
vx = Kp_distance * (h_ideal - h) / h_ideal  # 距离控制
```

**就这么简单！**

---

## 🔧 关键参数

```python
Kp_yaw = 0.002          # 偏航增益
Kp_distance = 1.5       # 距离增益
ideal_box_height = 160  # 理想框高(px)
max_yaw_rate = 0.5      # 最大偏航率
max_vx = 3.0           # 最大前后速度
```

---

## 📝 日志示例

### 良好状态
```
[✅居中] u= +12px h=165px | vx=+0.05 yaw=-0.024rad/s
```

### 需要调整
```
[⚠️偏离] u=-200px h=120px | vx=+0.50 yaw=+0.400rad/s
```

---

## 🎛️ 快速调优

### 偏航太慢？
```python
Kp_yaw = 0.003  # 提高（原0.002）
```

### 偏航太快？
```python
Kp_yaw = 0.001  # 降低（原0.002）
```

---

## ✨ 极简特性
- ✅ 核心算法仅40行
- ✅ 只有2个主要参数
- ✅ 无分级、无平滑、无复杂逻辑
- ✅ 参考XTDrone验证思路

---

**版本**：v10.1.2-simple  
**理念**：Simple is Beautiful!  
**详细文档**：[SIMPLE_TRACKING.md](SIMPLE_TRACKING.md)

