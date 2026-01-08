# EL4090 电机标定指南

## 标定的目的

建立 **URDF关节角度** 和 **电机物理位置** 之间的映射关系。

## 核心概念

### 1. 坐标系统
- **电机物理位置**: 电机反馈的原始读数（弧度），这是电机的绝对位置
- **URDF关节角度**: 在 URDF 模型中定义的关节角度（弧度）
- **标定偏移**: URDF零位时对应的电机物理位置

### 2. 转换关系

```
URDF关节角度 = 电机物理位置 - 标定偏移
电机目标位置 = URDF目标角度 + 标定偏移
```

### 3. 示例说明

假设电机1：
- URDF零位时，电机物理读数为 **0.523 弧度**
- 标定偏移 = 0.523

**读取状态时**：
```
电机反馈: 1.047 rad
URDF角度 = 1.047 - 0.523 = 0.524 rad
```

**发送命令时**：
```
URDF目标: 0.5 rad
电机命令 = 0.5 + 0.523 = 1.023 rad
```

## 标定步骤

### 方法一：使用 Python 脚本（推荐）

如果你有完整的 Python 环境：

```bash
cd /home/zht/motor_communication_code

# 1. 编辑 motor_calibration.py，修改标定姿态
# 将机器人调整到 URDF 零位姿态，而不是对齐姿态

# 2. 运行标定程序
python motor_calibration.py

# 3. 按照提示操作，将机器人调整到 URDF 零位
# 程序会记录每个电机的位置

# 4. 复制生成的标定文件
cp motor_calibration.yaml /home/zht/rl_sar/policy/el_4090/
```

### 方法二：手动标定

#### 步骤 1: 准备工作

1. 确保机器人可以安全移动（建议吊起）
2. 确认你知道 URDF 中的零位姿态是什么样的
3. 准备记录工具（文本文件或笔纸）

#### 步骤 2: 查看 URDF 零位

检查你的 URDF 文件或 base.yaml：

```yaml
# policy/el_4090/base.yaml
default_dof_pos: [0.0, 0.0, 0.0,    # 这些是 URDF 的默认姿态
                  0.0, 0.0, 0.0,    # 可能不是零位
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0]
```

**URDF 零位通常是**：所有关节角度都为 0 的姿态。

#### 步骤 3: 调整机器人到 URDF 零位

1. 手动或使用低增益控制将机器人移动到 URDF 零位姿态
2. 每个关节都应该在其 URDF 定义的 0 度位置

#### 步骤 4: 读取电机位置

编写一个简单的读取程序，或使用现有工具：

```cpp
// 在 rl_real_el4090.cpp 中临时添加打印代码
void RL_Real::HardwareRecv()
{
    for (int i = 0; i < num_dofs; ++i)
    {
        int motor_id = i + 1;
        auto it = motor_location_map_.find(motor_id);
        if (it != motor_location_map_.end()) {
            int slave = it->second.slave;
            int passage = it->second.passage;
            
            float position, velocity, torque;
            ReadMotorStatus(slave, passage, motor_id, position, velocity, torque);
            
            // 临时打印：显示原始电机位置
            static int print_counter = 0;
            if (print_counter++ % 200 == 0) {  // 每秒打印一次
                std::cout << "Motor " << motor_id << ": " << position << " rad" << std::endl;
            }
            
            // ... 其余代码
        }
    }
}
```

#### 步骤 5: 记录并填写标定文件

将读取到的每个电机位置填入 `motor_calibration.yaml`：

```yaml
# 示例：你测量到的值
1: 0.523
2: -0.314
3: 1.047
4: 0.0
5: 0.261
6: -0.785
# ... 继续填写其余电机
```

#### 步骤 6: 验证标定

1. 重新编译并运行程序
2. 让机器人站立到默认姿态（按 `0`）
3. 检查机器人姿态是否与 URDF 模型一致
4. 如果不一致，重新检查标定值

### 方法三：使用被动模式读取

最简单的方法：

```bash
# 1. 编译并运行程序
./cmake_build/bin/rl_real_el4090 enp3s0

# 2. 按 'P' 进入被动模式（电机阻尼）

# 3. 手动将机器人调整到 URDF 零位姿态

# 4. 查看程序输出的电机位置（需要添加打印代码）

# 5. 记录位置并填入 motor_calibration.yaml

# 6. 重启程序测试
```

## 验证标定是否正确

### 测试 1: 站立姿态

```bash
# 运行程序
./cmake_build/bin/rl_real_el4090 enp3s0

# 按 '0' 站立到默认姿态
# 检查机器人姿态是否与预期一致
```

### 测试 2: 比较 URDF 和实际

如果你有可视化工具（如 RViz）：

1. 在 RViz 中加载 URDF 模型
2. 发布当前关节状态
3. 比较实际机器人和 URDF 模型的姿态
4. 如果不匹配，调整标定值

### 测试 3: 小幅度运动

```bash
# 从默认姿态，给一个小的运动命令
# 观察机器人是否按预期方向移动
# 如果方向错误，可能是 joint_mapping 问题，不是标定问题
```

## 常见问题

### Q1: 标定值是弧度还是度？
**A**: **弧度（radians）**。所有值都必须是弧度。

### Q2: 如果我的 URDF 零位不是全 0 怎么办？
**A**: 没关系。标定偏移定义的是"当URDF关节角度为其零位值时，电机的物理位置"。

例如，如果 URDF 中某关节零位是 0.5 rad：
- 将机器人该关节调整到对应 0.5 rad 的物理姿态
- 读取电机位置（例如 1.023 rad）
- 标定偏移 = 1.023 - 0.5 = 0.523 rad

### Q3: 标定后机器人姿态还是不对？
**A**: 检查以下几点：
1. **joint_mapping** 是否正确？
2. 标定时机器人是否真的在 URDF 零位？
3. 电机ID映射是否正确？
4. 方向是否正确？（可能需要负号）

### Q4: 可以用度数吗？
**A**: 不可以。程序内部全部使用弧度，YAML 文件也必须用弧度。

转换公式：
```
弧度 = 度数 × π / 180
度数 = 弧度 × 180 / π
```

### Q5: 每次重新标定都要重启程序吗？
**A**: 是的。标定文件在程序启动时加载。修改后需要重启程序。

## 调试技巧

### 1. 添加调试输出

在 `HardwareRecv()` 中添加：

```cpp
// 打印原始电机位置和转换后的 URDF 角度
std::cout << "Motor " << motor_id 
          << " Raw: " << position 
          << " Offset: " << offset 
          << " URDF: " << (position - offset) 
          << std::endl;
```

### 2. 对比期望值

记录几个关键姿态：
- URDF 零位
- 默认站立姿态
- 特定测试姿态

对比实际电机位置和期望值。

### 3. 单独测试每个关节

逐个关节测试：
1. 其他关节保持不动
2. 单独移动一个关节
3. 验证运动方向和幅度

## 标定文件示例

### 示例 1: 理想情况（URDF零位 = 电机零位）

```yaml
1: 0.0
2: 0.0
3: 0.0
# ... 所有都是 0
```

### 示例 2: 实际情况（有偏移）

```yaml
# 实际测量的值
1: 0.523    # 电机1在URDF零位时读数为 0.523 rad
2: -0.314   # 电机2在URDF零位时读数为 -0.314 rad
3: 1.047    # 电机3在URDF零位时读数为 1.047 rad
4: 0.0      # 电机4正好对齐
5: 0.261
6: -0.785
7: 0.123
8: -0.456
9: 0.789
10: 0.234
11: -0.567
12: 0.891
13: 0.345
14: -0.678
15: 0.912
16: 0.456
17: -0.789
18: 1.023
```

## 备份和版本控制

建议：
1. 备份每次标定的结果
2. 在文件中添加日期和注释
3. 使用 git 管理标定文件

```bash
# 备份
cp motor_calibration.yaml motor_calibration_backup_$(date +%Y%m%d).yaml

# 提交到 git
git add policy/el_4090/motor_calibration.yaml
git commit -m "Update motor calibration: $(date)"
```

## 自动化标定脚本（可选）

可以创建一个简单的脚本来辅助标定：

```python
#!/usr/bin/env python3
import yaml

# 手动输入测量值
motor_positions = {
    1: 0.523,
    2: -0.314,
    # ... 继续添加
}

# 保存到 YAML
with open('motor_calibration.yaml', 'w') as f:
    yaml.dump(motor_positions, f, default_flow_style=False)

print("Calibration file saved!")
```

---

**记住**：标定是实物部署的关键步骤，值得花时间做好！
