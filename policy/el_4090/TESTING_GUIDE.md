# EtherCAT 通信测试指南

## 📝 测试程序说明

测试程序: `test_ethercat_communication`

**位置**: `/home/zht/rl_sar/cmake_build/bin/test_ethercat_communication`

**用途**: 分步测试 EtherCAT 通信功能，验证硬件连接和电机通信是否正常

---

## 🚀 运行测试

### 1. 准备工作

确保：
- [ ] 机器人电源已打开
- [ ] EtherCAT 网线已连接
- [ ] 知道网络接口名称（使用 `ip link` 查看）
- [ ] 有 sudo 权限

### 2. 查看网络接口

```bash
ip link
# 查找 EtherCAT 网口，例如：enp3s0, eth0, eno1 等
```

### 3. 运行测试程序

```bash
cd /home/zht/rl_sar/cmake_build/bin
sudo ./test_ethercat_communication enp3s0
```

**参数**:
- `enp3s0`: 你的 EtherCAT 网络接口名称

---

## 📊 测试项目

程序会依次执行以下测试：

### ✅ Test 1: EtherCAT 初始化
**测试内容**:
- 初始化 EtherCAT master
- 检测 EtherCAT slaves 数量
- 显示每个 slave 的信息

**预期结果**:
```
[PASS] Found 3 EtherCAT slave(s)
[Info] Slave 1:
       Name: ...
       State: 0x8
       Input bytes: 98
       Output bytes: 98
...
```

**失败排查**:
- 检查网络接口名称是否正确
- 检查 EtherCAT 设备是否上电
- 检查网线是否连接
- 确认有 root 权限

---

### ✅ Test 2: 通信线程启动
**测试内容**:
- 启动 EtherCAT 发送线程
- 启动 EtherCAT 接收线程
- 等待通信稳定

**预期结果**:
```
[PASS] Communication threads started successfully
[Info] Waiting for communication to stabilize (2 seconds)...
```

---

### ✅ Test 3: 读取电机状态（被动）
**测试内容**:
- 从所有 18 个电机读取状态
- 显示位置、速度、温度
- 检查电机 ID 是否匹配

**预期结果**:
```
[Info] Slave 0:
------------------------------------------------------------
Passage   Motor ID  Position(rad)  Velocity       Temp(°C)
------------------------------------------------------------
1         7         0.1234         0.0000         35
2         8         -0.5678        0.0000         36
...
[Info] Total motors with valid data: 18 / 18
[PASS] Most motors responding (>= 15/18)
```

**状态说明**:
- `[NO DATA]`: 电机未响应
- `[ID MISMATCH]`: 电机 ID 不匹配，可能接线错误
- 正常数据：显示实际的位置、速度、温度

**失败排查**:
- 如果 `motors_found < 15`：
  - 检查电机电源
  - 检查电机 ID 配置
  - 检查 `InitMotorMapping()` 中的映射

---

### ✅ Test 4: 发送阻尼命令
**测试内容**:
- 向所有电机发送阻尼模式命令
- kp=0 (无位置控制), kd=3 (轻阻尼)

**预期结果**:
```
[PASS] Damping commands sent successfully
[Info] You can now manually move the robot joints (should feel light damping)
```

**验证方法**:
- 手动移动机器人关节
- 应该感觉到轻微的阻尼
- 不应该有强烈的位置保持力

---

### ✅ Test 5: 持续监控（可选）
**测试内容**:
- 每秒打印一次所有电机位置
- 持续 10 秒
- 可手动移动关节观察变化

**预期结果**:
```
[0s] Current motor positions:
  Motor  1:   0.1234 rad,   0.0000 rad/s  Motor  2:  -0.5678 rad,   0.0000 rad/s
  ...
[1s] Current motor positions:
  Motor  1:   0.1250 rad,   0.0150 rad/s  Motor  2:  -0.5660 rad,   0.0120 rad/s
  ...
```

**操作**:
- 手动移动某个关节
- 观察对应电机的位置是否变化

---

## 📋 测试结果判断

### ✅ 全部通过
```
[SUCCESS] All tests passed! ✓

Next steps:
  1. Run motor calibration to get motor_calibration.yaml
  2. Test with rl_real_el4090 in suspended mode
  3. Deploy RL policy
```

**下一步**:
1. 运行电机标定程序
2. 在吊起状态测试 RL 程序
3. 部署 RL 策略

---

### ⚠️ 部分失败
```
[WARNING] Some tests failed or had warnings

Troubleshooting:
  - Check motor power supply
  - Verify motor IDs are configured correctly
  - Check EtherCAT network cables
  - Review motor_id_map in InitMotorMapping()
```

**排查步骤**:
1. **Test 1 失败**: EtherCAT 连接问题
   - 检查网络接口名称
   - 检查物理连接
   - 检查设备上电

2. **Test 3 失败**: 电机通信问题
   - 检查电机 ID 映射
   - 检查电机电源
   - 检查 CAN 通信

3. **Test 4/5 失败**: 命令发送问题
   - 检查电机模式
   - 检查控制权限

---

## 🔧 常见问题

### Q1: `No EtherCAT slaves found`
**原因**:
- 网络接口名称错误
- 设备未上电
- 网线未连接

**解决**:
```bash
# 检查网络接口
ip link

# 确认设备连接
sudo ethercat slaves  # 如果安装了 ethercat 工具
```

---

### Q2: `Some motors not responding`
**原因**:
- 电机电源问题
- 电机 ID 配置错误
- CAN 总线问题

**解决**:
1. 检查电机 ID 映射:
   ```cpp
   // rl_real_el4090.cpp: InitMotorMapping()
   motor_id_map_[0] = {7, 8, 9, 1, 2, 3};
   motor_id_map_[1] = {13, 17, 18, 16, 14, 15};
   motor_id_map_[2] = {4, 5, 6, 10, 11, 12};
   ```

2. 使用 Python 标定程序验证电机 ID

---

### Q3: `ID MISMATCH`
**原因**:
- 电机 ID 映射配置错误
- 电机 ID 设置错误

**解决**:
1. 记录实际收到的 motor_id
2. 修改 `InitMotorMapping()` 中的映射
3. 或者使用电机配置工具重设电机 ID

---

### Q4: 手动移动关节没有阻尼感
**原因**:
- 命令未生效
- 电机处于错误模式

**解决**:
- 检查 Test 4 是否显示 `[PASS]`
- 尝试重启测试程序
- 检查电机是否处于错误状态

---

## 💡 调试技巧

### 1. 查看详细输出
程序会打印详细的测试信息，包括：
- EtherCAT slave 信息
- 每个电机的实时数据
- 错误和警告信息

### 2. 分段测试
如果某个测试失败，可以注释后续测试，专注解决当前问题。

### 3. 对比 Python 程序
如果 Python 标定程序工作正常，但 C++ 测试失败：
- 对比电机 ID 映射
- 对比网络接口配置
- 对比控制参数

---

## 📝 测试日志

建议记录测试结果：

```bash
# 保存测试日志
sudo ./test_ethercat_communication enp3s0 | tee test_log_$(date +%Y%m%d_%H%M%S).txt
```

---

## ⚡ 快速测试流程

```bash
# 1. 进入目录
cd /home/zht/rl_sar/cmake_build/bin

# 2. 运行测试
sudo ./test_ethercat_communication enp3s0

# 3. 按提示操作
#    - 按 Enter 开始测试
#    - 观察测试结果
#    - 选择是否运行持续监控

# 4. 如果全部通过，继续标定
cd /home/zht/motor_communication_code
python motor_calibration.py
```

---

## 🎯 成功标准

测试成功的标志：
- ✅ 检测到 3 个 EtherCAT slaves
- ✅ 至少 15/18 个电机响应
- ✅ 能够发送阻尼命令
- ✅ 手动移动关节有轻微阻尼感
- ✅ 监控时能看到位置变化

如果达到以上标准，可以进行下一步标定和部署！

---

**测试程序源码**: `/home/zht/rl_sar/src/rl_sar/test/test_ethercat_communication.cpp`
