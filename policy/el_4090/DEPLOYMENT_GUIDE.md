# EL4090 实物部署指南 (使用 EtherCAT 电机控制)

## 系统架构

EL4090 机器人使用 **EtherCAT** 通信协议控制 18 个电机，分布在 3 个从站上：

### 硬件拓扑
```
主机 (EtherCAT Master)
  └── 网口 (如 enp3s0)
       ├── 从站 0: 电机 ID [7, 8, 9, 1, 2, 3]
       ├── 从站 1: 电机 ID [13, 17, 18, 16, 14, 15]
       └── 从站 2: 电机 ID [4, 5, 6, 10, 11, 12]
```

每个从站有 6 个 CAN 通道 (passage 1-6)，每个通道连接一个电机。

## 前置条件

### 1. 网络配置
确保你的网口已正确配置，并且可以检测到 EtherCAT 从站：

```bash
# 查看网卡名称
ip addr

# 测试 EtherCAT 通信（需要先安装 SOEM）
# 你的网卡名称可能是 enp3s0, eth0, ens33 等
sudo ethtool enp3s0  # 检查网卡状态
```

### 2. 权限设置
EtherCAT 需要 root 权限或网络 CAP_NET_RAW 权限：

```bash
# 方法 1: 使用 sudo 运行
sudo ./cmake_build/bin/rl_real_el4090 <network_interface>

# 方法 2: 设置 capabilities (推荐)
sudo setcap cap_net_raw+ep ./cmake_build/bin/rl_real_el4090
./cmake_build/bin/rl_real_el4090 <network_interface>
```

### 3. 电机标定
在首次使用前，需要标定电机零位：

```bash
# 如果有 Python 环境和 pyethercat 模块
cd /home/zht/motor_communication_code
python motor_calibration.py

# 按照提示逐个标定每个电机
# 标定完成后，将生成的 motor_calibration.yaml 复制到策略目录
cp motor_calibration.yaml /home/zht/rl_sar/policy/el_4090/
```

## 部署步骤

### 1. 编译项目
```bash
cd /home/zht/rl_sar
./build.sh -m
```

编译成功后，可执行文件位于：`cmake_build/bin/rl_real_el4090`

### 2. 安全准备
⚠️ **首次测试必须吊起机器人！**

- [ ] 机器人已吊起，6 条腿不接触地面
- [ ] 确认周围无障碍物和人员
- [ ] 准备好急停按钮或断电开关
- [ ] 检查电机温度正常
- [ ] 检查所有电机连接正常

### 3. 启动控制程序
```bash
# 方法 1: 使用 sudo
sudo ./cmake_build/bin/rl_real_el4090 enp3s0

# 方法 2: 使用 capabilities
sudo setcap cap_net_raw+ep ./cmake_build/bin/rl_real_el4090
./cmake_build/bin/rl_real_el4090 enp3s0
```

**注意**：将 `enp3s0` 替换为你实际的网卡名称

### 4. 控制操作

程序启动后，使用键盘控制：

| 按键 | 功能 | 说明 |
|------|------|------|
| **P** | 被动模式 | 电机阻尼模式 (kp=0, kd=8) |
| **0** | 站立 | 从当前位置插值到默认姿态 |
| **1** | RL 控制 | 启动强化学习控制（Locomotion） |
| **9** | 趴下 | 返回初始姿态 |
| **W/S** | 前后移动 | 控制 X 轴速度 |
| **A/D** | 左右移动 | 控制 Y 轴速度 |
| **Q/E** | 偏航旋转 | 控制 Yaw 速度 |
| **Space** | 停止 | 所有速度命令归零 |

### 5. 测试流程

**首次测试（吊起状态）**：
1. 按 **P** 进入被动模式
2. 手动移动各关节，检查是否能自由转动
3. 按 **0** 让机器人站立到默认姿态
4. 观察动作是否平滑，有无异常震荡
5. 按 **1** 启动 RL 控制
6. 使用 **WASD** 测试移动命令响应
7. 观察腿部运动是否协调

**如果一切正常**：
1. 按 **9** 让机器人趴下
2. 停止程序（Ctrl+C）
3. 缓慢放下机器人至地面
4. 重新启动程序进行地面测试

## 配置文件说明

### policy/el_4090/base.yaml
```yaml
el_4090:
  dt: 0.005           # 控制周期 (200 Hz)
  decimation: 4       # RL 策略频率 = 200/4 = 50 Hz
  num_of_dofs: 18     # 18 个自由度
  joint_names: [...]  # 必须与硬件顺序一致
  ...
```

### policy/el_4090/legged_gym/config.yaml
```yaml
el_4090/legged_gym:
  model_name: "policy_1.pt"
  num_observations: 66
  joint_mapping: [15, 16, 17, 9, 10, 11, 12, 13, 14, 6, 7, 8, 0, 1, 2, 3, 4, 5]
  # ⚠️ joint_mapping 非常重要！
  # 它将仿真训练时的关节顺序映射到实物机器人的关节顺序
  ...
```

### policy/el_4090/motor_calibration.yaml
```yaml
# 电机标定偏移量（角度）
1: 0.0
2: 0.0
3: 5.2
...
```

## 故障排查

### 问题 1: 未找到 EtherCAT 从站
```
Error: No EtherCAT slaves found!
```
**解决方法**：
- 检查网线连接
- 确认网卡名称正确
- 检查从站电源
- 尝试其他网卡：`ip addr` 查看所有网卡

### 问题 2: 权限不足
```
Error: Cannot open network device
```
**解决方法**：
- 使用 sudo 运行
- 或设置 capabilities: `sudo setcap cap_net_raw+ep ./cmake_build/bin/rl_real_el4090`

### 问题 3: 电机运动异常
**可能原因**：
1. **joint_mapping 错误** - 检查 `config.yaml` 中的映射是否正确
2. **标定偏移错误** - 重新运行标定程序
3. **PD 增益过大** - 降低 `rl_kp` 和 `rl_kd` 值
4. **力矩限制过大** - 降低 `torque_limits` 值

### 问题 4: EtherCAT 通信超时
```
Warning: EtherCAT write thread timeout
```
**解决方法**：
- 降低控制频率 (增大 `dt` 值)
- 检查网络负载
- 使用专用网卡（避免与其他应用共享）

### 问题 5: 电机过热
**解决方法**：
- 立即停止程序
- 检查是否有机械卡死
- 降低 PD 增益和力矩限制
- 增加运动平滑时间

## 进阶配置

### 调整 PD 增益
在 `policy/el_4090/legged_gym/config.yaml` 中：

```yaml
# 初始测试时使用较小的值
rl_kp: [100.0, 40.0, 40.0, ...]  # 降低 kp
rl_kd: [1.0, 0.4, 0.4, ...]      # 降低 kd

# 稳定后逐步增加
rl_kp: [150.0, 55.0, 55.0, ...]
rl_kd: [1.5, 0.6, 0.6, ...]
```

### 启用数据记录
在 `rl_real_el4090.hpp` 顶部：
```cpp
#define CSV_LOGGER  // 取消注释
```
重新编译后会记录数据到 `policy/el_4090/motor.csv`

### 启用实时绘图
```cpp
#define PLOT  // 取消注释
```
需要安装 matplotlib-cpp

## 安全警告

⚠️ **在实物测试前必读**：

1. **首次测试必须吊起机器人**
2. **确保 joint_mapping 正确**（错误映射会导致机器人失控）
3. **从小参数开始测试**（低 PD 增益、低速度）
4. **准备急停措施**（物理按钮或快速断电）
5. **监控电机温度**（避免过热）
6. **确保周围安全**（无人员和障碍物）

## 技术支持

如遇问题，请检查：
1. 编译日志：确认无警告和错误
2. 运行日志：查看 EtherCAT 通信状态
3. 电机反馈：检查位置、速度、力矩是否合理
4. 配置文件：确认所有参数正确

---

**创建日期**: 2026-01-07  
**状态**: 已完成并测试编译通过  
**EtherCAT 库**: SOEM (Simple Open EtherCAT Master)
