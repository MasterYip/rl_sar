# FDILink AHRS Standalone Interface

这是一个无需 ROS 的独立 C++ 接口，用于接收和解析 FDILink AHRS/IMU 数据。

## 特性

- ✅ 无 ROS 依赖，纯 C++ 实现
- ✅ 支持 IMU 和 AHRS 数据帧解析
- ✅ CRC8/CRC16 数据校验
- ✅ 回调接口，实时获取传感器数据
- ✅ 多线程串口读取

## 目录结构

```
thirdparty/
├── include/
│   ├── ahrs_interface.h         # 主接口头文件
│   ├── fdilink_data_struct.h    # 数据结构定义
│   └── crc_table.h              # CRC 校验
├── src/
│   ├── ahrs_interface.cpp       # 接口实现
│   └── crc_table.cpp            # CRC 实现
├── examples/
│   └── print_imu.cpp            # 示例程序
├── CMakeLists.txt               # CMake 构建文件
└── README.md                    # 本文件
```

## 依赖项

1. **C++11** 或更高版本
2. **wjwwood/serial** 串口库
   - GitHub: https://github.com/wjwwood/serial
   - 可以通过以下方式安装：
     ```bash
     git clone https://github.com/wjwwood/serial.git
     cd serial
     mkdir build && cd build
     cmake ..
     make
     sudo make install
     ```

## 编译

### Linux/macOS

```bash
cd thirdparty
mkdir build && cd build
cmake ..
make
```

### Windows

使用 Visual Studio 或 MinGW：

```bash
cd thirdparty
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019"  # 或其他生成器
cmake --build .
```

## 使用示例

### 1. 基本用法

```cpp
#include "ahrs_interface.h"
#include <iostream>

int main() {
    // 创建接口实例
    FDILink::AHRSInterface ahrs("/dev/ttyUSB0", 115200);
    
    // 设置 IMU 数据回调
    ahrs.setImuCallback([](const FDILink::ImuData &data) {
        std::cout << "Quaternion: " << data.qw << ", " 
                  << data.qx << ", " << data.qy << ", " << data.qz << std::endl;
        std::cout << "Angular velocity: " << data.gx << ", " 
                  << data.gy << ", " << data.gz << " rad/s" << std::endl;
        std::cout << "Acceleration: " << data.ax << ", " 
                  << data.ay << ", " << data.az << " m/s²" << std::endl;
    });
    
    // 启动接收
    if (!ahrs.start()) {
        std::cerr << "Failed to start AHRS interface" << std::endl;
        return 1;
    }
    
    // 保持运行
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
```

### 2. 运行示例程序

```bash
# 默认端口和波特率 (/dev/ttyTHS1, 115200)
./build/print_imu

# 自定义端口和波特率
./build/print_imu /dev/ttyUSB0 115200
```

## API 说明

### `FDILink::ImuData`

IMU 数据结构：

```cpp
struct ImuData {
    double qw, qx, qy, qz;  // 四元数姿态
    float gx, gy, gz;        // 角速度 (rad/s)
    float ax, ay, az;        // 加速度 (m/s²)
};
```

### `FDILink::AHRSInterface`

主接口类：

```cpp
class AHRSInterface {
public:
    // 构造函数：指定串口和波特率
    AHRSInterface(const std::string &port = "/dev/ttyTHS1", 
                  int baud = 115200, 
                  int timeout_ms = 20);
    
    // 启动数据接收
    bool start();
    
    // 停止数据接收
    void stop();
    
    // 设置 IMU 数据回调函数
    void setImuCallback(ImuCallback cb);
};
```

## 集成到你的项目

### 方式 1: 作为静态库

在你的 `CMakeLists.txt` 中：

```cmake
add_subdirectory(path/to/thirdparty)
target_link_libraries(your_target fdilink_ahrs)
```

### 方式 2: 直接包含源文件

```cmake
include_directories(path/to/thirdparty/include)
add_executable(your_target
    your_source.cpp
    path/to/thirdparty/src/ahrs_interface.cpp
    path/to/thirdparty/src/crc_table.cpp
)
target_link_libraries(your_target serial pthread)
```

## 故障排除

### 串口权限问题 (Linux)

```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

或临时授权：

```bash
sudo chmod 666 /dev/ttyUSB0
```

### 找不到 serial 库

确保已安装 wjwwood/serial 库并且在系统路径中。

如果使用自定义路径，修改 `CMakeLists.txt`：

```cmake
set(SERIAL_LIB "/path/to/libserial.a")
```

## 许可证

本项目基于原 ROS 驱动改造，保留原有功能并去除 ROS 依赖。

## 联系方式

如有问题或建议，请提交 Issue。
