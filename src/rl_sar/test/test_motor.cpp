#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <fstream>
#include <map>
#include <yaml-cpp/yaml.h>

extern "C"
{
#include "../library/thirdparty/el4090_motor_sdk/motor_control.h"
#include "../library/thirdparty/el4090_motor_sdk/config.h"
}

#include "../library/thirdparty/el4090_motor_sdk/transmit.h"
#include "../library/thirdparty/el4090_motor_sdk/motor_data.h"

extern MotorData motorData;
extern EtherCATThreadManager ethercatManager;

// 电机 ID 映射（根据 rl_real_el4090.cpp）
int motor_id_map[3][6] = {
    {7, 8, 9, 1, 2, 3},       // Slave 0
    {13, 17, 18, 16, 14, 15}, // Slave 1
    {4, 5, 6, 10, 11, 12}     // Slave 2
};

// 加载电机标定偏移量
std::map<int, float> LoadMotorCalibration(const std::string &yaml_path)
{
    std::map<int, float> calibration_offsets;

    try
    {
        YAML::Node config = YAML::LoadFile(yaml_path);

        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
        {
            int motor_id = static_cast<int>(it->first.as<float>());
            float offset = it->second.as<float>();
            calibration_offsets[motor_id] = offset;
            std::cout << "  Motor " << motor_id << ": offset = " << offset << " rad" << std::endl;
        }

        std::cout << "Loaded calibration for " << calibration_offsets.size() << " motors" << std::endl;
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Error loading calibration file: " << e.what() << std::endl;
    }

    return calibration_offsets;
}

int main()
{
    std::cout << "=== EL4090 Motor Movement to Calibration Positions ===" << std::endl;

    // 初始化 EtherCAT
    std::cout << "Initializing EtherCAT on enp86s0..." << std::endl;
    EtherCAT_Init((char *)"enp86s0");

    if (ec_slavecount <= 0)
    {
        std::cerr << "Error: No EtherCAT slaves found!" << std::endl;
        return 1;
    }
    std::cout << "Found " << ec_slavecount << " EtherCAT slave(s)" << std::endl;

    // 启动通信线程
    std::cout << "Starting communication threads..." << std::endl;
    ethercatManager.startThreads();

    // 等待通信稳定
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Communication stabilized" << std::endl;

    // 加载电机标定偏移量
    std::cout << "\n=== Loading Motor Calibration ===" << std::endl;
    std::string calib_path = "policy/el_4090/motor_calibration.yaml";
    std::cout << "Loading from: " << calib_path << std::endl;
    std::map<int, float> calibration_offsets = LoadMotorCalibration(calib_path);

    if (calibration_offsets.empty())
    {
        std::cerr << "Error: Failed to load calibration file!" << std::endl;
        ethercatManager.stopThreads();
        return 1;
    }

    // 先发送阻尼模式，让电机准备好
    std::cout << "\n=== Setting damping mode (kp=0, kd=3) ===" << std::endl;
    for (int slave = 0; slave < 3; ++slave)
    {
        for (int passage = 1; passage <= 6; ++passage)
        {
            int motor_id = motor_id_map[slave][passage - 1];

            EtherCAT_Msg tx_msg = motorData.getTxMsg(slave);
            send_motor_ctrl_cmd(&tx_msg, passage, motor_id,
                                0.0f,  // kp = 0
                                3.0f,  // kd = 3 (轻阻尼)
                                0.0f,  // pos
                                0.0f,  // spd
                                0.0f); // tor
            motorData.setTxMsg(slave, tx_msg);
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Damping mode set" << std::endl;

    // 发送命令让电机移动到标定位置
    std::cout << "\n=== Moving motors to calibration positions ===" << std::endl;
    std::cout << "Target positions (from motor_calibration.yaml):" << std::endl;

    for (int slave = 0; slave < 3; ++slave)
    {
        for (int passage = 1; passage <= 6; ++passage)
        {
            int motor_id = motor_id_map[slave][passage - 1];

            if (calibration_offsets.find(motor_id) != calibration_offsets.end())
            {
                float target_position = calibration_offsets[motor_id];

                std::cout << "  Motor " << motor_id << " -> " << std::fixed
                          << std::setprecision(4) << target_position << " rad" << std::endl;

                // 使用位置控制模式
                EtherCAT_Msg tx_msg = motorData.getTxMsg(slave);
                send_motor_ctrl_cmd(&tx_msg, passage, motor_id,
                                    8.0f,           // kp
                                    0.8f,            // kd
                                    target_position, // 目标位置
                                    0.2f,            // spd
                                    0.0f);           // tor
                motorData.setTxMsg(slave, tx_msg);
            }
            else
            {
                std::cout << "  Motor " << motor_id << " -> NO CALIBRATION DATA" << std::endl;
            }
        }
    }

    // 实时监控电机角度
    std::cout << "\n=== Real-time Motor Angle Monitoring (30 seconds) ===" << std::endl;
    std::cout << "Press Ctrl+C to stop early" << std::endl;

    for (int i = 0; i < 60; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // 每秒打印一次
        if (i % 2 == 0)
        {
            std::cout << "\n[" << (i / 2 + 1) << "s] Current motor angles (angle_actual_rad):" << std::endl;

            for (int slave = 0; slave < 3; ++slave)
            {
                for (int passage = 1; passage <= 6; ++passage)
                {
                    int motor_id = motor_id_map[slave][passage - 1];
                    OD_Motor_Msg motor_msg = motorData.getRxMotorMsg(slave, passage);

                    if (motor_msg.motor_id == motor_id)
                    {
                        float current_angle = motor_msg.angle_actual_rad;
                        float target_angle = calibration_offsets[motor_id];
                        float error = target_angle - current_angle;

                        std::cout << "  M" << std::setw(2) << motor_id << ": "
                                  << std::fixed << std::setprecision(4)
                                  << "actual=" << std::setw(8) << current_angle << " rad, "
                                  << "target=" << std::setw(8) << target_angle << " rad, "
                                  << "error=" << std::setw(7) << error << " rad";

                        if ((slave * 6 + passage) % 2 == 0)
                            std::cout << std::endl;
                    }
                }
            }
        }
    }

    std::cout << "\n=== Stopping communication ===" << std::endl;
    ethercatManager.stopThreads();

    std::cout << "Test completed successfully!" << std::endl;
    return 0;
}
