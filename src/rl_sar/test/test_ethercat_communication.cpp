/*
 * EtherCAT Communication Test Program
 * 测试 EtherCAT 基本通信功能
 * 
 * 编译方法:
 * cd /home/zht/rl_sar/cmake_build
 * g++ -o test_ethercat ../src/rl_sar/test/test_ethercat_communication.cpp \
 *     -I../src/rl_sar/library/thirdparty/el4090_motor_sdk \
 *     -I../src/rl_sar/library/thirdparty/el4090_motor_sdk/thirdparty/soem \
 *     -I../src/rl_sar/library/thirdparty/el4090_motor_sdk/thirdparty/soem/soem \
 *     -I../src/rl_sar/library/thirdparty/el4090_motor_sdk/thirdparty/soem/osal \
 *     -I../src/rl_sar/library/thirdparty/el4090_motor_sdk/thirdparty/soem/osal/linux \
 *     -I../src/rl_sar/library/thirdparty/el4090_motor_sdk/thirdparty/soem/oshw/linux \
 *     -L./lib -lel4090_motor_control -lpthread -lrt -lyaml-cpp
 * 
 * 运行方法:
 * sudo ./test_ethercat enp3s0
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <unistd.h>  // for geteuid()

// 先包含 C++ 头文件
#include "motor_data.h"

// 再包含 C 头文件
extern "C" {
#include "config.h"
#include "motor_control.h"
#include "ethercat.h"
}

// transmit.h 包含了 C++ 内容，不要放在 extern "C" 中
#include "transmit.h"

// 全局变量
std::atomic<bool> g_running{true};
std::string g_ifname = "enp3s0";

// 信号处理
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\n[Test] Received signal " << signal << ", shutting down..." << std::endl;
        g_running = false;
    }
}

// 打印分隔线
void print_separator(const std::string& title = "") {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    if (!title.empty()) {
        std::cout << "  " << title << std::endl;
        std::cout << std::string(80, '=') << std::endl;
    }
}

// 测试 1: EtherCAT 初始化
bool test_ethercat_init() {
    print_separator("TEST 1: EtherCAT Initialization");
    
    std::cout << "[Test] Initializing EtherCAT on interface: " << g_ifname << std::endl;
    
    // 初始化 EtherCAT master
    EtherCAT_Init((char*)g_ifname.c_str());
    
    // 检查是否找到 slaves
    if (ec_slavecount <= 0) {
        std::cout << "[FAIL] No EtherCAT slaves found!" << std::endl;
        return false;
    }
    
    std::cout << "[PASS] Found " << ec_slavecount << " EtherCAT slave(s)" << std::endl;
    
    // 打印每个 slave 的信息
    for (int i = 1; i <= ec_slavecount; i++) {
        std::cout << "\n[Info] Slave " << i << ":" << std::endl;
        std::cout << "       Name: " << ec_slave[i].name << std::endl;
        std::cout << "       State: 0x" << std::hex << ec_slave[i].state << std::dec << std::endl;
        std::cout << "       Input bytes: " << ec_slave[i].Ibytes << std::endl;
        std::cout << "       Output bytes: " << ec_slave[i].Obytes << std::endl;
    }
    
    return true;
}

// 测试 2: 启动通信线程
bool test_communication_threads() {
    print_separator("TEST 2: Communication Threads");
    
    std::cout << "[Test] Starting EtherCAT communication threads..." << std::endl;
    
    try {
        ethercatManager.startThreads();
        std::cout << "[PASS] Communication threads started successfully" << std::endl;
        
        // 等待通信稳定
        std::cout << "[Info] Waiting for communication to stabilize (2 seconds)..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        return true;
    } catch (const std::exception& e) {
        std::cout << "[FAIL] Failed to start threads: " << e.what() << std::endl;
        return false;
    }
}

// 测试 3: 读取电机状态（被动读取）
bool test_read_motor_status() {
    print_separator("TEST 3: Read Motor Status (Passive)");
    
    std::cout << "[Test] Reading motor status from all slaves..." << std::endl;
    std::cout << "[Info] This will read whatever data the motors are currently sending" << std::endl;
    
    // 电机 ID 映射（根据你的配置）
    int slave_to_motor_id[3][6] = {
        {7, 8, 9, 1, 2, 3},
        {13, 17, 18, 16, 14, 15},
        {4, 5, 6, 10, 11, 12}
    };
    
    int motors_found = 0;
    
    for (int slave = 0; slave < 3; ++slave) {
        std::cout << "\n[Info] Slave " << slave << ":" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << std::left << std::setw(10) << "Passage"
                  << std::setw(10) << "Motor ID"
                  << std::setw(15) << "Position(rad)"
                  << std::setw(15) << "Velocity"
                  << std::setw(10) << "Temp(°C)"
                  << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        
        for (int passage = 1; passage <= 6; ++passage) {
            OD_Motor_Msg motor_msg = motorData.getRxMotorMsg(slave, passage);
            int expected_motor_id = slave_to_motor_id[slave][passage - 1];
            
            if (motor_msg.motor_id == expected_motor_id) {
                motors_found++;
                std::cout << std::left 
                          << std::setw(10) << passage
                          << std::setw(10) << motor_msg.motor_id
                          << std::setw(15) << std::fixed << std::setprecision(4) << motor_msg.angle_actual_float
                          << std::setw(15) << std::fixed << std::setprecision(4) << motor_msg.speed_actual_float
                          << std::setw(10) << (int)motor_msg.temperature
                          << std::endl;
            } else if (motor_msg.motor_id != 0) {
                std::cout << std::left 
                          << std::setw(10) << passage
                          << std::setw(10) << ("ID:" + std::to_string(motor_msg.motor_id) + " (expected " + std::to_string(expected_motor_id) + ")")
                          << " [ID MISMATCH]"
                          << std::endl;
            } else {
                std::cout << std::left 
                          << std::setw(10) << passage
                          << std::setw(10) << expected_motor_id
                          << " [NO DATA]"
                          << std::endl;
            }
        }
    }
    
    std::cout << "\n[Info] Total motors with valid data: " << motors_found << " / 18" << std::endl;
    
    if (motors_found >= 15) {
        std::cout << "[PASS] Most motors responding (>= 15/18)" << std::endl;
        return true;
    } else if (motors_found > 0) {
        std::cout << "[WARN] Some motors responding, but not all" << std::endl;
        return true;
    } else {
        std::cout << "[FAIL] No motor data received" << std::endl;
        return false;
    }
}

// 测试 4: 发送被动命令（阻尼模式）
bool test_send_damping_command() {
    print_separator("TEST 4: Send Damping Commands");
    
    std::cout << "[Test] Sending damping commands (kp=0, kd=3) to all motors..." << std::endl;
    std::cout << "[Info] This will put motors in low-damping mode" << std::endl;
    
    // 电机 ID 映射
    int slave_to_motor_id[3][6] = {
        {7, 8, 9, 1, 2, 3},
        {13, 17, 18, 16, 14, 15},
        {4, 5, 6, 10, 11, 12}
    };
    
    try {
        // motor_mixed_control 模式: 对每个电机独立发送命令
        for (int slave = 0; slave < 3; ++slave) {
            for (int passage = 1; passage <= 6; ++passage) {
                int motor_id = slave_to_motor_id[slave][passage - 1];
                
                // 使用 motor_mixed_control 模式发送阻尼命令
                EtherCAT_Msg tx_msg = motorData.getTxMsg(slave);
                send_motor_ctrl_cmd(&tx_msg, passage, motor_id, 
                                    0.0f,   // kp = 0 (无位置控制)
                                    3.0f,   // kd = 3 (轻阻尼)
                                    0.0f,   // pos = 0
                                    0.0f,   // spd = 0
                                    0.0f);  // tor = 0
                motorData.setTxMsg(slave, tx_msg);
            }
            std::cout << "[Info] Sent damping commands to Slave " << slave << std::endl;
        }
        
        // 等待命令生效
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "[PASS] Damping commands sent successfully" << std::endl;
        std::cout << "[Info] You can now manually move the robot joints (should feel light damping)" << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cout << "[FAIL] Failed to send commands: " << e.what() << std::endl;
        return false;
    }
}

// 测试 5: 持续监控电机状态
void test_continuous_monitoring(int duration_sec = 10) {
    print_separator("TEST 5: Continuous Motor Monitoring");
    
    std::cout << "[Test] Monitoring motor status for " << duration_sec << " seconds..." << std::endl;
    std::cout << "[Info] Try manually moving the joints to see position changes" << std::endl;
    std::cout << "[Info] Press Ctrl+C to stop early" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int sample_count = 0;
    
    while (g_running) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (elapsed >= duration_sec) {
            break;
        }
        
        // 每秒打印一次状态
        if (sample_count % 10 == 0) {
            std::cout << "\n[" << elapsed << "s] Current motor positions:" << std::endl;
            
            for (int i = 1; i <= 18; ++i) {
                // 找到电机对应的 slave 和 passage
                int slave = -1, passage = -1;
                int slave_to_motor_id[3][6] = {
                    {7, 8, 9, 1, 2, 3},
                    {13, 17, 18, 16, 14, 15},
                    {4, 5, 6, 10, 11, 12}
                };
                
                for (int s = 0; s < 3; ++s) {
                    for (int p = 0; p < 6; ++p) {
                        if (slave_to_motor_id[s][p] == i) {
                            slave = s;
                            passage = p + 1;
                            break;
                        }
                    }
                    if (slave >= 0) break;
                }
                
                if (slave >= 0 && passage >= 0) {
                    OD_Motor_Msg motor_msg = motorData.getRxMotorMsg(slave, passage);
                    if (motor_msg.motor_id == i) {
                        std::cout << "  Motor " << std::setw(2) << i << ": " 
                                  << std::fixed << std::setprecision(4) 
                                  << std::setw(8) << motor_msg.angle_actual_float << " rad, "
                                  << std::setw(8) << motor_msg.speed_actual_float << " rad/s";
                        
                        if (i % 3 == 0) std::cout << std::endl;
                    }
                }
            }
        }
        
        sample_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "\n[PASS] Monitoring completed" << std::endl;
}

// 主测试流程
int main(int argc, char** argv) {
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    print_separator("EtherCAT Communication Test Program");
    std::cout << "Purpose: Test basic EtherCAT communication with el4090 motors" << std::endl;
    std::cout << "Date: 2026-01-08" << std::endl;
    
    // 检查命令行参数
    if (argc > 1) {
        g_ifname = argv[1];
    }
    std::cout << "\nNetwork Interface: " << g_ifname << std::endl;
    std::cout << "Expected Slaves: 3" << std::endl;
    std::cout << "Expected Motors: 18" << std::endl;
    
    // 权限检查
    if (geteuid() != 0) {
        std::cout << "\n[WARN] This program may need root privileges!" << std::endl;
        std::cout << "[WARN] If tests fail, try: sudo ./test_ethercat " << g_ifname << std::endl;
    }
    
    std::cout << "\nPress Enter to start tests...";
    std::cin.get();
    
    bool all_passed = true;
    
    // 执行测试
    try {
        // 测试 1: 初始化
        if (!test_ethercat_init()) {
            std::cout << "\n[ERROR] EtherCAT initialization failed. Cannot continue." << std::endl;
            std::cout << "[Hint] Check:" << std::endl;
            std::cout << "  1. Network interface name is correct (use 'ip link' to check)" << std::endl;
            std::cout << "  2. EtherCAT slaves are powered on" << std::endl;
            std::cout << "  3. Network cable is connected" << std::endl;
            std::cout << "  4. You have root/sudo privileges" << std::endl;
            return 1;
        }
        
        // 测试 2: 启动线程
        if (!test_communication_threads()) {
            std::cout << "\n[ERROR] Communication threads failed. Cannot continue." << std::endl;
            all_passed = false;
            goto cleanup;
        }
        
        // 测试 3: 读取状态
        if (!test_read_motor_status()) {
            std::cout << "\n[WARN] Motor status reading has issues, but continuing..." << std::endl;
            all_passed = false;
        }
        
        // 测试 4: 发送命令
        if (!test_send_damping_command()) {
            std::cout << "\n[WARN] Command sending has issues, but continuing..." << std::endl;
            all_passed = false;
        }
        
        // 等待用户确认
        std::cout << "\n" << std::string(80, '-') << std::endl;
        std::cout << "Do you want to run continuous monitoring? (y/n): ";
        char response;
        std::cin >> response;
        
        if (response == 'y' || response == 'Y') {
            // 测试 5: 持续监控
            test_continuous_monitoring(10);
        }
        
    } catch (const std::exception& e) {
        std::cout << "\n[ERROR] Exception caught: " << e.what() << std::endl;
        all_passed = false;
    }
    
cleanup:
    // 清理
    print_separator("Cleanup");
    std::cout << "[Info] Stopping EtherCAT communication..." << std::endl;
    ethercatManager.stopThreads();
    std::cout << "[Info] EtherCAT communication stopped" << std::endl;
    
    // 测试结果总结
    print_separator("Test Summary");
    if (all_passed) {
        std::cout << "[SUCCESS] All tests passed! ✓" << std::endl;
        std::cout << "\nNext steps:" << std::endl;
        std::cout << "  1. Run motor calibration to get motor_calibration.yaml" << std::endl;
        std::cout << "  2. Test with rl_real_el4090 in suspended mode" << std::endl;
        std::cout << "  3. Deploy RL policy" << std::endl;
        return 0;
    } else {
        std::cout << "[WARNING] Some tests failed or had warnings" << std::endl;
        std::cout << "\nTroubleshooting:" << std::endl;
        std::cout << "  - Check motor power supply" << std::endl;
        std::cout << "  - Verify motor IDs are configured correctly" << std::endl;
        std::cout << "  - Check EtherCAT network cables" << std::endl;
        std::cout << "  - Review motor_id_map in InitMotorMapping()" << std::endl;
        return 1;
    }
}
