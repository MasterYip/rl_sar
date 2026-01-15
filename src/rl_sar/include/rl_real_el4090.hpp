/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_EL4090_HPP
#define RL_REAL_EL4090_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_el4090.hpp"
#include "ahrs_interface.h"

#include <csignal>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <array>
#include <map>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

// EtherCAT motor control
extern "C"
{
#include "config.h"
#include "motor_control.h"
#include "transmit.h"
}
#include "motor_data.h"

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();

#if defined(USE_ROS2) && defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    // ==================== Core Functions ====================

    // RL functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // Config and buffer initialization
    void InitConfigAndBuffers();

    // EtherCAT functions
    bool InitEtherCAT(const char *ifname);
    void ShutdownEtherCAT();

    // Motor send/receive
    void HardwareSend();
    void HardwareRecv();

    // IMU receive
    void IMURecv();

    // Joystick functions
    void InitJoystick();
    void UpdateJoystick();
    void ShutdownJoystick();

    // ==================== Data Members ====================

    // Loop threads
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_hardware_send;
    std::shared_ptr<LoopFunc> loop_hardware_recv;
    std::shared_ptr<LoopFunc> loop_imu_recv;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // Mapping: Policy Index ↔ Motor ID
    std::vector<int> policy_to_motor_id_;   // Policy Index → Motor ID
    std::map<int, int> motor_id_to_policy_; // Motor ID → Policy Index

    // Mapping: Motor ID ↔ EtherCAT Address
    struct EtherCATAddr
    {
        int slave;
        int passage;
    };
    std::map<int, EtherCATAddr> motor_ethercat_addr_; // Motor ID → [slave, passage]

    // Motor properties (indexed by Motor ID)
    std::map<int, int> motor_direction_; // Motor ID → direction (+1 or -1)
    std::map<int, float> motor_offset_;  // Motor ID → calibration offset

    // State and command buffers (indexed by Policy Index)
    struct MotorState
    {
        std::vector<float> position;
        std::vector<float> velocity;
        std::vector<float> torque;
        std::vector<float> temperature;
    };

    struct IMUState
    {
        std::array<float, 4> quaternion;
        std::array<float, 3> gyroscope;
        std::array<float, 3> accelerometer;
    };

    struct MotorCommand
    {
        std::vector<float> target_position;
        std::vector<float> target_velocity;
        std::vector<float> kp;
        std::vector<float> kd;
        std::vector<float> feedforward_torque;
    };

    MotorState motor_state_buffer;
    IMUState imu_state_buffer;
    MotorCommand motor_command_buffer;

    // IMU interface
    std::unique_ptr<FDILink::AHRSInterface> imu_interface_;
    FDILink::ImuData last_imu_data_;

    // Velocity estimation
    std::array<float, 3> estimated_velocity_; // body frame linear velocity
    std::array<float, 3> last_accelerometer_; // for velocity integration
    std::chrono::steady_clock::time_point last_vel_update_time_;
    void UpdateVelocityEstimation();

    // Debug printing
    int debug_print_counter_;
    void PrintDebugInfo();

    // EtherCAT interface
    std::string ethercat_ifname_;
    int num_dofs;

    // Joystick interface
    int joystick_fd_;
    bool joystick_enabled_;
    struct js_event joystick_event_;
    std::map<int, bool> button_states_;
    std::map<int, float> axis_values_;

#ifdef PLOT
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
#endif

    // others

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif

#ifdef CSV_LOGGER
    // CSV logger for data recording
    std::ofstream csv_file;
    void InitCSVLogger();
    void LogToCSV();
#endif
};

#endif // RL_REAL_EL4090_HPP
