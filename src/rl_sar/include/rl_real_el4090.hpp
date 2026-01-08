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
#include "fsm_el4.hpp"

#include <csignal>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <array>
#include <map>
#include <fstream>

// EtherCAT motor control
extern "C" {
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
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_hardware_send;
    std::shared_ptr<LoopFunc> loop_hardware_recv;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

#ifdef PLOT
    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
#endif

    // hardware interface - EtherCAT motor control
    void HardwareSend();
    void HardwareRecv();
    
    // EtherCAT initialization
    bool InitEtherCAT(const char* ifname);
    void ShutdownEtherCAT();
    
    // EtherCAT network interface name
    std::string ethercat_ifname_;
    
    // Motor ID mapping: [slave][passage] -> motor_id
    // slave: EtherCAT slave index (0-2 for 3 slaves)
    // passage: CAN passage on slave (1-6)
    static constexpr int SLAVE_COUNT = 3;
    static constexpr int PASSAGE_PER_SLAVE = 6;
    std::array<std::array<int, PASSAGE_PER_SLAVE>, SLAVE_COUNT> motor_id_map_;
    
    // Reverse mapping: motor_id -> [slave, passage]
    struct MotorLocation {
        int slave;
        int passage;
    };
    std::map<int, MotorLocation> motor_location_map_;
    
    // Motor calibration offsets (from YAML file)
    std::map<int, float> motor_offsets_;
    
    // Helper functions
    void InitMotorMapping();
    void LoadMotorCalibration(const std::string& yaml_path);
    float GetCommandAngle(int motor_id, float target_angle);
    void SendMotorCommand(int slave, int passage, int motor_id, float kp, float kd, float pos, float spd, float tor);
    void ReadMotorStatus(int slave, int passage, int motor_id, float& position, float& velocity, float& torque);
    
    // Motor state buffers
    struct MotorState
    {
        std::vector<float> position;      // Joint positions (rad)
        std::vector<float> velocity;      // Joint velocities (rad/s)
        std::vector<float> torque;        // Joint torques (N·m)
        std::vector<float> temperature;   // Motor temperatures (°C)
    };
    
    // IMU state buffers
    struct IMUState
    {
        std::array<float, 4> quaternion;  // [w, x, y, z]
        std::array<float, 3> gyroscope;   // [roll_rate, pitch_rate, yaw_rate] (rad/s)
        std::array<float, 3> accelerometer; // [ax, ay, az] (m/s^2)
    };
    
    MotorState motor_state_buffer;
    IMUState imu_state_buffer;
    
    // Motor command buffers
    struct MotorCommand
    {
        std::vector<float> target_position;  // Target joint positions (rad)
        std::vector<float> target_velocity;  // Target joint velocities (rad/s)
        std::vector<float> kp;               // Position gains
        std::vector<float> kd;               // Velocity gains
        std::vector<float> feedforward_torque; // Feedforward torques (N·m)
    };
    
    MotorCommand motor_command_buffer;

    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;
    int num_dofs;

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
