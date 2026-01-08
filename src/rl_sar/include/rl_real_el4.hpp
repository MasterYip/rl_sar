/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_EL4_HPP
#define RL_REAL_EL4_HPP

#include "rl_sdk.hpp"
#include "fsm_all.hpp"
#include <memory>

class RL_El4
{
public:
    RL_El4() : robot_name_("el4")
    {
        rl_ = std::make_unique<RL>();
        rl_->robot_name = robot_name_;
    }

    ~RL_El4() = default;

    bool Init()
    {
        try
        {
            // Initialize robot parameters from base.yaml
            std::string base_config_path = robot_name_ + "/base";
            rl_->params.LoadYaml(base_config_path);

            // Initialize FSM
            rl_->fsm.Initialize(rl_.get(), robot_name_);

            // Initialize control interface (gamepad/keyboard)
            rl_->control.Init();

            std::cout << LOGGER::INFO << "El4 robot initialization complete" << std::endl;
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << LOGGER::ERROR << "Initialization failed: " << e.what() << std::endl;
            return false;
        }
    }

    void  Run()
    {
        // Main control loop
        const float control_dt = 0.002f; // 500 Hz control loop
        auto last_time = std::chrono::steady_clock::now();

        while (true)
        {
            auto current_time = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(current_time - last_time).count();

            if (dt >= control_dt)
            {
                // Update control inputs
                rl_->control.Update();

                // Run FSM
                rl_->fsm.Run();

                // Send motor commands
                SendMotorCommands();

                // Read motor states
                ReadMotorStates();

                last_time = current_time;
            }

            // Small sleep to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

private:
    void SendMotorCommands()
    {
        // TODO: Implement actual hardware communication
        // This is a placeholder - you need to implement the specific communication protocol
        // for your El4 robot hardware (e.g., CAN bus, serial, etc.)
        
        // Example structure:
        // for (int i = 0; i < num_dofs; ++i)
        // {
        //     hardware_interface.SetMotorCommand(i, 
        //         rl_->fsm.GetCurrentCommand().motor_command.q[i],
        //         rl_->fsm.GetCurrentCommand().motor_command.dq[i],
        //         rl_->fsm.GetCurrentCommand().motor_command.kp[i],
        //         rl_->fsm.GetCurrentCommand().motor_command.kd[i],
        //         rl_->fsm.GetCurrentCommand().motor_command.tau[i]);
        // }
    }

    void ReadMotorStates()
    {
        // TODO: Implement actual hardware communication
        // This is a placeholder - you need to implement the specific communication protocol
        // for your El4 robot hardware
        
        // Example structure:
        // for (int i = 0; i < num_dofs; ++i)
        // {
        //     rl_->fsm.GetCurrentState().motor_state.q[i] = hardware_interface.GetPosition(i);
        //     rl_->fsm.GetCurrentState().motor_state.dq[i] = hardware_interface.GetVelocity(i);
        //     rl_->fsm.GetCurrentState().motor_state.tau[i] = hardware_interface.GetTorque(i);
        // }
        
        // // Read IMU data
        // auto imu_data = hardware_interface.GetIMUData();
        // rl_->fsm.GetCurrentState().imu.quaternion = imu_data.quaternion;
        // rl_->fsm.GetCurrentState().imu.gyroscope = imu_data.gyroscope;
        // rl_->fsm.GetCurrentState().imu.accelerometer = imu_data.accelerometer;
    }

    std::string robot_name_;
    std::unique_ptr<RL> rl_;
};

#endif // RL_REAL_EL4_HPP
