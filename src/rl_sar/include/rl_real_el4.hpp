/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_EL4_HPP
#define RL_REAL_EL4_HPP

#include "rl_sdk.hpp"
#include "fsm_all.hpp"
#include <string>
#include <chrono>
#include <thread>

class RL_El4 : public RL
{
public:
    RL_El4(const std::string& gamepad_device = "/dev/input/js0")
    {
        this->robot_name = "el4";
        this->gamepad_device_path = gamepad_device;
    }

    ~RL_El4() = default;

    bool Init()
    {
        try
        {
            // Initialize robot parameters from base.yaml
            this->ReadYaml(this->robot_name, "base.yaml");

            // Initialize FSM via the FSMManager (same pattern as G1, A1, Go2, Lite3)
            if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
            {
                auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
                if (fsm_ptr)
                {
                    this->fsm = *fsm_ptr;
                }
                else
                {
                    std::cout << LOGGER::ERROR << "[FSM] Failed to create FSM for: " << this->robot_name << std::endl;
                    return false;
                }
            }
            else
            {
                std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
                return false;
            }

            // Initialize joint count, outputs, and control
            this->InitJointNum(this->params.Get<int>("num_of_dofs"));
            this->InitOutputs();
            this->InitControl();

            std::cout << LOGGER::INFO << "El4 robot initialization complete" << std::endl;
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << LOGGER::ERROR << "Initialization failed: " << e.what() << std::endl;
            return false;
        }
    }

    void Run()
    {
        const float control_dt = this->params.Get<float>("dt", 0.002f); // 500 Hz default
        auto last_time = std::chrono::steady_clock::now();

        while (true)
        {
            auto current_time = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(current_time - last_time).count();

            if (dt >= control_dt)
            {
                // 1. Read motor states into robot_state (hardware I/O)
                ReadMotorStates();

                // 2. Read keyboard and gamepad input (updates control struct)
                this->KeyboardInterface();
                this->GamepadInterface();

                // 3. Run FSM + axis commands + navigation toggle
                //    StateController handles FSM transitions based on
                //    keyboard/gamepad input AND axis accumulation (W/S/A/D/Q/E)
                this->StateController(&this->robot_state, &this->robot_command);

                // 4. Clear transient input for next control cycle
                this->control.ClearInput();

                // 5. Send motor commands from robot_command (hardware I/O)
                SendMotorCommands();

                last_time = current_time;
            }

            // Small sleep to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

private:
    // --- Pure virtual method implementations (stubs for now) ---
    std::vector<float> Forward() override
    {
        // TODO: Run RL inference
        return std::vector<float>(this->params.Get<int>("num_of_dofs"), 0.0f);
    }

    void GetState(RobotState<float> *state) override
    {
        // Overridden by ReadMotorStates() in the control loop.
        // This is called by RLFSMState internals (RLControl, etc).
        *state = this->robot_state;
    }

    void SetCommand(const RobotCommand<float> *command) override
    {
        // Overridden by SendMotorCommands() in the control loop.
        this->robot_command = *command;
    }

    void SendMotorCommands()
    {
        // TODO: Implement actual hardware communication
        // This is a placeholder - you need to implement the specific communication protocol
        // for your El4 robot hardware (e.g., CAN bus, serial, etc.)

        // Example structure:
        // for (int i = 0; i < num_dofs; ++i)
        // {
        //     hardware_interface.SetMotorCommand(i,
        //         this->robot_command.motor_command.q[i],
        //         this->robot_command.motor_command.dq[i],
        //         this->robot_command.motor_command.kp[i],
        //         this->robot_command.motor_command.kd[i],
        //         this->robot_command.motor_command.tau[i]);
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
        //     this->robot_state.motor_state.q[i] = hardware_interface.GetPosition(i);
        //     this->robot_state.motor_state.dq[i] = hardware_interface.GetVelocity(i);
        //     this->robot_state.motor_state.tau_est[i] = hardware_interface.GetTorque(i);
        // }

        // // Read IMU data
        // auto imu_data = hardware_interface.GetIMUData();
        // this->robot_state.imu.quaternion = imu_data.quaternion;
        // this->robot_state.imu.gyroscope = imu_data.gyroscope;
        // this->robot_state.imu.accelerometer = imu_data.accelerometer;
    }
};

#endif // RL_REAL_EL4_HPP
