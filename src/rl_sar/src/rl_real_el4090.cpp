/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_el4090.hpp"

RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        { this->CmdvelCallback(msg); });
#endif

    this->ang_vel_axis = "body";
    this->robot_name = "el_4090";
    this->ReadYaml(this->robot_name + "/" + "legged_gym", "config.yaml");
    this->num_dofs = this->params.Get<int>("num_of_dofs");

    this->InitJointNum(this->num_dofs);
    this->InitOutputs();
    this->InitControl();

    // Initialize config and buffers
    InitConfigAndBuffers();

    // Initialize Joystick
    InitJoystick();

    // Initialize EtherCAT (get interface name from command line)
    if (argc > 1)
    {
        ethercat_ifname_ = argv[1];
    }
    else
    {
        ethercat_ifname_ = "enp86s0";
        std::cout << LOGGER::WARNING << "Using default interface: " << ethercat_ifname_ << std::endl;
    }

    bool ethercat_ok = InitEtherCAT(ethercat_ifname_.c_str());
    if (!ethercat_ok)
    {
        std::cout << LOGGER::WARNING << "EtherCAT initialization failed - continuing without motor control" << std::endl;
        std::cout << LOGGER::WARNING << "IMU and other sensors will still function" << std::endl;
    }

    // Load FSM after hardware initialization
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
            std::cout << LOGGER::INFO << "FSM loaded for " << this->robot_name << std::endl;
        }
    }

    // Start control loops
    this->loop_hardware_recv = std::make_shared<LoopFunc>("loop_hardware_recv", 0.002, std::bind(&RL_Real::HardwareRecv, this), 3);
    this->loop_hardware_send = std::make_shared<LoopFunc>("loop_hardware_send", 0.002, std::bind(&RL_Real::HardwareSend, this), 3);
    this->loop_imu_recv = std::make_shared<LoopFunc>("loop_imu_recv", 0.002, std::bind(&RL_Real::IMURecv, this), 3);
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));

    this->loop_hardware_recv->start();
    this->loop_hardware_send->start();
    this->loop_imu_recv->start();
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->num_dofs);
    this->plot_target_joint_pos.resize(this->num_dofs);
    for (auto &vector : this->plot_real_joint_pos)
        vector = std::vector<float>(this->plot_size, 0);
    for (auto &vector : this->plot_target_joint_pos)
        vector = std::vector<float>(this->plot_size, 0);
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif

// #ifdef CSV_LOGGER
//     this->CSVInit(this->robot_name);
// #endif

    this->CSVInit(this->robot_name);

    std::cout << LOGGER::INFO << "RL_Real initialized (" << this->num_dofs << " DOFs)" << std::endl;
}

RL_Real::~RL_Real()
{
    this->loop_hardware_recv->shutdown();
    this->loop_hardware_send->shutdown();
    this->loop_imu_recv->shutdown();
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif

    ShutdownJoystick();

    // Shutdown IMU interface
    if (imu_interface_)
    {
        imu_interface_->stop();
    }

    ShutdownEtherCAT();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

// ==================== Config and Buffer Initialization ====================

void RL_Real::InitConfigAndBuffers()
{
    std::cout << LOGGER::INFO << "Initializing config and buffers..." << std::endl;

    // Read mapping from config.yaml
    // joint_mapping: Policy Index (0-17) → Motor ID (1-18)
    policy_to_motor_id_ = this->params.Get<std::vector<int>>("joint_mapping");

    // Read motor directions: +1 or -1 for each motor (按Policy顺序)
    auto motor_directions = this->params.Get<std::vector<int>>("motor_directions");

    // Read motor offsets: calibration offset for each motor (按Motor ID顺序)
    auto motor_offsets = this->params.Get<std::vector<float>>("motor_offsets");

    // Build reverse mapping: Motor ID → Policy Index
    motor_id_to_policy_.clear();
    for (int policy_idx = 0; policy_idx < num_dofs; ++policy_idx)
    {
        int motor_id = policy_to_motor_id_[policy_idx];
        motor_id_to_policy_[motor_id] = policy_idx;

        // Store direction and offset by motor_id
        motor_direction_[motor_id] = motor_directions[policy_idx];
        motor_offset_[motor_id] = motor_offsets[motor_id - 1]; // offsets按motor_id顺序
    }

    // Initialize EtherCAT topology: Motor ID → [slave, passage]
    // slave 0: motor_id [7,8,9,1,2,3] → passage [1-6]
    // slave 1: motor_id [13,17,18,16,14,15] → passage [1-6]
    // slave 2: motor_id [4,5,6,10,11,12] → passage [1-6]
    std::vector<std::vector<int>> motor_id_map = {
        {7, 8, 9, 1, 2, 3},
        {13, 17, 18, 16, 14, 15},
        {4, 5, 6, 10, 11, 12}};

    for (int slave = 0; slave < 3; ++slave)
    {
        for (int passage_idx = 0; passage_idx < 6; ++passage_idx)
        {
            int motor_id = motor_id_map[slave][passage_idx];
            motor_ethercat_addr_[motor_id] = {slave, passage_idx + 1};
        }
    }

    // Initialize state buffers (按Policy顺序存储)
    motor_state_buffer.position.resize(num_dofs, 0.0f);
    motor_state_buffer.velocity.resize(num_dofs, 0.0f);
    motor_state_buffer.torque.resize(num_dofs, 0.0f);
    motor_state_buffer.temperature.resize(num_dofs, 0.0f);

    // Initialize command buffers (按Policy顺序存储)
    motor_command_buffer.target_position.resize(num_dofs, 0.0f);
    motor_command_buffer.target_velocity.resize(num_dofs, 0.0f);
    motor_command_buffer.kp.resize(num_dofs, 0.0f);
    motor_command_buffer.kd.resize(num_dofs, 0.0f);
    motor_command_buffer.feedforward_torque.resize(num_dofs, 0.0f);

    // Initialize IMU state
    imu_state_buffer.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    imu_state_buffer.gyroscope = {0.0f, 0.0f, 0.0f};
    imu_state_buffer.accelerometer = {0.0f, 0.0f, 0.0f};

    // Initialize velocity estimation
    estimated_velocity_ = {0.0f, 0.0f, 0.0f};
    last_accelerometer_ = {0.0f, 0.0f, 0.0f};
    last_vel_update_time_ = std::chrono::steady_clock::now();
    debug_print_counter_ = 0;

    // Initialize IMU interface
    std::string imu_port = this->params.Get<std::string>("imu_port", "/dev/ttyUSB0");
    int imu_baud = this->params.Get<int>("imu_baud", 921600);
    int imu_timeout_ms = this->params.Get<int>("imu_timeout_ms", 20);

    std::cout << LOGGER::INFO << "IMU Configuration:" << std::endl;
    std::cout << "  Port: " << imu_port << std::endl;
    std::cout << "  Baud Rate: " << imu_baud << " bps" << std::endl;
    std::cout << "  Timeout: " << imu_timeout_ms << " ms" << std::endl;

    imu_interface_ = std::make_unique<FDILink::AHRSInterface>(imu_port, imu_baud, imu_timeout_ms);

    // Set IMU data callback to update last_imu_data_ (same logic as print_imu example)
    imu_interface_->setImuCallback([this](const FDILink::ImuData &imu_data)
                                   {
                                       last_imu_data_ = imu_data;
                                   });

    // Start IMU interface
    if (!imu_interface_->start())
    {
        std::cout << LOGGER::WARNING << "Failed to start IMU interface on " << imu_port
                  << " (baud: " << imu_baud << ", timeout: " << imu_timeout_ms << "ms)" << std::endl;
        std::cout << LOGGER::WARNING << "IMU will continue attempting to connect in background..." << std::endl;
    }
    else
    {
        std::cout << LOGGER::INFO << "IMU interface started on " << imu_port
                  << " (baud: " << imu_baud << ", timeout: " << imu_timeout_ms << "ms)" << std::endl;
        std::cout << LOGGER::INFO << "IMU data will be read by loop_imu_recv" << std::endl;
    }

    std::cout << LOGGER::INFO << "Config and buffers initialized" << std::endl;
}

// ==================== EtherCAT Functions ====================

bool RL_Real::InitEtherCAT(const char *ifname)
{
    std::cout << LOGGER::INFO << "Initializing EtherCAT on " << ifname << std::endl;

    EtherCAT_Init((char *)ifname);

    if (ec_slavecount <= 0)
    {
        std::cout << LOGGER::ERROR << "No EtherCAT slaves found!" << std::endl;
        return false;
    }

    std::cout << LOGGER::INFO << "Found " << ec_slavecount << " EtherCAT slave(s)" << std::endl;

    ethercatManager.startThreads();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << LOGGER::INFO << "EtherCAT initialized successfully" << std::endl;
    return true;
}

void RL_Real::ShutdownEtherCAT()
{
    std::cout << LOGGER::INFO << "Shutting down EtherCAT..." << std::endl;
    ethercatManager.stopThreads();
    std::cout << LOGGER::INFO << "EtherCAT shutdown complete" << std::endl;
}

// ==================== Motor Send/Receive ====================

void RL_Real::HardwareSend()
{
    // 发送流程: Policy Index → Motor ID → EtherCAT
    for (int policy_idx = 0; policy_idx < num_dofs; ++policy_idx)
    {
        int motor_id = policy_to_motor_id_[policy_idx];
        auto &addr = motor_ethercat_addr_[motor_id];

        // 坐标变换: URDF angle → Motor physical position
        // motor_pos = direction * urdf_angle + offset
        float urdf_angle = motor_command_buffer.target_position[policy_idx];
        float motor_pos = motor_direction_[motor_id] * urdf_angle + motor_offset_[motor_id];

        // 发送电机命令
        EtherCAT_Msg tx_msg = motorData.getTxMsg(addr.slave);
        send_motor_ctrl_cmd(&tx_msg,
                            addr.passage,
                            motor_id,
                            motor_command_buffer.kp[policy_idx],
                            motor_command_buffer.kd[policy_idx],
                            motor_pos,
                            motor_command_buffer.target_velocity[policy_idx],
                            motor_command_buffer.feedforward_torque[policy_idx]);
        motorData.setTxMsg(addr.slave, tx_msg);
    }
}

void RL_Real::HardwareRecv()
{
    // 接收流程: EtherCAT → Motor ID → Policy Index
    static int recv_print_counter = 0;
    static bool first_print = true;
    bool should_print = (recv_print_counter % 250 == 0); // 每250个周期打印一次

    // 先收集所有数据
    for (int policy_idx = 0; policy_idx < num_dofs; ++policy_idx)
    {
        int motor_id = policy_to_motor_id_[policy_idx];
        auto &addr = motor_ethercat_addr_[motor_id];

        // 读取电机状态
        OD_Motor_Msg motor_msg = motorData.getRxMotorMsg(addr.slave, addr.passage);

        if (motor_msg.motor_id == motor_id)
        {
            // 坐标变换: Motor physical position → URDF angle
            // urdf_angle = direction * (motor_pos - offset)
            float motor_pos = motor_msg.angle_actual_rad;
            float urdf_angle = motor_direction_[motor_id] * (motor_pos - motor_offset_[motor_id]);

            motor_state_buffer.position[policy_idx] = urdf_angle;
            motor_state_buffer.velocity[policy_idx] = motor_direction_[motor_id] * motor_msg.speed_actual_rad;
            motor_state_buffer.torque[policy_idx] = motor_msg.current_actual_int;
        }
    }

    UpdateVelocityEstimation();
}

void RL_Real::IMURecv()
{
    // Read IMU data from callback (already updated in background thread)
    // Here we use the callback-set data for the state buffer
    if (!imu_interface_)
    {
        return;
    }

    // Copy from last_imu_data_ that was set by the IMU's internal thread
    // The IMU SDK already runs its own thread to read serial data
    FDILink::ImuData imu_data = last_imu_data_;

    // Update IMU state buffer with coordinate transformation
    // IMU frame: X forward, Y right, Z down
    // URDF frame: X forward, Y left, Z up
    // Transformation: x_urdf = x_imu, y_urdf = -y_imu, z_urdf = -z_imu
    // This is equivalent to a 180° rotation around the X axis

    // Quaternion transformation: (w, x, y, z)_imu -> (w, x, -y, -z)_urdf
    // For rotation around X axis by 180°, only y and z components change sign
    imu_state_buffer.quaternion[0] = static_cast<float>(imu_data.qw);
    imu_state_buffer.quaternion[1] = static_cast<float>(imu_data.qx);
    imu_state_buffer.quaternion[2] = -static_cast<float>(imu_data.qy); // Y component inverted
    imu_state_buffer.quaternion[3] = -static_cast<float>(imu_data.qz); // Z component inverted

    // Angular velocity transformation: (ωx, ωy, ωz)_imu -> (ωx, -ωy, -ωz)_urdf
    imu_state_buffer.gyroscope[0] = imu_data.gx;
    imu_state_buffer.gyroscope[1] = -imu_data.gy; // Y axis inverted
    imu_state_buffer.gyroscope[2] = -imu_data.gz; // Z axis inverted

    // Linear acceleration transformation: (ax, ay, az)_imu -> (ax, -ay, -az)_urdf
    imu_state_buffer.accelerometer[0] = imu_data.ax;
    imu_state_buffer.accelerometer[1] = -imu_data.ay; // Y axis inverted
    imu_state_buffer.accelerometer[2] = -imu_data.az; // Z axis inverted

    // Debug printing is now done in the callback, no need to duplicate here
    // If you want to print here instead, comment out the callback printing
}

// ==================== FSM Related ====================

void RL_Real::GetState(RobotState<float> *state)
{
    // Update joystick input
    UpdateJoystick();

    // Copy IMU state
    state->imu.quaternion[0] = imu_state_buffer.quaternion[0];
    state->imu.quaternion[1] = imu_state_buffer.quaternion[1];
    state->imu.quaternion[2] = imu_state_buffer.quaternion[2];
    state->imu.quaternion[3] = imu_state_buffer.quaternion[3];

    for (int i = 0; i < 3; ++i)
        state->imu.gyroscope[i] = imu_state_buffer.gyroscope[i];

    // Debug: Print IMU data copied to state (every 250 calls)
    // static int state_print_counter = 0;
    // if (state_print_counter++ % 250 == 0)
    // {
    //     std::cout << "[GetState] IMU → State | Q=["
    //               << state->imu.quaternion[0] << "," << state->imu.quaternion[1] << ","
    //               << state->imu.quaternion[2] << "," << state->imu.quaternion[3]
    //               << "] | G=[" << state->imu.gyroscope[0] << "," << state->imu.gyroscope[1]
    //               << "," << state->imu.gyroscope[2] << "]" << std::endl;
    // }

    // Copy motor state (already in Policy order)
    for (int i = 0; i < this->num_dofs; ++i)
    {
        state->motor_state.q[i] = motor_state_buffer.position[i];
        state->motor_state.dq[i] = motor_state_buffer.velocity[i];
        state->motor_state.tau_est[i] = motor_state_buffer.torque[i];
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    // Store command (already in Policy order)
    for (int i = 0; i < this->num_dofs; ++i)
    {
        motor_command_buffer.target_position[i] = command->motor_command.q[i];
        motor_command_buffer.target_velocity[i] = command->motor_command.dq[i];
        motor_command_buffer.kp[i] = command->motor_command.kp[i];
        motor_command_buffer.kd[i] = command->motor_command.kd[i];
        motor_command_buffer.feedforward_torque[i] = command->motor_command.tau[i];
    }
}

// ==================== Control Loop ====================

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);

// #ifdef CSV_LOGGER
//     this->LogToCSV();
// #endif

    this->LogToCSV();
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;

        // 更新IMU数据
        this->obs.base_quat = this->robot_state.imu.quaternion;

        // 1. 角速度：直接使用IMU陀螺仪数据（body frame）
        this->obs.ang_vel = this->robot_state.imu.gyroscope;

        // 2. 重力向量：使用四元数将世界坐标系重力向量旋转到body frame
        // 世界坐标系重力加速度向量为 [0, 0, -g]（向下，指向地心）
        // 使用四元数旋转公式: v_body = R(q) * v_world
        // 其中 R(q) 是由四元数q构成的旋转矩阵
        float qw = this->obs.base_quat[0];
        float qx = this->obs.base_quat[1];
        float qy = this->obs.base_quat[2];
        float qz = this->obs.base_quat[3];

        // 计算 R(q) * [0, 0, -1] = -R(q)的第三列
        // 重力向量指向地心（向下），在body frame中应该根据姿态变化
        // 如果机器人水平放置，gravity_vec应该是[0, 0, -1]（指向地面）
        this->obs.gravity_vec[0] = -2.0f * (qx * qz + qw * qy);
        this->obs.gravity_vec[1] = -2.0f * (qy * qz - qw * qx);
        this->obs.gravity_vec[2] = -(1.0f - 2.0f * (qx * qx + qy * qy));

        // 归一化重力向量（确保长度为1）
        float grav_norm = std::sqrt(this->obs.gravity_vec[0] * this->obs.gravity_vec[0] +
                                    this->obs.gravity_vec[1] * this->obs.gravity_vec[1] +
                                    this->obs.gravity_vec[2] * this->obs.gravity_vec[2]);
        if (grav_norm > 0.01f)
        {
            this->obs.gravity_vec[0] /= grav_norm;
            this->obs.gravity_vec[1] /= grav_norm;
            this->obs.gravity_vec[2] /= grav_norm;
        }

        // 3. 线速度：使用改进的速度估计（从UpdateVelocityEstimation获取）
        this->obs.lin_vel[0] = estimated_velocity_[0];
        this->obs.lin_vel[1] = estimated_velocity_[1];
        // this->obs.lin_vel[2] = estimated_velocity_[2];
        this->obs.lin_vel[2] = 0;

        // 打印调试信息
        PrintDebugInfo();

        // Debug: Print obs IMU data before sending to policy network (every 50 calls)
        static int obs_print_counter = 0;
        if (obs_print_counter++ % 25 == 0)
        {
            std::cout << "\n========== OBSERVATION DATA (to Policy Network) ==========" << std::endl;
            std::cout << "[Obs] lin_vel (3): [" << this->obs.lin_vel[0] << ", " << this->obs.lin_vel[1] << ", " << this->obs.lin_vel[2] << "]" << std::endl;
            std::cout << "[Obs] ang_vel (3): [" << this->obs.ang_vel[0] << ", " << this->obs.ang_vel[1] << ", " << this->obs.ang_vel[2] << "]" << std::endl;
            std::cout << "[Obs] gravity_vec (3): [" << this->obs.gravity_vec[0] << ", " << this->obs.gravity_vec[1] << ", " << this->obs.gravity_vec[2] << "]" << std::endl;
            std::cout << "[Obs] commands (3): [" << this->obs.commands[0] << ", " << this->obs.commands[1] << ", " << this->obs.commands[2] << "]" << std::endl;
            std::cout << "[Obs] dof_pos (18): [";
            for (int i = 0; i < 18; i++)
                std::cout << this->obs.dof_pos[i] << (i < 17 ? ", " : "");
            std::cout << "]" << std::endl;
            std::cout << "[Obs] dof_vel (18): [";
            for (int i = 0; i < 18; i++)
                std::cout << this->obs.dof_vel[i] << (i < 17 ? ", " : "");
            std::cout << "]" << std::endl;
            std::cout << "[Obs] actions (18): [";
            for (int i = 0; i < 18; i++)
                std::cout << this->obs.actions[i] << (i < 17 ? ", " : "");
            std::cout << "]" << std::endl;
            std::cout << "Total: 3+3+3+3+18+18+18 = 66 observations" << std::endl;
            std::cout << "(base_quat not included - only used to compute gravity_vec)" << std::endl;
            std::cout << "=========================================================\n"
                      << std::endl;
        }

        // 更新命令和关节状态
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
#endif
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
            output_dof_pos_queue.push(this->output_dof_pos);
        if (!this->output_dof_vel.empty())
            output_dof_vel_queue.push(this->output_dof_vel);
        if (!this->output_dof_tau.empty())
            output_dof_tau_queue.push(this->output_dof_tau);
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"),
                     this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

// ==================== Utility Functions ====================

#ifdef PLOT
void RL_Real::Plot()
{
    static int plot_counter = 0;
    plot_counter++;

    for (int i = 0; i < this->plot_size - 1; ++i)
    {
        this->plot_t[i] = this->plot_t[i + 1];
        for (int j = 0; j < this->num_dofs; ++j)
        {
            this->plot_real_joint_pos[j][i] = this->plot_real_joint_pos[j][i + 1];
            this->plot_target_joint_pos[j][i] = this->plot_target_joint_pos[j][i + 1];
        }
    }

    this->plot_t[this->plot_size - 1] = plot_counter;
    for (int i = 0; i < this->num_dofs; ++i)
    {
        this->plot_real_joint_pos[i][this->plot_size - 1] = this->robot_state.motor_state.q[i];
        this->plot_target_joint_pos[i][this->plot_size - 1] = this->robot_command.motor_command.q[i];
    }

    plt::clf();
    for (int i = 0; i < this->num_dofs; ++i)
    {
        plt::named_plot("Real Joint " + std::to_string(i), this->plot_t, this->plot_real_joint_pos[i]);
        plt::named_plot("Target Joint " + std::to_string(i), this->plot_t, this->plot_target_joint_pos[i], "--");
    }
    plt::legend();
    plt::pause(0.001);
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->cmd_vel = *msg;
}
#elif defined(USE_ROS2) && defined(USE_ROS)
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}
#endif


void RL_Real::LogToCSV()
{
    if (csv_file.is_open())
    {
        std::cout<<LOGGER::DEBUG << "Logging data to CSV..." << std::endl;
        csv_file << this->episode_length_buf;
        for (int i = 0; i < this->num_dofs; ++i)
            csv_file << "," << this->robot_state.motor_state.q[i];
        for (int i = 0; i < this->num_dofs; ++i)
            csv_file << "," << this->robot_state.motor_state.dq[i];
        for (int i = 0; i < this->num_dofs; ++i)
            csv_file << "," << this->robot_state.motor_state.tau_est[i];
        for (int i = 0; i < this->num_dofs; ++i)
            csv_file << "," << this->robot_command.motor_command.q[i];
        csv_file << std::endl;
    }
}


// ==================== Joystick Functions ====================

void RL_Real::InitJoystick()
{
    joystick_fd_ = -1;
    joystick_enabled_ = false;

    const char *joystick_device = "/dev/input/js0";
    joystick_fd_ = open(joystick_device, O_RDONLY | O_NONBLOCK);

    if (joystick_fd_ < 0)
    {
        std::cout << LOGGER::WARNING << "Joystick not found at " << joystick_device << std::endl;
        std::cout << LOGGER::INFO << "Using keyboard control only" << std::endl;
        return;
    }

    char name[128];
    if (ioctl(joystick_fd_, JSIOCGNAME(sizeof(name)), name) < 0)
    {
        std::cout << LOGGER::WARNING << "Could not get joystick name" << std::endl;
    }
    else
    {
        std::cout << LOGGER::INFO << "Joystick detected: " << name << std::endl;
    }

    joystick_enabled_ = true;

    // Initialize button and axis maps
    for (int i = 0; i < 20; ++i)
        button_states_[i] = false;
    for (int i = 0; i < 10; ++i)
        axis_values_[i] = 0.0f;

    std::cout << LOGGER::INFO << "Joystick initialized" << std::endl;
}

void RL_Real::UpdateJoystick()
{
    if (!joystick_enabled_)
        return;

    static int print_counter = 0;
    bool should_print = (print_counter % 50 == 0); // 每50个周期打印一次
    should_print = false;

    // Read all pending joystick events
    int event_count = 0;
    while (read(joystick_fd_, &joystick_event_, sizeof(joystick_event_)) > 0)
    {
        event_count++;
        switch (joystick_event_.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_BUTTON:
            button_states_[joystick_event_.number] = joystick_event_.value;
            if (should_print)
            {
                std::cout << LOGGER::INFO << "Button " << (int)joystick_event_.number
                          << " = " << (int)joystick_event_.value << std::endl;
            }
            break;
        case JS_EVENT_AXIS:
            axis_values_[joystick_event_.number] = joystick_event_.value / 32767.0f;
            if (should_print)
            {
                std::cout << LOGGER::INFO << "Axis " << (int)joystick_event_.number
                          << " = " << axis_values_[joystick_event_.number]
                          << " (raw: " << joystick_event_.value << ")" << std::endl;
            }
            break;
        }
    }

    if (should_print && event_count > 0)
    {
        std::cout << LOGGER::INFO << "Read " << event_count << " joystick events" << std::endl;
    }
    print_counter++;

    // Map buttons to gamepad actions (按照通用游戏手柄布局)
    // Button mapping (may vary by controller):
    // 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=Back, 7=Start, 8=LStick, 9=RStick
    // 上下左右通常在 axis 或 hat 上

    if (button_states_[0])
        this->control.SetGamepad(Input::Gamepad::A);
    if (button_states_[1])
        this->control.SetGamepad(Input::Gamepad::B);
    if (button_states_[2])
        this->control.SetGamepad(Input::Gamepad::X);
    if (button_states_[3])
        this->control.SetGamepad(Input::Gamepad::Y);
    if (button_states_[4])
        this->control.SetGamepad(Input::Gamepad::LB);
    if (button_states_[5])
        this->control.SetGamepad(Input::Gamepad::RB);
    if (button_states_[8])
        this->control.SetGamepad(Input::Gamepad::LStick);
    if (button_states_[9])
        this->control.SetGamepad(Input::Gamepad::RStick);

    // D-Pad (如果在 axis 上)
    if (axis_values_.count(7))
    {
        if (axis_values_[7] > 0.5f)
            this->control.SetGamepad(Input::Gamepad::DPadDown);
        if (axis_values_[7] < -0.5f)
            this->control.SetGamepad(Input::Gamepad::DPadUp);
    }
    if (axis_values_.count(6))
    {
        if (axis_values_[6] > 0.5f)
            this->control.SetGamepad(Input::Gamepad::DPadRight);
        if (axis_values_[6] < -0.5f)
            this->control.SetGamepad(Input::Gamepad::DPadLeft);
    }

    // 组合键
    if (button_states_[4] && button_states_[0])
        this->control.SetGamepad(Input::Gamepad::LB_A);
    if (button_states_[4] && button_states_[1])
        this->control.SetGamepad(Input::Gamepad::LB_B);
    if (button_states_[4] && button_states_[2])
        this->control.SetGamepad(Input::Gamepad::LB_X);
    if (button_states_[4] && button_states_[3])
        this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (button_states_[5] && button_states_[0])
        this->control.SetGamepad(Input::Gamepad::RB_A);
    if (button_states_[5] && button_states_[1])
        this->control.SetGamepad(Input::Gamepad::RB_B);
    if (button_states_[5] && button_states_[2])
        this->control.SetGamepad(Input::Gamepad::RB_X);
    if (button_states_[5] && button_states_[3])
        this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (button_states_[4] && button_states_[5])
        this->control.SetGamepad(Input::Gamepad::LB_RB);

    // DPad组合键 (需要在单独的DPad检查之后)
    if (button_states_[5] && axis_values_.count(7) && axis_values_[7] < -0.5f)
        this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (button_states_[5] && axis_values_.count(7) && axis_values_[7] > 0.5f)
        this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (button_states_[5] && axis_values_.count(6) && axis_values_[6] < -0.5f)
        this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (button_states_[5] && axis_values_.count(6) && axis_values_[6] > 0.5f)
        this->control.SetGamepad(Input::Gamepad::RB_DPadRight);

    // Analog sticks for movement control
    // Left stick: axis 0 (X), axis 1 (Y)
    // Right stick: axis 3 (X), axis 4 (Y)
    // Note: axis 2 and 5 are triggers (LT/RT), default at -1.0
    if (axis_values_.count(0) && axis_values_.count(1))
    {
        this->control.y = -axis_values_[0]; // Left stick X
        this->control.x = -axis_values_[1]; // Left stick Y (forward/backward)
    }
    if (axis_values_.count(3))
    {
        this->control.yaw = -axis_values_[3]; // Right stick X (yaw)
    }
}

void RL_Real::ShutdownJoystick()
{
    if (joystick_fd_ >= 0)
    {
        close(joystick_fd_);
        std::cout << LOGGER::INFO << "Joystick closed" << std::endl;
    }
}

// ==================== Velocity Estimation ====================

void RL_Real::UpdateVelocityEstimation()
{
    // 改进的速度估计算法：使用IMU加速度积分（带重力补偿和漂移抑制）
    auto current_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_vel_update_time_).count();
    last_vel_update_time_ = current_time;

    if (dt > 0.01f || dt < 0.0001f)
        return; // 跳过异常时间间隔

    // 获取四元数，用于重力补偿
    float qw = imu_state_buffer.quaternion[0];
    float qx = imu_state_buffer.quaternion[1];
    float qy = imu_state_buffer.quaternion[2];
    float qz = imu_state_buffer.quaternion[3];

    // 计算重力在body系中的方向（世界坐标系 [0, 0, -1] 旋转到body系）
    float gravity_body_x = -2.0f * (qx * qz - qw * qy);
    float gravity_body_y = -2.0f * (qy * qz + qw * qx);
    float gravity_body_z = -(1.0f - 2.0f * (qx * qx + qy * qy));

    // 重力补偿：加速度计测量的是比力（specific force = a - g）
    // 真实加速度 = 测量值 - gravity_body * g
    const float g = 9.81f; // 重力加速度
    float ax_compensated = imu_state_buffer.accelerometer[0] - gravity_body_x * g;
    float ay_compensated = imu_state_buffer.accelerometer[1] - gravity_body_y * g;
    float az_compensated = imu_state_buffer.accelerometer[2] + 9.8 - gravity_body_z * g;

    // 计算角速度和补偿后加速度的幅值
    float gyro_norm = std::sqrt(
        imu_state_buffer.gyroscope[0] * imu_state_buffer.gyroscope[0] +
        imu_state_buffer.gyroscope[1] * imu_state_buffer.gyroscope[1] +
        imu_state_buffer.gyroscope[2] * imu_state_buffer.gyroscope[2]);
    float acc_norm = std::sqrt(
        ax_compensated * ax_compensated +
        ay_compensated * ay_compensated +
        az_compensated * az_compensated);

    // 静止检测：加速度和角速度都很小时，施加强衰减抑制漂移
    float decay_factor = 0.95f; // 默认衰减
    if (acc_norm < 0.5f && gyro_norm < 0.1f)
    {
        // 机器人可能静止，强烈衰减速度以抑制漂移
        decay_factor = 0.7f;
    }

    // 速度积分（梯形积分 + 衰减滤波）
    // v(t+dt) = decay * v(t) + 0.5 * (a(t) + a(t-dt)) * dt
    estimated_velocity_[0] = decay_factor * estimated_velocity_[0] +
                             0.5f * (ax_compensated + last_accelerometer_[0]) * dt;
    estimated_velocity_[1] = decay_factor * estimated_velocity_[1] +
                             0.5f * (ay_compensated + last_accelerometer_[1]) * dt;
    estimated_velocity_[2] = decay_factor * estimated_velocity_[2] +
                             0.5f * (az_compensated + last_accelerometer_[2]) * dt;

    // 保存当前加速度供下次使用
    last_accelerometer_[0] = ax_compensated;
    last_accelerometer_[1] = ay_compensated;
    last_accelerometer_[2] = az_compensated;

    // 速度限幅（防止异常值）
    const float max_vel = 2.0f; // 最大速度 2.0 m/s
    for (int i = 0; i < 3; ++i)
    {
        if (estimated_velocity_[i] > max_vel)
            estimated_velocity_[i] = max_vel;
        else if (estimated_velocity_[i] < -max_vel)
            estimated_velocity_[i] = -max_vel;
    }
}

void RL_Real::PrintDebugInfo()
{
    debug_print_counter_++;
    if (debug_print_counter_ % 25 != 0)
        return;

    std::cout << "\n========== Debug Info (Episode: " << this->episode_length_buf << ") ==========\n";

    // IMU 数据
    std::cout << "IMU Data:\n";
    std::cout << "  Quaternion (w,x,y,z): ["
              << std::fixed << std::setprecision(3)
              << imu_state_buffer.quaternion[0] << ", "
              << imu_state_buffer.quaternion[1] << ", "
              << imu_state_buffer.quaternion[2] << ", "
              << imu_state_buffer.quaternion[3] << "]\n";
    std::cout << "  Gyroscope (rad/s):    ["
              << imu_state_buffer.gyroscope[0] << ", "
              << imu_state_buffer.gyroscope[1] << ", "
              << imu_state_buffer.gyroscope[2] << "]\n";
    std::cout << "  Accelerometer (m/s²): ["
              << imu_state_buffer.accelerometer[0] << ", "
              << imu_state_buffer.accelerometer[1] << ", "
              << imu_state_buffer.accelerometer[2] << "]\n";

    // Observation 数据
    std::cout << "\nObservation Data:\n";
    std::cout << "  base_quat:    ["
              << this->obs.base_quat[0] << ", "
              << this->obs.base_quat[1] << ", "
              << this->obs.base_quat[2] << ", "
              << this->obs.base_quat[3] << "]\n";
    std::cout << "  ang_vel:      ["
              << this->obs.ang_vel[0] << ", "
              << this->obs.ang_vel[1] << ", "
              << this->obs.ang_vel[2] << "]\n";
    std::cout << "  lin_vel (est):["
              << this->obs.lin_vel[0] << ", "
              << this->obs.lin_vel[1] << ", "
              << this->obs.lin_vel[2] << "]\n";
    std::cout << "  gravity_vec:  ["
              << this->obs.gravity_vec[0] << ", "
              << this->obs.gravity_vec[1] << ", "
              << this->obs.gravity_vec[2] << "]\n";
    std::cout << "  commands:     ["
              << this->obs.commands[0] << ", "
              << this->obs.commands[1] << ", "
              << this->obs.commands[2] << "]\n";

    std::cout << "===============================================\n"
              << std::endl;
}

// ==================== Main ====================

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::init(argc, argv, "rl_real_node");
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
#endif

    std::cout << LOGGER::INFO << "Starting RL_Real for el_4090..." << std::endl;

    RL_Real rl_real(argc, argv);

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::spin(rl_real.ros2_node);
    rclcpp::shutdown();
#else
    while (true)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif

    return 0;
}
