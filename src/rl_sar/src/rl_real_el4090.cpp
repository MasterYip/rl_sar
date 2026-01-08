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
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "el_4090";
    this->ReadYaml(this->robot_name, "base.yaml");

    // get num_dofs
    this->num_dofs = this->params.Get<int>("num_of_dofs");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
            std::cout << LOGGER::INFO << "FSM loaded for " << this->robot_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->num_dofs);
    this->InitOutputs();
    this->InitControl();

    // Initialize motor state buffers
    motor_state_buffer.position.resize(num_dofs, 0.0f);
    motor_state_buffer.velocity.resize(num_dofs, 0.0f);
    motor_state_buffer.torque.resize(num_dofs, 0.0f);
    motor_state_buffer.temperature.resize(num_dofs, 0.0f);

    // Initialize motor command buffers
    motor_command_buffer.target_position.resize(num_dofs, 0.0f);
    motor_command_buffer.target_velocity.resize(num_dofs, 0.0f);
    motor_command_buffer.kp.resize(num_dofs, 0.0f);
    motor_command_buffer.kd.resize(num_dofs, 0.0f);
    motor_command_buffer.feedforward_torque.resize(num_dofs, 0.0f);

    // Initialize IMU state
    imu_state_buffer.quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
    imu_state_buffer.gyroscope = {0.0f, 0.0f, 0.0f};
    imu_state_buffer.accelerometer = {0.0f, 0.0f, 9.81f}; // Gravity in z-direction

    // Initialize mapped joint arrays
    mapped_joint_positions.resize(num_dofs, 0.0f);
    mapped_joint_velocities.resize(num_dofs, 0.0f);

    // Initialize EtherCAT motor control
    // Get network interface name from command line or use default
    if (argc > 1) {
        ethercat_ifname_ = argv[1];
    } else {
        ethercat_ifname_ = "enp3s0";  // Default interface
        std::cout << LOGGER::WARNING << "No network interface specified, using default: " << ethercat_ifname_ << std::endl;
        std::cout << LOGGER::INFO << "Usage: " << argv[0] << " <network_interface>" << std::endl;
    }
    
    // Initialize motor ID mapping
    InitMotorMapping();
    
    // Load motor calibration offsets
    std::string calib_path = this->robot_name + "/motor_calibration.yaml";
    LoadMotorCalibration(calib_path);
    
    // Initialize EtherCAT
    if (!InitEtherCAT(ethercat_ifname_.c_str())) {
        std::cout << LOGGER::ERROR << "Failed to initialize EtherCAT on interface: " << ethercat_ifname_ << std::endl;
        throw std::runtime_error("EtherCAT initialization failed");
    }
    
    std::cout << LOGGER::INFO << "EtherCAT initialized successfully on " << ethercat_ifname_ << std::endl;

    // loop
    this->loop_hardware_recv = std::make_shared<LoopFunc>("loop_hardware_recv", 0.002, std::bind(&RL_Real::HardwareRecv, this), 3);
    this->loop_hardware_send = std::make_shared<LoopFunc>("loop_hardware_send", 0.002, std::bind(&RL_Real::HardwareSend, this), 3);
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    
    this->loop_hardware_recv->start();
    this->loop_hardware_send->start();
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->num_dofs);
    this->plot_target_joint_pos.resize(this->num_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif

#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Real initialized for " << this->robot_name << " (" << this->num_dofs << " DOFs)" << std::endl;
}

RL_Real::~RL_Real()
{
    this->loop_hardware_recv->shutdown();
    this->loop_hardware_send->shutdown();
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif

    // Shutdown EtherCAT communication
    ShutdownEtherCAT();
    std::cout << LOGGER::INFO << "EtherCAT interface shutdown" << std::endl;

    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float> *state)
{
    // TODO: Read gamepad/joystick input from hardware
    // For now, only keyboard is supported via KeyboardInterface()
    
    // Copy IMU state from buffer
    state->imu.quaternion[0] = imu_state_buffer.quaternion[0]; // w
    state->imu.quaternion[1] = imu_state_buffer.quaternion[1]; // x
    state->imu.quaternion[2] = imu_state_buffer.quaternion[2]; // y
    state->imu.quaternion[3] = imu_state_buffer.quaternion[3]; // z

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = imu_state_buffer.gyroscope[i];
    }

    // Apply joint mapping and copy motor states
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < this->num_dofs; ++i)
    {
        int mapped_idx = joint_mapping[i];
        state->motor_state.q[i] = motor_state_buffer.position[mapped_idx];
        state->motor_state.dq[i] = motor_state_buffer.velocity[mapped_idx];
        state->motor_state.tau_est[i] = motor_state_buffer.torque[mapped_idx];
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    // Apply joint mapping and copy motor commands to buffer
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < this->num_dofs; ++i)
    {
        int mapped_idx = joint_mapping[i];
        motor_command_buffer.target_position[mapped_idx] = command->motor_command.q[i];
        motor_command_buffer.target_velocity[mapped_idx] = command->motor_command.dq[i];
        motor_command_buffer.kp[mapped_idx] = command->motor_command.kp[i];
        motor_command_buffer.kd[mapped_idx] = command->motor_command.kd[i];
        motor_command_buffer.feedforward_torque[mapped_idx] = command->motor_command.tau[i];
    }

    // TODO: Add safety checks here (optional but recommended)
    // - Check position limits
    // - Check velocity limits
    // - Check torque limits
    // - Temperature protection
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);

#ifdef CSV_LOGGER
    this->LogToCSV();
#endif
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
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

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::HardwareSend()
{
    // Send motor commands to hardware via EtherCAT
    for (int i = 0; i < num_dofs; ++i)
    {
        // Get motor location from DOF index
        // Assuming motor_location_map_ is indexed by motor_id
        // and motor IDs correspond to DOF indices + 1 (1-18)
        int motor_id = i + 1;  // Motor IDs are 1-based
        
        auto it = motor_location_map_.find(motor_id);
        if (it != motor_location_map_.end()) {
            int slave = it->second.slave;
            int passage = it->second.passage;
            
            // 转换关系:
            // motor_command_buffer.target_position[i] = URDF 中的目标关节角度
            // offset = URDF零位时对应的电机位置
            // motor_physical_position = urdf_angle + offset
            // 
            // 例如：如果要让 URDF 关节到 0.5 rad，offset = 0.523
            // 则发送给电机的命令 = 0.5 + 0.523 = 1.023 rad
            float command_pos = GetCommandAngle(motor_id, motor_command_buffer.target_position[i]);
            
            // Send mixed control command (position + velocity + torque with PD gains)
            SendMotorCommand(
                slave,
                passage,
                motor_id,
                motor_command_buffer.kp[i],
                motor_command_buffer.kd[i],
                command_pos,
                motor_command_buffer.target_velocity[i],
                motor_command_buffer.feedforward_torque[i]
            );
        }
    }
}

void RL_Real::HardwareRecv()
{
    // Read motor states from hardware via EtherCAT
    for (int i = 0; i < num_dofs; ++i)
    {
        int motor_id = i + 1;  // Motor IDs are 1-based
        
        auto it = motor_location_map_.find(motor_id);
        if (it != motor_location_map_.end()) {
            int slave = it->second.slave;
            int passage = it->second.passage;
            
            float position, velocity, torque;
            ReadMotorStatus(slave, passage, motor_id, position, velocity, torque);
            
            // 转换关系:
            // position = 电机物理位置（弧度）
            // offset = URDF零位时对应的电机位置（从 motor_calibration.yaml 加载）
            // urdf_angle = position - offset
            // 
            // 例如：如果 offset = 0.523，电机当前位置 = 1.047
            // 则 URDF 关节角度 = 1.047 - 0.523 = 0.524 rad
            float offset = motor_offsets_.count(motor_id) ? motor_offsets_[motor_id] : 0.0f;
            motor_state_buffer.position[i] = position - offset;
            motor_state_buffer.velocity[i] = velocity;
            motor_state_buffer.torque[i] = torque;
        }
    }
    
    // TODO: Read IMU data if available
    // For now, keep default IMU values or implement IMU reading separately
}

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

#ifdef CSV_LOGGER
void RL_Real::LogToCSV()
{
    // Log current state and command to CSV file
    if (csv_file.is_open())
    {
        csv_file << this->episode_length_buf;
        
        // Log motor positions
        for (int i = 0; i < this->num_dofs; ++i)
        {
            csv_file << "," << this->robot_state.motor_state.q[i];
        }
        
        // Log motor velocities
        for (int i = 0; i < this->num_dofs; ++i)
        {
            csv_file << "," << this->robot_state.motor_state.dq[i];
        }
        
        // Log motor torques
        for (int i = 0; i < this->num_dofs; ++i)
        {
            csv_file << "," << this->robot_state.motor_state.tau_est[i];
        }
        
        // Log target positions
        for (int i = 0; i < this->num_dofs; ++i)
        {
            csv_file << "," << this->robot_command.motor_command.q[i];
        }
        
        csv_file << std::endl;
    }
}
#endif

// ========== EtherCAT Implementation ==========

bool RL_Real::InitEtherCAT(const char* ifname)
{
    std::cout << LOGGER::INFO << "Initializing EtherCAT on interface: " << ifname << std::endl;
    
    // Initialize EtherCAT master
    EtherCAT_Init((char*)ifname);
    
    // Check if slaves are found
    if (ec_slavecount <= 0) {
        std::cout << LOGGER::ERROR << "No EtherCAT slaves found!" << std::endl;
        return false;
    }
    
    std::cout << LOGGER::INFO << "Found " << ec_slavecount << " EtherCAT slave(s)" << std::endl;
    
    if (ec_slavecount != SLAVE_COUNT) {
        std::cout << LOGGER::WARNING << "Expected " << SLAVE_COUNT << " slaves, found " << ec_slavecount << std::endl;
    }
    
    // Start EtherCAT communication threads
    ethercatManager.startThreads();
    
    std::cout << LOGGER::INFO << "EtherCAT threads started" << std::endl;
    
    // Wait for communication to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    return true;
}

void RL_Real::ShutdownEtherCAT()
{
    std::cout << LOGGER::INFO << "Shutting down EtherCAT..." << std::endl;
    
    // Stop communication threads
    ethercatManager.stopThreads();
    
    std::cout << LOGGER::INFO << "EtherCAT threads stopped" << std::endl;
}

void RL_Real::InitMotorMapping()
{
    // Motor ID mapping based on your Python script
    // _slave_index_to_id = [[7, 8, 9, 1, 2, 3], 
    //                       [13, 17, 18, 16, 14, 15],
    //                       [4, 5, 6, 10, 11, 12]]
    
    motor_id_map_[0] = {7, 8, 9, 1, 2, 3};
    motor_id_map_[1] = {13, 17, 18, 16, 14, 15};
    motor_id_map_[2] = {4, 5, 6, 10, 11, 12};
    
    // Build reverse mapping: motor_id -> [slave, passage]
    for (int slave = 0; slave < SLAVE_COUNT; ++slave) {
        for (int passage = 0; passage < PASSAGE_PER_SLAVE; ++passage) {
            int motor_id = motor_id_map_[slave][passage];
            if (motor_id > 0) {
                motor_location_map_[motor_id] = {slave, passage + 1};  // passage is 1-based
            }
        }
    }
    
    std::cout << LOGGER::INFO << "Motor ID mapping initialized for " << motor_location_map_.size() << " motors" << std::endl;
}

void RL_Real::LoadMotorCalibration(const std::string& yaml_path)
{
    std::string full_path = yaml_path;
    
    // 标定文件说明:
    // motor_calibration.yaml 存储的是 URDF 零位时对应的电机物理位置（弧度）
    // 例如: motor_id: 1, offset: 0.523 表示当电机读数为0.523时，对应URDF中该关节角度为0
    // 
    // 转换关系:
    // URDF关节角度 = 电机物理位置 - 标定偏移
    // 电机目标位置 = URDF目标角度 + 标定偏移
    
    try {
        YAML::Node calib = YAML::LoadFile(full_path);
        
        for (auto it = calib.begin(); it != calib.end(); ++it) {
            int motor_id = it->first.as<int>();
            float offset = it->second.as<float>();
            motor_offsets_[motor_id] = offset;
        }
        
        std::cout << LOGGER::INFO << "Loaded motor calibration offsets for " << motor_offsets_.size() << " motors" << std::endl;
        std::cout << LOGGER::INFO << "Calibration loaded: URDF joint angle = motor position - offset" << std::endl;
    } catch (const std::exception& e) {
        std::cout << LOGGER::WARNING << "Could not load motor calibration file: " << full_path << std::endl;
        std::cout << LOGGER::WARNING << "Using zero offsets. Error: " << e.what() << std::endl;
        
        // Initialize with zero offsets
        for (const auto& pair : motor_location_map_) {
            motor_offsets_[pair.first] = 0.0f;
        }
    }
}

float RL_Real::GetCommandAngle(int motor_id, float target_angle)
{
    // target_angle 是 URDF 中的关节角度
    // 需要转换为电机物理位置: motor_position = urdf_angle + offset
    // 其中 offset 是 URDF 零位时对应的电机位置
    float offset = motor_offsets_.count(motor_id) ? motor_offsets_[motor_id] : 0.0f;
    return target_angle + offset;
}

void RL_Real::SendMotorCommand(int slave, int passage, int motor_id, float kp, float kd, float pos, float spd, float tor)
{
    // motor_mixed_control 模式:
    // 1. 获取当前 slave 的 TX 消息
    // 2. 修改对应 passage 的电机命令
    // 3. 将修改后的消息设置回去
    EtherCAT_Msg tx_msg = motorData.getTxMsg(slave);
    send_motor_ctrl_cmd(&tx_msg, passage, motor_id, kp, kd, pos, spd, tor);
    motorData.setTxMsg(slave, tx_msg);
}

void RL_Real::ReadMotorStatus(int slave, int passage, int motor_id, float& position, float& velocity, float& torque)
{
    // Read motor status from RX message
    OD_Motor_Msg motor_msg = motorData.getRxMotorMsg(slave, passage);
    
    // Verify motor ID matches
    if (motor_msg.motor_id == motor_id) {
        // motor_msg.angle_actual_float 是电机的物理位置（弧度）
        // 返回的是原始电机位置，在 HardwareRecv() 中会减去标定偏移转换为 URDF 角度
        position = motor_msg.angle_actual_float;
        velocity = motor_msg.speed_actual_float;
        torque = motor_msg.current_actual_float;  // Current as proxy for torque
    } else {
        // Motor ID mismatch or no data yet
        position = 0.0f;
        velocity = 0.0f;
        torque = 0.0f;
    }
}

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
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    return 0;
}
