### Simulation

#### Gazebo

Open a terminal, launch the gazebo simulation environment

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=<ROBOT>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
```

Open a new terminal, launch the control program

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 run rl_sar rl_sim
```

> [!TIP]
> If you cannot see the robot after launching Gazebo in Ubuntu 22.04, it means the robot was initialized outside the field of view. The robot's position will be automatically reset after launching rl_sim. If the robot falls over during the standing process, use the keyboard `R` or the gamepad `RB+Y` to reset the robot.

If Gazebo cannot be opened when you start it for the first time, you need to download the model package

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

#### Mujoco

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
./cmake_build/bin/rl_sim_mujoco el4 scene
```

## Add Your Robot

The following uses **\<ROBOT\>/\<CONFIG\>** to represent your robot environment. You only need to create or modify the following files, and the names must exactly match those shown below. (You can refer to the corresponding files in go2w as examples.)

```yaml
# your robot description
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros1.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros2.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/robot.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/gazebo.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control.yaml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control_ros2.yaml

# your policy
policy/<ROBOT>/base.yaml  # This file must follow the physical robot's joint order
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # for libtorch, note that exporting JIT is required
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # for onnxruntime

# fsm for robot
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# your real robot code
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # You can customize the forward() function as needed to adapt to your policy
```