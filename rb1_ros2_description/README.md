This project provides a simple example of how to create and configure controls for an RB1 robot using ROS 2. It demonstrates both base movement and elevator control functionalities.

Introduction
This package includes example configurations and launch files for controlling an RB1 robot in simulation. The examples highlight the integration of ROS 2 controllers for differential drive and an elevator platform.

Disclaimer:
The ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py command may occasionally require running twice due to initialization quirks in Gazebo.
Use ctrl c to close the program and run the launch again.

Build and source the workspace:

cd ~/ros2_ws
colcon build
source install/setup.bash

Getting Started
Launching the Simulation
To launch the RB1 robot in a Gazebo simulation, use the following command:

ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py


Verifying the Robot
After launching, you can check the active controllers using:

ros2 control list_controllers
You should see the following controllers in an active state:

joint_state_broadcaster
rb1_base_controller
forward_position_controller


Controlling the RB1 Robot
Moving the Robot Base

Turn the Robot:
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

Move Forward:
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Stop the Robot:
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


Elevator Control
The elevator controller is automatically activated at launch. If for any reason it is not active, follow the instructions below to activate or deactivate it manually.

Activating/Deactivating the Elevator Controller

Activate the Elevator Controller:
ros2 control set_controller_state forward_position_controller start

Deactivate the Elevator Controller:
ros2 control set_controller_state forward_position_controller stop


Moving the Elevator

Move Elevator Up:
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.2]"

Move Elevator Down:
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]"

Notes and Troubleshooting
Effort Stability for Elevator:
To ensure the elevator remains stable and doesn't drop due to gravity, an effort is automatically applied at launch using the apply_joint_effort service. This holds the elevator steady while the position controller is active.


## Disclaimer:  
This package only modifies/adapts files from these repositories/packages:  
- [RobotnikAutomation/rb1_base_sim](https://github.com/RobotnikAutomation/rb1_base_sim) licensed under the BSD 2-Clause "Simplified" License
- [RobotnikAutomation/rb1_base_common/rb1_base_description](https://github.com/RobotnikAutomation/rb1_base_common/tree/melodic-devel/rb1_base_description), licensed under the BSD License
- [RobotnikAutomation/robotnik_sensors],(https://github.com/RobotnikAutomation/robotnik_sensors) licensed under the BSD License