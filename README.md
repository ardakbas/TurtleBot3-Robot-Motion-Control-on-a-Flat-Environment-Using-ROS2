# TurtleBot3-Robot-Motion-Control-on-a-Flat-Environment-Using-ROS2
This project focuses on controlling a TurtleBot3 robot in a simulated flat environment using ROS2. The robot is programmed to autonomously navigate to a specified target point based on odometry feedback. The system demonstrates basic autonomous motion control, publishing velocity commands to guide the robot accurately to the goal without obstacles.
# Necessary Libraries
rclpy is a Python library that provides a ROS2 client interface, allowing developers to create nodes, publishers, subscribers, and other ROS2 functionalities in Python.
geometry_msgs is a ROS2 package that defines message types for geometric data, such as the Twist message used on the /cmd_vel topic. 
nav_msgs is a ROS2 package that defines message types for navigation data, such as the Odometry message used on the /odom topic.
math is a library which contains a lot of mathematical functions.
# Usage
Firstly, "~/ROS_Project/install/setup.bash" path must be sourced to the terminals that package will be used. 
Secondly, the robot type should be exported to the terminals.
After that, flat environment should be launch with the command of "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
Finally, "ros2 run robot_control controller_node" code should be sent to terminal.
As a consequence, robot will go to target point (1,1) automatically.
# Limitations
This code is written for ROS2 Humble. Running it on other ROS2 distributions may cause errors.
