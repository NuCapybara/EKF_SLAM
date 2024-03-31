# ME495 Sensing, Navigation and Machine Learning For Robotics
* Jialu Yu
* Winter 2022
# Package List
This repository has one helper Libraries(Non-ROS)
- turtlelib:  A library for handling transformations in SE(2) and other turtlebot/lidar-related math.

    - diff_drive - related to the Kinematics of wheeled mobile robots, and is part of the turtlelib package
    - frame_main - generate a user-specified vector in visualization with a svg file
    - geometry2d - utilities for two-dimensional geometric operations, such as comparing floating-point numbers, converting between degrees and radians, normalizing angles, and manipulating 2D points and vectors.
    - ekf_slam - perform Extended Kalman Filter algorithm 
    - lidar - perform the line to circle intersection algorithm
    - circle_fitting - perform circle fitting algorithm

This repository consists of several ROS packages 
- nuturtle_description - Visualize turtlebots with user-specified parameters
- nusim - Simulates a Turtlebot3 in an rviz environment, mimicing  the real turtlebot
- nuturtle_control - provides an odometry estimate for the turtlebot that can interface with either a simulated or real robot.
- nuslam - Performs EKF slam to generate a map of the environment.




## Demo Videos
### EKF Slam Simulation video (HW3)
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/a1ad52a8-5b35-4b64-8e63-6002c8d7f1ff" controls title="EKF Simulation"></video>

### Turtlebot simulation Video
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/08c6739b-bc16-438b-948c-263814212cb3" controls title="Simulation"></video>

### Turtlebot Performance
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/7f7e29e0-7a01-46e1-9e1d-21a7796f2f04" controls title="Real scene"></video>

