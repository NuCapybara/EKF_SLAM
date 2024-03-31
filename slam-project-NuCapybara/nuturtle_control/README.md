# Nuturtle Control

## Overview

The Nuturtle Control package provides control algorithms and utilities for controlling the turtlebot-waffle. It is designed to work in conjunction with other packages in the SLAM project.

## Features

- Odometry estimation using wheel encoders.
- ROS interface for sending control commands and receiving sensor data.
- Functions to make the turtlebot go in circulation.

## Launch files
- To control the real turtlebot
```ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=circle```
- To just launch the blue robot in rviz while the localhost is is also launched
```ros2 launch nuturtle_control start_robot.launch.xml robot:=none cmd_src:=none```
- To launch only the simulated robot
```ros2 launch nuturtle_control start_robot.launch.xml```


