# Joystick_ROS_Airsim
ROS interface for manual steering the simulated quadcopter in Airsim

## Introduction

This repository is released to provide a manually controlled ROS interface. Part of the code comes from [Airsim](https://github.com/microsoft/AirSim) and [aerial_navigation_development_environment](https://github.com/caochao39/aerial_navigation_development_environment)

## Quick Start 
1. Download an AirSim environment from the [AirSim release page](https://github.com/Microsoft/AirSim/releases). Click any version of the releases and find the 'Assets' at the bottom of the page. Go to the 'AirSimNH/LinuxNoEditor' folder, open a terminal and launch the compiled AirSim.
```
./AirSimNH.sh -windowed
```
2. Build AirSim and AirSim ROS from [Airsim ros page](https://github.com/Microsoft/AirSim.git). Go to the 'AirSim/ros' folder, open another terminal and launch AirSim ROS Wrapper.
```
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch
```
3. Build this project, and launch our system in a third terminal,.
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.sh
roslaunch joystick_control system_airsim.launch
```

## Advanced

<div align="center"> <img src="/img/joystick.jpg" width = 500 height = 350 /> </div>

you can modify the code according to the buttons and axes of the joystick to meet your requirement. Joyful journey to steer :smiley:
