# Taurasim

## About
The TauraSim package provides a simulation environment for VSSS robots.
It includes configurations for robot models, controllers, and simulation worlds.

## Requirements
1. Gazebo-11  
   [Tutorial Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
2. ROS2  
   [Tutorial ROS2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

## Install Dependencies

```bash
mkdir taurasim_ws
cd ./taurasim_ws
cd ./src
git clone https://github.com/TauraBots/taurasim.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
```
## Build

```bash
colcon build
source ./install/setup.bash
```

## Launch Example

```bash
ros2 launch taurasim simulation_robot.launch.py
```

