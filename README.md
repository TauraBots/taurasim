# Taurasim

## About
The TauraSim package provides a simulation environment for VSSS robots.
It includes configurations for robot models, controllers, and simulation worlds.


## Requirements
1. Gazebo-11
2. Ros2

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

## Control test
```bash
ros2 topic pub /yellow_team/robot_0/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```


