import math
import argparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node as LaunchNode
import rclpy
from rclpy.node import Node

#!/usr/bin/env python3
# coding=utf-8

"""
    File:
        spawn_robots.py

    Description:
        Parametrized spawner for multiple robots in ROS2 with OOP
"""

PACKAGE_NAME = 'taurasim'
SPAWN_LAUNCHFILE = 'spawn_robot.launch.py'

COLORS = ["yellow", "blue"]

FORMATION_3X3 = {
    COLORS[0]: [(-0.2, 0), (-0.5, 0.3), (-0.5, -0.3)],
    COLORS[1]: [(0.2, 0), (0.5, 0.3), (0.5, -0.3)]
}

FORMATION_5X5 = {
    COLORS[0]: [(-0.2, 0), (-0.5, 0.3), (-0.5, -0.3), (-0.7, 0.5), (-0.7, -0.5)],
    COLORS[1]: [(0.2, 0), (0.5, 0.3), (0.5, -0.3), (0.7, 0.5), (0.7, -0.5)]
}

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.declare_parameter('team_color', 'blue')
        self.declare_parameter('robots_per_team', 3)
        self.team_color = self.get_parameter('team_color').get_parameter_value().string_value
        self.robots_per_team = self.get_parameter('robots_per_team').get_parameter_value().integer_value
        self.launch_service = LaunchService()

    def get_is_yellow(self, color):
        return "true" if color == "yellow" else "false"

    def get_yaw(self, color):
        return 0.0  if color == "yellow" else math.pi

    def spawn_robots(self):
        launch_files = []

        if self.robots_per_team == 3:
            formations = FORMATION_3X3
        elif self.robots_per_team == 5:
            formations = FORMATION_5X5
        else:
            raise ValueError('Invalid number of robots per team')

        for i in range(self.robots_per_team):
            launch_files.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([get_package_share_directory(PACKAGE_NAME), '/launch/', SPAWN_LAUNCHFILE]),
                    launch_arguments={
                        'namespace': f'{self.team_color}_team/robot_{i}',
                        'robot_number': str(i),
                        'is_yellow': str(self.get_is_yellow(self.team_color)),
                        'robot_name': f'{self.team_color}_team/robot_{i}',
                        'x': str(formations[self.team_color][i][0]),
                        'y': str(formations[self.team_color][i][1]),
                        'yaw': str(self.get_yaw(self.team_color))
                    }.items()
                )
            )

        for launch_file in launch_files:
            self.launch_service.include_launch_description(launch_file)

        self.launch_service.run()

def main(args=None):
    rclpy.init(args=args)
    robot_spawner = RobotSpawner()
    robot_spawner.spawn_robots()
    rclpy.spin(robot_spawner)
    robot_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()