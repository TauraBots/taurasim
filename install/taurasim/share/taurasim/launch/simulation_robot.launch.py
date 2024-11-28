from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('taurasim')
    
    # Set the world file path using PathJoinSubstitution
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'vss_field.world'])
    
    # Path to the launch files
    gazebo_launch = PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
    spawn_robot_launch = PathJoinSubstitution([pkg_share, 'launch', 'spawn_robot.launch.py'])

    # Return the LaunchDescription
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'world', 
            default_value=world_path,
            description='Path to the world file'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='-0.25',
            description='X position of the robot'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0',
            description='Y position of the robot'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0',
            description='Yaw orientation of the robot'
        ),
        DeclareLaunchArgument(
            'twist_interface',
            default_value='true',
            description='Enable twist interface for the robot'
        ),
        DeclareLaunchArgument(
            'controller_config_file',
            default_value='',
            description='Controller configuration file'
        ),
        DeclareLaunchArgument(
            'ros_control_config_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'ros_control_config.yml']),
            description='ROS control configuration file'
        ),
        DeclareLaunchArgument(
            'keyboard',
            default_value='false',
            description='Enable keyboard control'
        ),

        # Include the spawn_robot.launch.py file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_robot_launch),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'twist_interface': LaunchConfiguration('twist_interface'),
                'controller_config_file': LaunchConfiguration('controller_config_file'),
                'ros_control_config_file': LaunchConfiguration('ros_control_config_file')
            }.items()
        ),

        # Include the gazebo.launch.py file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': LaunchConfiguration('world')
            }.items()
        ),
        # Launch the keyboard controller node if enabled
    ])
