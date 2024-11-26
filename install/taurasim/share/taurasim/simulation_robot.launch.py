from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('taurasim')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'vss_field.world'])

    gazebo_launch= PathJoinSubstitution([pkg_share, 'worlds', 'gazebo.launch.py'])
    
    vision_proxy = PathJoinSubstitution([pkg_share, 'scripts', 'vision_proxy.py'])
    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path
        ),
        DeclareLaunchArgument(
            name='x',
            default_value='-0.25'
        ),
        DeclareLaunchArgument(
            name='y',
            default_value='0'
        ),
        DeclareLaunchArgument(
            name='yaw',
            default_value='0'
        ),
        DeclareLaunchArgument(
            name='twist_interface',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='controller_config_file',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='ros_control_config_file',
            default_value='$(find-pkg-share taurasim)/config/ros_control_config.yml'
        ),
        DeclareLaunchArgument(
            name='keyboard',
            default_value='false'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_path
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource
        ),
  
    ])
