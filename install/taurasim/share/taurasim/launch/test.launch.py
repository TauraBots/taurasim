from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, LogInfo)
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
def generate_launch_description():
    pkg_share = get_package_share_directory('taurasim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    gzserver_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])
    gzclient_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])
    
    gazebo_model_path = PathJoinSubstitution([pkg_share, 'models'])
    gazebo_resource_path = PathJoinSubstitution(pkg_share)
    
    vision_proxy = PathJoinSubstitution([pkg_share, 'scripts', 'vision_proxy.py'])
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value=[EnvironmentVariable(name="GAZEBO_RESOURCE_PATH"), ':', gazebo_resource_path]
        ),
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[EnvironmentVariable(name="GAZEBO_MODEL_PATH"), ':', gazebo_model_path]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch),
        )
    ])
