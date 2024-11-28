from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, LogInfo)
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = get_package_share_directory('taurasim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'vss_field.world'])

    gzserver_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])
    gzclient_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])
    
    gazebo_model_path = PathJoinSubstitution([pkg_share, 'models'])
    gazebo_resource_path = PathJoinSubstitution(pkg_share)
    
    vision_proxy = PathJoinSubstitution([pkg_share, 'scripts', 'vision_proxy.py'])
    return LaunchDescription([
        PushRosNamespace('vision'),
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value=[EnvironmentVariable(name="GAZEBO_RESOURCE_PATH"), ':', gazebo_resource_path]
        ),
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[EnvironmentVariable(name="GAZEBO_MODEL_PATH"), ':', gazebo_model_path]
        ),
        LogInfo(
    condition=None,
    msg=EnvironmentVariable('GAZEBO_MODEL_PATH'),
),
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path
        ),
        DeclareLaunchArgument(
            name='version',
            default_value='false'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'version': LaunchConfiguration('version')
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch),
        ),
        Node(
            package='taurasim',
            executable='vision_proxy',
            name='vision_proxy_node',
            output='screen',
            parameters=[
                {
                    'disable_pub_plugins': ['image_transport/compressedDepth'],
                    '/vision/std_dev': 0.1
                }
            ]
        )
    ])
