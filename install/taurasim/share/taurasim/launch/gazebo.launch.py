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
        DeclareLaunchArgument(
            'world', default_value='',
            description='Specify world file name'
        ),
        DeclareLaunchArgument(
            'version', default_value='false',
            description='Set "true" to output version information.'
        ),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Set "true" to increase messages written to terminal.'
        ),
        DeclareLaunchArgument(
            'lockstep', default_value='false',
            description='Set "true" to respect update rates'
        ),
        DeclareLaunchArgument(
            'help', default_value='false',
            description='Set "true" to produce gzserver help message.'
        ),
        DeclareLaunchArgument(
            'pause', default_value='false',
            description='Set "true" to start the server in a paused state.'
        ),
        DeclareLaunchArgument(
            'physics', default_value='',
            description='Specify a physics engine (ode|bullet|dart|simbody).'
        ),
        DeclareLaunchArgument(
            'play', default_value='',
            description='Play the specified log file.'
        ),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Set "true" to record state data.'
        ),
        DeclareLaunchArgument(
            'record_encoding', default_value='',
            description='Specify compression encoding format for log data (zlib|bz2|txt).'
        ),
        DeclareLaunchArgument(
            'record_path', default_value='',
            description='Absolute path in which to store state data.'
        ),
        DeclareLaunchArgument(
            'record_period', default_value='',
            description='Specify recording period (seconds).'
        ),
        DeclareLaunchArgument(
            'record_filter', default_value='',
            description='Specify recording filter (supports wildcard and regular expression).'
        ),
        DeclareLaunchArgument(
            'seed', default_value='', description='Start with a given a random number seed.'
        ),
        DeclareLaunchArgument(
            'iters', default_value='', description='Specify number of iterations to simulate.'
        ),
        DeclareLaunchArgument(
            'minimal_comms', default_value='false',
            description='Set "true" to reduce TCP/IP traffic output.'
        ),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='Specify physics preset profile name from the options in the world file.'
        ),
        DeclareLaunchArgument(
            'extra_gazebo_args', default_value='',
            description='Extra arguments to be passed to Gazebo'
        ),

        # Specific to gazebo_ros
        DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Set "true" to run gzserver with gdb'
        ),
        DeclareLaunchArgument(
            'valgrind', default_value='false',
            description='Set "true" to run gzserver with valgrind'
        ),
        DeclareLaunchArgument(
            'init', default_value='true',
            description='Set "false" not to load "libgazebo_ros_init.so"'
        ),
        DeclareLaunchArgument(
            'factory', default_value='true',
            description='Set "false" not to load "libgazebo_ros_factory.so"'
        ),
        DeclareLaunchArgument(
            'force_system', default_value='true',
            description='Set "false" not to load "libgazebo_ros_force_system.so"'
        ),
        DeclareLaunchArgument(
            'server_required', default_value='false',
            description='Set "true" to shut down launch script when server is terminated'
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
