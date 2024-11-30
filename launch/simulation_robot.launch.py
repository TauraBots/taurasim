from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get Paths 
    pkg_share = get_package_share_directory('taurasim')

    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'vss_field_camera.world'])
    gazebo_launch_path = PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])
    spawn_robot_launch_path = PathJoinSubstitution([pkg_share, 'launch', 'spawn_robot.launch.py'])

    # Declare launch arguments
    declare_world = DeclareLaunchArgument(
            'world', 
            default_value=world_path,
            description='Path to the world file'
    )
    declare_robot_x = DeclareLaunchArgument(
            'x',
            default_value='-0.25',
            description='X position of the robot'
    )
    declare_robot_y = DeclareLaunchArgument(
            'y',
            default_value='0',
            description='Y position of the robot'
    )
    declare_robot_yaw = DeclareLaunchArgument(
            'yaw',
            default_value='0',
            description='Yaw orientation of the robot'
    )
    declare_robot_twist = DeclareLaunchArgument(
            'twist_interface',
            default_value='true',
            description='Enable twist interface for the robot'
    )
    declare_controller_file = DeclareLaunchArgument(
            'controller_config_file',
            default_value='',
            description='Controller configuration file'
    )
    declare_ros_control_Config = DeclareLaunchArgument(
            'ros_control_config_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'ros_control_config.yml']),
            description='ROS control configuration file'
    )
    declare_keyboard = DeclareLaunchArgument(
            'keyboard',
            default_value='false',
            description='Enable keyboard control'
    )

    rqt_robot_steering_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        parameters=[{'default_topic': '/yellow_team/robot_0/diff_drive_controller/cmd_vel'}],
        output='screen',
        arguments=['--force-discover']
    )
    spawn_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_robot_launch_path),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'twist_interface': LaunchConfiguration('twist_interface'),
                'controller_config_file': LaunchConfiguration('controller_config_file'),
                'ros_control_config_file': LaunchConfiguration('ros_control_config_file')
            }.items()
    )
    
    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world': world_path
            }.items()
    )

    return LaunchDescription([

        declare_world,
        declare_robot_x,
        declare_robot_y,
        declare_robot_yaw,
        declare_robot_twist,
        declare_controller_file,
        declare_ros_control_Config,
        declare_keyboard,
        gazebo_launch,
        spawn_robot_launch,
        rqt_robot_steering_node
    ])
