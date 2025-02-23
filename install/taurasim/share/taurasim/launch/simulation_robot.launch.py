from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

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
    declare_keyboard = DeclareLaunchArgument(
            'keyboard',
            default_value='true',
            description='Enable keyboard control'
    )
    declare_robots = DeclareLaunchArgument('ROBOTS', default_value='1', description='Number of robots')

    keyboard_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('keyboard')),
        actions=[
            Node(
                package='taurasim',
                executable='keyboard_node',
                name='keyboard_controller',
                output='screen'
            )
        ]
    )


    no_keyboard_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('keyboard')),
        actions=[
            Node(
                package='rqt_robot_steering',
                executable='rqt_robot_steering',
                name='rqt_robot_steering',
                parameters=[{'default_topic': '/yellow_team/robot_0/diff_drive_controller/cmd_vel'}],
                output='screen',
                arguments=['--force-discover']
            )
        ]
    )

    spawn_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_robot_launch_path),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'twist_interface': LaunchConfiguration('twist_interface'),
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
        declare_keyboard,
        gazebo_launch,
        spawn_robot_launch,
        keyboard_group,
        no_keyboard_group
    ])
