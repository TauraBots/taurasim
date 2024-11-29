from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('taurasim')

    # Declaração de argumentos
    declare_ros_control_config = DeclareLaunchArgument('ros_control_config', default_value=PathJoinSubstitution([pkg_share, 'config', 'ros_control_config.yml']))
    declare_robot_number = DeclareLaunchArgument('robot_number', default_value='0')
    declare_is_yellow = DeclareLaunchArgument('is_yellow', default_value='true')
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='yellow_team/robot_0')
    declare_x = DeclareLaunchArgument('x', default_value='0')
    declare_y = DeclareLaunchArgument('y', default_value='0')
    declare_z = DeclareLaunchArgument('z', default_value='0.03')
    declare_row = DeclareLaunchArgument('row', default_value='0')
    declare_pitch = DeclareLaunchArgument('pitch', default_value='0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0')
    declare_model = DeclareLaunchArgument('model', default_value=PathJoinSubstitution([pkg_share, 'urdf', 'generic_vss_robot.xacro']))
    declare_namespace = DeclareLaunchArgument('namespace', default_value='yellow_team/robot_0')
    declare_twist_interface = DeclareLaunchArgument('twist_interface', default_value='true')
    declare_controller_config_file = DeclareLaunchArgument('controller_config_file', default_value='')

    # Comando para gerar o `robot_description` como string a partir do XACRO
    robot_description_content = Command([
        'xacro',
        ' ',
        LaunchConfiguration('model'),
        ' ',
        'robot_number:=', LaunchConfiguration('robot_number'),
        ' ',
        'is_yellow:=', LaunchConfiguration('is_yellow')
    ])


    # Nodo que publica o `robot_description`
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{'robot_description': robot_description_content}],
    )
    # Nodo para spawn do robô no Gazebo com o URDF gerado
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('row'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
            '--ros-args', '-r', '__node:=urdf_spawner'
        ],
        output='screen'
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {LaunchConfiguration('ros_control_config')},
            {'robot_description': robot_description_content}
        ]
    )

    # IMPLEMENTAR
    load_joint_broad_config = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    load_diff_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_controller', '--param-file', PathJoinSubstitution([pkg_share, 'config', 'ros_control_config.yml'])],
        output='screen'
    )

    load_direct_drive_config = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'left_controller', 'right_controller', "--param-file", LaunchConfiguration('ros_control_config')
        ],
        condition=UnlessCondition(LaunchConfiguration('twist_interface')),
        output='screen'
    )

    return LaunchDescription([
        declare_twist_interface,
        declare_controller_config_file,
        declare_ros_control_config,
        declare_robot_number,
        declare_is_yellow,
        declare_robot_name,
        declare_x,
        declare_y,
        declare_z,
        declare_row,
        declare_pitch,
        declare_yaw,
        declare_model,
        declare_namespace,
        robot_state_publisher,
        spawn_robot,
    ])
