from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('taurasim')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'vss_field_camera.world'])
    # Declaração de argumentos
    declare_robots_per_team = DeclareLaunchArgument('robots_per_team', default_value='3')
    declare_model = DeclareLaunchArgument('model', default_value=PathJoinSubstitution([pkg_share, 'urdf', 'generic_vss_robot.xacro']))
    declare_debug = DeclareLaunchArgument('debug', default_value='false')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_paused = DeclareLaunchArgument('paused', default_value='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world_name = DeclareLaunchArgument('world', default_value='vss_field_camera.world')
    declare_recording = DeclareLaunchArgument('recording', default_value='false')
    declare_twist_interface = DeclareLaunchArgument('twist_interface', default_value='true')
    declare_sound = DeclareLaunchArgument('sound', default_value='true')
    declare_keyboard = DeclareLaunchArgument('keyboard', default_value='true')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': world_path,
            'recording': LaunchConfiguration('recording')
        }.items()
    )

    # Grupo de nós para o controle por teclado
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

    # Incluir o script spawn_robots.py para lançar os robôs do time amarelo
    yellow_team_spawner = Node(
        package='taurasim',
        executable='spawn_robots',
        name='yellow_team_spawner',
        output='screen',
        parameters=[
            {'team_color': 'yellow'},
            {'robots_per_team': LaunchConfiguration('robots_per_team')},
            {'model': LaunchConfiguration('model')},
            {'sound': LaunchConfiguration('sound')},
        ]
    )

    blue_team_spawner = Node(
        package='taurasim',
        executable='spawn_robots',
        name='blue_team_spawner',
        output='screen',
        parameters=[
            {'team_color': 'blue'},
            {'robots_per_team': LaunchConfiguration('robots_per_team')},
            {'model': LaunchConfiguration('model')},
            {'sound': LaunchConfiguration('sound')},
        ]
    )



    return LaunchDescription([
        declare_robots_per_team,
        declare_model,
        declare_debug,
        declare_gui,
        declare_paused,
        declare_use_sim_time,
        declare_world_name,
        declare_recording,
        declare_twist_interface,
        declare_sound,
        declare_keyboard,
        keyboard_group,
        gazebo_launch,
        yellow_team_spawner,
        blue_team_spawner
    ])