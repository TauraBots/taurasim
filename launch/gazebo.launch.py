from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription)
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
def generate_launch_description():

    #Get Paths
    pkg_share = get_package_share_directory('taurasim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    gzserver_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])
    gzclient_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])
    
    gazebo_model_path = PathJoinSubstitution([pkg_share, 'models'])
    gazebo_resource_path = PathJoinSubstitution(pkg_share)

     # Set Environment Variables to find custom models and resources
    set_gazebo_resources = SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value=[EnvironmentVariable(name="GAZEBO_RESOURCE_PATH"), ':', gazebo_resource_path]
    )
    set_gazebo_model = SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[EnvironmentVariable(name="GAZEBO_MODEL_PATH"), ':', gazebo_model_path]
    )
    # Declare launch arguments to the world
    declare_world = DeclareLaunchArgument(
            'world', default_value='',
            description='Specify world file name'
    )
    declare_version = DeclareLaunchArgument(
            'version', default_value='false',
            description='Set "true" to output version information.'
    )
    declare_verbose = DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Set "true" to increase messages written to terminal.'
    )
    declare_lockstep = DeclareLaunchArgument(
            'lockstep', default_value='false',
            description='Set "true" to respect update rates'
    )
    declare_help = DeclareLaunchArgument(
            'help', default_value='false',
            description='Set "true" to produce gzserver help message.'
    )
    declare_pause = DeclareLaunchArgument(
            'pause', default_value='false',
            description='Set "true" to start the server in a paused state.'
    )
    declare_physics = DeclareLaunchArgument(
            'physics', default_value='',
            description='Specify a physics engine (ode|bullet|dart|simbody).'
    )
    declare_play = DeclareLaunchArgument(
            'play', default_value='',
            description='Play the specified log file.'
    )
    declare_record = DeclareLaunchArgument(
            'record', default_value='false',
            description='Set "true" to record state data.'
    )
    declare_record_encoding = DeclareLaunchArgument(
            'record_encoding', default_value='',
            description='Specify compression encoding format for log data (zlib|bz2|txt).'
    )
    declare_record_path = DeclareLaunchArgument(
            'record_path', default_value='',
            description='Absolute path in which to store state data.'
    )
    declare_record_period = DeclareLaunchArgument(
            'record_period', default_value='',
            description='Specify recording period (seconds).'
    )
    declare_record_filter = DeclareLaunchArgument(
            'record_filter', default_value='',
            description='Specify recording filter (supports wildcard and regular expression).'
    )
    declare_seed = DeclareLaunchArgument(
            'seed', default_value='', description='Start with a given a random number seed.'
    )
    declare_iterations = DeclareLaunchArgument(
            'iters', default_value='', description='Specify number of iterations to simulate.'
    )
    declare_minimal_comms = DeclareLaunchArgument(
            'minimal_comms', default_value='false',
            description='Set "true" to reduce TCP/IP traffic output.'
    )
    declare_profile = DeclareLaunchArgument(
            'profile', default_value='',
            description='Specify physics preset profile name from the options in the world file.'
    )
    declare_extra_args = DeclareLaunchArgument(
            'extra_gazebo_args', default_value='',
            description='Extra arguments to be passed to Gazebo'
    )
    declare_gdb = DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Set "true" to run gzserver with gdb'
    )
    declare_valgrind = DeclareLaunchArgument(
            'valgrind', default_value='false',
            description='Set "true" to run gzserver with valgrind'
    )
    declare_init = DeclareLaunchArgument(
            'init', default_value='true',
            description='Set "false" not to load "libgazebo_ros_init.so"'
    )
    declare_factory = DeclareLaunchArgument(
            'factory', default_value='true',
            description='Set "false" not to load "libgazebo_ros_factory.so"'
    )
    declare_force_system = DeclareLaunchArgument(
            'force_system', default_value='true',
            description='Set "false" not to load "libgazebo_ros_force_system.so"'
    )
    declare_server_required = DeclareLaunchArgument(
            'server_required', default_value='false',
            description='Set "true" to shut down launch script when server is terminated'
    )
    
    # Launch gzserver and gzclient with arguments
    gzserver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={
                'world'             : LaunchConfiguration('world'),
                'version'           : LaunchConfiguration('version'),
                'verbose'           : LaunchConfiguration('verbose'),
                'lockstep'          : LaunchConfiguration('lockstep'),
                'help'              : LaunchConfiguration('help'),
                'pause'             : LaunchConfiguration('pause'),
                'physics'           : LaunchConfiguration('physics'),
                'play'              : LaunchConfiguration('play'),
                'record'            : LaunchConfiguration('record'),
                'record_encoding'   : LaunchConfiguration('record_encoding'),
                'record_path'       : LaunchConfiguration('record_path'),
                'record_period'     : LaunchConfiguration('record_period'),
                'record_filter'     : LaunchConfiguration('record_filter'),
                'seed'              : LaunchConfiguration('seed'),
                'iters'             : LaunchConfiguration('iters'),
                'minimal_comms'     : LaunchConfiguration('minimal_comms'),
                'profile'           : LaunchConfiguration('profile'),
                'extra_gazebo_args' : LaunchConfiguration('extra_gazebo_args'),
                'gdb'               : LaunchConfiguration('gdb'),
                'valgrind'          : LaunchConfiguration('valgrind'),
                'init'              : LaunchConfiguration('init'),
                'factory'           : LaunchConfiguration('factory'),
                'force_system'      : LaunchConfiguration('force_system'),
                'server_required'   : LaunchConfiguration('server_required')
            }.items()
    )
    gzclient_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch),
    )

    # Run required nodes
    vision_proxy_node = Node(
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
    return LaunchDescription([
        PushRosNamespace('vision'),
        set_gazebo_resources,
        set_gazebo_model,
        declare_world,
        declare_version,
        declare_verbose,
        declare_lockstep,
        declare_help,
        declare_pause,
        declare_physics,
        declare_play,
        declare_record,
        declare_record_encoding,
        declare_record_filter,
        declare_record_path,
        declare_record_period,
        declare_seed,
        declare_iterations,
        declare_minimal_comms,
        declare_profile,
        declare_extra_args,
        declare_gdb,
        declare_valgrind,
        declare_init,
        declare_factory,
        declare_force_system,
        declare_server_required,
        gzserver_launch,
        gzclient_launch,
        vision_proxy_node
    ])
