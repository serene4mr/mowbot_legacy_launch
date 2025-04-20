from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

ARGS = [
    # Declare launch arguments
    DeclareLaunchArgument(
        "lat",
        default_value="36.114016",
        description="Original Latitude"
    ),

    DeclareLaunchArgument(
        "lon",
        default_value="128.418550",
        description="Original Longitude"
    ),
    
    DeclareLaunchArgument(
        "rl",
        default_value="false",
        description="Use robot_localization"
    ),
    
    DeclareLaunchArgument(
        "mapviz",
        default_value="true",
        description="Use mapviz"
    ),
]

def generate_launch_description():

    mapviz_config = PathJoinSubstitution(
        [FindPackageShare('mowbot_legacy_launch'), 'mvc', 'gps_wp_logger.mvc']
    )


    return LaunchDescription(ARGS + [
        
        # robot localization
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 
                'launch', 'main', 'components', 'rl_dual_ekf_navsat.launch.py']
            ),
            condition=IfCondition(LaunchConfiguration("rl"))
        ),
        
        Node(
            package='py_mowbot_utils',
            executable='gps_waypoints_logger',
            name='gps_waypoints_logger',
            output='screen',
            remappings=[
                ('/gps/fix', 'gps/fix_filtered'),
                ('/imu', '/gps/heading')
            ]
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 
                'launch', 'main', 'components', 'mapviz.launch.py']
            ),
            launch_arguments={
                "fix_topic": "gps/fix_filtered",
                "mvc_config": mapviz_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration("mapviz"))
        )
        
    ])
