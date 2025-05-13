from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace'
    ),

    DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start rviz'
    ),

    DeclareLaunchArgument(
        'imu',
        default_value='false',
        description='Whether to start the imu'
    ),

    DeclareLaunchArgument(
        'laser',
        default_value='false',
        description='Whether to start the laser'
    ),

    DeclareLaunchArgument(
        'dcam',
        default_value='false',
        description='Whether to start the depth camera'
    ),

    DeclareLaunchArgument(
        name='madgwick',
        default_value='false',
        description='Use madgwick to fuse imu and magnetometer'
    ),

    DeclareLaunchArgument(
        name='uros',
        default_value='false',
        description='Use micro-ros'
    ),
    
    DeclareLaunchArgument(
        'ntrip',
        default_value='false',
        description='Whether to start the ntrip client'
    ),

    DeclareLaunchArgument(
        'gps',
        default_value='false',
        description='Whether to start the gps'
    ),

    DeclareLaunchArgument(
        'foxglove',
        default_value='false',
        description='Whether to start the foxglove'
    ),

    DeclareLaunchArgument(
        'sensormon',
        default_value='false',
        description='Whether to start the sensor monitor'
    ),
    
    DeclareLaunchArgument(
        'rl',
        default_value='false',
        description='Whether to launch Robot Localization'
    ),
]

def generate_launch_description():

    extra_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_legacy_launch'), 'config', 'extra.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_legacy_launch'), 'rviz', 'mowbot.rviz']
    )

    return LaunchDescription(ARGS + [  
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '-p', '8888'],
            condition=IfCondition(LaunchConfiguration("uros"))
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_description'), 'launch', 'description.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': 'false'
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_bringup'), 'launch', 'twist_control.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_teleop_joy': 'false',
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 'launch', 'gui', 'components', 'sensors.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'imu': LaunchConfiguration('imu'),
                'laser': LaunchConfiguration('laser'),
                'dcam': LaunchConfiguration('dcam'),
                'ntrip': LaunchConfiguration('ntrip'),
                'gps': LaunchConfiguration('gps'),
            }.items()
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration("madgwick")),
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_node',
            output='screen',
            parameters=[extra_config_path],
            remappings=[
                ('imu/data_raw', 'mb_imu/data'),
                ('imu/mag', '/mb_imu/mag')
            ]
        ),

        # utilities
        Node(
            namespace=LaunchConfiguration('namespace'),
            package='py_mowbot_utils',
            executable='sensor_monitor_2',
            name='sensor_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('sensormon')),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml']
            ),
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),

        Node(
            namespace = LaunchConfiguration('namespace'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config_path]
        ),
        
        # robot localization
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 
                'launch', 'main', 'components', 'rl_dual_ekf_navsat.launch.py']
            ),
            condition=IfCondition(LaunchConfiguration("rl"))
        ),
    ])