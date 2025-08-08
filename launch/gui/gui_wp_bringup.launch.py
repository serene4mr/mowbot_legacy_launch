#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml
import os

# ---- Configurable paths ----
DEFAULT_CFG_PATH = '/mowbot_legacy_data/robot_config.yaml'   

def get_robot_model_from_yaml(config_path=DEFAULT_CFG_PATH):
    # Reads robot_model from YAML config; defaults to 'mowbot'
    try:
        with open(config_path) as f:
            config = yaml.safe_load(f)
        robot_model = config.get('robot_model', 'mowbot')
    except Exception as e:
        robot_model = 'mowbot'
    return robot_model

# ---- Declare launch arguments ----
ARGS = [
    DeclareLaunchArgument('namespace',     default_value='',          description='Namespace'),
    DeclareLaunchArgument('rviz',          default_value='false',     description='Whether to start rviz'),
    DeclareLaunchArgument('imu',           default_value='false',     description='Whether to start the imu'),
    DeclareLaunchArgument('laser',         default_value='false',     description='Whether to start the laser'),
    DeclareLaunchArgument('dcam',          default_value='false',     description='Whether to start the depth camera'),
    DeclareLaunchArgument('madgwick',      default_value='false',     description='Use madgwick to fuse imu and magnetometer'),
    DeclareLaunchArgument('uros',          default_value='false',     description='Use micro-ros'),
    DeclareLaunchArgument('ntrip',         default_value='false',     description='Whether to start the ntrip client'),
    DeclareLaunchArgument('gps',           default_value='false',     description='Whether to start the gps'),
    DeclareLaunchArgument('foxglove',      default_value='false',     description='Whether to start the foxglove'),
    DeclareLaunchArgument('sensormon',     default_value='false',     description='Whether to start the sensor monitor'),
    DeclareLaunchArgument('rl',            default_value='false',     description='Whether to launch Robot Localization'),
]

def generate_launch_description():
    # Paths
    extra_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_legacy_launch'), 'config', 'extra.yaml']
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_legacy_launch'), 'rviz', 'mowbot.rviz']
    )
    # ---- Load robot model from config file ----
    robot_model = get_robot_model_from_yaml(config_path=DEFAULT_CFG_PATH)

    # Log info for clarity
    log_model = LogInfo(msg=[f"[mowbot_legacy_launch] Using robot model: {robot_model} (config: {DEFAULT_CFG_PATH})"])

    ld = [
        log_model,

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
                'use_sim_time': 'false',
                'model': robot_model
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 'launch', 'gui', 'components', 'twist_control.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_teleop_joy': 'false',
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 
                 'launch', 'gui', 'components', 'sensors.launch.py']
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
            namespace=LaunchConfiguration('namespace'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config_path]
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 'launch', 'gui', 'components', 'rl_dual_ekf_navsat.launch.py']
            ),
            condition=IfCondition(LaunchConfiguration("rl"))
        ),
    ]

    return LaunchDescription(ARGS + ld)
