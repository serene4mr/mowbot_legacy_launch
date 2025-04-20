from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    rlgps_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_legacy_launch'), 'config', 'dual_ekf_gps_params.yaml']
    )

    return LaunchDescription([  
                              
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        
        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                rlgps_config_path,
                {"use_sim_time": False},
            ],
            remappings=[
                #input
                ("odom", "mowbot_base/odom"),
                ("imu", "imu/data"),
                ("imu/gps_heading", "gps/heading"),
                #output
                ("odometry/filtered", "odom/local")
            ]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                rlgps_config_path,
                {"use_sim_time": False},
            ],
            remappings=[
                #input
                ("odom", "mowbot_base/odom"),
                ("imu", "imu/data"),
                ("imu/gps_heading", "gps/heading"),
                ("odom/gps", "odom/gps"),
                #output
                ("odometry/filtered", "odom/global")
                
            ]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                rlgps_config_path,
                {"use_sim_time": False},
            ],
            remappings=[
                #input
                ("odometry/filtered", "odom/global"),
                ("gps/fix", "gps/fix"),
                ("imu", "imu/data"),
                #output
                ("odometry/gps", "odom/gps"),
                ("gps/filtered", "gps/fix_filtered"),
    
            ]
        ),
    ])