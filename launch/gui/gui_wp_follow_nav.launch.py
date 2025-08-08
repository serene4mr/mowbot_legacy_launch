from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    nav2_params = "/mowbot_legacy_data/__nav_params__.yaml"
    waypoints_file = "/mowbot_legacy_data/__waypoints__.yaml"
    cmdvel_scaler_params = "/mowbot_legacy_data/__cmdvel_scaler_params__.yaml"
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, 
        root_key="", 
        param_rewrites="", 
        convert_types=True
    )
    
    return LaunchDescription([
        
        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 
                     'launch', 'navigation_launch.py']
                )
            ),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": configured_params,
                "autostart": "True",
            }.items(),
        ),

        Node(
            package="py_mowbot_utils",
            executable="gui_gps_waypoint_follower",
            name="gui_gps_waypoint_follower",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "waypoints_file": waypoints_file
                }
            ],
        ),
    ])

