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
]

def generate_launch_description():
    
    return LaunchDescription(ARGS + [  
        # robot localization
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_legacy_launch'), 
                'launch', 'main', 'components', 'rl_dual_ekf_navsat.launch.py']
            ),
        ),
    ])