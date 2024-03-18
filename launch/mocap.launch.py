import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mocap_nokov',
            executable='mocap_node',
            name='mocap_node',
            parameters=[
                os.path.join(get_package_share_directory('mocap_nokov'), 'config', 'mocap.yaml')
            ],
            output='screen')
    ])
    
generate_launch_description()
