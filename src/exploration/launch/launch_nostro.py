from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    
    
    velocity_controller_node = Node(
            package='exploration',
            executable='velocity_controller',
            name='velocity_controller',
    )

    ld = LaunchDescription()

    ld.add_action(velocity_controller_node)

    return ld
        
