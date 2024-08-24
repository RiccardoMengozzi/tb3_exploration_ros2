from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    big_house_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'turtlebot3_big_house.launch.py')),
    )
    
    velocity_controller_node = Node(
            package='exploration',
            executable='velocity_controller',
            name='VelocityController',
    )

    ld = LaunchDescription()
    ld.add_action(big_house_launch)
    ld.add_action(velocity_controller_node)

    return ld
        
