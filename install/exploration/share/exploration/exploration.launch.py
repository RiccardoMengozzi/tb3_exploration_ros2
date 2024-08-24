import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

    exploration_node = Node(
            package='exploration',
            executable='exploration',
            name='Exploration',
    )

    ld = LaunchDescription()
    ld.add_action(exploration_node)

    return ld