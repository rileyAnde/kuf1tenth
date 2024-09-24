from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
   ld = LaunchDescription()

   brake_node = Node(
        package='kuf1tenth',
        executable='brake',
        name='brake',
   )

   wall_follow_node = Node(
        package='kuf1tenth',
        executable='wall_follow',
        name='wall_follow',
   )

   ld.add_action(brake_node)
   ld.add_action(wall_follow_node)

   return ld
