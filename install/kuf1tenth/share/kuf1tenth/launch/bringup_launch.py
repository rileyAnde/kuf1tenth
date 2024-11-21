from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    config_dir = PathJoinSubstitution([
        FindPackageShare('kuf1tenth'),
        'config'
    ])
    map_file = PathJoinSubstitution([
        FindPackageShare('kuf1tenth'),
        'map',
        'map.yaml'
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([config_dir, 'nav2_params.yaml']),
            description='Full path to param file to load'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': PathJoinSubstitution([config_dir, 'nav2_params.yaml']),
                'map': map_file
            }.items()
        ),
        Node(
            package='ackermann_vehicle',
            executable='ackermann_vehicle_node',
            name='ackermann_vehicle_node',
            output='screen'
        )
    ])
