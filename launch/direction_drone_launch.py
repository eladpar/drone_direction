import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the parameter file path
    config = os.path.join(
        get_package_share_directory('drone_direction'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='drone_direction',
            executable='accurate_model.py',
            name='accurate_model',
            parameters=[config]
        ),
        Node(
            package='drone_direction',
            executable='heuristic_model.py',
            name='heuristic_model',
            parameters=[config]
        ),
        Node(
            package='drone_direction',
            executable='controller_node.py',
            name='controller_node',
            parameters=[config]
        )
    ])
