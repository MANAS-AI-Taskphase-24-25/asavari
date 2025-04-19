from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hydrant_finder',
            executable='hydrant_finder_node',
            name='hydrant_finder_node',
            output='screen'
        )      
    ])

