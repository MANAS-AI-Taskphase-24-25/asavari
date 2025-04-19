from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonav_cpp',
            executable='astar_planner_node',
            name='astar_planner',
            output='screen'
        ),
        Node(
            package='autonav_cpp',
            executable='path_follower_node',
            name='path_follower',
            output='screen'
        )        
    ])

