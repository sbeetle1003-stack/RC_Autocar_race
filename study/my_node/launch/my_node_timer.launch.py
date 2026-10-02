from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_node',
            executable='student_timer',
            name='student',
        ),
        Node(
            package='my_node',
            executable='teacher',
            name='teacher'
        )
    ])
