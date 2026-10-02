from launch import LaunchDescription
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    cam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_cam'),
                'launch/xycar_cam.launch.py'))
    )
    
    return LaunchDescription([
        cam_include,
        Node(
            package='ar_detector',
            executable='my_ar_image',
            name='my_ar_image',
        ),
        Node(
            package='ar_detector',
            executable='ar_detector',
            name='ar_detector',
        ),
    ])

