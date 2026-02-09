from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bag_converter',
            executable='bag_converter',
            name='bag_converter',
            output='screen',
            parameters=[
                {
                    'input_bag': '/media/lenovo/1/rosbag/GNSS_denial02/',
                    'output_bag': '/media/lenovo/1/rosbag/GNSS_denial02_fix/',
                    'livox_topics': [
                        '/livox/avia/lidar',
                        '/livox/mid360/lidar'
                    ],
                    # Optional: unified frame_id
                    'output_frame_override': '',
                    'verbose': True,
                }
            ]
        )
    ])
