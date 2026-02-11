"""
Combined LIO and LEGO-LOAM Launch File with Bag Play and PGO

Author: Combined from both packages
Date: 2026-01-27
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Get package share directories
    lio_nav_bridge_pkg_share = get_package_share_directory('lio_nav_bridge')
    lio_with_pgo_pkg_share = get_package_share_directory('pgo')

    # LEGO-LOAM config path
    lio_nav_bridge_config = PathJoinSubstitution([lio_nav_bridge_pkg_share, 'config', 'nav_map_mid360.yaml'])

    # LEGO-LOAM RViz config
    lio_nav_bridge_rviz_config = os.path.join(lio_nav_bridge_pkg_share, 'rviz', 'livox.rviz')

    # Declare launch arguments for LIO
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/mid360/imu',
        description='IMU data topic'
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/mid360/lidar',
        description='LiDAR point cloud topic'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/lio/odometry',
        description='Odometry topic'
    )
    # Declare launch arguments for bag file
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/media/lenovo/1/rosbag/GNSS_denial02_fix/', 
        # default_value='/media/lenovo/1/rosbag/Outdoor01_fix/',
        description='Path to the bag file to play'
    )
    enable_bag_play_arg = DeclareLaunchArgument(
        'enable_bag_play',
        default_value='true',
        description='Enable ros2 bag play node'
    )
    open_bridge_rviz = DeclareLaunchArgument(
        'open_bridge_rviz',
        default_value='true',
        description='Open RViz for PGO'
    )

    # LIO-LOAM Node
    lio_nav_bridge_node = Node(
        package='lio_nav_bridge',
        executable='lio_nav_bridge_node',
        output='screen',
        respawn=False,
        parameters=[lio_nav_bridge_config],
        remappings=[
            ('/lslidar_point_cloud', '/lio/cloud_registered_lidar',),
            # ('/lslidar_point_cloud', LaunchConfiguration('lidar_topic'),),
            ('/odom', LaunchConfiguration('odom_topic'))
        ]
    )

    # LIO launch
    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lio_with_pgo_pkg_share, 'launch', 'pgo_sparklio.launch.py')
        ),
        launch_arguments={
            'open_rviz': "false"
        }.items()
    )

    # RViz2 for LEGO-LOAM
    rviz_lio_nav_bridge_node = Node(
        condition=IfCondition(LaunchConfiguration('open_bridge_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_lio_nav_bridge',
        arguments=['-d', lio_nav_bridge_rviz_config],
        output='screen'
    )

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor2baselink1',
        arguments=['0.0', '0', '-0.2', '0.0', '0.0', '0', 'base_link', 'base_footprint']
    )

    # Delayed ros2 bag play node (2 seconds delay)
    delayed_bag_play_node = TimerAction(
        period=2.0,  # 2 seconds delay
        actions=[
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('enable_bag_play')),
                cmd=['ros2', 'bag', 'play',
                     LaunchConfiguration('bag_file'),
                     '--topics',
                     LaunchConfiguration('lidar_topic'),
                     LaunchConfiguration('imu_topic')],
                shell=True,
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        bag_file_arg,
        enable_bag_play_arg,
        imu_topic_arg,
        lidar_topic_arg,
        odom_topic_arg,
        open_bridge_rviz,

        lio_nav_bridge_node,
        lio_launch,
        rviz_lio_nav_bridge_node,
        static_transform_node,

        delayed_bag_play_node,
    ])