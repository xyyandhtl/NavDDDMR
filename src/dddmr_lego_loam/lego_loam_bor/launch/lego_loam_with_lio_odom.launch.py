"""
Combined LIO and LEGO-LOAM Launch File with Bag Play

Author: Combined from both packages
Date: 2026-01-27
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    lego_loam_pkg_share = get_package_share_directory('lego_loam_bor')
    lio_pkg_share = get_package_share_directory('surfelio')
    
    # LEGO-LOAM config path
    lego_loam_config = PathJoinSubstitution([lego_loam_pkg_share, 'config', 'loam_mid360_config.yaml'])
    
    # LIO config path
    lio_default_config = os.path.join(lio_pkg_share, 'config', 'lidar_inertial_odometry', 'mid360.yaml')
    lio_default_rviz_config = os.path.join(lio_pkg_share, 'rviz', 'lio_rviz.rviz')
    
    # LEGO-LOAM RViz config
    lego_loam_rviz_config = os.path.join(lego_loam_pkg_share, 'rviz', 'lego_loam_livox.rviz')

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
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=lio_default_config,
        description='LIO configuration file path'
    )
    init_imu_samples_arg = DeclareLaunchArgument(
        'init_imu_samples',
        default_value='100',
        description='Number of IMU samples for gravity initialization'
    )
    
    # Declare launch arguments for bag file
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/media/lenovo/1/rosbag/Outdoor01_fix/',
        description='Path to the bag file to play'
    )
    enable_bag_play_arg = DeclareLaunchArgument(
        'enable_bag_play',
        default_value='true',
        description='Enable ros2 bag play node'
    )

    # LEGO-LOAM Node
    lego_loam_node = Node(
        package='lego_loam_bor',
        executable='lego_loam',
        output='screen',
        respawn=False,
        parameters=[lego_loam_config],
        remappings=[
            ('/lslidar_point_cloud', LaunchConfiguration('lidar_topic'),),
            ('/odom', '/lio/odometry')
        ]
    )

    # LIO Node
    lio_node = Node(
        package='surfelio',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'config_file': LaunchConfiguration('config_file'),
            'init_imu_samples': LaunchConfiguration('init_imu_samples'),
            'extrinsic_tx': 0.0,
            'extrinsic_ty': 0.0,
            'extrinsic_tz': -0.1,
            'extrinsic_rx': 0.0,
            'extrinsic_ry': -0.5,
            'extrinsic_rz': 0.0,
            'base_link_frame': 'base_link',
        }]
    )

    # RViz2 for LEGO-LOAM
    rviz_lego_loam_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lego_loam',
        arguments=['-d', lego_loam_rviz_config],
        output='screen'
    )
    
    # RViz2 for LIO
    rviz_lio_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lio',
        arguments=['-d', lio_default_rviz_config],
        output='screen'
    )

    # Static transform publisher
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor2baselink',
        arguments=['0.0', '0', '0.1',  '0.0', '0.5', '0', 'base_link', 'livox_frame'],
        output='screen'
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
        config_file_arg,
        init_imu_samples_arg,
        lio_node,
        # rviz_lio_node,

        lego_loam_node,
        rviz_lego_loam_node,
        static_transform_node,

        delayed_bag_play_node,
    ])