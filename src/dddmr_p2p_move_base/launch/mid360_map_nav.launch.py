from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get package share directories
    p2p_move_base_pkg_share = get_package_share_directory('p2p_move_base')
    # lego_loam_pkg_share = get_package_share_directory('lego_loam_bor')

    # Configuration files
    config_go2_mapping = PathJoinSubstitution([p2p_move_base_pkg_share, 'config', 'go2_mapping.yaml'])
    rviz_config = PathJoinSubstitution([p2p_move_base_pkg_share, 'rviz', 'move_base_mapping.rviz'])

    # Global planner node
    global_planner_node = Node(
        package='global_planner',
        executable='global_planner_node',
        output='screen',
        respawn=False,
        parameters=[
            config_go2_mapping,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odom', '/lio/odometry')
        ]
    )

    # P2P move base node
    p2p_move_base_node = Node(
        package='p2p_move_base',
        executable='p2p_move_base_node',
        output='screen',
        respawn=False,
        parameters=[
            config_go2_mapping,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odom', '/lio/odometry')
        ]
    )

    # Clicked to goal node
    clicked_to_goal_node = Node(
        package='p2p_move_base',
        executable='clicked2goal.py',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        respawn=False,
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Include LIO-LOAM launch file
    lio_nav_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lio_nav_bridge'), 'launch', 'lio_nav_bridge.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,

        # Original mapping and navigation nodes
        global_planner_node,
        p2p_move_base_node,
        clicked_to_goal_node,
        rviz_node,

        # LIO-LOAM integration
        lio_nav_bridge_launch,
    ])