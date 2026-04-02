# 获取当前脚本所在目录，防止路径错乱
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# 切换到工作空间根目录
cd "$SCRIPT_DIR" || exit

# 检查 install 目录是否存在
if [ ! -d "install" ]; then
    echo "❌ 未找到 install 目录，请先执行: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
    exit 1
fi

# 加载 ROS 2 环境和工作空间
echo "🔧 正在加载 ROS 2 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# SLAM+NAV 启动命令
# echo "🚀 启动 "
# ros2 launch lio_nav_bridge lio_nav_bridge_sim.launch.py
# ros2 launch lio_nav_bridge lio_nav_bridge_bag.launch.py
ros2 launch p2p_move_base move_base_mapping_mode.launch.py

# 原 DDRMR 启动命令
# ros2 launch lego_loam_bor lego_loam_bag.launch
# ros2 launch mcl_3dl mcl_3dlXfeatureXbag.launch
# ros2 launch perception_3d scanning_lidar_3d_ros_launch.py
# ros2 launch global_planner path_planning_on_static_layer.launch
# ros2 launch local_planner local_planner_play_ground.launch
# ros2 launch p2p_move_base go2_localization.launch
# ros2 launch p2p_move_base go2_mapping.launch

# 启动 Gazebo 环境
# ros2 launch go2_config gz_lidar_odom.launch.py    # with gt odom
# ros2 launch go2_config gazebo_lidar_gps.launch.py
# ros2 launch go2w_description gazebo.launch.py
# ros2 launch go2w_config go2w_lidar_gps.launch.py

# 单测 LIO 节点
# ros2 launch surfelio lio_mid360.launch.py
# ros2 launch fastlio2 lio_launch.py
# ros2 launch super_odometry livox_mid360.launch.py
# ros2 launch II_NVM run.launch.py
# ros2 launch ct_lio run_eskf.launch.py
# ros2 launch kiss_matcher_ros slam_with_livox.launch.yaml
# ros2 launch spark_fast_lio mapping_livox.launch.yaml

# LIO 带回环
# ros2 launch pgo pgo_fastlio2.launch.py
# ros2 launch pgo pgo_surfelio.launch.py
# ros2 launch pgo pgo_sparklio.launch.py
# ros2 launch localizer localizer_launch.py

# 工具类节点
# ros2 launch bag_converter convert_livox_bag.launch.py

