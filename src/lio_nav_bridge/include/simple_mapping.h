#ifndef LIO_NAV_BRIDGE_H
#define LIO_NAV_BRIDGE_H

#include "utility.h"
#include "channel.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// for tilted lidar
#include <tf2_eigen/tf2_eigen.hpp>

// get robot frame to sensor frame tf
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"

#include "tf2_ros/static_transform_broadcaster.h"

// ros
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>

#ifdef TRT_ENABLED
#include "dddmr_trt/yolov8.h"
#include <opencv2/cudaimgproc.hpp>
#endif

// Message filters for time synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_srvs/srv/empty.hpp"

namespace sync_policies = message_filters::sync_policies;

using namespace std::chrono_literals;

class SimpleMapping : public rclcpp::Node
{
  public:

    SimpleMapping(std::string name);

    ~SimpleMapping() = default;

    void callbackSync(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
                      const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void tfInitial();

  private:

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds

    void findStartEndAngle();
    void resetParameters();
    void projectPointCloud();
    void groundRemoval();
    void cloudSegmentation();
    void labelComponents(int row, int col);
    void publishClouds();
    bool allEssentialTFReady(std::string sensor_frame);

    void publishMaps();
    void addKeyFrame();
    bool isKeyFrame(const geometry_msgs::msg::Pose& current_pose);
    void transformPointCloud(pcl::PointCloud<PointType>::Ptr cloud,
                            const geometry_msgs::msg::Transform& transform,
                            pcl::PointCloud<PointType>::Ptr transformed_cloud);
    void transformPointCloud(pcl::PointCloud<PointType>::Ptr cloud,
                            const geometry_msgs::msg::Pose& pose,
                            pcl::PointCloud<PointType>::Ptr transformed_cloud);
    void pcdSaver(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);

    pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

    pcl::PointCloud<PointType>::Ptr _full_cloud;
    pcl::PointCloud<PointType>::Ptr _full_info_cloud;

    pcl::PointCloud<PointType>::Ptr _ground_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
    pcl::PointCloud<PointType>::Ptr _outlier_cloud;
    pcl::PointCloud<PointType>::Ptr patched_ground_;
    pcl::PointCloud<PointType>::Ptr patched_ground_edge_;

    pcl::VoxelGrid<PointType> dsf_patched_ground_;

    int _vertical_scans;
    int _horizontal_scans;
    double _scan_period;
    float _ang_bottom;
    float _ang_resolution_X;
    float _ang_resolution_Y;
    float _segment_alpha_X;
    float _segment_alpha_Y;
    float _segment_theta;
    int _segment_valid_point_num;
    int _segment_valid_line_num;
    int _ground_scan_index;
    double _sensor_mount_angle;
    double _sensor_yaw_angle;
    // std::string odom_type_;
    std::string baselink_frame_, sensor_frame_;

    // Publishers for maps
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGround;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odometry;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvSavePCD;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_ground_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud_pure;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloud;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_projected_image;
    
    CloudInfo _seg_msg;

    int _label_count;
    
    cv::Mat range_mat_removing_moving_object_;
    Eigen::MatrixXf _range_mat;   // range matrix for range image
    Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
    Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking

    float _maximum_detection_range;
    float _minimum_detection_range;
    double distance_for_patch_between_rings_;
    int first_frame_processed_;
    bool got_baselink2sensor_tf_;
    geometry_msgs::msg::TransformStamped trans_b2s_;
    tf2::Transform tf2_trans_b2s_, tf2_trans_c2s_, tf2_trans_c2b_;
    tf2::Transform tf2_trans_odom_to_pitch_removed_;
    geometry_msgs::msg::TransformStamped trans_c2s_;
    geometry_msgs::msg::TransformStamped trans_c2b_;
    geometry_msgs::msg::TransformStamped trans_odom_to_pitch_removed_;

    //@ list of pointcloud sticher for non-repetitive scan lidar
    std::list<pcl::PointCloud<PointType>> pcl_stitcher_;    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    double last_save_depth_img_time_;

    bool to_fa_;
    std::string mapping_dir_string_;
    
    // Global maps
    pcl::PointCloud<PointType>::Ptr _global_map;
    pcl::PointCloud<PointType>::Ptr _global_ground;

    // Keyframe storage
    std::vector<geometry_msgs::msg::Pose> _keyframe_poses;

    // Previous pose for keyframe selection
    geometry_msgs::msg::Pose _prev_keyframe_pose;
    bool _first_keyframe;

    // Parameters
    double _keyframe_distance_thresh;
    double _keyframe_angle_thresh;
    double _map_resolution;
    double _ground_resolution;

    // Voxel grids for downsampling
    pcl::VoxelGrid<PointType> _map_voxel_filter;
    pcl::VoxelGrid<PointType> _ground_voxel_filter;

    // Current odometry
    nav_msgs::msg::Odometry::SharedPtr _current_odom;
    bool _has_odom;
    bool _has_initialized;

    // Time synchronization
    rclcpp::Time _last_cloud_time;
    rclcpp::Time _last_odom_time;

    // Message filters for synchronized processing
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> _cloud_sub;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> _odom_sub;
    std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> _sync;

    // Storage for odometry history for point cloud stitching
    std::list<nav_msgs::msg::Odometry::SharedPtr> _odom_history;
    
    // Image projection parameters
    double time_step_between_depth_image_;
    
    int stitcher_num_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_img_;
    
    bool is_trt_engine_exist_;
    std::string trt_model_path_;
#ifdef TRT_ENABLED
    std::shared_ptr<YoloV8> yolov8_;
#endif
};

#endif // LIO_NAV_BRIDGE_H
