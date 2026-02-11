/*
 * Copyright (c) 2016-2018, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef SUBMAPS_H
#define SUBMAPS_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_eigen/tf2_eigen.hpp>

/*For normal markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// This is for euclidean distance segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

// RANSAC
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT PointTypePose;

typedef pcl::PointXYZI pcl_t;

class SubMaps  : public rclcpp::Node 
{

private:

  std::string pose_graph_dir_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr warm_up_timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_map_warmup_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_ground_warmup_;

  geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped warm_up_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_sub_map_pose_;
  
  pcl::PointCloud<PointTypePose>::Ptr pcd_poses_; //original poses from files
  pcl::PointCloud<PointTypePose> poses_; //Operating poses, we maitain this poses all the time
  std::vector<pcl::PointCloud<pcl_t>::Ptr> cornerCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<pcl_t>::Ptr> cornerCloudKeyFrames_baselink_; //original frame from files are base_link
  std::vector<pcl::PointCloud<pcl_t>::Ptr> surfCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<pcl_t>::Ptr> surfCloudKeyFrames_baselink_; //original frame from files are base_link
  
  void readPoseGraph();
  
  //@
  bool is_initial_;
  bool is_current_ready_;
  pcl::KdTreeFLANN<pcl_t>::Ptr kdtree_poses_;

  pcl::PointCloud<pcl_t>::Ptr map_warmup_;
  pcl::PointCloud<pcl_t>::Ptr ground_warmup_;

  //@
  bool prepare_warm_up_;
  bool is_warm_up_ready_;
  double sub_map_search_radius_;
  double sub_map_warmup_trigger_distance_;
  float complete_map_voxel_size_;
  
public:

  SubMaps(std::string name);
  ~SubMaps();

  pcl::PointCloud<pcl_t>::Ptr map_current_;
  pcl::PointCloud<pcl_t>::Ptr ground_current_;
  
  void setPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose);
  void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose);
  bool isWarmUpReady();
  void swapKdTree();
  
  void warmUpThread();
  bool isCurrentReady(){return is_current_ready_;};

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex sub_maps_mutex_t;
  sub_maps_mutex_t * getMutex()
  {
    return access_;
  }
  sub_maps_mutex_t * access_;

  pcl::KdTreeFLANN<pcl_t> kdtree_map_current_;
  pcl::KdTreeFLANN<pcl_t> kdtree_ground_current_;
  pcl::PointCloud<pcl::Normal> normals_ground_current_;

  pcl::KdTreeFLANN<pcl_t> kdtree_map_warmup_;
  pcl::KdTreeFLANN<pcl_t> kdtree_ground_warmup_;
  pcl::PointCloud<pcl::Normal> normals_ground_warmup_;

};

#endif  // SUBMAPS_H
