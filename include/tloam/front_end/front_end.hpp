/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/13 上午9:11
 * @FileName: front_end.hpp
 * @Description: LiDAR Odometry node by TLS
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_FRONT_END_HPP
#define TLOAM_FRONT_END_HPP

#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <omp.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <open3d/Open3D.h>
#include "tloam/open3d/open3d_to_ros.hpp"
#include "tloam/models/utils/sensor_data.hpp"
#include "tloam/models/utils/utils.hpp"
#include "tloam/models/utils/work_space_path.h"
#include "tloam/models/feature_extraction/feature_extract.hpp"
#include "tloam/models/registration/registration_interface.hpp"
#include "tloam/models/registration/registration.hpp"

#include "tloam/subscriber/cloud_subscriber.hpp"
#include "tloam/subscriber/tf_listener.hpp"
#include <tloam/subscriber/odometry_subscriber.hpp>

#include "tloam/publisher/cloud_publisher.hpp"
#include "tloam/publisher/odometry_publisher.hpp"
#include "tloam/publisher/tf_broadcaster.hpp"


namespace tloam{
class FrontEnd{
public:
  struct FrameBuffer{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    std::shared_ptr<open3d::geometry::PointCloud2> cloud_data_ptr = std::make_shared<open3d::geometry::PointCloud2>();
  };
  FrontEnd(ros::NodeHandle& nh, std::string& odom_topic);
  ~FrontEnd();

  void spinOnce();

  bool initWithConfig();

protected:
  bool readData();
  bool hasData();
  bool validData();
  void savePose(std::ofstream& ofs, const Eigen::Matrix4d& pose);

  bool setInitPose(const Eigen::Isometry3d& init_pose_);

  static bool initRegistraton(std::shared_ptr<RegistrationInterface>& registration_ptr_, const YAML::Node& config_node);

  void processCloud();

  void updateSubmap();

  bool updateLidarOdometry();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_ground_ptr_;
  std::shared_ptr<CloudSubscriber> cloud_sub_general_ptr_;
  std::shared_ptr<CloudSubscriber> cloud_sub_edge_ptr_;
  std::shared_ptr<CloudSubscriber> cloud_sub_raw_ptr_;

  // publisher
  std::shared_ptr<CloudPublisher> raw_publish_ptr_;
  std::shared_ptr<CloudPublisher> glocal_map_pub_ptr_;
  std::shared_ptr<OdometryPublisher> lidar_odom_ptr_;
  std::shared_ptr<OdometryPublisher> ground_truth_ptr_;
  std::shared_ptr<TFBroadCaster> lidar_odom_tf_;
  std::unique_ptr<TFListener> ground_truth_tf_ptr_;

  std::deque<CloudData> edge_data_buff_;
  std::deque<CloudData> ground_data_buff_;
  std::deque<CloudData> general_data_buff_;
  std::deque<CloudData> raw_data_buff_;

  std::shared_ptr<featureExtract> feature_extract_ptr_;
  std::shared_ptr<RegistrationInterface> local_registration_ptr_;

  CloudData current_ground_data;
  CloudData current_edge_data;
  CloudData current_general_data;
  CloudData current_raw_data;
  CloudData global_map_data;


  Frame current_scan{};
  Frame submap{};

  std::vector<size_t> planar_scan_index{};
  std::vector<size_t> planar_submap_index{};
  std::vector<size_t> sphere_scan_index{};
  std::vector<size_t> sphere_submap_index{};

  Eigen::Isometry3d lidar_odom_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d step_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d ground_truth = Eigen::Isometry3d::Identity();

  std::deque<FrameBuffer> submap_sphere_buffer;
  std::deque<FrameBuffer> submap_planar_buffer;
  std::deque<FrameBuffer> submap_ground_buffer;

  int num_threads_;
  double sensorPeriod{0.1};
  double ground_down_sample;
  double edge_down_sample;
  double ground_down_sample_submap;
  double edge_down_sample_submap;
  int sphere_frame_size;
  int planar_frame_size;
  double edge_crop_box_length;
  double ground_crop_box_length;
  bool mapping_flag;

};
}

#endif //TLOAM_FRONT_END_HPP
