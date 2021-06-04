/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/1 下午9:14
 * @FileName: lidar_odometry_nodelet.cpp.c
 * @Description: LiDAR Odometry Nodelet
 * @License: See LICENSE for the license information
 */

#include <iostream>
#include <string>

// for ros
#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "tloam/front_end/front_end.hpp"

namespace tloam{
class LidarOdometryNode : public nodelet::Nodelet {
public:
  LidarOdometryNode() : shutdown_requested_(false) {

  }

  ~LidarOdometryNode() override{
    NODELET_DEBUG_STREAM("waiting for lidar_odometry_nodelet update thread to finish.");
    shutdown_requested_ = true;
    odometry_update_thread_.join();
  }

public:
  void onInit() override{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // Analyze whether the currently initialized nodelet is complete
    std::string namespace_ = private_nh_.getNamespace();
    namespace_ = namespace_.substr(namespace_.find_last_of('/') + 1);
    NODELET_INFO_STREAM("Initialising nodelet... [" << namespace_ << "]");
    init_params();
    front_ent_ptr_ = std::make_shared<FrontEnd>(nh_, odom_topic_);

    if (front_ent_ptr_->initWithConfig()){
      NODELET_INFO_STREAM("Nodelet initialised. Spinning up update thread ... [" << namespace_ << "]");
      odometry_update_thread_.start(&LidarOdometryNode::update, *this);
      NODELET_INFO_STREAM("Nodelet initialised. [" << namespace_ << "]");
    } else {
      NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << namespace_ << "]");
    }

  }

private:
  void update(){
    ros::Rate loop_rate(10);
    while (ros::ok() && !shutdown_requested_) {
      front_ent_ptr_->spinOnce();
      ros::spinOnce();
    }
  }

  void init_params() {
    auto &ph_ = private_nh_;
    cloud_topic_ = ph_.param<std::string>("cloud_topic", "/velodyne_points");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string cloud_topic_;
  std::string ground_topic_;
  std::string edge_topic_;
  std::string general_topic_;
  std::string odom_topic_;

  std::shared_ptr<FrontEnd> front_ent_ptr_;

  ecl::Thread odometry_update_thread_;

  bool shutdown_requested_;

};

}


PLUGINLIB_EXPORT_CLASS(tloam::LidarOdometryNode, nodelet::Nodelet);