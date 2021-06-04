/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/27 下午1:52
 * @FileName: kitti_reader_nodelet.cpp
 * @Description: Load KITTI Odometry Dataset
 * @License: See LICENSE for the license information
 */

#include <iostream>
#include <string>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

// for nodelet
#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "tloam/models/io/kitti_reader.hpp"
#include "tloam/models/utils/utils.hpp"
#include "tloam/models/utils/work_space_path.h"


namespace tloam {

class KittiNode : public nodelet::Nodelet {
public:
  KittiNode() : shutdown_requested_(false) {

  }

  ~KittiNode() override {
    NODELET_DEBUG_STREAM("waiting for kitti_reader_nodelet update thread to finish.");
    shutdown_requested_ = true;
    kitti_update_thread_.join();
  }

public:
  void onInit() override {
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();
    // Analyze whether the currently initialized nodelet is complete
    std::string namespace_ = private_nh_.getNamespace();
    namespace_ = namespace_.substr(namespace_.find_last_of('/') + 1);
    NODELET_INFO_STREAM("Initialising nodelet... [" << namespace_ << "]");
    init_params();
    kitti_reader_ptr_ = std::make_unique<KittiReader>(nh_);
    if (kitti_reader_ptr_->initScanFilenames()) {
      NODELET_INFO_STREAM("Nodelet initialised. Spinning up update thread ... [" << namespace_ << "]");
      kitti_update_thread_.start(&KittiNode::update, *this);
      NODELET_INFO_STREAM("Nodelet initialised. [" << namespace_ << "]");
    } else {
      NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << namespace_ << "]");
    }
  }

private:
  void update() {
    static int file_index = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && !shutdown_requested_) {
      ros::spinOnce();
      kitti_reader_ptr_->spinOnce();
      ROS_INFO("Reading %d scan.", file_index);
      file_index++;
      loop_rate.sleep();
    }
  }

  void init_params() {
    auto &ph_ = private_nh_;
    cloud_topic_ = ph_.param<std::string>("cloud_topic", "/velodyne_points");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_raw_cloud_;
  std::string cloud_topic_;
  std::unique_ptr<KittiReader> kitti_reader_ptr_;
  ecl::Thread kitti_update_thread_;

  bool shutdown_requested_;
};

}

PLUGINLIB_EXPORT_CLASS(tloam::KittiNode, nodelet::Nodelet);
