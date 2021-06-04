/**
 * @Copyright 2021, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/1 上午9:55
 * @FileName: segmentation_nodelet.cpp
 * @Description: Complete the segmentation of point cloud
 * @License: See LICENSE for the license information
 */

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "tloam/models/segmentation/segmentation.hpp"
#include "tloam/models/utils/utils.hpp"


namespace tloam{
class SegmentationNode : public nodelet::Nodelet {
public:
  SegmentationNode() : shutdown_requested_(false){

  }

  ~SegmentationNode() override{
    NODELET_DEBUG_STREAM("Waiting for Segmentation Node to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
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
    segmentation_ptr_ = std::make_shared<Segmentation>(nh_, cloud_topic);

    if (segmentation_ptr_->initWithConfig()){
      NODELET_INFO_STREAM("Nodelet initialised. Spinning up update thread ...[" << namespace_ << "]");
      update_thread_.start(&SegmentationNode::update, *this);
      NODELET_INFO_STREAM("Nodelet finish initialising. [" << namespace_ << "]");
    }else{
      NODELET_ERROR_STREAM("Cloudn't initialise Nodelet! Please check parameters. [" << namespace_ << "]");
    }
  }

private:
  void update(void){
    ros::Rate loop_rate(10);

    while (!shutdown_requested_ && ros::ok()){
      ros::spinOnce();
      segmentation_ptr_->spinOnce();
      ros::Duration(0.001).sleep();
    }
  }

  bool init_params(void){
    auto& ph = private_nh_;
    cloud_topic = ph.param<std::string>("cloud_topic", "/velodyne_points");

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::shared_ptr<Segmentation> segmentation_ptr_;
  std::string cloud_topic;

  ecl::Thread update_thread_;
  bool shutdown_requested_;
};
}

PLUGINLIB_EXPORT_CLASS(tloam::SegmentationNode, nodelet::Nodelet);
