/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午2:03
 * @FileName: tf_broadcaster.cpp
 * @Description: publish tf transformation between different coordinate systems
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/tf_broadcaster.hpp"

namespace tloam {
TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
  transform_.frame_id_ = frame_id;
  transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(Eigen::Isometry3d &pose, double time) {
  Eigen::Quaterniond q;
  q = pose.linear();
  ros::Time ros_time(static_cast<float>(time));
  transform_.stamp_ = ros_time;
  transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  transform_.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
  broadcaster_.sendTransform(transform_);
}
}