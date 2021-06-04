/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午1:45
 * @FileName: odometry_publisher.cpp
 * @Description: publish the pose of laser odometry
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/odometry_publisher.hpp"

namespace tloam {

OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh, const std::string &topic_name,
                                     const std::string &base_frame_id, const std::string &child_frame_id,
                                     size_t buff_size) : nh_(nh) {
  publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const Eigen::Isometry3d &transform_matrix) {
  PublishData(transform_matrix, ros::Time::now());
}

void OdometryPublisher::Publish(const Eigen::Isometry3d &transform_matrix, double time) {
  ros::Time ros_time(static_cast<float>(time));
  PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::PublishData(const Eigen::Isometry3d &transform_matrix, ros::Time time) {
  odometry_.header.stamp = time;

  /// set the position
  odometry_.pose.pose.position.x = transform_matrix.translation().x();
  odometry_.pose.pose.position.y = transform_matrix.translation().y();
  odometry_.pose.pose.position.z = transform_matrix.translation().z();

  /// set orientation
  Eigen::Quaterniond q;
  q = transform_matrix.linear();
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  publisher_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
}

