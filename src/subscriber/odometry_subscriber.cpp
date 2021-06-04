/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午3:20
 * @FileName: odometry_subscriber.cpp
 * @Description: Subscribe odometry of specified topic
 * @License: See LICENSE for the license information
 */

#include "tloam/subscriber/odometry_subscriber.hpp"

namespace tloam {
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle &nh,
                                       std::string &topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
}

void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr &odom_msg_ptr) {
  buff_mutex_.lock();
  PoseData pose_data;
  pose_data.time = odom_msg_ptr->header.stamp.toSec();

  pose_data.pose.translation().x() = odom_msg_ptr->pose.pose.position.x;
  pose_data.pose.translation().y() = odom_msg_ptr->pose.pose.position.y;
  pose_data.pose.translation().z() = odom_msg_ptr->pose.pose.position.z;

  Eigen::Quaterniond q;
  q.x() = odom_msg_ptr->pose.pose.orientation.x;
  q.y() = odom_msg_ptr->pose.pose.orientation.y;
  q.z() = odom_msg_ptr->pose.pose.orientation.z;
  q.w() = odom_msg_ptr->pose.pose.orientation.w;
  pose_data.pose.linear() = q.matrix();

  pose_data.vel.x() = odom_msg_ptr->twist.twist.linear.x;
  pose_data.vel.y() = odom_msg_ptr->twist.twist.linear.y;
  pose_data.vel.z() = odom_msg_ptr->twist.twist.linear.z;

  new_pose_data_.emplace_back(pose_data);
  buff_mutex_.unlock();
}

void OdometrySubscriber::ParseData(std::deque<PoseData> &pose_data_buff) {
  buff_mutex_.lock();
  if (new_pose_data_.size() > 0) {
    pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
    new_pose_data_.clear();
  }
  buff_mutex_.unlock();
}

}