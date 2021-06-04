/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午11:04
 * @FileName: key_frames_publisher.cpp
 * @Description: publish key frames
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/key_frames_publisher.hpp"

namespace tloam {
KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle &nh, std::string &topic_name, std::string &frame_id,
                                       int buff_size) : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void KeyFramesPublisher::Publish(const std::deque<KeyFrame> &key_frames) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = frame_id_;

  for (auto key_frame : key_frames) {
    geometry_msgs::PoseStamped pose_stamped;
    ros::Time ros_time(key_frame.time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.header.seq = key_frame.index;

    pose_stamped.pose.position.x = key_frame.pose(0, 3);
    pose_stamped.pose.position.y = key_frame.pose(1, 3);
    pose_stamped.pose.position.z = key_frame.pose(2, 3);

    Eigen::Quaterniond q = key_frame.GetQuaternion();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    path.poses.push_back(pose_stamped);
  }

  publisher_.publish(path);
}

bool KeyFramesPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
}



