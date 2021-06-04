/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午3:03
 * @FileName: key_frames_subscriber.cpp
 * @Description: subscribe key-frames of specified topic
 * @License: See LICENSE for the license information
 */

#include "tloam/subscriber/key_frames_subscriber.hpp"

namespace tloam{
KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle &nh, std::string &topic_name,
                                         size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFramesSubscriber::msg_callback, this);
}

void KeyFramesSubscriber::msg_callback(const nav_msgs::Path::ConstPtr &key_frames_msg_ptr) {
  buff_mutex_.lock();
  new_key_frames_.clear();

  for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); i++) {
    KeyFrame key_frame;
    key_frame.time = key_frames_msg_ptr->poses.at(i).header.stamp.toSec();
    key_frame.index = static_cast<size_t>(i);

    key_frame.pose.translation().x() = key_frames_msg_ptr->poses.at(i).pose.position.x;
    key_frame.pose.translation().y() = key_frames_msg_ptr->poses.at(i).pose.position.y;
    key_frame.pose.translation().z() = key_frames_msg_ptr->poses.at(i).pose.position.z;

    Eigen::Quaterniond q;
    q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
    q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.y;
    q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.z;
    q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.w;
    key_frame.pose.linear() = q.matrix();

    new_key_frames_.emplace_back(key_frame);
  }
  buff_mutex_.unlock();
}

void KeyFramesSubscriber::ParseData(std::deque<KeyFrame> &key_frames_buff) {
  buff_mutex_.lock();
  if (new_key_frames_.size() > 0) {
    key_frames_buff = new_key_frames_;
    new_key_frames_.clear();
  }
  buff_mutex_.unlock();
}
}

