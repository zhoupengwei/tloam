/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午2:46
 * @FileName: key_frame_subscriber.cpp
 * @Description: subscribe key-frame of specified topic
 * @License: See LICENSE for the license information
 */

#include "tloam/subscriber/key_frame_subscriber.hpp"

namespace tloam{
KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle &nh, std::string &topic_name,
                                      size_t buff_size) : nh_(nh) {
   subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::msg_callback, this);
}

void KeyFrameSubscriber::msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &key_frame_msg_ptr) {
   buff_mutex_.lock();
   KeyFrame key_frame;
   key_frame.time = key_frame_msg_ptr->header.stamp.toSec();
   key_frame.index = static_cast<size_t>(key_frame_msg_ptr->pose.covariance[0]);

   key_frame.pose.translation().x() = key_frame_msg_ptr->pose.pose.position.x;
   key_frame.pose.translation().y() = key_frame_msg_ptr->pose.pose.position.y;
   key_frame.pose.translation().z() = key_frame_msg_ptr->pose.pose.position.z;

   Eigen::Quaterniond q;
   q.x() = key_frame_msg_ptr->pose.pose.orientation.x;
   q.y() = key_frame_msg_ptr->pose.pose.orientation.y;
   q.z() = key_frame_msg_ptr->pose.pose.orientation.z;
   q.w() = key_frame_msg_ptr->pose.pose.orientation.w;
   key_frame.pose.linear() = q.matrix();

   new_key_frame_.emplace_back(key_frame);
   buff_mutex_.unlock();
}

void KeyFrameSubscriber::ParseData(std::deque<KeyFrame> &key_frame_buff) {
   buff_mutex_.lock();
   if (new_key_frame_.size() > 0){
       key_frame_buff.insert(key_frame_buff.end(), new_key_frame_.begin(), new_key_frame_.end());
       new_key_frame_.clear();
   }
   buff_mutex_.unlock();
}
}
