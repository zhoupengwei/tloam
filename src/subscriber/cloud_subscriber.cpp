/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午2:31
 * @FileName: cloud_subscriber.cpp
 * @Description: subscribe point cloud of specified topic
 * @License: See LICENSE for the license information
 */

#include "tloam/subscriber/cloud_subscriber.hpp"

namespace tloam {
CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string &topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, const char *topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr) {
  buff_mutex_.lock();
  CloudData cloud_data;
  cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
  open3d::conversions::RosToOpen3d(*cloud_msg_ptr, *cloud_data.cloud_ptr);

  new_cloud_data_.emplace_back(cloud_data);
  buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff) {
  buff_mutex_.lock();
  if (!new_cloud_data_.empty()) {
    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }
  buff_mutex_.unlock();
}
}