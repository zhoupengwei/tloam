/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午9:33
 * @FileName: cloud_publisher.cpp
 * @Description: Publish Point Cloud
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/cloud_publisher.hpp"

namespace tloam {
CloudPublisher::CloudPublisher(ros::NodeHandle &nh, const std::string &topic_name, const std::string &frame_id,
                               size_t buff_size)
    : nh_(nh), frame_id_(frame_id){
  publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publisher(std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr) {
  ros::Time time = ros::Time::now();
  PublishData(cloud_ptr, time);
}

void CloudPublisher::Publisher(std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(cloud_ptr, ros_time);
}

void CloudPublisher::Publisher(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_ptr) {
  ros::Time time = ros::Time::now();
  PublishData(cloud_ptr, time);
}

void CloudPublisher::Publisher(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(cloud_ptr, ros_time);
}

void CloudPublisher::PublishData(std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr, ros::Time time) {
  sensor_msgs::PointCloud2Ptr msgs_ptr_{new sensor_msgs::PointCloud2()};
  open3d::conversions::Open3dToRos(*cloud_ptr, *msgs_ptr_);

  msgs_ptr_->header.stamp = time;
  msgs_ptr_->header.frame_id = frame_id_;

  publisher_.publish(*msgs_ptr_);
}

void CloudPublisher::PublishData(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_ptr, ros::Time time) {
  sensor_msgs::PointCloud2Ptr msgs_ptr_{new sensor_msgs::PointCloud2()};
  open3d::conversions::Open3dToRos(*cloud_ptr, *msgs_ptr_);

  msgs_ptr_->header.stamp = time;
  msgs_ptr_->header.frame_id = frame_id_;

  publisher_.publish(*msgs_ptr_);
}

bool CloudPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

}
