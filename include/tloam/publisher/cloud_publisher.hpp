/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午9:33
 * @FileName: cloud_publisher.hpp
 * @Description: Publish Point Cloud
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_CLOUD_PUBLISHER_HPP
#define TLOAM_CLOUD_PUBLISHER_HPP

#include <iostream>
#include <ros/ros.h>
#include <open3d/Open3D.h>
#include <sensor_msgs/PointCloud2.h>
#include "tloam/open3d/PointCloud2.hpp"
#include "tloam/open3d/open3d_to_ros.hpp"

namespace tloam {
class CloudPublisher {
public:
  CloudPublisher() = default;

  CloudPublisher(ros::NodeHandle &nh, const std::string &topic_name, const std::string &frame_id, size_t buff_size);

  void Publisher(std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr);

  void Publisher(std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr, double time);

  void Publisher(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_ptr);

  void Publisher(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_ptr, double time);

  bool HasSubscribers();

private:
  void PublishData(std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr, ros::Time time);

  void PublishData(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_ptr, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};
}


#endif //TLOAM_CLOUD_PUBLISHER_HPP
