/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午1:44
 * @FileName: odometry_publisher.hpp
 * @Description: publish the pose of LiDAR Odometry
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_ODOMETRY_PUBLISHER_HPP
#define TLOAM_ODOMETRY_PUBLISHER_HPP

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

namespace tloam {
class OdometryPublisher {
public:
  OdometryPublisher() = default;

  OdometryPublisher(ros::NodeHandle &nh,
                    const std::string &topic_name,
                    const std::string &base_frame_id,
                    const std::string &child_frame_id,
                    size_t buff_size);

  void Publish(const Eigen::Isometry3d &transform_matrix);

  void Publish(const Eigen::Isometry3d &transform_matrix, double time);

  bool HasSubscribers();

private:
  void PublishData(const Eigen::Isometry3d &transform_matrix, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  nav_msgs::Odometry odometry_;
};

}


#endif //TLOAM_ODOMETRY_PUBLISHER_HPP
