/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午5:02
 * @FileName: tf_listener.hpp
 * @Description: tf monitor the transformation between the child_frame and base_frame
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_TF_LISTENER_HPP
#define TLOAM_TF_LISTENER_HPP

#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace tloam {
class TFListener {
public:
  TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id);

  TFListener() = default;

  bool LookupData(Eigen::Isometry3d& transform_matrix);
  bool LookupData(Eigen::Matrix4d& transform_matrix);

private:
  bool TransformToMatrix(const tf::StampedTransform &transform, Eigen::Isometry3d& transform_matrix);

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_;
  std::string child_frame_id_;
};
}


#endif //TLOAM_TF_LISTENER_HPP
