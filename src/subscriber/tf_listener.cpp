/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午5:02
 * @FileName: tf_listener.cpp
 * @Description: tf monitor the transformation between the child_frame and base_frame
 * @License: See LICENSE for the license information
 */

#include "tloam/subscriber/tf_listener.hpp"

namespace tloam {
TFListener::TFListener(ros::NodeHandle &nh,
                       std::string base_frame_id,
                       std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {

}

bool TFListener::LookupData(Eigen::Isometry3d& transform_matrix) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
    TransformToMatrix(transform, transform_matrix);
    return true;
  }
  catch (tf::TransformException &ex) {
    return false;
  }
}

bool TFListener::LookupData(Eigen::Matrix4d &transform_matrix) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
    Eigen::Isometry3d transform_info;
    TransformToMatrix(transform, transform_info);
    transform_matrix = transform_info.matrix();
    return true;
  }
  catch (tf::TransformException &ex) {
    return false;
  }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform &transform, Eigen::Isometry3d& transform_matrix) {
  Eigen::Translation3d tf_trans(transform.getOrigin().getX(), transform.getOrigin().getY(),
                                transform.getOrigin().z());

  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());

  transform_matrix.matrix() = (tf_trans * rot_z * rot_y * rot_x).matrix();

  return true;
}

}



