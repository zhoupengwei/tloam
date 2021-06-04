/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午2:03
 * @FileName: tf_broadcaster.hpp
 * @Description: publish tf transformation between different coordinate systems
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_TF_BROADCASTER_HPP
#define TLOAM_TF_BROADCASTER_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace tloam {
class TFBroadCaster {
public:
  TFBroadCaster() = default;

  ~TFBroadCaster() = default;

  TFBroadCaster(std::string frame_id, std::string child_frame_id);

  void SendTransform(Eigen::Isometry3d &pose, double time);

private:
  tf::StampedTransform transform_{};
  tf::TransformBroadcaster broadcaster_{};
};
}

#endif //TLOAM_TF_BROADCASTER_HPP
