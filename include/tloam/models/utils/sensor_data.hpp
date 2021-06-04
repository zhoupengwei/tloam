/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午10:35
 * @FileName: key_frame.hpp
 * @Description: store key frames information
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_SENSOR_DATA_HPP
#define TLOAM_SENSOR_DATA_HPP

#include <Eigen/Dense>
#include "tloam/open3d/open3d_to_ros.hpp"

namespace tloam {
class CloudData {
public:
  CloudData() : time(0.0) {
    cloud_ptr = std::make_shared<open3d::geometry::PointCloud2>();
  }

  void reset() {
    cloud_ptr.reset(new open3d::geometry::PointCloud2);
    time = 0.0;
  }

  CloudData(const CloudData& src){
    cloud_ptr = src.cloud_ptr;
    time = src.time;
  }

  CloudData& operator=(const CloudData& src) noexcept{
    if (this == &src)
      return *this;
    cloud_ptr.reset(new open3d::geometry::PointCloud2);
    cloud_ptr = src.cloud_ptr;
    time = src.time;
    return *this;
  }

  CloudData(CloudData&& src){
    cloud_ptr.reset(new open3d::geometry::PointCloud2(*src.cloud_ptr));
    src.cloud_ptr = nullptr;
  }

  CloudData&operator=(CloudData&& src) noexcept{
    if (this == &src)
      return *this;
    cloud_ptr.reset(new open3d::geometry::PointCloud2(*src.cloud_ptr));
    src.cloud_ptr = nullptr;
    return *this;
  }

  double time{0.0};
  std::shared_ptr<open3d::geometry::PointCloud2> cloud_ptr{nullptr};
};

class KeyFrame {
public:
  double time{0.0};
  size_t index{0};
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

public:
  Eigen::Quaterniond GetQuaternion() {
    Eigen::Quaterniond q;
    q = pose.linear();
    return q;
  }
};

class PoseData {
public:
  double time{0.0};
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();

public:
  Eigen::Quaterniond GetQuaternion();
};

}

#endif //TLOAM_SENSOR_DATA_HPP
