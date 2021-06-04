/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/10 下午4:35
 * @FileName: registration_interface.hpp.h
 * @Description: ${DESCRIPTION}
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_REGISTRATION_INTERFACE_HPP
#define TLOAM_REGISTRATION_INTERFACE_HPP

#include <Eigen/Dense>
#include "tloam/models/utils/sensor_data.hpp"


namespace tloam{

struct Frame{
  Frame() : scan_cloud(new open3d::geometry::PointCloud2),
            edge_feature(new open3d::geometry::PointCloud2),
            sphere_feature(new open3d::geometry::PointCloud2),
            planar_feature(new open3d::geometry::PointCloud2),
            ground_feature(new open3d::geometry::PointCloud2){ };
  std::shared_ptr<open3d::geometry::PointCloud2> scan_cloud;
  std::shared_ptr<open3d::geometry::PointCloud2> edge_feature;
  std::shared_ptr<open3d::geometry::PointCloud2> sphere_feature;
  std::shared_ptr<open3d::geometry::PointCloud2> planar_feature;
  std::shared_ptr<open3d::geometry::PointCloud2> ground_feature;

  void reset(void){
    scan_cloud.reset(new open3d::geometry::PointCloud2);
    edge_feature.reset(new open3d::geometry::PointCloud2);
    sphere_feature.reset(new open3d::geometry::PointCloud2);
    planar_feature.reset(new open3d::geometry::PointCloud2);
    ground_feature.reset(new open3d::geometry::PointCloud2);
  }
};

class RegistrationInterface{
public:
  RegistrationInterface() = default;
  virtual ~RegistrationInterface() = default;
  virtual bool setInputSource(Frame& cloud_in_) = 0;
  virtual bool setInputTarget(Frame& cloud_in_) = 0;
  virtual bool scanMatching(Frame& out_result_, Eigen::Isometry3d& predict_pose_, Eigen::Isometry3d& result_pose_) = 0;
  virtual std::pair<double, double> getFitnessScore() = 0;
};
}
#endif //TLOAM_REGISTRATION_INTERFACE_HPP
