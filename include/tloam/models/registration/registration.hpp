/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/10 下午9:11
 * @FileName: registration.hpp
 * @Description: T-LOAM frontend Lidar odometry
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_REGISTRATION_HPP
#define TLOAM_REGISTRATION_HPP

#include <thread>
#include <deque>
#include <mutex>
#include <future>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <yaml-cpp/yaml.h>
#include <omp.h>

#include <open3d/Open3D.h>

#include "third_party/sophus/so3.hpp"
#include "third_party/sophus/se3.hpp"

#include "tloam/models/utils/work_space_path.h"
#include "tloam/models/registration/registration_interface.hpp"
#include "tloam/open3d/open3d_to_ros.hpp"

namespace tloam{

class PointToPointErr : public ceres::SizedCostFunction<3, 6>{
public:
  PointToPointErr(Eigen::Vector3d& source_, Eigen::Vector3d& target_, double& weight_, double* cost_);

  ~PointToPointErr() override { }

  virtual bool Evaluate(
    double const* const* parameters,
    double* residuals,
    double** jacobians
  ) const;

public:
  Eigen::Vector3d target;
  Eigen::Vector3d source;
  double weight;
  mutable double* cost;
};

class PointToLineErr : public ceres::SizedCostFunction<3, 6>{
public:
  PointToLineErr(
    Eigen::Vector3d& curr_point_,
    Eigen::Vector3d& line_point_a_,
    Eigen::Vector3d& line_point_b_,
    double& weight_, double* cost_
  );

  ~PointToLineErr() override { }

  virtual bool Evaluate(
    double const* const* parameters,
    double* residuals,
    double** jacobians
  ) const;

public:
  Eigen::Vector3d curr_point;
  Eigen::Vector3d line_point_a;
  Eigen::Vector3d line_point_b;
  double weight;
  mutable double* cost;
};

class PointToPlaneErr : public ceres::SizedCostFunction<1, 6>{
public:
  PointToPlaneErr(Eigen::Vector3d curr_point_, Eigen::Vector3d& unit_norm_, double& devia_, double& weight_, double* cost_);

  ~PointToPlaneErr() override { }

  virtual bool Evaluate(
    double const* const* parameters,
    double* residuals,
    double** jacobians
  ) const;

public:
  Eigen::Vector3d curr_point;
  Eigen::Vector3d unit_norm;
  double devia;
  double weight;
  mutable double* cost;
};

class PlaneToPlaneErr : public ceres::SizedCostFunction<3, 6>{
public:
  PlaneToPlaneErr(
    Eigen::Vector3d& source_point_,
    Eigen::Matrix3d& source_covs_,
    Eigen::Vector3d& target_point_,
    Eigen::Matrix3d& target_covs_,
    double& weight_, double* cost_
  );

  ~PlaneToPlaneErr() override { }

  virtual bool Evaluate(
    double const* const* parameters,
    double* residuals,
    double** jacobians
  ) const;

public:
  Eigen::Vector3d source_point;
  Eigen::Matrix3d source_covs;
  Eigen::Vector3d target_point;
  Eigen::Matrix3d target_covs;
  double weight;
  mutable double* cost;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
  PoseSE3Parameterization() = default;
  ~PoseSE3Parameterization() override = default;

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
  bool ComputeJacobian(const double* x, double* jacobian) const override;
  int GlobalSize() const override{
    return 6;
  }
  int LocalSize() const override{
    return 6;
  }
};

// local registration based on truncated least squares method
class LocalRegistration : public RegistrationInterface{
public:
  enum Factor{
    planar = 2,
    planarEdge = 3,
    planarEdgeSphere = 4
  };
  LocalRegistration(const YAML::Node& config_node);
  ~LocalRegistration() override;

  void initConfig(const YAML::Node& node);

  Eigen::Isometry3d getTransform(void);

  Eigen::Isometry3d getPoseIncrement(void);

  bool setInputSource(Frame& cloud_in_) override;
  bool setInputTarget(Frame& cloud_in_) override;
  bool scanMatching(Frame& out_result_, Eigen::Isometry3d& predict_pose_, Eigen::Isometry3d& result_pose_) override;

  std::pair<double, double> getFitnessScore() override;

  void resetKDTree(void);

protected:
  /**
   * @brief fit the best plane from the point cloud set
   * @param point_set_, input point set
   * @return plane model
   */
  Eigen::Vector4d fitBestPlane(std::vector<Eigen::Vector3d>& point_set_);

  /**
   * @brief calculate the covariance matrix
   * @param cloud_in_, input point cloud
   * @param out_covs_, output covariance matrix
   * @return true if success otherwise false
   */
  bool calculateCov(
    const std::shared_ptr<open3d::geometry::PointCloud2>& cloud_in_,
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& out_covs_
  );

  /**
   * @brief add edge features factor to ceres residual function
   * @param scan_edge_, current frame edge features
   * @param submap_edge_, edge_features in submap
   * @param edge_weights_, the weight value of each feature
   * @param edge_residuals_, the residual value of each feature
   * @param problem_, ceres problem
   * @param loss_function_, cerese loss function
   * @return true if success otherwise false
   */
  bool addEdgeCostFactor(
    const std::shared_ptr<open3d::geometry::PointCloud2>& scan_edge_,
    const std::shared_ptr<open3d::geometry::PointCloud2>& submap_edge_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& edge_weights_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& edge_residuals_,
    ceres::Problem& problem_,
    ceres::LossFunction* loss_function_
  );

  /**
   * @brief add sphere features factor to ceres residual function
   * @param scan_sphere_, current frame sphere features
   * @param submap_sphere_, sphere features in submap
   * @param sphere_weights_, the weight value of each feature
   * @param sphere_residuals_, the residual value of each feature
   * @param problem_, ceres problem
   * @param loss_function_, ceres loss function
   * @return true if success otherwise false
   */
  bool addSphereCostFactor(
    const std::shared_ptr<open3d::geometry::PointCloud2>& scan_sphere_,
    const std::shared_ptr<open3d::geometry::PointCloud2>& submap_sphere_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& sphere_weights_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& sphere_residuals_,
    ceres::Problem& problem_,
    ceres::LossFunction* loss_function_
  );

  /**
   * @brief add planar features factor to ceres residual function point_to_plane
   * @param scan_plane_, current frame planar features
   * @param submap_plane_, planar features in submap
   * @param plane_weights_, the weight of each feature
   * @param plane_residuals_, the residual value of each feature
   * @param problem, ceres problem
   * @param loss_function, ceres loss function
   * @return true if success otherwise false
   */
  bool addSurfCostFactor(
    const std::shared_ptr<open3d::geometry::PointCloud2>& scan_plane_,
    const std::shared_ptr<open3d::geometry::PointCloud2>& submap_plane_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& plane_weights_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& plane_residuals_,
    ceres::Problem& problem,
    ceres::LossFunction* loss_function
  );

  /**
   * @brief add planar features factor to ceres residual function plane_to_plane
   * @param scan_plane_, current frame planar features
   * @param submap_sphere_, planar features in submap
   * @param scan_plane_covs_, scan_plane covariance
   * @param submap_plane_covs_, submap_plane_ covariance
   * @param plane_weights_, the weight of each feature
   * @param plane_residuals_, the residual value of each feature
   * @param problem, ceres problem
   * @param loss_function, ceres loss function
   * @return true if success otherwise false
   */
  bool addSurfCostFactor2(
    const std::shared_ptr<open3d::geometry::PointCloud2>& scan_plane_,
    const std::shared_ptr<open3d::geometry::PointCloud2>& submap_plane_,
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& scan_plane_covs_,
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& submap_plane_covs_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& plane_weights_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& plane_residuals_,
    ceres::Problem& problem,
    ceres::LossFunction* loss_function
  );

  /**
   * @brief add ground features factor to ceres residual function
   * @param scan_ground_, current frame ground features
   * @param submap_ground_, ground features in submap
   * @param ground_weights_, the weight of each feature
   * @param ground_residuals_, the residual of each feature
   * @param problem, ceres problem
   * @param loss_function, ceres loss function
   * @return true if success otherwise false
   */
  bool addGroundCostFactor(
    const std::shared_ptr<open3d::geometry::PointCloud2>& scan_ground_,
    const std::shared_ptr<open3d::geometry::PointCloud2>& submap_ground_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& ground_weights_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& ground_residuals_,
    ceres::Problem& problem,
    ceres::LossFunction* loss_function
  );

  /**
   * @brief add ground features factor to ceres residual function
   * @param scan_ground_, current frame ground features
   * @param submap_ground_, ground features in submap
   * @param scan_ground_covs_, scan_ground covariance
   * @param submap_ground_covs_, submap_ground covariance
   * @param ground_weights_, the weight of each feature
   * @param ground_residuals_, the residual of each feature
   * @param problem, ceres problem
   * @param loss_function, ceres loss function
   * @return true if success otherwise false
   */
  bool addGroundCostFactor2(
      const std::shared_ptr<open3d::geometry::PointCloud2>& scan_ground_,
      const std::shared_ptr<open3d::geometry::PointCloud2>& submap_ground_,
      std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& scan_ground_covs_,
      std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& submap_ground_covs_,
      Eigen::Matrix<double, 1, Eigen::Dynamic>& ground_weights_,
      Eigen::Matrix<double, 1, Eigen::Dynamic>& ground_residuals_,
      ceres::Problem& problem,
      ceres::LossFunction* loss_function
  );

  /**
   * @brief update corresponding features weights
   * @param weights_, output weights
   * @param residuals_, feature residuals
   * @param totalSize_, feature size
   * @param noise_bound_sq_, noise upper bound
   * @param th1_, gnc control threshold, the weight is assigned one if the rq less than the th1
   * @param th2_, gnc control threshold, the weight is assigned zero if the rq more than the th2
   * @param mu_, gnc control parameters
   * @return true if success otherwise false
   */
  bool updateWeight(
    Eigen::Matrix<double, 1, Eigen::Dynamic>& weights_,
    Eigen::Matrix<double, 1, Eigen::Dynamic>& residuals_,
    double totalSize_, double noise_bound_sq_,
    double th1_, double th2_,
    double mu_
  );

private:
  // Note with optimized pose, the front 3d represents translation, and the back 3D represents rotation.
  double parameters[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Eigen::Map<Eigen::Matrix<double, 6, 1>> se3_pose_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(parameters);
  int num_threads_;
  int k_corr;

  int factor_num;
  double edge_dist_thres, sphere_dist_thres, planar_dist_thres, ground_dist_thres, edge_dir_thres, fitness_thres;
  int edge_maxnum, sphere_maxnum, planar_maxnum, ground_maxnum;
  std::mutex factor_mutex;

  int max_iterations, nr_iterations;
  double cost_threshold, gnc_factor, noise_bound;

  Eigen::Isometry3d curr_frame_pose;
  Eigen::Isometry3d last_frame_pose;

  std::shared_ptr<open3d::geometry::PointCloud2> scan_planar;
  std::shared_ptr<open3d::geometry::PointCloud2> scan_edge;
  std::shared_ptr<open3d::geometry::PointCloud2> scan_sphere;
  std::shared_ptr<open3d::geometry::PointCloud2> scan_ground;

  std::shared_ptr<open3d::geometry::PointCloud2> submap_planar;
  std::shared_ptr<open3d::geometry::PointCloud2> submap_edge;
  std::shared_ptr<open3d::geometry::PointCloud2> submap_sphere;
  std::shared_ptr<open3d::geometry::PointCloud2> submap_ground;

  std::shared_ptr<open3d::geometry::KDTreeFlann> planar_kd_tree;
  std::shared_ptr<open3d::geometry::KDTreeFlann> edge_kd_tree;
  std::shared_ptr<open3d::geometry::KDTreeFlann> sphere_kd_tree;
  std::shared_ptr<open3d::geometry::KDTreeFlann> ground_kd_tree;

  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> source_planar_covs;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> submap_planar_covs;

  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> source_ground_covs;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> submap_ground_covs;

};

// waiting for code coming
class GlobalRegistration : public RegistrationInterface{
  GlobalRegistration();
  ~GlobalRegistration() override;

  bool setInputSource(Frame& cloud_in_) override;
  bool setInputTarget(Frame& cloud_in_) override;
  bool scanMatching(Frame& out_result_, Eigen::Isometry3d& predict_pose_, Eigen::Isometry3d& result_pose_) override;
  std::pair<double, double> getFitnessScore() override;
};

}
#endif //TLOAM_REGISTRATION_HPP
