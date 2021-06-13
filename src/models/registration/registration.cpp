/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/10 下午9:12
 * @FileName: registration.cpp.c
 * @Description: ${DESCRIPTION}
 * @License: See LICENSE for the license information
 */

#include "tloam/models/registration/registration.hpp"

namespace tloam{

PointToPointErr::PointToPointErr(Eigen::Vector3d &source_, Eigen::Vector3d &target_, double &weight_, double *cost_)
  : target(target_), source(source_), weight(weight_), cost(cost_){

}

bool PointToPointErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(parameters[0]);

  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);

  auto source_world = T * source;

  Eigen::Vector3d res = target - source_world;

  residuals[0] = res[0] * weight;
  residuals[1] = res[1] * weight;
  residuals[2] = res[2] * weight;

  *cost = std::pow((residuals[0] + residuals[1] + residuals[2]), 2);

  if (jacobians != nullptr){
    if (jacobians[0] != nullptr){
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_se3_(jacobians[0]);

      Eigen::Matrix<double, 3, 6> dt_by_se3 = Eigen::Matrix<double, 3, 6>::Zero();
      dt_by_se3.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity() * weight;
      dt_by_se3.block(0, 3, 3, 3) = Sophus::SO3d::hat(source_world) * weight;

      J_se3_ = dt_by_se3;
    }
  }

  return true;
}

PointToLineErr::PointToLineErr(Eigen::Vector3d &curr_point_, Eigen::Vector3d &line_point_a_,
                               Eigen::Vector3d &line_point_b_, double &weight_, double *cost_)
  : curr_point(curr_point_), line_point_a(line_point_a_), line_point_b(line_point_b_), weight(weight_), cost(cost_){

}

bool PointToLineErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(parameters[0]);

  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);

  Eigen::Vector3d curr_world = T * curr_point;

  Eigen::Vector3d nu = (curr_world - line_point_a).cross(curr_world - line_point_b);
  Eigen::Vector3d de = line_point_a - line_point_b;

  residuals[0] = nu.x() / de.norm() * weight;
  residuals[1] = nu.y() / de.norm() * weight;
  residuals[2] = nu.z() / de.norm() * weight;

  *cost = std::pow((residuals[0] + residuals[1] + residuals[2]), 2);

  if (jacobians != nullptr){
    if (jacobians[0] != nullptr){
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_se3_(jacobians[0]);

      Eigen::Matrix<double, 3, 6> dt_by_se3 = Eigen::Matrix<double, 3, 6>::Zero();

      dt_by_se3.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * weight;
      dt_by_se3.block(0, 3, 3, 3) = -Sophus::SO3d::hat(curr_world) * weight;

      Eigen::Vector3d re = line_point_b - line_point_a;
      auto skew_re = Sophus::SO3d::hat(re);

      J_se3_ = skew_re * dt_by_se3 / de.norm();
    }
  }

  return true;
}

PointToPlaneErr::PointToPlaneErr(Eigen::Vector3d curr_point_, Eigen::Vector3d &unit_norm_, double &devia_,
                                 double &weight_, double *cost_)
  : curr_point(curr_point_), unit_norm(unit_norm_), devia(devia_), weight(weight_), cost(cost_){

}

bool PointToPlaneErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(parameters[0]);
  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);
  Eigen::Vector3d curr_world = T * curr_point;
  residuals[0] = unit_norm.dot(curr_world) + devia;
  *cost = std::pow(residuals[0], 2);

  if (jacobians != nullptr){
    if (jacobians[0] != nullptr){
      Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J_se3_(jacobians[0]);

      Eigen::Matrix<double, 3, 6> dt_by_se3 = Eigen::Matrix<double, 3, 6>::Zero();

      dt_by_se3.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * weight;
      dt_by_se3.block(0, 3, 3, 3) = -Sophus::SO3d::hat(curr_world) * weight;

      J_se3_.block<1, 6>(0, 0) = unit_norm.transpose() * dt_by_se3;
    }
  }

  return true;
}

PlaneToPlaneErr::PlaneToPlaneErr(Eigen::Vector3d &source_point_, Eigen::Matrix3d &source_covs_,
                                 Eigen::Vector3d &target_point_, Eigen::Matrix3d &target_covs_, double &weight_,
                                 double *cost_)
  : source_point(source_point_), source_covs(source_covs_), target_point(target_point_), target_covs(target_covs_), weight(weight_), cost(cost_){

}

bool PlaneToPlaneErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(parameters[0]);

  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);
  Eigen::Matrix3d R = T.matrix().block<3, 3>(0, 0);

  Eigen::Vector3d source_world = T * source_point;
  Eigen::Vector3d d = target_point - source_world;

  Eigen::Matrix3d RCR = target_covs + R * source_covs * R.transpose();
  Eigen::Matrix3d RCR_inv = RCR.inverse();
  Eigen::Vector3d RCR_d = RCR_inv * d;

  residuals[0] = RCR_d[0] * weight;
  residuals[1] = RCR_d[1] * weight;
  residuals[2] = RCR_d[2] * weight;

  *cost = std::pow((residuals[0] + residuals[1] + residuals[2]), 2);

  if (jacobians != nullptr){
    if (jacobians[0] != nullptr){
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_se3_(jacobians[0]);

      Eigen::Matrix<double, 3, 6> dt_by_se3 = Eigen::Matrix<double, 3, 6>::Zero();

      dt_by_se3.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity() * weight;
      dt_by_se3.block<3, 3>(0, 3) = Sophus::SO3d::hat(source_world) * weight;

      dt_by_se3 = RCR_inv * dt_by_se3.eval();
      J_se3_ = dt_by_se3;
    }
  }

  return true;
}

bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(x);
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_delta(delta);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> se3_x_plus_delta(x_plus_delta);

  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);
  Sophus::SE3d delta_T = Sophus::SE3d::exp(se3_delta);
  // Use left perturbation in the global coordinate system, use right perturbation in the local coordinate system
  se3_x_plus_delta = (delta_T * T).log();

  return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
  ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);

  return true;
}


LocalRegistration::LocalRegistration(const YAML::Node& config_node) {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  scan_planar = std::make_shared<open3d::geometry::PointCloud2>();
  scan_edge = std::make_shared<open3d::geometry::PointCloud2>();
  scan_sphere = std::make_shared<open3d::geometry::PointCloud2>();
  scan_ground = std::make_shared<open3d::geometry::PointCloud2>();

  submap_planar = std::make_shared<open3d::geometry::PointCloud2>();
  submap_edge = std::make_shared<open3d::geometry::PointCloud2>();
  submap_sphere = std::make_shared<open3d::geometry::PointCloud2>();
  submap_ground = std::make_shared<open3d::geometry::PointCloud2>();

  planar_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  edge_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  sphere_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  ground_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();

  // init config
  initConfig(config_node);
}

LocalRegistration::~LocalRegistration() noexcept {
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>().swap(source_planar_covs);
}

void LocalRegistration::initConfig(const YAML::Node &node) {
  k_corr = node["k_corr"].as<int>();
  factor_num = node["factor_num"].as<int>();
  edge_dist_thres = node["edge_dist_thres"].as<double>();
  sphere_dist_thres = node["sphere_dist_thres"].as<double>();
  planar_dist_thres = node["planar_dist_thres"].as<double>();
  ground_dist_thres = node["ground_dist_thres"].as<double>();
  edge_dir_thres = node["edge_dir_thres"].as<double>();
  edge_maxnum = node["edge_maxnum"].as<int>();
  sphere_maxnum = node["sphere_maxnum"].as<int>();
  planar_maxnum = node["planar_maxnum"].as<int>();
  ground_maxnum = node["ground_maxnum"].as<int>();

  max_iterations = node["max_iterations"].as<int>();
  cost_threshold = node["cost_threshold"].as<double>();
  gnc_factor = node["gnc_factor"].as<double>();
  noise_bound = node["noise_bound"].as<double>();
  fitness_thres = node["fitness_thres"].as<double>();
}

bool LocalRegistration::setInputSource(Frame &cloud_in_) {
  scan_edge = cloud_in_.edge_feature;
  scan_sphere = cloud_in_.sphere_feature;
  scan_planar = cloud_in_.planar_feature;
  scan_ground = cloud_in_.ground_feature;

  return true;
}

bool LocalRegistration::setInputTarget(Frame &cloud_in_) {
  submap_edge = cloud_in_.edge_feature;
  submap_sphere = cloud_in_.sphere_feature;
  submap_planar = cloud_in_.planar_feature;
  submap_ground = cloud_in_.ground_feature;

  return true;
}

void LocalRegistration::resetKDTree() {
  planar_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  edge_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  sphere_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  ground_kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
}

std::pair<double, double> LocalRegistration::getFitnessScore() {
  if (fitness_thres <= 0.0){
    ROS_WARN("The threshold value of fitness distance cannot be zero!");
    return std::make_pair(0.0, 0.0);
  }

  auto calculateFitness = [&](std::shared_ptr<open3d::geometry::PointCloud2>& source_, std::shared_ptr<open3d::geometry::PointCloud2>& target_,
      std::shared_ptr<open3d::geometry::KDTreeFlann>& kd_tree_)->std::pair<double, double>{
    double error{0.0};
    int corr_num = 0;
    size_t totalSize = source_->points_.size();
    for (size_t id = 0; id < totalSize; ++id){
      std::vector<int> knn_index(1);
      std::vector<double> knn_dist(1);
      Eigen::Vector3d curr_point = source_->points_[id];
      if (kd_tree_->SearchHybrid(curr_point, fitness_thres, 1, knn_index, knn_dist) > 0){
        error += knn_dist[0];
        corr_num++;
      }
    }

    if (corr_num == 0)
      return std::make_pair(0.0, 0.0);
    else{
      double fitness = static_cast<double>(corr_num) / static_cast<double>(source_->points_.size());
      double inlier_rmse = std::sqrt(error / static_cast<double>(corr_num));
      return std::make_pair(fitness, inlier_rmse);
    }
  };

  std::pair<double, double> edge_fitness = calculateFitness(scan_edge, submap_edge, edge_kd_tree);
  std::pair<double, double> sphere_fitness = calculateFitness(scan_sphere, submap_sphere, sphere_kd_tree);
  std::pair<double, double> planar_fitness = calculateFitness(scan_planar, submap_planar, planar_kd_tree);
  std::pair<double, double> ground_fitness = calculateFitness(scan_ground, submap_ground, ground_kd_tree);

  double totalFitness = edge_fitness.first + sphere_fitness.first + planar_fitness.first + ground_fitness.first;
  double totalRMSE = edge_fitness.second + sphere_fitness.second + planar_fitness.second + ground_fitness.second;

  return std::make_pair(totalFitness, totalRMSE);
}

/**
  * @brief fit the best plane from the point cloud set
  * @param point_set_, input point set
  * @return plane model
  */
Eigen::Vector4d LocalRegistration::fitBestPlane(std::vector<Eigen::Vector3d> &point_set_) {
  if (point_set_.empty()){
    ROS_ERROR("At least three points are needed to fit best plane. ");
    return Eigen::Vector4d::Zero();
  }

  double totalSize = static_cast<double>(point_set_.size());
  Eigen::Vector3d centroid(0, 0, 0);
  for (const auto& item : point_set_)
    centroid += item;
  centroid /= totalSize;

  double xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
  for (const auto& pt : point_set_){
    Eigen::Vector3d norm_pt = pt - centroid;
    xx += norm_pt(0) * norm_pt(0);
    xy += norm_pt(0) * norm_pt(1);
    xz += norm_pt(0) * norm_pt(2);
    yy += norm_pt(1) * norm_pt(1);
    yz += norm_pt(1) * norm_pt(2);
    zz += norm_pt(2) * norm_pt(2);
  }

  xx /= totalSize;
  xy /= totalSize;
  xz /= totalSize;
  yy /= totalSize;
  yz /= totalSize;
  zz /= totalSize;

  Eigen::Vector3d weighted_dir(0, 0, 0);

  {
    double det_x = yy*zz - yz*yz;
    Eigen::Vector3d axis_dir = Eigen::Vector3d(det_x, xz*yz - xy*zz, xy*yz - xz*yy);
    double weight = det_x * det_x;
    if (weighted_dir.dot(axis_dir) < 0.0)
      weight = -weight;
    weighted_dir += axis_dir * weight;
  }

  {
    double det_y = xx*zz - xz*xz;
    Eigen::Vector3d axis_dir = Eigen::Vector3d(xz*yz - xy*zz, det_y, xy*xz - yz*xx);
    double weight = det_y * det_y;
    if (weighted_dir.dot(axis_dir) < 0.0)
      weight = -weight;
    weighted_dir += axis_dir*weight;
  }
  {
    double det_z = xx*yy - xy*xy;
    Eigen::Vector3d axis_dir = Eigen::Vector3d(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
    double weight = det_z * det_z;
    if (weighted_dir.dot(axis_dir) < 0.0)
      weight = -weight;
    weighted_dir += axis_dir * weight;
  }

  double norm = weighted_dir.norm();
  if (norm == 0){
    return Eigen::Vector4d{0, 0, 0, 0};
  }
  weighted_dir.normalize();
  double d = -weighted_dir.dot(centroid);
  return Eigen::Vector4d{weighted_dir(0), weighted_dir(1), weighted_dir(2), d};
}

Eigen::Isometry3d LocalRegistration::getTransform() {
  return curr_frame_pose;
}

Eigen::Isometry3d LocalRegistration::getPoseIncrement() {
  return last_frame_pose.inverse() * curr_frame_pose;
}


/**
  * @brief calculate the covariance matrix
  * @param cloud_in_, input point cloud
  * @param out_covs_, output covariance matrix
  * @return true if success otherwise false
  */
bool LocalRegistration::calculateCov(const std::shared_ptr<open3d::geometry::PointCloud2> &cloud_in_,
                                     std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &out_covs) {
  std::shared_ptr<open3d::geometry::KDTreeFlann> kd_tree{new open3d::geometry::KDTreeFlann};
  kd_tree->SetGeometry(*cloud_in_);
  size_t totalSize = cloud_in_->points_.size();
  out_covs.resize(totalSize);

  for (size_t i = 0; i < totalSize; ++i){
    std::vector<int> neigh_index;
    std::vector<double> neigh_dist;
    kd_tree->SearchKNN(cloud_in_->points_[i], (k_corr + 1), neigh_index, neigh_dist);

    Eigen::Matrix<double, 3, -1> data(3, k_corr);

    for (size_t j = 1; j < neigh_index.size(); ++j){
      data.col(j-1) = cloud_in_->points_[neigh_index[j]];
    }

    data.colwise() -= data.rowwise().mean().eval();

    Eigen::Matrix3d cov = data * data.transpose() / k_corr;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d values = svd.singularValues() / svd.singularValues().maxCoeff();
    values = values.array().max(1e-3);

    out_covs[i] = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
  }

  return true;
}

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
bool LocalRegistration::addEdgeCostFactor(const std::shared_ptr<open3d::geometry::PointCloud2> &scan_edge_,
                                          const std::shared_ptr<open3d::geometry::PointCloud2> &submap_edge_,
                                          Eigen::Matrix<double, 1, Eigen::Dynamic> &edge_weights_,
                                          Eigen::Matrix<double, 1, Eigen::Dynamic> &edge_residuals_,
                                          ceres::Problem &problem_, ceres::LossFunction *loss_function_) {
  std::unique_lock<std::mutex> unique_lock(factor_mutex, std::defer_lock);
  unique_lock.lock();
  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose_);
  unique_lock.unlock();

  int edge_num = 0;
  size_t totalSize = scan_edge_->points_.size();
  for (size_t i = 0; i < totalSize; ++i){
    const auto& pt_w = T * scan_edge_->points_[i];

    std::vector<int> knn_index;
    std::vector<double> knn_dist;
    if (edge_kd_tree->SearchHybrid(pt_w, edge_dist_thres, 5, knn_index, knn_dist) > 0){
      if (knn_index.size() <= 3)
        continue;

      if (edge_num >= edge_maxnum)
        return true;

      Eigen::Matrix3d covariance;
      Eigen::Matrix<double, 9, 1> cumulants = Eigen::Matrix<double, 9, 1>::Zero();
      for (const auto& item : knn_index){
        const Eigen::Vector3d& pt = submap_edge_->points_[item];
        cumulants(0) += pt(0);
        cumulants(1) += pt(1);
        cumulants(2) += pt(2);
        cumulants(3) += pt(0) * pt(0);
        cumulants(4) += pt(0) * pt(1);
        cumulants(5) += pt(0) * pt(2);
        cumulants(6) += pt(1) * pt(1);
        cumulants(7) += pt(1) * pt(2);
        cumulants(8) += pt(2) * pt(2);
      }
      cumulants /= static_cast<double>(knn_index.size());
      covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
      covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
      covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
      covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
      covariance(1, 0) = covariance(0, 1);
      covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
      covariance(2, 0) = covariance(0, 2);
      covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
      covariance(2, 1) = covariance(1, 2);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
      solver.compute(covariance, Eigen::ComputeEigenvectors);
      Eigen::Vector3d eigen_value = solver.eigenvalues();
      Eigen::Vector3d unit_direction = solver.eigenvectors().col(2);

      if (eigen_value[2] > 3*eigen_value[1] && std::fabs(unit_direction.z()) > edge_dir_thres){
        Eigen::Vector3d pt_on_line = cumulants.block<3, 1>(0, 0);
        Eigen::Vector3d point_a = 0.1 * unit_direction + pt_on_line;
        Eigen::Vector3d point_b = -0.1 * unit_direction + pt_on_line;

        ceres::CostFunction* cost_function = new PointToLineErr(scan_edge_->points_[i], point_a, point_b, edge_weights_(i), &edge_residuals_(i));
        {
          unique_lock.lock();
          problem_.AddResidualBlock(cost_function, loss_function_, parameters);
          unique_lock.unlock();
        }
        edge_num++;
      }
    }

    std::vector<int>().swap(knn_index);
    std::vector<double>().swap(knn_dist);
  }

  if (edge_num <= 20){
    ROS_WARN("not enough edge points !!!");
  }

  return true;
}

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
bool LocalRegistration::addSphereCostFactor(const std::shared_ptr<open3d::geometry::PointCloud2> &scan_sphere_,
                                            const std::shared_ptr<open3d::geometry::PointCloud2> &submap_sphere_,
                                            Eigen::Matrix<double, 1, Eigen::Dynamic> &sphere_weights_,
                                            Eigen::Matrix<double, 1, Eigen::Dynamic> &sphere_residuals_,
                                            ceres::Problem &problem_, ceres::LossFunction *loss_function_) {
  std::unique_lock<std::mutex> unique_lock(factor_mutex, std::defer_lock);
  unique_lock.lock();
  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose_);
  unique_lock.unlock();

  int sphere_sum = 0;
  size_t totalSize = scan_sphere_->points_.size();

  for (size_t i = 0; i < totalSize; ++i){
    const auto& pt_w = T * scan_sphere_->points_[i];

    std::vector<int> knn_index;
    std::vector<double> knn_dist;
    if (sphere_kd_tree->SearchHybrid(pt_w, sphere_dist_thres, 1, knn_index, knn_dist) > 0){
      if (knn_index.empty() || knn_dist[0] > 0.2)
        continue;
      if (sphere_sum >= sphere_maxnum)
        return true;

      Eigen::Vector3d source = scan_sphere_->points_[i];
      Eigen::Vector3d target = submap_sphere_->points_[knn_index[0]];

      ceres::CostFunction* cost_function = new PointToPointErr(source, target, sphere_weights_(i), &sphere_residuals_(i));
      {
        unique_lock.lock();
        problem_.AddResidualBlock(cost_function, loss_function_, parameters);
        unique_lock.unlock();
      }
    }
    sphere_sum++;
  }

  if (sphere_sum <= 10){
    ROS_WARN("not enough sphere point..");
  }

  return true;
}

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
bool LocalRegistration::addSurfCostFactor(const std::shared_ptr<open3d::geometry::PointCloud2> &scan_plane_,
                                          const std::shared_ptr<open3d::geometry::PointCloud2> &submap_plane_,
                                          Eigen::Matrix<double, 1, Eigen::Dynamic> &plane_weights_,
                                          Eigen::Matrix<double, 1, Eigen::Dynamic> &plane_residuals_,
                                          ceres::Problem &problem, ceres::LossFunction *loss_function) {
    std::unique_lock<std::mutex> unique_lock(factor_mutex, std::defer_lock);
    unique_lock.lock();
    Sophus::SE3d T = Sophus::SE3d::exp(se3_pose_);
    unique_lock.unlock();

    int surf_num = 0;
    size_t totalSize = scan_plane_->points_.size();
    for (size_t i = 0; i < totalSize; ++i){
      const auto& pt_w = T * scan_plane_->points_[i];
      std::vector<int> knn_index;
      std::vector<double> knn_dist;

      if(planar_kd_tree->SearchHybrid(pt_w, planar_dist_thres, 5, knn_index, knn_dist) > 0){
        if (knn_index.size() <= 4)
          continue;

        if (surf_num >= planar_maxnum)
          return true;

        std::vector<Eigen::Vector3d> near_point;
        for (auto& id : knn_index){
          near_point.emplace_back(submap_plane_->points_[id]);
        }

        Eigen::Vector4d plane_model = fitBestPlane(near_point);
        Eigen::Vector3d unit_norm = plane_model.block<3, 1>(0, 0);
        double dot_norm = plane_model(3);

        bool planeVaild = true;
        for (auto& pt : near_point){
          double plane_dis = unit_norm.transpose() * pt + dot_norm;

          // check the plane model enough precision
          if (plane_dis > 0.2){
            planeVaild = false;
            break;
          }
        }

        if (planeVaild){
          ceres::CostFunction *costFunction = new PointToPlaneErr(scan_plane_->points_[i], unit_norm, dot_norm, plane_weights_(i), &plane_residuals_(i));
          {
            unique_lock.lock();
            problem.AddResidualBlock(costFunction, loss_function, parameters);
            unique_lock.unlock();
          }

          surf_num++;
        }
      }
    }

    //std::cout << "total add surf_num point: " << surf_num << std::endl;

    if(surf_num < 20){
      std::cout << "not enough ground points" << std::endl;
    }

    return true;
}

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
bool LocalRegistration::addSurfCostFactor2(const std::shared_ptr<open3d::geometry::PointCloud2> &scan_plane_,
                                           const std::shared_ptr<open3d::geometry::PointCloud2> &submap_plane_,
                                           std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& scan_plane_covs_,
                                           std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& submap_plane_covs_,
                                           Eigen::Matrix<double, 1, Eigen::Dynamic> &plane_weights_,
                                           Eigen::Matrix<double, 1, Eigen::Dynamic> &plane_residuals_,
                                           ceres::Problem &problem, ceres::LossFunction *loss_function) {
  std::unique_lock<std::mutex> unique_lock(factor_mutex, std::defer_lock);
  unique_lock.lock();
  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose_);
  unique_lock.unlock();

  std::vector<std::pair<int, int>> corres;
  size_t totalSize = scan_plane_->points_.size();
  for (size_t id = 0; id < totalSize; ++id){
    Eigen::Vector3d curr_point = scan_plane_->points_[id];
    Eigen::Vector3d pt_w = T * curr_point;

    std::vector<int> knn_index;
    std::vector<double> knn_dist;
    if (planar_kd_tree->SearchHybrid(pt_w, planar_dist_thres, 1, knn_index, knn_dist) > 0){
      if (knn_index.size() > 0)
        corres.emplace_back(std::make_pair(id, knn_index[0]));
    }
  }

  int planar_num = 0;
  for (auto& pair : corres){
    const auto& pi = pair.first;
    const auto& pj = pair.second;
    if (planar_num >= planar_maxnum){
      return true;
    }

    Eigen::Vector3d source_point = scan_plane_->points_[pi];
    Eigen::Matrix3d source_cov = scan_plane_covs_[pi];
    Eigen::Vector3d target_point = submap_plane_->points_[pj];
    Eigen::Matrix3d target_cov = submap_planar_covs[pj];

    ceres::CostFunction* cost_function = new PlaneToPlaneErr(source_point, source_cov, target_point, target_cov, plane_weights_(pi), &plane_residuals_(pi));
    {
      unique_lock.lock();
      problem.AddResidualBlock(cost_function, loss_function, parameters);
      unique_lock.unlock();
    }
    planar_num += 1;
  }

  if (planar_num <= 20){
    ROS_WARN("not enough planar point..");
  }

  return true;
}

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
bool LocalRegistration::addGroundCostFactor(const std::shared_ptr<open3d::geometry::PointCloud2> &scan_ground_,
                                            const std::shared_ptr<open3d::geometry::PointCloud2> &submap_ground_,
                                            Eigen::Matrix<double, 1, Eigen::Dynamic> &ground_weights_,
                                            Eigen::Matrix<double, 1, Eigen::Dynamic> &ground_residuals_,
                                            ceres::Problem &problem, ceres::LossFunction *loss_function) {
  std::unique_lock<std::mutex> unique_lock(factor_mutex, std::defer_lock);
  unique_lock.lock();
  Sophus::SE3d T = Sophus::SE3d::exp(se3_pose_);
  unique_lock.unlock();

  int ground_num = 0;
  size_t totalSize = scan_ground_->points_.size();
  for (size_t i = 0; i < totalSize; ++i){
    const auto& pt_w = T * scan_ground_->points_[i];

    std::vector<int> knn_index;
    std::vector<double> knn_dist;
    if (ground_kd_tree->SearchHybrid(pt_w, ground_dist_thres, 5, knn_index, knn_dist) > 0){
      if (knn_index.size() <= 4)
        continue;

      if (ground_num >= ground_maxnum)
        return true;

      std::vector<Eigen::Vector3d> near_point;
      for (auto& id : knn_index){
        near_point.emplace_back(submap_ground_->points_[id]);
      }

      Eigen::Vector4d plane_model = fitBestPlane(near_point);
      Eigen::Vector3d unit_norm = plane_model.block<3, 1>(0, 0);
      double dot_norm = plane_model(3);

      bool plane_valid = true;
      for (auto& pt : near_point){
        double plane_dis = unit_norm.transpose() * pt + dot_norm;

        // check the plane model enough precision
        if (plane_dis > 0.2){
          plane_valid = false;
          break;
        }
      }

      if (plane_valid){
        ceres::CostFunction* cost_function = new PointToPlaneErr(scan_ground_->points_[i], unit_norm, dot_norm, ground_weights_(i), &ground_residuals_(i));
        {
          unique_lock.lock();
          problem.AddResidualBlock(cost_function, loss_function, parameters);
          unique_lock.unlock();
        }

        ground_num++;
      }
    }
  }


  //std::cout << "ground_num tatal add................: " << ground_num << std::endl;
  if (ground_num <= 20){
    ROS_WARN("not enough ground point..");
  }

  return true;
}

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
bool LocalRegistration::addGroundCostFactor2(const std::shared_ptr<open3d::geometry::PointCloud2> &scan_ground_,
                                             const std::shared_ptr<open3d::geometry::PointCloud2> &submap_ground_,
                                             std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &scan_ground_covs_,
                                             std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &submap_ground_covs_,
                                             Eigen::Matrix<double, 1, Eigen::Dynamic> &ground_weights_,
                                             Eigen::Matrix<double, 1, Eigen::Dynamic> &ground_residuals_,
                                             ceres::Problem &problem, ceres::LossFunction *loss_function) {
    std::unique_lock<std::mutex> unique_lock(factor_mutex, std::defer_lock);
    unique_lock.lock();
    Sophus::SE3d T = Sophus::SE3d::exp(se3_pose_);
    unique_lock.unlock();

    int ground_num = 0;
    std::vector<std::pair<int, int>> corres;
    size_t totalSize = scan_ground_->points_.size();
    for (size_t id = 0; id < totalSize; ++id){
      Eigen::Vector3d curr_point = scan_ground_->points_[id];
      Eigen::Vector3d pt_w = T * curr_point;

      std::vector<int> knn_index;
      std::vector<double> knn_dist;
      if (ground_kd_tree->SearchHybrid(pt_w, planar_dist_thres, 1, knn_index, knn_dist) > 0){
        if (knn_index.size() > 0)
          corres.emplace_back(std::make_pair(id, knn_index[0]));
      }
    }

    for (auto& pair : corres){
      const auto& pi = pair.first;
      const auto& pj = pair.second;
      if (ground_num >= ground_maxnum){
        return true;
      }

      Eigen::Vector3d source_point = scan_ground_->points_[pi];
      Eigen::Matrix3d source_cov = scan_ground_covs_[pi];
      Eigen::Vector3d target_point = submap_ground_->points_[pj];
      Eigen::Matrix3d target_cov = submap_ground_covs_[pj];

      ceres::CostFunction* cost_function = new PlaneToPlaneErr(source_point, source_cov, target_point, target_cov, ground_weights_(pi), &ground_residuals_(pi));
      {
        unique_lock.lock();
        problem.AddResidualBlock(cost_function, loss_function, parameters);
        unique_lock.unlock();
      }
      ground_num += 1;
    }

    if (ground_num <= 20){
      ROS_WARN("not enough planar point..");
    }

    return true;
}

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
bool LocalRegistration::updateWeight(Eigen::Matrix<double, 1, Eigen::Dynamic> &weights_,
                                     Eigen::Matrix<double, 1, Eigen::Dynamic> &residuals_, double totalSize_,
                                     double noise_bound_sq_, double th1_, double th2_, double mu_) {
  for (size_t i = 0; i < totalSize_; ++i){
    if (residuals_(i) == 0)
      continue;

    if (residuals_(i) >= th1_)
      weights_(i) = 0.0;
    else if (residuals_(i) <= th2_)
      weights_(i) = 1.0;
    else{
      weights_(i) = std::sqrt(noise_bound_sq_ * mu_ * (mu_ + 1) / residuals_(i)) - mu_;
      assert(weights_(i) >= 0.0 && weights_(i) <= 1.0);
    }
  }

  return true;
}


bool LocalRegistration::scanMatching(Frame &out_result_, Eigen::Isometry3d& predict_pose_, Eigen::Isometry3d& result_pose_) {

  se3_pose_ = Sophus::SE3d(predict_pose_.matrix()).log();
  last_frame_pose = curr_frame_pose;

  if (se3_pose_.tail<3>().norm() < 1e-2){
    se3_pose_.tail<3>() = (Eigen::Vector3d::Random()).normalized() * 1e-4;
  }

  // reset kd_tree
  resetKDTree();

  // bulit kd_tree
  omp_set_num_threads(4);
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      #pragma omp task
      edge_kd_tree->SetGeometry(*submap_edge);
    }
    #pragma omp section
    {
      #pragma omp task
      sphere_kd_tree->SetGeometry(*submap_sphere);
    }
    #pragma omp section
    {
      #pragma omp task
      planar_kd_tree->SetGeometry(*submap_planar);
    }
    #pragma omp section
    {
      #pragma omp task
      ground_kd_tree->SetGeometry(*submap_ground);
    }
  }

  size_t scan_edge_size = scan_edge->points_.size();
  size_t scan_sphere_size = scan_sphere->points_.size();
  size_t scan_planar_size = scan_planar->points_.size();
  size_t scan_ground_size = scan_ground->points_.size();
  size_t submap_edge_size = submap_edge->points_.size();
  size_t submap_sphere_size = submap_sphere->points_.size();
  size_t submap_planar_size = submap_planar->points_.size();
  size_t submap_ground_size = submap_ground->points_.size();


  // check that all relevant point clouds are not empty.
  assert(scan_edge_size >= 10 && scan_sphere_size >= 10 && scan_planar_size >= 10 && scan_ground_size >= 10);
  assert(submap_edge_size >= 10 && submap_planar_size >= 10 && submap_sphere_size >= 10 && submap_ground_size >= 10);

  Eigen::Matrix<double, 1, Eigen::Dynamic> edge_weights(1, scan_edge_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> edge_residuals(1, scan_edge_size);
  edge_weights.setOnes();
  edge_residuals.setZero();

  Eigen::Matrix<double, 1, Eigen::Dynamic> sphere_weights(1, scan_sphere_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> sphere_residuals(1, scan_sphere_size);
  sphere_weights.setOnes();
  sphere_residuals.setZero();

  Eigen::Matrix<double, 1, Eigen::Dynamic> planar_weights(1, scan_planar_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> planar_residuals(1, scan_planar_size);
  planar_weights.setOnes();
  planar_residuals.setZero();

  Eigen::Matrix<double, 1, Eigen::Dynamic> ground_weights(1, scan_ground_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> ground_residuals(1, scan_ground_size);
  ground_weights.setOnes();
  ground_residuals.setZero();


  double edge_prev_cost = std::numeric_limits<double>::infinity();
  double edge_cost = std::numeric_limits<double>::infinity();
  double sphere_prev_cost = std::numeric_limits<double>::infinity();
  double sphere_cost  = std::numeric_limits<double>::infinity();
  double planar_prev_cost = std::numeric_limits<double>::infinity();
  double planar_cost = std::numeric_limits<double>::infinity();
  double ground_prev_cost = std::numeric_limits<double>::infinity();
  double ground_cost = std::numeric_limits<double>::infinity();

  double mu = 1.0;
  double noise_bound_sq = std::pow(noise_bound, 2);
  if (noise_bound_sq < 1e-16)
    noise_bound_sq = 1e-2;

  for (int iter = 0; iter < max_iterations; ++iter){
    nr_iterations = iter;

    //ceres::LossFunction* loss_function = new ceres::LossFunction();
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    problem.AddParameterBlock(parameters, 6, new PoseSE3Parameterization());

    std::vector<std::future<bool>> factor_build_thread(factor_num);
    std::vector<bool> results(factor_num);

    switch (factor_num) {
      case Factor::planarEdgeSphere:
        factor_build_thread[0] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addSurfCostFactor, this,
                                            std::ref(scan_planar), std::ref(submap_planar), std::ref(planar_weights), std::ref(planar_residuals),
                                            std::ref(problem), std::ref(loss_function));
        factor_build_thread[1] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addGroundCostFactor, this,
                                            std::ref(scan_ground), std::ref(submap_ground), std::ref(ground_weights), std::ref(ground_residuals),
                                            std::ref(problem), std::ref(loss_function));
        factor_build_thread[2] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addEdgeCostFactor, this,
                                            std::ref(scan_edge), std::ref(submap_edge), std::ref(edge_weights), std::ref(edge_residuals),
                                            std::ref(problem), std::ref(loss_function));
        factor_build_thread[3] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addSphereCostFactor, this,
                                            std::ref(scan_sphere), std::ref(submap_sphere), std::ref(sphere_weights), std::ref(sphere_residuals),
                                            std::ref(problem), std::ref(loss_function));
        break;
      case Factor::planarEdge:
        factor_build_thread[0] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addSurfCostFactor, this,
                                            std::ref(scan_planar), std::ref(submap_planar), std::ref(planar_weights), std::ref(planar_residuals),
                                            std::ref(problem), std::ref(loss_function));
        factor_build_thread[1] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addGroundCostFactor, this,
                                            std::ref(scan_ground), std::ref(submap_ground), std::ref(ground_weights), std::ref(ground_residuals),
                                            std::ref(problem), std::ref(loss_function));
        factor_build_thread[2] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addEdgeCostFactor, this,
                                            std::ref(scan_edge), std::ref(submap_edge), std::ref(edge_weights), std::ref(edge_residuals),
                                            std::ref(problem), std::ref(loss_function));
        break;
      case Factor::planar:
        factor_build_thread[0] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addSurfCostFactor, this,
                                            std::ref(scan_planar), std::ref(submap_planar), std::ref(planar_weights), std::ref(planar_residuals),
                                            std::ref(problem), std::ref(loss_function));
        factor_build_thread[1] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::addGroundCostFactor, this,
                                            std::ref(scan_ground), std::ref(submap_ground), std::ref(ground_weights), std::ref(ground_residuals),
                                            std::ref(problem), std::ref(loss_function));
        break;
      default:
        ROS_ERROR("Please check the type of feature points");
        break;
    }

    for (int fac_th = 0; fac_th < factor_num; ++fac_th){
      results[fac_th] = factor_build_thread[fac_th].get();
    }

    // addSurfCostFactor(scan_planar, submap_planar, planar_weights, planar_residuals, problem, loss_function);
    // addGroundCostFactor(scan_ground, submap_ground, ground_weights, ground_residuals, problem, loss_function);
    // addEdgeCostFactor(scan_edge, submap_edge, edge_weights, edge_residuals, problem, loss_function);
    // addSphereCostFactor(scan_sphere, submap_sphere, sphere_weights, sphere_residuals, problem, loss_function);

    if (iter == 0){
      double max_p_r = planar_residuals.maxCoeff(), max_e_r = edge_residuals.maxCoeff(), max_s_r = sphere_residuals.maxCoeff();
      double max_residual = max_p_r > max_e_r ? (max_p_r > max_s_r ? max_p_r : max_s_r) : (max_e_r > max_s_r ? max_e_r : max_s_r);
      mu = 1 / (2 * max_residual / noise_bound_sq - 1.0);
      if (mu <= 0)
        mu = 1e-10;
    }

    // perform solver;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = false;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    options.max_num_iterations = 4;
    options.num_threads = num_threads_/2;
    //options.max_solver_time_in_seconds = 0.10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double th1 = (mu + 1) / mu * noise_bound_sq;
    double th2 = mu / (mu + 1) * noise_bound_sq;

    // update corresponding weight
    std::vector<std::future<bool>> weight_update_thread(factor_num);
    switch (factor_num) {
      case Factor::planarEdgeSphere:
        weight_update_thread[0] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(planar_weights), std::ref(planar_residuals), scan_planar_size, noise_bound_sq, th1, th2, mu);
        weight_update_thread[1] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(ground_weights), std::ref(ground_residuals), scan_ground_size, noise_bound_sq, th1, th2, mu);
        weight_update_thread[2] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(edge_weights), std::ref(edge_residuals), scan_edge_size, noise_bound_sq, th1, th2, mu);
        weight_update_thread[3] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(sphere_weights), std::ref(sphere_residuals), scan_sphere_size, noise_bound_sq, th1, th2, mu);
        break;
      case Factor::planarEdge:
        weight_update_thread[0] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(planar_weights), std::ref(planar_residuals), scan_planar_size, noise_bound_sq, th1, th2, mu);
        weight_update_thread[1] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(ground_weights), std::ref(ground_residuals), scan_ground_size, noise_bound_sq, th1, th2, mu);
        weight_update_thread[2] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(edge_weights), std::ref(edge_residuals), scan_edge_size, noise_bound_sq, th1, th2, mu);
        break;
      case Factor::planar:
        weight_update_thread[0] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(planar_weights), std::ref(planar_residuals), scan_planar_size, noise_bound_sq, th1, th2, mu);
        weight_update_thread[1] = std::async(std::launch::async | std::launch::deferred, &LocalRegistration::updateWeight, this,
                                             std::ref(ground_weights), std::ref(ground_residuals), scan_ground_size, noise_bound_sq, th1, th2, mu);
        break;
      default:
        ROS_ERROR("Please check the type of feature points");
        break;
    }

    for (int fac_th = 0; fac_th < factor_num; ++fac_th){
      results[fac_th] = weight_update_thread[fac_th].get();
    }

    // increase mu
    mu = mu * std::exp(double(iter + 1) * gnc_factor);

    edge_cost = edge_residuals.sum();
    sphere_cost = sphere_residuals.sum();
    ground_cost = ground_residuals.sum();
    planar_cost = planar_residuals.sum();

    double planar_cost_diff = std::fabs(planar_cost - planar_prev_cost);
    double ground_cost_diff = std::fabs(ground_cost - ground_prev_cost);
    double edge_cost_diff = std::fabs(edge_cost - edge_prev_cost);
    double sphere_cost_diff = std::fabs(sphere_cost - sphere_prev_cost);

//    if (planar_cost_diff < cost_threshold || ground_cost_diff < cost_threshold ||
//        edge_prev_cost < cost_threshold || sphere_prev_cost < cost_threshold){
//      //ROS_INFO("The optimization has converged ahead of time");
//      break;
//    }
      // Please note that the convergence rates of various features are slightly different. In urban scences,
      // the optimization is finished after the planar features converge.
      if (planar_cost_diff < cost_threshold){
          //ROS_INFO("The optimization has converged ahead of time");
          break;
      }

    planar_prev_cost = planar_cost;
    ground_prev_cost = ground_cost;
    edge_prev_cost = edge_cost;
    sphere_prev_cost = sphere_cost;

    planar_residuals.setZero();
    ground_residuals.setZero();
    edge_residuals.setZero();
    sphere_residuals.setZero();
  }

  curr_frame_pose.matrix() = result_pose_.matrix() = Sophus::SE3d::exp(se3_pose_).matrix();

  if (!out_result_.scan_cloud->IsEmpty()){
    out_result_.scan_cloud->Transform(curr_frame_pose.matrix());
  }


  return true;

}

GlobalRegistration::GlobalRegistration() {
  int i = 0;
}

GlobalRegistration::~GlobalRegistration() noexcept {
  std::cout << "1" << std::endl;
}

bool GlobalRegistration::setInputSource(Frame &cloud_in_) {
  cloud_in_.reset();

  return true;
}

bool GlobalRegistration::setInputTarget(Frame &cloud_in_) {
  cloud_in_.reset();
  return true;
}

bool GlobalRegistration::scanMatching(Frame &out_result_, Eigen::Isometry3d &predict_pose_,
                                      Eigen::Isometry3d &result_pose_) {
  return true;
}

std::pair<double, double> GlobalRegistration::getFitnessScore() {
  return std::make_pair(0.0, 0.0);
}


}
