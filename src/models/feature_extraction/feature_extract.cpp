/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/3/12 下午11:20
 * @FileName: feature_extract.cpp
 * @Description: feature extract
 * @License: See LICENSE for the license information
 */

#include "tloam/models/feature_extraction/feature_extract.hpp"

namespace tloam{
featureExtract::featureExtract() {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/feature.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);
  initConfig(config_node["feature"]);
}

void featureExtract::initConfig(const YAML::Node &node) {
  radius = node["radius"].as<double>();
  K = node["K"].as<int>();
  planar_num = node["planar_num"].as<int>();
  sphere_num = node["sphere_num"].as<int>();
  min_neigh = node["min_neigh"].as<int>();
  cvr_scan = node["cvr_scan"].as<double>();
  cvr_submap = node["cvr_submap"].as<double>();
  planar_scan_thres = node["planar_scan_thres"].as<double>();
  planar_submap_thres = node["planar_submap_thres"].as<double>();
  planar_vertic_thres = node["planar_vertic_thres"].as<double>();

}

/**
 * @brief PCA information calculation by SVD
 * @param cloud_in_, input cloud
 * @param pca_info_, output info
 * @param r_, KNN radius
 * @param K_, domain number
 * @return true if success otherwise false
 */
bool featureExtract::calculatePCAInfo(CloudData &cloud_in_, std::vector<PCAInfo> &pca_info_, double r_, int K_) {
  // build kd_ree
  if (cloud_in_.cloud_ptr->IsEmpty()){
    ROS_ERROR("cloud_in_ does not contain points..");
    return false;
  }

  assert(r_ >= 0.0 && K_ >= 3);
  std::shared_ptr<open3d::geometry::KDTreeFlann> kd_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  kd_tree->SetGeometry(*cloud_in_.cloud_ptr);
  size_t totalSize = cloud_in_.cloud_ptr->points_.size();
  pca_info_.resize(totalSize);

#ifdef _OPENMP
#pragma omp parallel for num_threads(num_threads_) shared(kd_tree, cloud_in_, pca_info_)
#endif
  for (size_t i = 0; i < totalSize; ++i){
    std::vector<int> neigh_index{};
    std::vector<double> neigh_dists{};
    std::vector<int>().swap(neigh_index);
    std::vector<double>().swap(neigh_dists);

    const auto& cur_pt = cloud_in_.cloud_ptr->points_[i];
    if (kd_tree->SearchHybrid(cur_pt, r_, K_, neigh_index, neigh_dists) > 0){
      if (neigh_index.size() <= min_neigh)
        continue;
      else{
        pca_info_[i].cur_index = i;
        pca_info_[i].num_sum = static_cast<int>(neigh_index.size());
        Eigen::Matrix3d cov_temp;
        Eigen::Matrix<double, 9, 1> cumulants = Eigen::Matrix<double, 9, 1>::Zero();
        for (auto& item : neigh_index){
          Eigen::Vector3d& point = cloud_in_.cloud_ptr->points_[item];
          cumulants(0) += point(0);
          cumulants(1) += point(1);
          cumulants(2) += point(2);
          cumulants(3) += point(0) * point(0);
          cumulants(4) += point(0) * point(1);
          cumulants(5) += point(0) * point(2);
          cumulants(6) += point(1) * point(1);
          cumulants(7) += point(1) * point(2);
          cumulants(8) += point(2) * point(2);
        }
        cumulants /= (double)neigh_index.size();
        cov_temp(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
        cov_temp(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
        cov_temp(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
        cov_temp(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
        cov_temp(1, 0) = cov_temp(0, 1);
        cov_temp(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
        cov_temp(2, 0) = cov_temp(0, 2);
        cov_temp(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
        cov_temp(2, 1) = cov_temp(1, 2);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
        // TODO Ascending order of eigenvalues
        solver.compute(cov_temp, Eigen::ComputeEigenvectors);
        Eigen::Vector3d eigen_value = solver.eigenvalues();      // 这里特征值是从小到大进行排序的
        pca_info_[i].normal_dir = solver.eigenvectors().col(0);    // 特征向量按照列方向一次排列最后一个方向是主方向, 第一个方向是法向量的方向

        double sum_value = eigen_value.sum();
        if (sum_value == 0.0)
          pca_info_[i].cvr = 0.0;
        else{
          pca_info_[i].cvr = eigen_value[0] / sum_value;
        }

        pca_info_[i].flatness = (eigen_value[1] - eigen_value[0]) / eigen_value[2];
        pca_info_[i].sphericity = eigen_value[0] / eigen_value[2];
        neigh_index.swap(pca_info_[i].neigh_index);
      }
    }
  }

  return true;
}

/**
 * @brief extract planar features and sphere features
 * @param cloud_in_, input cloud
 * @param planar_scan_index, current scan planar features
 * @param planar_submap_index, submap planar features
 * @param sphere_scan_index, current scan planar features
 * @param sphere_submap_index, submap planar features
 * @return true if success otherwise false
 */
bool featureExtract::extractPlanarSphere(CloudData &cloud_in_, std::vector<size_t> &planar_scan_index,
                                         std::vector<size_t> &planar_submap_index, std::vector<size_t> &sphere_scan_index,
                                         std::vector<size_t> &sphere_submap_index) {
  size_t totalSize = cloud_in_.cloud_ptr->points_.size();
  std::vector<PCAInfo> pca_info;

  Timer test_time{};
  if (calculatePCAInfo(cloud_in_, pca_info, this->radius, this->K)){
#ifdef _OPENMP
#pragma omp declare reduction (merge : std::vector<std::pair<double, int>> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#endif
    std::vector<std::pair<double, int>> planar_info;
    std::vector<std::pair<double, int>> sphere_info;
#ifdef _OPENMP
#pragma omp parallel for num_threads(num_threads_) reduction(merge: planar_info, sphere_info)
#endif
    for (size_t id = 0; id < totalSize; ++id){
      if (pca_info[id].flatness > planar_submap_thres && std::fabs(pca_info[id].normal_dir.z()) < planar_vertic_thres){
        planar_info.emplace_back(std::make_pair(pca_info[id].flatness, pca_info[id].cur_index));
      }else if (pca_info[id].cvr > cvr_submap){
        bool max_uniform{true};
        for (auto& item : pca_info[id].neigh_index){
          if (pca_info[id].cvr < pca_info[item].cvr){
            max_uniform = false;
            break;
          }
        }

        if (max_uniform)
          sphere_info.emplace_back(std::make_pair(pca_info[id].flatness, pca_info[id].cur_index));
      }
    }

    // descending order
    std::sort(planar_info.begin(), planar_info.end(), [&](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs){
      return lhs.first > rhs.first;
    });

    std::sort(sphere_info.begin(), sphere_info.end(), [&](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs){
      return lhs.first > rhs.first;
    });

    size_t planarSize = planar_info.size(), sphereSize = sphere_info.size();

    for (size_t id = 0; id < planarSize; ++id){
      if (id < planar_num || planar_info[id].first > planar_scan_thres)
        planar_scan_index.emplace_back(planar_info[id].second);

      planar_submap_index.emplace_back(planar_info[id].second);
    }

    for (size_t id = 0; id < sphereSize; ++id){
      if (id < sphere_num || sphere_info[id].first > cvr_scan)
        sphere_scan_index.emplace_back(id);

      sphere_submap_index.emplace_back(id);
    }

    std::vector<PCAInfo>().swap(pca_info);
    std::vector<std::pair<double, int>>().swap(planar_info);
    std::vector<std::pair<double, int>>().swap(sphere_info);
  }

  return true;
}



}
