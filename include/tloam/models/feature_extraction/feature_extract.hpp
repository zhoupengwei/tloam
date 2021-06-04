/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/3/12 下午11:19
 * @FileName: feature_extract.hpp.h
 * @Description: ${DESCRIPTION}
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_FEATURE_EXTRACT_HPP
#define TLOAM_FEATURE_EXTRACT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <future>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "tloam/models/utils/sensor_data.hpp"
#include "tloam/open3d/open3d_to_ros.hpp"
#include "tloam/models/utils/utils.hpp"
#include "tloam/models/utils/work_space_path.h"

namespace tloam{
// feature extract based on Principe Component Analysis
class featureExtract{
public:
  struct PCAInfo{
    Eigen::Vector3d normal_dir;
    double cvr, flatness, sphericity;
    int cur_index, num_sum;
    std::vector<int> neigh_index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
  explicit featureExtract();
  ~featureExtract() = default;

  void initConfig(const YAML::Node& node);

  /**
   * @brief PCA information calculation by SVD
   * @param cloud_in_, input cloud
   * @param pca_info_, output info
   * @param r_, KNN radius
   * @param K_, domain number
   * @return true if success otherwise false
   */
  bool calculatePCAInfo(
    CloudData& cloud_in_,
    std::vector<PCAInfo>& pca_info_,
    double r_,
    int K_
  );

  /**
   * @brief extract planar features and sphere features
   * @param cloud_in_, input cloud
   * @param planar_scan_index, current scan planar features
   * @param planar_submap_index, submap planar features
   * @param sphere_scan_index, current scan planar features
   * @param sphere_submap_index, submap planar features
   * @return true if success otherwise false
   */
  bool extractPlanarSphere(
    CloudData& cloud_in_,
    std::vector<size_t>& planar_scan_index,
    std::vector<size_t>& planar_submap_index,
    std::vector<size_t>& sphere_scan_index,
    std::vector<size_t>& sphere_submap_index
  );


private:

  int num_threads_{};
  double radius;
  int K;

  double planar_submap_thres;
  double planar_vertic_thres;
  double planar_scan_thres;
  double cvr_submap;
  double cvr_scan;

  int planar_num;
  int sphere_num;
  int min_neigh;
};
}

#endif //TLOAM_FEATURE_EXTRACT_HPP
