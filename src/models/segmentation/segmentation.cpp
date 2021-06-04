/**
 * @Copyright 2021, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/14 上午10:47
 * @FileName: segmentation.cpp
 * @Description: Fast and precise point cloud segmentation
 * @License: See LICENSE for the license information
 */

#include "tloam/models/segmentation/segmentation.hpp"

namespace tloam{
Segmentation::Segmentation(ros::NodeHandle &nh, std::string &topic_name) {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads() / 2;
#else
  num_threads_ = 1;
#endif
  // subscribe to the raw point cloud
  cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(nh, topic_name, 10000);
  // listenr
  velo_link_to_map_ptr_ = std::make_unique<TFListener>(nh, "/map", "/velo_link");
  // publish ground, vertical, edge, general cloud,
  cloud_pub_ground_ptr_ = std::make_unique<CloudPublisher>(nh, "/ground_points", "/velo_link", 100);
  cloud_pub_object_ptr_ = std::make_unique<CloudPublisher>(nh, "/object_points", "/velo_link", 100);
  cloud_pub_edge_ptr_ = std::make_unique<CloudPublisher>(nh, "/edge_points", "/velo_link", 100);
  cloud_pub_general_ptr_ = std::make_unique<CloudPublisher>(nh, "/general_points", "/velo_link", 100);
  box_pub_ptr_ = std::make_unique<BoundingBoxPublisher>(nh, "/DCVC_Box", "/velo_link", 100);
}

Segmentation::~Segmentation() {
  for (int i = 0; i < quadrant; ++i){
    regionIndex[i]->clear();
    regionIndex[i] = nullptr;
  }

  delete[] regionIndex;
}

void Segmentation::spinOnce() {
  if (!readData())
    return;

  while (hasData()){
    if (!validData())
      continue;

    RemoveClosedNonFinitePoints(current_scan, near_dis, true, true);

    if (groundRemove()){
      ROS_DEBUG("segmentation node finish ground remove...");
    }else{
      ROS_ERROR("ground remove failed!!!");
      return;
    }

    if (objectSegmentation()){
      ROS_DEBUG("segmentation node finish object segmentation...");
    }else{
      ROS_ERROR("object segmentation failed!!!");
      return;
    }

    if (extractEdgePoint(segmented_scan, edge_scan, general_scan)){
      ROS_DEBUG("segmentation node finish edge point extraction...");
    }else{
      ROS_ERROR("edge point finished extract!!!");
      return;
    }

    if (cloud_pub_ground_ptr_->HasSubscribers()){
      cloud_pub_ground_ptr_->Publisher(ground_scan.cloud_ptr, current_time.toSec());
    }

    if (cloud_pub_object_ptr_->HasSubscribers()){
      cloud_pub_object_ptr_->Publisher(segmented_scan.cloud_ptr, current_time.toSec());
    }

    if (cloud_pub_edge_ptr_->HasSubscribers()){
      cloud_pub_edge_ptr_->Publisher(edge_scan.cloud_ptr, current_time.toSec());
    }

    if (cloud_pub_general_ptr_->HasSubscribers()){
      cloud_pub_general_ptr_->Publisher(general_scan.cloud_ptr, current_time.toSec());
    }

    if (box_pub_ptr_->HasSubscribers()){
      box_pub_ptr_->Publisher(boxInfo, current_time.toSec());
    }

    resetParams();
  }
}

bool Segmentation::initWithConfig() {
  std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/segmentation.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  initVeldyneConfig(config_node["velodyne"]);
  initGroundSegConfig(config_node["groundSeg"]);
  initDCVCConfig(config_node["DCVC"]);

  radiusTable = Eigen::VectorXd::Constant(sensorModel, 0.0);
  radiusTableDiff = Eigen::VectorXd::Constant(sensorModel, 0.0);

  regionIndex = new std::vector<size_t>*[quadrant];
  for (int i = 0; i < quadrant; ++i){
    regionIndex[i] = new std::vector<size_t>[numSec];
  }

  return true;
}

void Segmentation::initVeldyneConfig(const YAML::Node &node) {
  sensorModel = node["sensorModel"].as<int>();
  scanPeriod = node["scanPeriod"].as<double>();
  verticalRes = node["verticalRes"].as<double>();
  initAngle = node["initAngle"].as<double>();
  sensorHeight = node["sensorHeight"].as<double>();
  sensorMinRange = node["sensorMinRange"].as<double>();
  sensorMaxRange = node["sensorMaxRange"].as<double>();
  near_dis = node["near_dis"].as<double>();
}

void Segmentation::initGroundSegConfig(const YAML::Node &node) {
  planeDis = node["dis"].as<double>();
  numSec = node["numSec"].as<int>();
  quadrant = node["quadrant"].as<int>();
  groundSeedNum = node["ground_seed_num"].as<int>();
  maxIter = node["maxIter"].as<int>();
  ringMinNum = node["ringMinNum"].as<int>();

}

void Segmentation::initDCVCConfig(const YAML::Node &node) {
  startR = node["startR"].as<double>();
  deltaR = node["deltaR"].as<double>();
  deltaP = node["deltaP"].as<double>();
  deltaA = node["deltaA"].as<double>();
  minSeg = node["minSeg"].as<int>();
}

bool Segmentation::readData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  velo_link_to_map_ptr_->LookupData(velo_link_to_map);

  return true;
}

bool Segmentation::hasData() {
  return !cloud_data_buff_.empty();
}

bool Segmentation::validData() {
  bufferMutex.lock();
  if (cloud_data_buff_.empty()){
    bufferMutex.unlock();
    return false;
  }else{
    current_scan = cloud_data_buff_.front();
    current_time = ros::Time::now();
    cloud_data_buff_.pop_front();
    bufferMutex.unlock();

    return true;
  }
}

/**
 * @brief initialize ring radius table and section bounds
 * @param void
 * @return true if success otherwise false
 */
bool Segmentation::initSections() {
  int boundaryIndices[numSec];
  int sectionWidth = static_cast<int>(std::ceil(1.0 * sensorModel) / numSec);

  for (int i = 0; i < numSec; ++i){
    boundaryIndices[i] = sectionWidth * (i + 1) - 1;
  }

  for (int i = 0; i < quadrant; ++i){
    for (int j = 0; j < numSec; ++j){
      regionIndex[i][j].clear();
    }
  }

  double prevRadius = 0.0;
  double initAngleTemp = initAngle;
  sectionBounds.clear();
  int secBoundIndex{0};
  for (int i = 0; i < sensorModel; ++i){
    if ((sensorModel == 64) && (i == 31)){
      initAngleTemp += 1.7;
    }

    auto curRadius = static_cast<double>(sensorHeight / std::tan(std::fabs(initAngleTemp) / 180.0 * M_PI));
    curRadius = curRadius < sensorMaxRange ? curRadius : sensorMaxRange;
    radiusTable[i] = curRadius;
    if (i >= 1){
      auto dis = std::fabs(curRadius - prevRadius);
      if (dis >= 5.0 || dis <= 0.0){
        radiusTableDiff[i-1] = 0.0;
        continue;
      }
      radiusTableDiff[i-1] = dis;
    }
    if (i == boundaryIndices[secBoundIndex] && secBoundIndex <= 3){
      double theta = std::fabs(initAngleTemp / 180 * M_PI);
      if (theta != 0 && i < sensorModel){
        sectionBounds.insert(sectionBounds.end(), static_cast<float>(sensorHeight / tan(theta)));
      }
      else{
        sectionBounds.insert(sectionBounds.end(), sensorMaxRange);
      }
      secBoundIndex++;
    }
    prevRadius = curRadius;
    initAngleTemp += verticalRes;
  }

  return true;
}

/**
 * @brief determine the section number according to the polar diameter.
 * @param radius, polar diameter data of the point cloud
 * @return section number
 */
int Segmentation::getSection(const double &radius) {
  for (int i = 0; i < numSec; ++i){
    if (radius < sectionBounds[i])
      return i;
  }
  return numSec - 1;
}

/**
 * @brief
 * @param cloud_in_, the point cloud to be estimated
 * @param cloud_out_, output the beams and collected time information of each point and stored in the intensity channel.
 * @return true if success otherwise false
 */
bool Segmentation::estimateRingsAndTimes(CloudData &cloud_in_, CloudData &cloud_out_) const {
  size_t totalSize = cloud_in_.cloud_ptr->points_.size();
  std::shared_ptr<open3d::geometry::PointCloud2> raw_cloud{new open3d::geometry::PointCloud2(*cloud_in_.cloud_ptr)};
  cloud_in_.reset();

  auto get_quadrant = [](const Eigen::Vector3d& point)->int{
    int quadrantId;
    if (point.x() > 0 && point.y() >= 0){
      quadrantId = 1;
    }
    else if (point.x() <= 0 && point.y() > 0){
      quadrantId = 2;
    }
    else if (point.x() < 0 && point.y() <= 0){
      quadrantId = 3;
    }
    else{
      quadrantId = 4;
    }
    return quadrantId;
  };

  std::vector<std::vector<int>> ringScan(sensorModel);
  double mean_height{0.0};

  // More elegant way to deal with KITTI dataset point cloud
  raw_cloud->intensity_.reserve(totalSize);
  if (sensorModel == 64){
    Timer test_time;
    int prev_quadrant = 0, curr_quadrant = 0, ring = 0, totalRing = 0;
    Eigen::Vector3d cur_point;
    Timer intensity_time;
    for (size_t item = 0; item < totalSize; item++){
      cur_point = raw_cloud->points_[item];
      mean_height += cur_point.z();
      curr_quadrant = get_quadrant(cur_point);
      if ((curr_quadrant == 1) && (prev_quadrant == 4) && (ring < static_cast<int>(sensorModel - 1))){
        ring++;
        totalRing = 0;
      }
      totalRing += 1;

      //ringScan[ring].emplace_back(item);
      raw_cloud->intensity_.emplace_back(ring);

      prev_quadrant = curr_quadrant;
    }
    mean_height = mean_height / totalSize;
  }else{
    Eigen::Vector3d pt = Eigen::Vector3d::Zero();
    for (size_t item = 0; item < totalSize; item++){
      // calculate pitch angle
      pt =raw_cloud->points_[item];
      mean_height += pt.z();
      double pitch_angle = std::atan2(pt.z(), std::sqrt(pt.x() * pt.x() + pt.y() * pt.y())) * 180.0 / M_PI;
      auto ring = static_cast<size_t>((pitch_angle + std::fabs(initAngle) + 0.1) / verticalRes);
      ringScan[ring].emplace_back(item);
    }
    mean_height = mean_height / totalSize;
  }

  double ori, startOri, endOri, totalOri, realTime, ring_time;

  cloud_out_.cloud_ptr->points_.reserve(totalSize);
  cloud_out_.cloud_ptr->intensity_.reserve(totalSize);

  non_ground_scan.cloud_ptr->points_.reserve(totalSize);
  non_ground_scan.cloud_ptr->intensity_.reserve(totalSize);

  Eigen::Vector3d startPoint, endPoint, curPoint;
  int beam = 0;
  for (size_t i = 0; i < totalSize; ++i){
    curPoint = raw_cloud->points_[i];
    int ringInfo = static_cast<int>(raw_cloud->intensity_[i]);
    if (curPoint.z() > mean_height){
      non_ground_scan.cloud_ptr->points_.emplace_back(curPoint);
      non_ground_scan.cloud_ptr->intensity_.emplace_back(ring_time);
    }else{
      cloud_out_.cloud_ptr->points_.emplace_back(curPoint);
      cloud_out_.cloud_ptr->intensity_.emplace_back(ring_time);
    }
  }
  cloud_out_.cloud_ptr->points_.shrink_to_fit();
  cloud_out_.cloud_ptr->intensity_.shrink_to_fit();
  non_ground_scan.cloud_ptr->points_.shrink_to_fit();
  non_ground_scan.cloud_ptr->intensity_.shrink_to_fit();

  return true;
}

double Segmentation::estimateRingsAndTimes2(CloudData &cloud_in_) const {
  if (cloud_in_.cloud_ptr->IsEmpty()){
    ROS_ERROR("input cloud do not have enough point..");
    return 1.0;
  }

  // TODO More elegant way to deal with KITTI
  auto HDL_64E = [&](CloudData& cloud_in_)->double{
    size_t totalSize = cloud_in_.cloud_ptr->points_.size();
    cloud_in_.cloud_ptr->intensity_.clear();
    cloud_in_.cloud_ptr->intensity_.reserve(totalSize);

    auto get_quadrant = [](const Eigen::Vector3d& point)->int{
      int quadrant = 0;
      if (point.x() > 0 && point.y() >= 0){
        quadrant = 1;
      }
      else if (point.x() <= 0 && point.y() > 0){
        quadrant = 2;
      }
      else if (point.x() < 0 && point.y() <= 0){
        quadrant = 3;
      }
      else{
        quadrant = 4;
      }
      return quadrant;
    };

    int prev_q = 0, cur_q = 0, beam = 0, sumBeam = 0;
    double mean_height{0.0};

    for (size_t id = 0; id < totalSize; ++id){
      Eigen::Vector3d& pt = cloud_in_.cloud_ptr->points_[id];
      mean_height += pt.z();
      cur_q = get_quadrant(pt);
      if (cur_q == 1 && prev_q == 4 && beam < (sensorModel-1)){
        beam++;
        sumBeam = 0;
      }

      sumBeam+=1;
      cloud_in_.cloud_ptr->intensity_.emplace_back(static_cast<double>(beam));
      prev_q = cur_q;
    }

    return mean_height / double(totalSize);
  };

  auto VLP_16 = [&](CloudData& cloud_in_)->double{
    size_t totalSize = cloud_in_.cloud_ptr->points_.size();
    cloud_in_.cloud_ptr->intensity_.clear();
    cloud_in_.cloud_ptr->intensity_.reserve(totalSize);

    double startOri = std::atan2(cloud_in_.cloud_ptr->points_[0].y(), cloud_in_.cloud_ptr->points_[0].x());
    double endOri = std::atan2(cloud_in_.cloud_ptr->points_[totalSize-1].y(), cloud_in_.cloud_ptr->points_[totalSize-1].x());

    if (endOri - startOri > 3*M_PI)
      endOri -= 2*M_PI;
    else if (endOri - startOri < M_PI)
      endOri += 2*M_PI;

    double scanOri = endOri - startOri;

    double ang_bot = std::fabs(initAngle) + 0.1;
    double prevOri = 0.0, halfOri = 0.0, mean_height = 0.0;
    bool halfPass = false;
    for (size_t id = 0; id < totalSize; ++id){
      Eigen::Vector3d& pt = cloud_in_.cloud_ptr->points_[id];
      mean_height += pt.z();
      double pitch = std::atan2(pt.z(), std::sqrt(pt.x()*pt.x() + pt.y()*pt.y())) * 180.0 / M_PI;
      auto beamId = static_cast<int>(pitch + ang_bot) / verticalRes;

      double ori = std::atan2(pt.y(), pt.x());
      double correctTime = std::fabs(ori - startOri) / scanOri;

      if (prevOri < 0.0 && ori > 0.0){
        halfPass = true;
        halfOri = std::fabs(prevOri - startOri);
      }

      if (halfPass){
        ori = M_PI - ori;
        correctTime = (ori + halfOri) / scanOri;
        if (correctTime > 1.0)
          correctTime = 0.99999;
      }

      correctTime = static_cast<double>(beamId) + correctTime;
      cloud_in_.cloud_ptr->intensity_.emplace_back(correctTime);

      prevOri = ori;
    }

    return mean_height / double(totalSize);
  };

  double mean_height = 0.0;
  switch (sensorModel) {
    case LiDAR::HDL_64E:
      mean_height = HDL_64E(cloud_in_);
      break;
    case LiDAR::VLP_16:
      mean_height = VLP_16(cloud_in_);
      break;
    default:
      ROS_ERROR("unsupported LiDAR type");
      break;
  }

  return mean_height;
}

/**
  * @brief filter out part of the vertical point cloud according to the height value
  * @param cloud_in_
  * @param mean_height_
  * @param cloud_out_
  * @return true if success otherwise false
  */
bool Segmentation::filterByHeight(CloudData &cloud_in_, double &mean_height_, CloudData &cloud_out_) {
  std::shared_ptr<open3d::geometry::PointCloud2> raw_cloud(new open3d::geometry::PointCloud2(*cloud_in_.cloud_ptr));
  cloud_in_.reset();

  for (size_t i = 0, totalSize = raw_cloud->points_.size(); i < totalSize; ++i){
    Eigen::Vector3d pt = raw_cloud->points_[i];
    if (pt.z() > mean_height_){
      cloud_out_.cloud_ptr->points_.emplace_back(pt);
      cloud_out_.cloud_ptr->intensity_.emplace_back(raw_cloud->intensity_[i]);
      continue;
    }
    cloud_in_.cloud_ptr->points_.emplace_back(pt);
    cloud_in_.cloud_ptr->intensity_.emplace_back(raw_cloud->intensity_[i]);
  }

  return true;
}

void Segmentation::RemoveClosedNonFinitePoints(CloudData &cloud_in_, double dis_th, bool remove_nan,
                                               bool remove_infinite) {
    bool has_normal = cloud_in_.cloud_ptr->HasNormals();
    bool has_color = cloud_in_.cloud_ptr->HasColors();
    bool has_intensity = cloud_in_.cloud_ptr->HasIntensity();

    size_t old_point_num = cloud_in_.cloud_ptr->points_.size();
    size_t k = 0;

    for (size_t i = 0; i < old_point_num; i++) {  // old index
        Eigen::Vector3d pt = cloud_in_.cloud_ptr->points_[i];
        bool is_nan = remove_nan && (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]));
        bool is_infinite = remove_infinite && (std::isinf(pt[0]) || std::isinf(pt[1]) || std::isinf(pt[2]));
        if (!is_nan && !is_infinite && pt.norm() >= dis_th * dis_th ) {
            cloud_in_.cloud_ptr->points_[k] = pt;
            if (has_normal) cloud_in_.cloud_ptr->normals_[k] = cloud_in_.cloud_ptr->normals_[i];
            if (has_color) cloud_in_.cloud_ptr->colors_[k] = cloud_in_.cloud_ptr->colors_[i];
            if (has_intensity) cloud_in_.cloud_ptr->intensity_[k] = cloud_in_.cloud_ptr->intensity_[i];
            k++;
        }
    }

    cloud_in_.cloud_ptr->points_.resize(k);
    if (has_normal) cloud_in_.cloud_ptr->normals_.resize(k);
    if (has_color) cloud_in_.cloud_ptr->colors_.resize(k);
    if (has_intensity) cloud_in_.cloud_ptr->intensity_.resize(k);
    open3d::utility::LogDebug("[RemoveClosedNonFinitePoints] {:d} nan points have been removed.", (int)(old_point_num - k));
}


/**
 * @brief calculate the region index of each point
 * @param cloud_in_, input point cloud
 * @return true if success otherwise false
 */
bool Segmentation::fillSectionIndex(CloudData &cloud_in_) {

  size_t totalSize = cloud_in_.cloud_ptr->points_.size();
  for (int i = 0; i < quadrant; ++i){
    for (int j = 0; j < numSec; ++j){
      regionIndex[i][j].reserve(totalSize/quadrant);
    }
  }

  double x, y, r, theta; int s;
  for (size_t i = 0; i < totalSize; ++i){
    x = cloud_in_.cloud_ptr->points_[i].x();
    y = cloud_in_.cloud_ptr->points_[i].y();
    r = std::sqrt(x*x + y*y);

    theta = std::atan2(y, x) * 180.0 / M_PI;
    s = getSection(r);
    if (-145.0 <= theta && theta < -45.0){
      regionIndex[3][s].emplace_back(i);
    }else if (-45.0 <= theta && theta < 45.0){
      regionIndex[0][s].emplace_back(i);
    }else if (45.0 <= theta && theta <= 135.0){
      regionIndex[1][s].emplace_back(i);
    }else{
      regionIndex[2][s].emplace_back(i);
    }
  }
  return true;
}


/**
  * @brief Find best plane based on matrix-axis linear regression
  * @param cloud_in_, input point cloud data
  * @param out_plane_, output plane model
  * @return void
  */
void Segmentation::findBestPlane(CloudData &cloud_in_, Eigen::Vector4d &out_plane_) {
  if (cloud_in_.cloud_ptr->IsEmpty()){
    ROS_ERROR("At least three points are needed to estimate a plane model.");
    return;
  }

  Eigen::Vector3d centroid(0.0, 0.0, 0.0);
  for (const auto& item : cloud_in_.cloud_ptr->points_){
    centroid += item;
  }
  centroid /= static_cast<double>(cloud_in_.cloud_ptr->points_.size());
  double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (const auto& p : cloud_in_.cloud_ptr->points_){
    Eigen::Vector3d r = p - centroid;
    xx += r(0) * r(0);
    xy += r(0) * r(1);
    xz += r(0) * r(2);
    yy += r(1) * r(1);
    yz += r(1) * r(2);
    zz += r(2) * r(2);
  }
  auto totalSize = static_cast<double>(cloud_in_.cloud_ptr->points_.size());
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
    out_plane_ = Eigen::Vector4d{0, 0, 0, 0};
  }
  weighted_dir.normalize();
  double d = -weighted_dir.dot(centroid);

  out_plane_ = Eigen::Vector4d(weighted_dir(0), weighted_dir(1), weighted_dir(2), d);
}

/**
 * @brief thread function of ground extraction
 * @param q, quadrant index
 * @param cloud_in_, input point cloud
 * @param out_no_ground, output non-ground point cloud
 * @param out_ground, output ground point cloud
 * @return true if success otherwise false
 */
bool Segmentation::segmentGroundThread(int q, const CloudData &cloud_in_, CloudData &out_no_ground,
                                       CloudData &out_ground) {
  std::unique_lock<std::mutex> regionLock(regionMutex, std::defer_lock);
  for (int s = 0; s < numSec; s++){
    CloudData regionCloud{};
    // temporary data
    CloudData tempGround{}, groundSeed{}, tempVertical{};

    std::vector<size_t> regionLocalIndex = regionIndex[q][s];
    regionCloud.cloud_ptr = cloud_in_.cloud_ptr->SelectByIndex(regionLocalIndex);

    std::vector<size_t>().swap(regionLocalIndex);

    // store height and index information
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> point_info_;
    for (size_t k = 0, totalSize = regionCloud.cloud_ptr->points_.size(); k < totalSize; ++k){
      double r = regionCloud.cloud_ptr->points_[k].norm();
      double z = regionCloud.cloud_ptr->points_[k].z();
      if (z >= -1.5 * sensorHeight && r >= sensorMinRange && r <= sensorMaxRange){
        point_info_.emplace_back(Eigen::Vector2d(z, static_cast<double>(k)));
      }
    }

    std::sort(point_info_.begin(), point_info_.end(), [&](Eigen::Vector2d& lhs, Eigen::Vector2d& rhs)->bool {
      return lhs[0] < rhs[0];
    });

    double sum_z{0.0};
    int count{0};
    for (size_t k = 0, totalSize = point_info_.size(); k < totalSize && count < groundSeedNum; ++k, ++count){
      sum_z += point_info_[k][0];
    }
    double av_height = count != 0 ? sum_z / count : 0;
    for (auto & k : point_info_){
      if (k[0] < (av_height + planeDis)){
        groundSeed.cloud_ptr->points_.emplace_back(regionCloud.cloud_ptr->points_[int(k[1])]);
      }
    }

    if (groundSeed.cloud_ptr->points_.size() <= 3)
      continue;

    tempGround = std::move(groundSeed);
    for (int iter = 0; iter < maxIter; ++iter){
      Eigen::Vector4d planeModel = Eigen::Vector4d::Zero();
      findBestPlane(tempGround, planeModel);

      tempGround.reset();
      tempVertical.reset();
      for (auto& info : point_info_){
        auto id = static_cast<int>(info[1]);
        Eigen::Vector4d pt(regionCloud.cloud_ptr->points_[id].x(),
                           regionCloud.cloud_ptr->points_[id].y(),
                           regionCloud.cloud_ptr->points_[id].z(), 1.0);
        double dis = std::fabs(planeModel.dot(pt));
        if (dis < planeDis){
          tempGround.cloud_ptr->points_.emplace_back(pt.block(0, 0, 3, 1));
          if (iter == (maxIter - 1)){
            // delete beam information
            if (regionCloud.cloud_ptr->HasIntensity()){
              tempGround.cloud_ptr->intensity_.emplace_back(regionCloud.cloud_ptr->intensity_[id] -
                static_cast<int>(regionCloud.cloud_ptr->intensity_[id]));
            }
            if (regionCloud.cloud_ptr->HasNormals()){
              tempGround.cloud_ptr->normals_.emplace_back(regionCloud.cloud_ptr->normals_[id]);
            }
            if (regionCloud.cloud_ptr->HasColors()){
              tempGround.cloud_ptr->colors_.emplace_back(regionCloud.cloud_ptr->colors_[id]);
            }
          }
        }else{
          if (iter == (maxIter - 1)){
            tempVertical.cloud_ptr->points_.emplace_back(regionCloud.cloud_ptr->points_[id]);
            if (regionCloud.cloud_ptr->HasIntensity()){
              tempVertical.cloud_ptr->intensity_.emplace_back(regionCloud.cloud_ptr->intensity_[id]);
            }
            if (regionCloud.cloud_ptr->HasColors()){
              tempVertical.cloud_ptr->colors_.emplace_back(regionCloud.cloud_ptr->colors_[id]);
            }
            if (regionCloud.cloud_ptr->HasNormals()){
              tempVertical.cloud_ptr->normals_.emplace_back(regionCloud.cloud_ptr->normals_[id]);
            }
          }
        }
      }
    }

    // free memory
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>().swap(point_info_);
    {
      regionLock.lock();
      *out_ground.cloud_ptr += *tempGround.cloud_ptr;
      *out_no_ground.cloud_ptr += *tempVertical.cloud_ptr;
      regionLock.unlock();
    }
  }

  return true;
}

/**
 * @brief ground extraction
 * @param void
 * @return true if success otherwise false
 */
bool Segmentation::groundRemove() {
  /// step1.0 initialize multiple regions and quadrants
  initSections();

  /// step 2.0 Estimate the beam value of the lidar point cloud and the collected time of each point.
  double mean_height = estimateRingsAndTimes2(current_scan) + 0.5;
  filterByHeight(current_scan, mean_height, non_ground_scan);

  /// step 3.0 Fill in the index value of each sub-area according to the input point cloud data
  fillSectionIndex(current_scan);

  /// step 4.0 Multi-region ground extraction, each thread is responsible for one quadrant
  std::vector<std::future<bool>> threads(quadrant);
  std::vector<bool> results(quadrant);
  for (int q = 0; q < quadrant; ++q){
    threads[q] = std::async(std::launch::async | std::launch::deferred, &Segmentation::segmentGroundThread, this, q,
                            std::ref(current_scan), std::ref(object_scan), std::ref(ground_scan));
  }

  for (int q = 0; q < quadrant; ++q){
    results[q] = threads[q].get();
  }

  /// check that all threads have returned safely
  if (std::accumulate(results.begin(), results.end(), 0) == quadrant){
    *object_scan.cloud_ptr += *non_ground_scan.cloud_ptr;

    return true;
  }else{
    ROS_ERROR("multi-region ground extraction failed!");
    return false;
  }
}

/**
 * @brief get the index value in the polar radial direction
 * @param radius, polar diameter
 * @return polar diameter index
 */
int Segmentation::getPolarIndex(double &radius) {
  for (auto r = 0; r < polarNum; ++r){
    if (radius < polarBounds[r])
      return r;
  }

  return polarNum - 1;
}

/**
 * @brief converting rectangular coordinate to polar coordinates
 * @param cloud_in_, input point cloud
 * @return void
 */
void Segmentation::convertToPolar(const CloudData &cloud_in_) {
  if (cloud_in_.cloud_ptr->IsEmpty()){
    ROS_ERROR("object cloud don't have point, please check !");
    return;
  }

  auto azimuthCal = [&](double x, double y)->double{
    auto angle = static_cast<double>(std::atan2(y, x));
    return angle > 0.0 ? angle*180/M_PI : (angle+2*M_PI)*180/M_PI;
  };

  size_t totalSize = cloud_in_.cloud_ptr->points_.size();
  polarCor.resize(totalSize);

  Eigen::Vector3d cur = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < totalSize; i++){
    Eigen::Vector3d rpa = Eigen::Vector3d::Zero();
    cur = cloud_in_.cloud_ptr->points_[i];
    rpa.x() = cur.norm();                                     // polar
    rpa.y() = std::asin(cur.z() / rpa.x()) * 180.0 / M_PI;    // pitch
    rpa.z() = azimuthCal(cur.x(), cur.y());                   // azimuth

    if (rpa.x() >= sensorMaxRange || rpa.x() <= sensorMinRange)
      continue;

    minPitch = rpa.y() < minPitch ? rpa.y() : minPitch;
    maxPitch = rpa.y() > maxPitch ? rpa.y() : maxPitch;
    minPolar = rpa.x() < minPolar ? rpa.x() : minPolar;
    maxPolar = rpa.x() > maxPolar ? rpa.x() : maxPolar;

    polarCor[i] = rpa;
  }
  polarCor.shrink_to_fit();

  polarNum = 0;
  polarBounds.clear();
  width = static_cast<int>(std::round(360.0 / deltaA) + 1);
  height = static_cast<int>((maxPitch - minPitch) / deltaP);
  double range = minPolar;
  int step = 1;
  while (range <= maxPolar){
    range += (startR - step * deltaR);
    polarBounds.emplace_back(range);
    polarNum++, step++;
  }
}

/**
 * @brief 创建哈希表
 * @param void
 * @return true if success otherwise false
 */
bool Segmentation::createHashTable(void) {
  size_t totalSize = polarCor.size();
  if (totalSize <= 0){
    return false;
  }

  Eigen::Vector3d cur = Eigen::Vector3d::Zero();
  int polarIndex, pitchIndex, azimuthIndex, voxelIndex;
  voxelMap.reserve(totalSize);

  for (size_t item = 0; item < totalSize; ++item){
    cur = polarCor[item];
    polarIndex = getPolarIndex(cur.x());
    pitchIndex = static_cast<int>(std::round((cur.y() - minPitch) / deltaP));
    azimuthIndex = static_cast<int>(std::round(cur.z() / deltaA));

    voxelIndex = (azimuthIndex * (polarNum + 1) + polarIndex) + pitchIndex * (polarNum + 1) * (width + 1);

    auto iter = voxelMap.find(voxelIndex);
    if (iter != voxelMap.end()){
      //iter->second.index.emplace_back(item);
      iter->second.emplace_back(item);
    }else{
      std::vector<int> index{};
      index.emplace_back(item);
      voxelMap.insert(std::make_pair(voxelIndex, index));
    }
  }

  return true;
}


/**
  * @brief search for neighboring voxels
  * @param polar_index, polar diameter index
  * @param pitch_index, pitch angular index
  * @param azimuth_index, azimuth angular index
  * @param out_neighIndex, output adjacent voxel index set
  * @return void
  */
void Segmentation::searchKNN(int &polar_index, int &pitch_index, int &azimuth_index,
                                   std::vector<int> &out_neighIndex) const {

  for (auto z = pitch_index - 1; z <= pitch_index + 1; ++z){
    if (z < 0 || z > height)
      continue;
    for (int y = polar_index - 1; y <= polar_index + 1; ++y){
      if (y < 0 || y > polarNum)
        continue;

      for (int x = azimuth_index - 1; x <= azimuth_index + 1; ++x){
        int ax = x;
        if (ax < 0)
          ax = width - 1;
        if (ax > 300)
          ax = 300;

        out_neighIndex.emplace_back((ax*(polarNum+1)+y) + z*(polarNum+1)*(width+1));
      }
    }
  }
}

/**
 * @brief the Dynamic Curved-Voxle Clustering algoithm for fast and precise point cloud segmentaiton
 * @param label_info, output the category information of each point
 * @return true if success otherwise false
 */
bool Segmentation::DCVC(std::vector<int> &label_info) {
  int labelCount = 0;
  size_t totalSize = polarCor.size();
  if (totalSize <= 0){
    ROS_ERROR("there are not enough point clouds to complete the DCVC algorithm");
    return false;
  }

  label_info.resize(totalSize, -1);
  Eigen::Vector3d cur = Eigen::Vector3d::Zero();
  int polar_index, pitch_index, azimuth_index, voxel_index, currInfo, neighInfo;

  for (size_t i = 0; i < totalSize; ++i){
    if (label_info[i] != -1)
      continue;
    cur = polarCor[i];

    polar_index = getPolarIndex(cur.x());
    pitch_index = static_cast<int>(std::round((cur.y() - minPitch) / deltaP));
    azimuth_index = static_cast<int>(std::round(cur.z() / deltaA));
    voxel_index = (azimuth_index*(polarNum+1) + polar_index) + pitch_index*(polarNum+1)*(width+1);

    auto iter_find = voxelMap.find(voxel_index);
    std::vector<int> neighbors;
    if (iter_find != voxelMap.end()){

      std::vector<int> KNN{};
      searchKNN(polar_index, pitch_index, azimuth_index, KNN);

      for (auto& k : KNN){
        iter_find = voxelMap.find(k);

        if (iter_find != voxelMap.end()){
          neighbors.reserve(iter_find->second.size());
          for (auto& id : iter_find->second){
            neighbors.emplace_back(id);
          }
        }
      }
    }

    neighbors.swap(neighbors);

    if (!neighbors.empty()){
      for (auto& id : neighbors){
        currInfo = label_info[i];       // current label index
        neighInfo = label_info[id];     // voxel label index
        if (currInfo != -1 && neighInfo != -1 && currInfo != neighInfo){
          for (auto& seg : label_info){
            if (seg == currInfo)
              seg = neighInfo;
          }
        }else if (neighInfo != -1){
          label_info[i] = neighInfo;
        }else if (currInfo != -1){
          label_info[id] = currInfo;
        }else{
          continue;
        }
      }
    }

    // If there is no category information yet, then create a new label information
    if (label_info[i] == -1){
      labelCount++;
      label_info[i] = labelCount;
      for (auto& id : neighbors){
        label_info[id] = labelCount;
      }
    }
  }

  // free memory
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(polarCor);

  return true;
}

/**
 * @brief obtain the point cloud data of each category
 * @param label_info, input category information
 * @return void
 */
void Segmentation::labelAnalysis(std::vector<int> &label_info) {

  std::unordered_map<int, segInfo> histCounts;
  size_t totalSize = label_info.size();
  for (size_t i = 0; i < totalSize; ++i){
    if (histCounts.find(label_info[i]) == histCounts.end()){
      histCounts[label_info[i]].label = 1;
      histCounts[label_info[i]].index.emplace_back(i);
    }else{
      histCounts[label_info[i]].label += 1;
      histCounts[label_info[i]].index.emplace_back(i);
    }
  }

  std::vector<std::pair<int, segInfo>> labelStatic(histCounts.begin(), histCounts.end());
  std::sort(labelStatic.begin(), labelStatic.end(), [&](std::pair<int, segInfo>& a, std::pair<int, segInfo>& b)->bool{
    return a.second.label > b.second.label;
  });

  auto count{1};
  for (auto& info : labelStatic){
    if (info.second.label > minSeg){
      labelRecords.emplace_back(std::make_pair(count, info.second));
      count++;
    }
  }

  // free memory
  std::unordered_map<int, segInfo>().swap(histCounts);
  std::vector<std::pair<int, segInfo>>().swap(labelStatic);
}

/**
 * @brief statistics label information and render the colors for better visualization
 * @param void
 * @return true if success otherwise false
 */
bool Segmentation::colorSegmentation() {
  for (auto& label : labelRecords){
    // box
    jsk_recognition_msgs::BoundingBox box;
    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    for (auto& id : label.second.index){
      segmented_scan.cloud_ptr->points_.emplace_back(object_scan.cloud_ptr->points_[id]);
      segmented_scan.cloud_ptr->intensity_.emplace_back(object_scan.cloud_ptr->intensity_[id]);

      min_x = std::min(min_x, object_scan.cloud_ptr->points_[id].x());
      max_x = std::max(max_x, object_scan.cloud_ptr->points_[id].x());
      min_y = std::min(min_y, object_scan.cloud_ptr->points_[id].y());
      max_y = std::max(max_y, object_scan.cloud_ptr->points_[id].y());
      min_z = std::min(min_z, object_scan.cloud_ptr->points_[id].z());
      max_z = std::max(max_z, object_scan.cloud_ptr->points_[id].z());
    }

    double lengthBox = max_x - min_x;
    double widthBox = max_y - min_y;
    double heightBox = max_z - min_z;
    box.header.stamp = current_time;
    box.header.frame_id = "/velo_link";
    box.label = label.first;
    Eigen::Vector3d box_in_map(min_x + lengthBox / 2.0, min_y + widthBox / 2.0, min_z + heightBox / 2.0);
    box.pose.position.x = box_in_map.x();
    box.pose.position.y = box_in_map.y();
    box.pose.position.z = box_in_map.z();

    box.dimensions.x = ((lengthBox < 0) ? -1 * lengthBox : lengthBox);
    box.dimensions.y = ((widthBox < 0) ? -1 * widthBox : widthBox);
    box.dimensions.z = ((heightBox < 0) ? -1 * heightBox : heightBox);

    boxInfo.emplace_back(box);
  }

  boxInfo.shrink_to_fit();
  segmented_scan.cloud_ptr->points_.shrink_to_fit();
  segmented_scan.cloud_ptr->intensity_.shrink_to_fit();

  return true;
}

/**
 * @brief category segmentation for non-ground points
 * @brief void
 * @return true if success otherwise false
 */
bool Segmentation::objectSegmentation() {
  /// step 1.0 convert point cloud to polar coordinate system
  //Timer test_time;
  if (!object_scan.cloud_ptr->IsEmpty()){
    convertToPolar(object_scan);
  }else{
    ROS_ERROR("not enough point to convert");
    return false;
  }

  /// step 2.0 Create a hash table
  createHashTable();

  /// step 3.0 DCVC segmentation
  std::vector<int> labelInfo{};
  if (!DCVC(labelInfo)){
    ROS_ERROR("DCVC algorithm segmentation failure");
    return false;
  }

  /// step 4.0 statistics category record
  labelAnalysis(labelInfo);

  /// step 5.0 establish each category boundingbox
  colorSegmentation();

  return true;
}

/**
 * @brief reset related parameters for the next step
 * @param void
 * @return void
 */
void Segmentation::resetParams() {
  width = 0.0;
  height = 0.0;
  minPitch = maxPitch = 0.0;
  minPolar = maxPolar = 0.0;

  boxInfo.clear();
  polarCor.clear();
  voxelMap.clear();
  labelRecords.clear();
  polarBounds.clear();

  current_scan.reset();
  non_ground_scan.reset();
  ground_scan.reset();
  object_scan.reset();
  segmented_scan.reset();
  edge_scan.reset();
  general_scan.reset();

  for (int i = 0; i < quadrant; i++){
    regionIndex[i]->clear();
  }
}

void Segmentation::extractFromSection(CloudData &cloud_in_,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &curvature_,
    CloudData &out_edge_cloud_, CloudData &non_edge_cloud_) {

  std::sort(curvature_.begin(), curvature_.end(), [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b){
    return a[1] < b[1];
  });

  int largestPickedNum{0};
  std::vector<int> picked_points{};
  int point_info_count{0};

  for (int i = int(curvature_.size()) - 1; i >= 0; --i){

    auto id = static_cast<int>(curvature_[i][0]);
    if (std::find(picked_points.begin(), picked_points.end(), id) == picked_points.end()){
      if (curvature_[i][1] <= 0.1)
        break;

      largestPickedNum++;

      //std::cout << "pt i: " << i << " , largestPicked: " << largestPickedNum << std::endl;

      picked_points.emplace_back(id);
      if (largestPickedNum <= 20){
        out_edge_cloud_.cloud_ptr->points_.emplace_back(cloud_in_.cloud_ptr->points_[id]);
        out_edge_cloud_.cloud_ptr->intensity_.emplace_back(cloud_in_.cloud_ptr->intensity_[id]);
        point_info_count++;
      }else{
        break;
      }

      for (int k = 1; k <= 5; ++k){
        double diffX = cloud_in_.cloud_ptr->points_[id+k].x() - cloud_in_.cloud_ptr->points_[id+k-1].x();
        double diffY = cloud_in_.cloud_ptr->points_[id+k].y() - cloud_in_.cloud_ptr->points_[id+k-1].y();
        double diffZ = cloud_in_.cloud_ptr->points_[id+k].z() - cloud_in_.cloud_ptr->points_[id+k-1].z();

        if (diffX*diffX + diffY*diffY + diffZ*diffZ > 0.05){
          break;
        }

        picked_points.emplace_back(id + k);
      }

      for (int k = -1; k >= -5; k--){
        double diffX = cloud_in_.cloud_ptr->points_[id+k].x() - cloud_in_.cloud_ptr->points_[id+k+1].x();
        double diffY = cloud_in_.cloud_ptr->points_[id+k].y() - cloud_in_.cloud_ptr->points_[id+k+1].y();
        double diffZ = cloud_in_.cloud_ptr->points_[id+k].z() - cloud_in_.cloud_ptr->points_[id+k+1].z();

        if (diffX*diffX + diffY*diffY + diffZ*diffZ > 0.05){
          break;
        }

        picked_points.emplace_back(id + k);
      }

    }
  }

  for (size_t i = 0, totalSize = curvature_.size(); i <= totalSize - 1; ++i){
    auto id = static_cast<int>(curvature_[i][0]);

    if (std::find(picked_points.begin(), picked_points.end(), id) == picked_points.end()){
      non_edge_cloud_.cloud_ptr->points_.emplace_back(cloud_in_.cloud_ptr->points_[id]);
      non_edge_cloud_.cloud_ptr->intensity_.emplace_back(cloud_in_.cloud_ptr->intensity_[id]);
    }
  }
}

/**
 * @brief edge features extraction based on the smoothness of the local surface
 * @param cloud_in_, input non-ground point
 * @param out_edge_point, output edge features
 * @param non_edge_point, output non-edge features
 * @return true if success otherwise false
 */
bool Segmentation::extractEdgePoint(const CloudData &cloud_in_, CloudData &out_edge_point_,
                                    CloudData &non_edge_point_) {
  if (cloud_in_.cloud_ptr->IsEmpty()){
    ROS_ERROR("not enough points.");
    return false;
  }

  std::vector<CloudData> ringScans(sensorModel);

  size_t totalSize = cloud_in_.cloud_ptr->points_.size();
  int beamId{0};
  for (size_t i = 0; i < totalSize; ++i){
    beamId = static_cast<int>(cloud_in_.cloud_ptr->intensity_[i]);
    if (beamId >= 0 && beamId <= sensorModel){
      ringScans[beamId].cloud_ptr->points_.emplace_back(cloud_in_.cloud_ptr->points_[i]);
      ringScans[beamId].cloud_ptr->intensity_.emplace_back(cloud_in_.cloud_ptr->intensity_[i]);
    }
  }

  for (int i = 0; i < sensorModel; ++i){
    if (static_cast<int>(ringScans[i].cloud_ptr->points_.size()) < ringMinNum)
      continue;

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> curvature;

    auto totalSize = static_cast<int>(ringScans[i].cloud_ptr->points_.size());
    auto totalPoints = static_cast<int>(totalSize - 10);

    for (int j = 5, beamSize = totalSize-5; j < beamSize; ++j){
      double diffX = ringScans[i].cloud_ptr->points_[j - 5].x() +
                     ringScans[i].cloud_ptr->points_[j - 4].x() +
                     ringScans[i].cloud_ptr->points_[j - 3].x() +
                     ringScans[i].cloud_ptr->points_[j - 2].x() +
                     ringScans[i].cloud_ptr->points_[j - 1].x() -
                     10 * ringScans[i].cloud_ptr->points_[j].x() +
                     ringScans[i].cloud_ptr->points_[j + 1].x() +
                     ringScans[i].cloud_ptr->points_[j + 2].x() +
                     ringScans[i].cloud_ptr->points_[j + 3].x() +
                     ringScans[i].cloud_ptr->points_[j + 4].x() +
                     ringScans[i].cloud_ptr->points_[j + 5].x();
      double diffY = ringScans[i].cloud_ptr->points_[j - 5].y() +
                     ringScans[i].cloud_ptr->points_[j - 4].y() +
                     ringScans[i].cloud_ptr->points_[j - 3].y() +
                     ringScans[i].cloud_ptr->points_[j - 2].y() +
                     ringScans[i].cloud_ptr->points_[j - 1].y() -
                     10 * ringScans[i].cloud_ptr->points_[j].y() +
                     ringScans[i].cloud_ptr->points_[j + 1].y() +
                     ringScans[i].cloud_ptr->points_[j + 2].y() +
                     ringScans[i].cloud_ptr->points_[j + 3].y() +
                     ringScans[i].cloud_ptr->points_[j + 4].y() +
                     ringScans[i].cloud_ptr->points_[j + 5].y();
      double diffZ = ringScans[i].cloud_ptr->points_[j - 5].z() +
                     ringScans[i].cloud_ptr->points_[j - 4].z() +
                     ringScans[i].cloud_ptr->points_[j - 3].z() +
                     ringScans[i].cloud_ptr->points_[j - 2].z() +
                     ringScans[i].cloud_ptr->points_[j - 1].z() -
                     10 * ringScans[i].cloud_ptr->points_[j].z() +
                     ringScans[i].cloud_ptr->points_[j + 1].z() +
                     ringScans[i].cloud_ptr->points_[j + 2].z() +
                     ringScans[i].cloud_ptr->points_[j + 3].z() +
                     ringScans[i].cloud_ptr->points_[j + 4].z() +
                     ringScans[i].cloud_ptr->points_[j + 5].z();

      curvature.emplace_back(Eigen::Vector2d(static_cast<double>(j), diffX*diffX + diffY*diffY + diffZ*diffZ));
    }

    for (int j = 0; j < 6; j++){
      auto sector_length = static_cast<int>(totalPoints / 6);
      auto sector_start = sector_length * j;
      auto sector_end = j != 5 ? sector_length * (j + 1) - 1 : totalPoints -1;

      std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> subCurvature(curvature.begin() + sector_start, curvature.begin() + sector_end);
      extractFromSection(ringScans[i], subCurvature, out_edge_point_, non_edge_point_);
    }
  }
  out_edge_point_.cloud_ptr->points_.shrink_to_fit();
  out_edge_point_.cloud_ptr->intensity_.shrink_to_fit();
  non_edge_point_.cloud_ptr->points_.shrink_to_fit();
  non_edge_point_.cloud_ptr->intensity_.shrink_to_fit();


  return true;
}

}


