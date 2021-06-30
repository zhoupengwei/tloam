/**
 * @Copyright 2021, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/14 上午10:47
 * @FileName: segmentation.hpp
 * @Description: Fast and precise point cloud segmentation including multi-region ground extraction and dynamic curved-voxel clustering
 * @License: See LICENSE for the license information
 */
#ifndef TLOAM_SEGMENTATION_HPP
#define TLOAM_SEGMENTATION_HPP

// for std
#include <iostream>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <future>
#include <utility>
#include <algorithm>
// for Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
// for OpenCV
#include <opencv2/opencv.hpp>
// for ros
#include <omp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
// for Open3D
#include <open3d/Open3D.h>
#include "tloam/open3d/PointCloud2.hpp"
#include "tloam/open3d/open3d_to_ros.hpp"
// for cloud data
#include "tloam/models/utils/sensor_data.hpp"
// for subscriber
#include "tloam/subscriber/cloud_subscriber.hpp"
// for publisher
#include "tloam/publisher/boundingbox_publisher.hpp"
#include "tloam/publisher/cloud_publisher.hpp"
// for listener
#include "tloam/subscriber/tf_listener.hpp"
// for OpenMp
#include <omp.h>
// for ros
#include <ros/ros.h>

#include "tloam/models/utils/utils.hpp"
#include "tloam/models/utils/work_space_path.h"

namespace tloam{
class Segmentation{
public:
  enum LiDAR{
    HDL_64E = 64,
    VLP_16 = 16
  };
  Segmentation() = default;
  Segmentation(ros::NodeHandle& nh, std::string& topic_name);
  ~Segmentation();

  void spinOnce();

  bool initWithConfig();

protected:

  void initVeldyneConfig(const YAML::Node& node);

  void initGroundSegConfig(const YAML::Node& node);

  void initDCVCConfig(const YAML::Node& node);

  bool readData();

  bool hasData();

  bool validData();

  /**
   * @brief initialize ring radius table and section bounds
   * @param void
   * @return true if success otherwise false
   */
  bool initSections(void);

  /**
   * @brief determine the section number according to the polar diameter.
   * @param radius, polar diameter data of the point cloud
   * @return section number
   */
  int getSection(const double& radius);

  /**
   * @brief
   * @param cloud_in_, the point cloud to be estimated
   * @param cloud_out_, output the beams and collected time information of each point and stored in the intensity channel.
   * @return true if success otherwise false
   */
  bool estimateRingsAndTimes(
    CloudData& cloud_in_,
    CloudData& cloud_out_
  ) const;

  /**
   * @brief estimation of LiDAR beam and time information
   * @param cloud_in_, input point cloud
   * @return mean_height of point cloud
   */
  double estimateRingsAndTimes2(
    CloudData& cloud_in_
  ) const;

  /**
   * @brief filter out part of the vertical point cloud according to the height value
   * @param cloud_in_
   * @param mean_height_
   * @param cloud_out_
   * @return true if success otherwise false
   */
  bool filterByHeight(
    CloudData& cloud_in_,
    double& mean_height_,
    CloudData& cloud_out_
  );

  /**
   * @brief Remove some close points and invalid points
   * @param cloud_in_
   * @param dis_th
   * @param remove_nan
   * @param remove_infinite
   */
  void RemoveClosedNonFinitePoints(
     CloudData& cloud_in_,
     double dis_th = 3.0,
     bool remove_nan = true,
     bool remove_infinite = true
 );

  /**
   * @brief calculate the region index of each point
   * @param cloud_in_, input point cloud
   * @return true if success otherwise false
   */
  bool fillSectionIndex(CloudData& cloud_in_);

  /**
   * @brief Find best plane based on matrix-axis linear regression
   * @param cloud_in_, input point cloud data
   * @param out_plane_, output plane model
   * @return void
   */
  void findBestPlane(CloudData& cloud_in_, Eigen::Vector4d& out_plane_);

  /**
   * @brief thread function of ground extraction
   * @param q, quadrant index
   * @param cloud_in_, input point cloud
   * @param out_no_ground, output non-ground point cloud
   * @param out_ground, output ground point cloud
   * @return true if success otherwise false
   */
  bool segmentGroundThread(
    int q,
    const CloudData& cloud_in_,
    CloudData& out_no_ground,
    CloudData& out_ground
  );

  /**
   * @brief ground extraction
   * @param void
   * @return true if success otherwise false
   */
  bool groundRemove(void);

  /**
   * @brief get the index value in the polar radial direction
   * @param radius, polar diameter
   * @return polar diameter index
   */
  int getPolarIndex(double& radius);

  /**
   * @brief converting rectangular coordinate to polar coordinates
   * @param cloud_in_, input point cloud
   * @return void
   */
  void convertToPolar(const CloudData& cloud_in_);

  /**
   * @brief Create a hash table
   * @param void
   * @return true if success otherwise false
   */
  bool createHashTable(void);

  /**
   * @brief search for neighboring voxels
   * @param polar_index, polar diameter index
   * @param pitch_index, pitch angular index
   * @param azimuth_index, azimuth angular index
   * @param out_neighIndex, output adjacent voxel index set
   * @return void
   */
  void searchKNN(
    int& polar_index,
    int& pitch_index,
    int& azimuth_index,
    std::vector<int>& out_neighIndex
  ) const;

  /**
   * @brief the Dynamic Curved-Voxle Clustering algoithm for fast and precise point cloud segmentaiton
   * @param label_info, output the category information of each point
   * @return true if success otherwise false
   */
  bool DCVC(std::vector<int>& label_info);

  /**
   * @brief obtain the point cloud data of each category
   * @param label_info, input category information
   * @return void
   */
  void labelAnalysis(std::vector<int>& label_info);

  /**
   * @brief statistics label information and render the colors for better visualization
   * @param void
   * @return true if success otherwise false
   */
  bool colorSegmentation(void);

  /**
   * @brief category segmentation for non-ground points
   * @brief void
   * @return true if success otherwise false
   */
  bool objectSegmentation(void);

  /**
   * @brief reset related parameters for the next step
   * @param void
   * @return void
   */
  void resetParams(void);

  /**
   * @brief edge features extraction based on the smoothness of the local surface
   * @param cloud_in_, input non-ground point
   * @param out_edge_point, output edge features
   * @param non_edge_point, output non-edge features
   * @return true if success otherwise false
   */
  bool extractEdgePoint(
    const CloudData& cloud_in_,
    CloudData& out_edge_point_,
    CloudData& non_edge_point_
  );

  void extractFromSection(
    CloudData& cloud_in_,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& curvature_,
    CloudData& out_edge_cloud_,
    CloudData& non_edge_cloud_
  );

private:
  class Voxel{
  public:
    int label{-1};
    std::vector<int> index{};
  };

  class segInfo{
  public:
    int label{-1};
    std::vector<int> index{};
  };


  // subscriber
  std::unique_ptr<CloudSubscriber> cloud_sub_ptr_;
  // publisher
  std::unique_ptr<CloudPublisher> cloud_pub_ground_ptr_;
  std::unique_ptr<CloudPublisher> cloud_pub_object_ptr_;
  std::unique_ptr<CloudPublisher> cloud_pub_edge_ptr_;
  std::unique_ptr<CloudPublisher> cloud_pub_general_ptr_;
  std::unique_ptr<BoundingBoxPublisher> box_pub_ptr_;
  // listener
  std::unique_ptr<TFListener> velo_link_to_map_ptr_;
  Eigen::Matrix4d velo_link_to_map = Eigen::Matrix4d::Identity();

  // 3D point cloud data
  CloudData current_scan{};
  CloudData non_ground_scan{};
  CloudData ground_scan{};
  CloudData object_scan{};
  CloudData segmented_scan{};
  CloudData edge_scan{};
  CloudData general_scan{};

  // LiDAR params
  int sensorModel{64};
  double scanPeriod{0.0};
  double verticalRes{0.0};
  double initAngle{0.0};
  double sensorHeight{0.0};
  double sensorMinRange{0.0};
  double sensorMaxRange{0.0};
  double near_dis{3.0};

  // multi-ground extraction params
  double planeDis{0.0};
  int numSec{0};
  int quadrant{0};
  int groundSeedNum{0};
  int maxIter{0};
  int ringMinNum{0};

  Eigen::VectorXd radiusTableDiff;
  Eigen::VectorXd radiusTable;
  std::vector<size_t>** regionIndex{};
  std::vector<double> sectionBounds{};

  // DCVC params
  double minPitch{0.0};
  double maxPitch{0.0};
  double minPolar{5.0};
  double maxPolar{5.0};
  double startR{0.0};
  double deltaR{0.0};
  double deltaP{0.0};
  double deltaA{0.0};
  int width{0};
  int height{0};
  int minSeg{0};
  int polarNum{0};
  std::vector<double> polarBounds{};
  std::vector<jsk_recognition_msgs::BoundingBox> boxInfo{};

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> polarCor;
  //std::unordered_map<int, Voxel> voxelMap{};
  std::unordered_map<int, std::vector<int>> voxelMap{};
  std::vector<std::pair<int, segInfo>> labelRecords;

  std::deque<CloudData> cloud_data_buff_;

  int num_threads_;
  std::mutex regionMutex{};
  std::mutex bufferMutex{};

  ros::Time current_time;

  double rgb_table[37][3] = {
    {255, 218, 185},    // PeachPuff
    {255, 245, 238},    // Seashell
    {100, 149, 237},    // ComflowerBlue
    {132, 112, 255},    // LightSlateBlue
    {0, 0, 255},        // Blue
    {30, 144, 255},     // DogerBlue
    {0, 191, 255},      // DeepSkyBlue
    {0, 206, 209},      // DarkTurquoise
    {127, 255, 212},    // Aquamarine
    {0, 255, 127},      // SpringGreen
    {173, 255, 47},     // GreenYellow
    {255, 255, 0},      // Yellow
    {255, 215, 0},      // Gold
    {255, 140, 0},      // DarkOrange
    {255, 105, 180},    // HotPink
    {255, 20, 147},     // DeepPink
    {255, 0, 255},      // Magenta
    {186, 85, 211},     // MediumOrchild
    {138, 43, 226},     // BlueViolet
    {131, 111, 255},    // SlateBlue1
    {72, 118, 255},     // RoyalBlue1
    {0, 245, 255},      // Turquoise1
    {127, 255, 212},    // Aquamarine1
    {84, 255, 159},     // SeaGreen1
    {0, 255, 0},        // Green1
    {127, 255, 0},      // Chartreuse1
    {255, 255, 0},      // Yellow1
    {238, 121, 66},     // Sienna1
    {255, 193, 193},    // RosyBrown1
    {255, 140, 105},    // Salmon1
    {255, 52, 179},     // Maroon1
    {255, 62, 150},     // VioletRed1
    {255, 131, 250},    // Orchid1
    {255, 0, 255},      // Magenta1
    {191, 62, 255},     // DarkOrchild1
    {144, 238, 144},    // LightGreen
    {155, 48, 255}      // Purple1
  };
};

}

#endif //TLOAM_SEGMENTATION_HPP
