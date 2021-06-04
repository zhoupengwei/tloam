/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午4:40
 * @FileName: kitti_reader.hpp
 * @Description: read kitti odometry benchmark dataset include velodyne image
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_KITTI_READER_HPP
#define TLOAM_KITTI_READER_HPP

#include <cmath>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <open3d/Open3D.h>
#include "tloam/open3d/PointCloud2.hpp"
#include "tloam/open3d/open3d_to_ros.hpp"
#include "tloam/models/io/read_file.hpp"
#include "tloam/models/utils/work_space_path.h"

// publisher
// a. velodyne
#include "tloam/publisher/cloud_publisher.hpp"
// b. ground truth
#include "tloam/publisher/odometry_publisher.hpp"
// c. image
#include "tloam/publisher/image_publisher.hpp"
// d. tf
#include "tloam/publisher/tf_broadcaster.hpp"

namespace tloam {
class KittiReader {
public:
  KittiReader(ros::NodeHandle &nh);

  ~KittiReader() = default;

  /**
   * @brief Read a frame point cloud and image at each refresh
   * @param void
   * @return void
   */
  void spinOnce(void);

  /**
   * @brief Initialization configuration
   * @param void
   * @return true if success otherwise false
   */
  bool InitWithConfig(void);

  /**
   * @brief Initialization file name
   * @param void
   * @return true if success otherwise false
   */
  bool initScanFilenames(void);

private:
  /**
   * @brief Read a frame of point cloud
   * @param cloud_in, save point cloud data
   * @param index, filename index
   * @return true if success other false
   */
  bool readScan(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_in, size_t index);

  /**
   * @brief Read grayscale image
   * @param image_, save image data
   * @param kind_index, image kind index indicate which type of image is currently being read
   * @param index, filename index
   * @return true if success otherwise false
   */
  bool readImageGray(std::shared_ptr<cv::Mat> &image_, size_t kind_index, size_t index);

  /**
   * @brief Read rgb image
   * @param image_, save image data
   * @param kind_size, image kind index indicate which type of image is currently being read
   * @param index, filename index
   * @return true if success otherwise false
   */
  bool readImageColor(std::shared_ptr<cv::Mat> &image_, size_t kind_size, size_t index);

  /**
   * @brief read ground truth
   * @param file_name, file name
   * @param pose, save pose information
   * @return true if success otherwise false
   */
  bool readPose(const std::string &file_name,
                std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &pose);

  /**
   * @brief Read all folders on the current path
   * @param dir, the path needs to be read
   * @param dir_list, save file path
   * @return true if success otherwise false
   */
  bool getDirectoryList(const std::string &dir, std::vector<std::string> &dir_list);

  /**
   * @brief Delimits a string according to a specified delimter
   * @param content, save substr
   * @param line, a string to be delimited
   * @param delimiter, specified separator
   * @param skipEmpty, whether to skip an empty substring
   * @return void
   */
  void split(std::vector<std::string> &content, const std::string &line, const std::string &delimiter = " ",
             bool skipEmpty = false);

  /**
   * @brief Trim the specificed whitespaces of string
   * @param str_trim, result string after delete whitespaces
   * @param str, pending string
   * @param whitespaces, blank character to be deleted
   * @return void
   */
  void trim(std::string &str_trim, const std::string &str, const std::string &whitespaces = " \0\t\n\r\x0B");

  /**
   * @brief return the extension of the file
   * @param path, the file name
   * @param level, maximum number of extension levels. For instance, ".tar.gz" has level 2.
   * @return string, file extension
   */
  std::string extension(const std::string &path, size_t level = 1);

private:
  // publish
  // a. velodyne
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  // b. ground_truth
  std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
  // c. image
  std::shared_ptr<ImagePublisher> image_pub_ptr_;
  std::shared_ptr<ImagePublisher> image_pub2_ptr_;
  // d. tf
  std::shared_ptr<TFBroadCaster> laser_to_camera_pub_ptr_;
  std::shared_ptr<TFBroadCaster> ground_truth_pub_ptr_;
  std::shared_ptr<TFBroadCaster> map_to_camera_pub_ptr_;

  std::vector<std::string> scan_filenames_{};
  std::vector<std::vector<std::string>> image_filenames_{};
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> ground_truth_{};

  std::vector<float> timeStamp_;

  // calibration T_r
  Eigen::Isometry3d T_camera_laser_;
  // odometry sequence
  std::string seq_num_;
  // velodyne publish topic name
  std::string topic_name_;
  // kitti odometry benchmark odometry dataset path
  std::string data_path_;
  // image kind size image_0, image_1, image_2, image_3
  size_t image_kind_size;

};
}


#endif //TLOAM_KITTI_READER_HPP
