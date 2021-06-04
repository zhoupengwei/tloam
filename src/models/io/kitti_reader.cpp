/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午4:41
 * @FileName: kitti_reader.cpp
 * @Description: read kitti odometry benchmark dataset include velodyne image ground_truth
 * @License: See LICENSE for the license information
 */

#include "tloam/models/io/kitti_reader.hpp"

namespace tloam {
KittiReader::KittiReader(ros::NodeHandle &nh) {
  // Initialization configuration
  InitWithConfig();
  // publish
  // a. velodyne point cloud
  cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, topic_name_, "/velo_link", 10000);
  // b. gnss ground path
  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/gnss", "/map", "/velo_link", 10000);
  // c. image rgb
  image_pub_ptr_ = std::make_shared<ImagePublisher>(nh, "/image_0", "/camera", 10000, false);
  image_pub2_ptr_ = std::make_shared<ImagePublisher>(nh, "/image_1", "/camera", 10000, true);
  // d. tf /map /velo_link
  ground_truth_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/velo_link");
  // e. tf lidar to camera
  laser_to_camera_pub_ptr_ = std::make_shared<TFBroadCaster>("/camera", "/velo_link");
  // f. tf /map /camera
  map_to_camera_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/camera");
}

bool KittiReader::InitWithConfig() {
  std::string config_file_path = WORK_SPACE_PATH + "/config/kitti/kitti_reader.yaml";

  YAML::Node config_node = YAML::LoadFile(config_file_path);

  data_path_ = config_node["data_path"].as<std::string>();
  seq_num_ = config_node["sequence_num"].as<std::string>();
  topic_name_ = config_node["topic_name"].as<std::string>();
  image_kind_size = config_node["image_kind_size"].as<int>();
  assert(image_kind_size >= 0 && image_kind_size <= 4);
  image_filenames_.resize(image_kind_size);
  return true;
}

void KittiReader::spinOnce() {
  static size_t file_index = 0;

  std::shared_ptr<open3d::geometry::PointCloud2> velodyne_scan = std::make_shared<open3d::geometry::PointCloud2>();
  std::shared_ptr<cv::Mat> image_1 = std::make_shared<cv::Mat>();
  std::shared_ptr<cv::Mat> image_2 = std::make_shared<cv::Mat>();

  double time = ros::Time::now().toSec();
  if (readScan(velodyne_scan, file_index) && file_index < scan_filenames_.size()) {
    cloud_pub_ptr_->Publisher(velodyne_scan, time);
  }
  else{
    ROS_INFO("All the point cloud has been read, finish publishing and success exit.");
    ros::shutdown();
    return;
  }

  if (image_kind_size == 1 && readImageGray(image_1, 0, file_index)){
    if (image_pub_ptr_->HasSubscribers()){
      image_pub_ptr_->SetColor(false);
      image_pub_ptr_->Publish(image_1, time);
    }
  }
  else if (image_kind_size == 2 && readImageGray(image_1, 0, file_index) && readImageGray(image_2, 1, file_index)){
    if (image_pub_ptr_->HasSubscribers()){
      image_pub_ptr_->SetColor(false);
      image_pub_ptr_->Publish(image_1, time);
    }
    if (image_pub2_ptr_->HasSubscribers()){
      image_pub2_ptr_->SetColor(false);
      image_pub2_ptr_->Publish(image_2, time);
    }
  }
  else if (image_kind_size == 4 && readImageColor(image_1, 2, file_index) && readImageColor(image_2, 3, file_index)){
    if (image_pub_ptr_->HasSubscribers()){
      image_pub_ptr_->SetColor(true);
      image_pub_ptr_->Publish(image_1, time);
    }
    if (image_pub2_ptr_->HasSubscribers()){
      image_pub2_ptr_->SetColor(true);
      image_pub2_ptr_->Publish(image_2, time);
    }
  }

  if (file_index <= ground_truth_.size()){
    laser_to_camera_pub_ptr_->SendTransform(T_camera_laser_, time);

    Eigen::Isometry3d T_map_camera_init(ground_truth_[0]);
    Eigen::Isometry3d T_map_camera(ground_truth_[file_index]);

    Eigen::Isometry3d T_map_velo = T_camera_laser_.inverse() * T_map_camera_init * T_map_camera * T_camera_laser_;

    gnss_pub_ptr_->Publish(T_map_velo, time);
    ground_truth_pub_ptr_->SendTransform(T_map_velo, time);

    //Eigen::Isometry3d T_map_camera = T_map_velo * T_camera_laser_.translate();
    map_to_camera_pub_ptr_->SendTransform(T_map_camera, time);
  }
  file_index++;
}

bool KittiReader::initScanFilenames() {
  std::vector<std::string> listPath;
  getDirectoryList(data_path_, listPath);
  for (const auto &path : listPath) {
    std::string baseName = path.substr(path.find_last_of('/') + 1);
    if (baseName == "dataset") {
      std::string velodyne_path = path + "/sequences/" + seq_num_ + "/velodyne/";

      if (boost::filesystem::exists(velodyne_path)) {
        std::vector<std::string> velodyne_bin_path;
        getDirectoryList(velodyne_path, velodyne_bin_path);
        for (const auto &bin_path : velodyne_bin_path) {
          if (extension(bin_path) == ".bin" && boost::filesystem::exists(bin_path)) {
            scan_filenames_.emplace_back(bin_path);
          }
        }

        scan_filenames_.shrink_to_fit();
        std::sort(scan_filenames_.begin(), scan_filenames_.end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });
      } else {
        ROS_ERROR(
            "Please check the velodyne point cloud path of the dataset, the algorithm needs at least point cloud");
        exit(EXIT_FAILURE);
      }

      // for image
      std::string image_0_path = path + "/sequences/" + seq_num_ + "/image_0/";
      std::string image_1_path = path + "/sequences/" + seq_num_ + "/image_1/";
      std::string image_2_path = path + "/sequences/" + seq_num_ + "/image_2/";
      std::string image_3_path = path + "/sequences/" + seq_num_ + "/image_3/";

      if (image_kind_size == 0) {
        ROS_INFO("No image data available to read.");
        break;
      }

      if (image_kind_size == 1 && boost::filesystem::exists(image_0_path)) {
        std::vector<std::string> image_0_file_path;
        getDirectoryList(image_0_path, image_0_file_path);
        for (const auto &gray_image : image_0_file_path) {
          if (extension(gray_image) == ".png" && boost::filesystem::exists(gray_image)) {
            image_filenames_[0].emplace_back(gray_image);
          }
        }
        image_filenames_[0].shrink_to_fit();
        std::sort(image_filenames_[0].begin(), image_filenames_[0].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });
      } else if (image_kind_size == 2 && boost::filesystem::exists(image_0_path) &&
                 boost::filesystem::exists(image_1_path)) {
        std::vector<std::string> image_0_file_path;
        getDirectoryList(image_0_path, image_0_file_path);
        for (const auto &gray_image : image_0_file_path) {
          if (extension(gray_image) == ".png" && boost::filesystem::exists(gray_image)) {
            image_filenames_[0].emplace_back(gray_image);
          }
        }
        image_filenames_[0].shrink_to_fit();
        std::sort(image_filenames_[0].begin(), image_filenames_[0].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });

        std::vector<std::string> image_1_file_path;
        getDirectoryList(image_1_path, image_1_file_path);

        for (const auto &gray_image : image_1_file_path) {
          if (extension(gray_image) == ".png" && boost::filesystem::exists(gray_image)) {
            image_filenames_[1].emplace_back(gray_image);
          }
        }
        image_filenames_[1].shrink_to_fit();
        std::sort(image_filenames_[1].begin(), image_filenames_[1].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });
      } else if (image_kind_size == 4 && boost::filesystem::exists(image_0_path) &&
                 boost::filesystem::exists(image_1_path) &&
                 boost::filesystem::exists(image_2_path) &&
                 boost::filesystem::exists(image_3_path)) {

        std::vector<std::string> image_0_file_path;
        getDirectoryList(image_0_path, image_0_file_path);
        for (const auto &gray_image : image_0_file_path) {
          if (extension(gray_image) == ".png" && boost::filesystem::exists(gray_image)) {
            image_filenames_[0].emplace_back(gray_image);
          }
        }
        image_filenames_[0].shrink_to_fit();
        std::sort(image_filenames_[0].begin(), image_filenames_[0].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });

        std::vector<std::string> image_1_file_path;
        getDirectoryList(image_1_path, image_1_file_path);

        for (const auto &gray_image : image_1_file_path) {
          if (extension(gray_image) == ".png" && boost::filesystem::exists(gray_image)) {
            image_filenames_[1].emplace_back(gray_image);
          }
        }
        image_filenames_[1].shrink_to_fit();
        std::sort(image_filenames_[1].begin(), image_filenames_[1].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });

        std::vector<std::string> image_2_file_path;
        getDirectoryList(image_2_path, image_2_file_path);
        for (const auto &rgb_image : image_2_file_path) {
          if (extension(rgb_image) == ".png" && boost::filesystem::exists(rgb_image)) {
            image_filenames_[2].emplace_back(rgb_image);
          }
        }
        image_filenames_[2].shrink_to_fit();
        std::sort(image_filenames_[2].begin(), image_filenames_[2].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });

        std::vector<std::string> image_3_file_path;
        getDirectoryList(image_3_path, image_3_file_path);

        for (const auto &rgb_image : image_3_file_path) {
          if (extension(rgb_image) == ".png" && boost::filesystem::exists(rgb_image)) {
            image_filenames_[3].emplace_back(rgb_image);
          }
        }
        image_filenames_[3].shrink_to_fit();
        std::sort(image_filenames_[3].begin(), image_filenames_[3].end(),
                  [&](const std::string &a, const std::string &b) -> bool {
                    return std::stoi(a.substr(a.find_last_of('.') - 6, 6)) <
                           std::stoi(b.substr(b.find_last_of('.') - 6, 6));
                  });
      } else {
        ROS_INFO("error image kind size, kitti only four kind image");
        return false;
      }

      // for calib.txt lidar to camera
      std::string calibPath = path + "/sequences/" + seq_num_ + "/calib.txt";
      if (boost::filesystem::exists(calibPath)) {
        std::ifstream in(calibPath.c_str());
        std::string s, line;
        while (in.good()) {
          std::getline(in, line);
          if (line[0] == 'T') {
            std::string pose = line.substr(4);
            std::stringstream pose_calib(pose);
            for (std::size_t i = 0; i < 3; ++i) {
              for (std::size_t j = 0; j < 4; ++j) {
                std::getline(pose_calib, s, ' ');
                T_camera_laser_(i, j) = stof(s);
              }
            }
          }
        }
      }

      // for gnss ground truth
      std::string gnss_path = path + "/sequences/" + seq_num_ + "/" + seq_num_ + ".txt";
      if (boost::filesystem::exists(gnss_path)) {
        readPose(gnss_path, ground_truth_);
      }
    }
  }

  return true;
}

bool KittiReader::readScan(std::shared_ptr<open3d::geometry::PointCloud2> &cloud_in, size_t index) {

  if (!boost::filesystem::exists(scan_filenames_[index]) || index > scan_filenames_.size()) {
    return false;
  }
  cloud_in = readVelodyneToO3d<open3d::geometry::PointCloud2>(scan_filenames_[index]);
  return true;
}

bool KittiReader::readImageGray(std::shared_ptr<cv::Mat> &image_, size_t kind_index, size_t index) {
  if (image_filenames_[kind_index].empty() || index > image_filenames_[kind_index].size() ||
      !boost::filesystem::exists(image_filenames_[kind_index][index])) {
    return false;
  }

  *image_ = cv::imread(image_filenames_[kind_index][index], cv::IMREAD_GRAYSCALE);
  return true;
}

bool KittiReader::readImageColor(std::shared_ptr<cv::Mat> &image_, size_t kind_index, size_t index) {
  if (image_filenames_[kind_index].empty() || index > image_filenames_[kind_index].size() ||
      !boost::filesystem::exists(image_filenames_[kind_index][index])) {
    return false;
  }

  *image_ = cv::imread(image_filenames_[kind_index][index], cv::IMREAD_COLOR);
  return true;
}

bool KittiReader::readPose(const std::string &file_name,
                           std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &pose) {
  std::ifstream fp(file_name.c_str());
  std::string line;
  if (!fp.is_open())
    return false;
  fp.peek();
  while (fp.good()) {
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();

    std::getline(fp, line);
    std::vector<std::string> entries;
    split(entries, line);
    if (entries.size() < 12) {
      fp.peek();
      continue;
    }
    for (uint32_t i = 0; i < 12; ++i) {
      std::string no_white_blank;
      KittiReader::trim(no_white_blank, entries[i]);
      P(i / 4, i - int(i / 4) * 4) = boost::lexical_cast<double>(no_white_blank);
    }
    pose.emplace_back(P);
    fp.peek();
  }
  fp.close();
  pose.shrink_to_fit();
  return true;
}

bool KittiReader::getDirectoryList(const std::string &dir, std::vector<std::string> &dir_list) {
  if (!boost::filesystem::is_directory(dir)){
      ROS_ERROR("The path %s don't have directory", dir.c_str());
      return false;
  }
  using diter = boost::filesystem::directory_iterator;
  for (diter it = diter(dir); it != diter(); ++it){
      dir_list.emplace_back(it->path().string());
  }

  dir_list.shrink_to_fit();
  return true;
}

void KittiReader::split(std::vector<std::string> &content, const std::string &line, const std::string &delimiter,
                        bool skipEmpty) {
  if (!content.empty()) {
    content.clear();
  }
  boost::char_separator<char> separator(delimiter.c_str(), "",
                                        (skipEmpty ? boost::drop_empty_tokens : boost::keep_empty_tokens));
  boost::tokenizer<boost::char_separator<char>> tokenizer(line, separator);
  for (auto it = tokenizer.begin(); it != tokenizer.end(); ++it) {
    content.emplace_back(*it);
  }
}

void KittiReader::trim(std::string &str_trim, const std::string &str, const std::string &whitespaces) {
  size_t begin, end;

  // find the beginning
  for (begin = 0; begin < static_cast<size_t>(str.size()); begin++) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[begin] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  // find the end
  for (end = static_cast<size_t>(str.size()) - 1; end > begin; --end) {
    bool found = false;
    for (char whitespace : whitespaces) {
      if (str[end] == whitespace) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }
  str_trim = str.substr(begin, end - begin + 1);
}

std::string KittiReader::extension(const std::string &path, size_t level) {
  std::string filename = boost::filesystem::path(path).filename().string();
  if (filename.empty() || filename == "." || filename == "..")
    return "";
  std::string ext;
  while (level-- > 0) {
    std::string::size_type index = filename.rfind('.');
    ext.insert(0, filename.substr(index));
    filename.resize(index);
  }

  return ext;
}

}