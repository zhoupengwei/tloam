/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午11:03
 * @FileName: key_frames_publisher.hpp
 * @Description: publish key frames
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_KEY_FRAMES_PUBLISHER_HPP
#define TLOAM_KEY_FRAMES_PUBLISHER_HPP

#include <string>
#include <deque>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "tloam/models/utils/sensor_data.hpp"

namespace tloam {
class KeyFramesPublisher {
public:
  KeyFramesPublisher(ros::NodeHandle &nh,
                     std::string &topic_name,
                     std::string &frame_id,
                     int buff_size);

  KeyFramesPublisher() = default;

  void Publish(const std::deque<KeyFrame> &key_frames);

  bool HasSubscribers();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};
}


#endif //TLOAM_KEY_FRAMES_PUBLISHER_HPP
