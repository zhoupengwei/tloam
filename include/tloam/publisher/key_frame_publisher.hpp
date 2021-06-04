/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午10:32
 * @FileName: key_frame_publisher.hpp
 * @Description: Publish key frames
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_KEY_FRAME_PUBLISHER_HPP
#define TLOAM_KEY_FRAME_PUBLISHER_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tloam/models/utils/sensor_data.hpp"

namespace tloam {
class KeyFramePublisher {
public:
  KeyFramePublisher() = default;

  KeyFramePublisher(ros::NodeHandle &nh, std::string &topic_name, std::string &frame_id, int buff_size);

  void Publish(KeyFrame &key_frame);

  bool HasSubscribers();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};
}


#endif //TLOAM_KEY_FRAME_PUBLISHER_HPP
