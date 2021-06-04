/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午11:15
 * @FileName: boundingbox_publisher.hpp
 * @Description: publish different categories of objects boundingbox after DCVC segmentation
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_BOUNDINGBOX_PUBLISHER_HPP
#define TLOAM_BOUNDINGBOX_PUBLISHER_HPP

#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace tloam {
class BoundingBoxPublisher {
public:
  BoundingBoxPublisher() = default;

  void Publisher(const std::vector<jsk_recognition_msgs::BoundingBox> &objBoxList, double time);

  void Publisher(const std::vector<jsk_recognition_msgs::BoundingBox> &objBoxList);

  bool HasSubscribers();

  BoundingBoxPublisher(ros::NodeHandle &nh,
                       const std::string &topic_name,
                       const std::string &frame_id,
                       size_t buff_size);

private:
  void PublishData(const std::vector<jsk_recognition_msgs::BoundingBox> &objBoxList, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

}


#endif //TLOAM_BOUNDINGBOX_PUBLISHER_HPP
