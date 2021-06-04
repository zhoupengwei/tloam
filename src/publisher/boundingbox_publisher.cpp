/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午11:15
 * @FileName: boundingbox_publisher.cpp
 * @Description: publish different categories of objects boundingbox after DCVC segmentation
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/boundingbox_publisher.hpp"

namespace tloam {

BoundingBoxPublisher::BoundingBoxPublisher(ros::NodeHandle &nh, const std::string &topic_name,
                                           const std::string &frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name, buff_size);
}

void BoundingBoxPublisher::Publisher(const std::vector<jsk_recognition_msgs::BoundingBox> &objBoxList) {
  ros::Time time = ros::Time::now();
  PublishData(objBoxList, time);
}

void BoundingBoxPublisher::Publisher(const std::vector<jsk_recognition_msgs::BoundingBox> &objBoxList, double time) {
  ros::Time ros_time(static_cast<float>(time));
  PublishData(objBoxList, ros_time);
}

void BoundingBoxPublisher::PublishData(const std::vector<jsk_recognition_msgs::BoundingBox> &objBoxList,
                                       ros::Time time) {
  jsk_recognition_msgs::BoundingBoxArray boxArray;
  for (auto &box : objBoxList) {
    boxArray.boxes.emplace_back(box);
  }
  boxArray.header.stamp = time;
  boxArray.header.frame_id = frame_id_;

  publisher_.publish(boxArray);
}

bool BoundingBoxPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
}

