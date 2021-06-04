/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午11:30
 * @FileName: image_publisher.cpp
 * @Description: Publish gray or color images
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/image_publisher.hpp"

namespace tloam {
ImagePublisher::ImagePublisher(ros::NodeHandle &nh, const std::string &topic_name, const std::string &frame_id,
                               size_t buff_size, bool color) : transport_(nh), frame_id_(frame_id), color_(color) {
  publisher_ = transport_.advertise(topic_name, buff_size);
}

void ImagePublisher::Publish(std::shared_ptr<cv::Mat> &image_ptr) {
  ros::Time time = ros::Time::now();
  PublishData(image_ptr, time);
}

void ImagePublisher::Publish(std::shared_ptr<cv::Mat> &image_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(image_ptr, ros_time);
}

void ImagePublisher::PublishData(std::shared_ptr<cv::Mat> &image_ptr, ros::Time time) {
  std_msgs::Header imageHeader;
  imageHeader.frame_id = frame_id_;
  imageHeader.stamp = time;
  if (!color_) {
    sensor_msgs::ImagePtr image_msg_ptr = cv_bridge::CvImage(imageHeader, "mono8", *image_ptr).toImageMsg();
    publisher_.publish(image_msg_ptr);
  } else {
    sensor_msgs::ImagePtr image_msg_ptr = cv_bridge::CvImage(imageHeader, "bgr8", *image_ptr).toImageMsg();
    publisher_.publish(image_msg_ptr);
  }
}

void ImagePublisher::SetColor(bool color) {
  color_ = color;
}

bool ImagePublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

}


