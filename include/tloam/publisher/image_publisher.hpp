/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午11:29
 * @FileName: image_publisher.hpp
 * @Description: Publish gray or color images
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_IMAGE_PUBLISHER_HPP
#define TLOAM_IMAGE_PUBLISHER_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace tloam {
class ImagePublisher {
public:
  ImagePublisher() = default;

  /**
   * @brief Constructor
   * @param nh  ros NodeHandle
   * @param topic_name message topic
   * @param frame_id message frame
   * @param buff_size length of buff
   * @param color if false publish gray image, true publish color image
   */
  ImagePublisher(ros::NodeHandle &nh, const std::string &topic_name, const std::string &frame_id, size_t buff_size,
                 bool color = false);

  void Publish(std::shared_ptr<cv::Mat> &image_ptr, double time);

  void Publish(std::shared_ptr<cv::Mat> &image_ptr);

  bool HasSubscribers();

  void SetColor(bool color);

private:
  void PublishData(std::shared_ptr<cv::Mat> &image_ptr, ros::Time time);

private:
  image_transport::ImageTransport transport_;
  image_transport::Publisher publisher_;
  std::string frame_id_;
  bool color_;
};
}
#endif //TLOAM_IMAGE_PUBLISHER_HPP
