/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 上午10:53
 * @FileName: key_frame_publisher.cpp
 * @Description: Publish key frames
 * @License: See LICENSE for the license information
 */

#include "tloam/publisher/key_frame_publisher.hpp"

namespace tloam {
KeyFramePublisher::KeyFramePublisher(ros::NodeHandle &nh, std::string &topic_name, std::string &frame_id,
                                     int buff_size) {
  publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
}

void KeyFramePublisher::Publish(KeyFrame &key_frame) {
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;

  ros::Time ros_time((float) key_frame.time);
  pose_stamped.header.stamp = ros_time;
  pose_stamped.header.frame_id = frame_id_;

  pose_stamped.pose.pose.position.x = key_frame.pose(0, 3);
  pose_stamped.pose.pose.position.y = key_frame.pose(1, 3);
  pose_stamped.pose.pose.position.z = key_frame.pose(2, 3);

  Eigen::Quaterniond q = key_frame.GetQuaternion();
  pose_stamped.pose.pose.orientation.x = q.x();
  pose_stamped.pose.pose.orientation.y = q.y();
  pose_stamped.pose.pose.orientation.z = q.z();
  pose_stamped.pose.pose.orientation.w = q.w();

  pose_stamped.pose.covariance[0] = static_cast<double>(key_frame.index);

  publisher_.publish(pose_stamped);
}

bool KeyFramePublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

}


