/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午2:46
 * @FileName: key_frame_subscriber.hpp
 * @Description: subscribe key-frame of specified topic
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_KEY_FRAME_SUBSCRIBER_HPP
#define TLOAM_KEY_FRAME_SUBSCRIBER_HPP

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tloam/models/utils/sensor_data.hpp"

namespace tloam{
class KeyFrameSubscriber{
public:
    KeyFrameSubscriber() = default;
    KeyFrameSubscriber(ros::NodeHandle& nh, std::string& topic_name, size_t buff_size);

    void ParseData(std::deque<KeyFrame>& key_frame_buff);

private:
    void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msg_ptr);
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<KeyFrame> new_key_frame_;

    std::mutex buff_mutex_;
};
}

#endif //TLOAM_KEY_FRAME_SUBSCRIBER_HPP
