/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午3:02
 * @FileName: key_frames_subscriber.hpp
 * @Description: subscribe key-frames of specified topic
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_KEY_FRAMES_SUBSCRIBER_HPP
#define TLOAM_KEY_FRAMES_SUBSCRIBER_HPP

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "tloam/models/utils/sensor_data.hpp"

namespace tloam{
class KeyFramesSubscriber{
public:
    KeyFramesSubscriber() = default;
    KeyFramesSubscriber(ros::NodeHandle& nh, std::string& topic_name, size_t buff_size);

    void ParseData(std::deque<KeyFrame>& key_frames_buff);

private:
    void msg_callback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<KeyFrame> new_key_frames_;

    std::mutex buff_mutex_;
};
}


#endif //TLOAM_KEY_FRAMES_SUBSCRIBER_HPP
