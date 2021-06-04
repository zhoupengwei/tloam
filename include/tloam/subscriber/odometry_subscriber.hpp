/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午3:19
 * @FileName: odometry_subscriber.hpp
 * @Description: Subscribe odometry of specified topic
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_ODOMETRY_SUBSCRIBER_HPP
#define TLOAM_ODOMETRY_SUBSCRIBER_HPP

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "tloam/models/utils/sensor_data.hpp"

namespace tloam{
class OdometrySubscriber{
public:
    OdometrySubscriber() = default;
    OdometrySubscriber(ros::NodeHandle& nh, std::string& topic_name, size_t buff_size);

    void ParseData(std::deque<PoseData>& pose_data_buff);

private:
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;;
    std::deque<PoseData> new_pose_data_;

    std::mutex buff_mutex_;
};
}

#endif //TLOAM_ODOMETRY_SUBSCRIBER_HPP
