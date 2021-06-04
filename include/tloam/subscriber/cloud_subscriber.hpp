/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午2:31
 * @FileName: cloud_subscriber.hpp
 * @Description: subscribe point cloud of specified topic
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_CLOUD_SUBSCRIBER_HPP
#define TLOAM_CLOUD_SUBSCRIBER_HPP

#include <deque>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "tloam/open3d/PointCloud2.hpp"
#include "tloam/open3d/open3d_to_ros.hpp"
#include "tloam/models/utils/sensor_data.hpp"

namespace tloam{
class CloudSubscriber{
public:
    CloudSubscriber() = default;
    CloudSubscriber(ros::NodeHandle& nh, std::string& topic_name, size_t buff_size);
    CloudSubscriber(ros::NodeHandle& nh, const char* topic_name, size_t buff_size);
    void ParseData(std::deque<CloudData>& cloud_data_buff);

private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

    std::mutex buff_mutex_;
};
}


#endif //TLOAM_CLOUD_SUBSCRIBER_HPP
