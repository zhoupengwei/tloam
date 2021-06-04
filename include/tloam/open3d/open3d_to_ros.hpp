/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/27 下午4:03
 * @FileName: open3d_to_ros.hpp
 * @Description: This is a tool to complete the conversion of ROS message and Open3D
 * @License: See LICENSE for the license information
 */

#pragma once

#include <string>
#include <unordered_map>
#include <omp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <open3d/Open3D.h>
#include "tloam/open3d/PointCloud2.hpp"

namespace open3d {
namespace conversions {

/**
* @brief Converting Open3D basic PointCloud to ros sensor_msgs format
* @param o3d_pc Basic types of Open3D point cloud Library
* @param ros_msgs ros message
* @param frame_id Reference frame default value set to /velodyne
* @return void
*/
void Open3dToRos(const open3d::geometry::PointCloud &o3d_pc, sensor_msgs::PointCloud2 &ros_msgs,
                 const std::string &frame_id = "/velodyne");

/**
* @brief Converting Open3D extensional PointCloud2 to ros sensor_msgs format
* @param o3d_pc2 Extensional types of Open3D point cloud Library (including intensity channel)
* @param ros_msgs ros message
* @param frame_id Reference frame default value set to /velodyne
* @return void
*/
void Open3dToRos(const open3d::geometry::PointCloud2 &o3d_pc2, sensor_msgs::PointCloud2 &ros_msgs,
                 const std::string &frame_id = "/velodyne");

/**
* @brief Converting ros sensor_msgs message to Open3D PointCloud format
* @param ros_msgs ros message
* @param o3d_pc Basic types of Open3D point cloud Library
* @param skip_colors if true, only xyz fileds will be copied, false otherwise
* @return void
*/
void RosToOpen3d(const sensor_msgs::PointCloud2 &ros_msgs, open3d::geometry::PointCloud &o3d_pc,
                 bool skip_colors = false);

/**
* @brief Converting ros sensor_msgs message to Open3D PointCloud2 format
* @param ros_msgs ros message
* @param o3d_pc Extensional types of Open3D point cloud Library (including intensity channel)
* @param skip_colors if true, only xyz and intensity fileds will be copied, false otherwise
* @return void
*/
void RosToOpen3d(const sensor_msgs::PointCloud2 &ros_msgs, open3d::geometry::PointCloud2 &o3d_pc2,
                 bool skip_colors = false);
} // namespace
} // namespace