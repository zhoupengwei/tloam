/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/27 下午4:03
 * @FileName: open3d_to_ros.cpp
 * @Description: ${DESCRIPTION}
 * @License: See LICENSE for the license information
 */

#include "tloam/open3d/open3d_to_ros.hpp"

namespace open3d {
namespace conversions {

void Open3dToRos(const open3d::geometry::PointCloud &o3d_pc, sensor_msgs::PointCloud2 &ros_msgs,
                 const std::string &frame_id) {
    if (o3d_pc.points_.empty()) {
        ROS_ERROR("Unable to convert empty point cloud data.");
        return;
    }

    bool color_flag = o3d_pc.HasColors();
    sensor_msgs::PointCloud2Modifier modifier(ros_msgs);
    if (color_flag) {
        modifier.setPointCloud2Fields(6,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "r", 1, sensor_msgs::PointField::UINT16,
                                      "g", 1, sensor_msgs::PointField::UINT16,
                                      "b", 1, sensor_msgs::PointField::UINT16);
    } else {
        modifier.setPointCloud2Fields(3,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32);
    }

    modifier.resize(o3d_pc.points_.size());
    ros_msgs.header.frame_id = frame_id;
    ros_msgs.header.stamp = ros::Time::now();
    ros_msgs.width = o3d_pc.points_.size();
    ros_msgs.height = 1;

    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_msgs, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_msgs, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_msgs, "z");

    if (color_flag) {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_msgs, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_msgs, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_msgs, "b");
        for (size_t i = 0, Size = o3d_pc.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
            const Eigen::Vector3d &pt = o3d_pc.points_[i];
            const Eigen::Vector3d &color = o3d_pc.colors_[i];
            *ros_pc2_x = (float) pt(0);
            *ros_pc2_y = (float) pt(1);
            *ros_pc2_z = (float) pt(2);
            *ros_pc2_r = static_cast<int>(255 * color(0));
            *ros_pc2_g = static_cast<int>(255 * color(1));
            *ros_pc2_b = static_cast<int>(255 * color(2));
        }
    } else {
        for (size_t i = 0, Size = o3d_pc.points_.size(); i < Size; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
            const Eigen::Vector3d &pt = o3d_pc.points_[i];
            *ros_pc2_x = (float) pt.x();
            *ros_pc2_y = (float) pt.y();
            *ros_pc2_z = (float) pt.z();
        }
    }
}

void Open3dToRos(const open3d::geometry::PointCloud2 &o3d_pc2, sensor_msgs::PointCloud2 &ros_msgs,
                 const std::string &frame_id) {
    if (o3d_pc2.points_.empty()) {
        ROS_ERROR("Unable to convert empty point cloud data.");
        return;
    }
    sensor_msgs::PointCloud2Modifier modifier(ros_msgs);
    bool intensity_flag = o3d_pc2.HasIntensity();
    bool color_flag = o3d_pc2.HasColors();
    bool normal_flag = o3d_pc2.HasNormals();
    if (intensity_flag && !color_flag && !normal_flag) {
        /// x, y, z, intensity
        /// Note: you must use FLOAT32, FLOAT64 will not visualize pointcloud in rviz.
        modifier.setPointCloud2Fields(4,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);

    } else if (color_flag && !intensity_flag && !normal_flag) {
        /// x, y, z, r, g, b
        modifier.setPointCloud2Fields(6,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "r", 1, sensor_msgs::PointField::UINT16,
                                      "g", 1, sensor_msgs::PointField::UINT16,
                                      "b", 1, sensor_msgs::PointField::UINT16);
    } else if (normal_flag && !color_flag && !intensity_flag){
        /// x, y, z, normal_x, normal_y, normal_z
        modifier.setPointCloud2Fields(6,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_z", 1, sensor_msgs::PointField::FLOAT32);
    } else if (!normal_flag && color_flag && intensity_flag){   // 2
        /// x, y, z, r, g, b, intensity
        modifier.setPointCloud2Fields(7,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "r", 1, sensor_msgs::PointField::UINT16,
                                      "g", 1, sensor_msgs::PointField::UINT16,
                                      "b", 1, sensor_msgs::PointField::UINT16,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    } else if (!intensity_flag && color_flag && normal_flag) {
        /// x, y, z, r, g, b, normal_x, normal_y, normal_z
        modifier.setPointCloud2Fields(9,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "r", 1, sensor_msgs::PointField::UINT16,
                                      "g", 1, sensor_msgs::PointField::UINT16,
                                      "b", 1, sensor_msgs::PointField::UINT16,
                                      "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_z", 1, sensor_msgs::PointField::FLOAT32);
    } else if (!color_flag && normal_flag && intensity_flag) {
        /// x, y, z, normal_x, normal_y, normal_z, intensity
        modifier.setPointCloud2Fields(7,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    } else if (normal_flag && color_flag && intensity_flag){
        /// x, y, z, r, g, b, normal_x, normal_y, normal_z, intensity
        modifier.setPointCloud2Fields(10,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "r", 1, sensor_msgs::PointField::UINT16,
                                      "g", 1, sensor_msgs::PointField::UINT16,
                                      "b", 1, sensor_msgs::PointField::UINT16,
                                      "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                                      "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    } else {
        /// only x, y, z
        modifier.setPointCloud2Fields(3,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32);
    }

    modifier.resize(o3d_pc2.points_.size());
    ros_msgs.header.frame_id = frame_id;
    ros_msgs.header.stamp = ros::Time::now();
    ros_msgs.width = o3d_pc2.points_.size();
    ros_msgs.height = 1;

    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_msgs, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_msgs, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_msgs, "z");
    if (intensity_flag && !color_flag && !normal_flag) {
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_intensity(ros_msgs, "intensity");
        for (size_t i = 0, Size = o3d_pc2.points_.size();
             i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_intensity) {
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const double &it = o3d_pc2.intensity_[i];
            *ros_pc2_x = (float) pt.x();
            *ros_pc2_y = (float) pt.y();
            *ros_pc2_z = (float) pt.z();
            *ros_pc2_intensity = (float) it;
        }
    } else if (color_flag && !intensity_flag && !normal_flag) {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_msgs, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_msgs, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_msgs, "b");
        for (size_t i = 0, Size = o3d_pc2.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const Eigen::Vector3d &color = o3d_pc2.colors_[i];
            *ros_pc2_x = (float) pt(0);
            *ros_pc2_y = (float) pt(1);
            *ros_pc2_z = (float) pt(2);
            *ros_pc2_r = (int) (255 * color(0));
            *ros_pc2_g = (int) (255 * color(1));
            *ros_pc2_b = (int) (255 * color(2));
        }
    } else if (normal_flag && !color_flag && !intensity_flag){
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_z(ros_msgs, "normal_z");
        for (size_t i = 0, Size = o3d_pc2.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_normal_x, ++ros_pc2_normal_y, ++ros_pc2_normal_z){
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const Eigen::Vector3d &normal = o3d_pc2.normals_[i];
            *ros_pc2_x = (float) pt(0);
            *ros_pc2_y = (float) pt(1);
            *ros_pc2_z = (float) pt(2);
            *ros_pc2_normal_x = (float)normal(0);
            *ros_pc2_normal_y = (float)normal(1);
            *ros_pc2_normal_z = (float)normal(2);
        }
    } else if (!normal_flag && color_flag && intensity_flag){
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_msgs, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_msgs, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_msgs, "b");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_intensity(ros_msgs, "intensity");
        for (size_t i = 0, Size = o3d_pc2.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b, ++ros_pc2_intensity){
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const Eigen::Vector3d &color = o3d_pc2.colors_[i];
            const double &it = o3d_pc2.intensity_[i];
            *ros_pc2_x = (float) pt(0);
            *ros_pc2_y = (float) pt(1);
            *ros_pc2_z = (float) pt(2);
            *ros_pc2_r = (int) (255 * color(0));
            *ros_pc2_g = (int) (255 * color(1));
            *ros_pc2_b = (int) (255 * color(2));
            *ros_pc2_intensity = (float) (it);
        }
    } else if (!intensity_flag && color_flag && normal_flag){
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_msgs, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_msgs, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_msgs, "b");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_z(ros_msgs, "normal_z");
        for (size_t i = 0, Size = o3d_pc2.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b, ++ros_pc2_normal_x, ++ros_pc2_normal_y, ++ros_pc2_normal_z) {
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const Eigen::Vector3d &color = o3d_pc2.colors_[i];
            const Eigen::Vector3d &normal = o3d_pc2.normals_[i];
            *ros_pc2_x = (float) pt(0);
            *ros_pc2_y = (float) pt(1);
            *ros_pc2_z = (float) pt(2);
            *ros_pc2_r = (int) (255 * color(0));
            *ros_pc2_g = (int) (255 * color(1));
            *ros_pc2_b = (int) (255 * color(2));
            *ros_pc2_normal_x = (float)normal(0);
            *ros_pc2_normal_y = (float)normal(1);
            *ros_pc2_normal_z = (float)normal(2);
        }
    } else if (!color_flag && normal_flag && intensity_flag){
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_z(ros_msgs, "normal_z");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_intensity(ros_msgs, "intensity");
        for (size_t i = 0, Size = o3d_pc2.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_normal_x, ++ros_pc2_normal_y, ++ros_pc2_normal_z, ++ros_pc2_intensity){
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const Eigen::Vector3d &normal = o3d_pc2.normals_[i];
            const double& it = o3d_pc2.intensity_[i];
            *ros_pc2_x = (float) pt(0);
            *ros_pc2_y = (float) pt(1);
            *ros_pc2_z = (float) pt(2);
            *ros_pc2_normal_x = (float)normal(0);
            *ros_pc2_normal_y = (float)normal(1);
            *ros_pc2_normal_z = (float)normal(2);
            *ros_pc2_intensity = (float) (it);
        }
    }else if (normal_flag && color_flag && intensity_flag) {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_msgs, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_msgs, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_msgs, "b");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_normal_z(ros_msgs, "normal_z");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_intensity(ros_msgs, "intensity");
        for (size_t i = 0, Size = o3d_pc2.points_.size(); i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
                ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b, ++ros_pc2_normal_x, ++ros_pc2_normal_y, ++ros_pc2_normal_z, ++ros_pc2_intensity) {
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            const Eigen::Vector3d &color = o3d_pc2.colors_[i];
            const Eigen::Vector3d &normal = o3d_pc2.normals_[i];
            const double &it = o3d_pc2.intensity_[i];
            *ros_pc2_x = (float) pt.x();
            *ros_pc2_y = (float) pt.y();
            *ros_pc2_z = (float) pt.z();
            *ros_pc2_r = (int) (255 * color(0));
            *ros_pc2_g = (int) (255 * color(1));
            *ros_pc2_b = (int) (255 * color(2));
            *ros_pc2_normal_x = (float)normal(0);
            *ros_pc2_normal_y = (float)normal(1);
            *ros_pc2_normal_z = (float)normal(2);
            *ros_pc2_intensity = (float) it;
        }
    } else {
        for (size_t i = 0, Size = o3d_pc2.points_.size();
             i < Size; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
            const Eigen::Vector3d &pt = o3d_pc2.points_[i];
            *ros_pc2_x = (float) pt.x();
            *ros_pc2_y = (float) pt.y();
            *ros_pc2_z = (float) pt.z();
        }
    }
}

void RosToOpen3d(const sensor_msgs::PointCloud2 &ros_msgs, open3d::geometry::PointCloud &o3d_pc, bool skip_colors) {
    sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_x(ros_msgs, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_y(ros_msgs, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_z(ros_msgs, "z");

    size_t totalSize = ros_msgs.height * ros_msgs.width;
    if (totalSize == 0) {
        ROS_ERROR("RosToOpen3d has find this message don't have points.");
        return;
    }

    std::unordered_map<std::string, int> fields_name;
    for (size_t i = 0; i < ros_msgs.fields.size(); i++){
        fields_name[ros_msgs.fields[i].name] = i;
    }

    if (fields_name.find("r") != fields_name.end()){
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_r(ros_msgs, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_g(ros_msgs, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_b(ros_msgs, "b");

        o3d_pc.points_.resize(totalSize);
        o3d_pc.colors_.resize(totalSize);

        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_r, ++ros_msgs_g, ++ros_msgs_b){
            o3d_pc.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc.colors_[i] = Eigen::Vector3d((*ros_msgs_r)/255.0,  (*ros_msgs_g)/255.0, (*ros_msgs_b)/255.0);
        }
    } else {
        o3d_pc.points_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z){
            o3d_pc.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
        }
    }
}

void RosToOpen3d(const sensor_msgs::PointCloud2 &ros_msgs, open3d::geometry::PointCloud2 &o3d_pc2,
                 bool skip_colors) {

    sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_x(ros_msgs, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_y(ros_msgs, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_z(ros_msgs, "z");

    size_t totalSize = ros_msgs.height * ros_msgs.width;
    if (totalSize == 0) {
        ROS_ERROR("RosToOpen3d has find this message don't have points.");
        return;
    }

    std::unordered_map<std::string, int> fields_name;
    for (size_t i = 0; i < ros_msgs.fields.size(); i++){
        fields_name[ros_msgs.fields[i].name] = i;
    }
    // if find intensity information then save it x, y, z, r, g, b, intensity
    bool intensity_flag = fields_name.find("intensity") != fields_name.end();
    bool color_flag = fields_name.find("r") != fields_name.end();
    bool normal_flag = fields_name.find("normal_x") != fields_name.end();


    if (intensity_flag && !color_flag && !normal_flag){
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_intensity(ros_msgs, "intensity");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.intensity_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_intensity){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.intensity_[i] = *ros_msgs_intensity;
        }
    } else if (color_flag && !intensity_flag && !normal_flag) {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_r(ros_msgs, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_g(ros_msgs, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_b(ros_msgs, "b");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.colors_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_r, ++ros_msgs_g, ++ros_msgs_b){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.colors_[i] = Eigen::Vector3d((*ros_msgs_r)/255.0,  (*ros_msgs_g)/255.0, (*ros_msgs_b)/255.0);
        }
    } else if (normal_flag && !color_flag && !intensity_flag){
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_z(ros_msgs, "normal_z");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.normals_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_normal_x, ++ros_msgs_normal_y, ++ros_msgs_normal_z){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.normals_[i] = Eigen::Vector3d(*ros_msgs_normal_x, *ros_msgs_normal_y, *ros_msgs_normal_z);
        }
    } else if (!normal_flag && color_flag && intensity_flag){
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_r(ros_msgs, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_g(ros_msgs, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_b(ros_msgs, "b");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_intensity(ros_msgs, "intensity");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.colors_.resize(totalSize);
        o3d_pc2.intensity_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_r, ++ros_msgs_g, ++ros_msgs_b, ++ros_msgs_intensity){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.colors_[i] = Eigen::Vector3d((*ros_msgs_r)/255.0,  (*ros_msgs_g)/255.0, (*ros_msgs_b)/255.0);
            o3d_pc2.intensity_[i] = *ros_msgs_intensity;
        }
    } else if (!intensity_flag && color_flag && normal_flag){
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_r(ros_msgs, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_g(ros_msgs, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_b(ros_msgs, "b");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_z(ros_msgs, "normal_z");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.colors_.resize(totalSize);
        o3d_pc2.normals_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_r, ++ros_msgs_g, ++ros_msgs_b, ++ros_msgs_normal_x, ++ros_msgs_normal_y, ++ros_msgs_normal_z){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.colors_[i] = Eigen::Vector3d((*ros_msgs_r)/255.0,  (*ros_msgs_g)/255.0, (*ros_msgs_b)/255.0);
            o3d_pc2.normals_[i] = Eigen::Vector3d(*ros_msgs_normal_x, *ros_msgs_normal_y, *ros_msgs_normal_z);
        }
    } else if (!color_flag && normal_flag && intensity_flag){
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_z(ros_msgs, "normal_z");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_intensity(ros_msgs, "intensity");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.normals_.resize(totalSize);
        o3d_pc2.intensity_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_normal_x, ++ros_msgs_normal_y, ++ros_msgs_normal_z, ++ros_msgs_intensity){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.normals_[i] = Eigen::Vector3d(*ros_msgs_normal_x, *ros_msgs_normal_y, *ros_msgs_normal_z);
            o3d_pc2.intensity_[i] = *ros_msgs_intensity;
        }
    } else if (normal_flag && color_flag && intensity_flag){
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_r(ros_msgs, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_g(ros_msgs, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_msgs_b(ros_msgs, "b");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_x(ros_msgs, "normal_x");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_y(ros_msgs, "normal_y");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_normal_z(ros_msgs, "normal_z");
        sensor_msgs::PointCloud2ConstIterator<float> ros_msgs_intensity(ros_msgs, "intensity");
        o3d_pc2.points_.resize(totalSize);
        o3d_pc2.colors_.resize(totalSize);
        o3d_pc2.normals_.resize(totalSize);
        o3d_pc2.intensity_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z, ++ros_msgs_r, ++ros_msgs_g, ++ros_msgs_b,
                ++ros_msgs_normal_x, ++ros_msgs_normal_y, ++ros_msgs_normal_z, ++ros_msgs_intensity){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
            o3d_pc2.colors_[i] = Eigen::Vector3d((*ros_msgs_r)/255.0,  (*ros_msgs_g)/255.0, (*ros_msgs_b)/255.0);
            o3d_pc2.normals_[i] = Eigen::Vector3d(*ros_msgs_normal_x, *ros_msgs_normal_y, *ros_msgs_normal_z);
            o3d_pc2.intensity_[i] = *ros_msgs_intensity;
        }
    } else {  // x, y, z
        o3d_pc2.points_.resize(totalSize);
        for (size_t i = 0; i < totalSize; i++, ++ros_msgs_x, ++ros_msgs_y, ++ros_msgs_z){
            o3d_pc2.points_[i] = Eigen::Vector3d(*ros_msgs_x, *ros_msgs_y, *ros_msgs_z);
        }
    }
}

}  // namespace conversions
}  // namespace open3d

