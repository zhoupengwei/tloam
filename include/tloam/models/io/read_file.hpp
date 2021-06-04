/**
 * @Copyright 2020, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2020/12/28 下午3:53
 * @FileName: read_file.hpp
 * @Description: read point cloud from .txt .dat .bin .pcd
 * @License: See LICENSE for the license information
 */

#ifndef TLOAM_READ_FILE_HPP
#define TLOAM_READ_FILE_HPP

#include <fstream>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <memory>
#include <ros/ros.h>

namespace tloam {
class Point3f {
public:
  Point3f() : vec_(0.0f, 0.0f, 0.0f) {}

  Point3f(float x, float y, float z) : vec_(x, y, z) {
    assert(!HasNan());
  }

  Point3f(const Point3f &p) : vec_(p.vec_[0], p.vec_[1], p.vec_[2]) {
    assert(!p.HasNan());
  }

  static bool HasNan(float a, float b, float c) {
    return std::isnan(a) || std::isnan(b) || std::isnan(c);
  }

  bool HasNan() const {
    return std::isnan(vec_[0]) || std::isnan(vec_[1]) || std::isnan(vec_[2]);
  }

  float operator[](size_t i) const {
    assert(i >= 0 && i <= 2);
    return vec_[i];
  }

  float &operator[](size_t i) {
    assert(i >= 0 && i <= 2);
    return vec_[i];
  }

  inline const float &x() const {
    return vec_[0];
  }

  inline float &x() {
    return vec_[0];
  }

  inline const float &y() const {
    return vec_[1];
  }

  inline float &y() {
    return vec_[1];
  }

  inline const float &z() const {
    return vec_[2];
  }

  inline float &z() {
    return vec_[2];
  }

  friend std::ostream &operator<<(std::ostream &out, const Point3f &p) {
    out.width(6);
    out.precision(4);
    out << p.vec_[0] << ", " << p.vec_[1] << " , " << p.vec_[2] << std::endl;
    return out;
  }

  Eigen::Vector3f vec_;
};

class Point4f {
public:
  Point4f() : vec_(0.f, 0.f, 0.f, 1.f) {}

  Point4f(float x, float y, float z, float intensity) : vec_(x, y, z, intensity) {
    assert(!HasNaNs());
  }

  Point4f(const Point4f &p) : vec_(p.vec_[0], p.vec_[1], p.vec_[2], p.vec_[3]) {
    assert(!p.HasNaNs());
  }

  float operator[](int i) const {
    assert(i >= 0 && i <= 3);
    return vec_[i];
  }

  float &operator[](int i) {
    assert(i >= 0 && i <= 3);
    return vec_[i];
  }

  inline const float &x() const {
    return vec_[0];
  }

  inline float &x() {
    return vec_[0];
  }

  inline const float &y() const {
    return vec_[1];
  }

  inline float &y() {
    return vec_[1];
  }

  inline const float &z() const {
    return vec_[2];
  }

  inline float &z() {
    return vec_[2];
  }

  inline const float &intensity() const {
    return vec_[3];
  }

  inline float &intensity() {
    return vec_[3];
  }

  static bool HasNaNs(float a, float b, float c, float d) {
    return std::isnan(a) || std::isnan(b) || std::isnan(c) || std::isnan(d);
  }

  bool HasNaNs() const {
    return std::isnan(vec_[0]) || std::isnan(vec_[1]) || std::isnan(vec_[2]) || std::isnan(vec_[3]);
  }

  friend std::ostream &operator<<(std::ostream &out, const Point4f &p) {
    out.width(4);
    out.precision(3);
    out << p.vec_[0] << ", " << p.vec_[1] << ", " << p.vec_[2] << ", " << p.vec_[3] << std::endl;
    return out;
  }

  Eigen::Vector4f vec_;
};

class Point6f {
public:
  Point6f() {
    vec_.setZero();
  }

  Point6f(float x, float y, float z, float n_x, float n_y, float n_z) {
    vec_ << x, y, z, n_x, n_y, n_z;
    assert(!HasNaNs());
  }

  static bool HasNaNs(float a, float b, float c, float d, float e, float f) {
    return std::isnan(a) || std::isnan(b) || std::isnan(c) || std::isnan(d) || std::isnan(e) || std::isnan(f);
  }

  bool HasNaNs() const {
    return std::isnan(vec_[0]) || std::isnan(vec_[1]) || std::isnan(vec_[2]) || std::isnan(vec_[3]) ||
           std::isnan(vec_[4]) || std::isnan(vec_[5]);
  }

  float operator[](int i) const {
    assert(i >= 0 && i <= 5);
    return vec_[i];
  }

  float &operator[](int i) {
    assert(i >= 0 && i <= 5);
    return vec_[i];
  }

  inline const float &x() const {
    return vec_[0];
  }

  inline float &x() {
    return vec_[0];
  }

  inline const float &y() const {
    return vec_[1];
  }

  inline float &y() {
    return vec_[1];
  }

  inline const float &z() const {
    return vec_[2];
  }

  inline float &z() {
    return vec_[2];
  }

  inline const float &n_x() const {
    return vec_[3];
  }

  inline float &n_x() {
    return vec_[3];
  }

  inline const float &n_y() const {
    return vec_[4];
  }

  inline float &n_y() {
    return vec_[4];
  }

  inline const float &n_z() const {
    return vec_[5];
  }

  inline float &n_z() {
    return vec_[5];
  }

  friend std::ostream &operator<<(std::ostream &out, const Point6f &p) {
    out.width(4);
    out.precision(3);
    out << p.vec_[0] << ", " << p.vec_[1] << ", " << p.vec_[2] << ", " << p.vec_[3] << ", " << p.vec_[4] << ", "
        << p.vec_[5] << std::endl;
    return out;
  }

  Eigen::Matrix<float, 6, 1> vec_;
};

template<typename PointT, typename ContainerT>
void readPointsDat(const std::string &filename, ContainerT &points, int row_length) {
  std::ifstream in(filename.c_str());
  std::string line;
  boost::char_separator<char> separator(" ");
  // read .dat format file.
  // the first three dimensions represent normal vectors, and the second three dimensions represent coordinate information
  while (!in.eof()) {
    std::getline(in, line);
    in.peek();
    boost::tokenizer<boost::char_separator<char>> tokenIterator(line, separator);
    std::vector<std::string> tokens(tokenIterator.begin(), tokenIterator.end());
    if (tokens.size() != row_length)
      continue;
    auto x = boost::lexical_cast<float>(tokens[3]);
    auto y = boost::lexical_cast<float>(tokens[4]);
    auto z = boost::lexical_cast<float>(tokens[5]);
    points.emplace_back(PointT(x, y, z));
  }
  in.close();
}

template<typename PointT, typename ContainerT>
void readPointsText(const std::string &filename, ContainerT &points, int row_length) {
  std::ifstream in(filename.c_str());
  std::string line;
  boost::char_separator<char> separator(" ");
  while (!in.eof()) {
    std::getline(in, line);
    in.peek();
    boost::tokenizer<boost::char_separator<char>> tokenizer(line, separator);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
    if (tokens.size() != row_length)
      continue;
    auto x = boost::lexical_cast<float>(tokens[0]);
    auto y = boost::lexical_cast<float>(tokens[1]);
    auto z = boost::lexical_cast<float>(tokens[2]);

    points.emplace_back(PointT(x, y, z));
  }
  in.close();
}

template<typename PointT, typename ContainerT>
void readPoints(const std::string &filename, ContainerT &points, int row_length) {
  std::string surfix = filename.substr(filename.size() - 3, filename.size());
  if (std::strcmp(surfix.c_str(), "dat") == 0) {
    readPointsDat<PointT>(filename, points, row_length);
    return;
  }
  if (std::strcmp(surfix.c_str(), "txt") == 0) {
    readPointsText<PointT>(filename, points, row_length);
    return;
  }
  std::cerr << "wrong file format !!! " << std::endl;
  exit(EXIT_FAILURE);
}


template<typename T>
typename std::shared_ptr<T> readVelodyneToO3d(const std::string &filename) {
  std::fstream readFile(filename.c_str(), std::ios::in | std::ios::binary);
  if (!readFile.good()) {
    std::cerr << "Cloud not file..." << std::endl;
    exit(EXIT_FAILURE);
  }
  std::shared_ptr<T> pointCloud(new T);
  readFile.seekg(0, std::ios::beg);
  for (size_t i = 0; readFile.good() && !readFile.eof(); i++) {
    Point4f point;
    readFile.read((char *) &point.x(), 3 * sizeof(float));
    readFile.read((char *) &point.intensity(), sizeof(float));
    if (!point.HasNaNs()) {
      pointCloud->points_.emplace_back(Eigen::Vector3d(point.x(), point.y(), point.z()));
      pointCloud->intensity_.emplace_back(static_cast<double>(point.intensity()));
    }
  }
  readFile.close();
  return pointCloud;
}

}

#endif //TLOAM_READ_FILE_HPP
