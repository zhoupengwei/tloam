/**
 * @Copyright 2021, Wuhan University of Technology
 * @Author: Pengwei Zhou
 * @Date: 2021/2/10 上午8:40
 * @FileName: utils.hpp.h
 * @Description: ${DESCRIPTION}
 * @License: See LICENSE for the license information
 */
#ifndef TLOAM_UTILS_HPP
#define TLOAM_UTILS_HPP

#include <iostream>
#include <chrono>

namespace tloam{
class Timer{
public:
  Timer() : begin(std::chrono::high_resolution_clock::now()) {}
  void reset() {
    begin = std::chrono::high_resolution_clock::now();
  }
  // output milliseconds
  inline int64_t elapsed() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::high_resolution_clock::now() - begin).count();
  }
  // output microseconds
  inline int64_t elapsed_microseconds() const{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - begin).count();
  }
  // output seconds
  inline int64_t elapsed_seconds() const {
    return std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::high_resolution_clock::now() - begin).count();
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> begin;
};
}


#endif //TLOAM_UTILS_HPP
