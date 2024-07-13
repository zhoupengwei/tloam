cmake_minimum_required(VERSION 3.0.2)
project(tloam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(cmake/work_space_path.cmake)
include(cmake/googletest.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  rosbag
  sensor_msgs
  tf
  velodyne_msgs
  image_transport
  cv_bridge
  nodelet
  ecl_threads
  jsk_recognition_msgs
  roscpp rospy angles std_srvs geometry_msgs sensor_msgs nav_msgs urdf tf dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem thread serialization iostreams)
find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED QUIET)
find_package(Open3D REQUIRED QUIET)
find_package(Threads REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)

# Find OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

add_service_files(
  FILES
  saveMap.srv
  saveOdometry.srv
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs velodyne_msgs image_transport jsk_recognition_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src/
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${Open3D_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  yaml-cpp
)

set(ALL_TARGET_LIBRARIES
  ${catkin_LIBRARIES}
  ${Open3D_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  yaml-cpp
)

add_library(${PROJECT_NAME}_open3d_conversions
  SHARED
  src/open3d/open3d_to_ros.cpp
  src/open3d/PointCloud2.cpp
)

add_library(${PROJECT_NAME}_publisher
  SHARED
  src/publisher/boundingbox_publisher.cpp
  src/publisher/cloud_publisher.cpp
  src/publisher/image_publisher.cpp
  src/publisher/key_frame_publisher.cpp
  src/publisher/key_frames_publisher.cpp
  src/publisher/odometry_publisher.cpp
  src/publisher/tf_broadcaster.cpp
)

add_library(${PROJECT_NAME}_subscriber
  SHARED
  src/subscriber/cloud_subscriber.cpp
  src/subscriber/key_frame_subscriber.cpp
  src/subscriber/key_frames_subscriber.cpp
  src/subscriber/odometry_subscriber.cpp
  src/subscriber/tf_listener.cpp
)

list(APPEND ALL_TARGET_LIBRARIES ${PROJECT_NAME}_open3d_conversions)
list(APPEND ALL_TARGET_LIBRARIES ${PROJECT_NAME}_publisher)
list(APPEND ALL_TARGET_LIBRARIES ${PROJECT_NAME}_subscriber)


add_library(
  kitti_reader_nodelet
  src/core_node/kitti_reader_nodelet.cpp
  src/models/io/kitti_reader.cpp
)
add_dependencies(kitti_reader_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitti_reader_nodelet ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_library(
  segmentation_nodelet
  src/core_node/segmentation_nodelet.cpp
  src/models/segmentation/segmentation.cpp
)
add_dependencies(segmentation_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentation_nodelet ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_library(
  lidar_odometry_nodelet
  src/core_node/lidar_odometry_nodelet.cpp
  src/models/registration/registration.cpp
  src/models/feature_extraction/feature_extract.cpp
  src/front_end/front_end.cpp
)
add_dependencies(lidar_odometry_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_odometry_nodelet ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#############
## Install ##
#############

install(FILES
  launch/tloam_kitti.launch
  rviz/car/ex1.jpeg
  rviz/car/ex2.jpeg
  rviz/car/model.dae
  rviz/car/default.urdf
  rviz/tloam.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)
