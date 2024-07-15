FROM ros:melodic-ros-base-bionic

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget git build-essential vim sed \
    libc++1 \
    xvfb \
    python-catkin-tools \
    ros-melodic-ecl-threads \
    ros-melodic-jsk-recognition-msgs \
    ros-melodic-jsk-visualization \
    ros-melodic-velodyne-msgs \
    ros-melodic-joint-state-publisher \
    ros-melodic-robot-state-publisher \
    python3-pip \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev

RUN pip3 install catkin_pkg

RUN wget https://cmake.org/files/v3.22/cmake-3.22.6-linux-x86_64.tar.gz \
    && tar -xf cmake-3.22.6-linux-x86_64.tar.gz && rm cmake-3.22.6-linux-x86_64.tar.gz \
    && cd cmake-3.22.6-linux-x86_64/ && mv man share/ \
    && cp -r * /usr/local/

RUN wget https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz \
    && tar -xf open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz \
    && rm open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz \
    && cp -r open3d-devel-linux-x86_64-cxx11-abi-0.18.0/* /usr/local/

RUN ldconfig

RUN wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.3.tar.gz \
    && tar -zxvf yaml-cpp-0.6.3.tar.gz && rm yaml-cpp-0.6.3.tar.gz \
    && cd yaml-cpp-yaml-cpp-0.6.3 && mkdir build && cd build \
    && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DYAML_BUILD_SHARED_LIBS=ON .. \
    && make -j4 && make install

RUN wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz \
    && tar -xf ceres-solver-2.0.0.tar.gz \
    && rm ceres-solver-2.0.0.tar.gz \
    && cd ceres-solver-2.0.0 && mkdir build && cd build && cmake ../ && make -j4 && make install

RUN rm -rf cmake-3.22.6-linux-x86_64 yaml-cpp-yaml-cpp-0.6.3 open3d-devel-linux-x86_64-cxx11-abi-0.18.0 ceres-solver-2.0.0

RUN mkdir -p /tloam_ws/src/tloam/

WORKDIR /tloam_ws

COPY . src/tloam/

RUN sed -i '13s/^/#/' src/tloam/CMakeLists.txt
RUN sed -i 's/image_kind_size: 4/image_kind_size: 0/' src/tloam/config/kitti/kitti_reader.yaml
RUN sed -i 's|data_path: /media/zpw/SanDisk/Dataset/kitti/odometry|data_path: /data/|' src/tloam/config/kitti/kitti_reader.yaml

RUN catkin init \
    && catkin config --merge-devel \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin build"

RUN chmod +x /tloam_ws/src/tloam/tloam_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh", "/tloam_ws/src/tloam/tloam_entrypoint.sh"]
CMD ["bash"]
