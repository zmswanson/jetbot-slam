# ==========================================================
#  Stage 1: Base (ROS 2 Foxy on JetPack 4.6)
# ==========================================================
FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1 AS base

LABEL maintainer="Zach Swanson <zmswanson@unl.edu>"
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
WORKDIR /workspace

# Fix ROS 2 GPG key & repo duplicates
RUN apt-key del F42ED6FBAB17C654 || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu bionic main" \
        | tee /etc/apt/sources.list.d/ros2-latest.list && \
    rm -f /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Core dependencies
RUN apt-get install -y --no-install-recommends \
    git cmake build-essential libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3-pip python3-numpy python3-pytest curl gnupg2 lsb-release \
 && rm -rf /var/lib/apt/lists/*


# ==========================================================
#  Stage 2: librealsense
# ==========================================================
FROM base AS librealsense

WORKDIR /opt
RUN git clone https://github.com/IntelRealSense/librealsense.git 

WORKDIR /opt/librealsense
RUN git checkout v2.54.1 && ./scripts/setup_udev_rules.sh

RUN  mkdir build && cd build && \
    cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release \
             -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true \
             -DFORCE_LIBUVC=true && \
    make -j"$(($(nproc)-1))" && make install && ldconfig

# Python bindings
RUN cd /opt/librealsense/build && \
    cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true \
              -DPYTHON_EXECUTABLE=$(which python3) && \
    make -j"$(($(nproc)-1))" && make install && ldconfig

# ==========================================================
#  Stage 3: ros-realsense (JetPack 4.6 + ROS2 Foxy)
# ==========================================================
FROM librealsense AS ros-realsense

WORKDIR /workspace/ros2_ws/src
# Clean workspace src directory
RUN rm -rf *

# Intel RealSense ROS wrapper (v4.51.1 is last Foxy-compatible tag)
RUN git clone -b 4.51.1 https://github.com/IntelRealSense/realsense-ros.git && \
    # Xacro and Diagnostics built from source (Foxy/ROS2 branches)
    git clone -b 2.0.6 https://github.com/ros/xacro.git && \
    git clone -b stale/foxy https://github.com/ros/diagnostics.git

WORKDIR /workspace/ros2_ws
RUN bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
             rosdep update --include-eol-distros"
RUN bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
             rosdep install --from-paths src --ignore-src -y \
             --skip-keys 'librealsense2 launch_pytest ntpdate'"

RUN bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
             colcon build --symlink-install \
             --packages-skip diagnostic_remote_logging && \
             rm -rf /var/lib/apt/lists/*"


# ==========================================================
#  Final environment setup
# ==========================================================
# Ensure ROS 2 Foxy environment is sourced for all interactive shells
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]

