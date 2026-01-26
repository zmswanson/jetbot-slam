# ==========================================================
#  Unified JetBot + RealSense container (JetPack 4.6 / L4T r32.7.1)
#
#  - Starts from the working RealSense Dockerfile.foxy
#  - Adds JetBot-dev dependencies (i2c-tools, libi2c-dev, colcon extensions, rosdep)
#  - Removes auto-launch of realsense (no default ros2 launch in CMD)
#  - Builds realsense-ros (4.51.1) into a separate overlay: /opt/realsense_ws
# ==========================================================

# ==========================================================
#  Stage 1: Base (ROS 2 Foxy on JetPack 4.6)
# ==========================================================
FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1 AS base

LABEL maintainer="Zach Swanson <zmswanson@outlook.com>"
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

# ----------------------------------------------------------
# Fix ROS 2 GPG key & repo duplicates (common on older images)
# ----------------------------------------------------------
RUN apt-key del F42ED6FBAB17C654 || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu bionic main" \
        | tee /etc/apt/sources.list.d/ros2-latest.list && \
    rm -f /etc/apt/sources.list.d/ros2.list &&     apt-get update

# ----------------------------------------------------------
# Core dependencies (RealSense + JetBot dev)
# ----------------------------------------------------------
RUN apt-get install -y --no-install-recommends \
    git cmake build-essential libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3-pip python3-numpy python3-pytest curl gnupg2 lsb-release \
    python3-colcon-common-extensions python3-rosdep \
    i2c-tools libi2c-dev  && rm -rf /var/lib/apt/lists/*

# rosdep may already be initialized in Dusty's image
RUN rosdep init || true

# ==========================================================
#  Stage 2: librealsense
# ==========================================================
FROM base AS librealsense

WORKDIR /opt
RUN git clone https://github.com/IntelRealSense/librealsense.git

WORKDIR /opt/librealsense
RUN git checkout v2.54.1 && ./scripts/setup_udev_rules.sh

RUN mkdir -p build && cd build && \
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
#  Stage 3: realsense-ros overlay workspace (/opt/realsense_ws)
# ==========================================================
FROM librealsense AS jetbot-realsense

# Build realsense-ros into a dedicated overlay so you can mount /ros2_ws for dev
WORKDIR /opt/realsense_ws/src
RUN rm -rf * || true

# Intel RealSense ROS wrapper (4.51.1 is Foxy-compatible tag)
# xacro + diagnostics are pulled as source deps (your original approach)
RUN git clone -b 4.51.1 https://github.com/IntelRealSense/realsense-ros.git && \
    git clone -b 2.0.6 https://github.com/ros/xacro.git && \
    git clone -b stale/foxy https://github.com/ros/diagnostics.git

WORKDIR /opt/realsense_ws
RUN bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && rosdep update --include-eol-distros"
RUN bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && rosdep install --from-paths src --ignore-src -y              --skip-keys 'librealsense2 launch_pytest ntpdate'"
RUN bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && colcon build --symlink-install --packages-skip diagnostic_remote_logging &&              rm -rf /var/lib/apt/lists/*"

# ----------------------------------------------------------
# Entrypoint (no auto-launch; interactive by default)
# ----------------------------------------------------------
COPY entrypoint.unified.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Source ROS + overlays in interactive shells too
RUN echo "source /opt/ros/$ROS_DISTRO/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/realsense_ws/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
