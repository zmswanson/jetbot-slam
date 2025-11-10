FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1

LABEL maintainer="Zach Swanson <zmswanson@outlook.com>"
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

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    i2c-tools \
    libi2c-dev \
 && rm -rf /var/lib/apt/lists/*

# rosdep may already be initialized in Dusty's image
RUN rosdep init || true

WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
