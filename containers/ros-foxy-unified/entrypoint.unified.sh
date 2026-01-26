#!/usr/bin/env bash
set -e

# Base ROS
source /opt/ros/foxy/install/setup.bash

# RealSense overlay built into the image
if [ -f /opt/realsense_ws/install/setup.bash ]; then
  source /opt/realsense_ws/install/setup.bash
fi

# Optional: user workspace overlay (mounted from host)
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
