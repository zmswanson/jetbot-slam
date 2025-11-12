#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/foxy/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

echo "Launching RealSense node in background..."
ros2 launch realsense2_camera rs_launch.py config_file:="'/workspace/jetbot-slam/realsense_lowpower.yaml'" &

# Optional: wait a bit to confirm it's alive
sleep 3
ps aux | grep realsense2_camera

# Keep container running
echo "Container will remain active..."
tail -f /dev/null
