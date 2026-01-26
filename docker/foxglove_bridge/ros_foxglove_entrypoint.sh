#!/bin/bash
set -e

# Source ROS Galactic environment
source /opt/ros/galactic/install/setup.bash
source /workspace/foxglove_ws/install/setup.bash

echo "Starting Foxglove Bridge..."
ros2 run foxglove_bridge foxglove_bridge > /workspace/foxglove_bridge.log 2>&1 &

# Wait a moment to verify startup
sleep 3
ps aux | grep foxglove_bridge

echo "Foxglove Bridge is running. Logs: /workspace/foxglove_bridge.log"
echo "Keeping container alive..."
tail -f /dev/null
