#!/usr/bin/env bash
set -e

source /opt/ros/foxy/install/setup.bash

# Build on container start only if user asks; default to interactive.
exec "$@"
