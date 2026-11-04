#!/bin/bash
# Source ROS and the built workspace, then run the container's command.
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
exec "$@"
