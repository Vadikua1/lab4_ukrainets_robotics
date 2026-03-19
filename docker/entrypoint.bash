#!/usr/bin/env bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if it exists
if [ -f /opt/ws/install/setup.bash ]; then
    source /opt/ws/install/setup.bash
fi

# Change to workspace directory
cd /opt/ws

exec "$@"