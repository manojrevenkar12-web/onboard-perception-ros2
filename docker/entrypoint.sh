#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/opr/setup.bash
exec "$@"
