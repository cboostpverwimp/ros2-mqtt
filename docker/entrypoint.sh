#! /bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source /app/install/setup.bash

# TODO: use "$@" so user can specify which files to launch
exec "$@"