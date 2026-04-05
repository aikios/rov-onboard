#!/bin/bash
set +u
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
exec "$@"
