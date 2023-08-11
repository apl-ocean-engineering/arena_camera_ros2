#!/bin/bash
set -e

ROS_VERSION=${ROS_VERSION:-iron}

# setup ros environment
if [ "${ROS_WS}" == "" ]; then
    echo "Loading default environment: /opt/ros/$ROS_VERSION/setup.bash"
    source /opt/ros/$ROS_VERSION/setup.bash
else
    echo "Loading environment from workspace: $ROS_WS"
    source $ROS_WS/install/setup.bash
fi

exec "$@"
