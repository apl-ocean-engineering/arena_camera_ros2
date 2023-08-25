#!/bin/bash
set -e

ROS_VERSION=${ROS_VERSION:-iron}

# setup ros environment
if [[ "${ROS_WS}" != "" && -f $ROS_WS/install/setup.bash ]]; then
    echo "Loading environment from workspace: $ROS_WS"
    source $ROS_WS/install/setup.bash
else
    echo "Loading default environment: /opt/ros/$ROS_VERSION/setup.bash"
    source /opt/ros/$ROS_VERSION/setup.bash
fi

exec "$@"
