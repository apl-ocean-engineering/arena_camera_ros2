## Arena Camera Driver for ROS1

Based on the upstream [Arena Camera Driver](https://github.com/lucidvisionlabs/arena_camera_ros) from LucidVision.
# Getting Started

See the full instructions at Lucid:  https://support.thinklucid.com/using-ros-for-linux/

This package builds in the same way as other `catkin` ROS packages, however the [Arena SDK]() must be installed.
# ROS Packages

Contains three ROS packages:

## camera_control_msgs

Contains actions and service calls for adjusting camera settings (gain, etc).

## arena_camera

A "generic" ROS node for Lucid cameras.

## arena_imx490

This node has been tuned / simplified for the HDR modes of the IMX490-based Lucid cameras.
# License

This code is forked from the Lucid Vision's code, which in turn is forked from [Basler's ROS node for their Pylon library](https://github.com/magazino/pylon_camera), which is released under the [BSD 3-Clause License](LICENSE).
