## Arena Camera Driver for ROS2

**Project Status:**  This software is a work in progress.  At present the streaming node is functional with the sample cameras which are available to us (a [PHX122S](https://thinklucid.com/product/phoenix-12-2-mp-imx226/) and a [TRI054S](https://thinklucid.com/product/triton-5-mp-imx490/) for testing HDR modes).   The polled node is not functional.

---

This package is a ROS2 driver for LucidVision machine vision camera using their ["Arena" SDK for Ubuntu](https://thinklucid.com/downloads-hub/).

This particular version is forked from the upstream [driver published by LucidVision](https://github.com/lucidvisionlabs/arena_camera_ros2) and includes significant refactoring for functionality / readability.

See also [arena_camera_ros](https://github.com/apl-ocean-engineering/arena_camera_ros) for ROS1.

# Getting Started

This package builds as a standard ROS2 package using `colcon`, however the [Arena SDK](https://thinklucid.com/downloads-hub/) must be installed.

See the [ROS2 documentation at LucidVision](https://support.thinklucid.com/using-ros2-for-linux/) for additional details on installing the SDK.

# Getting Started -- with Docker

A full [Dockerfile](.docker/Dockerfile) is included in the [.docker/](.docker/) directory.  See the [README in that directory](.docker/README.md) for more details.

# Contents

This repo contains two ROS packages.  See the individual READMEs for each for details.

* **camera_control_msgs**:  Contains actions and service calls for adjusting camera settings (gain, etc).

* **arena_camera**:  A "generic" ROS node for Lucid cameras.  See the [README](arena_camera/README) in this package for more details.



# License

This code is forked from the [Lucid Vision's ROS2 node](https://github.com/lucidvisionlabs/arena_camera_ros2), which in turn is forked from [Basler's ROS node for their Pylon library](https://github.com/magazino/pylon_camera), which is released under the [BSD 3-Clause License](LICENSE).
