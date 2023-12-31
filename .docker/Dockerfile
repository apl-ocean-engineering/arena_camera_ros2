ARG ROS_VERSION=iron

#====================================================================
# Preliminary image with dependencies
#====================================================================
#
## This should work on both amd64 and arm64 through the magic of multi-arch?
FROM ros:${ROS_VERSION}-perception AS deps

# This should be set automatically by the build engine
ARG ROS_VERSION
ARG TARGETPLATFORM
ARG ARENA_SDK_URL_BASE=https://s3.us-east-1.wasabisys.com/amarburg-east/LucidVision_SDK

# Install the LucidVision SDK
WORKDIR /tmp

RUN apt-get update && apt-get install -y \
    git \
    python3-vcstool \
    ros-${ROS_VERSION}-diagnostic-updater \
    ros-${ROS_VERSION}-generate-parameter-library \
    wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${TARGETPLATFORM}" = "linux/amd64" ]; then \
      ARENA_SDK_FILE=ArenaSDK_v0.1.68_Linux_x64.tar.gz; \
    elif [ "${TARGETPLATFORM}" = "linux/arm64" ]; then \
      ARENA_SDK_FILE=ArenaSDK_v0.1.49_Linux_ARM64.tar.gz; \
    else \
      echo "Unknown target platform ${TARGETPLATFORM}"; \
      exit; \
    fi && \
    ARENA_SDK_URL=${ARENA_SDK_URL_BASE}/${ARENA_SDK_FILE} && \
    echo "Downloading ${ARENA_SDK_URL}" && \
    wget $ARENA_SDK_URL && \
    cd /usr/local && \
    tar -xf /tmp/$ARENA_SDK_FILE && \
    rm /tmp/$ARENA_SDK_FILE

ENV ARENA_ROOT=/usr/local/ArenaSDK
# The name of the directory in the tarball changes by architecture,
# so create a convenience symlink
RUN ln -s /usr/local/ArenaSDK_* ${ARENA_ROOT}
WORKDIR ${ARENA_ROOT}
RUN sh -c "sudo sh Arena_SDK_*.conf"


#====================================================================
# Intermediate image with prereq steps for running in CI
#====================================================================
FROM deps as ci

## ~~ Create non-root user ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && apt-get update \
  && apt-get install -y sudo \
  && apt autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/* \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && echo "Defaults:$USERNAME env_keep += \"http_proxy https_proxy ftp_proxy all_proxy no_proxy\"" > /etc/sudoers.d/proxy \
  && chmod 0440 /etc/sudoers.d/$USERNAME /etc/sudoers.d/proxy

## ~~ Switch to non-root user ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
USER $USERNAME

# Install the default ROS entrypoint
COPY --chmod=0755  ros2_entrypoint.sh /home/$USERNAME/ros2_entrypoint.sh
ENTRYPOINT [ "/home/ros/ros2_entrypoint.sh" ]
CMD ["/bin/bash"]

#====================================================================
# Intermediate image with cloned Git repos
#====================================================================
FROM ci as clone

ARG WS_DIR=/home/$USERNAME/ros_ws
WORKDIR ${WS_DIR}/src

ARG ARENA_CAMERA_ROS2_REPO=https://github.com/apl-ocean-engineering/arena_camera_ros2.git
ARG ARENA_CAMERA_ROS2_BRANCH=main
RUN echo "Cloning from ${ARENA_CAMERA_ROS2_BRANCH} branch ${ARENA_CAMERA_ROS2_REPO}"

# This will break cache when the repo changes
ADD https://api.github.com/repos/apl-ocean-engineering/arena_camera_ros2/git/refs/heads/${ARENA_CAMERA_ROS2_BRANCH} version.json
RUN git clone --depth 1 -b ${ARENA_CAMERA_ROS2_BRANCH} ${ARENA_CAMERA_ROS2_REPO}

#====================================================================
# Final image with built ROS2 packages
#====================================================================
# dockerfile_lint - ignore
FROM clone as arena_camera_ros2

ARG ROS_VERSION
ARG WS_DIR
LABEL Version=0.1
LABEL Name=arena_camera_ros2

WORKDIR ${WS_DIR}/src
RUN vcs import --shallow --skip-existing < arena_camera_ros2/arena_camera_ros2.repos

WORKDIR ${WS_DIR}

RUN rosdep update --rosdistro=${ROS_VERSION}

# This is quite expensive to run on every build...
# n.b. this must be run through sudo, not with USER root,
# because "rosdep install" runs as the user
RUN sudo apt-get update \
    && ~/ros2_entrypoint.sh \
    && rosdep install -y --ignore-src \
                    --rosdistro=${ROS_VERSION} \
                    --skip-keys=arena_sdk \
                    --from-paths src/\
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

RUN ~/ros2_entrypoint.sh \
                colcon build && \
                colcon test

# Convert ARG to ENV
ENV ROS_WS ${WS_DIR}
CMD ["ros2", "launch", "arena_camera", "streaming.launch.py"]
