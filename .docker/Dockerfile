ARG BUILD_TYPE=clone
ARG ROS_VERSION=iron

## This should work on both amd64 and arm64 through the magic of multi-arch?
FROM ros:${ROS_VERSION}-perception AS build_image

# This should be set automatically by the build engine
ARG TARGETPLATFORM
ARG ARENA_SDK_URL_BASE=https://s3.us-west-1.wasabisys.com/marburg/LucidVision_SDK

# Install the LucidVision SDK
WORKDIR /tmp

RUN apt-get update && apt-get install -y \
    git \
    python3-vcstool \
    wget \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${TARGETPLATFORM}" = "linux/amd64" ]; then \
      ARENA_SDK_FILE=ArenaSDK_v0.1.68_Linux_x64.tar.bz2; \
    elif [ "${TARGETPLATFORM}" = "linux/arm64" ]; then \
      ARENA_SDK_FILE=ArenaSDK_v0.1.49_Linux_ARM64.tar.bz2; \
    else \
      echo "Unknown target platform ${TARGETPLATFORM}"; \
      exit; \
    fi && \
    ARENA_SDK_URL=${ARENA_SDK_URL_BASE}/${ARENA_SDK_FILE} && \
    echo "Downloading ${ARENA_SDK_URL}" && \
    wget $ARENA_SDK_URL && \
    cd /usr/local && \
    tar -xjvf /tmp/$ARENA_SDK_FILE && \
    rm /tmp/$ARENA_SDK_FILE

ENV ARENA_ROOT=/usr/local/ArenaSDK
# The name of the directory in the tarball changes by architecture,
# so create a convenience symlink
RUN ln -s /usr/local/ArenaSDK_* ${ARENA_ROOT}
WORKDIR ${ARENA_ROOT}
RUN sh -c "sudo sh Arena_SDK_*.conf"

## ~~ Create non-root user ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

## ~~ Switch to non-root user ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

USER $USERNAME

# Install the default ROS entrypoint
COPY --chmod=0755  ros2_entrypoint.sh /home/$USERNAME/ros2_entrypoint.sh
#ENTRYPOINT [ "/home/ros/ros2_entrypoint.sh" ]

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  This build path clones from github
FROM build_image as build_clone


ARG WS_DIR=/home/$USERNAME/ros_ws
ONBUILD WORKDIR ${WS_DIR}/src

ARG ARENA_CAMERA_ROS2_REPO=https://github.com/apl-ocean-engineering/arena_camera_ros2.git
ARG ARENA_CAMERA_ROS2_BRANCH=master
ONBUILD RUN echo "Cloning from ${ARENA_CAMERA_ROS2_BRANCH} branch ${ARENA_CAMERA_ROS2_REPO}"

# This will break cache when the repo changes
ONBUILD ADD https://api.github.com/repos/apl-ocean-engineering/arena_camera_ros2/git/refs/heads/${ARENA_CAMERA_ROS2_BRANCH} version.json
ONBUILD RUN git clone -b ${ARENA_CAMERA_ROS2_BRANCH} ${ARENA_CAMERA_ROS2_REPO}

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# This build path copies from a local tree
#
# I find the structure of the arena_camera_ros drive extremely annoying and
# can see changing away from it.
# FROM build_image as build_local

# ARG LOCAL_SRC=local_src

# ARG WS_DIR
# ONBUILD WORKDIR ${WS_DIR}/src
# ONBUILD RUN echo "Pulling source code from local directory ${LOCAL_SRC}"
# ONBUILD COPY ${LOCAL_SRC}/ ${WS_DIR}/src/

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Then the two build paths re-merge here
# dockerfile_lint - ignore
FROM build_${BUILD_TYPE}

ARG WS_DIR
LABEL Version=0.1
LABEL Name=arena_camera_ros2

WORKDIR ${WS_DIR}/src
RUN vcs import --shallow --skip-existing < arena_camera_ros2/arena_camera_ros2.repos

WORKDIR ${WS_DIR}

# This is quite expensive to run on every build...
# RUN bash -c "apt-get update \
#               && source /opt/ros/noetic/setup.bash \
#               && rosdep install -y --ignore-src \
#                     --skip-keys=arena_sdk \
#                     --from-paths src/ \
#               && rm -rf /var/lib/apt/lists/*"

RUN bash -c "~/ros2_entrypoint.sh \
                colcon build && \
                colcon test"

# Convert ARG to ENV
ENV ROS_WS ${WS_DIR}
#CMD ["ros2", "launch", "arena_camera", "streaming_node.launch"]
