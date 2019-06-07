# Container for testing ABB 042 Driver
#
# Build:
#  docker build --rm -f Dockerfile -t gramaziokohler/abb-a042-driver .
#
# Usage outside DockerHub:
#  docker save -o abb-a042-driver-latest.tar gramaziokohler/abb-a042-driver:latest
#  docker load -i abb-a042-driver-latest.tar
#
# Usage:
#  docker pull gramaziokohler/abb-a042-driver

FROM ros:kinetic
LABEL maintainer "Gonzalo Casas <casas@arch.ethz.ch>"

SHELL ["/bin/bash","-c"]

# Install packages
RUN apt-get update && apt-get install -y \
    # Basic utilities
    iputils-ping \
    # ROS bridge server and related packages
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-tf2-web-republisher \
    --no-install-recommends \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
# Add ABB A042 Driver package
ADD . $CATKIN_WS/src/abb_042_driver
WORKDIR $CATKIN_WS/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get update \
    # ROS File Server
    && git clone https://github.com/gramaziokohler/ros_file_server.git \
    # ABB packages
    && git clone -b ${ROS_DISTRO}-devel https://github.com/ros-industrial/abb.git \
    && git clone -b ${ROS_DISTRO}-devel https://github.com/ros-industrial/abb_experimental.git \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && catkin_make

COPY ./.docker/ros_catkin_entrypoint.sh /

ENTRYPOINT ["/ros_catkin_entrypoint.sh"]
CMD ["bash"]
