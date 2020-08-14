# GPU support (only works if nvidia-container-toolkit is installed on the docker host)!
# FIXME: 20.04 image with GL support is not yet available. Update this file once it is ready
FROM nvidia/cudagl:10.1-runtime-ubuntu20.04
RUN apt-get update && apt-get install -y lsb-release curl && rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Following block originates from https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/bionic/ros-core/Dockerfile
# and http://wiki.ros.org/noetic/Installation/Ubuntu
# Copyright 2020 Open Source Robotics Foundation (CC-BY 3.0 license)
# setup timezone, otherwise we get prompted for it during docker build
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-perception && rm -rf /var/lib/apt/lists/*
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Install build tools
RUN apt-get update && apt-get install -y \
      python3-catkin-tools python3-rosdep python3-pip tmux vim \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install osrf-pycommon

# Setup workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Acquire source
#RUN git clone https://github.com/spencer-project/spencer_people_tracking.git
COPY . spencer_people_tracking/

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
      wget && \
    rosdep init && \
    rosdep update && \
    rosdep install -y -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# Build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build -c -s

 # Initialize bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> /root/.bashrc