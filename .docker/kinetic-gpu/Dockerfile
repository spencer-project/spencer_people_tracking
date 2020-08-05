# GPU support (only works if nvidia-container-toolkit is installed on the docker host)!
FROM nvidia/cudagl:10.1-runtime-ubuntu16.04
RUN apt-get update && apt-get install -y lsb-release && rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Following block originates from https://github.com/osrf/docker_images/blob/master/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile
# and http://wiki.ros.org/kinetic/Installation/Ubuntu
# Copyright 2020 Open Source Robotics Foundation (CC-BY 3.0 license)
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --no-install-recommends ros-kinetic-perception && rm -rf /var/lib/apt/lists/*
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO kinetic

# Install build tools
RUN apt-get update && apt-get install -y \
      python-catkin-tools tmux vim \
    && rm -rf /var/lib/apt/lists/*

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