FROM ros:indigo-perception

# Install build tools
RUN apt-get update && apt-get install -y \
      python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Acquire source
RUN git clone https://github.com/spencer-project/spencer_people_tracking.git
# COPY . spencer_people_tracking/

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
      wget && \
    rosdep update && \
    rosdep install -y -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# Build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build
