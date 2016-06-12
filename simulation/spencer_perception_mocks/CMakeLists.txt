cmake_minimum_required(VERSION 2.8.3)
project(spencer_perception_mocks)


set(PACKAGE_DEPENDENCIES
    spencer_tracking_msgs 
    spencer_spokesperson_msgs
)

find_package(catkin REQUIRED COMPONENTS
  rospy roscpp roslib ${PACKAGE_DEPENDENCIES}
)

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})
