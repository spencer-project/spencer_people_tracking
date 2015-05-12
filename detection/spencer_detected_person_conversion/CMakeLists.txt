cmake_minimum_required(VERSION 2.8.3)
project(spencer_detected_person_conversion)

find_package(catkin REQUIRED COMPONENTS rospy spencer_tracking_msgs)

catkin_package(
  CATKIN_DEPENDS rospy spencer_tracking_msgs
)
