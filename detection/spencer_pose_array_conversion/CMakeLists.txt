cmake_minimum_required(VERSION 2.8.3)
project(spencer_pose_array_conversion)

find_package(catkin REQUIRED COMPONENTS rospy spencer_tracking_msgs)

catkin_package(
  CATKIN_DEPENDS rospy spencer_tracking_msgs
)
