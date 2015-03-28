cmake_minimum_required(VERSION 2.8.3)
project(spencer_group_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy roscpp spencer_tracking_msgs spencer_social_relation_msgs tf
)

catkin_package()
