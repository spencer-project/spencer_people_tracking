cmake_minimum_required(VERSION 2.8.3)
project(spencer_people_tracking_launch)

find_package(catkin REQUIRED COMPONENTS)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
