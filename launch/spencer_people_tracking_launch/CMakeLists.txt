cmake_minimum_required(VERSION 2.8.3)
project(spencer_people_tracking_launch)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/download_example_bagfiles.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
