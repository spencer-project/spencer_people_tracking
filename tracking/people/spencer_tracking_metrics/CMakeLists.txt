cmake_minimum_required(VERSION 2.8.3)
project(spencer_tracking_metrics)
find_package(catkin REQUIRED COMPONENTS rospy tf sensor_msgs spencer_tracking_msgs)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/dataset_stats.py
  scripts/online_analysis.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
