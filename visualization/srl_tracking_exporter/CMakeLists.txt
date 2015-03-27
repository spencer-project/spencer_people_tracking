cmake_minimum_required(VERSION 2.8.3)
project(srl_tracking_exporter)
find_package(catkin REQUIRED COMPONENTS rospy tf spencer_tracking_msgs eigen_conversions tf_conversions message_generation)

add_service_files(
  FILES
  JobFinished.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

install(PROGRAMS scripts/tracks_to_svg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})