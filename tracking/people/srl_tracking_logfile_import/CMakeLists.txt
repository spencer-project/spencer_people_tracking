cmake_minimum_required(VERSION 2.8.3)
project(srl_tracking_logfile_import)
find_package(catkin REQUIRED COMPONENTS rospy tf srl_laser_segmentation)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/publish_logfile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})