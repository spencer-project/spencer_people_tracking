cmake_minimum_required(VERSION 2.8.3)
project(video_to_bagfile)

find_package(OpenCV REQUIRED core imgproc highgui)
find_package(catkin REQUIRED COMPONENTS image_transport rospy roscpp roslib rosbag cv_bridge)

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_to_bagfile src/video_to_bagfile/video_to_bagfile.cpp)
target_link_libraries(video_to_bagfile ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS video_to_bagfile
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)