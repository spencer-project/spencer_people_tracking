cmake_minimum_required(VERSION 2.8.3)
project(spencer_detected_person_association)

find_package(catkin REQUIRED COMPONENTS rospy roscpp message_filters spencer_tracking_msgs std_msgs message_generation nodelet eigen_conversions tf_conversions cmake_modules)


FIND_PACKAGE(Eigen REQUIRED)

#add_service_files(
#  FILES
#  LookupTrackId.srv
#  LookupTrackIds.srv
#)

#catkin_python_setup()

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_filters spencer_tracking_msgs std_msgs nodelet eigen_conversions tf_conversions
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/spencer_detected_person_association)


### Nodelet library for fusion of detected persons ###
add_library(${PROJECT_NAME}
  ${SOURCE_DIR}/convert_to_composite_detections.cpp
  ${SOURCE_DIR}/composite_detected_persons_synchronizer.cpp
  ${SOURCE_DIR}/aggregate_detections.cpp
  ${SOURCE_DIR}/nn_fuser.cpp
  ${SOURCE_DIR}/euclidean_nn_fuser.cpp
  ${SOURCE_DIR}/polar_nn_fuser.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
