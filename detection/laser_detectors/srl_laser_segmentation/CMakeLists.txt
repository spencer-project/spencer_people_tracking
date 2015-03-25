cmake_minimum_required(VERSION 2.8.3)
project(srl_laser_segmentation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  visualization_msgs
  eigen_conversions
  message_generation
)

add_message_files(
   FILES
   LaserscanSegment.msg
   LaserscanSegmentation.msg
)

catkin_python_setup()

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime 
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/srl_laser_segmentation)


add_executable(jump_distance_segmentation 
  ${SOURCE_DIR}/ros/ros_interface.cpp
  ${SOURCE_DIR}/ros/nodes/jump_distance_segmentation.cpp
  ${SOURCE_DIR}/jump_distance.cpp
)
add_dependencies(jump_distance_segmentation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(jump_distance_segmentation ${catkin_LIBRARIES})


add_executable(ahc_segmentation
  ${SOURCE_DIR}/ros/ros_interface.cpp
  ${SOURCE_DIR}/ros/nodes/ahc_segmentation.cpp
  ${SOURCE_DIR}/ahc.cpp
  ${SOURCE_DIR}/efficient_ahc/efficient_ahc.cpp
  ${SOURCE_DIR}/efficient_ahc/fib.c
)
add_dependencies(ahc_segmentation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ahc_segmentation ${catkin_LIBRARIES})
