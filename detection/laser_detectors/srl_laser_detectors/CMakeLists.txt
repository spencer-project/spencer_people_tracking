cmake_minimum_required(VERSION 2.8.3)
project(srl_laser_detectors)

find_package(OpenCV)
if(OpenCV_FOUND)
  message(STATUS "Using OpenCV version ${OpenCV_VERSION} installed at: ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OpenCV supported compute capabilities are: ${OpenCV_COMPUTE_CAPABILITIES}")
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  visualization_msgs
  srl_laser_segmentation
  srl_laser_features
  eigen_conversions
  message_filters
  message_generation
  spencer_tracking_msgs
  spencer_diagnostics
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs srl_laser_segmentation message_filters message_runtime 
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/srl_laser_detectors)

### Naive blob detector ###
add_executable(blob_detector 
  ${SOURCE_DIR}/ros/ros_interface.cpp
  ${SOURCE_DIR}/segments/segment_utils.cpp    
  ${SOURCE_DIR}/ros/nodes/blob_detector_node.cpp
  ${SOURCE_DIR}/naive_detectors/blob_detector.cpp
)
add_dependencies(blob_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(blob_detector ${catkin_LIBRARIES})


### Learned detectors ###
if(OpenCV_FOUND)
  set(DETECTOR_SOURCES
    ${SOURCE_DIR}/learned_detectors/opencv_detector.cpp
    ${SOURCE_DIR}/learned_detectors/svm_detector.cpp
    ${SOURCE_DIR}/learned_detectors/adaboost_detector.cpp
    ${SOURCE_DIR}/learned_detectors/random_forest_detector.cpp
    ${SOURCE_DIR}/naive_detectors/blob_detector.cpp

    ${SOURCE_DIR}/ros/ros_interface.cpp
    ${SOURCE_DIR}/detector_factory.cpp    
    ${SOURCE_DIR}/segments/segment_utils.cpp    
  )
  set(DETECTOR_LIBRARIES
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}    # laser feature lib is linked here
  )

  ### Generic detector node ###
  set(DETECTOR_NODE detector)
  add_executable(${DETECTOR_NODE}
    ${SOURCE_DIR}/ros/nodes/generic_detector_node.cpp
    ${DETECTOR_SOURCES}    
  )
  add_dependencies(${DETECTOR_NODE} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${DETECTOR_NODE} ${DETECTOR_LIBRARIES})

  ### Training node ###
  set(TRAINING_NODE train_detector)
  add_executable(${TRAINING_NODE}
    ${SOURCE_DIR}/ros/nodes/train_detector_node.cpp
    ${DETECTOR_SOURCES}    
  )
  add_dependencies(${TRAINING_NODE} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${TRAINING_NODE} ${DETECTOR_LIBRARIES})  # laser feature lib is linked here

else()
  message(WARNING "OpenCV not found, will not build learned detectors! ${DETECTOR_NODE} and ${TRAINING_NODE} executables will not be available.")
endif()
