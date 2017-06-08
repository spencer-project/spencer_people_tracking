# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 2.8.3)
project(srl_laser_detectors)

find_package(OpenCV 2)
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

install(TARGETS blob_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  PATTERN ".svn" EXCLUDE
)

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

  install(TARGETS ${TRAINING_NODE} ${DETECTOR_NODE}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


else()
  message(WARNING "OpenCV not found, will not build learned detectors! ${DETECTOR_NODE} and ${TRAINING_NODE} executables will not be available.")
endif()
