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
project(spencer_detected_person_association)

find_package(catkin REQUIRED COMPONENTS rospy roscpp message_filters spencer_tracking_msgs std_msgs message_generation nodelet eigen_conversions tf_conversions cmake_modules)


FIND_PACKAGE(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  LookupTrackId.srv
  LookupTrackIds.srv
)

catkin_python_setup()

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_filters spencer_tracking_msgs std_msgs nodelet eigen_conversions tf_conversions
)

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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/check_poses_and_covariances.py
  scripts/composite_detections_to_detections.py
  scripts/dump_detection_positions.py
  scripts/track_synchronizer_test.py
  scripts/visualize_composite_detections.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
