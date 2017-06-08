# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2016, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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
project(srl_nearest_neighbor_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp tf eigen_conversions tf_conversions spencer_diagnostics spencer_tracking_msgs cmake_modules angles srl_laser_segmentation)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package()

if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
  # Disable debug and info-level logging statements in release mode
  add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_WARN)
else()
  add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/srl_nearest_neighbor_tracker)

set(TRACKER_SOURCES
    ${SOURCE_DIR}/ros/geometry_utils.cpp
    ${SOURCE_DIR}/ros/params.cpp
    ${SOURCE_DIR}/ros/ros_interface.cpp
    ${SOURCE_DIR}/ekf.cpp
    ${SOURCE_DIR}/nearest_neighbor_tracker.cpp
    ${SOURCE_DIR}/logic_initiator.cpp
    ${SOURCE_DIR}/imm_filter.cpp

    ${SOURCE_DIR}/occlusion_handling/laser_shade_occlusion_manager.cpp
    ${SOURCE_DIR}/occlusion_handling/basic_occlusion_manager.cpp
    ${SOURCE_DIR}/occlusion_handling/polygon_occlusion_manager.cpp
    ${SOURCE_DIR}/occlusion_handling/occlusion_geodesics_manager.cpp

    ${SOURCE_DIR}/data_association/basic_nearest_neighbor_data_association.cpp
    ${SOURCE_DIR}/data_association/global_nearest_neighbor_data_association.cpp
    ${SOURCE_DIR}/data_association/greedy_nearest_neighbor_data_association.cpp

    ${SOURCE_DIR}/missed_observation_recovery/low_confidence_observations_recovery.cpp
)


add_executable(nnt_node ${SOURCE_DIR}/ros/nodes/nnt_node.cpp ${TRACKER_SOURCES})
add_dependencies(nnt_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nnt_node ${catkin_LIBRARIES})

add_executable(logger_node ${SOURCE_DIR}/ros/nodes/logger_node.cpp)
add_dependencies(logger_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(logger_node ${catkin_LIBRARIES})

install(TARGETS nnt_node logger_node
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
