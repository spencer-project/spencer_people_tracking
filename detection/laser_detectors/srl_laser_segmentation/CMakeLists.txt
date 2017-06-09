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


add_executable(extended_jump_distance_segmentation
  ${SOURCE_DIR}/ros/ros_interface.cpp
  ${SOURCE_DIR}/ros/nodes/extended_jump_distance_segmentation.cpp
  ${SOURCE_DIR}/extended_jump_distance.cpp
)
add_dependencies(extended_jump_distance_segmentation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(extended_jump_distance_segmentation ${catkin_LIBRARIES})


add_executable(ahc_segmentation
  ${SOURCE_DIR}/ros/ros_interface.cpp
  ${SOURCE_DIR}/ros/nodes/ahc_segmentation.cpp
  ${SOURCE_DIR}/ahc.cpp
  ${SOURCE_DIR}/efficient_ahc/efficient_ahc.cpp
  ${SOURCE_DIR}/efficient_ahc/fib.c
)
add_dependencies(ahc_segmentation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ahc_segmentation ${catkin_LIBRARIES})

install(TARGETS jump_distance_segmentation extended_jump_distance_segmentation ahc_segmentation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

