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
project(srl_laser_features)

set(LASER_FEATURE_LIB srl_laser_features)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LASER_FEATURE_LIB}
  CATKIN_DEPENDS roscpp eigen_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/srl_laser_features)


### Laser feature library ###
add_library(${LASER_FEATURE_LIB}
  ${SOURCE_DIR}/features/feature_registry.cpp
  ${SOURCE_DIR}/features/feature00.cpp
  ${SOURCE_DIR}/features/feature01.cpp
  ${SOURCE_DIR}/features/feature02.cpp
  ${SOURCE_DIR}/features/feature03.cpp
  ${SOURCE_DIR}/features/feature04.cpp
  ${SOURCE_DIR}/features/feature05.cpp
  ${SOURCE_DIR}/features/feature06.cpp
  ${SOURCE_DIR}/features/feature07.cpp
  ${SOURCE_DIR}/features/feature08.cpp
  ${SOURCE_DIR}/features/feature09.cpp
  ${SOURCE_DIR}/features/feature10.cpp
  ${SOURCE_DIR}/features/feature11.cpp
  ${SOURCE_DIR}/features/feature12.cpp
  ${SOURCE_DIR}/features/feature13.cpp
  ${SOURCE_DIR}/features/feature14.cpp
  ${SOURCE_DIR}/features/feature15.cpp
  ${SOURCE_DIR}/features/feature33.cpp
)
add_dependencies(${LASER_FEATURE_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LASER_FEATURE_LIB} ${catkin_LIBRARIES})


install(TARGETS ${LASER_FEATURE_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
