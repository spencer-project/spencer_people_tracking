# Software License Agreement (BSD License)
#
#  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
project(spencer_tracking_utils)

find_package(catkin REQUIRED COMPONENTS rospy roscpp tf spencer_tracking_msgs nav_msgs cmake_modules)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy roscpp spencer_tracking_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/spencer_tracking_utils)


### Executable for logical filtering of two sets of TrackedPersons (logical AND / OR based upon track IDs) ###
add_executable(filter_tracks_logically
  ${SOURCE_DIR}/filter_tracks_logically.cpp
)
add_dependencies(filter_tracks_logically ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_tracks_logically ${catkin_LIBRARIES})


### Executable for filtering out any tracks which have not been confirmed visually (based upon modality of the associated CompositeDetectedPerson instance) ###
add_executable(filter_visually_confirmed_tracks
  ${SOURCE_DIR}/filter_visually_confirmed_tracks.cpp
)
add_dependencies(filter_visually_confirmed_tracks ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_visually_confirmed_tracks ${catkin_LIBRARIES})


### Executable for filtering non-moving targets ###
add_executable(filter_non_moving_targets
  ${SOURCE_DIR}/filter_non_moving_targets.cpp
)
add_dependencies(filter_non_moving_targets ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_non_moving_targets ${catkin_LIBRARIES})


### Executable to fix the orientation of non-moving targets (use previous orientation while target was still moving; if unknown, face towards sensor) ###
add_executable(fix_orientation_of_non_moving_targets
  ${SOURCE_DIR}/fix_orientation_of_non_moving_targets.cpp
)
add_dependencies(fix_orientation_of_non_moving_targets ${catkin_EXPORTED_TARGETS})
target_link_libraries(fix_orientation_of_non_moving_targets ${catkin_LIBRARIES})


### Executable to simulate detections from tracks ###
add_executable(tracks_to_detections
  ${SOURCE_DIR}/tracks_to_detections.cpp
)
add_dependencies(tracks_to_detections ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracks_to_detections ${catkin_LIBRARIES})


### Executable to simulate occlusions among detections, by assuming robot in origin of specified sensor target frame ###
add_executable(simulate_occluded_detections_via_raytracing
  ${SOURCE_DIR}/simulate_occluded_detections_via_raytracing.cpp
)
add_dependencies(simulate_occluded_detections_via_raytracing ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_occluded_detections_via_raytracing ${catkin_LIBRARIES})


### Executable to filter tracks based upon minimum/maximum distance to a given sensor frame origin ###
add_executable(filter_tracks_by_distance
  ${SOURCE_DIR}/filter_tracks_by_distance.cpp
)
add_dependencies(filter_tracks_by_distance ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_tracks_by_distance ${catkin_LIBRARIES})


### Executable to mark all input tracks as occluded and otherwise pass them through as-is ###
add_executable(mark_all_tracks_as_occluded
  ${SOURCE_DIR}/mark_all_tracks_as_occluded.cpp
)
add_dependencies(mark_all_tracks_as_occluded ${catkin_EXPORTED_TARGETS})
target_link_libraries(mark_all_tracks_as_occluded ${catkin_LIBRARIES})


### Executable to match existing tracks against laser points, by synchronizing with a laserscan and checking if it has laser points at the person's coordinates ###
### NOTE: It is assumed that the tracks have initially been marked as occluded by the mark_all_tracks_as_occluded executable ###
add_executable(match_tracks_against_laser
  ${SOURCE_DIR}/match_tracks_against_laser.cpp
)
add_dependencies(match_tracks_against_laser ${catkin_EXPORTED_TARGETS})
target_link_libraries(match_tracks_against_laser ${catkin_LIBRARIES})


### Executable to filter out detections which appear as obstacles in a static map ###
add_executable(filter_detections_by_static_map
  ${SOURCE_DIR}/filter_detections_by_static_map.cpp
)
add_dependencies(filter_detections_by_static_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_detections_by_static_map ${catkin_LIBRARIES})


### Executable to filter out detections which are outside of the horizontal FOV of a sensor ###
add_executable(filter_detections_by_fov
  ${SOURCE_DIR}/filter_detections_by_fov.cpp
)
add_dependencies(filter_detections_by_fov ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_detections_by_fov ${catkin_LIBRARIES})


### Executable to filter out or set to occluded tracks which are outside of the horizontal FOV of a sensor ###
add_executable(filter_tracks_by_fov
  ${SOURCE_DIR}/filter_tracks_by_fov.cpp
)
add_dependencies(filter_tracks_by_fov ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_tracks_by_fov ${catkin_LIBRARIES})



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS 
	filter_tracks_logically
	filter_visually_confirmed_tracks
	filter_non_moving_targets
	fix_orientation_of_non_moving_targets
	tracks_to_detections
	simulate_occluded_detections_via_raytracing
	filter_tracks_by_distance
	mark_all_tracks_as_occluded
	match_tracks_against_laser
	filter_detections_by_static_map
	filter_detections_by_fov
	filter_tracks_by_fov	
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
