cmake_minimum_required(VERSION 2.8.3)
project(spencer_tracking_utils)

find_package(catkin REQUIRED COMPONENTS rospy roscpp tf spencer_tracking_msgs cmake_modules)

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
