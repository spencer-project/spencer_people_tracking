cmake_minimum_required(VERSION 2.8.3)
project(srl_nearest_neighbor_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp tf eigen_conversions tf_conversions spencer_diagnostics spencer_tracking_msgs)

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
)


add_executable(nnt_node ${SOURCE_DIR}/ros/nodes/nnt_node.cpp ${TRACKER_SOURCES})
add_dependencies(nnt_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nnt_node ${catkin_LIBRARIES})

add_executable(logger_node ${SOURCE_DIR}/ros/nodes/logger_node.cpp)
add_dependencies(logger_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(logger_node ${catkin_LIBRARIES})

