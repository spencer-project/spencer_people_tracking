cmake_minimum_required(VERSION 2.8.3)
project(spencer_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  roscpp diagnostic_updater
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)


add_library(spencer_diagnostics src/spencer_diagnostics/status.cpp)
add_dependencies(spencer_diagnostics ${catkin_EXPORTED_TARGETS})
target_link_libraries(spencer_diagnostics ${catkin_LIBRARIES})