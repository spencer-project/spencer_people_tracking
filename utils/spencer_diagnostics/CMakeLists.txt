cmake_minimum_required(VERSION 2.8.3)
project(spencer_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  roscpp diagnostic_updater tf diagnostic_msgs spencer_control_msgs
)

catkin_python_setup()

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

add_executable(tf_lookup_monitor src/spencer_diagnostics/tf_lookup_monitor.cpp)
add_dependencies(tf_lookup_monitor ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_lookup_monitor ${catkin_LIBRARIES})

install(TARGETS spencer_diagnostics tf_lookup_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
