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