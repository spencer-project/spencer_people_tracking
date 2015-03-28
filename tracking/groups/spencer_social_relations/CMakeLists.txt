cmake_minimum_required(VERSION 2.8.3)
project(spencer_social_relations)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS rospy roscpp roslib tf spencer_tracking_msgs spencer_social_relation_msgs cmake_modules)

find_package(LibSVM REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS} ${LIBSVM_INCLUDE_DIR})

#catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy roscpp spencer_tracking_msgs spencer_social_relation_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${LIBSVM_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src/spencer_social_relations)



### Spatial relations ###
add_executable(spatial_relations
  ${SOURCE_DIR}/spatial_relations.cpp
)
add_dependencies(spatial_relations ${catkin_EXPORTED_TARGETS})
target_link_libraries(spatial_relations ${catkin_LIBRARIES} ${LIBSVM_LIBRARIES})
