cmake_minimum_required(VERSION 2.8.3)
project(rwth_ground_plane)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge message_filters roscpp std_msgs rwth_perception_people_msgs tf sensor_msgs image_transport spencer_diagnostics)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs#   rwth_perception_people_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rwth_ground_plane
#  CATKIN_DEPENDS cv_bridge message_filters rosccp std_msgs rwth_perception_people_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  include include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rwth_ground_plane
#   src/${PROJECT_NAME}/rwth_ground_plane.cpp
# )

## Declare a cpp executable
add_executable(ground_plane_estimated src/estimated_gp.cpp src/Camera.cpp src/ConfigFile.cpp src/Globals.cpp src/groundplaneestimator.cpp src/Math.cpp src/pointcloud.cpp)
add_executable(ground_plane_fixed src/fixed_gp.cpp)
add_executable(ground_plane_really_fixed src/really_fixed_gp.cpp)
add_executable(ground_plane_tf_based_fixed src/tf_based_fixed_gp.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(ground_plane_estimated rwth_perception_people_msgs_generate_messages_cpp rwth_perception_people_msgs_gencpp)
add_dependencies(ground_plane_fixed rwth_perception_people_msgs_generate_messages_cpp rwth_perception_people_msgs_gencpp)
add_dependencies(ground_plane_really_fixed rwth_perception_people_msgs_generate_messages_cpp rwth_perception_people_msgs_gencpp)
add_dependencies(ground_plane_tf_based_fixed rwth_perception_people_msgs_generate_messages_cpp rwth_perception_people_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ground_plane_estimated
   ${catkin_LIBRARIES}
)
target_link_libraries(ground_plane_fixed
   ${catkin_LIBRARIES}
)
target_link_libraries(ground_plane_really_fixed
   ${catkin_LIBRARIES}
)
target_link_libraries(ground_plane_tf_based_fixed
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ground_plane_estimated ground_plane_fixed ground_plane_really_fixed ground_plane_tf_based_fixed
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rwth_ground_plane.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
