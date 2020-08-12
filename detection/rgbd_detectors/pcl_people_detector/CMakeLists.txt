cmake_minimum_required(VERSION 2.8.3)
project(pcl_people_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib pcl_conversions eigen_conversions tf_conversions cmake_modules
  spencer_tracking_msgs visualization_msgs spencer_vision_msgs
)


find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV version ${OpenCV_VERSION} installed at: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV supported compute capabilities are: ${OpenCV_COMPUTE_CAPABILITIES}, compiled with CUDA version: ${OpenCV_CUDA_VERSION}")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(include src ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

if(OpenCV_CUDA_VERSION)
  add_definitions(-DOPENCV_HAS_CUDA)
  message(STATUS "Building with OpenCV CUDA support enabled")
endif()

add_executable(detect_people src/detect_people.cpp)
add_dependencies(detect_people ${catkin_EXPORTED_TARGETS}) # for generated message headers etc.
target_link_libraries(detect_people ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS detect_people
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) # for copying the target to the devel folder

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY parameters/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/parameters
  PATTERN ".svn" EXCLUDE)
