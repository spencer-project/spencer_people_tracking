cmake_minimum_required(VERSION 2.8.3)
project(3rd_party)

##################################################################################################################
### CUDA HOG ###
message(STATUS "Preparing libcudaHOG")
message(STATUS "Created pkg-config file: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc
    @ONLY
)
set(groundHOG_tar http://www.vision.rwth-aachen.de/media/resource_files/groundHOG_v1.1_b.tar.gz)
message(STATUS "Will download libcudaHOG from: ${groundHOG_tar}")
include(ExternalProject)
ExternalProject_Add(
    libcudaHOG
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG
    URL ${groundHOG_tar}
		PATCH_COMMAND patch -p0 <SOURCE_DIR>/cudaHOG/cudaHOG.cpp < ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/patch/cudaHOG.cpp.diff
    CONFIGURE_COMMAND qmake -makefile <SOURCE_DIR>
    BUILD_COMMAND make
    INSTALL_COMMAND "" #Skipp install step. See below
)

### Install files ###
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG/cudaHOG
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so.1
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so.1.0
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so.1.0.0
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
) 

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
) 

##################################################################################################################
