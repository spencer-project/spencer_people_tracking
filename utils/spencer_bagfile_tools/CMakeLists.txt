# Software License Agreement (BSD License)
# 
# Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 2.8.3)
project(spencer_bagfile_tools)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs image_transport image_geometry cv_bridge tf message_generation dynamic_reconfigure camera_calibration_parsers)

add_message_files(
   FILES
   AdditionalOdometryData.msg        #  copied from former kindercar_odometry package
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ReconstructOdometry.cfg
)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


# # #  panorama_builder executable (receives from and publishes to ROS) # # # 
add_executable(panorama_builder src/spencer_bagfile_tools/panorama_builder.cpp)
target_link_libraries(panorama_builder ${catkin_LIBRARIES})


# # #  laser_scan_inverter executable (flips the signs in an inverted laser scan such that it can be used by laser_scan_matcher etc.) # # # 
add_executable(laser_scan_inverter src/spencer_bagfile_tools/laser_scan_inverter.cpp)
target_link_libraries(laser_scan_inverter ${catkin_LIBRARIES})


# # #  pose_to_odometry executable (publishes a PoseWithCovariance topic as nav_msgs/Odometry) # # # 
add_executable(pose_to_odometry src/spencer_bagfile_tools/pose_to_odometry.cpp)
target_link_libraries(pose_to_odometry ${catkin_LIBRARIES})


# # #  publish_camera_info executable (loads calibration YAML file from disk and publishes it at given rate) # # # 
add_executable(publish_camera_info src/spencer_bagfile_tools/publish_camera_info.cpp)
target_link_libraries(publish_camera_info ${catkin_LIBRARIES})


install(PROGRAMS
	scripts/create_thumbnails.py
	scripts/dump_encoder_values.py
	scripts/dump_tf_frames.py
	scripts/dump_timestamps.py
	scripts/interpolate_odometry.py
	scripts/odom_to_tf.py
	scripts/play.py
	scripts/plot_tf.py
	scripts/postprocess_bagfile.py
	scripts/reconstruct_odometry.py
	scripts/replace_frames.py
	scripts/shift_timestamps.py
	scripts/speedup_bagfile.py
	scripts/visualize_frustum.py
	scripts/visualize_plane.py
	scripts/tf_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS panorama_builder laser_scan_inverter pose_to_odometry publish_camera_info
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)



