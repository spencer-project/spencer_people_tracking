#!/bin/bash

source ~/Code/spencer-workspace/devel/setup.bash

# Laser only

roslaunch srl_nearest_neighbor_tracker mht_toulouse_laser_only_with_static_map.launch


# Multimodal

roslaunch srl_nearest_neighbor_tracker mht_toulouse_multimodal_baseline.launch

roslaunch srl_nearest_neighbor_tracker mht_toulouse_multimodal_with_static_map.launch


# Front RGB-D only

#roslaunch srl_nearest_neighbor_tracker mht_toulouse_front_rgbd_only_with_static_map.launch

mht_toulouse_front_rgbd_only_baseline.launch