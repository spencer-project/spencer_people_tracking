#!/bin/bash

#source ~/spencer_ws/devel/setup.bash
source ~/Code/spencer-workspace/devel/setup.bash

# Front RGB-D only

roslaunch srl_nearest_neighbor_tracker mdl_toulouse_front_rgbd_only_baseline.launch
