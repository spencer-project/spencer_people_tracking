#!/bin/bash

source ~/Code/spencer-workspace/devel/setup.bash

for i in {1..3}; do 
    rosrun srl_nearest_neighbor_tracker toulouse_nnt.sh;
    rosrun srl_nearest_neighbor_tracker toulouse_strands.sh;
    rosrun srl_nearest_neighbor_tracker toulouse_mht.sh;
    rosrun srl_nearest_neighbor_tracker toulouse_mdl.sh;
done