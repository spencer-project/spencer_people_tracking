#!/bin/bash

source ~/Code/spencer-workspace/devel/setup.bash

for i in {1..3}; do 
    rosrun srl_nearest_neighbor_tracker schiphol03_nnt.sh;
    rosrun srl_nearest_neighbor_tracker schiphol03_strands.sh;
    rosrun srl_nearest_neighbor_tracker schiphol03_mht.sh;
    rosrun srl_nearest_neighbor_tracker schiphol03_mdl.sh;
done