## Ground Plane
This package estimates the ground plane using depth images. 

It can also be used with a fixed ground plane which is just rotated according to the ptu tilt angle. In this case the optimal ground plane is assumed and represented by the normal vector [0,-1,0] and the distance of 1.7 (Distance of the origin of the camera coordinated system to the plane in meters). These values can be changed in the config/fixed_gp.yaml file. As you can see this assumes to be used with the head_xtion and is therefore exclusive to the robot but it also prevents failurs due to wrongly estimated planes in cases where the camera can't see the ground. All this was necessary because the camera on the head of the robot is rather high and therefore has a high risk of not seeing the actual ground plane. At the current state of development it is advised to use the fixed plane set-up when running it on the robot.

### Run
Parameters for estimation:
* `queue_size` _default = 5_: The synchronisation queue size
* `config_file` _default = ""_: The global config file. Can be found in rwth_ground_plane/config
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The estimated ground plane

Parameters for the fixed ground plane:
* `param_file` _default = ""_: The paramter file describing the fixed plane. Can be found in rwth_ground_plane/config/fixed_gp.yaml
* `ptu_state` _default = /ptu/state_: The current angles of the ptu
* `ground_plane` _default = /ground_plane_: The rotated ground plane

Parameters for the really fixed ground plane:
* `param_file` _default = ""_: The paramter file describing the fixed plane. Can be found in rwth_ground_plane/config/fixed_gp.yaml
* `ground_plane` _default = /ground_plane_: The rotated ground plane

rosrun:
```
rosrun rwth_ground_plane ground_plane_estimated [_parameter_name:=value]
```
or
```
rosrun rwth_ground_plane ground_plane_fixed [_parameter_name:=value]
```
or
```
rosrun rwth_ground_plane ground_plane_really_fixed [_parameter_name:=value]
```

roslaunch:
```
roslaunch rwth_ground_plane ground_plane_estimated.launch [parameter_name:=value]
```
or
```
roslaunch rwth_ground_plane ground_plane_fixed.launch [parameter_name:=value]
```
or
```
roslaunch rwth_ground_plane ground_plane_really_fixed.launch [parameter_name:=value]
```
