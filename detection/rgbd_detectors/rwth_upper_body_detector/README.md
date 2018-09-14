## Upper Body Detector
This package detects the upper bodies of persons using depth and colour images.

### Run
Parameters:
* `queue_size` _default = 20_: The synchronisation queue size
* `template_file` _default = ""_: The template file. Can be found in config.
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The estimated/fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The deteced upper bodies
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_image` _default = /upper_body_detector/image_: The resulting image showing the detections as a boundingbox
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `depth_image` _default = /depth/image_rect_: `camera_namespace` + `depth_image` = depth image topic
* `rgb_image` _default = /rgb/image_rect_color_: `camera_namespace` + `rgb_image` = rgb image topic
* `camera_info_depth` _default = /depth/camera_info_: `camera_namespace` + `camera_info_depth` = depth camera info topic
* `load_params_from_file` _default = true_: `false` tries to read parameters from datacentre, `true` reads parameters from YAML file specified by `config_file`
* `config_file` _default = $(find upper_body_detector)/config/upper_body_detector.yaml_: The config file containing all the essential parameters. Only used if `load_params_from_file == true`.
* `template_file` _default = $(find upper_body_detector)/config/upper_body_template.yaml_: The upper body template file. Read from the database if `load_params_from_file == true`.
* `machine` _default = localhost_: Determines on which machine this node should run.
* `user` _default = ""_: The user used for the ssh connection if machine is not localhost.


rosrun:
```
rosrun rwth_upper_body_detector upper_body_detector [_parameter_name:=value]
```

roslaunch:
```
roslaunch rwth_upper_body_detector upper_body_detector.launch [parameter_name:=value]
```

### Credits

This package is derived from:
[strands-project/strands_perception_people/upper_body_detector](https://github.com/strands-project/strands_perception_people/tree/indigo-devel/upper_body_detector)

Original author: Dennis Mitzel, Computer Vision Group, RWTH Aachen  
ROS integration: Christian Dondrup (@cdondrup), LCAS, University of Lincoln  
SPENCER integration: Stefan Breuers, Computer Vision Group, RWTH Aachen  


