## Upper Body Detector
This package detects the upper bodies of persons using depth and colour images.

### Run
Parameters:
* `queue_size` _default = 20_: The synchronisation queue size
* `config_file` _default = ""_: The global config file. Can be found in rwth_upper_bodydetector/config
* `template_file` _default = ""_: The template file. Can be found in config.
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The estimated/fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The deteced upper bodies
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_image` _default = /upper_body_detector/image_: The resulting image showing the detections as a boundingbox


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


