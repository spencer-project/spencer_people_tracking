#### Introduction

This ROS package provides a script and launch files for playing back old
annotated people tracking logfiles in CARMEN format, such as those found here (Freiburg City Center/Aspekt and Main Station):

[Annotated 2D laser datasets (Social Robotics Labs, University of Freiburg)](http://www2.informatik.uni-freiburg.de/~luber/people_tracker/logfiles/logfiles.html)


#### Usage

The script publishes the following topics for each available sensor:

- `sensor_msgs/LaserScan` with the original laser scan data
- `srl_laser_segmentation/LaserscanSegmentation` with the annotations (each laser endpoint is labelled with an integer label, corresponding to the track ID)
- `sensor_msgs/Image`, if a camera was used during recording and the corresponding .bin file is present in the same directory

The `srl_laser_segmentation/LaserscanSegmentation` messages can be converted into `spencer_tracking_msgs/TrackedPersons` via the following command-line
(note you might have to rename the input and output topics properly, check `rostopic list` and `rosnode info` on the involved ROS nodes):

   rosrun srl_laser_segmentation segmentation_to_tracks.py

The resulting `TrackedPersons` messages can then, for instance, be used as a groundtruth input to `spencer_tracking_metrics`.


##### Credits

©2014-2015 Timm Linder, Social Robotics Laboratory, University of Freiburg
