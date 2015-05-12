### PoseArray conversion scripts

This package contains Python scripts for converting `geometry_msgs/PoseArray` messages into `spencer_tracking_msgs/DetectedPersons`, and vice versa.
This can be useful to integrate external detectors, which output a `geometry_msgs/PoseArray`, into the SPENCER people tracking and visualization framework.

One such example is the [Leg Detector](https://github.com/wg-perception/people/tree/indigo-devel/leg_detector) from wg-perception.

The resulting `spencer_tracking_msgs/DetectedPersons` can then be processed by the [`srl_nearest_neighbor_tracker`](/tracking/people/srl_nearest_neighbor_tracker),
[spencer_detected_person_association](/detection/spencer_detected_person_association) (for multi-modal composite detections) or [`spencer_tracking_rviz_plugin`](/visualization/spencer_tracking_rviz_plugin) packages.

Note that a `PoseArray` only contains a subset of the information stored in a `DetectedPersons` message. Therefore,
the conversion from `DetectedPersons` to `PoseArray` is lossy.



#### Credits

(C) 2015 Timm Linder, Social Robotics Laboratory, University of Freiburg

