#### Scripts for conversion to and from spencer_tracking_msgs/DetectedPersons

This package contains Python scripts for converting `geometry_msgs/PoseArray` or `people_msgs/PositionMeasurementArray` messages into `spencer_tracking_msgs/DetectedPersons`, and vice versa.
This can be useful to integrate external detectors, which output one of the aforementioned message types, into the SPENCER people tracking and visualization framework.

One such example is the [leg detector](https://github.com/wg-perception/people/tree/indigo-devel/leg_detector) from wg-perception.

The resulting `spencer_tracking_msgs/DetectedPersons` can then be processed by the [`srl_nearest_neighbor_tracker`](/tracking/people/srl_nearest_neighbor_tracker),
[`spencer_detected_person_association`](/detection/spencer_detected_person_association) or [`spencer_tracking_rviz_plugin`](/visualization/spencer_tracking_rviz_plugin) packages.

Note that a `geometry_msgs/PoseArray` only contains a subset of the information stored in a `spencer_tracking_msgs/DetectedPersons` message. Therefore,
the reverse conversion from `spencer_tracking_msgs/DetectedPersons` to `geometry_msgs/PoseArray` is lossy.



##### Credits

©2015 Timm Linder, Social Robotics Laboratory, University of Freiburg

