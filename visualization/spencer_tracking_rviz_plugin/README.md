RVIZ PLUGIN FOR VISUALIZATION OF DETECTED AND TRACKED PERSONS, GROUPS, SOCIAL RELATIONS, ACTIVITIES AND HUMAN ATTRIBUTES
========================================================================================================================
Author: (C)2013-2014 Timm Linder, Social Robotics Laboratory, Albert-Ludwigs-University Freiburg, Germany

E-mail: linder@cs.uni-freiburg.de

Adds the following display types to Rviz:

- Detected persons (spencer_tracking_msgs/DetectedPersons)
- Tracked persons (spencer_tracking_msgs/TrackedPersons)
- Tracked groups (spencer_tracking_msgs/TrackedGroups)
- Social relations (spencer_social_relation_msgs/SocialRelations)
- Social activities (spencer_social_relation_msgs/SocialActivities)
- Human attributes (spencer_human_attribute_msgs/HumanAttributes)

These message types have been defined within the consortium of the SPENCER FP-7 European Research Project (http://www.spencer.eu).

See the "screenshots" folder for some example images of these displays.


Installation instructions
-------------------------
Just check out the package into the "src" folder of your ROS catkin workspace, run "catkin_make", make sure you have run
"source devel/setup.sh", and then run RViz using "rosrun rviz rviz". The plugin will be discovered automatically.

Usage
-----
Add displays for detected and tracked persons, groups or human attributes by clicking on "Add display" in the "Displays"
panel in RViz. Adjust the topic name if necessary, to point to the correct topic.

The displays for groups, social relations, and human attributes require two topic to be set in order to function,
the second topic being the tracked persons topic.

Known issues
------------
Rviz may crash with a segfault when exiting. This usually can be ignored.
Person visuals in the tracked groups and human attributes display do not support walking animations.
However, these displays can be combined with the tracked persons display (which does support animations) but disabling their person visuals.