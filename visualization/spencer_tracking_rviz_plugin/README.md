#### RViz plugins for visualization of detected and tracked persons, groups, social relations, activities and human attributes

This ROS package adds the following display types to RViz:
- Detected persons (spencer_tracking_msgs/DetectedPersons)
- Tracked persons (spencer_tracking_msgs/TrackedPersons)
- Tracked groups (spencer_tracking_msgs/TrackedGroups)
- Social relations (spencer_social_relation_msgs/SocialRelations)
- Social activities (spencer_social_relation_msgs/SocialActivities)
- Human attributes (spencer_human_attribute_msgs/HumanAttributes)

These message types have been defined within the consortium of the SPENCER FP-7 European Research Project (http://www.spencer.eu).


##### Installation instructions

Just check out the package into the "src" folder of your ROS catkin workspace, run "catkin_make", make sure you have run
"source devel/setup.sh", and then run RViz using "rosrun rviz rviz". The plugin will be discovered automatically.

##### Usage

Add displays for detected and tracked persons, groups or human attributes by clicking on "Add display" in the "Displays"
panel in RViz. Adjust the topic name if necessary, to point to the correct topic.

The displays for groups, social relations, and human attributes require two topics to be set in order to function, with
the second topic being the tracked persons topic.

##### Screenshots


###### DetectedPersons display

![DetectedPersons display](screenshots/detected_persons.png?raw=true)

###### TrackedPersons display

![TrackedPersons display](screenshots/tracked_persons_2.png?raw=true)

###### TrackedGroups display

![TrackedGroups display](screenshots/tracked_groups.png?raw=true)

###### SocialRelations display

![SocialRelations display](screenshots/social_relations.png?raw=true)


##### Known issues

- RViz may crash with a segfault when exiting. This usually can be ignored.
- RViz may crash when loading a configuration file. To avoid this, load the configuration directly at startup using the `-d CONFIG_FILENAME` parameter:

  `rviz -d test_visualization.rviz`

- Person visuals in the tracked groups and human attributes display do not support walking animations. However, these displays can be combined with the tracked persons display (which does support animations) by disabling their person visuals.


##### Credits

Author: ©2013-2015 Timm Linder, Social Robotics Laboratory, Albert-Ludwigs-University Freiburg, Germany
E-mail: linder(-at-)cs.uni-freiburg.de
