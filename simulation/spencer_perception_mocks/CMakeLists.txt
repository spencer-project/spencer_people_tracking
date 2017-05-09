cmake_minimum_required(VERSION 2.8.3)
project(spencer_perception_mocks)


set(PACKAGE_DEPENDENCIES
    spencer_tracking_msgs 
)

find_package(catkin REQUIRED COMPONENTS
  rospy roscpp roslib ${PACKAGE_DEPENDENCIES}
)

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})


install(PROGRAMS
	scripts/detected_and_tracked_persons.py
	scripts/driving_robot.py
	scripts/human_attributes.py
	scripts/social_activities.py
	scripts/social_relations.py
	scripts/spokespersons.py
	scripts/tracked_groups.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
