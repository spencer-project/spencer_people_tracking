#!/usr/bin/env python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Simulates a driving robot and publishes the corresponding odom --> base_footprint transform on /tf
# and odometry on /spencer/sensors/odom.

import rospy, tf, numpy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from math import sin, cos, atan2

rospy.init_node( 'mock_driving_robot' )

br = tf.TransformBroadcaster()
odomPublisher = rospy.Publisher("/spencer/sensors/odom", Odometry)

# State variables
updateRateHz = 30

rospy.loginfo("Publishing mock odom --> base_footprint transform on /tf and odometry on /odom")
rate = rospy.Rate(updateRateHz)

frameId = rospy.get_param("frame", "base_footprint")
parentFrameId = rospy.get_param("parent_frame", "odom")

lastPosition = None
while not rospy.is_shutdown():
    a = 5
    xOffset = 3
    yOffset = 2
    robotSpeed = 0.3

    currentTime = rospy.Time.now()
    t = currentTime.to_sec() * robotSpeed

    # Calculate position (robot drives an eight-figure, http://mathworld.wolfram.com/EightCurve.html)
    robotPosition = a*sin(t) + xOffset, a*sin(t)*cos(t) + yOffset, 0
    
    # Calculate velocity based upon last position
    robotVelocity = (0, 0, 0)
    if lastPosition is not None:
        robotVelocity = (numpy.array(robotPosition) - numpy.array(lastPosition)) * updateRateHz
    lastPosition = robotPosition

    # Calculate orientation from velocity
    robotYaw = atan2(robotVelocity[1], robotVelocity[0])
    robotOrientation = tf.transformations.quaternion_from_euler(0,  0,  robotYaw)

    # Send tf transform
    br.sendTransform(robotPosition, robotOrientation, currentTime, frameId, parentFrameId)

    # Initialize odometry message
    odom = Odometry()
    odom.header.stamp = currentTime
    odom.header.frame_id = parentFrameId
    odom.child_frame_id = frameId

    odom.pose.pose.position = Point(*robotPosition)
    odom.pose.pose.orientation = Quaternion(*robotOrientation)
    odom.twist.twist.linear = Point(*robotVelocity)

    # TODO: Set pose / twist covariances


    # Publish odometry message
    odomPublisher.publish(odom)

    rate.sleep()
