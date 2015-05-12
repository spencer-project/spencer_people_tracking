#!/usr/bin/env python
"""
Converts spencer_tracking_msgs/DetectedPersons into a geometry_msgs/PoseArray. This conversion is lossy,
as covariance matrix, modality, detection ID and confidence are omitted during the conversion.
"""

import rospy
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import PoseArray, Pose

def newMessageReceived(detectedPersons):
    poseArray = PoseArray()
    poseArray.header = detectedPersons.header

    for detectedPerson in detectedPersons.detections:
        poseArray.poses.append(detectedPerson.pose.pose)

    pub.publish(poseArray)


# Initialize node
rospy.init_node("detected_persons_to_pose_array")

# Create publisher and subscriber
inputTopic = rospy.resolve_name("/spencer/perception/detected_persons")
outputTopic = rospy.resolve_name("/pose_array")

sub = rospy.Subscriber(inputTopic, DetectedPersons, newMessageReceived, queue_size=5)
pub = rospy.Publisher(outputTopic, PoseArray, queue_size=5)
    
rospy.loginfo("Re-publishing spencer_tracking_msgs/DetectedPersons from %s as geometry_msgs/PoseArray at %s" % (inputTopic, outputTopic) )
rospy.spin()