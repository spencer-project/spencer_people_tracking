#!/usr/bin/env python

"""
Converts a geometry_msgs/PoseArray into spencer_tracking_msgs/DetectedPersons which can be processed using a tracker, or visualized using
spencer_tracking_rviz_plugin. This conversion is lossless.
"""
import rospy
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import PoseArray, Pose

def newMessageReceived(poseArray):
    detectedPersons = DetectedPersons()
    detectedPersons.header = poseArray.header

    global detectionId, detectionIdIncrement

    for pose in poseArray.poses:
        detectedPerson = DetectedPerson()
        detectedPerson.modality = modality;
        detectedPerson.confidence = confidence
        detectedPerson.detection_id = detectionId
        detectedPerson.pose.pose = pose
        
        for i in xrange(0, 6):
            detectedPerson.pose.covariance[i*6 + i] = posVariance if i < 3 else rotVariance

        detectedPersons.detections.append(detectedPerson)
        detectionId += detectionIdIncrement

    pub.publish(detectedPersons)


# Initialize node
rospy.init_node("pose_array_to_detected_persons")

# Configurable parameters
detectionId = rospy.get_param("~detection_id_offset", 0)
detectionIdIncrement = rospy.get_param("~detection_id_increment", 1)
posVariance = rospy.get_param("~pos_variance", 0.2)
rotVariance = rospy.get_param("~rot_variance", 99999)
confidence = rospy.get_param("~confidence", 1.0)
modality = rospy.get_param("~modality", "unspecified")

# Create publisher and subscriber
inputTopic = rospy.resolve_name("/pose_array")
outputTopic = rospy.resolve_name("/spencer/perception/detected_persons")
sub = rospy.Subscriber(inputTopic, PoseArray, newMessageReceived, queue_size=5)
pub = rospy.Publisher(outputTopic, DetectedPersons, queue_size=5)

rospy.loginfo("Re-publishing geometry_msgs/PoseArray from %s as spencer_tracking_msgs/DetectedPersons at %s, with a positional / rotational variance of %.2f / %.2f and modality '%s', confidence %.2f, ID increment %d and offset %d"
    % (inputTopic, outputTopic, posVariance, rotVariance, modality, confidence, detectionIdIncrement, detectionId) )
rospy.spin()