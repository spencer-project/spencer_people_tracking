#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
