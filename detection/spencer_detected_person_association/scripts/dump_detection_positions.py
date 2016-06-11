#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
Dumps positions of DetectedPersons into a CSV file for debugging
"""
import rospy, tf
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import PoseStamped


def newDetectedPersonsAvailable(detectedPersons):
    try:
        tfListener.waitForTransform(detectedPersons.header.frame_id, "odom", detectedPersons.header.stamp, rospy.Duration(0.05))
    except tf.Exception:
        return

    for detectedPerson in detectedPersons.detections:
        poseStamped = PoseStamped()
        poseStamped.pose = detectedPerson.pose.pose
        poseStamped.header = detectedPersons.header

        transformedPoseStamped = tfListener.transformPose("odom", poseStamped)

        pos = transformedPoseStamped.pose.position
        csvFile.write("%f\t%f\t%f\t%f\n" % (pos.x, pos.y, pos.z, detectedPersons.header.stamp.to_sec()) )

    global firstMessageOK
    if not firstMessageOK:
        firstMessageOK = True
        rospy.loginfo("First detections have been received, transformed and written to file!")


if __name__ == '__main__':
    rospy.init_node("dump_detected_person_positions")

    global csvFile
    csvFile = open('detected_person_positions.txt','w')
    csvFile.write("ros_x\tros_y\tros_z\ttimestamp")

    global tfListener
    tfListener = tf.TransformListener()

    global firstMessageOK
    firstMessageOK = False

    detectedPersonsTopic = "detected_persons"
    detectedPersonsSubscriber = rospy.Subscriber(detectedPersonsTopic, DetectedPersons, newDetectedPersonsAvailable, queue_size=500)

    rospy.loginfo("Dumping detected persons on topic %s to CSV file!" % (detectedPersonsTopic))
    rospy.spin()

    csvFile.close()
