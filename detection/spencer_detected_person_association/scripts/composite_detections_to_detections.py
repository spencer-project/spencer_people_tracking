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
Republishes spencer_tracking_msgs/CompositeDetectedPersons as spencer_tracking_msgs/DetectedPersons for direct feeding into
e.g. a people tracker node which does not directly support the former message type.
"""
import rospy
from spencer_tracking_msgs.msg import CompositeDetectedPersons, DetectedPersons, DetectedPerson

def newCompositeDetectedPersonsAvailable(compositeDetectedPersons):
    detectedPersons = DetectedPersons()
    detectedPersons.header = compositeDetectedPersons.header

    for compositeDetectedPerson in compositeDetectedPersons.elements:
        detectedPerson = DetectedPerson()
        detectedPerson.detection_id = compositeDetectedPerson.composite_detection_id
        detectedPerson.confidence = compositeDetectedPerson.max_confidence
        detectedPerson.pose = compositeDetectedPerson.pose

        involvedModalities = list(set([originalDetection.modality for originalDetection in compositeDetectedPerson.original_detections]))
        detectedPerson.modality = ",".join(sorted(involvedModalities))

        detectedPersons.detections.append(detectedPerson)

    detectedPersonsPublisher.publish(detectedPersons)


if __name__ == '__main__':
    arguments = rospy.myargv()
    rospy.init_node("composite_detections_to_detections")

    compositeDetectedPersonsTopic = "/spencer/perception/detected_persons_composite"
    compositeDetectedPersonsSubscriber = rospy.Subscriber(compositeDetectedPersonsTopic, CompositeDetectedPersons, newCompositeDetectedPersonsAvailable, queue_size=5)

    global detectedPersonsPublisher
    detectedPersonsTopic = "/spencer/perception/detected_persons"
    detectedPersonsPublisher = rospy.Publisher(detectedPersonsTopic, DetectedPersons, queue_size=5)

    rospy.loginfo("Re-publishing composite detections from %s as plain detections on %s" % (compositeDetectedPersonsTopic, detectedPersonsTopic))
    rospy.spin()
