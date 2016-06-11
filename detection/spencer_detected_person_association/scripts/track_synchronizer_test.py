#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import collections, rospy, message_filters
from spencer_tracking_msgs.msg import DetectedPersons
from spencer_detected_person_association import TrackSynchronizer

def detectedPersonsCallback(trackAssociation, detectedPersons):
    age = rospy.Time.now() - detectedPersons.header.stamp
    output = "New detections with track association available (age of detections = %.2f sec)! Detection to track association: " % age.to_sec()

    if detectedPersons.detections:
        for detectedPerson in detectedPersons.detections:
            # The TrackSynchronizer invoking this callback guarantees that the detectedPersons message is buffered until a
            # track association is available for these detections (by comparing message timestamps of tracks and detections).
            detectionId = detectedPerson.detection_id
            trackId = trackAssociation.lookupTrackId(detectionId) # <-- this is what this is all about

            output += "\n[det %d --> track %s]" % (detectionId, str(trackId))
    else:
        output += "Empty set of detections!"

    rospy.loginfo(output)



if __name__ == '__main__':
    arguments = rospy.myargv()
    rospy.init_node("track_synchronizer_test")

    detectionsTopic = "/spencer/perception_internal/detected_persons/rgbd_front_top"
    detected_person_sub = message_filters.Subscriber(detectionsTopic, DetectedPersons)

    track_sync = TrackSynchronizer(detected_person_sub, 100)
    track_sync.registerCallback(detectedPersonsCallback)

    rospy.loginfo("Subscribed to %s and waiting for detection-to-track association!" % detectionsTopic)

    rospy.spin()
