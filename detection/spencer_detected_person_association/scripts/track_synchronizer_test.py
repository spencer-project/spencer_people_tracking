#!/usr/bin/env python
# -*- coding: utf-8 -*-
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