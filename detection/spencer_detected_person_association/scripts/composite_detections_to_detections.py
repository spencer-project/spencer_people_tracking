#!/usr/bin/env python
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