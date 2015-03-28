#!/usr/bin/env python
"""
Visualizes spencer_tracking_msgs/CompositeDetectedPersons using visualization_msgs/MarkerArray. The markers
show which original spencer_tracking_msgs/DetectedPersons were fused together into which spencer_tracking_msgs/CompositeDetectedPerson.
"""
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from spencer_tracking_msgs.msg import CompositeDetectedPersons, DetectedPersons, DetectedPerson

def newCompositeDetectedPersonsAvailable(compositeDetectedPersons):
    global currentMarkerId, oldestActiveMarkerId
    markerArray = MarkerArray()

    markerZ = rospy.get_param("~marker_z", 2.0)
    markerScale = rospy.get_param("~marker_scale", 1.0)
    
    # Delete old markers
    for markerId in xrange(oldestActiveMarkerId, currentMarkerId):
        marker = Marker()
        marker.header = compositeDetectedPersons.header
        marker.id = markerId
        marker.action = Marker.DELETE
        markerArray.markers.append(marker)

    oldestActiveMarkerId = currentMarkerId

    # Create new markers
    for compositeDetectedPerson in compositeDetectedPersons.elements:
        if len(compositeDetectedPerson.original_detections) > 1:
            for originalDetectedPerson in compositeDetectedPerson.original_detections:
                marker = Marker()
                marker.header = compositeDetectedPersons.header
                marker.id = currentMarkerId
                marker.type = Marker.ARROW
                marker.color.r = marker.color.b = marker.color.a = 1.0
                marker.color.g = 0.3
                marker.points.append( Point(compositeDetectedPerson.pose.pose.position.x, compositeDetectedPerson.pose.pose.position.y, markerZ ) )
                marker.points.append( Point(originalDetectedPerson.pose.pose.position.x, originalDetectedPerson.pose.pose.position.y, markerZ ) )
                marker.scale.x = 0.01 * markerScale
                marker.scale.y = 0.05 * markerScale
                marker.scale.z = 0.05 * markerScale

                markerArray.markers.append(marker)
                currentMarkerId += 1

    markerArrayPublisher.publish(markerArray)


if __name__ == '__main__':
    arguments = rospy.myargv()
    rospy.init_node("composite_detections_to_detections")

    global currentMarkerId, oldestActiveMarkerId
    currentMarkerId = 0
    oldestActiveMarkerId = 0

    compositeDetectedPersonsTopic = "/spencer/perception/detected_persons_composite"
    compositeDetectedPersonsSubscriber = rospy.Subscriber(compositeDetectedPersonsTopic, CompositeDetectedPersons, newCompositeDetectedPersonsAvailable, queue_size=3)

    global markerArrayPublisher
    markerArrayTopic = "/spencer/perception_internal/detected_person_association/visualization/detected_persons_composite/marker_array"
    markerArrayPublisher = rospy.Publisher(markerArrayTopic, MarkerArray, queue_size=3)

    rospy.loginfo("Publishing visualization of composite detections from %s as marker array on %s" % (compositeDetectedPersonsTopic, markerArrayTopic))
    rospy.spin()