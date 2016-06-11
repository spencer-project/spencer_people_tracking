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
