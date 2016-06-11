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

import rospy, numpy, math
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

"""
Visualizes a a sensor_msgs/Laserscan by rendering its visible area using polygons and a line contour.
The resulting visualization is published as a visualization_msgs/MarkerArray.
"""
class VisibleLaserscanAreaVisualizer(object):
    def __init__(self):
        laserscanTopic = rospy.resolve_name("laser")
        markersTopic = laserscanTopic + "_visible_area"

        ### Configurable parameters ##########
        self.fillColor = rospy.get_param("~fill_color", [0.7, 0.7, 0.7, 0.7])
        self.lineColor = rospy.get_param("~line_color", [0.3, 0.3, 0.3, 0.7])
        self.lineWidth = rospy.get_param("~line_width", 0.02)
        self.minDistanceToSensor = rospy.get_param("~min_distance_to_sensor", 0.0)  # to filter out wrong echoes close to the sensor
        ### End of configurable parameters ###

        self.markerArrayPublisher = rospy.Publisher(markersTopic, MarkerArray, queue_size=1)
        laserSubscriber = rospy.Subscriber(laserscanTopic, LaserScan, self.newLaserscanReceived, queue_size=1)

        rospy.loginfo("Visualizing visible area of laser scan at %s on topic %s"
            % (laserscanTopic, markersTopic) )

        rospy.spin()

    def newLaserscanReceived(self, laserscan):
        cartesianCoordinates = []

        for pointIndex in xrange(0, len(laserscan.ranges)):
            if laserscan.ranges[pointIndex] >= self.minDistanceToSensor:
                cartesianCoordinates.append( self.calculateCartesianCoordinates(laserscan, pointIndex) )
        pointCount = len(cartesianCoordinates)

        header = laserscan.header
        markerArray = MarkerArray()

        lineMarker = Marker(header=header)
        lineMarker.type = Marker.LINE_STRIP
        lineMarker.ns = "Contour"
        lineMarker.color = ColorRGBA(r=self.lineColor[0], g=self.lineColor[1], b=self.lineColor[2], a=self.lineColor[3])
        lineMarker.scale.x = self.lineWidth

        fillMarker = Marker(header=header)
        fillMarker.type = Marker.TRIANGLE_LIST
        fillMarker.ns = "Fill"
        fillMarker.scale.x = fillMarker.scale.y = fillMarker.scale.z = 1
        fillMarker.color = ColorRGBA(r=self.fillColor[0], g=self.fillColor[1], b=self.fillColor[2], a=self.fillColor[3])

        for pointIndex in xrange(0, pointCount):
            lineMarker.points.append( cartesianCoordinates[pointIndex] )

            if pointIndex < pointCount - 1:
                fillMarker.points.append( Point(x=0, y=0, z=0) )  # sensor origin
                fillMarker.points.append( cartesianCoordinates[pointIndex+1] )
                fillMarker.points.append( cartesianCoordinates[pointIndex] )
                #fillMarker.colors.append(fillMarker.color)
                #fillMarker.colors.append(fillMarker.color)
                #fillMarker.colors.append(fillMarker.color)

        markerArray.markers.append(lineMarker)
        markerArray.markers.append(fillMarker)

        self.markerArrayPublisher.publish(markerArray)

    def calculateCartesianCoordinates(self, laserscan, pointIndex):
        rho = laserscan.ranges[pointIndex]
        phi = laserscan.angle_min + laserscan.angle_increment * pointIndex + math.pi / 2.0
        x = math.sin(phi) * rho
        y = -math.cos(phi) * rho
        return Point(x=x, y=y, z=0)


if __name__ == '__main__':
    arguments = rospy.myargv()

    rospy.init_node("visualize_visible_laser_area")
    visibleLaserscanAreaVisualizer = VisibleLaserscanAreaVisualizer()
