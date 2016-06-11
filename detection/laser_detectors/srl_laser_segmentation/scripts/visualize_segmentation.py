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

import rospy, numpy, message_filters, math, struct
from srl_laser_segmentation.msg import LaserscanSegmentation
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

"""
Visualizes a srl_laser_segmentation/LaserscanSegmentation message and a sensor_msgs/Laserscan as a
colored sensor_msgs/PointCloud2 (where the colors indicate different segments), and a visualization_msgs/MarkerArray
(where text markers centered over the segments show label IDs).
"""
class LaserscanSegmentationVisualizer(object):
    def __init__(self):
        laserscanTopic = rospy.resolve_name("laser")
        segmentationTopic = rospy.resolve_name("laser_segmentation")
        cloudTopic = segmentationTopic + "_cloud"
        markersTopic = segmentationTopic + "_markers"

        ### Configurable parameters ##########
        self.unlabelledColor = rospy.get_param("~unlabelled_color", [0.7, 0.7, 0.7])
        self.fontScale = rospy.get_param("~font_scale", 1.0)
        self.minDistanceToSensor = rospy.get_param("~min_distance_to_sensor", 0.0)  # to filter out wrong echoes close to the sensor
        ### End of configurable parameters ###

        self._lastMarkerCount = 0

        self.cloudPublisher = rospy.Publisher(cloudTopic, PointCloud2, queue_size=3)
        self.markerArrayPublisher = rospy.Publisher(markersTopic, MarkerArray, queue_size=3)

        laserSubscriber = message_filters.Subscriber(laserscanTopic, LaserScan, queue_size=3)
        segmentationSubscriber = message_filters.Subscriber(segmentationTopic, LaserscanSegmentation, queue_size=3)

        self.timeSynchronizer = message_filters.TimeSynchronizer([laserSubscriber, segmentationSubscriber], 20)
        self.timeSynchronizer.registerCallback(self.newSegmentationReceived)

        rospy.loginfo("Visualizing laser scan segmentation (for laser scans published at %s) at %s on topics %s and %s"
            % (laserscanTopic, segmentationTopic, cloudTopic, markersTopic) )

        rospy.spin()

    def newSegmentationReceived(self, laserscan, laserscanSegmentation):
        pointCount = len(laserscan.ranges)
        labelsOfPoint = []
        cartesianCoordinates = []
        centroidsOfLabel = dict()
        numIgnoredPoints = 0

        for pointIndex in xrange(0, pointCount):
            labelsOfPoint.append( set() )
            cartesianCoordinates.append( self.calculateCartesianCoordinates(laserscan, pointIndex) )
            numIgnoredPoints += 1 if laserscan.ranges[pointIndex] < self.minDistanceToSensor else 0

        for segment in laserscanSegmentation.segments:
            centroid = numpy.array([0.0, 0.0, 0.0])
            for pointIndex in segment.measurement_indices:
                labelsOfPoint[pointIndex].add(segment.label)
                centroid += cartesianCoordinates[pointIndex]

            centroid /= float(len(segment.measurement_indices))
            centroidsOfLabel[segment.label] = centroid

        header = laserscan.header
        cloud = PointCloud2(header=header, height=1, width=pointCount-numIgnoredPoints, point_step=16, row_step=16*(pointCount-numIgnoredPoints)) # or step 32?
        cloud.fields.append( PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1) )
        cloud.fields.append( PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1) )
        cloud.fields.append( PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1) )
        cloud.fields.append( PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1) ) # or offset 16?
        markerArray = MarkerArray()

        for pointIndex in xrange(0, pointCount):
            # Skip wrong echoes very close to sensor
            if laserscan.ranges[pointIndex] < self.minDistanceToSensor:
                continue

            # If one point has multiple labels, we just linearly interpolate between the corresponding label colors
            colors = []
            for label in labelsOfPoint[pointIndex]:
                colors.append( self.lookupColorForLabel(label) )

            if colors:
                resultingColor = numpy.array([0.0, 0.0, 0.0])
                for color in colors:
                    resultingColor += color
                resultingColor /= float(len(colors))
            else:
                resultingColor = numpy.array(self.unlabelledColor)

            # Add points to point cloud
            cloud.data += struct.pack('f', cartesianCoordinates[pointIndex][0])  # x
            cloud.data += struct.pack('f', cartesianCoordinates[pointIndex][1])  # y
            cloud.data += struct.pack('f', cartesianCoordinates[pointIndex][2])  # z

            cloud.data += chr(int(resultingColor[2] * 255))  # r
            cloud.data += chr(int(resultingColor[1] * 255))  # g
            cloud.data += chr(int(resultingColor[0] * 255))  # b
            cloud.data += chr(0) # a

        # Add text markers and line strip markers to marker array
        for segment in laserscanSegmentation.segments:
            centroid = centroidsOfLabel[segment.label]
            color = self.lookupColorForLabel(segment.label)

            # Ignore segments very close to sensor, caused by wrong echoes due to robot self-reflection etc.
            if math.hypot(centroid[0], centroid[1]) < self.minDistanceToSensor:
                continue

            textMarker = Marker(header=header)
            textMarker.type = Marker.TEXT_VIEW_FACING
            textMarker.id = len(markerArray.markers)
            textMarker.text = "%d" % segment.label
            textMarker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1)
            textMarker.scale.z = 0.6  * self.fontScale
            textMarker.pose.position.x = centroid[0] + 0.4  # for readability
            textMarker.pose.position.y = centroid[1]
            textMarker.pose.position.z = centroid[2]

            markerArray.markers.append(textMarker)

            if len(segment.measurement_indices) > 1:
                lineStripMarker = Marker(header=header)
                lineStripMarker.type = Marker.LINE_STRIP
                lineStripMarker.id = len(markerArray.markers)
                lineStripMarker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.7)
                lineStripMarker.scale.x = 0.02

                for pointIndex in segment.measurement_indices:
                    lineStripMarker.points.append( Point(x=cartesianCoordinates[pointIndex][0], y=cartesianCoordinates[pointIndex][1], z=0 ) )

                markerArray.markers.append(lineStripMarker)

        # Delete old markers which are not needed any more
        currentMarkerCount = len(markerArray.markers)  # must be before creating delete markers
        for markerId in xrange(len(markerArray.markers), self._lastMarkerCount):
            deleteMarker = Marker(header=header)
            deleteMarker.id = markerId
            deleteMarker.action = Marker.DELETE
            markerArray.markers.append(deleteMarker)

        self._lastMarkerCount = currentMarkerCount
        self.markerArrayPublisher.publish(markerArray)
        self.cloudPublisher.publish(cloud)

    def lookupColorForLabel(self, label):
        palette = [0xaf1f90, 0x000846, 0x00468a, 0x00953d, 0xb2c908, 0xfcd22a, 0xffa800, 0xff4500, 0xe0000b, 0xb22222]
        color = palette[label % len(palette)]
        r = (color & 0xff0000) >> 16
        g = (color & 0x00ff00) >> 8
        b = (color & 0x0000ff) >> 0
        return numpy.array([r / 255.0, g / 255.0, b / 255.0])

    def calculateCartesianCoordinates(self, laserscan, pointIndex):
        rho = laserscan.ranges[pointIndex]
        phi = laserscan.angle_min + laserscan.angle_increment * pointIndex + math.pi / 2.0
        x = math.sin(phi) * rho
        y = -math.cos(phi) * rho
        return numpy.array([x, y, 0])


if __name__ == '__main__':
    arguments = rospy.myargv()

    rospy.init_node("visualize_segmentation")
    laserscanSegmentationVisualizer = LaserscanSegmentationVisualizer()
