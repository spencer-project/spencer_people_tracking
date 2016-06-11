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

import rospy, numpy, message_filters, math, copy, tf, collections, sys
from srl_laser_segmentation.msg import LaserscanSegmentation
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson, DetectedPersons, DetectedPerson
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion


"""
Outputs spencer_tracking_msgs/TrackedPersons and spencer_tracking_msgs/DetectedPersons based upon a
srl_laser_segmentation/LaserscanSegmentation message and a sensor_msgs/Laserscan. This assumes that the segmentation
labels correspond to (groundtruth) person track IDs which are consistent over time. Does not work with arbitrary segmentation
output as from simple jump-distance clustering.
"""
class SegmentationToTrackConverter(object):
    def __init__(self):
        laserscanTopic = rospy.resolve_name("laser")
        segmentationTopic = rospy.resolve_name("laser_annotations")

        trackedPersonsTopic = rospy.resolve_name("tracked_persons")
        detectedPersonsTopic = rospy.resolve_name("detected_persons")

        self.trackedPersonsPublisher = rospy.Publisher(trackedPersonsTopic, TrackedPersons, queue_size=100000)
        self.detectedPersonsPublisher = rospy.Publisher(detectedPersonsTopic, DetectedPersons, queue_size=100000)

        self._detectionIdCounter = 0
        self._lastDataStamp = None
        self._firstTrackEncounterLookup = dict()
        self._previousCentroidLookup = dict()

        self.laserSubscriber = message_filters.Subscriber(laserscanTopic, LaserScan)
        self.segmentationSubscriber = message_filters.Subscriber(segmentationTopic, LaserscanSegmentation)

        self.timeSynchronizer = message_filters.TimeSynchronizer([self.laserSubscriber, self.segmentationSubscriber], 50)
        self.timeSynchronizer.registerCallback(self.newSegmentationReceived)

        rospy.loginfo("Publishing detected and tracked persons from laser scan segmentation (%s and %s) at %s and %s"
                      % (laserscanTopic, segmentationTopic, detectedPersonsTopic, trackedPersonsTopic) )

        rospy.spin()

    def newSegmentationReceived(self, laserscan, laserscanSegmentation):
        currentStamp = laserscanSegmentation.header.stamp
        pointCount = len(laserscan.ranges)
        cartesianCoordinates = []

        # Required for velocity calculations
        if self._lastDataStamp is None:
            self._lastDataStamp = laserscanSegmentation.header.stamp

        # Build lookup of cartesian coordinates per laser point
        for pointIndex in xrange(0, pointCount):
            cartesianCoordinates.append( self.calculateCartesianCoordinates(laserscan, pointIndex) )

        # For each labelled segment, create and append one TrackedPerson and DetectedPerson message
        trackedPersons = TrackedPersons(header=laserscanSegmentation.header)
        detectedPersons = DetectedPersons(header=laserscanSegmentation.header)
        for segment in laserscanSegmentation.segments:
            # Calculate centroid of tracked person
            centroid = numpy.array([0.0, 0.0, 0.0])
            for pointIndex in segment.measurement_indices:
                centroid += cartesianCoordinates[pointIndex]
            centroid /= float(len(segment.measurement_indices))

            # Lookup previous centroid (for velocity/twist calculation), assume zero velocity at track initialization
            if not segment.label in self._previousCentroidLookup:
                self._previousCentroidLookup[segment.label] = collections.deque()

            # Maintain centroid history
            centroidHistory = self._previousCentroidLookup[segment.label]
            while len(centroidHistory) > 20:
                centroidHistory.popleft()

            # Calculate average velocity over past few frames
            dt = 0
            velocity = accumulatedVelocity = numpy.array([0.0, 0.0, 0.0])

            if centroidHistory:
                previousCentroid = centroid
                previousStamp = currentStamp
                for historyStamp, historyCentroid in reversed(centroidHistory):
                    accumulatedVelocity += previousCentroid - historyCentroid
                    dt += abs((previousStamp - historyStamp).to_sec())
                    previousCentroid = historyCentroid
                    previousStamp = historyStamp

                velocity = accumulatedVelocity / dt

            centroidHistory.append( (currentStamp, centroid) )

            # Remember age of track
            if not segment.label in self._firstTrackEncounterLookup:
                self._firstTrackEncounterLookup[segment.label] = currentStamp

            # Initialize TrackedPerson message
            trackedPerson = TrackedPerson()

            trackedPerson.track_id = segment.label
            trackedPerson.age = currentStamp - self._firstTrackEncounterLookup[segment.label]
            trackedPerson.detection_id = self._detectionIdCounter
            trackedPerson.is_occluded = False

            # Set position
            LARGE_VARIANCE = 99999999
            trackedPerson.pose.pose.position.x = centroid[0]
            trackedPerson.pose.pose.position.y = centroid[1]
            trackedPerson.pose.pose.position.z = centroid[2]

            # Set orientation
            if dt > 0:
                yaw = math.atan2(velocity[1], velocity[0])
                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                trackedPerson.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

            trackedPerson.pose.covariance[2 * 6 + 2] = trackedPerson.pose.covariance[3 * 6 + 3] = trackedPerson.pose.covariance[4 * 6 + 4] = LARGE_VARIANCE  # z pos, roll, pitch

            # Set velocity
            if dt > 0:
                trackedPerson.twist.twist.linear.x = velocity[0]
                trackedPerson.twist.twist.linear.y = velocity[1]
                trackedPerson.twist.twist.linear.z = velocity[2]

            trackedPerson.twist.covariance[2 * 6 + 2] = trackedPerson.twist.covariance[3 * 6 + 3] = trackedPerson.twist.covariance[4 * 6 + 4] = trackedPerson.twist.covariance[5 * 6 + 5] = LARGE_VARIANCE  # linear z, angular x, y, z

            # Append to list of tracked persons
            trackedPersons.tracks.append(trackedPerson)

            # Initialize DetectedPerson message by copying data from TrackedPerson
            detectedPerson = DetectedPerson()
            detectedPerson.detection_id = trackedPerson.detection_id
            detectedPerson.confidence = 1.0
            detectedPerson.pose = copy.deepcopy(trackedPerson.pose)
            detectedPerson.pose.pose.orientation = Quaternion()
            for i in xrange(0, 2):
                detectedPerson.pose.covariance[i * 6 + i] = 0.17 * 0.17
            detectedPerson.pose.covariance[5 * 6 + 5] = LARGE_VARIANCE  # yaw

            detectedPersons.detections.append(detectedPerson)
            self._detectionIdCounter += 1

        # Publish tracked persons
        self.trackedPersonsPublisher.publish(trackedPersons)
        self.detectedPersonsPublisher.publish(detectedPersons)

        self._lastDataStamp = laserscanSegmentation.header.stamp

    def calculateCartesianCoordinates(self, laserscan, pointIndex):
        rho = laserscan.ranges[pointIndex]
        phi = laserscan.angle_min + laserscan.angle_increment * pointIndex + math.pi / 2.0
        x = math.sin(phi) * rho
        y = -math.cos(phi) * rho
        return numpy.array([x, y, 0])


if __name__ == '__main__':
    arguments = rospy.myargv()

    rospy.init_node("segmentation_to_tracks")
    segmentationToTrackConverter = SegmentationToTrackConverter()
    sys.exit(0)
