#!/usr/bin/env python

# Software License Agreement (BSD License)
# 
# Copyright (c) 2014, Timm Linder, Social Robotics Lab, University of Freiburg
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Allows to reconstruct odometry from /additional_odom_data topic, using
# new calibration factors, as well as plotting the overall path that was travelled.
# Requires a bag file to be played back using rosbag play / rqt_bag.

import rospy, math, numpy, tf
from collections import deque
from spencer_bagfile_tools.msg import AdditionalOdometryData
from dynamic_reconfigure.server import Server
from spencer_bagfile_tools.cfg import ReconstructOdometryConfig

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry


class State(object):
    def __init__(self):
        self.x = self.y = self.theta = 0
        self.totalDistance = 0
        self.stamp = rospy.Time(0)

class OdometryController(object):
    def __init__(self):
        self.msgHistory = []
        self.stateHistory = self.emptyStateHistory()
        self.previousMsg = self.previousState = None
        self.rebuildingEntirePath = False

        self.zeroPosition()

        self.WHEEL_BASE = 0.665
        self.TICKS_PER_METER_LEFT = 56263.5 
        self.TICKS_PER_METER_RIGHT = 57099.7

        self.previousTimestampMarkerCount = 0

    def zeroPosition(self):
        self.stateHistory.append(State())   
        self.previousState = self.stateHistory[0]    

    def run(self):
        self.markerArrayPublisher = rospy.Publisher("/spencer_bagfile_tools/reconstructed_odom_path", MarkerArray, queue_size=1)
        self.odomPublisher = rospy.Publisher("/spencer/sensors/odom", Odometry, queue_size=3)

        reconfigureServer = Server(ReconstructOdometryConfig, self.reconfigure)

        topicName = "/spencer/sensors/additional_odom_data"
        self.subscriber = rospy.Subscriber(topicName, AdditionalOdometryData, self.additionalOdometryDataCallback)
        
        rospy.loginfo("Reconstructing odometry from " + topicName + ", now listening for messages...")
        rospy.spin()

    def additionalOdometryDataCallback(self, msg):
        if not self.rebuildingEntirePath:
            self.updateState(msg)

        self.msgHistory.append(msg)
        self.publishOdom()
        self.visualizePath()

    def reconfigure(self, config, level):
        self.extraCalibOverallMultiplier = config["extra_calib_overall_multiplier"]
        self.extraCalibLeftMultiplier = config["extra_calib_left_multiplier"]
        self.lineWidth = config["line_width"]
        self.arrowLength = config["arrow_length"]
        self.showWaypoints = config["show_waypoints"]
        self.recalculatePath = config["recalculate_path"]

        if level > 0 and self.recalculatePath:
            self.rebuildEntirePath()

        return config

    def emptyStateHistory(self):
        # Limit max. state history length to prevent bad performance after driving for a while
        # NOTE: msgHistory might still grow unboundedly, but there's no way of avoiding that...
        # However, that is mainly a memory issue as the whole history is only processed in rebuildEntirePath()
        return deque(maxlen=5000)

    def rebuildEntirePath(self):
        rospy.loginfo("Odometry parameters have changed! Rebuilding entire path!")
        if self.rebuildingEntirePath:
            return            
        self.rebuildingEntirePath = True

        self.stateHistory = self.emptyStateHistory()
        self.zeroPosition()

        self.previousMsg = None
        for msg in self.msgHistory:
            self.updateState(msg)

        self.rebuildingEntirePath = False
        self.publishOdom()
        self.visualizePath()
        
    def updateState(self, msg):
        newState = State()
        newState.stamp = msg.header.stamp

        previousLeftTicks = self.previousMsg.ticksLeft if self.previousMsg else msg.ticksLeft
        previousRightTicks = self.previousMsg.ticksRight if self.previousMsg else msg.ticksRight

        leftDiff = msg.ticksLeft - previousLeftTicks
        rightDiff = msg.ticksRight - previousRightTicks

        # Calculate metric travelled distances of both wheels and the base
        metersTravelledLeft = leftDiff * msg.calibOverallMultiplier * self.extraCalibOverallMultiplier * msg.calibLeftEncMultiplier * self.extraCalibLeftMultiplier / self.TICKS_PER_METER_LEFT
        metersTravelledRight = rightDiff * msg.calibOverallMultiplier * self.extraCalibOverallMultiplier / self.TICKS_PER_METER_RIGHT
        distance = (metersTravelledLeft + metersTravelledRight) / 2.0

        # Update position and bearing
        newState.theta = self.previousState.theta + (metersTravelledLeft - metersTravelledRight) / self.WHEEL_BASE
        newState.theta -= (int((newState.theta/(2*math.pi) ))) * 2*math.pi  # clip to 2pi

        newState.totalDistance = self.previousState.totalDistance + math.fabs(distance)

        newState.x = self.previousState.x + distance * math.sin(newState.theta)
        newState.y = self.previousState.y + distance * math.cos(newState.theta)

        positionTolerance = 0.1 # in meters
        if math.hypot(newState.x - self.stateHistory[-1].x, newState.y - self.stateHistory[-1].y) > positionTolerance:
            # Do not cache every single state if the change in position is minimal, otherwise we'll soon run
            # out of memory (note we still store previousState, since it is needed by publishOdom() and updateState())
            self.stateHistory.append(newState)

        self.previousState = newState # FIXME
        self.previousMsg = msg

    def publishOdom(self):
        odom = Odometry()
        odom.header.stamp = self.previousMsg.header.stamp if self.previousMsg else rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.previousState.x
        odom.pose.pose.position.y = self.previousState.y

        for row in xrange(0, 6):
            for col in xrange(0, 6):
                odom.pose.covariance[6*row+col] = 0 if row != col else 0.1
                odom.twist.covariance[6*row+col] = 0 if row != col else 999999

        q = tf.transformations.quaternion_from_euler(0, 0, -self.previousState.theta + math.pi/2)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        if len(self.stateHistory) >= 2:
            odom.twist.twist.linear.x = odom.pose.pose.position.x - self.stateHistory[-2].x
            odom.twist.twist.linear.y = odom.pose.pose.position.y - self.stateHistory[-2].y

        self.odomPublisher.publish(odom)

    def visualizePath(self):
        if self.markerArrayPublisher.get_num_connections() <= 0:
            return

        markerArray = MarkerArray()

        pathMarker = Marker()
        pathMarker.header.stamp = rospy.Time.now()
        pathMarker.header.frame_id = "odom"
        pathMarker.ns = "Path"
        pathMarker.type = Marker.LINE_STRIP
        pathMarker.id = 0
        pathMarker.color = ColorRGBA(r=1, g=1, a=1)
        pathMarker.scale.x = 0.05 * self.lineWidth

        waypointMarker = Marker()
        waypointMarker.header = pathMarker.header
        waypointMarker.ns = "Waypoints"
        waypointMarker.type = Marker.SPHERE_LIST
        waypointMarker.id = 1
        waypointMarker.color = ColorRGBA(r=1, g=1, a=1)
        waypointMarker.scale.x = waypointMarker.scale.y = 0.1 * self.lineWidth

        lastWaypointTime = float("-inf")
        lastWaypointPos = (float("99999"), float("99999"))

        # Generate path and waypoints
        for state in self.stateHistory:
            pathMarker.points.append(Point(x=state.x, y=state.y))

            if state.stamp.to_sec() - lastWaypointTime > 5 and self.showWaypoints:
                dx = state.x - lastWaypointPos[0]
                dy = state.y - lastWaypointPos[1]
                if math.sqrt(dx*dx + dy*dy) > 1:
                    lastWaypointTime = state.stamp.to_sec()
                    lastWaypointPos = (state.x, state.y)
                    waypointMarker.points.append(Point(x=state.x, y=state.y))

                    timestampMarker = Marker()
                    timestampMarker.header = waypointMarker.header
                    timestampMarker.ns = "Timestamps"
                    timestampMarker.type = Marker.TEXT_VIEW_FACING
                    timestampMarker.id = 3 + len(markerArray.markers)
                    timestampMarker.color = ColorRGBA(r=0.6, a=1)
                    timestampMarker.scale.z = 0.1 * self.lineWidth
                    timestampMarker.pose.position.x = state.x
                    timestampMarker.pose.position.y = state.y
                    timestampMarker.text = "%.1f" % state.stamp.to_sec()
                    markerArray.markers.append(timestampMarker)

        # Delete old markers
        currentTimestampMarkerCount = len(markerArray.markers)
        for i in xrange(0, self.previousTimestampMarkerCount - currentTimestampMarkerCount):
            timestampMarker = Marker()
            timestampMarker.header = waypointMarker.header
            timestampMarker.ns = "Timestamps"
            timestampMarker.action = Marker.DELETE
            timestampMarker.id = 3 + currentTimestampMarkerCount + i
            markerArray.markers.append(timestampMarker)

        self.previousTimestampMarkerCount = currentTimestampMarkerCount

        # Velocity arrow
        velocitySmoothingNoPoints = 5
        if len(pathMarker.points) > velocitySmoothingNoPoints:
            arrowHeadMarker = Marker()
            arrowHeadMarker.header = pathMarker.header
            arrowHeadMarker.ns = "Path-ArrowHead"
            arrowHeadMarker.type = Marker.LINE_STRIP
            arrowHeadMarker.id = 2
            arrowHeadMarker.color = ColorRGBA(r=1, g=1, a=1)
            arrowHeadMarker.scale.x = arrowHeadMarker.scale.y = 0.1 * self.lineWidth

            pointTip = numpy.array([pathMarker.points[-1].x, pathMarker.points[-1].y])

            lastVelocity = numpy.array([pathMarker.points[-1].x - pathMarker.points[-velocitySmoothingNoPoints].x,
                                        pathMarker.points[-1].y - pathMarker.points[-velocitySmoothingNoPoints].y])

            speed = numpy.linalg.norm(lastVelocity)
            lastVelocity /= speed
            lastVelocity *= 0.3 * self.arrowLength

            steepnessAngle = numpy.interp(speed, [0.03, 0.3], [0, 75])
            pointLeft  = pointTip + self.rotateVector(lastVelocity,   90 + steepnessAngle )
            pointRight = pointTip + self.rotateVector(lastVelocity, -(90 + steepnessAngle) )

            arrowHeadMarker.points.append(Point(x=pointLeft[0], y=pointLeft[1]))
            arrowHeadMarker.points.append(Point(x=pointTip[0], y=pointTip[1]))
            arrowHeadMarker.points.append(Point(x=pointRight[0], y=pointRight[1]))

            markerArray.markers.append(arrowHeadMarker)


        markerArray.markers.append(pathMarker)
        markerArray.markers.append(waypointMarker)

        self.markerArrayPublisher.publish(markerArray)

    
    def rotateVector(self, vector, angleDeg):
        theta = (angleDeg/180.) * numpy.pi

        rotMatrix = numpy.array([[numpy.cos(theta), -numpy.sin(theta)], 
                                 [numpy.sin(theta),  numpy.cos(theta)]])

        return numpy.dot(rotMatrix, vector)


if __name__ == '__main__':  
    rospy.init_node("reconstruct_odometry")

    odometryController = OdometryController()
    odometryController.run()
