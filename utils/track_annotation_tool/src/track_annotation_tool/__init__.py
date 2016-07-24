# Software License Agreement (BSD License)
#
# Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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

import rospy, numpy, yaml, time, tf, math, copy
from threading import Thread, RLock

from visualization_msgs.msg import MarkerArray, Marker
from spencer_tracking_msgs.msg import DetectedPersons, TrackedPersons, TrackedPerson
from spencer_human_attribute_msgs.msg import HumanAttributes
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, PointStamped, Quaternion, PoseStamped


# Methods for playing back tracks in the track database
class Simulator(object):
    def __init__(self, database, editor, transformer):
        rospy.loginfo("Track simulator starting up...")

        self.trackingFrame = rospy.get_param("~tracking_frame", "odom")  # could also use "odom" here as a fixed frame

        self.database = database
        self.transformer = transformer
        self.editor = editor
        self.clockGenerator = ClockGenerator()

        self.detectedPersonSubscriber  = rospy.Subscriber("/spencer/perception/detected_persons", DetectedPersons, self.newDetectedPersonsReceived)
        self.trackedPersonPublisher  = rospy.Publisher("/spencer/perception/tracked_persons", TrackedPersons, queue_size = 10)
        self.trackedPersonWithoutActivePublisher  = rospy.Publisher("/track_annotation_tool/tracked_person_without_active", TrackedPersons, queue_size = 10)
        self.activeTrackedPersonPublisher  = rospy.Publisher("/track_annotation_tool/active_tracked_person", TrackedPersons, queue_size = 10)
        self.humanAttributePublisher = rospy.Publisher("/spencer/perception/human_attributes", HumanAttributes, queue_size = 10)

        self.synchronizeWithDetections = False

    def update(self):
        if not self.synchronizeWithDetections:
            with self.database.lock:
                trackedPersons  = self.createTrackedPersons(rospy.Time.now())
                humanAttributes = self.createHumanAttributes()
                self.publish(trackedPersons, humanAttributes)

    def newDetectedPersonsReceived(self, detectedPersons):
        if self.synchronizeWithDetections:
            with self.database.lock:
                trackedPersons = self.createTrackedPersons(detectedPersons.header.stamp)
                humanAttributes = self.createHumanAttributes()
                self.associateTracksWithDetections(trackedPersons, detectedPersons)
                self.publish(trackedPersons, humanAttributes)

    def publish(self, trackedPersons, humanAttributes):
        humanAttributes.header = trackedPersons.header
        self.trackedPersonPublisher.publish(trackedPersons)
        self.humanAttributePublisher.publish(humanAttributes)

        # Separately publish the currently active track in the editor, if any
        if self.trackedPersonWithoutActivePublisher.get_num_connections() > 0 or self.activeTrackedPersonPublisher.get_num_connections() > 0:
            activeTrackedPersons = TrackedPersons()
            activeTrackedPersons.header = trackedPersons.header

            trackedPersonsWithoutActive = TrackedPersons()
            trackedPersonsWithoutActive.header = trackedPersons.header

            for trackedPerson in trackedPersons.tracks:
                if self.editor is None or trackedPerson.track_id != self.editor.activeTrackId:
                    trackedPersonsWithoutActive.tracks.append(trackedPerson)
                else:
                    activeTrackedPersons.tracks.append(trackedPerson)

            self.activeTrackedPersonPublisher.publish(activeTrackedPersons)
            self.trackedPersonWithoutActivePublisher.publish(trackedPersonsWithoutActive)

    def createTrackedPersons(self, timestamp):
        trackedPersons = TrackedPersons()
        trackedPersons.header.stamp = timestamp
        trackedPersons.header.frame_id = self.trackingFrame

        for trackKey, track in self.database.getTracks().iteritems():
            waypoints = self.database.getWaypoints(track)

            if waypoints:
                firstWaypoint = waypoints[0]
                lastWaypoint  = waypoints[-1]

                # Check if we are before the first or past the last waypoint
                beforeFirstWaypoint = timestamp.to_sec() < firstWaypoint["timestamp"]
                behindLastWaypoint  = timestamp.to_sec() > lastWaypoint["timestamp"]

                if not beforeFirstWaypoint and not behindLastWaypoint:
                    trackedPerson = TrackedPerson()
                    trackedPerson.track_id = self.database.getTrackId(trackKey)
                    trackedPerson.is_occluded = False
                    trackedPerson.age = rospy.Time(firstWaypoint["timestamp"])

                    # Find adjacent waypoints for linear interpolation of tracked person's position
                    previousWaypoint = nextWaypoint = None
                    for waypoint in waypoints:
                        previousWaypoint = nextWaypoint
                        nextWaypoint = waypoint
                        if nextWaypoint["timestamp"] > timestamp.to_sec():
                            break

                    try:
                        # If there is only one waypoint for this track, previousWaypoint will be None
                        nextPosition = self.transformer.toCommonFrame(nextWaypoint, self.trackingFrame)
                        nextTimestamp = nextWaypoint["timestamp"]

                        # Determine waypoint segment duration and distance
                        if previousWaypoint is not None:
                            previousPosition = self.transformer.toCommonFrame(previousWaypoint, self.trackingFrame)
                            previousTimestamp = previousWaypoint["timestamp"]
                            segmentDuration = nextTimestamp - previousTimestamp
                            alpha = (timestamp.to_sec() - previousTimestamp) / segmentDuration
                            diffVector = nextPosition - previousPosition
                        else:
                            previousPosition = nextPosition
                            segmentDuration = 1.0
                            alpha = 0.0
                            diffVector = numpy.array([0, 0])

                    except tf.Exception:
                        continue

                    # Linear interpolation
                    interpolatedPosition = previousPosition + diffVector*alpha
                    velocity = diffVector / segmentDuration

                    trackedPerson.pose.pose.position.x, trackedPerson.pose.pose.position.y = interpolatedPosition
                    trackedPerson.pose.covariance[2 * 6 + 2] = trackedPerson.pose.covariance[3 * 6 + 3] = trackedPerson.pose.covariance[4 * 6 + 4] = 99999999

                    yaw = math.atan2(velocity[1], velocity[0])
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                    trackedPerson.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

                    trackedPerson.twist.twist.linear.x, trackedPerson.twist.twist.linear.y = velocity

                    trackedPersons.tracks.append(trackedPerson)

        return trackedPersons

    def createHumanAttributes(self):
        humanAttributes = HumanAttributes()
        return humanAttributes

    def associateTracksWithDetections(self, trackedPersons, detectedPersons):
        gatingDistance = 0.5
        alreadyAssignedDetectionIds = set()
        for trackedPerson in trackedPersons.tracks:
            trackPosition = numpy.array(trackedPerson.pose.pose.position)

            minDistance = float("inf")
            closestDetectedPerson = None

            for detectedPerson in detectedPersons.detections:
                if not detectedPerson.detection_id in alreadyAssignedDetectionIds:
                    poseStamped = PoseStamped(pose=detectedPerson.pose, header=detectedPersons.header)
                    transformedPose = self.transformer.tfListener.transformPose(trackedPersons.header.frame_id, poseStamped)
                    detectionPosition = numpy.array(transformedPose.pose.position)

                    distance = numpy.linalg.norm(trackPosition - detectionPosition)
                    if distance < gatingDistance and distance < minDistance:
                        minDistance = distance
                        closestDetectedPerson = detectedPerson

            if closestDetectedPerson is not None:
                trackedPerson.is_occluded = False
                trackedPerson.detection_id = detectedPerson.detection_id
                alreadyAssignedDetectionIds.add(detectedPerson.detection_id)
            else:
                trackedPerson.is_occluded = True


# Methods for transforming waypoints into a common coordinate frame
class Transformer(object):
    def __init__(self):
        self.tfListener = tf.TransformListener(True, rospy.Duration(10.0))
        self.transformCache = dict()
        self.lastLookupWarningAt = rospy.Time(0)

    def toCommonFrame(self, waypoint, targetFrame):
        transformedPoint = self.toCommonFrameAsPoint(waypoint, targetFrame)
        return numpy.array([transformedPoint.x, transformedPoint.y])

    def toCommonFrameAsPoint(self, waypoint, targetFrame):
        return self.toCommonFrameAsPointStamped(waypoint, targetFrame).point

    def toCommonFrameAsPointStamped(self, waypoint, targetFrame, recurse = True):
        secs = waypoint["timestamp"]
        sourceFrame = waypoint["frame_id"]

        if sourceFrame != targetFrame:
            # Lookup transform from source into target frame at given timestamp
            cacheLookupKey = self.createCacheLookupKey(sourceFrame, targetFrame)
            try:
                translation, rotation = self.tfListener.lookupTransform(targetFrame, sourceFrame, rospy.Time(secs) )
            except tf.Exception as e:
                # Transform not found in tfListener cache. This might be because playback started after this particular timestamp.
                # Instead, check our own cache, which we serialize to disk with the track database.
                try:
                    dictForTimestamp = self.transformCache[secs]
                    translation, rotation = tuple(dictForTimestamp[cacheLookupKey])
                    #rospy.logwarn("Transform from %s into %s at %.1f not found by TfListener, but available in transform cache on harddisk! Note this transform might be outdated if odometry or sensor locations have been changed!" % (sourceFrame, targetFrame, secs))
                except KeyError:
                    if rospy.Time.now() - self.lastLookupWarningAt > rospy.Duration(1.0):
                        rospy.logwarn("Caught a tf.Exception: " + str(e))
                        self.lastLookupWarningAt = rospy.Time.now()
                    raise e

            # Update our own transform cache
            if not secs in self.transformCache:
                self.transformCache[secs] = dict()
            self.transformCache[secs][cacheLookupKey] = [list(translation), list(rotation)]

            # HACK: Always also look up & cache transform into "odom" frame, if somebody later on (after saving the track database)
            # wants to publish tracks in a frame that is fixed relative to the world.
            if recurse:
                if targetFrame != "odom":
                    self.toCommonFrameAsPointStamped(waypoint, "odom", False)
                if targetFrame != "base_footprint":
                    self.toCommonFrameAsPointStamped(waypoint, "base_footprint", False)

            # Apply transform to point
            transformationMatrix = self.tfListener.fromTranslationRotation(tuple(translation), tuple(rotation))
            xyz = tuple(numpy.dot(transformationMatrix, numpy.array([waypoint["position"]["x"], waypoint["position"]["y"], 0, 1.0])))[:3]

        else:
            xyz = (waypoint["position"]["x"], waypoint["position"]["y"], 0)

        # Build resulting PointStamped msg
        result = PointStamped()
        result.header.stamp = rospy.Time(secs)
        result.header.frame_id = targetFrame
        result.point = Point(*xyz)
        return result

    def createCacheLookupKey(self, sourceFrame, targetFrame):
        return (sourceFrame + " --> " + targetFrame).strip()

    def saveTransformCache(self, databaseFilename):
        filename = self.buildTransformCacheFilename(databaseFilename)
        with open(filename, "w") as f:
            yaml.dump(self.transformCache, f, default_flow_style=False)
        rospy.loginfo("Transform cache has been saved into " + filename + "!")

    def loadTransformCache(self, databaseFilename):
        filename = self.buildTransformCacheFilename(databaseFilename)
        try:
            with open(filename, "r") as f:
                self.transformCache = yaml.load(f)
                rospy.loginfo("Transform cache has been loaded from " + filename + "! Delete this file if odometry or sensor locations have changed, and re-play entire dataset.")
        except IOError:
            rospy.loginfo("Transform cache file %s does not exist, initializing fresh cache!" % filename)

    def buildTransformCacheFilename(self, databaseFilename):
        return databaseFilename.rsplit(".", 1)[0] + ".transform-cache"


# Methods for playing back tracks in the track database
class Visualizer(object):
    def __init__(self, database, editor, transformer):
        self.visualizationFrame = "odom"
        self.database = database
        self.editor = editor
        self.transformer = transformer
        self.markerPublisher = rospy.Publisher("track_annotation_tool/waypoint_markers", MarkerArray, queue_size=1)
        self.hideInactiveTrajectories = False
        self.previousMarkerCount = 0

    def update(self):
        with self.database.lock:
            self.publishWaypointMarkers()

    def publishWaypointMarkers(self):
        markerArray = MarkerArray()

        currentTime = rospy.Time.now()

        header = Header()
        header.frame_id = self.visualizationFrame
        header.stamp = currentTime

        RED = ColorRGBA(r=1, a=1)
        YELLOW = ColorRGBA(r=1, g=1, a=1)
        GREEN = ColorRGBA(g=1, a=1)
        GREY = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1)
        WHITE = ColorRGBA(r=1, g=1, b=1, a=1)

        verticalOffset = +0.3
        dimmedAlphaOfNoncurrentTracks = 0.1

        oldestTimestamp = self.database.findGlobalOldestTimestamp()
        trackAlphas = dict()

        # Trajectory markers
        for trackKey, track in self.database.getTracks().iteritems():
            # Dim alpha of tracks outside of current time window
            oldestStamp = self.database.findOldestTimestamp(track)
            youngestStamp = self.database.findYoungestTimestamp(track)
            if (currentTime.to_sec() < oldestStamp or currentTime.to_sec() > youngestStamp) and (self.editor is None or track != self.editor.getActiveTrack()) :
                trackAlphas[trackKey] = dimmedAlphaOfNoncurrentTracks
            else:
                trackAlphas[trackKey] = 1.

            # Hide trajectory markers of inactive tracks if requested
            if self.editor is not None and self.hideInactiveTrajectories and track != self.editor.getActiveTrack():
                continue

            trajectoryMarker = Marker()
            trajectoryMarker.header = header
            trajectoryMarker.id = len(markerArray.markers)
            trajectoryMarker.type = Marker.LINE_STRIP
            trajectoryMarker.color = copy.deepcopy(GREY if self.editor is None or track != self.editor.getActiveTrack() else RED)
            trajectoryMarker.color.a *= trackAlphas[trackKey]
            trajectoryMarker.scale.x = 0.05

            for waypoint in self.database.getWaypoints(track):
                try:
                    point = self.transformer.toCommonFrameAsPoint(waypoint, self.visualizationFrame)
                    point.z += verticalOffset
                    trajectoryMarker.points.append(point)
                except tf.Exception:
                    pass

            markerArray.markers.append(trajectoryMarker)

        # Waypoint markers
        for trackKey, track in self.database.getTracks().iteritems():
            if self.editor is not None and self.hideInactiveTrajectories and track != self.editor.getActiveTrack():
                continue

            waypointMarker = Marker()
            waypointMarker.header = header
            waypointMarker.id = len(markerArray.markers)
            waypointMarker.type = Marker.SPHERE_LIST
            waypointMarker.scale.x = 0.2

            waypointIndex = 0
            for waypoint in self.database.getWaypoints(track):
                try:
                    point = self.transformer.toCommonFrameAsPoint(waypoint, self.visualizationFrame)
                    point.z += verticalOffset

                    color = copy.deepcopy(YELLOW if self.editor is None or track != self.editor.getActiveTrack() or waypointIndex != self.editor.activeWaypointIndex else GREEN)
                    color.a *= trackAlphas[trackKey]

                    waypointMarker.points.append(point)
                    waypointMarker.colors.append(color)
                except tf.Exception:
                    pass

                waypointIndex += 1

            markerArray.markers.append(waypointMarker)

        # Waypoint IDs and timestamps
        if self.editor is not None:
            waypointIndex = 0
            track = self.editor.getActiveTrack()
            trackKey = self.database.getTrackKey(self.editor.activeTrackId)
            for waypoint in self.database.getWaypoints(track):
                idMarker = Marker()
                idMarker.header = header
                idMarker.id = len(markerArray.markers)
                idMarker.type = Marker.TEXT_VIEW_FACING
                idMarker.color = copy.deepcopy(WHITE)
                idMarker.color.a *= trackAlphas[trackKey]
                idMarker.scale.z = 0.3
                idMarker.text = str(waypointIndex)

                try:
                    transformedPoint = self.transformer.toCommonFrameAsPoint(waypoint, self.visualizationFrame)

                    idMarker.pose.position = transformedPoint
                    idMarker.pose.position.z += 2*verticalOffset
                    markerArray.markers.append(idMarker)

                    waypointTimestamp = waypoint["timestamp"]  # - oldestTimestamp
                    timestampMarker = Marker()
                    timestampMarker.header = header
                    timestampMarker.id = len(markerArray.markers)
                    timestampMarker.type = Marker.TEXT_VIEW_FACING
                    timestampMarker.color = copy.deepcopy(GREY)
                    timestampMarker.color.a *= trackAlphas[trackKey]
                    timestampMarker.scale.z = 0.15
                    timestampMarker.text = "t=%.2f" % waypointTimestamp

                    timestampMarker.pose.position = transformedPoint
                    timestampMarker.pose.position.z += 1.5*verticalOffset
                    markerArray.markers.append(timestampMarker)

                except tf.Exception:
                    pass

                waypointIndex += 1

        # Track IDs
        for trackKey, track in self.database.getTracks().iteritems():
            if self.editor is not None and self.hideInactiveTrajectories and track != self.editor.getActiveTrack():
                continue

            if self.database.getWaypoints(track):
                idMarker = Marker()
                idMarker.header = header
                idMarker.id = len(markerArray.markers)
                idMarker.type = Marker.TEXT_VIEW_FACING
                idMarker.color = ColorRGBA(r=1, g=0.34, b=0.34, a=trackAlphas[trackKey])
                idMarker.scale.z = 0.4
                idMarker.text = "#" + str(self.database.getTrackId(trackKey))

                try:
                    waypoint = self.database.getWaypoints(track)[0]
                    idMarker.pose.position = self.transformer.toCommonFrameAsPoint(waypoint, self.visualizationFrame)
                    idMarker.pose.position.x -= 0.3
                    idMarker.pose.position.z += verticalOffset
                    markerArray.markers.append(idMarker)
                except tf.Exception:
                    pass

        # Delete old markers
        for markerId in xrange(len(markerArray.markers), self.previousMarkerCount):
            oldMarkerToDelete = Marker()
            oldMarkerToDelete.header = header
            oldMarkerToDelete.id = markerId
            oldMarkerToDelete.action = Marker.DELETE

            markerArray.markers.append(oldMarkerToDelete)

        # Publish MarkerArray message
        self.previousMarkerCount = len(markerArray.markers)
        self.markerPublisher.publish(markerArray)


# Publishes simulated time on /clock topic, requires rosparam set /use_sim_time true
class ClockGenerator(object):
    def __init__(self):
        self.enabled = self.wasEnabled = self.simulationEnded = False
        self.paused = False
        self.setSimulationLength(10.0)
        self.setSimulationStart(0.0)
        self.setSimulationRate(1.0)
        self.setLoop(True)
        self.clockPublisher = rospy.Publisher("/clock", Clock, queue_size = 1)
        self.parameterServerLock = RLock()

        self.thread = Thread(target=self.clockLoop)
        self.thread.daemon = True

    def start(self):
        self.thread.start()

    def clockLoop(self):
        while not rospy.is_shutdown():
            enabledNow = self.enabled
            if enabledNow != self.wasEnabled:
                if enabledNow:
                    rospy.loginfo("Starting to publish simulated time on /clock")
                    if self.loop:
                        rospy.loginfo("Simulated time will be reset to start time after %.1f seconds" % self.simulationLength)
                    rospy.loginfo("Setting ROS parameter /use_sim_time to TRUE")
                    self.lastWallTime = time.time()
                    self.currentSimulationOffset = 0.0
                    self.wasSimTimeEnabled = rospy.get_param("/use_sim_time", False)
                    rospy.set_param("/use_sim_time", True)
                else:
                    rospy.loginfo("Stopping to publish simulated time on /clock")
                    rospy.set_param("/use_sim_time", self.wasSimTimeEnabled)
                    rospy.loginfo("Resetting ROS parameter /use_sim_time to " + str(self.wasSimTimeEnabled).upper())

                self.wasEnabled = enabledNow

            if enabledNow:
                currentWallTime = time.time()
                wallTimePassed = currentWallTime - self.lastWallTime

                if not self.paused:
                    self.currentSimulationOffset += wallTimePassed * self.simulationRate

                if self.currentSimulationOffset > self.simulationLength:
                    if self.loop:
                        self.currentSimulationOffset %= self.simulationLength
                        rospy.loginfo("Maximum simulation length (%.1f sec) reached, restarting simulation" % self.simulationLength)
                    else:
                        self.simulationEnded = True

                clock = Clock()
                clock.clock = rospy.Time(self.simulationStart + self.currentSimulationOffset)

                try:
                    self.clockPublisher.publish(clock)
                except rospy.exceptions.ROSException as e:
                    pass

                self.lastWallTime = currentWallTime
                time.sleep(1.0 / 100)

    def setSimulationLength(self, seconds):
        self.simulationLength = seconds

    def setSimulationStart(self, seconds):
        self.simulationStart = seconds

    def setSimulationRate(self, rate):
        self.simulationRate = rate

    def setLoop(self, loop):
        self.loop = loop

    def setEnabled(self, enabled):
        self.enabled = enabled


# Track database that can be serialized into a YAML file
class Database(object):
    def __init__(self):
        self.data = dict()
        self.initDefaults()
        self.loadCallbacks = []
        self.saveCallbacks = []
        self.lock = RLock()

    def initDefaults(self):
        self.setFieldIfUnset("formatVersion", 1)
        self.hasUnsavedChanges = False

    def registerLoadCallback(self, callback):
        self.loadCallbacks.append(callback)

    def registerSaveCallback(self, callback):
        self.saveCallbacks.append(callback)

    def load(self, filename):
        with open(filename, "r") as f:
            self.data = yaml.load(f)

        self.initDefaults()

        rospy.loginfo("Track database %s has been loaded!" % filename)
        self.printStatistics()

        for loadCallback in self.loadCallbacks:
            loadCallback(filename)

    def save(self, filename):
        with open(filename, "w") as f:
            yaml.dump(self.data, f, default_flow_style=False)

        self.hasUnsavedChanges = False
        rospy.loginfo("Track database has been saved to %s!" % filename)

	for saveCallback in self.saveCallbacks:
            saveCallback(filename)

    def getField(self, name):
        return self.data[name]

    def setField(self, name, value):
        self.data[name] = value

    def setFieldIfUnset(self, name, value):
        if not name in self.data:
            self.data[name] = value

    def getDict(self, name, parentDict):
        if not name in parentDict:
            parentDict[name] = dict()
        return parentDict[name]

    def getList(self, name, parentDict):
        if not name in parentDict:
            parentDict[name] = []
        return parentDict[name]

    def getTracks(self):
        return self.getDict("tracks", self.data)

    def getTrackKey(self, trackId):
        return "track %03d" % trackId

    def getTrackId(self, trackKey):
        return int(trackKey[6:])

    def getTrackIds(self):
        return [ int(self.getTrackId(trackKey)) for trackKey in self.getTracks().iterkeys() ]

    def getTrack(self, trackId):
        return self.getDict(self.getTrackKey(trackId), self.getTracks())

    def getWaypoints(self, track):
        return self.getList("waypoints", track)

    def getScalarAttributes(self, track):
        return self.getDict("scalarAttributes", track)

    def getCategoricalAttributes(self, track):
        return self.getDict("categoricalAttributes", track)

    def findGlobalOldestTimestamp(self):
        # Find oldest timestamp amongst all data, i.e. where the data "begins"
        oldestTimestamp = float("inf")
        for track in self.getTracks().itervalues():
            oldestTimestamp = min(oldestTimestamp, self.findOldestTimestamp(track))
        return oldestTimestamp

    def findOldestTimestamp(self, track):
        # Find oldest timestamp for given track
        if self.getWaypoints(track):
            return self.getWaypoints(track)[0]["timestamp"]
        return float("inf")

    def findGlobalYoungestTimestamp(self):
        # Find youngest timestamp amongst all data, i.e. where the data "ends"
        youngestTimestamp = float("-inf")
        for track in self.getTracks().itervalues():
            youngestTimestamp = max(youngestTimestamp, self.findYoungestTimestamp(track))
        return youngestTimestamp

    def findYoungestTimestamp(self, track):
        # Find youngest timestamp for given track
        if self.getWaypoints(track):
            return self.getWaypoints(track)[-1]["timestamp"]
        return float("-inf")

    def findLargestTrackId(self):
        return max(self.getTrackIds())

    def printStatistics(self):
        tracks = self.getTracks()
        numTracks = len(tracks)
        numWaypoints = 0
        for track in tracks.itervalues():
            waypoints = self.getWaypoints(track)
            numWaypoints += len(waypoints)

        rospy.loginfo("Database statistics: %d track(s), %d waypoints in total" % (numTracks, numWaypoints) )


# Methods for editing the track database
class Editor(object):
    def __init__(self, database, transformer):
        self.activeTrackChangedCallbacks = []
        self.activeWaypointChangedCallbacks = []
        self.activeTrackId = None
        self.activeWaypointIndex = 0
        self.activeFrameId = None
        self.transformer = transformer
        self.database = database
        self.database.registerLoadCallback(self.onDatabaseLoaded)
        self.database.registerLoadCallback(transformer.loadTransformCache)
        self.database.registerSaveCallback(transformer.saveTransformCache)
        self.onDatabaseLoaded(None)

    def registerActiveTrackChangedCallback(self, callback):
        self.activeTrackChangedCallbacks.append(callback)

    def registerActiveWaypointChangedCallback(self, callback):
        self.activeWaypointChangedCallbacks.append(callback)

    def onDatabaseLoaded(self, filename):
        self.selectActiveTrack(0)
        self.selectActiveWaypoint(len(self.database.getWaypoints(self.getActiveTrack()))-1)

    def getActiveTrack(self):
        return self.database.getTrack(self.activeTrackId)

    def selectActiveTrack(self, trackId):
        self.activeTrackId = trackId
        rospy.loginfo("New active track selected with ID %d" % trackId)
        for callback in self.activeTrackChangedCallbacks:
            callback()

    def deleteActiveTrack(self):
        self.deleteTrack(self.activeTrackId)

    def deleteTrack(self, trackId):
        with self.database.lock:
            del self.database.getTracks()[self.database.getTrackKey(trackId)]
            if self.activeTrackId == trackId:
                lastTrackId = 0
                if self.database.getTracks():
                    lastTrackId = self.database.findLargestTrackId()
                self.selectActiveTrack(lastTrackId)
            self.database.hasUnsavedChanges = True

    def selectActiveWaypoint(self, waypointIndex):
        waypointCount = len(self.database.getWaypoints(self.getActiveTrack()))
        self.activeWaypointIndex = max(0, min(waypointIndex, waypointCount - 1))
        for callback in self.activeWaypointChangedCallbacks:
            callback()

    def addNewTrack(self):
        with self.database.lock:
            newTrackId = self.database.findLargestTrackId()+1
            rospy.loginfo("Creating new track with ID %d" % newTrackId)
            self.selectActiveTrack(newTrackId)
            self.database.hasUnsavedChanges = True

    def addWaypoint(self, pointStamped):
        with self.database.lock:
            # Important: Transform into specified coordinate frame for storage
            latestCommonTime = self.transformer.tfListener.getLatestCommonTime(pointStamped.header.frame_id, self.activeFrameId)
            if pointStamped.header.stamp - latestCommonTime < rospy.Duration(0.1):
	      pointStamped.header.stamp = latestCommonTime

            pointStamped = self.transformer.tfListener.transformPoint(self.activeFrameId, pointStamped)
            timestamp = pointStamped.header.stamp
            point = pointStamped.point

            waypoint = { "timestamp": timestamp.to_sec(), "frame_id": self.activeFrameId, "position": { "x": float(point.x), "y": float(point.y) } }

            waypoints = self.database.getWaypoints(self.getActiveTrack())
            for existingWaypoint in waypoints:
                if existingWaypoint["timestamp"] == waypoint["timestamp"]:
                    rospy.logwarn("Waypoint with timestamp %f already exists! Delete existing waypoint first before adding new one / make sure clock is running." % waypoint["timestamp"])
                    return

            waypoints.insert(self.activeWaypointIndex+1, waypoint)

            self.sortWaypoints(waypoints)  # sort waypoints by time
            for waypointIndex in xrange(0, len(waypoints)):
                if waypoints[waypointIndex]["timestamp"] == timestamp.to_sec():
                    self.selectActiveWaypoint(waypointIndex)

            rospy.loginfo("New waypoint #%d added to track %d: %s" % (self.activeWaypointIndex, self.activeTrackId, str(waypoint)) )
            self.database.hasUnsavedChanges = True

    def deleteActiveWaypoint(self):
        with self.database.lock:
            if self.activeWaypointIndex < 0:
                rospy.logwarn("Cannot delete next waypoint because list of waypoints is empty!")
            else:
                del self.database.getWaypoints(self.getActiveTrack())[self.activeWaypointIndex]
                rospy.loginfo("Waypoint #%d of track %d has been removed" % (self.activeWaypointIndex, self.activeTrackId))
                self.selectActiveWaypoint(self.activeWaypointIndex-1)
                self.database.hasUnsavedChanges = True

    def sortWaypoints(self, waypoints):
        waypoints.sort(key=lambda waypoint: waypoint["timestamp"])  # sort waypoints by time

    def mergeTracks(self, track1Id, track2Id):
        rospy.loginfo("Merging tracks %d and %d" % (track1Id, track2Id))

        track1 = self.database.getTrack(track1Id)
        track2 = self.database.getTrack(track2Id)

        wp1 = self.database.getWaypoints(track1)
        wp2 = self.database.getWaypoints(track2)

        wp1.extend(wp2)
        self.sortWaypoints(wp1)  # sort waypoints by time

        deletedTrackWasActive = self.activeTrackId == track2Id
        self.deleteTrack(track2Id)

        if deletedTrackWasActive:
            self.selectActiveTrack(track1Id)
            self.selectActiveWaypoint(0)

class WallRate():
    def __init__(self, rate):
        self.rate = rate
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.recorded_time = time.time()

    def sleep(self):
        current_time = time.time()
        elapsed = current_time - self.recorded_time
        if self.period - elapsed > 0:
            rospy.rostime.wallsleep(self.period - elapsed)
        self.recorded_time = time.time()
