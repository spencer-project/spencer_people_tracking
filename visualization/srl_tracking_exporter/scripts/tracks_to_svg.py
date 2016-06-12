#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
Renders tracks (spencer_tracking_msg/TrackedPersons) and odometry to an SVG file, (C)2014 Timm Linder
Parameters:
  _odom (bool):       render odometry published at /odom?, default: True
  _detections (bool): render detections as rectangles?, default: True
  _grid (bool):       render metric grid?, default: True
  _style (string):    rendering style for tracks, either "lines" or "points", default: "points"
  _colormap (string): colormap to use for tracks, either "rainbow", "srl" or "srlalternative", default: "rainbow"
  _occlusions (bool): if true, occlusions will be rendered as circles without fill color (only for style "points"), default: True
  _labels (bool):     render track IDs? default: True
  _timestamps (bool): render track deletion/creation timestamps? default: True
  _velocities (bool): render track and odometry velocities (as < shape), default: True
  _start (float):     time index at which to start recording, default: 0
  _end (float):       time index at which to stop recording, default: infinity
  _prefix (string):   prefix for filename of rendered SVGs, default: "Tracks "
  _autoquit (bool):   if true, will quit automatically if no data is received for 10 seconds, default: True
  _animate (bool):    add animation, default: False
  _speed (float):     animation speed, default: 1.0
  _history (bool):    show track history in animation, default: True
  _cycles (bool):     show cycle number on track (lots of text elements!), default: False
"""

import os, sys, math, time, socket, xmlrpclib, signal, codecs
import svgwrite
from multiprocessing import Lock

import roslib, rospy, tf; roslib.load_manifest('srl_tracking_exporter')
import geometry_msgs.msg
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson, DetectedPersons, DetectedPerson
from nav_msgs.msg import Odometry
from srl_tracking_exporter.srv import JobFinished

### Database for storing information ###
class TrackDatabase(object):
    def __init__(self):
        self.tracks = dict()
        self.trackStamps = []
        self.detections = []
        self.detectionStamps = []
        self.odom = []
        self.odomStamps = []
        self.minx = self.miny = float("+inf")
        self.maxx = self.maxy = float("-inf")
        self.firstTracksReceivedAt = None
        self.lastTracksReceivedAt = None
        self.firstDataReceivedAt = None
        self.lastDataReceivedAt = None

### Detection ###
class Detection(object):
    def __init__(self):
        self.position = None
        self.cycle = None

### Track ###
class Track(object):
    def __init__(self):
        self.positions = []
        self.velocities = []
        self.occluded = []
        self.cycles = []
        self.createdAt = None
        self.deletedAt = None

### Track receiver ###
class TrackReceiver(object):
    def __init__(self):
        self.mutex = Lock()
        self.firstTracksReceived = False
        self.trackCycle = -1
        self.detectionCycle = -1
        self.targetFrame = rospy.get_param('~frame', "odom")

        self.lastDataAt = None
        self.startLimit = rospy.get_param('~start', 0.0)
        self.endLimit = rospy.get_param('~end', float("inf"))
        self.autoQuit = rospy.get_param('~auto_quit', True)

        self.transformListener = tf.TransformListener()
        self.numDetectionTfErrors = 0
        self.numTrackTfErrors = 0
        self.numOdometryTfErrors = 0

        self.detectionSubscriber = rospy.Subscriber("/spencer/perception/detected_persons", DetectedPersons, self.newDetectionsReceived)
        self.trackSubscriber = rospy.Subscriber("/spencer/perception/tracked_persons", TrackedPersons, self.newTracksReceived)
        self.odomSubscriber  = rospy.Subscriber("/odom", Odometry, self.odometryReceived)

        rospy.loginfo("Will start processing tracks at time index %f and end at %f" % (self.startLimit, self.endLimit))
        if self.autoQuit:
            rospy.loginfo("Will end processing if no data is received for more than 10 seconds after last data.")
        
        rospy.loginfo("Listening for tracks... press CTRL+C to stop and generate SVG output.")

    def stop(self):
        self.detectionSubscriber.unregister()
        self.trackSubscriber.unregister()
        self.odomSubscriber.unregister()

    def dataReceivedAt(self, secs):
        database.lastDataReceivedAt = secs
        if database.firstDataReceivedAt is None:
            database.firstDataReceivedAt = secs

    def positionPassesSanityCheck(self, pos):
        if math.isnan(pos.x) or math.isnan(pos.y):
            return False

        MAX_REASONABLE_DISTANCE = 1000.0  # in meters
        if pos.x > MAX_REASONABLE_DISTANCE or pos.y > MAX_REASONABLE_DISTANCE:
            return False

        return True

    def updateCoordinateLimits(self, pos):
        database.minx = min(database.minx, pos.x)
        database.miny = min(database.miny, pos.y)
        database.maxx = max(database.maxx, pos.x)
        database.maxy = max(database.maxy, pos.y)
        
        self.lastDataAt = time.time()

    def transformIntoTargetFrame(self, poseWithCovariance, header):
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header = header
        poseStamped.pose = poseWithCovariance.pose
        transformedPose = self.transformListener.transformPose(self.targetFrame, poseStamped)
        if math.isnan(transformedPose.pose.position.x) or math.isnan(transformedPose.pose.position.y):
            rospy.logwarn("Encountered NaN value in position, raising a TF exception...")
            raise tf.Exception
        return transformedPose

    def getTrack(self, track_id):
        if track_id in database.tracks:
            track = database.tracks[track_id]
        else:
            track = Track()
            database.tracks[track_id] = track
        return track

    def newDetectionsReceived(self, detectedPersons):       
        time = rospy.get_time()
        if time < self.startLimit or time > self.endLimit:
            return

        with self.mutex:
            self.detectionCycle += 1
            secs = detectedPersons.header.stamp.to_sec()
            database.detectionStamps.append(secs)        
            self.dataReceivedAt(secs)
                
            for detectedPerson in detectedPersons.detections:
                detection = Detection()
                
                try:
                    pos = self.transformIntoTargetFrame(detectedPerson.pose, detectedPersons.header).pose.position
                except tf.Exception:
                    self.numDetectionTfErrors += 1
                    if self.numDetectionTfErrors < 2:
                        rospy.logwarn("TF exception encountered while transforming detection position into target frame!")
                    return

                pos.y = -pos.y  # due to SVG conventions
                detection.position = (pos.x, pos.y)
                detection.cycle = self.detectionCycle

                if self.positionPassesSanityCheck(pos):
                    self.updateCoordinateLimits(pos)
                    database.detections.append(detection)
                else:
                    rospy.logwarn("Ignoring DetectedPerson because its position does not pass heuristic sanity check:\n" + str(detectedPerson))

    def newTracksReceived(self, trackedPersons):        
        time = rospy.get_time()
        if time < self.startLimit or time > self.endLimit:
            return

        with self.mutex:
            secs = trackedPersons.header.stamp.to_sec()
            database.lastTracksReceivedAt = secs
            self.dataReceivedAt(secs)
            self.trackCycle += 1
            
            if not self.firstTracksReceived:
                self.firstTracksReceived = True
                database.firstTracksReceivedAt = secs
                rospy.loginfo("First track(s) received at %f secs" % secs)

            for trackedPerson in trackedPersons.tracks:
                track = self.getTrack(trackedPerson.track_id)

                try:
                    pos = self.transformIntoTargetFrame(trackedPerson.pose, trackedPersons.header).pose.position
                except tf.Exception:
                    self.numTrackTfErrors += 1
                    if self.numTrackTfErrors < 2:
                        rospy.logwarn("TF exception encountered while transforming track position into target frame!")
                    return

                pos.y = -pos.y  # due to SVG conventions

                if self.positionPassesSanityCheck(pos):
                    track.positions.append( (pos.x, pos.y) )
                    track.cycles.append(self.trackCycle)
                    self.updateCoordinateLimits(pos)

                    vel = trackedPerson.twist.twist.linear  # FIXME: Rotate into target frame!
                    vel.y = -vel.y  # due to SVG conventions
                    track.velocities.append( (vel.x, vel.y) )

                    track.occluded.append( trackedPerson.is_occluded )

                    if track.createdAt is None:
                        track.createdAt = secs
                    track.deletedAt = secs  # will be updated as long as track exists
                else:
                    rospy.logwarn("Ignoring TrackedPerson because its position does not pass heuristic sanity check:\n" + str(trackedPerson))

            database.trackStamps.append(secs)
                
    def odometryReceived(self, odom):
        try:
            posInTargetFrame = self.transformIntoTargetFrame(odom.pose, odom.header).pose.position    
            posInTargetFrame.y = -posInTargetFrame.y
            database.odom.append( (posInTargetFrame.x, posInTargetFrame.y) )

            secs = odom.header.stamp.to_sec()
            database.odomStamps.append(secs)
            self.dataReceivedAt(secs)

            self.updateCoordinateLimits(posInTargetFrame)
        except tf.Exception:
            self.numOdometryTfErrors += 1
            if self.numOdometryTfErrors < 2:
                rospy.logwarn("TF exception encountered while transforming odometry into target frame!")
            pass

    def spin(self):
        while not (quitRequested or rospy.is_shutdown() or rospy.get_time() > self.endLimit):
            time.sleep(0.1)
            if self.autoQuit and self.firstTracksReceived and not self.lastDataAt is None and time.time() - self.lastDataAt > 10.0:
                rospy.loginfo("No data received for more than 10 seconds, quitting.")
                break
        self.stop()

### Track renderer ###
class TrackRenderer(object):
    def __init__(self):
        self.colormap = rospy.get_param('~colormap', 'rainbow')
        self.style = rospy.get_param('~style', 'points')       
        self.showOcclusions = rospy.get_param('~occlusions', True)       
        self.filenamePrefix = rospy.get_param('~prefix', 'Tracks ')       
        self.gridEnabled = rospy.get_param('~grid', True)
        self.odomEnabled = rospy.get_param('~odom', True)
        self.detectionsEnabled = rospy.get_param('~detections', True)
        self.velocitiesEnabled = rospy.get_param('~velocities', True)
        self.labelsEnabled = rospy.get_param('~labels', True)
        self.timestampsEnabled = rospy.get_param('~timestamps', True)
        self.animationEnabled = rospy.get_param('~animate', False)
        self.animationSpeed = rospy.get_param('~speed', 1.0)
        self.historyEnabled = rospy.get_param('~history', True)
        self.cyclesEnabled = rospy.get_param('~cycles', False)

    def getTrackColor(self, track_id, trackCycle = 0):
        index = track_id
        
        if self.colormap == 'srl' or self.colormap == 'srlalternative':
            palette = [ 0xC00000, 0xFF0000, 0xFF5050, 0xFFA0A0,  # red
                        0x00C000, 0x00FF00, 0x50FF50, 0xA0FFA0,  # green
                        0x0000C0, 0x0000FF, 0x5050FF, 0xA0A0FF,  # blue
                        0xF20A86, 0xFF00FF, 0xFF50FF, 0xFFA0FF,  # magenta
                        0x00C0C0, 0x00FFFF, 0x50FFFF, 0xA0FFFF,  # cyan
                        0xF5A316, 0xFFFF00, 0xFFFF50, 0xFFFFA0 ] # yellow

            if self.colormap == 'srlalternative': # swap rows and columns
                index = index % len(palette)
                index = (index % 6)*4 + (index / 6)
        else:
            palette = [0xaf1f90, 0x000846, 0x00468a, 0x00953d, 0xb2c908, 0xfcd22a, 0xffa800, 0xff4500, 0xe0000b, 0xb22222] # rainbow color map
        
        return '#%06x' % palette[index % len(palette)]

    def animateElement(self, element, currentCycle, cycleStamps, showHistory = True):
        if not self.animationEnabled:
            return

        firstStamp = database.firstDataReceivedAt
        lastStamp = database.lastDataReceivedAt
        duration = lastStamp - firstStamp 
        keyTimes = "0;"
        opacityValues = [0.0, 0.0, 1.0]

        if currentCycle == 0:
            keyTimes += "0"
        else:
            keyTimes += str((cycleStamps[currentCycle - 1] - firstStamp) / duration)

        keyTimes += ";" + str((cycleStamps[currentCycle] - firstStamp) / duration)
            
        if not showHistory:
            if currentCycle < len(cycleStamps) - 1:
                keyTimes += ";" + str((cycleStamps[currentCycle + 1] - firstStamp) / duration) + ";1"
                opacityValues.append(0.0)
                opacityValues.append(0.0)
        else:
            keyTimes +=";1"
            opacityValues.append(1.0)

        animation = svg.animate(attributeName='opacity', values=opacityValues, keyTimes=keyTimes, calcMode='discrete', dur="%fs" % (duration / self.animationSpeed), repeatCount='indefinite', fill='freeze')
        element.add(animation)

    def renderGrid(self, left, top, width, height):
        pattern = svg.defs.add(svg.pattern(id="gridPattern", size=(1, 1), patternUnits="userSpaceOnUse"))
        pattern.add(svg.path(d="M 1 0 L 0 0 0 1", fill="none", stroke="#CCC", stroke_width=0.01))
        content.add(svg.rect(id='background', insert=(left, top), size=(width, height), fill="white"))
        content.add(svg.rect(id='grid', insert=(left, top), size=(width, height), fill="url(#gridPattern)"))

    def renderOdometry(self):
        if len(database.odom) > 0:
            content.add(svg.polyline(database.odom, id='odom', stroke='#999', stroke_width=0.02, stroke_linecap='round', fill='none'))

        if self.velocitiesEnabled or self.animationEnabled:
            odomVelocityGroup = content.add(svg.g(id='odomVelocities'))
            lookback = 3
            for step in range(lookback, len(database.odom), lookback):
                pos = database.odom[step]
                previousPos = database.odom[step-lookback] 
                avgPos = database.odom[step-lookback//2]
                deltatime = database.odomStamps[step] - database.odomStamps[step-lookback]
                vel = ((pos[0] - previousPos[0]) / deltatime, (pos[1] - previousPos[1]) / deltatime)
                theta = math.degrees(math.atan2(vel[1], vel[0]) + math.pi)

                sx = 0.2 * math.hypot(vel[0], vel[1])
                sy = 0.2
                arrowCoords = [(sx,-sy), (0,0), (sx,sy)] 
                
                arrow = svg.polyline(arrowCoords, id='arrow', stroke_width=0.015, stroke_opacity=0.35, stroke='#999', fill='none')
                arrow.translate(avgPos)
                arrow.rotate(theta)

                self.animateElement(arrow, step-lookback, database.odomStamps)
                odomVelocityGroup.add(arrow)

    def renderVelocities(self):
        velocityGroup = content.add(svg.g(id='velocities'))      

        # Track velocities
        for track_id, track in database.tracks.iteritems():
            trackVelocityGroup = velocityGroup.add(svg.g(id='velocities%d' % track_id))
            
            for step in range(0, len(track.positions)):
                vel = track.velocities[step]
                pos = track.positions[step]
                theta = math.degrees(math.atan2(vel[1], vel[0]) + math.pi)

                sx = 0.2 * math.hypot(vel[0], vel[1])
                sy = 0.2
                arrowCoords = [(sx,-sy), (0,0), (sx,sy)] 
                
                arrow = svg.polyline(arrowCoords, id='arrow', stroke_width=0.015, stroke_opacity=0.35, stroke=self.getTrackColor(track_id), fill='none')
                arrow.translate(pos)
                arrow.rotate(theta)

                self.animateElement(arrow, track.cycles[step], database.trackStamps)
                trackVelocityGroup.add(arrow)

    def renderTrack(self, track_id, track):        
        trackGroup = content.add(svg.g(id='track%d' % track_id))
        
        #if self.style == 'lines':
        if len(track.positions) > 0:
            trackGroup.add(svg.polyline(track.positions, stroke=self.getTrackColor(track_id), stroke_width=0.01, stroke_linecap='round', fill='none'))
        
        if self.style == 'points':
            for step in range(0, len(track.positions)):
                #alpha = 0.5 if track.occluded[step] and showOcclusions else 1.0
                alpha = 1.0
                trackCycle = track.cycles[step]
                if not track.occluded[step] or not self.showOcclusions:
                    trackVisual = svg.circle(track.positions[step], r=0.1, fill=self.getTrackColor(track_id, trackCycle))
                    
                else:
                    trackVisual = svg.circle(track.positions[step], r=0.09, fill='none', stroke=self.getTrackColor(track_id, trackCycle), stroke_width=0.01)

                self.animateElement(trackVisual, trackCycle, database.trackStamps, self.historyEnabled)
                trackGroup.add(trackVisual)
            
    def renderDetections(self):
        detectionsGroup = content.add(svg.g(id='detections'))        
        for detection in database.detections:
            size = 0.075
            rect = svg.rect(size=(size, size), fill='none', stroke='#999', stroke_width=0.005)
            rect.translate((detection.position[0] - 0.5*size, detection.position[1] - 0.5*size))
            rect.rotate(45)

            self.animateElement(rect, detection.cycle, database.detectionStamps, self.historyEnabled)
            detectionsGroup.add(rect)
            
    def renderTrackLabels(self):
        labelGroup = content.add(svg.g(id='labels'))        
        for track_id, track in database.tracks.iteritems():
            trackColor = self.getTrackColor(track_id)
            if len(track.positions) > 0:
                pos = track.positions[-1]
                pos = (pos[0], pos[1] - 0.15)
                fontSize = 0.25
                labelGroup.add(svg.text(str(track_id), insert=pos, text_anchor="middle", fill=trackColor, stroke='white', stroke_width=0.02, font_size=fontSize))       
                labelGroup.add(svg.text(str(track_id), insert=pos, text_anchor="middle", fill=trackColor, font_size=fontSize))       

    def renderTimestamps(self):
        timestampGroup = content.add(svg.g(id='timestamps'))        
        for track_id, track in database.tracks.iteritems():
            if len(track.positions) == 0:
                continue

            trackColor = self.getTrackColor(track_id)
            fontSize = 0.06

            # Creation
            pos = track.positions[0]
            pos = (pos[0], pos[1] + 0.17)
            timestamp = "+%.1f" % (track.createdAt - database.firstDataReceivedAt)
            creationTimestampGroup = content.add(svg.g())        
            creationTimestampGroup.add(svg.text(timestamp, insert=pos, text_anchor="middle", fill=trackColor, stroke='white', stroke_width=0.02, font_size=fontSize))       
            creationTimestampGroup.add(svg.text(timestamp, insert=pos, text_anchor="middle", fill=trackColor, font_size=fontSize))       

            #self.animateElement(creationTimestampGroup, 1, [track.createdAt - 0.01, track.createdAt])
            timestampGroup.add(creationTimestampGroup)

            # Deletion
            pos = track.positions[-1]
            pos = (pos[0], pos[1] + 0.17)
            timestamp = "-%.1f" % (track.deletedAt - database.firstDataReceivedAt)
            deletionTimestampGroup = content.add(svg.g())        
            deletionTimestampGroup.add(svg.text(timestamp, insert=pos, text_anchor="middle", fill=trackColor, stroke='white', stroke_width=0.02, font_size=fontSize))       
            deletionTimestampGroup.add(svg.text(timestamp, insert=pos, text_anchor="middle", fill=trackColor, font_size=fontSize))       

            timestampGroup.add(deletionTimestampGroup)

    def renderCycles(self):
        cyclesGroup = content.add(svg.g(id='cycles'))        
        for track_id, track in database.tracks.iteritems():
            trackColor = self.getTrackColor(track_id)
            fontSize = 0.06

            for i in xrange(0, len(track.positions), 2):
                pos = track.positions[i]
                pos = (pos[0], pos[1])
                cycleText = str(track.cycles[i])   
                cyclesGroup.add(svg.text(cycleText, insert=pos, text_anchor="middle", fill=trackColor, stroke='white', stroke_width=0.02, font_size=fontSize))       
                cyclesGroup.add(svg.text(cycleText, insert=pos, text_anchor="middle", fill=trackColor, font_size=fontSize))       

    def generateSVG(self):
        rospy.loginfo("Generating SVG...")

        if database.minx == float("+inf") or database.miny == float("+inf"):
            rospy.logwarn("Empty data, aborting!")
            return
    
        # Calculate dimensions
        left = database.minx - 0.5
        top = database.miny - 0.5
        width = database.maxx - database.minx + 1
        height = database.maxy - database.miny + 1

        # Create SVG
        global svg, content
        filename = '%s%s.svg' % (self.filenamePrefix, time.strftime("%Y-%m-%d %H:%M:%S"))
        svg = svgwrite.Drawing(filename, profile='full', size=('%f' % (150*width), '%f' % (150*height)), viewBox=('%f %f %f %f' % (left, top, width, height)))
        content = svg.g(id='content')
        svg.add(content)

        # Metric grid
        if self.gridEnabled:
            self.renderGrid(left, top, width, height)            

        # Odometry
        if self.odomEnabled:
            self.renderOdometry()

        # Velocities
        if self.velocitiesEnabled:
            self.renderVelocities()

        # Tracks
        rospy.loginfo("Writing %d tracks..." % len(database.tracks))
        for track_id, track in database.tracks.iteritems():
            self.renderTrack(track_id, track)

        # Detections
        if self.detectionsEnabled:
            self.renderDetections()

        # Track labels
        if self.labelsEnabled:
            self.renderTrackLabels()

        # Cycle numbers on tracks
        if self.cyclesEnabled:
            self.renderCycles()

        # Creation/deletion timestamps
        if self.timestampsEnabled:
            self.renderTimestamps()
          
        # Write SVG
        rospy.loginfo("Starting to write SVG...")
        if not self.animationEnabled:
            svg.save()
        else:
            htmlTemplateFilename = roslib.packages.get_pkg_dir('srl_tracking_exporter') + "/scripts/svg_frame.html"

            with open (htmlTemplateFilename, "r") as htmlTemplate:
                htmlContent = htmlTemplate.read()

            svgXml = svg.tostring()
            htmlOutput = htmlContent.replace("<!-- ${INSERT_SVG_HERE} -->", svgXml)

            filename = filename + ".html"
            with codecs.open (filename, "w", "utf-8") as outputFile:
                outputFile.write(htmlOutput)

        rospy.loginfo("SVG generation finished! Output filename: " + filename)

        # Notify job monitor, if running
        try:
            jobFinished = rospy.ServiceProxy('job_monitor', JobFinished)
            jobFinished(rospy.get_name())
        except rospy.ServiceException, e:
            rospy.loginfo("Failed to notify job monitor (maybe not running?): %s" % e)


### Signal handler for CTRL+C ###
def __signal_handler__(signal, frame):
    global quitRequested
    quitRequested = True

### Handle CTRL+C ###
def __installSignalHandler__():
    signal.signal(signal.SIGINT, __signal_handler__)
    signal.signal(signal.SIGTERM, __signal_handler__)

### Makes sure that ROS master is running ###
def __ensureMasterRunning__():
    caller_id = '/script'
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])

    messageShown = False
    while not quitRequested:
        try:
            code, msg, val = m.getSystemState(caller_id)
            return
        except socket.error:
            if not messageShown:
                messageShown = True
                print("Waiting for ROS master... please make sure that roscore is running!")
            time.sleep(0.1)
    sys.exit(0)
    
### Node initializer, also installs signal handler and waits for ROS master ###
def initNode(nodeName):
    # Ensure that ROS master is running, else wait
    __installSignalHandler__()
    __ensureMasterRunning__()

    # Initialize ROS node
    rospy.init_node(nodeName, disable_signals=True)

### Main method ###
def main(argv=None):
    initNode('tracks_to_svg')

    # Create receiver and renderer and load parameters (do this at startup so script doesn't crash if rosmaster is terminated)
    trackReceiver = TrackReceiver()
    trackRenderer = TrackRenderer()
    
    trackReceiver.spin()
    trackRenderer.generateSVG()

    sys.exit(0)

### Globals ###
quitRequested = False
svg = None
content = None
database = TrackDatabase()
if __name__ == "__main__":
    main()