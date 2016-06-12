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

import rospy, sys, os.path, time, cv2, cv_bridge, numpy, select, termios, tty, math, collections, copy, tf
from math import isinf
from threading import Thread
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Image
from srl_laser_segmentation.msg import LaserscanSegmentation, LaserscanSegment
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson, DetectedPersons, DetectedPerson
from geometry_msgs.msg import Quaternion




""" Reads data from a logfile and publishes to ROS topics. """
class LogfilePublisher(object):
    """ Values that define how fast data is published and if original timestamps are respected or not. """
    class TimingMode:
        use_timestamps = 0  # use timestamps from the logfile. Use "rate" parameter to multiply intervals by constant factor. 
        use_timestamps_with_remapping = 1  # as above, but remap/shift timestamps into present time, first message will be published immediately
        fixed_rate = 2      # publish frames at a constant, fixed rate (e.g. 30 Hz). Use "rate" parameter to set the publish rate in frames per second.
        immediate = 3       # publish all data as fast as possible (requires large subscriber queues!)
    
    """ Create a new LogfilePublisher, use read(filename) to open and publish a logfile. """
    def __init__(self):
        ### Configurable parameters ##########
        self.startAt = rospy.get_param("~start", float("-inf"))       # start time in seconds (log time)
        self.endAt   = rospy.get_param("~end",   float("inf"))        # end time in seconds (log time)
        self.timingMode = rospy.get_param("~timing_mode", LogfilePublisher.TimingMode.use_timestamps_with_remapping)
        self.rate = rospy.get_param("~rate", 1.0)
        self.publishClock = rospy.get_param("~publish_clock", False)  # publish clock, required if you want to pause or playback step-by-step
        self.clockRate = rospy.get_param('~clock_rate', 1.0)          # how fast published time ticks, if enabled
        self.initialWait = rospy.get_param("~initial_wait", 1.0)      # seconds to wait after advertising publishers before publishing first message
        self.queueSize = rospy.get_param("~queue_size", 100000)       # publisher queue sizes, use very low number for minimum latency 
        self.publishTracks = rospy.get_param("~publish_tracks", False)# tracks including occluded ones are published from here
        ### End of configurable parameters ###

        self._earliestTimestamp = float("inf")
        self._latestTimestamp = float("-inf")

        self._laserscanPublishers = dict()
        self._laserscanAnnotationsPublishers = dict()
        self._imagePublishers = dict()

        self._laserscanCounter = 0
        self._laserscanAnnotationCounter = 0
        self._imageCounter = 0

        self._paused = rospy.get_param("~paused", False)              # start paused?
        self._singleStep = False

        self._binaryFile = None
        self._cvBridge = cv_bridge.CvBridge()

        self.COLOR_RED = "\x1b[31m"
        self.COLOR_GREEN = "\x1b[32m"
        self.COLOR_YELLOW = "\x1b[33m"
        self.COLOR_DEFAULT = "\x1b[39;49m"
        
        self._activeTrackIDs = dict()
        self._deletetTrackIDs = dict()
        self._occludedIDs = dict()
        self._occludedIDsPerTimestamp = dict()
        
        if (self.publishTracks):
            self._toTrackConverter = SegmentationToTrackConverter()
        self._currentLaserMsgs= dict()
        self._currentAnnotationMsg = dict()

        if self._paused:
            print self.COLOR_YELLOW + "Paused! Press SPACE to continue, S for single step..." + self.COLOR_DEFAULT

        # Start clock generator if requested
        if self.publishClock:
            if(rospy.get_param('/use_sim_time', False)):
                clockRate = self.clockRate
                clockThread = Thread(target=self._publishClock, args=(clockRate, ))
                clockThread.daemon = True
                clockThread.start()
                time.sleep(1.0)
                print self.COLOR_GREEN + "Press SPACE to pause playback!" + self.COLOR_DEFAULT
            else:
                rospy.logfatal("You want to use simulated time for replaying this logfile. Therefore use_sim_time parameter has to bet set before initialization of the node."
                               + "Make sure you add following line at the top of your launchfile \n <param name=\"use_sim_time\" value=\"true\"\\> ")
                rospy.signal_shutdown()

    """ Central method to call for reading and publishing a logfile. """
    def read(self, filename):
        old_terminal_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())  # required to intercept keyboard input
            self._preprocess(filename)
            self._createPublishers()
            self._process(filename)
            while not rospy.is_shutdown():
                rospy. sleep(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)  # restore console settings

    def _preprocess(self, filename):
        self._laserIDs = set()
        self._cameraIDs = set()

        self._numLaserscans = 0
        self._numLaserscanAnnotations = 0
        self._numImages = 0
        self._numUnknownReadings = 0

        rospy.loginfo("Preprocessing logfile %s" % filename)
        self._lineCounter = 1

        if not os.path.isfile(filename):
            rospy.logerr("Logfile does not exist: " + filename)
            sys.exit(1)

        with open(filename) as f:
            for line in f:
                if not line.startswith("#"):
                    readingType, timestamp, content = self._parseReading(line, quiet=False)    
                    if readingType is not None:      
                        # Timestamps may be -1 to signal that the sensor itself did not provide any timestamp
                        if timestamp >= 0 and not self._isParam(readingType):      
                            self._earliestTimestamp = min(self._earliestTimestamp, timestamp)
                            self._latestTimestamp   = max(self._latestTimestamp, timestamp)

                        # Different types of sensor readings
                        if self._isLaserscan(readingType):
                            self._numLaserscans += 1
                            self._laserIDs.add( self._getLaserID(readingType, content) )

                        elif self._isLaserscanAnnotations(readingType):
                            self._numLaserscanAnnotations += 1
                            self._preprocessLaserscanAnnotations(readingType, timestamp, content)

                        elif self._isImage(readingType):
                            self._numImages += 1
                            numSubImages = int(content[0])
                            for i in xrange(0, numSubImages):
                                numFieldsPerImage = 22
                                subImageContent = content[1+i*numFieldsPerImage:1+(i+1)*numFieldsPerImage]
                                self._cameraIDs.add( self._getCameraID(readingType, subImageContent) )

                        elif self._isParam(readingType):
                            pass

                        else:
                            rospy.logwarn("Unknown reading of type %s in line %d" % (readingType, self._lineCounter) )
                            self._numUnknownReadings += 1

                self._lineCounter += 1

        # Try to open binary file for image data, if any
        if self._numImages > 0:
            binaryFilename = filename + ".bin"
            try:
                self._binaryFile = open(binaryFilename, "rb")
            except Exception:
                rospy.logwarn("Logfile contains images, but failed to open associated binary file %s!" % binaryFilename)

        # Print some statistics
        rospy.loginfo("Preprocessing finished, logfile contains %d laserscans, %d laserscan annotations, %d images and %d unknown readings!"
                      % (self._numLaserscans, self._numLaserscanAnnotations, self._numImages, self._numUnknownReadings)  )

        rospy.loginfo("Earliest timestamp is at %.3f, latest timestamp at %.3f seconds (duration: %.3f sec)" % (self._earliestTimestamp, self._latestTimestamp, self._latestTimestamp - self._earliestTimestamp) )

        rospy.loginfo("Laser ID(s) in logfile: %s" % ', '.join(self._laserIDs))
        rospy.loginfo("Camera ID(s) in logfile: %s" % ', '.join(self._cameraIDs))
        rospy.loginfo("Found occlusions in {} frames".format(len(filter(lambda x: len(x) > 0, self._occludedIDsPerTimestamp.values()))))

    def _parseReading(self, line, quiet=False):
        readingType = None
        timestamp = None
        content = None

        try:
            fields = line.split()
            if fields:
                readingType = fields[0].strip()
                if len(fields) < 4:
                    if not quiet:
                        rospy.logwarn("Malformed reading of type %s in line %d" % (readingType, self._lineCounter))
                else:
                    timestamp = float(fields[-3])
                    content = fields[1:-3]
        except Exception as e:
            rospy.logwarn("Parsing problem in line %d: %s" % (self._lineCounter, e) )
        
        return readingType, timestamp, content

    def _isLaserscan(self, readingType):
        return readingType.startswith("RAWLASER") or readingType.startswith("ROBOTLASER")

    def _getLaserID(self, readingType, content):
        return readingType.lower().strip()

    def _isLaserscanAnnotations(self, readingType):
        return readingType.startswith("LABELEDLASER")

    def _isImage(self, readingType):
        return readingType == "IMAGE_V2"

    def _getCameraID(self, readingType, content):
        return content[-1].strip()

    def _isParam(self, readingType):
        return readingType == "PARAM"

    def _createPublishers(self):
        rospy.loginfo("Creating publishers...")
        topics = []

        laserIndex = 0
        for laserID in self._laserIDs:
            suffix = ("_%s" % laserIndex if len(self._laserIDs) > 1 else "")

            topic = "laser%s" % suffix
            topics.append(topic)
            self._laserscanPublishers[laserID] = rospy.Publisher(topic, LaserScan, queue_size=self.queueSize)
            
            topic = "laser%s_annotations" % suffix
            topics.append(topic)
            self._laserscanAnnotationsPublishers[laserID] = rospy.Publisher(topic, LaserscanSegmentation, queue_size=self.queueSize)
            
            laserIndex += 1

        cameraIndex = 0
        for cameraID in self._cameraIDs:
            suffix = ("_%s" % cameraIndex if len(self._cameraIDs) > 1 else "")

            topic = "camera%s/image" % suffix
            topics.append(topic)
            self._imagePublishers[cameraID] = rospy.Publisher(topic, Image, queue_size=5)
            
            cameraIndex += 1

        for topic in topics:
            rospy.loginfo("  Topic: " + rospy.resolve_name(topic))

    def _process(self, filename):
        rospy.loginfo("Waiting %.1f seconds before playing back first message" % self.initialWait)
        time.sleep(self.initialWait)

        rospy.loginfo("Now playing back logfile %s" % filename)
        self._lineCounter = 1

        self._sensorsProcessedInCurrentFrame = set()
        self._lastFrameStartedAt = rospy.Time.now().to_sec() if self.timingMode != LogfilePublisher.TimingMode.use_timestamps else self._earliestTimestamp
        self._lastSensorTimestamp = None
        self._remappedTimestamp = rospy.Time(0)

        with open(filename) as f:
            for line in f:
                if rospy.is_shutdown():
                    self._quit()

                if not line.startswith("#"):
                    readingType, timestamp, content = self._parseReading(line, quiet=True)

                    if timestamp < self.startAt:
                        continue
                    elif timestamp > self.endAt:
                        break
                    annotationRead = False 
                    # Different types of sensor readings
                    if self._isLaserscan(readingType):
                        laserID = self._getLaserID(readingType, content)
                        self._newSensorReading(timestamp, "laser_%s" % laserID)
                        self._processLaserscan(readingType, self._remappedTimestamp, content)
                        self._lastLaserscanTimestamp = self._remappedTimestamp
                        self._lastLogTimestamp = timestamp
                        self._lastLaserID = laserID

                    elif self._isLaserscanAnnotations(readingType):
                        # Using timestamp of last laserscan reading here to allow exact-time synchronization
                        self._processLaserscanAnnotations(readingType, self._lastLaserscanTimestamp, content)
                        annotationRead = True

                    elif self._isImage(readingType):
                        self._newSensorReading(timestamp, "camera_%s" % self._getCameraID(readingType, content))
                        self._processImage(readingType, self._remappedTimestamp, content)
                    
                    if (self.publishTracks and annotationRead):
                        if (self._currentAnnotationMsg.has_key(self._lastLaserID) and self._currentLaserMsgs.has_key(self._lastLaserID)):
                            self._toTrackConverter.newSegmentationReceived(self._currentLaserMsgs[self._lastLaserID], self._currentAnnotationMsg[self._lastLaserID], self._occludedIDsPerTimestamp.get(self._lastLogTimestamp, []))

        rospy.loginfo("Playback of logfile has finished!")
       

    def _newSensorReading(self, timestamp, sensorId):
        if not timestamp < 0:
            if sensorId in self._sensorsProcessedInCurrentFrame or self._lastSensorTimestamp is None:
                self._beginNewFrame(timestamp)
                self._sensorsProcessedInCurrentFrame.clear()

        self._sensorsProcessedInCurrentFrame.add(sensorId)

    def _beginNewFrame(self, sensorTimestamp):
        # Check for key presses
        self._singleStep = False
        while True:
            if rospy.is_shutdown():
                self._quit()

            if self._keyPressed():
                key = sys.stdin.read(1)

                if key == '\x20':  # SPACE
                    if self.publishClock:
                        self._paused = not self._paused
                        if self._paused:
                            print self.COLOR_YELLOW + "Paused! Press SPACE to continue, S for single step..." + self.COLOR_DEFAULT
                        else:
                            print self.COLOR_GREEN + "Continuing..." + self.COLOR_DEFAULT
                    else:
                        print self.COLOR_RED + "Pause / single step mode requires publish_clock argument to be true" + self.COLOR_DEFAULT

                elif key == '\x73' and self._paused:  # s
                    print self.COLOR_GREEN + "Proceeding one single step!" + self.COLOR_DEFAULT
                    self._singleStep = True

            if not self._paused or self._singleStep:
                break

        # Determine start of next frame
        if self.timingMode == LogfilePublisher.TimingMode.immediate:
            self._frameStartsAt = self._lastFrameStartedAt
            
        elif self.timingMode == LogfilePublisher.TimingMode.fixed_rate:
            self._frameStartsAt = self._lastFrameStartedAt + 1.0 / self.rate

        elif self.timingMode == LogfilePublisher.TimingMode.use_timestamps or self.timingMode == LogfilePublisher.TimingMode.use_timestamps_with_remapping:
            duration = (sensorTimestamp - self._lastSensorTimestamp) if self._lastSensorTimestamp is not None else 0
            self._frameStartsAt = self._lastFrameStartedAt + duration / self.rate
           

        # Wait until it's time to publish the new frame.
        while rospy.Time.now().to_sec() < self._frameStartsAt:
            time.sleep(0.001)
            if rospy.is_shutdown():
                self._quit()

        # Use timestamp from logfile, or remap into current time frame if desired
        if self.timingMode == LogfilePublisher.TimingMode.use_timestamps:
            # Use value from logfile if possible
            self._remappedTimestamp = rospy.Time(sensorTimestamp)
        else:
            # Remap into current timeframe
            self._remappedTimestamp = rospy.Time(self._frameStartsAt)

        self._lastFrameStartedAt = self._frameStartsAt
        self._lastSensorTimestamp = sensorTimestamp

    def _processLaserscan(self, readingType, remappedTimestamp, content):
        # FIXME: For type ROBOTLASER, we should publish the robot/sensor pose using TF and use the correct TF frame
        laserscan = LaserScan()
        laserscan.header = Header(stamp=remappedTimestamp, frame_id="odom", seq=self._laserscanCounter)

        if readingType.startswith("RAWLASER") or readingType.startswith("ROBOTLASER"):
            laserscan.angle_min = float(content[1])
            laserscan.angle_max = laserscan.angle_min + float(content[2])
            laserscan.angle_increment = float(content[3])
            laserscan.time_increment = 0
            laserscan.scan_time = 0.0  # FIXME
            laserscan.range_min = 0
            laserscan.range_max = float(content[4])

            numRanges = int(content[7])
            for i in xrange(0, numRanges):
                laserscan.ranges.append( float(content[8 + i]) )

            numRemissions = int(content[8 + numRanges])
            for i in xrange(0, numRemissions):
                laserscan.intensities.append( float(content[9 + numRanges + i]) )

        else:
            rospy.logwarn("Unsupported laser of type %s in line %d" % (readingType, self._lineCounter) )

        publisher = self._laserscanPublishers[ self._getLaserID(readingType, content) ]
        publisher.publish(laserscan)
        self._laserscanCounter += 1
        if (self.publishTracks):
            self._currentLaserMsgs[self._getLaserID(readingType, content)] = laserscan

    def _processLaserscanAnnotations(self, readingType, remappedTimestamp, content):
        laserscanSegmentation = LaserscanSegmentation()
        laserscanSegmentation.header = Header(stamp=remappedTimestamp, frame_id="odom", seq=self._laserscanAnnotationCounter)

        segmentMap = dict()  # key is label ID, value is the set of laser point indices belonging to the segment

        # Read labelled laserscan from logfile and fill segmentMap
        numForegroundLabels = int(content[0])
        numTrackLabels = int(content[1 + numForegroundLabels])
        numAssignedTrackLabels = int(content[2 + numForegroundLabels])

        offset = 3 + numForegroundLabels
        for laserPointIndex in xrange(0, numTrackLabels):
            numTrackLabelsForThisPoint = int(content[offset])
            offset += 1
            for trackLabelIndex in xrange(0, numTrackLabelsForThisPoint):
                trackLabel = int(content[offset])
                offset += 1
                if not trackLabel in segmentMap:
                    segmentMap[trackLabel] = set()
                segmentMap[trackLabel].add(laserPointIndex)
                
        # Convert content of segmentMap into LaserscanSegment messages
        for trackLabel, laserPointIndices in segmentMap.iteritems():
            laserscanSegment = LaserscanSegment(label=trackLabel)
            for laserPointIndex in laserPointIndices:
                laserscanSegment.measurement_indices.append(laserPointIndex)
            laserscanSegmentation.segments.append(laserscanSegment)

        # Publish LaserscanSegmentation message
        publisher = self._laserscanAnnotationsPublishers[ self._lastLaserID ]
        publisher.publish(laserscanSegmentation)
        self._laserscanAnnotationCounter += 1
        if (self.publishTracks):
            self._currentAnnotationMsg[ self._lastLaserID ] = laserscanSegmentation

    
    
    def _preprocessLaserscanAnnotations(self, readingType, timestamp, content):
        segmentMap = dict()  # key is label ID, value is the set of laser point indices belonging to the segment

        # Read labelled laserscan from logfile and fill segmentMap
        numForegroundLabels = int(content[0])
        numTrackLabels = int(content[1 + numForegroundLabels])
        numAssignedTrackLabels = int(content[2 + numForegroundLabels])

        offset = 3 + numForegroundLabels
        for laserPointIndex in xrange(0, numTrackLabels):
            numTrackLabelsForThisPoint = int(content[offset])
            offset += 1
            for trackLabelIndex in xrange(0, numTrackLabelsForThisPoint):
                trackLabel = int(content[offset])
                offset += 1
                if not trackLabel in segmentMap:
                    segmentMap[trackLabel] = set()
                segmentMap[trackLabel].add(laserPointIndex)
                
        # Convert content of segmentMap into LaserscanSegment messages
        if not self._occludedIDsPerTimestamp.has_key(timestamp):
            self._occludedIDsPerTimestamp[timestamp] = []
        for trackLabel in segmentMap.keys():
            if (not self._activeTrackIDs.has_key(trackLabel) and not self._deletetTrackIDs.has_key(trackLabel)):
                self._activeTrackIDs[trackLabel] = timestamp
            elif (self._activeTrackIDs.has_key(trackLabel)):
                self._activeTrackIDs[trackLabel] = timestamp
            elif (self._deletetTrackIDs.has_key(trackLabel)):
                occludedTrack = dict()
                occludedTrack['t_begin'] = self._deletetTrackIDs[trackLabel]
                occludedTrack['t_end'] = timestamp
                occludedTrack['label'] = trackLabel
                rospy.logdebug("Track is occluded from time {} to {}".format(occludedTrack['t_begin'],occludedTrack['t_end'] ))
                if (self._occludedIDs.has_key(occludedTrack['t_begin'])):
                    self._occludedIDs[occludedTrack['t_begin']].append(occludedTrack)
                else:
                    self._occludedIDs[occludedTrack['t_begin']] = [occludedTrack ,]
                del self._deletetTrackIDs[trackLabel]
                sorted_timestamps = self._occludedIDsPerTimestamp.keys()
                for t in sorted_timestamps:
                    if (t < occludedTrack['t_end'] and t >= occludedTrack['t_begin']):
                        self._occludedIDsPerTimestamp[t].append(occludedTrack['label'])
    
                

        disappearedLabels = filter(lambda g: self._activeTrackIDs[g] != timestamp, self._activeTrackIDs.keys())
        if (len(disappearedLabels)):
            rospy.logdebug("{} track labels disappeared".format(len(disappearedLabels)))
        for disappearedLabel in disappearedLabels:
            self._deletetTrackIDs[disappearedLabel] = timestamp
            del self._activeTrackIDs[disappearedLabel]
            
                
                
                
            
            
            

    def _processImage(self, readingType, remappedTimestamp, content):
        if self._binaryFile is None:
            return
            
        numSubImages = int(content[0])
        for i in xrange(0, numSubImages):
            numFieldsPerImage = 22
            subImageContent = content[1+i*numFieldsPerImage:1+(i+1)*numFieldsPerImage]

            inputType = int(subImageContent[0])
            numBytes = int(subImageContent[19])
            imageStartsAtByte = int(subImageContent[20])

            imageBuffer = None
            try:
                self._binaryFile.seek(imageStartsAtByte)
                rawData = self._binaryFile.read(numBytes)
                imageBuffer = numpy.fromstring(rawData, dtype=numpy.uint8)
            except Exception, e:
                rospy.logwarn("Failed to read binary data of image #%d in line %d. Reason: %s" % (i, self._lineCounter, e) )
                continue

            cvImage = cv2.imdecode(imageBuffer, flags=1)
            if len(cvImage.shape) < 3:  # make sure there are 3 dimensions (not channels!), otherwise following call fails
                cvImage.shape = (cvImage.shape[0], cvImage.shape[1], 1)

            image = self._cvBridge.cv2_to_imgmsg(cvImage)
            image.header = Header(stamp=remappedTimestamp, frame_id="odom", seq=self._imageCounter)

            publisher = self._imagePublishers[ self._getCameraID(readingType, subImageContent) ]
            publisher.publish(image)
            self._imageCounter += 1

    def _publishClock(self, clockRate):
        print 'Starting to publish simulated time on /clock and setting /use_sim_time to true.'
        print 'Simulated time is running at %fx speed.' % clockRate
        clockPublisher = rospy.Publisher( "/clock", Clock, queue_size=1 )

        if self.timingMode == LogfilePublisher.TimingMode.use_timestamps:
            while isinf(self._earliestTimestamp):
                time.sleep(0.01)
            currentSecs = self._earliestTimestamp - 0.5
        else:
            currentSecs = time.time()

        lastRealTime = time.time()
        while not rospy.is_shutdown():
            currentRealTime = time.time()
            elapsedRealTime = currentRealTime - lastRealTime
            lastRealTime = currentRealTime
            
            if not self._paused or self._singleStep:
                currentSecs += elapsedRealTime * clockRate

            clock = Clock()
            clock.clock = rospy.Time.from_sec(currentSecs) 
            try:
                clockPublisher.publish(clock)
            except Exception:
                print 'Publisher closed' 
            time.sleep(0.01)

    def _keyPressed(self):
        try:
            return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        except Exception:
            return False

    def _quit(self):
        if self._binaryFile is not None:
            self._binaryFile.close()
        sys.exit(0)
        
""" 
Outputs spencer_tracking_msgs/TrackedPersons and spencer_tracking_msgs/DetectedPersons based upon a
srl_laser_segmentation/LaserscanSegmentation message and a sensor_msgs/Laserscan. This assumes that the segmentation
labels correspond to (groundtruth) person track IDs which are consistent over time. Does not work with arbitrary segmentation
output as from simple jump-distance clustering.
"""
class SegmentationToTrackConverter(object):
    def __init__(self):
        trackedPersonsTopic = rospy.resolve_name("tracked_persons")
        detectedPersonsTopic = rospy.resolve_name("detected_persons")
        
        self.trackedPersonsPublisher = rospy.Publisher(trackedPersonsTopic, TrackedPersons, queue_size=100000)
        self.detectedPersonsPublisher = rospy.Publisher(detectedPersonsTopic, DetectedPersons, queue_size=100000)

        self._detectionIdCounter = 0
        self._lastDataStamp = None
        self._firstTrackEncounterLookup = dict()
        self._previousCentroidLookup = dict()

        rospy.loginfo("Publishing detected and tracked persons from laser scan segmentation at %s and %s"
                      % (detectedPersonsTopic, trackedPersonsTopic) )


    def newSegmentationReceived(self, laserscan, laserscanSegmentation, occludedTracks):
        if (laserscan.header.stamp != laserscanSegmentation.header.stamp):
            rospy.logwarn("Different timestamps laser: {} segmentation:{}".format(laserscan.header.stamp, laserscanSegmentation.header.stamp))
        currentStamp = laserscanSegmentation.header.stamp
        pointCount = len(laserscan.ranges)
        cartesianCoordinates = []
        rospy.logdebug("Currently occluded track labels: {}".format(occludedTracks))

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
                if dt > 0:
                    velocity = accumulatedVelocity / dt
                else:
                    velocity = 0

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
            trackedPerson.is_matched = True

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
        
        for occludedTrackID in occludedTracks:
            centroidHistory = self._previousCentroidLookup[occludedTrackID]
            last_centroid = centroidHistory[-1][1]
            # Initialize TrackedPerson message
            occludedPerson = TrackedPerson()
 
            occludedPerson.track_id = occludedTrackID
            occludedPerson.age = currentStamp - self._firstTrackEncounterLookup[occludedTrackID]
            occludedPerson.detection_id = 0
            occludedPerson.is_occluded = True
            occludedPerson.is_matched = False
 
            # Set position
            LARGE_VARIANCE = 99999999
            occludedPerson.pose.pose.position.x = last_centroid[0]
            occludedPerson.pose.pose.position.y = last_centroid[1]
            occludedPerson.pose.pose.position.z = last_centroid[2]
            
            occludedPerson.pose.covariance[2 * 6 + 2] = occludedPerson.pose.covariance[3 * 6 + 3] = occludedPerson.pose.covariance[4 * 6 + 4] = LARGE_VARIANCE  # z pos, roll, pitch

             
            trackedPersons.tracks.append(occludedPerson)

             
        
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