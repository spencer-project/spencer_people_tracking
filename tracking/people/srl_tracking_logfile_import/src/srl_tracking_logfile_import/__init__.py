import rospy, sys, os.path, time, cv2, cv_bridge, numpy, select, termios, tty
from threading import Thread
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Image
from srl_laser_segmentation.msg import LaserscanSegmentation, LaserscanSegment


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
        self.initialWait = rospy.get_param("~initial_wait", 1.0)      # seconds to wait after advertising publishers before publishing first message
        ### End of configurable parameters ###

        self._laserscanPublishers = dict()
        self._laserscanAnnotationsPublishers = dict()
        self._imagePublishers = dict()

        self._laserscanCounter = 0
        self._laserscanAnnotationCounter = 0
        self._imageCounter = 0

        self._paused = False
        self._singleStep = False

        self._binaryFile = None
        self._cvBridge = cv_bridge.CvBridge()

        self.COLOR_RED = "\x1b[31m"
        self.COLOR_GREEN = "\x1b[32m"
        self.COLOR_YELLOW = "\x1b[33m"
        self.COLOR_DEFAULT = "\x1b[39;49m"

        # Start clock generator if requested
        if self.publishClock:
            rospy.set_param('/use_sim_time', True)
            clockRate = rospy.get_param('~clock_rate', 1.0)
            clockThread = Thread(target=self._publishClock, args=(clockRate, ))
            clockThread.daemon = True
            clockThread.start()
            time.sleep(0.5)
            print self.COLOR_GREEN + "Press SPACE to pause playback!" + self.COLOR_DEFAULT

    """ Central method to call for reading and publishing a logfile. """
    def read(self, filename):
        old_terminal_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())  # required to intercept keyboard input
            self._preprocess(filename)
            self._createPublishers()
            self._process(filename)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)  # restore console settings

    def _preprocess(self, filename):
        self._laserIDs = set()
        self._cameraIDs = set()

        self._numLaserscans = 0
        self._numLaserscanAnnotations = 0
        self._numImages = 0
        self._numUnknownReadings = 0

        self._earliestTimestamp = float("inf")
        self._latestTimestamp = float("-inf")

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
                        if timestamp >= 0:      
                            self._earliestTimestamp = min(self._earliestTimestamp, timestamp)
                            self._latestTimestamp   = max(self._latestTimestamp, timestamp)

                        # Different types of sensor readings
                        if self._isLaserscan(readingType):
                            self._numLaserscans += 1
                            self._laserIDs.add( self._getLaserID(readingType, content) )

                        elif self._isLaserscanAnnotations(readingType):
                            self._numLaserscanAnnotations += 1

                        elif self._isImage(readingType):
                            self._numImages += 1
                            numSubImages = int(content[0])
                            for i in xrange(0, numSubImages):
                                numFieldsPerImage = 22
                                subImageContent = content[1+i*numFieldsPerImage:1+(i+1)*numFieldsPerImage]
                                self._cameraIDs.add( self._getCameraID(readingType, subImageContent) )

                        elif self._isParam:
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
            self._laserscanPublishers[laserID] = rospy.Publisher(topic, LaserScan)
            
            topic = "laser%s_annotations" % suffix
            topics.append(topic)
            self._laserscanAnnotationsPublishers[laserID] = rospy.Publisher(topic, LaserscanSegmentation)
            
            laserIndex += 1

        cameraIndex = 0
        for cameraID in self._cameraIDs:
            suffix = ("_%s" % cameraIndex if len(self._cameraIDs) > 1 else "")

            topic = "camera%s/image" % suffix
            topics.append(topic)
            self._imagePublishers[cameraID] = rospy.Publisher(topic, Image)
            
            cameraIndex += 1

        for topic in topics:
            rospy.loginfo("  Topic: " + rospy.resolve_name(topic))

    def _process(self, filename):
        rospy.loginfo("Waiting %.1f seconds before playing back first message" % self.initialWait)
        rospy.sleep(self.initialWait)

        rospy.loginfo("Now playing back logfile %s" % filename)
        self._lineCounter = 1

        self._sensorsProcessedInCurrentFrame = set()
        self._lastFrameStartedAt = rospy.Time.now().to_sec()
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

                    # Different types of sensor readings
                    if self._isLaserscan(readingType):
                        laserID = self._getLaserID(readingType, content)
                        self._newSensorReading(timestamp, "laser_%s" % laserID)
                        self._processLaserscan(readingType, self._remappedTimestamp, content)
                        self._lastLaserscanTimestamp = self._remappedTimestamp
                        self._lastLaserID = laserID

                    elif self._isLaserscanAnnotations(readingType):
                        # Using timestamp of last laserscan reading here to allow exact-time synchronization
                        self._processLaserscanAnnotations(readingType, self._lastLaserscanTimestamp, content)

                    elif self._isImage(readingType):
                        self._newSensorReading(timestamp, "camera_%s" % self._getCameraID(readingType, content))
                        self._processImage(readingType, self._remappedTimestamp, content)

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
        clockPublisher = rospy.Publisher( "/clock", Clock )
        currentSecs = time.time()

        while not rospy.is_shutdown():
            if not self._paused or self._singleStep:
                currentSecs += 0.01 * clockRate

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
