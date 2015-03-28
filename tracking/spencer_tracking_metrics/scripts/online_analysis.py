#!/usr/bin/env python
import rospy, sys, time, message_filters, numpy, os
import tf, tf.transformations as transformations
import geometry_msgs.msg
from os.path import expanduser
from std_msgs.msg import Float32 as ResultMsg
from spencer_tracking_msgs.msg import TrackedPersons
from spencer_tracking_metrics import *
import spencer_tracking_metrics.ospa as ospa
from spencer_tracking_metrics.approxsync import ApproximateSynchronizer
import std_msgs


def newTracksAvailable(trackedPersons):
    if stopReceivingNewData:
        return

    global numTrackerCycles
    numTrackerCycles += 1

def newTracksAndGroundtruthAvailable(trackedPersons, groundtruthPersons):
    if stopReceivingNewData:
        return

    global lastDataReceivedAt, firstTracksReceived
    if not trackedPersons.tracks and not firstTracksReceived:
        return

    if groundtruthPersons.header.frame_id != trackedPersons.header.frame_id:
        # Wait for transform to become available
        try:
            transformListener.waitForTransform(trackedPersons.header.frame_id, groundtruthPersons.header.frame_id, groundtruthPersons.header.stamp, rospy.Duration(0.2))
            transformMatrix = transformListener.asMatrix(trackedPersons.header.frame_id, groundtruthPersons.header)
        except tf.Exception as e:
            rospy.logerr("Failed to lookup transform from %s to %s: %s" % (groundtruthPersons.header.frame_id, trackedPersons.header.frame_id, str(e)) )
            return

        # Transform pose of each groundtruth person
        for groundtruthPerson in groundtruthPersons.tracks:
            groundtruthPerson.pose.pose = transformPose(transformMatrix, groundtruthPerson.pose)
        groundtruthPersons.header.frame_id = trackedPersons.header.frame_id

    lastDataReceivedAt = time.time()

    if not firstTracksReceived:
        rospy.loginfo("First set of tracked persons and groundtruth tracked persons has been received!")
    firstTracksReceived = True

    trackedPersonsArray.append(trackedPersons)
    groundTruthArray.append(groundtruthPersons)

    if ospaFlag:
        ospaAnalysis.doOspaAnalysis(trackedPersons, groundtruthPersons)

def writeResults(results, filename):
    with open(filename, 'w') as f:
        # Clearmot results
        if type(results) is ClearMotResults:
            for key in results.iterkeys():
                f.write(key.ljust(max(len(key)+1, 16)) + " ")
            f.write("\n")
            for key in results.iterkeys():
                value = ("%.6f" % results[key]) if isinstance(results[key], float) else str(results[key])
                f.write(value.ljust(max(len(key)+1, 16)) + " ")
        # OSPA results
        elif type(results) is list:
            f.write("OSPA distance ; Localization Error ; Cardinalization Error ; Labeling Error ;  Timestamp [s] \n")
            for result in results:
                f.write(' ; '.join(map(str, result)) + "\n")

    rospy.loginfo("Results have been written to file " + filename)

def transformPose(mat44, ps):
    # pose44 is the given pose as a 4x4
    pose44 = numpy.dot(tf.listener.xyz_to_mat44(ps.pose.position), tf.listener.xyzw_to_mat44(ps.pose.orientation))

    # txpose is the new pose in target_frame as a 4x4
    txpose = numpy.dot(mat44, pose44)

    # xyz and quat are txpose's position and orientation
    xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
    quat = tuple(transformations.quaternion_from_matrix(txpose))

    # assemble return value Pose
    return geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat))


def saveAllParams(filename):
    with open(filename, 'w') as f:
        params = rospy.get_param_names()
        for param in params:
            f.write(param)
            f.write(" ; ")
            f.write(str(rospy.get_param(param)))
            f.write("\n")


if __name__ == '__main__':


    global evaluation_folder
    dateStamp = time.strftime("%Y-%m-%d-%H-%M")
    evaluation_folder = expanduser("~") + "/tracking_evaluation/tmp/"+dateStamp+"/"
    if not os.path.exists(evaluation_folder):
        print evaluation_folder
        os.makedirs(evaluation_folder)


    rospy.init_node("online_tracking_analysis")

    saveAllParams(evaluation_folder+"params_%s.txt" % dateStamp)

    # Read parameters
    matchingThreshold = rospy.get_param("~matching_threshold", 1.0)  # in meters
    approximateSync = rospy.get_param("~approximate_sync", False)  # use ApproximateTimeSynchronizer as opposed to ExactTimeSynchronizer?
    syncSlop = rospy.get_param("~sync_slop", 0.2)  # in seconds; messages closer in time than this are output by the approximate time synchronizer immediately
    syncQueueSize = rospy.get_param("~sync_queue_size", 50)  # size of the synchronizer queue
    clearMetrics = rospy.get_param("~clear_metrics", False)  # calculate ClearMetrics? Disabled by default, seems to be buggy, crashing sometimes!
    pyMot = rospy.get_param("~pymot", True) # calculate PyMot metrics?
    global ospaFlag
    ospaFlag = rospy.get_param("~ospa", True) # calculate OSPA metrics?

    # Initialize OSPA evaluation
    if ospaFlag:
        global ospaAnalysis
        ospaAnalysis = ospa.OspaAnalysis(a=1,p=2,c=10)



    clearmotPublisher = rospy.Publisher("/pymot_result", ResultMsg, queue_size=1)
    
    
    
    # Initialize state variables
    trackedPersonsArray = []
    groundTruthArray = []
    firstTracksReceived = False
    stopReceivingNewData = False
    numTrackerCycles = 0

    # Create subscriber filters and approximate time synchronizer
    transformListener = tf.TransformListener()

    synchronizedSubscribers = [ 
        message_filters.Subscriber("/spencer/perception/tracked_persons", TrackedPersons),
        message_filters.Subscriber("groundtruth", TrackedPersons)
    ]

    if approximateSync:
        timeSynchronizer = ApproximateSynchronizer(syncSlop, synchronizedSubscribers, syncQueueSize)
    else:
        timeSynchronizer = message_filters.TimeSynchronizer(synchronizedSubscribers, syncQueueSize)

    timeSynchronizer.registerCallback(newTracksAndGroundtruthAvailable)

    # Listen for tracked persons to count number of total tracker cycles
    trackedPersonsSubscriber = rospy.Subscriber("/spencer/perception/tracked_persons", TrackedPersons, newTracksAvailable, queue_size=1000)

    # Listen for tracked persons and groundtruth tracked persons messages to determine metrics
    rospy.loginfo("Listening for approximately synchronized tracked persons and groundtruth tracked persons messages...")
    lastProgress = 0
    while not rospy.is_shutdown():
        time.sleep(1.0)

        if trackedPersonsArray:
            # Display progress
            progress = len(trackedPersonsArray) / 100
            if progress != lastProgress:
                lastProgress = progress
                rospy.loginfo("%d tracking cycles received" % len(trackedPersonsArray))

            # Stop listening if no new tracks have been received for a while
            if time.time() - lastDataReceivedAt > 10.0:
                rospy.loginfo("Stopping to listen since no new tracks were received for more than 10 seconds!")
                break

    stopReceivingNewData = True

    if not trackedPersonsArray or not groundTruthArray:
        rospy.logerr("No data received, quitting without analysis")
        sys.exit(1)

    rospy.loginfo("Recorded %d tracking cycles in total!" % len(trackedPersonsArray))

    # Aggregate input data
    clearMotInput = ClearMotInput(trackedPersonsArray, groundTruthArray, matchingThreshold)

    # Run analysis
    

    clearResults = []
    if clearMetrics:
        rospy.loginfo("Running ClearMetrics analysis...")
        clearResults = calculateClearMetrics(clearMotInput)
        clearResults["cycles_synched_with_gt"] = len(groundTruthArray)
        clearResults["tracker_cycles"] = numTrackerCycles
        writeResults(clearResults, evaluation_folder+"clear_metrics_%s.txt" % dateStamp)
    
    pyMotResults = []
    if pyMot:
        rospy.loginfo("Running PyMot analysis...")
        pyMotResults = calculatePyMot(clearMotInput)
        pyMotResults["cycles_synched_with_gt"] = len(groundTruthArray)
        pyMotResults["tracker_cycles"] = numTrackerCycles
        msg = ResultMsg()
        msg.data = pyMotResults["mota"]
        clearmotPublisher.publish(msg)
        writeResults(pyMotResults, evaluation_folder+"pymot_metrics_%s.txt" % dateStamp)

    if ospaFlag:
        ospaAnalysis.writeResultsToFile(evaluation_folder+"ospa_%s.txt" % dateStamp)
        
    rospy.loginfo("ClearMetrics results:\n" + str(clearResults))
    rospy.loginfo("PyMot results:\n" + str(pyMotResults))


