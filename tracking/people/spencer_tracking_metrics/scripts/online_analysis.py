#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2015, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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


import rospy, sys, time, message_filters, numpy, os
import tf, tf.transformations as transformations
import geometry_msgs.msg
from os.path import expanduser
from std_msgs.msg import Float32 as ResultMsg
from spencer_tracking_msgs.msg import TrackedPersons, ImmDebugInfos, TrackingTimingMetrics
from spencer_tracking_metrics import *
import spencer_tracking_metrics.ospa as ospa
from spencer_tracking_metrics.approxsync import ApproximateSynchronizer
import spencer_tracking_metrics.aggregate_results_over_runs as aggregate_results_over_runs
import std_msgs
import pickle
import pprint
from time import sleep


def newTracksAvailable(trackedPersons):
    if stopReceivingNewData:
        return

    global numTrackerCycles
    numTrackerCycles += 1

def newTracksAndGroundtruthAvailable(trackedPersons, groundtruthPersons):
    if stopReceivingNewData:
        return

    global lastDataReceivedAt, firstTracksReceived
    if not trackedPersons and not firstTracksReceived:
        return

    if transformFlag:
        if groundtruthPersons.header.frame_id != trackedPersons.header.frame_id:
            # Wait for transform to become available
            try:
                transformListener.waitForTransform(trackedPersons.header.frame_id, groundtruthPersons.header.frame_id, groundtruthPersons.header.stamp, rospy.Duration(0.05))
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

## Uncomment for online analysis for direct output of mismatches
#     if ospaFlag:
#         ospaAnalysis.doOspaAnalysis(trackedPersons, groundtruthPersons)

    trackedPersonsArray.append(trackedPersons)
    groundTruthArray.append(groundtruthPersons)

    


def newDebugIngosAvailable(debugInfos):
    debugInfosArray.append(debugInfos)
    
    
def newTimingMetricAvailable(timingMetrics):
    timingMetricsArray.append(timingMetrics)




def writeResults(results, filename):
    with open(filename, 'w') as f:
        # Clearmot results
        if type(results) is ClearMotResults:
            sortedKeys = sorted(results.iterkeys())
            for key in sortedKeys:
                f.write(key.ljust(max(len(key)+1, 16)) + " ")
            f.write("\n")
            for key in sortedKeys:
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

    rospy.init_node("online_tracking_analysis")


    evaluationPrefix = rospy.get_param("~evaluation_prefix", "")
    if evaluationPrefix and not evaluationPrefix.endswith("/"):
        evaluationPrefix += "/"

    global evaluation_folder, ospaFlag, ospaAnalysis, transformFlag
    dateStamp = time.strftime("%Y-%m-%d-%H-%M")
    evaluation_base_folder = expanduser("~") + "/tracking_evaluation/" + evaluationPrefix
    evaluation_folder = evaluation_base_folder + dateStamp + "/"
    if not os.path.exists(evaluation_folder):
        print "Saving tracking metrics in folder: " + evaluation_folder
        os.makedirs(evaluation_folder)

    
    saveAllParams(evaluation_folder + "params_%s.txt" % dateStamp)

    # Read parameters
    matchingThreshold = rospy.get_param("~matching_threshold", 1.0)  # in meters
    approximateSync = rospy.get_param("~approximate_sync", False)  # use ApproximateTimeSynchronizer as opposed to ExactTimeSynchronizer?
    syncSlop = rospy.get_param("~sync_slop", 0.2)  # in seconds; messages closer in time than this are output by the approximate time synchronizer immediately
    syncQueueSize = rospy.get_param("~sync_queue_size", 50)  # size of the synchronizer queue
    pyMot = rospy.get_param("~pymot", True) # calculate PyMot metrics?
    ospaFlag = rospy.get_param("~ospa", False) # calculate OSPA metrics?
    immDebug = rospy.get_param("~imm_debug", False) # save imm debug infos?
    transformFlag = rospy.get_param("~transform_flag", True) # whether to transform groundtruth and tracker tracks into same frame. Only disable if they already are!!!
    timingMetrics = rospy.get_param("~timing_metrics", False) # save timing metrics?
    aggregateResults = rospy.get_param("~aggregate_results", False)  # aggregate results over all existing runs with the same evaluation prefix?
    numExpectedGTCycles = rospy.get_param("~num_expected_gt_cycles", 4000)  # number of expected synched GT cycles to include results in aggregated results (if enabled, see above)
    subscriberTimeout = rospy.get_param("~subscriber_timeout", 10.0) # calculate OSPA metrics?


    # Initialize OSPA evaluation
    if ospaFlag:
        ospaAnalysis = ospa.OspaAnalysis(a=10,p=1,c=matchingThreshold)



    motaPublisher = rospy.Publisher("/pymot_result", ResultMsg, queue_size=1)
    missmatchesPublisher = rospy.Publisher("/pymot_mismatches", ResultMsg, queue_size=1)



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
    if immDebug:
        immDebugSubscriber = rospy.Subscriber("/srl_nearest_neighbor_tracker/imm_debug_infos", ImmDebugInfos, newDebugIngosAvailable, queue_size=10)
        debugInfosArray = []
    if timingMetrics:
        timingMetricsSubscriber = rospy.Subscriber("/srl_nearest_neighbor_tracker/tracking_timing_metrics", TrackingTimingMetrics, newTimingMetricAvailable, queue_size=10)
        timingMetricsArray = []

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
            if time.time() - lastDataReceivedAt > subscriberTimeout:
                rospy.loginfo("Stopping to listen since no new tracks were received for more than %.1f seconds!" % subscriberTimeout)
                break

    stopReceivingNewData = True

    if not trackedPersonsArray or not groundTruthArray:
        rospy.logerr("No data received, quitting without analysis")
        sys.exit(1)

    totalDuration = (groundTruthArray[-1].header.stamp - groundTruthArray[0].header.stamp).to_sec()
    rospy.loginfo("Recorded %d tracking cycles in total over a duration (ROS time) of %.1f seconds!" % (len(trackedPersonsArray), totalDuration ) )

    # Run analysis
    
    # Aggregate input data
    clearMotInput = ClearMotInput(trackedPersonsArray, groundTruthArray, matchingThreshold)

    
    rospy.loginfo("Ignoring the following groundtruth track IDs in metrics calculations: " + str(rospy.get_param("~groundtruth_track_ids_to_ignore", [])))

    pyMotResults = []
    if pyMot:
        rospy.loginfo("Running PyMot analysis...")
        pyMotResults = calculatePyMot(clearMotInput)
        pyMotResults["cycles_synched_with_gt"] = len(groundTruthArray)
        pyMotResults["tracker_cycles"] = numTrackerCycles
        pyMotResults["duration"] = totalDuration
        msg = ResultMsg()
        if pyMotResults["cycles_synched_with_gt"] > numExpectedGTCycles:
            msg.data = pyMotResults["mota"]
        else:
            msg.data = -1
            rospy.logerr("Received less cycles_synched_with_gt than expected. Got %d, expected %d! Check that playback did not end prematurely and that proper synchronization is ensured! Maybe CPU load is too high?" % (pyMotResults["cycles_synched_with_gt"], numExpectedGTCycles) )

        motaPublisher.publish(msg)
        rospy.loginfo("Published pymot result %f" % msg.data)
        msg.data = pyMotResults["mismatches"]
        missmatchesPublisher.publish(msg)
        time.sleep(1)
        writeResults(pyMotResults, evaluation_folder+"pymot_metrics_%s.txt" % dateStamp)
    
    if ospaFlag:
        rospy.loginfo("Running OSPA analysis ...")
        for trackedPersons, groundtruthPersons in zip(trackedPersonsArray, groundTruthArray):
            ospaAnalysis.doOspaAnalysis(trackedPersons, groundtruthPersons)
        ospaAnalysis.writeResultsToFile(evaluation_folder+"ospa_%s.txt" % dateStamp)
        ospaAnalysis.printAverageResults()
        ospaAnalysis.printTotalResults()
    if immDebug:
        rospy.loginfo("Recorded {} imm debug messages".format(len(debugInfosArray)))
        with open(evaluation_folder+"imm_debug_infos_%s.pkl" % dateStamp,'wb') as f:
            pickle.dump(debugInfosArray, f)
        with open(evaluation_folder+"gt_array_%s.pkl" % dateStamp,'wb') as f:
            pickle.dump(groundTruthArray, f)
        with open(evaluation_folder+"track_array_%s.pkl" % dateStamp,'wb') as f:
            pickle.dump(trackedPersonsArray, f)
        
    if timingMetrics:
        rospy.loginfo("Save Timing Metrics ...")
        with open(evaluation_folder+"timing_metrics_%s.txt" % dateStamp,'w') as f:
            f.write('timestamp ; cycle_no ; track_count ; cycle_time ; average_cycle_time ; cpu_load ; average_cpu_load ; average_processing_rate ; elapsed_time ; elapsed_cpu_time\n')
            avgRateArray = []
            avgCpuLoadArray = []
            avgTrackCountArray = []
            for tm in timingMetricsArray:
                f.write('{};{};{};{};{};{};{};{};{};{}\n'.format(tm.header.stamp.to_sec(),tm.cycle_no, tm.track_count, tm.cycle_time, tm.average_cycle_time, tm.cpu_load, tm.average_cpu_load, tm.average_processing_rate, tm.elapsed_time, tm.elapsed_cpu_time))
                avgRateArray.append(tm.average_processing_rate)
                avgCpuLoadArray.append(tm.cpu_load)
                avgTrackCountArray.append(tm.track_count)

        if timingMetricsArray:
            overallAvgRate = numpy.median(avgRateArray[1:])  # skip first value since it is often too high
            worstCaseRate = numpy.min(avgRateArray[1:])  # skip first value since it is often too high
            overallAvgCpuLoad = numpy.median(avgCpuLoadArray[1:])
            overallAvgTrackCount = numpy.median(avgTrackCountArray[1:])
            maxTrackCount = numpy.array(avgTrackCountArray[1:]).max()
            rospy.loginfo("Overall average processing rate: %.1f Hz (Worst Case %.1f ), cpu load: %.1f%%, track count: %d (max. %d)" % (overallAvgRate, worstCaseRate, overallAvgCpuLoad, overallAvgTrackCount, maxTrackCount))
            quantile1 = numpy.percentile(avgRateArray[1:], 1)
            quantile3 = numpy.percentile(avgRateArray[1:], 3)
            quantile5 = numpy.percentile(avgRateArray[1:], 5)
            quantile10 = numpy.percentile(avgRateArray[1:], 10)
            rospy.loginfo(("Percentiles of Rate:1={} 3={} ; 5={} ; 10={}").format(quantile1, quantile3, quantile5, quantile10))
        else:
            rospy.logerr("No tracking timing metrics have been received! Check that the correct topic is being subscribed!")
        
    rospy.loginfo("PyMot results:\n" + pprint.pformat(pyMotResults))

    if aggregateResults:
        aggregate_results_over_runs.aggregateResults(evaluation_base_folder, numExpectedGTCycles)

