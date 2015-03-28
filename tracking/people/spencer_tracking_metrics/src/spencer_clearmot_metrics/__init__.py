import spencer_tracking_metrics.clearmetrics as clearmetrics
import spencer_tracking_metrics.pymot as pymot
import numpy, rospy

class ClearMotInput(object):
    """
    Input for both calculateClearMetrics() and calculatePyMot().

    trackedPersonsArray and groundTruthArray shall be lists of spencer_tracking_msgs/TrackedPersons.
    They must be of the same length; the array/list index corresponds to the frame number.

    It must be ensured that tracker output and groundtruth data are in the same coordinate frame.
    """
    def __init__(self, trackedPersonsArray, groundTruthArray, matchingThreshold):
        assert(len(trackedPersonsArray) == len(groundTruthArray))
        self.groundTruthArray = groundTruthArray
        self.trackedPersonsArray = trackedPersonsArray
        self.matchingThreshold = matchingThreshold

class ClearMotResults(dict):
    """
    Results returned by calculateClearMetrics() and calculatePyMot() will be stored in this dictionary
    """
    def __init__(self):
        self['fp'] = 0
        self['fn'] = 0
        self['mismatches'] = 0
        self['matches'] = 0
        self['gt_tracks'] = 0
        self['tracker_tracks'] = 0
        self['mota'] = 0.0
        self['motp'] = 0.0

def calculateClearMetrics(clearMotInput):
    """
    Calculate CLEARMOT metrics using the implementation by Matej Smid
    """

    # Remap track IDs (in groundtruth and tracker output) to zero-based indices
    rospy.logdebug("Remapping track IDs (in tracker output and groundtruth) to zero-based indices")
    zeroBasedGTIdMap = dict()
    zeroBasedTrackIdMap = dict()
    for cycleIdx in xrange(0, len(clearMotInput.groundTruthArray)):
        # Make sure input data is in the same coordinate frame
        assert(clearMotInput.groundTruthArray[cycleIdx].header.frame_id == clearMotInput.trackedPersonsArray[cycleIdx].header.frame_id)
    
        # Groundtruth
        for gtPerson in clearMotInput.groundTruthArray[cycleIdx].tracks:
            if not gtPerson.track_id in zeroBasedGTIdMap:
                zeroBasedGTIdMap[gtPerson.track_id] = len(zeroBasedGTIdMap)
        # Tracker output
        for trackedPerson in clearMotInput.trackedPersonsArray[cycleIdx].tracks:
            if not trackedPerson.track_id in zeroBasedTrackIdMap:
                zeroBasedTrackIdMap[trackedPerson.track_id] = len(zeroBasedTrackIdMap)

    # Format input data
    rospy.logdebug("Formatting input data into a format readable by ClearMetrics...")
    cmThreshold = clearMotInput.matchingThreshold
    cmGT = dict()
    cmTracks = dict()
    for cycleIdx in xrange(0, len(clearMotInput.groundTruthArray)):
        # Groundtruth
        cmGT_thisFrame = [ None for i in xrange(0, len(zeroBasedGTIdMap))]
        for gtPerson in clearMotInput.groundTruthArray[cycleIdx].tracks:
            if not gtPerson.is_occluded:
                cmId = zeroBasedGTIdMap[gtPerson.track_id]
                cmGT_thisFrame[cmId] = numpy.array( [ gtPerson.pose.pose.position.x, gtPerson.pose.pose.position.y ] )

        # Remove unnecessary None entries from the right
        while True:
            if cmGT_thisFrame and cmGT_thisFrame[-1] is None:
                del cmGT_thisFrame[-1]
            else:
                break

        cmGT[cycleIdx] = cmGT_thisFrame
        
        # Tracker output
        cmTracks_thisFrame = [ None for i in xrange(0, len(zeroBasedTrackIdMap))]
        for trackedPerson in clearMotInput.trackedPersonsArray[cycleIdx].tracks:
            # FIXME: Is it correct to handle occluded tracks as if they were matched!?
            cmId = zeroBasedTrackIdMap[trackedPerson.track_id]
            cmTracks_thisFrame[cmId] = numpy.array( [ trackedPerson.pose.pose.position.x, trackedPerson.pose.pose.position.y ] )

        # Remove unnecessary None entries from the right
        while True:
            if cmTracks_thisFrame and cmTracks_thisFrame[-1] is None:
                del cmTracks_thisFrame[-1]
            else:
                break

        cmTracks[cycleIdx] = cmTracks_thisFrame
    
    # Run analysis
    rospy.logdebug("Now executing actual ClearMetrics analysis...")
    cm = clearmetrics.ClearMetrics(cmGT, cmTracks, cmThreshold)
    cm.match_sequence()

    # Format results
    results = ClearMotResults()
    results['mota'] = cm.get_mota()
    results['motp'] = cm.get_motp()
    results['fn'] = cm.get_fn_count()
    results['fp'] = cm.get_fp_count()
    results['mismatches'] = cm.get_mismatches_count()
    results['matches'] = cm.get_matches_count()
    results['gt_tracks'] = len(zeroBasedGTIdMap)
    results['tracker_tracks'] = len(zeroBasedTrackIdMap)
    return results

def calculatePyMot(clearMotInput):
    """
    Calculate CLEARMOT metrics using the implementation by Markus Roth et al. (modified to use Euclidean distances instead of bbox overlap)
    """
    
    groundtruth = dict()
    groundtruth["frames"] = []
    
    hypotheses = dict()
    hypotheses["frames"] = []
    
    for cycleIdx in xrange(0, len(clearMotInput.groundTruthArray)):
        # Make sure input data is in the same coordinate frame
        assert(clearMotInput.groundTruthArray[cycleIdx].header.frame_id == clearMotInput.trackedPersonsArray[cycleIdx].header.frame_id)
            
        # Groundtruth
        frame = dict()
        frame["timestamp"] = float(cycleIdx)
        frame["num"] = cycleIdx
        frame["annotations"] = []

        for gtPerson in clearMotInput.groundTruthArray[cycleIdx].tracks:
            entity = dict()
            entity["id"] = gtPerson.track_id
            entity["x"] = gtPerson.pose.pose.position.x
            entity["y"] = gtPerson.pose.pose.position.y
            frame["annotations"].append(entity)
        
        groundtruth["frames"].append(frame)


        # Tracker output ("hypotheses")
        frame = dict()
        frame["timestamp"] = float(cycleIdx)
        frame["num"] = cycleIdx
        frame["hypotheses"] = []

        for trackedPerson in clearMotInput.trackedPersonsArray[cycleIdx].tracks:
            entity = dict()
            entity["id"] = trackedPerson.track_id
            entity["x"] = trackedPerson.pose.pose.position.x
            entity["y"] = trackedPerson.pose.pose.position.y
            frame["hypotheses"].append(entity)
        
        hypotheses["frames"].append(frame)

    evaluation = pymot.MOTEvaluation(groundtruth, hypotheses, clearMotInput.matchingThreshold)
    evaluation.evaluate()

    relativeStatistics = evaluation.getRelativeStatistics()
    absoluteStatistics = evaluation.getAbsoluteStatistics()

    results = ClearMotResults()
    results['mota'] = relativeStatistics['MOTA']
    results['motp'] = relativeStatistics['MOTP']
    results['fn'] = float('nan')  # FIXME: is this 'lonely ground truth tracks' !?
    results['fp'] = absoluteStatistics['false positives']
    results['mismatches'] = absoluteStatistics['mismatches']
    results['matches'] = absoluteStatistics['correspondences']
    results['gt_tracks'] = absoluteStatistics['ground truth tracks']
    results['tracker_tracks'] = absoluteStatistics['hypothesis tracks']

    # Extra results
    results['miss_rate'] = relativeStatistics['miss rate']
    results['fp_rate'] = relativeStatistics['false positive rate']
    results['mismatch_rate'] = relativeStatistics['mismatch rate']
    results['track_precision'] = relativeStatistics['track precision']
    results['track_recall'] = relativeStatistics['track recall']
    return results 