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
            if not gtPerson.track_id in rospy.get_param("~groundtruth_track_ids_to_ignore", []):
                entity = dict()
                entity["id"] = gtPerson.track_id
                entity["x"] = gtPerson.pose.pose.position.x
                entity["y"] = gtPerson.pose.pose.position.y
                entity["dco"] = gtPerson.is_occluded
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
            entity["dco"] = trackedPerson.is_occluded;
            frame["hypotheses"].append(entity)
        
        hypotheses["frames"].append(frame)

    evaluation = pymot.MOTEvaluation(groundtruth, hypotheses, clearMotInput.matchingThreshold)
    evaluation.evaluate()

    relativeStatistics = evaluation.getRelativeStatistics()
    absoluteStatistics = evaluation.getAbsoluteStatistics()
    mostlyStatistics = evaluation.getMostlyStatistics()

    results = ClearMotResults()
    results['mota'] = relativeStatistics['MOTA']
    results['motp'] = relativeStatistics['MOTP']
    results['fn'] = float('nan')  # FIXME: is this 'lonely ground truth tracks' !?
    results['fp'] = absoluteStatistics['false positives']
    results['mismatches'] = absoluteStatistics['mismatches']
    results['matches'] = absoluteStatistics['correspondences']
    results['gt_tracks'] = absoluteStatistics['ground truth tracks']
    results['tracker_tracks'] = absoluteStatistics['hypothesis tracks']
    results['correspondences'] = absoluteStatistics['correspondences']
    results['groundtruths'] = absoluteStatistics['ground truths']

    # Extra results
    results['miss_rate'] = relativeStatistics['miss rate']
    results['fp_rate'] = relativeStatistics['false positive rate']
    results['mismatch_rate'] = relativeStatistics['mismatch rate']  # relative to GT count
    results['relative_id_switches'] = relativeStatistics['relative ID switches']  # relative to recovered track count (i.e. tracker with higher track recall is allowed to have more ID switches), see MOTChallenge 2015
    results['track_precision'] = relativeStatistics['track precision']
    results['track_recall'] = relativeStatistics['track recall']
    results['misses'] = absoluteStatistics['misses']
    results['non-recoverable_mismatches'] = absoluteStatistics['non-recoverable mismatches']
    results['recoverable_mismatches'] = absoluteStatistics['recoverable mismatches']
    results['lonely_ground_truth_tracks'] = absoluteStatistics['lonely ground truth tracks']
    results['covered_ground_truth_tracks'] = absoluteStatistics['covered ground truth tracks']
    results['lonely_hypothesis_tracks'] = absoluteStatistics['lonely hypothesis tracks']
    
    results['mostly_tracked'] = mostlyStatistics['mostly_tracked']
    results['partially_tracked'] = mostlyStatistics['partially_tracked']
    results['mostly_lost'] = mostlyStatistics['mostly_lost']
    
    return results 


