"""
CLEAR multi target tracking metric evaluation.
"""

"""
The MIT License (MIT)

Copyright (c) 2014 Matej Smid

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy
import numpy as np
import sys
import math
import munkres


class ClearMetrics(object):
    """
    CLEAR multi target tracking metric evaluation.

    For arbitrary dimensional data.

    described in:
    Keni, Bernardin, and Stiefelhagen Rainer. "Evaluating multiple object tracking performance: the CLEAR MOT metrics."
    EURASIP Journal on Image and Video Processing 2008 (2008).

    Usage:

    # 1d ground truth and measurements for 3 frames
    groundtruth = {0: [2, 3, 6],
                   1: [3, 2, 6],
                   2: [4, 0, 6]
                   }

    measurements = {
        0: [1, 3, 8],
        1: [2, 3, None, 6],
        2: [0, 4, None, 6, 8]
    }
    clear = ClearMetrics(groundtruth, measurements, 1.5)
    clear.match_sequence()
    evaluation = [clear.get_mota(),
                  clear.get_motp(),
                  clear.get_fn_count(),
                  clear.get_fp_count(),
                  clear.get_mismatches_count(),
                  clear.get_object_count(),
                  clear.get_matches_count()]
    """

    def __init__(self, groundtruth, measurements, thresh):
        """
        Initialize ClearMetrics.

        @param groundtruth:     [frame nr]    [target nr]
                                dict/list     list        ndarray, shape=(n,) or number or None
                                                          - ndarray for input data with dimensionality n
                                                          - number for 1D input data
                                                          - None means target is not present

        @param measurements:    [frame nr]    [target nr]
                                dict/list     list        ndarray, shape=(n,) or number or None

        @param thresh: float, maximum distance of a measurement from ground truth to be considered as
                              true positive
        """
        self.groundtruth = groundtruth
        self.measurements = measurements
        self.thresh = thresh

        # following members hold evaluation results:

        # [frame nr]    [target nr]
        # dict          list        int  - for every measurement corresponding groundtruth index
        #                                  -1 if no match, None there was no measurement of the target
        self.measurements_matches = None

        # [frame nr]    [target nr]
        # dict          list        int  - for every ground truth target corresponding measurement index
        #                                  -1 if no match, None target is not present in the ground truth
        self.gt_matches = None

        # [frame nr]    [target nr]
        # dict          list        int  - for every ground truth target distance to matched measurement
        #                                  -1 if no match or grount truth not defined
        self.gt_distances = None

    def match_sequence(self):
        """
        Evaluate the sequence.

        Evaluation is done for all frames where ground truth and measurement is available.

        The method writes the results to:
            self.measurements_matches
            self.gt_matches
            self.gt_distances
        """
        rospy.logdebug("match_sequence() start")
        prev_gt_matches = [-1] * len(self.groundtruth[self.get_frames()[0]])
        prev_measurement_matches = [-1] * len(self.measurements[self.get_frames()[0]])
        self.gt_matches = {}
        self.gt_distances = {}
        self.measurements_matches = {}

        rospy.logdebug("match_sequence() begin loop")
        for frame in self.get_frames():
            if frame >= len(self.measurements):
                break

            if rospy.is_shutdown():
                rospy.logerr("SIGINT received, stopping analysis")
                return

            if frame % 50 == 0:
                rospy.loginfo("Cycle %5d of %5d" % (frame, len(self.measurements)) )

            self.gt_matches[frame], self.gt_distances[frame], self.measurements_matches[frame] = \
                self._match_frame(frame, prev_gt_matches, prev_measurement_matches)
            prev_gt_matches = self.gt_matches[frame]
            prev_measurement_matches = self.measurements_matches[frame]

    def get_fp_count(self):
        """
        Return number of false positives in the sequence.

        @return: FP count
        @rtype: int
        """
        count = 0
        for matches in self.measurements_matches.values():
            count += matches.count(-1)
        return count

    def get_fn_count(self):
        """
        Return number of false negatives in the sequence.

        @return: FN count
        @rtype: int
        """
        count = 0
        for matches in self.gt_matches.values():
            count += matches.count(-1)
        return count

    def get_mismatches_count(self):
        """
        Return number of identity mismatches.

        One mismatch occurs when measurement id assigned to a gt id changes.
        E.g. identity swap in one frame equals 2 identity mismatches.

        @return: number of mismatches in the sequence
        @rtype: int
        """
        frames = sorted(self.groundtruth.keys())
        last_matches = np.array(self.gt_matches[frames[0]])
        mismatches = 0
        for frame in frames[1:]:
            if frame >= len(self.measurements):
                break
            matches = np.array(self.gt_matches[frame])
            mask_match_in_both_frames = (matches != -1) & (last_matches != -1)
            mismatches += np.count_nonzero(
                matches[mask_match_in_both_frames] != last_matches[mask_match_in_both_frames])
            last_matches = matches
        return mismatches

    def get_object_count(self):
        """
        Return number of ground truth objects in all frames in the sequence.

        @return: number of gt objects
        @rtype: int
        """
        object_count = 0
        for frame in self.get_frames():
            if frame >= len(self.measurements):
                break
            targets = self.groundtruth[frame]
            object_count += len(targets) - targets.count(None)  # TODO np.array([]) empty arrays?
        return object_count

    def get_matches_count(self):
        """
        Return number of matches between ground truth and measurements in all frames in the sequence.

        @return: number of matches
        @rtype: int
        """
        distances = np.array([dists for dists in self.gt_distances.values()])
        matches_mask = distances != -1
        return distances[matches_mask].size

    def get_motp(self):
        """
        Return CLEAR MOTP score.

        MOTP is mean distance to ground truth / mean error. Lower is better.

        @return: MOTP score
        @rtype: float
        """
        distances = np.array([dists for dists in self.gt_distances.values()])
        matches_mask = distances != -1
        return distances[matches_mask].mean()

    def get_mota(self):
        """
        Return CLEAR MOTA score.

        Can be roughly understood as a ratio of correctly tracked objects. Bigger / closer to 1 is better.

        @return: MOTA score, <= 1
        @rtype: float
        """
        return 1 - (self.get_fp_count() + self.get_fn_count() + self.get_mismatches_count()) / \
               float(self.get_object_count())

    def _get_sq_distance_matrix(self, frame):
        """
        Compute squared distances between ground truth and measurements objects.

        Distance is np.nan when gt or measurement is not defined (None).

        @param frame: frame number
        @type frame: int
        @return: distance matrix (not symmetric!)
        @rtype: np.ndarray, shape=num ground truth, num measurements
        """
        n_gt = len(self.groundtruth[frame])
        n_meas = len(self.measurements[frame])
        distance_mat = np.zeros((n_gt, n_meas))
        for i in xrange(n_gt):
            gt_pos = self.groundtruth[frame][i]
            for j in xrange(n_meas):
                measured_pos = self.measurements[frame][j]
                if gt_pos is None or measured_pos is None:
                    distance_mat[i, j] = np.nan
                else:
                    distance_mat[i, j] = np.sum((measured_pos - gt_pos) ** 2)
        return distance_mat

    def _match_frame(self, frame, prev_gt_matches, prev_measurement_matches):
        """
        Matches measurements to ground truth for a frame.

        @type frame: int - frame number
        @type prev_gt_matches: list - ground truth matches for previous frame
        @prev_measurement_matches - measurement matches for previous frame
        @return: gt_matches - list of measurement ids to that the ground truth objects match
                              None for gt objects not present in the frame
                              -1 for FN
                 gt_distances - distances from ground truth objects to matched measured objects
                                None for objects not found in the frame
                 measurements_matches - list of ground truth ids to that the measured objects match
                                        None for measured object not present in the frame
                                        -1 for FP
        @rtype: list, list, list
        """
        rospy.logdebug("Initializing distance matrix...")
        sq_distance = self._get_sq_distance_matrix(frame)
        sq_distance_undefined = math.ceil(np.nanmax(sq_distance)) + 1
        sq_distance[np.isnan(sq_distance)] = sq_distance_undefined
        sq_distance[sq_distance > (self.thresh ** 2)] = sq_distance_undefined

        rospy.logdebug("Matching measurements...")
        
        # set all ground truth matches to FN or not defined
        gt_matches = []
        for i in xrange(len(self.groundtruth[frame])):
            if self.groundtruth[frame][i] is None:
                gt_matches.append(None)
            else:
                gt_matches.append(-1)

        # set all measurements matches to FP or not defined
        gt_distances = [-1] * len(self.groundtruth[frame])
        measurements_matches = []
        for i in xrange(len(self.measurements[frame])):
            if self.measurements[frame][i] is None:
                measurements_matches.append(None)
            else:
                measurements_matches.append(-1)

        # verify TP from previous frame
        prev_matches = [(prev_measurement_matches[m], m) for m in prev_gt_matches if (m is not None) and (m != -1)]
        for prev_gt, prev_m in prev_matches:
            if sq_distance[prev_gt, prev_m] != sq_distance_undefined:
                gt_matches[prev_gt] = prev_m
                measurements_matches[prev_m] = prev_gt
                gt_distances[prev_gt] = np.sqrt(sq_distance[prev_gt, prev_m])
                # prev_gt and prev_m are excluded from further matching
                sq_distance[prev_gt, :] = sq_distance_undefined
                sq_distance[:, prev_m] = sq_distance_undefined

        # fill in new TP
        rospy.logdebug("Running Munkres algorithm with %d tracks and %d groundtruth tracks..." % (len(self.measurements[frame]), len(self.groundtruth[frame])))

        sq_dist_list = sq_distance.tolist()
        if sq_dist_list:
            m = munkres.Munkres()
            matches = m.compute(sq_dist_list)
        else:
            matches = []

        for m in matches:
            if sq_distance[m[0], m[1]] == sq_distance_undefined:
                continue
            gt_matches[m[0]] = m[1]
            measurements_matches[m[1]] = m[0]
            gt_distances[m[0]] = np.sqrt(sq_distance[m[0], m[1]])
        rospy.logdebug("Finished with Munkres algorithm...")
        
        return gt_matches, gt_distances, measurements_matches

    def get_frames(self):
        """
        Return sorted list of frames.

        :return: list of frame numbers
        :rtype: list
        """
        if isinstance(self.groundtruth, dict):
            return sorted(self.groundtruth.keys())
        if isinstance(self.measurements, dict):
            return sorted(self.measurements.keys())
        else:
            return xrange(len(self.groundtruth))
