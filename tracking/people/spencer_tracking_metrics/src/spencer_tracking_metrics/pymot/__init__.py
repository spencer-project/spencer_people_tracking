#
# PyMot metrics
#
# Source: https://github.com/Videmo/pymot
# License unclear
#
# Contributors:
# Markus Roth (roth@videmo.de)
# Martin Baeuml (baeuml@kit.edu)
# Mika Fischer (fischer@videmo.de)
# Keni Bernardin (bernardin@videmo.de)
#
# Extended for use with 2D ground plane coordinates by
# Fabian Girrbach, Timm Linder, Social Robotics Lab, University of Freiburg
# 

import sys
import json
import argparse
import numpy, numpy.linalg
import rospy
from munkres import Munkres
from rect import Rect
from importers import MOT_hypo_import
from importers import MOT_groundtruth_import
from formatchecker import FormatChecker
from utilities import write_stderr_red
import logging
import time
LOG = logging.getLogger(__name__)


class MOTEvaluation:

    def __init__(self, groundtruth, hypotheses, matchingThreshold):
        """Constructor """
        
        self.matching_threshold_ = matchingThreshold
        """Threshold distance for matching in meters"""
    
        self.munkres_inf_ = 100000
        """Not quite infinite number for Munkres algorithm"""
    
        self.sync_delta_ = 0.001
        """Maximum offset considered for a match of hypothesis and ground truth"""

        self.groundtruth_ = groundtruth
        """Groundtruth. See groundtruth.json for a sample file"""
        
        self.hypotheses_ = hypotheses
        """Hypotheses. See hypotheses.json for a sample file"""

        self.use_lapjv = rospy.get_param("~use_lapjv", True)
        """ use LAPJV instead of Munkres? (requires library to be installed from http://www.starklab.org/members/kazmar/software/) """
        
        if self.use_lapjv:
            try:
                import lapjv
            except ImportError:
                rospy.logerr("Setting use_lapjv in PyMot to False because required library could not be found. Install from http://www.starklab.org/members/kazmar/software/!")
                self.use_lapjv = False
            else:
                self.lapjv = lapjv

        self.convertIDsToString()

        self.resetStatistics()

        # Set class and type for hypos and ground truths
        for f in self.hypotheses_["frames"]:
            for h in f["hypotheses"]:
                h["type"] = "hypothesis"
                h["class"] = "unevaluated"

        for f in self.groundtruth_["frames"]:
            for g in f["annotations"]:
                g["type"] = "groundtruth"
                g["class"] = "unevaluated"
            
        # List of dicts, containing ground truths and hypotheses for visual debugging
        self.visualDebugFrames_ = []


    def get_hypotheses_frame(self, timestamp):
        """Get list of hypotheses occuring chronologically close to ground truth timestamp, but at most with time difference self.sync_delta"""
        
        # Helper function for filter()
        def hypothesis_frame_chronologically_close(hypothesis):
            return abs(hypothesis["timestamp"] - timestamp) < self.sync_delta_

        # Hypotheses frames which are chronologically close to timestamp 
        # Use binary search, if this is to slow for you :)P
        hypotheses_frames = filter(hypothesis_frame_chronologically_close, self.hypotheses_["frames"])
        
        # We expect at most one hypotheses timestamp.
        if len(hypotheses_frames) > 1:
            raise Exception, "> 1 hypotheses timestamps found for timestamp %f with sync delta %f" % (timestamp, self.sync_delta_)

        if len(hypotheses_frames) == 0:
#            write_stderr_red("Warning:", "No hypothesis timestamp found for timestamp %f with sync delta %f" % (timestamp, self.sync_delta_))
            return {"hypotheses": []} # empty list of hypos
        
        return hypotheses_frames[0] # return first and only element of list


    def evaluate(self):
        """Compute MOTA metric from ground truth and hypotheses for all frames."""
        
        frames = self.groundtruth_["frames"]
        for frame in frames:
            self.evaluateFrame(frame)


    def evaluateFrame(self, frame):
        """Update statistics by evaluating a new frame."""

        timestamp = frame["timestamp"]
        groundtruths = frame["annotations"]
        hypotheses = self.get_hypotheses_frame(timestamp)["hypotheses"]

        visualDebugAnnotations = []

        # Save occuring ground truth ids
        for g in groundtruths:
            self.groundtruth_ids_.add(g["id"])
            # Added for Mostly Tracked calculation
            temp_id = g["id"]
            # If gt already seen increment counter for gt_id
            if temp_id in self.mostly_tracked_list.keys():            
                if not g.get("dco",False):
                    self.mostly_tracked_list[temp_id]["gt_count"] += 1
            # For new gt track initialize a new dict
            else :
                self.mostly_tracked_list[temp_id] = {'gt_count': 1, 'hyp_count': 0}
                
                 
            

        # Save occuring hypothesis ids
        for h in hypotheses:
            self.hypothesis_ids_.add(h["id"])
        
        rospy.logdebug("")
        rospy.logdebug("Timestamp: %s" % timestamp)
        
        rospy.logdebug("DIFF")
        rospy.logdebug("DIFF Time %.2f" % timestamp)
        
        logstr = ["DIFF Mappings:"]
        for gt_id in sorted(self.mappings_.keys()):
            logstr.append("%s-%s" % (gt_id, self.mappings_[gt_id]))
        rospy.logdebug(" ".join(logstr))

        # No need to evaluate this frame.
        if len(groundtruths) == 0 and len(hypotheses) == 0:
            rospy.logdebug("No gt and hypos for this frame.")
            return

        rospy.logdebug("GTs:")
        for groundtruth in groundtruths:
            rospy.logdebug(Rect(groundtruth))

        rospy.logdebug("Hypos:")
        for hypothesis in hypotheses:
            rospy.logdebug(Rect(hypothesis))
            

        # PAPER STEP 1
        # Valid mappings skip Munkres algorithm, if both ground truth and hypo are found in this frame
        # We call these pairs correspondences and fill the list each frame.
        correspondences = {} # truth id -> hypothesis id
        
        listofprints = []
        rospy.logdebug("")
        rospy.logdebug("STEP 1: KEEP CORRESPONDENCE")
#            print "DIFF Keep correspondence"
            
        for gt_id in self.mappings_.keys():
            groundtruth = filter(lambda g: g["id"] == gt_id, groundtruths) # Get ground truths with given ground truth id in current frame
            if len(groundtruth) > 1:
                rospy.logwarn("found %d > 1 ground truth tracks for id %s", len(groundtruth), gt_id)
            elif len(groundtruth) < 1:
                continue
            
            hypothesis = filter(lambda h: h["id"] == self.mappings_[gt_id], hypotheses) # Get hypothesis with hypothesis id according to mapping
            assert len(hypothesis) <= 1, "Multiple track hypotheses found with the same ID (#%s) at cycle no. %d! Make sure the tracker is outputting unique track IDs!" % (hypothesis[0]["id"] , int(timestamp))
            if len(hypothesis) != 1:
                continue
            
            # Hypothesis found for known mapping
            # Check hypothesis for overlap
            distance = numpy.linalg.norm(Rect(groundtruth[0]).asNumpyArray() - Rect(hypothesis[0]).asNumpyArray() )
            if distance <= self.matching_threshold_ and not groundtruth[0].get("dco",False):
                rospy.logdebug("Keeping correspondence between %s and %s" % (groundtruth[0]["id"], hypothesis[0]["id"]))
#                    print "DIFF Keep corr %s %s %.2f" % (groundtruth[0]["id"], hypothesis[0]["id"], Rect(groundtruth[0]).overlap(Rect(hypothesis[0])))
                #listofprints.append("DIFF Keep corr %s %s %.2f" % (groundtruth[0]["id"], hypothesis[0]["id"], Rect(groundtruth[0]).overlap(Rect(hypothesis[0]))))
                correspondences[gt_id] = hypothesis[0]["id"]
                self.total_distance_ += distance
            elif groundtruth[0].get("dco",False):
                correspondences[gt_id] = hypothesis[0]["id"]
            elif hypothesis[0].get("dco",False) and not groundtruth[0].get("dco",False) and distance <= 2*self.matching_threshold_:
                self.misses_ += 1 
                correspondences[gt_id] = hypothesis[0]["id"]
                self.total_distance_ += distance

        munk_hypotheses = filter(lambda h: h['id'] not in correspondences.values(), hypotheses)         
        munk_gts = filter(lambda g: g['id'] not in correspondences.keys(), groundtruths) 
        for p in sorted(listofprints):
            rospy.logdebug(p)

        # PAPER STEP 2
        rospy.logdebug("")
        rospy.logdebug("STEP 2: FIND CORRESPONDENCE")
        
        # Fill hungarian matrix with +inf
        munkres_matrix = [ [ self.munkres_inf_ for i in range(max(len(munk_hypotheses), len(munk_gts))) ] for j in range(max(len(munk_hypotheses), len(munk_gts))) ] # TODO make square matrix
        valid_counter = 0
        

        # Find correspondences
        for i in range(len(munk_gts)):
            groundtruth = munk_gts[i]
            
            # Skip groundtruth with correspondence from mapping
            if groundtruth["id"] in correspondences.keys():
                rospy.logdebug("Groundtruth %s already in correspondence" % groundtruth["id"])
                continue
            
            # Fill hungarian matrix with distance between gts and hypos
            for j in range(len(munk_hypotheses)):
                hypothesis = munk_hypotheses[j]
                
                # Skip hypotheses with correspondence from mapping
                if hypothesis["id"] in correspondences.values():
                    rospy.logdebug("Hypothesis %s already in correspondence" % hypothesis["id"])
                    continue
                
                rect_groundtruth = Rect(groundtruth)
                rect_hypothesis = Rect(hypothesis)
                distance = numpy.linalg.norm(rect_groundtruth.asNumpyArray() - rect_hypothesis.asNumpyArray())
                
                if distance <= self.matching_threshold_:
#                        print "Fill Hungarian", rect_groundtruth, rect_hypothesis, overlap
                    munkres_matrix[i][j] = distance
                    valid_counter += 1
                    rospy.logdebug("DIFF candidate %s %s %.2f" % (groundtruth["id"], hypothesis["id"], distance))
        
        # Do the Munkres / LAPJV        
        # Only run munkres on non-empty matrix
        if valid_counter > 1:
            # t = time.time()   
            if self.use_lapjv:
                np_array = numpy.array(munkres_matrix, dtype=numpy.float32 )
                lap_result = self.lapjv.lap(np_array)
                indices = zip(range(max(len(munk_hypotheses),len(munk_gts))), lap_result[1])
#               elapsed = time.time() - t
#               rospy.loginfo("LAPJV needed {}s".format(elapsed))
            else:
                m = Munkres()
                indices = m.compute(munkres_matrix)
#               elapsed = time.time() - t
#               rospy.loginfo("Munkres needed {}s".format(elapsed))                       
        elif valid_counter == 1:
            np_array = numpy.array(munkres_matrix, dtype=numpy.float32  )
            idx = numpy.unravel_index(np_array.argmin(), np_array.shape)
            indices = [idx]            
        else:
            rospy.logdebug("No need to run Hungarian with %d ground truths and %d hypothesis." % (len(groundtruths), len(hypotheses)))
            indices = []
        rospy.logdebug(indices)
        
        correspondencelist = []
        mismatcheslist = []
        
        for gt_index, hypo_index in indices:
            
            # Skip invalid self.mappings_
            # Check for max float distance matches (since Hungarian returns complete mapping)
            if (munkres_matrix[gt_index][hypo_index] >= self.munkres_inf_): # NO correspondence <=> overlap >= thresh
                continue
            
            gt_id   = munk_gts[gt_index]["id"]
            hypo_id = munk_hypotheses[hypo_index]["id"]
            
            # Assert no known mappings have been added to hungarian, since keep correspondence should have considered this case.
            if gt_id in self.mappings_:
                assert self.mappings_[gt_id] != hypo_id 
            
            
            # Add to correspondences
            rospy.logdebug("Correspondence found: %s and %s (distance: %f)" % (gt_id, hypo_id, munkres_matrix[gt_index][hypo_index]))
#                correspondencelist.append("DIFF correspondence %s %s %.2f" % (gt_id, hypo_id, 1.0 / munkres_matrix[gt_index][hypo_index]))
            correspondencelist.append("DIFF correspondence %s %s" % (gt_id, hypo_id))
            correspondences[gt_id] = hypo_id
            self.total_distance_ += munkres_matrix[gt_index][hypo_index]
            

            # Count "recoverable" and "non-recoverable" mismatches
            # "recoverable" mismatches
            if gt_id in self.gt_map_ and self.gt_map_[gt_id] != hypo_id and not munk_gts[gt_index].get("dco",False):
                rospy.logdebug("Look ma! We got a recoverable mismatch over here! (%s-%s) -> (%s-%s)" % (gt_id, self.gt_map_[gt_id], gt_id, hypo_id))
                self.recoverable_mismatches_ += 1

            # "non-recoverable" mismatches
            if hypo_id in self.hypo_map_ and self.hypo_map_[hypo_id] != gt_id:
                # Do not count non-recoverable mismatch, if both old ground truth and current ground truth are DCO.
                old_gt_id = self.hypo_map_[hypo_id]
                old_gt_dco = filter(lambda g: g["id"] == old_gt_id and g.get("dco",False), groundtruths)

                assert len(old_gt_dco) <= 1;
                if not (munk_gts[gt_index].get("dco",False) and len(old_gt_dco) == 1):
                    rospy.logdebug("Look ma! We got a non-recoverable mismatch over here! (%s-%s) -> (%s-%s)" % (self.hypo_map_[hypo_id], hypo_id, gt_id, hypo_id))
                    self.non_recoverable_mismatches_ += 1

            # Update yin-yang maps                    
            self.gt_map_[gt_id] = hypo_id
            self.hypo_map_[hypo_id] = gt_id

            # Correspondence contradicts previous mapping. Mark and count as mismatch, if ground truth is not a DCO
            # Iterate over all gt-hypo pairs of mapping, since we have to perform a two way check:
            # Correspondence: A-1
            # Mapping: A-2, B-1
            # We have to detect both forms of conflicts
            for mapping_gt_id, mapping_hypo_id in self.mappings_.items():
                
                # CAVE: Other than in perl script:
                # Do not consider for mismatch, if both old gt and new gt are DCO
                gt_with_mapping_gt_id_dco = filter(lambda g: g["id"] == mapping_gt_id and g.get("dco",False), groundtruths)
                if len (gt_with_mapping_gt_id_dco) == 1 and munk_gts[gt_index].get("dco",False):
                    rospy.logdebug("Ground truths %s and %s are DCO. Not considering for mismatch." % (mapping_gt_id, gt_id))
#                    print "DIFF DCO %s" % (gt_id), groundtruths[gt_index]
                    
                else:
                # Look ma, we got a conflict over here!
                # New hypothesis for mapped ground truth found
                    if (mapping_gt_id == gt_id and mapping_hypo_id != hypo_id)\
                    or (mapping_gt_id != gt_id and mapping_hypo_id == hypo_id):
                        rospy.logdebug("Correspondence %s-%s contradicts mapping %s-%s. Counting as mismatch and updating mapping." % (gt_id, hypo_id, mapping_gt_id, mapping_hypo_id))
                        mismatcheslist.append("DIFF Mismatch %s-%s -> %s-%s" % (mapping_gt_id, mapping_hypo_id, gt_id, hypo_id))
                        self.mismatches_ = self.mismatches_ + 1

                        # find groundtruth and hypothesis with given ids
                        g = filter(lambda g: g["id"] == gt_id, groundtruths)
                        h = filter(lambda h: h["id"] == hypo_id, hypotheses)

                        #assert(len(g) == 1)
                        if len(g) != 1:
                            rospy.logwarn('more than one gt: %s', str(g))
                        assert(len(h) == 1)

                        g = g[0]
                        h = h[0]

                        g["class"] = "mismatch"
                        h["class"] = "mismatch"

                        visualDebugAnnotations.append(g)
                        visualDebugAnnotations.append(h)

                        # mapping will be updated after loop
                        del self.mappings_[mapping_gt_id]
            
#                print "YIN: %d %d" % (self.recoverable_mismatches_, self.non_recoverable_mismatches_)
#                assert(self.recoverable_mismatches_ + self.non_recoverable_mismatches_ == self.mismatches_)
            if(self.recoverable_mismatches_ + self.non_recoverable_mismatches_ != self.mismatches_):
                rospy.logdebug("Look, mismatches differ: g %d b %d  other %d" % (self.recoverable_mismatches_, self.non_recoverable_mismatches_, self.mismatches_))
                rospy.logdebug(self.gt_map_)
                rospy.logdebug(self.hypo_map_)
        
            # Save (overwrite) mapping even if ground truth is dco
            self.mappings_[gt_id] = hypo_id # Update mapping
        
        # Sorted DIFF output
        for c in sorted(correspondencelist):
            rospy.logdebug(c)
            
        for m in sorted(mismatcheslist):
            rospy.logdebug(m)

        # Visual debug
        for g in groundtruths:
            if g["class"] != "mismatch" and g["id"] in correspondences.keys():
                g["class"] = "correspondence"
                visualDebugAnnotations.append(g)
            
        for h in hypotheses:
            if h["class"] != "mismatch" and h["id"] in correspondences.values():
                h["class"] = "correspondence"
                visualDebugAnnotations.append(h)

        
        # TODO get overlap ratio
        # Print out correspondences
#            for gt_id, hypo_id in correspondences.items():
#                print "Correspondence: %s-%s" % (gt_id, hypo_id)
        
        # PAPER STEP 4
        # Count miss, when groundtruth has no correspondence and is not dco
        for groundtruth in groundtruths:
            rospy.logdebug("DCO:", groundtruth)
            if groundtruth["id"] not in correspondences.keys() and groundtruth.get("dco", False) != True:
                rospy.logdebug("Miss: %s" % groundtruth["id"])
                rospy.logdebug("DEBUGMISS: %.2f" % timestamp)
                rospy.logdebug("DIFF Miss %s" % groundtruth["id"])
                groundtruth["class"] = "miss"
                visualDebugAnnotations.append(groundtruth)
                self.misses_ += 1

        # Count false positives
        for hypothesis in hypotheses:
            if hypothesis["id"] not in correspondences.values():
                rospy.logdebug("False positive: %s" % hypothesis["id"])
                rospy.logdebug("DIFF False positive %s" % hypothesis["id"])
                self.false_positives_ += 1
                visualDebugAnnotations.append(hypothesis)
                hypothesis["class"] = "false positive"
        
        self.total_correspondences_ += len(correspondences)

        # For counting matches, do not count groundtruths with "don't care"-flag
        self.total_matches_ += len(correspondences)
        for groundtruth in groundtruths:
            if groundtruth.get("dco", False) and groundtruth["id"] in correspondences.keys():
                self.total_matches_ -= 1

        
        # FIXED: Count only groundtruth objects that don't carry a 'don't care' flag. Otherwise we can get high MOTA scores even if the track recall is 0.0!
        self.total_groundtruths_ += len( filter(lambda groundtruth: not groundtruth.get("dco", False), groundtruths) )
        #self.total_groundtruths_ += len(groundtruths) # Number of objects (ground truths) in current frame

        visualDebugFrame = {
            "timestamp": timestamp,
 #           "class": frame["class"],
            "annotations": visualDebugAnnotations
        }
        if "num" in frame:
            visualDebugFrame["num"] = frame["num"]

        self.visualDebugFrames_.append(visualDebugFrame)
        
        # Added for Mostly tracked support
        # Increment hyp count for each found correspondence
        for gt_id, hyp_id in correspondences.iteritems():
            self.mostly_tracked_list[gt_id]['hyp_count'] += 1


    def getMOTA(self):
        mota = 0.0
        if self.total_groundtruths_ == 0:
            write_stderr_red("Warning", "No ground truth. MOTA calculation not possible")
#            raise("No ground truth. MOTA calculation not possible")
        else:
            mota = 1.0 - float(self.misses_ + self.false_positives_ + self.mismatches_) / float(self.total_groundtruths_)

        return mota


    def getMOTP(self):
        motp = 0.0
        if self.total_correspondences_ == 0:
            write_stderr_red("Warning", "No correspondences found. MOTP calculation not possible")
#            raise("No correspondence found. MOTP calculation not possible")
        else:
            motp = self.total_distance_ / self.total_correspondences_
        return motp


    def getAbsoluteStatistics(self):
        lonely_ground_truths = self.groundtruth_ids_ - set(self.gt_map_.keys())
        covered_ground_truths = self.groundtruth_ids_ & set(self.gt_map_.keys())
        lonely_hypotheses = self.hypothesis_ids_ - set(self.hypo_map_.keys())

        return {
            "ground truths":   self.total_groundtruths_,
            "false positives": self.false_positives_,
            "misses":          self.misses_,
            "mismatches":      self.mismatches_,
            "recoverable mismatches": self.recoverable_mismatches_,
            "non-recoverable mismatches":  self.non_recoverable_mismatches_,
            "correspondences": self.total_correspondences_,  # includes correspondences where GT has don't care-flag!
            "matches": self.total_matches_,  # this instead ignores GT with DCF
            "total distance":   self.total_distance_,
            "lonely ground truth tracks": len(lonely_ground_truths),
            "covered ground truth tracks": len(covered_ground_truths),
            "lonely hypothesis tracks":   len(lonely_hypotheses),
            "ground truth tracks": len(self.groundtruth_ids_),
            "hypothesis tracks":   len(self.hypothesis_ids_),
            "covering hypothesis tracks": len(self.hypo_map_.keys())
        }
    

    def getRelativeStatistics(self):
        gt = self.total_groundtruths_
        covered_ground_truths = self.groundtruth_ids_ & set(self.gt_map_.keys())
        lonely_hypotheses = self.hypothesis_ids_ - set(self.hypo_map_.keys())

        return {
            "MOTA":                 self.getMOTA(),
            "MOTP":                 self.getMOTP(),
            "miss rate":            float(self.misses_) / gt,
            "false positive rate":  float(self.false_positives_) / gt,  # FIXME: Better call this FP%, as FPR = FP / (FP + TN) not / GT!
            "mismatch rate":        float(self.mismatches_) / gt,
            "recoverable mismatch rate":   float(self.recoverable_mismatches_) / gt,
            "non-recoverable mismatch rate":    float(self.non_recoverable_mismatches_) / gt,
            "track precision":      float(len(self.hypo_map_.keys())) / len(self.hypothesis_ids_) if len(self.hypothesis_ids_) != 0 else 0.0,
            "track recall":         float(len(self.gt_map_.keys()))  / len(self.groundtruth_ids_) if len(self.groundtruth_ids_) != 0 else 0.0,
            "relative ID switches": float(self.mismatches_) * gt / self.total_matches_  # relative to recovered track count (i.e. tracker with higher track recall is allowed to have more ID switches), see MOTChallenge 2015
        }
        
    def calcMostlyTracked(self):
        mostly_tracked = 0
        partial_tracked = 0
        mostly_lost = 0
        for track in self.mostly_tracked_list.values():
            percent = float(track['hyp_count'])/float(track['gt_count'])
            if percent >= 0.8:
                mostly_tracked += 1
            elif percent < 0.2:
                mostly_lost += 1
            else:
                partial_tracked +=1
                
        rospy.logdebug('Mostly Tracked: {}; Partial Tracked: {}; Mostly Lost: {}'.format(mostly_tracked,partial_tracked,mostly_lost))
    
        return {
                'mostly_tracked': mostly_tracked,
                'partially_tracked': partial_tracked,
                'mostly_lost': mostly_lost
                }
         
    def getMostlyStatistics(self):
        result = self.calcMostlyTracked()
        return result

    def printTrackStatistics(self):
        # Lonely ground truths (no single correspondence)
        lonely_ground_truths = self.groundtruth_ids_ - set(self.gt_map_.keys())
        print "Lonely ground truth tracks %d" % len(lonely_ground_truths)
        print "Total ground truth tracks  %d" % len(self.groundtruth_ids_)
#        print "    ", lonely_ground_truths

        # Dirty false positive tracks (no single correspondence)
        lonely_hypotheses = self.hypothesis_ids_ - set(self.hypo_map_.keys())
        print "Lonely hypothesis tracks %d" % len(lonely_hypotheses)
        print "Total hypothesis tracks  %d" % len(self.hypothesis_ids_)
#        print "    ", lonely_hypotheses


    def printResults(self):
        """Print out results"""
        # Additional statistics
        print "Ground truths               %d" % self.total_groundtruths_
        print "False positives             %d" % self.false_positives_
        print "Misses                      %d" % self.misses_
        print "Mismatches                  %d" % self.mismatches_
        print "Recoverable mismatches      %d" % self.recoverable_mismatches_
        print "Non recoverable mismatches  %d" % self.non_recoverable_mismatches_
        print "Correspondences             %d" % self.total_correspondences_
        print ""
        print "MOTP", self.getMOTP()
        print "MOTA", self.getMOTA()
        
    def convertIDsToString(self):
        for f in self.groundtruth_["frames"]:
            for g in f["annotations"]:
                g["id"] = str(g.get("id", '__missing_id__'))

        for f in self.hypotheses_["frames"]:
            for h in f["hypotheses"]:
                h["id"] = str(h["id"])


    def getVisualDebug(self):
        fileitem = {
            'filename': self.groundtruth_["filename"],
            'class':    self.groundtruth_["class"],
            'frames': self.visualDebugFrames_
        }
        return [fileitem]


    def resetMapping(self):
        """Reset mapping. Useful for loading new ground truth and hypo and not counting shot-boundary caused mismatches."""
        self.mappings_ = {} # Mappings from ground truth id to hypothesis id, as described in paper: M_t (initial M_0 empty)

        # Helper dicts for "recoverable" and "non-recoverable" mismatch detection aka Yin Yang
        self.gt_map_ = {} # save most recent hypothesis id for each groundtruth id. Only updates, no deletions of keys
        self.hypo_map_ = {} # save move recent groundtruth id for each hypothesis id. Only updates, no deletions of keys.


    def resetStatistics(self):
        """Reset counters and mapping."""
        self.resetMapping()

        # yin-yang
        self.recoverable_mismatches_ = 0
        self.non_recoverable_mismatches_ = 0

        # MOTA related
        self.mismatches_ = 0
        self.misses_ = 0
        self.false_positives_ = 0
        self.total_groundtruths_ = 0
        self.total_distance_ = 0.0
        self.total_correspondences_ = 0
        self.total_matches_ = 0

        self.groundtruth_ids_ = set()
        self.hypothesis_ids_ = set()
        self.mostly_tracked_list = dict()
