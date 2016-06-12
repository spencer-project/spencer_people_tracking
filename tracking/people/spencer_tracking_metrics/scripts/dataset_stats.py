#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015-2016, Timm Linder, Social Robotics Lab, University of Freiburg
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
from spencer_tracking_msgs.msg import TrackedPersons

def newTracksAvailable(trackedPersons):
    if not stopReceiving:
        global trackIDs, numCycles, occludedTrackCounts, trackCounts

        numCycles += 1
        trackCounts.append(len(trackedPersons.tracks))
        for trackedPerson in trackedPersons.tracks:
            trackIDs.add(trackedPerson.track_id)
        occludedTracks = filter(lambda person: person.is_occluded, trackedPersons.tracks)
        occludedTrackCounts.append(len(occludedTracks)) 
        

if __name__ == '__main__':
    maxNumOccludedTracks = 0
    numOccludedTracks = 0
    numCycles = 0
    trackIDs = set()
    trackCounts = []
    occludedTrackCounts = []
    stopReceiving = False

    rospy.init_node("dataset_stats")
    trackedPersonsTopic = rospy.resolve_name("/groundtruth/tracked_persons")

    rospy.loginfo("Listening for tracked persons on " + trackedPersonsTopic + ". Press CTRL+C to stop listening!")

    # Listen for groundtruth tracked persons
    trackedPersonsSubscriber = rospy.Subscriber(trackedPersonsTopic, TrackedPersons, newTracksAvailable, queue_size=1000)
    rospy.spin()
    stopReceiving = True

    trackCountsArray = numpy.array(trackCounts)
    occludedTrackCountsArray = numpy.array(occludedTrackCounts)
    
    rospy.loginfo("### Recorded %d cycles with %d unique tracks, of which max. %d were visible at the same time! Average: %f Median: %f###" % (numCycles, len(trackIDs), numpy.max(trackCountsArray), numpy.average(trackCountsArray), numpy.median(trackCountsArray)))
    rospy.loginfo("### Recorded %d cycles with %d occluded track frames, of which max. %d were occluded at the same time! Average: %f Median: %f ###" % (numCycles, numpy.sum(occludedTrackCountsArray), numpy.max(occludedTrackCountsArray), numpy.average(occludedTrackCountsArray), numpy.median(occludedTrackCountsArray)))
    trackFile = open('dataset_stats.csv','w')
    cycle = 0
    for trackCount in trackCounts:
        trackFile.write("%05d    %.5f   %d\n" % (cycle, cycle / float(len(trackCounts)), trackCount) )
        cycle += 1
    trackFile.close() 

