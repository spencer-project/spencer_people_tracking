#!/usr/bin/env python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Subscribes to /spencer/perception/tracked_persons and publishes at /spencer/perception/social_relations
# the tracks' social relations based upon their euclidean distances, which are linearly mapped into the interval [0, 1].
#
# Parameters:
#   _max_distance (float):            Anything above this distance will be assigned a social relation strength of 0.0. Defaults to 3.0 meters

import os, sys, math, time
from multiprocessing import Lock

import roslib, rospy, tf, message_filters; roslib.load_manifest('spencer_perception_mocks')
import geometry_msgs.msg
import numpy, scipy, scipy.spatial.distance, scipy.misc
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from spencer_social_relation_msgs.msg import SocialRelations, SocialRelation
        
    
def newDataAvailable(trackedPersons):
    # Collect track positions in matrix (rows = tracks, cols = x and y coordinates)
    trackCount = len(trackedPersons.tracks)
    trackPositionsMatrix = extractTrackPositions(trackedPersons)      
    maxDistance = rospy.get_param('~_max_distance', 3.0)
    trackDistances = scipy.spatial.distance.pdist(trackPositionsMatrix)

    # Prepare output SocialRelations message
    socialRelations = SocialRelations()
    socialRelations.header = trackedPersons.header

    # Generate one SocialRelation per pair of tracks
    for t1index in xrange(0, trackCount):
        for t2index in xrange(t1index+1, trackCount):
            distanceIndex = scipy.misc.comb(trackCount, 2) - scipy.misc.comb(trackCount - t1index, 2) + (t2index - t1index - 1) # see docs on squareform()
            trackDistance = trackDistances[distanceIndex]
            mappedTrackDistance = 1.0 - min(trackDistance, maxDistance) / maxDistance 

            socialRelation = SocialRelation()
            socialRelation.track1_id = trackedPersons.tracks[t1index].track_id
            socialRelation.track2_id = trackedPersons.tracks[t2index].track_id
            socialRelation.type = SocialRelation.TYPE_SPATIAL
            socialRelation.strength = mappedTrackDistance

            socialRelations.elements.append(socialRelation)

    pub.publish(socialRelations)


def extractTrackPositions(trackedPersons):
    # This assumes that data arrives in "odom" frame (x, y = coordinates on groundplane)
    trackPositionsMatrix = numpy.zeros( (len(trackedPersons.tracks), 2) )

    for i in xrange(0, len(trackedPersons.tracks)) :
        pos = trackedPersons.tracks[i].pose.pose.position            
        trackPositionsMatrix[i, :] = [ pos.x, pos.y ]

    return trackPositionsMatrix


def main():
    rospy.init_node("mock_social_relations")

    trackedPersonsTopic = "/spencer/perception/tracked_persons"
    socialRelationsTopic = "/spencer/perception/social_relations"

    rospy.Subscriber(trackedPersonsTopic, TrackedPersons, newDataAvailable)
    rospy.loginfo("Subscribing to " + trackedPersonsTopic)

    global pub
    pub = rospy.Publisher(socialRelationsTopic, SocialRelations)
    rospy.loginfo("Publishing mock data on " + socialRelationsTopic)

    rospy.spin()


if __name__ == '__main__':
    main()