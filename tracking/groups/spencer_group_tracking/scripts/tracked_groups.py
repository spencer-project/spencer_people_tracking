#!/usr/bin/env python

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

"""
Subscribes to /spencer/perception/social_relations and /spencer/perception/tracked_persons, and publishes at /spencer/perception/tracked_groups
groups generated using single-linkage clustering based upon social relation strengths. Group ID consistency is maintained by
comparing contained track IDs.

Parameters:
publish_single_person_groups (bool):      self-explanatory, defaults to false
relation_threshold (float):               relation strength above which persons are considered to be in a group
"""

import os, sys, math, time
from collections import deque
from multiprocessing import Lock

import roslib, rospy, tf, message_filters
import geometry_msgs.msg
import numpy, scipy, scipy.spatial.distance, scipy.misc, scipy.cluster.hierarchy
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson, TrackedGroups, TrackedGroup
from spencer_social_relation_msgs.msg import SocialRelations, SocialRelation
     

# Main callback when new social relations and person tracks are available     
def newDataAvailable(trackedPersons, socialRelations):
    # Collect track positions in matrix (rows = tracks, cols = x and y coordinates)
    trackCount = len(trackedPersons.tracks)    

    # Create a lookup from arbitrary track IDs to zero-based track indices in distances matrix
    trackIndex = 0
    trackIdToIndex = dict()
    for trackedPerson in trackedPersons.tracks:
        trackIdToIndex[trackedPerson.track_id] = trackIndex;
        trackIndex += 1

    # Build full (symmetric) distances matrix from provided person relations. Diagonal is initialized with zeroes.
    socialRelationsMatrix = numpy.ones( (trackCount, trackCount) )
    for socialRelation in socialRelations.elements:
        try:
            index1 = trackIdToIndex[socialRelation.track1_id]
            index2 = trackIdToIndex[socialRelation.track2_id]
        except KeyError as e:
            rospy.logwarn("Key error while looking up tracks %d and %d!" % (socialRelation.track1_id, socialRelation.track2_id) )
            continue
            
        # Using min() here in case there are multiple types of relations per track pair -- in that case, use strongest type of relation
        socialRelationsMatrix[index1, index2] = min(socialRelationsMatrix[index1, index2], 1.0 - socialRelation.strength) # strong relation --> small distance
        socialRelationsMatrix[index2, index1] = min(socialRelationsMatrix[index2, index1], 1.0 - socialRelation.strength)

    for i in xrange(0, trackCount):
        socialRelationsMatrix[i,i] = 0.0 # diagonal elements have to be zero

    # Convert into condensed form
    trackDistances = scipy.spatial.distance.squareform(socialRelationsMatrix, force='tovector')
        
    # Cluster
    relationThreshold = rospy.get_param('~relation_threshold', 0.5) # relation strength above which we consider tracks to be in a group
    groupIndices = cluster(trackCount, trackDistances, relationThreshold) # outputs one group index per track

    # Create groups, by assembling one set of track IDs per group
    groups = createGroups(groupIndices, trackedPersons.tracks)

    # Associate newly created groups with groups from previous cycle for ID consistency, and generate TrackedGroup instances
    trackedGroups = trackGroups(groups, trackedPersons)

    # Publish groups
    publishGroups(trackedGroups, trackedPersons)


### Creates group indices by single-linkage clustering
def cluster(trackCount, trackDistances, threshold):
    # Safety check if there are less than 2 tracks, otherwise clustering fails
    if trackCount == 0:
        return []      
    elif trackCount == 1:
        return [0]       

    # Perform clustering
    linkage = scipy.cluster.hierarchy.linkage(trackDistances, method='single')
    groupIndices = scipy.cluster.hierarchy.fcluster(linkage, threshold, 'distance')
    return groupIndices


# Generates a set of track IDs per group
def createGroups(groupIndices, tracks):
    groups = dict()
    trackIndex = 0
    for groupIndex in groupIndices:
        if not groupIndex in groups:
            groups[groupIndex] = []

        trackId = tracks[trackIndex].track_id
        groups[groupIndex].append(trackId)
        trackIndex += 1
    return groups


# For remembering recent group ID assignments for a given set of track IDs
class GroupIdAssignment(object):
    def __init__(self, trackIds, groupId, createdAt):
        self.trackIds = set(trackIds)
        self.groupId = groupId
        self.createdAt = createdAt

    def __str__(self):
        return "%s = %d" % (str(list(self.trackIds)), self.groupId)

# For publishing consecutive group IDs, even if internally they are not consecutive
class GroupIdRemapping(object):
    def __init__(self, groupId, publishedGroupId):
        self.originalGroupId = groupId
        self.publishedGroupId = publishedGroupId

def remapGroupId(groupId):
    publishedGroupId = None
    for groupIdRemapping in trackGroups.groupIdRemapping:
        if groupIdRemapping.originalGroupId == groupId:
            publishedGroupId = groupIdRemapping.publishedGroupId
            break
    if publishedGroupId is None:
        trackGroups.largestPublishedGroupId += 1
        publishedGroupId = trackGroups.largestPublishedGroupId

    trackGroups.groupIdRemapping.append( GroupIdRemapping(groupId, publishedGroupId) )
    return publishedGroupId

# Associates current groups with previously tracked groups via track IDs
def trackGroups(groups, trackedPersons):
    # Initialize variables
    currentTime = trackedPersons.header.stamp
    publishSinglePersonGroups = rospy.get_param('~publish_single_person_groups', False)
    trackedGroups = []
    assignedGroupIds = []

    # Sort groups by smallest track ID per group to ensure reproducible group ID assignments
    sortedGroups = sorted(groups.iteritems(), key=lambda (clusterId, track_ids) : sorted(track_ids)[0])   

    # Used to calculate group centroids
    trackPositionsById = dict()
    for track in trackedPersons.tracks:
        pos = track.pose.pose.position            
        trackPositionsById[track.track_id] = numpy.array([ pos.x, pos.y ])

    # Create a TrackedGroup for each group, and assign a unique ID
    for clusterId, track_ids in sortedGroups:            
        # Check if we encountered this combination of track IDs, or a superset thereof, before
        bestGroupIdAssignment = None

        trackIdSet = set(track_ids)
        for groupIdAssignment in trackGroups.groupIdAssignmentMemory:
            if groupIdAssignment.trackIds.issuperset(trackIdSet) or groupIdAssignment.trackIds.issubset(trackIdSet):
                trackCount = len(groupIdAssignment.trackIds)
                bestTrackCount = None if bestGroupIdAssignment is None else len(bestGroupIdAssignment.trackIds)

                if bestGroupIdAssignment is None or trackCount > bestTrackCount or (trackCount == bestTrackCount and groupIdAssignment.createdAt < bestGroupIdAssignment.createdAt):
                    if groupIdAssignment.groupId not in assignedGroupIds:
                        bestGroupIdAssignment = groupIdAssignment

        groupId = None
        if bestGroupIdAssignment is not None:
            groupId = bestGroupIdAssignment.groupId

        if groupId == None or groupId in assignedGroupIds:
            groupId = trackGroups.largestGroupId + 1 # just generate a new ID

        # Remember that this group ID has been used in this cycle
        assignedGroupIds.append(groupId)

        # Remember which group ID we assigned to this combination of track IDs
        groupIdAssignmentsToRemove = []
        groupExistsSince = trackedPersons.header.stamp.to_sec()
        for groupIdAssignment in trackGroups.groupIdAssignmentMemory:
            if set(track_ids) == groupIdAssignment.trackIds:
                groupExistsSince = min(groupIdAssignment.createdAt, groupExistsSince)
                groupIdAssignmentsToRemove.append(groupIdAssignment)  # remove any old entries with same track IDs
        
        for groupIdAssignment in groupIdAssignmentsToRemove:
            trackGroups.groupIdAssignmentMemory.remove(groupIdAssignment) 

        trackGroups.groupIdAssignmentMemory.append( GroupIdAssignment(track_ids, groupId, groupExistsSince) )
        #rospy.loginfo(str([str(x) for x in trackGroups.groupIdAssignmentMemory]))

        # Remember largest group ID used so far
        if(groupId > trackGroups.largestGroupId):
            trackGroups.largestGroupId = groupId;

        # Do not publish single-person groups if not desired
        if publishSinglePersonGroups or len(track_ids) > 1:
            accumulatedPosition = numpy.array([0.0, 0.0])
            activeTrackCount = 0
            for track_id in track_ids:
                if track_id in trackPositionsById:
                    accumulatedPosition += trackPositionsById[track_id]
                    activeTrackCount += 1

            trackedGroup = TrackedGroup()
            trackedGroup.age = rospy.Duration( max(0, currentTime.to_sec() - groupExistsSince) )
            trackedGroup.group_id = remapGroupId(groupId)  # make sure published IDs are consecutive even if internally, they are not
            trackedGroup.track_ids = track_ids
            trackedGroup.centerOfGravity.pose.position.x = accumulatedPosition[0] / float(activeTrackCount)
            trackedGroup.centerOfGravity.pose.position.y = accumulatedPosition[1] / float(activeTrackCount)
            trackedGroups.append(trackedGroup)

    # Return currently tracked groups
    return trackedGroups


### Calculates group centroids for the current groups
def calculateGroupCentroids(groups, trackedPersons):
    # Get track positions
    trackPositionsById = dict()
    for track in trackedPersons.tracks:
        pos = track.pose.pose.position            
        trackPositionsById[track.track_id] = [ pos.x, pos.y ]

    # Determine centroids
    centroids = []
    for groupId, track_ids in groups.iteritems():
        positions = numpy.zeros( (len(track_ids), 2) )
        trackIndex = 0
        for track_id in track_ids:
            positions[trackIndex, :] = trackPositionsById[track_id]
            trackIndex += 1

        currentCentroid = GroupCentroid()
        currentCentroid.pos = numpy.mean(positions, 0)
        currentCentroid.groupId = groupId;
        currentCentroid.size = len(track_ids)
        centroids.append(currentCentroid)
    return centroids


### Publishes currently tracked groups
def publishGroups(groups, trackedPersons):
    # Prepare output TrackedGroups message
    msg = TrackedGroups()
    msg.header = trackedPersons.header
    msg.groups = groups
    pub.publish(msg)


### Main method
def main():
    rospy.init_node("tracked_groups")

    trackGroups.largestGroupId = -1
    trackGroups.groupIdAssignmentMemory = deque(maxlen=300)  # to remember which sets of track IDs were assigned which group IDs
    trackGroups.groupIdRemapping = deque(maxlen=50)  # to remap published(!) group IDs so that they are consecutive, even if internally they are not
    trackGroups.largestPublishedGroupId = -1
    
    trackedPersonsTopic = rospy.resolve_name("/spencer/perception/tracked_persons")
    socialRelationsTopic = rospy.resolve_name("/spencer/perception/social_relations")
    trackedGroupsTopic = rospy.resolve_name("/spencer/perception/tracked_groups")

    trackSubscriber = message_filters.Subscriber(trackedPersonsTopic, TrackedPersons)
    socialRelationsSubscriber = message_filters.Subscriber(socialRelationsTopic, SocialRelations)
    
    timeSynchronizer = message_filters.TimeSynchronizer([trackSubscriber, socialRelationsSubscriber], 100)
    timeSynchronizer.registerCallback(newDataAvailable)

    rospy.loginfo("Subscribing to " + socialRelationsTopic + " and " + trackedPersonsTopic)

    global pub
    pub = rospy.Publisher(trackedGroupsTopic, TrackedGroups, queue_size=3)
    rospy.loginfo("Publishing tracked groups on " + trackedGroupsTopic)

    rospy.spin()


### Entry point
if __name__ == '__main__':
    main()
