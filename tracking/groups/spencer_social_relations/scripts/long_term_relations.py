#!/usr/bin/env python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Subscribes to /spencer/perception/social_relations and /spencer/perception/tracked_persons, and publishes at /spencer/perception/long_term_social_relations
# which are a smoothed version of the input relations (i.e. relations will decay more slowly, and build up more slowly).
#
# Parameters:
#

import os, sys, math, time, copy, numpy
import roslib, rospy, message_filters
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from spencer_social_relation_msgs.msg import SocialRelations, SocialRelation
     
class StoredRelation(object):
    def __init__(self):
        self.strength = 0.0

def isObsolete(key, currentTrackIds):
    track1_id = key[1]
    track2_id = key[2]
    return track1_id not in currentTrackIds or track2_id not in currentTrackIds

# Main callback when new social relations and person tracks are available     
def newDataAvailable(trackedPersons, inputRelations):
    global storedRelations, lastDataReceivedAt

    # Parameters
    growthAmount = 0.5 / 3.5   # growth of relation strength (between 0.0 and 1.0) per second
    slowDecayAmount  = 0.5 / 40.0 # slow decay of relation strength (between 0.0 and 1.0) per second
    fastDecayAmount  = 0.5 / 5.0  # fast decay, when maximum distance is exceeded
    maxDistance = 4.0  # distance above which relation strength will be forced to 0.0

    # Get delta time
    currentTime = inputRelations.header.stamp
    deltaTime = (currentTime - lastDataReceivedAt).to_sec()

    # Remove stored relations for which any of the two tracks does not exist any more
    currentTrackIds = set([trackedPerson.track_id for trackedPerson in trackedPersons.tracks])
    for key in list(storedRelations.keys()):
        if isObsolete(key, currentTrackIds):
            del storedRelations[key]

    # Create lookup for tracked persons
    trackedPersonPoses = dict()
    for trackedPerson in trackedPersons.tracks:
        trackedPersonPoses[trackedPerson.track_id] = numpy.array([trackedPerson.pose.pose.position.x, trackedPerson.pose.pose.position.y])

    # Prepare output message
    outputRelations = SocialRelations(header=inputRelations.header)

    # Iterate over newly received social relations, and update stored ones
    for inputRelation in inputRelations.elements:
        smallerId = min(inputRelation.track1_id, inputRelation.track2_id)
        largerId  = max(inputRelation.track1_id, inputRelation.track2_id)

        if not smallerId in trackedPersonPoses or not largerId in trackedPersonPoses:
            continue

        # Lookup element in storedRelations memory
        key = inputRelation.type, smallerId, largerId
        if key in storedRelations:
            storedRelation = storedRelations[key]
        else:
            storedRelation = StoredRelation()
            storedRelations[key] = storedRelation

        # Set target relation strength and decay rate
        targetStrength = inputRelation.strength
        decayAmount = slowDecayAmount

        # Enfore strength 0.0 if distance between tracked persons is larger than maxDistance
        if maxDistance is not None:
            distance = numpy.linalg.norm(trackedPersonPoses[smallerId] - trackedPersonPoses[largerId])
            #rospy.loginfo("%d %d %f" % (smallerId, largerId, distance) )
            if distance > maxDistance:
                targetStrength = 0.0
                decayAmount = fastDecayAmount

        # Smooth relation strength
        if targetStrength >= storedRelation.strength:  
            storedRelation.strength += growthAmount * deltaTime  # growth
            if storedRelation.strength > targetStrength:  # cap at upper limit
                storedRelation.strength = targetStrength
        else:
            storedRelation.strength -= decayAmount * deltaTime  # decay
            if storedRelation.strength < targetStrength:  # cap at lower limit
                storedRelation.strength = targetStrength

        outputRelation = copy.deepcopy(inputRelation)
        outputRelation.strength = storedRelation.strength

        outputRelations.elements.append(outputRelation)

    pub.publish(outputRelations)
    lastDataReceivedAt = currentTime

### Main method
def main():
    rospy.init_node("long_term_relations")

    global storedRelations, lastDataReceivedAt
    storedRelations = dict()
    lastDataReceivedAt = rospy.Time.now()
 
    trackedPersonsTopic = rospy.resolve_name("/spencer/perception/tracked_persons")
    inputRelationsTopic = rospy.resolve_name("/spencer/perception/social_relations")
    outputRelationsTopic = rospy.resolve_name("/spencer/perception/long_term_social_relations")

    trackSubscriber = message_filters.Subscriber(trackedPersonsTopic, TrackedPersons)
    inputRelationsSubscriber = message_filters.Subscriber(inputRelationsTopic, SocialRelations)
    
    timeSynchronizer = message_filters.TimeSynchronizer([trackSubscriber, inputRelationsSubscriber], 5)
    timeSynchronizer.registerCallback(newDataAvailable)

    rospy.loginfo("Subscribing to " + inputRelationsTopic + " and " + trackedPersonsTopic)

    global pub
    pub = rospy.Publisher(outputRelationsTopic, SocialRelations)
    rospy.loginfo("Publishing long-term relations on " + outputRelationsTopic)

    rospy.spin()


### Entry point
if __name__ == '__main__':
    main()