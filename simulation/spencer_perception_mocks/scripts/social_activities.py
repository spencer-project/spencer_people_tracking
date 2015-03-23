#!/usr/bin/env python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Subscribes to /spencer/perception/tracked_groups and /spencer/perception/tracked_persons, and publishes at /spencer/perception/social_activities
# some random social activities (some only for groups, some only for individual persons not in groups).
#
# Parameters:

import math, random, copy
from multiprocessing import Lock

import rospy, message_filters
import numpy
from spencer_tracking_msgs.msg import TrackedPersons, TrackedGroups
from spencer_social_relation_msgs.msg import SocialActivities, SocialActivity
     

# Main callback when new social relations and person tracks are available     
def newDataAvailable(trackedPersons, trackedGroups):
    trackCount = len(trackedPersons.tracks)    
    tracksWithoutGroup = set()
    trackDict = dict()

    msg = SocialActivities()
    msg.header = trackedPersons.header

    for track in trackedPersons.tracks:
        tracksWithoutGroup.add(track.track_id)
        trackDict[track.track_id] = track

    for group in trackedGroups.groups:
        if len(group.track_ids) > 1:
            vx = 0
            vy = 0
            socialActivity = SocialActivity()
            
            for track_id in group.track_ids:
                socialActivity.track_ids.append(track_id)
                tracksWithoutGroup.remove(track_id)
                vx += trackDict[track_id].twist.twist.linear.x
                vy += trackDict[track_id].twist.twist.linear.y

            vx /= float(len(group.track_ids))
            vy /= float(len(group.track_ids))              

            # Generate some group-level activities
            standingActivities = [ SocialActivity.TYPE_WAITING_IN_QUEUE, SocialActivity.TYPE_GROUP_ASSEMBLING ]
            movingActivities = [ SocialActivity.TYPE_GROUP_MOVING, SocialActivity.TYPE_FLOW_WITH_ROBOT, SocialActivity.TYPE_ANTIFLOW_AGAINST_ROBOT ]

            if math.sqrt(vx*vx + vy*vy) > 0.4:
                activityType = movingActivities[ (randShift + group.group_id) % len(movingActivities) ]
                otherActivityType = movingActivities[ (randShift + group.group_id + 1) % len(movingActivities) ]
            else:
                activityType = standingActivities[ (randShift + group.group_id) % len(standingActivities) ]
                otherActivityType = None

            socialActivity.type = activityType
            socialActivity.confidence = random.random() * 0.3 + 0.7
            msg.elements.append(socialActivity);

            if otherActivityType is not None: # add second activity with very low certainty
                socialActivity = copy.deepcopy(socialActivity)
                socialActivity.type = otherActivityType
                socialActivity.confidence = 1.0 - socialActivity.confidence
                msg.elements.append(socialActivity)


    # Generate some person-level activities
    standingActivities = [ SocialActivity.TYPE_STANDING, SocialActivity.TYPE_LOOKING_AT_KIOSK, SocialActivity.TYPE_WAITING_FOR_OTHERS ]
    movingActivities = [ SocialActivity.TYPE_SHOPPING, SocialActivity.TYPE_INDIVIDUAL_MOVING ]

    firstTrack = True
    for track_id in tracksWithoutGroup:
        # First track has no activity
        if firstTrack:
            firstTrack = False
            continue

        vx = trackDict[track_id].twist.twist.linear.x
        vy = trackDict[track_id].twist.twist.linear.y
        if math.sqrt(vx*vx + vy*vy) > 0.4:
            activityType = movingActivities[ (randShift + track_id) % len(movingActivities) ]
            otherActivityType = movingActivities[ (randShift + track_id + 1) % len(movingActivities) ]
        else:
            activityType = standingActivities[ (randShift + track_id) % len(standingActivities) ]
            otherActivityType = standingActivities[ (randShift + track_id + 1) % len(standingActivities) ]

        socialActivity = SocialActivity()
        socialActivity.track_ids.append(track_id)
        socialActivity.type = activityType
        socialActivity.confidence = random.random() * 0.3 + 0.7
        msg.elements.append(socialActivity);

        if otherActivityType is not None: # add second activity with very low certainty
            socialActivity = copy.deepcopy(socialActivity)
            socialActivity.type = otherActivityType
            socialActivity.confidence = 1.0 - socialActivity.confidence
            msg.elements.append(socialActivity)

    # Publish SocialActivities message
    pub.publish(msg)


### Main method
def main():
    rospy.init_node("mock_social_activities")

    global randShift
    randShift = 0

    trackedPersonsTopic = "/spencer/perception/tracked_persons"
    trackedGroupsTopic = "/spencer/perception/tracked_groups"
    socialActivitiesTopic = "/spencer/perception/social_activities"
    
    trackSubscriber = message_filters.Subscriber(trackedPersonsTopic, TrackedPersons)
    groupsSubscriber = message_filters.Subscriber(trackedGroupsTopic, TrackedGroups)
    
    timeSynchronizer = message_filters.TimeSynchronizer([trackSubscriber, groupsSubscriber], 100)
    timeSynchronizer.registerCallback(newDataAvailable)

    rospy.loginfo("Subscribing to " + trackedPersonsTopic + " and " + trackedGroupsTopic)

    global pub
    pub = rospy.Publisher(socialActivitiesTopic, SocialActivities)
    rospy.loginfo("Publishing mock data on " + socialActivitiesTopic)

    rate = rospy.Rate(1.0 / 5.0)
    while not rospy.is_shutdown():
        randShift += 1
        rate.sleep()



### Entry point
if __name__ == '__main__':
    main()