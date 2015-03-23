#!/usr/bin/env python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Publishes fake tracked persons and the corresponding detections (if not occluded) at
# /spencer/perception/tracked_persons and /spencer/perception/detected_persons.

import roslib; roslib.load_manifest( 'spencer_perception_mocks' )
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson, DetectedPersons, DetectedPerson
import rospy
from math import cos, sin, tan, pi
import tf
import random
import copy

class TrackDictionaryEntry(object):
    def __init__(self):
        self.idShift = 0
        self.remainingOcclusionTime = 0
        self.lastIdShiftAt = rospy.Time.now()

def setPoseAndTwistFromAngle( pose, twist, angle, radius, moving ) :
    currentx = radius * cos(angle)
    currenty = radius * sin(angle)

    nextx = radius * cos(angle + angleStep)
    nexty = radius * sin(angle + angleStep)
    
    pose.pose.position.x = currentx
    pose.pose.position.y = currenty
    pose.pose.position.z = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, angle + pi/2.0)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    
    scale = random.random() * 0.1 + 1.0
    pose.covariance[0 + 0 * 6] = 0.1  * scale # x
    pose.covariance[1 + 1 * 6] = 0.08 * scale # y
    pose.covariance[2 + 2 * 6] = 999999 # z
    pose.covariance[3 + 3 * 6] = 999999 # x rotation
    pose.covariance[4 + 5 * 6] = 999999 # y rotation
    pose.covariance[4 + 5 * 6] = 999999 # z rotation

    if moving:
        deltaTime = 1.0 / updateRateHz
        twist.twist.linear.x = (nextx - currentx) / deltaTime
        twist.twist.linear.y = (nexty - currenty) / deltaTime
        twist.twist.linear.z = 0
    
    for i in range(0, 3):
        twist.covariance[i + i * 6] = 1.0 # linear velocity
    for i in range(3, 6):
        twist.covariance[i + i * 6] = float("inf") # rotational velocity


def createTrackAndDetection( tracks, detections, track_id, angle, radius, moving = True) :
    trackedPerson = TrackedPerson()
    
    global trackDictionary, detectionCounter
    trackDictionaryEntry = None

    if not track_id in trackDictionary:
        trackDictionaryEntry = TrackDictionaryEntry()
        trackDictionary[track_id] = trackDictionaryEntry
    else:
        trackDictionaryEntry = trackDictionary[track_id]

    # Generate detection ID
    detection_id = detectionCounter
    detectionCounter += 1

    #
    # Simulate occlusions
    #
    occlusionProbability = 0.02
    occlusionMinDuration =  1.0 / updateRateHz # at least 1 frame
    occlusionMaxDuration = 15.0 / updateRateHz # at most 15 frames

    # Currently not occluded?
    if trackDictionaryEntry.remainingOcclusionTime <= 0:
        if random.random() < occlusionProbability:
            trackDictionaryEntry.remainingOcclusionTime = random.random() * (occlusionMaxDuration - occlusionMinDuration) + occlusionMinDuration

    # Is the track occluded?
    if trackDictionaryEntry.remainingOcclusionTime <= 0:
        trackedPerson.detection_id = detection_id
        trackedPerson.is_occluded = False
    else :
        trackedPerson.is_occluded = True
        trackDictionaryEntry.remainingOcclusionTime -= 1.0 / updateRateHz

    #
    # Simulate track ID switches
    #

    idSwitchProbability = 0.001
    if random.random() < idSwitchProbability:
        trackDictionaryEntry.idShift += 1
        trackDictionaryEntry.lastIdShiftAt = rospy.Time.now()

    idShiftAmount = 66 # better don't change this as other mock components might rely upon this to consistently fake their data across ID switches
    trackedPerson.track_id = track_id + trackDictionaryEntry.idShift * idShiftAmount
    trackedPerson.age = rospy.Time.now() - trackDictionaryEntry.lastIdShiftAt

    # Determine track position
    setPoseAndTwistFromAngle(trackedPerson.pose, trackedPerson.twist, angle, radius, moving)

    # Track position noise
    trackedPerson.pose.pose.position.x += random.random() * 0.1 - 0.05
    trackedPerson.pose.pose.position.y += random.random() * 0.1 - 0.05

    tracks.append(trackedPerson)

    if not trackedPerson.is_occluded:
        detectedPerson = DetectedPerson()
        detectedPerson.detection_id = detection_id
        detectedPerson.confidence = random.random() * 0.5 + 0.5

        detectedPerson.pose = copy.deepcopy(trackedPerson.pose)
        detectedPerson.pose.pose.position.x += random.random() * 0.5 - 0.25 # introduce some noise on detected position
        detectedPerson.pose.pose.position.y += random.random() * 0.5 - 0.25

        detections.append(detectedPerson)

    return

# Main code
trackTopic = '/spencer/perception/tracked_persons'
trackPublisher = rospy.Publisher( trackTopic, TrackedPersons )

detectionTopic = '/spencer/perception/detected_persons' 
detectionPublisher = rospy.Publisher( detectionTopic, DetectedPersons )

rospy.init_node( 'mock_detected_and_tracked_persons' )
br = tf.TransformBroadcaster()

# State variables
startTime = rospy.Time.now()
currentCycle = 0
currentAngle = 0.0
angleStep = 1.0 * pi / 180.
idShift = 0
updateRateHz = 20
msgCounter = 0
detectionCounter = 0
trackDictionary = dict()

# Test coordinate frame for checking if mapping into the fixed frame works correctly
frameOffset = (0, 0, 0)
frameOrientation = tf.transformations.quaternion_from_euler(0,  0,  0) # 90.0 / 180.0 * pi

rospy.loginfo("Publishing mock data on " + detectionTopic + " and " + trackTopic)
rate = rospy.Rate(updateRateHz)

while not rospy.is_shutdown():
    detectionFrameId = "imaginary_sensor_frame"
    br.sendTransform(frameOffset, frameOrientation, rospy.Time.now(), detectionFrameId, "odom")

    detectedPersons = DetectedPersons()
    detectedPersons.header.seq = msgCounter + 123456
    detectedPersons.header.frame_id = detectionFrameId
    detectedPersons.header.stamp = rospy.Time.now()

    trackedPersons = TrackedPersons()
    trackedPersons.header.seq = msgCounter
    trackedPersons.header.frame_id = "odom"
    trackedPersons.header.stamp = detectedPersons.header.stamp # synchronized with detections!

    tracks = trackedPersons.tracks;
    detections = detectedPersons.detections;

    # These are our fake tracks, created on a polar coordinate grid
    createTrackAndDetection(tracks, detections, 0, currentAngle, 2.0)
    createTrackAndDetection(tracks, detections, 1, currentAngle + pi / 2, 2.5)
    createTrackAndDetection(tracks, detections, 2, currentAngle + pi / 1, 3.0)
    createTrackAndDetection(tracks, detections, 3, currentAngle + pi * 1.5, cos(currentAngle) * 3.5  + 7.0)

    createTrackAndDetection(tracks, detections, 4, 0.0, 0.0, moving = False)

    createTrackAndDetection(tracks, detections, 5, 1.0, 7.0, moving = False)
    createTrackAndDetection(tracks, detections, 6, 1.0, 7.8, moving = False)
    createTrackAndDetection(tracks, detections, 7, 1.1, 7.5, moving = False)

    createTrackAndDetection(tracks, detections, 8, 2.1, 10.0, moving = False)
    createTrackAndDetection(tracks, detections, 9, 2.1, 10.7, moving = False)
    createTrackAndDetection(tracks, detections, 10, 2.2, 10.7, moving = False)
    createTrackAndDetection(tracks, detections, 11, 2.2, 11.1, moving = False)
    createTrackAndDetection(tracks, detections, 12, 2.15, 11.4, moving = False)
    
    createTrackAndDetection(tracks, detections, 13, 2.3, 9.4, moving = False)

    createTrackAndDetection(tracks, detections, 14, 2.9, 8.3, moving = False)

    createTrackAndDetection(tracks, detections, 15, 0.7, 13.0, moving = False)

    createTrackAndDetection(tracks, detections, 16, currentAngle + pi / 1.05, 7.0)
    createTrackAndDetection(tracks, detections, 17, currentAngle + pi / 1, 7.0)
    createTrackAndDetection(tracks, detections, 18, currentAngle + pi * 1.05, 7.0)
    

    trackPublisher.publish( trackedPersons )
    detectionPublisher.publish( detectedPersons )

    currentAngle += angleStep
    currentCycle += 1

    # Periodically shift the IDs to simulate tracks being removed and new tracks being added
    #if(currentCycle % (updateRateHz * 15) == 0) :
    #    idShift += len(tracks)

    rate.sleep()