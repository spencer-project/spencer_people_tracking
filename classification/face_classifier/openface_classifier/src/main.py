#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib

# roslib.load_manifest('my_package')

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from rwth_perception_people_msgs.msg import UpperBodyDetector
from spencer_tracking_msgs.msg import (
    CompositeDetectedPersons,
    DetectedPerson,
    DetectedPersons,
    TrackedPerson,
    TrackedPersons,
    )

import message_filters

import spencer_detected_person_association

faceCascade = cv2.CascadeClassifier('/opt/ros/kinetic/share/OpenCV-3.2.0-dev/haarcascades/haarcascade_frontalface_default.xml')

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher('image_tracked', Image, queue_size=10)
        self.track_pub = rospy.Publisher('persons_tracked', TrackedPersons, queue_size=100)

        self.bridge = CvBridge()

        image_sub = \
            message_filters.Subscriber('/spencer/sensors/rgbd_front_top/rgb/image_rect_color'
                , Image)
        detections_sub = \
            message_filters.Subscriber('/spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector/detections'
                , UpperBodyDetector)
        detected_persons_sub = \
            message_filters.Subscriber('/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body'
                , DetectedPersons)
        tracked_persons_sub = \
            message_filters.Subscriber('persons_tracked'
                , TrackedPersons)

        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub, detected_persons_sub, tracked_persons_sub], 30, 1)
        self.time_sync.registerCallback(self.time_sync_callback)

        track_sync_sub = message_filters.Subscriber('/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body', DetectedPersons)
        self.track_sync = spencer_detected_person_association.TrackSynchronizer(track_sync_sub, 100)
        self.track_sync.registerCallback(self.track_sync_callback)

    def time_sync_callback(
        self,
        image,
        detections,
        detected_persons,
        tracked_persons,
        ):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError, e:
            print(e)
        print('time_sync_callback')

        detectionIndexLookup = {}
        for i, detected_person in enumerate(detected_persons.detections):
            detectionIndexLookup[detected_person.detection_id] = i
        trackLookups = {}
        for track in tracked_persons.tracks:
            track_id = track.track_id
            detection_id = track.detection_id
            try:
                index = detectionIndexLookup[detection_id]
                trackLookup = {'track_id': track_id, 'index':index}
                trackLookups[track.detection_id] = trackLookup
            except:
                pass

        if not trackLookups:
            print('No track.detection_id found in detected_persons.detections')
        else:
            for detection_id in trackLookups:
                trackLookup = trackLookups[detection_id]
                index = trackLookup['index']
                track_id = trackLookup['track_id']

                print('detection_id: {} track_id: {}'.format(detection_id, track_id))
                x = detections.pos_x[index]
                y = detections.pos_y[index]
                w = detections.width[index]
                h = detections.height[index]

                cv_image_crop = cv_image[y:y + h, x:x + w]
                gray = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2GRAY)
                faces = faceCascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=3,
                    minSize=(50, 50),
                    flags = cv2.CASCADE_SCALE_IMAGE
                )

                # Draw a rectangle around the faces
                for (x, y, w, h) in faces:
                    cv2.rectangle(cv_image_crop, (int(x-w*0.1), int(y-h*0.1)), (x+int(w*1.1), y+int(h*1.1)), (0, 255, 0), 2)

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(
                    cv_image_crop,
                    str(track_id%10000000),
                    (0, h),
                    font,
                    1*w/160.0,
                    (255, 255, 255),
                    2,
                    )

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_crop,
                                           'bgr8'))
                except CvBridgeError, e:
                    print(e)

                break

    def track_sync_callback(
        self,
        trackAssociation,
        detected_persons):
        tracked_persons = TrackedPersons()
        tracked_persons.header = detected_persons.header
        for detected_person in detected_persons.detections:
            detection_id = detected_person.detection_id
            track_id = trackAssociation.lookupTrackId(detection_id)
            if track_id is not None:
                # print('detection_id: {} track_id: {}'.format(detection_id, track_id))
                tracked_person = TrackedPerson()
                tracked_person.track_id = track_id
                tracked_person.detection_id = detection_id
                tracked_persons.tracks.append(tracked_person)
        self.track_pub.publish(tracked_persons)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
