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
        self.track_pub = rospy.Publisher('persons_tracked', Image, queue_size=10)

        self.bridge = CvBridge()

        image_sub = \
            message_filters.Subscriber('/spencer/sensors/rgbd_front_top/rgb/image_rect_color'
                , Image)
        detections_sub = \
            message_filters.Subscriber('/spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector/detections'
                , UpperBodyDetector)
        persons_sub = \
            message_filters.Subscriber('/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body'
                , DetectedPersons)

        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub, persons_sub], 30, 1)
        self.time_sync.registerCallback(self.time_sync_callback)

        track_sync_sub = message_filters.Subscriber('/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body', DetectedPersons)
        self.track_sync = spencer_detected_person_association.TrackSynchronizer(track_sync_sub, 100)
        self.track_sync.registerCallback(self.track_sync_callback)

    def time_sync_callback(
        self,
        image,
        detections,
        persons,
        ):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError, e:
            print(e)

        try:
            x = detections.pos_x[0]
            y = detections.pos_y[0]
            w = detections.width[0]
            h = detections.height[0]

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


            person = persons.detections[0]
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                cv_image_crop,
                str(person.detection_id%10000000),
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
        except:
            pass

    def track_sync_callback(
        self,
        trackAssociation,
        persons):
        person = persons.detections[0]
        person_id = person.detection_id
        tracking_id = trackAssociation.lookupTrackId(person_id)
        print('person_id: {} tracking_id: {}'.format(person_id, tracking_id))

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
