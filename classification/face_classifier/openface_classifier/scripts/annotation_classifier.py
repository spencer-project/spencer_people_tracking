#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import roslib
# import rospkg
import rospy
import message_filters

# roslib.load_manifest('my_package')

import sys
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rwth_perception_people_msgs.msg import (
    AnnotatedFrame,
    Annotation,
)
from spencer_tracking_msgs.msg import (
    TrackedPerson,
    TrackedPersons,
)

faceCascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')


class annotation_classifier:

    def __init__(self):
        self.image_pub = rospy.Publisher('annotations_vizulized', Image, queue_size=10)
        self.bridge = CvBridge()

        image_sub = message_filters.Subscriber('image', Image)
        annotation_sub = message_filters.Subscriber('annotations', AnnotatedFrame)
        tracked_persons_sub = message_filters.Subscriber('tracks', TrackedPersons)

        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, annotation_sub, tracked_persons_sub], 100, 0.033)
        self.time_sync.registerCallback(self.time_sync_callback)

    def time_sync_callback(self, image, annotatedFrame, trackedPersons):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError, e:
            print(e)
            return
        print('time_sync_callback')

        annotations = {}
        for i, annotation in enumerate(annotatedFrame.annotations):
            annotations[annotation.id] = annotation

        trackAnnotations = {}
        for track in trackedPersons.tracks:
            track_id = track.track_id
            detection_id = track.detection_id
            try:
                trackAnnotations[track_id] = annotations[detection_id]
            except:
                continue

        if trackAnnotations:
            for track_id in trackAnnotations:
                annotation = trackAnnotations[track_id]

                print('track_id: {} annotation_id: {}'.format(track_id, annotation.id))
                x = int(annotation.tlx)
                y = int(annotation.tly)
                w = int(annotation.width)
                h = int(annotation.height)

                cv_image_crop = cv_image[y:y + h, x:x + w]
                gray = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2GRAY)
                try:
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
                except Exception:
                    continue

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
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_crop, 'bgr8'))
                except CvBridgeError, e:
                    print(e)

                break
        else:
            print('No track.detection_id annotation.id found in common')

def main(args):
    ac = annotation_classifier()
    rospy.init_node('annotation_classifier')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main(sys.argv)
