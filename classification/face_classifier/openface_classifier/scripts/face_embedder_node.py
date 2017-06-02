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

import openface_classifier

import openface
import dlib

import numpy as np

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
from spencer_vision_msgs.msg import (
    PersonEmbedding,
    PersonEmbeddings,
)

faceCascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')


class face_embedder:

    def __init__(self):

        self.dlib_model_path = rospy.get_param('~dlib_model_path')
        self.openface_model_path = rospy.get_param('~openface_model_path')
        self.imgDim = 96
        self.cuda = False

        self.image_topic = rospy.get_param('~image_topic')
        self.annotations_topic = rospy.get_param('~annotations_topic')
        self.tracks_topic = rospy.get_param('~tracks_topic')
        self.embeddings_topic = rospy.get_param('~embeddings_topic')

        self.align = openface.AlignDlib(self.dlib_model_path)
        self.net = openface.TorchNeuralNet(
            self.openface_model_path,
            imgDim=self.imgDim,
            cuda=self.cuda)

        self.embedding_pub = rospy.Publisher(self.embeddings_topic, PersonEmbeddings, queue_size=10)
        self.bridge = CvBridge()

        image_sub = message_filters.Subscriber(self.image_topic, Image)
        annotation_sub = message_filters.Subscriber(self.annotations_topic, AnnotatedFrame)
        tracked_persons_sub = message_filters.Subscriber(self.tracks_topic, TrackedPersons)

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

        personEmbeddings = PersonEmbeddings()
        personEmbeddings.header = trackedPersons.header

        if trackAnnotations:
            for track_id in trackAnnotations:
                annotation = trackAnnotations[track_id]

                print('track_id: {} annotation_id: {}'.format(track_id, annotation.id))
                x = int(annotation.tlx)
                y = int(annotation.tly)
                w = int(annotation.width)
                h = int(annotation.height)

                x1 = np.clip(x, 0, cv_image.shape[0]-2)
                y1 = np.clip(y, 0, cv_image.shape[1]-2)
                x2 = np.clip(x + w, 1, cv_image.shape[0]-1)
                y2 = np.clip(y + h, 1, cv_image.shape[1]-1)

                cv_image_crop = cv_image[y1:y2, x1:x2]
                cv_image_crop = cv2.cvtColor(cv_image_crop,cv2.COLOR_BGR2RGB)
                cv_image_crop = cv2.cvtColor(cv_image_crop,cv2.COLOR_RGB2BGR)
                bb = self.align.getLargestFaceBoundingBox(cv_image_crop)

                scale = None
                if scale is not None:
                    bwImg = cv2.resize(bwImg, (0,0), fx=scale, fy=scale)
                    scale_inv = 1.0 / scale

                if bb is not None:
                    if scale is not None:
                        bb = dlib.rectangle(
                            left=long(bb.left()*scale_inv),
                            top=long(bb.top()*scale_inv),
                            right=long(bb.right()*scale_inv),
                            bottom=long(bb.bottom()*scale_inv))
                    alignedFace = self.align.align(
                        self.imgDim,
                        cv_image_crop,
                        bb,
                        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)

                    if alignedFace is not None:
                        rep = self.net.forward(alignedFace)

                        personEmbedding = PersonEmbedding()
                        personEmbedding.track_id = track_id
                        personEmbedding.detection_id = annotation.id
                        personEmbedding.embedding = rep
                        personEmbeddings.elements.append(personEmbedding)
            if len(personEmbeddings.elements) > 0:
                self.embedding_pub.publish(personEmbeddings)
        else:
            print('No track.detection_id annotation.id found in common')

def main(args):
    rospy.init_node('face_embedder')
    fe = face_embedder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main(sys.argv)
