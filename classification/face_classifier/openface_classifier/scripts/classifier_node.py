#! /usr/bin/env python
import rospy
import numpy as np
import openface_classifier.msg
import threading
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import LabelEncoder
from dynamic_reconfigure.server import Server
from openface_classifier.cfg import ClassifierConfig
from spencer_tracking_msgs.msg import (
    TrackedPerson,
    TrackedPersons,
    TrackIdentityAssociation,
    TrackIdentityAssociations
)
from spencer_vision_msgs.msg import (
    PersonEmbedding,
    PersonEmbeddings,
)

import pickle
import os

class Classifier:
    def __init__(self):
        self.lock = threading.Lock()
        self.classifier_path = rospy.get_param('~classifier_path')
        self.feature_path = rospy.get_param('~feature_path')
        self.embeddings_topic = rospy.get_param('~embeddings_topic')
        self.labeled_tracks_topic = rospy.get_param('~labeled_tracks_topic')
        self.clf = None
        self.features = None
        rospy.Subscriber(self.embeddings_topic, PersonEmbeddings, self.track_feature_msg_callback) #subscribes to (track,feature) message
        self.track_person_assoc_pub = rospy.Publisher(self.labeled_tracks_topic, TrackIdentityAssociations, queue_size=10)
        self.server = Server(ClassifierConfig, self.reconfigure_classifier_callback)
        self.load_model(self.classifier_path)
        self.load_features(self.feature_path)


    def load_model(self, classifier_path):
        # print 'load from ',classifier_path
        try:
            with open(classifier_path, 'r') as f:
                self.clf = pickle.load(f)
        except IOError:
            print 'File path for classifier not exist'

    def load_features(self, feature_path):
        # print 'load from ',feature_path
        try:
            self.features = np.load(feature_path)
        except IOError:
            print 'File path for features not exist'

    def reconfigure_classifier_callback(self, config, level):
        # print 'dynreconfig classifier callback'
        with self.lock:
            print 'lock acquired'
            self.classifier_path = config['classifier_path']
            self.feature_path = config['feature_path']
            self.load_model(self.classifier_path)
            self.load_features(self.feature_path)
        return config

    def classify(self,feature):
        '''Classification logic for unknown class included based on thresholded dot product'''
        dot_prod_threshold_unknown = 0.6
        dist,ind = self.clf.kneighbors([feature])
        nn_product = np.vdot(feature,self.features[ind[0]])
        if nn_product>dot_prod_threshold_unknown:
            feature = np.array([feature])
            label = str(self.clf.predict(feature))
        else:
            label = 'Unknown'
        return label

    def track_feature_msg_callback(self, personEmbeddings):
        #subscriber callback
        #Case when only single track and feature is there
        # track_id = data.track_id #int
        # feature = data.feature #array of floats
        # label = self.clf.predict(feature)[0]

        #Case when multiple tracks and features are there
        print 'subscriber callback'
        track_assocs = TrackIdentityAssociations()
        track_assocs.header = personEmbeddings.header
        elements = personEmbeddings.elements

        with self.lock:
            self.load_model(self.classifier_path)
            self.load_features(self.feature_path)
            if self.clf and self.features:
                for element in elements:
                    track_assoc = TrackIdentityAssociation()
                    label = self.classify(element.embedding)
                    track_assoc.track_id = element.track_id
                    track_assoc.detection_id = element.detection_id
                    track_assoc.person_name = label
                    track_assocs.tracks.append(track_assoc)
                self.track_person_assoc_pub.publish(track_assocs)
            else:
                print 'No model initialised'


if __name__ == '__main__':
    rospy.init_node('classifier_node')
    classifier_object = Classifier()
    rospy.spin()
