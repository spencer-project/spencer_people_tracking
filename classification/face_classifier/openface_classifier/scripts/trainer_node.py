#! /usr/bin/env python

import roslib
import rospy
import actionlib
import openface_classifier.msg
from collections import defaultdict
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import LabelEncoder

from dynamic_reconfigure.client import Client
from openface_classifier.msg import labels2FeaturesAction

from spencer_vision_msgs.msg import (
    PersonEmbedding,
    PersonEmbeddings,
)

import pickle
import numpy as np
import os

dir = os.path.dirname(__file__)

class Trainer:
	# create messages that are used to publish feedback/result
	
	def __init__(self):
		# self.classifier_path = rospy.get_param('~classifier_path')
		# self.feature_path = rospy.get_param('~feature_path')
		# self.labels_path = rospy.get_param('~labels_path')
		self.classifier_path =  os.path.join(dir, '../config/models/classifier.pkl') #TODO: pick from rosparam
		self.feature_path =  os.path.join(dir, '../config/models/features.npy')		 #TODO: pick from rosparam
		self.labels_path =  os.path.join(dir, '../config/models/labels.npy')		 #TODO: pick from rosparam

		self.action_server = actionlib.SimpleActionServer('labels2Features', labels2FeaturesAction, self.labels2Features_actionlib_callback, False)
		self.action_server.start()
		print 'ActionLib server started'
		# self.client = Client('/spencer/classification_internal/classified_tracks/classifier',timeout=30) #dynamic_reconfig for client

	def get_data(self):
		print 'get_data called'
		self.label_feature_map = defaultdict(list) #label->feature list. Needed for the reset feature.
		try:
			features = np.load(self.feature_path) #load feature file. features matrix is n_samples X num_features(128 for openface)
			labels = np.load(self.labels_path) #load labels file. (n_samples,)
			for label,feat in zip(labels,features):
				self.label_feature_map[label].append(feat)
		except IOError:
			print('Files cant be read')

	def labels2Features_actionlib_callback(self, goal):
		print 'labels2Features_actionlib_callback'
		
		self.get_data()

		for datum in goal.elements:
			shouldReset = datum.reset_label
			print 'shouldReset:',shouldReset
			person_name = datum.person_name
			print 'person_name:', person_name
			person_embeddings = datum.person_embeddings.elements
			if shouldReset:
				self.label_feature_map[person_name] = []
			for x in person_embeddings:
				self.label_feature_map[person_name].append(x.embedding)
		# print self.label_feature_map
		features = np.empty((0,128),float)
		labels =  np.empty((0),str)

		for label in self.label_feature_map:
			feature_list = self.label_feature_map[label]
			for f in feature_list:
				features = np.vstack((features,f))
				labels = np.hstack((labels,label))
		
		# print('features: ', features.shape)
		print 'Features'
		print features
		# print('labels: ', labels.shape)
		print labels
		#Training classifier
		clf = KNeighborsClassifier(n_neighbors=1)
		clf.fit(features,labels)

		#Saving features, labels and classifier
		np.save(self.feature_path,features)
		np.save(self.labels_path,labels)
		with open(self.classifier_path, 'w') as f:
			pickle.dump(clf, f)

		#update configuration on classifier_node
		# self.client.update_configuration(
		# 	{"classifier_path":self.classifier_path,
		# 	 "feature_path":self.feature_path}
		# )

		self.action_server.set_succeeded()

if __name__ == '__main__':
	rospy.init_node('trainer_node')
	trainer_object = Trainer()
	rospy.spin()
