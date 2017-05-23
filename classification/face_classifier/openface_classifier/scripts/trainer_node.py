#! /usr/bin/env python

import roslib
import rospy
import actionlib
import openface_classifier.msg
from collections import defaultdict
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import LabelEncoder

from dynamic_reconfigure.client import Client
from openface_classifier.msg import trackLabelAction

from spencer_vision_msgs.msg import (
    PersonEmbedding,
    PersonEmbeddings,
)

import pickle
import numpy as np

classifier_path = 'classifier.pkl'
feature_path = 'features.npy'
labels_path = 'labels.npy'

class Trainer:
	# create messages that are used to publish feedback/result
	_feedback = openface_classifier.msg.trackLabelFeedback()
	_result = openface_classifier.msg.trackLabelResult()

	def __init__(self):
		self.classifier_path = rospy.get_param('~classifier_path')
		self.feature_path = rospy.get_param('~feature_path')
		self.labels_path = rospy.get_param('~labels_path')

		self.track_feature_map = defaultdict(list) #track_id->feature

		rospy.Subscriber("input", PersonEmbeddings, self.track_feature_msg_callback) #subscribes to (track,feature) message
		self.action_server = actionlib.SimpleActionServer('trackLabel', trackLabelAction, self.track_label_actionlib_callback, False)
		self.action_server.start()
		self.client = Client('classifier_node',timeout=30) #dynamic_reconfig for client

	def get_data(self):
		try:
			features = np.load(self.feature_path) #load feature file. features matrix is n_samples X num_features(128 for openface)
			labels = np.load(self.labels_path) #load labels file. (n_samples,)
		except IOError:
			features = np.empty((0,128),float)
			labels = np.empty((0,1),str)
			print('Files cant be read')

		return features, labels

	#subscriber callback
	def track_feature_msg_callback(self, personEmbeddings):
		for element in personEmbeddings.elements:
			self.track_feature_map[element.track_id].append(element.embedding)

	def track_label_actionlib_callback(self, goal):
		self._result.training_success = False
		query_track = goal.track_id
		query_label = goal.person_name

		if query_track in self.track_feature_map:
			query_track_feature_list = self.track_feature_map[query_track]

			features,labels = self.get_data()

			for i,feat in enumerate(query_track_feature_list):
				features = np.append(features,np.array(feat),axis=0)
				labels = np.append(labels,query_label,axis=0)
				progress = (i+1.0)/len(query_track_feature_list)*100
				self._feedback.percent_complete = progress
				self.action_server.publish_feedback(self._feedback)

			#Training classifier
			clf = KNeighborsClassifier(n_neighbors=1)
			clf.fit(features,labels)

			#Saving features, labels and classifier
			np.save(self.feature_path,features)
			np.save(self.labels_path,labels)
			with open(self.classifier_path, 'w') as f:
				pickle.dump(clf, f)

			self._result.training_success = True
			#update configuration on classifier_node
			client.update_configuration(
				{"classifier_path":self.classifier_path,
				 "feature_path":self.feature_path}
			)

		self.action_server.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node('trainer_node')
	trainer_object = Trainer()
	rospy.spin()
