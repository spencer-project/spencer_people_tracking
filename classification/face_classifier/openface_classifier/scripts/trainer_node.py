#! /usr/bin/env python

import roslib
import rospy
import actionlib
import dynamic_reconfigure.client
import openface_classifier.msg
from collections import defaultdict
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import LabelEncoder
#from group of messages import Track-Feature message from Ruffin #Subscribe to it.

#These file names will come from the launch file
features_file = 'features.npy'
labels_file = 'labels.npy'
classifier_file = 'classifier.pkl'

class Trainer:
	# create messages that are used to publish feedback/result
	_feedback = openface_classifier.msg.trackLabelFeedback()
	_result = openface_classifier.msg.trackLabelResult()

	def __init__(self):
		rospy.Subscriber("input",,track_feature_msg_callback) #subscribes to (track,feature) message
		self.action_server = actionlib.SimpleActionServer('trackLabel', trackLabelAction, self.execute, False)
		self.action_server.start()
		self.client = dynamic_reconfigure.Client('classifier_node',timeout=30) #dynamic_reconfig for client
		self.track_feature_map = defaultdict(list) #track_id->feature

	def get_data(self):
		features = np.load('features.npy') #load feature file. features matrix is n_samples X num_features(128 for openface)
		labels = np.load('labels.npy') #load labels file. (n_samples,)
		return features,labels

	#subscriber callback
	def track_feature_msg_callback(self,data):
		#Case: when message has a single track and feature association
		# track_id = data.track_id #int
		# feature = data.feature #array of floats
		# self.track_feature_map[track_id].append(feature)

		#Case: when message has a list of tracks and list of features associated
		track_list = data.tracks
		features = data.features

		for (track,feature) in zip(track_list,features):
			self.track_feature_map[track].append(feature)

	def track_label_actionlib_callback(self, goal):
		self._result.training_success = False
		query_track = goal.track_id
		query_label = goal.person_name
		
		if query_track in self.track_feature_map:
			query_track_feature_list = self.track_feature_map[query_track]
			
			features,labels = self.get_data()
			
			for i,feat in enumerate(query_track_feature_list):
				features = np.vstack((features,feat))
				labels = np.hstack((labels,query_label))
				progress = (i+1.0)/len(query_track_feature_list)*100
				self._feedback.percent_complete = progress
				self.action_server.publish_feedback(self._feedback)
			
			#Training classifier
			clf = KNeighborsClassifier(n_neighbors=1)
			clf.fit(features,labels)
			
			#Saving features, labels and classifier
			np.save(feature_file,features)
			np.save(labels_file,labels)
			with open(classifier_file, 'w') as f:
				pickle.dump(clf, f)
			
			self._result.training_success = True
			#update configuration on classifier_node
			client.update_configuration(
				{"classifier_path":classifier_file,
				 "feature_path":feature_file}
			)
		
		self.action_server.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node('trainer_node')
	trainer_object = Trainer()
	rospy.spin()