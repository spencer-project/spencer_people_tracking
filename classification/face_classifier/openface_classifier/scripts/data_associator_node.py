#! /usr/bin/env python

import roslib
import rospy
import actionlib
import threading
from openface_classifier.msg import (
	labels2FeaturesAction,
	labels2FeaturesGoal,
	LabelToFeatures
)
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
from openface_classifier.srv import trackLabelAssociation
from collections import defaultdict

class DataAssociator:
	TRACK_ID_KEY = 'track_id'
	FEATURE_LIST_KEY = 'feature_list'
	FEATURE_SIZE_KEY =  'feature_size'
	
	def __init__(self):
		rospy.Subscriber("track_feature_input", PersonEmbeddings, self.track_feature_msg_callback) 		#subscribes to (track,feature) message
		rospy.Service('track_label', trackLabelAssociation, self.track_label_service_callback)
		self.actionlibClient = actionlib.SimpleActionClient('labels2Features', labels2FeaturesAction)
		self.actionlibClient.wait_for_server()
		self.label_track_feature_map = {}
		self.lock = threading.Lock()

	def update_and_correct_track_feature_map(self):
		shouldSendGoal = False
		goal = labels2FeaturesGoal()

		for label in self.label_track_feature_map:
			feature_list_length = len(self.label_track_feature_map[label][FEATURE_LIST_KEY])
			allowed_length = self.label_track_feature_map[label][FEATURE_SIZE_KEY]
			
			if feature_list_length >= allowed_length:
				shouldSendGoal = True
				diff = feature_list_length - allowed_length
				label2Features_msg = LabelToFeatures()
				label2Features_msg.person_name = label
				label2Features_msg.reset_label = False
				label2Features_msg.person_embeddings = PersonEmbeddings()

				for embedding in self.label_track_feature_map[label][FEATURE_LIST_KEY][diff:]:
					person_embedding = PersonEmbedding()
					person_embedding.embedding = embedding
					person_embedding.track_id = self.label_track_feature_map[label][TRACK_ID_KEY]
					label2Features_msg.person_embeddings.elements.append(person_embedding)	

				goal.elements.append(label2Features_msg)

		if shouldSendGoal:
			self.actionlibClient.send_goal(goal)
			self.actionlibClient.wait_for_result(rospy.Duration.from_sec(5.0))


	def track_feature_msg_callback(self, personEmbeddings):
		with self.lock:
			elements = personEmbeddings.elements
			features = []
			
			for element in elements:
				elem_track_id = element.track_id
				elem_embedding = element.embedding
				if elem_track_id == self.track_id:
					features.append(elem_embedding)

			if self.track_label in self.label_track_feature_map:
				
				#logic for adding features to list
				if self.label_track_feature_map[self.track_label][TRACK_ID_KEY] == self.track_id: #append features ti
					self.label_track_feature_map[self.track_label][FEATURE_LIST_KEY].extend(features)
				else: #the list is newly created with current features
					self.label_track_feature_map[self.track_label][FEATURE_LIST_KEY] = features

				#update feature size key
				self.label_track_feature_map[self.track_label][FEATURE_SIZE_KEY] = self.num_features_to_collect
			else:
				self.label_track_feature_map[self.track_label] = {TRACK_ID_KEY:self.track_id,FEATURE_LIST_KEY:features,FEATURE_SIZE_KEY:self.num_features_to_collect}
			
			self.update_and_correct_track_feature_map()
		
	def track_label_service_callback(self, trackIdentityAssociationServiceRequest):
		with self.lock:
			self.track_id = trackIdentityAssociationServiceRequest.track_id
			self.track_label = trackIdentityAssociationServiceRequest.person_name
			self.request_timeout_time = trackIdentityAssociationServiceRequest.end_time
			self.num_features_to_collect = trackIdentityAssociationServiceRequest.num_features
		return trackLabelAssociationResponse(True)


'''This node collects Track->Label and Track->Feature mappings, gets Label->Feature mappings from them and sends it to the trainer_node'''
if __name__ == '__main__':
	rospy.init_node('data_associator_node')
	data_associator_object = DataAssociator()
	rospy.spin()