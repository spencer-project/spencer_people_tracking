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
from openface_classifier.srv import *
from collections import defaultdict

class DataAssociator:

	def __init__(self):
		self.requests_map = {}
		self.lock = threading.Lock()

		self.embeddings_topic = rospy.get_param('~embeddings_topic')
		rospy.Subscriber(self.embeddings_topic, PersonEmbeddings, self.track_feature_msg_callback) 		#subscribes to (track,feature) message
		rospy.Service('track_label', TrackLabelAssociationService, self.track_label_service_callback)
		self.actionlibClient = actionlib.SimpleActionClient('labels2Features', labels2FeaturesAction)
		self.actionlibClient.wait_for_server()

	def check_requests(self):
		shouldSendGoal = False
		goal = labels2FeaturesGoal()
		keys_removed = []
		for track_id in self.requests_map:
			request = self.requests_map[track_id]['request']
			embeddings = self.requests_map[track_id]['embeddings']

			embeddings_length = len(embeddings)
			diff = embeddings_length - request.min_embeddings

			if diff >= 0:
				shouldSendGoal = True
				label2Features_msg = LabelToFeatures()
				label2Features_msg.person_name = request.track_label
				label2Features_msg.reset_label = request.reset_label
				label2Features_msg.person_embeddings = PersonEmbeddings()
				label2Features_msg.person_embeddings.elements = embeddings[diff:]

				goal.elements.append(label2Features_msg)
				keys_removed.append(track_id)

		if shouldSendGoal:
			self.actionlibClient.send_goal(goal)
			print(goal)
			# self.actionlibClient.wait_for_result(rospy.Duration.from_sec(5.0))
			for key in keys_removed:
				del self.requests_map[key]

	def track_feature_msg_callback(self, personEmbeddings):
		print 'track_feature subscriber callback'
		with self.lock:
			# Loop over each track embedding
			for personEmbedding in personEmbeddings.elements:
				track_id = personEmbedding.track_id
				embedding = personEmbedding.embedding
				# Check if track is being monitored
				if track_id in self.requests_map:
					# Add the embedding if so
					self.requests_map[track_id]['embeddings'].append(personEmbedding)
			# Check for completed requests
			self.check_requests()

	def track_label_service_callback(self, new_request):
		print("track_label_service_callback")
		print("trackIdentityAssociationServiceRequest: ", new_request)

		track_id = new_request.track_id
		# track_label = trackIdentityAssociationServiceRequest.person_name
		# request_timeout_time = trackIdentityAssociationServiceRequest.end_time
		# num_features_to_collect = trackIdentityAssociationServiceRequest.num_features

		with self.lock:
			# Check if the track is not already being monitored
			if track_id not in self.requests_map:
				# If not present, then add it
				self.requests_map[track_id] = {
					'request':new_request,
					'embeddings':[],
				}
			else:
				# Otherwise check of the label is still the same
				old_request = self.requests_map[track_id]['request']
				if old_request.track_id != track_id:
					# If not the same, then clear the old embeddings
					self.requests_map[track_id]['embeddings'] = []
				# Update the request
				self.requests_map[track_id]['request'] = new_request
			# Check for completed requests
			self.check_requests()

		return TrackLabelAssociationServiceResponse(True)


'''This node collects Track->Label and Track->Feature mappings, gets Label->Feature mappings from them and sends it to the trainer_node'''
if __name__ == '__main__':
	rospy.init_node('data_associator_node')
	data_associator_object = DataAssociator()
	rospy.spin()
