#! /usr/bin/env python

import roslib
import rospy
import actionlib
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

	def __init__(self):
		rospy.Subscriber("track_feature_input", PersonEmbeddings, self.track_feature_msg_callback) 		#subscribes to (track,feature) message
		rospy.Service('track_label', trackLabelAssociation, self.track_label_service_callback)
		self.actionlibClient = actionlib.SimpleActionClient('labels2Features', labels2FeaturesAction)
		self.actionlibClient.wait_for_server()

	def track_feature_msg_callback(self, personEmbeddings):
		elements = personEmbeddings.elements
		features = []
		
		for element in elements:
			elem_track_id = element.track_id
			elem_embedding = element.embedding
			if elem_track_id == self.track_id:
				features.append(elem_embedding)

		if len(features)==0:
			return
		
		goal = labels2FeaturesGoal()
		
		label2Features_msg = LabelToFeatures()
		label2Features_msg.person_name = self.track_label
		label2Features_msg.reset_label = False
		label2Features_msg.person_embeddings = PersonEmbeddings()

		for embedding in features:
			person_embedding = PersonEmbedding()
			person_embedding.embedding = embedding
			person_embedding.track_id = self.track_id
			label2Features_msg.person_embeddings.elements.append(person_embedding)

		goal.elements.append(label2Features_msg)
		self.actionlibClient.send_goal(goal)
		self.actionlibClient.wait_for_result(rospy.Duration.from_sec(10.0))
		
	def track_label_service_callback(self, trackIdentityAssociationServiceRequest):
		self.track_id = trackIdentityAssociationServiceRequest.track_id
		self.track_label = trackIdentityAssociationServiceRequest.person_name
		return trackLabelAssociationResponse(True)


'''This node collects Track->Label and Track->Feature mappings, gets Label->Feature mappings from them and sends it to the trainer_node'''
if __name__ == '__main__':
	rospy.init_node('data_associator_node')
	data_associator_object = DataAssociator()
	rospy.spin()