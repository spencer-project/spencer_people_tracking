#! /usr/bin/env python

import roslib
import rospy
import actionlib
from openface_classifier.msg import (
	labels2FeaturesAction,
	labels2FeaturesGoal,
	LabelToFeatures
)
from spencer_vision_msgs.msg import (
	PersonEmbedding,
	PersonEmbeddings,
)
import random

def make_rand_vector(dims):
	vec = [random.gauss(0, 1) for i in range(dims)]
	mag = sum(x**2 for x in vec) ** .5
	return [x/mag for x in vec]

# class DataAssociator:

# 	def __init__(self):
# 		self.track_feature_sub = rospy
# 		self.client = actionlib.SimpleActionClient('labels2Features', labels2FeaturesAction)


'''This node collects Track->Label and Track->Feature mappings, gets Label->Feature mappings from them and sends it to the trainer_node'''
if __name__ == '__main__':
	rospy.init_node('data_associator_node')
	# data_associator_object = DataAssociator()
	# rospy.spin()
	client = actionlib.SimpleActionClient('labels2Features', labels2FeaturesAction)
	client.wait_for_server()

	goal = labels2FeaturesGoal()
	# Fill in the goal here
	num_labels_to_send = 3
	label_list = ['Jaskaran','Ruffin','Shengye','Priyam','Henrik']
	for i in xrange(num_labels_to_send):
		label2Features_msg = LabelToFeatures()
		label2Features_msg.person_name = random.choice(label_list)
		
		label2Features_msg.person_embeddings = PersonEmbeddings()

		num_embeddings = 1
		for i in range(num_embeddings):
			person_embedding = PersonEmbedding()
			embedding = make_rand_vector(128)
			person_embedding.embedding = embedding

			label2Features_msg.person_embeddings.elements.append(person_embedding)

		label2Features_msg.reset_label = False
		goal.elements.append(label2Features_msg)
	# print goal
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))