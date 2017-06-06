#! /usr/bin/env python

import roslib
import rospy
from spencer_tracking_msgs.msg import (
    TrackedPerson,
    TrackedPersons,
    TrackIdentityAssociation,
    TrackIdentityAssociations
)
from std_msgs.msg import String
class PersonCaller:

	def __init__(self):
		self.labeled_tracks_topic = rospy.get_param('~labeled_tracks_topic')
		self.call_person_topic = rospy.get_param('~call_person_topic')
		self.track_label_map = {}
		self.label_track_map = {}
		self.timeout = rospy.Duration(60*60) #5 minutes. Time after which old entries are cleared from dicts
		rospy.Subscriber(self.labeled_tracks_topic, TrackIdentityAssociations, self.classified_track_callback) #subscribes to (track,feature) message
		self.person_caller_pub = rospy.Publisher(self.call_person_topic, String, queue_size=10)

	def call_person(self, labelled_track):
		track_id = labelled_track.track_id
		label    = labelled_track.person_name

		if (label not in self.label_track_map) or (label in self.label_track_map and track_id not in self.track_label_map):
			msg = 'Hello '+str(label)+'\n Nice to meet you'
			self.person_caller_pub.publish(msg)

	def update_track_label_mapping(self, labelled_track):
		track_id = labelled_track.track_id
		label    = labelled_track.person_name

		if label not in self.label_track_map:
			#new person seen for the first time
			print 'new person seen ',(label,track_id)
			self.label_track_map[label] = {
			 'track_id':track_id,
			 'timestamp':rospy.Time.now()
			}
			self.track_label_map[track_id] = label
		else:
			#check if track got updated
			old_track_id = self.label_track_map[label]['track_id']	
			if track_id != old_track_id:
				print 'person',label,' updated from track:',old_track_id,' to new track:',track_id
				self.label_track_map[label] = {
				'track_id':track_id,
				'timestamp':rospy.Time.now()
				}

				if old_track_id in self.track_label_map and self.track_label_map[old_track_id] == label:
					del self.track_label_map[old_track_id]

				if track_id in self.track_label_map:
					old_name = self.track_label_map[track_id]
					del self.label_track_map[old_name]

				self.track_label_map[track_id] = label 


	def remove_expired_people(self):
		time_now = rospy.Time.now()
		deleted_people_list = []
		
		for label in self.label_track_map:
			timestamp = self.label_track_map[label]['timestamp']
			track_id = self.label_track_map[label]['track_id']
			if time_now-timestamp > self.timeout:
				print 'removed (track_id.label)',(track_id,label)
				deleted_people_list.append({'label':label,'track_id':track_id})

		for deleted_track in deleted_people_list:
			track_id = deleted_track['track_id']
			label = deleted_track['label']
			del self.track_label_map[track_id]
			del self.label_track_map[label]

	def classified_track_callback(self,labelled_tracks):	
		for labelled_track in labelled_tracks.tracks:
			if labelled_track.person_name!='Unknown':
				self.call_person(labelled_track)
				self.update_track_label_mapping(labelled_track)
		
		self.remove_expired_people()	

if __name__ == '__main__':
    rospy.init_node('call_person_node')
    person_caller_object = PersonCaller()
    rospy.spin()