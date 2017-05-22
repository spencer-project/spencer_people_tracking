#! /usr/bin/env python
import rospy
import openface_classifier.msg
import threading
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import LabelEncoder
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from openface_classifier.cfg import classifierConfig as ConfigType
from spencer_tracking_msgs.msg import (
    TrackedPerson,
    TrackedPersons,
    TrackIdentityAssociation,
    TrackIdentityAssociations
)

classifier_path = 'classifier.pkl'
features_file = 'features.npy'

class Classifier:
	def __init__(self):
		rospy.Subscriber("input",,track_feature_msg_callback) #subscribes to (track,feature) message
		self.track_person_assoc_pub = rospy.Publisher('output', TrackIdentityAssociations, queue_size=10)
		self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_classifier_callback)
		self.lock = threading.lock()


	def load_model(self, classifier_path):
		with open(classifier_path, 'r') as f:
            clf = pickle.load(f)
        self.clf = clf

    def load_features(self, feature_path):
    	self.features = np.load(feature_path)
	
	def reconfigure_classifier_callback(self, config, level):
        with self.lock:
        	self.load_model(config['classifier_path'])
        	self.load_features(config['feature_path'])
        return config

	def track_feature_msg_callback(self,data):
		#subscriber callback
		#Case when only single track and feature is there
		# track_id = data.track_id #int
		# feature = data.feature #array of floats
		# label = self.clf.predict(feature)[0]

		#Case when multiple tracks and features are there
		track_assocs = TrackIdentityAssociations()
        track_assocs.header = data.header
        track_list = data.tracks
		features = data.features

		with self.lock:
			for (track,feature) in zip(track_list,features):
				track_assoc = TrackIdentityAssociation()
				label = self.clf.predict(feature)[0]
				track_assoc.track_id = track
				track_assoc.detection_id = track.detection_id
				track_assoc.person_name = label
				track_assocs.tracks.append(track_assoc)
			self.track_person_assoc_pub.publish(track_assocs)


if __name__ == '__main__':
	rospy.init_node('classifier_node')
	classifier_object = Classifier()
	rospy.spin()