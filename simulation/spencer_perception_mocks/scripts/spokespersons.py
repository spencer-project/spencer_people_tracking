#!/usr/bin/python
# Author: Lu Zhang(lu.zhang@tudelft.nl)

import rospy

from spencer_tracking_msgs.msg import TrackedPersons
from spencer_spokesperson_msgs.msg import Spokesperson


def newDataAvailable(trackedPersons):
  spokesperson = Spokesperson()
  spokesperson.header = trackedPersons.header

  for trackedPerson in trackedPersons.tracks:
    spokesperson.subject_id.append(trackedPerson.track_id)
    spokesperson.spokesperson_prob.append((trackedPerson.track_id /1.0)**1.0)

  sum = 0
  for i in range(0, len(spokesperson.subject_id)):
    sum += spokesperson.spokesperson_prob[i]

  for i in range(0, len(spokesperson.subject_id)):
    spokesperson.spokesperson_prob[i] /= sum;

  for i in range(0, len(spokesperson.subject_id)):
    spokesperson.spokesperson_prob[i] = 1-spokesperson.spokesperson_prob[i];

  for i in range(0, len(spokesperson.subject_id)):
    sum += spokesperson.spokesperson_prob[i]

  for i in range(0, len(spokesperson.subject_id)):
    spokesperson.spokesperson_prob[i] /= sum;

  #rospy.loginfo(spokesperson)
  pub.publish(spokesperson)

    
def main():
  rospy.init_node('mock_spokespersons')
  
  trackedPersonsTopic = "/spencer/perception/tracked_persons"
  spokespersonsTopic = "/spencer/perception/spokespersons"

  rospy.Subscriber(trackedPersonsTopic, TrackedPersons, newDataAvailable)
  rospy.loginfo("Subscribing to " + trackedPersonsTopic)
  
  global pub
  pub = rospy.Publisher(spokespersonsTopic, Spokesperson)
  rospy.loginfo("Publishing mock data on " + spokespersonsTopic)


  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
        

if __name__ == '__main__':
    main()