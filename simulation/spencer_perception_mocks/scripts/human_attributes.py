#!/usr/bin/python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Publishes fake human attributes (random gender, random age group, person height based upon age group)
# for tracked persons. Subscribes to /spencer/perception/tracked_persons and publishes at /spencer/perception/human_attributes

import rospy, random

from spencer_tracking_msgs.msg import TrackedPersons
from spencer_human_attribute_msgs.msg import HumanAttributes, ScalarAttribute, CategoricalAttribute

def newDataAvailable(trackedPersons):
  msg = HumanAttributes()
  msg.header = trackedPersons.header
  
  # For each tracked person...
  for tracked_person in trackedPersons.tracks:
    subjectId = tracked_person.track_id
    trackIdIgnoringIdSwitches = tracked_person.track_id % 66;


    #
    # Gender
    #

    genderAttribute = CategoricalAttribute()
    genderAttribute.subject_id = subjectId;
    genderAttribute.type = CategoricalAttribute.GENDER

    actualGender = CategoricalAttribute.GENDER_MALE if trackIdIgnoringIdSwitches % 2 == 0 else CategoricalAttribute.GENDER_FEMALE
    otherGender  = CategoricalAttribute.GENDER_MALE if actualGender == CategoricalAttribute.GENDER_FEMALE else CategoricalAttribute.GENDER_FEMALE

    genderConfidence = 0.5 + random.random() * 0.5
    genderAttribute.values = [ actualGender, otherGender ]
    genderAttribute.confidences = [ genderConfidence, 1.0 - genderConfidence ]

    msg.categoricalAttributes.append(genderAttribute)
    

    #
    # Age group
    #

    ageGroupOptions = {
      0: CategoricalAttribute.AGE_GROUP_0_TO_2,
      1: CategoricalAttribute.AGE_GROUP_3_TO_7,
      2: CategoricalAttribute.AGE_GROUP_8_TO_12,
      3: CategoricalAttribute.AGE_GROUP_13_TO_19,
      4: CategoricalAttribute.AGE_GROUP_20_TO_36,
      5: CategoricalAttribute.AGE_GROUP_37_TO_65,
      6: CategoricalAttribute.AGE_GROUP_66_TO_99
    }

    ageGroupIndex = trackIdIgnoringIdSwitches % len(ageGroupOptions)
    ageGroupConfidence = 0.5 + random.random() * 0.5

    ageGroupAttribute = CategoricalAttribute()
    ageGroupAttribute.subject_id = subjectId;
    ageGroupAttribute.type = CategoricalAttribute.AGE_GROUP

    # Also give some weight to the preceeding age group, and insert
    remainingConfidence = 1.0 - ageGroupConfidence
    if ageGroupIndex > 0:
      precedingAgeGroupConfidence = random.random() % remainingConfidence
      remainingConfidence -= precedingAgeGroupConfidence      
      ageGroupAttribute.values.append(ageGroupOptions[ageGroupIndex - 1])
      ageGroupAttribute.confidences.append(precedingAgeGroupConfidence)

    # Calculate weight for succeeding age group
    if ageGroupIndex < len(ageGroupOptions) - 1:
      succeedingAgeGroupConfidence = random.random() % remainingConfidence
      remainingConfidence -= succeedingAgeGroupConfidence      

    # Insert actual age group
    ageGroupAttribute.values.append(ageGroupOptions[ageGroupIndex])
    ageGroupAttribute.confidences.append(ageGroupConfidence + remainingConfidence)

    # Insert succeeding age group
    if ageGroupIndex < len(ageGroupOptions) - 1:
      ageGroupAttribute.values.append(ageGroupOptions[ageGroupIndex + 1])
      ageGroupAttribute.confidences.append(succeedingAgeGroupConfidence)

    msg.categoricalAttributes.append(ageGroupAttribute)
    

    #
    # Person height
    #

    personHeight = ageGroupIndex / float(len(ageGroupOptions) - 1) * 1.5 + 0.5
    personHeight += random.random() * 0.25 - 0.25/2.0

    heightAttribute = ScalarAttribute()
    heightAttribute.subject_id = subjectId;
    heightAttribute.type = ScalarAttribute.PERSON_HEIGHT

    heightAttribute.values.append(personHeight)
    heightAttribute.confidences.append(1.0)

    msg.scalarAttributes.append(heightAttribute)

  # Publish message
  pub.publish(msg)


def main():
  rospy.init_node('mock_human_attributes')

  trackedPersonsTopic = "/spencer/perception/tracked_persons"
  humanAttributesTopic = "/spencer/perception/human_attributes"

  rospy.Subscriber(trackedPersonsTopic, TrackedPersons, newDataAvailable)
  rospy.loginfo("Subscribing to " + trackedPersonsTopic)
  
  global pub
  pub = rospy.Publisher(humanAttributesTopic, HumanAttributes)
  rospy.loginfo("Publishing mock data on " + humanAttributesTopic)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
        

if __name__ == '__main__':
    main()
