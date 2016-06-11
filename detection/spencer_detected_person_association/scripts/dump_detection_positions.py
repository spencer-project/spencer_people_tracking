#!/usr/bin/env python
"""
Dumps positions of DetectedPersons into a CSV file for debugging
"""
import rospy, tf
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import PoseStamped


def newDetectedPersonsAvailable(detectedPersons):
    try:
        tfListener.waitForTransform(detectedPersons.header.frame_id, "odom", detectedPersons.header.stamp, rospy.Duration(0.05))
    except tf.Exception:
        return

    for detectedPerson in detectedPersons.detections:
        poseStamped = PoseStamped()
        poseStamped.pose = detectedPerson.pose.pose
        poseStamped.header = detectedPersons.header

        transformedPoseStamped = tfListener.transformPose("odom", poseStamped)

        pos = transformedPoseStamped.pose.position
        csvFile.write("%f\t%f\t%f\t%f\n" % (pos.x, pos.y, pos.z, detectedPersons.header.stamp.to_sec()) )

    global firstMessageOK
    if not firstMessageOK:
        firstMessageOK = True
        rospy.loginfo("First detections have been received, transformed and written to file!")


if __name__ == '__main__':
    rospy.init_node("dump_detected_person_positions")

    global csvFile
    csvFile = open('detected_person_positions.txt','w')
    csvFile.write("ros_x\tros_y\tros_z\ttimestamp")

    global tfListener
    tfListener = tf.TransformListener()

    global firstMessageOK
    firstMessageOK = False

    detectedPersonsTopic = "detected_persons"
    detectedPersonsSubscriber = rospy.Subscriber(detectedPersonsTopic, DetectedPersons, newDetectedPersonsAvailable, queue_size=500)

    rospy.loginfo("Dumping detected persons on topic %s to CSV file!" % (detectedPersonsTopic))
    rospy.spin()

    csvFile.close()
