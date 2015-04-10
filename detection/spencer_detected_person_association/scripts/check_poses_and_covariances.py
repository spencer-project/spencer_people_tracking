#!/usr/bin/env python
"""
Checks all poses and covariances on a spencer_tracking_msgs/CompositeDetectedPersons topic to see if the poses
look plausible (i.e. no extremely large values) and the covariance matrices are positive semi-definite and symmetric.
If not, the offending CompositeDetectedPerson is output to the console.
"""
import rospy, numpy, scipy.linalg
from spencer_tracking_msgs.msg import CompositeDetectedPersons, DetectedPersons, DetectedPerson

def isPositiveSemiDefinite(A, tol=1e-6):
  E,V = scipy.linalg.eigh(A)
  return numpy.all(E > -tol)

def newCompositeDetectedPersonsAvailable(compositeDetectedPersons):
    errorStr = ""
    for compositeDetectedPerson in compositeDetectedPersons.elements:
        errors = []
        
        # Check position for extreme values
        z = numpy.array([compositeDetectedPerson.pose.pose.position.x, compositeDetectedPerson.pose.pose.position.y, compositeDetectedPerson.pose.pose.position.z])
        if numpy.any(numpy.absolute(z) > 10000):
            errors.append("Extreme position value (x/y/z) detected!")

        # Extract covariance matrix
        R = numpy.zeros((3,3))
        for row in xrange(0, 3):
            for col in xrange(0, 3):
                R[row, col] = compositeDetectedPerson.pose.covariance[row*6 + col]

        # Check symmetry of covariance matrix
        symmetric = True
        for row in xrange(0, 3):
            for col in xrange(0, 3):
                if abs(R[row, col] - R[col, row]) > 1e-6:
                    errors.append("Covariance matrix is not symmetric")
                    symmetric = False
                    break
            if not symmetric:
                break

        # Check positive definiteness of covariance matrix
        if symmetric and not isPositiveSemiDefinite(R):
            errors.append("Covariance matrix is not positive (semi-)definite!")

        # Output errors to console, if any
        if errors:
            errorStr += "Suspicious CompositeDetectedPerson with ID #%d detected (modality: %s). Problems were:\n- %s\n\nOffending CompositeDetectedPerson:\n    %s\n---\n\n" % (compositeDetectedPerson.composite_detection_id,
                ",".join([origDet.modality for origDet in compositeDetectedPerson.original_detections]), "\n- ".join(errors), str(compositeDetectedPerson).replace("\n", "\n    "))
 
    if errorStr:
        rospy.logwarn(errorStr)


if __name__ == '__main__':
    arguments = rospy.myargv()
    rospy.init_node("check_poses_and_covariances")

    compositeDetectedPersonsTopic = "composite_detected_persons"
    compositeDetectedPersonsSubscriber = rospy.Subscriber(compositeDetectedPersonsTopic, CompositeDetectedPersons, newCompositeDetectedPersonsAvailable, queue_size=20)

    rospy.loginfo("Checking poses and covariances on CompositeDetectedPersons topic %s" % (compositeDetectedPersonsTopic))
    rospy.spin()