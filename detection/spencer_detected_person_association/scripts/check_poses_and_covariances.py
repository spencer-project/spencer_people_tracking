#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


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
