/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <pluginlib/class_list_macros.h>
#include "euclidean_nn_fuser.h"

#include <Eigen/Core>
#include <limits>
#include <eigen_conversions/eigen_msg.h>


namespace spencer_detected_person_association
{
    float EuclideanNNFuserNodelet::computeDistance(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2)
    {
        // Compute the Euclidean distance in X and Y
        float distance = hypot(d1.pose.pose.position.x - d2.pose.pose.position.x, d1.pose.pose.position.y - d2.pose.pose.position.y);

        // Gating: If it fails, set distance to infinity
        double gatingDistance = 0.5; getPrivateNodeHandle().getParamCached("gating_distance", gatingDistance);
        if(distance > gatingDistance) distance = std::numeric_limits<float>::infinity();

        return distance;
    }

    void EuclideanNNFuserNodelet::fusePoses(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2, geometry_msgs::PoseWithCovariance& fusedPose)
    {
        // Obtain and normalize weights of arithmetic mean
        double weight1 = 0.5; getPrivateNodeHandle().getParamCached("pose_weight_for_topic1", weight1);
        double weight2 = 0.5; getPrivateNodeHandle().getParamCached("pose_weight_for_topic2", weight2);

        const float weightSum = weight1 + weight2;
        weight1 /= weightSum; weight2 /= weightSum;

        // Compute the arithmetic mean of the X,Y,Z positions as well as the corresponding covariances.
        fusedPose.pose.position.x = weight1 * d1.pose.pose.position.x + weight2 * d2.pose.pose.position.x;
        fusedPose.pose.position.y = weight1 * d1.pose.pose.position.y + weight2 * d2.pose.pose.position.y;
        fusedPose.pose.position.z = weight1 * d1.pose.pose.position.z + weight2 * d2.pose.pose.position.z;

        // Interpolate the orientation using SLERP
        Eigen::Quaterniond q1, q2, qInterpolated;
        tf::quaternionMsgToEigen(d1.pose.pose.orientation, q1);
        tf::quaternionMsgToEigen(d2.pose.pose.orientation, q2);
        qInterpolated = q1.slerp(weight2, q2);
        tf::quaternionEigenToMsg(qInterpolated, fusedPose.pose.orientation);

        // Take component-wise minimum of the covariance matrices
        for(size_t i = 0; i < d1.pose.covariance.size(); i++) {
            // FIXME: Is this a good idea for tracking? This is similar to using the 'stronger peak' of the two distributions,
            // as we hopefully don't get less-informed by fusing the two detections (if our association makes sense)
            // There is probably no 'correct way' as the mixture of two Gaussians is not a Gaussian
            fusedPose.covariance[i] = std::min(d1.pose.covariance[i], d2.pose.covariance[i]);
        }
    }
}


PLUGINLIB_EXPORT_CLASS(spencer_detected_person_association::EuclideanNNFuserNodelet, nodelet::Nodelet)
