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
#include "polar_nn_fuser.h"

#include <Eigen/Core>
#include <limits>
#include <eigen_conversions/eigen_msg.h>


namespace spencer_detected_person_association
{
    float PolarNNFuserNodelet::computeDistance(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2)
    {
        // The distance we compute consists of two parts:
        // (1) The average difference in polar angle between the two detections, but expressed as a Euclidean distance between the angle's end points such that we can sum it up with...
        // (2) The absolute difference in depth, i.e. the absolute difference between their polar radius
        // Usually, the difference in depth is lower-weighted because depth estimates might not be reliable with monocular vision systems

        double phi1 = atan2(d1.pose.pose.position.y, d1.pose.pose.position.x);
        double phi2 = atan2(d2.pose.pose.position.y, d2.pose.pose.position.x);

        double rho1 = hypot(d1.pose.pose.position.x, d1.pose.pose.position.y);
        double rho2 = hypot(d2.pose.pose.position.x, d2.pose.pose.position.y);

        double rho_avg = (rho1 + rho2) / 2.0;

        // Point on the polar ray of detection 1 at the mean radius of the two detections
        double x1_at_rho_avg = rho_avg * cos(phi1);
        double y1_at_rho_avg = rho_avg * sin(phi1);

        // Point on the polar ray of detection 2 at the mean radius of the two detections
        double x2_at_rho_avg = rho_avg * cos(phi2);
        double y2_at_rho_avg = rho_avg * sin(phi2);

        // (1) Average Euclidean distance in polar angle
        double angular_distance = hypot(x1_at_rho_avg - x2_at_rho_avg, y1_at_rho_avg - y2_at_rho_avg);

        // (2) Absolute difference in polar radius
        double radial_distance = std::abs(rho2 - rho1);

        // Gating
        double angular_gating_distance = 0.5; getPrivateNodeHandle().getParamCached("angular_gating_distance", angular_gating_distance);
        if(angular_distance > angular_gating_distance) angular_distance = std::numeric_limits<float>::infinity();

        double radial_gating_distance = 2.5; getPrivateNodeHandle().getParamCached("radial_gating_distance", radial_gating_distance);
        if(radial_distance > radial_gating_distance) radial_distance = std::numeric_limits<float>::infinity();

        // Overall distance is weighted sum of the two components (similar to a 'weighted Manhattan distance', where the two components are angular and radial distances)
        double angular_importance = 0.8; getPrivateNodeHandle().getParamCached("angular_importance", angular_importance);
        double radial_importance = 0.2; getPrivateNodeHandle().getParamCached("radial_importance", radial_importance);

        const float weightSum = angular_importance + radial_importance;
        angular_importance /= weightSum; radial_importance /= weightSum;

        double distance = angular_importance * angular_distance + radial_importance * radial_distance;

        //if(std::isfinite(angular_distance) && std::isfinite(radial_distance))
        //    ROS_INFO("Polar-NN | %zu + %zu:  angular_d=%.1f, radial_d=%.1f, d=%.1f", d1.composite_detection_id, d2.composite_detection_id, angular_distance, radial_distance, distance);

        return distance;
    }

    void PolarNNFuserNodelet::fusePoses(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2, geometry_msgs::PoseWithCovariance& fusedPose)
    {
        // Obtain and normalize weights for angular component
        double angular_weight1 = 0.5; getPrivateNodeHandle().getParamCached("fused_angular_weight_for_topic1", angular_weight1);
        double angular_weight2 = 0.5; getPrivateNodeHandle().getParamCached("fused_angular_weight_for_topic2", angular_weight2);

        const float angularWeightSum = angular_weight1 + angular_weight2;
        angular_weight1 /= angularWeightSum; angular_weight2 /= angularWeightSum;

        // Obtain and normalize weights for radial component -- usually one of them should receive a lower weight in practice, when there is high uncertainty in the depth estimates
        double radial_weight1 = 0.5; getPrivateNodeHandle().getParamCached("fused_radial_weight_for_topic1", radial_weight1);
        double radial_weight2 = 0.5; getPrivateNodeHandle().getParamCached("fused_radial_weight_for_topic2", radial_weight2);

        const float radialWeightSum = radial_weight1 + radial_weight2;
        radial_weight1 /= radialWeightSum; radial_weight2 /= radialWeightSum;

        // Obtain and normalize weights for orientation component (if at all relevant, most detections don't have an orientation...)
        double orientation_weight1 = 0.5; getPrivateNodeHandle().getParamCached("fused_orientation_weight_for_topic1", orientation_weight1);
        double orientation_weight2 = 0.5; getPrivateNodeHandle().getParamCached("fused_orientation_weight_for_topic2", orientation_weight2);

        const float orientationWeightSum = orientation_weight1 + orientation_weight2;
        orientation_weight1 /= orientationWeightSum; orientation_weight2 /= orientationWeightSum;

        // Compute the average polar angle (note that averaging angles is not straightforward)
        double phi1 = atan2(d1.pose.pose.position.y, d1.pose.pose.position.x);
        double phi2 = atan2(d2.pose.pose.position.y, d2.pose.pose.position.x);

        double phi_avg = atan2(0.5*sin(phi1) + 0.5*sin(phi2), 0.5*cos(phi1) + 0.5*cos(phi2)); // http://en.wikipedia.org/wiki/Mean_of_circular_quantities

        // Compute the average polar radius
        double rho1 = hypot(d1.pose.pose.position.x, d1.pose.pose.position.y);
        double rho2 = hypot(d2.pose.pose.position.x, d2.pose.pose.position.y);

        double rho_avg = radial_weight1 * rho1 + radial_weight2 * rho2;

        // Compute resulting X and Y coordinates, for Z just take the arithmetic mean
        fusedPose.pose.position.x = rho_avg * cos(phi_avg);
        fusedPose.pose.position.y = rho_avg * sin(phi_avg);

        fusedPose.pose.position.z = 0.5 * d1.pose.pose.position.z + 0.5 * d2.pose.pose.position.z; // FIXME: Make weights configurable if we ever track in 3D

        // Interpolate the orientation using SLERP
        Eigen::Quaterniond q1, q2, qInterpolated;
        tf::quaternionMsgToEigen(d1.pose.pose.orientation, q1);
        tf::quaternionMsgToEigen(d2.pose.pose.orientation, q2);
        qInterpolated = q1.slerp(orientation_weight2, q2);
        tf::quaternionEigenToMsg(qInterpolated, fusedPose.pose.orientation);

        // Take component-wise minimum of the covariance matrices
        for(size_t i = 0; i < d1.pose.covariance.size(); i++) {
            // FIXME: Is this a good idea for tracking? This is similar to using the 'stronger peak' of the two distributions
            // as we hopefully don't get less-informed by fusing the two detections (if our association makes sense)
            // There is probably no 'correct way' as the mixture of two Gaussians is not a Gaussian
            fusedPose.covariance[i] = std::min(d1.pose.covariance[i], d2.pose.covariance[i]);
        }
    }
}


PLUGINLIB_DECLARE_CLASS(spencer_detected_person_association, PolarNNFuserNodelet, spencer_detected_person_association::PolarNNFuserNodelet, nodelet::Nodelet)
