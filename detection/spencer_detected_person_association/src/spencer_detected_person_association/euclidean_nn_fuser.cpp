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


PLUGINLIB_DECLARE_CLASS(spencer_detected_person_association, EuclideanNNFuserNodelet, spencer_detected_person_association::EuclideanNNFuserNodelet, nodelet::Nodelet) 
