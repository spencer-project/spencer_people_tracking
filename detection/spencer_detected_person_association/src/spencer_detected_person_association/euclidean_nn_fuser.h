#ifndef _EUCLIDEAN_NN_FUSER_H
#define _EUCLIDEAN_NN_FUSER_H

#include <nodelet/nodelet.h>
#include "nn_fuser.h"

namespace spencer_detected_person_association
{
    /// Fuses multiple spencer_tracking_msgs/CompositeDetectedPersons messages into a joint spencer_tracking_msgs/CompositeDetectedPerson message
    /// by associating detections received on different topics based upon their distance to each other.
    /// The input messages must be in the same coordinate frame (header.frame_id), which can be ensured via ConvertToCompositeDetectionsNodelet.
    class EuclideanNNFuserNodelet : public NearestNeighborFuserNodelet
    {
    protected:
        /// Compute the distance between a pair of composite detections using the Euclidean distance in X and Y. If outside of gating zone, returns infinity.
        virtual float computeDistance(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2);
    
        /// Fuse the poses of two composite detections by computing the arithmetic mean of the X,Y,Z positions as well as the corresponding covariances.
        virtual void fusePoses(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2, geometry_msgs::PoseWithCovariance& fusedPose);
    };
}


#endif // _EUCLIDEAN_NN_FUSER_H