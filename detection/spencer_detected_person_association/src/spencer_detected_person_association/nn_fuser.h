#ifndef _NN_FUSER_H
#define _NN_FUSER_H

#include <nodelet/nodelet.h>
#include "composite_detected_persons_synchronizer.h"

namespace spencer_detected_person_association
{
    /// Fuses multiple spencer_tracking_msgs/CompositeDetectedPersons messages into a joint spencer_tracking_msgs/CompositeDetectedPerson message
    /// by associating detections received on different topics based upon their distance to each other.
    /// The input messages must be in the same coordinate frame (header.frame_id), which can be ensured via ConvertToCompositeDetectionsNodelet.
    class NearestNeighborFuserNodelet : public nodelet::Nodelet, protected CompositeDetectedPersonsSynchronizer
    {
    public:
        virtual void onInit();

    protected:
        /// Implements method of abstract DetectedPersonsSynchronizer base class
        virtual void onNewInputMessagesReceived(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& msgs);

        /// Compute the distance between a pair of composite detections. If outside of gating zone, returns infinity.
        virtual float computeDistance(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2) = 0;
    
        /// Fuse the poses of two composite detections.
        virtual void fusePoses(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2, geometry_msgs::PoseWithCovariance& fusedPose) = 0;
    
    private:
        int m_seq, m_lastDetectionId, m_detectionIdIncrement;
    };
}


#endif // _NN_FUSER_H