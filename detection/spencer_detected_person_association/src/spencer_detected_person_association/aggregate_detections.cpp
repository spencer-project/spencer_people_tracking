#include <pluginlib/class_list_macros.h>
#include "aggregate_detections.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace spencer_detected_person_association
{
    void AggregateDetectionsNodelet::onInit() 
    {
        NODELET_INFO("Initializing AggregateDetectionsNodelet...");
        initSynchronizer(getName(), getNodeHandle(), getPrivateNodeHandle());
        m_seq = 0;
    }

    void AggregateDetectionsNodelet::onNewInputMessagesReceived(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& inputMsgs)
    {
        spencer_tracking_msgs::CompositeDetectedPersons::Ptr outputMsg(new spencer_tracking_msgs::CompositeDetectedPersons);

        outputMsg->header.frame_id = inputMsgs[0]->header.frame_id;
        outputMsg->header.seq = m_seq++;

        foreach(const spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr& inputMsg, inputMsgs)
        {
            // Ensure each topic uses the same coordinate frame ID
            ROS_ASSERT_MSG(inputMsg->header.frame_id == outputMsg->header.frame_id, "All input messages must already be in the same coordinate frame! Got %s and %s!",
                inputMsg->header.frame_id.c_str(), outputMsg->header.frame_id.c_str());

            // Use timestamp of latest message
            if(inputMsg->header.stamp > outputMsg->header.stamp)
                outputMsg->header.stamp = inputMsg->header.stamp;

            // Aggregate CompositeDetectedPerson elements
            outputMsg->elements.insert(outputMsg->elements.end(), inputMsg->elements.begin(), inputMsg->elements.end());
        }

        m_publisher->publish(outputMsg);
    }
}


PLUGINLIB_DECLARE_CLASS(spencer_detected_person_association, AggregateDetectionsNodelet, spencer_detected_person_association::AggregateDetectionsNodelet, nodelet::Nodelet) 
