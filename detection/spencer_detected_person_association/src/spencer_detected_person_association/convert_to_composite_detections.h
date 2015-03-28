#ifndef _CONVERT_TO_COMPOSITE_DETECTIONS_H
#define _CONVERT_TO_COMPOSITE_DETECTIONS_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>

#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/CompositeDetectedPersons.h>

namespace spencer_detected_person_association
{
    /// Converts detections into composite detections (retaining the original ID, pose, confidence), optionally transforming
    /// the pose into a different coordinate frame.
    class ConvertToCompositeDetectionsNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
        virtual ~ConvertToCompositeDetectionsNodelet();

    private:
        // Creates the publisher once the input topic becomes active
        void createPublisher();

        // Message callback
        void onNewInputMessageReceived(const spencer_tracking_msgs::DetectedPersons::ConstPtr& inputMsg);

        // Executed in a separate thread, continuously monitors if the number of active topics has changed.
        void monitorInputTopic();
        
        // Common frame that all detections are transformed into (if specified)
        std::string m_commonFrameId;
        boost::shared_ptr<tf::TransformListener> m_transformListener;

        // Subscriber for input spencer_tracking_msgs::DetectedPersons
        boost::shared_ptr< ros::Subscriber > m_subscriber;

        // Publisher for output spencer_tracking_msgs::CompositeDetectedPersons
        boost::shared_ptr< ros::Publisher > m_publisher;

        // For monitoring if there are any publishers on the subscribed topic
        boost::mutex m_monitorMutex;
        boost::thread m_monitorThread;
        double m_topicMonitorInterval;
    };
}


#endif // _CONVERT_TO_COMPOSITE_DETECTIONS_H