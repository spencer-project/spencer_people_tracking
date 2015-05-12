/* Created on: May 07, 2014. Author: Timm Linder */
#ifndef _ROSINTERFACE_H
#define _ROSINTERFACE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/circular_buffer.hpp>

#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

#include <spencer_diagnostics/publisher.h>

#include <srl_nearest_neighbor_tracker/base/tracker.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/ros/geometry_utils.h>
#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/track.h>


namespace srl_nnt {

/// A wrapper around the tracker which receives spencer_tracking_msgs/DetectedPersons as input, converts them to Observations, invokes the tracker's
/// processCycle() method, and outputs the resulting Tracks as spencer_tracking_msgs/TrackedPersons.
class ROSInterface
{
public:
    /// Creates a ROS interface that subscribes to a DetectedPersons topic using the provided nodeHandle, and
    /// reads parameters using the provided privateNodeHandle.
    ROSInterface(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);

    /// Connect the ROS interface with the provided tracker.
    void connect(Tracker* tracker);

    /// Start processing ROS messages (e.g. detections), returns when the node is shut down.
    /// This triggers kind of the "main loop" of the tracker. 
    void spin();


private:
    /// Callback that is invoked when new detections, and additional supporting low-confidence detections (can be NULL), arrive via ROS.
    void incomingObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr detectedPersons, const spencer_tracking_msgs::DetectedPersons::ConstPtr additionalLowConfidenceDetections);

    /// Converts the ROS DetectedPersons messages into Observation instances, and transforms them into our world frame.
    void convertObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr& detectedPersons, Observations& observations);

    /// Publishes tracks on ROS after completion of a tracking cycle.
    void publishTracks(ros::Time currentRosTime, const Tracks& tracks);

    // Publishes statistics, such as average processing cycle duration and processing rate.
    void publishStatistics();

    /// ROS handles for publisher and subscriber management
    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;
    ros::Subscriber m_detectedPersonsSubscriber;

    spencer_diagnostics::MonitoredPublisher m_trackedPersonsPublisher;
    ros::Publisher m_averageProcessingRatePublisher, m_averageCycleTimePublisher, m_trackCountPublisher;
    
    /// Message filters and subscribers for the case where we additionally subscribe to supporting low-confidence detections.
    typedef message_filters::Subscriber<spencer_tracking_msgs::DetectedPersons> SubscriberType;
    boost::shared_ptr<SubscriberType> m_mainSubscriber, m_additionalSubscriber;

    typedef message_filters::sync_policies::ApproximateTime<spencer_tracking_msgs::DetectedPersons, spencer_tracking_msgs::DetectedPersons> SyncPolicyWithTwoInputs;
    typedef message_filters::Synchronizer<SyncPolicyWithTwoInputs> SynchronizerWithTwoInputs;
    boost::shared_ptr<SynchronizerWithTwoInputs> m_synchronizerWithTwoInputs;

    /// Tracker instance
    Tracker* m_tracker;

    /// Tracker parameters
    Params m_params;

    /// Utility classes
    GeometryUtils m_geometryUtils;
    boost::circular_buffer<double> m_lastCycleTimes; 
};


} // end of namespace srl_nnt


#endif // _ROSINTERFACE_H