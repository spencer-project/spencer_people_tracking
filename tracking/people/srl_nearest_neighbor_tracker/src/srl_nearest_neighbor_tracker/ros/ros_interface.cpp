/* Created on: May 07, 2014. Author: Timm Linder */

#include <srl_nearest_neighbor_tracker/ros/ros_interface.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>


namespace srl_nnt {


ROSInterface::ROSInterface(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
    : m_nodeHandle(nodeHandle), m_privateNodeHandle(privateNodeHandle), m_params(privateNodeHandle), m_geometryUtils(), m_tracker(NULL)
{
    unsigned int queue_size = (unsigned) srl_nnt::Params::get<int>("queue_size", 5);
    
    // Set up circular buffer for benchmarking cycle times
    m_lastCycleTimes.set_capacity(Params::get<int>("cycle_time_buffer_length", 50)); // = size of window for averaging

    // Create ROS publishers
    m_trackedPersonsPublisher = m_nodeHandle.advertise<spencer_tracking_msgs::TrackedPersons>("/spencer/perception/tracked_persons", queue_size);
    m_trackedPersonsPublisher.setExpectedFrequency(20.0, 40.0);
    m_trackedPersonsPublisher.setMaximumTimestampOffset(0.3, 0.1);
    m_trackedPersonsPublisher.finalizeSetup();

    m_averageProcessingRatePublisher = m_privateNodeHandle.advertise<std_msgs::Float32>("average_processing_rate", 1); // for benchmarking
    m_averageCycleTimePublisher = m_privateNodeHandle.advertise<std_msgs::Float32>("average_cycle_time", 1);
    m_trackCountPublisher = m_privateNodeHandle.advertise<std_msgs::UInt16>("track_count", 1);

    // Create ROS subscribers
    const std::string detectedPersonsTopic = "/spencer/perception/detected_persons";
    const std::string additionalLowConfidenceDetectionsTopic = srl_nnt::Params::get<std::string>("additional_low_confidence_detections", "");

    if(additionalLowConfidenceDetectionsTopic.empty()) {
        // Simple variant, just subscribe to a single DetectedPersons topic.
        ROS_INFO_STREAM("Subscribing to single topic " << ros::names::remap(detectedPersonsTopic));
        m_detectedPersonsSubscriber = m_nodeHandle.subscribe<spencer_tracking_msgs::DetectedPersons>(detectedPersonsTopic, queue_size,
            boost::bind(&ROSInterface::incomingObservations, this, _1, spencer_tracking_msgs::DetectedPersons::ConstPtr() ));
    }
    else {
        // Additionally subscribe to a second topic that provides supporting low-confidence detections.
        m_mainSubscriber.reset(new SubscriberType(nodeHandle, detectedPersonsTopic, 1));
        m_additionalSubscriber.reset(new SubscriberType(nodeHandle, additionalLowConfidenceDetectionsTopic, 1));

        int agePenalty = 1000; // Set high age penalty to publish older data faster even if it might not be correctly synchronized.
        privateNodeHandle.getParam("synchronizer_age_penalty", agePenalty);

        int queueSize = 35;
        privateNodeHandle.getParam("synchronizer_queue_size", queueSize);        

        ROS_INFO_STREAM("Subscribing to detections on " << ros::names::remap(detectedPersonsTopic) << " and additional low-confidence detections on "
            << ros::names::remap(additionalLowConfidenceDetectionsTopic) << " with synchronizer queue size " << queueSize);
        
        // Create approximate-time synchronizer
        SyncPolicyWithTwoInputs syncPolicyWithTwoInputs(queueSize);
        syncPolicyWithTwoInputs.setAgePenalty(agePenalty); 
        const SyncPolicyWithTwoInputs constSyncPolicyWithTwoInputs = syncPolicyWithTwoInputs;

        SubscriberType& mainSubscriber = *m_mainSubscriber, &additionalSubscriber = *m_additionalSubscriber; // not sure why this line is necessary (compiler bug?)
        m_synchronizerWithTwoInputs.reset(new SynchronizerWithTwoInputs(constSyncPolicyWithTwoInputs, mainSubscriber, additionalSubscriber));
        m_synchronizerWithTwoInputs->registerCallback(&ROSInterface::incomingObservations, this);
    }
}


void ROSInterface::connect(Tracker* tracker)
{
    // The tracker is set separately after the Params class has been initialized.
    m_tracker = tracker;
}


void ROSInterface::spin()
{
    // This could also be replaced by a busy loop, combined with ros::spinOnce(), if the necessity arises.
    ROS_INFO_STREAM("Tracker is now listening for incoming observations at " << m_detectedPersonsSubscriber.getTopic() << "...");
    ros::spin();
}


void ROSInterface::incomingObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr detectedPersons, const spencer_tracking_msgs::DetectedPersons::ConstPtr additionalLowConfidenceDetections)
{
    double currentTime = detectedPersons->header.stamp.toSec();
    ros::Time currentRosTime = detectedPersons->header.stamp; // to make sure that timestamps remain exactly the same up to nanosecond precision (for ExactTime sync policy)
    
    // Convert DetectedPersons into Observation instances
    Observations observations;
    convertObservations(detectedPersons, observations);

    Observations additionalLowConfidenceObservations;
    convertObservations(additionalLowConfidenceDetections, additionalLowConfidenceObservations);
    
    // Save start time
    ros::WallTime startTime = ros::WallTime::now();

    // Initiate a new tracking cycle (this is where the fun begins!)
    assert(m_tracker != NULL);
    const Tracks& newTracks = m_tracker->processCycle(currentTime, observations, additionalLowConfidenceObservations);

    // Save end time
    ros::WallTime endTime = ros::WallTime::now();
    m_lastCycleTimes.push_back( (endTime - startTime).toSec() );

    // Publish new tracks and statistics (cycle times) 
    publishTracks(currentRosTime, newTracks);
    publishStatistics();
}


void ROSInterface::convertObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr& detectedPersons, Observations& observations)
{
    // Ignore if pointer is not set
    if(!detectedPersons) return;
    
    // We need to convert all coordinates into our fixed world reference frame
    double currentTime = detectedPersons->header.stamp.toSec();
    Eigen::Affine3d transformIntoWorldFrame;
    if(!m_geometryUtils.lookupTransformIntoWorldFrame(detectedPersons->header.stamp, detectedPersons->header.frame_id, transformIntoWorldFrame)) return;

    // Convert DetectedPerson instances (ROS messages) into Observation instances (our own format).
    foreach(const spencer_tracking_msgs::DetectedPerson& detectedPerson, detectedPersons->detections)
    {
        // Heuristic sanity check for detected person poses (if anything in the detector goes wrong or groundtruth annotations are invalid)
        if(!m_geometryUtils.posePassesSanityCheck(detectedPerson.pose, false)) {
            ROS_WARN_STREAM("Pose of DetectedPerson " << detectedPerson.detection_id << " does not pass sanity check, will ignore this detection: " << detectedPerson.pose.pose);
            continue;
        }

        Observation::Ptr observation(new Observation);
        observation->id = detectedPerson.detection_id;
        observation->createdAt = currentTime;
        observation->confidence = detectedPerson.confidence;
        m_geometryUtils.poseToMeanAndCovariance(detectedPerson.pose, observation->z, observation->R, transformIntoWorldFrame);

        observations.push_back(observation);

        ROS_DEBUG("Received observation from ROS (ID=%d) at x=%.2f, y=%.2f", (unsigned int) detectedPerson.detection_id, observation->z(0), observation->z(1));
    }

    //ROS_INFO_STREAM("Received " << observations.size() << " observation(s) from ROS!");
}


void ROSInterface::publishTracks(ros::Time currentRosTime, const Tracks& tracks)
{
    spencer_tracking_msgs::TrackedPersons trackedPersons;
    trackedPersons.header.stamp = currentRosTime;
    trackedPersons.header.seq = m_tracker->getCurrentCycleNo();
    trackedPersons.header.frame_id = m_geometryUtils.getWorldFrame();

    foreach(Track::Ptr track, tracks) {
        spencer_tracking_msgs::TrackedPerson trackedPerson;    

        trackedPerson.track_id = track->id;
        trackedPerson.age = ros::Duration(currentRosTime.toSec() - track->createdAt);
               
        switch(track->trackStatus){
        case Track::MATCHED:
        case Track::NEW:
            trackedPerson.is_matched = true;
            trackedPerson.is_occluded = false;
            trackedPerson.detection_id = track->observation->id;
            break;
        case Track::MISSED:
            trackedPerson.is_matched = false;
            trackedPerson.is_occluded = false;
            break;
        case Track::OCCLUDED:
            trackedPerson.is_matched = false;
            trackedPerson.is_occluded = true;
        }

        m_geometryUtils.meanAndCovarianceToPoseAndTwist(track->state->x(), track->state->C(), trackedPerson.pose, trackedPerson.twist); // state estimate (update)

        // Heuristic sanity check for tracked person poses (if anything in the tracker goes wrong)
        if(!m_geometryUtils.posePassesSanityCheck(trackedPerson.pose, true))
        {
            // Output track state history to console, best viewed using rqt Logger Console plugin
            if(ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN) {
                stringstream ss; size_t historyIndex  = track->stateHistory.size();                
                foreach(FilterState::Ptr historicState, track->stateHistory) {
                    ss << "\n\n=== Historic motion filter state #" << --historyIndex << ": ===\n\n" << historicState << "\n";
                }

                ROS_WARN_STREAM("Pose of TrackedPerson " << trackedPerson.track_id << " first tracked " << trackedPerson.age.toSec()
                    << " sec ago does not pass sanity check, will not publish this track:\n\n"
                    << trackedPerson
                    << "\n\n-----------------\n\n"
                    << "Offending track's motion filter state history:" << ss.str());
            }

            // Skip publishing this track.
            continue;
        }

        trackedPersons.tracks.push_back(trackedPerson);
    }

    // Publish tracked persons
    ROS_INFO("Publishing %zi tracked persons!", tracks.size());
    m_trackedPersonsPublisher.publish(trackedPersons);

    // Publish track count
    std_msgs::UInt16 trackCountMsg;
    trackCountMsg.data = tracks.size();
    m_trackCountPublisher.publish(trackCountMsg);
}


void ROSInterface::publishStatistics()
{
    // Wait until buffer is full
    if(m_lastCycleTimes.size() < m_lastCycleTimes.capacity()) return;

    // Calculate average
    double averageCycleTime = 0.0;
    foreach(float cycleTime, m_lastCycleTimes) {
        averageCycleTime += cycleTime;
    }
    averageCycleTime /= m_lastCycleTimes.size();

    // Publish average cycle time
    std_msgs::Float32 averageCycleTimeMsg;
    averageCycleTimeMsg.data = averageCycleTime;
    m_averageCycleTimePublisher.publish(averageCycleTimeMsg);    

    // Publish average processing time
    std_msgs::Float32 averageProcessingRateMsg;
    averageProcessingRateMsg.data = 1.0 / averageCycleTime;
    m_averageProcessingRatePublisher.publish(averageProcessingRateMsg); 
}

} // end of namespace srl_nnt