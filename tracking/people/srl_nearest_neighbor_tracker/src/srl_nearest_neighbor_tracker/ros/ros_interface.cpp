/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, Timm Linder, Social Robotics Lab, University of Freiburg
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

#include <srl_nearest_neighbor_tracker/ros/ros_interface.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <spencer_tracking_msgs/TrackingTimingMetrics.h>


namespace srl_nnt {


ROSInterface::ROSInterface(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
: m_nodeHandle(nodeHandle), m_privateNodeHandle(privateNodeHandle), m_params(privateNodeHandle), m_geometryUtils(), m_tracker(NULL), m_timingInitialized(false)
{
    unsigned int queue_size = (unsigned) srl_nnt::Params::get<int>("queue_size", 5);

    // Set up circular buffer for benchmarking cycle times
    m_lastCycleTimes.set_capacity(Params::get<int>("cycle_time_buffer_length", 50)); // = size of window for averaging

    // Create ROS publishers
    m_trackedPersonsPublisher = m_nodeHandle.advertise<spencer_tracking_msgs::TrackedPersons>("/spencer/perception/tracked_persons", queue_size);
    m_trackedPersonsPublisher.setExpectedFrequency(20.0, 40.0);
    m_trackedPersonsPublisher.setMaximumTimestampOffset(0.3, 0.1);
    m_trackedPersonsPublisher.finalizeSetup();

    // Forward prediction time for track center to take latencies into account
    m_forwardPredictTime = srl_nnt::Params::get<double>("published_track_forward_predict_time", 0.1); // in seconds (e.g. at 1.5m/s, shift centroid forward by 0.1*1.5=0.15m)
    
    // For benchmarking
    m_averageProcessingRatePublisher = m_privateNodeHandle.advertise<std_msgs::Float32>("average_processing_rate", 1);
    m_averageCycleTimePublisher = m_privateNodeHandle.advertise<std_msgs::Float32>("average_cycle_time", 1);
    m_trackCountPublisher = m_privateNodeHandle.advertise<std_msgs::UInt16>("track_count", 1);
    m_averageLoadPublisher = m_privateNodeHandle.advertise<std_msgs::Float32>("average_cpu_load", 1);
    m_timingMetricsPublisher = m_privateNodeHandle.advertise<spencer_tracking_msgs::TrackingTimingMetrics>("tracking_timing_metrics", 10);

    // Create ROS subscribers
    const std::string detectedPersonsTopic = "/spencer/perception/detected_persons";

    // Subscribe to a single DetectedPersons topic
    ROS_INFO_STREAM("Subscribing to detections topic " << ros::names::remap(detectedPersonsTopic));
    m_detectedPersonsSubscriber = m_nodeHandle.subscribe<spencer_tracking_msgs::DetectedPersons>(detectedPersonsTopic, queue_size, boost::bind(&ROSInterface::incomingObservations, this, _1 ));
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


void ROSInterface::incomingObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr detectedPersons)
{
    if (!m_timingInitialized)
    {
        m_startWallTime = m_wallTimeBefore = ros::WallTime::now();
        m_startClock = m_clockBefore =  clock();
        m_timingInitialized = true;
    }
    double currentTime = detectedPersons->header.stamp.toSec();
    ros::Time currentRosTime = detectedPersons->header.stamp; // to make sure that timestamps remain exactly the same up to nanosecond precision (for ExactTime sync policy)

    // Convert DetectedPersons into Observation instances
    Observations observations;
    m_geometryUtils.convertDetectedPersonsToObservations(detectedPersons, observations);

    // Save start time
    ros::WallTime startTime = ros::WallTime::now();
    clock_t startClock = clock();

    // Initiate a new tracking cycle (this is where the fun begins!)
    assert(m_tracker != NULL);
    const Tracks& newTracks = m_tracker->processCycle(currentTime, observations);

    // Save end time
    ros::WallTime endTime = ros::WallTime::now();
    double cycleTime = (endTime - startTime).toSec();
    m_lastCycleTimes.push_back( cycleTime );

    // Publish new tracks and statistics (cycle times) 
    publishTracks(currentRosTime, newTracks);
    publishStatistics(currentRosTime, newTracks.size());
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

        StateVector xp = track->state->x().head(STATE_DIM);
        xp.head(OBS_DIM) = xp.head(OBS_DIM) - m_forwardPredictTime * track->state->x().head(2*OBS_DIM).tail(OBS_DIM); // not sure why minus sign is needed, but empirically shown to work
        m_geometryUtils.meanAndCovarianceToPoseAndTwist(xp, track->state->C(), trackedPerson.pose, trackedPerson.twist); // state estimate (update)

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


void ROSInterface::publishStatistics(ros::Time currentRosTime, const unsigned int numberTracks)
{
    // Get walltime since start
    ros::WallTime endTime = ros::WallTime::now();

    // Get cpu time since start
    clock_t endClock = clock();
    double wallTimeSinceStart = (endTime - m_startWallTime).toSec();
    double clockTimeSinceStart = ((double) (endClock - m_startClock)) / CLOCKS_PER_SEC;
    double avgLoad = (clockTimeSinceStart/wallTimeSinceStart)*100.0;

    double wallTimeCycle = (endTime - m_wallTimeBefore).toSec();
    double clockTimeCycle = ((double) (endClock - m_clockBefore)) / CLOCKS_PER_SEC;
    double currentLoad = (clockTimeCycle/wallTimeCycle)*100.0;

    // Calculate average
    double averageCycleTime = 0.0;
    foreach(float cycleTime, m_lastCycleTimes) {
        averageCycleTime += cycleTime;
    }
    averageCycleTime /= m_lastCycleTimes.size();

    // Publish average cycle time
    std_msgs::Float32 averageCycleTimeMsg;
    averageCycleTimeMsg.data = averageCycleTime;
    std_msgs::Float32 averageProcessingRateMsg;
    averageProcessingRateMsg.data = 1.0 / averageCycleTime;

    // Wait until buffer is full
    if(m_lastCycleTimes.size() == m_lastCycleTimes.capacity())
    {
        m_averageCycleTimePublisher.publish(averageCycleTimeMsg);

        // Publish average processing time
        m_averageProcessingRatePublisher.publish(averageProcessingRateMsg);
    }

    // Publish average cpu load
    std_msgs::Float32 averageLoadMsg;
    averageLoadMsg.data = avgLoad;
    m_averageLoadPublisher.publish(averageLoadMsg);

    // Publish timing metrics
    spencer_tracking_msgs::TrackingTimingMetrics timingMetrics;
    timingMetrics.header.seq = m_tracker->getCurrentCycleNo();
    timingMetrics.header.frame_id = m_geometryUtils.getWorldFrame();
    timingMetrics.header.stamp = currentRosTime;

    timingMetrics.cycle_no = m_tracker->getCurrentCycleNo();
    timingMetrics.track_count = numberTracks;
    timingMetrics.average_cycle_time = averageCycleTime;
    timingMetrics.average_processing_rate = 1.0 / averageCycleTime;
    timingMetrics.cycle_time = m_lastCycleTimes.back();
    timingMetrics.elapsed_cpu_time = clockTimeSinceStart;
    timingMetrics.elapsed_time = wallTimeSinceStart;
    timingMetrics.cpu_load = currentLoad;
    timingMetrics.average_cpu_load = avgLoad;
    m_timingMetricsPublisher.publish(timingMetrics);

    m_wallTimeBefore = endTime;
    m_clockBefore = endClock;
}

} // end of namespace srl_nnt
