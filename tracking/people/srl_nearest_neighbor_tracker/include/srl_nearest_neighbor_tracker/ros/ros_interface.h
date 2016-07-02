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
    /// Callback that is invoked when new detections arrive via ROS.
    void incomingObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr detectedPersons);

    /// Publishes tracks on ROS after completion of a tracking cycle.
    void publishTracks(ros::Time currentRosTime, const Tracks& tracks);

    // Publishes statistics, such as average processing cycle duration and processing rate.
    void publishStatistics(ros::Time currentRosTime, const unsigned int numberTracks);

    /// ROS handles for publisher and subscriber management
    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;
    ros::Subscriber m_detectedPersonsSubscriber;

    spencer_diagnostics::MonitoredPublisher m_trackedPersonsPublisher;
    ros::Publisher m_averageProcessingRatePublisher, m_averageCycleTimePublisher, m_trackCountPublisher, m_averageLoadPublisher, m_timingMetricsPublisher;
    
    /// Tracker instance
    Tracker* m_tracker;

    /// Tracker parameters
    Params m_params;

    /// Utility classes
    GeometryUtils m_geometryUtils;
    boost::circular_buffer<double> m_lastCycleTimes;
    clock_t m_startClock;
    clock_t m_clockBefore;
    ros::WallTime m_startWallTime;
    ros::WallTime m_wallTimeBefore;
    bool m_timingInitialized;
    bool m_overwriteMeasurementNoise;
    float m_forwardPredictTime;
};


} // end of namespace srl_nnt


#endif // _ROSINTERFACE_H
