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

#ifndef _MISSED_OBSERVATION_RECOVERY_H
#define _MISSED_OBSERVATION_RECOVERY_H

#include <ros/ros.h>

#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/ros/geometry_utils.h>
#include <spencer_tracking_msgs/DetectedPersons.h>

#include <boost/shared_ptr.hpp>


namespace srl_nnt {

/// Interface for a mechanism that allows to recover missed detections by e.g. actively searching a laser scan for scan points
/// that might belong to a track which could not be matched against a detection (i.e. in MISSED state).
class MissedObservationRecovery {
public:
    // Virtual destructor
    virtual ~MissedObservationRecovery() {}

    /// Tries to recover observations by searching for measurements (e.g. in a laserscan) of the unmatched tracks.
    virtual void recoverObservations(const double currentTime, const Tracks& unmatchedTracks, const Tracks& matchedTracks, Observations& recoveredObservations) = 0;

    /// Initialization method
    virtual void init(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) {
        m_nodeHandle = nodeHandle;
        m_privateNodeHandle = privateNodeHandle;

        m_publisher = m_privateNodeHandle.advertise<spencer_tracking_msgs::DetectedPersons>("missed_observation_recovery/" + getName(), 3);

        m_minNumMatches = Params::get<int>("min_num_matches_to_allow_low_confidence_match", 10);
        m_maxConsecutiveWeakMatches = Params::get<int>("max_consecutive_low_confidence_matches", 60);

        m_maxTimestampDifference = Params::get<double>("max_timestamp_difference_for_missed_observation_recovery", 0.05);
    };

    /// Checks if the given track is eligible for observation recovery. This may e.g. not be the case when the track is too young, or occluded for too long.
    virtual bool isTrackEligibleForRecovery(Track::Ptr& track) {
        if(track->numberOfTotalMatches < m_minNumMatches) return false;
        if(track->numberOfConsecutiveWeakMatches > m_maxConsecutiveWeakMatches) return false;
        return true;
    }

    /// Publishes the recovered observations for debugging purposes.
    virtual void publishRecoveredObservations(const double currentTime, Observations& recoveredObservations) {
        if(m_publisher.getNumSubscribers() == 0) return;
        spencer_tracking_msgs::DetectedPersons detectedPersons;
        GeometryUtils::getInstance().convertObservationsToDetectedPersons(currentTime, recoveredObservations, detectedPersons);
        for(size_t i = 0; i < detectedPersons.detections.size(); i++) detectedPersons.detections[i].modality = "recover";
        m_publisher.publish(detectedPersons);
    }

    /// Returns a name of this recovery mechanism. Must be ROS-topic compatible, i.e. not contain spaces.
    virtual const std::string getName() = 0;

    /// typedefs for easier readability
    typedef boost::shared_ptr<MissedObservationRecovery> Ptr;
    typedef boost::shared_ptr<const MissedObservationRecovery> ConstPtr;

protected:
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;
    ros::Publisher m_publisher;

    int m_minNumMatches, m_maxConsecutiveWeakMatches;
    double m_maxTimestampDifference;
};

}

#endif //_MISSED_OBSERVATION_RECOVERY_H
