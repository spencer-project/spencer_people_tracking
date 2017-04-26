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

#include <srl_nearest_neighbor_tracker/missed_observation_recovery/low_confidence_observations_recovery.h>

#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/ros/geometry_utils.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


namespace srl_nnt {

void LowConfidenceObservationsRecovery::init(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
{
    // Call super method to save node handles
    MissedObservationRecovery::init(nodeHandle, privateNodeHandle);

    // Init variables
    m_syncWarningShown = false;
    m_numCyclesDroppedSoFar = 0;
    m_numCyclesTotal = 0;

    std::string additionalLowConfidenceDetectionsTopic = srl_nnt::Params::get<std::string>("additional_low_confidence_detections", "");
    if(additionalLowConfidenceDetectionsTopic.empty()) {
        ROS_WARN("Additional low-confidence detections topic is empty! LowConfidenceObservationsRecovery will be inactive.");
        return;
    }
    else {
        ROS_INFO_STREAM("Subscribing to additional low-confidence detections on topic " << additionalLowConfidenceDetectionsTopic);
    }

    // Create subscriber
    m_additionalLowConfidenceDetectionsSubscriber = nodeHandle.subscribe<spencer_tracking_msgs::DetectedPersons>(
        additionalLowConfidenceDetectionsTopic,
        (unsigned) srl_nnt::Params::get<int>("queue_size", 5),
        boost::bind(&LowConfidenceObservationsRecovery::onNewLowConfidenceDetectionsReceived, this, _1 ));
}


void LowConfidenceObservationsRecovery::onNewLowConfidenceDetectionsReceived(const spencer_tracking_msgs::DetectedPersons::ConstPtr detectedPersons)
{
    m_currentLowConfidenceDetections = detectedPersons;
}


void LowConfidenceObservationsRecovery::recoverObservations(const double currentTime, const Tracks& unmatchedTracks, const Tracks& matchedTracks, Observations& recoveredObservations)
{
    // Check if normal and low-confidence detections are sufficiently synchronized
    m_numCyclesTotal++;
    
    if (!m_currentLowConfidenceDetections) return;
    double timestampDelta = currentTime - m_currentLowConfidenceDetections->header.stamp.toSec();
    if(std::abs(timestampDelta) > m_maxTimestampDifference)
    {
        m_numCyclesDroppedSoFar++;
        float dropRatio = m_numCyclesDroppedSoFar / (float)m_numCyclesTotal;
        if(!m_syncWarningShown && dropRatio > 0.1) {
            ROS_WARN_THROTTLE(10.0, "Dropping low-confidence observations because detection timestamps are not sufficiently synchronized! "
                "Current delta: %.1f ms, %d cycles dropped so far (%.1f%%)!", 1000.0*timestampDelta, m_numCyclesDroppedSoFar, 100.0*dropRatio);
            m_syncWarningShown = true;
        }  
        return; // return in any case, since timestamps are out of sync
    }
    else {
        m_syncWarningShown = false;
    }

    // Generate observations from ROS message
    Observations currentLowConfidenceObservations;
    GeometryUtils::getInstance().convertDetectedPersonsToObservations(m_currentLowConfidenceDetections, currentLowConfidenceObservations);

    // Check if observations are not too close to any existing, matched tracks
    const double MAX_GATING_DISTANCE = Params::get<double>("min_distance_for_low_confidence_observations_to_tracks", 0.3);
    const double MAX_GATING_DISTANCE_SQR = MAX_GATING_DISTANCE * MAX_GATING_DISTANCE;

    ObsVector diff;
    foreach(Observation::Ptr ob, currentLowConfidenceObservations)
    {
        bool neighborObservationFound = false;

        foreach(Track::Ptr track, matchedTracks) {
            ROS_ASSERT(track->observation);
            diff = ob->z - track->observation->z;
            if (diff.squaredNorm() < MAX_GATING_DISTANCE_SQR) neighborObservationFound = true;
        }
        if(!neighborObservationFound) recoveredObservations.push_back(ob);
    }
}


} // end of namespace
