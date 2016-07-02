/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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
#ifndef _NEAREST_NEIGHBOR_TRACKER_H
#define _NEAREST_NEIGHBOR_TRACKER_H


#include <srl_nearest_neighbor_tracker/base/tracker.h>
#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/logic_initiator.h>
#include <srl_nearest_neighbor_tracker/imm_filter.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/data_association/data_association_interface.h>
#include <srl_nearest_neighbor_tracker/missed_observation_recovery/missed_observation_recovery.h>



namespace srl_nnt {

/// A nearest-neighbor standard filter (NNSF) tracker. Processes observations (usually at a fixed interval, depending on sensor refresh rate),
/// and outputs a new set of tracked objects.
class NearestNeighborTracker : public Tracker
{
public:
    /// Constructor.
    NearestNeighborTracker(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodehandle);

    /// Process a single tracking time-step using the new set of observations. Returns the currently tracked targets.
    virtual const Tracks& processCycle(double currentTime, const Observations& newObservations);

    /// Returns the number of the current tracking cycle, starts at 0 for the first set of observations
    virtual unsigned long int getCurrentCycleNo() { return m_cycleCounter; }


private:
    /// Resets all necessary variables to start a new tracking cycle.
    void beginCycle(double currentTime);

    /// Predict track movement based upon a motion model (e.g. constant velocity), not yet taking any new observations into account.
    void predictTrackStates();

    /// After predicting new track states, predict how a measurement corresponding to that state would look like.
    void predictMeasurements();

    /// Update the Extended Kalman filter for each track using the matched observation, if any, or just copy the predicted state into the posterior.
    void updateKalmanFilter(const Pairings& pairings);

    // Avoid creating duplicate tracks if there is a close-by track that has recently been matched
    bool checkForClosebyExistingTrack(const Observation::Ptr& observation);

    /// Creates a new track from the given observation.
    Track::Ptr createTrack(const Observation::Ptr& observation);

    /// Create new tracks from unmatched observations.
    void initNewTracksFromObservations(const Observations& newObservations);

    /// Create new tracks from the candidates of the track initiation logic.
    void initNewTracksFromCandidates(const InitiatorCandidates candidates);

    /// Delete tracks which have not seen any matching observation in a while.
    void deleteObsoleteTracks();

    /// Delete duplicate tracks
    void deleteDuplicateTracks();

    /// Increment cycle number and finish the current tracking cycle.
    void endCycle();


    /// Node handles
    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;

    /// FrameID where tracker operates
    string m_frameID;

    /// The tracks that are currently being maintained/tracked.
    Tracks m_tracks;

    /// The extended Kalman filter used for track prediction and update
    Filter::Ptr m_filter;

    /// Global track ID counter
    track_id m_trackIdCounter;

    /// The current cycle which we are in, starts at 0 with first set of observations
    unsigned long int m_cycleCounter;

    /// Time when the current tracking cycle started, in seconds; and seconds passed since last cycle started
    double m_cycleTime, m_deltaTime;

    /// The track initialization logic.
    LogicInitiator m_initiator;

    // Occlusion handling and track deletion logic.
    OcclusionManager::Ptr m_occlusionManager;

    // The currently used data association.
    DataAssociationInterface::Ptr m_dataAssociation;

    // Recovery mechanisms when a track has no matching observation.
    std::vector<MissedObservationRecovery::Ptr> m_missedObservationRecoveries;
};


} // end of namespace srl_nnt


#endif // _NEAREST_NEIGHBOR_TRACKER_H
