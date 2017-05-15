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

#include <srl_nearest_neighbor_tracker/nearest_neighbor_tracker.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/logic_initiator.h>

#include <srl_nearest_neighbor_tracker/occlusion_handling/basic_occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/laser_shade_occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/polygon_occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_geodesics_manager.h>

#include <srl_nearest_neighbor_tracker/data_association/basic_nearest_neighbor_data_association.h>
#include <srl_nearest_neighbor_tracker/data_association/global_nearest_neighbor_data_association.h>
#include <srl_nearest_neighbor_tracker/data_association/greedy_nearest_neighbor_data_association.h>

#include <srl_nearest_neighbor_tracker/missed_observation_recovery/low_confidence_observations_recovery.h>


#include <ros/ros.h>
#include <Eigen/LU>
#include <limits>
#include <map>



namespace srl_nnt {

NearestNeighborTracker::NearestNeighborTracker(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodehandle)
: m_nodeHandle(nodeHandle),
  m_privateNodeHandle(privateNodehandle),
  m_cycleCounter(0),
  m_cycleTime(0.0),
  m_deltaTime(0),
  m_trackIdCounter(0),
  m_initiator(),
  m_dataAssociation()
{
    m_frameID = Params::get<string>("world_frame", "odom");

    // Get settings for IMM or simple Kalman Filter
    if (Params::get<bool>("use_imm", false)){
        ROS_INFO_STREAM("Using IMM filter for NNT");
        m_filter.reset(new IMMFilter(m_nodeHandle, m_privateNodeHandle));
    }
    else {
        ROS_INFO_STREAM("Using Extendend Kalman filter for NNT");
        m_filter.reset(new EKF);
    }

    // Get setting for occlusion manager
    if (Params::get<bool>("use_laser_shade_occlusion_manager", false)) {
        ROS_INFO("Using laser shade occlusion manager for NNT");
        m_occlusionManager.reset(new LaserShadeOcclusionManager);
    }
    else if (Params::get<bool>("use_polygon_occlusion_manager", false)) {
        ROS_INFO("Using polygon occlusion manager for NNT");
        m_occlusionManager.reset(new PolygonOcclusionManager);
    }
    else if (Params::get<bool>("use_occlusion_geodescis_manager", false)) {
       ROS_INFO("Using occlusion geodesics manager for NNT");
       m_occlusionManager.reset(new OcclusionGeodesicsManager);
    }
    else {
        ROS_INFO("Using basic occlusion manager for NNT");
        m_occlusionManager.reset(new BasicOcclusionManager);
    }
    m_occlusionManager->initializeOcclusionManager(m_nodeHandle, m_privateNodeHandle);
    m_occlusionManager->setFrameIDofTracker(m_frameID);

    // Get setting for occlusion manager
    string dataAssociationStr = Params::get<string>("data_association_type", "greedy_nearest_neighbor");
    if (dataAssociationStr == "basic_nearest_neighbor"){
        m_dataAssociation.reset(new BasicNearestNeighborDataAssociation);
    }
    else if (dataAssociationStr == "global_nearest_neighbor") {
        m_dataAssociation.reset(new GlobalNearestNeighborDataAssociation);
    }
    else if (dataAssociationStr == "greedy_nearest_neighbor") {
        m_dataAssociation.reset(new GreedyNearestNeighborDataAssociation);
    }
    else{
        ROS_FATAL_STREAM("Data association method is invalid: " << dataAssociationStr.c_str());
    }
    ROS_INFO_STREAM("Selected data association method: \t" << dataAssociationStr);
    m_dataAssociation->initializeDataAssociation(m_nodeHandle, m_privateNodeHandle);

    // Check which missed observation recovery mechanisms are active
    if(!Params::get<string>("additional_low_confidence_detections", "").empty()) {
        ROS_INFO("Enabling LowConfidenceObservationsRecovery!");
        m_missedObservationRecoveries.push_back( MissedObservationRecovery::Ptr( new LowConfidenceObservationsRecovery ) );
    }

    // Initialize missed observation recovery mechanisms
    foreach(MissedObservationRecovery::Ptr missedObservationRecovery, m_missedObservationRecoveries) {
        missedObservationRecovery->init(m_nodeHandle, m_privateNodeHandle);
    }
}


const Tracks& NearestNeighborTracker::processCycle(double currentTime, const Observations& newObservations)
{
    ROS_INFO_STREAM("Received " << newObservations.size() << " observations.");

    beginCycle(currentTime);

    predictTrackStates();

    predictMeasurements();

    Tracks occludedTracks = m_occlusionManager->manageOcclusionsBeforeDataAssociation(m_tracks, ros::Time(currentTime), m_frameID);
    ROS_INFO_STREAM("Occlusion manager returned " << occludedTracks.size() << " tracks");
    Pairings pairings = m_dataAssociation->performDataAssociation(m_tracks, newObservations);

    // Extension to default NNT: If there are additional low-confidence observations (e.g. from a high-recall blob detector),
    // try to match these against the tracks which are so far unmatched (but only if we are sufficiently sure that it is a valid track).
    for(size_t i = 0; i < m_missedObservationRecoveries.size(); i++)
    {
        MissedObservationRecovery::Ptr& missedObservationRecovery = m_missedObservationRecoveries[i];

        Tracks unmatchedTracks;
        Tracks matchedTracks;

        foreach(Track::Ptr track, m_tracks)
        {
            if(track->trackStatus == Track::OCCLUDED || track->trackStatus == Track::MISSED)
            {
                if(missedObservationRecovery->isTrackEligibleForRecovery(track))        
                    unmatchedTracks.push_back(track);
            }
            else if (track->trackStatus == Track::MATCHED)
            {
                track->numberOfConsecutiveWeakMatches = 0;
                matchedTracks.push_back(track);
            }
        }

        if(!unmatchedTracks.empty()) {
            Observations recoveredObservations;
            missedObservationRecovery->recoverObservations(currentTime, unmatchedTracks, matchedTracks, recoveredObservations);
            missedObservationRecovery->publishRecoveredObservations(currentTime, recoveredObservations); // for debugging
            
            ROS_INFO_STREAM("Data association  with" << recoveredObservations.size() << " recovered (weak) observations and " << unmatchedTracks.size() << " unmatched tracks");
            Pairings additionalPairings = m_dataAssociation->performDataAssociation(unmatchedTracks, recoveredObservations);
            pairings.insert(pairings.end(), additionalPairings.begin(), additionalPairings.end());

            foreach(Pairing::Ptr pairing, additionalPairings) {
                // Increment counter. The value is checked inside isTrackEligibleForRecovery() to check if further weak associations are allowed.
                pairing->track->numberOfConsecutiveWeakMatches++;
            }
        }
    }

    Pairings reappearedParings = m_occlusionManager->occludedTrackAssociation(occludedTracks, newObservations, ros::Time(currentTime));


    Tracks mergedTracks;
    mergedTracks.reserve( m_tracks.size() + occludedTracks.size() ); // preallocate memory
    mergedTracks.insert( mergedTracks.end(), m_tracks.begin(), m_tracks.end() );
    mergedTracks.insert( mergedTracks.end(), occludedTracks.begin(), occludedTracks.end() );
    m_tracks = mergedTracks;
    ROS_DEBUG_STREAM("After merge " << m_tracks.size() << " tracks");


    Pairings mergedPairings;
    mergedPairings.reserve( pairings.size() + reappearedParings.size() ); // preallocate memory
    mergedPairings.insert( mergedPairings.end(), pairings.begin(), pairings.end() );
    mergedPairings.insert( mergedPairings.end(), reappearedParings.begin(), reappearedParings.end() );


    updateKalmanFilter(mergedPairings);


    bool useInitiationLogic = Params::get<bool>("use_initiation_logic", true);
    if(useInitiationLogic) {
        // Extension to default NNT: Use track initiation logic
        InitiatorCandidates confirmedTrackCandidates = m_initiator.processObservations(newObservations);
        initNewTracksFromCandidates(confirmedTrackCandidates);
    }
    else {
        // Any unmatched observations creates a new track, unless it is too close to an existing track
        // (less than a person radius).
        initNewTracksFromObservations(newObservations);
    }

    deleteObsoleteTracks();

    deleteDuplicateTracks();

    endCycle();

    return m_tracks;
}


void NearestNeighborTracker::beginCycle(double currentTime)
{
    if(currentTime < m_cycleTime) {
        // This usually happens when playing back a bagfile and looping
        ROS_WARN("Jump back in time detected. Deleting all existing tracks. Old time was %.3f, new time is %.3f (dt=%.3f sec)!", m_cycleTime, currentTime, currentTime - m_cycleTime);

        // Reset tracker state
        m_cycleTime = 0.0; m_cycleCounter = 0; m_trackIdCounter = 0;
        m_tracks.clear();
    }
    m_deltaTime = currentTime - m_cycleTime;
    m_cycleTime = currentTime;

    ROS_DEBUG("Beginning tracking cycle no. %lu. Time since last cycle: %.3f sec", m_cycleCounter, m_deltaTime);
}


void NearestNeighborTracker::endCycle()
{
    m_filter->visualizeFilterProperties(ros::Time().fromSec(m_cycleTime),m_frameID, m_tracks);

    // Increase cycle counter
    m_cycleCounter++;

    ROS_DEBUG("End of tracking cycle %lu", m_cycleCounter-1);
}


void NearestNeighborTracker::predictTrackStates()
{
    ROS_DEBUG("Predicting track states");

    foreach(Track::Ptr track, m_tracks) {
        if(track->trackStatus != Track::DELETED) {
            m_filter->setTransitionMatrix(track->state->x(), m_deltaTime);
            track->stateHistory.push_back(track->state->deepCopy()); // copy current state into history for later Debugging & duplicate track elimination
            m_filter->predictTrackState(track->state, m_deltaTime);
        }
    }
}


void NearestNeighborTracker::predictMeasurements()
{
    ROS_DEBUG("Predicting measurements");

    // For the moment, our "track-to-measurement model" is very simple: Just copy x and y position, and forget about the vx, vy (measurements don't have velocities)
    ObsStateMatrix fixed_H = ObsStateMatrix::Zero();
    fixed_H(0,0) = 1.0;
    fixed_H(1,1) = 1.0;

    foreach(Track::Ptr track, m_tracks) {
        track->state->updateMeasurementPrediction(fixed_H);
    }
}


void NearestNeighborTracker::updateKalmanFilter(const Pairings& pairings)
{
    ROS_DEBUG("Updating Extended Kalman filters of all tracks");

    foreach(Pairing::Ptr pairing, pairings) {
        // Only update tracks in validated pairings
        if(pairing->validated) {
            assert(pairing->track->observation);
            m_filter->updateMatchedTrack(pairing->track->state, pairing);
        }
    }
    ROS_DEBUG("Updating occluded tracks");

    foreach(Track::Ptr track, m_tracks) {
        // Update occluded tracks
        if(!track->observation) {
            m_filter->updateOccludedTrack(track->state);
        }
    }
}


bool NearestNeighborTracker::checkForClosebyExistingTrack(const Observation::Ptr& observation) {
    // HACK: Avoid creating new tracks if there is a close-by track that has recently been matched
    bool closebyExistingTrack = false;
    foreach(Track::Ptr track, m_tracks) {
        if(std::max(track->numberOfConsecutiveOcclusions, track->numberOfConsecutiveMisses) < 5) {
            ObsVector diff = track->state->x().head(2) - observation->z;
            if(diff.norm() < 0.25) {
                closebyExistingTrack = true;
                break;
            }
        }
    }
    return closebyExistingTrack;
}


Track::Ptr NearestNeighborTracker::createTrack(const Observation::Ptr& observation)
{
    Track::Ptr newTrack( new Track );
    newTrack->id = m_trackIdCounter++;
    newTrack->trackStatus = Track::NEW;
    newTrack->observation = observation;
    newTrack->createdAt = m_cycleTime;
    newTrack->numberOfTotalMatches = newTrack->numberOfConsecutiveOcclusions = newTrack->numberOfConsecutiveMisses = newTrack->numberOfConsecutiveWeakMatches = 0;
    newTrack->model_idx = 0;
    newTrack->detectionProbability = 1.0;
    newTrack->stateHistory.set_capacity(Params::get<int>("state_history_length", 30)); // for DEBUGging & elimination of duplicate tracks
    return newTrack;
}


void NearestNeighborTracker::initNewTracksFromObservations(const Observations& newObservations)
{
    ROS_DEBUG("Initializing new tracks");

    Tracks newTracks;
    foreach(Observation::Ptr observation, newObservations)
    {
        if(!observation->matched) {
            // Make sure no track exists closeby, in case we have duplicate detections for some reason
            if(checkForClosebyExistingTrack(observation)) continue;

            Track::Ptr newTrack = createTrack(observation);
            newTrack->state = m_filter->initializeTrackState(observation);
            newTracks.push_back(newTrack);
        }
    }

    if(!newTracks.empty()) ROS_DEBUG("%zu new track(s) have been initialized!", newTracks.size());
    appendTo(m_tracks, newTracks);
}


void NearestNeighborTracker::initNewTracksFromCandidates(const InitiatorCandidates candidates)
{
    ROS_DEBUG_STREAM("Initializing " << candidates.size() << " new tracks from initialization logic candidates" << std::endl);

    Tracks newTracks;
    foreach(InitiatorCandidate::Ptr candidate, candidates)
    {
        Observations& observations = candidate->observations;

        // Make sure no track exists closeby, in case we have duplicate detections for some reason
        if(checkForClosebyExistingTrack(observations.back())) continue;

        Track::Ptr newTrack = createTrack(observations.back());
        newTrack->state = m_filter->initializeTrackStateFromLogicInitiator(candidate);
        newTracks.push_back(newTrack);
    }

    if(!newTracks.empty()) ROS_DEBUG("%zu new track(s) have been initialized!", newTracks.size());
    appendTo(m_tracks, newTracks);
}


void NearestNeighborTracker::deleteDuplicateTracks()
{
    ROS_DEBUG("Deleting duplicate tracks");

    const size_t NUM_ENTRIES_TO_COMPARE = Params::get<int>("duplicate_track_num_history_entries_to_compare", 10);
    const size_t NUM_ENTRIES_MUST_MATCH = Params::get<int>("duplicate_track_num_history_entries_must_match", 7);
    const double MAX_DISTANCE = Params::get<double>("duplicate_track_max_dist", 0.15);
    const double MAX_DISTANCE_SQUARED = MAX_DISTANCE * MAX_DISTANCE;

    // Extension: Remove duplicate tracks
    // Step 1: Find duplicate tracks and mark younger duplicate for deletion
    std::vector<size_t> tracksToDelete;
    for(size_t t1 = 0; t1 < m_tracks.size(); t1++) {
        Track::Ptr& t1_ptr = m_tracks[t1];
        int requiredStateHistoryLength = std::min(t1_ptr->stateHistory.capacity(), NUM_ENTRIES_TO_COMPARE);
        if(t1_ptr->stateHistory.size() < requiredStateHistoryLength) continue; // not enough history available yet to compare tracks
        for(size_t t2 = t1+1; t2 < m_tracks.size(); t2++) {
            Track::Ptr& t2_ptr = m_tracks[t2];
            if(t2_ptr->stateHistory.size() < requiredStateHistoryLength) continue; // not enough history available yet to compare tracks

            size_t numMatches = 0;
            for(size_t historyIndex = 0; historyIndex < requiredStateHistoryLength; historyIndex++) {
                //if (historyIndex - numMatches > requiredStateHistoryLength - NUM_ENTRIES_MUST_MATCH) break;
                FilterState::Ptr& s1 = t1_ptr->stateHistory[ t1_ptr->stateHistory.size() - historyIndex - 1]; // get n-latest element
                FilterState::Ptr& s2 = t2_ptr->stateHistory[ t2_ptr->stateHistory.size() - historyIndex - 1];

                ObsVector diff = s1->x().head(2) - s2->x().head(2);
                if(diff.squaredNorm() < MAX_DISTANCE_SQUARED) numMatches++;
            }

            if(numMatches >= NUM_ENTRIES_MUST_MATCH) {
                // Delete the younger / lower-quality track
                if(t1_ptr->numberOfTotalMatches < t2_ptr->numberOfTotalMatches) {
                    tracksToDelete.push_back(t1);
                }
                else {
                    tracksToDelete.push_back(t2);
                }
            }
        }
    }

    ROS_INFO_STREAM("Deleting " << tracksToDelete.size() << " duplicated tracks.");

    if (tracksToDelete.size() > 0)
    {
        std::sort(tracksToDelete.begin(), tracksToDelete.end());

        // Delete marked tracks
        int trackIdx =-1;
        for(int i = tracksToDelete.size()-1; i >= 0; i--) {
            if (tracksToDelete.at(i) == trackIdx) continue; //Could happen that we want to delete same track more often
            trackIdx = tracksToDelete.at(i);
            m_tracks.erase(m_tracks.begin() + trackIdx);
        }
    }
}


void NearestNeighborTracker::deleteObsoleteTracks()
{
    m_occlusionManager->deleteOccludedTracks(m_tracks, ros::Time(m_cycleTime));
}


} // end of namespace srl_nnt
