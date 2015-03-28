#include <srl_nearest_neighbor_tracker/nearest_neighbor_tracker.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/logic_initiator.h>

#include <ros/ros.h>
#include <Eigen/LU>
#include <limits>
#include <map>



namespace srl_nnt {

NearestNeighborTracker::NearestNeighborTracker()
    : m_cycleCounter(0), m_cycleTime(0.0), m_deltaTime(0), m_trackIdCounter(0), m_initiator(0.2 , 2.0, 6)
{
}


const Tracks& NearestNeighborTracker::processCycle(double currentTime, const Observations& newObservations, const Observations& additionalLowConfidenceObservations)
{
    beginCycle(currentTime);

    predictTrackStates();

    predictMeasurements();

    Pairings pairings = performDataAssociation(m_tracks, newObservations);

    // Extension to default NNT: If there are additional low-confidence observations (e.g. from a high-recall blob detector),
    // try to match these against the tracks which are so far unmatched (but only if we are sufficiently sure that it is a valid track).
    if(!additionalLowConfidenceObservations.empty())
    {
        int minNumMatchesToAllowLowConfidenceMatch = Params::get<int>("min_num_matches_to_allow_low_confidence_match", 10);
        Tracks unmatchedTracks;
        foreach(Track::Ptr track, m_tracks) {
            if(track->trackStatus == Track::OCCLUDED && track->numberOfTotalMatches >= minNumMatchesToAllowLowConfidenceMatch) {
                unmatchedTracks.push_back(track);
            }
        }

        if(!unmatchedTracks.empty()) {
            Pairings additionalPairings = performDataAssociation(unmatchedTracks, additionalLowConfidenceObservations);
            pairings.insert(pairings.end(), additionalPairings.begin(), additionalPairings.end());
        }
    }

    updateKalmanFilter(pairings);

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

    ROS_INFO("Beginning tracking cycle no. %lu. Time since last cycle: %.3f sec", m_cycleCounter, m_deltaTime);
}


void NearestNeighborTracker::endCycle()
{
    // Increase cycle counter
    m_cycleCounter++;

    ROS_DEBUG("End of tracking cycle %lu", m_cycleCounter-1);
}


void NearestNeighborTracker::predictTrackStates()
{
    ROS_DEBUG("Predicting track states");

    // Set a constant velocity transition model for the Kalman filter
    StateMatrix A = StateMatrix::Identity();
    A(0, 2) = m_deltaTime;
    A(1, 3) = m_deltaTime;
    m_kalmanFilter.setTransitionMatrix(A);

    foreach(Track::Ptr track, m_tracks) {
        if(track->trackStatus != Track::DELETED) {
            track->stateHistory.push_back(track->state->deepCopy()); // copy current state into history for later debugging & duplicate track elimination
            m_kalmanFilter.predictTrackState(track->state, m_deltaTime);
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


Pairings NearestNeighborTracker::performDataAssociation(Tracks& tracks, const Observations& newObservations)
{
    ROS_DEBUG("Performing data association");

    //
    // Step 0: Make sure observations are in a good state
    //

    foreach(Observation::Ptr observation, newObservations) {
        observation->matched = false;
    }
    
    Pairings compatiblePairings;


    //
    // Step 1: go through all possible associations and store compatible ones
    //

    const double MATRIX_LN_EPS = -1e8; 
    const double MAX_GATING_DISTANCE = Params::get<double>("max_gating_distance", 1.0);

    typedef multimap<track_id, Pairing::Ptr> TrackSpecificPairings;
    TrackSpecificPairings trackSpecificPairings;

    foreach(Track::Ptr track, tracks)
    {
        foreach(Observation::Ptr observation, newObservations)
        {
            // Create a new pairing
            Pairing::Ptr pairing( new Pairing );

            pairing->track = track;
            pairing->observation = observation;
            pairing->validated = false;

            // Calculate innovation v and inverse of innovation covariance S
            pairing->v = observation->z - track->state->zp();
            
            ObsMatrix S = track->state->H() * track->state->Cp() * track->state->H().transpose() + observation->R;
            Eigen::FullPivLU<ObsMatrix> lu(S);
            double ln_det_S = log(lu.determinant());

            // Calculate inverse of innovation covariance if possible
            if (ln_det_S > MATRIX_LN_EPS) {
                pairing->Sinv = lu.inverse();
                pairing->d = (pairing->v.transpose() * pairing->Sinv * pairing->v)(0,0);
                pairing->singular = pairing->d < 0.0;
            }
            else {
                pairing->Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, numeric_limits<double>::quiet_NaN());
                pairing->d = numeric_limits<double>::quiet_NaN();
                pairing->singular = true;

                ROS_WARN_STREAM("Singular pairing encountered!\nTrack " << track->id << " measurement prediction:\n" << track->state->zp() << "\nTrack covariance prediction:\n" << track->state->Cp() 
                                 << "\nObservation " << observation->id << " mean:\n" << observation->z << "\nObservation covariance:\n" << observation->R );
            }
            
            // Perform gating
            if(!pairing->singular && pairing->d < CHI2INV_99[OBS_DIM] && pairing->v.norm() < MAX_GATING_DISTANCE) {
                // Store in list of compatible pairings
                compatiblePairings.push_back(pairing);
                trackSpecificPairings.insert( make_pair(track->id, pairing) );
            }
        }
    }

    ROS_DEBUG("%zu compatible pairings have been found for %zu existing tracks and %zu new observations!", compatiblePairings.size(), tracks.size(), newObservations.size() );



    //
    // Step 2: go through all tracks, choose and mark nearest neighbour
    //

    foreach(Track::Ptr track, tracks)
    {
        // Store best pairing for this track
        double best_d = numeric_limits<double>::max();
        Pairing::Ptr bestPairing;

        // Iterate over all pairings for this track
        pair<TrackSpecificPairings::iterator, TrackSpecificPairings::iterator> pairingsForThisTrack = trackSpecificPairings.equal_range(track->id);
        for(TrackSpecificPairings::iterator it = pairingsForThisTrack.first; it != pairingsForThisTrack.second; it++) {
            Pairing::Ptr pairing = it->second;

            // Check if the observation hasn't been matched with a different track already, and if the Mahalanobis distance
            // is better than what we have seen so far. 
            if(!pairing->observation->matched && pairing->d < best_d) {
                best_d = pairing->d;
                bestPairing = pairing;
            }
        }

        // See if we found a compatible pairing for this track.
        if(bestPairing) {
            // Track has been matched
            bestPairing->validated = true;
            bestPairing->observation->matched = true;
            track->observation = bestPairing->observation;
            track->trackStatus = Track::MATCHED;
            track->numberOfTotalMatches++;
            track->numberOfConsecutiveOcclusions = 0;
        }
        else {
            // Track is occluded
            track->observation.reset();
            track->trackStatus = Track::OCCLUDED;
            track->numberOfConsecutiveOcclusions++;
        }
    }

    return compatiblePairings;
}


void NearestNeighborTracker::updateKalmanFilter(const Pairings& pairings)
{
    ROS_DEBUG("Updating Extended Kalman filters of all tracks");

    foreach(Pairing::Ptr pairing, pairings) {
        // Only update tracks in validated pairings
        if(pairing->validated) {
            assert(pairing->track->observation);
            m_kalmanFilter.updateMatchedTrack(pairing->track->state, pairing);
        }
    }

    foreach(Track::Ptr track, m_tracks) {
        // Update occluded tracks
        if(!track->observation) {
            m_kalmanFilter.updateOccludedTrack(track->state);
        }
    }
}


void NearestNeighborTracker::initNewTracksFromObservations(const Observations& newObservations)
{
    ROS_DEBUG("Initializing new tracks");

    Tracks newTracks;
    foreach(Observation::Ptr observation, newObservations)
    {
        if(!observation->matched) {
            // HACK: Avoid creating duplicate tracks if there is a close-by track
            // that has recently been matched
            bool closebyExistingTrack = false;
            foreach(Track::Ptr track, m_tracks) {
            if(track->numberOfConsecutiveOcclusions < 5) {
                    ObsVector diff = track->state->x().head(2) - observation->z;
                    if(diff.norm() < 0.25) {
                        closebyExistingTrack = true;
                        break;
                    }
                }
            }
            if(closebyExistingTrack) continue;

            Track::Ptr newTrack( new Track );
            
            newTrack->id = m_trackIdCounter++;
            newTrack->trackStatus = Track::NEW;
            newTrack->observation = observation;
            newTrack->createdAt = m_cycleTime;
            newTrack->numberOfTotalMatches = newTrack->numberOfConsecutiveOcclusions = 0;

            newTrack->state = m_kalmanFilter.initializeTrackState(observation);
            newTrack->stateHistory.set_capacity(Params::get<int>("state_history_length", 30)); // for debugging & elimination of duplicate tracks

            newTracks.push_back(newTrack);            
        }
    }

    if(!newTracks.empty()) ROS_INFO("%zu new track(s) have been initialized!", newTracks.size());
    appendTo(m_tracks, newTracks);
}


void NearestNeighborTracker::initNewTracksFromCandidates(const InitiatorCandidates candidates)
{
    ROS_DEBUG_STREAM("Initializing " << candidates.size() << " new tracks from initialization logic candidates" << std::endl);

    Tracks newTracks;
    foreach(InitiatorCandidate::Ptr candidate, candidates)
      {
        Observations observations = candidate->observations;
        Track::Ptr newTrack( new Track );

        newTrack->id = m_trackIdCounter++;
        newTrack->trackStatus = Track::NEW;
        newTrack->observation = observations.back();
        newTrack->createdAt = m_cycleTime;
        newTrack->numberOfTotalMatches = newTrack->numberOfConsecutiveOcclusions = 0;

        newTrack->state = candidate->state;
        newTrack->stateHistory.set_capacity(Params::get<int>("state_history_length", 30)); // for debugging & elimination of duplicate tracks

        newTracks.push_back(newTrack);
      }

    if(!newTracks.empty()) ROS_INFO("%zu new track(s) have been initialized!", newTracks.size());
    appendTo(m_tracks, newTracks);
}


void NearestNeighborTracker::deleteDuplicateTracks()
{
    ROS_DEBUG("Deleting duplicate tracks");

    const size_t NUM_ENTRIES_TO_COMPARE = Params::get<int>("duplicate_track_num_history_entries_to_compare", 10);
    const size_t NUM_ENTRIES_MUST_MATCH = Params::get<int>("duplicate_track_num_history_entries_must_match", 7);
    const double MAX_DISTANCE = Params::get<double>("duplicate_track_max_dist", 0.15);

    // Extension: Remove duplicate tracks
    // Step 1: Find duplicate tracks and mark younger duplicate for deletion
    Tracks tracksToDelete;
    for(size_t t1 = 0; t1 < m_tracks.size(); t1++) {
        for(size_t t2 = t1 + 1; t2 < m_tracks.size(); t2++) {
            int requiredStateHistoryLength = std::min(m_tracks[t1]->stateHistory.capacity(), NUM_ENTRIES_TO_COMPARE);
            if(m_tracks[t1]->stateHistory.size() < requiredStateHistoryLength || m_tracks[t2]->stateHistory.size() < requiredStateHistoryLength) continue; // not enough history available yet to compare tracks

            size_t numMatches = 0;
            for(size_t historyIndex = 0; historyIndex < requiredStateHistoryLength; historyIndex++) {
                FilterState::Ptr s1 = m_tracks[t1]->stateHistory[ m_tracks[t1]->stateHistory.size() - historyIndex - 1]; // get n-latest element
                FilterState::Ptr s2 = m_tracks[t2]->stateHistory[ m_tracks[t2]->stateHistory.size() - historyIndex - 1];
                
                ObsVector diff = s1->x().head(2) - s2->x().head(2);
                if(diff.norm() < MAX_DISTANCE) numMatches++;
            }

            if(numMatches >= NUM_ENTRIES_MUST_MATCH) {
                // Delete the younger / lower-quality track
                if(m_tracks[t1]->numberOfTotalMatches < m_tracks[t2]->numberOfTotalMatches) {
                    tracksToDelete.push_back(m_tracks[t1]);
                }
                else {
                    tracksToDelete.push_back(m_tracks[t2]);
                }
            }            
        }
    }

    // Delete marked tracks
    while(!tracksToDelete.empty())
    {
        // Find track to delete in m_tracks
        Tracks::iterator trackToDelete = std::find(m_tracks.begin(), m_tracks.end(), tracksToDelete.back());
        if(trackToDelete != m_tracks.end()) {
            // Delete track
            ROS_INFO_STREAM("Deleting duplicate track " << (*trackToDelete)->id);
            m_tracks.erase(trackToDelete);
        }

        // Mark track as deleted
        tracksToDelete.pop_back();
    }
}


void NearestNeighborTracker::deleteObsoleteTracks()
{
    ROS_DEBUG("Deleting obsolete tracks");
    
    const int MAX_OCCLUSIONS_BEFORE_DELETION = Params::get<int>("max_occlusions_before_deletion", 20);
    const int MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK = Params::get<int>("max_occlusions_before_deletion_of_mature_track", 120);
    const int TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES = Params::get<int>("track_is_mature_after_total_num_matches", 100);

    size_t numDeletedTracks = 0;
    bool foundTrackToDelete;
    
    do {
        foundTrackToDelete = false;
        for(Tracks::iterator trackIt = m_tracks.begin(); trackIt != m_tracks.end(); ++trackIt) {
            Track::Ptr track = *trackIt;
            
            // Check if the track is considered as "mature", i.e. it has been there for a long time already.
            const bool trackIsMature = track->numberOfTotalMatches >= TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES;

            // Check if the track hasn't been seen for too long.
            const int occlusionFrameLimit = trackIsMature ? MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK : MAX_OCCLUSIONS_BEFORE_DELETION;
            if(track->numberOfConsecutiveOcclusions > occlusionFrameLimit) {
                m_tracks.erase(trackIt);
                foundTrackToDelete = true;
                numDeletedTracks++;
                break; // need to exit for loop because iterator has become invalid
            }
        }
    }
    while(foundTrackToDelete);

    if(numDeletedTracks) ROS_INFO("%zu track(s) have been deleted!", numDeletedTracks);
}


} // end of namespace srl_nnt
