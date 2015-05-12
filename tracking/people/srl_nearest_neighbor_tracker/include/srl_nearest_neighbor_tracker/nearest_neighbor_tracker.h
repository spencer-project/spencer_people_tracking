/* Created on: May 07, 2014. Author: Timm Linder */
#ifndef _NEAREST_NEIGHBOR_TRACKER_H
#define _NEAREST_NEIGHBOR_TRACKER_H


#include <srl_nearest_neighbor_tracker/base/tracker.h>
#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/logic_initiator.h>
#include <srl_nearest_neighbor_tracker/imm_filter.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_manager.h>



namespace srl_nnt {

/// A nearest-neighbor standard filter (NNSF) tracker. Processes observations (usually at a fixed interval, depending on sensor refresh rate),
/// and outputs a new set of tracked objects.
class NearestNeighborTracker : public Tracker
{
public:
    /// Constructor.
    NearestNeighborTracker(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodehandle);

    /// Process a single tracking time-step using the new set of observations. Returns the currently tracked targets.
    /// The additional low-confidence observations, if any, can be used to match so-far unmatched tracks, but not for the initialization of new tracks.
    virtual const Tracks& processCycle(double currentTime, const Observations& newObservations, const Observations& additionalLowConfidenceObservations);

    /// Returns the number of the current tracking cycle, starts at 0 for the first set of observations
    virtual unsigned long int getCurrentCycleNo() { return m_cycleCounter; }


private:
    /// Resets all necessary variables to start a new tracking cycle.
    void beginCycle(double currentTime);

    /// Predict track movement based upon a motion model (e.g. constant velocity), not yet taking any new observations into account. 
    void predictTrackStates();

    /// After predicting new track states, predict how a measurement corresponding to that state would look like.
    void predictMeasurements();

    /// Associate new observations with the provided tracks, based upon the previously predicted track states. Return pairings of tracks and observations.
    Pairings performDataAssociation(Tracks& tracks, const Observations& newObservations);
 
    /// Update the Extended Kalman filter for each track using the matched observation, if any, or just copy the predicted state into the posterior.
    void updateKalmanFilter(const Pairings& pairings);
   
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
};


} // end of namespace srl_nnt


#endif // _NEAREST_NEIGHBOR_TRACKER_H
