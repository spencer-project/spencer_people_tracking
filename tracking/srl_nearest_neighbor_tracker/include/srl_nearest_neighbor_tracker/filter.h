#ifndef _FILTER_H
#define _FILTER_H

#include <srl_nearest_neighbor_tracker/data/filter_state.h>
#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/pairing.h>

namespace srl_nnt {

/// Generic filter interface (e.g. for a Kalman filter / Particle filter / IMM filter)
class Filter
{
public:
    /// Initialize state and covariance of a new track, based upon the given observation.
    virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero()) = 0;

    /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new observation yet).
    virtual void predictTrackState(FilterState::Ptr state, double deltatime) = 0;

    /// Update state and covariance of a track using the provided pairing for that track.
    virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing) = 0;

    /// Update state and covariance of track which has no corresponding observation (i.e. just using the prediction).
    /// The default implementation just copies the prediction (see predictTrackState()) into the new state.
    virtual void updateOccludedTrack(FilterState::Ptr state) = 0;

    /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is time-dependent.
    virtual void setTransitionMatrix(const StateMatrix& A) = 0;
};


} // end of namespace srl_nnt


#endif // _FILTER_H
