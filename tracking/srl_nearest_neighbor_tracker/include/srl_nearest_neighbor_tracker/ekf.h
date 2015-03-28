#ifndef _EKF_H
#define _EKF_H

#include <srl_nearest_neighbor_tracker/filter.h>
#include <srl_nearest_neighbor_tracker/data/kalman_filter_state.h>


namespace srl_nnt {

/// Extended Kalman filter for track prediction and update.
/// Assumes the FilterState input argument provided to the functions is of type KalmanFilterState.
class EKF : public Filter
{
public:
    /// Constructor.
    EKF();

    /// Initialize state and covariance of a new track, based upon the given observation.
    virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero());

    /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new observation yet).
    virtual void predictTrackState(FilterState::Ptr state, double deltatime);

    /// Update state and covariance of a track using the provided pairing for that track.
    virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing);

    /// Update state and covariance of track which has no corresponding observation (i.e. just using the prediction).
    /// The default implementation just copies the prediction (see predictTrackState()) into the new state.
    virtual void updateOccludedTrack(FilterState::Ptr state);

    /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is time-dependent.
    virtual void setTransitionMatrix(const StateMatrix& A);


private:
    /// The state transition matrix A (also sometimes called F)
    StateMatrix m_A;

    /// Initial state covariance at track creation
    StateMatrix m_initialC;

    /// Default additive noise (only used if not using process noise)
    StateMatrix m_defaultQ;

    /// Use process noise? Otherwise use m_defaultQ
    bool m_useProcessNoise;
    double m_processNoiseLevel;
};


} // end of namespace srl_nnt


#endif // _EKF_H
