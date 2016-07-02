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
#ifndef _EKF_H
#define _EKF_H

#include <srl_nearest_neighbor_tracker/filter.h>
#include <srl_nearest_neighbor_tracker/data/kalman_filter_state.h>
#include <srl_nearest_neighbor_tracker/motion_models/motion_model.h>


namespace srl_nnt {

class IMMState;
class IMMFilter;

/// Extended Kalman filter for track prediction and update.
/// Assumes the FilterState input argument provided to the functions is of type KalmanFilterState.
class EKF : public Filter
{
public:
    /// Constructor.
    EKF(std::string parameterPrefix="");

    /// Initialize state and covariance of a new track, based upon the given observation.
    void initializeTrackState(FilterState::Ptr state, Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero());

    /// Initialize state and covariance of a new track, based upon the given observation.
    virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero());
    virtual FilterState::Ptr initializeTrackStateFromLogicInitiator(InitiatorCandidate::Ptr candidate);

    /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new observation yet).
    virtual void predictTrackState(FilterState::Ptr state, double deltatime);

    /// Update state and covariance of a track using the provided pairing for that track.
    virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing);

    /// Update state and covariance of track which has no corresponding observation (i.e. just using the prediction).
    /// The default implementation just copies the prediction (see predictTrackState()) into the new state.
    virtual void updateOccludedTrack(FilterState::Ptr state);

    /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is time-dependent.
    virtual void setTransitionMatrix(const StateVector& x,const double deltaT);

    /// Method for displaying specific visualization for filter implementation
    virtual void visualizeFilterProperties(const ros::Time& time, const std::string& trackerFrame, const Tracks tracks);

    friend class IMMState;
    friend class IMMFilter;

    typedef boost::shared_ptr<EKF>Ptr;

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

    MotionModel::Ptr m_motionModel;
};


} // end of namespace srl_nnt


#endif // _EKF_H
