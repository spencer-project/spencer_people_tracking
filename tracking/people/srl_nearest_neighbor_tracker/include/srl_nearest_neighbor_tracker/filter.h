/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
#ifndef _FILTER_H
#define _FILTER_H

#include <srl_nearest_neighbor_tracker/data/filter_state.h>
#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/pairing.h>
#include <srl_nearest_neighbor_tracker/data/initiator_candidate.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <ros/ros.h>


namespace srl_nnt {

/// Generic filter interface (e.g. for a Kalman filter / Particle filter / IMM filter)
class Filter
{
public:
    /// Initialize state and covariance of a new track, based upon the given observation.
    virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero()) = 0;

    virtual FilterState::Ptr initializeTrackStateFromLogicInitiator(InitiatorCandidate::Ptr candidate) = 0;


    /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new observation yet).
    virtual void predictTrackState(FilterState::Ptr state, double deltatime) = 0;

    /// Update state and covariance of a track using the provided pairing for that track.
    virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing) = 0;

    /// Update state and covariance of track which has no corresponding observation (i.e. just using the prediction).
    /// The default implementation just copies the prediction (see predictTrackState()) into the new state.
    virtual void updateOccludedTrack(FilterState::Ptr state) = 0;

    /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is time-dependent.
    virtual void setTransitionMatrix(const StateVector& x,const double deltaT) = 0;

    /// Method for displaying specific visualization for filter implementation
    virtual void visualizeFilterProperties(const ros::Time& time, const string& trackerFrame, const Tracks tracks) =0;

    typedef boost::shared_ptr<Filter>Ptr;
    typedef boost::shared_ptr<const Filter>ConstPtr;

};


} // end of namespace srl_nnt


#endif // _FILTER_H
