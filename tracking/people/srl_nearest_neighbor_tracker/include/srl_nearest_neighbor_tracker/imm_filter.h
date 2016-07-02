/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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
#ifndef _IMM_FILTER_H_
#define _IMM_FILTER_H_


#include <srl_nearest_neighbor_tracker/filter.h>
#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/data/imm_state.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <spencer_tracking_msgs/ImmDebugInfos.h>


namespace srl_nnt {

class IMMFilter : public Filter
{
public:
    IMMFilter(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle);

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

    /// Method for displaying specific visualization for filter implementation (probability bars for model, most probable model history)
    virtual void visualizeFilterProperties(const ros::Time& time, const string& trackerFrame, const Tracks tracks);


private:

    ///Node handles
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;

    bool m_immVisualization;
    ros::Publisher m_immVisualizationPublisher;

    std::vector<EKF::Ptr> m_kalmanFilters;
    int m_numberModels;

    IMMMatrix m_markovTransitionProbabilities;

    void computeMixingProbabilities(IMMState::Ptr state);
    void doMixing(IMMState::Ptr state);
    void mixPredictions(IMMState::Ptr state);
    void computeMixedMean(IMMState::Ptr state);
    void computeMixedCovariance(IMMState::Ptr state);
    double calcLikelihood(double d, double detS);
    void modeProbabilityUpdate(IMMState::Ptr state);
    void updateStateEstimate(IMMState::Ptr state);
    void updateCurrentHypothesis(IMMState::Ptr state, Track::Ptr track);

    void getDebugInformation(IMMState::Ptr state, Pairing::ConstPtr pairingTracker);
    void sendDebugInformation();


    bool m_sendDebugInformation;
    ros::Publisher m_immDebugPublisher;
    spencer_tracking_msgs::ImmDebugInfos m_debugMessages;
};

}


#endif /* _IMM_FILTER_H_ */
