/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/motion_models/constant_motion_model.h>
#include <srl_nearest_neighbor_tracker/motion_models/coordinated_turn_motion_model.h>
#include <srl_nearest_neighbor_tracker/motion_models/brownian_motion_model.h>
#include <srl_nearest_neighbor_tracker/motion_models/wiener_process_acceleration_motion_model.h>



namespace srl_nnt
{
EKF::EKF(string parameterPrefix)
: m_initialC(StateMatrix::Zero()), m_defaultQ(StateMatrix::Zero()), m_A(StateMatrix::Zero())
{
    if (Params::get<bool>("use_imm", false))
    {
        ROS_INFO_STREAM("Initializing EKF for IMM with parameter prefix " << parameterPrefix);
    }

    // initial covariance matrix for each track
    m_initialC = StateMatrix::Zero();
    m_initialC(STATE_X_IDX, STATE_X_IDX) = Params::get<double>(parameterPrefix+"cosxx", 0.7);   // initial x positional variance (=squared stddev)
    m_initialC(STATE_Y_IDX, STATE_Y_IDX) = Params::get<double>(parameterPrefix+"cosyy", 0.7);

    string modelType = Params::get<string>(parameterPrefix+"motion_model", "ConstantVelocity");
    if (modelType.compare("ConstantVelocity") == 0){
        m_motionModel.reset(new ConstantMotionModel);
        m_initialC(STATE_VX_IDX, STATE_VX_IDX) = Params::get<double>(parameterPrefix+"cosvxx", 2.0);  // initial assumed variance in x velocity
        m_initialC(STATE_VY_IDX, STATE_VY_IDX) = Params::get<double>(parameterPrefix+"cosvyy", 2.0);
    }
    else if (modelType.compare("CoordinatedTurn") == 0){
        m_motionModel.reset(new CoordinatedTurnMotionModel);
        m_initialC(STATE_VX_IDX, STATE_VX_IDX) = Params::get<double>(parameterPrefix+"cosvxx", 2.0);  // initial assumed variance in x velocity
        m_initialC(STATE_VY_IDX, STATE_VY_IDX) = Params::get<double>(parameterPrefix+"cosvyy", 2.0);
        m_initialC(STATE_OMEGA_IDX, STATE_OMEGA_IDX) = Params::get<double>(parameterPrefix+"cosw", 0.8);
    }
    else if (modelType.compare("BrownianMotion") == 0){
        m_motionModel.reset(new BrownianMotionModel);
    }
    else if (modelType.compare("WienerAcceleration") == 0){
        m_motionModel.reset(new WienerProcessAccelerationModel);
        m_initialC(STATE_VX_IDX, STATE_VX_IDX) = Params::get<double>(parameterPrefix+"cosvxx", 2.0);  // initial assumed variance in x velocity
        m_initialC(STATE_VY_IDX, STATE_VY_IDX) = Params::get<double>(parameterPrefix+"cosvyy", 2.0);
        m_initialC(STATE_AX_IDX, STATE_AX_IDX) = Params::get<double>(parameterPrefix+"cosaxx", 0.2);  // initial assumed variance in x velocity
        m_initialC(STATE_AY_IDX, STATE_AY_IDX) = Params::get<double>(parameterPrefix+"cosayy", 0.2);
    }
    else
        ROS_FATAL_STREAM("Unknown motion model for EKF filter with name :" << modelType << "\nReview settings!");




    // noise
    m_useProcessNoise = Params::get<bool>(parameterPrefix+"use_process_noise", true);

    if (m_useProcessNoise) {
        // see prediction
        m_processNoiseLevel = Params::get<double>(parameterPrefix+"process_noise_level", 0.1);
    }
    else {
        m_defaultQ(0, 0) = Params::get<double>(parameterPrefix+"qsxx", 0.0081);
        m_defaultQ(1, 1) = Params::get<double>(parameterPrefix+"qsyy", 0.0081);
        m_defaultQ(2, 2) = Params::get<double>(parameterPrefix+"qsvxx", 0.0064);
        m_defaultQ(3, 3) = Params::get<double>(parameterPrefix+"qsvyy", 0.0064);
    }
}


void EKF::initializeTrackState(FilterState::Ptr state,Observation::ConstPtr observation, const VelocityVector& initialVelocity )
{
    KalmanFilterState::Ptr kfs = boost::dynamic_pointer_cast<KalmanFilterState>(state);

    kfs->m_x = StateVector::Zero(); // unnecessary, but just to be safe

    kfs->m_x.head(OBS_DIM) = observation->z;
    kfs->m_x.tail(STATE_DIM - OBS_DIM) = initialVelocity;

    kfs->m_C = m_initialC;

    kfs->m_xp = kfs->m_x;
    kfs->m_Cp = kfs->m_C;

    ROS_DEBUG_STREAM("Track state initialized to " << kfs->m_x);

}


FilterState::Ptr EKF::initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity)
{
    KalmanFilterState::Ptr kfs(new KalmanFilterState);
    initializeTrackState(kfs, observation, initialVelocity);
    return kfs;
}


FilterState::Ptr EKF::initializeTrackStateFromLogicInitiator(InitiatorCandidate::Ptr candidate)
{
    return candidate->state;
}


void EKF::predictTrackState(FilterState::Ptr state, double deltatime)
{
    KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);

    // Noise
    MotionModel::MotionModelMatrix Q;
    if (m_useProcessNoise) {
        Q = m_motionModel->getProcessNoiseQ(deltatime, m_processNoiseLevel);
    }
    else {
        Q = m_defaultQ;
    }

    MotionModel::MotionModelVector x, xp;
    MotionModel::MotionModelMatrix A, Cp;
    // Apply state transition matrix
    A = m_motionModel->A(kfs.m_x,deltatime);
    xp = A * m_motionModel->convertToMotionModel(kfs.m_x);
    Cp = A * m_motionModel->convertToMotionModel(kfs.m_C) * A.transpose() + Q;


    kfs.m_xp = m_motionModel->convertToState(xp);
    kfs.m_Cp =  m_motionModel->convertToState(Cp);
    //  ROS_INFO_STREAM("A " << m_A);
    //  ROS_INFO_STREAM("x " << kfs.m_x);
    //  ROS_INFO_STREAM("C " << kfs.m_C);
    //  ROS_INFO_STREAM("Q " << Q);
    //  ROS_INFO_STREAM("xp " << kfs.m_xp);
    //  ROS_INFO_STREAM("Cp " << kfs.m_Cp);

    ROS_DEBUG_STREAM("Predicted track state " << kfs.m_xp);
}


void EKF::updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing)
{
    KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);

    // Update step of the Kalman filter
    StateObsMatrix K = kfs.m_Cp * kfs.m_H.transpose() * pairing->Sinv;
    kfs.m_x = kfs.m_xp + K * pairing->v;
    kfs.m_C = kfs.m_Cp - K * kfs.m_H * kfs.m_Cp;
    //ROS_WARN_STREAM("Turning rate= " << kfs.m_x(4) << "rad/sec ; " <<  kfs.m_x(4) * 57.2957795  << "deg/sec");
}


void EKF::updateOccludedTrack(FilterState::Ptr state)
{
    KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);
    kfs.m_x = kfs.m_xp;
    kfs.m_C = kfs.m_Cp;
}


void EKF::setTransitionMatrix(const StateVector& x, double deltaT) {
    // Unused since use of Motion Model Matrices and Vectors
    // m_A = m_motionModel->A(x,deltaT);
    // ROS_DEBUG_STREAM("EKF Transition Matrix A\n "<< m_A);
}


void EKF::visualizeFilterProperties(const ros::Time& time, const string& trackerFrame, const Tracks tracks){

}


} // end of namespace srl_nnt
