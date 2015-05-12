/* Created on: May 07, 2014. Author: Timm Linder, Fabian Girrbach */
#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/motion_models/constant_motion_model.h>
#include <srl_nearest_neighbor_tracker/motion_models/coordinated_turn_motion_model.h>
#include <srl_nearest_neighbor_tracker/motion_models/brownian_motion_model.h>


namespace srl_nnt
{
EKF::EKF(string parameterPrefix)
: m_initialC(StateMatrix::Zero()), m_defaultQ(StateMatrix::Zero()), m_A(StateMatrix::Zero())
{
    if (Params::get<bool>("use_imm", false))
    {
        ROS_INFO_STREAM("Initializing EKF for IMM with parameter prefix " << parameterPrefix);
    }

    string modelType = Params::get<string>(parameterPrefix+"motion_model", "ConstantVelocity");
    if (modelType.compare("ConstantVelocity") == 0)     m_motionModel.reset(new ConstantMotionModel);
    else if (modelType.compare("CoordinatedTurn") == 0) m_motionModel.reset(new CoordinatedTurnMotionModel);
    else if (modelType.compare("BrownianMotion") == 0)  m_motionModel.reset(new BrownianMotionModel);

    // initial covariance matrix for each track
    m_initialC(0, 0) = Params::get<double>(parameterPrefix+"cosxx", 0.7);   // initial x positional variance (=squared stddev)
    m_initialC(1, 1) = Params::get<double>(parameterPrefix+"cosyy", 0.7);
    m_initialC(2, 2) = Params::get<double>(parameterPrefix+"cosvxx", 2.0);  // initial assumed variance in x velocity
    m_initialC(3, 3) = Params::get<double>(parameterPrefix+"cosvyy", 2.0);
    if (STATE_DIM == 5) m_initialC(4, 4) = Params::get<double>(parameterPrefix+"cosw", 0.0);

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
    StateMatrix Q;
    if (m_useProcessNoise) {
        Q = m_motionModel->getProcessNoiseQ(deltatime, m_processNoiseLevel);
    }
    else {
        Q = m_defaultQ;
    }

    // Apply state transition matrix
    kfs.m_xp = m_A * kfs.m_x;
    kfs.m_Cp = m_A * kfs.m_C * m_A.transpose() + Q;
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
    m_A = m_motionModel->A(x,deltaT);
    ROS_DEBUG_STREAM("EKF Transition Matrix A\n "<< m_A);
}


void EKF::visualizeFilterProperties(const ros::Time& time, const string& trackerFrame, const Tracks tracks){

}


} // end of namespace srl_nnt
