#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>


namespace srl_nnt
{
    EKF::EKF()
        : m_initialC(StateMatrix::Zero()), m_defaultQ(StateMatrix::Zero()), m_A(StateMatrix::Zero())
    {
        // initial covariance matrix for each track
        m_initialC(0, 0) = Params::get<double>("cosxx", 0.7);   // initial x positional variance (=squared stddev)
        m_initialC(1, 1) = Params::get<double>("cosyy", 0.7);   
        m_initialC(2, 2) = Params::get<double>("cosvxx", 2.0);  // initial assumed variance in x velocity
        m_initialC(3, 3) = Params::get<double>("cosvyy", 2.0);

        // noise
        m_useProcessNoise = Params::get<bool>("use_process_noise", true);

        if (m_useProcessNoise) {
            // see prediction
            m_processNoiseLevel = Params::get<double>("process_noise_level", 0.1);
        } 
        else {
            m_defaultQ(0, 0) = Params::get<double>("qsxx", 0.0081);
            m_defaultQ(1, 1) = Params::get<double>("qsyy", 0.0081);
            m_defaultQ(2, 2) = Params::get<double>("qsvxx", 0.0064);
            m_defaultQ(3, 3) = Params::get<double>("qsvyy", 0.0064);
        }
    }


    FilterState::Ptr EKF::initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity)
    {
        KalmanFilterState::Ptr kfs(new KalmanFilterState);
        
        kfs->m_x = StateVector::Zero(); // unnecessary, but just to be safe
        
        kfs->m_x.head(OBS_DIM) = observation->z;
        kfs->m_x.tail(STATE_DIM - OBS_DIM) = initialVelocity;
        
        kfs->m_C = m_initialC;

        kfs->m_xp = kfs->m_x;
        kfs->m_Cp = kfs->m_C;

        return kfs;
    }


    void EKF::predictTrackState(FilterState::Ptr state, double deltatime)
    {
        KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);

        // Noise
        StateMatrix Q;
        if (m_useProcessNoise) {
            Q = StateMatrix::Zero();
            Q(0, 0) = (deltatime * deltatime * deltatime) * m_processNoiseLevel / 3.0;
            Q(1, 1) = Q(0, 0);
            Q(0, 2) = 0.5 * (deltatime * deltatime) * m_processNoiseLevel;
            Q(1, 3) = Q(0, 2);
            Q(2, 0) = Q(0, 2);
            Q(3, 1) = Q(0, 2);
            Q(2, 2) = deltatime * m_processNoiseLevel;
            Q(3, 3) = Q(2, 2);
        }
        else {
            Q = m_defaultQ;
        }

        // Apply state transition matrix
        kfs.m_xp = m_A * kfs.m_x;
        kfs.m_Cp = m_A * kfs.m_C * m_A.transpose() + Q;
    }


    void EKF::updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing)
    {
        KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);

        // Update step of the Kalman filter
        StateObsMatrix K = kfs.m_Cp * kfs.m_H.transpose() * pairing->Sinv;
        kfs.m_x = kfs.m_xp + K * pairing->v;
        kfs.m_C = kfs.m_Cp - K * kfs.m_H * kfs.m_Cp;
    }


    void EKF::updateOccludedTrack(FilterState::Ptr state)
    {
        KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);
        kfs.m_x = kfs.m_xp;
        kfs.m_C = kfs.m_Cp;
    }


    void EKF::setTransitionMatrix(const StateMatrix& A) {
        m_A = A;
    }

} // end of namespace srl_nnt
