#ifndef _LOGIC_INITIATOR_H
#define _LOGIC_INITIATOR_H

#include <srl_nearest_neighbor_tracker/base/tracker.h>
#include <srl_nearest_neighbor_tracker/ekf.h>


#define SYSTEMATIC_SCAN_ERROR 0.07

namespace srl_nnt {

struct InitiatorCandidate{

    // vector of observations assigned to the candidate
    Observations observations;

    double meanAngle;

    // current state of kalman filter
    KalmanFilterState::Ptr state;

    // extrapolation is stored as observation
    Observation::Ptr extrapolation;

    // Missed observations
    unsigned int missedObs;

    typedef boost::shared_ptr<InitiatorCandidate> Ptr;
    typedef boost::shared_ptr<const InitiatorCandidate> ConstPtr;
};

typedef std::vector< InitiatorCandidate::Ptr > InitiatorCandidates;

enum {
    MAHALANOBIS,
    EUCLIDEAN
};


class LogicInitiator
{
public:
    /// Constructor.
    LogicInitiator(const double velMin, const double velMax, const unsigned int numberScans);
    InitiatorCandidates processObservations(const Observations observations);


private:
    const double m_velMax;
    const double m_velMin;
    const unsigned int m_numberScans;
    const double m_maxAngleVariance;
    double m_lastObservationTime;
    double m_maxMahaDistance;
    bool doVelocityGating(const Observation::Ptr obs1,const  Observation::Ptr obs2,const double deltaTime);
    bool doAngleCheck(const Observation::Ptr obs1,const Observation::Ptr obs2,double &meanAngle);

    Observation::Ptr linearExtrapolation(const Observation::Ptr ob1,const Observation::Ptr ob2,const double timeDiffFrames);

    void predictKalman(KalmanFilterState::Ptr state, double deltatime);
    bool checkEuclideanDist(const Observation::Ptr obs1, const Observation::Ptr obs2,const double deltaTime);
    bool checkMahalonobisDistance(const Observation::Ptr obs1, const Observation::Ptr obs2,const double deltaTime);
    bool checkMahalonobisDistance(KalmanFilterState::Ptr state, const Observation::Ptr obs, double deltaTime);
    bool updateKalman(KalmanFilterState::Ptr state, Observation::ConstPtr observation);
    bool compareWithCandidate(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation);
    bool compareWithExtrapolation(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation);
    bool checkMahalonobisDistance(const Observation::Ptr obs1, const Observation::Ptr obs2,
                                  const double vMinX, const double vMaxX,
                                  const double vMinY, const double vMaxY ,const double deltaTime);

    bool simpleMahalanobisCheck(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation);


    const int m_distMethod;
    const bool m_useKalman;
    const bool m_incrementalCheck;
    const int m_maxMissedObs;
    const double m_velocityVariance;


    InitiatorCandidates m_candidates;
    EKF m_kalmanFilter;

};


} // end of namespace srl_nnt


#endif // _LOGIC_INITIATOR_H
