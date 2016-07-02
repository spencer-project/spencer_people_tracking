/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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
#ifndef _LOGIC_INITIATOR_H
#define _LOGIC_INITIATOR_H

#include <srl_nearest_neighbor_tracker/base/tracker.h>
#include <srl_nearest_neighbor_tracker/ekf.h>
#include <srl_nearest_neighbor_tracker/data/initiator_candidate.h>

#include <set>


namespace srl_nnt {


enum {
    MAHALANOBIS,
    EUCLIDEAN
};


class LogicInitiator
{
public:
    /// Constructor.
    LogicInitiator();
    InitiatorCandidates processObservations(const Observations observations);



private:
    const double m_velMax, m_velMaxHighConfidence;
    const double m_velMin, m_velMinHighConfidence;
    std::set<std::string> m_highConfidenceModalities;
    const unsigned int m_numberScans;
    const double m_maxAngleVariance;
    double m_lastObservationTime;
    double m_maxMahaDistance;

    void getApprovedVelocityLimits(const Observation::Ptr obs, double& velMin, double& velMax);
    bool doVelocityGating(const Observation::Ptr obs1,const  Observation::Ptr obs2,const double deltaTime);
    bool doAngleCheck(const Observation::Ptr obs1,const Observation::Ptr obs2,double &meanAngle);

    Observation::Ptr linearExtrapolation(const Observation::Ptr ob1,const Observation::Ptr ob2,const double timeDiffFrames);

    void predictKalman(KalmanFilterState::Ptr state, double deltatime);
    bool checkEuclideanDist(const Observation::Ptr obs1, const Observation::Ptr obs2,const double deltaTime);

    /*
    bool checkMahalonobisDistance(const Observation::Ptr obs1, const Observation::Ptr obs2,const double deltaTime);
    bool checkMahalonobisDistance(KalmanFilterState::Ptr state, const Observation::Ptr obs, double deltaTime);
    */

    bool updateKalman(KalmanFilterState::Ptr state, Observation::ConstPtr observation);
    bool compareWithCandidate(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation);
    bool compareWithExtrapolation(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation);
    bool checkMahalonobisDistance(const Observation::Ptr obs1, const Observation::Ptr obs2,
                                  const double vMinX, const double vMaxX,
                                  const double vMinY, const double vMaxY,
                                  const double deltaTime);

    bool simpleMahalanobisCheck(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation);


    const int m_distMethod;
    const bool m_incrementalCheck;
    const unsigned int m_maxMissedObs;
    const double m_velocityVariance;
    const double m_systematic_scan_error;


    InitiatorCandidates m_candidates;
    EKF m_kalmanFilter;
};


} // end of namespace srl_nnt


#endif // _LOGIC_INITIATOR_H
