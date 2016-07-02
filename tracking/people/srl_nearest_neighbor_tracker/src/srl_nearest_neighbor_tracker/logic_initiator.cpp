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

#include <srl_nearest_neighbor_tracker/nearest_neighbor_tracker.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/logic_initiator.h>
#include <srl_nearest_neighbor_tracker/data/observation.h>

#include <angles/angles.h>
#include <ros/ros.h>
#include <Eigen/LU>
#include <limits>
#include <map>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

namespace srl_nnt{


LogicInitiator::LogicInitiator()
: m_velMax(Params::get<double>("logic_initiator_max_velocity", 2.0)),
  m_velMin(Params::get<double>("logic_initiator_min_velocity", 0.2)),
  m_velMaxHighConfidence(Params::get<double>("logic_initiator_max_velocity_high_confidence", 2.0)),
  m_velMinHighConfidence(Params::get<double>("logic_initiator_min_velocity_high_confidence", 0.0)),
  m_numberScans(Params::get<int>("logic_initiator_number_scans_before_acceptance",6)),
  m_maxMissedObs(Params::get<int>("logic_initiator_max_number_consecutive_missed_observations",3)),
  m_incrementalCheck(Params::get<bool>("logic_initiator_use_incremental_checking", true)),
  m_systematic_scan_error(Params::get<double>("logic_initiator_systematic_scan_error", 0.07)),
  m_distMethod(EUCLIDEAN),
  m_maxMahaDistance(0.020100),
  m_maxAngleVariance(angles::from_degrees(30.0)),
  m_velocityVariance(0.1)
{
    std::string highConfidenceModalities = Params::get<std::string>("logic_initiator_high_confidence_modalities", "rgbd,mono,stereo");
    vector<std::string> modalities;
    boost::split(modalities, highConfidenceModalities, boost::is_any_of(","));
    foreach(std::string modality, modalities) {
        boost::trim(modality, std::locale(""));
        m_highConfidenceModalities.insert(modality);
    }

    ROS_INFO_STREAM("LogicInitiator set to the following configuration: \nMinimal velocity of tracks "
            << m_velMin << "m/s\nMaximal velocity of tracks " << m_velMax
            << "\nMinimal velocity when detection has any of the modalities > " << highConfidenceModalities
            << ": " << m_velMinHighConfidence << " m/s, max: " << m_velMaxHighConfidence
            << "m/s\nNumber of scans "<< m_numberScans << "\nAllowed missed observations "
            <<  m_maxMissedObs << "\nIncremental checking "<< m_incrementalCheck );

}

///
/// \brief LogicInitiator::processObservations  main function which processes new observations
/// to initiation candidates. And compares already existing candidates and observation with use of
/// cascaded logic.
/// \param observations new observations
/// \return new tracks to be initated to tracker
///
InitiatorCandidates LogicInitiator::processObservations(const Observations observations)
{        
    // declare structure for new tracks which should be initiated by tracker
    InitiatorCandidates newTracks;
    if(observations.empty()) return newTracks;

    // declare structure for accepted candidates
    InitiatorCandidates acceptedCandidates;

    // Get delta time
    double timeDiff = observations.front()->createdAt - m_lastObservationTime;
    m_lastObservationTime = observations.front()->createdAt;

    // Get observations which could not be matched by the tracker
    Observations newObs;
    foreach(Observation::Ptr ob, observations){
        if(!ob->matched)
        {
            newObs.push_back(ob);
        }
    }

    // Special case when no logic is required or desired
    if (m_numberScans < 2)
    {
        foreach(Observation::Ptr observation,newObs){
            if(!observation->matched)
            {
                InitiatorCandidate::Ptr newCandidate (new InitiatorCandidate);
                newCandidate->observations.push_back(observation);
                newCandidate->state = boost::dynamic_pointer_cast<KalmanFilterState>(
                        m_kalmanFilter.initializeTrackState(newCandidate->observations.back()) );
                newTracks.push_back(newCandidate);
            }
        }
        return newTracks;
    }


    // Associate new observations with existing candidates for initiation
    foreach(InitiatorCandidate::Ptr candidate, m_candidates){
        // initialize flag if pairing could be found for candidate
        bool pairingFound(false);


        // For first observation there exist no extrapolation just Gating
        if(candidate->observations.size() == 1){
            int ambigousCounter = 0;
            foreach(Observation::Ptr observation, newObs){
                if(compareWithCandidate(candidate, observation))
                {
                    pairingFound = true;
                    candidate->missedObs = 0;

                    VelocityVector initialVelocity;
                    initialVelocity(0) = (observation->z(0) - candidate->observations.back()->z(0)) / timeDiff;
                    initialVelocity(1) = (observation->z(1) - candidate->observations.back()->z(1)) / timeDiff;


                    if(updateKalman(candidate->state,observation))
                    {
                        candidate->observations.push_back(observation);
                        if (candidate->observations.size() != m_numberScans)
                            acceptedCandidates.push_back(candidate);
                        else
                            newTracks.push_back(candidate);
                        observation->matched= true;
                        ambigousCounter++;
                    }
                }
            }
            if(ambigousCounter > 1) ROS_DEBUG_STREAM("Encountered " << ambigousCounter << "ambigous tracks.");
        }
        // If selected number of scans will be reached tracks are created
        else if(candidate->observations.size() == m_numberScans-1)
        {
            predictKalman(candidate->state, timeDiff);
            candidate->extrapolation = linearExtrapolation(*(candidate->observations.rbegin()+1),
                                                           candidate->observations.back(),timeDiff);
            foreach(Observation::Ptr observation,newObs){
                if(compareWithExtrapolation(candidate,observation))
                {
                    if(compareWithCandidate(candidate, observation))
                    {
                        if(updateKalman(candidate->state,observation))
                        {
                            pairingFound = true;
                            candidate->missedObs = 0;
                            observation->matched = true;
                            candidate->observations.push_back(observation);
                            newTracks.push_back(candidate);
                        }
                    }
                }
            }
        }
        // Standard procedure: extrapolate with latest two observations and find candidates in gate
        else{
            predictKalman(candidate->state, timeDiff);
            candidate->extrapolation = linearExtrapolation(*(candidate->observations.rbegin()+1),
                                                           candidate->observations.back(),timeDiff);
            int ambigousCounter = 0;
            foreach(Observation::Ptr observation,newObs){
                if(compareWithExtrapolation(candidate,observation))
                {
                    if(compareWithCandidate(candidate, observation))
                    {
                        if(updateKalman(candidate->state,observation))
                        {
                            pairingFound = true;
                            candidate->missedObs = 0;
                            if(ambigousCounter > 0)
                            {
                                candidate.reset(new InitiatorCandidate(*candidate));
                                candidate->observations.back() = observation;
                            }
                            else
                                candidate->observations.push_back(observation);
                            acceptedCandidates.push_back(candidate);
                            observation->matched= true;
                            ambigousCounter++;
                        }
                    }
                }
            }
            if(ambigousCounter > 1) ROS_DEBUG_STREAM("Encountered " << ambigousCounter << "ambigous tracks.");
        }

        // Give secon chances to observation which are not continuous
        if(!pairingFound && candidate->missedObs < m_maxMissedObs)
        {
            ROS_DEBUG_STREAM("Allowed second chance with " << candidate->missedObs << " missed observations yet.");
            candidate->missedObs++;
            acceptedCandidates.push_back(candidate);
        }
    }
    foreach(Observation::Ptr observation,newObs){
        if(!observation->matched)
        {
            InitiatorCandidate::Ptr newCandidate (new InitiatorCandidate);
            newCandidate->observations.push_back(observation);
            newCandidate->state = boost::dynamic_pointer_cast<KalmanFilterState>(
                    m_kalmanFilter.initializeTrackState(newCandidate->observations.back()) );
            acceptedCandidates.push_back(newCandidate);
            ROS_DEBUG_STREAM("New Candidate with ID " << observation->id);
        }
    }
    m_candidates = acceptedCandidates;

    ROS_INFO_STREAM("Number of accepted initation candidates " << acceptedCandidates.size() << std::endl);
    return newTracks;
}


///
/// \brief LogicInitiator::compareWithCandidate compares new unmatched observation
/// with already existing candidates for track initiation.
/// \param candidate contains information about already seen tracks
/// \param newObservation new observation of current frame
/// \return boolean flag if association was sucessfull or not
///
bool LogicInitiator::compareWithCandidate(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation)
{
    // Switch on chosen distant method for association
    switch(m_distMethod)
    {
        // use euclidean distance as distance measure
        case EUCLIDEAN:
            // if all previous observations should be checked for associaction
            if(m_incrementalCheck)
            {
                foreach(Observation::Ptr associatedObservation, candidate->observations){
                    // calculate time difference between current observation and previous one
                    double timeDiff = newObservation->createdAt - associatedObservation->createdAt;
                    // check euclidean distance
                    if(!checkEuclideanDist(associatedObservation, newObservation, timeDiff))
                        return false;
                }
                return true;
            }
            // just check latest observation of candidate with new observation
            else
            {
                double timeDiff = newObservation->createdAt - candidate->observations.back()->createdAt;
                return checkEuclideanDist(candidate->observations.back(), newObservation, timeDiff);
            }
            break;

            // use mahalanobis distance as distance measure
        case MAHALANOBIS:
            // calculate velocity limits depending on current kalman filter state
            double velMin, velMax;
            getApprovedVelocityLimits(newObservation, velMin, velMax);
            double vminX (velMin);
            double vminY (velMin);
            double vmaxX (velMax);
            double vmaxY (velMax);

            if(std::abs(candidate->state->x()(2))- m_velocityVariance > velMin && std::abs(candidate->state->x()(2))+ m_velocityVariance < velMax)
            {
                vminX = std::abs(candidate->state->x()(2)) - m_velocityVariance;
                vmaxX = std::abs(candidate->state->x()(2)) + m_velocityVariance;
            }
            if(std::abs(candidate->state->x()(3)) - m_velocityVariance > velMin && std::abs(candidate->state->x()(3)) + m_velocityVariance < velMax)
            {
                vminY = std::abs(candidate->state->x()(3)) - m_velocityVariance;
                vmaxY = std::abs(candidate->state->x()(3)) + m_velocityVariance;
            }

            // if all previous observations should be checked for associaction
            if(m_incrementalCheck)
            {
                foreach(Observation::Ptr associatedObservation, candidate->observations){
                    // calculate time difference between current observation and previous one
                    double timeDiff = newObservation->createdAt - associatedObservation->createdAt;

                    // check mahlanobis distance with velocity limits
                    if(!checkMahalonobisDistance(associatedObservation,newObservation, vminX, vmaxX, vminY, vmaxY, timeDiff))
                        return false;
                }
                return true;
            }

            // just check latest observation of candidate with new observation
            else
            {
                double timeDiff = newObservation->createdAt - candidate->observations.back()->createdAt;
                return checkMahalonobisDistance(candidate->observations.back(),newObservation,
                                                vminX, vmaxX, vminY, vmaxY, timeDiff);
            }
            break;
    }

    return false;
}


///
/// \brief LogicInitiator::compareWithExtrapolation compare new observation
/// with extrapolation depending on previous observations
/// \param candidate
/// \param newObservation
/// \return boolean flag if check was sucessfull or not
///
bool LogicInitiator::compareWithExtrapolation(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation)
{
    // switch on chosen distance method for association
    switch (m_distMethod) {
        case EUCLIDEAN:
            return checkEuclideanDist(candidate->extrapolation, newObservation,0.0);
            break;

        case MAHALANOBIS:
            return simpleMahalanobisCheck(candidate, newObservation);
            break;

        default:
            ROS_ERROR("Unknown distance method");
            return false;
            break;
    }
}


///
/// \brief LogicInitiator::simpleMahalanobisCheck check mahalanobis distace
/// between current kalman filter state of candidate and new observation
/// \param candidate contains current kalman filter state
/// \param newObservation is new observation from current frame
/// \return boolean flag if check was sucessfull or not
///
bool LogicInitiator::simpleMahalanobisCheck(const InitiatorCandidate::Ptr candidate, const Observation::Ptr newObservation)
{
    ObsVector diff (candidate->state->xp().head(OBS_DIM));
    diff -= newObservation->z;
    ObsMatrix S ;
    S(0,0) = candidate->state->Cp()(0,0);
    S(0,1) = candidate->state->Cp()(0,1);
    S(1,0) = candidate->state->Cp()(1,0);
    S(1,1) = candidate->state->Cp()(1,1);
    double distance (diff.transpose()*S.inverse()*diff);
    if(distance < m_maxMahaDistance)
    {
        ROS_INFO_STREAM("Simple Mahalanobis Distance: " << distance << " obs " << newObservation->R << " candidate " << candidate->state->Cp());
        return true;
    }
    else {
        return false;
    }
}


///
/// \brief LogicInitiator::doAngleCheck UNUSED
/// \param obs1
/// \param obs2
/// \param meanAngle
/// \return
///
bool LogicInitiator::doAngleCheck(const Observation::Ptr obs1, const Observation::Ptr obs2,double& meanAngle)
{
    double angle =  std::atan2(obs2->z(1)- obs1->z(1),obs2->z(0)- obs1->z(0));

    if(std::abs(meanAngle-angle) < m_maxAngleVariance)
    {
        meanAngle += angle;
        meanAngle /= 2.0;
        return true;
    }
    else {
        return false;
    }
}

///
/// \brief Returns the minimum and maximum approved velocity for a track, depending on whether
/// the incoming detections are high-confidence or not.
///
void LogicInitiator::getApprovedVelocityLimits(const Observation::Ptr obs, double& velMin, double& velMax)
{
    bool isHighConfidenceObservation = false;

    foreach(const std::string& modality, obs->modalities) {
        if(m_highConfidenceModalities.find(modality) != m_highConfidenceModalities.end()) {
            isHighConfidenceObservation = true;
            break;
        }
    }

    if(isHighConfidenceObservation) {
        velMin = m_velMinHighConfidence;
        velMax = m_velMaxHighConfidence;
    }
    else {
        velMin = m_velMin;
        velMax = m_velMax;
    }
}

///
/// \brief LogicInitiator::linearExtrapolation calculates linear extrapolation
/// depending on two observations.
/// \param ob1 first observation
/// \param ob2 second observation
/// \param timeDiffFrames timedifference between observations
/// \return extrapolation in form of an observation
///
Observation::Ptr LogicInitiator::linearExtrapolation(Observation::Ptr ob1, Observation::Ptr ob2, double timeDiffFrames)
{
    ObsVector slope = (ob2->z - ob1->z) / (ob2->createdAt - ob1->createdAt) ;
    Observation prediction = *(ob2);
    prediction.z += (slope * timeDiffFrames);
    Observation::Ptr predictionPtr = boost::make_shared<Observation>(prediction);
    return predictionPtr;
}


bool LogicInitiator::checkEuclideanDist(const Observation::Ptr obs1,const Observation::Ptr obs2, const double deltaTime)
{
    double velMin, velMax;
    getApprovedVelocityLimits(obs2, velMin, velMax);
    double distMin((velMin*deltaTime)*(velMin*deltaTime));
    double distMax((velMax*deltaTime)*(velMax*deltaTime)+ m_systematic_scan_error);

    // Using euclidean distance
    ObsVector diffVector = obs2->z - obs1->z;
    double eucliDistance = (diffVector.transpose() * diffVector);

    if(eucliDistance > distMin && eucliDistance < distMax)
    {
        ROS_DEBUG_STREAM("Euclidean Distance: " << eucliDistance << "Limits [ "
                         << distMin << " ; "
                         << distMax
                         << " ] ; t="
                         << deltaTime);
        return true;
    }
    else if(eucliDistance < 0.4 && deltaTime  == 0.0)
    {
        ROS_DEBUG_STREAM("Euclidean Distance: " << eucliDistance << "Limits [ "
                         << distMin << " ; "
                         << distMax
                         << " ] ; t="
                         << deltaTime  << " Observation x :" << obs1->z[0] << std::endl );
        return false;
    }
    else
    {
        return false;
    }

}


//*/TODO: Implement with Mahalanobis Distance
/*
bool LogicInitiator::checkMahalonobisDistance(Observation::Ptr obs1, const Observation::Ptr obs2, double deltaTime)
{
    ObsVector dist;

    // x direction
    double factor = 1.0/deltaTime;
    double xMax = std::abs(obs2->z(0) - obs1->z(0))*factor - (m_velMax*deltaTime);
    if(xMax < 0)
    {
        xMax = 0.0;
    }
    double xMin = std::abs(obs2->z(0) - obs1->z(0))*factor - (m_velMin*deltaTime);
    if(xMin > 0)
    {
        xMin = 0.0;
    }
    else {
        xMin = std::abs(xMin);
    }

    // y direction
    double yMax = std::abs(obs2->z(1) - obs1->z(1))*factor - (m_velMax*deltaTime);
    if(yMax < 0)
    {
        yMax = 0.0;
    }
    double yMin = std::abs(obs2->z(1) - obs1->z(1))*factor - (m_velMin*deltaTime);
    if(yMin > 0)
    {
        yMin = 0.0;
    }
    else {
        yMin = std::abs(yMin);
    }

    dist(0) = xMax+xMin;
    dist(1) = yMax+yMin;

    // TODO: Check for not standard covariances because they are set to a large value now
    ObsMatrix covSum = obs1->R + obs2->R;
    ObsMatrix inverseSum = covSum.inverse();

    double maha = dist.transpose()*inverseSum*dist;
    ROS_DEBUG_STREAM("Ob[" << obs2->id << "] at x:" << obs2->z(0) << " y:" << obs2->z(1) << "with distance:" << maha << " to state x:" << obs1->z(0)  << " y:" << obs1->z(1) );
    if(maha <= CHI2INV_99[2])
    {
        ROS_DEBUG_STREAM("Mahalanobis Distance: " << maha << " delta t " << deltaTime);
        ROS_DEBUG_STREAM("Covariance: \n" << covSum << "\n dx" <<  std::abs(obs2->z(0) - obs1->z(0)) << " dy " << std::abs(obs2->z(1) - obs1->z(1)));
        return true;
    }

    return false;
}
*/


//*/TODO: Implement with Mahalanobis Distance
/*
bool LogicInitiator::checkMahalonobisDistance(KalmanFilterState::Ptr state, const Observation::Ptr obs, double deltaTime)
{
    ObsVector dist;

    // x direction
    ROS_DEBUG_STREAM("Predicted velocity: " << std::abs(state->m_xp(2)) << " ; " << std::abs(state->m_xp(3)));
    double factor = 1.0/deltaTime;
    double xMax = std::abs(state->m_xp(0) - obs->z(0))*factor - ((state->m_xp(2)+0.2));
    if(xMax < 0)
    {
        xMax = 0.0;
    }
    double xMin = std::abs(state->m_xp(0) - obs->z(0))*factor  - ((state->m_xp(2)-0.2));
    if(xMin > 0)
    {
        xMin = 0.0;
    }
    else {
        xMin = std::abs(xMin);
    }

    // y direction
    double yMax = std::abs(state->m_xp(1) - obs->z(1))*factor  - ((state->m_xp(3)+0.2));
    if(yMax < 0)
    {
        yMax = 0.0;
    }
    double yMin = std::abs(state->m_xp(1) - obs->z(1))*factor  - ((state->m_xp(3)-0.2));
    if(yMin > 0)
    {
        yMin = 0.0;
    }
    else {
        yMin = std::abs(yMin);
    }

    dist(0) = xMax+xMin;
    dist(1) = yMax+yMin;

    ObsMatrix covSum = obs->R;
    covSum(0,0) += state->m_C(0,0);
    covSum(0,1) += state->m_C(0,1);
    covSum(1,0) += state->m_C(1,0) ;
    covSum(1,1) += state->m_C(1,1) ;
    ObsMatrix inverseSum = covSum.inverse();
    double maha = (dist.transpose()*inverseSum*dist);


    if(maha <= CHI2INV_99[2])
    {
        ROS_DEBUG_STREAM("Ob[" << obs->id << "] at x:" << obs->z(0) << " y:" << obs->z(1)
                         << "with distance:" << maha << " to state \nx:" << state->m_xp(0)
                         <<"(" << state->m_x(0)<< ") y:" << state->m_xp(1) << " (" << state->m_x(1)<< ")" );
        ROS_INFO_STREAM("Mahalanobis Distance: " << maha << " delta t " << deltaTime);
        ROS_DEBUG_STREAM("Covariance: \n" << state->m_C << "\n ");
        ROS_DEBUG_STREAM("Covariance predicted: \n" << state->m_Cp << "\n ");
        return true;
    }

    return false;
}
*/

//*/TODO: Implement with Mahalanobis Distance
bool LogicInitiator::checkMahalonobisDistance(const Observation::Ptr obs1, const Observation::Ptr obs2,
                                              const double vMinX, const double vMaxX,
                                              const double vMinY, const double vMaxY, const double deltaTime)
{
    ObsVector dist;

    double xMax = std::abs(obs1->z(0) - obs2->z(0)) - (vMaxX*deltaTime);
    if(xMax < 0)
    {
        xMax = 0.0;
    }
    double xMin = std::abs(obs1->z(0) - obs2->z(0))  - (vMinX*deltaTime);
    if(xMin > 0)
    {
        xMin = 0.0;
    }
    else {
        xMin = std::abs(xMin);
    }

    // y direction
    double yMax = std::abs(obs1->z(1) - obs2->z(1))  - (vMaxY*deltaTime);
    if(yMax < 0)
    {
        yMax = 0.0;
    }
    double yMin = std::abs(obs1->z(1) - obs2->z(1)) - (vMinY*deltaTime);
    if(yMin > 0)
    {
        yMin = 0.0;
    }
    else {
        yMin = std::abs(yMin);
    }

    dist(0) = xMax+xMin;
    dist(1) = yMax+yMin;

    ObsMatrix covSum = obs2->R;
    //    covSum(0,0) += state->C(0,0);
    //    covSum(0,1) += state->C(0,1);
    //    covSum(1,0) += state->C(1,0) ;
    //    covSum(1,1) += state->C(1,1) ;
    ObsMatrix inverseSum = covSum.inverse();
    double maha = (dist.transpose()*inverseSum*dist);

    if(maha <= m_maxMahaDistance)
    {
        ROS_INFO_STREAM("Velocity Mahalanobis Distance: " << maha);
        return true;
    }

    return false;
}


void LogicInitiator::predictKalman(KalmanFilterState::Ptr state, double deltatime)
{
    StateMatrix A = StateMatrix::Identity();
    A(0, 2) = deltatime;
    A(1, 3) = deltatime;
    m_kalmanFilter.setTransitionMatrix(state->x(), deltatime);
    m_kalmanFilter.predictTrackState(state, deltatime);
    ROS_DEBUG_STREAM("Predicted state kalman x:" << state->m_xp(0) << " ; y" << state->m_xp(1) << "\nDelta t: " << deltatime);
}


bool LogicInitiator::updateKalman(KalmanFilterState::Ptr state, Observation::ConstPtr observation)
{
    ROS_DEBUG_STREAM("Updating Kalman Filter");

    // Create a new pairing
    Pairing::Ptr pairing( new Pairing );
    pairing->validated = false;

    // For the moment, our "track-to-measurement model" is very simple: Just copy x and y position, and forget about the vx, vy (measurements don't have velocities)
    ObsStateMatrix fixed_H = ObsStateMatrix::Zero();
    fixed_H(0,0) = 1.0;
    fixed_H(1,1) = 1.0;
    state->updateMeasurementPrediction(fixed_H);

    // Calculate innovation v and inverse of innovation covariance S
    pairing->v = observation->z - state->m_zp;

    ObsMatrix S = state->m_H * state->m_Cp * state->m_H.transpose() + observation->R;
    Eigen::FullPivLU<ObsMatrix> lu(S);
    double ln_det_S = log(lu.determinant());

    // Calculate inverse of innovation covariance if possible
    if(ln_det_S > -1e8) {
        pairing->Sinv = lu.inverse();
        pairing->d = (pairing->v.transpose() * pairing->Sinv * pairing->v)(0,0);
        pairing->singular = pairing->d < 0.0;
    }
    else {
        pairing->Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, numeric_limits<double>::quiet_NaN());
        pairing->d = numeric_limits<double>::quiet_NaN();
        pairing->singular = true;

        ROS_WARN_STREAM("Singular pairing encountered!\n");
    }

    // Perform gating
    if(!pairing->singular&& pairing->v.norm() < 1.0) {
        // Store in list of compatible pairings
        m_kalmanFilter.updateMatchedTrack(state, pairing);
        //ROS_DEBUG_STREAM(pairing->v.norm());
        return true;
    }
    return false;
}

} // end of namespace srl_nnt

