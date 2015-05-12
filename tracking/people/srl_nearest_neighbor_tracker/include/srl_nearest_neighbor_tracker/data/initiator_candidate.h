/* Created on: Feb 15, 2015. Author: Fabian Girrbach */
#ifndef _INITIATOR_CANDIDATE_H
#define _INITIATOR_CANDIDATE_H

#include <boost/shared_ptr.hpp>
#include <vector>


#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/kalman_filter_state.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>


namespace srl_nnt
{
struct InitiatorCandidate 
{
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


} // end of namespace srl_nnt


#endif // _INITIATOR_CANDIDATE_H
