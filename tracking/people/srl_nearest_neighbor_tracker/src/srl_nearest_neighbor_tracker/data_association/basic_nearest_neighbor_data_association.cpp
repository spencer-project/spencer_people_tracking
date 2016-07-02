/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#include <srl_nearest_neighbor_tracker/data_association/basic_nearest_neighbor_data_association.h>


#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/LU>
#include <limits>
#include <map>


namespace srl_nnt {


void BasicNearestNeighborDataAssociation::initializeDataAssociation(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle) {
    m_nodeHandle = nodeHandle;
    m_privateNodeHandle = privateNodeHandle;
}


Pairings BasicNearestNeighborDataAssociation::performDataAssociation(Tracks& tracks, const Observations& observations){

    ROS_DEBUG("Performing data association");

    //
    // Step 0: Make sure observations are in a good state
    //

    foreach(Observation::Ptr observation, observations) {
        observation->matched = false;
    }

    Pairings compatiblePairings;

    //
    // Step 1: go through all possible associations and store compatible ones
    //

    const double MATRIX_LN_EPS = -1e8;
    const double MAX_GATING_DISTANCE = Params::get<double>("max_gating_distance", 1.0);

    typedef multimap<track_id, Pairing::Ptr> TrackSpecificPairings;
    TrackSpecificPairings trackSpecificPairings;

    ObsVector v;
    foreach(Track::Ptr track, tracks)
    {
        foreach(Observation::Ptr observation, observations)
                                                                                                                        {
            v = observation->z - track->state->zp();

            if(v.norm() < MAX_GATING_DISTANCE)
            {
                // Create a new pairing
                Pairing::Ptr pairing( new Pairing );

                pairing->track = track;
                pairing->observation = observation;
                pairing->validated = false;

                // Calculate innovation v and inverse of innovation covariance S
                pairing->v = v;

                ObsMatrix S = track->state->H() * track->state->Cp() * track->state->H().transpose() + observation->R;
                Eigen::FullPivLU<ObsMatrix> lu(S);
                double ln_det_S = log(lu.determinant());

                // Calculate inverse of innovation covariance if possible
                if (ln_det_S > MATRIX_LN_EPS) {
                    pairing->Sinv = lu.inverse();
                    pairing->d = (pairing->v.transpose() * pairing->Sinv * pairing->v)(0,0);
                    pairing->singular = pairing->d < 0.0;
                }
                else {
                    pairing->Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, numeric_limits<double>::quiet_NaN());
                    pairing->d = numeric_limits<double>::quiet_NaN();
                    pairing->singular = true;

                    ROS_WARN_STREAM("Singular pairing encountered!\nTrack " << track->id << " measurement prediction:\n" << track->state->zp() << "\nTrack covariance prediction:\n" << track->state->Cp()
                                    << "\nObservation " << observation->id << " mean:\n" << observation->z << "\nObservation covariance:\n" << observation->R );
                }

                // Perform gating
                if(!pairing->singular && pairing->d < CHI2INV_99[OBS_DIM]) {
                    // Store in list of compatible pairings
                    trackSpecificPairings.insert( make_pair(track->id, pairing) );
                }
            }
                                                                                                                        }
    }

    ROS_DEBUG("%zu compatible pairings have been found for %zu existing tracks and %zu new observations!", compatiblePairings.size(), tracks.size(), observations.size() );



    //
    // Step 2: go through all tracks, choose and mark nearest neighbour
    //

    foreach(Track::Ptr track, tracks)
    {
        // Store best pairing for this track
        double best_d = numeric_limits<double>::max();
        Pairing::Ptr bestPairing;

        // Iterate over all pairings for this track
        pair<TrackSpecificPairings::iterator, TrackSpecificPairings::iterator> pairingsForThisTrack = trackSpecificPairings.equal_range(track->id);
        for(TrackSpecificPairings::iterator it = pairingsForThisTrack.first; it != pairingsForThisTrack.second; it++) {
            Pairing::Ptr pairing = it->second;

            // Allow that a observation can be matched to several tracks
            // is better than what we have seen so far.
            if(pairing->d < best_d) {
                best_d = pairing->d;
                bestPairing = pairing;
            }
        }

        // See if we found a compatible pairing for this track.
        if(bestPairing) {
            compatiblePairings.push_back(bestPairing);
            markAsMatched(track, bestPairing);
        }
        else if (track->trackStatus == Track::OCCLUDED){
            // Track is occluded
            track->observation.reset();
            track->trackStatus = Track::OCCLUDED;
            track->numberOfConsecutiveOcclusions++;
        }
        else{
            // Track is missed
            track->observation.reset();
            track->trackStatus = Track::MISSED;
            track->numberOfConsecutiveMisses++;
        }
    }

    return compatiblePairings;
}



}
