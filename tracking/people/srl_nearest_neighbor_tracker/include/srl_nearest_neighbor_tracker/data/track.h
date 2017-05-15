/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Timm Linder, Social Robotics Lab, University of Freiburg
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

#ifndef _TRACK_H
#define _TRACK_H

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <vector>


#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/kalman_filter_state.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>


namespace srl_nnt
{

/// Globally unique ID of a track which is valid over the entire lifetime of the tracker
typedef unsigned int track_id;
typedef unsigned int model_index;

/// A Track is the representation of an object, observed and tracked with an Extended Kalman Filter.
struct Track
{
    /// State of a track
    enum TrackStatus
    {
        NEW, MATCHED, MISSED, OCCLUDED, DELETED
    };

    /// State of the track. 
    TrackStatus trackStatus;

    /// Globally unique track ID
    track_id id;

    /// Model index for IMM Models
    model_index model_idx;

    /// Initializing or matching Observation. NULL for OCCLUDED or DELETED tracks.
    Observation::Ptr observation;

    /// Creation time of the track in seconds.
    double createdAt;

    /// Number of matches during the life-cycle of the Track. 
    unsigned int numberOfTotalMatches;

    /// Number of consecutive occlusions. 
    unsigned int numberOfConsecutiveOcclusions;

    /// Number of consecutive misses.
    unsigned int numberOfConsecutiveMisses;

    /// Number of consecutive weak matches (with a recovered observation, e.g. by actively searching the laser scan)
    unsigned int numberOfConsecutiveWeakMatches;

    /// Detection probability
    double detectionProbability;

    /// Filter state (e.g. from a Kalman filter), including current and predicted track position
    FilterState::Ptr state;

    /// History of filter states for debugging.
    typedef boost::circular_buffer<FilterState::Ptr> FilterStateHistory;
    FilterStateHistory stateHistory;

    /// Typedefs for easier readability
    typedef boost::shared_ptr<Track> Ptr;
    typedef boost::shared_ptr<const Track> ConstPtr;
};


/// Typedef for easier readability
typedef std::vector<Track::Ptr> Tracks;


} // end of namespace srl_nnt


#endif // _TRACK_H
