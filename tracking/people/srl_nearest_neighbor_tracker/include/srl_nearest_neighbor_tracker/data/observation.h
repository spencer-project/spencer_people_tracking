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

#ifndef _OBSERVATION_H
#define _OBSERVATION_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <set>
#include <Eigen/Core>

#include <srl_nearest_neighbor_tracker/base/defs.h>

namespace srl_nnt
{
/// Globally unique ID of an observation which is valid over the entire lifetime of the tracker
typedef unsigned int observation_id;

/// An Observation found in a sensor reading.
/// The Observation contains the 2D position z in cartesian space and its covariance R.
struct Observation
{
    /// ID of the Observation. 
    observation_id id;

    /// Creation time of the observation in seconds. Usually set from ROS header or logfile timestamp.
    double createdAt;

    /// Observation state vector z. 
    ObsVector z;

    /// Covariance R. 
    ObsMatrix R;

    /// Confidence in being a tracking target. 
    double confidence;

    /// If the observation has been matched against a track.
    bool matched;

    /// The modality or modalities that this observation originates from.
    std::set<std::string> modalities;

    /// Typedefs for easier readability
    typedef boost::shared_ptr<Observation> Ptr;
    typedef boost::shared_ptr<const Observation> ConstPtr;
};

/// Typedef for easier readability
typedef std::vector<Observation::Ptr> Observations;


} // end of namespace srl_nnt


#endif // _OBSERVATION_H
