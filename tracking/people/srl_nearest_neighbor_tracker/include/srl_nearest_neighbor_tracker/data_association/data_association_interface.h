/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#ifndef _DATA_ASSOCIATION_DATA_ASSOCIACTION_INTERFACE_H
#define _DATA_ASSOCIATION_DATA_ASSOCIACTION_INTERFACE_H

#include <ros/ros.h>
#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/data/pairing.h>
#include <boost/shared_ptr.hpp>


namespace srl_nnt {

class DataAssociationInterface {
public:
    /// Initialization method where parameters for data association are read and set accordingly
    virtual void initializeDataAssociation(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle) = 0;

    /// Actual data association algorithm which takes tracks and observations and returns Pairings
    virtual Pairings performDataAssociation(Tracks& tracks, const Observations& observations) = 0;

    /// typedefs for easier readability
    typedef boost::shared_ptr<DataAssociationInterface> Ptr;
    typedef boost::shared_ptr<const DataAssociationInterface> ConstPtr;


protected:
    /// Marks a track as matched with the observation in the given pairing
    virtual void markAsMatched(Track::Ptr& track, Pairing::Ptr& pairing) {
        ROS_ASSERT(pairing->observation != NULL);

        pairing->validated = true;
        pairing->observation->matched = true;
        track->observation = pairing->observation;
        track->trackStatus = Track::MATCHED;
        track->numberOfTotalMatches++;
        track->numberOfConsecutiveOcclusions = 0;
        track->numberOfConsecutiveMisses = 0;
    }

    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;
};


}

#endif //_DATA_ASSOCIATION_DATA_ASSOCIACTION_INTERFACE_H
