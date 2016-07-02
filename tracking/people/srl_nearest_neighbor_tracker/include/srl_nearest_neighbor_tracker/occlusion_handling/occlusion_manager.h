/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_MANAGER_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_MANAGER_H_

#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/data/pairing.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>


namespace srl_nnt {

/// Generic occlusion manager interface for deleting occluded tracks after a certain time 
class OcclusionManager
{
public:
    virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle)=0;
    virtual Tracks manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time, const std::string& trackFrameID)=0;
    virtual void deleteOccludedTracks(Tracks& tracks,const ros::Time& time)=0;
    virtual Pairings occludedTrackAssociation(Tracks tracks, Observations observations, const ros::Time& time) =0;

    void setFrameIDofTracker(const string& frameID)
    {
        m_frameIDTracker = frameID;
    }

    typedef boost::shared_ptr<OcclusionManager> Ptr;
    typedef boost::shared_ptr<const OcclusionManager> ConstPtr;

 protected:
 	/// Frame ID where tracker operates
    string m_frameIDTracker;
    
};

}


#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_MANAGER_H_ */
