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

#include <srl_nearest_neighbor_tracker/occlusion_handling/basic_occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/laser_shade_occlusion_manager.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace srl_nnt {

void BasicOcclusionManager::initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle)
{
    m_MAX_OCCLUSIONS_BEFORE_DELETION = Params::get<int>("max_occlusions_before_deletion", 20);
    m_MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK = Params::get<int>("max_occlusions_before_deletion_of_mature_track", 120);
    m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES = Params::get<int>("track_is_mature_after_total_num_matches", 100);
    m_viewFieldMinLimitX = 0;
    m_viewFieldMaxLimitX = 30;
    m_viewFieldMinLimitY = -20;
    m_viewFieldMaxLimitY = 20;
}


Tracks BasicOcclusionManager::manageOcclusionsBeforeDataAssociation(Tracks& tracks,const ros::Time& time, const std::string& trackFrameID)
{
    foreach(Track::Ptr track, tracks){
        track->trackStatus = Track::MISSED;
        track->detectionProbability = 1.0;
    }
    return Tracks();
}


void  BasicOcclusionManager::deleteOccludedTracks(Tracks& tracks,const ros::Time& time)
{
    ROS_DEBUG("Deleting obsolete tracks");

    size_t numDeletedTracks = 0;
    std::vector<Tracks::iterator> tracksToDelete;
    for(Tracks::iterator trackIt = tracks.begin(); trackIt != tracks.end(); ++trackIt) {
        Track::Ptr track = *trackIt;

        // Check if the track is considered as "mature", i.e. it has been there for a long time already.
        const bool trackIsMature = track->numberOfTotalMatches >= m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES;

        // Check if the track hasn't been seen for too long.
        const int occlusionFrameLimit = trackIsMature ? m_MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK : m_MAX_OCCLUSIONS_BEFORE_DELETION;
        if(track->numberOfConsecutiveOcclusions > occlusionFrameLimit || track->numberOfConsecutiveMisses > occlusionFrameLimit) {
            tracksToDelete.push_back(trackIt);
            numDeletedTracks++;
        }
//        //Hack for evaluation not valid for mobile robots
//        else if (track->state->x()(STATE_X_IDX) < m_viewFieldMinLimitX || track->state->x()(STATE_X_IDX) > m_viewFieldMaxLimitX)
//        {
//            tracksToDelete.push_back(trackIt);
//            numDeletedTracks++;
//        }
//        else if (track->state->x()(STATE_Y_IDX) < m_viewFieldMinLimitY || track->state->x()(STATE_Y_IDX) > m_viewFieldMaxLimitY)
//        {
//            tracksToDelete.push_back(trackIt);
//            numDeletedTracks++;
//        }
    }

    for(int i = tracksToDelete.size()-1; i >= 0; i--) {
        tracks.erase(tracksToDelete.at(i));
    }

    if(numDeletedTracks) ROS_DEBUG("%zu track(s) have been deleted!", numDeletedTracks);
}

}



