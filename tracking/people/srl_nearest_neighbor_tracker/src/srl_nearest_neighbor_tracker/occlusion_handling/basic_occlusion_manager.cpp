/* Created on: Mar 26, 2015. Author: Fabian Girrbach */
#include <srl_nearest_neighbor_tracker/occlusion_handling/basic_occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace srl_nnt {

void BasicOcclusionManager::initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle)
{
    m_MAX_OCCLUSIONS_BEFORE_DELETION = Params::get<int>("max_occlusions_before_deletion", 20);
    m_MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK = Params::get<int>("max_occlusions_before_deletion_of_mature_track", 120);
    m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES = Params::get<int>("track_is_mature_after_total_num_matches", 100);
}


void BasicOcclusionManager::manageOcclusionsBeforeDataAssociation(Tracks& tracks,const ros::Time& time, const std::string& trackFrameID)
{
    foreach(Track::Ptr track, tracks){
        track->trackStatus = Track::OCCLUDED;
    }
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
        if(track->numberOfConsecutiveOcclusions > occlusionFrameLimit) {
            tracksToDelete.push_back(trackIt);
            numDeletedTracks++;
        }
    }
    
    for(int i = tracksToDelete.size()-1; i >= 0; i--) {
        tracks.erase(tracksToDelete.at(i));
    }

    if(numDeletedTracks) ROS_DEBUG("%zu track(s) have been deleted!", numDeletedTracks);
}

}



