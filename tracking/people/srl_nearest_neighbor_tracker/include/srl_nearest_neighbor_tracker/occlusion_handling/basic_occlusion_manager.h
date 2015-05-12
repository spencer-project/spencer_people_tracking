/* Created on: Mar 26, 2015. Author: Fabian Girrbach */
#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_BASIC_OCCLUSION_MANAGER_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_BASIC_OCCLUSION_MANAGER_H_

#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/data/track.h>


namespace srl_nnt {

// Simple OcclusionManager that deletes occluded tracks after a fixed number of frames
class BasicOcclusionManager: public OcclusionManager
{
public:
    virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle);
    virtual void manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time, const std::string& trackFrameID);
    virtual void deleteOccludedTracks(Tracks& tracks,const ros::Time& time);

    typedef boost::shared_ptr<BasicOcclusionManager> Ptr;
    typedef boost::shared_ptr<const BasicOcclusionManager> ConstPtr;

private:
    int m_MAX_OCCLUSIONS_BEFORE_DELETION ;
    int m_MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK ;
    int m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES ;
};

}


#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_BASIC_OCCLUSION_MANAGER_H_ */
