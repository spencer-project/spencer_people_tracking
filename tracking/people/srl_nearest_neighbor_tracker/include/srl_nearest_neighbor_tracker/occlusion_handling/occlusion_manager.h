/* Created on: Mar 26, 2015. Author: Fabian Girrbach */
#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_MANAGER_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_MANAGER_H_

#include <srl_nearest_neighbor_tracker/data/track.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace srl_nnt {

/// Generic occlusion manager interface for deleting occluded tracks after a certain time 
class OcclusionManager
{
public:
    virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle)=0;
    virtual void manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time, const std::string& trackFrameID)=0;
    virtual void deleteOccludedTracks(Tracks& tracks,const ros::Time& time)=0;

    typedef boost::shared_ptr<OcclusionManager> Ptr;
    typedef boost::shared_ptr<const OcclusionManager> ConstPtr;
};

}


#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_MANAGER_H_ */
