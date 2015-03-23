#ifndef TRACKED_PERSONS_CACHE_H
#define TRACKED_PERSONS_CACHE_H

#include <map>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include "additional_topic_subscriber.h"

namespace spencer_tracking_rviz_plugin
{    
    typedef unsigned int track_id;

    /// Data structure for storing information about individual person tracks
    struct CachedTrackedPerson
    {
        Ogre::Vector3 center;
        geometry_msgs::PoseWithCovariance pose;
        geometry_msgs::TwistWithCovariance twist;
        bool isOccluded;
    };

    /// Subscribes to a TrackedPersons topic and caches all TrackedPersons of the current cycle, so that
    /// the owning rviz::Display can look up track positions etc for visualization.
    class TrackedPersonsCache {
    public:
        typedef std::map<track_id, shared_ptr<CachedTrackedPerson> > CachedTrackedPersonsMap;

        // Destructor
        ~TrackedPersonsCache();

        /// Create TrackedPersons subscriber and setup RViz properties.
        void initialize(rviz::Display* display, rviz::DisplayContext* context, ros::NodeHandle update_nh);

        /// Clear internal state, including all cached track positions.
        void reset();

        /// Lookup information for the given tracked person ID. Returns a null pointer if no information is available.
        const shared_ptr<CachedTrackedPerson> lookup(track_id trackId);

        /// Return internal map
        const CachedTrackedPersonsMap& getMap() {
            return m_cachedTrackedPersons;
        }

    private:
        // Callback when a new TrackedPersons message has arrived
        void processTrackedPersonsMessage(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);

        rviz::AdditionalTopicSubscriber<spencer_tracking_msgs::TrackedPersons>* m_tracked_person_subscriber;
        rviz::Display* m_display;
        rviz::DisplayContext* m_context;

        // Our TrackedPerson memory
        CachedTrackedPersonsMap m_cachedTrackedPersons;
    };
    

}

#endif // TRACKED_PERSONS_CACHE_H