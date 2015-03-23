#include "tracked_persons_cache.h"

#include <sstream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


namespace spencer_tracking_rviz_plugin
{

TrackedPersonsCache::~TrackedPersonsCache()
{
    m_cachedTrackedPersons.clear();
    delete m_tracked_person_subscriber;
}

void TrackedPersonsCache::initialize(rviz::Display* display, rviz::DisplayContext* context, ros::NodeHandle update_nh)
{
    m_display = display;
    m_context = context;

    m_tracked_person_subscriber = new rviz::AdditionalTopicSubscriber<spencer_tracking_msgs::TrackedPersons>("Tracked persons topic", display, context, update_nh,
        boost::bind(&TrackedPersonsCache::processTrackedPersonsMessage, this, _1));
}

void TrackedPersonsCache::reset()
{
    m_cachedTrackedPersons.clear();
}

const shared_ptr<CachedTrackedPerson> TrackedPersonsCache::lookup(track_id trackId)
{
    CachedTrackedPersonsMap::const_iterator entry = m_cachedTrackedPersons.find(trackId);
    if(entry == m_cachedTrackedPersons.end()) return shared_ptr<CachedTrackedPerson>();
    else return entry->second;
}

void TrackedPersonsCache::processTrackedPersonsMessage(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg)
{
    // Get transform of person tracks into fixed frame
    Ogre::Vector3 frameOrigin; Ogre::Quaternion frameOrientation;
    m_context->getFrameManager()->getTransform(msg->header, frameOrigin, frameOrientation);
    Ogre::Matrix4 transform(frameOrientation);
    transform.setTrans(frameOrigin);

    // Now iterate over all tracks and store their positions
    m_cachedTrackedPersons.clear();
    foreach(spencer_tracking_msgs::TrackedPerson trackedPerson, msg->tracks)
    {
        m_cachedTrackedPersons[trackedPerson.track_id] = shared_ptr<CachedTrackedPerson>(new CachedTrackedPerson);
        CachedTrackedPerson& cachedTrackedPerson = *m_cachedTrackedPersons[trackedPerson.track_id];

        const geometry_msgs::Point& position = trackedPerson.pose.pose.position;
        cachedTrackedPerson.center = transform * Ogre::Vector3(position.x, position.y, position.z);
        cachedTrackedPerson.pose = trackedPerson.pose;
        cachedTrackedPerson.twist = trackedPerson.twist;
        cachedTrackedPerson.isOccluded = trackedPerson.is_occluded;
    }

    std::stringstream ss;
    ss << msg->tracks.size() << " track(s)";
    m_display->setStatusStd(rviz::StatusProperty::Ok, "Tracks", ss.str());
}


} // end namespace spencer_tracking_rviz_plugin