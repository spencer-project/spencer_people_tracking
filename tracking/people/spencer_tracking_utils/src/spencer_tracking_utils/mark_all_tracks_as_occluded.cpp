#include <ros/ros.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

ros::Publisher g_filteredTracksPublisher;


void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons) {
    const double now = trackedPersons->header.stamp.toSec();
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    filteredTracks->header = trackedPersons->header;

    // Iterate over current tracks
    foreach(const TrackedPerson& trackedPerson, trackedPersons->tracks) {
        TrackedPerson clone = trackedPerson;
        clone.is_occluded = true;
        filteredTracks->tracks.push_back(clone);
    }

    // Publish filtered tracks
    g_filteredTracksPublisher.publish(filteredTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mark_all_tracks_as_occluded");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";

    ros::Subscriber tracksSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Marking all tracks on topic " << ros::names::remap(inputTopic) << " as occluded and publishing them to output topic "
        << ros::names::remap(outputTopic) << "!");
    ros::spin();
}
