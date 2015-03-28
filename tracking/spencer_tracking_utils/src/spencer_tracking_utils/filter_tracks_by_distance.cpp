#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <spencer_tracking_msgs/TrackedPersons.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

boost::shared_ptr<tf::TransformListener> g_transformListener;
ros::Publisher g_filteredTracksPublisher;
double g_minDistance, g_maxDistance;
std::string g_sensorTargetFrame;
bool g_hasWaitedForTransform = false;


void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons) {
    const double now = trackedPersons->header.stamp.toSec();
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    filteredTracks->header = trackedPersons->header;

    // Lookup transform into target frame
    tf::StampedTransform tfTransform;
    if(!g_sensorTargetFrame.empty()) {
        try {
            g_transformListener->waitForTransform(g_sensorTargetFrame, trackedPersons->header.frame_id, trackedPersons->header.stamp, ros::Duration(g_hasWaitedForTransform ? 0.1 : 3.0));
            g_hasWaitedForTransform = true;
            g_transformListener->lookupTransform(g_sensorTargetFrame, trackedPersons->header.frame_id, trackedPersons->header.stamp, tfTransform);
        }
        catch(tf::TransformException e) {
            ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup failed. Reason: " << e.what());
            return;
        }
    }
    else {
        tfTransform.setData(tf::Transform::getIdentity());
    }


    // Iterate over current tracks
    foreach(const TrackedPerson& trackedPerson, trackedPersons->tracks) {
        //
        // Temporarily transform person pose into a coordinate frame relative to the robot (if specified)
        //

        tf::Pose sourcePose; tf::poseMsgToTF(trackedPerson.pose.pose, sourcePose);
        tf::Pose targetPose = tfTransform * sourcePose;

        double distance = hypot(targetPose.getOrigin().x(), targetPose.getOrigin().y());

        if(distance >= g_minDistance && distance <= g_maxDistance) {
            filteredTracks->tracks.push_back(trackedPerson);
        }
    }

    // Publish filtered tracks
    g_filteredTracksPublisher.publish(filteredTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_tracks_by_distance");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    g_minDistance = 0.0;
    g_maxDistance = std::numeric_limits<double>::infinity();
    g_sensorTargetFrame = "laser_center_link";

    privateHandle.getParam("min_distance", g_minDistance);
    privateHandle.getParam("max_distance", g_maxDistance);
    privateHandle.getParam("sensor_target_frame", g_sensorTargetFrame);

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";

    g_transformListener.reset(new tf::TransformListener);

    ros::Subscriber tracksSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Filtering tracks on topic " << ros::names::remap(inputTopic) << " into output topic " << ros::names::remap(outputTopic)
        << ", will pass through tracks above minimum distance " << g_minDistance << " m and below maximum distance " << g_maxDistance << " m!");
    ros::spin();
}
