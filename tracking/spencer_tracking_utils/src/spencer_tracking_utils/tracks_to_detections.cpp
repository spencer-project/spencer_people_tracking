#include <ros/ros.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/DetectedPersons.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

ros::Publisher g_detectedPersonsPublisher;
unsigned int g_currentDetectionId = 0;
bool g_overridePoseCovariance;
double g_poseVariance;


void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons) {
    DetectedPersons::Ptr detectedPersons(new DetectedPersons);
    detectedPersons->header = trackedPersons->header;

    // Iterate over tracked persons and convert each one into a detected person, if not occluded.
    foreach(const TrackedPerson& trackedPerson, trackedPersons->tracks) {
        if(!trackedPerson.is_occluded) {
            DetectedPerson detectedPerson;
            detectedPerson.detection_id = g_currentDetectionId++;
            detectedPerson.confidence = 1.0;
            detectedPerson.pose = trackedPerson.pose;

            if(g_overridePoseCovariance) {
                // We assume that x, y are groundplane coordinates (e.g. base_footprint, base_link or odom frame)
                const double LARGE_VARIANCE = 999999999;
                for(size_t d = 0; d < 2; d++) detectedPerson.pose.covariance[d*6 + d] = g_poseVariance;
                for(size_t d = 2; d < 6; d++) detectedPerson.pose.covariance[d*6 + d] = LARGE_VARIANCE;  
            }

            detectedPersons->detections.push_back(detectedPerson);
        }
    }

    // Publish resulting detected persons
    g_detectedPersonsPublisher.publish(detectedPersons);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracks_to_detections");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    std::string inputTopic = "/spencer/perception/tracked_persons";
    std::string outputTopic = "/spencer/perception/detected_persons";

    privateHandle.param<bool>("override_pose_covariance", g_overridePoseCovariance, true);
    privateHandle.param<double>("pose_variance", g_poseVariance, 0.01); // only used if override_pose_covariance is true

    ros::Subscriber trackedPersonsSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_detectedPersonsPublisher = nodeHandle.advertise<DetectedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Publishing tracked persons from topic " << ros::names::resolve(inputTopic) << " as detected persons on topic " << ros::names::resolve(outputTopic) << ".");
    ros::spin();
}
