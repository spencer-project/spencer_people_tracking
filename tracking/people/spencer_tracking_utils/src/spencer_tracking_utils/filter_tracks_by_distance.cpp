/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
bool g_setToOccluded = false;


void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons) {
    const double now = trackedPersons->header.stamp.toSec();
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    filteredTracks->header = trackedPersons->header;

    // Lookup transform into target frame
    tf::StampedTransform tfTransform;
    if(!g_sensorTargetFrame.empty()) {
        try {
            g_transformListener->waitForTransform(g_sensorTargetFrame, trackedPersons->header.frame_id, trackedPersons->header.stamp, ros::Duration(g_hasWaitedForTransform ? 0.1 : 0.5));
            g_hasWaitedForTransform = true;
            g_transformListener->lookupTransform(g_sensorTargetFrame, trackedPersons->header.frame_id, trackedPersons->header.stamp, tfTransform);
        }
        catch(tf::TransformException e) {
            ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup failed in filter_tracks_by_distance. Reason: " << e.what());
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
        else {
            if(g_setToOccluded) {
                TrackedPerson copyOfTrackedPerson = trackedPerson;
                copyOfTrackedPerson.is_occluded = true;
                copyOfTrackedPerson.is_matched = false;
                filteredTracks->tracks.push_back(copyOfTrackedPerson);
            }
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
    g_setToOccluded = true;

    privateHandle.getParam("min_distance", g_minDistance);
    privateHandle.getParam("max_distance", g_maxDistance);
    privateHandle.getParam("sensor_target_frame", g_sensorTargetFrame);
    privateHandle.getParam("set_to_occluded", g_setToOccluded); // if true, tracks are not filtered out, but just set to occluded if outside of bounds

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";

    g_transformListener.reset(new tf::TransformListener);

    ros::Subscriber tracksSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Filtering tracks on topic " << ros::names::remap(inputTopic) << " into output topic " << ros::names::remap(outputTopic)
        << ", will " << (g_setToOccluded ? "set to occluded" : "filter out") << " all tracks below minimum distance " << g_minDistance << " m or above maximum distance " << g_maxDistance << " m!");
    ros::spin();
}
