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
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <map>
#include <boost/circular_buffer.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;
using namespace boost::adaptors;

struct TrackRecord {
    int notSeenForNumberOfFrames;
    geometry_msgs::Quaternion lastKnownOrientation;
    bool hasLastKnownOrientation;

    TrackRecord() : notSeenForNumberOfFrames(0), hasLastKnownOrientation(false) {}
};

typedef unsigned int track_id;

std::map<track_id, TrackRecord> g_trackRecords;
ros::Publisher g_resultingTracksPublisher;
boost::shared_ptr<tf::TransformListener> g_transformListener;
int g_deleteUnseenTracksAfterNumFrames;
double g_minRequiredAvgVelocity;
bool waitedForTransform = false;


double getSpeed(const TrackedPerson& trackedPerson) {
    return hypot(trackedPerson.twist.twist.linear.x, trackedPerson.twist.twist.linear.y);
}


void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons) {
    const double now = trackedPersons->header.stamp.toSec();
    TrackedPersons::Ptr resultingTracks(new TrackedPersons);
    resultingTracks->header = trackedPersons->header;

    // Initially mark all known tracks as "unseen"
    std::set<track_id> unseenTrackIds;
    foreach(track_id trackId, g_trackRecords | map_keys) {
        unseenTrackIds.insert(trackId);
    }

    // Iterate over current tracks
    foreach(const TrackedPerson& trackedPerson, trackedPersons->tracks) {
        // Mark track as "seen"
        unseenTrackIds.erase(trackedPerson.track_id);

        // Check if we "know" the track (from its ID)
        TrackRecord* trackRecord;
        std::map<track_id, TrackRecord>::iterator trackRecordIt = g_trackRecords.find(trackedPerson.track_id);
        if(trackRecordIt == g_trackRecords.end()) {
            // Track is seen for the first time
            g_trackRecords[trackedPerson.track_id] = TrackRecord();
            trackRecord = &g_trackRecords[trackedPerson.track_id];
        }
        else trackRecord = &trackRecordIt->second;

        // Check if track is moving fast enough to determine orientation
        TrackedPerson resultingTrackedPerson = trackedPerson;
        if(getSpeed(trackedPerson) > g_minRequiredAvgVelocity) {
            // Speed is fine, no need to override orientation. Just save it in case we need it later on.
            trackRecord->lastKnownOrientation = trackedPerson.pose.pose.orientation;
            trackRecord->hasLastKnownOrientation = true;
        }
        else {
            // Person is moving too slow. Check if we have previously stored its orientation.
            if(trackRecord->hasLastKnownOrientation) {
                resultingTrackedPerson.pose.pose.orientation = trackRecord->lastKnownOrientation;
            }
            else {
                // No previous orientation available. Just assume that the person is facing the sensor.

                // Transform person pose into a coordinate frame relative to the robot
                geometry_msgs::PoseStamped poseInSourceFrame, poseInTargetFrame;
                poseInSourceFrame.header = trackedPersons->header;
                poseInSourceFrame.pose = trackedPerson.pose.pose;

                const std::string targetFrame = "base_footprint";
                try {
                    if(!waitedForTransform) {
                        g_transformListener->waitForTransform(targetFrame, poseInSourceFrame.header.frame_id, poseInSourceFrame.header.stamp, ros::Duration(3.0));
                        waitedForTransform = true;
                    }
                    g_transformListener->transformPose(targetFrame, poseInSourceFrame, poseInTargetFrame);
                }
                catch(tf::TransformException e) {
                    ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup failed. Reason: " << e.what());
                    continue;
                }

                // Get yaw angle relative to robot
                double yaw = M_PI + std::atan2(poseInTargetFrame.pose.position.y, poseInTargetFrame.pose.position.x);
                resultingTrackedPerson.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
        }

        resultingTracks->tracks.push_back(resultingTrackedPerson);
    }

    // Remove tracks that haven't been seen for too long
    foreach(track_id trackId, unseenTrackIds) {
        TrackRecord& trackRecord = g_trackRecords[trackId];
        if(trackRecord.notSeenForNumberOfFrames++ > g_deleteUnseenTracksAfterNumFrames) {
            g_trackRecords.erase(trackId);
        }
    }

    // Publish filtered tracks
    g_resultingTracksPublisher.publish(resultingTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fix_orientation_of_non_moving_targets");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    g_deleteUnseenTracksAfterNumFrames = 10;
    g_minRequiredAvgVelocity = 0.15;

    privateHandle.getParam("delete_unseen_tracks_after_num_frames", g_deleteUnseenTracksAfterNumFrames);
    privateHandle.getParam("min_required_avg_velocity", g_minRequiredAvgVelocity);

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";

    g_transformListener.reset(new tf::TransformListener);

    ros::Subscriber tracksSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_resultingTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Fixing orientations of tracks on topic " << ros::names::remap(inputTopic) << " and publishing to output topic " << ros::names::remap(outputTopic)
        << ". Orientation of slow-moving targets (slower than " << g_minRequiredAvgVelocity << " m/s) will be set to their last known orientation, or facing towards base_footprint if last known orientation is not available.");

    ros::spin();
}
