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

#include <map>
#include <boost/circular_buffer.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;
using namespace boost::adaptors;


struct StampedPosition {
    double stamp;
    float x, y;
};

struct TrackRecord {
    bool approved;
    int notSeenForNumberOfFrames;
    boost::circular_buffer<StampedPosition> positionHistory;

    TrackRecord() : approved(false), notSeenForNumberOfFrames(0), positionHistory(1) {}
};

typedef unsigned int track_id;

std::map<track_id, TrackRecord> g_trackRecords;
ros::Publisher g_filteredTracksPublisher;
int g_numFramesToObserve, g_deleteUnseenTracksAfterNumFrames;
double g_maxTimespanToObserve;
double g_minRequiredAvgVelocity;
bool g_rememberTracksWhichOnceMoved;


void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons) {
    const double now = trackedPersons->header.stamp.toSec();
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    filteredTracks->header = trackedPersons->header;

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
        std::map<track_id, TrackRecord>::iterator trackRecordIt = g_trackRecords.find(trackedPerson.track_id);
        if(trackRecordIt == g_trackRecords.end()) {
            // Track is seen for the first time
            TrackRecord newRecord;
            newRecord.approved = false;
            newRecord.notSeenForNumberOfFrames = 0;
            newRecord.positionHistory = boost::circular_buffer<StampedPosition>(g_numFramesToObserve);
            g_trackRecords[trackedPerson.track_id] = newRecord;
        }
        else {
            TrackRecord& existingRecord = trackRecordIt->second;
            existingRecord.notSeenForNumberOfFrames = 0;

            if(existingRecord.approved) {
                // Track is approved -- copy it into the output message.
                filteredTracks->tracks.push_back(trackedPerson);
            }
            else {
                // Delete ancient position history
                while(!existingRecord.positionHistory.empty() && now - existingRecord.positionHistory.front().stamp > g_maxTimespanToObserve)
                    existingRecord.positionHistory.pop_front();

                // Track is not yet approved -- filter it out for the moment, and record its positions to see if it moved.
                // Only do this for non-occluded tracks that are backed by a detection.
                if(!trackedPerson.is_occluded || !g_rememberTracksWhichOnceMoved) {
                    StampedPosition stampedPosition;
                    stampedPosition.stamp = now;
                    stampedPosition.x = trackedPerson.pose.pose.position.x; // assumes x, y are groundplane coordinates (e.g. frame ID is base_footprint, odom, etc.)
                    stampedPosition.y = trackedPerson.pose.pose.position.y;
                    existingRecord.positionHistory.push_back(stampedPosition);

                    if(existingRecord.positionHistory.size() >= g_numFramesToObserve) {
                        StampedPosition& oldestPosition = existingRecord.positionHistory.front();
                        StampedPosition& newestPosition = existingRecord.positionHistory.back();

                        double dt = newestPosition.stamp - oldestPosition.stamp;
                        float dx = newestPosition.x - oldestPosition.x;
                        float dy = newestPosition.y - oldestPosition.y;
                        float ds = sqrtf(dx*dx + dy*dy);

                        if(ds / dt >= g_minRequiredAvgVelocity) {
                            // Approve track
                            existingRecord.approved = g_rememberTracksWhichOnceMoved;
                            if(existingRecord.approved) existingRecord.positionHistory.clear(); // not needed anymore since it's approved now
                            filteredTracks->tracks.push_back(trackedPerson);
                        }
                    }
                }
            }
        }
    }

    // Remove tracks that haven't been seen for too long
    foreach(track_id trackId, unseenTrackIds) {
        TrackRecord& trackRecord = g_trackRecords[trackId];
        if(trackRecord.notSeenForNumberOfFrames++ > g_deleteUnseenTracksAfterNumFrames) {
            g_trackRecords.erase(trackId);
        }
    }

    // Publish filtered tracks
    g_filteredTracksPublisher.publish(filteredTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_non_moving_targets");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    g_numFramesToObserve = 35 /* Hz */ * 1.5 /* seconds */ * 0.5 /* assumed occlusion probability */;  // NOTE that we only observe tracks in frames where they are not occluded!
    g_maxTimespanToObserve = 5.0; // seconds after which to delete old positions from history. Useful if track is seen only once, but then occluded thereafter. Should be larger than g_numFramesToObserve / expectedHz
    g_deleteUnseenTracksAfterNumFrames = 10;
    g_minRequiredAvgVelocity = 0.35;
    g_rememberTracksWhichOnceMoved = true;

    privateHandle.getParam("num_frames_to_observe", g_numFramesToObserve);
    privateHandle.getParam("max_timespan_to_observe", g_maxTimespanToObserve);
    privateHandle.getParam("delete_unseen_tracks_after_num_frames", g_deleteUnseenTracksAfterNumFrames);
    privateHandle.getParam("min_required_avg_velocity", g_minRequiredAvgVelocity);
    privateHandle.getParam("remember_tracks_which_once_moved", g_rememberTracksWhichOnceMoved);

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";

    ros::Subscriber tracksSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Filtering tracks on topic " << ros::names::remap(inputTopic) << " into output topic " << ros::names::remap(outputTopic)
        << ". Will observe input tracks for " << g_numFramesToObserve << " frames (but not more than " << g_maxTimespanToObserve << " seconds) and pass-through any tracks which "
        << "between first and last observed frame have an average velocity of at least " << g_minRequiredAvgVelocity << " m/s. Tracks not seen for more than " << g_deleteUnseenTracksAfterNumFrames << " frames will be deleted from cache.");

    ros::spin();
}
