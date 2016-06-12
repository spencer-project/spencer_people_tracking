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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <spencer_tracking_msgs/TrackedPersons.h>

#include <algorithm>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

ros::Publisher g_filteredTracksPublisher;
std::string g_logicOp;


/// Applies a logical (boolean) filter to two sets of TrackedPersons, by comparing their IDs only.
/// Useful for combining the results of two other filters in a logical OR or AND fashion (without averaging any positions, etc.)
void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons1, const TrackedPersons::ConstPtr& trackedPersons2) {
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    ROS_ASSERT_MSG(trackedPersons1->header.frame_id == trackedPersons2->header.frame_id, "Input topics must have identical header.frame_ids!");
    filteredTracks->header = trackedPersons1->header;

    if(g_logicOp == "OR") {
        std::set<uint64_t> existingIDs;
        foreach(const TrackedPerson& trackedPerson1, trackedPersons1->tracks) {
            filteredTracks->tracks.push_back(trackedPerson1);
            existingIDs.insert(trackedPerson1.track_id);       
        }
        foreach(const TrackedPerson& trackedPerson2, trackedPersons2->tracks) {
            if(existingIDs.find(trackedPerson2.track_id) == existingIDs.end()) filteredTracks->tracks.push_back(trackedPerson2);
        }

    }
    else if(g_logicOp == "AND") {
        std::set<uint64_t> existingIDs;
        foreach(const TrackedPerson& trackedPerson1, trackedPersons1->tracks) {
            existingIDs.insert(trackedPerson1.track_id);       
        }
        foreach(const TrackedPerson& trackedPerson2, trackedPersons2->tracks) {
            if(existingIDs.find(trackedPerson2.track_id) != existingIDs.end()) {
                filteredTracks->tracks.push_back(trackedPerson2);
            }
        }
    }
    else if(g_logicOp == "XOR") {
        std::set<uint64_t> existingIDs1, existingIDs2;
        foreach(const TrackedPerson& trackedPerson1, trackedPersons1->tracks) {
            existingIDs1.insert(trackedPerson1.track_id);       
        }
        foreach(const TrackedPerson& trackedPerson2, trackedPersons2->tracks) {
            if(existingIDs1.find(trackedPerson2.track_id) == existingIDs1.end()) {
                filteredTracks->tracks.push_back(trackedPerson2);
            }
            existingIDs2.insert(trackedPerson2.track_id);       
        }
        foreach(const TrackedPerson& trackedPerson1, trackedPersons1->tracks) {
            if(existingIDs2.find(trackedPerson1.track_id) == existingIDs2.end()) {
                filteredTracks->tracks.push_back(trackedPerson1);
            }
        }
    }

    // Publish filtered tracks
    g_filteredTracksPublisher.publish(filteredTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "logical_track_filter");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    g_logicOp = "OR";
    privateHandle.getParam("logic_op", g_logicOp);
    std::transform(g_logicOp.begin(), g_logicOp.end(), g_logicOp.begin(), ::toupper);
    ROS_ASSERT_MSG(g_logicOp == "AND" || g_logicOp == "OR" || g_logicOp == "XOR", "Invalid logical operation!");

    std::string inputTopic1 = "input_tracks1";
    std::string inputTopic2 = "input_tracks2";
    std::string outputTopic = "output_tracks";

    typedef message_filters::sync_policies::ExactTime<TrackedPersons, TrackedPersons> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    size_t queue_size = 5;
    message_filters::Subscriber<TrackedPersons> trackSubscriber1(nodeHandle, inputTopic1, queue_size);
    message_filters::Subscriber<TrackedPersons> trackSubscriber2(nodeHandle, inputTopic2, queue_size);

    Synchronizer inputSynchronizer(SyncPolicy(queue_size), trackSubscriber1, trackSubscriber2);
    inputSynchronizer.registerCallback(&newTrackedPersonsReceived);

    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, queue_size);

    ROS_INFO_STREAM("Applying logical " << g_logicOp << " filter (based upon track IDs) to tracks on topics " << ros::names::remap(inputTopic1) << " and " << ros::names::remap(inputTopic2)
        << ", using output topic " << ros::names::remap(outputTopic) << "!");
    ros::spin();
}
