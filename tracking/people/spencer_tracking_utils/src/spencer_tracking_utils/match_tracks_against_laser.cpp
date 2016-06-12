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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include <cmath>
#include <map>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

boost::shared_ptr<tf::TransformListener> g_transformListener;
ros::Publisher g_filteredTracksPublisher;
bool g_hasWaitedForTransform = false;
double g_personRadius, g_shiftOfPersonCenterTowardsSensor;
int g_minPointsForMatch;


void newTrackedPersonsAndLaserScanReceived(const TrackedPersons::ConstPtr& trackedPersons, const sensor_msgs::LaserScan::ConstPtr& laserScan) {
    const double now = trackedPersons->header.stamp.toSec();
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    filteredTracks->header = trackedPersons->header;

    // Lookup transform into target frame (laser sensor origin)
    tf::StampedTransform tfTransform;
    try {
        ros::Time meanTime( (trackedPersons->header.stamp.toSec() + laserScan->header.stamp.toSec()) / 2.0 );
        g_transformListener->waitForTransform(laserScan->header.frame_id, trackedPersons->header.frame_id, meanTime, ros::Duration(g_hasWaitedForTransform ? 0.1 : 0.5));
        g_hasWaitedForTransform = true;
        g_transformListener->lookupTransform(laserScan->header.frame_id, trackedPersons->header.frame_id, meanTime, tfTransform);
    }
    catch(tf::TransformException e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup failed. Reason: " << e.what());
        return;
    }

    // Ensure track IDs are unique and initialize number of laser matches per track
    typedef unsigned int track_id;
    std::map<track_id, int> numMatchesPerTrack;
    foreach(const TrackedPerson& trackedPerson, trackedPersons->tracks)
    {
        ROS_ASSERT_MSG(numMatchesPerTrack.find(trackedPerson.track_id) == numMatchesPerTrack.end(), "Incoming track IDs are not unique!");
        numMatchesPerTrack[trackedPerson.track_id] = 0;
        filteredTracks->tracks.push_back(trackedPerson);
    }
    
    // Iterate over all laser points and try to match them against tracks
    for(size_t pointIndex = 0; pointIndex < laserScan->ranges.size(); pointIndex++)
    {
        // Get polar coordinates of this point
        double phi = laserScan->angle_min + laserScan->angle_increment * pointIndex;
        double rho = laserScan->ranges[pointIndex];

        // Skip out-of-range measurements
        if(rho > laserScan->range_max || rho < laserScan->range_min) continue;

        // Convert laser point into Cartesian coordinates
        double laserX = cos(phi) * rho;
        double laserY = sin(phi) * rho;

        // Try to match any person against this point
        foreach(TrackedPerson& trackedPerson, filteredTracks->tracks)
        {
            // Skip person if already matched
            bool done = !trackedPerson.is_occluded;
            bool pointMatched = false;

            if(!done) {
                //
                // Transform person pose into coordinate frame of laser scanner
                //

                tf::Pose sourcePose; tf::poseMsgToTF(trackedPerson.pose.pose, sourcePose);
                tf::Pose targetPose = tfTransform * sourcePose;

                double personX = targetPose.getOrigin().x();
                double personY = targetPose.getOrigin().y();

                double normalizer = hypot(personX, personY);
                double personDirectionX = personX / normalizer;
                double personDirectionY = personY / normalizer;

                personX -= personDirectionX * g_shiftOfPersonCenterTowardsSensor;
                personY -= personDirectionY * g_shiftOfPersonCenterTowardsSensor;

                // Check distance of laser point to person center
                double distance = hypot(laserX - personX, laserY - personY);
                if(distance <= g_personRadius) {
                    pointMatched = true; // mark point as matched
                    if(++numMatchesPerTrack[trackedPerson.track_id] >= g_minPointsForMatch) {
                        trackedPerson.is_occluded = false; // mark person as matched
                        trackedPerson.is_matched = true; // not sure if this makes sense, as detection ID is invalid, but it's only a groundtruth anyway
                    }
                }

            }

            if(pointMatched) break; // stop matching this point against other persons
        }
    }

    // Publish filtered tracks
    g_filteredTracksPublisher.publish(filteredTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "match_tracks_against_laser");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    g_minPointsForMatch = 3;
    g_personRadius = 0.3; // in meters
    g_shiftOfPersonCenterTowardsSensor = 0.25; // in meters

    privateHandle.getParam("min_points_for_match", g_minPointsForMatch);
    privateHandle.getParam("person_radius", g_personRadius);
    privateHandle.getParam("shift_of_person_center_towards_sensor", g_shiftOfPersonCenterTowardsSensor);

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";
    std::string laserTopic = "laser";

    g_transformListener.reset(new tf::TransformListener);

    // Create publishers
    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, 5);
  
    // Create subscriber filters for approximate-time synchronizer
    typedef message_filters::sync_policies::ApproximateTime<spencer_tracking_msgs::TrackedPersons, sensor_msgs::LaserScan> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Subscriber<spencer_tracking_msgs::TrackedPersons> TrackedPersonsSubscriberType;
    typedef message_filters::Subscriber<sensor_msgs::LaserScan> LaserSubscriberType;
    
    TrackedPersonsSubscriberType trackedPersonsSubscriber(nodeHandle, inputTopic, 1);
    LaserSubscriberType laserSubscriber(nodeHandle, laserTopic, 1);
    
    // Create approximate-time synchronizer
    int agePenalty = 0.1; // Could set higher age penalty to publish older data faster even if it might not be correctly synchronized.
    privateHandle.getParam("synchronizer_age_penalty", agePenalty);

    int queueSize = 5;
    privateHandle.getParam("synchronizer_queue_size", queueSize);        

    SyncPolicy syncPolicy(queueSize);
    syncPolicy.setAgePenalty(agePenalty); 
    const SyncPolicy constSyncPolicy = syncPolicy;

    Synchronizer synchronizer(constSyncPolicy, trackedPersonsSubscriber, laserSubscriber);
    synchronizer.registerCallback(&newTrackedPersonsAndLaserScanReceived);

    ROS_INFO_STREAM("Publishing tracks from topic " << ros::names::remap(inputTopic) << " into output topic " << ros::names::remap(outputTopic)
        << ", simulating occlusions by listening to laserscans at " << ros::names::remap(laserTopic) << "!");
    ros::spin();
}
