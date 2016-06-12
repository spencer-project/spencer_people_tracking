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
