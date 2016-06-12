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
#include <spencer_tracking_msgs/DetectedPersons.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <list>
#include <set>

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <cmath>

using namespace spencer_tracking_msgs;

ros::Publisher g_resultingDetectedPersonsPublisher;
boost::shared_ptr<tf::TransformListener> g_transformListener;
std::vector<std::string> g_sensorFrames;
double g_sensorMaxRange, g_horizontalFovDegrees;

bool g_hasWaitedForTransform = false;


void newDetectedPersonsReceived(const DetectedPersons::ConstPtr& detectedPersons)
{
    // Prepare output message
    DetectedPersons::Ptr resultingDetectedPersons(new DetectedPersons);
    resultingDetectedPersons->header = detectedPersons->header;

    // Iterate over all sensors (detection must be at least in one sensor's FOV to not be filtered out)
    std::set<unsigned int> passedDetectionIds;
    foreach(std::string sensorFrame, g_sensorFrames)
    {
        // Lookup transform into target frame
        tf::StampedTransform tfTransform;
        if(!sensorFrame.empty()) {
            try {
                g_transformListener->waitForTransform(sensorFrame, detectedPersons->header.frame_id, detectedPersons->header.stamp, ros::Duration(g_hasWaitedForTransform ? 0.05 : 3.0));
                g_hasWaitedForTransform = true;
                g_transformListener->lookupTransform(sensorFrame, detectedPersons->header.frame_id, detectedPersons->header.stamp, tfTransform);
            }
            catch(tf::TransformException e) {
                ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup failed. Reason: " << e.what());
                return;
            }
        }
        else {
            tfTransform.setData(tf::Transform::getIdentity());
        }

        // First iteration over all detected persons
        foreach(const DetectedPerson& detectedPerson, detectedPersons->detections)
        {
            // Was the person already in another sensor's FOV? Then skip it, since it is already in the set of resulting detections
            if(passedDetectionIds.find(detectedPerson.detection_id) != passedDetectionIds.end()) continue;

            //
            // Temporarily transform person pose into sensor frame to check if within FOV limits
            //

            tf::Pose sourcePose; tf::poseMsgToTF(detectedPerson.pose.pose, sourcePose);
            tf::Pose targetPose = tfTransform * sourcePose;

            // If too far away from sensor, discard detection
            if(hypot(targetPose.getOrigin().x(), targetPose.getOrigin().y()) <= g_sensorMaxRange)
            {
                double personAngle = std::atan2( targetPose.getOrigin().y(), targetPose.getOrigin().x() );

                double fovMinAngle = -0.5 * (g_horizontalFovDegrees / 180.0 * M_PI);
                double fovMaxAngle = +0.5 * (g_horizontalFovDegrees / 180.0 * M_PI);

                if(personAngle >= fovMinAngle && personAngle <= fovMaxAngle) {
                    resultingDetectedPersons->detections.push_back(detectedPerson);
                    passedDetectionIds.insert(detectedPerson.detection_id);
                }
            }
        }
    }

    // Publish resulting detections
    g_resultingDetectedPersonsPublisher.publish(resultingDetectedPersons);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_detections_by_fov");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    privateHandle.param<double>("sensor_max_range", g_sensorMaxRange, 100.0);
    privateHandle.param<double>("horizontal_fov_degrees", g_horizontalFovDegrees, 58.0);
    privateHandle.getParam("sensor_frames", g_sensorFrames);

    std::string inputTopic = "input_detections";
    std::string outputTopic = "output_detections";

    g_transformListener.reset(new tf::TransformListener);

    ros::Subscriber detectedPersonsSubscriber = nodeHandle.subscribe<DetectedPersons>(inputTopic, 3, &newDetectedPersonsReceived);
    g_resultingDetectedPersonsPublisher = nodeHandle.advertise<DetectedPersons>(outputTopic, 3);

    ROS_INFO_STREAM("Reducing horizontal field of view for detected persons on topic " << ros::names::resolve(inputTopic) << " and publishing to output topic " << ros::names::resolve(outputTopic)
        << " to " << g_horizontalFovDegrees << " degrees with a max. sensor range " << g_sensorMaxRange << "m and sensor frames ["
        << boost::algorithm::join(g_sensorFrames, ", ") << "]." );

    ros::spin();
}
