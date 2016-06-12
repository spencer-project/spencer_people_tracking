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
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <list>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

// Parameters
double g_personRadius, g_minPercentFree, g_mapTimeout;
bool g_permitAllDetectionsIfNoMap, g_permitDetectionsOutsideOfMap;
std::set<std::string> g_modalitiesToAlwaysPassthrough;

// ROS stuff
ros::Publisher g_resultingDetectedPersonsPublisher, g_debugMapPublisher;
boost::shared_ptr<tf::TransformListener> g_transformListener;
std::string g_mapTopic;

// State
nav_msgs::OccupancyGrid g_map;
bool g_mapTimeoutAnnounced = false;
bool g_hasWaitedForTransform = false;
ros::Time g_nodeStartedAt;


void newDetectedPersonsReceived(const DetectedPersons::ConstPtr& detectedPersons)
{
    // This map will track all visited cells, to visually check if transforms are correct
    nav_msgs::OccupancyGrid debugMap;
    bool createDebugMap = g_debugMapPublisher.getNumSubscribers() > 0;
    
    // Check if we have a map that we can safely work with
    bool gotAnyMap = g_map.info.width * g_map.info.height > 0;
    double timeOffset = (detectedPersons->header.stamp - g_map.header.stamp).toSec();
    bool isMapUpToDate = std::abs(timeOffset) < g_mapTimeout;
    
    DetectedPersons::Ptr resultingDetectedPersons(new DetectedPersons);
            
    if(!gotAnyMap || !isMapUpToDate) {
        if(g_permitAllDetectionsIfNoMap) {
            // Pass-through all
            *resultingDetectedPersons = *detectedPersons;
        }
        else {
            // Pass-through none
            resultingDetectedPersons->header = detectedPersons->header;
        }

        // Logging
        if(!g_mapTimeoutAnnounced && ros::Time::now() - g_nodeStartedAt > ros::Duration(5.0)) {
            ROS_WARN_STREAM("Occupancy grid map on topic " << g_mapTopic << " timed out (or never received) by filter_detections_by_static_map! Will "
                << (g_permitAllDetectionsIfNoMap ? "pass through" : "filter out") << " all incoming detections until a map is received! Time offset was " << timeOffset
                << ", but timeout is " << g_mapTimeout << " sec!");
            g_mapTimeoutAnnounced = true;
        }
    }
    else {
        // Reset timeout flag
        g_mapTimeoutAnnounced = false;

        // Create debug map
        if(createDebugMap) {
            debugMap = g_map;
            debugMap.header.stamp = detectedPersons->header.stamp;
            for(int x = 0; x < g_map.info.width; x++) {
                for(int y = 0; y < g_map.info.height; y++) {
                    debugMap.data[y * g_map.info.width + x] = -1;
                }
            }
        }


        // Prepare output message
        resultingDetectedPersons->header = detectedPersons->header;
        
        // Lookup transform into target frame
        tf::StampedTransform tfTransform;
        try {
            std::string sensorTargetFrame = g_map.header.frame_id;
            g_transformListener->waitForTransform(sensorTargetFrame, detectedPersons->header.frame_id, detectedPersons->header.stamp, ros::Duration(0.05) /*g_hasWaitedForTransform ? 0.05 : 1.5)*/);
            g_hasWaitedForTransform = true;
            g_transformListener->lookupTransform(sensorTargetFrame, detectedPersons->header.frame_id, detectedPersons->header.stamp, tfTransform);
        }
        catch(tf::TransformException e) {
            ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup failed. Reason: " << e.what());
            return;
        }

        // First iteration over all detected persons
        foreach(const DetectedPerson& detectedPerson, detectedPersons->detections)
        {
            //
            // Apply filter only to specific modalities
            //

            if(!detectedPerson.modality.empty() && !g_modalitiesToAlwaysPassthrough.empty())
            {
                bool passThrough = false;
                boost::char_separator<char> modalitySeparator(",");
                boost::tokenizer< boost::char_separator<char> > modalities(detectedPerson.modality, modalitySeparator);

                foreach(std::string modality, modalities) {
                    boost::algorithm::trim(modality);
                    if(g_modalitiesToAlwaysPassthrough.find(modality) != g_modalitiesToAlwaysPassthrough.end()) {
                        passThrough = true;
                        break;
                    }
                }

                if(passThrough) {
                    resultingDetectedPersons->detections.push_back(detectedPerson);
                    continue;
                }
            }


            //
            // Check if detection coincides with a static obstacle on the map
            //

            // Transform person pose into coordinate frame of map
            tf::Pose sourcePose; tf::poseMsgToTF(detectedPerson.pose.pose, sourcePose);
            tf::Pose targetPose = tfTransform * sourcePose;

            // Transform with regard to map origin
            tf::Pose mapOrigin; tf::poseMsgToTF(g_map.info.origin, mapOrigin);
            tf::Pose finalPose = mapOrigin.inverseTimes(targetPose);

            // Check all cells in a circle at the person's pose
            int originX = finalPose.getOrigin().x() / g_map.info.resolution;
            int originY = finalPose.getOrigin().y() / g_map.info.resolution;

            int radius = (int) std::ceil(g_personRadius / g_map.info.resolution);
            int numVisited = 0, numFree = 0;

            for(int y = -radius; y <= radius; y++) {
                for(int x = -radius; x <= radius; x++) {
                    if(x*x + y*y <= radius * radius) {
                        int cellX = originX + x;
                        int cellY = originY + y;

                        if(cellX < 0 || cellX >= g_map.info.width) continue;
                        if(cellY < 0 || cellY >= g_map.info.height) continue;

                        bool cellFree = g_map.data[cellY * g_map.info.width + cellX] < 100;
                        
                        numVisited++;
                        if(cellFree) numFree++;

                        if(createDebugMap) {
                            debugMap.data[cellY * g_map.info.width + cellX] = cellFree ? 0 : 254;
                        }
                    }
                }
            }

            bool outsideOfMap = numVisited == 0;
            if(outsideOfMap && !g_permitDetectionsOutsideOfMap) continue;

            double percentFree = (double)numFree / numVisited;
            if(percentFree > g_minPercentFree)
            {
                resultingDetectedPersons->detections.push_back(detectedPerson);
            }
        }
    }

    // Publish resulting detections
    g_resultingDetectedPersonsPublisher.publish(resultingDetectedPersons);

    // Publish debug map of visited cells
    if(createDebugMap) g_debugMapPublisher.publish(debugMap);
}


void newMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    // Store map
    g_map = *map;
    ROS_WARN_STREAM_ONCE("Received first occupancy grid map! Will from now on filter all detections using static map on " << g_mapTopic << " topic!");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_detections_by_static_map");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    int queueSize;
    std::vector<std::string> modalitiesToAlwaysPassthrough;
    privateHandle.param<double>("person_radius", g_personRadius, 0.15);
    privateHandle.param<double>("min_percent_free", g_minPercentFree, 0.9);
    privateHandle.param<double>("map_timeout", g_mapTimeout, 999999999999);
    privateHandle.param<int>("queue_size", queueSize, 3);
    privateHandle.param<bool>("permit_all_detections_if_no_map", g_permitAllDetectionsIfNoMap, true);
    privateHandle.param<bool>("permit_detections_outside_of_map", g_permitDetectionsOutsideOfMap, true);
    privateHandle.getParam("modalities_to_always_passthrough", modalitiesToAlwaysPassthrough);

    foreach(std::string modality, modalitiesToAlwaysPassthrough) {
        boost::algorithm::trim(modality);
        g_modalitiesToAlwaysPassthrough.insert(modality);
    }

    std::string inputTopic = ros::names::resolve("input_detections");
    std::string outputTopic = ros::names::resolve("output_detections");
    g_mapTopic = ros::names::resolve("/map");
    
    g_transformListener.reset(new tf::TransformListener);

    ros::spinOnce();
    g_nodeStartedAt = ros::Time::now();


    ros::Subscriber detectedPersonsSubscriber = nodeHandle.subscribe<DetectedPersons>(inputTopic, queueSize, &newDetectedPersonsReceived);
    g_resultingDetectedPersonsPublisher = nodeHandle.advertise<DetectedPersons>(outputTopic, queueSize);

    ros::Subscriber mapSubscriber = nodeHandle.subscribe<nav_msgs::OccupancyGrid>(g_mapTopic, 1, &newMapReceived); // always use minimum queue size for map
    g_debugMapPublisher = privateHandle.advertise<nav_msgs::OccupancyGrid>("debug_map", 1);


    ROS_INFO_STREAM("Filtering detected persons on topic " << inputTopic << " and publishing to output topic " << outputTopic
        << " by excluding false positives from map on topic " << g_mapTopic << " with a person radius of " << g_personRadius << "m.");

    ros::spin();
}
