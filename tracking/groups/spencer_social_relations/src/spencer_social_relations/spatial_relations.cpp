/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012-2015, Matthias Luber, Timm Linder, Social Robotics Lab, University of Freiburg
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
#include <ros/package.h>

#include <tf/transform_datatypes.h>

#include <svm.h>

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_social_relation_msgs/SocialRelations.h>

#include <cmath>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;
using namespace spencer_social_relation_msgs;

ros::Publisher g_socialRelationsPublisher;

struct svm_model* g_svmModel;
struct svm_node* g_svmNode;
double g_maxDistance, g_maxSpeedDifference, g_maxOrientationDifference, g_minSpeedToConsiderOrientation;


/// Puts angle alpha into the interval [min..min+2*pi[
double set_angle_to_range(double alpha, double min)
{
    while (alpha >= min + 2.0 * M_PI) {
        alpha -= 2.0 * M_PI;
    }
    while (alpha < min) {
        alpha += 2.0 * M_PI;
    }
    return alpha;
}

/// Determines the minimal difference dalpha = alpha1 - alpha2 between two angles alpha1 and alpha2
double diff_angle_unwrap(double alpha1, double alpha2)
{
    double delta;

    // normalize angles alpha1 and alpha2
    alpha1 = set_angle_to_range(alpha1, 0);
    alpha2 = set_angle_to_range(alpha2, 0);

    // take difference and unwrap
    delta = alpha1 - alpha2;
    if (alpha1 > alpha2) {
        while (delta > M_PI) {
            delta -= 2.0 * M_PI;
        }
    } else if (alpha2 > alpha1) {
        while (delta < -M_PI) {
            delta += 2.0 * M_PI;
        }
    }
    return delta;
}

/// Callback when new tracks have arrived. This is where all the magic happens.
void newTrackedPersonsReceived(const TrackedPersons::ConstPtr& trackedPersons)
{
    ROS_ASSERT_MSG(trackedPersons->header.frame_id == "odom" || trackedPersons->header.frame_id == "map", "Input tracks must be in ground-plane (x y) coordinates!");    

    SocialRelations::Ptr socialRelations(new SocialRelations);
    socialRelations->header = trackedPersons->header;

    // TODO: Parallelize, but watch out for SVM node which is not thread-safe (maybe need to create a copy each time!?)
    const size_t trackCount = trackedPersons->tracks.size();
    for(size_t t1Index = 0; t1Index < trackCount; t1Index++)
    {
        for(size_t t2Index = t1Index + 1; t2Index < trackCount; t2Index++)
        {
            // Extract feature values for each pair of tracks
            const TrackedPerson& t1 = trackedPersons->tracks[t1Index];
            const TrackedPerson& t2 = trackedPersons->tracks[t2Index];

            const geometry_msgs::Point& pos1 = t1.pose.pose.position;
            const geometry_msgs::Point& pos2 = t2.pose.pose.position;

            const geometry_msgs::Vector3& v1 = t1.twist.twist.linear;
            const geometry_msgs::Vector3& v2 = t2.twist.twist.linear;

            const double speed1 = hypot(v1.x, v1.y);
            const double speed2 = hypot(v2.x, v2.y);
            
            double theta1 = 0.0;
            double theta2 = 0.0;

            if (speed1 >= g_minSpeedToConsiderOrientation) {
                theta1 = tf::getYaw(t1.pose.pose.orientation);
            }


            if (speed2 >= g_minSpeedToConsiderOrientation) {
                theta2 = tf::getYaw(t2.pose.pose.orientation);
            }

            // Calculate final feature values
            double distance = hypot(pos1.x - pos2.x, pos1.y - pos2.y);
            double deltaspeed = fabs(speed1 - speed2);
            double deltaangle = fabs(diff_angle_unwrap(theta1, theta2));

            double positiveRelationProbability, negativeRelationProbability;
            
            // Gating for large distance, very different velocities, or very different angle
            if (distance > g_maxDistance ||  deltaspeed > g_maxSpeedDifference  || deltaangle > g_maxOrientationDifference) {
                positiveRelationProbability = 0.1;
                negativeRelationProbability = 0.9;

            }
            else {
                // Prepare SVM classifier
                g_svmNode[0].value = distance;
                g_svmNode[1].value = deltaspeed;
                g_svmNode[2].value = deltaangle;

                // Run SVM classifier
                double probabilityEstimates[2];
                svm_predict_probability(g_svmModel, g_svmNode, probabilityEstimates);
                positiveRelationProbability = probabilityEstimates[0];
                negativeRelationProbability = probabilityEstimates[1];
            }

            // Store results for this pair of tracks
            SocialRelation socialRelation;
            socialRelation.type = SocialRelation::TYPE_SPATIAL;
            socialRelation.strength = positiveRelationProbability;
            socialRelation.track1_id = t1.track_id;
            socialRelation.track2_id = t2.track_id;

            socialRelations->elements.push_back(socialRelation);
        }
    }

    // Publish spatial relations
    g_socialRelationsPublisher.publish(socialRelations);
}


/// Main method, creates ROS subscribers and reads parameters
int main(int argc, char **argv)
{
    ros::init(argc, argv, "spatial_relations");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Create publishers and subscribers
    std::string inputTopic = nodeHandle.resolveName("/spencer/perception/tracked_persons");
    std::string outputTopic = nodeHandle.resolveName("/spencer/perception/spatial_relations");

    ros::Subscriber tracksSubscriber = nodeHandle.subscribe<TrackedPersons>(inputTopic, 3, &newTrackedPersonsReceived);
    g_socialRelationsPublisher = nodeHandle.advertise<SocialRelations>(outputTopic, 3);

    // Read parameters
    privateHandle.param<double>("max_distance", g_maxDistance, 3.0);
    privateHandle.param<double>("max_speed_difference", g_maxSpeedDifference, 1.0);
    privateHandle.param<double>("max_orientation_difference", g_maxOrientationDifference, M_PI_4);
    privateHandle.param<double>("min_speed_to_consider_orientation", g_minSpeedToConsiderOrientation, 0.1);
    
    // Initialize SVM
    std::string svmFilename;
    privateHandle.param<std::string>("svm_filename", svmFilename, ros::package::getPath(ROS_PACKAGE_NAME) + "/models/groups_probabilistic_small.model");
    
    if ((g_svmModel = svm_load_model(svmFilename.c_str())) == NULL) {
        ROS_FATAL("Failed to open SVM model %s!", svmFilename.c_str());
        std::terminate();
    }

    ROS_ASSERT(svm_check_probability_model(g_svmModel) != 0); // make sure the provided model is probabilistic

    g_svmNode = new svm_node[4];
    g_svmNode[0].index = 1; // libSVM indices are one-based
    g_svmNode[1].index = 2;
    g_svmNode[2].index = 3;
    g_svmNode[3].index = -1; // terminator

    // Run node, wait for callbacks
    ROS_INFO_STREAM("Subscribing to tracked persons on topic " << ros::names::remap(inputTopic) << " and publishing corresponding spatial relations on output topic " << ros::names::remap(outputTopic) << ".");
    ros::spin();

    // Clean up
    delete[] g_svmNode;
    svm_free_and_destroy_model(&g_svmModel);
}
