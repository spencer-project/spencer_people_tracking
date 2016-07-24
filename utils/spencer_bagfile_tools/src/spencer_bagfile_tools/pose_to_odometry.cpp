/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Timm Linder, Social Robotics Lab, University of Freiburg
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

/*
 * Re-publishes a PoseWithCovariance topic as nav_msgs/Odometry, and optionally also a
 * odom-->base_link tf transform. E.g. for use with hector_mapping, which only publishes a pose.
 *
 *  Created on: October 10, 2013
 *      Author: linder
 */

#include <memory>
#include <string>
#include <cstdio>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;
using namespace boost;

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

auto_ptr<ros::Publisher> g_odometryPublisher, g_additionalOdometryDataPublisher;
auto_ptr<tf::TransformBroadcaster> g_transformBroadcaster;

bool g_publishTF, g_stamped;
string g_odomFrame, g_baseFrame;

geometry_msgs::PoseWithCovariance g_lastPose;
ros::Time g_lastPoseGeneratedAt;
mutex g_lastPoseMutex;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void processPoseWithCovariance(const geometry_msgs::PoseWithCovariance* poseWithCovariance, ros::Time poseGeneratedAt)
{
	// Lock to make access to g_lastPose atomic
	{
		lock_guard<mutex> lock(g_lastPoseMutex);
    
		nav_msgs::Odometry odom;
		odom.header.stamp = poseGeneratedAt;
		odom.header.frame_id = g_odomFrame;
		odom.child_frame_id = g_baseFrame;

		// getPitch() and getRoll() do not exist yet
		double currentYaw 	= tf::getYaw(poseWithCovariance->pose.orientation);
		double currentPitch = 0; //tf::getPitch(poseWithCovariance->pose.orientation);
		double currentRoll 	= 0; //tf::getRoll(poseWithCovariance->pose.orientation);

		double lastYaw 		= tf::getYaw(g_lastPose.pose.orientation);
		double lastPitch 	= 0; //tf::getPitch(g_lastPose.pose.orientation);
		double lastRoll 	= 0; //tf::getRoll(g_lastPose.pose.orientation);

		ros::Duration elapsedTime = poseGeneratedAt - g_lastPoseGeneratedAt;		
		odom.pose = *poseWithCovariance;

		odom.twist.twist.linear.x = (poseWithCovariance->pose.position.x - g_lastPose.pose.position.x) / elapsedTime.toSec();
		odom.twist.twist.linear.y = (poseWithCovariance->pose.position.y - g_lastPose.pose.position.y) / elapsedTime.toSec();
		odom.twist.twist.linear.z = (poseWithCovariance->pose.position.z - g_lastPose.pose.position.z) / elapsedTime.toSec();
		odom.twist.twist.angular.x = (currentRoll - lastRoll) / elapsedTime.toSec();
		odom.twist.twist.angular.y = (currentPitch - lastPitch) / elapsedTime.toSec();
		odom.twist.twist.angular.z = (currentYaw - lastYaw) / elapsedTime.toSec();
		odom.twist.covariance = odom.pose.covariance; // FIXME: Is this a good idea?

		// publish the message
		g_odometryPublisher->publish(odom);

		g_lastPose = *poseWithCovariance;
		g_lastPoseGeneratedAt = poseGeneratedAt;
	}

	// Publish TF transform if requested
  	if(g_publishTF) {
		geometry_msgs::TransformStamped odomTF;
		odomTF.header.stamp = poseGeneratedAt;
		odomTF.header.frame_id = g_odomFrame;
		odomTF.child_frame_id = g_baseFrame;

		odomTF.transform.translation.x = poseWithCovariance->pose.position.x;
		odomTF.transform.translation.y = poseWithCovariance->pose.position.y;
		odomTF.transform.translation.z = poseWithCovariance->pose.position.z;
		odomTF.transform.rotation = poseWithCovariance->pose.orientation;

		// send the transform
		g_transformBroadcaster->sendTransform(odomTF);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void poseWithCovarianceCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& poseWithCovariance) {
	processPoseWithCovariance(&*poseWithCovariance, ros::Time::now());
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseWithCovarianceStamped) {
	processPoseWithCovariance(&(poseWithCovarianceStamped->pose), poseWithCovarianceStamped->header.stamp);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	if(argc == 2 && (string(argv[1]) == "help" || string(argv[1]) == "--help")) {
	cout << "Re-publishes a PoseWithCovariance topic as nav_msgs/Odometry, and optionally also a odom-->base_link tf transform." << endl;
	cout << " E.g. for use with hector_mapping, which only publishes a pose." << endl << endl;
	cout << "Subscribes to /poseWithCovariance, publishes to /odom." << endl;
	}

	ros::init(argc, argv, "pose_to_odometry");
	ros::NodeHandle rootNode("");
	ros::NodeHandle nodeHandle("~");

	ROS_INFO("Starting application...");
	ROS_INFO("Processing parameters...");

	// Process parameters	
	nodeHandle.param("stamped", g_stamped, true);
	nodeHandle.param("publish_tf", g_publishTF, false);
	nodeHandle.param<string>("base_frame", g_baseFrame, "base_link");
	nodeHandle.param<string>("odom_frame", g_odomFrame, "odom");

	// Create subscriber	
	ROS_INFO_STREAM("Creating " << (g_stamped ? "PoseWithCovarianceStamped" : "PoseWithCovariance") << " subscriber...");
	if(g_publishTF) ROS_INFO_STREAM("Will publish TF transform from " << g_odomFrame << " to " << g_baseFrame);
	
	ros::Subscriber poseWithCovarianceSubscriber;
  	if(g_stamped)
  		poseWithCovarianceSubscriber = rootNode.subscribe<geometry_msgs::PoseWithCovarianceStamped>("poseWithCovarianceStamped", 100, &poseWithCovarianceStampedCallback);
  	else
 		poseWithCovarianceSubscriber = rootNode.subscribe<geometry_msgs::PoseWithCovariance>("poseWithCovariance", 100, &poseWithCovarianceCallback);

	// Create publishers and broadcasters
	g_odometryPublisher = auto_ptr<ros::Publisher>( new ros::Publisher(rootNode.advertise<nav_msgs::Odometry>("odom", 100) ));
	g_transformBroadcaster = auto_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

	ros::spin();
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
