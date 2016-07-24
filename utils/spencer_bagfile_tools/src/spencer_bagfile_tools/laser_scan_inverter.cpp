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
 * Flips an inverted laser scan (where angle_max < angle_min) such that nodes
 * can work with it that do not expect this.
 */

#include <memory>
#include <string>
#include <cstdio>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace boost;

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

shared_ptr<ros::Publisher> g_laserPublisher; 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg)
{
  sensor_msgs::LaserScan invertedScan(*laserScanMsg);

  swap(invertedScan.angle_min, invertedScan.angle_max);
  invertedScan.angle_increment = -invertedScan.angle_increment;

  const size_t numRangeValues = laserScanMsg->ranges.size();
  for(size_t i = 0; i < numRangeValues; i++) {
  	invertedScan.ranges[i] = laserScanMsg->ranges[numRangeValues - i - 1];
  }

  const size_t numIntensityValues = laserScanMsg->intensities.size();
  for(size_t i = 0; i < numIntensityValues; i++) {
  	invertedScan.intensities[i] = laserScanMsg->intensities[numIntensityValues - i - 1];
  } 

  g_laserPublisher->publish(invertedScan);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	if(argc == 2 && (string(argv[1]) == "help" || string(argv[1]) == "--help")) {
	cout << "Converts a flipped laser scan (with max_angle < min_angle) such that it can be processed by laser_scan_matcher" << endl << endl;
	cout << "Subscribes to /laser, publishes to /inverted_laser" << endl;
	}

	ros::init(argc, argv, "laser_scan_inverter");
	ros::NodeHandle nodeHandle("~");

	ROS_INFO("Starting application...");
	ROS_INFO("Processing parameters...");

	// Process parameters


	// Create subscriber	
	ROS_INFO("Creating LaserScan subscriber...");
	ros::Subscriber laserSubscriber;
  	laserSubscriber = nodeHandle.subscribe<sensor_msgs::LaserScan>("laser", 512, &laserCallback);

	// Create output topic
	ROS_INFO("Creating LaserScan publisher...");
	g_laserPublisher = shared_ptr<ros::Publisher>(new ros::Publisher(nodeHandle.advertise<sensor_msgs::LaserScan>("inverted_laser", 512)));

	ros::spin();
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
