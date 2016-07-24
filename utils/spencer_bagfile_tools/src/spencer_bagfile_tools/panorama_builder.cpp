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
 * Merges multiple camera images into a panorama, possibly performing rotations specified by the user
 *
 *  Created on: August 16, 2013
 *      Author: linder
 */

#include <memory>
#include <string>
#include <cstdio>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h> 
#include <message_filters/sync_policies/approximate_time.h>

#define MAX_CAMERAS 4

vector<string> g_topics;
vector<float> g_inplaneRotations;
image_transport::Publisher g_panoramaPublisher;
bool vertical = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        size_t end_pos = start_pos + from.length();
        str.replace(start_pos, end_pos, to);
        start_pos += to.length();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotateImage(const cv::Mat& src, double angle, cv::Mat& dst)
{
	cv::RotatedRect rotatedRect(cv::Point2f(0,0), src.size(), angle);    
    cv::Rect boundingRect = rotatedRect.boundingRect();
    boundingRect.width -= 1; boundingRect.height -= 1;

	int len = std::max(boundingRect.width, boundingRect.height);
    cv::Point2f center(len / 2., len / 2.);    

    static cv::Mat paddedTmp(len, len, src.type());
    paddedTmp.setTo(cv::Scalar(0,0,0,0));
    cv::Rect paddedRect((len - src.cols) / 2.0, (len - src.rows) / 2.0, src.cols, src.rows);
    src.copyTo(paddedTmp(paddedRect));

    
	static cv::Mat rotatedTmp;    
    cv::Mat transform = cv::getRotationMatrix2D(center, angle, 1.0);

	cv::warpAffine(paddedTmp, rotatedTmp, transform, cv::Size(len, len));  
	cv::Rect finalRect((len - boundingRect.width) / 2.0, (len - boundingRect.height) / 2.0, boundingRect.width, boundingRect.height);
    rotatedTmp(finalRect).copyTo(dst);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void buildPanorama(const vector<sensor_msgs::ImageConstPtr>& images, const vector<sensor_msgs::CameraInfoConstPtr>& cameraInfos)
{
	double averageTimestampSecs = 0.0;
	static cv::Mat panorama;
	static cv::Mat rotatedImages[MAX_CAMERAS];

	std::string encoding = images[0]->encoding;
	if(encoding == sensor_msgs::image_encodings::YUV422) encoding = sensor_msgs::image_encodings::BGR8;
    //else encoding = sensor_msgs::image_encodings::MONO8; // debug hack for visualization

	// Rotate individual images
	for(size_t i = 0; i < images.size(); i++) {
		cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(images[i], encoding);
		rotateImage(imagePtr->image, g_inplaneRotations[i], rotatedImages[i]);
	}

	// Determine full panorama width from rotated camera images
	unsigned int panoramaHeight = 0, panoramaWidth = 0;
	for(size_t i = 0; i < images.size(); i++) {
		if(vertical) {
			panoramaHeight += rotatedImages[i].rows;
			panoramaWidth = max(panoramaWidth, (unsigned)rotatedImages[i].cols);
		}
		else {
			panoramaWidth += rotatedImages[i].cols;
			panoramaHeight = max(panoramaHeight, (unsigned)rotatedImages[i].rows);
		}
	}

	panorama.create(panoramaHeight, panoramaWidth, rotatedImages[0].type()); // only reallocates if size has changed

	// Merge rotated images into panorama
	int currentPos = 0;
	for(size_t i = 0; i < images.size(); i++)
	{
		if(vertical) {
			rotatedImages[i].copyTo(panorama(cv::Rect(0, currentPos, rotatedImages[i].cols, rotatedImages[i].rows)));
			currentPos += rotatedImages[i].rows;
		}
		else {
			rotatedImages[i].copyTo(panorama(cv::Rect(currentPos, 0, rotatedImages[i].cols, rotatedImages[i].rows)));
			currentPos += rotatedImages[i].cols;
		}
		averageTimestampSecs += images[i]->header.stamp.toSec(); 
	}

	averageTimestampSecs /= images.size();

	// Create output image message
	cv_bridge::CvImagePtr outputImagePtr(new cv_bridge::CvImage());
	
	outputImagePtr->image = panorama;
	outputImagePtr->header.seq = images[0]->header.seq;

	// Timestamp must be in the timeframe of the original images, in case we are operating on a bag file!
	outputImagePtr->header.stamp = ros::Time(averageTimestampSecs);
	outputImagePtr->encoding = encoding;

	g_panoramaPublisher.publish(outputImagePtr->toImageMsg());
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Repacks the individual images and camera infos into vectors and then invokes buildPanorama(). A TimeSynchronizer assures
// that the images are approximately synchronized.
void cameraImageCallback (	const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::CameraInfoConstPtr& cameraInfo0,
							const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::CameraInfoConstPtr& cameraInfo1,
							const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::CameraInfoConstPtr& cameraInfo2,
							const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::CameraInfoConstPtr& cameraInfo3) {
	vector<sensor_msgs::ImageConstPtr> images;
	vector<sensor_msgs::CameraInfoConstPtr> cameraInfos;

	if(image0) images.push_back(image0);
	if(image1) images.push_back(image1);
	if(image2) images.push_back(image2);
	if(image3) images.push_back(image3);

	if(cameraInfo0) cameraInfos.push_back(cameraInfo0);
	if(cameraInfo1) cameraInfos.push_back(cameraInfo1);
	if(cameraInfo2) cameraInfos.push_back(cameraInfo2);
	if(cameraInfo3) cameraInfos.push_back(cameraInfo3);
	
	buildPanorama(images, cameraInfos);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "panorama_builder");
	ros::NodeHandle node;
	ros::NodeHandle node_ns("~");

	ROS_INFO("Starting application...");
	ROS_INFO("Processing parameters...");

	// Process parameters
	string topics;
	node_ns.param("topics", topics, string(""));
	boost::split(g_topics, topics, boost::is_any_of(" "));

	string inplane_rotations;
	node_ns.param<std::string>("inplane_rotations", inplane_rotations, string(""));
	replaceAll(inplane_rotations, "'", "");
	replaceAll(inplane_rotations, "\"", "");

	string output_topic;
	node_ns.param("output_topic", output_topic, string("/panorama/image_raw"));
	ROS_INFO_STREAM("Output topic is set to " << output_topic);

	node_ns.param("vertical", vertical, false);

	vector<std::string> inplaneRotationList;
	boost::split(inplaneRotationList, inplane_rotations, boost::is_any_of(" "));
	g_inplaneRotations.resize(inplaneRotationList.size());
	for(size_t i = 0; i < inplaneRotationList.size(); i++) g_inplaneRotations[i] = boost::lexical_cast<float>(inplaneRotationList[i]);
	for(size_t j = g_inplaneRotations.size(); j < g_topics.size(); j++) g_inplaneRotations[j] = 0.0f; // fill up missing values

	for(size_t i = 0; i < g_inplaneRotations.size(); i++) ROS_INFO_STREAM("Camera image " << i << " will be rotated by " << g_inplaneRotations[i] << " degrees");

	// Create output topic
	ROS_INFO("Creating output publisher...");
	image_transport::ImageTransport it(node);
	g_panoramaPublisher = it.advertise(output_topic, 1);

	// Determine number of cameras from input arguments
	const size_t NUM_CAMERAS = g_topics.size();
	ROS_INFO_STREAM("Using " << NUM_CAMERAS << " cameras");
	
	// Subscribe to camera feeds
	ROS_INFO("Subscribing to topics...");
	vector<message_filters::Subscriber<sensor_msgs::Image>*> imageSubscribers(NUM_CAMERAS);
	vector<message_filters::Subscriber<sensor_msgs::CameraInfo>*> cameraInfoSubscribers(NUM_CAMERAS);  

	for(size_t i = 0; i < NUM_CAMERAS; i++) {
		const string& topic = g_topics[i];

		string imageTopic = topic  + "/image_raw";
		ROS_INFO_STREAM("Subscribing to " << imageTopic);
		imageSubscribers[i] = new message_filters::Subscriber<sensor_msgs::Image>(node, imageTopic, 1);		

		string cameraInfoTopic = topic + "/camera_info";
		ROS_INFO_STREAM("Subscribing to " << cameraInfoTopic);
		cameraInfoSubscribers[i] = new message_filters::Subscriber<sensor_msgs::CameraInfo>(node, cameraInfoTopic, 1);
	}	

	ROS_INFO_STREAM("" << imageSubscribers.size() << " image and " << cameraInfoSubscribers.size() << " camera info subscribers have been created");
	
	// Create time synchronizer for these topics
	ROS_INFO("Creating time synchronizer...");
	void* synchronizer;
	const int SYNC_QUEUE_SIZE = 5;	
	sensor_msgs::ImageConstPtr nullImage;
	sensor_msgs::CameraInfoConstPtr nullCameraInfo;	

	switch(NUM_CAMERAS) {
		case 0:
		{
			ROS_ERROR("No camera topics have been specified!");
			return 1;
		}
		break;

		case 1:
		{
			typedef message_filters::sync_policies::ApproximateTime<	sensor_msgs::Image, sensor_msgs::CameraInfo 	> SyncPolicy;
  			synchronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(SYNC_QUEUE_SIZE),
  				*imageSubscribers[0], *cameraInfoSubscribers[0]);
			reinterpret_cast<message_filters::Synchronizer<SyncPolicy>*>(synchronizer)->registerCallback(boost::bind(&cameraImageCallback, 
				_1, _2,
				nullImage, nullCameraInfo,
				nullImage, nullCameraInfo,
				nullImage, nullCameraInfo));
		}
		break;

		case 2:
		{
			typedef message_filters::sync_policies::ApproximateTime<	sensor_msgs::Image, sensor_msgs::CameraInfo,
																		sensor_msgs::Image, sensor_msgs::CameraInfo 	> SyncPolicy;
  			synchronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(SYNC_QUEUE_SIZE),
  				*imageSubscribers[0], *cameraInfoSubscribers[0],
  				*imageSubscribers[1], *cameraInfoSubscribers[1]);
			reinterpret_cast<message_filters::Synchronizer<SyncPolicy>*>(synchronizer)->registerCallback(boost::bind(&cameraImageCallback, 
				_1, _2,
				_3, _4,
				nullImage, nullCameraInfo,
				nullImage, nullCameraInfo));
		}
		break;

		case 3:
		{
			typedef message_filters::sync_policies::ApproximateTime<	sensor_msgs::Image, sensor_msgs::CameraInfo,
																		sensor_msgs::Image, sensor_msgs::CameraInfo,
																		sensor_msgs::Image, sensor_msgs::CameraInfo 	> SyncPolicy;
  			synchronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(SYNC_QUEUE_SIZE),
  				*imageSubscribers[0], *cameraInfoSubscribers[0],
  				*imageSubscribers[1], *cameraInfoSubscribers[1],
  				*imageSubscribers[2], *cameraInfoSubscribers[2]);
			reinterpret_cast<message_filters::Synchronizer<SyncPolicy>*>(synchronizer)->registerCallback(boost::bind(&cameraImageCallback, 
				_1, _2,
				_3, _4,
				_5, _6,
				nullImage, nullCameraInfo));
		}
		break;

		case 4:
		{
			typedef message_filters::sync_policies::ApproximateTime<	sensor_msgs::Image, sensor_msgs::CameraInfo,
																		sensor_msgs::Image, sensor_msgs::CameraInfo,
																		sensor_msgs::Image, sensor_msgs::CameraInfo,
																		sensor_msgs::Image, sensor_msgs::CameraInfo 	> SyncPolicy;
  			synchronizer = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(SYNC_QUEUE_SIZE),
  				*imageSubscribers[0], *cameraInfoSubscribers[0],
  				*imageSubscribers[1], *cameraInfoSubscribers[1],
  				*imageSubscribers[2], *cameraInfoSubscribers[2],
  				*imageSubscribers[3], *cameraInfoSubscribers[3]);
			reinterpret_cast<message_filters::Synchronizer<SyncPolicy>*>(synchronizer)->registerCallback(boost::bind(&cameraImageCallback, 
				_1, _2,
				_3, _4,
				_5, _6,
				_7, _8));
		}
		break;

		default:
		{
			ROS_ERROR("Only 1-4 cameras are supported!");
			exit(1);
		}
	}

	// Start loop
	ROS_INFO("From now on continuously merging camera images into panorama...");	
	ros::Rate playbackRate(30.0);
	
	while(node.ok()) {
		ros::spinOnce();
		playbackRate.sleep();
	}

	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
