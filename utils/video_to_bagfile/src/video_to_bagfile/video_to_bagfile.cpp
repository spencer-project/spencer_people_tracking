/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_to_bagfile");

  if(argc < 2) {
    ROS_ERROR("Must specify video filename as command-line argument!");
    return 1;
  }

  // Process command-line arguments
  string videoFilename = argv[1];
  string bagFilename = argc >= 3 ? argv[2] : videoFilename + ".bag";

  string imageTopic = "image";  ros::param::get("~image_topic", imageTopic);
  string compression = "jpeg";  ros::param::get("~compression", compression);
  string frameId = "base_link"; ros::param::get("~frame_id", frameId);
  bool timestampFromFile = true; ros::param::get("~timestamp_from_file", timestampFromFile);
  int jpegQuality = 80; ros::param::get("~jpeg_quality", jpegQuality);


  // Open video file
  cv::VideoCapture cap(videoFilename);
  if(!cap.isOpened()) {
    ROS_ERROR("Failed to open video file %s", videoFilename.c_str());
    return 2;
  }
  
  float fps = (float) cap.get(CV_CAP_PROP_FPS);
  ROS_INFO_STREAM("Video frame rate is " << fps << " FPS");
  
  size_t frameCount = cap.get(CV_CAP_PROP_FRAME_COUNT);
  ROS_INFO_STREAM("Video has " << frameCount << " frames");

  // Read timestamp
  string timeString;

  if(timestampFromFile) {
    stringstream ss; ss << videoFilename << ".TS";
    ifstream timestampFile(ss.str().c_str());
    if (timestampFile.is_open() && timestampFile.good())
    {
        getline(timestampFile, timeString);
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open timestamp file " << ss.str());
        return 3;
    }
  }

  boost::posix_time::ptime posixTime = boost::posix_time::second_clock::local_time();
  if(!timeString.empty()) posixTime = boost::posix_time::time_from_string(timeString);
  boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));
  boost::posix_time::time_duration diff = posixTime - epoch;

  double gmtOffset = 2 * 60 * 60;
  double secondsSinceEpoch = (double)diff.ticks() / diff.ticks_per_second() - gmtOffset;
  ros::Time currentTime(secondsSinceEpoch);
  ROS_INFO_STREAM("Start timestamp for video is " << std::fixed << secondsSinceEpoch);


  // Open bag file
  rosbag::Bag bag;
  bag.open(bagFilename, rosbag::bagmode::Write);

  size_t frameCounter = 0;
  while(true)
  {
    // Get next frame from video
    cv::Mat currentFrame;
    if(!cap.read(currentFrame)) {
        cout << endl;
        ROS_INFO("End of video stream has been reached!");
        ROS_INFO_STREAM("Finished writing " << bagFilename);
        break;
    }

    // Compress image if required
    if(compression == "jpeg" || compression == "png") {
        sensor_msgs::CompressedImage msg;
        msg.header.stamp = currentTime;
        msg.header.frame_id = frameId;

        msg.format = compression;

        vector<int> compressionParams;
        compressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compressionParams.push_back(9);
        compressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
        compressionParams.push_back(80);

        vector<unsigned char> data;
        string extension = "." + compression;
        cv::imencode(extension, currentFrame, data, compressionParams);
        msg.data = data;

        bag.write(imageTopic + "/compressed", currentTime, msg);
    }
    else if(compression.empty()) {
        cv_bridge::CvImage cvImage;
        cvImage.encoding = "bgr8";
        cvImage.image = currentFrame,
        cvImage.header.stamp = currentTime;
        cvImage.header.frame_id = frameId;

        bag.write(imageTopic, currentTime, cvImage.toImageMsg());
    }

    // Update timestamp
    currentTime += ros::Duration((double) 1/fps);
    frameCounter++;

    if(frameCounter % 50 == 0 || frameCounter >= frameCount) {
        cout << "\rvideo_to_bag: Processing frame " << frameCounter << " / " << frameCount << flush;
    }
  }

  bag.close();
}
