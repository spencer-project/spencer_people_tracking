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

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <string>

using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_camera_info");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Process parameters   
    if(argc < 2) {
        ROS_FATAL("Must specify calibration file name as argument!");
        return 1;
    }

    double hz; string frame; string calibrationFile;
    calibrationFile = argv[1];
    privateHandle.param("rate", hz, 30.0);
    privateHandle.param("frame", frame, string("camera_optical_frame"));

    // Load calibration file
    string cameraName;
    sensor_msgs::CameraInfo msg;
    if(!camera_calibration_parsers::readCalibration(calibrationFile, cameraName, msg)) {
        ROS_FATAL_STREAM("Failed to load calibation file " << calibrationFile);
        return 2;
    }

    msg.header.frame_id = frame;
        
    // Create publishers
    string topicName = "camera_info";
    ros::Publisher cameraInfoPublisher = nodeHandle.advertise<sensor_msgs::CameraInfo>(topicName, 1, true);

    ROS_INFO_STREAM("Publishing camera info loaded from " << calibrationFile << " at " << ros::names::remap(topicName) << " topic with frame ID " << frame << " at a rate of " << hz << " Hz!");

    ros::Rate rate(hz);
    while(ros::ok()) {
        msg.header.stamp = ros::Time::now();
        cameraInfoPublisher.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
   
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
