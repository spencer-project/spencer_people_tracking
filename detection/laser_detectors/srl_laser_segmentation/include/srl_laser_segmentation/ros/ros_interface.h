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

#ifndef _SRL_LASER_SEGMENTATION_ROS_INTERFACE_H
#define _SRL_LASER_SEGMENTATION_ROS_INTERFACE_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srl_laser_segmentation/LaserscanSegmentation.h>

#include <srl_laser_segmentation/segmentation_algorithm.h>


namespace srl_laser_segmentation {

/// Connects a SegmentationAlgorithm to ROS via topics.
class ROSInterface {
public:
    /// Constructor
    ROSInterface(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);

    /// Connect the segmentation algorithm to ROS, this will subscribe to a sensor_msgs/LaserScan topic and output a srl_laser_segmentation/LaserSegmentation topic.
    /// Remember to call ros::spin() after invoking this method.
    void connect(SegmentationAlgorithm* segmentationAlgorithm, const std::string& laserTopic = "laser", const std::string& segmentationTopic = "laser_segmentation");

private:
    void newLaserscanAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan);

    SegmentationAlgorithm* m_segmentationAlgorithm;

    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;
    ros::Publisher m_laserscanSegmentationPublisher;
    ros::Publisher m_laserscanSegmentationUnfilteredPublisher;
    ros::Subscriber m_laserscanSubscriber;
};


} // end of namespace srl_laser_segmentation

#endif // _SRL_LASER_SEGMENTATION_ROS_INTERFACE_H
