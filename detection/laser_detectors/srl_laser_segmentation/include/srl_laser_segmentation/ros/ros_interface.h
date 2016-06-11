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
