/* Created on: May 07, 2014. Author: Timm Linder */

#include <ros/ros.h>
#include <srl_nearest_neighbor_tracker/ros/ros_interface.h>
#include <srl_nearest_neighbor_tracker/nearest_neighbor_tracker.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "srl_nearest_neighbor_tracker");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Set up the ROS interface, which also establishes the connection to the parameter server
    srl_nnt::ROSInterface rosInterface(nodeHandle, privateHandle);

    // Now create the tracker and connect to the ROS interface
    srl_nnt::NearestNeighborTracker tracker(nodeHandle, privateHandle);

    rosInterface.connect(&tracker);
    rosInterface.spin();
}
