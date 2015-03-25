#include <srl_laser_segmentation/jump_distance.h>
#include <srl_laser_segmentation/ros/ros_interface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "jump_distance_segmentation");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Set up the ROS interface, which also establishes the connection to the parameter server
    srl_laser_segmentation::ROSInterface rosInterface(nodeHandle, privateHandle);

    // Read parameters
    double jumpDistance; privateHandle.param<double>("distance_threshold", jumpDistance, 0.4); // in meters
    
    // Initialize segmentation algorithm and connect to the ROS interface
    srl_laser_segmentation::JumpDistanceSegmentation segmentation(jumpDistance);    

    // Subscribe to laser scans and publish segmentations
    rosInterface.connect(&segmentation);
    ros::spin();
}
