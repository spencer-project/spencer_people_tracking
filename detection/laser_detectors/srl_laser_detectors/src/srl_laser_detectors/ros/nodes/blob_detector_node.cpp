#include <srl_laser_detectors/naive_detectors/blob_detector.h>
#include <srl_laser_detectors/ros/ros_interface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_detector");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Initialize detector and connect to the ROS interface
    srl_laser_detectors::BlobDetector detector(nodeHandle, privateHandle);

    // Subscribe to laser scans and segmentations
    srl_laser_detectors::ROSInterface rosInterface(nodeHandle, privateHandle);
    rosInterface.connect(&detector);
    ros::spin();
}
