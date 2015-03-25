#include <srl_laser_segmentation/ahc.h>
#include <srl_laser_segmentation/ros/ros_interface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ahc_segmentation");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Set up the ROS interface, which also establishes the connection to the parameter server
    srl_laser_segmentation::ROSInterface rosInterface(nodeHandle, privateHandle);

    // Read parameters
    double distanceThreshold; privateHandle.param<double>("distance_threshold", distanceThreshold, 0.4); // in meters

    std::string linkageName; privateHandle.param<std::string>("linkage", linkageName, "single");

    srl_laser_segmentation::EfficientAHC::Linkage linkage;
    if(linkageName == "average") {
        linkage = srl_laser_segmentation::EfficientAHC::AVERAGE_CPU;
    }
    else if(linkageName == "complete") {
        linkage = srl_laser_segmentation::EfficientAHC::COMPLETE;
    }
    else {
        linkage = srl_laser_segmentation::EfficientAHC::SINGLE;
    }

    // Initialize segmentation algorithm and connect to the ROS interface
    srl_laser_segmentation::AgglomerativeHierarchicalClustering segmentation(linkage, distanceThreshold);    

    // Subscribe to laser scans and publish segmentations
    rosInterface.connect(&segmentation);
    ros::spin();
}
