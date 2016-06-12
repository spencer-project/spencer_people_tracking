// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>

#include "rwth_perception_people_msgs/GroundPlane.h"

using namespace std;
using namespace rwth_perception_people_msgs;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "ground_plane");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pub_topic_gp;

    int _seq = 0;
    tf::Vector3 _normal;
    double _distance;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("distance", _distance, 1.67);
    XmlRpc::XmlRpcValue read_normal;
    if(!private_node_handle_.getParam("normal", read_normal)) { //Did not find a nicer way of setting a default
        ROS_INFO("No normal given. Will use default: 0.0, -1.0, 0.0");
        _normal.setX(0.0); _normal.setY(-1.0); _normal.setZ(0.0);
    } else {
        ROS_ASSERT(read_normal.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(read_normal[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        _normal.setX(static_cast<double>(read_normal[0]));
        ROS_ASSERT(read_normal[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        _normal.setY(static_cast<double>(read_normal[1]));
        ROS_ASSERT(read_normal[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        _normal.setZ(static_cast<double>(read_normal[2]));
        ROS_DEBUG_STREAM("Loading from YAML file:" << _normal.getX() << ", " << _normal.getY() << ", " << _normal.getZ());
    }

    // Create a topic publisher
    std::string camera_namespace;
    double publishRate;
    private_node_handle_.param("rate", publishRate, 30.0);
    private_node_handle_.param("ground_plane", pub_topic_gp, string("/ground_plane"));
    private_node_handle_.param("camera_namespace", camera_namespace, string("/camera/"));
    ros::Publisher _pub_ground_plane = n.advertise<GroundPlane>(pub_topic_gp.c_str(), 10);

    if(camera_namespace[camera_namespace.size()-1] == '/') camera_namespace = camera_namespace.substr(0, camera_namespace.size() - 1);

    // Can precompute.
    GroundPlane gp;
    gp.n.push_back(_normal.getX());
    gp.n.push_back(_normal.getY());
    gp.n.push_back(_normal.getZ());
    gp.d = _distance;

    ros::Rate r(publishRate);
    while (ros::ok()) {

        gp.header.frame_id = camera_namespace + "_depth_optical_frame";
        gp.header.stamp = ros::Time::now();
        gp.header.seq = ++_seq;
        _pub_ground_plane.publish(gp);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

