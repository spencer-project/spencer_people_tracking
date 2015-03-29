// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include "rwth_perception_people_msgs/GroundPlane.h"

using namespace std;
using namespace rwth_perception_people_msgs;

ros::Publisher _pub_ground_plane;

int _seq = 0;
tf::Vector3 _normal;
double _distance;

void callback(const sensor_msgs::JointState::ConstPtr &msg) {
    double tilt = 0.0;
    for(int i = 0; i < msg->name.size(); i++){
        if(strcmp(msg->name[i].c_str(),"tilt") == 0) {
            tilt = msg->position[i];
            ROS_DEBUG_STREAM("Received tilt of: " << tilt);
            break;
        } else {
            ROS_DEBUG_STREAM("Received no tilt value. Will use default: " << tilt);
        }
    }

    ROS_DEBUG_STREAM("Normal before rotation: " << _normal.getX() << ", " << _normal.getY() << ", " << _normal.getZ());
    tf::Vector3 n = _normal.rotate(tf::Vector3(1,0,0), tilt); //Rotate about x-axis using ptu tilt angle
    ROS_DEBUG_STREAM("Normal after rotation: " << n.getX() << ", " << n.getY() << ", " << n.getZ());
    GroundPlane gp;
    gp.header.frame_id = "/head_xtion_depth_optical_frame";
    gp.header.stamp = msg->header.stamp;
    gp.header.seq = ++_seq;
    ROS_DEBUG_STREAM("Created header:\n" << gp.header);
    gp.n.push_back(n.getX());
    gp.n.push_back(n.getY());
    gp.n.push_back(n.getZ());
    gp.d = _distance;
    _pub_ground_plane.publish(gp);
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(ros::NodeHandle &n, ros::Subscriber &sub, string topic) {
    if(!_pub_ground_plane.getNumSubscribers()) {
        ROS_DEBUG("Ground Plane fixed: No subscribers. Unsubscribing.");
        sub.shutdown();
    } else {
        ROS_DEBUG("Ground Plane fixed: New subscribers. Subscribing.");
        sub = n.subscribe(topic.c_str(), 10, &callback);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "ground_plane");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pub_topic_gp;
    string sub_ptu_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("ptu_state", sub_ptu_topic, string("/ptu/state"));
    private_node_handle_.param("distance", _distance, 1.7);
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

    // Create a subscriber.
    ros::Subscriber ptu_sub;
    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback, boost::ref(n), boost::ref(ptu_sub), sub_ptu_topic);

    // Create a topic publisher
    private_node_handle_.param("ground_plane", pub_topic_gp, string("/ground_plane"));
    _pub_ground_plane = n.advertise<GroundPlane>(pub_topic_gp.c_str(), 10, con_cb, con_cb);


    ros::spin();

    return 0;
}

