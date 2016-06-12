// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "rwth_perception_people_msgs/GroundPlane.h"

using namespace std;
using namespace rwth_perception_people_msgs;

int _seq = 0;
tf::Vector3 _normal;

tf::TransformListener* listener;

ros::Publisher _pub_ground_plane;

int main(int argc, char **argv)
{

    // Set up ROS.
    ros::init(argc, argv, "ground_plane");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    std::string pub_topic_gp;
    std::string camera_frame;
    std::string base_footprint;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("base_footprint", base_footprint, string("base_footprint"));
    private_node_handle_.param("camera_frame", camera_frame, string("/camera/"));

    // Create a topic publisher
    double publishRate;
    private_node_handle_.param("rate", publishRate, 30.0);
    private_node_handle_.param("ground_plane", pub_topic_gp, string("/ground_plane"));
    ros::Publisher _pub_ground_plane = n.advertise<GroundPlane>(pub_topic_gp.c_str(), 10);

    listener = new tf::TransformListener();

    ros::Rate r(publishRate);
    while (ros::ok()) {
        // Can precompute.
        GroundPlane gp;

        geometry_msgs::Vector3Stamped normalVectorStamped;
        geometry_msgs::Vector3Stamped normalVectorStampedCamera;

        geometry_msgs::PointStamped distancePointStamped;
        geometry_msgs::PointStamped distancePointStampedCamera;

        normalVectorStamped.header.frame_id = base_footprint;
        normalVectorStamped.header.stamp = ros::Time();
        normalVectorStamped.vector.x = 0.0;
        normalVectorStamped.vector.y = 0.0;
        normalVectorStamped.vector.z = 1.0;

        distancePointStamped.header.frame_id = camera_frame;
        distancePointStamped.header.stamp = ros::Time();
        distancePointStamped.point.x = 0.0;
        distancePointStamped.point.y = 0.0;
        distancePointStamped.point.z = 0.0;

        try {
            listener->waitForTransform(base_footprint, camera_frame, ros::Time(), ros::Duration(0.1));
            listener->transformVector(camera_frame, normalVectorStamped, normalVectorStampedCamera);
            listener->waitForTransform(camera_frame, base_footprint, ros::Time(), ros::Duration(0.1));
            listener->transformPoint(base_footprint, distancePointStamped, distancePointStampedCamera);

            _normal.setX(normalVectorStampedCamera.vector.x);
            _normal.setY(normalVectorStampedCamera.vector.y);
            _normal.setZ(normalVectorStampedCamera.vector.z);

            gp.n.push_back(_normal.getX());
            gp.n.push_back(_normal.getY());
            gp.n.push_back(_normal.getZ());
            gp.d = distancePointStampedCamera.point.z;//1.67;

            gp.header.frame_id = camera_frame;
            gp.header.stamp = ros::Time::now();
            gp.header.seq = ++_seq;
            _pub_ground_plane.publish(gp);
        }
        catch(tf::TransformException ex) {
            ROS_WARN_THROTTLE(20.0, "Failed transform lookup in rwth_ground_plane/tf_based_fixed_gp -- maybe the RGB-D drivers are not yet running!? Reason: %s. Message will re-appear within 20 seconds.", ex.what());
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

