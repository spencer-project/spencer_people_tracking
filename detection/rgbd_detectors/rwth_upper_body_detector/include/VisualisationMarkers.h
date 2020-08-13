#ifndef VISUALISATIONMARKERS_H
#define VISUALISATIONMARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

#include <string.h>
#include <vector>

class VisualisationMarkers {
public:
    VisualisationMarkers():
        marker_seq(0) {}

    visualization_msgs::Marker createMarker(std::string target_frame, geometry_msgs::Pose pose) {
        geometry_msgs::Vector3 scale;
        scale.x = 0.3;
        scale.y = 0.3;
        scale.z = 0.3;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 0.0F/255.0F;
        color.g = 140.0F/255.0F;
        color.b = 0.0F/255.0F;
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame;
        marker.header.stamp = ros::Time::now();
        marker.header.seq = ++marker_seq;
        marker.ns = "people_tracker";
        marker.id = marker_seq;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale = scale;
        marker.color = color;
        marker.lifetime = ros::Duration(0.2);
        return marker;
    }

private:
    unsigned long marker_seq;
};

#endif // VISUALISATIONMARKERS_H
