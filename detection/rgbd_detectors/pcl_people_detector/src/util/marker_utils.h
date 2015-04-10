/*
 * marker_utils.h
 *
 *  Created on: Dec 12, 2013
 *      Author: linder (Social Robotics Lab, University of Freiburg)
 */

#ifndef MARKER_UTILS_H_
#define MARKER_UTILS_H_

#include <visualization_msgs/MarkerArray.h>

class MarkerUtils {
public:
    static visualization_msgs::Marker createMarker(std::string ns, std::string detectionFrame) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = detectionFrame;
        marker.header.stamp = _timeOfLastMessage;
        marker.ns = ns; // namespace
        return marker;
    }

    static void setTimeOfLastMessage(ros::Time time) {
        _timeOfLastMessage = time;
    }

    static std_msgs::ColorRGBA getPaletteColor(unsigned int index) {
        std_msgs::ColorRGBA color;
        switch(index % 8) {
        case 0: color.r = 255; color.g = 051; color.b = 051; break;
        case 2: color.r = 255; color.g = 153; color.b = 051; break;
        case 4: color.r = 255; color.g = 255; color.b = 051; break;
        case 6: color.r = 153; color.g = 051; color.b = 051; break;
        case 1: color.r = 051; color.g = 255; color.b = 051; break;
        case 3: color.r = 051; color.g = 255; color.b = 153; break;
        case 5: color.r = 051; color.g = 153; color.b = 255; break;
        case 7: color.r = 255; color.g = 051; color.b = 255; break;
        }

        color.r /= 255.0; color.g /= 255.0; color.b /= 255.0; color.a = 1.0;
        return color;
    }

private:
    static ros::Time _timeOfLastMessage;
};

ros::Time MarkerUtils::_timeOfLastMessage;


#endif /* MARKER_UTILS_H_ */
