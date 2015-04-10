/*
 * person_cluster_visualizer.hpp
 *
 *  Created on: Dec 10, 2013
 *      Author: linder (Social Robotics Lab, University of Freiburg)
 */

#ifndef PERSON_CLUSTER_VISUALIZER_HPP_
#define PERSON_CLUSTER_VISUALIZER_HPP_

#include "person_cluster_visualizer.h"
#include "marker_utils.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/MarkerArray.h>


template<typename PointT> void PersonClusterVisualizer<PointT>::createPublisher(ros::NodeHandle& nodeHandle, std::string detectionFrame) {
    _markerArrayPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("cluster_marker_array", 5);
    _detectionFrame = detectionFrame;
}

template<typename PointT> void PersonClusterVisualizer<PointT>::endFrame() {
    _visualizationCounter = 0;
}

template<typename PointT> void PersonClusterVisualizer<PointT>::visualize(const std::string& nameSpace, std::vector<pcl::people::PersonCluster<PointT> >& clusters)
{
    // Save us some computation time if there are no subscribers.
    if(_markerArrayPublisher.getNumSubscribers() == 0) return;

    // Look up values for this particular namespace
    std::set<unsigned int>& oldMarkerIds = _oldMarkerIds[nameSpace];
    unsigned int& lastMarkerId = _lastMarkerIds[nameSpace];

    visualization_msgs::MarkerArray markerArray;

    // Remove old clusters
    for(std::set<unsigned int>::const_iterator it = oldMarkerIds.begin(); it != oldMarkerIds.end(); ++it)  {
        visualization_msgs::Marker oldClusterMarker = MarkerUtils::createMarker(nameSpace, _detectionFrame);
        oldClusterMarker.id = *it;
        oldClusterMarker.action = visualization_msgs::Marker::DELETE;
        markerArray.markers.push_back(oldClusterMarker);
    }
    oldMarkerIds.clear();

    double lifetime = 1.0 / 15;
    unsigned int k = lastMarkerId + 1;
    for (typename std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it) {
        //
        // 3D bounding box
        //
        visualization_msgs::Marker clusterMarker = MarkerUtils::createMarker(nameSpace, _detectionFrame);
        clusterMarker.id = k;
        clusterMarker.type = visualization_msgs::Marker::CUBE;
        clusterMarker.color = MarkerUtils::getPaletteColor(_visualizationCounter);
        clusterMarker.color.a = 0.35;
        clusterMarker.scale.x = it->getMax().x() - it->getMin().x();
        clusterMarker.scale.y = it->getHeight(); // note that in the detection frame, y is up
        clusterMarker.scale.z = it->getMax().z() - it->getMin().z();
        clusterMarker.lifetime.fromSec(lifetime); // just to be safe

        Eigen::Vector3f tcenter = it->getTCenter();
        tf::poseEigenToMsg(Eigen::Translation3d(tcenter.cast<double>()) * Eigen::Affine3d::Identity(), clusterMarker.pose);
        markerArray.markers.push_back(clusterMarker);
        oldMarkerIds.insert(clusterMarker.id);
        k++;

        //
        // Center of gravity
        //
        visualization_msgs::Marker cogMarker = MarkerUtils::createMarker(nameSpace, _detectionFrame);
        cogMarker.id = k;
        cogMarker.type = visualization_msgs::Marker::SPHERE;
        cogMarker.color = MarkerUtils::getPaletteColor(_visualizationCounter);
        cogMarker.color.a = 1.0;
        cogMarker.scale.x = cogMarker.scale.y = cogMarker.scale.z = 0.1; // 10 cm
        cogMarker.lifetime.fromSec(lifetime); // just to be safe

        Eigen::Vector3f center = it->getCenter();
        tf::poseEigenToMsg(Eigen::Translation3d(center.cast<double>()) * Eigen::Affine3d::Identity(), cogMarker.pose);
        markerArray.markers.push_back(cogMarker);
        oldMarkerIds.insert(cogMarker.id);
        k++;

        //
        // Person confidence (if set)
        //
        if(it->getPersonConfidence() == it->getPersonConfidence()) { // must not be NaN
            std::stringstream confidenceStr; confidenceStr << std::fixed << std::setprecision(1) << it->getPersonConfidence();
            visualization_msgs::Marker confidenceMarker = MarkerUtils::createMarker(nameSpace, _detectionFrame);
            confidenceMarker.id = k;
            confidenceMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            //confidenceMarker.color = MarkerUtils::getPaletteColor(_visualizationCounter);
            confidenceMarker.color.r = confidenceMarker.color.g = confidenceMarker.color.b = 1.0;
            confidenceMarker.color.a = 1.0;
            confidenceMarker.scale.z = 0.2; // height of letters in m(?)
            confidenceMarker.lifetime.fromSec(lifetime); // just to be safe
            confidenceMarker.text = confidenceStr.str();

            Eigen::Vector3f top = it->getTop();
            tf::poseEigenToMsg(Eigen::Translation3d(top.cast<double>()) * Eigen::Affine3d::Identity(), confidenceMarker.pose);
            markerArray.markers.push_back(confidenceMarker);
            oldMarkerIds.insert(confidenceMarker.id);
            k++;
        }
    }

    lastMarkerId = k;
    _markerArrayPublisher.publish(markerArray);
    _visualizationCounter++;
}


#endif /* PERSON_CLUSTER_VISUALIZER_HPP_ */
