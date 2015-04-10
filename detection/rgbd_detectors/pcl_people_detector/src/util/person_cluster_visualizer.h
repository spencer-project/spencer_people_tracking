/*
 * person_cluster_visualizer.h
 *
 *  Created on: Dec 10, 2013
 *      Author: linder (Social Robotics Lab, University of Freiburg)
 */

#ifndef PERSON_CLUSTER_VISUALIZER_H_
#define PERSON_CLUSTER_VISUALIZER_H_

#include <string>
#include <vector>
#include <map>
#include <set>

#include <ros/ros.h>

template<typename PointT> class PersonClusterVisualizer {
public:
    static void createPublisher(ros::NodeHandle& nodeHandle, std::string detectionFrame);
    static void endFrame();

    static void visualize(const std::string& nameSpace, std::vector<pcl::people::PersonCluster<PointT> >& clusters);

private:
    static ros::Publisher _markerArrayPublisher;
    static std::string _detectionFrame;
    static int _visualizationCounter;

    /// Map from namespace to the old marker IDs of that namespace which
    /// have to be deleted before publishing new markers.
    static std::map<std::string, std::set<unsigned int> > _oldMarkerIds;
    static std::map<std::string, unsigned int> _lastMarkerIds;
};

template<typename PointT> ros::Publisher PersonClusterVisualizer<PointT>::_markerArrayPublisher;
template<typename PointT> std::string PersonClusterVisualizer<PointT>::_detectionFrame;
template<typename PointT> int PersonClusterVisualizer<PointT>::_visualizationCounter;
template<typename PointT> std::map<std::string, std::set<unsigned int> > PersonClusterVisualizer<PointT>::_oldMarkerIds;
template<typename PointT> std::map<std::string, unsigned int> PersonClusterVisualizer<PointT>::_lastMarkerIds;


#include "person_cluster_visualizer.hpp"

#endif /* PERSON_CLUSTER_VISUALIZER_H_ */
