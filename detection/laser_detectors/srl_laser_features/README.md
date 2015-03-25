This package contains laser features (e.g. linearity, circularity, spline-fitting error, ...) for segmented laser range data.
The code in this package does not have many dependencies, and can thus be included in any C++ ROS application.

Dependencies: Eigen3

Usage example
=============

#include <srl_laser_features/features/feature_registry.h>
#include <srl_laser_features/segment.h>
using namespace srl_laser_features;

...

// Initialize segment data
Segment segment; 
segment.points = ...; // all points (in 2D Cartesian coordinates) in this segment (required)
segment.ranges = ...; // distance of each point from the sensor origin, can be empty: In that case, particular features will have constant value
segment.mean = ...;   // mean of all points (required)
segment.median = ...; // median of all points (required)
segment.precedingPoint = ...;  // last point of previous neighbouring segment. Set all coordinates to NaN if unknown.
segment.succeedingPoint = ...; // first point of following neighbouring segment. Set all coordinates to NaN if unknown.

// Retrieve list of all available features
const Features& features = FeatureRegistry::getAllFeatures()

// Calculate value of each feature on this segment
for(Features::const_iterator it = features.begin(); it != features.end(); ++it) {
    const Feature& feature = *it;
    Eigen::VectorXd result;
    feature->evaluate(segment, result);

    // Now do something with result. Dimensionality of result is equal to feature->getNDimensions()
}
