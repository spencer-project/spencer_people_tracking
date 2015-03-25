#ifndef _SRL_LASER_FEATURES_SEGMENTS_H
#define _SRL_LASER_FEATURES_SEGMENTS_H

#include <vector>
#include <Eigen/Core>


namespace srl_laser_features {

typedef Eigen::Vector2d Point2D;

/// Structure containing all points in a laser segment.
struct Segment {
    // Cartesian coordinates of all points in the segment
    std::vector<Point2D> points;

    // Metric distances of all points
    std::vector<double> ranges;

    // Original point indices in the laser scan, zero-based (used for matching segments with groundtruth annotations)
    std::vector<unsigned int> indices;

    // Moments and other useful characteristics
    Point2D mean, median, precedingPoint, succeedingPoint;
};

typedef std::vector<Segment> Segments;


} // end of namespace srl_laser_features

#endif // _SRL_LASER_FEATURES_SEGMENTS_H
