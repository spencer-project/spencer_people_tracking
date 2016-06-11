#ifndef _SRL_LASER_SEGMENTATION_SEGMENTATION_ALGORITHM_H
#define _SRL_LASER_SEGMENTATION_SEGMENTATION_ALGORITHM_H

#include <Eigen/Core>
#include <cmath>

#include <srl_laser_segmentation/LaserscanSegment.h>


namespace srl_laser_segmentation {

/// Cartesian coordinates of a point in 2D space.
typedef Eigen::Vector2d Point2D;
    
/// Interface of a segmentation algorithm.
class SegmentationAlgorithm {
public:
    // Destructor
    virtual ~SegmentationAlgorithm() {};

    /// Segment the given list of points. Consecutive points are assumed to be adjacent, i.e. the ordering of points is relevant.
    virtual void performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment::Ptr>& resultingSegments) = 0;

protected:
    /// Check if the given measurement is valid (i.e. not out-of-range)
    bool isValidMeasurement(const Point2D* point) {
        return !isnan(point->sum());
    }
};


} // end of namespace srl_laser_segmentation

#endif // _SRL_LASER_SEGMENTATION_SEGMENTATION_ALGORITHM_H
