#ifndef _SRL_LASER_SEGMENTATION_JUMP_DISTANCE_H
#define _SRL_LASER_SEGMENTATION_JUMP_DISTANCE_H

#include <srl_laser_segmentation/segmentation_algorithm.h>


namespace srl_laser_segmentation {

/// A segmentation algorithm.
class JumpDistanceSegmentation: public SegmentationAlgorithm {
public:
    /// Constructor.
    /// @param jumpDistance The jump distance above which a new segment is created.
    JumpDistanceSegmentation(double jumpDistance);

    /// Segment the given list of points. Consecutive points are assumed to be adjacent, i.e. the ordering of points is relevant.
    virtual void performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment>& resultingSegments);

private:
    /// Checks if there is a significant jump between two adjacent points.
    bool isJumpBetween(const Point2D* p1, const Point2D* p2);


    /// The jump distance above which a new segment is created.
    double m_jumpDistance;
};


} // end of namespace srl_laser_segmentation

#endif // _SRL_LASER_SEGMENTATION_JUMP_DISTANCE_H
