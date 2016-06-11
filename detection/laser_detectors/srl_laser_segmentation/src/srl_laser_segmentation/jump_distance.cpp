#include <srl_laser_segmentation/jump_distance.h>

namespace srl_laser_segmentation {

JumpDistanceSegmentation::JumpDistanceSegmentation(double jumpDistance) : m_jumpDistance(jumpDistance)
{
}

void JumpDistanceSegmentation::performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment::Ptr>& resultingSegments)
{
    unsigned int segmentCounter = 0;
    srl_laser_segmentation::LaserscanSegment::Ptr currentSegment(new srl_laser_segmentation::LaserscanSegment);
    const Point2D *previousPoint = NULL, *currentPoint = NULL;

    currentSegment->label = segmentCounter;

    for(size_t pointIndex = 0; pointIndex < points.size(); pointIndex++){
        // Skip measurement if it is out-of-range
        const Point2D *newPoint = &points[pointIndex];
        if(!isValidMeasurement(newPoint)) continue;

        previousPoint = currentPoint;
        currentPoint = newPoint;

        if(previousPoint && isJumpBetween(previousPoint, currentPoint)) {
            resultingSegments.push_back(currentSegment);

            currentSegment.reset(new srl_laser_segmentation::LaserscanSegment);
            currentSegment->label = segmentCounter++;
        }

        currentSegment->measurement_indices.push_back(pointIndex);
    }

    // Don't forget storing the last segment
    if(!currentSegment->measurement_indices.empty()) resultingSegments.push_back(currentSegment);
}

bool JumpDistanceSegmentation::isJumpBetween(const Point2D* p1, const Point2D* p2)
{
    const Point2D diff = *p1 - *p2;
    return diff.norm() > m_jumpDistance;
}


} // end of namespace srl_laser_segmentation