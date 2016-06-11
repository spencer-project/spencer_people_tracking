/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <srl_laser_segmentation/extended_jump_distance.h>
#include <ros/console.h>

#include <set>

namespace srl_laser_segmentation {

ExtendedJumpDistanceSegmentation::ExtendedJumpDistanceSegmentation(double jumpDistance) : m_jumpDistance(jumpDistance)
{
}

void ExtendedJumpDistanceSegmentation::performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment::Ptr>& resultingSegments)
{
    std::set<unsigned int> pushedLabels;

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
            if(pushedLabels.find(currentSegment->label) == pushedLabels.end()) {
                resultingSegments.push_back(currentSegment);
                pushedLabels.insert(currentSegment->label);
            }
            currentSegment.reset();

            // This idea is inspired by "Multimodal detection and tracking of pedestrians in urban environments with explicit ground plane extraction" by Spinello, Triebel & Siegwart
            // It is a good compromise between full AHC (slow) and simple jump distance clustering (not optimal).
            // As opposed to there, however, we only check against the last point of preceding segments, not their centroids.

            // First-order comparison: Check pre-predecessor segment to see if it is close to the current point
            if(resultingSegments.size() > 1) {
                srl_laser_segmentation::LaserscanSegment::Ptr otherSegment = resultingSegments[resultingSegments.size() - 2];
                size_t lastPointIndex = otherSegment->measurement_indices.back();
                if(!isJumpBetween(&points[lastPointIndex], currentPoint)) currentSegment = otherSegment; // in that case, use it as the new segment to add points to
            }

            // Second-order comparison: Check pre-pre-predecessor segment if it is close to the current one
            if(!currentSegment && resultingSegments.size() > 2) {
                srl_laser_segmentation::LaserscanSegment::Ptr otherSegment = resultingSegments[resultingSegments.size() - 3];
                size_t lastPointIndex = otherSegment->measurement_indices.back();
                if(!isJumpBetween(&points[lastPointIndex], currentPoint)) currentSegment = otherSegment;
            }

            // Else, create a completely new segment
            if(!currentSegment) {
                currentSegment.reset(new srl_laser_segmentation::LaserscanSegment); // begin new segment
                currentSegment->label = segmentCounter++;
            }
        }

        currentSegment->measurement_indices.push_back(pointIndex);
    }

    // Don't forget storing the last segment
    if(!currentSegment->measurement_indices.empty()) {
        if(pushedLabels.find(currentSegment->label) == pushedLabels.end()) resultingSegments.push_back(currentSegment);
    }
}

bool ExtendedJumpDistanceSegmentation::isJumpBetween(const Point2D* p1, const Point2D* p2)
{
    const Point2D diff = *p1 - *p2;
    return diff.norm() > m_jumpDistance;
}


} // end of namespace srl_laser_segmentation
