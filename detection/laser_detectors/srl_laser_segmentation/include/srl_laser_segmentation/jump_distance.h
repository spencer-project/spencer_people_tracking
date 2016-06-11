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
    virtual void performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment::Ptr>& resultingSegments);

private:
    /// Checks if there is a significant jump between two adjacent points.
    bool isJumpBetween(const Point2D* p1, const Point2D* p2);


    /// The jump distance above which a new segment is created.
    double m_jumpDistance;
};


} // end of namespace srl_laser_segmentation

#endif // _SRL_LASER_SEGMENTATION_JUMP_DISTANCE_H
