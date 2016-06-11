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

#include <limits>
#include <algorithm>
#include <srl_laser_detectors/naive_detectors/blob_detector.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;


namespace srl_laser_detectors {

BlobDetector::BlobDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) : Detector(nodeHandle, privateNodeHandle)
{
    ROS_INFO("Blob detector initialized!");
}

void BlobDetector::detect(const Segments& segments, Labels& labels, Confidences& confidences) {
    // Read parameters (do this each time so they can be adjusted at runtime)
    double min_points = 4;
    m_privateNodeHandle.getParamCached("min_points", min_points);

    double max_points = 25;
    m_privateNodeHandle.getParamCached("max_points", max_points);

    double min_width = 0.2; // Euclidean distance between first and last point of segment
    m_privateNodeHandle.getParamCached("min_width", min_width);

    double max_width = 0.7;
    m_privateNodeHandle.getParamCached("max_width", max_width);

    double max_range = 20.0; // if at least one point exceeds this distance to the origin, skip it
    m_privateNodeHandle.getParamCached("max_range", max_range);

    // Classify each segment individually
    for(int i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        Label& label = labels[i]; // initialized with BACKGROUND per default

        // Point count criteria
        if(segment.points.size() < min_points) continue;
        if(segment.points.size() > max_points) continue;

        // Segment width criteria
        Point2D firstPoint = segment.points.front();
        Point2D lastPoint  = segment.points.back();

        double width = (lastPoint - firstPoint).norm();
        if(width < min_width || width > max_width) continue;

        // Get point statistics
        double maxEncounteredRange = 0.0;
        foreach(const Point2D& point, segment.points) {
            maxEncounteredRange = max(maxEncounteredRange, point.norm() ); // assume sensor origin is at (0,0)
        }

        // Range criterion
        if(maxEncounteredRange > max_range) continue;

        // All criteria passed, looks like a valid detection
        label = FOREGROUND;
    }
}


} // end of namespace srl_laser_segmentation
