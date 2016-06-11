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

#ifndef _SRL_LASER_DETECTORS_TYPES_H
#define _SRL_LASER_DETECTORS_TYPES_H

#include <vector>
#include <ros/ros.h>
#include <Eigen/Core>

#include <srl_laser_features/segment.h>


namespace srl_laser_detectors {

typedef Eigen::Vector2d Point2D;

/// Class label for training or testing a detector.
enum Label {
    BACKGROUND,
    FOREGROUND,
    AMBIGUOUS
};

/// Detection statistics of a detector (for testing / cross-validation)
struct DetectionMetrics {
    size_t tp, fp, tn, fn;
    double precision, recall, accuracy, f1measure;
};

/// Typedef for easier readability
typedef srl_laser_features::Segment Segment;
typedef srl_laser_features::Segments Segments;


typedef std::vector<Label> Labels;
typedef std::vector<double> Confidences;


} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_TYPES_H
