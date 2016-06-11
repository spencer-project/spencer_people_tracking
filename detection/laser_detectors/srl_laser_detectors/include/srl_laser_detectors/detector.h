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

#ifndef _SRL_LASER_DETECTORS_DETECTOR_H
#define _SRL_LASER_DETECTORS_DETECTOR_H

#include <vector>
#include <ros/ros.h>
#include <Eigen/Core>

#include <srl_laser_detectors/types.h>


namespace srl_laser_detectors {

/// Abstract detector base class.
class Detector {
public:
    /// Constructs a new detector. The node handle can be used to create any additional subscribers/publishers for detector meta-data,
    /// if required. The private node handle can be used to retrieve parameters from the parameter server.
    Detector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
        : m_nodeHandle(nodeHandle), m_privateNodeHandle(privateNodeHandle)
    {}

    /// Detects persons from laser segments. The result is returned as a list of labels and confidences, of the same length as the input list of segments.
    /// Labels must be initialized with all BACKGROUND and confidences must all be set to 1.0 by the caller.
    /// If the detector is non-probabilistic, the returned confidences will be all 1.0.
    virtual void detect(const Segments& segments, Labels& labels, Confidences& confidences) = 0;

    /// Tests the detector on the given annotated groundtruth segments, and outputs detection metrics. Calls detect().
    /// The list of groundtruth labels must be of the same size as the list of segments, i.e. be based upon the same segmentation.
    virtual void test(const Segments& segments, const Labels& groundtruthLabels, DetectionMetrics& detectionMetrics) {
        assert(groundtruthLabels.size() == segments.size());

        Confidences confidences(segments.size(), 1.0);
        Labels labels(segments.size(), BACKGROUND);
        detect(segments, labels, confidences);

        detectionMetrics.tp = detectionMetrics.fp = detectionMetrics.tn = detectionMetrics.fn = 0;
        for(size_t i = 0; i < segments.size(); i++) {
            if(labels[i] == BACKGROUND) {
                if(groundtruthLabels[i] == BACKGROUND) detectionMetrics.tn++;
                else detectionMetrics.fn++;
            }
            else {
                if(groundtruthLabels[i] != BACKGROUND) detectionMetrics.tp++;
                else detectionMetrics.fp++;
            }
        }

        detectionMetrics.precision = detectionMetrics.tp / float(detectionMetrics.tp + detectionMetrics.fp); // = positive predictive value
        detectionMetrics.recall    = detectionMetrics.tp / float(detectionMetrics.tp + detectionMetrics.fn); // = sensitivity
        detectionMetrics.accuracy  = (detectionMetrics.tp + detectionMetrics.tn) / float(segments.size());

        detectionMetrics.f1measure = 2 * (detectionMetrics.precision * detectionMetrics.recall) / (detectionMetrics.precision + detectionMetrics.recall);

    }

    /// Default destructor
    virtual ~Detector() {}


protected:
    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;
};


} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_DETECTOR_H
