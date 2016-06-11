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

#ifndef _SRL_LASER_DETECTORS_LEARNED_DETECTOR_H
#define _SRL_LASER_DETECTORS_LEARNED_DETECTOR_H

#include <srl_laser_detectors/detector.h>


namespace srl_laser_detectors {

/// Base class for a learned detector that requires annotated groundtruth segments for training to learn a classification model.
class LearnedDetector : public Detector {
public:
    /// Constructor
    LearnedDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) : Detector(nodeHandle, privateNodeHandle) {};

    /// Detects persons from laser segments. The result is returned as a list of labels and confidences, of the same length as the input list of segments.
    /// Labels must be initialized with all BACKGROUND and confidences must all be set to 1.0 by the caller.
    /// If the detector is non-probabilistic, the returned confidences will be all 1.0.
    virtual void detect(const Segments& segments, Labels& labels, Confidences& confidences) = 0;

    /// Trains the detector on the given annotated groundtruth segments, which should contain both positive and negative samples.
    /// Implemented by the derived class.
    virtual void train(const Segments& segments, const Labels& labels) = 0;

    /// Loads a learned model from file. Returns false if loading failed.
    virtual bool loadModel(const std::string& filename) = 0;

    /// Saves a learned model to file. Returns false if saving failed.
    virtual bool saveModel(const std::string& filename) = 0;
};


} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_LEARNED_DETECTOR_H
