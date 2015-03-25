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
