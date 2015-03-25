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
