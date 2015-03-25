#ifndef _SRL_LASER_DETECTORS_BLOB_DETECTOR_H
#define _SRL_LASER_DETECTORS_BLOB_DETECTOR_H

#include <srl_laser_detectors/detector.h>


namespace srl_laser_detectors {

/// A simple blob detector that classifies segments based upon a pre-defined set of criteria.
class BlobDetector : public Detector {
public:
    BlobDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);

    virtual void detect(const Segments& segments, Labels& labels, Confidences& confidences);
};


} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_BLOB_DETECTOR_H
