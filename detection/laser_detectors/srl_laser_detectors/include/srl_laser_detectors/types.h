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
