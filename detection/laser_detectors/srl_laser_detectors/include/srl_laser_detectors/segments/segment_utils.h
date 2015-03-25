#ifndef _SRL_LASER_DETECTORS_SEGMENT_UTILS_H
#define _SRL_LASER_DETECTORS_SEGMENT_UTILS_H

#include <sensor_msgs/LaserScan.h>
#include <srl_laser_segmentation/LaserscanSegmentation.h>
#include <srl_laser_detectors/types.h>


namespace srl_laser_detectors {

/// Utility methods for extracting and processing segments from laser scans.
class SegmentUtils {
public:
    /// Extracts segments from the given ROS laserscan message and laserscan segmentation, which must be synchronized.
    static void extractSegments(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, Segments& segments);
};

} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_SEGMENT_UTILS_H
