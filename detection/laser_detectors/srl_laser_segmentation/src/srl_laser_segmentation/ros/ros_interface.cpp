#include <srl_laser_segmentation/ros/ros_interface.h>
#include <limits>

namespace srl_laser_segmentation {

ROSInterface::ROSInterface(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
    : m_nodeHandle(nodeHandle), m_privateNodeHandle(privateNodeHandle)
{
}

void ROSInterface::connect(SegmentationAlgorithm* segmentationAlgorithm, const std::string& laserTopic, const std::string& segmentationTopic)
{
    if(NULL == segmentationAlgorithm) {
        ROS_WARN("Segmentation algorithm is not set!");
        return;
    }
    m_segmentationAlgorithm = segmentationAlgorithm;

    size_t queue_size = 5;
    m_laserscanSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>(laserTopic, queue_size, &ROSInterface::newLaserscanAvailable, this);
    m_laserscanSegmentationPublisher = m_nodeHandle.advertise<srl_laser_segmentation::LaserscanSegmentation>(segmentationTopic, queue_size);
}

void ROSInterface::newLaserscanAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    // Convert laserscan into Cartesian coordinates
    std::vector<Point2D> pointsInCartesianCoords;
    for(size_t pointIndex = 0; pointIndex < laserscan->ranges.size(); pointIndex++) {
        double phi = laserscan->angle_min + laserscan->angle_increment * pointIndex;
        double rho = laserscan->ranges[pointIndex];

        Point2D point;
        if(rho > laserscan->range_max || rho < laserscan->range_min) {
            // Out-of-range measurement, set x and y to NaN
            // NOTE: We cannot omit the measurement completely since this would screw up point indices
            point(0) = point(1) = std::numeric_limits<double>::quiet_NaN();
        }
        else {
            point(0) =  cos(phi) * rho;
            point(1) = -sin(phi) * rho;
        }

        pointsInCartesianCoords.push_back(point);
    }    

    // Perform segmentation
    std::vector<srl_laser_segmentation::LaserscanSegment> resultingSegments;
    m_segmentationAlgorithm->performSegmentation(pointsInCartesianCoords, resultingSegments);

    // Read filter parameters
    int minPointsPerSegment = 3, maxPointsPerSegment = 50;
    m_privateNodeHandle.getParamCached("min_points_per_segment", minPointsPerSegment);
    m_privateNodeHandle.getParamCached("max_points_per_segment", maxPointsPerSegment);
    ROS_INFO_ONCE("Filtering out all resulting segments with less than %d or more than %d points!", minPointsPerSegment, maxPointsPerSegment);

    double minAvgDistanceFromSensor = 0;
    m_privateNodeHandle.getParamCached("min_avg_distance_from_sensor", minAvgDistanceFromSensor);   
    ROS_INFO_ONCE("Minimum allowed average segment distance from sensor is %f meters!", minAvgDistanceFromSensor);

    double maxAvgDistanceFromSensor = 10;
    m_privateNodeHandle.getParamCached("max_avg_distance_from_sensor", maxAvgDistanceFromSensor);   
    ROS_INFO_ONCE("Maximum allowed average segment distance from sensor is %f meters!", maxAvgDistanceFromSensor);

    // Filter segments
    srl_laser_segmentation::LaserscanSegmentation laserscanSegmentation;
    laserscanSegmentation.segments.reserve(resultingSegments.size());

    for(size_t i = 0; i < resultingSegments.size(); i++) {
        srl_laser_segmentation::LaserscanSegment& currentSegment = resultingSegments[i];

        Point2D mean = Point2D::Zero();
        size_t numValidPoints = 0;
        for(size_t j = 0; j < currentSegment.measurement_indices.size(); j++) {
            Point2D& point = pointsInCartesianCoords[currentSegment.measurement_indices[j]]; 
            if(std::isnan(point(0))) continue;
            mean += point;
            numValidPoints++;
        }

        // Filter by point count
        if(numValidPoints < minPointsPerSegment || numValidPoints > maxPointsPerSegment) continue;
        mean /= (double) numValidPoints;

        // Filter by maximum distance from sensor
        double sensorDistance = mean.norm();
        if(sensorDistance < minAvgDistanceFromSensor || sensorDistance > maxAvgDistanceFromSensor) continue;

        // Segment looks okay
        laserscanSegmentation.segments.push_back(currentSegment);
    }

    // Set message header and publish
    laserscanSegmentation.header = laserscan->header;
    m_laserscanSegmentationPublisher.publish(laserscanSegmentation);
}


} // end of namespace srl_laser_segmentation