#include <srl_laser_detectors/ros/ros_interface.h>
#include <srl_laser_detectors/detector.h>
#include <srl_laser_detectors/segments/segment_utils.h>
#include <limits>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace std;


namespace srl_laser_detectors {

ROSInterface::ROSInterface(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
    : m_nodeHandle(nodeHandle), m_privateNodeHandle(privateNodeHandle), m_lastDetectionId(0)
{
}


void ROSInterface::connect(Detector* detector, const string& laserTopic, const string& segmentationTopic, const string& detectedPersonsTopic)
{
    if(NULL == detector) {
        ROS_WARN("Detector is not set!");
        return;
    }
    m_detector = detector;

    int queue_size = 1;

    // Get parameters
    int detectionIdOffset;
    m_privateNodeHandle.param<int>("detection_id_increment", m_detectionIdIncrement, 1);
    m_privateNodeHandle.param<int>("detection_id_offset", detectionIdOffset, 0);
    m_privateNodeHandle.param<int>("subscriber_queue_size", queue_size, 1);
    m_lastDetectionId = detectionIdOffset;    

    int synchronizer_queue_size = queue_size;
    m_privateNodeHandle.param<int>("synchronizer_queue_size", synchronizer_queue_size, 5);

    // Subscribers
    m_laserscanSubscriber.reset( new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nodeHandle, laserTopic, queue_size) );
    m_segmentationSubscriber.reset (new message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation>(m_nodeHandle, segmentationTopic, queue_size) );

    m_inputSynchronizer.reset( new Synchronizer(SyncPolicy(synchronizer_queue_size), *m_laserscanSubscriber, *m_segmentationSubscriber) );
    m_inputSynchronizer->registerCallback(&ROSInterface::newLaserscanAndSegmentationAvailable, this);

    // Publisher
    m_detectedPersonsPublisher = m_nodeHandle.advertise<spencer_tracking_msgs::DetectedPersons>(detectedPersonsTopic, queue_size);

    double min_expected_frequency, max_expected_frequency;
    m_privateNodeHandle.param("min_expected_frequency", min_expected_frequency, 30.0);
    m_privateNodeHandle.param("max_expected_frequency", max_expected_frequency, 100.0);
    
    m_detectedPersonsPublisher.setExpectedFrequency(min_expected_frequency, max_expected_frequency);
    m_detectedPersonsPublisher.setMaximumTimestampOffset(0.3, 0.1);
    m_detectedPersonsPublisher.finalizeSetup();
}


void ROSInterface::newLaserscanAndSegmentationAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation)
{
    // Extract segments
    Segments segments;
    SegmentUtils::extractSegments(laserscan, segmentation, segments);

    // Run detector on all segments
    Confidences resultingConfidences(segments.size(), 0.0);
    Labels resultingLabels(segments.size(), BACKGROUND);
    m_detector->detect(segments, resultingLabels, resultingConfidences);

    //
    // Output DetectedPersons message
    //
    bool useMedianForPose = true;
    m_privateNodeHandle.getParamCached("use_median_for_pose", useMedianForPose);

    double poseVariance = 0.2; // default variance (on diagonal of covariance matrix) of published poses
    m_privateNodeHandle.getParamCached("pose_variance", poseVariance);

    spencer_tracking_msgs::DetectedPersons detectedPersons;
    detectedPersons.header = laserscan->header;
    
    for(size_t i = 0; i < segments.size(); i++) {
        // Skip segments classified as background
        Label label = resultingLabels[i];
        if(label == BACKGROUND) continue;

        // Create DetectedPerson
        const Segment& segment = segments[i]; 
        
        spencer_tracking_msgs::DetectedPerson detectedPerson;
        m_lastDetectionId += m_detectionIdIncrement;
        detectedPerson.detection_id = m_lastDetectionId;
        detectedPerson.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_LASER_2D;
        
        detectedPerson.confidence = resultingConfidences[i];

        detectedPerson.pose.pose.position.x = useMedianForPose ? segment.median(0) : segment.mean(0);
        detectedPerson.pose.pose.position.y = useMedianForPose ? segment.median(1) : segment.mean(1);
        detectedPerson.pose.pose.position.z = 0.0;

        const double LARGE_VARIANCE = 999999999;
        for(size_t d = 0; d < 2; d++) detectedPerson.pose.covariance[d*6 + d] = poseVariance;
        for(size_t d = 2; d < 6; d++) detectedPerson.pose.covariance[d*6 + d] = LARGE_VARIANCE;  

        detectedPersons.detections.push_back(detectedPerson);      
    }

    // Publish message
    m_detectedPersonsPublisher.publish(detectedPersons);
}


} // end of namespace srl_laser_detectors
