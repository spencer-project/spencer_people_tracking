#ifndef _SRL_LASER_DETECTORS_RANDOM_FOREST_DETECTOR_H
#define _SRL_LASER_DETECTORS_RANDOM_FOREST_DETECTOR_H

#include <boost/shared_ptr.hpp>
#include <srl_laser_detectors/learned_detectors/opencv_detector.h>


// Forward declarations
class CvRTrees;


namespace srl_laser_detectors {

/// Learned detector based upon a random forest classifier. Use ROS parameter server to configure.
class RandomForestDetector : public OpenCvDetector {
public:
    RandomForestDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);

    /// Unique name identifying the type of the detector.
    virtual const std::string getName();

    /// Get the underlying statistical model (base interface of all OpenCV classifiers). Must not be NULL.
    virtual CvStatModel* getStatModel();

    /// Classifies the feature vector of a single segment using the learned model. Returned confidence should be always 1.0 if classifier is non-probabilistic.
    virtual void classifyFeatureVector(const cv::Mat& featureVector, Label& label, double& confidence);

    /// Trains the classifier on the provided matrix of feature values (one row per segment), and the corresponding vector of segment labels.
    virtual void trainOnFeatures(const cv::Mat& featureMatrix, const cv::Mat& labelVector);

private:
    boost::shared_ptr< CvRTrees > m_randomForest;
    
    double m_decisionThreshold;
};


} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_RANDOM_FOREST_DETECTOR_H
