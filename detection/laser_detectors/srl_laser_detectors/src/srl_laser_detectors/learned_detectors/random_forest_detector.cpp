#include <srl_laser_detectors/learned_detectors/random_forest_detector.h>



namespace srl_laser_detectors {

RandomForestDetector::RandomForestDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) : OpenCvDetector(nodeHandle, privateNodeHandle)
{
    m_randomForest.reset( new CvRTrees() );
    loadModel();

    m_privateNodeHandle.param<double>("decision_threshold", m_decisionThreshold, 0.5); // probability in this case
    ROS_INFO("Random forest detector initialized!");
}

const std::string RandomForestDetector::getName()
{
    return "random_forest";
}

CvStatModel* RandomForestDetector::getStatModel()
{
    return m_randomForest.get();
}

void RandomForestDetector::classifyFeatureVector(const cv::Mat& featureVector, Label& label, double& confidence)
{
    confidence = m_randomForest->predict_prob(featureVector);
    label = confidence > m_decisionThreshold ? FOREGROUND : BACKGROUND;
}

void RandomForestDetector::trainOnFeatures(const cv::Mat& featureMatrix, const cv::Mat& labelVector)
{
    CvRTParams params;
    
    double sufficientAccuracy = 0.1; int numTrees = 50;
    m_privateNodeHandle.getParamCached("rf_sufficient_accuracy", sufficientAccuracy);
    m_privateNodeHandle.getParamCached("rf_num_trees", numTrees);

    params.term_crit = cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, numTrees, sufficientAccuracy );

    params.max_depth = 5;
    m_privateNodeHandle.getParamCached("rf_max_depth", params.max_depth);


    ROS_INFO_STREAM("Training Random Forest classifier with " << numTrees << " trees and depth " << params.max_depth << "... this may take a while!");
    m_randomForest->train(featureMatrix, CV_ROW_SAMPLE, labelVector, cv::Mat(), maskSamplesWithNonfiniteValues(featureMatrix), cv::Mat(), cv::Mat(), params);
}



} // end of namespace srl_laser_segmentation