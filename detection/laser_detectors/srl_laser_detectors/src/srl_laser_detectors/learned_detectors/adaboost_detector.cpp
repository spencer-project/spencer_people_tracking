#include <srl_laser_detectors/learned_detectors/adaboost_detector.h>



namespace srl_laser_detectors {

AdaboostDetector::AdaboostDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) : OpenCvDetector(nodeHandle, privateNodeHandle)
{
    m_adaboost.reset( new CvBoost() );
    loadModel();

    m_privateNodeHandle.param<double>("adaboost_threshold", m_decisionThreshold, 0.0);
    ROS_INFO("Adaboost detector initialized!");
}

const std::string AdaboostDetector::getName()
{
    return "adaboost";
}

CvStatModel* AdaboostDetector::getStatModel()
{
    return m_adaboost.get();
}

void AdaboostDetector::classifyFeatureVector(const cv::Mat& featureVector, Label& label, double& confidence)
{
    confidence = m_adaboost->predict(featureVector);
    label = confidence > m_decisionThreshold ? FOREGROUND : BACKGROUND;
}

void AdaboostDetector::trainOnFeatures(const cv::Mat& featureMatrix, const cv::Mat& labelVector)
{
    CvBoostParams params;
    params.boost_type=CvBoost::DISCRETE;
    params.weight_trim_rate = 0;
    params.weak_count = 100;
    m_privateNodeHandle.getParamCached("adaboost_weak_count", params.weak_count);

    ROS_INFO_STREAM("Training Adaboost classifier with " << params.weak_count << " weak classifiers... this may take a while!");
    m_adaboost->train(featureMatrix, CV_ROW_SAMPLE, labelVector, cv::Mat(), maskSamplesWithNonfiniteValues(featureMatrix), cv::Mat(), cv::Mat(), params);
}



} // end of namespace srl_laser_segmentation