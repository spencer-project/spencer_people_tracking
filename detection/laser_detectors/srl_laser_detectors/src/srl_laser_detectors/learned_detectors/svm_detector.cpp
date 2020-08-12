/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <srl_laser_detectors/learned_detectors/svm_detector.h>



namespace srl_laser_detectors {

SVMDetector::SVMDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) : OpenCvDetector(nodeHandle, privateNodeHandle)
{
    m_svm = cv::ml::SVM::create();
    loadModel();
    
    m_privateNodeHandle.param<double>("decision_threshold", m_decisionThreshold, 0); // probability in this case
    ROS_INFO("SVM detector initialized!");
}

const std::string SVMDetector::getName()
{
    return "svm";
}

cv::ml::StatModel* SVMDetector::getStatModel()
{
    return m_svm.get();
}

void SVMDetector::classifyFeatureVector(const cv::Mat& featureVector, Label& label, double& confidence)
{
    float signed_margin = m_svm->predict(featureVector, cv::noArray(), cv::ml::StatModel::RAW_OUTPUT);
    label = signed_margin > m_decisionThreshold ? FOREGROUND : BACKGROUND; // FIXME: Is the > sign correct?
    confidence = signed_margin;
}

void SVMDetector::trainOnFeatures(const cv::Mat& featureMatrix, const cv::Mat& labelVector)
{
    /* Experiment with class weights, doesn't really do any good (leads to very low recall!)
    // Calculate class weight from distribution of positive and negative samples
    cv::Mat classWeights(2, 1, CV_32FC1, cv::Scalar(0));
    for(size_t i = 0; i < labelVector.total(); i++) {
        signed label = labelVector.at<signed>(i);
        classWeights.at<float>(label) += 1.0f;
    }

    for(size_t j = 0; j < classWeights.total(); j++) {
        classWeights.at<float>(j) /= labelVector.total();
    }

    // Swap class weights (less frequent class should have higher weight)
    float tmp = classWeights.at<float>(0);
    classWeights.at<float>(0) = classWeights.at<float>(1);
    classWeights.at<float>(1) = tmp;

    CvMat classWeights_ = classWeights;
    params.class_weights = &classWeights_;

    ROS_INFO_STREAM("Automatically determined class weights for SVM: " << classWeights);
    */

    // Get type of SVM kernel
    std::string kernelName = "linear"; m_privateNodeHandle.getParamCached("svm_kernel_type", kernelName);

    int kernel = cv::ml::SVM::LINEAR;
    if(kernelName == "linear")
        kernel = cv::ml::SVM::LINEAR;
    else if(kernelName == "rbf")
        kernel = cv::ml::SVM::RBF;
    else if(kernelName == "poly")
        kernel = cv::ml::SVM::POLY; // polynomial
    else if(kernelName == "sigmoid")
        kernel = cv::ml::SVM::SIGMOID;
    else {
        ROS_ERROR_STREAM("Unknown SVM kernel type: " << kernelName);
        return;
    }

    m_svm->setType(cv::ml::SVM::C_SVC);
    m_svm->setC(0.1);
    m_svm->setKernel(kernel);

    ROS_INFO_STREAM("Automatically training SVM classifier with " << kernelName << " kernel using cross-validation... this may take a while!");
    
    cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(featureMatrix, cv::ml::ROW_SAMPLE, labelVector, cv::noArray(), maskSamplesWithNonfiniteValues(featureMatrix));
    m_svm->trainAuto(trainData);
}



} // end of namespace srl_laser_segmentation
