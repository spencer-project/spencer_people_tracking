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
    m_svm.reset( new CvSVM() );
    loadModel();
    ROS_INFO("SVM detector initialized!");
}

const std::string SVMDetector::getName()
{
    return "svm";
}

CvStatModel* SVMDetector::getStatModel()
{
    return m_svm.get();
}

void SVMDetector::classifyFeatureVector(const cv::Mat& featureVector, Label& label, double& confidence)
{
    label = (Label) m_svm->predict(featureVector);
    confidence = 1.0;
}

void SVMDetector::trainOnFeatures(const cv::Mat& featureMatrix, const cv::Mat& labelVector)
{
    CvSVMParams params;
    params.svm_type=CvSVM::C_SVC;
    params.C = 0.1;

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
    std::string kernelType = "linear"; m_privateNodeHandle.getParamCached("svm_kernel_type", kernelType);
    if(kernelType == "linear")
        params.kernel_type = CvSVM::LINEAR;
    else if(kernelType == "rbf")
        params.kernel_type = CvSVM::RBF;
    else if(kernelType == "poly")
        params.kernel_type = CvSVM::POLY; // polynomial
    else if(kernelType == "sigmoid")
        params.kernel_type = CvSVM::SIGMOID;
    else
        ROS_ERROR_STREAM("Unknown SVM kernel type: " << kernelType);

    ROS_INFO_STREAM("Automatically training SVM classifier using cross-validation... this may take a while!");
    m_svm->train_auto(featureMatrix, labelVector, cv::Mat(), maskSamplesWithNonfiniteValues(featureMatrix), params);
}



} // end of namespace srl_laser_segmentation
