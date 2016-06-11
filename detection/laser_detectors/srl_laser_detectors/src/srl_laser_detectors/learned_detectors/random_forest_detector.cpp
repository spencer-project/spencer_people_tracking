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
