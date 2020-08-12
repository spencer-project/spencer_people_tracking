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
    m_randomForest = cv::ml::RTrees::create();
    loadModel();

    m_privateNodeHandle.param<double>("decision_threshold", m_decisionThreshold, 0.5); // probability in this case
    ROS_INFO("Random forest detector initialized!");
}

const std::string RandomForestDetector::getName()
{
    return "random_forest";
}

cv::ml::StatModel* RandomForestDetector::getStatModel()
{
    return m_randomForest.get();
}

void RandomForestDetector::classifyFeatureVector(const cv::Mat& featureVector, Label& label, double& confidence)
{
    const int num_trees = m_randomForest->getTermCriteria().maxCount;
    double sum_votes = m_randomForest->predict(featureVector, cv::noArray(), cv::ml::DTrees::PREDICT_SUM);
    confidence = sum_votes / num_trees;
    label = confidence > m_decisionThreshold ? FOREGROUND : BACKGROUND;
}

void RandomForestDetector::trainOnFeatures(const cv::Mat& featureMatrix, const cv::Mat& labelVector)
{
    double sufficientAccuracy = -1.0; int numTrees = 50; int minSampleCount = 10; int maxTreeDepth = 5;
    m_privateNodeHandle.getParamCached("rf_sufficient_accuracy", sufficientAccuracy);
    m_privateNodeHandle.getParamCached("rf_num_trees", numTrees);
    m_privateNodeHandle.getParamCached("rf_min_sample_count", minSampleCount); // FIXME: unused
    m_privateNodeHandle.getParamCached("rf_max_depth", maxTreeDepth);

    ROS_INFO_STREAM("Training Random Forest classifier with " << numTrees << " trees and depth " << maxTreeDepth << "... this may take a while!");

    //m_randomForest->setRegressionAccuracy(sufficientAccuracy);
    m_randomForest->setMaxDepth(maxTreeDepth);
    m_randomForest->setTermCriteria(cv::TermCriteria( cv::TermCriteria::MAX_ITER + (sufficientAccuracy >= 0.0 ? cv::TermCriteria::EPS : 0), numTrees, sufficientAccuracy) );

    cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(featureMatrix, cv::ml::ROW_SAMPLE, labelVector, cv::noArray(), maskSamplesWithNonfiniteValues(featureMatrix));
    m_randomForest->train(trainData);
}



} // end of namespace srl_laser_segmentation
