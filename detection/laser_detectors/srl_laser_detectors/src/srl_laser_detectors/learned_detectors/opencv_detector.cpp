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

#include <srl_laser_detectors/learned_detectors/opencv_detector.h>
#include <srl_laser_features/features/feature_registry.h>

#include <fstream>
#include <opencv2/core/core.hpp>

using namespace srl_laser_features;

namespace srl_laser_detectors {


OpenCvDetector::OpenCvDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) : LearnedDetector(nodeHandle, privateNodeHandle)
{
    m_features = FeatureRegistry::getAllFeatures();
    m_featureDimensions = FeatureRegistry::getAllFeatureDimensions();
}


void OpenCvDetector::detect(const Segments& segments, Labels& labels, Confidences& confidences)
{
    cv::Mat featureMatrix = calculateFeatureMatrix(segments);

    for(size_t i = 0; i < segments.size(); i++) {
        // Implemented by the derived class.
        classifyFeatureVector(featureMatrix.row(i), labels[i], confidences[i]);
    }
}


void OpenCvDetector::train(const Segments& segments, const Labels& labels)
{
    cv::Mat featureMatrix = calculateFeatureMatrix(segments);

    cv::Mat labelVector(labels.size(), 1, CV_32SC1);
    for(size_t i = 0; i < labels.size(); i++) {
        labelVector.at<signed>(i) = labels[i];
    }

    // Dump feature values into file (for analysis with Matlab etc.)
    dumpFeatureMatrix(featureMatrix, labels);

    // Implemented by the derived class.
    trainOnFeatures(featureMatrix, labelVector);
}


void OpenCvDetector::dumpFeatureMatrix(const cv::Mat& featureMatrix, const Labels& labels)
{
    bool dump_feature_matrix = true; m_privateNodeHandle.getParamCached("dump_feature_matrix", dump_feature_matrix);
    if(!dump_feature_matrix) return;

    ROS_INFO("Dumping feature matrix with %d rows of sample segments and %d feature dimensions", featureMatrix.rows, featureMatrix.cols);
    std::ofstream file("laser_feature_matrix.csv");

    // Header
    for(int col = 0; col < featureMatrix.cols; col++) {
        file << m_featureDimensions[col] << "\t";
    }
    file << "label\n";

    // Data
    for(int row = 0; row < featureMatrix.rows; row++) {
        for(int col = 0; col < featureMatrix.cols; col++) {
            file << featureMatrix.at<double>(row, col) << "\t";
        }
        file << labels[row] << "\n";
    }
}


cv::Mat OpenCvDetector::maskSamplesWithNonfiniteValues(const cv::Mat& featureMatrix) {
    cv::Mat activeSamples(featureMatrix.rows, 1, CV_8UC1, cv::Scalar(1));
    size_t invalidSamples = 0;

    for(size_t sample = 0; sample < featureMatrix.rows; sample++) {
        for(size_t col = 0; col < featureMatrix.cols; col++) {
            if(!std::isfinite(featureMatrix.at<float>(sample, col))) {
                activeSamples.at<unsigned char>(sample) = 0;
                invalidSamples++;
                break;
            }
        }
    }

    ROS_INFO_STREAM("Out of " << featureMatrix.rows << " samples, " << invalidSamples << " had to be removed due to non-finite values!");
    return activeSamples;
}


bool OpenCvDetector::loadModel(const std::string& filename)
{
    std::string theFilename = filename;

    if(theFilename.empty()) m_privateNodeHandle.getParam("model", theFilename);

    if(theFilename.empty()) {
        ROS_WARN("No model to load has been specified, detector will not be initialized");
        return false;
    }

    cv::FileStorage fileStorage(theFilename, cv::FileStorage::READ);

    // Check type of detector that was used to train the model
    cv::FileNode detectorTypeNode = fileStorage["detector_type"];
    std::string detectorType = (std::string) detectorTypeNode[0];
    if(detectorType != getName()) {
        ROS_ERROR_STREAM("Detector of type '" << getName() << "' cannot load a model learned for detector of type '" << detectorType << "'!");
        return false;
    }

    // Check feature dimensions that were used to train the model
    const std::vector<FeatureDimension> allFeatureDimensions = FeatureRegistry::getAllFeatureDimensions();
    std::vector<FeatureDimension> featureDimensions;

    cv::FileNode featureDimensionsNode = fileStorage["used_feature_dimensions"];
    for(int i = 0; i < featureDimensionsNode.size(); i++) {
        FeatureDimension currentDimension = (FeatureDimension) featureDimensionsNode[i];

        if(std::find(allFeatureDimensions.begin(), allFeatureDimensions.end(), currentDimension) == allFeatureDimensions.end()) {
            ROS_ERROR_STREAM("Unknown feature dimension: " << currentDimension);
            return false;
        }
        featureDimensions.push_back( currentDimension );
    }
    m_featureDimensions = featureDimensions;
    ROS_INFO("Trained model uses %zu feature dimension(s)", featureDimensions.size());

    cv::FileNode classifierNode = fileStorage["classifier"];
    getStatModel()->read(*fileStorage, *classifierNode);
    return true; // FIXME: How to check success? Does load() throw an exception if it fails?
}


bool OpenCvDetector::saveModel(const std::string& filename)
{
    cv::FileStorage fileStorage(filename, cv::FileStorage::WRITE);

    fileStorage << "detector_type" << getName();
    fileStorage << "used_feature_dimensions" << "[";

    for(size_t i = 0; i < m_featureDimensions.size(); i++) {
        fileStorage << m_featureDimensions[i];
    }

    fileStorage << "]";

    getStatModel()->write(*fileStorage, "classifier");
    return true; // FIXME: How to check success?
}


cv::Mat OpenCvDetector::calculateFeatureMatrix(const Segments& segments)
{
    cv::Mat featureMatrix(segments.size(), m_featureDimensions.size(), CV_32FC1);

    for(size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        std::map<FeatureDimension, float> featureValueLookup; // one entry per dimension per feature

        for(size_t j = 0; j < m_features.size(); j++) {
            Feature::Ptr feature = m_features[j];
            Eigen::VectorXd values;
            feature->evaluate(segment, values);

            // Each feature can have multiple dimensions, store these in a map so we can look up
            // the requested dimensions in the next step
            for(size_t dim = 0; dim < feature->getNDimensions(); dim++) {
                featureValueLookup[ feature->getDescription(dim) ] = values(dim);
            }
        }

        // Look up the requested dimensions
        for(size_t k = 0; k < m_featureDimensions.size(); k++) {
            const FeatureDimension& featureDescription = m_featureDimensions[k];
            featureMatrix.at<float>(i, k) = featureValueLookup[featureDescription];
        }
    }

    return featureMatrix;
}


} // end of namespace srl_laser_segmentation
