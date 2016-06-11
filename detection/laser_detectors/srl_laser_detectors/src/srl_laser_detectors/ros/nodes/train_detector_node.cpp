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

#include <srl_laser_detectors/detector_factory.h>
#include <srl_laser_detectors/ros/ros_interface.h>
#include <srl_laser_detectors/learned_detector.h>
#include <srl_laser_detectors/segments/segment_utils.h>

#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>  // for visualization

using namespace srl_laser_detectors;
using namespace std;

Segments trainingAndTestSegments;
Labels trainingAndTestLabels;
int maxCycleLimit;
size_t numForegroundSegments = 0, numBackgroundSegments = 0, numAmbiguousSegments = 0;
bool stopListening = false;
ros::WallTime lastTrainingDataReceivedAt;
boost::shared_ptr<ros::Publisher> segmentCloudPublisher;

/// CTRL+C handler
void sigintHandler(int sgn __attribute__ ((unused)))
{
    stopListening = true;
}


/// Helper function for visualization
sensor_msgs::PointField createPointField(const std::string& name, size_t offset, uint8_t datatype) {
    sensor_msgs::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    return field;
}

float randomFloat(float minValue, float maxValue)
{
    float r = (float)rand() / RAND_MAX;
    return minValue + r * (maxValue - minValue);
}


/// Callback that is invoked everytime new training data (synchronized laserscan, segmentation and annotations) is received via ROS
void newTrainingDataReceived(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& annotations)
{
    lastTrainingDataReceivedAt = ros::WallTime::now();
    if(stopListening) return;

    // Get groundtruth foreground / background segmentation; the assumption is that annotated segments contain only foreground.
    vector<Label> groundtruth(laserscan->ranges.size(), BACKGROUND);
    foreach(srl_laser_segmentation::LaserscanSegment annotation, annotations->segments) {
        foreach(size_t pointIndex, annotation.measurement_indices) {
            groundtruth[pointIndex] = FOREGROUND; // all annotated points are considered as foreground
        }
    }

    // Extract segments from laserscan using provided segmentation (e.g. by jump-distance)
    Segments segments;
    SegmentUtils::extractSegments(laserscan, segmentation, segments);

    // Thresholds for considering a segment as foreground or background
    // These are necessary because the automatically generated segmentation will never exactly match the segmentation of the human-made annotations.
    double foregroundThreshold = 0.8; // FIXME: parameter
    double backgroundThreshold = 0.8; // FIXME: parameter

    // For later visualization
    vector<float> cloudData;

    // Compare each segment to the groundtruth annotations
    foreach(Segment& segment, segments) {
        // Count overlap in foreground / background points
        size_t numForegroundPoints = 0, numBackgroundPoints = 0;
        foreach(size_t pointIndex, segment.indices) {
            if(groundtruth[pointIndex] == FOREGROUND) numForegroundPoints++;
            else numBackgroundPoints++;
        }

        // Decide if this is a foreground, background or ambiguous segment
        const size_t numTotalPoints = numForegroundPoints + numBackgroundPoints;
        const double foregroundRatio = numForegroundPoints / (double)numTotalPoints;
        const double backgroundRatio = numBackgroundPoints / (double)numTotalPoints;

        Label label;

        double segmentWidth = hypot(segment.points.front()(0) - segment.points.back()(0), segment.points.front()(1) - segment.points.back()(1));

        int minPointsForForegroundSegment = 10;
        double maxSegmentWidth = 0.7;

        if(backgroundRatio >= backgroundThreshold) {
            trainingAndTestSegments.push_back(segment);
            label = BACKGROUND;
            numBackgroundSegments++;
        }
        else if(foregroundRatio >= foregroundThreshold) {
            if(numTotalPoints < minPointsForForegroundSegment || segmentWidth > maxSegmentWidth) {
                label = AMBIGUOUS;
                numAmbiguousSegments++;
            }
            else {
                trainingAndTestSegments.push_back(segment);
                label = FOREGROUND;
                numForegroundSegments++;
            }
        }
        else {
            label = AMBIGUOUS;
            numAmbiguousSegments++;
        }

        if(label != AMBIGUOUS) trainingAndTestLabels.push_back(label);

        // Visualization as point cloud (to be viewed in RViz), color visualizes label
        if(segmentCloudPublisher->getNumSubscribers()) {
            float r = 0, g = 0, b = 0;
            if(label == BACKGROUND || label == AMBIGUOUS) r = 255.0f;
            if(label == FOREGROUND || label == AMBIGUOUS) g = 255.0f;

            double intensity = randomFloat(0.2, 1.0f);
            r *= intensity;
            g *= intensity;

            for(size_t pointIndex = 0; pointIndex < segment.points.size(); pointIndex++) {
                typedef union { uint32_t i; float f; } ColorValue;
                ColorValue colorValue;
                colorValue.i = uint32_t(b) + (uint32_t(g) << 8) + (uint32_t(r) << 16);

                cloudData.push_back(segment.points[pointIndex](0)); // x
                cloudData.push_back(segment.points[pointIndex](1)); // y
                cloudData.push_back(0.0); // z
                cloudData.push_back(numeric_limits<float>::quiet_NaN()); // padding
                cloudData.push_back(colorValue.f); // color
            }
        }
    }

    // Show some statistics
    static size_t lastInfoShownInCycle = 0;
    static size_t currentCycle = 0;
    if(currentCycle++ > lastInfoShownInCycle + 10) {
        cout << "\rReceived so far (after " << currentCycle << " cycles): " << numForegroundSegments << " foreground segments, "
             << numBackgroundSegments << " background segments, " << numAmbiguousSegments << " ambiguous/unclear segments. ";

        if(numAmbiguousSegments / (numForegroundSegments + numBackgroundSegments + numAmbiguousSegments) > 0.3) cout << "Check if laserscan is under-segmented, or adjust thresholds!";

        cout << flush;
        lastInfoShownInCycle = currentCycle;
    }


    // Limit max. frame count
    if(currentCycle > maxCycleLimit) {
        stopListening = true;
        ROS_INFO("Reached cycle limit, stopping to listen!");
    }

    // Publish visualization
    if(segmentCloudPublisher->getNumSubscribers()) {
        sensor_msgs::PointCloud2 visualizationCloud;

        visualizationCloud.header = segmentation->header;
        visualizationCloud.is_bigendian = false;
        visualizationCloud.is_dense = false;
        visualizationCloud.fields.push_back(createPointField("x", 0, sensor_msgs::PointField::FLOAT32));
        visualizationCloud.fields.push_back(createPointField("y", 1 * sizeof(float), sensor_msgs::PointField::FLOAT32));
        visualizationCloud.fields.push_back(createPointField("z", 2 * sizeof(float), sensor_msgs::PointField::FLOAT32));
        visualizationCloud.fields.push_back(createPointField("rgb", 4 * sizeof(float), sensor_msgs::PointField::FLOAT32));

        visualizationCloud.point_step = sizeof(float) * 5;
        uint8_t* castedPointArray = reinterpret_cast<uint8_t*>(&cloudData[0]);
        visualizationCloud.width = cloudData.size() / 5;
        visualizationCloud.data = std::vector<uint8_t>(castedPointArray, castedPointArray + visualizationCloud.point_step * visualizationCloud.width);
        visualizationCloud.height = 1;
        visualizationCloud.row_step = visualizationCloud.width * visualizationCloud.point_step;

        segmentCloudPublisher->publish(visualizationCloud);
    }
}

/// ROS node for training a learned detector.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "train_detector");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");
    signal(SIGINT, sigintHandler);

    // Look up detector type
    DetectorFactory::init();
    DetectorType type; privateHandle.param<string>("type", type, "");
    boost::shared_ptr<Detector> detector = DetectorFactory::createDetector(type, nodeHandle, privateHandle);

    LearnedDetector* learnedDetector = dynamic_cast<LearnedDetector*>(detector.get());
    if(NULL == learnedDetector) {
        ROS_WARN("Specified detector type (%s) cannot be trained. Training stage will be skipped.", type.c_str());
    }

    privateHandle.param<int>("cycle_limit", maxCycleLimit, numeric_limits<int>::max());

    // Create publisher for visualization
    segmentCloudPublisher.reset( new ros::Publisher(nodeHandle.advertise<sensor_msgs::PointCloud2>("/train_fg_bg_segments", 10)) );

    // Create ROS subscribers to receive training and test data
    size_t queue_size = 100000; // allow very long queue in order not to miss any samples (memory-intense!)

    // Initialize subscribers
    message_filters::Subscriber<sensor_msgs::LaserScan> laserscanSubscriber(nodeHandle, "laser", queue_size);
    message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation> segmentationSubscriber(nodeHandle, "laser_segmentation", queue_size);
    message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation> annotationSubscriber(nodeHandle, "laser_annotations", queue_size);

    // Create exact time synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::LaserScan, srl_laser_segmentation::LaserscanSegmentation, srl_laser_segmentation::LaserscanSegmentation> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    Synchronizer synchronizer(SyncPolicy(queue_size), laserscanSubscriber, segmentationSubscriber, annotationSubscriber);
    synchronizer.registerCallback(&newTrainingDataReceived);


    //
    // Collect training / test data
    //
    double timeout;
    privateHandle.param<double>("timeout", timeout, 10.0);
    ROS_INFO("Listening for training / test data (laserscans, segmentation, groundtruth segment annotations)... these need to have exactly matching timestamps!");
    ROS_INFO("Will stop listening if not receiving training data for more than %.1f sec!", timeout);
    if(maxCycleLimit != numeric_limits<int>::max()) ROS_INFO("Will also stop listening after %d cycles.", maxCycleLimit);

    while(!stopListening) {
        ros::spinOnce();
        if(lastTrainingDataReceivedAt.toSec() > 0.0) {
            if(ros::WallTime::now() > lastTrainingDataReceivedAt + ros::WallDuration(timeout)) stopListening = true;
        }
    }

    ROS_INFO("Starting training phase!");
    ros::spinOnce();

    //
    // Create training and test folds
    //
    srand(0 /*unsigned(time(0))*/);
    assert(trainingAndTestSegments.size() == trainingAndTestLabels.size());

    vector<size_t> randomIndices; randomIndices.resize(trainingAndTestSegments.size());
    for(size_t i = 0; i < trainingAndTestSegments.size(); i++) randomIndices[i] = i;

    bool shuffleDataBeforeTrainTestSplit;
    privateHandle.param<bool>("shuffle_data_before_train_test_split", shuffleDataBeforeTrainTestSplit, true);
    if(shuffleDataBeforeTrainTestSplit) {
        random_shuffle(randomIndices.begin(), randomIndices.end());
    }

    double testSetRatio;
    privateHandle.param<double>("test_set_ratio", testSetRatio, 0.1);
    const size_t lastTestElement = randomIndices.size() * testSetRatio;

    Segments trainingSegments, testSegments;
    Labels trainingLabels, testLabels;
    size_t numForegroundTraining = 0, numForegroundTest = 0;

    for(size_t i = 0; i < randomIndices.size(); i++) {
        size_t randomIndex = randomIndices[i];

        Segment& segment = trainingAndTestSegments[randomIndex];
        Label& label = trainingAndTestLabels[randomIndex];

        if(i < lastTestElement) {
            testSegments.push_back(segment);
            testLabels.push_back(label);
            if(label == FOREGROUND) numForegroundTest++;
        }
        else {
            trainingSegments.push_back(segment);
            trainingLabels.push_back(label);
            if(label == FOREGROUND) numForegroundTraining++;
        }
    }

    ROS_INFO("Training set contains %zu segments, of which %zu (%.1f percent) are foreground", trainingSegments.size(), numForegroundTraining, 100.0f * numForegroundTraining / trainingSegments.size() );
    ROS_INFO("Test set contains %zu segments, of which %zu (%.1f percent) are foreground", testSegments.size(), numForegroundTest, 100.0f * numForegroundTest / testSegments.size() );
    ros::spinOnce();

    //
    // Train detector
    //
    string outputFolder, filenamePrefix;
    privateHandle.param<string>("output_folder", outputFolder, "");
    privateHandle.param<string>("filename_prefix", filenamePrefix, "learned_model_");
    if(!outputFolder.empty() && outputFolder[outputFolder.size()-1] != '/') outputFolder += "/";

    stringstream modelFilename;
    modelFilename << outputFolder << filenamePrefix << (long) ros::Time::now().toSec() << "." << type;

    if(NULL != learnedDetector) {
        ROS_INFO("Starting training phase... this may take a while, please be patient!");
        assert(trainingSegments.size() == trainingLabels.size());
        learnedDetector->train(trainingSegments, trainingLabels);

        learnedDetector->saveModel(modelFilename.str());
        ROS_INFO_STREAM("Learned model has been saved to " << modelFilename.str());
    }

    ros::spinOnce();


    //
    // Test detector
    //
    ROS_INFO("Starting test phase...");
    assert(testSegments.size() == testLabels.size());

    boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

    DetectionMetrics detectionMetrics;
    detector->test(testSegments, testLabels, detectionMetrics);

    boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();

    // Output test results to file and console
    stringstream results;
    results << "Test results (detection metrics): " << fixed << setprecision(3) << endl
            << " Accuracy=" << detectionMetrics.accuracy
            << " Precision=" << detectionMetrics.precision
            << " Recall=" << detectionMetrics.recall
            << " F1-Measure=" << detectionMetrics.f1measure
            << " FP-Rate=" << float(detectionMetrics.fp) / (detectionMetrics.fp + detectionMetrics.tn)
            << endl;

    results << " TP=" << detectionMetrics.tp
            << " TN=" << detectionMetrics.tn
            << " FP=" << detectionMetrics.fp
            << " FN=" << detectionMetrics.fn
            << endl;

    results << "Testing took " << (endTime - startTime).total_milliseconds() << " ms" << endl;

    ROS_INFO_STREAM(results.str());

    if(detectionMetrics.tp + detectionMetrics.fp == 0 || detectionMetrics.tn + detectionMetrics.fn == 0) {
        ROS_WARN("All segments have been classified into same class, classifier is not discriminative!");
    }

    stringstream resultsFilename;
    resultsFilename << modelFilename.str() << ".txt";
    ofstream resultsFile(resultsFilename.str().c_str());
    resultsFile << results.str();

    ros::spinOnce();
}
