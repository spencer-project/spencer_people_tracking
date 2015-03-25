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

using namespace srl_laser_detectors;
using namespace std;

Segments trainingAndTestSegments;
Labels trainingAndTestLabels;
size_t numForegroundSegments = 0, numBackgroundSegments = 0, numAmbiguousSegments = 0;
bool stopListening = false;

/// CTRL+C handler    
void sigintHandler(int sgn __attribute__ ((unused)))
{
    stopListening = true;
}


/// Callback that is invoked everytime new training data (synchronized laserscan, segmentation and annotations) is received via ROS
void newTrainingDataReceived(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& annotations)
{
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

    // Compare each segment to the groundtruth annotations
    foreach(Segment& segment, segments) {
        // Count overlap in foreground / background points
        size_t numForegroundPoints = 0, numBackgroundPoints = 0;
        foreach(size_t pointIndex, segment.indices) {
            if(groundtruth[pointIndex] == FOREGROUND) numForegroundPoints++;
            else numBackgroundPoints++;
        }

        const size_t numTotalPoints = numForegroundPoints + numBackgroundPoints;
        const double foregroundRatio = numForegroundPoints / (double)numTotalPoints;
        const double backgroundRatio = numBackgroundPoints / (double)numTotalPoints;

        if(backgroundRatio >= backgroundThreshold) {
            trainingAndTestSegments.push_back(segment);
            trainingAndTestLabels.push_back(BACKGROUND);
            numBackgroundSegments++;
        }
        else if(foregroundRatio >= foregroundThreshold) {
            trainingAndTestSegments.push_back(segment);
            trainingAndTestLabels.push_back(FOREGROUND);
            numForegroundSegments++;
        } 
        else {
            numAmbiguousSegments++;
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
    ROS_INFO("Listening for training / test data (laserscans, segmentation, groundtruth segment annotations)... these need to have exactly matching timestamps!");
    ROS_INFO("Press CTRL+C to stop listening and proceed with the next phase.");
    while(!stopListening) ros::spinOnce();

    ROS_INFO("Starting training phase!");
    ros::spinOnce();

    //
    // Create training and test folds
    //
    srand(0 /*unsigned(time(0))*/);
    assert(trainingAndTestSegments.size() == trainingAndTestLabels.size());

    vector<size_t> randomIndices; randomIndices.resize(trainingAndTestSegments.size());
    for(size_t i = 0; i < trainingAndTestSegments.size(); i++) randomIndices[i] = i;
    random_shuffle(randomIndices.begin(), randomIndices.end());
    
    double testSetRatio = 0.1; // FIXME: parameter
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
    stringstream modelFilename;
    modelFilename << "learned_model_" << (long) ros::Time::now().toSec() << "." << type;

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
            << " FP-Rate=" << float(detectionMetrics.fp) / (detectionMetrics.fp + detectionMetrics.tp + detectionMetrics.fn + detectionMetrics.tn)
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
