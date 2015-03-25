#include <srl_laser_detectors/detector_factory.h>
#include <srl_laser_detectors/naive_detectors/blob_detector.h>
#include <srl_laser_detectors/learned_detectors/svm_detector.h>
#include <srl_laser_detectors/learned_detectors/adaboost_detector.h>


namespace srl_laser_detectors {

Detector* createBlobDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) {
    return new BlobDetector(nodeHandle, privateNodeHandle);
}

Detector* createSVMDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) {
    return new SVMDetector(nodeHandle, privateNodeHandle);
}

Detector* createAdaboostDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) {
    return new AdaboostDetector(nodeHandle, privateNodeHandle);
}

void DetectorFactory::init()
{
    factoryMethods["blob"] = &createBlobDetector;
    factoryMethods["svm"]  = &createSVMDetector;
    factoryMethods["adaboost"]  = &createAdaboostDetector;
}


/// Map of detector types and the corresponding factory methods
std::map<DetectorType, FactoryMethod> DetectorFactory::factoryMethods;


} // end of namespace srl_laser_detectors