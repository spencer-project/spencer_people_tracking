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
#include <srl_laser_detectors/naive_detectors/blob_detector.h>
#include <srl_laser_detectors/learned_detectors/svm_detector.h>
#include <srl_laser_detectors/learned_detectors/adaboost_detector.h>
#include <srl_laser_detectors/learned_detectors/random_forest_detector.h>


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

Detector* createRandomForestDetector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) {
    return new RandomForestDetector(nodeHandle, privateNodeHandle);
}

void DetectorFactory::init()
{
    factoryMethods["blob"] = &createBlobDetector;
    factoryMethods["svm"]  = &createSVMDetector;
    factoryMethods["adaboost"]  = &createAdaboostDetector;
    factoryMethods["random_forest"] = &createRandomForestDetector;
}


/// Map of detector types and the corresponding factory methods
std::map<DetectorType, FactoryMethod> DetectorFactory::factoryMethods;


} // end of namespace srl_laser_detectors
