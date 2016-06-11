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

#ifndef _SRL_LASER_DETECTORS_DETECTOR_FACTORY_H
#define _SRL_LASER_DETECTORS_DETECTOR_FACTORY_H

#include <string>
#include <map>
#include <sstream>
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>


namespace srl_laser_detectors {

// Forward declaration
class Detector;

// Detector factory method
typedef Detector* (*FactoryMethod)(ros::NodeHandle&, ros::NodeHandle&);

// String that identifies the type of a detector
typedef std::string DetectorType;


/// Factory for creating detector instances using a unified interface. See the cpp file for available detector types.
class DetectorFactory {
public:
    /// Initializes the factory, registers all available detectors. Must be called before invoking any other method.
    static void init();

    /// List all available types of detectors.
    static std::vector<DetectorType> listAvailableTypes() {
        std::vector<DetectorType> availableTypes;
        for(std::map<DetectorType, FactoryMethod>::const_iterator it = factoryMethods.begin(); it != factoryMethods.end(); ++it) {
            availableTypes.push_back(it->first);
        }
        return availableTypes;
    }

    /// Creates the detector with the specified type (see detector_factory.cpp for valid values).
    /// The node handles are supplied to the detector to create any additional subscribers or publishers, and to fetch parameters from the ROS parameter server.
    static boost::shared_ptr<Detector> createDetector(const DetectorType& type, ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle) {
        std::vector<DetectorType> availableTypes = DetectorFactory::listAvailableTypes();
        if(type.empty() || factoryMethods.find(type) == factoryMethods.end()) {
            std::stringstream ss;

            // Show list of available detector types
            for(int i = 0; i < availableTypes.size(); i++) {
                ss << "- " << availableTypes[i] << "\n";
            }

            ROS_FATAL_STREAM("Detector type not specified or inexistent. Example: _type:=blob. Available types are:\n" << ss.str());
            exit(EXIT_FAILURE);
        }

        FactoryMethod factoryMethod = factoryMethods[type];
        return boost::shared_ptr<Detector>( factoryMethod(nodeHandle, privateNodeHandle) );
    }

private:
    /// Map of detector types and the corresponding factory methods
    static std::map<DetectorType, FactoryMethod> factoryMethods;
};


} // end of namespace srl_laser_detectors

#endif // _SRL_LASER_DETECTORS_DETECTOR_FACTORY_H
