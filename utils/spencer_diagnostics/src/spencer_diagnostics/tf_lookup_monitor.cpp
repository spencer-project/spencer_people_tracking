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

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <spencer_control_msgs/ComponentStatus.h>

#include <boost/asio/ip/host_name.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/foreach.hpp>


#include <vector>
#include <string>
#include <sstream>

#define foreach BOOST_FOREACH


using namespace std;
using namespace diagnostic_msgs;

typedef pair<ros::Time, ros::Time> TimePair;

struct DataForTransform {
    boost::circular_buffer< TimePair > timestampBuffer;
    ros::Time lastTransformAt;
    std::string sourceFrame;
    std::string targetFrame;
    double timeout;
    double maxAvgDelay;
    double minAvgRate;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_lookup_monitor");

    ros::NodeHandle nodeHandle, privateHandle("~");
    ros::Publisher pub = nodeHandle.advertise<DiagnosticArray>("/diagnostics", 1, true);
    ros::Publisher componentStatusPub = nodeHandle.advertise<spencer_control_msgs::ComponentStatus>("/spencer/diagnostics/tf_status", 30);
    
    tf::TransformListener tfListener;

    string hostname = boost::asio::ip::host_name();

    //
    // Read parameters
    //
    vector<string> transformStrings;
    privateHandle.getParam("transforms", transformStrings);

    double timeout = 1.0; vector<double> timeouts;
    privateHandle.getParam("timeout", timeout);
    privateHandle.getParam("timeouts", timeouts);

    double maxAvgDelay = 0.15; vector<double> maxAvgDelays;
    privateHandle.getParam("max_avg_delay", maxAvgDelay);
    privateHandle.getParam("max_avg_delays", maxAvgDelays);

    double minAvgRate = 0.0; vector<double> minAvgRates;
    privateHandle.getParam("min_avg_rate", minAvgRate);
    privateHandle.getParam("min_avg_rates", minAvgRates);

    int windowSize = 20;
    int updateRate = 50, publishRate = 5;
    ros::Rate rate(updateRate);
    
    //
    // Verify parameters
    //
    if(transformStrings.empty()) {
        ROS_ERROR("Transforms parameter must not be empty!");
        return 1;
    }

    if(timeouts.empty()) timeouts = vector<double>(transformStrings.size(), timeout);
    if(maxAvgDelays.empty()) maxAvgDelays = vector<double>(transformStrings.size(), maxAvgDelay);
    if(minAvgRates.empty()) minAvgRates = vector<double>(transformStrings.size(), minAvgRate);
       
    if(timeouts.size() != transformStrings.size()) {
        ROS_ERROR("Number of elements in 'timeouts' list must be equal to number of elements in 'transforms' list!");
        return 1;
    }
    if(maxAvgDelays.size() != transformStrings.size()) {
        ROS_ERROR("Number of elements in 'maxAvgDelays' list must be equal to number of elements in 'transforms' list!");
        return 1;
    }
    if(minAvgRates.size() != transformStrings.size()) {
        ROS_ERROR("Number of elements in 'minAvgRates' list must be equal to number of elements in 'transforms' list!");
        return 1;
    }

    // Setup transform memory
    int i = 0;
    vector<DataForTransform> transforms;
    foreach(std::string& transformString, transformStrings) {
        vector<string> tokens;
        boost::split_regex(tokens, transformString, boost::regex("-->"));

        DataForTransform dataForTransform;
        dataForTransform.sourceFrame = tokens[0];
        dataForTransform.targetFrame = tokens[1];

        dataForTransform.timestampBuffer.rset_capacity(windowSize);
        dataForTransform.timeout = timeouts[i];
        dataForTransform.maxAvgDelay = maxAvgDelays[i];
        dataForTransform.minAvgRate = minAvgRates[i];

        boost::trim(dataForTransform.sourceFrame, locale(""));
        boost::trim(dataForTransform.targetFrame, locale(""));

        if(dataForTransform.sourceFrame.empty() || dataForTransform.targetFrame.empty()) continue;

        ROS_INFO_STREAM("Monitoring transform: " << dataForTransform.sourceFrame << " to " << dataForTransform.targetFrame << ", with a transform timeout of " << dataForTransform.timeout
            << ", max avg delay " << dataForTransform.maxAvgDelay << ", min avg rate " << dataForTransform.minAvgRate);

        transforms.push_back(dataForTransform);
    }

    //
    // Main update loop
    //
    int publishCounter = 0;
    while(ros::ok()) {
        rate.sleep();

        publishCounter++;
        bool doPublish = publishCounter % int(updateRate / publishRate) == 0;

        ros::Time now = ros::Time::now();
        DiagnosticArray diagnosticArray;
        diagnosticArray.header.stamp = now;

        // Iterate over all monitored transforms
        foreach(DataForTransform& transform, transforms)
        {
            // Lookup TF transform
            std::string errorString;
            transform.lastTransformAt = ros::Time(0);
            tfListener.getLatestCommonTime(transform.sourceFrame, transform.targetFrame, transform.lastTransformAt, &errorString);
            transform.timestampBuffer.push_back( make_pair( transform.lastTransformAt, now) );

            // Compute elapsed time
            ros::Duration elapsedSinceLastTransform = now - transform.lastTransformAt;
            bool hasTimeout = elapsedSinceLastTransform > ros::Duration(transform.timeout);
            bool reallyTimedOut = elapsedSinceLastTransform > ros::Duration(3*transform.timeout);

            // Compute average time delay and publish rate
            double averageDelay = 0.0, maxDelay = 0.0;
            double averageInterval = 0.0;

            ros::Time previousTimestamp;
            int numIntervals = 0;
            foreach(TimePair p, transform.timestampBuffer) {
                ros::Time sentTimestamp = p.first;
                ros::Time receivedTimestamp = p.second;
                double delay = (receivedTimestamp - sentTimestamp).toSec();
                averageDelay += delay;
                maxDelay = std::max(delay, maxDelay);

                if(sentTimestamp != previousTimestamp) { 
                    if(previousTimestamp != ros::Time(0)) {
                        double interval = (sentTimestamp - previousTimestamp).toSec();
                        averageInterval += interval;
                        numIntervals++;
                    }

                    previousTimestamp = sentTimestamp;
                }
            }

            // Do not publish at every iteration, to reduce load
            if(doPublish) {
                DiagnosticStatus tfStatus;
                tfStatus.name = "TF link status: " + transform.sourceFrame + " --> " + transform.targetFrame;
                tfStatus.hardware_id = hostname;
                tfStatus.level = transform.lastTransformAt == ros::Time(0) ? DiagnosticStatus::WARN : (hasTimeout ? DiagnosticStatus::ERROR : DiagnosticStatus::OK);

                stringstream ss;

                if(transform.timestampBuffer.size() > 1) {
                    averageDelay /= (double) transform.timestampBuffer.size();
                    averageInterval /= (double) numIntervals;
                
                    KeyValue kv;

                    ss << transform.lastTransformAt.toSec();
                    kv.key = "Last received at";
                    kv.value = ss.str();
                    tfStatus.values.push_back(kv);

                    ss.str(""); ss << transform.timestampBuffer.size() << " messages";
                    kv.key = "Observation window size:";
                    kv.value = ss.str();
                    tfStatus.values.push_back(kv);

                    ss.str(""); ss << fixed << setprecision(3) << averageInterval << " sec";
                    kv.key = "Average interval between messages";
                    kv.value = ss.str();
                    tfStatus.values.push_back(kv);

                    ss.str(""); ss << fixed << setprecision(3) << maxDelay << " sec";
                    kv.key = "Maximum delay wrt. current ROS time";
                    kv.value = ss.str();
                    tfStatus.values.push_back(kv);

                    ss.str(""); ss << fixed << setprecision(3) << averageDelay << " sec";
                    kv.key = "Average delay wrt. current ROS time";
                    kv.value = ss.str();
                    tfStatus.values.push_back(kv);
                }

                if(transform.lastTransformAt == ros::Time(0)) {
                    tfStatus.message = "Missing, never received any transform";
                }
                else if(hasTimeout) {
                    ss.str(""); ss << "Timed out " << fixed << setprecision(2) << (elapsedSinceLastTransform.toSec() - transform.timeout) << " sec ago";
                    tfStatus.message = ss.str();
                }
                else {
                    ss.str(""); ss << "Active, receiving transforms (avg. delay " << fixed << setprecision(3) << averageDelay << " sec) at " << setprecision(1) << (1.0 / averageInterval) << " Hz";
                    tfStatus.message = ss.str();
                }

                if(transform.timestampBuffer.size() > 1) {
                    if(transform.maxAvgDelay > 0.0) {
                        ss.str(""); ss << fixed << setprecision(3) << transform.maxAvgDelay << " sec";
                        KeyValue kv;
                        kv.key = "Max. allowed avg delay";
                        kv.value = ss.str();
                        tfStatus.values.push_back(kv);

                        if(!hasTimeout && averageDelay > transform.maxAvgDelay) {
                            tfStatus.message += ", high delay";
                            if(tfStatus.level == DiagnosticStatus::OK)
                                tfStatus.level = DiagnosticStatus::WARN;
                        }
                    }

                    if(transform.minAvgRate > 0.0) {
                        ss.str(""); ss << fixed << setprecision(1) << transform.minAvgRate << " Hz";
                        KeyValue kv;
                        kv.key = "Min. allowed rate";
                        kv.value = ss.str();
                        tfStatus.values.push_back(kv);

                        if(!hasTimeout && 1.0 / averageInterval < transform.minAvgRate) {
                            tfStatus.message += ", low rate";
                            if(tfStatus.level == DiagnosticStatus::OK)
                                tfStatus.level = DiagnosticStatus::WARN;
                        }
                    }
                }

                diagnosticArray.status.push_back(tfStatus);

                spencer_control_msgs::ComponentStatus componentStatus;
                componentStatus.name = transform.sourceFrame + " --> " + transform.targetFrame;
                componentStatus.alive = !reallyTimedOut;
                componentStatus.detail = tfStatus.message;
                componentStatusPub.publish(componentStatus);

            } // end if publishCounter
        } // end foreach transform

        pub.publish(diagnosticArray);
    } // end while ros::ok
}
