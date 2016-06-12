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

#ifndef _SPENCER_DIAGNOSTICS_STATUS_H
#define _SPENCER_DIAGNOSTICS_STATUS_H

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <boost/shared_ptr.hpp>
#include <boost/asio/ip/host_name.hpp>


namespace spencer_diagnostics {
    
    class Status {
    private:
        diagnostic_updater::DiagnosticStatusWrapper m_statusWrapper;
        std::string m_nameSuffix;
        static std::map<std::string, boost::shared_ptr<ros::Publisher> > s_publisherMap;

    public:
        Status(std::string nameSuffix = "") {
            m_nameSuffix = nameSuffix;
            m_statusWrapper.hardware_id = boost::asio::ip::host_name();
            m_statusWrapper.name = ros::this_node::getName().substr(1) + nameSuffix;
        }

        void addError(const std::string& summary) {
            m_statusWrapper.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, summary);
        }

        void addWarning(const std::string& summary) {
            m_statusWrapper.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, summary);
        }

        /*
        void addStale(const std::string& summary) {
            m_statusWrapper.mergeSummary(diagnostic_msgs::DiagnosticStatus::STALE, summary);
        }
        */

        void addOK(const std::string& summary) {
            m_statusWrapper.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, summary);
        }

        void publish() {
            if(s_publisherMap.find(m_nameSuffix) == s_publisherMap.end()) {
                ros::NodeHandle nh;
                s_publisherMap[m_nameSuffix] = boost::shared_ptr<ros::Publisher>(new ros::Publisher(nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 50, false)));
            }

            diagnostic_msgs::DiagnosticArray diagnosticArray;
            diagnosticArray.header.stamp = ros::Time::now();
            diagnosticArray.status.push_back(m_statusWrapper);
            s_publisherMap[m_nameSuffix]->publish(diagnosticArray);
        }

        diagnostic_updater::DiagnosticStatusWrapper& getWrappedMessage() {
            return m_statusWrapper;
        }
    };

    // Static definition
    std::map<std::string, boost::shared_ptr<ros::Publisher> > Status::s_publisherMap;
}

#endif // _SPENCER_DIAGNOSTICS_STATUS_H