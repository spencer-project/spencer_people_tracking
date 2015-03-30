#ifndef _SPENCER_DIAGNOSTICS_STATUS_H
#define _SPENCER_DIAGNOSTICS_STATUS_H

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <boost/shared_ptr.hpp>
#include <boost/asio/ip/host_name.hpp>


namespace spencer_diagnostics {
    
    class Status {
    public:
        Status(std::string nameSuffix = "") {
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
            if(!s_publisher) {
                ros::NodeHandle nh;
                s_publisher.reset(new ros::Publisher(nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100, true)));
            }

            diagnostic_msgs::DiagnosticArray diagnosticArray;
            diagnosticArray.header.stamp = ros::Time::now();
            diagnosticArray.status.push_back(m_statusWrapper);
            s_publisher->publish(diagnosticArray);
        }

        diagnostic_updater::DiagnosticStatusWrapper& getWrappedMessage() {
            return m_statusWrapper;
        }

    private:
        static boost::shared_ptr<ros::Publisher> s_publisher;
        diagnostic_updater::DiagnosticStatusWrapper m_statusWrapper;
    };

}


#endif // _SPENCER_DIAGNOSTICS_STATUS_H