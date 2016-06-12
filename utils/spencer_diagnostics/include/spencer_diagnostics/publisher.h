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

#ifndef _SPENCER_DIAGNOSTICS_H
#define _SPENCER_DIAGNOSTICS_H

#include <diagnostic_updater/publisher.h>
#include <boost/shared_ptr.hpp>
#include <boost/asio/ip/host_name.hpp>


namespace spencer_diagnostics {
    /// A MonitoredPublisher is a thin wrapper around a ros::Publisher that also records and publishes diagnostic information
    /// such as average frequency and publish delay. This diagnostic information (diagnostic_msgs/DiagnosticArray) can be processed
    /// by diagnostic_aggregator and displayed by rqt_runtime_monitor and rqt_robot_monitor. If minimum and maximum expected frequency
    /// or maximum timestamp offset into the future and past are set, the diagnostic messages will have the warning status set
    /// if the wrapped publisher exceeds the frequency limits, and an error if it exceeds the maximum allowable timestamp offsets.
    /// 
    /// The MonitoredPublisher internally works as follows: Whenever monitoredPublisher.publish() is invoked,
    /// it calls the tick() method of an internal diagnostic_updater::TopicDiagnostic instance. Also, it invokes the
    /// update() method of a diagnostic_updater::Updater, which checks if the period specified by the ~diagnostic_period private
    /// parameter on the ROS parameter server has expired, and if yes, publishes a new diagnostic status.
    ///
    /// The recommended usage is as follows:
    ///   spencer_diagnostics::MonitoredPublisher monitoredPublisher;
    ///   monitoredPublisher = nodeHandle.advertise(...);
    ///   monitoredPublisher.setExpectedFrequency(20.0, 30.0);
    ///   monitoredPublisher.setMaximumTimestampOffset(0.5, 0.5);
    ///   monitoredPublisher.finalizeSetup();
    /// 
    /// This means that to convert an existing ros::Publisher into one that also offers diagnostic information, you usually just
    /// need to change the declared type from ros::Publisher into spencer_diagnostics::MonitoredPublisher, and optionally
    /// specify expected limits as shown.
    class MonitoredPublisher {
    public:
        /// Creates a monitored publisher, and initializes a new diagnostic_updater::Updater internally which
        /// retrieves its update frequency from the ROS parameter server (~diagnostic_period). The hardware ID
        /// is set to the host name of the computer running this component.
        /// This constructor is mainly useful if you only need one single MonitoredPublisher.
        MonitoredPublisher() {
            initDefaults();
        }

        /// Creates a monitored publisher that uses the provided diagnostic_updater::Updater.
        /// Useful if creating multiple MonitoredPublisher instances that shall share one diagnostic updater.
        MonitoredPublisher(boost::shared_ptr<diagnostic_updater::Updater> diagnosticUpdater) : m_diagnosticUpdater(diagnosticUpdater) {
            initDefaults();
        };

        /// Either set min and max frequency to different values, or set both to the same value and specify a tolerance (e.g. 0.1 = +/- 10%).
        void setExpectedFrequency(double minFrequency, double maxFrequency, double tolerance = 0, int windowSize = 5) {
            ROS_ASSERT(minFrequency <= maxFrequency);
            m_minFrequency = minFrequency;
            m_maxFrequency = maxFrequency;

            if(tolerance != m_frequencyTolerance || windowSize != m_windowSize) {
                ROS_ASSERT_MSG(!m_setupFinalized, "Cannot change parameters of MonitoredPublisher once finalizeSetup() has been called!");
            }

            m_frequencyTolerance = tolerance;
            m_windowSize = windowSize;
        }

        /// Sets limits for monitoring published message header timestamps. Useful to raise an error if the
        /// publishing component is lagging behind too much, which could endanger system safety.
        void setMaximumTimestampOffset(double maxSecondsIntoPast, double maxSecondsIntoFuture) {
            ROS_ASSERT_MSG(!m_setupFinalized, "Cannot change parameters of MonitoredPublisher once finalizeSetup() has been called!");
            m_maxSecondsIntoFuture = maxSecondsIntoFuture;
            m_maxSecondsIntoPast   = maxSecondsIntoPast;
        }

        /// Assigns a ros::Publisher that shall be monitored. Must be called before invoking any of the publish() methods.
        void operator=(ros::Publisher publisher) {
            ROS_ASSERT_MSG(!m_setupFinalized, "Cannot change publisher once finalizeSetup() has been called!");
            m_publisher = publisher;
            m_publisherSet = true;
        }

        /// Finalizes the setup, and creates a diagnostic_updater::Updater(). Can only be called once, and must be
        /// called before invoking any of the publish() methods. Requires that a ros::Publisher has been assigned using the assignment operator.
        void finalizeSetup() {
            ROS_ASSERT_MSG(!m_setupFinalized, "finalizeSetup() must only be called once");
            ROS_ASSERT_MSG(m_publisherSet, "No ros::publisher has been assigned to this MonitoredPublisher (using = operator)");

            // If DiagnosticUpdater was not provided by the user, create a new one. This must happen here, as opposed to
            // in the default constructor, since the updater creates node handles and it must be ensured that the user
            // has called ros::init() prior to that.
            if(!m_diagnosticUpdater) {
                m_diagnosticUpdater.reset(new diagnostic_updater::Updater());
                m_diagnosticUpdater->setHardwareID( boost::asio::ip::host_name() );
            }

            diagnostic_updater::FrequencyStatusParam frequencyParam(&m_minFrequency, &m_maxFrequency, m_frequencyTolerance, m_windowSize);
            diagnostic_updater::TimeStampStatusParam timestampParam(-m_maxSecondsIntoFuture, m_maxSecondsIntoPast);
        
            m_topicDiagnostic.reset( new diagnostic_updater::TopicDiagnostic(getTopic(), *m_diagnosticUpdater, frequencyParam, timestampParam) );
            m_setupFinalized = true;
        }


        /* --- Wrapped methods of ros::Publisher --- */

        template <typename M>
        void publish(const boost::shared_ptr<M>& message) const {
            ROS_ASSERT_MSG(m_setupFinalized, "You must call finalizeSetup() first on this MonitoredPublisher instance before you can publish messages!");
            m_publisher.publish(message);

            // record for diagnostics
            m_topicDiagnostic->tick(message->header.stamp);
            m_diagnosticUpdater->update();
        }

        template <typename M>
        void publish(const M& message) const {
            ROS_ASSERT_MSG(m_setupFinalized, "You must call finalizeSetup() first on this MonitoredPublisher instance before you can publish messages!");
            m_publisher.publish(message);

            // record for diagnostics
            m_topicDiagnostic->tick(message.header.stamp);
            m_diagnosticUpdater->update();
        }

        std::string getTopic() const {
            ROS_ASSERT(m_publisherSet);
            return m_publisher.getTopic();
        }

        uint32_t getNumSubscribers() const {
            ROS_ASSERT(m_publisherSet);
            return m_publisher.getNumSubscribers();
        }


    private:
        void initDefaults() {
            m_setupFinalized = m_publisherSet = false;
            setExpectedFrequency(0, std::numeric_limits<double>::max());
            setMaximumTimestampOffset(10, 1);
        }

        double m_minFrequency, m_maxFrequency, m_frequencyTolerance, m_maxSecondsIntoPast, m_maxSecondsIntoFuture;
        int m_windowSize;
        boost::shared_ptr<diagnostic_updater::Updater> m_diagnosticUpdater;
        boost::shared_ptr<diagnostic_updater::TopicDiagnostic> m_topicDiagnostic;
        ros::Publisher m_publisher; /// the wrapped publisher
        bool m_setupFinalized, m_publisherSet;
    };
}


#endif // _SPENCER_DIAGNOSTICS_H