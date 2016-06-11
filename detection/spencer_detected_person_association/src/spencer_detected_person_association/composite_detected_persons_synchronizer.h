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

#ifndef _COMPOSITE_DETECTED_PERSONS_SYNCHRONIZER_H
#define _COMPOSITE_DETECTED_PERSONS_SYNCHRONIZER_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <spencer_tracking_msgs/CompositeDetectedPersons.h>


namespace spencer_detected_person_association
{
    /// Abstract base class that synchronizes multiple spencer_tracking_msgs/CompositeDetectedPersons input topics, and creates a publisher for outputting spencer_tracking_msgs/CombinedDetectedPersons.
    /// The specialty of this class is that it automatically reconfigures its message filters (synchronizers) if any of the subscribed topics goes down, or a new one becomes available.
    /// This means that if e.g. the rear laser detector goes down, the front laser detections will still be output. Normally, this would cause the output to stop completely.
    class CompositeDetectedPersonsSynchronizer
    {
    public:
        /// Must be called by derived class in onInit().
        void initSynchronizer(const std::string& nodeletName, ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle, int minNumInputTopics = 1, int maxNumInputTopics = 9);

        virtual ~CompositeDetectedPersonsSynchronizer();


    protected:
        /// Must be overriden by derived classes to process the input messages.
        virtual void onNewInputMessagesReceived(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& inputMsgs) = 0;

        // Publisher for output spencer_tracking_msgs::CompositeDetectedPersons
        boost::shared_ptr< ros::Publisher > m_publisher;

    private:
        //
        // Synchronizers, subscribers and callbacks
        //

        // -- Variables used regardless of number of input topics -- //
        ros::NodeHandle m_nodeHandle, m_privateNodeHandle;

        typedef message_filters::Subscriber< spencer_tracking_msgs::CompositeDetectedPersons > SubscriberType;
        boost::ptr_vector< SubscriberType > m_subscribers;
        message_filters::Connection m_currentCallback;

        typedef std::vector<SubscriberType*> ActiveTopics;
        ActiveTopics m_previouslyActiveTopics;

        int m_agePenalty, m_queueSize;
        double m_topicMonitorInterval;
        boost::thread m_monitorThread;
        boost::mutex m_monitorMutex;

        std::string m_nodeletName;


        // --- Synchronizer management --- //

        // Executed in a separate thread, continuously monitors if the number of active topics has changed.
        void monitorActiveTopics();

        // Determines which of the subscribed topics are active (=have publishers)
        ActiveTopics determineActiveTopics();

        // Sets up new synchronizers if the count of active topics has changed
        void setupSynchronizers(ActiveTopics& activeTopics);


        // --- Callback handling --- //

        void handleNewInputMessages(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& inputMsgs);

        // --- For ONE input topic (=no synchronization!) -- //
        void onSingleInputMessageReceived(spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr msg);

        // --- For TWO input topics -- //
        void onTwoInputMessagesReceived(spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr msg1, spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr msg2);

        typedef message_filters::sync_policies::ApproximateTime<spencer_tracking_msgs::CompositeDetectedPersons, spencer_tracking_msgs::CompositeDetectedPersons> SyncPolicyWithTwoInputs;
        typedef message_filters::Synchronizer<SyncPolicyWithTwoInputs> SynchronizerWithTwoInputs;
        boost::shared_ptr< SynchronizerWithTwoInputs > m_synchronizerWithTwoInputs;
    };
}


#endif // _COMPOSITE_DETECTED_PERSONS_SYNCHRONIZER_H
