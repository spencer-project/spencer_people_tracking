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

#include <pluginlib/class_list_macros.h>
#include "composite_detected_persons_synchronizer.h"

#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace spencer_detected_person_association
{
    void CompositeDetectedPersonsSynchronizer::initSynchronizer(const std::string& nodeletName, ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle, int minNumInputTopics, int maxNumInputTopics)
    {
        // Save variables
        m_nodeletName = nodeletName;
        m_nodeHandle = nodeHandle;
        m_privateNodeHandle = privateNodeHandle;

        // Retrieve list of input topics from parameter server
        std::vector<std::string> inputTopics;
        privateNodeHandle.getParam("input_topics", inputTopics);

        ROS_ASSERT_MSG(inputTopics.size() >= minNumInputTopics, "At least %d input topics are required, but only %d were provided!", minNumInputTopics, (int)inputTopics.size() );
        ROS_ASSERT_MSG(inputTopics.size() <= maxNumInputTopics, "At most %d input topics are allowed, but %d were provided!", maxNumInputTopics, (int)inputTopics.size() );

        // Create message_filters::Subscriber<CompositeDetectedPersons> instances for all specified topics
        std::stringstream ss;
        foreach(std::string inputTopic, inputTopics) {
            boost::algorithm::trim(inputTopic);
            if(!inputTopic.empty()) m_subscribers.push_back(new SubscriberType(nodeHandle, inputTopic, 1));
            ss << "'" << m_nodeHandle.resolveName(inputTopic) << "' ";
        }

        // Read parameters
        m_agePenalty = 1000; // Set high age penalty to publish older data faster even if it might not be correctly synchronized.
        privateNodeHandle.getParam("synchronizer_age_penalty", m_agePenalty);

        m_queueSize = 10; // TODO: Find reasonable default
        privateNodeHandle.getParam("synchronizer_queue_size", m_queueSize);

        m_topicMonitorInterval = 3.0; // How often to check for new topics becoming active or topics going inactive, in seconds
        privateNodeHandle.getParam("topic_monitor_interval", m_topicMonitorInterval);

        ROS_INFO_STREAM_NAMED(m_nodeletName, "Synchronizing topics " << ss.str() << "with queue size " << m_queueSize << ", age penalty " << m_agePenalty);

        // Create monitor thread which monitors topics (=publishers) becoming active
        m_monitorThread = boost::thread(&CompositeDetectedPersonsSynchronizer::monitorActiveTopics, this);

    }


    CompositeDetectedPersonsSynchronizer::~CompositeDetectedPersonsSynchronizer()
    {
        m_monitorThread.interrupt();
        m_monitorThread.join();
    }


    void CompositeDetectedPersonsSynchronizer::monitorActiveTopics()
    {
        boost::posix_time::seconds monitorInterval( m_topicMonitorInterval );

        while(true) {
            // Get currently active topics
            ActiveTopics activeTopics = determineActiveTopics();

            // Check if anything has changed
            if(activeTopics != m_previouslyActiveTopics)
            {
                // Display info
                std::stringstream ss; ss << std::endl;
                foreach(SubscriberType* subscriber, activeTopics) {
                    ss << "- " << subscriber->getTopic() << std::endl;
                }
                ROS_INFO_NAMED(m_nodeletName, "Number of active input topics has changed from %zu to %zu. Re-configuring message filters. Active inputs for output topic '%s' are now: %s",
                    m_previouslyActiveTopics.size(), activeTopics.size(), m_nodeHandle.resolveName("output").c_str(), ss.str().c_str());

                // Assure thread safety
                boost::mutex::scoped_lock lock(m_monitorMutex);

                // Re-create synchronizers
                m_previouslyActiveTopics = activeTopics;
                setupSynchronizers(activeTopics);

                // Create or destroy publisher if there are any / no input topics active
                if(activeTopics.empty()) {
                    // Unregister publisher if no subscriptions exist any more
                    m_publisher.reset();
                }
                else if(!m_publisher) {
                    // Create publisher for output composite detected persons
                    m_publisher.reset(new ros::Publisher( m_nodeHandle.advertise<spencer_tracking_msgs::CompositeDetectedPersons>("output", 5) ));
                }
            }

            // Sleep for a while
            try { boost::this_thread::sleep(monitorInterval); }
            catch(boost::thread_interrupted&) { return; } // stop thread
        }
    }


    CompositeDetectedPersonsSynchronizer::ActiveTopics CompositeDetectedPersonsSynchronizer::determineActiveTopics()
    {
        // Find out which topics are currently being published
        ActiveTopics activeTopics, previouslyActiveTopics = m_previouslyActiveTopics;
        foreach(SubscriberType& subscriber, m_subscribers) {
            if(subscriber.getSubscriber().getNumPublishers() > 0) {
                activeTopics.push_back(&subscriber);

                ActiveTopics::iterator previouslyExisting = std::find(previouslyActiveTopics.begin(), previouslyActiveTopics.end(), &subscriber);
                if(previouslyExisting == previouslyActiveTopics.end()) {
                    ROS_INFO_STREAM_NAMED(m_nodeletName, "New active topic registered: " << m_nodeHandle.resolveName(subscriber.getTopic()));
                }
                else {
                    previouslyActiveTopics.erase(previouslyExisting);
                }
            }
        }

        // All topics still in previouslyActiveTopics have no active publisher any more.
        foreach(SubscriberType* oldSubscriber, previouslyActiveTopics) {
            ROS_WARN_STREAM_NAMED(m_nodeletName, "Unregistering subscriber as no publisher exists anymore for topic: " << m_nodeHandle.resolveName(oldSubscriber->getTopic()));
        }

        return activeTopics;
    }


    void CompositeDetectedPersonsSynchronizer::setupSynchronizers(ActiveTopics& activeTopics)
    {
        // Disconnect old callbacks
        m_currentCallback.disconnect();

        // Create sync policy and synchronizer
        switch(activeTopics.size()) {
        case 0: {
            break;
        }
        case 1: {
            m_currentCallback = activeTopics[0]->registerCallback(&CompositeDetectedPersonsSynchronizer::onSingleInputMessageReceived, this);
            break;
        }
        case 2: {
            SyncPolicyWithTwoInputs syncPolicyWithTwoInputs(m_queueSize);
            syncPolicyWithTwoInputs.setAgePenalty(m_agePenalty);
            const SyncPolicyWithTwoInputs constSyncPolicyWithTwoInputs = syncPolicyWithTwoInputs;

            m_synchronizerWithTwoInputs.reset(new SynchronizerWithTwoInputs(constSyncPolicyWithTwoInputs, *activeTopics[0], *activeTopics[1]));
            m_currentCallback = m_synchronizerWithTwoInputs->registerCallback(&CompositeDetectedPersonsSynchronizer::onTwoInputMessagesReceived, this);
            break;
        }
        default:
            ROS_ASSERT_MSG(false, "CompositeDetectedPersonsSynchronizer not implemented for %zu input topics!", activeTopics.size());
        }
    }


    void CompositeDetectedPersonsSynchronizer::handleNewInputMessages(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& inputMsgs)
    {
        // Make sure that the publisher does not get destroyed while following method is using it
        boost::mutex::scoped_lock lock(m_monitorMutex);

        // Invoke method implemented by derived class.
        onNewInputMessagesReceived(inputMsgs);
    }


    void CompositeDetectedPersonsSynchronizer::onSingleInputMessageReceived(spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr msg)
    {
        std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr> msgs;
        msgs.push_back(msg);
        handleNewInputMessages(msgs);
    }


    void CompositeDetectedPersonsSynchronizer::onTwoInputMessagesReceived(spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr msg1, spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr msg2)
    {
        std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        handleNewInputMessages(msgs);
    }

}
