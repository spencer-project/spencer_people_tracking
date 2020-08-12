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
#include "convert_to_composite_detections.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace spencer_detected_person_association
{
    void ConvertToCompositeDetectionsNodelet::onInit()
    {
        NODELET_DEBUG("Initializing ConvertToCompositeDetectionsNodelet...");

        getPrivateNodeHandle().getParam("common_frame_id", m_commonFrameId);
        m_transformListener.reset(new tf::TransformListener());
        m_lastMessageReceivedAt = ros::Time(0);

        // Subscribe to input topic
        m_subscriber.reset(new ros::Subscriber( getNodeHandle().subscribe<spencer_tracking_msgs::DetectedPersons>("input", 2, &ConvertToCompositeDetectionsNodelet::onNewInputMessageReceived, this) ));

        // Read parameters

        m_topicMonitorInterval = 3.0; // How often to check for new topics becoming active or topics going inactive, in seconds
        getPrivateNodeHandle().getParam("topic_monitor_interval", m_topicMonitorInterval);

        // If no input msg is received from any publisher within this amount of seconds,
        // assume all inputs are dead (e.g. due to detector crash) --> shutdown output topic to not block aggregator/fuser nodes
        m_assumeTopicDeadAfter = 9.0;
        getPrivateNodeHandle().getParam("assume_topic_dead_after", m_assumeTopicDeadAfter);

        // Create monitor thread which monitors topics (=publishers) becoming active
        m_monitorThread = boost::thread(&ConvertToCompositeDetectionsNodelet::monitorInputTopic, this);
    }


    ConvertToCompositeDetectionsNodelet::~ConvertToCompositeDetectionsNodelet()
    {
        m_monitorThread.interrupt();
        m_monitorThread.join();
    }


    void ConvertToCompositeDetectionsNodelet::createPublisher() {
        m_publisher.reset(new ros::Publisher( getNodeHandle().advertise<spencer_tracking_msgs::CompositeDetectedPersons>("output", 2) ));
    }


    void ConvertToCompositeDetectionsNodelet::monitorInputTopic()
    {
        boost::chrono::milliseconds monitorInterval( int(m_topicMonitorInterval * 1000.0) );

        while(true) {
            bool deadInputTopic = ros::Time::now().toSec() - m_lastMessageReceivedAt.toSec() > m_assumeTopicDeadAfter;
            if(m_subscriber->getNumPublishers() == 0 || deadInputTopic) {
                // Unregister publisher if our input topic has become inactive
                if(m_publisher) {
                    ROS_ERROR_STREAM_COND(deadInputTopic, "Input topic " << getNodeHandle().resolveName(m_subscriber->getTopic())
                      << " appears dead (no new messages received since " << std::fixed << std::setprecision(1)
                      << m_assumeTopicDeadAfter << " seconds)! Maybe the detector has crashed? Shutting down CompositeDetectedPersons publisher.");

                    boost::mutex::scoped_lock lock(m_monitorMutex);
                    m_publisher.reset();
                }
            }
            else {
                if(!m_publisher) {
                    boost::mutex::scoped_lock lock(m_monitorMutex);
                    createPublisher();
                }
            }

            // Sleep for a while
            try { boost::this_thread::sleep_for(monitorInterval); }
            catch(boost::thread_interrupted&) { return; } // stop thread
        }
    }


    void ConvertToCompositeDetectionsNodelet::onNewInputMessageReceived(const spencer_tracking_msgs::DetectedPersons::ConstPtr& inputMsg)
    {
        spencer_tracking_msgs::CompositeDetectedPersons::Ptr outputMsg(new spencer_tracking_msgs::CompositeDetectedPersons);

        // Remember current timestamp for topic monitoring
        m_lastMessageReceivedAt = ros::Time::now();

        // Copy stamp, frame ID, seq
        outputMsg->header = inputMsg->header;

        bool needTransform = false;
        Eigen::Affine3d eigenTransform;

        if(!m_commonFrameId.empty() && inputMsg->header.frame_id != m_commonFrameId) {
            needTransform = true;
            outputMsg->header.frame_id = m_commonFrameId;

            try {
              tf::StampedTransform transform;
              m_transformListener->waitForTransform(m_commonFrameId, inputMsg->header.frame_id, inputMsg->header.stamp, ros::Duration(0.1));
              m_transformListener->lookupTransform( m_commonFrameId, inputMsg->header.frame_id, inputMsg->header.stamp, transform);
              tf::transformTFToEigen(transform, eigenTransform);
            }
            catch (tf::TransformException ex){
              ROS_WARN_STREAM_THROTTLE(5.0, "Failed to lookup transform from frame " << inputMsg->header.frame_id << " into target frame " << m_commonFrameId << "! Dropping input DetectedPersons message on topic "
                << getNodeHandle().resolveName(m_subscriber->getTopic()) << ". Reason: " << ex.what());
              return;
            }
        }

        foreach(const spencer_tracking_msgs::DetectedPerson& detectedPerson, inputMsg->detections)
        {
            spencer_tracking_msgs::CompositeDetectedPerson compositeDetectedPerson;

            // ID
            compositeDetectedPerson.composite_detection_id = detectedPerson.detection_id;

            // Confidence
            compositeDetectedPerson.mean_confidence = compositeDetectedPerson.max_confidence = compositeDetectedPerson.min_confidence = detectedPerson.confidence;

            // Pose
            if(!needTransform) {
                compositeDetectedPerson.pose = detectedPerson.pose;
            }
            else {
                Eigen::Affine3d poseInSourceFrame;
                tf::poseMsgToEigen(detectedPerson.pose.pose, poseInSourceFrame);

                // ROS Covariance is a 6x6 matrix (xyz + xyz rotation) relative to the message's coordinate frame
                // We are not interested in pose rotation, so only take first 3 rows and columns
                Eigen::Matrix3d covInSourceFrame;
                for(int row = 0; row < 3; row++) {
                    for(int col = 0; col < 3; col++) {
                        covInSourceFrame(row, col) = detectedPerson.pose.covariance[row * 6 + col];
                    }
                }

                // Transform pose into target frame
                Eigen::Affine3d poseInTargetFrame = eigenTransform * poseInSourceFrame;
                tf::poseEigenToMsg(poseInTargetFrame, compositeDetectedPerson.pose.pose);

                // For transforming the covariance, only the coordinate frame rotation is relevant (invariant w.r.t. translation)
                Eigen::Matrix3d sourceFrameToTargetFrameRotation = eigenTransform.linear().matrix();
                Eigen::Matrix3d covInTargetFrame = sourceFrameToTargetFrameRotation * covInSourceFrame * sourceFrameToTargetFrameRotation.transpose();

                // Copy covariance back
                for(int row = 0; row < 3; row++) {
                    for(int col = 0; col < 3; col++) {
                        compositeDetectedPerson.pose.covariance[row * 6 + col] = covInTargetFrame(row, col);
                    }
                }
                for(int row = 3; row < 6; row++) for(int col = 3; col < 6; col++) compositeDetectedPerson.pose.covariance[row * 6 + col] = 999999;
            }

            // Original detection
            spencer_tracking_msgs::DetectedPerson transformedDetectedPerson = detectedPerson;
            transformedDetectedPerson.pose = compositeDetectedPerson.pose; // legal since here there is a 1:1 correspondence between composite and original detection
            compositeDetectedPerson.original_detections.push_back(transformedDetectedPerson);

            outputMsg->elements.push_back(compositeDetectedPerson);
        }

        boost::mutex::scoped_lock lock(m_monitorMutex);
        if(!m_publisher) createPublisher();
        m_publisher->publish(outputMsg);
    }
}


PLUGINLIB_EXPORT_CLASS(spencer_detected_person_association::ConvertToCompositeDetectionsNodelet, nodelet::Nodelet)
