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

#ifndef _CONVERT_TO_COMPOSITE_DETECTIONS_H
#define _CONVERT_TO_COMPOSITE_DETECTIONS_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>

#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/CompositeDetectedPersons.h>

namespace spencer_detected_person_association
{
    /// Converts detections into composite detections (retaining the original ID, pose, confidence), optionally transforming
    /// the pose into a different coordinate frame.
    class ConvertToCompositeDetectionsNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
        virtual ~ConvertToCompositeDetectionsNodelet();

    private:
        // Creates the publisher once the input topic becomes active
        void createPublisher();

        // Message callback
        void onNewInputMessageReceived(const spencer_tracking_msgs::DetectedPersons::ConstPtr& inputMsg);

        // Executed in a separate thread, continuously monitors if the number of active topics has changed.
        void monitorInputTopic();

        // Common frame that all detections are transformed into (if specified)
        std::string m_commonFrameId;
        boost::shared_ptr<tf::TransformListener> m_transformListener;

        // Subscriber for input spencer_tracking_msgs::DetectedPersons
        boost::shared_ptr< ros::Subscriber > m_subscriber;

        // Publisher for output spencer_tracking_msgs::CompositeDetectedPersons
        boost::shared_ptr< ros::Publisher > m_publisher;

        // For monitoring if there are any publishers on the subscribed topic
        boost::mutex m_monitorMutex;
        boost::thread m_monitorThread;
        double m_topicMonitorInterval;
    };
}


#endif // _CONVERT_TO_COMPOSITE_DETECTIONS_H
