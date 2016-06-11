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
#include "aggregate_detections.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace spencer_detected_person_association
{
    void AggregateDetectionsNodelet::onInit()
    {
        NODELET_INFO("Initializing AggregateDetectionsNodelet...");
        initSynchronizer(getName(), getNodeHandle(), getPrivateNodeHandle());
        m_seq = 0;
    }

    void AggregateDetectionsNodelet::onNewInputMessagesReceived(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& inputMsgs)
    {
        spencer_tracking_msgs::CompositeDetectedPersons::Ptr outputMsg(new spencer_tracking_msgs::CompositeDetectedPersons);

        outputMsg->header.frame_id = inputMsgs[0]->header.frame_id;
        outputMsg->header.seq = m_seq++;

        foreach(const spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr& inputMsg, inputMsgs)
        {
            // Ensure each topic uses the same coordinate frame ID
            ROS_ASSERT_MSG(inputMsg->header.frame_id == outputMsg->header.frame_id, "All input messages must already be in the same coordinate frame! Got %s and %s!",
                inputMsg->header.frame_id.c_str(), outputMsg->header.frame_id.c_str());

            // Use timestamp of latest message
            if(inputMsg->header.stamp > outputMsg->header.stamp)
                outputMsg->header.stamp = inputMsg->header.stamp;

            // Aggregate CompositeDetectedPerson elements
            outputMsg->elements.insert(outputMsg->elements.end(), inputMsg->elements.begin(), inputMsg->elements.end());
        }

        m_publisher->publish(outputMsg);
    }
}


PLUGINLIB_DECLARE_CLASS(spencer_detected_person_association, AggregateDetectionsNodelet, spencer_detected_person_association::AggregateDetectionsNodelet, nodelet::Nodelet)
