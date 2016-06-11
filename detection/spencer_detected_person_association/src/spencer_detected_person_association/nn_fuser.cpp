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

#include "nn_fuser.h"

#include <Eigen/Core>
#include <limits>
#include <set>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace spencer_detected_person_association
{
    void NearestNeighborFuserNodelet::onInit()
    {
        NODELET_INFO("Initializing NearestNeighborFuserNodelet...");
        initSynchronizer(getName(), getNodeHandle(), getPrivateNodeHandle(), 1, 2); // we require at most two input topics
        m_seq = 0;

        int detectionIdOffset;
        getPrivateNodeHandle().param<int>("detection_id_increment", m_detectionIdIncrement, 1);
        getPrivateNodeHandle().param<int>("detection_id_offset", detectionIdOffset, 0);
        m_lastDetectionId = detectionIdOffset;
    }

    void NearestNeighborFuserNodelet::onNewInputMessagesReceived(const std::vector<spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr>& inputMsgs)
    {
        spencer_tracking_msgs::CompositeDetectedPersons::Ptr outputMsg(new spencer_tracking_msgs::CompositeDetectedPersons);
        outputMsg->header.frame_id = inputMsgs[0]->header.frame_id;
        outputMsg->header.stamp = inputMsgs[0]->header.stamp;
        outputMsg->header.seq = m_seq++;

        // Get first set of composite detections to fuse
        const spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr& firstSet = inputMsgs[0];

        // Check if both sets of CompositeDetectedPersons contain elements
        if(inputMsgs.size() == 1 || inputMsgs[1]->elements.empty()) {
            // We are only subscribed to 1 topic or the second set is empty, so just fast-forward all elements
            outputMsg->elements.insert(outputMsg->elements.end(), firstSet->elements.begin(), firstSet->elements.end());
        }
        else if(firstSet->elements.empty()) {
            // Only the second set contains elements
            const spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr& secondSet = inputMsgs[1];
            outputMsg->elements.insert(outputMsg->elements.end(), secondSet->elements.begin(), secondSet->elements.end());
        }
        else {
            // Both sets contain elements, need to fuse the detections: This is where it get's interesting...

            // Use timestamp of latest message
            if(inputMsgs[1]->header.stamp > outputMsg->header.stamp) outputMsg->header.stamp = inputMsgs[1]->header.stamp;

            // Get second set of composite detections to fuse
            const spencer_tracking_msgs::CompositeDetectedPersons::ConstPtr& secondSet = inputMsgs[1];
            ROS_ASSERT(!firstSet->elements.empty() && !secondSet->elements.empty());

            // Ensure each topic uses the same coordinate frame ID
            ROS_ASSERT_MSG(secondSet->header.frame_id == firstSet->header.frame_id, "All input messages must already be in the same coordinate frame! Got %s and %s!",
                firstSet->header.frame_id.c_str(), secondSet->header.frame_id.c_str());

            // Initialize distance matrix with infinite values
            const float INF = std::numeric_limits<float>::infinity();
            Eigen::MatrixXf distanceMatrix = Eigen::MatrixXf::Constant(firstSet->elements.size(), secondSet->elements.size(), INF);

            // Remember indices of composite detections that are not associated
            std::set<size_t> unmatchedFirstSet, unmatchedSecondSet;

            // Compute pairwise distances
            for(size_t d1Index = 0; d1Index < firstSet->elements.size(); d1Index++) {
                for(size_t d2Index = 0; d2Index < secondSet->elements.size(); d2Index++) {
                    const spencer_tracking_msgs::CompositeDetectedPerson& d1 = firstSet->elements[d1Index];
                    const spencer_tracking_msgs::CompositeDetectedPerson& d2 = secondSet->elements[d2Index];

                    float distance = computeDistance(d1, d2);
                    distanceMatrix(d1Index, d2Index) = std::isfinite(distance) ? distance : INF;

                    unmatchedFirstSet.insert(d1Index);
                    unmatchedSecondSet.insert(d2Index);
                }
            }

            // Iteratively find minimum element in matrix, fuse the corresponding detections, and set entire row and column to infinity to mark it as 'used'
            Eigen::MatrixXf::Index rowIndex, colIndex;
            double minDistance;
            for(;;) {
                // Find minimum distance in matrix
                minDistance = distanceMatrix.minCoeff(&rowIndex, &colIndex);
                if(!std::isfinite(minDistance)) break;

                // Find corresponding CompositeDetectedPerson pair to fuse
                ROS_ASSERT(rowIndex >= 0 && rowIndex < firstSet->elements.size());
                ROS_ASSERT(colIndex >= 0 && colIndex < secondSet->elements.size());
                const spencer_tracking_msgs::CompositeDetectedPerson& d1 = firstSet->elements[rowIndex];
                const spencer_tracking_msgs::CompositeDetectedPerson& d2 = secondSet->elements[colIndex];

                // Fuse pose
                geometry_msgs::PoseWithCovariance fusedPose;
                fusePoses(d1, d2, fusedPose);

                // Build fused CompositeDetectedPerson msg, generate new unique ID
                spencer_tracking_msgs::CompositeDetectedPerson fusedDetection;
                fusedDetection.composite_detection_id = m_lastDetectionId;
                m_lastDetectionId += m_detectionIdIncrement;

                fusedDetection.min_confidence = std::min(d1.min_confidence, d2.min_confidence);
                fusedDetection.max_confidence = std::max(d2.max_confidence, d2.max_confidence);
                fusedDetection.mean_confidence = (d1.mean_confidence + d2.mean_confidence) / 2.0;

                fusedDetection.pose = fusedPose;

                fusedDetection.original_detections.insert(fusedDetection.original_detections.end(), d1.original_detections.begin(), d1.original_detections.end()); // d1
                fusedDetection.original_detections.insert(fusedDetection.original_detections.end(), d2.original_detections.begin(), d2.original_detections.end()); // d2

                // Add fused CompositeDetectedPerson to result set
                outputMsg->elements.push_back(fusedDetection);

                // Mark entire row and column as 'used'
                distanceMatrix.row(rowIndex).setConstant(INF);
                distanceMatrix.col(colIndex).setConstant(INF);

                // Mark indices of these DetectedPerson entries as matched
                unmatchedFirstSet.erase(rowIndex);
                unmatchedSecondSet.erase(colIndex);
            }

            // Fast-forward all remaining non-associated CompositeDetectedPerson entries from the first and second set of CompositeDetectedPersons
            foreach(size_t d1Index, unmatchedFirstSet) {
                outputMsg->elements.push_back(firstSet->elements[d1Index]);
            }
            foreach(size_t d2Index, unmatchedSecondSet) {
                outputMsg->elements.push_back(secondSet->elements[d2Index]);
            }

        } // end if only one topic subscribed

        m_publisher->publish(outputMsg);
    }
}
