/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/foreach.hpp>
#include <srl_nearest_neighbor_tracker/occlusion_handling/laser_shade_occlusion_manager.h>

#define foreach BOOST_FOREACH


#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>


namespace srl_nnt {

LaserShadeOcclusionManager::LaserShadeOcclusionManager() :
  m_numberLaserScanners(0), m_visualizationEnabled(false), m_detectionProbabilityOccluded(0.2), m_detectionProbabilityVisible(0.9)
{
}

void LaserShadeOcclusionManager::initializeOcclusionManager(const ros::NodeHandle& nodeHandle,const ros::NodeHandle& privateNodeHandle)
{
    // Save node handles in members
    m_nodeHandle = nodeHandle;
    m_privateNodeHandle = privateNodeHandle;

    // Initialize sampler
    m_sampler.reset(new MultiVariateNormalDistribution<double>(true, 1));
    m_visualizationEnabled = Params::get<bool>("occlusion_manager_visualization_enabled", false);
    m_numberSamples = Params::get<int>("occlusion_manager_number_samples", 1);
    m_addtionalTimeFactor = Params::get<double>("occlusion_manager_time_uncertainty_factor", 1.2);
    m_occlusionMaxRange = Params::get<double>("occlusion_manager_max_range", 60);

    // Get number of laser scanners and subscribe to the corresponding topics
    int laserSubscriptions = Params::get<int>("occlusion_manager_number_lasers", 1);
    for (size_t i = 0; i < laserSubscriptions; i++)
    {
        stringstream laserParamName;
        laserParamName << "occlusion_manager_laser_topic_" << i;
        string laserTopic = Params::get<string>(laserParamName.str(), "/laser");
        stringstream segmentationParamName;
        segmentationParamName << "occlusion_manager_segmentation_topic_" << i;
        string segmentationTopic = Params::get<string>(segmentationParamName.str(),"/laser_segmentation_unfiltered");

        subscribeToLaser(laserTopic, segmentationTopic);
    }

    m_visualizationPublisher = m_nodeHandle.advertise<visualization_msgs::Marker>("occlusion_markers", 10);

    m_MAX_MISSES_BEFORE_DELETION = Params::get<int>("max_misses_before_deletion", 8);
    m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK = Params::get<int>("max_misses_before_deletion_of_mature_track", 15);
    m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES = Params::get<int>("track_is_mature_after_total_num_matches", 100);

    ROS_INFO_STREAM("#### Laser Shade Occlusion Manager configured as follows: #####\n "
    << "occlusion_manager_visualization_enabled:" << m_visualizationEnabled << "\n"
    << "occlusion_manager_number_samples:" << m_numberSamples << "\n"
    << "occlusion_manager_time_uncertainty_factor:" << m_addtionalTimeFactor << "\n"
    << "occlusion_manager_max_range:" << m_occlusionMaxRange << "\n"
    << "occlusion_manager_number_lasers:" << laserSubscriptions << "\n"
    << "max_misses_before_deletion:" << m_MAX_MISSES_BEFORE_DELETION << "\n"
    << "max_misses_before_deletion_of_mature_track:" << m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK << "\n"
    << "track_is_mature_after_total_num_matches:" << m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES << "\n");
}

void LaserShadeOcclusionManager::deleteOccludedTracks(Tracks& tracks,const ros::Time& time)
{
    ROS_WARN("Deleting obsolete tracks");

    size_t numDeletedTracks = 0;
    std::vector<Tracks::iterator> tracksToDelete;
    for(Tracks::iterator trackIt = tracks.begin(); trackIt != tracks.end(); ++trackIt) {
        Track::Ptr track = *trackIt;
        switch (track->trackStatus){
            case Track::OCCLUDED:
                if(m_occludedTracks.find(track->id) != m_occludedTracks.end())
                {
                    ros::Duration timeDiff = time - m_occludedTracks.at(track->id)->occlusionTime;
                    if(timeDiff.toSec() > 0) {
                        ROS_INFO_STREAM("Deleting track with id " << track->id << " because occlusion time is " << timeDiff.toSec() << " sec in the past." );
                        tracksToDelete.push_back(trackIt);
                        numDeletedTracks++;
                        m_occludedTracks.erase(track->id);
                    }
                    else{
                        ROS_INFO_STREAM("Keeping track with id " << track->id << " because occlusion time is " << timeDiff.toSec() << " sec in the future." );
                    }
                }
                break;
            case Track::MISSED:
            {
                if (m_redetectableTracks.find(track->id) != m_redetectableTracks.end())
                {
                    StateVector zeroedState = StateVector::Zero();
                    zeroedState.head(2) = track->state->x().head(2);
                    track->state->setX(zeroedState);
                    ROS_ERROR_STREAM("Track " << track->id << " should have been visible again but could not be matched therefore we wait and set velocity components to 0");
                    break;
                }

                // Check if the track is considered as "mature", i.e. it has been there for a long time already.
                const bool trackIsMature = track->numberOfTotalMatches >= m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES;

                // Check if the track hasn't been seen for too long.
                const int missedFrameLimit = trackIsMature ? m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK : m_MAX_MISSES_BEFORE_DELETION;
                if(track->numberOfConsecutiveMisses > missedFrameLimit) {
                    tracksToDelete.push_back(trackIt);
                    ROS_INFO_STREAM("Deleting track with id " << track->id << " with  " << track->numberOfConsecutiveMisses << " consecutive misses." );
                    numDeletedTracks++;
                }
                break;
            }
            default:
                if (m_redetectableTracks.find(track->id) != m_redetectableTracks.end())
                {
                    m_redetectableTracks.erase(track->id);
                    ROS_INFO_STREAM("Redetectable track with id " << track->id << " was matched." );
                }
                break;
        }
    }
    for(int i = tracksToDelete.size()-1; i >= 0; i--) {
        tracks.erase(tracksToDelete.at(i));
    }

    if(numDeletedTracks) ROS_WARN("%zu track(s) have been deleted!", numDeletedTracks);

}

void LaserShadeOcclusionManager::subscribeToLaser(const std::string laserTopic, const std::string laserSegmentationTopic)
{
    int queue_size = Params::get<int>("occlusion_manager_subscriber_queue_size",35);
    int circular_buffer_size = Params::get<int>("occlusion_manager_sync_buffer_size",10);

    // Subscribers
    m_laserscanSubscriber.reset( new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nodeHandle, laserTopic, queue_size) );
    m_segmentationSubscriber.reset (new message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation>(m_nodeHandle, laserSegmentationTopic, queue_size) );

    m_inputSynchronizer.reset( new Synchronizer(SyncPolicy(queue_size), *m_laserscanSubscriber, *m_segmentationSubscriber) );
    m_inputSynchronizer->registerCallback(boost::bind(&LaserShadeOcclusionManager::newLaserscanAndSegmentationAvailable, this, _1,_2,m_numberLaserScanners));

    //Allocate buffer space for new laser
    BufferType dataBuffer(circular_buffer_size);
    m_bufferVector.push_back(dataBuffer);

    LaserInfo newLaserInfo = {
            /*.minAngle =*/0.0,
            /*.maxAngle=*/0.0,
            /*.angleIncrement=*/0.0,
            /*.numberMeasurements=*/0
    };
    m_laserInfos.push_back(newLaserInfo);

    // Increment number of lasers used for laserID
    m_numberLaserScanners++;



}

void LaserShadeOcclusionManager::newLaserscanAndSegmentationAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, unsigned int laserID)
{
    ROS_DEBUG_STREAM("Received new laser scan and segmentation with id " << laserID);
    // If it is first scan than initialize laser info struct
    if (m_laserInfos.at(laserID).numberMeasurements == 0)
    {
        m_laserInfos.at(laserID).minAngle = laserscan->angle_min;
        m_laserInfos.at(laserID).maxAngle = laserscan->angle_max;
        m_laserInfos.at(laserID).angleIncrement = laserscan->angle_increment;
        m_laserInfos.at(laserID).numberMeasurements = (unsigned int) ((laserscan->angle_max -laserscan->angle_min)/laserscan->angle_increment)+1;
    }
    // Get buffer for the laser depending on passed laserID
    BufferType& buffer = m_bufferVector.at(laserID);

    // save data in buffer
    LaserScanAndSegmentation newData (laserscan, segmentation);
    buffer.push_back(newData);
}

Tracks LaserShadeOcclusionManager::manageOcclusionsBeforeDataAssociation(Tracks& tracks,const ros::Time& time, const std::string& trackFrameID)
{
    for (unsigned int laserID = 0; laserID < m_bufferVector.size(); laserID++){
        BufferType& buffer = m_bufferVector.at(laserID);
        int idx = findCorrespondingDataInBuffer(time, buffer);
        if (idx < 0)
        {
            ROS_WARN_STREAM("Occlusion Manager could not find suitable data in buffer for timestamp " << time);
            continue;
        }
        else
            ROS_DEBUG_STREAM("Found corresponding laser and segmentation with idx " << idx);
        LaserScanAndSegmentation& laserData = buffer.at(idx);
        SegmentDataPolar segmentData;
        extractSegmentData(laserData, laserID, segmentData);
        Eigen::Affine3d transformationToLaserFrame;
        if(lookupTransformIntoSensorFrame(time,trackFrameID,laserData.first->header.frame_id, transformationToLaserFrame))
        {
            ROS_DEBUG_STREAM("Looping over " << tracks.size() <<" tracks ");
            foreach(Track::Ptr track, tracks){
                Eigen::Vector3d transformedMean;
                Eigen::Matrix3d transformedCov;
                transformTrackToSensorFrame(track,transformationToLaserFrame,transformedMean, transformedCov);
                Eigen::Matrix3Xd samples;
                transformedCov = transformedCov * track->detectionProbability;
                sampleFromDistribution(transformedMean, transformedCov, m_numberSamples, samples);
                OcclusionSegmentMap occlusionSegments;
                double occProb = getOcclusionProbabilityForSamples(samples, laserData, segmentData, laserID, occlusionSegments);
                track->detectionProbability = m_detectionProbabilityOccluded + (m_detectionProbabilityVisible - m_detectionProbabilityOccluded) * (1-occProb);
                ROS_INFO_STREAM("Probability of detection is " << track->detectionProbability << " for track id " << track->id);
                if (occProb > 0.5)
                {
                    if (m_redetectableTracks.find(track->id) != m_redetectableTracks.end())
                    {
                        StateVector stateReset = track->state->x();
                        stateReset.tail(STATE_DIM - OBS_DIM) = m_redetectableTracks.at(track->id)->endOcclusionState.tail(STATE_DIM - OBS_DIM);
                        track->state->setX(stateReset);
                        stateReset = track->state->xp();
                        stateReset.tail(STATE_DIM - OBS_DIM) = m_redetectableTracks.at(track->id)->endOcclusionState.tail(STATE_DIM - OBS_DIM);
                        track->state->setXp(stateReset);
                        m_redetectableTracks.erase(track->id);
                        ROS_INFO_STREAM("Redetectable track with id " << track->id << " is occluded we are resetting the velocity components." );
                    }

                    track->trackStatus = Track::OCCLUDED;
                    Eigen::Vector3d transformedVelocity = Eigen::Vector3d::Zero();
                    transformVelocityToSensorFrame(track,transformationToLaserFrame,transformedVelocity);
                    SegmentInfo::Ptr occlusionSegment;
                    identifyMostImportantOcclusionSegment(transformedVelocity, occlusionSegments, occlusionSegment);
                    Eigen::Vector3d intersectionWithShade = Eigen::Vector3d::Zero();
                    double occludedSeconds = calculateIntersectionWithOcclusionShade(occlusionSegment, laserData, transformedMean,transformedVelocity,intersectionWithShade);
                    if (occludedSeconds > 0)
                        visualizeOcclusionDistance(transformedMean, intersectionWithShade, laserData, track->id);
                    else
                        deleteOcclusionDistanceMarker(laserData, track->id);
                    if (occludedSeconds != 0)
                    {
                        if (m_occludedTracks.find(track->id) == m_occludedTracks.end())
                        {
                            OccludedTrack::Ptr newOccludedTrack(new OccludedTrack);
                            newOccludedTrack->track = track;
                            newOccludedTrack->beginOcclusionState = track->state->x();
                            newOccludedTrack->beginOcclusionCov = track->state->C();
                            m_occludedTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id, newOccludedTrack));
                        }
                        ROS_WARN_STREAM("Track with ID" << track->id << " is likely to be occluded "<< occProb << " for exact " << occludedSeconds << " sec scaled with uncertainty to " << occludedSeconds * m_addtionalTimeFactor );
                        if (occludedSeconds > 1e3)
                            occludedSeconds = 10;
                        m_occludedTracks.at(track->id)->occlusionTime = time + ros::Duration(occludedSeconds * m_addtionalTimeFactor);

                    }

                }
                else if(m_occludedTracks.find(track->id) != m_occludedTracks.end()){
                    // m_redetectableTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id,m_occludedTracks.at(track->id)));
                    m_occludedTracks.at(track->id)->endOcclusionState = track->state->x();
                    m_occludedTracks.at(track->id)->endOcclusionCov = track->state->C();
                    m_occludedTracks.erase(track->id);
                    deleteOcclusionDistanceMarker(laserData, track->id);
                    track->trackStatus = Track::MISSED;
                }
            }
            visualizeSamples(laserData);
        }
        else {
            ROS_WARN("Could not find transformation");
        }
    }
    return Tracks();
}

int LaserShadeOcclusionManager::getIndexFromAngle(double angle, unsigned int laserID)
{
    //ROS_INFO_STREAM("Laser min Angle =" << m_laserInfos.at(laserID).minAngle << " angle increment =" << m_laserInfos.at(laserID).angleIncrement);
    return (int) round((angle- m_laserInfos.at(laserID).minAngle)/m_laserInfos.at(laserID).angleIncrement);
}

int LaserShadeOcclusionManager::findCorrespondingDataInBuffer(const ros::Time& tracksTime, BufferType& buffer)
{
    int bestIdx = -1;
    double smallestDiff = 10.0;
    for (int i = 0; i < buffer.size(); i++)
    {
        ros::Duration duration = tracksTime - buffer.at(i).first->header.stamp;
        if (fabs(duration.toSec()) < smallestDiff)
        {
            bestIdx = i;
            smallestDiff = fabs(duration.toSec());
        }
    }
    return bestIdx;

}

void LaserShadeOcclusionManager::extractSegmentData(LaserScanAndSegmentation& data, unsigned int laserID, SegmentDataPolar& segmentData)
{
    ROS_DEBUG("Extracting segments from scan");
    const sensor_msgs::LaserScan::ConstPtr& laserscan = data.first;
    const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation = data.second;

    segmentData.clear();
    ROS_DEBUG_STREAM("Reserving segment infos for " << m_laserInfos.at(laserID).numberMeasurements << " measurements.");
    segmentData.resize(m_laserInfos.at(laserID).numberMeasurements);

    for (size_t i = 0; i < segmentation->segments.size(); i++){
        srl_laser_segmentation::LaserscanSegment segment = segmentation->segments.at(i);
        // Assume measurement indices are ordered
        SegmentInfo::Ptr info(new SegmentInfo);
        info->minAngle = laserscan->angle_min + laserscan->angle_increment * segment.measurement_indices.front() ;
        info->maxAngle = laserscan->angle_min + laserscan->angle_increment * segment.measurement_indices.back() ;
        info->minDist = laserscan->ranges.at(segment.measurement_indices.front());
        info->maxDist = laserscan->ranges.at(segment.measurement_indices.back());
        info->label = segment.label;
        info->idx = i;

        for (unsigned int point=0; point < segment.measurement_indices.size(); point++){
            segmentData.at(segment.measurement_indices.at(point)) = info;
        }
    }
}

bool LaserShadeOcclusionManager::lookupTransformIntoSensorFrame(ros::Time stamp, const std::string& sourceFrame ,const std::string& targetFrame, Eigen::Affine3d& resultingTransform)
{
    ROS_DEBUG_STREAM("Entering transform lookup from " << sourceFrame << " to " << targetFrame << " for time " << stamp.toNSec());
    tf::StampedTransform tfTransform;

    try {
        m_transformListener.waitForTransform(targetFrame, sourceFrame, stamp, ros::Duration(srl_nnt::Params::get<double>("transform_timeout", 0.2) ));
        m_transformListener.lookupTransform(targetFrame, sourceFrame, stamp, tfTransform);
    }
    catch(const tf::TransformException& ex) {
        ROS_ERROR_STREAM_THROTTLE(2.0, "Could not determine transform from observation frame \"" << sourceFrame << "\" into fixed world frame \"" << targetFrame << "\", "
                << "which may lead to observations being dropped. This message will re-appear every 2 seconds. Reason: " << ex.what());
        return false;
    }

    tf::transformTFToEigen(tfTransform, resultingTransform);
    ROS_DEBUG_STREAM("Resulting transform " << resultingTransform.matrix());

    return true;
}

void LaserShadeOcclusionManager::transformTrackToSensorFrame(Track::Ptr track, Eigen::Affine3d& transform, Eigen::Vector3d& meanTrackLF, Eigen::Matrix3d& covTrackLF)
{
    Eigen::Vector3d trackPosition = Eigen::Vector3d::Zero();
    trackPosition(0) = track->state->xp()(0);
    trackPosition(1) = track->state->xp()(1);

    // ROS Covariance is a 6x6 matrix (xyz + xyz rotation) relative to the message's coordinate frame
    // We are not interested in pose rotation, so only take first 3 rows and columns
    Eigen::Matrix3d trackCov = Eigen::Matrix3d::Zero();
    trackCov(0, 0) = track->state->Cp()(0,0);
    trackCov(1, 0) = track->state->Cp()(1,0);
    trackCov(0, 1) = track->state->Cp()(0,1);
    trackCov(1, 1) = track->state->Cp()(1,1);
    //set to one for cholesky solver to work
    trackCov(2, 2) = 1;

    // Transform pose and covariance into our reference frame, still in 3D
    meanTrackLF = transform * trackPosition;

    // For covariance, only the coordinate frame rotation is relevant (invariant w.r.t. translation)
    Eigen::Matrix3d detectionFrameToOurFrameRotation = transform.linear().matrix();
    covTrackLF = detectionFrameToOurFrameRotation * trackCov * detectionFrameToOurFrameRotation.transpose();
    /*ROS_INFO_STREAM("Mean transformed from \n" << trackPosition << " to sensor frame \n" << meanTrackLF);
    ROS_INFO_STREAM("Covariance transformed from \n" << trackCov << " to sensor frame \n" << covTrackLF);*/

}

void LaserShadeOcclusionManager::transformVelocityToSensorFrame(Track::Ptr track, Eigen::Affine3d& transform, Eigen::Vector3d& velocityLF)
{
    velocityLF = Eigen::Vector3d::Zero();
    velocityLF(0) = track->state->xp()(2);
    velocityLF(1) = track->state->xp()(3);
}

double LaserShadeOcclusionManager::calculateIntersectionWithOcclusionShade(SegmentInfo::Ptr segmentInfo,  LaserScanAndSegmentation& laserData, Eigen::Vector3d& trackPosition, Eigen::Vector3d& trackVelocity, Eigen::Vector3d& intersection)
{
    const double laserFrequency = 25.0;
    ROS_DEBUG_STREAM("Intersection calculation segment pointer set to" << segmentInfo);
    if (segmentInfo != 0)
    {
        const double extendDistance = 100.0;
        // Store the values for fast access and easy
        // equations-to-code conversion
        double absVelocity = hypot(trackVelocity(0), trackVelocity(1));
        std::vector<Point2D> trackLine;
        Point2D point;
        point.x = trackPosition(0);
        point.y = trackPosition(1);
        trackLine.push_back(point);
        point.x = trackPosition(0) + (trackVelocity(0)/absVelocity * extendDistance);
        point.y = trackPosition(1) + (trackVelocity(1)/absVelocity * extendDistance);
        trackLine.push_back(point);


        bool trackPassedSegment = false;
        const double thetaTrack = atan2(trackPosition(1), trackPosition(0));
        const double orientationTrack = atan2(trackVelocity(1), trackVelocity(0));
        std::vector<Point2D> verticesShade;
        // depending on orientation we calculate which edge is more likely to be intersected to save intersection checks
        if (orientationTrack > 0 )
        {
            ROS_DEBUG_STREAM("Y-Velocity of track is " << trackVelocity(1) << "taking max angle of " << segmentInfo->maxAngle);
            //Calculate vertices of polygon defined through laser shade
            point.x = segmentInfo->maxDist * cos(segmentInfo->maxAngle);
            point.y = segmentInfo->maxDist * sin(segmentInfo->maxAngle);
            // push vertice into vertices vector
            verticesShade.push_back(point);
            point.x = m_occlusionMaxRange  * cos(segmentInfo->maxAngle);
            point.y = m_occlusionMaxRange * sin(segmentInfo->maxAngle);
            verticesShade.push_back(point);

            point.x = m_occlusionMaxRange * cos(segmentInfo->minAngle);
            point.y = m_occlusionMaxRange * sin(segmentInfo->minAngle);
            verticesShade.push_back(point);

            point.x = segmentInfo->minDist * cos(segmentInfo->minAngle);
            point.y = segmentInfo->minDist * sin(segmentInfo->minAngle);
            verticesShade.push_back(point);
            verticesShade.push_back(verticesShade.front());

            //check if track already passed segment
            if (thetaTrack >= segmentInfo->maxAngle)
                trackPassedSegment = true;
        }
        else
        {
            ROS_DEBUG_STREAM("Y-Velocity of track is " << trackVelocity(1) << "taking min angle of " << segmentInfo->minAngle);
            point.x = segmentInfo->minDist * cos(segmentInfo->minAngle);
            point.y = segmentInfo->minDist * sin(segmentInfo->minAngle);
            verticesShade.push_back(point);
            point.x = m_occlusionMaxRange * cos(segmentInfo->minAngle);
            point.y = m_occlusionMaxRange * sin(segmentInfo->minAngle);
            verticesShade.push_back(point);

            point.x = m_occlusionMaxRange * cos(segmentInfo->maxAngle);
            point.y = m_occlusionMaxRange * sin(segmentInfo->maxAngle);
            verticesShade.push_back(point);

            point.x = segmentInfo->maxDist * cos(segmentInfo->maxAngle);
            point.y = segmentInfo->maxDist * sin(segmentInfo->maxAngle);
            verticesShade.push_back(point);
            verticesShade.push_back(verticesShade.front());

            if (thetaTrack <= segmentInfo->minAngle)
                trackPassedSegment = true;
        }

        if (!trackPassedSegment)
        {
            // calculate intersection with shade polygon
            bool intersectionFound = calculateIntersectionFromLineAndPolygon(trackLine, verticesShade, intersection);
            if (intersectionFound)
            {
                double distanceInShade = (trackPosition-intersection).norm();
                double occludedSeconds = distanceInShade/absVelocity;
                ROS_DEBUG_STREAM("Track occluded for " << occludedSeconds << " seconds for a distance in shade " << distanceInShade << " moving with an velocity of " << absVelocity);
                return occludedSeconds;
            }
            else
            {
                double rhoTrack = hypot(trackPosition(1), trackPosition(2));
                ROS_WARN_STREAM("No intersection found with polygon for track at X" << trackPosition(0) << "m;Y" << trackPosition(1) << "m with rho " << rhoTrack  << "m and theta " << thetaTrack << "rad with orientation " << orientationTrack << "rad" );
                if (rhoTrack > m_occlusionMaxRange){
                    ROS_INFO_STREAM("Track is out of laser max range we want to delete the track therefore retruning -1 sec");
                    return -1;
                }
                else if (rhoTrack < min(segmentInfo->maxDist, segmentInfo->minDist)){
                    ROS_INFO_STREAM("Track position is due to segment shape in front of segment therefore we keep the track by returning 0");
                    return 0;
                }
                else {
                    ROS_INFO_STREAM("Track is moving parallel to the occlusion segment therfore no intersection found we trust on occlusions before and keep the track in the system by returning 0");
                    return 0;
                }
            }
        }
        else{
            ROS_WARN_STREAM("Track already passed segment therefore not updating time should become visible in next step.");
            return 0;
        }
    }
    ROS_WARN_STREAM("No segment info was set for track. Returning -2.");
    return -2;

}

bool LaserShadeOcclusionManager::calculateIntersectionFromLineAndPolygon(const std::vector<Point2D> &line, const std::vector<Point2D> &polygon, Eigen::Vector3d& result)
{
    assert(line.size() == 2);
    for (size_t i = 0 ; i+1 < polygon.size(); i++)
    {

       bool found = calculateIntersectionFromTwoLineSegments(line.front().x,line.front().y, line.back().x,line.back().y, polygon.at(i).x,polygon.at(i).y, polygon.at(i+1).x,polygon.at(i+1).y, result);
       if (found)
        return true;
    }
    return false;
}

bool LaserShadeOcclusionManager::calculateIntersectionFromTwoLineSegments(const double xt1, const double yt1, const double xt2, const double yt2, const double xs1, const double ys1, const double xs2, const double ys2, Eigen::Vector3d& result)
{
    ROS_DEBUG_STREAM("calculate intersection for track [" <<xt1 <<";" << xt2
            << "],["<<yt1 <<";" << yt2 << "] Shade ["<<xs1 <<";" << xs2
            << "],["<<ys1 <<";" << ys2 << "]");
    double d = (xt1 - xt2) * (ys1 - ys2) - (yt1 - yt2) * (xs1 - xs2);
    // If d is zero, there is no intersection
    if (d == 0) return false;

    // Get the x and y
    double pre = (xt1*yt2 - yt1*xt2), post = (xs1*ys2 - ys1*xs2);
    double x = ( pre * (xs1 - xs2) - (xt1 - xt2) * post ) / d;
    double y = ( pre * (ys1 - ys2) - (yt1 - yt2) * post ) / d;



    // Check if the x and y coordinates are within both lines
    if ( x < std::min(xt1, xt2) || x > std::max(xt1, xt2) ||
            x < std::min(xs1, xs2) || x > std::max(xs1, xs2) ){
        ROS_WARN_STREAM("calculate intersection check1 for track failed [" <<xt1 <<";" << xt2
                << "],["<<yt1 <<";" << yt2 << "] Shade ["<<xs1 <<";" << xs2
                << "],["<<ys1 <<";" << ys2 << "]");
        return false;
    }
    if ( y < std::min(yt1, yt2) || y > std::max(yt1, yt2) ||
            y < std::min(ys1, ys2) || y > std::max(ys1, ys2) ) {
        ROS_WARN_STREAM("calculate intersection check2 for track failed [" <<xt1 <<";" << xt2
                << "],["<<yt1 <<";" << yt2 << "] Shade ["<<xs1 <<";" << xs2
                << "],["<<ys1 <<";" << ys2 << "]");
        return false;
    }

    // Return the point of intersection
    result(0) = x;
    result(1) = y;
    return true;
}

void LaserShadeOcclusionManager::sampleFromDistribution(Eigen::Vector3d& mean, Eigen::Matrix3d& cov, unsigned int n_samples, Eigen::Matrix3Xd& samples)
{
    samples = Eigen::Matrix3Xd::Zero(3, n_samples);
    if (n_samples > 1){
        m_sampler->setMu(mean);
        m_sampler->setC(cov);
        samples = m_sampler->generateSamples(n_samples);
        ROS_DEBUG_STREAM("Sampled " << n_samples
                         << " times from Gaussian with mean \n" << mean << "\n and covariance \n"
                         << cov << "\n Resulting samples: \n" << samples  );
    }
    else {
        samples(0,0) = mean(0);
        samples(1,0) = mean(1);
        ROS_DEBUG_STREAM("Just using one sample therefore using mean of prediction with " << samples );
    }
}

double LaserShadeOcclusionManager::getOcclusionProbabilityForSamples(Eigen::Matrix3Xd& samples, LaserScanAndSegmentation& laserData, SegmentDataPolar& segmentData,unsigned int laserID, OcclusionSegmentMap& responsibleSegments)
{
    ROS_DEBUG_STREAM("Samples " << samples);

    //Visualization of samples
    geometry_msgs::Point visSample;
    unsigned int counter = samples.cols();
    for (size_t sampleIdx = 0; sampleIdx <  samples.cols(); sampleIdx++)
    {
        Eigen::Vector3d sample = samples.col(sampleIdx);
        double rho = hypot(sample(0), sample(1));
        double phi = atan2(sample(1), sample(0));
        int compareIdx = getIndexFromAngle(phi, laserID);
        int label = -1;
        ROS_DEBUG_STREAM("Compare index is " << compareIdx << " for angle=" << phi << " and dist=" << rho);

        visSample.x = sample(0);
        visSample.y = sample(1);
        if (compareIdx > 0 && compareIdx < m_laserInfos.at(laserID).numberMeasurements)
        {
            if (laserData.first->ranges.at(compareIdx) + 0.4 < rho)
            {
                // Check if we have a segment info saved at found index
                if (segmentData.at(compareIdx))
                {
                    if (segmentData.at(compareIdx)->label != label && label != -1 )
                        ROS_DEBUG_STREAM("Different label of occlusions");
                    if (responsibleSegments.find(segmentData.at(compareIdx)) == responsibleSegments.end()){
                        responsibleSegments[segmentData.at(compareIdx)] = 0;
                    }
                    else{
                        responsibleSegments[segmentData.at(compareIdx)] += 1;
                    }
                    label = segmentData.at(compareIdx)->label;
                    ROS_DEBUG_STREAM("Occlusion encountered because of distance sample=" << rho << "; laser distance =" << laserData.first->ranges.at(compareIdx)
                            << " with label " << segmentData.at(compareIdx)->label);
                    counter--;
                    m_occluded.push_back(visSample);
                }
                else
                {
                    ROS_WARN_STREAM("No segment data available for index " << compareIdx << " laser measurement is " <<
                    laserData.first->ranges.at(compareIdx));
                }
            }
            else
            {
                m_visible.push_back(visSample);
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Occlusion encountered because view area");
            counter--;
            m_outOfView.push_back(visSample);
        }
    }
    return  1.0-(counter/(double)samples.cols()) ;
}

void LaserShadeOcclusionManager::identifyMostImportantOcclusionSegment(Eigen::Vector3d& trackVelocity, OcclusionSegmentMap& occlusionSegments, SegmentInfo::Ptr& importantSegment)
{
    double foundMin =  M_2_PI;
    double foundMax = - M_2_PI;

    if (trackVelocity(1) > 0 )
    {
        for(OcclusionSegmentMap::iterator it= occlusionSegments.begin(); it != occlusionSegments.end() ; ++it )
        {
            if (it->first->maxAngle > foundMax ){
                foundMax = it->first->maxAngle;
                importantSegment = it->first;
            }
        }

    }
    else
    {
        for(OcclusionSegmentMap::iterator it= occlusionSegments.begin(); it != occlusionSegments.end() ; ++it )
        {
            if (it->first->minAngle < foundMin){
                foundMin = it->first->minAngle;
                importantSegment = it->first;
            }
        }
    }

}



///*****************************************************************
/// VISUALIZATION STUFF
///*****************************************************************

void LaserShadeOcclusionManager::visualizeOcclusionDistance(Eigen::Vector3d& trackPosition, Eigen::Vector3d& intersection, LaserScanAndSegmentation& laserData, const unsigned int trackID)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = laserData.first->header.frame_id;
    marker.header.stamp = laserData.first->header.stamp;

    marker.ns = "occlusion_distances";
    marker.id = trackID;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.05;
    marker.scale.y = 0.2;
    marker.scale.z = 0.3;

    // Line strip is blue
    marker.color.g = 0.98;
    marker.color.b = 0.604;
    marker.color.a = 0.8;

    marker.lifetime = ros::Duration();

    geometry_msgs::Point point;
    point.x = trackPosition(0);
    point.y = trackPosition(1);

    marker.points.push_back(point);
    point.x = intersection(0);
    point.y = intersection(1);

    marker.points.push_back(point);

    m_visualizationPublisher.publish(marker);
}

void LaserShadeOcclusionManager::deleteOcclusionDistanceMarker(LaserScanAndSegmentation& laserData, const unsigned int trackID)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = laserData.first->header.frame_id;
    marker.header.stamp = laserData.first->header.stamp;

    marker.ns = "occlusion_distances";
    marker.id = trackID;
    marker.action = visualization_msgs::Marker::DELETE;
    marker.type = visualization_msgs::Marker::ARROW;

    m_visualizationPublisher.publish(marker);
}

void LaserShadeOcclusionManager::visualizeSamples(LaserScanAndSegmentation& laserData)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = laserData.first->header.frame_id;
    marker.header.stamp = laserData.first->header.stamp;
    marker.id = 0;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.03;


    marker.ns = "samples_visible";
    marker.color.g = 1.0;
    marker.color.a = 0.8;
    marker.points = m_visible;
    m_visualizationPublisher.publish(marker);

    marker.ns = "samples_occluded";
    marker.color.g = 0.0;
    marker.color.r = 1.0;
    marker.color.a = 0.8;
    marker.points = m_occluded;
    m_visualizationPublisher.publish(marker);

    marker.ns = "samples_outOfView";
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.2;
    marker.color.a = 0.8;
    marker.points = m_outOfView;
    m_visualizationPublisher.publish(marker);

    m_outOfView.clear();
    m_visible.clear();
    m_occluded.clear();
}


}
